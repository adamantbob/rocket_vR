#[cfg(feature = "verbose-utilization")]
use crate::info;
#[cfg(not(feature = "verbose-utilization"))]
use crate::warn;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use embassy_time::{Duration, Instant};
use portable_atomic::Ordering;
use {defmt_rtt as _, panic_probe as _};

use crate::instrumented_executor;

struct Percent(f32);

impl defmt::Format for Percent {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}.{}%", self.0 as u32, ((self.0 * 10.0) as u32) % 10)
    }
}

impl core::fmt::Display for Percent {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}.{}%", self.0 as u32, ((self.0 * 10.0) as u32) % 10)
    }
}

// Utilization Tracking
// ===================
//
// This module provides a simple way to track the utilization of tasks in the system.
// It supports a flexible number of tasks up to MAX_TASKS.
// Compile-time safety is ensured by validating the task count during setup.

#[derive(Clone, Copy)]
pub struct TaskId(pub usize);

pub const MAX_TASKS: usize = 16;

#[cfg(not(feature = "verbose-utilization"))]
pub const TASK_USAGE_THRESHOLD: f32 = 10.0;
#[cfg(not(feature = "verbose-utilization"))]
pub const TOTAL_USAGE_WARNING_THRESHOLD: f32 = 40.0;
#[cfg(not(feature = "verbose-utilization"))]
pub const TOTAL_USAGE_ALARM_THRESHOLD: f32 = 80.0;
#[cfg(not(feature = "verbose-utilization"))]
pub const HIGH_PRIORITY_TASK_USAGE_THRESHOLD: f32 = 20.0;

// Silent Profiling: We use raw u32s for task ticks to avoid atomic-induced event spins.
// Since these are only updated by the executor thread, it's safe from races.
static mut TASK_TICKS: [u32; MAX_TASKS] = [0; MAX_TASKS];
static mut TASK_POLLS: [u32; MAX_TASKS] = [0; MAX_TASKS];

fn get_task_ticks(id: usize) -> u32 {
    unsafe { core::ptr::read_volatile(&TASK_TICKS[id]) }
}

fn add_task_ticks(id: usize, delta: u32) {
    if id >= MAX_TASKS {
        return;
    }
    unsafe {
        let ptr = &mut TASK_TICKS[id] as *mut u32;
        core::ptr::write_volatile(ptr, core::ptr::read_volatile(ptr).wrapping_add(delta));
    }
}

fn get_task_polls(id: usize) -> u32 {
    unsafe { core::ptr::read_volatile(&TASK_POLLS[id]) }
}

fn add_task_polls(id: usize, delta: u32) {
    if id >= MAX_TASKS {
        return;
    }
    unsafe {
        let ptr = &mut TASK_POLLS[id] as *mut u32;
        core::ptr::write_volatile(ptr, core::ptr::read_volatile(ptr).wrapping_add(delta));
    }
}

pub struct TrackedFuture<F> {
    id: TaskId,
    inner: F,
}

impl<F: Future> Future for TrackedFuture<F> {
    type Output = F::Output;
    fn poll(mut self: Pin<&mut Self>, cx: &mut Context<'_>) -> Poll<Self::Output> {
        let start = Instant::now().as_ticks() as u32;
        let res = unsafe { self.as_mut().map_unchecked_mut(|s| &mut s.inner) }.poll(cx);
        let end = Instant::now().as_ticks() as u32;
        add_task_ticks(self.id.0, end.wrapping_sub(start));
        add_task_polls(self.id.0, 1);
        res
    }
}

pub trait TrackedExt: Future + Sized {
    fn tracked(self, id: TaskId) -> TrackedFuture<Self> {
        TrackedFuture { id, inner: self }
    }
}

impl<F: Future> TrackedExt for F {}

#[embassy_executor::task]
pub async fn stats_task(names: &'static [&'static str], stats_id: TaskId) {
    async move {
        let mut last_idle = 0u32;
        let mut last_task_ticks = [0u32; MAX_TASKS];
        let mut last_task_polls = [0u32; MAX_TASKS];
        let mut last_time = embassy_time::Instant::now();
        let mut last_interrupt_active = 0u32;
        let task_count = names.len();

        loop {
            embassy_time::Timer::after(Duration::from_secs(1)).await;
            let now = embassy_time::Instant::now();
            let idle = instrumented_executor::IDLE_TICKS.load(Ordering::Relaxed);
            let interrupt_active =
                instrumented_executor::INTERRUPT_ACTIVE_TICKS.load(Ordering::Relaxed);

            // Time Calculations
            let delta_idle = idle.wrapping_sub(last_idle);
            let delta_interrupt_active = interrupt_active.wrapping_sub(last_interrupt_active);
            let delta_time = (now - last_time).as_ticks() as u32;

            // info!("Idle: {}", delta_idle);
            // info!("Interrupt Active: {}", delta_interrupt_active);
            // info!("Time: {}", delta_time);

            // CPU Usage Calculations
            let delta_actual_idle = delta_idle.saturating_sub(delta_interrupt_active);
            let usage = 100.0 * (1.0 - (delta_actual_idle as f32 / delta_time as f32));
            let interrupt_usage = 100.0 * (delta_interrupt_active as f32 / delta_time as f32);
            let thread_usage = 100.0 * (delta_time.saturating_sub(delta_idle) as f32 / delta_time as f32);

            // Polls Calculations
            let thread_polls = instrumented_executor::POLL_COUNT.swap(0, Ordering::Relaxed);
            let interrupt_polls =
                instrumented_executor::INTERRUPT_POLL_COUNT.swap(0, Ordering::Relaxed);
            let total_polls = thread_polls + interrupt_polls;

            let mut task_usages = [0.0f32; MAX_TASKS];
            // Conditional state for threshold detection. Only compiled in when
            // constant tracking (verbose-utilization) is disabled.
            #[cfg(not(feature = "verbose-utilization"))]
            let mut any_task_exceeded = false;

            let mut task_poll_deltas = [0u32; MAX_TASKS];

            for i in 0..task_count {
                let ticks = get_task_ticks(i);
                let polls = get_task_polls(i);
                let delta = ticks.wrapping_sub(last_task_ticks[i]);
                let poll_delta = polls.wrapping_sub(last_task_polls[i]);
                let task_usage = 100.0 * (delta as f32 / delta_time as f32);
                task_usages[i] = task_usage;
                task_poll_deltas[i] = poll_delta;
                #[cfg(not(feature = "verbose-utilization"))]
                if task_usage > TASK_USAGE_THRESHOLD {
                    any_task_exceeded = true;
                }
                last_task_ticks[i] = ticks;
                last_task_polls[i] = polls;
            }

            // Diagnostic Switch: We use a "selection block" with direct #[cfg] attributes.
            // This ensures that either constant reporting or threshold logic is compiled in.
            #[cfg(feature = "verbose-utilization")]
            {
                info!(
                    "--- CPU Utilization: {} ({}/s) ---",
                    Percent(usage),
                    total_polls
                );
                info!(
                    "High Priority Task Utilization: {} ({}/s)",
                    Percent(interrupt_usage),
                    interrupt_polls
                );
                info!(
                    "Background Task Utilization: {} ({}/s)",
                    Percent(thread_usage),
                    thread_polls
                );
                for i in 0..task_count {
                    info!(
                        "  {}: {} ({}/s)",
                        names[i],
                        Percent(task_usages[i]),
                        task_poll_deltas[i]
                    );
                }
            }
            #[cfg(not(feature = "verbose-utilization"))]
            {
                if usage > TOTAL_USAGE_WARNING_THRESHOLD
                    || any_task_exceeded
                    || interrupt_usage > HIGH_PRIORITY_TASK_USAGE_THRESHOLD
                {
                    let level = if usage > TOTAL_USAGE_ALARM_THRESHOLD {
                        "ALARM"
                    } else {
                        "WARNING"
                    };

                    warn!(
                        "--- CPU Utilization {}: {} ({}/s) ---",
                        level,
                        Percent(usage),
                        total_polls
                    );
                    warn!(
                        "High Priority Task Utilization: {} ({}/s)",
                        Percent(interrupt_usage),
                        interrupt_polls
                    );
                    warn!(
                        "Background Task Utilization: {} ({}/s)",
                        Percent(thread_usage),
                        thread_polls
                    );

                    for i in 0..task_count {
                        if task_usages[i] > TASK_USAGE_THRESHOLD
                            || usage > TOTAL_USAGE_WARNING_THRESHOLD
                        {
                            warn!(
                                "  {}: {} ({}/s)",
                                names[i],
                                Percent(task_usages[i]),
                                task_poll_deltas[i]
                            );
                        }
                    }
                }
            }

            last_idle = idle;
            last_interrupt_active = interrupt_active;
            last_time = now;
        }
    }
    .tracked(stats_id)
    .await;
}

/// Defines the list of tasks to be tracked for utilization and verifies the count at compile-time.
///
/// This macro generates unique `TaskId` constants and a `TASK_NAMES` array for use in `stats_task`.
///
/// # Example
/// ```rust
/// define_utilization_tasks!(Wifi, Usb, Blinky);
/// ```
#[macro_export]
macro_rules! define_utilization_tasks {
    ($($name:ident),*) => {
        #[allow(non_camel_case_types)]
        pub enum TaskIdEnum {
            $($name),*
        }

        /// Human-readable names of the registered tasks.
        pub const TASK_NAMES: &[&'static str] = &[ $( stringify!($name) ),* ];

        // Compile-time assertion that we haven't exceeded MAX_TASKS.
        const _: () = assert!(TASK_NAMES.len() <= $crate::utilization::MAX_TASKS, "Too many tasks registered for utilization tracking!");

        $(
            #[allow(non_upper_case_globals)]
            /// Unique identifier for the task's utilization tracking.
            pub const $name: $crate::utilization::TaskId = $crate::utilization::TaskId(TaskIdEnum::$name as usize);
        )*
    }
}
