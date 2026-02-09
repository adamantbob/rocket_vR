#[cfg(feature = "verbose-utilization")]
use crate::info;
#[cfg(not(feature = "verbose-utilization"))]
use crate::warn;
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use embassy_time::{Duration, Instant};
use portable_atomic::{AtomicU32, Ordering};
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

// Multicore safe profiling using atomics.
static TASK_TICKS: [AtomicU32; MAX_TASKS] = [const { AtomicU32::new(0) }; MAX_TASKS];
static TASK_POLLS: [AtomicU32; MAX_TASKS] = [const { AtomicU32::new(0) }; MAX_TASKS];

fn get_task_ticks(id: usize) -> u32 {
    TASK_TICKS[id].load(Ordering::Relaxed)
}

fn add_task_ticks(id: usize, delta: u32) {
    if id < MAX_TASKS {
        TASK_TICKS[id].fetch_add(delta, Ordering::Relaxed);
    }
}

fn get_task_polls(id: usize) -> u32 {
    TASK_POLLS[id].load(Ordering::Relaxed)
}

fn add_task_polls(id: usize, delta: u32) {
    if id < MAX_TASKS {
        TASK_POLLS[id].fetch_add(delta, Ordering::Relaxed);
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
pub async fn stats_task(
    names: &'static [&'static str],
    _task_cores: &'static [u8],
    stats_id: TaskId,
) {
    async move {
        let mut last_idle_c0 = 0u32;
        let mut last_idle_c1 = 0u32;
        let mut last_task_ticks = [0u32; MAX_TASKS];
        let mut last_task_polls = [0u32; MAX_TASKS];
        let mut last_time = embassy_time::Instant::now();
        let mut last_interrupt_active = 0u32;
        let task_count = names.len();

        loop {
            embassy_time::Timer::after(Duration::from_secs(1)).await;
            let now = embassy_time::Instant::now();

            let metrics0 = &crate::METRICS_CORE0;
            let metrics1 = &crate::METRICS_CORE1;

            let idle0 = metrics0.idle_ticks.load(Ordering::Relaxed);
            let idle1 = metrics1.idle_ticks.load(Ordering::Relaxed);

            #[allow(unused_variables)]
            let poll0 = metrics0.poll_count.swap(0, Ordering::Relaxed);
            #[allow(unused_variables)]
            let poll1 = metrics1.poll_count.swap(0, Ordering::Relaxed);

            let interrupt_active =
                instrumented_executor::INTERRUPT_ACTIVE_TICKS.load(Ordering::Relaxed);
            #[allow(unused_variables)]
            let interrupt_polls =
                instrumented_executor::INTERRUPT_POLL_COUNT.swap(0, Ordering::Relaxed);

            // Time Calculations
            let delta_idle0 = idle0.wrapping_sub(last_idle_c0);
            let delta_idle1 = idle1.wrapping_sub(last_idle_c1);
            let delta_interrupt_active = interrupt_active.wrapping_sub(last_interrupt_active);
            let delta_time = (now - last_time).as_ticks() as u32;

            if delta_time == 0 {
                continue;
            }

            // --- CPU Usage Calculations ---

            // Core 0: Has both thread and interrupt activity.
            // Note: delta_idle0 includes time spent in interrupts that fire during WFE.
            // True Idle = (Total Idle Ticks reported by executor) - (Interrupt Active Ticks).
            let actual_idle0 = delta_idle0.saturating_sub(delta_interrupt_active);
            #[allow(unused_variables)]
            let usage0_total = 100.0 * (1.0 - (actual_idle0 as f32 / delta_time as f32));
            #[allow(unused_variables)]
            let usage0_high_prio = 100.0 * (delta_interrupt_active as f32 / delta_time as f32);
            #[allow(unused_variables)]
            let usage0_background = usage0_total - usage0_high_prio;

            // Core 1: Currently only thread activity tracked.
            #[allow(unused_variables)]
            let usage1_total = 100.0 * (1.0 - (delta_idle1 as f32 / delta_time as f32));

            let mut task_usages = [0.0f32; MAX_TASKS];
            let mut task_poll_deltas = [0u32; MAX_TASKS];

            for i in 0..task_count {
                let ticks = get_task_ticks(i);
                let polls = get_task_polls(i);
                let delta = ticks.wrapping_sub(last_task_ticks[i]);
                let poll_delta = polls.wrapping_sub(last_task_polls[i]);

                task_usages[i] = 100.0 * (delta as f32 / delta_time as f32);
                task_poll_deltas[i] = poll_delta;

                last_task_ticks[i] = ticks;
                last_task_polls[i] = polls;
            }

            #[cfg(feature = "verbose-utilization")]
            {
                info!("--- CPU Utilization (1s avg) ---");
                info!(
                    "Core 0 Total:   {} ({} poll/s)",
                    Percent(usage0_total),
                    poll0 + interrupt_polls
                );
                info!(
                    "  High Prio:    {} ({}/s)",
                    Percent(usage0_high_prio),
                    interrupt_polls
                );
                info!(
                    "  Background:   {} ({}/s)",
                    Percent(usage0_background.max(0.0)),
                    poll0
                );

                for i in 0..task_count {
                    if _task_cores[i] == 0 {
                        info!(
                            "    {}: {} ({}/s)",
                            names[i],
                            Percent(task_usages[i]),
                            task_poll_deltas[i]
                        );
                    }
                }

                info!(
                    "Core 1 Total:   {} ({} poll/s)",
                    Percent(usage1_total),
                    poll1
                );
                for i in 0..task_count {
                    if _task_cores[i] == 1 {
                        info!(
                            "    {}: {} ({}/s)",
                            names[i],
                            Percent(task_usages[i]),
                            task_poll_deltas[i]
                        );
                    }
                }
            }

            #[cfg(not(feature = "verbose-utilization"))]
            {
                // Summary/Warning logic can be added here if needed,
                // but let's stick to the verbose report the user is watching.
                if usage0_total > TOTAL_USAGE_ALARM_THRESHOLD
                    || usage1_total > TOTAL_USAGE_ALARM_THRESHOLD
                {
                    warn!(
                        "HIGH CPU LOAD: Core 0: {}, Core 1: {}",
                        Percent(usage0_total),
                        Percent(usage1_total)
                    );
                }
            }

            last_idle_c0 = idle0;
            last_idle_c1 = idle1;
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
    ($($name:ident [$core:expr]),*) => {
        #[allow(non_camel_case_types)]
        pub enum TaskIdEnum {
            $($name),*
        }

        /// Human-readable names of the registered tasks.
        pub const TASK_NAMES: &[&'static str] = &[ $( stringify!($name) ),* ];

        /// Core assignments for the registered tasks.
        pub const TASK_CORES: &[u8] = &[ $( $core ),* ];

        // Compile-time assertion that we haven't exceeded MAX_TASKS.
        const _: () = assert!(TASK_NAMES.len() <= $crate::utilization::MAX_TASKS, "Too many tasks registered for utilization tracking!");

        $(
            #[allow(non_upper_case_globals)]
            /// Unique identifier for the task's utilization tracking.
            pub const $name: $crate::utilization::TaskId = $crate::utilization::TaskId(TaskIdEnum::$name as usize);
        )*
    }
}
