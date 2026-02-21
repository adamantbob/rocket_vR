#[cfg(not(feature = "verbose-utilization"))]
use crate::warn;
#[cfg(feature = "verbose-utilization")]
use crate::{debug, warn};
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use defmt_rtt as _;
use embassy_time::{Duration, Instant};
use portable_atomic::{AtomicU32, Ordering};
use rocket_core::CPUHealth;

use crate::health::stack::{self, sample_stack_usage};
use crate::instrumented_executor;
#[cfg(feature = "verbose-utilization")]
use crate::state_machine::SYSTEM_HEALTH;
use rocket_core::log::{LOG_CHANNEL, LogEntry};

// Utilization Tracking
// ===================
//
// This module provides a simple way to track the utilization of tasks in the system.
// It supports a flexible number of tasks up to MAX_TASKS.
// Compile-time safety is ensured by validating the task count during setup.
//
// All percentages are represented as u32 in units of 0.1% (tenths of a percent).
// 1000 == 100.0%, 10 == 1.0%, 1 == 0.1%.
// This avoids all floating-point arithmetic on the hot stats path.

/// A utilization percentage stored as a `u32` in units of 0.1%.
///
/// `1000` = 100.0%, `10` = 1.0%, `1` = 0.1%.
#[derive(Clone, Copy)]
struct Percent(u32);

impl Percent {
    /// Compute `(numerator / denominator) * 100`, returning tenths of a percent.
    ///
    /// Uses `saturating_mul` to prevent overflow on large tick deltas.
    /// Returns `Percent(0)` if `denominator` is zero.
    fn from_ticks(numerator: u32, denominator: u32) -> Self {
        if denominator == 0 {
            return Percent(0);
        }
        Percent(numerator.saturating_mul(1000) / denominator)
    }

    /// Saturating subtraction — clamps to zero rather than wrapping.
    fn saturating_sub(self, other: Percent) -> Percent {
        Percent(self.0.saturating_sub(other.0))
    }

    /// Return the whole-percent part, rounded up, for use in coarse health metrics.
    fn ceil_percent(self) -> u8 {
        ((self.0 + 9) / 10).min(100) as u8
    }
}

impl defmt::Format for Percent {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}.{}%", self.0 / 10, self.0 % 10)
    }
}

impl core::fmt::Display for Percent {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}.{}%", self.0 / 10, self.0 % 10)
    }
}

#[derive(Clone, Copy)]
pub struct TaskId(pub usize);

pub const MAX_TASKS: usize = 16;

// Thresholds in tenths of a percent (1000 == 100.0%).
#[cfg(not(feature = "verbose-utilization"))]
pub const TASK_USAGE_THRESHOLD: u32 = 100; // 10.0%
#[cfg(not(feature = "verbose-utilization"))]
pub const TOTAL_USAGE_WARNING_THRESHOLD: u32 = 400; // 40.0%
#[cfg(not(feature = "verbose-utilization"))]
pub const TOTAL_USAGE_ALARM_THRESHOLD: u32 = 800; // 80.0%
#[cfg(not(feature = "verbose-utilization"))]
pub const HIGH_PRIORITY_TASK_USAGE_THRESHOLD: u32 = 200; // 20.0%

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
    _core1_stack_range: Option<(*const u32, *const u32)>,
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

            // Sample stack for the core this task is running on.
            sample_stack_usage(0);

            let now = embassy_time::Instant::now();

            let metrics0 = &crate::METRICS_CORE0;
            let metrics1 = &crate::METRICS_CORE1;

            let idle0 = metrics0.idle_ticks.load(Ordering::Relaxed);
            let idle1 = metrics1.idle_ticks.load(Ordering::Relaxed);
            let interrupt_active =
                instrumented_executor::INTERRUPT_ACTIVE_TICKS.load(Ordering::Relaxed);

            let delta_idle0 = idle0.wrapping_sub(last_idle_c0);
            let delta_idle1 = idle1.wrapping_sub(last_idle_c1);
            let delta_interrupt_active = interrupt_active.wrapping_sub(last_interrupt_active);
            let delta_time = (now - last_time).as_ticks() as u32;

            if delta_time == 0 {
                continue;
            }

            // --- CPU Usage Calculations ---
            //
            // Core 0: delta_idle0 includes time spent in interrupts that fire during WFE,
            // so we subtract interrupt active time to get true thread-mode idle.
            let actual_idle0 = delta_idle0.saturating_sub(delta_interrupt_active);
            let usage0_total =
                Percent::from_ticks(delta_time.saturating_sub(actual_idle0), delta_time);

            // Core 1: thread activity only.
            let usage1_total =
                Percent::from_ticks(delta_time.saturating_sub(delta_idle1), delta_time);

            // Publish coarse CPU health (whole percent, rounded up).
            let cpu_health = CPUHealth::new_from_readings(
                usage0_total.ceil_percent(),
                usage1_total.ceil_percent(),
            );
            SYSTEM_HEALTH.cpu_health.update(cpu_health);
            let _ = LOG_CHANNEL.try_send(LogEntry::CPUHealth(cpu_health));

            // Per-task usage.
            let mut task_usages = [Percent(0); MAX_TASKS];
            let mut task_poll_deltas = [0u32; MAX_TASKS];

            for i in 0..task_count {
                let ticks = get_task_ticks(i);
                let polls = get_task_polls(i);

                task_usages[i] =
                    Percent::from_ticks(ticks.wrapping_sub(last_task_ticks[i]), delta_time);
                task_poll_deltas[i] = polls.wrapping_sub(last_task_polls[i]);

                last_task_ticks[i] = ticks;
                last_task_polls[i] = polls;
            }

            #[cfg(feature = "verbose-utilization")]
            {
                let poll0 = metrics0.poll_count.swap(0, Ordering::Relaxed);
                let poll1 = metrics1.poll_count.swap(0, Ordering::Relaxed);
                let interrupt_polls =
                    instrumented_executor::INTERRUPT_POLL_COUNT.swap(0, Ordering::Relaxed);

                let usage0_high_prio = Percent::from_ticks(delta_interrupt_active, delta_time);
                let usage0_background = usage0_total.saturating_sub(usage0_high_prio);

                // --- COMPACT REPORT FORMAT ---
                //
                // [Core0 <total> HP:<high_prio> BG:<background> | <polls>/s]
                //   <TaskName>: <pct> (<polls>/s)
                //   ...
                // [Core1 <total> Stk:<peak> (<used>/<total> bytes) | <polls>/s]
                //   <TaskName>: <pct> (<polls>/s)
                //   ...
                // [Heartbeats]
                //   <Subsystem>: <age>ms

                debug!(
                    "[Core0 {} HP:{} BG:{} | {}p/s]",
                    usage0_total,
                    usage0_high_prio,
                    usage0_background,
                    poll0 + interrupt_polls,
                );

                for i in 0..task_count {
                    if _task_cores[i] == 0 {
                        debug!(
                            "  {}: {} ({}/s)",
                            names[i], task_usages[i], task_poll_deltas[i]
                        );
                    }
                }

                if let Some((bottom, top)) = _core1_stack_range {
                    let watermark = stack::get_stack_high_watermark(bottom, top);
                    let total_size = (top as u32) - (bottom as u32);
                    let used_bytes = (top as u32) - watermark;
                    let peak = Percent::from_ticks(used_bytes, total_size);

                    debug!(
                        "[Core1 {} Stk:{} ({}/{} bytes) | {}p/s]",
                        usage1_total, peak, used_bytes, total_size, poll1,
                    );

                    // 85.0% == 850 in tenths-of-a-percent
                    if peak.0 > 850 {
                        warn!("ALARM: Core 1 stack nearing overflow! ({})", peak);
                    }
                } else {
                    debug!("[Core1 {} | {}p/s]", usage1_total, poll1);
                }

                for i in 0..task_count {
                    if _task_cores[i] == 1 {
                        debug!(
                            "  {}: {} ({}/s)",
                            names[i], task_usages[i], task_poll_deltas[i]
                        );
                    }
                }

                debug!("[Heartbeats]");

                let now_ticks = Instant::now().as_ticks() as u32;
                let check_liveness = |last_update: u32, name: &'static str| {
                    if last_update == 0 {
                        debug!("  {}: never", name);
                    } else {
                        let elapsed = now_ticks.wrapping_sub(last_update);
                        let age_ms = elapsed / 1000;
                        if elapsed > 2_000_000 {
                            warn!("ALARM: {} SUBSYSTEM STALLED ({}ms)", name, age_ms);
                        } else {
                            debug!("  {}: {}ms", name, age_ms);
                        }
                    }
                };

                check_liveness(SYSTEM_HEALTH.imu_health.last_updated(), "IMU");
                check_liveness(SYSTEM_HEALTH.gps_health.last_updated(), "GPS");
                check_liveness(SYSTEM_HEALTH.wifi_health.last_updated(), "Wifi");
                check_liveness(SYSTEM_HEALTH.sd_health.last_updated(), "SD");
            }

            #[cfg(not(feature = "verbose-utilization"))]
            {
                // In non-verbose mode, only warn when alarm threshold is crossed.
                if usage0_total.0 > TOTAL_USAGE_ALARM_THRESHOLD
                    || usage1_total.0 > TOTAL_USAGE_ALARM_THRESHOLD
                {
                    warn!(
                        "HIGH CPU LOAD: Core 0: {}, Core 1: {}",
                        usage0_total, usage1_total,
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
/// define_utilization_tasks!(Wifi [0], Usb [0], Blinky [1]);
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
        const _: () = assert!(TASK_NAMES.len() <= $crate::health::utilization::MAX_TASKS, "Too many tasks registered for utilization tracking!");

        $(
            #[allow(non_upper_case_globals)]
            /// Unique identifier for the task's utilization tracking.
            pub const $name: $crate::health::utilization::TaskId = $crate::health::utilization::TaskId(TaskIdEnum::$name as usize);
        )*
    }
}
