use embassy_time::Duration;
#[cfg(feature = "verbose-utilization")]
use embassy_time::Instant;
use portable_atomic::Ordering;
use rocket_core::CPUHealth;
#[cfg(feature = "verbose-utilization")]
use rocket_core::{debug, error, warn};
#[cfg(not(feature = "verbose-utilization"))]
use rocket_core::{error, warn};

use crate::health::stack::{self, sample_stack_usage};
use crate::instrumented_executor;
use rocket_core::blackboard::SYSTEM_HEALTH;
use rocket_core::health_types::DeciPercent;
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

pub use rocket_core::utilization::{
    MAX_TASKS, TaskId, TrackedExt, TrackedFuture, add_task_polls, add_task_ticks,
};

fn get_task_ticks(id: usize) -> u32 {
    rocket_core::utilization::TASK_TICKS[id].load(Ordering::Relaxed)
}

fn get_task_polls(id: usize) -> u32 {
    rocket_core::utilization::TASK_POLLS[id].load(Ordering::Relaxed)
}

#[embassy_executor::task]
pub async fn stats_task(
    names: &'static [&'static str],
    _task_cores: &'static [u8],
    stats_id: TaskId,
    _core1_stack_range: (*const u32, *const u32),
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

            let metrics0 = &rocket_core::utilization::METRICS_CORE0;
            let metrics1 = &rocket_core::utilization::METRICS_CORE1;

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
                DeciPercent::from_ticks(delta_time.saturating_sub(actual_idle0), delta_time);

            let usage0_high_prio = DeciPercent::from_ticks(delta_interrupt_active, delta_time);
            let usage0_background = usage0_total.saturating_sub(usage0_high_prio);

            // Core 1: thread activity only.
            let usage1_total =
                DeciPercent::from_ticks(delta_time.saturating_sub(delta_idle1), delta_time);

            // Core 1: Stack Usage
            let (bottom, top) = _core1_stack_range;
            let usage1_stack_watermark = stack::get_stack_high_watermark(bottom, top);
            let usage1_stack_total_size = (top as u32) - (bottom as u32);
            let usage1_stack_used_bytes = (top as u32) - usage1_stack_watermark;
            let usage1_stack_hwm =
                DeciPercent::from_ticks(usage1_stack_used_bytes, usage1_stack_total_size);

            // Publish coarse CPU health (whole percent, rounded up).
            let cpu_health = CPUHealth::new_from_readings(
                usage0_total,
                usage0_high_prio,
                usage0_background,
                usage1_total,
                usage1_stack_hwm,
            );
            SYSTEM_HEALTH.cpu_health.update(cpu_health);
            let _ = LOG_CHANNEL.try_send(LogEntry::CPUHealth(cpu_health));

            // Per-task usage.
            let mut task_usages = [DeciPercent::ZERO; MAX_TASKS];
            let mut task_poll_deltas = [0u32; MAX_TASKS];

            for i in 0..task_count {
                let ticks = get_task_ticks(i);
                let polls = get_task_polls(i);

                task_usages[i] =
                    DeciPercent::from_ticks(ticks.wrapping_sub(last_task_ticks[i]), delta_time);
                task_poll_deltas[i] = polls.wrapping_sub(last_task_polls[i]);

                last_task_ticks[i] = ticks;
                last_task_polls[i] = polls;

                if task_usages[i] > rocket_core::utilization::TASK_USAGE_WARNING_THRESHOLD {
                    warn!("HIGH CPU USAGE: Task {}: {}", names[i], task_usages[i]);
                }
            }

            // Warn when alarm threshold is crossed.
            if usage0_total > rocket_core::utilization::TOTAL_USAGE_WARNING_THRESHOLD
                || usage1_total > rocket_core::utilization::TOTAL_USAGE_WARNING_THRESHOLD
            {
                warn!(
                    "HIGH CPU LOAD: Core 0: {}, Core 1: {}",
                    usage0_total, usage1_total,
                );
            }
            if usage0_total > rocket_core::utilization::TOTAL_USAGE_ALARM_THRESHOLD
                || usage1_total > rocket_core::utilization::TOTAL_USAGE_ALARM_THRESHOLD
            {
                error!(
                    "CRITICAL CPU LOAD: Core 0: {}, Core 1: {}",
                    usage0_total, usage1_total,
                );
            }
            if usage0_high_prio > rocket_core::utilization::HIGH_PRIORITY_TASK_USAGE_THRESHOLD {
                error!(
                    "CRITICAL HIGH PRIORITY CPU USAGE: Core 0: {}",
                    usage0_high_prio,
                );
            }
            if usage0_background > rocket_core::utilization::TOTAL_USAGE_ALARM_THRESHOLD {
                error!(
                    "CRITICAL BACKGROUND CPU USAGE: Core 0: {}",
                    usage0_background,
                );
            }
            if usage1_stack_hwm > rocket_core::utilization::STACK_USAGE_ALARM_THRESHOLD {
                warn!("HIGH STACK USAGE: Core 1: Stack HWM: {}", usage1_stack_hwm,);
            }

            #[cfg(feature = "verbose-utilization")]
            {
                let poll0 = metrics0.poll_count.swap(0, Ordering::Relaxed);
                let poll1 = metrics1.poll_count.swap(0, Ordering::Relaxed);
                let interrupt_polls =
                    instrumented_executor::INTERRUPT_POLL_COUNT.swap(0, Ordering::Relaxed);

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

                debug!(
                    "[Core1 {} Stk:{} ({}/{} bytes) | {}p/s]",
                    usage1_total,
                    usage1_stack_hwm,
                    usage1_stack_used_bytes,
                    usage1_stack_total_size,
                    poll1,
                );

                if usage1_stack_hwm > rocket_core::utilization::STACK_USAGE_ALARM_THRESHOLD {
                    warn!(
                        "ALARM: Core 1 stack nearing overflow! ({})",
                        usage1_stack_hwm
                    );
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

            last_idle_c0 = idle0;
            last_idle_c1 = idle1;
            last_interrupt_active = interrupt_active;
            last_time = now;
        }
    }
    .tracked(stats_id)
    .await;
}
