use crate::{deci_percent, health_types::DeciPercent};
use core::future::Future;
use core::pin::Pin;
use core::task::{Context, Poll};
use embassy_time::Instant;
use portable_atomic::{AtomicU32, Ordering};

#[derive(Clone, Copy, Debug)]
pub struct TaskId(pub usize);

pub const MAX_TASKS: usize = 16;

// Thresholds in tenths of a percent (1000 == 100.0%).
pub const TASK_USAGE_WARNING_THRESHOLD: DeciPercent = deci_percent!(15);
pub const TOTAL_USAGE_WARNING_THRESHOLD: DeciPercent = deci_percent!(40);
pub const TOTAL_USAGE_ALARM_THRESHOLD: DeciPercent = deci_percent!(80);
pub const HIGH_PRIORITY_TASK_USAGE_THRESHOLD: DeciPercent = deci_percent!(20);
pub const STACK_USAGE_ALARM_THRESHOLD: DeciPercent = deci_percent!(80);

// Multicore safe profiling using atomics.
pub static TASK_TICKS: [AtomicU32; MAX_TASKS] = [const { AtomicU32::new(0) }; MAX_TASKS];
pub static TASK_POLLS: [AtomicU32; MAX_TASKS] = [const { AtomicU32::new(0) }; MAX_TASKS];

pub fn add_task_ticks(id: usize, delta: u32) {
    if id < MAX_TASKS {
        TASK_TICKS[id].fetch_add(delta, Ordering::Relaxed);
    }
}

pub fn add_task_polls(id: usize, delta: u32) {
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

/// Accumulated metrics for one executor instance.
pub struct ExecutorMetrics {
    pub idle_ticks: AtomicU32,
    pub poll_count: AtomicU32,
}

impl ExecutorMetrics {
    pub const fn new() -> Self {
        Self {
            idle_ticks: AtomicU32::new(0),
            poll_count: AtomicU32::new(0),
        }
    }
}

pub static METRICS_CORE0: ExecutorMetrics = ExecutorMetrics::new();
pub static METRICS_CORE1: ExecutorMetrics = ExecutorMetrics::new();
