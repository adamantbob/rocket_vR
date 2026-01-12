use core::arch::asm;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};
use embassy_executor::Spawner;
use embassy_time::Instant;

pub static IDLE_TICKS: AtomicU32 = AtomicU32::new(0);
pub static POLL_COUNT: AtomicU32 = AtomicU32::new(0);
pub static WAKE: AtomicBool = AtomicBool::new(true);

#[unsafe(no_mangle)]
fn __pender(_: *mut ()) {
    WAKE.store(true, Ordering::Release);
    cortex_m::asm::sev();
}

/// This is a custom executor that:
/// 1. Uses the "Gated Sleep" pattern to avoid the Observer Effect (spin).
/// 2. Instruments the loop to count polls and idle usage.
pub struct InstrumentedExecutor {
    inner: embassy_executor::raw::Executor,
    not_send: PhantomData<*mut ()>,
}

impl InstrumentedExecutor {
    pub fn new() -> Self {
        // Initialize the internal executor with the address of our GLOBAL signal.
        // This is safe because WAKE is static and lives forever.
        let ctx = &WAKE as *const AtomicBool as *mut ();
        Self {
            inner: embassy_executor::raw::Executor::new(ctx),
            not_send: PhantomData,
        }
    }

    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        init(self.inner.spawner());

        log::info!("Starting InstrumentedExecutor (Global Signal Mode)");

        let mut local_polls = 0u32;
        let mut local_idle = 0u32;

        loop {
            // "Wake Gate": Only poll if the GLOBAL signal is set
            if WAKE.swap(false, Ordering::Acquire) {
                local_polls += 1;
                unsafe {
                    self.inner.poll();
                };
            }

            let start = Instant::now().as_ticks() as u32;

            // "Double-WFE": Clear any latch set by the poll atomics.
            unsafe {
                asm!("sev");
                asm!("wfe");
            }

            // Check signal again. If work arrived during the latch-clear, don't sleep.
            if !WAKE.load(Ordering::Relaxed) {
                unsafe { asm!("wfe") };
            }

            let end = Instant::now().as_ticks() as u32;
            local_idle = local_idle.wrapping_add(end.wrapping_sub(start));

            // Commit metrics
            if local_polls >= 10 || local_idle >= 50_000 {
                POLL_COUNT.fetch_add(local_polls, Ordering::Relaxed);
                IDLE_TICKS.fetch_add(local_idle, Ordering::Relaxed);
                local_polls = 0;
                local_idle = 0;
            }
        }
    }
}
