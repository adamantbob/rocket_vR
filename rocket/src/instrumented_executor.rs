use core::arch::asm;
use core::marker::PhantomData;
use embassy_executor::Spawner;
use embassy_executor::raw::Executor as RawExecutor;
use embassy_time::Instant;
use portable_atomic::{AtomicBool, AtomicU32, Ordering};

use core::cell::{Cell, UnsafeCell};
use core::mem::MaybeUninit;

use cortex_m::interrupt::InterruptNumber;
use cortex_m::peripheral::NVIC;
use critical_section::Mutex;

pub static IDLE_TICKS: AtomicU32 = AtomicU32::new(0);
pub static POLL_COUNT: AtomicU32 = AtomicU32::new(0);
pub static WAKE: AtomicBool = AtomicBool::new(true);

const THREAD_PENDER: usize = usize::MAX;

#[unsafe(no_mangle)]
fn __pender(context: *mut ()) {
    let ctx = context as usize;

    if ctx == THREAD_PENDER || (ctx > 0x10000000) {
        // Check if context looks like a pointer (stack/heap address > 0x10000000 approx)
        // If so, it's our AtomicBool signal.
        if ctx > 0x10000000 {
            unsafe {
                let signal = &*(context as *const portable_atomic::AtomicBool);
                signal.store(true, portable_atomic::Ordering::Release);
            }
        }
        WAKE.store(true, Ordering::Release);
        cortex_m::asm::sev();

        return;
    }

    #[derive(Clone, Copy)]
    struct Irq(u16);
    unsafe impl InterruptNumber for Irq {
        fn number(self) -> u16 {
            self.0
        }
    }

    let irq = Irq(ctx as u16);

    // STIR is faster, but is only available in v7 and higher.
    #[cfg(not(feature = "rp2040"))]
    {
        unsafe {
            let mut nvic: NVIC = core::mem::transmute(());
            nvic.request(irq);
        }
    }

    #[cfg(feature = "rp2040")]
    NVIC::pend(irq);
}

/// This is a custom executor that:
/// 1. Uses the "Gated Sleep" pattern to avoid the Observer Effect (spin).
/// 2. Instruments the loop to count polls and idle usage.
pub struct InstrumentedExecutor {
    inner: RawExecutor,
    not_send: PhantomData<*mut ()>,
}

impl InstrumentedExecutor {
    pub fn new() -> Self {
        // Initialize the internal executor with the address of our GLOBAL signal.
        // This is safe because WAKE is static and lives forever.
        let ctx = &WAKE as *const AtomicBool as *mut ();
        Self {
            inner: RawExecutor::new(ctx),
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

// Separate atomics for interrupt executor tracking
pub static INTERRUPT_ACTIVE_TICKS: AtomicU32 = AtomicU32::new(0);
pub static INTERRUPT_POLL_COUNT: AtomicU32 = AtomicU32::new(0);

/// Instrumented wrapper around embassy's raw executor for interrupt-driven execution.
/// This Module duplicates the cortex-m interrupt executor but with instrumentation.
/// Tracks poll counts and active time for interrupt-driven execution.
///
/// Unlike InstrumentedExecutor which runs in a loop with sleep,
/// this executor is event-driven and runs only when the interrupt fires.
///
/// This provides an API similar to embassy's InterruptExecutor but with instrumentation.
pub struct InstrumentedInterruptExecutor {
    started: Mutex<Cell<bool>>,
    executor: UnsafeCell<MaybeUninit<RawExecutor>>,
}
unsafe impl Send for InstrumentedInterruptExecutor {}
unsafe impl Sync for InstrumentedInterruptExecutor {}

impl InstrumentedInterruptExecutor {
    /// Create a new InstrumentedInterruptExecutor.
    ///
    /// This is a const function that can be used in static initialization.
    #[inline]
    pub const fn new() -> Self {
        Self {
            started: Mutex::new(Cell::new(false)),
            executor: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    /// Start the executor.
    ///
    /// This initializes the executor, enables the interrupt, and returns.
    /// The executor keeps running in the background through the interrupt.
    ///
    /// This returns a [`SendSpawner`] you can use to spawn tasks on it. A [`SendSpawner`]
    /// is returned instead of a [`Spawner`](crate::Spawner) because the executor effectively runs in a
    /// different "thread" (the interrupt), so spawning tasks on it is effectively
    /// sending them.
    ///
    /// To obtain a [`Spawner`](crate::Spawner) for this executor, use [`Spawner::for_current_executor()`](crate::Spawner::for_current_executor()) from
    /// a task running in it.
    ///
    /// # Interrupt requirements
    ///
    /// You must write the interrupt handler yourself, and make it call [`on_interrupt()`](Self::on_interrupt).
    ///
    /// This method already enables (unmasks) the interrupt, you must NOT do it yourself.
    ///
    /// You must set the interrupt priority before calling this method. You MUST NOT
    /// do it after.
    ///
    /// [`SendSpawner`]: crate::SendSpawner
    pub fn start(&'static self, irq: impl InterruptNumber) -> embassy_executor::SendSpawner {
        if critical_section::with(|cs| self.started.borrow(cs).replace(true)) {
            panic!("InterruptExecutor::start() called multiple times on the same executor.");
        }

        unsafe {
            (&mut *self.executor.get())
                .as_mut_ptr()
                .write(RawExecutor::new(irq.number() as *mut ()))
        }

        let executor = unsafe { (&*self.executor.get()).assume_init_ref() };

        unsafe { NVIC::unmask(irq) }

        executor.spawner().make_send()
    }

    /// Get the spawner for this executor.
    ///
    /// Returns a SendSpawner that can be used to spawn tasks on this executor.
    // pub fn spawner(&'static self) -> embassy_executor::SendSpawner {
    //     if !critical_section::with(|cs| self.started.borrow(cs).get()) {
    //         panic!("InterruptExecutor::spawner() called on uninitialized executor.");
    //     }
    //     let executor = unsafe { (&*self.executor.get()).assume_init_ref() };
    //     executor.spawner().make_send()
    // }

    /// Interrupt handler callback.
    ///
    /// This MUST be called from the interrupt handler.
    /// It polls the executor and tracks the poll count and active time.
    ///
    /// # Safety
    /// You MUST only call this from the interrupt handler for your chosen IRQ.
    pub unsafe fn on_interrupt(&'static self) {
        let start = Instant::now().as_ticks() as u32;

        INTERRUPT_POLL_COUNT.fetch_add(1, Ordering::Relaxed);

        let executor = unsafe { (&*self.executor.get()).assume_init_ref() };
        unsafe {
            executor.poll();
        }

        let end = Instant::now().as_ticks() as u32;
        INTERRUPT_ACTIVE_TICKS.fetch_add(end.wrapping_sub(start), Ordering::Relaxed);
    }
}
