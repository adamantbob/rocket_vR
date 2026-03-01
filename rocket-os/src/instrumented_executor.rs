//! # Instrumented Embassy Executor for RP2350 (Dual-Core)
//!
//! This module provides a custom Embassy executor that instruments idle time and poll
//! counts for CPU utilization monitoring. It wraps [`embassy_executor::raw::Executor`]
//! and layers a software wake-gate on top to prevent spurious wakeups.
//!
//! ## Background: Why This Exists
//!
//! The stock Embassy `cortex-m` thread executor uses `SEV`/`WFE` for power-efficient
//! sleeping between task polls. During profiling on the RP2350, the executor was observed
//! waking up **millions of times per second** at idle — producing >50% apparent CPU
//! utilization despite no real work being done.
//!
//! Root cause: the Cordyceps intrusive linked-list used by Embassy's raw run-queue
//! performs atomic operations (compare-exchange, etc.) whose memory-barrier side-effects
//! can set the ARM event register, immediately clearing `WFE` and causing a tight
//! wake-poll-sleep loop. This is a known pitfall of the ARM event model.
//!
//! ## The Wake-Gate Fix (Single Core)
//!
//! An [`AtomicBool`] (`wake`) is introduced as a software gate:
//!
//! 1. `__pender` sets `wake = true` then calls `SEV` to unblock `WFE`.
//! 2. The run loop checks `wake`: if true, clears it and calls `inner.poll()`.
//! 3. After polling, the loop does `WFE` to sleep.
//! 4. Spurious atomic-caused wakeups find `wake == false` and skip polling.
//!
//! This filters out the Cordyceps-induced noise on a single core and brought observed
//! utilization down to the expected near-zero level at idle.
//!
//! ## The Dual-Core Ping-Pong Problem
//!
//! When a second core's executor was started, utilization spiked again. The culprit is
//! a fundamental property of the ARM `SEV` instruction:
//!
//! > **`SEV` is always a broadcast. It sets the event register on ALL cores simultaneously.
//! > There is no core-local form.**
//!
//! The original sleep sequence (a standard single-core ARM idiom) was:
//!
//! ```asm
//! SEV       ; "drain": guarantee the event register is set...
//! WFE       ; ...then clear it immediately (returns without sleeping)
//! ; check wake flag
//! WFE       ; now sleep for real — only a *new* event wakes us
//! ```
//!
//! On two cores, the drain `SEV` from Core 0 sets Core 1's event register.
//! Core 1 wakes, sees `wake == false`, emits its own drain `SEV` — which wakes Core 0.
//! They bounce off each other at millions of iterations per second even with no work to do.
//!
//! The same effect occurs from the `sev()` call inside `__pender`: necessary to unblock
//! the target core's `WFE`, but it also fires on the idle core.
//!
//! ### The Fix
//!
//! Remove the drain `SEV`+`WFE` pair from the hot loop. Replace it with a double
//! `WFE` with no preceding `SEV`:
//!
//! ```rust
//! if !self.wake.load(Ordering::Acquire) {
//!     unsafe {
//!         asm!("wfe"); // drain stale event register — no broadcast
//!         asm!("wfe"); // real sleep
//!     }
//! }
//! ```
//!
//! The first `WFE` does the drain the old `SEV`+`WFE` pair used to do, but without
//! any broadcast. If the event register is already set, it clears immediately and the
//! second `WFE` sleeps for real. If the register is clear, the first `WFE` sleeps
//! until a `SEV` arrives, then the second returns immediately. Either way, without a
//! broadcast `SEV` in the loop, the ping-pong cannot occur.
//!
//! ## Metrics
//!
//! [`ExecutorMetrics`] accumulates `idle_ticks` and `poll_count` which can be sampled
//! from any context to compute a utilization ratio:
//!
//! ```
//! utilization = 1.0 - (idle_ticks / total_elapsed_ticks)
//! ```

use core::arch::asm;
use core::marker::PhantomData;
use core::mem::MaybeUninit;
use embassy_executor::Spawner;
use embassy_executor::raw::Executor as RawExecutor;
use embassy_time::Instant;
use portable_atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering};

use cortex_m::interrupt::InterruptNumber;
use cortex_m::peripheral::NVIC;

/// Lowest address in RP2350 RAM / flash space.
///
/// Used in `__pender` to distinguish a real RAM pointer (context is an `*const AtomicBool`)
/// from a small-integer IRQ number or `THREAD_PENDER` sentinel. All valid RAM and flash
/// addresses on the RP2350 are above this threshold; IRQ numbers are at most a few hundred.
const MIN_RAM_ADDR: usize = 0x1000_0000;

/// Read the hardware CPUID register to determine which core we are running on.
///
/// Returns `0` for core 0 and `1` for core 1. The register is read-only and always
/// reflects the calling core, so this is safe to call from any context.
#[inline(always)]
fn cpuid() -> usize {
    // Safety: 0xD000_0000 is the read-only SCS CPUID register on Cortex-M;
    // reading it has no side effects and is always valid.
    unsafe { (*(0xD000_0000 as *const u32)) as usize }
}

/// Per-core pointers to each executor's `wake` signal.
///
/// Indexed by CPUID (0 or 1). Populated by [`InstrumentedExecutor::run`] before the
/// run loop starts. Used by the CPUID-based branch of `__pender` to signal the correct
/// executor when the context pointer is not a direct `AtomicBool` reference.
static CORE_WAKE_SIGNALS: [AtomicPtr<AtomicBool>; 2] = [
    AtomicPtr::new(core::ptr::null_mut()),
    AtomicPtr::new(core::ptr::null_mut()),
];

pub use rocket_core::utilization::ExecutorMetrics;

/// Sentinel context value used by Embassy's cortex-m thread-mode executor.
/// We replicate it here so `__pender` can distinguish interrupt vs. thread contexts.
const THREAD_PENDER: usize = usize::MAX;

/// Embassy's executor pender hook.
///
/// Embassy calls this function (via its internal `__pender` weak symbol) to notify the
/// executor that at least one task is ready to be polled. The `context` value was passed
/// to [`RawExecutor::new`] at construction time.
///
/// # Context Dispatch
///
/// Two context encodings are supported:
///
/// - **Pointer (`ctx >= MIN_RAM_ADDR`)**: The context is a raw `*const AtomicBool` pointing
///   directly to the executor's `wake` signal. We set it to `true` and call `SEV`.
///   This is the path used by [`InstrumentedExecutor`].
///
/// - **Small integer (`ctx < MIN_RAM_ADDR`)**: The context encodes either `THREAD_PENDER`
///   (do nothing extra) or an IRQ number to pend via NVIC. In this case we fall back to
///   signalling via [`CORE_WAKE_SIGNALS`] indexed by CPUID. This branch is unreachable
///   when only [`InstrumentedExecutor`] instances are in use (they always pass a pointer
///   context), but is retained as a compatibility path for IRQ-context executors.
///
/// # SEV Note
///
/// The unconditional `sev()` below is required: the sleeping core's `WFE` will not
/// return without an event being signalled. However, `SEV` is a broadcast — it wakes
/// ALL cores. This is acceptable here because:
///
/// - `wake` is set *before* `sev()`, so any core that wakes and checks the flag will
///   find a consistent state.
/// - The run loop (see [`InstrumentedExecutor::run`]) does not emit its own `SEV`,
///   so there is no feedback loop that would cause ping-pong.
#[unsafe(no_mangle)]
fn __pender(context: *mut ()) {
    let ctx = context as usize;

    if ctx >= MIN_RAM_ADDR {
        // Context is a direct pointer to the executor's `wake` AtomicBool.
        // Set it so the run loop knows to call poll() on its next iteration.
        unsafe {
            let signal = &*(context as *const AtomicBool);
            signal.store(true, Ordering::Release);
        }
    } else {
        // Context is a small integer (IRQ number or THREAD_PENDER).
        // Use the CPUID-indexed table to find the correct core's wake signal.
        let core = cpuid();
        if core < 2 {
            let signal_ptr = CORE_WAKE_SIGNALS[core].load(Ordering::Acquire);
            if !signal_ptr.is_null() {
                unsafe { (*signal_ptr).store(true, Ordering::Release) };
            }
        }
    }

    // Unblock the sleeping core's WFE.
    //
    // SEV broadcasts to all cores. The idle core may wake prematurely, but it will
    // check `wake == false` and return to sleep without emitting another SEV.
    // See module-level docs for the full dual-core analysis.
    cortex_m::asm::sev();

    // If the context encodes an IRQ number, pend it so the NVIC fires the handler.
    // IRQ numbers fit in a u16, so any valid IRQ context is below 0x10000.
    const MAX_IRQ_CTX: usize = 0x10000;
    if ctx < MAX_IRQ_CTX && ctx != THREAD_PENDER {
        #[derive(Clone, Copy)]
        struct Irq(u16);
        unsafe impl InterruptNumber for Irq {
            fn number(self) -> u16 {
                self.0
            }
        }
        let irq = Irq(ctx as u16);
        #[cfg(not(feature = "rp2040"))]
        unsafe {
            let mut nvic: core::mem::ManuallyDrop<NVIC> = core::mem::transmute(());
            nvic.request(irq);
        }
        #[cfg(feature = "rp2040")]
        NVIC::pend(irq);
    }
}

/// Thread-mode executor with idle-time and poll-count instrumentation.
///
/// Wraps [`RawExecutor`] and adds a software wake-gate ([`AtomicBool`]) to suppress
/// spurious wakeups caused by atomic operations in Embassy's Cordyceps run-queue.
///
/// # Usage
///
/// ```rust
/// static METRICS: ExecutorMetrics = ExecutorMetrics::new();
/// static mut EXECUTOR: InstrumentedExecutor = InstrumentedExecutor::new(&METRICS);
///
/// // On each core:
/// unsafe { EXECUTOR.run(|spawner| { spawner.spawn(my_task()).unwrap(); }) }
/// ```
///
/// # `!Send` Invariant
///
/// The `PhantomData<*mut ()>` marker prevents this type from crossing thread boundaries.
/// One instance must be created per core and run exclusively on that core.
pub struct InstrumentedExecutor {
    /// The underlying Embassy raw executor. `MaybeUninit` allows `const` construction
    /// since `RawExecutor::new` cannot be called at compile time.
    inner: MaybeUninit<RawExecutor>,

    /// Software wake-gate. Set to `true` by `__pender`; cleared to `false` by the run
    /// loop before calling `inner.poll()`. The run loop skips `WFE` if this is `true`,
    /// ensuring a poll always follows a legitimate wake signal.
    wake: AtomicBool,

    metrics: &'static ExecutorMetrics,
    /// Prevents this executor from being sent across cores; each core owns its instance.
    not_send: PhantomData<*mut ()>,
}

impl InstrumentedExecutor {
    pub const fn new(metrics: &'static ExecutorMetrics) -> Self {
        Self {
            inner: MaybeUninit::uninit(),
            // Start as `true` so the first iteration polls immediately, draining any
            // tasks that were spawned before the run loop started.
            wake: AtomicBool::new(true),
            metrics,
            not_send: PhantomData,
        }
    }

    /// Start the executor run loop on the calling core. Never returns.
    ///
    /// Registers this executor's `wake` pointer in [`CORE_WAKE_SIGNALS`] so that
    /// `__pender` can find it via CPUID when needed.
    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        CORE_WAKE_SIGNALS[cpuid()].store(&self.wake as *const _ as *mut _, Ordering::Release);

        // Pass the wake AtomicBool's address as the executor context so __pender can
        // set it directly via the pointer branch (ctx >= MIN_RAM_ADDR).
        let ctx = &self.wake as *const AtomicBool as *mut ();

        // Safety: `self` is `&'static mut`, so the address is stable for the program
        // lifetime. The `MaybeUninit` write is safe exactly once here.
        let inner = unsafe {
            self.inner.as_mut_ptr().write(RawExecutor::new(ctx));
            &*self.inner.as_ptr()
        };

        init(inner.spawner());

        let mut local_polls = 0u32;
        let mut local_idle = 0u32;

        loop {
            // --- POLL GATE ---
            // Only poll if __pender has signalled a genuine wake. The `swap` atomically
            // clears the flag and returns its previous value. This prevents a TOCTOU
            // race where a new signal arrives between the load and the poll call.
            if self.wake.swap(false, Ordering::Acquire) {
                local_polls += 1;
                unsafe { inner.poll() };
            }

            // Acquire fence: ensures all stores from poll() are visible before we
            // read the wake flag below, preventing the compiler or CPU from hoisting
            // the load above the poll work. SeqCst is not needed here.
            core::sync::atomic::fence(Ordering::Acquire);

            // --- SLEEP LOGIC ---
            //
            // The classic single-core ARM drain idiom uses SEV+WFE to clear any stale
            // event before the real WFE sleep:
            //
            //   WRONG on dual core (SEV is a broadcast — wakes Core 1!):
            //     SEV          ; set event register on ALL cores
            //     WFE          ; drain own event register
            //     WFE          ; real sleep
            //
            // On dual core, the broadcast SEV causes ping-pong: Core 1 wakes, sees
            // wake==false, emits its own drain SEV, which wakes Core 0, ad infinitum.
            //
            // The fix is a double WFE with no preceding SEV:
            //
            //   CORRECT (drain without broadcast):
            //     WFE          ; drain any stale event already in the register
            //     WFE          ; real sleep — only a new SEV from __pender wakes us
            //
            // The first WFE handles the case where the event register is already set
            // (from a prior __pender SEV, a hardware interrupt, or leftover state). If
            // it is set, WFE clears it and returns immediately without sleeping. The
            // second WFE then sleeps for real.
            //
            // If a new __pender SEV arrives between the two WFEs, the second WFE wakes
            // immediately — no work is missed, since wake is checked at the top of the
            // next loop iteration.
            //
            // If the event register happens to be clear when we enter, the first WFE
            // sleeps until a SEV arrives, the second WFE returns immediately (the SEV
            // re-set the register), and we loop. One extra iteration — harmless.
            if !self.wake.load(Ordering::Acquire) {
                // Only measure idle time for genuine sleeps, not the flag check overhead.
                let start = Instant::now().as_ticks() as u32;
                unsafe {
                    asm!("wfe"); // drain stale event register
                    asm!("wfe"); // real sleep
                }
                let end = Instant::now().as_ticks() as u32;
                local_idle = local_idle.wrapping_add(end.wrapping_sub(start));
            }

            // Flush local counters to the shared metrics every 10 polls or once
            // enough idle time has accumulated. Batching avoids hammering the shared
            // atomics on every loop iteration.
            if local_polls >= 10 || local_idle >= 20_000 {
                self.metrics
                    .poll_count
                    .fetch_add(local_polls, Ordering::Relaxed);
                self.metrics
                    .idle_ticks
                    .fetch_add(local_idle, Ordering::Relaxed);
                local_polls = 0;
                local_idle = 0;
            }
        }
    }
}

/// Shared metrics for the interrupt-mode executor.
///
/// Monotonically increasing; take deltas between samples for per-interval rates.
pub static INTERRUPT_ACTIVE_TICKS: AtomicU32 = AtomicU32::new(0);
pub static INTERRUPT_POLL_COUNT: AtomicU32 = AtomicU32::new(0);

/// Global reference to the interrupt executor instance for use in the interrupt handler
pub static INTERRUPT_EX_PTR: AtomicPtr<InstrumentedInterruptExecutor> =
    AtomicPtr::new(core::ptr::null_mut());

/// Interrupt-mode executor with active-time instrumentation.
///
/// Runs Embassy tasks inside an interrupt handler. Unlike [`InstrumentedExecutor`],
/// this executor does not need a sleep/wake mechanism — it is driven entirely by
/// NVIC interrupts and never calls `WFE`.
///
/// # Safety
///
/// [`on_interrupt`] must only be called from the interrupt handler whose IRQ number
/// was passed to [`start`]. Calling it before [`start`] is undefined behavior.
///
/// [`on_interrupt`]: InstrumentedInterruptExecutor::on_interrupt
/// [`start`]: InstrumentedInterruptExecutor::start
pub struct InstrumentedInterruptExecutor {
    executor: MaybeUninit<RawExecutor>,
    started: AtomicBool,
}

// Safety: The executor is pinned in a `static` and access is serialized by the NVIC
// (only one interrupt handler runs at a time per priority level).
unsafe impl Send for InstrumentedInterruptExecutor {}
unsafe impl Sync for InstrumentedInterruptExecutor {}

impl InstrumentedInterruptExecutor {
    pub const fn new() -> Self {
        Self {
            executor: MaybeUninit::uninit(),
            started: AtomicBool::new(false),
        }
    }

    /// Initialize the executor and unmask its IRQ.
    ///
    /// May only be called once. Panics on subsequent calls.
    pub fn start(&'static mut self, irq: impl InterruptNumber) -> embassy_executor::SendSpawner {
        if self.started.swap(true, Ordering::SeqCst) {
            panic!("InterruptExecutor::start() called multiple times");
        }

        let ctx = irq.number() as *mut ();
        let executor = unsafe {
            self.executor.as_mut_ptr().write(RawExecutor::new(ctx));
            &*self.executor.as_ptr()
        };

        // STORE THE POINTER: Now the ISR knows where we are
        INTERRUPT_EX_PTR.store(self as *mut _, Ordering::Release);

        unsafe { NVIC::unmask(irq) }
        executor.spawner().make_send()
    }

    /// Poll all ready tasks. Call this from the IRQ handler.
    ///
    /// # Safety
    ///
    /// Must be called from the interrupt whose number was passed to [`start`].
    /// [`start`] must have been called prior to this function.
    pub unsafe fn on_interrupt(&'static self) {
        let start = Instant::now().as_ticks() as u32;
        INTERRUPT_POLL_COUNT.fetch_add(1, Ordering::Relaxed);

        // Safety: Caller guarantees start() was called and we are in the correct ISR.
        let executor = unsafe { &*self.executor.as_ptr() };
        unsafe { executor.poll() };

        let end = Instant::now().as_ticks() as u32;
        INTERRUPT_ACTIVE_TICKS.fetch_add(end.wrapping_sub(start), Ordering::Relaxed);
    }
}
