use portable_atomic::{AtomicU32, Ordering};

/// Sentinel pattern used to detect uninitialized stack memory.
const STACK_PAINT_PATTERN: u32 = 0xDEADBEEF;

/// Tracks the observed minimum stack pointer for each core via sampling.
/// Index 0 = Core 0, Index 1 = Core 1.
pub static OBSERVED_MIN_SP: [AtomicU32; 2] = [const { AtomicU32::new(u32::MAX) }; 2];

/// Returns the current value of the Main Stack Pointer (MSP).
#[inline(always)]
pub fn get_current_sp() -> u32 {
    let sp: u32;
    unsafe {
        core::arch::asm!("mrs {}, msp", out(reg) sp);
    }
    sp
}

/// Updates the observed minimum stack pointer for the current core.
/// This should be called periodically (e.g., in a background task).
pub fn sample_stack_usage(core: usize) {
    if core < 2 {
        let current_sp = get_current_sp();
        let _ = OBSERVED_MIN_SP[core].fetch_min(current_sp, Ordering::Relaxed);
    }
}

/// Fills the unused portion of a stack with a sentinel pattern.
///
/// # Safety
/// The caller must ensure that `stack_limit` and `stack_top` are valid boundaries
/// and that the stack is NOT currently using the memory between these points.
pub unsafe fn paint_stack(stack_bottom: *mut u32, stack_top: *mut u32) {
    let mut curr = stack_bottom;
    // We leave some margin at the top (where SP currently is) to avoid overwriting ourselves.
    // Since this is called at boot, we can be aggressive.
    while curr < stack_top {
        unsafe {
            curr.write_volatile(STACK_PAINT_PATTERN);
            curr = curr.add(1);
        }
    }
}

/// Scans the stack memory to find the first word that isn't the sentinel pattern.
/// This determines the absolute high-watermark since the last paint.
pub fn get_stack_high_watermark(stack_bottom: *const u32, stack_top: *const u32) -> u32 {
    let mut curr = stack_bottom;
    while curr < stack_top {
        unsafe {
            if curr.read_volatile() != STACK_PAINT_PATTERN {
                return curr as u32;
            }
        }
        curr = unsafe { curr.add(1) };
    }
    stack_top as u32
}

/// Sets up the high watermark tracking for core 1's stack.
///
/// # Arguments
/// * `stack` - The stack to track.
///
/// # Safety
/// Caller must ensure this is called at the very beginning of the core's execution context.
pub unsafe fn setup_core1_stack_high_watermark_tracking(stack_ptr: *mut u32, stack_size: usize) {
    // We leave 1KB at the top to avoid overwriting the current frame.
    unsafe {
        let stack_top = (stack_ptr as *mut u8).add(stack_size - 1024) as *mut u32;
        paint_stack(stack_ptr, stack_top)
    };
}
