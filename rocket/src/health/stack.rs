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

/// Safety wrapper to paint the current stack.
///
/// Calculates the stack range based on the current SP, a safety margin,
/// and the desired paint size.
///
/// # Arguments
/// * `safety_margin` - Number of bytes below current SP to leave untouched.
/// * `paint_size` - Number of bytes to paint below the safety margin.
///
/// # Returns
/// * `(start, end)` - The tuple of (bottom, top) addresses of the painted region.
///
/// # Safety
/// Caller must ensure `paint_size` does not exceed available stack memory.
pub unsafe fn paint_current_stack(safety_margin: u32, paint_size: u32) -> (u32, u32) {
    let sp = get_current_sp();
    let stack_top = (sp - safety_margin) as *mut u32;
    let stack_bottom = (sp - (safety_margin + paint_size)) as *mut u32;
    unsafe { paint_stack(stack_bottom, stack_top) };
    (stack_bottom as u32, stack_top as u32)
}

pub struct StackStats {
    pub current_usage_bytes: u32,
    pub peak_usage_bytes: u32,
    pub total_size_bytes: u32,
}

impl StackStats {
    pub fn usage_percent(&self) -> f32 {
        (self.peak_usage_bytes as f32 / self.total_size_bytes as f32) * 100.0
    }

    pub fn current_percent(&self) -> f32 {
        (self.current_usage_bytes as f32 / self.total_size_bytes as f32) * 100.0
    }
}
