#![no_std]
#![no_main]

use cortex_m_rt::entry;
use embassy_rp as _;
use panic_halt as _;

#[entry]
fn main() -> ! {
    let uart0 = 0x40070000usize as *mut u8;

    // Print the message 5 times then stop
    for _ in 0..5 {
        for &b in b"RENODE CANARY: ALIVE\n" {
            unsafe {
                core::ptr::write_volatile(uart0, b);
            }
        }

        // Simple delay
        for _ in 0..1_000_000 {
            cortex_m::asm::nop();
        }
    }

    // Trigger a Magic Exit in Renode by writing to SIO FIFO
    unsafe {
        core::ptr::write_volatile(0xD0000050usize as *mut u32, 0xDEADBEEF);
    }

    // Fallback stop
    cortex_m::asm::udf();
}
