use core::panic::PanicInfo;
use defmt::error;
use embassy_rp::pac;

pub const STATE_SIGNATURE_FINAL: u32 = 0xDEADC0DE;
pub const STATE_SIGNATURE_PARTIAL: u32 = 0xDEADC000;
pub const PANIC_MESSAGE_CAP: usize = 1024;
pub const PANIC_FILE_CAP: usize = 64;

/*
 * =========================================================================================
 *                              PANIC & HARD FAULT DEBUGGING GUIDE
 * =========================================================================================
 *
 * Faults accompanied by a stack dump should be debugged using the following procedure:
 *
 * 1. Open a terminal in the project root.
 * 2. Execute the `addr2line` command on the binary to resolve specific addresses.
 *
 * COMMAND:
 *   addr2line -e target/thumbv8m.main-none-eabihf/debug/rocket_vR <PC> <LR> <STACK_ADDRS...>
 *
 * EXAMPLE:
 *   addr2line -e target/thumbv8m.main-none-eabihf/debug/rocket_vR 0x1001bc56 0x100110d9 0x2000196c ...
 *
 * INTERPRETING OUTPUT:
 * - PC (Program Counter): The instruction being executed at the moment of the fault.
 * - LR (Link Register): The return address to the calling function.
 * - STACK: The list of addresses on the stack. Resolution helps reconstruct the call site.
 *
 * NOTES:
 * - Ensure the absolute path to the project binary is correct.
 * - `addr2line` must be installed: `cargo install addr2line`.
 * =========================================================================================
 */

#[repr(C)]
pub struct PanicRecord {
    pub state_signature: u32,
    pub core_id: u32,
    pub message: [u8; PANIC_MESSAGE_CAP],
    pub message_len: usize,
    pub file: [u8; PANIC_FILE_CAP],
    pub file_len: usize,
    pub line: u32,
}

/// Information extracted from a PanicRecord.
pub struct CrashReport {
    pub core_id: u32,
    pub message: heapless::String<PANIC_MESSAGE_CAP>,
    pub file: heapless::String<PANIC_FILE_CAP>,
    pub line: u32,
}

#[unsafe(link_section = ".uninit")]
pub static mut PANIC_RECORDS: [PanicRecord; 2] = [
    PanicRecord {
        state_signature: 0,
        core_id: 0,
        message: [0; PANIC_MESSAGE_CAP],
        message_len: 0,
        file: [0; PANIC_FILE_CAP],
        file_len: 0,
        line: 0,
    },
    PanicRecord {
        state_signature: 0,
        core_id: 0,
        message: [0; PANIC_MESSAGE_CAP],
        message_len: 0,
        file: [0; PANIC_FILE_CAP],
        file_len: 0,
        line: 0,
    },
];

struct VolatileBufWriter {
    buf_ptr: *mut u8,
    cap: usize,
    pos: usize,
}

impl core::fmt::Write for VolatileBufWriter {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let len = s.len().min(self.cap - self.pos);
        if len > 0 {
            // Copy byte-by-byte to volatile memory
            let src = s.as_bytes();
            for i in 0..len {
                unsafe {
                    core::ptr::write_volatile(self.buf_ptr.add(self.pos + i), src[i]);
                }
            }
            self.pos += len;
        }
        Ok(())
    }
}

/// Checks the state of a specific core for panic signatures.
///
/// This checks both the hardware FIFO (if called from the other core)
/// and the RAM-based state signature. If a crash is detected, the report is
/// extracted and the signature is optionally cleared.
pub fn check_core_panic(target_core: usize) -> Option<CrashReport> {
    if target_core >= 2 {
        return None;
    }

    unsafe {
        // 1. Hardware Sentinel Check (SIO FIFO)
        // Note: Reads the FIFO for the current core, which contains data sent by the other core.
        let sio = pac::SIO;
        if sio.fifo().st().read().vld() {
            let val = sio.fifo().rd().read();
            // We only care if the FIFO contains the sentinel
            if val == STATE_SIGNATURE_PARTIAL {
                // Return a partial report if the RAM isn't ready yet
                let sig = core::ptr::read_volatile(core::ptr::addr_of!(
                    PANIC_RECORDS[target_core].state_signature
                ));
                if sig != STATE_SIGNATURE_FINAL {
                    return Some(CrashReport {
                        core_id: target_core as u32,
                        message: heapless::String::try_from("(Incomplete formatting)").unwrap(),
                        file: heapless::String::try_from("?").unwrap(),
                        line: 0,
                    });
                }
            }
        }

        // 2. RAM Signature Check
        let sig = core::ptr::read_volatile(core::ptr::addr_of!(
            PANIC_RECORDS[target_core].state_signature
        ));
        if sig == STATE_SIGNATURE_FINAL || sig == STATE_SIGNATURE_PARTIAL {
            let core_id =
                core::ptr::read_volatile(core::ptr::addr_of!(PANIC_RECORDS[target_core].core_id));
            let msg_len = core::ptr::read_volatile(core::ptr::addr_of!(
                PANIC_RECORDS[target_core].message_len
            ))
            .min(PANIC_MESSAGE_CAP);
            let file_len =
                core::ptr::read_volatile(core::ptr::addr_of!(PANIC_RECORDS[target_core].file_len))
                    .min(PANIC_FILE_CAP);
            let line =
                core::ptr::read_volatile(core::ptr::addr_of!(PANIC_RECORDS[target_core].line));

            let mut report = CrashReport {
                core_id,
                message: heapless::String::new(),
                file: heapless::String::new(),
                line,
            };

            // Copy data out of volatile buffer
            for i in 0..msg_len {
                let byte = core::ptr::read_volatile(
                    (core::ptr::addr_of!(PANIC_RECORDS[target_core].message) as *const u8).add(i),
                );
                let _ = report.message.push(byte as char);
            }
            for i in 0..file_len {
                let byte = core::ptr::read_volatile(
                    (core::ptr::addr_of!(PANIC_RECORDS[target_core].file) as *const u8).add(i),
                );
                let _ = report.file.push(byte as char);
            }

            // Clear the signature so it doesn't repeat
            core::ptr::write_volatile(
                core::ptr::addr_of_mut!(PANIC_RECORDS[target_core].state_signature),
                0,
            );

            return Some(report);
        }
    }

    None
}

#[panic_handler]
fn panic(info: &PanicInfo) -> ! {
    let core_id = pac::SIO.cpuid().read();
    let idx = (core_id % 2) as usize;

    // Safety: The validation signature is written immediately. This ensures Core 0 detects
    // the crash state even if a recursive (double) fault occurs during formatting.
    unsafe {
        core::ptr::write_volatile(core::ptr::addr_of_mut!(PANIC_RECORDS[idx].core_id), core_id);
        core::ptr::write_volatile(core::ptr::addr_of_mut!(PANIC_RECORDS[idx].message_len), 0);
        core::ptr::write_volatile(core::ptr::addr_of_mut!(PANIC_RECORDS[idx].file_len), 0);
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(PANIC_RECORDS[idx].state_signature),
            STATE_SIGNATURE_PARTIAL,
        );

        // Signal other core via FIFO
        let sio = pac::SIO;
        while sio.fifo().st().read().vld() {
            let _ = sio.fifo().rd().read();
        }
        sio.fifo().wr().write_value(STATE_SIGNATURE_PARTIAL);
    }

    cortex_m::asm::dmb();

    error!("PANIC HANDLER REACHED on core {}", core_id);

    let msg_ptr = unsafe { core::ptr::addr_of_mut!(PANIC_RECORDS[idx].message) as *mut u8 };
    let msg_cap = PANIC_MESSAGE_CAP;

    let mut writer = VolatileBufWriter {
        buf_ptr: msg_ptr,
        cap: msg_cap,
        pos: 0,
    };
    let _ = core::fmt::write(&mut writer, format_args!("{}", info.message()));
    let msg_len = writer.pos;

    let file_ptr = unsafe { core::ptr::addr_of_mut!(PANIC_RECORDS[idx].file) as *mut u8 };
    let file_cap = PANIC_FILE_CAP;
    let mut file_writer = VolatileBufWriter {
        buf_ptr: file_ptr,
        cap: file_cap,
        pos: 0,
    };

    let mut line = 0;
    if let Some(location) = info.location() {
        use core::fmt::Write;
        let _ = file_writer.write_str(location.file());
        line = location.line();
    }
    let file_len = file_writer.pos;

    // Finally write the completion signature
    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(PANIC_RECORDS[idx].message_len),
            msg_len,
        );
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(PANIC_RECORDS[idx].file_len),
            file_len,
        );
        core::ptr::write_volatile(core::ptr::addr_of_mut!(PANIC_RECORDS[idx].line), line);
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(PANIC_RECORDS[idx].state_signature),
            STATE_SIGNATURE_FINAL,
        );
    }
    cortex_m::asm::dmb();

    // The details are preserved in the PANIC_RECORD in shared RAM for later retrieval.
    error!(
        "PANIC on core {}: at {}:{}",
        core_id,
        info.location().map(|l| l.file()).unwrap_or("?"),
        line
    );

    for _ in 0..10_000_000 {
        cortex_m::asm::nop();
    }

    loop {}
}

#[cortex_m_rt::exception]
unsafe fn HardFault(ef: &cortex_m_rt::ExceptionFrame) -> ! {
    let core_id = pac::SIO.cpuid().read();
    let idx = (core_id % 2) as usize;

    // Safety: The validation signature is written before any diagnostic processing.
    unsafe {
        core::ptr::write_volatile(core::ptr::addr_of_mut!(PANIC_RECORDS[idx].core_id), core_id);
        core::ptr::write_volatile(core::ptr::addr_of_mut!(PANIC_RECORDS[idx].message_len), 0);
        core::ptr::write_volatile(core::ptr::addr_of_mut!(PANIC_RECORDS[idx].file_len), 0);
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(PANIC_RECORDS[idx].state_signature),
            STATE_SIGNATURE_PARTIAL,
        );

        // Signal other core via FIFO
        let sio = pac::SIO;
        while sio.fifo().st().read().vld() {
            let _ = sio.fifo().rd().read();
        }
        sio.fifo().wr().write_value(STATE_SIGNATURE_PARTIAL);
    }
    cortex_m::asm::dmb();

    error!("HARD FAULT REACHED on core {}", core_id);

    let buf_ptr = unsafe { core::ptr::addr_of_mut!(PANIC_RECORDS[idx].message) as *mut u8 };
    let buf_cap = PANIC_MESSAGE_CAP;

    let mut writer = VolatileBufWriter {
        buf_ptr,
        cap: buf_cap,
        pos: 0,
    };

    use core::fmt::Write;
    let _ = write!(
        &mut writer,
        "HARD FAULT at PC={:#x} LR={:#x}",
        ef.pc(),
        ef.lr()
    );

    let msg_len = writer.pos;

    unsafe {
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(PANIC_RECORDS[idx].message_len),
            msg_len,
        );
        core::ptr::write_volatile(
            core::ptr::addr_of_mut!(PANIC_RECORDS[idx].state_signature),
            STATE_SIGNATURE_FINAL,
        );
    }
    cortex_m::asm::dmb();

    error!(
        "HARD FAULT on core {}: PC={:#x} LR={:#x}",
        core_id,
        ef.pc(),
        ef.lr()
    );

    for _ in 0..10_000_000 {
        cortex_m::asm::nop();
    }

    loop {}
}
