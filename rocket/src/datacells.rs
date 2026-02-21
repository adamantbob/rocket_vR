// datacells.rs
// This file holds the source for the "library" of data coming from different sensors
// Each cell represent one chunk of data that can be read together.

pub use rocket_core::datacells::DataCell;

pub use rocket_core::{FlightState, PersistentData};

pub enum ErrorLevel {
    NORMAL,
    WARNING,
    ERROR,
    FATAL,
}

// In PersistentData or datacells.rs
const BOOT_SIGNATURE_VALID: u32 = 0xDEADBEEF;

// Add a simple checksum to PersistentData
fn compute_checksum(ground_level: i32, state: FlightState) -> u32 {
    // XOR the raw bytes of ground_level and state discriminant together
    // Simple but effective for distinguishing garbage from real data
    (ground_level as u32)
        .wrapping_add(state as u32)
        .wrapping_add(0xA5A5A5A5)
}

#[unsafe(link_section = ".uninit")]
static mut PERSISTENT_STORAGE: PersistentData = PersistentData {
    ground_level: 0,
    state: FlightState::GroundIdle,
    checksum: 0,
};

#[unsafe(link_section = ".uninit")]
static mut BOOT_SIGNATURE: u32 = 0;

/// Attempts to recover flight data from RAM.
/// Returns Some(PersistentData) if the validation signature is valid.
pub fn try_recover_flight_data() -> Option<PersistentData> {
    unsafe {
        if BOOT_SIGNATURE == BOOT_SIGNATURE_VALID {
            let data = PERSISTENT_STORAGE;
            let expected = compute_checksum(data.ground_level, data.state);
            if data.checksum == expected {
                return Some(data);
            }
        }
        None
    }
}

/// Saves the current flight data to persistent RAM and sets the validation signature.
pub fn save_flight_data(ground_level: i32, state: FlightState) {
    unsafe {
        PERSISTENT_STORAGE = PersistentData {
            ground_level,
            state,
            checksum: compute_checksum(ground_level, state),
        };
        BOOT_SIGNATURE = BOOT_SIGNATURE_VALID;
    }
}

/// Invalidates the persistent storage. Use this for fresh launches.
pub fn _clear_persistence() {
    unsafe {
        BOOT_SIGNATURE = 0;
    }
}
