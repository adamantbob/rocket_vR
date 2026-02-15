// datacells.rs
// This file holds the source for the "library" of data coming from different sensors
// Each cell represent one chunk of data that can be read together.
use core::cell::Cell;
use defmt::info;

pub use rocket_core::datacells::DataCell;

pub use rocket_core::{FlightState, PersistentData};

pub enum ErrorLevel {
    NORMAL,
    WARNING,
    ERROR,
    FATAL,
}

const BOOT_SIGNATURE_VALID: u32 = 0xDEADBEEF;

#[unsafe(link_section = ".uninit")]
static mut PERSISTENT_STORAGE: PersistentData = PersistentData {
    ground_level: 0,
    state: FlightState::GroundIdle,
};

#[unsafe(link_section = ".uninit")]
static mut BOOT_SIGNATURE: u32 = 0;

/// Attempts to recover flight data from RAM.
/// Returns Some(PersistentData) if the validation signature is valid.
pub fn try_recover_flight_data() -> Option<PersistentData> {
    info!("Attempting to recover flight data...");
    info!("Address: 0x{:x}", &raw mut PERSISTENT_STORAGE as usize);
    unsafe {
        if BOOT_SIGNATURE == BOOT_SIGNATURE_VALID {
            Some(PERSISTENT_STORAGE)
        } else {
            None
        }
    }
}

/// Saves the current flight data to persistent RAM and sets the validation signature.
pub fn save_flight_data(data: PersistentData) {
    unsafe {
        PERSISTENT_STORAGE = data;
        BOOT_SIGNATURE = BOOT_SIGNATURE_VALID;
    }
}

/// Invalidates the persistent storage. Use this for fresh launches.
pub fn clear_persistence() {
    unsafe {
        BOOT_SIGNATURE = 0;
    }
}
