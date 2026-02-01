// datacells.rs
// This file holds the source for the "library" of data coming from different sensors
// Each cell represent one chunk of data that can be read together.
use core::cell::Cell;
use defmt::info;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

pub use rocket_core::datacells::DataCell;

pub use rocket_core::{FlightState, PersistentData};

pub enum ErrorLevel {
    NORMAL,
    WARNING,
    ERROR,
    FATAL,
}

const BOOT_KEY_VALID: u32 = 0xDEADBEEF;

#[unsafe(link_section = ".uninit")]
static mut PERSISTENT_STORAGE: PersistentData = PersistentData {
    ground_level: 0,
    state: FlightState::GroundIdle,
};

#[unsafe(link_section = ".uninit")]
static mut MAGIC_BOOT_KEY: u32 = 0;

/// Attempts to recover flight data from RAM.
/// Returns Some(PersistentData) if the magic key is valid.
pub fn try_recover_flight_data() -> Option<PersistentData> {
    info!("Attempting to recover flight data...");
    info!("Address: 0x{:x}", &raw mut PERSISTENT_STORAGE as usize);
    unsafe {
        if MAGIC_BOOT_KEY == BOOT_KEY_VALID {
            Some(PERSISTENT_STORAGE)
        } else {
            None
        }
    }
}

/// Saves the current flight data to persistent RAM and sets the magic key.
pub fn save_flight_data(data: PersistentData) {
    unsafe {
        PERSISTENT_STORAGE = data;
        MAGIC_BOOT_KEY = BOOT_KEY_VALID;
    }
}

/// Invalidates the persistent storage. Use this for fresh launches.
pub fn clear_persistence() {
    unsafe {
        MAGIC_BOOT_KEY = 0;
    }
}
