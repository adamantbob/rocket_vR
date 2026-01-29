// datacells.rs
// This file holds the source for the "library" of data coming from different sensors
// Each cell represent one chunk of data that can be read together.
use core::cell::Cell;
use defmt::info;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/// A generic thread-safe container for Blackboard-style data sharing.
pub struct DataCell<T: Copy + Default> {
    storage: Mutex<CriticalSectionRawMutex, Cell<T>>,
}

impl<T: Copy + Default> DataCell<T> {
    /// Create a new cell with default values.
    pub const fn new(init: T) -> Self {
        Self {
            storage: Mutex::new(Cell::new(init)),
        }
    }

    /// Update the data in the cell (The "Write").
    pub fn update(&self, data: T) {
        self.storage.lock(|cell| {
            cell.set(data);
        });
    }

    /// Fetch the latest data from the cell (The "Read").
    pub fn read(&self) -> T {
        self.storage.lock(|cell| cell.get())
    }
}

pub enum ErrorLevel {
    NORMAL,
    WARNING,
    ERROR,
    FATAL,
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum FlightState {
    Initializing = 0,  // GPS Calibration
    GroundIdle = 1,    // Waiting on the pad
    PoweredFlight = 2, // Motor is burning, high acceleration
    Coasting = 3,      // Motor out, gaining altitude via momentum
    ApogeeReached = 4, // The peak! Prepare for deployment
    Descent = 5,       // Coming down under parachute
    Landed = 6,        // Back on the ground
}

#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub struct PersistentData {
    pub ground_level: i32,
    pub state: FlightState,
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
