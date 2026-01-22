// datacells.rs
// This file holds the source for the "library" of data coming from different sensors
// Each cell represent one chunk of data that can be read together.
use defmt::info;
use core::cell::Cell;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

/// Telemetry data captured from the GPS module.
/// All fields use fixed-point integer arithmetic to ensure high-speed, 
/// deterministic performance without floating-point rounding errors.
#[derive(Debug, Default, Clone, Copy, defmt::Format)]
pub struct GpsData {
    /// UTC Time as HHMMSS (e.g., 143005 = 14:30:05).
    /// Note: This is "Real World" time from the satellite atomic clocks.
    pub utc_time_secs: u32, 
    
    /// Latitude in microdegrees (Degrees * 1,000,000).
    /// Positive = North, Negative = South.
    /// Resolution: ~11.1cm at the equator.
    pub lat_microdegrees: i32, 
    
    /// Longitude in microdegrees (Degrees * 1,000,000).
    /// Positive = East, Negative = West.
    /// Resolution: ~11.1cm at the equator.
    pub lon_microdegrees: i32, 
    
    /// Altitude in millimeters (mm) above Mean Sea Level.
    /// Derived from the GPGGA MSL Altitude field.
    pub altitude_mm: i32, 
    
    /// Ground Speed in millimeters per second (mm/s).
    /// Converted from raw Knots (1 Knot = 514.44 mm/s).
    pub speed_mm_per_sec: i32,
    
    /// Number of satellites currently used for the fix.
    pub satellites: u8,
    
    /// GPS Fix Status. 
    /// true = Valid navigation data. false = Searching or Lost.
    pub fix_valid: bool,
    
    /// Position Dilution of Precision (DOP * 10).
    /// Represents the 3D accuracy based on satellite geometry.
    /// Values < 20 (2.0) are excellent. Values > 50 (5.0) are suspect.
    pub pdop_x10: u16,
}

impl GpsData {
    pub const fn new() -> Self {
        Self {
            utc_time_secs: 0,
            lat_microdegrees: 0,
            lon_microdegrees: 0,
            altitude_mm: 0,
            speed_mm_per_sec: 0,
            satellites: 0,
            fix_valid: false,
            pdop_x10: 999, // High value means no precision yet
        }
    }
}

/// A global static to hold the latest GPS data
pub static GPS_DATA: Mutex<CriticalSectionRawMutex, Cell<GpsData>> = 
    Mutex::new(Cell::new(GpsData::new()));
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum FlightState {
    Initializing = 0,    // GPS Calibration
    GroundIdle = 1,      // Waiting on the pad
    PoweredFlight = 2,   // Motor is burning, high acceleration
    Coasting = 3,        // Motor out, gaining altitude via momentum
    ApogeeReached = 4,   // The peak! Prepare for deployment
    Descent = 5,         // Coming down under parachute
    Landed = 6,          // Back on the ground
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