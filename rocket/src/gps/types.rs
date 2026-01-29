use embassy_time::Instant;

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum SensorError {
    /// The sensor data is invalid.
    InvalidData,
    /// The sensor data is invalid.
    InvalidChecksum,
}

/// Telemetry data captured from the GPS module.
/// All fields use fixed-point integer arithmetic to ensure high-speed,
/// deterministic performance without floating-point rounding errors.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct GPSData {
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
    pub raw_alt_mm: i32,

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

    // Kalman Filtered Outputs
    /// Altitude above ground level in meters
    pub alt_agl_m: f32,

    /// Vertical velocity in meters per second
    pub velocity_z: f32,

    /// Last update time
    pub last_update: Instant,
}

impl GPSData {
    pub const fn new() -> Self {
        Self {
            utc_time_secs: 0,
            lat_microdegrees: 0,
            lon_microdegrees: 0,
            raw_alt_mm: 0,
            speed_mm_per_sec: 0,
            satellites: 0,
            fix_valid: false,
            pdop_x10: 999, // High value means no precision yet
            alt_agl_m: 0.0,
            velocity_z: 0.0,
            last_update: Instant::from_ticks(0),
        }
    }
}

impl Default for GPSData {
    fn default() -> Self {
        Self::new()
    }
}

#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct GPSHealth {
    pub checksum_errors: u8,
    pub parse_errors: u8,
    pub last_fix_age_ms: Instant,
    pub receive_timeout: u8,
}

impl GPSHealth {
    pub const fn new() -> Self {
        Self {
            checksum_errors: 0,
            parse_errors: 0,
            last_fix_age_ms: Instant::from_ticks(0),
            receive_timeout: 0,
        }
    }
}

impl Default for GPSHealth {
    fn default() -> Self {
        Self::new()
    }
}
