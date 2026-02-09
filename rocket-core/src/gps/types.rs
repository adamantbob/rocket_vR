use crate::log::{LogBuffer, Loggable};
use crate::types::FlightTicks;
use core::fmt::Write;
use proc_macros::TelemetryPayload;

/// Telemetry data captured from the GPS module.
/// All fields use fixed-point integer arithmetic to ensure high-speed,
/// deterministic performance without floating-point rounding errors,
/// while maintaining high-precision floating point estimates for
/// filtering and telemetry purposes.
#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize, TelemetryPayload)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GPSData {
    /// Tickstamp of when the data was captured.
    pub tickstamp: FlightTicks,

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
    /// Altitude above ground level in meters (f32).
    pub alt_agl_m: f32,

    /// Vertical velocity in millimeters per second (i32).
    /// Scaled from the filtered float for use in integer flight logic.
    pub velocity_z_mms: i32,

    /// Raw filtered vertical velocity in meters per second (f32).
    /// High-precision estimate used for telemetry and secondary logic.
    pub velocity_z_filt: f32,
}

impl GPSData {
    pub const fn new() -> Self {
        Self {
            tickstamp: 0,
            utc_time_secs: 0,
            lat_microdegrees: 0,
            lon_microdegrees: 0,
            raw_alt_mm: 0,
            speed_mm_per_sec: 0,
            satellites: 0,
            fix_valid: false,
            pdop_x10: 999,
            alt_agl_m: 0.0,
            velocity_z_mms: 0,
            velocity_z_filt: 0.0,
        }
    }
}

impl Loggable for GPSData {
    const TAG: &'static str = "G";
    fn format_payload<const SIZE: usize>(&self, cursor: &mut LogBuffer<SIZE>) -> core::fmt::Result {
        write!(
            cursor,
            "{},{},{},{},{},{},{},{},{},{},{}",
            self.utc_time_secs,
            self.lat_microdegrees,
            self.lon_microdegrees,
            self.raw_alt_mm,
            self.speed_mm_per_sec,
            self.satellites,
            self.fix_valid,
            self.pdop_x10,
            self.alt_agl_m,
            self.velocity_z_mms,
            self.velocity_z_filt
        )
    }
}

/// Health metrics and status indicators for the GPS module.
#[derive(Clone, Copy, Debug, Default, TelemetryPayload)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GPSHealth {
    /// Tickstamp of when the health was captured.
    pub tickstamp: FlightTicks,
    pub last_fix_time_secs: u32,
    pub receive_timeout: u16,
    pub checksum_errors: u16,
    pub parse_errors: u16,
}

impl GPSHealth {
    pub const fn new() -> Self {
        Self {
            tickstamp: 0,
            last_fix_time_secs: 0,
            receive_timeout: 0,
            checksum_errors: 0,
            parse_errors: 0,
        }
    }
}

impl Loggable for GPSHealth {
    const TAG: &'static str = "GH";
    fn format_payload<const SIZE: usize>(&self, cursor: &mut LogBuffer<SIZE>) -> core::fmt::Result {
        write!(
            cursor,
            "{},{},{},{}",
            self.last_fix_time_secs, self.receive_timeout, self.checksum_errors, self.parse_errors
        )
    }
}

/// Errors that can occur during GPS data parsing and processing.
#[derive(Debug)]
pub enum GPSSensorError {
    InvalidChecksum,
    InvalidData,
}
