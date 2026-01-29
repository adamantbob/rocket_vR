// gps/types.rs
#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GPSData {
    pub utc_time_secs: u32,
    pub lat_microdegrees: i32,
    pub lon_microdegrees: i32,
    pub raw_alt_mm: i32,
    pub speed_mm_per_sec: i32,
    pub satellites: u8,
    pub fix_valid: bool,
    pub pdop_x10: u16,
    pub alt_agl_m: f32,
    pub velocity_z: f32,
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
            pdop_x10: 999,
            alt_agl_m: 0.0,
            velocity_z: 0.0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct GPSHealth {
    pub last_fix_time_secs: u32,
    pub receive_timeout: u16,
    pub checksum_errors: u16,
    pub parse_errors: u16,
}

impl GPSHealth {
    pub const fn new() -> Self {
        Self {
            last_fix_time_secs: 0,
            receive_timeout: 0,
            checksum_errors: 0,
            parse_errors: 0,
        }
    }
}

#[derive(Debug)]
pub enum GPSSensorError {
    InvalidChecksum,
    InvalidData,
}
