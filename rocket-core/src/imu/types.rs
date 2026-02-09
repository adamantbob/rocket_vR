use crate::log::{LogBuffer, Loggable};
use crate::types::FlightTicks;
use core::fmt::Write;
use embassy_time::Instant;
use proc_macros::TelemetryPayload;

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize, TelemetryPayload)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IMUData {
    /// Tickstamp of when the data was captured.
    pub tickstamp: FlightTicks,
    pub accel_low_g: [i32; 3],
    pub accel_high_g: [i32; 3],
    pub gyro: [i32; 3],
    pub mag: [i32; 3],
}

impl IMUData {
    pub const fn new() -> Self {
        Self {
            tickstamp: 0,
            accel_low_g: [0; 3],
            accel_high_g: [0; 3],
            gyro: [0; 3],
            mag: [0; 3],
        }
    }
}

impl Loggable for IMUData {
    const TAG: &'static str = "I";
    fn format_payload<const SIZE: usize>(&self, cursor: &mut LogBuffer<SIZE>) -> core::fmt::Result {
        write!(
            cursor,
            "{},{},{},{},{},{},{},{},{},{},{},{}",
            self.accel_low_g[0],
            self.accel_low_g[1],
            self.accel_low_g[2],
            self.accel_high_g[0],
            self.accel_high_g[1],
            self.accel_high_g[2],
            self.gyro[0],
            self.gyro[1],
            self.gyro[2],
            self.mag[0],
            self.mag[1],
            self.mag[2]
        )
    }
}

#[derive(Clone, Copy, Debug, Default, TelemetryPayload)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IMUHealth {
    /// Tickstamp of when the health was captured.
    pub tickstamp: FlightTicks,
    pub adxl_error: u8,
    pub lsm_error: u8,
    pub mag_error: u8,
    pub total_error: u32,
    pub bus_utilization: u8,
}

impl IMUHealth {
    pub const fn new() -> Self {
        Self {
            tickstamp: 0,
            adxl_error: 0,
            lsm_error: 0,
            mag_error: 0,
            total_error: 0,
            bus_utilization: 0,
        }
    }

    pub fn new_from_readings(adxl: u8, lsm: u8, mag: u8, total: u32, util: u8) -> Self {
        Self {
            tickstamp: Instant::now().as_ticks() as u64,
            adxl_error: adxl,
            lsm_error: lsm,
            mag_error: mag,
            total_error: total,
            bus_utilization: util,
        }
    }
}

impl Loggable for IMUHealth {
    const TAG: &'static str = "IH";
    fn format_payload<const SIZE: usize>(&self, cursor: &mut LogBuffer<SIZE>) -> core::fmt::Result {
        write!(
            cursor,
            "{},{},{},{},{}",
            self.adxl_error, self.lsm_error, self.mag_error, self.total_error, self.bus_utilization
        )
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum IMUSensorError {
    BusError,
    InvalidData,
    DataNotReady,
    DeviceMissing,
    Timeout,
}
