// imu/types.rs
#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IMUData {
    pub accel_low_g: [i32; 3],
    pub accel_high_g: [i32; 3],
    pub gyro: [i32; 3],
    pub mag: [i32; 3],
}

impl IMUData {
    pub const fn new() -> Self {
        Self {
            accel_low_g: [0; 3],
            accel_high_g: [0; 3],
            gyro: [0; 3],
            mag: [0; 3],
        }
    }
}

#[derive(Clone, Copy, Debug, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct IMUHealth {
    pub adxl_error: u8,
    pub lsm_error: u8,
    pub mag_error: u8,
    pub total_error: u32,
    pub bus_utilization: u8,
}

impl IMUHealth {
    pub const fn new() -> Self {
        Self {
            adxl_error: 0,
            lsm_error: 0,
            mag_error: 0,
            total_error: 0,
            bus_utilization: 0,
        }
    }

    pub const fn new_from_readings(adxl: u8, lsm: u8, mag: u8, total: u32, util: u8) -> Self {
        Self {
            adxl_error: adxl,
            lsm_error: lsm,
            mag_error: mag,
            total_error: total,
            bus_utilization: util,
        }
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
