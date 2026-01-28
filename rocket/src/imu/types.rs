use embassy_time::Instant;

#[derive(Debug, Clone, Copy, PartialEq, defmt::Format)]
pub enum SensorError {
    /// The sensor is responding, but new data isn't ready yet (e.g., Mag at 100Hz).
    DataNotReady,
    /// The I2C bus failed (NACK, timeout, or arbitration lost).
    BusError,
    /// The sensor ID is wrong or it's not responding at all.
    DeviceMissing,
    /// The sensor read timed out.
    Timeout,
}

/// Represents a synchronized snapshot of all IMU sensors on the I2C bus.
///
/// All physical values use fixed-point arithmetic (milli-units) to avoid
/// floating-point overhead on the RP2350 and to maintain precision.
#[derive(Debug, Copy, Clone, defmt::Format)]
pub struct IMUData {
    /// Instant since system boot.
    /// Used for delta-time (dt) calculations in integration filters.
    pub timestamp_ms: Instant,

    /// High-Range Acceleration (ADXL375) in milli-Gs (mG).
    /// Range: +/- 200,000 mG. Resolution: 49 mG.
    /// Primary use: Motor thrust phase and impact detection.
    pub accel_high_g: [i32; 3],

    /// Low-Range Acceleration (LSM6DSOX) in milli-Gs (mG).
    /// Range: +/- 16,000 mG. Resolution: ~0.5 mG.
    /// Primary use: Tilt sensing, pre-launch orientation, and parachutes.
    pub accel_low_g: [i32; 3],

    /// Angular Velocity (LSM6DSOX) in milli-degrees per second (mdps).
    /// Range: +/- 2,000,000 mdps (2000 deg/s).
    /// Primary use: Spin rate monitoring and active stabilization.
    pub gyro: [i32; 3],

    /// Magnetic Field Strength (LIS3MDL) in milli-Gauss (mG).
    /// Range: +/- 4,000 mG.
    /// Primary use: Heading reference and gyro-drift compensation.
    pub mag: [i32; 3],
}

impl IMUData {
    pub const fn new() -> Self {
        Self {
            timestamp_ms: Instant::from_ticks(0),
            accel_high_g: [0; 3],
            accel_low_g: [0; 3],
            gyro: [0; 3],
            mag: [0; 3],
        }
    }
}

impl Default for IMUData {
    fn default() -> Self {
        Self::new()
    }
}

/// Tracks the communication integrity of the IMU sensors.
/// Updated at a lower frequency (e.g., 1Hz) to provide stability metrics.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub struct IMUHealth {
    /// Percentage of successful reads for the High-G sensor.
    pub adxl_health: u8,
    /// Percentage of successful reads for the Low-G/Gyro sensor.
    pub lsm_health: u8,
    /// Percentage of successful reads for the Magnetometer.
    pub mag_health: u8,
    /// Total cumulative I2C bus errors since boot.
    pub total_bus_errors: u32,
    /// Measured bus utilization percentage (0-100).
    pub bus_utilization: u8,
}

impl IMUHealth {
    pub const fn new() -> Self {
        Self {
            adxl_health: 0,
            lsm_health: 0,
            mag_health: 0,
            total_bus_errors: 0,
            bus_utilization: 0,
        }
    }
    pub const fn new_from_readings(
        adxl_health: u8,
        lsm_health: u8,
        mag_health: u8,
        total_bus_errors: u32,
        bus_utilization: u8,
    ) -> Self {
        Self {
            adxl_health,
            lsm_health,
            mag_health,
            total_bus_errors,
            bus_utilization,
        }
    }
}

impl Default for IMUHealth {
    fn default() -> Self {
        Self::new()
    }
}
