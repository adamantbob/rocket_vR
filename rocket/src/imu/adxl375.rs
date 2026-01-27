// src/imu/adxl375.rs
use crate::imu::types::SensorError;
use crate::info;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::i2c::{Async, I2c as RpI2c};
use embassy_rp::peripherals::I2C0;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;

use embedded_hal_async::i2c::I2c as AsyncI2c;

type SharedI2c0 = I2cDevice<'static, NoopRawMutex, RpI2c<'static, I2C0, Async>>;

pub struct Adxl375 {
    i2c: SharedI2c0,
}

impl Adxl375 {
    const ADDR: u8 = 0x53;

    pub fn new(i2c_dev: SharedI2c0) -> Self {
        Self { i2c: i2c_dev }
    }

    pub async fn init(&mut self) -> Result<(), SensorError> {
        // Register 0x31 (DATA_FORMAT): 0x0B = Full-Res, +/- 200g
        AsyncI2c::write(&mut self.i2c, Self::ADDR, &[0x31, 0x0B])
            .await
            .map_err(|_| SensorError::BusError)?;

        // Register 0x2C (BW_RATE): 0x0D = 800Hz ODR
        AsyncI2c::write(&mut self.i2c, Self::ADDR, &[0x2C, 0x0D])
            .await
            .map_err(|_| SensorError::BusError)?;

        // Register 0x2D (POWER_CTL): 0x08 = Start measurement
        AsyncI2c::write(&mut self.i2c, Self::ADDR, &[0x2D, 0x08])
            .await
            .map_err(|_| SensorError::BusError)?;

        info!("ADXL375: Initialized (200g @ 800Hz)");
        Ok(())
    }

    /// Reads X, Y, and Z acceleration in milli-Gs (mG).
    /// Returns [x, y, z] as i32 for fixed-point stability.
    pub async fn read(&mut self) -> Result<[i32; 3], SensorError> {
        let mut buf = [0u8; 6];

        // Read 6 bytes starting from 0x32
        AsyncI2c::write_read(&mut self.i2c, Self::ADDR, &[0x32], &mut buf)
            .await
            .map_err(|_| SensorError::BusError)?;

        // Conversion: Raw LSBs * 49 mG/LSB
        // We cast to i32 immediately to prevent overflow during multiplication
        let x = (i16::from_le_bytes([buf[0], buf[1]]) as i32) * 49;
        let y = (i16::from_le_bytes([buf[2], buf[3]]) as i32) * 49;
        let z = (i16::from_le_bytes([buf[4], buf[5]]) as i32) * 49;

        Ok([x, y, z])
    }
}
