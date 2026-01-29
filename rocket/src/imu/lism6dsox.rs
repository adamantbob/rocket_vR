use crate::info;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::i2c::{Async, I2c as RpI2c};
use embassy_rp::peripherals::I2C0;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embedded_hal_async::i2c::I2c as AsyncI2c;
use rocket_core::IMUSensorError;

type SharedI2c0 = I2cDevice<'static, NoopRawMutex, RpI2c<'static, I2C0, Async>>;

pub struct Lsm6dsox {
    i2c: SharedI2c0,
}

impl Lsm6dsox {
    // Default I2C address is 0x6B (can be 0x6A depending on ADDR pin)
    const ADDR: u8 = 0x6A;

    pub fn new(i2c_dev: SharedI2c0) -> Self {
        Self { i2c: i2c_dev }
    }

    pub async fn init(&mut self) -> Result<(), IMUSensorError> {
        // 1. WHO_AM_I (0x0F) -> Expected 0x6C
        let mut id = [0u8; 1];
        AsyncI2c::write_read(&mut self.i2c, Self::ADDR, &[0x0F], &mut id)
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        if id[0] != 0x6C {
            return Err(IMUSensorError::DeviceMissing);
        }

        // 2. CTRL1_XL (0x10): Set Accel to 104Hz, +/- 16g range
        // 0x44 = 0100 (104Hz) 01 (16g) 00 (Filter)
        AsyncI2c::write(&mut self.i2c, Self::ADDR, &[0x10, 0x44])
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        // 3. CTRL2_G (0x11): Set Gyro to 104Hz, 2000 dps range
        // 0x4C = 0100 (104Hz) 1100 (2000dps)
        AsyncI2c::write(&mut self.i2c, Self::ADDR, &[0x11, 0x4C])
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        info!("LSM6DSOX: Online (+/-16g, 2000dps)");
        Ok(())
    }

    /// Reads Accel (mG) and Gyro (mdps) in one burst to ensure sync
    pub async fn read(&mut self) -> Result<([i32; 3], [i32; 3]), IMUSensorError> {
        let mut data = [0u8; 12];

        // Burst read starting from 0x22 (Gyro X Low)
        AsyncI2c::write_read(&mut self.i2c, Self::ADDR, &[0x22], &mut data)
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        // --- Gyro Conversion (mdps) ---
        // Sensitivity @ 2000dps is roughly 70 mdps/LSB
        let gx = (i16::from_le_bytes([data[0], data[1]]) as i32) * 70;
        let gy = (i16::from_le_bytes([data[2], data[3]]) as i32) * 70;
        let gz = (i16::from_le_bytes([data[4], data[5]]) as i32) * 70;

        // --- Accel Conversion (mG) ---
        // Sensitivity @ 16g is 0.488 mG/LSB.
        // Logic: (val * 488) / 1000 to keep it integer based.
        let ax = (i16::from_le_bytes([data[6], data[7]]) as i32 * 488) / 1000;
        let ay = (i16::from_le_bytes([data[8], data[9]]) as i32 * 488) / 1000;
        let az = (i16::from_le_bytes([data[10], data[11]]) as i32 * 488) / 1000;

        Ok(([ax, ay, az], [gx, gy, gz]))
    }
}
