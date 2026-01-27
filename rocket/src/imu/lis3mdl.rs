use crate::imu::types::SensorError;
use crate::info;
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::i2c::{Async, I2c as RpI2c};
use embassy_rp::peripherals::I2C0;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embedded_hal_async::i2c::I2c as AsyncI2c;

type SharedI2c0 = I2cDevice<'static, NoopRawMutex, RpI2c<'static, I2C0, Async>>;

pub struct Lis3mdl {
    i2c: SharedI2c0,
}

impl Lis3mdl {
    const ADDR: u8 = 0x1C;

    pub fn new(i2c_dev: SharedI2c0) -> Self {
        Self { i2c: i2c_dev }
    }

    /// Initializes the Magnetometer. Returns SensorError::DeviceMissing if ID mismatch.
    pub async fn init(&mut self) -> Result<(), SensorError> {
        // 1. WHO_AM_I (0x0F) should be 0x3D
        let mut id = [0u8; 1];
        AsyncI2c::write_read(&mut self.i2c, Self::ADDR, &[0x0F], &mut id)
            .await
            .map_err(|_| SensorError::BusError)?;

        if id[0] != 0x3D {
            info!("MAG: Wrong ID 0x{:x}", id[0]);
            return Err(SensorError::DeviceMissing);
        }

        // 2. CTRL_REG1 (0x20): Ultra-high performance, 80Hz ODR
        AsyncI2c::write(&mut self.i2c, Self::ADDR, &[0x20, 0x7C])
            .await
            .map_err(|_| SensorError::BusError)?;

        // 3. CTRL_REG3 (0x22): Continuous conversion mode
        AsyncI2c::write(&mut self.i2c, Self::ADDR, &[0x22, 0x00])
            .await
            .map_err(|_| SensorError::BusError)?;

        info!("LIS3MDL: Online (Compass)");
        Ok(())
    }

    /// Polls the sensor for new data.
    /// Returns SensorError::DataNotReady if polled faster than the 80Hz ODR.
    pub async fn read(&mut self) -> Result<[i32; 3], SensorError> {
        // 1. Check Status Register (0x27)
        let mut status = [0u8; 1];
        AsyncI2c::write_read(&mut self.i2c, Self::ADDR, &[0x27], &mut status)
            .await
            .map_err(|_| SensorError::BusError)?;

        // Bit 3 (0x08) is ZYXDA: Data ready
        if (status[0] & 0x08) == 0 {
            return Err(SensorError::DataNotReady);
        }

        // 2. Data is ready, proceed with burst read.
        // We add 0x80 to the address for automatic register incrementing.
        let mut data = [0u8; 6];
        AsyncI2c::write_read(&mut self.i2c, Self::ADDR, &[0x28 | 0x80], &mut data)
            .await
            .map_err(|_| SensorError::BusError)?;

        // Convert to mGauss (Assuming +/- 4 Gauss range default)
        let x = (i16::from_le_bytes([data[0], data[1]]) as i32 * 1000) / 6842;
        let y = (i16::from_le_bytes([data[2], data[3]]) as i32 * 1000) / 6842;
        let z = (i16::from_le_bytes([data[4], data[5]]) as i32 * 1000) / 6842;

        Ok([x, y, z])
    }
}
