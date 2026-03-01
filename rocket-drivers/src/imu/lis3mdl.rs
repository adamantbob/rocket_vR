use embedded_hal_async::i2c::I2c as AsyncI2c;
use rocket_core::IMUSensorError;
use rocket_core::info;

pub struct Lis3mdl<I2C> {
    i2c: I2C,
}

impl<I2C: AsyncI2c> Lis3mdl<I2C> {
    const ADDR: u8 = 0x1C;

    pub fn new(i2c_dev: I2C) -> Self {
        Self { i2c: i2c_dev }
    }

    /// Initializes the Magnetometer. Returns IMUSensorError::DeviceMissing if ID mismatch.
    pub async fn init(&mut self) -> Result<(), IMUSensorError> {
        // 1. WHO_AM_I (0x0F) should be 0x3D
        let mut id = [0u8; 1];
        self.i2c
            .write_read(Self::ADDR, &[0x0F], &mut id)
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        if id[0] != 0x3D {
            info!("MAG: Wrong ID 0x{:x}", id[0]);
            return Err(IMUSensorError::DeviceMissing);
        }

        // 2. CTRL_REG1 (0x20): Ultra-high performance, 80Hz ODR
        self.i2c
            .write(Self::ADDR, &[0x20, 0x7C])
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        // 3. CTRL_REG3 (0x22): Continuous conversion mode
        self.i2c
            .write(Self::ADDR, &[0x22, 0x00])
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        info!("LIS3MDL: Online (Compass)");
        Ok(())
    }

    /// Polls the sensor for new data.
    /// Returns IMUSensorError::DataNotReady if polled faster than the 80Hz ODR.
    pub async fn read(&mut self) -> Result<[i32; 3], IMUSensorError> {
        let mut buf = [0u8; 6];

        // Ensure STATUS_REG (0x27) tells us ZYX data is ready (Bit 3)
        let mut status = [0u8; 1];
        self.i2c
            .write_read(Self::ADDR, &[0x27], &mut status)
            .await
            .map_err(|_| IMUSensorError::BusError)?;
        if status[0] & 0x08 == 0 {
            return Err(IMUSensorError::Timeout); // Not ready yet
        }

        // Read 6 bytes starting from OUT_X_L (0x28)
        self.i2c
            .write_read(Self::ADDR, &[0x28], &mut buf)
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        // Convert to mGauss (Assuming +/- 4 Gauss range default)
        // Sensitivity @ +/- 4 gauss is 6842 LSB/gauss
        // We want milli-gauss.
        let x = (i16::from_le_bytes([buf[0], buf[1]]) as i32 * 1000) / 6842;
        let y = (i16::from_le_bytes([buf[2], buf[3]]) as i32 * 1000) / 6842;
        let z = (i16::from_le_bytes([buf[4], buf[5]]) as i32 * 1000) / 6842;

        Ok([x, y, z])
    }
}
