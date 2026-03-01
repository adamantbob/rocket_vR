use embedded_hal_async::i2c::I2c as AsyncI2c;
use rocket_core::IMUSensorError;
use rocket_core::info;

pub struct Lsm6dsox<I2C> {
    i2c: I2C,
}

impl<I2C: AsyncI2c> Lsm6dsox<I2C> {
    // Default I2C address is 0x6B (can be 0x6A depending on ADDR pin)
    const ADDR: u8 = 0x6A;

    pub fn new(i2c_dev: I2C) -> Self {
        Self { i2c: i2c_dev }
    }

    pub async fn init(&mut self) -> Result<(), IMUSensorError> {
        // 1. WHO_AM_I (0x0F) -> Expected 0x6C
        let mut id = [0u8; 1];
        self.i2c
            .write_read(Self::ADDR, &[0x0F], &mut id)
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        if id[0] != 0x6C {
            return Err(IMUSensorError::DeviceMissing);
        }

        // 2. CTRL1_XL (0x10): Set Accel to 104Hz, +/- 16g range
        // 0x44 = 0100 (104Hz) 01 (16g) 00 (Filter)
        self.i2c
            .write(Self::ADDR, &[0x10, 0x44])
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        // 3. CTRL2_G (0x11): Set Gyro to 104Hz, 2000 dps range
        // 0x4C = 0100 (104Hz) 1100 (2000dps)
        self.i2c
            .write(Self::ADDR, &[0x11, 0x4C])
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        info!("LSM6DSOX: Online (+/-16g, 2000dps)");
        Ok(())
    }

    /// Reads Accel (mG) and Gyro (mdps) in one burst to ensure sync
    pub async fn read(&mut self) -> Result<([i32; 3], [i32; 3]), IMUSensorError> {
        let mut buf = [0u8; 12];
        // Read 12 bytes starting from 0x22 (OUTX_L_G) - Gyro first then Accel
        self.i2c
            .write_read(Self::ADDR, &[0x22], &mut buf)
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        // --- Gyro Conversion (mdps) ---
        // Sensitivity @ 2000dps is roughly 70 mdps/LSB
        let gx = (i16::from_le_bytes([buf[0], buf[1]]) as i32) * 70;
        let gy = (i16::from_le_bytes([buf[2], buf[3]]) as i32) * 70;
        let gz = (i16::from_le_bytes([buf[4], buf[5]]) as i32) * 70;

        // Accelerometer Data
        // Sensitivity @ +/-16g is roughly 0.488 mg/LSB.
        // Multiply by 488 and divide by 1000 to keep it integer (mG).
        let ax = (i16::from_le_bytes([buf[6], buf[7]]) as i32 * 488) / 1000;
        let ay = (i16::from_le_bytes([buf[8], buf[9]]) as i32 * 488) / 1000;
        let az = (i16::from_le_bytes([buf[10], buf[11]]) as i32 * 488) / 1000;

        Ok(([ax, ay, az], [gx, gy, gz]))
    }
}
