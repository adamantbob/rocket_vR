// src/imu/adxl375.rs
use embassy_time::{Duration, with_timeout};
use embedded_hal_async::i2c::I2c as AsyncI2c;
use rocket_core::IMUSensorError;
use rocket_core::info;

pub struct Adxl375<I2C> {
    i2c: I2C,
}

impl<I2C: AsyncI2c> Adxl375<I2C> {
    const ADDR: u8 = 0x53;
    const DEVID: u8 = 0x00;
    const BW_RATE: u8 = 0x2C;
    const POWER_CTL: u8 = 0x2D;
    const DATA_FORMAT: u8 = 0x31;
    const DATAX0: u8 = 0x32;
    const FIFO_CTL: u8 = 0x38;

    pub fn new(i2c_dev: I2C) -> Self {
        Self { i2c: i2c_dev }
    }

    pub async fn init(&mut self) -> Result<(), IMUSensorError> {
        // 1. Identity Check
        let mut id = [0u8; 1];
        self.i2c
            .write_read(Self::ADDR, &[Self::DEVID], &mut id)
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        if id[0] != 0xE5 {
            return Err(IMUSensorError::DeviceMissing);
        }

        // 2. Configure Format: +/- 200g, Full-Res
        // 0x0B = 0b00001011 (Full Res, +/- 200g)
        self.i2c
            .write(Self::ADDR, &[Self::DATA_FORMAT, 0x0B])
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        // 3. Configure Bandwidth/Rate: 800Hz ODR
        self.i2c
            .write(Self::ADDR, &[Self::BW_RATE, 0x0D])
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        // 4. FIFO Setup (Optional but recommended): Stream Mode
        // 0x80 = Stream mode (keeps latest 32 samples)
        self.i2c
            .write(Self::ADDR, &[Self::FIFO_CTL, 0x80])
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        // 5. Start Measurement
        self.i2c
            .write(Self::ADDR, &[Self::POWER_CTL, 0x08])
            .await
            .map_err(|_| IMUSensorError::BusError)?;

        info!("ADXL375: Verified and Armed (+/- 200g)");
        Ok(())
    }

    /// Reads raw data and converts to milli-Gs.
    /// Note: 49mg/LSB is the standard for ADXL375 Full-Res.
    pub async fn read(&mut self) -> Result<[i32; 3], IMUSensorError> {
        let mut buf = [0u8; 6];

        // Wrap the I2C transaction in a 5ms timeout.
        // At 400kHz I2C, 6 bytes should take < 200 microseconds.
        // 5ms is plenty of breathing room but fast enough to recover.
        let result = with_timeout(
            Duration::from_millis(5),
            self.i2c.write_read(Self::ADDR, &[Self::DATAX0], &mut buf),
        )
        .await;

        match result {
            Ok(Ok(_)) => {
                let x = i16::from_le_bytes([buf[0], buf[1]]) as i32;
                let y = i16::from_le_bytes([buf[2], buf[3]]) as i32;
                let z = i16::from_le_bytes([buf[4], buf[5]]) as i32;
                Ok([x * 49, y * 49, z * 49])
            }
            Ok(Err(_)) => Err(IMUSensorError::BusError), // Physical I2C error (NACK, etc.)
            Err(_) => {
                info!("ADXL375: I2C Timeout!");
                Err(IMUSensorError::Timeout) // Bus hung or device non-responsive
            }
        }
    }
}
