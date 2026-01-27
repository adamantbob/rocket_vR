use crate::state_machine::SENSOR_DATA;
use crate::{IMU, IMUResources, Irqs, info};
use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::gpio::{Level, Output, Pin};
use embassy_rp::i2c::{Async, I2c};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};
use proc_macros::tracked_task;
use static_cell::StaticCell;

// Sensors
mod adxl375;
mod lis3mdl;
mod lism6dsox;
pub mod types;
use crate::imu::types::*;

pub struct ImuManager;

#[tracked_task(IMU)]
#[embassy_executor::task]
pub async fn imu_task(r: IMUResources, irqs: Irqs) -> ! {
    // 1. Initialize the physical I2C peripheral
    let mut config = embassy_rp::i2c::Config::default();
    config.frequency = 400_000; // Fast mode for 100Hz+ sampling

    ImuManager::recover_bus(9, 8).await;
    let mut i2c_bus = I2c::new_async(r.i2c, r.scl, r.sda, irqs, config);

    ImuManager::verify_bus(&mut i2c_bus).await;

    // 2. Wrap the bus in a Mutex to allow sharing
    static I2C_BUS: StaticCell<
        Mutex<NoopRawMutex, I2c<'static, embassy_rp::peripherals::I2C0, Async>>,
    > = StaticCell::new();
    let i2c_mutex = Mutex::new(i2c_bus);
    let i2c_mutex_ref = I2C_BUS.init(i2c_mutex);

    // 3. Create individual "handles" for each sensor
    let lsm_dev = I2cDevice::new(i2c_mutex_ref);
    let mut lsm = lism6dsox::Lsm6dsox::new(lsm_dev);
    let adxl_dev = I2cDevice::new(i2c_mutex_ref);
    let mut adxl = adxl375::Adxl375::new(adxl_dev);
    let mag_dev = I2cDevice::new(i2c_mutex_ref);
    let mut mag = lis3mdl::Lis3mdl::new(mag_dev);

    // 4. Initialize Sensor Drivers
    if let Err(e) = adxl.init().await {
        info!("ADXL375 Init Failed: {:?}", e);
    }
    if let Err(e) = mag.init().await {
        info!("LIS3MDL Init Failed: {:?}", e);
    }
    if let Err(e) = lsm.init().await {
        info!("LSM6DSOX Init Failed: {:?}", e);
    }

    let mut ticker = Ticker::every(Duration::from_hz(100));
    let mut test_counter: u32 = 0;

    // Track "Health" metrics
    let mut adxl_error_level = 0u8;
    let mut lsm_error_level = 0u8;
    let mut mag_error_level = 0u8;

    loop {
        let mut imu_data = IMUData::new();
        let start_bus = Instant::now();
        test_counter += 1;

        // 1. Test ADXL375
        if let Ok(adxl_data) = adxl.read().await {
            imu_data.accel_high_g = adxl_data;
            // Slowly clear the error counter on success
            adxl_error_level = adxl_error_level.saturating_sub(1);
        } else {
            // Increase the error level on failure
            // Increase by 5 to make it sensitive
            adxl_error_level = adxl_error_level.saturating_add(5);
        }

        // 2. Test LSM6DSOX
        if let Ok((lsm_data, gyro_data)) = lsm.read().await {
            imu_data.accel_low_g = lsm_data;
            imu_data.gyro = gyro_data;
            // Slowly clear the error counter on success
            lsm_error_level = lsm_error_level.saturating_sub(1);
        } else {
            // Increase the error level on failure
            // Increase by 5 to make it sensitive
            lsm_error_level = lsm_error_level.saturating_add(5);
        }

        // 3. Test Magnetometer (Uses the DRDY check we wrote)
        match mag.read().await {
            Ok(data) => {
                imu_data.mag = data;
                mag_error_level = mag_error_level.saturating_sub(1);
            }
            Err(SensorError::DataNotReady) => {
                // Magnetometer data rate is 80Hz.
                // It won't be ready 20% of the time.
                // Do nothing. Don't punish the health score.
            }
            Err(_) => {
                // This is a real I2C NACK or Timeout.
                mag_error_level = mag_error_level.saturating_add(5); // Weight real errors more heavily
            }
        }

        let end_bus = Instant::now();
        let bus_time = end_bus - start_bus;
        let utilization: u8 = ((bus_time.as_micros() * 100) / 10_000) as u8;

        // Every 1 second (100 ticks @ 100Hz), print the "Heartbeat"
        if test_counter >= 100 {
            info!(
                "IMU Error Level: ADXL: {} | LSM: {} | MAG: {}",
                adxl_error_level, lsm_error_level, mag_error_level
            );
            info!("Bus Utilization: {}%", utilization);
            let total_error_level: u32 =
                adxl_error_level as u32 + lsm_error_level as u32 + mag_error_level as u32;
            let imu_error_level = IMUHealth::new_from_readings(
                adxl_error_level,
                lsm_error_level,
                mag_error_level,
                total_error_level,
                utilization,
            );
            SENSOR_DATA.imu_health.update(imu_error_level);

            // Brute-force print the last valid readings
            ImuManager::log_imu_row("HI-G ", imu_data.accel_high_g);
            ImuManager::log_imu_row("LOW-G", imu_data.accel_low_g);
            ImuManager::log_imu_row("GYRO ", imu_data.gyro);
            ImuManager::log_imu_row("MAG  ", imu_data.mag);

            // Reset metrics for the next second
            test_counter = 0;
        }

        SENSOR_DATA.imu.update(imu_data);
        ticker.next().await;
    }
}

impl ImuManager {
    pub async fn verify_bus(i2c: &mut I2c<'_, embassy_rp::peripherals::I2C0, Async>) {
        info!("Starting I2C Sensor Discovery on I2C0...");

        // 1. Check LSM6DSOX (Addr: 0x6A or 0x6B)
        // Register 0x0F should return 0x6C
        if let Ok(id) = Self::read_reg(i2c, 0x6A, 0x0F).await {
            info!("LSM6DSOX found! ID: 0x{:02x} (Expected 0x6c)", id);
        } else {
            info!("LSM6DSOX not found at 0x6A.");
        }

        // 2. Check LIS3MDL (Addr: 0x1C or 0x1E)
        // Register 0x0F should return 0x3D
        if let Ok(id) = Self::read_reg(i2c, 0x1C, 0x0F).await {
            info!("LIS3MDL found! ID: 0x{:02x} (Expected 0x3d)", id);
        } else {
            info!("LIS3MDL not found at 0x1C.");
        }

        // 3. Check ADXL375 (Addr: 0x53 or 0x1D)
        // Register 0x00 should return 0xE5
        if let Ok(id) = Self::read_reg(i2c, 0x53, 0x00).await {
            info!("ADXL375 found! ID: 0x{:02x} (Expected 0xe5)", id);
        } else {
            info!("ADXL375 not found at 0x53.");
        }
    }

    async fn read_reg(
        i2c: &mut I2c<'_, embassy_rp::peripherals::I2C0, Async>,
        addr: u8,
        reg: u8,
    ) -> Result<u8, &'static str> {
        let mut read_buf = [0u8; 1];

        // Wrap the I2C operation in a 50ms timeout
        match with_timeout(
            Duration::from_millis(50),
            i2c.write_read_async(addr, [reg], &mut read_buf),
        )
        .await
        {
            Ok(Ok(_)) => Ok(read_buf[0]),
            Ok(Err(_)) => Err("I2C Error (NACK/Bus)"),
            Err(_) => Err("I2C Timeout (Bus Hung)"),
        }
    }

    pub async fn recover_bus(scl_pin: u8, sda_pin: u8) {
        // We use "any_pin" to temporarily take control of the pins
        // This assumes you pass the pin numbers (e.g., 9 for SCL, 8 for SDA)
        let mut scl = Output::new(
            unsafe { embassy_rp::gpio::AnyPin::steal(scl_pin) },
            Level::High,
        );
        let sda = embassy_rp::gpio::Input::new(
            unsafe { embassy_rp::gpio::AnyPin::steal(sda_pin) },
            embassy_rp::gpio::Pull::Up,
        );

        // If SDA is already high, the bus is fine
        if sda.is_high() {
            return;
        }

        info!("I2C Bus hung low! Attempting recovery...");

        // Clock the SCL line 9 times
        for _ in 0..9 {
            scl.set_low();
            embassy_time::Timer::after_micros(5).await;
            scl.set_high();
            embassy_time::Timer::after_micros(5).await;

            // If the sensor releases the line, we can stop
            if sda.is_high() {
                info!("Bus recovered.");
                break;
            }
        }
    }
    fn log_imu_row(label: &str, m_units: [i32; 3]) {
        // We process each axis into (Sign, Whole, Milli)
        // To avoid the {:03} hint, we break the milli-part into 3 separate digits

        let mut parts = [(0i32, 0i32, 0i32, 0i32, 0i32); 3];

        for i in 0..3 {
            let val = m_units[i];
            let sign = if val < 0 { 1 } else { 0 }; // 1 for '-', 0 for ' '
            let abs_val = val.abs();
            let whole = abs_val / 1000;
            let rem = abs_val % 1000;

            let d1 = rem / 100; // Hundreds place
            let d2 = (rem / 10) % 10; // Tens place
            let d3 = rem % 10; // Units place

            parts[i] = (sign, whole, d1, d2, d3);
        }

        // Now we print using only the simplest {} display trait
        // Each axis gets: [SignChar][Whole].[D1][D2][D3]
        info!(
            "{} | X:{}{}.{}{}{} | Y:{}{}.{}{}{} | Z:{}{}.{}{}{}",
            label,
            if parts[0].0 == 1 { "-" } else { " " },
            parts[0].1,
            parts[0].2,
            parts[0].3,
            parts[0].4,
            if parts[1].0 == 1 { "-" } else { " " },
            parts[1].1,
            parts[1].2,
            parts[1].3,
            parts[1].4,
            if parts[2].0 == 1 { "-" } else { " " },
            parts[2].1,
            parts[2].2,
            parts[2].3,
            parts[2].4
        );
    }
}
