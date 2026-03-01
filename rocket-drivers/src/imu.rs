use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_rp::i2c::{Async, I2c};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::{Duration, Instant, Ticker, with_timeout};

use rocket_core::blackboard::{SENSOR_DATA, SYSTEM_HEALTH};
use rocket_core::info;
use rocket_core::log::{LOG_CHANNEL, LogEntry};

#[cfg(feature = "debug-imu")]
use rocket_core::log_triad_fixed_width;

// Sensors
mod adxl375;
mod lis3mdl;
mod lism6dsox;
use rocket_core::{IMUData, IMUHealth, IMUSensorError};

/// Runs the IMU (Inertial Measurement Unit)
/// To use this define a task in main.rs that calls this function with the I2Cx peripheral.
/// For example:
/// ```rust
/// #[embassy_executor::task]
/// pub async fn imu_task(
///     i2c: embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>,
/// ) -> ! {
///     imu_task_driver(i2c).await
/// }
/// ```
pub async fn imu_task_driver<I2C>(mut i2c_bus: I2c<'static, I2C, embassy_rp::i2c::Async>) -> !
where
    I2C: embassy_rp::i2c::Instance,
{
    verify_bus(&mut i2c_bus).await;

    // 2. Wrap the bus in a stack Mutex to allow sharing
    let i2c_mutex = Mutex::<NoopRawMutex, _>::new(i2c_bus);

    // 3. Create individual "handles" for each sensor
    let lsm_dev = I2cDevice::new(&i2c_mutex);
    let mut lsm = lism6dsox::Lsm6dsox::new(lsm_dev);
    let adxl_dev = I2cDevice::new(&i2c_mutex);
    let mut adxl = adxl375::Adxl375::new(adxl_dev);
    let mag_dev = I2cDevice::new(&i2c_mutex);
    let mut mag = lis3mdl::Lis3mdl::new(mag_dev);

    // Track "Health" metrics
    let mut adxl_error_level = 0u8;
    let mut lsm_error_level = 0u8;
    let mut mag_error_level = 0u8;

    // 4. Initialize Sensor Drivers
    match with_timeout(Duration::from_millis(50), adxl.init()).await {
        Ok(Ok(_)) => {
            info!("ADXL375 Online");
        }
        Ok(Err(e)) => {
            info!("ADXL375 Hardware Error: {:?}", e);
            adxl_error_level = 255;
        }
        Err(_) => {
            info!("ADXL375 Initialization Timed Out - check wiring/power");
            adxl_error_level = 255;
        }
    }
    match with_timeout(Duration::from_millis(50), mag.init()).await {
        Ok(Ok(_)) => {
            info!("LIS3MDL Online");
        }
        Ok(Err(e)) => {
            info!("LIS3MDL Hardware Error: {:?}", e);
            mag_error_level = 255;
        }
        Err(_) => {
            info!("LIS3MDL Initialization Timed Out - check wiring/power");
            mag_error_level = 255;
        }
    }
    match with_timeout(Duration::from_millis(50), lsm.init()).await {
        Ok(Ok(_)) => {
            info!("LSM6DSOX Online");
        }
        Ok(Err(e)) => {
            info!("LSM6DSOX Hardware Error: {:?}", e);
            lsm_error_level = 255;
        }
        Err(_) => {
            info!("LSM6DSOX Initialization Timed Out - check wiring/power");
            lsm_error_level = 255;
        }
    }
    if let Err(e) = mag.init().await {
        info!("LIS3MDL Init Failed: {:?}", e);
    }
    if let Err(e) = lsm.init().await {
        info!("LSM6DSOX Init Failed: {:?}", e);
    }

    let mut ticker = Ticker::every(Duration::from_hz(100));
    let mut test_counter: u32 = 0;

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
            Err(IMUSensorError::DataNotReady) => {
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
            #[cfg(feature = "debug-imu")]
            {
                info!(
                    "IMU Error Level: ADXL: {} | LSM: {} | MAG: {}",
                    adxl_error_level, lsm_error_level, mag_error_level
                );
                info!("Bus Utilization: {}%", utilization);
                // Print the last valid readings
                log_triad_fixed_width!("HI-G ", imu_data.accel_high_g);
                log_triad_fixed_width!("LOW-G", imu_data.accel_low_g);
                log_triad_fixed_width!("GYRO ", imu_data.gyro);
                log_triad_fixed_width!("MAG  ", imu_data.mag);
            }
            let total_error_level: u32 =
                adxl_error_level as u32 + lsm_error_level as u32 + mag_error_level as u32;
            let imu_error_level = IMUHealth::new_from_readings(
                adxl_error_level,
                lsm_error_level,
                mag_error_level,
                total_error_level,
                utilization,
            );
            SYSTEM_HEALTH.imu_health.update(imu_error_level);

            // Reset metrics for the next second
            test_counter = 0;
        }
        SENSOR_DATA.imu.update(imu_data);
        let _ = LOG_CHANNEL.try_send(LogEntry::Imu(imu_data));
        ticker.next().await;
    }
}

async fn verify_bus<I2C>(i2c: &mut I2c<'_, I2C, Async>)
where
    I2C: embassy_rp::i2c::Instance,
{
    info!("Starting I2C Sensor Discovery on I2C0...");

    // 1. Check LSM6DSOX (Addr: 0x6A or 0x6B)
    // Register 0x0F should return 0x6C
    if let Ok(id) = read_reg(i2c, 0x6A, 0x0F).await {
        info!("LSM6DSOX found! ID: 0x{:02x} (Expected 0x6c)", id);
    } else {
        info!("LSM6DSOX not found at 0x6A.");
    }

    // 2. Check LIS3MDL (Addr: 0x1C or 0x1E)
    // Register 0x0F should return 0x3D
    if let Ok(id) = read_reg(i2c, 0x1C, 0x0F).await {
        info!("LIS3MDL found! ID: 0x{:02x} (Expected 0x3d)", id);
    } else {
        info!("LIS3MDL not found at 0x1C.");
    }

    // 3. Check ADXL375 (Addr: 0x53 or 0x1D)
    // Register 0x00 should return 0xE5
    if let Ok(id) = read_reg(i2c, 0x53, 0x00).await {
        info!("ADXL375 found! ID: 0x{:02x} (Expected 0xe5)", id);
    } else {
        info!("ADXL375 not found at 0x53.");
    }
}

async fn read_reg<I2C>(i2c: &mut I2c<'_, I2C, Async>, addr: u8, reg: u8) -> Result<u8, &'static str>
where
    I2C: embassy_rp::i2c::Instance,
{
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
