use crate::{Irqs, SDCard, SDCardResources, error, info, warn};
use core::fmt::Write;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::Spi;
use embassy_time::{Delay, Duration, Instant, Timer};

// Note: We are specifically sticking to embedded-sdmmc v0.8.0.
// v0.9.0 removes Send/Sync from core types (VolumeManager, File, etc.),
// which breaks compatibility with async tasks that need to be Send.
use embedded_sdmmc::{
    Mode, RawDirectory, RawFile, SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager,
};
use proc_macros::tracked_task;

use rocket_core::log::{LOG_CHANNEL, LogBuffer, LogEntry, MAX_LOG_LINE_LEN};

// Define this to match the internal storage of VolumeManager
const MAX_OPEN_FILES: usize = 4;
// 11 bytes is for "LOG_XXX.CSV"
const MAX_NAME_LEN: usize = 11;

pub struct DummyTime;
impl TimeSource for DummyTime {
    fn get_timestamp(&self) -> Timestamp {
        Timestamp {
            year_since_1970: 56, // 2026
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

#[tracked_task(SDCard)]
#[embassy_executor::task]
pub async fn sd_card_task(r: SDCardResources, _irqs: Irqs) -> ! {
    // 1. Setup SPI with DMA
    // We start at a low frequency (400kHz) for reliable initialization phase
    let mut config = embassy_rp::spi::Config::default();
    config.frequency = 400_000;

    let mut spi = Spi::new(
        r.spi, r.clk, r.mosi, r.miso, r.dma_tx, r.dma_rx, Irqs, config,
    );
    let mut cs = Output::new(r.cs, Level::High);

    // 2. Discovery Phase: Use a scoped block with references to keep 'spi' available for speed boost later
    {
        let spi_dev = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(&mut spi, &mut cs);

        let mut volume_mgr = VolumeManager::new(SdCard::new(spi_dev, Delay), DummyTime);

        if let Err(_) = wait_for_sd_card(&mut volume_mgr, None).await {
            error!("SD Card initialization failed persistently. Task exiting.");
            loop {
                Timer::after_millis(1000).await;
            }
        }
    }

    // 3. Boost SPI frequency for performance after discovery
    // spi.set_frequency(16_000_000);
    spi.set_frequency(2_000_000);

    // 4. Final Setup: Now we move ownership of 'spi' and 'cs' for the long-running task
    let spi_dev = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, cs);
    let mut volume_mgr = VolumeManager::new(SdCard::new(spi_dev, Delay), DummyTime);

    // 0.8.0 API: Open volume then root.
    let v_idx = VolumeIdx(0);
    let v_handle = volume_mgr
        .open_raw_volume(v_idx)
        .expect("Failed to open volume");
    let r_handle = volume_mgr
        .open_root_dir(v_handle)
        .expect("Failed to open root");

    let (mut file_handle, log_name_raw) =
        find_next_log_file::<_, DummyTime>(&mut volume_mgr, r_handle);
    let log_name = core::str::from_utf8(&log_name_raw)
        .unwrap()
        .trim_matches(char::from(0));

    // Ask the Enum to describe itself to the cursor
    let mut buffer = LogBuffer::<2048>::new();
    LogEntry::write_schema(&mut buffer).ok();
    // Write the header to the SD card
    volume_mgr
        .write(file_handle, &buffer.get_active_buffer())
        .ok();
    buffer.reset();

    let mut last_sync = Instant::now();
    let sync_interval = Duration::from_secs(2); // Sync every 2 seconds

    loop {
        let entry = LOG_CHANNEL.receive().await;
        let _ = buffer.write_entry(&entry);

        while let Ok(next) = LOG_CHANNEL.try_receive() {
            if buffer.space_remaining() < MAX_LOG_LINE_LEN {
                // Buffer full! Write it, but don't stop the 'while' loop yet
                let _ = volume_mgr.write(file_handle, &buffer.get_active_buffer());
                buffer.reset();
            }
            let _ = buffer.write_entry(&next);
        }

        // Final flush of the remaining partial buffer
        if buffer.space_remaining() > 0 {
            if let Err(e) = volume_mgr.write(file_handle, &buffer.get_active_buffer()) {
                info!(
                    "SD Error during write: {:?}. Attempting recovery...",
                    defmt::Debug2Format(&e)
                );

                // --- RECOVERY BLOCK ---
                // 1. Give the hardware a moment to settle
                Timer::after_millis(100).await;

                // 2. Re-run your wait/init logic
                // Note: You might need to move this into a helper function
                if let Ok(_) = wait_for_sd_card(&mut volume_mgr, Some(3)).await {
                    if let Ok(h) = volume_mgr.open_file_in_dir(
                        r_handle,
                        log_name,
                        Mode::ReadWriteCreateOrAppend,
                    ) {
                        file_handle = h;
                        let _ = volume_mgr.file_seek_from_end(file_handle, 0);
                        info!("SD Recovery successful!");
                    }
                }
                // ----------------------
            }
            buffer.reset();
        }

        // After the while let Ok(next) loop finishes and you've flushed the cache:
        if last_sync.elapsed() >= sync_interval {
            // 1. Close the file to update the FAT32 Directory Entry (Size/Timestamp)
            if let Err(_) = volume_mgr.close_file(file_handle) {
                // If close fails, we don't even try to re-open here,
                // the next write attempt will trigger the recovery block above.
            } else {
                if let Ok(h) =
                    volume_mgr.open_file_in_dir(r_handle, log_name, Mode::ReadWriteCreateOrAppend)
                {
                    file_handle = h;
                    let _ = volume_mgr.file_seek_from_end(file_handle, 0);
                }
            }
            last_sync = Instant::now();
        }
    }
}

/// Attempts to initialize the SD card.
/// Returns Ok(()) if successful, or Err if it fails after multiple attempts.
async fn wait_for_sd_card<SPI, T>(
    mgr: &mut VolumeManager<SdCard<SPI, Delay>, T, MAX_OPEN_FILES>,
    retries: Option<u32>,
) -> Result<(), ()>
// You can use a specific error type here if preferred
where
    SPI: embedded_hal_1::spi::SpiDevice,
    T: TimeSource,
{
    let mut attempts = 0;
    loop {
        attempts += 1;
        // Attempt hardware init by requesting the card size.
        // In embedded-sdmmc 0.8.0, num_bytes() triggers initialization.
        match mgr.device().num_bytes() {
            Ok(num_bytes) if num_bytes > 0 => {
                let size_mb = num_bytes / 1_048_576;
                info!("SD Card ready: {} MiB", size_mb as u32);
                return Ok(());
            }
            Ok(_) => {
                info!("SD found, but reported 0 bytes (attempt {})", attempts);
            }
            Err(e) => {
                info!(
                    "Waiting for SD... attempt {} ({:?})",
                    attempts,
                    defmt::Debug2Format(&e)
                );
            }
        }

        if let Some(limit) = retries {
            if attempts >= limit {
                return Err(());
            }
        }

        embassy_time::Timer::after(Duration::from_millis(500)).await;
    }
}

/// Finds the next LOG_xxx.CSV. Returns the file and the name used.
fn find_next_log_file<SPI, T>(
    mgr: &mut VolumeManager<SdCard<SPI, Delay>, DummyTime, MAX_OPEN_FILES>,
    root: RawDirectory,
) -> (RawFile, [u8; MAX_NAME_LEN])
where
    SPI: embedded_hal_1::spi::SpiDevice,
    T: TimeSource,
{
    let mut file_num = 0;

    // Create a temporary LogBuffer on the stack for the filename
    // 11 bytes is enough for "LOG_XXX.CSV"
    let mut name_buffer = LogBuffer::<MAX_NAME_LEN>::new();

    loop {
        // Use the write! macro (which calls your Write implementation)
        let _ = write!(name_buffer, "LOG_{:03}.CSV", file_num);

        // Extract the slice and convert to a &str for the SD library
        let name_str = core::str::from_utf8(name_buffer.get_active_buffer()).unwrap();

        match mgr.open_file_in_dir(root, name_str, Mode::ReadOnly) {
            Ok(file) => {
                mgr.close_file(file).ok();
                file_num += 1;
                name_buffer.reset();
            }
            Err(_) => {
                // We found a name that doesn't exist yet! Create it.
                let file = mgr
                    .open_file_in_dir(root, name_str, Mode::ReadWriteCreateOrAppend)
                    .expect("Failed to create log file");

                // Return the file and the raw buffer
                // (We have to copy the buffer out because the LogBuffer object will drop)
                let mut final_name = [0u8; MAX_NAME_LEN];
                final_name.copy_from_slice(name_buffer.get_active_buffer());
                return (file, final_name);
            }
        }
        if file_num > 100 {
            warn!("Almost too many log files on SD card");
        }

        if file_num > 999 {
            error!("Too many log files on SD card");
        }
    }
}
