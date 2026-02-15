use crate::{Irqs, SDCard, SDCardResources, error, info, local_error, local_info, warn};
use core::fmt::Write;
use embassy_rp::gpio::{AnyPin, Level, Output, Pull};
use embassy_rp::spi::Spi;
use embassy_time::{Delay, Duration, Instant, Timer};

// Note: We are specifically sticking to embedded-sdmmc v0.8.0.
// v0.9.0 removes Send/Sync from core types (VolumeManager, File, etc.),
// which breaks compatibility with async tasks that need to be Send.
use embedded_sdmmc::{
    Mode, RawDirectory, RawFile, SdCard, TimeSource, Timestamp, VolumeIdx, VolumeManager,
};
use proc_macros::tracked_task;
use static_cell::StaticCell;

use embassy_futures::select::{Either, select};
use rocket_core::log::{LOG_CHANNEL, LogBuffer, LogEntry, MAX_LOG_LINE_LEN};

// Define this to match the internal storage of VolumeManager
const MAX_OPEN_FILES: usize = 4;
// 11 bytes is for "LOG_XXX.CSV"
const MAX_NAME_LEN: usize = 12;

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
    static BUFFER: StaticCell<LogBuffer<2048>> = StaticCell::new();
    let buffer = BUFFER.init(LogBuffer::new());
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

        if let Err(_) = wait_for_sd_card(&mut volume_mgr, Some(5)).await {
            error!("SD Card initialization failed persistently. Task exiting.");
            loop {
                Timer::after_millis(100).await;
            }
        }
    }

    // 3. Boost SPI frequency for performance after discovery
    spi.set_frequency(16_000_000);

    local_info!("SD Card initialized successfully.");
    // 4. Final Setup: Now we move ownership of 'spi' and 'cs' for the long-running task
    let spi_dev = embedded_hal_bus::spi::ExclusiveDevice::new_no_delay(spi, cs);
    let mut volume_mgr = VolumeManager::new(SdCard::new(spi_dev, Delay), DummyTime);

    local_info!("SD Card initialized successfully. 2");

    // Re-initialize after frequency boost!
    if let Err(_) = wait_for_sd_card(&mut volume_mgr, Some(5)).await {
        error!("SD Card re-initialization after boost failed. Task exiting.");
        loop {
            Timer::after_millis(100).await;
        }
    }

    // 0.8.0 API: Open volume then root.
    let v_idx = VolumeIdx(0);
    let v_handle = match volume_mgr.open_raw_volume(v_idx) {
        Ok(v) => v,
        Err(e) => {
            error!("Failed to open volume: {:?}", defmt::Debug2Format(&e));
            loop {
                Timer::after_millis(100).await;
            }
        }
    };
    let r_handle = match volume_mgr.open_root_dir(v_handle) {
        Ok(r) => r,
        Err(e) => {
            error!("Failed to open root: {:?}", defmt::Debug2Format(&e));
            loop {
                Timer::after_millis(100).await;
            }
        }
    };

    local_info!("SD Card initialized successfully. 3");
    let (mut file_handle, log_name_raw) =
        find_next_log_file::<_, DummyTime>(&mut volume_mgr, r_handle);
    let log_name = core::str::from_utf8(&log_name_raw)
        .unwrap()
        .trim_matches(char::from(0));

    local_info!("SD Card initialized successfully. 4");
    // Ask the Enum to describe itself to the cursor
    LogEntry::write_schema(buffer).ok();
    // Write the header to the SD card
    volume_mgr
        .write(file_handle, &buffer.get_active_buffer())
        .ok();
    buffer.reset();

    local_info!("SD Card initialized successfully. 5");

    let mut last_sync = Instant::now();
    let sync_interval = Duration::from_secs(10); // Sync every 10 seconds
    let mut last_flush = Instant::now();
    let flush_interval = Duration::from_secs(5); // Flush at least every 5s
    const MIN_FLUSH_SIZE: usize = 512; // One SD block

    let mut last_health_report = Instant::now();
    let health_interval = Duration::from_secs(10);
    let mut sd_ready = true;

    loop {
        // 1. Wait for data OR a flush timeout
        let entry_or_timeout = select(LOG_CHANNEL.receive(), Timer::after(flush_interval)).await;

        match entry_or_timeout {
            Either::First(entry) => {
                let _ = buffer.write_entry(&entry);

                // 2. Drain the channel as much as possible while we are awake
                while let Ok(next) = LOG_CHANNEL.try_receive() {
                    // If we are about to overflow the 2KB buffer, we MUST flush
                    if buffer.space_remaining() < MAX_LOG_LINE_LEN {
                        let _ = volume_mgr.write(file_handle, &buffer.get_active_buffer());
                        buffer.reset();
                        last_flush = Instant::now();
                    }
                    let _ = buffer.write_entry(&next);
                }
            }
            Either::Second(_) => {
                // Timeout reached, we will check if we should flush below
            }
        }

        // 3. Opportunistic Health Reporting:
        // If we woke up AND enough time has passed, append a health entry.
        // This avoids waking up the CPU JUST for health, and batches it with other data.
        if last_health_report.elapsed() >= health_interval {
            let health = rocket_core::log::LoggerHealth::capture(sd_ready);
            let _ = buffer.write_entry(&LogEntry::LoggerHealth(health));
            last_health_report = Instant::now();
        }

        // 4. Batch Flush Logic: Only write to SD if we have a full block OR the timeout passed
        if buffer.pos >= MIN_FLUSH_SIZE
            || (buffer.pos > 0 && last_flush.elapsed() >= flush_interval)
        {
            if let Err(e) = volume_mgr.write(file_handle, &buffer.get_active_buffer()) {
                info!(
                    "SD Error during write: {:?}. Attempting recovery...",
                    defmt::Debug2Format(&e)
                );
                sd_ready = false;

                // --- RECOVERY BLOCK ---
                Timer::after_millis(100).await;
                if let Ok(_) = wait_for_sd_card(&mut volume_mgr, Some(3)).await {
                    sd_ready = true;
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
            local_info!("SD write successful!");
            buffer.reset();
            last_flush = Instant::now();
        }

        // 5. Sync Logic: Every few seconds, close and reopen to update the file size in the FAT
        if last_sync.elapsed() >= sync_interval {
            if let Err(_) = volume_mgr.close_file(file_handle) {
                // Failure here will trigger recovery on the next write attempt
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
        local_info!("Checking for file: {:?}", name_buffer.get_active_buffer());

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
                local_info!("final name {:?}", final_name);
                return (file, final_name);
            }
        }
        if file_num > 100 {
            warn!("Almost too many log files on SD card");
        }

        if file_num > 999 {
            // Use the local error here, SD Card writes will fail.
            local_error!("Too many log files on SD card");
        }
    }
}
