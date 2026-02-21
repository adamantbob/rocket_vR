use crate::sd_card::{MAX_NAME_LEN, MAX_OPEN_FILES};
use crate::{error, info, local_info, local_warn};
use core::fmt::Write;
use embassy_time::{Delay, Duration, Instant, Timer};
use embedded_sdmmc::{Mode, RawDirectory, RawFile, SdCard, TimeSource, VolumeIdx, VolumeManager};
use rocket_core::log::{LOG_CHANNEL, LogBuffer, LogEntry, MAX_LOG_LINE_LEN};

/// Manages SD card logging state and operations.
///
/// Encapsulates volume management, file handles, buffering, and periodic maintenance tasks
/// such as flushing and syncing.
pub struct SdLogger<'a, SPI, T, const N: usize>
where
    SPI: embedded_hal_1::spi::SpiDevice,
    T: TimeSource,
{
    mgr: VolumeManager<SdCard<SPI, Delay>, T, MAX_OPEN_FILES>,
    root: RawDirectory,
    file: RawFile,
    log_name: [u8; MAX_NAME_LEN],
    buffer: &'a mut LogBuffer<N>,
    last_sync: Instant,
    last_flush: Instant,
    last_health_report: Instant,
    sd_ready: bool,
    sync_interval: Duration,
    flush_interval: Duration,
    health_interval: Duration,
}

impl<'a, SPI, T, const N: usize> SdLogger<'a, SPI, T, N>
where
    SPI: embedded_hal_1::spi::SpiDevice,
    T: TimeSource,
{
    /// Performs professional initialization of the SD logging system.
    ///
    /// This includes opening the operational volume, root directory, identifying the next
    /// sequential log file, and recording the protocol schema header.
    pub async fn initialize(
        mut mgr: VolumeManager<SdCard<SPI, Delay>, T, MAX_OPEN_FILES>,
        buffer: &'a mut LogBuffer<N>,
        flush_interval: Duration,
        sync_interval: Duration,
        health_interval: Duration,
    ) -> Result<Self, ()> {
        // Compile-time safety: Ensure buffer is large enough for professional operation.
        // 512 bytes is the minimum block size for SD cards and ensures we don't
        // flush too frequently.
        const {
            assert!(N >= 512, "SdLogger buffer size must be at least 512 bytes");
        }
        let v_idx = VolumeIdx(0);
        // Intentional Delay to let the bus settle
        Timer::after_millis(200).await;
        let v_handle = mgr.open_raw_volume(v_idx).map_err(|e| {
            error!("Failed to open volume: {:?}", defmt::Debug2Format(&e));
        })?;

        let r_handle = mgr.open_root_dir(v_handle).map_err(|e| {
            error!("Failed to open root: {:?}", defmt::Debug2Format(&e));
        })?;

        let (file_handle, log_name_raw) = find_next_log_file::<_, T>(&mut mgr, r_handle);

        // Record schema definition as log header
        LogEntry::write_schema(buffer).ok();
        mgr.write(file_handle, &buffer.get_active_buffer()).ok();
        buffer.reset();

        Ok(Self {
            mgr,
            root: r_handle,
            file: file_handle,
            log_name: log_name_raw,
            buffer,
            last_sync: Instant::now(),
            last_flush: Instant::now(),
            last_health_report: Instant::now(),
            sd_ready: true,
            sync_interval,
            flush_interval,
            health_interval,
        })
    }

    /// Processes a single log entry and performs all required maintenance tasks.
    ///
    /// Operations are executed in a stable sequence:
    /// 1. Buffering of the provided entry.
    /// 2. Draining of the global log channel.
    /// 3. Periodic health reporting.
    /// 4. Threshold-based buffer flushing to SD.
    /// 5. Periodic filesystem metadata synchronization.
    pub async fn process(&mut self, entry: Option<LogEntry>) {
        if let Some(e) = entry {
            self.buffer_entry(&e).await;
        }

        // 1. Drain any pending entries from the global channel
        self.drain_channel().await;

        // 2. Periodic Health Report
        if self.last_health_report.elapsed() >= self.health_interval {
            self.health_report().await;
        }

        // 3. Periodic Flush (respecting hardware minimum write size)
        if self.buffer.pos >= Self::MIN_SD_WRITE_SIZE
            && self.last_flush.elapsed() >= self.flush_interval
        {
            self.flush().await;
        }

        // 4. Periodic Sync
        if self.last_sync.elapsed() >= self.sync_interval {
            self.sync().await;
        }
    }

    /// Appends a log entry to the RAM buffer.
    ///
    /// Triggers an automatic flush to the SD card if the buffer exceeds the operational limit.
    async fn buffer_entry(&mut self, entry: &LogEntry) {
        // Enforce flush before buffer overflow
        if self.buffer.space_remaining() < MAX_LOG_LINE_LEN {
            self.flush().await;
        }
        let _ = self.buffer.write_entry(entry);
    }

    /// Drains all available entries from the global log channel into the local buffer.
    async fn drain_channel(&mut self) {
        while let Ok(next) = LOG_CHANNEL.try_receive() {
            self.buffer_entry(&next).await;
        }
    }

    /// Appends a health report entry to the log.
    async fn health_report(&mut self) {
        let health = rocket_core::log::LoggerHealth::capture(self.sd_ready);
        self.buffer_entry(&LogEntry::LoggerHealth(health)).await;
        self.last_health_report = Instant::now();
    }

    /// Minimum byte threshold for hardware-level SD writes to prevent driver errors.
    const MIN_SD_WRITE_SIZE: usize = 512;

    /// Writes the current buffer content to the SD card and resets the buffer.
    ///
    /// Initiates recovery procedures if the write operation fails.
    async fn flush(&mut self) {
        // Hard-safety check: Never attempt a write below the hardware-safe threshold.
        if self.buffer.pos < Self::MIN_SD_WRITE_SIZE && self.buffer.pos > 0 {
            local_warn!(
                "Blocked attempted sub-minimal SD write: {} bytes",
                self.buffer.pos
            );
            return;
        }

        if self.buffer.pos == 0 {
            return;
        }

        if let Err(e) = self.mgr.write(self.file, &self.buffer.get_active_buffer()) {
            info!(
                "SD Error during write: {:?}. Attempting recovery...",
                defmt::Debug2Format(&e)
            );
            self.sd_ready = false;
            self.recover().await;
        } else {
            local_info!("SD write successful ({} bytes)", self.buffer.pos);
            self.buffer.reset();
            self.last_flush = Instant::now();
        }
    }

    /// Attempts to restore SD card connectivity and reopen the log file.
    async fn recover(&mut self) {
        Timer::after_millis(100).await;
        if let Ok(_) = wait_for_sd_card(&mut self.mgr, Some(3)).await {
            let name_str = core::str::from_utf8(&self.log_name)
                .unwrap()
                .trim_matches(char::from(0));

            if let Ok(h) =
                self.mgr
                    .open_file_in_dir(self.root, name_str, Mode::ReadWriteCreateOrAppend)
            {
                self.file = h;
                let _ = self.mgr.file_seek_from_end(self.file, 0);
                self.sd_ready = true;
                info!("SD Recovery successful!");
            }
        }
    }

    /// Cycles the file handle to ensure metadata (e.g., file size) is updated on disk.
    async fn sync(&mut self) {
        let _ = self.mgr.close_file(self.file);
        let name_str = core::str::from_utf8(&self.log_name)
            .unwrap()
            .trim_matches(char::from(0));

        if let Ok(h) = self
            .mgr
            .open_file_in_dir(self.root, name_str, Mode::ReadWriteCreateOrAppend)
        {
            self.file = h;
            let _ = self.mgr.file_seek_from_end(self.file, 0);
        }
        self.last_sync = Instant::now();
    }

    /// Public getter for Flush Interval (needed for the main task select loop)
    pub fn flush_interval(&self) -> Duration {
        self.flush_interval
    }
}

/// Attempts to initialize the SD card.
/// Returns Ok(()) if successful, or Err if it fails after multiple attempts.
pub async fn wait_for_sd_card<SPI, T>(
    mgr: &mut VolumeManager<SdCard<SPI, Delay>, T, MAX_OPEN_FILES>,
    retries: Option<u32>,
) -> Result<(), ()>
where
    SPI: embedded_hal_1::spi::SpiDevice,
    T: TimeSource,
{
    let mut attempts = 0;
    loop {
        attempts += 1;
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

        Timer::after(Duration::from_millis(500)).await;
    }
}

/// Finds the next LOG_xxx.CSV. Returns the file and the name used.
pub fn find_next_log_file<SPI, T>(
    mgr: &mut VolumeManager<SdCard<SPI, Delay>, T, MAX_OPEN_FILES>,
    root: RawDirectory,
) -> (RawFile, [u8; MAX_NAME_LEN])
where
    SPI: embedded_hal_1::spi::SpiDevice,
    T: TimeSource,
{
    let mut file_num = 0;
    let mut name_buffer = LogBuffer::<MAX_NAME_LEN>::new();

    loop {
        let _ = write!(name_buffer, "LOG_{:03}.CSV", file_num);
        let name_str = core::str::from_utf8(name_buffer.get_active_buffer()).unwrap();

        match mgr.open_file_in_dir(root, name_str, Mode::ReadOnly) {
            Ok(file) => {
                mgr.close_file(file).ok();
                file_num += 1;
                name_buffer.reset();
            }
            Err(_) => {
                let file = mgr
                    .open_file_in_dir(root, name_str, Mode::ReadWriteCreateOrAppend)
                    .expect("Failed to create log file");

                let mut final_name = [0u8; MAX_NAME_LEN];
                final_name.copy_from_slice(name_buffer.get_active_buffer());
                local_info!("Created new log file: LOG_{:03}.CSV", file_num);
                return (file, final_name);
            }
        }
        if file_num > 999 {
            crate::local_error!("Too many log files on SD card");
        }
    }
}
