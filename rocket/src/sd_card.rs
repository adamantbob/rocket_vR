use crate::state_machine::SYSTEM_HEALTH;
use crate::{Irqs, SDCard, SDCardResources, error};
use embassy_rp::gpio::{Level, Output};
use embassy_rp::spi::Spi;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use embedded_sdmmc::{SdCard, VolumeManager};
use proc_macros::tracked_task;
use static_cell::StaticCell;

use embassy_futures::select::{Either, select};
use rocket_core::log::{LOG_CHANNEL, LogBuffer};

// Define this to match the internal storage of VolumeManager
const MAX_OPEN_FILES: usize = 4;
// 11 bytes is for "LOG_XXX.CSV"
const MAX_NAME_LEN: usize = 11;
// Logger buffer size
const LOG_BUFFER_SIZE: usize = 4096;
// Initialization Speed for the SD Card
const SD_INIT_BAUD_RATE: u32 = 400_000;
// Operational Speed for the SD Card
const SD_OP_BAUD_RATE: u32 = 2_000_000;
// Flush interval for the SD Card
const SD_FLUSH_INTERVAL: Duration = Duration::from_secs(5);
// Sync interval for the SD Card
const SD_SYNC_INTERVAL: Duration = Duration::from_secs(10);
// Health interval for the SD Card
const SD_HEALTH_INTERVAL: Duration = Duration::from_secs(1);

pub mod logger;

use logger::{SdLogger, wait_for_sd_card};

#[derive(Default)]
pub struct DummyTime;
impl embedded_sdmmc::TimeSource for DummyTime {
    fn get_timestamp(&self) -> embedded_sdmmc::Timestamp {
        embedded_sdmmc::Timestamp {
            year_since_1970: 56, // 2026
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

/// Orchestrates SD card initialization and continuous log recording.
///
/// Implements a multi-phase initialization:
/// 1. Discovery at 400kHz for hardware compatibility.
/// 2. Performance boost to 16MHz after successful discovery.
/// 3. Continuous log processing and periodic maintenance.
#[tracked_task(SDCard)]
#[embassy_executor::task]
pub async fn sd_card_task(r: SDCardResources, _irqs: Irqs) -> ! {
    static BUFFER: StaticCell<LogBuffer<LOG_BUFFER_SIZE>> = StaticCell::new();
    let buffer = BUFFER.init(LogBuffer::new());

    // Phase 1: Setup SPI with DMA at discovery frequency (400kHz)
    let mut config = embassy_rp::spi::Config::default();

    let mut spi = Spi::new(
        r.spi, r.clk, r.mosi, r.miso, r.dma_tx, r.dma_rx, Irqs, config,
    );
    let mut cs = Output::new(r.cs, Level::High);

    // Phase 2: Discovery and initial volume check
    {
        let spi_dev = ExclusiveDevice::new_no_delay(&mut spi, &mut cs);
        let mut volume_mgr = VolumeManager::new(SdCard::new(spi_dev, Delay), DummyTime);

        if let Err(_) = wait_for_sd_card(&mut volume_mgr, Some(5)).await {
            error!("SD Card initialization failed persistently. Task exiting.");
            loop {
                Timer::after_millis(100).await;
            }
        }
    }

    // Phase 3: Operational frequency boost
    spi.set_frequency(SD_OP_BAUD_RATE);
    let spi_dev = ExclusiveDevice::new_no_delay(spi, cs);
    let volume_mgr = VolumeManager::new(SdCard::new(spi_dev, Delay), DummyTime);

    // Phase 4: SD Logger Initialization
    let mut logger = match SdLogger::initialize(
        volume_mgr,
        buffer,
        SD_FLUSH_INTERVAL,
        SD_SYNC_INTERVAL,
        SD_HEALTH_INTERVAL,
    )
    .await
    {
        Ok(l) => l,
        Err(_) => {
            // Error logged by initialize()
            loop {
                Timer::after_millis(100).await;
            }
        }
    };

    // Phase 5: Continuous Processing Loop
    loop {
        SYSTEM_HEALTH.sd_health.update(rocket_core::SDCardHealth {
            tickstamp: embassy_time::Instant::now().as_ticks() as u64,
            initialized: true,
            buffer_usage: 0, // TODO: Get actual buffer usage
            flush_errors: 0,
        });

        // Block until data is available or a maintenance timeout occurs
        let entry_or_timeout =
            select(LOG_CHANNEL.receive(), Timer::after(logger.flush_interval())).await;

        match entry_or_timeout {
            Either::First(entry) => {
                logger.process(Some(entry)).await;
            }
            Either::Second(_) => {
                logger.process(None).await;
            }
        }
    }
}
