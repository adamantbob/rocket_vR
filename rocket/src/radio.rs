// use crate::state_machine::SYSTEM_HEALTH;
use crate::{Irqs, Radio, RadioResources, local_info};
use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::spi::Spi;
use embassy_time::{Duration, Ticker, Timer};
use embedded_hal_bus::spi::ExclusiveDevice;
use proc_macros::tracked_task;

use rocket_core::radio_types;

mod rfm95;
use rfm95::*;

/// Runs the RFM95 Radio
#[tracked_task(Radio)]
#[embassy_executor::task]
pub async fn radio_task(r: RadioResources, _irqs: Irqs) -> ! {
    let mut config = embassy_rp::spi::Config::default();
    config.frequency = 1_000_000;
    config.phase = embassy_rp::spi::Phase::CaptureOnFirstTransition;
    config.polarity = embassy_rp::spi::Polarity::IdleLow;

    // Use maximum drive strength on critical pins to ensure level shifter performance.
    let cs = Output::new(r.cs, Level::High);

    // RFM95 Reset is Active-Low (documented in Semtech datasheet).
    // Initialize HIGH (Enabled) for normal operation.
    let reset = Output::new(r.reset, Level::High);

    // Settling time for power rails.
    Timer::after(Duration::from_millis(50)).await;

    let mut spi = Spi::new(
        r.spi,
        r.clk,
        r.mosi,
        r.miso,
        r.dma_tx,
        r.dma_rx,
        Irqs,
        config.clone(),
    );
    let mut cs_pin = cs;
    let spi_dev = ExclusiveDevice::new_no_delay(&mut spi, &mut cs_pin);

    let lora_config = LoRaConfig {
        frequency_hz: 915_000_000,
        spreading_factor: SpreadingFactor::SF9,
        bandwidth: Bandwidth::BW125,
        coding_rate: CodingRate::CR4_5,
        tx_power_dbm: 17,
    };
    let interrupt = Input::new(r.dio0, Pull::Down);

    // Initialise the radio.
    // The driver handles the reset pulse internally.
    #[cfg(feature = "rp2040")]
    let mut radio = Rfm95::new(spi_dev, reset, interrupt, lora_config)
        .await
        .unwrap();
    #[cfg(feature = "rp2350")]
    let mut radio = Rfm95::new(spi_dev, reset, interrupt, lora_config)
        .await
        .unwrap();

    local_info!("Radio Initialized");
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut packet_count = 0;
    loop {
        let packet = radio_types::TelemetryPacket {
            tickstamp: embassy_time::Instant::now().as_ticks() as u64,
            altitude_mm: packet_count,
            velocity_z_mms: 0,
            accel_z_mg: 0,
            lat_raw: 0,
            lon_raw: 0,
            flight_state: 0,
            cpu0_utilization: 0,
            cpu1_utilization: 0,
        };
        radio.transmit(&packet).await.unwrap();
        local_info!("Telemetry sent: {}", packet_count);
        packet_count += 1;
        ticker.next().await;
    }
}
