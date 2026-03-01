use embassy_rp::gpio::{Input, Output};
use embassy_rp::spi::{Async, Spi};
use embassy_time::{Duration, Ticker};
use embedded_hal_bus::spi::ExclusiveDevice;
use rocket_core::local_info;

use rocket_core::radio_types;

mod rfm95;
use rfm95::*;

/// Runs the RFM95 Radio
pub async fn radio_task_driver(
    mut spi: Spi<'static, embassy_rp::peripherals::SPI1, Async>,
    mut cs: Output<'static>,
    reset: Output<'static>,
    interrupt: Input<'static>,
) -> ! {
    let spi_dev = ExclusiveDevice::new_no_delay(&mut spi, &mut cs);

    let lora_config = LoRaConfig {
        frequency_hz: 915_000_000,
        spreading_factor: SpreadingFactor::SF9,
        bandwidth: Bandwidth::BW125,
        coding_rate: CodingRate::CR4_5,
        tx_power_dbm: 17,
    };

    // Initialise the radio.
    // The driver handles the reset pulse internally.
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
