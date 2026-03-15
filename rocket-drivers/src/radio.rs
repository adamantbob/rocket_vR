use embassy_rp::gpio::{Input, Output};
use embassy_rp::spi::{Async, Spi};
use embassy_time::{Duration, Ticker};
use embedded_hal_bus::spi::ExclusiveDevice;
use rocket_core::blackboard::SENSOR_DATA;
use rocket_core::blackboard::SYSTEM_HEALTH;
use rocket_core::local_info;

use rocket_core::radio_types;

pub mod rfm95;
pub use rfm95::{Bandwidth, CodingRate, LoRaConfig, Rfm95, SignalQuality, SpreadingFactor};

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
        preamble_symbols: 12,
    };

    // Initialise the radio.
    // The driver handles the reset pulse internally.
    let mut radio = Rfm95::new(spi_dev, reset, interrupt, lora_config)
        .await
        .unwrap();

    local_info!("Radio Initialized");
    let mut ticker = Ticker::every(Duration::from_millis(10));
    let mut packet_count = 0;
    loop {
        // Check if it's time to send the schema heartbeat.
        // We use a non-blocking check by comparing against the next schema tick
        // or just by using a separate timer logic if we want to be more precise.
        // For simplicity, we'll just check if the schema_ticker has ticked.
        // Actually, since we are in a single loop with a 10ms ticker,
        // we can just count packets or use a separate task.
        // But the user asked to update radio.rs, so let's keep it simple.

        let cpu_utilization = SYSTEM_HEALTH.cpu_health.read().usage_c0.ceil_percent_u8()
            + SYSTEM_HEALTH.cpu_health.read().usage_c1.ceil_percent_u8();

        let telemetry = radio_types::TelemetryPacketV1 {
            tickstamp_seconds_tenths: (embassy_time::Instant::now().as_millis() / 100) as u16,
            altitude_m: packet_count,
            velocity_z_dms: 0,
            accel_z_dms2: 0,
            lat_offset_1e5: 0,
            lon_offset_1e5: 0,
            flight_state: 0,
            cpu_utilization: cpu_utilization,
        };

        let packet = radio_types::RadioPacket::TelemetryV1(telemetry);

        radio.transmit(&packet).await.unwrap();
        local_info!("Telemetry sent: {}", packet_count);
        packet_count += 1;
        ticker.next().await;
    }
}
