#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt_rtt as _;
use embassy_rp::peripherals::PIO0;
use embassy_rp::pio_programs::ws2812::{Grb, PioWs2812, PioWs2812Program};
use embassy_time::Duration;
use proc_macros::tracked_task;
use static_cell::StaticCell;

pub mod hardware;

use rocket_core::utilization::METRICS_CORE0;
use rocket_core::{define_utilization_tasks, spawn_tracked};
use rocket_drivers::usb_and_repl_task;
use rocket_os::InstrumentedExecutor;

// --- Utilization Tracking ---
// Matches the metrics pattern used in the main rocket crate.
define_utilization_tasks!(BLINKY[0], RADIO_RX[0], USB[0]);

#[tracked_task]
#[embassy_executor::task]
async fn blinky(ws: &'static mut PioWs2812<'static, PIO0, 0, 1, Grb>) -> ! {
    const SHORT: Duration = Duration::from_millis(100);
    const LONG: Duration = Duration::from_millis(900);
    loop {
        // Simple pulsing green for "Alive" status.
        let _ = ws.write(&[smart_leds::RGB8::new(0, 10, 0)]).await;
        embassy_time::Timer::after(SHORT).await;
        let _ = ws.write(&[smart_leds::RGB8::new(0, 0, 0)]).await;
        embassy_time::Timer::after(LONG).await;
    }
}

#[tracked_task]
#[embassy_executor::task]
async fn radio_rx_task(
    spi: embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI1, embassy_rp::spi::Async>,
    cs: embassy_rp::gpio::Output<'static>,
    reset: embassy_rp::gpio::Output<'static>,
    dio0: embassy_rp::gpio::Input<'static>,
) -> ! {
    use embedded_hal_bus::spi::ExclusiveDevice;
    use rocket_core::radio_types::TelemetryPacket;
    use rocket_drivers::radio::{Bandwidth, CodingRate, LoRaConfig, Rfm95, SpreadingFactor};

    // The Integrated Feather RP2040 RFM95 uses 915MHz LoRa.
    let config = LoRaConfig {
        frequency_hz: 915_000_000,
        bandwidth: Bandwidth::BW125,
        spreading_factor: SpreadingFactor::SF9,
        coding_rate: CodingRate::CR4_5,
        tx_power_dbm: 17,
        preamble_symbols: 12,
    };

    let spi_dev = ExclusiveDevice::new(spi, cs, embassy_time::Delay);
    let mut radio: Rfm95<
        ExclusiveDevice<
            embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI1, embassy_rp::spi::Async>,
            embassy_rp::gpio::Output<'static>,
            embassy_time::Delay,
        >,
        embassy_rp::gpio::Output<'static>,
        embassy_rp::gpio::Input<'static>,
    > = Rfm95::new(spi_dev, reset, dio0, config).await.unwrap();

    loop {
        match radio.receive::<TelemetryPacketV1>().await {
            Ok((packet, quality)) => {
                log::info!(
                    "RX: Time={}s FS={} alt={}m rssi={} snr={} cpu={}",
                    packet.tickstamp_seconds_tenths as f32 / 10.0,
                    packet.flight_state,
                    packet.altitude_m,
                    quality.rssi_dbm,
                    quality.snr_db_tenths,
                    packet.cpu_utilization,
                );
            }
            Err(e) => {
                log::error!("Radio RX error: {:?}", e);
            }
        }
    }
}

static EXECUTOR_LOW: StaticCell<InstrumentedExecutor> = StaticCell::new();
static NEOPIXEL: StaticCell<PioWs2812<'static, PIO0, 0, 1, Grb>> = StaticCell::new();

#[entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    let mut hw = hardware::Hardware::init(p);

    // Initialize the instrumented executor on Core 0.
    // This provides the metrics and task tracking utilized by the rocket project.
    let executor0 = EXECUTOR_LOW.init(InstrumentedExecutor::new(&METRICS_CORE0));

    executor0.run(|spawner| {
        // Neopixel setup
        let program = PioWs2812Program::new(&mut hw.neo_pio.common);
        let ws_driver = PioWs2812::new(
            &mut hw.neo_pio.common,
            hw.neo_pio.sm0,
            hw.neo_dma,
            hardware::Irqs,
            hw.neo_data,
            &program,
        );
        let ws = NEOPIXEL.init(ws_driver);

        // spawn_tracked! injects the TaskId argument added by #[tracked_task].
        spawn_tracked!(spawner, BLINKY, blinky(ws));

        spawn_tracked!(
            spawner,
            RADIO_RX,
            radio_rx_task(hw.radio_spi, hw.radio_cs, hw.radio_reset, hw.radio_dio0)
        );

        spawn_tracked!(
            spawner,
            USB,
            usb_and_repl_task(hw.usb_device, hw.usb_logger_class, hw.usb_class)
        );

        log::info!("Base Station started on Core 0");
    });
}
