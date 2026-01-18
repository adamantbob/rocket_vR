#![no_std]
#![no_main]

#[cfg(not(any(feature = "rp2040", feature = "rp2350")))]
compile_error!(
    "Please use a chip alias:\n  - For Pico (RP2040):   cargo run-pico\n  - For Pico 2 (RP2350): cargo run-pico2"
);

#[cfg(all(feature = "rp2040", not(target_arch = "arm")))]
compile_error!("Mismatched target for RP2040! Please use 'cargo run-pico'");

#[cfg(all(feature = "rp2350", not(target_arch = "arm")))]
compile_error!("Mismatched target for RP2350! Please use 'cargo run-pico2'");

use cortex_m_rt::entry;
use cyw43::Control;
use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::interrupt::{InterruptExt, Priority};
use embassy_rp::peripherals::{DMA_CH0, PIO0, UART0, USB};
use embassy_rp::pio::InterruptHandler as PioInterruptHandler;
use embassy_rp::uart::BufferedInterruptHandler;
use embassy_rp::usb::{Driver, Instance, InterruptHandler as USBInterruptHandler};
use embassy_rp::{bind_interrupts, interrupt};
use embassy_time::{Duration, Instant, TICK_HZ, Timer};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use heapless::Vec;
use proc_macros::tracked_task;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

mod instrumented_executor;
use instrumented_executor::{InstrumentedExecutor, InstrumentedInterruptExecutor};

mod gps;
mod macros;
mod usb;
mod utilization;
mod wifi;

use crate::utilization::{TrackedExt, stats_task};
use crate::wifi::WifiRunner;

define_utilization_tasks!(Stats, Wifi, Usb, Blinky, Repl, RunMed, Main, GPS);

assign_resources! {
    GpsResources {
        uart: UART0,
        tx: PIN_12,
        rx: PIN_13,
    }
    WifiResources {
        pwr: PIN_23,
        cs: PIN_25,
        dio: PIN_24,
        clk: PIN_29,
        pio: PIO0,
        dma: DMA_CH0,
    }
    USBResources {
        usb: USB,
    }
}

#[cfg(feature = "rp2350")]
const DESCRIPTION: embassy_rp::binary_info::EntryAddr = embassy_rp::binary_info::rp_program_description!(
    c"This example tests the RP Pico 2 W's onboard LED, connected to GPIO 0 of the cyw43 \
    (WiFi chip) via PIO 0 over the SPI bus."
);
#[cfg(feature = "rp2040")]
const DESCRIPTION: embassy_rp::binary_info::EntryAddr = embassy_rp::binary_info::rp_program_description!(
    c"This example tests the RP Pico W's onboard LED, connected to GPIO 0 of the cyw43 \
    (WiFi chip) via PIO 0 over the SPI bus."
);

// Program metadata for `picotool info`.
// This isn't needed, but it's recommended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Rocket vR"),
    DESCRIPTION,
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// Bind Interrupts to their appropriate interrupt handler
bind_interrupts!(pub struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    USBCTRL_IRQ => USBInterruptHandler<USB>;
    UART0_IRQ => BufferedInterruptHandler<UART0>;
});

#[tracked_task(GPS)]
pub async fn gps_task(r: GpsResources, irqs: Irqs) -> ! {
    // We pass the individual fields from our resource struct
    // into the actual driver logic.
    gps::GPS::run(r.uart, r.tx, r.rx, irqs).await
}

// Background Logger Task
// #[embassy_executor::task]
// async fn logger_task(driver: Driver<'static, USB>) {
//     embassy_usb_logger::run!(1024, log::LevelFilter::Info, driver);
// }

// useful Stuff
// adc example: ticker - execute every x time. Use this for main logging loop
// let mut ticker = Ticker::every(Duration::from_secs(1));

#[tracked_task(Main)]
#[embassy_executor::task]
async fn run_low(spawner: Spawner, p: embassy_rp::Peripherals) {
    spawner.spawn(unwrap!(stats_task(TASK_NAMES, Stats)));

    let r = AssignedResources::take(p);

    let (mut control, device) = wifi::init_and_spawn(spawner, r.WifiResources, Irqs).await;

    let (mut class, usb_runner) = usb::setup_usb(r.USBResources.usb);

    let gps_runner = gps_task(r.GpsResources, Irqs);

    spawner.spawn(unwrap!(blinky(control)));

    join(usb_runner, core0_repl(&mut class)).tracked(Usb).await;
}

#[tracked_task(Blinky)]
#[embassy_executor::task]
async fn blinky(mut control: Control<'static>) -> ! {
    let short_delay = Duration::from_millis(100);
    let long_delay = Duration::from_millis(900);
    loop {
        control.gpio_set(0, true).await;
        Timer::after(short_delay).await;
        control.gpio_set(0, false).await;
        Timer::after(long_delay).await;
    }
}

async fn core0_repl<'d, T: Instance + 'd>(serial: &mut CdcAcmClass<'d, Driver<'d, T>>) -> ! {
    async move {
        loop {
            serial.wait_connection().await;
            info!("Command REPL Connected");
            let _ = run_repl(serial).await;
            info!("Command REPL Disconnected");
        }
    }
    .tracked(Repl)
    .await
}

// Run the Command REPL.
// Returns when the connection is lost.
// By separating the inner logic out, the Disconnected Error can be handled in the main loop.
async fn run_repl<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), usb::Disconnected> {
    const RX_BUF_SIZE: usize = 64;
    const COMMAND_BUF_SIZE: usize = 64;
    const _: () = assert!(
        COMMAND_BUF_SIZE >= RX_BUF_SIZE,
        "REPL command buffer must be at least as large as the receive buffer!"
    );

    let mut rx_buf = [0; RX_BUF_SIZE];
    let mut command_buf = Vec::<u8, COMMAND_BUF_SIZE>::new();
    const BACKSPACE: u8 = 0x08; // \b
    loop {
        let n = class.read_packet(&mut rx_buf).await?;

        for &b in &rx_buf[..n] {
            if b == b'\n' || b == b'\r' {
                if !command_buf.is_empty() {
                    if let Ok(s) = core::str::from_utf8(command_buf.as_slice()) {
                        info!("command: {}", s);
                    } else {
                        info!("command (raw): {:?}", command_buf);
                    }
                    // echo the command back to the client
                    class.write_packet(b"\r\n").await?;
                    class.write_packet(command_buf.as_slice()).await?;
                    class.write_packet(b" received\r\n").await?;
                    command_buf.clear();
                }
            } else if b == BACKSPACE {
                command_buf.pop();
            } else if command_buf.extend_from_slice(&[b]).is_err() {
                error!("command_buf overflow");
                class.write_packet(b"command_buf overflow\n\r").await?;
                command_buf.clear();
            }
        }
        // echo the received data back to the client (this is so it appears on the terminal)
        class.write_packet(&rx_buf[..n]).await?;
    }
}

// Test task for the pre-emption test
// This is somewhat bad form to block for a long time in an interrupt.
#[embassy_executor::task]
async fn run_med() {
    async move {
        loop {
            let start = Instant::now();
            // info!("    [med] Starting long computation");

            // Spin-wait to simulate a long CPU computation
            // embassy_time::block_for(embassy_time::Duration::from_millis(50)); // ~1 second

            let end = Instant::now();
            let ms = end.duration_since(start).as_ticks() * 1000 / TICK_HZ;
            // info!("    [med] done in {} ms", ms);

            Timer::after(Duration::from_millis(900)).await;
        }
    }
    .tracked(RunMed)
    .await
}

static EXECUTOR_HIGH: InstrumentedInterruptExecutor = InstrumentedInterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<InstrumentedExecutor> = StaticCell::new();

#[interrupt]
unsafe fn SWI_IRQ_1() {
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

#[entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    // High-priority executor: SWI_IRQ_1, priority level 2
    interrupt::SWI_IRQ_1.set_priority(Priority::P2);
    let spawner = EXECUTOR_HIGH.start(interrupt::SWI_IRQ_1);
    spawner.spawn(unwrap!(run_med()));

    let executor = EXECUTOR_LOW.init(InstrumentedExecutor::new());
    executor.run(|spawner| {
        spawner.spawn(unwrap!(run_low(spawner, p)));
    });
}
