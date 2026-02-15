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
use defmt::unwrap;
use defmt_rtt as _;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::interrupt::{InterruptExt, Priority};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_rp::peripherals::{DMA_CH0, DMA_CH3, DMA_CH4, I2C0, PIO0, SPI0, UART0, USB};
use embassy_rp::usb::{Driver, Instance};
use embassy_rp::{bind_interrupts, interrupt};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use heapless::Vec;
use proc_macros::tracked_task;
use static_cell::StaticCell;

mod panic;

mod instrumented_executor;
use instrumented_executor::{InstrumentedExecutor, InstrumentedInterruptExecutor};

mod channels;
mod datacells;
mod gps;
mod imu;
mod macros;
mod sd_card;
mod state_machine;
mod usb;
mod utilization;
mod wifi;

pub use heapless;
pub use rocket_core;

use crate::utilization::{TrackedExt, stats_task};
use crate::wifi::LedState;

define_utilization_tasks!(
    Stats[0],
    Wifi[0],
    Blinky[0],
    Repl[0],
    StateMachine[0],
    InitLowPrioTasks[0],
    GPS[0],
    IMU[0],
    SDCard[1]
);

assign_resources! {
    GPSResources {
        uart: UART0,
        tx: PIN_12,
        rx: PIN_13,
        dma_tx: DMA_CH1,
        dma_rx: DMA_CH2,
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
    IMUResources {
        i2c: I2C0,
        scl: PIN_9,
        sda: PIN_8,
    }
    SDCardResources {
        spi: SPI0,
        miso: PIN_16,
        mosi: PIN_19,
        clk: PIN_18,
        cs: PIN_17,
        dma_tx: DMA_CH3,
        dma_rx: DMA_CH4,
    }
    CoreResources {
        core1: CORE1,
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
// Bind Interrupts to their appropriate interrupt handler
bind_interrupts!(pub struct Irqs {
    PIO0_IRQ_0 => embassy_rp::pio::InterruptHandler<PIO0>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
    UART0_IRQ => embassy_rp::uart::BufferedInterruptHandler<UART0>;
    I2C0_IRQ => embassy_rp::i2c::InterruptHandler<I2C0>;
    DMA_IRQ_0 => embassy_rp::dma::InterruptHandler<DMA_CH0>,
                 embassy_rp::dma::InterruptHandler<DMA_CH3>,
                 embassy_rp::dma::InterruptHandler<DMA_CH4>;
});

use panic::{PANIC_RECORDS, check_core_panic};

#[tracked_task(Blinky)]
#[embassy_executor::task]
async fn blinky() -> ! {
    let short_delay = Duration::from_millis(100);
    let long_delay = Duration::from_millis(900);
    loop {
        LedState::On.send().await;
        Timer::after(short_delay).await;
        LedState::Off.send().await;
        Timer::after(long_delay).await;
    }
}

/// Dedicated task to monitor the "other" core for crashes.
#[embassy_executor::task(pool_size = 2)]
pub(crate) async fn panic_monitor_task(target_core: usize) -> ! {
    loop {
        if let Some(report) = check_core_panic(target_core) {
            if report.message == "(Incomplete formatting)" {
                error!("CORE {} PANIC DETECTED (Partial Sentinel)", target_core);
            } else {
                error!(
                    "CORE {} PANIC DETECTED: {} at {}:{}",
                    target_core,
                    report.message.as_str(),
                    report.file.as_str(),
                    report.line
                );
            }
        }
        Timer::after_millis(500).await;
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

/* Init Tasks */
// These tasks are designed to start up, spawn all the tasks then return.
// This allows for a clean separation of concerns and makes it easier to reason about the code.

/// Spawns all low priority tasks.
/// Start each init and move on as we wait for init.
/// Critical tasks have their own state and will report their status via the
/// global task status channel.
#[embassy_executor::task]
pub async fn init_low_prio_tasks(
    spawner: Spawner,
    wifi: WifiResources,
    usb: USBResources,
    gps: GPSResources,
    imu: IMUResources,
) {
    // Check for previous panics on BOTH cores
    for core in 0..2 {
        if let Some(report) = check_core_panic(core) {
            error!(
                "PREVIOUS PANIC DETECTED on Core {}: {} at {}:{}",
                core,
                report.message.as_str(),
                report.file.as_str(),
                report.line
            );
        }
    }

    // 1. Initialize USB: It returns a class and a future to run.
    let (mut class, usb_runner) = usb::setup_usb(usb.usb);

    // 2. Spawn Hardware Tasks
    spawner.spawn(unwrap!(crate::wifi::wifi_task(wifi, Irqs)));
    spawner.spawn(unwrap!(crate::gps::gps_task(gps, Irqs)));
    spawner.spawn(unwrap!(crate::imu::imu_task(imu, Irqs)));

    // 3. Start Application Level Tasks
    spawner.spawn(unwrap!(stats_task(TASK_NAMES, TASK_CORES, Stats)));
    spawner.spawn(unwrap!(blinky()));

    // Target core 1 (the executor core)
    spawner.spawn(unwrap!(panic_monitor_task(1)));

    // Join the runners that need to stay alive for this core
    join(usb_runner, core0_repl(&mut class)).await;
}

pub static METRICS_CORE0: instrumented_executor::ExecutorMetrics =
    instrumented_executor::ExecutorMetrics::new();
pub static METRICS_CORE1: instrumented_executor::ExecutorMetrics =
    instrumented_executor::ExecutorMetrics::new();

static EXECUTOR_HIGH: InstrumentedInterruptExecutor = InstrumentedInterruptExecutor::new();
static EXECUTOR_LOW: StaticCell<InstrumentedExecutor> = StaticCell::new();

static mut CORE1_STACK: Stack<16384> = Stack::new();
static EXECUTOR_CORE1: StaticCell<InstrumentedExecutor> = StaticCell::new();

#[interrupt]
unsafe fn SWI_IRQ_1() {
    unsafe { EXECUTOR_HIGH.on_interrupt() }
}

#[entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    let r = AssignedResources::take(p);

    // High-priority executor: SWI_IRQ_1, priority level 2
    interrupt::SWI_IRQ_1.set_priority(Priority::P2);
    let spawner = EXECUTOR_HIGH.start(interrupt::SWI_IRQ_1);
    spawner.spawn(unwrap!(state_machine::state_machine_task()));

    // Spawn the SD Card task on Core 1.
    // The SD Card is blocking and the lowest priority, so it should run on a separate core.
    spawn_core1(
        r.CoreResources.core1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            let executor1 = EXECUTOR_CORE1.init(InstrumentedExecutor::new(&METRICS_CORE1));
            executor1.run(|spawner| {
                spawner.spawn(unwrap!(crate::sd_card::sd_card_task(
                    r.SDCardResources,
                    Irqs
                )));
                spawner.spawn(unwrap!(panic_monitor_task(0)));
            });
        },
    );

    let executor0 = EXECUTOR_LOW.init(InstrumentedExecutor::new(&METRICS_CORE0));
    executor0.run(|spawner| {
        spawner.spawn(unwrap!(init_low_prio_tasks(
            spawner,
            r.WifiResources,
            r.USBResources,
            r.GPSResources,
            r.IMUResources,
        )));
    });
}
