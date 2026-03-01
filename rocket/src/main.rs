#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt_rtt as _;
use embassy_futures::join::join;
use embassy_rp::interrupt;
use embassy_rp::interrupt::{InterruptExt, Priority};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use heapless::Vec;
use proc_macros::tracked_task;
use static_cell::StaticCell;

// New hardware module
pub mod hardware;

// Removed local modules, now using workspace crates
use rocket_drivers::{
    LedState, gps_task as driver_gps_task, imu_task as driver_imu_task,
    radio_task as driver_radio_task, sd_card_task as driver_sd_card_task,
    wifi_task as driver_wifi_task,
};
use rocket_os::{INTERRUPT_EX_PTR, InstrumentedExecutor, InstrumentedInterruptExecutor};

pub use rocket_core;
use rocket_core::utilization::{self, TrackedExt};
use rocket_core::{define_utilization_tasks, error, info, spawn_tracked};
pub use rocket_drivers;
use rocket_os::health::panic_monitor_task;
use rocket_os::health::utilization::stats_task;

mod datacells;
mod state_machine;
use state_machine::state_machine_task;

define_utilization_tasks!(
    STATS[0],
    WIFI[0],
    BLINKY[0],
    REPL[0],
    STATE_MACHINE[0],
    USB[0],
    GPS[0],
    IMU[0],
    PANIC_MONITOR0[0],
    RADIO[0],
    SDCARD[1],
    PANIC_MONITOR1[1]
);

use rocket_os::panic::check_core_panic;

#[tracked_task]
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

async fn core0_repl<'d, T: embassy_rp::usb::Instance + 'd>(
    serial: &mut CdcAcmClass<'d, embassy_rp::usb::Driver<'d, T>>,
    utilization_id: utilization::TaskId,
) -> ! {
    async move {
        loop {
            serial.wait_connection().await;
            info!("Command REPL Connected");
            let _ = run_repl(serial).await;
            info!("Command REPL Disconnected");
        }
    }
    .tracked(utilization_id) // Wait, core0_repl needs an ID injected
    .await
}

// Run the Command REPL.
// Returns when the connection is lost.
// By separating the inner logic out, the Disconnected Error can be handled in the main loop.
async fn run_repl<'d, T: embassy_rp::usb::Instance + 'd>(
    class: &mut CdcAcmClass<'d, embassy_rp::usb::Driver<'d, T>>,
) -> Result<(), rocket_drivers::usb::Disconnected> {
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
    spawner: embassy_executor::Spawner,
    usb: embassy_rp::Peri<'static, embassy_rp::peripherals::USB>,
    wifi_pwr: embassy_rp::gpio::Output<'static>,
    wifi_spi: cyw43_pio::PioSpi<'static, embassy_rp::peripherals::PIO0, 0>,
    gps_uart: embassy_rp::uart::BufferedUart,
    imu_i2c: embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>,
    radio_spi: embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI1, embassy_rp::spi::Async>,
    radio_cs: embassy_rp::gpio::Output<'static>,
    radio_reset: embassy_rp::gpio::Output<'static>,
    radio_dio0: embassy_rp::gpio::Input<'static>,
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
    let usb_driver = embassy_rp::usb::Driver::new(usb, hardware::Irqs);
    let (mut class, usb_runner) = rocket_drivers::usb::setup_usb(usb_driver);

    // 2. Spawn Hardware Tasks
    spawn_tracked!(spawner, WIFI, wifi_task(wifi_pwr, wifi_spi));
    spawn_tracked!(spawner, GPS, gps_task(gps_uart));
    spawn_tracked!(spawner, IMU, imu_task(imu_i2c));

    // 3. Start Application Level Tasks
    unsafe {
        let stack_ptr = core::ptr::addr_of_mut!(crate::CORE1_STACK);
        let stack_bottom = stack_ptr as *const u32;
        let stack_top = (stack_ptr as *const u8).add(16384 - 1024) as *const u32;

        spawner
            .spawn(stats_task(TASK_NAMES, TASK_CORES, STATS, (stack_bottom, stack_top)).unwrap());

        spawn_tracked!(spawner, BLINKY, blinky());

        spawn_tracked!(
            spawner,
            RADIO,
            radio_task(radio_spi, radio_cs, radio_reset, radio_dio0)
        );

        // Target core 1 (the executor core)
        spawn_tracked!(spawner, PANIC_MONITOR1, panic_monitor_task(1));

        // Join the runners that need to stay alive for this core
        join(usb_runner.tracked(USB), core0_repl(&mut class, REPL)).await;
    }
}

pub use rocket_core::utilization::{METRICS_CORE0, METRICS_CORE1};

static EXECUTOR_HIGH: StaticCell<InstrumentedInterruptExecutor> = StaticCell::new();
static EXECUTOR_LOW: StaticCell<InstrumentedExecutor> = StaticCell::new();

const CORE1_STACK_SIZE: usize = 16384;

static mut CORE1_STACK: Stack<CORE1_STACK_SIZE> = Stack::new();
static EXECUTOR_CORE1: StaticCell<InstrumentedExecutor> = StaticCell::new();

#[interrupt]
unsafe fn SWI_IRQ_1() {
    // Acquire the pointer set during start()
    let ptr = INTERRUPT_EX_PTR.load(portable_atomic::Ordering::Acquire);

    if !ptr.is_null() {
        unsafe {
            // Safety: We know this points to a 'static mut InstrumentedInterruptExecutor
            // because that's the only thing that stores to INTERRUPT_EX_PTR.
            (*ptr).on_interrupt();
        }
    }
}

#[entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());
    let hw = hardware::Hardware::init(p);

    let core1 = hw.core1;
    let sd_spi = hw.sd_spi;
    let sd_cs = hw.sd_cs;

    // High-priority executor: SWI_IRQ_1, priority level 2
    let exec_high = EXECUTOR_HIGH.init(InstrumentedInterruptExecutor::new());
    interrupt::SWI_IRQ_1.set_priority(Priority::P2);
    let spawner = exec_high.start(interrupt::SWI_IRQ_1);
    spawn_tracked!(spawner, STATE_MACHINE, state_machine_task());

    // Spawn the SD Card task on Core 1.
    // The SD Card is blocking and the lowest priority, so it should run on a separate core.
    spawn_core1(
        core1,
        unsafe { &mut *core::ptr::addr_of_mut!(CORE1_STACK) },
        move || {
            // Paint the stack for absolute watermark tracking.
            // Safety: We are at the very beginning of the core's execution context.
            // Only core 1 is tracked for high water mark as core 0's stack
            // is calculated for size.
            unsafe {
                rocket_os::health::stack::setup_core1_stack_high_watermark_tracking(
                    &raw mut CORE1_STACK as *mut u32,
                    CORE1_STACK_SIZE,
                );
            }
            let executor1 = EXECUTOR_CORE1.init(InstrumentedExecutor::new(&METRICS_CORE1));
            executor1.run(|spawner| {
                spawn_tracked!(spawner, SDCARD, sd_card_task(sd_spi, sd_cs));
                spawn_tracked!(spawner, PANIC_MONITOR0, panic_monitor_task(0));
            });
        },
    );

    let executor0 = EXECUTOR_LOW.init(InstrumentedExecutor::new(&METRICS_CORE0));
    executor0.run(|spawner| {
        spawner.spawn(
            init_low_prio_tasks(
                spawner,
                hw.usb,
                hw.wifi_pwr,
                hw.wifi_spi,
                hw.gps_uart,
                hw.imu_i2c,
                hw.radio_spi,
                hw.radio_cs,
                hw.radio_reset,
                hw.radio_dio0,
            )
            .unwrap(),
        );
    });
}

// --- Hardware Task Wrappers ---

#[tracked_task]
#[embassy_executor::task]
pub async fn wifi_task(
    pwr: embassy_rp::gpio::Output<'static>,
    spi: cyw43_pio::PioSpi<'static, embassy_rp::peripherals::PIO0, 0>,
) -> ! {
    driver_wifi_task(pwr, spi).await
}

#[tracked_task]
#[embassy_executor::task]
pub async fn gps_task(uart: embassy_rp::uart::BufferedUart) -> ! {
    driver_gps_task(uart).await
}

#[tracked_task]
#[embassy_executor::task]
pub async fn imu_task(
    i2c: embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>,
) -> ! {
    driver_imu_task(i2c).await
}

#[tracked_task]
#[embassy_executor::task]
pub async fn radio_task(
    spi: embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI1, embassy_rp::spi::Async>,
    cs: embassy_rp::gpio::Output<'static>,
    reset: embassy_rp::gpio::Output<'static>,
    dio0: embassy_rp::gpio::Input<'static>,
) -> ! {
    driver_radio_task(spi, cs, reset, dio0).await
}

#[tracked_task]
#[embassy_executor::task]
pub async fn sd_card_task(
    spi: embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Async>,
    cs: embassy_rp::gpio::Output<'static>,
) -> ! {
    driver_sd_card_task(spi, cs).await
}
