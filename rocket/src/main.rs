#![no_std]
#![no_main]

use cortex_m_rt::entry;
use defmt_rtt as _;
use embassy_rp::interrupt;
use embassy_rp::interrupt::{InterruptExt, Priority};
use embassy_rp::multicore::{Stack, spawn_core1};
use embassy_time::{Duration, Timer};
use proc_macros::tracked_task;
use static_cell::StaticCell;

pub mod hardware;

use rocket_drivers::{
    LedState, gps_task, imu_task_driver, radio_task_driver, sd_card_task_driver, usb_and_repl_task,
    wifi_task_driver,
};
use rocket_os::{INTERRUPT_EX_PTR, InstrumentedExecutor, InstrumentedInterruptExecutor};

pub use rocket_core;
use rocket_core::{define_utilization_tasks, error, spawn_tracked};
pub use rocket_drivers;
use rocket_os::health::panic_monitor_task;
use rocket_os::health::utilization::stats_task;

mod datacells;
mod state_machine;
use state_machine::state_machine_task;

pub use rocket_core::utilization::{METRICS_CORE0, METRICS_CORE1};

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

    // High-priority executor: SWI_IRQ_1, priority level 2
    let exec_high = EXECUTOR_HIGH.init(InstrumentedInterruptExecutor::new());
    interrupt::SWI_IRQ_1.set_priority(Priority::P2);
    let spawner = exec_high.start(interrupt::SWI_IRQ_1);
    spawn_tracked!(spawner, STATE_MACHINE, state_machine_task());

    // Spawn the SD Card task on Core 1.
    // The SD Card is blocking and the lowest priority, so it should run on a separate core.
    spawn_core1(
        hw.core1,
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
                spawn_tracked!(spawner, SDCARD, sd_card_task(hw.sd_spi, hw.sd_cs));
                spawn_tracked!(spawner, PANIC_MONITOR0, panic_monitor_task(0));
            });
        },
    );

    let executor0 = EXECUTOR_LOW.init(InstrumentedExecutor::new(&METRICS_CORE0));
    executor0.run(|spawner| {
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
        // Spawn the stats task on Core 0.
        unsafe {
            let stack_ptr = core::ptr::addr_of_mut!(crate::CORE1_STACK);
            let stack_bottom = stack_ptr as *const u32;
            let stack_top = (stack_ptr as *const u8).add(CORE1_STACK_SIZE - 1024) as *const u32;

            spawner.spawn(
                stats_task(TASK_NAMES, TASK_CORES, STATS, (stack_bottom, stack_top)).unwrap(),
            );
        }
        // Target core 1 (the executor core)
        spawn_tracked!(spawner, PANIC_MONITOR1, panic_monitor_task(1));

        // Spawn everything directly using the hw struct
        spawn_tracked!(spawner, WIFI, wifi_task(hw.wifi_pwr, hw.wifi_spi));
        spawn_tracked!(spawner, GPS, gps_task(hw.gps_uart));
        spawn_tracked!(spawner, IMU, imu_task(hw.imu_i2c));
        spawn_tracked!(
            spawner,
            RADIO,
            radio_task(hw.radio_spi, hw.radio_cs, hw.radio_reset, hw.radio_dio0)
        );
        spawn_tracked!(spawner, BLINKY, blinky());

        // Run the "blockers" in a single task or right here
        spawn_tracked!(
            spawner,
            USB,
            usb_and_repl_task(hw.usb_device, hw.usb_logger_class, hw.usb_class)
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
    wifi_task_driver(pwr, spi).await
}

#[tracked_task]
#[embassy_executor::task]
pub async fn imu_task(
    i2c: embassy_rp::i2c::I2c<'static, embassy_rp::peripherals::I2C0, embassy_rp::i2c::Async>,
) -> ! {
    imu_task_driver(i2c).await
}

#[tracked_task]
#[embassy_executor::task]
pub async fn radio_task(
    spi: embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI1, embassy_rp::spi::Async>,
    cs: embassy_rp::gpio::Output<'static>,
    reset: embassy_rp::gpio::Output<'static>,
    dio0: embassy_rp::gpio::Input<'static>,
) -> ! {
    radio_task_driver(spi, cs, reset, dio0).await
}

#[tracked_task]
#[embassy_executor::task]
pub async fn sd_card_task(
    spi: embassy_rp::spi::Spi<'static, embassy_rp::peripherals::SPI0, embassy_rp::spi::Async>,
    cs: embassy_rp::gpio::Output<'static>,
) -> ! {
    sd_card_task_driver(spi, cs).await
}
