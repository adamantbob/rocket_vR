#![no_std]
#![no_main]

use cyw43::Control;
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::Peripherals;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver, Instance, InterruptHandler as USBInterruptHandler};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::CdcAcmClass;
use heapless::Vec;
use portable_atomic::{AtomicU64, Ordering};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

mod usb;

static IDLE_TICKS: AtomicU64 = AtomicU64::new(0);

#[embassy_executor::task]
async fn stats_task() {
    let mut last_idle = 0;
    let mut last_time = embassy_time::Instant::now();
    loop {
        embassy_time::Timer::after(Duration::from_secs(1)).await;
        let now = embassy_time::Instant::now();
        let idle = IDLE_TICKS.load(Ordering::Relaxed);

        let delta_idle = idle - last_idle;
        let delta_time = (now - last_time).as_ticks();

        let usage = 100.0 * (1.0 - (delta_idle as f32 / delta_time as f32));
        log::info!("CPU Usage: {}%", usage);

        last_idle = idle;
        last_time = now;
    }
}

// Program metadata for `picotool info`.
// This isn't needed, but it's recommended to have these minimal entries.
#[unsafe(link_section = ".bi_entries")]
#[used]
pub static PICOTOOL_ENTRIES: [embassy_rp::binary_info::EntryAddr; 4] = [
    embassy_rp::binary_info::rp_program_name!(c"Blinky Example"),
    embassy_rp::binary_info::rp_program_description!(
        c"This example tests the RP Pico 2 W's onboard LED, connected to GPIO 0 of the cyw43 \
        (WiFi chip) via PIO 0 over the SPI bus."
    ),
    embassy_rp::binary_info::rp_cargo_version!(),
    embassy_rp::binary_info::rp_program_build_attribute!(),
];

// Bind Interrupts to their appropriate interrupt handler
bind_interrupts!(pub struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    USBCTRL_IRQ => USBInterruptHandler<USB>;
});

// Creating this as a task to keep it independent and isolated
// from failures in the main loop.
#[embassy_executor::task]
async fn cyw43_task(
    runner: cyw43::Runner<
        'static,
        cyw43::SpiBus<Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>,
    >,
) -> ! {
    runner.run().await
}

#[cortex_m_rt::entry]
fn main() -> ! {
    let p = embassy_rp::init(Default::default());

    static EXECUTOR_DATA: StaticCell<embassy_executor::raw::Executor> = StaticCell::new();
    let executor = EXECUTOR_DATA.init(embassy_executor::raw::Executor::new(core::ptr::null_mut()));
    let spawner = executor.spawner();

    spawner.spawn(unwrap!(async_main(spawner, p)));
    spawner.spawn(unwrap!(stats_task()));

    loop {
        unsafe {
            executor.poll();
        }
        let start = embassy_time::Instant::now();
        cortex_m::asm::wfe();
        let end = embassy_time::Instant::now();
        IDLE_TICKS.fetch_add((end - start).as_ticks(), Ordering::Relaxed);
    }
}

#[embassy_executor::task]
async fn async_main(spawner: Spawner, p: Peripherals) {
    // CYW43 Wifi Chip Setup
    // Firmware files are expected in the 'firmware' directory at the project root.
    let fw = include_bytes!("../firmware/cyw43-firmware/43439A0.bin");
    let clm = include_bytes!("../firmware/cyw43-firmware/43439A0_clm.bin");

    let pwr = Output::new(p.PIN_23, Level::Low);
    let cs = Output::new(p.PIN_25, Level::High);
    let mut pio = Pio::new(p.PIO0, Irqs);
    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        RM2_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        p.PIN_24,
        p.PIN_29,
        p.DMA_CH0,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());
    let (_net_device, mut control, runner) = cyw43::new(state, pwr, spi, fw).await;

    // Spawn the cyw43 task.
    // In this configuration, spawner.spawn returns () while the task creation itself returns a Result.
    spawner.spawn(unwrap!(cyw43_task(runner)));

    control.init(clm).await;
    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    //-----------------
    // End CYW43 Setup
    //-----------------

    let (mut class, usb_runner) = usb::setup_usb(p.USB);

    join(
        core0_loop(control),
        join(usb_runner, core0_repl(&mut class)),
    )
    .await;
}

async fn core0_loop<'a>(mut control: Control<'a>) -> ! {
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
    loop {
        serial.wait_connection().await;
        log::info!("Command REPL Connected");
        let _ = run_repl(serial).await;
        log::info!("Command REPL Disconnected");
    }
}

// Run the Command REPL.
// Returns when the connection is lost.
// By separating the inner logic out, the Disconnected Error can be handled in the main loop.
async fn run_repl<'d, T: Instance + 'd>(
    class: &mut CdcAcmClass<'d, Driver<'d, T>>,
) -> Result<(), usb::Disconnected> {
    let mut rx_buf = [0; 64];
    let mut command_buf = Vec::<u8, 64>::new();
    const BACKSPACE: u8 = 0x08; // \b
    loop {
        let n = class.read_packet(&mut rx_buf).await?;
        // if the buffer contains a newline or a return, we have a command
        if rx_buf[..n].iter().any(|&b| b == b'\n' || b == b'\r') {
            if let Ok(s) = core::str::from_utf8(command_buf.as_slice()) {
                log::info!("command: {}", s);
            } else {
                log::info!("command (raw): {:?}", command_buf);
            }
            // echo the command back to the client
            class.write_packet(b"\r\n").await?;
            class.write_packet(command_buf.as_slice()).await?;
            class.write_packet(b" received\r\n").await?;
            command_buf.clear();
        // if a backspace is received, remove the last character.
        // Note: This is intended to be used with a user terminal which sends one character at a time.
        // If somehow multiple backspaces are received, it will remove only a single character.
        } else if rx_buf[..n].contains(&BACKSPACE) {
            command_buf.pop();
        // if the buffer is full, clear it
        } else if command_buf.extend_from_slice(&rx_buf[..n]).is_err() {
            log::error!("command_buf overflow");
            class.write_packet(b"command_buf overflow\n\r").await?;
            command_buf.clear();
        }
        // echo the received data back to the client (this is so it appears on the terminal)
        class.write_packet(&rx_buf[..n]).await?;
    }
}
