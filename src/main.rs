#![no_std]
#![no_main]

use cyw43::Control;
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::{info, panic, unwrap};
use embassy_executor::Spawner;
use embassy_futures::join::join;
use embassy_rp::Peri;
use embassy_rp::bind_interrupts;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::{InterruptHandler as PioInterruptHandler, Pio};
use embassy_rp::usb::{Driver, Instance, InterruptHandler as USBInterruptHandler};
use embassy_time::{Duration, Timer};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

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
bind_interrupts!(struct Irqs {
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_rp::init(Default::default());

    // CYW43 Wifi Chip Setup (Break this into it's own function?)
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

    let (mut class, usb_runner) = setup_usb(p.USB);

    join(core0_loop(control), join(usb_runner, core0_REPL(&mut class))).await;
}

async fn core0_loop<'a>(mut control: Control<'a>) -> ! {
    let delay = Duration::from_millis(250);
    loop {
        log::info!("led on!");
        control.gpio_set(0, true).await;
        Timer::after(delay).await;
        log::info!("led off!");
        control.gpio_set(0, false).await;
        Timer::after(delay).await;
    }
}

async fn core0_REPL<'d, T: Instance + 'd>(serial: &mut CdcAcmClass<'d, Driver<'d, T>>) -> ! {
    loop {
        serial.wait_connection().await;
        log::info!("Connected");
        let _ = echo(serial).await;
        log::info!("Disconnected");
    }
}

async fn echo<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}

// Return a future that setup the USB Peripheral for Logging and a interactive shell
fn setup_usb(
    usb: Peri<'static, USB>,
) -> (
    CdcAcmClass<'static, Driver<'static, USB>>,
    impl core::future::Future<Output = ()>,
) {
    let usb_driver = Driver::new(usb, Irqs);

    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();

    let mut builder = Builder::new(
        usb_driver,
        config,
        CONFIG_DESCRIPTOR.init([0; 256]),
        BOS_DESCRIPTOR.init([0; 256]),
        &mut [], // no msos descriptors
        CONTROL_BUF.init([0; 64]),
    );

    // Create classes on the builder.
    static STATE: StaticCell<State> = StaticCell::new();
    let class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), 64);

    // Create a class for the logger
    static LOGGER_STATE: StaticCell<State> = StaticCell::new();
    let logger_class = CdcAcmClass::new(&mut builder, LOGGER_STATE.init(State::new()), 64);

    // Creates the logger and returns the logger future
    // Note: You'll need to use log::info! afterwards instead of info! for this to work (this also applies to all the other log::* macros)
    let log_fut = embassy_usb_logger::with_class!(1024, log::LevelFilter::Info, logger_class);

    // Build the builder.
    static DEVICE: StaticCell<embassy_usb::UsbDevice<'static, Driver<'static, USB>>> =
        StaticCell::new();
    let usb = DEVICE.init(builder.build());

    // Run the USB device.
    let usb_fut = usb.run();

    // Return the class and a future that runs everything concurrently.
    (class, async move {
        join(usb_fut, log_fut).await;
    })
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}
