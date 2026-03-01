use embassy_futures::join::join3;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_rp::usb::Driver as UsbDriver;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config, UsbDevice};
use heapless::Vec;
use proc_macros::tracked_task;
use rocket_core::{error, info};
use static_cell::StaticCell;

pub struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => defmt::panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

/// Approximate memory cost (in bytes) of adding a CDC ACM class instance.
const CDC_ACM_COST: usize = 66;

/// Conservative estimate for base configuration and string descriptors.
const BASE_DESCRIPTOR_COST: usize = 64;

/// Total required buffer size for the configuration descriptor based on active classes.
const REQUIRED_CONFIG_DESC_SIZE: usize = BASE_DESCRIPTOR_COST + (CDC_ACM_COST * 2);

const CONFIG_DESCRIPTOR_BUF_SIZE: usize = 256;
const BOS_DESCRIPTOR_BUF_SIZE: usize = 256;
const CONTROL_BUF_SIZE: usize = 64;

// Compile-time assertion to prevent runtime panics if descriptor size exceeds buffer.
const _: () = assert!(
    REQUIRED_CONFIG_DESC_SIZE <= CONFIG_DESCRIPTOR_BUF_SIZE,
    "USB CONFIG_DESCRIPTOR buffer is too small for the requested classes!"
);

#[tracked_task]
#[embassy_executor::task]
pub async fn usb_and_repl_task(
    usb_device: &'static mut UsbDevice<'static, UsbDriver<'static, USB>>,
    mut class: CdcAcmClass<'static, UsbDriver<'static, USB>>,
    logger_class: CdcAcmClass<'static, UsbDriver<'static, USB>>,
) {
    let log_fut = embassy_usb_logger::with_class!(1024, log::LevelFilter::Debug, logger_class);
    let usb_fut = usb_device.run();
    let repl_fut = repl(&mut class);
    join3(log_fut, usb_fut, repl_fut).await;

    unreachable!();
}

/// Initializes the USB peripheral and sets up a serial class for the REPL and a logger class.
///
/// Returns a tuple containing the REPL serial class and a future that runs the USB stack.
pub fn setup_usb(
    usb_driver: Driver<'static, embassy_rp::peripherals::USB>,
) -> (
    CdcAcmClass<'static, Driver<'static, USB>>,
    CdcAcmClass<'static, Driver<'static, USB>>,
    &'static mut UsbDevice<'static, Driver<'static, USB>>,
) {
    // Create embassy-usb Config
    let mut config = Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    static CONFIG_DESCRIPTOR: StaticCell<[u8; CONFIG_DESCRIPTOR_BUF_SIZE]> = StaticCell::new();
    static BOS_DESCRIPTOR: StaticCell<[u8; BOS_DESCRIPTOR_BUF_SIZE]> = StaticCell::new();
    static CONTROL_BUF: StaticCell<[u8; CONTROL_BUF_SIZE]> = StaticCell::new();

    let mut builder = Builder::new(
        usb_driver,
        config,
        CONFIG_DESCRIPTOR.init([0; CONFIG_DESCRIPTOR_BUF_SIZE]),
        BOS_DESCRIPTOR.init([0; BOS_DESCRIPTOR_BUF_SIZE]),
        &mut [], // no msos descriptors
        CONTROL_BUF.init([0; CONTROL_BUF_SIZE]),
    );

    // Create classes on the builder.
    static STATE: StaticCell<State> = StaticCell::new();
    let class = CdcAcmClass::new(&mut builder, STATE.init(State::new()), 64);

    // Create a class for the logger
    static LOGGER_STATE: StaticCell<State> = StaticCell::new();
    let logger_class = CdcAcmClass::new(&mut builder, LOGGER_STATE.init(State::new()), 64);

    // Creates the logger and returns the logger future
    // Note: You'll need to use log::info! afterwards instead of info! for this to work (this also applies to all the other log::* macros)

    // let log_fut = embassy_usb_logger::with_class!(1024, log::LevelFilter::Debug, logger_class);

    // Build the builder.
    static DEVICE: StaticCell<embassy_usb::UsbDevice<'static, Driver<'static, USB>>> =
        StaticCell::new();
    let usb = DEVICE.init(builder.build());

    // Run the USB device.
    // let usb_fut = usb.run();

    // Return the class and a future that runs everything concurrently.
    (class, logger_class, usb)
}

async fn repl<'d, T: embassy_rp::usb::Instance + 'd>(
    serial: &mut CdcAcmClass<'d, embassy_rp::usb::Driver<'d, T>>,
) -> ! {
    loop {
        serial.wait_connection().await;
        info!("Command REPL Connected");
        let _ = run_repl(serial).await;
        info!("Command REPL Disconnected");
    }
}

// Run the Command REPL.
// Returns when the connection is lost.
// By separating the inner logic out, the Disconnected Error can be handled in the main loop.
async fn run_repl<'d, T: embassy_rp::usb::Instance + 'd>(
    class: &mut CdcAcmClass<'d, embassy_rp::usb::Driver<'d, T>>,
) -> Result<(), Disconnected> {
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
