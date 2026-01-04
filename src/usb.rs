use crate::Irqs;
use embassy_rp::Peri;
use embassy_rp::peripherals::USB;
use embassy_rp::usb::Driver;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use embassy_usb::{Builder, Config};
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

// Return a future that setup the USB Peripheral for Logging and a interactive shell
pub fn setup_usb(
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
        embassy_futures::join::join(usb_fut, log_fut).await;
    })
}
