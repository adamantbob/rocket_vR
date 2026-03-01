use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{Async as I2cAsync, Config as I2cConfig, I2c};
use embassy_rp::peripherals::*;
use embassy_rp::pio::Pio;
use embassy_rp::spi::{Async as SpiAsync, Config as SpiConfig, Spi};
use embassy_rp::uart::{BufferedUart, Config as UartConfig};
use embassy_rp::usb::Driver as UsbDriver;
use embassy_rp::{bind_interrupts, dma, i2c, pio, uart, usb};
use embassy_usb::UsbDevice;
use embassy_usb::class::cdc_acm::CdcAcmClass;

use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use static_cell::StaticCell;

use rocket_core::assign_resources;

bind_interrupts!(pub struct Irqs {
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
    I2C0_IRQ => i2c::InterruptHandler<I2C0>;
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>,
                 dma::InterruptHandler<DMA_CH3>,
                 dma::InterruptHandler<DMA_CH4>,
                 dma::InterruptHandler<DMA_CH5>,
                 dma::InterruptHandler<DMA_CH6>;
});

assign_resources! {
    GpsR {
        uart: UART0,
        tx: PIN_12,
        rx: PIN_13
    }
    WifiR {
        pwr: PIN_23,
        cs: PIN_25,
        dio: PIN_24,
        clk: PIN_29,
        pio: PIO0,
        dma: DMA_CH0
    }
    UsbR {
        usb: USB
    }
    ImuR {
        i2c: I2C0,
        scl: PIN_5,
        sda: PIN_4
    }
    SdcardR {
        spi: SPI0,
        miso: PIN_16,
        mosi: PIN_19,
        clk: PIN_18,
        cs: PIN_17,
        dma_tx: DMA_CH3,
        dma_rx: DMA_CH4
    }
    CoreR {
        core1: CORE1
    }
    RadioR {
        spi: SPI1,
        miso: PIN_8,
        mosi: PIN_11,
        clk: PIN_10,
        cs: PIN_9,
        reset: PIN_7,
        dio0: PIN_6,
        dma_tx: DMA_CH5,
        dma_rx: DMA_CH6
    }
}

pub struct Hardware {
    pub usb_device: &'static mut UsbDevice<'static, UsbDriver<'static, USB>>,
    pub usb_logger_class: CdcAcmClass<'static, UsbDriver<'static, USB>>,
    pub usb_class: CdcAcmClass<'static, UsbDriver<'static, USB>>,
    pub core1: embassy_rp::Peri<'static, CORE1>,
    pub gps_uart: BufferedUart,
    pub imu_i2c: I2c<'static, I2C0, I2cAsync>,
    pub sd_spi: Spi<'static, SPI0, SpiAsync>,
    pub sd_cs: Output<'static>,
    pub radio_spi: Spi<'static, SPI1, SpiAsync>,
    pub radio_cs: Output<'static>,
    pub radio_reset: Output<'static>,
    pub radio_dio0: Input<'static>,
    pub wifi_pwr: Output<'static>,
    pub wifi_spi: PioSpi<'static, PIO0, 0>,
}

impl Hardware {
    pub fn init(p: embassy_rp::Peripherals) -> Self {
        let r = AssignedResources::take(p);

        // USB
        let usb = r.UsbR.usb.into();

        // Setup USB
        let usb_driver = UsbDriver::new(usb, Irqs);
        let (mut usb_class, usb_logger_class, usb_device) =
            rocket_drivers::usb::setup_usb(usb_driver);

        // Core 1
        let core1 = r.CoreR.core1.into();

        // GPS
        static GPS_TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        static GPS_RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
        let gps_uart = BufferedUart::new(
            r.GpsR.uart,
            r.GpsR.tx,
            r.GpsR.rx,
            Irqs,
            GPS_TX_BUF.init([0; 64]),
            GPS_RX_BUF.init([0; 256]),
            UartConfig::default(),
        );

        // IMU
        let mut imu_config = I2cConfig::default();
        imu_config.frequency = 400_000;
        let imu_i2c = I2c::new_async(r.ImuR.i2c, r.ImuR.scl, r.ImuR.sda, Irqs, imu_config);

        // SD Card
        let mut sd_config = SpiConfig::default();
        sd_config.frequency = 400_000;
        let sd_spi = Spi::new(
            r.SdcardR.spi,
            r.SdcardR.clk,
            r.SdcardR.mosi,
            r.SdcardR.miso,
            r.SdcardR.dma_tx,
            r.SdcardR.dma_rx,
            Irqs,
            sd_config,
        );
        let sd_cs = Output::new(r.SdcardR.cs, Level::High);

        // Radio
        let mut radio_config = SpiConfig::default();
        radio_config.frequency = 1_000_000;
        let radio_spi = Spi::new(
            r.RadioR.spi,
            r.RadioR.clk,
            r.RadioR.mosi,
            r.RadioR.miso,
            r.RadioR.dma_tx,
            r.RadioR.dma_rx,
            Irqs,
            radio_config,
        );
        let radio_cs = Output::new(r.RadioR.cs, Level::High);
        let radio_reset = Output::new(r.RadioR.reset, Level::High);
        let radio_dio0 = Input::new(r.RadioR.dio0, Pull::None);

        // Wifi
        let wifi_pwr = Output::new(r.WifiR.pwr, Level::Low);
        let wifi_cs = Output::new(r.WifiR.cs, Level::High);
        let mut pio = Pio::new(r.WifiR.pio, Irqs);
        let wifi_spi = PioSpi::new(
            &mut pio.common,
            pio.sm0,
            RM2_CLOCK_DIVIDER,
            pio.irq0,
            wifi_cs,
            r.WifiR.dio,
            r.WifiR.clk,
            embassy_rp::dma::Channel::new(r.WifiR.dma, Irqs),
        );

        Self {
            usb_class,
            usb_logger_class,
            usb_device,
            core1,
            gps_uart,
            imu_i2c,
            sd_spi,
            sd_cs,
            radio_spi,
            radio_cs,
            radio_reset,
            radio_dio0,
            wifi_pwr,
            wifi_spi,
        }
    }
}
