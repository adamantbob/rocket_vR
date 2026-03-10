//! Hardware resource definitions for the base station.
//!
//! Targets the integrated **Adafruit Feather RP2040 RFM95** board.
//!
//! # Pin Assignments (Integrated Board)
//!
//! | Function          | GPIO  | Notes                               |
//! |-------------------|-------|-------------------------------------|
//! | Radio SPI1 SCK    | 14    | Internal SPI1 SCK                   |
//! | Radio SPI1 MOSI   | 15    | Internal SPI1 MOSI                  |
//! | Radio SPI1 MISO   | 8     | Internal SPI1 MISO                  |
//! | Radio CS          | 16    | RFM_CS                              |
//! | Radio RESET       | 17    | RFM_RST                             |
//! | Radio DIO0        | 21    | RFM_IO0 (Interrupt)                 |
//! | NeoPixel Data     | 4     | Status NeoPixel                     |

use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::peripherals::*;
use embassy_rp::pio::Pio;
use embassy_rp::spi::{Async as SpiAsync, Config as SpiConfig, Spi};
use embassy_rp::usb::Driver as UsbDriver;
use embassy_rp::{bind_interrupts, dma, pio, usb};
use embassy_usb::UsbDevice;
use embassy_usb::class::cdc_acm::CdcAcmClass;

use rocket_core::assign_resources;

bind_interrupts!(pub struct Irqs {
    // Required for PIO-based WS2812 NeoPixel driver.
    PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;
    // DMA channels used by radio SPI and NeoPixel.
    DMA_IRQ_0 => dma::InterruptHandler<DMA_CH0>,
               dma::InterruptHandler<DMA_CH1>,
               dma::InterruptHandler<DMA_CH2>;
    USBCTRL_IRQ => embassy_rp::usb::InterruptHandler<USB>;
});

assign_resources! {
    RadioR {
        spi: SPI1,
        miso: PIN_8,
        mosi: PIN_15,
        clk: PIN_14,
        cs: PIN_16,
        reset: PIN_17,
        dio0: PIN_21,
        dma_tx: DMA_CH0,
        dma_rx: DMA_CH1
    }
    NeopixelR {
        data: PIN_4,
        pio: PIO0,
        dma: DMA_CH2
    }
    UsbR {
        usb: USB,
    }
}

/// Initialised hardware peripherals, ready to be handed to tasks.
pub struct Hardware {
    /// Async SPI bus for the RFM95 radio.
    pub radio_spi: Spi<'static, SPI1, SpiAsync>,
    /// Chip-select output for the RFM95 (active low, idle high).
    pub radio_cs: Output<'static>,
    /// Hardware reset line for the RFM95 (active low).
    pub radio_reset: Output<'static>,
    /// DIO0 interrupt input — signals RX-done / TX-done.
    pub radio_dio0: Input<'static>,
    /// PIO block for WS2812 (caller destructures into common/sm0).
    pub neo_pio: Pio<'static, PIO0>,
    /// DMA channel for the WS2812 PIO program.
    pub neo_dma: embassy_rp::Peri<'static, DMA_CH2>,
    /// Data pin for the WS2812 NeoPixel.
    pub neo_data: embassy_rp::Peri<'static, PIN_4>,
    /// USB driver.
    pub usb_device: &'static mut UsbDevice<'static, UsbDriver<'static, USB>>,
    /// USB logger class for logging.
    pub usb_logger_class: CdcAcmClass<'static, UsbDriver<'static, USB>>,
    /// USB class for REPL.
    pub usb_class: CdcAcmClass<'static, UsbDriver<'static, USB>>,
}

impl Hardware {
    pub fn init(p: embassy_rp::Peripherals) -> Self {
        let r = AssignedResources::take(p);

        // ----------------------------------------------------------------
        // Radio — Internal SPI1 @ 1 MHz
        // ----------------------------------------------------------------
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

        let usb_driver = UsbDriver::new(r.UsbR.usb, Irqs);
        let (usb_class, usb_logger_class, usb_device) = rocket_drivers::usb::setup_usb(usb_driver);

        // ----------------------------------------------------------------
        // NeoPixel — Hand PIO resources to the task.
        // ----------------------------------------------------------------
        let neo_pio = Pio::new(r.NeopixelR.pio, Irqs);

        Self {
            radio_spi,
            radio_cs,
            radio_reset,
            radio_dio0,
            neo_pio,
            neo_dma: r.NeopixelR.dma,
            neo_data: r.NeopixelR.data,
            usb_device: usb_device,
            usb_logger_class: usb_logger_class,
            usb_class: usb_class,
        }
    }
}
