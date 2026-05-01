use embassy_rp::gpio::{Input, Level, Output, Pull};
use embassy_rp::i2c::{Async as I2cAsync, Config as I2cConfig, I2c};
use embassy_rp::peripherals::*;
use embassy_rp::spi::{Async as SpiAsync, Config as SpiConfig, Spi};
use embassy_rp::{bind_interrupts, dma, i2c, usb};
// use embassy_rp::uart::{BufferedUart, Config as UartConfig}; // GPS is now I2C, not UART
// use embassy_rp::pio::Pio;                                    // No WiFi on this board
use embassy_rp::usb::Driver as UsbDriver;
use embassy_usb::UsbDevice;
use embassy_usb::class::cdc_acm::CdcAcmClass;
// use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};                 // No WiFi on this board

use rocket_core::assign_resources;

bind_interrupts!(pub struct Irqs {
    USBCTRL_IRQ => usb::InterruptHandler<USB>;
    I2C0_IRQ    => i2c::InterruptHandler<I2C0>;
    DMA_IRQ_0   => dma::InterruptHandler<DMA_CH0>,
                   dma::InterruptHandler<DMA_CH1>;
    // PIO0_IRQ_0 => pio::InterruptHandler<PIO0>;               // WiFi — not on this board
    // UART0_IRQ  => uart::BufferedInterruptHandler<UART0>;      // GPS is I2C on this board
});

assign_resources! {
    UsbR {
        usb: USB
    }
    LedsR {
        green:  PIN_29,   // board.A3 — was PIN_4 (yellow)
        yellow: PIN_4,    // GP4 — was green
        red:    PIN_3,    // GP3
    }

    // --- Pyro channels ---
    // pyro_0–3 → board.GP10–GP13
    PyrosR {
        ch0: PIN_10,
        ch1: PIN_11,
        ch2: PIN_12,
        ch3: PIN_13,
    }

    // --- Shared SPI bus: RFM9x radio + W25Q16 flash ---
    // Both devices share SPI0 (GP18/GP19/GP16) with separate CS pins.
    SpiSharedR {
        spi:    SPI0,
        clk:    PIN_18,    // board.SCK  / GP18
        mosi:   PIN_19,    // board.MOSI / GP19
        miso:   PIN_16,    // board.MISO / GP16
        dma_tx: DMA_CH0,
        dma_rx: DMA_CH1,
    }

    // --- Radio RFM9x CS + Reset ---
    // radio_cs    → board.D0 — TODO: confirm D0 == GP0 on this board
    // radio_reset → board.D2 — TODO: confirm D2 == GP2 on this board
    RadioR {
        cs:    PIN_0,    // board.D0
        reset: PIN_2,    // board.D2
    }

    // --- Flash W25Q16 CS ---
    // flash_cs → board.GP21
    FlashR {
        cs: PIN_21,    // GP21
    }

    // --- Sensor I2C bus ---
    // Devices: BMP3XX (0x76), BMP280, ADXL375, BNO055, GPS (I2C)
    // board.SCL / board.SDA — TODO: confirm exact pins for this board.
    // WARNING: GP4 (default SDA on Pico) conflicts with led_green (GP4).
    //          Verify with schematic before enabling. Placeholder: SCL=PIN_5, SDA=PIN_20.
    SensorsR {
        i2c: I2C0,
        scl: PIN_5,     // board.SCL — TODO: verify
        sda: PIN_20,    // board.SDA — TODO: verify (GP4 conflicts with led_green)
    }

    // --- Misc GPIO ---
    // buzzer           → board.D4  (PWM) — TODO: confirm pin; possibly GP4 (conflicts with led_green)
    // camera_power     → board.D12 — TODO: confirm; possibly GP12 (conflicts with pyro_2)
    // deployment_check → board.D25 — TODO: confirm; possibly GP25
    MiscR {
        deployment_check: PIN_25,   // board.D25 — tentatively GP25, verify
        camera_power:     PIN_14,   // board.D12 — TODO: replace PIN_14 with correct GPIO
        // buzzer: PIN_??,          // board.D4  — TODO: add when pin confirmed; needs PWM setup
    }
}

pub struct Hardware {
    // LEDs
    pub led_green: Output<'static>,
    pub led_yellow: Output<'static>,
    pub led_red: Output<'static>,

    // Pyros (active-low safe, driven HIGH to fire)
    pub pyro_0: Output<'static>,
    pub pyro_1: Output<'static>,
    pub pyro_2: Output<'static>,
    pub pyro_3: Output<'static>,

    // Shared SPI bus (RFM9x radio + W25Q16 flash share SPI0)
    pub spi_shared: Spi<'static, SPI0, SpiAsync>,

    // Radio RFM9x
    pub radio_cs: Output<'static>,
    pub radio_reset: Output<'static>,

    // Flash W25Q16
    pub flash_cs: Output<'static>,

    // Sensor I2C (BMP3XX, BMP280, ADXL375, BNO055, GPS)
    pub sensor_i2c: I2c<'static, I2C0, I2cAsync>,

    // Misc
    pub deployment_check: Input<'static>,
    pub camera_power: Output<'static>,
    // pub buzzer: ???,   // TODO: PWM driver

    // --- Commented out from proto-rocket (not present on this board) ---
    pub usb_device: &'static mut UsbDevice<'static, UsbDriver<'static, USB>>,
    pub usb_logger_class: CdcAcmClass<'static, UsbDriver<'static, USB>>,
    pub usb_class: CdcAcmClass<'static, UsbDriver<'static, USB>>,
    // pub core1: embassy_rp::Peri<'static, CORE1>,       // No Core 1 usage yet
    // pub gps_uart: BufferedUart,                         // GPS is now I2C on this board
    // pub wifi_pwr: Output<'static>,                      // No WiFi on this board
    // pub wifi_spi: PioSpi<'static, PIO0, 0>,             // No WiFi on this board
}

impl Hardware {
    pub fn init(p: embassy_rp::Peripherals) -> Self {
        let r = AssignedResources::take(p);

        // --- LEDs ---
        let led_green = Output::new(r.LedsR.green, Level::Low);
        let led_yellow = Output::new(r.LedsR.yellow, Level::Low);
        let led_red = Output::new(r.LedsR.red, Level::Low);

        // --- Pyros (default LOW = safe/disarmed) ---
        let pyro_0 = Output::new(r.PyrosR.ch0, Level::Low);
        let pyro_1 = Output::new(r.PyrosR.ch1, Level::Low);
        let pyro_2 = Output::new(r.PyrosR.ch2, Level::Low);
        let pyro_3 = Output::new(r.PyrosR.ch3, Level::Low);

        // --- Shared SPI bus (RFM9x + W25Q16 flash, SPI0 @ 1 MHz) ---
        let mut spi_config = SpiConfig::default();
        spi_config.frequency = 1_000_000;
        let spi_shared = Spi::new(
            r.SpiSharedR.spi,
            r.SpiSharedR.clk,
            r.SpiSharedR.mosi,
            r.SpiSharedR.miso,
            r.SpiSharedR.dma_tx,
            r.SpiSharedR.dma_rx,
            Irqs,
            spi_config,
        );

        // --- Radio CS + Reset (deasserted = HIGH) ---
        let radio_cs = Output::new(r.RadioR.cs, Level::High);
        let radio_reset = Output::new(r.RadioR.reset, Level::High);

        // --- Flash CS (deasserted = HIGH) ---
        let flash_cs = Output::new(r.FlashR.cs, Level::High);

        // --- Sensor I2C @ 400 kHz ---
        let mut i2c_config = I2cConfig::default();
        i2c_config.frequency = 400_000;
        let sensor_i2c = I2c::new_async(
            r.SensorsR.i2c,
            r.SensorsR.scl,
            r.SensorsR.sda,
            Irqs,
            i2c_config,
        );

        // --- Misc GPIO ---
        let deployment_check = Input::new(r.MiscR.deployment_check, Pull::Up);
        let camera_power = Output::new(r.MiscR.camera_power, Level::Low);
        // TODO: buzzer PWM setup

        // Setup USB
        let usb_driver = UsbDriver::new(r.UsbR.usb, Irqs);
        let (usb_class, usb_logger_class, usb_device) = rocket_drivers::usb::setup_usb(usb_driver);

        // Core 1
        // let core1 = r.CoreR.core1.into();
        // GPS was UART in proto-rocket, now I2C — no uart init needed
        // let wifi_pwr = Output::new(r.WifiR.pwr, Level::Low);
        // let wifi_cs  = Output::new(r.WifiR.cs,  Level::High);
        // let mut pio  = Pio::new(r.WifiR.pio, Irqs);
        // let wifi_spi = PioSpi::new(...);

        Self {
            usb_class,
            usb_logger_class,
            usb_device,
            led_green,
            led_yellow,
            led_red,
            pyro_0,
            pyro_1,
            pyro_2,
            pyro_3,
            spi_shared,
            radio_cs,
            radio_reset,
            flash_cs,
            sensor_i2c,
            deployment_check,
            camera_power,
        }
    }
}
