use embassy_rp::peripherals::*;
use embassy_rp::{bind_interrupts, dma, i2c, pio, uart, usb};

// Re-export the resource mapping macro
pub use crate::assign_resources;

// Bind Interrupts to their appropriate interrupt handler
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
        scl: PIN_5,
        sda: PIN_4,
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
    RadioResources {
        spi: SPI1,
        miso: PIN_8,
        mosi: PIN_11,
        clk: PIN_10,
        cs: PIN_9,
        reset: PIN_7,
        dio0: PIN_6,
        dma_tx: DMA_CH5,
        dma_rx: DMA_CH6,
    }
}
