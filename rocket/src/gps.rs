use crate::Irqs;
use embassy_rp::Peri;
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::uart::{
    BufferedInterruptHandler, BufferedUart, BufferedUartRx, Config, Instance, RxPin, TxPin,
};
use embassy_time::Timer;
use embedded_io_async::{Read, Write};
use proc_macros::tracked_task;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

pub struct GPS {}

impl GPS {
    pub async fn run<T: Instance>(
        _uart: Peri<'static, T>,
        tx: Peri<'static, impl TxPin<T>>,
        rx: Peri<'static, impl RxPin<T>>,
        _irq: impl Binding<T::Interrupt, BufferedInterruptHandler<T>>,
    ) -> ! {
        loop {
            Timer::after_secs(1).await;
        }
    }
}
