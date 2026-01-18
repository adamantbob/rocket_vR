use crate::{Irqs, WifiResources, info};
use cyw43::{SpiBus, aligned_bytes};
use cyw43_pio::{PioSpi, RM2_CLOCK_DIVIDER};
use defmt::unwrap;
use embassy_executor::Spawner;
use embassy_rp::gpio::{Level, Output};
use embassy_rp::peripherals::{DMA_CH0, PIO0};
use embassy_rp::pio::Pio;
use proc_macros::tracked_task;
use static_cell::StaticCell;

// Creating this as a task to keep it independent and isolated
// from failures in the main loop.
#[tracked_task(crate::Wifi)]
#[embassy_executor::task]
async fn cyw43_task(runner: WifiRunner) -> ! {
    runner.run().await
}

// Type alias to keep the Main task signature readable
pub type WifiRunner =
    cyw43::Runner<'static, SpiBus<Output<'static>, PioSpi<'static, PIO0, 0, DMA_CH0>>>;

pub async fn init_and_spawn(
    spawner: Spawner,
    r: WifiResources,
    irqs: Irqs,
) -> (cyw43::Control<'static>, cyw43::NetDriver<'static>) {
    // 1. Load Firmware and NVRAM
    let fw = aligned_bytes!("../../firmware/cyw43-firmware/43439A0.bin");
    let clm = aligned_bytes!("../../firmware/cyw43-firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../../firmware/cyw43-firmware/nvram_rp2040.bin");

    let pwr = Output::new(r.pwr, Level::Low);
    let cs = Output::new(r.cs, Level::High);
    let mut pio = Pio::new(r.pio, irqs);

    let spi = PioSpi::new(
        &mut pio.common,
        pio.sm0,
        RM2_CLOCK_DIVIDER,
        pio.irq0,
        cs,
        r.dio,
        r.clk,
        r.dma,
    );

    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    // 2. Create the cyw43 instance
    let (device, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;

    // 3. Spawn the runner (Crucial: must happen before control.init)
    spawner.spawn(unwrap!(cyw43_task(runner)));

    // 4. Load CLM and set power management
    control.init(clm).await;

    control
        .set_power_management(cyw43::PowerManagementMode::PowerSave)
        .await;

    info!("Wifi Hardware Ready");
    (control, device)
}
