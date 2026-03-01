use cyw43::aligned_bytes;
use cyw43_pio::PioSpi;
use embassy_futures::join::join;
use embassy_rp::gpio::Output;
use embassy_time::Instant;
use rocket_core::WifiHealth;
use rocket_core::blackboard::SYSTEM_HEALTH;
use rocket_core::info;
use static_cell::StaticCell;

// Communication Channels
use crate::channels::LED_CHANNEL;

/// Represents the desired state of the CYW43 onboard LED.
#[derive(Debug, Clone, Copy, defmt::Format)]
pub enum LedState {
    On,
    Off,
    // Toggle,
}

impl LedState {
    /// Utility method to send a state command to the wifi_task via the global channel.
    pub async fn send(&self) {
        LED_CHANNEL.send(*self).await;
    }
}

/// The primary Wi-Fi management task.
///
/// This task is responsible for:
/// 1. Initializing the CYW43 hardware (SpiBus, Firmware, CLM).
/// 2. Running the background driver (`runner.run()`).
/// 3. Listening for LED state changes via `LED_CHANNEL`.
///
/// By joining the runner and the receiver, we ensure the driver stays alive
/// while we process hardware commands.
///
/// Note: This task is monolithic as the "cyw43::Control" object cannot be
/// shared across tasks. A downside here is that if a task manager is written
/// in the future, if this task fails and needs to be restarted it will take
/// a significant amount of time to reinitialize the hardware.
///
/// Init takes about 1s but shouldn't bog down the cpu too much.
/// ~7% cpu usage during init for 1s.
pub async fn wifi_task_driver(
    pwr: Output<'static>,
    spi: PioSpi<'static, embassy_rp::peripherals::PIO0, 0>,
) -> ! {
    // --- 1. Hardware & Firmware Setup ---

    // Firmware and NVRAM blobs must be 4-byte aligned for the DMA to read them.
    let fw = aligned_bytes!("../../firmware/cyw43-firmware/43439A0.bin");
    let clm = aligned_bytes!("../../firmware/cyw43-firmware/43439A0_clm.bin");
    let nvram = aligned_bytes!("../../firmware/cyw43-firmware/nvram_rp2040.bin");

    // Driver state requires a 'static lifetime.
    static STATE: StaticCell<cyw43::State> = StaticCell::new();
    let state = STATE.init(cyw43::State::new());

    // --- 2. Driver Initialization ---

    // Create the device (NetDriver), control handle, and background runner.
    // Note: 'device' is required if you intend to initialize the embassy-net stack.
    let (_device, mut control, runner) = cyw43::new(state, pwr, spi, fw, nvram).await;

    // --- 3. Concurrent Logic Definition ---

    // This sub-future handles logic that requires the CYW43 runner to be active.
    let signal_receiver = async {
        // Initialization commands (CLM loading) require the runner to be polling
        // the SPI bus, so they must be performed within the joined future.
        control.init(clm).await;
        control
            .set_power_management(cyw43::PowerManagementMode::PowerSave)
            .await;
        info!("Wifi Hardware Ready and Initialized");

        // let mut led_on = false;
        loop {
            SYSTEM_HEALTH.wifi_health.update(WifiHealth {
                tickstamp: Instant::now().as_ticks() as u64,
                initialized: true,
            });
            // Wait for a message from the rest of the application.
            let state = LED_CHANNEL.receive().await;

            match state {
                LedState::On => {
                    control.gpio_set(0, true).await;
                    // led_on = true;
                }
                LedState::Off => {
                    control.gpio_set(0, false).await;
                    // led_on = false;
                } // LedState::Toggle => {
                  //     led_on = !led_on;
                  //     control.gpio_set(0, led_on).await;
                  // }
            }
        }
    };

    // --- 4. Main Execution Loop ---

    // Join both futures. runner.run() handles the low-level Wi-Fi chip communication
    // while signal_receiver handles our application-level GPIO requests.
    join(runner.run(), signal_receiver).await;

    unreachable!()
}
