use crate::panic::check_core_panic;
use defmt::error;
use embassy_time::Timer;

/// Dedicated task to monitor the "other" core for crashes.
#[embassy_executor::task(pool_size = 2)]
pub async fn panic_monitor_task(target_core: usize) -> ! {
    loop {
        if let Some(report) = check_core_panic(target_core) {
            if report.message == "(Incomplete formatting)" {
                error!("CORE {} PANIC DETECTED (Partial Sentinel)", target_core);
            } else {
                error!(
                    "CORE {} PANIC DETECTED: {} at {}:{}",
                    target_core,
                    report.message.as_str(),
                    report.file.as_str(),
                    report.line
                );
            }
        }
        Timer::after_millis(500).await;
    }
}
