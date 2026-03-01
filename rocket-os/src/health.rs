use crate::panic::check_core_panic;
use defmt::error;
use embassy_time::Timer;
use rocket_core::utilization::TrackedExt;

pub mod stack;
pub mod utilization;

/// Dedicated task to monitor the "other" core for crashes.
#[embassy_executor::task(pool_size = 2)]
pub async fn panic_monitor_task(
    target_core: usize,
    utilization_id: rocket_core::utilization::TaskId,
) -> ! {
    let current_core = if target_core == 1 { 0 } else { 1 };
    async {
        loop {
            stack::sample_stack_usage(current_core);
            if let Some(report) = check_core_panic(target_core) {
                if report.message == "(Incomplete formatting)" {
                    error!("CORE {} PANIC DETECTED (Partial Sentinel)", target_core);
                } else {
                    error!(
                        "CORE {} PANIC DETECTED: {} at {}:{}",
                        report.core_id,
                        report.message.as_str(),
                        report.file.as_str(),
                        report.line
                    );
                }
            }
            Timer::after_millis(500).await;
        }
    }
    .tracked(utilization_id)
    .await
}
