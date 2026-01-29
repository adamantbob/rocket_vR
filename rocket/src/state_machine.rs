use crate::datacells::{DataCell, FlightState, PersistentData};
use crate::{StateMachine, error, info};
use embassy_time::{Duration, Instant, Ticker, Timer};
use heapless::Vec;
use proc_macros::tracked_task;
use rocket_core::{GPSData, GPSHealth, IMUData, IMUHealth, RocketStateMachine};

/// The Global Blackboard
pub struct SensorData {
    pub gps: DataCell<GPSData>,
    pub gps_health: DataCell<GPSHealth>,
    pub imu: DataCell<IMUData>,
    pub imu_health: DataCell<IMUHealth>,
}

pub static SENSOR_DATA: SensorData = SensorData {
    imu: DataCell::new(IMUData::new()),
    imu_health: DataCell::new(IMUHealth::new()),
    gps: DataCell::new(GPSData::new()),
    gps_health: DataCell::new(GPSHealth::new()),
};

// State Machine Task.
// This task is designed to run at a higher priority and preempt other tasks
// to ensure deadlines are met.
#[tracked_task(StateMachine)]
#[embassy_executor::task]
pub async fn state_machine_task() {
    let mut sm = RocketStateMachine::new();
    // Create a ticker set to 100Hz (10ms period)
    let mut ticker = Ticker::every(Duration::from_hz(100));
    // We track the expected time of the NEXT tick
    let mut next_expected_tick = Instant::now() + Duration::from_hz(100);
    // Keep track of the last seen GPS time
    let mut last_seen_gps_time = 0u32;

    // --- ATTEMPT RECOVERY ---
    // This is here if the code crashes a trips the watchdog mid-flight.
    // It will resume the flight from where it left off.
    if let Some(recovered) = crate::datacells::try_recover_flight_data() {
        sm.ground_level_mm = Some(recovered.ground_level);
        sm.state = recovered.state;
        info!("Warm start! Resumed at state: {:?}", sm.state);
    } else {
        // --- EXPLICIT CALIBRATION SCOPE ---
        // Here the calibration runs in a dedicated scope
        // to ensure the memory scope is as small as possible.
        {
            let mut calibration_buffer: Vec<i32, 10> = Vec::new();
            info!("GPS Warm-up: Waiting for 10 unique samples...");

            while !calibration_buffer.is_full() {
                let gps = SENSOR_DATA.gps.read();

                if gps.fix_valid && gps.utc_time_secs != last_seen_gps_time {
                    let _ = calibration_buffer.push(gps.raw_alt_mm);
                    last_seen_gps_time = gps.utc_time_secs;
                    info!("Calibrating... {}/10", calibration_buffer.len());
                }
                // We still yield to the executor so other tasks (like GPS) can run
                Timer::after_millis(50).await;
            }

            let sum: i32 = calibration_buffer.iter().sum();
            sm.ground_level_mm = Some(sum / 10);

            sm.state = FlightState::GroundIdle;
            info!("Calibration complete. Transitioned to GroundIdle.");
        }
        // After calibration is done, initialize the persistence
        let initial_data = PersistentData {
            ground_level: sm.ground_level_mm.unwrap(),
            state: FlightState::GroundIdle,
        };
        crate::datacells::save_flight_data(initial_data);
    }

    // --- MAIN FLIGHT LOOP ---
    loop {
        // 1. Record when we actually started this loop
        let _start_time = Instant::now();

        // 2. Perform the logic
        let gps = SENSOR_DATA.gps.read();
        let imu = SENSOR_DATA.imu.read();
        sm.update(gps, imu);
        // In your loop, update this every time the state changes:

        let current_data = PersistentData {
            ground_level: sm.ground_level_mm.unwrap(),
            state: sm.state,
        };
        crate::datacells::save_flight_data(current_data);

        // 3. Wait for the ticker
        ticker.next().await;

        // 4. CRITICAL CHECK: Did we finish too late?
        // If the current time is significantly past when we wanted to wake up,
        // it means sm.update() or an interrupt took too long.
        let now = Instant::now();
        if now > next_expected_tick + Duration::from_micros(500) {
            // 0.5ms grace period
            let lateness = now - next_expected_tick;
            error!(
                "LOOP OVERRUN: State machine is late by {}us!",
                lateness.as_micros()
            );
            // You could set a flag here to be sent over telemetry
        }

        // Advance our internal clock for the next check
        next_expected_tick += Duration::from_hz(100);
    }
}
