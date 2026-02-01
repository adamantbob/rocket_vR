mod agent;

use agent::KspAgent;
use embassy_executor::Spawner;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Ticker};
use rocket_core::datacells::DataCell;
use rocket_core::{FlightAction, FlightController, FlightState, GPSData, IMUData};
use std::io::Write;

#[derive(Copy, Clone, Default)]
struct RawTelemetry {
    altitude: f64,
    v_speed: f64,
    g_force: f64,
}

#[derive(Copy, Clone, Default)]
struct StateSnapshot {
    altitude: f64,
    v_speed: f64,
    state: FlightState,
    telemetry_hz: f32,
    sm_hz: f32,
}

enum KspCommand {
    ActivateNextStage,
    DeployParachutes,
}

static TELEMETRY_CELL: DataCell<RawTelemetry> = DataCell::new(RawTelemetry {
    altitude: 0.0,
    v_speed: 0.0,
    g_force: 0.0,
});
static STATE_CELL: DataCell<StateSnapshot> = DataCell::new(StateSnapshot {
    altitude: 0.0,
    v_speed: 0.0,
    state: FlightState::Initializing,
    telemetry_hz: 0.0,
    sm_hz: 0.0,
});
static TELEMETRY_HZ_CELL: DataCell<f32> = DataCell::new(0.0);
static COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, KspCommand, 8> = Channel::new();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    env_logger::init();
    println!("Connecting to KSP kRPC server...");

    let agent = match KspAgent::new("Rocket SITL Bridge") {
        Ok(a) => a,
        Err(e) => {
            eprintln!("Failed to connect: {}", e);
            return;
        }
    };
    println!("Connected to {}", agent.get_vessel_name());

    // 1. Calibration (Blocking in main before tasks start)
    println!("SITL Bridge active. Calibrating ground level and orientation...");
    let mut calibration_samples = 0;
    let mut alt_sum = 0.0;
    let mut g_sum = 0.0;

    let mut agent = agent;
    while calibration_samples < 20 {
        let (altitude, _, g_force) = agent.poll_telemetry();
        alt_sum += altitude;
        g_sum += g_force;
        calibration_samples += 1;
        print!("\rCalibration: {}/20 samples...", calibration_samples);
        let _ = std::io::stdout().flush();
        std::thread::sleep(std::time::Duration::from_millis(50));
    }

    let ground_level = alt_sum / 20.0;
    let avg_g = g_sum / 20.0;
    let ground_level_mm = (ground_level * 1000.0) as i32;
    let avg_g_m_g = (avg_g * 1000.0) as i32;

    println!(
        "\nCalibration complete: Alt={:.1}m, G-Offset={}mG. Ready for launch!",
        ground_level, avg_g_m_g
    );

    println!("3... 2... 1... Ignition!");
    let _ = agent.activate_next_stage();

    // 2. Spawn Blocking KSP Thread
    std::thread::spawn(move || {
        ksp_polling_thread(agent);
    });

    // 3. Spawn Tasks
    spawner.spawn(state_machine_task(ground_level_mm, avg_g_m_g).unwrap());
    spawner.spawn(logging_task().unwrap());
}

fn ksp_polling_thread(mut agent: KspAgent) {
    let mut loop_count = 0;
    let tick_rate = std::time::Duration::from_millis(10);
    let mut next_tick = std::time::Instant::now();
    let mut last_hz_calc = std::time::Instant::now();

    loop {
        next_tick += tick_rate;
        loop_count += 1;

        if loop_count % 100 == 0 {
            let _ = agent.check_active_vessel();
            let elapsed = last_hz_calc.elapsed().as_secs_f32();
            if elapsed > 0.0 {
                TELEMETRY_HZ_CELL.update(100.0 / elapsed);
            }
            last_hz_calc = std::time::Instant::now();
        }

        // Process Commands from Channel
        while let Ok(cmd) = COMMAND_CHANNEL.try_receive() {
            match cmd {
                KspCommand::ActivateNextStage => {
                    let _ = agent.activate_next_stage();
                }
                KspCommand::DeployParachutes => {
                    let _ = agent.deploy_parachutes();
                }
            }
        }

        // Poll Telemetry
        let (altitude, v_speed, g_force) = agent.poll_telemetry();
        TELEMETRY_CELL.update(RawTelemetry {
            altitude,
            v_speed,
            g_force,
        });

        let now = std::time::Instant::now();
        if next_tick > now {
            std::thread::sleep(next_tick - now);
        }
    }
}

#[embassy_executor::task]
async fn state_machine_task(ground_level_mm: i32, accel_z_offset_m_g: i32) {
    let mut controller = FlightController::new();
    controller.set_ground_reference(ground_level_mm, accel_z_offset_m_g);

    let mut ticker = Ticker::every(Duration::from_millis(10));
    let start_time = Instant::now();
    let mut last_stage_fired = start_time;
    let mut last_hz_calc = Instant::now();
    let mut loop_cycles = 0;

    loop {
        ticker.next().await;
        loop_cycles += 1;

        let mut sm_hz = 0.0;
        if loop_cycles % 100 == 0 {
            let elapsed_micros = last_hz_calc.elapsed().as_micros();
            if elapsed_micros > 0 {
                sm_hz = 100_000_000.0 / elapsed_micros as f32; // (100 samples * 1e6) / micros
            }
            last_hz_calc = Instant::now();
        }

        let tel = TELEMETRY_CELL.read();

        // Map to rocket-core types
        let mut gps_data = GPSData::new();
        gps_data.fix_valid = true;
        gps_data.raw_alt_mm = (tel.altitude * 1000.0) as i32;
        gps_data.speed_mm_per_sec = (tel.v_speed.abs() * 1000.0) as i32;
        gps_data.velocity_z_mms = (tel.v_speed * 1000.0) as i32;
        gps_data.velocity_z_filt = tel.v_speed as f32;

        let mut imu_data = IMUData::new();
        imu_data.accel_high_g[2] = (tel.g_force * 1000.0) as i32;

        // Update Controller
        let old_state = controller.state;
        let (state, action) = controller.update(gps_data, imu_data);

        if state != old_state {
            println!(
                "\n[T+{:>7.2}s] STATE CHANGE: {:?} -> {:?}",
                (start_time.elapsed().as_micros() as f32 / 1_000_000.0),
                old_state,
                state
            );

            // Staging Logic using the decision from FlightController
            if start_time.elapsed() > Duration::from_millis(500)
                && last_stage_fired.elapsed() > Duration::from_millis(500)
            {
                match action {
                    FlightAction::StageNext => {
                        println!(
                            "[T+{:>7.2}s] [SITL] BURNOUT DETECTED. Firing Stage 2 (Coupler)...",
                            (start_time.elapsed().as_micros() as f32 / 1_000_000.0)
                        );
                        let _ = COMMAND_CHANNEL.try_send(KspCommand::ActivateNextStage);
                        last_stage_fired = Instant::now();
                    }
                    FlightAction::DeployParachutes => {
                        println!(
                            "[T+{:>7.2}s] [SITL] APOGEE REACHED. Firing Stage 3 (Parachute)...",
                            (start_time.elapsed().as_micros() as f32 / 1_000_000.0)
                        );
                        let _ = COMMAND_CHANNEL.try_send(KspCommand::DeployParachutes);
                        last_stage_fired = Instant::now();
                    }
                    FlightAction::None => {}
                }
            }
        }

        let tel_hz = TELEMETRY_HZ_CELL.read();
        let mut snap = StateSnapshot {
            altitude: tel.altitude,
            v_speed: tel.v_speed,
            state,
            telemetry_hz: tel_hz,
            sm_hz: STATE_CELL.read().sm_hz, // Keep old SM Hz if not updating this cycle
        };
        if loop_cycles % 100 == 0 {
            snap.sm_hz = sm_hz;
        }
        STATE_CELL.update(snap);
    }
}

#[embassy_executor::task]
async fn logging_task() {
    let mut ticker = Ticker::every(Duration::from_millis(100)); // 10Hz
    let mut last_hz_calc = Instant::now();
    let start_time = Instant::now();

    loop {
        ticker.next().await;

        let snap = STATE_CELL.read();
        let elapsed_secs = last_hz_calc.elapsed().as_micros() as f32 / 1_000_000.0;
        let log_hz = 1.0 / elapsed_secs;

        print!(
            "\r[T+{:>7.2}s] Alt: {:.1}m | V-Speed: {:.1}m/s | State: {:?} | Tel: {:.1}Hz | SM: {:.1}Hz | Log: {:.1}Hz",
            (start_time.elapsed().as_micros() as f32 / 1_000_000.0),
            snap.altitude,
            snap.v_speed,
            snap.state,
            snap.telemetry_hz,
            snap.sm_hz,
            log_hz
        );
        let _ = std::io::stdout().flush();
        last_hz_calc = Instant::now();
    }
}
