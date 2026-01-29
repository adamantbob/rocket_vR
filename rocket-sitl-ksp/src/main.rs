use krpc_client::{Client, services::space_center::SpaceCenter};
use rocket_core::{GPSData, IMUData, RocketStateMachine};
use std::thread::sleep;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    env_logger::init();
    println!("Connecting to KSP kRPC server...");

    // Connect using Client::new (Standard Ports: RPC=50000, Stream=50001)
    let client = Client::new("Rocket SITL Bridge", "127.0.0.1", 50000, 50001)?;
    let space_center = SpaceCenter::new(client);

    let vessel = space_center.get_active_vessel()?;
    println!("Connected to {}", vessel.get_name()?);

    let mut sm = RocketStateMachine::new();
    let rf = vessel.get_orbit()?.get_body()?.get_reference_frame()?;
    let flight = vessel.flight(Some(&rf))?;

    println!("SITL Bridge active. Calibrating ground level...");

    let mut calibration_samples = 0;
    let mut alt_sum = 0.0;

    while calibration_samples < 20 {
        let altitude = flight.get_mean_altitude()?;
        alt_sum += altitude;
        calibration_samples += 1;
        print!("\rCalibrating ground level... {}/20", calibration_samples);
        std::io::stdout().flush()?;
        sleep(Duration::from_millis(50));
    }

    let ground_level = alt_sum / 20.0;
    sm.ground_level_mm = Some((ground_level * 1000.0) as i32);
    sm.state = FlightState::GroundIdle;

    println!(
        "\nCalibration complete: {:.1}m. Ready for launch!",
        ground_level
    );

    loop {
        // 1. Gather Telemetry
        let altitude = flight.get_mean_altitude()?;
        let v_speed = flight.get_vertical_speed()?;
        let g_force = flight.get_g_force()?;

        // 2. Map to rocket-core types
        let mut gps_data = GPSData::new();
        gps_data.fix_valid = true;
        gps_data.raw_alt_mm = (altitude * 1000.0) as i32;
        gps_data.speed_mm_per_sec = (v_speed.abs() * 1000.0) as i32;

        let mut imu_data = IMUData::new();
        imu_data.accel_high_g[2] = (g_force * 1000.0) as i32;

        // 3. Update State Machine
        let old_state = sm.state;
        sm.update(gps_data, imu_data);

        if sm.state != old_state {
            println!("\nSTATE CHANGE: {:?} -> {:?}", old_state, sm.state);
        }

        // 4. Output Status
        print!(
            "\rAlt: {:.1}m | V-Speed: {:.1}m/s | State: {:?}",
            altitude, v_speed, sm.state
        );
        use std::io::{Write, stdout};
        stdout().flush()?;

        sleep(Duration::from_millis(100)); // 10Hz Update
    }
}
