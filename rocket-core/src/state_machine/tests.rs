// tests/state_machine_tests.rs
#[cfg(test)]
mod tests {
    use crate::{
        FlightController, FlightState, GPSData, IMUData, LAUNCH_VELOCITY_THRESHOLD_MMS,
        MIN_FLIGHT_ALTITUDE_MM,
    };

    #[test]
    fn test_initial_state() {
        let fc = FlightController::new();
        assert_eq!(fc.state, FlightState::Initializing);
    }

    #[test]
    fn test_launch_detection_velocity() {
        let mut fc = FlightController::new();
        fc.state = FlightState::GroundIdle;
        fc.ground_level_mm = Some(0);
        fc.accel_z_offset_m_g = 1000;

        let mut gps = GPSData::new();
        gps.fix_valid = true;
        gps.velocity_z_mms = LAUNCH_VELOCITY_THRESHOLD_MMS + 500;
        gps.raw_alt_mm = 1000;

        let mut imu = IMUData::new();
        imu.accel_high_g[2] = 1000; // Normal gravity

        // Need 2 sustained samples
        for _ in 0..2 {
            fc.update(gps, imu);
        }
        assert_eq!(fc.state, FlightState::PoweredFlight);
    }

    #[test]
    fn test_launch_detection_accel() {
        let mut fc = FlightController::new();
        fc.state = FlightState::GroundIdle;
        fc.ground_level_mm = Some(0);
        fc.accel_z_offset_m_g = 1000;

        let mut gps = GPSData::new();
        gps.fix_valid = true;
        gps.velocity_z_mms = 0;
        gps.raw_alt_mm = 0;

        let mut imu = IMUData::new();
        imu.accel_high_g[2] = 4000; // 4G total (3G net acceleration)

        // Need 2 sustained samples
        for _ in 0..2 {
            fc.update(gps, imu);
        }
        assert_eq!(fc.state, FlightState::PoweredFlight);
    }

    #[test]
    fn test_burnout_detection_accel() {
        let mut fc = FlightController::new();
        fc.state = FlightState::PoweredFlight;
        fc.last_velocity_z_mms = 50_000;
        fc.accel_z_offset_m_g = 1000;

        let mut gps = GPSData::new();
        gps.fix_valid = true;
        gps.velocity_z_mms = 50_000;

        let mut imu = IMUData::new();
        imu.accel_high_g[2] = 1200; // Only 0.2G net acceleration (burnout)

        // Need 11 sustained samples (> 10)
        for _ in 0..11 {
            fc.update(gps, imu);
        }
        assert_eq!(fc.state, FlightState::Coasting);
    }

    #[test]
    fn test_apogee_detection() {
        let mut fc = FlightController::new();
        fc.state = FlightState::Coasting;
        fc.safety_armed = true;
        fc.max_altitude_mm = 100_000; // 100m

        let mut gps = GPSData::new();
        gps.fix_valid = true;
        gps.raw_alt_mm = 96000; // 4m drop (threshold is 3m)

        // Need 11 sustained drops (> 10)
        for _ in 0..11 {
            fc.update(gps, IMUData::new());
        }

        assert_eq!(fc.state, FlightState::ApogeeReached);
    }

    #[test]
    fn test_safety_arming() {
        let mut fc = FlightController::new();
        fc.state = FlightState::GroundIdle;
        fc.ground_level_mm = Some(0);

        let mut gps = GPSData::new();
        gps.raw_alt_mm = MIN_FLIGHT_ALTITUDE_MM + 1000;

        fc.update(gps, IMUData::new());
        assert!(fc.safety_armed);
    }
}
