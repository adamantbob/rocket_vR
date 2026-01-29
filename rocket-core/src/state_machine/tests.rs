// tests/state_machine_tests.rs
#[cfg(test)]
mod tests {
    use crate::{
        FlightState, GPSData, IMUData, LAUNCH_VELOCITY_THRESHOLD_MMS, MIN_FLIGHT_ALTITUDE_MM,
        RocketStateMachine,
    };

    #[test]
    fn test_initial_state() {
        let sm = RocketStateMachine::new();
        assert_eq!(sm.state, FlightState::Initializing);
    }

    #[test]
    fn test_launch_detection() {
        let mut sm = RocketStateMachine::new();
        sm.state = FlightState::GroundIdle;
        sm.ground_level_mm = Some(0);

        let mut gps = GPSData::new();
        gps.fix_valid = true;
        gps.speed_mm_per_sec = LAUNCH_VELOCITY_THRESHOLD_MMS + 1000;
        gps.raw_alt_mm = 1000;

        sm.update(gps, IMUData::new());
        assert_eq!(sm.state, FlightState::PoweredFlight);
    }

    #[test]
    fn test_apogee_detection() {
        let mut sm = RocketStateMachine::new();
        sm.state = FlightState::Coasting;
        sm.safety_armed = true;
        sm.max_altitude_mm = 100_000; // 100m

        let mut gps = GPSData::new();
        gps.fix_valid = true;
        gps.raw_alt_mm = 97000; // 3m drop

        // Need 5 consecutive drops
        for _ in 0..6 {
            sm.update(gps, IMUData::new());
        }

        assert_eq!(sm.state, FlightState::ApogeeReached);
    }

    #[test]
    fn test_safety_arming() {
        let mut sm = RocketStateMachine::new();
        sm.state = FlightState::GroundIdle;
        sm.ground_level_mm = Some(0);

        let mut gps = GPSData::new();
        gps.raw_alt_mm = MIN_FLIGHT_ALTITUDE_MM + 1000;

        sm.update(gps, IMUData::new());
        assert!(sm.safety_armed);
    }
}
