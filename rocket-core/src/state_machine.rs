// state_machine.rs
use crate::gps::types::GPSData;
use crate::imu::types::IMUData;
use crate::types::FlightState;

pub const MIN_FLIGHT_ALTITUDE_MM: i32 = 50_000; // 50 Meters
pub const LAUNCH_VELOCITY_THRESHOLD_MMS: i32 = 15_000; // 15 m/s

pub struct RocketStateMachine {
    pub state: FlightState,
    pub last_altitude_mm: i32,
    pub max_altitude_mm: i32,
    pub apogee_detect_count: u8,
    pub safety_armed: bool,
    pub ground_level_mm: Option<i32>,
}

impl RocketStateMachine {
    pub const fn new() -> Self {
        Self {
            state: FlightState::Initializing,
            last_altitude_mm: 0,
            max_altitude_mm: 0,
            apogee_detect_count: 0,
            safety_armed: false,
            ground_level_mm: None,
        }
    }

    pub fn update(&mut self, gps_data: GPSData, _imu_data: IMUData) {
        if gps_data.raw_alt_mm > self.max_altitude_mm {
            self.max_altitude_mm = gps_data.raw_alt_mm;
        }

        if !self.safety_armed && gps_data.raw_alt_mm > MIN_FLIGHT_ALTITUDE_MM {
            self.safety_armed = true;
        }

        match self.state {
            FlightState::Initializing => {
                // Logic handled by task
            }
            FlightState::GroundIdle => {
                if gps_data.fix_valid && gps_data.speed_mm_per_sec > LAUNCH_VELOCITY_THRESHOLD_MMS {
                    self.state = FlightState::PoweredFlight;
                }
                if gps_data.fix_valid && self.ground_level_mm.is_none() {
                    self.ground_level_mm = Some(gps_data.raw_alt_mm);
                }

                if let Some(ground) = self.ground_level_mm {
                    if gps_data.raw_alt_mm > (ground + MIN_FLIGHT_ALTITUDE_MM) {
                        self.safety_armed = true;
                    }
                }
            }
            FlightState::PoweredFlight => {
                if gps_data.speed_mm_per_sec < (self.last_altitude_mm) {
                    self.state = FlightState::Coasting;
                }
            }
            FlightState::Coasting => {
                if self.safety_armed {
                    if gps_data.raw_alt_mm < self.max_altitude_mm - 2000 {
                        self.apogee_detect_count += 1;
                        if self.apogee_detect_count > 5 {
                            self.state = FlightState::ApogeeReached;
                        }
                    } else {
                        self.apogee_detect_count = 0;
                    }
                }
            }
            FlightState::ApogeeReached => {
                self.state = FlightState::Descent;
            }
            _ => {}
        }
        self.last_altitude_mm = gps_data.raw_alt_mm;
    }
}

#[cfg(test)]
mod tests;
