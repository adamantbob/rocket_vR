// state_machine.rs
use crate::gps::types::GPSData;
use crate::imu::types::IMUData;
use crate::types::FlightState;

/// Minimum altitude gain (in mm) required before the safety_armed flag is set.
/// This prevents premature parachute deployment due to sensor noise at ground level.
pub const MIN_FLIGHT_ALTITUDE_MM: i32 = 5_000; // 5 Meters

/// Vertical velocity threshold (in mm/s) to trigger the PoweredFlight state.
pub const LAUNCH_VELOCITY_THRESHOLD_MMS: i32 = 1_000; // 1 m/s

/// The core Flight State Machine logic.
/// Tracks altitude, velocity, and health to manage transitions through the launch lifecycle.
pub struct RocketStateMachine {
    /// The current operational state of the rocket.
    pub state: FlightState,
    /// The altitude recorded in the previous update cycle.
    pub last_altitude_mm: i32,
    /// The vertical velocity recorded in the previous update cycle (signed).
    pub last_velocity_z_mms: i32,
    /// The maximum altitude reached during the current flight flight.
    pub max_altitude_mm: i32,
    /// Counter for detecting apogee (sustained altitude drop).
    pub apogee_detect_count: u8,
    /// Flag indicating the rocket has reached a safe altitude for event triggers.
    pub safety_armed: bool,
    /// Calibrated ground level altitude (Mean Sea Level).
    pub ground_level_mm: Option<i32>,
}

impl RocketStateMachine {
    /// Constructs a new state machine in the Initializing state.
    pub const fn new() -> Self {
        Self {
            state: FlightState::Initializing,
            last_altitude_mm: 0,
            last_velocity_z_mms: 0,
            max_altitude_mm: 0,
            apogee_detect_count: 0,
            safety_armed: false,
            ground_level_mm: None,
        }
    }

    /// Primary entry point for updating flight state based on new sensor data.
    /// This should be called at a regular frequency (e.g., 10Hz-100Hz).
    pub fn update(&mut self, gps_data: GPSData, _imu_data: IMUData) {
        // --- 1. Global Peak Tracking ---
        if gps_data.raw_alt_mm > self.max_altitude_mm {
            self.max_altitude_mm = gps_data.raw_alt_mm;
        }

        // --- 2. Safety Arming ---
        // Arm the system once we've gained a minimum height above the assumed ground level.
        if !self.safety_armed && gps_data.raw_alt_mm > MIN_FLIGHT_ALTITUDE_MM {
            self.safety_armed = true;
        }

        // --- 3. State Transition Logic ---
        match self.state {
            FlightState::Initializing => {
                // Initializing state is typically transitioned out of by the
                // caller after a successful ground calibration.
            }
            FlightState::GroundIdle => {
                // Launch detection: triggered by a sustained vertical velocity.
                if gps_data.fix_valid && gps_data.velocity_z_mms > LAUNCH_VELOCITY_THRESHOLD_MMS {
                    self.state = FlightState::PoweredFlight;
                }

                // Backup ground level tracking for relative safety checks.
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
                // Burnout detection: Detect when thrust ends by monitoring for
                // a significant drop in vertical velocity (deceleration).
                // At high frequencies (100Hz), a 100mm/s drop per sample
                // corresponds to ~1G deceleration (drag/gravity taking over).
                if gps_data.velocity_z_mms < self.last_velocity_z_mms - 100 {
                    self.state = FlightState::Coasting;
                }
            }
            FlightState::Coasting => {
                // Apogee detection: Detect when the rocket has reached the peak
                // of its trajectory and has reliably begun to descend.
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
                // Auto-transition to descent phase once apogee is confirmed.
                self.state = FlightState::Descent;
            }
            _ => {}
        }

        // --- 4. History Tracking ---
        self.last_altitude_mm = gps_data.raw_alt_mm;
        self.last_velocity_z_mms = gps_data.velocity_z_mms;
    }
}

#[cfg(test)]
mod tests;
