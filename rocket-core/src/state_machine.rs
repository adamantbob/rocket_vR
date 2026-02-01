use crate::gps::types::GPSData;
use crate::imu::types::IMUData;
use crate::types::{FlightAction, FlightState};

/// Minimum altitude gain (in mm) required before the safety_armed flag is set.
/// This prevents premature parachute deployment due to sensor noise at ground level.
pub const MIN_FLIGHT_ALTITUDE_MM: i32 = 5_000; // 5 Meters

/// Vertical velocity threshold (in mm/s) to trigger the PoweredFlight state.
pub const LAUNCH_VELOCITY_THRESHOLD_MMS: i32 = 1_000; // 1 m/s

/// FlightController handles the flight state machine and autonomous decisions.
/// Tracks altitude, velocity, and health to manage transitions and trigger actions.
pub struct FlightController {
    /// The current operational state of the rocket.
    pub state: FlightState,
    /// The altitude recorded in the previous update cycle.
    pub last_altitude_mm: i32,
    /// The vertical velocity recorded in the previous update cycle (signed).
    pub last_velocity_z_mms: i32,
    /// The vertical acceleration recorded in the previous update cycle (mG).
    pub last_accel_z_m_g: i32,
    /// The maximum altitude reached during the current flight flight.
    pub max_altitude_mm: i32,
    /// Counter for detecting apogee (sustained altitude drop).
    pub apogee_detect_count: u8,
    /// Counter for detecting sustained acceleration (launch).
    pub launch_detect_count: u8,
    /// Counter for detecting sustained deceleration/low-G (burnout).
    pub burnout_detect_count: u8,
    /// Flag indicating the rocket has reached a safe altitude for event triggers.
    pub safety_armed: bool,
    /// Calibrated ground level altitude (Mean Sea Level).
    pub ground_level_mm: Option<i32>,
    /// Calibrated Z-axis acceleration offset (expected to be ~1000mG).
    pub accel_z_offset_m_g: i32,
}

impl FlightController {
    /// Constructs a new controller in the Initializing state.
    pub const fn new() -> Self {
        Self {
            state: FlightState::Initializing,
            last_altitude_mm: 0,
            last_velocity_z_mms: 0,
            last_accel_z_m_g: 0,
            max_altitude_mm: 0,
            apogee_detect_count: 0,
            launch_detect_count: 0,
            burnout_detect_count: 0,
            safety_armed: false,
            ground_level_mm: None,
            accel_z_offset_m_g: 1000, // Default to 1G
        }
    }

    /// Primary entry point for updating flight state based on new sensor data.
    /// Returns the current state and any action that should be taken.
    pub fn update(&mut self, gps_data: GPSData, imu_data: IMUData) -> (FlightState, FlightAction) {
        let old_state = self.state;

        // --- 1. Global Peak Tracking ---
        if gps_data.raw_alt_mm > self.max_altitude_mm {
            self.max_altitude_mm = gps_data.raw_alt_mm;
        }

        // --- 2. Safety Arming ---
        if !self.safety_armed {
            if let Some(ground) = self.ground_level_mm {
                if gps_data.raw_alt_mm > (ground + MIN_FLIGHT_ALTITUDE_MM) {
                    self.safety_armed = true;
                }
            }
        }

        let vert_accel_m_g = imu_data.accel_high_g[2] - self.accel_z_offset_m_g;

        // --- 3. State Transition Logic ---
        match self.state {
            FlightState::Initializing => {}
            FlightState::GroundIdle => {
                let velocity_trigger =
                    gps_data.fix_valid && gps_data.velocity_z_mms > LAUNCH_VELOCITY_THRESHOLD_MMS;
                let accel_trigger = vert_accel_m_g > 2000;

                if velocity_trigger || accel_trigger {
                    self.launch_detect_count += 1;
                    if self.launch_detect_count > 1 {
                        self.state = FlightState::PoweredFlight;
                    }
                } else {
                    self.launch_detect_count = 0;
                }

                if gps_data.fix_valid && self.ground_level_mm.is_none() {
                    self.ground_level_mm = Some(gps_data.raw_alt_mm);
                }
            }
            FlightState::PoweredFlight => {
                let velocity_drop = gps_data.velocity_z_mms < self.last_velocity_z_mms - 100;
                let low_accel = vert_accel_m_g < 500;

                if velocity_drop || low_accel {
                    self.burnout_detect_count += 1;
                    if self.burnout_detect_count > 10 {
                        self.state = FlightState::Coasting;
                    }
                } else {
                    self.burnout_detect_count = 0;
                }
            }
            FlightState::Coasting => {
                if self.safety_armed {
                    if gps_data.raw_alt_mm < self.max_altitude_mm - 3000 {
                        self.apogee_detect_count += 1;
                        if self.apogee_detect_count > 10 {
                            self.state = FlightState::ApogeeReached;
                        }
                    } else if gps_data.raw_alt_mm >= self.max_altitude_mm {
                        self.apogee_detect_count = 0;
                    }
                }
            }
            FlightState::ApogeeReached => {
                self.state = FlightState::Descent;
            }
            _ => {}
        }

        // --- 4. Autonomous Action Decisions ---
        let mut action = FlightAction::None;
        if self.state != old_state {
            match (old_state, self.state) {
                (FlightState::PoweredFlight, FlightState::Coasting) => {
                    action = FlightAction::StageNext;
                }
                (FlightState::Coasting, FlightState::ApogeeReached)
                | (FlightState::PoweredFlight, FlightState::ApogeeReached) => {
                    action = FlightAction::DeployParachutes;
                }
                _ => {}
            }
        }

        // --- 5. History Tracking ---
        self.last_altitude_mm = gps_data.raw_alt_mm;
        self.last_velocity_z_mms = gps_data.velocity_z_mms;
        self.last_accel_z_m_g = imu_data.accel_high_g[2];

        (self.state, action)
    }

    pub fn set_ground_reference(&mut self, alt_mm: i32, accel_z_offset_m_g: i32) {
        self.ground_level_mm = Some(alt_mm);
        self.accel_z_offset_m_g = accel_z_offset_m_g;
        self.state = FlightState::GroundIdle;
    }
}

#[cfg(test)]
mod tests;
