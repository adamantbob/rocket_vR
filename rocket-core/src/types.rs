// types.rs
#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlightState {
    Initializing = 0,  // GPS Calibration
    GroundIdle = 1,    // Waiting on the pad
    PoweredFlight = 2, // Motor is burning, high acceleration
    Coasting = 3,      // Motor out, gaining altitude via momentum
    ApogeeReached = 4, // The peak! Prepare for deployment
    Descent = 5,       // Coming down under parachute
    Landed = 6,        // Back on the ground
}

#[repr(C)]
#[derive(Clone, Copy, Debug, serde::Serialize, serde::Deserialize)]
pub struct PersistentData {
    pub ground_level: i32,
    pub state: FlightState,
}
