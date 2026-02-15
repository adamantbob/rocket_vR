// types.rs
/// FlightTicks is the number of ticks on a monotonic microsecond clock
/// which start at boot time. It is monotonic and will wrap around at 2^64 microseconds
/// which is approximately 5.8 million years.
/// The FlightTick rate is defined by embassy_time::TICK_HZ
/// which is embedded into the log schema.
pub type FlightTicks = u64;

pub trait TimedData {
    fn tickstamp(&self) -> FlightTicks;
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlightState {
    #[default]
    Initializing = 0, // GPS Calibration
    GroundIdle = 1,    // Waiting on the pad
    PoweredFlight = 2, // Motor is burning, high acceleration
    Coasting = 3,      // Motor out, gaining altitude via momentum
    ApogeeReached = 4, // The peak! Prepare for deployment
    Descent = 5,       // Coming down under parachute
    Landed = 6,        // Back on the ground
}

#[repr(u8)]
#[derive(Debug, Clone, Copy, PartialEq, Default, serde::Serialize, serde::Deserialize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlightAction {
    #[default]
    None,
    StageNext,
    DeployParachutes,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize)]
pub struct PersistentData {
    pub ground_level: i32,
    pub state: FlightState,
}
