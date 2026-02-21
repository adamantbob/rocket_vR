// health_types.rs
use crate::log::{LogBuffer, Loggable};
use crate::types::FlightTicks;
use core::fmt::Write;
use embassy_time::Instant;
use proc_macros::TelemetryPayload;

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize, TelemetryPayload)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct WifiHealth {
    pub tickstamp: FlightTicks,
    pub initialized: bool,
}

impl WifiHealth {
    pub const fn new() -> Self {
        Self {
            tickstamp: 0,
            initialized: false,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize, TelemetryPayload)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct SDCardHealth {
    pub tickstamp: FlightTicks,
    pub initialized: bool,
    pub buffer_usage: u8, // 0-100%
    pub flush_errors: u16,
}

impl SDCardHealth {
    pub const fn new() -> Self {
        Self {
            tickstamp: 0,
            initialized: false,
            buffer_usage: 0,
            flush_errors: 0,
        }
    }
}

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize, TelemetryPayload)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CPUHealth {
    pub tickstamp: FlightTicks,
    pub usage_c0: u8,
    pub usage_c1: u8,
}

impl CPUHealth {
    pub const fn new() -> Self {
        Self {
            tickstamp: 0,
            usage_c0: 0,
            usage_c1: 0,
        }
    }
    pub fn new_from_readings(usage_c0: u8, usage_c1: u8) -> Self {
        Self {
            tickstamp: Instant::now().as_ticks() as u64,
            usage_c0,
            usage_c1,
        }
    }
}

impl Loggable for CPUHealth {
    const TAG: &'static str = "CPUH";
    fn format_payload<const SIZE: usize>(&self, cursor: &mut LogBuffer<SIZE>) -> core::fmt::Result {
        write!(cursor, "{},{}", self.usage_c0, self.usage_c1)
    }
}
