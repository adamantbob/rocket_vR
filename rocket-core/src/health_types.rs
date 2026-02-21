// health_types.rs
use crate::log::{LogBuffer, Loggable};
use crate::types::FlightTicks;
use core::fmt::Write;
use core::ops::{Add, Sub};
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

/// A utilization percentage stored as a `u32` in units of 0.1%.
///
/// `1000` = 100.0%, `10` = 1.0%, `1` = 0.1%.
#[derive(
    Default,
    Clone,
    Copy,
    Debug,
    serde::Serialize,
    serde::Deserialize,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
)]
pub struct DeciPercent {
    num: u32,
}

impl DeciPercent {
    pub const ZERO: Self = DeciPercent { num: 0 };
    /// Compute `(numerator / denominator) * 100`, returning tenths of a Decipercent.
    ///
    /// Uses `saturating_mul` to prevent overflow on large tick deltas.
    /// Returns `DeciPercent(0)` if `denominator` is zero.
    pub const fn new(num: u32) -> Self {
        DeciPercent { num: num * 10 }
    }
    pub const fn from_ticks(numerator: u32, denominator: u32) -> Self {
        if denominator == 0 {
            return DeciPercent::ZERO;
        }
        DeciPercent {
            num: numerator.saturating_mul(1000) / denominator,
        }
    }

    /// Saturating subtraction — clamps to zero rather than wrapping.
    pub fn saturating_sub(self, other: DeciPercent) -> DeciPercent {
        DeciPercent {
            num: self.num.saturating_sub(other.num),
        }
    }

    /// Return the whole-Decipercent part, rounded up, for use in coarse health metrics.
    pub fn ceil_percent_u8(self) -> u8 {
        ((self.num + 9) / 10).min(100) as u8
    }
}

#[cfg(feature = "defmt")]
impl defmt::Format for DeciPercent {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(fmt, "{}.{}%", self.num / 10, self.num % 10)
    }
}

impl core::fmt::Display for DeciPercent {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{}.{}%", self.num / 10, self.num % 10)
    }
}

impl From<i32> for DeciPercent {
    fn from(value: i32) -> Self {
        if value < 0 {
            return DeciPercent::ZERO;
        }
        DeciPercent::new(value as u32)
    }
}

impl Add for DeciPercent {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        DeciPercent {
            num: self.num.saturating_add(rhs.num),
        }
    }
}

impl Sub for DeciPercent {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        DeciPercent {
            num: self.num.saturating_sub(rhs.num),
        }
    }
}

#[macro_export]
macro_rules! deci_percent {
    ($num:expr) => {
        rocket_core::health_types::DeciPercent::new($num)
    };
}

#[derive(Clone, Copy, Debug, Default, serde::Serialize, serde::Deserialize, TelemetryPayload)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct CPUHealth {
    pub tickstamp: FlightTicks,
    pub usage_c0: DeciPercent,
    pub usage_c0_high_prio: DeciPercent,
    pub usage_c0_background: DeciPercent,
    pub usage_c1: DeciPercent,
    pub usage_c1_stack_hwm: DeciPercent,
}

impl CPUHealth {
    pub const fn new() -> Self {
        Self {
            tickstamp: 0,
            usage_c0: DeciPercent::ZERO,
            usage_c0_high_prio: DeciPercent::ZERO,
            usage_c0_background: DeciPercent::ZERO,
            usage_c1: DeciPercent::ZERO,
            usage_c1_stack_hwm: DeciPercent::ZERO,
        }
    }
    pub fn new_from_readings(
        usage_c0: DeciPercent,
        usage_c0_high_prio: DeciPercent,
        usage_c0_background: DeciPercent,
        usage_c1: DeciPercent,
        usage_c1_stack_hwm: DeciPercent,
    ) -> Self {
        Self {
            tickstamp: Instant::now().as_ticks() as u64,
            usage_c0,
            usage_c0_high_prio,
            usage_c0_background,
            usage_c1,
            usage_c1_stack_hwm,
        }
    }
}

impl Loggable for CPUHealth {
    const TAG: &'static str = "CPUH";
    fn format_payload<const SIZE: usize>(&self, cursor: &mut LogBuffer<SIZE>) -> core::fmt::Result {
        write!(
            cursor,
            "{},{},{},{},{}",
            self.usage_c0,
            self.usage_c0_high_prio,
            self.usage_c0_background,
            self.usage_c1,
            self.usage_c1_stack_hwm,
        )
    }
}
