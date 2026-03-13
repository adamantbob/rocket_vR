use serde::{Deserialize, Serialize};

/// Telemetry packet transmitted from rocket to base station.
///
/// Both sides must use the same definition — postcard serialization is
/// not self-describing, so field order and types must match exactly.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelemetryPacketV1 {
    /// Timestamp in embassy ticks.
    pub tickstamp_seconds_tenths: u32,
    /// Altitude above ground in metres.
    pub altitude_m: i32,
    /// Vertical velocity in m/s (positive = ascending).
    pub velocity_z_ms: i32,
    /// Z-axis acceleration in m/s^2 (positive = ascending).
    pub accel_z_ms2: i32,
    /// GPS latitude in degrees * 1e7 (raw integer).
    pub lat_raw: i32,
    /// GPS longitude in degrees * 1e7 (raw integer).
    pub lon_raw: i32,
    /// Current flight state discriminant.
    pub flight_state: u8,
    /// CPU utilization in percent (200 = 200%).
    pub cpu_utilization: u8,
}

/// Command packet transmitted from base station to rocket.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandPacket {
    /// Sequence number for deduplication.
    pub sequence: u16,
    pub command: RocketCommand,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum RocketCommand {
    /// No-op / keepalive ping.
    Ping,
    /// Arm the pyrotechnic channels.
    Arm,
    /// Manually trigger parachute deployment.
    DeployParachutes,
    /// Abort and safe all pyrotechnics.
    Abort,
}
