/// Telemetry packet transmitted from rocket to base station.
///
/// Both sides must use the same definition — postcard serialization is
/// not self-describing, so field order and types must match exactly.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelemetryPacket {
    /// Timestamp in embassy ticks.
    pub tickstamp: u64,
    /// Altitude above ground in millimetres.
    pub altitude_mm: i32,
    /// Vertical velocity in mm/s (positive = ascending).
    pub velocity_z_mms: i32,
    /// Z-axis acceleration in milli-G.
    pub accel_z_mg: i32,
    /// GPS latitude in degrees * 1e7 (raw integer).
    pub lat_raw: i32,
    /// GPS longitude in degrees * 1e7 (raw integer).
    pub lon_raw: i32,
    /// Current flight state discriminant.
    pub flight_state: u8,
    /// Core 0 CPU utilization in tenths of a percent (1000 = 100.0%).
    pub cpu0_utilization: u16,
    /// Core 1 CPU utilization in tenths of a percent.
    pub cpu1_utilization: u16,
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
