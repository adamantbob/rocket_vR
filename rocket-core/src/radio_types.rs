use serde::{Deserialize, Serialize};

/// Enum wrapping all possible radio packets.
///
/// Postcard serialization adds a single byte discriminant at the start,
/// allowing the receiver to determine which packet type was sent.
///
/// To add adjust the packet encoding, add a new variant to this enum and
/// add a new Packet, leaving the old one around for backwards compatibility.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum RadioPacket {
    /// Telemetry from rocket to base station.
    TelemetryV1(TelemetryPacketV1),
    /// Command from base station to rocket.
    CommandV1(CommandPacketV1),
    /// Full location from rocket to base station.
    FullLocationV1(FullLocationPacketV1),
}

/// Telemetry packet transmitted from rocket to base station.
///
/// Both sides must use the same definition — postcard serialization is
/// not self-describing, so field order and types must match exactly.
///
/// To add adjust the packet encoding, add a new type of this Packet,
/// leaving the old one around for backwards compatibility and to not break
/// the encoding scheme.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TelemetryPacketV1 {
    /// Timestamp in embassy ticks.
    pub tickstamp_seconds_tenths: u16,
    /// Altitude above ground in metres.
    pub altitude_m: i16,
    /// Vertical velocity in decimeters per second (positive = ascending).
    pub velocity_z_dms: i16,
    /// Z-axis acceleration in decimeters per second squared (positive = ascending).
    pub accel_z_dms2: i16,
    /// GPS latitude offset from launch site in degrees * 1e5 (raw integer).
    pub lat_offset_1e5: i16,
    /// GPS longitude offset from launch site in degrees * 1e5 (raw integer).
    pub lon_offset_1e5: i16,
    /// Current flight state discriminant.
    pub flight_state: u8,
    /// Dual Core CPU utilization - 0-200%.
    pub cpu_utilization: u8,
}

/// Full location packet transmitted from Rocket to base station.
///
/// This packet is sent at different rates depending on flight stage:
/// - Every second during launch setup
/// - Not at all during the powered flight, coasting, and apogee
/// - Every second after parachute deployment
///
/// To add adjust the packet encoding, add a new type of this Packet,
/// leaving the old one around for backwards compatibility and to not break
/// the encoding scheme.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FullLocationPacketV1 {
    pub lat_1e7: i32,
    pub lon_1e7: i32,
    pub altitude_m: i16,
    pub tickstamp_seconds_tenths: u32,
}

/// Command packet transmitted from base station to rocket.
///
/// Both sides must use the same definition — postcard serialization is
/// not self-describing, so field order and types must match exactly.
///
/// To add adjust the packet encoding, add a new type of this Packet,
/// leaving the old one around for backwards compatibility and to not break
/// the encoding scheme.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CommandPacketV1 {
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
