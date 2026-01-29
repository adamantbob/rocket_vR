use crate::gps::types::{GPSData, SensorError};
use heapless::Vec;

/// Validates the checksum of a NMEA sentence.
/// Returns true if the checksum is valid, false otherwise.
pub fn validate_checksum(line: &[u8]) -> bool {
    let mut parts = line.split(|&b| b == b'*');
    let payload = parts.next().unwrap_or(&[]);
    let checksum_hex = parts.next().unwrap_or(&[]);
    if payload.is_empty() || checksum_hex.len() < 2 {
        return false;
    }

    let mut calc = 0u8;
    let start = if payload.starts_with(b"$") { 1 } else { 0 };
    for &b in &payload[start..] {
        calc ^= b;
    }

    let prov = core::str::from_utf8(checksum_hex)
        .ok()
        .and_then(|s| u8::from_str_radix(s, 16).ok())
        .unwrap_or(0);
    calc == prov
}

pub fn process_line(line: &[u8], gps_data: &mut GPSData) -> Result<(), SensorError> {
    if !validate_checksum(line) {
        return Err(SensorError::InvalidChecksum);
    }
    let Ok(sentence) = core::str::from_utf8(line) else {
        return Err(SensorError::InvalidData);
    };

    // We split with a fixed capacity to avoid heap allocation
    let fields: Vec<&str, 20> = sentence.split(',').collect();
    if fields.is_empty() {
        return Err(SensorError::InvalidData);
    }

    match fields[0] {
        "$GPGGA" => {
            // Field 1: UTC Time (HHMMSS.SS)
            gps_data.utc_time_secs = fields[1]
                .split('.')
                .next()
                .unwrap_or("0")
                .parse::<u32>()
                .unwrap_or(0);

            // Field 6: Quality
            gps_data.fix_valid = fields[6] != "0";

            // Field 7: Satellites
            gps_data.satellites = fields[7].parse::<u8>().unwrap_or(0);

            // Field 8: HDOP/PDOP (GPS units vary, but usually this is horizontal/positional dilution)
            // Parse "1.2" into 12
            gps_data.pdop_x10 = parse_to_fixed_x10(fields[8]);

            // Field 9: Altitude
            gps_data.raw_alt_mm = parse_to_mm(fields[9]);

            if gps_data.fix_valid {
                gps_data.lat_microdegrees = parse_degrees_int(fields[2], fields[3]);
                gps_data.lon_microdegrees = parse_degrees_int(fields[4], fields[5]);
            }
        }
        "$GPRMC" => {
            // RMC also has time at Field 1
            gps_data.utc_time_secs = fields[1]
                .split('.')
                .next()
                .unwrap_or("0")
                .parse::<u32>()
                .unwrap_or(0);
            gps_data.fix_valid = fields[2] == "A";
            gps_data.speed_mm_per_sec = parse_speed_to_mms(fields[7])?;
        }
        _ => {}
    }
    Ok(())
}

/// Parses a raw NMEA latitude/longitude string (DDMM.MMMM) into microdegrees.
///
/// NMEA format is degrees and decimal minutes combined. This function:
/// 1. Separates the whole degrees.
/// 2. Converts decimal minutes into microdegrees by dividing by 60.
/// 3. Applies the Direction (N/S/E/W) sign.
fn parse_degrees_int(raw: &str, dir: &str) -> i32 {
    if raw.is_empty() {
        return 0;
    }
    let dot_pos = raw.find('.').unwrap_or(0);
    if dot_pos < 2 {
        return 0;
    }

    // Split into Degrees, Minutes, and Fractional Minutes
    let degrees = raw[..dot_pos - 2].parse::<i32>().unwrap_or(0);
    let minutes = raw[dot_pos - 2..dot_pos].parse::<i32>().unwrap_or(0);
    let frac_min_str = &raw[dot_pos + 1..];

    // We need minutes / 60. To do this in integers:
    // (Minutes * 1,000,000 + FractionalPart) / 60
    // This preserves precision without floats.
    let mut frac_val = frac_min_str.parse::<i32>().unwrap_or(0);
    // Adjust frac_val based on string length (NMEA usually gives 4 digits)
    for _ in 0..(4 - frac_min_str.len()) {
        frac_val *= 10;
    }

    let total_micro_degrees = (degrees * 1_000_000) + ((minutes * 1_000_000 + frac_val * 100) / 60);

    if dir == "S" || dir == "W" {
        -total_micro_degrees
    } else {
        total_micro_degrees
    }
}

/// Converts a Knots string (e.g., "12.5") to Millimeters per Second (mm/s).
///
/// 1 Knot = 514.44 mm/s.
/// This implementation handles the decimal point manually to avoid f32 math.
fn parse_speed_to_mms(raw: &str) -> Result<i32, SensorError> {
    if raw.is_empty() {
        return Err(SensorError::InvalidData);
    }

    let mut parts = raw.split('.');
    let knots_whole = parts.next().unwrap_or("0").parse::<i32>().unwrap_or(0);

    // Take first 2 digits of fractional knots (tenths/hundredths)
    let knots_frac_str = parts.next().unwrap_or("0");
    let knots_frac = knots_frac_str
        .get(0..2)
        .unwrap_or(knots_frac_str)
        .parse::<i32>()
        .unwrap_or(0);

    // Scale whole knots by 514, and fractional knots accordingly
    // (Whole * 514) + (Fractional * 5.14)
    Ok((knots_whole * 514) + (knots_frac * 5))
}

fn parse_to_fixed_x10(raw: &str) -> u16 {
    let mut parts = raw.split('.');
    let whole = parts.next().unwrap_or("0").parse::<u16>().unwrap_or(0);
    let frac = parts
        .next()
        .unwrap_or("0")
        .get(0..1)
        .unwrap_or("0")
        .parse::<u16>()
        .unwrap_or(0);
    (whole * 10) + frac
}

/// Extracts the integer altitude in meters and converts it to millimeters.
///
/// Example: "540.5" meters becomes 540,500 mm.
fn parse_to_mm(raw: &str) -> i32 {
    let mut parts = raw.split('.');
    let meters = parts.next().unwrap_or("0").parse::<i32>().unwrap_or(0);
    let cm = parts
        .next()
        .unwrap_or("0")
        .get(0..2)
        .unwrap_or("0")
        .parse::<i32>()
        .unwrap_or(0);
    (meters * 1000) + (cm * 10)
}
