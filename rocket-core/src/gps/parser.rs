// gps/parser.rs
use crate::gps::types::{GPSData, GPSSensorError};
use heapless::Vec;

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

pub fn process_line(line: &[u8], gps_data: &mut GPSData) -> Result<(), GPSSensorError> {
    if !validate_checksum(line) {
        return Err(GPSSensorError::InvalidChecksum);
    }
    let Ok(sentence) = core::str::from_utf8(line) else {
        return Err(GPSSensorError::InvalidData);
    };

    let fields: Vec<&str, 20> = sentence.split(',').collect();
    if fields.is_empty() {
        return Err(GPSSensorError::InvalidData);
    }

    match fields[0] {
        "$GPGGA" => {
            gps_data.utc_time_secs = fields[1]
                .split('.')
                .next()
                .unwrap_or("0")
                .parse::<u32>()
                .unwrap_or(0);
            gps_data.fix_valid = fields[6] != "0";
            gps_data.satellites = fields[7].parse::<u8>().unwrap_or(0);
            gps_data.pdop_x10 = parse_to_fixed_x10(fields[8]);
            gps_data.raw_alt_mm = parse_to_mm(fields[9]);

            if gps_data.fix_valid {
                gps_data.lat_microdegrees = parse_degrees_int(fields[2], fields[3]);
                gps_data.lon_microdegrees = parse_degrees_int(fields[4], fields[5]);
            }
        }
        "$GPRMC" => {
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

fn parse_degrees_int(raw: &str, dir: &str) -> i32 {
    if raw.is_empty() {
        return 0;
    }
    let dot_pos = raw.find('.').unwrap_or(0);
    if dot_pos < 2 {
        return 0;
    }

    let degrees = raw[..dot_pos - 2].parse::<i32>().unwrap_or(0);
    let minutes = raw[dot_pos - 2..dot_pos].parse::<i32>().unwrap_or(0);
    let frac_min_str = &raw[dot_pos + 1..];

    let mut frac_val = frac_min_str.parse::<i32>().unwrap_or(0);
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

fn parse_speed_to_mms(raw: &str) -> Result<i32, GPSSensorError> {
    if raw.is_empty() {
        return Err(GPSSensorError::InvalidData);
    }
    let mut parts = raw.split('.');
    let knots_whole = parts.next().unwrap_or("0").parse::<i32>().unwrap_or(0);
    let knots_frac_str = parts.next().unwrap_or("0");
    let knots_frac = knots_frac_str
        .get(0..2)
        .unwrap_or(knots_frac_str)
        .parse::<i32>()
        .unwrap_or(0);
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

fn parse_to_mm(raw: &str) -> i32 {
    let mut parts = raw.split('.');
    let meters = parts.next().unwrap_or("0").parse::<i32>().unwrap_or(0);
    let frac_str = parts.next().unwrap_or("");

    let mut frac_val = 0;
    if !frac_str.is_empty() {
        let mut padded = [b'0'; 3];
        let len = core::cmp::min(frac_str.len(), 3);
        padded[..len].copy_from_slice(&frac_str.as_bytes()[..len]);
        let s = core::str::from_utf8(&padded).unwrap_or("000");
        frac_val = s.parse::<i32>().unwrap_or(0);
    }

    (meters * 1000) + frac_val
}

#[cfg(test)]
mod tests;
