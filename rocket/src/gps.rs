use crate::datacells::{POSITION_DATA, PositionData};
use crate::{GPS, GPSResources, Irqs, info};
use embassy_rp::uart::{BufferedUart, BufferedUartRx, Config, Instance};
use embassy_time::{Instant, Timer};
use embedded_io_async::{Read, Write};
use heapless::Vec;
use proc_macros::tracked_task;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[tracked_task(GPS)]
#[embassy_executor::task]
pub async fn gps_task(r: GPSResources, irqs: Irqs) -> ! {
    // We pass the individual fields from our resource struct
    // into the actual driver logic.
    GPSManager::init(r, irqs).await
}

pub struct GPSManager;

impl GPSManager {
    pub async fn init(r: GPSResources, irqs: Irqs) -> ! {
        // --- 1. Handshake at 9600 ---
        static TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new();
        let mut uart = BufferedUart::new(
            r.uart,
            r.tx,
            r.rx,
            irqs,
            TX_BUF.init([0; 64]),
            RX_BUF.init([0; 256]),
            Config::default(),
        );

        // Upgrade Link (Baudrate & 10Hz)
        Self::upgrade_link(&mut uart).await;

        let (_tx, mut rx) = uart.split();
        let mut current_pos = PositionData::new();
        let mut filter = VerticalKalman::new(0.0);
        let mut filter_init = false;
        let mut last_packet = Instant::now();

        // --- 2. Processing Loop ---
        let mut sentence_buffer = [0u8; 128];
        let mut pos = 0;

        loop {
            let mut byte = [0u8; 1];
            if let Ok(_) = rx.read(&mut byte).await {
                let b = byte[0];
                if b == b'\n' || b == b'\r' {
                    if pos > 0 {
                        let dt = (Instant::now() - last_packet).as_ticks() as f32
                            / embassy_time::TICK_HZ as f32;
                        last_packet = Instant::now();

                        Self::process_line(&sentence_buffer[..pos], &mut current_pos);

                        // Update Kalman Filter with the new data
                        if current_pos.fix_valid {
                            let alt_m = current_pos.raw_alt_mm as f32 / 1000.0;

                            // ERROR FIX: Check if dt is sane (e.g., first packet or long pause)
                            if !filter_init || dt > 1.0 {
                                filter = VerticalKalman::new(alt_m);
                                filter_init = true;
                            } else {
                                filter.update(dt, alt_m);
                                let (est_alt, est_vel) = filter.get_state();
                                current_pos.alt_agl_m = est_alt;
                                current_pos.velocity_z = est_vel;
                            }
                        }

                        // Share data
                        POSITION_DATA.lock(|cell| cell.set(current_pos));

                        pos = 0;
                    }
                } else if pos < sentence_buffer.len() {
                    sentence_buffer[pos] = b;
                    pos += 1;
                }
            }
        }
    }

    async fn upgrade_link(uart: &mut BufferedUart) {
        let _ = uart.write_all(b"$PMTK251,115200*1F\r\n").await;
        Timer::after_millis(100).await;
        uart.set_baudrate(115200);
        let _ = uart.write_all(b"$PMTK220,100*2F\r\n").await;
        Timer::after_millis(50).await;
        let _ = uart
            .write_all(b"$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n")
            .await;
    }

    fn process_line(line: &[u8], position_data: &mut PositionData) {
        if !Self::validate_checksum(line) {
            return;
        }
        let Ok(sentence) = core::str::from_utf8(line) else {
            return;
        };

        // We split with a fixed capacity to avoid heap allocation
        let fields: Vec<&str, 20> = sentence.split(',').collect();
        if fields.is_empty() {
            return;
        }

        match fields[0] {
            "$GPGGA" => {
                // Field 1: UTC Time (HHMMSS.SS)
                position_data.utc_time_secs = fields[1]
                    .split('.')
                    .next()
                    .unwrap_or("0")
                    .parse::<u32>()
                    .unwrap_or(0);

                // Field 6: Quality
                position_data.fix_valid = fields[6] != "0";

                // Field 7: Satellites
                position_data.satellites = fields[7].parse::<u8>().unwrap_or(0);

                // Field 8: HDOP/PDOP (GPS units vary, but usually this is horizontal/positional dilution)
                // Parse "1.2" into 12
                position_data.pdop_x10 = Self::parse_to_fixed_x10(fields[8]);

                // Field 9: Altitude
                position_data.raw_alt_mm = Self::parse_to_mm(fields[9]);

                if position_data.fix_valid {
                    position_data.lat_microdegrees = Self::parse_degrees_int(fields[2], fields[3]);
                    position_data.lon_microdegrees = Self::parse_degrees_int(fields[4], fields[5]);
                }
            }
            "$GPRMC" => {
                // RMC also has time at Field 1
                position_data.utc_time_secs = fields[1]
                    .split('.')
                    .next()
                    .unwrap_or("0")
                    .parse::<u32>()
                    .unwrap_or(0);
                position_data.fix_valid = fields[2] == "A";
                position_data.speed_mm_per_sec = Self::parse_speed_to_mms(fields[7]);
            }
            _ => {}
        }
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

        let total_micro_degrees =
            (degrees * 1_000_000) + ((minutes * 1_000_000 + frac_val * 100) / 60);

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
    fn parse_speed_to_mms(raw: &str) -> i32 {
        if raw.is_empty() {
            return 0;
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
        (knots_whole * 514) + (knots_frac * 5)
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

    /// Validates the checksum of a NMEA sentence.
    /// Returns true if the checksum is valid, false otherwise.
    fn validate_checksum(line: &[u8]) -> bool {
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
}

pub struct VerticalKalman {
    z: f32,    // Estimated Altitude (m)
    v: f32,    // Estimated Velocity (m/s)
    p_zz: f32, // Estimation Error Covariance (Alt)
    p_vv: f32, // Estimation Error Covariance (Vel)
    p_zv: f32, // Cross Covariance

    r_alt: f32,   // Measurement Noise (GPS Jitter - ~2.0m)
    q_accel: f32, // Process Noise (Rocket Dynamics - ~0.5)
}

impl VerticalKalman {
    pub fn new(initial_alt: f32) -> Self {
        Self {
            z: initial_alt,
            v: 0.0,
            p_zz: 1.0,
            p_vv: 1.0,
            p_zv: 0.0,
            r_alt: 2.0,   // GPS is usually noisy
            q_accel: 0.1, // How much we trust our "constant velocity" model
        }
    }

    pub fn update(&mut self, dt: f32, measured_z: f32) {
        let dt2 = dt * dt;
        let dt3 = dt2 * dt;
        let dt4 = dt3 * dt;

        // --- 1. Predict ---
        self.z += self.v * dt;

        // Update Error Covariance using simple multiplications
        self.p_zz += dt * (2.0 * self.p_zv + dt * self.p_vv) + 0.25 * dt4 * self.q_accel;
        self.p_zv += dt * self.p_vv + 0.5 * dt3 * self.q_accel;
        self.p_vv += dt2 * self.q_accel;

        // --- 2. Update (Correct with GPS) ---
        let innovation = measured_z - self.z;
        let s = self.p_zz + self.r_alt; // Innovation covariance

        let k_z = self.p_zz / s; // Kalman Gain for Altitude
        let k_v = self.p_zv / s; // Kalman Gain for Velocity

        // Apply Gain to State
        self.z += k_z * innovation;
        self.v += k_v * innovation;

        // Update Error Covariance (P = (I - KH)P)
        self.p_zz -= k_z * self.p_zz;
        self.p_zv -= k_z * self.p_zv;
        self.p_vv -= k_v * self.p_zv;
    }

    pub fn get_state(&self) -> (f32, f32) {
        (self.z, self.v)
    }
}
