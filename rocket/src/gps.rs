use crate::{Irqs, GPSResources, info, GPS};
use embassy_rp::uart::{BufferedUart, Config};
use embedded_io_async::{Read, Write};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};
use crate::datacells::{GpsData, GPS_DATA};
use heapless::Vec;
use proc_macros::tracked_task;

#[tracked_task(GPS)]
#[embassy_executor::task]
pub async fn gps_task(r: GPSResources, irqs: Irqs) -> ! {
    // We pass the individual fields from our resource struct
    // into the actual driver logic.
    GPSManager::init(r, irqs).await
}

pub struct GPSManager;

impl GPSManager {
    pub async fn init(
        r: GPSResources, // Match your macro case
        irqs: Irqs,
    ) -> ! {
        // --- 1. UART Setup ---
        // Adafruit GPS default is 9600 baud
        let mut config = Config::default();
        config.baudrate = 9600;

        static TX_BUF: StaticCell<[u8; 64]> = StaticCell::new();
        let tx_buf = &mut TX_BUF.init([0; 64])[..];
        static RX_BUF: StaticCell<[u8; 256]> = StaticCell::new(); // GPS sends big bursts, larger buffer helps
        let rx_buf = &mut RX_BUF.init([0; 256])[..];
        
        let uart = BufferedUart::new(r.uart, r.tx, r.rx, irqs, tx_buf, rx_buf, config);
        let (_tx, mut rx) = uart.split();

        let mut local_gps_data = GpsData::default();

        info!("GPS UART Initialized at 9600 baud");

        // --- 2. Processing Loop ---
        let mut sentence_buffer = [0u8; 128]; // Max NMEA length is 82, 128 is safe
        let mut pos = 0;

        loop {
            let mut byte = [0u8; 1];
            // Read exactly one byte
            if let Ok(_) = rx.read(&mut byte).await {
                let b = byte[0];

                if b == b'\n' || b == b'\r' {
                    if pos > 0 {
                        // We found a complete line!
                        let sentence = &sentence_buffer[..pos];
                        Self::process_line(sentence, &mut local_gps_data);
                        info!("GPS Data: {:?}", local_gps_data);
                        GPS_DATA.lock(|cell: &core::cell::Cell<GpsData>| cell.set(local_gps_data));
                        pos = 0; // Reset for next line
                    }
                } else {
                    // Add byte to buffer if there's room
                    if pos < sentence_buffer.len() {
                        sentence_buffer[pos] = b;
                        pos += 1;
                    } else {
                        // Buffer overflow - reset
                        pos = 0;
                    }
                }
            }
        }
    }

    fn process_line(line: &[u8], gps_data: &mut GpsData) {
        let Ok(sentence) = core::str::from_utf8(line) else { return };
        let clean_sentence = sentence.split('*').next().unwrap_or("");
        let fields: Vec<&str, 24> = clean_sentence.split(',').take(24).collect();
        if fields.is_empty() { return; }

        match fields[0] {
            "$GPGGA" => {
                // Field 1: Time (HHMMSS.SS) - We'll take the integer part
                gps_data.utc_time_secs = fields[1].split('.').next().unwrap_or("0").parse::<u32>().unwrap_or(0);
                
                // Field 6: Quality
                gps_data.fix_valid = fields[6] != "0";
                
                // Field 7: Satellites
                gps_data.satellites = fields[7].parse::<u8>().unwrap_or(0);
                
                // Field 9: Altitude (Meters) -> Convert to mm
                // We parse as i32 and multiply by 1000 manually to avoid float parsing
                gps_data.altitude_mm = Self::parse_to_mm(fields[9]);

                if gps_data.fix_valid {
                    gps_data.lat_microdegrees = Self::parse_degrees_int(fields[2], fields[3]);
                    gps_data.lon_microdegrees = Self::parse_degrees_int(fields[4], fields[5]);
                }
            }
            "$GPRMC" => {
                gps_data.fix_valid = fields[2] == "A";
                // Field 7: Speed in Knots. 1 knot = 514.444 mm/s
                // To keep it integer: (knots_integer * 514)
                gps_data.speed_mm_per_sec = Self::parse_speed_to_mms(fields[7]);
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
        if raw.is_empty() { return 0; }
        let dot_pos = raw.find('.').unwrap_or(0);
        if dot_pos < 2 { return 0; }

        // Split into Degrees, Minutes, and Fractional Minutes
        let degrees = raw[..dot_pos-2].parse::<i32>().unwrap_or(0);
        let minutes = raw[dot_pos-2..dot_pos].parse::<i32>().unwrap_or(0);
        let frac_min_str = &raw[dot_pos+1..];
        
        // We need minutes / 60. To do this in integers:
        // (Minutes * 1,000,000 + FractionalPart) / 60
        // This preserves precision without floats.
        let mut frac_val = frac_min_str.parse::<i32>().unwrap_or(0);
        // Adjust frac_val based on string length (NMEA usually gives 4 digits)
        for _ in 0..(4 - frac_min_str.len()) { frac_val *= 10; }

        let total_micro_degrees = (degrees * 1_000_000) + 
                                  ((minutes * 1_000_000 + frac_val * 100) / 60);

        if dir == "S" || dir == "W" { -total_micro_degrees } else { total_micro_degrees }
    }

    /// Converts a Knots string (e.g., "12.5") to Millimeters per Second (mm/s).
    /// 
    /// 1 Knot = 514.44 mm/s.
    /// This implementation handles the decimal point manually to avoid f32 math.
    fn parse_speed_to_mms(raw: &str) -> i32 {
        if raw.is_empty() { return 0; }

        let mut parts = raw.split('.');
        let knots_whole = parts.next().unwrap_or("0").parse::<i32>().unwrap_or(0);
        
        // Take first 2 digits of fractional knots (tenths/hundredths)
        let knots_frac_str = parts.next().unwrap_or("0");
        let knots_frac = knots_frac_str.get(0..2).unwrap_or(knots_frac_str).parse::<i32>().unwrap_or(0);

        // Scale whole knots by 514, and fractional knots accordingly
        // (Whole * 514) + (Fractional * 5.14)
        (knots_whole * 514) + (knots_frac * 5)
    }

    /// Extracts the integer altitude in meters and converts it to millimeters.
    /// 
    /// Example: "540.5" meters becomes 540,500 mm.
    fn parse_to_mm(raw: &str) -> i32 {
        let mut parts = raw.split('.');
        let meters = parts.next().unwrap_or("0").parse::<i32>().unwrap_or(0);
        let cm = parts.next().unwrap_or("0").get(0..2).unwrap_or("0").parse::<i32>().unwrap_or(0);
        (meters * 1000) + (cm * 10)
    }
}