use crate::{Irqs, GPSResources, info};
use embassy_rp::Peri;
use embassy_rp::interrupt::typelevel::Binding;
use embassy_rp::uart::{
    BufferedInterruptHandler, BufferedUart, BufferedUartRx, Config, Instance, RxPin, TxPin,
};
use embassy_time::Timer;
use embedded_io_async::{Read, Write};
use proc_macros::tracked_task;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

#[derive(Debug, Default, Clone, Copy, defmt::Format)]
pub struct GpsData {
    pub latitude: i32,  // Decidegrees * 10^5 (e.g., 45.12345 -> 4512345)
    pub longitude: i32,
    pub altitude: f32,  // Meters (f32 is okay for altitude)
    pub satellites: u8,
    pub fix_valid: bool,
}

pub struct GPSManager;

impl GPSManager {
    pub async fn init(
        r: crate::GPSResources, // Match your macro case
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
                        Self::process_line(sentence);
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

    fn process_line(line: &[u8]) {
        if let Ok(sentence) = core::str::from_utf8(line) {
            let mut data = GpsData::default();

            if sentence.starts_with("$GPGGA") {
                let mut fields = sentence.split(',');
                
                // Index 0: $GPGGA
                // Index 1: Time
                fields.next(); fields.next(); 

                // Index 2 & 3: Latitude & N/S
                let lat_raw = fields.next().unwrap_or("");
                let lat_dir = fields.next().unwrap_or("");
                
                // Index 4 & 5: Longitude & E/W
                let lon_raw = fields.next().unwrap_or("");
                let lon_dir = fields.next().unwrap_or("");

                // Index 6: Fix Quality (0 = Invalid, 1 = GPS fix, 2 = DGPS fix)
                let quality = fields.next().unwrap_or("0");
                data.fix_valid = quality != "0";

                // Index 7: Number of Satellites
                data.satellites = fields.next().unwrap_or("0").parse::<u8>().unwrap_or(0);

                // Index 9: Altitude
                fields.next(); // skip HDOP
                data.altitude = fields.next().unwrap_or("0.0").parse::<f32>().unwrap_or(0.0);

                if data.fix_valid {
                    data.latitude = Self::parse_degrees(lat_raw, lat_dir);
                    data.longitude = Self::parse_degrees(lon_raw, lon_dir);
                    
                    info!("FIX! Satellites: {}, Alt: {}m, Lat: {}, Lon: {}", 
                        data.satellites, data.altitude, data.latitude, data.longitude);
                } else {
                    info!("GPS: Searching... (Sats: {})", data.satellites);
                }
            }
        }
    }

    /// Converts NMEA "DDMM.MMMM" format to fixed-point micro-degrees
    fn parse_degrees(raw: &str, dir: &str) -> i32 {
        if raw.is_empty() { return 0; }
        
        // NMEA Lat: DDMM.MMMM | Lon: DDDMM.MMMM
        let dot_pos = raw.find('.').unwrap_or(0);
        if dot_pos < 2 { return 0; }

        let degrees_str = &raw[..dot_pos-2];
        let minutes_str = &raw[dot_pos-2..];

        let degrees = degrees_str.parse::<i32>().unwrap_or(0);
        let minutes = minutes_str.parse::<f32>().unwrap_or(0.0);

        // Convert to decimal degrees: Degrees + (Minutes / 60)
        let mut total = (degrees * 100_000) + ((minutes / 60.0) * 100_000.0) as i32;

        if dir == "S" || dir == "W" {
            total = -total;
        }
        total
    }
}