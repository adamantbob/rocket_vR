use embassy_futures::select::{Either, select};
use embassy_rp::uart::{BufferedUart, Error as UartError};
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};
use embedded_io_async::{Read, Write};
use proc_macros::tracked_task;
use rocket_core::{info, warn};

#[cfg(feature = "debug-gps")]
use rocket_core::local_info;

use rocket_core::blackboard::{SENSOR_DATA, SYSTEM_HEALTH};

use rocket_core::gps::{
    GPSData, GPSHealth, VerticalKalman,
    parser::{process_line, validate_checksum},
};
use rocket_core::log::{LOG_CHANNEL, LogEntry};

/// GPS Module Task
///
/// This task manages the UART lifecycle for the GPS sensor, handles NMEA sentence
/// accumulation, executes Kalman filtering for vertical state estimation, and
/// reports health metrics to the global `SENSOR_DATA` blackboard.
/// Since the GPS task takes a BufferedUart it can be generic.
#[tracked_task]
#[embassy_executor::task]
pub async fn gps_task(mut uart: BufferedUart) -> ! {
    // --- 1. Initialization & Link Search ---
    // Probes 9600/115200 until a valid link is established and upgraded to 10Hz.
    let mut current_baud = negotiate_link_speed(&mut uart).await;

    // Task-local state
    let mut current_pos = GPSData::new();
    let mut gps_health = GPSHealth::new();
    let mut filter = VerticalKalman::new(0.0);
    let mut filter_init = false;
    let mut last_packet_time = Instant::now();

    let mut sentence_buffer = [0u8; 128];
    let mut pos = 0;

    let mut ticker = Ticker::every(Duration::from_millis(1000));

    // --- 2. Main Event Loop ---
    loop {
        let mut read_buf = [0u8; 64];

        // Concurrently wait for UART data OR the 1-second health timer
        match select(
            with_timeout(Duration::from_millis(200), uart.read(&mut read_buf)),
            ticker.next(),
        )
        .await
        {
            Either::First(read_result) => {
                match read_result {
                    Ok(Ok(n)) => {
                        gps_health.receive_timeout =
                            gps_health.receive_timeout.saturating_sub(n as u16);

                        for i in 0..n {
                            let b = read_buf[i];

                            if b != b'\n' && b != b'\r' {
                                if pos < sentence_buffer.len() {
                                    sentence_buffer[pos] = b;
                                    pos += 1;
                                }
                                continue;
                            }

                            if pos > 0 {
                                #[cfg(feature = "debug-gps")]
                                if let Ok(s) = core::str::from_utf8(&sentence_buffer[..pos]) {
                                    local_info!("GPS Raw: {}", s);
                                }

                                handle_full_sentence(
                                    &sentence_buffer[..pos],
                                    &mut current_pos,
                                    &mut gps_health,
                                    &mut filter,
                                    &mut filter_init,
                                    &mut last_packet_time,
                                );
                                pos = 0;
                            }
                        }
                    }
                    Ok(Err(_)) => {
                        gps_health.parse_errors = gps_health.parse_errors.saturating_add(1);

                        // Background fallback: If link is lost in flight, drop back to 9600 search
                        if current_baud == 115200 && gps_health.parse_errors > 100 {
                            warn!("GPS: Link lost at 115200, falling back to search...");
                            current_baud = negotiate_link_speed(&mut uart).await;
                            gps_health.parse_errors = 0;
                        }
                    }
                    Err(_) => {
                        gps_health.receive_timeout = gps_health.receive_timeout.saturating_add(100);
                        current_pos.fix_valid = false;
                        SENSOR_DATA.gps.update(current_pos);
                        let _ = LOG_CHANNEL.try_send(LogEntry::Gps(current_pos));
                    }
                }
            }
            Either::Second(_) => {
                SYSTEM_HEALTH.gps_health.update(gps_health);
                let _ = LOG_CHANNEL.try_send(LogEntry::GPSHealth(gps_health));
            }
        }
    }
}

/// Probes 9600 and 115200 until valid NMEA data is seen.
/// If valid data is found at 9600, it attempts to upgrade the module to 115200/10Hz.
async fn negotiate_link_speed(uart: &mut BufferedUart) -> u32 {
    let mut current_baud = 9600;
    uart.set_baudrate(9600);
    info!("GPS: Starting link search (9600)...");

    let mut sentence_buffer = [0u8; 128];
    let mut pos = 0;

    let mut last_switch = Instant::now();
    let mut force_switch = false;
    let mut error_count = 0;

    loop {
        let mut buf = [0u8; 64];

        // 1. Check for data using standard reader
        match with_timeout(Duration::from_millis(100), uart.read(&mut buf)).await {
            Ok(Ok(n)) => {
                error_count = 0; // Valid bytes read, reset error counter
                for i in 0..n {
                    let b = buf[i];
                    // If we see a '$' in the middle of a buffer, it's likely a sync point
                    if b == b'$' {
                        pos = 0;
                    }

                    if b != b'\n' && b != b'\r' {
                        if pos < sentence_buffer.len() {
                            sentence_buffer[pos] = b;
                            pos += 1;
                        }

                        // ULTRA-FAST LOCK: If we have at least 3 chars, check for header proof-of-life
                        if pos >= 3 {
                            if let Ok(s) = core::str::from_utf8(&sentence_buffer[..pos]) {
                                if s.starts_with("$GP") || s.starts_with("$GN") {
                                    if current_baud == 9600 {
                                        info!("GPS: Valid header at 9600. Upgrading...");
                                        upgrade_link(uart).await;
                                        return 115200;
                                    } else {
                                        info!("GPS: Valid header at 115200.");
                                        return 115200;
                                    }
                                }
                            }
                        }
                    } else {
                        pos = 0;
                    }
                }
            }
            Ok(Err(e)) => {
                // If we hit framing or break errors, we are almost certainly at the wrong baud rate.
                // If we see a few in a row, trigger an immediate baud switch instead of waiting for timeout.
                if matches!(e, UartError::Framing | UartError::Break) {
                    error_count += 1;
                    if error_count >= 3 {
                        force_switch = true;
                    }
                }
            }
            _ => {}
        }

        // 2. Check for baud rate switch (timeout or forced)
        if force_switch || last_switch.elapsed() > Duration::from_millis(1100) {
            if current_baud == 9600 {
                current_baud = 115200;
            } else {
                current_baud = 9600;
            }
            uart.set_baudrate(current_baud);
            drain_uart(uart).await;
            last_switch = Instant::now();
            force_switch = false;
            pos = 0;
            error_count = 0;
        }

        // Nap briefly (1ms) to keep it responsive
        Timer::after_millis(1).await;
    }
}
/// Commands the GPS module to increase its baudrate to 115200 and
/// its output frequency to 10Hz using PMTK proprietary commands.
async fn upgrade_link(uart: &mut BufferedUart) {
    let baud_cmd = b"$PMTK251,115200*1F\r\n";

    // Drain any boot noise
    drain_uart(uart).await;

    // Try at current/default 9600 baud
    let _ = uart.write_all(baud_cmd).await;
    Timer::after_millis(50).await;

    // Switch local UART to 115200
    uart.set_baudrate(115200);
    Timer::after_millis(10).await;
    drain_uart(uart).await;

    // Try again at 115200 in case it was already there but missed the first command
    let _ = uart.write_all(baud_cmd).await;
    Timer::after_millis(100).await;

    // 2. Set output frequency to 10Hz (100ms interval)
    let _ = uart.write_all(b"$PMTK220,100*2F\r\n").await;
    Timer::after_millis(50).await;

    // 3. Set NMEA sentences: Only GPGGA and GPRMC
    let _ = uart
        .write_all(b"$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n")
        .await;
    Timer::after_millis(50).await;
    drain_uart(uart).await;
}

async fn drain_uart(uart: &mut BufferedUart) {
    let mut buf = [0u8; 64];
    while let Ok(Ok(_)) = with_timeout(Duration::from_millis(5), uart.read(&mut buf)).await {}
}

/// Orchestrates the validation, parsing, and filtering of a complete NMEA sentence.
/// Uses guard clauses to exit early if data is corrupt or invalid.
///
/// # Arguments
///
/// * `raw` - A reference to the raw NMEA sentence as a byte slice.
/// * `pos_data` - A mutable reference to the GPSData structure to be updated.
/// * `health` - A mutable reference to the GPSHealth structure to be updated.
/// * `filter` - A mutable reference to the VerticalKalman filter to be updated.
/// * `filter_init` - A mutable reference to a boolean indicating if the filter has been initialized.
/// * `last_time` - A mutable reference to the last update time.
///
/// # Returns
///
/// * `()` - No return value. Don't return an error status as that would
/// complicate error handling in the main loop and make it harder to read.
///
/// # Side Effects
///
/// * Updates the `pos_data`, `health`, `filter`, and `filter_init` parameters.
/// * Updates the `last_time` parameter.
fn handle_full_sentence(
    raw: &[u8],
    pos_data: &mut GPSData,
    health: &mut GPSHealth,
    filter: &mut VerticalKalman,
    filter_init: &mut bool,
    last_time: &mut Instant,
) {
    // Calculate delta-time (dt) for the Kalman Filter
    let now = Instant::now();
    let dt = (now - *last_time).as_ticks() as f32 / embassy_time::TICK_HZ as f32;
    *last_time = now;

    // Guard 1: Verify NMEA Checksum
    if !validate_checksum(raw) {
        health.checksum_errors = health.checksum_errors.saturating_add(5);
        return;
    }
    health.checksum_errors = health.checksum_errors.saturating_sub(1);

    // Guard 2: Parse raw string into structured GPSData
    if let Err(_) = process_line(raw, pos_data) {
        health.parse_errors = health.parse_errors.saturating_add(5);
        return;
    }
    health.parse_errors = health.parse_errors.saturating_sub(1);

    #[cfg(feature = "debug-gps")]
    local_info!("GPS Parsed: {:?}", pos_data);

    // Corrected logic: Always commit basic data to the global state,
    // even if we haven't filtered it yet (no 3D fix).
    // This allows UI/Telemetery to see sat counts and UTC time.
    SENSOR_DATA.gps.update(*pos_data);

    // Guard 3: Only update the state filter if we have a valid 3D Fix
    if !pos_data.fix_valid {
        return;
    }

    // --- 3. Kalman Filter Update ---
    let alt_m = pos_data.raw_alt_mm as f32 / 1000.0;

    // Re-initialize filter if data is too stale or this is the first packet
    if !*filter_init || dt > 1.0 {
        *filter = VerticalKalman::new(alt_m);
        *filter_init = true;
    } else {
        // Predict and Correct step
        filter.update(dt, alt_m);
        let (est_alt, est_vel) = filter.get_state();

        // Update data structure with filtered estimates
        pos_data.alt_agl_m = est_alt;
        pos_data.velocity_z_filt = est_vel;
        // Convert f32 (m/s) to i32 (mm/s) for the state machine
        pos_data.velocity_z_mms = (est_vel * 1000.0) as i32;

        // Commit final filtered data to the global state
        SENSOR_DATA.gps.update(*pos_data);
    }
}
