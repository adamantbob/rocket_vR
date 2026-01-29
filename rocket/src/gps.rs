use crate::state_machine::SENSOR_DATA;
use crate::{GPS, GPSResources, Irqs, info};
use embassy_futures::select::{Either, select};
use embassy_rp::uart::{BufferedUart, Config};
use embassy_time::{Duration, Instant, Ticker, Timer, with_timeout};
use embedded_io_async::{Read, Write};
use proc_macros::tracked_task;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

mod parser;
pub mod types;
mod vertical_kalman;

use crate::gps::parser::{process_line, validate_checksum};
use crate::gps::types::*;
use crate::gps::vertical_kalman::VerticalKalman;

/// GPS Module Task
///
/// This task manages the UART lifecycle for the GPS sensor, handles NMEA sentence
/// accumulation, executes Kalman filtering for vertical state estimation, and
/// reports health metrics to the global `SENSOR_DATA` blackboard.
#[tracked_task(GPS)]
#[embassy_executor::task]
pub async fn gps_task(r: GPSResources, irqs: Irqs) -> ! {
    // --- 1. Initialization & Memory Allocation ---

    // SAFETY: We use StaticCells to ensure UART buffers live for the duration of
    // the program without being stored on the task stack (which is limited).
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

    // Initial handshake: Negotiate baudrate (9600 -> 115200) and update rate (1Hz -> 10Hz)
    upgrade_link(&mut uart).await;

    // Split the UART so we can borrow the receiver independently
    let (_tx, mut rx) = uart.split();

    // Task-local state
    let mut current_pos = GPSData::new();
    let mut gps_health = GPSHealth::new();
    let mut filter = VerticalKalman::new(0.0);
    let mut filter_init = false;
    let mut last_packet_time = Instant::now();

    // Buffer for assembling individual NMEA sentences (e.g., $GPGGA,...)
    let mut sentence_buffer = [0u8; 128];
    let mut pos = 0;

    // Ticker for periodic health reporting (1Hz)
    let mut ticker = Ticker::every(Duration::from_millis(1000));

    // --- 2. Main Event Loop ---
    loop {
        let mut byte = [0u8; 1];

        // Concurrently wait for UART data OR the 1-second health timer
        match select(
            with_timeout(Duration::from_millis(200), rx.read(&mut byte)),
            ticker.next(),
        )
        .await
        {
            // BRANCH A: UART Data Activity
            Either::First(read_result) => {
                match read_result {
                    Ok(Ok(_)) => {
                        // Successful byte read: reduce timeout "fever"
                        gps_health.receive_timeout = gps_health.receive_timeout.saturating_sub(1);
                        let b = byte[0];

                        // Accumulate bytes until a newline (\n) or carriage return (\r)
                        if b != b'\n' && b != b'\r' {
                            if pos < sentence_buffer.len() {
                                sentence_buffer[pos] = b;
                                pos += 1;
                            }
                            continue;
                        }

                        // Complete sentence detected: Process it
                        if pos > 0 {
                            handle_full_sentence(
                                &sentence_buffer[..pos],
                                &mut current_pos,
                                &mut gps_health,
                                &mut filter,
                                &mut filter_init,
                                &mut last_packet_time,
                            );
                            pos = 0; // Reset buffer pointer
                        }
                    }
                    // Handle UART errors (e.g., framing errors)
                    Ok(Err(_)) => {
                        gps_health.receive_timeout = gps_health.receive_timeout.saturating_add(1)
                    }
                    // Handle silence: No bytes received for 200ms
                    Err(_) => {
                        gps_health.receive_timeout = gps_health.receive_timeout.saturating_add(100);
                        current_pos.fix_valid = false;
                        info!("GPS Alert: Link Timeout!");

                        // Immediate broadcast of failure status
                        SENSOR_DATA.gps.update(current_pos);
                        SENSOR_DATA.gps_health.update(gps_health);
                    }
                }
            }
            // BRANCH B: Periodic Update (1Hz)
            Either::Second(_) => {
                // Periodically sync health metrics to the blackboard
                SENSOR_DATA.gps_health.update(gps_health);
            }
        }
    }
}

/// Commands the GPS module to increase its baudrate to 115200 and
/// its output frequency to 10Hz using PMTK proprietary commands.
async fn upgrade_link(uart: &mut BufferedUart) {
    // 1. Set Baudrate to 115200
    let _ = uart.write_all(b"$PMTK251,115200*1F\r\n").await;
    Timer::after_millis(100).await;
    uart.set_baudrate(115200);

    // 2. Set output frequency to 10Hz (100ms interval)
    let _ = uart.write_all(b"$PMTK220,100*2F\r\n").await;
    Timer::after_millis(50).await;

    // 3. Set NMEA sentences: Only GPGGA and GPRMC
    let _ = uart
        .write_all(b"$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n")
        .await;
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
        pos_data.velocity_z = est_vel;
    }

    // Commit final data to the global state
    SENSOR_DATA.gps.update(*pos_data);
}
