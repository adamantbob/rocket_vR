use crate::{FlightTicks, GPSData, GPSHealth, IMUData, IMUHealth};
use core::fmt::Write;
use core::sync::atomic::Ordering;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Instant, TICK_HZ};
use portable_atomic::AtomicU32;
use proc_macros::TelemetryPayload;

// A safe upper bound for any single CSV row (Tag + Timestamp + Data + Newline)
pub const MAX_LOG_LINE_LEN: usize = 256;

/// Global counter for logs that were dropped because the channel was full
pub static DROPPED_LOGS: AtomicU32 = AtomicU32::new(0);

#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum LogLevel {
    Info,
    Warn,
    Error,
}

impl LogLevel {
    pub const fn tag(&self) -> &'static str {
        match self {
            LogLevel::Info => "I",
            LogLevel::Warn => "W",
            LogLevel::Error => "E",
        }
    }
}

#[derive(Clone)]
pub enum LogEntry {
    Imu(IMUData),
    Gps(GPSData),
    IMUHealth(IMUHealth),
    GPSHealth(GPSHealth),
    Event(&'static str),
    Log(LogLevel, heapless::String<32>),
    LoggerHealth(LoggerHealth),
}
impl LogEntry {
    pub fn write_schema<const SIZE: usize>(cursor: &mut LogBuffer<SIZE>) -> core::fmt::Result {
        writeln!(cursor, "# SCHEMA DEFINITION")?;
        writeln!(cursor, "# METADATA,tick_hz,{}", TICK_HZ)?;
        writeln!(
            cursor,
            "# {},tickstamp,{}",
            IMUData::TAG,
            IMUData::CSV_HEADER
        )?;
        writeln!(
            cursor,
            "# {},tickstamp,{}",
            IMUHealth::TAG,
            IMUHealth::CSV_HEADER
        )?;
        writeln!(
            cursor,
            "# {},tickstamp,{}",
            GPSData::TAG,
            GPSData::CSV_HEADER
        )?;
        writeln!(
            cursor,
            "# {},tickstamp,{}",
            GPSHealth::TAG,
            GPSHealth::CSV_HEADER
        )?;
        writeln!(cursor, "# E,tickstamp,event_msg")?;
        writeln!(cursor, "# L,tickstamp,level,msg")?;
        writeln!(
            cursor,
            "# {},tickstamp,{}",
            LoggerHealth::TAG,
            LoggerHealth::CSV_HEADER
        )?;
        Ok(())
    }

    pub fn format_to<const SIZE: usize>(&self, cursor: &mut LogBuffer<SIZE>) -> core::fmt::Result {
        match self {
            LogEntry::Imu(data) => self.write_line(data.tickstamp, data, cursor),
            LogEntry::Gps(data) => self.write_line(data.tickstamp, data, cursor),
            LogEntry::IMUHealth(data) => self.write_line(data.tickstamp, data, cursor),
            LogEntry::GPSHealth(data) => self.write_line(data.tickstamp, data, cursor),
            LogEntry::Event(data) => {
                self.write_line(Instant::now().as_ticks() as u64, data, cursor)
            }
            LogEntry::Log(level, msg) => {
                let ts = Instant::now().as_ticks() as u64;
                write!(cursor, "{},{},{},", "L", ts, level.tag())?;
                cursor.write_str(msg.as_str())?;
                write!(cursor, "\n")?;
                Ok(())
            }
            LogEntry::LoggerHealth(data) => self.write_line(data.tickstamp, data, cursor),
        }
    }

    // This is the "Contract" - it forces the format: TAG, TIMESTAMP, PAYLOAD... \n
    fn write_line<T: Loggable, const SIZE: usize>(
        &self,
        ts: FlightTicks,
        data: &T,
        cursor: &mut LogBuffer<SIZE>,
    ) -> core::fmt::Result {
        write!(cursor, "{},{},", T::TAG, ts)?;
        data.format_payload(cursor)?;
        write!(cursor, "\n")?;
        Ok(())
    }
}
// A trait for types that can be logged
pub trait Loggable {
    /// The 'Tag' that identifies this row (e.g., 'I', 'G', 'H')
    const TAG: &'static str;

    /// Only write the fields and commas. Do NOT write the tag, timestamp, or \n.
    fn format_payload<const SIZE: usize>(&self, cursor: &mut LogBuffer<SIZE>) -> core::fmt::Result;
}

// Capacity of 128 ensures we can handle bursts of data from multiple sensors
pub static LOG_CHANNEL: Channel<CriticalSectionRawMutex, LogEntry, 128> = Channel::new();

#[derive(Clone, Copy, Debug, TelemetryPayload)]
pub struct LoggerHealth {
    /// Tickstamp of when the health was captured.
    pub tickstamp: FlightTicks,
    pub sd_card_ready: bool,
    pub dropped_logs: u32,
    pub channel_len: usize,
    pub channel_cap: usize,
}

impl LoggerHealth {
    pub fn capture(sd_ready: bool) -> Self {
        Self {
            tickstamp: Instant::now().as_ticks() as u64,
            sd_card_ready: sd_ready,
            dropped_logs: DROPPED_LOGS.load(Ordering::Relaxed),
            channel_len: LOG_CHANNEL.len(),
            channel_cap: 128, // Matches Channel capacity
        }
    }
}

impl Loggable for LoggerHealth {
    const TAG: &'static str = "LH";
    fn format_payload<const SIZE: usize>(&self, cursor: &mut LogBuffer<SIZE>) -> core::fmt::Result {
        write!(
            cursor,
            "{},{},{},{}",
            self.sd_card_ready, self.dropped_logs, self.channel_len, self.channel_cap
        )
    }
}

impl Loggable for &'static str {
    const TAG: &'static str = "E";
    fn format_payload<const SIZE: usize>(&self, cursor: &mut LogBuffer<SIZE>) -> core::fmt::Result {
        // Just write the string directly
        cursor.write_str(self)
    }
}

// Helper for formatting into a buffer
#[repr(align(4))]
pub struct LogBuffer<const SIZE: usize> {
    buf: [u8; SIZE],
    pub pos: usize,
}

impl<const SIZE: usize> core::fmt::Write for LogBuffer<SIZE> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        let bytes = s.as_bytes();
        let remainder = self.buf.len() - self.pos;
        if remainder < bytes.len() {
            return Err(core::fmt::Error);
        }

        self.buf[self.pos..self.pos + bytes.len()].copy_from_slice(bytes);
        self.pos += bytes.len();
        Ok(())
    }
}

impl<const SIZE: usize> LogBuffer<SIZE> {
    pub fn new() -> Self {
        Self {
            buf: [0u8; SIZE],
            pos: 0,
        }
    }
    /// Attempts to format a LogEntry into the buffer.
    /// Returns the number of bytes written, or an error if it doesn't fit.
    pub fn write_entry(&mut self, entry: &LogEntry) -> Result<usize, core::fmt::Error> {
        let start_pos = self.pos;

        // Use the Entry's format_to logic, passing 'self' as the formatter
        entry.format_to(self)?;

        // Calculate how much we just added
        Ok(self.pos - start_pos)
    }

    pub fn space_remaining(&self) -> usize {
        self.buf.len() - self.pos
    }
    pub fn get_active_buffer(&self) -> &[u8] {
        &self.buf[..self.pos]
    }
    pub fn reset(&mut self) {
        self.pos = 0;
    }
}
