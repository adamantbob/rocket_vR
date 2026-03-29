//! # rocket-drivers
//!
//! The hardware-abstraction and driver layer for the `rocket_vR` project. This crate
//! provides asynchronous drivers for all external sensors, radios, and peripherals
//! used by the flight controller.
//!
//! ## Key Drivers
//!
//! - **IMU (`imu.rs`)**: Combines multiple sensors (ADXL375, LSM6DSOX, LIS3MDL) into
//!   a high-frequency sampling task with DMA support.
//! - **GPS (`gps.rs`)**: Manages UART lifecycle and NMEA parsing for 10Hz position data.
//! - **Radio (`radio.rs`)**: Provides a LoRa-based telemetry/uplink interface using the RFM95.
//! - **SD Card (`sd_card/`)**: Handles persistent telemetry logging to microSD cards.
//! - **WiFi / CYW43 (`wifi.rs`)**: Controls the on-chip wireless and LED status on the Pico 2 W.
//!
//! ## Design Patterns
//!
//! - **Task-Based**: Each driver runs as a standalone Embassy task, sampling hardware
//!   independently and publishing results to the global Blackboard.
//! - **DMA-First**: Large data transfers (SPI, UART, PIO) leverage DMA to keep the CPU
//!   free for flight-critical logic.
//! - **Bus Sharing**: Uses `embassy-sync` Mutexes to safely share physical SPI and I2C
//!   buses across multiple sensor tasks.

#![no_std]

pub mod channels;
pub mod gps;

pub mod imu;
pub mod radio;
pub mod sd_card;
pub mod usb;
pub mod wifi;

pub use gps::*;
pub use imu::*;
pub use radio::*;
pub use sd_card::*;
pub use usb::*;
pub use wifi::*;
