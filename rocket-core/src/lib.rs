// lib.rs
//! # rocket-core
//!
//! The "brain" of the `rocket_vR` project. This crate defines the fundamental types,
//! shared data structures (the Blackboard), and high-level flight logic that
//! coordinate the entire flight control system.
//!
//! ## Key Components
//!
//! - **Blackboard (`blackboard.rs`)**: A central, thread-safe hub for all system state.
//!   Uses [`DataCell`](crate::datacells::DataCell) for atomic, non-blocking access.
//! - **Flight Controller (`state_machine.rs`)**: The core logic that determines the
//!   vessel's flight state (e.g., `PoweredFlight`, `ApogeeReached`) based on sensor inputs.
//! - **Logging (`log.rs`, `macros.rs`)**: A multi-sink asynchronous logging pipeline that
//!   simultaneously targets RTT (live) and SD Card (persistent) storage.
//! - **Health Monitoring (`health_types.rs`, `utilization.rs`)**: Types and logic for
//!   tracking CPU utilization, stack watermarks, and sensor health.
//!
//! ## Principles
//!
//! - **Deterministic**: Uses fixed-point arithmetic ([`DeciPercent`](crate::health_types::DeciPercent))
//!   to ensure identical behavior across different hardware (SITL vs. Pico 2).
//! - **No-Heap**: Designed for `no_std` environments with zero dynamic allocation.
//! - **Decoupled**: All communication between drivers and logic must pass through the
//!   Blackboard to maintain a clean modular boundary.

#![no_std]

pub use log as external_log;

#[macro_use]
pub mod macros;

pub mod blackboard;
pub mod datacells;
pub mod gps;
pub mod health_types;
pub mod imu;
pub mod log;
pub mod radio_types;
pub mod state_machine;
pub mod types;
pub mod utilization;

pub use gps::parser::{process_line, validate_checksum};
pub use gps::types::*;
pub use gps::vertical_kalman::VerticalKalman;
pub use health_types::*;
pub use imu::types::*;
pub use state_machine::*;
pub use types::*;
