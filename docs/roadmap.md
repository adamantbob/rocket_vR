# Project Roadmap

This document outlines the current status of the `rocket_vR` project and the planned features for future releases.

## Current Status (v0.1.0 - WIP)

- [x] **Dual-Core Support**: Core 0 for I/O, Core 1 for critical flight logic.
- [x] **Priority Tasks**: 100Hz control loop with task-specific timing.
- [x] **Sensor Drivers**: Initial support for ADXL375, LSM6DSOX, LIS3MDL, and GPS.
- [x] **Panic Monitoring**: Cross-core heartbeats and persistent crash logs.
- [x] **Base Station**: Telemetry reception and LoRa link management.
- [x] **SITL Bridge**: Kerbal Space Program integration for dry-run testing.
- [x] **SD Logging**: Mirror all telemetry to an onboard microSD card for post-flight analysis.

## Near-Term Goals (Next 1-2 Months)
- [ ] **State Machine Revision**: Add more sensor data to the state machine and refine the state transitions.
- [ ] **Automated UF2 Generation**: Integrate `elf2uf2-rs` into the `cargo build` process for one-click manual loading.
- [ ] **Command Uplink**: Implement a task on the rocket to receive and handle commands from the base station.
- [ ] **GPS Optimization**: Further refinement of the Kalmann filtering for altitude estimation.
- [ ] **GPS Kalman Filter**: Refine the vertical velocity model to better handle spurious lock losses.
- [ ] **CPU Utilization Fix**: Improve individual task reporting accuracy by subtracting interrupt time (Low priority).

## Testing & Verification

Comprehensive verification of the system requirements is a high-priority goal for the next phase of development:

- [ ] **Concurrency Stress Testing**: Implement a dedicated suite for `rocket-core` to verify `DataCell` atomicity under high-frequency interrupt loads.
- [ ] **Hardware-in-the-Loop (HIL)**: Develop a test runner for the Picosystem that verifies non-deterministic behaviors like `rocket-os` panic persistence and core isolation.
- [ ] **Architecture Linting**: Integrate `cargo-deny` or custom linting to enforce the "Blackboard-only" communication boundary.
- [ ] **Stack Use Profiling**: Implement automated CI tests that trigger the "stack painting" and scanning logic to detect potential overflows.
- [ ] **Driver Anonymity Refactor**: Generalize the Radio and SD Card drivers to use generic peripherals (e.g., `<SPI: Instance>`) to enable cross-hardware reuse.

## Long-Term Vision

- **GCS Integration**: Bridge the base station telemetry to standard Ground Control Station (GCS) software.
- **Advanced State Detection**: Machine learning or more complex heuristics for apogee and landing verification.
- **Multi-Vehicle Support**: Coordination between multiple rockets/drones using a single base station.
