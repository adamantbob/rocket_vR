# Project Roadmap

This document outlines the current status of the `rocket_vR` project and the planned features for future releases.

## Current Status (v0.1.0 - WIP)

- [x] **Dual-Core Support**: Core 0 for I/O, Core 1 for critical flight logic.
- [x] **Priority Tasks**: 100Hz control loop with task-specific timing.
- [x] **Sensor Drivers**: Initial support for ADXL375, LSM6DSOX, LIS3MDL, and GPS.
- [x] **Panic Monitoring**: Cross-core heartbeats and persistent crash logs.
- [x] **Base Station**: Telemetry reception and LoRa link management.
- [x] **SITL Bridge**: Kerbal Space Program integration for dry-run testing.

## Near-Term Goals (Next 1-2 Months)
- [ ] **State Machine Revision**: Add more sensor data to the state machine and refine the state transitions.
- [ ] **Automated UF2 Generation**: Integrate `elf2uf2-rs` into the `cargo build` process for one-click manual loading.
- [ ] **Command Uplink**: Implement a task on the rocket to receive and handle commands from the base station.
- [ ] **SD Logging**: Mirror all telemetry to an onboard microSD card for post-flight analysis.
- [ ] **GPS Optimization**: Further refinement of the Kalmann filtering for altitude estimation.

## Long-Term Vision

- **GCS Integration**: Bridge the base station telemetry to standard Ground Control Station (GCS) software.
- **Advanced State Detection**: Machine learning or more complex heuristics for apogee and landing verification.
- **Multi-Vehicle Support**: Coordination between multiple rockets/drones using a single base station.
