# SITL Quick-Start Guide: KSP Integration

`rocket_vR` features a dedicated **Software-in-the-Loop (SITL)** bridge that binds the core flight logic to the physics engine of **Kerbal Space Program (KSP)**. This allows you to verify state transitions and staging logic in a high-fidelity simulator before flight.

## Prerequisites

1.  **Kerbal Space Program**: Installed on your PC.
2.  **kRPC Plugin**: The [kRPC - Remote Procedure Call Server](https://krpc.github.io/krpc/getting-started.html) mod must be installed in KSP.
3.  **Active kRPC Server**: Start KSP, enter a flight scene, and ensure the kRPC server is "Started" in the kRPC window (default port: 50000).

## Running the Bridge

The bridge acts as a translation layer between KSP and the `rocket-core` flight controller.

1.  **Open a Terminal**: Navigate to the root of the project.
2.  **Run the SITL Package**:
    ```bash
    cargo run -p rocket-sitl-ksp
    ```
3.  **Simulation Flow**:
    - The bridge will connect to KSP and identify your active vessel.
    - It triggers an automatic **Calibration** phase to find altitude and gravity offsets.
    - Upon completion, the bridge will automatically ignite the first stage and start the mission clock.

## Key Features

- **Automatic Staging**: If the `FlightController` detects burnout, it will trigger the next stage (`Space`) in KSP.
- **Telemetry Mirroring**: Real-time altitude, velocity, and state information are printed to the console at 10Hz.
- **Latency Monitoring**: The bridge tracks the sample rate of both the KSP telemetry (`Tel`) and the internal State Machine (`SM`) to verify timing health.

## Tips for Testing

- **Abort Signal**: Press `Ctrl+C` in the terminal to stop the bridge and return control to the manual KSP interface.
- **Vessel Switching**: The bridge defaults to the "Active Vessel". If you switch vessels in KSP, the bridge will attempt to re-sync.
- **Parachutes**: Apogee detection in `rocket-core` will trigger parachute deployment (Stage 3) automatically in the simulation.
