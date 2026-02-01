# KSP SITL Bridge

This crate provides a Software-in-the-Loop (SITL) bridge between Kerbal Space Program (KSP) and the `rocket-core` flight logic. It allows you to test your state machine transitions and sensor processing using real-time telemetry from a simulated rocket launch in KSP.

## Prerequisites

1.  **Kerbal Space Program**: A working installation of KSP.
2.  **kRPC Mod**: You must have the [kRPC mod](https://krpc.github.io/krpc/index.html) installed in your KSP `GameData` folder. 
    *   kRPC allows external programs (like this Rust crate) to interact with KSP via Remote Procedure Calls.

## Setup Instructions

### 1. Start KSP and Prepare the Vessel
*   Launch Kerbal Space Program.
*   Enter the **Vehicle Assembly Building (VAB)** and load or build your rocket.
*   Move the vessel to the **Launchpad**.

### 2. Configure the kRPC Server
*   In the flight view, find the **kRPC Icon** on the sidebar or in the app toolbar.
*   Open the kRPC window.
*   Ensure the server is **Started**.
*   Note the **RPC Port** (default: `50000`) and **Stream Port** (default: `50001`).
*   Ensure the **Address** is set to `127.0.0.1` (or whichever IP you are connecting from).

### 3. Connect the Bridge
Open a terminal in the project root and run:

```powershell
cargo run -p rocket-sitl-ksp
```

The bridge will:
1.  Attempt to connect to the kRPC server.
2.  Identify the active vessel.
3.  Begin streaming telemetry at 10Hz.
4.  Feed the telemetry into a local instance of the `RocketStateMachine`.

## Telemetry Mapping

The bridge currently maps the following KSP data points:

| KSP Telemetry | Rocket-Core Mapping | Notes |
| :--- | :--- | :--- |
| `Mean Altitude` | `gps_data.raw_alt_mm` | Scaled to millimeters |
| `Vertical Speed` | `gps_data.speed_mm_per_sec` | Absolute value in mm/s |
| `G-Force` | `imu_data.accel_high_g[2]` | Vertical acceleration in milli-Gs |

## Usage
As you launch and fly your rocket in KSP, the bridge will output the current flight state detected by the software logic. Look for `STATE CHANGE` messages in the terminal to verify that your launch, apogee, and landing detection codes are working correctly.
