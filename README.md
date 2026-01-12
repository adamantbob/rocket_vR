# rocket_vR

An embassy-based Rust firmware for the Raspberry Pi Pico 2 W (RP2350).

## Designs

The system uses both cores to isolated tasks. Rough Architecture (WIP):

### Timing

The system is roughly targetting 100Hz. It is set up with a n -> 1 task layout.

With the following setup, the data recorded by the system will be at most a single tick old. Rough Timing Chart:

| Time | State Machine Task                     | Input Task              | Output Task         |
| ---- | -------------------------------------- | ----------------------- | ------------------- |
| Tick | Copy data from input CHANNELS          | _waiting_               | _waiting_           |
| +1   | Calculate **State**                    | _waiting_               | _waiting_           |
| +2   | store data into output CHANNELS        | _waiting_               | _waiting_           |
| +3   | Signal to Output & Data Tasks to Start | _waiting_               | _waiting_           |
| +4   | Start STORE operation into Flash       | *Start Input*           | *Start Output*      |
| +5   | _waiting_                              | Send communications     | Set Outputs         |
| +5   | _waiting_                              | _waiting_               | *End Output*        |
|...   ||||
| +n   | Store completes, await next Tick       | _waiting_               | _waiting_           |
|...   ||||
| +m   | _waiting_                              | Comm Received           | _waiting_           |
| +m   | _waiting_                              | data conversion         | _waiting_           |
| +m1  | _waiting_                              | store data into CHANNEL | _waiting_           |
| +m1  | _waiting_                              | *End Input*             | _waiting_           |
|Go to Tick ||| |

On execution order: 
- The executor uses a run-queue to determine execution order, and the order in which items execute is not guaranteed. 
- times +n and +m will not happen in that order. That's intentional as they wait for communication.
- If data is not arrived by the TICK time for an input task, the task will set the signal indicating it's data is STALE. 
TODO: Implement executor which allows specific ordering of tasks. i.e., always run the state machine task first in each poll loop, then the pyros, etc.

### Task Types

#### State Machine (Singleton)
TASK: Using the embassy_timer::Ticker, animate the state machine, execute storing the data every tick, then signal to data collection.
The State Machine Task will contain all the business logic (i.e. in this state pyro 1 should be firing)
Question: Should the state machine disable some tasks from executing? 
- Example, if the state is "BURN", should the usb repl be disabled? <- Probably not, this would be annoying for debugging
- Example, if the state is "LANDED", should the pyro be disabled from running? <- Hmm, maybe safety wise this would be helpful?

#### Input (N tasks)
For offboard sensors, depending on how quickly they can be queried. Each will:
1. Start Collection of the latest data from their associated data.
2. Yeild until there is a response from the sensor.
3. When data arrives, store that data in a CHANNEL for retrieval by the data collection task.
4. Yeild until signal from data logger task to seek new data.
TASK: GPS (10Hz).
TASK: Accelerometer (100Hz). 

#### Output (N tasks)
TASK: LED Blinker
TASK: Fire the Pyros

#### Background tasks
Run the command REPL (Read Execute )
USB/900Mhz Logging

## Features

- **Embassy Async Framework**: Minimal power consumption and efficient task management.
- **RP2350 Support**: Targeted for the `thumbv8m.main-none-eabihf` architecture.
- **CYW43 Integration**: Controls the onboard LED connected to the WiFi chip via PIO/SPI.
- **Picotool Metadata**: Includes binary info for use with `picotool info`.

## Prerequisites

1.  **Rustup**: Download Rust from https://rust-lang.org/learn/get-started/ and run the installer.
2.  **Rust Toolchain**: Install Rust and the necessary target:
    ```bash
    rustup target add thumbv8m.main-none-eabihf
    ```
3.  **Picotool**: Required for loading the firmware onto the device.
4.  **Firmware Files**: Ensure `43439A0.bin` and `43439A0_clm.bin` are present in `firmware/cyw43-firmware/` as they are bundled into the binary.

## Project Structure

- `src/main.rs`: Core application logic, including the blink loop and setup.
- `src/instrumented_executor.rs`: Executor with additional statistics tracking.
- `src/utilization.rs`: CPU utilization monitoring system.
- `src/usb.rs`: USB Serial setup and REPL.
- `memory.x`: Memory layout for the RP2350.
- `build.rs`: Minimal build script for dependency management and ensuring rebuilds on memory changes.
- `.cargo/config.toml`: Configures the runner (`picotool`) and necessary linker flags (`-Tlink.x`, `-Tdefmt.x`).

## Building and Running

Connect your Pico 2 in BOOTSEL mode and run:

```powershell
cargo run
```

This will build the project and use `picotool` to load and execute it. 

Alternatively, to just build:

```powershell
cargo build
```

## Monitoring

The project uses `defmt` for logging. You can monitor logs using `probe-rs` or other compatible RTT consumers if using a debugger, although `picotool` is the default runner configured here.

Target is application without probe-rs and SWD debuggers. It should print debug output to a USB COM port.

## CPU Utilization Tracking

The system includes built-in CPU utilization tracking for individual tasks and total system usage. By default, it operates in an **Alarm Mode** to minimize log noise:

- **Silent**: During normal operation (all tasks < 10% and total usage < 40%).
- **Warning**: Logs a warning if any task exceeds 10% or total usage exceeds 40%.
- **Alarm**: Logs an alarm if total usage exceeds 80%.

### Verbose Mode
To enable constant, second-by-second reporting for debugging, build with the `verbose-utilization` feature:

```powershell
cargo run --features verbose-utilization
```

## To-Do (Unprioritized)

- Have 'cargo build' generate the uf2 file for manually loading onto the Pi
- Enable the multi-core
- Move the USB Logger over to Core 1
- [x] measure CPU consumption
- Implement executor which allows specific ordering of tasks. i.e., always run the state machine task first in each poll loop, then the pyros, etc.
- Design abstract base tasks for Input and Output Tasks. (Don't need one for State Machine since there only be the one)
- Design the async CHANNELS in order to communicate between the tasks.
- Design the flash log storage. 