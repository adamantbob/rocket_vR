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
These are grouped into tasks according to where the sensor is.
TASK: On-Chip Sensors (Core Temp)
TASK: I2C Bus 0 - Accelerometer 
TASK: UART0 - GPS (10Hz).

#### Output (N tasks)
TASK: On-Chip Outputs - Pyros
TASK: PIO0(wifi) - LED Blinker (This will move to on-chip if using a regular pico)

#### Background tasks
Run the command REPL (Read Execute )
USB/900Mhz Logging
Utilization Monitoring

## Features

- **Embassy Async Framework**: Minimal power consumption and efficient task management.
- **RP2350 Support**: Targeted for the `thumbv8m.main-none-eabihf` architecture.
- **CYW43 Integration**: Controls the onboard LED connected to the WiFi chip via PIO/SPI.
- **Picotool Metadata**: Includes binary info for use with `picotool info`.

## Chip Support (Pico vs Pico 2)

This firmware supports both the Raspberry Pi Pico (RP2040) and Raspberry Pi Pico 2 (RP2350). You can switch between them using Cargo features and targets.

## Chip Support (Pico vs Pico 2)

This firmware supports both the Raspberry Pi Pico (RP2040) and Raspberry Pi Pico 2 (RP2350). Target selection is simplified via **Cargo Aliases**.

### Building and Running

You no longer need to edit `.cargo/config.toml`. Just use the following commands:

#### Pico 2 (RP2350)
```powershell
cargo pico2
```
(Or `cargo pico2-build` to just compile)

#### Pico (RP2040)
```powershell
cargo pico
```
(Or `cargo pico-build` to just compile)

## Prerequisites

1.  **Rustup**: Download Rust from https://rust-lang.org/learn/get-started/ and run the installer.
2.  **Rust Toolchain**: Install the necessary target(s):
    ```bash
    rustup target add thumbv8m.main-none-eabihf # For Pico 2
    rustup target add thumbv6m-none-eabi        # For Pico
    ```
3.  **Picotool**: Required for loading the firmware.
4.  **Firmware Files**: Ensure `43439A0.bin` and `43439A0_clm.bin` are present in `firmware/cyw43-firmware/`.

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

## Robust Panic Handling

The system features a **Bidirectional Cross-Core Panic Monitoring** system designed for high reliability:

- **Dual Records**: Each core has an independent `PanicRecord` in `.uninit` RAM. This ensures that if both cores crash simultaneously, their unique state is preserved and not overwritten.
- **Hardware Signalling**: Uses the RP2350 SIO FIFO as a hardware-level sentinel. This provides an immediate "crash detected" signal that bypasses system caches and memory.
- **Post-Mortem Reporting**: Panic information (message, file, line) is stored in persistent RAM. After a reset, the system checks both records and reports any previous crashes to the console.
- **Cross-Core Heartbeat**: A dedicated `panic_monitor_task` on each core watches the "other" core. Core 0 monitors Core 1, and Core 1 monitors Core 0.

## To-Do (Unprioritized)

- Have 'cargo build' generate the uf2 file for manually loading onto the Pi
- [x] Enable the multi-core
- Move the USB Logger over to Core 1
- [x] measure CPU consumption
- [x] Implement robust bidirectional panic monitoring
- Implement executor which allows specific ordering of tasks. i.e., always run the state machine task first in each poll loop, then the pyros, etc.
- Design abstract base tasks for Input and Output Tasks. (Don't need one for State Machine since there only be the one)
- Design the async CHANNELS in order to communicate between the tasks.
- Design the flash log storage.