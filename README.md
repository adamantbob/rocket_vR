# rocket_vR

An embassy-based Rust firmware for the Raspberry Pi Pico 2 W (RP2350).

The system uses both cores to isolated tasks. Rough Architecture (WIP):

Core 0 (main core):
- Communication with peripherals:
- - CYW43 Wifi Chip (On board LED Control)
- Communication with Command REPL via USB Serial

Core 1 (Logging Core):
- Logging to USB Serial.

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
#TODO: Make cargo build generate the UF2 file for manually loading onto a Pi.

## Monitoring

The project uses `defmt` for logging. You can monitor logs using `probe-rs` or other compatible RTT consumers if using a debugger, although `picotool` is the default runner configured here.

Target is application without probe-rs and SWD debuggers. It should print debug output to a USB COM port.

## To-Do (Unprioritized)

- Have 'cargo build' generate the uf2 file for manually loading onto the Pi
- Enable the multi-core
- Move the USB Logger over to Core 1
- measure CPU consumption