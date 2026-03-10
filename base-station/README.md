# Rocket vR — Base Station

This crate provides a simple ground station firmware for the `rocket_vR` project, designed to receive and log telemetry packets from the rocket in real-time.

## Hardware Requirements

- **Integrated Board**: [Adafruit Feather RP2040 RFM95 (LoRa)](https://www.adafruit.com/product/5363)
- **USB Cable**: USB-C for power and data (RTT logging)
- **Antenna**: 915 MHz uFL or wire antenna

## Pin Assignments (Integrated Board)

| Function          | GPIO  | Notes                               |
|-------------------|-------|-------------------------------------|
| Radio SPI1 SCK    | 14    | Internal SPI1 SCK                   |
| Radio SPI1 MOSI   | 15    | Internal SPI1 MOSI                  |
| Radio SPI1 MISO   | 8     | Internal SPI1 MISO                  |
| Radio CS          | 16    | RFM_CS                              |
| Radio RESET       | 17    | RFM_RST                             |
| Radio DIO0        | 21    | RFM_IO0 (Interrupt)                 |
| NeoPixel Data     | 4     | Status NeoPixel                     |

> [!NOTE]
> The NeoPixel is powered directly from the 3.3V supply; no power-enable pin is used on this board.

## LoRa Configuration

Matches the transmit side in the `rocket` crate:
- **Frequency**: 915 MHz
- **Spreading Factor**: SF9
- **Bandwidth**: 125 kHz
- **Coding Rate**: 4/5

## Status Indication (NeoPixel)

- **Pulsing Green (100ms on, 900ms off)**: System is alive and listening for packets.

## Build and Flash

From the workspace root, use the provided cargo aliases:

```powershell
# Build and flash via probe-rs
cargo run-base

# Build only
cargo build-base

# Build and generate a UF2 file for bootloader drag-and-drop
cargo uf2-base
```

## Architecture

The firmware uses the single-core [Embassy](https://embassy.dev/) executor:
1.  **Blinky Task**: Handles the NeoPixel pulse using the RP2040 PIO.
2.  **Radio RX Task**: Wraps the `Rfm95` driver in receive mode, deserializing incoming `TelemetryPacket`s and logging them via `defmt`.

## Packet format

The base station expects the `TelemetryPacket` format defined in `rocket-core`. Every received packet is logged with its RSSI (Received Signal Strength Indicator) and SNR (Signal-to-Noise Ratio).

## Future Work

- **USB Serial Output**: Bridge received telemetry to a standard serial port for external GCS software (like GMAT or custom dashboards).
- **Command Uplink**: Add a task to send commands (e.g., flight state overrides) back to the rocket.
- **SD Logging**: Mirror telemetry to an onboard SD card if a FeatherWing is added.
