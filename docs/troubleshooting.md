# Troubleshooting Guide

This guide helps you resolve common issues encountered when building, flashing, or running the `rocket_vR` firmware.

## Build and Flash Issues

### `cargo pico2` command not found
**Solution**: Ensure you have configured the Cargo aliases in `.cargo/config.toml`. If you're running from the root of the workspace, these aliases are already defined.

### `probe-rs` fails to find the device
**Solution**: 
1.  Check the USB cable (data cables only, not power-only).
2.  If using the Pico 2 W, hold the **BOOTSEL** button while plugging it in to enter bootloader mode.
3.  Ensure `picotool` is installed and in your system PATH.

## Sensor Issues

### GPS Not Locking
**Symptom**: `GPS: Starting link search (9600)...` loops indefinitely in the console.
**Solution**:
1.  Check TX/RX wiring. TX on GPS should go to RX on Pico (GP13).
2.  Ensure the GPS module has a clear view of the sky.
3.  Verify the module is receiving 3.3V or 5V (depending on the breakout).
4.  Check for NMEA activity with a logic analyzer or serial monitor.

### IMU Initialization Timed Out
**Symptom**: `IMU Initialization Timed Out - check wiring/power` in the logs.
**Solution**:
1.  Check I2C wiring (SDA/SCL) and pull-up resistors.
2.  Ensure all three sensors (LSM6DSOX, ADXL375, LIS3MDL) are powered on the same bus.
3.  Check for I2C address conflicts. Defaults are 0x53, 0x6A, and 0x1C.

### "STALE" Data Flags
**Symptom**: The State Machine reports that data from a task is `STALE`.
**Solution**:
1.  Increase the task priority if the system is under heavy load.
2.  Check the sensor's update rate (e.g., GPS is 10Hz, IMU is 100Hz).
3.  Check the `utilization` logs to see if a task is taking too long to process.

## Networking and Telemetry

### No Telemetry on Base Station
**Symptom**: Base station NeoPixel pulses green, but no logs appear.
**Solution**:
1.  Ensure both the rocket and base station are on the same frequency (default: 915 MHz).
2.  Check the LoRa antenna connections.
3.  Verify the `Spreading Factor` and `Bandwidth` match in both crates.
