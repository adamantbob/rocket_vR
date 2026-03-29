# Hardware & Wiring

This document lists the hardware components and wiring required for the `rocket_vR` project.

## Flight Controller (Rocket)

The flight controller is based on the **Raspberry Pi Pico 2 W (RP2350)**.

### Bill of Materials (BOM)

| Component | Description | Interface | Address/Baud |
| :--- | :--- | :--- | :--- |
| **MCU** | Raspberry Pi Pico 2 W (RP2350) | - | - |
| **IMU (High-G)** | ADXL375 | I2C0 | 0x53 |
| **IMU (Low-G)** | LSM6DSOX | I2C0 | 0x6A |
| **Magnetometer** | LIS3MDL | I2C0 | 0x1C |
| **GPS** | NMEA-compatible module | UART0 | 115200 (10Hz) |
| **Radio/LoRa** | RFM95 (915MHz) | SPI1 | - |
| **Storage** | microSD Card Breakout | SPI0 | - |

### Pin Assignments (RP2350)

| Function | Pin (GPIO) | Notes |
| :--- | :--- | :--- |
| **I2C0 SCL** | 5 | Shared between all IMU sensors |
| **I2C0 SDA** | 4 | Shared between all IMU sensors |
| **UART0 TX** | 12 | GPS Transmit |
| **UART0 RX** | 13 | GPS Receive |
| **SPI1 SCK** | 10 | Radio Clock |
| **SPI1 MOSI** | 11 | Radio Data Out |
| **SPI1 MISO** | 8 | Radio Data In |
| **Radio CS** | 9 | Chip Select |
| **Radio Reset** | 7 | Hardware Reset |
| **Radio DIO0** | 6 | Interrupt Pin |
| **SPI0 SCK** | 18 | SD Card Clock |
| **SPI0 MOSI** | 19 | SD Card Data Out |
| **SPI0 MISO** | 16 | SD Card Data In |
| **SD Card CS** | 17 | SD Chip Select |

---

## Base Station

The base station uses an integrated board for simplicity.

### Hardware

- **Board**: [Adafruit Feather RP2040 RFM95 (LoRa)](https://www.adafruit.com/product/5363)
- **Antenna**: 915 MHz uFL or wire antenna.

### Pin Assignments (Internal)

| Function | GPIO |
| :--- | :--- |
| **Radio SCK** | 14 |
| **Radio MOSI** | 15 |
| **Radio MISO** | 8 |
| **Radio CS** | 16 |
| **Radio Reset** | 17 |
| **Radio DIO0** | 21 |
| **NeoPixel** | 4 |
