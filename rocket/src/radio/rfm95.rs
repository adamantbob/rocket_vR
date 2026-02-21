//! RFM95/96/97/98 LoRa driver for Embassy on RP2350.
//!
//! # Wiring Requirements
//!
//! | RFM95 Pin | Function                        | Required |
//! |-----------|----------------------------------|----------|
//! | NSS       | SPI chip select (active low)    | Yes      |
//! | SCK       | SPI clock                       | Yes      |
//! | MOSI      | SPI data in                     | Yes      |
//! | MISO      | SPI data out                    | Yes      |
//! | RESET     | Active-low hardware reset        | Yes      |
//! | DIO0      | TX-done / RX-done interrupt     | **Yes**  |
//! | 3.3V      | Power supply                    | Yes      |
//! | GND       | Ground                          | Yes      |
//!
//! **DIO0 must be connected** to a GPIO pin capable of edge detection.
//! The driver awaits a rising edge on DIO0 to detect TX/RX completion.
//! Without this connection the driver will hang indefinitely on transmit
//! and receive operations.
//!
//! DIO0 is configured by the radio to signal:
//! - `RxDone` in continuous RX mode
//! - `TxDone` after a transmit completes
//!
//! # Usage
//!
//! ```rust
//! let config = LoRaConfig {
//!     frequency_hz: 915_000_000,
//!     spreading_factor: SpreadingFactor::SF9,
//!     bandwidth: Bandwidth::BW125,
//!     coding_rate: CodingRate::CR4_5,
//!     tx_power_dbm: 17,
//! };
//!
//! let mut radio = Rfm95::new(spi_device, reset_pin, dio0_pin, config).await?;
//!
//! // Transmit a packet
//! let packet = TelemetryPacket { altitude_mm: 5000, velocity_mms: 100 };
//! radio.transmit(&packet).await?;
//!
//! // Receive a packet
//! let received: TelemetryPacket = radio.receive().await?;
//! ```
//!
//! # Packet Format
//!
//! Packets are serialized with [`postcard`] and framed with COBS encoding.
//! Hardware CRC is enabled on the RFM95, providing a second layer of error
//! detection on top of postcard's own integrity checking.

use embassy_time::{Duration, Timer};
use embedded_hal_async::digital::Wait;
use embedded_hal_async::spi::SpiDevice;
use postcard::{from_bytes, to_slice};
use serde::{Deserialize, Serialize};

// ---------------------------------------------------------------------------
// Register map (LoRa mode)
// ---------------------------------------------------------------------------

#[allow(dead_code)]
mod regs {
    pub const FIFO: u8 = 0x00;
    pub const OP_MODE: u8 = 0x01;
    pub const FR_MSB: u8 = 0x06;
    pub const FR_MID: u8 = 0x07;
    pub const FR_LSB: u8 = 0x08;
    pub const PA_CONFIG: u8 = 0x09;
    pub const PA_DAC: u8 = 0x4D;
    pub const LNA: u8 = 0x0C;
    pub const FIFO_ADDR_PTR: u8 = 0x0D;
    pub const FIFO_TX_BASE_ADDR: u8 = 0x0E;
    pub const FIFO_RX_BASE_ADDR: u8 = 0x0F;
    pub const FIFO_RX_CURRENT_ADDR: u8 = 0x10;
    pub const IRQ_FLAGS_MASK: u8 = 0x11;
    pub const IRQ_FLAGS: u8 = 0x12;
    pub const RX_NB_BYTES: u8 = 0x13;
    pub const PKT_SNR_VALUE: u8 = 0x19;
    pub const PKT_RSSI_VALUE: u8 = 0x1A;
    pub const MODEM_CONFIG1: u8 = 0x1D;
    pub const MODEM_CONFIG2: u8 = 0x1E;
    pub const MODEM_CONFIG3: u8 = 0x26;
    pub const PAYLOAD_LENGTH: u8 = 0x22;
    pub const MAX_PAYLOAD_LENGTH: u8 = 0x23;
    pub const SYNC_WORD: u8 = 0x39;
    pub const DIO_MAPPING1: u8 = 0x40;
    pub const VERSION: u8 = 0x42;
}

// IRQ flag bits
const IRQ_RX_DONE: u8 = 0x40;
const IRQ_TX_DONE: u8 = 0x08;
const IRQ_PAYLOAD_CRC_ERROR: u8 = 0x20;
const IRQ_RX_TIMEOUT: u8 = 0x80;

// OpMode register values
const MODE_SLEEP: u8 = 0x00;
const MODE_STDBY: u8 = 0x01;
const MODE_TX: u8 = 0x03;
const MODE_RX_CONTINUOUS: u8 = 0x05;
const MODE_LONG_RANGE: u8 = 0x80; // LoRa mode bit

// PA_CONFIG: use PA_BOOST pin (required for RFM95)
const PA_BOOST: u8 = 0x80;

// Sync word: 0x12 = private network (use 0x34 for LoRaWAN)
const LORA_SYNC_WORD: u8 = 0x12;

// Expected VERSION register value for RFM95/96/97/98
const RFM95_VERSION: u8 = 0x12;

// Crystal oscillator frequency used for frequency calculations
const FXOSC: u64 = 32_000_000;
// Frequency step = FXOSC / 2^19
const FSTEP: u64 = (FXOSC << 8) / 500_000;

// Maximum LoRa payload (hardware limit)
const MAX_PAYLOAD: usize = 255;
// Our practical buffer including postcard + COBS overhead
pub const PACKET_BUFFER_SIZE: usize = 128;

// ---------------------------------------------------------------------------
// Configuration types
// ---------------------------------------------------------------------------

/// LoRa spreading factor. Higher SF = longer range, lower data rate.
///
/// For rocket telemetry SF9 is a good balance — ~500 bytes/s at BW125,
/// robust to Doppler shift, and handles ~15km range with a good antenna.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SpreadingFactor {
    SF6 = 6,
    SF7 = 7,
    SF8 = 8,
    SF9 = 9,
    SF10 = 10,
    SF11 = 11,
    SF12 = 12,
}

/// LoRa bandwidth. Wider bandwidth = higher data rate, shorter range.
///
/// BW125 is the standard choice for most applications.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Bandwidth {
    BW7_8 = 0,
    BW10_4 = 1,
    BW15_6 = 2,
    BW20_8 = 3,
    BW31_25 = 4,
    BW41_7 = 5,
    BW62_5 = 6,
    BW125 = 7,
    BW250 = 8,
    BW500 = 9,
}

/// LoRa forward error correction coding rate.
///
/// Higher denominator = more redundancy = better error correction at the
/// cost of data rate. CR4_5 is the standard choice.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CodingRate {
    CR4_5 = 1,
    CR4_6 = 2,
    CR4_7 = 3,
    CR4_8 = 4,
}

/// Full radio configuration.
#[derive(Clone, Copy, Debug)]
pub struct LoRaConfig {
    /// Centre frequency in Hz. Must be in the 902–928 MHz US ISM band.
    pub frequency_hz: u32,
    pub spreading_factor: SpreadingFactor,
    pub bandwidth: Bandwidth,
    pub coding_rate: CodingRate,
    /// TX output power in dBm. Range: 2–20 dBm (PA_BOOST path).
    /// 17 dBm is the safe continuous-duty limit; 20 dBm requires PA_DAC boost.
    pub tx_power_dbm: i8,
}

impl Default for LoRaConfig {
    fn default() -> Self {
        Self {
            frequency_hz: 915_000_000,
            spreading_factor: SpreadingFactor::SF9,
            bandwidth: Bandwidth::BW125,
            coding_rate: CodingRate::CR4_5,
            tx_power_dbm: 17,
        }
    }
}

// ---------------------------------------------------------------------------
// RSSI / SNR report
// ---------------------------------------------------------------------------

/// Signal quality metrics from the last received packet.
#[derive(Debug, Clone, Copy)]
pub struct SignalQuality {
    /// Packet RSSI in dBm. Typical range: -120 to -30 dBm.
    pub rssi_dbm: i16,
    /// Packet SNR in tenths of a dB (divide by 10 for dB).
    /// Negative SNR is normal for LoRa — it can decode below the noise floor.
    pub snr_db_tenths: i16,
}

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

#[derive(Debug)]
pub enum RadioError {
    /// SPI transaction failed.
    Spi,
    /// Version register returned unexpected value — check wiring.
    InvalidVersion(u8),
    /// Postcard serialization failed.
    Serialize,
    /// Postcard deserialization failed.
    Deserialize,
    /// Hardware CRC error detected on received packet.
    CrcError,
    /// RX timeout (only in single-receive mode; not used in continuous mode).
    RxTimeout,
    /// Packet too large for buffer.
    PacketTooLarge,
}

// ---------------------------------------------------------------------------
// Driver
// ---------------------------------------------------------------------------

/// RFM95 LoRa driver.
///
/// # Type Parameters
/// - `SPI`: An async SPI device (includes CS management).
/// - `RST`: Output pin for hardware reset.
/// - `DIO0`: Input pin with async edge-wait capability.
///
/// # DIO0 Wiring
///
/// DIO0 **must** be physically connected to a GPIO pin on the microcontroller.
/// The driver calls `.wait_for_rising_edge()` on this pin to detect when the
/// radio has finished transmitting or has received a packet. Without this
/// connection all TX and RX operations will block indefinitely.
pub struct Rfm95<SPI, RST, DIO0> {
    spi: SPI,
    reset: RST,
    dio0: DIO0,
    config: LoRaConfig,
}

impl<SPI, RST, DIO0> Rfm95<SPI, RST, DIO0>
where
    SPI: SpiDevice,
    RST: embedded_hal::digital::OutputPin,
    DIO0: Wait,
{
    /// Initialise the radio.
    ///
    /// Performs a hardware reset, verifies the chip version, applies the
    /// provided configuration, and leaves the radio in standby mode.
    ///
    /// # Errors
    /// Returns [`RadioError::InvalidVersion`] if the VERSION register does not
    /// match the expected RFM95 value — most likely a wiring problem.
    pub async fn new(
        spi: SPI,
        mut reset: RST,
        dio0: DIO0,
        config: LoRaConfig,
    ) -> Result<Self, RadioError> {
        let mut radio = Self {
            spi,
            reset,
            dio0,
            config,
        };

        // Hardware reset: pull low for 10ms, release, wait 10ms for boot.
        radio.reset.set_low().ok();
        Timer::after(Duration::from_millis(10)).await;
        radio.reset.set_high().ok();
        Timer::after(Duration::from_millis(10)).await;

        // Verify chip identity.
        let version = radio.read_reg(regs::VERSION).await?;
        if version != RFM95_VERSION {
            return Err(RadioError::InvalidVersion(version));
        }

        // Enter sleep mode before switching to LoRa (required by datasheet).
        radio.write_reg(regs::OP_MODE, MODE_SLEEP).await?;
        Timer::after(Duration::from_millis(10)).await;

        // Set LoRa mode (bit 7) while in sleep.
        radio
            .write_reg(regs::OP_MODE, MODE_SLEEP | MODE_LONG_RANGE)
            .await?;
        Timer::after(Duration::from_millis(10)).await;

        // Set FIFO base addresses.
        radio.write_reg(regs::FIFO_TX_BASE_ADDR, 0x00).await?;
        radio.write_reg(regs::FIFO_RX_BASE_ADDR, 0x00).await?;

        // LNA: max gain, boost on.
        radio.write_reg(regs::LNA, 0x23).await?;

        // Apply user configuration.
        radio.apply_config().await?;

        // Standby.
        radio
            .write_reg(regs::OP_MODE, MODE_LONG_RANGE | MODE_STDBY)
            .await?;

        Ok(radio)
    }

    /// Transmit a packet.
    ///
    /// Serializes `packet` with postcard, writes it to the radio FIFO, and
    /// triggers transmission. Awaits DIO0 rising edge for TX-done confirmation.
    ///
    /// # DIO0
    /// DIO0 is mapped to `TxDone` during this call. The pin **must** be
    /// connected or this function will never return.
    pub async fn transmit<T: Serialize>(&mut self, packet: &T) -> Result<(), RadioError> {
        let mut buf = [0u8; PACKET_BUFFER_SIZE];
        let serialized = to_slice(packet, &mut buf).map_err(|_| RadioError::Serialize)?;
        let len = serialized.len();

        if len > MAX_PAYLOAD {
            return Err(RadioError::PacketTooLarge);
        }

        // Standby before touching FIFO.
        self.write_reg(regs::OP_MODE, MODE_LONG_RANGE | MODE_STDBY)
            .await?;

        // Reset FIFO pointer to TX base.
        self.write_reg(regs::FIFO_ADDR_PTR, 0x00).await?;
        self.write_reg(regs::PAYLOAD_LENGTH, len as u8).await?;

        // Burst-write payload into FIFO.
        self.write_fifo(&buf[..len]).await?;

        // Map DIO0 → TxDone (bits [7:6] = 01).
        self.write_reg(regs::DIO_MAPPING1, 0x40).await?;

        // Clear all IRQ flags.
        self.write_reg(regs::IRQ_FLAGS, 0xFF).await?;

        // Begin transmission.
        self.write_reg(regs::OP_MODE, MODE_LONG_RANGE | MODE_TX)
            .await?;

        // Wait for DIO0 rising edge (TxDone).
        // ⚠️  DIO0 must be physically connected — see module docs.
        self.dio0
            .wait_for_rising_edge()
            .await
            .map_err(|_| RadioError::Spi)?;

        // Clear TX done flag.
        self.write_reg(regs::IRQ_FLAGS, IRQ_TX_DONE).await?;

        Ok(())
    }

    /// Enter continuous receive mode and await one packet.
    ///
    /// Puts the radio into `RxContinuous` mode and awaits DIO0 rising edge
    /// (RxDone). Deserializes the received payload into `T`.
    ///
    /// Returns the deserialized packet and signal quality metrics.
    ///
    /// # DIO0
    /// DIO0 is mapped to `RxDone` during this call. The pin **must** be
    /// connected or this function will never return.
    ///
    /// # Notes
    /// The radio remains in continuous RX mode after this call returns.
    /// Call again immediately to keep listening.
    pub async fn receive<T: for<'de> Deserialize<'de>>(
        &mut self,
    ) -> Result<(T, SignalQuality), RadioError> {
        // Map DIO0 → RxDone (bits [7:6] = 00).
        self.write_reg(regs::DIO_MAPPING1, 0x00).await?;

        // Clear all IRQ flags.
        self.write_reg(regs::IRQ_FLAGS, 0xFF).await?;

        // Reset RX FIFO pointer.
        self.write_reg(regs::FIFO_ADDR_PTR, 0x00).await?;

        // Enter continuous RX mode.
        self.write_reg(regs::OP_MODE, MODE_LONG_RANGE | MODE_RX_CONTINUOUS)
            .await?;

        // Wait for DIO0 rising edge (RxDone).
        // ⚠️  DIO0 must be physically connected — see module docs.
        self.dio0
            .wait_for_rising_edge()
            .await
            .map_err(|_| RadioError::Spi)?;

        // Read and validate IRQ flags.
        let irq = self.read_reg(regs::IRQ_FLAGS).await?;

        if irq & IRQ_RX_TIMEOUT != 0 {
            self.write_reg(regs::IRQ_FLAGS, 0xFF).await?;
            return Err(RadioError::RxTimeout);
        }

        if irq & IRQ_PAYLOAD_CRC_ERROR != 0 {
            self.write_reg(regs::IRQ_FLAGS, 0xFF).await?;
            return Err(RadioError::CrcError);
        }

        // Read signal quality before touching FIFO.
        let quality = self.read_signal_quality().await?;

        // Read payload length and FIFO address.
        let len = self.read_reg(regs::RX_NB_BYTES).await? as usize;
        let fifo_addr = self.read_reg(regs::FIFO_RX_CURRENT_ADDR).await?;

        if len > PACKET_BUFFER_SIZE {
            self.write_reg(regs::IRQ_FLAGS, 0xFF).await?;
            return Err(RadioError::PacketTooLarge);
        }

        // Seek FIFO to start of received packet.
        self.write_reg(regs::FIFO_ADDR_PTR, fifo_addr).await?;

        // Burst-read payload from FIFO.
        let mut buf = [0u8; PACKET_BUFFER_SIZE];
        self.read_fifo(&mut buf[..len]).await?;

        // Clear IRQ flags.
        self.write_reg(regs::IRQ_FLAGS, 0xFF).await?;

        // Deserialize.
        let packet: T = from_bytes(&buf[..len]).map_err(|_| RadioError::Deserialize)?;

        Ok((packet, quality))
    }

    /// Read RSSI and SNR from the last received packet.
    pub async fn read_signal_quality(&mut self) -> Result<SignalQuality, RadioError> {
        let raw_rssi = self.read_reg(regs::PKT_RSSI_VALUE).await?;
        let raw_snr = self.read_reg(regs::PKT_SNR_VALUE).await? as i8;

        // RSSI formula for 915MHz (HF port): RSSI = -157 + raw_rssi
        let rssi_dbm = -157i16 + raw_rssi as i16;

        // SNR is in units of 0.25 dB, stored as signed byte.
        // Convert to tenths of dB: snr_tenths = (raw_snr * 10) / 4
        let snr_db_tenths = (raw_snr as i16 * 10) / 4;

        Ok(SignalQuality {
            rssi_dbm,
            snr_db_tenths,
        })
    }

    /// Reconfigure the radio at runtime.
    ///
    /// Returns to standby, applies new config, returns to standby.
    pub async fn reconfigure(&mut self, config: LoRaConfig) -> Result<(), RadioError> {
        self.config = config;
        self.write_reg(regs::OP_MODE, MODE_LONG_RANGE | MODE_STDBY)
            .await?;
        self.apply_config().await
    }

    /// Put the radio into sleep mode to save power.
    pub async fn sleep(&mut self) -> Result<(), RadioError> {
        self.write_reg(regs::OP_MODE, MODE_LONG_RANGE | MODE_SLEEP)
            .await
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    /// Apply the current `self.config` to the radio registers.
    async fn apply_config(&mut self) -> Result<(), RadioError> {
        // --- Frequency ---
        // frf = frequency_hz * 2^19 / FXOSC
        let frf = ((self.config.frequency_hz as u64) << 19) / FXOSC;
        self.write_reg(regs::FR_MSB, (frf >> 16) as u8).await?;
        self.write_reg(regs::FR_MID, (frf >> 8) as u8).await?;
        self.write_reg(regs::FR_LSB, frf as u8).await?;

        // --- Modem config 1: BW + CR + implicit header off ---
        let bw = self.config.bandwidth as u8;
        let cr = self.config.coding_rate as u8;
        self.write_reg(regs::MODEM_CONFIG1, (bw << 4) | (cr << 1) | 0x00)
            .await?;

        // --- Modem config 2: SF + TX continuous off + CRC on ---
        let sf = self.config.spreading_factor as u8;
        self.write_reg(regs::MODEM_CONFIG2, (sf << 4) | 0x04)
            .await?;

        // --- Modem config 3: LNA gain set by register, mobile node ---
        // Set bit 3 (AgcAutoOn) and bit 2 (LowDataRateOptimize for SF11/12)
        let ldro = if sf >= 11 { 0x08 } else { 0x00 };
        self.write_reg(regs::MODEM_CONFIG3, ldro | 0x04).await?;

        // --- TX power ---
        // PA_BOOST path: Pout = 2 + OutputPower (dBm), max 17 dBm normally.
        // For 20 dBm, PA_DAC must be set to 0x87.
        let (pa_config, pa_dac) = match self.config.tx_power_dbm {
            p if p >= 20 => (PA_BOOST | 0x0F, 0x87u8), // 20 dBm: PA_DAC boost
            p if p >= 2 => (PA_BOOST | ((p - 2) as u8 & 0x0F), 0x84u8),
            _ => (PA_BOOST | 0x00, 0x84u8), // clamp to minimum
        };
        self.write_reg(regs::PA_CONFIG, pa_config).await?;
        self.write_reg(regs::PA_DAC, pa_dac).await?;

        // --- Sync word: private network ---
        self.write_reg(regs::SYNC_WORD, LORA_SYNC_WORD).await?;

        // --- Max payload length ---
        self.write_reg(regs::MAX_PAYLOAD_LENGTH, MAX_PAYLOAD as u8)
            .await?;

        Ok(())
    }

    /// Write a single register.
    async fn write_reg(&mut self, reg: u8, value: u8) -> Result<(), RadioError> {
        // SPI write: address with bit 7 set, then value.
        let buf = [reg | 0x80, value];
        self.spi.write(&buf).await.map_err(|_| RadioError::Spi)
    }

    /// Read a single register.
    async fn read_reg(&mut self, reg: u8) -> Result<u8, RadioError> {
        let mut buf = [reg & 0x7F, 0x00];
        self.spi
            .transfer_in_place(&mut buf)
            .await
            .map_err(|_| RadioError::Spi)?;
        Ok(buf[1])
    }

    /// Burst-write bytes into the FIFO register.
    async fn write_fifo(&mut self, data: &[u8]) -> Result<(), RadioError> {
        // Manual transfer: send FIFO write address, then payload.
        // SpiDevice::write sends address + data in one CS assertion.
        let mut buf = [0u8; PACKET_BUFFER_SIZE + 1];
        buf[0] = regs::FIFO | 0x80;
        buf[1..=data.len()].copy_from_slice(data);
        self.spi
            .write(&buf[..=data.len()])
            .await
            .map_err(|_| RadioError::Spi)
    }

    /// Burst-read bytes from the FIFO register.
    async fn read_fifo(&mut self, data: &mut [u8]) -> Result<(), RadioError> {
        let mut buf = [0u8; PACKET_BUFFER_SIZE + 1];
        buf[0] = regs::FIFO & 0x7F;
        let len = data.len();
        self.spi
            .transfer_in_place(&mut buf[..=len])
            .await
            .map_err(|_| RadioError::Spi)?;
        data.copy_from_slice(&buf[1..=len]);
        Ok(())
    }
}
