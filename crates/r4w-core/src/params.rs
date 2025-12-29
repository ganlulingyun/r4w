//! LoRa Parameters and Configuration
//!
//! This module defines the configurable parameters for LoRa modulation,
//! including spreading factor, bandwidth, and coding rate.
//!
//! ## Understanding LoRa Parameters
//!
//! ### Spreading Factor (SF)
//!
//! The spreading factor determines how many chips (sub-symbols) are used to
//! encode each bit. Higher SF values provide:
//! - Better sensitivity (can decode weaker signals)
//! - Longer range
//! - But slower data rate
//!
//! | SF | Chips/Symbol | Bits/Symbol | Sensitivity Gain |
//! |----|--------------|-------------|------------------|
//! | 7  | 128          | 7           | Baseline         |
//! | 8  | 256          | 8           | +2.5 dB          |
//! | 9  | 512          | 9           | +5.0 dB          |
//! | 10 | 1024         | 10          | +7.5 dB          |
//! | 11 | 2048         | 11          | +10.0 dB         |
//! | 12 | 4096         | 12          | +12.5 dB         |
//!
//! ### Bandwidth (BW)
//!
//! The channel bandwidth affects data rate and noise immunity:
//! - 125 kHz: Standard, best sensitivity
//! - 250 kHz: Faster, moderate sensitivity
//! - 500 kHz: Fastest, lowest sensitivity
//!
//! ### Coding Rate (CR)
//!
//! Forward Error Correction adds redundancy:
//! - CR 4/5: Minimal redundancy, highest throughput
//! - CR 4/6: Light FEC
//! - CR 4/7: Medium FEC
//! - CR 4/8: Maximum FEC, most robust

use serde::{Deserialize, Serialize};
use std::fmt;

use crate::types::DspError;

/// Spreading Factor for LoRa modulation
///
/// The spreading factor determines the number of chips per symbol (2^SF)
/// and the number of bits encoded per symbol (SF bits).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum SpreadingFactor {
    SF5 = 5,
    SF6 = 6,
    SF7 = 7,
    SF8 = 8,
    SF9 = 9,
    SF10 = 10,
    SF11 = 11,
    SF12 = 12,
}

impl SpreadingFactor {
    /// Create a spreading factor from a raw value
    pub fn from_u8(value: u8) -> Result<Self, DspError> {
        match value {
            5 => Ok(Self::SF5),
            6 => Ok(Self::SF6),
            7 => Ok(Self::SF7),
            8 => Ok(Self::SF8),
            9 => Ok(Self::SF9),
            10 => Ok(Self::SF10),
            11 => Ok(Self::SF11),
            12 => Ok(Self::SF12),
            _ => Err(DspError::InvalidSpreadingFactor(value)),
        }
    }

    /// Get the raw value
    pub fn value(&self) -> u8 {
        *self as u8
    }

    /// Number of chips (samples at chip rate) per symbol
    ///
    /// This is 2^SF. For SF7, there are 128 chips per symbol.
    pub fn chips_per_symbol(&self) -> usize {
        1 << self.value()
    }

    /// Number of bits encoded per symbol
    ///
    /// This equals the SF value itself.
    pub fn bits_per_symbol(&self) -> u8 {
        self.value()
    }

    /// Typical SNR threshold for successful demodulation (in dB)
    ///
    /// These are approximate values from the LoRa specification.
    pub fn snr_threshold(&self) -> f64 {
        match self {
            Self::SF5 => -5.0,
            Self::SF6 => -5.0,
            Self::SF7 => -7.5,
            Self::SF8 => -10.0,
            Self::SF9 => -12.5,
            Self::SF10 => -15.0,
            Self::SF11 => -17.5,
            Self::SF12 => -20.0,
        }
    }

    /// Number of preamble symbols before payload
    ///
    /// Standard LoRa uses 8 upchirps + 2 sync words + 2.25 downchirps
    pub fn preamble_symbols(&self) -> usize {
        8 // Standard preamble length
    }
}

impl fmt::Display for SpreadingFactor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "SF{}", self.value())
    }
}

impl Default for SpreadingFactor {
    fn default() -> Self {
        Self::SF7
    }
}

/// Coding Rate for Forward Error Correction
///
/// LoRa uses Hamming codes for FEC. The coding rate 4/(4+CR) determines
/// the ratio of data bits to total bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum CodingRate {
    /// 4/5 - 1 redundant bit per 4 data bits
    CR4_5 = 1,
    /// 4/6 - 2 redundant bits per 4 data bits
    CR4_6 = 2,
    /// 4/7 - 3 redundant bits per 4 data bits
    CR4_7 = 3,
    /// 4/8 - 4 redundant bits per 4 data bits
    CR4_8 = 4,
}

impl CodingRate {
    pub fn from_u8(value: u8) -> Result<Self, DspError> {
        match value {
            1 => Ok(Self::CR4_5),
            2 => Ok(Self::CR4_6),
            3 => Ok(Self::CR4_7),
            4 => Ok(Self::CR4_8),
            _ => Err(DspError::InvalidCodingRate(value)),
        }
    }

    /// Get the raw value (number of redundant bits)
    pub fn value(&self) -> u8 {
        *self as u8
    }

    /// Get the coding rate as a fraction
    pub fn rate(&self) -> f64 {
        4.0 / (4.0 + self.value() as f64)
    }

    /// Number of output bits per 4 input bits
    pub fn output_bits(&self) -> u8 {
        4 + self.value()
    }
}

impl fmt::Display for CodingRate {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "4/{}", 4 + self.value())
    }
}

impl Default for CodingRate {
    fn default() -> Self {
        Self::CR4_5
    }
}

/// Bandwidth configuration for LoRa
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Bandwidth {
    /// 125 kHz - Standard bandwidth, best sensitivity
    Bw125kHz = 125_000,
    /// 250 kHz - Doubled bandwidth
    Bw250kHz = 250_000,
    /// 500 kHz - Maximum bandwidth
    Bw500kHz = 500_000,
}

impl Bandwidth {
    pub fn from_hz(hz: u32) -> Result<Self, DspError> {
        match hz {
            125_000 => Ok(Self::Bw125kHz),
            250_000 => Ok(Self::Bw250kHz),
            500_000 => Ok(Self::Bw500kHz),
            _ => Err(DspError::InvalidBandwidth(hz as f64)),
        }
    }

    /// Get bandwidth in Hz
    pub fn hz(&self) -> f64 {
        *self as u32 as f64
    }

    /// Get chip duration in seconds
    pub fn chip_duration(&self) -> f64 {
        1.0 / self.hz()
    }
}

impl Default for Bandwidth {
    fn default() -> Self {
        Self::Bw125kHz
    }
}

/// Regional frequency band configuration
#[derive(Debug, Clone, Copy, PartialEq, Serialize, Deserialize)]
pub enum Region {
    /// EU868 (863-870 MHz)
    EU868,
    /// US915 (902-928 MHz)
    US915,
    /// EU433 (433 MHz)
    EU433,
    /// AS923 (923 MHz)
    AS923,
}

impl Region {
    /// Get the center frequency for this region in Hz
    pub fn center_frequency(&self) -> f64 {
        match self {
            Self::EU868 => 868.0e6,
            Self::US915 => 915.0e6,
            Self::EU433 => 433.0e6,
            Self::AS923 => 923.0e6,
        }
    }
}

impl Default for Region {
    fn default() -> Self {
        Self::US915
    }
}

/// Complete LoRa parameter configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LoRaParams {
    /// Spreading Factor (5-12)
    pub sf: SpreadingFactor,
    /// Bandwidth in Hz
    pub bw: Bandwidth,
    /// Coding Rate
    pub cr: CodingRate,
    /// Center frequency in Hz
    pub frequency: f64,
    /// Sample rate (oversampling factor * bandwidth)
    pub sample_rate: f64,
    /// Oversampling factor (typically 1, 2, or 4)
    pub oversample: usize,
    /// Enable low data rate optimization (for SF11/12)
    pub low_data_rate_optimize: bool,
    /// Enable implicit header mode
    pub implicit_header: bool,
    /// Enable CRC
    pub crc_enabled: bool,
    /// Preamble length in symbols
    pub preamble_length: usize,
    /// Sync word (default 0x12 for private, 0x34 for LoRaWAN)
    pub sync_word: u8,
}

impl Default for LoRaParams {
    fn default() -> Self {
        let bw = Bandwidth::default();
        Self {
            sf: SpreadingFactor::default(),
            bw,
            cr: CodingRate::default(),
            frequency: Region::default().center_frequency(),
            sample_rate: bw.hz(), // 1x oversampling by default
            oversample: 1,
            low_data_rate_optimize: false,
            implicit_header: false,
            crc_enabled: true,
            preamble_length: 8,
            sync_word: 0x12,
        }
    }
}

impl LoRaParams {
    /// Create a new builder for LoRa parameters
    pub fn builder() -> LoRaParamsBuilder {
        LoRaParamsBuilder::default()
    }

    /// Number of chips per symbol
    pub fn chips_per_symbol(&self) -> usize {
        self.sf.chips_per_symbol()
    }

    /// Number of samples per symbol (chips * oversample factor)
    pub fn samples_per_symbol(&self) -> usize {
        self.chips_per_symbol() * self.oversample
    }

    /// Symbol duration in seconds
    pub fn symbol_duration(&self) -> f64 {
        self.chips_per_symbol() as f64 / self.bw.hz()
    }

    /// Chip duration in seconds
    pub fn chip_duration(&self) -> f64 {
        1.0 / self.bw.hz()
    }

    /// Sample duration in seconds
    pub fn sample_duration(&self) -> f64 {
        1.0 / self.sample_rate
    }

    /// Calculate the bit rate in bits per second
    pub fn bit_rate(&self) -> f64 {
        let sf = self.sf.value() as f64;
        let bw = self.bw.hz();
        let cr = self.cr.rate();

        // Account for low data rate optimization which reduces effective bits
        let effective_sf = if self.low_data_rate_optimize && self.sf.value() >= 11 {
            sf - 2.0
        } else {
            sf
        };

        effective_sf * bw * cr / (2.0_f64.powf(sf))
    }

    /// Calculate time on air for a given payload size (in seconds)
    pub fn time_on_air(&self, payload_bytes: usize) -> f64 {
        let sf = self.sf.value() as f64;
        let _bw = self.bw.hz();
        let cr = 4.0 + self.cr.value() as f64;

        // Preamble time
        let t_preamble = (self.preamble_length as f64 + 4.25) * self.symbol_duration();

        // Payload symbols (simplified calculation)
        let de = if self.low_data_rate_optimize { 1.0 } else { 0.0 };
        let h = if self.implicit_header { 0.0 } else { 1.0 };

        let payload_bits = 8.0 * payload_bytes as f64;
        let num = 8.0 * payload_bits - 4.0 * sf + 28.0 + 16.0 - 20.0 * h;
        let denom = 4.0 * (sf - 2.0 * de);

        let n_payload = 8.0 + (num / denom).ceil().max(0.0) * cr;
        let t_payload = n_payload * self.symbol_duration();

        t_preamble + t_payload
    }

    /// Receiver sensitivity in dBm
    pub fn sensitivity(&self) -> f64 {
        let bw_db = 10.0 * (self.bw.hz()).log10();
        let nf = 6.0; // Typical noise figure
        -174.0 + bw_db + nf + self.sf.snr_threshold()
    }
}

/// Builder for LoRaParams
#[derive(Default)]
pub struct LoRaParamsBuilder {
    params: LoRaParams,
}

impl LoRaParamsBuilder {
    pub fn spreading_factor(mut self, sf: u8) -> Self {
        self.params.sf = SpreadingFactor::from_u8(sf).unwrap_or_default();
        self
    }

    pub fn bandwidth(mut self, bw_hz: u32) -> Self {
        self.params.bw = Bandwidth::from_hz(bw_hz).unwrap_or_default();
        self.params.sample_rate = self.params.bw.hz() * self.params.oversample as f64;
        self
    }

    pub fn coding_rate(mut self, cr: u8) -> Self {
        self.params.cr = CodingRate::from_u8(cr).unwrap_or_default();
        self
    }

    pub fn frequency(mut self, freq_hz: f64) -> Self {
        self.params.frequency = freq_hz;
        self
    }

    pub fn oversample(mut self, factor: usize) -> Self {
        self.params.oversample = factor.max(1);
        self.params.sample_rate = self.params.bw.hz() * self.params.oversample as f64;
        self
    }

    pub fn preamble_length(mut self, len: usize) -> Self {
        self.params.preamble_length = len;
        self
    }

    pub fn crc_enabled(mut self, enabled: bool) -> Self {
        self.params.crc_enabled = enabled;
        self
    }

    pub fn sync_word(mut self, word: u8) -> Self {
        self.params.sync_word = word;
        self
    }

    pub fn build(self) -> LoRaParams {
        self.params
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_chips_per_symbol() {
        assert_eq!(SpreadingFactor::SF7.chips_per_symbol(), 128);
        assert_eq!(SpreadingFactor::SF8.chips_per_symbol(), 256);
        assert_eq!(SpreadingFactor::SF12.chips_per_symbol(), 4096);
    }

    #[test]
    fn test_symbol_duration() {
        let params = LoRaParams::builder()
            .spreading_factor(7)
            .bandwidth(125_000)
            .build();

        // SF7 at 125kHz: 128 chips / 125000 Hz = 1.024 ms
        assert!((params.symbol_duration() - 0.001024).abs() < 1e-9);
    }

    #[test]
    fn test_bit_rate() {
        let params = LoRaParams::builder()
            .spreading_factor(7)
            .bandwidth(125_000)
            .coding_rate(1)
            .build();

        // Approximate bit rate for SF7, 125kHz, CR 4/5
        let rate = params.bit_rate();
        assert!(rate > 5000.0 && rate < 6000.0);
    }
}
