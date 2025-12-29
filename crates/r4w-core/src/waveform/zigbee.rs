//! IEEE 802.15.4 / Zigbee PHY Layer
//!
//! The 802.15.4 standard defines the PHY layer for low-rate wireless personal
//! area networks (LR-WPANs), commonly known as Zigbee. The 2.4 GHz band uses
//! O-QPSK modulation with DSSS spreading.
//!
//! ## Key Specifications (2.4 GHz Band)
//!
//! - **Modulation**: O-QPSK (Offset Quadrature Phase Shift Keying)
//! - **Spreading**: DSSS with 32-chip sequences
//! - **Chip Rate**: 2 Mchip/s
//! - **Symbol Rate**: 62.5 ksym/s (4 bits per symbol)
//! - **Data Rate**: 250 kbps
//! - **Processing Gain**: 10*log10(32) ≈ 15 dB
//!
//! ## Symbol-to-Chip Mapping
//!
//! Each 4-bit symbol (0-15) maps to a unique 32-chip PN sequence.
//! The sequences are designed for good autocorrelation and cross-correlation.
//!
//! ## O-QPSK vs QPSK
//!
//! O-QPSK offsets the Q channel by half a symbol period, which:
//! - Limits phase transitions to ±90° (never 180°)
//! - Reduces envelope variations
//! - Improves performance with non-linear amplifiers
//!
//! ## Half-Sine Pulse Shaping
//!
//! 802.15.4 uses half-sine pulse shaping for spectral efficiency,
//! creating MSK-like (Minimum Shift Keying) characteristics.

use super::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};
use crate::types::IQSample;
use std::f64::consts::PI;

/// Unpack bytes to individual bits (MSB first)
fn bytes_to_bits(data: &[u8]) -> Vec<u8> {
    let mut bits = Vec::with_capacity(data.len() * 8);
    for byte in data {
        for i in (0..8).rev() {
            bits.push((byte >> i) & 1);
        }
    }
    bits
}

/// Pack individual bits to bytes (MSB first)
fn bits_to_bytes(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8)
        .map(|chunk| {
            chunk.iter()
                .enumerate()
                .fold(0u8, |acc, (i, &bit)| {
                    acc | ((bit & 1) << (7 - i))
                })
        })
        .collect()
}

/// Check if data is packed bytes (contains values > 1)
fn is_packed_bytes(data: &[u8]) -> bool {
    data.iter().any(|&b| b > 1)
}

/// IEEE 802.15.4 chip sequences for 2.4 GHz band
/// Each 4-bit symbol (0-15) maps to a 32-chip sequence
/// Chips are represented as +1/-1
const CHIP_SEQUENCES: [[i8; 32]; 16] = [
    // Symbol 0
    [1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0],
    // Symbol 1
    [1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0],
    // Symbol 2
    [0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0],
    // Symbol 3
    [0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1],
    // Symbol 4
    [0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1],
    // Symbol 5
    [0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1, 1, 1, 0, 0],
    // Symbol 6
    [1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1, 1, 0, 0, 1],
    // Symbol 7
    [1, 0, 0, 1, 1, 1, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 1, 0, 1, 1, 1, 0, 1, 1, 0, 1],
    // Symbol 8
    [1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1],
    // Symbol 9
    [1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1],
    // Symbol 10
    [0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1],
    // Symbol 11
    [0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0],
    // Symbol 12
    [0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0],
    // Symbol 13
    [0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 1],
    // Symbol 14
    [1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0, 1, 1, 0, 0],
    // Symbol 15
    [1, 1, 0, 0, 1, 0, 0, 1, 0, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1, 1, 0, 0, 0],
];

/// Convert chip sequence from 0/1 to +1/-1
fn chip_to_bipolar(chip: i8) -> f64 {
    if chip == 0 { -1.0 } else { 1.0 }
}

/// IEEE 802.15.4 / Zigbee PHY
#[derive(Debug, Clone)]
pub struct Zigbee {
    /// Common waveform parameters
    common: CommonParams,
    /// Samples per chip
    samples_per_chip: usize,
    /// Use half-sine pulse shaping
    half_sine_shaping: bool,
}

impl Zigbee {
    /// Create a new Zigbee modulator
    ///
    /// Standard 802.15.4 at 2.4 GHz:
    /// - Chip rate: 2 Mchip/s
    /// - Symbol rate: 62.5 ksym/s
    /// - Data rate: 250 kbps
    pub fn new(common: CommonParams, samples_per_chip: usize, half_sine_shaping: bool) -> Self {
        Self {
            common,
            samples_per_chip,
            half_sine_shaping,
        }
    }

    /// Create with standard 802.15.4 parameters
    pub fn standard(sample_rate: f64) -> Self {
        // Standard chip rate is 2 MHz
        // samples_per_chip = sample_rate / chip_rate
        // Minimum 4 samples per chip for proper half-sine pulse shaping
        let samples_per_chip = (sample_rate / 2_000_000.0).max(4.0) as usize;

        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };

        Self::new(common, samples_per_chip, true)
    }

    /// Create simplified version (no half-sine shaping)
    pub fn simple(sample_rate: f64) -> Self {
        // Minimum 4 samples per chip for consistent behavior
        let samples_per_chip = (sample_rate / 2_000_000.0).max(4.0) as usize;

        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };

        Self::new(common, samples_per_chip.max(4), false)
    }

    /// Get chip rate in chips/second
    pub fn chip_rate(&self) -> f64 {
        self.common.sample_rate / self.samples_per_chip as f64
    }

    /// Get symbol rate in symbols/second
    pub fn symbol_rate(&self) -> f64 {
        self.chip_rate() / 32.0 // 32 chips per symbol
    }

    /// Get data rate in bits/second
    pub fn data_rate(&self) -> f64 {
        self.symbol_rate() * 4.0 // 4 bits per symbol
    }

    /// Get processing gain in dB
    pub fn processing_gain_db(&self) -> f64 {
        10.0 * 32.0_f64.log10() // ~15 dB
    }

    /// Convert 4 bits to symbol index (0-15)
    fn bits_to_symbol(bits: &[u8]) -> usize {
        let mut symbol = 0usize;
        for (i, &bit) in bits.iter().take(4).enumerate() {
            symbol |= ((bit & 1) as usize) << i;
        }
        symbol
    }

    /// Convert symbol index to 4 bits
    fn symbol_to_bits(symbol: usize) -> [u8; 4] {
        [
            (symbol & 1) as u8,
            ((symbol >> 1) & 1) as u8,
            ((symbol >> 2) & 1) as u8,
            ((symbol >> 3) & 1) as u8,
        ]
    }

    /// Generate half-sine pulse shape
    #[allow(dead_code)]
    fn half_sine_pulse(&self, chip_idx: usize, num_chips: usize) -> f64 {
        let t = (chip_idx as f64 + 0.5) / num_chips as f64;
        (PI * t).sin()
    }

    /// Modulate one symbol using O-QPSK with DSSS
    fn modulate_symbol(&self, symbol: usize) -> Vec<IQSample> {
        let chips = &CHIP_SEQUENCES[symbol & 0x0F];
        let samples_per_symbol = 32 * self.samples_per_chip;
        let mut samples = Vec::with_capacity(samples_per_symbol);

        // O-QPSK: I and Q channels, Q is offset by 16 chips (half symbol)
        // First half: I chips 0-15, Q chips from previous symbol (we'll use 0)
        // Second half: I chips 16-31, Q chips 0-15
        // For simplicity, we'll implement standard O-QPSK within the symbol

        for chip_idx in 0..32 {
            let i_chip = chip_to_bipolar(chips[chip_idx]);

            // Q channel is offset by half the chip sequence
            let q_chip_idx = (chip_idx + 16) % 32;
            let q_chip = chip_to_bipolar(chips[q_chip_idx]);

            for sample_idx in 0..self.samples_per_chip {
                let (i_val, q_val) = if self.half_sine_shaping {
                    // Half-sine pulse shaping
                    let t = sample_idx as f64 / self.samples_per_chip as f64;
                    let pulse = (PI * t).sin();
                    (i_chip * pulse * self.common.amplitude,
                     q_chip * pulse * self.common.amplitude)
                } else {
                    // Rectangular pulses
                    (i_chip * self.common.amplitude,
                     q_chip * self.common.amplitude)
                };

                samples.push(IQSample::new(i_val, q_val));
            }
        }

        samples
    }

    /// Despread and demodulate received samples
    fn demodulate_symbol(&self, samples: &[IQSample]) -> usize {
        let samples_per_symbol = 32 * self.samples_per_chip;

        if samples.len() < samples_per_symbol {
            return 0;
        }

        // Correlate with all 16 chip sequences
        let mut best_symbol = 0;
        let mut best_correlation = f64::NEG_INFINITY;

        for symbol in 0..16 {
            let chips = &CHIP_SEQUENCES[symbol];
            let mut correlation = 0.0;

            for chip_idx in 0..32 {
                let i_chip = chip_to_bipolar(chips[chip_idx]);
                let q_chip_idx = (chip_idx + 16) % 32;
                let q_chip = chip_to_bipolar(chips[q_chip_idx]);

                // Integrate over chip duration
                for sample_idx in 0..self.samples_per_chip {
                    let idx = chip_idx * self.samples_per_chip + sample_idx;
                    if idx < samples.len() {
                        correlation += samples[idx].re * i_chip;
                        correlation += samples[idx].im * q_chip;
                    }
                }
            }

            if correlation > best_correlation {
                best_correlation = correlation;
                best_symbol = symbol;
            }
        }

        best_symbol
    }
}

impl Waveform for Zigbee {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "802.15.4",
            full_name: "IEEE 802.15.4 / Zigbee PHY",
            description: "O-QPSK with DSSS spreading - IoT and home automation standard",
            complexity: 3,
            bits_per_symbol: 4,
            carries_data: true,
            characteristics: &[
                "O-QPSK modulation (offset QPSK)",
                "32-chip DSSS spreading (~15 dB gain)",
                "2 Mchip/s chip rate",
                "250 kbps data rate",
                "Half-sine pulse shaping",
                "Low power, mesh networking",
            ],
            history: "IEEE 802.15.4 was published in 2003 for low-rate WPANs. The Zigbee \
                Alliance built the networking layers on top of 802.15.4. Originally designed \
                for home automation and industrial control. The 2.4 GHz band uses O-QPSK \
                for worldwide compatibility.",
            modern_usage: "Zigbee is widely used in smart home devices (lights, sensors, \
                locks), industrial automation, and smart energy. Thread (Google/Apple) \
                also uses 802.15.4 PHY. Competing with Bluetooth LE and WiFi for IoT. \
                Zigbee 3.0 unified previous versions. Matter protocol bridges ecosystems.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // Convert packed bytes to individual bits if needed
        let bits = if is_packed_bytes(data) {
            bytes_to_bits(data)
        } else {
            data.to_vec()
        };

        let mut samples = Vec::new();

        // Process 4 bits at a time (one symbol)
        for bit_chunk in bits.chunks(4) {
            let symbol = Self::bits_to_symbol(bit_chunk);
            let symbol_samples = self.modulate_symbol(symbol);
            samples.extend(symbol_samples);
        }

        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let mut result = DemodResult::default();
        let samples_per_symbol = 32 * self.samples_per_chip;

        // Collect individual bits first
        let mut individual_bits = Vec::new();

        for symbol_samples in samples.chunks(samples_per_symbol) {
            if symbol_samples.len() < samples_per_symbol {
                break;
            }

            let symbol = self.demodulate_symbol(symbol_samples);
            let bits = Self::symbol_to_bits(symbol);
            individual_bits.extend_from_slice(&bits);
            result.symbols.push(symbol as u16);
        }

        // Pack individual bits into bytes for consistent output
        result.bits = bits_to_bytes(&individual_bits);

        result.metadata.insert("processing_gain_db".to_string(), self.processing_gain_db());
        result.metadata.insert("chip_rate".to_string(), self.chip_rate());
        result.metadata.insert("data_rate".to_string(), self.data_rate());

        result
    }

    fn samples_per_symbol(&self) -> usize {
        32 * self.samples_per_chip
    }

    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);

        // O-QPSK constellation (rotated QPSK points)
        let s = 1.0 / 2.0_f64.sqrt();
        let constellation = vec![
            IQSample::new(s, s),
            IQSample::new(-s, s),
            IQSample::new(-s, -s),
            IQSample::new(s, -s),
        ];

        VisualizationData {
            samples,
            constellation,
            constellation_labels: vec!["00".into(), "01".into(), "11".into(), "10".into()],
            spectrum: Vec::new(),
            description: format!(
                "802.15.4: 32-chip DSSS, {:.1} dB gain, {:.0} kbps",
                self.processing_gain_db(),
                self.data_rate() / 1000.0
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zigbee_basic() {
        let zigbee = Zigbee::simple(8_000_000.0);

        assert_eq!(zigbee.samples_per_chip, 4);
        assert!((zigbee.processing_gain_db() - 15.05).abs() < 0.1);
    }

    #[test]
    fn test_zigbee_rates() {
        let zigbee = Zigbee::standard(8_000_000.0);

        // Chip rate should be ~2 MHz
        let chip_rate = zigbee.chip_rate();
        assert!((chip_rate - 2_000_000.0).abs() < 100_000.0);

        // Data rate should be ~250 kbps
        let data_rate = zigbee.data_rate();
        assert!((data_rate - 250_000.0).abs() < 10_000.0);
    }

    #[test]
    fn test_symbol_conversion() {
        for symbol in 0..16 {
            let bits = Zigbee::symbol_to_bits(symbol);
            let recovered = Zigbee::bits_to_symbol(&bits);
            assert_eq!(symbol, recovered);
        }
    }

    #[test]
    fn test_zigbee_roundtrip() {
        let zigbee = Zigbee::simple(8_000_000.0);

        // Test data: 8 bits = 2 symbols
        let data: Vec<u8> = vec![1, 0, 1, 0, 0, 1, 1, 0];

        let modulated = zigbee.modulate(&data);
        let result = zigbee.demodulate(&modulated);

        // Demodulate returns packed bytes, so compare against packed input
        let expected = bits_to_bytes(&data);
        assert_eq!(result.bits.len(), expected.len());
        assert_eq!(result.bits, expected);
    }

    #[test]
    fn test_chip_sequences_unique() {
        // All chip sequences should be different
        for i in 0..16 {
            for j in (i + 1)..16 {
                assert_ne!(CHIP_SEQUENCES[i], CHIP_SEQUENCES[j]);
            }
        }
    }

    #[test]
    fn test_modulation_output() {
        let zigbee = Zigbee::simple(8_000_000.0);

        let data: Vec<u8> = vec![0, 0, 0, 0]; // Symbol 0
        let samples = zigbee.modulate(&data);

        // Should have 32 chips * samples_per_chip samples
        assert_eq!(samples.len(), 32 * zigbee.samples_per_chip);

        // All samples should have reasonable amplitude
        assert!(samples.iter().all(|s|
            (s.re * s.re + s.im * s.im).sqrt() <= 2.0
        ));
    }
}
