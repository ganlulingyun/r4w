//! ASK (Amplitude Shift Keying) - Digital Amplitude Modulation
//!
//! ASK encodes digital data by mapping bits/symbols to discrete amplitude levels.
//! This is the digital counterpart to analog AM (Amplitude Modulation).
//!
//! ## Mathematical Definition
//!
//! ```text
//! s(t) = A[n] · cos(2π·fc·t)
//! ```
//!
//! Where:
//! - A[n] = amplitude level for symbol n
//! - fc = carrier frequency
//!
//! ## Variants
//!
//! - Binary ASK (2-ASK/OOK): Two amplitude levels (on/off)
//! - 4-ASK (PAM-4): Four amplitude levels, 2 bits per symbol
//! - M-ASK: M amplitude levels, log2(M) bits per symbol
//!
//! ## Note
//!
//! For analog AM (voice/audio modulation), see the `am` module.
//! ASK is specifically for digital data transmission.

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

/// ASK (Amplitude Shift Keying) modulator/demodulator
#[derive(Debug, Clone)]
pub struct ASK {
    /// Common waveform parameters
    common: CommonParams,
    /// Symbol rate in symbols per second
    symbol_rate: f64,
    /// Carrier frequency in Hz
    carrier_freq: f64,
    /// Modulation index (0.0 to 1.0 typical, >1 for over-modulation)
    modulation_index: f64,
    /// Number of amplitude levels (2 for binary, 4 for 4-ASK/PAM-4)
    num_levels: usize,
    /// Whether to suppress carrier (DSB-SC mode)
    suppress_carrier: bool,
}

impl ASK {
    /// Create a new ASK modulator
    pub fn new(
        common: CommonParams,
        symbol_rate: f64,
        carrier_freq: f64,
        modulation_index: f64,
    ) -> Self {
        Self {
            common,
            symbol_rate,
            carrier_freq,
            modulation_index,
            num_levels: 2,
            suppress_carrier: false,
        }
    }

    /// Create Binary ASK (2 amplitude levels)
    pub fn new_binary(common: CommonParams, symbol_rate: f64, carrier_freq: f64) -> Self {
        Self::new(common, symbol_rate, carrier_freq, 1.0)
    }

    /// Create 4-ASK (PAM-4, 4 amplitude levels, 2 bits per symbol)
    pub fn new_4ask(common: CommonParams, symbol_rate: f64, carrier_freq: f64) -> Self {
        let mut am = Self::new(common, symbol_rate, carrier_freq, 1.0);
        am.num_levels = 4;
        am
    }

    /// Set modulation index
    pub fn with_modulation_index(mut self, m: f64) -> Self {
        self.modulation_index = m;
        self
    }

    /// Enable carrier suppression (DSB-SC mode)
    pub fn with_suppressed_carrier(mut self, suppress: bool) -> Self {
        self.suppress_carrier = suppress;
        self
    }

    /// Get samples per symbol (minimum 1 to prevent division by zero)
    fn sps(&self) -> usize {
        if self.symbol_rate <= 0.0 {
            return 1;
        }
        ((self.common.sample_rate / self.symbol_rate) as usize).max(1)
    }

    /// Map symbol to amplitude level
    fn symbol_to_amplitude(&self, symbol: u8) -> f64 {
        if self.num_levels == 2 {
            // Binary: 0 -> low amplitude, 1 -> high amplitude
            if symbol == 0 {
                1.0 - self.modulation_index
            } else {
                1.0 + self.modulation_index
            }
        } else {
            // Multi-level: map symbol to amplitude
            // For 4-ASK: symbols 0,1,2,3 map to amplitudes
            let normalized = symbol as f64 / (self.num_levels - 1) as f64; // 0 to 1
            let modulated = 2.0 * normalized - 1.0; // -1 to 1
            1.0 + self.modulation_index * modulated
        }
    }

    /// Generate samples for one symbol
    fn generate_symbol(&self, symbol: u8, start_phase: f64) -> (Vec<IQSample>, f64) {
        let sps = self.sps();
        let omega = 2.0 * PI * self.carrier_freq / self.common.sample_rate;
        let base_amp = self.common.amplitude;

        let envelope = if self.suppress_carrier {
            // DSB-SC: amplitude varies from -m to +m
            let normalized = if self.num_levels == 2 {
                if symbol == 0 { -1.0 } else { 1.0 }
            } else {
                let norm = symbol as f64 / (self.num_levels - 1) as f64;
                2.0 * norm - 1.0
            };
            normalized * self.modulation_index
        } else {
            // Standard AM: DC offset + modulation
            self.symbol_to_amplitude(symbol)
        };

        let samples: Vec<IQSample> = (0..sps)
            .map(|n| {
                let phase = start_phase + omega * n as f64;
                let amp = base_amp * envelope;
                IQSample::new(amp * phase.cos(), amp * phase.sin())
            })
            .collect();

        let end_phase = start_phase + omega * sps as f64;
        (samples, end_phase)
    }

    /// Get modulation index
    pub fn modulation_index(&self) -> f64 {
        self.modulation_index
    }

    /// Get number of amplitude levels
    pub fn num_levels(&self) -> usize {
        self.num_levels
    }
}

impl Waveform for ASK {
    fn info(&self) -> WaveformInfo {
        let (name, full_name, bits) = match (self.num_levels, self.suppress_carrier) {
            (2, false) => ("ASK", "Amplitude Shift Keying", 1),
            (2, true) => ("ASK-SC", "ASK Suppressed Carrier", 1),
            (4, false) => ("4-ASK", "4-Level Amplitude Shift Keying", 2),
            (4, true) => ("4-ASK-SC", "4-Level ASK Suppressed Carrier", 2),
            _ => ("M-ASK", "Multi-level Amplitude Shift Keying",
                  (self.num_levels as f64).log2() as u8),
        };

        WaveformInfo {
            name,
            full_name,
            description: "Encodes digital data by mapping symbols to amplitude levels",
            complexity: 2,
            bits_per_symbol: bits,
            carries_data: true,
            characteristics: &[
                "Discrete amplitude levels for symbols",
                "Simple envelope detection",
                "Susceptible to noise/fading",
                "Used in RFID, NFC, barcode readers",
                "PAM-4 used in high-speed Ethernet",
            ],
            history: "ASK evolved from analog AM for digital data transmission. \
                On-Off Keying (OOK), the simplest form, has been used since early \
                telegraphy. Multi-level ASK (PAM) became important for increasing \
                data rates in bandwidth-limited channels.",
            modern_usage: "RFID systems (passive tags), NFC, optical communications, \
                barcode scanners, and PAM-4 in 100G/400G Ethernet. Often combined with \
                other modulations (QAM = ASK + PSK). Simple but noise-sensitive, so \
                typically used in controlled environments.",
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

        let bits_per_symbol = (self.num_levels as f64).log2() as usize;
        let mut samples = Vec::new();
        let mut phase = 0.0;

        if bits_per_symbol == 1 {
            // Binary ASK
            for &bit in &bits {
                let symbol = bit & 1;
                let (sym_samples, new_phase) = self.generate_symbol(symbol, phase);
                samples.extend(sym_samples);
                phase = new_phase;
            }
        } else {
            // Multi-level AM
            for chunk in bits.chunks(bits_per_symbol) {
                let mut symbol = 0u8;
                for (i, &bit) in chunk.iter().enumerate() {
                    symbol |= (bit & 1) << (bits_per_symbol - 1 - i);
                }
                let (sym_samples, new_phase) = self.generate_symbol(symbol, phase);
                samples.extend(sym_samples);
                phase = new_phase;
            }
        }

        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let sps = self.sps();
        let mut result = DemodResult::default();

        if samples.len() < sps {
            return result;
        }

        // Envelope detection: measure amplitude in each symbol period
        let envelopes: Vec<f64> = samples.chunks(sps)
            .map(|chunk| {
                // RMS amplitude
                let power: f64 = chunk.iter().map(|s| s.norm_sqr()).sum::<f64>() / chunk.len() as f64;
                power.sqrt()
            })
            .collect();

        // Calculate expected amplitude levels from modulation parameters
        // This is more robust than finding min/max from data (which fails if not all levels are present)
        let expected_levels: Vec<f64> = (0..self.num_levels)
            .map(|i| self.common.amplitude * self.symbol_to_amplitude(i as u8))
            .collect();

        let min_level = expected_levels[0];
        let max_level = expected_levels[self.num_levels - 1];

        // Collect individual bits first
        let mut individual_bits = Vec::new();

        // Decision thresholds
        if self.num_levels == 2 {
            let threshold = (max_level + min_level) / 2.0;
            for &env in &envelopes {
                let bit = if env > threshold { 1u8 } else { 0u8 };
                individual_bits.push(bit);
                result.symbols.push(bit as u16);
            }
        } else {
            // Multi-level: divide into regions using expected levels
            let range = max_level - min_level;
            let step = range / (self.num_levels - 1) as f64;

            for &env in &envelopes {
                // Find closest expected level
                let normalized = (env - min_level) / step;
                let symbol = normalized.round().min((self.num_levels - 1) as f64).max(0.0) as u8;

                result.symbols.push(symbol as u16);

                // Convert symbol to bits
                let bits_per_sym = (self.num_levels as f64).log2() as usize;
                for i in (0..bits_per_sym).rev() {
                    individual_bits.push((symbol >> i) & 1);
                }
            }
        }

        // Pack individual bits into bytes for consistent output
        result.bits = bits_to_bytes(&individual_bits);

        // Estimate SNR
        if !envelopes.is_empty() {
            let max_env = envelopes.iter().cloned().fold(0.0_f64, f64::max);
            let min_env = envelopes.iter().cloned().fold(f64::MAX, f64::min);
            if max_env > 0.0 && min_env < max_env {
                let snr_linear = max_env / min_env.max(1e-10);
                result.snr_estimate = Some(10.0 * snr_linear.log10());
            }
        }

        result
    }

    fn samples_per_symbol(&self) -> usize {
        self.sps()
    }

    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);

        // Constellation: points on real axis at different amplitudes
        let constellation: Vec<IQSample> = (0..self.num_levels)
            .map(|i| {
                let amp = if self.suppress_carrier {
                    let normalized = i as f64 / (self.num_levels - 1) as f64;
                    self.common.amplitude * self.modulation_index * (2.0 * normalized - 1.0)
                } else {
                    self.common.amplitude * self.symbol_to_amplitude(i as u8)
                };
                IQSample::new(amp, 0.0)
            })
            .collect();

        let labels: Vec<String> = if self.num_levels == 2 {
            vec!["0".to_string(), "1".to_string()]
        } else {
            (0..self.num_levels).map(|i| format!("{:02b}", i)).collect()
        };

        let mode = if self.suppress_carrier { "DSB-SC" } else { "ASK-DSB" };

        VisualizationData {
            samples,
            constellation,
            constellation_labels: labels,
            spectrum: Vec::new(),
            description: format!(
                "{}-level AM ({}): m={:.0}%, fc={:.0} Hz",
                self.num_levels,
                mode,
                self.modulation_index * 100.0,
                self.carrier_freq
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_am_modulation() {
        let common = CommonParams {
            sample_rate: 10000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let am = ASK::new_binary(common, 100.0, 1000.0);

        let data = vec![0, 1, 0, 1];
        let samples = am.modulate(&data);

        assert_eq!(samples.len(), 400); // 4 symbols * 100 samples
    }

    #[test]
    fn test_am_binary_roundtrip() {
        let common = CommonParams {
            sample_rate: 10000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let am = ASK::new_binary(common, 100.0, 1000.0);

        let data = vec![0, 1, 1, 0, 1, 0, 0, 1];
        let samples = am.modulate(&data);
        let result = am.demodulate(&samples);

        // Demodulate returns packed bytes, so compare against packed input
        assert_eq!(result.bits, bits_to_bytes(&data));
    }

    #[test]
    fn test_am_4level() {
        let common = CommonParams {
            sample_rate: 10000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let am = ASK::new_4ask(common, 100.0, 1000.0);

        assert_eq!(am.num_levels(), 4);
        assert_eq!(am.info().bits_per_symbol, 2);
    }

    #[test]
    fn test_am_modulation_index() {
        let common = CommonParams::default();
        let am = ASK::new_binary(common, 1000.0, 1000.0)
            .with_modulation_index(0.5);

        assert!((am.modulation_index() - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_ask_suppressed_carrier() {
        let common = CommonParams {
            sample_rate: 10000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let ask = ASK::new_binary(common, 100.0, 1000.0)
            .with_suppressed_carrier(true);

        let info = ask.info();
        assert_eq!(info.name, "ASK-SC");
    }
}
