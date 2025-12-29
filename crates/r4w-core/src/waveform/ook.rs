//! OOK (On-Off Keying) - The Simplest Digital Modulation
//!
//! OOK encodes binary data by switching the carrier on and off:
//! - '1' → Carrier ON
//! - '0' → Carrier OFF
//!
//! ## Characteristics
//!
//! - 1 bit per symbol
//! - Very simple to implement
//! - Used in car remotes, garage openers, RFID
//! - Poor noise immunity (0 and noise look similar)
//!
//! ## Demodulation
//!
//! Envelope detection: measure signal power in each symbol period

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

/// On-Off Keying modulator/demodulator
#[derive(Debug, Clone)]
pub struct OOK {
    /// Common waveform parameters
    common: CommonParams,
    /// Symbol rate in symbols per second
    symbol_rate: f64,
    /// Carrier frequency (relative to baseband)
    carrier_freq: f64,
}

impl OOK {
    /// Create a new OOK modulator
    ///
    /// # Arguments
    /// * `common` - Common parameters
    /// * `symbol_rate` - Symbols (bits) per second
    pub fn new(common: CommonParams, symbol_rate: f64) -> Self {
        Self {
            common,
            symbol_rate,
            carrier_freq: 1000.0, // Default 1kHz carrier
        }
    }

    /// Set carrier frequency
    pub fn with_carrier(mut self, freq: f64) -> Self {
        self.carrier_freq = freq;
        self
    }

    /// Get samples per symbol (minimum 1 to prevent division by zero)
    fn sps(&self) -> usize {
        if self.symbol_rate <= 0.0 {
            return 1;
        }
        ((self.common.sample_rate / self.symbol_rate) as usize).max(1)
    }

    /// Generate carrier samples for one symbol period
    fn generate_on(&self, start_phase: f64) -> (Vec<IQSample>, f64) {
        let sps = self.sps();
        let omega = 2.0 * PI * self.carrier_freq / self.common.sample_rate;
        let amp = self.common.amplitude;

        let samples: Vec<IQSample> = (0..sps)
            .map(|n| {
                let phase = start_phase + omega * n as f64;
                IQSample::new(amp * phase.cos(), amp * phase.sin())
            })
            .collect();

        let end_phase = start_phase + omega * sps as f64;
        (samples, end_phase)
    }

    /// Generate off (zero) samples for one symbol period
    fn generate_off(&self) -> Vec<IQSample> {
        vec![IQSample::new(0.0, 0.0); self.sps()]
    }
}

impl Waveform for OOK {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "OOK",
            full_name: "On-Off Keying",
            description: "Binary modulation by switching carrier on/off",
            complexity: 2,
            bits_per_symbol: 1,
            carries_data: true,
            characteristics: &[
                "Simplest digital modulation",
                "1 bit per symbol",
                "Carrier ON = 1, OFF = 0",
                "Envelope detection for demod",
                "Poor noise performance",
            ],
            history: "OOK evolved from spark-gap transmitters of the early 1900s. \
                The first practical digital modulation, it enabled Morse code without \
                manual keying. Adopted widely for simple remote control systems in \
                the 1970s-80s as electronics became affordable.",
            modern_usage: "Ubiquitous in low-cost IoT: garage door openers, car key fobs, \
                wireless doorbells, and 433/915 MHz sensors. Used by Oregon Scientific \
                weather stations, tire pressure monitors, and billions of simple RF devices. \
                Being replaced by more robust schemes in new designs but legacy is massive.",
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

        let mut samples = Vec::with_capacity(bits.len() * self.sps());
        let mut phase = 0.0;

        for &bit in &bits {
            if bit != 0 {
                let (on_samples, new_phase) = self.generate_on(phase);
                samples.extend(on_samples);
                phase = new_phase;
            } else {
                samples.extend(self.generate_off());
                // Keep phase continuous for when carrier comes back
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

        // Calculate threshold from signal statistics
        let powers: Vec<f64> = samples.chunks(sps)
            .map(|chunk| {
                chunk.iter().map(|s| s.norm_sqr()).sum::<f64>() / chunk.len() as f64
            })
            .collect();

        // Adaptive threshold: midpoint between min and max power
        let max_power = powers.iter().cloned().fold(0.0_f64, f64::max);
        let min_power = powers.iter().cloned().fold(f64::MAX, f64::min);
        let threshold = (max_power + min_power) / 2.0;

        // Collect individual bits
        let mut individual_bits = Vec::new();
        for &power in &powers {
            let bit = if power > threshold { 1 } else { 0 };
            individual_bits.push(bit);
            result.symbols.push(bit as u16);
        }

        // Pack individual bits into bytes for consistent output
        result.bits = bits_to_bytes(&individual_bits);

        // Estimate SNR
        if max_power > 0.0 && min_power < max_power {
            let snr_linear = max_power / min_power.max(1e-10);
            result.snr_estimate = Some(10.0 * snr_linear.log10());
        }

        result
    }

    fn samples_per_symbol(&self) -> usize {
        self.sps()
    }

    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);

        // Constellation for OOK: two points on real axis
        let constellation = vec![
            IQSample::new(0.0, 0.0),              // OFF
            IQSample::new(self.common.amplitude, 0.0), // ON
        ];

        VisualizationData {
            samples,
            constellation,
            constellation_labels: vec!["0 (OFF)".to_string(), "1 (ON)".to_string()],
            spectrum: Vec::new(),
            description: format!(
                "OOK at {} symbols/sec - carrier switches on/off",
                self.symbol_rate
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ook_modulation() {
        let common = CommonParams {
            sample_rate: 10000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let ook = OOK::new(common, 100.0); // 100 symbols/sec = 100 samples/symbol

        let data = vec![1, 0, 1, 1, 0];
        let samples = ook.modulate(&data);

        assert_eq!(samples.len(), 500); // 5 symbols * 100 samples

        // Check that '1' bits have signal and '0' bits don't
        let sps = 100;

        // First symbol is '1' - should have power
        let power_0: f64 = samples[0..sps].iter().map(|s| s.norm_sqr()).sum();
        assert!(power_0 > 0.0);

        // Second symbol is '0' - should be zero
        let power_1: f64 = samples[sps..2*sps].iter().map(|s| s.norm_sqr()).sum();
        assert!(power_1 < 1e-10);
    }

    #[test]
    fn test_ook_roundtrip() {
        let common = CommonParams {
            sample_rate: 10000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let ook = OOK::new(common, 100.0);

        let data = vec![1, 0, 1, 1, 0, 0, 1, 0];
        let samples = ook.modulate(&data);
        let result = ook.demodulate(&samples);

        // Demodulate returns packed bytes, so compare against packed input
        assert_eq!(result.bits, bits_to_bytes(&data));
    }
}
