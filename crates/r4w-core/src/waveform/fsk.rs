//! FSK (Frequency Shift Keying)
//!
//! FSK encodes data by shifting between different frequencies:
//! - Binary FSK (BFSK): Two frequencies for 0 and 1
//! - M-FSK: Multiple frequencies for log2(M) bits per symbol
//!
//! ## Mathematical Definition
//!
//! ```text
//! s(t) = A · e^(j·2π·(fc + d[n]·Δf)·t)
//! ```
//! Where d[n] is the data symbol and Δf is the frequency deviation.
//!
//! ## Key Parameters
//!
//! - **Deviation (Δf)**: Frequency offset from center
//! - **Modulation Index (h)**: h = 2·Δf / symbol_rate
//! - **Symbol Rate**: Symbols per second
//!
//! ## Variants
//!
//! - BFSK: Binary, 2 frequencies
//! - 4-FSK: 4 frequencies, 2 bits/symbol
//! - GFSK: Gaussian filtered (Bluetooth)
//! - MSK: Minimum Shift Keying (h = 0.5)

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

/// FSK modulator/demodulator
#[derive(Debug, Clone)]
pub struct FSK {
    /// Common waveform parameters
    common: CommonParams,
    /// Symbol rate in symbols per second
    symbol_rate: f64,
    /// Frequency deviation in Hz
    deviation: f64,
    /// Number of frequency levels (2 for BFSK, 4 for 4-FSK, etc.)
    num_levels: usize,
    /// Whether to use continuous phase (CPFSK)
    continuous_phase: bool,
}

impl FSK {
    /// Create a new FSK modulator
    pub fn new(
        common: CommonParams,
        symbol_rate: f64,
        deviation: f64,
        num_levels: usize,
    ) -> Self {
        Self {
            common,
            symbol_rate,
            deviation,
            num_levels,
            continuous_phase: true,
        }
    }

    /// Create Binary FSK (BFSK)
    pub fn new_bfsk(common: CommonParams, symbol_rate: f64, deviation: f64) -> Self {
        Self::new(common, symbol_rate, deviation, 2)
    }

    /// Create 4-FSK (2 bits per symbol)
    pub fn new_4fsk(common: CommonParams, symbol_rate: f64, deviation: f64) -> Self {
        Self::new(common, symbol_rate, deviation, 4)
    }

    /// Set continuous phase mode
    pub fn with_continuous_phase(mut self, enabled: bool) -> Self {
        self.continuous_phase = enabled;
        self
    }

    /// Get samples per symbol (minimum 1 to prevent division by zero)
    fn sps(&self) -> usize {
        if self.symbol_rate <= 0.0 {
            return 1;
        }
        ((self.common.sample_rate / self.symbol_rate) as usize).max(1)
    }

    /// Get modulation index
    pub fn modulation_index(&self) -> f64 {
        2.0 * self.deviation / self.symbol_rate
    }

    /// Map a symbol to frequency offset
    fn symbol_to_freq(&self, symbol: u8) -> f64 {
        // Map symbol to range [-1, 1] then scale by deviation
        let normalized = if self.num_levels == 2 {
            if symbol == 0 { -1.0 } else { 1.0 }
        } else {
            // For M-FSK, map to evenly spaced values
            let max_symbol = (self.num_levels - 1) as f64;
            2.0 * (symbol as f64 / max_symbol) - 1.0
        };
        normalized * self.deviation
    }

    /// Generate samples for one symbol
    fn generate_symbol(&self, symbol: u8, start_phase: f64) -> (Vec<IQSample>, f64) {
        let sps = self.sps();
        let freq = self.symbol_to_freq(symbol);
        let omega = 2.0 * PI * freq / self.common.sample_rate;
        let amp = self.common.amplitude;

        let samples: Vec<IQSample> = (0..sps)
            .map(|n| {
                let phase = start_phase + omega * n as f64;
                IQSample::new(amp * phase.cos(), amp * phase.sin())
            })
            .collect();

        let end_phase = if self.continuous_phase {
            start_phase + omega * sps as f64
        } else {
            0.0
        };

        (samples, end_phase)
    }
}

impl Waveform for FSK {
    fn info(&self) -> WaveformInfo {
        let (name, full_name, bits) = match self.num_levels {
            2 => ("BFSK", "Binary Frequency Shift Keying", 1),
            4 => ("4-FSK", "4-Level Frequency Shift Keying", 2),
            _ => ("M-FSK", "Multi-level Frequency Shift Keying",
                  (self.num_levels as f64).log2() as u8),
        };

        WaveformInfo {
            name,
            full_name,
            description: "Encodes data by shifting between frequencies",
            complexity: 3,
            bits_per_symbol: bits,
            carries_data: true,
            characteristics: &[
                "Frequency shifts encode data",
                "Constant envelope (good for non-linear amplifiers)",
                "More robust than OOK",
                "Used in pagers, Bluetooth (GFSK)",
            ],
            history: "Developed in the 1930s for radio teletype (RTTY). Became popular \
                in the 1960s for data modems. Bell 103 modem (1962) used FSK at 300 baud. \
                The constant-envelope property made it ideal for early power-limited \
                satellite and mobile systems.",
            modern_usage: "Extremely widespread: Bluetooth uses GFSK, DECT cordless phones, \
                pagers (POCSAG/FLEX), amateur radio (RTTY, packet), industrial telemetry, \
                and many ISM band devices. 4-FSK variants used in DMR digital radio and \
                P25 Phase 1 public safety radio. Still the go-to for simple, robust links.",
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
            // Binary FSK: each bit is a symbol
            for &bit in &bits {
                let symbol = bit & 1;
                let (sym_samples, new_phase) = self.generate_symbol(symbol, phase);
                samples.extend(sym_samples);
                phase = new_phase;
            }
        } else {
            // M-FSK: group bits into symbols
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

        // Collect individual bits first
        let mut individual_bits = Vec::new();

        // For each symbol period, estimate instantaneous frequency
        for chunk in samples.chunks(sps) {
            if chunk.len() < 2 {
                continue;
            }

            // Estimate frequency from phase differences
            let mut freq_estimates = Vec::new();
            for i in 1..chunk.len() {
                let phase_diff = (chunk[i] * chunk[i - 1].conj()).arg();
                let freq = phase_diff * self.common.sample_rate / (2.0 * PI);
                freq_estimates.push(freq);
            }

            let avg_freq: f64 = freq_estimates.iter().sum::<f64>() / freq_estimates.len() as f64;

            // Decision: find closest frequency level
            let mut best_symbol = 0u8;
            let mut best_error = f64::MAX;

            for sym in 0..self.num_levels as u8 {
                let expected_freq = self.symbol_to_freq(sym);
                let error = (avg_freq - expected_freq).abs();
                if error < best_error {
                    best_error = error;
                    best_symbol = sym;
                }
            }

            result.symbols.push(best_symbol as u16);

            // For BFSK, also store as bits
            if self.num_levels == 2 {
                individual_bits.push(best_symbol);
            } else {
                // For M-FSK, convert symbol to bits
                let bits_per_sym = (self.num_levels as f64).log2() as u8;
                for i in (0..bits_per_sym).rev() {
                    individual_bits.push((best_symbol >> i) & 1);
                }
            }
        }

        // Pack individual bits into bytes for consistent output
        result.bits = bits_to_bytes(&individual_bits);

        result
    }

    fn samples_per_symbol(&self) -> usize {
        self.sps()
    }

    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);

        // FSK doesn't have a traditional constellation
        // Show the frequency levels on the imaginary axis
        let constellation: Vec<IQSample> = (0..self.num_levels)
            .map(|i| {
                let freq = self.symbol_to_freq(i as u8);
                // Represent frequency as y-position
                IQSample::new(0.5, freq / self.deviation * 0.5)
            })
            .collect();

        let labels: Vec<String> = (0..self.num_levels)
            .map(|i| format!("f{} ({:+.0} Hz)", i, self.symbol_to_freq(i as u8)))
            .collect();

        VisualizationData {
            samples,
            constellation,
            constellation_labels: labels,
            spectrum: Vec::new(),
            description: format!(
                "{}-FSK: {} Hz deviation, h={:.2}",
                self.num_levels,
                self.deviation,
                self.modulation_index()
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bfsk_modulation() {
        let common = CommonParams {
            sample_rate: 10000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let fsk = FSK::new_bfsk(common, 100.0, 500.0);

        let data = vec![0, 1, 0, 1];
        let samples = fsk.modulate(&data);

        assert_eq!(samples.len(), 400); // 4 symbols * 100 samples

        // All samples should have magnitude ~1 (constant envelope)
        for s in &samples {
            assert!((s.norm() - 1.0).abs() < 0.01);
        }
    }

    #[test]
    fn test_bfsk_roundtrip() {
        let common = CommonParams {
            sample_rate: 10000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let fsk = FSK::new_bfsk(common, 100.0, 1000.0);

        let data = vec![1, 0, 1, 1, 0, 0, 1, 0];
        let samples = fsk.modulate(&data);
        let result = fsk.demodulate(&samples);

        // Demodulate returns packed bytes, so compare against packed input
        assert_eq!(result.bits, bits_to_bytes(&data));
    }

    #[test]
    fn test_modulation_index() {
        let common = CommonParams::default();
        let fsk = FSK::new_bfsk(common, 1000.0, 500.0);
        assert!((fsk.modulation_index() - 1.0).abs() < 0.001);
    }
}
