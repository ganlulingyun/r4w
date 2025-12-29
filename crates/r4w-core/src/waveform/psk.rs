//! PSK (Phase Shift Keying)
//!
//! PSK encodes data in the phase of the carrier signal.
//!
//! ## Variants
//!
//! - **BPSK**: 2 phases (0°, 180°), 1 bit/symbol
//! - **QPSK**: 4 phases (45°, 135°, 225°, 315°), 2 bits/symbol
//! - **8-PSK**: 8 phases, 3 bits/symbol
//!
//! ## Constellation
//!
//! ```text
//! BPSK:                    QPSK:
//!     Q                        Q
//!     │                   10   │   00
//!     │                     ●  │  ●
//! ●───┼───●              ─────┼─────
//! 0   │   1                 ●  │  ●
//!     │                   11   │   01
//! ```
//!
//! ## Gray Coding
//!
//! Adjacent constellation points differ by only 1 bit,
//! minimizing bit errors when symbol errors occur.

use super::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};
use crate::types::IQSample;
use std::f64::consts::PI;

/// PSK modulator/demodulator
#[derive(Debug, Clone)]
pub struct PSK {
    /// Common waveform parameters
    common: CommonParams,
    /// Symbol rate in symbols per second
    symbol_rate: f64,
    /// Number of phase levels (2 for BPSK, 4 for QPSK, etc.)
    num_phases: usize,
    /// Phase offset for constellation (radians)
    phase_offset: f64,
    /// Constellation points (pre-computed)
    constellation: Vec<IQSample>,
    /// Gray code mapping
    gray_map: Vec<u8>,
}

impl PSK {
    /// Create a new PSK modulator
    pub fn new(common: CommonParams, symbol_rate: f64, num_phases: usize) -> Self {
        let phase_offset = if num_phases == 4 { PI / 4.0 } else { 0.0 };
        let mut psk = Self {
            common,
            symbol_rate,
            num_phases,
            phase_offset,
            constellation: Vec::new(),
            gray_map: Vec::new(),
        };
        psk.compute_constellation();
        psk
    }

    /// Create BPSK (Binary PSK)
    pub fn new_bpsk(common: CommonParams, symbol_rate: f64) -> Self {
        Self::new(common, symbol_rate, 2)
    }

    /// Create QPSK (Quadrature PSK)
    pub fn new_qpsk(common: CommonParams, symbol_rate: f64) -> Self {
        Self::new(common, symbol_rate, 4)
    }

    /// Create 8-PSK
    pub fn new_8psk(common: CommonParams, symbol_rate: f64) -> Self {
        Self::new(common, symbol_rate, 8)
    }

    /// Compute constellation points with Gray coding
    fn compute_constellation(&mut self) {
        let amp = self.common.amplitude;

        self.constellation = (0..self.num_phases)
            .map(|i| {
                let angle = self.phase_offset + 2.0 * PI * i as f64 / self.num_phases as f64;
                IQSample::new(amp * angle.cos(), amp * angle.sin())
            })
            .collect();

        // Gray coding for common cases
        self.gray_map = match self.num_phases {
            2 => vec![0, 1],
            4 => vec![0, 1, 3, 2],        // 00, 01, 11, 10
            8 => vec![0, 1, 3, 2, 6, 7, 5, 4], // Gray code order
            _ => (0..self.num_phases as u8).collect(),
        };
    }

    /// Get samples per symbol (minimum 1 to prevent division by zero)
    fn sps(&self) -> usize {
        if self.symbol_rate <= 0.0 {
            return 1;
        }
        ((self.common.sample_rate / self.symbol_rate) as usize).max(1)
    }

    /// Bits per symbol
    fn bits_per_symbol(&self) -> u8 {
        (self.num_phases as f64).log2() as u8
    }

    /// Convert bits to symbol index using Gray coding
    fn bits_to_symbol(&self, bits: &[u8]) -> usize {
        let mut value = 0usize;
        for &bit in bits {
            value = (value << 1) | (bit as usize & 1);
        }
        // Find this value in gray_map
        self.gray_map.iter().position(|&g| g as usize == value).unwrap_or(0)
    }

    /// Convert symbol index to bits using Gray coding
    fn symbol_to_bits(&self, symbol: usize) -> Vec<u8> {
        let gray_value = self.gray_map[symbol % self.num_phases] as usize;
        let bps = self.bits_per_symbol() as usize;
        (0..bps)
            .rev()
            .map(|i| ((gray_value >> i) & 1) as u8)
            .collect()
    }

    /// Generate samples for one symbol
    fn generate_symbol(&self, symbol: usize) -> Vec<IQSample> {
        let point = self.constellation[symbol % self.num_phases];
        vec![point; self.sps()]
    }
}

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

impl Waveform for PSK {
    fn info(&self) -> WaveformInfo {
        let (name, full_name) = match self.num_phases {
            2 => ("BPSK", "Binary Phase Shift Keying"),
            4 => ("QPSK", "Quadrature Phase Shift Keying"),
            8 => ("8-PSK", "8-Phase Shift Keying"),
            _ => ("M-PSK", "Multi-Phase Shift Keying"),
        };

        WaveformInfo {
            name,
            full_name,
            description: "Encodes data in the phase of the carrier",
            complexity: 3,
            bits_per_symbol: self.bits_per_symbol(),
            carries_data: true,
            characteristics: &[
                "Phase encodes information",
                "Constant envelope",
                "Used in WiFi, satellites, deep space",
                "Gray coding minimizes bit errors",
            ],
            history: "BPSK was developed in the 1950s for satellite telemetry. QPSK emerged \
                in the 1960s for higher throughput. NASA's Voyager spacecraft (1977) uses \
                BPSK to communicate across billions of miles. Differential PSK variants \
                solved carrier recovery challenges in early digital radio.",
            modern_usage: "Foundation of modern wireless: QPSK used in DVB-S satellite TV, \
                GPS signals, 3G/4G cellular, and as base modulation in WiFi/LTE. BPSK used \
                for control channels and deep-space links (Voyager, Mars rovers). 8-PSK \
                in EDGE cellular and some satellite systems. Extremely active development.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        let bps = self.bits_per_symbol() as usize;
        let mut samples = Vec::new();

        // Convert packed bytes to individual bits if needed
        let bits = if is_packed_bytes(data) {
            bytes_to_bits(data)
        } else {
            data.to_vec()
        };

        // Group bits into symbols
        for chunk in bits.chunks(bps) {
            let mut symbol_bits = chunk.to_vec();
            // Pad with zeros if needed
            while symbol_bits.len() < bps {
                symbol_bits.push(0);
            }
            let symbol = self.bits_to_symbol(&symbol_bits);
            samples.extend(self.generate_symbol(symbol));
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

        for chunk in samples.chunks(sps) {
            // Average the samples in this symbol period
            let avg: IQSample = chunk.iter().fold(IQSample::new(0.0, 0.0), |acc, &s| acc + s)
                / chunk.len() as f64;

            // Find nearest constellation point
            let mut best_symbol = 0;
            let mut best_dist = f64::MAX;

            for (i, &point) in self.constellation.iter().enumerate() {
                let dist = (avg - point).norm_sqr();
                if dist < best_dist {
                    best_dist = dist;
                    best_symbol = i;
                }
            }

            result.symbols.push(best_symbol as u16);
            individual_bits.extend(self.symbol_to_bits(best_symbol));
        }

        // Pack individual bits into bytes for consistent output
        result.bits = bits_to_bytes(&individual_bits);

        // Estimate EVM (Error Vector Magnitude) as quality metric
        let mut evm_sum = 0.0;
        let mut count = 0;
        for (chunk, &symbol) in samples.chunks(sps).zip(result.symbols.iter()) {
            let reference = self.constellation[symbol as usize];
            let avg: IQSample = chunk.iter().fold(IQSample::new(0.0, 0.0), |acc, &s| acc + s)
                / chunk.len() as f64;
            let error = (avg - reference).norm();
            evm_sum += error * error;
            count += 1;
        }

        if count > 0 {
            let rms_evm = (evm_sum / count as f64).sqrt();
            result.metadata.insert("evm_rms".to_string(), rms_evm);
            // Rough SNR estimate from EVM
            if rms_evm > 0.0 {
                result.snr_estimate = Some(-20.0 * rms_evm.log10());
            }
        }

        result
    }

    fn samples_per_symbol(&self) -> usize {
        self.sps()
    }

    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);

        let labels: Vec<String> = (0..self.num_phases)
            .map(|i| {
                let bits = self.symbol_to_bits(i);
                let bit_str: String = bits.iter().map(|b| char::from(b'0' + b)).collect();
                format!("{} ({})", i, bit_str)
            })
            .collect();

        VisualizationData {
            samples,
            constellation: self.constellation.clone(),
            constellation_labels: labels,
            spectrum: Vec::new(),
            description: format!(
                "{}-PSK: {} phases, {} bits/symbol",
                self.num_phases,
                self.num_phases,
                self.bits_per_symbol()
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bpsk_constellation() {
        let common = CommonParams::default();
        let bpsk = PSK::new_bpsk(common, 1000.0);

        assert_eq!(bpsk.constellation.len(), 2);
        // BPSK: points at 0° and 180°
        assert!((bpsk.constellation[0].re - 1.0).abs() < 0.01);
        assert!((bpsk.constellation[1].re + 1.0).abs() < 0.01);
    }

    #[test]
    fn test_qpsk_constellation() {
        let common = CommonParams::default();
        let qpsk = PSK::new_qpsk(common, 1000.0);

        assert_eq!(qpsk.constellation.len(), 4);
        // QPSK: points at 45°, 135°, 225°, 315°
        for point in &qpsk.constellation {
            let mag = point.norm();
            assert!((mag - 1.0).abs() < 0.01);
        }
    }

    #[test]
    fn test_bpsk_roundtrip() {
        let common = CommonParams {
            sample_rate: 10000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let bpsk = PSK::new_bpsk(common, 1000.0);

        let data = vec![0, 1, 1, 0, 1, 0, 0, 1];
        let samples = bpsk.modulate(&data);
        let result = bpsk.demodulate(&samples);

        // Demodulate returns packed bytes, so compare against packed input
        assert_eq!(result.bits, bits_to_bytes(&data));
    }

    #[test]
    fn test_qpsk_roundtrip() {
        let common = CommonParams {
            sample_rate: 10000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let qpsk = PSK::new_qpsk(common, 1000.0);

        // QPSK: 2 bits per symbol, so 8 bits = 4 symbols
        let data = vec![0, 0, 0, 1, 1, 1, 1, 0];
        let samples = qpsk.modulate(&data);
        let result = qpsk.demodulate(&samples);

        // Demodulate returns packed bytes, so compare against packed input
        assert_eq!(result.bits, bits_to_bytes(&data));
    }

    #[test]
    fn test_gray_coding() {
        let common = CommonParams::default();
        let qpsk = PSK::new_qpsk(common, 1000.0);

        // Adjacent symbols should differ by 1 bit
        let bits_0 = qpsk.symbol_to_bits(0);
        let bits_1 = qpsk.symbol_to_bits(1);

        let diff: usize = bits_0.iter().zip(bits_1.iter())
            .filter(|(&a, &b)| a != b).count();

        assert_eq!(diff, 1, "Adjacent symbols should differ by 1 bit");
    }
}
