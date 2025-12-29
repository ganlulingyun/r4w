//! QAM (Quadrature Amplitude Modulation)
//!
//! QAM combines amplitude AND phase modulation for maximum spectral efficiency.
//! It's used in WiFi, LTE, cable modems, and more.
//!
//! ## Constellation
//!
//! ```text
//! 16-QAM (4 bits per symbol):
//!
//!      Q
//!      │
//!   ●  ●  │  ●  ●
//!   ●  ●  │  ●  ●
//!   ───────┼───────→ I
//!   ●  ●  │  ●  ●
//!   ●  ●  │  ●  ●
//!
//! 16 points = 4 bits per symbol
//! ```
//!
//! ## Variants
//!
//! - 4-QAM: Same as QPSK (4 points, 2 bits/symbol)
//! - 16-QAM: 16 points, 4 bits/symbol
//! - 64-QAM: 64 points, 6 bits/symbol
//! - 256-QAM: 256 points, 8 bits/symbol
//!
//! ## Trade-offs
//!
//! Higher QAM = more bits/symbol but requires higher SNR!
//! - 16-QAM needs ~17 dB SNR
//! - 64-QAM needs ~23 dB SNR
//! - 256-QAM needs ~30 dB SNR

use super::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};
use crate::types::IQSample;

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

/// QAM modulator/demodulator
#[derive(Debug, Clone)]
pub struct QAM {
    /// Common waveform parameters
    common: CommonParams,
    /// Symbol rate in symbols per second
    symbol_rate: f64,
    /// QAM order (16, 64, 256, etc.)
    order: usize,
    /// Constellation points (pre-computed)
    constellation: Vec<IQSample>,
    /// Gray-coded symbol mapping
    gray_map: Vec<usize>,
}

impl QAM {
    /// Create a new QAM modulator
    pub fn new(common: CommonParams, symbol_rate: f64, order: usize) -> Self {
        let mut qam = Self {
            common,
            symbol_rate,
            order,
            constellation: Vec::new(),
            gray_map: Vec::new(),
        };
        qam.compute_constellation();
        qam
    }

    /// Create 16-QAM
    pub fn new_16qam(common: CommonParams, symbol_rate: f64) -> Self {
        Self::new(common, symbol_rate, 16)
    }

    /// Create 64-QAM
    pub fn new_64qam(common: CommonParams, symbol_rate: f64) -> Self {
        Self::new(common, symbol_rate, 64)
    }

    /// Create 256-QAM
    pub fn new_256qam(common: CommonParams, symbol_rate: f64) -> Self {
        Self::new(common, symbol_rate, 256)
    }

    /// Compute constellation points
    fn compute_constellation(&mut self) {
        let side = (self.order as f64).sqrt() as usize;
        let amp = self.common.amplitude;

        // Normalization factor for average power = 1
        let mut sum_power = 0.0;
        let mut points = Vec::new();

        for i in 0..side {
            for q in 0..side {
                // Map to symmetric grid: -3, -1, +1, +3 for 16-QAM
                let i_val = (2.0 * i as f64 - (side - 1) as f64) as f64;
                let q_val = (2.0 * q as f64 - (side - 1) as f64) as f64;
                points.push((i_val, q_val));
                sum_power += i_val * i_val + q_val * q_val;
            }
        }

        // Normalize
        let norm = (sum_power / self.order as f64).sqrt();

        self.constellation = points
            .iter()
            .map(|&(i, q)| IQSample::new(amp * i / norm, amp * q / norm))
            .collect();

        // Simple gray coding for square QAM
        self.gray_map = self.compute_gray_map(side);
    }

    /// Compute Gray code mapping for square constellation
    fn compute_gray_map(&self, side: usize) -> Vec<usize> {
        // Standard QAM Gray coding
        let gray_1d: Vec<usize> = match side {
            2 => vec![0, 1],
            4 => vec![0, 1, 3, 2],
            8 => vec![0, 1, 3, 2, 6, 7, 5, 4],
            16 => vec![0, 1, 3, 2, 6, 7, 5, 4, 12, 13, 15, 14, 10, 11, 9, 8],
            _ => (0..side).collect(),
        };

        // 2D Gray code mapping
        let mut map = vec![0; self.order];
        for (idx, &gi) in gray_1d.iter().enumerate() {
            for (jdx, &gq) in gray_1d.iter().enumerate() {
                let symbol = idx * side + jdx;
                let gray_symbol = gi * side + gq;
                map[gray_symbol] = symbol;
            }
        }

        map
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
        (self.order as f64).log2() as u8
    }

    /// Convert bits to symbol index
    fn bits_to_symbol(&self, bits: &[u8]) -> usize {
        let mut value = 0usize;
        for &bit in bits {
            value = (value << 1) | (bit as usize & 1);
        }
        // Apply gray mapping
        self.gray_map.get(value).copied().unwrap_or(0)
    }

    /// Convert symbol index to bits
    fn symbol_to_bits(&self, symbol: usize) -> Vec<u8> {
        // Find the gray code value for this symbol
        let gray_value = self.gray_map
            .iter()
            .position(|&s| s == symbol)
            .unwrap_or(0);

        let bps = self.bits_per_symbol() as usize;
        (0..bps)
            .rev()
            .map(|i| ((gray_value >> i) & 1) as u8)
            .collect()
    }

    /// Generate samples for one symbol
    fn generate_symbol(&self, symbol: usize) -> Vec<IQSample> {
        let point = self.constellation[symbol % self.order];
        vec![point; self.sps()]
    }

    /// Get required SNR for this QAM order (approximate)
    pub fn required_snr_db(&self) -> f64 {
        match self.order {
            4 => 10.0,
            16 => 17.0,
            64 => 23.0,
            256 => 30.0,
            _ => 10.0 * (self.order as f64).log2(),
        }
    }
}

impl Waveform for QAM {
    fn info(&self) -> WaveformInfo {
        let name = match self.order {
            4 => "4-QAM",
            16 => "16-QAM",
            64 => "64-QAM",
            256 => "256-QAM",
            _ => "M-QAM",
        };

        WaveformInfo {
            name,
            full_name: "Quadrature Amplitude Modulation",
            description: "Combines amplitude and phase for high spectral efficiency",
            complexity: 4,
            bits_per_symbol: self.bits_per_symbol(),
            carries_data: true,
            characteristics: &[
                "Amplitude + phase modulation",
                "High spectral efficiency",
                "Used in WiFi, LTE, cable",
                "Requires higher SNR for higher orders",
            ],
            history: "QAM was developed at Bell Labs in the 1960s for telephone modems. \
                16-QAM appeared in V.32 modems (1984). DOCSIS cable modems (1997) pushed \
                to 64-QAM and beyond. The combination of amplitude and phase modulation \
                unlocked unprecedented spectral efficiency for wired and wireless links.",
            modern_usage: "Dominant in high-throughput systems: WiFi 6 uses up to 1024-QAM, \
                5G NR uses 256-QAM, DOCSIS 3.1 cable reaches 4096-QAM. DVB-C and ATSC 3.0 \
                digital TV, ADSL/VDSL broadband, and microwave backhaul all rely on QAM. \
                The workhorse of modern high-speed digital communications.",
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

        let bps = self.bits_per_symbol() as usize;
        let mut samples = Vec::new();

        for chunk in bits.chunks(bps) {
            let mut bit_chunk = chunk.to_vec();
            while bit_chunk.len() < bps {
                bit_chunk.push(0);
            }
            let symbol = self.bits_to_symbol(&bit_chunk);
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
            // Average the samples
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

        // Calculate EVM
        let mut evm_sum = 0.0;
        let mut count = 0;
        for (chunk, &symbol) in samples.chunks(sps).zip(result.symbols.iter()) {
            let reference = self.constellation[symbol as usize];
            let avg: IQSample = chunk.iter().fold(IQSample::new(0.0, 0.0), |acc, &s| acc + s)
                / chunk.len() as f64;
            let error = (avg - reference).norm_sqr();
            evm_sum += error;
            count += 1;
        }

        if count > 0 {
            let rms_evm = (evm_sum / count as f64).sqrt();
            result.metadata.insert("evm_rms".to_string(), rms_evm);
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

        let labels: Vec<String> = (0..self.order.min(16))
            .map(|i| {
                let bits = self.symbol_to_bits(i);
                let bit_str: String = bits.iter().map(|b| char::from(b'0' + b)).collect();
                bit_str
            })
            .collect();

        VisualizationData {
            samples,
            constellation: self.constellation.clone(),
            constellation_labels: labels,
            spectrum: Vec::new(),
            description: format!(
                "{}-QAM: {} bits/symbol, requires ~{:.0} dB SNR",
                self.order,
                self.bits_per_symbol(),
                self.required_snr_db()
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_16qam_constellation() {
        let common = CommonParams::default();
        let qam = QAM::new_16qam(common, 1000.0);

        assert_eq!(qam.constellation.len(), 16);
        assert_eq!(qam.bits_per_symbol(), 4);
    }

    #[test]
    fn test_16qam_roundtrip() {
        let common = CommonParams {
            sample_rate: 10000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let qam = QAM::new_16qam(common, 1000.0);

        // 16 bits = 4 symbols for 16-QAM
        let data = vec![0, 0, 0, 0, 1, 1, 1, 1, 0, 1, 0, 1, 1, 0, 1, 0];
        let samples = qam.modulate(&data);
        let result = qam.demodulate(&samples);

        // Demodulate returns packed bytes, so compare against packed input
        assert_eq!(result.bits, bits_to_bytes(&data));
    }

    #[test]
    fn test_64qam() {
        let common = CommonParams::default();
        let qam = QAM::new_64qam(common, 1000.0);

        assert_eq!(qam.constellation.len(), 64);
        assert_eq!(qam.bits_per_symbol(), 6);
    }

    #[test]
    fn test_constellation_power() {
        let common = CommonParams {
            sample_rate: 10000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let qam = QAM::new_16qam(common, 1000.0);

        // Average power should be approximately 1
        let avg_power: f64 = qam.constellation.iter()
            .map(|p| p.norm_sqr())
            .sum::<f64>() / qam.constellation.len() as f64;

        assert!((avg_power - 1.0).abs() < 0.1);
    }
}
