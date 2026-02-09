//! Constellation Soft Decoder — LLR output for soft-decision decoding
//!
//! Computes log-likelihood ratios (LLRs) from received constellation points,
//! enabling soft-decision decoding for turbo codes, LDPC, and convolutional
//! codes. Supports BPSK, QPSK, 8PSK, 16QAM, and 64QAM.
//! GNU Radio equivalent: `constellation_soft_decoder_cf`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::constellation_soft_decoder::{SoftDecoder, Modulation};
//! use num_complex::Complex64;
//!
//! let decoder = SoftDecoder::new(Modulation::Qpsk, 10.0);
//! let rx = Complex64::new(0.8, 0.9); // Near QPSK point (1,1)
//! let llrs = decoder.decode_symbol(rx);
//! assert_eq!(llrs.len(), 2); // 2 bits per QPSK symbol
//! // Positive LLR = bit likely 0, negative = likely 1
//! ```

use num_complex::Complex64;

/// Supported modulation schemes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Modulation {
    Bpsk,
    Qpsk,
    Psk8,
    Qam16,
    Qam64,
}

impl Modulation {
    /// Bits per symbol.
    pub fn bits_per_symbol(&self) -> usize {
        match self {
            Modulation::Bpsk => 1,
            Modulation::Qpsk => 2,
            Modulation::Psk8 => 3,
            Modulation::Qam16 => 4,
            Modulation::Qam64 => 6,
        }
    }

    /// Get constellation points.
    pub fn constellation(&self) -> Vec<Complex64> {
        match self {
            Modulation::Bpsk => vec![
                Complex64::new(1.0, 0.0),
                Complex64::new(-1.0, 0.0),
            ],
            Modulation::Qpsk => {
                let s = std::f64::consts::FRAC_1_SQRT_2;
                vec![
                    Complex64::new(s, s),    // 00
                    Complex64::new(-s, s),   // 01
                    Complex64::new(-s, -s),  // 11
                    Complex64::new(s, -s),   // 10
                ]
            }
            Modulation::Psk8 => {
                (0..8).map(|i| {
                    let angle = 2.0 * std::f64::consts::PI * i as f64 / 8.0;
                    Complex64::from_polar(1.0, angle)
                }).collect()
            }
            Modulation::Qam16 => {
                let levels = [-3.0, -1.0, 1.0, 3.0];
                let norm = 1.0 / (10.0f64).sqrt(); // Average power = 1
                let mut points = Vec::with_capacity(16);
                for &q in &levels {
                    for &i in &levels {
                        points.push(Complex64::new(i * norm, q * norm));
                    }
                }
                points
            }
            Modulation::Qam64 => {
                let levels = [-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0];
                let norm = 1.0 / (42.0f64).sqrt();
                let mut points = Vec::with_capacity(64);
                for &q in &levels {
                    for &i in &levels {
                        points.push(Complex64::new(i * norm, q * norm));
                    }
                }
                points
            }
        }
    }

    /// Get Gray-coded bit labels for each constellation point.
    pub fn bit_labels(&self) -> Vec<Vec<bool>> {
        match self {
            Modulation::Bpsk => vec![
                vec![false], // 0
                vec![true],  // 1
            ],
            Modulation::Qpsk => vec![
                vec![false, false], // 00
                vec![false, true],  // 01
                vec![true, true],   // 11
                vec![true, false],  // 10
            ],
            _ => {
                // Generic Gray code labeling
                let n = 1 << self.bits_per_symbol();
                let bps = self.bits_per_symbol();
                (0..n).map(|i| {
                    let gray = i ^ (i >> 1);
                    (0..bps).map(|b| (gray >> (bps - 1 - b)) & 1 == 1).collect()
                }).collect()
            }
        }
    }
}

/// Soft-decision constellation decoder.
#[derive(Debug, Clone)]
pub struct SoftDecoder {
    /// Constellation points.
    constellation: Vec<Complex64>,
    /// Bit labels per constellation point.
    labels: Vec<Vec<bool>>,
    /// Noise variance (σ² = N0/2).
    noise_var: f64,
    /// Bits per symbol.
    bps: usize,
}

impl SoftDecoder {
    /// Create a soft decoder.
    ///
    /// `snr_db`: estimated SNR in dB for LLR scaling.
    pub fn new(modulation: Modulation, snr_db: f64) -> Self {
        let constellation = modulation.constellation();
        let labels = modulation.bit_labels();
        let noise_var = 10f64.powf(-snr_db / 10.0) / 2.0;

        Self {
            constellation,
            labels,
            noise_var: noise_var.max(1e-10),
            bps: modulation.bits_per_symbol(),
        }
    }

    /// Create with explicit noise variance.
    pub fn with_noise_var(modulation: Modulation, noise_var: f64) -> Self {
        Self {
            constellation: modulation.constellation(),
            labels: modulation.bit_labels(),
            noise_var: noise_var.max(1e-10),
            bps: modulation.bits_per_symbol(),
        }
    }

    /// Decode a single received symbol to LLRs.
    ///
    /// Returns `bps` LLR values. Positive = bit likely 0, negative = bit likely 1.
    pub fn decode_symbol(&self, rx: Complex64) -> Vec<f64> {
        let mut llrs = vec![0.0; self.bps];

        for bit_idx in 0..self.bps {
            // Max-log MAP approximation:
            // LLR(b_k) = min_{s: b_k=1} |rx-s|²/(2σ²) - min_{s: b_k=0} |rx-s|²/(2σ²)
            let mut min_dist_0 = f64::INFINITY;
            let mut min_dist_1 = f64::INFINITY;

            for (i, point) in self.constellation.iter().enumerate() {
                let dist = (rx - point).norm_sqr();
                if self.labels[i][bit_idx] {
                    min_dist_1 = min_dist_1.min(dist);
                } else {
                    min_dist_0 = min_dist_0.min(dist);
                }
            }

            llrs[bit_idx] = (min_dist_1 - min_dist_0) / (2.0 * self.noise_var);
        }

        llrs
    }

    /// Decode a block of symbols.
    pub fn decode(&self, symbols: &[Complex64]) -> Vec<f64> {
        let mut llrs = Vec::with_capacity(symbols.len() * self.bps);
        for &s in symbols {
            llrs.extend(self.decode_symbol(s));
        }
        llrs
    }

    /// Hard decode a single symbol (nearest point).
    pub fn hard_decode(&self, rx: Complex64) -> Vec<bool> {
        let mut min_dist = f64::INFINITY;
        let mut best_idx = 0;
        for (i, point) in self.constellation.iter().enumerate() {
            let dist = (rx - point).norm_sqr();
            if dist < min_dist {
                min_dist = dist;
                best_idx = i;
            }
        }
        self.labels[best_idx].clone()
    }

    /// Set noise variance.
    pub fn set_noise_var(&mut self, noise_var: f64) {
        self.noise_var = noise_var.max(1e-10);
    }

    /// Set SNR in dB.
    pub fn set_snr_db(&mut self, snr_db: f64) {
        self.noise_var = (10f64.powf(-snr_db / 10.0) / 2.0).max(1e-10);
    }

    /// Get bits per symbol.
    pub fn bits_per_symbol(&self) -> usize {
        self.bps
    }

    /// Convert LLRs to hard bits.
    pub fn llr_to_bits(llrs: &[f64]) -> Vec<bool> {
        llrs.iter().map(|&l| l < 0.0).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bpsk_correct() {
        let decoder = SoftDecoder::new(Modulation::Bpsk, 20.0);
        // Received near +1 → bit 0 (positive LLR)
        let llrs = decoder.decode_symbol(Complex64::new(0.9, 0.0));
        assert_eq!(llrs.len(), 1);
        assert!(llrs[0] > 0.0, "expected positive LLR for bit near 0, got {}", llrs[0]);

        // Received near -1 → bit 1 (negative LLR)
        let llrs = decoder.decode_symbol(Complex64::new(-0.9, 0.0));
        assert!(llrs[0] < 0.0);
    }

    #[test]
    fn test_qpsk_decode() {
        let decoder = SoftDecoder::new(Modulation::Qpsk, 20.0);
        let s = std::f64::consts::FRAC_1_SQRT_2;
        // Near (s, s) → bits [0, 0]
        let llrs = decoder.decode_symbol(Complex64::new(s * 0.9, s * 0.9));
        assert_eq!(llrs.len(), 2);
        assert!(llrs[0] > 0.0); // bit 0 = 0
        assert!(llrs[1] > 0.0); // bit 1 = 0
    }

    #[test]
    fn test_hard_decode_bpsk() {
        let decoder = SoftDecoder::new(Modulation::Bpsk, 10.0);
        let bits = decoder.hard_decode(Complex64::new(0.5, 0.1));
        assert_eq!(bits, vec![false]); // Nearest to +1 (bit 0)

        let bits = decoder.hard_decode(Complex64::new(-0.5, -0.1));
        assert_eq!(bits, vec![true]); // Nearest to -1 (bit 1)
    }

    #[test]
    fn test_hard_decode_qpsk() {
        let decoder = SoftDecoder::new(Modulation::Qpsk, 10.0);
        let bits = decoder.hard_decode(Complex64::new(0.5, 0.5));
        assert_eq!(bits, vec![false, false]); // Nearest to (s,s) = 00
    }

    #[test]
    fn test_block_decode() {
        let decoder = SoftDecoder::new(Modulation::Bpsk, 20.0);
        let symbols = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
            Complex64::new(0.8, 0.0),
        ];
        let llrs = decoder.decode(&symbols);
        assert_eq!(llrs.len(), 3);
        assert!(llrs[0] > 0.0); // +1 → bit 0
        assert!(llrs[1] < 0.0); // -1 → bit 1
        assert!(llrs[2] > 0.0); // +0.8 → bit 0
    }

    #[test]
    fn test_llr_magnitude_vs_snr() {
        // Higher SNR should produce larger LLR magnitudes
        let high_snr = SoftDecoder::new(Modulation::Bpsk, 20.0);
        let low_snr = SoftDecoder::new(Modulation::Bpsk, 5.0);
        let rx = Complex64::new(0.9, 0.0);
        let llr_high = high_snr.decode_symbol(rx)[0];
        let llr_low = low_snr.decode_symbol(rx)[0];
        assert!(llr_high.abs() > llr_low.abs());
    }

    #[test]
    fn test_llr_to_bits() {
        let bits = SoftDecoder::llr_to_bits(&[1.0, -0.5, 2.0, -3.0]);
        assert_eq!(bits, vec![false, true, false, true]);
    }

    #[test]
    fn test_modulation_bps() {
        assert_eq!(Modulation::Bpsk.bits_per_symbol(), 1);
        assert_eq!(Modulation::Qpsk.bits_per_symbol(), 2);
        assert_eq!(Modulation::Psk8.bits_per_symbol(), 3);
        assert_eq!(Modulation::Qam16.bits_per_symbol(), 4);
        assert_eq!(Modulation::Qam64.bits_per_symbol(), 6);
    }

    #[test]
    fn test_constellation_sizes() {
        assert_eq!(Modulation::Bpsk.constellation().len(), 2);
        assert_eq!(Modulation::Qpsk.constellation().len(), 4);
        assert_eq!(Modulation::Psk8.constellation().len(), 8);
        assert_eq!(Modulation::Qam16.constellation().len(), 16);
        assert_eq!(Modulation::Qam64.constellation().len(), 64);
    }

    #[test]
    fn test_qam16_constellation_power() {
        let points = Modulation::Qam16.constellation();
        let avg_power = points.iter().map(|p| p.norm_sqr()).sum::<f64>() / points.len() as f64;
        assert!((avg_power - 1.0).abs() < 0.01, "avg power = {}", avg_power);
    }

    #[test]
    fn test_qam64_constellation_power() {
        let points = Modulation::Qam64.constellation();
        let avg_power = points.iter().map(|p| p.norm_sqr()).sum::<f64>() / points.len() as f64;
        assert!((avg_power - 1.0).abs() < 0.01, "avg power = {}", avg_power);
    }

    #[test]
    fn test_set_snr() {
        let mut decoder = SoftDecoder::new(Modulation::Bpsk, 10.0);
        decoder.set_snr_db(20.0);
        assert_eq!(decoder.bits_per_symbol(), 1);
    }

    #[test]
    fn test_with_noise_var() {
        let decoder = SoftDecoder::with_noise_var(Modulation::Qpsk, 0.01);
        assert_eq!(decoder.bits_per_symbol(), 2);
    }

    #[test]
    fn test_psk8_decode() {
        let decoder = SoftDecoder::new(Modulation::Psk8, 15.0);
        let rx = Complex64::new(1.0, 0.0); // Near 0-degree point
        let llrs = decoder.decode_symbol(rx);
        assert_eq!(llrs.len(), 3);
    }

    #[test]
    fn test_bit_labels_consistency() {
        for modulation in &[Modulation::Bpsk, Modulation::Qpsk, Modulation::Psk8, Modulation::Qam16, Modulation::Qam64] {
            let labels = modulation.bit_labels();
            let constellation = modulation.constellation();
            assert_eq!(labels.len(), constellation.len());
            for label in &labels {
                assert_eq!(label.len(), modulation.bits_per_symbol());
            }
        }
    }
}
