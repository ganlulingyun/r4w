//! Symbol Mapper / Demapper
//!
//! Maps bits to constellation points and demaps received symbols back to bits.
//! Supports both hard-decision (nearest point) and soft-decision (LLR) output
//! for turbo, LDPC, and iterative decoding.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::symbol_mapping::{SymbolMapper, Modulation};
//!
//! let mapper = SymbolMapper::new(Modulation::Qpsk);
//!
//! // Map bits to QPSK symbols
//! let bits = vec![0, 1, 1, 0, 1, 1, 0, 0];
//! let symbols = mapper.map(&bits);
//! assert_eq!(symbols.len(), 4); // 2 bits per QPSK symbol
//!
//! // Hard demap back to bits
//! let recovered = mapper.demap_hard(&symbols);
//! assert_eq!(bits, recovered);
//! ```

use num_complex::Complex64;

/// Modulation scheme.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Modulation {
    /// BPSK: 1 bit/symbol, {-1, +1}
    Bpsk,
    /// QPSK: 2 bits/symbol, Gray-coded
    Qpsk,
    /// 8-PSK: 3 bits/symbol, Gray-coded
    Psk8,
    /// 16-QAM: 4 bits/symbol, Gray-coded
    Qam16,
    /// 64-QAM: 6 bits/symbol, Gray-coded
    Qam64,
}

impl Modulation {
    /// Bits per symbol for this modulation.
    pub fn bits_per_symbol(&self) -> usize {
        match self {
            Modulation::Bpsk => 1,
            Modulation::Qpsk => 2,
            Modulation::Psk8 => 3,
            Modulation::Qam16 => 4,
            Modulation::Qam64 => 6,
        }
    }

    /// Number of constellation points.
    pub fn order(&self) -> usize {
        1 << self.bits_per_symbol()
    }

    /// Spectral efficiency in bits/s/Hz.
    pub fn spectral_efficiency(&self) -> f64 {
        self.bits_per_symbol() as f64
    }
}

/// Symbol mapper/demapper.
#[derive(Debug, Clone)]
pub struct SymbolMapper {
    modulation: Modulation,
    /// Constellation points indexed by Gray-coded symbol value.
    constellation: Vec<Complex64>,
    /// Average constellation energy (for normalization).
    avg_energy: f64,
}

impl SymbolMapper {
    /// Create a new symbol mapper for the given modulation.
    pub fn new(modulation: Modulation) -> Self {
        let constellation = match modulation {
            Modulation::Bpsk => Self::bpsk_constellation(),
            Modulation::Qpsk => Self::qpsk_constellation(),
            Modulation::Psk8 => Self::psk8_constellation(),
            Modulation::Qam16 => Self::qam16_constellation(),
            Modulation::Qam64 => Self::qam64_constellation(),
        };
        let avg_energy =
            constellation.iter().map(|s| s.norm_sqr()).sum::<f64>() / constellation.len() as f64;
        Self {
            modulation,
            constellation,
            avg_energy,
        }
    }

    /// Map bit sequence to constellation symbols.
    ///
    /// Bits are consumed in groups of `bits_per_symbol`. The bit sequence
    /// length should be a multiple of bits_per_symbol; remaining bits are
    /// zero-padded.
    pub fn map(&self, bits: &[u8]) -> Vec<Complex64> {
        let bps = self.modulation.bits_per_symbol();
        bits.chunks(bps)
            .map(|chunk| {
                let mut symbol_idx = 0usize;
                for (i, &b) in chunk.iter().enumerate() {
                    if b != 0 {
                        symbol_idx |= 1 << i;
                    }
                }
                self.constellation[symbol_idx % self.constellation.len()]
            })
            .collect()
    }

    /// Hard-decision demapping: find nearest constellation point.
    pub fn demap_hard(&self, symbols: &[Complex64]) -> Vec<u8> {
        let bps = self.modulation.bits_per_symbol();
        let mut bits = Vec::with_capacity(symbols.len() * bps);

        for s in symbols {
            let (idx, _) = self
                .constellation
                .iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    (s - *a)
                        .norm_sqr()
                        .partial_cmp(&(s - *b).norm_sqr())
                        .unwrap()
                })
                .unwrap();

            for i in 0..bps {
                bits.push(((idx >> i) & 1) as u8);
            }
        }

        bits
    }

    /// Soft-decision demapping: compute Log-Likelihood Ratios (LLRs).
    ///
    /// Returns one LLR per bit. Positive LLR = more likely bit 0,
    /// negative = more likely bit 1.
    ///
    /// ```text
    /// LLR(b_k) = ln( sum_{s: b_k=0} exp(-|r-s|^2/N0) / sum_{s: b_k=1} exp(-|r-s|^2/N0) )
    /// ```
    ///
    /// Approximated using max-log-MAP:
    /// ```text
    /// LLR(b_k) ≈ min_{s: b_k=1}(|r-s|^2) - min_{s: b_k=0}(|r-s|^2) / N0
    /// ```
    pub fn demap_soft(&self, symbols: &[Complex64], noise_var: f64) -> Vec<f64> {
        let bps = self.modulation.bits_per_symbol();
        let n0 = noise_var.max(1e-20);
        let mut llrs = Vec::with_capacity(symbols.len() * bps);

        for s in symbols {
            for bit_pos in 0..bps {
                let mut min_dist_0 = f64::MAX;
                let mut min_dist_1 = f64::MAX;

                for (idx, c) in self.constellation.iter().enumerate() {
                    let dist = (s - c).norm_sqr();
                    if (idx >> bit_pos) & 1 == 0 {
                        min_dist_0 = min_dist_0.min(dist);
                    } else {
                        min_dist_1 = min_dist_1.min(dist);
                    }
                }

                // Max-log-MAP LLR: positive means bit 0 more likely
                let llr = (min_dist_1 - min_dist_0) / n0;
                llrs.push(llr);
            }
        }

        llrs
    }

    /// Get the constellation points.
    pub fn constellation(&self) -> &[Complex64] {
        &self.constellation
    }

    /// Get the modulation type.
    pub fn modulation(&self) -> Modulation {
        self.modulation
    }

    /// Average constellation energy.
    pub fn avg_energy(&self) -> f64 {
        self.avg_energy
    }

    // Constellation generators (Gray-coded)

    fn bpsk_constellation() -> Vec<Complex64> {
        vec![
            Complex64::new(-1.0, 0.0), // bit 0
            Complex64::new(1.0, 0.0),  // bit 1
        ]
    }

    fn qpsk_constellation() -> Vec<Complex64> {
        let v = std::f64::consts::FRAC_1_SQRT_2;
        vec![
            Complex64::new(v, v),   // 00
            Complex64::new(-v, v),  // 01
            Complex64::new(v, -v),  // 10
            Complex64::new(-v, -v), // 11 (Gray: adjacent points differ by 1 bit)
        ]
    }

    fn psk8_constellation() -> Vec<Complex64> {
        // Gray coding for 8PSK: 000,001,011,010,110,111,101,100
        let gray_order = [0, 1, 3, 2, 6, 7, 5, 4];
        let mut constellation = vec![Complex64::new(0.0, 0.0); 8];
        for (i, &gray_idx) in gray_order.iter().enumerate() {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / 8.0;
            constellation[gray_idx] = Complex64::new(angle.cos(), angle.sin());
        }
        constellation
    }

    fn qam16_constellation() -> Vec<Complex64> {
        // 16-QAM: 4x4 grid, Gray-coded per axis
        // Gray code for 2 bits: 00=0, 01=1, 11=2, 10=3 → values: -3, -1, +1, +3
        let gray_vals = [-3.0, -1.0, 1.0, 3.0]; // maps Gray {00,01,11,10} → positions
        let gray_map = [0usize, 1, 3, 2]; // Gray code order

        let mut constellation = vec![Complex64::new(0.0, 0.0); 16];
        let norm = 1.0 / (10.0f64).sqrt(); // normalize average power to 1

        for i_bits in 0..4usize {
            for q_bits in 0..4usize {
                let symbol_idx = i_bits | (q_bits << 2);
                let i_val = gray_vals[gray_map[i_bits]] * norm;
                let q_val = gray_vals[gray_map[q_bits]] * norm;
                constellation[symbol_idx] = Complex64::new(i_val, q_val);
            }
        }
        constellation
    }

    fn qam64_constellation() -> Vec<Complex64> {
        // 64-QAM: 8x8 grid, Gray-coded per axis
        let gray_vals = [-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0];
        let gray_map = [0usize, 1, 3, 2, 6, 7, 5, 4]; // 3-bit Gray code

        let mut constellation = vec![Complex64::new(0.0, 0.0); 64];
        let norm = 1.0 / (42.0f64).sqrt(); // normalize average power to 1

        for i_bits in 0..8usize {
            for q_bits in 0..8usize {
                let symbol_idx = i_bits | (q_bits << 3);
                let i_val = gray_vals[gray_map[i_bits]] * norm;
                let q_val = gray_vals[gray_map[q_bits]] * norm;
                constellation[symbol_idx] = Complex64::new(i_val, q_val);
            }
        }
        constellation
    }
}

/// Compute BER from transmitted and received bit sequences.
pub fn compute_ber(tx_bits: &[u8], rx_bits: &[u8]) -> f64 {
    let errors = tx_bits
        .iter()
        .zip(rx_bits.iter())
        .filter(|(&t, &r)| t != r)
        .count();
    errors as f64 / tx_bits.len().max(1) as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bpsk_roundtrip() {
        let mapper = SymbolMapper::new(Modulation::Bpsk);
        let bits = vec![0, 1, 1, 0, 1, 0, 0, 1];
        let symbols = mapper.map(&bits);
        assert_eq!(symbols.len(), 8);
        let recovered = mapper.demap_hard(&symbols);
        assert_eq!(bits, recovered);
    }

    #[test]
    fn test_qpsk_roundtrip() {
        let mapper = SymbolMapper::new(Modulation::Qpsk);
        let bits = vec![0, 1, 1, 0, 1, 1, 0, 0];
        let symbols = mapper.map(&bits);
        assert_eq!(symbols.len(), 4);
        let recovered = mapper.demap_hard(&symbols);
        assert_eq!(bits, recovered);
    }

    #[test]
    fn test_8psk_roundtrip() {
        let mapper = SymbolMapper::new(Modulation::Psk8);
        let bits: Vec<u8> = (0..24).map(|i| (i % 2) as u8).collect();
        let symbols = mapper.map(&bits);
        assert_eq!(symbols.len(), 8);
        let recovered = mapper.demap_hard(&symbols);
        assert_eq!(bits, recovered);
    }

    #[test]
    fn test_16qam_roundtrip() {
        let mapper = SymbolMapper::new(Modulation::Qam16);
        // Test all 16 symbols
        for sym_idx in 0..16u8 {
            let bits: Vec<u8> = (0..4).map(|i| (sym_idx >> i) & 1).collect();
            let symbols = mapper.map(&bits);
            assert_eq!(symbols.len(), 1);
            let recovered = mapper.demap_hard(&symbols);
            assert_eq!(bits, recovered, "Failed for symbol {sym_idx}");
        }
    }

    #[test]
    fn test_64qam_roundtrip() {
        let mapper = SymbolMapper::new(Modulation::Qam64);
        // Test a subset of symbols
        for sym_idx in [0u8, 7, 15, 31, 42, 63] {
            let bits: Vec<u8> = (0..6).map(|i| (sym_idx >> i) & 1).collect();
            let symbols = mapper.map(&bits);
            let recovered = mapper.demap_hard(&symbols);
            assert_eq!(bits, recovered, "Failed for symbol {sym_idx}");
        }
    }

    #[test]
    fn test_soft_decision_sign() {
        let mapper = SymbolMapper::new(Modulation::Bpsk);

        // Symbol close to +1 (bit=1): LLR should be negative (favoring bit 1)
        let symbols = vec![Complex64::new(0.9, 0.0)];
        let llrs = mapper.demap_soft(&symbols, 0.1);
        assert!(
            llrs[0] < 0.0,
            "LLR for symbol near +1 should be negative (bit=1): got {:.3}",
            llrs[0]
        );

        // Symbol close to -1 (bit=0): LLR should be positive (favoring bit 0)
        let symbols = vec![Complex64::new(-0.9, 0.0)];
        let llrs = mapper.demap_soft(&symbols, 0.1);
        assert!(
            llrs[0] > 0.0,
            "LLR for symbol near -1 should be positive (bit=0): got {:.3}",
            llrs[0]
        );
    }

    #[test]
    fn test_soft_decision_magnitude() {
        let mapper = SymbolMapper::new(Modulation::Bpsk);

        // Close to decision boundary: LLR should be small
        let boundary = vec![Complex64::new(0.01, 0.0)];
        let llr_boundary = mapper.demap_soft(&boundary, 0.1);

        // Far from boundary: LLR should be large
        let far = vec![Complex64::new(2.0, 0.0)];
        let llr_far = mapper.demap_soft(&far, 0.1);

        assert!(
            llr_far[0].abs() > llr_boundary[0].abs(),
            "LLR magnitude should increase with distance from boundary"
        );
    }

    #[test]
    fn test_constellation_unit_energy() {
        for modulation in [Modulation::Bpsk, Modulation::Qpsk, Modulation::Psk8] {
            let mapper = SymbolMapper::new(modulation);
            assert!(
                (mapper.avg_energy() - 1.0).abs() < 0.01,
                "{modulation:?} should have unit average energy: got {:.3}",
                mapper.avg_energy()
            );
        }
    }

    #[test]
    fn test_qam_normalized_energy() {
        let mapper = SymbolMapper::new(Modulation::Qam16);
        assert!(
            (mapper.avg_energy() - 1.0).abs() < 0.1,
            "16-QAM should be approximately unit energy: got {:.3}",
            mapper.avg_energy()
        );
    }

    #[test]
    fn test_bits_per_symbol() {
        assert_eq!(Modulation::Bpsk.bits_per_symbol(), 1);
        assert_eq!(Modulation::Qpsk.bits_per_symbol(), 2);
        assert_eq!(Modulation::Psk8.bits_per_symbol(), 3);
        assert_eq!(Modulation::Qam16.bits_per_symbol(), 4);
        assert_eq!(Modulation::Qam64.bits_per_symbol(), 6);
    }

    #[test]
    fn test_compute_ber() {
        let tx = vec![0, 1, 0, 1, 0, 1, 0, 1, 0, 1];
        let rx = vec![0, 1, 0, 1, 0, 1, 0, 1, 0, 1];
        assert!((compute_ber(&tx, &rx) - 0.0).abs() < 1e-10);

        let rx_err = vec![1, 0, 0, 1, 0, 1, 0, 1, 0, 1]; // 2 errors
        assert!((compute_ber(&tx, &rx_err) - 0.2).abs() < 1e-10);
    }
}
