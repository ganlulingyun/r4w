//! Constellation Demapper — Soft bit decision from received symbols
//!
//! Maps received constellation points to soft bit values (LLRs)
//! for arbitrary constellations. Computes log-likelihood ratios
//! using minimum distance or exact max-log-MAP algorithms.
//! Essential input for LDPC and turbo decoders.
//! GNU Radio equivalent: `constellation_soft_decoder_cf`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::constellation_demapper::{ConstellationDemapper, soft_demap_bpsk};
//! use num_complex::Complex64;
//!
//! // BPSK soft demapping
//! let llrs = soft_demap_bpsk(&[Complex64::new(0.9, 0.1), Complex64::new(-1.1, -0.2)], 1.0);
//! assert!(llrs[0] > 0.0); // Likely bit 0
//! assert!(llrs[1] < 0.0); // Likely bit 1
//! ```

use num_complex::Complex64;

/// Soft-demap BPSK: LLR = 2 * Re(x) / sigma²
///
/// Positive LLR → bit 0 more likely, negative → bit 1 more likely.
pub fn soft_demap_bpsk(symbols: &[Complex64], noise_var: f64) -> Vec<f64> {
    let scale = 2.0 / noise_var.max(1e-30);
    symbols.iter().map(|s| s.re * scale).collect()
}

/// Soft-demap QPSK: returns 2 LLRs per symbol (I then Q).
///
/// Bit 0 from I, bit 1 from Q.
pub fn soft_demap_qpsk(symbols: &[Complex64], noise_var: f64) -> Vec<f64> {
    let scale = 2.0 * std::f64::consts::SQRT_2 / noise_var.max(1e-30);
    let mut llrs = Vec::with_capacity(symbols.len() * 2);
    for s in symbols {
        llrs.push(s.re * scale);
        llrs.push(s.im * scale);
    }
    llrs
}

/// Generic constellation demapper using max-log-MAP.
#[derive(Debug, Clone)]
pub struct ConstellationDemapper {
    /// Constellation points.
    points: Vec<Complex64>,
    /// Bit mapping for each point (bits_per_symbol bits per entry).
    bit_map: Vec<Vec<bool>>,
    /// Bits per symbol.
    bits_per_symbol: usize,
    /// Noise variance (sigma²).
    noise_var: f64,
}

impl ConstellationDemapper {
    /// Create a demapper with constellation points and bit mapping.
    ///
    /// - `points`: constellation points in signal space
    /// - `bit_map`: for each point, the corresponding bit pattern
    /// - `noise_var`: noise variance (sigma²)
    pub fn new(points: &[Complex64], bit_map: &[Vec<bool>], noise_var: f64) -> Self {
        let bits_per_symbol = if bit_map.is_empty() {
            0
        } else {
            bit_map[0].len()
        };
        Self {
            points: points.to_vec(),
            bit_map: bit_map.to_vec(),
            bits_per_symbol,
            noise_var: noise_var.max(1e-30),
        }
    }

    /// Create a BPSK demapper.
    pub fn bpsk(noise_var: f64) -> Self {
        Self::new(
            &[Complex64::new(1.0, 0.0), Complex64::new(-1.0, 0.0)],
            &[vec![false], vec![true]],
            noise_var,
        )
    }

    /// Create a QPSK demapper (Gray coded).
    pub fn qpsk(noise_var: f64) -> Self {
        let s = 1.0 / std::f64::consts::SQRT_2;
        Self::new(
            &[
                Complex64::new(s, s),
                Complex64::new(-s, s),
                Complex64::new(s, -s),
                Complex64::new(-s, -s),
            ],
            &[
                vec![false, false],
                vec![true, false],
                vec![false, true],
                vec![true, true],
            ],
            noise_var,
        )
    }

    /// Demap a single received symbol to soft bits (LLRs).
    ///
    /// Returns `bits_per_symbol` LLR values.
    /// Positive = bit 0 more likely, negative = bit 1 more likely.
    pub fn demap(&self, symbol: Complex64) -> Vec<f64> {
        if self.points.is_empty() || self.bits_per_symbol == 0 {
            return Vec::new();
        }

        let mut llrs = vec![0.0; self.bits_per_symbol];
        let scale = 1.0 / self.noise_var;

        for bit_idx in 0..self.bits_per_symbol {
            let mut min_dist_0 = f64::INFINITY; // Min distance for bit=0
            let mut min_dist_1 = f64::INFINITY; // Min distance for bit=1

            for (point_idx, point) in self.points.iter().enumerate() {
                let dist = (symbol - point).norm_sqr() * scale;
                if self.bit_map[point_idx][bit_idx] {
                    min_dist_1 = min_dist_1.min(dist);
                } else {
                    min_dist_0 = min_dist_0.min(dist);
                }
            }

            // LLR = min_dist_1 - min_dist_0
            llrs[bit_idx] = min_dist_1 - min_dist_0;
        }

        llrs
    }

    /// Demap a block of symbols.
    pub fn demap_block(&self, symbols: &[Complex64]) -> Vec<f64> {
        let mut llrs = Vec::with_capacity(symbols.len() * self.bits_per_symbol);
        for &s in symbols {
            llrs.extend(self.demap(s));
        }
        llrs
    }

    /// Hard-demap: find the closest constellation point.
    pub fn hard_demap(&self, symbol: Complex64) -> Vec<bool> {
        if self.points.is_empty() {
            return Vec::new();
        }
        let (closest_idx, _) = self
            .points
            .iter()
            .enumerate()
            .min_by(|(_, a), (_, b)| {
                let da = (symbol - **a).norm_sqr();
                let db = (symbol - **b).norm_sqr();
                da.partial_cmp(&db).unwrap()
            })
            .unwrap();
        self.bit_map[closest_idx].clone()
    }

    /// Hard-demap a block of symbols.
    pub fn hard_demap_block(&self, symbols: &[Complex64]) -> Vec<bool> {
        let mut bits = Vec::with_capacity(symbols.len() * self.bits_per_symbol);
        for &s in symbols {
            bits.extend(self.hard_demap(s));
        }
        bits
    }

    /// Set noise variance.
    pub fn set_noise_var(&mut self, noise_var: f64) {
        self.noise_var = noise_var.max(1e-30);
    }

    /// Get bits per symbol.
    pub fn bits_per_symbol(&self) -> usize {
        self.bits_per_symbol
    }

    /// Get constellation size.
    pub fn constellation_size(&self) -> usize {
        self.points.len()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bpsk_soft() {
        let llrs = soft_demap_bpsk(
            &[Complex64::new(0.9, 0.0), Complex64::new(-1.1, 0.0)],
            1.0,
        );
        assert!(llrs[0] > 0.0); // Close to +1 → bit 0
        assert!(llrs[1] < 0.0); // Close to -1 → bit 1
    }

    #[test]
    fn test_qpsk_soft() {
        let llrs = soft_demap_qpsk(&[Complex64::new(0.7, 0.7)], 1.0);
        assert_eq!(llrs.len(), 2);
        assert!(llrs[0] > 0.0); // I > 0 → bit 0 = 0
        assert!(llrs[1] > 0.0); // Q > 0 → bit 1 = 0
    }

    #[test]
    fn test_demapper_bpsk() {
        let dm = ConstellationDemapper::bpsk(0.5);
        let llrs = dm.demap(Complex64::new(0.8, 0.1));
        assert_eq!(llrs.len(), 1);
        assert!(llrs[0] > 0.0); // Closer to +1
    }

    #[test]
    fn test_demapper_qpsk() {
        let dm = ConstellationDemapper::qpsk(0.5);
        assert_eq!(dm.bits_per_symbol(), 2);
        assert_eq!(dm.constellation_size(), 4);
        let llrs = dm.demap(Complex64::new(0.6, 0.6));
        assert_eq!(llrs.len(), 2);
    }

    #[test]
    fn test_hard_demap() {
        let dm = ConstellationDemapper::bpsk(1.0);
        let bits = dm.hard_demap(Complex64::new(0.5, 0.0));
        assert_eq!(bits, vec![false]); // Closest to +1 → bit 0
        let bits = dm.hard_demap(Complex64::new(-0.3, 0.0));
        assert_eq!(bits, vec![true]); // Closest to -1 → bit 1
    }

    #[test]
    fn test_hard_demap_block() {
        let dm = ConstellationDemapper::bpsk(1.0);
        let symbols = vec![
            Complex64::new(0.9, 0.0),
            Complex64::new(-0.8, 0.0),
            Complex64::new(1.2, 0.0),
        ];
        let bits = dm.hard_demap_block(&symbols);
        assert_eq!(bits, vec![false, true, false]);
    }

    #[test]
    fn test_demap_block() {
        let dm = ConstellationDemapper::bpsk(1.0);
        let symbols = vec![
            Complex64::new(0.9, 0.0),
            Complex64::new(-0.8, 0.0),
        ];
        let llrs = dm.demap_block(&symbols);
        assert_eq!(llrs.len(), 2);
        assert!(llrs[0] > 0.0);
        assert!(llrs[1] < 0.0);
    }

    #[test]
    fn test_noise_scaling() {
        let dm_low = ConstellationDemapper::bpsk(0.1);
        let dm_high = ConstellationDemapper::bpsk(10.0);
        let s = Complex64::new(0.5, 0.0);
        let llr_low = dm_low.demap(s)[0];
        let llr_high = dm_high.demap(s)[0];
        // Lower noise → stronger LLR
        assert!(llr_low.abs() > llr_high.abs());
    }

    #[test]
    fn test_set_noise_var() {
        let mut dm = ConstellationDemapper::bpsk(1.0);
        dm.set_noise_var(0.5);
        let llrs = dm.demap(Complex64::new(0.9, 0.0));
        assert!(llrs[0] > 0.0);
    }

    #[test]
    fn test_empty_constellation() {
        let dm = ConstellationDemapper::new(&[], &[], 1.0);
        assert!(dm.demap(Complex64::new(1.0, 0.0)).is_empty());
        assert!(dm.hard_demap(Complex64::new(1.0, 0.0)).is_empty());
    }

    #[test]
    fn test_llr_sign_consistency() {
        // For BPSK, LLR sign should match the closest point's bit value
        let dm = ConstellationDemapper::bpsk(1.0);
        // Test points at various positions
        for x in [-2.0, -1.0, -0.5, 0.5, 1.0, 2.0] {
            let llrs = dm.demap(Complex64::new(x, 0.0));
            let hard = dm.hard_demap(Complex64::new(x, 0.0));
            if hard[0] {
                assert!(llrs[0] < 0.0, "bit=1 should have negative LLR");
            } else {
                assert!(llrs[0] > 0.0, "bit=0 should have positive LLR");
            }
        }
    }
}
