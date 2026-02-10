//! # Symbol Demapper
//!
//! Maps received complex symbols to the nearest constellation point
//! and outputs hard or soft bit decisions. Supports common modulations
//! (BPSK, QPSK, 8PSK, 16QAM, 64QAM) with configurable constellation.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::symbol_demapper::{SymbolDemapper, Modulation};
//!
//! let demapper = SymbolDemapper::new(Modulation::Qpsk);
//! let symbols = vec![(0.8, 0.9), (-0.7, 0.8), (-0.9, -1.1), (1.0, -0.8)];
//! let bits = demapper.demap_hard(&symbols);
//! assert_eq!(bits.len(), 8); // QPSK: 2 bits per symbol.
//! ```

use std::f64::consts::FRAC_1_SQRT_2;

/// Supported modulation schemes.
#[derive(Debug, Clone, Copy, PartialEq)]
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
            Self::Bpsk => 1,
            Self::Qpsk => 2,
            Self::Psk8 => 3,
            Self::Qam16 => 4,
            Self::Qam64 => 6,
        }
    }
}

/// Symbol demapper with configurable constellation.
#[derive(Debug, Clone)]
pub struct SymbolDemapper {
    constellation: Vec<(f64, f64)>,
    bit_mapping: Vec<Vec<bool>>,
    bits_per_symbol: usize,
}

impl SymbolDemapper {
    /// Create a demapper for a standard modulation.
    pub fn new(modulation: Modulation) -> Self {
        let (constellation, bit_mapping) = match modulation {
            Modulation::Bpsk => bpsk_constellation(),
            Modulation::Qpsk => qpsk_constellation(),
            Modulation::Psk8 => psk8_constellation(),
            Modulation::Qam16 => qam16_constellation(),
            Modulation::Qam64 => qam64_constellation(),
        };
        Self {
            bits_per_symbol: modulation.bits_per_symbol(),
            constellation,
            bit_mapping,
        }
    }

    /// Create from custom constellation and bit mapping.
    pub fn custom(
        constellation: Vec<(f64, f64)>,
        bit_mapping: Vec<Vec<bool>>,
        bits_per_symbol: usize,
    ) -> Self {
        Self {
            constellation,
            bit_mapping,
            bits_per_symbol,
        }
    }

    /// Hard demap: find nearest constellation point, output bits.
    pub fn demap_hard(&self, symbols: &[(f64, f64)]) -> Vec<bool> {
        let mut bits = Vec::with_capacity(symbols.len() * self.bits_per_symbol);
        for &(re, im) in symbols {
            let idx = self.nearest_point(re, im);
            if idx < self.bit_mapping.len() {
                bits.extend_from_slice(&self.bit_mapping[idx]);
            }
        }
        bits
    }

    /// Soft demap: output log-likelihood ratios (LLRs) per bit.
    ///
    /// Positive LLR → bit is more likely 0, negative → more likely 1.
    pub fn demap_soft(&self, symbols: &[(f64, f64)], noise_var: f64) -> Vec<f64> {
        let sigma2 = noise_var.max(1e-30);
        let mut llrs = Vec::with_capacity(symbols.len() * self.bits_per_symbol);

        for &(re, im) in symbols {
            for bit_idx in 0..self.bits_per_symbol {
                let mut max_0 = f64::NEG_INFINITY;
                let mut max_1 = f64::NEG_INFINITY;

                for (i, &(cr, ci)) in self.constellation.iter().enumerate() {
                    let dist2 = (re - cr) * (re - cr) + (im - ci) * (im - ci);
                    let metric = -dist2 / sigma2;

                    if i < self.bit_mapping.len()
                        && bit_idx < self.bit_mapping[i].len()
                    {
                        if !self.bit_mapping[i][bit_idx] {
                            if metric > max_0 {
                                max_0 = metric;
                            }
                        } else if metric > max_1 {
                            max_1 = metric;
                        }
                    }
                }

                llrs.push(max_0 - max_1);
            }
        }
        llrs
    }

    /// Find nearest constellation point index.
    fn nearest_point(&self, re: f64, im: f64) -> usize {
        let mut best_idx = 0;
        let mut best_dist = f64::MAX;
        for (i, &(cr, ci)) in self.constellation.iter().enumerate() {
            let dist = (re - cr) * (re - cr) + (im - ci) * (im - ci);
            if dist < best_dist {
                best_dist = dist;
                best_idx = i;
            }
        }
        best_idx
    }

    /// Get constellation points.
    pub fn constellation(&self) -> &[(f64, f64)] {
        &self.constellation
    }

    /// Get bits per symbol.
    pub fn bits_per_symbol(&self) -> usize {
        self.bits_per_symbol
    }
}

fn bpsk_constellation() -> (Vec<(f64, f64)>, Vec<Vec<bool>>) {
    (
        vec![(-1.0, 0.0), (1.0, 0.0)],
        vec![vec![true], vec![false]],
    )
}

fn qpsk_constellation() -> (Vec<(f64, f64)>, Vec<Vec<bool>>) {
    let s = FRAC_1_SQRT_2;
    (
        vec![(s, s), (-s, s), (-s, -s), (s, -s)],
        vec![
            vec![false, false],
            vec![false, true],
            vec![true, true],
            vec![true, false],
        ],
    )
}

fn psk8_constellation() -> (Vec<(f64, f64)>, Vec<Vec<bool>>) {
    let mut points = Vec::with_capacity(8);
    let mut bits = Vec::with_capacity(8);
    for i in 0..8u8 {
        let angle = std::f64::consts::PI / 4.0 * i as f64;
        points.push((angle.cos(), angle.sin()));
        bits.push(vec![
            (i >> 2) & 1 == 1,
            (i >> 1) & 1 == 1,
            i & 1 == 1,
        ]);
    }
    (points, bits)
}

fn qam16_constellation() -> (Vec<(f64, f64)>, Vec<Vec<bool>>) {
    let levels = [-3.0, -1.0, 1.0, 3.0];
    let norm = 1.0 / (10.0_f64).sqrt();
    let mut points = Vec::with_capacity(16);
    let mut bits = Vec::with_capacity(16);
    for (i, &re) in levels.iter().enumerate() {
        for (j, &im) in levels.iter().enumerate() {
            points.push((re * norm, im * norm));
            bits.push(vec![
                (i >> 1) & 1 == 1,
                i & 1 == 1,
                (j >> 1) & 1 == 1,
                j & 1 == 1,
            ]);
        }
    }
    (points, bits)
}

fn qam64_constellation() -> (Vec<(f64, f64)>, Vec<Vec<bool>>) {
    let levels = [-7.0, -5.0, -3.0, -1.0, 1.0, 3.0, 5.0, 7.0];
    let norm = 1.0 / (42.0_f64).sqrt();
    let mut points = Vec::with_capacity(64);
    let mut bits = Vec::with_capacity(64);
    for (i, &re) in levels.iter().enumerate() {
        for (j, &im) in levels.iter().enumerate() {
            points.push((re * norm, im * norm));
            bits.push(vec![
                (i >> 2) & 1 == 1,
                (i >> 1) & 1 == 1,
                i & 1 == 1,
                (j >> 2) & 1 == 1,
                (j >> 1) & 1 == 1,
                j & 1 == 1,
            ]);
        }
    }
    (points, bits)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bpsk_demap() {
        let dm = SymbolDemapper::new(Modulation::Bpsk);
        let symbols = vec![(1.0, 0.0), (-1.0, 0.0), (0.5, 0.1), (-0.8, 0.0)];
        let bits = dm.demap_hard(&symbols);
        assert_eq!(bits.len(), 4);
        assert_eq!(bits[0], false); // +1 → 0
        assert_eq!(bits[1], true);  // -1 → 1
    }

    #[test]
    fn test_qpsk_demap() {
        let dm = SymbolDemapper::new(Modulation::Qpsk);
        let s = FRAC_1_SQRT_2;
        let symbols = vec![(s, s), (-s, s), (-s, -s), (s, -s)];
        let bits = dm.demap_hard(&symbols);
        assert_eq!(bits.len(), 8);
    }

    #[test]
    fn test_qam16_constellation_size() {
        let dm = SymbolDemapper::new(Modulation::Qam16);
        assert_eq!(dm.constellation().len(), 16);
        assert_eq!(dm.bits_per_symbol(), 4);
    }

    #[test]
    fn test_qam64_constellation_size() {
        let dm = SymbolDemapper::new(Modulation::Qam64);
        assert_eq!(dm.constellation().len(), 64);
        assert_eq!(dm.bits_per_symbol(), 6);
    }

    #[test]
    fn test_psk8() {
        let dm = SymbolDemapper::new(Modulation::Psk8);
        assert_eq!(dm.constellation().len(), 8);
        assert_eq!(dm.bits_per_symbol(), 3);
    }

    #[test]
    fn test_soft_demap() {
        let dm = SymbolDemapper::new(Modulation::Bpsk);
        let symbols = vec![(0.9, 0.0), (-0.9, 0.0)];
        let llrs = dm.demap_soft(&symbols, 0.1);
        assert_eq!(llrs.len(), 2);
        assert!(llrs[0] > 0.0);  // More likely bit 0.
        assert!(llrs[1] < 0.0);  // More likely bit 1.
    }

    #[test]
    fn test_soft_demap_noise() {
        let dm = SymbolDemapper::new(Modulation::Qpsk);
        let s = FRAC_1_SQRT_2;
        let symbols = vec![(s + 0.1, s - 0.1)];
        let llrs = dm.demap_soft(&symbols, 0.5);
        assert_eq!(llrs.len(), 2);
    }

    #[test]
    fn test_nearest_point() {
        let dm = SymbolDemapper::new(Modulation::Qpsk);
        // Close to first point (+s, +s).
        let bits = dm.demap_hard(&[(0.8, 0.8)]);
        assert_eq!(bits.len(), 2);
    }

    #[test]
    fn test_custom_constellation() {
        let points = vec![(0.0, 1.0), (0.0, -1.0)];
        let bits = vec![vec![false], vec![true]];
        let dm = SymbolDemapper::custom(points, bits, 1);
        let result = dm.demap_hard(&[(0.0, 0.9)]);
        assert_eq!(result, vec![false]);
    }

    #[test]
    fn test_bits_per_symbol() {
        assert_eq!(Modulation::Bpsk.bits_per_symbol(), 1);
        assert_eq!(Modulation::Qpsk.bits_per_symbol(), 2);
        assert_eq!(Modulation::Psk8.bits_per_symbol(), 3);
        assert_eq!(Modulation::Qam16.bits_per_symbol(), 4);
        assert_eq!(Modulation::Qam64.bits_per_symbol(), 6);
    }
}
