//! Symbol Slicer (Decision Device)
//!
//! Maps received constellation points to the nearest valid symbol using
//! minimum-distance (ML) decision. Supports standard modulation schemes
//! and custom constellations.
//!
//! ## Supported Modulations
//!
//! - BPSK: {-1, +1}
//! - QPSK: {±1 ± j} / √2
//! - 8-PSK: 8 points on unit circle
//! - 16-QAM: 4×4 rectangular grid
//! - 64-QAM: 8×8 rectangular grid
//! - Custom: arbitrary constellation points
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::symbol_slicer::SymbolSlicer;
//! use num_complex::Complex64;
//!
//! let slicer = SymbolSlicer::qpsk();
//! let noisy = Complex64::new(0.8, -0.6);
//! let (index, point) = slicer.slice(noisy);
//! // Returns nearest QPSK point
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Symbol slicer / hard decision device.
#[derive(Debug, Clone)]
pub struct SymbolSlicer {
    /// Constellation points (reference)
    points: Vec<Complex64>,
    /// Bits per symbol (log2 of constellation size)
    bits_per_symbol: usize,
    /// Modulation name
    name: String,
}

impl SymbolSlicer {
    /// Create from a custom set of constellation points.
    pub fn new(points: Vec<Complex64>, name: &str) -> Self {
        let bps = (points.len() as f64).log2().ceil() as usize;
        Self {
            points,
            bits_per_symbol: bps,
            name: name.to_string(),
        }
    }

    /// BPSK slicer: {-1, +1}
    pub fn bpsk() -> Self {
        Self::new(
            vec![Complex64::new(-1.0, 0.0), Complex64::new(1.0, 0.0)],
            "BPSK",
        )
    }

    /// QPSK slicer: {±1 ± j}/√2
    pub fn qpsk() -> Self {
        let s = 1.0 / 2.0_f64.sqrt();
        Self::new(
            vec![
                Complex64::new(s, s),
                Complex64::new(-s, s),
                Complex64::new(-s, -s),
                Complex64::new(s, -s),
            ],
            "QPSK",
        )
    }

    /// 8-PSK slicer
    pub fn psk8() -> Self {
        let points: Vec<Complex64> = (0..8)
            .map(|i| {
                let angle = 2.0 * PI * i as f64 / 8.0;
                Complex64::new(angle.cos(), angle.sin())
            })
            .collect();
        Self::new(points, "8-PSK")
    }

    /// 16-QAM slicer (normalized)
    pub fn qam16() -> Self {
        let vals = [-3.0, -1.0, 1.0, 3.0];
        let norm = 1.0 / (10.0_f64).sqrt(); // √(average power)
        let mut points = Vec::with_capacity(16);
        for &re in &vals {
            for &im in &vals {
                points.push(Complex64::new(re * norm, im * norm));
            }
        }
        Self::new(points, "16-QAM")
    }

    /// 64-QAM slicer (normalized)
    pub fn qam64() -> Self {
        let vals: Vec<f64> = (-7..=7).step_by(2).map(|v| v as f64).collect();
        let norm = 1.0 / (42.0_f64).sqrt();
        let mut points = Vec::with_capacity(64);
        for &re in &vals {
            for &im in &vals {
                points.push(Complex64::new(re * norm, im * norm));
            }
        }
        Self::new(points, "64-QAM")
    }

    /// Slice a single received sample to the nearest constellation point.
    /// Returns (symbol_index, constellation_point).
    pub fn slice(&self, sample: Complex64) -> (usize, Complex64) {
        let mut min_dist = f64::MAX;
        let mut best_idx = 0;

        for (i, &point) in self.points.iter().enumerate() {
            let dist = (sample - point).norm_sqr();
            if dist < min_dist {
                min_dist = dist;
                best_idx = i;
            }
        }

        (best_idx, self.points[best_idx])
    }

    /// Slice a block of samples, returning symbol indices.
    pub fn slice_indices(&self, input: &[Complex64]) -> Vec<usize> {
        input.iter().map(|&s| self.slice(s).0).collect()
    }

    /// Slice a block of samples, returning nearest constellation points.
    pub fn slice_points(&self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.slice(s).1).collect()
    }

    /// Slice and convert to bit string (Gray-coded index → bits).
    pub fn slice_to_bits(&self, input: &[Complex64]) -> Vec<bool> {
        let mut bits = Vec::with_capacity(input.len() * self.bits_per_symbol);
        for &sample in input {
            let (idx, _) = self.slice(sample);
            for bit_pos in (0..self.bits_per_symbol).rev() {
                bits.push((idx >> bit_pos) & 1 != 0);
            }
        }
        bits
    }

    /// Compute error vector magnitude for a block of samples.
    pub fn evm(&self, input: &[Complex64]) -> f64 {
        if input.is_empty() {
            return 0.0;
        }
        let sum_sq: f64 = input
            .iter()
            .map(|&s| {
                let (_, point) = self.slice(s);
                (s - point).norm_sqr()
            })
            .sum();
        (sum_sq / input.len() as f64).sqrt()
    }

    /// Get constellation points.
    pub fn points(&self) -> &[Complex64] {
        &self.points
    }

    /// Get bits per symbol.
    pub fn bits_per_symbol(&self) -> usize {
        self.bits_per_symbol
    }

    /// Get constellation size.
    pub fn constellation_size(&self) -> usize {
        self.points.len()
    }

    /// Get modulation name.
    pub fn name(&self) -> &str {
        &self.name
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bpsk_slice() {
        let slicer = SymbolSlicer::bpsk();
        assert_eq!(slicer.constellation_size(), 2);
        assert_eq!(slicer.bits_per_symbol(), 1);

        let (idx, _) = slicer.slice(Complex64::new(0.8, 0.1));
        assert_eq!(idx, 1); // Nearest to +1

        let (idx, _) = slicer.slice(Complex64::new(-0.5, -0.2));
        assert_eq!(idx, 0); // Nearest to -1
    }

    #[test]
    fn test_qpsk_slice() {
        let slicer = SymbolSlicer::qpsk();
        assert_eq!(slicer.constellation_size(), 4);
        assert_eq!(slicer.bits_per_symbol(), 2);

        // Point near (+s, +s)
        let (idx, pt) = slicer.slice(Complex64::new(0.5, 0.6));
        assert_eq!(idx, 0);
        assert!(pt.re > 0.0 && pt.im > 0.0);

        // Point near (-s, -s)
        let (idx, pt) = slicer.slice(Complex64::new(-0.8, -0.3));
        assert_eq!(idx, 2);
        assert!(pt.re < 0.0 && pt.im < 0.0);
    }

    #[test]
    fn test_8psk_slice() {
        let slicer = SymbolSlicer::psk8();
        assert_eq!(slicer.constellation_size(), 8);
        assert_eq!(slicer.bits_per_symbol(), 3);

        // Point on the positive real axis → index 0
        let (idx, _) = slicer.slice(Complex64::new(0.9, 0.1));
        assert_eq!(idx, 0);
    }

    #[test]
    fn test_16qam_slice() {
        let slicer = SymbolSlicer::qam16();
        assert_eq!(slicer.constellation_size(), 16);
        assert_eq!(slicer.bits_per_symbol(), 4);

        // Slice a known point
        let (_, pt) = slicer.slice(Complex64::new(0.95, 0.95));
        assert!(pt.re > 0.0 && pt.im > 0.0);
    }

    #[test]
    fn test_64qam() {
        let slicer = SymbolSlicer::qam64();
        assert_eq!(slicer.constellation_size(), 64);
        assert_eq!(slicer.bits_per_symbol(), 6);
    }

    #[test]
    fn test_exact_point_recovery() {
        let slicer = SymbolSlicer::qpsk();
        for &point in slicer.points() {
            let (_, recovered) = slicer.slice(point);
            assert!((point - recovered).norm() < 1e-10);
        }
    }

    #[test]
    fn test_slice_to_bits() {
        let slicer = SymbolSlicer::bpsk();
        let input = vec![
            Complex64::new(0.9, 0.0),  // → index 1 → bit true
            Complex64::new(-0.9, 0.0), // → index 0 → bit false
        ];
        let bits = slicer.slice_to_bits(&input);
        assert_eq!(bits, vec![true, false]);
    }

    #[test]
    fn test_evm_perfect() {
        let slicer = SymbolSlicer::qpsk();
        let perfect: Vec<Complex64> = slicer.points().to_vec();
        let evm = slicer.evm(&perfect);
        assert!(evm < 1e-10, "EVM should be ~0 for perfect symbols, got {}", evm);
    }

    #[test]
    fn test_evm_noisy() {
        let slicer = SymbolSlicer::qpsk();
        let noisy: Vec<Complex64> = slicer
            .points()
            .iter()
            .map(|&p| p + Complex64::new(0.1, 0.1))
            .collect();
        let evm = slicer.evm(&noisy);
        assert!(evm > 0.05, "EVM should be > 0 for noisy symbols, got {}", evm);
    }

    #[test]
    fn test_custom_constellation() {
        let points = vec![
            Complex64::new(0.0, 1.0),
            Complex64::new(0.0, -1.0),
        ];
        let slicer = SymbolSlicer::new(points, "Custom");
        assert_eq!(slicer.name(), "Custom");

        let (idx, _) = slicer.slice(Complex64::new(0.1, 0.8));
        assert_eq!(idx, 0);
    }
}
