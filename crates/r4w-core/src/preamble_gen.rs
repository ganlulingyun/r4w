//! Preamble Generator — Standard preamble patterns for packet TX
//!
//! Generates common preamble sequences used in packet-based communications.
//! Includes alternating bits, Barker codes, Zadoff-Chu sequences, and custom
//! patterns. Preambles enable timing synchronization and AGC settling at the
//! receiver.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::preamble_gen::{PreambleGenerator, PreamblePattern};
//!
//! let gen = PreambleGenerator::new(PreamblePattern::Alternating, 32);
//! let preamble = gen.generate_bits();
//! assert_eq!(preamble.len(), 32);
//! assert_eq!(preamble[0], true);
//! assert_eq!(preamble[1], false);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Preamble pattern type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PreamblePattern {
    /// Alternating 1010... — good for clock recovery.
    Alternating,
    /// All ones (111...) — useful for AGC settling.
    AllOnes,
    /// Barker-7 repeated — good autocorrelation (low sidelobes).
    Barker7,
    /// Barker-11 repeated.
    Barker11,
    /// Barker-13 repeated.
    Barker13,
    /// PN sequence (m-sequence) of given degree.
    PnSequence,
    /// Custom repeated pattern from a sync word.
    Custom,
}

/// Preamble generator.
#[derive(Debug, Clone)]
pub struct PreambleGenerator {
    pattern: PreamblePattern,
    length: usize,
    custom_bits: Vec<bool>,
}

impl PreambleGenerator {
    /// Create a preamble generator.
    pub fn new(pattern: PreamblePattern, length: usize) -> Self {
        Self {
            pattern,
            length: length.max(1),
            custom_bits: Vec::new(),
        }
    }

    /// Create with a custom bit pattern that will be repeated to fill length.
    pub fn custom(bits: &[bool], length: usize) -> Self {
        Self {
            pattern: PreamblePattern::Custom,
            length: length.max(1),
            custom_bits: bits.to_vec(),
        }
    }

    /// Generate preamble as bits.
    pub fn generate_bits(&self) -> Vec<bool> {
        let base = match self.pattern {
            PreamblePattern::Alternating => {
                (0..self.length).map(|i| i % 2 == 0).collect()
            }
            PreamblePattern::AllOnes => {
                vec![true; self.length]
            }
            PreamblePattern::Barker7 => {
                let barker = [true, true, true, false, false, true, false];
                barker.iter().cycle().take(self.length).copied().collect()
            }
            PreamblePattern::Barker11 => {
                let barker = [true, true, true, false, false, false, true, false, false, true, false];
                barker.iter().cycle().take(self.length).copied().collect()
            }
            PreamblePattern::Barker13 => {
                let barker = [true, true, true, true, true, false, false, true, true, false, true, false, true];
                barker.iter().cycle().take(self.length).copied().collect()
            }
            PreamblePattern::PnSequence => {
                // Simple LFSR (x^5 + x^2 + 1)
                let mut lfsr: u32 = 0x1F;
                (0..self.length).map(|_| {
                    let bit = (lfsr & 1) != 0;
                    let feedback = ((lfsr >> 0) ^ (lfsr >> 2)) & 1;
                    lfsr = (lfsr >> 1) | (feedback << 4);
                    bit
                }).collect()
            }
            PreamblePattern::Custom => {
                if self.custom_bits.is_empty() {
                    vec![true; self.length]
                } else {
                    self.custom_bits.iter().cycle().take(self.length).copied().collect()
                }
            }
        };
        base
    }

    /// Generate preamble as BPSK symbols (±1.0).
    pub fn generate_bpsk(&self) -> Vec<f64> {
        self.generate_bits().iter()
            .map(|&b| if b { 1.0 } else { -1.0 })
            .collect()
    }

    /// Generate preamble as complex IQ samples (BPSK on I channel).
    pub fn generate_iq(&self) -> Vec<Complex64> {
        self.generate_bpsk().iter()
            .map(|&v| Complex64::new(v, 0.0))
            .collect()
    }

    /// Generate a Zadoff-Chu preamble sequence.
    ///
    /// Zadoff-Chu sequences have ideal autocorrelation (used in LTE PRACH).
    /// `root`: Root index (must be coprime to length).
    /// `length`: Sequence length (should be prime for best properties).
    pub fn zadoff_chu(root: usize, length: usize) -> Vec<Complex64> {
        let n = length;
        let u = root;
        (0..n)
            .map(|k| {
                let phase = -PI * (u as f64) * (k as f64) * (k as f64 + 1.0) / (n as f64);
                Complex64::from_polar(1.0, phase)
            })
            .collect()
    }

    /// Get the pattern type.
    pub fn pattern(&self) -> PreamblePattern {
        self.pattern
    }

    /// Get the preamble length.
    pub fn length(&self) -> usize {
        self.length
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_alternating() {
        let gen = PreambleGenerator::new(PreamblePattern::Alternating, 8);
        let bits = gen.generate_bits();
        assert_eq!(bits, vec![true, false, true, false, true, false, true, false]);
    }

    #[test]
    fn test_all_ones() {
        let gen = PreambleGenerator::new(PreamblePattern::AllOnes, 5);
        let bits = gen.generate_bits();
        assert_eq!(bits, vec![true; 5]);
    }

    #[test]
    fn test_barker7() {
        let gen = PreambleGenerator::new(PreamblePattern::Barker7, 7);
        let bits = gen.generate_bits();
        assert_eq!(bits, vec![true, true, true, false, false, true, false]);
    }

    #[test]
    fn test_barker7_repeated() {
        let gen = PreambleGenerator::new(PreamblePattern::Barker7, 14);
        let bits = gen.generate_bits();
        assert_eq!(bits.len(), 14);
        assert_eq!(&bits[..7], &bits[7..14]); // Should repeat
    }

    #[test]
    fn test_barker13() {
        let gen = PreambleGenerator::new(PreamblePattern::Barker13, 13);
        let bits = gen.generate_bits();
        assert_eq!(bits.len(), 13);
    }

    #[test]
    fn test_pn_sequence() {
        let gen = PreambleGenerator::new(PreamblePattern::PnSequence, 31);
        let bits = gen.generate_bits();
        assert_eq!(bits.len(), 31);
        // Should have roughly equal 1s and 0s
        let ones = bits.iter().filter(|&&b| b).count();
        assert!(ones > 10 && ones < 25, "PN should be ~balanced, got {} ones", ones);
    }

    #[test]
    fn test_custom() {
        let gen = PreambleGenerator::custom(&[true, false, true], 9);
        let bits = gen.generate_bits();
        assert_eq!(bits, vec![true, false, true, true, false, true, true, false, true]);
    }

    #[test]
    fn test_bpsk_output() {
        let gen = PreambleGenerator::new(PreamblePattern::Alternating, 4);
        let bpsk = gen.generate_bpsk();
        assert_eq!(bpsk, vec![1.0, -1.0, 1.0, -1.0]);
    }

    #[test]
    fn test_iq_output() {
        let gen = PreambleGenerator::new(PreamblePattern::AllOnes, 3);
        let iq = gen.generate_iq();
        assert_eq!(iq.len(), 3);
        for s in &iq {
            assert!((s.re - 1.0).abs() < 0.001);
            assert!(s.im.abs() < 0.001);
        }
    }

    #[test]
    fn test_zadoff_chu() {
        let zc = PreambleGenerator::zadoff_chu(1, 31);
        assert_eq!(zc.len(), 31);
        // All samples should have unit magnitude
        for s in &zc {
            assert!((s.norm() - 1.0).abs() < 1e-10, "ZC should have unit magnitude");
        }
    }

    #[test]
    fn test_zadoff_chu_autocorrelation() {
        let zc = PreambleGenerator::zadoff_chu(1, 31);
        // Compute autocorrelation at lag 0 and lag 1
        let r0: f64 = zc.iter().map(|s| s.norm_sqr()).sum();
        let r1: Complex64 = zc.iter().zip(zc[1..].iter())
            .map(|(a, b)| a * b.conj())
            .sum();
        // Autocorrelation at lag 0 >> lag 1
        assert!(r0 > r1.norm() * 3.0, "ZC should have good autocorrelation");
    }
}
