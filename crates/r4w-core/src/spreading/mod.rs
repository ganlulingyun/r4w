//! Spreading Codes for Spread Spectrum Systems
//!
//! This module provides pseudo-noise (PN) sequence generators used in
//! spread spectrum communications, particularly for DSSS (Direct Sequence
//! Spread Spectrum) systems.
//!
//! ## Sequence Types
//!
//! - **M-Sequences**: Maximum-length sequences from single LFSR
//! - **Gold Codes**: Preferred for CDMA due to bounded cross-correlation
//! - **Kasami Codes**: Even better cross-correlation for large user sets
//! - **Barker Codes**: Short codes with excellent autocorrelation
//!
//! ## Properties
//!
//! ```text
//! ┌────────────────────────────────────────────────────────────────┐
//! │                     PN Sequence Properties                     │
//! ├──────────────┬───────────────┬───────────────┬─────────────────┤
//! │ Code Type    │ Length        │ # Sequences   │ Cross-Corr      │
//! ├──────────────┼───────────────┼───────────────┼─────────────────┤
//! │ M-Sequence   │ 2^n - 1       │ φ(2^n-1)/n    │ Poor (-1, 2^k-1)│
//! │ Gold         │ 2^n - 1       │ 2^n + 1       │ Bounded (≤t(n)) │
//! │ Kasami       │ 2^n - 1       │ 2^(n/2)       │ Better          │
//! │ Barker       │ 2,3,4,5,7,11,13│ 1 each       │ Excellent       │
//! └──────────────┴───────────────┴───────────────┴─────────────────┘
//! ```
//!
//! ## LPD/LPI Considerations
//!
//! For Low Probability of Detection/Intercept:
//! - Longer sequences = higher processing gain
//! - Gold codes preferred for multi-user (good cross-correlation)
//! - Processing gain (dB) = 10 * log10(sequence_length)

pub mod barker;
pub mod gold;
pub mod lfsr;

pub use barker::{BarkerCode, BARKER_CODES};
pub use gold::GoldCodeGenerator;
pub use lfsr::{Lfsr, MSequence};

/// Common trait for all PN sequence generators
pub trait PnSequence: std::fmt::Debug + Send + Sync {
    /// Generate the next chip (+1 or -1)
    fn next_chip(&mut self) -> i8;

    /// Reset the sequence to its initial state
    fn reset(&mut self);

    /// Get the sequence length (period)
    fn length(&self) -> usize;

    /// Generate the full sequence as a vector
    fn generate_sequence(&mut self) -> Vec<i8> {
        self.reset();
        (0..self.length()).map(|_| self.next_chip()).collect()
    }

    /// Calculate processing gain in dB
    fn processing_gain_db(&self) -> f64 {
        10.0 * (self.length() as f64).log10()
    }
}

/// Calculate autocorrelation of a sequence at a given lag
pub fn autocorrelation(sequence: &[i8], lag: usize) -> i32 {
    let n = sequence.len();
    sequence
        .iter()
        .enumerate()
        .map(|(i, &chip)| chip as i32 * sequence[(i + lag) % n] as i32)
        .sum()
}

/// Calculate cross-correlation between two sequences at a given lag
pub fn cross_correlation(seq_a: &[i8], seq_b: &[i8], lag: usize) -> i32 {
    assert_eq!(seq_a.len(), seq_b.len(), "Sequences must have same length");
    let n = seq_a.len();
    seq_a
        .iter()
        .enumerate()
        .map(|(i, &chip)| chip as i32 * seq_b[(i + lag) % n] as i32)
        .sum()
}

/// Calculate the maximum absolute cross-correlation between two sequences
pub fn max_cross_correlation(seq_a: &[i8], seq_b: &[i8]) -> i32 {
    let n = seq_a.len();
    (0..n)
        .map(|lag| cross_correlation(seq_a, seq_b, lag).abs())
        .max()
        .unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_autocorrelation_peak() {
        let mut gen = MSequence::from_degree(5); // Use degree 5 for faster test
        let seq = gen.generate_sequence();

        // Autocorrelation at lag 0 should be sequence length
        assert_eq!(autocorrelation(&seq, 0), seq.len() as i32);

        // For m-sequences, off-peak autocorrelation is -1
        assert_eq!(autocorrelation(&seq, 1), -1);
    }

    #[test]
    fn test_processing_gain() {
        let mseq = MSequence::from_degree(7);
        let pg = mseq.processing_gain_db();
        // 127 chips = 21.03 dB
        assert!((pg - 21.03).abs() < 0.1);
    }
}
