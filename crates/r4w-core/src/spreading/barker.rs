//! Barker Codes
//!
//! Barker codes are short binary sequences with optimal autocorrelation
//! properties: the off-peak autocorrelation is at most 1 (in absolute value).
//!
//! ## Properties
//!
//! - Peak autocorrelation: N (sequence length)
//! - Off-peak autocorrelation: |R(k)| ≤ 1 for k ≠ 0
//! - Only lengths 2, 3, 4, 5, 7, 11, 13 exist
//!
//! ## Usage
//!
//! Barker codes are ideal for:
//! - Preamble/synchronization sequences
//! - Radar pulse compression
//! - Frame synchronization
//!
//! They are less suitable for spreading (too short for significant processing gain).
//!
//! ## Example
//!
//! ```rust,ignore
//! use r4w_core::spreading::{BarkerCode, BARKER_CODES};
//!
//! // Get Barker-13 (13 dB processing gain)
//! let barker13 = BarkerCode::new(13).unwrap();
//! let sequence = barker13.sequence();
//!
//! assert_eq!(sequence.len(), 13);
//! ```

use super::PnSequence;

/// Known Barker codes (only these lengths exist)
pub const BARKER_CODES: &[(usize, &[i8])] = &[
    (2, &[1, -1]),
    (3, &[1, 1, -1]),
    (4, &[1, 1, -1, 1]),    // Also: [1, 1, 1, -1]
    (5, &[1, 1, 1, -1, 1]),
    (7, &[1, 1, 1, -1, -1, 1, -1]),
    (11, &[1, 1, 1, -1, -1, -1, 1, -1, -1, 1, -1]),
    (13, &[1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1]),
];

/// Barker code sequence
#[derive(Debug, Clone)]
pub struct BarkerCode {
    /// The Barker sequence
    sequence: Vec<i8>,
    /// Current position for iteration
    position: usize,
}

impl BarkerCode {
    /// Create a Barker code of the specified length
    ///
    /// Returns None if the length is not a valid Barker code length.
    pub fn new(length: usize) -> Option<Self> {
        BARKER_CODES
            .iter()
            .find(|(len, _)| *len == length)
            .map(|(_, seq)| Self {
                sequence: seq.to_vec(),
                position: 0,
            })
    }

    /// Create Barker-13 (the longest Barker code)
    pub fn barker13() -> Self {
        Self::new(13).unwrap()
    }

    /// Create Barker-11
    pub fn barker11() -> Self {
        Self::new(11).unwrap()
    }

    /// Create Barker-7
    pub fn barker7() -> Self {
        Self::new(7).unwrap()
    }

    /// Get the sequence as a slice
    pub fn sequence(&self) -> &[i8] {
        &self.sequence
    }

    /// Get all available Barker code lengths
    pub fn available_lengths() -> Vec<usize> {
        BARKER_CODES.iter().map(|(len, _)| *len).collect()
    }

    /// Calculate the sidelobe level (should be 1/N for Barker codes)
    pub fn sidelobe_level_db(&self) -> f64 {
        // Peak sidelobe ratio = 1/N
        -10.0 * (self.sequence.len() as f64).log10()
    }
}

impl PnSequence for BarkerCode {
    fn next_chip(&mut self) -> i8 {
        let chip = self.sequence[self.position];
        self.position = (self.position + 1) % self.sequence.len();
        chip
    }

    fn reset(&mut self) {
        self.position = 0;
    }

    fn length(&self) -> usize {
        self.sequence.len()
    }
}

/// Correlate received signal with Barker code (matched filter)
///
/// Returns correlation values for each starting position.
pub fn barker_correlate(signal: &[f64], barker: &BarkerCode) -> Vec<f64> {
    let code = barker.sequence();
    let code_len = code.len();

    if signal.len() < code_len {
        return vec![];
    }

    (0..=signal.len() - code_len)
        .map(|start| {
            signal[start..start + code_len]
                .iter()
                .zip(code.iter())
                .map(|(s, &c)| s * c as f64)
                .sum()
        })
        .collect()
}

/// Find correlation peak (for synchronization)
pub fn find_barker_peak(signal: &[f64], barker: &BarkerCode) -> Option<(usize, f64)> {
    let correlations = barker_correlate(signal, barker);

    correlations
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(idx, &val)| (idx, val))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spreading::autocorrelation;

    #[test]
    fn test_barker_lengths() {
        assert!(BarkerCode::new(2).is_some());
        assert!(BarkerCode::new(7).is_some());
        assert!(BarkerCode::new(13).is_some());
        assert!(BarkerCode::new(6).is_none());
        assert!(BarkerCode::new(14).is_none());
    }

    #[test]
    fn test_barker13_sequence() {
        let barker = BarkerCode::barker13();
        assert_eq!(barker.sequence().len(), 13);
        assert_eq!(barker.sequence(), &[1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1]);
    }

    #[test]
    fn test_barker_autocorrelation() {
        let barker = BarkerCode::barker13();
        let seq = barker.sequence();

        // Peak autocorrelation should equal length
        assert_eq!(autocorrelation(seq, 0), 13);

        // Off-peak should be |R(k)| <= 1
        for lag in 1..seq.len() {
            let r = autocorrelation(seq, lag).abs();
            assert!(r <= 1, "Off-peak autocorrelation {} at lag {}", r, lag);
        }
    }

    #[test]
    fn test_barker_correlation_detection() {
        let barker = BarkerCode::barker13();

        // Create a signal with embedded Barker code
        let mut signal: Vec<f64> = vec![0.1; 50];
        for (i, &chip) in barker.sequence().iter().enumerate() {
            signal[20 + i] = chip as f64;
        }

        // Find the peak
        let (peak_idx, peak_val) = find_barker_peak(&signal, &barker).unwrap();

        assert_eq!(peak_idx, 20);
        assert!((peak_val - 13.0).abs() < 0.01);
    }

    #[test]
    fn test_sidelobe_level() {
        let barker = BarkerCode::barker13();
        let sll = barker.sidelobe_level_db();

        // 10*log10(1/13) ≈ -11.1 dB
        assert!((sll + 11.14).abs() < 0.1);
    }

    #[test]
    fn test_pn_sequence_trait() {
        let mut barker = BarkerCode::barker7();

        // Generate full sequence
        let seq = barker.generate_sequence();
        assert_eq!(seq, &[1, 1, 1, -1, -1, 1, -1]);

        // Processing gain should be ~8.45 dB for length 7
        let pg = barker.processing_gain_db();
        assert!((pg - 8.45).abs() < 0.1);
    }
}
