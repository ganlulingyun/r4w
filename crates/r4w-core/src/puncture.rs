//! Puncture and Depuncture for FEC Rate Adaptation
//!
//! Puncturing removes coded bits according to a pattern matrix to achieve
//! higher code rates from a base rate-1/2 convolutional code. Depuncturing
//! restores the punctured positions with erasure values for soft-decision
//! decoding (Viterbi).
//!
//! ## Standard Patterns
//!
//! | Puncture Pattern | Resulting Rate | Standard |
//! |-----------------|---------------|----------|
//! | `[1,1; 1,1]` | 1/2 (no puncture) | base |
//! | `[1,1; 1,0]` | 2/3 | 802.11a/g |
//! | `[1,1,0; 1,0,1]` | 3/4 | 802.11a/g, DVB-S |
//! | `[1,1,0,1,0,0; 1,0,1,0,1,1]` | 7/8 | DVB-S |
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::puncture::{Puncturer, Depuncturer};
//!
//! // Rate 3/4 puncture pattern (2 rows for rate-1/2 base)
//! let pattern = vec![
//!     vec![true, true, false],
//!     vec![true, false, true],
//! ];
//! let puncturer = Puncturer::new(&pattern);
//!
//! let coded = vec![0u8, 1, 0, 1, 1, 0]; // 6 bits from rate-1/2 encoder
//! let punctured = puncturer.puncture(&coded);
//! assert_eq!(punctured.len(), 4); // 6 * (4/6) = 4 bits transmitted
//!
//! let depuncturer = Depuncturer::new(&pattern, 128); // erasure=128 for soft
//! let restored = depuncturer.depuncture(&punctured);
//! assert_eq!(restored.len(), 6); // Back to 6 values
//! ```

/// Puncturer — removes coded bits to increase effective code rate.
#[derive(Debug, Clone)]
pub struct Puncturer {
    /// Flattened puncture pattern (row-major: row0 then row1)
    pattern: Vec<bool>,
    /// Number of rows (= number of coded streams, typically 2)
    num_rows: usize,
    /// Period of the pattern (columns)
    period: usize,
    /// Number of 1s in the pattern (bits kept per period)
    ones_count: usize,
}

impl Puncturer {
    /// Create from a 2D pattern matrix.
    ///
    /// - `pattern`: rows × cols matrix of keep/puncture decisions.
    ///   Row count = number of coded streams (typically 2 for rate-1/2).
    ///   Period = number of columns.
    pub fn new(pattern: &[Vec<bool>]) -> Self {
        assert!(!pattern.is_empty(), "Pattern must have at least 1 row");
        let num_rows = pattern.len();
        let period = pattern[0].len();
        assert!(period > 0, "Pattern period must be > 0");

        let flat: Vec<bool> = pattern.iter().flat_map(|r| r.iter().copied()).collect();
        let ones_count = flat.iter().filter(|&&b| b).count();

        Self {
            pattern: flat,
            num_rows,
            period,
            ones_count,
        }
    }

    /// Create standard rate-2/3 pattern: `[[1,1],[1,0]]`
    pub fn rate_2_3() -> Self {
        Self::new(&[vec![true, true], vec![true, false]])
    }

    /// Create standard rate-3/4 pattern: `[[1,1,0],[1,0,1]]`
    pub fn rate_3_4() -> Self {
        Self::new(&[vec![true, true, false], vec![true, false, true]])
    }

    /// Create standard rate-5/6 pattern
    pub fn rate_5_6() -> Self {
        Self::new(&[
            vec![true, true, false, true, false],
            vec![true, false, true, false, true],
        ])
    }

    /// Create standard rate-7/8 pattern
    pub fn rate_7_8() -> Self {
        Self::new(&[
            vec![true, true, false, true, false, false, true],
            vec![true, false, true, false, true, true, false],
        ])
    }

    /// Puncture coded bits (hard decision: u8 values 0/1).
    pub fn puncture(&self, input: &[u8]) -> Vec<u8> {
        let total_pattern = self.num_rows * self.period;
        let mut output = Vec::with_capacity(input.len());

        for (i, &bit) in input.iter().enumerate() {
            let pattern_idx = i % total_pattern;
            if self.pattern[pattern_idx] {
                output.push(bit);
            }
        }

        output
    }

    /// Puncture soft decision values (f64).
    pub fn puncture_soft(&self, input: &[f64]) -> Vec<f64> {
        let total_pattern = self.num_rows * self.period;
        let mut output = Vec::with_capacity(input.len());

        for (i, &val) in input.iter().enumerate() {
            let pattern_idx = i % total_pattern;
            if self.pattern[pattern_idx] {
                output.push(val);
            }
        }

        output
    }

    /// Puncture boolean bits.
    pub fn puncture_bits(&self, input: &[bool]) -> Vec<bool> {
        let total_pattern = self.num_rows * self.period;
        let mut output = Vec::with_capacity(input.len());

        for (i, &bit) in input.iter().enumerate() {
            let pattern_idx = i % total_pattern;
            if self.pattern[pattern_idx] {
                output.push(bit);
            }
        }

        output
    }

    /// Get the effective code rate after puncturing.
    /// For a base rate-1/2 code with this pattern, the result is period / ones_count.
    pub fn effective_rate(&self) -> f64 {
        let total = self.num_rows * self.period;
        self.ones_count as f64 / total as f64
    }

    /// Get the pattern period.
    pub fn period(&self) -> usize {
        self.period
    }
}

/// Depuncturer — restores punctured positions with erasure values.
#[derive(Debug, Clone)]
pub struct Depuncturer {
    /// Flattened puncture pattern
    pattern: Vec<bool>,
    /// Number of rows
    num_rows: usize,
    /// Period
    period: usize,
    /// Erasure value for soft-decision (e.g., 0 for LLR, 128 for unsigned 8-bit)
    erasure_value: u8,
}

impl Depuncturer {
    /// Create from pattern and erasure value.
    pub fn new(pattern: &[Vec<bool>], erasure_value: u8) -> Self {
        let num_rows = pattern.len();
        let period = pattern[0].len();
        let flat: Vec<bool> = pattern.iter().flat_map(|r| r.iter().copied()).collect();

        Self {
            pattern: flat,
            num_rows,
            period,
            erasure_value,
        }
    }

    /// Create rate-2/3 depuncturer.
    pub fn rate_2_3(erasure: u8) -> Self {
        Self::new(&[vec![true, true], vec![true, false]], erasure)
    }

    /// Create rate-3/4 depuncturer.
    pub fn rate_3_4(erasure: u8) -> Self {
        Self::new(&[vec![true, true, false], vec![true, false, true]], erasure)
    }

    /// Depuncture hard decision bits (u8).
    pub fn depuncture(&self, input: &[u8]) -> Vec<u8> {
        let total_pattern = self.num_rows * self.period;
        let ones: usize = self.pattern.iter().filter(|&&b| b).count();
        let output_len = if ones > 0 {
            (input.len() * total_pattern + ones - 1) / ones
        } else {
            0
        };

        let mut output = Vec::with_capacity(output_len);
        let mut input_idx = 0;

        for pattern_idx in 0.. {
            if input_idx >= input.len() && pattern_idx % total_pattern == 0 {
                break;
            }
            let pi = pattern_idx % total_pattern;

            if self.pattern[pi] {
                if input_idx < input.len() {
                    output.push(input[input_idx]);
                    input_idx += 1;
                } else {
                    break;
                }
            } else {
                output.push(self.erasure_value);
            }
        }

        output
    }

    /// Depuncture soft decision values (f64), using a float erasure value.
    pub fn depuncture_soft(&self, input: &[f64], erasure: f64) -> Vec<f64> {
        let total_pattern = self.num_rows * self.period;
        let ones: usize = self.pattern.iter().filter(|&&b| b).count();
        let output_len = if ones > 0 {
            (input.len() * total_pattern + ones - 1) / ones
        } else {
            0
        };

        let mut output = Vec::with_capacity(output_len);
        let mut input_idx = 0;

        for pattern_idx in 0.. {
            if input_idx >= input.len() && pattern_idx % total_pattern == 0 {
                break;
            }
            let pi = pattern_idx % total_pattern;

            if self.pattern[pi] {
                if input_idx < input.len() {
                    output.push(input[input_idx]);
                    input_idx += 1;
                } else {
                    break;
                }
            } else {
                output.push(erasure);
            }
        }

        output
    }

    /// Get the erasure value.
    pub fn erasure_value(&self) -> u8 {
        self.erasure_value
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_rate_2_3_puncture() {
        let p = Puncturer::rate_2_3();
        // Pattern: [1,1,1,0] → keeps 3 of 4
        let input = vec![0, 1, 0, 1, 1, 0, 1, 0];
        let output = p.puncture(&input);
        assert_eq!(output.len(), 6); // 8 * 3/4 = 6
    }

    #[test]
    fn test_rate_3_4_puncture() {
        let p = Puncturer::rate_3_4();
        // Pattern: [1,1,0,1,0,1] → keeps 4 of 6
        let input = vec![0, 1, 0, 1, 1, 0];
        let output = p.puncture(&input);
        assert_eq!(output.len(), 4); // 6 * 4/6 = 4
    }

    #[test]
    fn test_roundtrip_hard() {
        let pattern = vec![vec![true, true, false], vec![true, false, true]];
        let p = Puncturer::new(&pattern);
        let d = Depuncturer::new(&pattern, 255);

        let input = vec![1, 0, 1, 1, 0, 0];
        let punctured = p.puncture(&input);
        assert_eq!(punctured.len(), 4);

        let depunctured = d.depuncture(&punctured);
        assert_eq!(depunctured.len(), 6);

        // Kept positions should match
        let total = 6;
        let flat_pattern: Vec<bool> = pattern.iter().flat_map(|r| r.iter().copied()).collect();
        for i in 0..total {
            if flat_pattern[i] {
                assert_eq!(depunctured[i], input[i], "Mismatch at position {}", i);
            } else {
                assert_eq!(depunctured[i], 255, "Erasure expected at position {}", i);
            }
        }
    }

    #[test]
    fn test_roundtrip_soft() {
        let pattern = vec![vec![true, true], vec![true, false]];
        let p = Puncturer::new(&pattern);
        let d = Depuncturer::new(&pattern, 0);

        let input = vec![0.9, -0.5, 0.3, -0.8];
        let punctured = p.puncture_soft(&input);
        assert_eq!(punctured.len(), 3);

        let depunctured = d.depuncture_soft(&punctured, 0.0);
        assert_eq!(depunctured.len(), 4);

        // Position 3 (index 3) was punctured
        assert!((depunctured[0] - 0.9).abs() < 1e-10);
        assert!((depunctured[1] + 0.5).abs() < 1e-10);
        assert!((depunctured[2] - 0.3).abs() < 1e-10);
        assert!((depunctured[3] - 0.0).abs() < 1e-10); // erasure
    }

    #[test]
    fn test_puncture_bits() {
        let p = Puncturer::rate_3_4();
        let input = vec![true, false, true, false, true, false];
        let output = p.puncture_bits(&input);
        assert_eq!(output.len(), 4);
    }

    #[test]
    fn test_effective_rate() {
        let p2_3 = Puncturer::rate_2_3();
        assert!((p2_3.effective_rate() - 0.75).abs() < 0.01); // 3/4 of bits kept

        let p3_4 = Puncturer::rate_3_4();
        assert!((p3_4.effective_rate() - (4.0 / 6.0)).abs() < 0.01);
    }

    #[test]
    fn test_no_puncture() {
        // All-ones pattern = no puncturing
        let p = Puncturer::new(&[vec![true, true], vec![true, true]]);
        let input = vec![0, 1, 0, 1];
        let output = p.puncture(&input);
        assert_eq!(output, input);
    }

    #[test]
    fn test_rate_7_8() {
        let p = Puncturer::rate_7_8();
        // 14-element pattern (2 rows x 7 cols) with 8 ones
        assert_eq!(p.period(), 7);
        let input = vec![0u8; 14];
        let output = p.puncture(&input);
        assert_eq!(output.len(), 8);
    }

    #[test]
    fn test_multiple_periods() {
        let p = Puncturer::rate_2_3();
        let d = Depuncturer::rate_2_3(0);

        // 3 full periods = 12 input bits
        let input = vec![1u8; 12];
        let punctured = p.puncture(&input);
        assert_eq!(punctured.len(), 9); // 12 * 3/4 = 9

        let depunctured = d.depuncture(&punctured);
        assert_eq!(depunctured.len(), 12);
    }

    #[test]
    fn test_empty_input() {
        let p = Puncturer::rate_3_4();
        assert!(p.puncture(&[]).is_empty());

        let d = Depuncturer::rate_3_4(0);
        assert!(d.depuncture(&[]).is_empty());
    }
}
