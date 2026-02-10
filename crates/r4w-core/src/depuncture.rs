//! Depuncturing Block for FEC Rate Recovery
//!
//! Depuncturing is the inverse of puncturing: it inserts erasure values at
//! positions that were removed during puncturing, reconstructing the original
//! code rate so that a Viterbi or turbo decoder can process the full-length
//! codeword. Punctured positions are marked as erasures (unknown reliability)
//! which the decoder treats as equally likely to be 0 or 1.
//!
//! # How It Works
//!
//! Given a puncture pattern like `[true, true, false, true]` (rate 3/4),
//! the depuncturer walks through the pattern cyclically. At each `true`
//! position it consumes the next received bit/LLR; at each `false` position
//! it inserts an erasure value (`None` for hard decisions, a neutral LLR
//! such as `0.0` for soft decisions).
//!
//! # Example
//!
//! ```rust
//! use r4w_core::depuncture::{depuncture_hard, depuncture_soft, PuncturePattern};
//!
//! // Rate 2/3 pattern: keep, keep, punctured, keep
//! let pattern = PuncturePattern::rate_2_3();
//!
//! // Hard depuncture: 3 received bits expanded to 4
//! let received = vec![true, false, true];
//! let restored = depuncture_hard(&received, pattern.as_slice());
//! assert_eq!(restored, vec![Some(true), Some(false), None, Some(true)]);
//!
//! // Soft depuncture: 3 LLRs expanded to 4 with erasure = 0.0
//! let llrs = vec![2.5, -1.3, 0.8];
//! let restored_soft = depuncture_soft(&llrs, pattern.as_slice(), 0.0);
//! assert_eq!(restored_soft.len(), 4);
//! assert!((restored_soft[2] - 0.0).abs() < 1e-10); // erasure position
//! ```

/// Depuncture hard-decision bits by inserting `None` at punctured positions.
///
/// Walks through `pattern` cyclically. For each `true` in the pattern,
/// the next bit from `bits` is consumed and wrapped in `Some`. For each
/// `false`, `None` is inserted to indicate an erasure.
///
/// # Arguments
///
/// * `bits` - Received (punctured) hard-decision bits.
/// * `pattern` - Puncture pattern where `true` = kept, `false` = punctured.
///
/// # Returns
///
/// A vector of `Option<bool>` with the original code length restored.
pub fn depuncture_hard(bits: &[bool], pattern: &[bool]) -> Vec<Option<bool>> {
    if bits.is_empty() || pattern.is_empty() {
        return Vec::new();
    }

    let ones_count = pattern.iter().filter(|&&b| b).count();
    if ones_count == 0 {
        return Vec::new();
    }

    // Calculate how many full + partial pattern cycles we need
    let full_cycles = bits.len() / ones_count;
    let remainder = bits.len() % ones_count;

    // Determine total output length
    let mut output_len = full_cycles * pattern.len();
    if remainder > 0 {
        // Walk through one more partial cycle to find how many pattern
        // positions are needed to consume the remaining bits
        let mut count = 0;
        for (i, &keep) in pattern.iter().enumerate() {
            if keep {
                count += 1;
            }
            if count == remainder {
                output_len += i + 1;
                break;
            }
        }
    }

    let mut output = Vec::with_capacity(output_len);
    let mut input_idx = 0;

    for i in 0..output_len {
        let pi = i % pattern.len();
        if pattern[pi] {
            if input_idx < bits.len() {
                output.push(Some(bits[input_idx]));
                input_idx += 1;
            }
        } else {
            output.push(None);
        }
    }

    output
}

/// Depuncture soft-decision LLR values by inserting an erasure value at
/// punctured positions.
///
/// For soft-decision decoding (Viterbi, turbo, LDPC), punctured positions
/// are filled with a neutral value -- typically `0.0` for log-likelihood
/// ratios, indicating maximum uncertainty.
///
/// # Arguments
///
/// * `llrs` - Received (punctured) soft-decision values.
/// * `pattern` - Puncture pattern where `true` = kept, `false` = punctured.
/// * `erasure_value` - Value to insert at punctured positions (usually `0.0`).
///
/// # Returns
///
/// A vector of `f64` with the original code length restored.
pub fn depuncture_soft(llrs: &[f64], pattern: &[bool], erasure_value: f64) -> Vec<f64> {
    if llrs.is_empty() || pattern.is_empty() {
        return Vec::new();
    }

    let ones_count = pattern.iter().filter(|&&b| b).count();
    if ones_count == 0 {
        return Vec::new();
    }

    let full_cycles = llrs.len() / ones_count;
    let remainder = llrs.len() % ones_count;

    let mut output_len = full_cycles * pattern.len();
    if remainder > 0 {
        let mut count = 0;
        for (i, &keep) in pattern.iter().enumerate() {
            if keep {
                count += 1;
            }
            if count == remainder {
                output_len += i + 1;
                break;
            }
        }
    }

    let mut output = Vec::with_capacity(output_len);
    let mut input_idx = 0;

    for i in 0..output_len {
        let pi = i % pattern.len();
        if pattern[pi] {
            if input_idx < llrs.len() {
                output.push(llrs[input_idx]);
                input_idx += 1;
            }
        } else {
            output.push(erasure_value);
        }
    }

    output
}

/// Validate that a puncture pattern is usable.
///
/// A valid pattern must be non-empty and contain at least one `true` entry
/// (at least one bit must be kept per period).
pub fn validate_pattern(pattern: &[bool]) -> bool {
    !pattern.is_empty() && pattern.iter().any(|&b| b)
}

/// Compute the effective code rate of a puncture pattern.
///
/// Returns the ratio of kept bits (`true` entries) to the total pattern
/// length. For example, pattern `[1,1,0,1]` has effective rate 3/4 = 0.75.
pub fn effective_rate(pattern: &[bool]) -> f64 {
    if pattern.is_empty() {
        return 0.0;
    }
    let ones = pattern.iter().filter(|&&b| b).count();
    ones as f64 / pattern.len() as f64
}

/// Stateful depuncturing block that stores a pattern and provides
/// convenience methods for processing streams of bits or LLRs.
#[derive(Debug, Clone)]
pub struct DepunctureBlock {
    /// The puncture pattern (true = keep, false = punctured).
    pattern: Vec<bool>,
}

impl DepunctureBlock {
    /// Create a new depuncture block with the given pattern.
    ///
    /// # Panics
    ///
    /// Panics if the pattern is empty or contains no `true` entries.
    pub fn new(pattern: &[bool]) -> Self {
        assert!(
            validate_pattern(pattern),
            "Pattern must be non-empty and contain at least one true entry"
        );
        Self {
            pattern: pattern.to_vec(),
        }
    }

    /// Depuncture hard-decision bits, inserting `None` at punctured positions.
    pub fn process_hard(&self, bits: &[bool]) -> Vec<Option<bool>> {
        depuncture_hard(bits, &self.pattern)
    }

    /// Depuncture soft-decision LLRs, inserting `erasure_value` at punctured positions.
    pub fn process_soft(&self, llrs: &[f64], erasure_value: f64) -> Vec<f64> {
        depuncture_soft(llrs, &self.pattern, erasure_value)
    }

    /// Return a reference to the stored pattern.
    pub fn pattern(&self) -> &[bool] {
        &self.pattern
    }

    /// Compute the effective rate of this block's pattern.
    pub fn effective_rate(&self) -> f64 {
        effective_rate(&self.pattern)
    }
}

/// Common puncture patterns used in standard FEC systems.
///
/// Each pattern is a flat boolean slice where `true` means the bit is kept
/// and `false` means the bit was punctured.
#[derive(Debug, Clone)]
pub struct PuncturePattern {
    pattern: Vec<bool>,
}

impl PuncturePattern {
    /// Rate 1/2 -- no puncturing (all bits kept).
    ///
    /// Pattern: `[1, 1]`
    pub fn rate_1_2() -> Self {
        Self {
            pattern: vec![true, true],
        }
    }

    /// Rate 2/3 -- one bit punctured per 4-bit period.
    ///
    /// Pattern: `[1, 1, 0, 1]`
    ///
    /// Used in IEEE 802.11a/g, DVB.
    pub fn rate_2_3() -> Self {
        Self {
            pattern: vec![true, true, false, true],
        }
    }

    /// Rate 3/4 -- two bits punctured per 6-bit period.
    ///
    /// Pattern: `[1, 1, 0, 1, 1, 0]`
    ///
    /// Used in IEEE 802.11a/g, DVB-S.
    pub fn rate_3_4() -> Self {
        Self {
            pattern: vec![true, true, false, true, true, false],
        }
    }

    /// Rate 5/6 -- two bits punctured per 6-bit period.
    ///
    /// Pattern: `[1, 1, 0, 1, 0, 1, 1, 0, 1, 0]`
    pub fn rate_5_6() -> Self {
        Self {
            pattern: vec![
                true, true, false, true, false, true, true, false, true, false,
            ],
        }
    }

    /// Rate 7/8 -- two bits punctured per 8-bit period.
    ///
    /// Pattern: `[1, 1, 0, 1, 0, 1, 0, 1, 1, 0, 1, 0, 1, 0]`
    pub fn rate_7_8() -> Self {
        Self {
            pattern: vec![
                true, true, false, true, false, true, false, true, true, false, true, false, true,
                false,
            ],
        }
    }

    /// Return the pattern as a slice.
    pub fn as_slice(&self) -> &[bool] {
        &self.pattern
    }

    /// Compute the effective rate of this pattern.
    pub fn effective_rate(&self) -> f64 {
        effective_rate(&self.pattern)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_depuncture_hard_basic() {
        // Pattern [1, 1, 0, 1]: 3 kept, 1 punctured per period
        let pattern = [true, true, false, true];
        let bits = vec![true, false, true];
        let result = depuncture_hard(&bits, &pattern);
        assert_eq!(result.len(), 4);
        assert_eq!(result[0], Some(true));
        assert_eq!(result[1], Some(false));
        assert_eq!(result[2], None); // punctured position
        assert_eq!(result[3], Some(true));
    }

    #[test]
    fn test_depuncture_soft_basic() {
        // Pattern [1, 1, 0, 1]: insert erasure at position 2
        let pattern = [true, true, false, true];
        let llrs = vec![2.5, -1.3, 0.8];
        let result = depuncture_soft(&llrs, &pattern, 0.0);
        assert_eq!(result.len(), 4);
        assert!((result[0] - 2.5).abs() < 1e-10);
        assert!((result[1] - (-1.3)).abs() < 1e-10);
        assert!((result[2] - 0.0).abs() < 1e-10); // erasure
        assert!((result[3] - 0.8).abs() < 1e-10);
    }

    #[test]
    fn test_rate_1_2_passthrough() {
        // Rate 1/2 pattern [1,1] keeps everything -- no puncturing occurred.
        let pattern = PuncturePattern::rate_1_2();
        let bits = vec![true, false, true, true];
        let result = depuncture_hard(&bits, pattern.as_slice());
        assert_eq!(result.len(), 4);
        for (i, &bit) in bits.iter().enumerate() {
            assert_eq!(result[i], Some(bit), "Mismatch at position {}", i);
        }
    }

    #[test]
    fn test_rate_2_3_depuncture() {
        // Rate 2/3: pattern [1,1,0,1], 3 kept per 4
        let pattern = PuncturePattern::rate_2_3();
        let bits = vec![true, false, true, false, true, false];
        let result = depuncture_hard(&bits, pattern.as_slice());
        assert_eq!(result.len(), 8);
        // First period: positions 0,1 from input, 2=None, 3 from input
        assert_eq!(result[0], Some(true));
        assert_eq!(result[1], Some(false));
        assert_eq!(result[2], None);
        assert_eq!(result[3], Some(true));
        // Second period: positions 4,5 from input, 6=None, 7 from input
        assert_eq!(result[4], Some(false));
        assert_eq!(result[5], Some(true));
        assert_eq!(result[6], None);
        assert_eq!(result[7], Some(false));
    }

    #[test]
    fn test_rate_3_4_depuncture() {
        // Rate 3/4: pattern [1,1,0,1,1,0], 4 kept per 6
        let pattern = PuncturePattern::rate_3_4();
        let bits = vec![true, false, true, false];
        let result = depuncture_hard(&bits, pattern.as_slice());
        assert_eq!(result.len(), 6);
        assert_eq!(result[0], Some(true));
        assert_eq!(result[1], Some(false));
        assert_eq!(result[2], None); // punctured
        assert_eq!(result[3], Some(true));
        assert_eq!(result[4], Some(false));
        assert_eq!(result[5], None); // punctured
    }

    #[test]
    fn test_roundtrip_with_puncture() {
        // Simulate puncturing then depuncturing
        let pattern = [true, true, false, true];

        // Original coded bits (before puncturing)
        let original = vec![true, false, true, true];

        // Puncture: keep bits at true positions, skip false positions
        let mut punctured = Vec::new();
        for (i, &bit) in original.iter().enumerate() {
            if pattern[i % pattern.len()] {
                punctured.push(bit);
            }
        }
        assert_eq!(punctured.len(), 3); // 3 of 4 kept

        // Depuncture: restore to original length
        let restored = depuncture_hard(&punctured, &pattern);
        assert_eq!(restored.len(), 4);

        // Verify kept positions match original
        for (i, &orig) in original.iter().enumerate() {
            if pattern[i % pattern.len()] {
                assert_eq!(
                    restored[i],
                    Some(orig),
                    "Kept bit mismatch at position {}",
                    i
                );
            } else {
                assert_eq!(restored[i], None, "Expected erasure at position {}", i);
            }
        }
    }

    #[test]
    fn test_erasure_value_insertion() {
        // Verify that the specified erasure value is actually used
        let pattern = [true, false, true];
        let llrs = vec![3.0, -2.0];

        // With erasure = 0.0
        let result_zero = depuncture_soft(&llrs, &pattern, 0.0);
        assert!((result_zero[1] - 0.0).abs() < 1e-10);

        // With erasure = -99.0 (custom erasure marker)
        let result_custom = depuncture_soft(&llrs, &pattern, -99.0);
        assert!((result_custom[1] - (-99.0)).abs() < 1e-10);

        // With erasure = f64::NAN -- check that it is NaN
        let result_nan = depuncture_soft(&llrs, &pattern, f64::NAN);
        assert!(result_nan[1].is_nan());
    }

    #[test]
    fn test_empty_input() {
        let pattern = [true, true, false, true];

        // Empty bits
        let hard = depuncture_hard(&[], &pattern);
        assert!(hard.is_empty());

        // Empty LLRs
        let soft = depuncture_soft(&[], &pattern, 0.0);
        assert!(soft.is_empty());

        // Empty pattern
        let hard_empty_pat = depuncture_hard(&[true, false], &[]);
        assert!(hard_empty_pat.is_empty());

        let soft_empty_pat = depuncture_soft(&[1.0, -1.0], &[], 0.0);
        assert!(soft_empty_pat.is_empty());
    }

    #[test]
    fn test_validate_pattern() {
        // Valid patterns
        assert!(validate_pattern(&[true]));
        assert!(validate_pattern(&[true, false]));
        assert!(validate_pattern(&[true, true, false, true]));
        assert!(validate_pattern(&[false, false, true]));

        // Invalid patterns
        assert!(!validate_pattern(&[])); // empty
        assert!(!validate_pattern(&[false])); // no true entries
        assert!(!validate_pattern(&[false, false, false])); // all false
    }

    #[test]
    fn test_effective_rate() {
        // Rate 1/2: [1,1] -> 2/2 = 1.0
        assert!((effective_rate(&[true, true]) - 1.0).abs() < 1e-10);

        // Rate 2/3: [1,1,0,1] -> 3/4 = 0.75
        assert!((effective_rate(&[true, true, false, true]) - 0.75).abs() < 1e-10);

        // Rate 3/4: [1,1,0,1,1,0] -> 4/6 = 0.6667
        assert!((effective_rate(&[true, true, false, true, true, false]) - (4.0 / 6.0)).abs() < 1e-10);

        // Empty pattern
        assert!((effective_rate(&[]) - 0.0).abs() < 1e-10);

        // PuncturePattern helper rates
        assert!((PuncturePattern::rate_1_2().effective_rate() - 1.0).abs() < 1e-10);
        assert!((PuncturePattern::rate_2_3().effective_rate() - 0.75).abs() < 1e-10);
        assert!((PuncturePattern::rate_3_4().effective_rate() - (4.0 / 6.0)).abs() < 1e-10);
    }
}
