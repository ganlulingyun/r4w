//! Rate Matching for Channel Coding
//!
//! Adjusts the length of a coded bit stream to match the number of available
//! transmission resources. When the coded output is longer than the allocation,
//! bits are **punctured** (evenly removed). When the coded output is shorter,
//! bits are **repeated** (circular extension). An LTE-style sub-block
//! interleaver is also provided for standards-compliant rate matching.
//!
//! Rate matching sits between the channel encoder and the modulator in a
//! typical transmit chain, and its inverse (`unmatch_rate`) sits between the
//! demodulator and the channel decoder on the receive side.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::rate_matcher::{RateMatcher, RateMatchMode};
//!
//! // Puncture: reduce 10 coded bits down to 7 transmission slots
//! let rm = RateMatcher::new(7, RateMatchMode::Puncture);
//! let coded = vec![1.0; 10];
//! let matched = rm.match_rate(&coded);
//! assert_eq!(matched.len(), 7);
//!
//! // Recover original length on the receive side
//! let recovered = rm.unmatch_rate(&matched, 10);
//! assert_eq!(recovered.len(), 10);
//! ```

/// Operating mode for rate matching.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RateMatchMode {
    /// Remove evenly-spaced bits when the coded stream is too long.
    Puncture,
    /// Circularly repeat bits when the coded stream is too short.
    Repeat,
    /// Apply sub-block interleaving before bit selection (LTE-style).
    Interleave,
}

/// General-purpose rate matcher.
///
/// Adjusts an input soft-bit vector to a fixed `output_length` by puncturing
/// (if too long), repeating (if too short), or passing through (if equal).
#[derive(Debug, Clone)]
pub struct RateMatcher {
    /// Desired output length after rate matching.
    pub output_length: usize,
    /// Operating mode.
    pub mode: RateMatchMode,
}

impl RateMatcher {
    /// Create a new rate matcher.
    ///
    /// * `output_length` - target number of soft bits after matching.
    /// * `mode` - puncture, repeat, or interleave strategy.
    pub fn new(output_length: usize, mode: RateMatchMode) -> Self {
        Self {
            output_length,
            mode,
        }
    }

    /// Adjust `input` to `self.output_length` samples.
    ///
    /// - If `input.len() > output_length`: evenly-spaced puncturing.
    /// - If `input.len() < output_length`: circular repetition.
    /// - If `input.len() == output_length`: passthrough.
    pub fn match_rate(&self, input: &[f64]) -> Vec<f64> {
        let n = input.len();
        let m = self.output_length;

        if n == 0 || m == 0 {
            return vec![0.0; m];
        }

        if n == m {
            // Passthrough
            return input.to_vec();
        }

        if n > m {
            // Puncture: keep m of n values at evenly-spaced positions
            let pattern = compute_puncture_pattern(n, m);
            let mut output = Vec::with_capacity(m);
            for (i, &keep) in pattern.iter().enumerate() {
                if keep {
                    output.push(input[i]);
                }
            }
            output
        } else {
            // Repeat: circular extension
            let mut output = Vec::with_capacity(m);
            for i in 0..m {
                output.push(input[i % n]);
            }
            output
        }
    }

    /// Reverse rate matching to recover the original length.
    ///
    /// * `input` - the rate-matched soft bits (length == `self.output_length`).
    /// * `original_length` - length of the coded stream before rate matching.
    ///
    /// For punctured streams, zeros are inserted at the punctured positions
    /// (erasures for soft-decision decoding).  For repeated streams, repeated
    /// copies are averaged to combine the soft information.
    pub fn unmatch_rate(&self, input: &[f64], original_length: usize) -> Vec<f64> {
        let m = input.len();

        if original_length == 0 {
            return Vec::new();
        }
        if m == 0 {
            return vec![0.0; original_length];
        }

        if original_length == m {
            // No rate matching was applied
            return input.to_vec();
        }

        if original_length > m {
            // Was punctured: insert zeros (erasures) at punctured positions
            let pattern = compute_puncture_pattern(original_length, m);
            let mut output = Vec::with_capacity(original_length);
            let mut input_idx = 0;
            for &keep in &pattern {
                if keep {
                    if input_idx < m {
                        output.push(input[input_idx]);
                        input_idx += 1;
                    } else {
                        output.push(0.0);
                    }
                } else {
                    output.push(0.0); // erasure
                }
            }
            output
        } else {
            // Was repeated: average the repeated copies
            let mut output = vec![0.0; original_length];
            let mut counts = vec![0usize; original_length];
            for (i, &val) in input.iter().enumerate() {
                let idx = i % original_length;
                output[idx] += val;
                counts[idx] += 1;
            }
            for i in 0..original_length {
                if counts[i] > 0 {
                    output[i] /= counts[i] as f64;
                }
            }
            output
        }
    }
}

/// Circular buffer for rate matching.
///
/// Supports writing data that wraps around, then reading an arbitrary length
/// with wrap-around semantics.
#[derive(Debug, Clone)]
pub struct CircularBuffer {
    /// Internal storage.
    buffer: Vec<f64>,
    /// Allocated buffer size.
    buffer_size: usize,
}

impl CircularBuffer {
    /// Create a circular buffer of the given size, initialised to zero.
    pub fn new(buffer_size: usize) -> Self {
        Self {
            buffer: vec![0.0; buffer_size],
            buffer_size,
        }
    }

    /// Write `data` into the buffer with circular wrap-around.
    ///
    /// If `data` is longer than `buffer_size`, the buffer contains the last
    /// `buffer_size` elements; if shorter, only the first `data.len()`
    /// positions are overwritten.
    pub fn fill(&mut self, data: &[f64]) {
        if self.buffer_size == 0 {
            return;
        }
        for (i, &val) in data.iter().enumerate() {
            self.buffer[i % self.buffer_size] = val;
        }
    }

    /// Read `length` samples from the buffer with circular wrap-around.
    pub fn read(&self, length: usize) -> Vec<f64> {
        if self.buffer_size == 0 {
            return vec![0.0; length];
        }
        let mut output = Vec::with_capacity(length);
        for i in 0..length {
            output.push(self.buffer[i % self.buffer_size]);
        }
        output
    }

    /// Current buffer size.
    pub fn len(&self) -> usize {
        self.buffer_size
    }

    /// Returns `true` if the buffer size is zero.
    pub fn is_empty(&self) -> bool {
        self.buffer_size == 0
    }
}

/// LTE-style rate matcher with sub-block interleaving and bit selection.
///
/// Implements the sub-block interleaver permutation from 3GPP TS 36.212
/// (simplified). The interleaver uses a column permutation pattern to spread
/// coded bits across the transmission, improving resilience to bursty errors.
#[derive(Debug, Clone)]
pub struct LteRateMatcher {
    /// Number of filler bits inserted before interleaving (set to zero in the
    /// output after de-interleaving).
    pub num_filler_bits: usize,
}

/// Column permutation pattern from 3GPP TS 36.212 Table 5.1.4-1.
const LTE_COLUMN_PERM: [usize; 32] = [
    0, 16, 8, 24, 4, 20, 12, 28, 2, 18, 10, 26, 6, 22, 14, 30, 1, 17, 9, 25, 5, 21, 13, 29, 3,
    19, 11, 27, 7, 23, 15, 31,
];

impl LteRateMatcher {
    /// Create an LTE rate matcher.
    ///
    /// * `num_filler_bits` - number of `<NULL>` filler bits at the beginning of
    ///   the coded block (depends on turbo-code block size).
    pub fn new(num_filler_bits: usize) -> Self {
        Self { num_filler_bits }
    }

    /// Apply sub-block interleaving.
    ///
    /// The input is arranged row-by-row into a matrix with 32 columns, columns
    /// are permuted according to the LTE permutation table, and the output is
    /// read column-by-column.
    pub fn interleave(&self, input: &[f64]) -> Vec<f64> {
        let n = input.len();
        if n == 0 {
            return Vec::new();
        }

        let num_cols = 32;
        let num_rows = (n + num_cols - 1) / num_cols;
        let total = num_rows * num_cols;

        // Pad input to fill the matrix (filler = 0.0)
        let mut padded = vec![0.0; total];
        padded[..n].copy_from_slice(input);

        // Write row-by-row, read column-by-column with column permutation
        let mut output = Vec::with_capacity(total);
        for &col in &LTE_COLUMN_PERM {
            for row in 0..num_rows {
                let idx = row * num_cols + col;
                output.push(padded[idx]);
            }
        }

        // Trim to original length (remove interleaved padding)
        output.truncate(n);
        output
    }

    /// Reverse the sub-block interleaving (de-interleave).
    pub fn deinterleave(&self, input: &[f64]) -> Vec<f64> {
        let n = input.len();
        if n == 0 {
            return Vec::new();
        }

        let num_cols = 32;
        let num_rows = (n + num_cols - 1) / num_cols;
        let total = num_rows * num_cols;

        // Build the forward permutation index map
        let mut perm_indices = Vec::with_capacity(total);
        for &col in &LTE_COLUMN_PERM {
            for row in 0..num_rows {
                perm_indices.push(row * num_cols + col);
            }
        }

        // Place input values at their original positions
        let mut output = vec![0.0; total];
        for (out_pos, &orig_pos) in perm_indices.iter().enumerate() {
            if out_pos < n {
                output[orig_pos] = input[out_pos];
            }
        }

        output.truncate(n);
        output
    }
}

/// Compute a puncture pattern for reducing `input_len` to `output_len`.
///
/// Returns a boolean vector of length `input_len` where `true` means keep
/// and `false` means puncture. Punctured positions are distributed as evenly
/// as possible across the input.
///
/// # Panics
///
/// Panics if `output_len > input_len`.
pub fn compute_puncture_pattern(input_len: usize, output_len: usize) -> Vec<bool> {
    assert!(
        output_len <= input_len,
        "output_len ({}) must be <= input_len ({})",
        output_len,
        input_len
    );

    if input_len == 0 {
        return Vec::new();
    }

    if output_len == input_len {
        return vec![true; input_len];
    }

    if output_len == 0 {
        return vec![false; input_len];
    }

    // Use Bresenham-style even distribution to select which positions to keep
    let mut pattern = vec![false; input_len];
    let mut acc: i64 = 0;
    let mut kept = 0usize;

    for i in 0..input_len {
        acc += output_len as i64;
        if acc > 0 {
            // Map: we keep this position when the accumulated count crosses a threshold
            // Bresenham: keep when (i+1)*output_len / input_len > kept
            if ((i + 1) * output_len + input_len - 1) / input_len > kept {
                pattern[i] = true;
                kept += 1;
            }
        }
    }

    // Safety: ensure exactly output_len bits are kept.
    // The Bresenham approach above should produce exactly output_len trues,
    // but let's be defensive.
    let actual_kept = pattern.iter().filter(|&&b| b).count();
    if actual_kept != output_len {
        // Fall back to a simpler evenly-spaced selection
        pattern = vec![false; input_len];
        for k in 0..output_len {
            let idx = k * input_len / output_len;
            pattern[idx] = true;
        }
        // Resolve collisions: if two k values map to the same idx, shift right
        let mut kept_count = pattern.iter().filter(|&&b| b).count();
        if kept_count < output_len {
            for i in 0..input_len {
                if !pattern[i] {
                    pattern[i] = true;
                    kept_count += 1;
                    if kept_count == output_len {
                        break;
                    }
                }
            }
        }
    }

    pattern
}

/// Compute the effective code rate.
///
/// Code rate = input_bits / output_bits.  A rate of 1.0 means no redundancy;
/// values less than 1.0 indicate redundancy has been added (e.g., 1/2, 3/4).
///
/// Returns `0.0` if `output_bits == 0`.
pub fn code_rate(input_bits: usize, output_bits: usize) -> f64 {
    if output_bits == 0 {
        return 0.0;
    }
    input_bits as f64 / output_bits as f64
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_puncture() {
        let rm = RateMatcher::new(6, RateMatchMode::Puncture);
        let input: Vec<f64> = (1..=10).map(|x| x as f64).collect();
        let output = rm.match_rate(&input);
        assert_eq!(output.len(), 6, "Punctured output should have 6 elements");
        // All kept values must come from the original input
        for &v in &output {
            assert!(
                input.contains(&v),
                "Output value {} not found in input",
                v
            );
        }
    }

    #[test]
    fn test_repeat() {
        let rm = RateMatcher::new(8, RateMatchMode::Repeat);
        let input = vec![1.0, 2.0, 3.0];
        let output = rm.match_rate(&input);
        assert_eq!(output.len(), 8, "Repeated output should have 8 elements");
        // Circular pattern: 1,2,3,1,2,3,1,2
        assert_eq!(output, vec![1.0, 2.0, 3.0, 1.0, 2.0, 3.0, 1.0, 2.0]);
    }

    #[test]
    fn test_passthrough() {
        let rm = RateMatcher::new(5, RateMatchMode::Puncture);
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let output = rm.match_rate(&input);
        assert_eq!(output, input, "Passthrough should return input unchanged");
    }

    #[test]
    fn test_unmatch_puncture() {
        let rm = RateMatcher::new(6, RateMatchMode::Puncture);
        let input: Vec<f64> = (1..=10).map(|x| x as f64).collect();
        let matched = rm.match_rate(&input);
        assert_eq!(matched.len(), 6);

        let recovered = rm.unmatch_rate(&matched, 10);
        assert_eq!(
            recovered.len(),
            10,
            "Recovered should have original length"
        );

        // Punctured positions should be zero (erasure)
        let pattern = compute_puncture_pattern(10, 6);
        let mut match_idx = 0;
        for (i, &keep) in pattern.iter().enumerate() {
            if keep {
                assert!(
                    (recovered[i] - matched[match_idx]).abs() < 1e-12,
                    "Kept position {} should match",
                    i
                );
                match_idx += 1;
            } else {
                assert!(
                    recovered[i].abs() < 1e-12,
                    "Punctured position {} should be zero (erasure)",
                    i
                );
            }
        }
    }

    #[test]
    fn test_unmatch_repeat() {
        let rm = RateMatcher::new(9, RateMatchMode::Repeat);
        let input = vec![2.0, 4.0, 6.0];
        let matched = rm.match_rate(&input);
        assert_eq!(matched.len(), 9);
        // matched = [2,4,6, 2,4,6, 2,4,6]

        let recovered = rm.unmatch_rate(&matched, 3);
        assert_eq!(recovered.len(), 3);
        // Each position was repeated 3 times, average = same value
        assert!((recovered[0] - 2.0).abs() < 1e-12);
        assert!((recovered[1] - 4.0).abs() < 1e-12);
        assert!((recovered[2] - 6.0).abs() < 1e-12);
    }

    #[test]
    fn test_circular_buffer() {
        let mut cb = CircularBuffer::new(4);
        cb.fill(&[10.0, 20.0, 30.0, 40.0]);

        let r = cb.read(4);
        assert_eq!(r, vec![10.0, 20.0, 30.0, 40.0]);

        // Read with wrap-around
        let r2 = cb.read(7);
        assert_eq!(r2, vec![10.0, 20.0, 30.0, 40.0, 10.0, 20.0, 30.0]);

        // Fill with data longer than buffer (overwrites with wrap)
        cb.fill(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        // Position 0 = 5.0 (overwritten by index 4), position 1 = 6.0 (overwritten by index 5)
        let r3 = cb.read(4);
        assert_eq!(r3, vec![5.0, 6.0, 3.0, 4.0]);
    }

    #[test]
    fn test_lte_interleave() {
        let lte = LteRateMatcher::new(0);
        let input: Vec<f64> = (0..64).map(|x| x as f64).collect();
        let interleaved = lte.interleave(&input);
        assert_eq!(
            interleaved.len(),
            input.len(),
            "Interleaved length must match input"
        );

        // Interleaving must be a permutation (all values present)
        let mut sorted_out = interleaved.clone();
        sorted_out.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let mut sorted_in = input.clone();
        sorted_in.sort_by(|a, b| a.partial_cmp(b).unwrap());
        assert_eq!(sorted_out, sorted_in, "Interleave must be a permutation");

        // Verify it actually changes the order
        assert_ne!(interleaved, input, "Interleaving should reorder elements");

        // Round-trip
        let deinterleaved = lte.deinterleave(&interleaved);
        for (i, (&orig, &rec)) in input.iter().zip(deinterleaved.iter()).enumerate() {
            assert!(
                (orig - rec).abs() < 1e-12,
                "De-interleave mismatch at position {}",
                i
            );
        }
    }

    #[test]
    fn test_puncture_pattern() {
        // Keep 3 out of 5
        let pattern = compute_puncture_pattern(5, 3);
        assert_eq!(pattern.len(), 5);
        let kept = pattern.iter().filter(|&&b| b).count();
        assert_eq!(kept, 3, "Should keep exactly 3 positions");

        // Keep all
        let all = compute_puncture_pattern(5, 5);
        assert_eq!(all, vec![true; 5]);

        // Keep none
        let none = compute_puncture_pattern(5, 0);
        assert_eq!(none, vec![false; 5]);
    }

    #[test]
    fn test_code_rate() {
        // Rate 1/2 code: 100 info bits → 200 coded bits
        let r = code_rate(100, 200);
        assert!((r - 0.5).abs() < 1e-12, "Expected 0.5, got {}", r);

        // Rate 3/4
        let r2 = code_rate(3, 4);
        assert!((r2 - 0.75).abs() < 1e-12, "Expected 0.75, got {}", r2);

        // No output
        let r3 = code_rate(10, 0);
        assert!(r3.abs() < 1e-12, "Expected 0.0 for zero output");

        // Rate 1 (no coding)
        let r4 = code_rate(100, 100);
        assert!((r4 - 1.0).abs() < 1e-12, "Expected 1.0, got {}", r4);
    }

    #[test]
    fn test_empty_input() {
        let rm_punct = RateMatcher::new(5, RateMatchMode::Puncture);
        let out = rm_punct.match_rate(&[]);
        assert_eq!(out.len(), 5, "Empty input should produce output_length zeros");
        assert!(out.iter().all(|&v| v == 0.0));

        let rm_rep = RateMatcher::new(0, RateMatchMode::Repeat);
        let out2 = rm_rep.match_rate(&[1.0, 2.0]);
        assert!(out2.is_empty(), "Zero output_length should produce empty");

        // Unmatch with zero original length
        let out3 = rm_punct.unmatch_rate(&[1.0, 2.0, 3.0], 0);
        assert!(out3.is_empty());

        // CircularBuffer edge case
        let cb = CircularBuffer::new(0);
        assert!(cb.is_empty());
        let r = cb.read(3);
        assert_eq!(r.len(), 3);

        // LTE interleave empty
        let lte = LteRateMatcher::new(0);
        let out4 = lte.interleave(&[]);
        assert!(out4.is_empty());
    }
}
