//! Soft-Decision Decoding Utilities for FEC Codes
//!
//! Provides log-likelihood ratio (LLR) based processing for soft-decision
//! forward error correction decoding. Includes LLR computation from BPSK/QPSK
//! soft samples, repetition code combining, weighted majority voting, and
//! the boxplus operation used in LDPC belief propagation.
//!
//! ## Sign Convention
//!
//! LLR > 0 means bit = 0 is more likely; LLR < 0 means bit = 1 is more likely.
//! The magnitude indicates confidence.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::soft_decision_decoder::{Llr, hard_decide, llr_from_bpsk, clip_llr};
//!
//! // Received BPSK sample with noise variance 0.5
//! let sample = 0.8;
//! let noise_variance = 0.5;
//! let llr: Llr = llr_from_bpsk(sample, noise_variance);
//! assert!(llr > 0.0); // Positive LLR => bit 0 more likely
//!
//! let bit = hard_decide(llr);
//! assert_eq!(bit, false); // false represents bit 0
//!
//! // Clamp for numerical stability in iterative decoders
//! let clamped = clip_llr(llr, 20.0);
//! assert!(clamped.abs() <= 20.0);
//! ```

/// Log-likelihood ratio type alias.
///
/// Positive values indicate bit 0 is more likely; negative values indicate
/// bit 1 is more likely. The magnitude represents confidence.
pub type Llr = f64;

/// Make a hard decision from a single LLR value.
///
/// Returns `false` (bit 0) when LLR >= 0, `true` (bit 1) when LLR < 0.
///
/// # Arguments
///
/// * `llr` - Log-likelihood ratio value
pub fn hard_decide(llr: Llr) -> bool {
    llr < 0.0
}

/// Make hard decisions on a slice of LLR values.
///
/// Applies [`hard_decide`] to each element and returns the resulting bit vector.
///
/// # Arguments
///
/// * `llrs` - Slice of log-likelihood ratio values
pub fn hard_decide_slice(llrs: &[Llr]) -> Vec<bool> {
    llrs.iter().map(|&l| hard_decide(l)).collect()
}

/// Compute LLR from a BPSK soft sample.
///
/// For BPSK with mapping {+1 -> bit 0, -1 -> bit 1}, the LLR is:
///
///   LLR = 2 * sample / sigma^2
///
/// where sigma^2 is the noise variance.
///
/// # Arguments
///
/// * `sample` - Received soft sample (real-valued)
/// * `noise_variance` - Noise variance (sigma^2); must be positive
///
/// # Panics
///
/// Panics if `noise_variance` is not positive.
pub fn llr_from_bpsk(sample: f64, noise_variance: f64) -> Llr {
    assert!(noise_variance > 0.0, "noise_variance must be positive");
    2.0 * sample / noise_variance
}

/// Compute LLRs from QPSK I and Q soft samples.
///
/// For QPSK with independent I and Q channels, each channel is treated
/// as an independent BPSK signal:
///
///   LLR_I = 2 * i / sigma^2
///   LLR_Q = 2 * q / sigma^2
///
/// # Arguments
///
/// * `i` - In-phase soft sample
/// * `q` - Quadrature soft sample
/// * `noise_variance` - Noise variance (sigma^2); must be positive
///
/// # Returns
///
/// Tuple of (LLR for I-channel bit, LLR for Q-channel bit).
///
/// # Panics
///
/// Panics if `noise_variance` is not positive.
pub fn llr_from_qpsk(i: f64, q: f64, noise_variance: f64) -> (Llr, Llr) {
    assert!(noise_variance > 0.0, "noise_variance must be positive");
    let scale = 2.0 / noise_variance;
    (i * scale, q * scale)
}

/// Boxplus operation (min-sum approximation) for LDPC decoding.
///
/// Computes the check-node update in min-sum LDPC decoding:
///
///   boxplus(a, b) = sign(a) * sign(b) * min(|a|, |b|)
///
/// This approximation avoids the expensive tanh/atanh computations
/// of the exact formula, with minimal performance loss in practice.
///
/// # Arguments
///
/// * `a` - First LLR operand
/// * `b` - Second LLR operand
pub fn boxplus(a: Llr, b: Llr) -> Llr {
    a.signum() * b.signum() * a.abs().min(b.abs())
}

/// Exact boxplus operation for LDPC decoding.
///
/// Computes the exact check-node update using the sum-product algorithm:
///
///   boxplus_exact(a, b) = 2 * atanh(tanh(a/2) * tanh(b/2))
///
/// More accurate than [`boxplus`] but computationally more expensive.
/// May produce NaN/Inf for very large inputs; consider using [`clip_llr`]
/// before calling this function.
///
/// # Arguments
///
/// * `a` - First LLR operand
/// * `b` - Second LLR operand
pub fn boxplus_exact(a: Llr, b: Llr) -> Llr {
    let product = (a / 2.0).tanh() * (b / 2.0).tanh();
    // Clamp to avoid atanh domain issues at exactly +/-1
    let clamped = product.clamp(-1.0 + f64::EPSILON, 1.0 - f64::EPSILON);
    2.0 * clamped.atanh()
}

/// Clamp an LLR magnitude for numerical stability.
///
/// Restricts the LLR to the range [-max_abs, +max_abs]. Useful in iterative
/// decoding algorithms (turbo, LDPC) to prevent runaway values.
///
/// # Arguments
///
/// * `llr` - Log-likelihood ratio to clamp
/// * `max_abs` - Maximum allowed absolute value; must be non-negative
pub fn clip_llr(llr: Llr, max_abs: f64) -> Llr {
    llr.clamp(-max_abs, max_abs)
}

/// Soft-decision repetition code decoder.
///
/// Combines repeated transmissions of the same bit by summing their LLR
/// values. For a rate-1/N repetition code, this is the optimal soft combining
/// strategy under AWGN.
///
/// # Example
///
/// ```rust
/// use r4w_core::soft_decision_decoder::{SoftRepetitionDecoder, Llr};
///
/// let decoder = SoftRepetitionDecoder::new(3);
/// let llrs: Vec<Llr> = vec![1.0, 0.8, 1.2, -0.5, -0.7, -0.3];
/// let combined = decoder.decode(&llrs);
/// assert_eq!(combined.len(), 2); // 6 LLRs / 3 repetitions = 2 bits
/// ```
pub struct SoftRepetitionDecoder {
    repetitions: usize,
}

impl SoftRepetitionDecoder {
    /// Create a new soft repetition decoder.
    ///
    /// # Arguments
    ///
    /// * `repetitions` - Number of times each bit is repeated; must be >= 1
    ///
    /// # Panics
    ///
    /// Panics if `repetitions` is zero.
    pub fn new(repetitions: usize) -> Self {
        assert!(repetitions >= 1, "repetitions must be at least 1");
        Self { repetitions }
    }

    /// Decode by summing groups of `repetitions` LLRs.
    ///
    /// The input length must be a multiple of `repetitions`. Each group of
    /// consecutive LLRs is summed to produce one output LLR per original bit.
    ///
    /// # Arguments
    ///
    /// * `llrs` - Input LLR slice; length must be a multiple of `repetitions`
    ///
    /// # Panics
    ///
    /// Panics if the input length is not a multiple of `repetitions`.
    pub fn decode(&self, llrs: &[Llr]) -> Vec<Llr> {
        assert!(
            llrs.len() % self.repetitions == 0,
            "input length {} is not a multiple of repetitions {}",
            llrs.len(),
            self.repetitions
        );
        llrs.chunks(self.repetitions)
            .map(|chunk| chunk.iter().sum())
            .collect()
    }
}

/// Soft-decision weighted majority decoder.
///
/// Uses LLR magnitudes as weights for majority voting. Each input LLR
/// contributes its magnitude toward the bit decision weighted by its sign.
/// The output is a single combined LLR representing the weighted vote.
///
/// This is useful for diversity combining or simple repetition scenarios
/// where the reliability of each observation may differ.
pub struct SoftMajorityDecoder {
    voters: usize,
}

impl SoftMajorityDecoder {
    /// Create a new soft majority decoder.
    ///
    /// # Arguments
    ///
    /// * `voters` - Number of LLR inputs to combine per decision; must be >= 1
    ///
    /// # Panics
    ///
    /// Panics if `voters` is zero.
    pub fn new(voters: usize) -> Self {
        assert!(voters >= 1, "voters must be at least 1");
        Self { voters }
    }

    /// Decode by weighted majority voting on groups of `voters` LLRs.
    ///
    /// For each group of `voters` LLRs, the output is the sum of signed
    /// magnitudes: each LLR contributes its value (sign * magnitude) to the
    /// combined decision. This is equivalent to MRC (maximal ratio combining)
    /// for equal-variance observations.
    ///
    /// # Arguments
    ///
    /// * `llrs` - Input LLR slice; length must be a multiple of `voters`
    ///
    /// # Panics
    ///
    /// Panics if the input length is not a multiple of `voters`.
    pub fn decode(&self, llrs: &[Llr]) -> Vec<Llr> {
        assert!(
            llrs.len() % self.voters == 0,
            "input length {} is not a multiple of voters {}",
            llrs.len(),
            self.voters
        );
        llrs.chunks(self.voters)
            .map(|chunk| {
                // Weighted majority: sum of LLR values.
                // Positive LLRs vote for bit 0, negative for bit 1,
                // weighted by their magnitude (confidence).
                chunk.iter().sum()
            })
            .collect()
    }
}

/// Buffer for accumulating extrinsic LLR information in iterative decoding.
///
/// In turbo and LDPC decoders, extrinsic information is passed between
/// component decoders across iterations. This buffer stores the accumulated
/// extrinsic LLRs and provides methods for updating and retrieving them.
///
/// # Example
///
/// ```rust
/// use r4w_core::soft_decision_decoder::LlrBuffer;
///
/// let mut buf = LlrBuffer::new(4);
/// buf.add_extrinsic(&[0.5, -0.3, 0.1, 0.8]);
/// buf.add_extrinsic(&[0.2, -0.1, 0.3, -0.2]);
/// let accumulated = buf.llrs();
/// assert!((accumulated[0] - 0.7).abs() < 1e-10);
/// ```
pub struct LlrBuffer {
    data: Vec<Llr>,
}

impl LlrBuffer {
    /// Create a new LLR buffer initialized to zeros.
    ///
    /// # Arguments
    ///
    /// * `len` - Number of LLR values to store
    pub fn new(len: usize) -> Self {
        Self {
            data: vec![0.0; len],
        }
    }

    /// Add extrinsic information element-wise.
    ///
    /// Each element in `extrinsic` is added to the corresponding buffer element.
    ///
    /// # Arguments
    ///
    /// * `extrinsic` - Extrinsic LLR values to accumulate; must have the same
    ///   length as the buffer.
    ///
    /// # Panics
    ///
    /// Panics if the length of `extrinsic` does not match the buffer length.
    pub fn add_extrinsic(&mut self, extrinsic: &[Llr]) {
        assert_eq!(
            extrinsic.len(),
            self.data.len(),
            "extrinsic length {} does not match buffer length {}",
            extrinsic.len(),
            self.data.len()
        );
        for (d, &e) in self.data.iter_mut().zip(extrinsic.iter()) {
            *d += e;
        }
    }

    /// Get a reference to the accumulated LLR values.
    pub fn llrs(&self) -> &[Llr] {
        &self.data
    }

    /// Get the number of LLR values in the buffer.
    pub fn len(&self) -> usize {
        self.data.len()
    }

    /// Returns true if the buffer is empty.
    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    /// Reset all accumulated LLR values to zero.
    pub fn reset(&mut self) {
        self.data.iter_mut().for_each(|d| *d = 0.0);
    }

    /// Clip all accumulated LLR values to the given maximum magnitude.
    ///
    /// # Arguments
    ///
    /// * `max_abs` - Maximum allowed absolute value
    pub fn clip_all(&mut self, max_abs: f64) {
        self.data.iter_mut().for_each(|d| *d = clip_llr(*d, max_abs));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hard_decide_positive() {
        // Positive LLR => bit 0 (false)
        assert_eq!(hard_decide(2.5), false);
        assert_eq!(hard_decide(0.001), false);
    }

    #[test]
    fn test_hard_decide_negative() {
        // Negative LLR => bit 1 (true)
        assert_eq!(hard_decide(-1.0), true);
        assert_eq!(hard_decide(-0.001), true);
    }

    #[test]
    fn test_hard_decide_zero() {
        // Zero LLR => bit 0 (false) by convention (>= 0 => false)
        assert_eq!(hard_decide(0.0), false);
    }

    #[test]
    fn test_hard_decide_slice_batch() {
        let llrs = vec![1.0, -2.0, 0.0, -0.5, 3.0];
        let bits = hard_decide_slice(&llrs);
        assert_eq!(bits, vec![false, true, false, true, false]);
    }

    #[test]
    fn test_llr_from_bpsk_correctness() {
        // LLR = 2 * sample / noise_variance
        let llr = llr_from_bpsk(1.0, 0.5);
        assert!((llr - 4.0).abs() < 1e-12, "expected 4.0, got {}", llr);

        let llr_neg = llr_from_bpsk(-0.5, 1.0);
        assert!((llr_neg - (-1.0)).abs() < 1e-12, "expected -1.0, got {}", llr_neg);

        // Zero sample => zero LLR
        let llr_zero = llr_from_bpsk(0.0, 1.0);
        assert!((llr_zero).abs() < 1e-12);
    }

    #[test]
    fn test_llr_from_qpsk_correctness() {
        let (llr_i, llr_q) = llr_from_qpsk(1.0, -0.5, 0.5);
        // LLR_I = 2 * 1.0 / 0.5 = 4.0
        assert!((llr_i - 4.0).abs() < 1e-12, "expected I LLR 4.0, got {}", llr_i);
        // LLR_Q = 2 * (-0.5) / 0.5 = -2.0
        assert!((llr_q - (-2.0)).abs() < 1e-12, "expected Q LLR -2.0, got {}", llr_q);
    }

    #[test]
    fn test_soft_repetition_decoder_combining() {
        let decoder = SoftRepetitionDecoder::new(3);
        // Three repetitions of two bits
        let llrs = vec![1.0, 0.8, 1.2, -0.5, -0.7, -0.3];
        let combined = decoder.decode(&llrs);
        assert_eq!(combined.len(), 2);
        // First bit: 1.0 + 0.8 + 1.2 = 3.0
        assert!((combined[0] - 3.0).abs() < 1e-12);
        // Second bit: -0.5 + -0.7 + -0.3 = -1.5
        assert!((combined[1] - (-1.5)).abs() < 1e-12);
    }

    #[test]
    fn test_boxplus_min_sum() {
        // sign(2)*sign(3)*min(2,3) = 1*1*2 = 2
        assert!((boxplus(2.0, 3.0) - 2.0).abs() < 1e-12);

        // sign(-2)*sign(3)*min(2,3) = -1*1*2 = -2
        assert!((boxplus(-2.0, 3.0) - (-2.0)).abs() < 1e-12);

        // sign(1)*sign(-0.5)*min(1,0.5) = 1*(-1)*0.5 = -0.5
        assert!((boxplus(1.0, -0.5) - (-0.5)).abs() < 1e-12);

        // Both negative: sign(-3)*sign(-4)*min(3,4) = (-1)*(-1)*3 = 3
        assert!((boxplus(-3.0, -4.0) - 3.0).abs() < 1e-12);
    }

    #[test]
    fn test_boxplus_exact_known_values() {
        // For small values, boxplus_exact should be close to boxplus
        // but slightly less in magnitude (correction factor).
        let exact = boxplus_exact(2.0, 3.0);
        let approx = boxplus(2.0, 3.0);
        // Exact should be <= approx in magnitude (min-sum overestimates)
        assert!(exact.abs() <= approx.abs() + 1e-10,
            "exact {} should not exceed approx {} in magnitude", exact, approx);
        // Both should have the same sign
        assert!(exact.signum() == approx.signum());
        // Known: 2*atanh(tanh(1)*tanh(1.5)) ~ 1.6935
        assert!((exact - 1.6935).abs() < 0.001,
            "expected ~1.6935, got {}", exact);

        // Symmetric: boxplus_exact(a,b) == boxplus_exact(b,a)
        let ab = boxplus_exact(1.5, -2.0);
        let ba = boxplus_exact(-2.0, 1.5);
        assert!((ab - ba).abs() < 1e-12);
    }

    #[test]
    fn test_clip_llr_clamping() {
        assert!((clip_llr(5.0, 3.0) - 3.0).abs() < 1e-12);
        assert!((clip_llr(-5.0, 3.0) - (-3.0)).abs() < 1e-12);
        assert!((clip_llr(2.0, 3.0) - 2.0).abs() < 1e-12);
        assert!((clip_llr(-1.0, 3.0) - (-1.0)).abs() < 1e-12);
        assert!((clip_llr(0.0, 3.0)).abs() < 1e-12);
    }

    #[test]
    fn test_llr_buffer_accumulation() {
        let mut buf = LlrBuffer::new(3);
        assert_eq!(buf.len(), 3);
        assert!(!buf.is_empty());

        // Initially all zeros
        for &v in buf.llrs() {
            assert!((v).abs() < 1e-12);
        }

        // First accumulation
        buf.add_extrinsic(&[1.0, -2.0, 0.5]);
        assert!((buf.llrs()[0] - 1.0).abs() < 1e-12);
        assert!((buf.llrs()[1] - (-2.0)).abs() < 1e-12);
        assert!((buf.llrs()[2] - 0.5).abs() < 1e-12);

        // Second accumulation
        buf.add_extrinsic(&[0.5, 1.0, -0.5]);
        assert!((buf.llrs()[0] - 1.5).abs() < 1e-12);
        assert!((buf.llrs()[1] - (-1.0)).abs() < 1e-12);
        assert!((buf.llrs()[2]).abs() < 1e-12);

        // Reset
        buf.reset();
        for &v in buf.llrs() {
            assert!((v).abs() < 1e-12);
        }

        // Clip test
        buf.add_extrinsic(&[100.0, -50.0, 3.0]);
        buf.clip_all(10.0);
        assert!((buf.llrs()[0] - 10.0).abs() < 1e-12);
        assert!((buf.llrs()[1] - (-10.0)).abs() < 1e-12);
        assert!((buf.llrs()[2] - 3.0).abs() < 1e-12);
    }

    #[test]
    fn test_soft_majority_decoder_weighted_voting() {
        let decoder = SoftMajorityDecoder::new(3);

        // Three voters: two vote for bit 0 (positive), one for bit 1 (negative)
        // Voter magnitudes differ, so it's a weighted vote
        let llrs = vec![2.0, 1.0, -0.5];
        let result = decoder.decode(&llrs);
        assert_eq!(result.len(), 1);
        // Sum: 2.0 + 1.0 + (-0.5) = 2.5 => bit 0 wins
        assert!((result[0] - 2.5).abs() < 1e-12);
        assert!(result[0] > 0.0, "weighted majority should vote for bit 0");

        // Opposite case: strong negative voter outweighs two weak positives
        let llrs2 = vec![0.3, 0.2, -5.0];
        let result2 = decoder.decode(&llrs2);
        // Sum: 0.3 + 0.2 + (-5.0) = -4.5 => bit 1 wins
        assert!((result2[0] - (-4.5)).abs() < 1e-12);
        assert!(result2[0] < 0.0, "strong negative should dominate");

        // Multiple groups
        let llrs3 = vec![1.0, 1.0, 1.0, -1.0, -1.0, -1.0];
        let result3 = decoder.decode(&llrs3);
        assert_eq!(result3.len(), 2);
        assert!((result3[0] - 3.0).abs() < 1e-12);
        assert!((result3[1] - (-3.0)).abs() < 1e-12);
    }
}
