//! Gold Code Generator — Gold codes for CDMA and GNSS applications
//!
//! Gold codes are a class of binary sequences used extensively in CDMA
//! spread-spectrum systems and GNSS (e.g., GPS C/A codes). They are formed
//! by XORing two maximal-length sequences (m-sequences) generated from
//! preferred pairs of LFSR polynomials. The resulting code family has
//! 2^n + 1 members (including the two base m-sequences) with bounded
//! cross-correlation properties, making them ideal for multi-user and
//! multi-satellite environments.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::gold_code_generator::{GoldCodeGenerator, gold_code_5};
//!
//! // Create a degree-5 Gold code generator (family of 33 codes)
//! let gen = gold_code_5();
//! assert_eq!(gen.family_size(), 33);
//!
//! // Generate two different Gold codes
//! let code_a = gen.generate(0);
//! let code_b = gen.generate(1);
//! assert_eq!(code_a.len(), 31); // 2^5 - 1
//! assert_ne!(code_a, code_b);
//! ```

/// Linear Feedback Shift Register (LFSR) for generating maximal-length sequences.
///
/// An m-sequence of degree `n` has period 2^n - 1 and near-ideal autocorrelation
/// properties. The LFSR is configured with feedback taps that define the
/// generator polynomial.
#[derive(Debug, Clone)]
pub struct MSequence {
    /// Feedback tap positions (1-indexed bit positions in the register).
    pub taps: Vec<usize>,
    /// Current shift register state stored as a bitmask.
    pub register: u32,
    /// Length of the generated sequence (2^degree - 1).
    pub length: usize,
    /// Degree of the generator polynomial (register width).
    pub degree: usize,
}

impl MSequence {
    /// Create a new m-sequence generator.
    ///
    /// - `degree`: polynomial degree (register width, typically 3-20)
    /// - `taps`: feedback tap positions (1-indexed; tap `degree` is the MSB)
    ///
    /// The register is initialized to all ones.
    pub fn new(degree: usize, taps: Vec<usize>) -> Self {
        let length = (1usize << degree) - 1;
        let register = (1u32 << degree) - 1; // all ones
        Self {
            taps,
            register,
            length,
            degree,
        }
    }

    /// Generate the full m-sequence as bipolar (+1/-1) values.
    ///
    /// Returns a vector of length 2^degree - 1. Each output chip is
    /// +1 (for register output bit 0) or -1 (for register output bit 1),
    /// following the BPSK convention: chip = 1 - 2*bit.
    pub fn generate(&mut self) -> Vec<i8> {
        let mut seq = Vec::with_capacity(self.length);
        for _ in 0..self.length {
            // Output is the LSB of the register
            let output_bit = self.register & 1;
            seq.push(if output_bit == 0 { 1 } else { -1 });

            // Compute feedback as XOR of all tapped positions
            let mut feedback = 0u32;
            for &tap in &self.taps {
                feedback ^= (self.register >> (tap - 1)) & 1;
            }

            // Shift register right and insert feedback at MSB
            self.register >>= 1;
            self.register |= feedback << (self.degree - 1);
        }
        seq
    }

    /// Reset the register to all ones.
    pub fn reset(&mut self) {
        self.register = (1u32 << self.degree) - 1;
    }
}

/// Gold code generator using a preferred pair of m-sequences.
///
/// A Gold code family of degree `n` produces 2^n + 1 codes of length
/// 2^n - 1. Each code is formed by XORing the first m-sequence with a
/// cyclically shifted version of the second m-sequence. The resulting
/// codes have three-valued cross-correlation bounded by 2^((n+1)/2) + 1
/// (for odd n) or 2^((n+2)/2) + 1 (for even n).
#[derive(Debug, Clone)]
pub struct GoldCodeGenerator {
    /// Degree of the generator polynomials.
    pub degree: usize,
    /// Feedback taps for the first m-sequence.
    pub seq1_taps: Vec<usize>,
    /// Feedback taps for the second m-sequence.
    pub seq2_taps: Vec<usize>,
}

impl GoldCodeGenerator {
    /// Create a new Gold code generator from a preferred pair of LFSR tap sets.
    ///
    /// - `degree`: polynomial degree (both LFSRs must have the same degree)
    /// - `taps1`: feedback taps for the first m-sequence
    /// - `taps2`: feedback taps for the second m-sequence
    pub fn new(degree: usize, taps1: Vec<usize>, taps2: Vec<usize>) -> Self {
        Self {
            degree,
            seq1_taps: taps1,
            seq2_taps: taps2,
        }
    }

    /// Generate a Gold code for the given code index.
    ///
    /// - `code_index` 0 returns m-sequence 1 alone.
    /// - `code_index` 1..=2^degree - 1 returns m-seq1 XOR cyclic-shift of m-seq2.
    /// - `code_index` 2^degree returns m-sequence 2 alone.
    ///
    /// The XOR is performed in bipolar representation: multiplication of
    /// +1/-1 values is equivalent to XOR of the underlying binary bits.
    pub fn generate(&self, code_index: usize) -> Vec<i8> {
        let code_len = (1usize << self.degree) - 1;

        // Generate the two base m-sequences
        let mut mseq1 = MSequence::new(self.degree, self.seq1_taps.clone());
        let seq1 = mseq1.generate();

        let mut mseq2 = MSequence::new(self.degree, self.seq2_taps.clone());
        let seq2 = mseq2.generate();

        if code_index == 0 {
            // Return first m-sequence directly
            return seq1;
        }

        if code_index == code_len + 1 {
            // Return second m-sequence directly
            return seq2;
        }

        // XOR seq1 with cyclically shifted seq2
        // Shift amount is (code_index - 1) so code_index=1 gives shift=0, etc.
        let shift = code_index - 1;
        let mut gold = Vec::with_capacity(code_len);
        for i in 0..code_len {
            let j = (i + shift) % code_len;
            // Bipolar XOR: multiply +1/-1 values
            gold.push(seq1[i] * seq2[j]);
        }
        gold
    }

    /// Return the number of codes in the Gold code family.
    ///
    /// A degree-n Gold family has 2^n + 1 members: the 2^n - 1 XOR
    /// combinations plus the two base m-sequences.
    pub fn family_size(&self) -> usize {
        (1usize << self.degree) + 1
    }
}

/// Compute the peak (maximum absolute) cross-correlation between two bipolar codes.
///
/// The cross-correlation at lag `k` is the inner product of `code1` and the
/// cyclic shift of `code2` by `k` chips. This function returns the maximum
/// absolute value across all lags.
pub fn cross_correlation_peak(code1: &[i8], code2: &[i8]) -> i64 {
    assert_eq!(code1.len(), code2.len(), "codes must have equal length");
    let n = code1.len();
    let mut peak = 0i64;
    for shift in 0..n {
        let mut sum = 0i64;
        for i in 0..n {
            let j = (i + shift) % n;
            sum += (code1[i] as i64) * (code2[j] as i64);
        }
        let abs_sum = sum.abs();
        if abs_sum > peak {
            peak = abs_sum;
        }
    }
    peak
}

/// Factory for a degree-5 Gold code generator.
///
/// Uses preferred pair polynomials x^5+x^3+1 (taps [1,3]) and
/// x^5+x^4+x^3+x^2+1 (taps [1,2,3,4]). Produces 33 codes of length 31.
/// Three-valued cross-correlation: {-9, -1, 7}. Useful for testing and
/// small-scale CDMA simulations.
pub fn gold_code_5() -> GoldCodeGenerator {
    GoldCodeGenerator::new(5, vec![1, 3], vec![1, 2, 3, 4])
}

/// Factory for a degree-10 Gold code generator (GPS C/A standard).
///
/// Uses the GPS L1 C/A preferred pair: G1 polynomial x^10+x^3+1 (taps [1,8])
/// and G2 polynomial x^10+x^9+x^8+x^6+x^3+x^2+1 (taps [1,2,3,5,8,9]).
/// Produces 1025 codes of length 1023, matching the GPS C/A code structure.
/// Three-valued cross-correlation: {-65, -1, 63}.
pub fn gold_code_10() -> GoldCodeGenerator {
    GoldCodeGenerator::new(10, vec![1, 8], vec![1, 2, 3, 5, 8, 9])
}

/// Check the balance property of a bipolar code.
///
/// A balanced binary sequence has the number of +1 chips and -1 chips
/// differing by at most 1. M-sequences and most Gold codes satisfy this
/// property. Returns `true` if |count(+1) - count(-1)| <= 1.
pub fn balanced_property(code: &[i8]) -> bool {
    let ones: i64 = code.iter().filter(|&&c| c == 1).count() as i64;
    let neg_ones: i64 = code.iter().filter(|&&c| c == -1).count() as i64;
    (ones - neg_ones).abs() <= 1
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_m_sequence_length() {
        // Degree-5 m-sequence should have length 2^5 - 1 = 31
        let mut mseq = MSequence::new(5, vec![1, 3]);
        let seq = mseq.generate();
        assert_eq!(seq.len(), 31);

        // Degree-10 m-sequence should have length 2^10 - 1 = 1023
        let mut mseq10 = MSequence::new(10, vec![1, 8]);
        let seq10 = mseq10.generate();
        assert_eq!(seq10.len(), 1023);
    }

    #[test]
    fn test_m_sequence_balance() {
        // M-sequences have exactly 2^(n-1) ones and 2^(n-1)-1 zeros
        // In bipolar: count(+1) and count(-1) differ by exactly 1.
        let mut mseq = MSequence::new(5, vec![1, 3]);
        let seq = mseq.generate();
        assert!(balanced_property(&seq), "m-sequence should be balanced");

        let mut mseq10 = MSequence::new(10, vec![1, 8]);
        let seq10 = mseq10.generate();
        assert!(balanced_property(&seq10), "degree-10 m-sequence should be balanced");
    }

    #[test]
    fn test_gold_code_generation() {
        let gen = gold_code_5();
        let code = gen.generate(1);
        assert_eq!(code.len(), 31);

        // All chips must be +1 or -1
        for &c in &code {
            assert!(c == 1 || c == -1, "chip must be bipolar, got {}", c);
        }
    }

    #[test]
    fn test_family_size() {
        let gen5 = gold_code_5();
        assert_eq!(gen5.family_size(), 33); // 2^5 + 1

        let gen10 = gold_code_10();
        assert_eq!(gen10.family_size(), 1025); // 2^10 + 1
    }

    #[test]
    fn test_cross_correlation_bounded() {
        // For degree-5 Gold codes, the cross-correlation should be bounded
        // by 2^((5+1)/2) + 1 = 2^3 + 1 = 9
        let gen = gold_code_5();
        let code_a = gen.generate(1);
        let code_b = gen.generate(2);
        let peak = cross_correlation_peak(&code_a, &code_b);
        assert!(
            peak <= 9,
            "cross-correlation peak {} exceeds Gold bound 9",
            peak
        );
    }

    #[test]
    fn test_autocorrelation_peak() {
        // The autocorrelation of an m-sequence at zero lag equals the
        // sequence length. At all other lags it equals -1.
        let mut mseq = MSequence::new(5, vec![1, 3]);
        let seq = mseq.generate();
        let n = seq.len();

        // Zero-lag autocorrelation
        let zero_lag: i64 = seq.iter().map(|&c| (c as i64) * (c as i64)).sum();
        assert_eq!(zero_lag, n as i64, "zero-lag autocorrelation should equal sequence length");

        // Non-zero lag autocorrelation should be -1 for m-sequences
        for shift in 1..n {
            let mut corr = 0i64;
            for i in 0..n {
                let j = (i + shift) % n;
                corr += (seq[i] as i64) * (seq[j] as i64);
            }
            assert_eq!(
                corr, -1,
                "m-sequence autocorrelation at lag {} should be -1, got {}",
                shift, corr
            );
        }
    }

    #[test]
    fn test_different_codes() {
        // Different code indices should produce different codes
        let gen = gold_code_5();
        let code_0 = gen.generate(0);
        let code_1 = gen.generate(1);
        let code_2 = gen.generate(2);
        let code_last = gen.generate(gen.family_size() - 1);

        assert_ne!(code_0, code_1);
        assert_ne!(code_1, code_2);
        assert_ne!(code_0, code_last);

        // All codes should have the same length
        assert_eq!(code_0.len(), code_1.len());
        assert_eq!(code_1.len(), code_2.len());
        assert_eq!(code_2.len(), code_last.len());
    }

    #[test]
    fn test_gold_5() {
        let gen = gold_code_5();
        assert_eq!(gen.degree, 5);
        assert_eq!(gen.seq1_taps, vec![1, 3]);
        assert_eq!(gen.seq2_taps, vec![1, 2, 3, 4]);

        // Generate all codes and verify they are unique
        let mut codes = Vec::new();
        for i in 0..gen.family_size() {
            let code = gen.generate(i);
            assert_eq!(code.len(), 31);
            assert!(!codes.contains(&code), "code {} is a duplicate", i);
            codes.push(code);
        }
    }

    #[test]
    fn test_gold_10() {
        let gen = gold_code_10();
        assert_eq!(gen.degree, 10);
        assert_eq!(gen.seq1_taps, vec![1, 8]);
        assert_eq!(gen.seq2_taps, vec![1, 2, 3, 5, 8, 9]);

        // Generate a few codes and verify length
        for i in [0, 1, 100, 512, 1024] {
            let code = gen.generate(i);
            assert_eq!(code.len(), 1023);
        }
    }

    #[test]
    fn test_balanced() {
        // Gold codes from a degree-5 generator should mostly be balanced
        let gen = gold_code_5();
        let code = gen.generate(1);
        // Gold codes may not always be perfectly balanced, but for small
        // degree preferred pairs they often are. Verify the function itself.
        let result = balanced_property(&code);
        // Just verify the function runs without panic
        let _ = result;

        // Manually check a known balanced sequence
        let balanced = vec![1i8, -1, 1, -1, 1];
        assert!(balanced_property(&balanced));

        // Manually check an unbalanced sequence
        let unbalanced = vec![1i8, 1, 1, -1];
        assert!(!balanced_property(&unbalanced));
    }
}
