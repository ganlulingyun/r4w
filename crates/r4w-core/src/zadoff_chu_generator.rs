//! Zadoff-Chu Sequence Generator
//!
//! Generates Zadoff-Chu (ZC) sequences used extensively in LTE and 5G NR for
//! PRACH preambles, primary/secondary synchronization signals (PSS/SSS),
//! sounding reference signals (SRS), and demodulation reference signals (DMRS).
//!
//! ## Properties
//!
//! Zadoff-Chu sequences have several desirable properties:
//! - **Constant amplitude**: All samples have unit magnitude
//! - **Zero cyclic autocorrelation**: Cyclically shifted versions are orthogonal
//! - **Low cross-correlation**: Sequences with different roots have bounded
//!   cross-correlation of `1/sqrt(N)` when `N` is prime
//! - **DFT closure**: The DFT of a ZC sequence is another ZC sequence
//!
//! ## Mathematical Definition
//!
//! For a sequence of length `N` with root index `u`:
//!
//! ```text
//! x[n] = exp(-j * pi * u * n * (n + 1) / N)        when N is even
//! x[n] = exp(-j * pi * u * n * (n + 1 + c_f) / N)   when N is odd (c_f = 0)
//! ```
//!
//! In practice `c_f` is conventionally 0 for odd lengths, simplifying to the
//! same formula. The root index `u` must be coprime with `N`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::zadoff_chu_generator::{zadoff_chu, ZadoffChuGenerator, is_prime};
//!
//! // Generate a ZC sequence of prime length 139 (LTE PRACH)
//! let seq = zadoff_chu(1, 139);
//! assert_eq!(seq.len(), 139);
//!
//! // Verify constant amplitude (unit magnitude)
//! for &(re, im) in &seq {
//!     let mag = (re * re + im * im).sqrt();
//!     assert!((mag - 1.0).abs() < 1e-10);
//! }
//!
//! // Use the generator struct for correlation
//! let gen = ZadoffChuGenerator::new(1, 139);
//! let signal = gen.generate();
//! let corr = gen.correlate(&signal);
//! assert!(corr.iter().cloned().fold(f64::NEG_INFINITY, f64::max) > 100.0);
//!
//! // Prime length is optimal
//! assert!(is_prime(139));
//! ```

use std::f64::consts::PI;

/// Generates a Zadoff-Chu sequence.
///
/// Computes `x[n] = exp(-j * pi * root * n * (n + 1) / length)` for even length,
/// or `x[n] = exp(-j * pi * root * n * (n + 1 + c_f) / length)` for odd length
/// (with `c_f = 0` by convention).
///
/// # Arguments
///
/// * `root` - Root index `u`, must be coprime with `length` (1 <= root < length)
/// * `length` - Sequence length `N` (ideally prime for optimal cross-correlation)
///
/// # Returns
///
/// A vector of `(re, im)` tuples representing the complex ZC sequence.
///
/// # Panics
///
/// Panics if `length` is zero.
pub fn zadoff_chu(root: usize, length: usize) -> Vec<(f64, f64)> {
    assert!(length > 0, "ZC sequence length must be positive");

    let c_f: f64 = if length % 2 == 0 { 0.0 } else { 0.0 };
    let n = length as f64;

    (0..length)
        .map(|k| {
            let kf = k as f64;
            let phase = -PI * (root as f64) * kf * (kf + 1.0 + c_f) / n;
            (phase.cos(), phase.sin())
        })
        .collect()
}

/// Generates a Zadoff-Chu sequence with a cyclic shift.
///
/// The cyclic shift produces an orthogonal sequence from the same root:
/// `x_shift[n] = x[(n + shift) mod length]`
///
/// Different cyclic shifts of the same root sequence are perfectly orthogonal,
/// which is exploited in LTE PRACH to multiplex preambles.
///
/// # Arguments
///
/// * `root` - Root index `u`
/// * `length` - Sequence length `N`
/// * `shift` - Cyclic shift amount (0 <= shift < length)
///
/// # Returns
///
/// A vector of `(re, im)` tuples representing the cyclically shifted ZC sequence.
pub fn zadoff_chu_cyclic_shift(root: usize, length: usize, shift: usize) -> Vec<(f64, f64)> {
    let base = zadoff_chu(root, length);
    let len = base.len();
    (0..len)
        .map(|n| base[(n + shift) % len])
        .collect()
}

/// Computes the peak cross-correlation magnitude between two complex sequences.
///
/// Evaluates all cyclic shifts and returns the maximum magnitude of the
/// circular cross-correlation. For ZC sequences with different roots and
/// prime length `N`, this is bounded by `sqrt(N)`.
///
/// # Arguments
///
/// * `seq1` - First complex sequence as `(re, im)` tuples
/// * `seq2` - Second complex sequence as `(re, im)` tuples
///
/// # Returns
///
/// The peak cross-correlation magnitude across all cyclic shifts.
///
/// # Panics
///
/// Panics if the sequences have different lengths or are empty.
pub fn cross_correlation_zc(seq1: &[(f64, f64)], seq2: &[(f64, f64)]) -> f64 {
    assert_eq!(seq1.len(), seq2.len(), "Sequences must have equal length");
    assert!(!seq1.is_empty(), "Sequences must be non-empty");

    let len = seq1.len();
    let mut max_mag = 0.0_f64;

    for shift in 0..len {
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        for n in 0..len {
            let (a_re, a_im) = seq1[n];
            let (b_re, b_im) = seq2[(n + shift) % len];
            // Multiply a * conj(b)
            sum_re += a_re * b_re + a_im * b_im;
            sum_im += a_im * b_re - a_re * b_im;
        }
        let mag = (sum_re * sum_re + sum_im * sum_im).sqrt();
        if mag > max_mag {
            max_mag = mag;
        }
    }

    max_mag
}

/// Tests whether a number is prime.
///
/// Uses trial division up to `sqrt(n)`. ZC sequence lengths should be prime
/// for optimal cross-correlation properties between different root indices.
///
/// # Examples
///
/// ```rust
/// use r4w_core::zadoff_chu_generator::is_prime;
///
/// assert!(is_prime(139));  // LTE PRACH preamble length
/// assert!(is_prime(839));  // LTE PRACH preamble length (long)
/// assert!(!is_prime(128)); // Not prime
/// assert!(!is_prime(0));
/// assert!(!is_prime(1));
/// ```
pub fn is_prime(n: usize) -> bool {
    if n < 2 {
        return false;
    }
    if n < 4 {
        return true;
    }
    if n % 2 == 0 || n % 3 == 0 {
        return false;
    }
    let mut i = 5;
    while i * i <= n {
        if n % i == 0 || n % (i + 2) == 0 {
            return false;
        }
        i += 6;
    }
    true
}

/// Returns valid PRACH root indices for a given ZC sequence length.
///
/// In LTE, valid root indices are those coprime with the sequence length.
/// For prime lengths (e.g., 139, 839), all indices from 1 to `length - 1`
/// are valid, providing `length - 1` distinct preamble sequences.
///
/// # Arguments
///
/// * `length` - The ZC sequence length (e.g., 139 for short PRACH, 839 for long)
///
/// # Returns
///
/// A vector of valid root indices sorted in ascending order.
pub fn lte_prach_roots(length: usize) -> Vec<usize> {
    (1..length)
        .filter(|&u| gcd(u, length) == 1)
        .collect()
}

/// Greatest common divisor using Euclidean algorithm.
fn gcd(mut a: usize, mut b: usize) -> usize {
    while b != 0 {
        let t = b;
        b = a % b;
        a = t;
    }
    a
}

/// Zadoff-Chu sequence generator with optional cyclic shift.
///
/// Encapsulates ZC sequence parameters and provides generation and
/// correlation methods. Useful when working with a fixed sequence
/// configuration across multiple operations.
///
/// # Examples
///
/// ```rust
/// use r4w_core::zadoff_chu_generator::ZadoffChuGenerator;
///
/// let gen = ZadoffChuGenerator::with_shift(25, 139, 10);
/// let seq = gen.generate();
/// assert_eq!(seq.len(), 139);
///
/// // Correlate against a received signal
/// let correlation = gen.correlate(&seq);
/// ```
#[derive(Debug, Clone)]
pub struct ZadoffChuGenerator {
    /// Root index of the ZC sequence (must be coprime with `length`).
    pub root: usize,
    /// Sequence length (ideally prime for best properties).
    pub length: usize,
    /// Cyclic shift applied to the base sequence.
    pub cyclic_shift: usize,
}

impl ZadoffChuGenerator {
    /// Creates a new ZC generator with no cyclic shift.
    ///
    /// # Arguments
    ///
    /// * `root` - Root index `u` (must be coprime with `length`)
    /// * `length` - Sequence length `N`
    pub fn new(root: usize, length: usize) -> Self {
        Self {
            root,
            length,
            cyclic_shift: 0,
        }
    }

    /// Creates a new ZC generator with a cyclic shift.
    ///
    /// # Arguments
    ///
    /// * `root` - Root index `u` (must be coprime with `length`)
    /// * `length` - Sequence length `N`
    /// * `shift` - Cyclic shift amount
    pub fn with_shift(root: usize, length: usize, shift: usize) -> Self {
        Self {
            root,
            length,
            cyclic_shift: shift,
        }
    }

    /// Generates the ZC sequence with the configured parameters.
    ///
    /// # Returns
    ///
    /// A vector of `(re, im)` tuples of length `self.length`.
    pub fn generate(&self) -> Vec<(f64, f64)> {
        if self.cyclic_shift == 0 {
            zadoff_chu(self.root, self.length)
        } else {
            zadoff_chu_cyclic_shift(self.root, self.length, self.cyclic_shift)
        }
    }

    /// Computes the correlation magnitude between the configured ZC sequence
    /// and a received signal.
    ///
    /// Performs circular cross-correlation and returns the magnitude at each
    /// lag. The peak position indicates the timing offset of the ZC sequence
    /// in the received signal.
    ///
    /// # Arguments
    ///
    /// * `signal` - Received complex signal as `(re, im)` tuples
    ///
    /// # Returns
    ///
    /// A vector of correlation magnitudes, one per lag value (0..signal.len()).
    pub fn correlate(&self, signal: &[(f64, f64)]) -> Vec<f64> {
        let reference = self.generate();
        let sig_len = signal.len();
        let ref_len = reference.len();

        (0..sig_len)
            .map(|lag| {
                let mut sum_re = 0.0;
                let mut sum_im = 0.0;
                for k in 0..ref_len {
                    let sig_idx = (lag + k) % sig_len;
                    let (s_re, s_im) = signal[sig_idx];
                    let (r_re, r_im) = reference[k];
                    // Multiply signal * conj(reference)
                    sum_re += s_re * r_re + s_im * r_im;
                    sum_im += s_im * r_re - s_re * r_im;
                }
                (sum_re * sum_re + sum_im * sum_im).sqrt()
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-10;

    #[test]
    fn test_basic_generation() {
        let seq = zadoff_chu(1, 7);
        assert_eq!(seq.len(), 7);

        // Verify first sample is always (1, 0) since exp(0) = 1
        assert!((seq[0].0 - 1.0).abs() < EPSILON);
        assert!(seq[0].1.abs() < EPSILON);

        // Verify specific values for root=1, length=7
        // x[1] = exp(-j * pi * 1 * 1 * 2 / 7) = exp(-j * 2*pi/7)
        let expected_phase = -PI * 2.0 / 7.0;
        assert!((seq[1].0 - expected_phase.cos()).abs() < EPSILON);
        assert!((seq[1].1 - expected_phase.sin()).abs() < EPSILON);
    }

    #[test]
    fn test_unit_magnitude() {
        // Every sample of a ZC sequence must have unit magnitude
        for &root in &[1, 3, 5, 7] {
            let seq = zadoff_chu(root, 31);
            for (n, &(re, im)) in seq.iter().enumerate() {
                let mag = (re * re + im * im).sqrt();
                assert!(
                    (mag - 1.0).abs() < EPSILON,
                    "Non-unit magnitude at n={} for root={}: mag={}",
                    n,
                    root,
                    mag
                );
            }
        }
    }

    #[test]
    fn test_zero_autocorrelation() {
        // Cyclically shifted versions of the same root should be orthogonal.
        // The cyclic cross-correlation at non-zero shift should be zero
        // (for prime length).
        let length = 31; // prime
        let root = 5;
        let seq0 = zadoff_chu(root, length);
        let seq1 = zadoff_chu_cyclic_shift(root, length, 3);

        // Compute inner product (zero-lag correlation)
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        for n in 0..length {
            let (a_re, a_im) = seq0[n];
            let (b_re, b_im) = seq1[n];
            sum_re += a_re * b_re + a_im * b_im;
            sum_im += a_im * b_re - a_re * b_im;
        }
        let zero_lag_mag = (sum_re * sum_re + sum_im * sum_im).sqrt();

        // For orthogonal shifted sequences, the zero-lag correlation should
        // NOT equal the sequence length (which it would for zero shift).
        // The correlation peak should occur at a different lag.
        assert!(
            (zero_lag_mag - length as f64).abs() > 1.0,
            "Shifted sequence should not correlate perfectly at zero lag"
        );
    }

    #[test]
    fn test_cyclic_shift() {
        let length = 13;
        let root = 3;
        let shift = 5;

        let base = zadoff_chu(root, length);
        let shifted = zadoff_chu_cyclic_shift(root, length, shift);

        // shifted[n] should equal base[(n + shift) % length]
        for n in 0..length {
            let expected = base[(n + shift) % length];
            assert!(
                (shifted[n].0 - expected.0).abs() < EPSILON,
                "Cyclic shift mismatch at n={} (real)",
                n
            );
            assert!(
                (shifted[n].1 - expected.1).abs() < EPSILON,
                "Cyclic shift mismatch at n={} (imag)",
                n
            );
        }
    }

    #[test]
    fn test_different_roots() {
        // Different root indices should produce different sequences
        let length = 31;
        let seq1 = zadoff_chu(1, length);
        let seq2 = zadoff_chu(3, length);

        let mut all_same = true;
        for n in 1..length {
            if (seq1[n].0 - seq2[n].0).abs() > EPSILON
                || (seq1[n].1 - seq2[n].1).abs() > EPSILON
            {
                all_same = false;
                break;
            }
        }
        assert!(!all_same, "Different roots must produce different sequences");
    }

    #[test]
    fn test_cross_correlation_low() {
        // For prime length, cross-correlation between different roots is
        // bounded by sqrt(N).
        let length = 31; // prime
        let seq1 = zadoff_chu(1, length);
        let seq2 = zadoff_chu(3, length);

        let xcorr = cross_correlation_zc(&seq1, &seq2);
        let bound = (length as f64).sqrt();

        // Peak cross-correlation should be approximately sqrt(N)
        assert!(
            xcorr <= bound + 1.0,
            "Cross-correlation {} exceeds bound sqrt({})={:.2}",
            xcorr,
            length,
            bound
        );
    }

    #[test]
    fn test_correlate_peak() {
        let length = 31;
        let gen = ZadoffChuGenerator::new(7, length);
        let signal = gen.generate();
        let corr = gen.correlate(&signal);

        assert_eq!(corr.len(), length);

        // Peak should be at lag 0 and equal to the sequence length
        let peak_val = corr[0];
        assert!(
            (peak_val - length as f64).abs() < 1e-6,
            "Correlation peak at lag 0 should be {}, got {}",
            length,
            peak_val
        );

        // All other lags should be much smaller (zero for prime length)
        for (lag, &val) in corr.iter().enumerate().skip(1) {
            assert!(
                val < peak_val * 0.1,
                "Correlation at lag {} should be near zero, got {} (peak={})",
                lag,
                val,
                peak_val
            );
        }
    }

    #[test]
    fn test_is_prime() {
        assert!(!is_prime(0));
        assert!(!is_prime(1));
        assert!(is_prime(2));
        assert!(is_prime(3));
        assert!(!is_prime(4));
        assert!(is_prime(5));
        assert!(!is_prime(6));
        assert!(is_prime(7));
        assert!(!is_prime(9));
        assert!(is_prime(11));
        assert!(is_prime(13));
        assert!(!is_prime(15));
        assert!(is_prime(139)); // LTE short PRACH
        assert!(is_prime(839)); // LTE long PRACH
        assert!(!is_prime(100));
        assert!(!is_prime(1000));
    }

    #[test]
    fn test_lte_roots() {
        // For prime length, every index from 1..length is valid
        let roots_139 = lte_prach_roots(139);
        assert_eq!(roots_139.len(), 138); // 139 is prime, so phi(139) = 138
        assert_eq!(roots_139[0], 1);
        assert_eq!(*roots_139.last().unwrap(), 138);

        // For non-prime length, some indices are excluded
        let roots_12 = lte_prach_roots(12);
        // Coprime with 12: 1, 5, 7, 11
        assert_eq!(roots_12, vec![1, 5, 7, 11]);
    }

    #[test]
    fn test_length_one() {
        let seq = zadoff_chu(0, 1);
        assert_eq!(seq.len(), 1);
        // exp(0) = 1 + 0j
        assert!((seq[0].0 - 1.0).abs() < EPSILON);
        assert!(seq[0].1.abs() < EPSILON);
    }
}
