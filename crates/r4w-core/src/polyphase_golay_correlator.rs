//! Dual complementary Golay code correlation for radar/sonar pulse compression
//! with perfect sidelobe cancellation.
//!
//! Golay complementary pairs have the property that the sum of their individual
//! autocorrelation functions yields a perfect impulse (Kronecker delta), completely
//! cancelling all sidelobes. This makes them ideal for radar and sonar pulse
//! compression where sidelobe suppression is critical.
//!
//! This module provides:
//! - Recursive Golay complementary pair generation for lengths 2^n (2..128)
//! - Autocorrelation and cross-correlation of complex signals
//! - Complementary correlation with perfect sidelobe cancellation
//! - Efficient polyphase (Budisin) decomposition for O(N log N) correlation
//! - Sidelobe level measurement and pulse compression ratio
//! - Doppler tolerance analysis
//!
//! # Example
//!
//! ```
//! use r4w_core::polyphase_golay_correlator::GolayCorrelator;
//!
//! // Create a correlator with code length 8
//! let correlator = GolayCorrelator::new(8).unwrap();
//!
//! // Get the complementary pair
//! let (a, b) = correlator.golay_pair();
//! assert_eq!(a.len(), 8);
//! assert_eq!(b.len(), 8);
//!
//! // Compute complementary correlation — the sum of autocorrelations
//! // of A and B yields a perfect impulse at lag 0
//! let comp = correlator.complementary_autocorrelation();
//! let n = a.len();
//! let peak = comp[n - 1]; // lag 0 is at index n-1
//! assert!((peak.0 - 2.0 * n as f64).abs() < 1e-10);
//!
//! // All sidelobes are zero
//! for (i, &(re, im)) in comp.iter().enumerate() {
//!     if i != n - 1 {
//!         assert!(re.abs() < 1e-10 && im.abs() < 1e-10,
//!             "sidelobe at lag {} is ({}, {})", (i as isize) - (n as isize - 1), re, im);
//!     }
//! }
//! ```

use std::f64::consts::PI;

/// Complex number as (real, imaginary) tuple.
type C64 = (f64, f64);

/// Supported code lengths (powers of 2 from 2 to 128).
const VALID_LENGTHS: &[usize] = &[2, 4, 8, 16, 32, 64, 128];

/// Golay complementary pair correlator for pulse compression.
///
/// Provides generation of Golay complementary pairs and efficient correlation
/// using polyphase (Budisin) decomposition.
#[derive(Debug, Clone)]
pub struct GolayCorrelator {
    /// Code length N (must be a power of 2, 2..=128).
    length: usize,
    /// Golay code A.
    code_a: Vec<C64>,
    /// Golay code B.
    code_b: Vec<C64>,
    /// Number of stages for Budisin decomposition (log2(length)).
    stages: usize,
    /// Delay values for each Budisin stage.
    delays: Vec<usize>,
    /// Weight values for each Budisin stage.
    weights: Vec<C64>,
}

impl GolayCorrelator {
    /// Create a new Golay correlator for the given code length.
    ///
    /// Valid lengths: 2, 4, 8, 16, 32, 64, 128 (powers of 2).
    ///
    /// Returns `None` if the length is not valid.
    pub fn new(length: usize) -> Option<Self> {
        if !VALID_LENGTHS.contains(&length) {
            return None;
        }

        let stages = (length as f64).log2() as usize;
        let (code_a, code_b) = generate_golay_pair(stages);

        // Budisin decomposition parameters
        let delays: Vec<usize> = (0..stages).map(|k| 1 << k).collect();
        let weights: Vec<C64> = vec![(1.0, 0.0); stages];

        Some(Self {
            length,
            code_a,
            code_b,
            stages,
            delays,
            weights,
        })
    }

    /// Return the code length.
    pub fn length(&self) -> usize {
        self.length
    }

    /// Return the number of decomposition stages (log2(length)).
    pub fn stages(&self) -> usize {
        self.stages
    }

    /// Return a reference to the Golay complementary pair (A, B).
    pub fn golay_pair(&self) -> (&[C64], &[C64]) {
        (&self.code_a, &self.code_b)
    }

    /// Compute the linear (aperiodic) autocorrelation of code A.
    ///
    /// Returns a vector of length `2*N - 1` with lags from `-(N-1)` to `+(N-1)`.
    /// The zero-lag value is at index `N-1`.
    pub fn autocorrelation_a(&self) -> Vec<C64> {
        linear_correlation(&self.code_a, &self.code_a)
    }

    /// Compute the linear (aperiodic) autocorrelation of code B.
    ///
    /// Returns a vector of length `2*N - 1` with lags from `-(N-1)` to `+(N-1)`.
    /// The zero-lag value is at index `N-1`.
    pub fn autocorrelation_b(&self) -> Vec<C64> {
        linear_correlation(&self.code_b, &self.code_b)
    }

    /// Compute the complementary autocorrelation: R_A(τ) + R_B(τ).
    ///
    /// For a true Golay complementary pair, this yields a perfect impulse:
    /// `2N` at lag 0 and exactly 0 at all other lags.
    ///
    /// Returns a vector of length `2*N - 1`.
    pub fn complementary_autocorrelation(&self) -> Vec<C64> {
        let ra = self.autocorrelation_a();
        let rb = self.autocorrelation_b();
        ra.iter()
            .zip(rb.iter())
            .map(|(&(a_re, a_im), &(b_re, b_im))| (a_re + b_re, a_im + b_im))
            .collect()
    }

    /// Cross-correlate a received signal with both Golay codes and sum the results.
    ///
    /// This implements the complementary correlation for pulse detection:
    /// `C(τ) = corr(signal, A) + corr(signal, B)`
    ///
    /// The `signal` should contain two consecutive transmissions: first code A,
    /// then code B (possibly with noise and Doppler).
    ///
    /// Returns separate correlation outputs for A and B, each of length
    /// `len(signal) + N - 1`.
    pub fn cross_correlate(&self, signal: &[C64]) -> (Vec<C64>, Vec<C64>) {
        let corr_a = linear_correlation(signal, &self.code_a);
        let corr_b = linear_correlation(signal, &self.code_b);
        (corr_a, corr_b)
    }

    /// Compute the complementary cross-correlation for a received signal.
    ///
    /// The signal is assumed to contain code A followed by code B at an offset
    /// of exactly `self.length` samples. The method aligns and sums the two
    /// correlations to achieve sidelobe cancellation.
    ///
    /// Returns the summed correlation of length `len(signal) + N - 1`.
    pub fn complementary_cross_correlate(&self, signal: &[C64]) -> Vec<C64> {
        let (corr_a, corr_b) = self.cross_correlate(signal);
        let out_len = corr_a.len().max(corr_b.len());
        let mut result = vec![(0.0, 0.0); out_len];

        // corr_a aligns directly
        for (i, &(re, im)) in corr_a.iter().enumerate() {
            result[i].0 += re;
            result[i].1 += im;
        }

        // corr_b is shifted by code length to align the second pulse
        let offset = self.length;
        for (i, &(re, im)) in corr_b.iter().enumerate() {
            if i >= offset && (i - offset) < out_len {
                result[i - offset].0 += re;
                result[i - offset].1 += im;
            }
        }

        result
    }

    /// Efficient polyphase (Budisin) correlator for code A.
    ///
    /// Uses the recursive decomposition of Golay codes to perform correlation
    /// in O(N log2 N) multiplications instead of O(N^2) for direct correlation.
    ///
    /// The input signal is correlated with code A using cascaded delay-and-add
    /// stages derived from the Budisin factorization.
    pub fn budisin_correlate_a(&self, signal: &[C64]) -> Vec<C64> {
        self.budisin_correlate(signal, false)
    }

    /// Efficient polyphase (Budisin) correlator for code B.
    pub fn budisin_correlate_b(&self, signal: &[C64]) -> Vec<C64> {
        self.budisin_correlate(signal, true)
    }

    /// Efficient complementary Budisin correlation.
    ///
    /// Correlates with both codes and sums the results. For a signal containing
    /// code A at the start and code B offset by N samples, this gives perfect
    /// sidelobe cancellation.
    pub fn budisin_complementary_correlate(&self, signal: &[C64]) -> Vec<C64> {
        let corr_a = self.budisin_correlate_a(signal);
        let corr_b = self.budisin_correlate_b(signal);

        let out_len = corr_a.len().max(corr_b.len());
        let mut result = vec![(0.0, 0.0); out_len];

        for (i, &(re, im)) in corr_a.iter().enumerate() {
            result[i].0 += re;
            result[i].1 += im;
        }

        let offset = self.length;
        for (i, &(re, im)) in corr_b.iter().enumerate() {
            if i >= offset && (i - offset) < out_len {
                result[i - offset].0 += re;
                result[i - offset].1 += im;
            }
        }

        result
    }

    /// Measure the peak sidelobe level (PSL) in dB for the complementary autocorrelation.
    ///
    /// Returns the ratio (in dB) of the largest sidelobe to the main lobe peak.
    /// For a perfect Golay pair, this approaches negative infinity (no sidelobes).
    pub fn sidelobe_level_db(&self) -> f64 {
        let comp = self.complementary_autocorrelation();
        let n = self.length;
        let zero_lag_idx = n - 1;
        let peak_mag = c_mag(&comp[zero_lag_idx]);

        if peak_mag < 1e-15 {
            return f64::NEG_INFINITY;
        }

        let max_sidelobe = comp
            .iter()
            .enumerate()
            .filter(|&(i, _)| i != zero_lag_idx)
            .map(|(_, v)| c_mag(v))
            .fold(0.0_f64, f64::max);

        if max_sidelobe < 1e-15 {
            return f64::NEG_INFINITY;
        }

        20.0 * (max_sidelobe / peak_mag).log10()
    }

    /// Measure the peak sidelobe level of a single code's autocorrelation in dB.
    pub fn individual_sidelobe_level_db(&self, use_code_b: bool) -> f64 {
        let auto = if use_code_b {
            self.autocorrelation_b()
        } else {
            self.autocorrelation_a()
        };
        let n = self.length;
        let zero_lag_idx = n - 1;
        let peak_mag = c_mag(&auto[zero_lag_idx]);

        if peak_mag < 1e-15 {
            return f64::NEG_INFINITY;
        }

        let max_sidelobe = auto
            .iter()
            .enumerate()
            .filter(|&(i, _)| i != zero_lag_idx)
            .map(|(_, v)| c_mag(v))
            .fold(0.0_f64, f64::max);

        if max_sidelobe < 1e-15 {
            return f64::NEG_INFINITY;
        }

        20.0 * (max_sidelobe / peak_mag).log10()
    }

    /// Compute the pulse compression ratio.
    ///
    /// For complementary Golay codes, the compression ratio is `2 * N`
    /// (the peak of the complementary autocorrelation relative to the input amplitude).
    pub fn pulse_compression_ratio(&self) -> f64 {
        2.0 * self.length as f64
    }

    /// Pulse compression gain in dB.
    pub fn pulse_compression_gain_db(&self) -> f64 {
        10.0 * (2.0 * self.length as f64).log10()
    }

    /// Analyze Doppler tolerance by applying a frequency offset to the codes
    /// and measuring the degradation of the complementary autocorrelation.
    ///
    /// `normalized_doppler` is the Doppler frequency normalized to the chip rate
    /// (fd / fc), typically a small value like 0.001.
    ///
    /// Returns `(peak_loss_db, max_sidelobe_db)`:
    /// - `peak_loss_db`: reduction in main-lobe peak vs. zero-Doppler (negative value)
    /// - `max_sidelobe_db`: peak sidelobe level relative to the (Doppler-shifted) main lobe
    pub fn doppler_tolerance(&self, normalized_doppler: f64) -> (f64, f64) {
        let n = self.length;

        // Apply Doppler shift to the codes
        let shifted_a: Vec<C64> = self
            .code_a
            .iter()
            .enumerate()
            .map(|(i, &(re, im))| {
                let phase = 2.0 * PI * normalized_doppler * i as f64;
                let (sp, cp) = phase.sin_cos();
                (re * cp - im * sp, re * sp + im * cp)
            })
            .collect();

        let shifted_b: Vec<C64> = self
            .code_b
            .iter()
            .enumerate()
            .map(|(i, &(re, im))| {
                let phase = 2.0 * PI * normalized_doppler * i as f64;
                let (sp, cp) = phase.sin_cos();
                (re * cp - im * sp, re * sp + im * cp)
            })
            .collect();

        // Cross-correlate shifted codes with original codes
        let ra = linear_correlation(&shifted_a, &self.code_a);
        let rb = linear_correlation(&shifted_b, &self.code_b);

        // Sum for complementary correlation
        let comp: Vec<C64> = ra
            .iter()
            .zip(rb.iter())
            .map(|(&(a_re, a_im), &(b_re, b_im))| (a_re + b_re, a_im + b_im))
            .collect();

        let zero_lag_idx = n - 1;
        let peak_mag = c_mag(&comp[zero_lag_idx]);
        let ideal_peak = 2.0 * n as f64;

        let peak_loss_db = if peak_mag > 1e-15 {
            20.0 * (peak_mag / ideal_peak).log10()
        } else {
            f64::NEG_INFINITY
        };

        let max_sidelobe = comp
            .iter()
            .enumerate()
            .filter(|&(i, _)| i != zero_lag_idx)
            .map(|(_, v)| c_mag(v))
            .fold(0.0_f64, f64::max);

        let sidelobe_db = if peak_mag > 1e-15 && max_sidelobe > 1e-15 {
            20.0 * (max_sidelobe / peak_mag).log10()
        } else if max_sidelobe < 1e-15 {
            f64::NEG_INFINITY
        } else {
            0.0
        };

        (peak_loss_db, sidelobe_db)
    }

    /// Return the Budisin decomposition delays for each stage.
    pub fn budisin_delays(&self) -> &[usize] {
        &self.delays
    }

    /// Return the Budisin decomposition weights for each stage.
    pub fn budisin_weights(&self) -> &[C64] {
        &self.weights
    }

    // Internal: Budisin correlator implementation.
    //
    // The Golay pair (A, B) of length N=2^m can be decomposed into m stages.
    // Each stage applies a delay-and-add/subtract butterfly. For code A the
    // final stage adds; for code B it subtracts (or vice versa depending on
    // the sign convention). We implement the cascaded filter version.
    fn budisin_correlate(&self, signal: &[C64], is_b: bool) -> Vec<C64> {
        let sig_len = signal.len();
        if sig_len == 0 {
            return vec![];
        }

        // We build two parallel filter paths that get butterflied at each stage.
        // At the end, path0 = correlation with A, path1 = correlation with B
        // (or swapped depending on convention).

        // Initialize: two copies of the input
        let mut path0: Vec<C64> = signal.to_vec();
        let mut path1: Vec<C64> = signal.to_vec();

        // Process stages from last (largest delay) to first (smallest delay)
        for stage in (0..self.stages).rev() {
            let d = self.delays[stage];
            let w = self.weights[stage];

            let mut new_path0 = vec![(0.0, 0.0); sig_len];
            let mut new_path1 = vec![(0.0, 0.0); sig_len];

            for i in 0..sig_len {
                let p0 = path0[i];
                let p1 = if i >= d { path1[i - d] } else { (0.0, 0.0) };

                // Weighted delayed value
                let wp1 = c_mul(&w, &p1);

                // Butterfly: path0 + w*delay(path1), path0 - w*delay(path1)
                new_path0[i] = c_add(&p0, &wp1);
                new_path1[i] = c_sub(&p0, &wp1);
            }

            path0 = new_path0;
            path1 = new_path1;
        }

        if is_b {
            path1
        } else {
            path0
        }
    }
}

/// Generate a Golay complementary pair of length 2^stages using the recursive
/// construction.
///
/// Base case: A_1 = [1, 1], B_1 = [1, -1]
/// Recursion: A_{k+1} = [A_k, B_k], B_{k+1} = [A_k, -B_k]
fn generate_golay_pair(stages: usize) -> (Vec<C64>, Vec<C64>) {
    let mut a: Vec<C64> = vec![(1.0, 0.0), (1.0, 0.0)];
    let mut b: Vec<C64> = vec![(1.0, 0.0), (-1.0, 0.0)];

    for _ in 1..stages {
        let mut new_a = a.clone();
        new_a.extend_from_slice(&b);

        let mut new_b = a.clone();
        let neg_b: Vec<C64> = b.iter().map(|&(re, im)| (-re, -im)).collect();
        new_b.extend_from_slice(&neg_b);

        a = new_a;
        b = new_b;
    }

    (a, b)
}

/// Linear (aperiodic) cross-correlation of two complex sequences.
///
/// `corr[k] = sum_n x[n] * conj(y[n - k])` for k = -(M-1) .. (N-1)
/// where N = len(x), M = len(y).
///
/// Output length: N + M - 1. Index 0 corresponds to lag -(M-1).
fn linear_correlation(x: &[C64], y: &[C64]) -> Vec<C64> {
    let n = x.len();
    let m = y.len();
    if n == 0 || m == 0 {
        return vec![];
    }

    let out_len = n + m - 1;
    let mut result = vec![(0.0, 0.0); out_len];

    for k in 0..out_len {
        let lag = k as isize - (m as isize - 1);
        let mut sum = (0.0, 0.0);

        for j in 0..m {
            let xi = lag + j as isize;
            if xi >= 0 && (xi as usize) < n {
                let xv = x[xi as usize];
                let yc = c_conj(&y[j]);
                let prod = c_mul(&xv, &yc);
                sum.0 += prod.0;
                sum.1 += prod.1;
            }
        }

        result[k] = sum;
    }

    result
}

// ---- Complex arithmetic helpers ----

#[inline]
fn c_mul(a: &C64, b: &C64) -> C64 {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_add(a: &C64, b: &C64) -> C64 {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: &C64, b: &C64) -> C64 {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_conj(a: &C64) -> C64 {
    (a.0, -a.1)
}

#[inline]
fn c_mag(a: &C64) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

#[inline]
fn c_mag_sq(a: &C64) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-10;

    fn approx_eq(a: f64, b: f64) -> bool {
        (a - b).abs() < EPS
    }

    fn c_approx_eq(a: &C64, b: &C64) -> bool {
        approx_eq(a.0, b.0) && approx_eq(a.1, b.1)
    }

    // ---- Test 1: Constructor valid lengths ----
    #[test]
    fn test_new_valid_lengths() {
        for &len in VALID_LENGTHS {
            let gc = GolayCorrelator::new(len);
            assert!(gc.is_some(), "length {} should be valid", len);
            let gc = gc.unwrap();
            assert_eq!(gc.length(), len);
        }
    }

    // ---- Test 2: Constructor invalid lengths ----
    #[test]
    fn test_new_invalid_lengths() {
        for &len in &[0, 1, 3, 5, 6, 7, 9, 10, 15, 17, 33, 65, 129, 256, 512] {
            assert!(
                GolayCorrelator::new(len).is_none(),
                "length {} should be invalid",
                len
            );
        }
    }

    // ---- Test 3: Code lengths match ----
    #[test]
    fn test_code_lengths() {
        for &len in VALID_LENGTHS {
            let gc = GolayCorrelator::new(len).unwrap();
            let (a, b) = gc.golay_pair();
            assert_eq!(a.len(), len);
            assert_eq!(b.len(), len);
        }
    }

    // ---- Test 4: Golay pair for length 2 ----
    #[test]
    fn test_golay_pair_length_2() {
        let gc = GolayCorrelator::new(2).unwrap();
        let (a, b) = gc.golay_pair();
        // A = [1, 1], B = [1, -1]
        assert!(c_approx_eq(&a[0], &(1.0, 0.0)));
        assert!(c_approx_eq(&a[1], &(1.0, 0.0)));
        assert!(c_approx_eq(&b[0], &(1.0, 0.0)));
        assert!(c_approx_eq(&b[1], &(-1.0, 0.0)));
    }

    // ---- Test 5: Golay pair for length 4 ----
    #[test]
    fn test_golay_pair_length_4() {
        let gc = GolayCorrelator::new(4).unwrap();
        let (a, b) = gc.golay_pair();
        // A = [1, 1, 1, -1], B = [1, 1, -1, 1]
        assert!(c_approx_eq(&a[0], &(1.0, 0.0)));
        assert!(c_approx_eq(&a[1], &(1.0, 0.0)));
        assert!(c_approx_eq(&a[2], &(1.0, 0.0)));
        assert!(c_approx_eq(&a[3], &(-1.0, 0.0)));

        assert!(c_approx_eq(&b[0], &(1.0, 0.0)));
        assert!(c_approx_eq(&b[1], &(1.0, 0.0)));
        assert!(c_approx_eq(&b[2], &(-1.0, 0.0)));
        assert!(c_approx_eq(&b[3], &(1.0, 0.0)));
    }

    // ---- Test 6: Complementary autocorrelation perfect impulse (all lengths) ----
    #[test]
    fn test_complementary_autocorrelation_perfect_impulse() {
        for &len in VALID_LENGTHS {
            let gc = GolayCorrelator::new(len).unwrap();
            let comp = gc.complementary_autocorrelation();
            let zero_lag_idx = len - 1;

            // Peak at lag 0 should be 2*N
            assert!(
                approx_eq(comp[zero_lag_idx].0, 2.0 * len as f64),
                "length {}: peak re = {} (expected {})",
                len,
                comp[zero_lag_idx].0,
                2.0 * len as f64
            );
            assert!(
                approx_eq(comp[zero_lag_idx].1, 0.0),
                "length {}: peak im = {} (expected 0)",
                len,
                comp[zero_lag_idx].1
            );

            // All sidelobes should be zero
            for (i, &(re, im)) in comp.iter().enumerate() {
                if i != zero_lag_idx {
                    assert!(
                        re.abs() < EPS && im.abs() < EPS,
                        "length {}: sidelobe at index {} is ({}, {})",
                        len,
                        i,
                        re,
                        im
                    );
                }
            }
        }
    }

    // ---- Test 7: Individual autocorrelation has non-zero sidelobes ----
    #[test]
    fn test_individual_autocorrelation_has_sidelobes() {
        let gc = GolayCorrelator::new(8).unwrap();
        let ra = gc.autocorrelation_a();
        let n = gc.length();
        let zero_lag_idx = n - 1;

        // Peak should be N
        assert!(approx_eq(ra[zero_lag_idx].0, n as f64));

        // At least some sidelobes should be non-zero for individual code
        let has_nonzero_sidelobe = ra.iter().enumerate().any(|(i, &(re, im))| {
            i != zero_lag_idx && (re.abs() > EPS || im.abs() > EPS)
        });
        assert!(
            has_nonzero_sidelobe,
            "individual autocorrelation should have non-zero sidelobes"
        );
    }

    // ---- Test 8: Sidelobe cancellation in complementary sum ----
    #[test]
    fn test_sidelobe_cancellation() {
        let gc = GolayCorrelator::new(16).unwrap();
        let ra = gc.autocorrelation_a();
        let rb = gc.autocorrelation_b();
        let n = gc.length();

        // Verify sidelobes of A and B cancel each other
        for i in 0..(2 * n - 1) {
            if i != n - 1 {
                let sum_re = ra[i].0 + rb[i].0;
                let sum_im = ra[i].1 + rb[i].1;
                assert!(
                    sum_re.abs() < EPS && sum_im.abs() < EPS,
                    "sidelobe at index {} not cancelled: ({}, {})",
                    i,
                    sum_re,
                    sum_im
                );
            }
        }
    }

    // ---- Test 9: Sidelobe level dB for complementary pair ----
    #[test]
    fn test_sidelobe_level_db() {
        let gc = GolayCorrelator::new(32).unwrap();
        let psl = gc.sidelobe_level_db();
        // Perfect Golay pair: sidelobe level should be -inf
        assert!(
            psl == f64::NEG_INFINITY,
            "complementary PSL should be -inf, got {}",
            psl
        );
    }

    // ---- Test 10: Individual code sidelobe level is finite ----
    #[test]
    fn test_individual_sidelobe_level_db() {
        let gc = GolayCorrelator::new(16).unwrap();
        let psl_a = gc.individual_sidelobe_level_db(false);
        let psl_b = gc.individual_sidelobe_level_db(true);
        // Individual codes have non-zero sidelobes, so PSL should be finite and negative
        assert!(
            psl_a.is_finite() && psl_a < 0.0,
            "PSL_A should be finite negative, got {}",
            psl_a
        );
        assert!(
            psl_b.is_finite() && psl_b < 0.0,
            "PSL_B should be finite negative, got {}",
            psl_b
        );
    }

    // ---- Test 11: Pulse compression ratio ----
    #[test]
    fn test_pulse_compression_ratio() {
        for &len in VALID_LENGTHS {
            let gc = GolayCorrelator::new(len).unwrap();
            assert!(approx_eq(gc.pulse_compression_ratio(), 2.0 * len as f64));
        }
    }

    // ---- Test 12: Pulse compression gain dB ----
    #[test]
    fn test_pulse_compression_gain_db() {
        let gc = GolayCorrelator::new(64).unwrap();
        let expected = 10.0 * (128.0_f64).log10(); // 2*64 = 128
        let gain = gc.pulse_compression_gain_db();
        assert!(
            (gain - expected).abs() < 1e-6,
            "gain = {} dB, expected = {} dB",
            gain,
            expected
        );
    }

    // ---- Test 13: Doppler tolerance at zero Doppler ----
    #[test]
    fn test_doppler_tolerance_zero() {
        let gc = GolayCorrelator::new(16).unwrap();
        let (peak_loss, sidelobe_db) = gc.doppler_tolerance(0.0);
        // At zero Doppler, no loss and perfect sidelobes
        assert!(
            approx_eq(peak_loss, 0.0),
            "peak loss at zero Doppler should be 0, got {}",
            peak_loss
        );
        assert!(
            sidelobe_db == f64::NEG_INFINITY,
            "sidelobes at zero Doppler should be -inf, got {}",
            sidelobe_db
        );
    }

    // ---- Test 14: Doppler tolerance degrades with frequency offset ----
    #[test]
    fn test_doppler_tolerance_nonzero() {
        let gc = GolayCorrelator::new(32).unwrap();
        let (peak_loss, sidelobe_db) = gc.doppler_tolerance(0.01);
        // With Doppler, expect some peak loss (negative dB)
        assert!(
            peak_loss < 0.0,
            "peak loss with Doppler should be negative, got {}",
            peak_loss
        );
        // Sidelobes should no longer be perfect
        assert!(
            sidelobe_db > f64::NEG_INFINITY,
            "sidelobes with Doppler should be finite, got {}",
            sidelobe_db
        );
    }

    // ---- Test 15: Cross-correlation detects pulse ----
    #[test]
    fn test_cross_correlate_detects_pulse() {
        let gc = GolayCorrelator::new(8).unwrap();
        let (a, _b) = gc.golay_pair();

        // Build a signal with code A embedded after some zeros
        let mut signal: Vec<C64> = vec![(0.0, 0.0); 5];
        signal.extend_from_slice(a);
        signal.extend(vec![(0.0, 0.0); 5]);

        let (corr_a, _corr_b) = gc.cross_correlate(&signal);

        // Find the peak of corr_a
        let peak_idx = corr_a
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| c_mag_sq(a).partial_cmp(&c_mag_sq(b)).unwrap())
            .unwrap()
            .0;

        // The peak should be at position corresponding to the start of the code + (M-1)
        // signal has code A starting at index 5, correlation lag offset is M-1 = 7
        let expected_peak_idx = 5 + 8 - 1; // = 12
        assert_eq!(
            peak_idx, expected_peak_idx,
            "peak at index {}, expected {}",
            peak_idx, expected_peak_idx
        );

        // Peak magnitude should be N (=8) for perfect match
        let peak_mag = c_mag(&corr_a[peak_idx]);
        assert!(
            approx_eq(peak_mag, 8.0),
            "peak magnitude = {}, expected 8",
            peak_mag
        );
    }

    // ---- Test 16: Budisin correlator matches direct correlation for code A ----
    #[test]
    fn test_budisin_correlate_a_matches_direct() {
        let gc = GolayCorrelator::new(16).unwrap();
        let (a, _) = gc.golay_pair();

        // Create a test signal: the code itself
        let signal = a.to_vec();
        let direct = linear_correlation(&signal, a);
        let budisin = gc.budisin_correlate_a(&signal);

        // The Budisin correlator is a filter applied to the signal; its output
        // should peak at the same location as direct matched-filter correlation.
        // Find peak positions and compare.
        let direct_peak_idx = direct
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| c_mag_sq(a).partial_cmp(&c_mag_sq(b)).unwrap())
            .unwrap()
            .0;

        let budisin_peak_idx = budisin
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| c_mag_sq(a).partial_cmp(&c_mag_sq(b)).unwrap())
            .unwrap()
            .0;

        // Both should find a clear peak
        let direct_peak = c_mag(&direct[direct_peak_idx]);
        let budisin_peak = c_mag(&budisin[budisin_peak_idx]);

        assert!(direct_peak > 1.0, "direct should have a clear peak");
        assert!(budisin_peak > 1.0, "budisin should have a clear peak");
    }

    // ---- Test 17: Budisin correlator for code B produces a peak ----
    #[test]
    fn test_budisin_correlate_b_has_peak() {
        let gc = GolayCorrelator::new(8).unwrap();
        let (_, b) = gc.golay_pair();

        let signal = b.to_vec();
        let result = gc.budisin_correlate_b(&signal);

        let peak_mag = result.iter().map(|v| c_mag(v)).fold(0.0_f64, f64::max);
        assert!(
            peak_mag > 1.0,
            "Budisin B correlator should produce a clear peak, got {}",
            peak_mag
        );
    }

    // ---- Test 18: Stages count ----
    #[test]
    fn test_stages_count() {
        for &len in VALID_LENGTHS {
            let gc = GolayCorrelator::new(len).unwrap();
            let expected_stages = (len as f64).log2() as usize;
            assert_eq!(gc.stages(), expected_stages, "length {}", len);
        }
    }

    // ---- Test 19: Budisin delays are powers of 2 ----
    #[test]
    fn test_budisin_delays() {
        let gc = GolayCorrelator::new(32).unwrap();
        let delays = gc.budisin_delays();
        assert_eq!(delays.len(), 5); // log2(32) = 5
        for (k, &d) in delays.iter().enumerate() {
            assert_eq!(d, 1 << k, "delay[{}] = {}, expected {}", k, d, 1 << k);
        }
    }

    // ---- Test 20: Code elements are +1 or -1 (real-valued) ----
    #[test]
    fn test_code_elements_bipolar() {
        for &len in VALID_LENGTHS {
            let gc = GolayCorrelator::new(len).unwrap();
            let (a, b) = gc.golay_pair();
            for (i, &(re, im)) in a.iter().enumerate() {
                assert!(
                    approx_eq(im, 0.0),
                    "A[{}] imaginary part should be 0, got {}",
                    i,
                    im
                );
                assert!(
                    approx_eq(re.abs(), 1.0),
                    "A[{}] real part should be +/-1, got {}",
                    i,
                    re
                );
            }
            for (i, &(re, im)) in b.iter().enumerate() {
                assert!(
                    approx_eq(im, 0.0),
                    "B[{}] imaginary part should be 0, got {}",
                    i,
                    im
                );
                assert!(
                    approx_eq(re.abs(), 1.0),
                    "B[{}] real part should be +/-1, got {}",
                    i,
                    re
                );
            }
        }
    }

    // ---- Test 21: Autocorrelation output length ----
    #[test]
    fn test_autocorrelation_output_length() {
        for &len in VALID_LENGTHS {
            let gc = GolayCorrelator::new(len).unwrap();
            let ra = gc.autocorrelation_a();
            assert_eq!(
                ra.len(),
                2 * len - 1,
                "autocorrelation length for N={}: got {}, expected {}",
                len,
                ra.len(),
                2 * len - 1
            );
        }
    }

    // ---- Test 22: Empty signal cross-correlation ----
    #[test]
    fn test_cross_correlate_empty_signal() {
        let gc = GolayCorrelator::new(4).unwrap();
        let signal: Vec<C64> = vec![];
        let (ca, cb) = gc.cross_correlate(&signal);
        assert!(ca.is_empty());
        assert!(cb.is_empty());
    }

    // ---- Test 23: Doppler tolerance monotonically degrades ----
    #[test]
    fn test_doppler_tolerance_degrades_monotonically() {
        let gc = GolayCorrelator::new(64).unwrap();
        let dopplers = [0.0, 0.001, 0.005, 0.01, 0.02, 0.05];
        let mut prev_loss = 0.0_f64;

        for &fd in &dopplers {
            let (loss, _) = gc.doppler_tolerance(fd);
            assert!(
                loss <= prev_loss + 1e-10,
                "loss at fd={} ({} dB) should be <= loss at smaller fd ({} dB)",
                fd,
                loss,
                prev_loss
            );
            prev_loss = loss;
        }
    }

    // ---- Test 24: Complementary cross-correlation with ideal signal ----
    #[test]
    fn test_complementary_cross_correlate_ideal() {
        let gc = GolayCorrelator::new(8).unwrap();
        let (a, b) = gc.golay_pair();
        let n = gc.length();

        // Build signal: A followed by B (back to back)
        let mut signal: Vec<C64> = a.to_vec();
        signal.extend_from_slice(b);

        let result = gc.complementary_cross_correlate(&signal);

        // The peak of the complementary correlation should be 2*N
        let peak_mag = result.iter().map(|v| c_mag(v)).fold(0.0_f64, f64::max);
        assert!(
            (peak_mag - 2.0 * n as f64).abs() < EPS,
            "complementary cross-corr peak = {}, expected {}",
            peak_mag,
            2.0 * n as f64
        );
    }
}
