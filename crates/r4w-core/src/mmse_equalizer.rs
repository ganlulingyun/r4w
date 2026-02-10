//! # MMSE Frequency-Domain Equalizer
//!
//! Minimum Mean Square Error (MMSE) equalizer for multipath channel compensation
//! in OFDM and single-carrier systems. Also provides a Zero-Forcing (ZF) equalizer
//! for comparison.
//!
//! The MMSE equalizer computes per-subcarrier weights as:
//!
//! ```text
//! W[k] = H*[k] / (|H[k]|^2 + sigma^2)
//! ```
//!
//! where `H[k]` is the channel frequency response at subcarrier `k` and `sigma^2`
//! is the noise variance. This balances noise enhancement against residual ISI,
//! unlike the ZF equalizer which simply inverts the channel (`Y/H`).
//!
//! # Example
//!
//! ```rust
//! use r4w_core::mmse_equalizer::{MmseEqualizer, mse_metric};
//!
//! // 4 subcarriers with a known channel
//! let eq = MmseEqualizer::new(4);
//! let channel = vec![(1.0, 0.0), (0.5, 0.5), (0.0, 1.0), (-0.5, 0.5)];
//! let transmitted = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
//!
//! // Simulate received = transmitted * channel (element-wise)
//! let received: Vec<(f64, f64)> = transmitted.iter().zip(channel.iter())
//!     .map(|(&(tr, ti), &(hr, hi))| (tr * hr - ti * hi, tr * hi + ti * hr))
//!     .collect();
//!
//! let equalized = eq.equalize(&received, &channel, 0.01);
//! let mse = mse_metric(&equalized, &transmitted);
//! assert!(mse < 0.01, "MSE should be small for good equalization");
//! ```

/// Complex multiplication: (a + jb) * (c + jd)
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex conjugate
#[inline]
fn conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Squared magnitude |a|^2
#[inline]
fn mag_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Complex division: a / b
#[inline]
fn cdiv(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = mag_sq(b);
    if denom < 1e-30 {
        (0.0, 0.0)
    } else {
        let num = cmul(a, conj(b));
        (num.0 / denom, num.1 / denom)
    }
}

// ---------------------------------------------------------------------------
// MMSE Equalizer (frequency domain)
// ---------------------------------------------------------------------------

/// MMSE frequency-domain equalizer for OFDM and block-based systems.
///
/// Computes per-subcarrier weights: `W[k] = H*[k] / (|H[k]|^2 + sigma^2 + reg)`
/// where `reg` is an optional diagonal-loading regularization term.
pub struct MmseEqualizer {
    num_subcarriers: usize,
    regularization: f64,
}

impl MmseEqualizer {
    /// Create a new MMSE equalizer for the given number of subcarriers.
    pub fn new(num_subcarriers: usize) -> Self {
        Self {
            num_subcarriers,
            regularization: 0.0,
        }
    }

    /// Apply MMSE equalization to received symbols given a channel estimate and
    /// noise variance.
    ///
    /// # Panics
    ///
    /// Panics if `received` and `channel_estimate` lengths differ or do not match
    /// `num_subcarriers`.
    pub fn equalize(
        &self,
        received: &[(f64, f64)],
        channel_estimate: &[(f64, f64)],
        noise_variance: f64,
    ) -> Vec<(f64, f64)> {
        assert_eq!(received.len(), self.num_subcarriers);
        assert_eq!(channel_estimate.len(), self.num_subcarriers);

        let weights = mmse_weights_with_reg(channel_estimate, noise_variance, self.regularization);
        received
            .iter()
            .zip(weights.iter())
            .map(|(&r, &w)| cmul(w, r))
            .collect()
    }

    /// Set diagonal-loading regularization for numerical stability.
    ///
    /// A small positive value (e.g., 1e-6) prevents division by near-zero
    /// denominators on deeply-faded subcarriers.
    pub fn set_regularization(&mut self, reg: f64) {
        self.regularization = reg;
    }
}

// ---------------------------------------------------------------------------
// ZF Equalizer (frequency domain)
// ---------------------------------------------------------------------------

/// Zero-Forcing frequency-domain equalizer.
///
/// Computes `Y[k] / H[k]` per subcarrier. Optimal in the absence of noise but
/// amplifies noise on deeply-faded subcarriers.
pub struct ZfEqualizer {
    num_subcarriers: usize,
}

impl ZfEqualizer {
    /// Create a new ZF equalizer for the given number of subcarriers.
    pub fn new(num_subcarriers: usize) -> Self {
        Self { num_subcarriers }
    }

    /// Apply ZF equalization: `equalized[k] = received[k] / channel_estimate[k]`.
    ///
    /// # Panics
    ///
    /// Panics if `received` and `channel_estimate` lengths differ or do not match
    /// `num_subcarriers`.
    pub fn equalize(
        &self,
        received: &[(f64, f64)],
        channel_estimate: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        assert_eq!(received.len(), self.num_subcarriers);
        assert_eq!(channel_estimate.len(), self.num_subcarriers);

        received
            .iter()
            .zip(channel_estimate.iter())
            .map(|(&r, &h)| cdiv(r, h))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// Internal helper: MMSE weights with explicit regularization.
fn mmse_weights_with_reg(
    channel: &[(f64, f64)],
    noise_variance: f64,
    regularization: f64,
) -> Vec<(f64, f64)> {
    channel
        .iter()
        .map(|&h| {
            let denom = mag_sq(h) + noise_variance + regularization;
            if denom < 1e-30 {
                (0.0, 0.0)
            } else {
                let hc = conj(h);
                (hc.0 / denom, hc.1 / denom)
            }
        })
        .collect()
}

/// Compute MMSE equalizer weights for a given channel frequency response and
/// noise variance.
///
/// `W[k] = H*[k] / (|H[k]|^2 + noise_variance)`
pub fn mmse_weights(channel: &[(f64, f64)], noise_variance: f64) -> Vec<(f64, f64)> {
    mmse_weights_with_reg(channel, noise_variance, 0.0)
}

/// Compute Zero-Forcing equalizer weights for a given channel frequency response.
///
/// `W[k] = H*[k] / |H[k]|^2 = 1 / H[k]`
pub fn zf_weights(channel: &[(f64, f64)]) -> Vec<(f64, f64)> {
    channel
        .iter()
        .map(|&h| {
            let ms = mag_sq(h);
            if ms < 1e-30 {
                (0.0, 0.0)
            } else {
                let hc = conj(h);
                (hc.0 / ms, hc.1 / ms)
            }
        })
        .collect()
}

/// Compute the post-equalization SINR per subcarrier for MMSE equalization.
///
/// `SINR[k] = |H[k]|^2 / noise_variance`
///
/// This is the matched-filter bound; MMSE equalization approaches this SINR.
pub fn sinr_per_subcarrier(channel: &[(f64, f64)], noise_variance: f64) -> Vec<f64> {
    channel
        .iter()
        .map(|&h| {
            if noise_variance < 1e-30 {
                f64::INFINITY
            } else {
                mag_sq(h) / noise_variance
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Time-Domain MMSE Equalizer (single-carrier)
// ---------------------------------------------------------------------------

/// Time-domain MMSE equalizer for single-carrier systems.
///
/// Computes FIR equalizer taps via the Wiener-Hopf equation:
///
/// ```text
/// w = R^{-1} p
/// ```
///
/// where `R` is the autocorrelation matrix of the received signal (channel
/// convolution + noise) and `p` is the cross-correlation vector.
pub struct TimeDomainMmse {
    num_taps: usize,
}

impl TimeDomainMmse {
    /// Create a new time-domain MMSE equalizer with the given number of FIR taps.
    pub fn new(num_taps: usize) -> Self {
        Self { num_taps }
    }

    /// Compute MMSE-FIR equalizer taps via the Wiener-Hopf equation.
    ///
    /// `channel_ir` is the real-valued channel impulse response.
    /// `noise_variance` is the additive noise power.
    ///
    /// Returns a vector of `num_taps` FIR coefficients.
    pub fn compute_taps(&self, channel_ir: &[f64], noise_variance: f64) -> Vec<f64> {
        let n = self.num_taps;
        let h_len = channel_ir.len();

        // Build autocorrelation matrix R = H^T H + sigma^2 I
        // R[i][j] = sum_k h[k-i]*h[k-j] + sigma^2 * delta(i,j)
        // where indices are clamped to valid range.
        let mut r_matrix = vec![vec![0.0; n]; n];
        let mut p_vector = vec![0.0; n];

        // The desired output is a delayed delta; choose delay = (num_taps - 1) / 2
        // to centre the equalizer.
        let delay = (n - 1) / 2;

        // Autocorrelation of the channel: r_hh[lag] = sum_k h[k] * h[k + lag]
        let max_lag = n + h_len;
        let mut r_hh = vec![0.0; max_lag];
        for lag in 0..max_lag {
            let mut sum = 0.0;
            for k in 0..h_len {
                if k + lag < h_len {
                    sum += channel_ir[k] * channel_ir[k + lag];
                }
            }
            r_hh[lag] = sum;
        }

        // Build the Toeplitz-like R matrix
        for i in 0..n {
            for j in 0..n {
                let lag = if i >= j { i - j } else { j - i };
                r_matrix[i][j] = if lag < r_hh.len() { r_hh[lag] } else { 0.0 };
                if i == j {
                    r_matrix[i][j] += noise_variance;
                }
            }
        }

        // Cross-correlation p[i] = h[delay - i] (if in range)
        for i in 0..n {
            let idx = delay as isize - i as isize;
            if idx >= 0 && (idx as usize) < h_len {
                p_vector[i] = channel_ir[idx as usize];
            }
        }

        // Solve R * w = p via Gaussian elimination with partial pivoting
        solve_linear_system(&r_matrix, &p_vector)
    }

    /// Apply FIR equalizer taps to a received real-valued signal.
    ///
    /// Performs direct-form convolution: `y[n] = sum_k taps[k] * received[n - k]`.
    pub fn equalize(&self, taps: &[f64], received: &[f64]) -> Vec<f64> {
        let n_taps = taps.len();
        let n_out = received.len();
        let mut output = vec![0.0; n_out];

        for i in 0..n_out {
            let mut sum = 0.0;
            for (k, &tap) in taps.iter().enumerate() {
                if i >= k {
                    sum += tap * received[i - k];
                }
            }
            output[i] = sum;
        }

        output
    }
}

/// Solve a linear system Ax = b using Gaussian elimination with partial pivoting.
///
/// Returns the solution vector x.
fn solve_linear_system(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let n = b.len();
    // Augmented matrix
    let mut aug: Vec<Vec<f64>> = a
        .iter()
        .enumerate()
        .map(|(i, row)| {
            let mut r = row.clone();
            r.push(b[i]);
            r
        })
        .collect();

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_row = col;
        let mut max_val = aug[col][col].abs();
        for row in (col + 1)..n {
            if aug[row][col].abs() > max_val {
                max_val = aug[row][col].abs();
                max_row = row;
            }
        }
        aug.swap(col, max_row);

        let pivot = aug[col][col];
        if pivot.abs() < 1e-30 {
            continue; // Singular or near-singular
        }

        for row in (col + 1)..n {
            let factor = aug[row][col] / pivot;
            for j in col..=n {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = aug[i][n];
        for j in (i + 1)..n {
            sum -= aug[i][j] * x[j];
        }
        if aug[i][i].abs() < 1e-30 {
            x[i] = 0.0;
        } else {
            x[i] = sum / aug[i][i];
        }
    }

    x
}

// ---------------------------------------------------------------------------
// Metrics
// ---------------------------------------------------------------------------

/// Compute the Mean Square Error between equalized and reference complex symbols.
///
/// `MSE = (1/N) * sum_k |equalized[k] - reference[k]|^2`
///
/// # Panics
///
/// Panics if `equalized` and `reference` have different lengths or are empty.
pub fn mse_metric(equalized: &[(f64, f64)], reference: &[(f64, f64)]) -> f64 {
    assert_eq!(equalized.len(), reference.len());
    assert!(!equalized.is_empty());

    let sum: f64 = equalized
        .iter()
        .zip(reference.iter())
        .map(|(&(er, ei), &(rr, ri))| {
            let dr = er - rr;
            let di = ei - ri;
            dr * dr + di * di
        })
        .sum();

    sum / equalized.len() as f64
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: element-wise complex multiplication of two slices.
    fn channel_apply(tx: &[(f64, f64)], h: &[(f64, f64)]) -> Vec<(f64, f64)> {
        tx.iter().zip(h.iter()).map(|(&t, &h)| cmul(t, h)).collect()
    }

    /// Helper: add complex AWGN (deterministic "noise" for reproducibility).
    fn add_noise(signal: &[(f64, f64)], noise: &[(f64, f64)]) -> Vec<(f64, f64)> {
        signal
            .iter()
            .zip(noise.iter())
            .map(|(&(sr, si), &(nr, ni))| (sr + nr, si + ni))
            .collect()
    }

    #[test]
    fn test_mmse_flat_channel_identity() {
        // Flat channel H = [1, 1, 1, 1] should return received symbols unchanged.
        let eq = MmseEqualizer::new(4);
        let channel = vec![(1.0, 0.0); 4];
        let transmitted = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        let received = channel_apply(&transmitted, &channel);

        let equalized = eq.equalize(&received, &channel, 0.001);
        let mse = mse_metric(&equalized, &transmitted);
        assert!(mse < 1e-6, "Flat channel MSE should be near zero, got {}", mse);
    }

    #[test]
    fn test_mmse_frequency_selective() {
        // Frequency-selective channel with varying gains.
        let n = 8;
        let eq = MmseEqualizer::new(n);
        let channel: Vec<(f64, f64)> = (0..n)
            .map(|k| {
                let phase = std::f64::consts::PI * k as f64 / n as f64;
                (phase.cos() * 0.8 + 0.2, phase.sin() * 0.3)
            })
            .collect();

        let transmitted: Vec<(f64, f64)> = (0..n)
            .map(|k| if k % 2 == 0 { (1.0, 0.0) } else { (-1.0, 0.0) })
            .collect();

        let received = channel_apply(&transmitted, &channel);
        let equalized = eq.equalize(&received, &channel, 0.01);
        let mse = mse_metric(&equalized, &transmitted);
        assert!(mse < 0.05, "Frequency-selective MSE should be small, got {}", mse);
    }

    #[test]
    fn test_mmse_better_than_zf_low_snr() {
        // At low SNR, MMSE should outperform ZF due to noise regularization.
        let n = 8;
        let mmse_eq = MmseEqualizer::new(n);
        let zf_eq = ZfEqualizer::new(n);

        // Channel with a weak subcarrier to stress ZF.
        let channel: Vec<(f64, f64)> = (0..n)
            .map(|k| {
                if k == 3 {
                    (0.05, 0.0) // Very weak subcarrier
                } else {
                    (1.0, 0.0)
                }
            })
            .collect();

        let transmitted: Vec<(f64, f64)> = vec![(1.0, 0.0); n];

        let received_clean = channel_apply(&transmitted, &channel);
        // Add deterministic noise.
        let noise: Vec<(f64, f64)> = (0..n)
            .map(|k| {
                let v = 0.1 * ((k as f64 * 1.7).sin());
                (v, v * 0.5)
            })
            .collect();
        let received = add_noise(&received_clean, &noise);

        let noise_var = 0.01; // Matches noise power order of magnitude.
        let mmse_out = mmse_eq.equalize(&received, &channel, noise_var);
        let zf_out = zf_eq.equalize(&received, &channel);

        let mmse_mse = mse_metric(&mmse_out, &transmitted);
        let zf_mse = mse_metric(&zf_out, &transmitted);

        assert!(
            mmse_mse < zf_mse,
            "MMSE ({}) should have lower MSE than ZF ({}) at low SNR",
            mmse_mse,
            zf_mse
        );
    }

    #[test]
    fn test_zf_equalization_correctness() {
        // ZF should perfectly invert the channel in the noiseless case.
        let n = 4;
        let zf = ZfEqualizer::new(n);
        let channel = vec![(0.5, 0.5), (1.0, -0.5), (0.3, 0.8), (0.9, 0.1)];
        let transmitted = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        let received = channel_apply(&transmitted, &channel);

        let equalized = zf.equalize(&received, &channel);
        let mse = mse_metric(&equalized, &transmitted);
        assert!(mse < 1e-10, "ZF noiseless MSE should be near zero, got {}", mse);
    }

    #[test]
    fn test_mmse_weights_computation() {
        // Verify MMSE weight formula directly.
        let channel = vec![(1.0, 0.0), (0.0, 1.0)];
        let noise_var = 0.5;
        let w = mmse_weights(&channel, noise_var);

        // For H = (1, 0): W = (1, 0) / (1 + 0.5) = (1/1.5, 0)
        assert!((w[0].0 - 1.0 / 1.5).abs() < 1e-10);
        assert!(w[0].1.abs() < 1e-10);

        // For H = (0, 1): W = (0, -1) / (1 + 0.5) = (0, -1/1.5)
        assert!(w[1].0.abs() < 1e-10);
        assert!((w[1].1 - (-1.0 / 1.5)).abs() < 1e-10);
    }

    #[test]
    fn test_zf_weights_unity_channel() {
        // Unity channel should produce unity weights.
        let channel = vec![(1.0, 0.0); 8];
        let w = zf_weights(&channel);

        for (i, &(wr, wi)) in w.iter().enumerate() {
            assert!(
                (wr - 1.0).abs() < 1e-10 && wi.abs() < 1e-10,
                "ZF weight at {} should be (1,0), got ({}, {})",
                i,
                wr,
                wi
            );
        }
    }

    #[test]
    fn test_sinr_per_subcarrier() {
        let channel = vec![(1.0, 0.0), (0.5, 0.0), (0.0, 2.0)];
        let noise_var = 0.1;
        let sinr = sinr_per_subcarrier(&channel, noise_var);

        // |H[0]|^2 / sigma^2 = 1 / 0.1 = 10
        assert!((sinr[0] - 10.0).abs() < 1e-10);
        // |H[1]|^2 / sigma^2 = 0.25 / 0.1 = 2.5
        assert!((sinr[1] - 2.5).abs() < 1e-10);
        // |H[2]|^2 / sigma^2 = 4 / 0.1 = 40
        assert!((sinr[2] - 40.0).abs() < 1e-10);
    }

    #[test]
    fn test_time_domain_mmse_taps() {
        // Single-tap channel h = [1.0]: equalizer should converge to a near-unity
        // centre tap.
        let td = TimeDomainMmse::new(5);
        let channel_ir = vec![1.0];
        let taps = td.compute_taps(&channel_ir, 0.01);

        // The centre tap (index 2) should be close to 1.0.
        assert!(
            (taps[2] - 1.0).abs() < 0.05,
            "Centre tap should be ~1.0 for unit channel, got {}",
            taps[2]
        );

        // Other taps should be near zero.
        for (i, &t) in taps.iter().enumerate() {
            if i != 2 {
                assert!(
                    t.abs() < 0.05,
                    "Non-centre tap {} should be ~0 for unit channel, got {}",
                    i,
                    t
                );
            }
        }
    }

    #[test]
    fn test_mse_metric_computation() {
        let a = vec![(1.0, 0.0), (0.0, 1.0)];
        let b = vec![(1.0, 0.0), (0.0, 1.0)];
        assert!((mse_metric(&a, &b)).abs() < 1e-15, "Identical signals should have zero MSE");

        let c = vec![(2.0, 0.0), (0.0, 2.0)];
        // MSE = ((1)^2 + (1)^2) / 2 = 1.0
        let mse = mse_metric(&a, &c);
        assert!(
            (mse - 1.0).abs() < 1e-10,
            "MSE should be 1.0 for unit difference, got {}",
            mse
        );
    }

    #[test]
    fn test_regularization_effect() {
        // With noise_var ~ 0 the MMSE behaves like ZF, massively amplifying noise
        // on the weak subcarrier.  Regularization tames that amplification.
        let n = 4;
        let mut eq = MmseEqualizer::new(n);
        // One very weak subcarrier (0.01) where noise will be amplified ~100x by ZF.
        let channel = vec![(0.01, 0.0), (1.0, 0.0), (1.0, 0.0), (1.0, 0.0)];
        let transmitted = vec![(1.0, 0.0); n];
        let received_clean = channel_apply(&transmitted, &channel);

        // Moderate noise that will be hugely amplified on the weak subcarrier.
        let noise: Vec<(f64, f64)> = vec![(0.05, 0.02), (-0.03, 0.04), (0.01, -0.01), (0.02, 0.03)];
        let received = add_noise(&received_clean, &noise);

        // Near-zero noise_var means MMSE ~ ZF: no noise suppression.
        let noise_var = 1e-12;

        // Without regularization (effectively ZF)
        let out_no_reg = eq.equalize(&received, &channel, noise_var);
        let mse_no_reg = mse_metric(&out_no_reg, &transmitted);

        // With regularization: damps the huge gain on the weak subcarrier.
        eq.set_regularization(0.01);
        let out_with_reg = eq.equalize(&received, &channel, noise_var);
        let mse_with_reg = mse_metric(&out_with_reg, &transmitted);

        // Regularization should reduce the huge error from the weak subcarrier.
        assert!(
            mse_with_reg < mse_no_reg,
            "Regularized MSE ({}) should be less than unregularized ({})",
            mse_with_reg,
            mse_no_reg
        );
    }
}
