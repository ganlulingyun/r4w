//! OFDM Channel Estimation
//!
//! Estimates the channel frequency response from pilot symbols inserted at known
//! subcarrier positions in an OFDM symbol. The estimated channel can then be used
//! for zero-forcing equalization to recover transmitted data.
//!
//! ## Algorithms
//!
//! - **Least Squares (LS)**: `H_pilot = Y_pilot / X_pilot`. Simple and unbiased but
//!   amplifies noise on weak subcarriers.
//! - **Linear Interpolation**: LS at pilot positions followed by linear interpolation
//!   to fill in data subcarrier estimates.
//! - **Spline Interpolation**: LS at pilot positions followed by natural cubic spline
//!   interpolation for smoother estimates.
//! - **MMSE**: LS followed by Wiener smoothing that exploits known SNR to suppress
//!   noise in the channel estimate.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::channel_estimator::{ChannelEstimator, EstimationMethod};
//!
//! // 16-subcarrier OFDM symbol with pilots at indices 0, 5, 10, 15
//! let pilot_indices = vec![0, 5, 10, 15];
//! let estimator = ChannelEstimator::new(
//!     pilot_indices,
//!     16,
//!     EstimationMethod::LinearInterpolation,
//! );
//!
//! // Known pilot symbols (all ones)
//! let known_pilots = vec![(1.0, 0.0); 4];
//! // Received pilots (attenuated by flat channel with gain 0.8)
//! let received_pilots = vec![(0.8, 0.0); 4];
//!
//! let h_est = estimator.estimate(&received_pilots, &known_pilots);
//! assert_eq!(h_est.len(), 16);
//!
//! // All subcarriers should see ~0.8 gain on a flat channel
//! for h in &h_est {
//!     assert!((h.0 - 0.8).abs() < 1e-10);
//! }
//! ```

/// Channel estimation method.
#[derive(Debug, Clone, PartialEq)]
pub enum EstimationMethod {
    /// Least Squares: `H = Y / X` at pilot positions, then nearest-neighbor fill.
    Ls,
    /// Minimum Mean Square Error: LS followed by Wiener smoothing.
    Mmse {
        /// Assumed SNR in dB for Wiener filter weighting.
        snr_db: f64,
    },
    /// LS at pilots, then linear interpolation across all subcarriers.
    LinearInterpolation,
    /// LS at pilots, then natural cubic spline interpolation across all subcarriers.
    SplineInterpolation,
}

/// OFDM channel estimator using pilot symbols.
#[derive(Debug, Clone)]
pub struct ChannelEstimator {
    /// Subcarrier indices where pilots are located.
    pub pilot_indices: Vec<usize>,
    /// Total number of subcarriers (FFT size).
    pub fft_size: usize,
    /// Estimation algorithm to use.
    pub method: EstimationMethod,
}

impl ChannelEstimator {
    /// Create a new channel estimator.
    ///
    /// # Arguments
    ///
    /// * `pilot_indices` - Subcarrier indices carrying pilot symbols.
    /// * `fft_size` - Total number of OFDM subcarriers.
    /// * `method` - Channel estimation algorithm.
    pub fn new(pilot_indices: Vec<usize>, fft_size: usize, method: EstimationMethod) -> Self {
        Self {
            pilot_indices,
            fft_size,
            method,
        }
    }

    /// Estimate the channel frequency response at all subcarriers.
    ///
    /// Computes the channel at pilot positions using LS, then extends the estimate
    /// to all subcarriers according to the configured [`EstimationMethod`].
    ///
    /// # Arguments
    ///
    /// * `received_pilots` - Received pilot symbols `Y_pilot` as `(re, im)` tuples.
    /// * `known_pilots` - Transmitted pilot symbols `X_pilot` as `(re, im)` tuples.
    ///
    /// # Returns
    ///
    /// Channel estimate `H` at every subcarrier, length = `fft_size`.
    pub fn estimate(
        &self,
        received_pilots: &[(f64, f64)],
        known_pilots: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        if received_pilots.is_empty() || known_pilots.is_empty() || self.pilot_indices.is_empty() {
            return vec![(0.0, 0.0); self.fft_size];
        }

        // Step 1: LS estimate at pilot positions
        let pilot_h = ls_estimate(received_pilots, known_pilots);

        // Step 2: Extend to all subcarriers based on method
        match &self.method {
            EstimationMethod::Ls => {
                // Nearest-neighbor fill from pilot estimates
                let mut h = vec![(0.0, 0.0); self.fft_size];
                for (i, &idx) in self.pilot_indices.iter().enumerate() {
                    if i < pilot_h.len() && idx < self.fft_size {
                        h[idx] = pilot_h[i];
                    }
                }
                // Fill non-pilot subcarriers with nearest pilot
                for k in 0..self.fft_size {
                    if !self.pilot_indices.contains(&k) {
                        h[k] = nearest_pilot_value(k, &self.pilot_indices, &pilot_h, self.fft_size);
                    }
                }
                h
            }
            EstimationMethod::LinearInterpolation => {
                interpolate_linear(&self.pilot_indices, &pilot_h, self.fft_size)
            }
            EstimationMethod::SplineInterpolation => {
                interpolate_spline(&self.pilot_indices, &pilot_h, self.fft_size)
            }
            EstimationMethod::Mmse { snr_db } => {
                // LS + linear interpolation, then Wiener smoothing
                let h_ls = interpolate_linear(&self.pilot_indices, &pilot_h, self.fft_size);
                wiener_smooth(&h_ls, *snr_db)
            }
        }
    }

    /// Zero-forcing equalization: recover transmitted data by dividing by channel.
    ///
    /// Computes `X_hat = Y / H` for each subcarrier. Subcarriers where `|H|` is
    /// very small (< 1e-12) are zeroed to avoid noise amplification.
    ///
    /// # Arguments
    ///
    /// * `received` - Received frequency-domain symbols `Y`.
    /// * `channel_estimate` - Channel estimate `H` (from [`estimate`](Self::estimate)).
    ///
    /// # Returns
    ///
    /// Equalized symbols `X_hat`, same length as `received`.
    pub fn equalize(
        &self,
        received: &[(f64, f64)],
        channel_estimate: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        equalize(received, channel_estimate)
    }
}

// ---------------------------------------------------------------------------
// Standalone functions
// ---------------------------------------------------------------------------

/// Least-squares channel estimate at pilot positions.
///
/// Computes `H_pilot[i] = Y[i] / X[i]` using complex division.
///
/// # Arguments
///
/// * `rx_pilots` - Received pilot values.
/// * `tx_pilots` - Known transmitted pilot values.
///
/// # Returns
///
/// Channel estimate at each pilot position.
pub fn ls_estimate(rx_pilots: &[(f64, f64)], tx_pilots: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = rx_pilots.len().min(tx_pilots.len());
    let mut h = Vec::with_capacity(n);
    for i in 0..n {
        h.push(complex_div(rx_pilots[i], tx_pilots[i]));
    }
    h
}

/// Linear interpolation of pilot channel estimates to all subcarriers.
///
/// Pilots must be sorted by index. Subcarriers before the first pilot or after
/// the last pilot are extrapolated from the nearest pair.
///
/// # Arguments
///
/// * `pilot_indices` - Sorted subcarrier indices where pilots reside.
/// * `pilot_values` - Channel estimates at pilot positions.
/// * `total_size` - Total number of subcarriers.
///
/// # Returns
///
/// Interpolated channel estimate at every subcarrier.
pub fn interpolate_linear(
    pilot_indices: &[usize],
    pilot_values: &[(f64, f64)],
    total_size: usize,
) -> Vec<(f64, f64)> {
    if pilot_indices.is_empty() || pilot_values.is_empty() {
        return vec![(0.0, 0.0); total_size];
    }

    let n = pilot_indices.len().min(pilot_values.len());
    if n == 1 {
        // Single pilot: flat fill
        return vec![pilot_values[0]; total_size];
    }

    // Sort pilot indices with corresponding values
    let mut paired: Vec<(usize, (f64, f64))> = pilot_indices[..n]
        .iter()
        .copied()
        .zip(pilot_values[..n].iter().copied())
        .collect();
    paired.sort_by_key(|&(idx, _)| idx);

    let sorted_idx: Vec<usize> = paired.iter().map(|&(i, _)| i).collect();
    let sorted_val: Vec<(f64, f64)> = paired.iter().map(|&(_, v)| v).collect();

    let mut result = vec![(0.0, 0.0); total_size];

    for k in 0..total_size {
        // Find the surrounding pilot interval
        if k <= sorted_idx[0] {
            // Before or at first pilot: use first two pilots for extrapolation
            let (re, im) = lerp_complex(
                sorted_idx[0],
                sorted_idx[1],
                sorted_val[0],
                sorted_val[1],
                k,
            );
            result[k] = (re, im);
        } else if k >= sorted_idx[n - 1] {
            // After or at last pilot: use last two pilots for extrapolation
            let (re, im) = lerp_complex(
                sorted_idx[n - 2],
                sorted_idx[n - 1],
                sorted_val[n - 2],
                sorted_val[n - 1],
                k,
            );
            result[k] = (re, im);
        } else {
            // Between two pilots: find the enclosing pair
            let right = sorted_idx.partition_point(|&idx| idx < k);
            let left = right - 1;
            let (re, im) = lerp_complex(
                sorted_idx[left],
                sorted_idx[right],
                sorted_val[left],
                sorted_val[right],
                k,
            );
            result[k] = (re, im);
        }
    }

    result
}

/// Compute channel power |H|^2 per subcarrier.
///
/// # Arguments
///
/// * `estimate` - Channel estimate at each subcarrier.
///
/// # Returns
///
/// Squared magnitude of the channel at each subcarrier.
pub fn channel_power(estimate: &[(f64, f64)]) -> Vec<f64> {
    estimate
        .iter()
        .map(|&(re, im)| re * re + im * im)
        .collect()
}

/// Average SNR computed from a channel estimate and noise variance.
///
/// `SNR = mean(|H|^2) / noise_var`
///
/// Returns the result in linear scale (not dB).
///
/// # Arguments
///
/// * `estimate` - Channel estimate at each subcarrier.
/// * `noise_var` - Noise variance (linear power).
///
/// # Returns
///
/// Average SNR in linear scale, or 0.0 if inputs are empty or noise_var <= 0.
pub fn average_snr(estimate: &[(f64, f64)], noise_var: f64) -> f64 {
    if estimate.is_empty() || noise_var <= 0.0 {
        return 0.0;
    }
    let powers = channel_power(estimate);
    let mean_power: f64 = powers.iter().sum::<f64>() / powers.len() as f64;
    mean_power / noise_var
}

/// Zero-forcing equalization: `X_hat = Y / H`.
///
/// Subcarriers where `|H|^2 < 1e-24` are zeroed to prevent noise blow-up.
fn equalize(received: &[(f64, f64)], channel_estimate: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = received.len().min(channel_estimate.len());
    let mut out = Vec::with_capacity(n);
    for i in 0..n {
        let h_mag_sq = channel_estimate[i].0 * channel_estimate[i].0
            + channel_estimate[i].1 * channel_estimate[i].1;
        if h_mag_sq < 1e-24 {
            out.push((0.0, 0.0));
        } else {
            out.push(complex_div(received[i], channel_estimate[i]));
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Complex division: `a / b = a * conj(b) / |b|^2`.
fn complex_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom < 1e-30 {
        return (0.0, 0.0);
    }
    let re = (a.0 * b.0 + a.1 * b.1) / denom;
    let im = (a.1 * b.0 - a.0 * b.1) / denom;
    (re, im)
}

/// Complex multiplication: `a * b`.
fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Linear interpolation of a complex value between two pilot positions.
fn lerp_complex(
    idx_a: usize,
    idx_b: usize,
    val_a: (f64, f64),
    val_b: (f64, f64),
    target: usize,
) -> (f64, f64) {
    if idx_a == idx_b {
        return val_a;
    }
    let t = (target as f64 - idx_a as f64) / (idx_b as f64 - idx_a as f64);
    let re = val_a.0 + t * (val_b.0 - val_a.0);
    let im = val_a.1 + t * (val_b.1 - val_a.1);
    (re, im)
}

/// Find the value of the nearest pilot to subcarrier `k`.
fn nearest_pilot_value(
    k: usize,
    pilot_indices: &[usize],
    pilot_values: &[(f64, f64)],
    _fft_size: usize,
) -> (f64, f64) {
    let mut best_dist = usize::MAX;
    let mut best_val = (0.0, 0.0);
    for (i, &idx) in pilot_indices.iter().enumerate() {
        let dist = if k > idx { k - idx } else { idx - k };
        if dist < best_dist && i < pilot_values.len() {
            best_dist = dist;
            best_val = pilot_values[i];
        }
    }
    best_val
}

/// Natural cubic spline interpolation of complex channel estimates.
fn interpolate_spline(
    pilot_indices: &[usize],
    pilot_values: &[(f64, f64)],
    total_size: usize,
) -> Vec<(f64, f64)> {
    if pilot_indices.is_empty() || pilot_values.is_empty() {
        return vec![(0.0, 0.0); total_size];
    }

    let n = pilot_indices.len().min(pilot_values.len());
    if n <= 2 {
        // Fall back to linear interpolation for 1 or 2 points
        return interpolate_linear(pilot_indices, pilot_values, total_size);
    }

    // Sort pilots by index
    let mut paired: Vec<(usize, (f64, f64))> = pilot_indices[..n]
        .iter()
        .copied()
        .zip(pilot_values[..n].iter().copied())
        .collect();
    paired.sort_by_key(|&(idx, _)| idx);

    let x: Vec<f64> = paired.iter().map(|&(i, _)| i as f64).collect();
    let y_re: Vec<f64> = paired.iter().map(|&(_, v)| v.0).collect();
    let y_im: Vec<f64> = paired.iter().map(|&(_, v)| v.1).collect();

    let spline_re = cubic_spline_coeffs(&x, &y_re);
    let spline_im = cubic_spline_coeffs(&x, &y_im);

    let mut result = vec![(0.0, 0.0); total_size];
    for k in 0..total_size {
        let xk = k as f64;
        let re = eval_cubic_spline(&x, &y_re, &spline_re, xk);
        let im = eval_cubic_spline(&x, &y_im, &spline_im, xk);
        result[k] = (re, im);
    }

    result
}

/// Compute natural cubic spline second derivatives (tridiagonal solve).
fn cubic_spline_coeffs(x: &[f64], y: &[f64]) -> Vec<f64> {
    let n = x.len();
    if n < 3 {
        return vec![0.0; n];
    }

    let nm1 = n - 1;
    let mut h = vec![0.0; nm1];
    for i in 0..nm1 {
        h[i] = x[i + 1] - x[i];
    }

    // Right-hand side
    let mut alpha = vec![0.0; nm1];
    for i in 1..nm1 {
        alpha[i] = 3.0 / h[i] * (y[i + 1] - y[i]) - 3.0 / h[i - 1] * (y[i] - y[i - 1]);
    }

    // Tridiagonal solve
    let mut l = vec![1.0; n];
    let mut mu = vec![0.0; n];
    let mut z = vec![0.0; n];

    for i in 1..nm1 {
        l[i] = 2.0 * (x[i + 1] - x[i - 1]) - h[i - 1] * mu[i - 1];
        mu[i] = h[i] / l[i];
        z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
    }

    let mut c = vec![0.0; n];
    for j in (0..nm1).rev() {
        c[j] = z[j] - mu[j] * c[j + 1];
    }

    c
}

/// Evaluate natural cubic spline at point xk.
fn eval_cubic_spline(x: &[f64], y: &[f64], c: &[f64], xk: f64) -> f64 {
    let n = x.len();
    if n < 2 {
        return if n == 1 { y[0] } else { 0.0 };
    }

    // Clamp to spline domain for extrapolation
    let xk_clamped = xk.max(x[0]).min(x[n - 1]);

    // Find interval
    let mut i = 0;
    for j in 1..n - 1 {
        if xk_clamped >= x[j] {
            i = j;
        }
    }

    let h = x[i + 1] - x[i];
    if h.abs() < 1e-30 {
        return y[i];
    }

    let b = (y[i + 1] - y[i]) / h - h * (c[i + 1] + 2.0 * c[i]) / 3.0;
    let d = (c[i + 1] - c[i]) / (3.0 * h);
    let dx = xk_clamped - x[i];

    y[i] + b * dx + c[i] * dx * dx + d * dx * dx * dx
}

/// Wiener smoothing of channel estimate (MMSE).
///
/// Applies a simple low-pass averaging filter weighted by the SNR. At high SNR
/// the filter is close to identity; at low SNR it averages more aggressively.
fn wiener_smooth(h: &[(f64, f64)], snr_db: f64) -> Vec<(f64, f64)> {
    let n = h.len();
    if n == 0 {
        return vec![];
    }

    let snr_linear = 10.0_f64.powf(snr_db / 10.0);
    // Wiener factor: at high SNR -> 1 (keep), at low SNR -> smooth heavily
    // Window half-width scales inversely with SNR
    let half_win = ((3.0 / (1.0 + snr_linear)).ceil() as usize).max(1);

    let mut out = vec![(0.0, 0.0); n];
    for i in 0..n {
        let lo = if i >= half_win { i - half_win } else { 0 };
        let hi = (i + half_win + 1).min(n);
        let count = (hi - lo) as f64;
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        for j in lo..hi {
            sum_re += h[j].0;
            sum_im += h[j].1;
        }
        // Wiener weighting: blend between raw and smoothed
        let alpha = snr_linear / (1.0 + snr_linear);
        let smooth_re = sum_re / count;
        let smooth_im = sum_im / count;
        out[i] = (
            alpha * h[i].0 + (1.0 - alpha) * smooth_re,
            alpha * h[i].1 + (1.0 - alpha) * smooth_im,
        );
    }

    out
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-10;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn approx_eq_c(a: (f64, f64), b: (f64, f64), tol: f64) -> bool {
        approx_eq(a.0, b.0, tol) && approx_eq(a.1, b.1, tol)
    }

    /// LS on a flat channel with gain 1.0 should return H = 1.0 everywhere.
    #[test]
    fn test_ls_flat_channel() {
        let pilot_indices = vec![0, 4, 8, 12];
        let estimator = ChannelEstimator::new(pilot_indices, 16, EstimationMethod::Ls);

        let tx = vec![(1.0, 0.0); 4];
        let rx = vec![(1.0, 0.0); 4]; // Flat channel, gain = 1

        let h = estimator.estimate(&rx, &tx);
        assert_eq!(h.len(), 16);
        for val in &h {
            assert!(approx_eq_c(*val, (1.0, 0.0), EPS), "Expected (1,0), got {:?}", val);
        }
    }

    /// LS with a known frequency-selective channel.
    #[test]
    fn test_ls_known_channel() {
        let pilot_indices = vec![0, 1, 2, 3];
        let estimator = ChannelEstimator::new(pilot_indices, 4, EstimationMethod::Ls);

        // Channel: H = [0.5, 0.8+0.2j, 1.0, 0.3-0.1j]
        let tx = vec![(1.0, 0.0); 4];
        let rx = vec![(0.5, 0.0), (0.8, 0.2), (1.0, 0.0), (0.3, -0.1)];

        let h = estimator.estimate(&rx, &tx);
        assert_eq!(h.len(), 4);
        assert!(approx_eq_c(h[0], (0.5, 0.0), EPS));
        assert!(approx_eq_c(h[1], (0.8, 0.2), EPS));
        assert!(approx_eq_c(h[2], (1.0, 0.0), EPS));
        assert!(approx_eq_c(h[3], (0.3, -0.1), EPS));
    }

    /// Linear interpolation between pilots fills in intermediate subcarriers.
    #[test]
    fn test_linear_interpolation() {
        let pilot_indices = vec![0, 4];
        let estimator = ChannelEstimator::new(
            pilot_indices.clone(),
            5,
            EstimationMethod::LinearInterpolation,
        );

        // Pilots: H[0] = (1.0, 0.0), H[4] = (0.0, 1.0)
        let tx = vec![(1.0, 0.0), (1.0, 0.0)];
        let rx = vec![(1.0, 0.0), (0.0, 1.0)];

        let h = estimator.estimate(&rx, &tx);
        assert_eq!(h.len(), 5);
        // Check interpolated midpoint at index 2: (0.5, 0.5)
        assert!(approx_eq_c(h[0], (1.0, 0.0), EPS));
        assert!(approx_eq_c(h[2], (0.5, 0.5), EPS));
        assert!(approx_eq_c(h[4], (0.0, 1.0), EPS));
        // Check index 1: (0.75, 0.25)
        assert!(approx_eq_c(h[1], (0.75, 0.25), EPS));
        // Check index 3: (0.25, 0.75)
        assert!(approx_eq_c(h[3], (0.25, 0.75), EPS));
    }

    /// Zero-forcing equalization on a flat channel recovers the original symbols.
    #[test]
    fn test_equalize_flat() {
        let estimator = ChannelEstimator::new(vec![], 4, EstimationMethod::Ls);

        // Flat channel H = (0.5, 0.0), transmitted symbols are QPSK-like
        let channel = vec![(0.5, 0.0); 4];
        let tx_symbols = vec![(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)];
        let received: Vec<(f64, f64)> = tx_symbols
            .iter()
            .map(|&s| complex_mul(s, (0.5, 0.0)))
            .collect();

        let equalized = estimator.equalize(&received, &channel);
        assert_eq!(equalized.len(), 4);
        for (eq, &tx) in equalized.iter().zip(tx_symbols.iter()) {
            assert!(approx_eq_c(*eq, tx, EPS), "Expected {:?}, got {:?}", tx, eq);
        }
    }

    /// Zero-forcing equalization on a frequency-selective channel.
    #[test]
    fn test_equalize_frequency_selective() {
        let estimator = ChannelEstimator::new(vec![], 3, EstimationMethod::Ls);

        // Different channel gain per subcarrier
        let channel = vec![(0.5, 0.0), (0.0, 1.0), (1.0, 0.5)];
        let tx_symbols = vec![(1.0, 0.0), (0.0, 1.0), (1.0, 1.0)];

        // Received = H * X per subcarrier
        let received: Vec<(f64, f64)> = tx_symbols
            .iter()
            .zip(channel.iter())
            .map(|(&x, &h)| complex_mul(x, h))
            .collect();

        let equalized = estimator.equalize(&received, &channel);
        assert_eq!(equalized.len(), 3);
        for (eq, &tx) in equalized.iter().zip(tx_symbols.iter()) {
            assert!(
                approx_eq_c(*eq, tx, 1e-9),
                "Expected {:?}, got {:?}",
                tx,
                eq
            );
        }
    }

    /// Channel power |H|^2 computation.
    #[test]
    fn test_channel_power() {
        let h = vec![(1.0, 0.0), (0.0, 1.0), (3.0, 4.0), (0.0, 0.0)];
        let p = channel_power(&h);
        assert_eq!(p.len(), 4);
        assert!(approx_eq(p[0], 1.0, EPS));
        assert!(approx_eq(p[1], 1.0, EPS));
        assert!(approx_eq(p[2], 25.0, EPS));
        assert!(approx_eq(p[3], 0.0, EPS));
    }

    /// Average SNR from channel estimate.
    #[test]
    fn test_average_snr() {
        // Uniform channel |H|^2 = 1.0, noise_var = 0.1 -> SNR = 10.0
        let h = vec![(1.0, 0.0); 8];
        let snr = average_snr(&h, 0.1);
        assert!(approx_eq(snr, 10.0, EPS), "Expected 10.0, got {}", snr);

        // Mixed channel: |H|^2 = [1.0, 4.0], mean = 2.5, noise = 0.5 -> SNR = 5.0
        let h2 = vec![(1.0, 0.0), (2.0, 0.0)];
        let snr2 = average_snr(&h2, 0.5);
        assert!(approx_eq(snr2, 5.0, EPS), "Expected 5.0, got {}", snr2);
    }

    /// Single pilot: channel estimate is constant across all subcarriers.
    #[test]
    fn test_single_pilot() {
        let estimator = ChannelEstimator::new(
            vec![3],
            8,
            EstimationMethod::LinearInterpolation,
        );

        let tx = vec![(1.0, 0.0)];
        let rx = vec![(0.7, 0.3)];

        let h = estimator.estimate(&rx, &tx);
        assert_eq!(h.len(), 8);
        // Single pilot -> flat fill
        for val in &h {
            assert!(
                approx_eq_c(*val, (0.7, 0.3), EPS),
                "Expected (0.7, 0.3), got {:?}",
                val
            );
        }
    }

    /// All subcarriers are pilots: estimate exactly matches LS.
    #[test]
    fn test_all_pilots() {
        let fft_size = 4;
        let pilot_indices: Vec<usize> = (0..fft_size).collect();
        let estimator = ChannelEstimator::new(
            pilot_indices,
            fft_size,
            EstimationMethod::LinearInterpolation,
        );

        let tx = vec![(1.0, 0.0); 4];
        let rx = vec![(0.9, 0.1), (0.5, -0.2), (1.0, 0.0), (0.3, 0.8)];

        let h = estimator.estimate(&rx, &tx);
        assert_eq!(h.len(), 4);
        assert!(approx_eq_c(h[0], (0.9, 0.1), EPS));
        assert!(approx_eq_c(h[1], (0.5, -0.2), EPS));
        assert!(approx_eq_c(h[2], (1.0, 0.0), EPS));
        assert!(approx_eq_c(h[3], (0.3, 0.8), EPS));
    }

    /// Empty inputs return zero-filled vector.
    #[test]
    fn test_empty_input() {
        let estimator = ChannelEstimator::new(vec![0, 4], 8, EstimationMethod::Ls);

        // Empty received pilots
        let h1 = estimator.estimate(&[], &[(1.0, 0.0)]);
        assert_eq!(h1.len(), 8);
        for val in &h1 {
            assert!(approx_eq_c(*val, (0.0, 0.0), EPS));
        }

        // Empty known pilots
        let h2 = estimator.estimate(&[(1.0, 0.0)], &[]);
        assert_eq!(h2.len(), 8);
        for val in &h2 {
            assert!(approx_eq_c(*val, (0.0, 0.0), EPS));
        }

        // Empty pilot indices
        let estimator2 = ChannelEstimator::new(vec![], 8, EstimationMethod::Ls);
        let h3 = estimator2.estimate(&[(1.0, 0.0)], &[(1.0, 0.0)]);
        assert_eq!(h3.len(), 8);
        for val in &h3 {
            assert!(approx_eq_c(*val, (0.0, 0.0), EPS));
        }

        // channel_power on empty
        let p = channel_power(&[]);
        assert!(p.is_empty());

        // average_snr on empty
        let snr = average_snr(&[], 1.0);
        assert!(approx_eq(snr, 0.0, EPS));

        // average_snr with zero noise variance
        let snr2 = average_snr(&[(1.0, 0.0)], 0.0);
        assert!(approx_eq(snr2, 0.0, EPS));
    }
}
