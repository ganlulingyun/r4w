//! Wiener filter for optimal noise reduction.
//!
//! Implements the Wiener filter, which minimizes the mean-squared error (MSE)
//! between its output and a desired signal. The optimal tap weights are computed
//! as **w = R⁻¹ p**, where **R** is the autocorrelation matrix of the input
//! and **p** is the cross-correlation vector between the input and the desired
//! signal.
//!
//! # Features
//!
//! - **Time-domain Wiener filter** – solve the Wiener–Hopf equation via
//!   Gaussian elimination on the Toeplitz autocorrelation matrix.
//! - **Frequency-domain Wiener filter** – spectral subtraction variant:
//!   `W(f) = S_xx(f) / (S_xx(f) + S_nn(f))`.
//! - **Adaptive mode** – periodically re-estimate taps from running statistics.
//! - Noise-power estimation from signal-free segments.
//! - SNR improvement and MSE computation utilities.
//!
//! # Example
//!
//! ```
//! use r4w_core::wiener_filter::WienerFilter;
//!
//! // Create a simple desired signal and a noisy observation.
//! let desired: Vec<f64> = (0..128).map(|i| (i as f64 * 0.1).sin()).collect();
//! let noisy: Vec<f64> = desired.iter().enumerate()
//!     .map(|(i, &d)| d + 0.1 * ((i as f64) * 0.73).cos())
//!     .collect();
//!
//! // Learn a 16-tap Wiener filter from the data.
//! let mut wf = WienerFilter::new(16);
//! wf.estimate_from_data(&noisy, &desired);
//!
//! // Apply the filter.
//! let filtered = wf.filter(&noisy);
//! assert_eq!(filtered.len(), noisy.len());
//!
//! // The MSE after filtering should be lower than the MSE of the raw input.
//! let mse_before = WienerFilter::compute_mse(&noisy, &desired);
//! let mse_after  = WienerFilter::compute_mse(&filtered, &desired);
//! assert!(mse_after < mse_before, "Wiener filter should reduce MSE");
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Wiener filter (time-domain)
// ---------------------------------------------------------------------------

/// Time-domain Wiener filter with configurable tap count.
///
/// After construction, call [`estimate_from_data`](WienerFilter::estimate_from_data)
/// (or [`set_taps`](WienerFilter::set_taps)) before [`filter`](WienerFilter::filter).
#[derive(Debug, Clone)]
pub struct WienerFilter {
    /// Number of filter taps (filter order + 1).
    tap_count: usize,
    /// FIR coefficient vector (length = `tap_count`).
    taps: Vec<f64>,
    /// Whether the taps have been computed / set.
    trained: bool,
    /// Adaptive-mode block size (0 = non-adaptive).
    adaptive_block_size: usize,
}

impl WienerFilter {
    // ---- construction -----------------------------------------------------

    /// Create a new Wiener filter with `tap_count` taps.
    ///
    /// The filter is initially untrained – taps are all zero until
    /// [`estimate_from_data`] or [`set_taps`] is called.
    ///
    /// # Panics
    ///
    /// Panics if `tap_count` is zero.
    pub fn new(tap_count: usize) -> Self {
        assert!(tap_count > 0, "tap_count must be > 0");
        Self {
            tap_count,
            taps: vec![0.0; tap_count],
            trained: false,
            adaptive_block_size: 0,
        }
    }

    /// Enable adaptive mode: taps are re-estimated every `block_size` samples
    /// during [`filter`].
    ///
    /// Pass `0` to disable adaptive mode (the default).
    pub fn set_adaptive_block_size(&mut self, block_size: usize) {
        self.adaptive_block_size = block_size;
    }

    // ---- estimation -------------------------------------------------------

    /// Estimate optimal taps from observed `input` and `desired` signals.
    ///
    /// Internally computes the autocorrelation of `input` (R) and the
    /// cross-correlation between `input` and `desired` (p), then solves
    /// **w = R⁻¹ p** via Gaussian elimination on the Toeplitz matrix.
    ///
    /// Both slices must have the same length and be at least as long as
    /// `tap_count`.
    pub fn estimate_from_data(&mut self, input: &[f64], desired: &[f64]) {
        let n = input.len().min(desired.len());
        assert!(
            n >= self.tap_count,
            "signal length ({n}) must be >= tap_count ({})",
            self.tap_count
        );

        let r = autocorrelation(input, self.tap_count);
        let p = cross_correlation(input, desired, self.tap_count);

        self.taps = solve_toeplitz(&r, &p);
        self.trained = true;
    }

    /// Directly set the filter taps (e.g. from an external solver).
    pub fn set_taps(&mut self, taps: &[f64]) {
        self.tap_count = taps.len();
        self.taps = taps.to_vec();
        self.trained = true;
    }

    /// Return a reference to the current taps.
    pub fn taps(&self) -> &[f64] {
        &self.taps
    }

    /// Return the number of taps.
    pub fn tap_count(&self) -> usize {
        self.tap_count
    }

    /// Whether the filter has been trained.
    pub fn is_trained(&self) -> bool {
        self.trained
    }

    // ---- filtering --------------------------------------------------------

    /// Apply the Wiener filter to `input`, returning the filtered signal.
    ///
    /// If adaptive mode is enabled the taps are re-estimated every
    /// `adaptive_block_size` samples using the most-recent block as both
    /// input and desired signal (self-referential update). This is a
    /// simplified adaptation suitable for stationary noise removal.
    ///
    /// The output has the same length as `input`.
    pub fn filter(&mut self, input: &[f64]) -> Vec<f64> {
        if !self.trained {
            // If not trained, return zeros (safe default).
            return vec![0.0; input.len()];
        }

        if self.adaptive_block_size > 0 && input.len() >= self.adaptive_block_size {
            return self.filter_adaptive(input);
        }

        fir_filter(&self.taps, input)
    }

    /// Reset the filter to its untrained state.
    pub fn reset(&mut self) {
        self.taps = vec![0.0; self.tap_count];
        self.trained = false;
    }

    // ---- metrics ----------------------------------------------------------

    /// Compute mean-squared error between two equal-length signals.
    pub fn compute_mse(a: &[f64], b: &[f64]) -> f64 {
        let n = a.len().min(b.len());
        if n == 0 {
            return 0.0;
        }
        a.iter()
            .zip(b.iter())
            .map(|(x, y)| (x - y).powi(2))
            .sum::<f64>()
            / n as f64
    }

    /// Estimate SNR improvement (in dB) achieved by the filter.
    ///
    /// `noisy` is the raw input, `filtered` is the Wiener-filter output, and
    /// `desired` is the clean reference.
    pub fn snr_improvement_db(noisy: &[f64], filtered: &[f64], desired: &[f64]) -> f64 {
        let mse_before = Self::compute_mse(noisy, desired);
        let mse_after = Self::compute_mse(filtered, desired);
        if mse_after == 0.0 {
            return f64::INFINITY;
        }
        10.0 * (mse_before / mse_after).log10()
    }

    /// Estimate noise power from a signal-free (noise-only) segment.
    ///
    /// Returns the average power (mean of x²).
    pub fn estimate_noise_power(noise_segment: &[f64]) -> f64 {
        if noise_segment.is_empty() {
            return 0.0;
        }
        noise_segment.iter().map(|x| x * x).sum::<f64>() / noise_segment.len() as f64
    }

    // ---- internals --------------------------------------------------------

    /// Adaptive filtering: re-estimate taps block-by-block.
    fn filter_adaptive(&mut self, input: &[f64]) -> Vec<f64> {
        let bs = self.adaptive_block_size;
        let mut output = Vec::with_capacity(input.len());
        let mut pos = 0;

        while pos < input.len() {
            let end = (pos + bs).min(input.len());
            let block = &input[pos..end];

            // Re-estimate taps if we have enough samples.
            if block.len() >= self.tap_count {
                let r = autocorrelation(block, self.tap_count);
                let p = cross_correlation(block, block, self.tap_count);
                self.taps = solve_toeplitz(&r, &p);
            }

            output.extend(fir_filter(&self.taps, block));
            pos = end;
        }

        output
    }
}

// ---------------------------------------------------------------------------
// Frequency-domain Wiener filter
// ---------------------------------------------------------------------------

/// Frequency-domain Wiener filter: `W(f) = S_xx / (S_xx + S_nn)`.
///
/// Operates on power-spectral-density estimates. Useful when the signal and
/// noise PSDs can be estimated independently (e.g. speech enhancement).
#[derive(Debug, Clone)]
pub struct FrequencyDomainWienerFilter {
    /// FFT size (number of frequency bins).
    fft_size: usize,
    /// Estimated noise PSD (one-sided, `fft_size` bins).
    noise_psd: Vec<f64>,
}

impl FrequencyDomainWienerFilter {
    /// Create a new frequency-domain Wiener filter.
    ///
    /// `fft_size` is the number of frequency bins.
    ///
    /// # Panics
    ///
    /// Panics if `fft_size` is zero.
    pub fn new(fft_size: usize) -> Self {
        assert!(fft_size > 0, "fft_size must be > 0");
        Self {
            fft_size,
            noise_psd: vec![0.0; fft_size],
        }
    }

    /// Set the noise PSD from a noise-only segment.
    ///
    /// Computes the periodogram of `noise_segment` and stores it.
    pub fn estimate_noise_psd(&mut self, noise_segment: &[f64]) {
        self.noise_psd = periodogram(noise_segment, self.fft_size);
    }

    /// Set the noise PSD directly.
    pub fn set_noise_psd(&mut self, psd: &[f64]) {
        assert_eq!(psd.len(), self.fft_size);
        self.noise_psd = psd.to_vec();
    }

    /// Apply the frequency-domain Wiener filter to `input`.
    ///
    /// Internally: DFT -> multiply by gain `S_xx/(S_xx+S_nn)` -> IDFT.
    /// The signal PSD is estimated from `input` itself.
    pub fn filter(&self, input: &[f64]) -> Vec<f64> {
        let n = input.len();
        if n == 0 {
            return vec![];
        }

        // Forward DFT (real -> complex).
        let spectrum = real_dft(input, self.fft_size);

        // Estimate signal+noise PSD from the input spectrum.
        let noisy_psd: Vec<f64> = spectrum
            .iter()
            .map(|(re, im)| (re * re + im * im) / self.fft_size as f64)
            .collect();

        // Wiener gain per bin.
        let gain: Vec<f64> = noisy_psd
            .iter()
            .zip(self.noise_psd.iter())
            .map(|(&s_noisy, &s_nn)| {
                let s_xx = (s_noisy - s_nn).max(0.0);
                if s_xx + s_nn == 0.0 {
                    0.0
                } else {
                    s_xx / (s_xx + s_nn)
                }
            })
            .collect();

        // Apply gain.
        let filtered_spectrum: Vec<(f64, f64)> = spectrum
            .iter()
            .zip(gain.iter())
            .map(|(&(re, im), &g)| (re * g, im * g))
            .collect();

        // Inverse DFT and truncate to original length.
        let out = real_idft(&filtered_spectrum, self.fft_size);
        out[..n].to_vec()
    }

    /// Return the current noise PSD estimate.
    pub fn noise_psd(&self) -> &[f64] {
        &self.noise_psd
    }

    /// Compute the Wiener gain curve for a given signal PSD.
    ///
    /// Returns `fft_size` gain values in [0, 1].
    pub fn gain_curve(&self, signal_psd: &[f64]) -> Vec<f64> {
        signal_psd
            .iter()
            .zip(self.noise_psd.iter())
            .map(|(&s_xx, &s_nn)| {
                if s_xx + s_nn == 0.0 {
                    0.0
                } else {
                    s_xx / (s_xx + s_nn)
                }
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Helper functions (private)
// ---------------------------------------------------------------------------

/// Autocorrelation of `x` at lags 0..`max_lag`.
fn autocorrelation(x: &[f64], max_lag: usize) -> Vec<f64> {
    let n = x.len();
    (0..max_lag)
        .map(|lag| {
            let sum: f64 = (0..n - lag).map(|i| x[i] * x[i + lag]).sum();
            sum / n as f64
        })
        .collect()
}

/// Cross-correlation between `x` and `d` at lags 0..`max_lag`.
fn cross_correlation(x: &[f64], d: &[f64], max_lag: usize) -> Vec<f64> {
    let n = x.len().min(d.len());
    (0..max_lag)
        .map(|lag| {
            let sum: f64 = (0..n - lag).map(|i| x[i + lag] * d[i]).sum();
            sum / n as f64
        })
        .collect()
}

/// Build the Toeplitz matrix from autocorrelation vector and solve R w = p
/// via Gaussian elimination with partial pivoting.
fn solve_toeplitz(r: &[f64], p: &[f64]) -> Vec<f64> {
    let n = r.len();
    assert_eq!(p.len(), n);

    if n == 0 {
        return vec![];
    }

    // Edge case: r[0] == 0 -> return zeros.
    if r[0].abs() < 1e-30 {
        return vec![0.0; n];
    }

    // Build the Toeplitz matrix R.
    let mut mat = vec![vec![0.0; n]; n];
    for i in 0..n {
        for j in 0..n {
            let idx = if i >= j { i - j } else { j - i };
            mat[i][j] = r[idx];
        }
    }

    let mut b = p.to_vec();
    gauss_solve(&mut mat, &mut b)
}

/// Solve **A x = b** in-place by Gaussian elimination with partial pivoting.
fn gauss_solve(a: &mut [Vec<f64>], b: &mut [f64]) -> Vec<f64> {
    let n = a.len();
    // Forward elimination.
    for col in 0..n {
        // Partial pivoting.
        let mut max_row = col;
        let mut max_val = a[col][col].abs();
        for row in (col + 1)..n {
            if a[row][col].abs() > max_val {
                max_val = a[row][col].abs();
                max_row = row;
            }
        }
        a.swap(col, max_row);
        b.swap(col, max_row);

        let pivot = a[col][col];
        if pivot.abs() < 1e-30 {
            continue; // singular row - skip
        }

        for row in (col + 1)..n {
            let factor = a[row][col] / pivot;
            for k in col..n {
                a[row][k] -= factor * a[col][k];
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution.
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = b[i];
        for j in (i + 1)..n {
            sum -= a[i][j] * x[j];
        }
        if a[i][i].abs() < 1e-30 {
            x[i] = 0.0;
        } else {
            x[i] = sum / a[i][i];
        }
    }
    x
}

/// FIR filtering: y[n] = sum_k h[k] * x[n-k].
fn fir_filter(taps: &[f64], input: &[f64]) -> Vec<f64> {
    let n = input.len();
    let m = taps.len();
    (0..n)
        .map(|i| {
            let mut acc = 0.0;
            for k in 0..m {
                if i >= k {
                    acc += taps[k] * input[i - k];
                }
            }
            acc
        })
        .collect()
}

/// Simple periodogram PSD estimate (magnitude^2 of DFT / N).
fn periodogram(x: &[f64], fft_size: usize) -> Vec<f64> {
    let spectrum = real_dft(x, fft_size);
    spectrum
        .iter()
        .map(|(re, im)| (re * re + im * im) / fft_size as f64)
        .collect()
}

/// Compute the DFT of a real signal, returning `fft_size` complex bins
/// as `(re, im)` pairs.
///
/// Zero-pads or truncates `x` to `fft_size`.
fn real_dft(x: &[f64], fft_size: usize) -> Vec<(f64, f64)> {
    let n = fft_size;
    (0..n)
        .map(|k| {
            let mut re = 0.0;
            let mut im = 0.0;
            for (i, &xi) in x.iter().enumerate().take(n) {
                let angle = -2.0 * PI * (k as f64) * (i as f64) / (n as f64);
                re += xi * angle.cos();
                im += xi * angle.sin();
            }
            (re, im)
        })
        .collect()
}

/// Inverse DFT: take `fft_size` complex bins, return `fft_size` real samples.
fn real_idft(spectrum: &[(f64, f64)], fft_size: usize) -> Vec<f64> {
    let n = fft_size;
    (0..n)
        .map(|i| {
            let mut sum = 0.0;
            for (k, &(re, im)) in spectrum.iter().enumerate().take(n) {
                let angle = 2.0 * PI * (k as f64) * (i as f64) / (n as f64);
                sum += re * angle.cos() - im * angle.sin();
            }
            sum / n as f64
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: generate a sine wave.
    fn sine_wave(len: usize, freq: f64) -> Vec<f64> {
        (0..len)
            .map(|i| (2.0 * PI * freq * i as f64).sin())
            .collect()
    }

    // Helper: add deterministic "noise" to a signal.
    fn add_noise(signal: &[f64], amplitude: f64, seed_freq: f64) -> Vec<f64> {
        signal
            .iter()
            .enumerate()
            .map(|(i, &s)| s + amplitude * (seed_freq * i as f64).cos())
            .collect()
    }

    #[test]
    fn test_new_filter() {
        let wf = WienerFilter::new(8);
        assert_eq!(wf.tap_count(), 8);
        assert!(!wf.is_trained());
        assert_eq!(wf.taps().len(), 8);
    }

    #[test]
    #[should_panic(expected = "tap_count must be > 0")]
    fn test_zero_taps_panics() {
        WienerFilter::new(0);
    }

    #[test]
    fn test_estimate_and_filter_reduces_mse() {
        let desired = sine_wave(256, 0.05);
        let noisy = add_noise(&desired, 0.3, 0.73);

        let mut wf = WienerFilter::new(16);
        wf.estimate_from_data(&noisy, &desired);
        assert!(wf.is_trained());

        let filtered = wf.filter(&noisy);
        assert_eq!(filtered.len(), noisy.len());

        let mse_before = WienerFilter::compute_mse(&noisy, &desired);
        let mse_after = WienerFilter::compute_mse(&filtered, &desired);
        assert!(
            mse_after < mse_before,
            "MSE should decrease: before={mse_before}, after={mse_after}"
        );
    }

    #[test]
    fn test_snr_improvement_positive() {
        let desired = sine_wave(256, 0.05);
        let noisy = add_noise(&desired, 0.4, 0.73);

        let mut wf = WienerFilter::new(16);
        wf.estimate_from_data(&noisy, &desired);
        let filtered = wf.filter(&noisy);

        let improvement = WienerFilter::snr_improvement_db(&noisy, &filtered, &desired);
        assert!(
            improvement > 0.0,
            "SNR improvement should be positive, got {improvement}"
        );
    }

    #[test]
    fn test_noise_power_estimation() {
        let noise: Vec<f64> = (0..1000).map(|i| 0.5 * (0.123 * i as f64).sin()).collect();
        let power = WienerFilter::estimate_noise_power(&noise);
        // Power of 0.5*sin should be ~0.125.
        assert!((power - 0.125).abs() < 0.01, "noise power = {power}");
    }

    #[test]
    fn test_noise_power_empty() {
        assert_eq!(WienerFilter::estimate_noise_power(&[]), 0.0);
    }

    #[test]
    fn test_set_taps() {
        let mut wf = WienerFilter::new(4);
        wf.set_taps(&[1.0, 0.0, 0.0, 0.0]);
        assert!(wf.is_trained());
        // With identity tap [1,0,0,0], output == input.
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let out = wf.filter(&input);
        assert_eq!(out, input);
    }

    #[test]
    fn test_reset() {
        let mut wf = WienerFilter::new(8);
        wf.set_taps(&[1.0; 8]);
        assert!(wf.is_trained());
        wf.reset();
        assert!(!wf.is_trained());
        // After reset, filter returns zeros.
        let out = wf.filter(&[1.0; 16]);
        assert!(out.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_mse_identical_signals() {
        let a = vec![1.0, 2.0, 3.0];
        assert_eq!(WienerFilter::compute_mse(&a, &a), 0.0);
    }

    #[test]
    fn test_mse_known_value() {
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![2.0, 3.0, 4.0];
        // Each diff = 1.0, diff^2 = 1.0, mean = 1.0.
        assert!((WienerFilter::compute_mse(&a, &b) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn test_adaptive_mode() {
        let desired = sine_wave(512, 0.05);
        let noisy = add_noise(&desired, 0.3, 0.73);

        let mut wf = WienerFilter::new(16);
        wf.estimate_from_data(&noisy, &desired);
        wf.set_adaptive_block_size(128);

        let filtered = wf.filter(&noisy);
        assert_eq!(filtered.len(), noisy.len());
        // Should still produce reasonable output.
        let mse = WienerFilter::compute_mse(&filtered, &desired);
        assert!(mse < 1.0, "adaptive MSE should be bounded, got {mse}");
    }

    #[test]
    fn test_frequency_domain_filter() {
        let desired = sine_wave(128, 0.05);
        let noise_only: Vec<f64> =
            (0..128).map(|i| 0.3 * (0.73 * i as f64).cos()).collect();
        let noisy: Vec<f64> = desired
            .iter()
            .zip(noise_only.iter())
            .map(|(d, n)| d + n)
            .collect();

        let mut fdf = FrequencyDomainWienerFilter::new(128);
        fdf.estimate_noise_psd(&noise_only);
        let filtered = fdf.filter(&noisy);

        let mse_before = WienerFilter::compute_mse(&noisy, &desired);
        let mse_after = WienerFilter::compute_mse(&filtered, &desired);
        assert!(
            mse_after < mse_before,
            "Freq-domain filter should reduce MSE: before={mse_before}, after={mse_after}"
        );
    }

    #[test]
    fn test_frequency_domain_gain_curve() {
        let fdf = FrequencyDomainWienerFilter::new(4);
        let signal_psd = vec![1.0, 0.5, 0.1, 0.0];
        // noise PSD is all zeros -> gain should be 1 where signal > 0.
        let gain = fdf.gain_curve(&signal_psd);
        assert_eq!(gain.len(), 4);
        assert!((gain[0] - 1.0).abs() < 1e-12);
        assert!((gain[1] - 1.0).abs() < 1e-12);
        assert!((gain[2] - 1.0).abs() < 1e-12);
        assert_eq!(gain[3], 0.0); // 0/0 -> 0
    }

    #[test]
    fn test_autocorrelation_dc_signal() {
        let dc = vec![1.0; 64];
        let r = autocorrelation(&dc, 4);
        // For a DC signal of amplitude 1, all autocorrelation lags should be ~1.
        for &val in &r {
            assert!(
                (val - 1.0).abs() < 0.05,
                "autocorrelation of DC should be ~1, got {val}"
            );
        }
    }

    #[test]
    fn test_fir_identity() {
        // Single tap [1.0] should reproduce input.
        let input = vec![3.0, 1.0, 4.0, 1.0, 5.0];
        let out = fir_filter(&[1.0], &input);
        assert_eq!(out, input);
    }

    #[test]
    fn test_fir_delay() {
        // Taps [0, 1] should delay signal by one sample.
        let input = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let out = fir_filter(&[0.0, 1.0], &input);
        assert_eq!(out, vec![0.0, 1.0, 2.0, 3.0, 4.0]);
    }
}
