//! OFDM Channel Estimation
//!
//! Pilot-based channel estimation and equalization for OFDM systems.
//! Complements the `ofdm` module with receiver-side channel tracking.
//!
//! ## Algorithms
//!
//! - **Least Squares (LS)**: H_hat = Y_pilot / X_pilot. Simple, unbiased, but noisy.
//! - **Linear Interpolation**: Extends pilot estimates to data subcarriers.
//! - **Zero-Forcing Equalization**: Divides received data by channel estimate.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ofdm_channel_est::{OfdmChannelEstimator, EstimationMethod};
//! use num_complex::Complex64;
//!
//! let pilot_indices = vec![0, 4, 8, 12];
//! let pilot_values = vec![Complex64::new(1.0, 0.0); 4];
//! let mut estimator = OfdmChannelEstimator::new(
//!     16,
//!     pilot_indices,
//!     pilot_values,
//!     EstimationMethod::LeastSquares,
//! );
//!
//! // Simulate received OFDM symbol (frequency domain, after FFT)
//! let received: Vec<Complex64> = (0..16).map(|_| Complex64::new(0.8, 0.1)).collect();
//! let estimate = estimator.estimate(&received);
//! assert_eq!(estimate.len(), 16);
//! ```

use num_complex::Complex64;

/// Channel estimation method.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EstimationMethod {
    /// Least Squares: H = Y/X at pilot positions. Simple but noisy.
    LeastSquares,
    /// Moving average smoothing of LS estimates across subcarriers.
    Smoothed { window_size: usize },
}

/// Per-subcarrier channel estimate with quality metric.
#[derive(Debug, Clone)]
pub struct ChannelEstimate {
    /// Complex channel response per subcarrier.
    pub h: Vec<Complex64>,
    /// Estimated SNR per subcarrier (if available).
    pub snr_per_subcarrier: Option<Vec<f64>>,
    /// Number of OFDM symbols used for averaging.
    pub symbols_averaged: usize,
}

impl ChannelEstimate {
    /// Get the channel magnitude response.
    pub fn magnitude_response(&self) -> Vec<f64> {
        self.h.iter().map(|h| h.norm()).collect()
    }

    /// Get the channel phase response.
    pub fn phase_response(&self) -> Vec<f64> {
        self.h.iter().map(|h| h.arg()).collect()
    }

    /// Get the group delay (derivative of phase).
    pub fn group_delay(&self) -> Vec<f64> {
        let phase = self.phase_response();
        let mut gd = vec![0.0; phase.len()];
        for i in 1..phase.len() {
            let mut dp = phase[i] - phase[i - 1];
            // Unwrap phase
            while dp > std::f64::consts::PI {
                dp -= 2.0 * std::f64::consts::PI;
            }
            while dp < -std::f64::consts::PI {
                dp += 2.0 * std::f64::consts::PI;
            }
            gd[i] = -dp;
        }
        gd[0] = gd[1];
        gd
    }

    /// Average channel gain in dB.
    pub fn average_gain_db(&self) -> f64 {
        let avg_power: f64 = self.h.iter().map(|h| h.norm_sqr()).sum::<f64>() / self.h.len() as f64;
        10.0 * avg_power.log10()
    }
}

/// OFDM channel estimator using pilot subcarriers.
#[derive(Debug, Clone)]
pub struct OfdmChannelEstimator {
    /// Total number of subcarriers (FFT size).
    fft_size: usize,
    /// Indices of pilot subcarriers.
    pilot_indices: Vec<usize>,
    /// Known pilot values (what was transmitted).
    pilot_values: Vec<Complex64>,
    /// Estimation algorithm.
    method: EstimationMethod,
    /// Running average of channel estimates.
    avg_estimate: Option<Vec<Complex64>>,
    /// Exponential averaging factor (0 = no averaging, 1 = full).
    averaging_alpha: f64,
    /// Count of symbols processed.
    symbols_processed: usize,
}

impl OfdmChannelEstimator {
    pub fn new(
        fft_size: usize,
        pilot_indices: Vec<usize>,
        pilot_values: Vec<Complex64>,
        method: EstimationMethod,
    ) -> Self {
        assert_eq!(
            pilot_indices.len(),
            pilot_values.len(),
            "Pilot indices and values must have same length"
        );
        assert!(
            pilot_indices.iter().all(|&i| i < fft_size),
            "All pilot indices must be < fft_size"
        );
        Self {
            fft_size,
            pilot_indices,
            pilot_values,
            method,
            avg_estimate: None,
            averaging_alpha: 0.0,
            symbols_processed: 0,
        }
    }

    /// Enable exponential averaging across OFDM symbols.
    /// alpha = 0: no averaging (latest only), alpha near 1: heavy averaging.
    pub fn set_averaging(&mut self, alpha: f64) {
        self.averaging_alpha = alpha.clamp(0.0, 0.99);
    }

    /// Estimate the channel from a received frequency-domain OFDM symbol.
    ///
    /// `received_freq` should be the FFT output (after CP removal, before equalization).
    pub fn estimate(&mut self, received_freq: &[Complex64]) -> ChannelEstimate {
        assert!(
            received_freq.len() >= self.fft_size,
            "Received symbol must be at least fft_size samples"
        );

        // Step 1: LS estimation at pilot positions
        let pilot_estimates: Vec<Complex64> = self
            .pilot_indices
            .iter()
            .zip(self.pilot_values.iter())
            .map(|(&idx, &tx_pilot)| {
                let rx_pilot = received_freq[idx];
                if tx_pilot.norm() > 1e-20 {
                    rx_pilot / tx_pilot
                } else {
                    Complex64::new(1.0, 0.0)
                }
            })
            .collect();

        // Step 2: Interpolate to all subcarriers
        let mut h = self.interpolate_pilots(&pilot_estimates);

        // Step 3: Optional smoothing
        if let EstimationMethod::Smoothed { window_size } = self.method {
            h = self.smooth(&h, window_size);
        }

        // Step 4: Optional exponential averaging
        if self.averaging_alpha > 0.0 {
            if let Some(ref prev) = self.avg_estimate {
                let alpha = self.averaging_alpha;
                for (i, hi) in h.iter_mut().enumerate() {
                    *hi = prev[i] * alpha + *hi * (1.0 - alpha);
                }
            }
            self.avg_estimate = Some(h.clone());
        }

        self.symbols_processed += 1;

        ChannelEstimate {
            h,
            snr_per_subcarrier: None,
            symbols_averaged: self.symbols_processed,
        }
    }

    /// Zero-forcing equalization: divide received data by channel estimate.
    pub fn equalize(
        &self,
        received_freq: &[Complex64],
        estimate: &ChannelEstimate,
    ) -> Vec<Complex64> {
        received_freq
            .iter()
            .zip(estimate.h.iter())
            .map(|(&rx, &h)| {
                if h.norm() > 1e-20 {
                    rx / h
                } else {
                    rx // Don't divide by near-zero
                }
            })
            .collect()
    }

    /// MMSE equalization: H* / (|H|^2 + 1/SNR).
    pub fn equalize_mmse(
        &self,
        received_freq: &[Complex64],
        estimate: &ChannelEstimate,
        noise_variance: f64,
    ) -> Vec<Complex64> {
        received_freq
            .iter()
            .zip(estimate.h.iter())
            .map(|(&rx, &h)| {
                let h_conj = h.conj();
                let denom = h.norm_sqr() + noise_variance;
                if denom > 1e-20 {
                    rx * h_conj / denom
                } else {
                    rx
                }
            })
            .collect()
    }

    /// Linear interpolation of pilot estimates across all subcarriers.
    fn interpolate_pilots(&self, pilot_estimates: &[Complex64]) -> Vec<Complex64> {
        let mut h = vec![Complex64::new(1.0, 0.0); self.fft_size];

        if pilot_estimates.is_empty() {
            return h;
        }

        if pilot_estimates.len() == 1 {
            // Single pilot: flat estimate
            for hi in h.iter_mut() {
                *hi = pilot_estimates[0];
            }
            return h;
        }

        // Place pilot estimates
        for (i, &idx) in self.pilot_indices.iter().enumerate() {
            h[idx] = pilot_estimates[i];
        }

        // Linear interpolation between pilots
        let mut sorted: Vec<(usize, Complex64)> = self
            .pilot_indices
            .iter()
            .zip(pilot_estimates.iter())
            .map(|(&idx, &est)| (idx, est))
            .collect();
        sorted.sort_by_key(|&(idx, _)| idx);

        // Interpolate between consecutive pilot positions
        for w in sorted.windows(2) {
            let (idx0, h0) = w[0];
            let (idx1, h1) = w[1];
            for k in idx0..=idx1 {
                let t = if idx1 > idx0 {
                    (k - idx0) as f64 / (idx1 - idx0) as f64
                } else {
                    0.0
                };
                h[k] = h0 * (1.0 - t) + h1 * t;
            }
        }

        // Extrapolate before first pilot
        if let Some(&(first_idx, first_h)) = sorted.first() {
            for k in 0..first_idx {
                h[k] = first_h;
            }
        }

        // Extrapolate after last pilot
        if let Some(&(last_idx, last_h)) = sorted.last() {
            for k in (last_idx + 1)..self.fft_size {
                h[k] = last_h;
            }
        }

        h
    }

    /// Moving average smoothing.
    fn smooth(&self, h: &[Complex64], window_size: usize) -> Vec<Complex64> {
        let n = h.len();
        let half = window_size / 2;
        let mut smoothed = vec![Complex64::new(0.0, 0.0); n];

        for i in 0..n {
            let start = if i >= half { i - half } else { 0 };
            let end = (i + half + 1).min(n);
            let count = end - start;
            let sum: Complex64 = h[start..end].iter().sum();
            smoothed[i] = sum / count as f64;
        }

        smoothed
    }

    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    pub fn pilot_indices(&self) -> &[usize] {
        &self.pilot_indices
    }

    pub fn symbols_processed(&self) -> usize {
        self.symbols_processed
    }

    pub fn reset(&mut self) {
        self.avg_estimate = None;
        self.symbols_processed = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn make_estimator(fft_size: usize) -> OfdmChannelEstimator {
        let pilot_indices = vec![0, 4, 8, 12];
        let pilot_values = vec![Complex64::new(1.0, 0.0); 4];
        OfdmChannelEstimator::new(
            fft_size,
            pilot_indices,
            pilot_values,
            EstimationMethod::LeastSquares,
        )
    }

    #[test]
    fn test_ls_estimation_identity_channel() {
        let mut est = make_estimator(16);
        // Identity channel: received = transmitted
        let received: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 16];
        let result = est.estimate(&received);
        // At pilot positions, H should be ~1.0
        for &idx in &[0usize, 4, 8, 12] {
            assert!(
                (result.h[idx] - Complex64::new(1.0, 0.0)).norm() < 1e-10,
                "H at pilot {} should be 1.0",
                idx
            );
        }
    }

    #[test]
    fn test_ls_estimation_flat_channel() {
        let mut est = make_estimator(16);
        // Flat channel with gain 0.5
        let received: Vec<Complex64> = vec![Complex64::new(0.5, 0.0); 16];
        let result = est.estimate(&received);
        for &idx in &[0usize, 4, 8, 12] {
            assert!(
                (result.h[idx].re - 0.5).abs() < 1e-10,
                "H at pilot {} should be 0.5",
                idx
            );
        }
    }

    #[test]
    fn test_ls_estimation_phase_rotation() {
        let mut est = make_estimator(16);
        // Channel with 45-degree phase rotation
        let h_true = Complex64::from_polar(1.0, PI / 4.0);
        let received: Vec<Complex64> = vec![h_true; 16];
        let result = est.estimate(&received);
        for &idx in &[0usize, 4, 8, 12] {
            assert!(
                (result.h[idx] - h_true).norm() < 1e-10,
                "H at pilot {} should match",
                idx
            );
        }
    }

    #[test]
    fn test_interpolation_between_pilots() {
        let mut est = make_estimator(16);
        // Create frequency-selective channel
        let mut received = vec![Complex64::new(1.0, 0.0); 16];
        received[0] = Complex64::new(1.0, 0.0);
        received[4] = Complex64::new(0.5, 0.0);
        received[8] = Complex64::new(1.0, 0.0);
        received[12] = Complex64::new(0.5, 0.0);

        let result = est.estimate(&received);
        // Between pilots 0 and 4, H should interpolate between 1.0 and 0.5
        let h2 = result.h[2].re;
        assert!(
            h2 > 0.4 && h2 < 1.1,
            "Interpolated H[2] should be between 0.5 and 1.0, got {}",
            h2
        );
    }

    #[test]
    fn test_zero_forcing_equalization() {
        let mut est = make_estimator(16);
        let channel = Complex64::new(0.5, 0.3);
        let transmitted = vec![Complex64::new(1.0, 0.0); 16];
        let received: Vec<Complex64> = transmitted.iter().map(|&x| x * channel).collect();

        let estimate = est.estimate(&received);
        let equalized = est.equalize(&received, &estimate);

        // At pilot positions, equalized should be close to transmitted
        for &idx in &[0usize, 4, 8, 12] {
            assert!(
                (equalized[idx] - Complex64::new(1.0, 0.0)).norm() < 0.1,
                "Equalized signal at pilot {} should be ~1.0",
                idx
            );
        }
    }

    #[test]
    fn test_mmse_equalization() {
        let mut est = make_estimator(16);
        let channel = Complex64::new(0.8, 0.2);
        let received: Vec<Complex64> = vec![channel; 16];
        let estimate = est.estimate(&received);
        let equalized = est.equalize_mmse(&received, &estimate, 0.01);
        assert_eq!(equalized.len(), 16);
        // MMSE should still produce reasonable output
        assert!(equalized[0].norm() > 0.1);
    }

    #[test]
    fn test_smoothed_estimation() {
        let pilot_indices = vec![0, 4, 8, 12];
        let pilot_values = vec![Complex64::new(1.0, 0.0); 4];
        let mut est = OfdmChannelEstimator::new(
            16,
            pilot_indices,
            pilot_values,
            EstimationMethod::Smoothed { window_size: 3 },
        );

        let received = vec![Complex64::new(1.0, 0.0); 16];
        let result = est.estimate(&received);
        assert_eq!(result.h.len(), 16);
    }

    #[test]
    fn test_channel_estimate_metrics() {
        let h = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.5, 0.5),
            Complex64::new(0.0, 1.0),
            Complex64::new(-0.5, 0.5),
        ];
        let estimate = ChannelEstimate {
            h: h.clone(),
            snr_per_subcarrier: None,
            symbols_averaged: 1,
        };

        let mag = estimate.magnitude_response();
        assert_eq!(mag.len(), 4);
        assert!((mag[0] - 1.0).abs() < 1e-10);

        let phase = estimate.phase_response();
        assert_eq!(phase.len(), 4);
        assert!(phase[0].abs() < 1e-10); // Real positive → 0 phase

        let gd = estimate.group_delay();
        assert_eq!(gd.len(), 4);

        let avg_db = estimate.average_gain_db();
        assert!(avg_db.is_finite());
    }

    #[test]
    fn test_averaging() {
        let mut est = make_estimator(16);
        est.set_averaging(0.5);

        let received1 = vec![Complex64::new(1.0, 0.0); 16];
        let _r1 = est.estimate(&received1);

        let received2 = vec![Complex64::new(0.5, 0.0); 16];
        let r2 = est.estimate(&received2);

        // With averaging, estimate should be between 0.5 and 1.0
        assert!(
            r2.h[0].re > 0.4 && r2.h[0].re < 1.1,
            "Averaged estimate should blend: got {}",
            r2.h[0].re
        );
        assert_eq!(r2.symbols_averaged, 2);
    }

    #[test]
    fn test_reset() {
        let mut est = make_estimator(16);
        est.set_averaging(0.5);
        let received = vec![Complex64::new(1.0, 0.0); 16];
        est.estimate(&received);
        assert_eq!(est.symbols_processed(), 1);
        est.reset();
        assert_eq!(est.symbols_processed(), 0);
    }

    #[test]
    fn test_single_pilot() {
        let mut est = OfdmChannelEstimator::new(
            8,
            vec![4],
            vec![Complex64::new(1.0, 0.0)],
            EstimationMethod::LeastSquares,
        );
        let received = vec![Complex64::new(0.7, 0.0); 8];
        let result = est.estimate(&received);
        // Single pilot: should extrapolate flat
        for h in &result.h {
            assert!(
                (h.re - 0.7).abs() < 1e-10,
                "Single pilot should give flat estimate"
            );
        }
    }
}
