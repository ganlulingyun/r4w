//! CFO Estimator — Carrier Frequency Offset estimation
//!
//! Standalone carrier frequency offset measurement using
//! autocorrelation, Moose, and Kay algorithms. Outputs the
//! frequency error in Hz for downstream correction by PLL/NCO.
//! Unlike PLL/FLL which track and correct, this block only measures.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cfo_estimator::estimate_cfo_autocorr;
//! use num_complex::Complex64;
//! use std::f64::consts::PI;
//!
//! // Signal with 100 Hz CFO at 1000 Hz sample rate
//! let fs = 1000.0;
//! let cfo = 100.0;
//! let n = 256;
//! let samples: Vec<Complex64> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / fs;
//!         Complex64::from_polar(1.0, 2.0 * PI * cfo * t)
//!     })
//!     .collect();
//! let estimated = estimate_cfo_autocorr(&samples, 1, fs);
//! assert!((estimated - cfo).abs() < 5.0); // Within 5 Hz
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Estimate CFO using autocorrelation at lag D.
///
/// The phase difference between consecutive groups of D samples
/// gives the frequency offset: `f_cfo = angle(R(D)) / (2π * D/fs)`
///
/// - `samples`: complex input samples
/// - `lag`: correlation lag (typically 1 for fine estimate, larger for coarse)
/// - `sample_rate`: sample rate in Hz
///
/// Returns estimated CFO in Hz.
pub fn estimate_cfo_autocorr(samples: &[Complex64], lag: usize, sample_rate: f64) -> f64 {
    if samples.len() <= lag || lag == 0 {
        return 0.0;
    }

    let mut sum = Complex64::new(0.0, 0.0);
    for i in lag..samples.len() {
        sum += samples[i] * samples[i - lag].conj();
    }

    let phase = sum.arg();
    phase * sample_rate / (2.0 * PI * lag as f64)
}

/// Estimate CFO using the Moose algorithm (preamble-based).
///
/// Uses two identical halves of a known preamble to estimate CFO.
/// - `first_half`: first copy of repeated symbol
/// - `second_half`: second copy of repeated symbol
/// - `sample_rate`: sample rate in Hz
///
/// Returns estimated CFO in Hz.
pub fn estimate_cfo_moose(
    first_half: &[Complex64],
    second_half: &[Complex64],
    sample_rate: f64,
) -> f64 {
    let n = first_half.len().min(second_half.len());
    if n == 0 {
        return 0.0;
    }

    let mut sum = Complex64::new(0.0, 0.0);
    for i in 0..n {
        sum += second_half[i] * first_half[i].conj();
    }

    let phase = sum.arg();
    phase * sample_rate / (2.0 * PI * n as f64)
}

/// Estimate CFO using the Kay algorithm (ML estimator for single tone).
///
/// Weighted phase-difference estimator with lower variance than
/// simple autocorrelation. Optimal for single-tone signals in AWGN.
///
/// - `samples`: complex input samples
/// - `sample_rate`: sample rate in Hz
///
/// Returns estimated CFO in Hz.
pub fn estimate_cfo_kay(samples: &[Complex64], sample_rate: f64) -> f64 {
    let n = samples.len();
    if n < 2 {
        return 0.0;
    }

    // Kay weights: w[k] = 6*k*(N-1-k) / (N*(N^2-1))
    let nf = n as f64;
    let denom = nf * (nf * nf - 1.0);

    let mut weighted_sum = 0.0;
    for k in 1..n {
        let phase_diff = (samples[k] * samples[k - 1].conj()).arg();
        let weight = 6.0 * k as f64 * (n - 1 - k) as f64 / denom;
        weighted_sum += weight * phase_diff;
    }

    weighted_sum * sample_rate / (2.0 * PI)
}

/// Streaming CFO estimator with averaging.
#[derive(Debug, Clone)]
pub struct CfoEstimator {
    /// Correlation lag.
    lag: usize,
    /// Sample rate.
    sample_rate: f64,
    /// Exponential averaging factor.
    alpha: f64,
    /// Current CFO estimate.
    estimate: f64,
    /// Previous sample for lag-1 computation.
    prev_samples: Vec<Complex64>,
}

impl CfoEstimator {
    /// Create a new streaming CFO estimator.
    ///
    /// - `lag`: autocorrelation lag
    /// - `sample_rate`: in Hz
    /// - `alpha`: averaging factor (0.0 = no update, 1.0 = no averaging)
    pub fn new(lag: usize, sample_rate: f64, alpha: f64) -> Self {
        Self {
            lag: lag.max(1),
            sample_rate,
            alpha: alpha.clamp(0.0, 1.0),
            estimate: 0.0,
            prev_samples: Vec::new(),
        }
    }

    /// Process a block of samples, updating the CFO estimate.
    ///
    /// Returns the current averaged CFO estimate in Hz.
    pub fn process(&mut self, samples: &[Complex64]) -> f64 {
        // Combine with previous tail for continuity
        let mut all = std::mem::take(&mut self.prev_samples);
        all.extend_from_slice(samples);

        let instant = estimate_cfo_autocorr(&all, self.lag, self.sample_rate);
        self.estimate = self.alpha * instant + (1.0 - self.alpha) * self.estimate;

        // Keep last `lag` samples for next block
        if all.len() > self.lag {
            self.prev_samples = all[all.len() - self.lag..].to_vec();
        } else {
            self.prev_samples = all;
        }

        self.estimate
    }

    /// Get current CFO estimate in Hz.
    pub fn estimate_hz(&self) -> f64 {
        self.estimate
    }

    /// Get current CFO estimate as fraction of sample rate.
    pub fn estimate_normalized(&self) -> f64 {
        self.estimate / self.sample_rate
    }

    /// Reset the estimator.
    pub fn reset(&mut self) {
        self.estimate = 0.0;
        self.prev_samples.clear();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_cfo_signal(n: usize, cfo_hz: f64, fs: f64) -> Vec<Complex64> {
        (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                Complex64::from_polar(1.0, 2.0 * PI * cfo_hz * t)
            })
            .collect()
    }

    #[test]
    fn test_autocorr_zero_cfo() {
        let samples = vec![Complex64::new(1.0, 0.0); 256];
        let cfo = estimate_cfo_autocorr(&samples, 1, 1000.0);
        assert!(cfo.abs() < 1.0);
    }

    #[test]
    fn test_autocorr_positive_cfo() {
        let signal = make_cfo_signal(512, 50.0, 1000.0);
        let cfo = estimate_cfo_autocorr(&signal, 1, 1000.0);
        assert!((cfo - 50.0).abs() < 5.0, "expected ~50 Hz, got {cfo}");
    }

    #[test]
    fn test_autocorr_negative_cfo() {
        let signal = make_cfo_signal(512, -75.0, 1000.0);
        let cfo = estimate_cfo_autocorr(&signal, 1, 1000.0);
        assert!((cfo - (-75.0)).abs() < 5.0, "expected ~-75 Hz, got {cfo}");
    }

    #[test]
    fn test_moose() {
        // Moose uses two identical repeated symbols with CFO applied.
        // Max unambiguous CFO = fs / (2*N), so use small N for large CFO.
        let fs = 1000.0;
        let cfo_hz = 30.0;
        let n = 8; // Max CFO = 1000/(2*8) = 62.5 Hz → safe for 30 Hz
        let first: Vec<Complex64> = (0..n)
            .map(|i| Complex64::from_polar(1.0, 2.0 * PI * cfo_hz * i as f64 / fs))
            .collect();
        let second: Vec<Complex64> = (0..n)
            .map(|i| Complex64::from_polar(1.0, 2.0 * PI * cfo_hz * (i + n) as f64 / fs))
            .collect();
        let est = estimate_cfo_moose(&first, &second, fs);
        assert!((est - cfo_hz).abs() < 5.0, "Moose: expected ~30 Hz, got {est}");
    }

    #[test]
    fn test_kay() {
        let signal = make_cfo_signal(256, 80.0, 1000.0);
        let cfo = estimate_cfo_kay(&signal, 1000.0);
        assert!((cfo - 80.0).abs() < 5.0, "Kay: expected ~80 Hz, got {cfo}");
    }

    #[test]
    fn test_streaming() {
        let signal = make_cfo_signal(1024, 60.0, 1000.0);
        let mut est = CfoEstimator::new(1, 1000.0, 0.5);
        // Process in blocks
        for chunk in signal.chunks(256) {
            est.process(chunk);
        }
        let cfo = est.estimate_hz();
        assert!((cfo - 60.0).abs() < 10.0, "Streaming: expected ~60 Hz, got {cfo}");
    }

    #[test]
    fn test_streaming_normalized() {
        let mut est = CfoEstimator::new(1, 1000.0, 1.0);
        let signal = make_cfo_signal(256, 100.0, 1000.0);
        est.process(&signal);
        let norm = est.estimate_normalized();
        assert!((norm - 0.1).abs() < 0.01);
    }

    #[test]
    fn test_reset() {
        let mut est = CfoEstimator::new(1, 1000.0, 1.0);
        let signal = make_cfo_signal(256, 100.0, 1000.0);
        est.process(&signal);
        est.reset();
        assert_eq!(est.estimate_hz(), 0.0);
    }

    #[test]
    fn test_empty_input() {
        assert_eq!(estimate_cfo_autocorr(&[], 1, 1000.0), 0.0);
        assert_eq!(estimate_cfo_moose(&[], &[], 1000.0), 0.0);
        assert_eq!(estimate_cfo_kay(&[], 1000.0), 0.0);
    }

    #[test]
    fn test_single_sample() {
        let s = vec![Complex64::new(1.0, 0.0)];
        assert_eq!(estimate_cfo_autocorr(&s, 1, 1000.0), 0.0);
        assert_eq!(estimate_cfo_kay(&s, 1000.0), 0.0);
    }

    #[test]
    fn test_larger_lag() {
        let signal = make_cfo_signal(512, 40.0, 1000.0);
        let cfo = estimate_cfo_autocorr(&signal, 4, 1000.0);
        assert!((cfo - 40.0).abs() < 5.0, "lag=4: expected ~40 Hz, got {cfo}");
    }
}
