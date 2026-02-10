//! Wideband Signal Presence Detection via Energy/Power Threshold Crossing
//!
//! This module implements energy detection for wideband signal presence
//! sensing with adaptive noise floor estimation. It provides both simple
//! one-shot detection and stateful detectors with exponential moving average
//! noise floor tracking, per-subband detection, and theoretical detection
//! probability calculations.
//!
//! # Overview
//!
//! Energy detection is the simplest and most common method for spectrum
//! sensing in cognitive radio and SDR applications. The detector computes
//! the power spectral density (via DFT), estimates the noise floor using
//! an exponential moving average, and declares signal presence when the
//! observed power exceeds the noise floor by a configurable threshold.
//!
//! # Example
//!
//! ```rust
//! # use r4w_core::energy_detector::{EnergyDetector, DetectionResult};
//! // Create a detector with 64-point FFT and 10 dB threshold
//! let mut detector = EnergyDetector::new(64, 10.0);
//!
//! // Generate a strong tone (signal present)
//! let samples: Vec<(f64, f64)> = (0..64)
//!     .map(|i| {
//!         let phase = 2.0 * std::f64::consts::PI * 0.1 * i as f64;
//!         (phase.cos() * 10.0, phase.sin() * 10.0)
//!     })
//!     .collect();
//!
//! let result = detector.detect(&samples);
//! assert!(result.power_db > -50.0); // significant power present
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// DetectionResult
// ---------------------------------------------------------------------------

/// Result from an energy detection operation.
#[derive(Debug, Clone)]
pub struct DetectionResult {
    /// Whether the signal power exceeds the noise floor + threshold.
    pub detected: bool,
    /// Total observed power in dB.
    pub power_db: f64,
    /// Estimated noise floor in dB.
    pub noise_floor_db: f64,
    /// Estimated SNR in dB (`power_db - noise_floor_db`).
    pub snr_db: f64,
    /// Per-bin detection mask: `true` for each frequency bin whose power
    /// exceeds the noise floor + threshold.
    pub frequency_bins: Vec<bool>,
}

// ---------------------------------------------------------------------------
// NoiseFloorEstimator
// ---------------------------------------------------------------------------

/// Exponential moving average noise floor estimator.
///
/// Tracks the noise floor of a power spectrum using EMA smoothing.
/// A smaller `averaging_factor` gives a longer memory (slower adaptation).
#[derive(Debug, Clone)]
pub struct NoiseFloorEstimator {
    /// EMA smoothing factor in (0, 1].
    alpha: f64,
    /// Current noise floor estimate (linear power).
    estimate: Option<f64>,
}

impl NoiseFloorEstimator {
    /// Create a new noise floor estimator.
    ///
    /// `averaging_factor` controls the EMA weight for new observations.
    /// Typical values: 0.01 (slow) to 0.2 (fast).
    pub fn new(averaging_factor: f64) -> Self {
        Self {
            alpha: averaging_factor.clamp(1e-6, 1.0),
            estimate: None,
        }
    }

    /// Feed a power spectrum and return the updated noise floor estimate
    /// (linear power).
    ///
    /// The noise floor is taken as the median of the power spectrum bins,
    /// then smoothed with an EMA. The median is robust against narrowband
    /// signals that occupy only a fraction of the bins.
    pub fn update(&mut self, power_spectrum: &[f64]) -> f64 {
        if power_spectrum.is_empty() {
            return self.estimate.unwrap_or(0.0);
        }

        // Use the median bin power as the instantaneous noise floor.
        let mut sorted: Vec<f64> = power_spectrum.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median = sorted[sorted.len() / 2];

        match self.estimate {
            Some(prev) => {
                let updated = (1.0 - self.alpha) * prev + self.alpha * median;
                self.estimate = Some(updated);
                updated
            }
            None => {
                self.estimate = Some(median);
                median
            }
        }
    }

    /// Reset the estimator, discarding all history.
    pub fn reset(&mut self) {
        self.estimate = None;
    }

    /// Return the current noise floor estimate (linear power), or 0 if
    /// no data has been fed yet.
    fn current(&self) -> f64 {
        self.estimate.unwrap_or(0.0)
    }
}

// ---------------------------------------------------------------------------
// EnergyDetector
// ---------------------------------------------------------------------------

/// Wideband energy detector with adaptive noise floor.
///
/// Computes the power spectral density of the input via DFT, maintains a
/// running noise floor estimate, and declares detection when the total
/// power exceeds the noise floor by at least `threshold_db`.
#[derive(Debug, Clone)]
pub struct EnergyDetector {
    fft_size: usize,
    threshold_db: f64,
    noise_estimator: NoiseFloorEstimator,
}

impl EnergyDetector {
    /// Create a new energy detector.
    ///
    /// * `fft_size` - Number of frequency bins (DFT length).
    /// * `threshold_db` - Detection threshold above the noise floor, in dB.
    pub fn new(fft_size: usize, threshold_db: f64) -> Self {
        let fft_size = fft_size.max(2);
        Self {
            fft_size,
            threshold_db,
            noise_estimator: NoiseFloorEstimator::new(0.1),
        }
    }

    /// Run detection on a block of complex samples `(I, Q)`.
    ///
    /// The input is zero-padded or truncated to `fft_size` samples, then
    /// a DFT is computed, the power spectrum is formed, the noise floor is
    /// updated, and per-bin and aggregate detection decisions are made.
    pub fn detect(&mut self, samples: &[(f64, f64)]) -> DetectionResult {
        let spectrum = power_spectrum(samples, self.fft_size);

        // Update noise floor
        let noise_linear = self.noise_estimator.update(&spectrum);
        let noise_db = linear_to_db(noise_linear);

        // Total power (mean across bins)
        let total_power = spectrum.iter().copied().sum::<f64>() / spectrum.len() as f64;
        let power_db = linear_to_db(total_power);

        let snr_db = power_db - noise_db;
        let detected = snr_db >= self.threshold_db;

        // Per-bin detection: threshold is noise_linear * 10^(threshold_db/10)
        let bin_threshold = noise_linear * db_to_linear(self.threshold_db);
        let frequency_bins: Vec<bool> = spectrum.iter().map(|&p| p >= bin_threshold).collect();

        DetectionResult {
            detected,
            power_db,
            noise_floor_db: noise_db,
            snr_db,
            frequency_bins,
        }
    }

    /// Change the detection threshold.
    pub fn set_threshold(&mut self, threshold_db: f64) {
        self.threshold_db = threshold_db;
    }

    /// Return the current noise floor estimate in dB.
    pub fn noise_floor_db(&self) -> f64 {
        linear_to_db(self.noise_estimator.current())
    }
}

// ---------------------------------------------------------------------------
// SubbandDetector
// ---------------------------------------------------------------------------

/// Per-subband energy detector.
///
/// Divides the DFT bins into `num_subbands` equal groups and performs
/// independent energy detection in each subband.
#[derive(Debug, Clone)]
pub struct SubbandDetector {
    num_subbands: usize,
    fft_size: usize,
    threshold_db: f64,
    noise_estimators: Vec<NoiseFloorEstimator>,
}

impl SubbandDetector {
    /// Create a subband detector.
    ///
    /// * `num_subbands` - Number of frequency sub-bands.
    /// * `fft_size` - DFT length.
    /// * `threshold_db` - Detection threshold per subband in dB.
    pub fn new(num_subbands: usize, fft_size: usize, threshold_db: f64) -> Self {
        let num_subbands = num_subbands.max(1);
        let fft_size = fft_size.max(num_subbands);
        Self {
            num_subbands,
            fft_size,
            threshold_db,
            noise_estimators: (0..num_subbands)
                .map(|_| NoiseFloorEstimator::new(0.1))
                .collect(),
        }
    }

    /// Detect signal presence in each subband.
    ///
    /// Returns a `Vec<bool>` of length `num_subbands` indicating which
    /// subbands have signal energy above the threshold.
    pub fn detect(&mut self, samples: &[(f64, f64)]) -> Vec<bool> {
        let spectrum = power_spectrum(samples, self.fft_size);
        let bins_per_subband = self.fft_size / self.num_subbands;

        let mut results = Vec::with_capacity(self.num_subbands);

        for (i, estimator) in self.noise_estimators.iter_mut().enumerate() {
            let start = i * bins_per_subband;
            let end = if i == self.num_subbands - 1 {
                self.fft_size
            } else {
                start + bins_per_subband
            };
            let subband = &spectrum[start..end];

            let noise_linear = estimator.update(subband);
            let mean_power = subband.iter().copied().sum::<f64>() / subband.len() as f64;
            let noise_db = linear_to_db(noise_linear);
            let power_db = linear_to_db(mean_power);
            let snr_db = power_db - noise_db;

            results.push(snr_db >= self.threshold_db);
        }

        results
    }
}

// ---------------------------------------------------------------------------
// Free functions
// ---------------------------------------------------------------------------

/// One-shot energy detection.
///
/// Returns `true` if the total power of `samples` exceeds `threshold_db`
/// (in dBFS, i.e., compared against 0 dB full-scale).
pub fn energy_detect_simple(samples: &[(f64, f64)], threshold_db: f64) -> bool {
    if samples.is_empty() {
        return false;
    }
    let power = samples
        .iter()
        .map(|&(i, q)| i * i + q * q)
        .sum::<f64>()
        / samples.len() as f64;
    let power_db = linear_to_db(power);
    power_db >= threshold_db
}

/// Theoretical probability of detection (Pd) for an energy detector under
/// the Neyman-Pearson criterion.
///
/// Uses the Gaussian approximation to the central and non-central
/// chi-squared distributions, valid for large `num_samples`.
///
/// * `snr_linear` - Signal-to-noise ratio in linear scale.
/// * `num_samples` - Number of samples integrated.
/// * `pfa` - Probability of false alarm.
///
/// Returns the probability of detection in \[0, 1\].
pub fn probability_of_detection(snr_linear: f64, num_samples: usize, pfa: f64) -> f64 {
    let n = num_samples as f64;
    if n < 1.0 || pfa <= 0.0 || pfa >= 1.0 {
        return 0.0;
    }

    // Threshold factor from Pfa (inverse Q-function applied to Pfa)
    // Q^{-1}(Pfa) using the rational approximation
    let q_inv_pfa = inv_q(pfa);

    // Under H0 (noise only): test statistic T ~ N(n, n) for large n
    // gamma = threshold = n + sqrt(n) * Q^{-1}(Pfa)
    // Under H1: T ~ N(n(1+snr), n(1+2*snr))
    // Pd = Q( (gamma - n(1+snr)) / sqrt(n(1+2*snr)) )
    let gamma = n + n.sqrt() * q_inv_pfa;
    let mu1 = n * (1.0 + snr_linear);
    let sigma1 = (n * (1.0 + 2.0 * snr_linear)).sqrt();

    if sigma1 <= 0.0 {
        return if mu1 > gamma { 1.0 } else { 0.0 };
    }

    let z = (gamma - mu1) / sigma1;
    q_function(z)
}

/// Compute the detection threshold (linear scale) for a target
/// probability of false alarm.
///
/// Uses the Gaussian approximation: `threshold = N + sqrt(N) * Q^{-1}(Pfa)`
/// where N is `num_samples`.
///
/// Returns the threshold in linear scale (sum of squared magnitudes).
pub fn threshold_from_pfa(pfa: f64, num_samples: usize) -> f64 {
    let n = num_samples as f64;
    if n < 1.0 || pfa <= 0.0 || pfa >= 1.0 {
        return n;
    }
    let q_inv = inv_q(pfa);
    n + n.sqrt() * q_inv
}

/// Factory: create an energy detector sized for a given bandwidth and
/// sample rate.
///
/// The FFT size is chosen as the smallest power of two that yields at
/// least 1 Hz resolution, clamped to \[16, 65536\].
pub fn wideband_detector(bandwidth_hz: f64, sample_rate: f64) -> EnergyDetector {
    let resolution = bandwidth_hz.abs().max(1.0);
    let raw = (sample_rate / resolution).ceil() as usize;
    // Round up to next power of two
    let fft_size = next_power_of_two(raw).clamp(16, 65536);
    EnergyDetector::new(fft_size, 6.0)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Compute the power spectrum of complex samples via a DFT.
///
/// This is an `O(N^2)` DFT (no FFT library dependency). Fine for the
/// moderate sizes used in detection (typically <=4096). The output has
/// `fft_size` bins, each containing `|X[k]|^2 / N^2`.
fn power_spectrum(samples: &[(f64, f64)], fft_size: usize) -> Vec<f64> {
    let n = fft_size;
    let mut spectrum = vec![0.0f64; n];

    for k in 0..n {
        let mut re_sum = 0.0f64;
        let mut im_sum = 0.0f64;
        for (idx, &(si, sq)) in samples.iter().enumerate().take(n) {
            let angle = -2.0 * PI * (k as f64) * (idx as f64) / (n as f64);
            let (sin_a, cos_a) = angle.sin_cos();
            re_sum += si * cos_a - sq * sin_a;
            im_sum += si * sin_a + sq * cos_a;
        }
        spectrum[k] = (re_sum * re_sum + im_sum * im_sum) / (n as f64 * n as f64);
    }

    spectrum
}

/// Convert linear power to dB.  Returns -200 for zero/negative input.
fn linear_to_db(p: f64) -> f64 {
    if p <= 0.0 {
        -200.0
    } else {
        10.0 * p.log10()
    }
}

/// Convert dB to linear power.
fn db_to_linear(db: f64) -> f64 {
    10.0_f64.powf(db / 10.0)
}

/// Q-function: Q(x) = 0.5 * erfc(x / sqrt(2))
///
/// Uses a rational approximation to erfc for moderate accuracy.
fn q_function(x: f64) -> f64 {
    0.5 * erfc_approx(x / std::f64::consts::SQRT_2)
}

/// Inverse Q-function: returns z such that Q(z) = p.
///
/// Uses a rational approximation to the inverse of the normal CDF.
fn inv_q(p: f64) -> f64 {
    if p <= 0.0 {
        return f64::INFINITY;
    }
    if p >= 1.0 {
        return f64::NEG_INFINITY;
    }
    // Q(z) = p  =>  erfc(z/sqrt(2))/2 = p  =>  erfc(z/sqrt(2)) = 2p
    // z/sqrt(2) = erfc_inv(2p)
    // z = sqrt(2) * erfc_inv(2p)
    //
    // Use the Beasley-Springer-Moro approximation for the inverse normal CDF.
    // Phi^{-1}(1-p) = Q^{-1}(p)
    inv_normal_cdf(1.0 - p)
}

/// Approximate erfc(x) using the Abramowitz & Stegun approximation 7.1.26.
fn erfc_approx(x: f64) -> f64 {
    if x < 0.0 {
        return 2.0 - erfc_approx(-x);
    }
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t
        * (0.254829592
            + t * (-0.284496736
                + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    poly * (-x * x).exp()
}

/// Inverse of the standard normal CDF using the rational approximation
/// from Peter Acklam. Accurate to ~1.15e-9.
fn inv_normal_cdf(p: f64) -> f64 {
    // Coefficients
    const A: [f64; 6] = [
        -3.969683028665376e+01,
        2.209460984245205e+02,
        -2.759285104469687e+02,
        1.383577518672690e+02,
        -3.066479806614716e+01,
        2.506628277459239e+00,
    ];
    const B: [f64; 5] = [
        -5.447609879822406e+01,
        1.615858368580409e+02,
        -1.556989798598866e+02,
        6.680131188771972e+01,
        -1.328068155288572e+01,
    ];
    const C: [f64; 6] = [
        -7.784894002430293e-03,
        -3.223964580411365e-01,
        -2.400758277161838e+00,
        -2.549732539343734e+00,
        4.374664141464968e+00,
        2.938163982698783e+00,
    ];
    const D: [f64; 4] = [
        7.784695709041462e-03,
        3.224671290700398e-01,
        2.445134137142996e+00,
        3.754408661907416e+00,
    ];

    const P_LOW: f64 = 0.02425;
    const P_HIGH: f64 = 1.0 - P_LOW;

    if p <= 0.0 {
        return f64::NEG_INFINITY;
    }
    if p >= 1.0 {
        return f64::INFINITY;
    }

    if p < P_LOW {
        // Rational approximation for lower region
        let q = (-2.0 * p.ln()).sqrt();
        (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
    } else if p <= P_HIGH {
        // Rational approximation for central region
        let q = p - 0.5;
        let r = q * q;
        (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5]) * q
            / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r + 1.0)
    } else {
        // Rational approximation for upper region
        let q = (-2.0 * (1.0 - p).ln()).sqrt();
        -(((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0)
    }
}

/// Smallest power of two >= n.
fn next_power_of_two(n: usize) -> usize {
    if n == 0 {
        return 1;
    }
    1_usize << (usize::BITS - (n - 1).leading_zeros())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Simple deterministic pseudo-random number generator (LCG) for tests.
    /// Returns values in [-1, 1).
    struct TestRng(u64);
    impl TestRng {
        fn new(seed: u64) -> Self {
            Self(seed)
        }
        fn next_f64(&mut self) -> f64 {
            // LCG parameters from Numerical Recipes
            self.0 = self.0.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            // Map to [-1, 1)
            (self.0 >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0
        }
    }

    /// Helper: generate broadband noise-only samples using LCG PRNG.
    fn noise_samples(n: usize, amplitude: f64) -> Vec<(f64, f64)> {
        let mut rng = TestRng::new(42);
        (0..n)
            .map(|_| {
                (rng.next_f64() * amplitude, rng.next_f64() * amplitude)
            })
            .collect()
    }

    /// Helper: generate a strong single-tone signal.
    fn tone_samples(n: usize, amplitude: f64, freq_norm: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let phase = 2.0 * PI * freq_norm * i as f64;
                (phase.cos() * amplitude, phase.sin() * amplitude)
            })
            .collect()
    }

    #[test]
    fn test_construction_and_defaults() {
        let det = EnergyDetector::new(128, 10.0);
        assert_eq!(det.fft_size, 128);
        assert!((det.threshold_db - 10.0).abs() < 1e-12);
        // Before any data, noise floor should be -200 (no estimate)
        assert!(det.noise_floor_db() < -100.0);
    }

    #[test]
    fn test_detect_strong_signal() {
        let mut det = EnergyDetector::new(64, 6.0);

        // First, feed noise to establish a noise floor
        let noise = noise_samples(64, 0.001);
        for _ in 0..10 {
            det.detect(&noise);
        }

        // Now feed a strong tone
        let signal = tone_samples(64, 5.0, 0.25);
        let result = det.detect(&signal);
        assert!(
            result.detected,
            "Strong signal should be detected. SNR_dB={:.1}, power_dB={:.1}, noise_dB={:.1}",
            result.snr_db, result.power_db, result.noise_floor_db
        );
        assert!(result.snr_db > 6.0);
    }

    #[test]
    fn test_no_detection_noise_only() {
        let mut det = EnergyDetector::new(64, 10.0);

        // Feed the same noise repeatedly to converge the noise floor
        let noise = noise_samples(64, 0.01);
        let mut result = det.detect(&noise);
        for _ in 0..50 {
            result = det.detect(&noise);
        }

        // After convergence, noise-only should NOT trigger detection
        // because the power should be very close to the noise floor.
        assert!(
            !result.detected,
            "Noise-only should not be detected. SNR_dB={:.1}",
            result.snr_db
        );
    }

    #[test]
    fn test_noise_floor_estimation_convergence() {
        let mut estimator = NoiseFloorEstimator::new(0.3);

        // Feed a constant power spectrum
        let spectrum = vec![1.0; 64];
        let mut estimate = 0.0;
        for _ in 0..50 {
            estimate = estimator.update(&spectrum);
        }

        // Should converge to the median (1.0)
        assert!(
            (estimate - 1.0).abs() < 0.01,
            "Noise floor should converge to 1.0, got {}",
            estimate
        );
    }

    #[test]
    fn test_threshold_setting() {
        let mut det = EnergyDetector::new(64, 3.0);
        assert!((det.threshold_db - 3.0).abs() < 1e-12);

        det.set_threshold(15.0);
        assert!((det.threshold_db - 15.0).abs() < 1e-12);

        // With a very high threshold, even a moderate signal should not trigger
        det.set_threshold(100.0);
        let noise = noise_samples(64, 0.01);
        for _ in 0..20 {
            det.detect(&noise);
        }
        let signal = tone_samples(64, 1.0, 0.2);
        let result = det.detect(&signal);
        assert!(
            !result.detected,
            "Very high threshold should prevent detection"
        );
    }

    #[test]
    fn test_simple_detection_function() {
        // Very strong signal: should be detected above -10 dBFS
        let strong = tone_samples(256, 10.0, 0.1);
        assert!(
            energy_detect_simple(&strong, -10.0),
            "Strong signal should be detected"
        );

        // Very weak signal: should not exceed 0 dBFS
        let weak = noise_samples(256, 0.001);
        assert!(
            !energy_detect_simple(&weak, 0.0),
            "Weak noise should not exceed 0 dBFS"
        );

        // Empty input
        assert!(!energy_detect_simple(&[], -100.0));
    }

    #[test]
    fn test_subband_detection() {
        let fft_size = 64;
        let num_subbands = 4;
        let mut det = SubbandDetector::new(num_subbands, fft_size, 6.0);

        // First establish noise floor
        let noise = noise_samples(fft_size, 0.001);
        for _ in 0..20 {
            det.detect(&noise);
        }

        // Inject a tone at normalized frequency 0.25 (bin 16 of 64),
        // which falls in the second subband (bins 16-31)
        let signal: Vec<(f64, f64)> = (0..fft_size)
            .map(|i| {
                let phase = 2.0 * PI * 0.25 * i as f64;
                (phase.cos() * 5.0 + noise[i].0, phase.sin() * 5.0 + noise[i].1)
            })
            .collect();

        let result = det.detect(&signal);
        assert_eq!(result.len(), num_subbands);

        // At least one subband should detect the signal
        assert!(
            result.iter().any(|&d| d),
            "At least one subband should detect the tone"
        );
    }

    #[test]
    fn test_probability_of_detection() {
        // At high SNR, Pd should be close to 1.0
        let pd_high = probability_of_detection(10.0, 1000, 0.01);
        assert!(
            pd_high > 0.99,
            "Pd at high SNR should be near 1.0, got {}",
            pd_high
        );

        // At zero SNR, Pd should be close to Pfa
        let pd_zero = probability_of_detection(0.0, 1000, 0.01);
        assert!(
            (pd_zero - 0.01).abs() < 0.05,
            "Pd at SNR=0 should be near Pfa, got {}",
            pd_zero
        );

        // At moderate SNR, Pd should be between Pfa and 1
        let pd_mod = probability_of_detection(0.5, 100, 0.01);
        assert!(pd_mod > 0.01 && pd_mod < 1.0);
    }

    #[test]
    fn test_threshold_from_pfa() {
        let n = 1000;
        let pfa = 0.01;
        let thr = threshold_from_pfa(pfa, n);

        // Threshold should be greater than N (mean under H0)
        assert!(
            thr > n as f64,
            "Threshold should exceed N={}, got {}",
            n,
            thr
        );

        // Lower Pfa => higher threshold
        let thr_low = threshold_from_pfa(0.001, n);
        let thr_high = threshold_from_pfa(0.1, n);
        assert!(
            thr_low > thr_high,
            "Lower Pfa should give higher threshold: {} vs {}",
            thr_low,
            thr_high
        );
    }

    #[test]
    fn test_wideband_factory() {
        let det = wideband_detector(1000.0, 48000.0);
        // FFT size should be at least 48000/1000 = 48, rounded to next pow2 = 64
        assert!(
            det.fft_size >= 48,
            "FFT size should be >= 48, got {}",
            det.fft_size
        );
        // Should be a power of two
        assert!(
            det.fft_size.is_power_of_two(),
            "FFT size should be power of two, got {}",
            det.fft_size
        );
        // Default threshold should be 6 dB
        assert!((det.threshold_db - 6.0).abs() < 1e-12);
    }
}
