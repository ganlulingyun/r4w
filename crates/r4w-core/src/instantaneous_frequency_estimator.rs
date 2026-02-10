//! Instantaneous frequency estimation from analytic (complex) signals.
//!
//! This module computes time-varying instantaneous frequency using phase
//! differentiation, the cross-product (Kay) estimator, and zero-crossing
//! analysis. It also provides utilities for Hilbert transform, phase
//! unwrapping, smoothed IF estimation, IF jitter measurement, and chirp-rate
//! estimation.
//!
//! Complex samples are represented as `(f64, f64)` tuples `(re, im)`.
//!
//! # Example
//!
//! ```
//! use r4w_core::instantaneous_frequency_estimator::InstantaneousFrequencyEstimator;
//!
//! let sample_rate = 1000.0;
//! let est = InstantaneousFrequencyEstimator::new(sample_rate);
//!
//! // Generate a 100 Hz tone: exp(j * 2pi * 100 * t)
//! let n = 64;
//! let signal: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / sample_rate;
//!         let phase = 2.0 * std::f64::consts::PI * 100.0 * t;
//!         (phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let freqs = est.phase_diff(&signal);
//! // Interior samples should be close to 100 Hz
//! for &f in &freqs[1..freqs.len().saturating_sub(1)] {
//!     assert!((f - 100.0).abs() < 1.0, "got {f}");
//! }
//! ```

use std::f64::consts::PI;

// --- helpers -----------------------------------------------------------------

/// Complex multiplication: (a+jb)(c+jd)
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex conjugate
#[inline]
fn conj(z: (f64, f64)) -> (f64, f64) {
    (z.0, -z.1)
}

/// atan2 of a complex sample, giving the phase angle in radians.
#[inline]
fn carg(z: (f64, f64)) -> f64 {
    z.1.atan2(z.0)
}

// --- public API --------------------------------------------------------------

/// Estimates instantaneous frequency from analytic (complex) signals.
///
/// All frequency outputs are in Hz.
#[derive(Debug, Clone)]
pub struct InstantaneousFrequencyEstimator {
    sample_rate: f64,
}

impl InstantaneousFrequencyEstimator {
    /// Create a new estimator with the given sample rate (samples/sec).
    ///
    /// # Panics
    ///
    /// Panics if `sample_rate` is not positive and finite.
    pub fn new(sample_rate: f64) -> Self {
        assert!(
            sample_rate > 0.0 && sample_rate.is_finite(),
            "sample_rate must be positive and finite"
        );
        Self { sample_rate }
    }

    /// Return the configured sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    // -- Phase differentiation ------------------------------------------------

    /// Estimate instantaneous frequency via phase differentiation.
    ///
    /// Steps: compute the instantaneous phase of each sample, unwrap it, then
    /// take the discrete derivative and scale to Hz.
    ///
    /// Returns a vector of length `signal.len()`. The first sample is set
    /// equal to the second to avoid an undefined derivative at the boundary.
    pub fn phase_diff(&self, signal: &[(f64, f64)]) -> Vec<f64> {
        if signal.len() < 2 {
            return vec![0.0; signal.len()];
        }

        let phases: Vec<f64> = signal.iter().map(|&s| carg(s)).collect();
        let unwrapped = Self::unwrap_phase(&phases);

        let scale = self.sample_rate / (2.0 * PI);
        let mut freq = Vec::with_capacity(unwrapped.len());

        // Forward difference for all but the last, backward for the last.
        for i in 0..unwrapped.len() {
            if i < unwrapped.len() - 1 {
                freq.push((unwrapped[i + 1] - unwrapped[i]) * scale);
            } else {
                freq.push((unwrapped[i] - unwrapped[i - 1]) * scale);
            }
        }

        // Copy second value into first for smoother boundary.
        if freq.len() >= 2 {
            freq[0] = freq[1];
        }
        freq
    }

    // -- Cross-product (Kay) estimator ----------------------------------------

    /// Cross-product (Kay) instantaneous frequency estimator.
    ///
    /// Uses `Im{ x[n] * conj(x[n-1]) }` which avoids explicit phase
    /// unwrapping and is more robust to noise.
    ///
    /// Returns a vector of length `signal.len()`.  The first element is
    /// duplicated from the second.
    pub fn kay_estimator(&self, signal: &[(f64, f64)]) -> Vec<f64> {
        if signal.len() < 2 {
            return vec![0.0; signal.len()];
        }

        let scale = self.sample_rate / (2.0 * PI);
        let mut freq = Vec::with_capacity(signal.len());

        freq.push(0.0); // placeholder for index 0

        for i in 1..signal.len() {
            let prod = cmul(signal[i], conj(signal[i - 1]));
            let angle = carg(prod);
            freq.push(angle * scale);
        }

        freq[0] = freq[1];
        freq
    }

    // -- Hilbert transform ----------------------------------------------------

    /// Create an analytic signal from a real-valued input using a
    /// frequency-domain Hilbert transform.
    ///
    /// The returned vector has the same length as `real_signal`. The real
    /// part equals the input and the imaginary part is the Hilbert
    /// transform.
    pub fn hilbert(real_signal: &[f64]) -> Vec<(f64, f64)> {
        let n = real_signal.len();
        if n == 0 {
            return vec![];
        }

        // DFT (naive O(n^2) -- acceptable for the sizes we target)
        let mut spectrum: Vec<(f64, f64)> = vec![(0.0, 0.0); n];
        for k in 0..n {
            let mut re = 0.0;
            let mut im = 0.0;
            for (t, &x) in real_signal.iter().enumerate() {
                let angle = -2.0 * PI * k as f64 * t as f64 / n as f64;
                re += x * angle.cos();
                im += x * angle.sin();
            }
            spectrum[k] = (re, im);
        }

        // Apply the analytic-signal multiplier:
        //   H[0] = 1, H[n/2] = 1 (if n even)
        //   H[1..n/2] = 2
        //   H[n/2+1..] = 0
        let half = n / 2;
        for k in 0..n {
            if k == 0 || (n % 2 == 0 && k == half) {
                // multiply by 1 -- no-op
            } else if k <= half {
                spectrum[k].0 *= 2.0;
                spectrum[k].1 *= 2.0;
            } else {
                spectrum[k] = (0.0, 0.0);
            }
        }

        // Inverse DFT
        let inv_n = 1.0 / n as f64;
        let mut analytic = Vec::with_capacity(n);
        for t in 0..n {
            let mut re = 0.0;
            let mut im = 0.0;
            for (k, &s) in spectrum.iter().enumerate() {
                let angle = 2.0 * PI * k as f64 * t as f64 / n as f64;
                // (s.re + j*s.im) * (cos a + j sin a)
                re += s.0 * angle.cos() - s.1 * angle.sin();
                im += s.0 * angle.sin() + s.1 * angle.cos();
            }
            analytic.push((re * inv_n, im * inv_n));
        }

        analytic
    }

    // -- Phase unwrapping -----------------------------------------------------

    /// Unwrap a phase sequence so that successive differences never exceed pi.
    pub fn unwrap_phase(phases: &[f64]) -> Vec<f64> {
        if phases.is_empty() {
            return vec![];
        }
        let mut out = Vec::with_capacity(phases.len());
        out.push(phases[0]);
        for i in 1..phases.len() {
            let mut d = phases[i] - phases[i - 1];
            // Bring d into (-pi, pi]
            while d > PI {
                d -= 2.0 * PI;
            }
            while d <= -PI {
                d += 2.0 * PI;
            }
            out.push(out[i - 1] + d);
        }
        out
    }

    // -- Smoothed IF estimation -----------------------------------------------

    /// Estimate instantaneous frequency with a moving-average smoother.
    ///
    /// Uses the Kay estimator internally, then applies a centered
    /// moving-average window of the given `window_size` (clamped to an odd
    /// value >= 1).
    pub fn smoothed_if(&self, signal: &[(f64, f64)], window_size: usize) -> Vec<f64> {
        let raw = self.kay_estimator(signal);
        let win = if window_size < 1 { 1 } else { window_size | 1 }; // ensure odd & >=1
        if win <= 1 {
            return raw;
        }

        let half = win / 2;
        let n = raw.len();
        let mut smoothed = Vec::with_capacity(n);

        for i in 0..n {
            let lo = if i >= half { i - half } else { 0 };
            let hi = if i + half < n { i + half } else { n - 1 };
            let count = (hi - lo + 1) as f64;
            let sum: f64 = raw[lo..=hi].iter().sum();
            smoothed.push(sum / count);
        }

        smoothed
    }

    // -- IF variance / jitter -------------------------------------------------

    /// Compute the variance (jitter) of the instantaneous frequency.
    ///
    /// Returns `(mean_hz, variance_hz2)`.  Uses the Kay estimator.
    pub fn if_jitter(&self, signal: &[(f64, f64)]) -> (f64, f64) {
        let freqs = self.kay_estimator(signal);
        if freqs.is_empty() {
            return (0.0, 0.0);
        }
        let n = freqs.len() as f64;
        let mean = freqs.iter().sum::<f64>() / n;
        let var = freqs.iter().map(|f| (f - mean).powi(2)).sum::<f64>() / n;
        (mean, var)
    }

    // -- Chirp-rate estimation ------------------------------------------------

    /// Estimate the frequency rate of change (chirp rate) in Hz/s.
    ///
    /// Performs a least-squares linear fit of IF vs. time.  Returns
    /// `(slope_hz_per_s, intercept_hz)`.
    pub fn chirp_rate(&self, signal: &[(f64, f64)]) -> (f64, f64) {
        let freqs = self.kay_estimator(signal);
        let n = freqs.len();
        if n < 2 {
            return (0.0, freqs.first().copied().unwrap_or(0.0));
        }

        let dt = 1.0 / self.sample_rate;
        // Least-squares: y = a*t + b
        let mut sum_t = 0.0;
        let mut sum_y = 0.0;
        let mut sum_tt = 0.0;
        let mut sum_ty = 0.0;
        for (i, &f) in freqs.iter().enumerate() {
            let t = i as f64 * dt;
            sum_t += t;
            sum_y += f;
            sum_tt += t * t;
            sum_ty += t * f;
        }
        let nf = n as f64;
        let denom = nf * sum_tt - sum_t * sum_t;
        if denom.abs() < 1e-30 {
            return (0.0, sum_y / nf);
        }
        let slope = (nf * sum_ty - sum_t * sum_y) / denom;
        let intercept = (sum_y - slope * sum_t) / nf;
        (slope, intercept)
    }

    // -- Zero-crossing frequency estimator ------------------------------------

    /// Estimate frequency from zero crossings of a real signal.
    ///
    /// Counts positive-going zero crossings and derives frequency. For a
    /// sinusoid at frequency *f*, there is one positive-going crossing per
    /// cycle, so `f = crossings / duration`.
    ///
    /// Returns the estimated frequency in Hz, or 0.0 if the signal is too
    /// short or has no crossings.
    pub fn zero_crossing_frequency(&self, real_signal: &[f64]) -> f64 {
        if real_signal.len() < 2 {
            return 0.0;
        }

        let mut crossings = 0usize;
        for i in 1..real_signal.len() {
            if real_signal[i - 1] < 0.0 && real_signal[i] >= 0.0 {
                crossings += 1;
            }
        }

        let duration = (real_signal.len() - 1) as f64 / self.sample_rate;
        if duration <= 0.0 {
            return 0.0;
        }
        crossings as f64 / duration
    }

    // -- Convenience: IF from real signal -------------------------------------

    /// Compute instantaneous frequency from a real-valued signal.
    ///
    /// Internally applies the Hilbert transform to create an analytic signal,
    /// then uses the Kay estimator.
    pub fn if_from_real(&self, real_signal: &[f64]) -> Vec<f64> {
        let analytic = Self::hilbert(real_signal);
        self.kay_estimator(&analytic)
    }
}

// --- Tests -------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const TWO_PI: f64 = 2.0 * PI;

    /// Helper: generate a complex tone at `freq` Hz for `n` samples.
    fn tone(freq: f64, sample_rate: f64, n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let phase = TWO_PI * freq * i as f64 / sample_rate;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Helper: generate a real sinusoid.
    fn real_tone(freq: f64, sample_rate: f64, n: usize) -> Vec<f64> {
        (0..n)
            .map(|i| {
                let phase = TWO_PI * freq * i as f64 / sample_rate;
                phase.cos()
            })
            .collect()
    }

    /// Helper: generate a linear chirp (complex).
    fn chirp_signal(f0: f64, f1: f64, sample_rate: f64, n: usize) -> Vec<(f64, f64)> {
        let duration = n as f64 / sample_rate;
        let rate = (f1 - f0) / duration;
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = TWO_PI * (f0 * t + 0.5 * rate * t * t);
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    // --- 1. Construction -----------------------------------------------------

    #[test]
    fn test_new_valid() {
        let e = InstantaneousFrequencyEstimator::new(48000.0);
        assert_eq!(e.sample_rate(), 48000.0);
    }

    #[test]
    #[should_panic]
    fn test_new_zero_rate_panics() {
        InstantaneousFrequencyEstimator::new(0.0);
    }

    #[test]
    #[should_panic]
    fn test_new_negative_rate_panics() {
        InstantaneousFrequencyEstimator::new(-100.0);
    }

    // --- 2. Phase differentiation --------------------------------------------

    #[test]
    fn test_phase_diff_constant_tone() {
        let fs = 8000.0;
        let freq = 200.0;
        let sig = tone(freq, fs, 128);
        let est = InstantaneousFrequencyEstimator::new(fs);
        let freqs = est.phase_diff(&sig);

        assert_eq!(freqs.len(), 128);
        // Skip first sample (boundary copy) and check interior.
        for &f in &freqs[1..127] {
            assert!(
                (f - freq).abs() < 0.5,
                "expected ~{freq}, got {f}"
            );
        }
    }

    #[test]
    fn test_phase_diff_empty() {
        let est = InstantaneousFrequencyEstimator::new(1000.0);
        assert!(est.phase_diff(&[]).is_empty());
    }

    #[test]
    fn test_phase_diff_single_sample() {
        let est = InstantaneousFrequencyEstimator::new(1000.0);
        let result = est.phase_diff(&[(1.0, 0.0)]);
        assert_eq!(result.len(), 1);
        assert_eq!(result[0], 0.0);
    }

    // --- 3. Kay estimator ----------------------------------------------------

    #[test]
    fn test_kay_constant_tone() {
        let fs = 10000.0;
        let freq = 350.0;
        let sig = tone(freq, fs, 256);
        let est = InstantaneousFrequencyEstimator::new(fs);
        let freqs = est.kay_estimator(&sig);

        assert_eq!(freqs.len(), 256);
        for &f in &freqs[1..] {
            assert!(
                (f - freq).abs() < 0.5,
                "expected ~{freq}, got {f}"
            );
        }
    }

    #[test]
    fn test_kay_negative_frequency() {
        let fs = 4000.0;
        let freq = -150.0;
        let sig = tone(freq, fs, 128);
        let est = InstantaneousFrequencyEstimator::new(fs);
        let freqs = est.kay_estimator(&sig);

        for &f in &freqs[1..] {
            assert!(
                (f - freq).abs() < 0.5,
                "expected ~{freq}, got {f}"
            );
        }
    }

    #[test]
    fn test_kay_empty() {
        let est = InstantaneousFrequencyEstimator::new(1000.0);
        assert!(est.kay_estimator(&[]).is_empty());
    }

    // --- 4. Phase unwrapping -------------------------------------------------

    #[test]
    fn test_unwrap_linear_phase() {
        // Phase that wraps around: 0, 0.8, 1.6, ... (mod 2pi wrap expected)
        let phases: Vec<f64> = (0..20).map(|i| (i as f64 * 0.8) % TWO_PI).collect();
        let unwrapped = InstantaneousFrequencyEstimator::unwrap_phase(&phases);

        // Successive differences should be close to 0.8
        for i in 1..unwrapped.len() {
            let d = unwrapped[i] - unwrapped[i - 1];
            assert!(
                (d - 0.8).abs() < 1e-10,
                "diff at {i}: expected ~0.8, got {d}"
            );
        }
    }

    #[test]
    fn test_unwrap_empty() {
        assert!(InstantaneousFrequencyEstimator::unwrap_phase(&[]).is_empty());
    }

    // --- 5. Hilbert transform ------------------------------------------------

    #[test]
    fn test_hilbert_real_part_preserved() {
        let fs = 1000.0;
        let sig = real_tone(50.0, fs, 64);
        let analytic = InstantaneousFrequencyEstimator::hilbert(&sig);

        assert_eq!(analytic.len(), sig.len());
        for (i, (&x, &(re, _))) in sig.iter().zip(analytic.iter()).enumerate() {
            assert!(
                (re - x).abs() < 1e-8,
                "real part mismatch at {i}: input {x}, got {re}"
            );
        }
    }

    #[test]
    fn test_hilbert_analytic_magnitude() {
        // For a single-frequency cosine the analytic signal should have
        // roughly constant magnitude (away from edges).
        let fs = 1000.0;
        let sig = real_tone(100.0, fs, 128);
        let analytic = InstantaneousFrequencyEstimator::hilbert(&sig);

        // Check middle 80% of the signal
        let lo = 13;
        let hi = 115;
        for i in lo..hi {
            let mag = (analytic[i].0.powi(2) + analytic[i].1.powi(2)).sqrt();
            assert!(
                (mag - 1.0).abs() < 0.05,
                "magnitude at {i}: expected ~1.0, got {mag}"
            );
        }
    }

    #[test]
    fn test_hilbert_empty() {
        assert!(InstantaneousFrequencyEstimator::hilbert(&[]).is_empty());
    }

    // --- 6. Smoothed IF ------------------------------------------------------

    #[test]
    fn test_smoothed_if_reduces_noise() {
        let fs = 8000.0;
        let freq = 500.0;
        let sig = tone(freq, fs, 256);
        let est = InstantaneousFrequencyEstimator::new(fs);

        let raw = est.kay_estimator(&sig);
        let smoothed = est.smoothed_if(&sig, 7);

        assert_eq!(smoothed.len(), raw.len());

        // Smoothed values should still be close to the true frequency
        for &f in &smoothed[3..253] {
            assert!(
                (f - freq).abs() < 1.0,
                "smoothed: expected ~{freq}, got {f}"
            );
        }
    }

    #[test]
    fn test_smoothed_if_window_1_equals_raw() {
        let fs = 4000.0;
        let sig = tone(100.0, fs, 64);
        let est = InstantaneousFrequencyEstimator::new(fs);

        let raw = est.kay_estimator(&sig);
        let sm = est.smoothed_if(&sig, 1);

        for (a, b) in raw.iter().zip(sm.iter()) {
            assert!((a - b).abs() < 1e-12);
        }
    }

    // --- 7. IF jitter --------------------------------------------------------

    #[test]
    fn test_jitter_pure_tone() {
        let fs = 10000.0;
        let freq = 250.0;
        let sig = tone(freq, fs, 512);
        let est = InstantaneousFrequencyEstimator::new(fs);

        let (mean, var) = est.if_jitter(&sig);
        assert!(
            (mean - freq).abs() < 1.0,
            "mean: expected ~{freq}, got {mean}"
        );
        // Pure tone -> near-zero variance
        assert!(var < 1.0, "variance too high: {var}");
    }

    // --- 8. Chirp rate estimation --------------------------------------------

    #[test]
    fn test_chirp_rate_linear_sweep() {
        let fs = 20000.0;
        let f0 = 100.0;
        let f1 = 2100.0;
        let n = 2048;
        let sig = chirp_signal(f0, f1, fs, n);
        let est = InstantaneousFrequencyEstimator::new(fs);

        let duration = n as f64 / fs;
        let expected_rate = (f1 - f0) / duration; // Hz/s

        let (slope, _intercept) = est.chirp_rate(&sig);
        let rel_err = (slope - expected_rate).abs() / expected_rate;
        assert!(
            rel_err < 0.05,
            "chirp rate: expected ~{expected_rate}, got {slope} (rel err {rel_err})"
        );
    }

    #[test]
    fn test_chirp_rate_constant_tone_is_zero() {
        let fs = 8000.0;
        let sig = tone(440.0, fs, 256);
        let est = InstantaneousFrequencyEstimator::new(fs);

        let (slope, _) = est.chirp_rate(&sig);
        assert!(
            slope.abs() < 100.0,
            "slope should be ~0 for constant tone, got {slope}"
        );
    }

    // --- 9. Zero-crossing frequency estimator --------------------------------

    #[test]
    fn test_zero_crossing_pure_tone() {
        let fs = 44100.0;
        let freq = 440.0;
        let duration_samples = (fs * 0.1) as usize; // 100 ms
        let sig = real_tone(freq, fs, duration_samples);
        let est = InstantaneousFrequencyEstimator::new(fs);

        let zc_freq = est.zero_crossing_frequency(&sig);
        assert!(
            (zc_freq - freq).abs() < 15.0,
            "zero-crossing freq: expected ~{freq}, got {zc_freq}"
        );
    }

    #[test]
    fn test_zero_crossing_empty() {
        let est = InstantaneousFrequencyEstimator::new(1000.0);
        assert_eq!(est.zero_crossing_frequency(&[]), 0.0);
    }

    // --- 10. IF from real signal ---------------------------------------------

    #[test]
    fn test_if_from_real_single_tone() {
        let fs = 4000.0;
        let freq = 200.0;
        let sig = real_tone(freq, fs, 128);
        let est = InstantaneousFrequencyEstimator::new(fs);

        let freqs = est.if_from_real(&sig);
        assert_eq!(freqs.len(), 128);

        // Middle region should be close to 200 Hz
        let mid = &freqs[16..112];
        let avg: f64 = mid.iter().sum::<f64>() / mid.len() as f64;
        assert!(
            (avg - freq).abs() < 5.0,
            "avg IF from real: expected ~{freq}, got {avg}"
        );
    }

    // --- 11. Consistency between estimators ----------------------------------

    #[test]
    fn test_phase_diff_and_kay_agree() {
        let fs = 8000.0;
        let freq = 600.0;
        let sig = tone(freq, fs, 128);
        let est = InstantaneousFrequencyEstimator::new(fs);

        let pd = est.phase_diff(&sig);
        let kay = est.kay_estimator(&sig);

        // Interior samples should agree to within 1 Hz
        for i in 1..127 {
            assert!(
                (pd[i] - kay[i]).abs() < 1.0,
                "disagreement at {i}: pd={}, kay={}",
                pd[i],
                kay[i]
            );
        }
    }
}
