//! Real-time formant frequency and bandwidth estimation using LPC analysis.
//!
//! This module implements Linear Predictive Coding (LPC) with polynomial root-finding
//! to extract formant frequencies and bandwidths from speech signals. It provides:
//!
//! - Autocorrelation-based LPC coefficient computation (Levinson-Durbin recursion)
//! - Polynomial root finding via Durand-Kerner iteration
//! - Formant extraction from LPC roots (frequency and bandwidth)
//! - Formant tracking across frames with continuity constraints
//! - Pre-emphasis filtering for speech preprocessing
//! - LPC spectral envelope computation
//! - Prediction error (residual) energy estimation
//!
//! # Example
//!
//! ```
//! use r4w_core::speech_formant_tracker::SpeechFormantTracker;
//!
//! // Create a tracker for 8 kHz audio, LPC order 10
//! let mut tracker = SpeechFormantTracker::new(10, 8000.0);
//!
//! // Generate a simple test signal (a windowed sinusoid at ~1000 Hz)
//! let n = 160; // 20 ms frame at 8 kHz
//! let signal: Vec<f64> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / 8000.0;
//!         (2.0 * std::f64::consts::PI * 1000.0 * t).sin()
//!             * (0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / n as f64).cos())
//!     })
//!     .collect();
//!
//! // Analyze the frame
//! let result = tracker.analyze_frame(&signal);
//! assert!(!result.formants.is_empty(), "should detect at least one formant");
//!
//! // Each formant has a frequency (Hz) and bandwidth (Hz)
//! for f in &result.formants {
//!     assert!(f.frequency > 0.0 && f.frequency < 4000.0);
//!     assert!(f.bandwidth > 0.0);
//! }
//! ```

use std::f64::consts::PI;

// ── Complex arithmetic helpers (using (f64, f64) tuples) ─────────────

/// Complex addition.
#[inline]
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex subtraction.
#[inline]
fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Complex multiplication.
#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex division.
#[inline]
fn c_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = b.0 * b.0 + b.1 * b.1;
    if denom < 1e-300 {
        return (0.0, 0.0);
    }
    ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
}

/// Complex magnitude.
#[inline]
fn c_abs(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

/// Complex magnitude squared.
#[inline]
fn c_abs2(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Complex negation.
#[inline]
fn c_neg(a: (f64, f64)) -> (f64, f64) {
    (-a.0, -a.1)
}

// ── Public types ─────────────────────────────────────────────────────

/// A single formant with frequency and bandwidth in Hz.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Formant {
    /// Formant center frequency in Hz.
    pub frequency: f64,
    /// Formant 3-dB bandwidth in Hz.
    pub bandwidth: f64,
}

/// Result of analyzing a single speech frame.
#[derive(Debug, Clone)]
pub struct FrameAnalysis {
    /// Detected formants sorted by frequency.
    pub formants: Vec<Formant>,
    /// LPC coefficients (length = lpc_order + 1, first element is 1.0).
    pub lpc_coefficients: Vec<f64>,
    /// Prediction error energy (residual power).
    pub prediction_error_energy: f64,
    /// Total frame energy.
    pub frame_energy: f64,
}

/// Configuration for formant continuity tracking across frames.
#[derive(Debug, Clone)]
pub struct TrackingConfig {
    /// Maximum frequency jump (Hz) allowed between consecutive frames.
    pub max_frequency_jump: f64,
    /// Number of formants to track (typically 3-5 for speech).
    pub num_formants_to_track: usize,
}

impl Default for TrackingConfig {
    fn default() -> Self {
        Self {
            max_frequency_jump: 500.0,
            num_formants_to_track: 4,
        }
    }
}

/// Real-time formant frequency and bandwidth estimator.
///
/// Uses LPC analysis with polynomial root-finding to extract formants from
/// speech frames and optionally tracks them across frames for continuity.
pub struct SpeechFormantTracker {
    /// LPC analysis order (typically 8-16 for speech).
    lpc_order: usize,
    /// Sample rate in Hz.
    sample_rate: f64,
    /// Pre-emphasis coefficient (0.0 to disable).
    pre_emphasis_coeff: f64,
    /// Tracking configuration.
    tracking_config: TrackingConfig,
    /// Previously tracked formants (for continuity).
    prev_formants: Vec<Formant>,
    /// Minimum formant frequency (Hz) to accept.
    min_formant_freq: f64,
    /// Maximum formant frequency (Hz) to accept.
    max_formant_freq: f64,
}

impl SpeechFormantTracker {
    /// Create a new tracker with the given LPC order and sample rate.
    ///
    /// # Arguments
    /// * `lpc_order` - Number of LPC coefficients (typically 8-16).
    /// * `sample_rate` - Audio sample rate in Hz.
    pub fn new(lpc_order: usize, sample_rate: f64) -> Self {
        assert!(lpc_order >= 2, "LPC order must be at least 2");
        assert!(sample_rate > 0.0, "sample rate must be positive");
        Self {
            lpc_order,
            sample_rate,
            pre_emphasis_coeff: 0.97,
            tracking_config: TrackingConfig::default(),
            prev_formants: Vec::new(),
            min_formant_freq: 90.0,
            max_formant_freq: sample_rate / 2.0,
        }
    }

    /// Set the pre-emphasis filter coefficient (0.0 disables pre-emphasis).
    pub fn set_pre_emphasis(&mut self, coeff: f64) {
        self.pre_emphasis_coeff = coeff;
    }

    /// Set the tracking configuration for frame-to-frame continuity.
    pub fn set_tracking_config(&mut self, config: TrackingConfig) {
        self.tracking_config = config;
    }

    /// Set the minimum acceptable formant frequency (Hz).
    pub fn set_min_formant_freq(&mut self, freq: f64) {
        self.min_formant_freq = freq;
    }

    /// Set the maximum acceptable formant frequency (Hz).
    pub fn set_max_formant_freq(&mut self, freq: f64) {
        self.max_formant_freq = freq;
    }

    /// Return the current LPC order.
    pub fn lpc_order(&self) -> usize {
        self.lpc_order
    }

    /// Return the sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Apply pre-emphasis filter: y[n] = x[n] - coeff * x[n-1].
    pub fn pre_emphasis(&self, signal: &[f64]) -> Vec<f64> {
        if signal.is_empty() || self.pre_emphasis_coeff == 0.0 {
            return signal.to_vec();
        }
        let mut out = Vec::with_capacity(signal.len());
        out.push(signal[0]);
        for i in 1..signal.len() {
            out.push(signal[i] - self.pre_emphasis_coeff * signal[i - 1]);
        }
        out
    }

    /// Compute autocorrelation of `signal` for lags 0..=`order`.
    pub fn autocorrelation(signal: &[f64], order: usize) -> Vec<f64> {
        let n = signal.len();
        let mut r = vec![0.0; order + 1];
        for lag in 0..=order {
            let mut sum = 0.0;
            for i in lag..n {
                sum += signal[i] * signal[i - lag];
            }
            r[lag] = sum;
        }
        r
    }

    /// Levinson-Durbin recursion to compute LPC coefficients from autocorrelation.
    ///
    /// Returns `(coeffs, prediction_error)` where `coeffs[0] == 1.0` and
    /// `coeffs[1..=order]` are the negated LP coefficients for the polynomial
    /// A(z) = 1 + coeffs[1]*z^-1 + ... + coeffs[p]*z^-p.
    pub fn levinson_durbin(r: &[f64], order: usize) -> (Vec<f64>, f64) {
        assert!(r.len() > order, "autocorrelation vector too short");
        if r[0] <= 0.0 {
            // Silent frame
            let mut coeffs = vec![0.0; order + 1];
            coeffs[0] = 1.0;
            return (coeffs, 0.0);
        }

        let mut a = vec![0.0; order + 1];
        a[0] = 1.0;
        let mut e = r[0];

        for i in 1..=order {
            // Compute reflection coefficient
            let mut lambda = 0.0;
            for j in 1..i {
                lambda += a[j] * r[i - j];
            }
            lambda = (r[i] - lambda) / e;

            // Update coefficients
            let mut a_new = a.clone();
            a_new[i] = lambda;
            for j in 1..i {
                a_new[j] = a[j] - lambda * a[i - j];
            }
            a = a_new;

            e *= 1.0 - lambda * lambda;
            if e <= 0.0 {
                break;
            }
        }

        // Build the LPC polynomial: A(z) = 1 - a1*z^-1 - a2*z^-2 - ...
        // Store as coeffs[0]=1, coeffs[k]=-a[k] for k=1..order
        let mut coeffs = vec![0.0; order + 1];
        coeffs[0] = 1.0;
        for k in 1..=order {
            coeffs[k] = -a[k];
        }

        (coeffs, e)
    }

    /// Find roots of a polynomial using the Durand-Kerner method.
    ///
    /// `poly` is ordered as `[a_0, a_1, ..., a_n]` representing
    /// `a_0 + a_1*z + a_2*z^2 + ... + a_n*z^n`.
    ///
    /// Returns up to `n` roots as complex `(f64, f64)` tuples.
    pub fn find_roots(poly: &[f64]) -> Vec<(f64, f64)> {
        let n = poly.len();
        if n <= 1 {
            return vec![];
        }
        let degree = n - 1;
        if degree == 0 {
            return vec![];
        }

        // Normalize by leading coefficient
        let lead = poly[degree];
        if lead.abs() < 1e-300 {
            return vec![];
        }
        let p: Vec<f64> = poly.iter().map(|c| c / lead).collect();

        // Initialize roots on a circle with slightly offset angles to break symmetry
        let mut roots: Vec<(f64, f64)> = Vec::with_capacity(degree);
        let radius = 0.9;
        for k in 0..degree {
            let angle = 2.0 * PI * (k as f64 + 0.3) / degree as f64;
            roots.push((radius * angle.cos(), radius * angle.sin()));
        }

        let max_iter = 1000;
        let tol = 1e-12;

        for _iter in 0..max_iter {
            let mut max_correction = 0.0;

            for i in 0..degree {
                // Evaluate polynomial at roots[i]
                let mut val = (p[degree], 0.0);
                for j in (0..degree).rev() {
                    val = c_add(c_mul(val, roots[i]), (p[j], 0.0));
                }

                // Compute product of (roots[i] - roots[j]) for j != i
                let mut denom = (1.0, 0.0);
                for j in 0..degree {
                    if j != i {
                        denom = c_mul(denom, c_sub(roots[i], roots[j]));
                    }
                }

                let correction = c_div(val, denom);
                roots[i] = c_sub(roots[i], correction);

                let corr_mag = c_abs(correction);
                if corr_mag > max_correction {
                    max_correction = corr_mag;
                }
            }

            if max_correction < tol {
                break;
            }
        }

        roots
    }

    /// Extract formants from LPC polynomial roots.
    ///
    /// A root at angle theta with magnitude r gives:
    /// - frequency = theta * sample_rate / (2*pi)
    /// - bandwidth = -ln(r) * sample_rate / pi
    fn extract_formants_from_roots(
        &self,
        roots: &[(f64, f64)],
    ) -> Vec<Formant> {
        let mut formants = Vec::new();
        let nyquist = self.sample_rate / 2.0;

        for &root in roots {
            let (_re, im) = root;
            // Only consider roots with positive imaginary part (upper half plane)
            if im <= 0.0 {
                continue;
            }

            let r = c_abs(root);
            // Skip roots too far from unit circle
            if r < 0.5 || r > 1.0 {
                continue;
            }

            let angle = im.atan2(root.0);
            let freq = angle * self.sample_rate / (2.0 * PI);
            let bw = -(r.ln()) * self.sample_rate / PI;

            if freq < self.min_formant_freq || freq > nyquist {
                continue;
            }

            // Reject unreasonably wide bandwidths
            if bw > 1000.0 || bw < 1.0 {
                continue;
            }

            formants.push(Formant {
                frequency: freq,
                bandwidth: bw,
            });
        }

        formants.sort_by(|a, b| a.frequency.partial_cmp(&b.frequency).unwrap());
        formants
    }

    /// Analyze a single frame of speech samples.
    ///
    /// Returns formant frequencies/bandwidths, LPC coefficients, and energy.
    pub fn analyze_frame(&mut self, frame: &[f64]) -> FrameAnalysis {
        let frame_energy: f64 = frame.iter().map(|&x| x * x).sum();

        let emphasized = self.pre_emphasis(frame);

        let r = Self::autocorrelation(&emphasized, self.lpc_order);

        let (coeffs, pred_err) = Self::levinson_durbin(&r, self.lpc_order);

        // Build polynomial in z for root finding.
        // A(z) = coeffs[0]*z^p + coeffs[1]*z^(p-1) + ... + coeffs[p]
        // In ascending-power form: poly[k] = coeffs[p-k]
        let poly: Vec<f64> = coeffs.iter().rev().cloned().collect();

        let roots = Self::find_roots(&poly);

        let mut formants = self.extract_formants_from_roots(&roots);

        if !self.prev_formants.is_empty() {
            formants = self.track_formants(formants);
        }

        formants.truncate(self.tracking_config.num_formants_to_track);

        self.prev_formants = formants.clone();

        FrameAnalysis {
            formants,
            lpc_coefficients: coeffs,
            prediction_error_energy: pred_err,
            frame_energy,
        }
    }

    /// Track formants across frames using continuity constraints.
    fn track_formants(&self, candidates: Vec<Formant>) -> Vec<Formant> {
        let max_jump = self.tracking_config.max_frequency_jump;
        let mut used = vec![false; candidates.len()];
        let mut tracked = Vec::new();

        for prev in &self.prev_formants {
            let mut best_idx = None;
            let mut best_dist = f64::MAX;

            for (i, cand) in candidates.iter().enumerate() {
                if used[i] {
                    continue;
                }
                let dist = (cand.frequency - prev.frequency).abs();
                if dist < best_dist && dist <= max_jump {
                    best_dist = dist;
                    best_idx = Some(i);
                }
            }

            if let Some(idx) = best_idx {
                tracked.push(candidates[idx]);
                used[idx] = true;
            }
        }

        for (i, cand) in candidates.iter().enumerate() {
            if !used[i] {
                tracked.push(*cand);
            }
        }

        tracked.sort_by(|a, b| a.frequency.partial_cmp(&b.frequency).unwrap());
        tracked
    }

    /// Compute the LPC spectral envelope at `num_points` frequencies from 0 to Nyquist.
    ///
    /// Returns `(frequencies, magnitudes_db)`.
    pub fn lpc_spectrum(&self, coeffs: &[f64], num_points: usize) -> (Vec<f64>, Vec<f64>) {
        let nyquist = self.sample_rate / 2.0;
        let mut freqs = Vec::with_capacity(num_points);
        let mut mags_db = Vec::with_capacity(num_points);

        for k in 0..num_points {
            let f = nyquist * k as f64 / num_points as f64;
            let omega = 2.0 * PI * f / self.sample_rate;

            // Evaluate A(e^{j*omega}) = sum_{k=0}^{p} coeffs[k] * e^{-j*omega*k}
            let mut re_sum = 0.0;
            let mut im_sum = 0.0;
            for (i, &c) in coeffs.iter().enumerate() {
                let angle = -(omega * i as f64);
                re_sum += c * angle.cos();
                im_sum += c * angle.sin();
            }

            let mag_sq = re_sum * re_sum + im_sum * im_sum;
            let mag_db = if mag_sq > 1e-30 {
                -10.0 * mag_sq.log10()
            } else {
                150.0
            };

            freqs.push(f);
            mags_db.push(mag_db);
        }

        (freqs, mags_db)
    }

    /// Compute the LPC prediction residual (error signal).
    ///
    /// `coeffs` should be `[1, c1, c2, ...]` from Levinson-Durbin.
    pub fn prediction_residual(coeffs: &[f64], signal: &[f64]) -> Vec<f64> {
        let p = coeffs.len() - 1;
        let n = signal.len();
        let mut residual = Vec::with_capacity(n);

        for i in 0..n {
            let mut val = coeffs[0] * signal[i];
            for k in 1..=p {
                if i >= k {
                    val += coeffs[k] * signal[i - k];
                }
            }
            residual.push(val);
        }

        residual
    }

    /// Compute the residual energy (sum of squared residual).
    pub fn residual_energy(coeffs: &[f64], signal: &[f64]) -> f64 {
        let residual = Self::prediction_residual(coeffs, signal);
        residual.iter().map(|&x| x * x).sum()
    }

    /// Reset the tracker state (clear previous formants).
    pub fn reset(&mut self) {
        self.prev_formants.clear();
    }
}

// ── Tests ────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a windowed sinusoid.
    fn windowed_sine(freq: f64, sample_rate: f64, num_samples: usize) -> Vec<f64> {
        (0..num_samples)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let window =
                    0.5 - 0.5 * (2.0 * PI * i as f64 / num_samples as f64).cos();
                (2.0 * PI * freq * t).sin() * window
            })
            .collect()
    }

    /// Helper: generate a sum of windowed sinusoids.
    fn multi_sine(freqs: &[f64], sample_rate: f64, num_samples: usize) -> Vec<f64> {
        (0..num_samples)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let window =
                    0.5 - 0.5 * (2.0 * PI * i as f64 / num_samples as f64).cos();
                let sum: f64 = freqs.iter().map(|&f| (2.0 * PI * f * t).sin()).sum();
                sum * window / freqs.len() as f64
            })
            .collect()
    }

    // ── Test 1: Construction ──

    #[test]
    fn test_new_tracker() {
        let t = SpeechFormantTracker::new(10, 8000.0);
        assert_eq!(t.lpc_order(), 10);
        assert_eq!(t.sample_rate(), 8000.0);
    }

    // ── Test 2: Pre-emphasis disabled ──

    #[test]
    fn test_pre_emphasis_identity_when_disabled() {
        let mut t = SpeechFormantTracker::new(10, 8000.0);
        t.set_pre_emphasis(0.0);
        let signal = vec![1.0, 2.0, 3.0, 4.0];
        let out = t.pre_emphasis(&signal);
        assert_eq!(out, signal);
    }

    // ── Test 3: Pre-emphasis filter ──

    #[test]
    fn test_pre_emphasis_filter() {
        let t = SpeechFormantTracker::new(10, 8000.0);
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let out = t.pre_emphasis(&signal);
        assert_eq!(out.len(), 5);
        assert!((out[0] - 1.0).abs() < 1e-10);
        assert!((out[1] - 1.03).abs() < 1e-10);
        assert!((out[2] - 1.06).abs() < 1e-10);
    }

    // ── Test 4: Autocorrelation of constant signal ──

    #[test]
    fn test_autocorrelation_constant() {
        let signal = vec![1.0; 10];
        let r = SpeechFormantTracker::autocorrelation(&signal, 3);
        assert_eq!(r.len(), 4);
        assert!((r[0] - 10.0).abs() < 1e-10);
        assert!((r[1] - 9.0).abs() < 1e-10);
        assert!((r[2] - 8.0).abs() < 1e-10);
        assert!((r[3] - 7.0).abs() < 1e-10);
    }

    // ── Test 5: Autocorrelation lag-0 equals energy ──

    #[test]
    fn test_autocorrelation_lag0_is_energy() {
        let signal = vec![1.0, -1.0, 1.0, -1.0];
        let r = SpeechFormantTracker::autocorrelation(&signal, 2);
        let energy: f64 = signal.iter().map(|&x| x * x).sum();
        assert!((r[0] - energy).abs() < 1e-10);
    }

    // ── Test 6: Levinson-Durbin on white noise ──

    #[test]
    fn test_levinson_durbin_white_noise() {
        let r = vec![1.0, 0.0, 0.0, 0.0, 0.0];
        let (coeffs, e) = SpeechFormantTracker::levinson_durbin(&r, 4);
        assert!((coeffs[0] - 1.0).abs() < 1e-10);
        for k in 1..=4 {
            assert!(coeffs[k].abs() < 1e-10, "coeffs[{}] = {}", k, coeffs[k]);
        }
        assert!((e - 1.0).abs() < 1e-10);
    }

    // ── Test 7: Levinson-Durbin on AR(1) ──

    #[test]
    fn test_levinson_durbin_ar1() {
        let order = 4;
        let a = 0.9_f64;
        let r: Vec<f64> = (0..=order).map(|k| a.powi(k as i32)).collect();
        let (coeffs, _e) = SpeechFormantTracker::levinson_durbin(&r, order);
        // coeffs[1] should be close to -0.9
        assert!(
            (coeffs[1] - (-0.9)).abs() < 0.05,
            "coeffs[1] = {}, expected ~-0.9",
            coeffs[1]
        );
    }

    // ── Test 8: Root finding - quadratic ──

    #[test]
    fn test_find_roots_quadratic() {
        // z^2 - 3z + 2 = (z-1)(z-2), roots at 1 and 2
        let poly = vec![2.0, -3.0, 1.0];
        let roots = SpeechFormantTracker::find_roots(&poly);
        assert_eq!(roots.len(), 2);

        let mut real_parts: Vec<f64> = roots.iter().map(|r| r.0).collect();
        real_parts.sort_by(|a, b| a.partial_cmp(b).unwrap());

        assert!((real_parts[0] - 1.0).abs() < 1e-6, "root 0 = {}", real_parts[0]);
        assert!((real_parts[1] - 2.0).abs() < 1e-6, "root 1 = {}", real_parts[1]);

        for r in &roots {
            assert!(r.1.abs() < 1e-6, "imaginary part = {}", r.1);
        }
    }

    // ── Test 9: Root finding - complex conjugate pair ──

    #[test]
    fn test_find_roots_complex_conjugate() {
        // z^2 + 1 = 0, roots at +i, -i
        let poly = vec![1.0, 0.0, 1.0];
        let roots = SpeechFormantTracker::find_roots(&poly);
        assert_eq!(roots.len(), 2);

        let mut im_parts: Vec<f64> = roots.iter().map(|r| r.1).collect();
        im_parts.sort_by(|a, b| a.partial_cmp(b).unwrap());

        assert!((im_parts[0] - (-1.0)).abs() < 1e-6);
        assert!((im_parts[1] - 1.0).abs() < 1e-6);
    }

    // ── Test 10: Root finding - linear ──

    #[test]
    fn test_find_roots_linear() {
        let poly = vec![-6.0, 2.0];
        let roots = SpeechFormantTracker::find_roots(&poly);
        assert_eq!(roots.len(), 1);
        assert!((roots[0].0 - 3.0).abs() < 1e-6);
        assert!(roots[0].1.abs() < 1e-6);
    }

    // ── Test 11: Analyze frame detects formants ──

    #[test]
    fn test_analyze_frame_single_tone() {
        let mut tracker = SpeechFormantTracker::new(10, 8000.0);
        tracker.set_min_formant_freq(50.0);
        let signal = windowed_sine(1000.0, 8000.0, 256);
        let result = tracker.analyze_frame(&signal);

        assert!(!result.formants.is_empty(), "should detect formants");
        assert!(result.frame_energy > 0.0);
        assert!(result.prediction_error_energy >= 0.0);
        assert_eq!(result.lpc_coefficients.len(), 11);
        assert!((result.lpc_coefficients[0] - 1.0).abs() < 1e-10);
    }

    // ── Test 12: LPC spectrum length ──

    #[test]
    fn test_lpc_spectrum_length() {
        let tracker = SpeechFormantTracker::new(10, 8000.0);
        let coeffs = vec![1.0, -0.5, 0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let (freqs, mags) = tracker.lpc_spectrum(&coeffs, 128);
        assert_eq!(freqs.len(), 128);
        assert_eq!(mags.len(), 128);
        assert!((freqs[0] - 0.0).abs() < 1e-10);
    }

    // ── Test 13: Prediction residual identity ──

    #[test]
    fn test_prediction_residual_white() {
        let coeffs = vec![1.0, 0.0, 0.0];
        let signal = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let residual = SpeechFormantTracker::prediction_residual(&coeffs, &signal);
        assert_eq!(residual.len(), 5);
        for (r, s) in residual.iter().zip(signal.iter()) {
            assert!((r - s).abs() < 1e-10);
        }
    }

    // ── Test 14: Residual energy ──

    #[test]
    fn test_residual_energy() {
        let coeffs = vec![1.0, 0.0];
        let signal = vec![3.0, 4.0];
        let e = SpeechFormantTracker::residual_energy(&coeffs, &signal);
        assert!((e - 25.0).abs() < 1e-10);
    }

    // ── Test 15: Formant tracking continuity ──

    #[test]
    fn test_formant_tracking_continuity() {
        let mut tracker = SpeechFormantTracker::new(12, 16000.0);
        tracker.set_tracking_config(TrackingConfig {
            max_frequency_jump: 300.0,
            num_formants_to_track: 4,
        });

        let frame1 = windowed_sine(1000.0, 16000.0, 320);
        let frame2 = windowed_sine(1050.0, 16000.0, 320);

        let r1 = tracker.analyze_frame(&frame1);
        let r2 = tracker.analyze_frame(&frame2);

        if !r1.formants.is_empty() && !r2.formants.is_empty() {
            let min_dist: f64 = r2
                .formants
                .iter()
                .flat_map(|f2| r1.formants.iter().map(move |f1| (f1.frequency - f2.frequency).abs()))
                .fold(f64::MAX, f64::min);
            assert!(
                min_dist < 500.0,
                "formants should be reasonably close between similar frames"
            );
        }
    }

    // ── Test 16: Empty / silent frame ──

    #[test]
    fn test_analyze_empty_frame() {
        let mut tracker = SpeechFormantTracker::new(4, 8000.0);
        let signal = vec![0.0; 100];
        let result = tracker.analyze_frame(&signal);
        assert_eq!(result.frame_energy, 0.0);
        assert!(result.formants.is_empty());
    }

    // ── Test 17: Reset clears state ──

    #[test]
    fn test_reset_clears_state() {
        let mut tracker = SpeechFormantTracker::new(10, 8000.0);
        let signal = windowed_sine(500.0, 8000.0, 160);
        tracker.analyze_frame(&signal);
        tracker.reset();
        assert!(tracker.prev_formants.is_empty());
    }

    // ── Test 18: Complex arithmetic helpers ──

    #[test]
    fn test_complex_arithmetic() {
        let a = (3.0, 4.0);
        let b = (1.0, -2.0);

        let sum = c_add(a, b);
        assert!((sum.0 - 4.0).abs() < 1e-10);
        assert!((sum.1 - 2.0).abs() < 1e-10);

        let diff = c_sub(a, b);
        assert!((diff.0 - 2.0).abs() < 1e-10);
        assert!((diff.1 - 6.0).abs() < 1e-10);

        let prod = c_mul(a, b);
        assert!((prod.0 - 11.0).abs() < 1e-10);
        assert!((prod.1 - (-2.0)).abs() < 1e-10);

        let quot = c_div(a, b);
        assert!((quot.0 - (-1.0)).abs() < 1e-10);
        assert!((quot.1 - 2.0).abs() < 1e-10);

        assert!((c_abs((3.0, 4.0)) - 5.0).abs() < 1e-10);
        assert!((c_abs2((3.0, 4.0)) - 25.0).abs() < 1e-10);

        let neg = c_neg(a);
        assert!((neg.0 - (-3.0)).abs() < 1e-10);
        assert!((neg.1 - (-4.0)).abs() < 1e-10);
    }

    // ── Test 19: Pre-emphasis on empty signal ──

    #[test]
    fn test_pre_emphasis_empty() {
        let t = SpeechFormantTracker::new(10, 8000.0);
        let out = t.pre_emphasis(&[]);
        assert!(out.is_empty());
    }

    // ── Test 20: LPC spectrum peak near resonance ──

    #[test]
    fn test_lpc_spectrum_peak_near_resonance() {
        let mut tracker = SpeechFormantTracker::new(12, 16000.0);
        let signal = windowed_sine(2000.0, 16000.0, 512);
        let result = tracker.analyze_frame(&signal);

        let (freqs, mags) = tracker.lpc_spectrum(&result.lpc_coefficients, 512);

        let mut max_mag = f64::MIN;
        let mut peak_freq = 0.0;
        for (&f, &m) in freqs.iter().zip(mags.iter()) {
            if f > 100.0 && m > max_mag {
                max_mag = m;
                peak_freq = f;
            }
        }

        assert!(
            (peak_freq - 2000.0).abs() < 1500.0,
            "peak at {} Hz, expected near 2000 Hz",
            peak_freq
        );
    }

    // ── Test 21: Multi-tone detection ──

    #[test]
    fn test_multi_tone_detection() {
        let mut tracker = SpeechFormantTracker::new(14, 16000.0);
        tracker.set_min_formant_freq(50.0);
        let signal = multi_sine(&[500.0, 1500.0, 2500.0], 16000.0, 512);
        let result = tracker.analyze_frame(&signal);

        assert!(
            result.formants.len() >= 1,
            "expected at least one formant, got {}",
            result.formants.len()
        );
    }

    // ── Test 22: Formant struct Copy/Clone ──

    #[test]
    fn test_formant_struct() {
        let f = Formant {
            frequency: 500.0,
            bandwidth: 100.0,
        };
        let g = f;
        assert_eq!(f.frequency, g.frequency);
        assert_eq!(f.bandwidth, g.bandwidth);
    }

    // ── Test 23: Prediction residual for AR(1) signal ──

    #[test]
    fn test_prediction_residual_ar1() {
        let n = 200;
        let a = 0.8;
        let mut signal = vec![0.0; n];
        let mut seed = 42u64;
        for i in 0..n {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let noise = ((seed >> 33) as f64 / (1u64 << 31) as f64) - 0.5;
            signal[i] = if i > 0 { a * signal[i - 1] } else { 0.0 } + noise * 0.1;
        }

        let r = SpeechFormantTracker::autocorrelation(&signal, 2);
        let (coeffs, _e) = SpeechFormantTracker::levinson_durbin(&r, 2);

        let sig_energy: f64 = signal.iter().map(|&x| x * x).sum();
        let res_energy = SpeechFormantTracker::residual_energy(&coeffs, &signal);
        assert!(
            res_energy < sig_energy,
            "residual energy {} should be less than signal energy {}",
            res_energy,
            sig_energy
        );
    }
}
