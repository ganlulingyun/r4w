//! Multi-lag autocorrelation for detecting signal periodicity and estimating
//! fundamental frequency.
//!
//! This module provides [`PeriodicAutocorrelator`], which computes the
//! autocorrelation function (ACF) of real or complex signals over a
//! configurable range of lags. It can detect dominant periodicities, estimate
//! periodicity strength, find harmonics, and operate in a sliding-window mode
//! for time-varying analysis.
//!
//! Only the Rust standard library is used — no external crates.
//!
//! # Example
//!
//! ```
//! use r4w_core::periodic_autocorrelator::PeriodicAutocorrelator;
//!
//! // Build a pure sinusoid with period = 20 samples
//! let n = 200;
//! let period = 20.0_f64;
//! let signal: Vec<f64> = (0..n)
//!     .map(|i| (2.0 * std::f64::consts::PI * i as f64 / period).sin())
//!     .collect();
//!
//! let pa = PeriodicAutocorrelator::new(100, n);
//! let result = pa.analyze_real(&signal);
//!
//! // The detected fundamental period should be close to 20
//! let detected = result.fundamental_period().unwrap();
//! assert!((detected as f64 - period).abs() < 2.0);
//! ```

use std::f64::consts::PI;

// ─── complex helpers (re, im) tuples ────────────────────────────────────────

type C64 = (f64, f64);

#[inline]
fn c_mul(a: C64, b: C64) -> C64 {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: C64) -> C64 {
    (a.0, -a.1)
}

#[inline]
fn c_add(a: C64, b: C64) -> C64 {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_mag_sq(a: C64) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn c_mag(a: C64) -> f64 {
    c_mag_sq(a).sqrt()
}

// ─── public types ───────────────────────────────────────────────────────────

/// Selects biased or unbiased normalisation of the autocorrelation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NormMode {
    /// Divide every lag by N (biased — always positive-semi-definite).
    Biased,
    /// Divide lag *k* by N − |k| (unbiased — larger variance at high lags).
    Unbiased,
}

/// One entry in the autocorrelation function.
#[derive(Debug, Clone, Copy)]
pub struct AcfEntry {
    /// Lag index (0 .. max_lag).
    pub lag: usize,
    /// Complex autocorrelation value at this lag.
    pub value: C64,
}

/// Result of a full autocorrelation analysis.
#[derive(Debug, Clone)]
pub struct AcfResult {
    /// The computed ACF entries (lag 0 .. max_lag inclusive).
    pub entries: Vec<AcfEntry>,
    /// Detected peaks (lag indices), sorted by descending real-part value.
    pub peaks: Vec<usize>,
}

impl AcfResult {
    /// The fundamental period in samples (lag of the strongest peak
    /// **excluding** lag 0).
    pub fn fundamental_period(&self) -> Option<usize> {
        self.peaks.iter().copied().find(|&l| l > 0)
    }

    /// Periodicity strength: ratio of the strongest non-zero-lag peak
    /// real value to the zero-lag real value.  Returns a value in \[0, 1\]
    /// for normalised ACF of real signals.
    pub fn periodicity_strength(&self) -> f64 {
        let r0 = if self.entries.is_empty() {
            return 0.0;
        } else {
            self.entries[0].value.0
        };
        if r0 == 0.0 {
            return 0.0;
        }
        self.fundamental_period()
            .map(|lag| self.entries[lag].value.0 / r0)
            .unwrap_or(0.0)
            .max(0.0)
    }

    /// Return up to `n` harmonic candidates — lags whose real value is a
    /// local peak, sorted by lag.
    pub fn harmonics(&self, n: usize) -> Vec<usize> {
        let mut h: Vec<usize> = self.peaks.iter().copied().filter(|&l| l > 0).collect();
        // Sort by lag (ascending) for harmonic interpretation.
        h.sort();
        h.truncate(n);
        h
    }

    /// Return the normalised (correlation-coefficient) ACF.  Each value is
    /// divided by the zero-lag real value so that entry\[0\] == 1.0.
    pub fn normalized(&self) -> Vec<(usize, f64)> {
        let r0 = self.entries.first().map(|e| e.value.0).unwrap_or(0.0);
        if r0 == 0.0 {
            return self.entries.iter().map(|e| (e.lag, 0.0)).collect();
        }
        self.entries
            .iter()
            .map(|e| (e.lag, e.value.0 / r0))
            .collect()
    }
}

/// Multi-lag autocorrelator with configurable maximum lag and window size.
#[derive(Debug, Clone)]
pub struct PeriodicAutocorrelator {
    /// Maximum lag to compute (inclusive).
    max_lag: usize,
    /// Number of samples used in each computation window.
    window_size: usize,
    /// Normalisation mode.
    norm: NormMode,
    /// Minimum relative height for a peak to be reported (0..1).
    peak_threshold: f64,
}

impl PeriodicAutocorrelator {
    /// Create a new autocorrelator.
    ///
    /// * `max_lag`     – maximum lag index (inclusive).
    /// * `window_size` – number of samples in the analysis window.
    ///
    /// Defaults to biased normalisation and a 0.1 peak threshold.
    pub fn new(max_lag: usize, window_size: usize) -> Self {
        assert!(max_lag > 0, "max_lag must be > 0");
        assert!(window_size > max_lag, "window_size must be > max_lag");
        Self {
            max_lag,
            window_size,
            norm: NormMode::Biased,
            peak_threshold: 0.1,
        }
    }

    /// Set the normalisation mode (biased / unbiased).
    pub fn with_norm(mut self, norm: NormMode) -> Self {
        self.norm = norm;
        self
    }

    /// Set the minimum relative height for peak detection (0..1).
    pub fn with_peak_threshold(mut self, t: f64) -> Self {
        self.peak_threshold = t.clamp(0.0, 1.0);
        self
    }

    // ── core computation ────────────────────────────────────────────────

    /// Compute the ACF of a **complex** signal slice.
    ///
    /// The slice length must be >= `window_size`; only the first
    /// `window_size` samples are used.
    pub fn compute_complex(&self, signal: &[C64]) -> Vec<AcfEntry> {
        assert!(
            signal.len() >= self.window_size,
            "signal length ({}) < window_size ({})",
            signal.len(),
            self.window_size
        );
        let n = self.window_size;
        let mut out = Vec::with_capacity(self.max_lag + 1);
        for lag in 0..=self.max_lag {
            let mut sum: C64 = (0.0, 0.0);
            for i in 0..(n - lag) {
                sum = c_add(sum, c_mul(signal[i + lag], c_conj(signal[i])));
            }
            let denom = match self.norm {
                NormMode::Biased => n as f64,
                NormMode::Unbiased => (n - lag) as f64,
            };
            let value = (sum.0 / denom, sum.1 / denom);
            out.push(AcfEntry { lag, value });
        }
        out
    }

    /// Compute the ACF of a **real** signal slice.
    pub fn compute_real(&self, signal: &[f64]) -> Vec<AcfEntry> {
        assert!(
            signal.len() >= self.window_size,
            "signal length ({}) < window_size ({})",
            signal.len(),
            self.window_size
        );
        let n = self.window_size;
        let mut out = Vec::with_capacity(self.max_lag + 1);
        for lag in 0..=self.max_lag {
            let mut sum = 0.0_f64;
            for i in 0..(n - lag) {
                sum += signal[i + lag] * signal[i];
            }
            let denom = match self.norm {
                NormMode::Biased => n as f64,
                NormMode::Unbiased => (n - lag) as f64,
            };
            out.push(AcfEntry {
                lag,
                value: (sum / denom, 0.0),
            });
        }
        out
    }

    // ── peak detection ──────────────────────────────────────────────────

    /// Detect peaks in the real part of the ACF.
    ///
    /// Periodicity shows up as local maxima of Re{R(k)} for k > 0.
    /// This works correctly for both real signals (where ACF is real) and
    /// complex signals (where Re{R(k)} = cos(2*pi*k/P) for a tone).
    fn detect_peaks(entries: &[AcfEntry], threshold: f64) -> Vec<usize> {
        if entries.is_empty() {
            return Vec::new();
        }
        let r0 = entries[0].value.0;
        if r0 == 0.0 {
            return Vec::new();
        }
        let mut peaks = Vec::new();
        // Start at lag 1 (skip the trivial zero-lag peak).
        for i in 1..entries.len() {
            let val = entries[i].value.0;
            let prev = entries[i - 1].value.0;
            let next = if i + 1 < entries.len() {
                entries[i + 1].value.0
            } else {
                f64::NEG_INFINITY
            };
            if val >= prev && val >= next && val / r0 >= threshold {
                peaks.push(i);
            }
        }
        // Sort descending by real-part value.
        peaks.sort_by(|&a, &b| {
            let va = entries[a].value.0;
            let vb = entries[b].value.0;
            vb.partial_cmp(&va).unwrap_or(std::cmp::Ordering::Equal)
        });
        peaks
    }

    // ── high-level API ──────────────────────────────────────────────────

    /// Full analysis of a real signal: ACF + peak detection.
    pub fn analyze_real(&self, signal: &[f64]) -> AcfResult {
        let entries = self.compute_real(signal);
        let peaks = Self::detect_peaks(&entries, self.peak_threshold);
        AcfResult { entries, peaks }
    }

    /// Full analysis of a complex signal: ACF + peak detection.
    pub fn analyze_complex(&self, signal: &[C64]) -> AcfResult {
        let entries = self.compute_complex(signal);
        let peaks = Self::detect_peaks(&entries, self.peak_threshold);
        AcfResult { entries, peaks }
    }

    // ── sliding window ──────────────────────────────────────────────────

    /// Sliding-window analysis of a real signal.
    ///
    /// Returns one [`AcfResult`] for every position where a full window
    /// fits, stepping by `hop` samples.
    pub fn sliding_real(&self, signal: &[f64], hop: usize) -> Vec<AcfResult> {
        assert!(hop > 0, "hop must be > 0");
        let mut results = Vec::new();
        let mut start = 0;
        while start + self.window_size <= signal.len() {
            let window = &signal[start..start + self.window_size];
            results.push(self.analyze_real(window));
            start += hop;
        }
        results
    }

    /// Sliding-window analysis of a complex signal.
    pub fn sliding_complex(&self, signal: &[C64], hop: usize) -> Vec<AcfResult> {
        assert!(hop > 0, "hop must be > 0");
        let mut results = Vec::new();
        let mut start = 0;
        while start + self.window_size <= signal.len() {
            let window = &signal[start..start + self.window_size];
            results.push(self.analyze_complex(window));
            start += hop;
        }
        results
    }

    // ── accessors ───────────────────────────────────────────────────────

    /// Maximum lag.
    pub fn max_lag(&self) -> usize {
        self.max_lag
    }

    /// Window size.
    pub fn window_size(&self) -> usize {
        self.window_size
    }
}

// ─── tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a real sinusoid with a given period (in samples).
    fn real_sine(n: usize, period: f64) -> Vec<f64> {
        (0..n)
            .map(|i| (2.0 * PI * i as f64 / period).sin())
            .collect()
    }

    /// Helper: generate a complex tone.
    fn complex_tone(n: usize, period: f64) -> Vec<C64> {
        (0..n)
            .map(|i| {
                let phase = 2.0 * PI * i as f64 / period;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    // 1. Zero-lag equals signal power (biased).
    #[test]
    fn test_zero_lag_is_power_biased() {
        let sig = real_sine(256, 32.0);
        let pa = PeriodicAutocorrelator::new(64, 256);
        let res = pa.compute_real(&sig);
        // R(0) = (1/N) sum x^2 — for a unit-amplitude sine ~ 0.5
        let r0 = res[0].value.0;
        assert!((r0 - 0.5).abs() < 0.01, "R(0) = {r0}, expected ~0.5");
    }

    // 2. Zero-lag equals signal power (unbiased).
    #[test]
    fn test_zero_lag_is_power_unbiased() {
        let sig = real_sine(256, 32.0);
        let pa = PeriodicAutocorrelator::new(64, 256).with_norm(NormMode::Unbiased);
        let res = pa.compute_real(&sig);
        let r0 = res[0].value.0;
        // Unbiased R(0) divides by N-0 = N, same as biased at lag 0
        assert!((r0 - 0.5).abs() < 0.01, "R(0) = {r0}");
    }

    // 3. Real sinusoid period detection.
    #[test]
    fn test_real_sine_period_detection() {
        let period = 20.0;
        let sig = real_sine(400, period);
        let pa = PeriodicAutocorrelator::new(100, 400);
        let result = pa.analyze_real(&sig);
        let detected = result.fundamental_period().unwrap();
        assert!(
            (detected as f64 - period).abs() < 2.0,
            "detected {detected}, expected {period}"
        );
    }

    // 4. Complex tone period detection.
    #[test]
    fn test_complex_tone_period_detection() {
        let period = 25.0;
        let sig = complex_tone(500, period);
        let pa = PeriodicAutocorrelator::new(100, 500);
        let result = pa.analyze_complex(&sig);
        let detected = result.fundamental_period().unwrap();
        assert!(
            (detected as f64 - period).abs() < 2.0,
            "detected {detected}, expected {period}"
        );
    }

    // 5. Periodicity strength for a perfect sinusoid is high.
    #[test]
    fn test_periodicity_strength_sine() {
        let sig = real_sine(400, 20.0);
        let pa = PeriodicAutocorrelator::new(100, 400);
        let result = pa.analyze_real(&sig);
        let strength = result.periodicity_strength();
        assert!(strength > 0.8, "strength = {strength}, expected > 0.8");
    }

    // 6. White noise has low periodicity strength.
    #[test]
    fn test_white_noise_low_strength() {
        // Pseudo-random noise via simple LCG.
        let n = 1024;
        let mut rng: u64 = 12345;
        let noise: Vec<f64> = (0..n)
            .map(|_| {
                rng = rng.wrapping_mul(6364136223846793005).wrapping_add(1);
                (rng as f64 / u64::MAX as f64) * 2.0 - 1.0
            })
            .collect();
        let pa = PeriodicAutocorrelator::new(200, n);
        let result = pa.analyze_real(&noise);
        let strength = result.periodicity_strength();
        assert!(strength < 0.3, "noise strength = {strength}, expected < 0.3");
    }

    // 7. Harmonic detection finds multiple period candidates.
    #[test]
    fn test_harmonic_detection() {
        let sig = real_sine(400, 20.0);
        let pa = PeriodicAutocorrelator::new(100, 400);
        let result = pa.analyze_real(&sig);
        let harmonics = result.harmonics(5);
        assert!(!harmonics.is_empty(), "should find at least one harmonic");
        // First harmonic should be near the fundamental period.
        assert!(
            (harmonics[0] as f64 - 20.0).abs() < 2.0,
            "first harmonic = {}, expected ~20",
            harmonics[0]
        );
    }

    // 8. Normalised ACF has unity at lag 0.
    #[test]
    fn test_normalized_acf_unity_at_zero() {
        let sig = real_sine(256, 16.0);
        let pa = PeriodicAutocorrelator::new(50, 256);
        let result = pa.analyze_real(&sig);
        let norm = result.normalized();
        assert!(
            (norm[0].1 - 1.0).abs() < 1e-12,
            "norm[0] = {}",
            norm[0].1
        );
    }

    // 9. ACF length equals max_lag + 1.
    #[test]
    fn test_acf_length() {
        let sig = real_sine(128, 16.0);
        let pa = PeriodicAutocorrelator::new(40, 128);
        let entries = pa.compute_real(&sig);
        assert_eq!(entries.len(), 41);
    }

    // 10. Real ACF has zero imaginary part.
    #[test]
    fn test_real_input_acf_imaginary_zero() {
        let sig = real_sine(200, 25.0);
        let pa = PeriodicAutocorrelator::new(50, 200);
        let entries = pa.compute_real(&sig);
        for e in &entries {
            assert!(
                e.value.1.abs() < 1e-15,
                "imaginary part at lag {} = {}",
                e.lag,
                e.value.1
            );
        }
    }

    // 11. Sliding window returns correct number of frames.
    #[test]
    fn test_sliding_window_frame_count() {
        let sig = real_sine(1000, 20.0);
        let pa = PeriodicAutocorrelator::new(50, 200);
        let results = pa.sliding_real(&sig, 100);
        // frames: start=0,100,200,...,800 => 9 frames
        assert_eq!(results.len(), 9);
    }

    // 12. Sliding window detects consistent period.
    #[test]
    fn test_sliding_window_consistent_period() {
        let sig = real_sine(1000, 30.0);
        let pa = PeriodicAutocorrelator::new(80, 200);
        let results = pa.sliding_real(&sig, 200);
        for (i, r) in results.iter().enumerate() {
            if let Some(p) = r.fundamental_period() {
                assert!(
                    (p as f64 - 30.0).abs() < 2.0,
                    "frame {i}: detected {p}, expected ~30"
                );
            }
        }
    }

    // 13. Biased ACF: R(0) >= R(k) for all k (real signal).
    #[test]
    fn test_biased_acf_r0_dominates() {
        let sig = real_sine(256, 20.0);
        let pa = PeriodicAutocorrelator::new(80, 256);
        let entries = pa.compute_real(&sig);
        let r0 = entries[0].value.0;
        for e in &entries {
            assert!(
                e.value.0 <= r0 + 1e-12,
                "R({}) = {} > R(0) = {}",
                e.lag,
                e.value.0,
                r0
            );
        }
    }

    // 14. Unbiased normalisation differs from biased at non-zero lag.
    #[test]
    fn test_unbiased_differs_from_biased() {
        let sig = real_sine(128, 16.0);
        let biased = PeriodicAutocorrelator::new(40, 128).compute_real(&sig);
        let unbiased = PeriodicAutocorrelator::new(40, 128)
            .with_norm(NormMode::Unbiased)
            .compute_real(&sig);
        // At lag 0 they should be identical.
        assert!((biased[0].value.0 - unbiased[0].value.0).abs() < 1e-12);
        // At lag > 0, unbiased divides by (N-k) < N, so |unbiased| >= |biased|.
        for k in 1..biased.len() {
            let ub = unbiased[k].value.0.abs();
            let b = biased[k].value.0.abs();
            assert!(
                ub + 1e-12 >= b,
                "lag {k}: unbiased {ub} < biased {b}",
            );
        }
    }

    // 15. DC signal has linearly decreasing biased ACF.
    #[test]
    fn test_dc_signal_constant_acf() {
        let sig = vec![1.0_f64; 256];
        let pa = PeriodicAutocorrelator::new(50, 256);
        let entries = pa.compute_real(&sig);
        for e in &entries {
            // Biased: R(k) = (N-k)/N
            let expected = (256 - e.lag) as f64 / 256.0;
            assert!(
                (e.value.0 - expected).abs() < 1e-10,
                "lag {}: got {}, expected {}",
                e.lag,
                e.value.0,
                expected
            );
        }
    }

    // 16. Complex sliding window works.
    #[test]
    fn test_complex_sliding_window() {
        let sig = complex_tone(600, 15.0);
        let pa = PeriodicAutocorrelator::new(40, 100);
        let results = pa.sliding_complex(&sig, 100);
        assert_eq!(results.len(), 6);
        for r in &results {
            if let Some(p) = r.fundamental_period() {
                assert!(
                    (p as f64 - 15.0).abs() < 2.0,
                    "detected {p}, expected ~15"
                );
            }
        }
    }

    // 17. Peak threshold filters weak peaks.
    #[test]
    fn test_peak_threshold_filtering() {
        let sig = real_sine(400, 20.0);
        let pa_low = PeriodicAutocorrelator::new(100, 400).with_peak_threshold(0.0);
        let pa_high = PeriodicAutocorrelator::new(100, 400).with_peak_threshold(0.9);
        let res_low = pa_low.analyze_real(&sig);
        let res_high = pa_high.analyze_real(&sig);
        // Higher threshold should yield fewer (or equal) peaks.
        assert!(
            res_high.peaks.len() <= res_low.peaks.len(),
            "high {} > low {}",
            res_high.peaks.len(),
            res_low.peaks.len()
        );
    }

    // 18. Accessors return configured values.
    #[test]
    fn test_accessors() {
        let pa = PeriodicAutocorrelator::new(42, 100);
        assert_eq!(pa.max_lag(), 42);
        assert_eq!(pa.window_size(), 100);
    }

    // 19. Two-tone signal detects both periods.
    #[test]
    fn test_two_tone_harmonics() {
        // Combine period 20 and period 40 — harmonics of each other.
        let n = 800;
        let sig: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64;
                (2.0 * PI * t / 20.0).sin() + 0.5 * (2.0 * PI * t / 40.0).sin()
            })
            .collect();
        let pa = PeriodicAutocorrelator::new(100, n);
        let result = pa.analyze_real(&sig);
        let harmonics = result.harmonics(10);
        // Should find peaks near 20 and 40.
        let has_20 = harmonics.iter().any(|&h| (h as f64 - 20.0).abs() < 2.0);
        let has_40 = harmonics.iter().any(|&h| (h as f64 - 40.0).abs() < 2.0);
        assert!(has_20, "should detect period ~20, got {:?}", harmonics);
        assert!(has_40, "should detect period ~40, got {:?}", harmonics);
    }

    // 20. Zero signal produces no peaks.
    #[test]
    fn test_zero_signal_no_peaks() {
        let sig = vec![0.0_f64; 256];
        let pa = PeriodicAutocorrelator::new(50, 256);
        let result = pa.analyze_real(&sig);
        assert!(
            result.peaks.is_empty(),
            "zero signal should have no peaks"
        );
        assert_eq!(result.periodicity_strength(), 0.0);
    }
}
