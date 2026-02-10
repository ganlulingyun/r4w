//! Non-coherent post-detection integration for weak signal detection.
//!
//! This module provides non-coherent integration techniques used in GPS acquisition
//! and radar signal detection to improve SNR when coherent integration alone is
//! insufficient due to unknown carrier phase, Doppler uncertainty, or data-bit
//! transitions.
//!
//! The key idea: coherently integrate over short intervals (where the carrier phase
//! is approximately constant), then combine the power/magnitude of those intervals
//! non-coherently. This trades a small loss in ideal SNR gain for robustness against
//! phase discontinuities.
//!
//! # Detection modes
//!
//! - **Square-law detection** (`|x|^2` accumulation) -- optimal for Gaussian noise,
//!   used in GPS PCPS acquisition.
//! - **Linear envelope detection** (`|x|` accumulation) -- lower loss at high SNR,
//!   sometimes preferred in radar.
//!
//! # Example
//!
//! ```
//! use r4w_core::incoherent_detector::{IncoherentDetector, DetectionMode};
//!
//! // 4 coherent intervals, each of length 8 samples, square-law detection
//! let mut det = IncoherentDetector::new(8, 4, DetectionMode::SquareLaw);
//!
//! // Feed samples (complex as (re, im) tuples)
//! let samples: Vec<(f64, f64)> = (0..32)
//!     .map(|i| {
//!         let phase = i as f64 * 0.3;
//!         (phase.cos() * 2.0, phase.sin() * 2.0)
//!     })
//!     .collect();
//!
//! let result = det.process(&samples);
//! assert!(result.is_some());
//! let output = result.unwrap();
//! assert!(output.metric > 0.0);
//! ```

use std::f64::consts::PI;

// ── Public types ──────────────────────────────────────────────────────

/// Detection mode for non-coherent accumulation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectionMode {
    /// Accumulate `|x|^2` -- optimal under Gaussian noise.
    SquareLaw,
    /// Accumulate `|x|` -- lower loss at high SNR.
    LinearEnvelope,
}

/// Result of a complete non-coherent integration cycle.
#[derive(Debug, Clone)]
pub struct DetectionResult {
    /// The accumulated detection metric (sum of power or magnitude).
    pub metric: f64,
    /// Individual coherent-integration outputs (complex).
    pub coherent_outputs: Vec<(f64, f64)>,
    /// Individual non-coherent contributions (power or magnitude per interval).
    pub interval_values: Vec<f64>,
}

/// M-of-N detection state: declares detection when at least `m` of the last `n`
/// integration cycles exceed the threshold.
#[derive(Debug, Clone)]
pub struct MofNDetector {
    m: usize,
    n: usize,
    history: Vec<bool>,
}

/// Non-coherent post-detection integrator.
///
/// Accumulates complex samples over short *coherent* intervals, then combines
/// the resulting power (or magnitude) of those intervals non-coherently.
#[derive(Debug, Clone)]
pub struct IncoherentDetector {
    /// Number of samples per coherent integration interval.
    coherent_len: usize,
    /// Number of coherent intervals to combine non-coherently.
    noncoherent_count: usize,
    /// Detection mode (square-law or linear envelope).
    mode: DetectionMode,
    /// Internal buffer of incoming samples.
    buffer: Vec<(f64, f64)>,
}

// ── IncoherentDetector implementation ─────────────────────────────────

impl IncoherentDetector {
    /// Create a new detector.
    ///
    /// * `coherent_len` -- number of complex samples per coherent interval.
    /// * `noncoherent_count` -- number of coherent intervals to accumulate.
    /// * `mode` -- square-law or linear envelope.
    ///
    /// # Panics
    ///
    /// Panics if `coherent_len` or `noncoherent_count` is zero.
    pub fn new(coherent_len: usize, noncoherent_count: usize, mode: DetectionMode) -> Self {
        assert!(coherent_len > 0, "coherent_len must be > 0");
        assert!(noncoherent_count > 0, "noncoherent_count must be > 0");
        Self {
            coherent_len,
            noncoherent_count,
            mode,
            buffer: Vec::new(),
        }
    }

    /// Total number of samples required for one complete detection cycle.
    pub fn required_samples(&self) -> usize {
        self.coherent_len * self.noncoherent_count
    }

    /// Reset internal state, discarding any buffered samples.
    pub fn reset(&mut self) {
        self.buffer.clear();
    }

    /// Feed samples into the detector.
    ///
    /// Returns `Some(DetectionResult)` when enough samples have been accumulated
    /// to complete a full non-coherent integration cycle. Remaining samples beyond
    /// the required count are kept for the next cycle.
    pub fn process(&mut self, samples: &[(f64, f64)]) -> Option<DetectionResult> {
        self.buffer.extend_from_slice(samples);
        let required = self.required_samples();
        if self.buffer.len() < required {
            return None;
        }

        // Drain exactly `required` samples.
        let work: Vec<(f64, f64)> = self.buffer.drain(..required).collect();
        Some(self.integrate(&work))
    }

    /// Perform integration on exactly `required_samples()` samples.
    fn integrate(&self, samples: &[(f64, f64)]) -> DetectionResult {
        let mut coherent_outputs = Vec::with_capacity(self.noncoherent_count);
        let mut interval_values = Vec::with_capacity(self.noncoherent_count);
        let mut metric = 0.0;

        for k in 0..self.noncoherent_count {
            let start = k * self.coherent_len;
            let end = start + self.coherent_len;
            let (coh_re, coh_im) = coherent_accumulate(&samples[start..end]);
            coherent_outputs.push((coh_re, coh_im));

            let val = match self.mode {
                DetectionMode::SquareLaw => coh_re * coh_re + coh_im * coh_im,
                DetectionMode::LinearEnvelope => (coh_re * coh_re + coh_im * coh_im).sqrt(),
            };
            interval_values.push(val);
            metric += val;
        }

        DetectionResult {
            metric,
            coherent_outputs,
            interval_values,
        }
    }

    /// Compute the SNR improvement factor (in dB) from non-coherent integration
    /// of `noncoherent_count` intervals, each of `coherent_len` samples.
    ///
    /// Coherent integration gives `10 * log10(coherent_len)` dB gain.
    /// Non-coherent integration adds approximately `10 * log10(noncoherent_count)`
    /// minus a *non-coherent loss* that depends on the number of intervals and
    /// the detection mode. For square-law detection the loss factor is
    /// approximately `10 * log10(1 + 1/sqrt(noncoherent_count))`.
    pub fn snr_improvement_db(&self) -> f64 {
        let coh_gain = 10.0 * (self.coherent_len as f64).log10();
        let nc = self.noncoherent_count as f64;
        let nc_gain = 10.0 * nc.log10();
        let nc_loss = match self.mode {
            DetectionMode::SquareLaw => 10.0 * (1.0 + 1.0 / nc.sqrt()).log10(),
            DetectionMode::LinearEnvelope => 10.0 * (1.0 + 0.5 / nc.sqrt()).log10(),
        };
        coh_gain + nc_gain - nc_loss
    }
}

// ── MofNDetector implementation ───────────────────────────────────────

impl MofNDetector {
    /// Create a new M-of-N detector.
    ///
    /// # Panics
    ///
    /// Panics if `m > n` or either is zero.
    pub fn new(m: usize, n: usize) -> Self {
        assert!(m > 0 && n > 0, "m and n must be > 0");
        assert!(m <= n, "m must be <= n");
        Self {
            m,
            n,
            history: Vec::new(),
        }
    }

    /// Record whether the latest integration cycle exceeded the threshold, and
    /// return `true` if detection is declared (at least `m` of the last `n`
    /// entries are `true`).
    pub fn update(&mut self, exceeded: bool) -> bool {
        self.history.push(exceeded);
        if self.history.len() > self.n {
            self.history.remove(0);
        }
        self.detections() >= self.m && self.history.len() == self.n
    }

    /// Number of threshold crossings in the current window.
    pub fn detections(&self) -> usize {
        self.history.iter().filter(|&&v| v).count()
    }

    /// Reset history.
    pub fn reset(&mut self) {
        self.history.clear();
    }
}

// ── Free functions ────────────────────────────────────────────────────

/// Coherent accumulation: complex sum of a slice of `(re, im)` samples.
pub fn coherent_accumulate(samples: &[(f64, f64)]) -> (f64, f64) {
    samples
        .iter()
        .fold((0.0, 0.0), |(ar, ai), &(sr, si)| (ar + sr, ai + si))
}

/// Non-coherent accumulation of pre-computed coherent outputs using square-law
/// detection: returns the sum of `|x_k|^2`.
pub fn noncoherent_square_law(coherent_outputs: &[(f64, f64)]) -> f64 {
    coherent_outputs
        .iter()
        .map(|&(re, im)| re * re + im * im)
        .sum()
}

/// Non-coherent accumulation using linear envelope detection: returns the sum of
/// `|x_k|`.
pub fn noncoherent_linear_envelope(coherent_outputs: &[(f64, f64)]) -> f64 {
    coherent_outputs
        .iter()
        .map(|&(re, im)| (re * re + im * im).sqrt())
        .sum()
}

/// Estimate detection probability using a Gaussian approximation for square-law
/// non-coherent integration.
///
/// For `n` non-coherent integrations of a signal with per-sample `snr_linear`
/// (linear, not dB) in complex Gaussian noise, the detection statistic under H1
/// is approximately Gaussian for moderate `n`:
///
/// * H0: mean = `n`, variance = `n`  (normalised to unit noise power)
/// * H1: mean = `n * (1 + SNR)`, variance = `n * (1 + 2*SNR)`
///
/// Given a false alarm probability `pfa`, the threshold is derived from H0, then
/// `Pd = Q((sqrt(n) * Q^{-1}(pfa) - n * SNR) / sqrt(n * (1 + 2*SNR)))`.
///
/// Returns a value in `[0, 1]`.
pub fn detection_probability_albersheim(snr_linear: f64, pfa: f64, n: usize) -> f64 {
    if pfa <= 0.0 || pfa >= 1.0 || snr_linear <= 0.0 || n == 0 {
        return 0.0;
    }

    let nf = n as f64;
    let qinv_pfa = q_function_inv(pfa);

    let numerator = nf.sqrt() * qinv_pfa - nf * snr_linear;
    let denominator = (nf * (1.0 + 2.0 * snr_linear)).sqrt();

    if denominator <= 0.0 {
        return 0.0;
    }

    q_function(numerator / denominator).clamp(0.0, 1.0)
}

/// Compute false alarm probability from a threshold and noise power.
///
/// For a square-law detector with `n` non-coherent integrations of
/// complex Gaussian noise with power `noise_power`, the noise-only
/// detection statistic has mean `n * noise_power` and standard deviation
/// `noise_power * sqrt(n)` (Gaussian approximation for moderate `n`).
///
/// `Pfa = Q((threshold - mean) / std_dev)`
pub fn false_alarm_probability(threshold: f64, noise_power: f64, n: usize) -> f64 {
    if noise_power <= 0.0 || n == 0 {
        return 0.0;
    }
    let mean = noise_power * n as f64;
    let std_dev = noise_power * (n as f64).sqrt();
    if std_dev <= 0.0 {
        return 0.0;
    }
    let z = (threshold - mean) / std_dev;
    q_function(z)
}

/// CFAR-like adaptive threshold: computes a threshold from a set of reference
/// cells (noise-only power estimates).
///
/// `threshold = alpha * mean(reference_cells)`
///
/// where `alpha > 1` is a multiplier that controls the false alarm rate.
///
/// Returns `(threshold, noise_estimate)`.
pub fn cfar_threshold(reference_cells: &[f64], alpha: f64) -> (f64, f64) {
    if reference_cells.is_empty() {
        return (0.0, 0.0);
    }
    let sum: f64 = reference_cells.iter().sum();
    let noise_est = sum / reference_cells.len() as f64;
    (alpha * noise_est, noise_est)
}

/// SNR improvement factor (linear) from `n` non-coherent integrations
/// under square-law detection. Approximately `sqrt(n)` for moderate `n`,
/// but accounting for non-coherent loss:
///
/// `improvement = n / (1 + 1/sqrt(n))`
pub fn snr_improvement_factor(n: usize) -> f64 {
    if n == 0 {
        return 0.0;
    }
    let nf = n as f64;
    nf / (1.0 + 1.0 / nf.sqrt())
}

// ── Internal helpers ──────────────────────────────────────────────────

/// Gaussian Q-function: `Q(x) = 0.5 * erfc(x / sqrt(2))`.
fn q_function(x: f64) -> f64 {
    0.5 * erfc_approx(x / std::f64::consts::SQRT_2)
}

/// Inverse Q-function: given `p`, find `x` such that `Q(x) = p`.
/// Uses Newton's method with a rational initial approximation.
fn q_function_inv(p: f64) -> f64 {
    if p <= 0.0 {
        return f64::INFINITY;
    }
    if p >= 1.0 {
        return f64::NEG_INFINITY;
    }
    if (p - 0.5).abs() < 1e-15 {
        return 0.0;
    }

    // Initial approximation using Beasley-Springer-Moro for the normal quantile.
    // Q(x) = p  =>  erfc(x/sqrt2) = 2p  =>  x = sqrt(2) * erfc_inv(2p)
    // Use the approximation: for p < 0.5, x > 0.
    let mut x = if p < 0.5 {
        // Rational approx for small p
        let t = (-2.0 * p.ln()).sqrt();
        t - (2.515517 + t * (0.802853 + t * 0.010328))
            / (1.0 + t * (1.432788 + t * (0.189269 + t * 0.001308)))
    } else {
        let pp = 1.0 - p;
        let t = (-2.0 * pp.ln()).sqrt();
        -(t - (2.515517 + t * (0.802853 + t * 0.010328))
            / (1.0 + t * (1.432788 + t * (0.189269 + t * 0.001308))))
    };

    // Refine with Newton iterations: Q'(x) = -phi(x)
    for _ in 0..8 {
        let qx = q_function(x);
        let phi = (-x * x / 2.0).exp() / (2.0 * PI).sqrt();
        if phi.abs() < 1e-300 {
            break;
        }
        x -= (qx - p) / (-phi);
    }

    x
}

/// Approximate complementary error function using Horner-form rational
/// approximation (Abramowitz & Stegun 7.1.26, max error < 1.5e-7).
fn erfc_approx(x: f64) -> f64 {
    if x < 0.0 {
        return 2.0 - erfc_approx(-x);
    }
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t
        * (0.254829592
            + t * (-0.284496736 + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    poly * (-x * x).exp()
}

// ── Tests ─────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a tone at a given frequency (normalised), with amplitude.
    fn tone(n: usize, freq: f64, amplitude: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let phase = 2.0 * PI * freq * i as f64;
                (amplitude * phase.cos(), amplitude * phase.sin())
            })
            .collect()
    }

    // ── Basic construction ────────────────────────────────────────────

    #[test]
    fn test_new_detector() {
        let det = IncoherentDetector::new(16, 4, DetectionMode::SquareLaw);
        assert_eq!(det.required_samples(), 64);
    }

    #[test]
    #[should_panic]
    fn test_new_zero_coherent_len() {
        IncoherentDetector::new(0, 4, DetectionMode::SquareLaw);
    }

    #[test]
    #[should_panic]
    fn test_new_zero_noncoherent_count() {
        IncoherentDetector::new(16, 0, DetectionMode::SquareLaw);
    }

    // ── Coherent accumulation ─────────────────────────────────────────

    #[test]
    fn test_coherent_accumulate_simple() {
        let samples = vec![(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)];
        let (re, im) = coherent_accumulate(&samples);
        assert!((re - 9.0).abs() < 1e-12);
        assert!((im - 12.0).abs() < 1e-12);
    }

    #[test]
    fn test_coherent_accumulate_cancellation() {
        // Opposite-phase samples should cancel.
        let samples = vec![(1.0, 0.0), (-1.0, 0.0)];
        let (re, im) = coherent_accumulate(&samples);
        assert!(re.abs() < 1e-12);
        assert!(im.abs() < 1e-12);
    }

    // ── Non-coherent free functions ───────────────────────────────────

    #[test]
    fn test_noncoherent_square_law() {
        let outputs = vec![(3.0, 4.0), (1.0, 0.0)];
        // |3+4i|^2 = 25, |1+0i|^2 = 1 -> 26
        let val = noncoherent_square_law(&outputs);
        assert!((val - 26.0).abs() < 1e-12);
    }

    #[test]
    fn test_noncoherent_linear_envelope() {
        let outputs = vec![(3.0, 4.0), (0.0, 1.0)];
        // |3+4i| = 5, |0+1i| = 1 -> 6
        let val = noncoherent_linear_envelope(&outputs);
        assert!((val - 6.0).abs() < 1e-12);
    }

    // ── Detector process ──────────────────────────────────────────────

    #[test]
    fn test_process_returns_none_when_insufficient() {
        let mut det = IncoherentDetector::new(8, 4, DetectionMode::SquareLaw);
        let samples = vec![(1.0, 0.0); 10]; // need 32
        assert!(det.process(&samples).is_none());
    }

    #[test]
    fn test_process_returns_result_when_sufficient() {
        let mut det = IncoherentDetector::new(4, 2, DetectionMode::SquareLaw);
        let samples = tone(8, 0.1, 1.0);
        let result = det.process(&samples);
        assert!(result.is_some());
        let r = result.unwrap();
        assert_eq!(r.coherent_outputs.len(), 2);
        assert_eq!(r.interval_values.len(), 2);
        assert!(r.metric > 0.0);
    }

    #[test]
    fn test_process_buffers_excess_samples() {
        let mut det = IncoherentDetector::new(4, 2, DetectionMode::SquareLaw);
        // Feed 12 samples: first 8 consumed, 4 remain.
        let samples = tone(12, 0.1, 1.0);
        let r1 = det.process(&samples);
        assert!(r1.is_some());
        // Feed 4 more -> should complete second cycle.
        let more = tone(4, 0.1, 1.0);
        let r2 = det.process(&more);
        assert!(r2.is_some());
    }

    #[test]
    fn test_square_law_vs_linear_envelope() {
        let samples = tone(16, 0.05, 3.0);
        let mut sq = IncoherentDetector::new(4, 4, DetectionMode::SquareLaw);
        let mut le = IncoherentDetector::new(4, 4, DetectionMode::LinearEnvelope);
        let r_sq = sq.process(&samples).unwrap();
        let r_le = le.process(&samples).unwrap();
        // Square-law metric should be larger (power vs. magnitude).
        assert!(r_sq.metric > r_le.metric);
    }

    #[test]
    fn test_reset_clears_buffer() {
        let mut det = IncoherentDetector::new(8, 4, DetectionMode::SquareLaw);
        det.process(&tone(10, 0.1, 1.0));
        det.reset();
        // After reset, we should need all 32 samples again.
        assert!(det.process(&tone(10, 0.1, 1.0)).is_none());
    }

    // ── SNR improvement ───────────────────────────────────────────────

    #[test]
    fn test_snr_improvement_db_positive() {
        let det = IncoherentDetector::new(16, 8, DetectionMode::SquareLaw);
        let gain = det.snr_improvement_db();
        // Must be positive (integration always helps).
        assert!(gain > 0.0);
        // Coherent gain alone: 10*log10(16) ~ 12 dB, plus NC gain, so > 12 dB.
        assert!(gain > 12.0);
    }

    #[test]
    fn test_snr_improvement_linear_lower_loss() {
        // Linear envelope has lower NC loss at same parameters.
        let sq = IncoherentDetector::new(8, 4, DetectionMode::SquareLaw);
        let le = IncoherentDetector::new(8, 4, DetectionMode::LinearEnvelope);
        assert!(le.snr_improvement_db() > sq.snr_improvement_db());
    }

    #[test]
    fn test_snr_improvement_factor_monotonic() {
        let v: Vec<f64> = (1..=10).map(snr_improvement_factor).collect();
        for i in 1..v.len() {
            assert!(v[i] > v[i - 1], "SNR improvement should increase with n");
        }
    }

    // ── Detection probability ─────────────────────────────────────────

    #[test]
    fn test_albersheim_high_snr_gives_high_pd() {
        // 20 dB single-pulse SNR should easily detect
        let pd = detection_probability_albersheim(100.0, 1e-6, 1);
        assert!(pd > 0.8, "High SNR should give high Pd, got {pd}");
    }

    #[test]
    fn test_albersheim_low_snr_gives_low_pd() {
        // -20 dB SNR should barely detect
        let pd = detection_probability_albersheim(0.01, 1e-6, 1);
        assert!(pd < 0.5, "Low SNR should give low Pd, got {pd}");
    }

    #[test]
    fn test_albersheim_more_pulses_improves_pd() {
        let pd1 = detection_probability_albersheim(3.0, 1e-6, 1);
        let pd10 = detection_probability_albersheim(3.0, 1e-6, 10);
        assert!(pd10 >= pd1, "More pulses should help: pd1={pd1}, pd10={pd10}");
    }

    // ── False alarm probability ───────────────────────────────────────

    #[test]
    fn test_pfa_high_threshold_low_pfa() {
        let pfa = false_alarm_probability(100.0, 1.0, 4);
        assert!(pfa < 0.01, "Very high threshold should give Pfa near 0, got {pfa}");
    }

    #[test]
    fn test_pfa_low_threshold_high_pfa() {
        let pfa = false_alarm_probability(0.0, 1.0, 4);
        assert!(pfa > 0.4, "Threshold at 0 should give high Pfa, got {pfa}");
    }

    // ── CFAR threshold ────────────────────────────────────────────────

    #[test]
    fn test_cfar_threshold_basic() {
        let cells = vec![1.0, 2.0, 3.0, 4.0];
        let (thr, noise) = cfar_threshold(&cells, 3.0);
        assert!((noise - 2.5).abs() < 1e-12);
        assert!((thr - 7.5).abs() < 1e-12);
    }

    #[test]
    fn test_cfar_threshold_empty() {
        let (thr, noise) = cfar_threshold(&[], 5.0);
        assert_eq!(thr, 0.0);
        assert_eq!(noise, 0.0);
    }

    // ── M-of-N detector ──────────────────────────────────────────────

    #[test]
    fn test_mofn_basic() {
        let mut det = MofNDetector::new(3, 5);
        assert!(!det.update(true));  // 1/1 -- window not full
        assert!(!det.update(false)); // 1/2
        assert!(!det.update(true));  // 2/3
        assert!(!det.update(true));  // 3/4 -- window not full yet
        assert!(det.update(true));   // 4/5 -- 4 >= 3, detect!
    }

    #[test]
    fn test_mofn_sliding_window() {
        let mut det = MofNDetector::new(2, 3);
        det.update(true);
        det.update(true);
        let d = det.update(false); // window: [T, T, F] -> 2 >= 2 -> detect
        assert!(d);
        let d2 = det.update(false); // window: [T, F, F] -> 1 < 2 -> no
        assert!(!d2);
    }

    #[test]
    fn test_mofn_reset() {
        let mut det = MofNDetector::new(1, 2);
        det.update(true);
        det.update(true);
        det.reset();
        assert_eq!(det.detections(), 0);
    }

    #[test]
    #[should_panic]
    fn test_mofn_m_greater_than_n() {
        MofNDetector::new(5, 3);
    }
}
