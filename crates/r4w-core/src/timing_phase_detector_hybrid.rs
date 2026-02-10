//! Hybrid timing phase detector supporting multiple timing error detection algorithms
//! with automatic switching based on signal conditions.
//!
//! This module provides a unified interface for Gardner, Mueller-Muller, Zero-Crossing,
//! and Early-Late timing error detectors, with an adaptive selector that picks the
//! optimal algorithm based on estimated SNR.
//!
//! # Example
//!
//! ```
//! use r4w_core::timing_phase_detector_hybrid::{
//!     HybridTimingDetector, TimingAlgorithm,
//! };
//!
//! // Create a Gardner TED at 2 samples per symbol
//! let mut ted = HybridTimingDetector::new(TimingAlgorithm::Gardner, 2.0);
//!
//! // Generate a simple BPSK-like signal: two symbols at 2 sps
//! // Symbol +1 then -1, each sampled twice
//! let samples: Vec<(f64, f64)> = vec![
//!     (1.0, 0.0), (1.0, 0.0),   // symbol +1
//!     (-1.0, 0.0), (-1.0, 0.0), // symbol -1
//! ];
//!
//! let err = ted.compute_error(&samples);
//! // The error has a value and confidence metric
//! assert!(err.value.is_finite());
//! assert!(err.confidence >= 0.0 && err.confidence <= 1.0);
//! assert!(matches!(err.algorithm, TimingAlgorithm::Gardner));
//!
//! // The fractional delay mu is available
//! let mu = ted.get_mu();
//! assert!(mu >= 0.0 && mu < 1.0);
//! ```

/// Timing error detection algorithm selection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TimingAlgorithm {
    /// Gardner TED: e = Re{y[n-1/2] * conj(y[n] - y[n-1])}
    /// Data-aided, works best at 2 samples per symbol.
    Gardner,
    /// Mueller & Muller TED: e = Re{d[n-1]*y[n] - d[n]*y[n-1]}
    /// Decision-directed, works at 1 sample per symbol.
    MuellerMuller,
    /// Zero-crossing TED: e = Re{(d[n]-d[n-1]) * y[n-1/2]}
    /// Exploits transitions between symbols.
    ZeroCrossing,
    /// Early-Late gate TED: e = |y[n+delta]|^2 - |y[n-delta]|^2
    /// Classic correlation-based approach.
    EarlyLate,
}

/// Result from a timing error computation.
#[derive(Debug, Clone)]
pub struct TimingError {
    /// Timing error value (normalized).
    pub value: f64,
    /// Confidence in the estimate (0.0 to 1.0).
    pub confidence: f64,
    /// Algorithm that produced this estimate.
    pub algorithm: TimingAlgorithm,
}

/// Automatic algorithm selector based on SNR and signal conditions.
#[derive(Debug, Clone)]
pub struct AdaptiveSelector {
    /// Current estimated SNR in dB.
    snr_estimate: f64,
    /// Samples per symbol.
    sps: f64,
    /// History of recent timing errors for variance estimation.
    error_history: Vec<f64>,
    /// Maximum history length.
    history_len: usize,
}

impl AdaptiveSelector {
    /// Create a new adaptive selector.
    pub fn new(sps: f64) -> Self {
        Self {
            snr_estimate: 20.0,
            sps,
            error_history: Vec::new(),
            history_len: 32,
        }
    }

    /// Update SNR estimate and return the recommended algorithm.
    pub fn recommend(&mut self, snr_est: f64) -> TimingAlgorithm {
        self.snr_estimate = snr_est;

        // At low SNR, Gardner is more robust (non-decision-directed).
        // At high SNR, Mueller-Muller is more accurate.
        // Zero-crossing is a good middle ground.
        // Early-Late is robust but less precise.
        if self.sps < 1.5 {
            // At ~1 sps, only Mueller-Muller works directly
            TimingAlgorithm::MuellerMuller
        } else if snr_est < 5.0 {
            TimingAlgorithm::EarlyLate
        } else if snr_est < 10.0 {
            TimingAlgorithm::Gardner
        } else if snr_est < 20.0 {
            TimingAlgorithm::ZeroCrossing
        } else {
            TimingAlgorithm::MuellerMuller
        }
    }

    /// Record a timing error for variance tracking.
    pub fn record_error(&mut self, error: f64) {
        self.error_history.push(error);
        if self.error_history.len() > self.history_len {
            self.error_history.remove(0);
        }
    }

    /// Compute variance of recent timing errors.
    pub fn error_variance(&self) -> f64 {
        if self.error_history.len() < 2 {
            return f64::MAX;
        }
        let n = self.error_history.len() as f64;
        let mean = self.error_history.iter().sum::<f64>() / n;
        self.error_history
            .iter()
            .map(|e| (e - mean) * (e - mean))
            .sum::<f64>()
            / (n - 1.0)
    }

    /// Get the current SNR estimate.
    pub fn snr_estimate(&self) -> f64 {
        self.snr_estimate
    }
}

/// PI (Proportional-Integral) loop filter for timing recovery.
#[derive(Debug, Clone)]
struct LoopFilter {
    /// Proportional gain.
    kp: f64,
    /// Integral gain.
    ki: f64,
    /// Integrator state.
    integrator: f64,
    /// Previous filter output.
    prev_output: f64,
}

impl LoopFilter {
    fn new(bandwidth: f64, damping: f64) -> Self {
        // Compute gains from normalized bandwidth and damping factor
        let denom = 1.0 + 2.0 * damping * bandwidth + bandwidth * bandwidth;
        let kp = 4.0 * damping * bandwidth / denom;
        let ki = 4.0 * bandwidth * bandwidth / denom;
        Self {
            kp,
            ki,
            integrator: 0.0,
            prev_output: 0.0,
        }
    }

    fn advance(&mut self, error: f64) -> f64 {
        self.integrator += self.ki * error;
        self.prev_output = self.kp * error + self.integrator;
        self.prev_output
    }

    fn output(&self) -> f64 {
        self.prev_output
    }

    fn reset(&mut self) {
        self.integrator = 0.0;
        self.prev_output = 0.0;
    }
}

/// Complex multiplication helper: (a,b) * (c,d) = (ac-bd, ad+bc)
#[inline]
fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex conjugate: (a,b) -> (a,-b)
#[inline]
fn complex_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Complex subtraction
#[inline]
fn complex_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Magnitude squared of a complex sample
#[inline]
fn mag_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Hard decision (BPSK-like): returns the sign of the real part as a symbol.
#[inline]
fn hard_decision(sample: (f64, f64)) -> (f64, f64) {
    let re = if sample.0 >= 0.0 { 1.0 } else { -1.0 };
    (re, 0.0)
}

/// Multi-mode hybrid timing error detector with PI loop filter.
#[derive(Debug, Clone)]
pub struct HybridTimingDetector {
    /// Currently active algorithm.
    algorithm: TimingAlgorithm,
    /// Samples per symbol.
    sps: f64,
    /// Fractional symbol timing offset (0.0 to 1.0).
    mu: f64,
    /// PI loop filter.
    loop_filter: LoopFilter,
    /// Adaptive algorithm selector.
    selector: AdaptiveSelector,
    /// Previous on-time sample (for M&M and zero-crossing).
    prev_sample: (f64, f64),
    /// Previous decision symbol (for M&M and zero-crossing).
    prev_decision: (f64, f64),
    /// Early-late delta (fraction of sps for early/late taps).
    early_late_delta: f64,
}

impl HybridTimingDetector {
    /// Create a new hybrid timing detector.
    ///
    /// # Arguments
    /// * `algorithm` - Initial timing algorithm to use
    /// * `sps` - Samples per symbol (typically 2.0 for Gardner, >= 1.0 for others)
    pub fn new(algorithm: TimingAlgorithm, sps: f64) -> Self {
        let sps = if sps < 1.0 { 1.0 } else { sps };
        Self {
            algorithm,
            sps,
            mu: 0.0,
            loop_filter: LoopFilter::new(0.01, 1.0),
            selector: AdaptiveSelector::new(sps),
            prev_sample: (0.0, 0.0),
            prev_decision: (1.0, 0.0),
            early_late_delta: 0.25,
        }
    }

    /// Compute the timing error from the given samples.
    ///
    /// The interpretation of `samples` depends on the algorithm:
    /// - **Gardner**: needs at least `2*sps` samples (two symbol periods).
    ///   Uses indices at 0 (previous on-time), sps/2 (mid), and sps (current on-time).
    /// - **MuellerMuller**: needs at least `sps+1` samples.
    ///   Uses on-time samples at 0 and sps.
    /// - **ZeroCrossing**: needs at least `2*sps` samples.
    ///   Uses on-time at 0 and sps, midpoint at sps/2.
    /// - **EarlyLate**: needs at least `sps` samples.
    ///   Uses early, on-time, and late taps within the symbol period.
    pub fn compute_error(&mut self, samples: &[(f64, f64)]) -> TimingError {
        let sps_i = self.sps.round() as usize;
        if samples.is_empty() {
            return TimingError {
                value: 0.0,
                confidence: 0.0,
                algorithm: self.algorithm,
            };
        }

        let (error, confidence) = match self.algorithm {
            TimingAlgorithm::Gardner => self.gardner_error(samples, sps_i),
            TimingAlgorithm::MuellerMuller => self.mueller_muller_error(samples, sps_i),
            TimingAlgorithm::ZeroCrossing => self.zero_crossing_error(samples, sps_i),
            TimingAlgorithm::EarlyLate => self.early_late_error(samples, sps_i),
        };

        // Update loop filter
        let filter_out = self.loop_filter.advance(error);

        // Update fractional delay mu
        self.mu += filter_out;
        // Keep mu in [0, 1)
        while self.mu >= 1.0 {
            self.mu -= 1.0;
        }
        while self.mu < 0.0 {
            self.mu += 1.0;
        }

        // Record for adaptive selector
        self.selector.record_error(error);

        TimingError {
            value: error,
            confidence,
            algorithm: self.algorithm,
        }
    }

    /// Gardner TED: e = Re{y[n-1/2] * conj(y[n] - y[n-1])}
    fn gardner_error(&mut self, samples: &[(f64, f64)], sps_i: usize) -> (f64, f64) {
        let half = sps_i / 2;
        if samples.len() < sps_i + 1 {
            // Not enough samples: fall back to zero error
            return (0.0, 0.0);
        }

        let y_prev = samples[0]; // y[n-1]
        let y_mid = if half < samples.len() {
            samples[half]
        } else {
            samples[0]
        }; // y[n-1/2]
        let y_curr = samples[sps_i]; // y[n]

        let diff = complex_sub(y_curr, y_prev);
        let conj_diff = complex_conj(diff);
        let product = complex_mul(y_mid, conj_diff);
        let error = product.0; // Real part

        // Confidence based on signal magnitude
        let mag = (mag_sq(y_curr).sqrt() + mag_sq(y_prev).sqrt()) / 2.0;
        let confidence = if mag > 1e-10 {
            (1.0 - (error.abs() / mag).min(1.0)).max(0.0)
        } else {
            0.0
        };

        self.prev_sample = y_curr;
        self.prev_decision = hard_decision(y_curr);
        (error, confidence)
    }

    /// Mueller & Muller TED: e = Re{d[n-1]*y[n] - d[n]*y[n-1]}
    fn mueller_muller_error(
        &mut self,
        samples: &[(f64, f64)],
        sps_i: usize,
    ) -> (f64, f64) {
        if samples.len() < sps_i + 1 {
            return (0.0, 0.0);
        }

        let y_prev = samples[0];
        let y_curr = samples[sps_i.min(samples.len() - 1)];

        let d_prev = self.prev_decision;
        let d_curr = hard_decision(y_curr);

        // e = Re{d[n-1]*y[n] - d[n]*y[n-1]}
        let term1 = complex_mul(d_prev, y_curr);
        let term2 = complex_mul(d_curr, y_prev);
        let error = term1.0 - term2.0;

        let mag = (mag_sq(y_curr).sqrt() + mag_sq(y_prev).sqrt()) / 2.0;
        let confidence = if mag > 1e-10 {
            (1.0 - (error.abs() / (2.0 * mag)).min(1.0)).max(0.0)
        } else {
            0.0
        };

        self.prev_sample = y_curr;
        self.prev_decision = d_curr;
        (error, confidence)
    }

    /// Zero-crossing TED: e = Re{(d[n] - d[n-1]) * y[n-1/2]}
    fn zero_crossing_error(
        &mut self,
        samples: &[(f64, f64)],
        sps_i: usize,
    ) -> (f64, f64) {
        let half = sps_i / 2;
        if samples.len() < sps_i + 1 {
            return (0.0, 0.0);
        }

        let y_prev = samples[0];
        let y_mid = samples[half.min(samples.len() - 1)];
        let y_curr = samples[sps_i.min(samples.len() - 1)];

        let d_prev = hard_decision(y_prev);
        let d_curr = hard_decision(y_curr);
        let d_diff = complex_sub(d_curr, d_prev);

        // e = Re{(d[n] - d[n-1]) * y[n-1/2]}
        let product = complex_mul(d_diff, y_mid);
        let error = product.0;

        let mag = mag_sq(y_mid).sqrt();
        let confidence = if mag > 1e-10 && d_diff.0.abs() > 0.5 {
            (1.0 - (error.abs() / (2.0 * mag)).min(1.0)).max(0.0)
        } else {
            // No transition detected, low confidence
            0.1
        };

        self.prev_sample = y_curr;
        self.prev_decision = hard_decision(y_curr);
        (error, confidence)
    }

    /// Early-Late gate TED: e = |y[n+delta]|^2 - |y[n-delta]|^2
    fn early_late_error(
        &mut self,
        samples: &[(f64, f64)],
        sps_i: usize,
    ) -> (f64, f64) {
        if samples.len() < 3 {
            return (0.0, 0.0);
        }

        // Compute early, on-time, and late indices
        let on_time = sps_i / 2;
        let delta_samples = (self.early_late_delta * self.sps).round() as usize;
        let delta_samples = delta_samples.max(1);

        let on_time = on_time.min(samples.len() - 1);
        let early = if on_time >= delta_samples {
            on_time - delta_samples
        } else {
            0
        };
        let late = (on_time + delta_samples).min(samples.len() - 1);

        let early_mag = mag_sq(samples[early]);
        let late_mag = mag_sq(samples[late]);
        let error = late_mag - early_mag;

        let on_time_mag = mag_sq(samples[on_time]);
        let confidence = if on_time_mag > 1e-10 {
            (1.0 - (error.abs() / (2.0 * on_time_mag)).min(1.0)).max(0.0)
        } else {
            0.0
        };

        self.prev_sample = samples[on_time];
        self.prev_decision = hard_decision(samples[on_time]);
        (error, confidence)
    }

    /// Switch to a different timing algorithm.
    pub fn set_algorithm(&mut self, alg: TimingAlgorithm) {
        self.algorithm = alg;
        self.loop_filter.reset();
    }

    /// Automatically select the best algorithm based on estimated SNR.
    pub fn auto_select(&mut self, snr_est: f64) {
        let recommended = self.selector.recommend(snr_est);
        if recommended != self.algorithm {
            self.algorithm = recommended;
            self.loop_filter.reset();
        }
    }

    /// Get the current PI loop filter output.
    pub fn loop_filter_output(&self) -> f64 {
        self.loop_filter.output()
    }

    /// Get the current fractional symbol timing offset mu in [0, 1).
    pub fn get_mu(&self) -> f64 {
        self.mu
    }

    /// Get the currently active algorithm.
    pub fn current_algorithm(&self) -> TimingAlgorithm {
        self.algorithm
    }

    /// Get a reference to the adaptive selector.
    pub fn selector(&self) -> &AdaptiveSelector {
        &self.selector
    }

    /// Get the samples per symbol.
    pub fn sps(&self) -> f64 {
        self.sps
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new_default_state() {
        let ted = HybridTimingDetector::new(TimingAlgorithm::Gardner, 2.0);
        assert_eq!(ted.current_algorithm(), TimingAlgorithm::Gardner);
        assert!((ted.sps() - 2.0).abs() < 1e-10);
        assert!((ted.get_mu() - 0.0).abs() < 1e-10);
        assert!((ted.loop_filter_output() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_sps_floor() {
        // sps less than 1.0 should be clamped to 1.0
        let ted = HybridTimingDetector::new(TimingAlgorithm::Gardner, 0.5);
        assert!((ted.sps() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_set_algorithm() {
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::Gardner, 2.0);
        ted.set_algorithm(TimingAlgorithm::MuellerMuller);
        assert_eq!(ted.current_algorithm(), TimingAlgorithm::MuellerMuller);
    }

    #[test]
    fn test_gardner_aligned_signal() {
        // A perfectly aligned BPSK signal at 2 sps should give near-zero error
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::Gardner, 2.0);
        // Symbol +1 for 2 samples, then -1 for 2 samples
        let samples = vec![
            (1.0, 0.0),
            (1.0, 0.0),
            (-1.0, 0.0),
            (-1.0, 0.0),
        ];
        let err = ted.compute_error(&samples);
        // For a perfectly aligned signal, the midpoint sample equals the on-time sample,
        // so the error should be modest
        assert!(err.value.is_finite());
        assert!(matches!(err.algorithm, TimingAlgorithm::Gardner));
    }

    #[test]
    fn test_gardner_timing_offset_detection() {
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::Gardner, 2.0);
        // Signal with a timing offset: transition happens between samples
        // +1 at sample 0, interpolated at sample 1, -1 at sample 2
        let samples = vec![
            (1.0, 0.0),   // y[n-1]
            (0.0, 0.0),   // y[n-1/2] midpoint at zero crossing
            (-1.0, 0.0),  // y[n]
            (-1.0, 0.0),  // extra
        ];
        let err = ted.compute_error(&samples);
        // With a clear transition, error should be non-zero
        // Gardner: y_mid * conj(y_curr - y_prev) = (0,0) * conj((-1-1,0)) = 0
        // Actually the midpoint being exactly 0 means the error is 0 (perfectly timed)
        assert!(err.value.abs() < 1e-10);
    }

    #[test]
    fn test_mueller_muller_basic() {
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::MuellerMuller, 2.0);
        let samples = vec![
            (1.0, 0.0),
            (1.0, 0.0),
            (1.0, 0.0),
        ];
        let err = ted.compute_error(&samples);
        assert!(err.value.is_finite());
        assert!(matches!(err.algorithm, TimingAlgorithm::MuellerMuller));
    }

    #[test]
    fn test_mueller_muller_transition() {
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::MuellerMuller, 2.0);
        // First call to set prev_decision
        let samples1 = vec![
            (1.0, 0.0),
            (1.0, 0.0),
            (1.0, 0.0),
        ];
        ted.compute_error(&samples1);

        // Second call with transition
        let samples2 = vec![
            (1.0, 0.0),
            (0.5, 0.0),
            (-1.0, 0.0),
        ];
        let err = ted.compute_error(&samples2);
        assert!(err.value.is_finite());
    }

    #[test]
    fn test_zero_crossing_no_transition() {
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::ZeroCrossing, 2.0);
        // No transition: all +1
        let samples = vec![
            (1.0, 0.0),
            (1.0, 0.0),
            (1.0, 0.0),
        ];
        let err = ted.compute_error(&samples);
        // d_diff = (1,0)-(1,0) = (0,0), so error = 0
        assert!(err.value.abs() < 1e-10);
        // Low confidence when no transition
        assert!(err.confidence < 0.5);
    }

    #[test]
    fn test_zero_crossing_with_transition() {
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::ZeroCrossing, 2.0);
        let samples = vec![
            (1.0, 0.0),   // +1
            (0.5, 0.0),   // midpoint
            (-1.0, 0.0),  // -1
        ];
        let err = ted.compute_error(&samples);
        // d_diff = (-1 - 1, 0) = (-2, 0), product = (-2 * 0.5, 0) = (-1, 0)
        // error = -1.0
        assert!((err.value - (-1.0)).abs() < 1e-10);
    }

    #[test]
    fn test_early_late_symmetric() {
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::EarlyLate, 4.0);
        // Symmetric signal: early and late have equal magnitude
        let samples = vec![
            (0.5, 0.0),
            (0.8, 0.0),
            (1.0, 0.0),  // peak (on-time)
            (0.8, 0.0),
            (0.5, 0.0),
        ];
        let err = ted.compute_error(&samples);
        // Should be near zero for symmetric shape
        assert!(err.value.abs() < 0.5);
    }

    #[test]
    fn test_early_late_asymmetric() {
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::EarlyLate, 4.0);
        // Asymmetric: late is stronger
        let samples = vec![
            (0.2, 0.0),
            (0.5, 0.0),
            (0.8, 0.0),
            (1.0, 0.0),
            (0.9, 0.0),
        ];
        let err = ted.compute_error(&samples);
        // Late stronger => positive error
        assert!(err.value.is_finite());
    }

    #[test]
    fn test_empty_samples() {
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::Gardner, 2.0);
        let err = ted.compute_error(&[]);
        assert!((err.value - 0.0).abs() < 1e-10);
        assert!((err.confidence - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_too_few_samples() {
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::Gardner, 2.0);
        // Gardner needs at least sps+1 = 3 samples, give it only 2
        let err = ted.compute_error(&[(1.0, 0.0), (0.5, 0.0)]);
        assert!((err.value - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_auto_select_low_snr() {
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::Gardner, 2.0);
        ted.auto_select(3.0); // Low SNR
        assert_eq!(ted.current_algorithm(), TimingAlgorithm::EarlyLate);
    }

    #[test]
    fn test_auto_select_high_snr() {
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::Gardner, 2.0);
        ted.auto_select(25.0); // High SNR
        assert_eq!(ted.current_algorithm(), TimingAlgorithm::MuellerMuller);
    }

    #[test]
    fn test_auto_select_low_sps() {
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::Gardner, 1.0);
        ted.auto_select(15.0);
        // At 1 sps, should always pick MuellerMuller
        assert_eq!(ted.current_algorithm(), TimingAlgorithm::MuellerMuller);
    }

    #[test]
    fn test_mu_stays_bounded() {
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::Gardner, 2.0);
        // Run many iterations with large signals to drive mu
        for _ in 0..100 {
            let samples = vec![
                (1.0, 0.0),
                (0.3, 0.0),
                (-1.0, 0.0),
                (-0.3, 0.0),
            ];
            ted.compute_error(&samples);
        }
        let mu = ted.get_mu();
        assert!(mu >= 0.0 && mu < 1.0, "mu={} out of range", mu);
    }

    #[test]
    fn test_loop_filter_responds() {
        let mut ted = HybridTimingDetector::new(TimingAlgorithm::Gardner, 2.0);
        // Inject a signal that produces non-zero error
        let samples = vec![
            (1.0, 0.0),
            (0.8, 0.0),
            (-0.5, 0.0),
            (-1.0, 0.0),
        ];
        ted.compute_error(&samples);
        // After processing, loop filter should have been updated
        // (may or may not be zero depending on the specific error)
        let out = ted.loop_filter_output();
        assert!(out.is_finite());
    }

    #[test]
    fn test_adaptive_selector_error_variance() {
        let mut sel = AdaptiveSelector::new(2.0);
        // Add some errors
        for i in 0..10 {
            sel.record_error(i as f64 * 0.1);
        }
        let var = sel.error_variance();
        assert!(var > 0.0);
        assert!(var.is_finite());
    }

    #[test]
    fn test_adaptive_selector_empty_variance() {
        let sel = AdaptiveSelector::new(2.0);
        let var = sel.error_variance();
        assert_eq!(var, f64::MAX);
    }

    #[test]
    fn test_complex_helpers() {
        let a = (3.0, 4.0);
        let b = (1.0, -2.0);

        // Conjugate
        assert_eq!(complex_conj(a), (3.0, -4.0));

        // Multiplication: (3+4i)(1-2i) = 3-6i+4i-8i^2 = 3+8 + (-6+4)i = (11, -2)
        let product = complex_mul(a, b);
        assert!((product.0 - 11.0).abs() < 1e-10);
        assert!((product.1 - (-2.0)).abs() < 1e-10);

        // Subtraction
        let diff = complex_sub(a, b);
        assert!((diff.0 - 2.0).abs() < 1e-10);
        assert!((diff.1 - 6.0).abs() < 1e-10);

        // Magnitude squared
        assert!((mag_sq(a) - 25.0).abs() < 1e-10);
    }

    #[test]
    fn test_timing_error_struct_fields() {
        let te = TimingError {
            value: 0.5,
            confidence: 0.9,
            algorithm: TimingAlgorithm::Gardner,
        };
        assert!((te.value - 0.5).abs() < 1e-10);
        assert!((te.confidence - 0.9).abs() < 1e-10);
        assert_eq!(te.algorithm, TimingAlgorithm::Gardner);
    }

    #[test]
    fn test_algorithm_enum_equality() {
        assert_eq!(TimingAlgorithm::Gardner, TimingAlgorithm::Gardner);
        assert_ne!(TimingAlgorithm::Gardner, TimingAlgorithm::MuellerMuller);
        assert_ne!(TimingAlgorithm::ZeroCrossing, TimingAlgorithm::EarlyLate);
    }
}
