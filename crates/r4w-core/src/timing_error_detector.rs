//! # Timing Error Detector (TED)
//!
//! Standalone timing error detectors for symbol synchronization. Provides
//! Mueller-Muller, Gardner, and Early-Late gate algorithms that can be
//! combined with any loop filter for clock recovery.
//!
//! ## Detectors
//!
//! - **Mueller-Muller**: Uses decision-directed error (requires symbol decisions)
//! - **Gardner**: Non-data-aided, uses mid-sample interpolation
//! - **Early-Late Gate**: Correlation-based, configurable early/late offset
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::timing_error_detector::{TimingErrorDetector, TedType};
//!
//! let mut ted = TimingErrorDetector::new(TedType::Gardner, 4);
//!
//! // Feed samples (4 samples per symbol)
//! let samples = vec![(1.0, 0.0), (0.5, 0.0), (0.0, 0.0), (-0.5, 0.0)];
//! for &s in &samples {
//!     ted.push_sample(s);
//! }
//! let error = ted.compute_error();
//! ```

/// Type of timing error detector.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TedType {
    /// Mueller-Muller TED: decision-directed, optimal for PAM/PSK.
    MuellerMuller,
    /// Gardner TED: non-data-aided, works at 2 samples/symbol.
    Gardner,
    /// Early-Late Gate TED: correlation-based with configurable offset.
    EarlyLateGate,
    /// Zero-Crossing TED: detects transitions between symbols.
    ZeroCrossing,
}

/// Timing error detector for symbol synchronization.
#[derive(Debug, Clone)]
pub struct TimingErrorDetector {
    /// TED algorithm type.
    ted_type: TedType,
    /// Samples per symbol (SPS).
    sps: usize,
    /// Sample buffer.
    buffer: Vec<(f64, f64)>,
    /// Write position.
    write_pos: usize,
    /// Buffer full flag.
    buffer_full: bool,
    /// Previous symbol decision (for M&M).
    prev_decision: (f64, f64),
    /// Current symbol decision (for M&M).
    curr_decision: (f64, f64),
    /// Early-late offset (fraction of symbol period, default 0.5).
    early_late_offset: f64,
    /// Accumulated timing error.
    accumulated_error: f64,
    /// Total error computations.
    error_count: u64,
}

impl TimingErrorDetector {
    /// Create a new timing error detector.
    ///
    /// # Arguments
    /// * `ted_type` - Algorithm type
    /// * `sps` - Samples per symbol (must be ≥ 2)
    pub fn new(ted_type: TedType, sps: usize) -> Self {
        let sps = sps.max(2);
        let buf_len = sps * 3; // need at least 2 symbol periods + margins

        Self {
            ted_type,
            sps,
            buffer: vec![(0.0, 0.0); buf_len],
            write_pos: 0,
            buffer_full: false,
            prev_decision: (0.0, 0.0),
            curr_decision: (0.0, 0.0),
            early_late_offset: 0.5,
            accumulated_error: 0.0,
            error_count: 0,
        }
    }

    /// Create an Early-Late gate TED with custom offset.
    pub fn early_late(sps: usize, offset: f64) -> Self {
        let mut ted = Self::new(TedType::EarlyLateGate, sps);
        ted.early_late_offset = offset.clamp(0.1, 0.9);
        ted
    }

    /// Push a new sample into the detector.
    pub fn push_sample(&mut self, sample: (f64, f64)) {
        self.buffer[self.write_pos] = sample;
        self.write_pos += 1;
        if self.write_pos >= self.buffer.len() {
            self.write_pos = 0;
            self.buffer_full = true;
        }
    }

    /// Push multiple samples.
    pub fn push_samples(&mut self, samples: &[(f64, f64)]) {
        for &s in samples {
            self.push_sample(s);
        }
    }

    /// Compute the timing error from buffered samples.
    ///
    /// Call this once per symbol period (every `sps` samples).
    pub fn compute_error(&mut self) -> f64 {
        let error = match self.ted_type {
            TedType::MuellerMuller => self.mueller_muller_error(),
            TedType::Gardner => self.gardner_error(),
            TedType::EarlyLateGate => self.early_late_error(),
            TedType::ZeroCrossing => self.zero_crossing_error(),
        };
        self.accumulated_error += error.abs();
        self.error_count += 1;
        error
    }

    /// Provide a symbol decision for decision-directed TEDs (Mueller-Muller).
    pub fn set_decision(&mut self, decision: (f64, f64)) {
        self.prev_decision = self.curr_decision;
        self.curr_decision = decision;
    }

    fn get_sample(&self, offset_from_current: usize) -> (f64, f64) {
        let len = self.buffer.len();
        let idx = (self.write_pos + len - 1 - offset_from_current) % len;
        self.buffer[idx]
    }

    fn mueller_muller_error(&self) -> f64 {
        // M&M TED: e(n) = Re{d*(n-1)*x(n) - d*(n)*x(n-1)}
        // where d(n) is the decision and x(n) is the sample at symbol time.
        let x_curr = self.get_sample(0);
        let x_prev = self.get_sample(self.sps);

        let d_curr = self.curr_decision;
        let d_prev = self.prev_decision;

        // conj(d_prev) * x_curr
        let term1_re = d_prev.0 * x_curr.0 + d_prev.1 * x_curr.1;
        // conj(d_curr) * x_prev
        let term2_re = d_curr.0 * x_prev.0 + d_curr.1 * x_prev.1;

        term1_re - term2_re
    }

    fn gardner_error(&self) -> f64 {
        // Gardner TED: e(n) = Re{(x(n) - x(n-1)) * conj(x(n-1/2))}
        // x(n) = current symbol sample, x(n-1) = previous, x(n-1/2) = mid-point.
        let x_curr = self.get_sample(0);
        let x_prev = self.get_sample(self.sps);
        let x_mid = self.get_sample(self.sps / 2);

        let diff = (x_curr.0 - x_prev.0, x_curr.1 - x_prev.1);
        // Re{diff * conj(x_mid)}
        diff.0 * x_mid.0 + diff.1 * x_mid.1
    }

    fn early_late_error(&self) -> f64 {
        // E-L Gate: e = |x_early|^2 - |x_late|^2
        let offset_samples = (self.early_late_offset * self.sps as f64) as usize;
        let offset_samples = offset_samples.max(1).min(self.sps - 1);

        let x_early = self.get_sample(offset_samples);
        let x_late = self.get_sample(self.sps - offset_samples);

        let mag_early = x_early.0 * x_early.0 + x_early.1 * x_early.1;
        let mag_late = x_late.0 * x_late.0 + x_late.1 * x_late.1;

        mag_early - mag_late
    }

    fn zero_crossing_error(&self) -> f64 {
        // Zero-crossing: error based on sign change at mid-sample.
        let x_curr = self.get_sample(0);
        let x_prev = self.get_sample(self.sps);
        let x_mid = self.get_sample(self.sps / 2);

        // Real component only.
        let sign_change = if x_curr.0 * x_prev.0 < 0.0 {
            1.0
        } else {
            0.0
        };
        sign_change * x_mid.0
    }

    /// Get the TED type.
    pub fn ted_type(&self) -> TedType {
        self.ted_type
    }

    /// Get samples per symbol.
    pub fn sps(&self) -> usize {
        self.sps
    }

    /// Get the average absolute timing error.
    pub fn average_error(&self) -> f64 {
        if self.error_count == 0 {
            0.0
        } else {
            self.accumulated_error / self.error_count as f64
        }
    }

    /// Get the total number of error computations.
    pub fn error_count(&self) -> u64 {
        self.error_count
    }

    /// Reset the detector state.
    pub fn reset(&mut self) {
        self.buffer = vec![(0.0, 0.0); self.buffer.len()];
        self.write_pos = 0;
        self.buffer_full = false;
        self.prev_decision = (0.0, 0.0);
        self.curr_decision = (0.0, 0.0);
        self.accumulated_error = 0.0;
        self.error_count = 0;
    }
}

/// Simple first-order loop filter for use with TEDs.
#[derive(Debug, Clone)]
pub struct LoopFilter {
    /// Proportional gain.
    pub kp: f64,
    /// Integral gain.
    pub ki: f64,
    /// Integrator state.
    integrator: f64,
    /// Previous output.
    prev_output: f64,
}

impl LoopFilter {
    /// Create a loop filter from loop bandwidth and damping factor.
    ///
    /// Uses the standard second-order loop design equations:
    /// - Kp = 2 * zeta * Bn / (zeta + 1/(4*zeta))
    /// - Ki = Bn^2 / (zeta + 1/(4*zeta))^2
    pub fn from_bandwidth(loop_bw: f64, damping: f64) -> Self {
        let denom = damping + 1.0 / (4.0 * damping);
        let kp = 2.0 * damping * loop_bw / denom;
        let ki = (loop_bw / denom) * (loop_bw / denom);
        Self {
            kp,
            ki,
            integrator: 0.0,
            prev_output: 0.0,
        }
    }

    /// Create a loop filter with explicit gains.
    pub fn new(kp: f64, ki: f64) -> Self {
        Self {
            kp,
            ki,
            integrator: 0.0,
            prev_output: 0.0,
        }
    }

    /// Filter a timing error sample. Returns the clock correction.
    pub fn filter(&mut self, error: f64) -> f64 {
        self.integrator += self.ki * error;
        let output = self.kp * error + self.integrator;
        self.prev_output = output;
        output
    }

    /// Get the current integrator state.
    pub fn integrator(&self) -> f64 {
        self.integrator
    }

    /// Reset the loop filter.
    pub fn reset(&mut self) {
        self.integrator = 0.0;
        self.prev_output = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn make_bpsk_signal(sps: usize, symbols: &[f64]) -> Vec<(f64, f64)> {
        let mut samples = Vec::new();
        for &sym in symbols {
            for _ in 0..sps {
                samples.push((sym, 0.0));
            }
        }
        samples
    }

    #[test]
    fn test_gardner_on_time() {
        // When perfectly aligned, Gardner error should be near zero.
        let sps = 4;
        let symbols = [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0];
        let signal = make_bpsk_signal(sps, &symbols);

        let mut ted = TimingErrorDetector::new(TedType::Gardner, sps);
        ted.push_samples(&signal);

        // Compute error at each symbol boundary.
        let error = ted.compute_error();
        // With rectangular pulses, error magnitude is bounded by signal power.
        assert!(
            error.abs() <= 2.0,
            "Gardner error should be bounded, got {}",
            error
        );
    }

    #[test]
    fn test_mueller_muller() {
        let sps = 4;
        let symbols = [1.0, -1.0, 1.0, 1.0, -1.0];
        let signal = make_bpsk_signal(sps, &symbols);

        let mut ted = TimingErrorDetector::new(TedType::MuellerMuller, sps);
        ted.push_samples(&signal);

        // Provide decisions.
        ted.set_decision((1.0, 0.0));
        ted.set_decision((-1.0, 0.0));
        let error = ted.compute_error();
        // Just verify it returns a finite value.
        assert!(error.is_finite(), "M&M error should be finite");
    }

    #[test]
    fn test_early_late_gate() {
        let sps = 8;
        let symbols = [1.0, -1.0, 1.0, -1.0];
        let signal = make_bpsk_signal(sps, &symbols);

        let mut ted = TimingErrorDetector::early_late(sps, 0.5);
        ted.push_samples(&signal);
        let error = ted.compute_error();
        assert!(error.is_finite());
    }

    #[test]
    fn test_zero_crossing() {
        let sps = 4;
        let symbols = [1.0, -1.0, 1.0, -1.0, 1.0];
        let signal = make_bpsk_signal(sps, &symbols);

        let mut ted = TimingErrorDetector::new(TedType::ZeroCrossing, sps);
        ted.push_samples(&signal);
        let error = ted.compute_error();
        assert!(error.is_finite());
    }

    #[test]
    fn test_ted_type() {
        let ted = TimingErrorDetector::new(TedType::Gardner, 4);
        assert_eq!(ted.ted_type(), TedType::Gardner);
        assert_eq!(ted.sps(), 4);
    }

    #[test]
    fn test_reset() {
        let mut ted = TimingErrorDetector::new(TedType::Gardner, 4);
        let signal = make_bpsk_signal(4, &[1.0, -1.0, 1.0]);
        ted.push_samples(&signal);
        ted.compute_error();
        assert!(ted.error_count() > 0);
        ted.reset();
        assert_eq!(ted.error_count(), 0);
        assert!((ted.average_error() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_loop_filter_from_bandwidth() {
        let lf = LoopFilter::from_bandwidth(0.01, 1.0);
        assert!(lf.kp > 0.0);
        assert!(lf.ki > 0.0);
    }

    #[test]
    fn test_loop_filter_integration() {
        let mut lf = LoopFilter::new(0.1, 0.01);
        // Constant error should cause integrator to ramp.
        for _ in 0..100 {
            lf.filter(1.0);
        }
        assert!(lf.integrator() > 0.5, "Integrator should accumulate");
    }

    #[test]
    fn test_loop_filter_reset() {
        let mut lf = LoopFilter::new(0.1, 0.01);
        lf.filter(1.0);
        lf.filter(1.0);
        lf.reset();
        assert!((lf.integrator() - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_average_error() {
        let mut ted = TimingErrorDetector::new(TedType::Gardner, 4);
        let signal = make_bpsk_signal(4, &[1.0, -1.0, 1.0, -1.0, 1.0, -1.0]);
        ted.push_samples(&signal);
        ted.compute_error();
        ted.compute_error();
        assert_eq!(ted.error_count(), 2);
        assert!(ted.average_error() >= 0.0);
    }
}
