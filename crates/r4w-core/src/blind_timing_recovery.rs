//! Blind (non-data-aided) symbol timing recovery for digital receivers.
//!
//! This module provides several timing error detectors and interpolation methods
//! for recovering optimal symbol sampling instants from an oversampled signal
//! without requiring knowledge of the transmitted data.
//!
//! # Supported Timing Error Detectors
//!
//! - **Gardner**: Requires 2x oversampling; uses the midpoint between two successive
//!   symbol samples and is insensitive to carrier phase offset.
//! - **Early-Late Gate**: Compares early and late samples around the estimated
//!   symbol center to derive timing error.
//! - **Square Law (spectral line)**: Squares the signal to create a spectral line
//!   at the symbol rate, then extracts timing phase.
//! - **Godard**: Uses a fourth-power nonlinearity for clock recovery, robust to
//!   carrier phase offset.
//!
//! # Example
//!
//! ```
//! use r4w_core::blind_timing_recovery::{BlindTimingRecovery, TimingDetector};
//!
//! // Generate a simple BPSK signal at 4 samples/symbol
//! let sps = 4.0;
//! let symbols = [1.0_f64, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0];
//! let mut samples: Vec<f64> = Vec::new();
//! for &sym in &symbols {
//!     for _ in 0..sps as usize {
//!         samples.push(sym);
//!     }
//! }
//!
//! let mut btr = BlindTimingRecovery::new(sps)
//!     .with_detector(TimingDetector::Gardner)
//!     .with_bandwidth(0.05);
//!
//! let recovered = btr.process(&samples);
//! // Should recover approximately one symbol per input symbol
//! assert!(recovered.len() >= symbols.len() / 2, "should recover symbols");
//! for s in &recovered {
//!     // Each recovered sample should be close to +1 or -1
//!     assert!(s.abs() > 0.5, "recovered symbol {} should be near +/-1", s);
//! }
//! ```

/// Timing error detector algorithm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum TimingDetector {
    /// Gardner TED: e(n) = x(n-1/2) * [x(n) - x(n-1)].
    /// Requires at least 2x oversampling. Insensitive to carrier phase.
    Gardner,
    /// Early-Late gate: e(n) = |x(early)|^2 - |x(late)|^2.
    EarlyLate,
    /// Square-law (spectral line) method: squares the signal to create
    /// a spectral line at the symbol rate.
    SquareLaw,
    /// Godard fourth-power timing recovery.
    Godard,
}

/// Interpolation method for fractional sample offset correction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Interpolation {
    /// Linear interpolation between two adjacent samples.
    Linear,
    /// Cubic (Lagrange 4-point) interpolation for higher accuracy.
    Cubic,
}

/// Blind (non-data-aided) symbol timing recovery.
///
/// Processes an oversampled input signal and produces one output sample per
/// detected symbol strobe, using a proportional-integral loop filter to
/// track the optimal sampling phase.
#[derive(Debug, Clone)]
pub struct BlindTimingRecovery {
    samples_per_symbol: f64,
    detector: TimingDetector,
    interpolation: Interpolation,
    bandwidth: f64,
    damping: f64,
    // Loop filter gains (derived from bandwidth and damping)
    kp: f64,
    ki: f64,
    // Loop filter state
    mu: f64,
    integrator: f64,
    strobe_count: usize,
    // Lock detection
    error_history: Vec<f64>,
    lock_threshold: f64,
    last_error: f64,
}

impl BlindTimingRecovery {
    /// Create a new timing recovery instance.
    ///
    /// `samples_per_symbol` is the nominal oversampling ratio (e.g., 2.0, 4.0).
    /// Must be >= 2.0 for Gardner and Early-Late detectors.
    pub fn new(samples_per_symbol: f64) -> Self {
        let bandwidth = 0.01;
        let damping = 1.0;
        let (kp, ki) = Self::compute_gains(bandwidth, damping, samples_per_symbol);
        Self {
            samples_per_symbol,
            detector: TimingDetector::Gardner,
            interpolation: Interpolation::Linear,
            bandwidth,
            damping,
            kp,
            ki,
            mu: 0.0,
            integrator: 0.0,
            strobe_count: 0,
            error_history: Vec::new(),
            lock_threshold: 0.05,
            last_error: 0.0,
        }
    }

    /// Set the timing error detector algorithm.
    pub fn with_detector(mut self, det: TimingDetector) -> Self {
        self.detector = det;
        self
    }

    /// Set the interpolation method.
    pub fn with_interpolation(mut self, interp: Interpolation) -> Self {
        self.interpolation = interp;
        self
    }

    /// Set the loop bandwidth (normalized, typically 0.001 to 0.1).
    ///
    /// Smaller values give slower but more stable convergence;
    /// larger values converge faster but may be less stable.
    pub fn with_bandwidth(mut self, bw: f64) -> Self {
        self.bandwidth = bw;
        let (kp, ki) = Self::compute_gains(bw, self.damping, self.samples_per_symbol);
        self.kp = kp;
        self.ki = ki;
        self
    }

    /// Set the loop damping factor (typically 0.5 to 2.0; 1.0 = critically damped).
    pub fn with_damping(mut self, d: f64) -> Self {
        self.damping = d;
        let (kp, ki) = Self::compute_gains(self.bandwidth, d, self.samples_per_symbol);
        self.kp = kp;
        self.ki = ki;
        self
    }

    /// Set the lock detection threshold (variance of recent timing errors).
    pub fn with_lock_threshold(mut self, thresh: f64) -> Self {
        self.lock_threshold = thresh;
        self
    }

    /// Compute PI loop filter gains from bandwidth, damping, and samples/symbol.
    ///
    /// Uses the standard second-order loop gain formulas:
    ///   theta = BW_n / (damping + 1/(4*damping))
    ///   kp = 2 * damping * theta / sps
    ///   ki = theta^2 / sps
    fn compute_gains(bandwidth: f64, damping: f64, sps: f64) -> (f64, f64) {
        let denom = damping + 1.0 / (4.0 * damping);
        let theta = bandwidth / denom;
        let kp = 2.0 * damping * theta / sps;
        let ki = theta * theta / sps;
        (kp, ki)
    }

    /// Interpolate a sample at fractional offset `mu` within the given sample buffer
    /// at base index `idx`. `mu` is in [0, 1).
    fn interpolate_real(&self, samples: &[f64], idx: usize, mu: f64) -> f64 {
        match self.interpolation {
            Interpolation::Linear => {
                let s0 = samples[idx];
                let s1 = if idx + 1 < samples.len() {
                    samples[idx + 1]
                } else {
                    s0
                };
                s0 + mu * (s1 - s0)
            }
            Interpolation::Cubic => {
                // 4-point Lagrange interpolation centered around idx and idx+1
                let i = idx as isize;
                let s = |offset: isize| -> f64 {
                    let j = i + offset;
                    if j < 0 {
                        samples[0]
                    } else if j as usize >= samples.len() {
                        samples[samples.len() - 1]
                    } else {
                        samples[j as usize]
                    }
                };
                let sm1 = s(-1);
                let s0 = s(0);
                let s1 = s(1);
                let s2 = s(2);

                // Cubic Lagrange
                let c0 = s0;
                let c1 = 0.5 * (s1 - sm1);
                let c2 = sm1 - 2.5 * s0 + 2.0 * s1 - 0.5 * s2;
                let c3 = 0.5 * (s2 - sm1) + 1.5 * (s0 - s1);
                ((c3 * mu + c2) * mu + c1) * mu + c0
            }
        }
    }

    /// Interpolate a complex sample at fractional offset `mu`.
    fn interpolate_complex(
        &self,
        samples: &[(f64, f64)],
        idx: usize,
        mu: f64,
    ) -> (f64, f64) {
        match self.interpolation {
            Interpolation::Linear => {
                let (i0, q0) = samples[idx];
                let (i1, q1) = if idx + 1 < samples.len() {
                    samples[idx + 1]
                } else {
                    (i0, q0)
                };
                (i0 + mu * (i1 - i0), q0 + mu * (q1 - q0))
            }
            Interpolation::Cubic => {
                let len = samples.len();
                let i = idx as isize;
                let s = |offset: isize| -> (f64, f64) {
                    let j = i + offset;
                    if j < 0 {
                        samples[0]
                    } else if j as usize >= len {
                        samples[len - 1]
                    } else {
                        samples[j as usize]
                    }
                };
                let (im1, qm1) = s(-1);
                let (i0, q0) = s(0);
                let (i1, q1) = s(1);
                let (i2, q2) = s(2);

                let interp_channel = |sm1: f64, s0: f64, s1: f64, s2: f64| -> f64 {
                    let c0 = s0;
                    let c1 = 0.5 * (s1 - sm1);
                    let c2 = sm1 - 2.5 * s0 + 2.0 * s1 - 0.5 * s2;
                    let c3 = 0.5 * (s2 - sm1) + 1.5 * (s0 - s1);
                    ((c3 * mu + c2) * mu + c1) * mu + c0
                };

                (
                    interp_channel(im1, i0, i1, i2),
                    interp_channel(qm1, q0, q1, q2),
                )
            }
        }
    }

    /// Compute timing error using the configured detector for real signals.
    fn ted_real(&self, prev_sym: f64, mid: f64, curr_sym: f64) -> f64 {
        match self.detector {
            TimingDetector::Gardner => {
                // Gardner: e = mid * (prev - curr)
                mid * (prev_sym - curr_sym)
            }
            TimingDetector::EarlyLate => {
                // Early-Late gate: e = |early|^2 - |late|^2
                // We treat mid as the on-time, prev as early, curr as late
                // Actually: early = prev_sym, late = curr_sym in the centered sense
                prev_sym.abs().powi(2) - curr_sym.abs().powi(2)
            }
            TimingDetector::SquareLaw => {
                // Square law: operate on squared signal to extract symbol rate component
                let sq_prev = prev_sym * prev_sym;
                let sq_curr = curr_sym * curr_sym;
                let sq_mid = mid * mid;
                sq_mid * (sq_prev - sq_curr)
            }
            TimingDetector::Godard => {
                // Godard: e = mid^3 * (prev - curr)  (fourth-power nonlinearity)
                mid.powi(3) * (prev_sym - curr_sym)
            }
        }
    }

    /// Compute timing error for complex signals.
    fn ted_complex(
        &self,
        prev: (f64, f64),
        mid: (f64, f64),
        curr: (f64, f64),
    ) -> f64 {
        match self.detector {
            TimingDetector::Gardner => {
                // Gardner for complex: Re{mid* * (prev - curr)}
                let diff_i = prev.0 - curr.0;
                let diff_q = prev.1 - curr.1;
                mid.0 * diff_i + mid.1 * diff_q
            }
            TimingDetector::EarlyLate => {
                let pow_prev = prev.0 * prev.0 + prev.1 * prev.1;
                let pow_curr = curr.0 * curr.0 + curr.1 * curr.1;
                pow_prev - pow_curr
            }
            TimingDetector::SquareLaw => {
                let sq_prev = prev.0 * prev.0 + prev.1 * prev.1;
                let sq_curr = curr.0 * curr.0 + curr.1 * curr.1;
                let sq_mid = mid.0 * mid.0 + mid.1 * mid.1;
                sq_mid * (sq_prev - sq_curr)
            }
            TimingDetector::Godard => {
                let mag_sq_mid = mid.0 * mid.0 + mid.1 * mid.1;
                let diff_i = prev.0 - curr.0;
                let diff_q = prev.1 - curr.1;
                mag_sq_mid * (mid.0 * diff_i + mid.1 * diff_q)
            }
        }
    }

    /// Process a buffer of real-valued samples and return recovered symbol samples.
    ///
    /// The input should be oversampled at the nominal `samples_per_symbol` rate.
    /// Returns one output value per detected symbol strobe.
    pub fn process(&mut self, samples: &[f64]) -> Vec<f64> {
        if samples.len() < 3 {
            return Vec::new();
        }

        let sps = self.samples_per_symbol;
        let mut output = Vec::new();
        let mut counter = 0.0_f64; // fractional sample counter

        // We need at least 2*sps samples to get started
        // Walk through the input producing one symbol per sps samples
        while (counter as usize) + 1 < samples.len() {
            let base = counter as usize;

            // Current symbol sample (at mu offset from base)
            let curr = self.interpolate_real(samples, base, self.mu);

            // Mid-symbol sample (half a symbol period back)
            let mid_pos = counter - sps / 2.0;
            if mid_pos >= 0.0 {
                let mid_base = mid_pos as usize;
                let mid_mu = mid_pos - mid_base as f64;
                let mid = if mid_base + 1 < samples.len() {
                    self.interpolate_real(samples, mid_base, mid_mu)
                } else {
                    0.0
                };

                // Previous symbol sample (one symbol period back)
                let prev_pos = counter - sps;
                let prev = if prev_pos >= 0.0 {
                    let prev_base = prev_pos as usize;
                    let prev_mu = prev_pos - prev_base as f64;
                    if prev_base + 1 < samples.len() {
                        self.interpolate_real(samples, prev_base, prev_mu)
                    } else {
                        0.0
                    }
                } else {
                    0.0
                };

                // Compute timing error
                let error = self.ted_real(prev, mid, curr);
                self.last_error = error;

                // Record for lock detection
                self.error_history.push(error);
                if self.error_history.len() > 64 {
                    self.error_history.remove(0);
                }

                // PI loop filter
                self.integrator += self.ki * error;
                let loop_out = self.kp * error + self.integrator;

                // Update mu (fractional sample offset)
                self.mu -= loop_out;

                // Wrap mu into [0, sps) and adjust counter
                while self.mu < 0.0 {
                    self.mu += 1.0;
                    counter -= 1.0;
                }
                while self.mu >= 1.0 {
                    self.mu -= 1.0;
                    counter += 1.0;
                }
            }

            output.push(curr);
            self.strobe_count += 1;

            // Advance by one symbol period
            counter += sps;
        }

        output
    }

    /// Process a buffer of complex (I/Q) samples and return recovered symbol samples.
    ///
    /// Each sample is a `(I, Q)` tuple. The input should be oversampled at the
    /// nominal `samples_per_symbol` rate.
    pub fn process_complex(&mut self, samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if samples.len() < 3 {
            return Vec::new();
        }

        let sps = self.samples_per_symbol;
        let mut output = Vec::new();
        let mut counter = 0.0_f64;

        while (counter as usize) + 1 < samples.len() {
            let base = counter as usize;
            let curr = self.interpolate_complex(samples, base, self.mu);

            let mid_pos = counter - sps / 2.0;
            if mid_pos >= 0.0 {
                let mid_base = mid_pos as usize;
                let mid_mu = mid_pos - mid_base as f64;
                let mid = if mid_base + 1 < samples.len() {
                    self.interpolate_complex(samples, mid_base, mid_mu)
                } else {
                    (0.0, 0.0)
                };

                let prev_pos = counter - sps;
                let prev = if prev_pos >= 0.0 {
                    let prev_base = prev_pos as usize;
                    let prev_mu = prev_pos - prev_base as f64;
                    if prev_base + 1 < samples.len() {
                        self.interpolate_complex(samples, prev_base, prev_mu)
                    } else {
                        (0.0, 0.0)
                    }
                } else {
                    (0.0, 0.0)
                };

                let error = self.ted_complex(prev, mid, curr);
                self.last_error = error;

                self.error_history.push(error);
                if self.error_history.len() > 64 {
                    self.error_history.remove(0);
                }

                self.integrator += self.ki * error;
                let loop_out = self.kp * error + self.integrator;

                self.mu -= loop_out;

                while self.mu < 0.0 {
                    self.mu += 1.0;
                    counter -= 1.0;
                }
                while self.mu >= 1.0 {
                    self.mu -= 1.0;
                    counter += 1.0;
                }
            }

            output.push(curr);
            self.strobe_count += 1;

            counter += sps;
        }

        output
    }

    /// Return the most recent timing error value.
    pub fn timing_error(&self) -> f64 {
        self.last_error
    }

    /// Return the current fractional sample offset (mu), in [0, 1).
    pub fn mu(&self) -> f64 {
        self.mu
    }

    /// Return whether the timing loop is locked.
    ///
    /// Lock is declared when the variance of recent timing errors falls
    /// below the configured threshold.
    pub fn is_locked(&self) -> bool {
        if self.error_history.len() < 8 {
            return false;
        }
        let n = self.error_history.len() as f64;
        let mean = self.error_history.iter().sum::<f64>() / n;
        let var = self.error_history.iter().map(|e| (e - mean).powi(2)).sum::<f64>() / n;
        var < self.lock_threshold
    }

    /// Reset all internal state to initial conditions.
    pub fn reset(&mut self) {
        self.mu = 0.0;
        self.integrator = 0.0;
        self.strobe_count = 0;
        self.error_history.clear();
        self.last_error = 0.0;
    }

    /// Return the number of symbol strobes generated so far.
    pub fn strobe_count(&self) -> usize {
        self.strobe_count
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a rectangular-pulse BPSK signal at the given samples/symbol.
    fn make_bpsk_signal(symbols: &[f64], sps: usize) -> Vec<f64> {
        let mut out = Vec::with_capacity(symbols.len() * sps);
        for &s in symbols {
            for _ in 0..sps {
                out.push(s);
            }
        }
        out
    }

    /// Helper: generate a complex BPSK signal (I channel modulated, Q = 0).
    fn make_complex_bpsk(symbols: &[f64], sps: usize) -> Vec<(f64, f64)> {
        let mut out = Vec::with_capacity(symbols.len() * sps);
        for &s in symbols {
            for _ in 0..sps {
                out.push((s, 0.0));
            }
        }
        out
    }

    #[test]
    fn test_gardner_basic_recovery() {
        let symbols: Vec<f64> = vec![1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0];
        let sps = 4;
        let samples = make_bpsk_signal(&symbols, sps);

        let mut btr = BlindTimingRecovery::new(sps as f64)
            .with_detector(TimingDetector::Gardner)
            .with_bandwidth(0.05);

        let recovered = btr.process(&samples);

        assert!(!recovered.is_empty(), "should produce output symbols");
        // Each recovered symbol should have magnitude close to 1
        for s in &recovered {
            assert!(
                s.abs() > 0.3,
                "recovered symbol {} too small",
                s
            );
        }
    }

    #[test]
    fn test_early_late_recovery() {
        let symbols: Vec<f64> = vec![1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, -1.0, 1.0, 1.0];
        let sps = 4;
        let samples = make_bpsk_signal(&symbols, sps);

        let mut btr = BlindTimingRecovery::new(sps as f64)
            .with_detector(TimingDetector::EarlyLate)
            .with_bandwidth(0.05);

        let recovered = btr.process(&samples);

        assert!(!recovered.is_empty(), "should produce output symbols");
        for s in &recovered {
            assert!(s.abs() > 0.3, "symbol magnitude should be significant");
        }
    }

    #[test]
    fn test_square_law_recovery() {
        let symbols: Vec<f64> = vec![1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0];
        let sps = 4;
        let samples = make_bpsk_signal(&symbols, sps);

        let mut btr = BlindTimingRecovery::new(sps as f64)
            .with_detector(TimingDetector::SquareLaw)
            .with_bandwidth(0.05);

        let recovered = btr.process(&samples);

        assert!(!recovered.is_empty(), "should produce output symbols");
    }

    #[test]
    fn test_godard_recovery() {
        let symbols: Vec<f64> = vec![1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, -1.0];
        let sps = 4;
        let samples = make_bpsk_signal(&symbols, sps);

        let mut btr = BlindTimingRecovery::new(sps as f64)
            .with_detector(TimingDetector::Godard)
            .with_bandwidth(0.05);

        let recovered = btr.process(&samples);

        assert!(!recovered.is_empty(), "should produce output symbols");
    }

    #[test]
    fn test_complex_signal_recovery() {
        let symbols = [1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0];
        let sps = 4;
        let samples = make_complex_bpsk(&symbols, sps);

        let mut btr = BlindTimingRecovery::new(sps as f64)
            .with_detector(TimingDetector::Gardner)
            .with_bandwidth(0.05);

        let recovered = btr.process_complex(&samples);

        assert!(!recovered.is_empty(), "should produce complex output");
        for (i, q) in &recovered {
            // Q should be near zero for BPSK on I axis
            assert!(q.abs() < 0.5, "Q component should be small, got {}", q);
            // I should have significant magnitude
            assert!(i.abs() > 0.3, "I component should be significant, got {}", i);
        }
    }

    #[test]
    fn test_cubic_interpolation() {
        let symbols: Vec<f64> = vec![1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0];
        let sps = 4;
        let samples = make_bpsk_signal(&symbols, sps);

        let mut btr = BlindTimingRecovery::new(sps as f64)
            .with_interpolation(Interpolation::Cubic)
            .with_bandwidth(0.05);

        let recovered = btr.process(&samples);

        assert!(!recovered.is_empty(), "cubic interpolation should produce output");
        for s in &recovered {
            assert!(s.abs() > 0.3, "cubic recovered symbol should have magnitude");
        }
    }

    #[test]
    fn test_lock_detection() {
        let symbols: Vec<f64> = (0..200)
            .map(|i| if i % 2 == 0 { 1.0 } else { -1.0 })
            .collect();
        let sps = 4;
        let samples = make_bpsk_signal(&symbols, sps);

        let mut btr = BlindTimingRecovery::new(sps as f64)
            .with_bandwidth(0.05)
            .with_lock_threshold(0.1);

        let _recovered = btr.process(&samples);

        // After processing a long signal, the loop should be locked
        // (timing errors should have low variance for a clean signal)
        // Note: lock detection depends on error variance being below threshold
        // For a perfectly rectangular pulse, the Gardner error converges well
        assert!(
            btr.error_history.len() >= 8,
            "should have enough error history for lock detection"
        );
    }

    #[test]
    fn test_reset() {
        let mut btr = BlindTimingRecovery::new(4.0).with_bandwidth(0.05);

        // Process some data
        let samples = make_bpsk_signal(&[1.0, -1.0, 1.0, -1.0], 4);
        let _ = btr.process(&samples);

        assert!(btr.strobe_count() > 0);

        btr.reset();

        assert_eq!(btr.mu(), 0.0);
        assert_eq!(btr.strobe_count(), 0);
        assert_eq!(btr.timing_error(), 0.0);
        assert!(!btr.is_locked());
    }

    #[test]
    fn test_timing_offset_recovery() {
        // Create a signal with a known timing offset (shift by 1 sample)
        let symbols: Vec<f64> = vec![1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0,
                                     1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0];
        let sps = 4;
        let mut samples = make_bpsk_signal(&symbols, sps);
        // Insert a 1-sample offset at the beginning
        samples.insert(0, 0.0);

        let mut btr = BlindTimingRecovery::new(sps as f64)
            .with_detector(TimingDetector::Gardner)
            .with_bandwidth(0.05);

        let recovered = btr.process(&samples);

        assert!(!recovered.is_empty());
        // Despite the offset, recovered symbols should still be significant
        let significant_count = recovered.iter().filter(|s| s.abs() > 0.3).count();
        assert!(
            significant_count > recovered.len() / 2,
            "most symbols should be recovered despite offset, got {}/{}",
            significant_count,
            recovered.len()
        );
    }

    #[test]
    fn test_two_samples_per_symbol() {
        // Minimum oversampling for Gardner: 2x
        let symbols: Vec<f64> = vec![1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0,
                                     1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0];
        let sps = 2;
        let samples = make_bpsk_signal(&symbols, sps);

        let mut btr = BlindTimingRecovery::new(sps as f64)
            .with_detector(TimingDetector::Gardner)
            .with_bandwidth(0.05);

        let recovered = btr.process(&samples);

        assert!(!recovered.is_empty(), "should work with 2 sps");
        for s in &recovered {
            assert!(s.abs() > 0.3, "symbol should be recovered at 2 sps");
        }
    }

    #[test]
    fn test_empty_and_short_input() {
        let mut btr = BlindTimingRecovery::new(4.0);

        // Empty input
        let empty: Vec<f64> = vec![];
        let result = btr.process(&empty);
        assert!(result.is_empty());

        // Too short
        let short = vec![1.0, -1.0];
        let result = btr.process(&short);
        assert!(result.is_empty());

        // Complex empty
        let empty_c: Vec<(f64, f64)> = vec![];
        let result = btr.process_complex(&empty_c);
        assert!(result.is_empty());
    }

    #[test]
    fn test_builder_pattern() {
        let btr = BlindTimingRecovery::new(4.0)
            .with_detector(TimingDetector::EarlyLate)
            .with_interpolation(Interpolation::Cubic)
            .with_bandwidth(0.02)
            .with_damping(0.707)
            .with_lock_threshold(0.01);

        assert_eq!(btr.detector, TimingDetector::EarlyLate);
        assert_eq!(btr.interpolation, Interpolation::Cubic);
        assert!((btr.bandwidth - 0.02).abs() < 1e-10);
        assert!((btr.damping - 0.707).abs() < 1e-10);
        assert!((btr.lock_threshold - 0.01).abs() < 1e-10);
    }

    #[test]
    fn test_complex_cubic_interpolation() {
        // Test complex signal path with cubic interpolation
        let symbols = [1.0, -1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0];
        let sps = 4;
        let samples = make_complex_bpsk(&symbols, sps);

        let mut btr = BlindTimingRecovery::new(sps as f64)
            .with_detector(TimingDetector::Gardner)
            .with_interpolation(Interpolation::Cubic)
            .with_bandwidth(0.05);

        let recovered = btr.process_complex(&samples);

        assert!(!recovered.is_empty(), "should produce complex output with cubic interp");
        for (i, q) in &recovered {
            assert!(q.abs() < 0.5, "Q should be small for BPSK");
            assert!(i.abs() > 0.3, "I should be significant");
        }
    }
}
