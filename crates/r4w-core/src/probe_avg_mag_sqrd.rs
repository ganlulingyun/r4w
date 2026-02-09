//! Probe Average Magnitude Squared — Power level measurement
//!
//! Computes a running exponential average of |x|² for power measurements,
//! AGC feedback, and squelch threshold detection. Lighter than a full
//! moving average for continuous monitoring.
//! GNU Radio equivalent: `probe_avg_mag_sqrd_cf`, `probe_avg_mag_sqrd_f`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::probe_avg_mag_sqrd::ProbeAvgMagSqrd;
//! use num_complex::Complex64;
//!
//! let mut probe = ProbeAvgMagSqrd::new(-20.0, 0.01);
//! let signal = vec![Complex64::new(0.1, 0.0); 200];
//! probe.update(&signal);
//! let level = probe.level_db();
//! assert!(level < 0.0); // Below 0 dB (unit power)
//! assert!(probe.unmuted()); // Above -20 dB threshold
//! ```

use num_complex::Complex64;

/// Probe for average magnitude squared (power level).
#[derive(Debug, Clone)]
pub struct ProbeAvgMagSqrd {
    /// Exponential averaging coefficient.
    alpha: f64,
    /// One minus alpha.
    one_minus_alpha: f64,
    /// Running average of |x|².
    avg: f64,
    /// Threshold in linear power.
    threshold: f64,
    /// Threshold in dB (for display).
    threshold_db: f64,
    /// Whether initialized.
    initialized: bool,
}

impl ProbeAvgMagSqrd {
    /// Create a probe.
    ///
    /// `threshold_db`: detection threshold in dB.
    /// `alpha`: averaging coefficient (0 < α ≤ 1). Smaller = more smoothing.
    pub fn new(threshold_db: f64, alpha: f64) -> Self {
        let alpha = alpha.clamp(1e-10, 1.0);
        Self {
            alpha,
            one_minus_alpha: 1.0 - alpha,
            avg: 0.0,
            threshold: 10f64.powf(threshold_db / 10.0),
            threshold_db,
            initialized: false,
        }
    }

    /// Create with default alpha (0.001).
    pub fn with_threshold(threshold_db: f64) -> Self {
        Self::new(threshold_db, 0.001)
    }

    /// Update with complex samples.
    pub fn update(&mut self, samples: &[Complex64]) {
        for &x in samples {
            self.update_sample_complex(x);
        }
    }

    /// Update with a single complex sample.
    #[inline]
    pub fn update_sample_complex(&mut self, x: Complex64) {
        let mag_sqrd = x.norm_sqr();
        if !self.initialized {
            self.avg = mag_sqrd;
            self.initialized = true;
        } else {
            self.avg = self.alpha * mag_sqrd + self.one_minus_alpha * self.avg;
        }
    }

    /// Update with real samples (uses x² as power).
    pub fn update_real(&mut self, samples: &[f64]) {
        for &x in samples {
            let power = x * x;
            if !self.initialized {
                self.avg = power;
                self.initialized = true;
            } else {
                self.avg = self.alpha * power + self.one_minus_alpha * self.avg;
            }
        }
    }

    /// Get current average power (linear).
    pub fn level(&self) -> f64 {
        self.avg
    }

    /// Get current average power in dB.
    pub fn level_db(&self) -> f64 {
        10.0 * self.avg.max(1e-30).log10()
    }

    /// Check if signal is above threshold (unmuted).
    pub fn unmuted(&self) -> bool {
        self.avg >= self.threshold
    }

    /// Get threshold in dB.
    pub fn threshold_db(&self) -> f64 {
        self.threshold_db
    }

    /// Set threshold in dB.
    pub fn set_threshold(&mut self, threshold_db: f64) {
        self.threshold_db = threshold_db;
        self.threshold = 10f64.powf(threshold_db / 10.0);
    }

    /// Get alpha.
    pub fn alpha(&self) -> f64 {
        self.alpha
    }

    /// Set alpha.
    pub fn set_alpha(&mut self, alpha: f64) {
        self.alpha = alpha.clamp(1e-10, 1.0);
        self.one_minus_alpha = 1.0 - self.alpha;
    }

    /// Reset the average.
    pub fn reset(&mut self) {
        self.avg = 0.0;
        self.initialized = false;
    }

    /// Get RMS level (square root of average power).
    pub fn rms(&self) -> f64 {
        self.avg.sqrt()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_constant_signal() {
        let mut probe = ProbeAvgMagSqrd::new(-40.0, 0.1);
        let signal = vec![Complex64::new(1.0, 0.0); 200];
        probe.update(&signal);
        // |1+0j|² = 1.0, should converge to 1.0
        assert!((probe.level() - 1.0).abs() < 0.01);
        assert!(probe.level_db().abs() < 0.1); // ~0 dB
    }

    #[test]
    fn test_zero_signal() {
        let mut probe = ProbeAvgMagSqrd::new(-20.0, 0.1);
        let signal = vec![Complex64::new(0.0, 0.0); 100];
        probe.update(&signal);
        assert!(probe.level() < 1e-20);
        assert!(!probe.unmuted());
    }

    #[test]
    fn test_threshold() {
        let mut probe = ProbeAvgMagSqrd::new(-10.0, 0.5);
        // Signal with power = 0.01 (-20 dB), below -10 dB threshold
        let weak = vec![Complex64::new(0.1, 0.0); 100];
        probe.update(&weak);
        assert!(!probe.unmuted());

        // Strong signal with power = 1.0 (0 dB), above -10 dB threshold
        probe.reset();
        let strong = vec![Complex64::new(1.0, 0.0); 100];
        probe.update(&strong);
        assert!(probe.unmuted());
    }

    #[test]
    fn test_set_threshold() {
        let mut probe = ProbeAvgMagSqrd::new(-20.0, 0.1);
        probe.set_threshold(-10.0);
        assert_eq!(probe.threshold_db(), -10.0);
    }

    #[test]
    fn test_real_samples() {
        let mut probe = ProbeAvgMagSqrd::new(-40.0, 0.1);
        let signal = vec![2.0; 200]; // power = 4.0
        probe.update_real(&signal);
        assert!((probe.level() - 4.0).abs() < 0.1);
    }

    #[test]
    fn test_rms() {
        let mut probe = ProbeAvgMagSqrd::new(-40.0, 0.5);
        // Unit signal: |1+0j|² = 1.0, rms = 1.0
        probe.update(&vec![Complex64::new(1.0, 0.0); 50]);
        assert!((probe.rms() - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_alpha() {
        let mut probe = ProbeAvgMagSqrd::new(-20.0, 0.01);
        assert!((probe.alpha() - 0.01).abs() < 1e-10);
        probe.set_alpha(0.1);
        assert!((probe.alpha() - 0.1).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut probe = ProbeAvgMagSqrd::new(-20.0, 0.1);
        probe.update(&vec![Complex64::new(1.0, 0.0); 50]);
        probe.reset();
        assert_eq!(probe.level(), 0.0);
    }

    #[test]
    fn test_with_threshold() {
        let probe = ProbeAvgMagSqrd::with_threshold(-30.0);
        assert_eq!(probe.threshold_db(), -30.0);
        assert!((probe.alpha() - 0.001).abs() < 1e-10);
    }

    #[test]
    fn test_complex_magnitude() {
        let mut probe = ProbeAvgMagSqrd::new(-40.0, 1.0); // alpha=1 = no smoothing
        probe.update_sample_complex(Complex64::new(3.0, 4.0));
        // |3+4j|² = 25
        assert!((probe.level() - 25.0).abs() < 1e-10);
    }

    #[test]
    fn test_convergence_rate() {
        // With alpha=0.01, should converge slower than alpha=0.5
        let mut slow = ProbeAvgMagSqrd::new(-40.0, 0.01);
        let mut fast = ProbeAvgMagSqrd::new(-40.0, 0.5);
        // Start with some zeros to initialize, then step to 1.0
        let init = vec![Complex64::new(0.0, 0.0); 5];
        slow.update(&init);
        fast.update(&init);
        let signal = vec![Complex64::new(1.0, 0.0); 10];
        slow.update(&signal);
        fast.update(&signal);
        // Fast should be closer to 1.0 after 10 samples of step
        assert!((fast.level() - 1.0).abs() < (slow.level() - 1.0).abs(),
            "fast={}, slow={}", fast.level(), slow.level());
    }
}
