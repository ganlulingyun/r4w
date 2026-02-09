//! Probe Avg Mag² — Running average power measurement
//!
//! Measures the running average power (magnitude-squared) of a complex signal
//! using exponential averaging. Provides threshold-based gating for carrier
//! sensing and spectrum monitoring. Pass-through: input samples are forwarded
//! unchanged while power is measured as a side effect.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::probe_power::ProbeAvgMagSqrd;
//! use num_complex::Complex64;
//!
//! let mut probe = ProbeAvgMagSqrd::new(0.01, -20.0);
//! let signal = vec![Complex64::new(0.5, 0.0); 100];
//! let output = probe.process(&signal);
//! assert_eq!(output.len(), 100); // Pass-through
//! let power_db = probe.level_db();
//! assert!(power_db > -10.0 && power_db < 0.0);
//! ```

use num_complex::Complex64;

/// Running average magnitude-squared probe.
#[derive(Debug, Clone)]
pub struct ProbeAvgMagSqrd {
    /// Smoothing factor for exponential averaging.
    alpha: f64,
    /// Threshold in dB for gate/unmute.
    threshold_db: f64,
    /// Current average power (linear).
    avg_power: f64,
    /// Peak power seen (linear).
    peak_power: f64,
    /// Number of samples processed.
    count: u64,
}

impl ProbeAvgMagSqrd {
    /// Create a power probe.
    ///
    /// `alpha`: Smoothing factor (0.001 = slow, 0.1 = fast).
    /// `threshold_db`: Gate threshold in dB (signal above → unmuted).
    pub fn new(alpha: f64, threshold_db: f64) -> Self {
        Self {
            alpha: alpha.clamp(0.0001, 1.0),
            threshold_db,
            avg_power: 0.0,
            peak_power: 0.0,
            count: 0,
        }
    }

    /// Process a block of samples. Pass-through: returns samples unchanged.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        for &s in input {
            let power = s.norm_sqr();
            self.avg_power = (1.0 - self.alpha) * self.avg_power + self.alpha * power;
            if power > self.peak_power {
                self.peak_power = power;
            }
            self.count += 1;
        }
        input.to_vec()
    }

    /// Process real-valued samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        for &s in input {
            let power = s * s;
            self.avg_power = (1.0 - self.alpha) * self.avg_power + self.alpha * power;
            if power > self.peak_power {
                self.peak_power = power;
            }
            self.count += 1;
        }
        input.to_vec()
    }

    /// Current average power level (linear).
    pub fn level(&self) -> f64 {
        self.avg_power
    }

    /// Current average power level in dB.
    pub fn level_db(&self) -> f64 {
        if self.avg_power > 1e-30 {
            10.0 * self.avg_power.log10()
        } else {
            -300.0
        }
    }

    /// Peak power seen (linear).
    pub fn peak(&self) -> f64 {
        self.peak_power
    }

    /// Peak power in dB.
    pub fn peak_db(&self) -> f64 {
        if self.peak_power > 1e-30 {
            10.0 * self.peak_power.log10()
        } else {
            -300.0
        }
    }

    /// Whether the signal is above the threshold (unmuted).
    pub fn unmuted(&self) -> bool {
        self.level_db() > self.threshold_db
    }

    /// Total samples processed.
    pub fn count(&self) -> u64 {
        self.count
    }

    /// Crest factor (peak-to-average power ratio) in dB.
    pub fn crest_factor_db(&self) -> f64 {
        if self.avg_power > 1e-30 {
            10.0 * (self.peak_power / self.avg_power).log10()
        } else {
            0.0
        }
    }

    pub fn reset(&mut self) {
        self.avg_power = 0.0;
        self.peak_power = 0.0;
        self.count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_constant_power() {
        let mut probe = ProbeAvgMagSqrd::new(0.1, -30.0);
        let signal = vec![Complex64::new(1.0, 0.0); 100];
        let output = probe.process(&signal);
        assert_eq!(output.len(), 100);
        // Power should converge to 1.0 (|1+0j|² = 1.0)
        assert!((probe.level() - 1.0).abs() < 0.1, "Power should be ~1.0, got {}", probe.level());
    }

    #[test]
    fn test_level_db() {
        let mut probe = ProbeAvgMagSqrd::new(0.5, -30.0);
        let signal = vec![Complex64::new(0.1, 0.0); 50];
        probe.process(&signal);
        // |0.1|² = 0.01, 10*log10(0.01) = -20 dB
        let db = probe.level_db();
        assert!(db > -25.0 && db < -15.0, "Expected ~-20 dB, got {}", db);
    }

    #[test]
    fn test_unmuted() {
        let mut probe = ProbeAvgMagSqrd::new(0.5, -10.0);
        // Strong signal
        probe.process(&vec![Complex64::new(1.0, 0.0); 20]);
        assert!(probe.unmuted(), "Strong signal should be above threshold");

        // Weak signal
        let mut probe2 = ProbeAvgMagSqrd::new(0.5, -10.0);
        probe2.process(&vec![Complex64::new(0.01, 0.0); 20]);
        assert!(!probe2.unmuted(), "Weak signal should be below threshold");
    }

    #[test]
    fn test_peak_tracking() {
        let mut probe = ProbeAvgMagSqrd::new(0.01, -30.0);
        let mut signal = vec![Complex64::new(0.1, 0.0); 100];
        signal[50] = Complex64::new(5.0, 0.0);
        probe.process(&signal);
        assert!((probe.peak() - 25.0).abs() < 0.1, "Peak should be 25.0 (|5|²)");
    }

    #[test]
    fn test_crest_factor() {
        let mut probe = ProbeAvgMagSqrd::new(0.1, -30.0);
        let signal = vec![Complex64::new(1.0, 0.0); 100];
        probe.process(&signal);
        // Constant signal: crest factor ≈ 0 dB
        assert!(probe.crest_factor_db().abs() < 1.0);
    }

    #[test]
    fn test_silence() {
        let mut probe = ProbeAvgMagSqrd::new(0.1, -30.0);
        let signal = vec![Complex64::new(0.0, 0.0); 100];
        probe.process(&signal);
        assert!(probe.level_db() < -200.0);
        assert!(!probe.unmuted());
    }

    #[test]
    fn test_count() {
        let mut probe = ProbeAvgMagSqrd::new(0.1, -30.0);
        probe.process(&vec![Complex64::new(1.0, 0.0); 50]);
        assert_eq!(probe.count(), 50);
        probe.process(&vec![Complex64::new(1.0, 0.0); 30]);
        assert_eq!(probe.count(), 80);
    }

    #[test]
    fn test_reset() {
        let mut probe = ProbeAvgMagSqrd::new(0.1, -30.0);
        probe.process(&vec![Complex64::new(1.0, 0.0); 50]);
        probe.reset();
        assert_eq!(probe.level(), 0.0);
        assert_eq!(probe.count(), 0);
    }

    #[test]
    fn test_pass_through() {
        let mut probe = ProbeAvgMagSqrd::new(0.01, -30.0);
        let signal: Vec<Complex64> = (0..10)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let output = probe.process(&signal);
        // Exact pass-through
        for (s, o) in signal.iter().zip(output.iter()) {
            assert_eq!(s, o);
        }
    }

    #[test]
    fn test_real_processing() {
        let mut probe = ProbeAvgMagSqrd::new(0.5, -30.0);
        let signal = vec![1.0_f64; 20];
        let output = probe.process_real(&signal);
        assert_eq!(output.len(), 20);
        assert!((probe.level() - 1.0).abs() < 0.1);
    }
}
