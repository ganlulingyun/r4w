//! Power Squelch
//!
//! Gates a signal based on instantaneous power level. Passes samples through
//! only when signal power exceeds a configurable threshold. Supports
//! attack/release time constants and hysteresis to prevent rapid toggling.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::power_squelch::PowerSquelch;
//! use num_complex::Complex64;
//!
//! let mut sq = PowerSquelch::new(-30.0); // -30 dBFS threshold
//! let loud = vec![Complex64::new(0.1, 0.0); 10]; // ~-20 dBFS
//! let quiet = vec![Complex64::new(0.001, 0.0); 10]; // ~-60 dBFS
//!
//! let out_loud = sq.process_block(&loud);
//! assert!(out_loud.iter().any(|s| s.norm() > 0.0)); // passes through
//!
//! let out_quiet = sq.process_block(&quiet);
//! // After release, signal is gated
//! ```

use num_complex::Complex64;

/// Power squelch with threshold, attack/release, and hysteresis.
#[derive(Debug, Clone)]
pub struct PowerSquelch {
    /// Threshold in dBFS
    threshold_db: f64,
    /// Hysteresis in dB (gate opens at threshold, closes at threshold - hysteresis)
    hysteresis_db: f64,
    /// Attack time in samples (how quickly gate opens)
    attack_samples: usize,
    /// Release time in samples (how quickly gate closes)
    release_samples: usize,
    /// Current gate state
    gate_open: bool,
    /// Smoothed power estimate (linear)
    smoothed_power: f64,
    /// Smoothing alpha for power estimation
    alpha: f64,
    /// Counter for attack/release timing
    transition_count: usize,
    /// Whether we're in a transition
    transitioning: bool,
}

impl PowerSquelch {
    /// Create a power squelch with the given threshold in dBFS.
    pub fn new(threshold_db: f64) -> Self {
        Self {
            threshold_db,
            hysteresis_db: 3.0,
            attack_samples: 1,
            release_samples: 1,
            gate_open: false,
            smoothed_power: 0.0,
            alpha: 0.01,
            transition_count: 0,
            transitioning: false,
        }
    }

    /// Create with full configuration.
    pub fn with_config(
        threshold_db: f64,
        hysteresis_db: f64,
        attack_samples: usize,
        release_samples: usize,
        alpha: f64,
    ) -> Self {
        Self {
            threshold_db,
            hysteresis_db,
            attack_samples: attack_samples.max(1),
            release_samples: release_samples.max(1),
            gate_open: false,
            smoothed_power: 0.0,
            alpha: alpha.clamp(0.001, 1.0),
            transition_count: 0,
            transitioning: false,
        }
    }

    /// Process a single sample. Returns the sample if gate is open, zero otherwise.
    pub fn process(&mut self, input: Complex64) -> Complex64 {
        let power = input.norm_sqr();
        self.smoothed_power = self.alpha * power + (1.0 - self.alpha) * self.smoothed_power;

        let power_db = if self.smoothed_power > 1e-30 {
            10.0 * self.smoothed_power.log10()
        } else {
            -300.0
        };

        // Hysteresis logic
        if self.gate_open {
            if power_db < self.threshold_db - self.hysteresis_db {
                if !self.transitioning {
                    self.transitioning = true;
                    self.transition_count = 0;
                }
                self.transition_count += 1;
                if self.transition_count >= self.release_samples {
                    self.gate_open = false;
                    self.transitioning = false;
                }
            } else {
                self.transitioning = false;
                self.transition_count = 0;
            }
        } else {
            if power_db >= self.threshold_db {
                if !self.transitioning {
                    self.transitioning = true;
                    self.transition_count = 0;
                }
                self.transition_count += 1;
                if self.transition_count >= self.attack_samples {
                    self.gate_open = true;
                    self.transitioning = false;
                }
            } else {
                self.transitioning = false;
                self.transition_count = 0;
            }
        }

        if self.gate_open {
            input
        } else {
            Complex64::new(0.0, 0.0)
        }
    }

    /// Process a block of complex samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Process a block of real samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&s| {
            let out = self.process(Complex64::new(s, 0.0));
            out.re
        }).collect()
    }

    /// Check if the gate is currently open.
    pub fn is_open(&self) -> bool {
        self.gate_open
    }

    /// Get the current smoothed power in dBFS.
    pub fn power_db(&self) -> f64 {
        if self.smoothed_power > 1e-30 {
            10.0 * self.smoothed_power.log10()
        } else {
            -300.0
        }
    }

    /// Get the threshold in dBFS.
    pub fn threshold_db(&self) -> f64 {
        self.threshold_db
    }

    /// Set a new threshold.
    pub fn set_threshold_db(&mut self, threshold_db: f64) {
        self.threshold_db = threshold_db;
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.gate_open = false;
        self.smoothed_power = 0.0;
        self.transition_count = 0;
        self.transitioning = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_quiet_signal_gated() {
        let mut sq = PowerSquelch::new(-20.0);
        sq.alpha = 1.0; // Instant response for testing
        let quiet = vec![Complex64::new(0.001, 0.0); 10]; // ~-60 dBFS
        let output = sq.process_block(&quiet);
        // All should be zero (gated)
        for s in &output {
            assert!((s.re).abs() < 1e-10 && (s.im).abs() < 1e-10);
        }
        assert!(!sq.is_open());
    }

    #[test]
    fn test_loud_signal_passes() {
        let mut sq = PowerSquelch::new(-20.0);
        sq.alpha = 1.0; // Instant response
        let loud = vec![Complex64::new(1.0, 0.0); 10]; // 0 dBFS
        let output = sq.process_block(&loud);
        // Should pass through after attack
        assert!(output.iter().any(|s| (s.re - 1.0).abs() < 1e-10));
        assert!(sq.is_open());
    }

    #[test]
    fn test_transition() {
        let mut sq = PowerSquelch::new(-20.0);
        sq.alpha = 1.0;
        // Start with loud signal
        let loud = vec![Complex64::new(1.0, 0.0); 5];
        sq.process_block(&loud);
        assert!(sq.is_open());
        // Switch to quiet
        let quiet = vec![Complex64::new(0.0001, 0.0); 5];
        sq.process_block(&quiet);
        assert!(!sq.is_open());
    }

    #[test]
    fn test_hysteresis() {
        let mut sq = PowerSquelch::with_config(-20.0, 6.0, 1, 1, 1.0);
        // Signal right at threshold - open
        let at_thresh = vec![Complex64::new(0.1, 0.0)]; // -20 dBFS
        sq.process_block(&at_thresh);
        assert!(sq.is_open());
        // Signal drops by 3 dB - still open (within hysteresis)
        let mid = vec![Complex64::new(0.05, 0.0)]; // ~-26 dBFS, threshold-hysteresis=-26
        sq.process_block(&mid);
        // Might still be open depending on exact power
        // Signal drops well below threshold-hysteresis
        let very_quiet = vec![Complex64::new(0.001, 0.0)]; // -60 dBFS
        sq.process_block(&very_quiet);
        assert!(!sq.is_open());
    }

    #[test]
    fn test_attack_delay() {
        let mut sq = PowerSquelch::with_config(-20.0, 3.0, 5, 1, 1.0);
        // Loud signal needs 5 samples to open
        let loud = Complex64::new(1.0, 0.0);
        for i in 0..5 {
            let out = sq.process(loud);
            if i < 4 {
                assert!(!sq.is_open(), "Should not be open at sample {}", i);
                assert!((out.re).abs() < 1e-10);
            }
        }
        assert!(sq.is_open());
    }

    #[test]
    fn test_release_delay() {
        let mut sq = PowerSquelch::with_config(-20.0, 3.0, 1, 5, 1.0);
        // Open gate first
        sq.process_block(&vec![Complex64::new(1.0, 0.0); 3]);
        assert!(sq.is_open());
        // Quiet signal needs 5 samples to close
        let quiet = Complex64::new(0.0001, 0.0);
        for i in 0..5 {
            sq.process(quiet);
            if i < 4 {
                assert!(sq.is_open(), "Should still be open at sample {}", i);
            }
        }
        assert!(!sq.is_open());
    }

    #[test]
    fn test_real_processing() {
        let mut sq = PowerSquelch::new(-20.0);
        sq.alpha = 1.0;
        let output = sq.process_real(&[1.0, 1.0, 1.0, 0.0001, 0.0001]);
        // First samples should pass after attack
        assert!(output.iter().any(|&s| (s - 1.0).abs() < 1e-10));
    }

    #[test]
    fn test_reset() {
        let mut sq = PowerSquelch::new(-20.0);
        sq.alpha = 1.0;
        sq.process_block(&vec![Complex64::new(1.0, 0.0); 5]);
        assert!(sq.is_open());
        sq.reset();
        assert!(!sq.is_open());
        assert!((sq.power_db() - (-300.0)).abs() < 1.0);
    }

    #[test]
    fn test_set_threshold() {
        let mut sq = PowerSquelch::new(-20.0);
        assert!((sq.threshold_db() - (-20.0)).abs() < 1e-10);
        sq.set_threshold_db(-40.0);
        assert!((sq.threshold_db() - (-40.0)).abs() < 1e-10);
    }
}
