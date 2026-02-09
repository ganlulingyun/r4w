//! Power Squelch
//!
//! Gates signal output based on input power level. When the input power
//! drops below a threshold, the output is zeroed. Essential for PTT radio
//! receivers and burst-mode communications.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::squelch::{PowerSquelch, SquelchConfig};
//! use num_complex::Complex64;
//!
//! let mut sq = PowerSquelch::new(SquelchConfig {
//!     threshold_db: -20.0,  // Gate signals below -20 dBFS
//!     alpha: 0.5,           // Fast power tracking
//!     ramp_samples: 0,      // Instant gate transitions
//! });
//!
//! let loud = vec![Complex64::new(1.0, 0.0); 50];
//! let output = sq.process_block(&loud);
//! // After first sample (power converges), output passes through
//! assert!(output.last().unwrap().norm() > 0.5);
//! ```

use num_complex::Complex64;

/// Configuration for power squelch.
#[derive(Debug, Clone)]
pub struct SquelchConfig {
    /// Power threshold in dB (relative to full scale). Default: -40.0 dBFS
    pub threshold_db: f64,
    /// Smoothing factor for power estimate (0-1). Higher = faster. Default: 0.01
    pub alpha: f64,
    /// Ramp time in samples for gate on/off transitions. Default: 10
    pub ramp_samples: usize,
}

impl Default for SquelchConfig {
    fn default() -> Self {
        Self {
            threshold_db: -40.0,
            alpha: 0.01,
            ramp_samples: 10,
        }
    }
}

/// Power squelch gate.
///
/// Continuously estimates input power using a single-pole IIR filter.
/// When power exceeds the threshold, the gate opens (output = input).
/// When power drops below, the gate closes (output = zero).
///
/// The ramp feature provides smooth transitions to avoid clicks.
///
/// This is equivalent to GNU Radio's `pwr_squelch_cc`.
#[derive(Debug, Clone)]
pub struct PowerSquelch {
    config: SquelchConfig,
    /// Smoothed power estimate (linear)
    power_estimate: f64,
    /// Threshold in linear scale
    threshold_linear: f64,
    /// Current gate state (0.0 = closed, 1.0 = open)
    gate: f64,
    /// Ramp increment per sample
    ramp_inc: f64,
    /// Whether the gate is currently open
    is_open: bool,
}

impl PowerSquelch {
    /// Create a new power squelch.
    pub fn new(config: SquelchConfig) -> Self {
        let threshold_linear = 10.0f64.powf(config.threshold_db / 10.0);
        let ramp_inc = if config.ramp_samples > 0 {
            1.0 / config.ramp_samples as f64
        } else {
            1.0
        };
        Self {
            config,
            power_estimate: 0.0,
            threshold_linear,
            gate: 0.0,
            ramp_inc,
            is_open: false,
        }
    }

    /// Check if the squelch gate is currently open.
    pub fn is_open(&self) -> bool {
        self.is_open
    }

    /// Get the current estimated power in dB.
    pub fn power_db(&self) -> f64 {
        if self.power_estimate > 1e-30 {
            10.0 * self.power_estimate.log10()
        } else {
            -300.0
        }
    }

    /// Process a single sample.
    pub fn process(&mut self, input: Complex64) -> Complex64 {
        // Update power estimate (single-pole IIR)
        let inst_power = input.norm_sqr();
        self.power_estimate =
            (1.0 - self.config.alpha) * self.power_estimate + self.config.alpha * inst_power;

        // Update gate state
        self.is_open = self.power_estimate > self.threshold_linear;

        // Ramp gate up/down
        if self.is_open {
            self.gate = (self.gate + self.ramp_inc).min(1.0);
        } else {
            self.gate = (self.gate - self.ramp_inc).max(0.0);
        }

        input * self.gate
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Process samples in place.
    pub fn process_inplace(&mut self, samples: &mut [Complex64]) {
        for s in samples.iter_mut() {
            *s = self.process(*s);
        }
    }

    /// Reset squelch to initial state.
    pub fn reset(&mut self) {
        self.power_estimate = 0.0;
        self.gate = 0.0;
        self.is_open = false;
    }
}

impl crate::filters::Filter for PowerSquelch {
    fn process(&mut self, input: Complex64) -> Complex64 {
        PowerSquelch::process(self, input)
    }

    fn reset(&mut self) {
        PowerSquelch::reset(self);
    }

    fn group_delay(&self) -> f64 {
        0.0
    }

    fn order(&self) -> usize {
        0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_squelch_passes_loud_signal() {
        let mut sq = PowerSquelch::new(SquelchConfig {
            threshold_db: -20.0,
            alpha: 0.1,
            ramp_samples: 0, // instant gate
            ..Default::default()
        });

        // Strong signal (well above threshold)
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
        let output = sq.process_block(&input);

        // After initial convergence, output should pass through
        let last = output.last().unwrap();
        assert!(
            last.norm() > 0.5,
            "Loud signal should pass: got {:.3}",
            last.norm()
        );
        assert!(sq.is_open());
    }

    #[test]
    fn test_squelch_blocks_quiet_signal() {
        let mut sq = PowerSquelch::new(SquelchConfig {
            threshold_db: -20.0,
            alpha: 0.1,
            ramp_samples: 0,
            ..Default::default()
        });

        // Very weak signal (below threshold)
        let input: Vec<Complex64> = vec![Complex64::new(0.001, 0.0); 200];
        let output = sq.process_block(&input);

        // Output should be gated
        let last = output.last().unwrap();
        assert!(
            last.norm() < 0.01,
            "Weak signal should be squelched: got {:.3}",
            last.norm()
        );
        assert!(!sq.is_open());
    }

    #[test]
    fn test_squelch_ramp() {
        let ramp = 20;
        let mut sq = PowerSquelch::new(SquelchConfig {
            threshold_db: -30.0,
            alpha: 1.0, // instant power tracking
            ramp_samples: ramp,
        });

        // Start with strong signal
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 50];
        let output = sq.process_block(&input);

        // First sample should not be at full amplitude due to ramp
        assert!(
            output[0].norm() < 0.2,
            "Ramp should start low: got {:.3}",
            output[0].norm()
        );
        // After ramp, should be at full amplitude
        assert!(
            output[ramp + 1].norm() > 0.9,
            "Should reach full amplitude after ramp"
        );
    }

    #[test]
    fn test_squelch_power_db() {
        let mut sq = PowerSquelch::new(SquelchConfig {
            alpha: 1.0, // instant tracking
            ..Default::default()
        });

        let _ = sq.process(Complex64::new(1.0, 0.0));
        assert!(
            (sq.power_db() - 0.0).abs() < 0.1,
            "Unit amplitude should be ~0 dBFS: got {:.1}",
            sq.power_db()
        );
    }

    #[test]
    fn test_squelch_reset() {
        let mut sq = PowerSquelch::new(SquelchConfig::default());
        let _ = sq.process_block(&vec![Complex64::new(1.0, 0.0); 100]);
        assert!(sq.is_open());

        sq.reset();
        assert!(!sq.is_open());
        assert!(sq.power_db() < -100.0);
    }

    #[test]
    fn test_squelch_filter_trait() {
        use crate::filters::Filter;
        let mut sq = PowerSquelch::new(SquelchConfig {
            alpha: 1.0,
            ramp_samples: 0,
            threshold_db: -30.0,
        });
        let output = Filter::process(&mut sq, Complex64::new(1.0, 0.0));
        assert!(output.norm() > 0.0);
    }
}
