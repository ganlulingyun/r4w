//! Automatic Gain Control (AGC)
//!
//! Normalizes signal amplitude to a target level. Essential for any receiver
//! chain where input power varies due to distance, fading, or interference.
//!
//! ## Variants
//!
//! - [`Agc`] - Basic AGC with single exponential rate
//! - [`Agc2`] - Dual-rate AGC with separate attack/decay
//! - [`Agc3`] - Fast acquisition AGC (linear acquire, logarithmic track)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::agc::{Agc, AgcConfig};
//! use num_complex::Complex64;
//!
//! let mut agc = Agc::new(AgcConfig {
//!     target_amplitude: 1.0,
//!     rate: 0.01,
//!     max_gain: 1e5,
//!     ..Default::default()
//! });
//!
//! // Process samples - output will converge to target amplitude
//! let input = vec![Complex64::new(0.001, 0.0); 1000];
//! let output = agc.process_block(&input);
//! ```

use num_complex::Complex64;

/// Configuration for basic AGC.
#[derive(Debug, Clone)]
pub struct AgcConfig {
    /// Target output amplitude (default: 1.0)
    pub target_amplitude: f64,
    /// Gain update rate - higher = faster tracking (default: 0.01)
    pub rate: f64,
    /// Maximum gain (clamp to prevent runaway on silence) (default: 1e5)
    pub max_gain: f64,
    /// Initial gain (default: 1.0)
    pub initial_gain: f64,
}

impl Default for AgcConfig {
    fn default() -> Self {
        Self {
            target_amplitude: 1.0,
            rate: 0.01,
            max_gain: 1e5,
            initial_gain: 1.0,
        }
    }
}

/// Basic AGC with single exponential tracking rate.
///
/// Uses a simple gain loop: `gain += rate * (target - |output|)`
///
/// This is equivalent to GNU Radio's `agc_cc`.
#[derive(Debug, Clone)]
pub struct Agc {
    config: AgcConfig,
    gain: f64,
}

impl Agc {
    /// Create a new AGC with the given configuration.
    pub fn new(config: AgcConfig) -> Self {
        let gain = config.initial_gain;
        Self { config, gain }
    }

    /// Create a basic AGC with default settings.
    pub fn default_agc() -> Self {
        Self::new(AgcConfig::default())
    }

    /// Get the current gain value.
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// Process a single complex sample.
    pub fn process(&mut self, input: Complex64) -> Complex64 {
        let output = input * self.gain;
        let output_mag = output.norm();
        self.gain += self.config.rate * (self.config.target_amplitude - output_mag);
        if self.gain < 0.0 {
            self.gain = 0.0;
        }
        if self.gain > self.config.max_gain {
            self.gain = self.config.max_gain;
        }
        output
    }

    /// Process a block of complex samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Process samples in place.
    pub fn process_inplace(&mut self, samples: &mut [Complex64]) {
        for s in samples.iter_mut() {
            *s = self.process(*s);
        }
    }

    /// Reset AGC to initial gain.
    pub fn reset(&mut self) {
        self.gain = self.config.initial_gain;
    }
}

/// Configuration for dual-rate AGC (AGC2).
#[derive(Debug, Clone)]
pub struct Agc2Config {
    /// Target output amplitude (default: 1.0)
    pub target_amplitude: f64,
    /// Attack rate - fast response to increasing signal (default: 0.1)
    pub attack_rate: f64,
    /// Decay rate - slow response to decreasing signal (default: 0.01)
    pub decay_rate: f64,
    /// Maximum gain (default: 1e5)
    pub max_gain: f64,
    /// Initial gain (default: 1.0)
    pub initial_gain: f64,
}

impl Default for Agc2Config {
    fn default() -> Self {
        Self {
            target_amplitude: 1.0,
            attack_rate: 0.1,
            decay_rate: 0.01,
            max_gain: 1e5,
            initial_gain: 1.0,
        }
    }
}

/// Dual-rate AGC with separate attack and decay rates.
///
/// Uses different rates for signal increase (attack) vs decrease (decay).
/// This prevents overshoot on sudden power increases while maintaining
/// smooth gain adjustment during fading.
///
/// This is equivalent to GNU Radio's `agc2_cc`.
#[derive(Debug, Clone)]
pub struct Agc2 {
    config: Agc2Config,
    gain: f64,
}

impl Agc2 {
    /// Create a new AGC2 with the given configuration.
    pub fn new(config: Agc2Config) -> Self {
        let gain = config.initial_gain;
        Self { config, gain }
    }

    /// Get the current gain value.
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// Process a single complex sample.
    pub fn process(&mut self, input: Complex64) -> Complex64 {
        let output = input * self.gain;
        let output_mag = output.norm();
        let error = self.config.target_amplitude - output_mag;

        // Use attack rate when signal is too strong (need to reduce gain)
        // Use decay rate when signal is too weak (need to increase gain)
        let rate = if error < 0.0 {
            self.config.attack_rate
        } else {
            self.config.decay_rate
        };

        self.gain += rate * error;
        if self.gain < 0.0 {
            self.gain = 0.0;
        }
        if self.gain > self.config.max_gain {
            self.gain = self.config.max_gain;
        }
        output
    }

    /// Process a block of complex samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Process samples in place.
    pub fn process_inplace(&mut self, samples: &mut [Complex64]) {
        for s in samples.iter_mut() {
            *s = self.process(*s);
        }
    }

    /// Reset AGC2 to initial gain.
    pub fn reset(&mut self) {
        self.gain = self.config.initial_gain;
    }
}

/// Configuration for fast-acquisition AGC (AGC3).
#[derive(Debug, Clone)]
pub struct Agc3Config {
    /// Target output amplitude (default: 1.0)
    pub target_amplitude: f64,
    /// Attack rate for acquisition phase (fast) (default: 0.5)
    pub attack_rate: f64,
    /// Decay rate for tracking phase (slow) (default: 0.01)
    pub decay_rate: f64,
    /// Maximum gain (default: 1e5)
    pub max_gain: f64,
    /// Initial gain (default: 1.0)
    pub initial_gain: f64,
    /// Number of samples for initial acquisition phase (default: 100)
    pub acquisition_samples: usize,
}

impl Default for Agc3Config {
    fn default() -> Self {
        Self {
            target_amplitude: 1.0,
            attack_rate: 0.5,
            decay_rate: 0.01,
            max_gain: 1e5,
            initial_gain: 1.0,
            acquisition_samples: 100,
        }
    }
}

/// Fast-acquisition AGC with two-phase operation.
///
/// Phase 1 (Acquisition): Uses linear gain calculation for fast lock.
/// Phase 2 (Tracking): Switches to logarithmic-domain tracking for stability.
///
/// This is equivalent to GNU Radio's `agc3_cc`.
#[derive(Debug, Clone)]
pub struct Agc3 {
    config: Agc3Config,
    gain: f64,
    sample_count: usize,
}

impl Agc3 {
    /// Create a new AGC3 with the given configuration.
    pub fn new(config: Agc3Config) -> Self {
        let gain = config.initial_gain;
        Self {
            config,
            gain,
            sample_count: 0,
        }
    }

    /// Get the current gain value.
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// Check if in acquisition phase.
    pub fn is_acquiring(&self) -> bool {
        self.sample_count < self.config.acquisition_samples
    }

    /// Process a single complex sample.
    pub fn process(&mut self, input: Complex64) -> Complex64 {
        let output = input * self.gain;
        let output_mag = output.norm();

        if self.sample_count < self.config.acquisition_samples {
            // Phase 1: Fast acquisition using linear calculation
            if output_mag > 1e-10 {
                let desired_gain = self.config.target_amplitude / (input.norm().max(1e-20));
                self.gain += self.config.attack_rate * (desired_gain - self.gain);
            }
        } else {
            // Phase 2: Slow tracking in log domain
            let error = self.config.target_amplitude - output_mag;
            self.gain += self.config.decay_rate * error;
        }

        if self.gain < 0.0 {
            self.gain = 0.0;
        }
        if self.gain > self.config.max_gain {
            self.gain = self.config.max_gain;
        }

        self.sample_count = self.sample_count.saturating_add(1);
        output
    }

    /// Process a block of complex samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Process samples in place.
    pub fn process_inplace(&mut self, samples: &mut [Complex64]) {
        for s in samples.iter_mut() {
            *s = self.process(*s);
        }
    }

    /// Reset AGC3 to initial state.
    pub fn reset(&mut self) {
        self.gain = self.config.initial_gain;
        self.sample_count = 0;
    }
}

// Implement the Filter trait for AGC variants so they work in the filter pipeline
impl crate::filters::Filter for Agc {
    fn process(&mut self, input: Complex64) -> Complex64 {
        Agc::process(self, input)
    }

    fn reset(&mut self) {
        Agc::reset(self);
    }

    fn group_delay(&self) -> f64 {
        0.0 // AGC has zero group delay
    }

    fn order(&self) -> usize {
        0
    }
}

impl crate::filters::Filter for Agc2 {
    fn process(&mut self, input: Complex64) -> Complex64 {
        Agc2::process(self, input)
    }

    fn reset(&mut self) {
        Agc2::reset(self);
    }

    fn group_delay(&self) -> f64 {
        0.0
    }

    fn order(&self) -> usize {
        0
    }
}

impl crate::filters::Filter for Agc3 {
    fn process(&mut self, input: Complex64) -> Complex64 {
        Agc3::process(self, input)
    }

    fn reset(&mut self) {
        Agc3::reset(self);
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
    fn test_agc_basic_convergence() {
        let mut agc = Agc::new(AgcConfig {
            target_amplitude: 1.0,
            rate: 0.1, // Fast rate for weak signal convergence
            max_gain: 1e5,
            initial_gain: 1.0,
        });

        // Feed a weak signal (amplitude = 0.1)
        let input: Vec<Complex64> = (0..5000)
            .map(|i| {
                let phase = 2.0 * std::f64::consts::PI * 0.1 * i as f64;
                Complex64::new(0.1 * phase.cos(), 0.1 * phase.sin())
            })
            .collect();

        let output = agc.process_block(&input);

        // After convergence, output amplitude should be near target
        let final_mag = output.last().unwrap().norm();
        assert!(
            (final_mag - 1.0).abs() < 0.5,
            "AGC should converge near target: got {}",
            final_mag
        );

        // Gain should have increased significantly
        assert!(agc.gain() > 5.0, "Gain should be large for weak signal");
    }

    #[test]
    fn test_agc_strong_signal() {
        let mut agc = Agc::new(AgcConfig {
            target_amplitude: 1.0,
            rate: 0.001, // Slower rate fine for strong signal (starts overshooting quickly)
            max_gain: 1e5,
            initial_gain: 1.0,
        });

        // Feed a strong signal (amplitude = 10)
        let input: Vec<Complex64> = (0..5000)
            .map(|i| {
                let phase = 2.0 * std::f64::consts::PI * 0.1 * i as f64;
                Complex64::new(10.0 * phase.cos(), 10.0 * phase.sin())
            })
            .collect();

        let output = agc.process_block(&input);

        // After convergence, output amplitude should be near target
        let final_mag = output.last().unwrap().norm();
        assert!(
            (final_mag - 1.0).abs() < 1.0,
            "AGC should converge near target: got {}",
            final_mag
        );

        // Gain should be reduced for strong signal
        assert!(agc.gain() < 1.0, "Gain should be small for strong signal: got {}", agc.gain());
    }

    #[test]
    fn test_agc2_asymmetric_rates() {
        let mut agc = Agc2::new(Agc2Config {
            target_amplitude: 1.0,
            attack_rate: 0.1,
            decay_rate: 0.001,
            max_gain: 1e5,
            initial_gain: 1.0,
        });

        // Feed moderate signal
        let input: Vec<Complex64> = vec![Complex64::new(0.5, 0.0); 500];
        let _ = agc.process_block(&input);

        // Attack should be fast
        let gain_after_500 = agc.gain();
        assert!(gain_after_500 > 1.0, "AGC2 should increase gain for weak signal");
    }

    #[test]
    fn test_agc3_fast_acquisition() {
        let mut agc = Agc3::new(Agc3Config {
            target_amplitude: 1.0,
            attack_rate: 0.5,
            decay_rate: 0.001,
            max_gain: 1e5,
            initial_gain: 1.0,
            acquisition_samples: 50,
        });

        assert!(agc.is_acquiring());

        // Feed signal during acquisition
        let input: Vec<Complex64> = vec![Complex64::new(0.01, 0.0); 100];
        let output = agc.process_block(&input);

        assert!(!agc.is_acquiring());

        // Should converge faster than basic AGC during acquisition
        let mid_mag = output[49].norm();
        assert!(
            mid_mag > 0.1,
            "AGC3 should acquire quickly: got {} at sample 50",
            mid_mag
        );
    }

    #[test]
    fn test_agc_max_gain_clamp() {
        let mut agc = Agc::new(AgcConfig {
            target_amplitude: 1.0,
            rate: 0.01,
            max_gain: 100.0,
            initial_gain: 1.0,
        });

        // Feed silence - gain should be clamped at max
        let input = vec![Complex64::new(0.0, 0.0); 10000];
        let _ = agc.process_block(&input);

        assert!(
            agc.gain() <= 100.0,
            "AGC gain should be clamped at max: got {}",
            agc.gain()
        );
    }

    #[test]
    fn test_agc_reset() {
        let mut agc = Agc::new(AgcConfig {
            initial_gain: 5.0,
            ..Default::default()
        });

        let input = vec![Complex64::new(1.0, 0.0); 100];
        let _ = agc.process_block(&input);
        assert!((agc.gain() - 5.0).abs() > 0.01);

        agc.reset();
        assert!((agc.gain() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_agc_filter_trait() {
        use crate::filters::Filter;
        let mut agc = Agc::default_agc();
        let input = Complex64::new(0.5, 0.0);
        let output = Filter::process(&mut agc, input);
        assert!(output.norm() > 0.0);
        assert_eq!(agc.group_delay(), 0.0);
    }
}
