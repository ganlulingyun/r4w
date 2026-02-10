//! # AGC Attack/Decay
//!
//! Automatic Gain Control with separate attack and decay time constants.
//! Fast attack responds quickly to sudden amplitude increases (preventing
//! clipping), while slow decay smoothly reduces gain for quiet passages.
//! Suitable for communications receivers and audio processing.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::agc_attack_decay::AgcAttackDecay;
//!
//! let mut agc = AgcAttackDecay::new(0.01, 0.001, 1.0);
//! let input = vec![(0.5, 0.0), (2.0, 0.0), (0.1, 0.0)];
//! let output = agc.process(&input);
//! // Output amplitude tends toward target (1.0).
//! ```

/// AGC with separate attack and decay time constants.
#[derive(Debug, Clone)]
pub struct AgcAttackDecay {
    /// Attack rate (fast, for increasing signals).
    attack: f64,
    /// Decay rate (slow, for decreasing signals).
    decay: f64,
    /// Target output magnitude.
    target: f64,
    /// Current gain.
    gain: f64,
    /// Maximum gain.
    max_gain: f64,
    /// Minimum gain.
    min_gain: f64,
    /// Smoothed power estimate.
    power_est: f64,
}

impl AgcAttackDecay {
    /// Create a new AGC.
    ///
    /// * `attack` - Attack rate (0..1, higher = faster attack)
    /// * `decay` - Decay rate (0..1, higher = faster decay)
    /// * `target` - Target output RMS magnitude
    pub fn new(attack: f64, decay: f64, target: f64) -> Self {
        Self {
            attack: attack.clamp(0.0001, 1.0),
            decay: decay.clamp(0.0001, 1.0),
            target: target.max(0.001),
            gain: 1.0,
            max_gain: 1e6,
            min_gain: 1e-6,
            power_est: 0.0,
        }
    }

    /// Set gain limits.
    pub fn set_gain_limits(&mut self, min: f64, max: f64) {
        self.min_gain = min.max(1e-12);
        self.max_gain = max.max(self.min_gain);
    }

    /// Process complex samples.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut output = Vec::with_capacity(input.len());
        for &(re, im) in input {
            let power = re * re + im * im;

            // Choose attack or decay rate.
            let alpha = if power > self.power_est {
                self.attack
            } else {
                self.decay
            };
            self.power_est = alpha * power + (1.0 - alpha) * self.power_est;

            // Compute desired gain.
            let rms = self.power_est.sqrt();
            if rms > 1e-30 {
                let desired_gain = self.target / rms;
                // Smoothly approach desired gain.
                let rate = if desired_gain < self.gain {
                    self.attack
                } else {
                    self.decay
                };
                self.gain = rate * desired_gain + (1.0 - rate) * self.gain;
            }
            self.gain = self.gain.clamp(self.min_gain, self.max_gain);

            output.push((re * self.gain, im * self.gain));
        }
        output
    }

    /// Process real samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let complex: Vec<(f64, f64)> = input.iter().map(|&x| (x, 0.0)).collect();
        let out = self.process(&complex);
        out.iter().map(|&(re, _)| re).collect()
    }

    /// Get current gain.
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// Get current power estimate.
    pub fn power_estimate(&self) -> f64 {
        self.power_est
    }

    /// Set gain directly.
    pub fn set_gain(&mut self, gain: f64) {
        self.gain = gain.clamp(self.min_gain, self.max_gain);
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.gain = 1.0;
        self.power_est = 0.0;
    }
}

/// Simple peak-hold AGC for real signals.
#[derive(Debug, Clone)]
pub struct PeakAgc {
    target: f64,
    attack: f64,
    decay: f64,
    peak: f64,
}

impl PeakAgc {
    /// Create peak AGC.
    pub fn new(target: f64, attack: f64, decay: f64) -> Self {
        Self {
            target,
            attack: attack.clamp(0.0001, 1.0),
            decay: decay.clamp(0.0001, 1.0),
            peak: 0.0,
        }
    }

    /// Process samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            let abs = x.abs();
            let alpha = if abs > self.peak { self.attack } else { self.decay };
            self.peak = alpha * abs + (1.0 - alpha) * self.peak;
            let gain = if self.peak > 1e-30 {
                self.target / self.peak
            } else {
                1.0
            };
            output.push(x * gain);
        }
        output
    }

    /// Get current peak estimate.
    pub fn peak(&self) -> f64 {
        self.peak
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.peak = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_agc() {
        let mut agc = AgcAttackDecay::new(0.1, 0.01, 1.0);
        let input = vec![(5.0, 0.0); 100];
        let output = agc.process(&input);
        // After convergence, output should be near target.
        let last_mag = (output.last().unwrap().0.powi(2) + output.last().unwrap().1.powi(2)).sqrt();
        assert!(last_mag < 3.0);
    }

    #[test]
    fn test_attack_vs_decay() {
        // Verify attack is faster than decay.
        let mut agc = AgcAttackDecay::new(0.5, 0.01, 1.0);
        // Sudden loud signal.
        let loud = vec![(10.0, 0.0); 20];
        agc.process(&loud);
        let gain_after_attack = agc.gain();
        // Sudden quiet signal.
        let quiet = vec![(0.01, 0.0); 20];
        agc.process(&quiet);
        let gain_after_decay = agc.gain();
        // Gain should have decreased during attack and increased during decay.
        assert!(gain_after_attack < 1.0);
        assert!(gain_after_decay > gain_after_attack);
    }

    #[test]
    fn test_gain_limits() {
        let mut agc = AgcAttackDecay::new(0.1, 0.01, 1.0);
        agc.set_gain_limits(0.5, 2.0);
        // Very quiet signal → gain should max at 2.0.
        let quiet = vec![(0.001, 0.0); 100];
        agc.process(&quiet);
        assert!(agc.gain() <= 2.0);
    }

    #[test]
    fn test_process_real() {
        let mut agc = AgcAttackDecay::new(0.1, 0.01, 1.0);
        let input = vec![3.0; 50];
        let output = agc.process_real(&input);
        assert_eq!(output.len(), 50);
    }

    #[test]
    fn test_reset() {
        let mut agc = AgcAttackDecay::new(0.1, 0.01, 1.0);
        agc.process(&[(5.0, 0.0); 50]);
        agc.reset();
        assert_eq!(agc.gain(), 1.0);
        assert_eq!(agc.power_estimate(), 0.0);
    }

    #[test]
    fn test_set_gain() {
        let mut agc = AgcAttackDecay::new(0.1, 0.01, 1.0);
        agc.set_gain(0.5);
        assert!((agc.gain() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_peak_agc() {
        let mut agc = PeakAgc::new(1.0, 0.5, 0.01);
        let input = vec![5.0; 50];
        let output = agc.process(&input);
        // After convergence, output should be near 1.0.
        assert!(output.last().unwrap().abs() < 3.0);
    }

    #[test]
    fn test_peak_agc_reset() {
        let mut agc = PeakAgc::new(1.0, 0.5, 0.01);
        agc.process(&[5.0; 10]);
        assert!(agc.peak() > 0.0);
        agc.reset();
        assert_eq!(agc.peak(), 0.0);
    }

    #[test]
    fn test_constant_signal() {
        let mut agc = AgcAttackDecay::new(0.1, 0.1, 1.0);
        let input = vec![(1.0, 0.0); 200];
        let output = agc.process(&input);
        // For constant unit signal targeting unit output, gain → 1.0.
        let last_mag = output.last().unwrap().0.abs();
        assert!((last_mag - 1.0).abs() < 0.5);
    }

    #[test]
    fn test_zero_signal() {
        let mut agc = AgcAttackDecay::new(0.1, 0.01, 1.0);
        let input = vec![(0.0, 0.0); 10];
        let output = agc.process(&input);
        for &(re, im) in &output {
            assert!(re.abs() < 1e-10);
            assert!(im.abs() < 1e-10);
        }
    }
}
