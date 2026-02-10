//! # Noise Gate
//!
//! Attenuates signals below a power threshold, letting through
//! only signals above the gate level. Includes hysteresis to avoid
//! rapid on/off switching near the threshold. Used for squelch,
//! voice activity detection, and noise reduction.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::noise_gate::{NoiseGate, GateState};
//!
//! let mut gate = NoiseGate::new(-40.0); // -40 dB threshold
//! let loud = vec![(1.0, 0.0); 20];
//! let quiet = vec![(0.001, 0.0); 100];
//! let out_loud = gate.process(&loud);
//! assert!(out_loud.last().unwrap().0.abs() > 0.5); // Passes through after attack.
//! let out_quiet = gate.process(&quiet);
//! assert!(out_quiet.last().unwrap().0.abs() < 0.01); // Gated after release.
//! ```

/// Gate state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GateState {
    Open,
    Closed,
    Attack,
    Release,
}

/// Noise gate with hysteresis and attack/release.
#[derive(Debug, Clone)]
pub struct NoiseGate {
    /// Open threshold (linear power).
    open_threshold: f64,
    /// Close threshold (linear power, with hysteresis).
    close_threshold: f64,
    /// Attack time in samples.
    attack_samples: usize,
    /// Release time in samples.
    release_samples: usize,
    /// Current state.
    state: GateState,
    /// Current gain (0..1).
    gain: f64,
    /// Transition counter.
    transition_count: usize,
    /// Smoothed power estimate.
    power_est: f64,
    /// Smoothing alpha.
    alpha: f64,
}

impl NoiseGate {
    /// Create a new noise gate.
    ///
    /// * `threshold_db` - Gate threshold in dB (relative to full scale)
    pub fn new(threshold_db: f64) -> Self {
        let open_linear = 10.0_f64.powf(threshold_db / 10.0);
        let hysteresis = 3.0; // 3 dB hysteresis.
        let close_linear = 10.0_f64.powf((threshold_db - hysteresis) / 10.0);
        Self {
            open_threshold: open_linear,
            close_threshold: close_linear,
            attack_samples: 10,
            release_samples: 50,
            state: GateState::Closed,
            gain: 0.0,
            transition_count: 0,
            power_est: 0.0,
            alpha: 0.01,
        }
    }

    /// Create with custom attack/release.
    pub fn with_timing(threshold_db: f64, attack_samples: usize, release_samples: usize) -> Self {
        let mut gate = Self::new(threshold_db);
        gate.attack_samples = attack_samples.max(1);
        gate.release_samples = release_samples.max(1);
        gate
    }

    /// Set hysteresis in dB.
    pub fn set_hysteresis(&mut self, hysteresis_db: f64) {
        let open_db = 10.0 * self.open_threshold.log10();
        self.close_threshold = 10.0_f64.powf((open_db - hysteresis_db) / 10.0);
    }

    /// Process complex samples.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut output = Vec::with_capacity(input.len());
        for &(re, im) in input {
            let power = re * re + im * im;
            self.power_est = self.alpha * power + (1.0 - self.alpha) * self.power_est;

            match self.state {
                GateState::Closed => {
                    if self.power_est > self.open_threshold {
                        self.state = GateState::Attack;
                        self.transition_count = 0;
                    }
                    self.gain = 0.0;
                }
                GateState::Attack => {
                    self.transition_count += 1;
                    self.gain = self.transition_count as f64 / self.attack_samples as f64;
                    if self.transition_count >= self.attack_samples {
                        self.state = GateState::Open;
                        self.gain = 1.0;
                    }
                }
                GateState::Open => {
                    if self.power_est < self.close_threshold {
                        self.state = GateState::Release;
                        self.transition_count = 0;
                    }
                    self.gain = 1.0;
                }
                GateState::Release => {
                    self.transition_count += 1;
                    self.gain =
                        1.0 - self.transition_count as f64 / self.release_samples as f64;
                    if self.transition_count >= self.release_samples {
                        self.state = GateState::Closed;
                        self.gain = 0.0;
                    }
                }
            }

            output.push((re * self.gain, im * self.gain));
        }
        output
    }

    /// Process real samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let complex: Vec<(f64, f64)> = input.iter().map(|&x| (x, 0.0)).collect();
        self.process(&complex).iter().map(|&(re, _)| re).collect()
    }

    /// Get current gate state.
    pub fn state(&self) -> GateState {
        self.state
    }

    /// Get current gain (0..1).
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// Check if gate is open.
    pub fn is_open(&self) -> bool {
        matches!(self.state, GateState::Open | GateState::Attack)
    }

    /// Reset to closed state.
    pub fn reset(&mut self) {
        self.state = GateState::Closed;
        self.gain = 0.0;
        self.transition_count = 0;
        self.power_est = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gate_opens() {
        let mut gate = NoiseGate::new(-20.0);
        gate.alpha = 1.0; // Instant response for testing.
        let loud = vec![(1.0, 0.0); 20];
        let output = gate.process(&loud);
        // After attack period, signal should pass.
        assert!(output.last().unwrap().0.abs() > 0.5);
    }

    #[test]
    fn test_gate_closes() {
        let mut gate = NoiseGate::new(-20.0);
        gate.alpha = 1.0;
        // Open the gate.
        gate.process(&vec![(1.0, 0.0); 20]);
        assert!(gate.is_open());
        // Feed quiet signal.
        let quiet = vec![(0.001, 0.0); 100];
        gate.process(&quiet);
        assert_eq!(gate.state(), GateState::Closed);
    }

    #[test]
    fn test_quiet_stays_gated() {
        let mut gate = NoiseGate::new(-10.0);
        let quiet = vec![(0.001, 0.0); 50];
        let output = gate.process(&quiet);
        for &(re, im) in &output {
            assert!(re.abs() < 0.01);
            assert!(im.abs() < 0.01);
        }
    }

    #[test]
    fn test_attack_release_timing() {
        let mut gate = NoiseGate::with_timing(-30.0, 5, 10);
        gate.alpha = 1.0;
        assert_eq!(gate.state(), GateState::Closed);
        // Loud signal triggers attack.
        gate.process(&vec![(1.0, 0.0); 3]);
        assert_eq!(gate.state(), GateState::Attack);
        gate.process(&vec![(1.0, 0.0); 5]);
        assert_eq!(gate.state(), GateState::Open);
    }

    #[test]
    fn test_hysteresis() {
        let mut gate = NoiseGate::new(-20.0);
        // open_threshold != close_threshold.
        assert!(gate.open_threshold > gate.close_threshold);
    }

    #[test]
    fn test_process_real() {
        let mut gate = NoiseGate::new(-20.0);
        gate.alpha = 1.0;
        let input = vec![1.0; 20];
        let output = gate.process_real(&input);
        assert_eq!(output.len(), 20);
        assert!(output.last().unwrap().abs() > 0.5);
    }

    #[test]
    fn test_gain_range() {
        let mut gate = NoiseGate::new(-20.0);
        gate.alpha = 1.0;
        let input = vec![(1.0, 0.0); 50];
        gate.process(&input);
        assert!(gate.gain() >= 0.0 && gate.gain() <= 1.0);
    }

    #[test]
    fn test_reset() {
        let mut gate = NoiseGate::new(-20.0);
        gate.alpha = 1.0;
        gate.process(&vec![(1.0, 0.0); 20]);
        gate.reset();
        assert_eq!(gate.state(), GateState::Closed);
        assert_eq!(gate.gain(), 0.0);
    }

    #[test]
    fn test_set_hysteresis() {
        let mut gate = NoiseGate::new(-20.0);
        let close_before = gate.close_threshold;
        gate.set_hysteresis(6.0);
        assert!(gate.close_threshold < close_before);
    }

    #[test]
    fn test_empty_input() {
        let mut gate = NoiseGate::new(-20.0);
        let output = gate.process(&[]);
        assert!(output.is_empty());
    }
}
