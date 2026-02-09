//! Phase Modulator — Baseband to PM conversion
//!
//! Converts a real-valued baseband signal into a complex output by modulating
//! the instantaneous phase: `output[n] = exp(j * sensitivity * input[n])`.
//! PM counterpart to the existing `frequency_modulator`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::phase_modulator::PhaseModulator;
//! use num_complex::Complex64;
//!
//! let mut pm = PhaseModulator::new(std::f64::consts::PI);
//! let input = vec![0.0, 0.5, 1.0, 0.5, 0.0];
//! let output = pm.process(&input);
//! assert_eq!(output.len(), 5);
//! assert!((output[0].re - 1.0).abs() < 0.001); // exp(j*0) = 1
//! ```

use num_complex::Complex64;

/// Phase modulator: real input → complex PM output.
#[derive(Debug, Clone)]
pub struct PhaseModulator {
    /// Phase sensitivity (radians per unit input).
    sensitivity: f64,
    /// Maximum phase deviation (optional limiter).
    max_deviation: Option<f64>,
}

impl PhaseModulator {
    /// Create a phase modulator with given sensitivity (radians/unit).
    pub fn new(sensitivity: f64) -> Self {
        Self {
            sensitivity,
            max_deviation: None,
        }
    }

    /// Create with frequency deviation specification.
    ///
    /// `deviation_hz`: Maximum phase deviation in radians.
    pub fn with_max_deviation(sensitivity: f64, max_deviation: f64) -> Self {
        Self {
            sensitivity,
            max_deviation: Some(max_deviation.abs()),
        }
    }

    /// Process a block of real samples into PM output.
    pub fn process(&mut self, input: &[f64]) -> Vec<Complex64> {
        input.iter().map(|&x| self.process_sample(x)).collect()
    }

    /// Process a single sample.
    pub fn process_sample(&mut self, sample: f64) -> Complex64 {
        let mut phase = self.sensitivity * sample;
        if let Some(max) = self.max_deviation {
            phase = phase.clamp(-max, max);
        }
        Complex64::from_polar(1.0, phase)
    }

    /// Process complex input (modulates phase of input signal).
    pub fn process_complex(&mut self, input: &[Complex64], modulating: &[f64]) -> Vec<Complex64> {
        input.iter().zip(modulating.iter()).map(|(&sig, &m)| {
            let phase = self.sensitivity * m;
            sig * Complex64::from_polar(1.0, phase)
        }).collect()
    }

    /// Get sensitivity.
    pub fn sensitivity(&self) -> f64 {
        self.sensitivity
    }

    /// Set sensitivity.
    pub fn set_sensitivity(&mut self, sensitivity: f64) {
        self.sensitivity = sensitivity;
    }
}

/// Continuous-phase modulator (maintains phase state).
#[derive(Debug, Clone)]
pub struct ContinuousPhaseModulator {
    sensitivity: f64,
    phase: f64,
}

impl ContinuousPhaseModulator {
    /// Create a continuous-phase modulator.
    pub fn new(sensitivity: f64) -> Self {
        Self {
            sensitivity,
            phase: 0.0,
        }
    }

    /// Process a block: accumulates phase (like FM but input is phase increment).
    pub fn process(&mut self, input: &[f64]) -> Vec<Complex64> {
        input.iter().map(|&x| {
            self.phase += self.sensitivity * x;
            // Keep phase in [-π, π] to prevent float overflow
            while self.phase > std::f64::consts::PI {
                self.phase -= 2.0 * std::f64::consts::PI;
            }
            while self.phase < -std::f64::consts::PI {
                self.phase += 2.0 * std::f64::consts::PI;
            }
            Complex64::from_polar(1.0, self.phase)
        }).collect()
    }

    /// Get current phase.
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Reset phase to zero.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_zero_input() {
        let mut pm = PhaseModulator::new(PI);
        let output = pm.process(&[0.0; 5]);
        for s in &output {
            assert!((s.re - 1.0).abs() < 1e-10);
            assert!(s.im.abs() < 1e-10);
        }
    }

    #[test]
    fn test_pi_input() {
        let mut pm = PhaseModulator::new(1.0);
        let output = pm.process_sample(PI);
        // exp(jπ) = -1
        assert!((output.re - (-1.0)).abs() < 1e-10);
        assert!(output.im.abs() < 1e-10);
    }

    #[test]
    fn test_half_pi() {
        let mut pm = PhaseModulator::new(1.0);
        let output = pm.process_sample(PI / 2.0);
        // exp(jπ/2) = j
        assert!(output.re.abs() < 1e-10);
        assert!((output.im - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_unit_magnitude() {
        let mut pm = PhaseModulator::new(5.0);
        let input: Vec<f64> = (0..100).map(|i| (i as f64 * 0.1).sin()).collect();
        let output = pm.process(&input);
        for s in &output {
            assert!((s.norm() - 1.0).abs() < 1e-10, "PM output must have unit magnitude");
        }
    }

    #[test]
    fn test_max_deviation() {
        let mut pm = PhaseModulator::with_max_deviation(10.0, PI);
        let output = pm.process_sample(1.0);
        // sensitivity*1.0 = 10.0, but clamped to π
        assert!((output.re - (-1.0)).abs() < 1e-10, "Should clamp to π");
    }

    #[test]
    fn test_sensitivity() {
        let mut pm = PhaseModulator::new(2.0);
        assert_eq!(pm.sensitivity(), 2.0);
        pm.set_sensitivity(3.0);
        assert_eq!(pm.sensitivity(), 3.0);
    }

    #[test]
    fn test_process_complex() {
        let mut pm = PhaseModulator::new(PI / 2.0);
        let carrier = vec![Complex64::new(1.0, 0.0); 3];
        let modulating = vec![0.0, 1.0, 0.0];
        let output = pm.process_complex(&carrier, &modulating);
        // modulating=0 → no phase change, modulating=1 → π/2 shift
        assert!((output[0].re - 1.0).abs() < 1e-10);
        assert!(output[1].re.abs() < 1e-10); // cos(π/2) ≈ 0
        assert!((output[1].im - 1.0).abs() < 1e-10); // sin(π/2) = 1
    }

    #[test]
    fn test_continuous_phase() {
        let mut cpm = ContinuousPhaseModulator::new(0.1);
        let input = vec![1.0; 10];
        let output = cpm.process(&input);
        assert_eq!(output.len(), 10);
        // Phase should accumulate: 0.1, 0.2, 0.3, ...
        for (i, &s) in output.iter().enumerate() {
            let expected_phase = 0.1 * (i + 1) as f64;
            let expected = Complex64::from_polar(1.0, expected_phase);
            assert!((s.re - expected.re).abs() < 1e-10);
        }
    }

    #[test]
    fn test_continuous_phase_wrap() {
        let mut cpm = ContinuousPhaseModulator::new(PI);
        let input = vec![1.0; 10];
        cpm.process(&input);
        // Phase should wrap, remaining in [-π, π]
        assert!(cpm.phase().abs() <= PI + 0.01);
    }

    #[test]
    fn test_continuous_phase_reset() {
        let mut cpm = ContinuousPhaseModulator::new(1.0);
        cpm.process(&[1.0; 5]);
        assert!(cpm.phase().abs() > 0.0);
        cpm.reset();
        assert_eq!(cpm.phase(), 0.0);
    }
}
