//! Voltage-Controlled Oscillator — Input-driven complex exponential
//!
//! Generates `exp(j * 2π * sensitivity * ∫x(t)dt)` where the input signal
//! controls the instantaneous frequency. Unlike `nco` (fixed frequency) or
//! `signal_source` (test tones), the VCO takes a real input stream that
//! specifies the instantaneous frequency offset per sample.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::vco::VcoC;
//! use num_complex::Complex64;
//!
//! let mut vco = VcoC::new(1.0, 48000.0);
//! let control = vec![0.0; 100]; // Zero frequency → constant output
//! let output = vco.process(&control);
//! assert_eq!(output.len(), 100);
//! assert!((output[0].re - 1.0).abs() < 0.001); // exp(j*0) = 1
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Complex VCO: real frequency input → complex exponential output.
///
/// Output: `exp(j * phase)` where `phase += 2π * sensitivity * input[n] / sample_rate`.
#[derive(Debug, Clone)]
pub struct VcoC {
    /// Frequency sensitivity (Hz per unit input).
    sensitivity: f64,
    /// Sample rate for frequency-to-phase conversion.
    sample_rate: f64,
    /// Current phase (radians).
    phase: f64,
    /// Amplitude of output.
    amplitude: f64,
}

impl VcoC {
    /// Create a complex VCO.
    ///
    /// `sensitivity`: Hz per unit input.
    /// `sample_rate`: Sample rate in Hz.
    pub fn new(sensitivity: f64, sample_rate: f64) -> Self {
        Self {
            sensitivity,
            sample_rate: sample_rate.max(1.0),
            phase: 0.0,
            amplitude: 1.0,
        }
    }

    /// Create with custom amplitude.
    pub fn with_amplitude(sensitivity: f64, sample_rate: f64, amplitude: f64) -> Self {
        Self {
            sensitivity,
            sample_rate: sample_rate.max(1.0),
            phase: 0.0,
            amplitude,
        }
    }

    /// Process a block of frequency control samples.
    pub fn process(&mut self, freq_input: &[f64]) -> Vec<Complex64> {
        let phase_step_per_unit = 2.0 * PI * self.sensitivity / self.sample_rate;
        freq_input.iter().map(|&f| {
            let output = Complex64::from_polar(self.amplitude, self.phase);
            self.phase += phase_step_per_unit * f;
            // Wrap phase to prevent float overflow
            if self.phase > PI {
                self.phase -= 2.0 * PI;
            } else if self.phase < -PI {
                self.phase += 2.0 * PI;
            }
            output
        }).collect()
    }

    /// Process a single sample.
    pub fn process_sample(&mut self, freq: f64) -> Complex64 {
        let output = Complex64::from_polar(self.amplitude, self.phase);
        self.phase += 2.0 * PI * self.sensitivity * freq / self.sample_rate;
        if self.phase > PI {
            self.phase -= 2.0 * PI;
        } else if self.phase < -PI {
            self.phase += 2.0 * PI;
        }
        output
    }

    /// Get current phase.
    pub fn phase(&self) -> f64 {
        self.phase
    }

    /// Reset phase to zero.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }

    /// Set sensitivity.
    pub fn set_sensitivity(&mut self, sensitivity: f64) {
        self.sensitivity = sensitivity;
    }

    /// Get sensitivity.
    pub fn sensitivity(&self) -> f64 {
        self.sensitivity
    }
}

/// Real-valued VCO: real frequency input → real (cosine) output.
#[derive(Debug, Clone)]
pub struct VcoF {
    /// Inner complex VCO.
    inner: VcoC,
}

impl VcoF {
    /// Create a real VCO.
    pub fn new(sensitivity: f64, sample_rate: f64) -> Self {
        Self {
            inner: VcoC::new(sensitivity, sample_rate),
        }
    }

    /// Process a block of frequency control samples, output cosine.
    pub fn process(&mut self, freq_input: &[f64]) -> Vec<f64> {
        self.inner.process(freq_input).iter().map(|c| c.re).collect()
    }

    /// Process a single sample.
    pub fn process_sample(&mut self, freq: f64) -> f64 {
        self.inner.process_sample(freq).re
    }

    /// Reset phase to zero.
    pub fn reset(&mut self) {
        self.inner.reset();
    }

    /// Get current phase.
    pub fn phase(&self) -> f64 {
        self.inner.phase()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zero_input_constant_output() {
        let mut vco = VcoC::new(1.0, 48000.0);
        let control = vec![0.0; 100];
        let output = vco.process(&control);
        for s in &output {
            assert!((s.re - 1.0).abs() < 1e-10, "Zero input should give exp(j*0)=1");
            assert!(s.im.abs() < 1e-10);
        }
    }

    #[test]
    fn test_unit_magnitude() {
        let mut vco = VcoC::new(1000.0, 48000.0);
        let control: Vec<f64> = (0..500).map(|i| (i as f64 * 0.1).sin()).collect();
        let output = vco.process(&control);
        for s in &output {
            assert!((s.norm() - 1.0).abs() < 1e-10, "VCO output should have unit magnitude");
        }
    }

    #[test]
    fn test_constant_frequency() {
        // 1 Hz at 100 Hz sample rate → 100 samples per cycle
        let mut vco = VcoC::new(1.0, 100.0);
        let control = vec![1.0; 100];
        let output = vco.process(&control);
        // After 100 samples at 1 Hz, phase completes one cycle
        // First sample should be 1.0 (phase=0), last should also be near 1.0
        assert!((output[0].re - 1.0).abs() < 1e-10);
        // At quarter cycle (25 samples), should be near j
        assert!(output[25].im > 0.9);
    }

    #[test]
    fn test_phase_wraps() {
        let mut vco = VcoC::new(10000.0, 48000.0);
        let control = vec![1.0; 10000];
        vco.process(&control);
        assert!(vco.phase().abs() <= PI + 0.01);
    }

    #[test]
    fn test_reset() {
        let mut vco = VcoC::new(1000.0, 48000.0);
        vco.process(&[1.0; 100]);
        assert!(vco.phase().abs() > 0.0);
        vco.reset();
        assert_eq!(vco.phase(), 0.0);
    }

    #[test]
    fn test_set_sensitivity() {
        let mut vco = VcoC::new(1.0, 48000.0);
        assert_eq!(vco.sensitivity(), 1.0);
        vco.set_sensitivity(100.0);
        assert_eq!(vco.sensitivity(), 100.0);
    }

    #[test]
    fn test_amplitude() {
        let mut vco = VcoC::with_amplitude(1.0, 48000.0, 2.0);
        let output = vco.process_sample(0.0);
        assert!((output.re - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_vco_f_real_output() {
        let mut vco = VcoF::new(1.0, 100.0);
        let control = vec![1.0; 100];
        let output = vco.process(&control);
        assert_eq!(output.len(), 100);
        // First sample: cos(0) = 1
        assert!((output[0] - 1.0).abs() < 1e-10);
        // Quarter cycle: cos(π/2) ≈ 0
        assert!(output[25].abs() < 0.1);
    }

    #[test]
    fn test_vco_f_reset() {
        let mut vco = VcoF::new(1000.0, 48000.0);
        vco.process(&[1.0; 50]);
        vco.reset();
        assert_eq!(vco.phase(), 0.0);
    }

    #[test]
    fn test_varying_frequency() {
        let mut vco = VcoC::new(1.0, 1000.0);
        // Frequency ramps from 0 to 100 Hz
        let control: Vec<f64> = (0..1000).map(|i| i as f64 * 0.1).collect();
        let output = vco.process(&control);
        assert_eq!(output.len(), 1000);
        // Should produce chirp-like output
        for s in &output {
            assert!((s.norm() - 1.0).abs() < 1e-10);
        }
    }
}
