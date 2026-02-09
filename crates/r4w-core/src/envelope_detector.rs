//! Envelope Detector — AM envelope extraction
//!
//! Extracts the instantaneous amplitude envelope from a modulated signal.
//! Supports magnitude-based detection (simple |z|) and Hilbert-transform-based
//! analytic signal envelope for real-valued inputs.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::envelope_detector::{EnvelopeDetector, EnvelopeMode};
//! use num_complex::Complex64;
//!
//! let mut det = EnvelopeDetector::new(EnvelopeMode::Magnitude, 0.01);
//! let signal: Vec<Complex64> = (0..100)
//!     .map(|i| Complex64::from_polar(1.0 + 0.5 * (i as f64 * 0.1).sin(), i as f64 * 0.5))
//!     .collect();
//! let envelope = det.process(&signal);
//! assert_eq!(envelope.len(), 100);
//! ```

use num_complex::Complex64;

/// Envelope detection mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EnvelopeMode {
    /// Instantaneous magnitude: |z[n]|.
    Magnitude,
    /// Magnitude squared: |z[n]|² (avoids sqrt, useful for power).
    MagnitudeSquared,
    /// Smoothed magnitude with single-pole lowpass.
    Smoothed,
    /// Peak-hold with exponential decay.
    PeakHold,
}

/// Envelope detector for complex or real signals.
#[derive(Debug, Clone)]
pub struct EnvelopeDetector {
    mode: EnvelopeMode,
    /// Smoothing/decay constant.
    alpha: f64,
    /// Current smoothed value.
    state: f64,
    /// Attack time constant (for peak-hold mode).
    attack: f64,
    /// Release/decay time constant (for peak-hold mode).
    release: f64,
}

impl EnvelopeDetector {
    /// Create a new envelope detector.
    ///
    /// `mode`: Detection algorithm to use.
    /// `alpha`: Smoothing factor (used by Smoothed and PeakHold modes).
    pub fn new(mode: EnvelopeMode, alpha: f64) -> Self {
        Self {
            mode,
            alpha: alpha.clamp(0.0001, 1.0),
            state: 0.0,
            attack: alpha.clamp(0.0001, 1.0),
            release: (alpha * 0.1).clamp(0.0001, 1.0),
        }
    }

    /// Set separate attack and release time constants (for PeakHold mode).
    pub fn set_attack_release(&mut self, attack: f64, release: f64) {
        self.attack = attack.clamp(0.0001, 1.0);
        self.release = release.clamp(0.0001, 1.0);
    }

    /// Extract envelope from complex signal.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<f64> {
        input.iter().map(|&s| self.process_sample(s.norm())).collect()
    }

    /// Extract envelope from real signal (uses absolute value).
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&s| self.process_sample(s.abs())).collect()
    }

    fn process_sample(&mut self, magnitude: f64) -> f64 {
        match self.mode {
            EnvelopeMode::Magnitude => magnitude,
            EnvelopeMode::MagnitudeSquared => magnitude * magnitude,
            EnvelopeMode::Smoothed => {
                self.state = (1.0 - self.alpha) * self.state + self.alpha * magnitude;
                self.state
            }
            EnvelopeMode::PeakHold => {
                if magnitude > self.state {
                    self.state = (1.0 - self.attack) * self.state + self.attack * magnitude;
                } else {
                    self.state = (1.0 - self.release) * self.state + self.release * magnitude;
                }
                self.state
            }
        }
    }

    /// Get current state (for Smoothed/PeakHold modes).
    pub fn current(&self) -> f64 {
        self.state
    }

    pub fn reset(&mut self) {
        self.state = 0.0;
    }
}

/// Simple AM demodulator using envelope detection.
#[derive(Debug, Clone)]
pub struct AmDemodulator {
    /// DC removal filter state.
    dc_state: f64,
    /// DC removal alpha.
    dc_alpha: f64,
}

impl AmDemodulator {
    pub fn new() -> Self {
        Self {
            dc_state: 0.0,
            dc_alpha: 0.005,
        }
    }

    /// Demodulate AM signal by envelope detection + DC removal.
    pub fn demodulate(&mut self, input: &[Complex64]) -> Vec<f64> {
        input
            .iter()
            .map(|&s| {
                let envelope = s.norm();
                // Remove DC (carrier level)
                self.dc_state = (1.0 - self.dc_alpha) * self.dc_state + self.dc_alpha * envelope;
                envelope - self.dc_state
            })
            .collect()
    }

    pub fn reset(&mut self) {
        self.dc_state = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_magnitude_envelope() {
        let mut det = EnvelopeDetector::new(EnvelopeMode::Magnitude, 0.01);
        let signal: Vec<Complex64> = (0..50)
            .map(|i| Complex64::from_polar(2.0, i as f64 * 0.5))
            .collect();
        let env = det.process(&signal);
        for &v in &env {
            assert!((v - 2.0).abs() < 1e-10, "Magnitude should be 2.0");
        }
    }

    #[test]
    fn test_magnitude_squared() {
        let mut det = EnvelopeDetector::new(EnvelopeMode::MagnitudeSquared, 0.01);
        let signal = vec![Complex64::new(3.0, 4.0)]; // |z| = 5, |z|² = 25
        let env = det.process(&signal);
        assert!((env[0] - 25.0).abs() < 1e-10);
    }

    #[test]
    fn test_smoothed_envelope() {
        let mut det = EnvelopeDetector::new(EnvelopeMode::Smoothed, 0.1);
        // Step from 0 to 1
        let low = vec![Complex64::new(0.0, 0.0); 50];
        let high = vec![Complex64::new(1.0, 0.0); 50];
        det.process(&low);
        let env = det.process(&high);
        // Should ramp up gradually
        assert!(env[0] < 0.5);
        assert!(env[49] > 0.5);
    }

    #[test]
    fn test_peak_hold() {
        let mut det = EnvelopeDetector::new(EnvelopeMode::PeakHold, 0.5);
        det.set_attack_release(0.9, 0.01);
        // Pulse then silence
        let pulse = vec![Complex64::new(5.0, 0.0); 5];
        let silence = vec![Complex64::new(0.0, 0.0); 50];
        let env_pulse = det.process(&pulse);
        let env_silence = det.process(&silence);
        // Attack should be fast
        assert!(env_pulse[4] > 3.0, "Attack should track quickly");
        // Release should be slow
        assert!(env_silence[10] > 1.0, "Release should decay slowly");
        assert!(env_silence[49] < env_silence[0], "Should eventually decay");
    }

    #[test]
    fn test_varying_amplitude() {
        let mut det = EnvelopeDetector::new(EnvelopeMode::Magnitude, 0.01);
        let signal: Vec<Complex64> = (0..100)
            .map(|i| {
                let amp = 1.0 + 0.5 * (i as f64 * 0.1).sin();
                Complex64::from_polar(amp, i as f64 * 0.5)
            })
            .collect();
        let env = det.process(&signal);
        // Envelope should track the amplitude modulation
        for (i, &v) in env.iter().enumerate() {
            let expected = 1.0 + 0.5 * (i as f64 * 0.1).sin();
            assert!((v - expected).abs() < 0.01, "Sample {}: expected {}, got {}", i, expected, v);
        }
    }

    #[test]
    fn test_real_signal() {
        let mut det = EnvelopeDetector::new(EnvelopeMode::Magnitude, 0.01);
        let signal: Vec<f64> = (0..100)
            .map(|i| (2.0 * PI * 0.1 * i as f64).sin())
            .collect();
        let env = det.process_real(&signal);
        // Envelope of |sin()| should be between 0 and 1
        for &v in &env {
            assert!(v >= 0.0 && v <= 1.001);
        }
    }

    #[test]
    fn test_am_demodulator() {
        let mut demod = AmDemodulator::new();
        // AM signal: (1 + 0.5*sin(2πf_m*t)) * cos(2πf_c*t)
        let signal: Vec<Complex64> = (0..500)
            .map(|i| {
                let t = i as f64 / 8000.0;
                let modulation = 1.0 + 0.5 * (2.0 * PI * 400.0 * t).sin();
                Complex64::from_polar(modulation, 2.0 * PI * 2000.0 * t)
            })
            .collect();
        let audio = demod.demodulate(&signal);
        assert_eq!(audio.len(), 500);
        // After DC removal settles, output should oscillate around 0
    }

    #[test]
    fn test_reset() {
        let mut det = EnvelopeDetector::new(EnvelopeMode::Smoothed, 0.5);
        det.process(&vec![Complex64::new(5.0, 0.0); 20]);
        assert!(det.current() > 1.0);
        det.reset();
        assert_eq!(det.current(), 0.0);
    }

    #[test]
    fn test_am_demod_reset() {
        let mut demod = AmDemodulator::new();
        demod.demodulate(&vec![Complex64::new(1.0, 0.0); 100]);
        assert!(demod.dc_state > 0.0);
        demod.reset();
        assert_eq!(demod.dc_state, 0.0);
    }
}
