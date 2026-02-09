//! DC Blocker — DC offset removal filter
//!
//! Removes the DC component (mean value) from a signal using a single-pole
//! highpass IIR filter. Essential for SDR receivers where hardware or ADC
//! introduces DC offset that would corrupt demodulation.
//!
//! The filter is: y[n] = x[n] - x[n-1] + alpha * y[n-1]
//! where alpha controls the -3dB cutoff frequency.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::dc_blocker::DcBlocker;
//! use num_complex::Complex64;
//!
//! let mut blocker = DcBlocker::new(0.99);
//! let signal: Vec<Complex64> = (0..2000)
//!     .map(|i| Complex64::new(0.5 + (i as f64 * 0.1).sin(), 0.0))
//!     .collect();
//! let output = blocker.process(&signal);
//! // DC offset (0.5) should be removed after settling
//! let dc: f64 = output[500..].iter().map(|s| s.re).sum::<f64>() / 1500.0;
//! assert!(dc.abs() < 0.1, "DC should be removed, got {}", dc);
//! ```

use num_complex::Complex64;

/// DC blocking filter using a single-pole IIR highpass.
///
/// Transfer function: H(z) = (1 - z⁻¹) / (1 - alpha*z⁻¹)
/// Cutoff frequency: f_c = (1-alpha) * fs / (2π)
#[derive(Debug, Clone)]
pub struct DcBlocker {
    /// Pole location (0.99-0.9999 typical). Higher = narrower notch.
    alpha: f64,
    /// Previous input sample.
    prev_input: Complex64,
    /// Previous output sample.
    prev_output: Complex64,
}

impl DcBlocker {
    /// Create a DC blocker.
    ///
    /// `alpha`: Pole location (0.99 = wide notch, 0.9999 = narrow notch).
    /// Higher alpha = less distortion of low-frequency content, slower settling.
    pub fn new(alpha: f64) -> Self {
        Self {
            alpha: alpha.clamp(0.9, 0.99999),
            prev_input: Complex64::new(0.0, 0.0),
            prev_output: Complex64::new(0.0, 0.0),
        }
    }

    /// Create from desired cutoff frequency.
    ///
    /// `cutoff_hz`: -3dB point of the highpass filter.
    /// `sample_rate`: Sample rate in Hz.
    pub fn from_cutoff(cutoff_hz: f64, sample_rate: f64) -> Self {
        let alpha = 1.0 - (2.0 * std::f64::consts::PI * cutoff_hz / sample_rate);
        Self::new(alpha)
    }

    /// Process a block of complex samples.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len());

        for &sample in input {
            // y[n] = x[n] - x[n-1] + alpha * y[n-1]
            let out = sample - self.prev_input + self.prev_output * self.alpha;
            self.prev_input = sample;
            self.prev_output = out;
            output.push(out);
        }

        output
    }

    /// Process real-valued samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let complex: Vec<Complex64> = input.iter()
            .map(|&r| Complex64::new(r, 0.0))
            .collect();
        self.process(&complex).iter().map(|c| c.re).collect()
    }

    /// Get the -3dB cutoff frequency for a given sample rate.
    pub fn cutoff_hz(&self, sample_rate: f64) -> f64 {
        (1.0 - self.alpha) * sample_rate / (2.0 * std::f64::consts::PI)
    }

    /// Get the current DC estimate (complement of what was removed).
    pub fn dc_estimate(&self) -> Complex64 {
        self.prev_input - self.prev_output
    }

    /// Reset the filter state.
    pub fn reset(&mut self) {
        self.prev_input = Complex64::new(0.0, 0.0);
        self.prev_output = Complex64::new(0.0, 0.0);
    }
}

/// Real-valued DC blocker (no complex overhead).
#[derive(Debug, Clone)]
pub struct RealDcBlocker {
    alpha: f64,
    prev_input: f64,
    prev_output: f64,
}

impl RealDcBlocker {
    pub fn new(alpha: f64) -> Self {
        Self {
            alpha: alpha.clamp(0.9, 0.99999),
            prev_input: 0.0,
            prev_output: 0.0,
        }
    }

    pub fn from_cutoff(cutoff_hz: f64, sample_rate: f64) -> Self {
        let alpha = 1.0 - (2.0 * std::f64::consts::PI * cutoff_hz / sample_rate);
        Self::new(alpha)
    }

    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());

        for &sample in input {
            let out = sample - self.prev_input + self.alpha * self.prev_output;
            self.prev_input = sample;
            self.prev_output = out;
            output.push(out);
        }

        output
    }

    pub fn reset(&mut self) {
        self.prev_input = 0.0;
        self.prev_output = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_removes_dc_offset() {
        let mut blocker = DcBlocker::new(0.99);
        // Signal with DC offset of 2.0
        let input: Vec<Complex64> = (0..2000)
            .map(|i| Complex64::new(2.0 + (i as f64 * 0.1).sin(), 0.0))
            .collect();
        let output = blocker.process(&input);
        // After settling (time constant = 1/(1-0.99) = 100 samples), DC should be removed
        let dc: f64 = output[500..].iter().map(|s| s.re).sum::<f64>() / 1500.0;
        assert!(dc.abs() < 0.1, "DC should be removed, got {}", dc);
    }

    #[test]
    fn test_passes_ac_signal() {
        let mut blocker = DcBlocker::new(0.998);
        // Pure AC signal (no DC offset)
        let input: Vec<Complex64> = (0..500)
            .map(|i| Complex64::new((2.0 * PI * 0.05 * i as f64).sin(), 0.0))
            .collect();
        let output = blocker.process(&input);
        // AC power should be mostly preserved
        let input_power: f64 = input[200..].iter().map(|s| s.norm_sqr()).sum::<f64>();
        let output_power: f64 = output[200..].iter().map(|s| s.norm_sqr()).sum::<f64>();
        assert!(output_power > input_power * 0.8, "AC signal should be mostly preserved");
    }

    #[test]
    fn test_zero_signal() {
        let mut blocker = DcBlocker::new(0.998);
        let input = vec![Complex64::new(0.0, 0.0); 100];
        let output = blocker.process(&input);
        for &s in &output {
            assert_eq!(s.norm(), 0.0);
        }
    }

    #[test]
    fn test_from_cutoff() {
        let blocker = DcBlocker::from_cutoff(100.0, 48000.0);
        let cutoff = blocker.cutoff_hz(48000.0);
        assert!((cutoff - 100.0).abs() < 20.0, "Cutoff should be ~100 Hz, got {}", cutoff);
    }

    #[test]
    fn test_output_length() {
        let mut blocker = DcBlocker::new(0.998);
        let input = vec![Complex64::new(1.0, 0.0); 50];
        let output = blocker.process(&input);
        assert_eq!(output.len(), 50);
    }

    #[test]
    fn test_complex_dc() {
        let mut blocker = DcBlocker::new(0.99);
        // Complex DC offset
        let input = vec![Complex64::new(1.0, 2.0); 2000];
        let output = blocker.process(&input);
        // Both I and Q DC should be removed
        let dc_i: f64 = output[500..].iter().map(|s| s.re).sum::<f64>() / 1500.0;
        let dc_q: f64 = output[500..].iter().map(|s| s.im).sum::<f64>() / 1500.0;
        assert!(dc_i.abs() < 0.1, "I-channel DC should be removed, got {}", dc_i);
        assert!(dc_q.abs() < 0.1, "Q-channel DC should be removed, got {}", dc_q);
    }

    #[test]
    fn test_real_dc_blocker() {
        let mut blocker = RealDcBlocker::new(0.99);
        let input: Vec<f64> = (0..2000)
            .map(|i| 3.0 + (i as f64 * 0.1).sin())
            .collect();
        let output = blocker.process(&input);
        let dc: f64 = output[500..].iter().sum::<f64>() / 1500.0;
        assert!(dc.abs() < 0.1, "Real DC should be removed, got {}", dc);
    }

    #[test]
    fn test_streaming() {
        let mut blocker = DcBlocker::new(0.99);
        let input1 = vec![Complex64::new(5.0, 0.0); 500];
        let input2 = vec![Complex64::new(5.0, 0.0); 500];
        blocker.process(&input1);
        let output2 = blocker.process(&input2);
        // After 1000 samples with alpha=0.99 (tc=100), DC should be converged
        let dc: f64 = output2[200..].iter().map(|s| s.re).sum::<f64>() / 300.0;
        assert!(dc.abs() < 0.1);
    }

    #[test]
    fn test_reset() {
        let mut blocker = DcBlocker::new(0.998);
        blocker.process(&vec![Complex64::new(10.0, 0.0); 100]);
        blocker.reset();
        let output = blocker.process(&vec![Complex64::new(0.0, 0.0); 10]);
        for &s in &output {
            assert!(s.norm() < 0.01);
        }
    }

    #[test]
    fn test_narrow_notch_preserves_low_freq() {
        let mut blocker = DcBlocker::new(0.9999); // Very narrow notch
        // Low frequency signal (50 Hz at 48 kHz) should pass with minimal attenuation
        let input: Vec<Complex64> = (0..2000)
            .map(|i| Complex64::new((2.0 * PI * 50.0 * i as f64 / 48000.0).sin(), 0.0))
            .collect();
        let output = blocker.process(&input);
        let input_power: f64 = input[500..].iter().map(|s| s.norm_sqr()).sum::<f64>();
        let output_power: f64 = output[500..].iter().map(|s| s.norm_sqr()).sum::<f64>();
        let ratio = output_power / input_power;
        assert!(ratio > 0.9, "Narrow notch should preserve 50 Hz, ratio = {}", ratio);
    }

    #[test]
    fn test_process_real_method() {
        let mut blocker = DcBlocker::new(0.99);
        let input: Vec<f64> = vec![2.0; 1000];
        let output = blocker.process_real(&input);
        assert_eq!(output.len(), 1000);
        let dc: f64 = output[500..].iter().sum::<f64>() / 500.0;
        assert!(dc.abs() < 0.1);
    }
}
