//! FIR Filter implementations
//!
//! Provides lowpass, highpass, and bandpass FIR filters using windowed sinc design.

use num_complex::Complex64;
use std::f64::consts::PI;

/// FIR filter using direct convolution
#[derive(Debug, Clone)]
pub struct FirFilter {
    /// Filter coefficients (impulse response)
    coeffs: Vec<f64>,
    /// Delay line for input samples
    delay_line: Vec<Complex64>,
    /// Current position in delay line
    delay_idx: usize,
}

impl FirFilter {
    /// Create a new FIR filter with the given coefficients
    pub fn new(coeffs: Vec<f64>) -> Self {
        let len = coeffs.len();
        Self {
            coeffs,
            delay_line: vec![Complex64::new(0.0, 0.0); len],
            delay_idx: 0,
        }
    }

    /// Design a lowpass filter using windowed sinc method
    ///
    /// # Arguments
    /// * `cutoff_hz` - Cutoff frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    /// * `num_taps` - Number of filter taps (odd recommended)
    pub fn lowpass(cutoff_hz: f64, sample_rate: f64, num_taps: usize) -> Self {
        let coeffs = design_lowpass_sinc(cutoff_hz, sample_rate, num_taps);
        Self::new(coeffs)
    }

    /// Process a single sample through the filter
    pub fn process_sample(&mut self, input: Complex64) -> Complex64 {
        // Add input to delay line
        self.delay_line[self.delay_idx] = input;

        // Compute output using convolution
        let mut output = Complex64::new(0.0, 0.0);
        let len = self.coeffs.len();

        for i in 0..len {
            let delay_pos = (self.delay_idx + len - i) % len;
            output += self.delay_line[delay_pos] * self.coeffs[i];
        }

        // Advance delay line position
        self.delay_idx = (self.delay_idx + 1) % len;

        output
    }

    /// Process a block of samples
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process_sample(s)).collect()
    }

    /// Process samples in place
    pub fn process_inplace(&mut self, samples: &mut [Complex64]) {
        for sample in samples.iter_mut() {
            *sample = self.process_sample(*sample);
        }
    }

    /// Reset the filter state (clear delay line)
    pub fn reset(&mut self) {
        for s in self.delay_line.iter_mut() {
            *s = Complex64::new(0.0, 0.0);
        }
        self.delay_idx = 0;
    }

    /// Get the number of taps
    pub fn num_taps(&self) -> usize {
        self.coeffs.len()
    }

    /// Get the filter coefficients
    pub fn coefficients(&self) -> &[f64] {
        &self.coeffs
    }

    /// Get the group delay in samples (approximately half the filter length for linear phase)
    pub fn group_delay_samples(&self) -> usize {
        (self.coeffs.len() - 1) / 2
    }
}

/// Design a lowpass filter using windowed sinc method with Blackman window
fn design_lowpass_sinc(cutoff_hz: f64, sample_rate: f64, num_taps: usize) -> Vec<f64> {
    let num_taps = if num_taps % 2 == 0 { num_taps + 1 } else { num_taps };
    let fc = cutoff_hz / sample_rate; // Normalized cutoff (0 to 0.5)
    let m = (num_taps - 1) as f64;
    let mid = m / 2.0;

    let mut coeffs = Vec::with_capacity(num_taps);

    for i in 0..num_taps {
        let n = i as f64;

        // Sinc function
        let sinc = if (n - mid).abs() < 1e-10 {
            2.0 * PI * fc
        } else {
            (2.0 * PI * fc * (n - mid)).sin() / (n - mid)
        };

        // Blackman window
        let window = 0.42 - 0.5 * (2.0 * PI * n / m).cos() + 0.08 * (4.0 * PI * n / m).cos();

        coeffs.push(sinc * window);
    }

    // Normalize to unity gain at DC
    let sum: f64 = coeffs.iter().sum();
    if sum.abs() > 1e-10 {
        for c in coeffs.iter_mut() {
            *c /= sum;
        }
    }

    coeffs
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lowpass_filter_creation() {
        let filter = FirFilter::lowpass(1e6, 5e6, 63);
        assert_eq!(filter.num_taps(), 63);
    }

    #[test]
    fn test_lowpass_unity_dc_gain() {
        let filter = FirFilter::lowpass(1e6, 5e6, 63);
        let sum: f64 = filter.coefficients().iter().sum();
        assert!((sum - 1.0).abs() < 1e-6, "DC gain should be unity, got {}", sum);
    }

    #[test]
    fn test_filter_dc_passthrough() {
        let mut filter = FirFilter::lowpass(1e6, 5e6, 31);

        // Feed DC signal (all ones)
        let dc_input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
        let output = filter.process_block(&dc_input);

        // After settling, output should be close to 1.0
        let last_10: Vec<f64> = output.iter().rev().take(10).map(|c| c.re).collect();
        let avg: f64 = last_10.iter().sum::<f64>() / 10.0;
        assert!((avg - 1.0).abs() < 0.01, "DC passthrough failed, got {}", avg);
    }

    #[test]
    fn test_filter_attenuates_high_freq() {
        let mut filter = FirFilter::lowpass(1e6, 5e6, 63);

        // Generate high frequency signal (2.4 MHz, close to Nyquist)
        let freq = 2.4e6;
        let sample_rate = 5e6;
        let input: Vec<Complex64> = (0..200)
            .map(|i| {
                let t = i as f64 / sample_rate;
                Complex64::new((2.0 * PI * freq * t).cos(), 0.0)
            })
            .collect();

        let output = filter.process_block(&input);

        // Compute power of last half of output (after settling)
        let input_power: f64 = input.iter().skip(100).map(|c| c.norm_sqr()).sum::<f64>();
        let output_power: f64 = output.iter().skip(100).map(|c| c.norm_sqr()).sum::<f64>();

        let attenuation_db = 10.0 * (output_power / input_power).log10();
        assert!(attenuation_db < -20.0, "High freq should be attenuated >20dB, got {} dB", attenuation_db);
    }
}
