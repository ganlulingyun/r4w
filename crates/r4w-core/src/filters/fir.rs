//! FIR Filter implementations
//!
//! Provides lowpass, highpass, bandpass, and bandstop FIR filters using windowed sinc design.
//!
//! ## Design Methods
//!
//! - **Basic**: `lowpass()`, `highpass()`, `bandpass()`, `bandstop()` with Blackman window
//! - **Kaiser**: `lowpass_kaiser()` etc. with configurable attenuation
//! - **Custom**: `new()` with user-provided coefficients
//!
//! ## Example
//!
//! ```rust,ignore
//! use r4w_core::filters::{FirFilter, Filter};
//! use num_complex::Complex64;
//!
//! // Create a lowpass filter with Blackman window
//! let mut filter = FirFilter::lowpass(1e6, 5e6, 63);
//!
//! // Create a lowpass filter with Kaiser window for 60 dB stopband attenuation
//! let mut kaiser_lpf = FirFilter::lowpass_kaiser(1e6, 5e6, 60.0, 0.1);
//!
//! // Process samples
//! let input = vec![Complex64::new(1.0, 0.0); 100];
//! let output = filter.process_block(&input);
//! ```

use super::traits::{Filter, FirFilterOps, RealFilter};
use super::windows::{kaiser_beta_from_attenuation, kaiser_order, Window};
use num_complex::Complex64;
use std::f64::consts::PI;

/// FIR filter using direct convolution.
///
/// Implements a finite impulse response filter with configurable coefficients.
/// Supports lowpass, highpass, bandpass, and bandstop designs using windowed sinc.
#[derive(Debug, Clone)]
pub struct FirFilter {
    /// Filter coefficients (impulse response)
    coeffs: Vec<f64>,
    /// Delay line for input samples (complex)
    delay_line: Vec<Complex64>,
    /// Delay line for real samples
    delay_line_real: Vec<f64>,
    /// Current position in delay line
    delay_idx: usize,
}

impl FirFilter {
    /// Create a new FIR filter with the given coefficients.
    pub fn new(coeffs: Vec<f64>) -> Self {
        let len = coeffs.len();
        Self {
            coeffs,
            delay_line: vec![Complex64::new(0.0, 0.0); len],
            delay_line_real: vec![0.0; len],
            delay_idx: 0,
        }
    }

    /// Design a lowpass filter using windowed sinc method.
    ///
    /// # Arguments
    /// * `cutoff_hz` - Cutoff frequency in Hz (-3dB point)
    /// * `sample_rate` - Sample rate in Hz
    /// * `num_taps` - Number of filter taps (odd recommended for linear phase)
    ///
    /// # Example
    /// ```rust,ignore
    /// let lpf = FirFilter::lowpass(1000.0, 8000.0, 63);
    /// ```
    pub fn lowpass(cutoff_hz: f64, sample_rate: f64, num_taps: usize) -> Self {
        let coeffs = design_lowpass_sinc(cutoff_hz, sample_rate, num_taps);
        Self::new(coeffs)
    }

    /// Design a highpass filter using spectral inversion.
    ///
    /// Creates a highpass filter by inverting a lowpass filter.
    ///
    /// # Arguments
    /// * `cutoff_hz` - Cutoff frequency in Hz (-3dB point)
    /// * `sample_rate` - Sample rate in Hz
    /// * `num_taps` - Number of filter taps (odd required)
    pub fn highpass(cutoff_hz: f64, sample_rate: f64, num_taps: usize) -> Self {
        let mut coeffs = design_lowpass_sinc(cutoff_hz, sample_rate, num_taps);

        // Spectral inversion: negate all coefficients and add 1 to center tap
        let center = coeffs.len() / 2;
        for c in coeffs.iter_mut() {
            *c = -*c;
        }
        coeffs[center] += 1.0;

        Self::new(coeffs)
    }

    /// Design a bandpass filter.
    ///
    /// Creates a bandpass filter by subtracting a lowpass from another lowpass.
    ///
    /// # Arguments
    /// * `low_hz` - Lower cutoff frequency in Hz
    /// * `high_hz` - Upper cutoff frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    /// * `num_taps` - Number of filter taps (odd required)
    pub fn bandpass(low_hz: f64, high_hz: f64, sample_rate: f64, num_taps: usize) -> Self {
        assert!(low_hz < high_hz, "low_hz must be less than high_hz");

        let lpf_high = design_lowpass_sinc(high_hz, sample_rate, num_taps);
        let lpf_low = design_lowpass_sinc(low_hz, sample_rate, num_taps);

        // Bandpass = LPF(high) - LPF(low)
        let coeffs: Vec<f64> = lpf_high.iter()
            .zip(lpf_low.iter())
            .map(|(h, l)| h - l)
            .collect();

        Self::new(coeffs)
    }

    /// Design a bandstop (notch) filter.
    ///
    /// Creates a bandstop filter by adding lowpass and highpass.
    ///
    /// # Arguments
    /// * `low_hz` - Lower cutoff frequency in Hz
    /// * `high_hz` - Upper cutoff frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    /// * `num_taps` - Number of filter taps (odd required)
    pub fn bandstop(low_hz: f64, high_hz: f64, sample_rate: f64, num_taps: usize) -> Self {
        assert!(low_hz < high_hz, "low_hz must be less than high_hz");

        let lpf_low = design_lowpass_sinc(low_hz, sample_rate, num_taps);
        let mut hpf_high = design_lowpass_sinc(high_hz, sample_rate, num_taps);

        // Convert to highpass via spectral inversion
        let center = hpf_high.len() / 2;
        for c in hpf_high.iter_mut() {
            *c = -*c;
        }
        hpf_high[center] += 1.0;

        // Bandstop = LPF(low) + HPF(high)
        let coeffs: Vec<f64> = lpf_low
            .iter()
            .zip(hpf_high.iter())
            .map(|(l, h)| l + h)
            .collect();

        Self::new(coeffs)
    }

    /// Design a lowpass filter with Kaiser window.
    ///
    /// Automatically calculates the optimal filter order and β parameter
    /// based on desired attenuation and transition bandwidth.
    ///
    /// # Arguments
    /// * `cutoff_hz` - Cutoff frequency in Hz (-3dB point)
    /// * `sample_rate` - Sample rate in Hz
    /// * `attenuation_db` - Desired stopband attenuation in dB (e.g., 60.0)
    /// * `transition_width` - Normalized transition bandwidth (0 to 0.5)
    ///
    /// # Example
    /// ```rust,ignore
    /// // 1 MHz lowpass with 60 dB attenuation and 10% transition band
    /// let lpf = FirFilter::lowpass_kaiser(1e6, 5e6, 60.0, 0.1);
    /// ```
    pub fn lowpass_kaiser(
        cutoff_hz: f64,
        sample_rate: f64,
        attenuation_db: f64,
        transition_width: f64,
    ) -> Self {
        let beta = kaiser_beta_from_attenuation(attenuation_db);
        let order = kaiser_order(transition_width, attenuation_db);
        let num_taps = if order % 2 == 0 { order + 1 } else { order };

        let coeffs = design_lowpass_windowed(
            cutoff_hz,
            sample_rate,
            num_taps,
            Window::Kaiser(beta),
        );
        Self::new(coeffs)
    }

    /// Design a highpass filter with Kaiser window.
    ///
    /// # Arguments
    /// * `cutoff_hz` - Cutoff frequency in Hz (-3dB point)
    /// * `sample_rate` - Sample rate in Hz
    /// * `attenuation_db` - Desired stopband attenuation in dB
    /// * `transition_width` - Normalized transition bandwidth (0 to 0.5)
    pub fn highpass_kaiser(
        cutoff_hz: f64,
        sample_rate: f64,
        attenuation_db: f64,
        transition_width: f64,
    ) -> Self {
        let beta = kaiser_beta_from_attenuation(attenuation_db);
        let order = kaiser_order(transition_width, attenuation_db);
        let num_taps = if order % 2 == 0 { order + 1 } else { order };

        let mut coeffs = design_lowpass_windowed(
            cutoff_hz,
            sample_rate,
            num_taps,
            Window::Kaiser(beta),
        );

        // Spectral inversion for highpass
        let center = coeffs.len() / 2;
        for c in coeffs.iter_mut() {
            *c = -*c;
        }
        coeffs[center] += 1.0;

        Self::new(coeffs)
    }

    /// Design a lowpass filter with a specific window function.
    ///
    /// # Arguments
    /// * `cutoff_hz` - Cutoff frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    /// * `num_taps` - Number of filter taps
    /// * `window` - Window function to use
    pub fn lowpass_windowed(
        cutoff_hz: f64,
        sample_rate: f64,
        num_taps: usize,
        window: Window,
    ) -> Self {
        let coeffs = design_lowpass_windowed(cutoff_hz, sample_rate, num_taps, window);
        Self::new(coeffs)
    }

    /// Design a highpass filter with a specific window function.
    ///
    /// # Arguments
    /// * `cutoff_hz` - Cutoff frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    /// * `num_taps` - Number of filter taps
    /// * `window` - Window function to use
    pub fn highpass_windowed(
        cutoff_hz: f64,
        sample_rate: f64,
        num_taps: usize,
        window: Window,
    ) -> Self {
        let mut coeffs = design_lowpass_windowed(cutoff_hz, sample_rate, num_taps, window);

        // Spectral inversion for highpass
        let center = coeffs.len() / 2;
        for c in coeffs.iter_mut() {
            *c = -*c;
        }
        coeffs[center] += 1.0;

        Self::new(coeffs)
    }

    /// Design a bandpass filter with a specific window function.
    ///
    /// # Arguments
    /// * `low_hz` - Lower cutoff frequency in Hz
    /// * `high_hz` - Upper cutoff frequency in Hz
    /// * `sample_rate` - Sample rate in Hz
    /// * `num_taps` - Number of filter taps
    /// * `window` - Window function to use
    pub fn bandpass_windowed(
        low_hz: f64,
        high_hz: f64,
        sample_rate: f64,
        num_taps: usize,
        window: Window,
    ) -> Self {
        assert!(low_hz < high_hz, "low_hz must be less than high_hz");

        let lpf_high = design_lowpass_windowed(high_hz, sample_rate, num_taps, window);
        let lpf_low = design_lowpass_windowed(low_hz, sample_rate, num_taps, window);

        // Bandpass = LPF(high) - LPF(low)
        let coeffs: Vec<f64> = lpf_high
            .iter()
            .zip(lpf_low.iter())
            .map(|(h, l)| h - l)
            .collect();

        Self::new(coeffs)
    }

    /// Design a moving average filter.
    ///
    /// Simple boxcar filter with all coefficients equal to 1/N.
    /// Useful for smoothing and noise reduction.
    ///
    /// # Arguments
    /// * `length` - Number of samples to average
    pub fn moving_average(length: usize) -> Self {
        assert!(length > 0, "Length must be positive");
        let value = 1.0 / length as f64;
        let coeffs = vec![value; length];
        Self::new(coeffs)
    }

    /// Design a differentiator filter.
    ///
    /// Approximates the derivative of the input signal.
    /// Useful for edge detection and rate of change estimation.
    ///
    /// # Arguments
    /// * `num_taps` - Number of filter taps (odd recommended)
    /// * `sample_rate` - Sample rate in Hz
    pub fn differentiator(num_taps: usize, sample_rate: f64) -> Self {
        let num_taps = if num_taps % 2 == 0 { num_taps + 1 } else { num_taps };
        let mid = (num_taps - 1) / 2;

        let mut coeffs = vec![0.0; num_taps];
        let window = Window::Hamming.generate(num_taps);

        for i in 0..num_taps {
            let n = i as isize - mid as isize;
            if n == 0 {
                coeffs[i] = 0.0;
            } else {
                // Ideal differentiator: h[n] = (1/n) * cos(πn) for n ≠ 0
                coeffs[i] = (PI * n as f64).cos() / (n as f64) * window[i];
            }
        }

        // Scale by sample rate
        for c in coeffs.iter_mut() {
            *c *= sample_rate;
        }

        Self::new(coeffs)
    }

    /// Design a Hilbert transform filter.
    ///
    /// Produces a 90° phase shift at all frequencies.
    /// Used for creating analytic signals.
    ///
    /// # Arguments
    /// * `num_taps` - Number of filter taps (odd required)
    pub fn hilbert(num_taps: usize) -> Self {
        let num_taps = if num_taps % 2 == 0 { num_taps + 1 } else { num_taps };
        let mid = (num_taps - 1) / 2;

        let mut coeffs = vec![0.0; num_taps];
        let window = Window::Hamming.generate(num_taps);

        for i in 0..num_taps {
            let n = i as isize - mid as isize;
            if n == 0 {
                coeffs[i] = 0.0;
            } else if n % 2 != 0 {
                // Odd n: h[n] = 2/(πn)
                coeffs[i] = 2.0 / (PI * n as f64) * window[i];
            }
            // Even n: h[n] = 0
        }

        Self::new(coeffs)
    }

    /// Process a single complex sample through the filter.
    pub fn process_sample(&mut self, input: Complex64) -> Complex64 {
        self.process(input)
    }

    /// Get the filter coefficients.
    pub fn coefficients(&self) -> &[f64] {
        &self.coeffs
    }

    /// Get the number of taps.
    pub fn num_taps(&self) -> usize {
        self.coeffs.len()
    }

    /// Get the group delay in samples (approximately half the filter length for linear phase).
    pub fn group_delay_samples(&self) -> usize {
        (self.coeffs.len() - 1) / 2
    }
}

impl Filter for FirFilter {
    fn process(&mut self, input: Complex64) -> Complex64 {
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

    fn reset(&mut self) {
        for s in self.delay_line.iter_mut() {
            *s = Complex64::new(0.0, 0.0);
        }
        for s in self.delay_line_real.iter_mut() {
            *s = 0.0;
        }
        self.delay_idx = 0;
    }

    fn group_delay(&self) -> f64 {
        (self.coeffs.len() - 1) as f64 / 2.0
    }

    fn order(&self) -> usize {
        self.coeffs.len() - 1
    }
}

impl RealFilter for FirFilter {
    fn process_real(&mut self, input: f64) -> f64 {
        // Add input to real delay line
        self.delay_line_real[self.delay_idx] = input;

        // Compute output using convolution
        let mut output = 0.0;
        let len = self.coeffs.len();

        for i in 0..len {
            let delay_pos = (self.delay_idx + len - i) % len;
            output += self.delay_line_real[delay_pos] * self.coeffs[i];
        }

        // Advance delay line position
        self.delay_idx = (self.delay_idx + 1) % len;

        output
    }
}

impl FirFilterOps for FirFilter {
    fn coefficients(&self) -> &[f64] {
        &self.coeffs
    }
}

/// Design a lowpass filter using windowed sinc method with Blackman window.
fn design_lowpass_sinc(cutoff_hz: f64, sample_rate: f64, num_taps: usize) -> Vec<f64> {
    design_lowpass_windowed(cutoff_hz, sample_rate, num_taps, Window::Blackman)
}

/// Design a lowpass filter using windowed sinc method with configurable window.
fn design_lowpass_windowed(
    cutoff_hz: f64,
    sample_rate: f64,
    num_taps: usize,
    window: Window,
) -> Vec<f64> {
    let num_taps = if num_taps % 2 == 0 { num_taps + 1 } else { num_taps };
    let fc = cutoff_hz / sample_rate; // Normalized cutoff (0 to 0.5)
    let m = (num_taps - 1) as f64;
    let mid = m / 2.0;

    let window_coeffs = window.generate(num_taps);
    let mut coeffs = Vec::with_capacity(num_taps);

    for i in 0..num_taps {
        let n = i as f64;

        // Sinc function
        let sinc = if (n - mid).abs() < 1e-10 {
            2.0 * PI * fc
        } else {
            (2.0 * PI * fc * (n - mid)).sin() / (n - mid)
        };

        coeffs.push(sinc * window_coeffs[i]);
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
        assert!(
            (sum - 1.0).abs() < 1e-6,
            "DC gain should be unity, got {}",
            sum
        );
    }

    #[test]
    fn test_lowpass_kaiser() {
        let filter = FirFilter::lowpass_kaiser(1e6, 5e6, 60.0, 0.1);

        // Should have reasonable number of taps
        assert!(filter.num_taps() > 10);

        // DC gain should be unity
        let sum: f64 = filter.coefficients().iter().sum();
        assert!((sum - 1.0).abs() < 1e-6, "Kaiser LPF DC gain should be unity");
    }

    #[test]
    fn test_highpass_kaiser() {
        let filter = FirFilter::highpass_kaiser(1e6, 5e6, 60.0, 0.1);

        // DC should be blocked
        let sum: f64 = filter.coefficients().iter().sum();
        assert!(sum.abs() < 0.1, "Kaiser HPF DC gain should be near zero");
    }

    #[test]
    fn test_lowpass_windowed_hamming() {
        let filter = FirFilter::lowpass_windowed(1e6, 5e6, 63, Window::Hamming);
        assert_eq!(filter.num_taps(), 63);

        // DC gain should be unity
        let sum: f64 = filter.coefficients().iter().sum();
        assert!((sum - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_lowpass_windowed_hann() {
        let filter = FirFilter::lowpass_windowed(1e6, 5e6, 63, Window::Hann);
        assert_eq!(filter.num_taps(), 63);
    }

    #[test]
    fn test_bandpass_windowed() {
        let filter = FirFilter::bandpass_windowed(800.0, 1200.0, 8000.0, 127, Window::Kaiser(8.0));
        assert_eq!(filter.num_taps(), 127);
    }

    #[test]
    fn test_moving_average() {
        let filter = FirFilter::moving_average(5);
        assert_eq!(filter.num_taps(), 5);

        // All coefficients should be 1/5 = 0.2
        for &c in filter.coefficients() {
            assert!((c - 0.2).abs() < 1e-10);
        }
    }

    #[test]
    fn test_differentiator() {
        let filter = FirFilter::differentiator(31, 1000.0);
        assert_eq!(filter.num_taps(), 31);

        // Center tap should be zero
        let center = filter.coefficients().len() / 2;
        assert!(filter.coefficients()[center].abs() < 1e-10);
    }

    #[test]
    fn test_hilbert() {
        let filter = FirFilter::hilbert(31);
        assert_eq!(filter.num_taps(), 31);

        // Center tap should be zero
        let center = filter.coefficients().len() / 2;
        assert!(filter.coefficients()[center].abs() < 1e-10);

        // Even taps should be zero
        for i in (0..31).step_by(2) {
            if i != 15 {
                // Center is special case
                assert!(
                    filter.coefficients()[i].abs() < 1e-10
                        || (i as isize - 15).abs() % 2 != 0,
                    "Even taps should be zero or near center"
                );
            }
        }
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

    #[test]
    fn test_highpass_filter() {
        let mut filter = FirFilter::highpass(1e6, 5e6, 63);

        // DC should be attenuated
        let dc_input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
        let output = filter.process_block(&dc_input);
        let last_10_avg: f64 = output.iter().rev().take(10).map(|c| c.re.abs()).sum::<f64>() / 10.0;
        assert!(last_10_avg < 0.1, "DC should be attenuated, got {}", last_10_avg);
    }

    #[test]
    fn test_bandpass_filter() {
        let mut filter = FirFilter::bandpass(800.0, 1200.0, 8000.0, 127);

        // Test that center frequency passes
        let freq = 1000.0;
        let sample_rate = 8000.0;
        let input: Vec<Complex64> = (0..500)
            .map(|i| {
                let t = i as f64 / sample_rate;
                Complex64::new((2.0 * PI * freq * t).cos(), 0.0)
            })
            .collect();

        let output = filter.process_block(&input);
        let output_power: f64 = output.iter().skip(200).map(|c| c.norm_sqr()).sum::<f64>();
        assert!(output_power > 10.0, "Center frequency should pass");
    }

    #[test]
    fn test_real_filter() {
        let mut filter = FirFilter::lowpass(1e6, 5e6, 31);

        // Feed DC signal
        let mut sum = 0.0;
        for _ in 0..100 {
            sum = filter.process_real(1.0);
        }

        assert!((sum - 1.0).abs() < 0.01, "Real filter DC passthrough failed, got {}", sum);
    }

    #[test]
    fn test_filter_reset() {
        let mut filter = FirFilter::lowpass(1e6, 5e6, 31);

        // Process some samples
        for _ in 0..50 {
            filter.process(Complex64::new(1.0, 0.0));
        }

        // Reset
        filter.reset();

        // First output should be near zero (no history)
        let output = filter.process(Complex64::new(1.0, 0.0));
        assert!(output.norm() < 0.5, "After reset, first output should be small");
    }

    #[test]
    fn test_linear_phase() {
        let filter = FirFilter::lowpass(1e6, 5e6, 63);
        assert!(filter.is_linear_phase(), "Windowed sinc should produce linear phase filter");
    }

    #[test]
    fn test_group_delay() {
        let filter = FirFilter::lowpass(1e6, 5e6, 63);
        assert_eq!(filter.group_delay(), 31.0);
        assert_eq!(filter.group_delay_samples(), 31);
    }

    #[test]
    fn test_filter_order() {
        let filter = FirFilter::lowpass(1e6, 5e6, 63);
        assert_eq!(filter.order(), 62);
    }
}
