//! Feedforward and feedback comb filters for harmonic signal processing.
//!
//! Comb filters create a series of equally-spaced notches or peaks in the
//! frequency response, making them useful for:
//! - Removing or boosting harmonics of a fundamental frequency
//! - Reverb and echo effects in audio
//! - Periodic interference cancellation in communications
//!
//! The fundamental frequency of the comb pattern is `f0 = fs / M`, where `fs`
//! is the sample rate and `M` is the delay in samples.
//!
//! # Variants
//!
//! - **Feedforward (FIR)**: `y[n] = x[n] + g * x[n-M]` — always stable
//! - **Feedback (IIR)**: `y[n] = x[n] + g * y[n-M]` — stable when `|g| < 1`
//!
//! Both variants support notch mode (removes harmonics) and peak mode (boosts
//! harmonics), controlled by the sign of the gain parameter.
//!
//! # Example
//!
//! ```
//! use r4w_core::comb_filter::{CombFilter, CombType};
//!
//! // Create a feedforward comb with delay M=4 and gain 0.8
//! let mut comb = CombFilter::new(CombType::Feedforward, 4, 0.8);
//!
//! // Process a batch of samples
//! let input = vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
//! let output = comb.process_batch(&input);
//!
//! // First sample passes through, delayed copy appears at index 4
//! assert!((output[0] - 1.0).abs() < 1e-10);
//! assert!((output[4] - 0.8).abs() < 1e-10);
//! ```

use std::f64::consts::PI;

/// Type of comb filter.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CombType {
    /// FIR comb: `y[n] = x[n] + g * x[n-M]`
    Feedforward,
    /// IIR comb: `y[n] = x[n] + g * y[n-M]`
    Feedback,
}

/// A real-valued comb filter with configurable delay and gain.
///
/// Supports both feedforward (FIR) and feedback (IIR) topologies.
#[derive(Debug, Clone)]
pub struct CombFilter {
    comb_type: CombType,
    delay: usize,
    gain: f64,
    buffer: Vec<f64>,
    write_pos: usize,
}

impl CombFilter {
    /// Creates a new comb filter.
    ///
    /// # Arguments
    /// * `comb_type` - Feedforward (FIR) or Feedback (IIR)
    /// * `delay` - Delay M in samples (must be >= 1)
    /// * `gain` - Gain factor g (for feedback, should satisfy `|g| < 1` for stability)
    ///
    /// # Panics
    /// Panics if `delay` is 0.
    pub fn new(comb_type: CombType, delay: usize, gain: f64) -> Self {
        assert!(delay >= 1, "Delay must be at least 1 sample");
        Self {
            comb_type,
            delay,
            gain,
            buffer: vec![0.0; delay],
            write_pos: 0,
        }
    }

    /// Creates a notch comb filter that removes harmonics of `f0 = fs / M`.
    ///
    /// Uses feedforward topology with negative gain, creating nulls at
    /// integer multiples of `fs/M`.
    pub fn notch(delay: usize, gain: f64) -> Self {
        // Feedforward with g=-|g| has nulls at f = k * fs/M
        Self::new(CombType::Feedforward, delay, -gain.abs())
    }

    /// Creates a peak comb filter that boosts harmonics of `f0 = fs / M`.
    ///
    /// Uses feedback topology with positive gain, creating resonant peaks at
    /// multiples of `f0`.
    pub fn peak(delay: usize, gain: f64) -> Self {
        let g = gain.abs().min(0.9999); // ensure stability
        Self::new(CombType::Feedback, delay, g)
    }

    /// Processes a single real-valued sample.
    pub fn process(&mut self, input: f64) -> f64 {
        let delayed = self.buffer[self.write_pos];
        let output = match self.comb_type {
            CombType::Feedforward => {
                // y[n] = x[n] + g * x[n-M]
                let y = input + self.gain * delayed;
                self.buffer[self.write_pos] = input;
                y
            }
            CombType::Feedback => {
                // y[n] = x[n] + g * y[n-M]
                let y = input + self.gain * delayed;
                self.buffer[self.write_pos] = y;
                y
            }
        };
        self.write_pos = (self.write_pos + 1) % self.delay;
        output
    }

    /// Processes a batch of real-valued samples.
    pub fn process_batch(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.process(x)).collect()
    }

    /// Resets the internal state (clears the delay buffer).
    pub fn reset(&mut self) {
        self.buffer.fill(0.0);
        self.write_pos = 0;
    }

    /// Returns the delay M in samples.
    pub fn delay(&self) -> usize {
        self.delay
    }

    /// Returns the gain factor g.
    pub fn gain(&self) -> f64 {
        self.gain
    }

    /// Returns the comb filter type.
    pub fn comb_type(&self) -> CombType {
        self.comb_type
    }

    /// Computes the frequency response magnitude at a normalized frequency.
    ///
    /// # Arguments
    /// * `freq_normalized` - Frequency normalized to [0, 1], where 1 = sample rate
    ///
    /// # Returns
    /// The magnitude of the frequency response `|H(e^{j2*pi*f})|`.
    pub fn frequency_response_magnitude(&self, freq_normalized: f64) -> f64 {
        let omega = 2.0 * PI * freq_normalized;
        let (mag, _phase) = self.frequency_response(omega);
        mag
    }

    /// Computes the complex frequency response at angular frequency omega.
    ///
    /// # Arguments
    /// * `omega` - Angular frequency in radians (2*pi*f/fs)
    ///
    /// # Returns
    /// `(magnitude, phase)` of the frequency response.
    pub fn frequency_response(&self, omega: f64) -> (f64, f64) {
        let m = self.delay as f64;
        // e^{-j*omega*M}
        let re_delay = (omega * m).cos();
        let im_delay = -(omega * m).sin();

        match self.comb_type {
            CombType::Feedforward => {
                // H(z) = 1 + g * z^{-M}
                let re = 1.0 + self.gain * re_delay;
                let im = self.gain * im_delay;
                let mag = (re * re + im * im).sqrt();
                let phase = im.atan2(re);
                (mag, phase)
            }
            CombType::Feedback => {
                // H(z) = 1 / (1 - g * z^{-M})
                let re_denom = 1.0 - self.gain * re_delay;
                let im_denom = -(self.gain * im_delay);
                let denom_mag_sq = re_denom * re_denom + im_denom * im_denom;
                let mag = 1.0 / denom_mag_sq.sqrt();
                // phase of 1/D = -phase(D)
                let phase = -im_denom.atan2(re_denom);
                (mag, phase)
            }
        }
    }
}

/// A complex-valued comb filter for IQ signal processing.
///
/// Uses `(f64, f64)` tuples for complex samples where `.0` = real (I)
/// and `.1` = imaginary (Q).
#[derive(Debug, Clone)]
pub struct ComplexCombFilter {
    comb_type: CombType,
    delay: usize,
    gain: f64,
    buffer: Vec<(f64, f64)>,
    write_pos: usize,
}

impl ComplexCombFilter {
    /// Creates a new complex-valued comb filter.
    ///
    /// # Panics
    /// Panics if `delay` is 0.
    pub fn new(comb_type: CombType, delay: usize, gain: f64) -> Self {
        assert!(delay >= 1, "Delay must be at least 1 sample");
        Self {
            comb_type,
            delay,
            gain,
            buffer: vec![(0.0, 0.0); delay],
            write_pos: 0,
        }
    }

    /// Processes a single complex sample.
    pub fn process(&mut self, input: (f64, f64)) -> (f64, f64) {
        let delayed = self.buffer[self.write_pos];
        let output = match self.comb_type {
            CombType::Feedforward => {
                let y = (
                    input.0 + self.gain * delayed.0,
                    input.1 + self.gain * delayed.1,
                );
                self.buffer[self.write_pos] = input;
                y
            }
            CombType::Feedback => {
                let y = (
                    input.0 + self.gain * delayed.0,
                    input.1 + self.gain * delayed.1,
                );
                self.buffer[self.write_pos] = y;
                y
            }
        };
        self.write_pos = (self.write_pos + 1) % self.delay;
        output
    }

    /// Processes a batch of complex samples.
    pub fn process_batch(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        input.iter().map(|&x| self.process(x)).collect()
    }

    /// Resets the internal state.
    pub fn reset(&mut self) {
        self.buffer.fill((0.0, 0.0));
        self.write_pos = 0;
    }

    /// Returns the delay M in samples.
    pub fn delay(&self) -> usize {
        self.delay
    }

    /// Returns the gain factor g.
    pub fn gain(&self) -> f64 {
        self.gain
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPS: f64 = 1e-10;

    #[test]
    fn test_feedforward_impulse_response() {
        // For an impulse [1, 0, 0, 0, ...], feedforward comb with M=3, g=0.5
        // should produce [1, 0, 0, 0.5, 0, 0, 0, ...]
        let mut comb = CombFilter::new(CombType::Feedforward, 3, 0.5);
        let input = vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let output = comb.process_batch(&input);

        assert!((output[0] - 1.0).abs() < EPS);
        assert!((output[1]).abs() < EPS);
        assert!((output[2]).abs() < EPS);
        assert!((output[3] - 0.5).abs() < EPS);
        assert!((output[4]).abs() < EPS);
    }

    #[test]
    fn test_feedback_impulse_response() {
        // For an impulse, feedback comb with M=2, g=0.5
        // y[0] = 1, y[1] = 0, y[2] = 0.5*y[0] = 0.5, y[3] = 0.5*y[1] = 0,
        // y[4] = 0.5*y[2] = 0.25, ...
        let mut comb = CombFilter::new(CombType::Feedback, 2, 0.5);
        let input = vec![1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
        let output = comb.process_batch(&input);

        assert!((output[0] - 1.0).abs() < EPS);
        assert!((output[1]).abs() < EPS);
        assert!((output[2] - 0.5).abs() < EPS);
        assert!((output[3]).abs() < EPS);
        assert!((output[4] - 0.25).abs() < EPS);
        assert!((output[5]).abs() < EPS);
        assert!((output[6] - 0.125).abs() < EPS);
    }

    #[test]
    fn test_feedforward_dc_response() {
        // At DC (omega=0), feedforward H = 1 + g
        let comb = CombFilter::new(CombType::Feedforward, 4, 0.7);
        let (mag, _) = comb.frequency_response(0.0);
        assert!((mag - 1.7).abs() < EPS);
    }

    #[test]
    fn test_feedback_dc_response() {
        // At DC (omega=0), feedback H = 1 / (1 - g)
        let comb = CombFilter::new(CombType::Feedback, 4, 0.5);
        let (mag, _) = comb.frequency_response(0.0);
        assert!((mag - 2.0).abs() < EPS);
    }

    #[test]
    fn test_feedforward_notch_frequencies() {
        // Feedforward with g > 0: nulls at f = (2k+1) * fs/(2M)
        // With M=4, g=1.0: null at omega = pi/4, 3pi/4, 5pi/4, 7pi/4
        // That is f_normalized = 1/8, 3/8, 5/8, 7/8
        let comb = CombFilter::new(CombType::Feedforward, 4, 1.0);
        // Check null at omega = pi/4 (f_normalized = 1/8)
        let mag = comb.frequency_response_magnitude(1.0 / 8.0);
        assert!(mag < 1e-9, "Expected null at f=fs/8, got mag={}", mag);
    }

    #[test]
    fn test_notch_constructor() {
        // notch() uses negative gain feedforward
        let comb = CombFilter::notch(4, 0.9);
        assert_eq!(comb.comb_type(), CombType::Feedforward);
        assert!((comb.gain() + 0.9).abs() < EPS, "gain should be -0.9");
        // With g=-1: nulls at f = k * fs/M. Check null at f=fs/M => omega=2*pi/M
        let comb_unity = CombFilter::notch(4, 1.0);
        let omega_null = 2.0 * PI / 4.0; // fs/M with M=4
        let (mag, _) = comb_unity.frequency_response(omega_null);
        assert!(mag < 1e-9, "Expected null at fs/M, got mag={}", mag);
    }

    #[test]
    fn test_peak_constructor_stability() {
        // peak() should clamp gain for stability
        let comb = CombFilter::peak(4, 1.5);
        assert_eq!(comb.comb_type(), CombType::Feedback);
        assert!(comb.gain() < 1.0, "Gain should be clamped below 1.0");
    }

    #[test]
    fn test_peak_resonance() {
        // Feedback peak comb: resonance at multiples of fs/M
        let comb = CombFilter::peak(4, 0.9);
        // Peak at DC (omega=0): H = 1/(1-0.9) = 10
        let (mag_dc, _) = comb.frequency_response(0.0);
        // At half-harmonic: H should be smaller
        let omega_half = PI / 4.0; // halfway between DC and fs/M
        let (mag_half, _) = comb.frequency_response(omega_half);
        assert!(
            mag_dc > mag_half,
            "DC peak ({}) should be larger than inter-peak ({})",
            mag_dc,
            mag_half
        );
    }

    #[test]
    fn test_reset() {
        let mut comb = CombFilter::new(CombType::Feedback, 3, 0.9);
        // Process some samples to fill buffer
        let _ = comb.process_batch(&[1.0, 2.0, 3.0, 4.0, 5.0]);
        // Reset and verify clean state
        comb.reset();
        // After reset, processing an impulse should behave like a fresh filter
        let output = comb.process(1.0);
        assert!((output - 1.0).abs() < EPS, "After reset, first sample should pass through unchanged");
    }

    #[test]
    fn test_single_vs_batch_processing() {
        let mut comb_single = CombFilter::new(CombType::Feedforward, 3, 0.6);
        let mut comb_batch = CombFilter::new(CombType::Feedforward, 3, 0.6);

        let input = vec![1.0, -0.5, 0.3, 0.8, -1.0, 0.2, 0.0, -0.7];

        let single_output: Vec<f64> = input.iter().map(|&x| comb_single.process(x)).collect();
        let batch_output = comb_batch.process_batch(&input);

        for (a, b) in single_output.iter().zip(batch_output.iter()) {
            assert!((a - b).abs() < EPS, "Single and batch should match");
        }
    }

    #[test]
    fn test_complex_feedforward() {
        // Complex feedforward with M=2, g=0.5
        let mut comb = ComplexCombFilter::new(CombType::Feedforward, 2, 0.5);
        let input: Vec<(f64, f64)> = vec![
            (1.0, 0.0),
            (0.0, 1.0),
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
        ];
        let output = comb.process_batch(&input);

        // y[0] = (1,0) + 0.5*(0,0) = (1,0)
        assert!((output[0].0 - 1.0).abs() < EPS);
        assert!((output[0].1).abs() < EPS);
        // y[1] = (0,1) + 0.5*(0,0) = (0,1)
        assert!((output[1].0).abs() < EPS);
        assert!((output[1].1 - 1.0).abs() < EPS);
        // y[2] = (0,0) + 0.5*(1,0) = (0.5,0)
        assert!((output[2].0 - 0.5).abs() < EPS);
        assert!((output[2].1).abs() < EPS);
        // y[3] = (0,0) + 0.5*(0,1) = (0,0.5)
        assert!((output[3].0).abs() < EPS);
        assert!((output[3].1 - 0.5).abs() < EPS);
    }

    #[test]
    fn test_complex_feedback() {
        // Complex feedback with M=1, g=0.5: y[n] = x[n] + 0.5*y[n-1]
        let mut comb = ComplexCombFilter::new(CombType::Feedback, 1, 0.5);
        let input: Vec<(f64, f64)> = vec![
            (1.0, 1.0),
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
        ];
        let output = comb.process_batch(&input);

        // y[0] = (1,1) + 0.5*(0,0) = (1,1)
        assert!((output[0].0 - 1.0).abs() < EPS);
        assert!((output[0].1 - 1.0).abs() < EPS);
        // y[1] = (0,0) + 0.5*(1,1) = (0.5,0.5)
        assert!((output[1].0 - 0.5).abs() < EPS);
        assert!((output[1].1 - 0.5).abs() < EPS);
        // y[2] = (0,0) + 0.5*(0.5,0.5) = (0.25,0.25)
        assert!((output[2].0 - 0.25).abs() < EPS);
        assert!((output[2].1 - 0.25).abs() < EPS);
    }

    #[test]
    fn test_complex_reset() {
        let mut comb = ComplexCombFilter::new(CombType::Feedback, 2, 0.8);
        let _ = comb.process_batch(&[(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)]);
        comb.reset();
        let output = comb.process((1.0, 0.0));
        assert!((output.0 - 1.0).abs() < EPS);
        assert!((output.1).abs() < EPS);
    }

    #[test]
    fn test_accessors() {
        let comb = CombFilter::new(CombType::Feedforward, 5, -0.3);
        assert_eq!(comb.delay(), 5);
        assert!((comb.gain() - (-0.3)).abs() < EPS);
        assert_eq!(comb.comb_type(), CombType::Feedforward);

        let ccomb = ComplexCombFilter::new(CombType::Feedback, 10, 0.7);
        assert_eq!(ccomb.delay(), 10);
        assert!((ccomb.gain() - 0.7).abs() < EPS);
    }

    #[test]
    #[should_panic(expected = "Delay must be at least 1")]
    fn test_zero_delay_panics() {
        let _ = CombFilter::new(CombType::Feedforward, 0, 0.5);
    }

    #[test]
    #[should_panic(expected = "Delay must be at least 1")]
    fn test_complex_zero_delay_panics() {
        let _ = ComplexCombFilter::new(CombType::Feedforward, 0, 0.5);
    }

    #[test]
    fn test_frequency_response_symmetry() {
        // Feedforward frequency response should have period 2*pi/M
        let m = 5;
        let comb = CombFilter::new(CombType::Feedforward, m, 0.7);
        let omega1 = 0.3;
        let omega2 = omega1 + 2.0 * PI / (m as f64);
        let (mag1, _) = comb.frequency_response(omega1);
        let (mag2, _) = comb.frequency_response(omega2);
        assert!(
            (mag1 - mag2).abs() < 1e-9,
            "Frequency response should be periodic with period 2*pi/M: {} vs {}",
            mag1,
            mag2
        );
    }
}
