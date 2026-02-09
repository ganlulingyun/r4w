//! Moving Average — Sliding window average filter
//!
//! Pure moving average without decimation (unlike `moving_avg_decim`).
//! Supports real, complex, and magnitude-squared modes. Uses efficient
//! circular buffer with running sum (O(1) per sample regardless of length).
//! GNU Radio equivalent: `moving_average_ff`, `moving_average_cc`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::moving_average::MovingAverage;
//!
//! let mut ma = MovingAverage::new(5);
//! let input = vec![0.0, 0.0, 0.0, 0.0, 5.0, 5.0, 5.0, 5.0, 5.0];
//! let output = ma.process(&input);
//! // After 5 samples of 5.0, average = 5.0
//! assert!((output[8] - 5.0).abs() < 1e-10);
//! ```

use num_complex::Complex64;

/// Real-valued moving average filter.
#[derive(Debug, Clone)]
pub struct MovingAverage {
    /// Window length.
    length: usize,
    /// Circular buffer.
    buffer: Vec<f64>,
    /// Write index.
    write_idx: usize,
    /// Running sum.
    sum: f64,
    /// Scaling factor (1/length or custom).
    scale: f64,
}

impl MovingAverage {
    /// Create a moving average with given window length.
    pub fn new(length: usize) -> Self {
        let length = length.max(1);
        Self {
            length,
            buffer: vec![0.0; length],
            write_idx: 0,
            sum: 0.0,
            scale: 1.0 / length as f64,
        }
    }

    /// Create with custom scale factor.
    pub fn with_scale(length: usize, scale: f64) -> Self {
        let mut ma = Self::new(length);
        ma.scale = scale;
        ma
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            output.push(self.process_sample(x));
        }
        output
    }

    /// Process a single sample.
    #[inline]
    pub fn process_sample(&mut self, x: f64) -> f64 {
        // Remove oldest, add newest
        self.sum -= self.buffer[self.write_idx];
        self.buffer[self.write_idx] = x;
        self.sum += x;
        self.write_idx = (self.write_idx + 1) % self.length;
        self.sum * self.scale
    }

    /// Get current average.
    pub fn average(&self) -> f64 {
        self.sum * self.scale
    }

    /// Get window length.
    pub fn length(&self) -> usize {
        self.length
    }

    /// Set window length (resets state).
    pub fn set_length(&mut self, length: usize) {
        let length = length.max(1);
        self.length = length;
        self.buffer = vec![0.0; length];
        self.write_idx = 0;
        self.sum = 0.0;
        self.scale = 1.0 / length as f64;
    }

    /// Reset state.
    pub fn reset(&mut self) {
        self.buffer.fill(0.0);
        self.write_idx = 0;
        self.sum = 0.0;
    }
}

/// Complex-valued moving average filter.
#[derive(Debug, Clone)]
pub struct MovingAverageComplex {
    /// Window length.
    length: usize,
    /// Circular buffer.
    buffer: Vec<Complex64>,
    /// Write index.
    write_idx: usize,
    /// Running sum.
    sum: Complex64,
    /// Scale factor.
    scale: f64,
}

impl MovingAverageComplex {
    /// Create a complex moving average.
    pub fn new(length: usize) -> Self {
        let length = length.max(1);
        Self {
            length,
            buffer: vec![Complex64::new(0.0, 0.0); length],
            write_idx: 0,
            sum: Complex64::new(0.0, 0.0),
            scale: 1.0 / length as f64,
        }
    }

    /// Process a block of complex samples.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            output.push(self.process_sample(x));
        }
        output
    }

    /// Process a single complex sample.
    #[inline]
    pub fn process_sample(&mut self, x: Complex64) -> Complex64 {
        self.sum -= self.buffer[self.write_idx];
        self.buffer[self.write_idx] = x;
        self.sum += x;
        self.write_idx = (self.write_idx + 1) % self.length;
        self.sum * self.scale
    }

    /// Get current average.
    pub fn average(&self) -> Complex64 {
        self.sum * self.scale
    }

    /// Get length.
    pub fn length(&self) -> usize {
        self.length
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.buffer.fill(Complex64::new(0.0, 0.0));
        self.write_idx = 0;
        self.sum = Complex64::new(0.0, 0.0);
    }
}

/// Moving average of magnitude squared (power averaging).
#[derive(Debug, Clone)]
pub struct MovingAverageMagSqrd {
    inner: MovingAverage,
}

impl MovingAverageMagSqrd {
    /// Create a power-averaging filter.
    pub fn new(length: usize) -> Self {
        Self { inner: MovingAverage::new(length) }
    }

    /// Process complex samples, outputting average power.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            output.push(self.process_sample(x));
        }
        output
    }

    /// Process a single complex sample.
    #[inline]
    pub fn process_sample(&mut self, x: Complex64) -> f64 {
        self.inner.process_sample(x.norm_sqr())
    }

    /// Get current average power.
    pub fn power(&self) -> f64 {
        self.inner.average()
    }

    /// Get average power in dB.
    pub fn power_db(&self) -> f64 {
        10.0 * self.inner.average().max(1e-30).log10()
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.inner.reset();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_constant_input() {
        let mut ma = MovingAverage::new(5);
        let input = vec![3.0; 10];
        let output = ma.process(&input);
        // After filling the window, output should be 3.0
        assert!((output[9] - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_step_response() {
        let mut ma = MovingAverage::new(4);
        let mut input = vec![0.0; 4];
        input.extend(vec![4.0; 4]);
        let output = ma.process(&input);
        // After step, ramps up: 0, 0, 0, 0, 1, 2, 3, 4
        assert!((output[4] - 1.0).abs() < 1e-10);
        assert!((output[5] - 2.0).abs() < 1e-10);
        assert!((output[6] - 3.0).abs() < 1e-10);
        assert!((output[7] - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_impulse_response() {
        let mut ma = MovingAverage::new(4);
        let mut input = vec![0.0; 8];
        input[0] = 4.0;
        let output = ma.process(&input);
        // Impulse response: 1.0 for 4 samples, then 0
        assert!((output[0] - 1.0).abs() < 1e-10);
        assert!((output[3] - 1.0).abs() < 1e-10);
        assert!((output[4]).abs() < 1e-10);
    }

    #[test]
    fn test_length_one() {
        let mut ma = MovingAverage::new(1);
        let input = vec![1.0, 2.0, 3.0];
        let output = ma.process(&input);
        assert_eq!(output, input); // Passthrough
    }

    #[test]
    fn test_set_length() {
        let mut ma = MovingAverage::new(5);
        ma.process(&[1.0; 5]);
        ma.set_length(3);
        assert_eq!(ma.length(), 3);
        assert_eq!(ma.average(), 0.0); // Reset after set_length
    }

    #[test]
    fn test_reset() {
        let mut ma = MovingAverage::new(5);
        ma.process(&[1.0; 5]);
        ma.reset();
        assert_eq!(ma.average(), 0.0);
    }

    #[test]
    fn test_with_scale() {
        let mut ma = MovingAverage::with_scale(4, 1.0); // Sum, not average
        let output = ma.process(&[1.0, 1.0, 1.0, 1.0]);
        assert!((output[3] - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex() {
        let mut ma = MovingAverageComplex::new(3);
        let input = vec![
            Complex64::new(3.0, 6.0),
            Complex64::new(3.0, 6.0),
            Complex64::new(3.0, 6.0),
        ];
        let output = ma.process(&input);
        assert!((output[2].re - 3.0).abs() < 1e-10);
        assert!((output[2].im - 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_reset() {
        let mut ma = MovingAverageComplex::new(3);
        ma.process(&[Complex64::new(1.0, 1.0); 5]);
        ma.reset();
        assert_eq!(ma.average(), Complex64::new(0.0, 0.0));
    }

    #[test]
    fn test_mag_sqrd() {
        let mut ma = MovingAverageMagSqrd::new(4);
        // |3+4j|^2 = 25
        let input = vec![Complex64::new(3.0, 4.0); 4];
        let output = ma.process(&input);
        assert!((output[3] - 25.0).abs() < 1e-10);
    }

    #[test]
    fn test_mag_sqrd_power_db() {
        let mut ma = MovingAverageMagSqrd::new(4);
        let input = vec![Complex64::new(1.0, 0.0); 4];
        ma.process(&input);
        let db = ma.power_db();
        assert!(db.abs() < 0.01); // 10*log10(1) = 0 dB
    }

    #[test]
    fn test_mag_sqrd_reset() {
        let mut ma = MovingAverageMagSqrd::new(4);
        ma.process(&[Complex64::new(1.0, 0.0); 4]);
        ma.reset();
        assert_eq!(ma.power(), 0.0);
    }

    #[test]
    fn test_empty() {
        let mut ma = MovingAverage::new(5);
        let output = ma.process(&[]);
        assert!(output.is_empty());
    }

    #[test]
    fn test_streaming() {
        let mut ma = MovingAverage::new(3);
        let out1 = ma.process(&[3.0, 3.0]);
        let out2 = ma.process(&[3.0, 3.0]);
        // After 3 samples of 3.0, average should be 3.0
        assert!((out2[0] - 3.0).abs() < 1e-10);
    }
}
