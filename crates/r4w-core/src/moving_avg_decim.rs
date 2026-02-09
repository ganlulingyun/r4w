//! Moving Average Decimator — Boxcar averaging with decimation
//!
//! Combines a simple moving average (boxcar) filter with decimation. Computes
//! the average of N samples and outputs one value per N samples. Useful for
//! coarse power detection, spectral averaging, and first-stage decimation
//! where filter shape is not critical.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::moving_avg_decim::MovingAvgDecim;
//! use num_complex::Complex64;
//!
//! let mut mad = MovingAvgDecim::new(4);
//! let input: Vec<Complex64> = (0..20)
//!     .map(|i| Complex64::new(i as f64, 0.0))
//!     .collect();
//! let output = mad.process(&input);
//! assert_eq!(output.len(), 5); // 20 / 4 = 5
//! ```

use num_complex::Complex64;

/// Moving average with decimation.
///
/// Computes the mean of each block of N samples. Each output sample is
/// the average of the next N input samples. This is equivalent to a
/// boxcar (rectangular window) FIR filter followed by N:1 decimation.
#[derive(Debug, Clone)]
pub struct MovingAvgDecim {
    /// Averaging/decimation factor.
    length: usize,
    /// Running accumulator for current block.
    accumulator: Complex64,
    /// Number of samples accumulated in current block.
    count: usize,
}

impl MovingAvgDecim {
    /// Create a moving average decimator with given averaging length.
    pub fn new(length: usize) -> Self {
        Self {
            length: length.max(1),
            accumulator: Complex64::new(0.0, 0.0),
            count: 0,
        }
    }

    /// Process a block of complex samples.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() / self.length + 1);

        for &sample in input {
            self.accumulator += sample;
            self.count += 1;

            if self.count >= self.length {
                output.push(self.accumulator / self.length as f64);
                self.accumulator = Complex64::new(0.0, 0.0);
                self.count = 0;
            }
        }

        output
    }

    /// Process real-valued samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() / self.length + 1);
        let mut acc = 0.0;
        let mut count = 0;

        for &sample in input {
            acc += sample;
            count += 1;

            if count >= self.length {
                output.push(acc / self.length as f64);
                acc = 0.0;
                count = 0;
            }
        }

        output
    }

    /// Process and return the sum (not average) — useful for power accumulation.
    pub fn process_sum(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() / self.length + 1);

        for &sample in input {
            self.accumulator += sample;
            self.count += 1;

            if self.count >= self.length {
                output.push(self.accumulator);
                self.accumulator = Complex64::new(0.0, 0.0);
                self.count = 0;
            }
        }

        output
    }

    /// Get averaging/decimation length.
    pub fn length(&self) -> usize {
        self.length
    }

    /// Reset accumulator state.
    pub fn reset(&mut self) {
        self.accumulator = Complex64::new(0.0, 0.0);
        self.count = 0;
    }
}

/// Moving average decimator for power (magnitude-squared) measurements.
///
/// Computes the average |x[n]|² over blocks of N samples.
#[derive(Debug, Clone)]
pub struct PowerAvgDecim {
    length: usize,
    accumulator: f64,
    count: usize,
}

impl PowerAvgDecim {
    pub fn new(length: usize) -> Self {
        Self {
            length: length.max(1),
            accumulator: 0.0,
            count: 0,
        }
    }

    /// Process complex samples, output average power per block.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len() / self.length + 1);

        for &sample in input {
            self.accumulator += sample.norm_sqr();
            self.count += 1;

            if self.count >= self.length {
                output.push(self.accumulator / self.length as f64);
                self.accumulator = 0.0;
                self.count = 0;
            }
        }

        output
    }

    /// Process and output average power in dB.
    pub fn process_db(&mut self, input: &[Complex64]) -> Vec<f64> {
        self.process(input).iter()
            .map(|&p| if p > 1e-30 { 10.0 * p.log10() } else { -300.0 })
            .collect()
    }

    pub fn reset(&mut self) {
        self.accumulator = 0.0;
        self.count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_decimation_length() {
        let mut mad = MovingAvgDecim::new(4);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = mad.process(&input);
        assert_eq!(output.len(), 25);
    }

    #[test]
    fn test_average_value() {
        let mut mad = MovingAvgDecim::new(4);
        let input = vec![
            Complex64::new(0.0, 0.0),
            Complex64::new(2.0, 0.0),
            Complex64::new(4.0, 0.0),
            Complex64::new(6.0, 0.0),
        ];
        let output = mad.process(&input);
        assert_eq!(output.len(), 1);
        assert!((output[0].re - 3.0).abs() < 0.001); // (0+2+4+6)/4 = 3
    }

    #[test]
    fn test_constant_signal() {
        let mut mad = MovingAvgDecim::new(5);
        let input = vec![Complex64::new(3.0, 1.0); 50];
        let output = mad.process(&input);
        assert_eq!(output.len(), 10);
        for &s in &output {
            assert!((s.re - 3.0).abs() < 0.001);
            assert!((s.im - 1.0).abs() < 0.001);
        }
    }

    #[test]
    fn test_partial_block() {
        let mut mad = MovingAvgDecim::new(4);
        let input = vec![Complex64::new(1.0, 0.0); 7]; // 4 + 3 partial
        let output = mad.process(&input);
        assert_eq!(output.len(), 1); // Only complete blocks
    }

    #[test]
    fn test_streaming() {
        let mut mad = MovingAvgDecim::new(4);
        let input1 = vec![Complex64::new(1.0, 0.0); 6]; // 4+2 partial
        let input2 = vec![Complex64::new(1.0, 0.0); 6]; // 2+4 complete
        let out1 = mad.process(&input1);
        let out2 = mad.process(&input2);
        assert_eq!(out1.len(), 1);
        assert_eq!(out2.len(), 2); // Partial from first call completes
    }

    #[test]
    fn test_real_processing() {
        let mut mad = MovingAvgDecim::new(5);
        let input = vec![10.0; 25];
        let output = mad.process_real(&input);
        assert_eq!(output.len(), 5);
        for &v in &output {
            assert!((v - 10.0).abs() < 0.001);
        }
    }

    #[test]
    fn test_sum_mode() {
        let mut mad = MovingAvgDecim::new(4);
        let input = vec![Complex64::new(2.0, 0.0); 8];
        let output = mad.process_sum(&input);
        assert_eq!(output.len(), 2);
        assert!((output[0].re - 8.0).abs() < 0.001); // 4 * 2.0 = 8.0
    }

    #[test]
    fn test_power_avg_decim() {
        let mut pad = PowerAvgDecim::new(4);
        let input = vec![Complex64::new(1.0, 0.0); 20]; // |1+0j|² = 1.0
        let output = pad.process(&input);
        assert_eq!(output.len(), 5);
        for &v in &output {
            assert!((v - 1.0).abs() < 0.001);
        }
    }

    #[test]
    fn test_power_avg_db() {
        let mut pad = PowerAvgDecim::new(4);
        let input = vec![Complex64::new(0.1, 0.0); 8]; // |0.1|² = 0.01, -20 dB
        let output = pad.process_db(&input);
        assert_eq!(output.len(), 2);
        for &v in &output {
            assert!((v - (-20.0)).abs() < 0.1);
        }
    }

    #[test]
    fn test_length_1_passthrough() {
        let mut mad = MovingAvgDecim::new(1);
        let input: Vec<Complex64> = (0..10)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let output = mad.process(&input);
        assert_eq!(output.len(), 10);
        for (i, &s) in output.iter().enumerate() {
            assert!((s.re - i as f64).abs() < 0.001);
        }
    }

    #[test]
    fn test_reset() {
        let mut mad = MovingAvgDecim::new(4);
        mad.process(&vec![Complex64::new(5.0, 0.0); 2]); // Partial
        mad.reset();
        let output = mad.process(&vec![Complex64::new(1.0, 0.0); 4]);
        assert_eq!(output.len(), 1);
        assert!((output[0].re - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_noise_reduction() {
        // Averaging should reduce noise power by ~N
        let mut mad = MovingAvgDecim::new(16);
        // Constant signal + noise-like pattern
        let input: Vec<Complex64> = (0..160)
            .map(|i| Complex64::new(1.0 + (i as f64 * 7.3).sin() * 0.5, 0.0))
            .collect();
        let output = mad.process(&input);
        assert_eq!(output.len(), 10);
        // Output should be close to 1.0 (noise averaged out)
        let variance: f64 = output.iter().map(|s| (s.re - 1.0).powi(2)).sum::<f64>() / output.len() as f64;
        assert!(variance < 0.1, "Averaging should reduce variance, got {}", variance);
    }
}
