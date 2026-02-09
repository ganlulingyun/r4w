//! Arbitrary Resampler — Non-rational sample rate conversion
//!
//! Resamples a signal by an arbitrary (possibly irrational) ratio using
//! cubic interpolation. Unlike the polyphase rational resampler, this handles
//! any conversion ratio (e.g., 48000 → 44100 Hz without computing the exact
//! 147/160 ratio).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::arbitrary_resampler::ArbitraryResampler;
//! use num_complex::Complex64;
//!
//! // Resample from 48 kHz to 44.1 kHz (ratio = 0.91875)
//! let mut resampler = ArbitraryResampler::new(0.91875);
//! let input: Vec<Complex64> = (0..100)
//!     .map(|i| Complex64::new((i as f64 * 0.1).sin(), 0.0))
//!     .collect();
//! let output = resampler.process(&input);
//! assert!(output.len() < input.len()); // Downsampled
//! ```

use num_complex::Complex64;

/// Arbitrary sample rate converter using cubic interpolation.
#[derive(Debug, Clone)]
pub struct ArbitraryResampler {
    /// Resampling ratio (output_rate / input_rate).
    ratio: f64,
    /// Fractional sample position.
    mu: f64,
    /// History buffer for interpolation (4 samples for cubic).
    history: [Complex64; 4],
    /// Number of samples consumed.
    consumed: u64,
    /// Number of samples produced.
    produced: u64,
}

impl ArbitraryResampler {
    /// Create an arbitrary resampler.
    ///
    /// `ratio`: Output rate / input rate. < 1.0 = downsample, > 1.0 = upsample.
    pub fn new(ratio: f64) -> Self {
        Self {
            ratio: ratio.max(0.01).min(100.0),
            mu: 0.0,
            history: [Complex64::new(0.0, 0.0); 4],
            consumed: 0,
            produced: 0,
        }
    }

    /// Create from input and output sample rates.
    pub fn from_rates(input_rate: f64, output_rate: f64) -> Self {
        Self::new(output_rate / input_rate)
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let estimated_output = (input.len() as f64 * self.ratio) as usize + 2;
        let mut output = Vec::with_capacity(estimated_output);

        for &sample in input {
            // Shift history
            self.history[0] = self.history[1];
            self.history[1] = self.history[2];
            self.history[2] = self.history[3];
            self.history[3] = sample;
            self.consumed += 1;

            // Generate output samples while mu < 1
            while self.mu < 1.0 {
                if self.consumed >= 4 {
                    let interpolated = cubic_interpolate(&self.history, self.mu);
                    output.push(interpolated);
                    self.produced += 1;
                }
                self.mu += 1.0 / self.ratio;
            }
            self.mu -= 1.0;
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

    /// Get the resampling ratio.
    pub fn ratio(&self) -> f64 {
        self.ratio
    }

    /// Get total samples consumed.
    pub fn consumed(&self) -> u64 {
        self.consumed
    }

    /// Get total samples produced.
    pub fn produced(&self) -> u64 {
        self.produced
    }

    /// Reset the resampler state.
    pub fn reset(&mut self) {
        self.mu = 0.0;
        self.history = [Complex64::new(0.0, 0.0); 4];
        self.consumed = 0;
        self.produced = 0;
    }
}

/// Cubic (Hermite) interpolation between 4 samples.
///
/// `h`: 4 samples [h[-1], h[0], h[1], h[2]].
/// `mu`: Fractional position between h[1] and h[2] (0 to 1).
fn cubic_interpolate(h: &[Complex64; 4], mu: f64) -> Complex64 {
    let a0 = -0.5 * h[0] + 1.5 * h[1] - 1.5 * h[2] + 0.5 * h[3];
    let a1 = h[0] - 2.5 * h[1] + 2.0 * h[2] - 0.5 * h[3];
    let a2 = -0.5 * h[0] + 0.5 * h[2];
    let a3 = h[1];

    ((a0 * mu + a1) * mu + a2) * mu + a3
}

/// Linear interpolation resampler (simpler, less quality).
#[derive(Debug, Clone)]
pub struct LinearResampler {
    ratio: f64,
    mu: f64,
    prev: Complex64,
    consumed: u64,
}

impl LinearResampler {
    pub fn new(ratio: f64) -> Self {
        Self {
            ratio: ratio.max(0.01).min(100.0),
            mu: 0.0,
            prev: Complex64::new(0.0, 0.0),
            consumed: 0,
        }
    }

    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity((input.len() as f64 * self.ratio) as usize + 2);

        for &sample in input {
            self.consumed += 1;

            while self.mu < 1.0 {
                if self.consumed >= 2 {
                    let interpolated = self.prev * (1.0 - self.mu) + sample * self.mu;
                    output.push(interpolated);
                }
                self.mu += 1.0 / self.ratio;
            }
            self.mu -= 1.0;
            self.prev = sample;
        }

        output
    }

    pub fn reset(&mut self) {
        self.mu = 0.0;
        self.prev = Complex64::new(0.0, 0.0);
        self.consumed = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_downsample_ratio() {
        let mut resampler = ArbitraryResampler::new(0.5); // 2:1 downsample
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = resampler.process(&input);
        // Should produce roughly half the samples
        assert!(output.len() >= 45 && output.len() <= 55,
            "Expected ~50 output samples, got {}", output.len());
    }

    #[test]
    fn test_upsample_ratio() {
        let mut resampler = ArbitraryResampler::new(2.0); // 1:2 upsample
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = resampler.process(&input);
        // Should produce roughly double the samples
        assert!(output.len() >= 190 && output.len() <= 210,
            "Expected ~200 output samples, got {}", output.len());
    }

    #[test]
    fn test_ratio_1_passthrough() {
        let mut resampler = ArbitraryResampler::new(1.0);
        let input: Vec<Complex64> = (0..50)
            .map(|i| Complex64::new(i as f64 * 0.1, 0.0))
            .collect();
        let output = resampler.process(&input);
        // Should produce approximately same number of samples
        assert!((output.len() as i32 - input.len() as i32).abs() <= 5);
    }

    #[test]
    fn test_from_rates() {
        let resampler = ArbitraryResampler::from_rates(48000.0, 44100.0);
        assert!((resampler.ratio() - 0.91875).abs() < 0.001);
    }

    #[test]
    fn test_streaming() {
        let mut resampler = ArbitraryResampler::new(0.75);
        let input1 = vec![Complex64::new(1.0, 0.0); 50];
        let input2 = vec![Complex64::new(1.0, 0.0); 50];
        let out1 = resampler.process(&input1);
        let out2 = resampler.process(&input2);
        let total = out1.len() + out2.len();
        assert!(total >= 70 && total <= 80, "Expected ~75, got {}", total);
    }

    #[test]
    fn test_preserves_low_frequency() {
        let mut resampler = ArbitraryResampler::new(0.5);
        // Low frequency tone (well below Nyquist after downsampling)
        let input: Vec<Complex64> = (0..400)
            .map(|i| Complex64::new((2.0 * PI * 0.02 * i as f64).sin(), 0.0))
            .collect();
        let output = resampler.process(&input);
        // Output should still be sinusoidal
        assert!(output.len() > 100);
        // Check that output has some variation (not all zeros)
        let max_abs = output.iter().map(|s| s.re.abs()).fold(0.0_f64, f64::max);
        assert!(max_abs > 0.1, "Should preserve low frequency content");
    }

    #[test]
    fn test_real_processing() {
        let mut resampler = ArbitraryResampler::new(0.75);
        let input: Vec<f64> = (0..100).map(|i| (i as f64 * 0.1).sin()).collect();
        let output = resampler.process_real(&input);
        assert!(output.len() >= 65 && output.len() <= 80);
    }

    #[test]
    fn test_linear_resampler() {
        let mut resampler = LinearResampler::new(0.5);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = resampler.process(&input);
        assert!(output.len() >= 45 && output.len() <= 55);
    }

    #[test]
    fn test_reset() {
        let mut resampler = ArbitraryResampler::new(0.5);
        resampler.process(&vec![Complex64::new(1.0, 0.0); 50]);
        assert!(resampler.consumed() > 0);
        resampler.reset();
        assert_eq!(resampler.consumed(), 0);
        assert_eq!(resampler.produced(), 0);
    }

    #[test]
    fn test_irrational_ratio() {
        // sqrt(2) ≈ 1.4142... (irrational)
        let mut resampler = ArbitraryResampler::new(std::f64::consts::SQRT_2);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = resampler.process(&input);
        assert!(output.len() >= 130 && output.len() <= 150,
            "Expected ~141, got {}", output.len());
    }
}
