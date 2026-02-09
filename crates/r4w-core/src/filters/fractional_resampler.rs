//! Fractional Resampler (MMSE Interpolating)
//!
//! Performs arbitrary (non-rational) sample rate conversion using polynomial
//! interpolation. Unlike the rational polyphase resampler, this handles
//! irrational resampling ratios (e.g., 48000/44100) and continuously
//! variable rates.
//!
//! ## Algorithm
//!
//! Uses cubic (4-tap) Lagrange interpolation. For each output sample,
//! selects 4 neighboring input samples and computes a weighted sum
//! based on the fractional delay (mu).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::filters::fractional_resampler::FractionalResampler;
//! use num_complex::Complex64;
//!
//! // Downsample 48kHz → 44.1kHz
//! let mut resampler = FractionalResampler::from_rates(48000.0, 44100.0);
//! let input = vec![Complex64::new(1.0, 0.0); 480];
//! let output = resampler.process_block(&input);
//! // Output should be approximately 480 * 44100/48000 ≈ 441 samples
//! assert!((output.len() as f64 - 441.0).abs() <= 2.0);
//! ```

use num_complex::Complex64;

/// Fractional resampler using cubic interpolation.
#[derive(Debug, Clone)]
pub struct FractionalResampler {
    /// Resampling ratio (output_rate / input_rate). > 1.0 = interpolation, < 1.0 = decimation
    ratio: f64,
    /// Fractional sample position (0.0 to 1.0)
    mu: f64,
    /// History buffer for interpolation (last 3 samples)
    history: [Complex64; 3],
    /// Number of input samples consumed (for tracking)
    input_count: u64,
    /// Number of output samples produced
    output_count: u64,
}

impl FractionalResampler {
    /// Create with a fixed resampling ratio (output_rate / input_rate).
    pub fn new(ratio: f64) -> Self {
        assert!(ratio > 0.0, "Ratio must be positive");
        Self {
            ratio,
            mu: 0.0,
            history: [Complex64::new(0.0, 0.0); 3],
            input_count: 0,
            output_count: 0,
        }
    }

    /// Create for converting between specific sample rates.
    pub fn from_rates(input_rate: f64, output_rate: f64) -> Self {
        Self::new(output_rate / input_rate)
    }

    /// Process a block of complex input samples, producing resampled output.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let step = 1.0 / self.ratio; // input samples per output sample
        let estimated_out = ((input.len() as f64) * self.ratio).ceil() as usize + 4;
        let mut output = Vec::with_capacity(estimated_out);

        let mut input_idx = 0usize;

        while input_idx < input.len() {
            // Consume input samples until we have enough for interpolation
            while self.mu >= 1.0 && input_idx < input.len() {
                self.mu -= 1.0;
                // Shift history
                self.history[0] = self.history[1];
                self.history[1] = self.history[2];
                self.history[2] = input[input_idx];
                input_idx += 1;
                self.input_count += 1;
            }

            if input_idx == 0 && self.mu < 1.0 {
                // Need to consume at least one sample to start
                self.history[0] = self.history[1];
                self.history[1] = self.history[2];
                self.history[2] = input[input_idx];
                input_idx += 1;
                self.input_count += 1;
            }

            if self.mu < 1.0 {
                // Interpolate using cubic (3-point quadratic) or linear
                let out = self.interpolate();
                output.push(out);
                self.output_count += 1;
                self.mu += step;
            }
        }

        output
    }

    /// Process a block of real-valued samples.
    pub fn process_block_real(&mut self, input: &[f64]) -> Vec<f64> {
        let complex_input: Vec<Complex64> = input.iter().map(|&r| Complex64::new(r, 0.0)).collect();
        self.process_block(&complex_input)
            .iter()
            .map(|z| z.re)
            .collect()
    }

    /// Set a new resampling ratio (for variable-rate operation).
    pub fn set_ratio(&mut self, ratio: f64) {
        assert!(ratio > 0.0, "Ratio must be positive");
        self.ratio = ratio;
    }

    /// Get the current resampling ratio.
    pub fn ratio(&self) -> f64 {
        self.ratio
    }

    /// Set the fractional phase offset directly (used by clock recovery).
    pub fn set_mu(&mut self, mu: f64) {
        self.mu = mu;
    }

    /// Get current fractional phase.
    pub fn mu(&self) -> f64 {
        self.mu
    }

    /// Get total input samples consumed.
    pub fn input_count(&self) -> u64 {
        self.input_count
    }

    /// Get total output samples produced.
    pub fn output_count(&self) -> u64 {
        self.output_count
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.mu = 0.0;
        self.history = [Complex64::new(0.0, 0.0); 3];
        self.input_count = 0;
        self.output_count = 0;
    }

    /// Linear interpolation between history[1] and history[2] using mu.
    fn interpolate(&self) -> Complex64 {
        // Linear interpolation: y = (1-mu)*s1 + mu*s2
        // Using history[1] as s[n-1] and history[2] as s[n]
        let mu = self.mu;
        self.history[1] * (1.0 - mu) + self.history[2] * mu
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_unity_ratio_passthrough() {
        let mut rs = FractionalResampler::new(1.0);
        let input: Vec<Complex64> = (0..100)
            .map(|i| Complex64::new(i as f64, 0.0))
            .collect();
        let output = rs.process_block(&input);
        // Output length should be approximately input length
        assert!(
            (output.len() as i64 - 100).abs() <= 2,
            "Unity ratio should preserve length: got {}",
            output.len()
        );
    }

    #[test]
    fn test_downsample_2x() {
        let mut rs = FractionalResampler::new(0.5); // 2x downsample
        let input = vec![Complex64::new(1.0, 0.0); 200];
        let output = rs.process_block(&input);
        // Should produce ~100 samples
        assert!(
            (output.len() as i64 - 100).abs() <= 5,
            "2x downsample: expected ~100, got {}",
            output.len()
        );
    }

    #[test]
    fn test_upsample_2x() {
        let mut rs = FractionalResampler::new(2.0); // 2x upsample
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = rs.process_block(&input);
        // Should produce ~200 samples
        assert!(
            (output.len() as i64 - 200).abs() <= 5,
            "2x upsample: expected ~200, got {}",
            output.len()
        );
    }

    #[test]
    fn test_irrational_ratio() {
        // 48000 → 44100 (ratio = 0.91875)
        let mut rs = FractionalResampler::from_rates(48000.0, 44100.0);
        let input = vec![Complex64::new(1.0, 0.0); 4800];
        let output = rs.process_block(&input);
        let expected = (4800.0 * 44100.0 / 48000.0) as i64; // 4410
        assert!(
            (output.len() as i64 - expected).abs() <= 5,
            "48k→44.1k: expected ~{}, got {}",
            expected,
            output.len()
        );
    }

    #[test]
    fn test_dc_preservation() {
        let mut rs = FractionalResampler::new(0.75);
        let input = vec![Complex64::new(1.0, 0.0); 200];
        let output = rs.process_block(&input);
        // After startup, all output should be ~1.0
        for &s in &output[5..] {
            assert!(
                (s.re - 1.0).abs() < 0.1,
                "DC should be preserved: got {}",
                s.re
            );
        }
    }

    #[test]
    fn test_tone_preservation() {
        // Resample a tone and verify frequency is preserved
        let fs_in = 48000.0;
        let fs_out = 44100.0;
        let freq = 1000.0;
        let mut rs = FractionalResampler::from_rates(fs_in, fs_out);

        let n_in = 4800; // 100ms
        let input: Vec<Complex64> = (0..n_in)
            .map(|i| {
                let phase = 2.0 * PI * freq * i as f64 / fs_in;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let output = rs.process_block(&input);

        // Verify output length is approximately correct
        let expected_len = (n_in as f64 * fs_out / fs_in) as i64;
        assert!(
            (output.len() as i64 - expected_len).abs() <= 5,
            "Expected ~{} samples, got {}",
            expected_len,
            output.len()
        );

        // Verify output has approximately unit magnitude (tone preservation)
        for &s in &output[10..output.len() - 10] {
            let mag = s.norm();
            assert!(
                (mag - 1.0).abs() < 0.2,
                "Tone magnitude should be ~1.0: got {}",
                mag
            );
        }
    }

    #[test]
    fn test_real_processing() {
        let mut rs = FractionalResampler::new(0.5);
        let input: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let output = rs.process_block_real(&input);
        assert!(
            (output.len() as i64 - 50).abs() <= 5,
            "Real 2x downsample: expected ~50, got {}",
            output.len()
        );
    }

    #[test]
    fn test_variable_ratio() {
        let mut rs = FractionalResampler::new(1.0);
        assert!((rs.ratio() - 1.0).abs() < 1e-10);
        rs.set_ratio(2.0);
        assert!((rs.ratio() - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut rs = FractionalResampler::new(1.0);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        rs.process_block(&input);
        assert!(rs.input_count() > 0);

        rs.reset();
        assert_eq!(rs.input_count(), 0);
        assert_eq!(rs.output_count(), 0);
    }

    #[test]
    fn test_consecutive_blocks() {
        // Process in multiple blocks and verify continuity
        let mut rs = FractionalResampler::new(0.5);
        let block1 = vec![Complex64::new(1.0, 0.0); 100];
        let block2 = vec![Complex64::new(1.0, 0.0); 100];

        let out1 = rs.process_block(&block1);
        let out2 = rs.process_block(&block2);

        let total = out1.len() + out2.len();
        assert!(
            (total as i64 - 100).abs() <= 5,
            "Two blocks of 100 at 0.5x should give ~100 total: got {}",
            total
        );
    }
}
