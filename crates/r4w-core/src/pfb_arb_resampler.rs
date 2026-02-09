//! PFB Arbitrary Resampler — Polyphase Filterbank Arbitrary Resampler
//!
//! High-quality arbitrary sample rate conversion using a polyphase
//! filterbank with linear interpolation between filter branches.
//! Superior to simple fractional resampler for non-rational rate
//! conversions. Each output sample selects two adjacent filters and
//! interpolates, giving smooth rate control.
//! GNU Radio equivalent: `gr::filter::pfb_arb_resampler_ccf`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::pfb_arb_resampler::PfbArbResampler;
//! use num_complex::Complex64;
//!
//! let mut resampler = PfbArbResampler::new(1.5, 32, 8);
//! let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
//! let output = resampler.process(&input);
//! // Output should be ~150 samples (1.5x rate)
//! assert!((output.len() as f64 - 150.0).abs() < 5.0);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Polyphase filterbank arbitrary resampler.
#[derive(Debug, Clone)]
pub struct PfbArbResampler {
    /// Resampling rate (output_rate / input_rate).
    rate: f64,
    /// Number of polyphase filter branches.
    num_filters: usize,
    /// Taps per filter branch.
    taps_per_filter: usize,
    /// Polyphase filter branches (num_filters x taps_per_filter).
    branches: Vec<Vec<f64>>,
    /// Derivative filter branches for interpolation.
    diff_branches: Vec<Vec<f64>>,
    /// Delay line.
    delay_line: Vec<Complex64>,
    /// Fractional filter index accumulator.
    frac_index: f64,
    /// Input sample counter.
    input_idx: usize,
}

impl PfbArbResampler {
    /// Create a new PFB arbitrary resampler.
    ///
    /// `rate`: resampling rate (output_rate / input_rate).
    /// `num_filters`: number of polyphase branches (32 typical).
    /// `taps_per_filter`: taps per branch (8 typical).
    pub fn new(rate: f64, num_filters: usize, taps_per_filter: usize) -> Self {
        assert!(rate > 0.0, "Rate must be positive");
        assert!(num_filters > 0, "Must have at least 1 filter");
        assert!(taps_per_filter > 0, "Must have at least 1 tap per filter");

        let total_taps = num_filters * taps_per_filter;
        let prototype = design_prototype(total_taps, num_filters);

        // Decompose into polyphase branches
        let mut branches = vec![vec![0.0; taps_per_filter]; num_filters];
        for (i, &tap) in prototype.iter().enumerate() {
            let branch = i % num_filters;
            let tap_idx = i / num_filters;
            if tap_idx < taps_per_filter {
                branches[branch][tap_idx] = tap;
            }
        }

        // Compute derivative filters for linear interpolation between branches
        let mut diff_branches = vec![vec![0.0; taps_per_filter]; num_filters];
        for i in 0..num_filters {
            let next = (i + 1) % num_filters;
            for j in 0..taps_per_filter {
                diff_branches[i][j] = branches[next][j] - branches[i][j];
            }
        }

        Self {
            rate,
            num_filters,
            taps_per_filter,
            branches,
            diff_branches,
            delay_line: vec![Complex64::new(0.0, 0.0); taps_per_filter],
            frac_index: 0.0,
            input_idx: 0,
        }
    }

    /// Set resampling rate at runtime.
    pub fn set_rate(&mut self, rate: f64) {
        assert!(rate > 0.0, "Rate must be positive");
        self.rate = rate;
    }

    /// Get current resampling rate.
    pub fn rate(&self) -> f64 {
        self.rate
    }

    /// Process input samples and produce resampled output.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let estimated_output = (input.len() as f64 * self.rate).ceil() as usize + 2;
        let mut output = Vec::with_capacity(estimated_output);

        let step = self.num_filters as f64 / self.rate;
        let mut in_idx = 0usize;

        while in_idx < input.len() {
            // Push input sample into delay line
            self.delay_line.rotate_right(1);
            self.delay_line[0] = input[in_idx];

            // Generate outputs while filter index is within current input
            while self.frac_index < self.num_filters as f64 {
                let filter_idx = self.frac_index as usize;
                let mu = self.frac_index - filter_idx as f64;
                let branch_idx = filter_idx % self.num_filters;

                // Compute filter output with linear interpolation
                let mut sum = Complex64::new(0.0, 0.0);
                for k in 0..self.taps_per_filter.min(self.delay_line.len()) {
                    let coeff = self.branches[branch_idx][k]
                        + mu * self.diff_branches[branch_idx][k];
                    sum += self.delay_line[k] * coeff;
                }

                output.push(sum);
                self.frac_index += step;
            }

            self.frac_index -= self.num_filters as f64;
            in_idx += 1;
        }

        output
    }

    /// Process real-valued samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let complex_input: Vec<Complex64> = input
            .iter()
            .map(|&x| Complex64::new(x, 0.0))
            .collect();
        self.process(&complex_input)
            .iter()
            .map(|c| c.re)
            .collect()
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.delay_line = vec![Complex64::new(0.0, 0.0); self.taps_per_filter];
        self.frac_index = 0.0;
        self.input_idx = 0;
    }

    /// Get number of filter branches.
    pub fn num_filters(&self) -> usize {
        self.num_filters
    }

    /// Get taps per filter.
    pub fn taps_per_filter(&self) -> usize {
        self.taps_per_filter
    }
}

/// Design lowpass prototype filter for polyphase decomposition.
pub fn design_prototype(num_taps: usize, num_filters: usize) -> Vec<f64> {
    let cutoff = 1.0 / num_filters as f64;
    let m = num_taps as f64;
    let mid = (num_taps - 1) as f64 / 2.0;

    let mut taps = Vec::with_capacity(num_taps);
    for i in 0..num_taps {
        let n = i as f64 - mid;
        // Sinc function
        let sinc = if n.abs() < 1e-10 {
            1.0
        } else {
            (PI * cutoff * n).sin() / (PI * cutoff * n)
        };
        // Blackman-Harris window
        let w = 0.35875
            - 0.48829 * (2.0 * PI * i as f64 / (m - 1.0)).cos()
            + 0.14128 * (4.0 * PI * i as f64 / (m - 1.0)).cos()
            - 0.01168 * (6.0 * PI * i as f64 / (m - 1.0)).cos();
        taps.push(sinc * w);
    }

    // Normalize for unity DC gain across polyphase branches
    let sum: f64 = taps.iter().sum();
    if sum.abs() > 1e-10 {
        let scale = num_filters as f64 / sum;
        for t in taps.iter_mut() {
            *t *= scale;
        }
    }

    taps
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_creation() {
        let resampler = PfbArbResampler::new(2.0, 32, 8);
        assert_eq!(resampler.rate(), 2.0);
        assert_eq!(resampler.num_filters(), 32);
        assert_eq!(resampler.taps_per_filter(), 8);
    }

    #[test]
    fn test_upsample_2x() {
        let mut resampler = PfbArbResampler::new(2.0, 32, 8);
        let input: Vec<Complex64> = (0..100)
            .map(|i| Complex64::new((2.0 * PI * 0.01 * i as f64).cos(), 0.0))
            .collect();
        let output = resampler.process(&input);
        // Should produce ~200 output samples
        assert!(
            (output.len() as f64 - 200.0).abs() < 10.0,
            "Expected ~200 samples, got {}",
            output.len()
        );
    }

    #[test]
    fn test_downsample_half() {
        let mut resampler = PfbArbResampler::new(0.5, 32, 8);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 200];
        let output = resampler.process(&input);
        assert!(
            (output.len() as f64 - 100.0).abs() < 10.0,
            "Expected ~100 samples, got {}",
            output.len()
        );
    }

    #[test]
    fn test_unity_rate() {
        let mut resampler = PfbArbResampler::new(1.0, 32, 8);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 50];
        let output = resampler.process(&input);
        assert!(
            (output.len() as f64 - 50.0).abs() < 5.0,
            "Unity rate should produce ~same number of samples: got {}",
            output.len()
        );
    }

    #[test]
    fn test_fractional_rate() {
        let mut resampler = PfbArbResampler::new(1.5, 32, 8);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
        let output = resampler.process(&input);
        assert!(
            (output.len() as f64 - 150.0).abs() < 10.0,
            "1.5x rate: expected ~150, got {}",
            output.len()
        );
    }

    #[test]
    fn test_set_rate() {
        let mut resampler = PfbArbResampler::new(1.0, 32, 8);
        resampler.set_rate(2.5);
        assert_eq!(resampler.rate(), 2.5);
    }

    #[test]
    fn test_process_real() {
        let mut resampler = PfbArbResampler::new(2.0, 32, 8);
        let input: Vec<f64> = (0..50).map(|i| (0.1 * i as f64).sin()).collect();
        let output = resampler.process_real(&input);
        assert!(output.len() > 80); // Should be ~100
    }

    #[test]
    fn test_reset() {
        let mut resampler = PfbArbResampler::new(1.5, 32, 8);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 50];
        let _ = resampler.process(&input);
        resampler.reset();
        assert_eq!(resampler.frac_index, 0.0);
    }

    #[test]
    fn test_prototype_filter() {
        let taps = design_prototype(256, 32);
        assert_eq!(taps.len(), 256);
        // Taps should be symmetric
        let n = taps.len();
        for i in 0..n / 2 {
            assert!(
                (taps[i] - taps[n - 1 - i]).abs() < 1e-10,
                "Prototype should be symmetric"
            );
        }
    }

    #[test]
    fn test_dc_passthrough() {
        let mut resampler = PfbArbResampler::new(1.0, 32, 8);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
        let output = resampler.process(&input);
        // After settling, DC should pass through
        if output.len() > 20 {
            let steady = &output[10..];
            let avg = steady.iter().map(|s| s.re).sum::<f64>() / steady.len() as f64;
            assert!(
                (avg - 1.0).abs() < 0.3,
                "DC should pass through: avg = {}",
                avg
            );
        }
    }
}
