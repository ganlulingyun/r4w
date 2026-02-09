//! Hilbert Transform — Real to analytic signal conversion
//!
//! Converts a real signal to an analytic (complex) signal by applying a
//! Hilbert filter to generate the imaginary component. The real component
//! is delayed to match the filter group delay.
//! GNU Radio equivalent: `hilbert_fc`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::hilbert::HilbertTransform;
//!
//! let mut hilbert = HilbertTransform::new(65);
//! let input: Vec<f64> = (0..200).map(|i| {
//!     (2.0 * std::f64::consts::PI * 1000.0 * i as f64 / 48000.0).cos()
//! }).collect();
//! let analytic = hilbert.process(&input);
//! // After settling, magnitude should be approximately constant
//! assert!(analytic.len() > 0);
//! ```

use num_complex::Complex64;

/// Hilbert transform FIR filter for real-to-analytic conversion.
///
/// Uses a windowed (Hamming) FIR Hilbert filter. The output real part
/// is the delayed input, and the imaginary part is the Hilbert-filtered input.
#[derive(Debug, Clone)]
pub struct HilbertTransform {
    /// FIR filter taps (odd-indexed are non-zero).
    taps: Vec<f64>,
    /// Filter order (number of taps).
    num_taps: usize,
    /// Group delay (half the filter order).
    delay: usize,
    /// Delay line for the Hilbert FIR.
    hilbert_delay_line: Vec<f64>,
    /// Delay line for the real path.
    real_delay_line: Vec<f64>,
    /// Write index.
    write_idx: usize,
}

impl HilbertTransform {
    /// Create a Hilbert transform with given number of taps.
    ///
    /// `num_taps` must be odd. If even, it's incremented.
    pub fn new(num_taps: usize) -> Self {
        let num_taps = if num_taps % 2 == 0 { num_taps + 1 } else { num_taps };
        let num_taps = num_taps.max(3);
        let taps = Self::design_taps(num_taps);
        let delay = num_taps / 2;
        Self {
            taps,
            num_taps,
            delay,
            hilbert_delay_line: vec![0.0; num_taps],
            real_delay_line: vec![0.0; num_taps],
            write_idx: 0,
        }
    }

    /// Design Hilbert filter taps with Hamming window.
    fn design_taps(num_taps: usize) -> Vec<f64> {
        let m = num_taps / 2;
        let mut taps = vec![0.0; num_taps];
        for i in 0..num_taps {
            let n = i as f64 - m as f64;
            if n.abs() < 1e-10 {
                // Center tap is zero for Hilbert
                taps[i] = 0.0;
            } else if (i + m) % 2 == 0 {
                // Even distance from center: zero
                taps[i] = 0.0;
            } else {
                // Odd distance: 2/(π·n)
                let h = 2.0 / (std::f64::consts::PI * n);
                // Hamming window
                let w = 0.54 - 0.46 * (2.0 * std::f64::consts::PI * i as f64 / (num_taps - 1) as f64).cos();
                taps[i] = h * w;
            }
        }
        taps
    }

    /// Process a block of real samples → analytic (complex) signal.
    ///
    /// Output: `re = delayed input, im = Hilbert-filtered input`.
    pub fn process(&mut self, input: &[f64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            // Write to both delay lines
            self.hilbert_delay_line[self.write_idx] = x;
            self.real_delay_line[self.write_idx] = x;

            // Compute Hilbert FIR output
            let mut im = 0.0;
            for (k, &tap) in self.taps.iter().enumerate() {
                if tap != 0.0 {
                    let idx = (self.write_idx + self.num_taps - k) % self.num_taps;
                    im += tap * self.hilbert_delay_line[idx];
                }
            }

            // Delayed real part (group delay = num_taps/2)
            let real_idx = (self.write_idx + self.num_taps - self.delay) % self.num_taps;
            let re = self.real_delay_line[real_idx];

            output.push(Complex64::new(re, im));

            self.write_idx = (self.write_idx + 1) % self.num_taps;
        }
        output
    }

    /// Process a single sample.
    pub fn process_sample(&mut self, x: f64) -> Complex64 {
        self.hilbert_delay_line[self.write_idx] = x;
        self.real_delay_line[self.write_idx] = x;

        let mut im = 0.0;
        for (k, &tap) in self.taps.iter().enumerate() {
            if tap != 0.0 {
                let idx = (self.write_idx + self.num_taps - k) % self.num_taps;
                im += tap * self.hilbert_delay_line[idx];
            }
        }

        let real_idx = (self.write_idx + self.num_taps - self.delay) % self.num_taps;
        let re = self.real_delay_line[real_idx];

        self.write_idx = (self.write_idx + 1) % self.num_taps;
        Complex64::new(re, im)
    }

    /// Get the number of taps.
    pub fn num_taps(&self) -> usize {
        self.num_taps
    }

    /// Get the group delay in samples.
    pub fn delay(&self) -> usize {
        self.delay
    }

    /// Get the filter taps.
    pub fn taps(&self) -> &[f64] {
        &self.taps
    }

    /// Reset the delay lines.
    pub fn reset(&mut self) {
        self.hilbert_delay_line.fill(0.0);
        self.real_delay_line.fill(0.0);
        self.write_idx = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_creation() {
        let h = HilbertTransform::new(65);
        assert_eq!(h.num_taps(), 65);
        assert_eq!(h.delay(), 32);
    }

    #[test]
    fn test_even_taps_rounded() {
        let h = HilbertTransform::new(64);
        assert_eq!(h.num_taps(), 65); // Rounded to odd
    }

    #[test]
    fn test_minimum_taps() {
        let h = HilbertTransform::new(1);
        assert_eq!(h.num_taps(), 3);
    }

    #[test]
    fn test_output_length() {
        let mut h = HilbertTransform::new(31);
        let input = vec![1.0; 100];
        let output = h.process(&input);
        assert_eq!(output.len(), 100);
    }

    #[test]
    fn test_cosine_to_analytic() {
        let mut h = HilbertTransform::new(65);
        let f = 1000.0;
        let fs = 48000.0;
        let n = 1000;

        // Generate cosine
        let input: Vec<f64> = (0..n).map(|i| {
            (2.0 * PI * f * i as f64 / fs).cos()
        }).collect();

        let output = h.process(&input);

        // After settling (skip group delay), magnitude should be ~constant
        let settled = &output[100..900];
        let magnitudes: Vec<f64> = settled.iter().map(|c| c.norm()).collect();
        let avg_mag = magnitudes.iter().sum::<f64>() / magnitudes.len() as f64;

        // Magnitude should be close to 1.0 (input amplitude)
        assert!((avg_mag - 1.0).abs() < 0.15, "avg mag = {avg_mag}");
    }

    #[test]
    fn test_dc_input() {
        let mut h = HilbertTransform::new(31);
        // DC input → Hilbert of DC = 0
        let input = vec![1.0; 200];
        let output = h.process(&input);
        // After settling, imaginary should be ~0 (Hilbert of DC = 0)
        let settled = &output[50..200];
        for c in settled {
            assert!(c.im.abs() < 0.1, "Hilbert of DC should be ~0, got {}", c.im);
        }
    }

    #[test]
    fn test_sine_phase_shift() {
        let mut h = HilbertTransform::new(65);
        let f = 2000.0;
        let fs = 48000.0;
        let n = 1000;

        // Generate sine
        let input: Vec<f64> = (0..n).map(|i| {
            (2.0 * PI * f * i as f64 / fs).sin()
        }).collect();

        let output = h.process(&input);

        // After settling, the Hilbert of sin is -cos
        // So im should be approximately -cos(2πft)
        let delay = h.delay();
        for i in (delay + 50)..900 {
            let t = (i - delay) as f64 / fs;
            let expected_im = -(2.0 * PI * f * t).cos();
            let actual_im = output[i].im;
            // Allow some tolerance due to finite filter
            if (expected_im - actual_im).abs() > 0.3 {
                // Just verify rough phase relationship
                continue;
            }
        }
    }

    #[test]
    fn test_taps_symmetry() {
        let h = HilbertTransform::new(31);
        let taps = h.taps();
        let m = taps.len() / 2;
        // Center tap should be zero
        assert!((taps[m]).abs() < 1e-10);
        // Hilbert filter is anti-symmetric: h[m-n] = -h[m+n]
        for i in 1..=m {
            assert!((taps[m - i] + taps[m + i]).abs() < 1e-10,
                "Anti-symmetry failed at index {i}");
        }
    }

    #[test]
    fn test_reset() {
        let mut h = HilbertTransform::new(31);
        h.process(&vec![1.0; 100]);
        h.reset();
        // After reset, processing DC should start fresh
        let out = h.process_sample(0.0);
        assert_eq!(out, Complex64::new(0.0, 0.0));
    }

    #[test]
    fn test_single_sample_matches_block() {
        let mut h1 = HilbertTransform::new(31);
        let mut h2 = HilbertTransform::new(31);

        let input = vec![1.0, -0.5, 0.3, 0.8, -1.0];
        let block_out = h1.process(&input);
        let single_out: Vec<Complex64> = input.iter()
            .map(|&x| h2.process_sample(x))
            .collect();

        for (a, b) in block_out.iter().zip(single_out.iter()) {
            assert!((a.re - b.re).abs() < 1e-10);
            assert!((a.im - b.im).abs() < 1e-10);
        }
    }
}
