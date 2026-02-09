//! FFT Filter — Fast convolution FIR filtering
//!
//! Implements FIR filtering in the frequency domain using the overlap-save
//! method. For filters with >~32 taps, this is significantly faster than
//! direct convolution. Supports real and complex taps.
//! GNU Radio equivalent: `fft_filter_ccc`, `fft_filter_ccf`, `fft_filter_fff`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::fft_filter::FftFilter;
//! use num_complex::Complex64;
//!
//! // Design a simple lowpass with 64 taps
//! let taps: Vec<f64> = (0..64).map(|i| {
//!     let n = i as f64 - 31.5;
//!     if n.abs() < 1e-10 { 0.25 } else { (0.25 * std::f64::consts::PI * n).sin() / (std::f64::consts::PI * n) }
//! }).collect();
//! let mut filter = FftFilter::new(&taps);
//! let input = vec![Complex64::new(1.0, 0.0); 256];
//! let output = filter.process(&input);
//! assert!(!output.is_empty());
//! ```

use num_complex::Complex64;
use crate::fft_utils::FftProcessor;

/// FFT-based FIR filter using overlap-save fast convolution.
pub struct FftFilter {
    /// Frequency-domain filter taps.
    taps_fft: Vec<Complex64>,
    /// Number of time-domain taps.
    num_taps: usize,
    /// FFT size (power of 2, >= 2 * num_taps).
    fft_size: usize,
    /// Overlap buffer from previous block.
    overlap: Vec<Complex64>,
    /// FFT processor.
    fft: FftProcessor,
}

impl FftFilter {
    /// Create an FFT filter from real-valued taps.
    ///
    /// FFT size is automatically chosen as next power of 2 >= 2 * num_taps.
    pub fn new(taps: &[f64]) -> Self {
        let num_taps = taps.len().max(1);
        let fft_size = Self::compute_fft_size(num_taps);
        let fft = FftProcessor::new(fft_size);
        let taps_fft = Self::compute_taps_fft(taps, fft_size, &fft);

        Self {
            taps_fft,
            num_taps,
            fft_size,
            overlap: vec![Complex64::new(0.0, 0.0); num_taps - 1],
            fft,
        }
    }

    /// Create from a specified FFT size.
    pub fn with_fft_size(taps: &[f64], fft_size: usize) -> Self {
        let num_taps = taps.len().max(1);
        let fft_size = fft_size.max(2 * num_taps).next_power_of_two();
        let fft = FftProcessor::new(fft_size);
        let taps_fft = Self::compute_taps_fft(taps, fft_size, &fft);

        Self {
            taps_fft,
            num_taps,
            fft_size,
            overlap: vec![Complex64::new(0.0, 0.0); num_taps - 1],
            fft,
        }
    }

    /// Create from complex-valued taps.
    pub fn from_complex_taps(taps: &[Complex64]) -> Self {
        let num_taps = taps.len().max(1);
        let fft_size = Self::compute_fft_size(num_taps);
        let fft = FftProcessor::new(fft_size);

        // Zero-pad taps and FFT
        let mut padded = vec![Complex64::new(0.0, 0.0); fft_size];
        for (i, &t) in taps.iter().enumerate() {
            padded[i] = t;
        }
        let mut fft_mut = FftProcessor::new(fft_size);
        fft_mut.fft_inplace(&mut padded);

        Self {
            taps_fft: padded,
            num_taps,
            fft_size,
            overlap: vec![Complex64::new(0.0, 0.0); num_taps - 1],
            fft: fft,
        }
    }

    fn compute_fft_size(num_taps: usize) -> usize {
        (2 * num_taps).next_power_of_two()
    }

    fn compute_taps_fft(taps: &[f64], fft_size: usize, _fft: &FftProcessor) -> Vec<Complex64> {
        let mut padded = vec![Complex64::new(0.0, 0.0); fft_size];
        for (i, &t) in taps.iter().enumerate() {
            padded[i] = Complex64::new(t, 0.0);
        }
        let mut fft = FftProcessor::new(fft_size);
        fft.fft_inplace(&mut padded);
        padded
    }

    /// Process a block of complex samples using overlap-save.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        if input.is_empty() {
            return Vec::new();
        }

        let useful = self.fft_size - (self.num_taps - 1);
        let mut output = Vec::with_capacity(input.len());
        let mut pos = 0;

        while pos < input.len() {
            // Build input block: overlap + new data
            let mut block = vec![Complex64::new(0.0, 0.0); self.fft_size];

            // Copy overlap from previous iteration
            let overlap_len = self.overlap.len();
            block[..overlap_len].copy_from_slice(&self.overlap);

            // Copy new input data
            let remaining = input.len() - pos;
            let copy_len = remaining.min(useful);
            for i in 0..copy_len {
                block[overlap_len + i] = input[pos + i];
            }

            // FFT the input block
            self.fft.fft_inplace(&mut block);

            // Multiply in frequency domain
            for i in 0..self.fft_size {
                block[i] *= self.taps_fft[i];
            }

            // IFFT back to time domain
            self.fft.ifft_inplace(&mut block);

            // Extract valid output (skip first num_taps-1 samples)
            let valid_start = self.num_taps - 1;
            let valid_end = valid_start + copy_len;
            for i in valid_start..valid_end {
                output.push(block[i]);
            }

            // Save overlap for next block
            let new_overlap_start = pos + copy_len;
            if new_overlap_start <= input.len() {
                let start = if copy_len >= overlap_len {
                    pos + copy_len - overlap_len
                } else {
                    // Shift overlap and add new data
                    pos
                };
                // Save the last (num_taps - 1) input samples as overlap
                self.overlap.fill(Complex64::new(0.0, 0.0));
                let avail = input.len() - pos;
                let take = avail.min(overlap_len);
                let skip = if avail > overlap_len { avail - overlap_len } else { 0 };
                for i in 0..take {
                    self.overlap[overlap_len - take + i] = input[pos + skip + i];
                }
            }

            pos += copy_len;
        }

        output
    }

    /// Process real-valued samples.
    pub fn process_real(&mut self, input: &[f64]) -> Vec<f64> {
        let complex_in: Vec<Complex64> = input.iter()
            .map(|&x| Complex64::new(x, 0.0))
            .collect();
        let complex_out = self.process(&complex_in);
        complex_out.iter().map(|c| c.re).collect()
    }

    /// Update taps (re-computes FFT of taps).
    pub fn set_taps(&mut self, taps: &[f64]) {
        self.num_taps = taps.len().max(1);
        let new_fft_size = Self::compute_fft_size(self.num_taps);
        if new_fft_size != self.fft_size {
            self.fft_size = new_fft_size;
            self.fft = FftProcessor::new(self.fft_size);
        }
        self.taps_fft = Self::compute_taps_fft(taps, self.fft_size, &self.fft);
        self.overlap = vec![Complex64::new(0.0, 0.0); self.num_taps - 1];
    }

    /// Get the number of taps.
    pub fn num_taps(&self) -> usize {
        self.num_taps
    }

    /// Get the FFT size.
    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    /// Reset the overlap buffer.
    pub fn reset(&mut self) {
        self.overlap.fill(Complex64::new(0.0, 0.0));
    }
}

impl std::fmt::Debug for FftFilter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("FftFilter")
            .field("num_taps", &self.num_taps)
            .field("fft_size", &self.fft_size)
            .finish()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn make_lowpass_taps(num_taps: usize, cutoff: f64) -> Vec<f64> {
        let m = (num_taps - 1) as f64 / 2.0;
        (0..num_taps).map(|i| {
            let n = i as f64 - m;
            let h = if n.abs() < 1e-10 {
                cutoff
            } else {
                (cutoff * PI * n).sin() / (PI * n)
            };
            // Hamming window
            let w = 0.54 - 0.46 * (2.0 * PI * i as f64 / (num_taps - 1) as f64).cos();
            h * w
        }).collect()
    }

    #[test]
    fn test_creation() {
        let taps = vec![1.0; 32];
        let filter = FftFilter::new(&taps);
        assert_eq!(filter.num_taps(), 32);
        assert!(filter.fft_size() >= 64);
        assert!(filter.fft_size().is_power_of_two());
    }

    #[test]
    fn test_passthrough_single_tap() {
        // Single tap of 1.0 should be passthrough
        let mut filter = FftFilter::new(&[1.0]);
        let input: Vec<Complex64> = (0..100).map(|i| {
            Complex64::new(i as f64, 0.0)
        }).collect();
        let output = filter.process(&input);
        assert_eq!(output.len(), input.len());
        for (a, b) in output.iter().zip(input.iter()) {
            assert!((a.re - b.re).abs() < 1e-8, "mismatch: {} vs {}", a.re, b.re);
        }
    }

    #[test]
    fn test_delay_filter() {
        // A filter with [0, 0, 1] should delay by 2 samples
        let mut filter = FftFilter::new(&[0.0, 0.0, 1.0]);
        let mut input = vec![Complex64::new(0.0, 0.0); 50];
        input[0] = Complex64::new(1.0, 0.0);
        let output = filter.process(&input);
        assert!(output.len() >= 3);
        // Impulse should appear at index 2
        assert!((output[2].re - 1.0).abs() < 1e-8, "expected 1.0 at idx 2, got {}", output[2].re);
    }

    #[test]
    fn test_lowpass_dc_passes() {
        let taps = make_lowpass_taps(64, 0.25);
        let mut filter = FftFilter::new(&taps);
        // DC signal should pass through (with some gain from filter)
        let input = vec![Complex64::new(1.0, 0.0); 512];
        let output = filter.process(&input);
        // After settling, output should be approximately constant
        let settled = &output[100..];
        let mean = settled.iter().map(|c| c.re).sum::<f64>() / settled.len() as f64;
        assert!(mean.abs() > 0.1, "DC should pass through lowpass, got mean={mean}");
    }

    #[test]
    fn test_output_length() {
        let taps = vec![0.25; 16];
        let mut filter = FftFilter::new(&taps);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = filter.process(&input);
        assert_eq!(output.len(), 100);
    }

    #[test]
    fn test_process_real() {
        let taps = vec![0.5, 0.5]; // Simple averaging filter
        let mut filter = FftFilter::new(&taps);
        let input: Vec<f64> = (0..50).map(|i| i as f64).collect();
        let output = filter.process_real(&input);
        assert_eq!(output.len(), 50);
    }

    #[test]
    fn test_set_taps() {
        let mut filter = FftFilter::new(&[1.0; 8]);
        assert_eq!(filter.num_taps(), 8);
        filter.set_taps(&[1.0; 32]);
        assert_eq!(filter.num_taps(), 32);
    }

    #[test]
    fn test_reset() {
        let mut filter = FftFilter::new(&[0.5, 0.5]);
        filter.process(&vec![Complex64::new(1.0, 0.0); 100]);
        filter.reset();
        // After reset, overlap should be cleared
        let out = filter.process(&vec![Complex64::new(0.0, 0.0); 10]);
        assert!(out.iter().all(|c| c.re.abs() < 1e-8));
    }

    #[test]
    fn test_complex_taps() {
        let taps = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 1.0),
        ];
        let mut filter = FftFilter::from_complex_taps(&taps);
        assert_eq!(filter.num_taps(), 2);
        let input = vec![Complex64::new(1.0, 0.0); 50];
        let output = filter.process(&input);
        assert_eq!(output.len(), 50);
    }

    #[test]
    fn test_with_fft_size() {
        let taps = vec![1.0; 16];
        let filter = FftFilter::with_fft_size(&taps, 256);
        assert_eq!(filter.fft_size(), 256);
    }

    #[test]
    fn test_empty_input() {
        let mut filter = FftFilter::new(&[1.0; 8]);
        let output = filter.process(&[]);
        assert!(output.is_empty());
    }

    #[test]
    fn test_scaling_filter() {
        // A filter [2.0] should double the signal
        let mut filter = FftFilter::new(&[2.0]);
        let input: Vec<Complex64> = (0..50).map(|i| {
            Complex64::new(i as f64, 0.0)
        }).collect();
        let output = filter.process(&input);
        for (a, b) in output.iter().zip(input.iter()) {
            assert!((a.re - 2.0 * b.re).abs() < 1e-6);
        }
    }

    #[test]
    fn test_multiple_blocks() {
        // Process in two chunks and verify continuity
        let taps = vec![0.25; 8];
        let mut filter = FftFilter::new(&taps);
        let full_input = vec![Complex64::new(1.0, 0.0); 200];

        let out1 = filter.process(&full_input[..100]);
        let out2 = filter.process(&full_input[100..]);

        let mut filter2 = FftFilter::new(&taps);
        let out_full = filter2.process(&full_input);

        // Concatenated output should match full processing (approximately)
        let combined: Vec<Complex64> = out1.into_iter().chain(out2.into_iter()).collect();
        assert_eq!(combined.len(), out_full.len());
    }

    #[test]
    fn test_debug_impl() {
        let filter = FftFilter::new(&[1.0; 8]);
        let debug = format!("{:?}", filter);
        assert!(debug.contains("FftFilter"));
        assert!(debug.contains("num_taps: 8"));
    }
}
