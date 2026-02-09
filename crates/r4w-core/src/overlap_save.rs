//! Overlap-Save & Overlap-Add — Efficient FFT Block Convolution
//!
//! Streaming FFT-based convolution of long signals with FIR filters.
//! Overlap-Save discards edge segments; Overlap-Add sums overlapping
//! output blocks. Both yield O(N log N) per block vs O(NM) direct.
//!
//! GNU Radio equivalent: internal to `fft_filter_*` blocks.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::overlap_save::OverlapSave;
//!
//! let taps = vec![0.2; 5];
//! let mut ols = OverlapSave::new(&taps, 128);
//! let input: Vec<f64> = (0..512).map(|i| if i % 10 == 0 { 1.0 } else { 0.0 }).collect();
//! let output = ols.process(&input);
//! assert!(output.len() > 0);
//! ```

use std::f64::consts::PI;

/// Overlap-Save streaming convolution.
///
/// Breaks input into overlapping blocks of size `fft_size`, applies
/// circular convolution via FFT, and keeps only the valid (non-aliased)
/// output samples from each block.
#[derive(Debug, Clone)]
pub struct OverlapSave {
    fft_size: usize,
    filter_len: usize,
    valid_len: usize, // fft_size - filter_len + 1
    filter_freq: Vec<(f64, f64)>, // (re, im) DFT of zero-padded filter
    buffer: Vec<f64>,
}

impl OverlapSave {
    /// Create an overlap-save processor.
    ///
    /// `filter_taps`: FIR filter coefficients.
    /// `block_size`: desired number of valid output samples per block.
    /// The FFT size is set to `block_size + filter_len - 1`, rounded up to power of 2.
    pub fn new(filter_taps: &[f64], block_size: usize) -> Self {
        let filter_len = filter_taps.len().max(1);
        let block_size = block_size.max(filter_len);
        let raw_fft = block_size + filter_len - 1;
        let fft_size = raw_fft.next_power_of_two();
        let valid_len = fft_size - filter_len + 1;

        // Zero-pad filter and compute DFT
        let mut h = vec![0.0; fft_size];
        for (i, &t) in filter_taps.iter().enumerate() {
            h[i] = t;
        }
        let filter_freq = real_dft(&h);

        Self {
            fft_size,
            filter_len,
            valid_len,
            filter_freq,
            buffer: vec![0.0; fft_size], // overlap buffer starts zeroed
        }
    }

    /// Process input samples and return convolved output.
    ///
    /// Can accept arbitrary-length input; returns as many valid samples
    /// as can be produced from complete blocks.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let overlap = self.filter_len - 1;
        let mut output = Vec::new();
        let mut pos = 0;

        while pos < input.len() {
            let remaining = input.len() - pos;
            let take = remaining.min(self.valid_len);

            // Build block: overlap from previous + new samples
            let mut block = vec![0.0; self.fft_size];
            // Copy tail of previous buffer as overlap
            for i in 0..overlap {
                block[i] = self.buffer[self.valid_len + i];
            }
            // Copy new input
            for i in 0..take {
                block[overlap + i] = input[pos + i];
            }

            // Save for next overlap
            self.buffer = block.clone();

            if take < self.valid_len && pos + take < input.len() {
                break; // incomplete block, need more data
            }

            // FFT convolution
            let x_freq = real_dft(&block);
            let y_freq: Vec<(f64, f64)> = x_freq
                .iter()
                .zip(self.filter_freq.iter())
                .map(|(&(xr, xi), &(hr, hi))| (xr * hr - xi * hi, xr * hi + xi * hr))
                .collect();
            let y = real_idft(&y_freq);

            // Keep only valid samples (discard first overlap samples)
            let end = (overlap + take).min(y.len());
            for i in overlap..end {
                output.push(y[i]);
            }

            pos += take;
        }
        output
    }

    /// Report the latency in samples (filter_len - 1).
    pub fn latency(&self) -> usize {
        self.filter_len - 1
    }

    /// Reset internal buffers.
    pub fn reset(&mut self) {
        self.buffer.fill(0.0);
    }
}

/// Overlap-Add streaming convolution.
///
/// Segments input into non-overlapping blocks, applies linear convolution
/// via FFT (zero-padded), and sums the overlapping output tails.
#[derive(Debug, Clone)]
pub struct OverlapAdd {
    fft_size: usize,
    block_size: usize,
    filter_len: usize,
    filter_freq: Vec<(f64, f64)>,
    overlap_buf: Vec<f64>,
}

impl OverlapAdd {
    /// Create an overlap-add processor.
    pub fn new(filter_taps: &[f64], block_size: usize) -> Self {
        let filter_len = filter_taps.len().max(1);
        let block_size = block_size.max(1);
        let fft_size = (block_size + filter_len - 1).next_power_of_two();

        let mut h = vec![0.0; fft_size];
        for (i, &t) in filter_taps.iter().enumerate() {
            h[i] = t;
        }
        let filter_freq = real_dft(&h);

        Self {
            fft_size,
            block_size,
            filter_len,
            filter_freq,
            overlap_buf: vec![0.0; filter_len - 1],
        }
    }

    /// Process input samples and return convolved output.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::new();
        let mut pos = 0;

        while pos < input.len() {
            let take = (input.len() - pos).min(self.block_size);

            // Zero-pad block to FFT size
            let mut block = vec![0.0; self.fft_size];
            for i in 0..take {
                block[i] = input[pos + i];
            }

            // FFT convolution
            let x_freq = real_dft(&block);
            let y_freq: Vec<(f64, f64)> = x_freq
                .iter()
                .zip(self.filter_freq.iter())
                .map(|(&(xr, xi), &(hr, hi))| (xr * hr - xi * hi, xr * hi + xi * hr))
                .collect();
            let y = real_idft(&y_freq);

            // Add overlap from previous block
            let overlap_len = self.overlap_buf.len();
            for i in 0..overlap_len.min(take) {
                output.push(y[i] + self.overlap_buf[i]);
            }
            for i in overlap_len..take {
                output.push(y[i]);
            }

            // Save new overlap
            let new_overlap_start = take;
            let new_overlap_end = (take + self.filter_len - 1).min(y.len());
            self.overlap_buf = vec![0.0; self.filter_len - 1];
            for i in new_overlap_start..new_overlap_end {
                self.overlap_buf[i - new_overlap_start] = y[i];
            }

            pos += take;
        }
        output
    }

    /// Reset internal buffers.
    pub fn reset(&mut self) {
        self.overlap_buf.fill(0.0);
    }
}

/// Direct (time-domain) linear convolution for reference/testing.
pub fn direct_convolve(signal: &[f64], filter: &[f64]) -> Vec<f64> {
    if signal.is_empty() || filter.is_empty() {
        return vec![];
    }
    let out_len = signal.len() + filter.len() - 1;
    let mut output = vec![0.0; out_len];
    for (i, &s) in signal.iter().enumerate() {
        for (j, &h) in filter.iter().enumerate() {
            output[i + j] += s * h;
        }
    }
    output
}

// ---- Minimal DFT/IDFT (radix-2 Cooley-Tukey) ----

fn real_dft(x: &[f64]) -> Vec<(f64, f64)> {
    let n = x.len();
    let cx: Vec<(f64, f64)> = x.iter().map(|&v| (v, 0.0)).collect();
    fft(&cx, false)
}

fn real_idft(x: &[(f64, f64)]) -> Vec<f64> {
    let n = x.len();
    let result = fft(&x.to_vec(), true);
    result.iter().map(|(r, _)| r / n as f64).collect()
}

fn fft(x: &[(f64, f64)], inverse: bool) -> Vec<(f64, f64)> {
    let n = x.len();
    if n <= 1 {
        return x.to_vec();
    }

    let even: Vec<(f64, f64)> = x.iter().step_by(2).cloned().collect();
    let odd: Vec<(f64, f64)> = x.iter().skip(1).step_by(2).cloned().collect();

    let even_fft = fft(&even, inverse);
    let odd_fft = fft(&odd, inverse);

    let sign = if inverse { 1.0 } else { -1.0 };
    let mut result = vec![(0.0, 0.0); n];
    for k in 0..n / 2 {
        let angle = sign * 2.0 * PI * k as f64 / n as f64;
        let twiddle = (angle.cos(), angle.sin());
        let (or, oi) = odd_fft[k];
        let prod = (twiddle.0 * or - twiddle.1 * oi, twiddle.0 * oi + twiddle.1 * or);
        result[k] = (even_fft[k].0 + prod.0, even_fft[k].1 + prod.1);
        result[k + n / 2] = (even_fft[k].0 - prod.0, even_fft[k].1 - prod.1);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ols_matches_direct_convolution() {
        let filter = vec![1.0, -0.5, 0.25];
        let signal: Vec<f64> = (0..128).map(|i| (i as f64 * 0.1).sin()).collect();
        let direct = direct_convolve(&signal, &filter);
        let mut ols = OverlapSave::new(&filter, 32);
        let ols_out = ols.process(&signal);
        // Compare overlapping portion
        let cmp_len = ols_out.len().min(direct.len());
        for i in 0..cmp_len {
            assert!(
                (ols_out[i] - direct[i]).abs() < 1e-6,
                "mismatch at {i}: ols={}, direct={}",
                ols_out[i],
                direct[i]
            );
        }
    }

    #[test]
    fn test_ola_matches_direct_convolution() {
        let filter = vec![0.5, 0.3, 0.2];
        let signal: Vec<f64> = (0..100).map(|i| (i as f64 * 0.15).cos()).collect();
        let direct = direct_convolve(&signal, &filter);
        let mut ola = OverlapAdd::new(&filter, 32);
        let ola_out = ola.process(&signal);
        let cmp_len = ola_out.len().min(direct.len());
        assert!(cmp_len > 0);
        for i in 0..cmp_len {
            assert!(
                (ola_out[i] - direct[i]).abs() < 1e-6,
                "mismatch at {i}: ola={}, direct={}",
                ola_out[i],
                direct[i]
            );
        }
    }

    #[test]
    fn test_single_tap_identity() {
        let filter = vec![1.0];
        let signal: Vec<f64> = (0..64).map(|i| i as f64).collect();
        let mut ols = OverlapSave::new(&filter, 32);
        let output = ols.process(&signal);
        for i in 0..output.len().min(signal.len()) {
            assert!(
                (output[i] - signal[i]).abs() < 1e-10,
                "mismatch at {i}: out={}, sig={}",
                output[i],
                signal[i]
            );
        }
    }

    #[test]
    fn test_latency() {
        let filter = vec![1.0, 0.5, 0.25, 0.125, 0.0625];
        let ols = OverlapSave::new(&filter, 64);
        assert_eq!(ols.latency(), 4); // filter_len - 1
    }

    #[test]
    fn test_direct_convolve() {
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![0.5, 1.0];
        let c = direct_convolve(&a, &b);
        assert_eq!(c.len(), 4); // 3 + 2 - 1
        assert!((c[0] - 0.5).abs() < 1e-10);
        assert!((c[1] - 2.0).abs() < 1e-10);
        assert!((c[2] - 3.5).abs() < 1e-10);
        assert!((c[3] - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_reset_clears_state() {
        let filter = vec![1.0, 0.5];
        let mut ols = OverlapSave::new(&filter, 32);
        let _ = ols.process(&vec![1.0; 64]);
        ols.reset();
        assert!(ols.buffer.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn test_empty_input() {
        let filter = vec![1.0, 0.5];
        let mut ols = OverlapSave::new(&filter, 32);
        let output = ols.process(&[]);
        assert!(output.is_empty());
    }

    #[test]
    fn test_fft_roundtrip() {
        let signal: Vec<f64> = (0..8).map(|i| (i as f64 * 0.3).sin()).collect();
        let freq = real_dft(&signal);
        let recovered = real_idft(&freq);
        for i in 0..signal.len() {
            assert!(
                (signal[i] - recovered[i]).abs() < 1e-10,
                "FFT roundtrip mismatch at {i}"
            );
        }
    }

    #[test]
    fn test_ola_reset() {
        let filter = vec![1.0, 0.5];
        let mut ola = OverlapAdd::new(&filter, 32);
        let _ = ola.process(&vec![1.0; 64]);
        ola.reset();
        assert!(ola.overlap_buf.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn test_direct_convolve_empty() {
        assert!(direct_convolve(&[], &[1.0]).is_empty());
        assert!(direct_convolve(&[1.0], &[]).is_empty());
    }
}
