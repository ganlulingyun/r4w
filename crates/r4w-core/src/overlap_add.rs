//! Overlap-Add — FFT-based Fast Convolution
//!
//! Implements the overlap-add method for efficient FIR filtering of long
//! or streaming signals. The input is segmented into non-overlapping blocks,
//! each block is zero-padded, convolved with the filter kernel via FFT
//! multiplication, and overlapping output tails are summed to produce the
//! linear convolution result.
//!
//! Complexity: O(N log N) per block, vs O(NM) for direct convolution.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::overlap_add::OverlapAdd;
//!
//! // Create a simple averaging filter
//! let kernel = vec![0.25, 0.25, 0.25, 0.25];
//! let mut ola = OverlapAdd::new(&kernel, 64);
//!
//! // Filter a signal in one shot
//! let signal: Vec<f64> = (0..256).map(|i| (i as f64 * 0.1).sin()).collect();
//! let output = ola.process_block(&signal);
//! assert!(output.len() >= signal.len());
//!
//! // Or stream in chunks
//! ola.reset();
//! let chunk1 = ola.process_block(&signal[..128]);
//! let chunk2 = ola.process_block(&signal[128..]);
//! // Streaming output matches one-shot for the same samples
//! ```

use std::f64::consts::PI;

/// Overlap-Add streaming convolution processor.
///
/// Segments input into non-overlapping blocks of `block_size`, zero-pads
/// each to `fft_size`, applies spectral multiplication with the filter
/// kernel, and sums overlapping output tails across blocks.
#[derive(Debug, Clone)]
pub struct OverlapAdd {
    /// FFT length (power of 2, >= block_size + kernel_len - 1)
    fft_size: usize,
    /// Non-overlapping input block size
    block_size: usize,
    /// Length of the filter kernel
    kernel_len: usize,
    /// Frequency-domain filter kernel (complex pairs)
    kernel_freq: Vec<(f64, f64)>,
    /// Overlap buffer for tail accumulation (real mode)
    overlap_buf: Vec<f64>,
    /// Overlap buffer for tail accumulation (complex mode)
    overlap_buf_complex: Vec<(f64, f64)>,
    /// Residual input samples not yet forming a complete block
    residual: Vec<f64>,
    /// Residual complex input samples
    residual_complex: Vec<(f64, f64)>,
}

impl OverlapAdd {
    /// Create a new overlap-add processor.
    ///
    /// # Arguments
    /// - `kernel`: FIR filter coefficients (tap weights).
    /// - `block_size`: Number of new input samples per processing block.
    ///   The FFT size is automatically chosen as the next power of 2
    ///   >= `block_size + kernel.len() - 1`.
    ///
    /// # Panics
    /// Panics if `kernel` is empty or `block_size` is zero.
    pub fn new(kernel: &[f64], block_size: usize) -> Self {
        assert!(!kernel.is_empty(), "kernel must not be empty");
        assert!(block_size > 0, "block_size must be > 0");

        let kernel_len = kernel.len();
        let min_fft = block_size + kernel_len - 1;
        let fft_size = min_fft.next_power_of_two();

        // Zero-pad kernel and compute its FFT
        let mut h = vec![(0.0, 0.0); fft_size];
        for (i, &t) in kernel.iter().enumerate() {
            h[i] = (t, 0.0);
        }
        let kernel_freq = fft(&h, false);

        let overlap_len = kernel_len - 1;

        Self {
            fft_size,
            block_size,
            kernel_len,
            kernel_freq,
            overlap_buf: vec![0.0; overlap_len],
            overlap_buf_complex: vec![(0.0, 0.0); overlap_len],
            residual: Vec::new(),
            residual_complex: Vec::new(),
        }
    }

    /// Create an overlap-add processor with automatic FFT size selection.
    ///
    /// Chooses a block size of `max(4 * kernel.len(), 64)` for a good
    /// balance between FFT overhead and efficiency.
    pub fn auto(kernel: &[f64]) -> Self {
        assert!(!kernel.is_empty(), "kernel must not be empty");
        let block_size = (4 * kernel.len()).max(64);
        Self::new(kernel, block_size)
    }

    /// Process a block of real-valued input samples.
    ///
    /// Accepts arbitrary-length input. Internally buffers residual samples
    /// that don't fill a complete block; these are flushed on the next call
    /// or can be retrieved by calling `flush()`.
    ///
    /// Returns filtered output samples.
    pub fn process_block(&mut self, input: &[f64]) -> Vec<f64> {
        // Prepend any residual from previous call
        let mut samples = Vec::with_capacity(self.residual.len() + input.len());
        samples.append(&mut self.residual);
        samples.extend_from_slice(input);

        let mut output = Vec::new();
        let mut pos = 0;

        while pos + self.block_size <= samples.len() {
            let block = &samples[pos..pos + self.block_size];
            self.convolve_real_block(block, &mut output);
            pos += self.block_size;
        }

        // Save leftover as residual
        if pos < samples.len() {
            self.residual = samples[pos..].to_vec();
        }

        output
    }

    /// Process a block of complex-valued (I/Q) samples.
    ///
    /// Each sample is a `(f64, f64)` tuple representing `(real, imag)`.
    /// The filter kernel (real-valued) is applied to the complex signal
    /// via FFT-domain multiplication.
    ///
    /// Returns filtered complex output samples.
    pub fn process_block_complex(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if input.is_empty() {
            return Vec::new();
        }

        let mut samples = Vec::with_capacity(self.residual_complex.len() + input.len());
        samples.append(&mut self.residual_complex);
        samples.extend_from_slice(input);

        let mut output = Vec::new();
        let mut pos = 0;

        while pos + self.block_size <= samples.len() {
            let block = &samples[pos..pos + self.block_size];
            self.convolve_complex_block(block, &mut output);
            pos += self.block_size;
        }

        // Save leftover as residual
        if pos < samples.len() {
            self.residual_complex = samples[pos..].to_vec();
        }

        output
    }

    /// Flush any remaining residual samples by zero-padding to a full block.
    ///
    /// Returns the final output samples including the convolution tail.
    pub fn flush(&mut self) -> Vec<f64> {
        if self.residual.is_empty() {
            // Return the overlap tail
            let tail = self.overlap_buf.clone();
            self.overlap_buf.fill(0.0);
            return tail;
        }

        // Pad residual to block_size and process
        let mut padded = std::mem::take(&mut self.residual);
        padded.resize(self.block_size, 0.0);
        let mut output = Vec::new();
        self.convolve_real_block(&padded, &mut output);

        // Also drain the overlap buffer for the tail
        output.extend_from_slice(&self.overlap_buf);
        self.overlap_buf.fill(0.0);

        output
    }

    /// Update the filter kernel at runtime.
    ///
    /// The new kernel must be the same length as the original.
    ///
    /// # Panics
    /// Panics if `kernel.len() != self.kernel_len`.
    pub fn set_kernel(&mut self, kernel: &[f64]) {
        assert_eq!(
            kernel.len(),
            self.kernel_len,
            "new kernel length ({}) must match original ({})",
            kernel.len(),
            self.kernel_len,
        );

        let mut h = vec![(0.0, 0.0); self.fft_size];
        for (i, &t) in kernel.iter().enumerate() {
            h[i] = (t, 0.0);
        }
        self.kernel_freq = fft(&h, false);
    }

    /// Report the processing latency in samples.
    ///
    /// For the overlap-add method, latency equals `block_size` samples
    /// (the processor must accumulate a full block before producing output).
    pub fn latency(&self) -> usize {
        self.block_size
    }

    /// Reset all internal state (overlap buffers and residuals).
    ///
    /// Does not change the filter kernel or block size.
    pub fn reset(&mut self) {
        self.overlap_buf.fill(0.0);
        self.overlap_buf_complex.iter_mut().for_each(|v| *v = (0.0, 0.0));
        self.residual.clear();
        self.residual_complex.clear();
    }

    /// Return the FFT size in use.
    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    /// Return the block size (number of new input samples per block).
    pub fn block_size(&self) -> usize {
        self.block_size
    }

    /// Return the filter kernel length.
    pub fn kernel_len(&self) -> usize {
        self.kernel_len
    }

    // ---- internal helpers ----

    fn convolve_real_block(&mut self, block: &[f64], output: &mut Vec<f64>) {
        debug_assert!(block.len() == self.block_size);

        // Zero-pad input block to fft_size
        let mut x = vec![(0.0, 0.0); self.fft_size];
        for (i, &s) in block.iter().enumerate() {
            x[i] = (s, 0.0);
        }

        // FFT -> multiply -> IFFT
        let x_freq = fft(&x, false);
        let y_freq: Vec<(f64, f64)> = x_freq
            .iter()
            .zip(self.kernel_freq.iter())
            .map(|(&(xr, xi), &(hr, hi))| (xr * hr - xi * hi, xr * hi + xi * hr))
            .collect();
        let y_complex = fft(&y_freq, true);
        let n = self.fft_size as f64;

        let out_len = self.block_size;
        let overlap_len = self.kernel_len - 1;

        // Output block_size samples, adding in overlap from previous block
        for i in 0..out_len {
            let val = y_complex[i].0 / n
                + if i < overlap_len { self.overlap_buf[i] } else { 0.0 };
            output.push(val);
        }

        // Save new overlap: samples [block_size .. block_size + overlap_len - 1]
        for i in 0..overlap_len {
            let yi = out_len + i;
            self.overlap_buf[i] = if yi < self.fft_size {
                y_complex[yi].0 / n
            } else {
                0.0
            };
        }
    }

    fn convolve_complex_block(
        &mut self,
        block: &[(f64, f64)],
        output: &mut Vec<(f64, f64)>,
    ) {
        debug_assert!(block.len() == self.block_size);

        // Zero-pad input block to fft_size
        let mut x = vec![(0.0, 0.0); self.fft_size];
        for (i, &(re, im)) in block.iter().enumerate() {
            x[i] = (re, im);
        }

        // FFT -> multiply -> IFFT
        let x_freq = fft(&x, false);
        let y_freq: Vec<(f64, f64)> = x_freq
            .iter()
            .zip(self.kernel_freq.iter())
            .map(|(&(xr, xi), &(hr, hi))| (xr * hr - xi * hi, xr * hi + xi * hr))
            .collect();
        let y = fft(&y_freq, true);
        let n = self.fft_size as f64;

        let out_len = self.block_size;
        let overlap_len = self.kernel_len - 1;

        // Output block_size samples, adding in overlap from previous block
        for i in 0..out_len {
            let re = y[i].0 / n + self.overlap_buf_complex.get(i).map_or(0.0, |v| v.0);
            let im = y[i].1 / n + self.overlap_buf_complex.get(i).map_or(0.0, |v| v.1);
            output.push((re, im));
        }

        // Save new overlap
        for i in 0..overlap_len {
            let yi = out_len + i;
            self.overlap_buf_complex[i] = if yi < self.fft_size {
                (y[yi].0 / n, y[yi].1 / n)
            } else {
                (0.0, 0.0)
            };
        }
    }
}

/// Direct (time-domain) linear convolution for testing and reference.
pub fn direct_convolve(signal: &[f64], kernel: &[f64]) -> Vec<f64> {
    if signal.is_empty() || kernel.is_empty() {
        return vec![];
    }
    let out_len = signal.len() + kernel.len() - 1;
    let mut output = vec![0.0; out_len];
    for (i, &s) in signal.iter().enumerate() {
        for (j, &h) in kernel.iter().enumerate() {
            output[i + j] += s * h;
        }
    }
    output
}

/// Direct (time-domain) complex linear convolution for testing.
pub fn direct_convolve_complex(
    signal: &[(f64, f64)],
    kernel: &[(f64, f64)],
) -> Vec<(f64, f64)> {
    if signal.is_empty() || kernel.is_empty() {
        return vec![];
    }
    let out_len = signal.len() + kernel.len() - 1;
    let mut output = vec![(0.0, 0.0); out_len];
    for (i, &(sr, si)) in signal.iter().enumerate() {
        for (j, &(hr, hi)) in kernel.iter().enumerate() {
            output[i + j].0 += sr * hr - si * hi;
            output[i + j].1 += sr * hi + si * hr;
        }
    }
    output
}

// ---- Radix-2 Cooley-Tukey FFT (iterative) ----

fn fft(x: &[(f64, f64)], inverse: bool) -> Vec<(f64, f64)> {
    let n = x.len();
    if n <= 1 {
        return x.to_vec();
    }
    debug_assert!(n.is_power_of_two(), "FFT size must be a power of 2");

    // Bit-reversal permutation
    let mut result = vec![(0.0, 0.0); n];
    let bits = n.trailing_zeros();
    for i in 0..n {
        let rev = bit_reverse(i, bits);
        result[rev] = x[i];
    }

    // Iterative butterfly computation
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut size = 2;
    while size <= n {
        let half = size / 2;
        let angle_step = sign * 2.0 * PI / size as f64;
        let w_base = (angle_step.cos(), angle_step.sin());
        let mut k = 0;
        while k < n {
            let mut w = (1.0, 0.0);
            for j in 0..half {
                let u = result[k + j];
                let t = complex_mul(w, result[k + j + half]);
                result[k + j] = (u.0 + t.0, u.1 + t.1);
                result[k + j + half] = (u.0 - t.0, u.1 - t.1);
                w = complex_mul(w, w_base);
            }
            k += size;
        }
        size *= 2;
    }

    result
}

#[inline]
fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn bit_reverse(mut x: usize, bits: u32) -> usize {
    let mut result = 0;
    for _ in 0..bits {
        result = (result << 1) | (x & 1);
        x >>= 1;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-9;

    /// Helper: max absolute difference between two slices.
    fn max_err(a: &[f64], b: &[f64]) -> f64 {
        a.iter()
            .zip(b.iter())
            .map(|(x, y)| (x - y).abs())
            .fold(0.0f64, f64::max)
    }

    #[test]
    fn test_matches_direct_convolution() {
        let kernel = vec![1.0, -0.5, 0.25, -0.125];
        let signal: Vec<f64> = (0..256).map(|i| (i as f64 * 0.1).sin()).collect();
        let direct = direct_convolve(&signal, &kernel);

        let mut ola = OverlapAdd::new(&kernel, 32);
        let output = ola.process_block(&signal);

        // output should be signal.len() samples (tail is in overlap buffer)
        assert_eq!(output.len(), signal.len());

        // Compare with direct (only the first signal.len() samples)
        let err = max_err(&output, &direct[..output.len()]);
        assert!(err < 1e-10, "max error = {err}");
    }

    #[test]
    fn test_streaming_consistency() {
        // Processing in chunks should produce the same result as one shot
        let kernel = vec![0.2, 0.3, 0.5];
        let signal: Vec<f64> = (0..256).map(|i| (i as f64 * 0.07).cos()).collect();

        // One shot
        let mut ola1 = OverlapAdd::new(&kernel, 64);
        let one_shot = ola1.process_block(&signal);

        // Streaming in 50-sample chunks
        let mut ola2 = OverlapAdd::new(&kernel, 64);
        let mut streamed = Vec::new();
        for chunk in signal.chunks(50) {
            streamed.extend(ola2.process_block(chunk));
        }

        let cmp_len = one_shot.len().min(streamed.len());
        assert!(cmp_len > 0);
        let err = max_err(&one_shot[..cmp_len], &streamed[..cmp_len]);
        assert!(err < 1e-10, "streaming vs one-shot max error = {err}");
    }

    #[test]
    fn test_identity_filter() {
        let kernel = vec![1.0];
        let signal: Vec<f64> = (0..128).map(|i| i as f64 * 0.5).collect();

        let mut ola = OverlapAdd::new(&kernel, 32);
        let output = ola.process_block(&signal);

        assert_eq!(output.len(), signal.len());
        let err = max_err(&output, &signal);
        assert!(err < 1e-12, "identity filter max error = {err}");
    }

    #[test]
    fn test_delay_filter() {
        // A delay of 1 sample: kernel = [0, 1]
        let kernel = vec![0.0, 1.0];
        let signal: Vec<f64> = (0..64).map(|i| (i + 1) as f64).collect();

        let mut ola = OverlapAdd::new(&kernel, 32);
        let output = ola.process_block(&signal);

        // output[0] should be 0 (delayed), output[1] = signal[0], etc.
        assert!((output[0]).abs() < TOL);
        for i in 1..output.len() {
            assert!(
                (output[i] - signal[i - 1]).abs() < TOL,
                "delay mismatch at {i}: got {}, expected {}",
                output[i],
                signal[i - 1],
            );
        }
    }

    #[test]
    fn test_impulse_response() {
        let kernel = vec![0.5, 0.3, 0.2, 0.1];
        // Impulse input
        let mut signal = vec![0.0; 64];
        signal[0] = 1.0;

        let mut ola = OverlapAdd::new(&kernel, 16);
        let output = ola.process_block(&signal);

        // First kernel_len samples should equal the kernel itself
        for (i, &k) in kernel.iter().enumerate() {
            assert!(
                (output[i] - k).abs() < TOL,
                "impulse response mismatch at {i}: got {}, expected {k}",
                output[i],
            );
        }
        // Rest should be zero
        for i in kernel.len()..output.len() {
            assert!(
                output[i].abs() < TOL,
                "non-zero at {i}: {}",
                output[i],
            );
        }
    }

    #[test]
    fn test_complex_convolution() {
        // Real kernel applied to complex signal
        let kernel = vec![0.5, 0.3, 0.2];
        let signal: Vec<(f64, f64)> = (0..128)
            .map(|i| {
                let t = i as f64 * 0.1;
                (t.cos(), t.sin())
            })
            .collect();

        // Direct complex convolution (kernel is real, so imag kernel = 0)
        let complex_kernel: Vec<(f64, f64)> = kernel.iter().map(|&k| (k, 0.0)).collect();
        let direct = direct_convolve_complex(&signal, &complex_kernel);

        let mut ola = OverlapAdd::new(&kernel, 32);
        let output = ola.process_block_complex(&signal);

        assert_eq!(output.len(), signal.len());
        for i in 0..output.len() {
            let err_re = (output[i].0 - direct[i].0).abs();
            let err_im = (output[i].1 - direct[i].1).abs();
            assert!(
                err_re < 1e-8 && err_im < 1e-8,
                "complex mismatch at {i}: got ({}, {}), expected ({}, {})",
                output[i].0,
                output[i].1,
                direct[i].0,
                direct[i].1,
            );
        }
    }

    #[test]
    fn test_set_kernel() {
        let kernel1 = vec![1.0, 0.0, 0.0];
        let kernel2 = vec![0.0, 0.0, 1.0];
        let signal: Vec<f64> = (0..64).map(|i| (i + 1) as f64).collect();

        let mut ola = OverlapAdd::new(&kernel1, 32);
        let out1 = ola.process_block(&signal);
        // kernel1 is identity, output should match input
        let err1 = max_err(&out1, &signal[..out1.len()]);
        assert!(err1 < TOL);

        // Switch to delay-by-2 kernel
        ola.reset();
        ola.set_kernel(&kernel2);
        let out2 = ola.process_block(&signal);
        // out2[0] and out2[1] should be ~0, out2[2] = signal[0], etc.
        assert!(out2[0].abs() < TOL);
        assert!(out2[1].abs() < TOL);
        for i in 2..out2.len() {
            assert!(
                (out2[i] - signal[i - 2]).abs() < TOL,
                "delay-2 mismatch at {i}: got {}, expected {}",
                out2[i],
                signal[i - 2],
            );
        }
    }

    #[test]
    fn test_reset() {
        let kernel = vec![1.0, 0.5, 0.25];
        let mut ola = OverlapAdd::new(&kernel, 16);
        let _ = ola.process_block(&vec![1.0; 64]);

        ola.reset();

        assert!(ola.overlap_buf.iter().all(|&x| x == 0.0));
        assert!(ola.residual.is_empty());
    }

    #[test]
    fn test_latency_and_sizes() {
        let kernel = vec![1.0; 5];
        let ola = OverlapAdd::new(&kernel, 64);

        assert_eq!(ola.latency(), 64);
        assert_eq!(ola.kernel_len(), 5);
        assert_eq!(ola.block_size(), 64);
        // FFT size should be next power of 2 >= 64 + 5 - 1 = 68
        assert_eq!(ola.fft_size(), 128);
    }

    #[test]
    fn test_empty_input() {
        let kernel = vec![1.0, 0.5];
        let mut ola = OverlapAdd::new(&kernel, 32);
        let output = ola.process_block(&[]);
        assert!(output.is_empty());
    }

    #[test]
    fn test_fft_roundtrip() {
        let signal: Vec<(f64, f64)> = (0..16)
            .map(|i| ((i as f64 * 0.3).sin(), (i as f64 * 0.7).cos()))
            .collect();
        let freq = fft(&signal, false);
        let recovered = fft(&freq, true);
        let n = signal.len() as f64;
        for i in 0..signal.len() {
            let (re, im) = (recovered[i].0 / n, recovered[i].1 / n);
            assert!(
                (re - signal[i].0).abs() < 1e-12 && (im - signal[i].1).abs() < 1e-12,
                "FFT roundtrip mismatch at {i}",
            );
        }
    }

    #[test]
    fn test_auto_constructor() {
        let kernel = vec![0.1; 10];
        let ola = OverlapAdd::auto(&kernel);
        assert_eq!(ola.kernel_len(), 10);
        // block_size should be max(4*10, 64) = 64
        assert_eq!(ola.block_size(), 64);
        // FFT size >= 64 + 10 - 1 = 73, next pow2 = 128
        assert_eq!(ola.fft_size(), 128);
    }

    #[test]
    fn test_various_block_sizes() {
        // Ensure correctness across different block sizes
        let kernel = vec![0.3, -0.5, 0.7, -0.2, 0.1];
        let signal: Vec<f64> = (0..300).map(|i| (i as f64 * 0.05).sin()).collect();
        let direct = direct_convolve(&signal, &kernel);

        for &bs in &[8, 16, 32, 64, 128] {
            let mut ola = OverlapAdd::new(&kernel, bs);
            let output = ola.process_block(&signal);
            let cmp_len = output.len().min(direct.len());
            let err = max_err(&output[..cmp_len], &direct[..cmp_len]);
            assert!(
                err < 1e-9,
                "block_size={bs}: max error = {err}",
            );
        }
    }

    #[test]
    fn test_flush_produces_tail() {
        let kernel = vec![1.0, 1.0, 1.0];
        let signal = vec![1.0, 0.0, 0.0];
        // Direct convolution: [1, 1, 1, 0, 0]
        let direct = direct_convolve(&signal, &kernel);
        assert_eq!(direct, vec![1.0, 1.0, 1.0, 0.0, 0.0]);

        // Use block_size = 3 so signal fits in exactly one block
        let mut ola = OverlapAdd::new(&kernel, 3);
        let out = ola.process_block(&signal);
        // Should have 3 output samples
        assert_eq!(out.len(), 3);
        for i in 0..3 {
            assert!(
                (out[i] - direct[i]).abs() < TOL,
                "main output mismatch at {i}",
            );
        }

        // Flush should give us the tail (kernel_len - 1 = 2 samples)
        let tail = ola.flush();
        // Tail values should be [0, 0] from the convolution tail
        assert!(tail.len() >= 2);
        for i in 0..2 {
            assert!(
                (tail[i] - direct[3 + i]).abs() < TOL,
                "tail mismatch at {i}: got {}, expected {}",
                tail[i],
                direct[3 + i],
            );
        }
    }

    #[test]
    #[should_panic(expected = "kernel must not be empty")]
    fn test_empty_kernel_panics() {
        let _ = OverlapAdd::new(&[], 32);
    }

    #[test]
    #[should_panic(expected = "block_size must be > 0")]
    fn test_zero_block_size_panics() {
        let _ = OverlapAdd::new(&[1.0], 0);
    }

    #[test]
    #[should_panic(expected = "new kernel length")]
    fn test_set_kernel_wrong_size_panics() {
        let mut ola = OverlapAdd::new(&[1.0, 0.5], 32);
        ola.set_kernel(&[1.0, 0.5, 0.25]); // wrong length
    }
}
