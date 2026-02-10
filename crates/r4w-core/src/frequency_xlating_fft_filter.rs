//! # Frequency-Translating FFT Filter
//!
//! Combines frequency translation (mixing) with FFT-based FIR filtering and
//! decimation in a single efficient operation. More efficient than separate
//! mixer + FIR filter for long filter kernels, as filtering is done via
//! overlap-save FFT convolution.
//!
//! ## Algorithm
//!
//! 1. Mix input with NCO at center frequency → baseband
//! 2. Apply FIR filter via FFT overlap-save convolution
//! 3. Decimate by factor M
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::frequency_xlating_fft_filter::FreqXlatingFftFilter;
//!
//! // Design a filter: center freq 10kHz, 48kHz sample rate, decimate by 4
//! let taps = FreqXlatingFftFilter::design_lowpass(31, 5000.0, 48000.0);
//! let mut filter = FreqXlatingFftFilter::new(&taps, 10000.0, 48000.0, 4);
//!
//! let input = vec![(1.0, 0.0); 256];
//! let output = filter.process(&input);
//! assert_eq!(output.len(), 256 / 4);
//! ```

use std::f64::consts::PI;

/// Frequency-translating FFT-based FIR filter with decimation.
#[derive(Debug, Clone)]
pub struct FreqXlatingFftFilter {
    /// FIR filter taps (complex, frequency-shifted).
    taps: Vec<(f64, f64)>,
    /// Original (real) taps length.
    num_taps: usize,
    /// FFT size (next power of 2 >= 2 * num_taps).
    fft_size: usize,
    /// Frequency-domain filter response (precomputed).
    filter_fft: Vec<(f64, f64)>,
    /// Overlap-save input buffer.
    input_buffer: Vec<(f64, f64)>,
    /// Number of valid output samples per FFT block.
    block_size: usize,
    /// Decimation factor.
    decimation: usize,
    /// Center frequency (Hz).
    center_freq: f64,
    /// Sample rate (Hz).
    sample_rate: f64,
    /// NCO phase accumulator.
    nco_phase: f64,
    /// NCO phase increment per sample.
    nco_increment: f64,
    /// Decimation phase counter.
    decim_count: usize,
    /// Total samples processed.
    samples_processed: u64,
}

impl FreqXlatingFftFilter {
    /// Create a new frequency-translating FFT filter.
    ///
    /// # Arguments
    /// * `taps` - Real-valued FIR filter taps (lowpass prototype)
    /// * `center_freq` - Center frequency to translate to baseband (Hz)
    /// * `sample_rate` - Input sample rate (Hz)
    /// * `decimation` - Decimation factor (≥1)
    pub fn new(taps: &[f64], center_freq: f64, sample_rate: f64, decimation: usize) -> Self {
        let num_taps = taps.len().max(1);
        let decimation = decimation.max(1);

        // FFT size: next power of 2 >= 2 * num_taps for overlap-save.
        let fft_size = (2 * num_taps).next_power_of_two();
        let block_size = fft_size - num_taps + 1;

        // Use real-valued taps (NCO handles frequency translation).
        let nco_inc = 2.0 * PI * center_freq / sample_rate;
        let real_taps: Vec<(f64, f64)> = taps.iter().map(|&t| (t, 0.0)).collect();

        // Zero-pad taps to FFT size and compute FFT.
        let mut padded = vec![(0.0, 0.0); fft_size];
        for (i, &tap) in real_taps.iter().enumerate() {
            padded[i] = tap;
        }
        let filter_fft = fft(&padded);

        Self {
            taps: real_taps,
            num_taps,
            fft_size,
            filter_fft,
            input_buffer: vec![(0.0, 0.0); fft_size],
            block_size,
            decimation,
            center_freq,
            sample_rate,
            nco_phase: 0.0,
            nco_increment: nco_inc,
            decim_count: 0,
            samples_processed: 0,
        }
    }

    /// Process a block of IQ samples.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut output = Vec::new();

        // Mix with NCO first, then apply FFT filter + decimation.
        let mixed: Vec<(f64, f64)> = input
            .iter()
            .map(|&(re, im)| {
                let cos_p = self.nco_phase.cos();
                let sin_p = self.nco_phase.sin();
                self.nco_phase += self.nco_increment;
                if self.nco_phase > PI {
                    self.nco_phase -= 2.0 * PI;
                }
                if self.nco_phase < -PI {
                    self.nco_phase += 2.0 * PI;
                }
                // Complex multiply: (re + j*im) * (cos - j*sin) for downconversion
                (
                    re * cos_p + im * sin_p,
                    -re * sin_p + im * cos_p,
                )
            })
            .collect();

        // Process in overlap-save blocks.
        let mut pos = 0;
        while pos < mixed.len() {
            let remaining = mixed.len() - pos;
            let chunk = remaining.min(self.block_size);

            // Shift old data left, insert new data.
            let overlap = self.num_taps - 1;
            for i in 0..overlap {
                self.input_buffer[i] = self.input_buffer[self.block_size + i];
            }
            for i in 0..chunk {
                self.input_buffer[overlap + i] = mixed[pos + i];
            }
            // Zero-pad if chunk < block_size.
            for i in chunk..self.block_size {
                self.input_buffer[overlap + i] = (0.0, 0.0);
            }

            // FFT convolution.
            let input_fft = fft(&self.input_buffer[..self.fft_size]);
            let mut product = Vec::with_capacity(self.fft_size);
            for i in 0..self.fft_size {
                let (ar, ai) = input_fft[i];
                let (br, bi) = self.filter_fft[i];
                product.push((ar * br - ai * bi, ar * bi + ai * br));
            }
            let conv = ifft(&product);

            // Extract valid samples (skip first num_taps-1) and decimate.
            let valid_start = self.num_taps - 1;
            for i in 0..chunk {
                self.samples_processed += 1;
                if self.decim_count == 0 {
                    output.push(conv[valid_start + i]);
                }
                self.decim_count = (self.decim_count + 1) % self.decimation;
            }

            pos += chunk;
        }

        output
    }

    /// Design a lowpass FIR filter using windowed-sinc method.
    pub fn design_lowpass(num_taps: usize, cutoff_hz: f64, sample_rate: f64) -> Vec<f64> {
        let n = if num_taps % 2 == 0 {
            num_taps + 1
        } else {
            num_taps
        };
        let m = (n - 1) as f64;
        let fc = cutoff_hz / sample_rate;

        (0..n)
            .map(|i| {
                let x = i as f64 - m / 2.0;
                let sinc = if x.abs() < 1e-12 {
                    2.0 * fc
                } else {
                    (2.0 * PI * fc * x).sin() / (PI * x)
                };
                // Hamming window.
                let window = 0.54 - 0.46 * (2.0 * PI * i as f64 / m).cos();
                sinc * window
            })
            .collect()
    }

    /// Retune to a new center frequency.
    pub fn set_center_freq(&mut self, freq: f64) {
        self.center_freq = freq;
        self.nco_increment = 2.0 * PI * freq / self.sample_rate;
    }

    /// Get the current center frequency.
    pub fn center_freq(&self) -> f64 {
        self.center_freq
    }

    /// Get the decimation factor.
    pub fn decimation(&self) -> usize {
        self.decimation
    }

    /// Get the output sample rate.
    pub fn output_sample_rate(&self) -> f64 {
        self.sample_rate / self.decimation as f64
    }

    /// Get the number of filter taps.
    pub fn num_taps(&self) -> usize {
        self.num_taps
    }

    /// Get the FFT size used internally.
    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    /// Get total input samples processed.
    pub fn samples_processed(&self) -> u64 {
        self.samples_processed
    }

    /// Reset the filter state.
    pub fn reset(&mut self) {
        self.input_buffer = vec![(0.0, 0.0); self.fft_size];
        self.nco_phase = 0.0;
        self.decim_count = 0;
        self.samples_processed = 0;
    }
}

// ---- Built-in FFT (Cooley-Tukey radix-2 + DFT fallback) ----

fn fft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    if n <= 1 {
        return input.to_vec();
    }
    if n.is_power_of_two() {
        fft_radix2(input, false)
    } else {
        dft(input, false)
    }
}

fn ifft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    if n <= 1 {
        return input.to_vec();
    }
    let mut result = if n.is_power_of_two() {
        fft_radix2(input, true)
    } else {
        dft(input, true)
    };
    let scale = 1.0 / n as f64;
    for s in &mut result {
        s.0 *= scale;
        s.1 *= scale;
    }
    result
}

fn fft_radix2(input: &[(f64, f64)], inverse: bool) -> Vec<(f64, f64)> {
    let n = input.len();
    if n == 1 {
        return vec![input[0]];
    }

    let even: Vec<(f64, f64)> = input.iter().step_by(2).cloned().collect();
    let odd: Vec<(f64, f64)> = input.iter().skip(1).step_by(2).cloned().collect();

    let even_fft = fft_radix2(&even, inverse);
    let odd_fft = fft_radix2(&odd, inverse);

    let sign = if inverse { 1.0 } else { -1.0 };
    let mut result = vec![(0.0, 0.0); n];
    for k in 0..n / 2 {
        let angle = sign * 2.0 * PI * k as f64 / n as f64;
        let twiddle = (angle.cos(), angle.sin());
        let (or, oi) = odd_fft[k];
        let t = (twiddle.0 * or - twiddle.1 * oi, twiddle.0 * oi + twiddle.1 * or);
        result[k] = (even_fft[k].0 + t.0, even_fft[k].1 + t.1);
        result[k + n / 2] = (even_fft[k].0 - t.0, even_fft[k].1 - t.1);
    }
    result
}

fn dft(input: &[(f64, f64)], inverse: bool) -> Vec<(f64, f64)> {
    let n = input.len();
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut result = vec![(0.0, 0.0); n];
    for k in 0..n {
        for (j, &(re, im)) in input.iter().enumerate() {
            let angle = sign * 2.0 * PI * k as f64 * j as f64 / n as f64;
            let (cos_a, sin_a) = (angle.cos(), angle.sin());
            result[k].0 += re * cos_a - im * sin_a;
            result[k].1 += re * sin_a + im * cos_a;
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_output_length() {
        let taps = FreqXlatingFftFilter::design_lowpass(15, 5000.0, 48000.0);
        let mut filter = FreqXlatingFftFilter::new(&taps, 10000.0, 48000.0, 4);
        let input = vec![(1.0, 0.0); 256];
        let output = filter.process(&input);
        assert_eq!(output.len(), 256 / 4);
    }

    #[test]
    fn test_no_decimation() {
        let taps = FreqXlatingFftFilter::design_lowpass(7, 10000.0, 48000.0);
        let mut filter = FreqXlatingFftFilter::new(&taps, 0.0, 48000.0, 1);
        let input = vec![(1.0, 0.0); 100];
        let output = filter.process(&input);
        assert_eq!(output.len(), 100);
    }

    #[test]
    fn test_dc_passthrough() {
        // With center_freq=0 and a lowpass filter, DC should pass through.
        let taps = FreqXlatingFftFilter::design_lowpass(15, 10000.0, 48000.0);
        let mut filter = FreqXlatingFftFilter::new(&taps, 0.0, 48000.0, 1);
        let input = vec![(1.0, 0.0); 200];
        let output = filter.process(&input);
        // After filter settles, output should have non-trivial energy at DC.
        let energy: f64 = output[50..].iter().map(|(r, i)| r * r + i * i).sum();
        assert!(energy > 1.0, "DC signal should pass through lowpass filter");
    }

    #[test]
    fn test_retune() {
        let taps = FreqXlatingFftFilter::design_lowpass(15, 5000.0, 48000.0);
        let mut filter = FreqXlatingFftFilter::new(&taps, 10000.0, 48000.0, 4);
        assert!((filter.center_freq() - 10000.0).abs() < 1e-10);
        filter.set_center_freq(20000.0);
        assert!((filter.center_freq() - 20000.0).abs() < 1e-10);
    }

    #[test]
    fn test_output_sample_rate() {
        let taps = vec![1.0; 5];
        let filter = FreqXlatingFftFilter::new(&taps, 1000.0, 48000.0, 4);
        assert!((filter.output_sample_rate() - 12000.0).abs() < 1e-10);
    }

    #[test]
    fn test_lowpass_design() {
        let taps = FreqXlatingFftFilter::design_lowpass(31, 5000.0, 48000.0);
        assert_eq!(taps.len(), 31);
        // Taps should be symmetric (linear phase).
        for i in 0..taps.len() / 2 {
            assert!(
                (taps[i] - taps[taps.len() - 1 - i]).abs() < 1e-10,
                "Taps should be symmetric"
            );
        }
    }

    #[test]
    fn test_reset() {
        let taps = vec![1.0; 5];
        let mut filter = FreqXlatingFftFilter::new(&taps, 1000.0, 48000.0, 2);
        let input = vec![(1.0, 0.0); 50];
        filter.process(&input);
        assert!(filter.samples_processed() > 0);
        filter.reset();
        assert_eq!(filter.samples_processed(), 0);
    }

    #[test]
    fn test_tone_extraction() {
        // Generate a tone at 10 kHz and extract it.
        let fs = 48000.0;
        let f_tone = 10000.0;
        let n = 512;
        let input: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let phase = 2.0 * PI * f_tone * t;
                (phase.cos(), phase.sin())
            })
            .collect();

        let taps = FreqXlatingFftFilter::design_lowpass(31, 5000.0, fs);
        let mut filter = FreqXlatingFftFilter::new(&taps, f_tone, fs, 4);
        let output = filter.process(&input);

        // After mixing 10kHz tone down by 10kHz, we get DC.
        // After lowpass, DC should pass. Check output has energy.
        let energy: f64 = output[10..].iter().map(|(r, i)| r * r + i * i).sum();
        assert!(energy > 1.0, "Tone at center_freq should pass through");
    }

    #[test]
    fn test_fft_size() {
        let taps = vec![1.0; 15];
        let filter = FreqXlatingFftFilter::new(&taps, 1000.0, 48000.0, 1);
        assert!(filter.fft_size().is_power_of_two());
        assert!(filter.fft_size() >= 2 * 15);
    }

    #[test]
    fn test_samples_processed() {
        let taps = vec![1.0; 5];
        let mut filter = FreqXlatingFftFilter::new(&taps, 1000.0, 48000.0, 1);
        let input = vec![(1.0, 0.0); 100];
        filter.process(&input);
        assert_eq!(filter.samples_processed(), 100);
    }
}
