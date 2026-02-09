//! Frequency Xlating FIR Filter
//!
//! Combines frequency translation (mixing), FIR filtering, and decimation
//! in a single efficient operation. Extracts a narrowband signal from a
//! wideband input at an arbitrary center frequency.
//!
//! ## Signal Flow
//!
//! ```text
//! input → [×exp(-j2πf_c·n)] → [FIR lowpass] → [↓R] → output
//! ```
//!
//! The mixing step shifts the desired signal to baseband, the FIR filter
//! removes out-of-band signals, and decimation reduces the sample rate.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::freq_xlating_fir::FreqXlatingFirFilter;
//! use num_complex::Complex64;
//!
//! // Extract a signal at +10 kHz from a 1 MHz stream, decimate by 10
//! let mut xlat = FreqXlatingFirFilter::new(10_000.0, 1_000_000.0, 10, 64);
//!
//! let input: Vec<Complex64> = (0..1000)
//!     .map(|i| {
//!         let t = i as f64 / 1_000_000.0;
//!         let phase = 2.0 * std::f64::consts::PI * 10_000.0 * t;
//!         Complex64::new(phase.cos(), phase.sin())
//!     })
//!     .collect();
//!
//! let output = xlat.process_block(&input);
//! assert_eq!(output.len(), 100); // 1000 / 10
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Frequency-translating FIR filter with decimation.
#[derive(Debug, Clone)]
pub struct FreqXlatingFirFilter {
    /// Center frequency (Hz)
    center_freq: f64,
    /// Sample rate (Hz)
    sample_rate: f64,
    /// Decimation factor
    decimation: usize,
    /// FIR filter coefficients
    taps: Vec<f64>,
    /// Filter delay line
    delay_line: Vec<Complex64>,
    /// Write index into delay line
    write_idx: usize,
    /// NCO phase accumulator
    phase: f64,
    /// NCO phase increment per sample
    phase_inc: f64,
    /// Decimation counter
    decim_count: usize,
}

impl FreqXlatingFirFilter {
    /// Create a new frequency-translating FIR filter.
    ///
    /// # Arguments
    /// * `center_freq` - Frequency to translate to baseband (Hz)
    /// * `sample_rate` - Input sample rate (Hz)
    /// * `decimation` - Decimation factor (output rate = sample_rate / decimation)
    /// * `num_taps` - Number of FIR filter taps (auto-designed lowpass)
    pub fn new(center_freq: f64, sample_rate: f64, decimation: usize, num_taps: usize) -> Self {
        let decimation = decimation.max(1);
        let num_taps = num_taps.max(3) | 1; // Ensure odd

        // Design lowpass filter with cutoff at output_rate / 2
        let cutoff = sample_rate / (2.0 * decimation as f64);
        let taps = design_lowpass(cutoff, sample_rate, num_taps);

        Self {
            center_freq,
            sample_rate,
            decimation,
            taps,
            delay_line: vec![Complex64::new(0.0, 0.0); num_taps],
            write_idx: 0,
            phase: 0.0,
            phase_inc: -2.0 * PI * center_freq / sample_rate,
            decim_count: 0,
        }
    }

    /// Create with custom FIR taps.
    pub fn with_taps(center_freq: f64, sample_rate: f64, decimation: usize, taps: Vec<f64>) -> Self {
        let n = taps.len().max(1);
        Self {
            center_freq,
            sample_rate,
            decimation: decimation.max(1),
            taps,
            delay_line: vec![Complex64::new(0.0, 0.0); n],
            write_idx: 0,
            phase: 0.0,
            phase_inc: -2.0 * PI * center_freq / sample_rate,
            decim_count: 0,
        }
    }

    /// Process a block of input samples.
    ///
    /// Returns decimated, frequency-shifted, filtered output.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len() / self.decimation + 1);

        for &sample in input {
            // Mix (frequency translate)
            let mixed = sample * Complex64::new(self.phase.cos(), self.phase.sin());
            self.phase += self.phase_inc;
            // Wrap phase
            if self.phase > PI {
                self.phase -= 2.0 * PI;
            } else if self.phase < -PI {
                self.phase += 2.0 * PI;
            }

            // Insert into delay line
            self.delay_line[self.write_idx] = mixed;
            self.write_idx = (self.write_idx + 1) % self.taps.len();

            // Decimation
            self.decim_count += 1;
            if self.decim_count >= self.decimation {
                self.decim_count = 0;

                // FIR filter (dot product with circular buffer)
                let mut acc = Complex64::new(0.0, 0.0);
                let n = self.taps.len();
                for k in 0..n {
                    let idx = (self.write_idx + n - 1 - k) % n;
                    acc += self.delay_line[idx] * self.taps[k];
                }
                output.push(acc);
            }
        }

        output
    }

    /// Process a single sample. Returns Some(output) on decimation output.
    pub fn process_sample(&mut self, sample: Complex64) -> Option<Complex64> {
        let mixed = sample * Complex64::new(self.phase.cos(), self.phase.sin());
        self.phase += self.phase_inc;
        if self.phase > PI {
            self.phase -= 2.0 * PI;
        } else if self.phase < -PI {
            self.phase += 2.0 * PI;
        }

        self.delay_line[self.write_idx] = mixed;
        self.write_idx = (self.write_idx + 1) % self.taps.len();

        self.decim_count += 1;
        if self.decim_count >= self.decimation {
            self.decim_count = 0;
            let n = self.taps.len();
            let mut acc = Complex64::new(0.0, 0.0);
            for k in 0..n {
                let idx = (self.write_idx + n - 1 - k) % n;
                acc += self.delay_line[idx] * self.taps[k];
            }
            Some(acc)
        } else {
            None
        }
    }

    /// Set center frequency (Hz).
    pub fn set_center_frequency(&mut self, freq: f64) {
        self.center_freq = freq;
        self.phase_inc = -2.0 * PI * freq / self.sample_rate;
    }

    /// Get current center frequency (Hz).
    pub fn center_frequency(&self) -> f64 {
        self.center_freq
    }

    /// Get decimation factor.
    pub fn decimation(&self) -> usize {
        self.decimation
    }

    /// Get output sample rate.
    pub fn output_sample_rate(&self) -> f64 {
        self.sample_rate / self.decimation as f64
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.decim_count = 0;
        self.write_idx = 0;
        self.delay_line.fill(Complex64::new(0.0, 0.0));
    }
}

/// Design a simple windowed-sinc lowpass filter.
fn design_lowpass(cutoff_hz: f64, sample_rate: f64, num_taps: usize) -> Vec<f64> {
    let fc = cutoff_hz / sample_rate;
    let m = num_taps - 1;
    let mid = m as f64 / 2.0;

    let mut taps: Vec<f64> = (0..num_taps)
        .map(|i| {
            let n = i as f64 - mid;
            let sinc = if n.abs() < 1e-10 {
                2.0 * PI * fc
            } else {
                (2.0 * PI * fc * n).sin() / n
            };
            // Blackman window
            let window = 0.42 - 0.5 * (2.0 * PI * i as f64 / m as f64).cos()
                + 0.08 * (4.0 * PI * i as f64 / m as f64).cos();
            sinc * window
        })
        .collect();

    // Normalize
    let sum: f64 = taps.iter().sum();
    if sum.abs() > 1e-10 {
        for t in &mut taps {
            *t /= sum;
        }
    }

    taps
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_freq_xlating_decimation() {
        let mut xlat = FreqXlatingFirFilter::new(0.0, 1_000_000.0, 10, 31);
        let input = vec![Complex64::new(1.0, 0.0); 1000];
        let output = xlat.process_block(&input);
        assert_eq!(output.len(), 100);
    }

    #[test]
    fn test_freq_xlating_extracts_tone() {
        let fs = 48000.0;
        let f_target = 5000.0;
        let decim = 4;
        let mut xlat = FreqXlatingFirFilter::new(f_target, fs, decim, 63);

        // Generate tone at target frequency + some noise at other frequencies
        let n = 4800;
        let input: Vec<Complex64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                // Target tone
                let target = Complex64::new(
                    (2.0 * PI * f_target * t).cos(),
                    (2.0 * PI * f_target * t).sin(),
                );
                // Interferer at 15 kHz
                let interferer = Complex64::new(
                    (2.0 * PI * 15000.0 * t).cos(),
                    (2.0 * PI * 15000.0 * t).sin(),
                ) * 0.5;
                target + interferer
            })
            .collect();

        let output = xlat.process_block(&input);
        assert_eq!(output.len(), n / decim);

        // After settling, output should be near-DC (target was shifted to baseband)
        // Check last 100 samples are relatively constant phase
        let last = &output[output.len() - 100..];
        let mean_power: f64 = last.iter().map(|s| s.norm_sqr()).sum::<f64>() / last.len() as f64;
        assert!(
            mean_power > 0.1,
            "Should have signal after extraction: power = {:.4}",
            mean_power
        );
    }

    #[test]
    fn test_freq_xlating_set_frequency() {
        let mut xlat = FreqXlatingFirFilter::new(1000.0, 48000.0, 1, 31);
        assert!((xlat.center_frequency() - 1000.0).abs() < 1e-10);

        xlat.set_center_frequency(2000.0);
        assert!((xlat.center_frequency() - 2000.0).abs() < 1e-10);
    }

    #[test]
    fn test_freq_xlating_output_rate() {
        let xlat = FreqXlatingFirFilter::new(0.0, 1_000_000.0, 10, 31);
        assert!((xlat.output_sample_rate() - 100_000.0).abs() < 1e-10);
    }

    #[test]
    fn test_freq_xlating_reset() {
        let mut xlat = FreqXlatingFirFilter::new(1000.0, 48000.0, 4, 31);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        xlat.process_block(&input);

        xlat.reset();
        // After reset, delay line should be zeroed
        let output = xlat.process_block(&[Complex64::new(0.0, 0.0); 4]);
        assert_eq!(output.len(), 1);
        assert!(output[0].norm() < 1e-10);
    }

    #[test]
    fn test_freq_xlating_with_custom_taps() {
        let taps = vec![0.25, 0.5, 0.25]; // Simple 3-tap average
        let mut xlat = FreqXlatingFirFilter::with_taps(0.0, 48000.0, 2, taps);
        let input = vec![Complex64::new(1.0, 0.0); 20];
        let output = xlat.process_block(&input);
        assert_eq!(output.len(), 10);
    }

    #[test]
    fn test_freq_xlating_sample_by_sample() {
        let mut xlat = FreqXlatingFirFilter::new(0.0, 48000.0, 4, 15);
        let input = vec![Complex64::new(1.0, 0.0); 100];

        let mut count = 0;
        for &s in &input {
            if xlat.process_sample(s).is_some() {
                count += 1;
            }
        }
        assert_eq!(count, 25);
    }
}
