//! # Sample Rate Converter
//!
//! Arbitrary rational sample rate conversion using polyphase FIR interpolation
//! and decimation. Converts between any two sample rates expressed as a rational
//! ratio L/M (interpolate by L, then decimate by M).
//!
//! ## Common Conversions
//! - 44.1 kHz ↔ 48 kHz (L=160, M=147)
//! - 8 kHz → 48 kHz (L=6, M=1)
//! - 2.048 MHz → 256 kHz (L=1, M=8)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::sample_rate_converter::SampleRateConverter;
//!
//! // Convert 48kHz → 44.1kHz
//! let mut src = SampleRateConverter::new(48000.0, 44100.0, 32);
//! let input = vec![1.0_f64; 480];
//! let output = src.process(&input);
//! // Output should be approximately 441 samples
//! assert!((output.len() as f64 - 441.0).abs() < 5.0);
//! ```

use std::f64::consts::PI;

/// Rational sample rate converter.
#[derive(Debug, Clone)]
pub struct SampleRateConverter {
    /// Interpolation factor.
    interp: usize,
    /// Decimation factor.
    decim: usize,
    /// Polyphase filter branches (interp branches of length ceil(ntaps/interp)).
    branches: Vec<Vec<f64>>,
    /// Input buffer (delay line).
    buffer: Vec<f64>,
    /// Buffer write position.
    buf_pos: usize,
    /// Decimation phase counter.
    decim_phase: usize,
    /// Input sample rate.
    input_rate: f64,
    /// Output sample rate.
    output_rate: f64,
    /// Total input samples processed.
    samples_in: u64,
    /// Total output samples produced.
    samples_out: u64,
}

impl SampleRateConverter {
    /// Create a new sample rate converter.
    ///
    /// # Arguments
    /// * `input_rate` - Input sample rate (Hz)
    /// * `output_rate` - Output sample rate (Hz)
    /// * `taps_per_phase` - Number of filter taps per polyphase branch
    pub fn new(input_rate: f64, output_rate: f64, taps_per_phase: usize) -> Self {
        let taps_per_phase = taps_per_phase.max(4);
        let (interp, decim) = find_rational(input_rate, output_rate);

        // Design the prototype lowpass filter.
        let total_taps = interp * taps_per_phase;
        let cutoff = 0.5 / (interp.max(decim) as f64);
        let prototype = design_lowpass(total_taps, cutoff);

        // Decompose into polyphase branches.
        let mut branches = vec![vec![0.0; taps_per_phase]; interp];
        for (i, &tap) in prototype.iter().enumerate() {
            let branch = i % interp;
            let phase = i / interp;
            if phase < taps_per_phase {
                branches[branch][phase] = tap * interp as f64;
            }
        }

        let buf_len = taps_per_phase;

        Self {
            interp,
            decim,
            branches,
            buffer: vec![0.0; buf_len],
            buf_pos: 0,
            decim_phase: 0,
            input_rate,
            output_rate,
            samples_in: 0,
            samples_out: 0,
        }
    }

    /// Create a converter for common audio rate conversions.
    pub fn audio(input_rate: f64, output_rate: f64) -> Self {
        Self::new(input_rate, output_rate, 32)
    }

    /// Process a block of input samples. Returns resampled output.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::new();
        let branch_len = self.branches[0].len();

        for &sample in input {
            // Insert sample into circular buffer.
            self.buffer[self.buf_pos] = sample;
            self.buf_pos = (self.buf_pos + 1) % branch_len;
            self.samples_in += 1;

            // For each input sample, produce interp output candidates.
            for phase in 0..self.interp {
                if self.decim_phase == 0 {
                    // Compute polyphase FIR output for this branch.
                    let branch = &self.branches[phase];
                    let mut sum = 0.0;
                    for (k, &tap) in branch.iter().enumerate() {
                        let idx = (self.buf_pos + branch_len - 1 - k) % branch_len;
                        sum += tap * self.buffer[idx];
                    }
                    output.push(sum);
                    self.samples_out += 1;
                }
                self.decim_phase = (self.decim_phase + 1) % self.decim;
            }
        }

        output
    }

    /// Process complex IQ samples.
    pub fn process_complex(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let re: Vec<f64> = input.iter().map(|s| s.0).collect();
        let im: Vec<f64> = input.iter().map(|s| s.1).collect();

        // Process I and Q independently (need separate converter instances in practice).
        // For simplicity, use a single instance and interleave.
        let mut out_re = Vec::new();
        let mut out_im = Vec::new();

        // Save and restore state for Q channel.
        let saved_buffer = self.buffer.clone();
        let saved_pos = self.buf_pos;
        let saved_phase = self.decim_phase;
        let saved_in = self.samples_in;
        let saved_out = self.samples_out;

        out_re = self.process(&re);

        // Restore state for Q channel.
        self.buffer = saved_buffer;
        self.buf_pos = saved_pos;
        self.decim_phase = saved_phase;
        self.samples_in = saved_in;
        self.samples_out = saved_out;

        out_im = self.process(&im);

        let len = out_re.len().min(out_im.len());
        (0..len).map(|i| (out_re[i], out_im[i])).collect()
    }

    /// Get the interpolation factor.
    pub fn interpolation(&self) -> usize {
        self.interp
    }

    /// Get the decimation factor.
    pub fn decimation(&self) -> usize {
        self.decim
    }

    /// Get the actual output rate.
    pub fn actual_output_rate(&self) -> f64 {
        self.input_rate * self.interp as f64 / self.decim as f64
    }

    /// Get total input samples processed.
    pub fn samples_in(&self) -> u64 {
        self.samples_in
    }

    /// Get total output samples produced.
    pub fn samples_out(&self) -> u64 {
        self.samples_out
    }

    /// Reset the converter state.
    pub fn reset(&mut self) {
        let len = self.buffer.len();
        self.buffer = vec![0.0; len];
        self.buf_pos = 0;
        self.decim_phase = 0;
        self.samples_in = 0;
        self.samples_out = 0;
    }
}

/// Find rational L/M approximation for rate conversion.
fn find_rational(input_rate: f64, output_rate: f64) -> (usize, usize) {
    let ratio = output_rate / input_rate;

    // Try to find exact rational fraction.
    let max_factor = 1000;
    let mut best_l = 1usize;
    let mut best_m = 1usize;
    let mut best_error = f64::MAX;

    for m in 1..=max_factor {
        let l = (ratio * m as f64).round() as usize;
        if l == 0 {
            continue;
        }
        let error = (l as f64 / m as f64 - ratio).abs();
        if error < best_error {
            best_error = error;
            best_l = l;
            best_m = m;
            if error < 1e-10 {
                break;
            }
        }
    }

    // Reduce by GCD.
    let g = gcd(best_l, best_m);
    (best_l / g, best_m / g)
}

fn gcd(mut a: usize, mut b: usize) -> usize {
    while b != 0 {
        let t = b;
        b = a % b;
        a = t;
    }
    a
}

/// Design a lowpass FIR filter using windowed-sinc (Hamming window).
fn design_lowpass(num_taps: usize, cutoff: f64) -> Vec<f64> {
    let n = num_taps;
    let m = (n - 1) as f64;

    let taps: Vec<f64> = (0..n)
        .map(|i| {
            let x = i as f64 - m / 2.0;
            let sinc = if x.abs() < 1e-12 {
                2.0 * cutoff
            } else {
                (2.0 * PI * cutoff * x).sin() / (PI * x)
            };
            let window = 0.54 - 0.46 * (2.0 * PI * i as f64 / m).cos();
            sinc * window
        })
        .collect();

    taps
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_unity_conversion() {
        // Same rate in/out should pass through roughly unchanged.
        let mut src = SampleRateConverter::new(48000.0, 48000.0, 16);
        assert_eq!(src.interpolation(), 1);
        assert_eq!(src.decimation(), 1);
    }

    #[test]
    fn test_2x_upsample() {
        let mut src = SampleRateConverter::new(24000.0, 48000.0, 16);
        assert_eq!(src.interpolation(), 2);
        assert_eq!(src.decimation(), 1);
        let input = vec![1.0; 100];
        let output = src.process(&input);
        assert_eq!(output.len(), 200);
    }

    #[test]
    fn test_2x_downsample() {
        let mut src = SampleRateConverter::new(48000.0, 24000.0, 16);
        assert_eq!(src.interpolation(), 1);
        assert_eq!(src.decimation(), 2);
        let input = vec![1.0; 100];
        let output = src.process(&input);
        assert_eq!(output.len(), 50);
    }

    #[test]
    fn test_44100_to_48000() {
        let mut src = SampleRateConverter::new(44100.0, 48000.0, 16);
        let (l, m) = (src.interpolation(), src.decimation());
        // 48000/44100 = 160/147
        assert_eq!(l, 160);
        assert_eq!(m, 147);
    }

    #[test]
    fn test_output_rate_accuracy() {
        let src = SampleRateConverter::new(44100.0, 48000.0, 16);
        assert!((src.actual_output_rate() - 48000.0).abs() < 1.0);
    }

    #[test]
    fn test_dc_passthrough() {
        // DC signal should pass through with approximately unity gain.
        let mut src = SampleRateConverter::new(48000.0, 44100.0, 32);
        let input = vec![1.0; 1000];
        let output = src.process(&input);
        // After settling, output should be near 1.0.
        let avg: f64 = output[100..].iter().sum::<f64>() / (output.len() - 100) as f64;
        assert!(
            (avg - 1.0).abs() < 0.2,
            "DC passthrough average {} should be near 1.0",
            avg
        );
    }

    #[test]
    fn test_output_count() {
        let mut src = SampleRateConverter::new(48000.0, 16000.0, 16);
        let input = vec![0.5; 480];
        let output = src.process(&input);
        // 480 * (16000/48000) = 160
        assert!((output.len() as f64 - 160.0).abs() < 5.0);
    }

    #[test]
    fn test_reset() {
        let mut src = SampleRateConverter::new(48000.0, 24000.0, 16);
        src.process(&[1.0; 100]);
        assert!(src.samples_in() > 0);
        src.reset();
        assert_eq!(src.samples_in(), 0);
        assert_eq!(src.samples_out(), 0);
    }

    #[test]
    fn test_gcd() {
        assert_eq!(gcd(12, 8), 4);
        assert_eq!(gcd(160, 147), 1);
        assert_eq!(gcd(6, 1), 1);
    }

    #[test]
    fn test_find_rational() {
        let (l, m) = find_rational(44100.0, 48000.0);
        assert_eq!(l, 160);
        assert_eq!(m, 147);

        let (l, m) = find_rational(48000.0, 24000.0);
        assert_eq!(l, 1);
        assert_eq!(m, 2);
    }
}
