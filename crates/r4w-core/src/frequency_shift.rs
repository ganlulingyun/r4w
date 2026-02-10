//! # Frequency Shift
//!
//! Shifts complex IQ samples by a fixed frequency offset using an NCO
//! (Numerically Controlled Oscillator). Essential for baseband conversion,
//! channel selection, and Doppler correction.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::frequency_shift::FrequencyShift;
//!
//! let mut shifter = FrequencyShift::new(1000.0, 48000.0);
//! let input = vec![(1.0, 0.0); 48];
//! let output = shifter.process(&input);
//! assert_eq!(output.len(), 48);
//! ```

use std::f64::consts::PI;

/// NCO-based frequency shifter.
#[derive(Debug, Clone)]
pub struct FrequencyShift {
    /// Frequency shift in Hz.
    shift_hz: f64,
    /// Sample rate in Hz.
    sample_rate: f64,
    /// Phase increment per sample (radians).
    phase_inc: f64,
    /// Current phase accumulator.
    phase: f64,
    /// Total samples processed.
    count: u64,
}

impl FrequencyShift {
    /// Create a new frequency shifter.
    ///
    /// * `shift_hz` - Frequency shift in Hz (positive = up, negative = down)
    /// * `sample_rate` - Sample rate in Hz
    pub fn new(shift_hz: f64, sample_rate: f64) -> Self {
        let phase_inc = 2.0 * PI * shift_hz / sample_rate;
        Self {
            shift_hz,
            sample_rate,
            phase_inc,
            phase: 0.0,
            count: 0,
        }
    }

    /// Shift complex IQ samples by the configured frequency.
    pub fn process(&mut self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let mut output = Vec::with_capacity(input.len());

        for &(re, im) in input {
            let cos_phi = self.phase.cos();
            let sin_phi = self.phase.sin();

            // Complex multiply: (re + j*im) * (cos + j*sin)
            let out_re = re * cos_phi - im * sin_phi;
            let out_im = re * sin_phi + im * cos_phi;
            output.push((out_re, out_im));

            self.phase += self.phase_inc;
            if self.phase > PI {
                self.phase -= 2.0 * PI;
            } else if self.phase < -PI {
                self.phase += 2.0 * PI;
            }
            self.count += 1;
        }

        output
    }

    /// Shift real samples (converts to analytic signal first).
    pub fn process_real(&mut self, input: &[f64]) -> Vec<(f64, f64)> {
        let complex: Vec<(f64, f64)> = input.iter().map(|&x| (x, 0.0)).collect();
        self.process(&complex)
    }

    /// Process in-place.
    pub fn process_inplace(&mut self, samples: &mut [(f64, f64)]) {
        for sample in samples.iter_mut() {
            let cos_phi = self.phase.cos();
            let sin_phi = self.phase.sin();

            let re = sample.0;
            let im = sample.1;
            sample.0 = re * cos_phi - im * sin_phi;
            sample.1 = re * sin_phi + im * cos_phi;

            self.phase += self.phase_inc;
            if self.phase > PI {
                self.phase -= 2.0 * PI;
            } else if self.phase < -PI {
                self.phase += 2.0 * PI;
            }
            self.count += 1;
        }
    }

    /// Get the frequency shift in Hz.
    pub fn shift_hz(&self) -> f64 {
        self.shift_hz
    }

    /// Set a new frequency shift.
    pub fn set_shift_hz(&mut self, shift_hz: f64) {
        self.shift_hz = shift_hz;
        self.phase_inc = 2.0 * PI * shift_hz / self.sample_rate;
    }

    /// Get the sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Get total samples processed.
    pub fn count(&self) -> u64 {
        self.count
    }

    /// Reset the phase accumulator.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.count = 0;
    }
}

/// Shift a block of samples by a fixed frequency (one-shot, no state).
pub fn frequency_shift(
    samples: &[(f64, f64)],
    shift_hz: f64,
    sample_rate: f64,
) -> Vec<(f64, f64)> {
    let mut shifter = FrequencyShift::new(shift_hz, sample_rate);
    shifter.process(samples)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zero_shift() {
        let mut shifter = FrequencyShift::new(0.0, 48000.0);
        let input = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)];
        let output = shifter.process(&input);
        for (i, o) in input.iter().zip(output.iter()) {
            assert!((i.0 - o.0).abs() < 1e-10);
            assert!((i.1 - o.1).abs() < 1e-10);
        }
    }

    #[test]
    fn test_magnitude_preservation() {
        let mut shifter = FrequencyShift::new(1000.0, 48000.0);
        let input: Vec<(f64, f64)> = (0..100)
            .map(|i| {
                let t = i as f64 / 48000.0;
                let p = 2.0 * PI * 500.0 * t;
                (p.cos(), p.sin())
            })
            .collect();
        let output = shifter.process(&input);

        for (inp, out) in input.iter().zip(output.iter()) {
            let mag_in = (inp.0 * inp.0 + inp.1 * inp.1).sqrt();
            let mag_out = (out.0 * out.0 + out.1 * out.1).sqrt();
            assert!(
                (mag_in - mag_out).abs() < 1e-10,
                "Magnitude should be preserved"
            );
        }
    }

    #[test]
    fn test_frequency_content() {
        // A tone at 1kHz shifted by +2kHz should appear at 3kHz.
        let fs = 48000.0;
        let n = 4800; // 100ms
        let input: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let p = 2.0 * PI * 1000.0 * t;
                (p.cos(), p.sin())
            })
            .collect();

        let mut shifter = FrequencyShift::new(2000.0, fs);
        let output = shifter.process(&input);

        // Check that output is at 3kHz by correlating with a 3kHz reference.
        let mut corr_3k = 0.0;
        let mut corr_1k = 0.0;
        for i in 0..n {
            let t = i as f64 / fs;
            let ref_3k_re = (2.0 * PI * 3000.0 * t).cos();
            let ref_1k_re = (2.0 * PI * 1000.0 * t).cos();
            corr_3k += output[i].0 * ref_3k_re;
            corr_1k += output[i].0 * ref_1k_re;
        }
        assert!(
            corr_3k.abs() > corr_1k.abs() * 10.0,
            "3kHz correlation should dominate: 3k={}, 1k={}",
            corr_3k.abs(), corr_1k.abs()
        );
    }

    #[test]
    fn test_negative_shift() {
        let mut shifter = FrequencyShift::new(-1000.0, 48000.0);
        let input = vec![(1.0, 0.0); 48];
        let output = shifter.process(&input);
        assert_eq!(output.len(), 48);
    }

    #[test]
    fn test_process_real() {
        let mut shifter = FrequencyShift::new(1000.0, 48000.0);
        let input = vec![1.0, 0.0, -1.0, 0.0];
        let output = shifter.process_real(&input);
        assert_eq!(output.len(), 4);
    }

    #[test]
    fn test_inplace() {
        let mut shifter1 = FrequencyShift::new(1000.0, 48000.0);
        let mut shifter2 = FrequencyShift::new(1000.0, 48000.0);
        let input = vec![(1.0, 0.5), (0.3, -0.2), (-0.7, 0.8)];

        let output = shifter1.process(&input);
        let mut inplace = input.clone();
        shifter2.process_inplace(&mut inplace);

        for (o, ip) in output.iter().zip(inplace.iter()) {
            assert!((o.0 - ip.0).abs() < 1e-10);
            assert!((o.1 - ip.1).abs() < 1e-10);
        }
    }

    #[test]
    fn test_set_shift() {
        let mut shifter = FrequencyShift::new(1000.0, 48000.0);
        assert!((shifter.shift_hz() - 1000.0).abs() < 1e-10);
        shifter.set_shift_hz(2000.0);
        assert!((shifter.shift_hz() - 2000.0).abs() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut shifter = FrequencyShift::new(1000.0, 48000.0);
        shifter.process(&[(1.0, 0.0); 100]);
        assert_eq!(shifter.count(), 100);
        shifter.reset();
        assert_eq!(shifter.count(), 0);
    }

    #[test]
    fn test_one_shot() {
        let input = vec![(1.0, 0.0); 10];
        let output = frequency_shift(&input, 1000.0, 48000.0);
        assert_eq!(output.len(), 10);
    }

    #[test]
    fn test_count() {
        let mut shifter = FrequencyShift::new(0.0, 48000.0);
        shifter.process(&[(1.0, 0.0); 50]);
        assert_eq!(shifter.count(), 50);
    }
}
