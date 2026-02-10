//! # Signal Generator
//!
//! Configurable test signal generator for SDR development and testing.
//! Generates tone, chirp, noise, multitone, and pulse waveforms with
//! precise frequency and amplitude control.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::signal_generator::{SignalGenerator, SignalType};
//!
//! let mut gen = SignalGenerator::new(48000.0);
//! gen.set_signal(SignalType::Tone { freq_hz: 1000.0 });
//! gen.set_amplitude(0.5);
//!
//! let samples = gen.generate(100);
//! assert_eq!(samples.len(), 100);
//! ```

use std::f64::consts::PI;

/// Type of signal to generate.
#[derive(Debug, Clone)]
pub enum SignalType {
    /// Single tone at a fixed frequency.
    Tone { freq_hz: f64 },
    /// Two-tone (for intermod testing).
    TwoTone { freq1_hz: f64, freq2_hz: f64 },
    /// Linear chirp sweep.
    Chirp {
        start_hz: f64,
        stop_hz: f64,
        sweep_time_s: f64,
    },
    /// Gaussian white noise.
    Noise,
    /// Impulse (single sample = 1, rest = 0).
    Impulse,
    /// Square wave.
    Square { freq_hz: f64 },
    /// Sawtooth wave.
    Sawtooth { freq_hz: f64 },
    /// DC offset.
    Dc,
}

/// Configurable signal generator.
#[derive(Debug, Clone)]
pub struct SignalGenerator {
    /// Sample rate (Hz).
    sample_rate: f64,
    /// Signal type.
    signal_type: SignalType,
    /// Amplitude (0.0 to 1.0).
    amplitude: f64,
    /// DC offset.
    dc_offset: f64,
    /// Phase accumulator (radians).
    phase: f64,
    /// Sample counter.
    sample_count: u64,
    /// PRNG state for noise.
    rng_state: u64,
}

impl SignalGenerator {
    /// Create a new signal generator at the given sample rate.
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            signal_type: SignalType::Tone { freq_hz: 1000.0 },
            amplitude: 1.0,
            dc_offset: 0.0,
            phase: 0.0,
            sample_count: 0,
            rng_state: 0x12345678_9ABCDEF0,
        }
    }

    /// Set the signal type.
    pub fn set_signal(&mut self, signal_type: SignalType) {
        self.signal_type = signal_type;
        self.phase = 0.0;
        self.sample_count = 0;
    }

    /// Set the amplitude (0.0 to 1.0).
    pub fn set_amplitude(&mut self, amplitude: f64) {
        self.amplitude = amplitude.clamp(0.0, 10.0);
    }

    /// Set a DC offset.
    pub fn set_dc_offset(&mut self, offset: f64) {
        self.dc_offset = offset;
    }

    /// Generate N complex IQ samples.
    pub fn generate(&mut self, num_samples: usize) -> Vec<(f64, f64)> {
        let mut output = Vec::with_capacity(num_samples);
        for _ in 0..num_samples {
            let sample = self.next_sample();
            output.push(sample);
        }
        output
    }

    /// Generate N real samples (I channel only).
    pub fn generate_real(&mut self, num_samples: usize) -> Vec<f64> {
        let mut output = Vec::with_capacity(num_samples);
        for _ in 0..num_samples {
            let (re, _) = self.next_sample();
            output.push(re);
        }
        output
    }

    fn next_sample(&mut self) -> (f64, f64) {
        let t = self.sample_count as f64 / self.sample_rate;
        let sample = match &self.signal_type {
            SignalType::Tone { freq_hz } => {
                let phase_inc = 2.0 * PI * freq_hz / self.sample_rate;
                let s = (self.amplitude * self.phase.cos() + self.dc_offset,
                         self.amplitude * self.phase.sin());
                self.phase += phase_inc;
                if self.phase > 2.0 * PI {
                    self.phase -= 2.0 * PI;
                }
                s
            }
            SignalType::TwoTone { freq1_hz, freq2_hz } => {
                let p1 = 2.0 * PI * freq1_hz * t;
                let p2 = 2.0 * PI * freq2_hz * t;
                let amp = self.amplitude * 0.5;
                (amp * (p1.cos() + p2.cos()) + self.dc_offset,
                 amp * (p1.sin() + p2.sin()))
            }
            SignalType::Chirp { start_hz, stop_hz, sweep_time_s } => {
                let sweep_t = t % sweep_time_s;
                let inst_freq = start_hz + (stop_hz - start_hz) * sweep_t / sweep_time_s;
                let phase = 2.0 * PI * (start_hz * sweep_t
                    + 0.5 * (stop_hz - start_hz) * sweep_t * sweep_t / sweep_time_s);
                (self.amplitude * phase.cos() + self.dc_offset,
                 self.amplitude * phase.sin())
            }
            SignalType::Noise => {
                let (n1, n2) = self.gaussian_pair();
                (self.amplitude * n1 + self.dc_offset,
                 self.amplitude * n2)
            }
            SignalType::Impulse => {
                if self.sample_count == 0 {
                    (self.amplitude + self.dc_offset, 0.0)
                } else {
                    (self.dc_offset, 0.0)
                }
            }
            SignalType::Square { freq_hz } => {
                let period = self.sample_rate / freq_hz;
                let phase = (self.sample_count as f64 % period) / period;
                let val = if phase < 0.5 { self.amplitude } else { -self.amplitude };
                (val + self.dc_offset, 0.0)
            }
            SignalType::Sawtooth { freq_hz } => {
                let period = self.sample_rate / freq_hz;
                let phase = (self.sample_count as f64 % period) / period;
                let val = self.amplitude * (2.0 * phase - 1.0);
                (val + self.dc_offset, 0.0)
            }
            SignalType::Dc => {
                (self.amplitude + self.dc_offset, 0.0)
            }
        };

        self.sample_count += 1;
        sample
    }

    fn gaussian_pair(&mut self) -> (f64, f64) {
        // Box-Muller using xorshift64.
        let u1 = self.next_uniform();
        let u2 = self.next_uniform();
        let r = (-2.0 * u1.max(1e-30).ln()).sqrt();
        let theta = 2.0 * PI * u2;
        (r * theta.cos(), r * theta.sin())
    }

    fn next_uniform(&mut self) -> f64 {
        self.rng_state ^= self.rng_state << 13;
        self.rng_state ^= self.rng_state >> 7;
        self.rng_state ^= self.rng_state << 17;
        (self.rng_state as f64) / (u64::MAX as f64)
    }

    /// Get the sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Get total samples generated.
    pub fn sample_count(&self) -> u64 {
        self.sample_count
    }

    /// Reset the generator.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.sample_count = 0;
        self.rng_state = 0x12345678_9ABCDEF0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tone_output_length() {
        let mut gen = SignalGenerator::new(48000.0);
        gen.set_signal(SignalType::Tone { freq_hz: 1000.0 });
        let output = gen.generate(100);
        assert_eq!(output.len(), 100);
    }

    #[test]
    fn test_tone_amplitude() {
        let mut gen = SignalGenerator::new(48000.0);
        gen.set_signal(SignalType::Tone { freq_hz: 1000.0 });
        gen.set_amplitude(0.5);
        let output = gen.generate(1000);
        let max_mag = output
            .iter()
            .map(|(r, i)| (r * r + i * i).sqrt())
            .fold(0.0_f64, f64::max);
        assert!((max_mag - 0.5).abs() < 0.05);
    }

    #[test]
    fn test_two_tone() {
        let mut gen = SignalGenerator::new(48000.0);
        gen.set_signal(SignalType::TwoTone {
            freq1_hz: 1000.0,
            freq2_hz: 2000.0,
        });
        let output = gen.generate(480);
        assert_eq!(output.len(), 480);
        // Signal should have energy.
        let energy: f64 = output.iter().map(|(r, i)| r * r + i * i).sum();
        assert!(energy > 0.0);
    }

    #[test]
    fn test_chirp() {
        let mut gen = SignalGenerator::new(48000.0);
        gen.set_signal(SignalType::Chirp {
            start_hz: 100.0,
            stop_hz: 5000.0,
            sweep_time_s: 0.01,
        });
        let output = gen.generate(480);
        assert_eq!(output.len(), 480);
    }

    #[test]
    fn test_noise() {
        let mut gen = SignalGenerator::new(48000.0);
        gen.set_signal(SignalType::Noise);
        let output = gen.generate(1000);
        // Noise should have varying values.
        let unique: std::collections::HashSet<u64> = output
            .iter()
            .map(|(r, _)| r.to_bits())
            .collect();
        assert!(unique.len() > 100, "Noise should have many unique values");
    }

    #[test]
    fn test_impulse() {
        let mut gen = SignalGenerator::new(48000.0);
        gen.set_signal(SignalType::Impulse);
        gen.set_amplitude(1.0);
        let output = gen.generate(10);
        assert!((output[0].0 - 1.0).abs() < 1e-10);
        for i in 1..10 {
            assert!((output[i].0 - 0.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_square_wave() {
        let mut gen = SignalGenerator::new(48000.0);
        gen.set_signal(SignalType::Square { freq_hz: 1000.0 });
        let output = gen.generate(48);
        // 48 samples at 48kHz = 1 period at 1kHz. First half +1, second half -1.
        assert!(output[0].0 > 0.0);
        assert!(output[30].0 < 0.0);
    }

    #[test]
    fn test_dc() {
        let mut gen = SignalGenerator::new(48000.0);
        gen.set_signal(SignalType::Dc);
        gen.set_amplitude(0.75);
        let output = gen.generate(10);
        for s in &output {
            assert!((s.0 - 0.75).abs() < 1e-10);
        }
    }

    #[test]
    fn test_dc_offset() {
        let mut gen = SignalGenerator::new(48000.0);
        gen.set_signal(SignalType::Dc);
        gen.set_amplitude(0.0);
        gen.set_dc_offset(0.5);
        let output = gen.generate(10);
        for s in &output {
            assert!((s.0 - 0.5).abs() < 1e-10);
        }
    }

    #[test]
    fn test_reset() {
        let mut gen = SignalGenerator::new(48000.0);
        gen.set_signal(SignalType::Tone { freq_hz: 1000.0 });
        let out1 = gen.generate(10);
        gen.reset();
        let out2 = gen.generate(10);
        assert_eq!(out1, out2);
    }
}
