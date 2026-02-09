//! Signal Source / Waveform Generator
//!
//! Generates standard test signals: tone (CW), chirp, noise, square wave, etc.
//! Essential for testing receiver chains without external hardware.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::signal_source::{SignalSource, SignalType};
//!
//! // Generate a 1 kHz tone at 48 kHz sample rate
//! let mut src = SignalSource::new(SignalType::Tone {
//!     frequency: 1000.0,
//!     amplitude: 1.0,
//! }, 48000.0);
//!
//! let samples = src.generate(1024);
//! assert_eq!(samples.len(), 1024);
//! ```

use num_complex::Complex64;

/// Signal type for the source generator.
#[derive(Debug, Clone)]
pub enum SignalType {
    /// Continuous wave (single tone)
    Tone {
        frequency: f64,
        amplitude: f64,
    },
    /// Two-tone signal for intermodulation testing
    TwoTone {
        freq1: f64,
        freq2: f64,
        amplitude: f64,
    },
    /// Linear chirp (swept frequency)
    Chirp {
        start_freq: f64,
        end_freq: f64,
        amplitude: f64,
        sweep_time: f64,
    },
    /// White Gaussian noise
    Noise {
        power: f64,
    },
    /// Square wave
    Square {
        frequency: f64,
        amplitude: f64,
    },
    /// Constant DC value
    Dc {
        value: Complex64,
    },
    /// Impulse (single spike then zeros)
    Impulse {
        amplitude: f64,
    },
}

/// Signal source / waveform generator.
#[derive(Debug, Clone)]
pub struct SignalSource {
    signal_type: SignalType,
    sample_rate: f64,
    phase: f64,
    sample_count: u64,
    rng_state: u64,
}

impl SignalSource {
    /// Create a new signal source.
    pub fn new(signal_type: SignalType, sample_rate: f64) -> Self {
        Self {
            signal_type,
            sample_rate,
            phase: 0.0,
            sample_count: 0,
            rng_state: 0x12345678_9ABCDEF0,
        }
    }

    /// Create a tone source.
    pub fn tone(frequency: f64, amplitude: f64, sample_rate: f64) -> Self {
        Self::new(
            SignalType::Tone {
                frequency,
                amplitude,
            },
            sample_rate,
        )
    }

    /// Create a noise source.
    pub fn noise(power: f64, sample_rate: f64) -> Self {
        Self::new(SignalType::Noise { power }, sample_rate)
    }

    /// Generate a block of samples.
    pub fn generate(&mut self, num_samples: usize) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(num_samples);
        for _ in 0..num_samples {
            output.push(self.next_sample());
        }
        output
    }

    /// Generate a single sample.
    pub fn next_sample(&mut self) -> Complex64 {
        let t = self.sample_count as f64 / self.sample_rate;
        let sample = match &self.signal_type {
            SignalType::Tone {
                frequency,
                amplitude,
            } => {
                let phase = 2.0 * std::f64::consts::PI * frequency * t;
                Complex64::new(amplitude * phase.cos(), amplitude * phase.sin())
            }
            SignalType::TwoTone {
                freq1,
                freq2,
                amplitude,
            } => {
                let p1 = 2.0 * std::f64::consts::PI * freq1 * t;
                let p2 = 2.0 * std::f64::consts::PI * freq2 * t;
                let a = amplitude * 0.5;
                Complex64::new(a * (p1.cos() + p2.cos()), a * (p1.sin() + p2.sin()))
            }
            SignalType::Chirp {
                start_freq,
                end_freq,
                amplitude,
                sweep_time,
            } => {
                let t_mod = t % sweep_time;
                let k = (end_freq - start_freq) / sweep_time;
                let freq = start_freq + k * t_mod;
                let phase = 2.0
                    * std::f64::consts::PI
                    * (start_freq * t_mod + 0.5 * k * t_mod * t_mod);
                Complex64::new(amplitude * phase.cos(), amplitude * phase.sin())
                    * Complex64::new(1.0, 0.0)
                    * if freq.is_finite() { 1.0 } else { 0.0 }
            }
            SignalType::Noise { power } => {
                let p = *power;
                // Box-Muller transform for Gaussian noise
                let (u1, u2) = self.next_uniform_pair();
                let sigma = (p / 2.0).sqrt(); // split across I and Q
                let r = (-2.0 * u1.ln()).sqrt();
                let theta = 2.0 * std::f64::consts::PI * u2;
                Complex64::new(sigma * r * theta.cos(), sigma * r * theta.sin())
            }
            SignalType::Square {
                frequency,
                amplitude,
            } => {
                let phase = 2.0 * std::f64::consts::PI * frequency * t;
                let sign = if phase.sin() >= 0.0 { 1.0 } else { -1.0 };
                Complex64::new(amplitude * sign, 0.0)
            }
            SignalType::Dc { value } => *value,
            SignalType::Impulse { amplitude } => {
                if self.sample_count == 0 {
                    Complex64::new(*amplitude, 0.0)
                } else {
                    Complex64::new(0.0, 0.0)
                }
            }
        };
        self.sample_count += 1;
        sample
    }

    /// Simple LCG-based PRNG for noise generation.
    fn next_uniform_pair(&mut self) -> (f64, f64) {
        self.rng_state = self.rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let u1 = (self.rng_state >> 11) as f64 / (1u64 << 53) as f64;
        let u1 = u1.max(1e-20); // avoid log(0)
        self.rng_state = self.rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        let u2 = (self.rng_state >> 11) as f64 / (1u64 << 53) as f64;
        (u1, u2)
    }

    /// Reset the source to initial state.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.sample_count = 0;
        self.rng_state = 0x12345678_9ABCDEF0;
    }

    /// Get the sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tone_frequency() {
        let freq = 1000.0;
        let sr = 48000.0;
        let mut src = SignalSource::tone(freq, 1.0, sr);
        let samples = src.generate(480); // 10ms at 48kHz

        // Check amplitude is ~1.0
        for s in &samples {
            assert!(
                (s.norm() - 1.0).abs() < 0.01,
                "Tone amplitude should be 1.0"
            );
        }

        // Check frequency via zero crossings of real part
        let zero_crossings = samples
            .windows(2)
            .filter(|w| w[0].re * w[1].re < 0.0)
            .count();
        // Expect ~2 crossings per cycle, 10ms of 1kHz = 10 cycles = ~20 crossings
        assert!(
            (zero_crossings as f64 - 20.0).abs() < 3.0,
            "Should have ~20 zero crossings: got {zero_crossings}"
        );
    }

    #[test]
    fn test_noise_power() {
        let power = 1.0;
        let mut src = SignalSource::noise(power, 48000.0);
        let samples = src.generate(10000);

        // Estimate power: mean |s|^2
        let est_power: f64 =
            samples.iter().map(|s| s.norm_sqr()).sum::<f64>() / samples.len() as f64;
        assert!(
            (est_power - power).abs() < 0.3,
            "Noise power should be ~{power}: got {est_power}"
        );
    }

    #[test]
    fn test_chirp_increasing_frequency() {
        let mut src = SignalSource::new(
            SignalType::Chirp {
                start_freq: 100.0,
                end_freq: 1000.0,
                amplitude: 1.0,
                sweep_time: 0.01, // 10ms sweep
            },
            48000.0,
        );
        let samples = src.generate(480);
        // Chirp should have increasing instantaneous frequency
        assert_eq!(samples.len(), 480);
        // All samples should have unit amplitude
        for s in &samples {
            assert!(
                (s.norm() - 1.0).abs() < 0.1,
                "Chirp amplitude should be ~1.0"
            );
        }
    }

    #[test]
    fn test_dc_constant() {
        let val = Complex64::new(0.5, -0.3);
        let mut src = SignalSource::new(SignalType::Dc { value: val }, 48000.0);
        let samples = src.generate(100);
        for s in &samples {
            assert!((s - val).norm() < 1e-10, "DC should be constant");
        }
    }

    #[test]
    fn test_impulse() {
        let mut src = SignalSource::new(
            SignalType::Impulse { amplitude: 5.0 },
            48000.0,
        );
        let samples = src.generate(10);
        assert!((samples[0].re - 5.0).abs() < 1e-10);
        for s in &samples[1..] {
            assert!(s.norm() < 1e-10, "Impulse should be zero after first sample");
        }
    }

    #[test]
    fn test_square_wave() {
        let mut src = SignalSource::new(
            SignalType::Square {
                frequency: 1000.0,
                amplitude: 2.0,
            },
            48000.0,
        );
        let samples = src.generate(480);
        for s in &samples {
            assert!(
                (s.re.abs() - 2.0).abs() < 0.01,
                "Square wave amplitude should be +/-2.0"
            );
        }
    }

    #[test]
    fn test_two_tone() {
        let mut src = SignalSource::new(
            SignalType::TwoTone {
                freq1: 1000.0,
                freq2: 2000.0,
                amplitude: 1.0,
            },
            48000.0,
        );
        let samples = src.generate(480);
        // Peak amplitude should be around 1.0 (two 0.5-amplitude tones)
        let max_mag = samples.iter().map(|s| s.norm()).fold(0.0f64, f64::max);
        assert!(
            max_mag > 0.5 && max_mag < 1.1,
            "Two-tone peak should be ~1.0: got {max_mag}"
        );
    }

    #[test]
    fn test_reset() {
        let mut src = SignalSource::tone(1000.0, 1.0, 48000.0);
        let batch1 = src.generate(100);
        src.reset();
        let batch2 = src.generate(100);
        for (a, b) in batch1.iter().zip(batch2.iter()) {
            assert!(
                (a - b).norm() < 1e-10,
                "Reset should produce identical output"
            );
        }
    }
}
