//! Colored Noise Generator
//!
//! Generates noise with various spectral shapes by filtering white Gaussian
//! noise through IIR shaping filters.
//!
//! ## Noise Colors
//!
//! | Color  | PSD     | Slope       | Application                    |
//! |--------|---------|-------------|--------------------------------|
//! | White  | Flat    | 0 dB/oct    | AWGN channel, reference        |
//! | Pink   | 1/f     | -3 dB/oct   | Flicker noise, environmental   |
//! | Brown  | 1/f²    | -6 dB/oct   | Brownian motion, drift         |
//! | Blue   | f       | +3 dB/oct   | Dithering                      |
//! | Violet | f²      | +6 dB/oct   | Differentiated noise           |
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::noise::{NoiseGenerator, NoiseColor};
//!
//! let mut gen = NoiseGenerator::new(NoiseColor::Pink, 42);
//! let samples = gen.generate(1000);
//! assert_eq!(samples.len(), 1000);
//! ```

use num_complex::Complex64;

/// Noise color / spectral shape.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum NoiseColor {
    /// Flat spectrum (0 dB/octave).
    White,
    /// 1/f spectrum (-3 dB/octave). Also called flicker noise.
    Pink,
    /// 1/f² spectrum (-6 dB/octave). Also called red noise or Brownian noise.
    Brown,
    /// f spectrum (+3 dB/octave).
    Blue,
    /// f² spectrum (+6 dB/octave).
    Violet,
}

impl NoiseColor {
    /// Spectral slope in dB/octave.
    pub fn slope_db_per_octave(&self) -> f64 {
        match self {
            NoiseColor::White => 0.0,
            NoiseColor::Pink => -3.0,
            NoiseColor::Brown => -6.0,
            NoiseColor::Blue => 3.0,
            NoiseColor::Violet => 6.0,
        }
    }
}

/// Noise generator with configurable spectral shape.
#[derive(Debug, Clone)]
pub struct NoiseGenerator {
    color: NoiseColor,
    /// Simple xorshift64 PRNG state
    rng_state: u64,
    /// IIR filter state for pink noise (Voss-McCartney approximation)
    pink_state: [f64; 7],
    pink_running_sum: f64,
    pink_counter: u32,
    /// Integrator state for brown noise
    brown_state: f64,
    /// Previous sample for blue/violet differentiation
    prev_white: f64,
    prev_blue: f64,
    /// Output amplitude
    amplitude: f64,
}

impl NoiseGenerator {
    /// Create a new noise generator with the given color and seed.
    pub fn new(color: NoiseColor, seed: u64) -> Self {
        Self {
            color,
            rng_state: seed.max(1),
            pink_state: [0.0; 7],
            pink_running_sum: 0.0,
            pink_counter: 0,
            brown_state: 0.0,
            prev_white: 0.0,
            prev_blue: 0.0,
            amplitude: 1.0,
        }
    }

    /// Set output amplitude.
    pub fn set_amplitude(&mut self, amplitude: f64) {
        self.amplitude = amplitude;
    }

    /// Generate white Gaussian noise sample using Box-Muller transform.
    fn white_sample(&mut self) -> f64 {
        let u1 = self.uniform();
        let u2 = self.uniform();
        // Box-Muller
        let r = (-2.0 * u1.max(1e-30).ln()).sqrt();
        r * (2.0 * std::f64::consts::PI * u2).cos()
    }

    /// Uniform random [0, 1)
    fn uniform(&mut self) -> f64 {
        self.rng_state ^= self.rng_state << 13;
        self.rng_state ^= self.rng_state >> 7;
        self.rng_state ^= self.rng_state << 17;
        (self.rng_state as f64) / (u64::MAX as f64)
    }

    /// Generate a single noise sample.
    pub fn sample(&mut self) -> f64 {
        let s = match self.color {
            NoiseColor::White => self.white_sample(),

            NoiseColor::Pink => {
                // Voss-McCartney algorithm: sum of staggered random values
                // Updated at different rates (powers of 2)
                self.pink_counter = self.pink_counter.wrapping_add(1);
                let last_val = self.pink_counter;

                // Find which generators to update (trailing zeros)
                for i in 0..7 {
                    if last_val & (1 << i) != 0 {
                        // Update generator i
                        self.pink_running_sum -= self.pink_state[i];
                        self.pink_state[i] = self.white_sample();
                        self.pink_running_sum += self.pink_state[i];
                        break;
                    }
                }

                // Add a white noise component for high-frequency content
                (self.pink_running_sum + self.white_sample()) / 4.0
            }

            NoiseColor::Brown => {
                // Integrate white noise
                let w = self.white_sample();
                self.brown_state += w * 0.02; // Scale factor for stability
                // Soft clip to prevent drift
                self.brown_state = self.brown_state.clamp(-10.0, 10.0);
                self.brown_state
            }

            NoiseColor::Blue => {
                // Differentiate white noise
                let w = self.white_sample();
                let blue = w - self.prev_white;
                self.prev_white = w;
                blue
            }

            NoiseColor::Violet => {
                // Differentiate twice (differentiate blue noise)
                let w = self.white_sample();
                let blue = w - self.prev_white;
                self.prev_white = w;
                let violet = blue - self.prev_blue;
                self.prev_blue = blue;
                violet
            }
        };

        s * self.amplitude
    }

    /// Generate a block of real-valued noise samples.
    pub fn generate(&mut self, num_samples: usize) -> Vec<f64> {
        (0..num_samples).map(|_| self.sample()).collect()
    }

    /// Generate complex noise samples (independent I/Q).
    pub fn generate_complex(&mut self, num_samples: usize) -> Vec<Complex64> {
        (0..num_samples)
            .map(|_| {
                let re = self.sample();
                let im = self.sample();
                Complex64::new(re, im)
            })
            .collect()
    }

    /// Get the noise color.
    pub fn color(&self) -> NoiseColor {
        self.color
    }

    /// Reset generator state (keeps seed-derived state).
    pub fn reset(&mut self) {
        self.pink_state = [0.0; 7];
        self.pink_running_sum = 0.0;
        self.pink_counter = 0;
        self.brown_state = 0.0;
        self.prev_white = 0.0;
        self.prev_blue = 0.0;
    }
}

/// Generate AWGN (Additive White Gaussian Noise) with specified power in dB.
///
/// Convenience function for adding noise to a signal.
pub fn awgn_complex(num_samples: usize, power_db: f64, seed: u64) -> Vec<Complex64> {
    let power_linear = 10.0f64.powf(power_db / 10.0);
    let sigma = (power_linear / 2.0).sqrt(); // Per-component sigma
    let mut gen = NoiseGenerator::new(NoiseColor::White, seed);
    gen.set_amplitude(sigma);
    gen.generate_complex(num_samples)
}

/// Add AWGN to an existing signal at a specified SNR.
pub fn add_awgn(signal: &[Complex64], snr_db: f64, seed: u64) -> Vec<Complex64> {
    if signal.is_empty() {
        return Vec::new();
    }

    // Measure signal power
    let signal_power = signal.iter().map(|s| s.norm_sqr()).sum::<f64>() / signal.len() as f64;

    // Calculate noise power from SNR
    let noise_power = signal_power / 10.0f64.powf(snr_db / 10.0);
    let sigma = (noise_power / 2.0).sqrt();

    let mut gen = NoiseGenerator::new(NoiseColor::White, seed);
    gen.set_amplitude(sigma);

    signal
        .iter()
        .map(|&s| {
            let noise = Complex64::new(gen.sample(), gen.sample());
            s + noise
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_white_noise_zero_mean() {
        let mut gen = NoiseGenerator::new(NoiseColor::White, 42);
        let samples = gen.generate(10000);
        let mean = samples.iter().sum::<f64>() / samples.len() as f64;
        assert!(
            mean.abs() < 0.1,
            "White noise should have ~zero mean: got {mean:.4}"
        );
    }

    #[test]
    fn test_white_noise_unit_variance() {
        let mut gen = NoiseGenerator::new(NoiseColor::White, 42);
        let samples = gen.generate(10000);
        let mean = samples.iter().sum::<f64>() / samples.len() as f64;
        let variance =
            samples.iter().map(|&s| (s - mean).powi(2)).sum::<f64>() / samples.len() as f64;
        assert!(
            (variance - 1.0).abs() < 0.2,
            "White noise should have ~unit variance: got {variance:.3}"
        );
    }

    #[test]
    fn test_pink_noise_generation() {
        let mut gen = NoiseGenerator::new(NoiseColor::Pink, 42);
        let samples = gen.generate(10000);

        // Pink noise should have finite variance
        let variance =
            samples.iter().map(|s| s.powi(2)).sum::<f64>() / samples.len() as f64;
        assert!(
            variance > 0.001 && variance < 100.0,
            "Pink noise variance should be finite: got {variance:.3}"
        );
    }

    #[test]
    fn test_brown_noise_bounded() {
        let mut gen = NoiseGenerator::new(NoiseColor::Brown, 42);
        let samples = gen.generate(10000);

        // Brown noise should be bounded by soft clipping
        let max_abs = samples.iter().map(|s| s.abs()).fold(0.0f64, f64::max);
        assert!(
            max_abs <= 10.1,
            "Brown noise should be bounded: max={max_abs:.3}"
        );
    }

    #[test]
    fn test_blue_noise_generation() {
        let mut gen = NoiseGenerator::new(NoiseColor::Blue, 42);
        let samples = gen.generate(10000);

        let variance = samples.iter().map(|s| s.powi(2)).sum::<f64>() / samples.len() as f64;
        assert!(
            variance > 0.001,
            "Blue noise should have non-zero variance: got {variance:.3}"
        );
    }

    #[test]
    fn test_violet_noise_generation() {
        let mut gen = NoiseGenerator::new(NoiseColor::Violet, 42);
        let samples = gen.generate(10000);

        let variance = samples.iter().map(|s| s.powi(2)).sum::<f64>() / samples.len() as f64;
        assert!(
            variance > 0.001,
            "Violet noise should have non-zero variance: got {variance:.3}"
        );
    }

    #[test]
    fn test_complex_noise() {
        let mut gen = NoiseGenerator::new(NoiseColor::White, 42);
        let samples = gen.generate_complex(1000);
        assert_eq!(samples.len(), 1000);

        // I and Q should be independent (check variance of both)
        let re_var = samples.iter().map(|s| s.re.powi(2)).sum::<f64>() / samples.len() as f64;
        let im_var = samples.iter().map(|s| s.im.powi(2)).sum::<f64>() / samples.len() as f64;
        assert!(re_var > 0.1 && im_var > 0.1, "Both I and Q should have variance");
    }

    #[test]
    fn test_awgn_function() {
        let noise = awgn_complex(1000, -10.0, 42);
        assert_eq!(noise.len(), 1000);

        let power = noise.iter().map(|s| s.norm_sqr()).sum::<f64>() / noise.len() as f64;
        let power_db = 10.0 * power.log10();
        assert!(
            (power_db - (-10.0)).abs() < 3.0,
            "AWGN power should be ~-10 dB: got {power_db:.1}"
        );
    }

    #[test]
    fn test_add_awgn_snr() {
        // Create a DC signal
        let signal: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 10000];
        let noisy = add_awgn(&signal, 10.0, 42);

        assert_eq!(noisy.len(), signal.len());

        // Measure actual SNR
        let noise: Vec<Complex64> = noisy
            .iter()
            .zip(signal.iter())
            .map(|(&n, &s)| n - s)
            .collect();
        let signal_power = signal.iter().map(|s| s.norm_sqr()).sum::<f64>() / signal.len() as f64;
        let noise_power = noise.iter().map(|s| s.norm_sqr()).sum::<f64>() / noise.len() as f64;
        let actual_snr = 10.0 * (signal_power / noise_power).log10();

        assert!(
            (actual_snr - 10.0).abs() < 2.0,
            "Actual SNR should be ~10 dB: got {actual_snr:.1}"
        );
    }

    #[test]
    fn test_noise_amplitude() {
        let mut gen = NoiseGenerator::new(NoiseColor::White, 42);
        gen.set_amplitude(0.5);
        let samples = gen.generate(10000);
        let variance = samples.iter().map(|s| s.powi(2)).sum::<f64>() / samples.len() as f64;
        assert!(
            (variance - 0.25).abs() < 0.1,
            "Amplitude 0.5 should give variance ~0.25: got {variance:.3}"
        );
    }

    #[test]
    fn test_noise_reset() {
        let mut gen = NoiseGenerator::new(NoiseColor::Brown, 42);
        let _ = gen.generate(1000);
        assert!(gen.brown_state.abs() > 0.0);
        gen.reset();
        assert!((gen.brown_state).abs() < 1e-10);
    }

    #[test]
    fn test_noise_color_slope() {
        assert!((NoiseColor::White.slope_db_per_octave()).abs() < 0.1);
        assert!((NoiseColor::Pink.slope_db_per_octave() - (-3.0)).abs() < 0.1);
        assert!((NoiseColor::Brown.slope_db_per_octave() - (-6.0)).abs() < 0.1);
        assert!((NoiseColor::Blue.slope_db_per_octave() - 3.0).abs() < 0.1);
        assert!((NoiseColor::Violet.slope_db_per_octave() - 6.0).abs() < 0.1);
    }
}
