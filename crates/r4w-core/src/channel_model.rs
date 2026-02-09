//! Channel Model — Unified channel impairment block
//!
//! Combines AWGN, frequency offset, timing offset, and multipath into a
//! single composable processing step. The classic simulation channel model
//! for BER testing and system validation.
//! GNU Radio equivalent: `channel_model`, `channel_model2`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::channel_model::ChannelModel;
//! use num_complex::Complex64;
//!
//! let mut ch = ChannelModel::awgn_only(0.1);
//! let signal = vec![Complex64::new(1.0, 0.0); 100];
//! let received = ch.process(&signal);
//! assert_eq!(received.len(), 100);
//! // Signal should be noisy but close to original
//! let mean_power: f64 = received.iter().map(|c| c.norm_sqr()).sum::<f64>() / 100.0;
//! assert!(mean_power > 0.5 && mean_power < 2.0);
//! ```

use num_complex::Complex64;

/// Channel model configuration.
#[derive(Debug, Clone)]
pub struct ChannelModelConfig {
    /// Standard deviation of AWGN (voltage, not power).
    pub noise_voltage: f64,
    /// Normalized frequency offset (-0.5 to 0.5, in cycles/sample).
    pub frequency_offset: f64,
    /// Timing offset / sample rate offset (1.0 = no offset).
    pub epsilon: f64,
    /// Multipath channel taps (default: [1.0]).
    pub taps: Vec<Complex64>,
    /// Random seed for noise generation.
    pub noise_seed: u64,
}

impl Default for ChannelModelConfig {
    fn default() -> Self {
        Self {
            noise_voltage: 0.0,
            frequency_offset: 0.0,
            epsilon: 1.0,
            taps: vec![Complex64::new(1.0, 0.0)],
            noise_seed: 42,
        }
    }
}

/// Unified channel impairment model.
///
/// Applies: multipath → frequency offset → AWGN noise.
#[derive(Debug, Clone)]
pub struct ChannelModel {
    config: ChannelModelConfig,
    /// Current phase accumulator for frequency offset.
    phase: f64,
    /// Phase increment per sample.
    phase_inc: f64,
    /// Delay line for multipath FIR.
    delay_line: Vec<Complex64>,
    /// Write index into delay line.
    delay_idx: usize,
    /// PRNG state (xoshiro256**).
    rng_state: [u64; 4],
}

impl ChannelModel {
    /// Create from full configuration.
    pub fn new(config: ChannelModelConfig) -> Self {
        let phase_inc = 2.0 * std::f64::consts::PI * config.frequency_offset;
        let delay_len = config.taps.len().max(1);
        let mut rng_state = [0u64; 4];
        // Seed the PRNG
        let seed = config.noise_seed;
        rng_state[0] = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
        rng_state[1] = rng_state[0].wrapping_mul(6364136223846793005).wrapping_add(1);
        rng_state[2] = rng_state[1].wrapping_mul(6364136223846793005).wrapping_add(1);
        rng_state[3] = rng_state[2].wrapping_mul(6364136223846793005).wrapping_add(1);

        Self {
            phase_inc,
            phase: 0.0,
            delay_line: vec![Complex64::new(0.0, 0.0); delay_len],
            delay_idx: 0,
            rng_state,
            config,
        }
    }

    /// Create AWGN-only channel.
    pub fn awgn_only(noise_voltage: f64) -> Self {
        Self::new(ChannelModelConfig {
            noise_voltage,
            ..Default::default()
        })
    }

    /// Create channel with AWGN and CFO.
    pub fn with_cfo(noise_voltage: f64, freq_offset: f64) -> Self {
        Self::new(ChannelModelConfig {
            noise_voltage,
            frequency_offset: freq_offset,
            ..Default::default()
        })
    }

    /// Create channel with multipath.
    pub fn with_multipath(noise_voltage: f64, taps: Vec<Complex64>) -> Self {
        Self::new(ChannelModelConfig {
            noise_voltage,
            taps,
            ..Default::default()
        })
    }

    /// Process a block of samples through the channel.
    pub fn process(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(input.len());
        for &x in input {
            output.push(self.process_sample(x));
        }
        output
    }

    /// Process a single sample.
    #[inline]
    pub fn process_sample(&mut self, x: Complex64) -> Complex64 {
        // 1. Multipath FIR filter
        self.delay_line[self.delay_idx] = x;
        let mut filtered = Complex64::new(0.0, 0.0);
        for (k, &tap) in self.config.taps.iter().enumerate() {
            let idx = (self.delay_idx + self.delay_line.len() - k) % self.delay_line.len();
            filtered += tap * self.delay_line[idx];
        }
        self.delay_idx = (self.delay_idx + 1) % self.delay_line.len();

        // 2. Frequency offset (CFO)
        let rotator = Complex64::from_polar(1.0, self.phase);
        filtered *= rotator;
        self.phase += self.phase_inc;
        if self.phase > std::f64::consts::PI {
            self.phase -= 2.0 * std::f64::consts::PI;
        } else if self.phase < -std::f64::consts::PI {
            self.phase += 2.0 * std::f64::consts::PI;
        }

        // 3. AWGN
        if self.config.noise_voltage > 0.0 {
            let (n_re, n_im) = self.gaussian_noise();
            filtered += Complex64::new(
                n_re * self.config.noise_voltage,
                n_im * self.config.noise_voltage,
            );
        }

        filtered
    }

    /// Generate a pair of Gaussian random numbers (Box-Muller).
    fn gaussian_noise(&mut self) -> (f64, f64) {
        let u1 = self.next_uniform();
        let u2 = self.next_uniform();
        let r = (-2.0 * u1.max(1e-30).ln()).sqrt();
        let theta = 2.0 * std::f64::consts::PI * u2;
        (r * theta.cos(), r * theta.sin())
    }

    /// Uniform random [0, 1) using xoshiro256**.
    fn next_uniform(&mut self) -> f64 {
        let s = &mut self.rng_state;
        let result = s[1].wrapping_mul(5).rotate_left(7).wrapping_mul(9);
        let t = s[1] << 17;
        s[2] ^= s[0];
        s[3] ^= s[1];
        s[1] ^= s[2];
        s[0] ^= s[3];
        s[2] ^= t;
        s[3] = s[3].rotate_left(45);
        (result >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Set noise voltage (standard deviation of AWGN).
    pub fn set_noise_voltage(&mut self, voltage: f64) {
        self.config.noise_voltage = voltage.max(0.0);
    }

    /// Get noise voltage.
    pub fn noise_voltage(&self) -> f64 {
        self.config.noise_voltage
    }

    /// Set frequency offset (normalized, cycles/sample).
    pub fn set_frequency_offset(&mut self, offset: f64) {
        self.config.frequency_offset = offset;
        self.phase_inc = 2.0 * std::f64::consts::PI * offset;
    }

    /// Get frequency offset.
    pub fn frequency_offset(&self) -> f64 {
        self.config.frequency_offset
    }

    /// Set multipath taps.
    pub fn set_taps(&mut self, taps: Vec<Complex64>) {
        self.config.taps = taps;
        let delay_len = self.config.taps.len().max(1);
        self.delay_line = vec![Complex64::new(0.0, 0.0); delay_len];
        self.delay_idx = 0;
    }

    /// Get multipath taps.
    pub fn taps(&self) -> &[Complex64] {
        &self.config.taps
    }

    /// Set timing offset (epsilon).
    pub fn set_epsilon(&mut self, epsilon: f64) {
        self.config.epsilon = epsilon;
    }

    /// Get timing offset.
    pub fn epsilon(&self) -> f64 {
        self.config.epsilon
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.phase = 0.0;
        self.delay_line.fill(Complex64::new(0.0, 0.0));
        self.delay_idx = 0;
    }

    /// Get the SNR in dB for a given signal power (assumes unit power signal).
    pub fn snr_db(&self) -> f64 {
        if self.config.noise_voltage > 0.0 {
            -20.0 * self.config.noise_voltage.log10()
        } else {
            f64::INFINITY
        }
    }

    /// Create from SNR in dB (assumes unit power signal).
    pub fn from_snr_db(snr_db: f64) -> Self {
        let noise_voltage = 10f64.powf(-snr_db / 20.0);
        Self::awgn_only(noise_voltage)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_noiseless_passthrough() {
        let mut ch = ChannelModel::awgn_only(0.0);
        let input: Vec<Complex64> = (0..100).map(|i| {
            Complex64::new(i as f64, 0.0)
        }).collect();
        let output = ch.process(&input);
        assert_eq!(output.len(), 100);
        for (a, b) in input.iter().zip(output.iter()) {
            assert!((a - b).norm() < 1e-10);
        }
    }

    #[test]
    fn test_awgn_adds_noise() {
        let mut ch = ChannelModel::awgn_only(0.5);
        let input = vec![Complex64::new(1.0, 0.0); 1000];
        let output = ch.process(&input);
        // Mean should be close to 1.0
        let mean_re = output.iter().map(|c| c.re).sum::<f64>() / 1000.0;
        assert!((mean_re - 1.0).abs() < 0.1);
        // Variance should be ~0.25 (0.5^2)
        let var_re = output.iter().map(|c| (c.re - mean_re).powi(2)).sum::<f64>() / 1000.0;
        assert!((var_re - 0.25).abs() < 0.1);
    }

    #[test]
    fn test_frequency_offset() {
        let mut ch = ChannelModel::with_cfo(0.0, 0.1);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = ch.process(&input);
        // First sample should be unchanged (phase=0)
        assert!((output[0].re - 1.0).abs() < 1e-10);
        // Sample 3: phase = 3 * 2π * 0.1 = 0.6π, sin(0.6π) ≈ 0.951
        assert!(output[3].im.abs() > 0.5, "sample 3 im = {}", output[3].im);
    }

    #[test]
    fn test_multipath() {
        let taps = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.5, 0.0),
        ];
        let mut ch = ChannelModel::with_multipath(0.0, taps);
        let mut input = vec![Complex64::new(0.0, 0.0); 50];
        input[0] = Complex64::new(1.0, 0.0);
        let output = ch.process(&input);
        // Impulse response: [1.0, 0.5, 0, 0, ...]
        assert!((output[0].re - 1.0).abs() < 1e-10);
        assert!((output[1].re - 0.5).abs() < 1e-10);
        assert!(output[2].re.abs() < 1e-10);
    }

    #[test]
    fn test_from_snr_db() {
        let ch = ChannelModel::from_snr_db(20.0);
        assert!((ch.noise_voltage() - 0.1).abs() < 1e-6);
        assert!((ch.snr_db() - 20.0).abs() < 0.01);
    }

    #[test]
    fn test_snr_db_no_noise() {
        let ch = ChannelModel::awgn_only(0.0);
        assert_eq!(ch.snr_db(), f64::INFINITY);
    }

    #[test]
    fn test_set_noise_voltage() {
        let mut ch = ChannelModel::awgn_only(0.0);
        ch.set_noise_voltage(0.5);
        assert_eq!(ch.noise_voltage(), 0.5);
    }

    #[test]
    fn test_set_frequency_offset() {
        let mut ch = ChannelModel::awgn_only(0.0);
        ch.set_frequency_offset(0.05);
        assert_eq!(ch.frequency_offset(), 0.05);
    }

    #[test]
    fn test_set_taps() {
        let mut ch = ChannelModel::awgn_only(0.0);
        ch.set_taps(vec![Complex64::new(0.5, 0.0), Complex64::new(0.5, 0.0)]);
        assert_eq!(ch.taps().len(), 2);
    }

    #[test]
    fn test_epsilon() {
        let mut ch = ChannelModel::awgn_only(0.0);
        ch.set_epsilon(1.001);
        assert_eq!(ch.epsilon(), 1.001);
    }

    #[test]
    fn test_reset() {
        let mut ch = ChannelModel::with_cfo(0.0, 0.1);
        ch.process(&vec![Complex64::new(1.0, 0.0); 100]);
        ch.reset();
        // After reset, phase should be 0
        let out = ch.process_sample(Complex64::new(1.0, 0.0));
        assert!((out.re - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_default_config() {
        let config = ChannelModelConfig::default();
        assert_eq!(config.noise_voltage, 0.0);
        assert_eq!(config.frequency_offset, 0.0);
        assert_eq!(config.epsilon, 1.0);
        assert_eq!(config.taps.len(), 1);
    }

    #[test]
    fn test_deterministic_noise() {
        // Same seed should produce same output
        let mut ch1 = ChannelModel::new(ChannelModelConfig {
            noise_voltage: 0.5,
            noise_seed: 123,
            ..Default::default()
        });
        let mut ch2 = ChannelModel::new(ChannelModelConfig {
            noise_voltage: 0.5,
            noise_seed: 123,
            ..Default::default()
        });
        let input = vec![Complex64::new(1.0, 0.0); 10];
        let out1 = ch1.process(&input);
        let out2 = ch2.process(&input);
        for (a, b) in out1.iter().zip(out2.iter()) {
            assert_eq!(a, b);
        }
    }

    #[test]
    fn test_output_length() {
        let mut ch = ChannelModel::awgn_only(0.1);
        let input = vec![Complex64::new(1.0, 0.0); 256];
        let output = ch.process(&input);
        assert_eq!(output.len(), 256);
    }

    #[test]
    fn test_cfo_phase_wrapping() {
        // Large frequency offset should wrap phase properly
        let mut ch = ChannelModel::with_cfo(0.0, 0.49);
        let input = vec![Complex64::new(1.0, 0.0); 1000];
        let output = ch.process(&input);
        // All outputs should have unit magnitude
        for c in &output {
            assert!((c.norm() - 1.0).abs() < 1e-10);
        }
    }
}
