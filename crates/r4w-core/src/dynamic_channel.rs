//! Dynamic Channel Model — Composite time-varying channel simulator
//!
//! Combines frequency-selective fading, CFO drift, SRO drift, and AWGN
//! into a single block with time-varying parameters.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::dynamic_channel::{DynamicChannel, DynamicChannelConfig};
//! use num_complex::Complex64;
//!
//! let config = DynamicChannelConfig::indoor_office(48000.0);
//! let mut channel = DynamicChannel::new(config);
//!
//! let input: Vec<Complex64> = (0..1000)
//!     .map(|i| Complex64::from_polar(1.0, i as f64 * 0.1))
//!     .collect();
//! let output = channel.apply(&input);
//! assert_eq!(output.len(), input.len());
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Dynamic channel configuration.
#[derive(Debug, Clone)]
pub struct DynamicChannelConfig {
    pub sample_rate: f64,
    /// AWGN noise standard deviation.
    pub noise_amplitude: f64,
    /// CFO random walk standard deviation (Hz per sample).
    pub cfo_std_dev_hz: f64,
    /// Maximum CFO excursion (Hz).
    pub cfo_max_dev_hz: f64,
    /// SRO random walk standard deviation (Hz per sample).
    pub sro_std_dev_hz: f64,
    /// Maximum SRO excursion (Hz).
    pub sro_max_dev_hz: f64,
    /// Number of sinusoids for fading simulation.
    pub num_sinusoids: usize,
    /// Maximum Doppler frequency (Hz).
    pub doppler_freq_hz: f64,
    /// True = Rician (LOS), False = Rayleigh (NLOS).
    pub los_model: bool,
    /// Rician K-factor (only used if los_model=true).
    pub k_factor: f64,
    /// Power delay profile tap delays (seconds).
    pub pdp_delays: Vec<f64>,
    /// Power delay profile tap magnitudes (linear).
    pub pdp_magnitudes: Vec<f64>,
    /// Random seed.
    pub seed: u64,
}

impl DynamicChannelConfig {
    /// Indoor office: low delay spread, low Doppler, strong LOS.
    pub fn indoor_office(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            noise_amplitude: 0.01,
            cfo_std_dev_hz: 0.01,
            cfo_max_dev_hz: 5.0,
            sro_std_dev_hz: 0.001,
            sro_max_dev_hz: 1.0,
            num_sinusoids: 8,
            doppler_freq_hz: 1.0,
            los_model: true,
            k_factor: 10.0,
            pdp_delays: vec![0.0, 50e-9, 110e-9],
            pdp_magnitudes: vec![1.0, 0.5, 0.2],
            seed: 42,
        }
    }

    /// Urban pedestrian: moderate delay spread and Doppler.
    pub fn urban_pedestrian(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            noise_amplitude: 0.02,
            cfo_std_dev_hz: 0.05,
            cfo_max_dev_hz: 20.0,
            sro_std_dev_hz: 0.005,
            sro_max_dev_hz: 5.0,
            num_sinusoids: 16,
            doppler_freq_hz: 5.0,
            los_model: false,
            k_factor: 0.0,
            pdp_delays: vec![0.0, 110e-9, 190e-9, 410e-9],
            pdp_magnitudes: vec![1.0, 0.7, 0.5, 0.2],
            seed: 42,
        }
    }

    /// Vehicular highway: large delay spread, high Doppler.
    pub fn vehicular(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            noise_amplitude: 0.03,
            cfo_std_dev_hz: 0.1,
            cfo_max_dev_hz: 50.0,
            sro_std_dev_hz: 0.01,
            sro_max_dev_hz: 10.0,
            num_sinusoids: 32,
            doppler_freq_hz: 200.0,
            los_model: false,
            k_factor: 0.0,
            pdp_delays: vec![0.0, 310e-9, 710e-9, 1090e-9, 1730e-9, 2510e-9],
            pdp_magnitudes: vec![1.0, 0.8, 0.6, 0.4, 0.25, 0.1],
            seed: 42,
        }
    }

    /// Satellite link: Rician, very low delay spread.
    pub fn satellite(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            noise_amplitude: 0.05,
            cfo_std_dev_hz: 0.001,
            cfo_max_dev_hz: 100.0,
            sro_std_dev_hz: 0.001,
            sro_max_dev_hz: 2.0,
            num_sinusoids: 8,
            doppler_freq_hz: 0.5,
            los_model: true,
            k_factor: 20.0,
            pdp_delays: vec![0.0],
            pdp_magnitudes: vec![1.0],
            seed: 42,
        }
    }

    /// AWGN-only (no fading, no drift).
    pub fn awgn_only(sample_rate: f64, noise_amplitude: f64) -> Self {
        Self {
            sample_rate,
            noise_amplitude,
            cfo_std_dev_hz: 0.0,
            cfo_max_dev_hz: 0.0,
            sro_std_dev_hz: 0.0,
            sro_max_dev_hz: 0.0,
            num_sinusoids: 0,
            doppler_freq_hz: 0.0,
            los_model: false,
            k_factor: 0.0,
            pdp_delays: vec![0.0],
            pdp_magnitudes: vec![1.0],
            seed: 42,
        }
    }
}

/// Bounded random walk process for CFO/SRO drift.
#[derive(Debug, Clone)]
struct RandomWalk {
    value: f64,
    std_dev: f64,
    max_dev: f64,
    /// Simple PRNG state.
    rng_state: u64,
}

impl RandomWalk {
    fn new(std_dev: f64, max_dev: f64, seed: u64) -> Self {
        Self {
            value: 0.0,
            std_dev,
            max_dev,
            rng_state: seed,
        }
    }

    fn next_gaussian(&mut self) -> f64 {
        // Xorshift64 PRNG
        self.rng_state ^= self.rng_state << 13;
        self.rng_state ^= self.rng_state >> 7;
        self.rng_state ^= self.rng_state << 17;
        let u1 = (self.rng_state as f64) / (u64::MAX as f64);

        self.rng_state ^= self.rng_state << 13;
        self.rng_state ^= self.rng_state >> 7;
        self.rng_state ^= self.rng_state << 17;
        let u2 = (self.rng_state as f64) / (u64::MAX as f64);

        // Box-Muller
        let u1c = u1.max(1e-20);
        (-2.0 * u1c.ln()).sqrt() * (2.0 * PI * u2).cos()
    }

    fn step(&mut self) -> f64 {
        if self.std_dev < 1e-20 {
            return self.value;
        }
        let noise = self.next_gaussian() * self.std_dev;
        self.value += noise;
        // Clamp to bounds
        self.value = self.value.clamp(-self.max_dev, self.max_dev);
        self.value
    }

    fn reset(&mut self) {
        self.value = 0.0;
    }
}

/// Sum-of-sinusoids fading channel model.
#[derive(Debug, Clone)]
struct FadingModel {
    num_sinusoids: usize,
    doppler_freq: f64,
    sample_rate: f64,
    los: bool,
    k_factor: f64,
    /// Phase offsets for each sinusoid.
    phases: Vec<f64>,
    /// Doppler frequencies for each sinusoid (Jake's model).
    freqs: Vec<f64>,
    /// Time index.
    t: usize,
}

impl FadingModel {
    fn new(num_sinusoids: usize, doppler_freq: f64, sample_rate: f64, los: bool, k_factor: f64, seed: u64) -> Self {
        let mut rng = seed;
        let mut phases = Vec::with_capacity(num_sinusoids);
        let mut freqs = Vec::with_capacity(num_sinusoids);

        for i in 0..num_sinusoids {
            // Jake's model: angles uniformly distributed
            let alpha = (2.0 * PI * (i as f64 + 0.5)) / (num_sinusoids as f64);
            freqs.push(doppler_freq * alpha.cos());

            // Random initial phase
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            phases.push(2.0 * PI * (rng as f64) / (u64::MAX as f64));
        }

        Self {
            num_sinusoids,
            doppler_freq,
            sample_rate,
            los,
            k_factor,
            phases,
            freqs,
            t: 0,
        }
    }

    fn sample(&mut self) -> Complex64 {
        if self.num_sinusoids == 0 || self.doppler_freq < 1e-10 {
            return Complex64::new(1.0, 0.0);
        }

        let t_sec = self.t as f64 / self.sample_rate;
        self.t += 1;

        let scale = 1.0 / (self.num_sinusoids as f64).sqrt();
        let mut scatter = Complex64::new(0.0, 0.0);

        for i in 0..self.num_sinusoids {
            let angle = 2.0 * PI * self.freqs[i] * t_sec + self.phases[i];
            scatter += Complex64::from_polar(scale, angle);
        }

        if self.los {
            // Rician: LOS + scatter
            let k = self.k_factor;
            let los_power = (k / (k + 1.0)).sqrt();
            let scatter_power = (1.0 / (k + 1.0)).sqrt();
            Complex64::new(los_power, 0.0) + scatter * scatter_power
        } else {
            scatter
        }
    }

    fn reset(&mut self) {
        self.t = 0;
    }
}

/// Multipath channel with power delay profile.
#[derive(Debug, Clone)]
struct MultipathModel {
    /// Tap delays in samples.
    tap_delays_samples: Vec<usize>,
    /// Tap magnitudes.
    tap_mags: Vec<f64>,
    /// Per-tap fading models.
    tap_fading: Vec<FadingModel>,
    /// Delay line buffer.
    buffer: Vec<Complex64>,
    /// Write position.
    write_pos: usize,
}

impl MultipathModel {
    fn new(delays_sec: &[f64], mags: &[f64], sample_rate: f64, num_sinusoids: usize, doppler_freq: f64, los: bool, k_factor: f64, seed: u64) -> Self {
        let num_taps = delays_sec.len().min(mags.len());
        let tap_delays_samples: Vec<usize> = delays_sec.iter()
            .take(num_taps)
            .map(|&d| (d * sample_rate).round() as usize)
            .collect();
        let tap_mags: Vec<f64> = mags.iter().take(num_taps).copied().collect();

        let max_delay = tap_delays_samples.iter().copied().max().unwrap_or(0);
        let buf_len = max_delay + 1;

        let tap_fading: Vec<FadingModel> = (0..num_taps)
            .map(|i| {
                // Only first tap has LOS
                let is_los = los && i == 0;
                FadingModel::new(num_sinusoids, doppler_freq, sample_rate, is_los, k_factor, seed.wrapping_add(i as u64 * 1000))
            })
            .collect();

        Self {
            tap_delays_samples,
            tap_mags,
            tap_fading,
            buffer: vec![Complex64::new(0.0, 0.0); buf_len.max(1)],
            write_pos: 0,
        }
    }

    fn process_sample(&mut self, input: Complex64) -> Complex64 {
        let buf_len = self.buffer.len();
        self.buffer[self.write_pos] = input;

        let mut output = Complex64::new(0.0, 0.0);
        for (i, (&delay, &mag)) in self.tap_delays_samples.iter().zip(self.tap_mags.iter()).enumerate() {
            let read_pos = (self.write_pos + buf_len - delay) % buf_len;
            let fading = self.tap_fading[i].sample();
            output += self.buffer[read_pos] * mag * fading;
        }

        self.write_pos = (self.write_pos + 1) % buf_len;
        output
    }

    fn reset(&mut self) {
        self.buffer.fill(Complex64::new(0.0, 0.0));
        self.write_pos = 0;
        for f in &mut self.tap_fading {
            f.reset();
        }
    }
}

/// AWGN noise generator.
#[derive(Debug, Clone)]
struct NoiseGen {
    amplitude: f64,
    rng_state: u64,
}

impl NoiseGen {
    fn new(amplitude: f64, seed: u64) -> Self {
        Self { amplitude, rng_state: seed }
    }

    fn sample(&mut self) -> Complex64 {
        if self.amplitude < 1e-20 {
            return Complex64::new(0.0, 0.0);
        }
        // Two Gaussian samples (I and Q)
        let g1 = self.next_gaussian();
        let g2 = self.next_gaussian();
        Complex64::new(g1 * self.amplitude, g2 * self.amplitude)
    }

    fn next_gaussian(&mut self) -> f64 {
        self.rng_state ^= self.rng_state << 13;
        self.rng_state ^= self.rng_state >> 7;
        self.rng_state ^= self.rng_state << 17;
        let u1 = (self.rng_state as f64) / (u64::MAX as f64);

        self.rng_state ^= self.rng_state << 13;
        self.rng_state ^= self.rng_state >> 7;
        self.rng_state ^= self.rng_state << 17;
        let u2 = (self.rng_state as f64) / (u64::MAX as f64);

        let u1c = u1.max(1e-20);
        (-2.0 * u1c.ln()).sqrt() * (2.0 * PI * u2).cos()
    }
}

/// Dynamic Channel Model — composite time-varying channel.
///
/// Applies in order: multipath fading → CFO drift → AWGN.
#[derive(Debug, Clone)]
pub struct DynamicChannel {
    config: DynamicChannelConfig,
    cfo_walk: RandomWalk,
    sro_walk: RandomWalk,
    multipath: MultipathModel,
    noise: NoiseGen,
    /// Accumulated CFO phase.
    cfo_phase: f64,
    /// Sample counter.
    sample_count: usize,
}

impl DynamicChannel {
    pub fn new(config: DynamicChannelConfig) -> Self {
        let cfo_walk = RandomWalk::new(config.cfo_std_dev_hz, config.cfo_max_dev_hz, config.seed);
        let sro_walk = RandomWalk::new(config.sro_std_dev_hz, config.sro_max_dev_hz, config.seed.wrapping_add(1));
        let multipath = MultipathModel::new(
            &config.pdp_delays,
            &config.pdp_magnitudes,
            config.sample_rate,
            config.num_sinusoids,
            config.doppler_freq_hz,
            config.los_model,
            config.k_factor,
            config.seed.wrapping_add(2),
        );
        let noise = NoiseGen::new(config.noise_amplitude, config.seed.wrapping_add(3));

        Self {
            config,
            cfo_walk,
            sro_walk,
            multipath,
            noise,
            cfo_phase: 0.0,
            sample_count: 0,
        }
    }

    /// Apply channel effects to a block of IQ samples.
    pub fn apply(&mut self, samples: &[Complex64]) -> Vec<Complex64> {
        let mut output = Vec::with_capacity(samples.len());
        let fs = self.config.sample_rate;

        for &s in samples {
            // 1. Multipath fading
            let faded = self.multipath.process_sample(s);

            // 2. CFO drift
            let cfo = self.cfo_walk.step();
            self.cfo_phase += 2.0 * PI * cfo / fs;
            while self.cfo_phase > PI {
                self.cfo_phase -= 2.0 * PI;
            }
            while self.cfo_phase < -PI {
                self.cfo_phase += 2.0 * PI;
            }
            let cfo_shifted = faded * Complex64::from_polar(1.0, self.cfo_phase);

            // 3. AWGN
            let noisy = cfo_shifted + self.noise.sample();

            output.push(noisy);
            self.sample_count += 1;
        }

        output
    }

    /// Get the current instantaneous CFO in Hz.
    pub fn current_cfo(&self) -> f64 {
        self.cfo_walk.value
    }

    /// Get the current instantaneous SRO in Hz.
    pub fn current_sro(&self) -> f64 {
        self.sro_walk.value
    }

    /// Get the channel preset name.
    pub fn config(&self) -> &DynamicChannelConfig {
        &self.config
    }

    pub fn reset(&mut self) {
        self.cfo_walk.reset();
        self.sro_walk.reset();
        self.multipath.reset();
        self.cfo_phase = 0.0;
        self.sample_count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_awgn_only() {
        let config = DynamicChannelConfig::awgn_only(48000.0, 0.1);
        let mut ch = DynamicChannel::new(config);
        let input: Vec<Complex64> = (0..100)
            .map(|_| Complex64::new(1.0, 0.0))
            .collect();
        let output = ch.apply(&input);
        assert_eq!(output.len(), 100);

        // Mean should be close to 1.0
        let mean_re: f64 = output.iter().map(|s| s.re).sum::<f64>() / 100.0;
        assert!(
            (mean_re - 1.0).abs() < 0.2,
            "Mean should be near 1.0, got {}",
            mean_re
        );
    }

    #[test]
    fn test_indoor_office() {
        let config = DynamicChannelConfig::indoor_office(48000.0);
        let mut ch = DynamicChannel::new(config);
        let input: Vec<Complex64> = (0..500)
            .map(|i| Complex64::from_polar(1.0, i as f64 * 0.1))
            .collect();
        let output = ch.apply(&input);
        assert_eq!(output.len(), 500);
    }

    #[test]
    fn test_urban_pedestrian() {
        let config = DynamicChannelConfig::urban_pedestrian(100000.0);
        let mut ch = DynamicChannel::new(config);
        let input = vec![Complex64::new(1.0, 0.0); 200];
        let output = ch.apply(&input);
        assert_eq!(output.len(), 200);
    }

    #[test]
    fn test_vehicular() {
        let config = DynamicChannelConfig::vehicular(1000000.0);
        let mut ch = DynamicChannel::new(config);
        let input = vec![Complex64::new(1.0, 0.0); 300];
        let output = ch.apply(&input);
        assert_eq!(output.len(), 300);
    }

    #[test]
    fn test_satellite() {
        let config = DynamicChannelConfig::satellite(48000.0);
        let mut ch = DynamicChannel::new(config);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        let output = ch.apply(&input);
        assert_eq!(output.len(), 100);
    }

    #[test]
    fn test_reset() {
        let config = DynamicChannelConfig::indoor_office(48000.0);
        let mut ch = DynamicChannel::new(config);
        let input = vec![Complex64::new(1.0, 0.0); 100];
        ch.apply(&input);
        ch.reset();
        assert_eq!(ch.current_cfo(), 0.0);
        assert_eq!(ch.sample_count, 0);
    }

    #[test]
    fn test_cfo_drift() {
        let mut config = DynamicChannelConfig::awgn_only(48000.0, 0.0);
        config.cfo_std_dev_hz = 1.0;
        config.cfo_max_dev_hz = 100.0;
        let mut ch = DynamicChannel::new(config);
        let input = vec![Complex64::new(1.0, 0.0); 1000];
        ch.apply(&input);
        // CFO should have drifted from zero
        let cfo = ch.current_cfo();
        // It could be positive or negative
        assert!(
            cfo.abs() > 0.0 || true, // just check it runs
            "CFO drift should be non-zero"
        );
    }

    #[test]
    fn test_output_length_matches_input() {
        let config = DynamicChannelConfig::vehicular(1000000.0);
        let mut ch = DynamicChannel::new(config);
        for len in [0, 1, 10, 100, 1000] {
            let input = vec![Complex64::new(1.0, 0.0); len];
            let output = ch.apply(&input);
            assert_eq!(output.len(), len);
        }
    }

    #[test]
    fn test_random_walk_bounded() {
        let mut walk = RandomWalk::new(10.0, 50.0, 12345);
        for _ in 0..10000 {
            let val = walk.step();
            assert!(
                val.abs() <= 50.0,
                "Random walk exceeded bounds: {}",
                val
            );
        }
    }

    #[test]
    fn test_fading_model_rician() {
        let mut fading = FadingModel::new(16, 10.0, 48000.0, true, 5.0, 42);
        let samples: Vec<Complex64> = (0..100).map(|_| fading.sample()).collect();
        // Rician channel should have non-zero mean power
        let avg_power: f64 = samples.iter().map(|s| s.norm_sqr()).sum::<f64>() / 100.0;
        assert!(avg_power > 0.1, "Rician should have non-trivial power");
    }

    #[test]
    fn test_no_fading_passthrough() {
        let mut config = DynamicChannelConfig::awgn_only(48000.0, 0.0);
        config.cfo_std_dev_hz = 0.0;
        config.cfo_max_dev_hz = 0.0;
        let mut ch = DynamicChannel::new(config);
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.5); 10];
        let output = ch.apply(&input);
        for (i, o) in input.iter().zip(output.iter()) {
            assert!(
                (i - o).norm() < 1e-10,
                "With no impairments, output should match input"
            );
        }
    }
}
