//! Channel Models for SDR Simulation
//!
//! This module provides various channel models to simulate realistic
//! wireless propagation effects in software.
//!
//! ## Channel Effects
//!
//! Real wireless channels introduce several impairments:
//!
//! 1. **AWGN (Additive White Gaussian Noise)**: Thermal noise
//! 2. **Path Loss**: Signal attenuation with distance
//! 3. **Fading**: Time-varying signal strength
//! 4. **Multipath**: Multiple signal copies arriving at different times
//! 5. **Frequency Offset**: Carrier frequency mismatch
//! 6. **Timing Drift**: Clock differences between TX and RX
//!
//! ## Usage
//!
//! ```rust
//! use r4w_sim::channel::{Channel, ChannelConfig, ChannelModel};
//! use r4w_core::types::Complex;
//!
//! let config = ChannelConfig {
//!     model: ChannelModel::Awgn,
//!     snr_db: 10.0,
//!     ..Default::default()
//! };
//!
//! // Some example I/Q samples
//! let clean_samples: Vec<Complex> = vec![Complex::new(1.0, 0.0); 100];
//!
//! let mut channel = Channel::new(config);
//! let noisy = channel.apply(&clean_samples);
//! ```

use r4w_core::types::{Complex, IQSample};
use rand::rngs::StdRng;
use rand::SeedableRng;
use rand_distr::{Distribution, Normal};
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

/// Channel model type
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ChannelModel {
    /// Perfect channel (no impairments)
    Ideal,
    /// Additive White Gaussian Noise only
    Awgn,
    /// AWGN + frequency offset
    AwgnWithCfo,
    /// AWGN + multipath (2-ray model)
    Multipath,
    /// Rayleigh fading (for mobile scenarios)
    Rayleigh,
    /// Rician fading (line-of-sight + multipath)
    Rician,
}

impl Default for ChannelModel {
    fn default() -> Self {
        Self::Awgn
    }
}

/// Channel configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ChannelConfig {
    /// Channel model to use
    pub model: ChannelModel,
    /// Target SNR in dB
    pub snr_db: f64,
    /// Carrier frequency offset in Hz
    pub cfo_hz: f64,
    /// Clock drift in PPM (parts per million)
    pub clock_drift_ppm: f64,
    /// Multipath delay in samples
    pub multipath_delay: usize,
    /// Multipath relative amplitude (0-1)
    pub multipath_amplitude: f64,
    /// Rician K-factor (ratio of LOS to scattered power)
    pub rician_k: f64,
    /// Sample rate (needed for some calculations)
    pub sample_rate: f64,
    /// Path loss in dB
    pub path_loss_db: f64,
}

impl Default for ChannelConfig {
    fn default() -> Self {
        Self {
            model: ChannelModel::Awgn,
            snr_db: 20.0,
            cfo_hz: 0.0,
            clock_drift_ppm: 0.0,
            multipath_delay: 0,
            multipath_amplitude: 0.0,
            rician_k: 10.0,
            sample_rate: 125_000.0,
            path_loss_db: 0.0,
        }
    }
}

impl ChannelConfig {
    /// Create a clean channel with only specified SNR
    pub fn with_snr(snr_db: f64) -> Self {
        Self {
            snr_db,
            ..Default::default()
        }
    }

    /// Create a channel with CFO
    pub fn with_cfo(snr_db: f64, cfo_hz: f64) -> Self {
        Self {
            model: ChannelModel::AwgnWithCfo,
            snr_db,
            cfo_hz,
            ..Default::default()
        }
    }

    /// Create a multipath channel
    pub fn multipath(snr_db: f64, delay_samples: usize, amplitude: f64) -> Self {
        Self {
            model: ChannelModel::Multipath,
            snr_db,
            multipath_delay: delay_samples,
            multipath_amplitude: amplitude,
            ..Default::default()
        }
    }
}

/// Channel simulator
#[derive(Debug)]
pub struct Channel {
    config: ChannelConfig,
    rng: StdRng,
    /// Phase accumulator for CFO simulation
    cfo_phase: f64,
    /// Sample counter for timing drift
    sample_count: u64,
    /// Multipath history buffer
    multipath_buffer: Vec<IQSample>,
}

impl Channel {
    /// Create a new channel with the given configuration
    pub fn new(config: ChannelConfig) -> Self {
        let multipath_buffer = vec![Complex::new(0.0, 0.0); config.multipath_delay + 1];

        Self {
            config,
            rng: StdRng::from_entropy(),
            cfo_phase: 0.0,
            sample_count: 0,
            multipath_buffer,
        }
    }

    /// Reset channel state
    pub fn reset(&mut self) {
        self.cfo_phase = 0.0;
        self.sample_count = 0;
        self.multipath_buffer.fill(Complex::new(0.0, 0.0));
    }

    /// Get current configuration
    pub fn config(&self) -> &ChannelConfig {
        &self.config
    }

    /// Update configuration
    pub fn set_config(&mut self, config: ChannelConfig) {
        self.config = config;
        self.multipath_buffer
            .resize(self.config.multipath_delay + 1, Complex::new(0.0, 0.0));
    }

    /// Apply channel effects to samples
    pub fn apply(&mut self, samples: &[IQSample]) -> Vec<IQSample> {
        match self.config.model {
            ChannelModel::Ideal => samples.to_vec(),
            ChannelModel::Awgn => self.apply_awgn(samples),
            ChannelModel::AwgnWithCfo => {
                let with_cfo = self.apply_cfo(samples);
                self.apply_awgn(&with_cfo)
            }
            ChannelModel::Multipath => {
                let with_multipath = self.apply_multipath(samples);
                self.apply_awgn(&with_multipath)
            }
            ChannelModel::Rayleigh => self.apply_rayleigh(samples),
            ChannelModel::Rician => self.apply_rician(samples),
        }
    }

    /// Apply AWGN (Additive White Gaussian Noise)
    fn apply_awgn(&mut self, samples: &[IQSample]) -> Vec<IQSample> {
        // Calculate noise power from SNR
        // SNR = signal_power / noise_power
        // noise_power = signal_power / 10^(SNR_dB/10)

        // Estimate signal power (assume normalized input)
        let signal_power: f64 = samples.iter().map(|s| s.norm_sqr()).sum::<f64>() / samples.len() as f64;

        // Apply path loss
        let path_loss_linear = 10.0_f64.powf(-self.config.path_loss_db / 20.0);

        // Calculate noise standard deviation
        let snr_linear = 10.0_f64.powf(self.config.snr_db / 10.0);
        let noise_power = signal_power / snr_linear;
        let noise_std = (noise_power / 2.0).sqrt(); // Divide by 2 for I and Q

        // Generate noise distribution
        let noise_dist = Normal::new(0.0, noise_std).unwrap();

        samples
            .iter()
            .map(|&s| {
                // Apply path loss and add noise
                let attenuated = s * path_loss_linear;
                let noise = Complex::new(
                    noise_dist.sample(&mut self.rng),
                    noise_dist.sample(&mut self.rng),
                );
                attenuated + noise
            })
            .collect()
    }

    /// Apply carrier frequency offset
    fn apply_cfo(&mut self, samples: &[IQSample]) -> Vec<IQSample> {
        let cfo_rad_per_sample = 2.0 * PI * self.config.cfo_hz / self.config.sample_rate;

        samples
            .iter()
            .map(|&s| {
                let rotation = Complex::new(self.cfo_phase.cos(), self.cfo_phase.sin());
                self.cfo_phase += cfo_rad_per_sample;

                // Keep phase in reasonable range
                if self.cfo_phase > 2.0 * PI {
                    self.cfo_phase -= 2.0 * PI;
                }

                s * rotation
            })
            .collect()
    }

    /// Apply multipath (2-ray model)
    fn apply_multipath(&mut self, samples: &[IQSample]) -> Vec<IQSample> {
        let delay = self.config.multipath_delay;
        let amp = self.config.multipath_amplitude;

        if delay == 0 || amp == 0.0 {
            return samples.to_vec();
        }

        let mut output = Vec::with_capacity(samples.len());

        for &sample in samples {
            // Shift buffer
            self.multipath_buffer.remove(0);
            self.multipath_buffer.push(sample);

            // Sum direct path and delayed path
            let direct = sample;
            let delayed = self.multipath_buffer[0] * amp;
            output.push(direct + delayed);
        }

        output
    }

    /// Apply Rayleigh fading
    fn apply_rayleigh(&mut self, samples: &[IQSample]) -> Vec<IQSample> {
        let noise_dist = Normal::new(0.0, 1.0 / 2.0_f64.sqrt()).unwrap();

        samples
            .iter()
            .map(|&s| {
                // Rayleigh fading coefficient: complex Gaussian
                let h = Complex::new(
                    noise_dist.sample(&mut self.rng),
                    noise_dist.sample(&mut self.rng),
                );
                s * h
            })
            .collect()
    }

    /// Apply Rician fading
    fn apply_rician(&mut self, samples: &[IQSample]) -> Vec<IQSample> {
        let k = self.config.rician_k;

        // LOS component amplitude
        let los_amp = (k / (k + 1.0)).sqrt();
        // Scattered component amplitude
        let scatter_amp = (1.0 / (k + 1.0)).sqrt();

        let noise_dist = Normal::new(0.0, scatter_amp / 2.0_f64.sqrt()).unwrap();

        samples
            .iter()
            .map(|&s| {
                // Rician = LOS + scattered (Rayleigh)
                let los = Complex::new(los_amp, 0.0);
                let scattered = Complex::new(
                    noise_dist.sample(&mut self.rng),
                    noise_dist.sample(&mut self.rng),
                );
                s * (los + scattered)
            })
            .collect()
    }

    /// Calculate theoretical BER for AWGN channel
    pub fn theoretical_ber_awgn(snr_db: f64, spreading_factor: u8) -> f64 {
        // Simplified approximation for LoRa
        let snr_linear = 10.0_f64.powf(snr_db / 10.0);
        let m = 2.0_f64.powi(spreading_factor as i32);

        // Q-function approximation
        let arg = (2.0 * snr_linear * spreading_factor as f64 / m).sqrt();
        0.5 * (-arg * arg / 2.0).exp()
    }
}

/// Generate channel statistics for visualization
#[derive(Debug, Clone)]
pub struct ChannelStats {
    /// Mean signal power
    pub signal_power: f64,
    /// Mean noise power
    pub noise_power: f64,
    /// Measured SNR in dB
    pub measured_snr_db: f64,
    /// Peak to average power ratio
    pub papr_db: f64,
}

impl ChannelStats {
    /// Compute statistics from samples
    pub fn compute(clean: &[IQSample], noisy: &[IQSample]) -> Self {
        let signal_power: f64 = clean.iter().map(|s| s.norm_sqr()).sum::<f64>() / clean.len() as f64;

        // Estimate noise by differencing
        let noise_power: f64 = clean
            .iter()
            .zip(noisy.iter())
            .map(|(c, n)| (n - c).norm_sqr())
            .sum::<f64>()
            / clean.len() as f64;

        let measured_snr_db = 10.0 * (signal_power / noise_power).log10();

        let peak_power = noisy.iter().map(|s| s.norm_sqr()).fold(0.0_f64, f64::max);
        let avg_power: f64 = noisy.iter().map(|s| s.norm_sqr()).sum::<f64>() / noisy.len() as f64;
        let papr_db = 10.0 * (peak_power / avg_power).log10();

        Self {
            signal_power,
            noise_power,
            measured_snr_db,
            papr_db,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_awgn_channel() {
        let config = ChannelConfig::with_snr(20.0);
        let mut channel = Channel::new(config);

        // Create test signal (constant amplitude)
        let samples: Vec<IQSample> = (0..1000)
            .map(|_| Complex::new(1.0, 0.0))
            .collect();

        let noisy = channel.apply(&samples);

        // Output should be same length
        assert_eq!(noisy.len(), samples.len());

        // Check that noise was added (samples shouldn't be identical)
        let diff: f64 = samples
            .iter()
            .zip(noisy.iter())
            .map(|(a, b)| (a - b).norm())
            .sum();
        assert!(diff > 0.0);
    }

    #[test]
    fn test_cfo_channel() {
        let config = ChannelConfig::with_cfo(100.0, 1000.0); // 1 kHz offset
        let mut channel = Channel::new(config);

        let samples: Vec<IQSample> = (0..1000)
            .map(|_| Complex::new(1.0, 0.0))
            .collect();

        let output = channel.apply(&samples);

        // Phase should rotate between consecutive samples
        // With 1kHz CFO and 125kHz sample rate: 2π * 1000 / 125000 = 0.05 rad/sample
        let phase_diff = (output[1].arg() - output[0].arg()).abs();
        assert!(phase_diff > 0.01, "Phase should rotate between samples, got diff = {}", phase_diff);

        // Verify all samples have same magnitude (CFO only rotates phase)
        for sample in &output {
            assert!((sample.norm() - 1.0).abs() < 0.01, "Magnitude should be preserved");
        }
    }

    #[test]
    fn test_ideal_channel() {
        let config = ChannelConfig {
            model: ChannelModel::Ideal,
            ..Default::default()
        };
        let mut channel = Channel::new(config);

        let samples: Vec<IQSample> = (0..100)
            .map(|i| Complex::new(i as f64, 0.0))
            .collect();

        let output = channel.apply(&samples);

        // Ideal channel should pass through unchanged
        assert_eq!(samples, output);
    }
}
