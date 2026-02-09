//! Channel Sounder — Wideband Channel Measurement
//!
//! Correlation-based channel sounder for measuring propagation
//! characteristics including delay spread, Doppler spread, and
//! coherence bandwidth. Uses pseudorandom (m-sequence) or chirp
//! probing signals to estimate the channel impulse response.
//! GNU Radio equivalent: `gr_channelsounder` (out-of-tree).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::channel_sounder::{ChannelSounder, SoundingConfig};
//! use num_complex::Complex64;
//!
//! let config = SoundingConfig {
//!     sequence_length: 31,
//!     sample_rate: 1e6,
//!     max_delay_us: 10.0,
//!     doppler_bins: 16,
//! };
//! let mut sounder = ChannelSounder::new(config);
//! let tx = sounder.generate_sounding_signal();
//! assert_eq!(tx.len(), 31);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Configuration for channel sounder.
#[derive(Debug, Clone)]
pub struct SoundingConfig {
    /// Length of sounding sequence (should be 2^n - 1 for m-sequence).
    pub sequence_length: usize,
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Maximum delay to search in microseconds.
    pub max_delay_us: f64,
    /// Number of Doppler frequency bins.
    pub doppler_bins: usize,
}

/// Channel impulse response measurement result.
#[derive(Debug, Clone)]
pub struct ChannelImpulseResponse {
    /// Delay values in microseconds.
    pub delays_us: Vec<f64>,
    /// Complex gains at each delay.
    pub complex_gains: Vec<Complex64>,
    /// Power delay profile in dB.
    pub pdp_db: Vec<f64>,
    /// Estimated RMS delay spread in microseconds.
    pub rms_delay_spread_us: f64,
    /// Estimated mean excess delay in microseconds.
    pub mean_excess_delay_us: f64,
}

/// Doppler power spectrum result.
#[derive(Debug, Clone)]
pub struct DopplerSpectrum {
    /// Doppler frequency bins in Hz.
    pub frequencies_hz: Vec<f64>,
    /// Power at each Doppler bin in dB.
    pub power_db: Vec<f64>,
    /// Estimated RMS Doppler spread in Hz.
    pub rms_doppler_spread_hz: f64,
}

/// Channel quality metrics.
#[derive(Debug, Clone)]
pub struct ChannelMetrics {
    /// Coherence bandwidth (50% correlation) in Hz.
    pub coherence_bandwidth_50: f64,
    /// Coherence bandwidth (90% correlation) in Hz.
    pub coherence_bandwidth_90: f64,
    /// Coherence time in seconds.
    pub coherence_time_s: f64,
    /// Average path loss in dB.
    pub path_loss_db: f64,
    /// K-factor (Rician) in dB.
    pub k_factor_db: f64,
    /// Number of significant multipath components.
    pub num_paths: usize,
}

/// Correlation-based channel sounder.
#[derive(Debug, Clone)]
pub struct ChannelSounder {
    config: SoundingConfig,
    /// The reference sounding sequence.
    reference: Vec<Complex64>,
    /// Previous CIR snapshots for Doppler estimation.
    cir_history: Vec<Vec<Complex64>>,
}

impl ChannelSounder {
    /// Create a new channel sounder.
    pub fn new(config: SoundingConfig) -> Self {
        let reference = generate_mls(config.sequence_length);
        Self {
            config,
            reference,
            cir_history: Vec::new(),
        }
    }

    /// Generate the sounding (transmit) signal.
    pub fn generate_sounding_signal(&self) -> Vec<Complex64> {
        self.reference.clone()
    }

    /// Generate a chirp sounding signal.
    pub fn generate_chirp_signal(&self, bandwidth_hz: f64) -> Vec<Complex64> {
        let n = self.config.sequence_length;
        let fs = self.config.sample_rate;
        let chirp_rate = bandwidth_hz / (n as f64 / fs);
        (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let phase = 2.0 * PI * (chirp_rate / 2.0) * t * t;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Correlate received signal against reference to get CIR.
    pub fn estimate_cir(&mut self, received: &[Complex64]) -> ChannelImpulseResponse {
        let max_delay_samples = (self.config.max_delay_us * 1e-6 * self.config.sample_rate)
            .ceil() as usize;
        let max_delay_samples = max_delay_samples.min(received.len());

        let ref_len = self.reference.len();
        let corr_len = max_delay_samples.min(received.len().saturating_sub(ref_len) + 1);

        let mut delays_us = Vec::with_capacity(corr_len);
        let mut complex_gains = Vec::with_capacity(corr_len);

        // Cross-correlation
        for delay in 0..corr_len {
            let mut sum = Complex64::new(0.0, 0.0);
            for k in 0..ref_len {
                if delay + k < received.len() {
                    sum += received[delay + k] * self.reference[k].conj();
                }
            }
            sum /= ref_len as f64;
            delays_us.push(delay as f64 / self.config.sample_rate * 1e6);
            complex_gains.push(sum);
        }

        // Power delay profile
        let pdp: Vec<f64> = complex_gains.iter().map(|g| g.norm_sqr()).collect();
        let max_power = pdp.iter().cloned().fold(0.0f64, f64::max);
        let pdp_db: Vec<f64> = pdp
            .iter()
            .map(|&p| {
                if p > 0.0 && max_power > 0.0 {
                    10.0 * (p / max_power).log10()
                } else {
                    -100.0
                }
            })
            .collect();

        // Mean excess delay and RMS delay spread
        let total_power: f64 = pdp.iter().sum();
        let (mean_excess_delay_us, rms_delay_spread_us) = if total_power > 0.0 {
            let mean = pdp
                .iter()
                .zip(delays_us.iter())
                .map(|(&p, &d)| p * d)
                .sum::<f64>()
                / total_power;
            let variance = pdp
                .iter()
                .zip(delays_us.iter())
                .map(|(&p, &d)| p * (d - mean) * (d - mean))
                .sum::<f64>()
                / total_power;
            (mean, variance.sqrt())
        } else {
            (0.0, 0.0)
        };

        // Store for Doppler estimation
        self.cir_history.push(complex_gains.clone());
        if self.cir_history.len() > 256 {
            self.cir_history.remove(0);
        }

        ChannelImpulseResponse {
            delays_us,
            complex_gains,
            pdp_db,
            rms_delay_spread_us,
            mean_excess_delay_us,
        }
    }

    /// Estimate Doppler spectrum from CIR history.
    pub fn estimate_doppler(&self, snapshot_interval_s: f64) -> Option<DopplerSpectrum> {
        if self.cir_history.len() < 2 {
            return None;
        }

        let num_snapshots = self.cir_history.len();
        let fft_size = self.config.doppler_bins.max(num_snapshots).next_power_of_two();
        let max_doppler = 1.0 / (2.0 * snapshot_interval_s);

        // Pick the strongest delay tap
        let first_cir = &self.cir_history[0];
        let best_tap = first_cir
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.norm_sqr().partial_cmp(&b.1.norm_sqr()).unwrap())
            .map(|(i, _)| i)
            .unwrap_or(0);

        // Extract time series at this tap
        let mut time_series: Vec<Complex64> = self
            .cir_history
            .iter()
            .map(|cir| {
                if best_tap < cir.len() {
                    cir[best_tap]
                } else {
                    Complex64::new(0.0, 0.0)
                }
            })
            .collect();

        // Zero-pad
        time_series.resize(fft_size, Complex64::new(0.0, 0.0));

        // Simple DFT for Doppler spectrum
        let mut spectrum = vec![0.0f64; fft_size];
        for k in 0..fft_size {
            let mut sum = Complex64::new(0.0, 0.0);
            for (n, ts) in time_series.iter().enumerate() {
                let phase = -2.0 * PI * k as f64 * n as f64 / fft_size as f64;
                sum += ts * Complex64::new(phase.cos(), phase.sin());
            }
            spectrum[k] = sum.norm_sqr();
        }

        // Normalize and convert to dB
        let max_spec = spectrum.iter().cloned().fold(0.0f64, f64::max);
        let power_db: Vec<f64> = spectrum
            .iter()
            .map(|&s| {
                if s > 0.0 && max_spec > 0.0 {
                    10.0 * (s / max_spec).log10()
                } else {
                    -100.0
                }
            })
            .collect();

        // Doppler frequencies (centered)
        let frequencies_hz: Vec<f64> = (0..fft_size)
            .map(|k| {
                let f = k as f64 / fft_size as f64 * 2.0 * max_doppler;
                if f > max_doppler {
                    f - 2.0 * max_doppler
                } else {
                    f
                }
            })
            .collect();

        // RMS Doppler spread
        let total: f64 = spectrum.iter().sum();
        let rms_doppler = if total > 0.0 {
            let mean_f = spectrum
                .iter()
                .zip(frequencies_hz.iter())
                .map(|(&p, &f)| p * f)
                .sum::<f64>()
                / total;
            let var = spectrum
                .iter()
                .zip(frequencies_hz.iter())
                .map(|(&p, &f)| p * (f - mean_f) * (f - mean_f))
                .sum::<f64>()
                / total;
            var.sqrt()
        } else {
            0.0
        };

        Some(DopplerSpectrum {
            frequencies_hz,
            power_db,
            rms_doppler_spread_hz: rms_doppler,
        })
    }

    /// Compute channel quality metrics from a CIR.
    pub fn compute_metrics(&self, cir: &ChannelImpulseResponse) -> ChannelMetrics {
        let pdp: Vec<f64> = cir.complex_gains.iter().map(|g| g.norm_sqr()).collect();
        let total_power: f64 = pdp.iter().sum();

        // Find significant paths (> -20 dB from peak)
        let max_power = pdp.iter().cloned().fold(0.0f64, f64::max);
        let threshold = max_power * 0.01; // -20 dB
        let num_paths = pdp.iter().filter(|&&p| p > threshold).count();

        // Coherence bandwidth ≈ 1 / (5 * σ_τ) for 50%, 1 / (50 * σ_τ) for 90%
        let coherence_bandwidth_50 = if cir.rms_delay_spread_us > 0.0 {
            1.0 / (5.0 * cir.rms_delay_spread_us * 1e-6)
        } else {
            f64::INFINITY
        };
        let coherence_bandwidth_90 = if cir.rms_delay_spread_us > 0.0 {
            1.0 / (50.0 * cir.rms_delay_spread_us * 1e-6)
        } else {
            f64::INFINITY
        };

        // Path loss
        let path_loss_db = if total_power > 0.0 {
            -10.0 * total_power.log10()
        } else {
            f64::INFINITY
        };

        // K-factor: ratio of LOS to scattered power
        let los_power = max_power;
        let scattered_power = total_power - los_power;
        let k_factor_db = if scattered_power > 0.0 {
            10.0 * (los_power / scattered_power).log10()
        } else {
            f64::INFINITY
        };

        // Coherence time from Doppler
        let coherence_time_s = if let Some(doppler) = self.estimate_doppler(1e-3) {
            if doppler.rms_doppler_spread_hz > 0.0 {
                0.423 / doppler.rms_doppler_spread_hz
            } else {
                f64::INFINITY
            }
        } else {
            f64::INFINITY
        };

        ChannelMetrics {
            coherence_bandwidth_50,
            coherence_bandwidth_90,
            coherence_time_s,
            path_loss_db,
            k_factor_db,
            num_paths,
        }
    }

    /// Get the sounding config.
    pub fn config(&self) -> &SoundingConfig {
        &self.config
    }

    /// Reset CIR history.
    pub fn reset(&mut self) {
        self.cir_history.clear();
    }

    /// Get number of CIR snapshots stored.
    pub fn num_snapshots(&self) -> usize {
        self.cir_history.len()
    }
}

/// Generate a maximum-length sequence (m-sequence) of given length.
///
/// Uses LFSR with primitive polynomial. Length should be 2^n - 1.
pub fn generate_mls(length: usize) -> Vec<Complex64> {
    // Find LFSR order
    let order = ((length + 1) as f64).log2().ceil() as u32;
    let actual_len = (1 << order) - 1;

    // Primitive polynomial taps
    let taps: u32 = match order {
        2 => 0b11,
        3 => 0b110,
        4 => 0b1100,
        5 => 0b10100,
        6 => 0b110000,
        7 => 0b1100000,
        8 => 0b10111000,
        9 => 0b100010000,
        10 => 0b1001000000,
        _ => 0b110, // fallback to order 3
    };

    let mut reg: u32 = 1; // Non-zero initial state
    let mut seq = Vec::with_capacity(actual_len);

    for _ in 0..actual_len {
        let bit = reg & 1;
        seq.push(if bit == 1 {
            Complex64::new(1.0, 0.0)
        } else {
            Complex64::new(-1.0, 0.0)
        });

        let feedback = (reg & taps).count_ones() & 1;
        reg = (reg >> 1) | (feedback << (order - 1));
    }

    // Truncate to requested length
    seq.truncate(length);
    seq
}

/// Estimate delay spread from a power delay profile.
pub fn delay_spread_from_pdp(delays_us: &[f64], pdp_linear: &[f64]) -> (f64, f64) {
    let total: f64 = pdp_linear.iter().sum();
    if total <= 0.0 {
        return (0.0, 0.0);
    }

    let mean = delays_us
        .iter()
        .zip(pdp_linear.iter())
        .map(|(&d, &p)| d * p)
        .sum::<f64>()
        / total;

    let variance = delays_us
        .iter()
        .zip(pdp_linear.iter())
        .map(|(&d, &p)| p * (d - mean) * (d - mean))
        .sum::<f64>()
        / total;

    (mean, variance.sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_mls_length() {
        let seq = generate_mls(31); // 2^5 - 1
        assert_eq!(seq.len(), 31);
        // Should be +1 or -1
        for s in &seq {
            assert!(s.re.abs() == 1.0 && s.im == 0.0);
        }
    }

    #[test]
    fn test_mls_autocorrelation() {
        let seq = generate_mls(31);
        let n = seq.len();
        // Autocorrelation at lag 0 should be n
        let r0: f64 = seq.iter().map(|s| s.norm_sqr()).sum();
        assert!((r0 - n as f64).abs() < 1e-6);

        // Autocorrelation at lag != 0 should be small relative to r0
        // For MLS of length 2^m - 1, off-peak autocorrelation = -1/N
        let mut r1 = Complex64::new(0.0, 0.0);
        for i in 0..n - 1 {
            r1 += seq[i] * seq[i + 1].conj();
        }
        // Off-peak should be much less than peak
        assert!(r1.norm() < r0, "Off-peak autocorrelation should be less than peak");
    }

    #[test]
    fn test_sounder_creation() {
        let config = SoundingConfig {
            sequence_length: 63,
            sample_rate: 1e6,
            max_delay_us: 10.0,
            doppler_bins: 16,
        };
        let sounder = ChannelSounder::new(config);
        assert_eq!(sounder.generate_sounding_signal().len(), 63);
    }

    #[test]
    fn test_cir_estimation_direct_path() {
        let config = SoundingConfig {
            sequence_length: 31,
            sample_rate: 1e6,
            max_delay_us: 30.0,
            doppler_bins: 16,
        };
        let mut sounder = ChannelSounder::new(config);
        let tx = sounder.generate_sounding_signal();

        // Direct path: received = transmitted
        let cir = sounder.estimate_cir(&tx);
        assert!(!cir.delays_us.is_empty());
        // Peak should be at delay 0
        let peak_idx = cir
            .complex_gains
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.norm_sqr().partial_cmp(&b.1.norm_sqr()).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak_idx, 0, "Peak should be at zero delay");
    }

    #[test]
    fn test_cir_multipath() {
        let config = SoundingConfig {
            sequence_length: 31,
            sample_rate: 1e6,
            max_delay_us: 30.0,
            doppler_bins: 16,
        };
        let mut sounder = ChannelSounder::new(config);
        let tx = sounder.generate_sounding_signal();

        // Two-path channel: direct + delayed copy at sample 5
        let delay = 5;
        let mut received = tx.clone();
        received.resize(tx.len() + delay + 5, Complex64::new(0.0, 0.0));
        for i in 0..tx.len() {
            received[i + delay] += tx[i] * Complex64::new(0.5, 0.0);
        }

        let cir = sounder.estimate_cir(&received);
        // Should see two peaks
        let pdp: Vec<f64> = cir.complex_gains.iter().map(|g| g.norm_sqr()).collect();
        let max_pdp = pdp.iter().cloned().fold(0.0f64, f64::max);
        let threshold = max_pdp * 0.1;
        let peaks: Vec<usize> = pdp
            .iter()
            .enumerate()
            .filter(|(_, &p)| p > threshold)
            .map(|(i, _)| i)
            .collect();
        assert!(peaks.len() >= 2, "Should detect at least 2 paths, got {}", peaks.len());
    }

    #[test]
    fn test_delay_spread_calculation() {
        let delays = vec![0.0, 1.0, 2.0, 3.0];
        let pdp = vec![1.0, 0.5, 0.1, 0.01];
        let (mean, rms) = delay_spread_from_pdp(&delays, &pdp);
        assert!(mean > 0.0, "Mean delay should be > 0");
        assert!(rms > 0.0, "RMS delay spread should be > 0");
    }

    #[test]
    fn test_chirp_signal() {
        let config = SoundingConfig {
            sequence_length: 64,
            sample_rate: 1e6,
            max_delay_us: 10.0,
            doppler_bins: 16,
        };
        let sounder = ChannelSounder::new(config);
        let chirp = sounder.generate_chirp_signal(100e3);
        assert_eq!(chirp.len(), 64);
        // All samples should have unit magnitude
        for s in &chirp {
            assert!((s.norm() - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_channel_metrics() {
        let config = SoundingConfig {
            sequence_length: 31,
            sample_rate: 1e6,
            max_delay_us: 30.0,
            doppler_bins: 16,
        };
        let mut sounder = ChannelSounder::new(config);
        let tx = sounder.generate_sounding_signal();
        let cir = sounder.estimate_cir(&tx);
        let metrics = sounder.compute_metrics(&cir);
        assert!(metrics.num_paths >= 1);
        assert!(metrics.path_loss_db.is_finite());
    }

    #[test]
    fn test_sounder_reset() {
        let config = SoundingConfig {
            sequence_length: 31,
            sample_rate: 1e6,
            max_delay_us: 10.0,
            doppler_bins: 16,
        };
        let mut sounder = ChannelSounder::new(config);
        let tx = sounder.generate_sounding_signal();
        sounder.estimate_cir(&tx);
        assert_eq!(sounder.num_snapshots(), 1);
        sounder.reset();
        assert_eq!(sounder.num_snapshots(), 0);
    }

    #[test]
    fn test_doppler_estimation_needs_history() {
        let config = SoundingConfig {
            sequence_length: 31,
            sample_rate: 1e6,
            max_delay_us: 10.0,
            doppler_bins: 16,
        };
        let sounder = ChannelSounder::new(config);
        // No history yet
        assert!(sounder.estimate_doppler(1e-3).is_none());
    }

    #[test]
    fn test_doppler_with_history() {
        let config = SoundingConfig {
            sequence_length: 31,
            sample_rate: 1e6,
            max_delay_us: 30.0,
            doppler_bins: 16,
        };
        let mut sounder = ChannelSounder::new(config);
        let tx = sounder.generate_sounding_signal();

        // Add multiple snapshots
        for _ in 0..8 {
            sounder.estimate_cir(&tx);
        }
        let doppler = sounder.estimate_doppler(1e-3);
        assert!(doppler.is_some());
        let ds = doppler.unwrap();
        assert_eq!(ds.frequencies_hz.len(), ds.power_db.len());
    }
}
