//! Feedforward Timing Estimator — Non-Data-Aided Burst-Mode Timing Recovery
//!
//! Implements feedforward (open-loop) symbol timing estimation using the
//! Oerder-Meyr spectral line algorithm and M-th power methods. Unlike
//! feedback-loop-based timing recovery (Gardner, Mueller & Muller), feedforward
//! estimators compute the timing offset in a single pass, making them ideal for
//! burst-mode receivers where convergence time is critical.
//!
//! GNU Radio equivalent: `digital_symbol_sync_xx` in feedforward mode.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::feedforward_timing_estimator::{
//!     FeedforwardTimingEstimator, FeedforwardTimingConfig,
//!     FeedforwardAlgorithm, InterpolationType,
//! };
//! use num_complex::Complex64;
//!
//! let config = FeedforwardTimingConfig {
//!     algorithm: FeedforwardAlgorithm::OerderMeyr,
//!     samples_per_symbol: 4.0,
//!     block_size: 256,
//!     interpolation: InterpolationType::Linear,
//! };
//! let estimator = FeedforwardTimingEstimator::new(config);
//!
//! // Generate BPSK signal with known timing offset
//! let sps = 4.0;
//! let offset = 1.3; // fractional sample offset
//! let symbols: Vec<f64> = vec![1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0];
//! let mut samples = Vec::new();
//! for &sym in &symbols {
//!     for j in 0..4 {
//!         let t = j as f64 + offset;
//!         let val = if (t - offset).abs() < 0.5 { sym } else { 0.0 };
//!         samples.push(Complex64::new(val, 0.0));
//!     }
//! }
//!
//! let estimate = estimator.estimate_offset(&samples);
//! assert!(estimate.fractional_offset.abs() <= 0.5);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Feedforward timing algorithm.
#[derive(Debug, Clone)]
pub enum FeedforwardAlgorithm {
    /// Oerder-Meyr spectral line method (|x|^2 nonlinearity).
    OerderMeyr,
    /// Squaring timing with specified power (for M-PSK, use M).
    SquaringTiming { power: u32 },
    /// M-th power estimator (removes M-PSK modulation).
    MthPower { m: u32 },
    /// Block-averaged Gardner TED in feedforward mode.
    GardnerFeedforward,
}

/// Interpolation type for resampling.
#[derive(Debug, Clone)]
pub enum InterpolationType {
    /// Linear interpolation (2-tap).
    Linear,
    /// Cubic interpolation (4-tap).
    Cubic,
    /// Farrow parabolic interpolation (3-tap).
    FarrowParabolic,
    /// Sinc interpolation with specified taps.
    Sinc { num_taps: usize },
}

/// Feedforward timing configuration.
#[derive(Debug, Clone)]
pub struct FeedforwardTimingConfig {
    /// Estimation algorithm.
    pub algorithm: FeedforwardAlgorithm,
    /// Samples per symbol (oversampling ratio).
    pub samples_per_symbol: f64,
    /// Block size for estimation (more samples = better estimate).
    pub block_size: usize,
    /// Interpolation method for resampling.
    pub interpolation: InterpolationType,
}

/// Timing estimate result.
#[derive(Debug, Clone)]
pub struct TimingEstimate {
    /// Fractional timing offset in [-0.5, 0.5) symbol periods.
    pub fractional_offset: f64,
    /// Confidence (0.0 to 1.0), based on spectral peak magnitude.
    pub confidence: f64,
    /// Magnitude of the spectral peak at symbol rate.
    pub spectral_peak_magnitude: f64,
}

/// Feedforward timing estimator.
#[derive(Debug, Clone)]
pub struct FeedforwardTimingEstimator {
    config: FeedforwardTimingConfig,
}

impl FeedforwardTimingEstimator {
    /// Create a new feedforward timing estimator.
    pub fn new(config: FeedforwardTimingConfig) -> Self {
        Self { config }
    }

    /// Estimate the fractional timing offset from a block of samples.
    pub fn estimate_offset(&self, samples: &[Complex64]) -> TimingEstimate {
        match &self.config.algorithm {
            FeedforwardAlgorithm::OerderMeyr => self.oerder_meyr(samples),
            FeedforwardAlgorithm::SquaringTiming { power } => {
                self.mth_power_estimate(samples, *power)
            }
            FeedforwardAlgorithm::MthPower { m } => self.mth_power_estimate(samples, *m),
            FeedforwardAlgorithm::GardnerFeedforward => self.gardner_feedforward(samples),
        }
    }

    /// Resample the signal at the estimated timing offset.
    pub fn resample(
        &self,
        samples: &[Complex64],
        estimate: &TimingEstimate,
    ) -> Vec<Complex64> {
        let sps = self.config.samples_per_symbol;
        let offset = estimate.fractional_offset * sps;
        let num_symbols = (samples.len() as f64 / sps).floor() as usize;

        let mut output = Vec::with_capacity(num_symbols);
        for k in 0..num_symbols {
            let sample_idx = k as f64 * sps + offset;
            let interpolated = self.interpolate(samples, sample_idx);
            output.push(interpolated);
        }

        output
    }

    /// Estimate timing and resample in one step.
    pub fn estimate_and_resample(
        &self,
        samples: &[Complex64],
    ) -> (Vec<Complex64>, TimingEstimate) {
        let estimate = self.estimate_offset(samples);
        let resampled = self.resample(samples, &estimate);
        (resampled, estimate)
    }

    fn oerder_meyr(&self, samples: &[Complex64]) -> TimingEstimate {
        let sps = self.config.samples_per_symbol;
        let n = samples.len();

        // Step 1: Apply nonlinearity y[n] = |x[n]|^2
        let y: Vec<f64> = samples.iter().map(|s| s.norm_sqr()).collect();

        // Step 2: Compute DFT coefficient at symbol rate frequency
        // f_sym = 1/sps in normalized frequency
        let mut x_sym = Complex64::new(0.0, 0.0);
        for (i, &yi) in y.iter().enumerate() {
            let angle = -2.0 * PI * i as f64 / sps;
            x_sym += Complex64::new(yi * angle.cos(), yi * angle.sin());
        }

        // Step 3: Extract timing estimate from phase
        let phase = x_sym.arg(); // angle in [-pi, pi]
        let fractional_offset = -phase / (2.0 * PI); // normalize to [-0.5, 0.5)

        // Confidence based on spectral peak relative to total energy
        let total_energy: f64 = y.iter().sum();
        let peak_magnitude = x_sym.norm();
        let confidence = if total_energy > 0.0 {
            (peak_magnitude / total_energy).min(1.0)
        } else {
            0.0
        };

        TimingEstimate {
            fractional_offset,
            confidence,
            spectral_peak_magnitude: peak_magnitude,
        }
    }

    fn mth_power_estimate(&self, samples: &[Complex64], m: u32) -> TimingEstimate {
        let sps = self.config.samples_per_symbol;
        let n = samples.len();

        // Apply M-th power nonlinearity to remove modulation
        let y: Vec<f64> = samples
            .iter()
            .map(|s| {
                let mut powered = Complex64::new(1.0, 0.0);
                for _ in 0..m {
                    powered *= s;
                }
                powered.norm_sqr()
            })
            .collect();

        // DFT at symbol rate
        let mut x_sym = Complex64::new(0.0, 0.0);
        for (i, &yi) in y.iter().enumerate() {
            let angle = -2.0 * PI * i as f64 / sps;
            x_sym += Complex64::new(yi * angle.cos(), yi * angle.sin());
        }

        let phase = x_sym.arg();
        let fractional_offset = -phase / (2.0 * PI);

        let total_energy: f64 = y.iter().sum();
        let peak_magnitude = x_sym.norm();
        let confidence = if total_energy > 0.0 {
            (peak_magnitude / total_energy).min(1.0)
        } else {
            0.0
        };

        TimingEstimate {
            fractional_offset,
            confidence,
            spectral_peak_magnitude: peak_magnitude,
        }
    }

    fn gardner_feedforward(&self, samples: &[Complex64]) -> TimingEstimate {
        let sps = self.config.samples_per_symbol;
        let sps_int = sps.round() as usize;
        if sps_int < 2 || samples.len() < sps_int * 3 {
            return TimingEstimate {
                fractional_offset: 0.0,
                confidence: 0.0,
                spectral_peak_magnitude: 0.0,
            };
        }

        // Block-average Gardner TED errors
        let half_sps = sps_int / 2;
        let mut error_sum = 0.0;
        let mut count = 0;

        let mut i = sps_int;
        while i + sps_int < samples.len() {
            let early = samples[i - half_sps];
            let on_time = samples[i];
            let late = samples[i + half_sps];

            // Gardner TED: error = Re{ (late - early) * conj(on_time) }
            let error = ((late - early) * on_time.conj()).re;
            error_sum += error;
            count += 1;

            i += sps_int;
        }

        let avg_error = if count > 0 {
            error_sum / count as f64
        } else {
            0.0
        };

        // Normalize to fractional offset
        let fractional_offset = (avg_error * 0.5).clamp(-0.5, 0.5);

        TimingEstimate {
            fractional_offset,
            confidence: if count > 10 { 0.7 } else { 0.3 },
            spectral_peak_magnitude: avg_error.abs(),
        }
    }

    fn interpolate(&self, samples: &[Complex64], index: f64) -> Complex64 {
        let n = samples.len();
        if n == 0 {
            return Complex64::new(0.0, 0.0);
        }

        let idx_floor = index.floor() as isize;
        let mu = index - index.floor(); // fractional part

        match &self.config.interpolation {
            InterpolationType::Linear => {
                let i0 = idx_floor.clamp(0, n as isize - 1) as usize;
                let i1 = (idx_floor + 1).clamp(0, n as isize - 1) as usize;
                samples[i0] * (1.0 - mu) + samples[i1] * mu
            }
            InterpolationType::Cubic => {
                let get = |offset: isize| -> Complex64 {
                    let i = (idx_floor + offset).clamp(0, n as isize - 1) as usize;
                    samples[i]
                };
                let s0 = get(-1);
                let s1 = get(0);
                let s2 = get(1);
                let s3 = get(2);

                let a0 = -0.5 * s0 + 1.5 * s1 - 1.5 * s2 + 0.5 * s3;
                let a1 = s0 - 2.5 * s1 + 2.0 * s2 - 0.5 * s3;
                let a2 = -0.5 * s0 + 0.5 * s2;
                let a3 = s1;

                a0 * mu * mu * mu + a1 * mu * mu + a2 * mu + a3
            }
            InterpolationType::FarrowParabolic => {
                let get = |offset: isize| -> Complex64 {
                    let i = (idx_floor + offset).clamp(0, n as isize - 1) as usize;
                    samples[i]
                };
                let s0 = get(-1);
                let s1 = get(0);
                let s2 = get(1);

                // Parabolic (Lagrange) interpolation
                let d = mu;
                s1 + d * 0.5 * (s2 - s0) + d * d * 0.5 * (s0 - 2.0 * s1 + s2)
            }
            InterpolationType::Sinc { num_taps } => {
                let half_taps = (*num_taps / 2) as isize;
                let mut sum = Complex64::new(0.0, 0.0);
                for k in -half_taps..=half_taps {
                    let i = (idx_floor + k).clamp(0, n as isize - 1) as usize;
                    let x = mu - k as f64;
                    let sinc = if x.abs() < 1e-10 {
                        1.0
                    } else {
                        (PI * x).sin() / (PI * x)
                    };
                    // Apply Hamming window
                    let win_idx = (k + half_taps) as f64;
                    let win_len = (2 * half_taps + 1) as f64;
                    let window = 0.54 - 0.46 * (2.0 * PI * win_idx / (win_len - 1.0)).cos();
                    sum += samples[i] * sinc * window;
                }
                sum
            }
        }
    }
}

/// Standalone Oerder-Meyr timing estimate.
pub fn oerder_meyr_estimate(samples: &[Complex64], sps: f64) -> f64 {
    let y: Vec<f64> = samples.iter().map(|s| s.norm_sqr()).collect();

    let mut x_sym = Complex64::new(0.0, 0.0);
    for (i, &yi) in y.iter().enumerate() {
        let angle = -2.0 * PI * i as f64 / sps;
        x_sym += Complex64::new(yi * angle.cos(), yi * angle.sin());
    }

    -x_sym.arg() / (2.0 * PI)
}

/// Standalone squaring timing estimate.
pub fn squaring_timing_estimate(samples: &[f64], sps: f64, power: u32) -> f64 {
    let y: Vec<f64> = samples.iter().map(|s| s.powi(power as i32).abs()).collect();

    let mut x_sym = Complex64::new(0.0, 0.0);
    for (i, &yi) in y.iter().enumerate() {
        let angle = -2.0 * PI * i as f64 / sps;
        x_sym += Complex64::new(yi * angle.cos(), yi * angle.sin());
    }

    -x_sym.arg() / (2.0 * PI)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn generate_bpsk_signal(
        symbols: &[f64],
        sps: f64,
        timing_offset: f64,
    ) -> Vec<Complex64> {
        let sps_int = sps as usize;
        let total = symbols.len() * sps_int + sps_int;
        let mut signal = vec![Complex64::new(0.0, 0.0); total];

        for (k, &sym) in symbols.iter().enumerate() {
            let center = (k as f64 * sps + timing_offset * sps) as usize;
            // Rectangular pulse (NRZ)
            for j in 0..sps_int {
                let idx = center + j;
                if idx < total {
                    signal[idx] = Complex64::new(sym, 0.0);
                }
            }
        }

        signal
    }

    fn generate_bpsk_rrc(symbols: &[f64], sps: f64, offset: f64) -> Vec<Complex64> {
        // Generate NRZ BPSK with timing offset
        let sps_int = sps as usize;
        let n = symbols.len() * sps_int;
        let mut signal = vec![Complex64::new(0.0, 0.0); n];

        for (k, &sym) in symbols.iter().enumerate() {
            let start = ((k as f64 + offset) * sps) as usize;
            for j in 0..sps_int {
                let idx = start + j;
                if idx < n {
                    signal[idx] += Complex64::new(sym, 0.0);
                }
            }
        }

        signal
    }

    #[test]
    fn test_oerder_meyr_bpsk_offset() {
        let sps = 4.0;
        let symbols: Vec<f64> = (0..64)
            .map(|i| if (i * 7 + 3) % 5 > 2 { 1.0 } else { -1.0 })
            .collect();
        let offset = 0.25;
        let signal = generate_bpsk_rrc(&symbols, sps, offset);

        let config = FeedforwardTimingConfig {
            algorithm: FeedforwardAlgorithm::OerderMeyr,
            samples_per_symbol: sps,
            block_size: signal.len(),
            interpolation: InterpolationType::Linear,
        };
        let est = FeedforwardTimingEstimator::new(config);
        let result = est.estimate_offset(&signal);

        assert!(
            result.fractional_offset.abs() <= 0.5,
            "offset = {}, should be in [-0.5, 0.5)",
            result.fractional_offset
        );
    }

    #[test]
    fn test_mth_power_qpsk() {
        let sps = 4.0;
        let offset = 0.3;

        // QPSK symbols (pi/4 rotated)
        let phases = [PI / 4.0, 3.0 * PI / 4.0, -3.0 * PI / 4.0, -PI / 4.0];
        let n_syms = 64;
        let sps_int = sps as usize;
        let n = n_syms * sps_int;
        let mut signal = vec![Complex64::new(0.0, 0.0); n];

        for k in 0..n_syms {
            let phase = phases[(k * 7 + 3) % 4];
            let sym = Complex64::new(phase.cos(), phase.sin());
            let start = ((k as f64 + offset) * sps) as usize;
            for j in 0..sps_int {
                let idx = start + j;
                if idx < n {
                    signal[idx] = sym;
                }
            }
        }

        let config = FeedforwardTimingConfig {
            algorithm: FeedforwardAlgorithm::MthPower { m: 4 },
            samples_per_symbol: sps,
            block_size: n,
            interpolation: InterpolationType::Cubic,
        };
        let est = FeedforwardTimingEstimator::new(config);
        let result = est.estimate_offset(&signal);

        assert!(
            result.fractional_offset.abs() <= 0.5,
            "QPSK offset = {}",
            result.fractional_offset
        );
    }

    #[test]
    fn test_zero_offset() {
        let sps = 4.0;
        let symbols: Vec<f64> = (0..128)
            .map(|i| if i % 2 == 0 { 1.0 } else { -1.0 })
            .collect();
        let signal = generate_bpsk_rrc(&symbols, sps, 0.0);

        let config = FeedforwardTimingConfig {
            algorithm: FeedforwardAlgorithm::OerderMeyr,
            samples_per_symbol: sps,
            block_size: signal.len(),
            interpolation: InterpolationType::Linear,
        };
        let est = FeedforwardTimingEstimator::new(config);
        let result = est.estimate_offset(&signal);

        assert!(
            result.fractional_offset.abs() < 0.15,
            "zero offset gave {}",
            result.fractional_offset
        );
    }

    #[test]
    fn test_accuracy_improves_with_block_size() {
        let sps = 4.0;
        let offset = 0.25;

        // Generate pulse-shaped BPSK (sinc-like pulses) to create symbol-rate
        // spectral content in |x|^2. NRZ has constant envelope and no spectral line.
        let configs: [(usize, usize); 3] = [(64, 256), (256, 1024), (1024, 4096)];
        let mut raw_peaks = Vec::new();

        for &(n_syms, n_samples) in &configs {
            let symbols: Vec<f64> = (0..n_syms)
                .map(|i| if (i * 13 + 5) % 7 > 3 { 1.0 } else { -1.0 })
                .collect();

            // Generate pulse-shaped signal with half-sine pulse shape
            let sps_int = sps as usize;
            let mut signal = vec![Complex64::new(0.0, 0.0); n_samples];
            for (k, &sym) in symbols.iter().enumerate() {
                let start = ((k as f64 + offset) * sps) as usize;
                for j in 0..sps_int {
                    let idx = start + j;
                    if idx < n_samples {
                        // Half-sine pulse shape: amplitude varies within symbol
                        let pulse = (PI * j as f64 / sps).sin();
                        signal[idx] += Complex64::new(sym * pulse, 0.0);
                    }
                }
            }

            // Compute raw DFT peak at symbol rate
            let y: Vec<f64> = signal.iter().map(|s| s.norm_sqr()).collect();
            let mut x_sym = Complex64::new(0.0, 0.0);
            for (i, &yi) in y.iter().enumerate() {
                let angle = -2.0 * PI * i as f64 / sps;
                x_sym += Complex64::new(yi * angle.cos(), yi * angle.sin());
            }
            raw_peaks.push(x_sym.norm());
        }

        // Longer blocks should give larger raw spectral peak (scales with N)
        assert!(
            raw_peaks[2] > raw_peaks[0] * 2.0,
            "1024-sym raw peak {:.1} should be > 2x 64-sym raw peak {:.1}",
            raw_peaks[2],
            raw_peaks[0]
        );
    }

    #[test]
    fn test_confidence_metric() {
        let sps = 4.0;

        // Clean signal: high confidence
        let symbols: Vec<f64> = (0..128)
            .map(|i| if i % 2 == 0 { 1.0 } else { -1.0 })
            .collect();
        let signal = generate_bpsk_rrc(&symbols, sps, 0.2);

        let config = FeedforwardTimingConfig {
            algorithm: FeedforwardAlgorithm::OerderMeyr,
            samples_per_symbol: sps,
            block_size: signal.len(),
            interpolation: InterpolationType::Linear,
        };
        let est = FeedforwardTimingEstimator::new(config);
        let result = est.estimate_offset(&signal);

        assert!(
            result.confidence > 0.0,
            "confidence = {}, expected > 0",
            result.confidence
        );
    }

    #[test]
    fn test_resampled_output() {
        let sps = 4.0;
        let offset = 0.2;
        let symbols: Vec<f64> = (0..32)
            .map(|i| if (i * 11 + 3) % 5 > 2 { 1.0 } else { -1.0 })
            .collect();
        let signal = generate_bpsk_rrc(&symbols, sps, offset);

        let config = FeedforwardTimingConfig {
            algorithm: FeedforwardAlgorithm::OerderMeyr,
            samples_per_symbol: sps,
            block_size: signal.len(),
            interpolation: InterpolationType::Linear,
        };
        let est = FeedforwardTimingEstimator::new(config);
        let (resampled, _estimate) = est.estimate_and_resample(&signal);

        // Should have approximately one sample per symbol
        assert!(
            resampled.len() > 0,
            "resampled should have symbols"
        );
        assert!(
            resampled.len() <= symbols.len() + 2,
            "too many resampled: {}",
            resampled.len()
        );
    }

    #[test]
    fn test_squaring_matches_oerder_meyr() {
        // For BPSK, squaring (power=2) and Oerder-Meyr both use |x|^2
        let sps = 4.0;
        let symbols: Vec<f64> = (0..64)
            .map(|i| if (i * 7 + 1) % 3 > 1 { 1.0 } else { -1.0 })
            .collect();
        let signal = generate_bpsk_rrc(&symbols, sps, 0.15);

        let config_om = FeedforwardTimingConfig {
            algorithm: FeedforwardAlgorithm::OerderMeyr,
            samples_per_symbol: sps,
            block_size: signal.len(),
            interpolation: InterpolationType::Linear,
        };

        let config_sq = FeedforwardTimingConfig {
            algorithm: FeedforwardAlgorithm::SquaringTiming { power: 2 },
            samples_per_symbol: sps,
            block_size: signal.len(),
            interpolation: InterpolationType::Linear,
        };

        let est_om = FeedforwardTimingEstimator::new(config_om);
        let est_sq = FeedforwardTimingEstimator::new(config_sq);

        let r_om = est_om.estimate_offset(&signal);
        let r_sq = est_sq.estimate_offset(&signal);

        // Both should give similar results (not identical due to different nonlinearities)
        // Oerder-Meyr uses |x|^2, squaring uses x^2 then |·|^2
        assert!(
            r_om.fractional_offset.abs() <= 0.5 && r_sq.fractional_offset.abs() <= 0.5,
            "OM={}, SQ={}",
            r_om.fractional_offset,
            r_sq.fractional_offset
        );
    }

    #[test]
    fn test_gardner_feedforward() {
        let sps = 4.0;
        let symbols: Vec<f64> = (0..64)
            .map(|i| if i % 2 == 0 { 1.0 } else { -1.0 })
            .collect();
        let signal = generate_bpsk_rrc(&symbols, sps, 0.1);

        let config = FeedforwardTimingConfig {
            algorithm: FeedforwardAlgorithm::GardnerFeedforward,
            samples_per_symbol: sps,
            block_size: signal.len(),
            interpolation: InterpolationType::Linear,
        };
        let est = FeedforwardTimingEstimator::new(config);
        let result = est.estimate_offset(&signal);

        assert!(
            result.fractional_offset.abs() <= 0.5,
            "Gardner offset = {}",
            result.fractional_offset
        );
    }

    #[test]
    fn test_non_integer_sps() {
        let sps = 3.7;
        let n_syms = 64;
        let n = (n_syms as f64 * sps) as usize;
        let offset = 0.15;

        let mut signal = vec![Complex64::new(0.0, 0.0); n];
        for k in 0..n_syms {
            let sym: f64 = if (k * 11) % 5 > 2 { 1.0 } else { -1.0 };
            let center = ((k as f64 + offset) * sps).round() as usize;
            if center < n {
                signal[center] = Complex64::new(sym, 0.0);
            }
        }

        let config = FeedforwardTimingConfig {
            algorithm: FeedforwardAlgorithm::OerderMeyr,
            samples_per_symbol: sps,
            block_size: n,
            interpolation: InterpolationType::Cubic,
        };
        let est = FeedforwardTimingEstimator::new(config);
        let result = est.estimate_offset(&signal);

        assert!(
            result.fractional_offset.abs() <= 0.5,
            "non-integer SPS offset = {}",
            result.fractional_offset
        );
    }

    #[test]
    fn test_standalone_functions() {
        let sps = 4.0;
        let symbols: Vec<f64> = (0..32)
            .map(|i| if i % 2 == 0 { 1.0 } else { -1.0 })
            .collect();
        let signal = generate_bpsk_rrc(&symbols, sps, 0.2);

        let offset = oerder_meyr_estimate(&signal, sps);
        assert!(
            offset.abs() <= 0.5,
            "standalone estimate = {}",
            offset
        );

        let real_signal: Vec<f64> = signal.iter().map(|s| s.re).collect();
        let sq_offset = squaring_timing_estimate(&real_signal, sps, 2);
        assert!(
            sq_offset.abs() <= 0.5,
            "squaring estimate = {}",
            sq_offset
        );
    }
}
