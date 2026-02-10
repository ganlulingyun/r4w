//! Multipath profile extraction from channel impulse response measurements.
//!
//! This module provides tools for analyzing channel impulse responses (CIR) to
//! extract delay-magnitude-phase multipath profiles. Key capabilities include:
//!
//! - Peak detection in CIR measurements
//! - Per-path delay, magnitude, and phase extraction
//! - RMS delay spread and mean excess delay computation
//! - Coherence bandwidth estimation
//! - Rician K-factor estimation
//! - Path loss exponent estimation from multiple measurements
//! - Power delay profile (PDP) averaging
//! - Maximum excess delay (X dB threshold)
//!
//! # Example
//!
//! ```
//! use r4w_core::multipath_profile_extractor::{MultipathProfileExtractor, ExtractorConfig};
//!
//! // Create an extractor with default configuration
//! let config = ExtractorConfig {
//!     peak_threshold_db: -20.0,
//!     min_peak_separation_samples: 2,
//!     sample_rate_hz: 1e6,
//! };
//! let extractor = MultipathProfileExtractor::new(config);
//!
//! // A simple CIR with two paths: direct (sample 0) and reflection (sample 5)
//! let mut cir = vec![(0.0, 0.0); 20];
//! cir[0] = (1.0, 0.0);   // direct path, magnitude 1.0
//! cir[5] = (0.3, 0.4);   // reflected path
//!
//! let profile = extractor.extract(&cir);
//! assert_eq!(profile.paths.len(), 2);
//! assert!((profile.paths[0].delay_s - 0.0).abs() < 1e-12);
//! assert!((profile.paths[0].magnitude - 1.0).abs() < 1e-6);
//! ```

use std::f64::consts::PI;

/// Configuration for the multipath profile extractor.
#[derive(Debug, Clone)]
pub struct ExtractorConfig {
    /// Threshold in dB below the peak for detecting multipath components.
    pub peak_threshold_db: f64,
    /// Minimum separation in samples between detected peaks.
    pub min_peak_separation_samples: usize,
    /// Sample rate in Hz, used to convert sample indices to delays in seconds.
    pub sample_rate_hz: f64,
}

impl Default for ExtractorConfig {
    fn default() -> Self {
        Self {
            peak_threshold_db: -20.0,
            min_peak_separation_samples: 1,
            sample_rate_hz: 1e6,
        }
    }
}

/// A single detected multipath component.
#[derive(Debug, Clone)]
pub struct MultipathComponent {
    /// Delay of this path in seconds relative to the first detected path.
    pub delay_s: f64,
    /// Delay of this path in samples (index into the CIR).
    pub delay_samples: usize,
    /// Linear magnitude of the path.
    pub magnitude: f64,
    /// Phase of the path in radians (-pi, pi].
    pub phase_rad: f64,
    /// Power of the path (magnitude squared).
    pub power: f64,
    /// Power of the path in dB relative to the strongest path.
    pub power_db: f64,
}

/// Result of multipath profile extraction.
#[derive(Debug, Clone)]
pub struct MultipathProfile {
    /// Detected multipath components, sorted by delay.
    pub paths: Vec<MultipathComponent>,
    /// Mean excess delay (first moment of the PDP) in seconds.
    pub mean_excess_delay_s: f64,
    /// RMS delay spread (square root of the second central moment) in seconds.
    pub rms_delay_spread_s: f64,
    /// Maximum excess delay in seconds (last path delay minus first path delay).
    pub max_excess_delay_s: f64,
    /// Estimated coherence bandwidth in Hz (0.2 / rms_delay_spread).
    pub coherence_bandwidth_hz: f64,
    /// Estimated Rician K-factor in linear scale.
    /// Ratio of the strongest (direct) path power to the total scattered power.
    pub k_factor_linear: f64,
    /// Estimated Rician K-factor in dB.
    pub k_factor_db: f64,
}

/// Power delay profile: averaged power vs delay.
#[derive(Debug, Clone)]
pub struct PowerDelayProfile {
    /// Power values at each sample index (linear scale).
    pub power: Vec<f64>,
    /// Sample rate used when computing this PDP.
    pub sample_rate_hz: f64,
}

impl PowerDelayProfile {
    /// Return the delay axis in seconds for each sample.
    pub fn delay_axis(&self) -> Vec<f64> {
        (0..self.power.len())
            .map(|i| i as f64 / self.sample_rate_hz)
            .collect()
    }

    /// Return the power values in dB (relative to peak).
    pub fn power_db(&self) -> Vec<f64> {
        let peak = self.power.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        if peak <= 0.0 {
            return vec![f64::NEG_INFINITY; self.power.len()];
        }
        self.power.iter().map(|&p| 10.0 * (p / peak).log10()).collect()
    }
}

/// Multipath profile extractor.
///
/// Analyzes channel impulse response measurements to extract multipath
/// components and compute channel statistics.
#[derive(Debug, Clone)]
pub struct MultipathProfileExtractor {
    config: ExtractorConfig,
}

impl MultipathProfileExtractor {
    /// Create a new extractor with the given configuration.
    pub fn new(config: ExtractorConfig) -> Self {
        Self { config }
    }

    /// Extract a multipath profile from a channel impulse response.
    ///
    /// The CIR is given as a slice of complex samples `(re, im)`.
    /// Returns a `MultipathProfile` with detected paths and channel statistics.
    pub fn extract(&self, cir: &[(f64, f64)]) -> MultipathProfile {
        if cir.is_empty() {
            return MultipathProfile {
                paths: Vec::new(),
                mean_excess_delay_s: 0.0,
                rms_delay_spread_s: 0.0,
                max_excess_delay_s: 0.0,
                coherence_bandwidth_hz: f64::INFINITY,
                k_factor_linear: f64::INFINITY,
                k_factor_db: f64::INFINITY,
            };
        }

        let magnitudes: Vec<f64> = cir.iter().map(|&(re, im)| (re * re + im * im).sqrt()).collect();
        let peak_mag = magnitudes.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        if peak_mag <= 0.0 {
            return MultipathProfile {
                paths: Vec::new(),
                mean_excess_delay_s: 0.0,
                rms_delay_spread_s: 0.0,
                max_excess_delay_s: 0.0,
                coherence_bandwidth_hz: f64::INFINITY,
                k_factor_linear: f64::INFINITY,
                k_factor_db: f64::INFINITY,
            };
        }

        // Threshold in linear scale
        let threshold_linear = peak_mag * 10.0_f64.powf(self.config.peak_threshold_db / 20.0);

        // Detect peaks
        let peak_indices = self.detect_peaks(&magnitudes, threshold_linear);

        // Build multipath components
        let peak_power = peak_mag * peak_mag;
        let mut paths: Vec<MultipathComponent> = Vec::new();
        let first_delay = if peak_indices.is_empty() { 0 } else { peak_indices[0] };

        for &idx in &peak_indices {
            let (re, im) = cir[idx];
            let mag = magnitudes[idx];
            let power = mag * mag;
            let phase = im.atan2(re);
            let delay_samples_relative = idx - first_delay;
            let delay_s = delay_samples_relative as f64 / self.config.sample_rate_hz;
            let power_db = 10.0 * (power / peak_power).log10();

            paths.push(MultipathComponent {
                delay_s,
                delay_samples: idx,
                magnitude: mag,
                phase_rad: phase,
                power,
                power_db,
            });
        }

        // Compute statistics from detected paths
        let (mean_excess_delay_s, rms_delay_spread_s) = self.compute_delay_statistics(&paths);
        let max_excess_delay_s = if paths.len() >= 2 {
            paths.last().unwrap().delay_s - paths[0].delay_s
        } else {
            0.0
        };
        let coherence_bandwidth_hz = if rms_delay_spread_s > 0.0 {
            0.2 / rms_delay_spread_s
        } else {
            f64::INFINITY
        };
        let (k_factor_linear, k_factor_db) = self.compute_k_factor(&paths);

        MultipathProfile {
            paths,
            mean_excess_delay_s,
            rms_delay_spread_s,
            max_excess_delay_s,
            coherence_bandwidth_hz,
            k_factor_linear,
            k_factor_db,
        }
    }

    /// Detect peaks in the magnitude array above the threshold, respecting minimum
    /// separation between peaks.
    fn detect_peaks(&self, magnitudes: &[f64], threshold: f64) -> Vec<usize> {
        let n = magnitudes.len();
        if n == 0 {
            return Vec::new();
        }

        // Find all local maxima above threshold
        let mut candidates: Vec<(usize, f64)> = Vec::new();

        for i in 0..n {
            if magnitudes[i] < threshold {
                continue;
            }
            let left_ok = i == 0 || magnitudes[i] >= magnitudes[i - 1];
            let right_ok = i == n - 1 || magnitudes[i] >= magnitudes[i + 1];
            if left_ok && right_ok {
                candidates.push((i, magnitudes[i]));
            }
        }

        // Sort by magnitude descending for greedy selection
        candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

        // Greedy selection respecting minimum separation
        let mut selected: Vec<usize> = Vec::new();
        let min_sep = self.config.min_peak_separation_samples;

        for (idx, _mag) in candidates {
            let too_close = selected.iter().any(|&s| {
                let diff = if idx > s { idx - s } else { s - idx };
                diff < min_sep
            });
            if !too_close {
                selected.push(idx);
            }
        }

        // Sort by index (delay order)
        selected.sort();
        selected
    }

    /// Compute mean excess delay and RMS delay spread from detected paths.
    fn compute_delay_statistics(&self, paths: &[MultipathComponent]) -> (f64, f64) {
        if paths.is_empty() {
            return (0.0, 0.0);
        }

        let total_power: f64 = paths.iter().map(|p| p.power).sum();
        if total_power <= 0.0 {
            return (0.0, 0.0);
        }

        // Mean excess delay: weighted average of delays
        let mean_delay: f64 = paths.iter().map(|p| p.delay_s * p.power).sum::<f64>() / total_power;

        // RMS delay spread: sqrt of second central moment
        let variance: f64 = paths
            .iter()
            .map(|p| {
                let diff = p.delay_s - mean_delay;
                diff * diff * p.power
            })
            .sum::<f64>()
            / total_power;

        (mean_delay, variance.sqrt())
    }

    /// Compute Rician K-factor: ratio of direct (strongest) path power to total
    /// scattered power.
    fn compute_k_factor(&self, paths: &[MultipathComponent]) -> (f64, f64) {
        if paths.is_empty() {
            return (f64::INFINITY, f64::INFINITY);
        }
        if paths.len() == 1 {
            return (f64::INFINITY, f64::INFINITY);
        }

        // The direct path is the strongest one
        let direct_power = paths
            .iter()
            .map(|p| p.power)
            .fold(f64::NEG_INFINITY, f64::max);

        let total_scattered: f64 = paths.iter().map(|p| p.power).sum::<f64>() - direct_power;

        if total_scattered <= 0.0 {
            return (f64::INFINITY, f64::INFINITY);
        }

        let k_linear = direct_power / total_scattered;
        let k_db = 10.0 * k_linear.log10();

        (k_linear, k_db)
    }

    /// Compute a power delay profile from a CIR.
    ///
    /// Returns the magnitude-squared of each sample.
    pub fn compute_pdp(&self, cir: &[(f64, f64)]) -> PowerDelayProfile {
        let power: Vec<f64> = cir.iter().map(|&(re, im)| re * re + im * im).collect();
        PowerDelayProfile {
            power,
            sample_rate_hz: self.config.sample_rate_hz,
        }
    }

    /// Average multiple power delay profiles.
    ///
    /// All input CIRs must have the same length. Returns the element-wise
    /// average power delay profile.
    pub fn average_pdp(&self, cirs: &[Vec<(f64, f64)>]) -> PowerDelayProfile {
        if cirs.is_empty() {
            return PowerDelayProfile {
                power: Vec::new(),
                sample_rate_hz: self.config.sample_rate_hz,
            };
        }

        let n = cirs[0].len();
        let count = cirs.len() as f64;
        let mut avg_power = vec![0.0; n];

        for cir in cirs {
            let len = cir.len().min(n);
            for i in 0..len {
                let (re, im) = cir[i];
                avg_power[i] += (re * re + im * im) / count;
            }
        }

        PowerDelayProfile {
            power: avg_power,
            sample_rate_hz: self.config.sample_rate_hz,
        }
    }

    /// Compute the maximum excess delay for a given threshold in dB.
    ///
    /// Returns the delay (in seconds) of the last path whose power is within
    /// `threshold_db` of the peak power, relative to the first arriving path.
    /// Returns 0.0 if no paths exceed the threshold.
    pub fn max_excess_delay_db(&self, cir: &[(f64, f64)], threshold_db: f64) -> f64 {
        if cir.is_empty() {
            return 0.0;
        }

        let powers: Vec<f64> = cir.iter().map(|&(re, im)| re * re + im * im).collect();
        let peak_power = powers.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

        if peak_power <= 0.0 {
            return 0.0;
        }

        let threshold_linear = peak_power * 10.0_f64.powf(threshold_db / 10.0);

        // Find first sample above threshold
        let first = powers.iter().position(|&p| p >= threshold_linear);
        // Find last sample above threshold
        let last = powers.iter().rposition(|&p| p >= threshold_linear);

        match (first, last) {
            (Some(f), Some(l)) if l > f => (l - f) as f64 / self.config.sample_rate_hz,
            _ => 0.0,
        }
    }

    /// Estimate path loss exponent from multiple CIR measurements at known distances.
    ///
    /// Takes pairs of `(distance_m, cir)` and estimates the exponent `n` in:
    ///   PL(d) = PL(d0) + 10*n*log10(d/d0)
    ///
    /// Uses linear regression on log-distance vs received power (in dB).
    /// Returns `(path_loss_exponent, pl_d0_db)` where `pl_d0_db` is the path
    /// loss at the reference distance (1 m).
    ///
    /// Returns `None` if fewer than 2 measurements are provided or regression
    /// fails.
    pub fn estimate_path_loss_exponent(
        &self,
        measurements: &[(f64, Vec<(f64, f64)>)],
    ) -> Option<(f64, f64)> {
        if measurements.len() < 2 {
            return None;
        }

        // Compute total received power for each measurement
        let mut points: Vec<(f64, f64)> = Vec::new();
        for (distance, cir) in measurements {
            if *distance <= 0.0 {
                continue;
            }
            let total_power: f64 = cir.iter().map(|&(re, im)| re * re + im * im).sum();
            if total_power <= 0.0 {
                continue;
            }
            let power_db = 10.0 * total_power.log10();
            let log_dist = distance.log10();
            points.push((log_dist, power_db));
        }

        if points.len() < 2 {
            return None;
        }

        // Linear regression: power_db = a + b * log_dist
        // Path loss exponent n = -b / 10
        let n = points.len() as f64;
        let sum_x: f64 = points.iter().map(|(x, _)| x).sum();
        let sum_y: f64 = points.iter().map(|(_, y)| y).sum();
        let sum_xx: f64 = points.iter().map(|(x, _)| x * x).sum();
        let sum_xy: f64 = points.iter().map(|(x, y)| x * y).sum();

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return None;
        }

        let b = (n * sum_xy - sum_x * sum_y) / denom;
        let a = (sum_y - b * sum_x) / n;

        // n_pl is the path loss exponent (positive)
        let path_loss_exponent = -b / 10.0;

        // PL at d0=1m: at d=1m, log10(d)=0, power_db = a
        let pl_d0_db = -a;

        Some((path_loss_exponent, pl_d0_db))
    }
}

/// Utility: compute the magnitude of a complex sample.
#[inline]
pub fn complex_magnitude(sample: (f64, f64)) -> f64 {
    (sample.0 * sample.0 + sample.1 * sample.1).sqrt()
}

/// Utility: compute the phase of a complex sample in radians.
#[inline]
pub fn complex_phase(sample: (f64, f64)) -> f64 {
    sample.1.atan2(sample.0)
}

/// Utility: compute power (magnitude squared) of a complex sample.
#[inline]
pub fn complex_power(sample: (f64, f64)) -> f64 {
    sample.0 * sample.0 + sample.1 * sample.1
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_extractor() -> MultipathProfileExtractor {
        MultipathProfileExtractor::new(ExtractorConfig::default())
    }

    fn make_extractor(threshold_db: f64, min_sep: usize, sample_rate: f64) -> MultipathProfileExtractor {
        MultipathProfileExtractor::new(ExtractorConfig {
            peak_threshold_db: threshold_db,
            min_peak_separation_samples: min_sep,
            sample_rate_hz: sample_rate,
        })
    }

    #[test]
    fn test_empty_cir() {
        let ext = default_extractor();
        let profile = ext.extract(&[]);
        assert!(profile.paths.is_empty());
        assert_eq!(profile.mean_excess_delay_s, 0.0);
        assert_eq!(profile.rms_delay_spread_s, 0.0);
    }

    #[test]
    fn test_single_path() {
        let ext = default_extractor();
        let mut cir = vec![(0.0, 0.0); 10];
        cir[3] = (1.0, 0.0);

        let profile = ext.extract(&cir);
        assert_eq!(profile.paths.len(), 1);
        assert!((profile.paths[0].magnitude - 1.0).abs() < 1e-10);
        assert!((profile.paths[0].phase_rad - 0.0).abs() < 1e-10);
        assert_eq!(profile.paths[0].delay_samples, 3);
        assert_eq!(profile.mean_excess_delay_s, 0.0);
        assert_eq!(profile.rms_delay_spread_s, 0.0);
    }

    #[test]
    fn test_two_paths_delay() {
        let ext = make_extractor(-20.0, 1, 1e6);
        let mut cir = vec![(0.0, 0.0); 20];
        cir[2] = (1.0, 0.0);  // direct
        cir[7] = (0.5, 0.0);  // reflected

        let profile = ext.extract(&cir);
        assert_eq!(profile.paths.len(), 2);

        // First path delay should be 0 (relative)
        assert!((profile.paths[0].delay_s - 0.0).abs() < 1e-12);
        // Second path delay = (7-2) / 1e6 = 5e-6
        assert!((profile.paths[1].delay_s - 5e-6).abs() < 1e-12);
    }

    #[test]
    fn test_phase_extraction() {
        let ext = default_extractor();
        let mut cir = vec![(0.0, 0.0); 10];
        // 45 degrees: (cos(pi/4), sin(pi/4))
        let angle = PI / 4.0;
        cir[0] = (angle.cos(), angle.sin());

        let profile = ext.extract(&cir);
        assert_eq!(profile.paths.len(), 1);
        assert!((profile.paths[0].phase_rad - PI / 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_magnitude_extraction() {
        let ext = default_extractor();
        let mut cir = vec![(0.0, 0.0); 10];
        cir[0] = (3.0, 4.0); // magnitude = 5.0

        let profile = ext.extract(&cir);
        assert_eq!(profile.paths.len(), 1);
        assert!((profile.paths[0].magnitude - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_threshold_filtering() {
        // Threshold -6 dB means only paths within 6 dB of the peak are detected
        let ext = make_extractor(-6.0, 1, 1e6);
        let mut cir = vec![(0.0, 0.0); 20];
        cir[0] = (1.0, 0.0);   // 0 dB
        cir[5] = (0.5, 0.0);   // -6.02 dB (magnitude), just at threshold
        cir[10] = (0.1, 0.0);  // -20 dB, should be filtered out

        let profile = ext.extract(&cir);
        // The path at index 10 is well below -6 dB threshold
        assert!(profile.paths.len() >= 1);
        assert!(profile.paths.len() <= 2);
        // The weak path at index 10 should definitely not appear
        for p in &profile.paths {
            assert!(p.delay_samples != 10);
        }
    }

    #[test]
    fn test_min_peak_separation() {
        let ext = make_extractor(-20.0, 5, 1e6);
        let mut cir = vec![(0.0, 0.0); 20];
        cir[0] = (1.0, 0.0);
        cir[2] = (0.8, 0.0);  // too close (separation 2 < 5)
        cir[10] = (0.6, 0.0); // far enough

        let profile = ext.extract(&cir);
        // Should detect path at 0 and 10, but not 2 (too close to 0)
        assert_eq!(profile.paths.len(), 2);
        assert_eq!(profile.paths[0].delay_samples, 0);
        assert_eq!(profile.paths[1].delay_samples, 10);
    }

    #[test]
    fn test_mean_excess_delay() {
        let ext = make_extractor(-30.0, 1, 1e6);
        let mut cir = vec![(0.0, 0.0); 20];
        // Two equal-power paths
        cir[0] = (1.0, 0.0);
        cir[10] = (1.0, 0.0);

        let profile = ext.extract(&cir);
        // Mean excess delay = (0*1 + 10e-6*1) / (1+1) = 5e-6
        assert!((profile.mean_excess_delay_s - 5e-6).abs() < 1e-12);
    }

    #[test]
    fn test_rms_delay_spread() {
        let ext = make_extractor(-30.0, 1, 1e6);
        let mut cir = vec![(0.0, 0.0); 20];
        // Two equal-power paths at indices 0 and 10
        cir[0] = (1.0, 0.0);
        cir[10] = (1.0, 0.0);

        let profile = ext.extract(&cir);
        // Mean delay = 5e-6
        // Variance = ((0-5e-6)^2 * 1 + (10e-6-5e-6)^2 * 1) / 2 = 25e-12
        // RMS = 5e-6
        assert!((profile.rms_delay_spread_s - 5e-6).abs() < 1e-12);
    }

    #[test]
    fn test_coherence_bandwidth() {
        let ext = make_extractor(-30.0, 1, 1e6);
        let mut cir = vec![(0.0, 0.0); 20];
        cir[0] = (1.0, 0.0);
        cir[10] = (1.0, 0.0);

        let profile = ext.extract(&cir);
        // Coherence BW = 0.2 / rms_delay_spread = 0.2 / 5e-6 = 40000 Hz
        assert!((profile.coherence_bandwidth_hz - 40000.0).abs() < 1.0);
    }

    #[test]
    fn test_k_factor_strong_direct() {
        let ext = make_extractor(-30.0, 1, 1e6);
        let mut cir = vec![(0.0, 0.0); 20];
        cir[0] = (1.0, 0.0);   // direct: power=1.0
        cir[5] = (0.1, 0.0);   // scattered: power=0.01

        let profile = ext.extract(&cir);
        // K = 1.0 / 0.01 = 100
        assert!((profile.k_factor_linear - 100.0).abs() < 1e-6);
        assert!((profile.k_factor_db - 20.0).abs() < 0.01);
    }

    #[test]
    fn test_k_factor_equal_paths() {
        let ext = make_extractor(-30.0, 1, 1e6);
        let mut cir = vec![(0.0, 0.0); 20];
        cir[0] = (1.0, 0.0);
        cir[5] = (1.0, 0.0);

        let profile = ext.extract(&cir);
        // K = 1.0 / 1.0 = 1.0 (0 dB)
        assert!((profile.k_factor_linear - 1.0).abs() < 1e-10);
        assert!((profile.k_factor_db - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_k_factor_single_path() {
        let ext = default_extractor();
        let mut cir = vec![(0.0, 0.0); 10];
        cir[0] = (1.0, 0.0);

        let profile = ext.extract(&cir);
        assert!(profile.k_factor_linear.is_infinite());
        assert!(profile.k_factor_db.is_infinite());
    }

    #[test]
    fn test_compute_pdp() {
        let ext = default_extractor();
        let cir = vec![(1.0, 0.0), (0.0, 1.0), (0.5, 0.5)];
        let pdp = ext.compute_pdp(&cir);
        assert_eq!(pdp.power.len(), 3);
        assert!((pdp.power[0] - 1.0).abs() < 1e-10);
        assert!((pdp.power[1] - 1.0).abs() < 1e-10);
        assert!((pdp.power[2] - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_average_pdp() {
        let ext = make_extractor(-20.0, 1, 1e6);

        let cir1 = vec![(1.0, 0.0), (0.0, 0.0), (0.0, 0.0)];
        let cir2 = vec![(0.0, 0.0), (0.0, 0.0), (1.0, 0.0)];

        let pdp = ext.average_pdp(&[cir1, cir2]);
        assert_eq!(pdp.power.len(), 3);
        // avg power[0] = (1.0 + 0.0) / 2 = 0.5
        assert!((pdp.power[0] - 0.5).abs() < 1e-10);
        assert!((pdp.power[1] - 0.0).abs() < 1e-10);
        assert!((pdp.power[2] - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_average_pdp_empty() {
        let ext = default_extractor();
        let pdp = ext.average_pdp(&[]);
        assert!(pdp.power.is_empty());
    }

    #[test]
    fn test_max_excess_delay_db() {
        let ext = make_extractor(-30.0, 1, 1e6);
        let mut cir = vec![(0.0, 0.0); 100];
        cir[10] = (1.0, 0.0);  // peak
        cir[60] = (0.1, 0.0);  // -20 dB in power

        // With -10 dB threshold, the path at 60 is below (power = 0.01, -20 dB)
        let delay_10db = ext.max_excess_delay_db(&cir, -10.0);
        assert!((delay_10db - 0.0).abs() < 1e-12);

        // With -25 dB threshold, the path at 60 is included
        let delay_25db = ext.max_excess_delay_db(&cir, -25.0);
        assert!((delay_25db - 50e-6).abs() < 1e-12);
    }

    #[test]
    fn test_path_loss_exponent_estimation() {
        let ext = default_extractor();

        // Simulate measurements at different distances with n=2 (free space)
        // amplitude ~ 1/d, power ~ 1/d^2, power_db = -20*log10(d), so n = 2
        // Power ~ 1/d^(2n) in linear, so for n=2: power ~ 1/d^4
        let mut measurements = Vec::new();
        for &d in &[10.0, 20.0, 50.0, 100.0] {
            let amplitude = 1.0 / d; // n=2: amplitude ~ 1/d, power ~ 1/d^2
            let cir = vec![(amplitude, 0.0)];
            measurements.push((d, cir));
        }

        let result = ext.estimate_path_loss_exponent(&measurements);
        assert!(result.is_some());
        let (n, _pl_d0) = result.unwrap();
        // Should be close to 2.0
        assert!((n - 2.0).abs() < 0.1, "Expected ~2.0, got {}", n);
    }

    #[test]
    fn test_path_loss_exponent_insufficient_data() {
        let ext = default_extractor();
        let measurements = vec![(10.0, vec![(1.0, 0.0)])];
        assert!(ext.estimate_path_loss_exponent(&measurements).is_none());
    }

    #[test]
    fn test_complex_magnitude_utility() {
        assert!((complex_magnitude((3.0, 4.0)) - 5.0).abs() < 1e-10);
        assert!((complex_magnitude((0.0, 0.0)) - 0.0).abs() < 1e-10);
        assert!((complex_magnitude((1.0, 0.0)) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_phase_utility() {
        assert!((complex_phase((1.0, 0.0)) - 0.0).abs() < 1e-10);
        assert!((complex_phase((0.0, 1.0)) - PI / 2.0).abs() < 1e-10);
        assert!((complex_phase((-1.0, 0.0)) - PI).abs() < 1e-10);
    }

    #[test]
    fn test_complex_power_utility() {
        assert!((complex_power((3.0, 4.0)) - 25.0).abs() < 1e-10);
    }

    #[test]
    fn test_power_delay_profile_db() {
        let pdp = PowerDelayProfile {
            power: vec![1.0, 0.1, 0.01],
            sample_rate_hz: 1e6,
        };
        let db = pdp.power_db();
        assert!((db[0] - 0.0).abs() < 1e-10);
        assert!((db[1] - (-10.0)).abs() < 1e-6);
        assert!((db[2] - (-20.0)).abs() < 1e-6);
    }

    #[test]
    fn test_pdp_delay_axis() {
        let pdp = PowerDelayProfile {
            power: vec![1.0, 0.5, 0.25],
            sample_rate_hz: 1e6,
        };
        let delays = pdp.delay_axis();
        assert_eq!(delays.len(), 3);
        assert!((delays[0] - 0.0).abs() < 1e-12);
        assert!((delays[1] - 1e-6).abs() < 1e-12);
        assert!((delays[2] - 2e-6).abs() < 1e-12);
    }

    #[test]
    fn test_max_excess_delay_s() {
        let ext = make_extractor(-30.0, 1, 1e6);
        let mut cir = vec![(0.0, 0.0); 30];
        cir[5] = (1.0, 0.0);
        cir[25] = (0.5, 0.0);

        let profile = ext.extract(&cir);
        // max excess delay = (25-5)/1e6 = 20e-6
        assert!((profile.max_excess_delay_s - 20e-6).abs() < 1e-12);
    }

    #[test]
    fn test_all_zero_cir() {
        let ext = default_extractor();
        let cir = vec![(0.0, 0.0); 10];
        let profile = ext.extract(&cir);
        assert!(profile.paths.is_empty());
    }

    #[test]
    fn test_power_db_relative() {
        let ext = make_extractor(-30.0, 1, 1e6);
        let mut cir = vec![(0.0, 0.0); 20];
        cir[0] = (1.0, 0.0);   // power = 1.0
        cir[10] = (0.5, 0.0);  // power = 0.25 -> -6.02 dB

        let profile = ext.extract(&cir);
        assert_eq!(profile.paths.len(), 2);
        assert!((profile.paths[0].power_db - 0.0).abs() < 1e-10);
        assert!((profile.paths[1].power_db - (-6.020599913279624)).abs() < 1e-6);
    }
}
