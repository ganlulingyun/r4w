//! Range-Doppler target detection using CFAR (Constant False Alarm Rate) algorithms.
//!
//! This module detects targets in a range-Doppler map by finding peaks above a CFAR
//! threshold and extracting target parameters (range bin, Doppler bin, SNR, power).
//!
//! Two CFAR variants are provided:
//!
//! - **CA-CFAR** (Cell Averaging): estimates the noise floor from the mean power in
//!   surrounding training cells, then applies a threshold factor.
//! - **OS-CFAR** (Ordered Statistic): sorts the training cell powers and selects the
//!   k-th ordered sample as the noise estimate, providing robustness against
//!   interfering targets in the training window.
//!
//! Adjacent detections are merged via peak clustering so that a single extended target
//! does not produce multiple reports.
//!
//! # Example
//!
//! ```
//! use r4w_core::range_doppler_detector::{RangeDopplerDetector, DetectionParams, CfarMode};
//!
//! // Build a small 8x8 range-Doppler map with one target at (3, 5).
//! let mut rdm = vec![vec![1.0_f64; 8]; 8];
//! rdm[3][5] = 100.0; // inject a strong target
//!
//! let params = DetectionParams {
//!     threshold_factor: 3.0,
//!     guard_cells: 1,
//!     training_cells: 2,
//!     min_peak_separation: 2,
//!     cfar_mode: CfarMode::CellAveraging,
//! };
//!
//! let detector = RangeDopplerDetector::new(params);
//! let detections = detector.detect(&rdm);
//!
//! assert!(!detections.is_empty());
//! assert_eq!(detections[0].range_bin, 3);
//! assert_eq!(detections[0].doppler_bin, 5);
//! assert!(detections[0].snr_db > 10.0);
//! ```

use std::fmt;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// CFAR operating mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CfarMode {
    /// Cell-Averaging CFAR -- noise estimate is the mean of training-cell powers.
    CellAveraging,
    /// Ordered-Statistic CFAR -- noise estimate is the k-th smallest training-cell
    /// power, where `k = 3*N/4` and N is the number of training cells collected.
    OrderedStatistic,
}

/// Parameters that control CFAR detection and peak clustering.
#[derive(Debug, Clone)]
pub struct DetectionParams {
    /// Multiplicative threshold factor applied to the estimated noise floor.
    /// A CUT (Cell Under Test) is declared a detection when
    /// `power > threshold_factor * noise_estimate`.
    pub threshold_factor: f64,
    /// Number of guard cells on each side of the CUT (in each dimension).
    /// Guard cells are excluded from the noise estimate to avoid target leakage.
    pub guard_cells: usize,
    /// Number of training cells on each side of the CUT (in each dimension),
    /// located outside the guard band.
    pub training_cells: usize,
    /// Minimum Chebyshev (L-infinity) distance between retained peaks.  Closer
    /// peaks are merged by keeping the strongest.
    pub min_peak_separation: usize,
    /// Which CFAR variant to use.
    pub cfar_mode: CfarMode,
}

impl Default for DetectionParams {
    fn default() -> Self {
        Self {
            threshold_factor: 4.0,
            guard_cells: 2,
            training_cells: 4,
            min_peak_separation: 3,
            cfar_mode: CfarMode::CellAveraging,
        }
    }
}

/// A single target detection report.
#[derive(Debug, Clone, PartialEq)]
pub struct TargetDetection {
    /// Row index in the range-Doppler map (range axis).
    pub range_bin: usize,
    /// Column index in the range-Doppler map (Doppler / velocity axis).
    pub doppler_bin: usize,
    /// Absolute power of the CUT in decibels: `10 * log10(power)`.
    pub power_db: f64,
    /// Estimated signal-to-noise ratio in decibels:
    /// `10 * log10(power / noise_estimate)`.
    pub snr_db: f64,
}

impl fmt::Display for TargetDetection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "Target(range_bin={}, doppler_bin={}, power={:.1} dB, SNR={:.1} dB)",
            self.range_bin, self.doppler_bin, self.power_db, self.snr_db,
        )
    }
}

/// Summary statistics for a detection run.
#[derive(Debug, Clone)]
pub struct DetectionStatistics {
    /// Total number of detections after clustering.
    pub total_detections: usize,
    /// Estimated false-alarm probability based on `1 / threshold_factor`.
    /// This is a rough order-of-magnitude estimate for Rayleigh-distributed noise.
    pub estimated_pfa: f64,
    /// Number of raw (pre-clustering) detections.
    pub raw_detections: usize,
}

/// Range-Doppler target detector.
///
/// Wraps [`DetectionParams`] and exposes methods to run CFAR detection on a 2-D
/// range-Doppler map.
#[derive(Debug, Clone)]
pub struct RangeDopplerDetector {
    params: DetectionParams,
}

// ---------------------------------------------------------------------------
// Implementation
// ---------------------------------------------------------------------------

impl RangeDopplerDetector {
    /// Create a new detector with the given parameters.
    pub fn new(params: DetectionParams) -> Self {
        Self { params }
    }

    /// Create a detector with default parameters.
    pub fn with_defaults() -> Self {
        Self {
            params: DetectionParams::default(),
        }
    }

    /// Return a reference to the current detection parameters.
    pub fn params(&self) -> &DetectionParams {
        &self.params
    }

    // -- core detection -----------------------------------------------------

    /// Run CFAR detection on the 2-D `range_doppler_map`.
    ///
    /// Each element `range_doppler_map[r][d]` represents the *linear* power
    /// (magnitude squared) at range bin `r` and Doppler bin `d`.
    ///
    /// Returns a list of [`TargetDetection`] reports sorted by descending SNR.
    pub fn detect(&self, range_doppler_map: &[Vec<f64>]) -> Vec<TargetDetection> {
        let n_range = range_doppler_map.len();
        if n_range == 0 {
            return Vec::new();
        }
        let n_doppler = range_doppler_map[0].len();
        if n_doppler == 0 {
            return Vec::new();
        }

        // Phase 1: raw CFAR detections
        let mut raw: Vec<TargetDetection> = Vec::new();
        for r in 0..n_range {
            for d in 0..n_doppler {
                if let Some(det) = self.cfar_test(range_doppler_map, r, d, n_range, n_doppler) {
                    raw.push(det);
                }
            }
        }

        // Phase 2: cluster / merge adjacent detections
        let clustered = self.cluster_peaks(raw);

        // Sort descending by SNR
        let mut out = clustered;
        out.sort_by(|a, b| b.snr_db.partial_cmp(&a.snr_db).unwrap_or(std::cmp::Ordering::Equal));
        out
    }

    /// Run detection and also return [`DetectionStatistics`].
    pub fn detect_with_stats(
        &self,
        range_doppler_map: &[Vec<f64>],
    ) -> (Vec<TargetDetection>, DetectionStatistics) {
        let n_range = range_doppler_map.len();
        if n_range == 0 {
            return (
                Vec::new(),
                DetectionStatistics {
                    total_detections: 0,
                    estimated_pfa: 0.0,
                    raw_detections: 0,
                },
            );
        }
        let n_doppler = range_doppler_map[0].len();

        // Phase 1
        let mut raw: Vec<TargetDetection> = Vec::new();
        for r in 0..n_range {
            for d in 0..n_doppler {
                if let Some(det) = self.cfar_test(range_doppler_map, r, d, n_range, n_doppler) {
                    raw.push(det);
                }
            }
        }
        let raw_count = raw.len();

        // Phase 2
        let mut clustered = self.cluster_peaks(raw);
        clustered
            .sort_by(|a, b| b.snr_db.partial_cmp(&a.snr_db).unwrap_or(std::cmp::Ordering::Equal));

        let total = clustered.len();
        // Rough Pfa estimate for exponential (Rayleigh-envelope) noise:
        //   Pfa ~ threshold_factor^{-N}, simplified for display.
        let pfa = if self.params.threshold_factor > 0.0 {
            1.0 / self.params.threshold_factor.powi(self.params.training_cells as i32)
        } else {
            1.0
        };

        let stats = DetectionStatistics {
            total_detections: total,
            estimated_pfa: pfa,
            raw_detections: raw_count,
        };
        (clustered, stats)
    }

    // -- conversion helpers -------------------------------------------------

    /// Convert a range bin index to an absolute range value.
    ///
    /// `range_resolution` is the distance (in metres) per bin,
    /// typically `c / (2 * bandwidth)`.
    pub fn range_from_bin(range_bin: usize, range_resolution: f64) -> f64 {
        range_bin as f64 * range_resolution
    }

    /// Convert a Doppler bin index to a radial velocity.
    ///
    /// `velocity_resolution` is the velocity (in m/s) per bin,
    /// typically `lambda / (2 * T_cpi)`.
    ///
    /// The Doppler axis is assumed centred: negative bins represent closing
    /// targets, positive bins receding targets.  `n_doppler` is the total
    /// number of Doppler bins.
    pub fn velocity_from_bin(
        doppler_bin: usize,
        n_doppler: usize,
        velocity_resolution: f64,
    ) -> f64 {
        let half = n_doppler as f64 / 2.0;
        (doppler_bin as f64 - half) * velocity_resolution
    }

    // -- internal -----------------------------------------------------------

    /// Perform CFAR test on a single Cell Under Test (CUT).
    fn cfar_test(
        &self,
        map: &[Vec<f64>],
        r: usize,
        d: usize,
        n_range: usize,
        n_doppler: usize,
    ) -> Option<TargetDetection> {
        let guard = self.params.guard_cells;
        let train = self.params.training_cells;
        let window = guard + train;

        let cut_power = map[r][d];
        if cut_power <= 0.0 {
            return None;
        }

        // Collect training-cell powers (skip guard cells around the CUT).
        let mut training_powers: Vec<f64> = Vec::new();

        let r_lo = if r >= window { r - window } else { 0 };
        let r_hi = if r + window < n_range {
            r + window
        } else {
            n_range - 1
        };
        let d_lo = if d >= window { d - window } else { 0 };
        let d_hi = if d + window < n_doppler {
            d + window
        } else {
            n_doppler - 1
        };

        for ri in r_lo..=r_hi {
            for di in d_lo..=d_hi {
                let dr = if ri >= r { ri - r } else { r - ri };
                let dd = if di >= d { di - d } else { d - di };
                // Skip the CUT itself and its guard band
                if dr <= guard && dd <= guard {
                    continue;
                }
                let v = map[ri][di];
                if v > 0.0 {
                    training_powers.push(v);
                }
            }
        }

        if training_powers.is_empty() {
            return None;
        }

        let noise_est = match self.params.cfar_mode {
            CfarMode::CellAveraging => {
                let sum: f64 = training_powers.iter().sum();
                sum / training_powers.len() as f64
            }
            CfarMode::OrderedStatistic => {
                training_powers.sort_by(|a, b| {
                    a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal)
                });
                let k = (3 * training_powers.len()) / 4;
                let k = k.min(training_powers.len() - 1);
                training_powers[k]
            }
        };

        let threshold = self.params.threshold_factor * noise_est;
        if cut_power > threshold && noise_est > 0.0 {
            let snr_linear = cut_power / noise_est;
            Some(TargetDetection {
                range_bin: r,
                doppler_bin: d,
                power_db: 10.0 * cut_power.log10(),
                snr_db: 10.0 * snr_linear.log10(),
            })
        } else {
            None
        }
    }

    /// Merge raw detections that are within `min_peak_separation` of each other,
    /// keeping the one with the highest power.
    fn cluster_peaks(&self, mut raw: Vec<TargetDetection>) -> Vec<TargetDetection> {
        if raw.is_empty() {
            return raw;
        }
        // Sort descending by power so we greedily keep the strongest first.
        raw.sort_by(|a, b| b.power_db.partial_cmp(&a.power_db).unwrap_or(std::cmp::Ordering::Equal));

        let sep = self.params.min_peak_separation;
        let mut kept: Vec<TargetDetection> = Vec::new();

        for det in raw {
            let dominated = kept.iter().any(|k| {
                let dr = if det.range_bin >= k.range_bin {
                    det.range_bin - k.range_bin
                } else {
                    k.range_bin - det.range_bin
                };
                let dd = if det.doppler_bin >= k.doppler_bin {
                    det.doppler_bin - k.doppler_bin
                } else {
                    k.doppler_bin - det.doppler_bin
                };
                // Chebyshev (L-inf) distance
                dr.max(dd) < sep
            });
            if !dominated {
                kept.push(det);
            }
        }
        kept
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: create a uniform-noise RDM of given size.
    fn uniform_map(n_range: usize, n_doppler: usize, floor: f64) -> Vec<Vec<f64>> {
        vec![vec![floor; n_doppler]; n_range]
    }

    /// Helper: inject a target into the map.
    fn inject(map: &mut [Vec<f64>], r: usize, d: usize, power: f64) {
        map[r][d] = power;
    }

    #[test]
    fn test_single_target_ca_cfar() {
        let mut rdm = uniform_map(32, 32, 1.0);
        inject(&mut rdm, 16, 16, 200.0);

        let det = RangeDopplerDetector::new(DetectionParams {
            threshold_factor: 3.0,
            guard_cells: 1,
            training_cells: 2,
            min_peak_separation: 3,
            cfar_mode: CfarMode::CellAveraging,
        });
        let results = det.detect(&rdm);

        assert_eq!(results.len(), 1, "expected exactly one detection");
        assert_eq!(results[0].range_bin, 16);
        assert_eq!(results[0].doppler_bin, 16);
        assert!(results[0].snr_db > 15.0);
    }

    #[test]
    fn test_single_target_os_cfar() {
        let mut rdm = uniform_map(32, 32, 1.0);
        inject(&mut rdm, 10, 20, 150.0);

        let det = RangeDopplerDetector::new(DetectionParams {
            threshold_factor: 3.0,
            guard_cells: 1,
            training_cells: 2,
            min_peak_separation: 3,
            cfar_mode: CfarMode::OrderedStatistic,
        });
        let results = det.detect(&rdm);

        assert!(!results.is_empty(), "OS-CFAR should detect the target");
        assert_eq!(results[0].range_bin, 10);
        assert_eq!(results[0].doppler_bin, 20);
    }

    #[test]
    fn test_no_targets_below_threshold() {
        // All cells at the same power -- nothing should be detected.
        let rdm = uniform_map(16, 16, 5.0);
        let det = RangeDopplerDetector::with_defaults();
        let results = det.detect(&rdm);
        assert!(results.is_empty(), "uniform map should yield zero detections");
    }

    #[test]
    fn test_empty_map() {
        let rdm: Vec<Vec<f64>> = Vec::new();
        let det = RangeDopplerDetector::with_defaults();
        assert!(det.detect(&rdm).is_empty());
    }

    #[test]
    fn test_empty_doppler_dimension() {
        let rdm = vec![vec![]; 4];
        let det = RangeDopplerDetector::with_defaults();
        assert!(det.detect(&rdm).is_empty());
    }

    #[test]
    fn test_two_well_separated_targets() {
        let mut rdm = uniform_map(64, 64, 1.0);
        inject(&mut rdm, 10, 10, 300.0);
        inject(&mut rdm, 50, 50, 250.0);

        let det = RangeDopplerDetector::new(DetectionParams {
            threshold_factor: 3.0,
            guard_cells: 1,
            training_cells: 2,
            min_peak_separation: 5,
            cfar_mode: CfarMode::CellAveraging,
        });
        let results = det.detect(&rdm);
        assert_eq!(results.len(), 2, "should detect both separated targets");
    }

    #[test]
    fn test_clustering_merges_adjacent() {
        let mut rdm = uniform_map(32, 32, 1.0);
        // Two very close targets -- clustering should merge them.
        inject(&mut rdm, 15, 15, 300.0);
        inject(&mut rdm, 16, 16, 280.0);

        let det = RangeDopplerDetector::new(DetectionParams {
            threshold_factor: 3.0,
            guard_cells: 1,
            training_cells: 2,
            min_peak_separation: 3, // merge anything within 3 bins
            cfar_mode: CfarMode::CellAveraging,
        });
        let results = det.detect(&rdm);
        assert_eq!(
            results.len(),
            1,
            "adjacent targets should be clustered into one"
        );
        // The strongest one should survive.
        assert_eq!(results[0].range_bin, 15);
        assert_eq!(results[0].doppler_bin, 15);
    }

    #[test]
    fn test_detection_statistics() {
        let mut rdm = uniform_map(32, 32, 1.0);
        inject(&mut rdm, 10, 10, 200.0);
        inject(&mut rdm, 25, 25, 180.0);

        let det = RangeDopplerDetector::new(DetectionParams {
            threshold_factor: 3.0,
            guard_cells: 1,
            training_cells: 2,
            min_peak_separation: 3,
            cfar_mode: CfarMode::CellAveraging,
        });
        let (results, stats) = det.detect_with_stats(&rdm);
        assert_eq!(stats.total_detections, results.len());
        assert!(stats.raw_detections >= stats.total_detections);
        assert!(stats.estimated_pfa > 0.0);
        assert!(stats.estimated_pfa < 1.0);
    }

    #[test]
    fn test_range_conversion() {
        let range = RangeDopplerDetector::range_from_bin(10, 15.0);
        assert!((range - 150.0).abs() < 1e-9);
    }

    #[test]
    fn test_velocity_conversion() {
        // 64 Doppler bins, bin 32 (centre) should be 0 m/s.
        let v = RangeDopplerDetector::velocity_from_bin(32, 64, 2.0);
        assert!(v.abs() < 1e-9, "centre bin should map to ~0 m/s");

        // Bin 48 is 16 bins above centre -> +32 m/s.
        let v2 = RangeDopplerDetector::velocity_from_bin(48, 64, 2.0);
        assert!((v2 - 32.0).abs() < 1e-9);
    }

    #[test]
    fn test_power_db_positive() {
        let mut rdm = uniform_map(16, 16, 1.0);
        inject(&mut rdm, 8, 8, 500.0);

        let det = RangeDopplerDetector::new(DetectionParams {
            threshold_factor: 2.0,
            guard_cells: 1,
            training_cells: 2,
            min_peak_separation: 2,
            cfar_mode: CfarMode::CellAveraging,
        });
        let results = det.detect(&rdm);
        assert!(!results.is_empty());
        // 10*log10(500) ~ 26.99 dB
        assert!(
            (results[0].power_db - 26.9897).abs() < 0.01,
            "power_db = {} expected ~27 dB",
            results[0].power_db
        );
    }

    #[test]
    fn test_snr_ordering() {
        let mut rdm = uniform_map(64, 64, 1.0);
        inject(&mut rdm, 10, 10, 400.0);
        inject(&mut rdm, 50, 50, 100.0);

        let det = RangeDopplerDetector::new(DetectionParams {
            threshold_factor: 3.0,
            guard_cells: 1,
            training_cells: 2,
            min_peak_separation: 5,
            cfar_mode: CfarMode::CellAveraging,
        });
        let results = det.detect(&rdm);
        assert!(results.len() >= 2);
        assert!(
            results[0].snr_db >= results[1].snr_db,
            "results should be sorted descending by SNR"
        );
    }

    #[test]
    fn test_target_detection_display() {
        let td = TargetDetection {
            range_bin: 5,
            doppler_bin: 12,
            power_db: 23.5,
            snr_db: 15.2,
        };
        let s = format!("{}", td);
        assert!(s.contains("range_bin=5"));
        assert!(s.contains("doppler_bin=12"));
    }

    #[test]
    fn test_default_params() {
        let p = DetectionParams::default();
        assert_eq!(p.guard_cells, 2);
        assert_eq!(p.training_cells, 4);
        assert_eq!(p.threshold_factor, 4.0);
        assert_eq!(p.min_peak_separation, 3);
        assert_eq!(p.cfar_mode, CfarMode::CellAveraging);
    }
}
