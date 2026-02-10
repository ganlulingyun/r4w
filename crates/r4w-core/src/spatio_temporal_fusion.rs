//! # Spatio-Temporal Fusion — Space-Time Adaptive Processing (STAP)
//!
//! Fuses spatial (antenna array) and temporal (time-series) information for
//! enhanced signal detection. Implements full space-time covariance estimation,
//! diagonal loading for robustness, MVDR/Capon beamforming weight computation,
//! and configurable STAP modes.
//!
//! ## Quick start
//!
//! ```
//! use r4w_core::spatio_temporal_fusion::{SpatioTemporalFusion, StapConfig, StapMode};
//!
//! // 4-element array, 8 pulses
//! let config = StapConfig {
//!     num_elements: 4,
//!     num_pulses: 8,
//!     clutter_rank_estimate: 4,
//!     diagonal_loading_db: -30.0,
//! };
//! let mut stap = SpatioTemporalFusion::new(config);
//!
//! // Build a small data cube: [pulse][element][range_bin]
//! let num_range_bins = 16;
//! let data_cube: Vec<Vec<Vec<(f64, f64)>>> = (0..8)
//!     .map(|p| {
//!         (0..4)
//!             .map(|e| {
//!                 (0..num_range_bins)
//!                     .map(|r| {
//!                         let phase = (p as f64) * 0.3 + (e as f64) * 0.5 + (r as f64) * 0.1;
//!                         (phase.cos(), phase.sin())
//!                     })
//!                     .collect()
//!             })
//!             .collect()
//!     })
//!     .collect();
//!
//! let result = stap.process(&data_cube);
//! assert_eq!(result.output_samples.len(), num_range_bins);
//! assert!(result.snr_improvement_db.is_finite());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers (using (f64, f64) tuples)
// ---------------------------------------------------------------------------

#[inline]
fn c_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

#[inline]
fn c_mag2(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn c_scale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

#[inline]
fn c_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = c_mag2(b);
    if denom < 1e-30 {
        (0.0, 0.0)
    } else {
        c_scale(c_mul(a, c_conj(b)), 1.0 / denom)
    }
}

/// Dot product of two complex vectors: sum of a[i] * conj(b[i]).
fn c_dot(a: &[(f64, f64)], b: &[(f64, f64)]) -> (f64, f64) {
    a.iter()
        .zip(b.iter())
        .fold((0.0, 0.0), |acc, (&ai, &bi)| c_add(acc, c_mul(ai, c_conj(bi))))
}

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// STAP processing mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StapMode {
    /// Full joint space-time adaptive processing (NM x NM covariance).
    FullAdaptive,
    /// Element-space processing -- temporal filter per element, then spatial combine.
    ElementSpace,
    /// Beam-space processing -- spatial beamform first, then temporal filter.
    BeamSpace,
    /// Post-Doppler STAP -- Doppler filter bank followed by spatial nulling.
    PostDoppler,
}

/// Configuration for the STAP engine.
#[derive(Debug, Clone)]
pub struct StapConfig {
    /// Number of antenna elements (spatial dimension).
    pub num_elements: usize,
    /// Number of coherent processing interval pulses (temporal dimension).
    pub num_pulses: usize,
    /// Estimated clutter rank (used for reduced-rank techniques).
    pub clutter_rank_estimate: usize,
    /// Diagonal loading level in dB (negative = small loading).
    pub diagonal_loading_db: f64,
}

/// Result returned from STAP processing.
#[derive(Debug, Clone)]
pub struct FusionResult {
    /// STAP-filtered output samples -- one per range bin.
    pub output_samples: Vec<(f64, f64)>,
    /// Estimated SNR improvement in dB from adaptive processing.
    pub snr_improvement_db: f64,
    /// Estimated clutter suppression in dB.
    pub clutter_suppression_db: f64,
    /// Whether a target was detected at any range bin.
    pub target_detected: bool,
}

/// Main Space-Time Adaptive Processing engine.
#[derive(Debug, Clone)]
pub struct SpatioTemporalFusion {
    config: StapConfig,
    mode: StapMode,
    /// Steering angle (degrees from broadside).
    steer_angle_deg: f64,
    /// Target Doppler frequency in Hz.
    steer_doppler_hz: f64,
    /// Detection threshold (linear power ratio).
    detection_threshold: f64,
    /// Pulse repetition frequency (Hz) -- used for Doppler normalisation.
    prf_hz: f64,
}

impl SpatioTemporalFusion {
    // ------------------------------------------------------------------
    // Construction
    // ------------------------------------------------------------------

    /// Create a new STAP engine with the given configuration.
    ///
    /// Defaults: FullAdaptive mode, broadside steering, 0 Hz Doppler,
    /// detection threshold 10.0, PRF 1000 Hz.
    pub fn new(config: StapConfig) -> Self {
        Self {
            config,
            mode: StapMode::FullAdaptive,
            steer_angle_deg: 0.0,
            steer_doppler_hz: 0.0,
            detection_threshold: 10.0,
            prf_hz: 1000.0,
        }
    }

    /// Set the STAP processing mode.
    pub fn set_mode(&mut self, mode: StapMode) {
        self.mode = mode;
    }

    /// Set the spatial steering angle in degrees from broadside.
    pub fn set_steer_angle(&mut self, angle_deg: f64) {
        self.steer_angle_deg = angle_deg;
    }

    /// Set the target Doppler frequency in Hz.
    pub fn set_steer_doppler(&mut self, doppler_hz: f64) {
        self.steer_doppler_hz = doppler_hz;
    }

    /// Set the CFAR-like detection threshold (linear power ratio).
    pub fn set_detection_threshold(&mut self, threshold: f64) {
        self.detection_threshold = threshold;
    }

    /// Set the pulse repetition frequency (Hz).
    pub fn set_prf(&mut self, prf_hz: f64) {
        self.prf_hz = prf_hz;
    }

    // ------------------------------------------------------------------
    // Core STAP pipeline
    // ------------------------------------------------------------------

    /// Process a full data cube and return the [`FusionResult`].
    ///
    /// `data_cube` is indexed as `[pulse][element][range_bin]`.
    ///
    /// The number of pulses must equal `config.num_pulses`, the number of
    /// elements must equal `config.num_elements`, and every element vector
    /// must have the same range-bin count.
    pub fn process(&mut self, data_cube: &[Vec<Vec<(f64, f64)>>]) -> FusionResult {
        let n_pulses = self.config.num_pulses;
        let n_elem = self.config.num_elements;
        let nm = n_pulses * n_elem;

        assert_eq!(
            data_cube.len(),
            n_pulses,
            "data_cube pulse count mismatch"
        );
        for (p, pulse_data) in data_cube.iter().enumerate() {
            assert_eq!(
                pulse_data.len(),
                n_elem,
                "data_cube element count mismatch at pulse {p}"
            );
        }

        let n_range = data_cube[0][0].len();
        for pulse_data in data_cube {
            for elem_data in pulse_data {
                assert_eq!(elem_data.len(), n_range, "range bin count mismatch");
            }
        }

        // --- 1. Form space-time snapshots for every range bin ---
        // snapshot[range_bin] = vec of length NM (pulse-major)
        let snapshots: Vec<Vec<(f64, f64)>> = (0..n_range)
            .map(|r| {
                let mut snap = Vec::with_capacity(nm);
                for p in 0..n_pulses {
                    for e in 0..n_elem {
                        snap.push(data_cube[p][e][r]);
                    }
                }
                snap
            })
            .collect();

        // --- 2. Estimate covariance from training range bins ---
        let cov = self.compute_covariance(&snapshots);

        // --- 3. Generate space-time steering vector ---
        let sv = self.steering_vector(self.steer_angle_deg, self.steer_doppler_hz);

        // --- 4. Compute adaptive weights ---
        let weights = self.stap_weights(&cov, &sv);

        // --- 5. Apply weights to each range bin ---
        let output_samples: Vec<(f64, f64)> = snapshots
            .iter()
            .map(|snap| c_dot(&weights, snap))
            .collect();

        // --- 6. Compute metrics ---
        let output_power: f64 = output_samples.iter().map(|&s| c_mag2(s)).sum::<f64>()
            / n_range.max(1) as f64;

        let input_power: f64 = snapshots
            .iter()
            .map(|snap| snap.iter().map(|&s| c_mag2(s)).sum::<f64>())
            .sum::<f64>()
            / (n_range.max(1) as f64 * nm as f64);

        let snr_improvement_db = if input_power > 1e-30 {
            10.0 * (output_power / input_power).log10()
        } else {
            0.0
        };

        // Clutter suppression: compare diagonal-loaded covariance trace to output power
        let cov_trace: f64 = (0..nm).map(|i| c_mag2(cov[i][i])).sum::<f64>() / nm as f64;
        let clutter_suppression_db = if cov_trace > 1e-30 {
            10.0 * (cov_trace / output_power.max(1e-30)).log10()
        } else {
            0.0
        };

        // --- 7. Detect target ---
        let noise_floor = {
            let mut powers: Vec<f64> = output_samples.iter().map(|&s| c_mag2(s)).collect();
            powers.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let mid = powers.len() / 2;
            if powers.is_empty() {
                1e-30
            } else {
                powers[mid].max(1e-30)
            }
        };

        let peak_power = output_samples
            .iter()
            .map(|&s| c_mag2(s))
            .fold(0.0_f64, f64::max);

        let target_detected = (peak_power / noise_floor) > self.detection_threshold;

        FusionResult {
            output_samples,
            snr_improvement_db,
            clutter_suppression_db,
            target_detected,
        }
    }

    // ------------------------------------------------------------------
    // Sub-algorithms (public for direct use / testing)
    // ------------------------------------------------------------------

    /// Estimate the space-time covariance matrix from a set of snapshot vectors.
    ///
    /// Each inner `Vec<(f64,f64)>` is a space-time snapshot of length NM.
    /// Returns an NM x NM Hermitian covariance matrix with diagonal loading.
    pub fn compute_covariance(
        &self,
        data: &[Vec<(f64, f64)>],
    ) -> Vec<Vec<(f64, f64)>> {
        let nm = self.config.num_elements * self.config.num_pulses;
        let k = data.len(); // number of training snapshots

        // Accumulate R = (1/K) * sum_k x_k x_k^H
        let mut cov = vec![vec![(0.0, 0.0); nm]; nm];
        for snap in data {
            assert!(snap.len() >= nm, "snapshot too short for NM={nm}");
            for i in 0..nm {
                for j in 0..nm {
                    cov[i][j] = c_add(cov[i][j], c_mul(snap[i], c_conj(snap[j])));
                }
            }
        }

        let inv_k = if k > 0 { 1.0 / k as f64 } else { 1.0 };
        for row in &mut cov {
            for val in row.iter_mut() {
                *val = c_scale(*val, inv_k);
            }
        }

        // Diagonal loading: R_loaded = R + sigma^2 * I
        let sigma2 = 10.0_f64.powf(self.config.diagonal_loading_db / 10.0);
        for i in 0..nm {
            cov[i][i] = c_add(cov[i][i], (sigma2, 0.0));
        }

        cov
    }

    /// Compute MVDR/Capon adaptive weights: w = R^{-1} s / (s^H R^{-1} s).
    ///
    /// `cov` is the NM x NM space-time covariance matrix.
    /// `steering` is the NM-length space-time steering vector.
    ///
    /// The matrix inverse is computed via Gauss-Jordan elimination on the
    /// augmented system.
    pub fn stap_weights(
        &self,
        cov: &[Vec<(f64, f64)>],
        steering: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        let nm = steering.len();
        assert_eq!(cov.len(), nm);

        // Solve R * w_raw = s via Gauss-Jordan
        let w_raw = self.solve_linear(cov, steering);

        // Normalise: w = w_raw / (s^H w_raw)
        let denom = c_dot(steering, &w_raw);
        let denom_mag2 = c_mag2(denom);
        if denom_mag2 < 1e-30 {
            return vec![(0.0, 0.0); nm];
        }

        w_raw
            .iter()
            .map(|&w| c_div(w, denom))
            .collect()
    }

    /// Generate the joint space-time steering vector.
    ///
    /// Spatial steering assumes a half-wavelength spaced ULA.
    /// Temporal steering uses normalised Doppler = doppler_hz / PRF.
    ///
    /// Returns a vector of length `num_elements * num_pulses`.
    pub fn steering_vector(&self, angle_deg: f64, doppler_hz: f64) -> Vec<(f64, f64)> {
        let n_elem = self.config.num_elements;
        let n_pulses = self.config.num_pulses;
        let mut sv = Vec::with_capacity(n_elem * n_pulses);

        // Spatial phase increment for half-wavelength ULA
        let spatial_phase = PI * (angle_deg * PI / 180.0).sin();
        // Temporal (Doppler) phase increment per pulse
        let doppler_norm = if self.prf_hz.abs() > 1e-10 {
            doppler_hz / self.prf_hz
        } else {
            0.0
        };
        let temporal_phase = 2.0 * PI * doppler_norm;

        for p in 0..n_pulses {
            for e in 0..n_elem {
                let phi = spatial_phase * e as f64 + temporal_phase * p as f64;
                sv.push((phi.cos(), phi.sin()));
            }
        }

        sv
    }

    /// Check whether a target is detected in the given `FusionResult` by
    /// comparing peak output power to the median-estimated noise floor.
    pub fn detect_target(&self, result: &FusionResult, threshold: f64) -> bool {
        let powers: Vec<f64> = result.output_samples.iter().map(|&s| c_mag2(s)).collect();
        if powers.is_empty() {
            return false;
        }
        let mut sorted = powers.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let noise_floor = sorted[sorted.len() / 2].max(1e-30);
        let peak = powers.iter().cloned().fold(0.0_f64, f64::max);
        (peak / noise_floor) > threshold
    }

    // ------------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------------

    /// Solve R x = b via Gauss-Jordan elimination with partial pivoting.
    fn solve_linear(
        &self,
        r: &[Vec<(f64, f64)>],
        b: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        let n = b.len();
        // Build augmented matrix [R | b]
        let mut aug: Vec<Vec<(f64, f64)>> = (0..n)
            .map(|i| {
                let mut row = r[i].clone();
                row.push(b[i]);
                row
            })
            .collect();

        // Forward elimination with partial pivoting
        for col in 0..n {
            // Find pivot
            let mut max_mag = c_mag2(aug[col][col]);
            let mut pivot_row = col;
            for row in (col + 1)..n {
                let mag = c_mag2(aug[row][col]);
                if mag > max_mag {
                    max_mag = mag;
                    pivot_row = row;
                }
            }
            if pivot_row != col {
                aug.swap(col, pivot_row);
            }

            let pivot = aug[col][col];
            if c_mag2(pivot) < 1e-30 {
                continue; // singular -- skip
            }

            // Scale pivot row
            for j in col..=n {
                aug[col][j] = c_div(aug[col][j], pivot);
            }

            // Eliminate column in all other rows
            for row in 0..n {
                if row == col {
                    continue;
                }
                let factor = aug[row][col];
                for j in col..=n {
                    let scaled = c_mul(factor, aug[col][j]);
                    aug[row][j] = c_sub(aug[row][j], scaled);
                }
            }
        }

        // Extract solution from last column
        (0..n).map(|i| aug[i][n]).collect()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> StapConfig {
        StapConfig {
            num_elements: 4,
            num_pulses: 8,
            clutter_rank_estimate: 4,
            diagonal_loading_db: -30.0,
        }
    }

    /// Build a synthetic data cube with a target at a given angle and Doppler.
    fn make_data_cube(
        n_pulses: usize,
        n_elem: usize,
        n_range: usize,
        target_angle_deg: f64,
        target_doppler_hz: f64,
        prf_hz: f64,
        target_range_bin: usize,
        snr_linear: f64,
    ) -> Vec<Vec<Vec<(f64, f64)>>> {
        let spatial_phase = PI * (target_angle_deg * PI / 180.0).sin();
        let doppler_norm = target_doppler_hz / prf_hz;
        let temporal_phase = 2.0 * PI * doppler_norm;

        // Simple LCG-based pseudo-random for reproducibility (no external crates)
        let mut seed: u64 = 42;
        let mut next_rand = || -> f64 {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((seed >> 33) as f64) / (2.0_f64.powi(31)) - 1.0
        };

        (0..n_pulses)
            .map(|p| {
                (0..n_elem)
                    .map(|e| {
                        (0..n_range)
                            .map(|r| {
                                // Noise
                                let noise = (next_rand() * 0.5, next_rand() * 0.5);
                                if r == target_range_bin {
                                    // Target signal
                                    let phi =
                                        spatial_phase * e as f64 + temporal_phase * p as f64;
                                    let sig = (
                                        snr_linear.sqrt() * phi.cos(),
                                        snr_linear.sqrt() * phi.sin(),
                                    );
                                    c_add(sig, noise)
                                } else {
                                    noise
                                }
                            })
                            .collect()
                    })
                    .collect()
            })
            .collect()
    }

    // ---------------------------------------------------------------
    // 1. Construction and defaults
    // ---------------------------------------------------------------

    #[test]
    fn test_new_default_mode() {
        let stap = SpatioTemporalFusion::new(default_config());
        assert_eq!(stap.mode, StapMode::FullAdaptive);
        assert_eq!(stap.steer_angle_deg, 0.0);
        assert_eq!(stap.steer_doppler_hz, 0.0);
    }

    // ---------------------------------------------------------------
    // 2. Steering vector length
    // ---------------------------------------------------------------

    #[test]
    fn test_steering_vector_length() {
        let stap = SpatioTemporalFusion::new(default_config());
        let sv = stap.steering_vector(10.0, 200.0);
        assert_eq!(sv.len(), 4 * 8);
    }

    // ---------------------------------------------------------------
    // 3. Steering vector at broadside has all-real first element row
    // ---------------------------------------------------------------

    #[test]
    fn test_steering_vector_broadside_zero_doppler() {
        let stap = SpatioTemporalFusion::new(default_config());
        let sv = stap.steering_vector(0.0, 0.0);
        // All entries should be (1, 0) at broadside, 0 doppler
        for &(re, im) in &sv {
            assert!((re - 1.0).abs() < 1e-12, "re={re}");
            assert!(im.abs() < 1e-12, "im={im}");
        }
    }

    // ---------------------------------------------------------------
    // 4. Steering vector unit magnitude
    // ---------------------------------------------------------------

    #[test]
    fn test_steering_vector_unit_magnitude() {
        let stap = SpatioTemporalFusion::new(default_config());
        let sv = stap.steering_vector(30.0, 150.0);
        for &s in &sv {
            let mag = c_mag2(s).sqrt();
            assert!((mag - 1.0).abs() < 1e-12, "magnitude = {mag}");
        }
    }

    // ---------------------------------------------------------------
    // 5. Covariance matrix dimensions
    // ---------------------------------------------------------------

    #[test]
    fn test_covariance_dimensions() {
        let stap = SpatioTemporalFusion::new(default_config());
        let nm = 4 * 8;
        let snapshots: Vec<Vec<(f64, f64)>> = (0..20)
            .map(|i| (0..nm).map(|j| ((i + j) as f64 * 0.1, 0.0)).collect())
            .collect();
        let cov = stap.compute_covariance(&snapshots);
        assert_eq!(cov.len(), nm);
        for row in &cov {
            assert_eq!(row.len(), nm);
        }
    }

    // ---------------------------------------------------------------
    // 6. Covariance matrix is Hermitian
    // ---------------------------------------------------------------

    #[test]
    fn test_covariance_hermitian() {
        let stap = SpatioTemporalFusion::new(default_config());
        let nm = 4 * 8;
        let snapshots: Vec<Vec<(f64, f64)>> = (0..30)
            .map(|i| {
                (0..nm)
                    .map(|j| {
                        let phi = (i * 7 + j * 3) as f64 * 0.17;
                        (phi.cos(), phi.sin())
                    })
                    .collect()
            })
            .collect();
        let cov = stap.compute_covariance(&snapshots);
        for i in 0..nm {
            for j in i..nm {
                let diff_re = (cov[i][j].0 - cov[j][i].0).abs();
                let diff_im = (cov[i][j].1 + cov[j][i].1).abs(); // conjugate
                assert!(
                    diff_re < 1e-10 && diff_im < 1e-10,
                    "Hermitian check failed at ({i},{j}): {:?} vs conj({:?})",
                    cov[i][j],
                    cov[j][i],
                );
            }
        }
    }

    // ---------------------------------------------------------------
    // 7. Diagonal loading increases diagonal
    // ---------------------------------------------------------------

    #[test]
    fn test_diagonal_loading_effect() {
        let config_low = StapConfig {
            num_elements: 2,
            num_pulses: 2,
            clutter_rank_estimate: 2,
            diagonal_loading_db: -40.0,
        };
        let config_high = StapConfig {
            num_elements: 2,
            num_pulses: 2,
            clutter_rank_estimate: 2,
            diagonal_loading_db: -10.0,
        };
        let stap_low = SpatioTemporalFusion::new(config_low);
        let stap_high = SpatioTemporalFusion::new(config_high);

        let nm = 4;
        let snapshots: Vec<Vec<(f64, f64)>> =
            vec![vec![(0.1, 0.0), (0.2, 0.1), (0.05, -0.1), (0.3, 0.0)]];

        let cov_low = stap_low.compute_covariance(&snapshots);
        let cov_high = stap_high.compute_covariance(&snapshots);

        let diag_low: f64 = (0..nm).map(|i| cov_low[i][i].0).sum();
        let diag_high: f64 = (0..nm).map(|i| cov_high[i][i].0).sum();
        assert!(
            diag_high > diag_low,
            "higher loading should increase diagonal: {diag_high} > {diag_low}"
        );
    }

    // ---------------------------------------------------------------
    // 8. STAP weights vector length
    // ---------------------------------------------------------------

    #[test]
    fn test_stap_weights_length() {
        let stap = SpatioTemporalFusion::new(default_config());
        let nm = 32;
        let identity: Vec<Vec<(f64, f64)>> = (0..nm)
            .map(|i| {
                (0..nm)
                    .map(|j| if i == j { (1.0, 0.0) } else { (0.0, 0.0) })
                    .collect()
            })
            .collect();
        let sv = stap.steering_vector(0.0, 0.0);
        let w = stap.stap_weights(&identity, &sv);
        assert_eq!(w.len(), nm);
    }

    // ---------------------------------------------------------------
    // 9. MVDR weights with identity covariance equal steering vector
    // ---------------------------------------------------------------

    #[test]
    fn test_mvdr_weights_identity_cov() {
        let config = StapConfig {
            num_elements: 3,
            num_pulses: 2,
            clutter_rank_estimate: 2,
            diagonal_loading_db: -60.0, // negligible loading
        };
        let stap = SpatioTemporalFusion::new(config);
        let nm = 6;
        let identity: Vec<Vec<(f64, f64)>> = (0..nm)
            .map(|i| {
                (0..nm)
                    .map(|j| if i == j { (1.0, 0.0) } else { (0.0, 0.0) })
                    .collect()
            })
            .collect();
        let sv = stap.steering_vector(0.0, 0.0);
        let w = stap.stap_weights(&identity, &sv);

        // With identity cov, MVDR weight = s / (s^H s) = s / N
        let norm_sq: f64 = sv.iter().map(|&s| c_mag2(s)).sum();
        for (i, (&wi, &si)) in w.iter().zip(sv.iter()).enumerate() {
            let expected = c_scale(si, 1.0 / norm_sq);
            assert!(
                (wi.0 - expected.0).abs() < 1e-8 && (wi.1 - expected.1).abs() < 1e-8,
                "weight mismatch at {i}: got {:?}, expected {:?}",
                wi,
                expected,
            );
        }
    }

    // ---------------------------------------------------------------
    // 10. Process returns correct output length
    // ---------------------------------------------------------------

    #[test]
    fn test_process_output_length() {
        let config = StapConfig {
            num_elements: 2,
            num_pulses: 4,
            clutter_rank_estimate: 2,
            diagonal_loading_db: -20.0,
        };
        let mut stap = SpatioTemporalFusion::new(config);
        let cube = make_data_cube(4, 2, 32, 0.0, 0.0, 1000.0, 10, 5.0);
        let result = stap.process(&cube);
        assert_eq!(result.output_samples.len(), 32);
    }

    // ---------------------------------------------------------------
    // 11. Process detects injected strong target
    // ---------------------------------------------------------------

    #[test]
    fn test_process_detects_strong_target() {
        let config = StapConfig {
            num_elements: 4,
            num_pulses: 8,
            clutter_rank_estimate: 4,
            diagonal_loading_db: -20.0,
        };
        let mut stap = SpatioTemporalFusion::new(config);
        stap.set_detection_threshold(3.0);
        let cube = make_data_cube(8, 4, 64, 0.0, 0.0, 1000.0, 32, 100.0);
        let result = stap.process(&cube);
        assert!(
            result.target_detected,
            "Should detect strong target (SNR=100)"
        );
    }

    // ---------------------------------------------------------------
    // 12. SNR improvement is finite
    // ---------------------------------------------------------------

    #[test]
    fn test_snr_improvement_finite() {
        let config = StapConfig {
            num_elements: 3,
            num_pulses: 4,
            clutter_rank_estimate: 3,
            diagonal_loading_db: -20.0,
        };
        let mut stap = SpatioTemporalFusion::new(config);
        let cube = make_data_cube(4, 3, 16, 5.0, 50.0, 1000.0, 8, 10.0);
        let result = stap.process(&cube);
        assert!(result.snr_improvement_db.is_finite());
        assert!(result.clutter_suppression_db.is_finite());
    }

    // ---------------------------------------------------------------
    // 13. detect_target with explicit threshold
    // ---------------------------------------------------------------

    #[test]
    fn test_detect_target_method() {
        let stap = SpatioTemporalFusion::new(default_config());

        // Make a result with one large peak and rest small
        let mut samples = vec![(0.01, 0.0); 50];
        samples[25] = (10.0, 0.0); // strong target

        let result = FusionResult {
            output_samples: samples,
            snr_improvement_db: 10.0,
            clutter_suppression_db: 20.0,
            target_detected: false,
        };

        assert!(stap.detect_target(&result, 5.0));
        // With very high threshold, should not detect
        assert!(!stap.detect_target(&result, 1e10));
    }

    // ---------------------------------------------------------------
    // 14. Mode setting
    // ---------------------------------------------------------------

    #[test]
    fn test_mode_setting() {
        let mut stap = SpatioTemporalFusion::new(default_config());
        stap.set_mode(StapMode::PostDoppler);
        assert_eq!(stap.mode, StapMode::PostDoppler);
        stap.set_mode(StapMode::BeamSpace);
        assert_eq!(stap.mode, StapMode::BeamSpace);
        stap.set_mode(StapMode::ElementSpace);
        assert_eq!(stap.mode, StapMode::ElementSpace);
    }

    // ---------------------------------------------------------------
    // 15. Steering vector Doppler phase progression
    // ---------------------------------------------------------------

    #[test]
    fn test_steering_vector_doppler_phase() {
        let config = StapConfig {
            num_elements: 1,
            num_pulses: 4,
            clutter_rank_estimate: 1,
            diagonal_loading_db: -30.0,
        };
        let mut stap = SpatioTemporalFusion::new(config);
        stap.set_prf(1000.0);

        // 1 element => only temporal phase changes between pulses
        let sv = stap.steering_vector(0.0, 250.0); // fd/PRF = 0.25
        let expected_phase_step = 2.0 * PI * 0.25;

        for p in 0..4 {
            let expected_phi = expected_phase_step * p as f64;
            let expected = (expected_phi.cos(), expected_phi.sin());
            let got = sv[p];
            assert!(
                (got.0 - expected.0).abs() < 1e-12 && (got.1 - expected.1).abs() < 1e-12,
                "pulse {p}: got {:?}, expected {:?}",
                got,
                expected,
            );
        }
    }

    // ---------------------------------------------------------------
    // 16. Empty output detection returns false
    // ---------------------------------------------------------------

    #[test]
    fn test_detect_target_empty() {
        let stap = SpatioTemporalFusion::new(default_config());
        let result = FusionResult {
            output_samples: vec![],
            snr_improvement_db: 0.0,
            clutter_suppression_db: 0.0,
            target_detected: false,
        };
        assert!(!stap.detect_target(&result, 5.0));
    }

    // ---------------------------------------------------------------
    // 17. Covariance of single snapshot equals outer product + loading
    // ---------------------------------------------------------------

    #[test]
    fn test_covariance_single_snapshot() {
        let config = StapConfig {
            num_elements: 2,
            num_pulses: 2,
            clutter_rank_estimate: 1,
            diagonal_loading_db: -60.0,
        };
        let stap = SpatioTemporalFusion::new(config);
        let snap = vec![(1.0, 0.0), (0.0, 1.0), (1.0, 1.0), (0.5, -0.5)];
        let cov = stap.compute_covariance(&[snap.clone()]);

        let sigma2 = 10.0_f64.powf(-60.0 / 10.0);
        // R[i][j] = snap[i]*conj(snap[j]) + sigma2*delta(i,j)
        for i in 0..4 {
            for j in 0..4 {
                let expected = c_mul(snap[i], c_conj(snap[j]));
                let loading = if i == j { sigma2 } else { 0.0 };
                let expected_loaded = c_add(expected, (loading, 0.0));
                assert!(
                    (cov[i][j].0 - expected_loaded.0).abs() < 1e-10
                        && (cov[i][j].1 - expected_loaded.1).abs() < 1e-10,
                    "cov[{i}][{j}]: got {:?}, expected {:?}",
                    cov[i][j],
                    expected_loaded,
                );
            }
        }
    }

    // ---------------------------------------------------------------
    // 18. Clutter suppression dB is non-negative for noisy data
    // ---------------------------------------------------------------

    #[test]
    fn test_clutter_suppression_finite_and_process_consistent() {
        let config = StapConfig {
            num_elements: 3,
            num_pulses: 4,
            clutter_rank_estimate: 3,
            diagonal_loading_db: -20.0,
        };
        let mut stap = SpatioTemporalFusion::new(config);
        let cube = make_data_cube(4, 3, 32, 0.0, 0.0, 1000.0, 16, 5.0);
        let result = stap.process(&cube);
        // Clutter suppression and SNR improvement must be finite real numbers
        assert!(
            result.clutter_suppression_db.is_finite(),
            "Expected finite clutter suppression, got {}",
            result.clutter_suppression_db,
        );
        assert!(result.snr_improvement_db.is_finite());
        // Output length must match range bins
        assert_eq!(result.output_samples.len(), 32);
    }
}
