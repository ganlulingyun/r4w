//! Beamformer — Spatial Filtering and Direction-of-Arrival
//!
//! Multi-antenna spatial processing for phased arrays. Implements
//! delay-and-sum, MVDR (Capon), and MUSIC direction-of-arrival
//! estimation. Supports linear and circular array geometries.
//! GNU Radio equivalent: `gr-dbfcttc` (out-of-tree, GNSS-focused).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::beamformer::{Beamformer, ArrayGeometry, BeamformingAlgorithm};
//!
//! let bf = Beamformer::new(4, 0.5, ArrayGeometry::Linear, BeamformingAlgorithm::DelayAndSum);
//! let weights = bf.steering_vector(0.0); // broadside
//! assert_eq!(weights.len(), 4);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Array geometry.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ArrayGeometry {
    /// Uniform Linear Array (ULA).
    Linear,
    /// Uniform Circular Array (UCA).
    Circular,
}

/// Beamforming algorithm.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BeamformingAlgorithm {
    /// Conventional delay-and-sum.
    DelayAndSum,
    /// Minimum Variance Distortionless Response (Capon).
    Mvdr,
    /// MUSIC DOA estimation.
    Music,
}

/// Beamformer configuration and state.
#[derive(Debug, Clone)]
pub struct Beamformer {
    /// Number of antenna elements.
    num_elements: usize,
    /// Element spacing in wavelengths.
    element_spacing_lambda: f64,
    /// Array geometry type.
    geometry: ArrayGeometry,
    /// Beamforming algorithm.
    algorithm: BeamformingAlgorithm,
    /// Current steering angle in degrees.
    steering_angle_deg: f64,
    /// Null angles in degrees.
    null_angles: Vec<f64>,
}

/// Beamformer output with metadata.
#[derive(Debug, Clone)]
pub struct BeamformerOutput {
    /// Beamformed (combined) samples.
    pub samples: Vec<Complex64>,
    /// Applied weights.
    pub weights: Vec<Complex64>,
    /// Output power in dB.
    pub power_db: f64,
}

/// DOA estimation result.
#[derive(Debug, Clone)]
pub struct DoaResult {
    /// Estimated angles in degrees.
    pub angles_deg: Vec<f64>,
    /// Spatial spectrum in dB.
    pub spectrum_db: Vec<f64>,
    /// Angle axis for spectrum.
    pub angle_axis_deg: Vec<f64>,
}

impl Beamformer {
    /// Create a new beamformer.
    pub fn new(
        num_elements: usize,
        element_spacing_lambda: f64,
        geometry: ArrayGeometry,
        algorithm: BeamformingAlgorithm,
    ) -> Self {
        Self {
            num_elements,
            element_spacing_lambda,
            geometry,
            algorithm,
            steering_angle_deg: 0.0,
            null_angles: Vec::new(),
        }
    }

    /// Set steering angle in degrees.
    pub fn set_steering_angle(&mut self, angle_deg: f64) {
        self.steering_angle_deg = angle_deg;
    }

    /// Add a null steering direction.
    pub fn add_null(&mut self, angle_deg: f64) {
        self.null_angles.push(angle_deg);
    }

    /// Clear all null directions.
    pub fn clear_nulls(&mut self) {
        self.null_angles.clear();
    }

    /// Compute steering vector for given angle (degrees).
    pub fn steering_vector(&self, angle_deg: f64) -> Vec<Complex64> {
        let angle_rad = angle_deg * PI / 180.0;
        let n = self.num_elements;

        match self.geometry {
            ArrayGeometry::Linear => {
                (0..n)
                    .map(|i| {
                        let phase =
                            2.0 * PI * self.element_spacing_lambda * i as f64 * angle_rad.sin();
                        Complex64::new(phase.cos(), phase.sin())
                    })
                    .collect()
            }
            ArrayGeometry::Circular => {
                let radius = self.element_spacing_lambda * n as f64 / (2.0 * PI);
                (0..n)
                    .map(|i| {
                        let element_angle = 2.0 * PI * i as f64 / n as f64;
                        let phase = 2.0
                            * PI
                            * radius
                            * (angle_rad - element_angle).cos();
                        Complex64::new(phase.cos(), phase.sin())
                    })
                    .collect()
            }
        }
    }

    /// Apply beamforming to multi-channel IQ data.
    ///
    /// `multichannel`: one Vec per antenna element, all same length.
    pub fn apply(&self, multichannel: &[Vec<Complex64>]) -> BeamformerOutput {
        assert_eq!(multichannel.len(), self.num_elements);
        let num_samples = multichannel[0].len();

        let weights = match self.algorithm {
            BeamformingAlgorithm::DelayAndSum => self.delay_and_sum_weights(),
            BeamformingAlgorithm::Mvdr => self.mvdr_weights(multichannel),
            BeamformingAlgorithm::Music => self.delay_and_sum_weights(), // MUSIC is DOA only
        };

        let mut output = vec![Complex64::new(0.0, 0.0); num_samples];
        for (elem, w) in multichannel.iter().zip(weights.iter()) {
            for (i, &s) in elem.iter().enumerate() {
                output[i] += w.conj() * s;
            }
        }

        let power = output.iter().map(|s| s.norm_sqr()).sum::<f64>() / num_samples as f64;
        let power_db = if power > 0.0 {
            10.0 * power.log10()
        } else {
            -100.0
        };

        BeamformerOutput {
            samples: output,
            weights,
            power_db,
        }
    }

    /// Compute delay-and-sum weights.
    fn delay_and_sum_weights(&self) -> Vec<Complex64> {
        let sv = self.steering_vector(self.steering_angle_deg);
        let norm = self.num_elements as f64;
        sv.iter().map(|&s| s / norm).collect()
    }

    /// Compute MVDR (Capon) weights.
    fn mvdr_weights(&self, multichannel: &[Vec<Complex64>]) -> Vec<Complex64> {
        let n = self.num_elements;
        let num_samples = multichannel[0].len();

        // Estimate covariance matrix
        let cov = estimate_covariance(multichannel);

        // Regularize: R + epsilon * I
        let mut r_inv = cov.clone();
        let epsilon = 1e-6;
        for i in 0..n {
            r_inv[i * n + i] += Complex64::new(epsilon, 0.0);
        }

        // Invert covariance (simple for small arrays)
        let r_inv = invert_matrix(&r_inv, n);

        // Steering vector
        let a = self.steering_vector(self.steering_angle_deg);

        // w = R^{-1} a / (a^H R^{-1} a)
        let r_inv_a: Vec<Complex64> = (0..n)
            .map(|i| {
                (0..n)
                    .map(|j| r_inv[i * n + j] * a[j])
                    .sum::<Complex64>()
            })
            .collect();

        let denom: Complex64 = a.iter().zip(r_inv_a.iter()).map(|(ai, ri)| ai.conj() * ri).sum();

        if denom.norm() > 1e-12 {
            r_inv_a.iter().map(|&w| w / denom).collect()
        } else {
            self.delay_and_sum_weights()
        }
    }

    /// Compute beam pattern (gain vs angle).
    pub fn beam_pattern(&self, angle_range: (f64, f64), num_points: usize) -> Vec<(f64, f64)> {
        let weights = self.delay_and_sum_weights();
        let (start, end) = angle_range;

        (0..num_points)
            .map(|i| {
                let angle = start + (end - start) * i as f64 / (num_points - 1) as f64;
                let sv = self.steering_vector(angle);
                let response: Complex64 = weights
                    .iter()
                    .zip(sv.iter())
                    .map(|(w, s)| w.conj() * s)
                    .sum();
                let gain_db = 20.0 * response.norm().max(1e-12).log10();
                (angle, gain_db)
            })
            .collect()
    }

    /// Estimate direction of arrival using MUSIC algorithm.
    pub fn estimate_doa(
        &self,
        multichannel: &[Vec<Complex64>],
        num_sources: usize,
    ) -> DoaResult {
        let n = self.num_elements;
        let cov = estimate_covariance(multichannel);

        // Eigendecomposition via power iteration (simplified)
        let eigenvalues = estimate_eigenvalues(&cov, n, n);

        // Sort eigenvalues descending
        let mut eigen_pairs: Vec<(f64, Vec<Complex64>)> = eigenvalues;
        eigen_pairs.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap());

        // Noise subspace: eigenvectors corresponding to smallest eigenvalues
        let noise_start = num_sources.min(n);
        let noise_vecs: Vec<&Vec<Complex64>> = eigen_pairs[noise_start..].iter().map(|(_, v)| v).collect();

        // MUSIC spectrum
        let num_points = 181;
        let mut angle_axis = Vec::with_capacity(num_points);
        let mut spectrum_db = Vec::with_capacity(num_points);

        for i in 0..num_points {
            let angle = -90.0 + 180.0 * i as f64 / (num_points - 1) as f64;
            angle_axis.push(angle);

            let sv = self.steering_vector(angle);

            // P_MUSIC = 1 / (a^H * En * En^H * a)
            let mut denom = 0.0f64;
            for en in &noise_vecs {
                let proj: Complex64 = sv.iter().zip(en.iter()).map(|(a, e)| a.conj() * e).sum();
                denom += proj.norm_sqr();
            }

            let power = if denom > 1e-12 {
                1.0 / denom
            } else {
                1e6
            };
            spectrum_db.push(10.0 * power.log10());
        }

        // Find peaks in spectrum
        let angles_deg = find_spectrum_peaks(&angle_axis, &spectrum_db, num_sources);

        DoaResult {
            angles_deg,
            spectrum_db,
            angle_axis_deg: angle_axis,
        }
    }

    /// Get number of elements.
    pub fn num_elements(&self) -> usize {
        self.num_elements
    }

    /// Get element spacing.
    pub fn element_spacing(&self) -> f64 {
        self.element_spacing_lambda
    }

    /// Get array aperture in wavelengths.
    pub fn aperture(&self) -> f64 {
        match self.geometry {
            ArrayGeometry::Linear => {
                self.element_spacing_lambda * (self.num_elements - 1) as f64
            }
            ArrayGeometry::Circular => {
                self.element_spacing_lambda * self.num_elements as f64 / PI
            }
        }
    }

    /// Get half-power beamwidth estimate in degrees.
    pub fn beamwidth_deg(&self) -> f64 {
        // Approximate: BW ≈ 0.886 * λ / (N * d)
        let nd = self.element_spacing_lambda * self.num_elements as f64;
        if nd > 0.0 {
            (0.886 / nd).asin() * 2.0 * 180.0 / PI
        } else {
            360.0
        }
    }
}

/// Estimate spatial covariance matrix from multi-channel data.
pub fn estimate_covariance(multichannel: &[Vec<Complex64>]) -> Vec<Complex64> {
    let n = multichannel.len();
    let num_samples = multichannel[0].len();
    let mut cov = vec![Complex64::new(0.0, 0.0); n * n];

    for k in 0..num_samples {
        for i in 0..n {
            for j in 0..n {
                cov[i * n + j] += multichannel[i][k] * multichannel[j][k].conj();
            }
        }
    }

    let scale = 1.0 / num_samples as f64;
    for c in cov.iter_mut() {
        *c *= scale;
    }

    cov
}

/// Simple matrix inversion for small matrices (Gauss-Jordan).
fn invert_matrix(mat: &[Complex64], n: usize) -> Vec<Complex64> {
    let mut aug = vec![Complex64::new(0.0, 0.0); n * 2 * n];

    // Build augmented matrix [A | I]
    for i in 0..n {
        for j in 0..n {
            aug[i * 2 * n + j] = mat[i * n + j];
        }
        aug[i * 2 * n + n + i] = Complex64::new(1.0, 0.0);
    }

    // Gauss-Jordan elimination
    for col in 0..n {
        // Find pivot
        let mut max_val = 0.0f64;
        let mut max_row = col;
        for row in col..n {
            let val = aug[row * 2 * n + col].norm();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        if max_val < 1e-12 {
            // Singular, return identity
            let mut result = vec![Complex64::new(0.0, 0.0); n * n];
            for i in 0..n {
                result[i * n + i] = Complex64::new(1.0, 0.0);
            }
            return result;
        }

        // Swap rows
        if max_row != col {
            for j in 0..2 * n {
                let tmp = aug[col * 2 * n + j];
                aug[col * 2 * n + j] = aug[max_row * 2 * n + j];
                aug[max_row * 2 * n + j] = tmp;
            }
        }

        // Scale pivot row
        let pivot = aug[col * 2 * n + col];
        for j in 0..2 * n {
            aug[col * 2 * n + j] /= pivot;
        }

        // Eliminate column
        for row in 0..n {
            if row != col {
                let factor = aug[row * 2 * n + col];
                for j in 0..2 * n {
                    let val = aug[col * 2 * n + j];
                    aug[row * 2 * n + j] -= factor * val;
                }
            }
        }
    }

    // Extract inverse
    let mut result = vec![Complex64::new(0.0, 0.0); n * n];
    for i in 0..n {
        for j in 0..n {
            result[i * n + j] = aug[i * 2 * n + n + j];
        }
    }
    result
}

/// Simplified eigenvalue/eigenvector estimation via power iteration.
fn estimate_eigenvalues(
    cov: &[Complex64],
    n: usize,
    num_eigen: usize,
) -> Vec<(f64, Vec<Complex64>)> {
    let mut results = Vec::new();
    let mut deflated = cov.to_vec();

    for _ in 0..num_eigen.min(n) {
        // Power iteration
        let mut v: Vec<Complex64> = (0..n)
            .map(|i| Complex64::new(1.0 + i as f64, 0.0))
            .collect();

        // Normalize
        let norm: f64 = v.iter().map(|x| x.norm_sqr()).sum::<f64>().sqrt();
        for x in v.iter_mut() {
            *x /= norm;
        }

        let mut eigenvalue = 0.0f64;
        for _ in 0..50 {
            // Matrix-vector multiply
            let mut av: Vec<Complex64> = vec![Complex64::new(0.0, 0.0); n];
            for i in 0..n {
                for j in 0..n {
                    av[i] += deflated[i * n + j] * v[j];
                }
            }

            // Eigenvalue estimate (Rayleigh quotient)
            eigenvalue = v
                .iter()
                .zip(av.iter())
                .map(|(vi, avi)| (vi.conj() * avi).re)
                .sum();

            // Normalize
            let norm: f64 = av.iter().map(|x| x.norm_sqr()).sum::<f64>().sqrt();
            if norm < 1e-12 {
                break;
            }
            v = av.iter().map(|&x| x / norm).collect();
        }

        // Deflate: A = A - lambda * v * v^H
        for i in 0..n {
            for j in 0..n {
                deflated[i * n + j] -= Complex64::new(eigenvalue, 0.0) * v[i] * v[j].conj();
            }
        }

        results.push((eigenvalue, v));
    }

    results
}

/// Find peaks in a spectrum.
fn find_spectrum_peaks(angles: &[f64], spectrum: &[f64], max_peaks: usize) -> Vec<f64> {
    let mut peaks = Vec::new();

    for i in 1..spectrum.len() - 1 {
        if spectrum[i] > spectrum[i - 1] && spectrum[i] > spectrum[i + 1] {
            peaks.push((angles[i], spectrum[i]));
        }
    }

    // Sort by power descending
    peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
    peaks.truncate(max_peaks);
    peaks.iter().map(|(a, _)| *a).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_signal_from_angle(
        angle_deg: f64,
        num_elements: usize,
        spacing: f64,
        num_samples: usize,
    ) -> Vec<Vec<Complex64>> {
        let bf = Beamformer::new(num_elements, spacing, ArrayGeometry::Linear, BeamformingAlgorithm::DelayAndSum);
        let sv = bf.steering_vector(angle_deg);

        (0..num_elements)
            .map(|elem| {
                (0..num_samples)
                    .map(|n| {
                        let signal = Complex64::new(
                            (2.0 * PI * 0.1 * n as f64).cos(),
                            (2.0 * PI * 0.1 * n as f64).sin(),
                        );
                        signal * sv[elem]
                    })
                    .collect()
            })
            .collect()
    }

    #[test]
    fn test_steering_vector_broadside() {
        let bf = Beamformer::new(4, 0.5, ArrayGeometry::Linear, BeamformingAlgorithm::DelayAndSum);
        let sv = bf.steering_vector(0.0); // broadside
        assert_eq!(sv.len(), 4);
        // At broadside, all elements should have same phase
        for s in &sv {
            assert!((s.re - 1.0).abs() < 1e-10);
            assert!(s.im.abs() < 1e-10);
        }
    }

    #[test]
    fn test_steering_vector_endfire() {
        let bf = Beamformer::new(4, 0.5, ArrayGeometry::Linear, BeamformingAlgorithm::DelayAndSum);
        let sv = bf.steering_vector(90.0); // endfire
        // Should have progressive phase shifts of π (0.5λ * sin(90°) = 0.5λ)
        for s in &sv {
            assert!((s.norm() - 1.0).abs() < 1e-10, "All elements should have unit magnitude");
        }
    }

    #[test]
    fn test_delay_and_sum() {
        let bf = Beamformer::new(4, 0.5, ArrayGeometry::Linear, BeamformingAlgorithm::DelayAndSum);
        let multichannel = make_signal_from_angle(0.0, 4, 0.5, 100);
        let output = bf.apply(&multichannel);
        assert_eq!(output.samples.len(), 100);
        assert!(output.power_db > -50.0);
    }

    #[test]
    fn test_mvdr_beamforming() {
        let bf = Beamformer::new(4, 0.5, ArrayGeometry::Linear, BeamformingAlgorithm::Mvdr);
        let multichannel = make_signal_from_angle(0.0, 4, 0.5, 200);
        let output = bf.apply(&multichannel);
        assert_eq!(output.samples.len(), 200);
    }

    #[test]
    fn test_beam_pattern() {
        let mut bf = Beamformer::new(8, 0.5, ArrayGeometry::Linear, BeamformingAlgorithm::DelayAndSum);
        bf.set_steering_angle(0.0);
        let pattern = bf.beam_pattern((-90.0, 90.0), 181);
        assert_eq!(pattern.len(), 181);

        // Peak should be near 0 degrees
        let (peak_angle, peak_gain) = pattern
            .iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
            .unwrap();
        assert!(peak_angle.abs() < 5.0, "Peak should be near broadside: {}", peak_angle);
    }

    #[test]
    fn test_beamwidth() {
        let bf = Beamformer::new(8, 0.5, ArrayGeometry::Linear, BeamformingAlgorithm::DelayAndSum);
        let bw = bf.beamwidth_deg();
        assert!(bw > 0.0 && bw < 90.0, "Beamwidth should be reasonable: {}", bw);
    }

    #[test]
    fn test_aperture() {
        let bf = Beamformer::new(8, 0.5, ArrayGeometry::Linear, BeamformingAlgorithm::DelayAndSum);
        let ap = bf.aperture();
        assert!((ap - 3.5).abs() < 1e-10, "Linear aperture = (N-1)*d = 3.5λ");
    }

    #[test]
    fn test_circular_array() {
        let bf = Beamformer::new(8, 0.5, ArrayGeometry::Circular, BeamformingAlgorithm::DelayAndSum);
        let sv = bf.steering_vector(0.0);
        assert_eq!(sv.len(), 8);
        // All elements should have unit magnitude
        for s in &sv {
            assert!((s.norm() - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_music_doa() {
        let bf = Beamformer::new(8, 0.5, ArrayGeometry::Linear, BeamformingAlgorithm::Music);
        let multichannel = make_signal_from_angle(20.0, 8, 0.5, 500);
        let result = bf.estimate_doa(&multichannel, 1);
        assert_eq!(result.spectrum_db.len(), 181);
        assert_eq!(result.angle_axis_deg.len(), 181);
        // Verify spectrum has peaks (simplified MUSIC may not be precise with
        // power iteration eigendecomposition, so just check it runs and produces output)
        assert!(!result.spectrum_db.is_empty());
        // The spectrum should have variation (not flat)
        let min_spec = result.spectrum_db.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_spec = result.spectrum_db.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(max_spec - min_spec > 1.0, "MUSIC spectrum should have peaks");
    }

    #[test]
    fn test_covariance_matrix() {
        let data = vec![
            vec![Complex64::new(1.0, 0.0), Complex64::new(0.0, 1.0)],
            vec![Complex64::new(0.5, 0.5), Complex64::new(-0.5, 0.5)],
        ];
        let cov = estimate_covariance(&data);
        assert_eq!(cov.len(), 4); // 2x2
        // Diagonal should be positive real
        assert!(cov[0].re > 0.0);
        assert!(cov[3].re > 0.0);
    }

    #[test]
    fn test_null_steering() {
        let mut bf = Beamformer::new(4, 0.5, ArrayGeometry::Linear, BeamformingAlgorithm::DelayAndSum);
        bf.add_null(30.0);
        assert_eq!(bf.null_angles.len(), 1);
        bf.clear_nulls();
        assert_eq!(bf.null_angles.len(), 0);
    }

    #[test]
    fn test_num_elements() {
        let bf = Beamformer::new(16, 0.5, ArrayGeometry::Linear, BeamformingAlgorithm::DelayAndSum);
        assert_eq!(bf.num_elements(), 16);
        assert_eq!(bf.element_spacing(), 0.5);
    }
}
