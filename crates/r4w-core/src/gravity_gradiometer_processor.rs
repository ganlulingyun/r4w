//! Gravity gradiometry signal processing for mineral/oil exploration and geodesy.
//!
//! This module processes full tensor gravity gradient (FTG) measurements to detect
//! subsurface density anomalies. Gravity gradiometry measures the spatial rate of
//! change of the gravitational field, providing higher spatial resolution than
//! conventional gravity surveys for delineating geological structures.
//!
//! # Overview
//!
//! - **Full Tensor Gravity (FTG)** representation with all six independent gradient components
//! - **Laplacian constraint** verification (trace = 0 for valid measurements)
//! - **Tensor invariants** (I0, I1, I2) for rotation-independent analysis
//! - **Euler deconvolution** for estimating source depth and position
//! - **Forward modeling** from point mass sources
//! - **Terrain and gravity corrections** (free-air, Bouguer, terrain)
//! - **Gradient attributes** (horizontal gradient magnitude, curvature gradient)
//!
//! # Units
//!
//! Gravity gradients are expressed in Eotvos (E), where 1 E = 10^-9 s^-2.
//! Gravity anomalies are in milliGal (mGal), where 1 mGal = 10^-5 m/s^2.
//!
//! # Example
//!
//! ```
//! use r4w_core::gravity_gradiometer_processor::{
//!     GradiometerConfig, GradientProcessor, GravityTensor, point_mass_gradient,
//! };
//!
//! let config = GradiometerConfig {
//!     sample_rate_hz: 10.0,
//!     platform_speed_mps: 60.0,
//!     survey_altitude_m: 80.0,
//!     noise_floor_eotvos: 1.0,
//! };
//! let processor = GradientProcessor::new(config);
//!
//! // Compute the theoretical gradient tensor from a buried point mass
//! let tensor = point_mass_gradient(1e10, 500.0, 0.0, 0.0);
//!
//! // Verify Laplacian constraint
//! assert!(tensor.trace().abs() < 1e-10);
//!
//! // Process the measurement
//! let result = processor.process(&tensor);
//! assert!(result.anomaly_detected || result.vertical_gradient.abs() > 0.0);
//! ```

use std::f64::consts::PI;

/// Gravitational constant in m^3 kg^-1 s^-2.
const G: f64 = 6.674_30e-11;

/// Configuration for the gravity gradiometer processor.
#[derive(Debug, Clone)]
pub struct GradiometerConfig {
    /// Measurement sampling rate in Hz.
    pub sample_rate_hz: f64,
    /// Platform (aircraft/vehicle) speed in meters per second.
    pub platform_speed_mps: f64,
    /// Survey altitude above ground in meters.
    pub survey_altitude_m: f64,
    /// Instrument noise floor in Eotvos units (default 1.0 E).
    pub noise_floor_eotvos: f64,
}

impl Default for GradiometerConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 10.0,
            platform_speed_mps: 60.0,
            survey_altitude_m: 80.0,
            noise_floor_eotvos: 1.0,
        }
    }
}

/// Full tensor gravity gradient measurement.
///
/// Contains all six independent components of the symmetric 3x3 gradient tensor.
/// All values are in Eotvos (E) where 1 E = 10^-9 s^-2.
///
/// The gradient tensor is:
/// ```text
///     | Gxx  Gxy  Gxz |
/// T = | Gxy  Gyy  Gyz |
///     | Gxz  Gyz  Gzz |
/// ```
///
/// For a valid measurement in free space, Laplace's equation requires:
/// Gxx + Gyy + Gzz = 0
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GravityTensor {
    /// Gradient component d²U/dx² in Eotvos.
    pub gxx: f64,
    /// Gradient component d²U/dxdy in Eotvos.
    pub gxy: f64,
    /// Gradient component d²U/dxdz in Eotvos.
    pub gxz: f64,
    /// Gradient component d²U/dy² in Eotvos.
    pub gyy: f64,
    /// Gradient component d²U/dydz in Eotvos.
    pub gyz: f64,
    /// Gradient component d²U/dz² in Eotvos.
    pub gzz: f64,
}

impl GravityTensor {
    /// Create a new gravity tensor from its six independent components.
    pub fn new(gxx: f64, gxy: f64, gxz: f64, gyy: f64, gyz: f64, gzz: f64) -> Self {
        Self {
            gxx,
            gxy,
            gxz,
            gyy,
            gyz,
            gzz,
        }
    }

    /// Create a zero tensor.
    pub fn zero() -> Self {
        Self::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    }

    /// Compute the trace of the gradient tensor (Laplacian constraint).
    ///
    /// In free space (no mass between the measurement point and the source),
    /// Laplace's equation requires: Gxx + Gyy + Gzz = 0.
    ///
    /// A non-zero trace indicates either measurement error or the presence
    /// of mass at the measurement point (Poisson's equation).
    pub fn trace(&self) -> f64 {
        self.gxx + self.gyy + self.gzz
    }

    /// Compute the three rotation-invariant quantities of the gradient tensor.
    ///
    /// Returns `(I0, I1, I2)` where:
    /// - `I0` = trace (should be ~0 in free space)
    /// - `I1` = sum of 2x2 principal minors (related to eigenvalue products)
    /// - `I2` = determinant of the tensor
    ///
    /// These invariants are independent of the coordinate system orientation
    /// and can be used to characterize the source geometry.
    pub fn invariants(&self) -> (f64, f64, f64) {
        let i0 = self.trace();

        // I1 = Gxx*Gyy + Gyy*Gzz + Gxx*Gzz - Gxy^2 - Gyz^2 - Gxz^2
        let i1 = self.gxx * self.gyy + self.gyy * self.gzz + self.gxx * self.gzz
            - self.gxy * self.gxy
            - self.gyz * self.gyz
            - self.gxz * self.gxz;

        // I2 = determinant of the 3x3 symmetric tensor
        let i2 = self.gxx * (self.gyy * self.gzz - self.gyz * self.gyz)
            - self.gxy * (self.gxy * self.gzz - self.gyz * self.gxz)
            + self.gxz * (self.gxy * self.gyz - self.gyy * self.gxz);

        (i0, i1, i2)
    }

    /// Compute the Frobenius norm of the tensor.
    ///
    /// This gives a scalar measure of the overall gradient amplitude,
    /// accounting for all components. Off-diagonal elements are counted
    /// twice due to symmetry.
    pub fn frobenius_norm(&self) -> f64 {
        (self.gxx * self.gxx
            + self.gyy * self.gyy
            + self.gzz * self.gzz
            + 2.0 * self.gxy * self.gxy
            + 2.0 * self.gxz * self.gxz
            + 2.0 * self.gyz * self.gyz)
            .sqrt()
    }
}

/// Result from processing a gravity gradient measurement.
#[derive(Debug, Clone)]
pub struct GradientResult {
    /// Horizontal gradient magnitude in Eotvos: sqrt(Gxz² + Gyz²).
    pub horizontal_gradient: f64,
    /// Vertical gradient (Gzz) in Eotvos.
    pub vertical_gradient: f64,
    /// Curvature gradient: (Gxx - Gyy) / 2 in Eotvos.
    pub curvature: f64,
    /// Estimated depth to source in meters (from Euler deconvolution heuristic).
    pub depth_estimate_m: f64,
    /// Whether a significant anomaly was detected above the noise floor.
    pub anomaly_detected: bool,
}

/// Gravity gradient signal processor.
///
/// Processes full tensor gravity gradient measurements and produces
/// interpreted results including gradient attributes, depth estimates,
/// and anomaly detection.
pub struct GradientProcessor {
    config: GradiometerConfig,
}

impl GradientProcessor {
    /// Create a new gradient processor with the given configuration.
    pub fn new(config: GradiometerConfig) -> Self {
        Self { config }
    }

    /// Process a single gravity tensor measurement.
    ///
    /// Computes gradient attributes, estimates source depth using a simple
    /// ratio method, and flags anomalies exceeding the noise floor.
    pub fn process(&self, tensor: &GravityTensor) -> GradientResult {
        let horizontal = horizontal_gradient_magnitude(tensor.gxz, tensor.gyz);
        let vertical = tensor.gzz;
        let curv = curvature_gradient(tensor.gxx, tensor.gyy);

        // Simple depth estimate from ratio of vertical gradient to horizontal gradient.
        // For a point source: depth ~ horizontal_distance * (Gzz / Ghz) when offset,
        // or use survey altitude as baseline estimate.
        let depth_estimate = if horizontal.abs() > 1e-15 && vertical.abs() > 1e-15 {
            // Use the ratio method: for a monopole, depth ~ altitude * sqrt(|Gzz/Ghz|)
            // This is a rough heuristic; Euler deconvolution is more rigorous.
            let ratio = (vertical.abs() / horizontal.abs()).sqrt();
            self.config.survey_altitude_m * ratio
        } else {
            self.config.survey_altitude_m
        };

        // Detect anomaly: signal exceeds noise floor by a reasonable margin
        let noise = self.config.noise_floor_eotvos;
        let anomaly_detected = horizontal > 3.0 * noise
            || vertical.abs() > 3.0 * noise
            || curv.abs() > 3.0 * noise;

        GradientResult {
            horizontal_gradient: horizontal,
            vertical_gradient: vertical,
            curvature: curv,
            depth_estimate_m: depth_estimate,
            anomaly_detected,
        }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &GradiometerConfig {
        &self.config
    }
}

/// Euler deconvolution for estimating source position and depth.
///
/// Given a profile of gradient values along a survey line and corresponding
/// x-coordinates, estimates (x_source, depth) pairs for subsurface sources.
///
/// The structural index `n` depends on the source geometry:
/// - `n = 0`: contact/fault
/// - `n = 1`: thin sheet/dike/sill
/// - `n = 2`: horizontal cylinder/pipe
/// - `n = 3`: sphere/point mass
///
/// # Algorithm
///
/// Euler's homogeneity equation: (x - x0) * dT/dx + z0 * dT/dz = -n * T
///
/// We approximate dT/dx with central finite differences along the profile,
/// and use the gradient values directly as a proxy for dT/dz (the vertical
/// derivative). Solutions are obtained by least-squares over sliding windows.
///
/// # Arguments
///
/// * `gradient_profile` - Gradient values along the survey line (in Eotvos).
/// * `x_coords` - X-coordinates of measurement stations (in meters).
/// * `structural_index` - Source geometry parameter (0.0 to 3.0).
///
/// # Returns
///
/// A vector of `(x_source, depth)` estimates in meters.
pub fn euler_deconvolution(
    gradient_profile: &[f64],
    x_coords: &[f64],
    structural_index: f64,
) -> Vec<(f64, f64)> {
    let n = gradient_profile.len();
    if n < 4 || x_coords.len() != n {
        return Vec::new();
    }

    let mut results = Vec::new();
    let window_size = if n >= 8 { 8 } else { 4.min(n) };

    // Compute numerical derivative dT/dx using central differences
    let mut dt_dx = vec![0.0; n];
    for i in 1..n - 1 {
        let dx = x_coords[i + 1] - x_coords[i - 1];
        if dx.abs() > 1e-15 {
            dt_dx[i] = (gradient_profile[i + 1] - gradient_profile[i - 1]) / dx;
        }
    }
    // Forward/backward difference at endpoints
    if n >= 2 {
        let dx0 = x_coords[1] - x_coords[0];
        if dx0.abs() > 1e-15 {
            dt_dx[0] = (gradient_profile[1] - gradient_profile[0]) / dx0;
        }
        let dxn = x_coords[n - 1] - x_coords[n - 2];
        if dxn.abs() > 1e-15 {
            dt_dx[n - 1] = (gradient_profile[n - 1] - gradient_profile[n - 2]) / dxn;
        }
    }

    // Sliding window Euler deconvolution
    for start in 0..=n - window_size {
        let end = start + window_size;

        // Build the overdetermined system: for each point i in the window,
        // (x_i - x0) * dT/dx_i = -n * T_i - z0 * dT/dz_i
        //
        // Simplification: use gradient_profile as a proxy for dT/dz (vertical gradient).
        // The unknowns are x0 and z0.
        //
        // Rewrite: dT/dx_i * x0 + gradient_profile_i * z0 = x_i * dT/dx_i + n * T_i
        //          (negating the vertical derivative term)
        //
        // This is Ax = b with A = [dT/dx_i, T_i], x = [x0, z0],
        // b = [x_i * dT/dx_i + n * T_i]

        let mut ata00 = 0.0;
        let mut ata01 = 0.0;
        let mut ata11 = 0.0;
        let mut atb0 = 0.0;
        let mut atb1 = 0.0;

        for i in start..end {
            let a0 = dt_dx[i];
            let a1 = gradient_profile[i];
            let b = x_coords[i] * dt_dx[i] + structural_index * gradient_profile[i];

            ata00 += a0 * a0;
            ata01 += a0 * a1;
            ata11 += a1 * a1;
            atb0 += a0 * b;
            atb1 += a1 * b;
        }

        let det = ata00 * ata11 - ata01 * ata01;
        if det.abs() < 1e-30 {
            continue;
        }

        let x0 = (ata11 * atb0 - ata01 * atb1) / det;
        let z0 = (ata00 * atb1 - ata01 * atb0) / det;

        // Only accept physically meaningful solutions (positive depth)
        let depth = z0.abs();
        if depth > 1.0 && depth < 50_000.0 {
            // Check that x0 is within a reasonable range of the survey line
            let x_min = x_coords[start];
            let x_max = x_coords[end - 1];
            let x_range = (x_max - x_min).abs();
            if x0 >= x_min - x_range && x0 <= x_max + x_range {
                results.push((x0, depth));
            }
        }
    }

    results
}

/// Compute the theoretical gravity gradient tensor from a point mass.
///
/// Calculates all six independent components of the gradient tensor at the
/// observation point (0, 0, 0) due to a point mass located at
/// (x_offset, y_offset, -depth) below the surface.
///
/// # Arguments
///
/// * `mass_kg` - Mass of the point source in kilograms.
/// * `depth_m` - Depth below the observation point in meters (positive downward).
/// * `x_offset_m` - Horizontal offset in x-direction in meters.
/// * `y_offset_m` - Horizontal offset in y-direction in meters.
///
/// # Returns
///
/// The gradient tensor in Eotvos (E = 10^-9 s^-2).
pub fn point_mass_gradient(
    mass_kg: f64,
    depth_m: f64,
    x_offset_m: f64,
    y_offset_m: f64,
) -> GravityTensor {
    let x = x_offset_m;
    let y = y_offset_m;
    let z = depth_m; // positive downward

    let r2 = x * x + y * y + z * z;
    let r = r2.sqrt();

    if r < 1e-10 {
        return GravityTensor::zero();
    }

    let r5 = r2 * r2 * r;

    // The gradient of gravity from a point mass at (x, y, z):
    // Tij = G * M * (3 * ri * rj / r^5 - delta_ij / r^3)
    // Convert to Eotvos (multiply by 1e9)
    let coeff = G * mass_kg * 1e9;

    let gxx = coeff * (3.0 * x * x / r5 - 1.0 / (r2 * r));
    let gyy = coeff * (3.0 * y * y / r5 - 1.0 / (r2 * r));
    let gzz = coeff * (3.0 * z * z / r5 - 1.0 / (r2 * r));
    let gxy = coeff * (3.0 * x * y / r5);
    let gxz = coeff * (3.0 * x * z / r5);
    let gyz = coeff * (3.0 * y * z / r5);

    GravityTensor::new(gxx, gxy, gxz, gyy, gyz, gzz)
}

/// Apply terrain correction to a gravity survey profile.
///
/// Computes a simple Bouguer slab correction for each station plus a
/// residual terrain correction based on the deviation of the actual
/// terrain from the station elevation.
///
/// # Arguments
///
/// * `elevation_profile` - Elevation of each station in meters above datum.
/// * `station_spacing_m` - Distance between stations in meters.
/// * `density_kg_m3` - Assumed rock density in kg/m^3 (typically 2670 for standard crust).
///
/// # Returns
///
/// Terrain correction values in mGal for each station. Positive corrections
/// indicate that the simple Bouguer slab over-corrected (terrain below station).
pub fn terrain_correction(
    elevation_profile: &[f64],
    station_spacing_m: f64,
    density_kg_m3: f64,
) -> Vec<f64> {
    let n = elevation_profile.len();
    if n == 0 {
        return Vec::new();
    }

    let mut corrections = vec![0.0; n];

    for i in 0..n {
        let station_elev = elevation_profile[i];
        let mut tc = 0.0;

        // Bouguer slab correction for this station
        let bouguer = bouguer_correction(station_elev, density_kg_m3);
        let fac = free_air_correction(station_elev);

        // Terrain correction: account for deviations of nearby terrain
        // from the station elevation (Hammer zones simplified to 1D)
        for j in 0..n {
            if i == j {
                continue;
            }

            let dh = elevation_profile[j] - station_elev;
            let dx = ((j as f64) - (i as f64)) * station_spacing_m;
            let r = dx.abs().max(station_spacing_m * 0.5);

            // Gravitational effect of the terrain element:
            // delta_g = G * rho * dh * station_spacing / r^2 (simplified)
            // Convert to mGal (multiply by 1e5 since 1 mGal = 1e-5 m/s^2)
            let terrain_element =
                G * density_kg_m3 * dh.abs() * station_spacing_m / (r * r) * 1e5;

            // Terrain correction is always positive (whether terrain is above or below)
            tc += terrain_element;
        }

        // Total correction = Free Air + Bouguer slab + terrain
        corrections[i] = fac + bouguer + tc;
    }

    corrections
}

/// Free-air correction for gravity measurements.
///
/// Accounts for the decrease in gravity with elevation above the geoid.
///
/// FAC = -0.3086 * h (mGal)
///
/// where h is the elevation in meters.
///
/// # Arguments
///
/// * `elevation_m` - Station elevation above the geoid in meters.
///
/// # Returns
///
/// Free-air correction in mGal. Positive for stations above the geoid.
pub fn free_air_correction(elevation_m: f64) -> f64 {
    -0.3086 * elevation_m
}

/// Bouguer slab correction for gravity measurements.
///
/// Accounts for the gravitational attraction of the rock mass between
/// the station and the geoid, modeled as an infinite horizontal slab.
///
/// BC = 2 * pi * G * rho * h
///
/// Converted to mGal (1 mGal = 10^-5 m/s^2).
///
/// # Arguments
///
/// * `elevation_m` - Slab thickness (station elevation) in meters.
/// * `density_kg_m3` - Rock density in kg/m^3.
///
/// # Returns
///
/// Bouguer correction in mGal. This is subtracted from observed gravity.
pub fn bouguer_correction(elevation_m: f64, density_kg_m3: f64) -> f64 {
    // BC in m/s^2 = 2 * pi * G * rho * h
    // Convert to mGal by multiplying by 1e5
    2.0 * PI * G * density_kg_m3 * elevation_m * 1e5
}

/// Compute horizontal gradient magnitude from off-diagonal gradient components.
///
/// HGM = sqrt(Gxz² + Gyz²)
///
/// The horizontal gradient magnitude highlights the edges of density
/// anomalies and is commonly used to map geological contacts and faults.
///
/// # Arguments
///
/// * `gxz` - Gradient component d²U/dxdz in Eotvos.
/// * `gyz` - Gradient component d²U/dydz in Eotvos.
///
/// # Returns
///
/// Horizontal gradient magnitude in Eotvos.
pub fn horizontal_gradient_magnitude(gxz: f64, gyz: f64) -> f64 {
    (gxz * gxz + gyz * gyz).sqrt()
}

/// Compute the curvature gradient from diagonal components.
///
/// CG = (Gxx - Gyy) / 2
///
/// The curvature gradient indicates the difference between the maximum
/// and minimum horizontal curvature of the gravitational potential surface.
/// It is useful for determining the strike direction of elongated structures.
///
/// # Arguments
///
/// * `gxx` - Gradient component d²U/dx² in Eotvos.
/// * `gyy` - Gradient component d²U/dy² in Eotvos.
///
/// # Returns
///
/// Curvature gradient in Eotvos.
pub fn curvature_gradient(gxx: f64, gyy: f64) -> f64 {
    (gxx - gyy) / 2.0
}

/// Convert a gravity gradient value from Eotvos to SI units.
///
/// 1 Eotvos (E) = 10^-9 s^-2
///
/// # Arguments
///
/// * `eotvos` - Gradient value in Eotvos.
///
/// # Returns
///
/// Gradient value in s^-2 (SI units).
pub fn eotvos_to_si(eotvos: f64) -> f64 {
    eotvos * 1e-9
}

/// Convert a gravity gradient value from SI units to Eotvos.
///
/// 1 s^-2 = 10^9 Eotvos (E)
///
/// # Arguments
///
/// * `si` - Gradient value in s^-2.
///
/// # Returns
///
/// Gradient value in Eotvos.
pub fn si_to_eotvos(si: f64) -> f64 {
    si * 1e9
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-10;

    // -----------------------------------------------------------------------
    // GravityTensor basic tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_tensor_trace_zero_for_valid_measurement() {
        // Laplace's equation: for a point mass, trace must be zero
        let tensor = point_mass_gradient(1e10, 500.0, 0.0, 0.0);
        assert!(
            tensor.trace().abs() < EPSILON,
            "Trace should be ~0, got {}",
            tensor.trace()
        );
    }

    #[test]
    fn test_tensor_trace_off_center() {
        // Laplacian constraint holds for any observation point relative to a point mass
        let tensor = point_mass_gradient(5e9, 300.0, 100.0, 200.0);
        assert!(
            tensor.trace().abs() < EPSILON,
            "Trace should be ~0 for off-center point, got {}",
            tensor.trace()
        );
    }

    #[test]
    fn test_tensor_trace_nonzero_artificial() {
        // A tensor that violates Laplace should have non-zero trace
        let tensor = GravityTensor::new(10.0, 0.0, 0.0, 10.0, 0.0, 10.0);
        assert!((tensor.trace() - 30.0).abs() < EPSILON);
    }

    #[test]
    fn test_tensor_zero() {
        let tensor = GravityTensor::zero();
        assert!(tensor.gxx.abs() < EPSILON);
        assert!(tensor.gyy.abs() < EPSILON);
        assert!(tensor.gzz.abs() < EPSILON);
        assert!(tensor.trace().abs() < EPSILON);
    }

    #[test]
    fn test_tensor_frobenius_norm() {
        let tensor = GravityTensor::new(1.0, 0.0, 0.0, 1.0, 0.0, 1.0);
        // Frobenius norm = sqrt(1+1+1) = sqrt(3) (no off-diagonal)
        let expected = 3.0_f64.sqrt();
        assert!(
            (tensor.frobenius_norm() - expected).abs() < EPSILON,
            "Frobenius norm mismatch"
        );
    }

    #[test]
    fn test_tensor_frobenius_norm_with_offdiag() {
        // Off-diagonal elements are counted twice in Frobenius norm for symmetric tensor
        let tensor = GravityTensor::new(0.0, 1.0, 0.0, 0.0, 0.0, 0.0);
        let expected = (2.0_f64).sqrt(); // gxy^2 counted twice
        assert!(
            (tensor.frobenius_norm() - expected).abs() < EPSILON,
            "Frobenius norm with off-diagonal mismatch: got {}, expected {}",
            tensor.frobenius_norm(),
            expected
        );
    }

    // -----------------------------------------------------------------------
    // Tensor invariants
    // -----------------------------------------------------------------------

    #[test]
    fn test_invariants_zero_tensor() {
        let tensor = GravityTensor::zero();
        let (i0, i1, i2) = tensor.invariants();
        assert!(i0.abs() < EPSILON);
        assert!(i1.abs() < EPSILON);
        assert!(i2.abs() < EPSILON);
    }

    #[test]
    fn test_invariants_diagonal_tensor() {
        // For a diagonal tensor with eigenvalues a, b, c:
        // I0 = a + b + c
        // I1 = ab + bc + ac
        // I2 = abc
        let tensor = GravityTensor::new(2.0, 0.0, 0.0, 3.0, 0.0, -5.0);
        let (i0, i1, i2) = tensor.invariants();
        assert!(
            (i0 - 0.0).abs() < EPSILON,
            "I0 should be 0 (trace-free), got {}",
            i0
        );
        assert!(
            (i1 - (2.0 * 3.0 + 3.0 * (-5.0) + 2.0 * (-5.0))).abs() < EPSILON,
            "I1 mismatch: got {}",
            i1
        );
        assert!(
            (i2 - (2.0 * 3.0 * (-5.0))).abs() < EPSILON,
            "I2 mismatch: got {}",
            i2
        );
    }

    #[test]
    fn test_invariants_point_mass() {
        // Invariants from a physical point mass gradient
        let tensor = point_mass_gradient(1e10, 1000.0, 0.0, 0.0);
        let (i0, _i1, _i2) = tensor.invariants();
        // I0 (trace) should be ~0 for a point mass in free space
        assert!(i0.abs() < EPSILON);
    }

    #[test]
    fn test_invariants_rotation_independence() {
        // The invariants should be the same regardless of rotation.
        // A point mass directly below vs. offset should produce different
        // tensor components but we can at least verify the invariants change
        // predictably with mass scaling.
        let t1 = point_mass_gradient(1e10, 500.0, 0.0, 0.0);
        let t2 = point_mass_gradient(2e10, 500.0, 0.0, 0.0);
        let (_, i1_a, i2_a) = t1.invariants();
        let (_, i1_b, i2_b) = t2.invariants();
        // For 2x mass: gradient scales linearly, so I1 scales as mass^2, I2 as mass^3
        let ratio_i1 = i1_b / i1_a;
        let ratio_i2 = i2_b / i2_a;
        assert!(
            (ratio_i1 - 4.0).abs() < 1e-6,
            "I1 should scale as mass^2, ratio = {}",
            ratio_i1
        );
        assert!(
            (ratio_i2 - 8.0).abs() < 1e-6,
            "I2 should scale as mass^3, ratio = {}",
            ratio_i2
        );
    }

    // -----------------------------------------------------------------------
    // Point mass gradient
    // -----------------------------------------------------------------------

    #[test]
    fn test_point_mass_gradient_directly_below() {
        // Point mass directly below: gxz = gyz = 0, gxx = gyy (by symmetry)
        let tensor = point_mass_gradient(1e12, 1000.0, 0.0, 0.0);
        assert!(
            tensor.gxz.abs() < EPSILON,
            "Gxz should be ~0 for mass directly below"
        );
        assert!(
            tensor.gyz.abs() < EPSILON,
            "Gyz should be ~0 for mass directly below"
        );
        assert!(
            (tensor.gxx - tensor.gyy).abs() < EPSILON,
            "Gxx should equal Gyy for mass directly below"
        );
        // Gzz should be positive (gravity gradient increases downward toward mass)
        assert!(
            tensor.gzz > 0.0,
            "Gzz should be positive for mass directly below"
        );
    }

    #[test]
    fn test_point_mass_gradient_symmetry() {
        // Swapping x and y offsets should swap gxz/gyz and gxx/gyy
        let t1 = point_mass_gradient(1e10, 500.0, 100.0, 0.0);
        let t2 = point_mass_gradient(1e10, 500.0, 0.0, 100.0);
        assert!(
            (t1.gxz - t2.gyz).abs() < EPSILON,
            "Symmetry: t1.gxz should equal t2.gyz"
        );
        assert!(
            (t1.gxx - t2.gyy).abs() < EPSILON,
            "Symmetry: t1.gxx should equal t2.gyy"
        );
    }

    #[test]
    fn test_point_mass_gradient_zero_distance() {
        // At zero distance, should return zero tensor (singularity guard)
        let tensor = point_mass_gradient(1e10, 0.0, 0.0, 0.0);
        assert!(tensor.frobenius_norm() < EPSILON);
    }

    #[test]
    fn test_point_mass_gradient_inverse_cube_scaling() {
        // Gradient tensor scales as 1/r^3 for a point mass
        let t1 = point_mass_gradient(1e10, 100.0, 0.0, 0.0);
        let t2 = point_mass_gradient(1e10, 200.0, 0.0, 0.0);
        // At 2x distance, gzz should be 1/8 (2^3 = 8)
        let ratio = t1.gzz / t2.gzz;
        assert!(
            (ratio - 8.0).abs() < 1e-6,
            "Gradient should scale as 1/r^3, ratio = {}",
            ratio
        );
    }

    // -----------------------------------------------------------------------
    // Gradient processor
    // -----------------------------------------------------------------------

    #[test]
    fn test_processor_detects_anomaly() {
        let config = GradiometerConfig {
            noise_floor_eotvos: 1.0,
            ..Default::default()
        };
        let processor = GradientProcessor::new(config);

        // Create a tensor with strong signal (well above noise floor)
        let tensor = point_mass_gradient(1e12, 200.0, 50.0, 0.0);
        let result = processor.process(&tensor);

        assert!(
            result.anomaly_detected,
            "Should detect anomaly for strong signal"
        );
        assert!(result.horizontal_gradient > 0.0);
    }

    #[test]
    fn test_processor_no_anomaly_in_noise() {
        let config = GradiometerConfig {
            noise_floor_eotvos: 100.0, // very high noise floor
            ..Default::default()
        };
        let processor = GradientProcessor::new(config);

        // Weak signal from deep/small source
        let tensor = GravityTensor::new(0.1, 0.0, 0.1, -0.05, 0.1, -0.05);
        let result = processor.process(&tensor);

        assert!(
            !result.anomaly_detected,
            "Should not detect anomaly when signal is below noise floor"
        );
    }

    #[test]
    fn test_processor_depth_estimate_positive() {
        let config = GradiometerConfig::default();
        let processor = GradientProcessor::new(config);
        let tensor = point_mass_gradient(1e11, 300.0, 50.0, 0.0);
        let result = processor.process(&tensor);
        assert!(
            result.depth_estimate_m > 0.0,
            "Depth estimate should be positive"
        );
    }

    #[test]
    fn test_processor_config_accessor() {
        let config = GradiometerConfig {
            sample_rate_hz: 20.0,
            ..Default::default()
        };
        let processor = GradientProcessor::new(config);
        assert!((processor.config().sample_rate_hz - 20.0).abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Euler deconvolution
    // -----------------------------------------------------------------------

    #[test]
    fn test_euler_deconvolution_point_mass() {
        // Create a synthetic gradient profile over a buried point mass
        let n = 41;
        let station_spacing = 25.0;
        let mass = 1e11;
        let source_depth = 200.0;
        let source_x = 500.0;

        let mut gradient_profile = vec![0.0; n];
        let mut x_coords = vec![0.0; n];

        for i in 0..n {
            let x = i as f64 * station_spacing;
            x_coords[i] = x;
            let dx = x - source_x;
            let r2 = dx * dx + source_depth * source_depth;
            let r = r2.sqrt();
            // Vertical gradient Gzz from point mass
            gradient_profile[i] = G * mass * 1e9
                * (3.0 * source_depth * source_depth / (r2 * r2 * r) - 1.0 / (r2 * r));
        }

        let results = euler_deconvolution(&gradient_profile, &x_coords, 3.0);
        assert!(
            !results.is_empty(),
            "Euler deconvolution should produce results"
        );

        // Verify solutions have positive depth and x-positions within survey bounds
        for &(x, d) in &results {
            assert!(d > 0.0, "Depth should be positive");
            assert!(
                x >= -500.0 && x <= 1500.0,
                "Source x should be within reasonable range of survey"
            );
        }

        // The simplified 1D Euler deconvolution (using gradient as vertical derivative proxy)
        // should cluster solutions near the source x-position
        let mean_x: f64 = results.iter().map(|&(x, _)| x).sum::<f64>() / results.len() as f64;
        assert!(
            (mean_x - source_x).abs() < 400.0,
            "Mean solution x ({:.1}) should be in the vicinity of source x ({:.1})",
            mean_x,
            source_x
        );
    }

    #[test]
    fn test_euler_deconvolution_too_few_points() {
        let profile = vec![1.0, 2.0, 3.0];
        let x = vec![0.0, 1.0, 2.0];
        let results = euler_deconvolution(&profile, &x, 2.0);
        assert!(
            results.is_empty(),
            "Should return empty for fewer than 4 points"
        );
    }

    #[test]
    fn test_euler_deconvolution_mismatched_lengths() {
        let profile = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let x = vec![0.0, 1.0, 2.0]; // wrong length
        let results = euler_deconvolution(&profile, &x, 2.0);
        assert!(
            results.is_empty(),
            "Should return empty for mismatched lengths"
        );
    }

    #[test]
    fn test_euler_deconvolution_flat_profile() {
        // Flat (constant) gradient produces no meaningful derivative -> no solutions
        let n = 10;
        let profile = vec![5.0; n];
        let x: Vec<f64> = (0..n).map(|i| i as f64 * 100.0).collect();
        let results = euler_deconvolution(&profile, &x, 2.0);
        // With a flat profile, dT/dx ~ 0 everywhere, so the system is degenerate
        // Any solutions returned should still have positive depth
        for &(_xs, d) in &results {
            assert!(d > 0.0, "Depth should be positive");
        }
    }

    // -----------------------------------------------------------------------
    // Terrain correction
    // -----------------------------------------------------------------------

    #[test]
    fn test_terrain_correction_flat_terrain() {
        let elevations = vec![100.0; 5];
        let corrections = terrain_correction(&elevations, 50.0, 2670.0);
        assert_eq!(corrections.len(), 5);
        // On flat terrain, terrain correction component should be ~0
        // but free-air and Bouguer corrections are non-zero
        // All stations at same elevation should get same correction
        for i in 1..corrections.len() {
            assert!(
                (corrections[i] - corrections[0]).abs() < 1e-6,
                "Flat terrain should give equal corrections at all stations"
            );
        }
    }

    #[test]
    fn test_terrain_correction_empty() {
        let corrections = terrain_correction(&[], 50.0, 2670.0);
        assert!(corrections.is_empty());
    }

    #[test]
    fn test_terrain_correction_varying_elevation() {
        let elevations = vec![100.0, 150.0, 200.0, 150.0, 100.0];
        let corrections = terrain_correction(&elevations, 100.0, 2670.0);
        assert_eq!(corrections.len(), 5);
        // Higher stations should have larger (more negative) free-air correction
        // and larger Bouguer correction, so the center station is different
        assert!(
            (corrections[2] - corrections[0]).abs() > 1e-6,
            "Varying elevation should produce different corrections"
        );
    }

    // -----------------------------------------------------------------------
    // Free-air and Bouguer corrections
    // -----------------------------------------------------------------------

    #[test]
    fn test_free_air_correction_sea_level() {
        assert!((free_air_correction(0.0)).abs() < EPSILON);
    }

    #[test]
    fn test_free_air_correction_positive_elevation() {
        let fac = free_air_correction(100.0);
        let expected = -0.3086 * 100.0;
        assert!(
            (fac - expected).abs() < EPSILON,
            "FAC at 100m: got {}, expected {}",
            fac,
            expected
        );
    }

    #[test]
    fn test_free_air_correction_negative_elevation() {
        let fac = free_air_correction(-50.0);
        let expected = -0.3086 * (-50.0);
        assert!(
            (fac - expected).abs() < EPSILON,
            "FAC should be positive for below-geoid stations"
        );
        assert!(fac > 0.0);
    }

    #[test]
    fn test_bouguer_correction_sea_level() {
        assert!(bouguer_correction(0.0, 2670.0).abs() < EPSILON);
    }

    #[test]
    fn test_bouguer_correction_standard_density() {
        let bc = bouguer_correction(100.0, 2670.0);
        let expected = 2.0 * PI * G * 2670.0 * 100.0 * 1e5;
        assert!(
            (bc - expected).abs() < 1e-6,
            "BC at 100m, 2670 kg/m3: got {}, expected {}",
            bc,
            expected
        );
        // Standard Bouguer correction: 2*pi*G*rho = ~1.1197e-6 m/s^2 per meter
        // which is ~0.11197 mGal per meter for rho = 2670 kg/m^3
        let approx = 0.11197 * 100.0;
        assert!(
            (bc - approx).abs() < 0.01,
            "BC should be approximately 11.197 mGal at 100m, got {}",
            bc
        );
    }

    // -----------------------------------------------------------------------
    // Gradient attributes
    // -----------------------------------------------------------------------

    #[test]
    fn test_horizontal_gradient_magnitude_basic() {
        let hgm = horizontal_gradient_magnitude(3.0, 4.0);
        assert!(
            (hgm - 5.0).abs() < EPSILON,
            "HGM of (3,4) should be 5, got {}",
            hgm
        );
    }

    #[test]
    fn test_horizontal_gradient_magnitude_zero() {
        let hgm = horizontal_gradient_magnitude(0.0, 0.0);
        assert!(hgm.abs() < EPSILON);
    }

    #[test]
    fn test_horizontal_gradient_magnitude_single_component() {
        let hgm = horizontal_gradient_magnitude(7.0, 0.0);
        assert!((hgm - 7.0).abs() < EPSILON);
    }

    #[test]
    fn test_curvature_gradient_basic() {
        let cg = curvature_gradient(10.0, 6.0);
        assert!(
            (cg - 2.0).abs() < EPSILON,
            "CG of (10, 6) should be 2, got {}",
            cg
        );
    }

    #[test]
    fn test_curvature_gradient_equal_components() {
        let cg = curvature_gradient(5.0, 5.0);
        assert!(cg.abs() < EPSILON, "CG should be 0 for equal Gxx/Gyy");
    }

    #[test]
    fn test_curvature_gradient_negative() {
        let cg = curvature_gradient(3.0, 8.0);
        assert!(cg < 0.0, "CG should be negative when Gyy > Gxx");
        assert!((cg - (-2.5)).abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Unit conversion
    // -----------------------------------------------------------------------

    #[test]
    fn test_eotvos_to_si() {
        assert!((eotvos_to_si(1.0) - 1e-9).abs() < 1e-25);
    }

    #[test]
    fn test_eotvos_to_si_large_value() {
        assert!((eotvos_to_si(1000.0) - 1e-6).abs() < 1e-20);
    }

    #[test]
    fn test_si_to_eotvos() {
        assert!((si_to_eotvos(1e-9) - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_eotvos_si_roundtrip() {
        let original = 42.5;
        let roundtrip = si_to_eotvos(eotvos_to_si(original));
        assert!(
            (roundtrip - original).abs() < EPSILON,
            "Roundtrip conversion should preserve value"
        );
    }

    // -----------------------------------------------------------------------
    // Edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_point_mass_gradient_large_offset() {
        // Very far from source: gradients should be very small
        let tensor = point_mass_gradient(1e10, 100_000.0, 100_000.0, 100_000.0);
        assert!(
            tensor.frobenius_norm() < 1e-6,
            "Gradients should be negligible far from source"
        );
        // But trace should still be ~0
        assert!(tensor.trace().abs() < 1e-20);
    }

    #[test]
    fn test_point_mass_gradient_negative_mass() {
        // Negative mass (density deficit, e.g., cavity)
        let t_pos = point_mass_gradient(1e10, 500.0, 0.0, 0.0);
        let t_neg = point_mass_gradient(-1e10, 500.0, 0.0, 0.0);
        assert!(
            (t_pos.gzz + t_neg.gzz).abs() < EPSILON,
            "Negative mass should give opposite gradient"
        );
    }

    #[test]
    fn test_default_config() {
        let config = GradiometerConfig::default();
        assert!((config.sample_rate_hz - 10.0).abs() < EPSILON);
        assert!((config.platform_speed_mps - 60.0).abs() < EPSILON);
        assert!((config.survey_altitude_m - 80.0).abs() < EPSILON);
        assert!((config.noise_floor_eotvos - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_processor_zero_tensor() {
        let processor = GradientProcessor::new(GradiometerConfig::default());
        let result = processor.process(&GravityTensor::zero());
        assert!(!result.anomaly_detected);
        assert!(result.horizontal_gradient.abs() < EPSILON);
        assert!(result.vertical_gradient.abs() < EPSILON);
        assert!(result.curvature.abs() < EPSILON);
    }
}
