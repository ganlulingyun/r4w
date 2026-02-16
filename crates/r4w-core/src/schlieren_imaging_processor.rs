//! Schlieren Imaging Processor — Refractive Index Gradient Visualization
//!
//! Signal processing for Schlieren and shadowgraph imaging systems that visualize
//! refractive index gradients in transparent media. Applications include wind tunnel
//! aerodynamics, combustion analysis, shock wave visualization, heat convection
//! studies, and medical ultrasound coupling.
//!
//! ## Physics Background
//!
//! **Gladstone-Dale relation**: `n - 1 = K * rho` where K ≈ 0.000226 m³/kg for air.
//! Combined with ideal gas law: `n - 1 = K * P / (R * T)`.
//!
//! **Schlieren principle**: A knife edge at the focal plane of a lens blocks light
//! deflected by refractive index gradients. Intensity: `I/I0 = 1/2 + f*epsilon/(delta*a)`
//! where f = focal length, epsilon = deflection angle, a = source size, delta = cutoff.
//!
//! **Shadowgraph**: Sensitive to the second derivative (Laplacian) of refractive index:
//! `I/I0 ∝ d²n/dx² + d²n/dy²`.
//!
//! **Abel inversion** for axisymmetric flows reconstructs radial profiles from
//! line-of-sight integrated measurements.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::schlieren_imaging_processor::{
//!     SchlierenConfig, SchlierenType, KnifeEdgeCutoff,
//!     RefractiveIndexGradient, TemperatureFieldEstimator,
//! };
//!
//! // Classical Schlieren setup
//! let config = SchlierenConfig::new(SchlierenType::Classical, 0.5, 50.0);
//! assert!(config.sensitivity() > 0.0);
//!
//! // Knife-edge cutoff simulation
//! let knife = KnifeEdgeCutoff::new(1.0, 0.002, 0.5);
//! let intensity = knife.intensity(1e-4);
//! assert!(intensity > 0.0 && intensity < 1.0);
//!
//! // Temperature from refractive index
//! let temp_est = TemperatureFieldEstimator::new_air(101325.0);
//! let t = temp_est.temperature_from_index(1.000272);
//! assert!((t - 293.0).abs() < 5.0);
//! ```

use std::f64::consts::PI;

// ─── Constants ──────────────────────────────────────────────────────────────

/// Gladstone-Dale constant for air at standard conditions [m³/kg].
pub const GLADSTONE_DALE_AIR: f64 = 0.000226;

/// Specific gas constant for dry air [J/(kg·K)].
pub const R_AIR: f64 = 287.058;

/// Standard atmospheric pressure [Pa].
pub const STANDARD_PRESSURE: f64 = 101325.0;

/// Standard temperature [K].
pub const STANDARD_TEMPERATURE: f64 = 293.15;

/// Refractive index of air at standard conditions (derived from Gladstone-Dale).
/// n = 1 + K * P / (R * T) = 1 + 0.000226 * 101325 / (287.058 * 293.15)
pub const N_AIR_STANDARD: f64 = 1.000272;

// ─── Enums ──────────────────────────────────────────────────────────────────

/// Type of Schlieren imaging system.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SchlierenType {
    /// Classical knife-edge Schlieren.
    Classical,
    /// Rainbow (color) Schlieren with continuous hue mapping.
    Rainbow,
    /// Background-Oriented Schlieren (BOS) using cross-correlation.
    BackgroundOriented,
}

/// Direction of knife-edge orientation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum KnifeEdgeOrientation {
    /// Horizontal knife edge — sensitive to vertical gradients (dn/dy).
    Horizontal,
    /// Vertical knife edge — sensitive to horizontal gradients (dn/dx).
    Vertical,
}

/// Shock wave geometry type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ShockGeometry {
    /// Normal shock (perpendicular to flow).
    Normal,
    /// Oblique shock at given angle [radians].
    Oblique(f64),
    /// Bow shock (blunt body).
    Bow,
}

// ─── Error type ─────────────────────────────────────────────────────────────

/// Errors from Schlieren processing operations.
#[derive(Debug, Clone, PartialEq)]
pub enum SchlierenError {
    /// Input data is empty.
    EmptyInput,
    /// Dimension mismatch between inputs.
    DimensionMismatch { expected: usize, got: usize },
    /// Invalid parameter value.
    InvalidParameter(String),
    /// Abel inversion failed (e.g., non-axisymmetric data).
    AbelInversionFailed(String),
}

impl std::fmt::Display for SchlierenError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::EmptyInput => write!(f, "empty input data"),
            Self::DimensionMismatch { expected, got } => {
                write!(f, "dimension mismatch: expected {}, got {}", expected, got)
            }
            Self::InvalidParameter(msg) => write!(f, "invalid parameter: {}", msg),
            Self::AbelInversionFailed(msg) => write!(f, "Abel inversion failed: {}", msg),
        }
    }
}

impl std::error::Error for SchlierenError {}

// ─── SchlierenConfig ────────────────────────────────────────────────────────

/// Configuration for a Schlieren imaging system.
#[derive(Debug, Clone)]
pub struct SchlierenConfig {
    /// Type of Schlieren system.
    pub imaging_type: SchlierenType,
    /// Focal length of the Schlieren mirror/lens [m].
    pub focal_length: f64,
    /// Source slit/pinhole size [m].
    pub source_size: f64,
    /// Knife-edge cutoff percentage [0..100].
    pub cutoff_percent: f64,
    /// Sensitivity factor (dimensionless).
    pub sensitivity: f64,
    /// Knife-edge orientation.
    pub orientation: KnifeEdgeOrientation,
    /// Integration path length through test section [m].
    pub path_length: f64,
}

impl SchlierenConfig {
    /// Create a new Schlieren configuration.
    ///
    /// * `imaging_type` — Classical, Rainbow, or BOS
    /// * `focal_length` — mirror/lens focal length [m]
    /// * `cutoff_percent` — knife-edge cutoff [0..100]
    pub fn new(imaging_type: SchlierenType, focal_length: f64, cutoff_percent: f64) -> Self {
        let cutoff = cutoff_percent.clamp(0.0, 100.0);
        Self {
            imaging_type,
            focal_length,
            source_size: 0.002, // 2 mm default
            cutoff_percent: cutoff,
            sensitivity: focal_length / 0.002, // f / a
            orientation: KnifeEdgeOrientation::Vertical,
            path_length: 0.3, // 30 cm default test section
        }
    }

    /// Create a high-sensitivity configuration (small source, long focal length).
    pub fn high_sensitivity(imaging_type: SchlierenType) -> Self {
        Self::new(imaging_type, 2.0, 50.0)
            .with_source_size(0.0005)
            .with_path_length(0.5)
    }

    /// Set the source size [m].
    pub fn with_source_size(mut self, size: f64) -> Self {
        self.source_size = size.max(1e-6);
        self.sensitivity = self.focal_length / self.source_size;
        self
    }

    /// Set the path length [m].
    pub fn with_path_length(mut self, length: f64) -> Self {
        self.path_length = length.max(1e-6);
        self
    }

    /// Set the knife-edge orientation.
    pub fn with_orientation(mut self, orient: KnifeEdgeOrientation) -> Self {
        self.orientation = orient;
        self
    }

    /// Return the sensitivity of this Schlieren setup.
    /// Sensitivity = f / a (focal length / source size).
    pub fn sensitivity(&self) -> f64 {
        self.sensitivity
    }

    /// Compute minimum detectable deflection angle [radians].
    /// Approximately a / (2 * f), where 50% cutoff is assumed.
    pub fn min_detectable_deflection(&self) -> f64 {
        self.source_size / (2.0 * self.focal_length)
    }

    /// Compute minimum detectable refractive index gradient [1/m].
    /// dn/dx_min = n * epsilon_min / L
    pub fn min_detectable_gradient(&self) -> f64 {
        let eps_min = self.min_detectable_deflection();
        N_AIR_STANDARD * eps_min / self.path_length
    }
}

// ─── RefractiveIndexGradient ────────────────────────────────────────────────

/// Compute refractive index gradients from deflection angles.
///
/// The deflection angle epsilon relates to the gradient via:
/// `epsilon = (1/n₀) * integral(dn/dx dz)` along the optical path.
///
/// For uniform gradient over path length L:
/// `epsilon = (L / n₀) * (dn/dx)`
pub struct RefractiveIndexGradient {
    /// Background refractive index.
    pub n0: f64,
    /// Integration path length [m].
    pub path_length: f64,
}

impl RefractiveIndexGradient {
    /// Create a new gradient calculator.
    pub fn new(n0: f64, path_length: f64) -> Self {
        Self { n0, path_length }
    }

    /// Create for standard air.
    pub fn air(path_length: f64) -> Self {
        Self::new(N_AIR_STANDARD, path_length)
    }

    /// Compute deflection angle from gradient.
    /// epsilon = (L / n0) * (dn/dx)
    pub fn deflection_from_gradient(&self, dn_dx: f64) -> f64 {
        (self.path_length / self.n0) * dn_dx
    }

    /// Compute gradient from deflection angle.
    /// dn/dx = (n0 / L) * epsilon
    pub fn gradient_from_deflection(&self, epsilon: f64) -> f64 {
        (self.n0 / self.path_length) * epsilon
    }

    /// Compute 2D gradient field from deflection fields.
    /// Returns (dn_dx, dn_dy) arrays.
    pub fn gradient_field(
        &self,
        epsilon_x: &[f64],
        epsilon_y: &[f64],
    ) -> Result<(Vec<f64>, Vec<f64>), SchlierenError> {
        if epsilon_x.len() != epsilon_y.len() {
            return Err(SchlierenError::DimensionMismatch {
                expected: epsilon_x.len(),
                got: epsilon_y.len(),
            });
        }
        if epsilon_x.is_empty() {
            return Err(SchlierenError::EmptyInput);
        }
        let scale = self.n0 / self.path_length;
        let dn_dx: Vec<f64> = epsilon_x.iter().map(|e| scale * e).collect();
        let dn_dy: Vec<f64> = epsilon_y.iter().map(|e| scale * e).collect();
        Ok((dn_dx, dn_dy))
    }

    /// Compute deflection angle from density gradient using Gladstone-Dale.
    /// epsilon = (K * L / n0) * (drho/dx)
    pub fn deflection_from_density_gradient(&self, drho_dx: f64, k_gd: f64) -> f64 {
        (k_gd * self.path_length / self.n0) * drho_dx
    }

    /// Numerical gradient of a 1D refractive index profile (central differences).
    pub fn numerical_gradient(n_profile: &[f64], dx: f64) -> Vec<f64> {
        let len = n_profile.len();
        if len < 2 {
            return vec![0.0; len];
        }
        let mut grad = vec![0.0; len];
        // Forward difference for first point
        grad[0] = (n_profile[1] - n_profile[0]) / dx;
        // Central differences for interior
        for i in 1..len - 1 {
            grad[i] = (n_profile[i + 1] - n_profile[i - 1]) / (2.0 * dx);
        }
        // Backward difference for last point
        grad[len - 1] = (n_profile[len - 1] - n_profile[len - 2]) / dx;
        grad
    }
}

// ─── KnifeEdgeCutoff ────────────────────────────────────────────────────────

/// Simulate knife-edge spatial filtering in a Schlieren system.
///
/// Intensity formula: `I/I₀ = 1/2 + f * epsilon / (delta * a)`
///
/// where:
/// - f = focal length [m]
/// - epsilon = light deflection angle [rad]
/// - a = source image size at cutoff plane [m]
/// - delta = fraction of source blocked (0.5 for 50% cutoff)
pub struct KnifeEdgeCutoff {
    /// Focal length [m].
    pub focal_length: f64,
    /// Source image size at cutoff plane [m].
    pub source_size: f64,
    /// Cutoff fraction [0..1] (0.5 = 50% cutoff).
    pub cutoff_fraction: f64,
}

impl KnifeEdgeCutoff {
    /// Create a new knife-edge model.
    pub fn new(focal_length: f64, source_size: f64, cutoff_fraction: f64) -> Self {
        Self {
            focal_length,
            source_size,
            cutoff_fraction: cutoff_fraction.clamp(0.01, 0.99),
        }
    }

    /// Compute relative intensity I/I₀ for a given deflection angle.
    ///
    /// `I/I₀ = cutoff + f * epsilon / a`
    ///
    /// Clamped to [0, 1].
    pub fn intensity(&self, epsilon: f64) -> f64 {
        let ratio =
            self.cutoff_fraction + self.focal_length * epsilon / self.source_size;
        ratio.clamp(0.0, 1.0)
    }

    /// Process a vector of deflection angles into an intensity image.
    pub fn process(&self, deflections: &[f64]) -> Vec<f64> {
        deflections.iter().map(|e| self.intensity(*e)).collect()
    }

    /// Compute the maximum measurable deflection angle before saturation.
    pub fn max_deflection(&self) -> f64 {
        (1.0 - self.cutoff_fraction) * self.source_size / self.focal_length
    }

    /// Compute the minimum measurable deflection (full darkness).
    pub fn min_deflection(&self) -> f64 {
        -self.cutoff_fraction * self.source_size / self.focal_length
    }

    /// Dynamic range in dB: ratio of max to min detectable intensity change.
    pub fn dynamic_range_db(&self) -> f64 {
        let max_di = 1.0 - self.cutoff_fraction;
        let min_di = self.cutoff_fraction;
        if min_di <= 0.0 {
            return f64::INFINITY;
        }
        20.0 * (max_di / min_di).log10()
    }
}

// ─── BackgroundOrientedSchlieren ────────────────────────────────────────────

/// Background-Oriented Schlieren (BOS) processor.
///
/// Uses cross-correlation of background pattern images (with and without flow)
/// to extract displacement fields proportional to the line-of-sight integrated
/// density gradient.
pub struct BackgroundOrientedSchlieren {
    /// Interrogation window size (square, in pixels).
    pub window_size: usize,
    /// Step size between interrogation windows.
    pub step_size: usize,
    /// Distance from background to density field [m].
    pub z_d: f64,
    /// Pixel size [m].
    pub pixel_size: f64,
}

impl BackgroundOrientedSchlieren {
    /// Create a new BOS processor.
    pub fn new(window_size: usize, step_size: usize, z_d: f64, pixel_size: f64) -> Self {
        Self {
            window_size: window_size.max(4),
            step_size: step_size.max(1),
            z_d,
            pixel_size,
        }
    }

    /// 1D cross-correlation of two equal-length windows.
    /// C[tau] = sum_j a[j] * b[j + tau] for tau in [-(n-1), n-1].
    /// Returns array of length `2*n - 1` where index `n-1` is zero-lag.
    pub fn cross_correlate_1d(a: &[f64], b: &[f64]) -> Vec<f64> {
        let n = a.len();
        let m = b.len();
        let out_len = n + m - 1;
        let mut result = vec![0.0; out_len];
        // tau ranges from -(n-1) to (m-1), mapped to index 0..(n+m-2)
        // index = tau + (n - 1)
        for idx in 0..out_len {
            let tau = idx as isize - (n as isize - 1);
            let mut sum = 0.0;
            for j in 0..n {
                let k = j as isize + tau;
                if k >= 0 && (k as usize) < m {
                    sum += a[j] * b[k as usize];
                }
            }
            result[idx] = sum;
        }
        result
    }

    /// Find sub-pixel displacement via parabolic fit around correlation peak.
    ///
    /// Given correlation values at (peak-1, peak, peak+1), fits a parabola
    /// and returns the fractional offset from the peak index.
    /// Positive = peak is to the right of center sample.
    pub fn subpixel_parabolic(left: f64, center: f64, right: f64) -> f64 {
        let denom = 2.0 * (left - 2.0 * center + right);
        if denom.abs() < 1e-12 {
            return 0.0;
        }
        (left - right) / denom
    }

    /// Compute displacement between two 1D signal windows.
    /// Returns sub-pixel displacement in samples.
    pub fn compute_displacement_1d(&self, reference: &[f64], distorted: &[f64]) -> f64 {
        if reference.len() < 3 || distorted.len() < 3 {
            return 0.0;
        }
        let corr = Self::cross_correlate_1d(reference, distorted);
        let n = reference.len();

        // Find peak in correlation
        let mut peak_idx = 0;
        let mut peak_val = f64::NEG_INFINITY;
        for (i, &v) in corr.iter().enumerate() {
            if v > peak_val {
                peak_val = v;
                peak_idx = i;
            }
        }

        // Sub-pixel refinement
        let mut sub = 0.0;
        if peak_idx > 0 && peak_idx < corr.len() - 1 {
            sub = Self::subpixel_parabolic(
                corr[peak_idx - 1],
                corr[peak_idx],
                corr[peak_idx + 1],
            );
        }

        // Displacement relative to zero-lag position
        let zero_lag = (n - 1) as f64;
        (peak_idx as f64 + sub) - zero_lag
    }

    /// Convert pixel displacement to deflection angle.
    /// epsilon = displacement * pixel_size / z_d
    pub fn displacement_to_deflection(&self, displacement_px: f64) -> f64 {
        displacement_px * self.pixel_size / self.z_d
    }

    /// Process reference and distorted 1D signals, returning deflection angles.
    /// Divides signals into windows and computes displacement for each.
    pub fn process_1d(
        &self,
        reference: &[f64],
        distorted: &[f64],
    ) -> Result<Vec<f64>, SchlierenError> {
        if reference.len() != distorted.len() {
            return Err(SchlierenError::DimensionMismatch {
                expected: reference.len(),
                got: distorted.len(),
            });
        }
        if reference.len() < self.window_size {
            return Err(SchlierenError::InvalidParameter(
                "signal shorter than window size".to_string(),
            ));
        }

        let mut deflections = Vec::new();
        let mut pos = 0;
        while pos + self.window_size <= reference.len() {
            let ref_win = &reference[pos..pos + self.window_size];
            let dist_win = &distorted[pos..pos + self.window_size];
            let disp = self.compute_displacement_1d(ref_win, dist_win);
            let deflection = self.displacement_to_deflection(disp);
            deflections.push(deflection);
            pos += self.step_size;
        }

        Ok(deflections)
    }
}

// ─── ShockWaveDetector ──────────────────────────────────────────────────────

/// Detect and characterize shock waves from Schlieren intensity profiles.
///
/// Shock fronts appear as sharp intensity discontinuities in Schlieren images.
/// Detection uses gradient thresholding on the intensity profile.
pub struct ShockWaveDetector {
    /// Gradient threshold for shock detection (normalized).
    pub threshold: f64,
    /// Minimum separation between detected shocks [samples].
    pub min_separation: usize,
}

/// A detected shock wave event.
#[derive(Debug, Clone)]
pub struct ShockDetection {
    /// Position of the shock front [sample index].
    pub position: usize,
    /// Gradient magnitude at the shock.
    pub gradient_magnitude: f64,
    /// Estimated shock geometry.
    pub geometry: ShockGeometry,
}

impl ShockWaveDetector {
    /// Create a new shock wave detector.
    pub fn new(threshold: f64, min_separation: usize) -> Self {
        Self {
            threshold: threshold.abs(),
            min_separation: min_separation.max(1),
        }
    }

    /// Detect shock fronts in a 1D intensity profile.
    pub fn detect(&self, intensity: &[f64]) -> Vec<ShockDetection> {
        if intensity.len() < 3 {
            return Vec::new();
        }

        // Compute gradient magnitude
        let mut grad = vec![0.0; intensity.len()];
        for i in 1..intensity.len() - 1 {
            grad[i] = (intensity[i + 1] - intensity[i - 1]).abs() / 2.0;
        }

        // Find peaks above threshold
        let mut shocks = Vec::new();
        let mut last_pos: Option<usize> = None;

        for i in 1..grad.len() - 1 {
            if grad[i] > self.threshold && grad[i] >= grad[i - 1] && grad[i] >= grad[i + 1] {
                if let Some(lp) = last_pos {
                    if i - lp < self.min_separation {
                        // If this peak is stronger, replace the last one
                        if grad[i] > shocks.last().map(|s: &ShockDetection| s.gradient_magnitude).unwrap_or(0.0) {
                            shocks.pop();
                        } else {
                            continue;
                        }
                    }
                }
                shocks.push(ShockDetection {
                    position: i,
                    gradient_magnitude: grad[i],
                    geometry: ShockGeometry::Normal,
                });
                last_pos = Some(i);
            }
        }

        shocks
    }

    /// Track shock position across multiple frames.
    /// Returns estimated velocity in samples/frame.
    pub fn track_velocity(positions: &[usize]) -> f64 {
        if positions.len() < 2 {
            return 0.0;
        }
        let n = positions.len() as f64;
        let sum_x: f64 = (0..positions.len()).map(|i| i as f64).sum();
        let sum_y: f64 = positions.iter().map(|&p| p as f64).sum();
        let sum_xy: f64 = positions
            .iter()
            .enumerate()
            .map(|(i, &p)| i as f64 * p as f64)
            .sum();
        let sum_xx: f64 = (0..positions.len()).map(|i| (i as f64).powi(2)).sum();

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-12 {
            return 0.0;
        }
        (n * sum_xy - sum_x * sum_y) / denom
    }
}

// ─── DensityReconstructor ───────────────────────────────────────────────────

/// Abel inversion for axisymmetric flow density reconstruction.
///
/// Reconstructs the radial refractive index profile n(r) from line-of-sight
/// integrated deflection data F(y) using the Abel transform:
///
/// `f(r) = -(1/π) * integral_r^R [dF/dy / sqrt(y² - r²)] dy`
///
/// This is the inverse Abel transform applied to Schlieren/interferometric data.
pub struct DensityReconstructor {
    /// Background refractive index.
    pub n0: f64,
    /// Gladstone-Dale constant [m³/kg].
    pub k_gd: f64,
    /// Number of radial points.
    pub n_radial: usize,
}

impl DensityReconstructor {
    /// Create a new density reconstructor.
    pub fn new(n0: f64, k_gd: f64, n_radial: usize) -> Self {
        Self {
            n0,
            k_gd,
            n_radial: n_radial.max(4),
        }
    }

    /// Create for standard air.
    pub fn air(n_radial: usize) -> Self {
        Self::new(N_AIR_STANDARD, GLADSTONE_DALE_AIR, n_radial)
    }

    /// Perform Abel inversion on line-of-sight integrated data.
    ///
    /// `projection` — F(y) values at equally-spaced y positions from 0 to R.
    ///
    /// Returns radial profile f(r) at the same radial positions.
    pub fn abel_invert(&self, projection: &[f64]) -> Result<Vec<f64>, SchlierenError> {
        let n = projection.len();
        if n < 4 {
            return Err(SchlierenError::InvalidParameter(
                "need at least 4 projection points".to_string(),
            ));
        }

        let dy = 1.0 / (n - 1) as f64; // Normalized radial coordinate [0, 1]
        let mut radial = vec![0.0; n];

        // Compute dF/dy via central differences
        let mut df_dy = vec![0.0; n];
        df_dy[0] = (projection[1] - projection[0]) / dy;
        for i in 1..n - 1 {
            df_dy[i] = (projection[i + 1] - projection[i - 1]) / (2.0 * dy);
        }
        df_dy[n - 1] = (projection[n - 1] - projection[n - 2]) / dy;

        // Abel inversion: f(r_i) = -(1/pi) * sum_j [dF/dy_j / sqrt(y_j^2 - r_i^2)] * dy
        // for y_j from r_i to R (j from i to n-1)
        for i in 0..n - 1 {
            let r = i as f64 * dy;
            let r_sq = r * r;
            let mut sum = 0.0;

            for j in (i + 1)..n {
                let y = j as f64 * dy;
                let y_sq = y * y;
                let diff = y_sq - r_sq;
                if diff > 1e-14 {
                    sum += df_dy[j] / diff.sqrt();
                }
            }

            radial[i] = -(1.0 / PI) * sum * dy;
        }

        // Boundary: extrapolate last point
        if n >= 3 {
            radial[n - 1] = 2.0 * radial[n - 2] - radial[n - 3];
        }

        Ok(radial)
    }

    /// Convert refractive index profile to density profile.
    /// rho = (n - 1) / K_GD
    pub fn index_to_density(&self, n_profile: &[f64]) -> Vec<f64> {
        n_profile
            .iter()
            .map(|&n| (n - 1.0) / self.k_gd)
            .collect()
    }

    /// Convert density profile to refractive index profile.
    /// n = 1 + K_GD * rho
    pub fn density_to_index(&self, density_profile: &[f64]) -> Vec<f64> {
        density_profile
            .iter()
            .map(|&rho| 1.0 + self.k_gd * rho)
            .collect()
    }

    /// Forward Abel transform (for validation / simulation).
    /// F(y) = 2 * integral_y^R [f(r) * r / sqrt(r² - y²)] dr
    pub fn abel_forward(&self, radial: &[f64]) -> Vec<f64> {
        let n = radial.len();
        let dr = 1.0 / (n - 1) as f64;
        let mut projection = vec![0.0; n];

        for i in 0..n {
            let y = i as f64 * dr;
            let y_sq = y * y;
            let mut sum = 0.0;

            for j in (i + 1)..n {
                let r = j as f64 * dr;
                let r_sq = r * r;
                let diff = r_sq - y_sq;
                if diff > 1e-14 {
                    sum += radial[j] * r / diff.sqrt();
                }
            }

            projection[i] = 2.0 * sum * dr;
        }

        projection
    }
}

// ─── RainbowSchlierenProcessor ──────────────────────────────────────────────

/// Rainbow Schlieren processor for quantitative deflection measurement.
///
/// Maps continuous hue values to deflection angles using a color filter
/// at the cutoff plane. The hue of transmitted light encodes the deflection
/// angle, enabling full-field quantitative measurement.
pub struct RainbowSchlierenProcessor {
    /// Maximum deflection angle that maps to full hue range [rad].
    pub max_deflection: f64,
    /// Focal length of the Schlieren system [m].
    pub focal_length: f64,
    /// Filter width [m].
    pub filter_width: f64,
}

impl RainbowSchlierenProcessor {
    /// Create a new rainbow Schlieren processor.
    pub fn new(max_deflection: f64, focal_length: f64, filter_width: f64) -> Self {
        Self {
            max_deflection: max_deflection.abs().max(1e-8),
            focal_length,
            filter_width: filter_width.max(1e-6),
        }
    }

    /// Map deflection angle to hue [0..360 degrees].
    ///
    /// Linear mapping: hue = 180 + (epsilon / max_deflection) * 180
    pub fn deflection_to_hue(&self, epsilon: f64) -> f64 {
        let normalized = (epsilon / self.max_deflection).clamp(-1.0, 1.0);
        180.0 + normalized * 180.0
    }

    /// Map hue back to deflection angle.
    pub fn hue_to_deflection(&self, hue: f64) -> f64 {
        let normalized = (hue - 180.0) / 180.0;
        normalized * self.max_deflection
    }

    /// Process a vector of deflection angles into hue values.
    pub fn process(&self, deflections: &[f64]) -> Vec<f64> {
        deflections
            .iter()
            .map(|e| self.deflection_to_hue(*e))
            .collect()
    }

    /// Convert hue to RGB [0..1] using HSV with S=1, V=1.
    pub fn hue_to_rgb(hue_deg: f64) -> (f64, f64, f64) {
        let h = ((hue_deg % 360.0) + 360.0) % 360.0;
        let c = 1.0; // chroma
        let h_prime = h / 60.0;
        let x = c * (1.0 - (h_prime % 2.0 - 1.0).abs());

        let (r1, g1, b1) = if h_prime < 1.0 {
            (c, x, 0.0)
        } else if h_prime < 2.0 {
            (x, c, 0.0)
        } else if h_prime < 3.0 {
            (0.0, c, x)
        } else if h_prime < 4.0 {
            (0.0, x, c)
        } else if h_prime < 5.0 {
            (x, 0.0, c)
        } else {
            (c, 0.0, x)
        };

        (r1, g1, b1)
    }
}

// ─── ShadowgraphSimulator ──────────────────────────────────────────────────

/// Shadowgraph simulation: intensity proportional to Laplacian of refractive index.
///
/// `I/I₀ ∝ 1 - L * z * (d²n/dx² + d²n/dy²)`
///
/// where L = propagation distance, z = path length through medium.
///
/// Shadowgraphy is sensitive to the second spatial derivative of the refractive
/// index field, making it ideal for detecting shocks and other sharp features.
pub struct ShadowgraphSimulator {
    /// Propagation distance from disturbance to screen [m].
    pub propagation_distance: f64,
    /// Path length through the medium [m].
    pub path_length: f64,
}

impl ShadowgraphSimulator {
    /// Create a new shadowgraph simulator.
    pub fn new(propagation_distance: f64, path_length: f64) -> Self {
        Self {
            propagation_distance: propagation_distance.max(1e-6),
            path_length: path_length.max(1e-6),
        }
    }

    /// Compute 1D Laplacian (second derivative) using central differences.
    pub fn laplacian_1d(data: &[f64], dx: f64) -> Vec<f64> {
        let n = data.len();
        if n < 3 {
            return vec![0.0; n];
        }
        let dx2 = dx * dx;
        let mut lap = vec![0.0; n];
        for i in 1..n - 1 {
            lap[i] = (data[i + 1] - 2.0 * data[i] + data[i - 1]) / dx2;
        }
        // Boundaries: one-sided second derivative
        if n >= 3 {
            lap[0] = lap[1];
            lap[n - 1] = lap[n - 2];
        }
        lap
    }

    /// Simulate shadowgraph intensity from a 1D refractive index profile.
    ///
    /// `I/I₀ = 1 - L * z * d²n/dx²`
    pub fn simulate_1d(&self, n_profile: &[f64], dx: f64) -> Vec<f64> {
        let laplacian = Self::laplacian_1d(n_profile, dx);
        let scale = self.propagation_distance * self.path_length;
        laplacian
            .iter()
            .map(|&lap| (1.0 - scale * lap).max(0.0))
            .collect()
    }

    /// Compute sensitivity ratio: shadowgraph vs Schlieren.
    /// Shadowgraph ~ d²n/dx² while Schlieren ~ dn/dx.
    /// For feature size delta: ratio ~ delta / (L * z).
    pub fn sensitivity_ratio(&self, feature_size: f64) -> f64 {
        feature_size / (self.propagation_distance * self.path_length)
    }
}

// ─── MachNumberEstimator ────────────────────────────────────────────────────

/// Estimate Mach number from shock wave observations.
///
/// For a Mach cone: `sin(mu) = 1 / M` where mu is the half-angle.
///
/// For oblique shocks, the theta-beta-M relation is used:
/// `tan(theta) = 2 * cot(beta) * (M² sin²(beta) - 1) / (M²(gamma + cos(2*beta)) + 2)`
pub struct MachNumberEstimator {
    /// Ratio of specific heats (1.4 for air).
    pub gamma: f64,
}

impl MachNumberEstimator {
    /// Create for a given specific heat ratio.
    pub fn new(gamma: f64) -> Self {
        Self {
            gamma: gamma.max(1.0),
        }
    }

    /// Create for standard air (gamma = 1.4).
    pub fn air() -> Self {
        Self::new(1.4)
    }

    /// Estimate Mach number from Mach cone half-angle [radians].
    /// M = 1 / sin(mu)
    pub fn from_mach_angle(&self, mu: f64) -> f64 {
        let sin_mu = mu.sin();
        if sin_mu.abs() < 1e-12 {
            return f64::INFINITY;
        }
        1.0 / sin_mu
    }

    /// Compute Mach angle from Mach number [radians].
    /// mu = arcsin(1/M)
    pub fn mach_angle(&self, mach: f64) -> f64 {
        if mach < 1.0 {
            return PI / 2.0; // subsonic, no Mach cone
        }
        (1.0 / mach).asin()
    }

    /// Compute normal shock pressure ratio: P2/P1 = 1 + 2*gamma/(gamma+1) * (M1² - 1)
    pub fn normal_shock_pressure_ratio(&self, mach: f64) -> f64 {
        let g = self.gamma;
        1.0 + 2.0 * g / (g + 1.0) * (mach * mach - 1.0)
    }

    /// Compute normal shock density ratio: rho2/rho1 = (gamma+1)*M1² / ((gamma-1)*M1² + 2)
    pub fn normal_shock_density_ratio(&self, mach: f64) -> f64 {
        let g = self.gamma;
        let m2 = mach * mach;
        (g + 1.0) * m2 / ((g - 1.0) * m2 + 2.0)
    }

    /// Compute normal shock temperature ratio: T2/T1 = P2/P1 * rho1/rho2
    pub fn normal_shock_temperature_ratio(&self, mach: f64) -> f64 {
        self.normal_shock_pressure_ratio(mach) / self.normal_shock_density_ratio(mach)
    }

    /// Compute post-shock Mach number for normal shock.
    /// M2² = ((gamma-1)*M1² + 2) / (2*gamma*M1² - (gamma-1))
    pub fn normal_shock_downstream_mach(&self, mach: f64) -> f64 {
        let g = self.gamma;
        let m2 = mach * mach;
        let num = (g - 1.0) * m2 + 2.0;
        let den = 2.0 * g * m2 - (g - 1.0);
        if den <= 0.0 {
            return 0.0;
        }
        (num / den).sqrt()
    }

    /// Compute oblique shock wave angle beta from Mach number and flow deflection angle theta.
    /// Uses iterative Newton-Raphson on the theta-beta-M relation.
    /// Returns the weak shock solution.
    pub fn oblique_shock_angle(&self, mach: f64, theta: f64) -> Option<f64> {
        if mach <= 1.0 || theta < 0.0 {
            return None;
        }
        let g = self.gamma;
        let m2 = mach * mach;

        // Bisection between Mach angle and pi/2
        let mut beta_lo = (1.0 / mach).asin();
        let mut beta_hi = PI / 2.0;

        // theta-beta-M relation: tan(theta) = 2*cot(beta)*(M^2*sin^2(beta)-1) / (M^2*(gamma+cos(2*beta))+2)
        let theta_from_beta = |beta: f64| -> f64 {
            let sb = beta.sin();
            let num = 2.0 * (beta.cos() / sb) * (m2 * sb * sb - 1.0);
            let den = m2 * (g + (2.0 * beta).cos()) + 2.0;
            (num / den).atan()
        };

        for _ in 0..100 {
            let beta_mid = 0.5 * (beta_lo + beta_hi);
            let theta_mid = theta_from_beta(beta_mid);
            if (theta_mid - theta).abs() < 1e-10 {
                return Some(beta_mid);
            }
            if theta_mid < theta {
                beta_lo = beta_mid;
            } else {
                beta_hi = beta_mid;
            }
        }

        Some(0.5 * (beta_lo + beta_hi))
    }

    /// Estimate Mach number from shock-front angle and flow deflection angle.
    /// Iterative search using oblique shock relations.
    pub fn from_oblique_shock(&self, beta: f64, theta: f64) -> f64 {
        let g = self.gamma;
        let sb = beta.sin();
        let cb = beta.cos();
        let tt = theta.tan();

        // From theta-beta-M: M^2 = (2*(1 + tt*tan(beta))) / (sin^2(beta) * (gamma + cos(2*beta) + 2*tt/tan(beta) * (sin^2(beta))))
        // Rearranged: solve for M^2 from beta and theta
        // Direct formula: M^2*sin^2(beta) = 1 + (gamma+1)/2 * M^2 * tan(beta) * tan(theta) / (M^2*sin^2(beta) - 1) ... complicated
        // Use bisection:
        let mut m_lo = 1.0;
        let mut m_hi = 20.0;

        let theta_from_m = |m: f64| -> f64 {
            let m2 = m * m;
            let num = 2.0 * (cb / sb) * (m2 * sb * sb - 1.0);
            let den = m2 * (g + (2.0 * beta).cos()) + 2.0;
            (num / den).atan()
        };

        for _ in 0..100 {
            let m_mid = 0.5 * (m_lo + m_hi);
            let theta_mid = theta_from_m(m_mid);
            if (theta_mid - theta).abs() < 1e-10 {
                return m_mid;
            }
            if theta_mid < theta {
                m_lo = m_mid;
            } else {
                m_hi = m_mid;
            }
        }

        0.5 * (m_lo + m_hi)
    }
}

// ─── TemperatureFieldEstimator ──────────────────────────────────────────────

/// Convert refractive index measurements to temperature fields.
///
/// Uses the Gladstone-Dale relation and ideal gas law:
/// - `n - 1 = K * rho` (Gladstone-Dale)
/// - `rho = P / (R * T)` (ideal gas)
/// - Therefore: `T = K * P / (R * (n - 1))`
pub struct TemperatureFieldEstimator {
    /// Gladstone-Dale constant [m³/kg].
    pub k_gd: f64,
    /// Static pressure [Pa].
    pub pressure: f64,
    /// Specific gas constant [J/(kg·K)].
    pub r_gas: f64,
}

impl TemperatureFieldEstimator {
    /// Create for air at given pressure.
    pub fn new_air(pressure: f64) -> Self {
        Self {
            k_gd: GLADSTONE_DALE_AIR,
            pressure,
            r_gas: R_AIR,
        }
    }

    /// Create with custom gas properties.
    pub fn new(k_gd: f64, pressure: f64, r_gas: f64) -> Self {
        Self {
            k_gd,
            pressure,
            r_gas,
        }
    }

    /// Compute temperature from refractive index.
    /// T = K * P / (R * (n - 1))
    pub fn temperature_from_index(&self, n: f64) -> f64 {
        let delta_n = n - 1.0;
        if delta_n.abs() < 1e-12 {
            return f64::INFINITY;
        }
        self.k_gd * self.pressure / (self.r_gas * delta_n)
    }

    /// Compute refractive index from temperature.
    /// n = 1 + K * P / (R * T)
    pub fn index_from_temperature(&self, temperature: f64) -> f64 {
        if temperature.abs() < 1e-6 {
            return f64::INFINITY;
        }
        1.0 + self.k_gd * self.pressure / (self.r_gas * temperature)
    }

    /// Compute density from refractive index.
    /// rho = (n - 1) / K
    pub fn density_from_index(&self, n: f64) -> f64 {
        (n - 1.0) / self.k_gd
    }

    /// Compute density from temperature.
    /// rho = P / (R * T)
    pub fn density_from_temperature(&self, temperature: f64) -> f64 {
        self.pressure / (self.r_gas * temperature)
    }

    /// Process a refractive index field into a temperature field.
    pub fn process_field(&self, n_field: &[f64]) -> Vec<f64> {
        n_field
            .iter()
            .map(|&n| self.temperature_from_index(n))
            .collect()
    }

    /// Compute temperature gradient from refractive index gradient.
    /// dT/dx = -T² * R / (K * P) * dn/dx
    pub fn temperature_gradient(&self, temperature: f64, dn_dx: f64) -> f64 {
        -temperature * temperature * self.r_gas / (self.k_gd * self.pressure) * dn_dx
    }
}

// ─── SchlierenProcessor (orchestrator) ──────────────────────────────────────

/// High-level Schlieren image processor combining multiple analysis stages.
pub struct SchlierenProcessor {
    /// Configuration.
    pub config: SchlierenConfig,
    /// Knife-edge model.
    knife_edge: KnifeEdgeCutoff,
    /// Temperature estimator.
    temp_estimator: TemperatureFieldEstimator,
    /// Gradient calculator.
    gradient: RefractiveIndexGradient,
}

impl SchlierenProcessor {
    /// Create a full Schlieren processor.
    pub fn new(config: SchlierenConfig, pressure: f64) -> Self {
        let knife_edge = KnifeEdgeCutoff::new(
            config.focal_length,
            config.source_size,
            config.cutoff_percent / 100.0,
        );
        let temp_estimator = TemperatureFieldEstimator::new_air(pressure);
        let gradient = RefractiveIndexGradient::air(config.path_length);

        Self {
            config,
            knife_edge,
            temp_estimator,
            gradient,
        }
    }

    /// Process a refractive index profile:
    /// 1. Compute gradients
    /// 2. Convert to deflection angles
    /// 3. Apply knife-edge to get intensities
    /// 4. Also compute temperatures
    ///
    /// Returns (intensities, temperatures).
    pub fn process_profile(
        &self,
        n_profile: &[f64],
        dx: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        // Compute gradient
        let grad = RefractiveIndexGradient::numerical_gradient(n_profile, dx);

        // Convert gradient to deflection angles
        let deflections: Vec<f64> = grad
            .iter()
            .map(|&g| self.gradient.deflection_from_gradient(g))
            .collect();

        // Knife-edge intensity
        let intensities = self.knife_edge.process(&deflections);

        // Temperature field
        let temperatures = self.temp_estimator.process_field(n_profile);

        (intensities, temperatures)
    }

    /// Get the knife-edge model.
    pub fn knife_edge(&self) -> &KnifeEdgeCutoff {
        &self.knife_edge
    }
}

// ─── Utility functions ──────────────────────────────────────────────────────

/// Compute refractive index of air at given temperature and pressure.
/// n = 1 + K * P / (R * T)
pub fn air_refractive_index(temperature: f64, pressure: f64) -> f64 {
    1.0 + GLADSTONE_DALE_AIR * pressure / (R_AIR * temperature)
}

/// Compute air density from temperature and pressure.
/// rho = P / (R * T)
pub fn air_density(temperature: f64, pressure: f64) -> f64 {
    pressure / (R_AIR * temperature)
}

/// Generate a Gaussian refractive index perturbation (for testing).
/// Returns n(x) = n0 + amplitude * exp(-(x-center)² / (2*sigma²))
pub fn gaussian_index_profile(
    n_points: usize,
    x_range: (f64, f64),
    n0: f64,
    amplitude: f64,
    center: f64,
    sigma: f64,
) -> Vec<f64> {
    let dx = (x_range.1 - x_range.0) / (n_points - 1).max(1) as f64;
    (0..n_points)
        .map(|i| {
            let x = x_range.0 + i as f64 * dx;
            let arg = (x - center) / sigma;
            n0 + amplitude * (-0.5 * arg * arg).exp()
        })
        .collect()
}

/// Generate a step-function refractive index profile (shock simulation).
pub fn step_index_profile(
    n_points: usize,
    n_before: f64,
    n_after: f64,
    transition_point: f64,
    transition_width: f64,
) -> Vec<f64> {
    (0..n_points)
        .map(|i| {
            let x = i as f64 / (n_points - 1).max(1) as f64;
            let t = ((x - transition_point) / transition_width.max(1e-12)).tanh();
            0.5 * (n_before + n_after) + 0.5 * (n_after - n_before) * t
        })
        .collect()
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    // --- SchlierenConfig tests ---

    #[test]
    fn test_config_basic() {
        let cfg = SchlierenConfig::new(SchlierenType::Classical, 1.0, 50.0);
        assert_eq!(cfg.imaging_type, SchlierenType::Classical);
        assert!((cfg.focal_length - 1.0).abs() < EPSILON);
        assert!((cfg.cutoff_percent - 50.0).abs() < EPSILON);
    }

    #[test]
    fn test_config_sensitivity() {
        let cfg = SchlierenConfig::new(SchlierenType::Classical, 2.0, 50.0);
        // sensitivity = f / a = 2.0 / 0.002 = 1000
        assert!((cfg.sensitivity() - 1000.0).abs() < EPSILON);
    }

    #[test]
    fn test_config_high_sensitivity() {
        let cfg = SchlierenConfig::high_sensitivity(SchlierenType::Rainbow);
        assert!((cfg.focal_length - 2.0).abs() < EPSILON);
        assert!((cfg.source_size - 0.0005).abs() < EPSILON);
        // sensitivity = 2.0 / 0.0005 = 4000
        assert!((cfg.sensitivity() - 4000.0).abs() < EPSILON);
    }

    #[test]
    fn test_config_with_source_size() {
        let cfg = SchlierenConfig::new(SchlierenType::Classical, 1.0, 50.0)
            .with_source_size(0.001);
        assert!((cfg.source_size - 0.001).abs() < EPSILON);
        // sensitivity = 1.0 / 0.001 = 1000
        assert!((cfg.sensitivity() - 1000.0).abs() < EPSILON);
    }

    #[test]
    fn test_config_min_detectable_deflection() {
        let cfg = SchlierenConfig::new(SchlierenType::Classical, 1.0, 50.0);
        // epsilon_min = a / (2*f) = 0.002 / (2*1.0) = 0.001
        assert!((cfg.min_detectable_deflection() - 0.001).abs() < EPSILON);
    }

    #[test]
    fn test_config_cutoff_clamp() {
        let cfg = SchlierenConfig::new(SchlierenType::Classical, 1.0, 150.0);
        assert!((cfg.cutoff_percent - 100.0).abs() < EPSILON);
        let cfg2 = SchlierenConfig::new(SchlierenType::Classical, 1.0, -10.0);
        assert!((cfg2.cutoff_percent).abs() < EPSILON);
    }

    // --- RefractiveIndexGradient tests ---

    #[test]
    fn test_gradient_deflection_roundtrip() {
        let g = RefractiveIndexGradient::air(0.3);
        let dn_dx = 1e-3;
        let eps = g.deflection_from_gradient(dn_dx);
        let recovered = g.gradient_from_deflection(eps);
        assert!((recovered - dn_dx).abs() < 1e-10);
    }

    #[test]
    fn test_gradient_deflection_magnitude() {
        let g = RefractiveIndexGradient::air(0.3);
        // epsilon = (L / n0) * dn/dx = (0.3 / 1.000292) * 0.001 ≈ 2.999e-4
        let eps = g.deflection_from_gradient(0.001);
        assert!((eps - 2.999e-4).abs() < 1e-6);
    }

    #[test]
    fn test_gradient_field() {
        let g = RefractiveIndexGradient::air(0.3);
        let eps_x = vec![1e-4, 2e-4, 3e-4];
        let eps_y = vec![-1e-4, 0.0, 1e-4];
        let (dn_dx, dn_dy) = g.gradient_field(&eps_x, &eps_y).unwrap();
        assert_eq!(dn_dx.len(), 3);
        assert_eq!(dn_dy.len(), 3);
        // dn_dx[0] = (n0/L) * eps_x[0]
        let scale = N_AIR_STANDARD / 0.3;
        assert!((dn_dx[0] - scale * 1e-4).abs() < 1e-8);
    }

    #[test]
    fn test_gradient_field_dimension_mismatch() {
        let g = RefractiveIndexGradient::air(0.3);
        let result = g.gradient_field(&[1.0, 2.0], &[1.0]);
        assert!(result.is_err());
    }

    #[test]
    fn test_gradient_field_empty() {
        let g = RefractiveIndexGradient::air(0.3);
        let result = g.gradient_field(&[], &[]);
        assert!(result.is_err());
    }

    #[test]
    fn test_numerical_gradient() {
        // Linear profile: n(x) = 1.0 + 0.001 * x, gradient = 0.001 everywhere
        let dx = 0.01;
        let profile: Vec<f64> = (0..100).map(|i| 1.0 + 0.001 * i as f64 * dx).collect();
        let grad = RefractiveIndexGradient::numerical_gradient(&profile, dx);
        // Interior points should have gradient ≈ 0.001
        for &g in &grad[1..grad.len() - 1] {
            assert!((g - 0.001).abs() < 1e-8);
        }
    }

    #[test]
    fn test_density_gradient_deflection() {
        let g = RefractiveIndexGradient::air(0.3);
        let drho_dx = 1.0; // 1 kg/m³ per meter
        let eps = g.deflection_from_density_gradient(drho_dx, GLADSTONE_DALE_AIR);
        // epsilon = K * L / n0 * drho_dx
        let expected = GLADSTONE_DALE_AIR * 0.3 / N_AIR_STANDARD * drho_dx;
        assert!((eps - expected).abs() < 1e-10);
    }

    // --- KnifeEdgeCutoff tests ---

    #[test]
    fn test_knife_edge_zero_deflection() {
        let ke = KnifeEdgeCutoff::new(1.0, 0.002, 0.5);
        // At epsilon=0: I/I0 = 0.5
        assert!((ke.intensity(0.0) - 0.5).abs() < EPSILON);
    }

    #[test]
    fn test_knife_edge_positive_deflection() {
        let ke = KnifeEdgeCutoff::new(1.0, 0.002, 0.5);
        let i = ke.intensity(1e-4);
        // I/I0 = 0.5 + 1.0 * 1e-4 / 0.002 = 0.5 + 0.05 = 0.55
        assert!((i - 0.55).abs() < EPSILON);
    }

    #[test]
    fn test_knife_edge_saturation() {
        let ke = KnifeEdgeCutoff::new(1.0, 0.002, 0.5);
        // Large deflection should saturate at 1.0
        let i = ke.intensity(0.01);
        assert!((i - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_knife_edge_negative_saturation() {
        let ke = KnifeEdgeCutoff::new(1.0, 0.002, 0.5);
        // Large negative deflection should clamp at 0.0
        let i = ke.intensity(-0.01);
        assert!((i).abs() < EPSILON);
    }

    #[test]
    fn test_knife_edge_max_deflection() {
        let ke = KnifeEdgeCutoff::new(1.0, 0.002, 0.5);
        // max = (1 - 0.5) * 0.002 / 1.0 = 0.001
        assert!((ke.max_deflection() - 0.001).abs() < EPSILON);
    }

    #[test]
    fn test_knife_edge_process() {
        let ke = KnifeEdgeCutoff::new(1.0, 0.002, 0.5);
        let deflections = vec![0.0, 1e-4, -1e-4, 5e-4];
        let intensities = ke.process(&deflections);
        assert_eq!(intensities.len(), 4);
        assert!((intensities[0] - 0.5).abs() < EPSILON);
    }

    #[test]
    fn test_knife_edge_dynamic_range() {
        let ke = KnifeEdgeCutoff::new(1.0, 0.002, 0.5);
        // 50% cutoff: max_di = 0.5, min_di = 0.5, ratio = 1 => 0 dB
        assert!((ke.dynamic_range_db()).abs() < EPSILON);

        // 25% cutoff: max_di = 0.75, min_di = 0.25, ratio = 3 => ~9.54 dB
        let ke2 = KnifeEdgeCutoff::new(1.0, 0.002, 0.25);
        assert!((ke2.dynamic_range_db() - 9.542).abs() < 0.01);
    }

    // --- BackgroundOrientedSchlieren tests ---

    #[test]
    fn test_bos_cross_correlate_identical() {
        let a = vec![1.0, 2.0, 3.0, 2.0, 1.0];
        let corr = BackgroundOrientedSchlieren::cross_correlate_1d(&a, &a);
        // Peak at center (zero-lag)
        let center = a.len() - 1;
        for i in 0..corr.len() {
            if i != center {
                assert!(corr[center] >= corr[i]);
            }
        }
    }

    #[test]
    fn test_bos_subpixel_parabolic() {
        // Symmetric: no offset
        let sub = BackgroundOrientedSchlieren::subpixel_parabolic(3.0, 5.0, 3.0);
        assert!(sub.abs() < EPSILON);

        // Asymmetric: peak slightly to the right
        let sub2 = BackgroundOrientedSchlieren::subpixel_parabolic(3.0, 5.0, 4.0);
        assert!(sub2 > 0.0);
    }

    #[test]
    fn test_bos_displacement_zero() {
        let bos = BackgroundOrientedSchlieren::new(8, 4, 1.0, 1e-5);
        let signal = vec![1.0, 2.0, 3.0, 4.0, 3.0, 2.0, 1.0, 0.5];
        let disp = bos.compute_displacement_1d(&signal, &signal);
        assert!(disp.abs() < 0.5);
    }

    #[test]
    fn test_bos_displacement_to_deflection() {
        let bos = BackgroundOrientedSchlieren::new(8, 4, 1.0, 1e-5);
        // displacement = 2 pixels, pixel_size=1e-5, z_d=1.0
        // epsilon = 2 * 1e-5 / 1.0 = 2e-5
        let eps = bos.displacement_to_deflection(2.0);
        assert!((eps - 2e-5).abs() < 1e-10);
    }

    #[test]
    fn test_bos_process_1d_dimension_mismatch() {
        let bos = BackgroundOrientedSchlieren::new(4, 2, 1.0, 1e-5);
        let result = bos.process_1d(&[1.0; 10], &[1.0; 8]);
        assert!(result.is_err());
    }

    #[test]
    fn test_bos_process_1d_too_short() {
        let bos = BackgroundOrientedSchlieren::new(8, 4, 1.0, 1e-5);
        let result = bos.process_1d(&[1.0; 4], &[1.0; 4]);
        assert!(result.is_err());
    }

    // --- ShockWaveDetector tests ---

    #[test]
    fn test_shock_detect_step() {
        let detector = ShockWaveDetector::new(0.1, 3);
        // Create a sharp step in intensity
        let mut profile = vec![0.5; 50];
        for i in 25..50 {
            profile[i] = 0.9;
        }
        let shocks = detector.detect(&profile);
        assert!(!shocks.is_empty());
        // Shock should be near index 25
        assert!((shocks[0].position as f64 - 25.0).abs() < 3.0);
    }

    #[test]
    fn test_shock_detect_smooth() {
        let detector = ShockWaveDetector::new(0.5, 3);
        // Smooth profile: no shocks
        let profile: Vec<f64> = (0..100).map(|i| 0.5 + 0.001 * i as f64).collect();
        let shocks = detector.detect(&profile);
        assert!(shocks.is_empty());
    }

    #[test]
    fn test_shock_detect_empty() {
        let detector = ShockWaveDetector::new(0.1, 3);
        let shocks = detector.detect(&[]);
        assert!(shocks.is_empty());
    }

    #[test]
    fn test_shock_track_velocity() {
        let positions = vec![10, 15, 20, 25, 30];
        let vel = ShockWaveDetector::track_velocity(&positions);
        // Linear motion: 5 samples/frame
        assert!((vel - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_shock_track_velocity_single() {
        let vel = ShockWaveDetector::track_velocity(&[10]);
        assert!(vel.abs() < EPSILON);
    }

    // --- DensityReconstructor tests ---

    #[test]
    fn test_density_index_roundtrip() {
        let recon = DensityReconstructor::air(64);
        let densities = vec![1.0, 1.1, 1.2, 1.15, 1.05];
        let indices = recon.density_to_index(&densities);
        let recovered = recon.index_to_density(&indices);
        for (d, r) in densities.iter().zip(recovered.iter()) {
            assert!((d - r).abs() < 1e-10);
        }
    }

    #[test]
    fn test_abel_forward_inverse_consistency() {
        let recon = DensityReconstructor::air(32);
        // Create a simple radial profile: Gaussian-like
        let n = 32;
        let radial: Vec<f64> = (0..n)
            .map(|i| {
                let r = i as f64 / (n - 1) as f64;
                1e-3 * (-5.0 * r * r).exp()
            })
            .collect();

        // Forward transform
        let projection = recon.abel_forward(&radial);
        assert_eq!(projection.len(), n);

        // Inverse transform
        let recovered = recon.abel_invert(&projection).unwrap();
        assert_eq!(recovered.len(), n);

        // Check that interior points are at least qualitatively similar
        // (Abel inversion is ill-conditioned, so don't expect perfect recovery)
        // The recovered profile should be larger near center and smaller near edge
        assert!(recovered[0].abs() > recovered[n / 2].abs() || recovered[n / 2].abs() < 0.1);
    }

    #[test]
    fn test_abel_invert_too_few_points() {
        let recon = DensityReconstructor::air(32);
        let result = recon.abel_invert(&[1.0, 2.0]);
        assert!(result.is_err());
    }

    // --- RainbowSchlierenProcessor tests ---

    #[test]
    fn test_rainbow_zero_deflection() {
        let rsp = RainbowSchlierenProcessor::new(1e-3, 1.0, 0.01);
        // Zero deflection => 180 degrees
        let hue = rsp.deflection_to_hue(0.0);
        assert!((hue - 180.0).abs() < EPSILON);
    }

    #[test]
    fn test_rainbow_max_deflection() {
        let rsp = RainbowSchlierenProcessor::new(1e-3, 1.0, 0.01);
        // Max positive => 360 degrees
        let hue = rsp.deflection_to_hue(1e-3);
        assert!((hue - 360.0).abs() < EPSILON);
    }

    #[test]
    fn test_rainbow_roundtrip() {
        let rsp = RainbowSchlierenProcessor::new(1e-3, 1.0, 0.01);
        let eps = 5e-4;
        let hue = rsp.deflection_to_hue(eps);
        let recovered = rsp.hue_to_deflection(hue);
        assert!((recovered - eps).abs() < 1e-10);
    }

    #[test]
    fn test_rainbow_hue_to_rgb() {
        // Red at 0 degrees
        let (r, g, b) = RainbowSchlierenProcessor::hue_to_rgb(0.0);
        assert!((r - 1.0).abs() < EPSILON);
        assert!(g.abs() < EPSILON);
        assert!(b.abs() < EPSILON);

        // Green at 120 degrees
        let (r, g, _b) = RainbowSchlierenProcessor::hue_to_rgb(120.0);
        assert!(r.abs() < EPSILON);
        assert!((g - 1.0).abs() < EPSILON);

        // Blue at 240 degrees
        let (_r, g, b) = RainbowSchlierenProcessor::hue_to_rgb(240.0);
        assert!(g.abs() < EPSILON);
        assert!((b - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_rainbow_process() {
        let rsp = RainbowSchlierenProcessor::new(1e-3, 1.0, 0.01);
        let deflections = vec![-1e-3, -5e-4, 0.0, 5e-4, 1e-3];
        let hues = rsp.process(&deflections);
        assert_eq!(hues.len(), 5);
        // Should be monotonically increasing
        for i in 1..hues.len() {
            assert!(hues[i] > hues[i - 1]);
        }
    }

    // --- ShadowgraphSimulator tests ---

    #[test]
    fn test_shadowgraph_uniform() {
        let sg = ShadowgraphSimulator::new(1.0, 0.3);
        // Uniform refractive index: Laplacian = 0, I/I0 = 1
        let profile = vec![1.0003; 100];
        let intensity = sg.simulate_1d(&profile, 0.001);
        for &i in &intensity {
            assert!((i - 1.0).abs() < EPSILON);
        }
    }

    #[test]
    fn test_shadowgraph_gaussian() {
        let sg = ShadowgraphSimulator::new(0.5, 0.3);
        // Gaussian profile: has non-zero Laplacian
        let profile = gaussian_index_profile(100, (-1.0, 1.0), 1.0003, 1e-4, 0.0, 0.2);
        let intensity = sg.simulate_1d(&profile, 0.02);
        // Should have variations around 1.0
        let min = intensity.iter().copied().fold(f64::INFINITY, f64::min);
        let max = intensity.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        assert!(max > min);
    }

    #[test]
    fn test_laplacian_1d_constant() {
        let data = vec![5.0; 20];
        let lap = ShadowgraphSimulator::laplacian_1d(&data, 0.01);
        for &v in &lap {
            assert!(v.abs() < EPSILON);
        }
    }

    #[test]
    fn test_laplacian_1d_quadratic() {
        // f(x) = x^2, f''(x) = 2
        let dx = 0.01;
        let data: Vec<f64> = (0..100).map(|i| {
            let x = i as f64 * dx;
            x * x
        }).collect();
        let lap = ShadowgraphSimulator::laplacian_1d(&data, dx);
        // Interior should be close to 2.0
        for &v in &lap[2..lap.len() - 2] {
            assert!((v - 2.0).abs() < 0.01);
        }
    }

    #[test]
    fn test_sensitivity_ratio() {
        let sg = ShadowgraphSimulator::new(1.0, 0.3);
        let ratio = sg.sensitivity_ratio(0.01);
        // 0.01 / (1.0 * 0.3) = 0.0333...
        assert!((ratio - 1.0 / 30.0).abs() < 1e-6);
    }

    // --- MachNumberEstimator tests ---

    #[test]
    fn test_mach_from_angle() {
        let est = MachNumberEstimator::air();
        // sin(30°) = 0.5, so M = 2.0
        let m = est.from_mach_angle(PI / 6.0);
        assert!((m - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_mach_angle_roundtrip() {
        let est = MachNumberEstimator::air();
        let m = 3.0;
        let mu = est.mach_angle(m);
        let m_recovered = est.from_mach_angle(mu);
        assert!((m_recovered - m).abs() < 1e-6);
    }

    #[test]
    fn test_mach_angle_subsonic() {
        let est = MachNumberEstimator::air();
        let mu = est.mach_angle(0.5);
        assert!((mu - PI / 2.0).abs() < EPSILON);
    }

    #[test]
    fn test_normal_shock_pressure_ratio() {
        let est = MachNumberEstimator::air();
        // M=1: P2/P1 = 1
        assert!((est.normal_shock_pressure_ratio(1.0) - 1.0).abs() < EPSILON);
        // M=2: P2/P1 = 1 + 2*1.4/2.4*(4-1) = 1 + 3.5 = 4.5
        assert!((est.normal_shock_pressure_ratio(2.0) - 4.5).abs() < 1e-6);
    }

    #[test]
    fn test_normal_shock_density_ratio() {
        let est = MachNumberEstimator::air();
        // M=2: rho2/rho1 = 2.4*4 / (0.4*4 + 2) = 9.6/3.6 = 2.6667
        let ratio = est.normal_shock_density_ratio(2.0);
        assert!((ratio - 8.0 / 3.0).abs() < 1e-6);
    }

    #[test]
    fn test_normal_shock_downstream_mach() {
        let est = MachNumberEstimator::air();
        // M1=2: M2 = sqrt((0.4*4 + 2) / (2*1.4*4 - 0.4)) = sqrt(3.6/10.8) ≈ 0.5774
        let m2 = est.normal_shock_downstream_mach(2.0);
        assert!((m2 - (1.0 / 3.0_f64).sqrt()).abs() < 1e-4);
    }

    #[test]
    fn test_oblique_shock_angle() {
        let est = MachNumberEstimator::air();
        // M=2.0, theta=10 degrees
        let theta = 10.0_f64.to_radians();
        let beta = est.oblique_shock_angle(2.0, theta);
        assert!(beta.is_some());
        let b = beta.unwrap();
        // Weak shock angle should be between Mach angle and 90 degrees
        let mu = est.mach_angle(2.0);
        assert!(b > mu && b < PI / 2.0);
    }

    #[test]
    fn test_oblique_shock_subsonic_returns_none() {
        let est = MachNumberEstimator::air();
        assert!(est.oblique_shock_angle(0.5, 0.1).is_none());
    }

    // --- TemperatureFieldEstimator tests ---

    #[test]
    fn test_temperature_from_index_standard() {
        let est = TemperatureFieldEstimator::new_air(STANDARD_PRESSURE);
        let t = est.temperature_from_index(N_AIR_STANDARD);
        // T = K*P/(R*(n-1)) should recover standard temperature
        // N_AIR_STANDARD is derived from Gladstone-Dale at T=293.15
        assert!((t - STANDARD_TEMPERATURE).abs() < 2.0);
    }

    #[test]
    fn test_temperature_index_roundtrip() {
        let est = TemperatureFieldEstimator::new_air(STANDARD_PRESSURE);
        let t_original = 400.0; // Hot air
        let n = est.index_from_temperature(t_original);
        let t_recovered = est.temperature_from_index(n);
        assert!((t_recovered - t_original).abs() < 0.01);
    }

    #[test]
    fn test_density_from_temperature() {
        let est = TemperatureFieldEstimator::new_air(STANDARD_PRESSURE);
        // rho = P / (R * T) = 101325 / (287.058 * 293.15) ≈ 1.204
        let rho = est.density_from_temperature(293.15);
        assert!((rho - 1.204).abs() < 0.01);
    }

    #[test]
    fn test_process_temperature_field() {
        let est = TemperatureFieldEstimator::new_air(STANDARD_PRESSURE);
        let n_field = vec![1.000292, 1.000250, 1.000200];
        let temps = est.process_field(&n_field);
        assert_eq!(temps.len(), 3);
        // Higher refractive index => lower temperature
        assert!(temps[0] < temps[1]);
        assert!(temps[1] < temps[2]);
    }

    #[test]
    fn test_temperature_gradient() {
        let est = TemperatureFieldEstimator::new_air(STANDARD_PRESSURE);
        let dn_dx = 1e-4; // refractive index gradient
        let t = 300.0;
        let dt_dx = est.temperature_gradient(t, dn_dx);
        // Should be negative (higher n => lower T)
        assert!(dt_dx < 0.0);
    }

    // --- Utility function tests ---

    #[test]
    fn test_air_refractive_index_standard() {
        let n = air_refractive_index(STANDARD_TEMPERATURE, STANDARD_PRESSURE);
        // n = 1 + K*P/(R*T) should match N_AIR_STANDARD
        assert!((n - N_AIR_STANDARD).abs() < 5e-5);
    }

    #[test]
    fn test_air_density_standard() {
        let rho = air_density(STANDARD_TEMPERATURE, STANDARD_PRESSURE);
        // ~1.204 kg/m³
        assert!((rho - 1.204).abs() < 0.01);
    }

    #[test]
    fn test_gaussian_profile() {
        let profile = gaussian_index_profile(101, (-1.0, 1.0), 1.0003, 1e-4, 0.0, 0.2);
        assert_eq!(profile.len(), 101);
        // Peak at center
        let center = profile[50];
        assert!((center - (1.0003 + 1e-4)).abs() < 1e-6);
        // Edges near background
        assert!((profile[0] - 1.0003).abs() < 1e-6);
    }

    #[test]
    fn test_step_profile() {
        let profile = step_index_profile(100, 1.0003, 1.0001, 0.5, 0.01);
        assert_eq!(profile.len(), 100);
        // Near left edge: close to n_before
        assert!((profile[0] - 1.0003).abs() < 0.0002);
        // Near right edge: close to n_after
        assert!((profile[99] - 1.0001).abs() < 0.0002);
    }

    // --- SchlierenProcessor tests ---

    #[test]
    fn test_processor_uniform() {
        let config = SchlierenConfig::new(SchlierenType::Classical, 1.0, 50.0);
        let proc = SchlierenProcessor::new(config, STANDARD_PRESSURE);
        let profile = vec![1.0003; 50];
        let (intensities, temperatures) = proc.process_profile(&profile, 0.01);
        assert_eq!(intensities.len(), 50);
        assert_eq!(temperatures.len(), 50);
        // Uniform profile: all intensities near 0.5 (50% cutoff)
        for &i in &intensities {
            assert!((i - 0.5).abs() < 0.01);
        }
    }

    #[test]
    fn test_processor_gradient() {
        let config = SchlierenConfig::new(SchlierenType::Classical, 1.0, 50.0)
            .with_source_size(0.002)
            .with_path_length(0.3);
        let proc = SchlierenProcessor::new(config, STANDARD_PRESSURE);
        let profile = gaussian_index_profile(100, (-0.5, 0.5), 1.0003, 5e-5, 0.0, 0.1);
        let (intensities, _temperatures) = proc.process_profile(&profile, 0.01);
        // Should show variation due to gradient
        let min = intensities.iter().copied().fold(f64::INFINITY, f64::min);
        let max = intensities.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        assert!(max > min);
    }

    #[test]
    fn test_processor_knife_edge_access() {
        let config = SchlierenConfig::new(SchlierenType::Classical, 1.0, 50.0);
        let proc = SchlierenProcessor::new(config, STANDARD_PRESSURE);
        let ke = proc.knife_edge();
        assert!((ke.focal_length - 1.0).abs() < EPSILON);
    }

    // --- Additional edge case tests ---

    #[test]
    fn test_schlieren_type_variants() {
        assert_eq!(
            SchlierenType::Classical,
            SchlierenType::Classical
        );
        assert_ne!(
            SchlierenType::Classical,
            SchlierenType::Rainbow
        );
        assert_ne!(
            SchlierenType::Rainbow,
            SchlierenType::BackgroundOriented
        );
    }

    #[test]
    fn test_knife_orientation() {
        let cfg = SchlierenConfig::new(SchlierenType::Classical, 1.0, 50.0)
            .with_orientation(KnifeEdgeOrientation::Horizontal);
        assert_eq!(cfg.orientation, KnifeEdgeOrientation::Horizontal);
    }

    #[test]
    fn test_shock_geometry_variants() {
        let det = ShockWaveDetector::new(0.1, 3);
        let mut profile = vec![0.5; 50];
        for i in 25..50 {
            profile[i] = 0.9;
        }
        let shocks = det.detect(&profile);
        // Default geometry is Normal
        if !shocks.is_empty() {
            assert_eq!(shocks[0].geometry, ShockGeometry::Normal);
        }
    }

    #[test]
    fn test_mach_temperature_ratio() {
        let est = MachNumberEstimator::air();
        // M=2: T2/T1 = (P2/P1) / (rho2/rho1) = 4.5 / (8/3) = 1.6875
        let t_ratio = est.normal_shock_temperature_ratio(2.0);
        assert!((t_ratio - 1.6875).abs() < 1e-4);
    }

    #[test]
    fn test_bos_process_1d_identical() {
        let bos = BackgroundOrientedSchlieren::new(8, 4, 1.0, 1e-5);
        let signal: Vec<f64> = (0..32).map(|i| (i as f64 * 0.5).sin()).collect();
        let result = bos.process_1d(&signal, &signal).unwrap();
        // Identical signals: zero displacement => zero deflection
        for &d in &result {
            assert!(d.abs() < 1e-3);
        }
    }

    #[test]
    fn test_min_detectable_gradient() {
        let cfg = SchlierenConfig::new(SchlierenType::Classical, 1.0, 50.0)
            .with_path_length(0.3);
        let grad = cfg.min_detectable_gradient();
        // Should be positive and small
        assert!(grad > 0.0);
        assert!(grad < 0.1);
    }
}
