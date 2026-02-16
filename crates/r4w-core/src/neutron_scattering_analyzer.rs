// trace:FR-NEUTRON-SANS | ai:claude
//! # Neutron Scattering Analyzer — SANS Data Reduction and Modeling
//!
//! Implements signal processing for Small-Angle Neutron Scattering (SANS) and
//! neutron diffraction data analysis. Provides momentum transfer calculation,
//! detector data reduction, Guinier and Porod regime analysis, form factor and
//! structure factor computation, scattering invariant calculation, Kratky plot
//! analysis, and least-squares model fitting.
//!
//! Applications: polymer science, biological macromolecules, colloidal systems,
//! magnetic structures, nanoparticle characterization.
//!
//! All math is implemented from scratch using only the standard library.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::neutron_scattering_analyzer::{QCalculator, SansConfig, GuinierAnalyzer};
//!
//! let config = SansConfig::new(6.0, 4.0, 0.005);
//! let q_calc = QCalculator::new(&config);
//! let q = q_calc.q_from_pixel(100.0);
//! assert!(q > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// SANS instrument configuration.
#[derive(Debug, Clone)]
pub struct SansConfig {
    /// Neutron wavelength in Angstroms.
    pub wavelength: f64,
    /// Sample-to-detector distance in meters.
    pub sdd: f64,
    /// Detector pixel size in meters.
    pub pixel_size: f64,
    /// Minimum Q value (Å^-1) for analysis range.
    pub q_min: f64,
    /// Maximum Q value (Å^-1) for analysis range.
    pub q_max: f64,
    /// Wavelength spread delta_lambda/lambda (fractional).
    pub wavelength_spread: f64,
}

impl SansConfig {
    /// Create a new SANS configuration.
    ///
    /// # Arguments
    /// * `wavelength` — neutron wavelength in Angstroms
    /// * `sdd` — sample-to-detector distance in meters
    /// * `pixel_size` — detector pixel size in meters
    pub fn new(wavelength: f64, sdd: f64, pixel_size: f64) -> Self {
        Self {
            wavelength,
            sdd,
            pixel_size,
            q_min: 0.001,
            q_max: 1.0,
            wavelength_spread: 0.10,
        }
    }

    /// Set Q range for analysis.
    pub fn with_q_range(mut self, q_min: f64, q_max: f64) -> Self {
        self.q_min = q_min;
        self.q_max = q_max;
        self
    }

    /// Set wavelength spread (delta_lambda / lambda).
    pub fn with_wavelength_spread(mut self, spread: f64) -> Self {
        self.wavelength_spread = spread;
        self
    }
}

// ---------------------------------------------------------------------------
// Q Calculator — Momentum Transfer
// ---------------------------------------------------------------------------

/// Calculates momentum transfer Q from geometry.
///
/// Q = (4 * pi / lambda) * sin(theta / 2)
///
/// where theta is the full scattering angle.
#[derive(Debug, Clone)]
pub struct QCalculator {
    wavelength: f64,
    sdd: f64,
    pixel_size: f64,
}

impl QCalculator {
    /// Create a new Q calculator from instrument config.
    pub fn new(config: &SansConfig) -> Self {
        Self {
            wavelength: config.wavelength,
            sdd: config.sdd,
            pixel_size: config.pixel_size,
        }
    }

    /// Calculate Q from scattering angle theta (radians).
    ///
    /// Q = (4*pi/lambda) * sin(theta/2)
    pub fn q_from_angle(&self, theta: f64) -> f64 {
        (4.0 * PI / self.wavelength) * (theta / 2.0).sin()
    }

    /// Calculate Q from detector pixel offset (in pixels from beam center).
    ///
    /// Uses tan(theta) = pixel_offset * pixel_size / SDD
    pub fn q_from_pixel(&self, pixel_offset: f64) -> f64 {
        let r = pixel_offset * self.pixel_size;
        let theta = (r / self.sdd).atan();
        self.q_from_angle(theta)
    }

    /// Calculate Q from detector radial distance r (in meters from beam center).
    pub fn q_from_distance(&self, r: f64) -> f64 {
        let theta = (r / self.sdd).atan();
        self.q_from_angle(theta)
    }

    /// Calculate scattering angle theta from Q value (radians).
    pub fn angle_from_q(&self, q: f64) -> f64 {
        2.0 * (q * self.wavelength / (4.0 * PI)).asin()
    }

    /// Q resolution due to wavelength spread and geometry.
    ///
    /// sigma_Q / Q = sqrt((delta_lambda/lambda)^2 + (delta_theta/theta)^2)
    pub fn q_resolution(&self, q: f64, delta_lambda_over_lambda: f64, pixel_size: f64) -> f64 {
        let theta = self.angle_from_q(q);
        if theta.abs() < 1e-15 {
            return 0.0;
        }
        let delta_theta = pixel_size / self.sdd;
        let geom_term = (delta_theta / theta).powi(2);
        let wave_term = delta_lambda_over_lambda.powi(2);
        q * (geom_term + wave_term).sqrt()
    }
}

// ---------------------------------------------------------------------------
// Data Reducer — Detector Normalization
// ---------------------------------------------------------------------------

/// A single reduced scattering data point: I(Q) ± sigma.
#[derive(Debug, Clone, Copy)]
pub struct ScatteringPoint {
    pub q: f64,
    pub intensity: f64,
    pub error: f64,
}

/// Reduces raw 2D detector data to 1D I(Q) profile.
///
/// I(Q) = (signal - background) / (monitor * transmission * solid_angle)
#[derive(Debug, Clone)]
pub struct DataReducer {
    wavelength: f64,
    sdd: f64,
    pixel_size: f64,
}

impl DataReducer {
    /// Create a new data reducer from config.
    pub fn new(config: &SansConfig) -> Self {
        Self {
            wavelength: config.wavelength,
            sdd: config.sdd,
            pixel_size: config.pixel_size,
        }
    }

    /// Solid angle subtended by a pixel at position r (meters from center).
    ///
    /// Omega = pixel_size^2 * cos^3(theta) / SDD^2
    pub fn solid_angle(&self, r: f64) -> f64 {
        let theta = (r / self.sdd).atan();
        let cos_theta = theta.cos();
        self.pixel_size * self.pixel_size * cos_theta.powi(3) / (self.sdd * self.sdd)
    }

    /// Reduce a single pixel measurement.
    ///
    /// # Arguments
    /// * `signal` — raw detector counts
    /// * `background` — background counts (dark current + blocked beam)
    /// * `monitor` — beam monitor counts (normalization)
    /// * `transmission` — sample transmission factor (0..1)
    /// * `r` — pixel radial distance from beam center in meters
    ///
    /// Returns intensity and estimated error.
    pub fn reduce_pixel(
        &self,
        signal: f64,
        background: f64,
        monitor: f64,
        transmission: f64,
        r: f64,
    ) -> (f64, f64) {
        let omega = self.solid_angle(r);
        let net = signal - background;
        let denom = monitor * transmission * omega;
        if denom.abs() < 1e-30 {
            return (0.0, 0.0);
        }
        let intensity = net / denom;
        // Poisson error propagation: sigma = sqrt(signal + background) / denom
        let sigma = (signal + background).sqrt() / denom;
        (intensity, sigma)
    }

    /// Azimuthally average detector data into radial I(Q) bins.
    ///
    /// # Arguments
    /// * `pixel_data` — list of (pixel_x, pixel_y, counts) relative to beam center
    /// * `background` — flat background to subtract
    /// * `monitor` — monitor normalization
    /// * `transmission` — sample transmission
    /// * `n_bins` — number of radial Q bins
    /// * `q_min` — minimum Q
    /// * `q_max` — maximum Q
    pub fn azimuthal_average(
        &self,
        pixel_data: &[(f64, f64, f64)],
        background: f64,
        monitor: f64,
        transmission: f64,
        n_bins: usize,
        q_min: f64,
        q_max: f64,
    ) -> Vec<ScatteringPoint> {
        let q_calc = QCalculator {
            wavelength: self.wavelength,
            sdd: self.sdd,
            pixel_size: self.pixel_size,
        };

        let log_q_min = q_min.ln();
        let log_q_max = q_max.ln();
        let bin_width = (log_q_max - log_q_min) / n_bins as f64;

        let mut bin_sum = vec![0.0_f64; n_bins];
        let mut bin_err_sq = vec![0.0_f64; n_bins];
        let mut bin_count = vec![0_u64; n_bins];

        for &(px, py, counts) in pixel_data {
            let r = (px * px + py * py).sqrt() * self.pixel_size;
            let q = q_calc.q_from_distance(r);
            if q < q_min || q >= q_max {
                continue;
            }

            let (intensity, sigma) = self.reduce_pixel(counts, background, monitor, transmission, r);

            let bin_idx = ((q.ln() - log_q_min) / bin_width) as usize;
            if bin_idx < n_bins {
                bin_sum[bin_idx] += intensity;
                bin_err_sq[bin_idx] += sigma * sigma;
                bin_count[bin_idx] += 1;
            }
        }

        let mut result = Vec::new();
        for i in 0..n_bins {
            if bin_count[i] > 0 {
                let n = bin_count[i] as f64;
                let q_center = ((log_q_min + (i as f64 + 0.5) * bin_width)).exp();
                let avg_i = bin_sum[i] / n;
                let avg_err = (bin_err_sq[i]).sqrt() / n;
                result.push(ScatteringPoint {
                    q: q_center,
                    intensity: avg_i,
                    error: avg_err,
                });
            }
        }
        result
    }
}

// ---------------------------------------------------------------------------
// Guinier Analyzer — Low-Q Regime
// ---------------------------------------------------------------------------

/// Result of Guinier analysis.
#[derive(Debug, Clone, Copy)]
pub struct GuinierResult {
    /// Radius of gyration (Å).
    pub rg: f64,
    /// Forward scattering intensity I(0).
    pub i0: f64,
    /// R-squared goodness of fit.
    pub r_squared: f64,
    /// Maximum Q*Rg used in fit.
    pub q_rg_max: f64,
}

/// Guinier regime analysis: ln(I) = ln(I0) - Rg^2 * Q^2 / 3.
///
/// Extracts radius of gyration Rg from low-Q data via linear regression
/// of ln(I) vs Q^2.
#[derive(Debug, Clone)]
pub struct GuinierAnalyzer {
    /// Maximum Q*Rg allowed (typically ~1.3 for globular particles).
    pub q_rg_limit: f64,
}

impl GuinierAnalyzer {
    /// Create analyzer with default Q*Rg limit of 1.3.
    pub fn new() -> Self {
        Self { q_rg_limit: 1.3 }
    }

    /// Create analyzer with custom Q*Rg limit.
    pub fn with_limit(q_rg_limit: f64) -> Self {
        Self { q_rg_limit }
    }

    /// Perform Guinier fit on I(Q) data.
    ///
    /// Fits ln(I) = ln(I0) - Rg^2/3 * Q^2 in the Guinier regime.
    /// Returns None if insufficient data or invalid fit.
    pub fn fit(&self, data: &[ScatteringPoint]) -> Option<GuinierResult> {
        if data.len() < 3 {
            return None;
        }

        // Iterative: first guess Rg, then refine Q range
        // Start with all data that has positive intensity
        let positive: Vec<&ScatteringPoint> = data.iter().filter(|p| p.intensity > 0.0).collect();
        if positive.len() < 3 {
            return None;
        }

        // First pass: use all positive-intensity points
        let result = self.linear_fit(&positive)?;

        // Second pass: restrict to Q*Rg < limit
        let q_max = self.q_rg_limit / result.rg;
        let restricted: Vec<&ScatteringPoint> = positive
            .iter()
            .filter(|p| p.q <= q_max)
            .copied()
            .collect();

        if restricted.len() < 3 {
            return Some(result);
        }

        self.linear_fit(&restricted)
    }

    /// Linear regression of ln(I) vs Q^2.
    fn linear_fit(&self, data: &[&ScatteringPoint]) -> Option<GuinierResult> {
        let n = data.len() as f64;
        if n < 3.0 {
            return None;
        }

        // x = Q^2, y = ln(I)
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xx = 0.0;
        let mut sum_xy = 0.0;

        for &p in data {
            let x = p.q * p.q;
            let y = p.intensity.ln();
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
        }

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return None;
        }

        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n;

        // slope = -Rg^2 / 3 => Rg = sqrt(-3 * slope)
        if slope >= 0.0 {
            return None; // Guinier requires negative slope
        }

        let rg = (-3.0 * slope).sqrt();
        let i0 = intercept.exp();

        // R-squared
        let y_mean = sum_y / n;
        let mut ss_tot = 0.0;
        let mut ss_res = 0.0;
        for &p in data {
            let x = p.q * p.q;
            let y = p.intensity.ln();
            let y_fit = intercept + slope * x;
            ss_tot += (y - y_mean).powi(2);
            ss_res += (y - y_fit).powi(2);
        }

        let r_squared = if ss_tot > 1e-30 {
            1.0 - ss_res / ss_tot
        } else {
            0.0
        };

        let q_max_used = data.iter().map(|p| p.q).fold(0.0_f64, |a, b| a.max(b));

        Some(GuinierResult {
            rg,
            i0,
            r_squared,
            q_rg_max: q_max_used * rg,
        })
    }
}

impl Default for GuinierAnalyzer {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Porod Analyzer — High-Q Regime
// ---------------------------------------------------------------------------

/// Result of Porod analysis.
#[derive(Debug, Clone, Copy)]
pub struct PorodResult {
    /// Porod constant K_p = 2*pi*S_v * (delta_rho)^2 (cm^-1 Å^-4).
    pub porod_constant: f64,
    /// Specific surface area S_v (cm^-1) if contrast is known.
    pub specific_surface: Option<f64>,
    /// Porod exponent (ideally 4 for sharp interfaces).
    pub exponent: f64,
    /// R-squared of the power-law fit.
    pub r_squared: f64,
}

/// Porod regime analysis: I(Q) ~ K_p * Q^-n.
///
/// For sharp interfaces n=4, for Gaussian chains n=2, for rods n=1.
/// Extracts Porod constant and specific surface area.
#[derive(Debug, Clone)]
pub struct PorodAnalyzer;

impl PorodAnalyzer {
    /// Fit power-law I(Q) = A * Q^(-n) in the high-Q Porod regime.
    ///
    /// Uses linear regression of log(I) vs log(Q).
    ///
    /// # Arguments
    /// * `data` — scattering data points
    /// * `q_min` — minimum Q for Porod region
    /// * `q_max` — maximum Q for Porod region
    /// * `contrast_sq` — optional (delta_rho)^2 for surface area calculation
    pub fn fit(
        &self,
        data: &[ScatteringPoint],
        q_min: f64,
        q_max: f64,
        contrast_sq: Option<f64>,
    ) -> Option<PorodResult> {
        let region: Vec<&ScatteringPoint> = data
            .iter()
            .filter(|p| p.q >= q_min && p.q <= q_max && p.intensity > 0.0)
            .collect();

        if region.len() < 3 {
            return None;
        }

        let n = region.len() as f64;
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xx = 0.0;
        let mut sum_xy = 0.0;

        for &p in &region {
            let x = p.q.ln();
            let y = p.intensity.ln();
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
        }

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return None;
        }

        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n;

        let exponent = -slope;
        let amplitude = intercept.exp();

        // R-squared
        let y_mean = sum_y / n;
        let mut ss_tot = 0.0;
        let mut ss_res = 0.0;
        for &p in &region {
            let x = p.q.ln();
            let y = p.intensity.ln();
            let y_fit = intercept + slope * x;
            ss_tot += (y - y_mean).powi(2);
            ss_res += (y - y_fit).powi(2);
        }

        let r_squared = if ss_tot > 1e-30 {
            1.0 - ss_res / ss_tot
        } else {
            0.0
        };

        // Porod constant K_p = A (the amplitude of I*Q^4 in the Porod limit)
        // For ideal Porod: I(Q) = K_p * Q^-4, so K_p = A when exponent ~ 4
        let porod_constant = amplitude;

        // S_v = K_p / (2*pi*(delta_rho)^2)
        let specific_surface = contrast_sq.map(|dr2| porod_constant / (2.0 * PI * dr2));

        Some(PorodResult {
            porod_constant,
            specific_surface,
            exponent,
            r_squared,
        })
    }

    /// Compute Porod plot: I(Q)*Q^4 vs Q.
    ///
    /// A plateau in this plot indicates ideal Porod behavior.
    pub fn porod_plot(data: &[ScatteringPoint]) -> Vec<(f64, f64)> {
        data.iter()
            .map(|p| (p.q, p.intensity * p.q.powi(4)))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Form Factor Calculator — P(Q)
// ---------------------------------------------------------------------------

/// Standard particle shapes for form factor calculation.
#[derive(Debug, Clone, Copy)]
pub enum ParticleShape {
    /// Uniform sphere of radius R (Å).
    Sphere(f64),
    /// Cylinder of radius R (Å) and length L (Å).
    Cylinder { radius: f64, length: f64 },
    /// Ellipsoid with semi-axes a, b (Å), where b is the rotational axis.
    Ellipsoid { a: f64, b: f64 },
    /// Gaussian coil with radius of gyration Rg (Å).
    GaussianCoil(f64),
    /// Core-shell sphere: core radius R_core, shell outer radius R_shell (Å).
    CoreShell {
        core_radius: f64,
        shell_radius: f64,
        /// Ratio of shell SLD contrast to core SLD contrast.
        sld_ratio: f64,
    },
}

/// Calculates form factor P(Q) for standard particle shapes.
#[derive(Debug, Clone)]
pub struct FormFactorCalculator;

impl FormFactorCalculator {
    /// Compute form factor P(Q) for a given shape.
    pub fn compute(shape: &ParticleShape, q: f64) -> f64 {
        match shape {
            ParticleShape::Sphere(r) => Self::sphere(q, *r),
            ParticleShape::Cylinder { radius, length } => Self::cylinder(q, *radius, *length),
            ParticleShape::Ellipsoid { a, b } => Self::ellipsoid(q, *a, *b),
            ParticleShape::GaussianCoil(rg) => Self::gaussian_coil(q, *rg),
            ParticleShape::CoreShell {
                core_radius,
                shell_radius,
                sld_ratio,
            } => Self::core_shell(q, *core_radius, *shell_radius, *sld_ratio),
        }
    }

    /// Sphere form factor: P(Q) = [3*(sin(QR) - QR*cos(QR)) / (QR)^3]^2
    pub fn sphere(q: f64, r: f64) -> f64 {
        let qr = q * r;
        if qr.abs() < 1e-10 {
            return 1.0;
        }
        let phi = 3.0 * (qr.sin() - qr * qr.cos()) / (qr * qr * qr);
        phi * phi
    }

    /// Gaussian coil (Debye function):
    /// P(Q) = 2*(exp(-x) - 1 + x) / x^2 where x = (Q*Rg)^2
    pub fn gaussian_coil(q: f64, rg: f64) -> f64 {
        let x = (q * rg).powi(2);
        if x < 1e-10 {
            return 1.0;
        }
        2.0 * ((-x).exp() - 1.0 + x) / (x * x)
    }

    /// Cylinder form factor (orientationally averaged).
    ///
    /// Uses Gauss-Legendre quadrature over orientation angle alpha.
    pub fn cylinder(q: f64, r: f64, l: f64) -> f64 {
        if q.abs() < 1e-15 {
            return 1.0;
        }
        // 20-point Gauss-Legendre on [0, pi/2]
        let n_points = 20;
        let mut sum = 0.0;
        for i in 0..n_points {
            let alpha = (i as f64 + 0.5) * PI / (2.0 * n_points as f64);
            let sin_a = alpha.sin();
            let cos_a = alpha.cos();

            // Radial part: 2*J1(QR*sin(alpha)) / (QR*sin(alpha))
            let qr_sin = q * r * sin_a;
            let radial = if qr_sin.abs() < 1e-10 {
                1.0
            } else {
                2.0 * bessel_j1(qr_sin) / qr_sin
            };

            // Axial part: sin(QL*cos(alpha)/2) / (QL*cos(alpha)/2)
            let ql_cos = q * l * cos_a / 2.0;
            let axial = if ql_cos.abs() < 1e-10 {
                1.0
            } else {
                ql_cos.sin() / ql_cos
            };

            let f = radial * axial;
            sum += f * f * sin_a;
        }
        sum * PI / (2.0 * n_points as f64)
    }

    /// Ellipsoid form factor (orientationally averaged).
    ///
    /// Oblate (a > b) or prolate (b > a) ellipsoid of revolution.
    pub fn ellipsoid(q: f64, a: f64, b: f64) -> f64 {
        if q.abs() < 1e-15 {
            return 1.0;
        }
        // Average over orientation angle
        let n_points = 40;
        let mut sum = 0.0;
        for i in 0..n_points {
            let mu = (i as f64 + 0.5) / n_points as f64; // cos(alpha) from 0 to 1
            let r_eff = (a * a * (1.0 - mu * mu) + b * b * mu * mu).sqrt();
            let pq = Self::sphere(q, r_eff);
            sum += pq;
        }
        sum / n_points as f64
    }

    /// Core-shell sphere form factor.
    ///
    /// F(Q) = V_core*(rho_core-rho_shell)*Phi(Q,R_core) + V_shell*(rho_shell-rho_solvent)*Phi(Q,R_shell)
    /// P(Q) = |F(Q)|^2 / |F(0)|^2
    pub fn core_shell(q: f64, r_core: f64, r_shell: f64, sld_ratio: f64) -> f64 {
        // sld_ratio = (rho_core - rho_shell) / (rho_shell - rho_solvent)
        let v_core = 4.0 / 3.0 * PI * r_core.powi(3);
        let v_shell = 4.0 / 3.0 * PI * r_shell.powi(3);

        let phi_core = Self::sphere_amplitude(q, r_core);
        let phi_shell = Self::sphere_amplitude(q, r_shell);

        let f = sld_ratio * v_core * phi_core + v_shell * phi_shell;
        let f0 = sld_ratio * v_core + v_shell;

        if f0.abs() < 1e-30 {
            return 0.0;
        }
        (f / f0).powi(2)
    }

    /// Sphere amplitude (not squared): Phi(Q,R) = 3*(sin(QR) - QR*cos(QR))/(QR)^3
    fn sphere_amplitude(q: f64, r: f64) -> f64 {
        let qr = q * r;
        if qr.abs() < 1e-10 {
            return 1.0;
        }
        3.0 * (qr.sin() - qr * qr.cos()) / (qr * qr * qr)
    }

    /// Compute P(Q) over a Q range.
    pub fn compute_range(shape: &ParticleShape, q_values: &[f64]) -> Vec<f64> {
        q_values.iter().map(|&q| Self::compute(shape, q)).collect()
    }
}

/// First-order Bessel function J1(x) — polynomial approximation.
fn bessel_j1(x: f64) -> f64 {
    let ax = x.abs();
    if ax < 8.0 {
        // Approximation for |x| < 8
        let y = x * x;
        let num = x
            * (72362614232.0
                + y * (-7895059235.0
                    + y * (242396853.1
                        + y * (-2972611.439 + y * (15704.48260 + y * (-30.16036606))))));
        let den = 144725228442.0
            + y * (2300535178.0
                + y * (18583304.74 + y * (99447.43394 + y * (376.9991397 + y))));
        num / den
    } else {
        // Asymptotic approximation for |x| >= 8
        let z = 8.0 / ax;
        let y = z * z;
        let xx = ax - 2.356194491; // ax - 3*pi/4
        let p = 1.0
            + y * (0.183105e-2
                + y * (-0.3516396496e-4 + y * (0.2457520174e-5 + y * (-0.240337019e-6))));
        let q = 0.04687499995
            + y * (-0.2002690873e-3
                + y * (0.8449199096e-5 + y * (-0.88228987e-6 + y * 0.105787412e-6)));
        let ans = (0.5641895835 / ax.sqrt()) * (xx.cos() * p - z * xx.sin() * q);
        if x < 0.0 {
            -ans
        } else {
            ans
        }
    }
}

// ---------------------------------------------------------------------------
// Structure Factor Estimator — S(Q)
// ---------------------------------------------------------------------------

/// Result of structure factor extraction.
#[derive(Debug, Clone)]
pub struct StructureFactorResult {
    /// Q values.
    pub q: Vec<f64>,
    /// Structure factor S(Q).
    pub s_q: Vec<f64>,
}

/// Estimates inter-particle structure factor S(Q) from measured I(Q) and model P(Q).
///
/// I(Q) = N * V^2 * (delta_rho)^2 * P(Q) * S(Q)
/// => S(Q) = I(Q) / (scale * P(Q))
#[derive(Debug, Clone)]
pub struct StructureFactorEstimator;

impl StructureFactorEstimator {
    /// Extract S(Q) by dividing measured intensity by scaled form factor.
    ///
    /// S(Q) = I(Q) / (scale * P(Q))
    ///
    /// At high Q, S(Q) → 1 (no correlations), so scale is calibrated
    /// from the high-Q region if not provided.
    pub fn extract(
        data: &[ScatteringPoint],
        shape: &ParticleShape,
        scale: Option<f64>,
    ) -> StructureFactorResult {
        let scale_factor = match scale {
            Some(s) => s,
            None => Self::calibrate_scale(data, shape),
        };

        let mut q_vals = Vec::with_capacity(data.len());
        let mut s_vals = Vec::with_capacity(data.len());

        for p in data {
            let pq = FormFactorCalculator::compute(shape, p.q);
            let denom = scale_factor * pq;
            let sq = if denom.abs() > 1e-30 {
                p.intensity / denom
            } else {
                1.0
            };
            q_vals.push(p.q);
            s_vals.push(sq);
        }

        StructureFactorResult {
            q: q_vals,
            s_q: s_vals,
        }
    }

    /// Calibrate scale so S(Q) → 1 at high Q.
    fn calibrate_scale(data: &[ScatteringPoint], shape: &ParticleShape) -> f64 {
        if data.is_empty() {
            return 1.0;
        }

        // Use top 20% of Q range for calibration
        let q_max = data.iter().map(|p| p.q).fold(0.0_f64, |a, b| a.max(b));
        let q_threshold = q_max * 0.8;

        let mut sum_ratio = 0.0;
        let mut count = 0;
        for p in data {
            if p.q >= q_threshold {
                let pq = FormFactorCalculator::compute(shape, p.q);
                if pq > 1e-30 {
                    sum_ratio += p.intensity / pq;
                    count += 1;
                }
            }
        }

        if count > 0 {
            sum_ratio / count as f64
        } else {
            1.0
        }
    }

    /// Hard-sphere Percus-Yevick structure factor.
    ///
    /// Analytical solution for hard spheres of radius R_hs at volume fraction eta.
    pub fn percus_yevick(q: f64, r_hs: f64, eta: f64) -> f64 {
        let x = 2.0 * q * r_hs;
        if x.abs() < 1e-4 {
            // S(0) = (1-eta)^4 / (1+2*eta)^2 for hard spheres (compressibility equation)
            let alpha = (1.0 + 2.0 * eta).powi(2) / (1.0 - eta).powi(4);
            return 1.0 / alpha;
        }

        // Direct correlation function in Fourier space
        let c_q = Self::py_c_hat(x, eta);
        // S(Q) = 1 / (1 - n * c(Q))
        // where n = 6*eta / (pi * sigma^3) and c(Q) is DCF fourier transform
        // For PY, the combined result is:
        1.0 / (1.0 - 24.0 * eta * c_q)
    }

    /// Percus-Yevick direct correlation function Fourier transform.
    fn py_c_hat(x: f64, eta: f64) -> f64 {
        let eta2 = eta * eta;

        let alpha = (1.0 + 2.0 * eta).powi(2) / (1.0 - eta).powi(4);
        let beta = -6.0 * eta * (1.0 + eta / 2.0).powi(2) / (1.0 - eta).powi(4);
        let gamma = 0.5 * eta * alpha;

        let x2 = x * x;
        let x3 = x2 * x;
        let x4 = x3 * x;
        let x6 = x4 * x2;
        let sin_x = x.sin();
        let cos_x = x.cos();

        let g1 = alpha * (sin_x - x * cos_x) / x3;
        let g2 = beta * (2.0 * x * sin_x + (2.0 - x2) * cos_x - 2.0) / x4;
        let g3 = gamma
            * (-x4 * cos_x + 4.0 * ((3.0 * x2 - 6.0) * cos_x + (x3 - 6.0 * x) * sin_x + 6.0))
            / x6;

        -(g1 + g2 + g3)
    }
}

// ---------------------------------------------------------------------------
// Invariant Calculator — Scattering Invariant Q*
// ---------------------------------------------------------------------------

/// Result of invariant calculation.
#[derive(Debug, Clone, Copy)]
pub struct InvariantResult {
    /// Scattering invariant Q* = integral(I(Q)*Q^2 dQ).
    pub invariant: f64,
    /// Volume fraction phi (if contrast is known).
    pub volume_fraction: Option<f64>,
    /// Low-Q extrapolated contribution.
    pub low_q_contribution: f64,
    /// Measured region contribution.
    pub measured_contribution: f64,
    /// High-Q extrapolated contribution.
    pub high_q_contribution: f64,
}

/// Computes the scattering invariant Q* = integral(I(Q)*Q^2 dQ).
///
/// For a two-phase system: Q* = 2*pi^2 * phi * (1-phi) * (delta_rho)^2
/// which gives volume fraction if the contrast is known.
#[derive(Debug, Clone)]
pub struct InvariantCalculator;

impl InvariantCalculator {
    /// Compute scattering invariant from I(Q) data.
    ///
    /// Uses trapezoidal integration over the measured Q range,
    /// with optional Guinier extrapolation at low Q and Porod extrapolation at high Q.
    pub fn compute(
        data: &[ScatteringPoint],
        guinier: Option<&GuinierResult>,
        porod_constant: Option<f64>,
        contrast_sq: Option<f64>,
    ) -> Option<InvariantResult> {
        if data.len() < 2 {
            return None;
        }

        // Sort by Q (should already be sorted, but ensure)
        let mut sorted: Vec<ScatteringPoint> = data.to_vec();
        sorted.sort_by(|a, b| a.q.partial_cmp(&b.q).unwrap_or(std::cmp::Ordering::Equal));

        // Low-Q extrapolation via Guinier
        let low_q = if let Some(g) = guinier {
            // Integrate I0*exp(-Rg^2*Q^2/3)*Q^2 from 0 to Q_min
            let q_min = sorted[0].q;
            Self::guinier_integral(g.i0, g.rg, 0.0, q_min)
        } else {
            0.0
        };

        // Measured region: trapezoidal rule for integral(I(Q)*Q^2 dQ)
        let mut measured = 0.0;
        for i in 1..sorted.len() {
            let dq = sorted[i].q - sorted[i - 1].q;
            let f1 = sorted[i - 1].intensity * sorted[i - 1].q.powi(2);
            let f2 = sorted[i].intensity * sorted[i].q.powi(2);
            measured += 0.5 * (f1 + f2) * dq;
        }

        // High-Q extrapolation via Porod: integral(K_p * Q^(-4) * Q^2 dQ) = integral(K_p * Q^(-2) dQ)
        let high_q = if let Some(kp) = porod_constant {
            let q_max = sorted.last().unwrap().q;
            // integral from q_max to infinity of K_p * Q^-2 dQ = K_p / q_max
            kp / q_max
        } else {
            0.0
        };

        let invariant = low_q + measured + high_q;

        // Volume fraction: Q* = 2*pi^2 * phi * (1-phi) * (delta_rho)^2
        // phi*(1-phi) = Q* / (2*pi^2 * delta_rho^2)
        let volume_fraction = contrast_sq.and_then(|dr2| {
            let product = invariant / (2.0 * PI * PI * dr2);
            if product < 0.0 || product > 0.25 {
                None
            } else {
                // phi*(1-phi) = product => phi = (1 - sqrt(1 - 4*product)) / 2
                let discriminant = 1.0 - 4.0 * product;
                if discriminant < 0.0 {
                    None
                } else {
                    Some((1.0 - discriminant.sqrt()) / 2.0)
                }
            }
        });

        Some(InvariantResult {
            invariant,
            volume_fraction,
            low_q_contribution: low_q,
            measured_contribution: measured,
            high_q_contribution: high_q,
        })
    }

    /// Integrate Guinier function * Q^2 from q_lo to q_hi.
    ///
    /// Uses numerical integration (Simpson's rule).
    fn guinier_integral(i0: f64, rg: f64, q_lo: f64, q_hi: f64) -> f64 {
        let n = 100;
        let dq = (q_hi - q_lo) / n as f64;
        if dq <= 0.0 {
            return 0.0;
        }

        let mut sum = 0.0;
        for i in 0..=n {
            let q = q_lo + i as f64 * dq;
            let f = i0 * (-rg * rg * q * q / 3.0).exp() * q * q;
            let w = if i == 0 || i == n {
                1.0
            } else if i % 2 == 1 {
                4.0
            } else {
                2.0
            };
            sum += w * f;
        }
        sum * dq / 3.0
    }
}

// ---------------------------------------------------------------------------
// Kratky Plot Analyzer
// ---------------------------------------------------------------------------

/// Result of Kratky plot analysis.
#[derive(Debug, Clone, Copy)]
pub struct KratkyResult {
    /// Whether a bell-shaped peak is observed (globular particle).
    pub is_globular: bool,
    /// Q position of the Kratky peak (if present).
    pub peak_q: Option<f64>,
    /// Peak height I(Q)*Q^2 at the peak.
    pub peak_height: Option<f64>,
    /// Whether the plot plateaus at high Q (unfolded/disordered).
    pub is_unfolded: bool,
}

/// Kratky plot analysis: I(Q)*Q^2 vs Q.
///
/// - Globular particles: bell-shaped peak at Q ≈ sqrt(3)/Rg
/// - Gaussian chains: plateau at high Q
/// - Rod-like: monotonic increase
/// - Unfolded proteins: no clear peak, rising plateau
#[derive(Debug, Clone)]
pub struct KratkyPlotAnalyzer;

impl KratkyPlotAnalyzer {
    /// Generate Kratky plot data: (Q, I(Q)*Q^2).
    pub fn kratky_data(data: &[ScatteringPoint]) -> Vec<(f64, f64)> {
        data.iter().map(|p| (p.q, p.intensity * p.q * p.q)).collect()
    }

    /// Analyze Kratky plot for particle shape characteristics.
    pub fn analyze(data: &[ScatteringPoint]) -> KratkyResult {
        if data.len() < 5 {
            return KratkyResult {
                is_globular: false,
                peak_q: None,
                peak_height: None,
                is_unfolded: false,
            };
        }

        let kratky: Vec<(f64, f64)> = Self::kratky_data(data);

        // Find the maximum of I*Q^2
        let (peak_idx, peak_val) =
            kratky
                .iter()
                .enumerate()
                .fold((0, 0.0_f64), |(pi, pv), (i, &(_, v))| {
                    if v > pv {
                        (i, v)
                    } else {
                        (pi, pv)
                    }
                });

        // Check if peak is interior (not at the boundary)
        let is_interior_peak = peak_idx > 0 && peak_idx < kratky.len() - 1;

        // Check if the Kratky value decreases after the peak (globular)
        let decreases_after_peak = if is_interior_peak && peak_idx + 2 < kratky.len() {
            let tail_avg =
                kratky[kratky.len() - 3..].iter().map(|&(_, v)| v).sum::<f64>() / 3.0;
            tail_avg < 0.7 * peak_val
        } else {
            false
        };

        // Check for plateau (unfolded): relatively constant I*Q^2 in the second half
        let is_plateau = if kratky.len() >= 6 {
            let half = kratky.len() / 2;
            let second_half: Vec<f64> = kratky[half..].iter().map(|&(_, v)| v).collect();
            let mean = second_half.iter().sum::<f64>() / second_half.len() as f64;
            if mean > 1e-30 {
                let variance = second_half.iter().map(|&v| (v - mean).powi(2)).sum::<f64>()
                    / second_half.len() as f64;
                let cv = variance.sqrt() / mean;
                cv < 0.3
            } else {
                false
            }
        } else {
            false
        };

        let is_globular = is_interior_peak && decreases_after_peak;

        KratkyResult {
            is_globular,
            peak_q: if is_interior_peak {
                Some(kratky[peak_idx].0)
            } else {
                None
            },
            peak_height: if is_interior_peak {
                Some(peak_val)
            } else {
                None
            },
            is_unfolded: is_plateau && !is_globular,
        }
    }

    /// Dimensionless Kratky plot: (Q*Rg)^2 * I(Q)/I(0) vs Q*Rg.
    ///
    /// Normalizes by Rg and I(0) for comparison between different samples.
    pub fn dimensionless_kratky(
        data: &[ScatteringPoint],
        rg: f64,
        i0: f64,
    ) -> Vec<(f64, f64)> {
        if i0.abs() < 1e-30 {
            return Vec::new();
        }
        data.iter()
            .map(|p| {
                let qrg = p.q * rg;
                (qrg, qrg * qrg * p.intensity / i0)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Model Fitter — Least-Squares Fitting
// ---------------------------------------------------------------------------

/// Result of model fitting.
#[derive(Debug, Clone)]
pub struct FitResult {
    /// Best-fit parameters.
    pub parameters: Vec<f64>,
    /// Parameter names.
    pub names: Vec<String>,
    /// Chi-squared statistic.
    pub chi_squared: f64,
    /// Reduced chi-squared (chi^2 / DOF).
    pub reduced_chi_squared: f64,
    /// R-squared goodness of fit.
    pub r_squared: f64,
    /// Number of data points used.
    pub n_points: usize,
    /// Degrees of freedom.
    pub dof: usize,
}

/// Scattering model for fitting.
#[derive(Debug, Clone, Copy)]
pub enum ScatteringModel {
    /// Sphere: params = [scale, radius, background]
    Sphere,
    /// Gaussian coil: params = [scale, rg, background]
    GaussianCoil,
    /// Cylinder: params = [scale, radius, length, background]
    Cylinder,
    /// Power law: params = [scale, exponent, background]
    PowerLaw,
    /// Guinier + power law: params = [i0, rg, exponent, background]
    GuinierPowerLaw,
}

/// Least-squares model fitter for SANS data.
///
/// Uses Levenberg-Marquardt-style iterative optimization.
#[derive(Debug, Clone)]
pub struct ModelFitter {
    /// Maximum iterations.
    pub max_iter: usize,
    /// Convergence tolerance.
    pub tolerance: f64,
}

impl ModelFitter {
    /// Create a new fitter.
    pub fn new() -> Self {
        Self {
            max_iter: 200,
            tolerance: 1e-8,
        }
    }

    /// Evaluate a scattering model at given Q with given parameters.
    pub fn evaluate(model: &ScatteringModel, q: f64, params: &[f64]) -> f64 {
        match model {
            ScatteringModel::Sphere => {
                let (scale, radius, bg) = (params[0], params[1], params[2]);
                scale * FormFactorCalculator::sphere(q, radius) + bg
            }
            ScatteringModel::GaussianCoil => {
                let (scale, rg, bg) = (params[0], params[1], params[2]);
                scale * FormFactorCalculator::gaussian_coil(q, rg) + bg
            }
            ScatteringModel::Cylinder => {
                let (scale, radius, length, bg) = (params[0], params[1], params[2], params[3]);
                scale * FormFactorCalculator::cylinder(q, radius, length) + bg
            }
            ScatteringModel::PowerLaw => {
                let (scale, exponent, bg) = (params[0], params[1], params[2]);
                scale * q.powf(-exponent) + bg
            }
            ScatteringModel::GuinierPowerLaw => {
                let (i0, rg, exponent, bg) = (params[0], params[1], params[2], params[3]);
                let q_cross = (1.5 * exponent / (rg * rg)).sqrt();
                if q <= q_cross {
                    // Guinier regime
                    i0 * (-rg * rg * q * q / 3.0).exp() + bg
                } else {
                    // Power law regime with continuity
                    let i_cross = i0 * (-rg * rg * q_cross * q_cross / 3.0).exp();
                    let scale_pl = i_cross * q_cross.powf(exponent);
                    scale_pl * q.powf(-exponent) + bg
                }
            }
        }
    }

    /// Parameter names for a given model.
    pub fn param_names(model: &ScatteringModel) -> Vec<String> {
        match model {
            ScatteringModel::Sphere => {
                vec!["scale".into(), "radius".into(), "background".into()]
            }
            ScatteringModel::GaussianCoil => {
                vec!["scale".into(), "rg".into(), "background".into()]
            }
            ScatteringModel::Cylinder => vec![
                "scale".into(),
                "radius".into(),
                "length".into(),
                "background".into(),
            ],
            ScatteringModel::PowerLaw => {
                vec!["scale".into(), "exponent".into(), "background".into()]
            }
            ScatteringModel::GuinierPowerLaw => vec![
                "i0".into(),
                "rg".into(),
                "exponent".into(),
                "background".into(),
            ],
        }
    }

    /// Fit a model to scattering data using grid-search + local refinement.
    ///
    /// # Arguments
    /// * `model` — scattering model to fit
    /// * `data` — I(Q) data points with errors
    /// * `initial_params` — starting parameter values
    pub fn fit(
        &self,
        model: &ScatteringModel,
        data: &[ScatteringPoint],
        initial_params: &[f64],
    ) -> Option<FitResult> {
        if data.len() <= initial_params.len() {
            return None;
        }

        let n_params = initial_params.len();
        let mut params = initial_params.to_vec();

        // Coordinate descent optimization
        let mut best_chi2 = self.chi_squared(model, data, &params);

        for _iter in 0..self.max_iter {
            let prev_chi2 = best_chi2;

            for p in 0..n_params {
                // Try perturbations along each parameter
                let base = params[p];
                let step = if base.abs() > 1e-15 {
                    base.abs() * 0.01
                } else {
                    0.01
                };

                // Try positive step
                params[p] = base + step;
                let chi2_plus = self.chi_squared(model, data, &params);

                // Try negative step
                params[p] = base - step;
                let chi2_minus = self.chi_squared(model, data, &params);

                if chi2_plus < best_chi2 && chi2_plus <= chi2_minus {
                    params[p] = base + step;
                    best_chi2 = chi2_plus;
                    // Continue in this direction
                    for _j in 0..20 {
                        params[p] += step;
                        if params[p] < 0.0
                            && !matches!(model, ScatteringModel::PowerLaw | ScatteringModel::GuinierPowerLaw)
                        {
                            // Most SANS parameters are positive
                            params[p] -= step;
                            break;
                        }
                        let chi2_next = self.chi_squared(model, data, &params);
                        if chi2_next < best_chi2 {
                            best_chi2 = chi2_next;
                        } else {
                            params[p] -= step;
                            break;
                        }
                    }
                } else if chi2_minus < best_chi2 {
                    params[p] = base - step;
                    best_chi2 = chi2_minus;
                    // Continue in this direction
                    for _j in 0..20 {
                        params[p] -= step;
                        if params[p] < 0.0
                            && !matches!(model, ScatteringModel::PowerLaw | ScatteringModel::GuinierPowerLaw)
                        {
                            params[p] += step;
                            break;
                        }
                        let chi2_next = self.chi_squared(model, data, &params);
                        if chi2_next < best_chi2 {
                            best_chi2 = chi2_next;
                        } else {
                            params[p] += step;
                            break;
                        }
                    }
                } else {
                    params[p] = base; // restore
                }
            }

            // Convergence check
            if (prev_chi2 - best_chi2).abs() / (prev_chi2 + 1e-30) < self.tolerance {
                break;
            }
        }

        let n_points = data.len();
        let dof = n_points.saturating_sub(n_params);

        // R-squared
        let mean_i: f64 = data.iter().map(|p| p.intensity).sum::<f64>() / n_points as f64;
        let ss_tot: f64 = data.iter().map(|p| (p.intensity - mean_i).powi(2)).sum();
        let ss_res: f64 = data
            .iter()
            .map(|p| {
                let y_fit = Self::evaluate(model, p.q, &params);
                (p.intensity - y_fit).powi(2)
            })
            .sum();
        let r_squared = if ss_tot > 1e-30 {
            1.0 - ss_res / ss_tot
        } else {
            0.0
        };

        Some(FitResult {
            parameters: params,
            names: Self::param_names(model),
            chi_squared: best_chi2,
            reduced_chi_squared: if dof > 0 {
                best_chi2 / dof as f64
            } else {
                best_chi2
            },
            r_squared,
            n_points,
            dof,
        })
    }

    /// Compute chi-squared statistic.
    fn chi_squared(&self, model: &ScatteringModel, data: &[ScatteringPoint], params: &[f64]) -> f64 {
        data.iter()
            .map(|p| {
                let y_calc = Self::evaluate(model, p.q, params);
                let residual = p.intensity - y_calc;
                let sigma = if p.error > 1e-30 { p.error } else { 1.0 };
                (residual / sigma).powi(2)
            })
            .sum()
    }
}

impl Default for ModelFitter {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Utility: generate synthetic scattering data
// ---------------------------------------------------------------------------

/// Generate synthetic SANS data from a model for testing.
///
/// Returns data points with Poisson-like noise.
pub fn generate_synthetic_data(
    shape: &ParticleShape,
    scale: f64,
    background: f64,
    q_min: f64,
    q_max: f64,
    n_points: usize,
) -> Vec<ScatteringPoint> {
    let mut data = Vec::with_capacity(n_points);
    let log_q_min = q_min.ln();
    let log_q_max = q_max.ln();
    let step = (log_q_max - log_q_min) / (n_points - 1).max(1) as f64;

    for i in 0..n_points {
        let q = (log_q_min + i as f64 * step).exp();
        let pq = FormFactorCalculator::compute(shape, q);
        let intensity = scale * pq + background;
        // Error estimate: ~sqrt(intensity) / sqrt(exposure)
        let error = (intensity.abs().sqrt()).max(background * 0.01);
        data.push(ScatteringPoint {
            q,
            intensity,
            error,
        });
    }
    data
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;

    // --- SansConfig tests ---

    #[test]
    fn test_sans_config_creation() {
        let config = SansConfig::new(6.0, 4.0, 0.005);
        assert!((config.wavelength - 6.0).abs() < TOL);
        assert!((config.sdd - 4.0).abs() < TOL);
        assert!((config.pixel_size - 0.005).abs() < TOL);
        assert!(config.q_min > 0.0);
        assert!(config.q_max > config.q_min);
    }

    #[test]
    fn test_sans_config_q_range() {
        let config = SansConfig::new(6.0, 4.0, 0.005).with_q_range(0.01, 0.5);
        assert!((config.q_min - 0.01).abs() < TOL);
        assert!((config.q_max - 0.5).abs() < TOL);
    }

    #[test]
    fn test_sans_config_wavelength_spread() {
        let config = SansConfig::new(6.0, 4.0, 0.005).with_wavelength_spread(0.15);
        assert!((config.wavelength_spread - 0.15).abs() < TOL);
    }

    // --- QCalculator tests ---

    #[test]
    fn test_q_from_angle_zero() {
        let config = SansConfig::new(6.0, 4.0, 0.005);
        let calc = QCalculator::new(&config);
        assert!((calc.q_from_angle(0.0)).abs() < TOL);
    }

    #[test]
    fn test_q_from_angle_formula() {
        let config = SansConfig::new(6.0, 4.0, 0.005);
        let calc = QCalculator::new(&config);
        let theta: f64 = 0.02; // small angle in radians
        let expected = (4.0_f64 * PI / 6.0) * (theta / 2.0_f64).sin();
        let q = calc.q_from_angle(theta);
        assert!((q - expected).abs() < TOL);
    }

    #[test]
    fn test_q_from_pixel() {
        let config = SansConfig::new(6.0, 4.0, 0.005);
        let calc = QCalculator::new(&config);
        let q = calc.q_from_pixel(100.0);
        // r = 100 * 0.005 = 0.5 m, theta = atan(0.5/4) ≈ 0.1244
        assert!(q > 0.0);
        assert!(q < 1.0);
    }

    #[test]
    fn test_q_from_distance() {
        let config = SansConfig::new(6.0, 4.0, 0.005);
        let calc = QCalculator::new(&config);
        let q = calc.q_from_distance(0.5);
        // Same as pixel offset 100
        let q_pixel = calc.q_from_pixel(100.0);
        assert!((q - q_pixel).abs() < TOL);
    }

    #[test]
    fn test_angle_from_q_roundtrip() {
        let config = SansConfig::new(6.0, 4.0, 0.005);
        let calc = QCalculator::new(&config);
        let theta_in = 0.05;
        let q = calc.q_from_angle(theta_in);
        let theta_out = calc.angle_from_q(q);
        assert!((theta_in - theta_out).abs() < 1e-10);
    }

    #[test]
    fn test_q_resolution() {
        let config = SansConfig::new(6.0, 4.0, 0.005);
        let calc = QCalculator::new(&config);
        let q = 0.05;
        let sigma_q = calc.q_resolution(q, 0.10, 0.005);
        assert!(sigma_q > 0.0);
        assert!(sigma_q < q);
    }

    // --- DataReducer tests ---

    #[test]
    fn test_solid_angle_at_center() {
        let config = SansConfig::new(6.0, 4.0, 0.005);
        let reducer = DataReducer::new(&config);
        let omega = reducer.solid_angle(0.0);
        // At center: cos^3(0) = 1, omega = pixel_size^2 / SDD^2
        let expected = 0.005 * 0.005 / (4.0 * 4.0);
        assert!((omega - expected).abs() < TOL);
    }

    #[test]
    fn test_solid_angle_decreases_with_r() {
        let config = SansConfig::new(6.0, 4.0, 0.005);
        let reducer = DataReducer::new(&config);
        let omega_center = reducer.solid_angle(0.0);
        let omega_edge = reducer.solid_angle(0.5);
        assert!(omega_edge < omega_center);
    }

    #[test]
    fn test_reduce_pixel_basic() {
        let config = SansConfig::new(6.0, 4.0, 0.005);
        let reducer = DataReducer::new(&config);
        let (intensity, error) = reducer.reduce_pixel(1000.0, 100.0, 1e6, 0.9, 0.1);
        assert!(intensity > 0.0);
        assert!(error > 0.0);
    }

    #[test]
    fn test_reduce_pixel_zero_monitor() {
        let config = SansConfig::new(6.0, 4.0, 0.005);
        let reducer = DataReducer::new(&config);
        let (intensity, _error) = reducer.reduce_pixel(1000.0, 100.0, 0.0, 0.9, 0.1);
        assert!((intensity).abs() < TOL);
    }

    // --- Form Factor tests ---

    #[test]
    fn test_sphere_form_factor_at_zero() {
        let pq = FormFactorCalculator::sphere(0.0, 50.0);
        assert!((pq - 1.0).abs() < TOL);
    }

    #[test]
    fn test_sphere_form_factor_positive() {
        for q in [0.01, 0.05, 0.1, 0.2] {
            let pq = FormFactorCalculator::sphere(q, 50.0);
            assert!(pq >= 0.0, "P(Q) must be >= 0 at Q={}", q);
            assert!(pq <= 1.0, "P(Q) must be <= 1 at Q={}", q);
        }
    }

    #[test]
    fn test_sphere_form_factor_first_minimum() {
        // First zero of sphere form factor at QR ≈ 4.493
        let r = 50.0;
        let q_zero = 4.493 / r;
        let pq = FormFactorCalculator::sphere(q_zero, r);
        assert!(pq < 0.001, "P(Q) should be near zero at first minimum: {}", pq);
    }

    #[test]
    fn test_gaussian_coil_at_zero() {
        let pq = FormFactorCalculator::gaussian_coil(0.0, 100.0);
        assert!((pq - 1.0).abs() < TOL);
    }

    #[test]
    fn test_gaussian_coil_debye_function() {
        // At Q*Rg = 1, Debye function = 2*(e^(-1) - 1 + 1)/1 = 2*e^(-1) ≈ 0.7358
        let rg = 100.0;
        let q = 1.0 / rg;
        let pq = FormFactorCalculator::gaussian_coil(q, rg);
        let expected = 2.0 * ((-1.0_f64).exp());
        assert!(
            (pq - expected).abs() < 0.01,
            "Debye at Q*Rg=1: got {}, expected {}",
            pq,
            expected
        );
    }

    #[test]
    fn test_cylinder_at_zero() {
        let pq = FormFactorCalculator::cylinder(0.0, 20.0, 100.0);
        assert!((pq - 1.0).abs() < 0.05); // numerical integration tolerance
    }

    #[test]
    fn test_cylinder_positive() {
        for q in [0.01, 0.05, 0.1] {
            let pq = FormFactorCalculator::cylinder(q, 20.0, 100.0);
            assert!(pq >= 0.0);
            assert!(pq <= 1.1); // slightly above 1 possible from orientation averaging
        }
    }

    #[test]
    fn test_ellipsoid_sphere_limit() {
        // When a = b, ellipsoid should equal sphere
        let r = 50.0;
        let q = 0.05;
        let pq_sphere = FormFactorCalculator::sphere(q, r);
        let pq_ellipsoid = FormFactorCalculator::ellipsoid(q, r, r);
        assert!(
            (pq_sphere - pq_ellipsoid).abs() < 0.02,
            "Ellipsoid(a=b) should match sphere: {} vs {}",
            pq_ellipsoid,
            pq_sphere
        );
    }

    #[test]
    fn test_core_shell_at_zero() {
        let pq = FormFactorCalculator::core_shell(0.0, 30.0, 50.0, 0.5);
        assert!((pq - 1.0).abs() < TOL);
    }

    #[test]
    fn test_form_factor_compute_range() {
        let shape = ParticleShape::Sphere(50.0);
        let q_values = vec![0.001, 0.01, 0.05, 0.1, 0.2];
        let pq = FormFactorCalculator::compute_range(&shape, &q_values);
        assert_eq!(pq.len(), 5);
        // Should be monotonically decreasing for sphere
        for i in 1..pq.len() {
            assert!(pq[i] <= pq[i - 1] + 0.01); // small tolerance for numerical
        }
    }

    // --- Guinier Analyzer tests ---

    #[test]
    fn test_guinier_with_sphere_data() {
        let shape = ParticleShape::Sphere(50.0);
        let data = generate_synthetic_data(&shape, 100.0, 0.0, 0.001, 0.05, 50);
        let analyzer = GuinierAnalyzer::new();
        let result = analyzer.fit(&data).unwrap();
        // Rg for a sphere of radius R is R * sqrt(3/5)
        let expected_rg = 50.0 * (3.0_f64 / 5.0).sqrt();
        assert!(
            (result.rg - expected_rg).abs() / expected_rg < 0.1,
            "Guinier Rg: expected ~{:.1}, got {:.1}",
            expected_rg,
            result.rg
        );
        assert!(result.r_squared > 0.9);
    }

    #[test]
    fn test_guinier_with_coil_data() {
        let rg = 80.0;
        let shape = ParticleShape::GaussianCoil(rg);
        let data = generate_synthetic_data(&shape, 100.0, 0.0, 0.001, 0.02, 50);
        let analyzer = GuinierAnalyzer::new();
        let result = analyzer.fit(&data).unwrap();
        assert!(
            (result.rg - rg).abs() / rg < 0.15,
            "Guinier Rg for coil: expected ~{:.1}, got {:.1}",
            rg,
            result.rg
        );
    }

    #[test]
    fn test_guinier_insufficient_data() {
        let analyzer = GuinierAnalyzer::new();
        let result = analyzer.fit(&[]);
        assert!(result.is_none());
    }

    #[test]
    fn test_guinier_q_rg_max() {
        let shape = ParticleShape::Sphere(50.0);
        let data = generate_synthetic_data(&shape, 100.0, 0.0, 0.001, 0.05, 50);
        let analyzer = GuinierAnalyzer::with_limit(1.0);
        let result = analyzer.fit(&data).unwrap();
        assert!(result.q_rg_max <= 1.3); // should be within limit
    }

    // --- Porod Analyzer tests ---

    #[test]
    fn test_porod_power_law_fit() {
        // Generate Q^-4 data
        let data: Vec<ScatteringPoint> = (0..50)
            .map(|i| {
                let q = 0.1 + i as f64 * 0.02;
                ScatteringPoint {
                    q,
                    intensity: 1e5 * q.powi(-4),
                    error: 1.0,
                }
            })
            .collect();

        let analyzer = PorodAnalyzer;
        let result = analyzer.fit(&data, 0.1, 1.0, None).unwrap();
        assert!(
            (result.exponent - 4.0).abs() < 0.1,
            "Porod exponent: expected ~4, got {:.2}",
            result.exponent
        );
        assert!(result.r_squared > 0.99);
    }

    #[test]
    fn test_porod_plot() {
        let data = vec![
            ScatteringPoint { q: 0.1, intensity: 100.0, error: 1.0 },
            ScatteringPoint { q: 0.2, intensity: 6.25, error: 0.5 },
        ];
        let plot = PorodAnalyzer::porod_plot(&data);
        assert_eq!(plot.len(), 2);
        // I*Q^4: 100 * 0.1^4 = 0.01, 6.25 * 0.2^4 = 0.01
        assert!((plot[0].1 - 0.01).abs() < 0.001);
        assert!((plot[1].1 - 0.01).abs() < 0.001);
    }

    // --- Structure Factor tests ---

    #[test]
    fn test_structure_factor_dilute_limit() {
        // For dilute system (no interactions), S(Q) ≈ 1
        let shape = ParticleShape::Sphere(50.0);
        let data = generate_synthetic_data(&shape, 100.0, 0.0, 0.01, 0.5, 30);
        let result = StructureFactorEstimator::extract(&data, &shape, Some(100.0));
        // At all Q, S(Q) should be ~ 1
        for &sq in &result.s_q {
            assert!(
                (sq - 1.0).abs() < 0.1,
                "S(Q) should be ~1 for dilute system: got {}",
                sq
            );
        }
    }

    #[test]
    fn test_percus_yevick_low_eta() {
        // At very low volume fraction, S(Q) ≈ 1
        let sq = StructureFactorEstimator::percus_yevick(0.1, 50.0, 0.001);
        assert!(
            (sq - 1.0).abs() < 0.1,
            "PY at low eta should be ~1: got {}",
            sq
        );
    }

    #[test]
    fn test_percus_yevick_high_q() {
        // At high Q, S(Q) → 1
        let sq = StructureFactorEstimator::percus_yevick(10.0, 50.0, 0.2);
        assert!(
            (sq - 1.0).abs() < 0.5,
            "PY at high Q should approach 1: got {}",
            sq
        );
    }

    #[test]
    fn test_percus_yevick_zero_q() {
        // At Q=0, S(0) = (1-eta)^4 / (1+2*eta)^2 for hard spheres (compressibility)
        let eta = 0.2;
        let sq = StructureFactorEstimator::percus_yevick(1e-10, 50.0, eta);
        let expected = (1.0 - eta).powi(4) / (1.0 + 2.0 * eta).powi(2);
        assert!(
            (sq - expected).abs() / expected < 0.05,
            "PY at Q=0: got {}, expected {}",
            sq,
            expected
        );
    }

    // --- Invariant Calculator tests ---

    #[test]
    fn test_invariant_basic() {
        let data: Vec<ScatteringPoint> = (0..100)
            .map(|i| {
                let q = 0.01 + i as f64 * 0.01;
                ScatteringPoint {
                    q,
                    intensity: 100.0 * (-50.0 * q * q).exp(),
                    error: 1.0,
                }
            })
            .collect();

        let result = InvariantCalculator::compute(&data, None, None, None).unwrap();
        assert!(result.invariant > 0.0);
        assert!(result.measured_contribution > 0.0);
    }

    #[test]
    fn test_invariant_with_extrapolations() {
        let data: Vec<ScatteringPoint> = (0..50)
            .map(|i| {
                let q = 0.01 + i as f64 * 0.01;
                ScatteringPoint {
                    q,
                    intensity: 100.0 * (-50.0 * q * q).exp(),
                    error: 1.0,
                }
            })
            .collect();

        let guinier = GuinierResult {
            rg: 30.0,
            i0: 100.0,
            r_squared: 0.99,
            q_rg_max: 1.0,
        };

        let result = InvariantCalculator::compute(&data, Some(&guinier), Some(1.0), None).unwrap();
        assert!(result.low_q_contribution > 0.0);
        assert!(result.high_q_contribution > 0.0);
        assert!(result.invariant > result.measured_contribution);
    }

    #[test]
    fn test_invariant_insufficient_data() {
        let result = InvariantCalculator::compute(&[], None, None, None);
        assert!(result.is_none());
    }

    // --- Kratky Plot tests ---

    #[test]
    fn test_kratky_globular_particle() {
        // Sphere should give bell-shaped Kratky
        let shape = ParticleShape::Sphere(50.0);
        let data = generate_synthetic_data(&shape, 100.0, 0.001, 0.005, 0.3, 100);
        let result = KratkyPlotAnalyzer::analyze(&data);
        assert!(result.is_globular, "Sphere should be identified as globular");
        assert!(result.peak_q.is_some());
    }

    #[test]
    fn test_kratky_data_generation() {
        let data = vec![
            ScatteringPoint { q: 0.01, intensity: 100.0, error: 1.0 },
            ScatteringPoint { q: 0.1, intensity: 10.0, error: 0.5 },
        ];
        let kratky = KratkyPlotAnalyzer::kratky_data(&data);
        assert_eq!(kratky.len(), 2);
        assert!((kratky[0].1 - 100.0 * 0.01 * 0.01).abs() < TOL);
        assert!((kratky[1].1 - 10.0 * 0.1 * 0.1).abs() < TOL);
    }

    #[test]
    fn test_dimensionless_kratky() {
        let data = vec![
            ScatteringPoint { q: 0.01, intensity: 100.0, error: 1.0 },
            ScatteringPoint { q: 0.05, intensity: 80.0, error: 1.0 },
        ];
        let dk = KratkyPlotAnalyzer::dimensionless_kratky(&data, 50.0, 100.0);
        assert_eq!(dk.len(), 2);
        // First point: Q*Rg = 0.5, (Q*Rg)^2 * I/I0 = 0.25 * 1.0 = 0.25
        assert!((dk[0].0 - 0.5).abs() < TOL);
        assert!((dk[0].1 - 0.25).abs() < TOL);
    }

    // --- Model Fitter tests ---

    #[test]
    fn test_model_evaluate_sphere() {
        let i = ModelFitter::evaluate(&ScatteringModel::Sphere, 0.0, &[100.0, 50.0, 0.01]);
        assert!((i - 100.01).abs() < 0.1); // scale*P(0) + bg = 100 + 0.01
    }

    #[test]
    fn test_model_evaluate_power_law() {
        let q = 0.1;
        let i = ModelFitter::evaluate(&ScatteringModel::PowerLaw, q, &[1e5, 4.0, 0.0]);
        let expected = 1e5 * q.powf(-4.0);
        assert!((i - expected).abs() / expected < TOL);
    }

    #[test]
    fn test_model_fit_sphere() {
        let shape = ParticleShape::Sphere(50.0);
        let data = generate_synthetic_data(&shape, 100.0, 0.01, 0.005, 0.3, 80);
        let fitter = ModelFitter::new();
        let result = fitter
            .fit(&ScatteringModel::Sphere, &data, &[80.0, 40.0, 0.005])
            .unwrap();

        // Check that the fit found reasonable parameters
        assert!(
            result.parameters[1] > 30.0 && result.parameters[1] < 70.0,
            "Fitted radius: {:.1} (expected ~50)",
            result.parameters[1]
        );
        assert!(result.r_squared > 0.8, "R^2 = {}", result.r_squared);
    }

    #[test]
    fn test_model_fit_gaussian_coil() {
        let rg = 80.0;
        let shape = ParticleShape::GaussianCoil(rg);
        let data = generate_synthetic_data(&shape, 50.0, 0.01, 0.001, 0.1, 60);
        let fitter = ModelFitter::new();
        let result = fitter
            .fit(&ScatteringModel::GaussianCoil, &data, &[40.0, 60.0, 0.005])
            .unwrap();

        assert!(result.r_squared > 0.8, "R^2 = {}", result.r_squared);
    }

    #[test]
    fn test_model_param_names() {
        let names = ModelFitter::param_names(&ScatteringModel::Sphere);
        assert_eq!(names, vec!["scale", "radius", "background"]);

        let names = ModelFitter::param_names(&ScatteringModel::Cylinder);
        assert_eq!(names, vec!["scale", "radius", "length", "background"]);
    }

    #[test]
    fn test_chi_squared() {
        let data = vec![
            ScatteringPoint { q: 0.01, intensity: 100.0, error: 10.0 },
            ScatteringPoint { q: 0.02, intensity: 99.0, error: 10.0 },
        ];
        let fitter = ModelFitter::new();
        let chi2 = fitter.chi_squared(
            &ScatteringModel::Sphere,
            &data,
            &[100.0, 50.0, 0.0],
        );
        assert!(chi2 >= 0.0);
    }

    // --- Bessel J1 tests ---

    #[test]
    fn test_bessel_j1_at_zero() {
        assert!(bessel_j1(0.0).abs() < TOL);
    }

    #[test]
    fn test_bessel_j1_known_values() {
        // J1(3.8317) ≈ 0 (first zero of J1)
        assert!(bessel_j1(3.8317).abs() < 0.01);
        // J1(1.0) ≈ 0.4401
        assert!((bessel_j1(1.0) - 0.4401).abs() < 0.01);
    }

    #[test]
    fn test_bessel_j1_large_argument() {
        // J1 should remain bounded for large arguments
        let j = bessel_j1(100.0);
        assert!(j.abs() < 1.0);
    }

    #[test]
    fn test_bessel_j1_negative() {
        // J1(-x) = -J1(x) (odd function)
        let x = 2.5;
        assert!((bessel_j1(-x) + bessel_j1(x)).abs() < 1e-10);
    }

    // --- Synthetic data tests ---

    #[test]
    fn test_generate_synthetic_data() {
        let shape = ParticleShape::Sphere(50.0);
        let data = generate_synthetic_data(&shape, 100.0, 0.01, 0.01, 0.5, 50);
        assert_eq!(data.len(), 50);
        // Q values should be in range
        assert!(data[0].q >= 0.009);
        assert!(data[49].q <= 0.6);
        // Intensities should be positive
        for p in &data {
            assert!(p.intensity > 0.0);
            assert!(p.error > 0.0);
        }
    }

    #[test]
    fn test_generate_synthetic_data_log_spacing() {
        let shape = ParticleShape::Sphere(50.0);
        let data = generate_synthetic_data(&shape, 100.0, 0.01, 0.01, 1.0, 100);
        // Check that Q values are roughly logarithmically spaced
        let ratio1 = data[1].q / data[0].q;
        let ratio_mid = data[51].q / data[50].q;
        assert!((ratio1 - ratio_mid).abs() / ratio1 < 0.01);
    }

    // --- Azimuthal average test ---

    #[test]
    fn test_azimuthal_average() {
        let config = SansConfig::new(6.0, 4.0, 0.005);
        let reducer = DataReducer::new(&config);

        // Generate ring of pixels at constant radius
        let mut pixels = Vec::new();
        for angle_deg in (0..360).step_by(10) {
            let angle = angle_deg as f64 * PI / 180.0;
            let r_pixels = 50.0;
            let px = r_pixels * angle.cos();
            let py = r_pixels * angle.sin();
            pixels.push((px, py, 1000.0));
        }

        let result = reducer.azimuthal_average(&pixels, 10.0, 1e6, 0.9, 20, 0.001, 1.0);
        assert!(!result.is_empty());
    }

    // --- GuinierPowerLaw model test ---

    #[test]
    fn test_guinier_power_law_continuity() {
        let rg: f64 = 50.0;
        let exponent: f64 = 4.0;
        let i0: f64 = 100.0;
        let params = [i0, rg, exponent, 0.0];
        let q_cross = (1.5_f64 * exponent / (rg * rg)).sqrt();

        // Values just below and above crossover should be close
        let i_below = ModelFitter::evaluate(&ScatteringModel::GuinierPowerLaw, q_cross * 0.999, &params);
        let i_above = ModelFitter::evaluate(&ScatteringModel::GuinierPowerLaw, q_cross * 1.001, &params);
        assert!(
            (i_below - i_above).abs() / i_below < 0.01,
            "Discontinuity at crossover: {} vs {}",
            i_below,
            i_above
        );
    }

    // --- Porod with surface area test ---

    #[test]
    fn test_porod_specific_surface() {
        let data: Vec<ScatteringPoint> = (0..50)
            .map(|i| {
                let q = 0.1 + i as f64 * 0.02;
                ScatteringPoint {
                    q,
                    intensity: 1e5 * q.powi(-4),
                    error: 1.0,
                }
            })
            .collect();

        let analyzer = PorodAnalyzer;
        let contrast_sq = 1e-12; // cm^-4
        let result = analyzer.fit(&data, 0.1, 1.0, Some(contrast_sq)).unwrap();
        assert!(result.specific_surface.is_some());
        assert!(result.specific_surface.unwrap() > 0.0);
    }
}
