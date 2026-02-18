//! # Small-Angle X-ray Scattering (SAXS) Analyzer
//!
//! Implements SAXS data processing for analyzing nanostructure in materials
//! by processing 1D scattering intensity profiles I(q) where q = 4pi sin(theta)/lambda.
//!
//! ## Key Features
//! - Guinier analysis for radius of gyration (Rg)
//! - Porod analysis for surface area
//! - Kratky plot for particle shape classification
//! - Analytical form factor models (sphere, cylinder, ellipsoid, Gaussian chain)
//! - Structure factor (Percus-Yevick hard sphere)
//! - Indirect Fourier Transform for P(r) pair distance distribution
//! - Absolute intensity scaling
//! - Polydisperse size distribution fitting

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// SaxsProfile
// ---------------------------------------------------------------------------

/// A 1D SAXS scattering intensity profile I(q).
#[derive(Debug, Clone)]
pub struct SaxsProfile {
    /// Scattering vector magnitudes in 1/Angstrom.
    pub q_values: Vec<f64>,
    /// Scattering intensities in arbitrary units.
    pub intensities: Vec<f64>,
    /// Optional measurement uncertainties (standard deviations).
    pub errors: Option<Vec<f64>>,
}

impl SaxsProfile {
    /// Create a new SAXS profile from q values and intensities.
    pub fn new(q_values: Vec<f64>, intensities: Vec<f64>) -> Self {
        assert_eq!(q_values.len(), intensities.len(), "q and I arrays must have equal length");
        assert!(!q_values.is_empty(), "Profile must have at least one data point");
        Self {
            q_values,
            intensities,
            errors: None,
        }
    }

    /// Create a SAXS profile with error bars.
    pub fn with_errors(q_values: Vec<f64>, intensities: Vec<f64>, sigma: Vec<f64>) -> Self {
        assert_eq!(q_values.len(), intensities.len(), "q and I arrays must have equal length");
        assert_eq!(q_values.len(), sigma.len(), "q and sigma arrays must have equal length");
        assert!(!q_values.is_empty(), "Profile must have at least one data point");
        Self {
            q_values,
            intensities,
            errors: Some(sigma),
        }
    }

    /// Return the q range (min, max).
    pub fn q_range(&self) -> (f64, f64) {
        let mut min = f64::MAX;
        let mut max = f64::MIN;
        for &q in &self.q_values {
            if q < min {
                min = q;
            }
            if q > max {
                max = q;
            }
        }
        (min, max)
    }

    /// Linear interpolation of I(q) at an arbitrary q value.
    pub fn interpolate(&self, q: f64) -> f64 {
        let n = self.q_values.len();
        if n == 0 {
            return 0.0;
        }
        if n == 1 || q <= self.q_values[0] {
            return self.intensities[0];
        }
        if q >= self.q_values[n - 1] {
            return self.intensities[n - 1];
        }
        // Find bracketing indices via linear scan
        for i in 0..n - 1 {
            if self.q_values[i] <= q && q <= self.q_values[i + 1] {
                let dq = self.q_values[i + 1] - self.q_values[i];
                if dq.abs() < 1e-30 {
                    return self.intensities[i];
                }
                let t = (q - self.q_values[i]) / dq;
                return self.intensities[i] * (1.0 - t) + self.intensities[i + 1] * t;
            }
        }
        self.intensities[n - 1]
    }

    /// Number of data points.
    pub fn len(&self) -> usize {
        self.q_values.len()
    }

    /// Whether the profile is empty.
    pub fn is_empty(&self) -> bool {
        self.q_values.is_empty()
    }

    /// Extract a sub-range of the profile where q_min <= q <= q_max.
    pub fn slice_q_range(&self, q_min: f64, q_max: f64) -> SaxsProfile {
        let mut q_out = Vec::new();
        let mut i_out = Vec::new();
        let mut e_out = Vec::new();
        let has_errors = self.errors.is_some();
        for idx in 0..self.q_values.len() {
            let q = self.q_values[idx];
            if q >= q_min && q <= q_max {
                q_out.push(q);
                i_out.push(self.intensities[idx]);
                if has_errors {
                    e_out.push(self.errors.as_ref().unwrap()[idx]);
                }
            }
        }
        if has_errors && !e_out.is_empty() {
            SaxsProfile::with_errors(q_out, i_out, e_out)
        } else {
            SaxsProfile::new(q_out, i_out)
        }
    }
}

// ---------------------------------------------------------------------------
// BackgroundSubtraction
// ---------------------------------------------------------------------------

/// Background subtraction methods for SAXS data.
pub struct BackgroundSubtraction;

impl BackgroundSubtraction {
    /// Subtract a constant background level from all intensities.
    pub fn subtract_constant(profile: &SaxsProfile, bg: f64) -> SaxsProfile {
        let intensities: Vec<f64> = profile.intensities.iter().map(|&i| (i - bg).max(0.0)).collect();
        SaxsProfile {
            q_values: profile.q_values.clone(),
            intensities,
            errors: profile.errors.clone(),
        }
    }

    /// Subtract a buffer profile scaled by transmission ratio.
    /// I_corrected = I_sample - transmission * I_buffer
    pub fn subtract_profile(
        sample: &SaxsProfile,
        buffer: &SaxsProfile,
        transmission: f64,
    ) -> SaxsProfile {
        let intensities: Vec<f64> = sample
            .q_values
            .iter()
            .enumerate()
            .map(|(idx, &q)| {
                let buf_i = buffer.interpolate(q);
                (sample.intensities[idx] - transmission * buf_i).max(0.0)
            })
            .collect();
        SaxsProfile {
            q_values: sample.q_values.clone(),
            intensities,
            errors: sample.errors.clone(),
        }
    }

    /// Automatically estimate background from the high-q tail.
    /// Takes the mean of the last 10% of data points (or last 3 at minimum).
    pub fn auto_background(profile: &SaxsProfile) -> f64 {
        let n = profile.intensities.len();
        let tail_count = (n / 10).max(3).min(n);
        let start = n - tail_count;
        let sum: f64 = profile.intensities[start..].iter().sum();
        sum / tail_count as f64
    }
}

// ---------------------------------------------------------------------------
// GuinierAnalysis
// ---------------------------------------------------------------------------

/// Result of a Guinier fit.
#[derive(Debug, Clone, Copy)]
pub struct GuinierResult {
    /// Radius of gyration in Angstroms.
    pub rg: f64,
    /// Forward scattering intensity I(0).
    pub i0: f64,
    /// Quality of fit (R-squared).
    pub quality: f64,
}

/// Guinier analysis: I(q) ~ I(0) exp(-q^2 Rg^2 / 3) for qRg < 1.3.
/// Fit ln(I) vs q^2 as a straight line: ln(I) = ln(I0) - Rg^2/3 * q^2.
pub struct GuinierAnalysis;

impl GuinierAnalysis {
    /// Perform Guinier fit on the specified q range.
    pub fn fit(profile: &SaxsProfile, q_min: f64, q_max: f64) -> GuinierResult {
        // Collect data points in [q_min, q_max] with positive intensity
        let mut x_vals = Vec::new(); // q^2
        let mut y_vals = Vec::new(); // ln(I)
        for i in 0..profile.q_values.len() {
            let q = profile.q_values[i];
            let intensity = profile.intensities[i];
            if q >= q_min && q <= q_max && intensity > 0.0 {
                x_vals.push(q * q);
                y_vals.push(intensity.ln());
            }
        }
        assert!(x_vals.len() >= 2, "Need at least 2 points for Guinier fit");
        // Linear least squares: y = a + b*x where a = ln(I0), b = -Rg^2/3
        let (a, b, r_squared) = linear_regression(&x_vals, &y_vals);
        let i0 = a.exp();
        let rg_sq = -3.0 * b;
        let rg = if rg_sq > 0.0 { rg_sq.sqrt() } else { 0.0 };
        GuinierResult {
            rg,
            i0,
            quality: r_squared,
        }
    }

    /// Generate Guinier plot data: (q^2, ln(I)) pairs.
    pub fn guinier_plot(profile: &SaxsProfile) -> Vec<(f64, f64)> {
        profile
            .q_values
            .iter()
            .zip(profile.intensities.iter())
            .filter(|(_, &i)| i > 0.0)
            .map(|(&q, &i)| (q * q, i.ln()))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// PorodAnalysis
// ---------------------------------------------------------------------------

/// Result of a Porod fit.
#[derive(Debug, Clone, Copy)]
pub struct PorodResult {
    /// Porod constant K_p.
    pub porod_constant: f64,
    /// Surface-to-volume ratio S/V = pi * K_p / Q where Q is the invariant.
    pub surface_to_volume: f64,
}

/// Porod analysis: I(q) -> K_p / q^4 as q -> infinity.
pub struct PorodAnalysis;

impl PorodAnalysis {
    /// Fit the Porod region to extract K_p.
    /// In the Porod region: I(q)*q^4 = K_p (constant).
    pub fn fit(profile: &SaxsProfile, q_min: f64, q_max: f64) -> PorodResult {
        let mut sum = 0.0;
        let mut count = 0;
        for i in 0..profile.q_values.len() {
            let q = profile.q_values[i];
            if q >= q_min && q <= q_max && q > 0.0 {
                let q4 = q.powi(4);
                sum += profile.intensities[i] * q4;
                count += 1;
            }
        }
        assert!(count > 0, "No data points in Porod region");
        let porod_constant = sum / count as f64;
        // S/V = pi * K_p / Q; compute Q (invariant) over entire profile
        let invariant = Self::invariant(profile);
        let surface_to_volume = if invariant > 0.0 {
            PI * porod_constant / invariant
        } else {
            0.0
        };
        PorodResult {
            porod_constant,
            surface_to_volume,
        }
    }

    /// Compute the scattering invariant Q = integral of I(q) * q^2 dq.
    /// Uses trapezoidal rule.
    pub fn invariant(profile: &SaxsProfile) -> f64 {
        let n = profile.q_values.len();
        if n < 2 {
            return 0.0;
        }
        let mut integral = 0.0;
        for i in 0..n - 1 {
            let q0 = profile.q_values[i];
            let q1 = profile.q_values[i + 1];
            let f0 = profile.intensities[i] * q0 * q0;
            let f1 = profile.intensities[i + 1] * q1 * q1;
            integral += 0.5 * (f0 + f1) * (q1 - q0);
        }
        integral
    }

    /// Porod plot data: (q, I(q)*q^4) pairs.
    pub fn porod_plot(profile: &SaxsProfile) -> Vec<(f64, f64)> {
        profile
            .q_values
            .iter()
            .zip(profile.intensities.iter())
            .map(|(&q, &i)| (q, i * q.powi(4)))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// KratkyPlot
// ---------------------------------------------------------------------------

/// Particle shape classification from Kratky analysis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ParticleShape {
    /// Compact, globular particle (bell-shaped Kratky plot with peak then decay).
    Globular,
    /// Extended, rod-like particle (Kratky plot increases then plateaus).
    Extended,
    /// Unfolded / flexible chain (Kratky plot increases monotonically).
    Unfolded,
}

/// Kratky plot: I(q)*q^2 vs q for assessing compactness.
pub struct KratkyPlot;

impl KratkyPlot {
    /// Compute the Kratky plot: returns (q, I(q)*q^2) pairs.
    pub fn compute(profile: &SaxsProfile) -> Vec<(f64, f64)> {
        profile
            .q_values
            .iter()
            .zip(profile.intensities.iter())
            .map(|(&q, &i)| (q, i * q * q))
            .collect()
    }

    /// Dimensionless Kratky plot: ((qRg), (qRg)^2 * I(q)/I(0)) pairs.
    pub fn compute_dimensionless(profile: &SaxsProfile, rg: f64, i0: f64) -> Vec<(f64, f64)> {
        profile
            .q_values
            .iter()
            .zip(profile.intensities.iter())
            .map(|(&q, &i)| {
                let qrg = q * rg;
                (qrg, qrg * qrg * i / i0)
            })
            .collect()
    }

    /// Classify particle shape from Kratky plot data.
    /// - Globular: peak followed by decay toward zero
    /// - Extended: plateau at high q
    /// - Unfolded: monotonically increasing
    pub fn classify_shape(kratky: &[(f64, f64)]) -> ParticleShape {
        if kratky.len() < 5 {
            return ParticleShape::Globular;
        }

        // Find the maximum I*q^2 value and its position
        let mut max_val = f64::MIN;
        let mut max_idx = 0;
        for (idx, &(_, y)) in kratky.iter().enumerate() {
            if y > max_val {
                max_val = y;
                max_idx = idx;
            }
        }

        // If peak is in the last 20% of data, likely still rising -> Unfolded
        let n = kratky.len();
        if max_idx > n * 4 / 5 {
            return ParticleShape::Unfolded;
        }

        // Check behavior after the peak
        let tail_start = (max_idx + n) / 2; // midpoint between peak and end
        if tail_start >= n {
            return ParticleShape::Globular;
        }

        let tail_mean: f64 = kratky[tail_start..].iter().map(|&(_, y)| y).sum::<f64>()
            / (n - tail_start) as f64;

        // If tail mean is much less than peak -> Globular (decays)
        // If tail mean is comparable to peak -> Extended (plateaus)
        let ratio = tail_mean / max_val;
        if ratio < 0.5 {
            ParticleShape::Globular
        } else {
            ParticleShape::Extended
        }
    }
}

// ---------------------------------------------------------------------------
// FormFactorModels
// ---------------------------------------------------------------------------

/// Type of analytical form factor model.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum FormFactorType {
    Sphere,
    Cylinder,
    Ellipsoid,
    GaussianChain,
}

/// Result of form factor fitting.
#[derive(Debug, Clone)]
pub struct FitResult {
    /// Fitted parameters (model-dependent).
    pub params: Vec<f64>,
    /// Parameter names.
    pub param_names: Vec<String>,
    /// Scaling factor.
    pub scale: f64,
    /// Chi-squared goodness of fit.
    pub chi_squared: f64,
}

/// Analytical form factor models for single-particle scattering.
pub struct FormFactorModels;

impl FormFactorModels {
    /// Sphere form factor:
    /// P(q) = [3 (sin(qR) - qR cos(qR)) / (qR)^3]^2
    pub fn sphere(q: f64, radius: f64) -> f64 {
        let qr = q * radius;
        if qr.abs() < 1e-10 {
            return 1.0;
        }
        let amplitude = 3.0 * (qr.sin() - qr * qr.cos()) / (qr * qr * qr);
        amplitude * amplitude
    }

    /// Cylinder form factor (orientationally averaged).
    /// Numerical integration over orientation angle alpha (0 to pi/2).
    pub fn cylinder(q: f64, radius: f64, length: f64) -> f64 {
        if q.abs() < 1e-10 {
            return 1.0;
        }
        let num_steps = 50;
        let d_alpha = (PI / 2.0) / num_steps as f64;
        let mut integral = 0.0;
        for k in 0..=num_steps {
            let alpha = k as f64 * d_alpha;
            let sin_a = alpha.sin();
            let cos_a = alpha.cos();

            // Axial part: sinc(qL cos(alpha)/2)
            let qh = q * (length / 2.0) * cos_a;
            let axial = if qh.abs() < 1e-10 { 1.0 } else { qh.sin() / qh };

            // Radial part: 2 J1(qR sin(alpha)) / (qR sin(alpha))
            let qr_sin = q * radius * sin_a;
            let radial = if qr_sin.abs() < 1e-10 {
                1.0
            } else {
                2.0 * bessel_j1(qr_sin) / qr_sin
            };

            let f_sq = (axial * radial).powi(2);
            let weight = if k == 0 || k == num_steps { 0.5 } else { 1.0 };
            integral += weight * f_sq * sin_a;
        }
        integral * d_alpha
    }

    /// Ellipsoid of revolution form factor (orientationally averaged).
    /// Semi-axes: a (polar), b (equatorial).
    pub fn ellipsoid(q: f64, a: f64, b: f64) -> f64 {
        if q.abs() < 1e-10 {
            return 1.0;
        }
        let num_steps = 50;
        let d_mu = 1.0 / num_steps as f64;
        let mut integral = 0.0;
        for k in 0..=num_steps {
            let mu = k as f64 * d_mu; // cos(alpha)
            let r_eff = (a * a * mu * mu + b * b * (1.0 - mu * mu)).sqrt();
            let qr = q * r_eff;
            let amplitude = if qr.abs() < 1e-10 {
                1.0
            } else {
                3.0 * (qr.sin() - qr * qr.cos()) / (qr * qr * qr)
            };
            let weight = if k == 0 || k == num_steps { 0.5 } else { 1.0 };
            integral += weight * amplitude * amplitude;
        }
        integral * d_mu
    }

    /// Gaussian chain (Debye function):
    /// P(q) = 2 (exp(-x) - 1 + x) / x^2 where x = (qRg)^2
    pub fn gaussian_chain(q: f64, rg: f64) -> f64 {
        let x = (q * rg).powi(2);
        if x < 1e-10 {
            return 1.0;
        }
        2.0 * ((-x).exp() - 1.0 + x) / (x * x)
    }

    /// Fit a form factor model to experimental data.
    /// Uses simple grid search + refinement for robustness.
    pub fn fit_form_factor(profile: &SaxsProfile, model: FormFactorType) -> FitResult {
        match model {
            FormFactorType::Sphere => fit_sphere(profile),
            FormFactorType::Cylinder => fit_cylinder(profile),
            FormFactorType::Ellipsoid => fit_ellipsoid(profile),
            FormFactorType::GaussianChain => fit_gaussian_chain(profile),
        }
    }
}

// ---------------------------------------------------------------------------
// StructureFactors
// ---------------------------------------------------------------------------

/// A peak found in the scattering profile.
#[derive(Debug, Clone, Copy)]
pub struct Peak {
    /// q position of the peak.
    pub q: f64,
    /// Intensity at the peak.
    pub intensity: f64,
    /// Real-space d-spacing: d = 2*pi/q.
    pub d_spacing: f64,
}

/// Inter-particle structure factor models.
pub struct StructureFactors;

impl StructureFactors {
    /// Percus-Yevick hard sphere structure factor.
    /// S(q) for hard spheres of given radius and volume fraction eta.
    pub fn hard_sphere(q: f64, radius: f64, volume_fraction: f64) -> f64 {
        let eta = volume_fraction;
        let qr2 = q * 2.0 * radius; // q * diameter

        if qr2.abs() < 1e-10 {
            // S(0) limiting form
            let denom = (1.0 + 2.0 * eta).powi(2) / (1.0 - eta).powi(4);
            return 1.0 / denom;
        }

        // Percus-Yevick direct correlation function in Fourier space
        let a = qr2;
        let alpha = (1.0 + 2.0 * eta).powi(2) / (1.0 - eta).powi(4);
        let beta = -6.0 * eta * (1.0 + eta / 2.0).powi(2) / (1.0 - eta).powi(4);
        let gamma = 0.5 * eta * alpha;

        let sin_a = a.sin();
        let cos_a = a.cos();
        let a2 = a * a;
        let a3 = a2 * a;
        let a4 = a3 * a;
        let a6 = a3 * a3;

        let g = alpha * (sin_a - a * cos_a) / a3
            + beta * (2.0 * a * sin_a + (2.0 - a2) * cos_a - 2.0) / a4
            + gamma
                * (-a4 * cos_a + 4.0 * ((3.0 * a2 - 6.0) * cos_a + (a3 - 6.0 * a) * sin_a + 6.0))
                / a6;

        let c_q = -24.0 * eta * g;
        1.0 / (1.0 - c_q)
    }

    /// Find peaks in the scattering profile using a simple local maxima search.
    pub fn peak_analysis(profile: &SaxsProfile) -> Vec<Peak> {
        let n = profile.q_values.len();
        let mut peaks = Vec::new();
        if n < 3 {
            return peaks;
        }
        for i in 1..n - 1 {
            let prev = profile.intensities[i - 1];
            let curr = profile.intensities[i];
            let next = profile.intensities[i + 1];
            if curr > prev && curr > next {
                peaks.push(Peak {
                    q: profile.q_values[i],
                    intensity: curr,
                    d_spacing: Self::d_spacing(profile.q_values[i]),
                });
            }
        }
        peaks
    }

    /// Convert a peak q value to real-space d-spacing: d = 2*pi/q.
    pub fn d_spacing(q_peak: f64) -> f64 {
        if q_peak.abs() < 1e-30 {
            return f64::INFINITY;
        }
        2.0 * PI / q_peak
    }
}

// ---------------------------------------------------------------------------
// IndirectFourierTransform
// ---------------------------------------------------------------------------

/// Indirect Fourier Transform to compute P(r) pair distance distribution.
///
/// P(r) is related to I(q) by:
/// I(q) = 4*pi * integral_0^d_max P(r) sin(qr)/(qr) dr
pub struct IndirectFourierTransform;

impl IndirectFourierTransform {
    /// Compute P(r) from I(q) using regularized indirect Fourier transform.
    ///
    /// Uses a discretized sine transform with Tikhonov regularization.
    /// Returns pairs of (r, P(r)).
    pub fn compute(
        profile: &SaxsProfile,
        d_max: f64,
        num_points: usize,
    ) -> Vec<(f64, f64)> {
        assert!(d_max > 0.0, "d_max must be positive");
        assert!(num_points >= 2, "need at least 2 output points");

        let dr = d_max / (num_points - 1) as f64;
        let n_q = profile.q_values.len();

        // Build the transformation matrix A where
        // I(q_j) = sum_k A[j,k] * P(r_k)
        // A[j,k] = 4*pi * r_k^2 * sinc(q_j * r_k) * dr
        let mut a_matrix = vec![vec![0.0; num_points]; n_q];
        let r_values: Vec<f64> = (0..num_points).map(|k| k as f64 * dr).collect();

        for j in 0..n_q {
            let q = profile.q_values[j];
            for k in 0..num_points {
                let r = r_values[k];
                let qr = q * r;
                let sinc = if qr.abs() < 1e-10 { 1.0 } else { qr.sin() / qr };
                a_matrix[j][k] = 4.0 * PI * r * r * sinc * dr;
            }
        }

        // Solve via A^T A p = A^T i  with Tikhonov regularization
        // (A^T A + lambda I) p = A^T i
        let lambda = 1e-3; // regularization parameter

        // Compute A^T A (num_points x num_points)
        let mut ata = vec![vec![0.0; num_points]; num_points];
        for i in 0..num_points {
            for j in 0..num_points {
                let mut sum = 0.0;
                for k in 0..n_q {
                    sum += a_matrix[k][i] * a_matrix[k][j];
                }
                ata[i][j] = sum;
                if i == j {
                    ata[i][j] += lambda;
                }
            }
        }

        // Compute A^T i (num_points)
        let mut ati = vec![0.0; num_points];
        for i in 0..num_points {
            let mut sum = 0.0;
            for k in 0..n_q {
                sum += a_matrix[k][i] * profile.intensities[k];
            }
            ati[i] = sum;
        }

        // Solve (A^T A + lambda I) p = A^T i using Gauss elimination
        let p_r = solve_linear_system(&ata, &ati);

        r_values
            .iter()
            .zip(p_r.iter())
            .map(|(&r, &p)| (r, p))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// AbsoluteScaling
// ---------------------------------------------------------------------------

/// Absolute intensity scaling for SAXS data.
pub struct AbsoluteScaling;

impl AbsoluteScaling {
    /// Scale relative intensity to absolute scale (cm^-1).
    ///
    /// I_abs = I_raw / (thickness * transmission * flux * efficiency * solid_angle_per_pixel)
    /// For simplicity we combine factors into a single calibration constant.
    pub fn scale_to_absolute(
        profile: &SaxsProfile,
        thickness_cm: f64,
        transmission: f64,
        flux: f64,
        detector_efficiency: f64,
    ) -> SaxsProfile {
        assert!(thickness_cm > 0.0, "thickness must be positive");
        assert!(transmission > 0.0 && transmission <= 1.0, "transmission must be in (0, 1]");
        assert!(flux > 0.0, "flux must be positive");
        assert!(detector_efficiency > 0.0, "detector efficiency must be positive");

        let cal = thickness_cm * transmission * flux * detector_efficiency;
        let intensities: Vec<f64> = profile.intensities.iter().map(|&i| i / cal).collect();
        let errors = profile.errors.as_ref().map(|e| e.iter().map(|&s| s / cal).collect());
        SaxsProfile {
            q_values: profile.q_values.clone(),
            intensities,
            errors,
        }
    }
}

// ---------------------------------------------------------------------------
// SizeDistribution
// ---------------------------------------------------------------------------

/// Type of size distribution function.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DistributionType {
    LogNormal,
    Gaussian,
    SchulzZimm,
}

/// Result of a polydisperse size distribution fit.
#[derive(Debug, Clone)]
pub struct SizeDistResult {
    /// Distribution type used.
    pub dist_type: DistributionType,
    /// Mean particle size (radius) in Angstroms.
    pub mean_radius: f64,
    /// Width parameter (sigma or z-parameter).
    pub width: f64,
    /// Polydispersity index PDI = sigma/mean.
    pub polydispersity: f64,
    /// Chi-squared of the fit.
    pub chi_squared: f64,
    /// Evaluated distribution as (radius, weight) pairs.
    pub distribution: Vec<(f64, f64)>,
}

/// Polydisperse size distribution fitting.
pub struct SizeDistribution;

impl SizeDistribution {
    /// Fit a size distribution to the SAXS profile using sphere form factors.
    pub fn fit_distribution(
        profile: &SaxsProfile,
        model: FormFactorType,
        dist_type: DistributionType,
    ) -> SizeDistResult {
        // Use grid search over mean_radius and width
        let mut best_chi2 = f64::MAX;
        let mut best_mean = 10.0;
        let mut best_width = 1.0;

        // Estimate Rg from Guinier region to set search range
        let q_min = profile.q_values[0];
        let q_upper = profile.q_values[profile.q_values.len().min(10) - 1];
        let rg = if let Some(rg) = try_guinier_rg(profile, q_min, q_upper) {
            rg
        } else {
            10.0
        };
        // For a sphere, R = Rg * sqrt(5/3)
        let r_est = rg * (5.0_f64 / 3.0).sqrt();
        let r_lo = (r_est * 0.2).max(1.0);
        let r_hi = r_est * 3.0;

        // Coarse grid search
        for ir in 0..20 {
            let mean_r = r_lo + (r_hi - r_lo) * ir as f64 / 19.0;
            for iw in 1..10 {
                let width = mean_r * iw as f64 * 0.05; // 5% to 45% polydispersity
                let chi2 = evaluate_size_dist(profile, model, dist_type, mean_r, width);
                if chi2 < best_chi2 {
                    best_chi2 = chi2;
                    best_mean = mean_r;
                    best_width = width;
                }
            }
        }

        // Fine refinement around best
        let dr = (r_hi - r_lo) / 19.0;
        let dw = best_mean * 0.05;
        for ir in 0..10 {
            let mean_r = (best_mean - dr) + 2.0 * dr * ir as f64 / 9.0;
            if mean_r <= 0.0 {
                continue;
            }
            for iw in 0..10 {
                let width = (best_width - dw).max(0.1) + 2.0 * dw * iw as f64 / 9.0;
                if width <= 0.0 {
                    continue;
                }
                let chi2 = evaluate_size_dist(profile, model, dist_type, mean_r, width);
                if chi2 < best_chi2 {
                    best_chi2 = chi2;
                    best_mean = mean_r;
                    best_width = width;
                }
            }
        }

        let pdi = best_width / best_mean;

        // Evaluate the best-fit distribution
        let dist = compute_distribution(dist_type, best_mean, best_width, 50);

        SizeDistResult {
            dist_type,
            mean_radius: best_mean,
            width: best_width,
            polydispersity: pdi,
            chi_squared: best_chi2,
            distribution: dist,
        }
    }

    /// Log-normal distribution function.
    /// f(R) = 1/(R*sigma*sqrt(2*pi)) * exp(-(ln(R)-mu)^2 / (2*sigma^2))
    pub fn log_normal(r: f64, median: f64, sigma: f64) -> f64 {
        if r <= 0.0 || sigma <= 0.0 {
            return 0.0;
        }
        let mu = median.ln();
        let exponent = -((r.ln() - mu).powi(2)) / (2.0 * sigma * sigma);
        exponent.exp() / (r * sigma * (2.0 * PI).sqrt())
    }

    /// Gaussian distribution function.
    pub fn gaussian(r: f64, mean: f64, sigma: f64) -> f64 {
        if sigma <= 0.0 {
            return 0.0;
        }
        let exponent = -((r - mean).powi(2)) / (2.0 * sigma * sigma);
        exponent.exp() / (sigma * (2.0 * PI).sqrt())
    }

    /// Schulz-Zimm distribution function.
    /// f(R) = ((z+1)/R_avg)^(z+1) * R^z * exp(-(z+1)*R/R_avg) / Gamma(z+1)
    pub fn schulz_zimm(r: f64, r_avg: f64, z: f64) -> f64 {
        if r <= 0.0 || r_avg <= 0.0 || z < 0.0 {
            return 0.0;
        }
        let zp1 = z + 1.0;
        let ratio = zp1 / r_avg;
        // log form for numerical stability
        let log_f = zp1 * ratio.ln() + z * r.ln() - ratio * r - ln_gamma(zp1);
        log_f.exp()
    }
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Simple linear regression: y = a + b*x. Returns (a, b, r_squared).
fn linear_regression(x: &[f64], y: &[f64]) -> (f64, f64, f64) {
    let n = x.len() as f64;
    let sum_x: f64 = x.iter().sum();
    let sum_y: f64 = y.iter().sum();
    let sum_xx: f64 = x.iter().map(|&xi| xi * xi).sum();
    let sum_xy: f64 = x.iter().zip(y.iter()).map(|(&xi, &yi)| xi * yi).sum();

    let denom = n * sum_xx - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (sum_y / n, 0.0, 0.0);
    }

    let b = (n * sum_xy - sum_x * sum_y) / denom;
    let a = (sum_y - b * sum_x) / n;

    // R-squared
    let y_mean = sum_y / n;
    let ss_tot: f64 = y.iter().map(|&yi| (yi - y_mean).powi(2)).sum();
    let ss_res: f64 = x
        .iter()
        .zip(y.iter())
        .map(|(&xi, &yi)| (yi - a - b * xi).powi(2))
        .sum();
    let r_squared = if ss_tot > 0.0 { 1.0 - ss_res / ss_tot } else { 0.0 };

    (a, b, r_squared)
}

/// Solve a linear system Ax = b using Gaussian elimination with partial pivoting.
fn solve_linear_system(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let n = b.len();
    // Augmented matrix
    let mut m = vec![vec![0.0; n + 1]; n];
    for i in 0..n {
        for j in 0..n {
            m[i][j] = a[i][j];
        }
        m[i][n] = b[i];
    }

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = m[col][col].abs();
        let mut max_row = col;
        for row in col + 1..n {
            if m[row][col].abs() > max_val {
                max_val = m[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-30 {
            continue; // Singular column
        }
        m.swap(col, max_row);

        let pivot = m[col][col];
        for row in col + 1..n {
            let factor = m[row][col] / pivot;
            for j in col..=n {
                m[row][j] -= factor * m[col][j];
            }
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        if m[i][i].abs() < 1e-30 {
            x[i] = 0.0;
            continue;
        }
        let mut sum = m[i][n];
        for j in i + 1..n {
            sum -= m[i][j] * x[j];
        }
        x[i] = sum / m[i][i];
    }
    x
}

/// Bessel function J1(x) using polynomial approximation.
fn bessel_j1(x: f64) -> f64 {
    let ax = x.abs();
    if ax < 8.0 {
        let y = x * x;
        let ans1 = x
            * (72362614232.0
                + y * (-7895059235.0
                    + y * (242396853.1
                        + y * (-2972611.439
                            + y * (15704.48260 + y * (-30.16036606))))));
        let ans2 = 144725228442.0
            + y * (2300535178.0
                + y * (18583304.74
                    + y * (99447.43394
                        + y * (376.9991397 + y * 1.0))));
        ans1 / ans2
    } else {
        let z = 8.0 / ax;
        let y = z * z;
        let xx = ax - 2.356194491;
        let p = 1.0
            + y * (0.183105e-2
                + y * (-0.3516396496e-4
                    + y * (0.2457520174e-5 + y * (-0.240337019e-6))));
        let q = 0.04687499995
            + y * (-0.2002690873e-3
                + y * (0.8449199096e-5
                    + y * (-0.88228987e-6 + y * 0.105787412e-6)));
        let ans = (0.636619772 / ax).sqrt() * (xx.cos() * p - z * xx.sin() * q);
        if x < 0.0 { -ans } else { ans }
    }
}

/// Stirling approximation for ln(Gamma(x)) for x > 0.
fn ln_gamma(x: f64) -> f64 {
    if x <= 0.0 {
        return 0.0;
    }
    if x < 0.5 {
        // Use reflection formula: Gamma(x) Gamma(1-x) = pi / sin(pi x)
        let log_pi_sin = (PI * x).sin().abs().ln();
        return (PI).ln() - log_pi_sin - ln_gamma(1.0 - x);
    }
    // Lanczos approximation
    let g = 7.0;
    let coeffs = [
        0.99999999999980993,
        676.5203681218851,
        -1259.1392167224028,
        771.32342877765313,
        -176.61502916214059,
        12.507343278686905,
        -0.13857109526572012,
        9.9843695780195716e-6,
        1.5056327351493116e-7,
    ];
    let x_shifted = x - 1.0;
    let mut ag = coeffs[0];
    for i in 1..coeffs.len() {
        ag += coeffs[i] / (x_shifted + i as f64);
    }
    let t = x_shifted + g + 0.5;
    0.5 * (2.0 * PI).ln() + (x_shifted + 0.5) * t.ln() - t + ag.ln()
}

/// Try to estimate Rg from the low-q region. Returns None if insufficient data.
fn try_guinier_rg(profile: &SaxsProfile, q_min: f64, q_max: f64) -> Option<f64> {
    let mut x_vals = Vec::new();
    let mut y_vals = Vec::new();
    for i in 0..profile.q_values.len() {
        let q = profile.q_values[i];
        let intensity = profile.intensities[i];
        if q >= q_min && q <= q_max && intensity > 0.0 {
            x_vals.push(q * q);
            y_vals.push(intensity.ln());
        }
    }
    if x_vals.len() < 2 {
        return None;
    }
    let (_, b, _) = linear_regression(&x_vals, &y_vals);
    let rg_sq = -3.0 * b;
    if rg_sq > 0.0 {
        Some(rg_sq.sqrt())
    } else {
        None
    }
}

/// Fit a sphere form factor to the profile.
fn fit_sphere(profile: &SaxsProfile) -> FitResult {
    let mut best_chi2 = f64::MAX;
    let mut best_r = 10.0;
    let mut best_scale = 1.0;

    // Coarse search
    for ir in 1..100 {
        let r = ir as f64;
        let (scale, chi2) = fit_scale_and_chi2(profile, |q| FormFactorModels::sphere(q, r));
        if chi2 < best_chi2 {
            best_chi2 = chi2;
            best_r = r;
            best_scale = scale;
        }
    }

    // Fine search around best
    for ir in 0..20 {
        let r = best_r - 1.0 + 0.1 * ir as f64;
        if r <= 0.0 {
            continue;
        }
        let (scale, chi2) = fit_scale_and_chi2(profile, |q| FormFactorModels::sphere(q, r));
        if chi2 < best_chi2 {
            best_chi2 = chi2;
            best_r = r;
            best_scale = scale;
        }
    }

    FitResult {
        params: vec![best_r],
        param_names: vec!["radius".to_string()],
        scale: best_scale,
        chi_squared: best_chi2,
    }
}

/// Fit a cylinder form factor.
fn fit_cylinder(profile: &SaxsProfile) -> FitResult {
    let mut best_chi2 = f64::MAX;
    let mut best_r = 10.0;
    let mut best_l = 100.0;
    let mut best_scale = 1.0;

    for ir in 1..20 {
        let r = ir as f64 * 2.0;
        for il in 1..20 {
            let l = il as f64 * 20.0;
            let (scale, chi2) =
                fit_scale_and_chi2(profile, |q| FormFactorModels::cylinder(q, r, l));
            if chi2 < best_chi2 {
                best_chi2 = chi2;
                best_r = r;
                best_l = l;
                best_scale = scale;
            }
        }
    }

    FitResult {
        params: vec![best_r, best_l],
        param_names: vec!["radius".to_string(), "length".to_string()],
        scale: best_scale,
        chi_squared: best_chi2,
    }
}

/// Fit an ellipsoid form factor.
fn fit_ellipsoid(profile: &SaxsProfile) -> FitResult {
    let mut best_chi2 = f64::MAX;
    let mut best_a = 10.0;
    let mut best_b = 20.0;
    let mut best_scale = 1.0;

    for ia in 1..20 {
        let a = ia as f64 * 2.0;
        for ib in 1..20 {
            let b = ib as f64 * 2.0;
            let (scale, chi2) =
                fit_scale_and_chi2(profile, |q| FormFactorModels::ellipsoid(q, a, b));
            if chi2 < best_chi2 {
                best_chi2 = chi2;
                best_a = a;
                best_b = b;
                best_scale = scale;
            }
        }
    }

    FitResult {
        params: vec![best_a, best_b],
        param_names: vec!["semi_axis_a".to_string(), "semi_axis_b".to_string()],
        scale: best_scale,
        chi_squared: best_chi2,
    }
}

/// Fit a Gaussian chain (Debye function).
fn fit_gaussian_chain(profile: &SaxsProfile) -> FitResult {
    let mut best_chi2 = f64::MAX;
    let mut best_rg = 10.0;
    let mut best_scale = 1.0;

    for irg in 1..100 {
        let rg = irg as f64;
        let (scale, chi2) =
            fit_scale_and_chi2(profile, |q| FormFactorModels::gaussian_chain(q, rg));
        if chi2 < best_chi2 {
            best_chi2 = chi2;
            best_rg = rg;
            best_scale = scale;
        }
    }

    // Fine search
    for irg in 0..20 {
        let rg = best_rg - 1.0 + 0.1 * irg as f64;
        if rg <= 0.0 {
            continue;
        }
        let (scale, chi2) =
            fit_scale_and_chi2(profile, |q| FormFactorModels::gaussian_chain(q, rg));
        if chi2 < best_chi2 {
            best_chi2 = chi2;
            best_rg = rg;
            best_scale = scale;
        }
    }

    FitResult {
        params: vec![best_rg],
        param_names: vec!["rg".to_string()],
        scale: best_scale,
        chi_squared: best_chi2,
    }
}

/// Given a form factor model function, find the optimal scale factor and chi2.
/// scale = sum(I_data * P(q)) / sum(P(q)^2)
fn fit_scale_and_chi2<F: Fn(f64) -> f64>(profile: &SaxsProfile, model: F) -> (f64, f64) {
    let mut sum_ip = 0.0;
    let mut sum_pp = 0.0;
    for i in 0..profile.q_values.len() {
        let p = model(profile.q_values[i]);
        sum_ip += profile.intensities[i] * p;
        sum_pp += p * p;
    }
    let scale = if sum_pp > 1e-30 { sum_ip / sum_pp } else { 1.0 };

    let mut chi2 = 0.0;
    for i in 0..profile.q_values.len() {
        let p = model(profile.q_values[i]);
        let diff = profile.intensities[i] - scale * p;
        let weight = if let Some(ref e) = profile.errors {
            if e[i] > 0.0 {
                1.0 / (e[i] * e[i])
            } else {
                1.0
            }
        } else {
            1.0
        };
        chi2 += diff * diff * weight;
    }
    (scale, chi2)
}

/// Evaluate a polydisperse size distribution fit (chi-squared).
fn evaluate_size_dist(
    profile: &SaxsProfile,
    model: FormFactorType,
    dist_type: DistributionType,
    mean_r: f64,
    width: f64,
) -> f64 {
    let dist = compute_distribution(dist_type, mean_r, width, 20);
    // I_model(q) = sum_k w_k * P(q, r_k) * dr
    let form_factor = |q: f64, r: f64| -> f64 {
        match model {
            FormFactorType::Sphere => FormFactorModels::sphere(q, r),
            FormFactorType::GaussianChain => FormFactorModels::gaussian_chain(q, r),
            FormFactorType::Cylinder => FormFactorModels::cylinder(q, r, r * 5.0),
            FormFactorType::Ellipsoid => FormFactorModels::ellipsoid(q, r, r * 1.5),
        }
    };

    // Compute model intensity
    let mut model_i = vec![0.0; profile.q_values.len()];
    for (idx, &q) in profile.q_values.iter().enumerate() {
        for &(r, w) in &dist {
            if w > 0.0 && r > 0.0 {
                let dr = if dist.len() > 1 {
                    (dist.last().unwrap().0 - dist[0].0) / (dist.len() - 1) as f64
                } else {
                    1.0
                };
                model_i[idx] += w * form_factor(q, r) * dr;
            }
        }
    }

    // Find optimal scale
    let mut sum_di = 0.0;
    let mut sum_mm = 0.0;
    for idx in 0..profile.q_values.len() {
        sum_di += profile.intensities[idx] * model_i[idx];
        sum_mm += model_i[idx] * model_i[idx];
    }
    let scale = if sum_mm > 1e-30 { sum_di / sum_mm } else { 1.0 };

    // Chi-squared
    let mut chi2 = 0.0;
    for idx in 0..profile.q_values.len() {
        let diff = profile.intensities[idx] - scale * model_i[idx];
        chi2 += diff * diff;
    }
    chi2
}

/// Compute a size distribution as (radius, weight) pairs.
fn compute_distribution(
    dist_type: DistributionType,
    mean_r: f64,
    width: f64,
    num_points: usize,
) -> Vec<(f64, f64)> {
    let r_min = (mean_r - 4.0 * width).max(0.1);
    let r_max = mean_r + 4.0 * width;
    let dr = (r_max - r_min) / (num_points - 1).max(1) as f64;

    (0..num_points)
        .map(|k| {
            let r = r_min + k as f64 * dr;
            let w = match dist_type {
                DistributionType::LogNormal => {
                    // Convert mean/sigma to lognormal parameters
                    let sigma_ln = (1.0 + (width / mean_r).powi(2)).ln().sqrt();
                    let median = mean_r / (0.5 * sigma_ln * sigma_ln).exp();
                    SizeDistribution::log_normal(r, median, sigma_ln)
                }
                DistributionType::Gaussian => SizeDistribution::gaussian(r, mean_r, width),
                DistributionType::SchulzZimm => {
                    let z = (mean_r / width).powi(2) - 1.0;
                    SizeDistribution::schulz_zimm(r, mean_r, z.max(0.0))
                }
            };
            (r, w)
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // Generate a synthetic Guinier profile: I(q) = I0 * exp(-q^2 * Rg^2 / 3)
    fn make_guinier_profile(i0: f64, rg: f64, q_min: f64, q_max: f64, n: usize) -> SaxsProfile {
        let dq = (q_max - q_min) / (n - 1) as f64;
        let q_values: Vec<f64> = (0..n).map(|k| q_min + k as f64 * dq).collect();
        let intensities: Vec<f64> = q_values
            .iter()
            .map(|&q| i0 * (-q * q * rg * rg / 3.0).exp())
            .collect();
        SaxsProfile::new(q_values, intensities)
    }

    // Generate a sphere scattering profile
    fn make_sphere_profile(radius: f64, scale: f64, q_min: f64, q_max: f64, n: usize) -> SaxsProfile {
        let dq = (q_max - q_min) / (n - 1) as f64;
        let q_values: Vec<f64> = (0..n).map(|k| q_min + k as f64 * dq).collect();
        let intensities: Vec<f64> = q_values
            .iter()
            .map(|&q| scale * FormFactorModels::sphere(q, radius))
            .collect();
        SaxsProfile::new(q_values, intensities)
    }

    // --- SaxsProfile tests ---

    #[test]
    fn test_profile_new() {
        let p = SaxsProfile::new(vec![0.01, 0.02, 0.03], vec![100.0, 50.0, 25.0]);
        assert_eq!(p.len(), 3);
        assert!(!p.is_empty());
    }

    #[test]
    fn test_profile_with_errors() {
        let p = SaxsProfile::with_errors(
            vec![0.01, 0.02],
            vec![100.0, 50.0],
            vec![1.0, 0.5],
        );
        assert!(p.errors.is_some());
        assert_eq!(p.errors.as_ref().unwrap().len(), 2);
    }

    #[test]
    fn test_profile_q_range() {
        let p = SaxsProfile::new(vec![0.01, 0.05, 0.02, 0.1], vec![1.0; 4]);
        let (qmin, qmax) = p.q_range();
        assert!(approx_eq(qmin, 0.01, EPSILON));
        assert!(approx_eq(qmax, 0.1, EPSILON));
    }

    #[test]
    fn test_profile_interpolate_exact() {
        let p = SaxsProfile::new(vec![0.01, 0.02, 0.03], vec![100.0, 50.0, 25.0]);
        assert!(approx_eq(p.interpolate(0.01), 100.0, EPSILON));
        assert!(approx_eq(p.interpolate(0.02), 50.0, EPSILON));
    }

    #[test]
    fn test_profile_interpolate_midpoint() {
        let p = SaxsProfile::new(vec![0.0, 1.0], vec![0.0, 10.0]);
        assert!(approx_eq(p.interpolate(0.5), 5.0, EPSILON));
    }

    #[test]
    fn test_profile_interpolate_below_range() {
        let p = SaxsProfile::new(vec![1.0, 2.0], vec![10.0, 20.0]);
        assert!(approx_eq(p.interpolate(0.5), 10.0, EPSILON));
    }

    #[test]
    fn test_profile_interpolate_above_range() {
        let p = SaxsProfile::new(vec![1.0, 2.0], vec![10.0, 20.0]);
        assert!(approx_eq(p.interpolate(3.0), 20.0, EPSILON));
    }

    #[test]
    fn test_profile_slice_q_range() {
        let p = SaxsProfile::new(vec![0.01, 0.02, 0.03, 0.04, 0.05], vec![100.0, 80.0, 60.0, 40.0, 20.0]);
        let sliced = p.slice_q_range(0.02, 0.04);
        assert_eq!(sliced.len(), 3);
        assert!(approx_eq(sliced.q_values[0], 0.02, EPSILON));
        assert!(approx_eq(sliced.q_values[2], 0.04, EPSILON));
    }

    #[test]
    #[should_panic(expected = "q and I arrays must have equal length")]
    fn test_profile_mismatched_lengths() {
        SaxsProfile::new(vec![0.01, 0.02], vec![100.0]);
    }

    #[test]
    fn test_profile_single_point() {
        let p = SaxsProfile::new(vec![0.01], vec![100.0]);
        assert_eq!(p.len(), 1);
        assert!(approx_eq(p.interpolate(0.01), 100.0, EPSILON));
        assert!(approx_eq(p.interpolate(0.05), 100.0, EPSILON));
    }

    // --- BackgroundSubtraction tests ---

    #[test]
    fn test_subtract_constant() {
        let p = SaxsProfile::new(vec![0.01, 0.02], vec![100.0, 50.0]);
        let result = BackgroundSubtraction::subtract_constant(&p, 10.0);
        assert!(approx_eq(result.intensities[0], 90.0, EPSILON));
        assert!(approx_eq(result.intensities[1], 40.0, EPSILON));
    }

    #[test]
    fn test_subtract_constant_no_negative() {
        let p = SaxsProfile::new(vec![0.01], vec![5.0]);
        let result = BackgroundSubtraction::subtract_constant(&p, 10.0);
        assert!(approx_eq(result.intensities[0], 0.0, EPSILON));
    }

    #[test]
    fn test_subtract_profile() {
        let sample = SaxsProfile::new(vec![0.01, 0.02], vec![100.0, 50.0]);
        let buffer = SaxsProfile::new(vec![0.01, 0.02], vec![20.0, 10.0]);
        let result = BackgroundSubtraction::subtract_profile(&sample, &buffer, 0.5);
        assert!(approx_eq(result.intensities[0], 90.0, EPSILON));
        assert!(approx_eq(result.intensities[1], 45.0, EPSILON));
    }

    #[test]
    fn test_auto_background() {
        let q: Vec<f64> = (0..100).map(|k| 0.01 + k as f64 * 0.01).collect();
        let i: Vec<f64> = q.iter().map(|&q_val| 1000.0 * (-q_val * 10.0).exp() + 5.0).collect();
        let p = SaxsProfile::new(q, i);
        let bg = BackgroundSubtraction::auto_background(&p);
        // High-q tail should be close to the constant 5.0
        assert!(bg > 4.5 && bg < 6.0, "auto_background = {}, expected ~5.0", bg);
    }

    // --- GuinierAnalysis tests ---

    #[test]
    fn test_guinier_fit_known_rg() {
        let rg_true = 20.0;
        let i0_true = 1000.0;
        let profile = make_guinier_profile(i0_true, rg_true, 0.001, 0.05, 100);
        let result = GuinierAnalysis::fit(&profile, 0.001, 0.05);
        assert!(approx_eq(result.rg, rg_true, 0.5), "Rg = {}, expected {}", result.rg, rg_true);
        assert!(approx_eq(result.i0, i0_true, 10.0), "I0 = {}, expected {}", result.i0, i0_true);
        assert!(result.quality > 0.99, "R^2 = {}, expected > 0.99", result.quality);
    }

    #[test]
    fn test_guinier_fit_small_rg() {
        let rg_true = 5.0;
        let profile = make_guinier_profile(500.0, rg_true, 0.01, 0.2, 50);
        let result = GuinierAnalysis::fit(&profile, 0.01, 0.2);
        assert!(approx_eq(result.rg, rg_true, 1.0), "Rg = {}", result.rg);
    }

    #[test]
    fn test_guinier_plot() {
        let profile = make_guinier_profile(100.0, 10.0, 0.01, 0.1, 20);
        let plot = GuinierAnalysis::guinier_plot(&profile);
        assert_eq!(plot.len(), 20);
        // First point: q^2 ~ 0.0001, ln(I) ~ ln(100 * exp(-0.0001*100/3)) ~ ln(100)
        assert!(plot[0].0 > 0.0);
        assert!(plot[0].1 > 0.0);
    }

    // --- PorodAnalysis tests ---

    #[test]
    fn test_porod_constant() {
        // I(q) = K_p / q^4 with K_p = 100
        let kp = 100.0;
        let q: Vec<f64> = (1..50).map(|k| 0.5 + k as f64 * 0.01).collect();
        let i: Vec<f64> = q.iter().map(|&qv| kp / qv.powi(4)).collect();
        let profile = SaxsProfile::new(q, i);
        let result = PorodAnalysis::fit(&profile, 0.5, 1.0);
        assert!(approx_eq(result.porod_constant, kp, 1.0), "Kp = {}", result.porod_constant);
    }

    #[test]
    fn test_porod_invariant() {
        // Q = integral of I(q) * q^2 dq for a simple case
        // I(q) = 1 for all q, Q = integral q^2 dq from 0.1 to 1.0
        let q: Vec<f64> = (0..100).map(|k| 0.1 + k as f64 * 0.009).collect();
        let i = vec![1.0; 100];
        let profile = SaxsProfile::new(q, i);
        let inv = PorodAnalysis::invariant(&profile);
        // Analytical: integral of q^2 from 0.1 to 1.0 = (1^3 - 0.1^3)/3 = 0.9997/3 ≈ 0.333
        assert!(inv > 0.3 && inv < 0.4, "Invariant = {}", inv);
    }

    #[test]
    fn test_porod_plot() {
        let profile = SaxsProfile::new(vec![0.1, 0.2, 0.3], vec![10.0, 5.0, 2.0]);
        let plot = PorodAnalysis::porod_plot(&profile);
        assert_eq!(plot.len(), 3);
        // First point: 10.0 * 0.1^4 = 0.001
        assert!(approx_eq(plot[0].1, 10.0 * 0.1_f64.powi(4), EPSILON));
    }

    // --- KratkyPlot tests ---

    #[test]
    fn test_kratky_compute() {
        let profile = SaxsProfile::new(vec![0.1, 0.2, 0.3], vec![100.0, 25.0, 11.1]);
        let kratky = KratkyPlot::compute(&profile);
        assert_eq!(kratky.len(), 3);
        // First: 100 * 0.01 = 1.0
        assert!(approx_eq(kratky[0].1, 100.0 * 0.01, EPSILON));
    }

    #[test]
    fn test_kratky_globular() {
        // Bell-shaped: peak then decay
        let profile = make_guinier_profile(1000.0, 20.0, 0.001, 0.3, 100);
        let kratky = KratkyPlot::compute(&profile);
        let shape = KratkyPlot::classify_shape(&kratky);
        assert_eq!(shape, ParticleShape::Globular);
    }

    #[test]
    fn test_kratky_unfolded() {
        // Monotonically increasing I*q^2 = like I ~ q^0 constant
        let q: Vec<f64> = (1..100).map(|k| k as f64 * 0.01).collect();
        let i = vec![1.0; 99]; // constant I -> I*q^2 increases
        let profile = SaxsProfile::new(q, i);
        let kratky = KratkyPlot::compute(&profile);
        let shape = KratkyPlot::classify_shape(&kratky);
        assert_eq!(shape, ParticleShape::Unfolded);
    }

    #[test]
    fn test_kratky_dimensionless() {
        let profile = make_guinier_profile(100.0, 10.0, 0.01, 0.5, 50);
        let dim_kratky = KratkyPlot::compute_dimensionless(&profile, 10.0, 100.0);
        assert_eq!(dim_kratky.len(), 50);
        // At q=0, dimensionless qRg=0 and y=0
        assert!(dim_kratky[0].0 < 0.2);
    }

    // --- FormFactorModels tests ---

    #[test]
    fn test_sphere_form_factor_zero_q() {
        let pq = FormFactorModels::sphere(0.0, 10.0);
        assert!(approx_eq(pq, 1.0, EPSILON));
    }

    #[test]
    fn test_sphere_form_factor_positive() {
        let pq = FormFactorModels::sphere(0.1, 10.0);
        // P(q) should be between 0 and 1
        assert!(pq >= 0.0 && pq <= 1.0, "P(q) = {}", pq);
    }

    #[test]
    fn test_sphere_form_factor_decreasing() {
        let p1 = FormFactorModels::sphere(0.01, 10.0);
        let p2 = FormFactorModels::sphere(0.05, 10.0);
        assert!(p1 > p2, "Form factor should decrease with q");
    }

    #[test]
    fn test_sphere_form_factor_known_value() {
        // At qR = pi, sin(qR)-qR cos(qR) = sin(pi)-pi*cos(pi) = pi
        // P = [3*pi / pi^3]^2 = [3/pi^2]^2
        let q = PI / 10.0;
        let pq = FormFactorModels::sphere(q, 10.0);
        let expected = (3.0 / (PI * PI)).powi(2);
        assert!(approx_eq(pq, expected, 1e-4), "P(q) = {}, expected {}", pq, expected);
    }

    #[test]
    fn test_cylinder_form_factor_zero_q() {
        let pq = FormFactorModels::cylinder(0.0, 5.0, 50.0);
        assert!(approx_eq(pq, 1.0, 0.05));
    }

    #[test]
    fn test_cylinder_form_factor_positive() {
        let pq = FormFactorModels::cylinder(0.1, 5.0, 50.0);
        assert!(pq >= 0.0, "P(q) = {}", pq);
    }

    #[test]
    fn test_ellipsoid_form_factor_sphere_limit() {
        // When a == b, ellipsoid should give same as sphere
        let pq_sphere = FormFactorModels::sphere(0.1, 10.0);
        let pq_ellipsoid = FormFactorModels::ellipsoid(0.1, 10.0, 10.0);
        assert!(
            approx_eq(pq_sphere, pq_ellipsoid, 0.01),
            "sphere={}, ellipsoid={}",
            pq_sphere,
            pq_ellipsoid
        );
    }

    #[test]
    fn test_gaussian_chain_zero_q() {
        let pq = FormFactorModels::gaussian_chain(0.0, 10.0);
        assert!(approx_eq(pq, 1.0, EPSILON));
    }

    #[test]
    fn test_gaussian_chain_known() {
        // At x = (qRg)^2 = 1: P = 2(e^-1 - 1 + 1)/1 = 2/e ~ 0.7358
        let q = 1.0 / 10.0; // qRg = 1
        let pq = FormFactorModels::gaussian_chain(q, 10.0);
        let expected = 2.0 / std::f64::consts::E;
        assert!(approx_eq(pq, expected, 0.001), "P(q) = {}, expected {}", pq, expected);
    }

    #[test]
    fn test_fit_sphere_form_factor() {
        let radius = 20.0;
        let profile = make_sphere_profile(radius, 500.0, 0.005, 0.5, 100);
        let result = FormFactorModels::fit_form_factor(&profile, FormFactorType::Sphere);
        assert!(
            approx_eq(result.params[0], radius, 2.0),
            "Fitted radius = {}, expected {}",
            result.params[0],
            radius
        );
    }

    #[test]
    fn test_fit_gaussian_chain() {
        let rg = 30.0;
        let q: Vec<f64> = (1..100).map(|k| k as f64 * 0.002).collect();
        let i: Vec<f64> = q.iter().map(|&qv| 200.0 * FormFactorModels::gaussian_chain(qv, rg)).collect();
        let profile = SaxsProfile::new(q, i);
        let result = FormFactorModels::fit_form_factor(&profile, FormFactorType::GaussianChain);
        assert!(
            approx_eq(result.params[0], rg, 2.0),
            "Fitted Rg = {}, expected {}",
            result.params[0],
            rg
        );
    }

    // --- StructureFactors tests ---

    #[test]
    fn test_hard_sphere_zero_q() {
        let sq = StructureFactors::hard_sphere(0.0, 10.0, 0.3);
        // S(0) should be finite and positive for eta < 1
        assert!(sq > 0.0 && sq < 1.0, "S(0) = {}", sq);
    }

    #[test]
    fn test_hard_sphere_low_concentration() {
        // At very low volume fraction, S(q) -> 1
        let sq = StructureFactors::hard_sphere(0.5, 10.0, 0.001);
        assert!(approx_eq(sq, 1.0, 0.1), "S(q) = {}", sq);
    }

    #[test]
    fn test_d_spacing() {
        let d = StructureFactors::d_spacing(0.1);
        let expected = 2.0 * PI / 0.1;
        assert!(approx_eq(d, expected, EPSILON));
    }

    #[test]
    fn test_d_spacing_zero() {
        let d = StructureFactors::d_spacing(0.0);
        assert!(d.is_infinite());
    }

    #[test]
    fn test_peak_analysis() {
        // Profile with a clear peak
        let profile = SaxsProfile::new(
            vec![0.01, 0.02, 0.03, 0.04, 0.05],
            vec![10.0, 30.0, 50.0, 30.0, 10.0],
        );
        let peaks = StructureFactors::peak_analysis(&profile);
        assert_eq!(peaks.len(), 1);
        assert!(approx_eq(peaks[0].q, 0.03, EPSILON));
        assert!(approx_eq(peaks[0].intensity, 50.0, EPSILON));
    }

    #[test]
    fn test_peak_analysis_multiple() {
        let profile = SaxsProfile::new(
            vec![0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07],
            vec![10.0, 50.0, 10.0, 10.0, 40.0, 10.0, 5.0],
        );
        let peaks = StructureFactors::peak_analysis(&profile);
        assert_eq!(peaks.len(), 2);
    }

    // --- IndirectFourierTransform tests ---

    #[test]
    fn test_ift_sphere() {
        // For a uniform sphere, P(r) is known analytically
        let radius = 10.0;
        let profile = make_sphere_profile(radius, 1.0, 0.01, 1.0, 100);
        let pr = IndirectFourierTransform::compute(&profile, 30.0, 30);
        // P(r) should be zero beyond 2*radius = 20 Angstroms
        // and have a peak around r ~ radius
        let max_pr = pr.iter().map(|&(_, p)| p).fold(f64::MIN, f64::max);
        assert!(max_pr > 0.0, "P(r) should have positive values");
    }

    #[test]
    fn test_ift_output_size() {
        let profile = SaxsProfile::new(vec![0.01, 0.1, 0.5], vec![100.0, 10.0, 1.0]);
        let pr = IndirectFourierTransform::compute(&profile, 50.0, 20);
        assert_eq!(pr.len(), 20);
    }

    #[test]
    fn test_ift_r_range() {
        let profile = SaxsProfile::new(vec![0.01, 0.1], vec![100.0, 10.0]);
        let pr = IndirectFourierTransform::compute(&profile, 50.0, 10);
        assert!(approx_eq(pr[0].0, 0.0, EPSILON));
        assert!(approx_eq(pr[9].0, 50.0, 0.1));
    }

    // --- AbsoluteScaling tests ---

    #[test]
    fn test_absolute_scaling() {
        let p = SaxsProfile::new(vec![0.01, 0.02], vec![100.0, 50.0]);
        let scaled = AbsoluteScaling::scale_to_absolute(&p, 0.1, 0.9, 1e10, 0.8);
        let cal = 0.1 * 0.9 * 1e10 * 0.8;
        assert!(approx_eq(scaled.intensities[0], 100.0 / cal, scaled.intensities[0] * 1e-6));
    }

    #[test]
    fn test_absolute_scaling_preserves_errors() {
        let p = SaxsProfile::with_errors(
            vec![0.01, 0.02],
            vec![100.0, 50.0],
            vec![1.0, 0.5],
        );
        let scaled = AbsoluteScaling::scale_to_absolute(&p, 0.1, 0.9, 1e10, 0.8);
        assert!(scaled.errors.is_some());
        let cal = 0.1 * 0.9 * 1e10 * 0.8;
        assert!(approx_eq(scaled.errors.as_ref().unwrap()[0], 1.0 / cal, 1e-20));
    }

    #[test]
    #[should_panic(expected = "thickness must be positive")]
    fn test_absolute_scaling_zero_thickness() {
        let p = SaxsProfile::new(vec![0.01], vec![100.0]);
        AbsoluteScaling::scale_to_absolute(&p, 0.0, 0.9, 1e10, 0.8);
    }

    // --- SizeDistribution tests ---

    #[test]
    fn test_log_normal_distribution() {
        let f = SizeDistribution::log_normal(10.0, 10.0, 0.3);
        assert!(f > 0.0, "f = {}", f);
    }

    #[test]
    fn test_log_normal_zero_radius() {
        let f = SizeDistribution::log_normal(0.0, 10.0, 0.3);
        assert!(approx_eq(f, 0.0, EPSILON));
    }

    #[test]
    fn test_gaussian_distribution() {
        let f = SizeDistribution::gaussian(10.0, 10.0, 2.0);
        // At mean, max = 1/(sigma*sqrt(2*pi))
        let expected = 1.0 / (2.0 * (2.0 * PI).sqrt());
        assert!(approx_eq(f, expected, 1e-4), "f = {}, expected {}", f, expected);
    }

    #[test]
    fn test_schulz_zimm_distribution() {
        let f = SizeDistribution::schulz_zimm(10.0, 10.0, 5.0);
        assert!(f > 0.0, "f = {}", f);
    }

    #[test]
    fn test_schulz_zimm_zero_radius() {
        let f = SizeDistribution::schulz_zimm(0.0, 10.0, 5.0);
        assert!(approx_eq(f, 0.0, EPSILON));
    }

    #[test]
    fn test_fit_size_distribution() {
        let radius = 15.0;
        let profile = make_sphere_profile(radius, 1000.0, 0.01, 0.5, 50);
        let result = SizeDistribution::fit_distribution(
            &profile,
            FormFactorType::Sphere,
            DistributionType::Gaussian,
        );
        // The fitted mean should be reasonably close to the true radius
        assert!(
            result.mean_radius > 5.0 && result.mean_radius < 50.0,
            "mean_radius = {}",
            result.mean_radius
        );
        assert!(result.distribution.len() > 0);
    }

    // --- Utility function tests ---

    #[test]
    fn test_linear_regression_perfect() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![2.0, 4.0, 6.0, 8.0, 10.0]; // y = 2x
        let (a, b, r2) = linear_regression(&x, &y);
        assert!(approx_eq(a, 0.0, 1e-10));
        assert!(approx_eq(b, 2.0, 1e-10));
        assert!(approx_eq(r2, 1.0, 1e-10));
    }

    #[test]
    fn test_linear_regression_with_intercept() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![5.0, 7.0, 9.0, 11.0]; // y = 5 + 2x
        let (a, b, r2) = linear_regression(&x, &y);
        assert!(approx_eq(a, 5.0, 1e-10));
        assert!(approx_eq(b, 2.0, 1e-10));
        assert!(approx_eq(r2, 1.0, 1e-10));
    }

    #[test]
    fn test_bessel_j1_zero() {
        let j = bessel_j1(0.0);
        assert!(approx_eq(j, 0.0, 1e-10));
    }

    #[test]
    fn test_bessel_j1_known() {
        // J1(3.8317...) = 0 (first zero)
        let j = bessel_j1(3.8317);
        assert!(j.abs() < 0.01, "J1(3.8317) = {}", j);
    }

    #[test]
    fn test_bessel_j1_small() {
        // J1(x) ~ x/2 for small x
        let x = 0.01;
        let j = bessel_j1(x);
        assert!(approx_eq(j, x / 2.0, 1e-5), "J1({}) = {}", x, j);
    }

    #[test]
    fn test_bessel_j1_large() {
        let j = bessel_j1(10.0);
        // Known value: J1(10) ≈ 0.04347
        assert!(approx_eq(j, 0.04347, 0.001), "J1(10) = {}", j);
    }

    #[test]
    fn test_ln_gamma_integer() {
        // Gamma(5) = 4! = 24, ln(24) ~ 3.178
        let lg = ln_gamma(5.0);
        assert!(approx_eq(lg, 24.0_f64.ln(), 1e-6), "ln(Gamma(5)) = {}", lg);
    }

    #[test]
    fn test_ln_gamma_half() {
        // Gamma(0.5) = sqrt(pi), ln(sqrt(pi)) ~ 0.5724
        let lg = ln_gamma(0.5);
        let expected = (PI).sqrt().ln();
        assert!(approx_eq(lg, expected, 1e-4), "ln(Gamma(0.5)) = {}, expected {}", lg, expected);
    }

    #[test]
    fn test_solve_linear_system_2x2() {
        // 2x + y = 5, x + 3y = 7 => x = 1.6, y = 1.8
        let a = vec![vec![2.0, 1.0], vec![1.0, 3.0]];
        let b = vec![5.0, 7.0];
        let x = solve_linear_system(&a, &b);
        assert!(approx_eq(x[0], 1.6, 1e-10));
        assert!(approx_eq(x[1], 1.8, 1e-10));
    }

    #[test]
    fn test_solve_linear_system_3x3() {
        // 2x + y - z = 1, x - y + 2z = 5, x + 2y + z = 8
        // Solution: x=1, y=2, z=3
        // Verify: 2+2-3=1, 1-2+6=5, 1+4+3=8
        let a = vec![
            vec![2.0, 1.0, -1.0],
            vec![1.0, -1.0, 2.0],
            vec![1.0, 2.0, 1.0],
        ];
        let b = vec![1.0, 5.0, 8.0];
        let x = solve_linear_system(&a, &b);
        assert!(approx_eq(x[0], 1.0, 1e-8), "x[0] = {}", x[0]);
        assert!(approx_eq(x[1], 2.0, 1e-8), "x[1] = {}", x[1]);
        assert!(approx_eq(x[2], 3.0, 1e-8), "x[2] = {}", x[2]);
    }

    // --- Integration / consistency tests ---

    #[test]
    fn test_guinier_consistency_with_sphere() {
        // For a sphere of radius R, Rg = R * sqrt(3/5)
        let r = 20.0;
        let expected_rg = r * (3.0_f64 / 5.0).sqrt();
        let profile = make_sphere_profile(r, 1000.0, 0.001, 0.03, 100);
        let result = GuinierAnalysis::fit(&profile, 0.001, 0.03);
        assert!(
            approx_eq(result.rg, expected_rg, 1.0),
            "Rg = {}, expected {}",
            result.rg,
            expected_rg
        );
    }

    #[test]
    fn test_background_then_guinier() {
        let rg = 15.0;
        let i0 = 500.0;
        let bg = 10.0;
        let q: Vec<f64> = (1..100).map(|k| k as f64 * 0.001).collect();
        let i: Vec<f64> = q.iter().map(|&q_val| i0 * (-q_val * q_val * rg * rg / 3.0).exp() + bg).collect();
        let profile = SaxsProfile::new(q, i);
        let corrected = BackgroundSubtraction::subtract_constant(&profile, bg);
        let result = GuinierAnalysis::fit(&corrected, 0.001, 0.05);
        assert!(approx_eq(result.rg, rg, 1.0), "Rg = {}", result.rg);
    }

    #[test]
    fn test_form_factor_normalization() {
        // All form factors should be 1.0 at q=0
        assert!(approx_eq(FormFactorModels::sphere(0.0, 10.0), 1.0, EPSILON));
        assert!(approx_eq(FormFactorModels::gaussian_chain(0.0, 10.0), 1.0, EPSILON));
        assert!(approx_eq(FormFactorModels::ellipsoid(0.0, 10.0, 20.0), 1.0, 0.05));
    }

    #[test]
    fn test_hard_sphere_structure_factor_is_finite() {
        for q_val in [0.01, 0.05, 0.1, 0.5, 1.0, 5.0] {
            let sq = StructureFactors::hard_sphere(q_val, 10.0, 0.3);
            assert!(sq.is_finite(), "S({}) = {}", q_val, sq);
            assert!(sq > 0.0, "S({}) should be positive, got {}", q_val, sq);
        }
    }

    #[test]
    fn test_porod_surface_to_volume_positive() {
        let q: Vec<f64> = (1..100).map(|k| k as f64 * 0.01).collect();
        let i: Vec<f64> = q.iter().map(|&qv| 50.0 / qv.powi(4) + 0.1).collect();
        let profile = SaxsProfile::new(q, i);
        let result = PorodAnalysis::fit(&profile, 0.5, 0.99);
        assert!(result.surface_to_volume >= 0.0);
    }

    #[test]
    fn test_invariant_positive_for_positive_data() {
        let profile = SaxsProfile::new(
            vec![0.01, 0.05, 0.1, 0.5],
            vec![1000.0, 100.0, 10.0, 1.0],
        );
        let inv = PorodAnalysis::invariant(&profile);
        assert!(inv > 0.0, "Invariant = {}", inv);
    }

    #[test]
    fn test_profile_errors_preserved() {
        let p = SaxsProfile::with_errors(
            vec![0.01, 0.02, 0.03],
            vec![100.0, 50.0, 25.0],
            vec![2.0, 1.0, 0.5],
        );
        let bg_sub = BackgroundSubtraction::subtract_constant(&p, 5.0);
        assert!(bg_sub.errors.is_some());
        assert_eq!(bg_sub.errors.as_ref().unwrap().len(), 3);
    }

    #[test]
    fn test_fit_result_has_param_names() {
        let profile = make_sphere_profile(10.0, 100.0, 0.01, 0.5, 50);
        let result = FormFactorModels::fit_form_factor(&profile, FormFactorType::Sphere);
        assert_eq!(result.param_names.len(), 1);
        assert_eq!(result.param_names[0], "radius");
    }

    #[test]
    fn test_cylinder_fit_has_two_params() {
        let q: Vec<f64> = (1..50).map(|k| k as f64 * 0.01).collect();
        let i: Vec<f64> = q.iter().map(|&qv| FormFactorModels::cylinder(qv, 10.0, 100.0) * 500.0).collect();
        let profile = SaxsProfile::new(q, i);
        let result = FormFactorModels::fit_form_factor(&profile, FormFactorType::Cylinder);
        assert_eq!(result.params.len(), 2);
        assert_eq!(result.param_names[0], "radius");
        assert_eq!(result.param_names[1], "length");
    }

    #[test]
    fn test_ellipsoid_fit_has_two_params() {
        let q: Vec<f64> = (1..50).map(|k| k as f64 * 0.01).collect();
        let i: Vec<f64> = q.iter().map(|&qv| FormFactorModels::ellipsoid(qv, 10.0, 20.0) * 500.0).collect();
        let profile = SaxsProfile::new(q, i);
        let result = FormFactorModels::fit_form_factor(&profile, FormFactorType::Ellipsoid);
        assert_eq!(result.params.len(), 2);
    }

    #[test]
    fn test_size_distribution_output() {
        let profile = make_sphere_profile(10.0, 100.0, 0.01, 0.5, 30);
        let result = SizeDistribution::fit_distribution(
            &profile,
            FormFactorType::Sphere,
            DistributionType::LogNormal,
        );
        assert!(!result.distribution.is_empty());
        assert!(result.polydispersity > 0.0);
    }

    #[test]
    fn test_schulz_zimm_distribution_normalization() {
        // Integral of Schulz-Zimm should be approximately 1
        let r_avg = 10.0;
        let z = 10.0;
        let n = 1000;
        let r_max = 30.0;
        let dr = r_max / n as f64;
        let integral: f64 = (1..n)
            .map(|k| {
                let r = k as f64 * dr;
                SizeDistribution::schulz_zimm(r, r_avg, z) * dr
            })
            .sum();
        assert!(
            approx_eq(integral, 1.0, 0.05),
            "Schulz-Zimm integral = {}",
            integral
        );
    }

    #[test]
    fn test_gaussian_distribution_normalization() {
        let mean = 10.0;
        let sigma = 2.0;
        let n = 1000;
        let r_min = 0.0;
        let r_max = 20.0;
        let dr = (r_max - r_min) / n as f64;
        let integral: f64 = (0..n)
            .map(|k| {
                let r = r_min + k as f64 * dr;
                SizeDistribution::gaussian(r, mean, sigma) * dr
            })
            .sum();
        assert!(
            approx_eq(integral, 1.0, 0.05),
            "Gaussian integral = {}",
            integral
        );
    }
}
