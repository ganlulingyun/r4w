// trace:FR-NEUTRON-DIFF | ai:claude
//! # Neutron Diffraction Pattern Analyzer
//!
//! Implements neutron diffraction pattern analysis for crystallography and
//! materials science applications. Provides Bragg's law calculations, crystal
//! system d-spacing formulas, peak analysis with Gaussian fitting, Scherrer
//! crystallite size estimation, Williamson-Hall strain analysis, lattice
//! parameter refinement, phase identification, and Rietveld-style residuals.
//!
//! All math is implemented from scratch using only the standard library.

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Planck constant (J*s)
const PLANCK_H: f64 = 6.62607015e-34;
/// Neutron mass (kg)
const NEUTRON_MASS: f64 = 1.6749274980e-27;
/// Boltzmann constant (J/K)
const BOLTZMANN_K: f64 = 1.380649e-23;

// ---------------------------------------------------------------------------
// Miller Index
// ---------------------------------------------------------------------------

/// Miller index (h, k, l) for crystallographic planes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct MillerIndex {
    pub h: i32,
    pub k: i32,
    pub l: i32,
}

impl MillerIndex {
    pub fn new(h: i32, k: i32, l: i32) -> Self {
        Self { h, k, l }
    }

    /// Check if this reflection is allowed for BCC lattice.
    /// BCC rule: h + k + l must be even.
    pub fn is_allowed_bcc(&self) -> bool {
        (self.h + self.k + self.l) % 2 == 0
    }

    /// Check if this reflection is allowed for FCC lattice.
    /// FCC rule: h, k, l must be all odd or all even.
    pub fn is_allowed_fcc(&self) -> bool {
        let h_even = self.h % 2 == 0;
        let k_even = self.k % 2 == 0;
        let l_even = self.l % 2 == 0;
        (h_even == k_even) && (k_even == l_even)
    }

    /// Generate all Miller indices up to max_hkl (inclusive),
    /// excluding (0,0,0).
    pub fn generate_all(max_hkl: i32) -> Vec<MillerIndex> {
        let mut indices = Vec::new();
        for h in 0..=max_hkl {
            for k in 0..=max_hkl {
                for l in 0..=max_hkl {
                    if h == 0 && k == 0 && l == 0 {
                        continue;
                    }
                    indices.push(MillerIndex::new(h, k, l));
                }
            }
        }
        indices
    }

    /// Generate allowed reflections for BCC lattice up to max_hkl.
    pub fn generate_bcc(max_hkl: i32) -> Vec<MillerIndex> {
        Self::generate_all(max_hkl)
            .into_iter()
            .filter(|m| m.is_allowed_bcc())
            .collect()
    }

    /// Generate allowed reflections for FCC lattice up to max_hkl.
    pub fn generate_fcc(max_hkl: i32) -> Vec<MillerIndex> {
        Self::generate_all(max_hkl)
            .into_iter()
            .filter(|m| m.is_allowed_fcc())
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Crystal System
// ---------------------------------------------------------------------------

/// Crystal system for d-spacing calculations.
#[derive(Debug, Clone)]
pub enum CrystalSystem {
    /// Cubic: a = b = c, alpha = beta = gamma = 90 deg
    Cubic { a: f64 },
    /// Tetragonal: a = b != c, alpha = beta = gamma = 90 deg
    Tetragonal { a: f64, c: f64 },
    /// Hexagonal: a = b != c, alpha = beta = 90 deg, gamma = 120 deg
    Hexagonal { a: f64, c: f64 },
    /// Orthorhombic: a != b != c, alpha = beta = gamma = 90 deg
    Orthorhombic { a: f64, b: f64, c: f64 },
}

impl CrystalSystem {
    /// Calculate d-spacing for the given Miller index.
    /// Returns d in the same units as the lattice parameters (angstrom).
    pub fn d_spacing(&self, hkl: &MillerIndex) -> f64 {
        let (h, k, l) = (hkl.h as f64, hkl.k as f64, hkl.l as f64);
        let inv_d_sq = match self {
            CrystalSystem::Cubic { a } => {
                (h * h + k * k + l * l) / (a * a)
            }
            CrystalSystem::Tetragonal { a, c } => {
                (h * h + k * k) / (a * a) + (l * l) / (c * c)
            }
            CrystalSystem::Hexagonal { a, c } => {
                (4.0 / 3.0) * (h * h + h * k + k * k) / (a * a) + (l * l) / (c * c)
            }
            CrystalSystem::Orthorhombic { a, b, c } => {
                (h * h) / (a * a) + (k * k) / (b * b) + (l * l) / (c * c)
            }
        };
        if inv_d_sq <= 0.0 {
            return f64::INFINITY;
        }
        1.0 / inv_d_sq.sqrt()
    }

    /// Calculate unit cell volume.
    pub fn cell_volume(&self) -> f64 {
        match self {
            CrystalSystem::Cubic { a } => a * a * a,
            CrystalSystem::Tetragonal { a, c } => a * a * c,
            CrystalSystem::Hexagonal { a, c } => {
                // V = a^2 * c * sqrt(3)/2
                a * a * c * 3.0_f64.sqrt() / 2.0
            }
            CrystalSystem::Orthorhombic { a, b, c } => a * b * c,
        }
    }
}

// ---------------------------------------------------------------------------
// Bragg's Law helpers
// ---------------------------------------------------------------------------

/// Calculate d-spacing from a peak at `two_theta_deg` with wavelength in angstrom.
/// Bragg's law: n*lambda = 2*d*sin(theta), for n=1.
pub fn d_from_two_theta(two_theta_deg: f64, wavelength_angstrom: f64) -> f64 {
    let theta = two_theta_deg.to_radians() / 2.0;
    let sin_theta = theta.sin();
    if sin_theta.abs() < 1e-15 {
        return f64::INFINITY;
    }
    wavelength_angstrom / (2.0 * sin_theta)
}

/// Calculate 2-theta (degrees) from d-spacing and wavelength.
/// Returns None if the reflection is geometrically impossible (d too small).
pub fn two_theta_from_d(d_angstrom: f64, wavelength_angstrom: f64) -> Option<f64> {
    let sin_theta = wavelength_angstrom / (2.0 * d_angstrom);
    if sin_theta.abs() > 1.0 {
        return None;
    }
    Some(2.0 * sin_theta.asin().to_degrees())
}

/// Calculate d-spacing from Bragg's law with arbitrary order n.
pub fn d_from_two_theta_order(two_theta_deg: f64, wavelength_angstrom: f64, n: u32) -> f64 {
    let theta = two_theta_deg.to_radians() / 2.0;
    let sin_theta = theta.sin();
    if sin_theta.abs() < 1e-15 {
        return f64::INFINITY;
    }
    (n as f64) * wavelength_angstrom / (2.0 * sin_theta)
}

// ---------------------------------------------------------------------------
// De Broglie wavelength for neutrons
// ---------------------------------------------------------------------------

/// De Broglie wavelength for a neutron with velocity v (m/s).
/// Returns wavelength in angstrom.
pub fn de_broglie_wavelength(velocity_m_s: f64) -> f64 {
    let lambda_m = PLANCK_H / (NEUTRON_MASS * velocity_m_s);
    lambda_m * 1e10 // convert m to angstrom
}

/// Thermal neutron wavelength at temperature T (Kelvin).
/// lambda = h / sqrt(2 * m_n * k_B * T), returned in angstrom.
pub fn thermal_neutron_wavelength(temperature_k: f64) -> f64 {
    let lambda_m = PLANCK_H / (2.0 * NEUTRON_MASS * BOLTZMANN_K * temperature_k).sqrt();
    lambda_m * 1e10
}

// ---------------------------------------------------------------------------
// Diffraction Pattern
// ---------------------------------------------------------------------------

/// A measured diffraction pattern (2-theta, intensity).
#[derive(Debug, Clone)]
pub struct DiffractionPattern {
    /// 2-theta values in degrees.
    pub two_theta_deg: Vec<f64>,
    /// Measured intensities (counts).
    pub intensity: Vec<f64>,
    /// Neutron wavelength in angstrom.
    pub wavelength_angstrom: f64,
}

/// A found peak with position, amplitude, width, and area.
#[derive(Debug, Clone)]
pub struct Peak {
    /// Peak centre in 2-theta degrees.
    pub two_theta_deg: f64,
    /// Peak amplitude (counts above background).
    pub amplitude: f64,
    /// Gaussian sigma (degrees).
    pub sigma_deg: f64,
    /// Background level.
    pub background: f64,
}

impl Peak {
    /// Full-width at half-maximum (FWHM) in degrees.
    pub fn fwhm_deg(&self) -> f64 {
        2.0 * (2.0 * 2.0_f64.ln()).sqrt() * self.sigma_deg
    }

    /// FWHM in radians.
    pub fn fwhm_rad(&self) -> f64 {
        self.fwhm_deg().to_radians()
    }

    /// Integrated intensity (area under Gaussian, no background).
    pub fn integrated_intensity(&self) -> f64 {
        self.amplitude * self.sigma_deg * (2.0 * PI).sqrt()
    }

    /// Evaluate Gaussian at x (2-theta in degrees).
    pub fn evaluate(&self, x: f64) -> f64 {
        let dx = x - self.two_theta_deg;
        self.amplitude * (-dx * dx / (2.0 * self.sigma_deg * self.sigma_deg)).exp()
            + self.background
    }
}

impl DiffractionPattern {
    /// Create a new diffraction pattern.
    pub fn new(two_theta_deg: Vec<f64>, intensity: Vec<f64>, wavelength_angstrom: f64) -> Self {
        assert_eq!(two_theta_deg.len(), intensity.len());
        Self {
            two_theta_deg,
            intensity,
            wavelength_angstrom,
        }
    }

    /// Number of data points.
    pub fn len(&self) -> usize {
        self.two_theta_deg.len()
    }

    /// Whether the pattern is empty.
    pub fn is_empty(&self) -> bool {
        self.two_theta_deg.is_empty()
    }

    /// Find peaks above `threshold` by detecting local maxima.
    /// Returns indices of found peaks.
    pub fn find_peaks(&self, threshold: f64) -> Vec<usize> {
        let n = self.intensity.len();
        if n < 3 {
            return Vec::new();
        }
        let mut peaks = Vec::new();
        for i in 1..n - 1 {
            if self.intensity[i] > threshold
                && self.intensity[i] >= self.intensity[i - 1]
                && self.intensity[i] >= self.intensity[i + 1]
            {
                peaks.push(i);
            }
        }
        peaks
    }

    /// Refine peak position using parabolic (3-point) interpolation
    /// around the given index. Returns refined 2-theta in degrees.
    pub fn refine_peak_position(&self, idx: usize) -> f64 {
        let n = self.intensity.len();
        if idx == 0 || idx >= n - 1 {
            return self.two_theta_deg[idx];
        }
        let y_m1 = self.intensity[idx - 1];
        let y_0 = self.intensity[idx];
        let y_p1 = self.intensity[idx + 1];
        let denom = 2.0 * (y_m1 - 2.0 * y_0 + y_p1);
        if denom.abs() < 1e-15 {
            return self.two_theta_deg[idx];
        }
        let delta = (y_m1 - y_p1) / denom;
        let step = self.two_theta_deg[idx] - self.two_theta_deg[idx.saturating_sub(1)];
        self.two_theta_deg[idx] + delta * step
    }

    /// Fit a Gaussian peak around index `idx` within a window of `half_width`
    /// points on each side. Returns the fitted Peak.
    pub fn fit_gaussian(&self, idx: usize, half_width: usize) -> Peak {
        let n = self.intensity.len();
        let lo = if idx >= half_width { idx - half_width } else { 0 };
        let hi = if idx + half_width < n { idx + half_width } else { n - 1 };

        // Estimate background from wing averages
        let bg_lo = self.intensity[lo];
        let bg_hi = self.intensity[hi];
        let background = (bg_lo + bg_hi) / 2.0;

        // Refined peak position
        let x0 = self.refine_peak_position(idx);
        let amplitude = self.intensity[idx] - background;

        // Estimate sigma from second moment of the peak region
        let mut sum_w = 0.0;
        let mut sum_wx2 = 0.0;
        for i in lo..=hi {
            let w = (self.intensity[i] - background).max(0.0);
            let dx = self.two_theta_deg[i] - x0;
            sum_w += w;
            sum_wx2 += w * dx * dx;
        }
        let sigma = if sum_w > 0.0 {
            (sum_wx2 / sum_w).sqrt().max(1e-6)
        } else {
            0.1
        };

        Peak {
            two_theta_deg: x0,
            amplitude: amplitude.max(0.0),
            sigma_deg: sigma,
            background,
        }
    }

    /// Extract d-spacings from detected peaks above threshold.
    pub fn extract_d_spacings(&self, threshold: f64) -> Vec<f64> {
        self.find_peaks(threshold)
            .iter()
            .map(|&idx| {
                let tt = self.refine_peak_position(idx);
                d_from_two_theta(tt, self.wavelength_angstrom)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Scherrer crystallite size
// ---------------------------------------------------------------------------

/// Scherrer equation: D = K * lambda / (beta * cos(theta))
/// where beta = FWHM in radians, K ~ 0.9.
pub fn scherrer_size(
    wavelength_angstrom: f64,
    fwhm_rad: f64,
    two_theta_deg: f64,
    k: f64,
) -> f64 {
    let theta = two_theta_deg.to_radians() / 2.0;
    k * wavelength_angstrom / (fwhm_rad * theta.cos())
}

/// Correct observed broadening for instrumental broadening.
/// beta_sample^2 = beta_obs^2 - beta_inst^2
/// Returns corrected beta (radians), or 0 if instrumental dominates.
pub fn correct_broadening(beta_observed_rad: f64, beta_instrument_rad: f64) -> f64 {
    let diff = beta_observed_rad * beta_observed_rad
        - beta_instrument_rad * beta_instrument_rad;
    if diff <= 0.0 {
        0.0
    } else {
        diff.sqrt()
    }
}

// ---------------------------------------------------------------------------
// Williamson-Hall strain analysis
// ---------------------------------------------------------------------------

/// Result from a Williamson-Hall analysis.
#[derive(Debug, Clone)]
pub struct WilliamsonHallResult {
    /// Crystallite size D (angstrom).
    pub size_angstrom: f64,
    /// Microstrain epsilon (dimensionless).
    pub strain: f64,
    /// R-squared of the linear fit.
    pub r_squared: f64,
}

/// Perform Williamson-Hall analysis from a set of peaks.
/// Plots beta*cos(theta) vs 4*sin(theta) and fits a line:
///   beta*cos(theta) = K*lambda/D + 4*epsilon*sin(theta)
/// where beta is FWHM in radians.
pub fn williamson_hall(
    peaks: &[Peak],
    wavelength_angstrom: f64,
    k: f64,
) -> Option<WilliamsonHallResult> {
    if peaks.len() < 2 {
        return None;
    }
    // x = 4*sin(theta), y = beta*cos(theta)
    let mut xs = Vec::with_capacity(peaks.len());
    let mut ys = Vec::with_capacity(peaks.len());
    for p in peaks {
        let theta = p.two_theta_deg.to_radians() / 2.0;
        let beta = p.fwhm_rad();
        xs.push(4.0 * theta.sin());
        ys.push(beta * theta.cos());
    }
    let (slope, intercept, r_sq) = linear_fit(&xs, &ys);
    let size = if intercept.abs() > 1e-15 {
        k * wavelength_angstrom / intercept
    } else {
        f64::INFINITY
    };
    Some(WilliamsonHallResult {
        size_angstrom: size.abs(),
        strain: slope.abs(),
        r_squared: r_sq,
    })
}

// ---------------------------------------------------------------------------
// Lattice parameter refinement (least-squares)
// ---------------------------------------------------------------------------

/// Refine the cubic lattice parameter `a` from a set of (d, hkl) observations.
/// Uses least-squares: a^2 = mean( d^2 * (h^2+k^2+l^2) ).
pub fn refine_cubic(
    observations: &[(f64, MillerIndex)],
) -> f64 {
    if observations.is_empty() {
        return 0.0;
    }
    let mut sum_a_sq = 0.0;
    for (d, hkl) in observations {
        let q = (hkl.h * hkl.h + hkl.k * hkl.k + hkl.l * hkl.l) as f64;
        sum_a_sq += d * d * q;
    }
    (sum_a_sq / observations.len() as f64).sqrt()
}

/// Nelson-Riley extrapolation function for systematic error correction.
/// F(theta) = cos^2(theta)/sin(theta) + cos^2(theta)/theta
/// Used to extrapolate lattice parameter to theta = 90 deg.
pub fn nelson_riley(two_theta_deg: f64) -> f64 {
    let theta = two_theta_deg.to_radians() / 2.0;
    let cos_t = theta.cos();
    let sin_t = theta.sin();
    if sin_t.abs() < 1e-15 || theta.abs() < 1e-15 {
        return f64::INFINITY;
    }
    cos_t * cos_t / sin_t + cos_t * cos_t / theta
}

/// Refine cubic lattice parameter using Nelson-Riley extrapolation.
/// Performs linear fit of a_i vs F(theta_i) and extrapolates to F=0.
pub fn refine_cubic_nelson_riley(
    observations: &[(f64, MillerIndex)],
) -> f64 {
    if observations.len() < 2 {
        if let Some((d, hkl)) = observations.first() {
            let q = (hkl.h * hkl.h + hkl.k * hkl.k + hkl.l * hkl.l) as f64;
            return d * q.sqrt();
        }
        return 0.0;
    }
    let mut xs = Vec::with_capacity(observations.len());
    let mut ys = Vec::with_capacity(observations.len());
    for (d, hkl) in observations {
        let q = (hkl.h * hkl.h + hkl.k * hkl.k + hkl.l * hkl.l) as f64;
        let a_i = d * q.sqrt();
        let two_theta = two_theta_from_d(*d, 1.0).unwrap_or(90.0); // placeholder
        // Recompute two_theta from the actual d-spacing and a rough wavelength
        // For N-R, we just need F(theta) which depends on the angle
        // We'll compute theta directly from d and q:  sin(theta) = lambda/(2d)
        // but we don't have lambda here, so use the observation angle proxy
        let nr = nelson_riley(two_theta);
        if nr.is_finite() {
            xs.push(nr);
            ys.push(a_i);
        }
    }
    if xs.len() < 2 {
        return refine_cubic(observations);
    }
    let (_slope, intercept, _r_sq) = linear_fit(&xs, &ys);
    intercept
}

/// Refine cubic lattice parameter with Nelson-Riley using known wavelength.
pub fn refine_cubic_nelson_riley_with_wavelength(
    observations: &[(f64, MillerIndex)],
    wavelength_angstrom: f64,
) -> f64 {
    if observations.len() < 2 {
        return refine_cubic(observations);
    }
    let mut xs = Vec::with_capacity(observations.len());
    let mut ys = Vec::with_capacity(observations.len());
    for (d, hkl) in observations {
        let q = (hkl.h * hkl.h + hkl.k * hkl.k + hkl.l * hkl.l) as f64;
        let a_i = d * q.sqrt();
        if let Some(two_theta) = two_theta_from_d(*d, wavelength_angstrom) {
            let nr = nelson_riley(two_theta);
            if nr.is_finite() {
                xs.push(nr);
                ys.push(a_i);
            }
        }
    }
    if xs.len() < 2 {
        return refine_cubic(observations);
    }
    let (_slope, intercept, _r_sq) = linear_fit(&xs, &ys);
    intercept
}

// ---------------------------------------------------------------------------
// Phase identification
// ---------------------------------------------------------------------------

/// A reference phase entry with name and expected d-spacings.
#[derive(Debug, Clone)]
pub struct PhaseReference {
    pub name: String,
    pub d_spacings: Vec<f64>,
}

/// Result of phase matching.
#[derive(Debug, Clone)]
pub struct PhaseMatch {
    pub name: String,
    /// Figure of merit: fraction of reference d-spacings matched.
    pub figure_of_merit: f64,
    /// Number of matched peaks.
    pub matched_count: usize,
}

/// Match observed d-spacings against reference phases.
/// `tolerance` is the allowed relative difference (e.g. 0.02 = 2%).
pub fn identify_phase(
    observed_d: &[f64],
    references: &[PhaseReference],
    tolerance: f64,
) -> Vec<PhaseMatch> {
    let mut results = Vec::new();
    for reference in references {
        let mut matched = 0usize;
        for ref_d in &reference.d_spacings {
            for obs_d in observed_d {
                let rel_diff = ((obs_d - ref_d) / ref_d).abs();
                if rel_diff < tolerance {
                    matched += 1;
                    break;
                }
            }
        }
        let fom = if reference.d_spacings.is_empty() {
            0.0
        } else {
            matched as f64 / reference.d_spacings.len() as f64
        };
        results.push(PhaseMatch {
            name: reference.name.clone(),
            figure_of_merit: fom,
            matched_count: matched,
        });
    }
    results.sort_by(|a, b| b.figure_of_merit.partial_cmp(&a.figure_of_merit).unwrap());
    results
}

// ---------------------------------------------------------------------------
// Rietveld-style residuals
// ---------------------------------------------------------------------------

/// Rietveld residual results.
#[derive(Debug, Clone)]
pub struct RietveldResiduals {
    /// Weighted profile R-factor.
    pub r_wp: f64,
    /// Profile R-factor.
    pub r_p: f64,
    /// Expected R-factor from counting statistics.
    pub r_exp: f64,
    /// Goodness-of-fit chi-squared.
    pub chi_squared: f64,
}

/// Compute Rietveld-style residuals between observed and calculated patterns.
/// Weights are 1/y_obs for counting statistics (Poisson).
pub fn rietveld_residuals(y_obs: &[f64], y_calc: &[f64]) -> RietveldResiduals {
    assert_eq!(y_obs.len(), y_calc.len());
    let n = y_obs.len() as f64;

    let mut sum_w_diff2 = 0.0;
    let mut sum_w_obs2 = 0.0;
    let mut sum_abs_diff = 0.0;
    let mut sum_obs = 0.0;
    let mut sum_w = 0.0;
    let mut num_params = 0.0_f64; // simplified: 0 free params

    for i in 0..y_obs.len() {
        let yo = y_obs[i];
        let yc = y_calc[i];
        // Weight: w_i = 1/yo for Poisson counting statistics
        let w = if yo > 0.0 { 1.0 / yo } else { 1.0 };
        let diff = yo - yc;
        sum_w_diff2 += w * diff * diff;
        sum_w_obs2 += w * yo * yo;
        sum_abs_diff += diff.abs();
        sum_obs += yo;
        sum_w += w;
    }
    let _ = num_params;

    let r_wp = if sum_w_obs2 > 0.0 {
        (sum_w_diff2 / sum_w_obs2).sqrt()
    } else {
        0.0
    };

    let r_p = if sum_obs > 0.0 {
        sum_abs_diff / sum_obs
    } else {
        0.0
    };

    // R_exp = sqrt( (N - P) / sum(w_i * y_obs_i^2) )
    // Simplified with P=0
    let r_exp = if sum_w_obs2 > 0.0 {
        (n / sum_w_obs2).sqrt()
    } else {
        0.0
    };

    let chi_squared = if r_exp > 0.0 {
        (r_wp / r_exp) * (r_wp / r_exp)
    } else {
        0.0
    };

    RietveldResiduals {
        r_wp,
        r_p,
        r_exp,
        chi_squared,
    }
}

// ---------------------------------------------------------------------------
// Generate a synthetic pattern from a crystal system
// ---------------------------------------------------------------------------

/// Generate a simulated diffraction pattern from a crystal system.
/// Each allowed reflection produces a Gaussian peak with Lorentz-polarization
/// factor approximation.
pub fn simulate_pattern(
    crystal: &CrystalSystem,
    hkl_list: &[MillerIndex],
    wavelength_angstrom: f64,
    two_theta_range: (f64, f64),
    num_points: usize,
    sigma_deg: f64,
) -> DiffractionPattern {
    let step = (two_theta_range.1 - two_theta_range.0) / (num_points - 1) as f64;
    let two_theta: Vec<f64> = (0..num_points)
        .map(|i| two_theta_range.0 + i as f64 * step)
        .collect();
    let mut intensity = vec![0.0; num_points];

    for hkl in hkl_list {
        let d = crystal.d_spacing(hkl);
        if let Some(tt) = two_theta_from_d(d, wavelength_angstrom) {
            if tt >= two_theta_range.0 && tt <= two_theta_range.1 {
                let theta = tt.to_radians() / 2.0;
                // Simple Lorentz-polarization factor
                let lp = (1.0 + (2.0 * theta).cos().powi(2))
                    / (theta.sin().powi(2) * theta.cos());
                let amp = lp.abs().min(1000.0); // clamp for display
                for (i, y) in intensity.iter_mut().enumerate() {
                    let dx = two_theta[i] - tt;
                    *y += amp * (-dx * dx / (2.0 * sigma_deg * sigma_deg)).exp();
                }
            }
        }
    }

    DiffractionPattern::new(two_theta, intensity, wavelength_angstrom)
}

// ---------------------------------------------------------------------------
// Internal helper: simple linear regression y = slope*x + intercept
// ---------------------------------------------------------------------------

fn linear_fit(xs: &[f64], ys: &[f64]) -> (f64, f64, f64) {
    let n = xs.len() as f64;
    if n < 2.0 {
        return (0.0, ys.first().copied().unwrap_or(0.0), 0.0);
    }
    let sum_x: f64 = xs.iter().sum();
    let sum_y: f64 = ys.iter().sum();
    let sum_xy: f64 = xs.iter().zip(ys.iter()).map(|(x, y)| x * y).sum();
    let sum_x2: f64 = xs.iter().map(|x| x * x).sum();

    let denom = n * sum_x2 - sum_x * sum_x;
    if denom.abs() < 1e-30 {
        return (0.0, sum_y / n, 0.0);
    }
    let slope = (n * sum_xy - sum_x * sum_y) / denom;
    let intercept = (sum_y - slope * sum_x) / n;

    // R-squared
    let mean_y = sum_y / n;
    let ss_tot: f64 = ys.iter().map(|y| (y - mean_y).powi(2)).sum();
    let ss_res: f64 = xs
        .iter()
        .zip(ys.iter())
        .map(|(x, y)| {
            let pred = slope * x + intercept;
            (y - pred).powi(2)
        })
        .sum();
    let r_sq = if ss_tot > 0.0 { 1.0 - ss_res / ss_tot } else { 1.0 };

    (slope, intercept, r_sq)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -- Miller Index tests ------------------------------------------------

    #[test]
    fn test_miller_index_bcc_allowed() {
        // BCC: h+k+l even
        assert!(MillerIndex::new(1, 1, 0).is_allowed_bcc());
        assert!(MillerIndex::new(2, 0, 0).is_allowed_bcc());
        assert!(MillerIndex::new(2, 1, 1).is_allowed_bcc());
        assert!(!MillerIndex::new(1, 0, 0).is_allowed_bcc());
        assert!(!MillerIndex::new(1, 1, 1).is_allowed_bcc());
    }

    #[test]
    fn test_miller_index_fcc_allowed() {
        // FCC: all odd or all even
        assert!(MillerIndex::new(1, 1, 1).is_allowed_fcc());
        assert!(MillerIndex::new(2, 0, 0).is_allowed_fcc());
        assert!(MillerIndex::new(2, 2, 0).is_allowed_fcc());
        assert!(!MillerIndex::new(1, 1, 0).is_allowed_fcc());
        assert!(!MillerIndex::new(2, 1, 0).is_allowed_fcc());
    }

    #[test]
    fn test_generate_all_excludes_origin() {
        let all = MillerIndex::generate_all(1);
        assert!(!all.contains(&MillerIndex::new(0, 0, 0)));
        // 2^3 - 1 = 7 combinations for max=1 with h,k,l in [0,1]
        assert_eq!(all.len(), 7);
    }

    #[test]
    fn test_generate_bcc() {
        let bcc = MillerIndex::generate_bcc(2);
        for hkl in &bcc {
            assert!(hkl.is_allowed_bcc());
        }
        // Verify (1,0,0) which is h+k+l=1 (odd) is NOT present
        assert!(!bcc.contains(&MillerIndex::new(1, 0, 0)));
    }

    #[test]
    fn test_generate_fcc() {
        let fcc = MillerIndex::generate_fcc(2);
        for hkl in &fcc {
            assert!(hkl.is_allowed_fcc());
        }
        assert!(fcc.contains(&MillerIndex::new(1, 1, 1)));
        assert!(fcc.contains(&MillerIndex::new(2, 0, 0)));
        assert!(!fcc.contains(&MillerIndex::new(1, 1, 0)));
    }

    // -- Crystal system d-spacing tests ------------------------------------

    #[test]
    fn test_cubic_d_spacing() {
        let cubic = CrystalSystem::Cubic { a: 5.0 };
        let d100 = cubic.d_spacing(&MillerIndex::new(1, 0, 0));
        assert!(approx_eq(d100, 5.0, EPSILON));
        let d110 = cubic.d_spacing(&MillerIndex::new(1, 1, 0));
        assert!(approx_eq(d110, 5.0 / 2.0_f64.sqrt(), EPSILON));
        let d111 = cubic.d_spacing(&MillerIndex::new(1, 1, 1));
        assert!(approx_eq(d111, 5.0 / 3.0_f64.sqrt(), EPSILON));
    }

    #[test]
    fn test_tetragonal_d_spacing() {
        let tet = CrystalSystem::Tetragonal { a: 4.0, c: 6.0 };
        let d100 = tet.d_spacing(&MillerIndex::new(1, 0, 0));
        assert!(approx_eq(d100, 4.0, EPSILON));
        let d001 = tet.d_spacing(&MillerIndex::new(0, 0, 1));
        assert!(approx_eq(d001, 6.0, EPSILON));
    }

    #[test]
    fn test_hexagonal_d_spacing() {
        let hex = CrystalSystem::Hexagonal { a: 3.0, c: 5.0 };
        let d001 = hex.d_spacing(&MillerIndex::new(0, 0, 1));
        assert!(approx_eq(d001, 5.0, EPSILON));
        // (1,0,0): 1/d^2 = (4/3)/a^2 => d = a * sqrt(3)/2
        let d100 = hex.d_spacing(&MillerIndex::new(1, 0, 0));
        let expected = 3.0 * 3.0_f64.sqrt() / 2.0;
        assert!(approx_eq(d100, expected, EPSILON));
    }

    #[test]
    fn test_orthorhombic_d_spacing() {
        let orth = CrystalSystem::Orthorhombic {
            a: 3.0,
            b: 4.0,
            c: 5.0,
        };
        let d100 = orth.d_spacing(&MillerIndex::new(1, 0, 0));
        assert!(approx_eq(d100, 3.0, EPSILON));
        let d010 = orth.d_spacing(&MillerIndex::new(0, 1, 0));
        assert!(approx_eq(d010, 4.0, EPSILON));
        let d001 = orth.d_spacing(&MillerIndex::new(0, 0, 1));
        assert!(approx_eq(d001, 5.0, EPSILON));
    }

    #[test]
    fn test_cell_volume_cubic() {
        let c = CrystalSystem::Cubic { a: 3.0 };
        assert!(approx_eq(c.cell_volume(), 27.0, EPSILON));
    }

    #[test]
    fn test_cell_volume_hexagonal() {
        let h = CrystalSystem::Hexagonal { a: 3.0, c: 5.0 };
        let expected = 9.0 * 5.0 * 3.0_f64.sqrt() / 2.0;
        assert!(approx_eq(h.cell_volume(), expected, EPSILON));
    }

    #[test]
    fn test_cell_volume_orthorhombic() {
        let o = CrystalSystem::Orthorhombic {
            a: 2.0,
            b: 3.0,
            c: 4.0,
        };
        assert!(approx_eq(o.cell_volume(), 24.0, EPSILON));
    }

    // -- Bragg's law tests -------------------------------------------------

    #[test]
    fn test_d_from_two_theta() {
        // Known: NaCl (100), a=5.64 A, Cu Kalpha lambda=1.5406 A
        // 2theta for (200) ~ 31.7 deg
        let lambda = 1.5406;
        let d = d_from_two_theta(31.7, lambda);
        // d(200) = a/2 = 2.82 A
        assert!(approx_eq(d, 2.82, 0.01));
    }

    #[test]
    fn test_two_theta_from_d() {
        let lambda = 1.5406;
        let d = 2.82;
        let tt = two_theta_from_d(d, lambda).unwrap();
        // Roundtrip check
        let d_back = d_from_two_theta(tt, lambda);
        assert!(approx_eq(d_back, d, 1e-4));
    }

    #[test]
    fn test_two_theta_impossible() {
        // d < lambda/2 => sin(theta) > 1 => impossible
        assert!(two_theta_from_d(0.5, 1.5406).is_none());
    }

    #[test]
    fn test_bragg_roundtrip() {
        let lambda = 1.8;
        let d = 3.5;
        let tt = two_theta_from_d(d, lambda).unwrap();
        let d2 = d_from_two_theta(tt, lambda);
        assert!(approx_eq(d, d2, 1e-10));
    }

    #[test]
    fn test_d_from_two_theta_order() {
        let lambda = 1.5;
        let d1 = d_from_two_theta(30.0, lambda);
        let d2 = d_from_two_theta_order(30.0, lambda, 2);
        assert!(approx_eq(d2, 2.0 * d1, 1e-10));
    }

    // -- De Broglie wavelength tests ---------------------------------------

    #[test]
    fn test_thermal_neutron_wavelength() {
        // At room temperature (293 K), thermal neutrons ~ 1.45 A
        let lambda = thermal_neutron_wavelength(293.0);
        assert!(lambda > 1.0 && lambda < 2.0);
    }

    #[test]
    fn test_de_broglie_wavelength() {
        // Neutron at 2200 m/s (thermal) should be ~1.8 A
        let lambda = de_broglie_wavelength(2200.0);
        assert!(lambda > 1.5 && lambda < 2.0);
    }

    #[test]
    fn test_de_broglie_higher_velocity_shorter() {
        let l1 = de_broglie_wavelength(2000.0);
        let l2 = de_broglie_wavelength(4000.0);
        assert!(l2 < l1);
    }

    // -- Peak analysis tests -----------------------------------------------

    #[test]
    fn test_find_peaks_simple() {
        let two_theta: Vec<f64> = (0..100).map(|i| i as f64 * 0.5).collect();
        let mut intensity = vec![1.0; 100];
        intensity[20] = 10.0;
        intensity[50] = 15.0;
        intensity[80] = 8.0;
        let pat = DiffractionPattern::new(two_theta, intensity, 1.5);
        let peaks = pat.find_peaks(5.0);
        assert_eq!(peaks.len(), 3);
        assert!(peaks.contains(&20));
        assert!(peaks.contains(&50));
        assert!(peaks.contains(&80));
    }

    #[test]
    fn test_find_peaks_below_threshold() {
        let two_theta: Vec<f64> = (0..50).map(|i| i as f64).collect();
        let intensity = vec![1.0; 50];
        let pat = DiffractionPattern::new(two_theta, intensity, 1.5);
        let peaks = pat.find_peaks(5.0);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_refine_peak_position() {
        // Symmetric peak: maximum already at centre
        let two_theta = vec![9.0, 10.0, 11.0];
        let intensity = vec![5.0, 10.0, 5.0];
        let pat = DiffractionPattern::new(two_theta, intensity, 1.5);
        let refined = pat.refine_peak_position(1);
        assert!(approx_eq(refined, 10.0, 0.01));
    }

    #[test]
    fn test_refine_peak_asymmetric() {
        // Peak slightly to the right
        let two_theta = vec![9.0, 10.0, 11.0];
        let intensity = vec![4.0, 10.0, 6.0];
        let pat = DiffractionPattern::new(two_theta, intensity, 1.5);
        let refined = pat.refine_peak_position(1);
        // Should be shifted slightly positive
        assert!(refined > 10.0);
        assert!(refined < 11.0);
    }

    #[test]
    fn test_gaussian_peak_fwhm() {
        let sigma = 0.5;
        let peak = Peak {
            two_theta_deg: 30.0,
            amplitude: 100.0,
            sigma_deg: sigma,
            background: 5.0,
        };
        let fwhm_expected = 2.0 * (2.0 * 2.0_f64.ln()).sqrt() * sigma;
        assert!(approx_eq(peak.fwhm_deg(), fwhm_expected, EPSILON));
    }

    #[test]
    fn test_gaussian_peak_evaluate() {
        let peak = Peak {
            two_theta_deg: 30.0,
            amplitude: 100.0,
            sigma_deg: 1.0,
            background: 5.0,
        };
        // At centre
        assert!(approx_eq(peak.evaluate(30.0), 105.0, EPSILON));
        // At +/- 1 sigma
        let val = peak.evaluate(31.0);
        let expected = 100.0 * (-0.5_f64).exp() + 5.0;
        assert!(approx_eq(val, expected, EPSILON));
    }

    #[test]
    fn test_integrated_intensity() {
        let peak = Peak {
            two_theta_deg: 30.0,
            amplitude: 100.0,
            sigma_deg: 1.0,
            background: 0.0,
        };
        let area = peak.integrated_intensity();
        let expected = 100.0 * (2.0 * PI).sqrt();
        assert!(approx_eq(area, expected, 0.01));
    }

    #[test]
    fn test_fit_gaussian_synthetic() {
        // Build a synthetic Gaussian peak
        let sigma = 0.3;
        let centre = 45.0;
        let amp = 200.0;
        let bg = 10.0;
        let n = 201;
        let two_theta: Vec<f64> = (0..n).map(|i| 40.0 + i as f64 * 0.05).collect();
        let intensity: Vec<f64> = two_theta
            .iter()
            .map(|&x| {
                let dx = x - centre;
                amp * (-dx * dx / (2.0 * sigma * sigma)).exp() + bg
            })
            .collect();
        let pat = DiffractionPattern::new(two_theta, intensity, 1.5);
        // Peak at index 100 (centre)
        let fitted = pat.fit_gaussian(100, 30);
        assert!(approx_eq(fitted.two_theta_deg, centre, 0.05));
        assert!(approx_eq(fitted.amplitude, amp, 5.0));
        assert!(approx_eq(fitted.sigma_deg, sigma, 0.1));
    }

    // -- Scherrer equation tests -------------------------------------------

    #[test]
    fn test_scherrer_size() {
        let lambda = 1.5406;
        let fwhm_rad = 0.002; // ~0.115 deg
        let two_theta = 44.0;
        let size = scherrer_size(lambda, fwhm_rad, two_theta, 0.9);
        // D ~ 0.9 * 1.5406 / (0.002 * cos(22deg)) ~ 747 A
        assert!(size > 500.0 && size < 1000.0);
    }

    #[test]
    fn test_correct_broadening() {
        let beta_obs = 0.005;
        let beta_inst = 0.003;
        let beta_sample = correct_broadening(beta_obs, beta_inst);
        let expected = (0.005_f64 * 0.005 - 0.003 * 0.003).sqrt();
        assert!(approx_eq(beta_sample, expected, EPSILON));
    }

    #[test]
    fn test_correct_broadening_instrumental_dominates() {
        let beta = correct_broadening(0.002, 0.005);
        assert!(approx_eq(beta, 0.0, EPSILON));
    }

    // -- Williamson-Hall tests ---------------------------------------------

    #[test]
    fn test_williamson_hall_basic() {
        // Create synthetic peaks with known size and strain
        let lambda = 1.5406;
        let d_size = 500.0; // 500 angstrom crystallite
        let strain = 0.001;
        let k = 0.9;
        let angles = [30.0, 45.0, 60.0, 75.0, 90.0];
        let peaks: Vec<Peak> = angles
            .iter()
            .map(|&tt: &f64| {
                let theta = tt.to_radians() / 2.0;
                // beta*cos(theta) = K*lambda/D + 4*epsilon*sin(theta)
                let beta_cos = k * lambda / d_size + 4.0 * strain * theta.sin();
                let beta = beta_cos / theta.cos();
                let sigma = beta / (2.0 * (2.0 * 2.0_f64.ln()).sqrt());
                Peak {
                    two_theta_deg: tt,
                    amplitude: 100.0,
                    sigma_deg: sigma.to_degrees(),
                    background: 0.0,
                }
            })
            .collect();
        let result = williamson_hall(&peaks, lambda, k).unwrap();
        assert!(approx_eq(result.size_angstrom, d_size, 5.0));
        assert!(approx_eq(result.strain, strain, 1e-4));
    }

    #[test]
    fn test_williamson_hall_insufficient_peaks() {
        let result = williamson_hall(&[], 1.5, 0.9);
        assert!(result.is_none());
    }

    // -- Lattice parameter refinement tests --------------------------------

    #[test]
    fn test_refine_cubic() {
        let a_true = 4.05; // Aluminum
        let cubic = CrystalSystem::Cubic { a: a_true };
        let hkls = vec![
            MillerIndex::new(1, 1, 1),
            MillerIndex::new(2, 0, 0),
            MillerIndex::new(2, 2, 0),
            MillerIndex::new(3, 1, 1),
        ];
        let obs: Vec<(f64, MillerIndex)> = hkls
            .iter()
            .map(|hkl| (cubic.d_spacing(hkl), *hkl))
            .collect();
        let a_refined = refine_cubic(&obs);
        assert!(approx_eq(a_refined, a_true, 1e-6));
    }

    #[test]
    fn test_nelson_riley_function() {
        // At 2theta = 90 deg, theta = 45 deg
        let nr = nelson_riley(90.0);
        // cos^2(45)/sin(45) + cos^2(45)/(45*pi/180)
        let theta = 45.0_f64.to_radians();
        let expected = theta.cos().powi(2) / theta.sin()
            + theta.cos().powi(2) / theta;
        assert!(approx_eq(nr, expected, EPSILON));
    }

    // -- Phase identification tests ----------------------------------------

    #[test]
    fn test_phase_identification() {
        let references = vec![
            PhaseReference {
                name: "Alpha".to_string(),
                d_spacings: vec![3.0, 2.5, 2.0, 1.5],
            },
            PhaseReference {
                name: "Beta".to_string(),
                d_spacings: vec![3.5, 2.8, 1.8],
            },
        ];
        // Observed matches Alpha closely
        let observed = vec![3.01, 2.49, 2.01, 1.51];
        let matches = identify_phase(&observed, &references, 0.02);
        assert_eq!(matches[0].name, "Alpha");
        assert!(approx_eq(matches[0].figure_of_merit, 1.0, EPSILON));
    }

    #[test]
    fn test_phase_identification_partial_match() {
        let references = vec![PhaseReference {
            name: "Gamma".to_string(),
            d_spacings: vec![4.0, 3.0, 2.0, 1.0],
        }];
        let observed = vec![4.01, 2.01]; // 2 of 4
        let matches = identify_phase(&observed, &references, 0.02);
        assert!(approx_eq(matches[0].figure_of_merit, 0.5, EPSILON));
        assert_eq!(matches[0].matched_count, 2);
    }

    // -- Rietveld residuals tests ------------------------------------------

    #[test]
    fn test_rietveld_perfect_fit() {
        let y_obs = vec![10.0, 20.0, 30.0, 40.0];
        let y_calc = y_obs.clone();
        let res = rietveld_residuals(&y_obs, &y_calc);
        assert!(approx_eq(res.r_wp, 0.0, EPSILON));
        assert!(approx_eq(res.r_p, 0.0, EPSILON));
    }

    #[test]
    fn test_rietveld_nonzero() {
        let y_obs = vec![100.0, 200.0, 300.0];
        let y_calc = vec![95.0, 210.0, 290.0];
        let res = rietveld_residuals(&y_obs, &y_calc);
        assert!(res.r_wp > 0.0);
        assert!(res.r_p > 0.0);
        assert!(res.chi_squared > 0.0);
    }

    // -- Pattern simulation test -------------------------------------------

    #[test]
    fn test_simulate_pattern() {
        let cubic = CrystalSystem::Cubic { a: 5.64 }; // NaCl
        let hkls = MillerIndex::generate_fcc(3);
        let lambda = 1.5406;
        let pat = simulate_pattern(&cubic, &hkls, lambda, (10.0, 120.0), 1000, 0.2);
        assert_eq!(pat.len(), 1000);
        // Should have some peaks above background
        let max_intensity = pat.intensity.iter().cloned().fold(0.0f64, f64::max);
        assert!(max_intensity > 1.0);
    }

    #[test]
    fn test_extract_d_spacings() {
        let cubic = CrystalSystem::Cubic { a: 4.05 }; // Al
        let hkls = MillerIndex::generate_fcc(3);
        let lambda = 1.5406;
        let pat = simulate_pattern(&cubic, &hkls, lambda, (20.0, 120.0), 2000, 0.15);
        let d_vals = pat.extract_d_spacings(0.5);
        // Should find several d-spacings
        assert!(d_vals.len() >= 2);
    }

    // -- Diffraction pattern length / is_empty tests ----------------------

    #[test]
    fn test_diffraction_pattern_empty() {
        let pat = DiffractionPattern::new(vec![], vec![], 1.5);
        assert!(pat.is_empty());
        assert_eq!(pat.len(), 0);
    }

    #[test]
    fn test_diffraction_pattern_len() {
        let pat = DiffractionPattern::new(vec![10.0, 20.0, 30.0], vec![1.0, 2.0, 3.0], 1.5);
        assert_eq!(pat.len(), 3);
        assert!(!pat.is_empty());
    }

    // -- Comprehensive integration test ------------------------------------

    #[test]
    fn test_full_analysis_workflow() {
        // 1. Define crystal
        let a_true = 3.567; // Diamond cubic
        let cubic = CrystalSystem::Cubic { a: a_true };
        let lambda = 1.08; // Typical neutron wavelength

        // 2. Generate allowed FCC reflections
        let hkls = MillerIndex::generate_fcc(4);
        assert!(!hkls.is_empty());

        // 3. Compute d-spacings and 2-theta
        for hkl in &hkls {
            let d = cubic.d_spacing(hkl);
            if let Some(tt) = two_theta_from_d(d, lambda) {
                // Verify roundtrip
                let d_back = d_from_two_theta(tt, lambda);
                assert!(approx_eq(d, d_back, 1e-8));
            }
        }

        // 4. Simulate pattern
        let pat = simulate_pattern(&cubic, &hkls, lambda, (10.0, 160.0), 3000, 0.2);
        assert!(!pat.is_empty());

        // 5. Find peaks and extract d-spacings
        let d_vals = pat.extract_d_spacings(0.5);
        assert!(d_vals.len() >= 2);

        // 6. Check cell volume
        let vol = cubic.cell_volume();
        assert!(approx_eq(vol, a_true.powi(3), EPSILON));
    }
}
