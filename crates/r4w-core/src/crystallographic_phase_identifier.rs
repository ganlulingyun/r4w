//! X-ray diffraction (XRD) pattern analysis and crystallographic phase identification.
//!
//! This module implements signal processing algorithms for analysing powder X-ray
//! diffraction patterns. It covers the full workflow from raw intensity data to
//! quantitative phase analysis:
//!
//! 1. Background estimation and subtraction
//! 2. K-alpha2 stripping (Rachinger method)
//! 3. Peak finding with parabolic interpolation
//! 4. d-spacing calculation via Bragg's law
//! 5. Search/match against reference phase databases
//! 6. Quantitative phase analysis (Reference Intensity Ratio method)
//! 7. Lattice parameter refinement (least-squares)
//! 8. Crystallite size estimation (Scherrer equation)
//! 9. Strain analysis (Williamson-Hall method)
//! 10. Preferred orientation / texture detection
//!
//! # Physics
//!
//! Bragg's law relates the X-ray wavelength, diffraction angle, and lattice
//! plane spacing:
//!
//! ```text
//!   n * lambda = 2 * d * sin(theta)
//! ```
//!
//! For Cu K-alpha radiation (the most common laboratory source):
//! - K-alpha1: 1.5406 A
//! - K-alpha2: 1.5444 A
//! - K-alpha2 / K-alpha1 intensity ratio: 0.5
//!
//! The Scherrer equation relates peak broadening to crystallite size:
//!
//! ```text
//!   D = K * lambda / (beta * cos(theta))
//! ```
//!
//! where K ~ 0.9 is a shape factor, beta is the integral breadth (radians), and
//! D is the volume-weighted mean crystallite size.
//!
//! Williamson-Hall analysis separates size and strain broadening:
//!
//! ```text
//!   beta * cos(theta) = K * lambda / D  +  4 * epsilon * sin(theta)
//! ```
//!
//! # Components
//!
//! | Struct / Function | Purpose |
//! |---|---|
//! | [`XrdConfig`] | X-ray source and measurement parameters |
//! | [`PeakFinder`] | Locate diffraction peaks with parabolic interpolation |
//! | [`BackgroundEstimator`] | Rolling-ball or polynomial background fitting |
//! | [`KAlphaStripper`] | Remove K-alpha2 doublet contribution (Rachinger) |
//! | [`DSpacingCalculator`] | Convert 2-theta positions to d-spacings (Bragg) |
//! | [`PhaseSearchMatch`] | Hanawalt search/match against reference database |
//! | [`QuantitativeAnalyzer`] | RIR-based quantitative phase analysis |
//! | [`LatticeParameterRefiner`] | Least-squares unit cell refinement |
//! | [`CrystalliteSizeEstimator`] | Scherrer equation crystallite size |
//! | [`StrainAnalyzer`] | Williamson-Hall size-strain separation |
//! | [`PreferredOrientationDetector`] | Texture coefficient calculation |
//!
//! # Example
//!
//! ```rust
//! use r4w_core::crystallographic_phase_identifier::{
//!     XrdConfig, XraySource, PeakFinder, DSpacingCalculator,
//!     BackgroundEstimator, BackgroundMethod,
//! };
//!
//! let config = XrdConfig::cu_ka();
//! let two_theta: Vec<f64> = (200..=800).map(|i| i as f64 * 0.1).collect();
//! let intensity: Vec<f64> = two_theta.iter().map(|t| {
//!     100.0 * (-((t - 28.4) / 0.3).powi(2)).exp()
//!     + 60.0 * (-((t - 47.3) / 0.4).powi(2)).exp()
//!     + 5.0
//! }).collect();
//!
//! let bg = BackgroundEstimator::new(BackgroundMethod::RollingBall { radius: 50 });
//! let background = bg.estimate(&two_theta, &intensity);
//! let net: Vec<f64> = intensity.iter().zip(&background).map(|(i, b)| (i - b).max(0.0)).collect();
//!
//! let finder = PeakFinder::new(10.0, 0.05);
//! let peaks = finder.find_peaks(&two_theta, &net);
//! assert!(peaks.len() >= 2);
//!
//! let calc = DSpacingCalculator::new(&config);
//! for p in &peaks {
//!     let d = calc.two_theta_to_d(p.two_theta);
//!     assert!(d > 0.0);
//! }
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Common X-ray source wavelengths.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum XraySource {
    /// Cu K-alpha (most common laboratory source).
    CuKa,
    /// Mo K-alpha (shorter wavelength, used for single-crystal work).
    MoKa,
    /// Co K-alpha (reduces fluorescence from Fe-bearing samples).
    CoKa,
    /// Custom wavelength pair (ka1_angstrom, ka2_angstrom, ka2_ka1_ratio).
    Custom {
        ka1: f64,
        ka2: f64,
        ratio: f64,
    },
}

impl XraySource {
    /// K-alpha1 wavelength in angstroms.
    pub fn ka1(&self) -> f64 {
        match self {
            Self::CuKa => 1.5406,
            Self::MoKa => 0.7107,
            Self::CoKa => 1.7890,
            Self::Custom { ka1, .. } => *ka1,
        }
    }

    /// K-alpha2 wavelength in angstroms.
    pub fn ka2(&self) -> f64 {
        match self {
            Self::CuKa => 1.5444,
            Self::MoKa => 0.7136,
            Self::CoKa => 1.7929,
            Self::Custom { ka2, .. } => *ka2,
        }
    }

    /// Intensity ratio K-alpha2 / K-alpha1.
    pub fn ka2_ratio(&self) -> f64 {
        match self {
            Self::CuKa | Self::MoKa | Self::CoKa => 0.5,
            Self::Custom { ratio, .. } => *ratio,
        }
    }
}

/// XRD measurement configuration.
#[derive(Debug, Clone)]
pub struct XrdConfig {
    /// X-ray source.
    pub source: XraySource,
    /// Divergence slit in degrees.
    pub divergence_slit_deg: f64,
    /// Step size in degrees 2-theta.
    pub step_size_deg: f64,
    /// Start of 2-theta range.
    pub two_theta_start: f64,
    /// End of 2-theta range.
    pub two_theta_end: f64,
}

impl XrdConfig {
    /// Standard Cu K-alpha configuration (5-90 deg, 0.02 deg step).
    pub fn cu_ka() -> Self {
        Self {
            source: XraySource::CuKa,
            divergence_slit_deg: 1.0,
            step_size_deg: 0.02,
            two_theta_start: 5.0,
            two_theta_end: 90.0,
        }
    }

    /// Standard Mo K-alpha configuration (3-60 deg, 0.02 deg step).
    pub fn mo_ka() -> Self {
        Self {
            source: XraySource::MoKa,
            divergence_slit_deg: 0.5,
            step_size_deg: 0.02,
            two_theta_start: 3.0,
            two_theta_end: 60.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Peak representation
// ---------------------------------------------------------------------------

/// A single diffraction peak.
#[derive(Debug, Clone, PartialEq)]
pub struct DiffractionPeak {
    /// Peak position in degrees 2-theta (after parabolic interpolation).
    pub two_theta: f64,
    /// Peak intensity (counts or arbitrary units).
    pub intensity: f64,
    /// Full width at half maximum in degrees 2-theta.
    pub fwhm: f64,
    /// d-spacing in angstroms (filled after conversion).
    pub d_spacing: f64,
    /// Relative intensity as percentage of strongest peak.
    pub relative_intensity: f64,
}

// ---------------------------------------------------------------------------
// Background estimation
// ---------------------------------------------------------------------------

/// Background estimation method.
#[derive(Debug, Clone)]
pub enum BackgroundMethod {
    /// Rolling-ball algorithm with given radius (number of points).
    RollingBall { radius: usize },
    /// Polynomial fit of given degree.
    Polynomial { degree: usize },
}

/// Estimates and removes background from XRD patterns.
pub struct BackgroundEstimator {
    method: BackgroundMethod,
}

impl BackgroundEstimator {
    pub fn new(method: BackgroundMethod) -> Self {
        Self { method }
    }

    /// Estimate the background intensity at each point.
    pub fn estimate(&self, two_theta: &[f64], intensity: &[f64]) -> Vec<f64> {
        assert_eq!(two_theta.len(), intensity.len());
        match &self.method {
            BackgroundMethod::RollingBall { radius } => {
                self.rolling_ball(intensity, *radius)
            }
            BackgroundMethod::Polynomial { degree } => {
                self.polynomial_fit(two_theta, intensity, *degree)
            }
        }
    }

    /// Rolling-ball background: slide a ball of given radius along the bottom
    /// of the pattern. At each point the background is the minimum in a window.
    fn rolling_ball(&self, intensity: &[f64], radius: usize) -> Vec<f64> {
        let n = intensity.len();
        if n == 0 {
            return vec![];
        }
        let r = radius.max(1);
        // First pass: sliding minimum (erosion).
        let mut eroded = vec![0.0f64; n];
        for i in 0..n {
            let lo = if i >= r { i - r } else { 0 };
            let hi = (i + r).min(n - 1);
            let mut min_val = f64::MAX;
            for j in lo..=hi {
                if intensity[j] < min_val {
                    min_val = intensity[j];
                }
            }
            eroded[i] = min_val;
        }
        // Second pass: sliding maximum (dilation) to reconstruct the opening.
        let mut background = vec![0.0f64; n];
        for i in 0..n {
            let lo = if i >= r { i - r } else { 0 };
            let hi = (i + r).min(n - 1);
            let mut max_val = f64::MIN;
            for j in lo..=hi {
                if eroded[j] > max_val {
                    max_val = eroded[j];
                }
            }
            background[i] = max_val;
        }
        background
    }

    /// Least-squares polynomial background fit.
    fn polynomial_fit(&self, x: &[f64], y: &[f64], degree: usize) -> Vec<f64> {
        let n = x.len();
        let d = degree + 1;
        // Build normal equations: (X^T X) c = X^T y
        // X_ij = x_i^j
        let mut ata = vec![0.0f64; d * d];
        let mut atb = vec![0.0f64; d];
        for i in 0..n {
            let mut xi_pow = vec![1.0f64; d];
            for k in 1..d {
                xi_pow[k] = xi_pow[k - 1] * x[i];
            }
            for r in 0..d {
                atb[r] += xi_pow[r] * y[i];
                for c in 0..d {
                    ata[r * d + c] += xi_pow[r] * xi_pow[c];
                }
            }
        }
        // Solve with Gauss elimination.
        let coeffs = gauss_solve(d, &mut ata, &mut atb);
        // Evaluate polynomial.
        let mut bg = vec![0.0f64; n];
        for i in 0..n {
            let mut val = 0.0;
            let mut xi = 1.0;
            for c in &coeffs {
                val += c * xi;
                xi *= x[i];
            }
            bg[i] = val;
        }
        bg
    }
}

// ---------------------------------------------------------------------------
// K-alpha2 stripping (Rachinger algorithm)
// ---------------------------------------------------------------------------

/// Removes K-alpha2 contribution from a diffraction pattern using the
/// Rachinger algorithm.
///
/// The K-alpha2 component at angle theta2 corresponds to a K-alpha1 peak at
/// theta1, where sin(theta2) = (lambda2/lambda1) * sin(theta1). The K-alpha2
/// intensity is `ratio * I_ka1(theta1)`.
pub struct KAlphaStripper {
    ka1: f64,
    ka2: f64,
    ratio: f64,
}

impl KAlphaStripper {
    pub fn new(source: &XraySource) -> Self {
        Self {
            ka1: source.ka1(),
            ka2: source.ka2(),
            ratio: source.ka2_ratio(),
        }
    }

    /// Strip K-alpha2 from an evenly-spaced pattern.
    /// Returns the stripped intensity array.
    pub fn strip(&self, two_theta: &[f64], intensity: &[f64]) -> Vec<f64> {
        assert_eq!(two_theta.len(), intensity.len());
        let n = two_theta.len();
        if n < 2 {
            return intensity.to_vec();
        }
        let step = two_theta[1] - two_theta[0];
        let mut stripped = intensity.to_vec();

        // Process from low angle to high angle.
        for i in 0..n {
            let theta1_rad = two_theta[i].to_radians() / 2.0;
            // Corresponding Ka2 angle for this Ka1 peak.
            let sin_theta2 = (self.ka2 / self.ka1) * theta1_rad.sin();
            if sin_theta2.abs() > 1.0 {
                continue;
            }
            let theta2_rad = sin_theta2.asin();
            let two_theta2 = theta2_rad.to_degrees() * 2.0;
            let delta = two_theta2 - two_theta[i];
            // Find the index where the Ka2 contribution would appear.
            let target_idx_f = i as f64 + delta / step;
            let j = target_idx_f.round() as isize;
            if j >= 0 && (j as usize) < n {
                let j = j as usize;
                stripped[j] -= self.ratio * stripped[i].max(0.0);
                if stripped[j] < 0.0 {
                    stripped[j] = 0.0;
                }
            }
        }
        stripped
    }
}

// ---------------------------------------------------------------------------
// Peak finding
// ---------------------------------------------------------------------------

/// Find diffraction peaks using a second-derivative method with parabolic
/// interpolation for sub-step precision.
pub struct PeakFinder {
    /// Minimum intensity for a peak to be reported.
    min_intensity: f64,
    /// Minimum separation between peaks in degrees 2-theta.
    min_separation: f64,
}

impl PeakFinder {
    pub fn new(min_intensity: f64, min_separation: f64) -> Self {
        Self {
            min_intensity,
            min_separation,
        }
    }

    /// Find peaks in a (two_theta, intensity) pattern.
    /// Returns peaks sorted by decreasing intensity.
    pub fn find_peaks(&self, two_theta: &[f64], intensity: &[f64]) -> Vec<DiffractionPeak> {
        assert_eq!(two_theta.len(), intensity.len());
        let n = two_theta.len();
        if n < 3 {
            return vec![];
        }

        // Second derivative approximation.
        let mut d2 = vec![0.0f64; n];
        for i in 1..n - 1 {
            d2[i] = intensity[i + 1] - 2.0 * intensity[i] + intensity[i - 1];
        }

        // Find local maxima: intensity[i] > neighbours and d2 < 0.
        let mut raw_peaks = Vec::new();
        for i in 1..n - 1 {
            if intensity[i] > intensity[i - 1]
                && intensity[i] > intensity[i + 1]
                && d2[i] < 0.0
                && intensity[i] >= self.min_intensity
            {
                // Parabolic interpolation for sub-step precision.
                let alpha = intensity[i - 1];
                let beta = intensity[i];
                let gamma = intensity[i + 1];
                let denom = alpha - 2.0 * beta + gamma;
                let (interp_pos, interp_int) = if denom.abs() > 1e-15 {
                    let p = 0.5 * (alpha - gamma) / denom;
                    let pos = two_theta[i]
                        + p * (two_theta[i] - two_theta[i.saturating_sub(1)]);
                    let int = beta - 0.25 * (alpha - gamma) * p;
                    (pos, int)
                } else {
                    (two_theta[i], intensity[i])
                };

                // Estimate FWHM: walk left and right to half-max.
                let half_max = interp_int / 2.0;
                let mut left = two_theta[i];
                for j in (0..i).rev() {
                    if intensity[j] <= half_max {
                        // Linear interpolation.
                        let frac = (half_max - intensity[j])
                            / (intensity[j + 1] - intensity[j]).max(1e-15);
                        left = two_theta[j] + frac * (two_theta[j + 1] - two_theta[j]);
                        break;
                    }
                    if j == 0 {
                        left = two_theta[0];
                    }
                }
                let mut right = two_theta[i];
                for j in (i + 1)..n {
                    if intensity[j] <= half_max {
                        let frac = (half_max - intensity[j])
                            / (intensity[j - 1] - intensity[j]).max(1e-15);
                        right = two_theta[j] - frac * (two_theta[j] - two_theta[j - 1]);
                        break;
                    }
                    if j == n - 1 {
                        right = two_theta[n - 1];
                    }
                }
                let fwhm = (right - left).abs();

                raw_peaks.push(DiffractionPeak {
                    two_theta: interp_pos,
                    intensity: interp_int,
                    fwhm,
                    d_spacing: 0.0,
                    relative_intensity: 0.0,
                });
            }
        }

        // Enforce minimum separation: keep strongest peaks.
        raw_peaks.sort_by(|a, b| b.intensity.partial_cmp(&a.intensity).unwrap());
        let mut kept = Vec::new();
        for p in &raw_peaks {
            let too_close = kept.iter().any(|k: &DiffractionPeak| {
                (k.two_theta - p.two_theta).abs() < self.min_separation
            });
            if !too_close {
                kept.push(p.clone());
            }
        }

        // Compute relative intensities.
        let max_int = kept.iter().map(|p| p.intensity).fold(0.0f64, f64::max);
        if max_int > 0.0 {
            for p in &mut kept {
                p.relative_intensity = 100.0 * p.intensity / max_int;
            }
        }

        // Sort by 2-theta ascending.
        kept.sort_by(|a, b| a.two_theta.partial_cmp(&b.two_theta).unwrap());
        kept
    }
}

// ---------------------------------------------------------------------------
// d-spacing calculation
// ---------------------------------------------------------------------------

/// Converts 2-theta peak positions to d-spacings using Bragg's law.
pub struct DSpacingCalculator {
    lambda: f64,
}

impl DSpacingCalculator {
    pub fn new(config: &XrdConfig) -> Self {
        Self {
            lambda: config.source.ka1(),
        }
    }

    /// Create from an explicit wavelength in angstroms.
    pub fn from_wavelength(lambda_angstrom: f64) -> Self {
        Self {
            lambda: lambda_angstrom,
        }
    }

    /// Convert 2-theta (degrees) to d-spacing (angstroms).
    ///
    /// d = lambda / (2 * sin(theta))
    pub fn two_theta_to_d(&self, two_theta_deg: f64) -> f64 {
        let theta = two_theta_deg.to_radians() / 2.0;
        let sin_theta = theta.sin();
        if sin_theta.abs() < 1e-15 {
            return f64::INFINITY;
        }
        self.lambda / (2.0 * sin_theta)
    }

    /// Convert d-spacing (angstroms) to 2-theta (degrees).
    ///
    /// 2*theta = 2 * arcsin(lambda / (2*d))
    pub fn d_to_two_theta(&self, d_angstrom: f64) -> Option<f64> {
        if d_angstrom <= 0.0 {
            return None;
        }
        let arg = self.lambda / (2.0 * d_angstrom);
        if arg.abs() > 1.0 {
            return None;
        }
        Some(2.0 * arg.asin().to_degrees())
    }

    /// Compute d-spacings for a list of peaks (modifies in-place).
    pub fn compute_d_spacings(&self, peaks: &mut [DiffractionPeak]) {
        for p in peaks.iter_mut() {
            p.d_spacing = self.two_theta_to_d(p.two_theta);
        }
    }
}

/// Compute d-spacing for a cubic lattice with Miller indices (h, k, l).
///
/// d = a / sqrt(h^2 + k^2 + l^2)
pub fn d_spacing_cubic(a: f64, h: i32, k: i32, l: i32) -> f64 {
    let hkl_sq = (h * h + k * k + l * l) as f64;
    if hkl_sq == 0.0 {
        return f64::INFINITY;
    }
    a / hkl_sq.sqrt()
}

/// Compute d-spacing for a tetragonal lattice (a = b != c).
///
/// 1/d^2 = (h^2 + k^2)/a^2 + l^2/c^2
pub fn d_spacing_tetragonal(a: f64, c: f64, h: i32, k: i32, l: i32) -> f64 {
    let inv_d2 = ((h * h + k * k) as f64) / (a * a) + ((l * l) as f64) / (c * c);
    if inv_d2 <= 0.0 {
        return f64::INFINITY;
    }
    1.0 / inv_d2.sqrt()
}

/// Compute d-spacing for a hexagonal lattice.
///
/// 1/d^2 = 4/3 * (h^2 + hk + k^2)/a^2 + l^2/c^2
pub fn d_spacing_hexagonal(a: f64, c: f64, h: i32, k: i32, l: i32) -> f64 {
    let hk_term = (h * h + h * k + k * k) as f64;
    let inv_d2 = (4.0 / 3.0) * hk_term / (a * a) + ((l * l) as f64) / (c * c);
    if inv_d2 <= 0.0 {
        return f64::INFINITY;
    }
    1.0 / inv_d2.sqrt()
}

// ---------------------------------------------------------------------------
// Phase search/match
// ---------------------------------------------------------------------------

/// A reference phase entry in the database.
#[derive(Debug, Clone)]
pub struct PhaseEntry {
    /// Phase name (e.g., "Quartz", "Calcite").
    pub name: String,
    /// Chemical formula (e.g., "SiO2", "CaCO3").
    pub formula: String,
    /// Reference d-spacings in angstroms (strongest first).
    pub d_spacings: Vec<f64>,
    /// Corresponding relative intensities (0-100).
    pub intensities: Vec<f64>,
    /// Reference Intensity Ratio (corundum number).
    pub rir: f64,
}

/// Result of a search/match operation.
#[derive(Debug, Clone)]
pub struct MatchResult {
    /// Phase name.
    pub phase_name: String,
    /// Figure of merit (0-1, higher = better match).
    pub figure_of_merit: f64,
    /// Number of matched peaks.
    pub matched_peaks: usize,
    /// Total peaks in reference.
    pub total_ref_peaks: usize,
}

/// Hanawalt search/match: compare measured (d, I) list against reference
/// database entries.
pub struct PhaseSearchMatch {
    /// d-spacing tolerance in angstroms.
    d_tolerance: f64,
    /// Minimum relative intensity to consider.
    min_rel_intensity: f64,
}

impl PhaseSearchMatch {
    pub fn new(d_tolerance: f64, min_rel_intensity: f64) -> Self {
        Self {
            d_tolerance,
            min_rel_intensity,
        }
    }

    /// Search the database for phases matching the measured peaks.
    /// Returns matches sorted by figure of merit (best first).
    pub fn search(
        &self,
        measured_peaks: &[DiffractionPeak],
        database: &[PhaseEntry],
    ) -> Vec<MatchResult> {
        let measured_d: Vec<(f64, f64)> = measured_peaks
            .iter()
            .filter(|p| p.relative_intensity >= self.min_rel_intensity)
            .map(|p| (p.d_spacing, p.relative_intensity))
            .collect();

        let mut results = Vec::new();
        for entry in database {
            let mut matched = 0usize;
            let mut intensity_score = 0.0f64;
            let total = entry.d_spacings.len();

            for (idx, ref_d) in entry.d_spacings.iter().enumerate() {
                let ref_i = if idx < entry.intensities.len() {
                    entry.intensities[idx]
                } else {
                    0.0
                };
                // Find closest measured peak within tolerance.
                let best = measured_d
                    .iter()
                    .filter(|(d, _)| (*d - ref_d).abs() < self.d_tolerance)
                    .min_by(|a, b| {
                        (a.0 - ref_d).abs().partial_cmp(&(b.0 - ref_d).abs()).unwrap()
                    });
                if let Some((_, meas_i)) = best {
                    matched += 1;
                    // Weight by intensity similarity.
                    let i_diff = (meas_i - ref_i).abs() / 100.0;
                    intensity_score += 1.0 - i_diff.min(1.0);
                }
            }

            if matched > 0 {
                let peak_fraction = matched as f64 / total.max(1) as f64;
                let int_fraction = intensity_score / total.max(1) as f64;
                let fom = 0.6 * peak_fraction + 0.4 * int_fraction;
                results.push(MatchResult {
                    phase_name: entry.name.clone(),
                    figure_of_merit: fom,
                    matched_peaks: matched,
                    total_ref_peaks: total,
                });
            }
        }

        results.sort_by(|a, b| b.figure_of_merit.partial_cmp(&a.figure_of_merit).unwrap());
        results
    }
}

// ---------------------------------------------------------------------------
// Quantitative analysis (RIR method)
// ---------------------------------------------------------------------------

/// RIR-based quantitative phase analysis.
///
/// Weight fraction of phase alpha:
///
/// ```text
/// W_alpha = (I_alpha / RIR_alpha) / sum_i(I_i / RIR_i)
/// ```
///
/// where I_alpha is the integrated intensity of the strongest peak.
pub struct QuantitativeAnalyzer;

/// Phase measurement for quantitative analysis.
#[derive(Debug, Clone)]
pub struct PhaseMeasurement {
    /// Phase name.
    pub name: String,
    /// Integrated intensity of the strongest peak.
    pub intensity: f64,
    /// Reference Intensity Ratio (corundum number).
    pub rir: f64,
}

/// Result of quantitative phase analysis.
#[derive(Debug, Clone)]
pub struct PhaseQuantity {
    /// Phase name.
    pub name: String,
    /// Weight fraction (0.0 to 1.0).
    pub weight_fraction: f64,
    /// Weight percent (0 to 100).
    pub weight_percent: f64,
}

impl QuantitativeAnalyzer {
    /// Compute weight fractions using the RIR method.
    pub fn analyze(measurements: &[PhaseMeasurement]) -> Vec<PhaseQuantity> {
        if measurements.is_empty() {
            return vec![];
        }
        // Compute I/RIR for each phase.
        let ratios: Vec<f64> = measurements
            .iter()
            .map(|m| {
                if m.rir > 0.0 {
                    m.intensity / m.rir
                } else {
                    0.0
                }
            })
            .collect();
        let total: f64 = ratios.iter().sum();
        if total <= 0.0 {
            return measurements
                .iter()
                .map(|m| PhaseQuantity {
                    name: m.name.clone(),
                    weight_fraction: 0.0,
                    weight_percent: 0.0,
                })
                .collect();
        }
        measurements
            .iter()
            .zip(&ratios)
            .map(|(m, r)| {
                let wf = r / total;
                PhaseQuantity {
                    name: m.name.clone(),
                    weight_fraction: wf,
                    weight_percent: wf * 100.0,
                }
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Lattice parameter refinement
// ---------------------------------------------------------------------------

/// Crystal system for lattice parameter refinement.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CrystalSystem {
    Cubic,
    Tetragonal,
    Hexagonal,
}

/// A peak with Miller indices assigned for lattice refinement.
#[derive(Debug, Clone)]
pub struct IndexedPeak {
    /// Observed 2-theta in degrees.
    pub two_theta_obs: f64,
    /// Miller indices.
    pub h: i32,
    pub k: i32,
    pub l: i32,
}

/// Refined lattice parameters.
#[derive(Debug, Clone)]
pub struct LatticeParameters {
    /// Lattice parameter a in angstroms.
    pub a: f64,
    /// Lattice parameter c in angstroms (for tetragonal/hexagonal).
    pub c: f64,
    /// RMS residual in degrees 2-theta.
    pub rms_residual: f64,
}

/// Least-squares refinement of unit cell parameters from indexed peak positions.
pub struct LatticeParameterRefiner {
    lambda: f64,
}

impl LatticeParameterRefiner {
    pub fn new(config: &XrdConfig) -> Self {
        Self {
            lambda: config.source.ka1(),
        }
    }

    /// Refine cubic lattice parameter from indexed peaks.
    /// Uses: d = a / sqrt(h^2 + k^2 + l^2), d = lambda / (2*sin(theta)).
    ///
    /// For cubic: 1/d^2 = (h^2 + k^2 + l^2) / a^2
    /// So: sin^2(theta) = lambda^2 * (h^2+k^2+l^2) / (4*a^2)
    ///
    /// Least-squares over a: minimize sum_i [sin^2(theta_i) - lambda^2*Q_i/(4*a^2)]^2
    /// where Q_i = h^2 + k^2 + l^2.
    pub fn refine_cubic(&self, peaks: &[IndexedPeak]) -> LatticeParameters {
        if peaks.is_empty() {
            return LatticeParameters {
                a: 0.0,
                c: 0.0,
                rms_residual: f64::NAN,
            };
        }
        // Linear regression: sin^2(theta) = (lambda^2/(4*a^2)) * Q
        // Let y = sin^2(theta), x = Q, slope = lambda^2/(4*a^2)
        let mut sum_xy = 0.0f64;
        let mut sum_xx = 0.0f64;
        for p in peaks {
            let theta = p.two_theta_obs.to_radians() / 2.0;
            let sin2 = theta.sin().powi(2);
            let q = (p.h * p.h + p.k * p.k + p.l * p.l) as f64;
            sum_xy += q * sin2;
            sum_xx += q * q;
        }
        let slope = sum_xy / sum_xx.max(1e-30);
        let a = self.lambda / (2.0 * slope.sqrt());

        // RMS residual.
        let mut sse = 0.0f64;
        for p in peaks {
            let theta_obs = p.two_theta_obs.to_radians() / 2.0;
            let q = (p.h * p.h + p.k * p.k + p.l * p.l) as f64;
            let d_calc = a / q.sqrt();
            let two_theta_calc = 2.0 * (self.lambda / (2.0 * d_calc)).asin().to_degrees();
            let diff = p.two_theta_obs - two_theta_calc;
            sse += diff * diff;
            let _ = theta_obs; // used above
        }
        let rms = (sse / peaks.len() as f64).sqrt();

        LatticeParameters {
            a,
            c: a, // cubic: a = c
            rms_residual: rms,
        }
    }

    /// Refine tetragonal lattice parameters (a, c) from indexed peaks.
    ///
    /// 1/d^2 = (h^2 + k^2)/a^2 + l^2/c^2
    /// sin^2(theta) = lambda^2/4 * [(h^2+k^2)/a^2 + l^2/c^2]
    ///
    /// Linear in p1 = 1/a^2 and p2 = 1/c^2:
    /// sin^2(theta) = (lambda^2/4) * [p1*(h^2+k^2) + p2*l^2]
    pub fn refine_tetragonal(&self, peaks: &[IndexedPeak]) -> LatticeParameters {
        if peaks.len() < 2 {
            return LatticeParameters {
                a: 0.0,
                c: 0.0,
                rms_residual: f64::NAN,
            };
        }
        let factor = self.lambda * self.lambda / 4.0;
        // y_i = sin^2(theta_i) / factor
        // y_i = p1 * x1_i + p2 * x2_i
        // where x1 = h^2+k^2, x2 = l^2
        let mut a11 = 0.0f64;
        let mut a12 = 0.0f64;
        let mut a22 = 0.0f64;
        let mut b1 = 0.0f64;
        let mut b2 = 0.0f64;
        for p in peaks {
            let theta = p.two_theta_obs.to_radians() / 2.0;
            let y = theta.sin().powi(2) / factor;
            let x1 = (p.h * p.h + p.k * p.k) as f64;
            let x2 = (p.l * p.l) as f64;
            a11 += x1 * x1;
            a12 += x1 * x2;
            a22 += x2 * x2;
            b1 += x1 * y;
            b2 += x2 * y;
        }
        let det = a11 * a22 - a12 * a12;
        if det.abs() < 1e-30 {
            return LatticeParameters {
                a: 0.0,
                c: 0.0,
                rms_residual: f64::NAN,
            };
        }
        let p1 = (a22 * b1 - a12 * b2) / det;
        let p2 = (a11 * b2 - a12 * b1) / det;

        let a = if p1 > 0.0 { 1.0 / p1.sqrt() } else { 0.0 };
        let c = if p2 > 0.0 { 1.0 / p2.sqrt() } else { 0.0 };

        // RMS residual.
        let mut sse = 0.0f64;
        for p in peaks {
            let d_calc = d_spacing_tetragonal(a, c, p.h, p.k, p.l);
            let arg = self.lambda / (2.0 * d_calc);
            if arg.abs() <= 1.0 {
                let two_theta_calc = 2.0 * arg.asin().to_degrees();
                let diff = p.two_theta_obs - two_theta_calc;
                sse += diff * diff;
            }
        }
        let rms = (sse / peaks.len() as f64).sqrt();

        LatticeParameters {
            a,
            c,
            rms_residual: rms,
        }
    }

    /// Refine hexagonal lattice parameters (a, c) from indexed peaks.
    pub fn refine_hexagonal(&self, peaks: &[IndexedPeak]) -> LatticeParameters {
        if peaks.len() < 2 {
            return LatticeParameters {
                a: 0.0,
                c: 0.0,
                rms_residual: f64::NAN,
            };
        }
        let factor = self.lambda * self.lambda / 4.0;
        // 1/d^2 = (4/3)*(h^2+hk+k^2)/a^2 + l^2/c^2
        // sin^2(theta)/factor = p1*(4/3)*(h^2+hk+k^2) + p2*l^2
        // p1 = 1/a^2, p2 = 1/c^2
        let mut a11 = 0.0f64;
        let mut a12 = 0.0f64;
        let mut a22 = 0.0f64;
        let mut b1 = 0.0f64;
        let mut b2 = 0.0f64;
        for p in peaks {
            let theta = p.two_theta_obs.to_radians() / 2.0;
            let y = theta.sin().powi(2) / factor;
            let hk_term = (p.h * p.h + p.h * p.k + p.k * p.k) as f64;
            let x1 = (4.0 / 3.0) * hk_term;
            let x2 = (p.l * p.l) as f64;
            a11 += x1 * x1;
            a12 += x1 * x2;
            a22 += x2 * x2;
            b1 += x1 * y;
            b2 += x2 * y;
        }
        let det = a11 * a22 - a12 * a12;
        if det.abs() < 1e-30 {
            return LatticeParameters {
                a: 0.0,
                c: 0.0,
                rms_residual: f64::NAN,
            };
        }
        let p1 = (a22 * b1 - a12 * b2) / det;
        let p2 = (a11 * b2 - a12 * b1) / det;

        let a = if p1 > 0.0 { 1.0 / p1.sqrt() } else { 0.0 };
        let c = if p2 > 0.0 { 1.0 / p2.sqrt() } else { 0.0 };

        let mut sse = 0.0f64;
        for p in peaks {
            let d_calc = d_spacing_hexagonal(a, c, p.h, p.k, p.l);
            let arg = self.lambda / (2.0 * d_calc);
            if arg.abs() <= 1.0 {
                let two_theta_calc = 2.0 * arg.asin().to_degrees();
                let diff = p.two_theta_obs - two_theta_calc;
                sse += diff * diff;
            }
        }
        let rms = (sse / peaks.len() as f64).sqrt();

        LatticeParameters {
            a,
            c,
            rms_residual: rms,
        }
    }
}

// ---------------------------------------------------------------------------
// Crystallite size (Scherrer equation)
// ---------------------------------------------------------------------------

/// Crystallite size estimation using the Scherrer equation.
///
/// D = K * lambda / (beta * cos(theta))
///
/// K: shape factor (default 0.9)
/// beta: peak broadening at FWHM in **radians** after instrumental correction
/// theta: Bragg angle in radians
pub struct CrystalliteSizeEstimator {
    lambda: f64,
    shape_factor: f64,
}

impl CrystalliteSizeEstimator {
    pub fn new(config: &XrdConfig) -> Self {
        Self {
            lambda: config.source.ka1(),
            shape_factor: 0.9,
        }
    }

    /// Create with custom shape factor.
    pub fn with_shape_factor(config: &XrdConfig, k: f64) -> Self {
        Self {
            lambda: config.source.ka1(),
            shape_factor: k,
        }
    }

    /// Estimate crystallite size from a single peak.
    ///
    /// * `two_theta_deg` - peak position in degrees 2-theta
    /// * `fwhm_deg` - observed FWHM in degrees 2-theta
    /// * `instrument_fwhm_deg` - instrumental broadening FWHM in degrees
    ///
    /// Returns size in angstroms.
    pub fn estimate(
        &self,
        two_theta_deg: f64,
        fwhm_deg: f64,
        instrument_fwhm_deg: f64,
    ) -> f64 {
        // Correct for instrumental broadening (Gaussian subtraction).
        let beta_sq = fwhm_deg * fwhm_deg - instrument_fwhm_deg * instrument_fwhm_deg;
        if beta_sq <= 0.0 {
            return f64::INFINITY; // Broadening is purely instrumental.
        }
        let beta_deg = beta_sq.sqrt();
        let beta_rad = beta_deg.to_radians();
        let theta_rad = two_theta_deg.to_radians() / 2.0;
        let cos_theta = theta_rad.cos();
        if cos_theta.abs() < 1e-15 || beta_rad < 1e-15 {
            return f64::INFINITY;
        }
        self.shape_factor * self.lambda / (beta_rad * cos_theta)
    }

    /// Estimate from multiple peaks and return the average size.
    pub fn estimate_average(
        &self,
        peaks: &[DiffractionPeak],
        instrument_fwhm_deg: f64,
    ) -> f64 {
        let sizes: Vec<f64> = peaks
            .iter()
            .map(|p| self.estimate(p.two_theta, p.fwhm, instrument_fwhm_deg))
            .filter(|s| s.is_finite())
            .collect();
        if sizes.is_empty() {
            return f64::NAN;
        }
        sizes.iter().sum::<f64>() / sizes.len() as f64
    }
}

// ---------------------------------------------------------------------------
// Williamson-Hall strain analysis
// ---------------------------------------------------------------------------

/// Williamson-Hall analysis separates crystallite size and microstrain
/// contributions to peak broadening.
///
/// beta * cos(theta) = K * lambda / D + 4 * epsilon * sin(theta)
///
/// A linear plot of beta*cos(theta) vs 4*sin(theta) yields:
/// - y-intercept = K*lambda/D (size broadening)
/// - slope = epsilon (microstrain)
pub struct StrainAnalyzer {
    lambda: f64,
    shape_factor: f64,
}

/// Result of Williamson-Hall analysis.
#[derive(Debug, Clone)]
pub struct WilliamsonHallResult {
    /// Crystallite size D in angstroms.
    pub crystallite_size: f64,
    /// Microstrain epsilon (dimensionless).
    pub microstrain: f64,
    /// R-squared of the linear fit.
    pub r_squared: f64,
    /// Data points: (4*sin(theta), beta*cos(theta)).
    pub plot_points: Vec<(f64, f64)>,
}

impl StrainAnalyzer {
    pub fn new(config: &XrdConfig) -> Self {
        Self {
            lambda: config.source.ka1(),
            shape_factor: 0.9,
        }
    }

    /// Perform Williamson-Hall analysis on a set of peaks.
    ///
    /// * `peaks` - peaks with measured FWHM
    /// * `instrument_fwhm_deg` - instrumental broadening for correction
    pub fn analyze(
        &self,
        peaks: &[DiffractionPeak],
        instrument_fwhm_deg: f64,
    ) -> WilliamsonHallResult {
        let mut x_vals = Vec::new(); // 4*sin(theta)
        let mut y_vals = Vec::new(); // beta*cos(theta)

        for p in peaks {
            let beta_sq = p.fwhm * p.fwhm - instrument_fwhm_deg * instrument_fwhm_deg;
            if beta_sq <= 0.0 {
                continue;
            }
            let beta_rad = beta_sq.sqrt().to_radians();
            let theta_rad = p.two_theta.to_radians() / 2.0;
            let x = 4.0 * theta_rad.sin();
            let y = beta_rad * theta_rad.cos();
            x_vals.push(x);
            y_vals.push(y);
        }

        if x_vals.len() < 2 {
            return WilliamsonHallResult {
                crystallite_size: f64::NAN,
                microstrain: f64::NAN,
                r_squared: f64::NAN,
                plot_points: vec![],
            };
        }

        // Linear regression: y = intercept + slope * x
        let n = x_vals.len() as f64;
        let sx: f64 = x_vals.iter().sum();
        let sy: f64 = y_vals.iter().sum();
        let sxx: f64 = x_vals.iter().map(|x| x * x).sum();
        let sxy: f64 = x_vals.iter().zip(&y_vals).map(|(x, y)| x * y).sum();

        let denom = n * sxx - sx * sx;
        let (slope, intercept) = if denom.abs() < 1e-30 {
            (0.0, sy / n)
        } else {
            let slope = (n * sxy - sx * sy) / denom;
            let intercept = (sy - slope * sx) / n;
            (slope, intercept)
        };

        // Crystallite size from intercept: intercept = K*lambda/D
        let size = if intercept.abs() > 1e-15 {
            (self.shape_factor * self.lambda / intercept).abs()
        } else {
            f64::INFINITY
        };

        // Microstrain is the slope.
        let strain = slope;

        // R-squared.
        let y_mean = sy / n;
        let ss_tot: f64 = y_vals.iter().map(|y| (y - y_mean).powi(2)).sum();
        let ss_res: f64 = x_vals
            .iter()
            .zip(&y_vals)
            .map(|(x, y)| {
                let y_pred = intercept + slope * x;
                (y - y_pred).powi(2)
            })
            .sum();
        let r_sq = if ss_tot > 1e-30 {
            1.0 - ss_res / ss_tot
        } else {
            f64::NAN
        };

        let plot_points: Vec<(f64, f64)> =
            x_vals.into_iter().zip(y_vals).collect();

        WilliamsonHallResult {
            crystallite_size: size,
            microstrain: strain,
            r_squared: r_sq,
            plot_points,
        }
    }
}

// ---------------------------------------------------------------------------
// Preferred orientation (texture coefficient)
// ---------------------------------------------------------------------------

/// Detects preferred orientation (texture) by computing texture coefficients.
///
/// TC(hkl) = (I_obs/I_ref) / ((1/N) * sum_i(I_obs_i/I_ref_i))
///
/// TC = 1.0 for random orientation; TC > 1.0 indicates preferred orientation
/// along that (hkl) direction.
pub struct PreferredOrientationDetector;

/// Texture coefficient for one reflection.
#[derive(Debug, Clone)]
pub struct TextureCoefficient {
    /// Miller indices label.
    pub hkl: String,
    /// Texture coefficient.
    pub tc: f64,
}

impl PreferredOrientationDetector {
    /// Compute texture coefficients for a set of reflections.
    ///
    /// * `observed` - observed intensities for each (hkl)
    /// * `reference` - reference (random) intensities for each (hkl)
    /// * `labels` - Miller indices labels (e.g., "(111)", "(200)")
    pub fn compute(
        observed: &[f64],
        reference: &[f64],
        labels: &[&str],
    ) -> Vec<TextureCoefficient> {
        assert_eq!(observed.len(), reference.len());
        assert_eq!(observed.len(), labels.len());
        let n = observed.len();
        if n == 0 {
            return vec![];
        }

        // Compute I_obs/I_ref for each reflection.
        let ratios: Vec<f64> = observed
            .iter()
            .zip(reference)
            .map(|(o, r)| if *r > 0.0 { o / r } else { 0.0 })
            .collect();
        let mean_ratio: f64 = ratios.iter().sum::<f64>() / n as f64;
        if mean_ratio < 1e-15 {
            return labels
                .iter()
                .map(|l| TextureCoefficient {
                    hkl: l.to_string(),
                    tc: 0.0,
                })
                .collect();
        }

        labels
            .iter()
            .zip(&ratios)
            .map(|(l, r)| TextureCoefficient {
                hkl: l.to_string(),
                tc: r / mean_ratio,
            })
            .collect()
    }

    /// Compute the degree of preferred orientation sigma.
    /// sigma = sqrt( (1/N) * sum( (TC_i - 1)^2 ) )
    /// sigma = 0 for perfectly random, increases with texture.
    pub fn orientation_index(tcs: &[TextureCoefficient]) -> f64 {
        if tcs.is_empty() {
            return 0.0;
        }
        let n = tcs.len() as f64;
        let sum_sq: f64 = tcs.iter().map(|t| (t.tc - 1.0).powi(2)).sum();
        (sum_sq / n).sqrt()
    }
}

// ---------------------------------------------------------------------------
// Helper: Gauss elimination solver
// ---------------------------------------------------------------------------

fn gauss_solve(n: usize, a: &mut [f64], b: &mut [f64]) -> Vec<f64> {
    // Forward elimination with partial pivoting.
    for col in 0..n {
        // Find pivot.
        let mut max_val = a[col * n + col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            let val = a[row * n + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }
        // Swap rows.
        if max_row != col {
            for k in 0..n {
                a.swap(col * n + k, max_row * n + k);
            }
            b.swap(col, max_row);
        }
        let pivot = a[col * n + col];
        if pivot.abs() < 1e-30 {
            continue;
        }
        // Eliminate below.
        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for k in col..n {
                a[row * n + k] -= factor * a[col * n + k];
            }
            b[row] -= factor * b[col];
        }
    }
    // Back substitution.
    let mut x = vec![0.0f64; n];
    for i in (0..n).rev() {
        let mut sum = b[i];
        for j in (i + 1)..n {
            sum -= a[i * n + j] * x[j];
        }
        let diag = a[i * n + i];
        x[i] = if diag.abs() > 1e-30 { sum / diag } else { 0.0 };
    }
    x
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- XrdConfig ---

    #[test]
    fn test_cu_ka_config() {
        let cfg = XrdConfig::cu_ka();
        assert!((cfg.source.ka1() - 1.5406).abs() < 1e-4);
        assert!((cfg.source.ka2() - 1.5444).abs() < 1e-4);
        assert!((cfg.source.ka2_ratio() - 0.5).abs() < 1e-10);
        assert!((cfg.step_size_deg - 0.02).abs() < 1e-10);
    }

    #[test]
    fn test_mo_ka_config() {
        let cfg = XrdConfig::mo_ka();
        assert!((cfg.source.ka1() - 0.7107).abs() < 1e-4);
    }

    #[test]
    fn test_custom_source() {
        let src = XraySource::Custom {
            ka1: 1.0,
            ka2: 1.1,
            ratio: 0.45,
        };
        assert!((src.ka1() - 1.0).abs() < 1e-10);
        assert!((src.ka2() - 1.1).abs() < 1e-10);
        assert!((src.ka2_ratio() - 0.45).abs() < 1e-10);
    }

    #[test]
    fn test_co_ka_source() {
        let src = XraySource::CoKa;
        assert!((src.ka1() - 1.7890).abs() < 1e-4);
        assert!((src.ka2() - 1.7929).abs() < 1e-4);
        assert!((src.ka2_ratio() - 0.5).abs() < 1e-10);
    }

    // --- Background estimation ---

    #[test]
    fn test_rolling_ball_background() {
        let two_theta: Vec<f64> = (0..100).map(|i| 20.0 + i as f64 * 0.1).collect();
        let intensity: Vec<f64> = two_theta
            .iter()
            .map(|t| {
                100.0 * (-((t - 28.0) / 0.3).powi(2)).exp() + 10.0 // peak + flat bg
            })
            .collect();
        let bg = BackgroundEstimator::new(BackgroundMethod::RollingBall { radius: 20 });
        let background = bg.estimate(&two_theta, &intensity);
        assert_eq!(background.len(), 100);
        // Background should be less than the peak intensity.
        let peak_idx = 80; // (28.0 - 20.0) / 0.1 = 80
        assert!(background[peak_idx] < intensity[peak_idx]);
        // Background far from peak should be close to the flat background level.
        assert!((background[0] - 10.0).abs() < 5.0);
    }

    #[test]
    fn test_polynomial_background() {
        let two_theta: Vec<f64> = (0..50).map(|i| 10.0 + i as f64 * 1.0).collect();
        // Linear background: y = 2 * x + 5.
        let intensity: Vec<f64> = two_theta.iter().map(|t| 2.0 * t + 5.0).collect();
        let bg = BackgroundEstimator::new(BackgroundMethod::Polynomial { degree: 1 });
        let background = bg.estimate(&two_theta, &intensity);
        // Should closely match the linear function.
        for i in 0..50 {
            assert!(
                (background[i] - intensity[i]).abs() < 0.1,
                "at i={}: bg={} vs int={}",
                i,
                background[i],
                intensity[i]
            );
        }
    }

    #[test]
    fn test_background_empty() {
        let bg = BackgroundEstimator::new(BackgroundMethod::RollingBall { radius: 5 });
        let result = bg.estimate(&[], &[]);
        assert!(result.is_empty());
    }

    // --- K-alpha2 stripping ---

    #[test]
    fn test_ka2_stripping_reduces_intensity() {
        let cfg = XrdConfig::cu_ka();
        let stripper = KAlphaStripper::new(&cfg.source);
        let two_theta: Vec<f64> = (0..500).map(|i| 10.0 + i as f64 * 0.1).collect();
        let intensity: Vec<f64> = two_theta
            .iter()
            .map(|t| 1000.0 * (-((t - 28.4) / 0.15).powi(2)).exp())
            .collect();
        let stripped = stripper.strip(&two_theta, &intensity);
        assert_eq!(stripped.len(), two_theta.len());
        // Total intensity should be less after stripping.
        let total_before: f64 = intensity.iter().sum();
        let total_after: f64 = stripped.iter().sum();
        assert!(total_after < total_before);
    }

    #[test]
    fn test_ka2_stripping_short_pattern() {
        let stripper = KAlphaStripper::new(&XraySource::CuKa);
        let result = stripper.strip(&[28.0], &[100.0]);
        assert_eq!(result.len(), 1);
    }

    // --- Peak finding ---

    #[test]
    fn test_find_single_peak() {
        let two_theta: Vec<f64> = (0..200).map(|i| 25.0 + i as f64 * 0.05).collect();
        let intensity: Vec<f64> = two_theta
            .iter()
            .map(|t| 500.0 * (-((t - 28.4) / 0.2).powi(2)).exp())
            .collect();
        let finder = PeakFinder::new(10.0, 0.1);
        let peaks = finder.find_peaks(&two_theta, &intensity);
        assert_eq!(peaks.len(), 1);
        assert!((peaks[0].two_theta - 28.4).abs() < 0.1);
        assert!(peaks[0].intensity > 400.0);
        assert!((peaks[0].relative_intensity - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_find_multiple_peaks() {
        let two_theta: Vec<f64> = (0..1000).map(|i| 20.0 + i as f64 * 0.05).collect();
        let intensity: Vec<f64> = two_theta
            .iter()
            .map(|t| {
                100.0 * (-((t - 28.4) / 0.2).powi(2)).exp()
                    + 60.0 * (-((t - 47.3) / 0.25).powi(2)).exp()
                    + 30.0 * (-((t - 56.1) / 0.3).powi(2)).exp()
            })
            .collect();
        let finder = PeakFinder::new(5.0, 1.0);
        let peaks = finder.find_peaks(&two_theta, &intensity);
        assert!(peaks.len() >= 3, "found {} peaks", peaks.len());
    }

    #[test]
    fn test_peak_finder_min_intensity_filter() {
        let two_theta: Vec<f64> = (0..200).map(|i| 20.0 + i as f64 * 0.1).collect();
        let intensity: Vec<f64> = two_theta
            .iter()
            .map(|t| {
                100.0 * (-((t - 28.0) / 0.2).powi(2)).exp()
                    + 3.0 * (-((t - 35.0) / 0.2).powi(2)).exp() // below threshold
            })
            .collect();
        let finder = PeakFinder::new(5.0, 0.1);
        let peaks = finder.find_peaks(&two_theta, &intensity);
        // Should find the strong peak but not the weak one.
        assert_eq!(peaks.len(), 1);
    }

    #[test]
    fn test_peak_fwhm_estimation() {
        let sigma = 0.2;
        let two_theta: Vec<f64> = (0..400).map(|i| 25.0 + i as f64 * 0.02).collect();
        let intensity: Vec<f64> = two_theta
            .iter()
            .map(|t| 100.0 * (-((t - 28.0) / sigma).powi(2) / 2.0).exp())
            .collect();
        let finder = PeakFinder::new(1.0, 0.01);
        let peaks = finder.find_peaks(&two_theta, &intensity);
        assert_eq!(peaks.len(), 1);
        // FWHM of Gaussian = 2 * sqrt(2*ln(2)) * sigma ~ 2.355 * sigma
        let expected_fwhm = 2.355 * sigma;
        assert!(
            (peaks[0].fwhm - expected_fwhm).abs() < 0.05,
            "fwhm={} expected={}",
            peaks[0].fwhm,
            expected_fwhm
        );
    }

    #[test]
    fn test_peak_finder_empty() {
        let finder = PeakFinder::new(1.0, 0.1);
        assert!(finder.find_peaks(&[], &[]).is_empty());
        assert!(finder.find_peaks(&[1.0, 2.0], &[1.0, 2.0]).is_empty());
    }

    // --- d-spacing ---

    #[test]
    fn test_bragg_law_quartz() {
        // Quartz (101) peak at ~26.65 deg 2-theta with Cu Ka.
        let calc = DSpacingCalculator::from_wavelength(1.5406);
        let d = calc.two_theta_to_d(26.65);
        // Expected d ~ 3.343 A.
        assert!(
            (d - 3.343).abs() < 0.01,
            "d={} expected ~3.343",
            d
        );
    }

    #[test]
    fn test_d_to_two_theta_roundtrip() {
        let calc = DSpacingCalculator::from_wavelength(1.5406);
        let original = 45.0;
        let d = calc.two_theta_to_d(original);
        let recovered = calc.d_to_two_theta(d).unwrap();
        assert!(
            (recovered - original).abs() < 1e-10,
            "recovered={} original={}",
            recovered,
            original
        );
    }

    #[test]
    fn test_d_to_two_theta_impossible() {
        let calc = DSpacingCalculator::from_wavelength(1.5406);
        // d-spacing too small for the wavelength.
        assert!(calc.d_to_two_theta(0.5).is_none());
        // Negative d.
        assert!(calc.d_to_two_theta(-1.0).is_none());
    }

    #[test]
    fn test_compute_d_spacings_on_peaks() {
        let calc = DSpacingCalculator::from_wavelength(1.5406);
        let mut peaks = vec![
            DiffractionPeak {
                two_theta: 26.65,
                intensity: 100.0,
                fwhm: 0.1,
                d_spacing: 0.0,
                relative_intensity: 100.0,
            },
            DiffractionPeak {
                two_theta: 50.0,
                intensity: 50.0,
                fwhm: 0.1,
                d_spacing: 0.0,
                relative_intensity: 50.0,
            },
        ];
        calc.compute_d_spacings(&mut peaks);
        assert!(peaks[0].d_spacing > 3.0);
        assert!(peaks[1].d_spacing > 1.5);
    }

    // --- d-spacing formulas for crystal systems ---

    #[test]
    fn test_d_spacing_cubic_silicon() {
        // Silicon a=5.431 A, (111) d = 5.431/sqrt(3) = 3.135 A.
        let d = d_spacing_cubic(5.431, 1, 1, 1);
        assert!((d - 3.135).abs() < 0.01, "d={}", d);
    }

    #[test]
    fn test_d_spacing_cubic_200() {
        // (200): d = a/sqrt(4) = a/2.
        let a = 4.0;
        let d = d_spacing_cubic(a, 2, 0, 0);
        assert!((d - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_d_spacing_tetragonal() {
        // Tetragonal: a=4.0, c=6.0, (001) => d = c = 6.0.
        let d = d_spacing_tetragonal(4.0, 6.0, 0, 0, 1);
        assert!((d - 6.0).abs() < 1e-10, "d={}", d);
    }

    #[test]
    fn test_d_spacing_hexagonal() {
        // Hexagonal: a=3.0, c=5.0, (001) => d = c = 5.0.
        let d = d_spacing_hexagonal(3.0, 5.0, 0, 0, 1);
        assert!((d - 5.0).abs() < 1e-10, "d={}", d);
    }

    // --- Phase search/match ---

    #[test]
    fn test_search_match_single_phase() {
        let quartz = PhaseEntry {
            name: "Quartz".into(),
            formula: "SiO2".into(),
            d_spacings: vec![3.343, 4.257, 1.818],
            intensities: vec![100.0, 22.0, 17.0],
            rir: 3.4,
        };
        let peaks = vec![
            DiffractionPeak {
                two_theta: 26.65,
                intensity: 100.0,
                fwhm: 0.1,
                d_spacing: 3.340,
                relative_intensity: 100.0,
            },
            DiffractionPeak {
                two_theta: 20.85,
                intensity: 22.0,
                fwhm: 0.1,
                d_spacing: 4.255,
                relative_intensity: 22.0,
            },
        ];
        let matcher = PhaseSearchMatch::new(0.02, 5.0);
        let results = matcher.search(&peaks, &[quartz]);
        assert!(!results.is_empty());
        assert_eq!(results[0].phase_name, "Quartz");
        assert!(results[0].figure_of_merit > 0.3);
        assert!(results[0].matched_peaks >= 2);
    }

    #[test]
    fn test_search_match_no_match() {
        let phase = PhaseEntry {
            name: "NaCl".into(),
            formula: "NaCl".into(),
            d_spacings: vec![2.821, 1.994, 1.628],
            intensities: vec![100.0, 55.0, 15.0],
            rir: 5.0,
        };
        let peaks = vec![DiffractionPeak {
            two_theta: 26.65,
            intensity: 100.0,
            fwhm: 0.1,
            d_spacing: 3.343, // Not matching NaCl.
            relative_intensity: 100.0,
        }];
        let matcher = PhaseSearchMatch::new(0.01, 5.0);
        let results = matcher.search(&peaks, &[phase]);
        assert!(results.is_empty());
    }

    #[test]
    fn test_search_match_ranking() {
        let quartz = PhaseEntry {
            name: "Quartz".into(),
            formula: "SiO2".into(),
            d_spacings: vec![3.343, 4.257],
            intensities: vec![100.0, 22.0],
            rir: 3.4,
        };
        let calcite = PhaseEntry {
            name: "Calcite".into(),
            formula: "CaCO3".into(),
            d_spacings: vec![3.035, 2.285],
            intensities: vec![100.0, 18.0],
            rir: 2.0,
        };
        let peaks = vec![
            DiffractionPeak {
                two_theta: 0.0,
                intensity: 100.0,
                fwhm: 0.1,
                d_spacing: 3.340, // matches quartz
                relative_intensity: 100.0,
            },
            DiffractionPeak {
                two_theta: 0.0,
                intensity: 22.0,
                fwhm: 0.1,
                d_spacing: 4.255, // matches quartz
                relative_intensity: 22.0,
            },
            DiffractionPeak {
                two_theta: 0.0,
                intensity: 80.0,
                fwhm: 0.1,
                d_spacing: 3.033, // matches calcite
                relative_intensity: 80.0,
            },
        ];
        let matcher = PhaseSearchMatch::new(0.02, 5.0);
        let results = matcher.search(&peaks, &[quartz, calcite]);
        assert!(results.len() >= 2);
        // Quartz should rank higher (2/2 matched vs 1/2).
        assert_eq!(results[0].phase_name, "Quartz");
    }

    // --- Quantitative analysis ---

    #[test]
    fn test_rir_quantitative_two_phases() {
        let measurements = vec![
            PhaseMeasurement {
                name: "Quartz".into(),
                intensity: 1000.0,
                rir: 3.4,
            },
            PhaseMeasurement {
                name: "Corundum".into(),
                intensity: 500.0,
                rir: 1.0, // corundum is the reference, RIR = 1.0
            },
        ];
        let result = QuantitativeAnalyzer::analyze(&measurements);
        assert_eq!(result.len(), 2);
        let total: f64 = result.iter().map(|r| r.weight_fraction).sum();
        assert!((total - 1.0).abs() < 1e-10, "total={}", total);
        // Quartz: (1000/3.4) / (1000/3.4 + 500/1.0) = 294.1/794.1 = 0.370
        assert!((result[0].weight_fraction - 0.370).abs() < 0.01);
    }

    #[test]
    fn test_rir_quantitative_single_phase() {
        let measurements = vec![PhaseMeasurement {
            name: "Only".into(),
            intensity: 100.0,
            rir: 2.0,
        }];
        let result = QuantitativeAnalyzer::analyze(&measurements);
        assert_eq!(result.len(), 1);
        assert!((result[0].weight_fraction - 1.0).abs() < 1e-10);
        assert!((result[0].weight_percent - 100.0).abs() < 1e-8);
    }

    #[test]
    fn test_rir_quantitative_empty() {
        let result = QuantitativeAnalyzer::analyze(&[]);
        assert!(result.is_empty());
    }

    // --- Lattice parameter refinement ---

    #[test]
    fn test_cubic_refinement_silicon() {
        // Silicon: a=5.431 A, Cu Ka. Generate peaks for (111), (220), (311).
        let lambda = 1.5406;
        let a_true = 5.431;
        let reflections = [(1, 1, 1), (2, 2, 0), (3, 1, 1)];
        let peaks: Vec<IndexedPeak> = reflections
            .iter()
            .map(|&(h, k, l)| {
                let d = d_spacing_cubic(a_true, h, k, l);
                let two_theta = 2.0 * (lambda / (2.0 * d)).asin().to_degrees();
                IndexedPeak {
                    two_theta_obs: two_theta,
                    h,
                    k,
                    l,
                }
            })
            .collect();
        let cfg = XrdConfig::cu_ka();
        let refiner = LatticeParameterRefiner::new(&cfg);
        let result = refiner.refine_cubic(&peaks);
        assert!(
            (result.a - a_true).abs() < 0.001,
            "a={} expected {}",
            result.a,
            a_true
        );
        assert!(result.rms_residual < 0.01);
    }

    #[test]
    fn test_tetragonal_refinement() {
        let lambda = 1.5406;
        let a_true = 4.594;
        let c_true = 2.959;
        let reflections = [(1, 1, 0), (1, 0, 1), (2, 0, 0), (1, 1, 1), (2, 1, 0)];
        let peaks: Vec<IndexedPeak> = reflections
            .iter()
            .map(|&(h, k, l)| {
                let d = d_spacing_tetragonal(a_true, c_true, h, k, l);
                let two_theta = 2.0 * (lambda / (2.0 * d)).asin().to_degrees();
                IndexedPeak {
                    two_theta_obs: two_theta,
                    h,
                    k,
                    l,
                }
            })
            .collect();
        let cfg = XrdConfig::cu_ka();
        let refiner = LatticeParameterRefiner::new(&cfg);
        let result = refiner.refine_tetragonal(&peaks);
        assert!(
            (result.a - a_true).abs() < 0.01,
            "a={} expected {}",
            result.a,
            a_true
        );
        assert!(
            (result.c - c_true).abs() < 0.01,
            "c={} expected {}",
            result.c,
            c_true
        );
    }

    #[test]
    fn test_hexagonal_refinement() {
        let lambda = 1.5406;
        let a_true = 4.913;
        let c_true = 5.405;
        let reflections = [(1, 0, 0), (0, 0, 1), (1, 0, 1), (1, 1, 0), (1, 1, 1)];
        let peaks: Vec<IndexedPeak> = reflections
            .iter()
            .map(|&(h, k, l)| {
                let d = d_spacing_hexagonal(a_true, c_true, h, k, l);
                let two_theta = 2.0 * (lambda / (2.0 * d)).asin().to_degrees();
                IndexedPeak {
                    two_theta_obs: two_theta,
                    h,
                    k,
                    l,
                }
            })
            .collect();
        let cfg = XrdConfig::cu_ka();
        let refiner = LatticeParameterRefiner::new(&cfg);
        let result = refiner.refine_hexagonal(&peaks);
        assert!(
            (result.a - a_true).abs() < 0.01,
            "a={} expected {}",
            result.a,
            a_true
        );
        assert!(
            (result.c - c_true).abs() < 0.01,
            "c={} expected {}",
            result.c,
            c_true
        );
    }

    // --- Crystallite size ---

    #[test]
    fn test_scherrer_size_estimation() {
        let cfg = XrdConfig::cu_ka();
        let estimator = CrystalliteSizeEstimator::new(&cfg);
        // 10 nm crystallite: FWHM ~ K*lambda/(D*cos(theta)) in radians.
        let d_angstrom = 100.0; // 10 nm = 100 A.
        let two_theta: f64 = 28.4;
        let theta_rad = two_theta.to_radians() / 2.0;
        let fwhm_rad: f64 = 0.9 * 1.5406 / (d_angstrom * theta_rad.cos());
        let fwhm_deg = fwhm_rad.to_degrees();
        let size = estimator.estimate(two_theta, fwhm_deg, 0.0);
        assert!(
            (size - d_angstrom).abs() < 5.0,
            "size={} expected ~{}",
            size,
            d_angstrom
        );
    }

    #[test]
    fn test_scherrer_with_instrumental_broadening() {
        let cfg = XrdConfig::cu_ka();
        let estimator = CrystalliteSizeEstimator::new(&cfg);
        // If observed FWHM equals instrumental FWHM, size should be infinite.
        let size = estimator.estimate(30.0, 0.1, 0.1);
        assert!(size.is_infinite());
    }

    #[test]
    fn test_scherrer_custom_shape_factor() {
        let cfg = XrdConfig::cu_ka();
        let est1 = CrystalliteSizeEstimator::new(&cfg);
        let est2 = CrystalliteSizeEstimator::with_shape_factor(&cfg, 1.0);
        let s1 = est1.estimate(30.0, 0.5, 0.0);
        let s2 = est2.estimate(30.0, 0.5, 0.0);
        // s2/s1 should equal 1.0/0.9.
        assert!((s2 / s1 - 1.0 / 0.9).abs() < 0.01);
    }

    #[test]
    fn test_scherrer_average() {
        let cfg = XrdConfig::cu_ka();
        let estimator = CrystalliteSizeEstimator::new(&cfg);
        let peaks = vec![
            DiffractionPeak {
                two_theta: 28.4,
                intensity: 100.0,
                fwhm: 0.5,
                d_spacing: 3.14,
                relative_intensity: 100.0,
            },
            DiffractionPeak {
                two_theta: 47.3,
                intensity: 60.0,
                fwhm: 0.6,
                d_spacing: 1.92,
                relative_intensity: 60.0,
            },
        ];
        let avg = estimator.estimate_average(&peaks, 0.05);
        assert!(avg.is_finite());
        assert!(avg > 0.0);
    }

    // --- Williamson-Hall ---

    #[test]
    fn test_williamson_hall_pure_size() {
        // Create peaks with no strain broadening (constant beta*cos(theta)).
        let cfg = XrdConfig::cu_ka();
        let analyzer = StrainAnalyzer::new(&cfg);
        let d_angstrom = 200.0; // 20 nm
        let lambda = 1.5406;
        let k = 0.9;
        let angles = [20.0, 30.0, 40.0, 50.0, 60.0];
        let peaks: Vec<DiffractionPeak> = angles
            .iter()
            .map(|&tt: &f64| {
                let theta = tt.to_radians() / 2.0;
                // beta*cos(theta) = K*lambda/D => beta = K*lambda/(D*cos(theta))
                let beta_rad: f64 = k * lambda / (d_angstrom * theta.cos());
                let fwhm_deg = beta_rad.to_degrees();
                DiffractionPeak {
                    two_theta: tt,
                    intensity: 100.0,
                    fwhm: fwhm_deg,
                    d_spacing: 0.0,
                    relative_intensity: 100.0,
                }
            })
            .collect();
        let result = analyzer.analyze(&peaks, 0.0);
        assert!(
            (result.crystallite_size - d_angstrom).abs() < 10.0,
            "size={} expected ~{}",
            result.crystallite_size,
            d_angstrom
        );
        assert!(
            result.microstrain.abs() < 0.001,
            "strain={} expected ~0",
            result.microstrain
        );
    }

    #[test]
    fn test_williamson_hall_with_strain() {
        let cfg = XrdConfig::cu_ka();
        let analyzer = StrainAnalyzer::new(&cfg);
        let lambda = 1.5406;
        let k = 0.9;
        let d_angstrom = 200.0;
        let epsilon = 0.002; // microstrain
        let angles = [25.0, 35.0, 45.0, 55.0, 65.0, 75.0];
        let peaks: Vec<DiffractionPeak> = angles
            .iter()
            .map(|&tt: &f64| {
                let theta = tt.to_radians() / 2.0;
                // beta*cos(theta) = K*lambda/D + 4*epsilon*sin(theta)
                let bc = k * lambda / d_angstrom + 4.0 * epsilon * theta.sin();
                let beta_rad: f64 = bc / theta.cos();
                let fwhm_deg = beta_rad.to_degrees();
                DiffractionPeak {
                    two_theta: tt,
                    intensity: 100.0,
                    fwhm: fwhm_deg,
                    d_spacing: 0.0,
                    relative_intensity: 100.0,
                }
            })
            .collect();
        let result = analyzer.analyze(&peaks, 0.0);
        assert!(
            (result.crystallite_size - d_angstrom).abs() < 20.0,
            "size={}",
            result.crystallite_size
        );
        assert!(
            (result.microstrain - epsilon).abs() < 0.0005,
            "strain={}",
            result.microstrain
        );
        assert!(result.r_squared > 0.99);
        assert_eq!(result.plot_points.len(), 6);
    }

    // --- Preferred orientation ---

    #[test]
    fn test_texture_coefficient_random() {
        // Random orientation: observed matches reference.
        let obs = [100.0, 50.0, 30.0, 20.0];
        let ref_ = [100.0, 50.0, 30.0, 20.0];
        let labels = ["(111)", "(200)", "(220)", "(311)"];
        let tcs = PreferredOrientationDetector::compute(&obs, &ref_, &labels);
        for tc in &tcs {
            assert!(
                (tc.tc - 1.0).abs() < 1e-10,
                "TC({})={} expected 1.0",
                tc.hkl,
                tc.tc
            );
        }
        let sigma = PreferredOrientationDetector::orientation_index(&tcs);
        assert!(sigma < 1e-10);
    }

    #[test]
    fn test_texture_coefficient_preferred() {
        // (111) is much stronger than random.
        let obs = [200.0, 50.0, 30.0, 20.0];
        let ref_ = [100.0, 50.0, 30.0, 20.0];
        let labels = ["(111)", "(200)", "(220)", "(311)"];
        let tcs = PreferredOrientationDetector::compute(&obs, &ref_, &labels);
        assert!(tcs[0].tc > 1.0, "TC(111)={}", tcs[0].tc);
        let sigma = PreferredOrientationDetector::orientation_index(&tcs);
        assert!(sigma > 0.1);
    }

    #[test]
    fn test_texture_coefficient_empty() {
        let result = PreferredOrientationDetector::compute(&[], &[], &[]);
        assert!(result.is_empty());
    }

    // --- Gauss solver ---

    #[test]
    fn test_gauss_solve_2x2() {
        // x + 2y = 5, 3x + y = 5 => x=1, y=2
        let mut a = vec![1.0, 2.0, 3.0, 1.0];
        let mut b = vec![5.0, 5.0];
        let x = gauss_solve(2, &mut a, &mut b);
        assert!((x[0] - 1.0).abs() < 1e-10);
        assert!((x[1] - 2.0).abs() < 1e-10);
    }

    // --- Integration / workflow ---

    #[test]
    fn test_full_workflow() {
        // Simulate a pattern with two quartz peaks, find them, compute d-spacings.
        let two_theta: Vec<f64> = (0..1500).map(|i| 10.0 + i as f64 * 0.05).collect();
        let intensity: Vec<f64> = two_theta
            .iter()
            .map(|t| {
                1000.0 * (-((t - 26.65) / 0.15).powi(2)).exp()
                    + 220.0 * (-((t - 20.85) / 0.15).powi(2)).exp()
                    + 15.0 // flat background
            })
            .collect();

        // 1. Background.
        let bg_est = BackgroundEstimator::new(BackgroundMethod::RollingBall { radius: 30 });
        let bg = bg_est.estimate(&two_theta, &intensity);
        let net: Vec<f64> = intensity
            .iter()
            .zip(&bg)
            .map(|(i, b)| (i - b).max(0.0))
            .collect();

        // 2. Find peaks.
        let finder = PeakFinder::new(20.0, 1.0);
        let mut peaks = finder.find_peaks(&two_theta, &net);
        assert!(peaks.len() >= 2, "found {} peaks", peaks.len());

        // 3. d-spacings.
        let cfg = XrdConfig::cu_ka();
        let calc = DSpacingCalculator::new(&cfg);
        calc.compute_d_spacings(&mut peaks);
        // Verify quartz d-spacings.
        let d_values: Vec<f64> = peaks.iter().map(|p| p.d_spacing).collect();
        let has_3_34 = d_values.iter().any(|d| (*d - 3.34).abs() < 0.05);
        let has_4_26 = d_values.iter().any(|d| (*d - 4.26).abs() < 0.05);
        assert!(has_3_34, "missing d~3.34: {:?}", d_values);
        assert!(has_4_26, "missing d~4.26: {:?}", d_values);

        // 4. Search/match.
        let quartz = PhaseEntry {
            name: "Quartz".into(),
            formula: "SiO2".into(),
            d_spacings: vec![3.343, 4.257],
            intensities: vec![100.0, 22.0],
            rir: 3.4,
        };
        let matcher = PhaseSearchMatch::new(0.05, 5.0);
        let matches = matcher.search(&peaks, &[quartz]);
        assert!(!matches.is_empty());
        assert_eq!(matches[0].phase_name, "Quartz");
    }

    #[test]
    fn test_d_spacing_zero_angle() {
        let calc = DSpacingCalculator::from_wavelength(1.5406);
        let d = calc.two_theta_to_d(0.0);
        assert!(d.is_infinite());
    }

    #[test]
    fn test_lattice_refinement_empty() {
        let cfg = XrdConfig::cu_ka();
        let refiner = LatticeParameterRefiner::new(&cfg);
        let result = refiner.refine_cubic(&[]);
        assert!((result.a - 0.0).abs() < 1e-10);
    }
}
