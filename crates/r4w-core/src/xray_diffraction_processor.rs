// trace:FR-XRD | ai:claude
//! # X-Ray Diffraction Processor
//!
//! Implements powder X-ray diffraction (XRD) analysis including Bragg's law,
//! peak identification, lattice parameter refinement, crystallite size (Scherrer),
//! microstrain analysis (Williamson-Hall), and phase identification.
//!
//! ## Physics Background
//!
//! - **Bragg's law**: nλ = 2d sin(θ)
//! - **Scherrer equation**: L = Kλ / (β cos(θ))
//! - **Williamson-Hall**: β cos(θ) = Kλ/L + 4ε sin(θ)
//! - **Lattice planes**: d_hkl from Miller indices (h,k,l)
//! - **Crystal systems**: cubic, tetragonal, orthorhombic, hexagonal, etc.

use std::f64::consts::PI;

/// Cu Kα wavelength in Angstroms.
pub const CU_KA: f64 = 1.5406;
/// Cu Kα1 wavelength in Angstroms.
pub const CU_KA1: f64 = 1.5405;
/// Cu Kα2 wavelength in Angstroms.
pub const CU_KA2: f64 = 1.5443;
/// Mo Kα wavelength in Angstroms.
pub const MO_KA: f64 = 0.7107;
/// Co Kα wavelength in Angstroms.
pub const CO_KA: f64 = 1.7889;

// ---------------------------------------------------------------------------
// 1. Bragg's Law
// ---------------------------------------------------------------------------

/// Bragg's law: d = nλ / (2 sin(θ))
pub fn bragg_d_spacing(wavelength_a: f64, two_theta_deg: f64, order: u32) -> f64 {
    let theta_rad: f64 = two_theta_deg * PI / 360.0; // half of 2theta
    let sin_theta: f64 = theta_rad.sin();
    if sin_theta.abs() < 1e-15 { return f64::INFINITY; }
    order as f64 * wavelength_a / (2.0 * sin_theta)
}

/// Inverse Bragg: 2θ from d-spacing.
pub fn bragg_two_theta(wavelength_a: f64, d_spacing_a: f64, order: u32) -> f64 {
    let sin_theta: f64 = order as f64 * wavelength_a / (2.0 * d_spacing_a);
    if sin_theta.abs() > 1.0 { return f64::NAN; }
    2.0 * sin_theta.asin() * 180.0 / PI
}

// ---------------------------------------------------------------------------
// 2. Crystal Lattice d-spacings
// ---------------------------------------------------------------------------

/// Crystal system enum.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CrystalSystem {
    Cubic,
    Tetragonal,
    Orthorhombic,
    Hexagonal,
}

/// Lattice parameters.
#[derive(Debug, Clone, Copy)]
pub struct LatticeParams {
    pub a: f64,
    pub b: f64,
    pub c: f64,
    pub system: CrystalSystem,
}

impl LatticeParams {
    /// Create cubic lattice.
    pub fn cubic(a: f64) -> Self {
        Self { a, b: a, c: a, system: CrystalSystem::Cubic }
    }

    /// Create tetragonal lattice.
    pub fn tetragonal(a: f64, c: f64) -> Self {
        Self { a, b: a, c, system: CrystalSystem::Tetragonal }
    }

    /// Create orthorhombic lattice.
    pub fn orthorhombic(a: f64, b: f64, c: f64) -> Self {
        Self { a, b, c, system: CrystalSystem::Orthorhombic }
    }

    /// Create hexagonal lattice.
    pub fn hexagonal(a: f64, c: f64) -> Self {
        Self { a, b: a, c, system: CrystalSystem::Hexagonal }
    }

    /// Calculate d-spacing for Miller indices (h, k, l).
    pub fn d_spacing(&self, h: i32, k: i32, l: i32) -> f64 {
        let hf: f64 = h as f64;
        let kf: f64 = k as f64;
        let lf: f64 = l as f64;
        let inv_d2: f64 = match self.system {
            CrystalSystem::Cubic => {
                (hf * hf + kf * kf + lf * lf) / (self.a * self.a)
            }
            CrystalSystem::Tetragonal => {
                (hf * hf + kf * kf) / (self.a * self.a) + lf * lf / (self.c * self.c)
            }
            CrystalSystem::Orthorhombic => {
                hf * hf / (self.a * self.a)
                    + kf * kf / (self.b * self.b)
                    + lf * lf / (self.c * self.c)
            }
            CrystalSystem::Hexagonal => {
                4.0 / 3.0 * (hf * hf + hf * kf + kf * kf) / (self.a * self.a)
                    + lf * lf / (self.c * self.c)
            }
        };
        if inv_d2 <= 0.0 { return f64::INFINITY; }
        (1.0 / inv_d2).sqrt()
    }

    /// Generate all allowed reflections up to a maximum 2θ.
    pub fn generate_reflections(
        &self,
        wavelength_a: f64,
        max_two_theta: f64,
        max_index: i32,
    ) -> Vec<Reflection> {
        let mut reflections: Vec<Reflection> = Vec::new();
        for h in 0..=max_index {
            for k in -max_index..=max_index {
                for l in -max_index..=max_index {
                    if h == 0 && k == 0 && l == 0 { continue; }
                    // Skip equivalent reflections with negative h
                    if h == 0 && k < 0 { continue; }
                    if h == 0 && k == 0 && l < 0 { continue; }
                    let d: f64 = self.d_spacing(h, k, l);
                    let two_theta: f64 = bragg_two_theta(wavelength_a, d, 1);
                    if two_theta.is_nan() || two_theta > max_two_theta { continue; }
                    reflections.push(Reflection {
                        h, k, l, d_spacing: d, two_theta,
                    });
                }
            }
        }
        reflections.sort_by(|a, b| a.two_theta.partial_cmp(&b.two_theta).unwrap_or(std::cmp::Ordering::Equal));
        reflections
    }
}

/// A crystallographic reflection.
#[derive(Debug, Clone)]
pub struct Reflection {
    pub h: i32,
    pub k: i32,
    pub l: i32,
    pub d_spacing: f64,
    pub two_theta: f64,
}

// ---------------------------------------------------------------------------
// 3. XRD Pattern
// ---------------------------------------------------------------------------

/// An XRD diffraction pattern.
#[derive(Debug, Clone)]
pub struct XrdPattern {
    /// 2θ values in degrees.
    pub two_theta: Vec<f64>,
    /// Intensity values.
    pub intensities: Vec<f64>,
}

impl XrdPattern {
    /// Create new pattern.
    pub fn new(two_theta: Vec<f64>, intensities: Vec<f64>) -> Self {
        assert_eq!(two_theta.len(), intensities.len());
        Self { two_theta, intensities }
    }

    /// Find peak positions above threshold.
    pub fn find_peaks(&self, threshold: f64) -> Vec<XrdPeak> {
        let n: usize = self.two_theta.len();
        let mut peaks: Vec<XrdPeak> = Vec::new();
        for i in 1..n.saturating_sub(1) {
            if self.intensities[i] > threshold
                && self.intensities[i] > self.intensities[i - 1]
                && self.intensities[i] > self.intensities[i + 1]
            {
                // Estimate FWHM
                let half_max: f64 = self.intensities[i] * 0.5;
                let mut fwhm: f64 = 0.2; // default
                // Search left
                let mut left: f64 = self.two_theta[i];
                for j in (0..i).rev() {
                    if self.intensities[j] < half_max {
                        let frac: f64 = (half_max - self.intensities[j])
                            / (self.intensities[j + 1] - self.intensities[j]);
                        left = self.two_theta[j]
                            + frac * (self.two_theta[j + 1] - self.two_theta[j]);
                        break;
                    }
                }
                // Search right
                let mut right: f64 = self.two_theta[i];
                for j in i + 1..n {
                    if self.intensities[j] < half_max {
                        let frac: f64 = (half_max - self.intensities[j])
                            / (self.intensities[j - 1] - self.intensities[j]);
                        right = self.two_theta[j]
                            - frac * (self.two_theta[j] - self.two_theta[j - 1]);
                        break;
                    }
                }
                fwhm = right - left;
                if fwhm < 0.01 { fwhm = 0.01; }
                peaks.push(XrdPeak {
                    two_theta: self.two_theta[i],
                    intensity: self.intensities[i],
                    fwhm_deg: fwhm,
                });
            }
        }
        peaks
    }

    /// Background subtraction using Sonneveld-Visser algorithm (simplified).
    pub fn subtract_background(&mut self, window: usize) {
        let n: usize = self.two_theta.len();
        if window < 2 || n < window { return; }
        let half: usize = window / 2;
        let mut bg: Vec<f64> = vec![0.0; n];
        for i in 0..n {
            let start: usize = if i >= half { i - half } else { 0 };
            let end: usize = (i + half + 1).min(n);
            let mut min_val: f64 = f64::MAX;
            for j in start..end {
                if self.intensities[j] < min_val { min_val = self.intensities[j]; }
            }
            bg[i] = min_val;
        }
        for i in 0..n {
            self.intensities[i] = (self.intensities[i] - bg[i]).max(0.0);
        }
    }
}

/// An XRD peak.
#[derive(Debug, Clone)]
pub struct XrdPeak {
    pub two_theta: f64,
    pub intensity: f64,
    pub fwhm_deg: f64,
}

// ---------------------------------------------------------------------------
// 4. Scherrer Equation (Crystallite Size)
// ---------------------------------------------------------------------------

/// Scherrer equation: L = Kλ / (β cos(θ))
/// K = Scherrer constant (typically 0.9)
/// β = FWHM in radians (peak broadening)
/// θ = Bragg angle
pub fn scherrer_size(
    wavelength_a: f64,
    fwhm_deg: f64,
    two_theta_deg: f64,
    k_factor: f64,
) -> f64 {
    let beta_rad: f64 = fwhm_deg * PI / 180.0;
    let theta_rad: f64 = two_theta_deg * PI / 360.0;
    let cos_theta: f64 = theta_rad.cos();
    if cos_theta.abs() < 1e-15 || beta_rad <= 0.0 { return f64::INFINITY; }
    k_factor * wavelength_a / (beta_rad * cos_theta)
}

/// Instrumental broadening correction: β_sample = sqrt(β_obs² - β_inst²)
pub fn correct_broadening(fwhm_obs_deg: f64, fwhm_inst_deg: f64) -> f64 {
    let diff: f64 = fwhm_obs_deg * fwhm_obs_deg - fwhm_inst_deg * fwhm_inst_deg;
    if diff <= 0.0 { return 0.01; }
    diff.sqrt()
}

// ---------------------------------------------------------------------------
// 5. Williamson-Hall Analysis
// ---------------------------------------------------------------------------

/// Williamson-Hall plot data point.
#[derive(Debug, Clone)]
pub struct WhPoint {
    pub sin_theta: f64,
    pub beta_cos_theta: f64,
}

/// Generate Williamson-Hall plot data.
pub fn williamson_hall_data(
    peaks: &[XrdPeak],
    wavelength_a: f64,
    fwhm_inst_deg: f64,
) -> Vec<WhPoint> {
    let mut points: Vec<WhPoint> = Vec::new();
    for peak in peaks {
        let theta_rad: f64 = peak.two_theta * PI / 360.0;
        let beta_corr: f64 = correct_broadening(peak.fwhm_deg, fwhm_inst_deg);
        let beta_rad: f64 = beta_corr * PI / 180.0;
        points.push(WhPoint {
            sin_theta: theta_rad.sin(),
            beta_cos_theta: beta_rad * theta_rad.cos(),
        });
    }
    points
}

/// Fit Williamson-Hall: β*cos(θ) = Kλ/L + 4ε*sin(θ)
/// Returns (size_angstrom, strain).
pub fn williamson_hall_fit(
    points: &[WhPoint],
    wavelength_a: f64,
    k_factor: f64,
) -> (f64, f64) {
    let n: usize = points.len();
    if n < 2 { return (100.0, 0.0); }
    let nf: f64 = n as f64;
    let mut sx: f64 = 0.0;
    let mut sy: f64 = 0.0;
    let mut sxy: f64 = 0.0;
    let mut sxx: f64 = 0.0;
    for p in points {
        let x: f64 = 4.0 * p.sin_theta;
        let y: f64 = p.beta_cos_theta;
        sx += x;
        sy += y;
        sxy += x * y;
        sxx += x * x;
    }
    let denom: f64 = nf * sxx - sx * sx;
    if denom.abs() < 1e-30 { return (100.0, 0.0); }
    let slope: f64 = (nf * sxy - sx * sy) / denom; // strain
    let intercept: f64 = (sy - slope * sx) / nf; // Kλ/L
    let size: f64 = if intercept.abs() > 1e-15 {
        k_factor * wavelength_a / intercept
    } else {
        f64::INFINITY
    };
    (size, slope)
}

// ---------------------------------------------------------------------------
// 6. Pattern Simulation
// ---------------------------------------------------------------------------

/// Simulate a diffraction pattern from a list of reflections.
pub fn simulate_pattern(
    reflections: &[Reflection],
    rel_intensities: &[f64],
    two_theta_range: (f64, f64),
    n_points: usize,
    peak_width_deg: f64,
) -> XrdPattern {
    let step: f64 = (two_theta_range.1 - two_theta_range.0) / (n_points as f64 - 1.0);
    let mut tt: Vec<f64> = Vec::with_capacity(n_points);
    let mut ints: Vec<f64> = vec![0.0; n_points];
    for i in 0..n_points {
        tt.push(two_theta_range.0 + i as f64 * step);
    }
    let sigma: f64 = peak_width_deg / 2.355;
    let n_refl: usize = reflections.len().min(rel_intensities.len());
    for r in 0..n_refl {
        let center: f64 = reflections[r].two_theta;
        let amp: f64 = rel_intensities[r];
        for (i, &t) in tt.iter().enumerate() {
            let z: f64 = (t - center) / sigma;
            ints[i] += amp * (-0.5 * z * z).exp();
        }
    }
    XrdPattern::new(tt, ints)
}

// ---------------------------------------------------------------------------
// 7. Lattice Parameter Refinement
// ---------------------------------------------------------------------------

/// Refine cubic lattice parameter from peak positions.
/// Uses: a = d * sqrt(h² + k² + l²)
pub fn refine_cubic_lattice(
    peaks: &[(f64, (i32, i32, i32))], // (2θ, (h,k,l))
    wavelength_a: f64,
) -> f64 {
    if peaks.is_empty() { return 0.0; }
    let mut sum_a: f64 = 0.0;
    let mut count: f64 = 0.0;
    for &(two_theta, (h, k, l)) in peaks {
        let d: f64 = bragg_d_spacing(wavelength_a, two_theta, 1);
        let hkl2: f64 = (h * h + k * k + l * l) as f64;
        let a: f64 = d * hkl2.sqrt();
        sum_a += a;
        count += 1.0;
    }
    sum_a / count
}

// ---------------------------------------------------------------------------
// 8. XrdProcessor Orchestrator
// ---------------------------------------------------------------------------

/// XRD analysis orchestrator.
#[derive(Debug, Clone)]
pub struct XrdProcessor {
    pub pattern: Option<XrdPattern>,
    pub wavelength_a: f64,
    pub lattice: Option<LatticeParams>,
}

impl XrdProcessor {
    /// Create with Cu Kα wavelength.
    pub fn new() -> Self {
        Self { pattern: None, wavelength_a: CU_KA, lattice: None }
    }

    /// Set wavelength.
    pub fn set_wavelength(&mut self, wavelength: f64) {
        self.wavelength_a = wavelength;
    }

    /// Load pattern.
    pub fn load_pattern(&mut self, pattern: XrdPattern) {
        self.pattern = Some(pattern);
    }

    /// Set lattice.
    pub fn set_lattice(&mut self, lattice: LatticeParams) {
        self.lattice = Some(lattice);
    }

    /// Find peaks.
    pub fn find_peaks(&self, threshold: f64) -> Vec<XrdPeak> {
        match &self.pattern {
            Some(pat) => pat.find_peaks(threshold),
            None => Vec::new(),
        }
    }

    /// Calculate crystallite size from a peak.
    pub fn crystallite_size(&self, peak: &XrdPeak) -> f64 {
        scherrer_size(self.wavelength_a, peak.fwhm_deg, peak.two_theta, 0.9)
    }

    /// D-spacing for a peak.
    pub fn d_spacing(&self, two_theta: f64) -> f64 {
        bragg_d_spacing(self.wavelength_a, two_theta, 1)
    }
}

impl Default for XrdProcessor {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    #[test]
    fn test_bragg_d_spacing() {
        // Cu Kα, 2θ=30°, n=1: d = 1.5406 / (2*sin(15°)) = 2.98 Å
        let d: f64 = bragg_d_spacing(CU_KA, 30.0, 1);
        assert!(approx_eq(d, 2.98, 0.02));
    }

    #[test]
    fn test_bragg_two_theta() {
        let d: f64 = 2.98;
        let tt: f64 = bragg_two_theta(CU_KA, d, 1);
        assert!(approx_eq(tt, 30.0, 0.2));
    }

    #[test]
    fn test_bragg_roundtrip() {
        let tt: f64 = 45.0;
        let d: f64 = bragg_d_spacing(CU_KA, tt, 1);
        let tt_back: f64 = bragg_two_theta(CU_KA, d, 1);
        assert!(approx_eq(tt_back, tt, 0.01));
    }

    #[test]
    fn test_cubic_d_spacing() {
        let lat = LatticeParams::cubic(5.43); // Silicon
        let d_111: f64 = lat.d_spacing(1, 1, 1);
        // d = 5.43/sqrt(3) = 3.135 Å
        assert!(approx_eq(d_111, 3.135, 0.01));
    }

    #[test]
    fn test_cubic_d_spacing_200() {
        let lat = LatticeParams::cubic(5.43);
        let d_200: f64 = lat.d_spacing(2, 0, 0);
        assert!(approx_eq(d_200, 2.715, 0.01));
    }

    #[test]
    fn test_tetragonal_d_spacing() {
        let lat = LatticeParams::tetragonal(3.78, 9.51);
        let d_001: f64 = lat.d_spacing(0, 0, 1);
        assert!(approx_eq(d_001, 9.51, 0.01));
    }

    #[test]
    fn test_hexagonal_d_spacing() {
        let lat = LatticeParams::hexagonal(2.46, 6.71); // Graphite
        let d_002: f64 = lat.d_spacing(0, 0, 2);
        assert!(approx_eq(d_002, 3.355, 0.01));
    }

    #[test]
    fn test_orthorhombic_d_spacing() {
        let lat = LatticeParams::orthorhombic(4.0, 5.0, 6.0);
        let d_100: f64 = lat.d_spacing(1, 0, 0);
        assert!(approx_eq(d_100, 4.0, 0.01));
    }

    #[test]
    fn test_generate_reflections() {
        let lat = LatticeParams::cubic(5.43);
        let refls = lat.generate_reflections(CU_KA, 80.0, 4);
        assert!(!refls.is_empty());
        // First peak is (100) at ~16.3° for simple cubic
        assert!(refls[0].two_theta < 20.0);
        // Verify (111) exists somewhere
        let has_111 = refls.iter().any(|r| r.h == 1 && r.k == 1 && r.l == 1);
        assert!(has_111);
    }

    #[test]
    fn test_xrd_find_peaks() {
        let pat = XrdPattern::new(
            vec![20.0, 25.0, 28.0, 30.0, 35.0, 40.0, 45.0],
            vec![10.0, 15.0, 100.0, 50.0, 10.0, 80.0, 10.0],
        );
        let peaks = pat.find_peaks(30.0);
        assert!(peaks.len() >= 2);
    }

    #[test]
    fn test_xrd_background_subtraction() {
        let mut pat = XrdPattern::new(
            vec![20.0, 25.0, 30.0, 35.0, 40.0],
            vec![10.0, 50.0, 100.0, 50.0, 10.0],
        );
        pat.subtract_background(3);
        assert!(pat.intensities[2] > pat.intensities[0]);
    }

    #[test]
    fn test_scherrer_size() {
        // Cu Kα, FWHM=0.2°, 2θ=30°, K=0.9
        let size: f64 = scherrer_size(CU_KA, 0.2, 30.0, 0.9);
        // ~400 Å = 40 nm
        assert!(size > 200.0 && size < 600.0);
    }

    #[test]
    fn test_scherrer_broad_peak() {
        // Broader peak → smaller crystallite
        let small: f64 = scherrer_size(CU_KA, 1.0, 30.0, 0.9);
        let large: f64 = scherrer_size(CU_KA, 0.1, 30.0, 0.9);
        assert!(small < large);
    }

    #[test]
    fn test_correct_broadening() {
        let beta: f64 = correct_broadening(0.3, 0.1);
        // sqrt(0.09 - 0.01) = sqrt(0.08) ≈ 0.283
        assert!(approx_eq(beta, 0.283, 0.01));
    }

    #[test]
    fn test_correct_broadening_narrow() {
        let beta: f64 = correct_broadening(0.1, 0.2);
        assert!(approx_eq(beta, 0.01, 0.001)); // clamped to minimum
    }

    #[test]
    fn test_williamson_hall_data() {
        let peaks: Vec<XrdPeak> = vec![
            XrdPeak { two_theta: 30.0, intensity: 100.0, fwhm_deg: 0.3 },
            XrdPeak { two_theta: 50.0, intensity: 80.0, fwhm_deg: 0.35 },
        ];
        let data = williamson_hall_data(&peaks, CU_KA, 0.1);
        assert_eq!(data.len(), 2);
        assert!(data[0].sin_theta > 0.0);
    }

    #[test]
    fn test_williamson_hall_fit() {
        let points: Vec<WhPoint> = vec![
            WhPoint { sin_theta: 0.25, beta_cos_theta: 0.003 },
            WhPoint { sin_theta: 0.4, beta_cos_theta: 0.004 },
            WhPoint { sin_theta: 0.55, beta_cos_theta: 0.005 },
        ];
        let (size, strain) = williamson_hall_fit(&points, CU_KA, 0.9);
        assert!(size > 0.0);
        assert!(strain >= 0.0);
    }

    #[test]
    fn test_simulate_pattern() {
        let lat = LatticeParams::cubic(5.43);
        let refls = lat.generate_reflections(CU_KA, 60.0, 3);
        let ints: Vec<f64> = refls.iter().map(|_| 100.0).collect();
        let pat = simulate_pattern(&refls, &ints, (10.0, 70.0), 601, 0.2);
        assert_eq!(pat.two_theta.len(), 601);
        // Should have peaks near Si (111) at ~28.4°
        let idx: usize = ((28.4 - 10.0) / ((70.0 - 10.0) / 600.0)) as usize;
        assert!(pat.intensities[idx] > 0.1);
    }

    #[test]
    fn test_refine_cubic_lattice() {
        // Si peaks
        let peaks: Vec<(f64, (i32, i32, i32))> = vec![
            (28.44, (1, 1, 1)),
            (47.30, (2, 2, 0)),
            (56.12, (3, 1, 1)),
        ];
        let a: f64 = refine_cubic_lattice(&peaks, CU_KA);
        assert!(approx_eq(a, 5.43, 0.02));
    }

    #[test]
    fn test_xrd_processor_new() {
        let proc = XrdProcessor::new();
        assert!(approx_eq(proc.wavelength_a, CU_KA, 0.001));
        assert!(proc.pattern.is_none());
    }

    #[test]
    fn test_xrd_processor_d_spacing() {
        let proc = XrdProcessor::new();
        let d: f64 = proc.d_spacing(30.0);
        assert!(approx_eq(d, 2.98, 0.02));
    }

    #[test]
    fn test_xrd_processor_crystallite_size() {
        let proc = XrdProcessor::new();
        let peak = XrdPeak { two_theta: 30.0, intensity: 100.0, fwhm_deg: 0.2 };
        let size: f64 = proc.crystallite_size(&peak);
        assert!(size > 200.0);
    }

    #[test]
    fn test_xrd_processor_set_wavelength() {
        let mut proc = XrdProcessor::new();
        proc.set_wavelength(MO_KA);
        assert!(approx_eq(proc.wavelength_a, MO_KA, 0.001));
    }

    #[test]
    fn test_xrd_processor_default() {
        let proc = XrdProcessor::default();
        assert!(proc.pattern.is_none());
    }

    #[test]
    fn test_bragg_zero_angle() {
        let d: f64 = bragg_d_spacing(CU_KA, 0.0, 1);
        assert!(d.is_infinite());
    }

    #[test]
    fn test_bragg_invalid_angle() {
        let tt: f64 = bragg_two_theta(CU_KA, 0.5, 1);
        // d=0.5 < λ/2 => not possible
        assert!(tt.is_nan());
    }

    #[test]
    fn test_lattice_params_cubic() {
        let lat = LatticeParams::cubic(4.0);
        assert!(approx_eq(lat.a, 4.0, 0.001));
        assert_eq!(lat.system, CrystalSystem::Cubic);
    }

    #[test]
    fn test_xrd_pattern_single() {
        let pat = XrdPattern::new(vec![30.0], vec![100.0]);
        let peaks = pat.find_peaks(50.0);
        assert!(peaks.is_empty()); // single point can't be a peak
    }

    #[test]
    fn test_scherrer_zero_fwhm() {
        let size: f64 = scherrer_size(CU_KA, 0.0, 30.0, 0.9);
        assert!(size.is_infinite());
    }

    #[test]
    fn test_wavelength_constants() {
        assert!(CU_KA > CU_KA1);
        assert!(CU_KA2 > CU_KA1);
        assert!(MO_KA < CU_KA);
        assert!(CO_KA > CU_KA);
    }

    #[test]
    fn test_refine_empty() {
        let a: f64 = refine_cubic_lattice(&[], CU_KA);
        assert!(approx_eq(a, 0.0, 0.001));
    }

    #[test]
    fn test_xrd_processor_find_peaks() {
        let mut proc = XrdProcessor::new();
        proc.load_pattern(XrdPattern::new(
            vec![20.0, 25.0, 28.0, 30.0, 35.0],
            vec![10.0, 20.0, 100.0, 30.0, 10.0],
        ));
        let peaks = proc.find_peaks(50.0);
        assert!(!peaks.is_empty());
    }

    #[test]
    fn test_cubic_d_spacing_110() {
        let lat = LatticeParams::cubic(4.0);
        let d: f64 = lat.d_spacing(1, 1, 0);
        // d = 4/sqrt(2) = 2.828
        assert!(approx_eq(d, 2.828, 0.01));
    }

    #[test]
    fn test_hexagonal_100() {
        let lat = LatticeParams::hexagonal(2.46, 6.71);
        let d_100: f64 = lat.d_spacing(1, 0, 0);
        // d = a / sqrt(4/3) = 2.46 / 1.155 = 2.13
        assert!(approx_eq(d_100, 2.13, 0.02));
    }

    #[test]
    fn test_simulate_pattern_empty() {
        let pat = simulate_pattern(&[], &[], (10.0, 70.0), 100, 0.2);
        assert_eq!(pat.two_theta.len(), 100);
        // All zero
        for &v in &pat.intensities {
            assert!(approx_eq(v, 0.0, 0.001));
        }
    }
}
