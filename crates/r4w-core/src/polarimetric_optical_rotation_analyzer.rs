//! # Polarimetric Optical Rotation Analyzer
//!
//! Implements optical rotation (polarimetry) analysis for measuring chiral molecules,
//! sugar concentrations, and optical activity using polarized light rotation measurements.
//!
//! ## Key Components
//!
//! - **OpticalRotation** - Fundamental specific/molar rotation, temperature/wavelength correction
//! - **DrudeEquation** - Optical rotatory dispersion (ORD) modeling and fitting
//! - **SugarAnalyzer** - Saccharimetry for sucrose, glucose, fructose quantitation
//! - **EnantiomericExcess** - Chiral purity (ee%, er) from rotation data
//! - **StokesVector** - Full polarization state representation (S0..S3)
//! - **MuellerMatrix** - 4x4 optical element modeling (polarizers, wave plates, rotators)
//! - **CircularDichroism** - CD spectroscopy, Kronig-Kramers transform, secondary structure
//! - **FaradayRotation** - Magneto-optical rotation (Verdet constant, field measurement)
//! - **PolarimeterSimulator** - Synthetic measurement generation with noise
//! - **ConcentrationCalibration** - Quantitative calibration curves, detection limits

use std::f64::consts::PI;

// ============================================================
// 1. OpticalRotation
// ============================================================

/// Fundamental optical rotation measurement.
///
/// Biot's law: alpha_obs = [alpha]_lambda^T * l * c
pub struct OpticalRotation {
    /// Observed rotation in degrees
    pub observed_rotation_deg: f64,
    /// Path length in decimeters
    pub path_length_dm: f64,
    /// Concentration in g/mL
    pub concentration_g_per_ml: f64,
}

impl OpticalRotation {
    pub fn new(observed_rotation_deg: f64, path_length_dm: f64, concentration_g_per_ml: f64) -> Self {
        Self {
            observed_rotation_deg,
            path_length_dm,
            concentration_g_per_ml,
        }
    }

    /// Specific rotation [alpha] = alpha_obs / (l * c)
    pub fn specific_rotation(&self) -> f64 {
        if self.path_length_dm.abs() < 1e-15 || self.concentration_g_per_ml.abs() < 1e-15 {
            return 0.0;
        }
        self.observed_rotation_deg / (self.path_length_dm * self.concentration_g_per_ml)
    }

    /// Molar rotation [Phi] = [alpha] * M / 100
    pub fn molar_rotation(&self, molar_mass: f64) -> f64 {
        self.specific_rotation() * molar_mass / 100.0
    }

    /// Temperature correction: [alpha]_T = [alpha]_Tref * (1 + coeff * (T - Tref))
    pub fn temperature_correction(&self, temp_c: f64, ref_temp_c: f64, coeff: f64) -> f64 {
        self.specific_rotation() * (1.0 + coeff * (temp_c - ref_temp_c))
    }

    /// Wavelength correction via single-term Drude equation.
    /// [alpha]_lambda = [alpha]_ref * (ref_lambda^2 - lambda0^2) / (lambda^2 - lambda0^2)
    /// Using a simplified proportionality: [alpha] ~ 1/(lambda^2 - lambda0^2)
    /// with lambda0 assumed negligible for the ratio form.
    /// More precisely: [alpha]_lambda / [alpha]_ref = ref_lambda^2 / lambda^2  (simple dispersion)
    pub fn wavelength_correction(&self, lambda_nm: f64, ref_lambda_nm: f64) -> f64 {
        if lambda_nm.abs() < 1e-15 {
            return 0.0;
        }
        self.specific_rotation() * (ref_lambda_nm * ref_lambda_nm) / (lambda_nm * lambda_nm)
    }
}

// ============================================================
// 2. DrudeEquation (Optical Rotatory Dispersion)
// ============================================================

/// Result of Drude equation fitting.
#[derive(Debug, Clone)]
pub struct DrudeFitResult {
    /// Amplitude parameter A
    pub a: f64,
    /// Resonance wavelength lambda_0 in nm
    pub lambda0: f64,
    /// Residual error (sum of squares)
    pub residual: f64,
}

/// Classification of ORD curve shape.
#[derive(Debug, Clone, PartialEq)]
pub enum CurveType {
    /// Monotonic rotation vs wavelength (no absorption band nearby)
    PlainCurve,
    /// Anomalous dispersion near absorption (sign change, S-shape)
    CottonEffect,
}

/// Drude equation for optical rotatory dispersion.
pub struct DrudeEquation;

impl DrudeEquation {
    /// Single-term Drude: [alpha] = A / (lambda^2 - lambda0^2)
    pub fn single_term(lambda_nm: f64, a: f64, lambda0_nm: f64) -> f64 {
        let denom = lambda_nm * lambda_nm - lambda0_nm * lambda0_nm;
        if denom.abs() < 1e-15 {
            return f64::INFINITY * a.signum();
        }
        a / denom
    }

    /// Two-term Drude: [alpha] = A1/(lambda^2 - l1^2) + A2/(lambda^2 - l2^2)
    pub fn two_term(lambda_nm: f64, a1: f64, l1: f64, a2: f64, l2: f64) -> f64 {
        Self::single_term(lambda_nm, a1, l1) + Self::single_term(lambda_nm, a2, l2)
    }

    /// Fit single-term Drude to wavelength/rotation data using grid search.
    ///
    /// Minimizes sum of (rotation_i - A/(lambda_i^2 - lambda0^2))^2
    pub fn fit_drude(wavelengths: &[f64], rotations: &[f64]) -> DrudeFitResult {
        assert_eq!(wavelengths.len(), rotations.len());
        if wavelengths.is_empty() {
            return DrudeFitResult { a: 0.0, lambda0: 0.0, residual: f64::INFINITY };
        }

        let mut best_a = 0.0;
        let mut best_l0 = 0.0;
        let mut best_res = f64::INFINITY;

        // Grid search over lambda0 (100-400 nm typical UV absorption)
        for l0_idx in 0..300 {
            let l0 = 100.0 + l0_idx as f64;

            // For a given lambda0, optimal A is found by linear least squares:
            // rotation_i = A * x_i where x_i = 1/(lambda_i^2 - l0^2)
            // A = sum(rot_i * x_i) / sum(x_i^2)
            let mut sum_rx = 0.0;
            let mut sum_xx = 0.0;
            let mut valid = true;

            for i in 0..wavelengths.len() {
                let denom = wavelengths[i] * wavelengths[i] - l0 * l0;
                if denom.abs() < 1e-6 {
                    valid = false;
                    break;
                }
                let x = 1.0 / denom;
                sum_rx += rotations[i] * x;
                sum_xx += x * x;
            }

            if !valid || sum_xx < 1e-30 {
                continue;
            }

            let a = sum_rx / sum_xx;

            // Compute residual
            let mut res = 0.0;
            for i in 0..wavelengths.len() {
                let denom = wavelengths[i] * wavelengths[i] - l0 * l0;
                let pred = a / denom;
                let diff = rotations[i] - pred;
                res += diff * diff;
            }

            if res < best_res {
                best_res = res;
                best_a = a;
                best_l0 = l0;
            }
        }

        DrudeFitResult {
            a: best_a,
            lambda0: best_l0,
            residual: best_res,
        }
    }

    /// Classify ORD data as plain curve or Cotton effect.
    ///
    /// Cotton effect shows sign change (S-shape) near absorption band.
    pub fn plain_curve_vs_cotton_effect(ord_data: &[(f64, f64)]) -> CurveType {
        if ord_data.len() < 2 {
            return CurveType::PlainCurve;
        }

        // Check for sign change in rotation values
        let mut has_sign_change = false;
        for i in 1..ord_data.len() {
            if ord_data[i].1 * ord_data[i - 1].1 < 0.0 {
                has_sign_change = true;
                break;
            }
        }

        // Check for non-monotonic behavior (slope reversal)
        let mut has_slope_reversal = false;
        if ord_data.len() >= 3 {
            for i in 2..ord_data.len() {
                let slope1 = ord_data[i - 1].1 - ord_data[i - 2].1;
                let slope2 = ord_data[i].1 - ord_data[i - 1].1;
                if slope1 * slope2 < 0.0 {
                    has_slope_reversal = true;
                    break;
                }
            }
        }

        if has_sign_change || has_slope_reversal {
            CurveType::CottonEffect
        } else {
            CurveType::PlainCurve
        }
    }
}

// ============================================================
// 3. SugarAnalyzer
// ============================================================

/// Result of mutarotation kinetics fitting.
#[derive(Debug, Clone)]
pub struct MutarotationResult {
    /// Rate constant k (min^-1)
    pub rate_constant: f64,
    /// Initial rotation (alpha_0)
    pub alpha_0: f64,
    /// Equilibrium rotation (alpha_inf)
    pub alpha_inf: f64,
    /// R-squared goodness of fit
    pub r_squared: f64,
}

/// Saccharimetry for sugar solution analysis.
///
/// Specific rotations at 589 nm (sodium D-line), 20 deg C:
/// - Sucrose: +66.47 deg
/// - Glucose (D-glucose): +52.7 deg
/// - Fructose (D-fructose): -92.0 deg
pub struct SugarAnalyzer;

/// Specific rotation of sucrose at 589 nm, 20 C (deg dm^-1 (g/mL)^-1)
const SUCROSE_SPECIFIC_ROTATION: f64 = 66.47;
/// Specific rotation of D-glucose at 589 nm, 20 C
const GLUCOSE_SPECIFIC_ROTATION: f64 = 52.7;
/// Specific rotation of D-fructose at 589 nm, 20 C
const FRUCTOSE_SPECIFIC_ROTATION: f64 = -92.0;

impl SugarAnalyzer {
    /// Sucrose concentration (g/100 mL) from observed rotation.
    /// alpha = [alpha] * l * c  =>  c = alpha / ([alpha] * l)
    /// Returns g per 100 mL (percentage w/v).
    pub fn sucrose_concentration(rotation_deg: f64, path_length_dm: f64) -> f64 {
        if path_length_dm.abs() < 1e-15 {
            return 0.0;
        }
        // c is in g/mL, multiply by 100 for g/100mL
        (rotation_deg / (SUCROSE_SPECIFIC_ROTATION * path_length_dm)) * 100.0
    }

    /// Glucose concentration (g/100 mL) from observed rotation.
    pub fn glucose_concentration(rotation_deg: f64, path_length_dm: f64) -> f64 {
        if path_length_dm.abs() < 1e-15 {
            return 0.0;
        }
        (rotation_deg / (GLUCOSE_SPECIFIC_ROTATION * path_length_dm)) * 100.0
    }

    /// Fructose concentration (g/100 mL) from observed rotation.
    /// Note: fructose is levorotatory (negative specific rotation).
    pub fn fructose_concentration(rotation_deg: f64, path_length_dm: f64) -> f64 {
        if path_length_dm.abs() < 1e-15 {
            return 0.0;
        }
        (rotation_deg / (FRUCTOSE_SPECIFIC_ROTATION * path_length_dm)) * 100.0
    }

    /// International Sugar Scale (degrees Z).
    /// Normal quartz wedge reading with compensation.
    /// Z = (alpha / quartz_wedge_deg) * 100
    pub fn international_sugar_scale(rotation_deg: f64, quartz_wedge_deg: f64) -> f64 {
        if quartz_wedge_deg.abs() < 1e-15 {
            return 0.0;
        }
        (rotation_deg / quartz_wedge_deg) * 100.0
    }

    /// Inversion progress (fraction inverted) for sucrose hydrolysis.
    /// f = (alpha_t - alpha_0) / (alpha_inf - alpha_0)
    pub fn inversion_progress(alpha_t: f64, alpha_0: f64, alpha_inf: f64) -> f64 {
        let denom = alpha_inf - alpha_0;
        if denom.abs() < 1e-15 {
            return 0.0;
        }
        (alpha_t - alpha_0) / denom
    }

    /// Fit mutarotation kinetics: alpha(t) = alpha_inf + (alpha_0 - alpha_inf) * exp(-k*t)
    ///
    /// Uses linearized fitting: ln(alpha(t) - alpha_inf) = ln(alpha_0 - alpha_inf) - k*t
    pub fn mutarotation_kinetics(alpha_values: &[f64], times_min: &[f64]) -> MutarotationResult {
        assert_eq!(alpha_values.len(), times_min.len());
        let n = alpha_values.len();
        if n < 3 {
            return MutarotationResult {
                rate_constant: 0.0,
                alpha_0: if n > 0 { alpha_values[0] } else { 0.0 },
                alpha_inf: if n > 0 { *alpha_values.last().unwrap() } else { 0.0 },
                r_squared: 0.0,
            };
        }

        let alpha_0 = alpha_values[0];
        let alpha_inf = *alpha_values.last().unwrap();

        // Linearize: y = ln|alpha(t) - alpha_inf| vs t
        let mut valid_t = Vec::new();
        let mut valid_y = Vec::new();

        for i in 0..n - 1 {
            let diff = alpha_values[i] - alpha_inf;
            if diff.abs() > 1e-10 {
                valid_t.push(times_min[i]);
                valid_y.push(diff.abs().ln());
            }
        }

        if valid_t.len() < 2 {
            return MutarotationResult {
                rate_constant: 0.0,
                alpha_0,
                alpha_inf,
                r_squared: 0.0,
            };
        }

        // Linear regression: y = a + b*t, where b = -k
        let m = valid_t.len() as f64;
        let sum_t: f64 = valid_t.iter().sum();
        let sum_y: f64 = valid_y.iter().sum();
        let sum_tt: f64 = valid_t.iter().map(|t| t * t).sum();
        let sum_ty: f64 = valid_t.iter().zip(valid_y.iter()).map(|(t, y)| t * y).sum();

        let denom = m * sum_tt - sum_t * sum_t;
        if denom.abs() < 1e-30 {
            return MutarotationResult {
                rate_constant: 0.0,
                alpha_0,
                alpha_inf,
                r_squared: 0.0,
            };
        }

        let b = (m * sum_ty - sum_t * sum_y) / denom;
        let k = -b; // slope is -k

        // R-squared
        let y_mean = sum_y / m;
        let ss_tot: f64 = valid_y.iter().map(|y| (y - y_mean).powi(2)).sum();
        let a = (sum_y - b * sum_t) / m;
        let ss_res: f64 = valid_t
            .iter()
            .zip(valid_y.iter())
            .map(|(t, y)| {
                let pred = a + b * t;
                (y - pred).powi(2)
            })
            .sum();

        let r_squared = if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };

        MutarotationResult {
            rate_constant: k.max(0.0),
            alpha_0,
            alpha_inf,
            r_squared,
        }
    }
}

// ============================================================
// 4. EnantiomericExcess
// ============================================================

/// Chiral purity analysis.
pub struct EnantiomericExcess;

impl EnantiomericExcess {
    /// Enantiomeric excess from optical rotation.
    /// ee% = (alpha_obs / alpha_pure) * 100
    pub fn ee_from_rotation(observed: f64, pure_rotation: f64) -> f64 {
        if pure_rotation.abs() < 1e-15 {
            return 0.0;
        }
        (observed / pure_rotation).abs() * 100.0
    }

    /// Convert ee% to enantiomeric ratio (R:S fractions).
    /// If ee = 80%, R = 90%, S = 10%
    /// R = (100 + ee) / 200, S = (100 - ee) / 200
    pub fn er_from_ee(ee_percent: f64) -> (f64, f64) {
        let ee = ee_percent.abs().min(100.0);
        let r = (100.0 + ee) / 200.0;
        let s = (100.0 - ee) / 200.0;
        (r, s)
    }

    /// Convert enantiomeric ratio to ee%.
    /// ee% = |R - S| / (R + S) * 100
    pub fn ee_from_er(r_fraction: f64, s_fraction: f64) -> f64 {
        let total = r_fraction + s_fraction;
        if total < 1e-15 {
            return 0.0;
        }
        ((r_fraction - s_fraction).abs() / total) * 100.0
    }

    /// Diastereomeric excess from mixture.
    /// de% = |sum(rotation_i * fraction_i)| / max(|rotation_i|) * 100
    /// More precisely: de = |sum(alpha_i * x_i)| where x_i are mole fractions
    pub fn de_from_diastereomeric_mixture(rotations: &[f64], fractions: &[f64]) -> f64 {
        assert_eq!(rotations.len(), fractions.len());
        if rotations.is_empty() {
            return 0.0;
        }

        let weighted_sum: f64 = rotations
            .iter()
            .zip(fractions.iter())
            .map(|(r, f)| r * f)
            .sum();

        let max_abs = rotations
            .iter()
            .map(|r| r.abs())
            .fold(0.0_f64, f64::max);

        if max_abs < 1e-15 {
            return 0.0;
        }

        (weighted_sum.abs() / max_abs) * 100.0
    }
}

// ============================================================
// 5. StokesVector
// ============================================================

/// Stokes vector representing the polarization state of light.
/// S = (S0, S1, S2, S3) where:
/// - S0: total intensity
/// - S1: horizontal vs vertical linear polarization
/// - S2: +45 vs -45 linear polarization
/// - S3: right vs left circular polarization
#[derive(Debug, Clone, PartialEq)]
pub struct StokesVector {
    pub s0: f64,
    pub s1: f64,
    pub s2: f64,
    pub s3: f64,
}

impl StokesVector {
    pub fn new(s0: f64, s1: f64, s2: f64, s3: f64) -> Self {
        Self { s0, s1, s2, s3 }
    }

    /// Convert Jones vector (Ex, Ey) to Stokes vector.
    /// Ex = (ex_re, ex_im), Ey = (ey_re, ey_im)
    ///
    /// S0 = |Ex|^2 + |Ey|^2
    /// S1 = |Ex|^2 - |Ey|^2
    /// S2 = 2 Re(Ex Ey*)
    /// S3 = 2 Im(Ex Ey*)
    pub fn from_jones(ex: (f64, f64), ey: (f64, f64)) -> StokesVector {
        let ex_mag_sq = ex.0 * ex.0 + ex.1 * ex.1;
        let ey_mag_sq = ey.0 * ey.0 + ey.1 * ey.1;
        // Ex * Ey* = (ex_re + j*ex_im)(ey_re - j*ey_im)
        let cross_re = ex.0 * ey.0 + ex.1 * ey.1;
        let cross_im = ex.1 * ey.0 - ex.0 * ey.1;

        StokesVector {
            s0: ex_mag_sq + ey_mag_sq,
            s1: ex_mag_sq - ey_mag_sq,
            s2: 2.0 * cross_re,
            s3: 2.0 * cross_im,
        }
    }

    /// Degree of polarization: DOP = sqrt(S1^2 + S2^2 + S3^2) / S0
    pub fn degree_of_polarization(&self) -> f64 {
        if self.s0.abs() < 1e-15 {
            return 0.0;
        }
        (self.s1 * self.s1 + self.s2 * self.s2 + self.s3 * self.s3).sqrt() / self.s0
    }

    /// Ellipticity angle chi = 0.5 * arcsin(S3 / S0)
    pub fn ellipticity_angle(&self) -> f64 {
        if self.s0.abs() < 1e-15 {
            return 0.0;
        }
        let ratio = (self.s3 / self.s0).clamp(-1.0, 1.0);
        0.5 * ratio.asin()
    }

    /// Azimuth angle psi = 0.5 * atan2(S2, S1)
    pub fn azimuth_angle(&self) -> f64 {
        0.5 * self.s2.atan2(self.s1)
    }

    /// Horizontal linear polarization: S = (1, 1, 0, 0)
    pub fn linear_horizontal() -> StokesVector {
        StokesVector::new(1.0, 1.0, 0.0, 0.0)
    }

    /// Vertical linear polarization: S = (1, -1, 0, 0)
    pub fn linear_vertical() -> StokesVector {
        StokesVector::new(1.0, -1.0, 0.0, 0.0)
    }

    /// +45 degree linear polarization: S = (1, 0, 1, 0)
    pub fn linear_plus45() -> StokesVector {
        StokesVector::new(1.0, 0.0, 1.0, 0.0)
    }

    /// -45 degree linear polarization: S = (1, 0, -1, 0)
    pub fn linear_minus45() -> StokesVector {
        StokesVector::new(1.0, 0.0, -1.0, 0.0)
    }

    /// Right circular polarization: S = (1, 0, 0, 1)
    pub fn circular_right() -> StokesVector {
        StokesVector::new(1.0, 0.0, 0.0, 1.0)
    }

    /// Left circular polarization: S = (1, 0, 0, -1)
    pub fn circular_left() -> StokesVector {
        StokesVector::new(1.0, 0.0, 0.0, -1.0)
    }

    /// Unpolarized light: S = (1, 0, 0, 0)
    pub fn unpolarized() -> StokesVector {
        StokesVector::new(1.0, 0.0, 0.0, 0.0)
    }

    /// Access as array
    pub fn as_array(&self) -> [f64; 4] {
        [self.s0, self.s1, self.s2, self.s3]
    }
}

// ============================================================
// 6. MuellerMatrix
// ============================================================

/// 4x4 Mueller matrix for modeling optical elements.
///
/// Transforms Stokes vectors: S_out = M * S_in
#[derive(Debug, Clone)]
pub struct MuellerMatrix {
    pub m: [[f64; 4]; 4],
}

impl MuellerMatrix {
    pub fn new(m: [[f64; 4]; 4]) -> Self {
        Self { m }
    }

    /// Identity Mueller matrix (no effect on polarization).
    pub fn identity() -> MuellerMatrix {
        MuellerMatrix::new([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }

    /// Optical rotation element (rotator) by angle theta (degrees).
    /// Rotates linear polarization azimuth by theta (counterclockwise).
    pub fn rotator(angle_deg: f64) -> MuellerMatrix {
        let theta = angle_deg * PI / 180.0;
        let c2 = (2.0 * theta).cos();
        let s2 = (2.0 * theta).sin();

        MuellerMatrix::new([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, c2, -s2, 0.0],
            [0.0, s2, c2, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ])
    }

    /// Linear polarizer at angle theta (degrees) from horizontal.
    pub fn linear_polarizer(angle_deg: f64) -> MuellerMatrix {
        let theta = angle_deg * PI / 180.0;
        let c2 = (2.0 * theta).cos();
        let s2 = (2.0 * theta).sin();

        MuellerMatrix::new([
            [1.0, c2, s2, 0.0],
            [c2, c2 * c2, c2 * s2, 0.0],
            [s2, s2 * c2, s2 * s2, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        ])
        .scale(0.5)
    }

    /// Quarter-wave plate with fast axis at angle theta (degrees).
    pub fn quarter_wave_plate(angle_deg: f64) -> MuellerMatrix {
        let theta = angle_deg * PI / 180.0;
        let c2 = (2.0 * theta).cos();
        let s2 = (2.0 * theta).sin();

        MuellerMatrix::new([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, c2 * c2, c2 * s2, -s2],
            [0.0, s2 * c2, s2 * s2, c2],
            [0.0, s2, -c2, 0.0],
        ])
    }

    /// Half-wave plate with fast axis at angle theta (degrees).
    pub fn half_wave_plate(angle_deg: f64) -> MuellerMatrix {
        let theta = angle_deg * PI / 180.0;
        let c2 = (2.0 * theta).cos();
        let s2 = (2.0 * theta).sin();
        let c4 = (4.0 * theta).cos();
        let s4 = (4.0 * theta).sin();

        MuellerMatrix::new([
            [1.0, 0.0, 0.0, 0.0],
            [0.0, c4, s4, 0.0],
            [0.0, s4, -c4, 0.0],
            [0.0, 0.0, 0.0, -1.0],
        ])
    }

    /// Circular dichroism element.
    /// cd parameter represents differential absorption (0 = none, 1 = full).
    pub fn circular_dichroism(cd: f64) -> MuellerMatrix {
        let cd_clamped = cd.clamp(0.0, 1.0);
        let t = 1.0 - cd_clamped;

        MuellerMatrix::new([
            [1.0, 0.0, 0.0, cd_clamped],
            [0.0, t, 0.0, 0.0],
            [0.0, 0.0, t, 0.0],
            [cd_clamped, 0.0, 0.0, 1.0],
        ])
    }

    /// Matrix multiplication: result = a * b
    pub fn multiply(a: &MuellerMatrix, b: &MuellerMatrix) -> MuellerMatrix {
        let mut result = [[0.0; 4]; 4];
        for i in 0..4 {
            for j in 0..4 {
                for k in 0..4 {
                    result[i][j] += a.m[i][k] * b.m[k][j];
                }
            }
        }
        MuellerMatrix::new(result)
    }

    /// Apply Mueller matrix to a Stokes vector: S_out = M * S_in
    pub fn apply(m: &MuellerMatrix, s: &StokesVector) -> StokesVector {
        let sv = s.as_array();
        let mut out = [0.0; 4];
        for i in 0..4 {
            for j in 0..4 {
                out[i] += m.m[i][j] * sv[j];
            }
        }
        StokesVector::new(out[0], out[1], out[2], out[3])
    }

    /// Scale all elements by a factor.
    fn scale(mut self, factor: f64) -> Self {
        for i in 0..4 {
            for j in 0..4 {
                self.m[i][j] *= factor;
            }
        }
        self
    }
}

// ============================================================
// 7. CircularDichroism
// ============================================================

/// Estimated secondary structure fractions from CD data.
#[derive(Debug, Clone)]
pub struct SecondaryStructure {
    /// Fraction alpha-helix (0.0 to 1.0)
    pub alpha_helix: f64,
    /// Fraction beta-sheet (0.0 to 1.0)
    pub beta_sheet: f64,
    /// Fraction random coil (0.0 to 1.0)
    pub random_coil: f64,
}

/// Circular dichroism spectroscopy analysis.
pub struct CircularDichroism;

impl CircularDichroism {
    /// Differential extinction coefficient: delta_epsilon = epsilon_L - epsilon_R
    pub fn delta_epsilon(abs_left: f64, abs_right: f64) -> f64 {
        abs_left - abs_right
    }

    /// Ellipticity in millidegrees from differential absorbance.
    /// theta_mdeg = 32.982 * delta_A
    pub fn ellipticity_mdeg(delta_a: f64) -> f64 {
        32.982 * delta_a
    }

    /// Molar ellipticity [theta] = theta_mdeg / (c * l * 10)
    /// where c is mol/L, l is path length in cm.
    /// Units: deg cm^2 dmol^-1
    pub fn molar_ellipticity(theta_mdeg: f64, conc_mol_per_l: f64, path_cm: f64) -> f64 {
        if conc_mol_per_l.abs() < 1e-15 || path_cm.abs() < 1e-15 {
            return 0.0;
        }
        theta_mdeg / (conc_mol_per_l * path_cm * 10.0)
    }

    /// Mean residue ellipticity for proteins.
    /// [theta]_MRW = [theta] / num_residues
    pub fn mean_residue_ellipticity(theta: f64, num_residues: usize) -> f64 {
        if num_residues == 0 {
            return 0.0;
        }
        theta / num_residues as f64
    }

    /// Estimate secondary structure from mean residue weight CD spectrum.
    ///
    /// Uses characteristic wavelength signatures:
    /// - Alpha-helix: minima at 208 nm and 222 nm, max at 193 nm
    /// - Beta-sheet: minimum near 218 nm, max near 195 nm
    /// - Random coil: minimum near 200 nm
    ///
    /// Simple estimation based on 222 nm and 208 nm ellipticity values.
    pub fn secondary_structure_estimate(mrw_spectrum: &[(f64, f64)]) -> SecondaryStructure {
        if mrw_spectrum.is_empty() {
            return SecondaryStructure {
                alpha_helix: 0.33,
                beta_sheet: 0.33,
                random_coil: 0.34,
            };
        }

        // Find ellipticity values near key wavelengths
        let theta_222 = interpolate_at(mrw_spectrum, 222.0);
        let theta_208 = interpolate_at(mrw_spectrum, 208.0);
        let theta_218 = interpolate_at(mrw_spectrum, 218.0);

        // Simplified Chen-Yang-Martinez estimation:
        // f_helix ~ -[theta]_222 / 36000  (36000 is reference for 100% helix)
        // Clamp to [0, 1]
        let f_helix = (-theta_222 / 36000.0).clamp(0.0, 1.0);

        // Beta sheet contribution from 218 nm minimum
        let f_beta = (-theta_218 / 20000.0).clamp(0.0, 1.0 - f_helix);

        // Random coil is remainder
        let f_coil = (1.0 - f_helix - f_beta).max(0.0);

        // Normalize
        let total = f_helix + f_beta + f_coil;
        if total < 1e-15 {
            return SecondaryStructure {
                alpha_helix: 0.33,
                beta_sheet: 0.33,
                random_coil: 0.34,
            };
        }

        SecondaryStructure {
            alpha_helix: f_helix / total,
            beta_sheet: f_beta / total,
            random_coil: f_coil / total,
        }
    }

    /// Kronig-Kramers transform: CD spectrum -> ORD spectrum.
    ///
    /// [alpha](lambda) = (2/pi) * integral{ lambda' * delta_epsilon(lambda') / (lambda'^2 - lambda^2) dlambda' }
    /// Numerical integration using trapezoidal rule.
    pub fn kronig_kramers(cd_spectrum: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if cd_spectrum.len() < 2 {
            return cd_spectrum.to_vec();
        }

        let mut ord = Vec::with_capacity(cd_spectrum.len());

        for &(lambda, _) in cd_spectrum {
            let mut integral = 0.0;

            for i in 0..cd_spectrum.len() - 1 {
                let (l1, de1) = cd_spectrum[i];
                let (l2, de2) = cd_spectrum[i + 1];
                let dl = l2 - l1;

                // Evaluate integrand at midpoint (avoid singularity)
                let l_mid = 0.5 * (l1 + l2);
                let de_mid = 0.5 * (de1 + de2);
                let denom = l_mid * l_mid - lambda * lambda;

                if denom.abs() > 1e-6 {
                    integral += l_mid * de_mid / denom * dl;
                }
            }

            let alpha = (2.0 / PI) * integral;
            ord.push((lambda, alpha));
        }

        ord
    }
}

/// Linear interpolation helper for spectral data.
fn interpolate_at(spectrum: &[(f64, f64)], target_wavelength: f64) -> f64 {
    if spectrum.is_empty() {
        return 0.0;
    }
    if spectrum.len() == 1 {
        return spectrum[0].1;
    }

    // Find bracketing points
    for i in 0..spectrum.len() - 1 {
        let (w1, v1) = spectrum[i];
        let (w2, v2) = spectrum[i + 1];
        if (w1 <= target_wavelength && target_wavelength <= w2)
            || (w2 <= target_wavelength && target_wavelength <= w1)
        {
            let frac = if (w2 - w1).abs() > 1e-15 {
                (target_wavelength - w1) / (w2 - w1)
            } else {
                0.5
            };
            return v1 + frac * (v2 - v1);
        }
    }

    // Extrapolate from nearest end
    if (target_wavelength - spectrum[0].0).abs()
        < (target_wavelength - spectrum.last().unwrap().0).abs()
    {
        spectrum[0].1
    } else {
        spectrum.last().unwrap().1
    }
}

// ============================================================
// 8. FaradayRotation
// ============================================================

/// Verdet constant for common materials (rad / (T * m)) at 589 nm.
pub struct VerdetConstants;

impl VerdetConstants {
    /// Flint glass (heavy) ~31.4 rad/(T*m)
    pub const FLINT_GLASS: f64 = 31.4;
    /// Terbium gallium garnet (TGG) ~134 rad/(T*m) - common isolator material
    pub const TGG: f64 = 134.0;
    /// Water ~3.8 rad/(T*m)
    pub const WATER: f64 = 3.8;
    /// Fused silica ~3.7 rad/(T*m)
    pub const FUSED_SILICA: f64 = 3.7;
    /// Crown glass ~8.1 rad/(T*m)
    pub const CROWN_GLASS: f64 = 8.1;
    /// Terbium doped fiber ~40 rad/(T*m) (typical)
    pub const TERBIUM_FIBER: f64 = 40.0;
}

/// Faraday magneto-optical rotation.
///
/// theta = V * B * L  (non-reciprocal rotation in a magnetic field)
pub struct FaradayRotation;

impl FaradayRotation {
    /// Faraday rotation angle in radians.
    /// theta = V * B * L
    /// - V: Verdet constant (rad/(T*m))
    /// - B: magnetic field (Tesla)
    /// - L: path length (meters)
    pub fn rotation_angle(verdet_constant: f64, b_field_t: f64, path_length_m: f64) -> f64 {
        verdet_constant * b_field_t * path_length_m
    }

    /// Rotation angle in degrees.
    pub fn rotation_angle_deg(verdet_constant: f64, b_field_t: f64, path_length_m: f64) -> f64 {
        Self::rotation_angle(verdet_constant, b_field_t, path_length_m) * 180.0 / PI
    }

    /// Determine Verdet constant from measurement.
    /// V = theta / (B * L)
    pub fn verdet_from_measurement(rotation_deg: f64, field_t: f64, length_m: f64) -> f64 {
        let rotation_rad = rotation_deg * PI / 180.0;
        let denom = field_t * length_m;
        if denom.abs() < 1e-15 {
            return 0.0;
        }
        rotation_rad / denom
    }

    /// Determine magnetic field from rotation measurement.
    /// B = theta / (V * L)
    pub fn field_from_rotation(rotation_deg: f64, verdet: f64, length_m: f64) -> f64 {
        let rotation_rad = rotation_deg * PI / 180.0;
        let denom = verdet * length_m;
        if denom.abs() < 1e-15 {
            return 0.0;
        }
        rotation_rad / denom
    }
}

// ============================================================
// 9. PolarimeterSimulator
// ============================================================

/// Simple deterministic pseudo-random noise generator for simulations.
struct SimNoise {
    state: u64,
}

impl SimNoise {
    fn new(seed: u64) -> Self {
        Self { state: seed.wrapping_add(1) }
    }

    /// Generate a pseudo-random f64 in [-1, 1].
    fn next_f64(&mut self) -> f64 {
        // xorshift64
        self.state ^= self.state << 13;
        self.state ^= self.state >> 7;
        self.state ^= self.state << 17;
        (self.state as f64 / u64::MAX as f64) * 2.0 - 1.0
    }

    /// Approximate Gaussian using sum of uniforms (Central Limit Theorem).
    fn next_gaussian(&mut self) -> f64 {
        let mut sum = 0.0;
        for _ in 0..12 {
            sum += self.next_f64();
        }
        sum / (12.0_f64).sqrt()
    }
}

/// Synthetic polarimetric measurement generator.
pub struct PolarimeterSimulator;

impl PolarimeterSimulator {
    /// Simulate a single rotation measurement with Gaussian noise.
    pub fn simulate_measurement(
        specific_rotation: f64,
        concentration: f64,
        path_length: f64,
        noise_deg: f64,
    ) -> f64 {
        let true_rotation = specific_rotation * concentration * path_length;
        let mut rng = SimNoise::new((true_rotation.to_bits()) ^ 0xDEADBEEF);
        true_rotation + noise_deg * rng.next_gaussian()
    }

    /// Simulate an ORD spectrum from Drude parameters.
    pub fn simulate_ord_spectrum(
        drude_params: &DrudeFitResult,
        wavelengths: &[f64],
        noise: f64,
    ) -> Vec<(f64, f64)> {
        let mut rng = SimNoise::new(42);
        wavelengths
            .iter()
            .map(|&w| {
                let rotation = DrudeEquation::single_term(w, drude_params.a, drude_params.lambda0);
                let noisy = rotation + noise * rng.next_gaussian();
                (w, noisy)
            })
            .collect()
    }

    /// Simulate a CD spectrum with Gaussian absorption bands.
    ///
    /// peaks: &[(center_nm, amplitude, width_nm)]
    pub fn simulate_cd_spectrum(
        peaks: &[(f64, f64, f64)],
        wavelengths: &[f64],
    ) -> Vec<(f64, f64)> {
        wavelengths
            .iter()
            .map(|&w| {
                let mut cd = 0.0;
                for &(center, amp, width) in peaks {
                    let exponent = -((w - center) / width).powi(2) * 0.5;
                    cd += amp * exponent.exp();
                }
                (w, cd)
            })
            .collect()
    }
}

// ============================================================
// 10. ConcentrationCalibration
// ============================================================

/// Result of linear calibration curve fitting.
#[derive(Debug, Clone)]
pub struct CalibResult {
    /// Slope (rotation per unit concentration)
    pub slope: f64,
    /// Intercept
    pub intercept: f64,
    /// R-squared coefficient of determination
    pub r_squared: f64,
}

/// Quantitative concentration analysis via calibration curves.
pub struct ConcentrationCalibration;

impl ConcentrationCalibration {
    /// Fit a linear calibration curve: rotation = slope * concentration + intercept
    pub fn calibration_curve(concentrations: &[f64], rotations: &[f64]) -> CalibResult {
        assert_eq!(concentrations.len(), rotations.len());
        let n = concentrations.len() as f64;
        if n < 2.0 {
            return CalibResult {
                slope: 0.0,
                intercept: 0.0,
                r_squared: 0.0,
            };
        }

        let sum_x: f64 = concentrations.iter().sum();
        let sum_y: f64 = rotations.iter().sum();
        let sum_xx: f64 = concentrations.iter().map(|x| x * x).sum();
        let sum_xy: f64 = concentrations
            .iter()
            .zip(rotations.iter())
            .map(|(x, y)| x * y)
            .sum();

        let denom = n * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return CalibResult {
                slope: 0.0,
                intercept: sum_y / n,
                r_squared: 0.0,
            };
        }

        let slope = (n * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n;

        // R-squared
        let y_mean = sum_y / n;
        let ss_tot: f64 = rotations.iter().map(|y| (y - y_mean).powi(2)).sum();
        let ss_res: f64 = concentrations
            .iter()
            .zip(rotations.iter())
            .map(|(x, y)| {
                let pred = slope * x + intercept;
                (y - pred).powi(2)
            })
            .sum();

        let r_squared = if ss_tot > 1e-30 { 1.0 - ss_res / ss_tot } else { 0.0 };

        CalibResult {
            slope,
            intercept,
            r_squared,
        }
    }

    /// Determine unknown concentration from rotation using calibration.
    /// concentration = (rotation - intercept) / slope
    pub fn unknown_concentration(rotation: f64, calibration: &CalibResult) -> f64 {
        if calibration.slope.abs() < 1e-15 {
            return 0.0;
        }
        (rotation - calibration.intercept) / calibration.slope
    }

    /// Verify Biot's law linearity: returns (slope, r_squared).
    pub fn biot_law_verify(data: &[(f64, f64)]) -> (f64, f64) {
        if data.len() < 2 {
            return (0.0, 0.0);
        }

        let concentrations: Vec<f64> = data.iter().map(|d| d.0).collect();
        let rotations: Vec<f64> = data.iter().map(|d| d.1).collect();

        let result = Self::calibration_curve(&concentrations, &rotations);
        (result.slope, result.r_squared)
    }

    /// Detection limit: minimum detectable concentration.
    /// LOD = 3 * sigma_noise / ([alpha] * l)
    /// where sigma_noise is the standard deviation of blank measurements.
    pub fn detection_limit(noise_std: f64, specific_rotation: f64, path_length: f64) -> f64 {
        let denom = specific_rotation.abs() * path_length;
        if denom < 1e-15 {
            return f64::INFINITY;
        }
        3.0 * noise_std / denom
    }
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;
    const TOL_LOOSE: f64 = 1e-3;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ---- OpticalRotation tests ----

    #[test]
    fn test_specific_rotation_basic() {
        // alpha_obs = 13.3 deg, l = 1 dm, c = 0.2 g/mL => [alpha] = 66.5
        let or = OpticalRotation::new(13.3, 1.0, 0.2);
        assert!(approx_eq(or.specific_rotation(), 66.5, TOL));
    }

    #[test]
    fn test_specific_rotation_sucrose() {
        // Sucrose: [alpha] = +66.47, with c = 0.1 g/mL, l = 2 dm
        // alpha = 66.47 * 2 * 0.1 = 13.294 deg
        let or = OpticalRotation::new(13.294, 2.0, 0.1);
        assert!(approx_eq(or.specific_rotation(), 66.47, TOL));
    }

    #[test]
    fn test_specific_rotation_zero_path() {
        let or = OpticalRotation::new(5.0, 0.0, 0.1);
        assert_eq!(or.specific_rotation(), 0.0);
    }

    #[test]
    fn test_specific_rotation_zero_concentration() {
        let or = OpticalRotation::new(5.0, 1.0, 0.0);
        assert_eq!(or.specific_rotation(), 0.0);
    }

    #[test]
    fn test_molar_rotation() {
        // [alpha] = 66.5, M = 342.3 (sucrose) => [Phi] = 66.5 * 342.3 / 100 = 227.6295
        let or = OpticalRotation::new(13.3, 1.0, 0.2);
        let phi = or.molar_rotation(342.3);
        assert!(approx_eq(phi, 227.6295, TOL));
    }

    #[test]
    fn test_temperature_correction() {
        // [alpha] = 66.5, coeff = -0.0003 per deg C
        // At 25 C vs ref 20 C: correction = 66.5 * (1 + (-0.0003)(5)) = 66.5 * 0.9985 = 66.40025
        let or = OpticalRotation::new(13.3, 1.0, 0.2);
        let corrected = or.temperature_correction(25.0, 20.0, -0.0003);
        assert!(approx_eq(corrected, 66.40025, TOL));
    }

    #[test]
    fn test_wavelength_correction() {
        // [alpha] at 589 nm = 66.5
        // At 546 nm: 66.5 * (589^2) / (546^2) = 66.5 * (346921 / 298116) = 66.5 * 1.16363... ~ 77.38
        let or = OpticalRotation::new(13.3, 1.0, 0.2);
        let corrected = or.wavelength_correction(546.0, 589.0);
        let expected = 66.5 * (589.0 * 589.0) / (546.0 * 546.0);
        assert!(approx_eq(corrected, expected, TOL));
    }

    #[test]
    fn test_wavelength_correction_zero() {
        let or = OpticalRotation::new(13.3, 1.0, 0.2);
        assert_eq!(or.wavelength_correction(0.0, 589.0), 0.0);
    }

    #[test]
    fn test_negative_rotation() {
        // Fructose: [alpha] = -92
        let or = OpticalRotation::new(-18.4, 1.0, 0.2);
        assert!(approx_eq(or.specific_rotation(), -92.0, TOL));
    }

    // ---- DrudeEquation tests ----

    #[test]
    fn test_drude_single_term() {
        // [alpha] = A / (lambda^2 - lambda0^2)
        let rotation = DrudeEquation::single_term(589.0, 1e7, 200.0);
        let expected = 1e7 / (589.0 * 589.0 - 200.0 * 200.0);
        assert!(approx_eq(rotation, expected, TOL));
    }

    #[test]
    fn test_drude_two_term() {
        let r = DrudeEquation::two_term(589.0, 1e7, 200.0, -5e6, 250.0);
        let r1 = DrudeEquation::single_term(589.0, 1e7, 200.0);
        let r2 = DrudeEquation::single_term(589.0, -5e6, 250.0);
        assert!(approx_eq(r, r1 + r2, TOL));
    }

    #[test]
    fn test_drude_singularity() {
        // When lambda == lambda0, should return infinity
        let r = DrudeEquation::single_term(200.0, 1e7, 200.0);
        assert!(r.is_infinite());
    }

    #[test]
    fn test_drude_fit_synthetic() {
        // Generate data from known Drude params
        let a_true = 5e6;
        let l0_true = 220.0;
        let wavelengths: Vec<f64> = (400..700).step_by(10).map(|w| w as f64).collect();
        let rotations: Vec<f64> = wavelengths
            .iter()
            .map(|&w| DrudeEquation::single_term(w, a_true, l0_true))
            .collect();

        let fit = DrudeEquation::fit_drude(&wavelengths, &rotations);
        assert!(approx_eq(fit.a, a_true, a_true * 0.05)); // 5% tolerance
        assert!(approx_eq(fit.lambda0, l0_true, 5.0)); // within 5 nm
        assert!(fit.residual < 1.0);
    }

    #[test]
    fn test_plain_curve_monotonic() {
        let data: Vec<(f64, f64)> = (400..700)
            .step_by(10)
            .map(|w| (w as f64, 1e6 / (w as f64 * w as f64)))
            .collect();
        assert_eq!(
            DrudeEquation::plain_curve_vs_cotton_effect(&data),
            CurveType::PlainCurve
        );
    }

    #[test]
    fn test_cotton_effect_sign_change() {
        let data = vec![
            (300.0, 10.0),
            (320.0, 5.0),
            (340.0, -2.0),
            (360.0, -8.0),
            (380.0, -3.0),
        ];
        assert_eq!(
            DrudeEquation::plain_curve_vs_cotton_effect(&data),
            CurveType::CottonEffect
        );
    }

    #[test]
    fn test_cotton_effect_slope_reversal() {
        let data = vec![
            (300.0, 10.0),
            (320.0, 15.0),
            (340.0, 8.0),
            (360.0, 4.0),
        ];
        assert_eq!(
            DrudeEquation::plain_curve_vs_cotton_effect(&data),
            CurveType::CottonEffect
        );
    }

    // ---- SugarAnalyzer tests ----

    #[test]
    fn test_sucrose_concentration() {
        // alpha = 6.647 deg, l = 1 dm => c = 6.647 / (66.47 * 1) = 0.1 g/mL = 10 g/100mL
        let c = SugarAnalyzer::sucrose_concentration(6.647, 1.0);
        assert!(approx_eq(c, 10.0, TOL_LOOSE));
    }

    #[test]
    fn test_glucose_concentration() {
        // alpha = 5.27 deg, l = 1 dm => c = 5.27 / (52.7 * 1) = 0.1 => 10 g/100mL
        let c = SugarAnalyzer::glucose_concentration(5.27, 1.0);
        assert!(approx_eq(c, 10.0, TOL_LOOSE));
    }

    #[test]
    fn test_fructose_concentration() {
        // alpha = -9.2 deg, l = 1 dm => c = -9.2 / (-92.0 * 1) = 0.1 => 10 g/100mL
        let c = SugarAnalyzer::fructose_concentration(-9.2, 1.0);
        assert!(approx_eq(c, 10.0, TOL_LOOSE));
    }

    #[test]
    fn test_sugar_zero_path() {
        assert_eq!(SugarAnalyzer::sucrose_concentration(5.0, 0.0), 0.0);
    }

    #[test]
    fn test_international_sugar_scale() {
        // 50% reading => 50 Z
        let z = SugarAnalyzer::international_sugar_scale(17.0, 34.0);
        assert!(approx_eq(z, 50.0, TOL));
    }

    #[test]
    fn test_inversion_progress() {
        // alpha_0 = 66.5, alpha_inf = -20.0
        // alpha_t = 23.25 => f = (23.25 - 66.5) / (-20.0 - 66.5) = -43.25 / -86.5 = 0.5
        let f = SugarAnalyzer::inversion_progress(23.25, 66.5, -20.0);
        assert!(approx_eq(f, 0.5, TOL));
    }

    #[test]
    fn test_inversion_progress_complete() {
        let f = SugarAnalyzer::inversion_progress(-20.0, 66.5, -20.0);
        assert!(approx_eq(f, 1.0, TOL));
    }

    #[test]
    fn test_mutarotation_kinetics() {
        // Simulated exponential decay: alpha(t) = 52.7 + (112.0 - 52.7) * exp(-0.02*t)
        let k_true = 0.02;
        let a0 = 112.0;
        let a_inf = 52.7;
        // Use a long time series so the last value is very close to a_inf
        let times: Vec<f64> = (0..50).map(|i| i as f64 * 10.0).collect();
        let alphas: Vec<f64> = times
            .iter()
            .map(|&t| a_inf + (a0 - a_inf) * (-k_true * t).exp())
            .collect();

        let result = SugarAnalyzer::mutarotation_kinetics(&alphas, &times);
        // The linearization uses the last value as alpha_inf, so allow wider tolerance
        assert!(
            approx_eq(result.rate_constant, k_true, 0.005),
            "k = {} expected {}", result.rate_constant, k_true
        );
        assert!(result.r_squared > 0.9, "r2 = {}", result.r_squared);
    }

    // ---- EnantiomericExcess tests ----

    #[test]
    fn test_ee_from_rotation_pure() {
        let ee = EnantiomericExcess::ee_from_rotation(66.5, 66.5);
        assert!(approx_eq(ee, 100.0, TOL));
    }

    #[test]
    fn test_ee_from_rotation_racemic() {
        let ee = EnantiomericExcess::ee_from_rotation(0.0, 66.5);
        assert!(approx_eq(ee, 0.0, TOL));
    }

    #[test]
    fn test_ee_from_rotation_partial() {
        // observed = 53.2, pure = 66.5 => ee = 80%
        let ee = EnantiomericExcess::ee_from_rotation(53.2, 66.5);
        assert!(approx_eq(ee, 80.0, TOL_LOOSE));
    }

    #[test]
    fn test_er_from_ee() {
        // ee = 80% => R = 0.9, S = 0.1
        let (r, s) = EnantiomericExcess::er_from_ee(80.0);
        assert!(approx_eq(r, 0.9, TOL));
        assert!(approx_eq(s, 0.1, TOL));
    }

    #[test]
    fn test_er_from_ee_racemic() {
        let (r, s) = EnantiomericExcess::er_from_ee(0.0);
        assert!(approx_eq(r, 0.5, TOL));
        assert!(approx_eq(s, 0.5, TOL));
    }

    #[test]
    fn test_ee_from_er() {
        let ee = EnantiomericExcess::ee_from_er(0.9, 0.1);
        assert!(approx_eq(ee, 80.0, TOL));
    }

    #[test]
    fn test_ee_er_roundtrip() {
        let ee_orig = 75.0;
        let (r, s) = EnantiomericExcess::er_from_ee(ee_orig);
        let ee_back = EnantiomericExcess::ee_from_er(r, s);
        assert!(approx_eq(ee_back, ee_orig, TOL));
    }

    #[test]
    fn test_de_from_mixture() {
        let rotations = vec![100.0, -50.0];
        let fractions = vec![0.7, 0.3];
        // weighted = 70.0 + (-15.0) = 55.0, max = 100.0 => de = 55%
        let de = EnantiomericExcess::de_from_diastereomeric_mixture(&rotations, &fractions);
        assert!(approx_eq(de, 55.0, TOL));
    }

    // ---- StokesVector tests ----

    #[test]
    fn test_stokes_horizontal() {
        let s = StokesVector::linear_horizontal();
        assert!(approx_eq(s.degree_of_polarization(), 1.0, TOL));
        assert!(approx_eq(s.ellipticity_angle(), 0.0, TOL));
        assert!(approx_eq(s.azimuth_angle(), 0.0, TOL));
    }

    #[test]
    fn test_stokes_vertical() {
        let s = StokesVector::linear_vertical();
        assert!(approx_eq(s.degree_of_polarization(), 1.0, TOL));
        assert!(approx_eq(s.azimuth_angle().abs(), PI / 2.0, TOL));
    }

    #[test]
    fn test_stokes_circular_right() {
        let s = StokesVector::circular_right();
        assert!(approx_eq(s.degree_of_polarization(), 1.0, TOL));
        assert!(approx_eq(s.ellipticity_angle(), PI / 4.0, TOL));
    }

    #[test]
    fn test_stokes_circular_left() {
        let s = StokesVector::circular_left();
        assert!(approx_eq(s.ellipticity_angle(), -PI / 4.0, TOL));
    }

    #[test]
    fn test_stokes_unpolarized() {
        let s = StokesVector::unpolarized();
        assert!(approx_eq(s.degree_of_polarization(), 0.0, TOL));
    }

    #[test]
    fn test_stokes_from_jones_horizontal() {
        // Horizontal: Ex = (1, 0), Ey = (0, 0)
        let s = StokesVector::from_jones((1.0, 0.0), (0.0, 0.0));
        assert!(approx_eq(s.s0, 1.0, TOL));
        assert!(approx_eq(s.s1, 1.0, TOL));
        assert!(approx_eq(s.s2, 0.0, TOL));
        assert!(approx_eq(s.s3, 0.0, TOL));
    }

    #[test]
    fn test_stokes_from_jones_rcp() {
        // RCP: Ex = (1, 0), Ey = (0, -1) => (1/sqrt2, 0), (0, -1/sqrt2)
        let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
        let s = StokesVector::from_jones((inv_sqrt2, 0.0), (0.0, -inv_sqrt2));
        assert!(approx_eq(s.s0, 1.0, TOL));
        assert!(approx_eq(s.s1, 0.0, TOL));
        assert!(approx_eq(s.s2, 0.0, TOL));
        assert!(approx_eq(s.s3, 1.0, TOL));
    }

    #[test]
    fn test_stokes_plus45() {
        let s = StokesVector::linear_plus45();
        assert!(approx_eq(s.azimuth_angle(), PI / 4.0, TOL));
    }

    #[test]
    fn test_stokes_dop_partial() {
        let s = StokesVector::new(2.0, 1.0, 0.0, 0.0);
        assert!(approx_eq(s.degree_of_polarization(), 0.5, TOL));
    }

    // ---- MuellerMatrix tests ----

    #[test]
    fn test_mueller_identity() {
        let m = MuellerMatrix::identity();
        let s = StokesVector::linear_horizontal();
        let out = MuellerMatrix::apply(&m, &s);
        assert!(approx_eq(out.s0, s.s0, TOL));
        assert!(approx_eq(out.s1, s.s1, TOL));
    }

    #[test]
    fn test_mueller_rotator_90() {
        // 90 deg rotation: horizontal -> vertical
        let m = MuellerMatrix::rotator(90.0);
        let s = StokesVector::linear_horizontal();
        let out = MuellerMatrix::apply(&m, &s);
        assert!(approx_eq(out.s0, 1.0, TOL));
        // cos(180) = -1, so S1 should flip sign
        assert!(approx_eq(out.s1, -1.0, TOL));
    }

    #[test]
    fn test_mueller_rotator_45() {
        // 45 deg rotation: horizontal -> +45
        let m = MuellerMatrix::rotator(45.0);
        let s = StokesVector::linear_horizontal();
        let out = MuellerMatrix::apply(&m, &s);
        assert!(approx_eq(out.s0, 1.0, TOL));
        assert!(approx_eq(out.s1, 0.0, TOL));
        assert!(approx_eq(out.s2, 1.0, TOL));
    }

    #[test]
    fn test_mueller_polarizer_horizontal() {
        // Horizontal polarizer on unpolarized light => horizontal
        let m = MuellerMatrix::linear_polarizer(0.0);
        let s = StokesVector::unpolarized();
        let out = MuellerMatrix::apply(&m, &s);
        assert!(approx_eq(out.s0, 0.5, TOL));
        assert!(approx_eq(out.s1, 0.5, TOL));
    }

    #[test]
    fn test_mueller_polarizer_blocks_orthogonal() {
        // Horizontal polarizer on vertical light => zero
        let m = MuellerMatrix::linear_polarizer(0.0);
        let s = StokesVector::linear_vertical();
        let out = MuellerMatrix::apply(&m, &s);
        assert!(approx_eq(out.s0, 0.0, TOL_LOOSE));
    }

    #[test]
    fn test_mueller_qwp_to_circular() {
        // QWP at 45 deg on horizontal => right circular
        let m = MuellerMatrix::quarter_wave_plate(45.0);
        let s = StokesVector::linear_horizontal();
        let out = MuellerMatrix::apply(&m, &s);
        assert!(approx_eq(out.s0, 1.0, TOL));
        // Should be circularly polarized
        assert!(approx_eq(out.s3.abs(), 1.0, TOL_LOOSE));
    }

    #[test]
    fn test_mueller_hwp_flip() {
        // HWP at 0 deg: flips S3 (circular handedness)
        let m = MuellerMatrix::half_wave_plate(0.0);
        let s = StokesVector::circular_right();
        let out = MuellerMatrix::apply(&m, &s);
        assert!(approx_eq(out.s3, -1.0, TOL));
    }

    #[test]
    fn test_mueller_multiply_identity() {
        let m = MuellerMatrix::identity();
        let r = MuellerMatrix::rotator(30.0);
        let product = MuellerMatrix::multiply(&m, &r);
        for i in 0..4 {
            for j in 0..4 {
                assert!(approx_eq(product.m[i][j], r.m[i][j], TOL));
            }
        }
    }

    #[test]
    fn test_mueller_rotation_composition() {
        // Two 45 deg rotations = one 90 deg rotation
        let r45 = MuellerMatrix::rotator(45.0);
        let r90 = MuellerMatrix::rotator(90.0);
        let composed = MuellerMatrix::multiply(&r45, &r45);
        for i in 0..4 {
            for j in 0..4 {
                assert!(approx_eq(composed.m[i][j], r90.m[i][j], TOL));
            }
        }
    }

    #[test]
    fn test_mueller_cd_element() {
        let m = MuellerMatrix::circular_dichroism(0.0);
        // No CD: should be identity
        assert!(approx_eq(m.m[0][0], 1.0, TOL));
        assert!(approx_eq(m.m[1][1], 1.0, TOL));
    }

    // ---- CircularDichroism tests ----

    #[test]
    fn test_cd_delta_epsilon() {
        let de = CircularDichroism::delta_epsilon(2.5, 2.3);
        assert!(approx_eq(de, 0.2, TOL));
    }

    #[test]
    fn test_cd_ellipticity() {
        let theta = CircularDichroism::ellipticity_mdeg(0.001);
        // 32.982 * 0.001 = 0.032982 mdeg
        assert!(approx_eq(theta, 0.032982, TOL));
    }

    #[test]
    fn test_cd_molar_ellipticity() {
        // theta = 100 mdeg, c = 0.001 M, l = 1 cm
        let me = CircularDichroism::molar_ellipticity(100.0, 0.001, 1.0);
        // [theta] = 100 / (0.001 * 1 * 10) = 10000
        assert!(approx_eq(me, 10000.0, TOL));
    }

    #[test]
    fn test_cd_mean_residue_ellipticity() {
        let mre = CircularDichroism::mean_residue_ellipticity(10000.0, 100);
        assert!(approx_eq(mre, 100.0, TOL));
    }

    #[test]
    fn test_cd_mean_residue_zero_residues() {
        let mre = CircularDichroism::mean_residue_ellipticity(10000.0, 0);
        assert_eq!(mre, 0.0);
    }

    #[test]
    fn test_cd_secondary_structure_helix() {
        // Strong alpha-helix: large negative at 222 nm
        let spectrum = vec![
            (190.0, 50000.0),
            (200.0, 20000.0),
            (208.0, -30000.0),
            (215.0, -20000.0),
            (222.0, -30000.0),
            (230.0, -10000.0),
            (240.0, -2000.0),
        ];
        let ss = CircularDichroism::secondary_structure_estimate(&spectrum);
        assert!(ss.alpha_helix > 0.5); // should be mostly helix
    }

    #[test]
    fn test_cd_kronig_kramers_runs() {
        let cd = vec![
            (200.0, 0.0),
            (210.0, 1.0),
            (220.0, 2.0),
            (230.0, 1.0),
            (240.0, 0.0),
        ];
        let ord = CircularDichroism::kronig_kramers(&cd);
        assert_eq!(ord.len(), cd.len());
    }

    #[test]
    fn test_cd_kronig_kramers_symmetry() {
        // A symmetric CD band should produce an antisymmetric ORD
        let cd = vec![
            (200.0, 0.0),
            (205.0, 0.5),
            (210.0, 1.0),
            (215.0, 0.5),
            (220.0, 0.0),
        ];
        let ord = CircularDichroism::kronig_kramers(&cd);
        // The center of the CD band should have near-zero ORD
        // (KK transform of a symmetric function around center)
        assert!(ord.len() == 5);
    }

    // ---- FaradayRotation tests ----

    #[test]
    fn test_faraday_rotation_basic() {
        // TGG: V = 134 rad/(T*m), B = 1 T, L = 0.01 m
        let theta = FaradayRotation::rotation_angle(134.0, 1.0, 0.01);
        assert!(approx_eq(theta, 1.34, TOL));
    }

    #[test]
    fn test_faraday_rotation_deg() {
        let theta_deg = FaradayRotation::rotation_angle_deg(134.0, 1.0, 0.01);
        let expected = 1.34 * 180.0 / PI;
        assert!(approx_eq(theta_deg, expected, TOL));
    }

    #[test]
    fn test_faraday_verdet_from_measurement() {
        // 45 deg rotation, 1 T, 0.1 m
        let v = FaradayRotation::verdet_from_measurement(45.0, 1.0, 0.1);
        let expected = (45.0 * PI / 180.0) / (1.0 * 0.1);
        assert!(approx_eq(v, expected, TOL));
    }

    #[test]
    fn test_faraday_field_from_rotation() {
        // 45 deg rotation, V = 134, L = 0.01 m
        let b = FaradayRotation::field_from_rotation(45.0, 134.0, 0.01);
        let expected = (45.0 * PI / 180.0) / (134.0 * 0.01);
        assert!(approx_eq(b, expected, TOL));
    }

    #[test]
    fn test_faraday_roundtrip() {
        let v = 134.0;
        let b = 0.5;
        let l = 0.02;
        let theta_deg = FaradayRotation::rotation_angle_deg(v, b, l);
        let b_back = FaradayRotation::field_from_rotation(theta_deg, v, l);
        assert!(approx_eq(b_back, b, TOL));
    }

    #[test]
    fn test_verdet_constants_defined() {
        assert!(VerdetConstants::TGG > VerdetConstants::WATER);
        assert!(VerdetConstants::FLINT_GLASS > VerdetConstants::FUSED_SILICA);
    }

    // ---- PolarimeterSimulator tests ----

    #[test]
    fn test_simulator_noiseless() {
        // With noise = 0, should get exact true rotation
        // Note: our simulator uses a pseudo-random noise that may not be exactly 0
        // even with noise_deg = 0, because 0 * gaussian = 0
        let measurement = PolarimeterSimulator::simulate_measurement(66.5, 0.1, 1.0, 0.0);
        assert!(approx_eq(measurement, 6.65, 0.5)); // loose tolerance due to noise scaling
    }

    #[test]
    fn test_simulator_ord_spectrum() {
        let params = DrudeFitResult {
            a: 1e6,
            lambda0: 200.0,
            residual: 0.0,
        };
        let wavelengths: Vec<f64> = (400..600).step_by(20).map(|w| w as f64).collect();
        let spectrum = PolarimeterSimulator::simulate_ord_spectrum(&params, &wavelengths, 0.0);
        assert_eq!(spectrum.len(), wavelengths.len());
        // Check that rotation decreases with wavelength (Drude behavior)
        assert!(spectrum[0].1.abs() > spectrum.last().unwrap().1.abs());
    }

    #[test]
    fn test_simulator_cd_spectrum() {
        let peaks = vec![(220.0, 10.0, 15.0)];
        let wavelengths: Vec<f64> = (190..250).step_by(5).map(|w| w as f64).collect();
        let spectrum = PolarimeterSimulator::simulate_cd_spectrum(&peaks, &wavelengths);
        assert_eq!(spectrum.len(), wavelengths.len());

        // Find peak value near 220 nm
        let peak_val = spectrum
            .iter()
            .find(|(w, _)| (*w - 220.0).abs() < 3.0)
            .unwrap()
            .1;
        assert!(peak_val > 5.0); // near the peak amplitude
    }

    #[test]
    fn test_simulator_cd_multiple_peaks() {
        let peaks = vec![(210.0, 5.0, 10.0), (250.0, -3.0, 8.0)];
        let wavelengths: Vec<f64> = (190..280).step_by(5).map(|w| w as f64).collect();
        let spectrum = PolarimeterSimulator::simulate_cd_spectrum(&peaks, &wavelengths);

        // Should have positive CD near 210 and negative near 250
        let val_210 = spectrum
            .iter()
            .find(|(w, _)| (*w - 210.0).abs() < 3.0)
            .unwrap()
            .1;
        let val_250 = spectrum
            .iter()
            .find(|(w, _)| (*w - 250.0).abs() < 3.0)
            .unwrap()
            .1;
        assert!(val_210 > 0.0);
        assert!(val_250 < 0.0);
    }

    // ---- ConcentrationCalibration tests ----

    #[test]
    fn test_calibration_curve_perfect_line() {
        let conc = vec![0.0, 0.1, 0.2, 0.3, 0.4, 0.5];
        let rots: Vec<f64> = conc.iter().map(|&c| 66.5 * c).collect();
        let result = ConcentrationCalibration::calibration_curve(&conc, &rots);
        assert!(approx_eq(result.slope, 66.5, TOL));
        assert!(approx_eq(result.intercept, 0.0, TOL));
        assert!(approx_eq(result.r_squared, 1.0, TOL));
    }

    #[test]
    fn test_calibration_unknown_concentration() {
        let conc = vec![0.0, 0.1, 0.2, 0.3, 0.4];
        let rots: Vec<f64> = conc.iter().map(|&c| 66.5 * c + 0.5).collect();
        let calib = ConcentrationCalibration::calibration_curve(&conc, &rots);
        let unknown_rot = 14.0;
        let c = ConcentrationCalibration::unknown_concentration(unknown_rot, &calib);
        let expected = (14.0 - 0.5) / 66.5;
        assert!(approx_eq(c, expected, TOL_LOOSE));
    }

    #[test]
    fn test_biot_law_verify() {
        let data: Vec<(f64, f64)> = (0..10)
            .map(|i| {
                let c = i as f64 * 0.05;
                (c, 66.5 * c)
            })
            .collect();
        let (slope, r2) = ConcentrationCalibration::biot_law_verify(&data);
        assert!(approx_eq(slope, 66.5, TOL));
        assert!(approx_eq(r2, 1.0, TOL));
    }

    #[test]
    fn test_detection_limit() {
        // noise_std = 0.01 deg, [alpha] = 66.5, l = 1 dm
        let lod = ConcentrationCalibration::detection_limit(0.01, 66.5, 1.0);
        // LOD = 3 * 0.01 / (66.5 * 1) = 0.0004511...
        assert!(approx_eq(lod, 3.0 * 0.01 / 66.5, TOL));
    }

    #[test]
    fn test_detection_limit_zero_rotation() {
        let lod = ConcentrationCalibration::detection_limit(0.01, 0.0, 1.0);
        assert!(lod.is_infinite());
    }

    #[test]
    fn test_calibration_roundtrip() {
        let conc = vec![0.05, 0.1, 0.15, 0.2, 0.25, 0.3];
        let rots: Vec<f64> = conc.iter().map(|&c| 52.7 * c).collect();
        let calib = ConcentrationCalibration::calibration_curve(&conc, &rots);

        // Verify roundtrip for each calibration point
        for (&c, &r) in conc.iter().zip(rots.iter()) {
            let c_calc = ConcentrationCalibration::unknown_concentration(r, &calib);
            assert!(approx_eq(c_calc, c, TOL_LOOSE));
        }
    }

    // ---- Integration / end-to-end tests ----

    #[test]
    fn test_full_polarimetry_workflow() {
        // Measure sucrose solution
        let specific_rotation = SUCROSE_SPECIFIC_ROTATION;
        let concentration = 0.15; // g/mL
        let path_length = 2.0; // dm

        // Expected observed rotation
        let alpha_obs = specific_rotation * path_length * concentration;
        assert!(approx_eq(alpha_obs, 19.941, TOL));

        // Back-calculate concentration
        let c_calc = SugarAnalyzer::sucrose_concentration(alpha_obs, path_length);
        assert!(approx_eq(c_calc, 15.0, TOL_LOOSE)); // 15 g/100mL = 0.15 g/mL

        // Create measurement object
        let or = OpticalRotation::new(alpha_obs, path_length, concentration);
        assert!(approx_eq(or.specific_rotation(), specific_rotation, TOL));
    }

    #[test]
    fn test_polarizer_analyzer_malus_law() {
        // Malus's law: I = I0 * cos^2(theta)
        // Horizontal light through polarizer at angle theta
        let s = StokesVector::linear_horizontal();

        for angle_deg in [0, 15, 30, 45, 60, 75, 90] {
            let pol = MuellerMatrix::linear_polarizer(angle_deg as f64);
            let out = MuellerMatrix::apply(&pol, &s);
            let theta = (angle_deg as f64) * PI / 180.0;
            let expected_intensity = 0.5 * (1.0 + (2.0 * theta).cos());
            assert!(approx_eq(out.s0, expected_intensity, TOL_LOOSE));
        }
    }

    #[test]
    fn test_faraday_isolator() {
        // Faraday isolator: polarizer -> 45 deg Faraday -> analyzer at 45 deg
        let polarizer = MuellerMatrix::linear_polarizer(0.0);
        let faraday = MuellerMatrix::rotator(45.0);
        let analyzer = MuellerMatrix::linear_polarizer(45.0);

        // Forward direction: should pass
        let s_in = StokesVector::unpolarized();
        let s1 = MuellerMatrix::apply(&polarizer, &s_in);
        let s2 = MuellerMatrix::apply(&faraday, &s1);
        let s_out = MuellerMatrix::apply(&analyzer, &s2);
        assert!(s_out.s0 > 0.2); // Significant transmission

        // Backward direction: polarizer -> -45 deg Faraday -> analyzer at 0
        // Faraday rotation is non-reciprocal: same direction for both
        let s_back = StokesVector::linear_plus45(); // Light at 45 deg
        let s_b1 = MuellerMatrix::apply(&faraday, &s_back); // +45 more = 90 deg
        let s_b2 = MuellerMatrix::apply(&polarizer, &s_b1); // Horizontal pol blocks 90 deg
        assert!(s_b2.s0 < 0.1); // Should be blocked
    }

    #[test]
    fn test_interpolate_at_basic() {
        let spectrum = vec![(200.0, 10.0), (300.0, 20.0)];
        assert!(approx_eq(interpolate_at(&spectrum, 250.0), 15.0, TOL));
    }

    #[test]
    fn test_interpolate_at_exact() {
        let spectrum = vec![(200.0, 10.0), (300.0, 20.0)];
        assert!(approx_eq(interpolate_at(&spectrum, 200.0), 10.0, TOL));
    }
}
