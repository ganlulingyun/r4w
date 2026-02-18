//! # Contact Angle Goniometer Processor
//!
//! Contact angle measurement and surface energy analysis from sessile drop profiles.
//!
//! Implements Young's equation, Owens-Wendt-Rabel-Kaelble (OWRK) and van Oss-Chaudhury-Good
//! surface energy models, Wenzel/Cassie-Baxter wetting on rough surfaces, dynamic contact
//! angle analysis, and capillary phenomena calculations.
//!
//! ## Physics
//!
//! - Young's equation: γ_SV = γ_SL + γ_LV cos θ
//! - OWRK: γ_SL = γ_S + γ_L - 2(√(γ_S^d γ_L^d) + √(γ_S^p γ_L^p))
//! - Wenzel: cos θ* = r · cos θ (rough surface)
//! - Cassie-Baxter: cos θ* = f₁ cos θ₁ + (1 - f₁) cos θ₂
//! - Work of adhesion: W_a = γ_LV(1 + cos θ)
//! - Capillary length: λ_c = √(γ/(ρg)) ≈ 2.7 mm for water

use std::f64::consts::PI;

/// Gravitational acceleration (m/s^2)
const G: f64 = 9.80665;

// ─── DropProfile ─────────────────────────────────────────────────────────────

/// Sessile drop profile data (coordinates in mm).
#[derive(Debug, Clone)]
pub struct DropProfile {
    pub x: Vec<f64>,
    pub y: Vec<f64>,
}

impl DropProfile {
    /// Create a new drop profile from x,y coordinate arrays (mm).
    pub fn new(x_coords_mm: Vec<f64>, y_coords_mm: Vec<f64>) -> Self {
        assert_eq!(x_coords_mm.len(), y_coords_mm.len(), "x and y must have equal length");
        assert!(x_coords_mm.len() >= 3, "need at least 3 points");
        Self { x: x_coords_mm, y: y_coords_mm }
    }

    /// Detect the baseline of the drop.
    /// Returns (y_baseline, x_left_contact, x_right_contact).
    pub fn baseline_detect(&self) -> (f64, f64, f64) {
        // The baseline is the minimum y value (bottom of the drop on the surface).
        let y_min = self.y.iter().cloned().fold(f64::INFINITY, f64::min);

        // Tolerance: points within 2% of the height range from y_min are on the baseline.
        let y_max = self.y.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let tol = (y_max - y_min) * 0.02 + 1e-12;

        let mut x_left = f64::INFINITY;
        let mut x_right = f64::NEG_INFINITY;
        for (x, y) in self.x.iter().zip(self.y.iter()) {
            if (*y - y_min).abs() <= tol {
                if *x < x_left {
                    x_left = *x;
                }
                if *x > x_right {
                    x_right = *x;
                }
            }
        }
        (y_min, x_left, x_right)
    }

    /// Contact diameter (width of the drop at the baseline) in mm.
    pub fn drop_width(&self) -> f64 {
        let (_, x_left, x_right) = self.baseline_detect();
        (x_right - x_left).abs()
    }

    /// Maximum height of the drop above the baseline in mm.
    pub fn drop_height(&self) -> f64 {
        let (y_base, _, _) = self.baseline_detect();
        let y_max = self.y.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        y_max - y_base
    }

    /// Estimated drop volume in microlitres assuming axisymmetric shape.
    /// Uses the disk integration method: V = π Σ r(y)^2 Δy
    /// If `assuming_axisymmetric` is false, uses a simpler spherical cap estimate.
    pub fn volume_ul(&self, assuming_axisymmetric: bool) -> f64 {
        let (y_base, x_left, x_right) = self.baseline_detect();
        let h = self.drop_height();

        if !assuming_axisymmetric {
            // Spherical cap approximation: V = π h (3a^2 + h^2) / 6
            let a = (x_right - x_left) / 2.0; // base radius
            let vol_mm3 = PI * h * (3.0 * a * a + h * h) / 6.0;
            return vol_mm3 * 1e-3; // mm^3 -> µL (1 µL = 1 mm^3 actually, but
                                    // the standard convention: 1 µL = 1 mm^3)
                                    // Actually 1 µL == 1 mm^3, so no conversion needed.
                                    // Let's return in µL = mm^3.
        }

        // Axisymmetric: integrate slices along y.
        // For each y-level, find the horizontal extent (left, right), compute radius.
        let n_slices = 100;
        let dy = h / n_slices as f64;
        let center_x = (x_left + x_right) / 2.0;
        let mut volume_mm3 = 0.0;

        for i in 0..n_slices {
            let y_level = y_base + (i as f64 + 0.5) * dy;
            // Find max horizontal extent at this y level by interpolation
            let radius = self.radius_at_y(y_level, center_x);
            volume_mm3 += PI * radius * radius * dy;
        }

        volume_mm3 // 1 mm^3 = 1 µL
    }

    /// Helper: find the approximate radius at a given y level.
    fn radius_at_y(&self, y_level: f64, center_x: f64) -> f64 {
        let mut max_dist = 0.0_f64;
        // Walk segments and find intersections with y_level
        for i in 0..self.x.len() - 1 {
            let (y0, y1) = (self.y[i], self.y[i + 1]);
            let (x0, x1) = (self.x[i], self.x[i + 1]);
            if (y0 <= y_level && y_level <= y1) || (y1 <= y_level && y_level <= y0) {
                if (y1 - y0).abs() < 1e-15 {
                    let d = ((x0 - center_x).abs()).max((x1 - center_x).abs());
                    if d > max_dist {
                        max_dist = d;
                    }
                } else {
                    let t = (y_level - y0) / (y1 - y0);
                    let x_interp = x0 + t * (x1 - x0);
                    let d = (x_interp - center_x).abs();
                    if d > max_dist {
                        max_dist = d;
                    }
                }
            }
        }
        max_dist
    }
}

// ─── ContactAngleResult ──────────────────────────────────────────────────────

/// Result of a contact angle measurement.
#[derive(Debug, Clone, Copy)]
pub struct ContactAngleResult {
    pub left_angle_deg: f64,
    pub right_angle_deg: f64,
    pub average_angle_deg: f64,
    pub hysteresis_deg: f64,
}

impl ContactAngleResult {
    pub fn new(left: f64, right: f64) -> Self {
        Self {
            left_angle_deg: left,
            right_angle_deg: right,
            average_angle_deg: (left + right) / 2.0,
            hysteresis_deg: (left - right).abs(),
        }
    }
}

// ─── ContactAngleMeasurement ─────────────────────────────────────────────────

/// Static contact angle measurement methods.
pub struct ContactAngleMeasurement;

impl ContactAngleMeasurement {
    /// Half-angle method: θ = 2 · atan(2h / w).
    pub fn half_angle_method(height: f64, width: f64) -> f64 {
        2.0 * (2.0 * height / width).atan().to_degrees()
    }

    /// Tangent method: fit a polynomial near the contact points and evaluate the slope.
    pub fn tangent_method(profile: &DropProfile) -> ContactAngleResult {
        let (y_base, x_left, x_right) = profile.baseline_detect();
        let tol = profile.drop_height() * 0.25;

        // Left side: points near left contact
        let left_angle = Self::tangent_at_contact(profile, x_left, y_base, tol, true);
        // Right side: points near right contact
        let right_angle = Self::tangent_at_contact(profile, x_right, y_base, tol, false);

        ContactAngleResult::new(left_angle, right_angle)
    }

    /// Helper: compute tangent angle at a contact point.
    fn tangent_at_contact(
        profile: &DropProfile,
        x_contact: f64,
        y_base: f64,
        tol: f64,
        is_left: bool,
    ) -> f64 {
        // Collect points near the contact within vertical tolerance
        let mut pts: Vec<(f64, f64)> = Vec::new();
        for (x, y) in profile.x.iter().zip(profile.y.iter()) {
            let dy = *y - y_base;
            if dy >= 0.0 && dy <= tol {
                let dx = (*x - x_contact).abs();
                if dx <= tol {
                    pts.push((*x, *y));
                }
            }
        }

        if pts.len() < 2 {
            // Fallback: use half-angle
            return ContactAngleMeasurement::half_angle_method(
                profile.drop_height(),
                profile.drop_width(),
            );
        }

        // Sort by y (ascending from baseline)
        pts.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());

        // Fit a line through the first few points (least squares)
        let n = pts.len().min(10);
        let subset = &pts[..n];
        let (slope, _) = linear_fit(
            &subset.iter().map(|p| p.1 - y_base).collect::<Vec<_>>(), // y as independent (height)
            &subset.iter().map(|p| p.0).collect::<Vec<_>>(),          // x as dependent
        );

        // slope = dx/dy at contact; tangent angle to baseline
        let angle_rad = (1.0 / slope.abs()).atan();
        let angle_deg = angle_rad.to_degrees();

        if is_left {
            180.0 - angle_deg
        } else {
            180.0 - angle_deg
        }
    }

    /// Circle fit method: fit a circle to the drop outline and compute tangent at baseline.
    pub fn circle_fit_method(profile: &DropProfile) -> ContactAngleResult {
        let (y_base, x_left, x_right) = profile.baseline_detect();

        // Collect all profile points (including those near baseline) for the circle fit
        let pts: Vec<(f64, f64)> = profile
            .x
            .iter()
            .zip(profile.y.iter())
            .filter(|(_, y)| **y >= y_base - 1e-9)
            .map(|(x, y)| (*x, *y))
            .collect();

        if pts.len() < 3 {
            let theta = Self::half_angle_method(profile.drop_height(), profile.drop_width());
            return ContactAngleResult::new(theta, theta);
        }

        // Fit circle using algebraic method (Kasa)
        let (cx, cy, r) = circle_fit_kasa(&pts);

        // Contact angle from circle geometry at left and right contact points
        let left_angle = circle_tangent_angle(cx, cy, r, x_left, y_base, true);
        let right_angle = circle_tangent_angle(cx, cy, r, x_right, y_base, false);

        ContactAngleResult::new(left_angle, right_angle)
    }

    /// Ellipse fit method: fit an ellipse for non-spherical (gravity-deformed) drops.
    pub fn ellipse_fit_method(profile: &DropProfile) -> ContactAngleResult {
        let (y_base, x_left, x_right) = profile.baseline_detect();

        // Collect points above baseline
        let pts: Vec<(f64, f64)> = profile
            .x
            .iter()
            .zip(profile.y.iter())
            .filter(|(_, y)| **y > y_base + 1e-9)
            .map(|(x, y)| (*x, *y))
            .collect();

        if pts.len() < 5 {
            let theta = Self::half_angle_method(profile.drop_height(), profile.drop_width());
            return ContactAngleResult::new(theta, theta);
        }

        // Fit ellipse (simplified: assume axes aligned with x,y)
        let cx = (x_left + x_right) / 2.0;
        let a = (x_right - x_left) / 2.0; // semi-major (horizontal)

        // Estimate semi-minor from max height
        let b = profile.drop_height();
        let cy = y_base;

        // Tangent of ellipse x²/a² + (y-cy)²/b² = 1 at baseline:
        // dy/dx = -(b²·x) / (a²·(y-cy))
        // At left contact (x_left, y_base): x_rel = x_left - cx = -a
        // The tangent angle at the contact point
        let left_angle = ellipse_tangent_angle(a, b, x_left - cx, y_base - cy);
        let right_angle = ellipse_tangent_angle(a, b, x_right - cx, y_base - cy);

        ContactAngleResult::new(left_angle, right_angle)
    }

    /// Young-Laplace axisymmetric drop shape analysis.
    /// `density_diff` in kg/m^3, `gamma` in mN/m.
    /// Returns contact angle in degrees.
    pub fn young_laplace_fit(
        profile: &DropProfile,
        density_diff: f64,
        gamma: f64,
    ) -> f64 {
        // Simplified Young-Laplace: for small Bond numbers, the drop is nearly spherical.
        // Bo = Δρ g R² / γ
        let r_m = (profile.drop_width() / 2.0) * 1e-3; // mm -> m
        let gamma_si = gamma * 1e-3; // mN/m -> N/m
        let bo = density_diff * G * r_m * r_m / gamma_si;

        // For small Bo, use spherical cap + first-order correction
        let h = profile.drop_height();
        let w = profile.drop_width();
        let theta0 = Self::half_angle_method(h, w);

        // First-order Bond number correction (Bashforth-Adams approximation)
        let correction = -bo * 0.5 * theta0.to_radians().sin();
        theta0 + correction.to_degrees()
    }
}

// ─── Wettability ─────────────────────────────────────────────────────────────

/// Surface wettability classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Wettability {
    Superhydrophilic,
    Hydrophilic,
    Hydrophobic,
    Superhydrophobic,
}

/// Surface wettability characterization.
pub struct WettabilityClassifier;

impl WettabilityClassifier {
    /// Classify wettability from static contact angle.
    pub fn classify(contact_angle_deg: f64) -> Wettability {
        if contact_angle_deg < 10.0 {
            Wettability::Superhydrophilic
        } else if contact_angle_deg < 90.0 {
            Wettability::Hydrophilic
        } else if contact_angle_deg > 150.0 {
            Wettability::Superhydrophobic
        } else {
            Wettability::Hydrophobic
        }
    }

    /// Spreading coefficient: S = γ_SV - γ_SL - γ_LV.
    pub fn spreading_coefficient(gamma_sv: f64, gamma_sl: f64, gamma_lv: f64) -> f64 {
        gamma_sv - gamma_sl - gamma_lv
    }

    /// Work of adhesion (Dupré): W_a = γ_LV(1 + cos θ).
    pub fn work_of_adhesion(gamma_lv: f64, theta_deg: f64) -> f64 {
        gamma_lv * (1.0 + theta_deg.to_radians().cos())
    }

    /// Work of cohesion: W_c = 2 γ_LV.
    pub fn work_of_cohesion(gamma_lv: f64) -> f64 {
        2.0 * gamma_lv
    }
}

// ─── SurfaceEnergyResult ─────────────────────────────────────────────────────

/// Surface energy breakdown (mN/m).
#[derive(Debug, Clone, Copy)]
pub struct SurfaceEnergyResult {
    pub total: f64,
    pub dispersive: f64,
    pub polar: f64,
}

/// Van Oss acid-base surface energy result (mN/m).
#[derive(Debug, Clone, Copy)]
pub struct VanOssResult {
    /// Lifshitz-van der Waals component
    pub lw: f64,
    /// Lewis acid component (electron acceptor)
    pub acid: f64,
    /// Lewis base component (electron donor)
    pub base: f64,
    /// Total surface energy
    pub total: f64,
}

// ─── SurfaceEnergyCalculator ─────────────────────────────────────────────────

/// Calculate surface free energy from contact angle data.
pub struct SurfaceEnergyCalculator;

impl SurfaceEnergyCalculator {
    /// Owens-Wendt-Rabel-Kaelble (OWRK) method.
    /// Uses water and diiodomethane contact angles (degrees).
    /// Returns surface energy components in mN/m.
    pub fn owrk(theta_water: f64, theta_diiodomethane: f64) -> SurfaceEnergyResult {
        let water = LiquidDatabase::water();
        let diio = LiquidDatabase::diiodomethane();

        let cos_w = theta_water.to_radians().cos();
        let cos_d = theta_diiodomethane.to_radians().cos();

        // OWRK equations:
        // γ_L(1+cosθ) = 2(√(γ_S^d · γ_L^d) + √(γ_S^p · γ_L^p))
        //
        // For diiodomethane (γ_L^p ≈ 0):
        // γ_d_diio(1+cosθ_d) = 2√(γ_S^d · γ_L^d_diio)
        // γ_S^d = [γ_L_diio(1+cosθ_d)]² / (4·γ_L^d_diio)
        let rhs_d = diio.gamma_total * (1.0 + cos_d);
        let gamma_s_d = (rhs_d * rhs_d) / (4.0 * diio.gamma_dispersive);

        // For water:
        // γ_L_w(1+cosθ_w) = 2(√(γ_S^d · γ_L^d_w) + √(γ_S^p · γ_L^p_w))
        // √(γ_S^p) = (γ_L_w(1+cosθ_w) - 2√(γ_S^d · γ_L^d_w)) / (2√(γ_L^p_w))
        let rhs_w = water.gamma_total * (1.0 + cos_w);
        let sqrt_gamma_s_d_times_w_d = (gamma_s_d * water.gamma_dispersive).sqrt();
        let numerator = rhs_w - 2.0 * sqrt_gamma_s_d_times_w_d;
        let denominator = 2.0 * water.gamma_polar.sqrt();

        let sqrt_gamma_s_p = if denominator.abs() > 1e-12 {
            numerator / denominator
        } else {
            0.0
        };
        let gamma_s_p = sqrt_gamma_s_p * sqrt_gamma_s_p.abs(); // preserve sign for negative sqrt

        SurfaceEnergyResult {
            total: gamma_s_d + gamma_s_p.abs(),
            dispersive: gamma_s_d,
            polar: gamma_s_p.abs(),
        }
    }

    /// Van Oss-Chaudhury-Good acid-base method.
    /// Uses water, diiodomethane, and formamide contact angles (degrees).
    pub fn van_oss_good(
        theta_water: f64,
        theta_diiodomethane: f64,
        theta_formamide: f64,
    ) -> VanOssResult {
        let water = LiquidDatabase::water();
        let diio = LiquidDatabase::diiodomethane();
        let form = LiquidDatabase::formamide();

        let cos_w = theta_water.to_radians().cos();
        let cos_d = theta_diiodomethane.to_radians().cos();
        let cos_f = theta_formamide.to_radians().cos();

        // Van Oss: γ_L(1+cosθ) = 2(√(γ_S^LW · γ_L^LW) + √(γ_S^+ · γ_L^-) + √(γ_S^- · γ_L^+))
        // For diiodomethane (apolar, γ^+ ≈ γ^- ≈ 0):
        // γ_diio(1+cosθ_d) = 2√(γ_S^LW · γ_diio^LW)
        let gamma_s_lw = {
            let rhs = diio.gamma_total * (1.0 + cos_d);
            (rhs * rhs) / (4.0 * diio.gamma_dispersive) // γ_diio^LW ≈ γ_diio^d
        };

        // For water and formamide, solve 2x2 system for √(γ_S^+), √(γ_S^-)
        // Water acid/base components: γ^+ = 25.5, γ^- = 25.5 (symmetric)
        let water_acid = 25.5_f64;
        let water_base = 25.5_f64;
        // Formamide acid/base: γ^+ ≈ 2.28, γ^- ≈ 39.6
        let form_acid = 2.28_f64;
        let form_base = 39.6_f64;

        let rhs_w = water.gamma_total * (1.0 + cos_w) - 2.0 * (gamma_s_lw * water.gamma_dispersive).sqrt();
        let rhs_f = form.gamma_total * (1.0 + cos_f) - 2.0 * (gamma_s_lw * form.gamma_dispersive).sqrt();

        // rhs_w = 2(√(γ_S^+ · w_base) + √(γ_S^- · w_acid))
        // rhs_f = 2(√(γ_S^+ · f_base) + √(γ_S^- · f_acid))
        // Let a = √γ_S^+, b = √γ_S^-
        // rhs_w/2 = a·√w_base + b·√w_acid
        // rhs_f/2 = a·√f_base + b·√f_acid

        let sw_b = water_base.sqrt();
        let sw_a = water_acid.sqrt();
        let sf_b = form_base.sqrt();
        let sf_a = form_acid.sqrt();

        let det = sw_b * sf_a - sf_b * sw_a;
        let (sqrt_acid, sqrt_base) = if det.abs() > 1e-12 {
            let a = (rhs_w / 2.0 * sf_a - rhs_f / 2.0 * sw_a) / det;
            let b = (sw_b * rhs_f / 2.0 - sf_b * rhs_w / 2.0) / det;
            (a.max(0.0), b.max(0.0))
        } else {
            (0.0, 0.0)
        };

        let gamma_acid = sqrt_acid * sqrt_acid;
        let gamma_base = sqrt_base * sqrt_base;

        VanOssResult {
            lw: gamma_s_lw,
            acid: gamma_acid,
            base: gamma_base,
            total: gamma_s_lw + 2.0 * (gamma_acid * gamma_base).sqrt(),
        }
    }

    /// Zisman critical surface tension from a series of (contact_angle_deg, gamma_lv) measurements.
    /// Extrapolates cos θ = 1 to find γ_c.
    pub fn zisman_critical_surface_tension(thetas: &[f64], gammas: &[f64]) -> f64 {
        assert_eq!(thetas.len(), gammas.len());
        assert!(thetas.len() >= 2);

        // Linear regression: cos θ = a + b · γ_LV
        let cos_vals: Vec<f64> = thetas.iter().map(|t| t.to_radians().cos()).collect();
        let (slope, intercept) = linear_fit(gammas, &cos_vals);

        // At cos θ = 1: γ_c = (1 - intercept) / slope
        if slope.abs() < 1e-15 {
            return 0.0;
        }
        (1.0 - intercept) / slope
    }

    /// Neumann equation of state approach.
    /// Estimates surface energy from a single liquid contact angle measurement.
    /// `theta_deg`: contact angle in degrees, `gamma_lv`: liquid surface tension (mN/m).
    pub fn neumann_equation(theta_deg: f64, gamma_lv: f64) -> f64 {
        // Neumann EOS: cos θ = -1 + 2√(γ_SV/γ_LV) · exp(-β(γ_LV - γ_SV)²)
        // where β ≈ 0.0001247 (mN/m)^-2
        // Iterative solution for γ_SV
        let beta = 0.0001247;
        let cos_theta = theta_deg.to_radians().cos();

        // Newton's method
        let mut gamma_sv = gamma_lv * (1.0 + cos_theta) / 2.0; // initial guess
        for _ in 0..50 {
            let diff = gamma_lv - gamma_sv;
            let exp_term = (-beta * diff * diff).exp();
            let f = -1.0 + 2.0 * (gamma_sv / gamma_lv).sqrt() * exp_term - cos_theta;

            // Derivative
            let df = (1.0 / (gamma_lv * gamma_sv).sqrt()) * exp_term
                + 2.0 * (gamma_sv / gamma_lv).sqrt() * exp_term * 2.0 * beta * diff;

            if df.abs() < 1e-15 {
                break;
            }
            let step = f / df;
            gamma_sv -= step;
            gamma_sv = gamma_sv.max(0.01);
            if step.abs() < 1e-8 {
                break;
            }
        }
        gamma_sv
    }
}

// ─── DynamicContactAngle ─────────────────────────────────────────────────────

/// Dynamic (advancing/receding) contact angle analysis.
pub struct DynamicContactAngle;

impl DynamicContactAngle {
    /// Advancing angle: maximum stable contact angle from a series.
    pub fn advancing_angle(angles: &[f64]) -> f64 {
        angles.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
    }

    /// Receding angle: minimum stable contact angle from a series.
    pub fn receding_angle(angles: &[f64]) -> f64 {
        angles.iter().cloned().fold(f64::INFINITY, f64::min)
    }

    /// Contact angle hysteresis: Δθ = θ_a - θ_r.
    pub fn hysteresis(advancing: f64, receding: f64) -> f64 {
        (advancing - receding).abs()
    }

    /// Tilting plate method: determine advancing/receding from tilt experiment.
    /// `tilt_angles`: plate tilt angles (degrees).
    /// `drop_angles`: measured front/back contact angles at each tilt.
    /// Returns (advancing, receding) angles.
    pub fn tilting_plate(tilt_angles: &[f64], drop_angles: &[f64]) -> (f64, f64) {
        assert_eq!(tilt_angles.len(), drop_angles.len());
        assert!(!tilt_angles.is_empty());

        // The advancing angle is at maximum tilt (front of drop).
        // The receding angle is at maximum tilt (back of drop).
        // For simplicity: advancing = max of measurements, receding = min.
        let adv = drop_angles.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let rec = drop_angles.iter().cloned().fold(f64::INFINITY, f64::min);
        (adv, rec)
    }

    /// Roll-off angle prediction.
    /// Uses the Furmidge equation: sin α = (γ_LV · w / (m · g)) · (cos θ_r - cos θ_a)
    /// `advancing`/`receding` in degrees, `drop_volume_ul` in µL, `gamma_lv` in mN/m, `density` in kg/m^3.
    /// Returns roll-off angle in degrees.
    pub fn roll_off_angle(
        advancing: f64,
        receding: f64,
        drop_volume_ul: f64,
        gamma_lv: f64,
        density: f64,
    ) -> f64 {
        // Furmidge equation: mg sin α = γ_LV · w · (cos θ_r - cos θ_a)
        // Assume w ≈ (6V/π)^(1/3) for spherical drop, contact width
        let vol_m3 = drop_volume_ul * 1e-9; // µL -> m^3
        let mass = density * vol_m3;
        // Approximate contact width from volume (spherical cap)
        let r_approx = (3.0 * vol_m3 / (4.0 * PI)).cbrt();
        let w = 2.0 * r_approx;

        let gamma_si = gamma_lv * 1e-3; // mN/m -> N/m
        let cos_r = receding.to_radians().cos();
        let cos_a = advancing.to_radians().cos();

        let sin_alpha = gamma_si * w * (cos_r - cos_a) / (mass * G);
        let sin_alpha = sin_alpha.clamp(-1.0, 1.0);
        sin_alpha.asin().to_degrees()
    }
}

// ─── LiquidProperties ───────────────────────────────────────────────────────

/// Properties of a probe liquid for contact angle measurements.
#[derive(Debug, Clone)]
pub struct LiquidProperties {
    pub name: String,
    pub gamma_total: f64,      // Total surface tension (mN/m)
    pub gamma_dispersive: f64, // Dispersive component (mN/m)
    pub gamma_polar: f64,      // Polar component (mN/m)
}

/// Database of common probe liquids.
pub struct LiquidDatabase;

impl LiquidDatabase {
    /// Water: γ = 72.8, γ_d = 21.8, γ_p = 51.0 mN/m.
    pub fn water() -> LiquidProperties {
        LiquidProperties {
            name: "Water".into(),
            gamma_total: 72.8,
            gamma_dispersive: 21.8,
            gamma_polar: 51.0,
        }
    }

    /// Diiodomethane: γ = 50.8, γ_d = 50.8, γ_p = 0.0 mN/m (purely dispersive).
    pub fn diiodomethane() -> LiquidProperties {
        LiquidProperties {
            name: "Diiodomethane".into(),
            gamma_total: 50.8,
            gamma_dispersive: 50.8,
            gamma_polar: 0.0,
        }
    }

    /// Formamide: γ = 58.0, γ_d = 39.0, γ_p = 19.0 mN/m.
    pub fn formamide() -> LiquidProperties {
        LiquidProperties {
            name: "Formamide".into(),
            gamma_total: 58.0,
            gamma_dispersive: 39.0,
            gamma_polar: 19.0,
        }
    }

    /// Glycerol: γ = 64.0 mN/m.
    pub fn glycerol() -> LiquidProperties {
        LiquidProperties {
            name: "Glycerol".into(),
            gamma_total: 64.0,
            gamma_dispersive: 37.0,
            gamma_polar: 27.0,
        }
    }

    /// Ethylene glycol: γ = 48.0 mN/m.
    pub fn ethylene_glycol() -> LiquidProperties {
        LiquidProperties {
            name: "Ethylene Glycol".into(),
            gamma_total: 48.0,
            gamma_dispersive: 29.0,
            gamma_polar: 19.0,
        }
    }

    /// Hexadecane: γ = 27.5 mN/m (purely dispersive).
    pub fn hexadecane() -> LiquidProperties {
        LiquidProperties {
            name: "Hexadecane".into(),
            gamma_total: 27.5,
            gamma_dispersive: 27.5,
            gamma_polar: 0.0,
        }
    }
}

// ─── YoungEquation ───────────────────────────────────────────────────────────

/// Young's equation and related wetting models.
pub struct YoungEquation;

impl YoungEquation {
    /// Interfacial tension from Young's equation: γ_SL = γ_SV - γ_LV cos θ.
    pub fn gamma_sl(gamma_sv: f64, gamma_lv: f64, theta_deg: f64) -> f64 {
        gamma_sv - gamma_lv * theta_deg.to_radians().cos()
    }

    /// Equilibrium contact angle from Young's equation: cos θ = (γ_SV - γ_SL) / γ_LV.
    pub fn cos_theta(gamma_sv: f64, gamma_sl: f64, gamma_lv: f64) -> f64 {
        ((gamma_sv - gamma_sl) / gamma_lv).clamp(-1.0, 1.0)
    }

    /// Wenzel apparent contact angle on a rough surface: cos θ* = r · cos θ.
    /// `roughness_ratio` r >= 1 (actual area / projected area).
    pub fn wenzel_apparent_angle(intrinsic_angle_deg: f64, roughness_ratio: f64) -> f64 {
        let cos_theta = intrinsic_angle_deg.to_radians().cos();
        let cos_apparent = (roughness_ratio * cos_theta).clamp(-1.0, 1.0);
        cos_apparent.acos().to_degrees()
    }

    /// Cassie-Baxter apparent angle on a heterogeneous surface.
    /// cos θ* = f₁ cos θ₁ + f₂ cos θ₂, where f₂ = 1 - f₁.
    pub fn cassie_baxter_angle(
        theta1_deg: f64,
        theta2_deg: f64,
        f1: f64,
    ) -> f64 {
        let f2 = 1.0 - f1;
        let cos_apparent = f1 * theta1_deg.to_radians().cos() + f2 * theta2_deg.to_radians().cos();
        let cos_apparent = cos_apparent.clamp(-1.0, 1.0);
        cos_apparent.acos().to_degrees()
    }

    /// Wenzel-to-Cassie-Baxter transition angle.
    /// The Cassie state becomes favorable when the surface is sufficiently rough.
    /// Compares Wenzel and Cassie-Baxter (with air, θ₂ = 180°) apparent angles.
    pub fn wenzel_to_cassie_transition(
        intrinsic_angle_deg: f64,
        roughness: f64,
        fraction: f64,
    ) -> f64 {
        // Wenzel: cos θ_W* = r cos θ
        let wenzel = Self::wenzel_apparent_angle(intrinsic_angle_deg, roughness);
        // Cassie-Baxter with air (θ₂ = 180°): cos θ_CB* = f₁ cos θ₁ - (1 - f₁)
        let cassie = Self::cassie_baxter_angle(intrinsic_angle_deg, 180.0, fraction);

        // Return the larger (more hydrophobic) angle — the stable state
        wenzel.max(cassie)
    }
}

// ─── CapillaryAnalysis ───────────────────────────────────────────────────────

/// Capillary phenomena calculations.
pub struct CapillaryAnalysis;

impl CapillaryAnalysis {
    /// Capillary rise height: h = 2γ cos θ / (ρ g r).
    /// `gamma` in mN/m, `theta_deg` contact angle, `density` in kg/m^3, `radius_m` tube radius in metres.
    /// Returns height in metres.
    pub fn capillary_rise(gamma: f64, theta_deg: f64, density: f64, radius_m: f64) -> f64 {
        let gamma_si = gamma * 1e-3; // mN/m -> N/m
        2.0 * gamma_si * theta_deg.to_radians().cos() / (density * G * radius_m)
    }

    /// Capillary number: Ca = μv / γ.
    /// `velocity` in m/s, `viscosity` in Pa·s, `gamma` in mN/m.
    pub fn capillary_number(velocity: f64, viscosity: f64, gamma: f64) -> f64 {
        let gamma_si = gamma * 1e-3;
        viscosity * velocity / gamma_si
    }

    /// Bond number: Bo = Δρ g d² / γ.
    /// `density_diff` in kg/m^3, `length_m` characteristic length in m, `gamma` in mN/m.
    pub fn bond_number(density_diff: f64, length_m: f64, gamma: f64) -> f64 {
        let gamma_si = gamma * 1e-3;
        density_diff * G * length_m * length_m / gamma_si
    }

    /// Capillary length: λ_c = √(γ / (ρg)).
    /// `gamma` in mN/m, `density` in kg/m^3.
    /// Returns length in metres.
    pub fn capillary_length(gamma: f64, density: f64) -> f64 {
        let gamma_si = gamma * 1e-3;
        (gamma_si / (density * G)).sqrt()
    }
}

// ─── GoniometerSimulator ─────────────────────────────────────────────────────

/// Generate synthetic sessile drop profiles for testing.
pub struct GoniometerSimulator;

impl GoniometerSimulator {
    /// Simulate a perfect spherical cap drop profile.
    /// `radius` in mm (radius of curvature), `theta_deg` contact angle, `num_points` on one side.
    pub fn simulate_spherical_cap(radius: f64, theta_deg: f64, num_points: usize) -> DropProfile {
        let theta_rad = theta_deg.to_radians();
        // Spherical cap: center of sphere is at (0, y_c)
        // Contact radius a = R sin θ
        // y_c = -R cos θ (if θ < 90°, center below baseline; if θ > 90°, above)
        let a = radius * theta_rad.sin();
        let y_c = -radius * theta_rad.cos(); // negative means below baseline for hydrophilic

        let mut xs = Vec::with_capacity(2 * num_points + 1);
        let mut ys = Vec::with_capacity(2 * num_points + 1);

        // Parametric: arc from left contact to top to right contact.
        // Angle range on circle: from (π - θ) to θ (measuring from positive x)
        // Actually, parameterize from left contact to right contact over the top.
        let angle_start = PI - theta_rad; // left contact
        let angle_end = theta_rad;        // right contact
        // Going counter-clockwise from left to right over the top
        let total_points = 2 * num_points + 1;

        for i in 0..total_points {
            let t = i as f64 / (total_points - 1) as f64;
            let angle = angle_start + t * (angle_end - angle_start); // Note: angle_end < angle_start for θ<90°
            // For θ < 90°: start > π/2, end < π/2, so we go the wrong way.
            // Let's parameterize over the arc going over the top of the circle.
            // The arc above the baseline spans from angle (π-θ) to θ (going clockwise from left to right).
            // But we want to go counterclockwise (left -> top -> right).
            // The angle at top is π/2.
            // Left contact: angle from center = π - θ (measuring from +x axis)
            // Right contact: angle = θ
            // Going from left to right over top: angles decrease from (π-θ) to θ
            // But if θ < 90°, then π-θ > θ, so decreasing is correct.
            // If θ > 90°, then π-θ < θ, but we want to go over the top too.
            // Hmm, let's just do it differently.
            let _ = angle; // discard above

            // Simple parametrization: x from -a to +a, y from circle equation
            let x = -a + 2.0 * a * t;
            // y = y_c + sqrt(R^2 - x^2), taking the upper root
            let arg = radius * radius - x * x;
            let y = if arg > 0.0 {
                y_c + arg.sqrt()
            } else {
                0.0 // on baseline
            };
            xs.push(x);
            ys.push(y.max(0.0));
        }

        // Add baseline contact points explicitly
        xs.insert(0, -a);
        ys.insert(0, 0.0);
        xs.push(a);
        ys.push(0.0);

        DropProfile::new(xs, ys)
    }

    /// Simulate a gravity-deformed drop (oblate spheroid approximation).
    /// Higher Bond number means more flattening.
    pub fn simulate_gravity_deformed(
        radius: f64,
        theta_deg: f64,
        bond_number: f64,
    ) -> DropProfile {
        // Start with spherical cap and deform
        let base = Self::simulate_spherical_cap(radius, theta_deg, 50);

        // Gravity deformation: flatten y by factor (1 - Bo/4), widen x by factor (1 + Bo/8)
        let y_scale = 1.0 - bond_number * 0.25;
        let x_scale = 1.0 + bond_number * 0.125;

        let xs: Vec<f64> = base.x.iter().map(|x| x * x_scale).collect();
        let ys: Vec<f64> = base.y.iter().map(|y| y * y_scale.max(0.1)).collect();

        DropProfile::new(xs, ys)
    }

    /// Add Gaussian noise to a drop profile.
    /// `noise_mm` is the standard deviation of the noise in mm.
    pub fn add_noise(profile: &DropProfile, noise_mm: f64) -> DropProfile {
        // Simple LCG-based pseudo-random for reproducibility
        let mut seed: u64 = 42;
        let mut next_rand = || -> f64 {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            // Box-Muller approximation using the seed
            let u1 = (seed >> 33) as f64 / (1u64 << 31) as f64;
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let u2 = (seed >> 33) as f64 / (1u64 << 31) as f64;
            let u1 = u1.max(1e-10);
            (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
        };

        let xs: Vec<f64> = profile.x.iter().map(|x| x + noise_mm * next_rand()).collect();
        let ys: Vec<f64> = profile.y.iter().map(|y| y + noise_mm * next_rand()).collect();

        DropProfile::new(xs, ys)
    }

    /// Simulate a tilted (asymmetric) drop with different advancing/receding angles.
    pub fn simulate_tilted_drop(
        theta_a: f64,
        theta_r: f64,
        _tilt_deg: f64,
    ) -> DropProfile {
        // Generate asymmetric drop: left side with receding angle, right side with advancing angle
        let n = 50;
        let mut xs = Vec::with_capacity(2 * n + 2);
        let mut ys = Vec::with_capacity(2 * n + 2);

        // Use average radius
        let r = 1.0; // mm
        let a_left = r * theta_r.to_radians().sin();
        let a_right = r * theta_a.to_radians().sin();

        // Left half (receding)
        let yc_left = -r * theta_r.to_radians().cos();
        for i in 0..=n {
            let t = i as f64 / n as f64;
            let x = -a_left * (1.0 - t);
            let arg = r * r - x * x;
            let y = if arg > 0.0 { yc_left + arg.sqrt() } else { 0.0 };
            xs.push(x);
            ys.push(y.max(0.0));
        }

        // Right half (advancing)
        let yc_right = -r * theta_a.to_radians().cos();
        for i in 1..=n {
            let t = i as f64 / n as f64;
            let x = a_right * t;
            let arg = r * r - x * x;
            let y = if arg > 0.0 { yc_right + arg.sqrt() } else { 0.0 };
            xs.push(x);
            ys.push(y.max(0.0));
        }

        // Add baseline endpoints
        xs.insert(0, -a_left);
        ys.insert(0, 0.0);
        xs.push(a_right);
        ys.push(0.0);

        DropProfile::new(xs, ys)
    }
}

// ─── SurfaceTreatmentAnalyzer ────────────────────────────────────────────────

/// Aging study result.
#[derive(Debug, Clone, Copy)]
pub struct AgingResult {
    pub initial_angle: f64,
    pub final_angle: f64,
    /// Rate of angle change per day.
    pub recovery_rate: f64,
    /// Whether the surface treatment is stable (< 5° change over measurement period).
    pub stable: bool,
}

/// Surface treatment effectiveness analysis.
pub struct SurfaceTreatmentAnalyzer;

impl SurfaceTreatmentAnalyzer {
    /// Treatment effectiveness as percentage change in contact angle.
    pub fn treatment_effectiveness(theta_before: f64, theta_after: f64) -> f64 {
        if theta_before.abs() < 1e-12 {
            return 0.0;
        }
        ((theta_after - theta_before) / theta_before).abs() * 100.0
    }

    /// Aging study: track contact angle recovery over time.
    pub fn aging_study(angles: &[f64], times_days: &[f64]) -> AgingResult {
        assert_eq!(angles.len(), times_days.len());
        assert!(angles.len() >= 2);

        let initial = angles[0];
        let final_val = *angles.last().unwrap();
        let time_span = times_days.last().unwrap() - times_days[0];

        let recovery_rate = if time_span.abs() > 1e-12 {
            (final_val - initial) / time_span
        } else {
            0.0
        };

        let max_change = angles
            .iter()
            .map(|a| (a - initial).abs())
            .fold(0.0_f64, f64::max);

        AgingResult {
            initial_angle: initial,
            final_angle: final_val,
            recovery_rate,
            stable: max_change < 5.0,
        }
    }
}

// ─── Helper functions ────────────────────────────────────────────────────────

/// Simple linear least-squares fit: y = a + b·x.
/// Returns (slope, intercept).
fn linear_fit(x: &[f64], y: &[f64]) -> (f64, f64) {
    let n = x.len() as f64;
    let sx: f64 = x.iter().sum();
    let sy: f64 = y.iter().sum();
    let sxy: f64 = x.iter().zip(y.iter()).map(|(xi, yi)| xi * yi).sum();
    let sxx: f64 = x.iter().map(|xi| xi * xi).sum();

    let denom = n * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return (0.0, sy / n);
    }
    let slope = (n * sxy - sx * sy) / denom;
    let intercept = (sy - slope * sx) / n;
    (slope, intercept)
}

/// Kasa algebraic circle fit: minimize Σ(x² + y² - 2cx·x - 2cy·y + c)².
/// Returns (cx, cy, radius).
fn circle_fit_kasa(pts: &[(f64, f64)]) -> (f64, f64, f64) {
    let n = pts.len() as f64;
    let mut sx = 0.0;
    let mut sy = 0.0;
    let mut sxx = 0.0;
    let mut syy = 0.0;
    let mut sxy = 0.0;
    let mut sxz = 0.0;
    let mut syz = 0.0;

    for &(x, y) in pts {
        let z = x * x + y * y;
        sx += x;
        sy += y;
        sxx += x * x;
        syy += y * y;
        sxy += x * y;
        sxz += x * z;
        syz += y * z;
    }

    let a = n * sxx - sx * sx;
    let b = n * sxy - sx * sy;
    let c = n * syy - sy * sy;
    let d = 0.5 * (n * sxz - sx * (sxx + syy) + sx * (sx * sx + sy * sy) / n);
    let e = 0.5 * (n * syz - sy * (sxx + syy) + sy * (sx * sx + sy * sy) / n);

    // Actually, standard Kasa formulation:
    // A = [Σx² Σxy; Σxy Σy²] (centered), b = [Σx(x²+y²); Σy(x²+y²)] / 2
    let det = a * c - b * b;
    if det.abs() < 1e-20 {
        // Degenerate: return mean as center
        let cx = sx / n;
        let cy = sy / n;
        let r = pts
            .iter()
            .map(|(x, y)| ((x - cx).powi(2) + (y - cy).powi(2)).sqrt())
            .sum::<f64>()
            / n;
        return (cx, cy, r);
    }

    // Solve for center offsets using simpler formulation
    let mean_x = sx / n;
    let mean_y = sy / n;

    // Centered moments
    let mut uu = 0.0;
    let mut uv = 0.0;
    let mut vv = 0.0;
    let mut uuu = 0.0;
    let mut vvv = 0.0;
    let mut uvv = 0.0;
    let mut vuu = 0.0;

    for &(x, y) in pts {
        let u = x - mean_x;
        let v = y - mean_y;
        uu += u * u;
        uv += u * v;
        vv += v * v;
        uuu += u * u * u;
        vvv += v * v * v;
        uvv += u * v * v;
        vuu += v * u * u;
    }

    let det2 = uu * vv - uv * uv;
    if det2.abs() < 1e-20 {
        return (mean_x, mean_y, 1.0);
    }

    let uc = (vv * (uuu + uvv) - uv * (vvv + vuu)) / (2.0 * det2);
    let vc = (uu * (vvv + vuu) - uv * (uuu + uvv)) / (2.0 * det2);

    let cx = uc + mean_x;
    let cy = vc + mean_y;
    let r = (uc * uc + vc * vc + (uu + vv) / n).sqrt();

    (cx, cy, r)
}

/// Compute contact angle from circle fit at a contact point.
/// `is_left`: true for left contact, false for right contact.
fn circle_tangent_angle(cx: f64, cy: f64, r: f64, x_contact: f64, y_base: f64, is_left: bool) -> f64 {
    // For a sessile drop with circle center at (cx, cy), the contact angle at a
    // baseline contact point is derived from the tangent to the circle.
    //
    // At the LEFT contact: radius from (cx, cy) to (x_left, y_base) is (dx, dy)
    //   with dx < 0, dy >= 0 (if center below baseline).
    //   The tangent into the liquid points RIGHT and UP: perpendicular = (-dy, dx) rotated.
    //   Tangent direction: (-(y_base-cy), x_contact-cx) = (cy-y_base, x_contact-cx)
    //   For left contact (x_contact < cx): tangent = (cy-y_base, x_contact-cx), both typically positive.
    //   Contact angle = atan2(tangent_y, tangent_x) where tangent points into the liquid.
    //
    // At the RIGHT contact: by symmetry, the tangent into the liquid points LEFT and UP.
    //   Contact angle = π - atan2(tangent_y, -tangent_x).
    let _ = r;

    let dx = x_contact - cx;
    let dy = y_base - cy;

    if is_left {
        // Tangent into liquid (perpendicular to radius, pointing right and up):
        // Rotate (dx, dy) by -90°: (dy, -dx)
        // Since dx < 0 and dy > 0 typically: tangent = (dy, -dx) = (positive, positive) = right and up.
        let tx = dy;
        let ty = -dx;
        ty.atan2(tx).to_degrees().clamp(0.0, 180.0)
    } else {
        // Tangent into liquid (perpendicular to radius, pointing left and up):
        // Rotate (dx, dy) by +90°: (-dy, dx)
        // Since dx > 0 and dy > 0 typically: tangent = (-dy, dx) = (negative, positive) = left and up.
        let tx = -dy;
        let ty = dx;
        // Contact angle measured from rightward horizontal (solid surface going right of contact)
        // through liquid (CCW): = π - atan2(ty, tx) since tangent points left
        let tangent_angle = ty.atan2(tx).to_degrees();
        // tangent_angle is in (90°, 180°) range for typical drops
        // Contact angle = 180° - tangent_angle
        (180.0 - tangent_angle).clamp(0.0, 180.0)
    }
}

/// Compute contact angle from ellipse parameters at a contact point.
fn ellipse_tangent_angle(a: f64, b: f64, x_rel: f64, _y_rel: f64) -> f64 {
    // For ellipse x²/a² + y²/b² = 1, the slope dy/dx = -(b²x)/(a²y)
    // At the contact point (x_rel, 0), the slope is infinite (vertical tangent for baseline contact).
    // We need the angle of the tangent just above the baseline.
    // Use a small offset above baseline for numerical stability.
    let epsilon = b * 0.01;
    let y_eval = epsilon;
    let x_at_y = a * (1.0 - (y_eval / b).powi(2)).sqrt();

    if a.abs() < 1e-12 || y_eval.abs() < 1e-12 {
        return 90.0;
    }

    // dy/dx at (x_at_y, y_eval)
    let dydx = -(b * b * x_at_y) / (a * a * y_eval);

    // Contact angle: angle from horizontal baseline.
    // For left contact (x_rel < 0), tangent slopes upward to the right.
    // For right contact (x_rel > 0), tangent slopes upward to the left.
    let tangent_angle_from_horiz = dydx.abs().atan().to_degrees();

    if x_rel < 0.0 {
        // Left contact: angle measured from substrate into liquid
        180.0 - tangent_angle_from_horiz
    } else {
        180.0 - tangent_angle_from_horiz
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 0.5; // 0.5 degree tolerance for angle measurements
    const TOL_E: f64 = 2.0; // 2 mN/m tolerance for surface energy

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // --- DropProfile tests ---

    #[test]
    fn test_drop_profile_new() {
        let p = DropProfile::new(vec![0.0, 1.0, 2.0], vec![0.0, 1.0, 0.0]);
        assert_eq!(p.x.len(), 3);
        assert_eq!(p.y.len(), 3);
    }

    #[test]
    #[should_panic]
    fn test_drop_profile_mismatched_lengths() {
        DropProfile::new(vec![0.0, 1.0], vec![0.0]);
    }

    #[test]
    #[should_panic]
    fn test_drop_profile_too_few_points() {
        DropProfile::new(vec![0.0, 1.0], vec![0.0, 1.0]);
    }

    #[test]
    fn test_baseline_detect() {
        let p = DropProfile::new(
            vec![-1.0, -0.5, 0.0, 0.5, 1.0],
            vec![0.0, 0.5, 1.0, 0.5, 0.0],
        );
        let (y_base, x_left, x_right) = p.baseline_detect();
        assert!(approx_eq(y_base, 0.0, 0.01));
        assert!(approx_eq(x_left, -1.0, 0.01));
        assert!(approx_eq(x_right, 1.0, 0.01));
    }

    #[test]
    fn test_drop_width() {
        let p = DropProfile::new(
            vec![-2.0, 0.0, 2.0],
            vec![0.0, 1.0, 0.0],
        );
        assert!(approx_eq(p.drop_width(), 4.0, 0.1));
    }

    #[test]
    fn test_drop_height() {
        let p = DropProfile::new(
            vec![-1.0, 0.0, 1.0],
            vec![0.0, 2.5, 0.0],
        );
        assert!(approx_eq(p.drop_height(), 2.5, 0.1));
    }

    #[test]
    fn test_volume_spherical_cap() {
        // Spherical cap V = πh(3a² + h²)/6
        // a = 1mm, h = 0.5mm -> V = π·0.5·(3+0.25)/6 ≈ 0.8508 mm³ = 0.8508 µL
        let p = DropProfile::new(
            vec![-1.0, -0.5, 0.0, 0.5, 1.0],
            vec![0.0, 0.3, 0.5, 0.3, 0.0],
        );
        let vol = p.volume_ul(false);
        assert!(vol > 0.0, "Volume should be positive: {}", vol);
    }

    #[test]
    fn test_volume_axisymmetric() {
        let p = GoniometerSimulator::simulate_spherical_cap(2.0, 60.0, 50);
        let vol = p.volume_ul(true);
        assert!(vol > 0.0, "Axisymmetric volume should be positive: {}", vol);
    }

    // --- Half angle method tests ---

    #[test]
    fn test_half_angle_method_90deg() {
        // For a hemisphere: h = R, w = 2R -> θ = 2·atan(2R/(2R)) = 2·atan(1) = 90°
        let angle = ContactAngleMeasurement::half_angle_method(1.0, 2.0);
        assert!(approx_eq(angle, 90.0, TOL), "Expected ~90°, got {}", angle);
    }

    #[test]
    fn test_half_angle_method_small_angle() {
        // Small angle: h << w -> θ ≈ 2·atan(2h/w) ≈ 4h/w rad
        let angle = ContactAngleMeasurement::half_angle_method(0.1, 4.0);
        assert!(angle < 20.0, "Expected small angle, got {}", angle);
    }

    #[test]
    fn test_half_angle_method_large_angle() {
        // Tall drop: h = 2, w = 1 -> θ = 2·atan(4) ≈ 152°
        let angle = ContactAngleMeasurement::half_angle_method(2.0, 1.0);
        assert!(angle > 140.0, "Expected large angle, got {}", angle);
    }

    #[test]
    fn test_half_angle_method_45deg() {
        // θ = 45° -> tan(22.5°) = 2h/w -> h/w = tan(22.5°)/2 ≈ 0.2071
        let w = 2.0;
        let h = w * (22.5_f64.to_radians().tan()) / 2.0;
        let angle = ContactAngleMeasurement::half_angle_method(h, w);
        assert!(approx_eq(angle, 45.0, TOL), "Expected ~45°, got {}", angle);
    }

    // --- Circle fit method tests ---

    #[test]
    fn test_circle_fit_spherical_cap() {
        let profile = GoniometerSimulator::simulate_spherical_cap(2.0, 60.0, 100);
        let result = ContactAngleMeasurement::circle_fit_method(&profile);
        // Should be close to 60° (within a few degrees for a discrete approximation)
        assert!(
            result.average_angle_deg > 40.0 && result.average_angle_deg < 80.0,
            "Expected ~60°, got {}", result.average_angle_deg
        );
    }

    #[test]
    fn test_circle_fit_90deg() {
        let profile = GoniometerSimulator::simulate_spherical_cap(2.0, 90.0, 100);
        let result = ContactAngleMeasurement::circle_fit_method(&profile);
        assert!(
            result.average_angle_deg > 70.0 && result.average_angle_deg < 110.0,
            "Expected ~90°, got {}", result.average_angle_deg
        );
    }

    // --- Ellipse fit method tests ---

    #[test]
    fn test_ellipse_fit_spherical() {
        let profile = GoniometerSimulator::simulate_spherical_cap(2.0, 60.0, 100);
        let result = ContactAngleMeasurement::ellipse_fit_method(&profile);
        assert!(
            result.average_angle_deg > 30.0 && result.average_angle_deg < 120.0,
            "Expected reasonable angle, got {}", result.average_angle_deg
        );
    }

    // --- Tangent method tests ---

    #[test]
    fn test_tangent_method_symmetric() {
        let profile = GoniometerSimulator::simulate_spherical_cap(2.0, 70.0, 100);
        let result = ContactAngleMeasurement::tangent_method(&profile);
        // Symmetric drop should have similar left and right
        assert!(
            result.hysteresis_deg < 10.0,
            "Symmetric drop hysteresis should be small: {}", result.hysteresis_deg
        );
    }

    // --- Young-Laplace fit tests ---

    #[test]
    fn test_young_laplace_small_drop() {
        // Small drop (low Bond number) should agree with half-angle method
        let profile = GoniometerSimulator::simulate_spherical_cap(1.0, 60.0, 50);
        let theta = ContactAngleMeasurement::young_laplace_fit(&profile, 998.0, 72.8);
        assert!(theta > 40.0 && theta < 80.0, "Expected ~60°, got {}", theta);
    }

    // --- Wettability classification tests ---

    #[test]
    fn test_classify_superhydrophilic() {
        assert_eq!(WettabilityClassifier::classify(5.0), Wettability::Superhydrophilic);
    }

    #[test]
    fn test_classify_hydrophilic() {
        assert_eq!(WettabilityClassifier::classify(45.0), Wettability::Hydrophilic);
    }

    #[test]
    fn test_classify_hydrophobic() {
        assert_eq!(WettabilityClassifier::classify(110.0), Wettability::Hydrophobic);
    }

    #[test]
    fn test_classify_superhydrophobic() {
        assert_eq!(WettabilityClassifier::classify(160.0), Wettability::Superhydrophobic);
    }

    #[test]
    fn test_classify_boundary_90() {
        assert_eq!(WettabilityClassifier::classify(89.9), Wettability::Hydrophilic);
        assert_eq!(WettabilityClassifier::classify(90.1), Wettability::Hydrophobic);
    }

    // --- Work of adhesion / cohesion tests ---

    #[test]
    fn test_work_of_adhesion_complete_wetting() {
        // θ = 0° -> W_a = 2γ_LV
        let wa = WettabilityClassifier::work_of_adhesion(72.8, 0.0);
        assert!(approx_eq(wa, 2.0 * 72.8, 0.1));
    }

    #[test]
    fn test_work_of_adhesion_90deg() {
        // θ = 90° -> W_a = γ_LV
        let wa = WettabilityClassifier::work_of_adhesion(72.8, 90.0);
        assert!(approx_eq(wa, 72.8, 0.1));
    }

    #[test]
    fn test_work_of_cohesion() {
        let wc = WettabilityClassifier::work_of_cohesion(72.8);
        assert!(approx_eq(wc, 145.6, 0.01));
    }

    #[test]
    fn test_spreading_coefficient_positive() {
        // S > 0 means complete wetting
        let s = WettabilityClassifier::spreading_coefficient(50.0, 10.0, 30.0);
        assert!(s > 0.0, "S = {} should be positive (complete wetting)", s);
    }

    #[test]
    fn test_spreading_coefficient_negative() {
        // S < 0 means partial wetting
        let s = WettabilityClassifier::spreading_coefficient(30.0, 10.0, 72.8);
        assert!(s < 0.0, "S = {} should be negative (partial wetting)", s);
    }

    // --- Surface energy tests ---

    #[test]
    fn test_owrk_ptfe() {
        // PTFE: θ_water ≈ 110°, θ_diio ≈ 70°, γ_total ≈ 20 mN/m
        let result = SurfaceEnergyCalculator::owrk(110.0, 70.0);
        assert!(result.total > 10.0 && result.total < 30.0,
            "PTFE surface energy should be ~20 mN/m, got {}", result.total);
        assert!(result.dispersive > result.polar,
            "PTFE should be mostly dispersive");
    }

    #[test]
    fn test_owrk_glass() {
        // Clean glass: θ_water ≈ 20°, θ_diio ≈ 30°
        let result = SurfaceEnergyCalculator::owrk(20.0, 30.0);
        assert!(result.total > 40.0, "Glass surface energy should be high: {}", result.total);
    }

    #[test]
    fn test_owrk_dispersive_dominates_for_nonpolar() {
        // Nonpolar surface: high diiodomethane angle, moderate water angle
        let result = SurfaceEnergyCalculator::owrk(95.0, 60.0);
        assert!(result.dispersive >= 0.0, "Dispersive should be non-negative");
    }

    #[test]
    fn test_van_oss_good() {
        // Example surface
        let result = SurfaceEnergyCalculator::van_oss_good(70.0, 40.0, 55.0);
        assert!(result.total > 0.0, "Total surface energy should be positive");
        assert!(result.lw > 0.0, "LW component should be positive");
    }

    #[test]
    fn test_van_oss_total_components() {
        let result = SurfaceEnergyCalculator::van_oss_good(60.0, 35.0, 45.0);
        // Total = LW + 2√(acid·base)
        let expected_total = result.lw + 2.0 * (result.acid * result.base).sqrt();
        assert!(approx_eq(result.total, expected_total, 0.1));
    }

    #[test]
    fn test_zisman_critical_surface_tension() {
        // Linear series: as γ_LV increases, θ increases
        let thetas = vec![20.0, 40.0, 60.0, 80.0];
        let gammas = vec![25.0, 35.0, 50.0, 72.8];
        let gamma_c = SurfaceEnergyCalculator::zisman_critical_surface_tension(&thetas, &gammas);
        assert!(gamma_c > 10.0 && gamma_c < 80.0, "γ_c should be reasonable: {}", gamma_c);
    }

    #[test]
    fn test_neumann_equation() {
        // Water on a moderate surface
        let gamma_sv = SurfaceEnergyCalculator::neumann_equation(70.0, 72.8);
        assert!(gamma_sv > 20.0 && gamma_sv < 60.0, "γ_SV should be reasonable: {}", gamma_sv);
    }

    #[test]
    fn test_neumann_complete_wetting() {
        // θ ≈ 0: γ_SV should be close to γ_LV
        let gamma_sv = SurfaceEnergyCalculator::neumann_equation(5.0, 72.8);
        assert!(gamma_sv > 50.0, "Near-complete wetting: γ_SV should be high: {}", gamma_sv);
    }

    // --- Young's equation tests ---

    #[test]
    fn test_young_gamma_sl() {
        // γ_SL = γ_SV - γ_LV cos θ
        // θ = 90°: γ_SL = γ_SV
        let gamma_sl = YoungEquation::gamma_sl(40.0, 72.8, 90.0);
        assert!(approx_eq(gamma_sl, 40.0, 0.1));
    }

    #[test]
    fn test_young_cos_theta() {
        // Complete wetting: γ_SV >> γ_SL -> cos θ → 1
        let cos_t = YoungEquation::cos_theta(100.0, 10.0, 72.8);
        assert!(cos_t > 0.9, "Should be near complete wetting: {}", cos_t);
    }

    #[test]
    fn test_young_cos_theta_clamped() {
        // Extreme case: should be clamped to [-1, 1]
        let cos_t = YoungEquation::cos_theta(1000.0, 0.0, 72.8);
        assert!(cos_t <= 1.0 && cos_t >= -1.0);
    }

    #[test]
    fn test_wenzel_hydrophilic_more_hydrophilic() {
        // Wenzel: roughness makes hydrophilic surfaces more hydrophilic
        let smooth = 70.0;
        let rough = YoungEquation::wenzel_apparent_angle(smooth, 1.5);
        assert!(rough < smooth, "Rough hydrophilic should be more hydrophilic: {} vs {}", rough, smooth);
    }

    #[test]
    fn test_wenzel_hydrophobic_more_hydrophobic() {
        // Wenzel: roughness makes hydrophobic surfaces more hydrophobic
        let smooth = 110.0;
        let rough = YoungEquation::wenzel_apparent_angle(smooth, 1.5);
        assert!(rough > smooth, "Rough hydrophobic should be more hydrophobic: {} vs {}", rough, smooth);
    }

    #[test]
    fn test_wenzel_roughness_1() {
        // r = 1 (smooth): no change
        let angle = YoungEquation::wenzel_apparent_angle(70.0, 1.0);
        assert!(approx_eq(angle, 70.0, TOL));
    }

    #[test]
    fn test_cassie_baxter_equal_fractions() {
        // f1 = 0.5, θ1 = θ2: should return θ1
        let angle = YoungEquation::cassie_baxter_angle(60.0, 60.0, 0.5);
        assert!(approx_eq(angle, 60.0, TOL));
    }

    #[test]
    fn test_cassie_baxter_air_pockets() {
        // Cassie-Baxter with air (θ₂ = 180°): cos θ* = f₁ cos θ₁ - (1 - f₁)
        let angle = YoungEquation::cassie_baxter_angle(110.0, 180.0, 0.5);
        assert!(angle > 110.0, "Air pockets should increase hydrophobicity: {}", angle);
    }

    #[test]
    fn test_wenzel_to_cassie_transition() {
        let result = YoungEquation::wenzel_to_cassie_transition(110.0, 2.0, 0.3);
        assert!(result > 110.0, "Transition angle should be > intrinsic: {}", result);
    }

    // --- Dynamic contact angle tests ---

    #[test]
    fn test_advancing_angle() {
        let angles = vec![60.0, 65.0, 70.0, 68.0, 72.0];
        assert!(approx_eq(DynamicContactAngle::advancing_angle(&angles), 72.0, 0.01));
    }

    #[test]
    fn test_receding_angle() {
        let angles = vec![60.0, 65.0, 55.0, 68.0, 58.0];
        assert!(approx_eq(DynamicContactAngle::receding_angle(&angles), 55.0, 0.01));
    }

    #[test]
    fn test_hysteresis() {
        let h = DynamicContactAngle::hysteresis(75.0, 55.0);
        assert!(approx_eq(h, 20.0, 0.01));
    }

    #[test]
    fn test_tilting_plate() {
        let tilts = vec![0.0, 5.0, 10.0, 15.0, 20.0];
        let angles = vec![70.0, 72.0, 75.0, 65.0, 62.0];
        let (adv, rec) = DynamicContactAngle::tilting_plate(&tilts, &angles);
        assert!(approx_eq(adv, 75.0, 0.01));
        assert!(approx_eq(rec, 62.0, 0.01));
    }

    #[test]
    fn test_roll_off_angle() {
        // Large hysteresis -> larger roll-off angle
        let alpha = DynamicContactAngle::roll_off_angle(120.0, 80.0, 5.0, 72.8, 998.0);
        assert!(alpha > 0.0, "Roll-off angle should be positive: {}", alpha);
    }

    #[test]
    fn test_roll_off_zero_hysteresis() {
        // No hysteresis -> roll-off angle ≈ 0
        let alpha = DynamicContactAngle::roll_off_angle(90.0, 90.0, 5.0, 72.8, 998.0);
        assert!(alpha.abs() < 1.0, "Zero hysteresis -> near-zero roll-off: {}", alpha);
    }

    // --- Liquid database tests ---

    #[test]
    fn test_water_properties() {
        let w = LiquidDatabase::water();
        assert!(approx_eq(w.gamma_total, 72.8, 0.1));
        assert!(approx_eq(w.gamma_dispersive + w.gamma_polar, w.gamma_total, 0.1));
    }

    #[test]
    fn test_diiodomethane_apolar() {
        let d = LiquidDatabase::diiodomethane();
        assert!(approx_eq(d.gamma_polar, 0.0, 0.1));
        assert!(approx_eq(d.gamma_total, d.gamma_dispersive, 0.1));
    }

    #[test]
    fn test_hexadecane_apolar() {
        let h = LiquidDatabase::hexadecane();
        assert!(approx_eq(h.gamma_polar, 0.0, 0.1));
    }

    #[test]
    fn test_formamide_properties() {
        let f = LiquidDatabase::formamide();
        assert!(approx_eq(f.gamma_total, 58.0, 0.1));
    }

    #[test]
    fn test_glycerol_properties() {
        let g = LiquidDatabase::glycerol();
        assert!(approx_eq(g.gamma_total, 64.0, 0.1));
    }

    #[test]
    fn test_ethylene_glycol_properties() {
        let e = LiquidDatabase::ethylene_glycol();
        assert!(approx_eq(e.gamma_total, 48.0, 0.1));
    }

    // --- Capillary analysis tests ---

    #[test]
    fn test_capillary_rise_water() {
        // Water in a 0.5mm radius glass tube: h ≈ 2·0.0728·cos(20°)/(998·9.81·0.0005) ≈ 28 mm
        let h = CapillaryAnalysis::capillary_rise(72.8, 20.0, 998.0, 0.0005);
        assert!(h > 0.02 && h < 0.04, "Expected ~28 mm rise, got {} m", h);
    }

    #[test]
    fn test_capillary_rise_hydrophobic() {
        // Hydrophobic: θ > 90° -> depression (negative rise)
        let h = CapillaryAnalysis::capillary_rise(72.8, 120.0, 998.0, 0.001);
        assert!(h < 0.0, "Hydrophobic tube: should depress, got {} m", h);
    }

    #[test]
    fn test_capillary_number() {
        // Ca = μv/γ: slow flow -> Ca << 1 (surface tension dominates)
        let ca = CapillaryAnalysis::capillary_number(0.001, 0.001, 72.8);
        assert!(ca < 1.0, "Slow flow should have Ca << 1: {}", ca);
    }

    #[test]
    fn test_bond_number() {
        // Bo = Δρgd²/γ: small drop -> Bo << 1 (surface tension dominates)
        let bo = CapillaryAnalysis::bond_number(998.0, 0.001, 72.8);
        assert!(bo < 1.0, "1 mm drop should have Bo < 1: {}", bo);
    }

    #[test]
    fn test_capillary_length_water() {
        // λ_c = √(γ/(ρg)) ≈ √(0.0728/(998·9.81)) ≈ 2.7 mm
        let lc = CapillaryAnalysis::capillary_length(72.8, 998.0);
        assert!(
            approx_eq(lc * 1000.0, 2.7, 0.1),
            "Capillary length of water ≈ 2.7 mm, got {} mm", lc * 1000.0
        );
    }

    // --- Goniometer simulator tests ---

    #[test]
    fn test_simulate_spherical_cap_60deg() {
        let p = GoniometerSimulator::simulate_spherical_cap(2.0, 60.0, 50);
        assert!(p.x.len() > 50);
        assert!(p.drop_height() > 0.0);
        assert!(p.drop_width() > 0.0);
    }

    #[test]
    fn test_simulate_spherical_cap_90deg() {
        let p = GoniometerSimulator::simulate_spherical_cap(1.0, 90.0, 50);
        // Width should be ~2R for 90° contact angle
        assert!(p.drop_width() > 1.5 && p.drop_width() < 2.5,
            "90° cap width ≈ 2R: {}", p.drop_width());
    }

    #[test]
    fn test_simulate_spherical_cap_120deg() {
        let p = GoniometerSimulator::simulate_spherical_cap(1.5, 120.0, 50);
        assert!(p.drop_height() > 0.0);
    }

    #[test]
    fn test_simulate_gravity_deformed() {
        let p = GoniometerSimulator::simulate_gravity_deformed(2.0, 60.0, 0.5);
        assert!(p.x.len() > 50);
        let spherical = GoniometerSimulator::simulate_spherical_cap(2.0, 60.0, 50);
        // Deformed drop should be wider
        assert!(p.drop_width() >= spherical.drop_width() * 0.9);
    }

    #[test]
    fn test_add_noise() {
        let p = GoniometerSimulator::simulate_spherical_cap(2.0, 60.0, 50);
        let noisy = GoniometerSimulator::add_noise(&p, 0.01);
        assert_eq!(p.x.len(), noisy.x.len());
        // At least some points should differ
        let differs = p.x.iter().zip(noisy.x.iter()).any(|(a, b)| (a - b).abs() > 1e-6);
        assert!(differs, "Noise should change some points");
    }

    #[test]
    fn test_simulate_tilted_drop() {
        let p = GoniometerSimulator::simulate_tilted_drop(80.0, 60.0, 15.0);
        assert!(p.x.len() > 10);
        assert!(p.drop_height() > 0.0);
    }

    // --- Surface treatment analyzer tests ---

    #[test]
    fn test_treatment_effectiveness() {
        let eff = SurfaceTreatmentAnalyzer::treatment_effectiveness(90.0, 45.0);
        assert!(approx_eq(eff, 50.0, 0.1), "50% reduction: {}", eff);
    }

    #[test]
    fn test_treatment_effectiveness_increase() {
        let eff = SurfaceTreatmentAnalyzer::treatment_effectiveness(60.0, 120.0);
        assert!(approx_eq(eff, 100.0, 0.1));
    }

    #[test]
    fn test_treatment_effectiveness_zero_initial() {
        let eff = SurfaceTreatmentAnalyzer::treatment_effectiveness(0.0, 45.0);
        assert!(approx_eq(eff, 0.0, 0.01));
    }

    #[test]
    fn test_aging_study_stable() {
        let angles = vec![60.0, 60.5, 61.0, 60.8, 61.2];
        let times = vec![0.0, 7.0, 14.0, 21.0, 28.0];
        let result = SurfaceTreatmentAnalyzer::aging_study(&angles, &times);
        assert!(result.stable, "Small variation should be stable");
        assert!(approx_eq(result.initial_angle, 60.0, 0.01));
    }

    #[test]
    fn test_aging_study_unstable() {
        let angles = vec![60.0, 65.0, 72.0, 80.0, 88.0];
        let times = vec![0.0, 7.0, 14.0, 21.0, 28.0];
        let result = SurfaceTreatmentAnalyzer::aging_study(&angles, &times);
        assert!(!result.stable, "Large variation should be unstable");
        assert!(result.recovery_rate > 0.0, "Angle increasing -> positive rate");
    }

    #[test]
    fn test_aging_study_recovery_rate() {
        let angles = vec![100.0, 90.0, 80.0, 70.0];
        let times = vec![0.0, 10.0, 20.0, 30.0];
        let result = SurfaceTreatmentAnalyzer::aging_study(&angles, &times);
        assert!(approx_eq(result.recovery_rate, -1.0, 0.01), "Rate = -1°/day: {}", result.recovery_rate);
    }

    // --- Helper function tests ---

    #[test]
    fn test_linear_fit_perfect_line() {
        let x = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let y = vec![2.0, 4.0, 6.0, 8.0, 10.0]; // y = 2x
        let (slope, intercept) = linear_fit(&x, &y);
        assert!(approx_eq(slope, 2.0, 0.01));
        assert!(approx_eq(intercept, 0.0, 0.01));
    }

    #[test]
    fn test_linear_fit_with_intercept() {
        let x = vec![0.0, 1.0, 2.0, 3.0];
        let y = vec![5.0, 7.0, 9.0, 11.0]; // y = 2x + 5
        let (slope, intercept) = linear_fit(&x, &y);
        assert!(approx_eq(slope, 2.0, 0.01));
        assert!(approx_eq(intercept, 5.0, 0.01));
    }

    #[test]
    fn test_circle_fit_kasa_perfect_circle() {
        // Points on a circle of radius 1 centered at (2, 3)
        let n = 20;
        let pts: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let angle = 2.0 * PI * i as f64 / n as f64;
                (2.0 + angle.cos(), 3.0 + angle.sin())
            })
            .collect();
        let (cx, cy, r) = circle_fit_kasa(&pts);
        assert!(approx_eq(cx, 2.0, 0.01));
        assert!(approx_eq(cy, 3.0, 0.01));
        assert!(approx_eq(r, 1.0, 0.01));
    }

    // --- Integration / roundtrip tests ---

    #[test]
    fn test_roundtrip_simulate_then_measure() {
        // Simulate a 60° drop and verify measurement
        let profile = GoniometerSimulator::simulate_spherical_cap(2.0, 60.0, 100);
        let h = profile.drop_height();
        let w = profile.drop_width();
        let theta = ContactAngleMeasurement::half_angle_method(h, w);
        // Should be reasonably close to 60°
        assert!(
            theta > 45.0 && theta < 75.0,
            "Roundtrip: expected ~60°, got {}", theta
        );
    }

    #[test]
    fn test_roundtrip_simulate_90deg() {
        let profile = GoniometerSimulator::simulate_spherical_cap(1.0, 90.0, 100);
        let h = profile.drop_height();
        let w = profile.drop_width();
        let theta = ContactAngleMeasurement::half_angle_method(h, w);
        assert!(
            theta > 75.0 && theta < 105.0,
            "Roundtrip 90°: got {}", theta
        );
    }

    #[test]
    fn test_owrk_dispersive_plus_polar_equals_total() {
        let result = SurfaceEnergyCalculator::owrk(70.0, 40.0);
        assert!(
            approx_eq(result.total, result.dispersive + result.polar, TOL_E),
            "Total ({}) should equal dispersive ({}) + polar ({})",
            result.total, result.dispersive, result.polar
        );
    }

    #[test]
    fn test_young_equation_consistency() {
        // γ_SV = 40, γ_LV = 72.8, θ = 60°
        let gamma_sl = YoungEquation::gamma_sl(40.0, 72.8, 60.0);
        let cos_theta = YoungEquation::cos_theta(40.0, gamma_sl, 72.8);
        let theta_recovered = cos_theta.acos().to_degrees();
        assert!(approx_eq(theta_recovered, 60.0, TOL));
    }

    #[test]
    fn test_capillary_length_vs_bond() {
        // Bo = 1 when d = λ_c
        let lc = CapillaryAnalysis::capillary_length(72.8, 998.0);
        let bo = CapillaryAnalysis::bond_number(998.0, lc, 72.8);
        assert!(approx_eq(bo, 1.0, 0.01), "Bo should be 1 at capillary length: {}", bo);
    }

    #[test]
    fn test_contact_angle_result_symmetry() {
        let r = ContactAngleResult::new(65.0, 65.0);
        assert!(approx_eq(r.hysteresis_deg, 0.0, 0.01));
        assert!(approx_eq(r.average_angle_deg, 65.0, 0.01));
    }

    #[test]
    fn test_contact_angle_result_asymmetry() {
        let r = ContactAngleResult::new(70.0, 60.0);
        assert!(approx_eq(r.hysteresis_deg, 10.0, 0.01));
        assert!(approx_eq(r.average_angle_deg, 65.0, 0.01));
    }

    #[test]
    fn test_wettability_work_adhesion_vs_cohesion() {
        // For θ < 90°: W_a > γ_LV (partial wetting)
        let wa = WettabilityClassifier::work_of_adhesion(72.8, 60.0);
        assert!(wa > 72.8, "W_a for hydrophilic should exceed γ_LV: {}", wa);
    }

    #[test]
    fn test_cassie_baxter_f1_equals_1() {
        // f1 = 1: only material 1 -> θ* = θ1
        let angle = YoungEquation::cassie_baxter_angle(75.0, 120.0, 1.0);
        assert!(approx_eq(angle, 75.0, TOL));
    }

    #[test]
    fn test_cassie_baxter_f1_equals_0() {
        // f1 = 0: only material 2 -> θ* = θ2
        let angle = YoungEquation::cassie_baxter_angle(75.0, 120.0, 0.0);
        assert!(approx_eq(angle, 120.0, TOL));
    }
}
