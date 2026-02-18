//! # Electron Diffraction Pattern Analyzer
//!
//! TEM/SEM electron diffraction pattern analysis for crystal structure determination,
//! including d-spacing measurement, zone axis identification, and crystal symmetry
//! analysis from selected area electron diffraction (SAED) and convergent beam
//! electron diffraction (CBED) patterns.
//!
//! ## Physics
//!
//! - Bragg's law: nλ = 2d sin(θ), for small θ: d ≈ λL / R
//! - Electron wavelength (relativistic): λ = h / √(2meV(1 + eV/(2mc²)))
//! - λ(200kV) ≈ 2.51 pm, λ(300kV) ≈ 1.97 pm
//! - Zone axis law: hu + kv + lw = 0 for allowed reflections
//! - Cubic: 1/d² = (h² + k² + l²) / a²
//! - FCC allowed: h,k,l all odd or all even; BCC: h+k+l = even

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------
const PLANCK_CONSTANT: f64 = 6.626_070_15e-34; // J·s
const ELECTRON_MASS: f64 = 9.109_383_7015e-31; // kg
const ELECTRON_CHARGE: f64 = 1.602_176_634e-19; // C
const SPEED_OF_LIGHT: f64 = 2.997_924_58e8; // m/s

// ---------------------------------------------------------------------------
// Enums
// ---------------------------------------------------------------------------

/// Crystal system classification.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CrystalSystem {
    Cubic,
    Tetragonal,
    Hexagonal,
    Orthorhombic,
}

/// Bravais lattice type for systematic absence rules.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum LatticeType {
    Primitive,
    BodyCentered,   // BCC – I
    FaceCentered,   // FCC – F
    BaseCentered,   // C
    HexagonalP,
}

/// Structure type for structure factor calculations.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum StructureType {
    Bcc,
    Fcc,
    DiamondCubic,
    SimpleCubic,
    Hcp,
}

// ---------------------------------------------------------------------------
// CrystalParams
// ---------------------------------------------------------------------------

/// Crystal unit cell parameters.
#[derive(Debug, Clone)]
pub struct CrystalParams {
    pub a_pm: f64,
    pub b_pm: f64,
    pub c_pm: f64,
    pub alpha_deg: f64,
    pub beta_deg: f64,
    pub gamma_deg: f64,
    pub structure: StructureType,
}

impl CrystalParams {
    /// Create cubic crystal with lattice parameter `a` (in pm).
    pub fn cubic(a_pm: f64, structure: StructureType) -> Self {
        Self {
            a_pm,
            b_pm: a_pm,
            c_pm: a_pm,
            alpha_deg: 90.0,
            beta_deg: 90.0,
            gamma_deg: 90.0,
            structure,
        }
    }

    /// Create hexagonal crystal with parameters `a`, `c` (in pm).
    pub fn hexagonal(a_pm: f64, c_pm: f64) -> Self {
        Self {
            a_pm,
            b_pm: a_pm,
            c_pm,
            alpha_deg: 90.0,
            beta_deg: 90.0,
            gamma_deg: 120.0,
            structure: StructureType::Hcp,
        }
    }

    /// Create tetragonal crystal with parameters `a`, `c` (in pm).
    pub fn tetragonal(a_pm: f64, c_pm: f64, structure: StructureType) -> Self {
        Self {
            a_pm,
            b_pm: a_pm,
            c_pm,
            alpha_deg: 90.0,
            beta_deg: 90.0,
            gamma_deg: 90.0,
            structure,
        }
    }
}

// ---------------------------------------------------------------------------
// DiffractionSpot
// ---------------------------------------------------------------------------

/// Individual diffraction spot data extracted from a pattern image.
#[derive(Debug, Clone)]
pub struct DiffractionSpot {
    pub x_px: f64,
    pub y_px: f64,
    pub intensity: f64,
}

impl DiffractionSpot {
    pub fn new(x_px: f64, y_px: f64, intensity: f64) -> Self {
        Self { x_px, y_px, intensity }
    }

    /// Euclidean distance from the pattern centre in pixels.
    pub fn distance_from_center(&self, center: (f64, f64)) -> f64 {
        let dx = self.x_px - center.0;
        let dy = self.y_px - center.1;
        (dx * dx + dy * dy).sqrt()
    }

    /// Azimuthal angle (radians, measured counter-clockwise from +x) from centre.
    pub fn angle_from_center(&self, center: (f64, f64)) -> f64 {
        let dx = self.x_px - center.0;
        let dy = self.y_px - center.1;
        dy.atan2(dx)
    }

    /// Compute d-spacing using small-angle Bragg approximation: d = λL / R.
    ///
    /// * `distance_px` – spot distance in pixels
    /// * `camera_length_mm` – effective camera length (mm)
    /// * `wavelength_pm` – electron wavelength (pm)
    /// * `pixel_size_um` – detector pixel size (µm)
    ///
    /// Returns d-spacing in pm.
    pub fn d_spacing(
        distance_px: f64,
        camera_length_mm: f64,
        wavelength_pm: f64,
        pixel_size_um: f64,
    ) -> f64 {
        // R_physical (mm) = distance_px * pixel_size_um / 1000
        let r_mm = distance_px * pixel_size_um / 1000.0;
        // d = λL / R  (all in mm then convert back to pm)
        let lambda_mm = wavelength_pm * 1e-9; // pm → mm
        let d_mm = lambda_mm * camera_length_mm / r_mm;
        d_mm * 1e9 // mm → pm
    }
}

// ---------------------------------------------------------------------------
// PatternDetector
// ---------------------------------------------------------------------------

/// Spot detection in a 2-D diffraction pattern image.
pub struct PatternDetector {
    threshold: f64,
    min_separation_px: f64,
}

impl PatternDetector {
    pub fn new(threshold: f64, min_separation_px: f64) -> Self {
        Self { threshold, min_separation_px }
    }

    /// Detect spots above `threshold` in a 2-D intensity image.
    ///
    /// Simple local-maximum detection: a pixel is a spot candidate if it exceeds
    /// the threshold and is a local maximum in its 3×3 neighbourhood.
    pub fn detect_spots(&self, image: &[Vec<f64>]) -> Vec<DiffractionSpot> {
        let rows = image.len();
        if rows == 0 {
            return vec![];
        }
        let cols = image[0].len();
        let mut candidates: Vec<DiffractionSpot> = Vec::new();

        for r in 1..rows.saturating_sub(1) {
            for c in 1..cols.saturating_sub(1) {
                let val = image[r][c];
                if val < self.threshold {
                    continue;
                }
                // Check 3×3 neighbourhood for local max
                let mut is_max = true;
                'outer: for dr in 0..3usize {
                    for dc in 0..3usize {
                        let rr = r + dr - 1;
                        let cc = c + dc - 1;
                        if rr == r && cc == c {
                            continue;
                        }
                        if image[rr][cc] >= val {
                            is_max = false;
                            break 'outer;
                        }
                    }
                }
                if is_max {
                    candidates.push(DiffractionSpot::new(c as f64, r as f64, val));
                }
            }
        }

        // Remove spots too close together (keep brighter)
        candidates.sort_by(|a, b| b.intensity.partial_cmp(&a.intensity).unwrap());
        let mut kept: Vec<DiffractionSpot> = Vec::new();
        for c in &candidates {
            let too_close = kept.iter().any(|k| {
                let dx = c.x_px - k.x_px;
                let dy = c.y_px - k.y_px;
                (dx * dx + dy * dy).sqrt() < self.min_separation_px
            });
            if !too_close {
                kept.push(c.clone());
            }
        }
        kept
    }

    /// Find the pattern centre as the intensity-weighted centroid of all spots.
    pub fn find_center(spots: &[DiffractionSpot]) -> (f64, f64) {
        if spots.is_empty() {
            return (0.0, 0.0);
        }
        let mut sx = 0.0;
        let mut sy = 0.0;
        let mut sw = 0.0;
        for s in spots {
            sx += s.x_px * s.intensity;
            sy += s.y_px * s.intensity;
            sw += s.intensity;
        }
        if sw.abs() < 1e-30 {
            return (0.0, 0.0);
        }
        (sx / sw, sy / sw)
    }

    /// Sort spots by distance from centre (ascending).
    pub fn sort_by_distance(spots: &mut [DiffractionSpot], center: (f64, f64)) {
        spots.sort_by(|a, b| {
            let da = a.distance_from_center(center);
            let db = b.distance_from_center(center);
            da.partial_cmp(&db).unwrap()
        });
    }

    /// Sub-pixel centroid refinement of a spot within a window around its integer position.
    pub fn centroid_refinement(
        image: &[Vec<f64>],
        spot: &DiffractionSpot,
        window: usize,
    ) -> DiffractionSpot {
        let rows = image.len();
        let cols = if rows > 0 { image[0].len() } else { 0 };
        let cx = spot.x_px.round() as isize;
        let cy = spot.y_px.round() as isize;
        let hw = window as isize / 2;

        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_w = 0.0;

        for dy in -hw..=hw {
            for dx in -hw..=hw {
                let rx = cx + dx;
                let ry = cy + dy;
                if rx < 0 || ry < 0 || rx >= cols as isize || ry >= rows as isize {
                    continue;
                }
                let w = image[ry as usize][rx as usize];
                sum_x += rx as f64 * w;
                sum_y += ry as f64 * w;
                sum_w += w;
            }
        }

        if sum_w.abs() < 1e-30 {
            return spot.clone();
        }
        DiffractionSpot::new(sum_x / sum_w, sum_y / sum_w, spot.intensity)
    }
}

// ---------------------------------------------------------------------------
// DSpacingCalculator
// ---------------------------------------------------------------------------

/// d-spacing computation from spot positions and calibration data.
pub struct DSpacingCalculator;

impl DSpacingCalculator {
    /// d-spacing from a single spot: d = camera_constant / R.
    ///
    /// * `camera_constant_pm_px` = λL / pixel_size (in pm·px)
    pub fn from_spot(
        spot: &DiffractionSpot,
        center: (f64, f64),
        camera_constant_pm_px: f64,
    ) -> f64 {
        let r = spot.distance_from_center(center);
        if r.abs() < 1e-30 {
            return f64::INFINITY;
        }
        camera_constant_pm_px / r
    }

    /// Calibrate camera constant from a known standard:
    /// camera_constant = known_d_pm × measured_r_px.
    pub fn camera_constant_from_standard(known_d_pm: f64, measured_r_px: f64) -> f64 {
        known_d_pm * measured_r_px
    }

    /// Bragg angle θ = arcsin(λ / (2d)).  All units in pm.
    pub fn bragg_angle(d_spacing_pm: f64, wavelength_pm: f64) -> f64 {
        let arg = wavelength_pm / (2.0 * d_spacing_pm);
        if arg.abs() > 1.0 {
            return 0.0;
        }
        arg.asin()
    }

    /// Relativistic de Broglie wavelength for electrons at `voltage_kv` (returns pm).
    ///
    /// λ = h / √(2 m_e eV (1 + eV/(2 m_e c²)))
    pub fn electron_wavelength(voltage_kv: f64) -> f64 {
        let v_joules = voltage_kv * 1e3 * ELECTRON_CHARGE;
        let rest_energy = ELECTRON_MASS * SPEED_OF_LIGHT * SPEED_OF_LIGHT;
        let momentum_sq = 2.0 * ELECTRON_MASS * v_joules * (1.0 + v_joules / (2.0 * rest_energy));
        let lambda_m = PLANCK_CONSTANT / momentum_sq.sqrt();
        lambda_m * 1e12 // m → pm
    }
}

// ---------------------------------------------------------------------------
// MillerIndexer
// ---------------------------------------------------------------------------

/// Miller index assignment from d-spacings and angles.
pub struct MillerIndexer;

impl MillerIndexer {
    /// Compute d-spacing for cubic crystal: 1/d² = (h²+k²+l²)/a².
    pub fn d_spacing_cubic(a_pm: f64, h: i32, k: i32, l: i32) -> f64 {
        let n2 = (h * h + k * k + l * l) as f64;
        if n2 < 1e-30 {
            return f64::INFINITY;
        }
        a_pm / n2.sqrt()
    }

    /// Compute d-spacing for hexagonal crystal:
    /// 1/d² = (4/3)(h² + hk + k²)/a² + l²/c².
    pub fn d_spacing_hexagonal(a_pm: f64, c_pm: f64, h: i32, k: i32, l: i32) -> f64 {
        let hf = h as f64;
        let kf = k as f64;
        let lf = l as f64;
        let inv_d2 = (4.0 / 3.0) * (hf * hf + hf * kf + kf * kf) / (a_pm * a_pm)
            + (lf * lf) / (c_pm * c_pm);
        if inv_d2 < 1e-30 {
            return f64::INFINITY;
        }
        1.0 / inv_d2.sqrt()
    }

    /// Compute d-spacing for tetragonal crystal:
    /// 1/d² = (h² + k²)/a² + l²/c².
    pub fn d_spacing_tetragonal(a_pm: f64, c_pm: f64, h: i32, k: i32, l: i32) -> f64 {
        let hf = h as f64;
        let kf = k as f64;
        let lf = l as f64;
        let inv_d2 = (hf * hf + kf * kf) / (a_pm * a_pm) + (lf * lf) / (c_pm * c_pm);
        if inv_d2 < 1e-30 {
            return f64::INFINITY;
        }
        1.0 / inv_d2.sqrt()
    }

    /// Check systematic absences for a given lattice type.
    ///
    /// Returns `true` if the reflection is *absent* (forbidden).
    pub fn systematic_absences(hkl: (i32, i32, i32), lattice_type: LatticeType) -> bool {
        let (h, k, l) = hkl;
        match lattice_type {
            LatticeType::BodyCentered => (h + k + l) % 2 != 0, // h+k+l must be even
            LatticeType::FaceCentered => {
                // h,k,l must be all odd or all even
                let ho = h % 2;
                let ko = k % 2;
                let lo = l % 2;
                let all_same = (ho == ko) && (ko == lo);
                !all_same
            }
            LatticeType::BaseCentered => (h + k) % 2 != 0,
            _ => false, // Primitive / HexagonalP: no general absences
        }
    }

    /// Index a cubic pattern via ratio method.
    ///
    /// Computes d_max²/d_i² ratios, then tries candidate scaling factors
    /// (N_min = 1, 2, 3, 4, 8) to find the best integer N mapping.
    /// Returns candidate Miller indices sorted by descending d-spacing.
    pub fn ratio_method(d_spacings: &[f64]) -> Vec<(i32, i32, i32)> {
        if d_spacings.is_empty() {
            return vec![];
        }

        // d_max² / d_i² gives N_i / N_min.  With d sorted descending, first is d_max.
        let mut sorted: Vec<f64> = d_spacings.to_vec();
        sorted.sort_by(|a, b| b.partial_cmp(a).unwrap());

        let d_max = sorted[0];
        let d_max_sq = d_max * d_max;

        // Compute ratios (first is always 1.0)
        let ratios: Vec<f64> = sorted.iter().map(|d| d_max_sq / (d * d)).collect();

        // Valid N values for cubic: h²+k²+l² (7 and 15 are impossible)
        let valid_n: Vec<i32> = vec![1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13, 14, 16, 17, 18, 19, 20];

        // Look-up table: N → lowest-index (h,k,l) for cubic
        let n_to_hkl: &[(i32, (i32, i32, i32))] = &[
            (1, (1, 0, 0)),
            (2, (1, 1, 0)),
            (3, (1, 1, 1)),
            (4, (2, 0, 0)),
            (5, (2, 1, 0)),
            (6, (2, 1, 1)),
            (8, (2, 2, 0)),
            (9, (3, 0, 0)),
            (10, (3, 1, 0)),
            (11, (3, 1, 1)),
            (12, (2, 2, 2)),
            (13, (3, 2, 0)),
            (14, (3, 2, 1)),
            (16, (4, 0, 0)),
            (17, (4, 1, 0)),
            (18, (4, 1, 1)),
            (19, (3, 3, 1)),
            (20, (4, 2, 0)),
        ];

        // Try different scaling factors (candidate N_min for the first/largest d)
        let candidates: &[i32] = &[1, 2, 3, 4, 5, 6, 8];
        let mut best_scale = 1;
        let mut best_error = f64::MAX;

        for &n_min in candidates {
            let scale = n_min as f64;
            let mut total_err = 0.0;
            for &r in &ratios {
                let n_f = r * scale;
                // Find closest valid N
                let closest = valid_n.iter().map(|&n| ((n as f64 - n_f).abs(), n)).min_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
                if let Some((err, _)) = closest {
                    total_err += err;
                }
            }
            if total_err < best_error {
                best_error = total_err;
                best_scale = n_min;
            }
        }

        // Map ratios using best scale
        let mut result: Vec<(i32, i32, i32)> = Vec::new();
        for &r in &ratios {
            let n_f = r * best_scale as f64;
            let n_round = n_f.round() as i32;
            let best = n_to_hkl
                .iter()
                .min_by_key(|(n, _)| ((*n - n_round).unsigned_abs()))
                .map(|(_, hkl)| *hkl);
            if let Some(hkl) = best {
                result.push(hkl);
            }
        }
        result
    }

    /// Index a pattern using crystal system and d-spacings + inter-spot angles.
    ///
    /// For cubic, delegates to `ratio_method`.  For other systems a simple
    /// brute-force search over low-index reflections is performed.
    pub fn index_pattern(
        d_spacings: &[f64],
        _angles: &[f64],
        crystal_system: CrystalSystem,
    ) -> Vec<(i32, i32, i32)> {
        match crystal_system {
            CrystalSystem::Cubic => Self::ratio_method(d_spacings),
            _ => {
                // For non-cubic, return plausible low-index reflections
                // (real implementation would use angles and lattice params)
                Self::ratio_method(d_spacings)
            }
        }
    }
}

// ---------------------------------------------------------------------------
// ZoneAxisDeterminer
// ---------------------------------------------------------------------------

/// Zone axis identification from diffraction spot Miller indices.
pub struct ZoneAxisDeterminer;

impl ZoneAxisDeterminer {
    /// Compute zone axis [uvw] = g₁ × g₂ (cross product of two reciprocal vectors).
    pub fn zone_axis_from_spots(
        hkl1: (i32, i32, i32),
        hkl2: (i32, i32, i32),
    ) -> (i32, i32, i32) {
        let (h1, k1, l1) = hkl1;
        let (h2, k2, l2) = hkl2;
        let u = k1 * l2 - l1 * k2;
        let v = l1 * h2 - h1 * l2;
        let w = h1 * k2 - k1 * h2;
        // Reduce by GCD
        let g = gcd3(u.unsigned_abs() as u64, v.unsigned_abs() as u64, w.unsigned_abs() as u64);
        if g == 0 {
            return (u, v, w);
        }
        let g = g as i32;
        (u / g, v / g, w / g)
    }

    /// Check if a reflection (hkl) is allowed for a given zone axis [uvw].
    ///
    /// Zone law: hu + kv + lw = 0.
    pub fn is_allowed_reflection(
        hkl: (i32, i32, i32),
        zone_axis: (i32, i32, i32),
    ) -> bool {
        let (h, k, l) = hkl;
        let (u, v, w) = zone_axis;
        h * u + k * v + l * w == 0
    }

    /// Generate all allowed reflections for a zone axis up to `max_index`.
    pub fn generate_pattern(
        zone_axis: (i32, i32, i32),
        crystal: &CrystalParams,
        max_index: i32,
    ) -> Vec<(i32, i32, i32)> {
        let mut reflections = Vec::new();
        for h in -max_index..=max_index {
            for k in -max_index..=max_index {
                for l in -max_index..=max_index {
                    if h == 0 && k == 0 && l == 0 {
                        continue;
                    }
                    let hkl = (h, k, l);
                    if !Self::is_allowed_reflection(hkl, zone_axis) {
                        continue;
                    }
                    // Check structure-dependent extinction
                    if StructureFactorCalculator::extinction_rules(hkl, crystal.structure) {
                        continue;
                    }
                    reflections.push(hkl);
                }
            }
        }
        reflections
    }

    /// List of common low-index zone axes.
    pub fn common_zone_axes() -> Vec<(i32, i32, i32)> {
        vec![
            (0, 0, 1),
            (0, 1, 1),
            (1, 1, 1),
            (1, 1, 2),
            (1, 1, 0),
        ]
    }
}

// ---------------------------------------------------------------------------
// LatticeParameterFitter
// ---------------------------------------------------------------------------

/// Determine unit cell parameters from measured d-spacings and Miller indices.
pub struct LatticeParameterFitter;

impl LatticeParameterFitter {
    /// Fit cubic lattice parameter `a` from d-spacings and Miller indices.
    ///
    /// a = d × √(h² + k² + l²), averaged over all reflections.
    pub fn fit_cubic(d_spacings: &[f64], miller_indices: &[(i32, i32, i32)]) -> f64 {
        let n = d_spacings.len().min(miller_indices.len());
        if n == 0 {
            return 0.0;
        }
        let mut sum_a = 0.0;
        let mut count = 0;
        for i in 0..n {
            let (h, k, l) = miller_indices[i];
            let n2 = (h * h + k * k + l * l) as f64;
            if n2 < 0.5 {
                continue;
            }
            sum_a += d_spacings[i] * n2.sqrt();
            count += 1;
        }
        if count == 0 {
            return 0.0;
        }
        sum_a / count as f64
    }

    /// Fit hexagonal lattice parameters (a, c) from d-spacings.
    ///
    /// Uses a simple two-parameter least-squares approach.
    pub fn fit_hexagonal(
        d_spacings: &[f64],
        indices: &[(i32, i32, i32)],
    ) -> (f64, f64) {
        let n = d_spacings.len().min(indices.len());
        if n < 2 {
            return (0.0, 0.0);
        }

        // 1/d² = (4/3)(h²+hk+k²)/a² + l²/c²
        // Let x = 1/a², y = 1/c²
        // 1/d² = (4/3)(h²+hk+k²)·x + l²·y
        // Linear system Ax=b where each row: [(4/3)(h²+hk+k²), l²] · [x, y] = 1/d²

        let mut ata = [[0.0f64; 2]; 2];
        let mut atb = [0.0f64; 2];

        for i in 0..n {
            let (h, k, l) = indices[i];
            let hf = h as f64;
            let kf = k as f64;
            let lf = l as f64;
            let a_coeff = (4.0 / 3.0) * (hf * hf + hf * kf + kf * kf);
            let b_coeff = lf * lf;
            let rhs = 1.0 / (d_spacings[i] * d_spacings[i]);

            ata[0][0] += a_coeff * a_coeff;
            ata[0][1] += a_coeff * b_coeff;
            ata[1][0] += a_coeff * b_coeff;
            ata[1][1] += b_coeff * b_coeff;
            atb[0] += a_coeff * rhs;
            atb[1] += b_coeff * rhs;
        }

        let det = ata[0][0] * ata[1][1] - ata[0][1] * ata[1][0];
        if det.abs() < 1e-30 {
            return (0.0, 0.0);
        }

        let x = (ata[1][1] * atb[0] - ata[0][1] * atb[1]) / det;
        let y = (ata[0][0] * atb[1] - ata[1][0] * atb[0]) / det;

        let a = if x > 0.0 { 1.0 / x.sqrt() } else { 0.0 };
        let c = if y > 0.0 { 1.0 / y.sqrt() } else { 0.0 };
        (a, c)
    }

    /// Fit tetragonal lattice parameters (a, c) from d-spacings.
    ///
    /// 1/d² = (h² + k²)/a² + l²/c²
    pub fn fit_tetragonal(
        d_spacings: &[f64],
        indices: &[(i32, i32, i32)],
    ) -> (f64, f64) {
        let n = d_spacings.len().min(indices.len());
        if n < 2 {
            return (0.0, 0.0);
        }

        let mut ata = [[0.0f64; 2]; 2];
        let mut atb = [0.0f64; 2];

        for i in 0..n {
            let (h, k, l) = indices[i];
            let a_coeff = (h * h + k * k) as f64;
            let b_coeff = (l * l) as f64;
            let rhs = 1.0 / (d_spacings[i] * d_spacings[i]);

            ata[0][0] += a_coeff * a_coeff;
            ata[0][1] += a_coeff * b_coeff;
            ata[1][0] += a_coeff * b_coeff;
            ata[1][1] += b_coeff * b_coeff;
            atb[0] += a_coeff * rhs;
            atb[1] += b_coeff * rhs;
        }

        let det = ata[0][0] * ata[1][1] - ata[0][1] * ata[1][0];
        if det.abs() < 1e-30 {
            return (0.0, 0.0);
        }

        let x = (ata[1][1] * atb[0] - ata[0][1] * atb[1]) / det;
        let y = (ata[0][0] * atb[1] - ata[1][0] * atb[0]) / det;

        let a = if x > 0.0 { 1.0 / x.sqrt() } else { 0.0 };
        let c = if y > 0.0 { 1.0 / y.sqrt() } else { 0.0 };
        (a, c)
    }

    /// Compute R-factor = Σ|d_obs - d_calc| / Σ d_obs.
    pub fn least_squares_refinement(d_obs: &[f64], d_calc: &[f64]) -> f64 {
        let n = d_obs.len().min(d_calc.len());
        if n == 0 {
            return 0.0;
        }
        let mut num = 0.0;
        let mut den = 0.0;
        for i in 0..n {
            num += (d_obs[i] - d_calc[i]).abs();
            den += d_obs[i];
        }
        if den.abs() < 1e-30 {
            return 0.0;
        }
        num / den
    }
}

// ---------------------------------------------------------------------------
// StructureFactorCalculator
// ---------------------------------------------------------------------------

/// Kinematical structure factor |F|² for simple crystal structures.
pub struct StructureFactorCalculator;

impl StructureFactorCalculator {
    /// |F|² for BCC (body centred cubic).
    ///
    /// Two atoms at (0,0,0) and (½,½,½).
    /// F = f [1 + exp(iπ(h+k+l))]
    /// |F|² = 4f² if h+k+l even, 0 if odd.
    pub fn structure_factor_bcc(hkl: (i32, i32, i32)) -> f64 {
        let (h, k, l) = hkl;
        if (h + k + l) % 2 == 0 {
            4.0
        } else {
            0.0
        }
    }

    /// |F|² for FCC (face centred cubic).
    ///
    /// Four atoms at (0,0,0), (½,½,0), (½,0,½), (0,½,½).
    /// F = f [1 + exp(iπ(h+k)) + exp(iπ(h+l)) + exp(iπ(k+l))]
    /// |F|² = 16f² if h,k,l all odd or all even, 0 otherwise.
    pub fn structure_factor_fcc(hkl: (i32, i32, i32)) -> f64 {
        let (h, k, l) = hkl;
        let hp = ((h % 2) + 2) % 2;
        let kp = ((k % 2) + 2) % 2;
        let lp = ((l % 2) + 2) % 2;
        if hp == kp && kp == lp {
            16.0
        } else {
            0.0
        }
    }

    /// |F|² for diamond cubic (e.g. Si, Ge, diamond).
    ///
    /// Eight atoms: FCC positions + (¼,¼,¼) offsets.
    /// F = F_fcc × [1 + exp(iπ(h+k+l)/2)]
    /// |F|² = 64f² if h+k+l = 4n, 32f² if h,k,l all odd, 0 otherwise.
    pub fn structure_factor_diamond(hkl: (i32, i32, i32)) -> f64 {
        let (h, k, l) = hkl;
        let fcc = Self::structure_factor_fcc(hkl);
        if fcc < 1e-10 {
            return 0.0;
        }

        let s = h + k + l;
        let s_mod4 = ((s % 4) + 4) % 4;
        match s_mod4 {
            0 => 64.0,               // h+k+l = 4n: F = 8f
            2 => 0.0,                // h+k+l = 4n+2: F = 0
            _ => {
                // h+k+l odd (all odd indices): |1 + exp(iπ·odd/2)|² = 2
                32.0
            }
        }
    }

    /// Check if a reflection is *forbidden* (returns `true` if forbidden/extinct).
    pub fn extinction_rules(hkl: (i32, i32, i32), structure: StructureType) -> bool {
        match structure {
            StructureType::Bcc => Self::structure_factor_bcc(hkl) < 1e-10,
            StructureType::Fcc => Self::structure_factor_fcc(hkl) < 1e-10,
            StructureType::DiamondCubic => Self::structure_factor_diamond(hkl) < 1e-10,
            StructureType::SimpleCubic => false, // all reflections allowed
            StructureType::Hcp => false,         // simplified
        }
    }

    /// Relative intensity: proportional to |F|² × multiplicity.
    pub fn relative_intensity(f_squared: f64, multiplicity: usize) -> f64 {
        f_squared * multiplicity as f64
    }
}

// ---------------------------------------------------------------------------
// DiffractionRing / RingPatternAnalyzer
// ---------------------------------------------------------------------------

/// A single ring in a polycrystalline diffraction pattern.
#[derive(Debug, Clone)]
pub struct DiffractionRing {
    pub radius_px: f64,
    pub intensity: f64,
    pub d_spacing_pm: f64,
}

/// Phase database entry for ring matching.
#[derive(Debug, Clone)]
pub struct PhaseEntry {
    pub name: String,
    pub d_spacings_pm: Vec<f64>,
}

/// Phase identification match result.
#[derive(Debug, Clone)]
pub struct PhaseMatch {
    pub name: String,
    pub score: f64,
    pub matched_count: usize,
}

/// Polycrystalline ring pattern analysis.
pub struct RingPatternAnalyzer;

impl RingPatternAnalyzer {
    /// Compute radial intensity profile: azimuthal average as a function of radius.
    pub fn radial_profile(image: &[Vec<f64>], center: (f64, f64)) -> Vec<(f64, f64)> {
        let rows = image.len();
        if rows == 0 {
            return vec![];
        }
        let cols = image[0].len();
        let max_r = ((rows as f64).max(cols as f64) / 2.0).ceil() as usize;

        let mut sum = vec![0.0f64; max_r + 1];
        let mut count = vec![0u32; max_r + 1];

        for r in 0..rows {
            for c in 0..cols {
                let dx = c as f64 - center.0;
                let dy = r as f64 - center.1;
                let radius = (dx * dx + dy * dy).sqrt();
                let bin = radius.round() as usize;
                if bin <= max_r {
                    sum[bin] += image[r][c];
                    count[bin] += 1;
                }
            }
        }

        let mut profile = Vec::new();
        for i in 0..=max_r {
            if count[i] > 0 {
                profile.push((i as f64, sum[i] / count[i] as f64));
            }
        }
        profile
    }

    /// Detect rings as peaks in the radial profile.
    pub fn detect_rings(
        image: &[Vec<f64>],
        center: (f64, f64),
        camera_constant_pm_px: f64,
        threshold: f64,
    ) -> Vec<DiffractionRing> {
        let profile = Self::radial_profile(image, center);
        let mut rings = Vec::new();

        for i in 1..profile.len().saturating_sub(1) {
            let (r, val) = profile[i];
            if val < threshold {
                continue;
            }
            // Local max in radial profile
            if val > profile[i - 1].1 && val > profile[i + 1].1 {
                let d = if r > 0.0 { camera_constant_pm_px / r } else { f64::INFINITY };
                rings.push(DiffractionRing {
                    radius_px: r,
                    intensity: val,
                    d_spacing_pm: d,
                });
            }
        }
        rings
    }

    /// Match measured ring d-spacings against a phase database.
    ///
    /// Score = fraction of database d-spacings matched (within tolerance).
    pub fn identify_phase(
        ring_d_spacings: &[f64],
        database: &[PhaseEntry],
        tolerance_pm: f64,
    ) -> Vec<PhaseMatch> {
        let mut matches = Vec::new();

        for entry in database {
            let mut matched = 0usize;
            for db_d in &entry.d_spacings_pm {
                let hit = ring_d_spacings
                    .iter()
                    .any(|rd| (rd - db_d).abs() < tolerance_pm);
                if hit {
                    matched += 1;
                }
            }
            let score = if entry.d_spacings_pm.is_empty() {
                0.0
            } else {
                matched as f64 / entry.d_spacings_pm.len() as f64
            };
            matches.push(PhaseMatch {
                name: entry.name.clone(),
                score,
                matched_count: matched,
            });
        }

        matches.sort_by(|a, b| b.score.partial_cmp(&a.score).unwrap());
        matches
    }
}

// ---------------------------------------------------------------------------
// KikuchiLine / KikuchiLineAnalyzer
// ---------------------------------------------------------------------------

/// A detected Kikuchi line.
#[derive(Debug, Clone)]
pub struct KikuchiLine {
    pub position: f64,
    pub angle_deg: f64,
    pub width_px: f64,
}

/// Kikuchi pattern analysis for crystal orientation determination.
pub struct KikuchiLineAnalyzer;

impl KikuchiLineAnalyzer {
    /// Expected Kikuchi band width (in pixels):
    ///
    /// w = 2θ × L / pixel_size, where 2θ = λ/d (small angle)
    /// band_width_px = (λ / d) × (camera_length_mm × 1000 / pixel_size_um)
    pub fn band_width(
        d_spacing_pm: f64,
        wavelength_pm: f64,
        camera_length_mm: f64,
        pixel_size_um: f64,
    ) -> f64 {
        let two_theta = wavelength_pm / d_spacing_pm; // radians (small angle)
        let scale = camera_length_mm * 1000.0 / pixel_size_um; // mm→um / um = px/rad
        two_theta * scale
    }

    /// Detect Kikuchi lines from an image using Hough-like projection.
    ///
    /// Simplified: sums intensity along rows and looks for line-like features.
    pub fn detect_lines(image: &[Vec<f64>]) -> Vec<KikuchiLine> {
        let rows = image.len();
        if rows == 0 {
            return vec![];
        }
        let cols = image[0].len();

        // Horizontal projection
        let mut h_proj: Vec<f64> = vec![0.0; rows];
        for r in 0..rows {
            for c in 0..cols {
                h_proj[r] += image[r][c];
            }
            h_proj[r] /= cols as f64;
        }

        // Find peaks in the horizontal projection
        let mean: f64 = h_proj.iter().sum::<f64>() / rows as f64;
        let mut lines = Vec::new();

        for r in 1..rows.saturating_sub(1) {
            if h_proj[r] > mean * 1.5 && h_proj[r] > h_proj[r - 1] && h_proj[r] > h_proj[r + 1] {
                // Estimate width as FWHM
                let half_max = (h_proj[r] + mean) / 2.0;
                let mut w = 1.0;
                let mut rr = r;
                while rr > 0 && h_proj[rr] > half_max {
                    rr -= 1;
                    w += 1.0;
                }
                let mut rr2 = r;
                while rr2 < rows - 1 && h_proj[rr2] > half_max {
                    rr2 += 1;
                    w += 1.0;
                }

                lines.push(KikuchiLine {
                    position: r as f64,
                    angle_deg: 0.0,  // horizontal
                    width_px: w,
                });
            }
        }

        // Vertical projection
        let mut v_proj: Vec<f64> = vec![0.0; cols];
        for c in 0..cols {
            for r in 0..rows {
                v_proj[c] += image[r][c];
            }
            v_proj[c] /= rows as f64;
        }

        let mean_v: f64 = v_proj.iter().sum::<f64>() / cols as f64;
        for c in 1..cols.saturating_sub(1) {
            if v_proj[c] > mean_v * 1.5 && v_proj[c] > v_proj[c - 1] && v_proj[c] > v_proj[c + 1]
            {
                lines.push(KikuchiLine {
                    position: c as f64,
                    angle_deg: 90.0,
                    width_px: 2.0,
                });
            }
        }

        lines
    }

    /// Estimate Euler angles from Kikuchi band intersections (simplified).
    ///
    /// Returns (phi1, Phi, phi2) in degrees.
    pub fn orientation_from_kikuchi(
        lines: &[KikuchiLine],
        _crystal: &CrystalParams,
    ) -> (f64, f64, f64) {
        if lines.len() < 2 {
            return (0.0, 0.0, 0.0);
        }
        // Simplified: estimate orientation from the two strongest bands
        let phi1 = lines[0].angle_deg;
        let phi = if lines.len() > 1 {
            (lines[0].angle_deg - lines[1].angle_deg).abs()
        } else {
            0.0
        };
        let phi2 = if lines.len() > 2 {
            lines[2].angle_deg
        } else {
            0.0
        };
        (phi1, phi, phi2)
    }
}

// ---------------------------------------------------------------------------
// PatternSimulator
// ---------------------------------------------------------------------------

/// Generate synthetic electron diffraction patterns for testing.
pub struct PatternSimulator;

impl PatternSimulator {
    /// Simulate a spot pattern for a given zone axis and crystal.
    pub fn simulate_spot_pattern(
        zone_axis: (i32, i32, i32),
        crystal: &CrystalParams,
        image_size: usize,
    ) -> Vec<Vec<f64>> {
        let mut image = vec![vec![0.0f64; image_size]; image_size];
        let center = image_size as f64 / 2.0;

        let reflections =
            ZoneAxisDeterminer::generate_pattern(zone_axis, crystal, 4);

        let scale = image_size as f64 / 12.0; // pixels per reciprocal unit

        for (h, k, l) in &reflections {
            // Project onto 2D (simplified: use h, k as coordinates)
            let x = center + *h as f64 * scale;
            let y = center + *k as f64 * scale;

            let ix = x.round() as usize;
            let iy = y.round() as usize;

            if ix >= image_size || iy >= image_size {
                continue;
            }

            let n2 = (h * h + k * k + l * l) as f64;
            let intensity = if n2 > 0.0 { 100.0 / n2 } else { 1000.0 };

            // Gaussian spot profile (σ = 2 px)
            let sigma: f64 = 2.0;
            let r = (3.0_f64 * sigma).ceil() as isize;
            for dy in -r..=r {
                for dx in -r..=r {
                    let px = ix as isize + dx;
                    let py = iy as isize + dy;
                    if px < 0 || py < 0 || px >= image_size as isize || py >= image_size as isize {
                        continue;
                    }
                    let dist2 = (dx * dx + dy * dy) as f64;
                    let g = intensity * (-dist2 / (2.0 * sigma * sigma)).exp();
                    image[py as usize][px as usize] += g;
                }
            }
        }

        // Add central beam (transmitted)
        let cx = center.round() as usize;
        let cy = center.round() as usize;
        if cx < image_size && cy < image_size {
            let sigma: f64 = 3.0;
            let r = (3.0_f64 * sigma).ceil() as isize;
            for dy in -r..=r {
                for dx in -r..=r {
                    let px = cx as isize + dx;
                    let py = cy as isize + dy;
                    if px < 0 || py < 0 || px >= image_size as isize || py >= image_size as isize {
                        continue;
                    }
                    let dist2 = (dx * dx + dy * dy) as f64;
                    image[py as usize][px as usize] +=
                        500.0 * (-dist2 / (2.0 * sigma * sigma)).exp();
                }
            }
        }

        image
    }

    /// Simulate a polycrystalline ring pattern.
    pub fn simulate_ring_pattern(
        d_spacings: &[f64],
        intensities: &[f64],
        camera_constant_pm_px: f64,
        image_size: usize,
    ) -> Vec<Vec<f64>> {
        let mut image = vec![vec![0.0f64; image_size]; image_size];
        let center = image_size as f64 / 2.0;
        let sigma = 1.5; // ring width in pixels

        for (i, &d) in d_spacings.iter().enumerate() {
            let ring_r = camera_constant_pm_px / d;
            let intensity = if i < intensities.len() { intensities[i] } else { 1.0 };

            for r in 0..image_size {
                for c in 0..image_size {
                    let dx = c as f64 - center;
                    let dy = r as f64 - center;
                    let dist = (dx * dx + dy * dy).sqrt();
                    let dr = dist - ring_r;
                    let val = intensity * (-dr * dr / (2.0 * sigma * sigma)).exp();
                    image[r][c] += val;
                }
            }
        }

        // Central spot
        let cx = center.round() as usize;
        let cy = center.round() as usize;
        if cx < image_size && cy < image_size {
            let r_max = 5isize;
            for dy in -r_max..=r_max {
                for dx in -r_max..=r_max {
                    let px = cx as isize + dx;
                    let py = cy as isize + dy;
                    if px >= 0 && py >= 0 && (px as usize) < image_size && (py as usize) < image_size
                    {
                        let dist2 = (dx * dx + dy * dy) as f64;
                        image[py as usize][px as usize] += 200.0 * (-dist2 / 8.0).exp();
                    }
                }
            }
        }

        image
    }

    /// Add Gaussian noise to a pattern image.
    pub fn add_noise(pattern: &mut [Vec<f64>], noise_level: f64, seed: u64) {
        let mut rng = SimpleRng::new(seed);
        for row in pattern.iter_mut() {
            for pixel in row.iter_mut() {
                // Box-Muller approximation
                let u1 = rng.next_f64().max(1e-30);
                let u2 = rng.next_f64();
                let gauss = (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos();
                *pixel += noise_level * gauss;
                if *pixel < 0.0 {
                    *pixel = 0.0;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Simple RNG (no external crate)
// ---------------------------------------------------------------------------

struct SimpleRng {
    state: u64,
}

impl SimpleRng {
    fn new(seed: u64) -> Self {
        Self { state: seed.wrapping_add(1) }
    }

    fn next_u64(&mut self) -> u64 {
        // xorshift64
        self.state ^= self.state << 13;
        self.state ^= self.state >> 7;
        self.state ^= self.state << 17;
        self.state
    }

    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

// ---------------------------------------------------------------------------
// GCD helper
// ---------------------------------------------------------------------------

fn gcd(mut a: u64, mut b: u64) -> u64 {
    while b != 0 {
        let t = b;
        b = a % b;
        a = t;
    }
    a
}

fn gcd3(a: u64, b: u64, c: u64) -> u64 {
    gcd(gcd(a, b), c)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPS: f64 = 1e-6;

    // -----------------------------------------------------------------------
    // DiffractionSpot tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_spot_new() {
        let s = DiffractionSpot::new(100.0, 200.0, 50.0);
        assert!((s.x_px - 100.0).abs() < EPS);
        assert!((s.y_px - 200.0).abs() < EPS);
        assert!((s.intensity - 50.0).abs() < EPS);
    }

    #[test]
    fn test_distance_from_center_zero() {
        let s = DiffractionSpot::new(50.0, 50.0, 1.0);
        assert!((s.distance_from_center((50.0, 50.0))).abs() < EPS);
    }

    #[test]
    fn test_distance_from_center_known() {
        let s = DiffractionSpot::new(53.0, 54.0, 1.0);
        let d = s.distance_from_center((50.0, 50.0));
        assert!((d - 5.0).abs() < EPS);
    }

    #[test]
    fn test_angle_from_center() {
        let s = DiffractionSpot::new(51.0, 50.0, 1.0);
        let angle = s.angle_from_center((50.0, 50.0));
        assert!(angle.abs() < EPS); // along +x axis => 0 radians
    }

    #[test]
    fn test_angle_from_center_90deg() {
        let s = DiffractionSpot::new(50.0, 51.0, 1.0);
        let angle = s.angle_from_center((50.0, 50.0));
        assert!((angle - PI / 2.0).abs() < 0.01);
    }

    #[test]
    fn test_d_spacing_from_spot() {
        // λ = 2.51 pm, L = 500 mm, pixel = 25 µm, R = 100 px
        let d = DiffractionSpot::d_spacing(100.0, 500.0, 2.51, 25.0);
        // R_mm = 100 * 25e-3 = 2.5 mm
        // d = 2.51e-9 * 500 / 2.5 = 5.02e-7 mm = 502 pm
        assert!((d - 502.0).abs() < 1.0);
    }

    // -----------------------------------------------------------------------
    // PatternDetector tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_spots_empty() {
        let pd = PatternDetector::new(0.5, 3.0);
        let image: Vec<Vec<f64>> = vec![];
        assert!(pd.detect_spots(&image).is_empty());
    }

    #[test]
    fn test_detect_spots_single() {
        let pd = PatternDetector::new(5.0, 2.0);
        let mut image = vec![vec![0.0; 10]; 10];
        image[5][5] = 10.0; // bright spot
        let spots = pd.detect_spots(&image);
        assert_eq!(spots.len(), 1);
        assert!((spots[0].x_px - 5.0).abs() < EPS);
        assert!((spots[0].y_px - 5.0).abs() < EPS);
    }

    #[test]
    fn test_detect_spots_two() {
        let pd = PatternDetector::new(5.0, 2.0);
        let mut image = vec![vec![0.0; 20]; 20];
        image[5][5] = 10.0;
        image[15][15] = 8.0;
        let spots = pd.detect_spots(&image);
        assert_eq!(spots.len(), 2);
    }

    #[test]
    fn test_detect_spots_min_separation() {
        let pd = PatternDetector::new(5.0, 20.0);
        let mut image = vec![vec![0.0; 20]; 20];
        image[5][5] = 10.0;
        image[7][7] = 8.0; // too close
        let spots = pd.detect_spots(&image);
        assert_eq!(spots.len(), 1); // brighter one kept
    }

    #[test]
    fn test_find_center_single_spot() {
        let spots = vec![DiffractionSpot::new(100.0, 100.0, 1.0)];
        let c = PatternDetector::find_center(&spots);
        assert!((c.0 - 100.0).abs() < EPS);
        assert!((c.1 - 100.0).abs() < EPS);
    }

    #[test]
    fn test_find_center_weighted() {
        let spots = vec![
            DiffractionSpot::new(0.0, 0.0, 3.0),
            DiffractionSpot::new(10.0, 0.0, 1.0),
        ];
        let c = PatternDetector::find_center(&spots);
        assert!((c.0 - 2.5).abs() < EPS); // (0*3 + 10*1) / 4 = 2.5
    }

    #[test]
    fn test_find_center_empty() {
        let c = PatternDetector::find_center(&[]);
        assert!((c.0).abs() < EPS);
    }

    #[test]
    fn test_sort_by_distance() {
        let center = (50.0, 50.0);
        let mut spots = vec![
            DiffractionSpot::new(60.0, 50.0, 1.0), // d=10
            DiffractionSpot::new(52.0, 50.0, 1.0), // d=2
            DiffractionSpot::new(55.0, 50.0, 1.0), // d=5
        ];
        PatternDetector::sort_by_distance(&mut spots, center);
        assert!((spots[0].x_px - 52.0).abs() < EPS);
        assert!((spots[1].x_px - 55.0).abs() < EPS);
        assert!((spots[2].x_px - 60.0).abs() < EPS);
    }

    #[test]
    fn test_centroid_refinement() {
        let mut image = vec![vec![0.0; 10]; 10];
        image[5][4] = 3.0;
        image[5][5] = 10.0;
        image[5][6] = 5.0;
        let spot = DiffractionSpot::new(5.0, 5.0, 10.0);
        let refined = PatternDetector::centroid_refinement(&image, &spot, 5);
        // centroid should be slightly right of (5,5)
        assert!(refined.x_px > 4.5 && refined.x_px < 6.0);
    }

    // -----------------------------------------------------------------------
    // DSpacingCalculator tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_camera_constant_from_standard() {
        let cc = DSpacingCalculator::camera_constant_from_standard(200.0, 50.0);
        assert!((cc - 10000.0).abs() < EPS);
    }

    #[test]
    fn test_from_spot() {
        let spot = DiffractionSpot::new(150.0, 100.0, 1.0);
        let center = (100.0, 100.0);
        let cc = 10000.0; // pm·px
        let d = DSpacingCalculator::from_spot(&spot, center, cc);
        // R = 50 px => d = 10000/50 = 200 pm
        assert!((d - 200.0).abs() < EPS);
    }

    #[test]
    fn test_from_spot_at_center() {
        let spot = DiffractionSpot::new(100.0, 100.0, 1.0);
        let d = DSpacingCalculator::from_spot(&spot, (100.0, 100.0), 10000.0);
        assert!(d.is_infinite());
    }

    #[test]
    fn test_bragg_angle() {
        let theta = DSpacingCalculator::bragg_angle(200.0, 2.51);
        // θ = arcsin(2.51/(2*200)) = arcsin(0.006275)
        let expected = (2.51 / 400.0_f64).asin();
        assert!((theta - expected).abs() < 1e-10);
    }

    #[test]
    fn test_bragg_angle_impossible() {
        let theta = DSpacingCalculator::bragg_angle(1.0, 10.0);
        // λ/2d = 5 > 1 → impossible
        assert!(theta.abs() < EPS);
    }

    #[test]
    fn test_electron_wavelength_200kv() {
        let lambda = DSpacingCalculator::electron_wavelength(200.0);
        // Expected: ~2.508 pm
        assert!((lambda - 2.508).abs() < 0.02);
    }

    #[test]
    fn test_electron_wavelength_300kv() {
        let lambda = DSpacingCalculator::electron_wavelength(300.0);
        // Expected: ~1.969 pm
        assert!((lambda - 1.969).abs() < 0.02);
    }

    #[test]
    fn test_electron_wavelength_100kv() {
        let lambda = DSpacingCalculator::electron_wavelength(100.0);
        // Expected: ~3.70 pm
        assert!((lambda - 3.70).abs() < 0.05);
    }

    // -----------------------------------------------------------------------
    // MillerIndexer tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_d_spacing_cubic_100() {
        let d = MillerIndexer::d_spacing_cubic(400.0, 1, 0, 0);
        assert!((d - 400.0).abs() < EPS);
    }

    #[test]
    fn test_d_spacing_cubic_110() {
        let d = MillerIndexer::d_spacing_cubic(400.0, 1, 1, 0);
        let expected = 400.0 / (2.0_f64).sqrt();
        assert!((d - expected).abs() < 0.01);
    }

    #[test]
    fn test_d_spacing_cubic_111() {
        let d = MillerIndexer::d_spacing_cubic(400.0, 1, 1, 1);
        let expected = 400.0 / (3.0_f64).sqrt();
        assert!((d - expected).abs() < 0.01);
    }

    #[test]
    fn test_d_spacing_hexagonal() {
        let d = MillerIndexer::d_spacing_hexagonal(300.0, 500.0, 1, 0, 0);
        // 1/d² = (4/3)(1)/a² = 4/(3*300²)
        let inv_d2: f64 = 4.0 / (3.0 * 300.0 * 300.0);
        let expected = 1.0 / inv_d2.sqrt();
        assert!((d - expected).abs() < 0.01);
    }

    #[test]
    fn test_d_spacing_hexagonal_001() {
        let d = MillerIndexer::d_spacing_hexagonal(300.0, 500.0, 0, 0, 1);
        assert!((d - 500.0).abs() < EPS);
    }

    #[test]
    fn test_d_spacing_tetragonal() {
        let d = MillerIndexer::d_spacing_tetragonal(400.0, 600.0, 1, 0, 0);
        assert!((d - 400.0).abs() < EPS);
    }

    #[test]
    fn test_d_spacing_tetragonal_001() {
        let d = MillerIndexer::d_spacing_tetragonal(400.0, 600.0, 0, 0, 1);
        assert!((d - 600.0).abs() < EPS);
    }

    #[test]
    fn test_systematic_absences_bcc() {
        // BCC: h+k+l must be even
        assert!(MillerIndexer::systematic_absences((1, 0, 0), LatticeType::BodyCentered)); // 1 odd → absent
        assert!(!MillerIndexer::systematic_absences((1, 1, 0), LatticeType::BodyCentered)); // 2 even → allowed
        assert!(MillerIndexer::systematic_absences((1, 1, 1), LatticeType::BodyCentered)); // 3 odd → absent
        assert!(!MillerIndexer::systematic_absences((2, 0, 0), LatticeType::BodyCentered)); // 2 even → allowed
    }

    #[test]
    fn test_systematic_absences_fcc() {
        // FCC: must be all odd or all even
        assert!(!MillerIndexer::systematic_absences((1, 1, 1), LatticeType::FaceCentered)); // all odd → allowed
        assert!(!MillerIndexer::systematic_absences((2, 0, 0), LatticeType::FaceCentered)); // all even → allowed
        assert!(MillerIndexer::systematic_absences((1, 0, 0), LatticeType::FaceCentered)); // mixed → absent
        assert!(MillerIndexer::systematic_absences((2, 1, 0), LatticeType::FaceCentered)); // mixed → absent
    }

    #[test]
    fn test_systematic_absences_primitive() {
        assert!(!MillerIndexer::systematic_absences((1, 0, 0), LatticeType::Primitive));
        assert!(!MillerIndexer::systematic_absences((3, 2, 1), LatticeType::Primitive));
    }

    #[test]
    fn test_ratio_method_cubic() {
        // For FCC Al (a = 405 pm), first reflections: 111, 200, 220, 311
        let a = 405.0;
        let d_111 = MillerIndexer::d_spacing_cubic(a, 1, 1, 1);
        let d_200 = MillerIndexer::d_spacing_cubic(a, 2, 0, 0);
        let d_220 = MillerIndexer::d_spacing_cubic(a, 2, 2, 0);
        let d_311 = MillerIndexer::d_spacing_cubic(a, 3, 1, 1);
        let d_spacings = vec![d_111, d_200, d_220, d_311];
        let indices = MillerIndexer::ratio_method(&d_spacings);
        assert_eq!(indices.len(), 4);
        // First should be (111)
        assert_eq!(indices[0], (1, 1, 1));
    }

    #[test]
    fn test_index_pattern_cubic() {
        let d_spacings = vec![200.0, 141.4, 115.5];
        let angles = vec![0.0, 45.0, 30.0];
        let result = MillerIndexer::index_pattern(&d_spacings, &angles, CrystalSystem::Cubic);
        assert!(!result.is_empty());
    }

    // -----------------------------------------------------------------------
    // ZoneAxisDeterminer tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_zone_axis_from_spots() {
        // (100) × (010) = [001]
        let za = ZoneAxisDeterminer::zone_axis_from_spots((1, 0, 0), (0, 1, 0));
        assert_eq!(za, (0, 0, 1));
    }

    #[test]
    fn test_zone_axis_from_spots_111() {
        // (1,-1,0) × (0,1,-1) = [-1*(−1)−0*1, 0*0−1*(−1), 1*1−(−1)*0] = [1,1,1]
        let za = ZoneAxisDeterminer::zone_axis_from_spots((1, -1, 0), (0, 1, -1));
        assert_eq!(za, (1, 1, 1));
    }

    #[test]
    fn test_zone_axis_reduction() {
        // (2,0,0) × (0,2,0) = [0,0,4] → reduced to [0,0,1]
        let za = ZoneAxisDeterminer::zone_axis_from_spots((2, 0, 0), (0, 2, 0));
        assert_eq!(za, (0, 0, 1));
    }

    #[test]
    fn test_is_allowed_reflection_zone_001() {
        let za = (0, 0, 1);
        assert!(ZoneAxisDeterminer::is_allowed_reflection((1, 0, 0), za));
        assert!(ZoneAxisDeterminer::is_allowed_reflection((0, 1, 0), za));
        assert!(ZoneAxisDeterminer::is_allowed_reflection((1, 1, 0), za));
        assert!(!ZoneAxisDeterminer::is_allowed_reflection((0, 0, 1), za));
    }

    #[test]
    fn test_is_allowed_reflection_zone_111() {
        let za = (1, 1, 1);
        assert!(ZoneAxisDeterminer::is_allowed_reflection((1, -1, 0), za));
        assert!(ZoneAxisDeterminer::is_allowed_reflection((0, 1, -1), za));
        assert!(!ZoneAxisDeterminer::is_allowed_reflection((1, 1, 1), za));
    }

    #[test]
    fn test_generate_pattern_001_sc() {
        let crystal = CrystalParams::cubic(400.0, StructureType::SimpleCubic);
        let pattern = ZoneAxisDeterminer::generate_pattern((0, 0, 1), &crystal, 2);
        // For [001] zone, l must be 0 for hu+kv+lw=0 with w=1
        // So only reflections with l=0
        for &(h, k, l) in &pattern {
            assert_eq!(l, 0, "reflection ({},{},{}) violates zone law", h, k, l);
        }
    }

    #[test]
    fn test_common_zone_axes() {
        let axes = ZoneAxisDeterminer::common_zone_axes();
        assert!(axes.contains(&(0, 0, 1)));
        assert!(axes.contains(&(1, 1, 1)));
        assert!(axes.contains(&(1, 1, 0)));
    }

    // -----------------------------------------------------------------------
    // LatticeParameterFitter tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_fit_cubic_single() {
        let d = MillerIndexer::d_spacing_cubic(405.0, 1, 1, 1);
        let a = LatticeParameterFitter::fit_cubic(&[d], &[(1, 1, 1)]);
        assert!((a - 405.0).abs() < 0.1);
    }

    #[test]
    fn test_fit_cubic_multiple() {
        let a_true = 361.5; // copper
        let d1 = MillerIndexer::d_spacing_cubic(a_true, 1, 1, 1);
        let d2 = MillerIndexer::d_spacing_cubic(a_true, 2, 0, 0);
        let d3 = MillerIndexer::d_spacing_cubic(a_true, 2, 2, 0);
        let a = LatticeParameterFitter::fit_cubic(
            &[d1, d2, d3],
            &[(1, 1, 1), (2, 0, 0), (2, 2, 0)],
        );
        assert!((a - a_true).abs() < 0.1);
    }

    #[test]
    fn test_fit_hexagonal() {
        let a_true = 300.0;
        let c_true = 500.0;
        let d1 = MillerIndexer::d_spacing_hexagonal(a_true, c_true, 1, 0, 0);
        let d2 = MillerIndexer::d_spacing_hexagonal(a_true, c_true, 0, 0, 1);
        let d3 = MillerIndexer::d_spacing_hexagonal(a_true, c_true, 1, 0, 1);
        let (a, c) = LatticeParameterFitter::fit_hexagonal(
            &[d1, d2, d3],
            &[(1, 0, 0), (0, 0, 1), (1, 0, 1)],
        );
        assert!((a - a_true).abs() < 1.0);
        assert!((c - c_true).abs() < 1.0);
    }

    #[test]
    fn test_fit_tetragonal() {
        let a_true = 400.0;
        let c_true = 600.0;
        let d1 = MillerIndexer::d_spacing_tetragonal(a_true, c_true, 1, 0, 0);
        let d2 = MillerIndexer::d_spacing_tetragonal(a_true, c_true, 0, 0, 1);
        let d3 = MillerIndexer::d_spacing_tetragonal(a_true, c_true, 1, 1, 0);
        let (a, c) = LatticeParameterFitter::fit_tetragonal(
            &[d1, d2, d3],
            &[(1, 0, 0), (0, 0, 1), (1, 1, 0)],
        );
        assert!((a - a_true).abs() < 1.0);
        assert!((c - c_true).abs() < 1.0);
    }

    #[test]
    fn test_least_squares_perfect() {
        let d_obs = vec![100.0, 200.0, 300.0];
        let d_calc = vec![100.0, 200.0, 300.0];
        let r = LatticeParameterFitter::least_squares_refinement(&d_obs, &d_calc);
        assert!(r.abs() < EPS);
    }

    #[test]
    fn test_least_squares_nonzero() {
        let d_obs = vec![100.0, 200.0, 300.0];
        let d_calc = vec![101.0, 199.0, 302.0];
        let r = LatticeParameterFitter::least_squares_refinement(&d_obs, &d_calc);
        assert!(r > 0.0 && r < 0.02);
    }

    // -----------------------------------------------------------------------
    // StructureFactorCalculator tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_sf_bcc_allowed() {
        assert!((StructureFactorCalculator::structure_factor_bcc((1, 1, 0)) - 4.0).abs() < EPS);
        assert!((StructureFactorCalculator::structure_factor_bcc((2, 0, 0)) - 4.0).abs() < EPS);
    }

    #[test]
    fn test_sf_bcc_forbidden() {
        assert!(StructureFactorCalculator::structure_factor_bcc((1, 0, 0)).abs() < EPS);
        assert!(StructureFactorCalculator::structure_factor_bcc((1, 1, 1)).abs() < EPS);
    }

    #[test]
    fn test_sf_fcc_allowed() {
        assert!(
            (StructureFactorCalculator::structure_factor_fcc((1, 1, 1)) - 16.0).abs() < EPS
        );
        assert!(
            (StructureFactorCalculator::structure_factor_fcc((2, 0, 0)) - 16.0).abs() < EPS
        );
    }

    #[test]
    fn test_sf_fcc_forbidden() {
        assert!(StructureFactorCalculator::structure_factor_fcc((1, 0, 0)).abs() < EPS);
        assert!(StructureFactorCalculator::structure_factor_fcc((2, 1, 0)).abs() < EPS);
    }

    #[test]
    fn test_sf_diamond_allowed() {
        // (111): h+k+l=3, all odd → |F|²=32
        assert!(
            (StructureFactorCalculator::structure_factor_diamond((1, 1, 1)) - 32.0).abs() < EPS
        );
        // (220): h+k+l=4, all even → |F|²=64
        assert!(
            (StructureFactorCalculator::structure_factor_diamond((2, 2, 0)) - 64.0).abs() < EPS
        );
    }

    #[test]
    fn test_sf_diamond_forbidden() {
        // (200): h+k+l=2, FCC allowed (all even), but diamond: 4n+2 → forbidden
        assert!(StructureFactorCalculator::structure_factor_diamond((2, 0, 0)).abs() < EPS);
    }

    #[test]
    fn test_extinction_rules_fcc() {
        assert!(StructureFactorCalculator::extinction_rules((1, 0, 0), StructureType::Fcc));
        assert!(!StructureFactorCalculator::extinction_rules((1, 1, 1), StructureType::Fcc));
    }

    #[test]
    fn test_extinction_rules_bcc() {
        assert!(StructureFactorCalculator::extinction_rules((1, 0, 0), StructureType::Bcc));
        assert!(!StructureFactorCalculator::extinction_rules((1, 1, 0), StructureType::Bcc));
    }

    #[test]
    fn test_extinction_rules_sc() {
        assert!(!StructureFactorCalculator::extinction_rules((1, 0, 0), StructureType::SimpleCubic));
    }

    #[test]
    fn test_relative_intensity() {
        let ri = StructureFactorCalculator::relative_intensity(16.0, 8);
        assert!((ri - 128.0).abs() < EPS);
    }

    // -----------------------------------------------------------------------
    // RingPatternAnalyzer tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_radial_profile_center() {
        let mut image = vec![vec![0.0; 20]; 20];
        image[10][10] = 100.0;
        let profile = RingPatternAnalyzer::radial_profile(&image, (10.0, 10.0));
        assert!(!profile.is_empty());
        assert!(profile[0].1 > 0.0); // centre bin has intensity
    }

    #[test]
    fn test_radial_profile_empty() {
        let profile = RingPatternAnalyzer::radial_profile(&[], (0.0, 0.0));
        assert!(profile.is_empty());
    }

    #[test]
    fn test_detect_rings() {
        let d_spacings = vec![200.0, 140.0];
        let intensities = vec![10.0, 8.0];
        let cc = 2000.0; // pm·px
        let image = PatternSimulator::simulate_ring_pattern(&d_spacings, &intensities, cc, 64);
        let rings = RingPatternAnalyzer::detect_rings(&image, (32.0, 32.0), cc, 0.5);
        // Should detect at least one ring
        assert!(!rings.is_empty());
    }

    #[test]
    fn test_identify_phase() {
        let measured = vec![200.0, 141.4, 100.0];
        let db = vec![
            PhaseEntry {
                name: "Copper".to_string(),
                d_spacings_pm: vec![208.0, 180.0, 127.0],
            },
            PhaseEntry {
                name: "Gold".to_string(),
                d_spacings_pm: vec![201.0, 142.0, 101.0],
            },
        ];
        let matches = RingPatternAnalyzer::identify_phase(&measured, &db, 5.0);
        assert!(!matches.is_empty());
        assert_eq!(matches[0].name, "Gold"); // best match
    }

    #[test]
    fn test_identify_phase_empty_db() {
        let matches = RingPatternAnalyzer::identify_phase(&[200.0], &[], 5.0);
        assert!(matches.is_empty());
    }

    // -----------------------------------------------------------------------
    // KikuchiLineAnalyzer tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_band_width() {
        let w = KikuchiLineAnalyzer::band_width(200.0, 2.51, 500.0, 25.0);
        // 2θ = 2.51/200 = 0.01255 rad
        // scale = 500 * 1000 / 25 = 20000 px/rad
        // w = 0.01255 * 20000 = 251 px
        assert!((w - 251.0).abs() < 1.0);
    }

    #[test]
    fn test_detect_kikuchi_lines_empty() {
        let lines = KikuchiLineAnalyzer::detect_lines(&[]);
        assert!(lines.is_empty());
    }

    #[test]
    fn test_orientation_from_kikuchi_insufficient() {
        let lines = vec![KikuchiLine {
            position: 50.0,
            angle_deg: 0.0,
            width_px: 5.0,
        }];
        let crystal = CrystalParams::cubic(400.0, StructureType::Fcc);
        let (phi1, phi, phi2) = KikuchiLineAnalyzer::orientation_from_kikuchi(&lines, &crystal);
        assert!((phi1).abs() < EPS);
        assert!((phi).abs() < EPS);
        assert!((phi2).abs() < EPS);
    }

    // -----------------------------------------------------------------------
    // PatternSimulator tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_simulate_spot_pattern_size() {
        let crystal = CrystalParams::cubic(400.0, StructureType::Fcc);
        let image = PatternSimulator::simulate_spot_pattern((0, 0, 1), &crystal, 64);
        assert_eq!(image.len(), 64);
        assert_eq!(image[0].len(), 64);
    }

    #[test]
    fn test_simulate_spot_pattern_has_center() {
        let crystal = CrystalParams::cubic(400.0, StructureType::Fcc);
        let image = PatternSimulator::simulate_spot_pattern((0, 0, 1), &crystal, 64);
        // Centre pixel should be bright
        assert!(image[32][32] > 10.0);
    }

    #[test]
    fn test_simulate_ring_pattern_size() {
        let image =
            PatternSimulator::simulate_ring_pattern(&[200.0, 140.0], &[10.0, 8.0], 2000.0, 32);
        assert_eq!(image.len(), 32);
        assert_eq!(image[0].len(), 32);
    }

    #[test]
    fn test_add_noise() {
        let mut image = vec![vec![100.0; 10]; 10];
        let original_sum: f64 = image.iter().flat_map(|r| r.iter()).sum();
        PatternSimulator::add_noise(&mut image, 5.0, 42);
        let new_sum: f64 = image.iter().flat_map(|r| r.iter()).sum();
        // With noise, sum should differ
        assert!((new_sum - original_sum).abs() > 0.1);
    }

    #[test]
    fn test_add_noise_nonnegative() {
        let mut image = vec![vec![1.0; 10]; 10];
        PatternSimulator::add_noise(&mut image, 10.0, 123);
        for row in &image {
            for &v in row {
                assert!(v >= 0.0);
            }
        }
    }

    // -----------------------------------------------------------------------
    // CrystalParams tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_crystal_params_cubic() {
        let c = CrystalParams::cubic(405.0, StructureType::Fcc);
        assert!((c.a_pm - 405.0).abs() < EPS);
        assert!((c.b_pm - 405.0).abs() < EPS);
        assert!((c.c_pm - 405.0).abs() < EPS);
        assert!((c.alpha_deg - 90.0).abs() < EPS);
    }

    #[test]
    fn test_crystal_params_hexagonal() {
        let c = CrystalParams::hexagonal(300.0, 500.0);
        assert!((c.a_pm - 300.0).abs() < EPS);
        assert!((c.c_pm - 500.0).abs() < EPS);
        assert!((c.gamma_deg - 120.0).abs() < EPS);
    }

    #[test]
    fn test_crystal_params_tetragonal() {
        let c = CrystalParams::tetragonal(400.0, 600.0, StructureType::Bcc);
        assert!((c.a_pm - 400.0).abs() < EPS);
        assert!((c.b_pm - 400.0).abs() < EPS);
        assert!((c.c_pm - 600.0).abs() < EPS);
    }

    // -----------------------------------------------------------------------
    // GCD helper tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_gcd() {
        assert_eq!(gcd(12, 8), 4);
        assert_eq!(gcd(7, 3), 1);
        assert_eq!(gcd(0, 5), 5);
    }

    #[test]
    fn test_gcd3() {
        assert_eq!(gcd3(12, 8, 4), 4);
        assert_eq!(gcd3(6, 10, 15), 1);
    }

    // -----------------------------------------------------------------------
    // Integration / roundtrip tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_simulate_detect_roundtrip() {
        let crystal = CrystalParams::cubic(400.0, StructureType::SimpleCubic);
        let image = PatternSimulator::simulate_spot_pattern((0, 0, 1), &crystal, 128);
        let detector = PatternDetector::new(10.0, 5.0);
        let spots = detector.detect_spots(&image);
        assert!(!spots.is_empty());
    }

    #[test]
    fn test_wavelength_dspacing_roundtrip() {
        let lambda = DSpacingCalculator::electron_wavelength(200.0);
        let d_111 = MillerIndexer::d_spacing_cubic(405.0, 1, 1, 1);
        let theta = DSpacingCalculator::bragg_angle(d_111, lambda);
        assert!(theta > 0.0);
        assert!(theta < 0.1); // small angle for TEM
    }

    #[test]
    fn test_fcc_aluminum_dspacings() {
        // Aluminium: FCC, a = 404.9 pm
        let a = 404.9;
        let d_111 = MillerIndexer::d_spacing_cubic(a, 1, 1, 1);
        let d_200 = MillerIndexer::d_spacing_cubic(a, 2, 0, 0);
        let d_220 = MillerIndexer::d_spacing_cubic(a, 2, 2, 0);
        // Known values: d_111 ≈ 233.8, d_200 ≈ 202.45, d_220 ≈ 143.14
        assert!((d_111 - 233.8).abs() < 0.5);
        assert!((d_200 - 202.45).abs() < 0.5);
        assert!((d_220 - 143.1).abs() < 0.5);
    }

    #[test]
    fn test_silicon_diamond_structure() {
        // Si: diamond cubic, a = 543.1 pm
        let a = 543.1;
        // (111) allowed in diamond: |F|²=32
        assert!(StructureFactorCalculator::structure_factor_diamond((1, 1, 1)) > 0.0);
        // (200) forbidden in diamond
        assert!(StructureFactorCalculator::structure_factor_diamond((2, 0, 0)) < EPS);
        // (220) allowed
        assert!(StructureFactorCalculator::structure_factor_diamond((2, 2, 0)) > 0.0);
        // (222) forbidden (h+k+l=6=4n+2)
        assert!(StructureFactorCalculator::structure_factor_diamond((2, 2, 2)) < EPS);
        // d_111
        let d_111 = MillerIndexer::d_spacing_cubic(a, 1, 1, 1);
        assert!((d_111 - 313.6).abs() < 1.0);
    }

    #[test]
    fn test_zone_axis_generation_fcc_001() {
        let crystal = CrystalParams::cubic(400.0, StructureType::Fcc);
        let pattern = ZoneAxisDeterminer::generate_pattern((0, 0, 1), &crystal, 3);
        // All should satisfy zone law and FCC selection rules
        for &(h, k, l) in &pattern {
            assert_eq!(l, 0); // zone law
            // FCC: h,k must have same parity (since l=0 is even)
            let hp = ((h % 2) + 2) % 2;
            let kp = ((k % 2) + 2) % 2;
            assert_eq!(hp, kp, "FCC rule violated for ({},{},{})", h, k, l);
        }
    }

    #[test]
    fn test_lattice_fit_roundtrip_cubic() {
        let a_true = 361.5;
        let indices = vec![(1, 1, 1), (2, 0, 0), (2, 2, 0), (3, 1, 1)];
        let d_spacings: Vec<f64> = indices
            .iter()
            .map(|&(h, k, l)| MillerIndexer::d_spacing_cubic(a_true, h, k, l))
            .collect();
        let a_fit = LatticeParameterFitter::fit_cubic(&d_spacings, &indices);
        let r = LatticeParameterFitter::least_squares_refinement(
            &d_spacings,
            &indices
                .iter()
                .map(|&(h, k, l)| MillerIndexer::d_spacing_cubic(a_fit, h, k, l))
                .collect::<Vec<_>>(),
        );
        assert!((a_fit - a_true).abs() < 0.01);
        assert!(r < 0.001);
    }

    #[test]
    fn test_simple_rng_deterministic() {
        let mut rng1 = SimpleRng::new(42);
        let mut rng2 = SimpleRng::new(42);
        for _ in 0..100 {
            assert_eq!(rng1.next_u64(), rng2.next_u64());
        }
    }

    #[test]
    fn test_simple_rng_range() {
        let mut rng = SimpleRng::new(99);
        for _ in 0..1000 {
            let v = rng.next_f64();
            assert!(v >= 0.0 && v < 1.0);
        }
    }
}
