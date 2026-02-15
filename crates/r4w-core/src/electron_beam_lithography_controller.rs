//! # Electron Beam Lithography (EBL) Controller
//!
//! This module implements electron beam lithography pattern generation and
//! proximity effect correction for nanofabrication. EBL uses a focused beam
//! of electrons to write custom patterns on a substrate coated with an
//! electron-sensitive resist (e.g., PMMA, ZEP-520A, HSQ).
//!
//! ## Principle of Operation
//!
//! A finely focused electron beam is scanned across the resist surface,
//! breaking (positive-tone) or cross-linking (negative-tone) polymer chains.
//! After development, the exposed (or unexposed) regions are removed, creating
//! nanoscale patterns for device fabrication.
//!
//! ## Key Concepts
//!
//! - **Proximity Effect**: Backscattered electrons expose resist far from the
//!   primary beam, causing pattern distortion. Corrected via dose modulation.
//! - **Point Spread Function (PSF)**: Double-Gaussian model capturing forward
//!   scattering (alpha) and backscattering (beta) ranges.
//! - **Dose**: Charge deposited per unit area, typically 100-500 uC/cm² for PMMA.
//! - **Write Field**: Maximum beam deflection range; patterns larger than one
//!   field require field stitching.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::electron_beam_lithography_controller::*;
//!
//! let config = BeamConfig {
//!     acceleration_voltage_kv: 100.0,
//!     beam_current_pa: 100.0,
//!     spot_size_nm: 2.0,
//!     step_size_nm: 5.0,
//! };
//!
//! let dose = DoseCalculation::base_dose_uc_cm2(config.beam_current_pa, 1e-6, config.step_size_nm);
//! assert!(dose > 0.0);
//!
//! let psf = ProximityParams::from_voltage(config.acceleration_voltage_kv);
//! assert!(psf.alpha_nm > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Beam Configuration
// ---------------------------------------------------------------------------

/// Electron beam configuration parameters.
#[derive(Debug, Clone, Copy)]
pub struct BeamConfig {
    /// Acceleration voltage in kilovolts (typical: 10-100 kV).
    pub acceleration_voltage_kv: f64,
    /// Beam current in picoamperes (typical: 10-1000 pA).
    pub beam_current_pa: f64,
    /// Spot size (1/e beam diameter) in nanometers.
    pub spot_size_nm: f64,
    /// Step size (beam address grid) in nanometers.
    pub step_size_nm: f64,
}

impl BeamConfig {
    /// Create a new beam configuration.
    pub fn new(voltage_kv: f64, current_pa: f64, spot_nm: f64, step_nm: f64) -> Self {
        Self {
            acceleration_voltage_kv: voltage_kv,
            beam_current_pa: current_pa,
            spot_size_nm: spot_nm,
            step_size_nm: step_nm,
        }
    }

    /// Typical 100 kV high-resolution configuration.
    pub fn high_resolution() -> Self {
        Self::new(100.0, 50.0, 2.0, 1.0)
    }

    /// Typical 30 kV general purpose configuration.
    pub fn general_purpose() -> Self {
        Self::new(30.0, 200.0, 5.0, 5.0)
    }

    /// Typical 10 kV low-voltage configuration (less backscattering).
    pub fn low_voltage() -> Self {
        Self::new(10.0, 500.0, 10.0, 10.0)
    }
}

// ---------------------------------------------------------------------------
// Pattern Primitives
// ---------------------------------------------------------------------------

/// A 2D point in nanometers (pattern coordinate space).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Point {
    pub x_nm: f64,
    pub y_nm: f64,
}

impl Point {
    pub fn new(x_nm: f64, y_nm: f64) -> Self {
        Self { x_nm, y_nm }
    }

    /// Euclidean distance to another point.
    pub fn distance_to(&self, other: &Point) -> f64 {
        let dx = self.x_nm - other.x_nm;
        let dy = self.y_nm - other.y_nm;
        (dx * dx + dy * dy).sqrt()
    }
}

/// Pattern primitive types for EBL.
#[derive(Debug, Clone)]
pub enum Pattern {
    /// Single pixel exposure at a point.
    PointExposure {
        position: Point,
        dose_uc_cm2: f64,
    },
    /// Line from start to end with given dose.
    Line {
        start: Point,
        end: Point,
        dose_uc_cm2: f64,
    },
    /// Axis-aligned rectangle.
    Rectangle {
        origin: Point,
        width_nm: f64,
        height_nm: f64,
        dose_uc_cm2: f64,
    },
    /// Circle or arc defined by center, radius, and angular range.
    CircleArc {
        center: Point,
        radius_nm: f64,
        start_angle_rad: f64,
        end_angle_rad: f64,
        dose_uc_cm2: f64,
    },
    /// Polygon with arbitrary vertices (filled via raster scan).
    Polygon {
        vertices: Vec<Point>,
        dose_uc_cm2: f64,
    },
}

/// An exposure pixel on the beam address grid.
#[derive(Debug, Clone, Copy)]
pub struct ExposurePixel {
    /// Grid x index.
    pub ix: usize,
    /// Grid y index.
    pub iy: usize,
    /// Center x position in nm.
    pub x_nm: f64,
    /// Center y position in nm.
    pub y_nm: f64,
    /// Assigned dose in uC/cm².
    pub dose_uc_cm2: f64,
    /// Computed dwell time in seconds.
    pub dwell_time_s: f64,
}

// ---------------------------------------------------------------------------
// Dose Calculation
// ---------------------------------------------------------------------------

/// Dose calculation utilities for electron beam lithography.
pub struct DoseCalculation;

impl DoseCalculation {
    /// Compute base dose in uC/cm² given beam current, dwell time, and pixel area.
    ///
    /// D = I * t / A
    /// where I = current in pA, t = dwell time in seconds, A = pixel area in cm².
    pub fn base_dose_uc_cm2(current_pa: f64, dwell_time_s: f64, step_size_nm: f64) -> f64 {
        let current_a = current_pa * 1e-12; // pA to A
        let charge_c = current_a * dwell_time_s; // Coulombs
        let charge_uc = charge_c * 1e6; // microCoulombs
        let step_cm = step_size_nm * 1e-7; // nm to cm
        let area_cm2 = step_cm * step_cm;
        if area_cm2 <= 0.0 {
            return 0.0;
        }
        charge_uc / area_cm2
    }

    /// Compute dwell time in seconds for a desired dose.
    ///
    /// t = D * A / I
    pub fn dwell_time_s(dose_uc_cm2: f64, step_size_nm: f64, current_pa: f64) -> f64 {
        let step_cm = step_size_nm * 1e-7;
        let area_cm2 = step_cm * step_cm;
        let dose_c_cm2 = dose_uc_cm2 * 1e-6; // uC to C
        let current_a = current_pa * 1e-12;
        if current_a <= 0.0 {
            return 0.0;
        }
        dose_c_cm2 * area_cm2 / current_a
    }

    /// Total charge deposited for a single pixel in femtoCoulombs.
    pub fn pixel_charge_fc(dose_uc_cm2: f64, step_size_nm: f64) -> f64 {
        let step_cm = step_size_nm * 1e-7;
        let area_cm2 = step_cm * step_cm;
        let charge_uc = dose_uc_cm2 * area_cm2;
        charge_uc * 1e9 // uC to fC
    }

    /// Number of electrons per pixel.
    pub fn electrons_per_pixel(dose_uc_cm2: f64, step_size_nm: f64) -> f64 {
        let charge_fc = Self::pixel_charge_fc(dose_uc_cm2, step_size_nm);
        let charge_c = charge_fc * 1e-15;
        let e_charge = 1.602176634e-19;
        charge_c / e_charge
    }
}

// ---------------------------------------------------------------------------
// Proximity Effect Correction (PEC)
// ---------------------------------------------------------------------------

/// Parameters for the double-Gaussian point spread function model.
#[derive(Debug, Clone, Copy)]
pub struct ProximityParams {
    /// Forward scattering range in nm (alpha).
    pub alpha_nm: f64,
    /// Backscattering range in nm (beta).
    pub beta_nm: f64,
    /// Backscattering coefficient (eta).
    pub eta: f64,
}

impl ProximityParams {
    /// Create proximity parameters from explicit values.
    pub fn new(alpha_nm: f64, beta_nm: f64, eta: f64) -> Self {
        Self { alpha_nm, beta_nm, eta }
    }

    /// Approximate proximity parameters from acceleration voltage.
    ///
    /// Uses empirical fits for silicon substrate with PMMA resist:
    /// - alpha ~ 10 nm at 100 kV, scales roughly as V^(-0.6)
    /// - beta ~ 30 um at 100 kV, scales roughly as V^1.75 / 100^1.75 * 30000
    /// - eta ~ 0.7 for Si (relatively constant, slight V dependence)
    pub fn from_voltage(voltage_kv: f64) -> Self {
        // Forward scattering range: roughly proportional to resist thickness / voltage
        // Approximate: alpha = 5 + 500/V (nm)
        let alpha = 5.0 + 500.0 / voltage_kv;

        // Backscatter range: proportional to V^1.75
        // Normalized to ~30 um at 100 kV
        let beta = 30000.0 * (voltage_kv / 100.0).powf(1.75);

        // Backscatter coefficient for silicon: ~0.7, slight voltage dependence
        let eta = 0.5 + 0.2 * (voltage_kv / 100.0).min(1.5);

        Self { alpha_nm: alpha, beta_nm: beta, eta }
    }

    /// Evaluate the double-Gaussian PSF at distance r (nm).
    ///
    /// f(r) = 1/(pi*(1+eta)) * [1/alpha² * exp(-r²/alpha²) + eta/beta² * exp(-r²/beta²)]
    pub fn psf(&self, r_nm: f64) -> f64 {
        let norm = 1.0 / (PI * (1.0 + self.eta));
        let forward = (1.0 / (self.alpha_nm * self.alpha_nm))
            * (-r_nm * r_nm / (self.alpha_nm * self.alpha_nm)).exp();
        let back = (self.eta / (self.beta_nm * self.beta_nm))
            * (-r_nm * r_nm / (self.beta_nm * self.beta_nm)).exp();
        norm * (forward + back)
    }

    /// Evaluate only the backscatter kernel at distance r (nm).
    /// Used for proximity effect correction dose modulation.
    pub fn backscatter_kernel(&self, r_nm: f64) -> f64 {
        (-r_nm * r_nm / (self.beta_nm * self.beta_nm)).exp()
    }
}

/// Proximity effect correction engine.
pub struct ProximityCorrector {
    params: ProximityParams,
    max_iterations: usize,
    convergence_threshold: f64,
}

impl ProximityCorrector {
    /// Create a new proximity corrector.
    pub fn new(params: ProximityParams, max_iterations: usize, convergence_threshold: f64) -> Self {
        Self {
            params,
            max_iterations,
            convergence_threshold,
        }
    }

    /// Self-consistent iterative proximity effect correction.
    ///
    /// Given pixel positions and a target dose, modulate doses so that:
    /// D_i * (1 + eta * sum_j!=i(D_j/D_target * kernel(r_ij))) = D_target
    ///
    /// Returns corrected dose for each pixel.
    pub fn correct_doses(
        &self,
        positions: &[Point],
        target_dose: f64,
    ) -> Vec<f64> {
        let n = positions.len();
        if n == 0 {
            return vec![];
        }

        let eta = self.params.eta;
        let mut doses: Vec<f64> = vec![target_dose; n];

        for _iteration in 0..self.max_iterations {
            let mut max_change = 0.0_f64;

            for i in 0..n {
                // Compute backscatter contribution from all other pixels
                let mut backscatter_sum = 0.0;
                for j in 0..n {
                    if i == j {
                        continue;
                    }
                    let r = positions[i].distance_to(&positions[j]);
                    let kernel = self.params.backscatter_kernel(r);
                    backscatter_sum += (doses[j] / target_dose) * kernel;
                }

                // Solve: D_i * (1 + eta * backscatter_sum) = D_target
                let new_dose = target_dose / (1.0 + eta * backscatter_sum);
                let change = (new_dose - doses[i]).abs() / target_dose;
                max_change = max_change.max(change);
                doses[i] = new_dose;
            }

            if max_change < self.convergence_threshold {
                break;
            }
        }

        doses
    }
}

// ---------------------------------------------------------------------------
// Beam Deflection and Scan Patterns
// ---------------------------------------------------------------------------

/// Scan pattern mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ScanMode {
    /// Raster scan (serpentine / boustrophedon).
    Raster,
    /// Vector scan: jump between features.
    Vector,
}

/// A write field definition (maximum beam deflection range).
#[derive(Debug, Clone, Copy)]
pub struct WriteField {
    /// Write field width in micrometers.
    pub width_um: f64,
    /// Write field height in micrometers.
    pub height_um: f64,
    /// Center x position in um (stage coordinates).
    pub center_x_um: f64,
    /// Center y position in um (stage coordinates).
    pub center_y_um: f64,
}

impl WriteField {
    pub fn new(width_um: f64, height_um: f64, center_x_um: f64, center_y_um: f64) -> Self {
        Self { width_um, height_um, center_x_um, center_y_um }
    }

    /// Number of pixels along x for given step size.
    pub fn pixels_x(&self, step_size_nm: f64) -> usize {
        let width_nm = self.width_um * 1000.0;
        (width_nm / step_size_nm).ceil() as usize
    }

    /// Number of pixels along y for given step size.
    pub fn pixels_y(&self, step_size_nm: f64) -> usize {
        let height_nm = self.height_um * 1000.0;
        (height_nm / step_size_nm).ceil() as usize
    }

    /// Total pixel count in this write field.
    pub fn total_pixels(&self, step_size_nm: f64) -> usize {
        self.pixels_x(step_size_nm) * self.pixels_y(step_size_nm)
    }
}

/// Generate a serpentine raster scan order for a grid.
///
/// Returns (ix, iy) pairs in scan order. Even rows go left-to-right,
/// odd rows go right-to-left.
pub fn raster_scan_order(nx: usize, ny: usize) -> Vec<(usize, usize)> {
    let mut order = Vec::with_capacity(nx * ny);
    for iy in 0..ny {
        if iy % 2 == 0 {
            // Left to right
            for ix in 0..nx {
                order.push((ix, iy));
            }
        } else {
            // Right to left (serpentine)
            for ix in (0..nx).rev() {
                order.push((ix, iy));
            }
        }
    }
    order
}

/// Generate vector scan order: visit only exposed pixels, sorted by proximity.
///
/// Uses a greedy nearest-neighbor traversal starting from (0,0).
pub fn vector_scan_order(exposed_pixels: &[(usize, usize)]) -> Vec<(usize, usize)> {
    if exposed_pixels.is_empty() {
        return vec![];
    }

    let n = exposed_pixels.len();
    let mut visited = vec![false; n];
    let mut order = Vec::with_capacity(n);

    // Start from the pixel nearest to origin
    let mut current_idx = 0;
    let mut min_dist = f64::MAX;
    for (i, &(ix, iy)) in exposed_pixels.iter().enumerate() {
        let d = ((ix * ix + iy * iy) as f64).sqrt();
        if d < min_dist {
            min_dist = d;
            current_idx = i;
        }
    }

    visited[current_idx] = 0 == 1; // false initially, we mark below
    // Actually start greedy
    visited[current_idx] = true;
    order.push(exposed_pixels[current_idx]);

    for _ in 1..n {
        let (cx, cy) = exposed_pixels[current_idx];
        let mut best_idx = 0;
        let mut best_dist = f64::MAX;
        for (j, &(jx, jy)) in exposed_pixels.iter().enumerate() {
            if visited[j] {
                continue;
            }
            let dx = cx as f64 - jx as f64;
            let dy = cy as f64 - jy as f64;
            let d = dx * dx + dy * dy;
            if d < best_dist {
                best_dist = d;
                best_idx = j;
            }
        }
        visited[best_idx] = true;
        order.push(exposed_pixels[best_idx]);
        current_idx = best_idx;
    }

    order
}

/// Field stitching: split a design into write fields with overlap.
pub fn field_stitch(
    design_width_um: f64,
    design_height_um: f64,
    field_width_um: f64,
    field_height_um: f64,
    overlap_um: f64,
) -> Vec<WriteField> {
    let mut fields = Vec::new();
    let step_x = field_width_um - overlap_um;
    let step_y = field_height_um - overlap_um;

    if step_x <= 0.0 || step_y <= 0.0 {
        return fields;
    }

    let nx = ((design_width_um / step_x).ceil() as usize).max(1);
    let ny = ((design_height_um / step_y).ceil() as usize).max(1);

    for iy in 0..ny {
        for ix in 0..nx {
            let cx = ix as f64 * step_x + field_width_um / 2.0;
            let cy = iy as f64 * step_y + field_height_um / 2.0;
            fields.push(WriteField::new(field_width_um, field_height_um, cx, cy));
        }
    }

    fields
}

// ---------------------------------------------------------------------------
// Resolution Limits
// ---------------------------------------------------------------------------

/// Resolution limit estimations.
pub struct ResolutionLimits;

impl ResolutionLimits {
    /// Forward scattering broadening in resist (approximate).
    ///
    /// delta_r ~ 0.9 * (R_t / V_b)^1.5  (nm)
    /// where R_t = resist thickness in nm, V_b = beam voltage in kV.
    pub fn forward_scatter_broadening_nm(resist_thickness_nm: f64, voltage_kv: f64) -> f64 {
        if voltage_kv <= 0.0 {
            return 0.0;
        }
        0.9 * (resist_thickness_nm / voltage_kv).powf(1.5)
    }

    /// Effective beam diameter considering spot size and forward scattering.
    pub fn effective_beam_diameter_nm(
        spot_size_nm: f64,
        resist_thickness_nm: f64,
        voltage_kv: f64,
    ) -> f64 {
        let scatter = Self::forward_scatter_broadening_nm(resist_thickness_nm, voltage_kv);
        (spot_size_nm * spot_size_nm + scatter * scatter).sqrt()
    }

    /// Minimum feature size estimate (beam + scatter + development).
    /// Development broadening is approximated as ~5 nm for typical conditions.
    pub fn min_feature_size_nm(
        spot_size_nm: f64,
        resist_thickness_nm: f64,
        voltage_kv: f64,
        development_broadening_nm: f64,
    ) -> f64 {
        let eff_beam = Self::effective_beam_diameter_nm(spot_size_nm, resist_thickness_nm, voltage_kv);
        eff_beam + development_broadening_nm
    }

    /// Electron range in resist (Kanaya-Okayama, approximate for PMMA).
    ///
    /// R_e ~ 0.0276 * A / (Z^0.89 * rho) * E^1.67  (um)
    /// For PMMA: A~6, Z~3.6, rho~1.19 g/cm³
    pub fn electron_range_um(voltage_kv: f64) -> f64 {
        let a = 6.0; // average atomic mass for PMMA
        let z: f64 = 3.6; // average atomic number
        let rho: f64 = 1.19; // density g/cm³
        let e_kev: f64 = voltage_kv; // already in keV
        0.0276_f64 * a / (z.powf(0.89) * rho) * e_kev.powf(1.67)
    }
}

// ---------------------------------------------------------------------------
// Exposure Grid
// ---------------------------------------------------------------------------

/// Exposure grid: pixelates patterns onto the beam address grid.
pub struct ExposureGrid {
    /// Step size in nm.
    pub step_size_nm: f64,
    /// Grid width in pixels.
    pub nx: usize,
    /// Grid height in pixels.
    pub ny: usize,
    /// Dose per pixel in uC/cm² (0 = unexposed).
    pub doses: Vec<f64>,
}

impl ExposureGrid {
    /// Create an empty exposure grid.
    pub fn new(width_nm: f64, height_nm: f64, step_size_nm: f64) -> Self {
        let nx = (width_nm / step_size_nm).ceil() as usize;
        let ny = (height_nm / step_size_nm).ceil() as usize;
        Self {
            step_size_nm,
            nx,
            ny,
            doses: vec![0.0; nx * ny],
        }
    }

    /// Get dose at grid position.
    pub fn get_dose(&self, ix: usize, iy: usize) -> f64 {
        if ix < self.nx && iy < self.ny {
            self.doses[iy * self.nx + ix]
        } else {
            0.0
        }
    }

    /// Set dose at grid position.
    pub fn set_dose(&mut self, ix: usize, iy: usize, dose: f64) {
        if ix < self.nx && iy < self.ny {
            self.doses[iy * self.nx + ix] = dose;
        }
    }

    /// Center position of pixel (ix, iy) in nm.
    pub fn pixel_center(&self, ix: usize, iy: usize) -> Point {
        Point::new(
            (ix as f64 + 0.5) * self.step_size_nm,
            (iy as f64 + 0.5) * self.step_size_nm,
        )
    }

    /// Rasterize a point exposure onto the grid.
    pub fn rasterize_point(&mut self, pos: &Point, dose: f64) {
        let ix = (pos.x_nm / self.step_size_nm) as usize;
        let iy = (pos.y_nm / self.step_size_nm) as usize;
        self.set_dose(ix, iy, dose);
    }

    /// Rasterize a line using Bresenham-like stepping.
    pub fn rasterize_line(&mut self, start: &Point, end: &Point, dose: f64) {
        let ix0 = (start.x_nm / self.step_size_nm) as isize;
        let iy0 = (start.y_nm / self.step_size_nm) as isize;
        let ix1 = (end.x_nm / self.step_size_nm) as isize;
        let iy1 = (end.y_nm / self.step_size_nm) as isize;

        let dx = (ix1 - ix0).abs();
        let dy = -(iy1 - iy0).abs();
        let sx: isize = if ix0 < ix1 { 1 } else { -1 };
        let sy: isize = if iy0 < iy1 { 1 } else { -1 };
        let mut err = dx + dy;
        let mut x = ix0;
        let mut y = iy0;

        loop {
            if x >= 0 && y >= 0 {
                self.set_dose(x as usize, y as usize, dose);
            }
            if x == ix1 && y == iy1 {
                break;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += sx;
            }
            if e2 <= dx {
                err += dx;
                y += sy;
            }
        }
    }

    /// Rasterize an axis-aligned rectangle.
    pub fn rasterize_rectangle(&mut self, origin: &Point, width_nm: f64, height_nm: f64, dose: f64) {
        let ix_start = (origin.x_nm / self.step_size_nm) as usize;
        let iy_start = (origin.y_nm / self.step_size_nm) as usize;
        let ix_end = ((origin.x_nm + width_nm) / self.step_size_nm).ceil() as usize;
        let iy_end = ((origin.y_nm + height_nm) / self.step_size_nm).ceil() as usize;

        for iy in iy_start..iy_end.min(self.ny) {
            for ix in ix_start..ix_end.min(self.nx) {
                self.set_dose(ix, iy, dose);
            }
        }
    }

    /// Rasterize a circle or arc.
    pub fn rasterize_circle_arc(
        &mut self,
        center: &Point,
        radius_nm: f64,
        start_angle: f64,
        end_angle: f64,
        dose: f64,
    ) {
        // Fill approach: scan bounding box, check if pixel is inside arc sector
        let r_pixels = (radius_nm / self.step_size_nm).ceil() as isize;
        let cx = (center.x_nm / self.step_size_nm) as isize;
        let cy = (center.y_nm / self.step_size_nm) as isize;

        for dy in -r_pixels..=r_pixels {
            for dx in -r_pixels..=r_pixels {
                let px = cx + dx;
                let py = cy + dy;
                if px < 0 || py < 0 || px >= self.nx as isize || py >= self.ny as isize {
                    continue;
                }
                let fx = dx as f64 * self.step_size_nm;
                let fy = dy as f64 * self.step_size_nm;
                let r = (fx * fx + fy * fy).sqrt();
                if r > radius_nm {
                    continue;
                }

                // Check angle for arc
                let angle = fy.atan2(fx);
                // Normalize angle to [0, 2*PI)
                let angle_norm = if angle < 0.0 { angle + 2.0 * PI } else { angle };
                let start_norm = if start_angle < 0.0 { start_angle + 2.0 * PI } else { start_angle };
                let end_norm = if end_angle < 0.0 { end_angle + 2.0 * PI } else { end_angle };

                let in_arc = if start_norm <= end_norm {
                    angle_norm >= start_norm && angle_norm <= end_norm
                } else {
                    // Arc wraps around 0
                    angle_norm >= start_norm || angle_norm <= end_norm
                };

                if in_arc {
                    self.set_dose(px as usize, py as usize, dose);
                }
            }
        }
    }

    /// Rasterize a polygon using scanline fill.
    pub fn rasterize_polygon(&mut self, vertices: &[Point], dose: f64) {
        if vertices.len() < 3 {
            return;
        }

        // Find bounding box
        let min_y = vertices.iter().map(|v| v.y_nm).fold(f64::MAX, f64::min);
        let max_y = vertices.iter().map(|v| v.y_nm).fold(f64::MIN, f64::max);

        let iy_start = (min_y / self.step_size_nm) as usize;
        let iy_end = ((max_y / self.step_size_nm).ceil() as usize).min(self.ny);

        for iy in iy_start..iy_end {
            let scan_y = (iy as f64 + 0.5) * self.step_size_nm;

            // Find intersection x-coordinates with polygon edges
            let mut intersections = Vec::new();
            let n = vertices.len();
            for i in 0..n {
                let j = (i + 1) % n;
                let y0 = vertices[i].y_nm;
                let y1 = vertices[j].y_nm;
                if (y0 <= scan_y && y1 > scan_y) || (y1 <= scan_y && y0 > scan_y) {
                    let t = (scan_y - y0) / (y1 - y0);
                    let x = vertices[i].x_nm + t * (vertices[j].x_nm - vertices[i].x_nm);
                    intersections.push(x);
                }
            }

            intersections.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

            // Fill between pairs of intersections
            let mut k = 0;
            while k + 1 < intersections.len() {
                let ix_start = (intersections[k] / self.step_size_nm) as usize;
                let ix_end = ((intersections[k + 1] / self.step_size_nm).ceil() as usize).min(self.nx);
                for ix in ix_start..ix_end {
                    self.set_dose(ix, iy, dose);
                }
                k += 2;
            }
        }
    }

    /// Rasterize any pattern primitive onto this grid.
    pub fn rasterize_pattern(&mut self, pattern: &Pattern) {
        match pattern {
            Pattern::PointExposure { position, dose_uc_cm2 } => {
                self.rasterize_point(position, *dose_uc_cm2);
            }
            Pattern::Line { start, end, dose_uc_cm2 } => {
                self.rasterize_line(start, end, *dose_uc_cm2);
            }
            Pattern::Rectangle { origin, width_nm, height_nm, dose_uc_cm2 } => {
                self.rasterize_rectangle(origin, *width_nm, *height_nm, *dose_uc_cm2);
            }
            Pattern::CircleArc { center, radius_nm, start_angle_rad, end_angle_rad, dose_uc_cm2 } => {
                self.rasterize_circle_arc(center, *radius_nm, *start_angle_rad, *end_angle_rad, *dose_uc_cm2);
            }
            Pattern::Polygon { vertices, dose_uc_cm2 } => {
                self.rasterize_polygon(vertices, *dose_uc_cm2);
            }
        }
    }

    /// Count total exposed pixels (dose > 0).
    pub fn exposed_pixel_count(&self) -> usize {
        self.doses.iter().filter(|&&d| d > 0.0).count()
    }

    /// Collect all exposed pixel positions.
    pub fn exposed_positions(&self) -> Vec<Point> {
        let mut positions = Vec::new();
        for iy in 0..self.ny {
            for ix in 0..self.nx {
                if self.get_dose(ix, iy) > 0.0 {
                    positions.push(self.pixel_center(ix, iy));
                }
            }
        }
        positions
    }

    /// Total write time estimation in seconds.
    ///
    /// Sum of dwell times + settling times between pixels.
    pub fn total_write_time_s(
        &self,
        beam_config: &BeamConfig,
        settling_time_s: f64,
    ) -> f64 {
        let mut total = 0.0;
        let mut exposed_count = 0;
        for &dose in &self.doses {
            if dose > 0.0 {
                let dwell = DoseCalculation::dwell_time_s(
                    dose,
                    self.step_size_nm,
                    beam_config.beam_current_pa,
                );
                total += dwell;
                exposed_count += 1;
            }
        }
        // Add settling time between pixels
        if exposed_count > 0 {
            total += (exposed_count - 1) as f64 * settling_time_s;
        }
        total
    }
}

// ---------------------------------------------------------------------------
// Pattern Fracturing
// ---------------------------------------------------------------------------

/// A trapezoid primitive for hardware pattern generation.
#[derive(Debug, Clone)]
pub struct Trapezoid {
    /// Lower-left x in nm.
    pub x_nm: f64,
    /// Lower-left y in nm.
    pub y_nm: f64,
    /// Width at bottom in nm.
    pub width_bottom_nm: f64,
    /// Width at top in nm.
    pub width_top_nm: f64,
    /// Height in nm.
    pub height_nm: f64,
    /// Dose in uC/cm².
    pub dose_uc_cm2: f64,
}

/// Fracture a rectangle into hardware-compatible trapezoids.
///
/// If the rectangle exceeds max_size, split it into smaller pieces.
pub fn fracture_rectangle(
    origin: &Point,
    width_nm: f64,
    height_nm: f64,
    dose: f64,
    max_size_nm: f64,
) -> Vec<Trapezoid> {
    let mut traps = Vec::new();
    let nx = (width_nm / max_size_nm).ceil() as usize;
    let ny = (height_nm / max_size_nm).ceil() as usize;

    for iy in 0..ny {
        for ix in 0..nx {
            let x = origin.x_nm + ix as f64 * max_size_nm;
            let y = origin.y_nm + iy as f64 * max_size_nm;
            let w = (width_nm - ix as f64 * max_size_nm).min(max_size_nm);
            let h = (height_nm - iy as f64 * max_size_nm).min(max_size_nm);
            traps.push(Trapezoid {
                x_nm: x,
                y_nm: y,
                width_bottom_nm: w,
                width_top_nm: w,
                height_nm: h,
                dose_uc_cm2: dose,
            });
        }
    }

    traps
}

/// Check if a feature meets minimum size requirements.
pub fn check_minimum_feature(width_nm: f64, height_nm: f64, min_feature_nm: f64) -> bool {
    width_nm >= min_feature_nm && height_nm >= min_feature_nm
}

/// Fracture a polygon into trapezoids using horizontal scanline decomposition.
pub fn fracture_polygon(
    vertices: &[Point],
    dose: f64,
    slice_height_nm: f64,
) -> Vec<Trapezoid> {
    if vertices.len() < 3 || slice_height_nm <= 0.0 {
        return vec![];
    }

    let min_y = vertices.iter().map(|v| v.y_nm).fold(f64::MAX, f64::min);
    let max_y = vertices.iter().map(|v| v.y_nm).fold(f64::MIN, f64::max);

    let mut traps = Vec::new();
    let n_slices = ((max_y - min_y) / slice_height_nm).ceil() as usize;

    for s in 0..n_slices {
        let y_bot = min_y + s as f64 * slice_height_nm;
        let y_top = (y_bot + slice_height_nm).min(max_y);
        let h = y_top - y_bot;
        if h <= 0.0 {
            continue;
        }

        // Find intersections at bottom and top scanlines
        let find_intersections = |y: f64| -> Vec<f64> {
            let mut xs = Vec::new();
            let n = vertices.len();
            for i in 0..n {
                let j = (i + 1) % n;
                let y0 = vertices[i].y_nm;
                let y1 = vertices[j].y_nm;
                if (y0 <= y && y1 > y) || (y1 <= y && y0 > y) {
                    let t = (y - y0) / (y1 - y0);
                    xs.push(vertices[i].x_nm + t * (vertices[j].x_nm - vertices[i].x_nm));
                }
            }
            xs.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            xs
        };

        let xs_bot = find_intersections(y_bot + 1e-6);
        let xs_top = find_intersections(y_top - 1e-6);

        // Pair intersections into trapezoids
        let n_pairs = xs_bot.len().min(xs_top.len()) / 2;
        for p in 0..n_pairs {
            let x_bot_left = xs_bot[2 * p];
            let x_bot_right = xs_bot[2 * p + 1];
            let x_top_left = xs_top[2 * p];
            let x_top_right = xs_top[2 * p + 1];
            traps.push(Trapezoid {
                x_nm: x_bot_left.min(x_top_left),
                y_nm: y_bot,
                width_bottom_nm: x_bot_right - x_bot_left,
                width_top_nm: x_top_right - x_top_left,
                height_nm: h,
                dose_uc_cm2: dose,
            });
        }
    }

    traps
}

// ---------------------------------------------------------------------------
// Line Edge Roughness (LER)
// ---------------------------------------------------------------------------

/// Line edge roughness analysis.
pub struct LineEdgeRoughness;

impl LineEdgeRoughness {
    /// Shot noise limited LER (3-sigma) in nm.
    ///
    /// sigma_LER ~ k / sqrt(dose * pixel_area * n_electrons_per_uC)
    /// Simplified: sigma ~ C / sqrt(D) where C is a resist/process constant.
    ///
    /// For PMMA: C ~ 5.0 nm * sqrt(uC/cm²)
    pub fn shot_noise_ler_nm(dose_uc_cm2: f64, resist_constant: f64) -> f64 {
        if dose_uc_cm2 <= 0.0 {
            return f64::MAX;
        }
        resist_constant / dose_uc_cm2.sqrt()
    }

    /// 3-sigma LER from shot noise.
    pub fn three_sigma_ler_nm(dose_uc_cm2: f64, resist_constant: f64) -> f64 {
        3.0 * Self::shot_noise_ler_nm(dose_uc_cm2, resist_constant)
    }

    /// Power spectral density of edge roughness (1D).
    ///
    /// Models LER PSD as Palasantzas model:
    /// PSD(f) = 4 * sigma² * xi / (1 + (2*pi*f*xi)^2)^((H+0.5))
    ///
    /// f: spatial frequency in 1/nm
    /// sigma: RMS roughness in nm
    /// xi: correlation length in nm
    /// h: Hurst (roughness) exponent (0 < H < 1)
    pub fn ler_psd(freq_inv_nm: f64, sigma_nm: f64, correlation_length_nm: f64, hurst_exponent: f64) -> f64 {
        let xi = correlation_length_nm;
        let arg = 2.0 * PI * freq_inv_nm * xi;
        4.0 * sigma_nm * sigma_nm * xi / (1.0 + arg * arg).powf(hurst_exponent + 0.5)
    }

    /// Generate simulated rough edge positions using random walk (simplified).
    ///
    /// Returns y deviations from ideal straight edge at evenly spaced x positions.
    pub fn simulate_rough_edge(
        n_points: usize,
        sigma_nm: f64,
        seed: u64,
    ) -> Vec<f64> {
        // Simple LCG-based pseudo-random for reproducibility
        let mut rng_state = seed;
        let lcg_next = |state: &mut u64| -> f64 {
            *state = state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            // Box-Muller-ish: use two uniform values for Gaussian
            let u = (*state >> 11) as f64 / (1u64 << 53) as f64;
            u
        };

        let mut edge = vec![0.0; n_points];
        let mut prev = 0.0_f64;
        for i in 0..n_points {
            // Generate approximately Gaussian using central limit theorem (sum of 6 uniforms - 3)
            let mut sum = 0.0;
            for _ in 0..6 {
                sum += lcg_next(&mut rng_state);
            }
            let gauss = (sum - 3.0) * sigma_nm * 0.5; // scale
            // Low-pass filter to create correlation
            prev = 0.7 * prev + 0.3 * gauss;
            edge[i] = prev;
        }

        edge
    }

    /// Compute RMS roughness from an edge profile.
    pub fn compute_rms(edge: &[f64]) -> f64 {
        if edge.is_empty() {
            return 0.0;
        }
        let mean = edge.iter().sum::<f64>() / edge.len() as f64;
        let variance = edge.iter().map(|&e| (e - mean) * (e - mean)).sum::<f64>() / edge.len() as f64;
        variance.sqrt()
    }
}

// ---------------------------------------------------------------------------
// Integrated EBL Controller
// ---------------------------------------------------------------------------

/// Main EBL controller that combines all components.
pub struct EblController {
    pub config: BeamConfig,
    pub proximity_params: ProximityParams,
    pub write_field: WriteField,
}

impl EblController {
    /// Create a new EBL controller.
    pub fn new(config: BeamConfig, write_field: WriteField) -> Self {
        let proximity_params = ProximityParams::from_voltage(config.acceleration_voltage_kv);
        Self { config, proximity_params, write_field }
    }

    /// Create exposure grid for the write field.
    pub fn create_grid(&self) -> ExposureGrid {
        ExposureGrid::new(
            self.write_field.width_um * 1000.0,
            self.write_field.height_um * 1000.0,
            self.config.step_size_nm,
        )
    }

    /// Expose a pattern with proximity effect correction.
    pub fn expose_with_pec(
        &self,
        grid: &mut ExposureGrid,
        pattern: &Pattern,
        target_dose: f64,
        pec_iterations: usize,
    ) {
        // First rasterize to get exposed positions
        let mut temp_grid = ExposureGrid::new(
            grid.nx as f64 * grid.step_size_nm,
            grid.ny as f64 * grid.step_size_nm,
            grid.step_size_nm,
        );
        temp_grid.rasterize_pattern(pattern);

        // Collect exposed positions
        let mut positions = Vec::new();
        let mut indices = Vec::new();
        for iy in 0..temp_grid.ny {
            for ix in 0..temp_grid.nx {
                if temp_grid.get_dose(ix, iy) > 0.0 {
                    positions.push(temp_grid.pixel_center(ix, iy));
                    indices.push((ix, iy));
                }
            }
        }

        if positions.is_empty() {
            return;
        }

        // Apply PEC
        let corrector = ProximityCorrector::new(self.proximity_params, pec_iterations, 1e-4);
        let corrected_doses = corrector.correct_doses(&positions, target_dose);

        // Write corrected doses to grid
        for (k, &(ix, iy)) in indices.iter().enumerate() {
            grid.set_dose(ix, iy, corrected_doses[k]);
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-9;

    // --- BeamConfig ---

    #[test]
    fn test_beam_config_new() {
        let config = BeamConfig::new(100.0, 50.0, 2.0, 1.0);
        assert!((config.acceleration_voltage_kv - 100.0).abs() < EPSILON);
        assert!((config.beam_current_pa - 50.0).abs() < EPSILON);
        assert!((config.spot_size_nm - 2.0).abs() < EPSILON);
        assert!((config.step_size_nm - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_beam_config_presets() {
        let hr = BeamConfig::high_resolution();
        assert!((hr.acceleration_voltage_kv - 100.0).abs() < EPSILON);
        assert!((hr.spot_size_nm - 2.0).abs() < EPSILON);

        let gp = BeamConfig::general_purpose();
        assert!((gp.acceleration_voltage_kv - 30.0).abs() < EPSILON);

        let lv = BeamConfig::low_voltage();
        assert!((lv.acceleration_voltage_kv - 10.0).abs() < EPSILON);
    }

    // --- Point ---

    #[test]
    fn test_point_distance() {
        let p1 = Point::new(0.0, 0.0);
        let p2 = Point::new(3.0, 4.0);
        assert!((p1.distance_to(&p2) - 5.0).abs() < EPSILON);
    }

    #[test]
    fn test_point_distance_same() {
        let p = Point::new(10.0, 20.0);
        assert!(p.distance_to(&p) < EPSILON);
    }

    // --- DoseCalculation ---

    #[test]
    fn test_base_dose_calculation() {
        // 100 pA, 1 us dwell, 5 nm step
        let dose = DoseCalculation::base_dose_uc_cm2(100.0, 1e-6, 5.0);
        // I = 100e-12 A, t = 1e-6 s, Q = 1e-16 C = 1e-10 uC
        // A = (5e-7)^2 = 25e-14 cm²
        // D = 1e-10 / 25e-14 = 400 uC/cm²
        assert!((dose - 400.0).abs() < 1.0);
    }

    #[test]
    fn test_dwell_time_from_dose() {
        let dose = 300.0; // uC/cm²
        let step = 5.0; // nm
        let current = 100.0; // pA
        let dwell = DoseCalculation::dwell_time_s(dose, step, current);
        // Inverse: compute dose from dwell
        let dose_check = DoseCalculation::base_dose_uc_cm2(current, dwell, step);
        assert!((dose_check - dose).abs() < 1e-6);
    }

    #[test]
    fn test_dwell_time_roundtrip() {
        for dose in [100.0, 200.0, 500.0] {
            let dwell = DoseCalculation::dwell_time_s(dose, 10.0, 200.0);
            let dose_back = DoseCalculation::base_dose_uc_cm2(200.0, dwell, 10.0);
            assert!((dose - dose_back).abs() < 1e-6, "Roundtrip failed for dose={}", dose);
        }
    }

    #[test]
    fn test_pixel_charge() {
        let charge = DoseCalculation::pixel_charge_fc(300.0, 10.0);
        // 300 uC/cm² * (10e-7 cm)² = 300 * 1e-12 cm² * uC/cm² = 3e-10 uC = 3e-4 fC...
        // Actually: step_cm = 10e-7 = 1e-6, area = 1e-12 cm²
        // charge_uc = 300 * 1e-12 = 3e-10 uC
        // charge_fc = 3e-10 * 1e9 = 0.3 fC
        assert!((charge - 0.3).abs() < 0.01);
    }

    #[test]
    fn test_electrons_per_pixel() {
        let electrons = DoseCalculation::electrons_per_pixel(300.0, 10.0);
        // charge = 0.3 fC = 3e-16 C
        // electrons = 3e-16 / 1.602e-19 ~ 1873
        assert!(electrons > 1800.0 && electrons < 1950.0);
    }

    #[test]
    fn test_zero_step_dose() {
        let dose = DoseCalculation::base_dose_uc_cm2(100.0, 1e-6, 0.0);
        assert!((dose - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_zero_current_dwell() {
        let dwell = DoseCalculation::dwell_time_s(300.0, 5.0, 0.0);
        assert!((dwell - 0.0).abs() < EPSILON);
    }

    // --- ProximityParams ---

    #[test]
    fn test_proximity_from_voltage_100kv() {
        let params = ProximityParams::from_voltage(100.0);
        assert!(params.alpha_nm > 0.0 && params.alpha_nm < 100.0);
        assert!(params.beta_nm > 1000.0); // backscatter range in um scale
        assert!(params.eta > 0.0 && params.eta < 2.0);
    }

    #[test]
    fn test_proximity_from_voltage_30kv() {
        let params = ProximityParams::from_voltage(30.0);
        let params_100 = ProximityParams::from_voltage(100.0);
        // At lower voltage, alpha should be larger (more forward scattering)
        assert!(params.alpha_nm > params_100.alpha_nm);
        // Beta should be smaller (less backscatter range)
        assert!(params.beta_nm < params_100.beta_nm);
    }

    #[test]
    fn test_psf_peak_at_origin() {
        let params = ProximityParams::new(20.0, 5000.0, 0.7);
        let psf_0 = params.psf(0.0);
        let psf_10 = params.psf(10.0);
        let psf_100 = params.psf(100.0);
        // PSF should be maximum at r=0
        assert!(psf_0 > psf_10);
        assert!(psf_10 > psf_100);
    }

    #[test]
    fn test_psf_normalization() {
        // The double-Gaussian PSF integrates to 1 over 2D space
        // Numerical integration: integral = sum(psf(r) * 2*pi*r * dr)
        let params = ProximityParams::new(20.0, 5000.0, 0.7);
        let dr = 1.0;
        let mut integral = 0.0;
        let r_max = 50000.0;
        let mut r = 0.0;
        while r < r_max {
            integral += params.psf(r) * 2.0 * PI * r * dr;
            r += dr;
        }
        // Should be close to 1.0
        assert!((integral - 1.0).abs() < 0.05, "PSF integral = {}", integral);
    }

    #[test]
    fn test_backscatter_kernel_decay() {
        let params = ProximityParams::new(20.0, 5000.0, 0.7);
        let k0 = params.backscatter_kernel(0.0);
        let k_beta = params.backscatter_kernel(5000.0);
        assert!((k0 - 1.0).abs() < EPSILON);
        assert!((k_beta - (-1.0_f64).exp()).abs() < 1e-6);
    }

    // --- Proximity Corrector ---

    #[test]
    fn test_pec_single_pixel() {
        let params = ProximityParams::new(20.0, 5000.0, 0.7);
        let corrector = ProximityCorrector::new(params, 50, 1e-6);
        let positions = vec![Point::new(0.0, 0.0)];
        let doses = corrector.correct_doses(&positions, 300.0);
        // Single pixel: no neighbors, dose should equal target
        assert_eq!(doses.len(), 1);
        assert!((doses[0] - 300.0).abs() < 1.0);
    }

    #[test]
    fn test_pec_two_nearby_pixels() {
        let params = ProximityParams::new(20.0, 5000.0, 0.7);
        let corrector = ProximityCorrector::new(params, 100, 1e-6);
        let positions = vec![
            Point::new(0.0, 0.0),
            Point::new(100.0, 0.0), // 100 nm apart
        ];
        let doses = corrector.correct_doses(&positions, 300.0);
        // Close neighbors should have reduced dose
        assert_eq!(doses.len(), 2);
        // Symmetry: both should have same dose
        assert!((doses[0] - doses[1]).abs() < 1.0);
    }

    #[test]
    fn test_pec_empty() {
        let params = ProximityParams::new(20.0, 5000.0, 0.7);
        let corrector = ProximityCorrector::new(params, 50, 1e-6);
        let doses = corrector.correct_doses(&[], 300.0);
        assert!(doses.is_empty());
    }

    // --- Scan Patterns ---

    #[test]
    fn test_raster_scan_order() {
        let order = raster_scan_order(3, 3);
        assert_eq!(order.len(), 9);
        // Row 0: left to right
        assert_eq!(order[0], (0, 0));
        assert_eq!(order[1], (1, 0));
        assert_eq!(order[2], (2, 0));
        // Row 1: right to left (serpentine)
        assert_eq!(order[3], (2, 1));
        assert_eq!(order[4], (1, 1));
        assert_eq!(order[5], (0, 1));
        // Row 2: left to right
        assert_eq!(order[6], (0, 2));
    }

    #[test]
    fn test_vector_scan_order() {
        let pixels = vec![(0, 0), (10, 10), (1, 0)];
        let order = vector_scan_order(&pixels);
        assert_eq!(order.len(), 3);
        // Should start from nearest to origin: (0,0)
        assert_eq!(order[0], (0, 0));
        // Next nearest: (1,0)
        assert_eq!(order[1], (1, 0));
    }

    #[test]
    fn test_vector_scan_empty() {
        let order = vector_scan_order(&[]);
        assert!(order.is_empty());
    }

    // --- Write Field ---

    #[test]
    fn test_write_field_pixels() {
        let wf = WriteField::new(100.0, 100.0, 50.0, 50.0);
        let px = wf.pixels_x(5.0);
        let py = wf.pixels_y(5.0);
        // 100 um = 100000 nm / 5 nm = 20000 pixels
        assert_eq!(px, 20000);
        assert_eq!(py, 20000);
    }

    #[test]
    fn test_field_stitching() {
        let fields = field_stitch(300.0, 300.0, 100.0, 100.0, 5.0);
        // Step = 95 um, need ceil(300/95) = 4 fields per axis => 16 fields? No: ceil(300/95)=ceil(3.157)=4
        // Actually: ceil(300/95) = 4 in each direction
        assert!(!fields.is_empty());
        // Each field should have correct dimensions
        for f in &fields {
            assert!((f.width_um - 100.0).abs() < EPSILON);
            assert!((f.height_um - 100.0).abs() < EPSILON);
        }
    }

    #[test]
    fn test_field_stitch_single() {
        let fields = field_stitch(50.0, 50.0, 100.0, 100.0, 5.0);
        assert_eq!(fields.len(), 1);
    }

    // --- Resolution Limits ---

    #[test]
    fn test_forward_scatter_broadening() {
        let br = ResolutionLimits::forward_scatter_broadening_nm(100.0, 100.0);
        // 0.9 * (100/100)^1.5 = 0.9 nm
        assert!((br - 0.9).abs() < 0.01);
    }

    #[test]
    fn test_effective_beam_diameter() {
        let eff = ResolutionLimits::effective_beam_diameter_nm(2.0, 100.0, 100.0);
        // sqrt(4 + 0.81) ~ 2.19
        assert!(eff > 2.0 && eff < 3.0);
    }

    #[test]
    fn test_min_feature_size() {
        let mfs = ResolutionLimits::min_feature_size_nm(2.0, 100.0, 100.0, 5.0);
        // effective beam (~2.19) + 5 nm development ~ 7.19
        assert!(mfs > 7.0 && mfs < 8.0);
    }

    #[test]
    fn test_electron_range() {
        let range = ResolutionLimits::electron_range_um(100.0);
        // Should be tens of um at 100 kV in PMMA
        assert!(range > 10.0 && range < 200.0);
    }

    #[test]
    fn test_forward_scatter_zero_voltage() {
        let br = ResolutionLimits::forward_scatter_broadening_nm(100.0, 0.0);
        assert!((br - 0.0).abs() < EPSILON);
    }

    // --- Exposure Grid ---

    #[test]
    fn test_exposure_grid_creation() {
        let grid = ExposureGrid::new(100.0, 200.0, 10.0);
        assert_eq!(grid.nx, 10);
        assert_eq!(grid.ny, 20);
        assert_eq!(grid.doses.len(), 200);
    }

    #[test]
    fn test_grid_set_get_dose() {
        let mut grid = ExposureGrid::new(100.0, 100.0, 10.0);
        grid.set_dose(3, 5, 300.0);
        assert!((grid.get_dose(3, 5) - 300.0).abs() < EPSILON);
        assert!((grid.get_dose(0, 0) - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_rasterize_point() {
        let mut grid = ExposureGrid::new(100.0, 100.0, 10.0);
        grid.rasterize_point(&Point::new(25.0, 35.0), 300.0);
        assert!((grid.get_dose(2, 3) - 300.0).abs() < EPSILON);
        assert_eq!(grid.exposed_pixel_count(), 1);
    }

    #[test]
    fn test_rasterize_horizontal_line() {
        let mut grid = ExposureGrid::new(100.0, 100.0, 10.0);
        grid.rasterize_line(&Point::new(10.0, 50.0), &Point::new(80.0, 50.0), 250.0);
        let count = grid.exposed_pixel_count();
        assert!(count >= 7, "Expected at least 7 exposed pixels, got {}", count);
    }

    #[test]
    fn test_rasterize_rectangle() {
        let mut grid = ExposureGrid::new(200.0, 200.0, 10.0);
        grid.rasterize_rectangle(&Point::new(10.0, 10.0), 50.0, 30.0, 300.0);
        let count = grid.exposed_pixel_count();
        // 50/10 * 30/10 = 5*3 = 15 pixels approximately
        assert!(count >= 12 && count <= 20, "Got {} exposed pixels", count);
    }

    #[test]
    fn test_rasterize_full_circle() {
        let mut grid = ExposureGrid::new(200.0, 200.0, 5.0);
        grid.rasterize_circle_arc(
            &Point::new(100.0, 100.0),
            30.0,
            0.0,
            2.0 * PI,
            300.0,
        );
        let count = grid.exposed_pixel_count();
        // Area = pi * 30² = ~2827 nm², pixel area = 25 nm², expect ~113 pixels
        assert!(count > 80, "Circle should have many pixels, got {}", count);
    }

    #[test]
    fn test_rasterize_polygon_triangle() {
        let mut grid = ExposureGrid::new(200.0, 200.0, 5.0);
        let triangle = vec![
            Point::new(50.0, 10.0),
            Point::new(150.0, 10.0),
            Point::new(100.0, 100.0),
        ];
        grid.rasterize_polygon(&triangle, 300.0);
        let count = grid.exposed_pixel_count();
        // Triangle area = 0.5 * 100 * 90 = 4500 nm², pixel area = 25 nm², ~180 pixels
        assert!(count > 100, "Triangle should have many pixels, got {}", count);
    }

    #[test]
    fn test_rasterize_pattern_enum() {
        let mut grid = ExposureGrid::new(100.0, 100.0, 10.0);
        let pat = Pattern::Rectangle {
            origin: Point::new(10.0, 10.0),
            width_nm: 40.0,
            height_nm: 40.0,
            dose_uc_cm2: 300.0,
        };
        grid.rasterize_pattern(&pat);
        assert!(grid.exposed_pixel_count() > 0);
    }

    #[test]
    fn test_total_write_time() {
        let mut grid = ExposureGrid::new(100.0, 100.0, 10.0);
        grid.rasterize_rectangle(&Point::new(0.0, 0.0), 50.0, 50.0, 300.0);
        let config = BeamConfig::general_purpose();
        let time = grid.total_write_time_s(&config, 1e-6);
        assert!(time > 0.0);
    }

    #[test]
    fn test_exposed_positions() {
        let mut grid = ExposureGrid::new(100.0, 100.0, 10.0);
        grid.set_dose(2, 3, 300.0);
        grid.set_dose(5, 7, 250.0);
        let positions = grid.exposed_positions();
        assert_eq!(positions.len(), 2);
    }

    // --- Pattern Fracturing ---

    #[test]
    fn test_fracture_small_rectangle() {
        let traps = fracture_rectangle(&Point::new(0.0, 0.0), 50.0, 50.0, 300.0, 100.0);
        // Fits in one trapezoid
        assert_eq!(traps.len(), 1);
        assert!((traps[0].width_bottom_nm - 50.0).abs() < EPSILON);
    }

    #[test]
    fn test_fracture_large_rectangle() {
        let traps = fracture_rectangle(&Point::new(0.0, 0.0), 250.0, 100.0, 300.0, 100.0);
        // 250/100=3 pieces in x, 100/100=1 in y => 3 trapezoids
        assert_eq!(traps.len(), 3);
    }

    #[test]
    fn test_check_minimum_feature() {
        assert!(check_minimum_feature(20.0, 20.0, 10.0));
        assert!(!check_minimum_feature(5.0, 20.0, 10.0));
        assert!(!check_minimum_feature(20.0, 5.0, 10.0));
    }

    #[test]
    fn test_fracture_polygon() {
        let triangle = vec![
            Point::new(0.0, 0.0),
            Point::new(100.0, 0.0),
            Point::new(50.0, 100.0),
        ];
        let traps = fracture_polygon(&triangle, 300.0, 20.0);
        assert!(!traps.is_empty(), "Should produce trapezoids");
        // Each trapezoid should have positive height
        for t in &traps {
            assert!(t.height_nm > 0.0);
        }
    }

    // --- Line Edge Roughness ---

    #[test]
    fn test_shot_noise_ler() {
        let ler_low = LineEdgeRoughness::shot_noise_ler_nm(100.0, 5.0);
        let ler_high = LineEdgeRoughness::shot_noise_ler_nm(400.0, 5.0);
        // Higher dose => lower LER
        assert!(ler_high < ler_low);
    }

    #[test]
    fn test_three_sigma_ler() {
        let ler1 = LineEdgeRoughness::shot_noise_ler_nm(300.0, 5.0);
        let ler3 = LineEdgeRoughness::three_sigma_ler_nm(300.0, 5.0);
        assert!((ler3 - 3.0 * ler1).abs() < EPSILON);
    }

    #[test]
    fn test_ler_psd() {
        let psd_low = LineEdgeRoughness::ler_psd(0.001, 3.0, 50.0, 0.5);
        let psd_high = LineEdgeRoughness::ler_psd(0.1, 3.0, 50.0, 0.5);
        // PSD should decrease with frequency
        assert!(psd_low > psd_high);
        assert!(psd_low > 0.0);
    }

    #[test]
    fn test_simulate_rough_edge() {
        let edge = LineEdgeRoughness::simulate_rough_edge(100, 3.0, 42);
        assert_eq!(edge.len(), 100);
        let rms = LineEdgeRoughness::compute_rms(&edge);
        assert!(rms > 0.0 && rms < 10.0);
    }

    #[test]
    fn test_compute_rms_flat() {
        let flat = vec![5.0; 100];
        let rms = LineEdgeRoughness::compute_rms(&flat);
        assert!(rms < EPSILON);
    }

    #[test]
    fn test_compute_rms_empty() {
        let rms = LineEdgeRoughness::compute_rms(&[]);
        assert!((rms - 0.0).abs() < EPSILON);
    }

    // --- EBL Controller ---

    #[test]
    fn test_ebl_controller_creation() {
        let config = BeamConfig::high_resolution();
        let wf = WriteField::new(100.0, 100.0, 50.0, 50.0);
        let ctrl = EblController::new(config, wf);
        assert!(ctrl.proximity_params.alpha_nm > 0.0);
    }

    #[test]
    fn test_ebl_controller_expose() {
        let config = BeamConfig::new(30.0, 200.0, 5.0, 10.0);
        let wf = WriteField::new(1.0, 1.0, 0.5, 0.5); // 1 um x 1 um
        let ctrl = EblController::new(config, wf);
        let mut grid = ctrl.create_grid();

        let pattern = Pattern::Rectangle {
            origin: Point::new(200.0, 200.0),
            width_nm: 200.0,
            height_nm: 200.0,
            dose_uc_cm2: 300.0,
        };

        ctrl.expose_with_pec(&mut grid, &pattern, 300.0, 10);
        assert!(grid.exposed_pixel_count() > 0);
    }
}
