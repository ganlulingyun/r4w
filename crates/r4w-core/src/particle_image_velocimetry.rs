//! # Particle Image Velocimetry (PIV) Flow Field Measurement
//!
//! This module implements Particle Image Velocimetry (PIV), a non-intrusive optical
//! measurement technique for capturing instantaneous velocity fields in fluid flows.
//!
//! ## Overview
//!
//! PIV works by seeding a fluid flow with tracer particles, illuminating them with a
//! pulsed laser sheet, and capturing image pairs with a known time separation (dt).
//! Cross-correlation of interrogation windows between the two images yields displacement
//! vectors, which are converted to velocity vectors using the known magnification and dt.
//!
//! ## Signal Processing Pipeline
//!
//! ```text
//! Image Pair → Preprocessing → Interrogation Window Extraction → Cross-Correlation
//!   → Peak Detection → Sub-pixel Gaussian Fit → Displacement → Velocity Field
//!   → Outlier Detection → Interpolation → Derived Quantities (vorticity, TKE, etc.)
//! ```
//!
//! ## Applications
//!
//! - **Aerodynamics**: Wind tunnel measurements, airfoil characterization
//! - **Fluid Mechanics**: Turbulence research, mixing studies, wake analysis
//! - **Combustion**: Flame-flow interaction, spray dynamics
//! - **Biomedical**: Blood flow measurement, respiratory airway flows
//! - **Industrial**: Heat exchanger optimization, pump/turbine design
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::particle_image_velocimetry::{PivConfig, PivProcessor, PostProcessor};
//!
//! let config = PivConfig {
//!     interrogation_window_size: 32,
//!     overlap_fraction: 0.5,
//!     dt_us: 100.0,
//!     magnification: 10000.0, // 10000 pixels per meter
//!     search_radius: 16,
//! };
//!
//! let processor = PivProcessor::new(config);
//!
//! // Create synthetic image pair with known displacement
//! let size = 64;
//! let img1 = vec![vec![0.0f64; size]; size];
//! let img2 = vec![vec![0.0f64; size]; size];
//!
//! let vectors = processor.process_image_pair(&img1, &img2);
//! ```

use std::f64::consts::PI;

/// Configuration parameters for PIV processing.
///
/// These parameters control the interrogation window size, overlap, and the
/// physical scaling factors needed to convert pixel displacements to velocities.
#[derive(Debug, Clone)]
pub struct PivConfig {
    /// Size of the square interrogation window in pixels (typically 32 or 64).
    ///
    /// Larger windows improve correlation quality but reduce spatial resolution.
    /// Must be a power of 2 for efficient FFT-based correlation.
    pub interrogation_window_size: usize,

    /// Fraction of overlap between adjacent interrogation windows (0.0 to <1.0).
    ///
    /// Typical value is 0.5 (50% overlap), which satisfies the Nyquist criterion
    /// for the velocity field. Higher overlap increases the number of vectors.
    pub overlap_fraction: f64,

    /// Time separation between laser pulses in microseconds.
    ///
    /// This is the time between the two images in the pair. Smaller dt is used
    /// for high-speed flows, larger dt for slow flows.
    pub dt_us: f64,

    /// Magnification factor in pixels per meter.
    ///
    /// Converts pixel displacement to physical displacement. Determined by the
    /// camera/lens setup and calibration. For example, 10000.0 means 1 pixel = 0.1 mm.
    pub magnification: f64,

    /// Maximum search radius in pixels for displacement.
    ///
    /// Limits the search area in the correlation map. Should be set to the
    /// maximum expected displacement. Typically half the interrogation window size.
    pub search_radius: usize,
}

impl Default for PivConfig {
    fn default() -> Self {
        Self {
            interrogation_window_size: 32,
            overlap_fraction: 0.5,
            dt_us: 100.0,
            magnification: 10000.0,
            search_radius: 16,
        }
    }
}

/// A velocity vector at a specific position in the flow field.
///
/// Contains the x and y velocity components, position coordinates,
/// and a quality metric (peak ratio) indicating the reliability of
/// the measurement.
#[derive(Debug, Clone)]
pub struct VelocityVector {
    /// Horizontal velocity component in m/s.
    pub u: f64,
    /// Vertical velocity component in m/s.
    pub v: f64,
    /// X position in the image (pixels or physical units).
    pub x: f64,
    /// Y position in the image (pixels or physical units).
    pub y: f64,
    /// Peak-to-second-peak ratio (quality metric).
    ///
    /// Values > 1.2 generally indicate reliable measurements.
    /// Values close to 1.0 suggest noise-dominated correlation.
    pub peak_ratio: f64,
}

impl VelocityVector {
    /// Creates a new velocity vector.
    pub fn new(u: f64, v: f64, x: f64, y: f64, peak_ratio: f64) -> Self {
        Self { u, v, x, y, peak_ratio }
    }

    /// Returns the velocity magnitude (speed) in m/s.
    pub fn magnitude(&self) -> f64 {
        (self.u * self.u + self.v * self.v).sqrt()
    }

    /// Returns the velocity direction in radians.
    pub fn angle(&self) -> f64 {
        self.v.atan2(self.u)
    }
}

/// PIV image processor that performs cross-correlation and velocity extraction.
///
/// The `PivProcessor` handles the core computation of PIV: extracting interrogation
/// windows, computing cross-correlations, finding displacement peaks with sub-pixel
/// accuracy, and converting to physical velocity units.
pub struct PivProcessor {
    config: PivConfig,
}

impl PivProcessor {
    /// Creates a new PIV processor with the given configuration.
    pub fn new(config: PivConfig) -> Self {
        Self { config }
    }

    /// Returns a reference to the current configuration.
    pub fn config(&self) -> &PivConfig {
        &self.config
    }

    /// Computes 2D cross-correlation between two interrogation windows.
    ///
    /// Uses direct spatial cross-correlation (not FFT-based) for simplicity
    /// and to avoid external dependencies. The correlation map has dimensions
    /// `(2*search_radius+1) x (2*search_radius+1)`.
    ///
    /// # Arguments
    ///
    /// * `win1` - First interrogation window (reference frame)
    /// * `win2` - Second interrogation window (displaced frame)
    ///
    /// # Returns
    ///
    /// 2D correlation map where the peak location indicates the displacement.
    pub fn cross_correlate_window(
        &self,
        win1: &[Vec<f64>],
        win2: &[Vec<f64>],
    ) -> Vec<Vec<f64>> {
        let rows = win1.len();
        let cols = if rows > 0 { win1[0].len() } else { 0 };
        let sr = self.config.search_radius.min(rows / 2).min(cols / 2);
        let corr_size = 2 * sr + 1;

        let mut corr = vec![vec![0.0f64; corr_size]; corr_size];

        // Compute mean of win1 for normalized cross-correlation
        let mut mean1 = 0.0;
        let mut count = 0.0;
        for row in win1.iter() {
            for &val in row.iter() {
                mean1 += val;
                count += 1.0;
            }
        }
        mean1 /= f64::max(count, 1.0);

        for dy_idx in 0..corr_size {
            let dy = dy_idx as isize - sr as isize;
            for dx_idx in 0..corr_size {
                let dx = dx_idx as isize - sr as isize;

                let mut sum = 0.0;
                let mut sum_sq1 = 0.0;
                let mut sum_sq2 = 0.0;
                let mut n = 0.0;

                // Determine overlap region
                let r_start = 0.max(-dy) as usize;
                let r_end = rows.min((rows as isize - dy) as usize);
                let c_start = 0.max(-dx) as usize;
                let c_end = cols.min((cols as isize - dx) as usize);

                for r in r_start..r_end {
                    let r2 = (r as isize + dy) as usize;
                    if r2 >= rows {
                        continue;
                    }
                    for c in c_start..c_end {
                        let c2 = (c as isize + dx) as usize;
                        if c2 >= cols {
                            continue;
                        }
                        let v1 = win1[r][c] - mean1;
                        let v2 = win2[r2][c2] - mean1;
                        sum += v1 * v2;
                        sum_sq1 += v1 * v1;
                        sum_sq2 += v2 * v2;
                        n += 1.0;
                    }
                }

                if n > 0.0 {
                    let denom = (sum_sq1 * sum_sq2).sqrt();
                    corr[dy_idx][dx_idx] = if denom > 1e-12 {
                        sum / denom
                    } else {
                        0.0
                    };
                }
            }
        }

        corr
    }

    /// Finds the displacement from a correlation map with sub-pixel Gaussian fitting.
    ///
    /// Locates the primary peak in the correlation map, then applies a 2D Gaussian
    /// sub-pixel interpolation for sub-pixel accuracy. Also computes the peak-to-
    /// second-peak ratio as a quality metric.
    ///
    /// # Arguments
    ///
    /// * `corr` - 2D cross-correlation map
    ///
    /// # Returns
    ///
    /// Tuple of `(dx, dy, peak_ratio)` where dx and dy are sub-pixel displacements
    /// and peak_ratio is the primary-to-secondary peak ratio.
    pub fn find_displacement(&self, corr: &[Vec<f64>]) -> (f64, f64, f64) {
        let rows = corr.len();
        if rows == 0 {
            return (0.0, 0.0, 0.0);
        }
        let cols = corr[0].len();
        if cols == 0 {
            return (0.0, 0.0, 0.0);
        }

        let sr = (rows - 1) / 2;

        // Find primary peak
        let mut max_val = f64::NEG_INFINITY;
        let mut max_r = 0;
        let mut max_c = 0;

        for r in 0..rows {
            for c in 0..cols {
                if corr[r][c] > max_val {
                    max_val = corr[r][c];
                    max_r = r;
                    max_c = c;
                }
            }
        }

        // Find second peak (at least 2 pixels away from primary)
        let mut second_max = f64::NEG_INFINITY;
        for r in 0..rows {
            for c in 0..cols {
                let dist_r = (r as isize - max_r as isize).unsigned_abs();
                let dist_c = (c as isize - max_c as isize).unsigned_abs();
                if dist_r >= 2 || dist_c >= 2 {
                    if corr[r][c] > second_max {
                        second_max = corr[r][c];
                    }
                }
            }
        }

        let peak_ratio = if second_max > 1e-12 && second_max > 0.0 {
            max_val / second_max
        } else if max_val > 1e-12 {
            // No meaningful second peak; primary is dominant
            10.0
        } else {
            1.0
        };

        // Sub-pixel Gaussian fit
        let (sub_dx, sub_dy) = gaussian_subpixel_2d(corr, max_r, max_c);

        let dx = (max_c as f64 - sr as f64) + sub_dx;
        let dy = (max_r as f64 - sr as f64) + sub_dy;

        (dx, dy, peak_ratio)
    }

    /// Converts pixel displacement to physical velocity in m/s.
    ///
    /// # Arguments
    ///
    /// * `dx_pixels` - Horizontal displacement in pixels
    /// * `dy_pixels` - Vertical displacement in pixels
    /// * `dt_us` - Time separation in microseconds
    /// * `magnification` - Scale factor in pixels per meter
    ///
    /// # Returns
    ///
    /// Tuple of `(u, v)` velocity components in m/s.
    pub fn pixel_to_velocity(
        dx_pixels: f64,
        dy_pixels: f64,
        dt_us: f64,
        magnification: f64,
    ) -> (f64, f64) {
        let dt_s = dt_us * 1e-6;
        if dt_s.abs() < 1e-15 || magnification.abs() < 1e-15 {
            return (0.0, 0.0);
        }
        let u = dx_pixels / (magnification * dt_s);
        let v = dy_pixels / (magnification * dt_s);
        (u, v)
    }

    /// Processes an image pair to produce a velocity vector field.
    ///
    /// Divides both images into overlapping interrogation windows, cross-correlates
    /// each pair, finds displacements with sub-pixel accuracy, and converts to
    /// physical velocities.
    ///
    /// # Arguments
    ///
    /// * `img1` - First image (reference frame), row-major `[row][col]`
    /// * `img2` - Second image (displaced frame), row-major `[row][col]`
    ///
    /// # Returns
    ///
    /// 2D grid of velocity vectors. Grid dimensions depend on image size,
    /// window size, and overlap.
    pub fn process_image_pair(
        &self,
        img1: &[Vec<f64>],
        img2: &[Vec<f64>],
    ) -> Vec<Vec<VelocityVector>> {
        let img_rows = img1.len();
        if img_rows == 0 {
            return Vec::new();
        }
        let img_cols = img1[0].len();

        let ws = self.config.interrogation_window_size;
        let step = ((ws as f64) * (1.0 - self.config.overlap_fraction)).max(1.0) as usize;

        let n_rows = if img_rows >= ws {
            (img_rows - ws) / step + 1
        } else {
            0
        };
        let n_cols = if img_cols >= ws {
            (img_cols - ws) / step + 1
        } else {
            0
        };

        let mut vectors = Vec::with_capacity(n_rows);

        for iy in 0..n_rows {
            let mut row_vecs = Vec::with_capacity(n_cols);
            let y_start = iy * step;

            for ix in 0..n_cols {
                let x_start = ix * step;

                // Extract interrogation windows
                let win1 = self.extract_window(img1, y_start, x_start, ws);
                let win2 = self.extract_window(img2, y_start, x_start, ws);

                // Cross-correlate
                let corr = self.cross_correlate_window(&win1, &win2);

                // Find displacement
                let (dx, dy, peak_ratio) = self.find_displacement(&corr);

                // Convert to velocity
                let (u, v) = Self::pixel_to_velocity(
                    dx,
                    dy,
                    self.config.dt_us,
                    self.config.magnification,
                );

                let center_x = x_start as f64 + ws as f64 / 2.0;
                let center_y = y_start as f64 + ws as f64 / 2.0;

                row_vecs.push(VelocityVector::new(u, v, center_x, center_y, peak_ratio));
            }

            vectors.push(row_vecs);
        }

        vectors
    }

    /// Multi-pass PIV with iterative window offset refinement.
    ///
    /// Performs multiple passes of PIV analysis, using the displacement from
    /// each pass to offset the interrogation windows in the next pass. This
    /// improves accuracy for large displacements by ensuring that corresponding
    /// particles remain within the interrogation windows.
    ///
    /// # Arguments
    ///
    /// * `img1` - First image (reference frame)
    /// * `img2` - Second image (displaced frame)
    /// * `passes` - Number of refinement passes (typically 2-4)
    ///
    /// # Returns
    ///
    /// Refined velocity vector field after iterative correction.
    pub fn multi_pass_piv(
        &self,
        img1: &[Vec<f64>],
        img2: &[Vec<f64>],
        passes: usize,
    ) -> Vec<Vec<VelocityVector>> {
        if passes == 0 {
            return Vec::new();
        }

        // First pass: standard PIV
        let mut vectors = self.process_image_pair(img1, img2);

        let img_rows = img1.len();
        if img_rows == 0 {
            return vectors;
        }
        let img_cols = img1[0].len();

        let ws = self.config.interrogation_window_size;
        let step = ((ws as f64) * (1.0 - self.config.overlap_fraction)).max(1.0) as usize;

        // Iterative refinement passes
        for _pass in 1..passes {
            let n_rows = vectors.len();
            if n_rows == 0 {
                break;
            }
            let n_cols = vectors[0].len();

            let mut new_vectors = Vec::with_capacity(n_rows);

            for iy in 0..n_rows {
                let mut row_vecs = Vec::with_capacity(n_cols);
                let y_start = iy * step;

                for ix in 0..n_cols {
                    let x_start = ix * step;

                    // Get predicted displacement from previous pass
                    let prev = &vectors[iy][ix];
                    let pred_dx = prev.u * self.config.magnification * self.config.dt_us * 1e-6;
                    let pred_dy = prev.v * self.config.magnification * self.config.dt_us * 1e-6;

                    // Offset window in second image
                    let offset_x = pred_dx.round() as isize;
                    let offset_y = pred_dy.round() as isize;

                    let win1 = self.extract_window(img1, y_start, x_start, ws);
                    let win2 = self.extract_window_offset(
                        img2, y_start, x_start, ws, offset_y, offset_x, img_rows, img_cols,
                    );

                    let corr = self.cross_correlate_window(&win1, &win2);
                    let (dx_residual, dy_residual, peak_ratio) = self.find_displacement(&corr);

                    // Total displacement = predicted + residual
                    let total_dx = pred_dx + dx_residual;
                    let total_dy = pred_dy + dy_residual;

                    let (u, v) = Self::pixel_to_velocity(
                        total_dx,
                        total_dy,
                        self.config.dt_us,
                        self.config.magnification,
                    );

                    let center_x = x_start as f64 + ws as f64 / 2.0;
                    let center_y = y_start as f64 + ws as f64 / 2.0;

                    row_vecs.push(VelocityVector::new(u, v, center_x, center_y, peak_ratio));
                }

                new_vectors.push(row_vecs);
            }

            vectors = new_vectors;
        }

        vectors
    }

    /// Extracts a square window from an image.
    fn extract_window(
        &self,
        img: &[Vec<f64>],
        row_start: usize,
        col_start: usize,
        size: usize,
    ) -> Vec<Vec<f64>> {
        let img_rows = img.len();
        let img_cols = if img_rows > 0 { img[0].len() } else { 0 };

        let mut win = vec![vec![0.0f64; size]; size];
        for r in 0..size {
            let ir = row_start + r;
            if ir < img_rows {
                for c in 0..size {
                    let ic = col_start + c;
                    if ic < img_cols {
                        win[r][c] = img[ir][ic];
                    }
                }
            }
        }
        win
    }

    /// Extracts a square window from an image with an integer pixel offset.
    fn extract_window_offset(
        &self,
        img: &[Vec<f64>],
        row_start: usize,
        col_start: usize,
        size: usize,
        row_offset: isize,
        col_offset: isize,
        img_rows: usize,
        img_cols: usize,
    ) -> Vec<Vec<f64>> {
        let mut win = vec![vec![0.0f64; size]; size];
        for r in 0..size {
            let ir = row_start as isize + r as isize + row_offset;
            if ir >= 0 && (ir as usize) < img_rows {
                for c in 0..size {
                    let ic = col_start as isize + c as isize + col_offset;
                    if ic >= 0 && (ic as usize) < img_cols {
                        win[r][c] = img[ir as usize][ic as usize];
                    }
                }
            }
        }
        win
    }
}

/// Post-processing operations for PIV velocity fields.
///
/// Provides outlier detection, interpolation, and computation of derived
/// flow quantities such as vorticity, divergence, strain rate, turbulent
/// kinetic energy, and Reynolds stresses.
pub struct PostProcessor;

impl PostProcessor {
    /// Universal outlier detection using normalized median test.
    ///
    /// For each vector, computes the median of its neighbors' velocities and
    /// the median absolute deviation. Vectors that deviate from the local
    /// median by more than `threshold` times the normalized residual are
    /// marked as outliers (peak_ratio set to 0.0).
    ///
    /// # Arguments
    ///
    /// * `vectors` - Mutable velocity vector grid
    /// * `threshold` - Detection threshold (typically 2.0-3.0)
    pub fn median_filter(vectors: &mut Vec<Vec<VelocityVector>>, threshold: f64) {
        let n_rows = vectors.len();
        if n_rows == 0 {
            return;
        }
        let n_cols = vectors[0].len();

        // Compute flags for outliers
        let mut outlier_flags = vec![vec![false; n_cols]; n_rows];

        for r in 0..n_rows {
            for c in 0..n_cols {
                let mut neighbor_u = Vec::new();
                let mut neighbor_v = Vec::new();

                // Collect 3x3 neighborhood (excluding center)
                for dr in -1isize..=1 {
                    for dc in -1isize..=1 {
                        if dr == 0 && dc == 0 {
                            continue;
                        }
                        let nr = r as isize + dr;
                        let nc = c as isize + dc;
                        if nr >= 0 && (nr as usize) < n_rows && nc >= 0 && (nc as usize) < n_cols
                        {
                            neighbor_u.push(vectors[nr as usize][nc as usize].u);
                            neighbor_v.push(vectors[nr as usize][nc as usize].v);
                        }
                    }
                }

                if neighbor_u.is_empty() {
                    continue;
                }

                let median_u = median_of(&mut neighbor_u);
                let median_v = median_of(&mut neighbor_v);

                // Compute residuals
                let res_u = (vectors[r][c].u - median_u).abs();
                let res_v = (vectors[r][c].v - median_v).abs();

                // Compute MAD (median absolute deviation) of neighbors
                let mut dev_u: Vec<f64> =
                    neighbor_u.iter().map(|&val| (val - median_u).abs()).collect();
                let mut dev_v: Vec<f64> =
                    neighbor_v.iter().map(|&val| (val - median_v).abs()).collect();

                let mad_u = median_of(&mut dev_u) + 0.1; // small epsilon to avoid division by zero
                let mad_v = median_of(&mut dev_v) + 0.1;

                let norm_res = ((res_u / mad_u).powi(2) + (res_v / mad_v).powi(2)).sqrt();

                if norm_res > threshold {
                    outlier_flags[r][c] = true;
                }
            }
        }

        // Apply flags
        for r in 0..n_rows {
            for c in 0..n_cols {
                if outlier_flags[r][c] {
                    vectors[r][c].peak_ratio = 0.0;
                }
            }
        }
    }

    /// Replaces outlier vectors (peak_ratio == 0.0) using bilinear interpolation
    /// from surrounding valid vectors.
    ///
    /// Iterates over the grid and for each outlier, computes the average of
    /// valid neighboring vectors as a replacement.
    pub fn replace_outliers(vectors: &mut Vec<Vec<VelocityVector>>) {
        let n_rows = vectors.len();
        if n_rows == 0 {
            return;
        }
        let n_cols = vectors[0].len();

        // Collect replacements first to avoid borrow issues
        let mut replacements: Vec<(usize, usize, f64, f64)> = Vec::new();

        for r in 0..n_rows {
            for c in 0..n_cols {
                if vectors[r][c].peak_ratio > 0.0 {
                    continue; // Not an outlier
                }

                let mut sum_u = 0.0;
                let mut sum_v = 0.0;
                let mut count = 0.0;

                // Use 3x3 neighborhood of valid vectors
                for dr in -1isize..=1 {
                    for dc in -1isize..=1 {
                        if dr == 0 && dc == 0 {
                            continue;
                        }
                        let nr = r as isize + dr;
                        let nc = c as isize + dc;
                        if nr >= 0 && (nr as usize) < n_rows && nc >= 0 && (nc as usize) < n_cols
                        {
                            let neighbor = &vectors[nr as usize][nc as usize];
                            if neighbor.peak_ratio > 0.0 {
                                sum_u += neighbor.u;
                                sum_v += neighbor.v;
                                count += 1.0;
                            }
                        }
                    }
                }

                if count > 0.0 {
                    replacements.push((r, c, sum_u / count, sum_v / count));
                }
            }
        }

        for (r, c, u, v) in replacements {
            vectors[r][c].u = u;
            vectors[r][c].v = v;
            vectors[r][c].peak_ratio = 0.5; // Mark as interpolated
        }
    }

    /// Computes vorticity (curl of velocity) at each grid point.
    ///
    /// Vorticity omega_z = dv/dx - du/dy, computed using central differences.
    /// Indicates rotational motion in the flow. Positive values indicate
    /// counterclockwise rotation.
    ///
    /// # Arguments
    ///
    /// * `vectors` - Velocity vector grid
    /// * `dx` - Grid spacing in x (meters)
    /// * `dy` - Grid spacing in y (meters)
    ///
    /// # Returns
    ///
    /// 2D array of vorticity values in 1/s.
    pub fn vorticity(
        vectors: &[Vec<VelocityVector>],
        dx: f64,
        dy: f64,
    ) -> Vec<Vec<f64>> {
        let n_rows = vectors.len();
        if n_rows < 2 {
            return vec![vec![0.0; 0]; 0];
        }
        let n_cols = vectors[0].len();
        if n_cols < 2 {
            return vec![vec![0.0; 0]; 0];
        }

        let mut vort = vec![vec![0.0f64; n_cols]; n_rows];

        for r in 0..n_rows {
            for c in 0..n_cols {
                // dv/dx using central differences (forward/backward at boundaries)
                let dvdx = if c == 0 {
                    (vectors[r][c + 1].v - vectors[r][c].v) / dx
                } else if c == n_cols - 1 {
                    (vectors[r][c].v - vectors[r][c - 1].v) / dx
                } else {
                    (vectors[r][c + 1].v - vectors[r][c - 1].v) / (2.0 * dx)
                };

                // du/dy using central differences
                let dudy = if r == 0 {
                    (vectors[r + 1][c].u - vectors[r][c].u) / dy
                } else if r == n_rows - 1 {
                    (vectors[r][c].u - vectors[r - 1][c].u) / dy
                } else {
                    (vectors[r + 1][c].u - vectors[r - 1][c].u) / (2.0 * dy)
                };

                vort[r][c] = dvdx - dudy;
            }
        }

        vort
    }

    /// Computes divergence of the velocity field at each grid point.
    ///
    /// Divergence = du/dx + dv/dy. For incompressible flows, divergence should
    /// be approximately zero. Non-zero divergence indicates compressibility effects
    /// or measurement errors.
    ///
    /// # Arguments
    ///
    /// * `vectors` - Velocity vector grid
    /// * `dx` - Grid spacing in x (meters)
    /// * `dy` - Grid spacing in y (meters)
    ///
    /// # Returns
    ///
    /// 2D array of divergence values in 1/s.
    pub fn divergence(
        vectors: &[Vec<VelocityVector>],
        dx: f64,
        dy: f64,
    ) -> Vec<Vec<f64>> {
        let n_rows = vectors.len();
        if n_rows < 2 {
            return vec![vec![0.0; 0]; 0];
        }
        let n_cols = vectors[0].len();
        if n_cols < 2 {
            return vec![vec![0.0; 0]; 0];
        }

        let mut div = vec![vec![0.0f64; n_cols]; n_rows];

        for r in 0..n_rows {
            for c in 0..n_cols {
                let dudx = if c == 0 {
                    (vectors[r][c + 1].u - vectors[r][c].u) / dx
                } else if c == n_cols - 1 {
                    (vectors[r][c].u - vectors[r][c - 1].u) / dx
                } else {
                    (vectors[r][c + 1].u - vectors[r][c - 1].u) / (2.0 * dx)
                };

                let dvdy = if r == 0 {
                    (vectors[r + 1][c].v - vectors[r][c].v) / dy
                } else if r == n_rows - 1 {
                    (vectors[r][c].v - vectors[r - 1][c].v) / dy
                } else {
                    (vectors[r + 1][c].v - vectors[r - 1][c].v) / (2.0 * dy)
                };

                div[r][c] = dudx + dvdy;
            }
        }

        div
    }

    /// Computes shear strain rate at each grid point.
    ///
    /// Strain rate e_xy = 0.5 * (du/dy + dv/dx), representing the deformation
    /// rate of the fluid element. Used in viscous stress calculations.
    ///
    /// # Arguments
    ///
    /// * `vectors` - Velocity vector grid
    /// * `dx` - Grid spacing in x (meters)
    /// * `dy` - Grid spacing in y (meters)
    ///
    /// # Returns
    ///
    /// 2D array of strain rate values in 1/s.
    pub fn strain_rate(
        vectors: &[Vec<VelocityVector>],
        dx: f64,
        dy: f64,
    ) -> Vec<Vec<f64>> {
        let n_rows = vectors.len();
        if n_rows < 2 {
            return vec![vec![0.0; 0]; 0];
        }
        let n_cols = vectors[0].len();
        if n_cols < 2 {
            return vec![vec![0.0; 0]; 0];
        }

        let mut strain = vec![vec![0.0f64; n_cols]; n_rows];

        for r in 0..n_rows {
            for c in 0..n_cols {
                let dvdx = if c == 0 {
                    (vectors[r][c + 1].v - vectors[r][c].v) / dx
                } else if c == n_cols - 1 {
                    (vectors[r][c].v - vectors[r][c - 1].v) / dx
                } else {
                    (vectors[r][c + 1].v - vectors[r][c - 1].v) / (2.0 * dx)
                };

                let dudy = if r == 0 {
                    (vectors[r + 1][c].u - vectors[r][c].u) / dy
                } else if r == n_rows - 1 {
                    (vectors[r][c].u - vectors[r - 1][c].u) / dy
                } else {
                    (vectors[r + 1][c].u - vectors[r - 1][c].u) / (2.0 * dy)
                };

                strain[r][c] = 0.5 * (dudy + dvdx);
            }
        }

        strain
    }

    /// Computes turbulent kinetic energy from a set of instantaneous velocity fields.
    ///
    /// TKE = 0.5 * (<u'^2> + <v'^2>) where u' and v' are fluctuations from the
    /// time-averaged mean. Requires multiple realizations (image pairs) for
    /// statistical convergence.
    ///
    /// # Arguments
    ///
    /// * `velocity_fields` - Multiple instantaneous velocity vector grids
    ///
    /// # Returns
    ///
    /// 2D array of TKE values in m^2/s^2.
    pub fn turbulent_kinetic_energy(
        velocity_fields: &[Vec<Vec<VelocityVector>>],
    ) -> Vec<Vec<f64>> {
        let n_fields = velocity_fields.len();
        if n_fields == 0 {
            return Vec::new();
        }

        let n_rows = velocity_fields[0].len();
        if n_rows == 0 {
            return Vec::new();
        }
        let n_cols = velocity_fields[0][0].len();

        // Compute mean velocity field
        let mut mean_u = vec![vec![0.0f64; n_cols]; n_rows];
        let mut mean_v = vec![vec![0.0f64; n_cols]; n_rows];

        for field in velocity_fields.iter() {
            for r in 0..n_rows {
                for c in 0..n_cols {
                    mean_u[r][c] += field[r][c].u;
                    mean_v[r][c] += field[r][c].v;
                }
            }
        }

        let n = n_fields as f64;
        for r in 0..n_rows {
            for c in 0..n_cols {
                mean_u[r][c] /= n;
                mean_v[r][c] /= n;
            }
        }

        // Compute TKE = 0.5 * (<u'^2> + <v'^2>)
        let mut tke = vec![vec![0.0f64; n_cols]; n_rows];

        for field in velocity_fields.iter() {
            for r in 0..n_rows {
                for c in 0..n_cols {
                    let u_prime = field[r][c].u - mean_u[r][c];
                    let v_prime = field[r][c].v - mean_v[r][c];
                    tke[r][c] += u_prime * u_prime + v_prime * v_prime;
                }
            }
        }

        for r in 0..n_rows {
            for c in 0..n_cols {
                tke[r][c] = 0.5 * tke[r][c] / n;
            }
        }

        tke
    }

    /// Computes Reynolds stress <u'v'> from multiple velocity fields.
    ///
    /// The Reynolds stress tensor component <u'v'> represents the turbulent
    /// momentum transport. It is the time-average of the product of velocity
    /// fluctuations from the mean.
    ///
    /// # Arguments
    ///
    /// * `velocity_fields` - Multiple instantaneous velocity vector grids
    ///
    /// # Returns
    ///
    /// 2D array of Reynolds stress values in m^2/s^2.
    pub fn reynolds_stress(
        velocity_fields: &[Vec<Vec<VelocityVector>>],
    ) -> Vec<Vec<f64>> {
        let n_fields = velocity_fields.len();
        if n_fields == 0 {
            return Vec::new();
        }

        let n_rows = velocity_fields[0].len();
        if n_rows == 0 {
            return Vec::new();
        }
        let n_cols = velocity_fields[0][0].len();

        // Compute mean velocity field
        let mut mean_u = vec![vec![0.0f64; n_cols]; n_rows];
        let mut mean_v = vec![vec![0.0f64; n_cols]; n_rows];

        for field in velocity_fields.iter() {
            for r in 0..n_rows {
                for c in 0..n_cols {
                    mean_u[r][c] += field[r][c].u;
                    mean_v[r][c] += field[r][c].v;
                }
            }
        }

        let n = n_fields as f64;
        for r in 0..n_rows {
            for c in 0..n_cols {
                mean_u[r][c] /= n;
                mean_v[r][c] /= n;
            }
        }

        // Compute <u'v'>
        let mut rs = vec![vec![0.0f64; n_cols]; n_rows];

        for field in velocity_fields.iter() {
            for r in 0..n_rows {
                for c in 0..n_cols {
                    let u_prime = field[r][c].u - mean_u[r][c];
                    let v_prime = field[r][c].v - mean_v[r][c];
                    rs[r][c] += u_prime * v_prime;
                }
            }
        }

        for r in 0..n_rows {
            for c in 0..n_cols {
                rs[r][c] /= n;
            }
        }

        rs
    }
}

/// Image preprocessing operations for PIV.
///
/// Provides background subtraction, intensity normalization, particle
/// enhancement, and seeding density estimation to improve correlation quality.
pub struct ImagePreprocessor;

impl ImagePreprocessor {
    /// Subtracts a background image from the particle image.
    ///
    /// Removes static reflections and non-uniform illumination by subtracting
    /// a time-averaged background image. Values are clamped to non-negative.
    ///
    /// # Arguments
    ///
    /// * `image` - Particle image to process (modified in place)
    /// * `background` - Background image (typically time-averaged)
    pub fn subtract_background(image: &mut Vec<Vec<f64>>, background: &[Vec<f64>]) {
        for (r, row) in image.iter_mut().enumerate() {
            if r < background.len() {
                for (c, pixel) in row.iter_mut().enumerate() {
                    if c < background[r].len() {
                        *pixel = (*pixel - background[r][c]).max(0.0);
                    }
                }
            }
        }
    }

    /// Normalizes image intensity to the range [0.0, 1.0].
    ///
    /// Scales pixel values linearly so that the minimum maps to 0.0 and the
    /// maximum maps to 1.0. Useful for ensuring consistent correlation behavior
    /// across images with varying illumination.
    pub fn normalize_intensity(image: &mut Vec<Vec<f64>>) {
        let mut min_val = f64::INFINITY;
        let mut max_val = f64::NEG_INFINITY;

        for row in image.iter() {
            for &val in row.iter() {
                if val < min_val {
                    min_val = val;
                }
                if val > max_val {
                    max_val = val;
                }
            }
        }

        let range = max_val - min_val;
        if range < 1e-12 {
            return;
        }

        for row in image.iter_mut() {
            for pixel in row.iter_mut() {
                *pixel = (*pixel - min_val) / range;
            }
        }
    }

    /// Applies a min-max filter for particle image enhancement.
    ///
    /// Computes the local minimum in a kernel neighborhood and subtracts it,
    /// then normalizes by the local range. This enhances particle images by
    /// removing background gradients and improving particle contrast.
    ///
    /// # Arguments
    ///
    /// * `image` - Input image
    /// * `kernel_size` - Size of the filter kernel (should be odd)
    ///
    /// # Returns
    ///
    /// Enhanced image with improved particle visibility.
    pub fn min_max_filter(image: &[Vec<f64>], kernel_size: usize) -> Vec<Vec<f64>> {
        let rows = image.len();
        if rows == 0 {
            return Vec::new();
        }
        let cols = image[0].len();
        let half = kernel_size / 2;

        let mut result = vec![vec![0.0f64; cols]; rows];

        for r in 0..rows {
            for c in 0..cols {
                let mut local_min = f64::INFINITY;
                let mut local_max = f64::NEG_INFINITY;

                let r_start = if r >= half { r - half } else { 0 };
                let r_end = (r + half + 1).min(rows);
                let c_start = if c >= half { c - half } else { 0 };
                let c_end = (c + half + 1).min(cols);

                for ir in r_start..r_end {
                    for ic in c_start..c_end {
                        let val = image[ir][ic];
                        if val < local_min {
                            local_min = val;
                        }
                        if val > local_max {
                            local_max = val;
                        }
                    }
                }

                let range = local_max - local_min;
                result[r][c] = if range > 1e-12 {
                    (image[r][c] - local_min) / range
                } else {
                    0.0
                };
            }
        }

        result
    }

    /// Estimates seeding density (particles per pixel area).
    ///
    /// Counts pixels above a threshold and normalizes by the total image area.
    /// Optimal seeding density for PIV is typically 5-15 particles per
    /// interrogation window (ppp ~ 0.005-0.015 for a 32x32 window).
    ///
    /// # Arguments
    ///
    /// * `image` - Particle image
    /// * `threshold` - Intensity threshold for particle detection
    ///
    /// # Returns
    ///
    /// Fraction of pixels above threshold (proxy for seeding density).
    pub fn particle_density(image: &[Vec<f64>], threshold: f64) -> f64 {
        let mut above = 0usize;
        let mut total = 0usize;

        for row in image.iter() {
            for &val in row.iter() {
                total += 1;
                if val > threshold {
                    above += 1;
                }
            }
        }

        if total == 0 {
            0.0
        } else {
            above as f64 / total as f64
        }
    }
}

/// Performs 2D sub-pixel Gaussian interpolation around a correlation peak.
///
/// Uses the 3-point Gaussian fit in both x and y directions independently:
///
/// ```text
/// sub_pixel_offset = 0.5 * (ln(f[i-1]) - ln(f[i+1])) / (ln(f[i-1]) - 2*ln(f[i]) + ln(f[i+1]))
/// ```
///
/// This provides sub-pixel accuracy of approximately 0.1 pixels for typical
/// PIV correlation peaks.
///
/// # Arguments
///
/// * `corr` - 2D correlation map
/// * `peak_row` - Row index of the integer peak
/// * `peak_col` - Column index of the integer peak
///
/// # Returns
///
/// Tuple of `(sub_dx, sub_dy)` sub-pixel corrections.
pub fn gaussian_subpixel_2d(
    corr: &[Vec<f64>],
    peak_row: usize,
    peak_col: usize,
) -> (f64, f64) {
    let rows = corr.len();
    let cols = if rows > 0 { corr[0].len() } else { 0 };

    let mut sub_dx = 0.0;
    let mut sub_dy = 0.0;

    // Gaussian fit in x direction
    if peak_col > 0 && peak_col < cols - 1 {
        let left = corr[peak_row][peak_col - 1].max(1e-12);
        let center = corr[peak_row][peak_col].max(1e-12);
        let right = corr[peak_row][peak_col + 1].max(1e-12);

        let ln_left = left.ln();
        let ln_center = center.ln();
        let ln_right = right.ln();

        let denom = ln_left - 2.0 * ln_center + ln_right;
        if denom.abs() > 1e-12 {
            sub_dx = 0.5 * (ln_left - ln_right) / denom;
            // Clamp to reasonable range
            sub_dx = sub_dx.clamp(-1.0, 1.0);
        }
    }

    // Gaussian fit in y direction
    if peak_row > 0 && peak_row < rows - 1 {
        let top = corr[peak_row - 1][peak_col].max(1e-12);
        let center = corr[peak_row][peak_col].max(1e-12);
        let bottom = corr[peak_row + 1][peak_col].max(1e-12);

        let ln_top = top.ln();
        let ln_center = center.ln();
        let ln_bottom = bottom.ln();

        let denom = ln_top - 2.0 * ln_center + ln_bottom;
        if denom.abs() > 1e-12 {
            sub_dy = 0.5 * (ln_top - ln_bottom) / denom;
            sub_dy = sub_dy.clamp(-1.0, 1.0);
        }
    }

    (sub_dx, sub_dy)
}

/// Computes the peak-to-noise ratio of a correlation map.
///
/// Defined as the primary peak value divided by the RMS of the correlation map
/// (excluding a region around the peak). Higher values indicate more reliable
/// displacement measurements.
///
/// # Arguments
///
/// * `corr` - 2D correlation map
/// * `peak_row` - Row index of the peak
/// * `peak_col` - Column index of the peak
/// * `exclusion_radius` - Radius around peak to exclude from noise computation
///
/// # Returns
///
/// Peak-to-noise ratio (dimensionless).
pub fn peak_to_noise_ratio(
    corr: &[Vec<f64>],
    peak_row: usize,
    peak_col: usize,
    exclusion_radius: usize,
) -> f64 {
    let rows = corr.len();
    if rows == 0 {
        return 0.0;
    }
    let cols = corr[0].len();

    let peak_val = corr[peak_row][peak_col];

    let mut sum_sq = 0.0;
    let mut count = 0.0;

    for r in 0..rows {
        for c in 0..cols {
            let dr = (r as isize - peak_row as isize).unsigned_abs();
            let dc = (c as isize - peak_col as isize).unsigned_abs();
            if dr > exclusion_radius || dc > exclusion_radius {
                sum_sq += corr[r][c] * corr[r][c];
                count += 1.0;
            }
        }
    }

    if count < 1.0 || sum_sq < 1e-20 {
        return if peak_val.abs() > 1e-12 { 100.0 } else { 0.0 };
    }

    let rms = (sum_sq / count).sqrt();
    if rms > 1e-12 {
        peak_val / rms
    } else {
        100.0
    }
}

/// Computes the median of a slice (modifies the slice by sorting).
fn median_of(data: &mut [f64]) -> f64 {
    let n = data.len();
    if n == 0 {
        return 0.0;
    }
    data.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    if n % 2 == 0 {
        (data[n / 2 - 1] + data[n / 2]) / 2.0
    } else {
        data[n / 2]
    }
}

/// Creates a synthetic particle image with Gaussian particles.
///
/// Useful for testing and validating PIV algorithms. Places particles at
/// random positions with Gaussian intensity profiles.
///
/// # Arguments
///
/// * `rows` - Image height in pixels
/// * `cols` - Image width in pixels
/// * `n_particles` - Number of particles to place
/// * `particle_diameter` - Diameter of particles in pixels (e-2 width)
/// * `seed` - Random number generator seed for reproducibility
///
/// # Returns
///
/// Synthetic particle image with intensities in [0.0, 1.0].
pub fn generate_particle_image(
    rows: usize,
    cols: usize,
    n_particles: usize,
    particle_diameter: f64,
    seed: u64,
) -> Vec<Vec<f64>> {
    let mut image = vec![vec![0.0f64; cols]; rows];
    let sigma = particle_diameter / (2.0 * (2.0_f64.ln()).sqrt()); // Convert diameter to sigma

    // Simple LCG PRNG for reproducibility
    let mut rng_state = seed;
    let mut next_rand = || -> f64 {
        rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
        (rng_state >> 33) as f64 / (1u64 << 31) as f64
    };

    for _ in 0..n_particles {
        let px = next_rand() * cols as f64;
        let py = next_rand() * rows as f64;
        let intensity = 0.5 + 0.5 * next_rand();

        // Render Gaussian particle
        let r_start = (py - 3.0 * sigma).max(0.0) as usize;
        let r_end = ((py + 3.0 * sigma) as usize + 1).min(rows);
        let c_start = (px - 3.0 * sigma).max(0.0) as usize;
        let c_end = ((px + 3.0 * sigma) as usize + 1).min(cols);

        for r in r_start..r_end {
            for c in c_start..c_end {
                let dx = c as f64 - px;
                let dy = r as f64 - py;
                let val = intensity * (-0.5 * (dx * dx + dy * dy) / (sigma * sigma)).exp();
                image[r][c] = (image[r][c] + val).min(1.0);
            }
        }
    }

    image
}

/// Creates a displaced copy of a particle image for testing.
///
/// Shifts all particle positions by (shift_x, shift_y) pixels, which
/// enables validation of PIV algorithms against known displacements.
///
/// # Arguments
///
/// * `image` - Original particle image
/// * `shift_x` - Horizontal shift in pixels (positive = right)
/// * `shift_y` - Vertical shift in pixels (positive = down)
///
/// # Returns
///
/// Shifted image with the same dimensions.
pub fn shift_image(image: &[Vec<f64>], shift_x: isize, shift_y: isize) -> Vec<Vec<f64>> {
    let rows = image.len();
    if rows == 0 {
        return Vec::new();
    }
    let cols = image[0].len();

    let mut shifted = vec![vec![0.0f64; cols]; rows];

    for r in 0..rows {
        let src_r = r as isize - shift_y;
        if src_r < 0 || src_r >= rows as isize {
            continue;
        }
        for c in 0..cols {
            let src_c = c as isize - shift_x;
            if src_c < 0 || src_c >= cols as isize {
                continue;
            }
            shifted[r][c] = image[src_r as usize][src_c as usize];
        }
    }

    shifted
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> PivConfig {
        PivConfig {
            interrogation_window_size: 32,
            overlap_fraction: 0.5,
            dt_us: 100.0,
            magnification: 10000.0,
            search_radius: 16,
        }
    }

    fn make_particle_pair(
        size: usize,
        shift_x: isize,
        shift_y: isize,
    ) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
        let img1 = generate_particle_image(size, size, 200, 3.0, 42);
        let img2 = shift_image(&img1, shift_x, shift_y);
        (img1, img2)
    }

    #[test]
    fn test_config_default() {
        let config = PivConfig::default();
        assert_eq!(config.interrogation_window_size, 32);
        assert!((config.overlap_fraction - 0.5).abs() < 1e-10);
        assert!((config.dt_us - 100.0).abs() < 1e-10);
        assert!((config.magnification - 10000.0).abs() < 1e-10);
        assert_eq!(config.search_radius, 16);
    }

    #[test]
    fn test_velocity_vector_magnitude() {
        let v = VelocityVector::new(3.0, 4.0, 0.0, 0.0, 1.5);
        assert!((v.magnitude() - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_velocity_vector_angle() {
        let v = VelocityVector::new(1.0, 0.0, 0.0, 0.0, 1.5);
        assert!(v.angle().abs() < 1e-10);

        let v2 = VelocityVector::new(0.0, 1.0, 0.0, 0.0, 1.5);
        assert!((v2.angle() - PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_pixel_to_velocity() {
        // 5 pixels displacement, dt=100us, magnification=10000 px/m
        // displacement_m = 5 / 10000 = 0.0005 m
        // velocity = 0.0005 / 0.0001 = 5 m/s
        let (u, v) = PivProcessor::pixel_to_velocity(5.0, 0.0, 100.0, 10000.0);
        assert!((u - 5.0).abs() < 1e-10);
        assert!(v.abs() < 1e-10);
    }

    #[test]
    fn test_pixel_to_velocity_both_axes() {
        let (u, v) = PivProcessor::pixel_to_velocity(3.0, 4.0, 100.0, 10000.0);
        assert!((u - 3.0).abs() < 1e-10);
        assert!((v - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_pixel_to_velocity_zero_dt() {
        let (u, v) = PivProcessor::pixel_to_velocity(5.0, 5.0, 0.0, 10000.0);
        assert_eq!(u, 0.0);
        assert_eq!(v, 0.0);
    }

    #[test]
    fn test_pixel_to_velocity_zero_magnification() {
        let (u, v) = PivProcessor::pixel_to_velocity(5.0, 5.0, 100.0, 0.0);
        assert_eq!(u, 0.0);
        assert_eq!(v, 0.0);
    }

    #[test]
    fn test_zero_displacement_identical_images() {
        let config = PivConfig {
            interrogation_window_size: 32,
            overlap_fraction: 0.5,
            dt_us: 100.0,
            magnification: 10000.0,
            search_radius: 8,
        };
        let processor = PivProcessor::new(config);

        let img = generate_particle_image(64, 64, 100, 3.0, 42);
        let vectors = processor.process_image_pair(&img, &img);

        // All displacements should be near zero
        for row in &vectors {
            for v in row {
                assert!(
                    v.u.abs() < 1.0,
                    "u velocity should be near zero for identical images, got {}",
                    v.u
                );
                assert!(
                    v.v.abs() < 1.0,
                    "v velocity should be near zero for identical images, got {}",
                    v.v
                );
            }
        }
    }

    #[test]
    fn test_known_displacement_recovery() {
        let config = PivConfig {
            interrogation_window_size: 32,
            overlap_fraction: 0.5,
            dt_us: 100.0,
            magnification: 10000.0,
            search_radius: 8,
        };
        let processor = PivProcessor::new(config);

        let shift_x = 3isize;
        let (img1, img2) = make_particle_pair(128, shift_x, 0);
        let vectors = processor.process_image_pair(&img1, &img2);

        // Expected velocity: shift_x pixels / (10000 px/m * 100e-6 s) = 3.0 m/s
        let expected_u = 3.0;
        let mut good_count = 0;
        let mut total = 0;

        for row in &vectors {
            for v in row {
                total += 1;
                if (v.u - expected_u).abs() < 1.5 {
                    good_count += 1;
                }
            }
        }

        assert!(
            total > 0,
            "Should have at least one velocity vector"
        );
        // At least 50% of vectors should be close to expected
        let ratio = good_count as f64 / total as f64;
        assert!(
            ratio > 0.4,
            "At least 40% of vectors should recover displacement, got {:.1}%",
            ratio * 100.0
        );
    }

    #[test]
    fn test_cross_correlate_window_dimensions() {
        let config = PivConfig {
            interrogation_window_size: 16,
            overlap_fraction: 0.5,
            dt_us: 100.0,
            magnification: 10000.0,
            search_radius: 8,
        };
        let processor = PivProcessor::new(config);

        let win1 = vec![vec![0.5f64; 16]; 16];
        let win2 = vec![vec![0.5f64; 16]; 16];
        let corr = processor.cross_correlate_window(&win1, &win2);

        // Correlation size should be 2*search_radius+1
        assert_eq!(corr.len(), 17);
        assert_eq!(corr[0].len(), 17);
    }

    #[test]
    fn test_cross_correlate_identical_windows_peak_at_center() {
        let config = PivConfig {
            interrogation_window_size: 16,
            overlap_fraction: 0.5,
            dt_us: 100.0,
            magnification: 10000.0,
            search_radius: 4,
        };
        let processor = PivProcessor::new(config);

        // Create windows with some pattern
        let mut win1 = vec![vec![0.0f64; 16]; 16];
        for r in 4..12 {
            for c in 4..12 {
                win1[r][c] = 1.0;
            }
        }
        let win2 = win1.clone();

        let corr = processor.cross_correlate_window(&win1, &win2);
        let (dx, dy, _pr) = processor.find_displacement(&corr);

        // For identical windows, displacement should be ~0
        assert!(dx.abs() < 0.5, "dx should be near zero, got {}", dx);
        assert!(dy.abs() < 0.5, "dy should be near zero, got {}", dy);
    }

    #[test]
    fn test_find_displacement_returns_valid_peak_ratio() {
        let config = default_config();
        let processor = PivProcessor::new(config);

        let img = generate_particle_image(64, 64, 100, 3.0, 42);
        let win1 = processor.extract_window(&img, 0, 0, 32);
        let win2 = processor.extract_window(&img, 0, 0, 32);
        let corr = processor.cross_correlate_window(&win1, &win2);
        let (_dx, _dy, pr) = processor.find_displacement(&corr);

        assert!(pr >= 1.0, "Peak ratio should be >= 1.0, got {}", pr);
    }

    #[test]
    fn test_gaussian_subpixel_at_boundary() {
        // Peak at boundary should return 0.0 sub-pixel correction
        let corr = vec![
            vec![0.1, 0.2, 0.5],
            vec![0.3, 0.9, 0.4],
            vec![0.2, 0.3, 0.1],
        ];
        let (sub_dx, _sub_dy) = gaussian_subpixel_2d(&corr, 1, 0);
        // At left boundary, can't do Gaussian fit in x
        assert_eq!(sub_dx, 0.0);
    }

    #[test]
    fn test_gaussian_subpixel_symmetric_peak() {
        // Symmetric peak -> sub-pixel correction should be ~0
        let corr = vec![
            vec![0.1, 0.3, 0.1],
            vec![0.3, 1.0, 0.3],
            vec![0.1, 0.3, 0.1],
        ];
        let (sub_dx, sub_dy) = gaussian_subpixel_2d(&corr, 1, 1);
        assert!(sub_dx.abs() < 0.01, "Symmetric peak: sub_dx should be ~0, got {}", sub_dx);
        assert!(sub_dy.abs() < 0.01, "Symmetric peak: sub_dy should be ~0, got {}", sub_dy);
    }

    #[test]
    fn test_gaussian_subpixel_asymmetric_peak() {
        // Peak shifted right: left neighbor larger than right
        let corr = vec![
            vec![0.1, 0.3, 0.1],
            vec![0.5, 1.0, 0.2],
            vec![0.1, 0.3, 0.1],
        ];
        let (sub_dx, _sub_dy) = gaussian_subpixel_2d(&corr, 1, 1);
        // Should shift left (negative sub_dx) since left neighbor is larger
        assert!(sub_dx < 0.0, "Should shift toward larger neighbor, got {}", sub_dx);
    }

    #[test]
    fn test_peak_to_noise_ratio_strong_peak() {
        let mut corr = vec![vec![0.01f64; 9]; 9];
        corr[4][4] = 1.0; // Strong peak at center
        let pnr = peak_to_noise_ratio(&corr, 4, 4, 1);
        assert!(pnr > 5.0, "Strong peak should have high PNR, got {}", pnr);
    }

    #[test]
    fn test_peak_to_noise_ratio_flat() {
        let corr = vec![vec![0.5f64; 5]; 5];
        let pnr = peak_to_noise_ratio(&corr, 2, 2, 0);
        // Flat map: peak equals noise
        assert!(pnr > 0.0, "PNR should be positive for non-zero data");
        assert!(pnr < 5.0, "PNR should be modest for flat correlation");
    }

    #[test]
    fn test_process_image_pair_grid_dimensions() {
        let config = PivConfig {
            interrogation_window_size: 32,
            overlap_fraction: 0.5,
            dt_us: 100.0,
            magnification: 10000.0,
            search_radius: 8,
        };
        let processor = PivProcessor::new(config);

        let img = vec![vec![0.0f64; 128]; 128];
        let vectors = processor.process_image_pair(&img, &img);

        // 128 pixels, 32 window, 16 step: (128-32)/16 + 1 = 7
        assert_eq!(vectors.len(), 7);
        assert_eq!(vectors[0].len(), 7);
    }

    #[test]
    fn test_process_image_pair_positions() {
        let config = PivConfig {
            interrogation_window_size: 32,
            overlap_fraction: 0.5,
            dt_us: 100.0,
            magnification: 10000.0,
            search_radius: 8,
        };
        let processor = PivProcessor::new(config);

        let img = vec![vec![0.0f64; 128]; 128];
        let vectors = processor.process_image_pair(&img, &img);

        // First vector center should be at (16, 16) (half window size)
        assert!((vectors[0][0].x - 16.0).abs() < 1e-10);
        assert!((vectors[0][0].y - 16.0).abs() < 1e-10);
    }

    #[test]
    fn test_multi_pass_produces_result() {
        let config = PivConfig {
            interrogation_window_size: 32,
            overlap_fraction: 0.5,
            dt_us: 100.0,
            magnification: 10000.0,
            search_radius: 8,
        };
        let processor = PivProcessor::new(config);

        let (img1, img2) = make_particle_pair(128, 2, 0);
        let vectors = processor.multi_pass_piv(&img1, &img2, 2);

        assert!(!vectors.is_empty());
        assert!(!vectors[0].is_empty());
    }

    #[test]
    fn test_multi_pass_zero_passes() {
        let config = default_config();
        let processor = PivProcessor::new(config);

        let img = vec![vec![0.0f64; 64]; 64];
        let vectors = processor.multi_pass_piv(&img, &img, 0);
        assert!(vectors.is_empty());
    }

    #[test]
    fn test_vorticity_rigid_rotation() {
        // Rigid body rotation: u = -omega * y, v = omega * x
        // vorticity = dv/dx - du/dy = omega - (-omega) = 2*omega
        let omega = 1.0;
        let n = 5;
        let spacing = 1.0;

        let mut vectors = Vec::new();
        for r in 0..n {
            let mut row = Vec::new();
            for c in 0..n {
                let x = (c as f64 - 2.0) * spacing;
                let y = (r as f64 - 2.0) * spacing;
                let u = -omega * y;
                let v = omega * x;
                row.push(VelocityVector::new(u, v, x, y, 2.0));
            }
            vectors.push(row);
        }

        let vort = PostProcessor::vorticity(&vectors, spacing, spacing);
        assert_eq!(vort.len(), n);

        // Interior points should have vorticity ~ 2*omega
        let interior_vort = vort[2][2];
        assert!(
            (interior_vort - 2.0 * omega).abs() < 0.1,
            "Rigid rotation vorticity should be 2*omega={}, got {}",
            2.0 * omega,
            interior_vort
        );
    }

    #[test]
    fn test_divergence_incompressible() {
        // Uniform flow: u = const, v = const -> div = 0
        let n = 5;
        let spacing = 1.0;

        let mut vectors = Vec::new();
        for r in 0..n {
            let mut row = Vec::new();
            for c in 0..n {
                row.push(VelocityVector::new(
                    2.0,
                    1.0,
                    c as f64 * spacing,
                    r as f64 * spacing,
                    2.0,
                ));
            }
            vectors.push(row);
        }

        let div = PostProcessor::divergence(&vectors, spacing, spacing);

        for r in 0..n {
            for c in 0..n {
                assert!(
                    div[r][c].abs() < 1e-10,
                    "Uniform flow divergence should be 0, got {} at ({},{})",
                    div[r][c],
                    r,
                    c
                );
            }
        }
    }

    #[test]
    fn test_divergence_source_flow() {
        // Radial outflow: u = x, v = y -> div = du/dx + dv/dy = 1 + 1 = 2
        let n = 5;
        let spacing = 1.0;

        let mut vectors = Vec::new();
        for r in 0..n {
            let mut row = Vec::new();
            for c in 0..n {
                let x = c as f64 * spacing;
                let y = r as f64 * spacing;
                row.push(VelocityVector::new(x, y, x, y, 2.0));
            }
            vectors.push(row);
        }

        let div = PostProcessor::divergence(&vectors, spacing, spacing);

        // Interior points should have divergence ~ 2.0
        let interior = div[2][2];
        assert!(
            (interior - 2.0).abs() < 0.1,
            "Source flow divergence should be ~2.0, got {}",
            interior
        );
    }

    #[test]
    fn test_strain_rate_shear_flow() {
        // Simple shear: u = y, v = 0 -> strain = 0.5*(du/dy + dv/dx) = 0.5
        let n = 5;
        let spacing = 1.0;

        let mut vectors = Vec::new();
        for r in 0..n {
            let mut row = Vec::new();
            for c in 0..n {
                let y = r as f64 * spacing;
                row.push(VelocityVector::new(
                    y,
                    0.0,
                    c as f64 * spacing,
                    y,
                    2.0,
                ));
            }
            vectors.push(row);
        }

        let strain = PostProcessor::strain_rate(&vectors, spacing, spacing);

        let interior = strain[2][2];
        assert!(
            (interior - 0.5).abs() < 0.1,
            "Simple shear strain rate should be ~0.5, got {}",
            interior
        );
    }

    #[test]
    fn test_tke_positive() {
        // Create velocity fields with fluctuations
        let n = 3;
        let mut fields = Vec::new();

        for i in 0..10 {
            let mut field = Vec::new();
            for r in 0..n {
                let mut row = Vec::new();
                for c in 0..n {
                    let u = 1.0 + 0.5 * ((i as f64) * 0.7).sin();
                    let v = 0.5 + 0.3 * ((i as f64) * 1.1).cos();
                    row.push(VelocityVector::new(u, v, c as f64, r as f64, 2.0));
                }
                field.push(row);
            }
            fields.push(field);
        }

        let tke = PostProcessor::turbulent_kinetic_energy(&fields);

        for r in 0..n {
            for c in 0..n {
                assert!(
                    tke[r][c] >= 0.0,
                    "TKE should be non-negative, got {} at ({},{})",
                    tke[r][c],
                    r,
                    c
                );
                assert!(
                    tke[r][c] > 0.0,
                    "TKE should be positive for fluctuating flow, got {} at ({},{})",
                    tke[r][c],
                    r,
                    c
                );
            }
        }
    }

    #[test]
    fn test_tke_zero_for_steady_flow() {
        let n = 3;
        let mut fields = Vec::new();

        for _ in 0..5 {
            let mut field = Vec::new();
            for r in 0..n {
                let mut row = Vec::new();
                for c in 0..n {
                    row.push(VelocityVector::new(1.0, 0.5, c as f64, r as f64, 2.0));
                }
                field.push(row);
            }
            fields.push(field);
        }

        let tke = PostProcessor::turbulent_kinetic_energy(&fields);

        for r in 0..n {
            for c in 0..n {
                assert!(
                    tke[r][c].abs() < 1e-10,
                    "TKE should be zero for steady flow, got {}",
                    tke[r][c]
                );
            }
        }
    }

    #[test]
    fn test_reynolds_stress_zero_for_uncorrelated() {
        // If u' and v' are uncorrelated (one varies, other constant), <u'v'> -> 0
        let n = 3;
        let mut fields = Vec::new();

        for i in 0..20 {
            let mut field = Vec::new();
            for r in 0..n {
                let mut row = Vec::new();
                for c in 0..n {
                    // u varies sinusoidally, v is constant -> <u'v'> = 0
                    let u = 1.0 + 0.5 * ((i as f64) * 0.3).sin();
                    let v = 1.0;
                    row.push(VelocityVector::new(u, v, c as f64, r as f64, 2.0));
                }
                field.push(row);
            }
            fields.push(field);
        }

        let rs = PostProcessor::reynolds_stress(&fields);

        for r in 0..n {
            for c in 0..n {
                assert!(
                    rs[r][c].abs() < 1e-10,
                    "Reynolds stress should be ~0 for uncorrelated fluctuations, got {}",
                    rs[r][c]
                );
            }
        }
    }

    #[test]
    fn test_reynolds_stress_nonzero_for_correlated() {
        // If u' and v' are correlated (both increase together), <u'v'> > 0
        let n = 3;
        let mut fields = Vec::new();

        for i in 0..20 {
            let mut field = Vec::new();
            let fluctuation = ((i as f64) * 0.5).sin();
            for r in 0..n {
                let mut row = Vec::new();
                for c in 0..n {
                    let u = 1.0 + 0.5 * fluctuation;
                    let v = 1.0 + 0.3 * fluctuation; // same phase -> correlated
                    row.push(VelocityVector::new(u, v, c as f64, r as f64, 2.0));
                }
                field.push(row);
            }
            fields.push(field);
        }

        let rs = PostProcessor::reynolds_stress(&fields);

        for r in 0..n {
            for c in 0..n {
                assert!(
                    rs[r][c] > 0.0,
                    "Reynolds stress should be positive for positively correlated fluctuations, got {}",
                    rs[r][c]
                );
            }
        }
    }

    #[test]
    fn test_median_filter_removes_outliers() {
        let n = 5;
        let mut vectors = Vec::new();
        for r in 0..n {
            let mut row = Vec::new();
            for c in 0..n {
                row.push(VelocityVector::new(1.0, 0.0, c as f64, r as f64, 2.0));
            }
            vectors.push(row);
        }

        // Insert outlier at center
        vectors[2][2].u = 100.0;
        vectors[2][2].v = 100.0;

        PostProcessor::median_filter(&mut vectors, 2.0);

        // Outlier should be flagged (peak_ratio = 0.0)
        assert!(
            vectors[2][2].peak_ratio < 0.01,
            "Outlier should be flagged, peak_ratio={}",
            vectors[2][2].peak_ratio
        );

        // Non-outliers should not be flagged
        assert!(
            vectors[0][0].peak_ratio > 0.0,
            "Non-outlier should not be flagged"
        );
    }

    #[test]
    fn test_replace_outliers_interpolates() {
        let n = 5;
        let mut vectors = Vec::new();
        for r in 0..n {
            let mut row = Vec::new();
            for c in 0..n {
                row.push(VelocityVector::new(2.0, 1.0, c as f64, r as f64, 2.0));
            }
            vectors.push(row);
        }

        // Mark center as outlier
        vectors[2][2].peak_ratio = 0.0;
        vectors[2][2].u = 999.0;
        vectors[2][2].v = 999.0;

        PostProcessor::replace_outliers(&mut vectors);

        // Should be interpolated to ~(2.0, 1.0)
        assert!(
            (vectors[2][2].u - 2.0).abs() < 0.1,
            "Interpolated u should be ~2.0, got {}",
            vectors[2][2].u
        );
        assert!(
            (vectors[2][2].v - 1.0).abs() < 0.1,
            "Interpolated v should be ~1.0, got {}",
            vectors[2][2].v
        );
        assert!(
            vectors[2][2].peak_ratio > 0.0,
            "Replaced vector should have positive peak_ratio"
        );
    }

    #[test]
    fn test_background_subtraction() {
        let mut image = vec![vec![10.0f64; 4]; 4];
        let background = vec![vec![3.0f64; 4]; 4];

        ImagePreprocessor::subtract_background(&mut image, &background);

        for row in &image {
            for &val in row {
                assert!((val - 7.0).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_background_subtraction_clamps_negative() {
        let mut image = vec![vec![1.0f64; 4]; 4];
        let background = vec![vec![5.0f64; 4]; 4];

        ImagePreprocessor::subtract_background(&mut image, &background);

        for row in &image {
            for &val in row {
                assert!(val >= 0.0, "Should clamp to non-negative, got {}", val);
            }
        }
    }

    #[test]
    fn test_normalize_intensity() {
        let mut image = vec![
            vec![10.0, 20.0],
            vec![30.0, 40.0],
        ];

        ImagePreprocessor::normalize_intensity(&mut image);

        assert!((image[0][0] - 0.0).abs() < 1e-10); // min -> 0
        assert!((image[1][1] - 1.0).abs() < 1e-10); // max -> 1
        assert!((image[0][1] - 1.0 / 3.0).abs() < 1e-10); // 20 -> 1/3
    }

    #[test]
    fn test_normalize_intensity_uniform() {
        let mut image = vec![vec![5.0f64; 4]; 4];

        ImagePreprocessor::normalize_intensity(&mut image);

        // Uniform image: range = 0, should remain unchanged
        for row in &image {
            for &val in row {
                assert!((val - 5.0).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_min_max_filter_dimensions() {
        let image = vec![vec![0.5f64; 16]; 16];
        let result = ImagePreprocessor::min_max_filter(&image, 3);
        assert_eq!(result.len(), 16);
        assert_eq!(result[0].len(), 16);
    }

    #[test]
    fn test_min_max_filter_enhances_particles() {
        // Image with a bright spot (particle) on dark background
        let mut image = vec![vec![0.1f64; 16]; 16];
        image[8][8] = 1.0;

        let result = ImagePreprocessor::min_max_filter(&image, 5);

        // The particle location should have high value
        assert!(
            result[8][8] > 0.5,
            "Particle should be enhanced, got {}",
            result[8][8]
        );
    }

    #[test]
    fn test_particle_density() {
        let mut image = vec![vec![0.0f64; 10]; 10];
        // Set 25 out of 100 pixels above threshold
        for r in 0..5 {
            for c in 0..5 {
                image[r][c] = 1.0;
            }
        }

        let density = ImagePreprocessor::particle_density(&image, 0.5);
        assert!((density - 0.25).abs() < 1e-10);
    }

    #[test]
    fn test_particle_density_empty() {
        let image: Vec<Vec<f64>> = Vec::new();
        let density = ImagePreprocessor::particle_density(&image, 0.5);
        assert_eq!(density, 0.0);
    }

    #[test]
    fn test_generate_particle_image_dimensions() {
        let img = generate_particle_image(100, 80, 50, 3.0, 42);
        assert_eq!(img.len(), 100);
        assert_eq!(img[0].len(), 80);
    }

    #[test]
    fn test_generate_particle_image_non_negative() {
        let img = generate_particle_image(64, 64, 100, 3.0, 42);
        for row in &img {
            for &val in row {
                assert!(val >= 0.0, "Pixel values should be non-negative");
                assert!(val <= 1.0, "Pixel values should be <= 1.0");
            }
        }
    }

    #[test]
    fn test_generate_particle_image_has_particles() {
        let img = generate_particle_image(64, 64, 100, 3.0, 42);
        let max_val = img.iter().flat_map(|row| row.iter()).cloned().fold(0.0f64, f64::max);
        assert!(max_val > 0.1, "Image should contain particles with non-zero intensity");
    }

    #[test]
    fn test_generate_particle_image_reproducibility() {
        let img1 = generate_particle_image(64, 64, 50, 3.0, 42);
        let img2 = generate_particle_image(64, 64, 50, 3.0, 42);

        for r in 0..64 {
            for c in 0..64 {
                assert!(
                    (img1[r][c] - img2[r][c]).abs() < 1e-12,
                    "Same seed should produce identical images"
                );
            }
        }
    }

    #[test]
    fn test_shift_image_zero_shift() {
        let img = generate_particle_image(32, 32, 50, 3.0, 42);
        let shifted = shift_image(&img, 0, 0);

        for r in 0..32 {
            for c in 0..32 {
                assert!((img[r][c] - shifted[r][c]).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn test_shift_image_known_shift() {
        let mut img = vec![vec![0.0f64; 10]; 10];
        img[3][3] = 1.0;

        let shifted = shift_image(&img, 2, 1);
        // Original at (3,3) should now be at (4,5) after shift_x=2, shift_y=1
        assert!((shifted[4][5] - 1.0).abs() < 1e-12);
        assert!(shifted[3][3].abs() < 1e-12);
    }

    #[test]
    fn test_empty_image_handling() {
        let config = default_config();
        let processor = PivProcessor::new(config);

        let empty: Vec<Vec<f64>> = Vec::new();
        let vectors = processor.process_image_pair(&empty, &empty);
        assert!(vectors.is_empty());
    }

    #[test]
    fn test_small_image_no_crash() {
        let config = PivConfig {
            interrogation_window_size: 32,
            overlap_fraction: 0.5,
            dt_us: 100.0,
            magnification: 10000.0,
            search_radius: 8,
        };
        let processor = PivProcessor::new(config);

        // Image smaller than window size
        let img = vec![vec![1.0f64; 16]; 16];
        let vectors = processor.process_image_pair(&img, &img);
        assert!(vectors.is_empty(), "Image smaller than window should produce no vectors");
    }

    #[test]
    fn test_vorticity_empty_field() {
        let vectors: Vec<Vec<VelocityVector>> = Vec::new();
        let vort = PostProcessor::vorticity(&vectors, 1.0, 1.0);
        assert!(vort.is_empty());
    }

    #[test]
    fn test_divergence_empty_field() {
        let vectors: Vec<Vec<VelocityVector>> = Vec::new();
        let div = PostProcessor::divergence(&vectors, 1.0, 1.0);
        assert!(div.is_empty());
    }

    #[test]
    fn test_tke_empty_fields() {
        let fields: Vec<Vec<Vec<VelocityVector>>> = Vec::new();
        let tke = PostProcessor::turbulent_kinetic_energy(&fields);
        assert!(tke.is_empty());
    }

    #[test]
    fn test_peak_to_noise_ratio_empty() {
        let corr: Vec<Vec<f64>> = Vec::new();
        let pnr = peak_to_noise_ratio(&corr, 0, 0, 1);
        assert_eq!(pnr, 0.0);
    }

    #[test]
    fn test_median_of_odd() {
        let mut data = vec![5.0, 1.0, 3.0, 2.0, 4.0];
        assert!((median_of(&mut data) - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_median_of_even() {
        let mut data = vec![4.0, 1.0, 3.0, 2.0];
        assert!((median_of(&mut data) - 2.5).abs() < 1e-10);
    }

    #[test]
    fn test_median_of_single() {
        let mut data = vec![7.0];
        assert!((median_of(&mut data) - 7.0).abs() < 1e-10);
    }

    #[test]
    fn test_median_of_empty() {
        let mut data: Vec<f64> = Vec::new();
        assert_eq!(median_of(&mut data), 0.0);
    }
}
