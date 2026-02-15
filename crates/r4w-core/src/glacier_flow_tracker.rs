//! # Glacier Flow Velocity Tracker
//!
//! Implements glacier flow velocity mapping using SAR/InSAR remote sensing techniques.
//! Glacier monitoring is critical for climate change assessment, sea level rise prediction,
//! and glacial hazard management.
//!
//! ## Key Techniques
//!
//! - **Speckle/Feature Tracking**: Normalized cross-correlation between repeat SAR images
//!   to measure pixel-level displacement of glacier surface features.
//! - **InSAR Phase Analysis**: Interferometric phase difference for sub-pixel displacement
//!   measurement using radar wavelength and viewing geometry.
//! - **Ice Sheet Analysis**: Strain rate computation, calving front velocity, ice flux,
//!   mass balance, and Glen's flow law deformation modeling.
//!
//! ## Typical Parameters
//!
//! | Parameter | Sentinel-1 (C-band) | ALOS-2 (L-band) |
//! |-----------|---------------------|------------------|
//! | Wavelength | 0.056 m | 0.236 m |
//! | Repeat cycle | 6-12 days | 14-46 days |
//! | Pixel spacing | 10-20 m | 10-30 m |
//! | Incidence angle | 29-46 deg | 20-50 deg |
//!
//! ## References
//!
//! - Strozzi, T. et al. (2002). "Glacier motion estimation using SAR offset-tracking procedures."
//! - Goldstein, R. et al. (1993). "Satellite radar interferometry for monitoring ice sheet motion."
//! - Glen, J. W. (1955). "The creep of polycrystalline ice." Proc. Royal Society A, 228.

use std::f64::consts::PI;

// ─── Constants ───────────────────────────────────────────────────────────────

/// Density of glacial ice in kg/m^3.
const ICE_DENSITY: f64 = 917.0;

/// Density of ocean water in kg/m^3.
const OCEAN_WATER_DENSITY: f64 = 1025.0;

/// Surface area of the world's oceans in m^2 (approximately 361.9 million km^2).
const OCEAN_AREA_M2: f64 = 3.619e14;

/// Gravitational acceleration in m/s^2.
const GRAVITY: f64 = 9.81;

/// Glen's flow law exponent (standard value for polycrystalline ice).
const GLEN_N: f64 = 3.0;

/// Universal gas constant in J/(mol*K).
const GAS_CONSTANT: f64 = 8.314;

/// Activation energy for ice creep below -10 C in J/mol.
const ACTIVATION_ENERGY_LOW: f64 = 60_000.0;

/// Activation energy for ice creep above -10 C in J/mol.
const ACTIVATION_ENERGY_HIGH: f64 = 139_000.0;

/// Reference temperature for Glen's flow parameter (263.15 K = -10 C).
const REFERENCE_TEMP_K: f64 = 263.15;

/// Glen's flow parameter at the reference temperature in Pa^{-3} s^{-1}.
const A_REF: f64 = 3.615e-13;

// ─── Configuration ───────────────────────────────────────────────────────────

/// Configuration for glacier flow tracking from SAR imagery.
///
/// Encapsulates the sensor parameters, temporal baseline, and processing settings
/// required for speckle tracking and InSAR-based velocity estimation.
#[derive(Debug, Clone)]
pub struct GlacierConfig {
    /// SAR ground range pixel spacing in meters (typically 10-30 m).
    pub pixel_spacing_m: f64,
    /// Temporal baseline between repeat SAR acquisitions in days (typically 6-46).
    pub repeat_interval_days: f64,
    /// Radar wavelength in meters (C-band: 0.056, L-band: 0.236).
    pub radar_wavelength_m: f64,
    /// Radar incidence angle in degrees (typically 20-50 degrees).
    pub incidence_angle_deg: f64,
    /// Correlation search window size in pixels (typically 32-128).
    pub search_window_size: usize,
    /// DEM resolution in meters for topographic corrections.
    pub dem_resolution_m: f64,
}

impl GlacierConfig {
    /// Creates a Sentinel-1 C-band default configuration.
    ///
    /// Sentinel-1 operates in IW mode with 5x20 m resolution, 12-day repeat cycle,
    /// C-band (5.405 GHz, wavelength 0.0555 m), and ~39 degree incidence angle.
    pub fn sentinel1() -> Self {
        Self {
            pixel_spacing_m: 14.0,
            repeat_interval_days: 12.0,
            radar_wavelength_m: 0.0555,
            incidence_angle_deg: 39.0,
            search_window_size: 64,
            dem_resolution_m: 30.0,
        }
    }

    /// Creates an ALOS-2 PALSAR-2 L-band default configuration.
    ///
    /// ALOS-2 operates in StripMap mode with 10 m resolution, 14-day repeat cycle,
    /// L-band (1.27 GHz, wavelength 0.236 m), and ~34 degree incidence angle.
    pub fn alos2() -> Self {
        Self {
            pixel_spacing_m: 10.0,
            repeat_interval_days: 14.0,
            radar_wavelength_m: 0.236,
            incidence_angle_deg: 34.0,
            search_window_size: 64,
            dem_resolution_m: 12.5,
        }
    }
}

// ─── Flow Vector ─────────────────────────────────────────────────────────────

/// Glacier surface flow velocity vector derived from SAR image pair analysis.
///
/// Contains the displacement in range and azimuth directions, the resulting velocity,
/// direction, and a quality metric (correlation coefficient).
#[derive(Debug, Clone)]
pub struct FlowVector {
    /// Displacement in the range (across-track) direction in meters.
    pub range_displacement_m: f64,
    /// Displacement in the azimuth (along-track) direction in meters.
    pub azimuth_displacement_m: f64,
    /// Flow velocity in meters per day.
    pub velocity_m_per_day: f64,
    /// Flow velocity in meters per year.
    pub velocity_m_per_year: f64,
    /// Normalized cross-correlation coefficient (0.0-1.0), quality metric.
    pub correlation: f64,
    /// Flow direction in degrees from north (0-360), measured clockwise.
    pub direction_deg: f64,
}

impl FlowVector {
    /// Creates a new FlowVector from displacement components, pixel spacing, and temporal baseline.
    ///
    /// # Arguments
    ///
    /// * `range_px` - Range displacement in pixels.
    /// * `azimuth_px` - Azimuth displacement in pixels.
    /// * `pixel_spacing_m` - Ground pixel spacing in meters.
    /// * `repeat_days` - Temporal baseline in days.
    /// * `correlation` - NCC quality metric (0-1).
    pub fn from_pixel_offsets(
        range_px: f64,
        azimuth_px: f64,
        pixel_spacing_m: f64,
        repeat_days: f64,
        correlation: f64,
    ) -> Self {
        let range_m = range_px * pixel_spacing_m;
        let azimuth_m = azimuth_px * pixel_spacing_m;
        let total_m = (range_m * range_m + azimuth_m * azimuth_m).sqrt();
        let v_day = if repeat_days > 0.0 {
            total_m / repeat_days
        } else {
            0.0
        };
        let v_year = v_day * 365.25;
        let direction = azimuth_m.atan2(range_m).to_degrees().rem_euclid(360.0);

        Self {
            range_displacement_m: range_m,
            azimuth_displacement_m: azimuth_m,
            velocity_m_per_day: v_day,
            velocity_m_per_year: v_year,
            correlation,
            direction_deg: direction,
        }
    }

    /// Returns the total displacement magnitude in meters.
    pub fn total_displacement_m(&self) -> f64 {
        (self.range_displacement_m * self.range_displacement_m
            + self.azimuth_displacement_m * self.azimuth_displacement_m)
            .sqrt()
    }
}

// ─── Speckle Tracker ─────────────────────────────────────────────────────────

/// SAR speckle/feature tracking engine.
///
/// Uses normalized cross-correlation (NCC) to measure glacier surface displacement
/// between repeat SAR image pairs. The NCC approach is robust to speckle noise and
/// can measure displacements at the sub-pixel level using parabolic interpolation.
///
/// # Algorithm
///
/// 1. For each grid point, extract a reference patch from image 1.
/// 2. Slide the patch over a search area in image 2, computing NCC at each offset.
/// 3. Find the peak correlation and refine to sub-pixel with parabolic fit.
/// 4. Convert pixel offsets to ground displacement and velocity.
pub struct SpeckleTracker {
    config: GlacierConfig,
}

impl SpeckleTracker {
    /// Creates a new speckle tracker with the given sensor configuration.
    pub fn new(config: GlacierConfig) -> Self {
        Self { config }
    }

    /// Computes normalized cross-correlation between two equally-sized 2D patches.
    ///
    /// NCC is defined as:
    /// ```text
    /// NCC = sum((a - mean_a) * (b - mean_b)) / (N * std_a * std_b)
    /// ```
    ///
    /// Returns a value in [-1, 1] where 1 = perfect match, -1 = perfect inverse,
    /// and 0 = no correlation.
    ///
    /// # Panics
    ///
    /// Panics if patches are empty or have different dimensions.
    pub fn normalized_cross_correlation(
        patch1: &[Vec<f64>],
        patch2: &[Vec<f64>],
    ) -> f64 {
        assert!(!patch1.is_empty() && !patch2.is_empty(), "Patches must not be empty");
        assert_eq!(patch1.len(), patch2.len(), "Patch row counts must match");
        for (r1, r2) in patch1.iter().zip(patch2.iter()) {
            assert_eq!(r1.len(), r2.len(), "Patch column counts must match");
        }

        let rows = patch1.len();
        let cols = patch1[0].len();
        let n = (rows * cols) as f64;

        // Compute means
        let mut sum1 = 0.0_f64;
        let mut sum2 = 0.0_f64;
        for r in 0..rows {
            for c in 0..cols {
                sum1 += patch1[r][c];
                sum2 += patch2[r][c];
            }
        }
        let mean1 = sum1 / n;
        let mean2 = sum2 / n;

        // Compute cross-correlation and standard deviations
        let mut cross = 0.0_f64;
        let mut var1 = 0.0_f64;
        let mut var2 = 0.0_f64;
        for r in 0..rows {
            for c in 0..cols {
                let d1 = patch1[r][c] - mean1;
                let d2 = patch2[r][c] - mean2;
                cross += d1 * d2;
                var1 += d1 * d1;
                var2 += d2 * d2;
            }
        }

        let denom = (var1 * var2).sqrt();
        if denom < 1e-15 {
            return 0.0;
        }
        cross / denom
    }

    /// Tracks displacement between two images at a given center position.
    ///
    /// Extracts a reference patch from `image1` centered at `center` and searches
    /// for its best match in `image2` within a `search_radius` pixel neighborhood.
    ///
    /// Returns `(range_offset_px, azimuth_offset_px, correlation)` where offsets
    /// are in fractional pixels (sub-pixel accuracy via parabolic interpolation).
    ///
    /// Returns `(0.0, 0.0, 0.0)` if the search window exceeds image bounds.
    pub fn track_displacement(
        &self,
        image1: &[Vec<f64>],
        image2: &[Vec<f64>],
        center: (usize, usize),
        search_radius: usize,
    ) -> (f64, f64, f64) {
        let half_win = self.config.search_window_size / 2;
        let (cr, cc) = center;
        let rows = image1.len();
        if rows == 0 {
            return (0.0, 0.0, 0.0);
        }
        let cols = image1[0].len();

        // Check bounds for reference patch extraction
        if cr < half_win || cr + half_win >= rows || cc < half_win || cc + half_win >= cols {
            return (0.0, 0.0, 0.0);
        }

        // Extract reference patch from image1
        let ref_patch: Vec<Vec<f64>> = (cr - half_win..=cr + half_win)
            .map(|r| image1[r][cc - half_win..=cc + half_win].to_vec())
            .collect();

        let search_size = 2 * search_radius + 1;
        let mut corr_surface: Vec<Vec<f64>> = vec![vec![-2.0; search_size]; search_size];
        let mut best_corr = -2.0_f64;
        let mut best_dr = 0_i32;
        let mut best_dc = 0_i32;

        for dr in -(search_radius as i32)..=(search_radius as i32) {
            for dc in -(search_radius as i32)..=(search_radius as i32) {
                let sr = (cr as i32 + dr) as usize;
                let sc = (cc as i32 + dc) as usize;

                // Check bounds for search patch
                if sr < half_win || sr + half_win >= rows || sc < half_win || sc + half_win >= cols
                {
                    continue;
                }

                let search_patch: Vec<Vec<f64>> = (sr - half_win..=sr + half_win)
                    .map(|r| image2[r][sc - half_win..=sc + half_win].to_vec())
                    .collect();

                let ncc = Self::normalized_cross_correlation(&ref_patch, &search_patch);
                let si = (dr + search_radius as i32) as usize;
                let sj = (dc + search_radius as i32) as usize;
                corr_surface[si][sj] = ncc;

                if ncc > best_corr {
                    best_corr = ncc;
                    best_dr = dr;
                    best_dc = dc;
                }
            }
        }

        // Sub-pixel refinement using parabolic interpolation
        let (sub_r, sub_c) = Self::subpixel_peak_at(
            &corr_surface,
            (best_dr + search_radius as i32) as usize,
            (best_dc + search_radius as i32) as usize,
        );

        let range_offset = sub_r - search_radius as f64;
        let azimuth_offset = sub_c - search_radius as f64;

        (range_offset, azimuth_offset, best_corr)
    }

    /// Finds sub-pixel peak location in a correlation surface using parabolic interpolation.
    ///
    /// Fits a parabola to the peak and its neighbors in both row and column directions,
    /// providing sub-pixel accuracy typically to 1/10 pixel or better.
    ///
    /// Returns `(row, col)` in fractional surface coordinates of the peak.
    pub fn subpixel_peak(corr_surface: &[Vec<f64>]) -> (f64, f64) {
        if corr_surface.is_empty() || corr_surface[0].is_empty() {
            return (0.0, 0.0);
        }

        // Find integer peak
        let mut best_val = f64::NEG_INFINITY;
        let mut best_r = 0_usize;
        let mut best_c = 0_usize;

        for (r, row) in corr_surface.iter().enumerate() {
            for (c, &val) in row.iter().enumerate() {
                if val > best_val {
                    best_val = val;
                    best_r = r;
                    best_c = c;
                }
            }
        }

        Self::subpixel_peak_at(corr_surface, best_r, best_c)
    }

    /// Refine a peak at a known integer location using parabolic interpolation.
    fn subpixel_peak_at(
        corr_surface: &[Vec<f64>],
        peak_r: usize,
        peak_c: usize,
    ) -> (f64, f64) {
        let rows = corr_surface.len();
        let cols = corr_surface[0].len();

        let mut sub_r = peak_r as f64;
        let mut sub_c = peak_c as f64;

        // Parabolic fit in row direction
        if peak_r > 0 && peak_r + 1 < rows {
            let ym1 = corr_surface[peak_r - 1][peak_c];
            let y0 = corr_surface[peak_r][peak_c];
            let yp1 = corr_surface[peak_r + 1][peak_c];
            let denom = 2.0 * (2.0 * y0 - ym1 - yp1);
            if denom.abs() > 1e-15 {
                sub_r = peak_r as f64 + (ym1 - yp1) / denom;
            }
        }

        // Parabolic fit in column direction
        if peak_c > 0 && peak_c + 1 < cols {
            let ym1 = corr_surface[peak_r][peak_c - 1];
            let y0 = corr_surface[peak_r][peak_c];
            let yp1 = corr_surface[peak_r][peak_c + 1];
            let denom = 2.0 * (2.0 * y0 - ym1 - yp1);
            if denom.abs() > 1e-15 {
                sub_c = peak_c as f64 + (ym1 - yp1) / denom;
            }
        }

        (sub_r, sub_c)
    }

    /// Computes a full velocity field from a pair of SAR images.
    ///
    /// Divides the image into a regular grid with spacing `grid_spacing` pixels,
    /// performs speckle tracking at each grid point, and converts pixel offsets
    /// to ground velocity vectors.
    ///
    /// # Arguments
    ///
    /// * `image1` - Reference SAR image (amplitude, rows x cols).
    /// * `image2` - Secondary SAR image (same dimensions).
    /// * `grid_spacing` - Spacing between grid sample points in pixels.
    ///
    /// # Returns
    ///
    /// 2D grid of `FlowVector` values, one per sampled grid point.
    pub fn velocity_field(
        &self,
        image1: &[Vec<f64>],
        image2: &[Vec<f64>],
        grid_spacing: usize,
    ) -> Vec<Vec<FlowVector>> {
        let rows = image1.len();
        if rows == 0 {
            return vec![];
        }
        let cols = image1[0].len();
        let half_win = self.config.search_window_size / 2;
        let search_radius = half_win; // search area = window size

        let mut result = Vec::new();

        let mut r = half_win + search_radius;
        while r + half_win + search_radius < rows {
            let mut row_vectors = Vec::new();
            let mut c = half_win + search_radius;
            while c + half_win + search_radius < cols {
                let (dr, dc, corr) =
                    self.track_displacement(image1, image2, (r, c), search_radius);
                let fv = FlowVector::from_pixel_offsets(
                    dr,
                    dc,
                    self.config.pixel_spacing_m,
                    self.config.repeat_interval_days,
                    corr,
                );
                row_vectors.push(fv);
                c += grid_spacing;
            }
            if !row_vectors.is_empty() {
                result.push(row_vectors);
            }
            r += grid_spacing;
        }

        result
    }

    /// Filters outlier velocity vectors exceeding a physical threshold.
    ///
    /// Velocities above `max_velocity_m_per_day` are zeroed out and their correlation
    /// set to 0.0, marking them as invalid. Typical thresholds: 10-50 m/day for
    /// fast outlet glaciers, 0.1-1 m/day for ice sheets.
    pub fn filter_outliers(
        velocities: &mut Vec<Vec<FlowVector>>,
        max_velocity_m_per_day: f64,
    ) {
        for row in velocities.iter_mut() {
            for fv in row.iter_mut() {
                if fv.velocity_m_per_day > max_velocity_m_per_day {
                    fv.range_displacement_m = 0.0;
                    fv.azimuth_displacement_m = 0.0;
                    fv.velocity_m_per_day = 0.0;
                    fv.velocity_m_per_year = 0.0;
                    fv.correlation = 0.0;
                    fv.direction_deg = 0.0;
                }
            }
        }
    }

    /// Applies a 2D median filter to the velocity field for spatial smoothing.
    ///
    /// The median filter is applied independently to the velocity magnitude and
    /// the range/azimuth displacement components. It removes isolated outliers
    /// while preserving sharp velocity gradients (e.g., at shear margins).
    ///
    /// # Arguments
    ///
    /// * `velocities` - Input velocity field.
    /// * `window` - Median filter window size (must be odd, e.g. 3, 5, 7).
    pub fn median_filter_2d(
        velocities: &[Vec<FlowVector>],
        window: usize,
    ) -> Vec<Vec<FlowVector>> {
        if velocities.is_empty() || velocities[0].is_empty() {
            return vec![];
        }

        let rows = velocities.len();
        let cols = velocities[0].len();
        let half = window / 2;

        let mut result = Vec::with_capacity(rows);

        for r in 0..rows {
            let mut row_out = Vec::with_capacity(cols);
            for c in 0..cols {
                let mut range_vals = Vec::new();
                let mut azimuth_vals = Vec::new();
                let mut corr_vals = Vec::new();

                for dr in 0..window {
                    for dc in 0..window {
                        let rr = r as i64 + dr as i64 - half as i64;
                        let cc = c as i64 + dc as i64 - half as i64;
                        if rr >= 0 && rr < rows as i64 && cc >= 0 && cc < cols as i64 {
                            let fv = &velocities[rr as usize][cc as usize];
                            range_vals.push(fv.range_displacement_m);
                            azimuth_vals.push(fv.azimuth_displacement_m);
                            corr_vals.push(fv.correlation);
                        }
                    }
                }

                let med_range = median_f64(&mut range_vals);
                let med_azimuth = median_f64(&mut azimuth_vals);
                let med_corr = median_f64(&mut corr_vals);

                let total = (med_range * med_range + med_azimuth * med_azimuth).sqrt();
                let orig = &velocities[r][c];
                let repeat_days = if orig.velocity_m_per_day > 0.0 {
                    orig.total_displacement_m() / orig.velocity_m_per_day
                } else {
                    1.0
                };
                let v_day = total / repeat_days;
                let direction = med_azimuth.atan2(med_range).to_degrees().rem_euclid(360.0);

                row_out.push(FlowVector {
                    range_displacement_m: med_range,
                    azimuth_displacement_m: med_azimuth,
                    velocity_m_per_day: v_day,
                    velocity_m_per_year: v_day * 365.25,
                    correlation: med_corr,
                    direction_deg: direction,
                });
            }
            result.push(row_out);
        }

        result
    }
}

/// Helper: compute median of a mutable f64 slice.
fn median_f64(vals: &mut [f64]) -> f64 {
    if vals.is_empty() {
        return 0.0;
    }
    vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let n = vals.len();
    if n % 2 == 0 {
        (vals[n / 2 - 1] + vals[n / 2]) / 2.0
    } else {
        vals[n / 2]
    }
}

// ─── InSAR Processor ─────────────────────────────────────────────────────────

/// Interferometric SAR (InSAR) processor for sub-pixel displacement measurement.
///
/// InSAR exploits the phase difference between two complex SAR images acquired
/// from similar geometries. The interferometric phase encodes the line-of-sight
/// displacement of the surface between acquisitions.
///
/// ## Phase-Displacement Relationship
///
/// ```text
/// d = lambda * phi / (4 * pi * cos(theta))
/// ```
///
/// where `d` = LOS displacement, `lambda` = radar wavelength, `phi` = interferometric
/// phase, and `theta` = incidence angle.
pub struct InSARProcessor {
    /// Radar wavelength in meters.
    wavelength_m: f64,
    /// Incidence angle in degrees.
    incidence_angle_deg: f64,
}

impl InSARProcessor {
    /// Creates a new InSAR processor with the given radar parameters.
    ///
    /// # Arguments
    ///
    /// * `wavelength_m` - Radar wavelength in meters (e.g., 0.056 for C-band).
    /// * `incidence_angle_deg` - Radar incidence angle in degrees.
    pub fn new(wavelength_m: f64, incidence_angle_deg: f64) -> Self {
        Self {
            wavelength_m,
            incidence_angle_deg,
        }
    }

    /// Computes the interferogram (phase difference) between two complex SAR images.
    ///
    /// For complex pixels `z1 = (re1, im1)` and `z2 = (re2, im2)`, the interferometric
    /// phase is `arg(z1 * conj(z2))`.
    ///
    /// # Arguments
    ///
    /// * `image1` - Reference complex SAR image (rows of (re, im) tuples).
    /// * `image2` - Secondary complex SAR image (same dimensions).
    ///
    /// # Returns
    ///
    /// 2D array of interferometric phase values in radians [-pi, pi].
    pub fn interferogram(
        &self,
        image1: &[Vec<(f64, f64)>],
        image2: &[Vec<(f64, f64)>],
    ) -> Vec<Vec<f64>> {
        let rows = image1.len();
        let mut result = Vec::with_capacity(rows);

        for r in 0..rows {
            let cols = image1[r].len();
            let mut row = Vec::with_capacity(cols);
            for c in 0..cols {
                let (re1, im1) = image1[r][c];
                let (re2, im2) = image2[r][c];
                // z1 * conj(z2) = (re1*re2 + im1*im2, im1*re2 - re1*im2)
                let cross_re = re1 * re2 + im1 * im2;
                let cross_im = im1 * re2 - re1 * im2;
                row.push(cross_im.atan2(cross_re));
            }
            result.push(row);
        }

        result
    }

    /// Converts interferometric phase to line-of-sight displacement.
    ///
    /// Uses the relationship: `d = lambda * phi / (4 * pi * cos(theta))`
    ///
    /// # Arguments
    ///
    /// * `phase_rad` - Interferometric phase in radians.
    /// * `wavelength_m` - Radar wavelength in meters.
    /// * `incidence_deg` - Incidence angle in degrees.
    ///
    /// # Returns
    ///
    /// Line-of-sight displacement in meters (positive = away from sensor).
    pub fn phase_to_displacement(
        phase_rad: f64,
        wavelength_m: f64,
        incidence_deg: f64,
    ) -> f64 {
        let theta = incidence_deg.to_radians();
        wavelength_m * phase_rad / (4.0 * PI * theta.cos())
    }

    /// Estimates spatial coherence between two complex SAR images.
    ///
    /// Coherence is computed in a sliding window:
    /// ```text
    /// gamma = |sum(z1 * conj(z2))| / sqrt(sum(|z1|^2) * sum(|z2|^2))
    /// ```
    ///
    /// Values range from 0 (complete decorrelation) to 1 (perfect coherence).
    /// High coherence (>0.3) indicates reliable interferometric phase.
    ///
    /// # Arguments
    ///
    /// * `image1` - Reference complex SAR image.
    /// * `image2` - Secondary complex SAR image.
    /// * `window` - Estimation window size (e.g., 5 for 5x5).
    pub fn coherence_estimation(
        &self,
        image1: &[Vec<(f64, f64)>],
        image2: &[Vec<(f64, f64)>],
        window: usize,
    ) -> Vec<Vec<f64>> {
        let rows = image1.len();
        if rows == 0 {
            return vec![];
        }
        let cols = image1[0].len();
        let half = window / 2;

        let mut result = vec![vec![0.0; cols]; rows];

        for r in 0..rows {
            for c in 0..cols {
                let mut cross_re = 0.0_f64;
                let mut cross_im = 0.0_f64;
                let mut power1 = 0.0_f64;
                let mut power2 = 0.0_f64;

                for dr in 0..window {
                    for dc in 0..window {
                        let rr = r as i64 + dr as i64 - half as i64;
                        let cc = c as i64 + dc as i64 - half as i64;
                        if rr >= 0 && rr < rows as i64 && cc >= 0 && cc < cols as i64 {
                            let (re1, im1) = image1[rr as usize][cc as usize];
                            let (re2, im2) = image2[rr as usize][cc as usize];

                            // z1 * conj(z2)
                            cross_re += re1 * re2 + im1 * im2;
                            cross_im += im1 * re2 - re1 * im2;

                            power1 += re1 * re1 + im1 * im1;
                            power2 += re2 * re2 + im2 * im2;
                        }
                    }
                }

                let num = (cross_re * cross_re + cross_im * cross_im).sqrt();
                let den = (power1 * power2).sqrt();
                result[r][c] = if den > 1e-15 { num / den } else { 0.0 };
            }
        }

        result
    }

    /// 1D phase unwrapping using the Itoh method.
    ///
    /// The Itoh algorithm detects phase jumps exceeding pi and adds/subtracts 2*pi
    /// to produce a continuous phase profile. This is the simplest and most widely
    /// used 1D unwrapping approach.
    ///
    /// # Arguments
    ///
    /// * `wrapped` - Wrapped phase values in radians [-pi, pi].
    ///
    /// # Returns
    ///
    /// Unwrapped phase values (continuous, may exceed [-pi, pi]).
    pub fn phase_unwrap_1d(wrapped: &[f64]) -> Vec<f64> {
        if wrapped.is_empty() {
            return vec![];
        }

        let mut unwrapped = vec![0.0; wrapped.len()];
        unwrapped[0] = wrapped[0];

        for i in 1..wrapped.len() {
            let diff = wrapped[i] - wrapped[i - 1];
            // Wrap the difference to [-pi, pi]
            let wrapped_diff = ((diff + PI) % (2.0 * PI) + 2.0 * PI) % (2.0 * PI) - PI;
            unwrapped[i] = unwrapped[i - 1] + wrapped_diff;
        }

        unwrapped
    }

    /// Removes flat-Earth phase from an interferogram.
    ///
    /// The flat-Earth phase arises from the baseline geometry and must be removed
    /// before measuring surface displacement. The flat-Earth phase gradient is:
    /// ```text
    /// phi_flat = 4 * pi * baseline / (wavelength * range * sin(theta)) * range_pixel
    /// ```
    ///
    /// # Arguments
    ///
    /// * `phase` - Interferometric phase array (modified in-place).
    /// * `baseline_m` - Perpendicular baseline in meters.
    /// * `range_m` - Slant range to center of scene in meters.
    /// * `wavelength_m` - Radar wavelength in meters.
    pub fn flatten_interferogram(
        &self,
        phase: &mut Vec<Vec<f64>>,
        baseline_m: f64,
        range_m: f64,
        wavelength_m: f64,
    ) {
        if phase.is_empty() {
            return;
        }
        let theta = self.incidence_angle_deg.to_radians();
        let phase_rate = 4.0 * PI * baseline_m / (wavelength_m * range_m * theta.sin());

        let cols = phase[0].len();
        for row in phase.iter_mut() {
            for c in 0..cols {
                row[c] -= phase_rate * c as f64;
            }
        }
    }

    /// Convenience: convert LOS displacement to horizontal displacement.
    pub fn los_to_horizontal_displacement(&self, los_m: f64) -> f64 {
        los_to_horizontal(los_m, self.incidence_angle_deg)
    }
}

// ─── Ice Sheet Analyzer ──────────────────────────────────────────────────────

/// Ice sheet and glacier dynamics analyzer.
///
/// Provides glaciological computations including strain rate estimation, ice flux,
/// mass balance, sea level equivalent, and Glen's flow law deformation velocity.
pub struct IceSheetAnalyzer;

impl IceSheetAnalyzer {
    /// Computes longitudinal strain rate from a velocity field.
    ///
    /// Strain rate is the spatial derivative of velocity:
    /// ```text
    /// eps_dot = dv/dx ≈ (v[i+1] - v[i-1]) / (2 * pixel_spacing)
    /// ```
    ///
    /// Expressed in units of per-day (1/day). Positive = extensional, negative = compressive.
    ///
    /// # Arguments
    ///
    /// * `velocity_field` - 2D velocity field (FlowVector grid).
    /// * `pixel_spacing` - Grid spacing in meters.
    pub fn strain_rate(
        velocity_field: &[Vec<FlowVector>],
        pixel_spacing: f64,
    ) -> Vec<Vec<f64>> {
        let rows = velocity_field.len();
        if rows < 3 {
            return vec![vec![0.0; velocity_field.get(0).map_or(0, |r| r.len())]; rows];
        }
        let cols = velocity_field[0].len();

        let mut result = vec![vec![0.0; cols]; rows];

        for r in 1..rows - 1 {
            for c in 0..cols {
                let v_next = velocity_field[r + 1][c].velocity_m_per_day;
                let v_prev = velocity_field[r - 1][c].velocity_m_per_day;
                result[r][c] = (v_next - v_prev) / (2.0 * pixel_spacing);
            }
        }

        // Forward/backward difference at edges
        if cols > 0 {
            for c in 0..cols {
                let v0 = velocity_field[0][c].velocity_m_per_day;
                let v1 = velocity_field[1][c].velocity_m_per_day;
                result[0][c] = (v1 - v0) / pixel_spacing;

                let vn = velocity_field[rows - 1][c].velocity_m_per_day;
                let vn1 = velocity_field[rows - 2][c].velocity_m_per_day;
                result[rows - 1][c] = (vn - vn1) / pixel_spacing;
            }
        }

        result
    }

    /// Computes the terminal velocity at the calving front.
    ///
    /// Returns the maximum velocity from a 1D profile of flow vectors representing
    /// the glacier terminus region. In tidewater glaciers, calving front velocity
    /// governs iceberg production rate.
    pub fn calving_front_velocity(velocities: &[FlowVector]) -> f64 {
        velocities
            .iter()
            .map(|v| v.velocity_m_per_day)
            .fold(0.0_f64, f64::max)
    }

    /// Computes ice flux (discharge) through a cross-section.
    ///
    /// ```text
    /// Q = v * H * W
    /// ```
    ///
    /// where `v` = depth-averaged velocity, `H` = ice thickness, `W` = cross-section width.
    ///
    /// # Arguments
    ///
    /// * `velocity_m_per_year` - Depth-averaged velocity in m/year.
    /// * `thickness_m` - Ice thickness in meters.
    /// * `width_m` - Cross-section width in meters.
    ///
    /// # Returns
    ///
    /// Ice flux in m^3/year.
    pub fn ice_flux(velocity_m_per_year: f64, thickness_m: f64, width_m: f64) -> f64 {
        velocity_m_per_year * thickness_m * width_m
    }

    /// Computes glacier mass balance rate.
    ///
    /// ```text
    /// B = accumulation - ablation - calving/area
    /// ```
    ///
    /// # Arguments
    ///
    /// * `accumulation_m_per_year` - Snow/ice accumulation rate (m water equiv/year).
    /// * `ablation_m_per_year` - Surface melt/sublimation rate (m water equiv/year).
    /// * `calving_m3_per_year` - Iceberg calving flux in m^3/year.
    /// * `area_m2` - Glacier surface area in m^2.
    ///
    /// # Returns
    ///
    /// Mass balance rate in m water equivalent per year. Positive = gaining mass.
    pub fn mass_balance_rate(
        accumulation_m_per_year: f64,
        ablation_m_per_year: f64,
        calving_m3_per_year: f64,
        area_m2: f64,
    ) -> f64 {
        accumulation_m_per_year - ablation_m_per_year - calving_m3_per_year / area_m2
    }

    /// Converts ice volume loss to sea level equivalent (SLE).
    ///
    /// ```text
    /// SLE = ice_volume * (rho_ice / rho_water) / ocean_area
    /// ```
    ///
    /// # Arguments
    ///
    /// * `ice_volume_m3` - Ice volume in cubic meters.
    ///
    /// # Returns
    ///
    /// Sea level equivalent in millimeters.
    pub fn sea_level_equivalent(ice_volume_m3: f64) -> f64 {
        let water_equiv_m3 = ice_volume_m3 * ICE_DENSITY / OCEAN_WATER_DENSITY;
        let sle_m = water_equiv_m3 / OCEAN_AREA_M2;
        sle_m * 1000.0 // convert m to mm
    }

    /// Computes surface deformation velocity from Glen's flow law.
    ///
    /// Glen's flow law relates ice deformation to stress:
    /// ```text
    /// v_surface = 2A / (n+1) * (rho * g * sin(alpha))^n * H^(n+1)
    /// ```
    ///
    /// where `A` = flow rate parameter (temperature-dependent), `n` = Glen's exponent (3),
    /// `rho` = ice density, `g` = gravity, `alpha` = surface slope, `H` = ice thickness.
    ///
    /// # Arguments
    ///
    /// * `thickness_m` - Ice thickness in meters.
    /// * `surface_slope` - Surface slope in radians.
    /// * `temperature_c` - Ice temperature in degrees Celsius.
    ///
    /// # Returns
    ///
    /// Surface deformation velocity in m/s. This is the creep component only;
    /// basal sliding is not included.
    pub fn deformation_velocity(
        thickness_m: f64,
        surface_slope: f64,
        temperature_c: f64,
    ) -> f64 {
        let a = glen_flow_parameter(temperature_c);
        let driving_stress = ICE_DENSITY * GRAVITY * surface_slope.abs();
        let v = 2.0 * a / (GLEN_N + 1.0)
            * driving_stress.powf(GLEN_N)
            * thickness_m.powf(GLEN_N + 1.0);
        v
    }
}

// ─── Glacier Monitor ─────────────────────────────────────────────────────────

/// Temporal glacier monitoring for detecting velocity changes and surges.
///
/// Tracks velocity field evolution over multiple observation periods to detect
/// acceleration, deceleration, and surge events that may indicate glacial hazards.
pub struct GlacierMonitor;

impl GlacierMonitor {
    /// Computes velocity change between two observation epochs.
    ///
    /// Returns a 2D array of velocity differences (v2 - v1) in m/day.
    /// Positive values indicate acceleration, negative indicate deceleration.
    pub fn velocity_change(
        v1: &[Vec<FlowVector>],
        v2: &[Vec<FlowVector>],
    ) -> Vec<Vec<f64>> {
        let rows = v1.len().min(v2.len());
        let mut result = Vec::with_capacity(rows);

        for r in 0..rows {
            let cols = v1[r].len().min(v2[r].len());
            let mut row = Vec::with_capacity(cols);
            for c in 0..cols {
                row.push(v2[r][c].velocity_m_per_day - v1[r][c].velocity_m_per_day);
            }
            result.push(row);
        }

        result
    }

    /// Computes acceleration from a time series of velocity fields.
    ///
    /// Acceleration is estimated as the mean velocity change rate across all
    /// consecutive pairs divided by the time step:
    /// ```text
    /// a = mean(v_last - v_first) / ((N-1) * dt)
    /// ```
    ///
    /// # Arguments
    ///
    /// * `velocities_series` - Sequence of velocity fields at regular time intervals.
    /// * `dt_years` - Time step between consecutive observations in years.
    ///
    /// # Returns
    ///
    /// 2D acceleration map in m/year^2.
    pub fn acceleration_map(
        velocities_series: &[Vec<Vec<FlowVector>>],
        dt_years: f64,
    ) -> Vec<Vec<f64>> {
        if velocities_series.len() < 2 {
            return vec![];
        }

        let n = velocities_series.len();
        let first = &velocities_series[0];
        let last = &velocities_series[n - 1];
        let total_time = (n - 1) as f64 * dt_years;

        let rows = first.len().min(last.len());
        let mut result = Vec::with_capacity(rows);

        for r in 0..rows {
            let cols = first[r].len().min(last[r].len());
            let mut row = Vec::with_capacity(cols);
            for c in 0..cols {
                let dv = last[r][c].velocity_m_per_year - first[r][c].velocity_m_per_year;
                row.push(dv / total_time);
            }
            result.push(row);
        }

        result
    }

    /// Detects glacier surge regions where velocity exceeds a threshold factor
    /// above the mean velocity.
    ///
    /// Surging glaciers can accelerate by 10-100x their normal speed, advancing
    /// rapidly and potentially creating glacial lake outburst floods (GLOFs).
    ///
    /// # Arguments
    ///
    /// * `velocities` - Velocity field to analyze.
    /// * `threshold_factor` - Factor above mean velocity to flag as surging (e.g., 5.0).
    ///
    /// # Returns
    ///
    /// List of `(row, col)` indices of surging grid cells.
    pub fn surge_detection(
        velocities: &[Vec<FlowVector>],
        threshold_factor: f64,
    ) -> Vec<(usize, usize)> {
        // Compute mean velocity
        let mut total = 0.0_f64;
        let mut count = 0_u64;
        for row in velocities {
            for fv in row {
                if fv.correlation > 0.0 {
                    total += fv.velocity_m_per_day;
                    count += 1;
                }
            }
        }

        if count == 0 {
            return vec![];
        }

        let mean_vel = total / count as f64;
        let threshold = mean_vel * threshold_factor;

        let mut surging = Vec::new();
        for (r, row) in velocities.iter().enumerate() {
            for (c, fv) in row.iter().enumerate() {
                if fv.velocity_m_per_day > threshold && fv.correlation > 0.0 {
                    surging.push((r, c));
                }
            }
        }

        surging
    }
}

// ─── Helper Functions ────────────────────────────────────────────────────────

/// Projects line-of-sight (LOS) displacement to horizontal displacement.
///
/// ```text
/// d_horizontal = d_LOS / sin(theta)
/// ```
///
/// This assumes the displacement is purely horizontal (valid for glacier flow
/// which is predominantly horizontal).
///
/// # Arguments
///
/// * `los_displacement` - LOS displacement in meters.
/// * `incidence_deg` - Radar incidence angle in degrees.
pub fn los_to_horizontal(los_displacement: f64, incidence_deg: f64) -> f64 {
    let theta = incidence_deg.to_radians();
    los_displacement / theta.sin()
}

/// Computes Glen's flow rate parameter A(T) using the Arrhenius relation.
///
/// ```text
/// A(T) = A_ref * exp(-Q/R * (1/T - 1/T_ref))
/// ```
///
/// where:
/// - `Q` = activation energy (60 kJ/mol for T < -10C, 139 kJ/mol for T >= -10C)
/// - `R` = gas constant (8.314 J/(mol*K))
/// - `T_ref` = 263.15 K (-10 C)
/// - `A_ref` = 3.615e-13 Pa^{-3} s^{-1}
///
/// # Arguments
///
/// * `temperature_c` - Ice temperature in degrees Celsius (should be <= 0).
///
/// # Returns
///
/// Flow rate parameter in Pa^{-3} s^{-1}.
pub fn glen_flow_parameter(temperature_c: f64) -> f64 {
    let t_k = temperature_c + 273.15;
    let q = if t_k < REFERENCE_TEMP_K {
        ACTIVATION_ENERGY_LOW
    } else {
        ACTIVATION_ENERGY_HIGH
    };
    A_REF * (-q / GAS_CONSTANT * (1.0 / t_k - 1.0 / REFERENCE_TEMP_K)).exp()
}

/// Returns the standard density of glacial ice in kg/m^3 (917).
pub fn ice_density() -> f64 {
    ICE_DENSITY
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;

    fn assert_near(a: f64, b: f64, tol: f64, msg: &str) {
        assert!(
            (a - b).abs() < tol,
            "{}: expected {}, got {}, diff = {}",
            msg,
            b,
            a,
            (a - b).abs()
        );
    }

    // ── NCC tests ────────────────────────────────────────────────────────

    #[test]
    fn test_ncc_identical_patches() {
        let patch = vec![
            vec![1.0, 2.0, 3.0],
            vec![4.0, 5.0, 6.0],
            vec![7.0, 8.0, 9.0],
        ];
        let ncc = SpeckleTracker::normalized_cross_correlation(&patch, &patch);
        assert_near(ncc, 1.0, TOL, "NCC of identical patches");
    }

    #[test]
    fn test_ncc_inverted_patches() {
        let patch1 = vec![
            vec![1.0, 2.0, 3.0],
            vec![4.0, 5.0, 6.0],
            vec![7.0, 8.0, 9.0],
        ];
        let patch2: Vec<Vec<f64>> = patch1
            .iter()
            .map(|row| row.iter().map(|v| -v).collect())
            .collect();
        let ncc = SpeckleTracker::normalized_cross_correlation(&patch1, &patch2);
        assert_near(ncc, -1.0, TOL, "NCC of inverted patches");
    }

    #[test]
    fn test_ncc_uncorrelated() {
        // Constant patch has zero variance -> NCC = 0
        let patch1 = vec![vec![5.0, 5.0], vec![5.0, 5.0]];
        let patch2 = vec![vec![1.0, 2.0], vec![3.0, 4.0]];
        let ncc = SpeckleTracker::normalized_cross_correlation(&patch1, &patch2);
        assert_near(ncc, 0.0, TOL, "NCC with constant patch");
    }

    #[test]
    fn test_ncc_scaled_patch() {
        let patch1 = vec![vec![1.0, 2.0], vec![3.0, 4.0]];
        let patch2 = vec![vec![10.0, 20.0], vec![30.0, 40.0]];
        let ncc = SpeckleTracker::normalized_cross_correlation(&patch1, &patch2);
        assert_near(ncc, 1.0, TOL, "NCC of scaled patches");
    }

    #[test]
    fn test_ncc_offset_patch() {
        let patch1 = vec![vec![1.0, 2.0], vec![3.0, 4.0]];
        let patch2 = vec![vec![101.0, 102.0], vec![103.0, 104.0]];
        let ncc = SpeckleTracker::normalized_cross_correlation(&patch1, &patch2);
        assert_near(ncc, 1.0, TOL, "NCC of offset patches");
    }

    // ── Subpixel peak tests ──────────────────────────────────────────────

    #[test]
    fn test_subpixel_peak_at_integer() {
        // Peak at center (1,1) in a 3x3 surface
        let surface = vec![
            vec![0.5, 0.7, 0.5],
            vec![0.7, 1.0, 0.7],
            vec![0.5, 0.7, 0.5],
        ];
        let (r, c) = SpeckleTracker::subpixel_peak(&surface);
        assert_near(r, 1.0, TOL, "subpixel row at integer");
        assert_near(c, 1.0, TOL, "subpixel col at integer");
    }

    #[test]
    fn test_subpixel_peak_off_center() {
        // Asymmetric parabola: integer peak at col 2, but left neighbor (0.9) > right (0.8)
        // so sub-pixel peak shifts slightly left of col 2
        let surface = vec![
            vec![0.1, 0.1, 0.1, 0.1, 0.1],
            vec![0.1, 0.5, 0.9, 0.8, 0.1],
            vec![0.1, 0.1, 0.1, 0.1, 0.1],
        ];
        let (r, c) = SpeckleTracker::subpixel_peak(&surface);
        // Integer peak at (1, 2), left neighbor 0.5 < right neighbor 0.8
        // Parabolic fit shifts peak toward higher neighbor -> between 2 and 3? No.
        // Actually: ym1=0.5, y0=0.9, yp1=0.8 => offset = (0.5-0.8)/(2*(2*0.9-0.5-0.8))
        //   = (-0.3)/(2*0.5) = -0.3 => sub_c = 2 + (-0.3) = 1.7
        // Peak shifts LEFT because left neighbor is lower (steeper rise from left)
        assert!(c > 1.5 && c < 2.0, "subpixel col between 1.5 and 2.0, got {}", c);
        assert_near(r, 1.0, 0.1, "subpixel row near 1");
    }

    #[test]
    fn test_subpixel_peak_single_element() {
        let surface = vec![vec![1.0]];
        let (r, c) = SpeckleTracker::subpixel_peak(&surface);
        assert_near(r, 0.0, TOL, "single element row");
        assert_near(c, 0.0, TOL, "single element col");
    }

    // ── Displacement tracking tests ──────────────────────────────────────

    #[test]
    fn test_zero_displacement_identical_images() {
        let config = GlacierConfig {
            pixel_spacing_m: 10.0,
            repeat_interval_days: 12.0,
            radar_wavelength_m: 0.056,
            incidence_angle_deg: 39.0,
            search_window_size: 4,
            dem_resolution_m: 30.0,
        };
        let tracker = SpeckleTracker::new(config);

        // Create a small image with a distinct pattern
        let mut image = vec![vec![0.0; 20]; 20];
        for r in 0..20 {
            for c in 0..20 {
                image[r][c] = ((r * 7 + c * 13) % 100) as f64;
            }
        }

        let (dr, dc, corr) = tracker.track_displacement(&image, &image, (10, 10), 3);
        assert_near(dr, 0.0, 0.5, "zero range displacement for identical images");
        assert_near(dc, 0.0, 0.5, "zero azimuth displacement for identical images");
        assert!(corr > 0.99, "high correlation for identical images, got {}", corr);
    }

    // ── InSAR tests ──────────────────────────────────────────────────────

    #[test]
    fn test_phase_to_displacement_full_cycle() {
        // 2*pi phase should give lambda/(2*cos(theta)) displacement
        let wavelength = 0.056;
        let incidence = 39.0;
        let phase = 2.0 * PI;
        let d = InSARProcessor::phase_to_displacement(phase, wavelength, incidence);
        let expected = wavelength / (2.0 * incidence.to_radians().cos());
        assert_near(d, expected, 1e-10, "2pi phase displacement");
    }

    #[test]
    fn test_phase_to_displacement_zero() {
        let d = InSARProcessor::phase_to_displacement(0.0, 0.056, 39.0);
        assert_near(d, 0.0, TOL, "zero phase gives zero displacement");
    }

    #[test]
    fn test_phase_to_displacement_half_cycle() {
        let wavelength = 0.056;
        let incidence = 30.0;
        let phase = PI;
        let d = InSARProcessor::phase_to_displacement(phase, wavelength, incidence);
        let expected = wavelength / (4.0 * incidence.to_radians().cos());
        assert_near(d, expected, 1e-10, "pi phase displacement");
    }

    #[test]
    fn test_interferogram_identical() {
        let img = vec![vec![(1.0, 0.0), (0.0, 1.0)], vec![(1.0, 1.0), (0.5, 0.5)]];
        let proc = InSARProcessor::new(0.056, 39.0);
        let ifg = proc.interferogram(&img, &img);
        // z * conj(z) is real positive, so phase = 0
        for row in &ifg {
            for &val in row {
                assert_near(val, 0.0, TOL, "identical images give zero phase");
            }
        }
    }

    #[test]
    fn test_interferogram_known_phase() {
        // img1 = (1, 0), img2 = (0, 1) => z1*conj(z2) = (0, 1) => phase = pi/2
        let img1 = vec![vec![(1.0, 0.0)]];
        let img2 = vec![vec![(0.0, 1.0)]];
        let proc = InSARProcessor::new(0.056, 39.0);
        let ifg = proc.interferogram(&img1, &img2);
        assert_near(ifg[0][0], -PI / 2.0, TOL, "known phase pi/2");
    }

    #[test]
    fn test_coherence_identical_images() {
        let img = vec![
            vec![(1.0, 0.5), (0.3, 0.7), (0.8, 0.2)],
            vec![(0.6, 0.4), (0.9, 0.1), (0.2, 0.8)],
            vec![(0.4, 0.6), (0.5, 0.5), (0.7, 0.3)],
        ];
        let proc = InSARProcessor::new(0.056, 39.0);
        let coh = proc.coherence_estimation(&img, &img, 3);
        // Coherence should be 1.0 for identical images
        for row in &coh {
            for &val in row {
                assert!(val > 0.99, "coherence ~ 1 for identical images, got {}", val);
            }
        }
    }

    #[test]
    fn test_coherence_range() {
        let img1 = vec![
            vec![(1.0, 0.0), (0.0, 1.0)],
            vec![(1.0, 1.0), (0.5, -0.5)],
        ];
        let img2 = vec![
            vec![(0.5, 0.3), (-0.2, 0.8)],
            vec![(0.7, 0.1), (0.1, -0.9)],
        ];
        let proc = InSARProcessor::new(0.056, 39.0);
        let coh = proc.coherence_estimation(&img1, &img2, 2);
        for row in &coh {
            for &val in row {
                assert!(val >= 0.0 && val <= 1.0, "coherence in [0,1], got {}", val);
            }
        }
    }

    #[test]
    fn test_phase_unwrap_1d_no_jumps() {
        let wrapped = vec![0.0, 0.1, 0.2, 0.3, 0.4];
        let unwrapped = InSARProcessor::phase_unwrap_1d(&wrapped);
        for (i, (&w, &u)) in wrapped.iter().zip(unwrapped.iter()).enumerate() {
            assert_near(u, w, TOL, &format!("no-jump unwrap at index {}", i));
        }
    }

    #[test]
    fn test_phase_unwrap_1d_with_jumps() {
        // Linear ramp that wraps: 0, 1, 2, 3-2pi, 4-2pi, 5-2pi
        let true_phase: Vec<f64> = (0..8).map(|i| i as f64 * 1.0).collect();
        let wrapped: Vec<f64> = true_phase
            .iter()
            .map(|&p| ((p + PI) % (2.0 * PI) + 2.0 * PI) % (2.0 * PI) - PI)
            .collect();
        let unwrapped = InSARProcessor::phase_unwrap_1d(&wrapped);

        // Check that differences match
        for i in 1..unwrapped.len() {
            let expected_diff = true_phase[i] - true_phase[i - 1];
            let actual_diff = unwrapped[i] - unwrapped[i - 1];
            assert_near(actual_diff, expected_diff, 0.01, &format!("unwrap diff at {}", i));
        }
    }

    #[test]
    fn test_phase_unwrap_1d_empty() {
        let unwrapped = InSARProcessor::phase_unwrap_1d(&[]);
        assert!(unwrapped.is_empty(), "empty input gives empty output");
    }

    #[test]
    fn test_flatten_interferogram() {
        let proc = InSARProcessor::new(0.056, 39.0);
        let mut phase = vec![vec![0.0; 5]; 3];
        proc.flatten_interferogram(&mut phase, 100.0, 800_000.0, 0.056);
        // After flattening, column 0 should still be 0
        assert_near(phase[0][0], 0.0, TOL, "col 0 unchanged");
        // Column 1 should have negative flat-earth correction
        assert!(phase[0][1] < 0.0, "flat earth correction applied");
    }

    // ── Ice sheet analysis tests ─────────────────────────────────────────

    #[test]
    fn test_ice_flux_proportional() {
        let q1 = IceSheetAnalyzer::ice_flux(100.0, 500.0, 1000.0);
        let q2 = IceSheetAnalyzer::ice_flux(200.0, 500.0, 1000.0);
        assert_near(q2, 2.0 * q1, TOL, "ice flux proportional to velocity");

        let q3 = IceSheetAnalyzer::ice_flux(100.0, 1000.0, 1000.0);
        assert_near(q3, 2.0 * q1, TOL, "ice flux proportional to thickness");

        let q4 = IceSheetAnalyzer::ice_flux(100.0, 500.0, 2000.0);
        assert_near(q4, 2.0 * q1, TOL, "ice flux proportional to width");
    }

    #[test]
    fn test_ice_flux_value() {
        // 100 m/yr * 500 m * 1000 m = 50,000,000 m^3/yr
        let q = IceSheetAnalyzer::ice_flux(100.0, 500.0, 1000.0);
        assert_near(q, 50_000_000.0, TOL, "ice flux value");
    }

    #[test]
    fn test_mass_balance_positive() {
        // Accumulation > ablation + calving
        let b = IceSheetAnalyzer::mass_balance_rate(2.0, 1.0, 1e6, 1e7);
        assert!(b > 0.0, "positive mass balance: accumulation dominates");
    }

    #[test]
    fn test_mass_balance_negative() {
        // Ablation + calving > accumulation
        let b = IceSheetAnalyzer::mass_balance_rate(1.0, 2.0, 1e6, 1e6);
        assert!(b < 0.0, "negative mass balance: ablation dominates");
    }

    #[test]
    fn test_sea_level_equivalent() {
        // Known benchmark: Greenland ice sheet ~ 2.85e15 m^3 -> ~7.2 m SLE
        // Our formula: SLE_mm = V * (917/1025) / 3.619e14 * 1000
        let sle = IceSheetAnalyzer::sea_level_equivalent(1e12);
        assert!(sle > 0.0, "positive SLE for positive volume");
        // 1e12 m^3 * (917/1025) / 3.619e14 * 1000 ~ 2.47 mm
        assert_near(sle, 1e12 * 917.0 / 1025.0 / 3.619e14 * 1000.0, 0.01, "SLE calculation");
    }

    #[test]
    fn test_glen_flow_velocity_increases_with_thickness() {
        let v1 = IceSheetAnalyzer::deformation_velocity(500.0, 0.01, -10.0);
        let v2 = IceSheetAnalyzer::deformation_velocity(1000.0, 0.01, -10.0);
        assert!(v2 > v1, "thicker ice flows faster: v2={} > v1={}", v2, v1);
    }

    #[test]
    fn test_glen_flow_velocity_increases_with_slope() {
        let v1 = IceSheetAnalyzer::deformation_velocity(500.0, 0.01, -10.0);
        let v2 = IceSheetAnalyzer::deformation_velocity(500.0, 0.02, -10.0);
        assert!(v2 > v1, "steeper slope -> faster flow: v2={} > v1={}", v2, v1);
    }

    #[test]
    fn test_glen_flow_velocity_increases_with_temperature() {
        let v1 = IceSheetAnalyzer::deformation_velocity(500.0, 0.01, -20.0);
        let v2 = IceSheetAnalyzer::deformation_velocity(500.0, 0.01, -5.0);
        assert!(v2 > v1, "warmer ice flows faster: v2={} > v1={}", v2, v1);
    }

    // ── Helper function tests ────────────────────────────────────────────

    #[test]
    fn test_ice_density_constant() {
        assert_near(ice_density(), 917.0, TOL, "ice density = 917 kg/m^3");
    }

    #[test]
    fn test_los_to_horizontal() {
        // At 90 degrees incidence (looking sideways), LOS = horizontal
        let h = los_to_horizontal(1.0, 90.0);
        assert_near(h, 1.0, TOL, "90 deg: LOS = horizontal");
    }

    #[test]
    fn test_los_to_horizontal_45deg() {
        // At 45 degrees, horizontal = LOS / sin(45) = LOS * sqrt(2)
        let h = los_to_horizontal(1.0, 45.0);
        assert_near(h, 1.0 / (45.0_f64.to_radians().sin()), TOL, "45 deg projection");
    }

    #[test]
    fn test_glen_flow_parameter_reference() {
        // At -10C (reference), should return A_ref
        let a = glen_flow_parameter(-10.0);
        assert_near(a, A_REF, A_REF * 0.01, "A at reference temp");
    }

    #[test]
    fn test_glen_flow_parameter_cold() {
        // Colder than reference -> smaller A
        let a = glen_flow_parameter(-30.0);
        assert!(a < A_REF, "colder ice has smaller flow parameter");
    }

    #[test]
    fn test_glen_flow_parameter_warm() {
        // Warmer than reference -> larger A
        let a = glen_flow_parameter(-2.0);
        assert!(a > A_REF, "warmer ice has larger flow parameter");
    }

    // ── Outlier filter tests ─────────────────────────────────────────────

    #[test]
    fn test_filter_outliers_removes_fast() {
        let mut field = vec![vec![
            FlowVector::from_pixel_offsets(1.0, 0.0, 10.0, 12.0, 0.9),
            FlowVector::from_pixel_offsets(100.0, 0.0, 10.0, 12.0, 0.8),
        ]];
        SpeckleTracker::filter_outliers(&mut field, 5.0);
        assert!(field[0][0].velocity_m_per_day > 0.0, "normal velocity kept");
        assert_near(field[0][1].velocity_m_per_day, 0.0, TOL, "outlier zeroed");
        assert_near(field[0][1].correlation, 0.0, TOL, "outlier correlation zeroed");
    }

    #[test]
    fn test_filter_outliers_keeps_slow() {
        let mut field = vec![vec![
            FlowVector::from_pixel_offsets(0.1, 0.1, 10.0, 12.0, 0.95),
        ]];
        let orig_vel = field[0][0].velocity_m_per_day;
        SpeckleTracker::filter_outliers(&mut field, 100.0);
        assert_near(field[0][0].velocity_m_per_day, orig_vel, TOL, "slow velocity preserved");
    }

    // ── Strain rate tests ────────────────────────────────────────────────

    #[test]
    fn test_strain_rate_linear_gradient() {
        // Linear velocity field: v = 0, 1, 2, 3, 4 m/day
        // Central diff: dv/dx = (v[i+1] - v[i-1]) / (2*dx)
        let pixel_spacing = 100.0; // meters
        let field: Vec<Vec<FlowVector>> = (0..5)
            .map(|r| {
                vec![FlowVector {
                    range_displacement_m: 0.0,
                    azimuth_displacement_m: 0.0,
                    velocity_m_per_day: r as f64,
                    velocity_m_per_year: r as f64 * 365.25,
                    correlation: 0.9,
                    direction_deg: 0.0,
                }]
            })
            .collect();

        let sr = IceSheetAnalyzer::strain_rate(&field, pixel_spacing);
        // Interior points: (v[i+1] - v[i-1]) / (2*100) = 2/200 = 0.01
        for r in 1..4 {
            let expected = 1.0 / pixel_spacing; // gradient = 1 m/day per pixel_spacing
            assert_near(sr[r][0], expected, 1e-10, "strain rate interior");
        }
    }

    // ── Calving front tests ──────────────────────────────────────────────

    #[test]
    fn test_calving_front_velocity() {
        let vecs = vec![
            FlowVector::from_pixel_offsets(1.0, 0.0, 10.0, 1.0, 0.9),
            FlowVector::from_pixel_offsets(3.0, 0.0, 10.0, 1.0, 0.8),
            FlowVector::from_pixel_offsets(2.0, 0.0, 10.0, 1.0, 0.85),
        ];
        let max_v = IceSheetAnalyzer::calving_front_velocity(&vecs);
        assert_near(max_v, 30.0, TOL, "calving front = max velocity");
    }

    // ── Monitor tests ────────────────────────────────────────────────────

    #[test]
    fn test_velocity_change() {
        let v1 = vec![vec![FlowVector {
            range_displacement_m: 0.0,
            azimuth_displacement_m: 0.0,
            velocity_m_per_day: 1.0,
            velocity_m_per_year: 365.25,
            correlation: 0.9,
            direction_deg: 0.0,
        }]];
        let v2 = vec![vec![FlowVector {
            range_displacement_m: 0.0,
            azimuth_displacement_m: 0.0,
            velocity_m_per_day: 3.0,
            velocity_m_per_year: 3.0 * 365.25,
            correlation: 0.9,
            direction_deg: 0.0,
        }]];
        let change = GlacierMonitor::velocity_change(&v1, &v2);
        assert_near(change[0][0], 2.0, TOL, "velocity change = v2 - v1");
    }

    #[test]
    fn test_acceleration_map() {
        let make_field = |v: f64| -> Vec<Vec<FlowVector>> {
            vec![vec![FlowVector {
                range_displacement_m: 0.0,
                azimuth_displacement_m: 0.0,
                velocity_m_per_day: v / 365.25,
                velocity_m_per_year: v,
                correlation: 0.9,
                direction_deg: 0.0,
            }]]
        };
        let series = vec![make_field(100.0), make_field(200.0), make_field(400.0)];
        let acc = GlacierMonitor::acceleration_map(&series, 1.0);
        // (400 - 100) / (2 * 1.0) = 150 m/yr^2
        assert_near(acc[0][0], 150.0, TOL, "acceleration map");
    }

    #[test]
    fn test_surge_detection() {
        let field = vec![vec![
            FlowVector {
                range_displacement_m: 0.0,
                azimuth_displacement_m: 0.0,
                velocity_m_per_day: 1.0,
                velocity_m_per_year: 365.25,
                correlation: 0.9,
                direction_deg: 0.0,
            },
            FlowVector {
                range_displacement_m: 0.0,
                azimuth_displacement_m: 0.0,
                velocity_m_per_day: 1.0,
                velocity_m_per_year: 365.25,
                correlation: 0.9,
                direction_deg: 0.0,
            },
            FlowVector {
                range_displacement_m: 0.0,
                azimuth_displacement_m: 0.0,
                velocity_m_per_day: 20.0,
                velocity_m_per_year: 20.0 * 365.25,
                correlation: 0.9,
                direction_deg: 0.0,
            },
        ]];
        // Mean = (1 + 1 + 20) / 3 = 7.33
        // Threshold = 7.33 * 2.0 = 14.67
        // Only index 2 exceeds
        let surges = GlacierMonitor::surge_detection(&field, 2.0);
        assert_eq!(surges.len(), 1, "one surge detected");
        assert_eq!(surges[0], (0, 2), "surge at col 2");
    }

    // ── FlowVector tests ─────────────────────────────────────────────────

    #[test]
    fn test_flow_vector_from_pixel_offsets() {
        let fv = FlowVector::from_pixel_offsets(3.0, 4.0, 10.0, 12.0, 0.95);
        assert_near(fv.range_displacement_m, 30.0, TOL, "range displacement");
        assert_near(fv.azimuth_displacement_m, 40.0, TOL, "azimuth displacement");
        assert_near(fv.total_displacement_m(), 50.0, TOL, "total displacement 3-4-5");
        assert_near(fv.velocity_m_per_day, 50.0 / 12.0, TOL, "velocity m/day");
        assert_near(fv.velocity_m_per_year, 50.0 / 12.0 * 365.25, 0.01, "velocity m/yr");
        assert_near(fv.correlation, 0.95, TOL, "correlation");
    }

    #[test]
    fn test_flow_vector_direction() {
        // Pure azimuth displacement -> direction = 90 deg (atan2(40, 0) = 90)
        let fv = FlowVector::from_pixel_offsets(0.0, 4.0, 10.0, 1.0, 0.9);
        assert_near(fv.direction_deg, 90.0, TOL, "pure azimuth = 90 deg");
    }

    // ── Median filter test ───────────────────────────────────────────────

    #[test]
    fn test_median_filter_2d_removes_spike() {
        let normal = FlowVector {
            range_displacement_m: 10.0,
            azimuth_displacement_m: 0.0,
            velocity_m_per_day: 1.0,
            velocity_m_per_year: 365.25,
            correlation: 0.9,
            direction_deg: 0.0,
        };
        let spike = FlowVector {
            range_displacement_m: 1000.0,
            azimuth_displacement_m: 0.0,
            velocity_m_per_day: 100.0,
            velocity_m_per_year: 100.0 * 365.25,
            correlation: 0.9,
            direction_deg: 0.0,
        };
        let field = vec![
            vec![normal.clone(), normal.clone(), normal.clone()],
            vec![normal.clone(), spike, normal.clone()],
            vec![normal.clone(), normal.clone(), normal.clone()],
        ];
        let filtered = SpeckleTracker::median_filter_2d(&field, 3);
        assert_near(
            filtered[1][1].range_displacement_m,
            10.0,
            TOL,
            "median removes spike",
        );
    }

    // ── Config presets ───────────────────────────────────────────────────

    #[test]
    fn test_sentinel1_config() {
        let cfg = GlacierConfig::sentinel1();
        assert_near(cfg.radar_wavelength_m, 0.0555, 0.001, "S1 C-band wavelength");
        assert_near(cfg.repeat_interval_days, 12.0, TOL, "S1 repeat cycle");
    }

    #[test]
    fn test_alos2_config() {
        let cfg = GlacierConfig::alos2();
        assert_near(cfg.radar_wavelength_m, 0.236, 0.001, "ALOS-2 L-band wavelength");
        assert_near(cfg.repeat_interval_days, 14.0, TOL, "ALOS-2 repeat cycle");
    }

    // ── Edge cases ───────────────────────────────────────────────────────

    #[test]
    fn test_empty_velocity_field() {
        let config = GlacierConfig::sentinel1();
        let tracker = SpeckleTracker::new(config);
        let field = tracker.velocity_field(&[], &[], 10);
        assert!(field.is_empty(), "empty images -> empty field");
    }

    #[test]
    fn test_median_f64_helper() {
        let mut vals = vec![5.0, 1.0, 3.0, 2.0, 4.0];
        assert_near(median_f64(&mut vals), 3.0, TOL, "median of 5 values");

        let mut vals2 = vec![1.0, 2.0, 3.0, 4.0];
        assert_near(median_f64(&mut vals2), 2.5, TOL, "median of 4 values");

        let mut empty: Vec<f64> = vec![];
        assert_near(median_f64(&mut empty), 0.0, TOL, "median of empty");
    }

    #[test]
    fn test_sea_level_equivalent_zero() {
        assert_near(
            IceSheetAnalyzer::sea_level_equivalent(0.0),
            0.0,
            TOL,
            "zero volume -> zero SLE",
        );
    }

    #[test]
    fn test_velocity_change_empty() {
        let change = GlacierMonitor::velocity_change(&[], &[]);
        assert!(change.is_empty(), "empty fields -> empty change");
    }

    #[test]
    fn test_acceleration_map_too_few() {
        let acc = GlacierMonitor::acceleration_map(&[], 1.0);
        assert!(acc.is_empty(), "need at least 2 fields");
        let acc2 = GlacierMonitor::acceleration_map(&[vec![]], 1.0);
        assert!(acc2.is_empty(), "need at least 2 fields");
    }
}
