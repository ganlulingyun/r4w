// Atomic Force Microscope (AFM) Processor
// Surface topography and mechanical properties: force-distance curves, adhesion force,
// Young's modulus (Hertz/DMT/JKR models), surface roughness, grain analysis, phase imaging
//
// No external crate dependencies - all math from scratch using only std.

/// AFM scan data (height image)
#[derive(Debug, Clone)]
pub struct AfmImage {
    /// Height values in nm, row-major
    pub heights_nm: Vec<f64>,
    /// Number of rows (Y pixels)
    pub rows: usize,
    /// Number of columns (X pixels)
    pub cols: usize,
    /// Scan size in nm
    pub scan_size_nm: f64,
}

/// Force-distance curve data
#[derive(Debug, Clone)]
pub struct ForceCurve {
    /// Z piezo displacement (nm)
    pub z_nm: Vec<f64>,
    /// Deflection force (nN)
    pub force_nn: Vec<f64>,
}

/// Adhesion force result
#[derive(Debug, Clone)]
pub struct AdhesionResult {
    pub adhesion_force_nn: f64,
    pub snap_in_distance_nm: f64,
    pub contact_point_nm: f64,
}

/// Young's modulus result
#[derive(Debug, Clone)]
pub struct ModulusResult {
    pub youngs_modulus_gpa: f64,
    pub model: String,
    pub indentation_nm: f64,
}

/// Surface roughness metrics
#[derive(Debug, Clone)]
pub struct RoughnessMetrics {
    pub ra_nm: f64,  // Arithmetic average roughness
    pub rq_nm: f64,  // Root mean square roughness
    pub rmax_nm: f64, // Maximum height range
    pub rsk: f64,     // Skewness
    pub rku: f64,     // Kurtosis
    pub rz_nm: f64,   // Ten-point height
}

/// Grain analysis result
#[derive(Debug, Clone)]
pub struct GrainResult {
    pub grain_count: usize,
    pub mean_area_nm2: f64,
    pub mean_height_nm: f64,
    pub total_coverage_fraction: f64,
}

/// AFM processor
pub struct AfmProcessor {
    pub image: AfmImage,
}

impl AfmProcessor {
    pub fn new(image: AfmImage) -> Self {
        Self { image }
    }

    /// Get pixel size in nm
    pub fn pixel_size_nm(&self) -> f64 {
        if self.image.cols < 2 { return 0.0; }
        self.image.scan_size_nm / (self.image.cols as f64 - 1.0)
    }

    /// Plane-fit background subtraction (first-order flatten)
    /// Fits z = ax + by + c and subtracts
    pub fn plane_subtract(&self) -> Vec<f64> {
        plane_subtract(&self.image.heights_nm, self.image.rows, self.image.cols)
    }

    /// Line-by-line leveling (subtract median of each row)
    pub fn line_level(&self) -> Vec<f64> {
        line_level_median(&self.image.heights_nm, self.image.rows, self.image.cols)
    }

    /// Calculate roughness metrics
    pub fn roughness(&self) -> RoughnessMetrics {
        calculate_roughness(&self.image.heights_nm)
    }

    /// Calculate roughness on flattened data
    pub fn roughness_flattened(&self) -> RoughnessMetrics {
        let flat = self.plane_subtract();
        calculate_roughness(&flat)
    }

    /// Simple grain analysis using height threshold
    pub fn grain_analysis(&self, threshold_nm: f64) -> GrainResult {
        grain_analysis(&self.image.heights_nm, self.image.rows, self.image.cols,
                      self.pixel_size_nm(), threshold_nm)
    }

    /// Profile extraction along a row
    pub fn profile_row(&self, row: usize) -> Vec<f64> {
        if row >= self.image.rows { return Vec::new(); }
        let start = row * self.image.cols;
        let end = start + self.image.cols;
        self.image.heights_nm[start..end].to_vec()
    }

    /// Profile extraction along a column
    pub fn profile_col(&self, col: usize) -> Vec<f64> {
        if col >= self.image.cols { return Vec::new(); }
        (0..self.image.rows)
            .map(|r| self.image.heights_nm[r * self.image.cols + col])
            .collect()
    }

    /// Power spectral density of surface (1D, along rows, averaged)
    pub fn surface_psd(&self) -> Vec<f64> {
        surface_psd_1d(&self.image.heights_nm, self.image.rows, self.image.cols)
    }

    /// Bearing ratio (Abbott-Firestone) curve
    pub fn bearing_ratio(&self, n_levels: usize) -> Vec<(f64, f64)> {
        bearing_ratio_curve(&self.image.heights_nm, n_levels)
    }
}

/// Plane subtraction using least-squares fit
pub fn plane_subtract(heights: &[f64], rows: usize, cols: usize) -> Vec<f64> {
    let n = rows * cols;
    if n == 0 || n != heights.len() { return heights.to_vec(); }

    // Fit z = ax + by + c using normal equations
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_z = 0.0;
    let mut sum_xx = 0.0;
    let mut sum_yy = 0.0;
    let mut sum_xy = 0.0;
    let mut sum_xz = 0.0;
    let mut sum_yz = 0.0;
    let nf = n as f64;

    for r in 0..rows {
        for c in 0..cols {
            let x = c as f64;
            let y = r as f64;
            let z = heights[r * cols + c];
            sum_x += x;
            sum_y += y;
            sum_z += z;
            sum_xx += x * x;
            sum_yy += y * y;
            sum_xy += x * y;
            sum_xz += x * z;
            sum_yz += y * z;
        }
    }

    // Solve 3x3 system [A]{a,b,c} = {rhs}
    let a00 = sum_xx - sum_x * sum_x / nf;
    let a01 = sum_xy - sum_x * sum_y / nf;
    let a10 = a01;
    let a11 = sum_yy - sum_y * sum_y / nf;
    let r0 = sum_xz - sum_x * sum_z / nf;
    let r1 = sum_yz - sum_y * sum_z / nf;

    let det = a00 * a11 - a01 * a10;
    if det.abs() < 1e-30 { return heights.to_vec(); }

    let a = (a11 * r0 - a01 * r1) / det;
    let b = (a00 * r1 - a10 * r0) / det;
    let c_val = (sum_z - a * sum_x - b * sum_y) / nf;

    let mut result = Vec::with_capacity(n);
    for r in 0..rows {
        for c_idx in 0..cols {
            let plane_val = a * (c_idx as f64) + b * (r as f64) + c_val;
            result.push(heights[r * cols + c_idx] - plane_val);
        }
    }
    result
}

/// Line-by-line leveling using median subtraction
pub fn line_level_median(heights: &[f64], rows: usize, cols: usize) -> Vec<f64> {
    let n = rows * cols;
    if n == 0 || n != heights.len() { return heights.to_vec(); }

    let mut result = heights.to_vec();
    for r in 0..rows {
        let start = r * cols;
        let end = start + cols;
        let mut row_vals: Vec<f64> = result[start..end].to_vec();
        row_vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median = if cols % 2 == 0 {
            (row_vals[cols / 2 - 1] + row_vals[cols / 2]) / 2.0
        } else {
            row_vals[cols / 2]
        };
        for c in 0..cols {
            result[start + c] -= median;
        }
    }
    result
}

/// Calculate surface roughness metrics
pub fn calculate_roughness(heights: &[f64]) -> RoughnessMetrics {
    let n = heights.len();
    if n == 0 {
        return RoughnessMetrics { ra_nm: 0.0, rq_nm: 0.0, rmax_nm: 0.0, rsk: 0.0, rku: 0.0, rz_nm: 0.0 };
    }

    let mean: f64 = heights.iter().sum::<f64>() / n as f64;
    let deviations: Vec<f64> = heights.iter().map(|h| h - mean).collect();

    let ra = deviations.iter().map(|d| d.abs()).sum::<f64>() / n as f64;
    let rq = (deviations.iter().map(|d| d * d).sum::<f64>() / n as f64).sqrt();

    let min_h = heights.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_h = heights.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let rmax = max_h - min_h;

    // Skewness: Rsk = (1/N) * Σ(z_i - mean)^3 / Rq^3
    let rsk = if rq > 1e-30 {
        deviations.iter().map(|d| d.powi(3)).sum::<f64>() / (n as f64 * rq.powi(3))
    } else { 0.0 };

    // Kurtosis: Rku = (1/N) * Σ(z_i - mean)^4 / Rq^4
    let rku = if rq > 1e-30 {
        deviations.iter().map(|d| d.powi(4)).sum::<f64>() / (n as f64 * rq.powi(4))
    } else { 0.0 };

    // Rz: average of 5 highest peaks + 5 deepest valleys (simplified)
    let mut sorted = heights.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let k = 5.min(n);
    let top5: f64 = sorted[n - k..].iter().sum::<f64>() / k as f64;
    let bot5: f64 = sorted[..k].iter().sum::<f64>() / k as f64;
    let rz = top5 - bot5;

    RoughnessMetrics { ra_nm: ra, rq_nm: rq, rmax_nm: rmax, rsk, rku, rz_nm: rz }
}

/// Force-distance curve analysis
pub fn analyze_force_curve(curve: &ForceCurve) -> AdhesionResult {
    let n = curve.z_nm.len().min(curve.force_nn.len());
    if n < 3 {
        return AdhesionResult { adhesion_force_nn: 0.0, snap_in_distance_nm: 0.0, contact_point_nm: 0.0 };
    }

    // Find minimum force (maximum adhesion = maximum negative force on retract)
    let mut min_force = f64::INFINITY;
    let mut min_idx = 0;
    for i in 0..n {
        if curve.force_nn[i] < min_force {
            min_force = curve.force_nn[i];
            min_idx = i;
        }
    }

    let adhesion = -min_force.min(0.0);

    // Find contact point (first large deflection change on approach)
    let mut contact_z = curve.z_nm[0];
    for i in 1..n {
        if curve.force_nn[i] > 0.1 * adhesion {
            contact_z = curve.z_nm[i];
            break;
        }
    }

    let snap_in = (curve.z_nm[min_idx] - contact_z).abs();

    AdhesionResult {
        adhesion_force_nn: adhesion,
        snap_in_distance_nm: snap_in,
        contact_point_nm: contact_z,
    }
}

/// Hertz model: F = (4/3) * E* * sqrt(R) * δ^(3/2)
/// where E* = reduced modulus, R = tip radius, δ = indentation
/// E* = E / (1 - ν²) for rigid tip
pub fn hertz_modulus_gpa(force_nn: f64, indentation_nm: f64, tip_radius_nm: f64, poisson_ratio: f64) -> f64 {
    if indentation_nm <= 0.0 || tip_radius_nm <= 0.0 { return 0.0; }
    // F = (4/3) * E/(1-ν²) * sqrt(R) * δ^1.5
    // E = F * (1-ν²) * 3 / (4 * sqrt(R) * δ^1.5)
    let force_n = force_nn * 1e-9; // nN to N
    let indent_m = indentation_nm * 1e-9;
    let radius_m = tip_radius_nm * 1e-9;
    let e_star = force_n * 3.0 / (4.0 * radius_m.sqrt() * indent_m.powf(1.5));
    let e = e_star * (1.0 - poisson_ratio * poisson_ratio);
    e * 1e-9 // Pa to GPa
}

/// DMT (Derjaguin-Muller-Toporov) model: F - F_adh = (4/3) * E* * sqrt(R) * δ^(3/2)
pub fn dmt_modulus_gpa(force_nn: f64, adhesion_nn: f64, indentation_nm: f64, tip_radius_nm: f64, poisson_ratio: f64) -> f64 {
    let net_force = force_nn - (-adhesion_nn);
    hertz_modulus_gpa(net_force, indentation_nm, tip_radius_nm, poisson_ratio)
}

/// JKR (Johnson-Kendall-Roberts) contact radius
/// a³ = (R/K) * [F + 3πWR + sqrt(6πWRF + (3πWR)²)]
/// where K = (4/3)E*, W = work of adhesion
pub fn jkr_contact_radius_nm(force_nn: f64, tip_radius_nm: f64, work_of_adhesion_mj_m2: f64, e_star_gpa: f64) -> f64 {
    let r = tip_radius_nm * 1e-9;
    let k = (4.0 / 3.0) * e_star_gpa * 1e9;
    let w = work_of_adhesion_mj_m2 * 1e-3;
    let f = force_nn * 1e-9;
    let three_pi_wr = 3.0 * std::f64::consts::PI * w * r;
    let discriminant = 6.0 * std::f64::consts::PI * w * r * f + three_pi_wr * three_pi_wr;
    if discriminant < 0.0 || k < 1e-30 { return 0.0; }
    let a_cubed = (r / k) * (f + three_pi_wr + discriminant.sqrt());
    if a_cubed <= 0.0 { return 0.0; }
    a_cubed.cbrt() * 1e9 // m to nm
}

/// Spring constant calibration: Sader method (simplified)
/// k = 0.1906 * ρ_f * b² * L * Q * Γ_i * f² (simplified expression)
pub fn sader_spring_constant(freq_hz: f64, q_factor: f64, length_um: f64, width_um: f64) -> f64 {
    // Simplified: k ≈ 0.2427 * ρ * w * L³ * Q * (2πf)² for rectangular cantilevers
    // Using air density 1.18 kg/m³
    let rho_air = 1.18; // kg/m³
    let w = width_um * 1e-6;
    let l = length_um * 1e-6;
    let omega = 2.0 * std::f64::consts::PI * freq_hz;
    0.1906 * rho_air * w * w * l * q_factor * omega * omega / (4.0 * std::f64::consts::PI * std::f64::consts::PI)
}

/// Thermal noise calibration: k = kB*T / <z²>
pub fn thermal_noise_spring_constant(temperature_k: f64, mean_sq_deflection_nm2: f64) -> f64 {
    if mean_sq_deflection_nm2 <= 0.0 { return 0.0; }
    let kb = 1.38064852e-23; // J/K
    let z2_m2 = mean_sq_deflection_nm2 * 1e-18; // nm² to m²
    kb * temperature_k / z2_m2 // N/m
}

/// Grain analysis: find connected regions above threshold
pub fn grain_analysis(heights: &[f64], rows: usize, cols: usize, pixel_size_nm: f64, threshold: f64) -> GrainResult {
    let n = rows * cols;
    if n == 0 || n != heights.len() {
        return GrainResult { grain_count: 0, mean_area_nm2: 0.0, mean_height_nm: 0.0, total_coverage_fraction: 0.0 };
    }

    // Binary mask
    let mask: Vec<bool> = heights.iter().map(|h| *h > threshold).collect();

    // Connected component labeling (4-connectivity)
    let mut labels = vec![0_usize; n];
    let mut label_count = 0;

    for r in 0..rows {
        for c in 0..cols {
            let idx = r * cols + c;
            if !mask[idx] || labels[idx] > 0 { continue; }
            // BFS flood fill
            label_count += 1;
            let mut stack = vec![(r, c)];
            while let Some((cr, cc)) = stack.pop() {
                let ci = cr * cols + cc;
                if labels[ci] > 0 || !mask[ci] { continue; }
                labels[ci] = label_count;
                if cr > 0 { stack.push((cr - 1, cc)); }
                if cr + 1 < rows { stack.push((cr + 1, cc)); }
                if cc > 0 { stack.push((cr, cc - 1)); }
                if cc + 1 < cols { stack.push((cr, cc + 1)); }
            }
        }
    }

    if label_count == 0 {
        return GrainResult { grain_count: 0, mean_area_nm2: 0.0, mean_height_nm: 0.0, total_coverage_fraction: 0.0 };
    }

    // Compute per-grain statistics
    let mut grain_areas = vec![0_usize; label_count];
    let mut grain_height_sum = vec![0.0; label_count];

    for i in 0..n {
        if labels[i] > 0 {
            let g = labels[i] - 1;
            grain_areas[g] += 1;
            grain_height_sum[g] += heights[i];
        }
    }

    let pixel_area = pixel_size_nm * pixel_size_nm;
    let total_pixels_above: usize = grain_areas.iter().sum();

    let mean_area = grain_areas.iter().map(|a| *a as f64 * pixel_area).sum::<f64>() / label_count as f64;
    let mean_height = if total_pixels_above > 0 {
        grain_height_sum.iter().sum::<f64>() / total_pixels_above as f64
    } else { 0.0 };

    GrainResult {
        grain_count: label_count,
        mean_area_nm2: mean_area,
        mean_height_nm: mean_height,
        total_coverage_fraction: total_pixels_above as f64 / n as f64,
    }
}

/// 1D surface PSD (averaged over rows)
pub fn surface_psd_1d(heights: &[f64], rows: usize, cols: usize) -> Vec<f64> {
    if cols < 2 || rows == 0 { return Vec::new(); }

    let n_freq = cols / 2 + 1;
    let mut psd = vec![0.0; n_freq];

    for r in 0..rows {
        // DFT of each row
        let row_start = r * cols;
        for k in 0..n_freq {
            let mut re = 0.0;
            let mut im = 0.0;
            for j in 0..cols {
                let angle = -2.0 * std::f64::consts::PI * (k as f64) * (j as f64) / (cols as f64);
                re += heights[row_start + j] * angle.cos();
                im += heights[row_start + j] * angle.sin();
            }
            psd[k] += (re * re + im * im) / (cols as f64);
        }
    }

    // Average over rows
    for p in psd.iter_mut() {
        *p /= rows as f64;
    }
    psd
}

/// Bearing ratio (Abbott-Firestone) curve
pub fn bearing_ratio_curve(heights: &[f64], n_levels: usize) -> Vec<(f64, f64)> {
    if heights.is_empty() || n_levels == 0 { return Vec::new(); }

    let min_h = heights.iter().cloned().fold(f64::INFINITY, f64::min);
    let max_h = heights.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let range = max_h - min_h;
    if range < 1e-30 { return vec![(min_h, 1.0)]; }

    let n = heights.len() as f64;
    let mut curve = Vec::with_capacity(n_levels);
    for i in 0..n_levels {
        let level = max_h - (i as f64) * range / (n_levels as f64 - 1.0);
        let count = heights.iter().filter(|h| **h >= level).count();
        curve.push((level, count as f64 / n));
    }
    curve
}

/// Phase imaging: calculate phase shift from amplitude and drive
/// Phase = atan(A_exc / (Q * A_free)) for tapping mode quality
pub fn phase_shift_from_amplitude(amplitude: f64, free_amplitude: f64, q_factor: f64) -> f64 {
    if free_amplitude <= 0.0 || q_factor <= 0.0 { return 0.0; }
    let ratio = amplitude / free_amplitude;
    if ratio >= 1.0 { return 0.0; }
    // sin(φ) ≈ A/A_free + A_free/(Q*A) * (1 - (A/A_free)²)
    // Simplified: φ ≈ acos(A/A_free) for repulsive regime
    (ratio).acos()
}

/// Tip deconvolution: estimate true feature width
/// w_true ≈ sqrt(w_measured² - 4*R*h)
/// where R = tip radius, h = feature height
pub fn tip_deconvolve_width(measured_width_nm: f64, tip_radius_nm: f64, feature_height_nm: f64) -> f64 {
    let correction = 4.0 * tip_radius_nm * feature_height_nm;
    let w2 = measured_width_nm * measured_width_nm - correction;
    if w2 <= 0.0 { return 0.0; }
    w2.sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn make_flat_image(rows: usize, cols: usize, height: f64) -> AfmImage {
        AfmImage {
            heights_nm: vec![height; rows * cols],
            rows,
            cols,
            scan_size_nm: 1000.0,
        }
    }

    fn make_tilted_image(rows: usize, cols: usize) -> AfmImage {
        let mut heights = Vec::with_capacity(rows * cols);
        for r in 0..rows {
            for c in 0..cols {
                heights.push(0.5 * c as f64 + 0.3 * r as f64 + 10.0);
            }
        }
        AfmImage { heights_nm: heights, rows, cols, scan_size_nm: 1000.0 }
    }

    fn make_peak_image(rows: usize, cols: usize, peak_height: f64) -> AfmImage {
        let mut heights = Vec::with_capacity(rows * cols);
        let cr = rows as f64 / 2.0;
        let cc = cols as f64 / 2.0;
        let sigma = 3.0;
        for r in 0..rows {
            for c in 0..cols {
                let dx = c as f64 - cc;
                let dy = r as f64 - cr;
                heights.push(peak_height * (-0.5 * (dx * dx + dy * dy) / (sigma * sigma)).exp());
            }
        }
        AfmImage { heights_nm: heights, rows, cols, scan_size_nm: 1000.0 }
    }

    #[test]
    fn test_pixel_size() {
        let img = make_flat_image(10, 10, 0.0);
        let proc = AfmProcessor::new(img);
        assert!(approx_eq(proc.pixel_size_nm(), 1000.0 / 9.0, 0.01));
    }

    #[test]
    fn test_plane_subtract_flat() {
        let img = make_flat_image(10, 10, 5.0);
        let result = plane_subtract(&img.heights_nm, img.rows, img.cols);
        for v in &result {
            assert!(approx_eq(*v, 0.0, 0.01));
        }
    }

    #[test]
    fn test_plane_subtract_tilted() {
        let img = make_tilted_image(10, 10);
        let result = plane_subtract(&img.heights_nm, img.rows, img.cols);
        let rq_before = calculate_roughness(&img.heights_nm).rq_nm;
        let rq_after = calculate_roughness(&result).rq_nm;
        assert!(rq_after < rq_before);
    }

    #[test]
    fn test_line_level_median() {
        let img = make_tilted_image(10, 10);
        let result = line_level_median(&img.heights_nm, img.rows, img.cols);
        // Each row median should be near zero
        for r in 0..10 {
            let start = r * 10;
            let mut row: Vec<f64> = result[start..start + 10].to_vec();
            row.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let median = (row[4] + row[5]) / 2.0;
            assert!(approx_eq(median, 0.0, 0.01));
        }
    }

    #[test]
    fn test_roughness_flat() {
        let heights = vec![5.0; 100];
        let r = calculate_roughness(&heights);
        assert!(approx_eq(r.ra_nm, 0.0, 0.01));
        assert!(approx_eq(r.rq_nm, 0.0, 0.01));
        assert!(approx_eq(r.rmax_nm, 0.0, 0.01));
    }

    #[test]
    fn test_roughness_known() {
        // Heights: -1, 0, 1, 0, -1 (mean = -0.2)
        let heights = vec![-1.0, 0.0, 1.0, 0.0, -1.0];
        let r = calculate_roughness(&heights);
        assert!(r.ra_nm > 0.0);
        assert!(r.rq_nm > 0.0);
        assert!(approx_eq(r.rmax_nm, 2.0, 0.01));
    }

    #[test]
    fn test_roughness_rq_ge_ra() {
        let heights = vec![0.0, 1.0, -1.0, 0.5, -0.5, 2.0, -2.0, 0.0];
        let r = calculate_roughness(&heights);
        assert!(r.rq_nm >= r.ra_nm);
    }

    #[test]
    fn test_roughness_symmetric_skewness() {
        // Symmetric distribution should have near-zero skewness
        let heights: Vec<f64> = (-50..=50).map(|i| i as f64 * 0.1).collect();
        let r = calculate_roughness(&heights);
        assert!(approx_eq(r.rsk, 0.0, 0.1));
    }

    #[test]
    fn test_hertz_modulus() {
        // 10 nN force, 5 nm indentation, 20 nm tip radius, ν=0.3
        let e = hertz_modulus_gpa(10.0, 5.0, 20.0, 0.3);
        assert!(e > 0.0);
        assert!(e < 100.0); // Reasonable for polymers/soft materials
    }

    #[test]
    fn test_hertz_zero_indent() {
        let e = hertz_modulus_gpa(10.0, 0.0, 20.0, 0.3);
        assert!(approx_eq(e, 0.0, 0.01));
    }

    #[test]
    fn test_dmt_modulus() {
        let e = dmt_modulus_gpa(10.0, 2.0, 5.0, 20.0, 0.3);
        assert!(e > 0.0);
    }

    #[test]
    fn test_jkr_contact_radius() {
        let a = jkr_contact_radius_nm(10.0, 20.0, 30.0, 1.0);
        assert!(a > 0.0);
    }

    #[test]
    fn test_force_curve_analysis() {
        let z: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let mut force = vec![0.0; 100];
        // Approach: snap-in at z=30, contact, repulsive after
        for i in 0..30 { force[i] = 0.0; }
        for i in 30..50 { force[i] = -5.0 + 0.2 * (i as f64 - 30.0); }
        for i in 50..100 { force[i] = -1.0 + 0.5 * (i as f64 - 50.0); }
        let curve = ForceCurve { z_nm: z, force_nn: force };
        let result = analyze_force_curve(&curve);
        assert!(result.adhesion_force_nn > 0.0);
    }

    #[test]
    fn test_sader_spring_constant() {
        // Typical cantilever: 75 kHz, Q=200, L=225 μm, w=30 μm
        let k = sader_spring_constant(75000.0, 200.0, 225.0, 30.0);
        assert!(k > 0.0);
    }

    #[test]
    fn test_thermal_noise_spring_constant() {
        // kB*T at 300K ≈ 4.14e-21 J
        // If <z²> = 0.1 nm² = 1e-19 m²
        let k = thermal_noise_spring_constant(300.0, 0.1);
        // k ≈ 4.14e-21 / 1e-19 = 0.0414 N/m
        assert!(approx_eq(k, 0.0414, 0.005));
    }

    #[test]
    fn test_grain_analysis_no_grains() {
        let img = make_flat_image(10, 10, 0.0);
        let proc = AfmProcessor::new(img);
        let result = proc.grain_analysis(1.0);
        assert_eq!(result.grain_count, 0);
    }

    #[test]
    fn test_grain_analysis_one_grain() {
        let img = make_peak_image(21, 21, 10.0);
        let proc = AfmProcessor::new(img);
        let result = proc.grain_analysis(1.0);
        assert_eq!(result.grain_count, 1);
        assert!(result.mean_area_nm2 > 0.0);
        assert!(result.mean_height_nm > 1.0);
    }

    #[test]
    fn test_bearing_ratio() {
        let heights: Vec<f64> = (0..100).map(|i| i as f64 / 10.0).collect();
        let curve = bearing_ratio_curve(&heights, 11);
        assert_eq!(curve.len(), 11);
        // First entry (highest level): should be small ratio
        assert!(curve[0].1 < 0.2);
        // Last entry (lowest level): should be 1.0
        assert!(approx_eq(curve[10].1, 1.0, 0.01));
    }

    #[test]
    fn test_phase_shift() {
        let phase = phase_shift_from_amplitude(0.5, 1.0, 100.0);
        assert!(phase > 0.0); // acos(0.5) ≈ π/3
        assert!(approx_eq(phase, std::f64::consts::PI / 3.0, 0.01));
    }

    #[test]
    fn test_phase_shift_free() {
        let phase = phase_shift_from_amplitude(1.0, 1.0, 100.0);
        assert!(approx_eq(phase, 0.0, 0.01));
    }

    #[test]
    fn test_tip_deconvolve() {
        // Measured 50 nm width, 10 nm tip radius, 5 nm feature height
        let true_w = tip_deconvolve_width(50.0, 10.0, 5.0);
        // w² = 50² - 4*10*5 = 2500-200 = 2300 => w ≈ 47.96
        assert!(approx_eq(true_w, 47.96, 0.1));
    }

    #[test]
    fn test_tip_deconvolve_too_small() {
        // Correction larger than measurement
        let w = tip_deconvolve_width(5.0, 10.0, 5.0);
        assert!(approx_eq(w, 0.0, 0.01));
    }

    #[test]
    fn test_profile_row() {
        let img = make_tilted_image(5, 5);
        let proc = AfmProcessor::new(img);
        let profile = proc.profile_row(0);
        assert_eq!(profile.len(), 5);
        // Row 0: 10.0, 10.5, 11.0, 11.5, 12.0
        assert!(approx_eq(profile[0], 10.0, 0.01));
        assert!(approx_eq(profile[4], 12.0, 0.01));
    }

    #[test]
    fn test_profile_col() {
        let img = make_tilted_image(5, 5);
        let proc = AfmProcessor::new(img);
        let profile = proc.profile_col(0);
        assert_eq!(profile.len(), 5);
        // Col 0: 10.0, 10.3, 10.6, 10.9, 11.2
        assert!(approx_eq(profile[0], 10.0, 0.01));
        assert!(approx_eq(profile[1], 10.3, 0.01));
    }

    #[test]
    fn test_surface_psd() {
        let img = make_peak_image(16, 16, 5.0);
        let proc = AfmProcessor::new(img);
        let psd = proc.surface_psd();
        assert_eq!(psd.len(), 9); // 16/2 + 1
        assert!(psd[0] > 0.0); // DC component
    }

    #[test]
    fn test_processor_roughness_flattened() {
        let img = make_tilted_image(10, 10);
        let proc = AfmProcessor::new(img);
        let r_raw = proc.roughness();
        let r_flat = proc.roughness_flattened();
        assert!(r_flat.rq_nm < r_raw.rq_nm);
    }

    #[test]
    fn test_roughness_empty() {
        let r = calculate_roughness(&[]);
        assert!(approx_eq(r.ra_nm, 0.0, 0.01));
    }

    #[test]
    fn test_grain_coverage() {
        // All above threshold
        let img = make_flat_image(10, 10, 5.0);
        let result = grain_analysis(&img.heights_nm, img.rows, img.cols, 100.0, 1.0);
        assert_eq!(result.grain_count, 1); // One big connected region
        assert!(approx_eq(result.total_coverage_fraction, 1.0, 0.01));
    }

    #[test]
    fn test_thermal_noise_zero() {
        let k = thermal_noise_spring_constant(300.0, 0.0);
        assert!(approx_eq(k, 0.0, 0.01));
    }

    #[test]
    fn test_roughness_kurtosis_gaussian() {
        // For Gaussian distribution, kurtosis should be ~3
        // Generate pseudo-Gaussian using CLT
        let n = 10000;
        let mut heights = Vec::with_capacity(n);
        for i in 0..n {
            // Simple deterministic "normal-like" distribution
            let x = (i as f64 / n as f64) * 2.0 * std::f64::consts::PI;
            heights.push(x.sin() + (2.0 * x).sin() * 0.5 + (3.0 * x).cos() * 0.3);
        }
        let r = calculate_roughness(&heights);
        // Kurtosis should be finite and positive
        assert!(r.rku > 0.0);
        assert!(r.rku.is_finite());
    }
}
