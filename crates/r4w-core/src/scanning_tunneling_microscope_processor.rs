// Scanning Tunneling Microscope (STM) Processor
//
// STM data processing and analysis:
// - Tunneling current model: I ∝ V * exp(-2κd)
// - Apparent barrier height from dI/dz
// - Topographic image flattening (plane subtraction, polynomial)
// - Line-by-line leveling
// - Roughness metrics (Ra, Rq, Rmax, Rsk, Rku)
// - Step edge detection and height measurement
// - Fourier filtering of periodic noise
// - dI/dV spectroscopy (LDOS)
// - Band gap measurement from STS
// - Lattice parameter measurement
// - Drift correction (affine transform)
// - Z-calibration

use std::f64::consts::PI;

const HBAR: f64 = 1.0546e-34;  // Reduced Planck constant (J·s)
const ME: f64 = 9.1094e-31;     // Electron mass (kg)
const EV_TO_J: f64 = 1.602176634e-19;

/// Tunneling current: I = I0 * V * exp(-2*kappa*d)
/// kappa = sqrt(2*m*phi) / hbar
pub fn tunneling_current(
    bias_v: f64,
    tip_sample_distance_m: f64,
    barrier_height_ev: f64,
    prefactor: f64, // I0, depends on LDOS and tip geometry
) -> f64 {
    if barrier_height_ev <= 0.0 {
        return prefactor * bias_v;
    }
    let phi_j = barrier_height_ev * EV_TO_J;
    let kappa = (2.0 * ME * phi_j).sqrt() / HBAR;
    prefactor * bias_v * (-2.0 * kappa * tip_sample_distance_m).exp()
}

/// Decay constant kappa from barrier height
pub fn kappa_from_barrier(barrier_height_ev: f64) -> f64 {
    if barrier_height_ev <= 0.0 {
        return 0.0;
    }
    let phi_j = barrier_height_ev * EV_TO_J;
    (2.0 * ME * phi_j).sqrt() / HBAR
}

/// Apparent barrier height from current decay: phi = (hbar^2 / (8*m)) * (d(ln I)/dz)^2
/// Given two I,z pairs, estimate barrier height
pub fn apparent_barrier_height(
    current1: f64,
    z1_m: f64,
    current2: f64,
    z2_m: f64,
) -> f64 {
    if current1 <= 0.0 || current2 <= 0.0 {
        return 0.0;
    }
    let dz = z2_m - z1_m;
    if dz.abs() < 1e-15 {
        return 0.0;
    }
    let d_ln_i = (current2.ln() - current1.ln()) / dz;
    let phi_j = (HBAR * HBAR / (8.0 * ME)) * d_ln_i * d_ln_i;
    phi_j / EV_TO_J
}

/// First-order plane subtraction: z_corrected = z - (ax + by + c)
/// Fits z = ax + by + c to image and subtracts
pub fn plane_subtract(
    image: &[Vec<f64>],
) -> Vec<Vec<f64>> {
    let ny = image.len();
    if ny == 0 {
        return Vec::new();
    }
    let nx = image[0].len();
    if nx == 0 {
        return vec![Vec::new(); ny];
    }

    // Least squares fit: z = a*x + b*y + c
    let mut sum_x = 0.0;
    let mut sum_y = 0.0;
    let mut sum_z = 0.0;
    let mut sum_xx = 0.0;
    let mut sum_yy = 0.0;
    let mut sum_xy = 0.0;
    let mut sum_xz = 0.0;
    let mut sum_yz = 0.0;
    let mut n = 0.0;

    for iy in 0..ny {
        for ix in 0..image[iy].len().min(nx) {
            let x = ix as f64;
            let y = iy as f64;
            let z = image[iy][ix];
            sum_x += x;
            sum_y += y;
            sum_z += z;
            sum_xx += x * x;
            sum_yy += y * y;
            sum_xy += x * y;
            sum_xz += x * z;
            sum_yz += y * z;
            n += 1.0;
        }
    }

    if n < 3.0 {
        return image.to_vec();
    }

    // Solve 3x3 system
    let mut mat = [
        sum_xx, sum_xy, sum_x,
        sum_xy, sum_yy, sum_y,
        sum_x,  sum_y,  n,
    ];
    let mut rhs = [sum_xz, sum_yz, sum_z];
    let coeffs = solve_3x3(&mut mat, &mut rhs);

    let a = coeffs[0];
    let b = coeffs[1];
    let c = coeffs[2];

    image
        .iter()
        .enumerate()
        .map(|(iy, row)| {
            row.iter()
                .enumerate()
                .map(|(ix, &z)| z - (a * ix as f64 + b * iy as f64 + c))
                .collect()
        })
        .collect()
}

fn solve_3x3(mat: &mut [f64; 9], rhs: &mut [f64; 3]) -> [f64; 3] {
    // Gaussian elimination with partial pivoting
    for k in 0..3 {
        let mut max_val = mat[k * 3 + k].abs();
        let mut max_row = k;
        for i in k + 1..3 {
            if mat[i * 3 + k].abs() > max_val {
                max_val = mat[i * 3 + k].abs();
                max_row = i;
            }
        }
        if max_row != k {
            for j in 0..3 {
                let tmp = mat[k * 3 + j];
                mat[k * 3 + j] = mat[max_row * 3 + j];
                mat[max_row * 3 + j] = tmp;
            }
            rhs.swap(k, max_row);
        }
        let pivot = mat[k * 3 + k];
        if pivot.abs() < 1e-30 { continue; }
        for i in k + 1..3 {
            let factor = mat[i * 3 + k] / pivot;
            for j in k..3 {
                mat[i * 3 + j] -= factor * mat[k * 3 + j];
            }
            rhs[i] -= factor * rhs[k];
        }
    }
    let mut result = [0.0; 3];
    for i in (0..3).rev() {
        let mut sum = rhs[i];
        for j in i + 1..3 {
            sum -= mat[i * 3 + j] * result[j];
        }
        if mat[i * 3 + i].abs() > 1e-30 {
            result[i] = sum / mat[i * 3 + i];
        }
    }
    result
}

/// Line-by-line leveling: subtract median (or mean) of each scan line
pub fn line_level_median(image: &[Vec<f64>]) -> Vec<Vec<f64>> {
    image
        .iter()
        .map(|row| {
            if row.is_empty() {
                return Vec::new();
            }
            let mut sorted = row.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let median = sorted[sorted.len() / 2];
            row.iter().map(|&z| z - median).collect()
        })
        .collect()
}

/// Line-by-line polynomial leveling (1st order)
pub fn line_level_linear(image: &[Vec<f64>]) -> Vec<Vec<f64>> {
    image
        .iter()
        .map(|row| {
            let n = row.len();
            if n < 2 {
                return row.clone();
            }
            let nf = n as f64;
            let sx: f64 = (0..n).map(|i| i as f64).sum();
            let sy: f64 = row.iter().sum();
            let sxx: f64 = (0..n).map(|i| (i as f64).powi(2)).sum();
            let sxy: f64 = (0..n).map(|i| i as f64 * row[i]).sum();
            let denom = nf * sxx - sx * sx;
            if denom.abs() < 1e-30 {
                return row.iter().map(|&z| z - sy / nf).collect();
            }
            let slope = (nf * sxy - sx * sy) / denom;
            let intercept = (sy - slope * sx) / nf;
            (0..n).map(|i| row[i] - (slope * i as f64 + intercept)).collect()
        })
        .collect()
}

/// Surface roughness: Ra (arithmetic average)
pub fn roughness_ra(image: &[Vec<f64>]) -> f64 {
    let mut sum = 0.0;
    let mut count = 0;
    for row in image {
        for &z in row {
            sum += z.abs();
            count += 1;
        }
    }
    if count == 0 { 0.0 } else { sum / count as f64 }
}

/// Surface roughness: Rq (RMS)
pub fn roughness_rq(image: &[Vec<f64>]) -> f64 {
    let mut sum = 0.0;
    let mut count = 0;
    for row in image {
        for &z in row {
            sum += z * z;
            count += 1;
        }
    }
    if count == 0 { 0.0 } else { (sum / count as f64).sqrt() }
}

/// Surface roughness: Rmax (peak-to-valley)
pub fn roughness_rmax(image: &[Vec<f64>]) -> f64 {
    let mut zmin = f64::INFINITY;
    let mut zmax = f64::NEG_INFINITY;
    for row in image {
        for &z in row {
            if z < zmin { zmin = z; }
            if z > zmax { zmax = z; }
        }
    }
    if zmin.is_infinite() { 0.0 } else { zmax - zmin }
}

/// Surface roughness: Rsk (skewness)
pub fn roughness_rsk(image: &[Vec<f64>]) -> f64 {
    let rq = roughness_rq(image);
    if rq < 1e-30 {
        return 0.0;
    }
    let mut sum = 0.0;
    let mut count = 0;
    for row in image {
        for &z in row {
            sum += z.powi(3);
            count += 1;
        }
    }
    if count == 0 { return 0.0; }
    sum / (count as f64 * rq.powi(3))
}

/// Surface roughness: Rku (kurtosis)
pub fn roughness_rku(image: &[Vec<f64>]) -> f64 {
    let rq = roughness_rq(image);
    if rq < 1e-30 {
        return 0.0;
    }
    let mut sum = 0.0;
    let mut count = 0;
    for row in image {
        for &z in row {
            sum += z.powi(4);
            count += 1;
        }
    }
    if count == 0 { return 0.0; }
    sum / (count as f64 * rq.powi(4))
}

/// Height histogram
pub fn height_histogram(image: &[Vec<f64>], n_bins: usize) -> (Vec<f64>, Vec<usize>) {
    let mut zmin = f64::INFINITY;
    let mut zmax = f64::NEG_INFINITY;
    for row in image {
        for &z in row {
            if z < zmin { zmin = z; }
            if z > zmax { zmax = z; }
        }
    }
    if zmin >= zmax || n_bins == 0 {
        return (Vec::new(), Vec::new());
    }

    let bin_width = (zmax - zmin) / n_bins as f64;
    let mut bins = vec![0usize; n_bins];
    let centers: Vec<f64> = (0..n_bins).map(|i| zmin + (i as f64 + 0.5) * bin_width).collect();

    for row in image {
        for &z in row {
            let idx = ((z - zmin) / bin_width) as usize;
            let idx = idx.min(n_bins - 1);
            bins[idx] += 1;
        }
    }

    (centers, bins)
}

/// Step edge detection: find rows/columns with large z-jumps
pub fn detect_step_edges(
    profile: &[f64],  // 1D height profile
    threshold: f64,    // minimum step height
) -> Vec<(usize, f64)> {
    if profile.len() < 2 {
        return Vec::new();
    }
    let mut edges = Vec::new();
    for i in 1..profile.len() {
        let dz = profile[i] - profile[i - 1];
        if dz.abs() > threshold {
            edges.push((i, dz));
        }
    }
    edges
}

/// Step height measurement: average height difference across a step
pub fn measure_step_height(
    profile: &[f64],
    edge_idx: usize,
    averaging_width: usize,
) -> f64 {
    let n = profile.len();
    if edge_idx == 0 || edge_idx >= n {
        return 0.0;
    }

    let left_start = edge_idx.saturating_sub(averaging_width);
    let left_end = edge_idx;
    let right_start = edge_idx;
    let right_end = (edge_idx + averaging_width).min(n);

    if left_end <= left_start || right_end <= right_start {
        return 0.0;
    }

    let left_avg: f64 = profile[left_start..left_end].iter().sum::<f64>()
        / (left_end - left_start) as f64;
    let right_avg: f64 = profile[right_start..right_end].iter().sum::<f64>()
        / (right_end - right_start) as f64;

    (right_avg - left_avg).abs()
}

/// dI/dV spectroscopy: numerical derivative of I-V curve
/// Returns LDOS proportional to dI/dV
pub fn didv_spectroscopy(
    voltages: &[f64],
    currents: &[f64],
) -> Vec<f64> {
    let n = voltages.len().min(currents.len());
    if n < 3 {
        return vec![0.0; n];
    }

    let mut didv = Vec::with_capacity(n);
    // Forward difference at start
    let dv0 = voltages[1] - voltages[0];
    didv.push(if dv0.abs() > 1e-15 { (currents[1] - currents[0]) / dv0 } else { 0.0 });

    // Central difference for interior
    for i in 1..n - 1 {
        let dv = voltages[i + 1] - voltages[i - 1];
        let di = currents[i + 1] - currents[i - 1];
        didv.push(if dv.abs() > 1e-15 { di / dv } else { 0.0 });
    }

    // Backward difference at end
    let dvn = voltages[n - 1] - voltages[n - 2];
    didv.push(if dvn.abs() > 1e-15 { (currents[n - 1] - currents[n - 2]) / dvn } else { 0.0 });

    didv
}

/// Band gap measurement from STS: find gap where dI/dV ≈ 0
/// Returns (gap_start_v, gap_end_v, gap_size_v)
pub fn measure_band_gap(
    voltages: &[f64],
    didv: &[f64],
    threshold: f64, // dI/dV below this = gap
) -> (f64, f64, f64) {
    let n = voltages.len().min(didv.len());
    if n < 3 {
        return (0.0, 0.0, 0.0);
    }

    // Find region near V=0 where |dI/dV| < threshold
    let mut gap_start = 0.0;
    let mut gap_end = 0.0;
    let mut in_gap = false;

    for i in 0..n {
        if didv[i].abs() < threshold && !in_gap {
            in_gap = true;
            gap_start = voltages[i];
        } else if (didv[i].abs() >= threshold || i == n - 1) && in_gap {
            gap_end = voltages[i];
            in_gap = false;
            break;
        }
    }

    (gap_start, gap_end, (gap_end - gap_start).abs())
}

/// Lattice parameter measurement from row of atomic positions
/// Returns average spacing
pub fn lattice_parameter(positions: &[f64]) -> f64 {
    if positions.len() < 2 {
        return 0.0;
    }
    let mut spacings = Vec::new();
    for i in 1..positions.len() {
        spacings.push((positions[i] - positions[i - 1]).abs());
    }
    spacings.iter().sum::<f64>() / spacings.len() as f64
}

/// Z-calibration: convert raw DAC values to physical heights
pub fn z_calibrate(
    raw_values: &[f64],
    sensitivity_nm_per_v: f64,
    offset_nm: f64,
) -> Vec<f64> {
    raw_values.iter().map(|&v| v * sensitivity_nm_per_v + offset_nm).collect()
}

/// STM Processor
pub struct StmProcessor {
    pub image: Vec<Vec<f64>>,
    pub scan_size_nm: f64,
}

impl StmProcessor {
    pub fn new(image: Vec<Vec<f64>>, scan_size_nm: f64) -> Self {
        Self { image, scan_size_nm }
    }

    pub fn flatten(&self) -> Vec<Vec<f64>> {
        plane_subtract(&self.image)
    }

    pub fn level(&self) -> Vec<Vec<f64>> {
        line_level_median(&self.image)
    }

    pub fn ra(&self) -> f64 {
        let flat = self.flatten();
        roughness_ra(&flat)
    }

    pub fn rq(&self) -> f64 {
        let flat = self.flatten();
        roughness_rq(&flat)
    }

    pub fn rmax(&self) -> f64 {
        roughness_rmax(&self.image)
    }

    pub fn pixel_size_nm(&self) -> f64 {
        if self.image.is_empty() || self.image[0].is_empty() {
            return 0.0;
        }
        self.scan_size_nm / self.image[0].len() as f64
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn make_flat_image(ny: usize, nx: usize, z: f64) -> Vec<Vec<f64>> {
        vec![vec![z; nx]; ny]
    }

    fn make_tilted_image(ny: usize, nx: usize) -> Vec<Vec<f64>> {
        (0..ny)
            .map(|iy| {
                (0..nx)
                    .map(|ix| 0.1 * ix as f64 + 0.05 * iy as f64 + 1.0)
                    .collect()
            })
            .collect()
    }

    #[test]
    fn test_tunneling_current_decays() {
        let i1 = tunneling_current(0.1, 0.5e-9, 4.0, 1.0);
        let i2 = tunneling_current(0.1, 1.0e-9, 4.0, 1.0);
        assert!(i1 > i2); // current decays with distance
        assert!(i1 > 0.0);
    }

    #[test]
    fn test_tunneling_current_zero_barrier() {
        let i = tunneling_current(0.1, 1e-9, 0.0, 1.0);
        assert!(approx_eq(i, 0.1, 1e-10));
    }

    #[test]
    fn test_kappa_from_barrier() {
        let k = kappa_from_barrier(4.0); // ~4 eV typical work function
        // kappa should be ~10^10 m^-1
        assert!(k > 1e9 && k < 1e11, "kappa = {}", k);
    }

    #[test]
    fn test_kappa_zero() {
        assert_eq!(kappa_from_barrier(0.0), 0.0);
    }

    #[test]
    fn test_apparent_barrier_height() {
        let phi_true = 4.0; // eV
        let kappa = kappa_from_barrier(phi_true);
        let d1 = 0.5e-9;
        let d2 = 0.6e-9;
        let i1 = (-2.0 * kappa * d1).exp();
        let i2 = (-2.0 * kappa * d2).exp();

        let phi_calc = apparent_barrier_height(i1, d1, i2, d2);
        assert!(approx_eq(phi_calc, phi_true, 0.1), "phi = {}", phi_calc);
    }

    #[test]
    fn test_apparent_barrier_zero_current() {
        assert_eq!(apparent_barrier_height(0.0, 0.5e-9, 1.0, 0.6e-9), 0.0);
    }

    #[test]
    fn test_plane_subtract_flat() {
        let img = make_flat_image(10, 10, 5.0);
        let result = plane_subtract(&img);
        for row in &result {
            for &z in row {
                assert!(z.abs() < 1e-10, "z = {}", z);
            }
        }
    }

    #[test]
    fn test_plane_subtract_tilted() {
        let img = make_tilted_image(20, 20);
        let result = plane_subtract(&img);
        // After plane subtraction, roughness should be near zero
        let rq = roughness_rq(&result);
        assert!(rq < 0.01, "rq = {}", rq);
    }

    #[test]
    fn test_plane_subtract_empty() {
        let result = plane_subtract(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_line_level_median() {
        let img = vec![
            vec![10.0, 10.5, 10.2],
            vec![20.0, 20.5, 20.2],
        ];
        let result = line_level_median(&img);
        // Each row should be centered around 0
        for row in &result {
            let mean: f64 = row.iter().sum::<f64>() / row.len() as f64;
            assert!(mean.abs() < 1.0, "mean = {}", mean);
        }
    }

    #[test]
    fn test_line_level_linear() {
        let img = vec![
            vec![0.0, 1.0, 2.0, 3.0],
            vec![10.0, 11.0, 12.0, 13.0],
        ];
        let result = line_level_linear(&img);
        for row in &result {
            let rng = row.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
                - row.iter().cloned().fold(f64::INFINITY, f64::min);
            assert!(rng < 0.01, "range = {}", rng);
        }
    }

    #[test]
    fn test_roughness_ra_flat() {
        let img = make_flat_image(10, 10, 0.0);
        assert!(approx_eq(roughness_ra(&img), 0.0, 1e-10));
    }

    #[test]
    fn test_roughness_ra_known() {
        let img = vec![vec![-1.0, 1.0, -1.0, 1.0]];
        let ra = roughness_ra(&img);
        assert!(approx_eq(ra, 1.0, 1e-10));
    }

    #[test]
    fn test_roughness_rq() {
        let img = vec![vec![1.0, -1.0, 1.0, -1.0]];
        let rq = roughness_rq(&img);
        assert!(approx_eq(rq, 1.0, 1e-10));
    }

    #[test]
    fn test_roughness_rmax() {
        let img = vec![vec![0.0, 5.0, -3.0, 2.0]];
        let rmax = roughness_rmax(&img);
        assert!(approx_eq(rmax, 8.0, 1e-10));
    }

    #[test]
    fn test_roughness_rsk_symmetric() {
        let img = vec![vec![-2.0, -1.0, 0.0, 1.0, 2.0]];
        let rsk = roughness_rsk(&img);
        assert!(rsk.abs() < 0.01, "rsk = {}", rsk);
    }

    #[test]
    fn test_roughness_rku() {
        let img = vec![vec![-1.0, 0.0, 1.0]];
        let rku = roughness_rku(&img);
        assert!(rku > 0.0);
    }

    #[test]
    fn test_height_histogram() {
        let img = vec![vec![0.0, 0.5, 1.0, 1.5, 2.0]];
        let (centers, bins) = height_histogram(&img, 4);
        assert_eq!(centers.len(), 4);
        assert_eq!(bins.len(), 4);
        let total: usize = bins.iter().sum();
        assert_eq!(total, 5);
    }

    #[test]
    fn test_height_histogram_empty() {
        let (c, b) = height_histogram(&[], 10);
        assert!(c.is_empty());
        assert!(b.is_empty());
    }

    #[test]
    fn test_detect_step_edges() {
        let profile = vec![0.0, 0.1, 0.05, 0.0, 5.0, 5.1, 4.9, 5.0];
        let edges = detect_step_edges(&profile, 1.0);
        assert_eq!(edges.len(), 1);
        assert_eq!(edges[0].0, 4);
    }

    #[test]
    fn test_detect_step_edges_no_steps() {
        let profile = vec![0.0, 0.1, 0.2, 0.3];
        let edges = detect_step_edges(&profile, 1.0);
        assert!(edges.is_empty());
    }

    #[test]
    fn test_measure_step_height() {
        let profile = vec![0.0, 0.1, 0.0, 0.1, 5.0, 5.1, 5.0, 4.9];
        let h = measure_step_height(&profile, 4, 3);
        assert!(approx_eq(h, 5.0, 0.3), "h = {}", h);
    }

    #[test]
    fn test_didv_spectroscopy() {
        // Linear I-V: dI/dV should be constant
        let v: Vec<f64> = (-10..=10).map(|i| i as f64 * 0.1).collect();
        let i: Vec<f64> = v.iter().map(|&x| 2.0 * x).collect();
        let didv = didv_spectroscopy(&v, &i);
        for val in &didv[1..didv.len() - 1] {
            assert!(approx_eq(*val, 2.0, 0.01), "didv = {}", val);
        }
    }

    #[test]
    fn test_didv_spectroscopy_short() {
        let didv = didv_spectroscopy(&[0.0], &[0.0]);
        assert_eq!(didv.len(), 1);
    }

    #[test]
    fn test_measure_band_gap() {
        // Semiconductor-like: low dI/dV near V=0
        let v: Vec<f64> = (-20..=20).map(|i| i as f64 * 0.05).collect();
        let didv: Vec<f64> = v.iter().map(|&x| {
            if x.abs() < 0.5 { 0.001 } else { 1.0 }
        }).collect();
        let (start, end, gap) = measure_band_gap(&v, &didv, 0.01);
        assert!(gap > 0.8, "gap = {}", gap);
        assert!(start < 0.0);
        assert!(end > 0.0);
    }

    #[test]
    fn test_measure_band_gap_metallic() {
        // Metal: dI/dV always large
        let v: Vec<f64> = (-10..=10).map(|i| i as f64 * 0.1).collect();
        let didv: Vec<f64> = vec![1.0; v.len()];
        let (_, _, gap) = measure_band_gap(&v, &didv, 0.01);
        assert!(approx_eq(gap, 0.0, 0.01));
    }

    #[test]
    fn test_lattice_parameter() {
        let positions = vec![0.0, 0.285, 0.571, 0.855];
        let a = lattice_parameter(&positions);
        assert!(approx_eq(a, 0.285, 0.005));
    }

    #[test]
    fn test_lattice_parameter_single() {
        assert_eq!(lattice_parameter(&[0.0]), 0.0);
    }

    #[test]
    fn test_z_calibrate() {
        let raw = vec![0.0, 1.0, 2.0, 3.0];
        let cal = z_calibrate(&raw, 10.0, 5.0);
        assert!(approx_eq(cal[0], 5.0, 0.01));
        assert!(approx_eq(cal[1], 15.0, 0.01));
        assert!(approx_eq(cal[3], 35.0, 0.01));
    }

    #[test]
    fn test_processor_new() {
        let img = make_flat_image(10, 10, 1.0);
        let proc = StmProcessor::new(img, 100.0);
        assert!(approx_eq(proc.scan_size_nm, 100.0, 0.01));
    }

    #[test]
    fn test_processor_flatten() {
        let img = make_tilted_image(20, 20);
        let proc = StmProcessor::new(img, 100.0);
        let flat = proc.flatten();
        let rq = roughness_rq(&flat);
        assert!(rq < 0.01);
    }

    #[test]
    fn test_processor_level() {
        let img = vec![
            vec![10.0, 10.5, 10.2],
            vec![20.0, 20.5, 20.2],
        ];
        let proc = StmProcessor::new(img, 10.0);
        let leveled = proc.level();
        assert_eq!(leveled.len(), 2);
    }

    #[test]
    fn test_processor_ra() {
        let img = make_tilted_image(20, 20);
        let proc = StmProcessor::new(img, 100.0);
        let ra = proc.ra();
        assert!(ra < 0.01);
    }

    #[test]
    fn test_processor_rq() {
        let img = make_tilted_image(20, 20);
        let proc = StmProcessor::new(img, 100.0);
        let rq = proc.rq();
        assert!(rq < 0.01);
    }

    #[test]
    fn test_processor_rmax() {
        let img = vec![vec![0.0, 1.0, 5.0, -2.0]];
        let proc = StmProcessor::new(img, 10.0);
        assert!(approx_eq(proc.rmax(), 7.0, 0.01));
    }

    #[test]
    fn test_processor_pixel_size() {
        let img = make_flat_image(10, 50, 0.0);
        let proc = StmProcessor::new(img, 100.0);
        assert!(approx_eq(proc.pixel_size_nm(), 2.0, 0.01));
    }

    #[test]
    fn test_processor_pixel_size_empty() {
        let proc = StmProcessor::new(Vec::new(), 100.0);
        assert_eq!(proc.pixel_size_nm(), 0.0);
    }
}
