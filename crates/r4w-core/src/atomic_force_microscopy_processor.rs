//! Atomic Force Microscopy (AFM) Signal Processing
//!
//! Implements AFM signal processing for nanoscale surface imaging and force
//! measurement. Supports contact mode force curves (Hertz/DMT/JKR models),
//! tapping mode amplitude/phase extraction, FM-AFM frequency shift analysis
//! with Sader-Jarvis force reconstruction, and comprehensive image processing
//! (flattening, plane subtraction, roughness, PSD, Sobel edge detection).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::atomic_force_microscopy_processor::{
//!     AfmImage, CantileverConfig, ForceCurve, hertz_modulus,
//! };
//!
//! // Create a 4x4 test image
//! let data = vec![1.0, 2.0, 3.0, 4.0,
//!                 5.0, 6.0, 7.0, 8.0,
//!                 2.0, 3.0, 4.0, 5.0,
//!                 6.0, 7.0, 8.0, 9.0];
//! let mut img = AfmImage::new(data, 4, 4, 1.0);
//! let rq = img.rms_roughness();
//! assert!(rq > 0.0);
//! ```

use std::f64::consts::PI;

// ─── Cantilever Configuration ───────────────────────────────────────

/// AFM cantilever configuration parameters.
#[derive(Debug, Clone)]
pub struct CantileverConfig {
    /// Spring constant in N/m.
    pub spring_constant_nm: f64,
    /// Resonance frequency in Hz.
    pub resonance_freq_hz: f64,
    /// Quality factor (dimensionless).
    pub quality_factor: f64,
    /// Optical lever sensitivity in nm/V.
    pub sensitivity_nm_per_v: f64,
}

impl CantileverConfig {
    pub fn new(
        spring_constant_nm: f64,
        resonance_freq_hz: f64,
        quality_factor: f64,
        sensitivity_nm_per_v: f64,
    ) -> Self {
        Self {
            spring_constant_nm,
            resonance_freq_hz,
            quality_factor,
            sensitivity_nm_per_v,
        }
    }

    /// Typical tapping-mode cantilever (e.g., Bruker TESP).
    pub fn tapping_mode_preset() -> Self {
        Self {
            spring_constant_nm: 42.0,
            resonance_freq_hz: 320_000.0,
            quality_factor: 500.0,
            sensitivity_nm_per_v: 50.0,
        }
    }

    /// Typical contact-mode cantilever (e.g., Bruker SNL).
    pub fn contact_mode_preset() -> Self {
        Self {
            spring_constant_nm: 0.35,
            resonance_freq_hz: 65_000.0,
            quality_factor: 100.0,
            sensitivity_nm_per_v: 100.0,
        }
    }
}

// ─── AFM Image ──────────────────────────────────────────────────────

/// 2D AFM height image (units: nm).
#[derive(Debug, Clone)]
pub struct AfmImage {
    /// Height data in nm, row-major.
    pub data: Vec<f64>,
    /// Number of pixels in X (fast scan).
    pub pixels_x: usize,
    /// Number of pixels in Y (slow scan).
    pub pixels_y: usize,
    /// Physical scan size in micrometers.
    pub scan_size_um: f64,
}

impl AfmImage {
    pub fn new(data: Vec<f64>, pixels_x: usize, pixels_y: usize, scan_size_um: f64) -> Self {
        assert_eq!(data.len(), pixels_x * pixels_y);
        Self {
            data,
            pixels_x,
            pixels_y,
            scan_size_um,
        }
    }

    /// Get height value at (x, y).
    pub fn get(&self, x: usize, y: usize) -> f64 {
        self.data[y * self.pixels_x + x]
    }

    /// Set height value at (x, y).
    pub fn set(&mut self, x: usize, y: usize, val: f64) {
        self.data[y * self.pixels_x + x] = val;
    }

    /// Line-by-line flattening: subtract linear fit from each row.
    pub fn flatten_lines(&mut self) {
        let nx = self.pixels_x;
        for row in 0..self.pixels_y {
            let offset = row * nx;
            let row_data = &self.data[offset..offset + nx];

            // Least-squares linear fit: z = a*x + b
            let n = nx as f64;
            let mut sx = 0.0;
            let mut sy = 0.0;
            let mut sxx = 0.0;
            let mut sxy = 0.0;
            for i in 0..nx {
                let x = i as f64;
                let z = row_data[i];
                sx += x;
                sy += z;
                sxx += x * x;
                sxy += x * z;
            }
            let denom = n * sxx - sx * sx;
            if denom.abs() < 1e-30 {
                continue;
            }
            let a = (n * sxy - sx * sy) / denom;
            let b = (sy * sxx - sx * sxy) / denom;

            for i in 0..nx {
                self.data[offset + i] -= a * i as f64 + b;
            }
        }
    }

    /// Plane subtraction: fit z = a*x + b*y + c by least squares and subtract.
    pub fn subtract_plane(&mut self) {
        let nx = self.pixels_x;
        let ny = self.pixels_y;
        let n = (nx * ny) as f64;

        // Accumulate sums for 3-parameter least squares
        let mut sx = 0.0;
        let mut sy = 0.0;
        let mut sz = 0.0;
        let mut sxx = 0.0;
        let mut syy = 0.0;
        let mut sxy = 0.0;
        let mut sxz = 0.0;
        let mut syz = 0.0;

        for row in 0..ny {
            for col in 0..nx {
                let x = col as f64;
                let y = row as f64;
                let z = self.data[row * nx + col];
                sx += x;
                sy += y;
                sz += z;
                sxx += x * x;
                syy += y * y;
                sxy += x * y;
                sxz += x * z;
                syz += y * z;
            }
        }

        // Solve 3x3 normal equations:
        // [n   sx  sy ] [c]   [sz ]
        // [sx  sxx sxy] [a] = [sxz]
        // [sy  sxy syy] [b]   [syz]
        let mat = [
            [n, sx, sy],
            [sx, sxx, sxy],
            [sy, sxy, syy],
        ];
        let rhs = [sz, sxz, syz];

        if let Some(coeffs) = solve_3x3(&mat, &rhs) {
            let c = coeffs[0];
            let a = coeffs[1];
            let b = coeffs[2];

            for row in 0..ny {
                for col in 0..nx {
                    let x = col as f64;
                    let y = row as f64;
                    self.data[row * nx + col] -= a * x + b * y + c;
                }
            }
        }
    }

    /// RMS roughness Rq = sqrt(mean((z_i - z_mean)^2)).
    pub fn rms_roughness(&self) -> f64 {
        let n = self.data.len() as f64;
        let mean = self.data.iter().sum::<f64>() / n;
        let var = self.data.iter().map(|z| (z - mean).powi(2)).sum::<f64>() / n;
        var.sqrt()
    }

    /// Arithmetic average roughness Ra = mean(|z_i - z_mean|).
    pub fn ra_roughness(&self) -> f64 {
        let n = self.data.len() as f64;
        let mean = self.data.iter().sum::<f64>() / n;
        self.data.iter().map(|z| (z - mean).abs()).sum::<f64>() / n
    }

    /// Peak-to-valley: max(z) - min(z).
    pub fn peak_to_valley(&self) -> f64 {
        let max = self.data.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min = self.data.iter().cloned().fold(f64::INFINITY, f64::min);
        max - min
    }

    /// Height histogram with `num_bins` bins.
    pub fn height_histogram(&self, num_bins: usize) -> (Vec<f64>, Vec<usize>) {
        let min = self.data.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = self.data.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let range = max - min;
        if range < 1e-30 || num_bins == 0 {
            return (vec![min], vec![self.data.len()]);
        }
        let bin_width = range / num_bins as f64;
        let mut counts = vec![0usize; num_bins];
        let mut edges = Vec::with_capacity(num_bins);
        for i in 0..num_bins {
            edges.push(min + i as f64 * bin_width);
        }
        for &z in &self.data {
            let bin = ((z - min) / bin_width).floor() as usize;
            let bin = bin.min(num_bins - 1);
            counts[bin] += 1;
        }
        (edges, counts)
    }

    /// Polynomial background subtraction (nth order per line).
    pub fn polynomial_line_subtract(&mut self, order: usize) {
        let nx = self.pixels_x;
        for row in 0..self.pixels_y {
            let offset = row * nx;
            let row_data: Vec<f64> = self.data[offset..offset + nx].to_vec();
            let coeffs = polyfit(&row_data, order);
            for i in 0..nx {
                let x = i as f64;
                let mut val = 0.0;
                for (p, c) in coeffs.iter().enumerate() {
                    val += c * x.powi(p as i32);
                }
                self.data[offset + i] -= val;
            }
        }
    }

    /// Median line correction: align each row's median to the global median.
    pub fn median_line_correction(&mut self) {
        let nx = self.pixels_x;
        let ny = self.pixels_y;

        // Compute row medians
        let mut row_medians = Vec::with_capacity(ny);
        for row in 0..ny {
            let offset = row * nx;
            let mut sorted: Vec<f64> = self.data[offset..offset + nx].to_vec();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap());
            let med = if nx % 2 == 0 {
                (sorted[nx / 2 - 1] + sorted[nx / 2]) / 2.0
            } else {
                sorted[nx / 2]
            };
            row_medians.push(med);
        }

        // Global median of row medians
        let mut sorted_medians = row_medians.clone();
        sorted_medians.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let global_med = if ny % 2 == 0 {
            (sorted_medians[ny / 2 - 1] + sorted_medians[ny / 2]) / 2.0
        } else {
            sorted_medians[ny / 2]
        };

        // Adjust each row
        for row in 0..ny {
            let offset = row * nx;
            let shift = global_med - row_medians[row];
            for i in 0..nx {
                self.data[offset + i] += shift;
            }
        }
    }

    /// 2D Gaussian smoothing with given sigma (in pixels).
    pub fn gaussian_smooth(&mut self, sigma: f64) {
        let radius = (3.0 * sigma).ceil() as usize;
        let ksize = 2 * radius + 1;

        // Build 1D Gaussian kernel
        let mut kernel = vec![0.0; ksize];
        let mut sum = 0.0;
        for i in 0..ksize {
            let x = i as f64 - radius as f64;
            let val = (-x * x / (2.0 * sigma * sigma)).exp();
            kernel[i] = val;
            sum += val;
        }
        for k in &mut kernel {
            *k /= sum;
        }

        // Separable convolution: horizontal pass
        let nx = self.pixels_x;
        let ny = self.pixels_y;
        let mut temp = vec![0.0; nx * ny];

        for row in 0..ny {
            for col in 0..nx {
                let mut acc = 0.0;
                for ki in 0..ksize {
                    let sc = col as isize + ki as isize - radius as isize;
                    let sc = sc.max(0).min(nx as isize - 1) as usize;
                    acc += self.data[row * nx + sc] * kernel[ki];
                }
                temp[row * nx + col] = acc;
            }
        }

        // Vertical pass
        for row in 0..ny {
            for col in 0..nx {
                let mut acc = 0.0;
                for ki in 0..ksize {
                    let sr = row as isize + ki as isize - radius as isize;
                    let sr = sr.max(0).min(ny as isize - 1) as usize;
                    acc += temp[sr * nx + col] * kernel[ki];
                }
                self.data[row * nx + col] = acc;
            }
        }
    }

    /// Sobel edge detection. Returns gradient magnitude image.
    pub fn sobel_edge_detect(&self) -> AfmImage {
        let nx = self.pixels_x;
        let ny = self.pixels_y;
        let mut result = vec![0.0; nx * ny];

        // Sobel kernels
        // Gx:  -1 0 1       Gy:  -1 -2 -1
        //      -2 0 2             0  0  0
        //      -1 0 1             1  2  1

        for row in 1..ny.saturating_sub(1) {
            for col in 1..nx.saturating_sub(1) {
                let tl = self.get(col - 1, row - 1);
                let tc = self.get(col, row - 1);
                let tr = self.get(col + 1, row - 1);
                let ml = self.get(col - 1, row);
                let mr = self.get(col + 1, row);
                let bl = self.get(col - 1, row + 1);
                let bc = self.get(col, row + 1);
                let br = self.get(col + 1, row + 1);

                let gx = -tl + tr - 2.0 * ml + 2.0 * mr - bl + br;
                let gy = -tl - 2.0 * tc - tr + bl + 2.0 * bc + br;
                result[row * nx + col] = (gx * gx + gy * gy).sqrt();
            }
        }

        AfmImage::new(result, nx, ny, self.scan_size_um)
    }

    /// 1D Power Spectral Density along the fast-scan direction (averaged over rows).
    /// Returns (frequency in 1/um, PSD values).
    pub fn psd_1d(&self) -> (Vec<f64>, Vec<f64>) {
        let nx = self.pixels_x;
        let ny = self.pixels_y;
        let n_freq = nx / 2 + 1;
        let pixel_size_um = self.scan_size_um / nx as f64;
        let df = 1.0 / (nx as f64 * pixel_size_um);

        let mut psd = vec![0.0; n_freq];

        for row in 0..ny {
            let offset = row * nx;
            let row_data = &self.data[offset..offset + nx];

            // Apply Hann window and compute DFT
            let mut windowed = vec![0.0; nx];
            for i in 0..nx {
                let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / (nx as f64 - 1.0)).cos());
                windowed[i] = row_data[i] * w;
            }

            // DFT (real input, only positive frequencies)
            for k in 0..n_freq {
                let mut re = 0.0;
                let mut im = 0.0;
                for n in 0..nx {
                    let angle = 2.0 * PI * k as f64 * n as f64 / nx as f64;
                    re += windowed[n] * angle.cos();
                    im -= windowed[n] * angle.sin();
                }
                psd[k] += (re * re + im * im) / nx as f64;
            }
        }

        // Average over rows
        for p in &mut psd {
            *p /= ny as f64;
        }

        let freqs: Vec<f64> = (0..n_freq).map(|k| k as f64 * df).collect();
        (freqs, psd)
    }
}

// ─── Force Curves ───────────────────────────────────────────────────

/// Approach/retract force curve data.
#[derive(Debug, Clone)]
pub struct ForceCurve {
    /// Z-piezo displacement (nm), approach portion.
    pub z_approach: Vec<f64>,
    /// Deflection (nm), approach portion.
    pub defl_approach: Vec<f64>,
    /// Z-piezo displacement (nm), retract portion.
    pub z_retract: Vec<f64>,
    /// Deflection (nm), retract portion.
    pub defl_retract: Vec<f64>,
}

impl ForceCurve {
    pub fn new(
        z_approach: Vec<f64>,
        defl_approach: Vec<f64>,
        z_retract: Vec<f64>,
        defl_retract: Vec<f64>,
    ) -> Self {
        Self {
            z_approach,
            defl_approach,
            z_retract,
            defl_retract,
        }
    }

    /// Detect contact point on approach curve (snap-to-contact).
    /// Returns index of the contact point.
    ///
    /// Uses derivative threshold: the contact point is where the
    /// deflection changes most rapidly (largest negative derivative).
    pub fn contact_point_index(&self) -> usize {
        if self.defl_approach.len() < 3 {
            return 0;
        }
        let n = self.defl_approach.len();
        let mut max_deriv = f64::NEG_INFINITY;
        let mut idx = 0;
        for i in 1..n - 1 {
            let dz = self.z_approach[i + 1] - self.z_approach[i - 1];
            if dz.abs() < 1e-30 {
                continue;
            }
            let deriv = (self.defl_approach[i + 1] - self.defl_approach[i - 1]) / dz;
            // For approach (z decreasing, deflection goes negative at contact),
            // we look for the steepest negative slope = most negative deriv.
            // Actually for typical approach: z decreases, defl becomes negative at contact.
            // But sign conventions vary. Use magnitude of change.
            let abs_deriv = deriv.abs();
            if abs_deriv > max_deriv {
                max_deriv = abs_deriv;
                idx = i;
            }
        }
        idx
    }

    /// Adhesion force (nN) from pull-off during retract.
    /// Returns the most negative deflection during retract times spring constant.
    pub fn adhesion_force(&self, k: f64) -> f64 {
        let min_defl = self
            .defl_retract
            .iter()
            .cloned()
            .fold(f64::INFINITY, f64::min);
        // Adhesion force = k * |min deflection|
        // (negative deflection means attractive force)
        k * min_defl.abs()
    }

    /// Extract indentation and force arrays from the contact region of approach.
    /// `k` = spring constant in N/m. Returns (indentation_nm, force_nN).
    pub fn indentation_force(&self, k: f64, contact_idx: usize) -> (Vec<f64>, Vec<f64>) {
        let mut indentation = Vec::new();
        let mut force = Vec::new();

        let z_contact = self.z_approach[contact_idx];
        let d_contact = self.defl_approach[contact_idx];

        for i in contact_idx..self.defl_approach.len() {
            let z = self.z_approach[i];
            let d = self.defl_approach[i];

            // Indentation = (z - z_contact) - (d - d_contact)
            // positive when tip indents sample
            let delta = (z_contact - z) - (d - d_contact);
            if delta >= 0.0 {
                indentation.push(delta);
                // Force in nN: F = k * deflection
                force.push(k * (d - d_contact) * 1e-9 * 1e9); // nm * (N/m) -> nN conversion
            }
        }

        (indentation, force)
    }
}

/// Hertz model: F = (4/3) * E_eff * sqrt(R) * delta^(3/2)
/// for a sphere of radius R (nm) indenting a flat surface.
/// Given force F (nN) and indentation delta (nm), returns E_eff in Pa.
///
/// `tip_radius_nm`: tip radius in nm.
pub fn hertz_modulus(force_nn: f64, indentation_nm: f64, tip_radius_nm: f64) -> f64 {
    if indentation_nm <= 0.0 || force_nn <= 0.0 {
        return 0.0;
    }
    // Convert to SI: F in N, delta in m, R in m
    let f = force_nn * 1e-9;
    let delta = indentation_nm * 1e-9;
    let r = tip_radius_nm * 1e-9;

    // F = (4/3) * E * sqrt(R) * delta^(3/2)
    // E = F * 3 / (4 * sqrt(R) * delta^(3/2))
    let e = f * 3.0 / (4.0 * r.sqrt() * delta.powf(1.5));
    e
}

/// Hertz model: compute force (nN) from modulus (Pa), indentation (nm), tip radius (nm).
pub fn hertz_force(modulus_pa: f64, indentation_nm: f64, tip_radius_nm: f64) -> f64 {
    let delta = indentation_nm * 1e-9;
    let r = tip_radius_nm * 1e-9;
    let f = (4.0 / 3.0) * modulus_pa * r.sqrt() * delta.powf(1.5);
    f * 1e9 // Convert N to nN
}

/// DMT model pull-off force: F = 2*pi*R*W_adhesion.
/// `tip_radius_nm` in nm, `w_adhesion` in J/m^2. Returns force in nN.
pub fn dmt_pull_off(tip_radius_nm: f64, w_adhesion: f64) -> f64 {
    let r = tip_radius_nm * 1e-9;
    2.0 * PI * r * w_adhesion * 1e9
}

/// JKR model pull-off force: F = (3/2)*pi*R*W_adhesion.
/// `tip_radius_nm` in nm, `w_adhesion` in J/m^2. Returns force in nN.
pub fn jkr_pull_off(tip_radius_nm: f64, w_adhesion: f64) -> f64 {
    let r = tip_radius_nm * 1e-9;
    1.5 * PI * r * w_adhesion * 1e9
}

// ─── Tapping Mode (AM-AFM) ─────────────────────────────────────────

/// Tapping mode measurement results.
#[derive(Debug, Clone)]
pub struct TappingModeResult {
    /// Oscillation amplitude (nm).
    pub amplitude: f64,
    /// Phase shift (radians).
    pub phase: f64,
}

/// Lock-in detection at a given drive frequency.
/// Extracts amplitude and phase from a time-domain signal.
///
/// `signal`: cantilever deflection time series.
/// `sample_rate`: in Hz.
/// `drive_freq`: cantilever drive frequency in Hz.
pub fn lock_in_detect(signal: &[f64], sample_rate: f64, drive_freq: f64) -> TappingModeResult {
    let n = signal.len();
    if n == 0 {
        return TappingModeResult {
            amplitude: 0.0,
            phase: 0.0,
        };
    }

    let mut x_sum = 0.0;
    let mut y_sum = 0.0;

    for i in 0..n {
        let t = i as f64 / sample_rate;
        let angle = 2.0 * PI * drive_freq * t;
        x_sum += signal[i] * angle.cos();
        y_sum += signal[i] * angle.sin();
    }

    x_sum *= 2.0 / n as f64;
    y_sum *= 2.0 / n as f64;

    let amplitude = (x_sum * x_sum + y_sum * y_sum).sqrt();
    let phase = y_sum.atan2(x_sum);

    TappingModeResult { amplitude, phase }
}

/// Amplitude ratio (setpoint ratio): r_sp = A_sp / A0.
pub fn amplitude_ratio(a_sp: f64, a0: f64) -> f64 {
    if a0.abs() < 1e-30 {
        return 0.0;
    }
    a_sp / a0
}

/// Tip-sample energy dissipation per cycle (eV or aJ).
/// E_dis = (pi * k * A * A0) / Q * (sin(phi) - A/A0)
///
/// `k`: spring constant (N/m), `a`: amplitude (nm), `a0`: free amplitude (nm),
/// `q`: quality factor, `phi`: phase (radians).
/// Returns energy in aJ (attojoules).
pub fn energy_dissipation(k: f64, a: f64, a0: f64, q: f64, phi: f64) -> f64 {
    // Convert amplitudes from nm to m
    let a_m = a * 1e-9;
    let a0_m = a0 * 1e-9;

    let e_dis = (PI * k * a_m * a0_m) / q * (phi.sin() - a_m / a0_m);
    // Convert from J to aJ (1e18)
    e_dis * 1e18
}

// ─── FM-AFM (Frequency Modulation) ─────────────────────────────────

/// Frequency shift to force gradient.
/// delta_f ≈ -(f0 / (2*k)) * dF/dz
/// Returns force gradient dF/dz in N/m.
pub fn freq_shift_to_force_gradient(delta_f: f64, f0: f64, k: f64) -> f64 {
    // dF/dz = -2*k * delta_f / f0
    -2.0 * k * delta_f / f0
}

/// Sader-Jarvis method for force reconstruction from frequency shift vs distance.
///
/// Given arrays of distance `z` (nm, tip-sample separation) and frequency shift
/// `delta_f` (Hz), reconstructs the tip-sample force F(z).
///
/// `f0`: resonance frequency (Hz), `k`: spring constant (N/m), `a`: amplitude (nm).
///
/// Returns force in nN at each z point.
pub fn sader_jarvis_force(
    z: &[f64],
    delta_f: &[f64],
    f0: f64,
    k: f64,
    a_nm: f64,
) -> Vec<f64> {
    let n = z.len();
    if n < 3 {
        return vec![0.0; n];
    }

    let a = a_nm * 1e-9; // Convert amplitude to meters
    let mut force = vec![0.0; n];

    // Sader-Jarvis inversion formula (simplified):
    // F(z) = 2*k * integral from z to inf of
    //   [ (1 + a^(1/2)/(8*sqrt(pi*(t-z)))) * Omega(t)
    //     - a^(3/2)/sqrt(2*(t-z)) * dOmega/dt ] dt
    // where Omega(t) = delta_f(t)/f0

    for i in 0..n - 2 {
        let zi = z[i] * 1e-9; // Convert to meters
        let mut integral = 0.0;

        for j in (i + 1)..n - 1 {
            let tj = z[j] * 1e-9;
            let tj1 = z[j + 1] * 1e-9;
            let dt = (tj1 - tj).abs();
            if dt < 1e-30 {
                continue;
            }

            let sep = tj - zi;
            if sep <= 0.0 {
                continue;
            }

            let omega_j = delta_f[j] / f0;
            let omega_j1 = delta_f[j + 1] / f0;
            let d_omega = (omega_j1 - omega_j) / dt;

            let term1 = (1.0 + a.sqrt() / (8.0 * (PI * sep).sqrt())) * omega_j;
            let term2 = a.powf(1.5) / (2.0 * sep).sqrt() * d_omega;

            integral += (term1 - term2) * dt;
        }

        force[i] = 2.0 * k * integral * 1e9; // Convert to nN
    }

    // Extrapolate last two points
    if n >= 3 {
        force[n - 2] = force[n - 3];
        force[n - 1] = force[n - 3];
    }

    force
}

// ─── Calibration ────────────────────────────────────────────────────

/// Z-piezo calibration: compute calibration factor from known step height.
/// `measured_height`: measured step in raw units.
/// `known_height_nm`: true step height in nm.
/// Returns calibration factor (nm per raw unit).
pub fn z_piezo_calibration(measured_height: f64, known_height_nm: f64) -> f64 {
    if measured_height.abs() < 1e-30 {
        return 1.0;
    }
    known_height_nm / measured_height
}

/// Sensitivity calibration from force curve on hard surface.
/// On a hard surface, deflection = z displacement in the contact region.
/// Sensitivity = slope of deflection vs z (nm/V or nm/nm).
///
/// `z`: z-piezo positions, `defl`: deflection values.
/// Returns sensitivity (slope) via linear regression.
pub fn sensitivity_calibration(z: &[f64], defl: &[f64]) -> f64 {
    let n = z.len().min(defl.len());
    if n < 2 {
        return 1.0;
    }

    let nf = n as f64;
    let mut sx = 0.0;
    let mut sy = 0.0;
    let mut sxx = 0.0;
    let mut sxy = 0.0;
    for i in 0..n {
        sx += z[i];
        sy += defl[i];
        sxx += z[i] * z[i];
        sxy += z[i] * defl[i];
    }
    let denom = nf * sxx - sx * sx;
    if denom.abs() < 1e-30 {
        return 1.0;
    }
    (nf * sxy - sx * sy) / denom
}

/// Spring constant from thermal noise (equipartition theorem, simplified Sader method).
/// k = k_B * T / <z^2>
///
/// `variance_nm2`: variance of cantilever deflection in nm^2.
/// `temperature_k`: temperature in Kelvin.
/// Returns spring constant in N/m.
pub fn spring_constant_thermal(variance_nm2: f64, temperature_k: f64) -> f64 {
    const K_B: f64 = 1.380649e-23; // Boltzmann constant, J/K
    if variance_nm2 <= 0.0 {
        return 0.0;
    }
    let variance_m2 = variance_nm2 * 1e-18; // nm^2 -> m^2
    K_B * temperature_k / variance_m2
}

// ─── Creep and Hysteresis ───────────────────────────────────────────

/// Logarithmic creep model: z_creep(t) = z0 * (1 + beta * ln(1 + t/tau)).
///
/// `z0`: initial displacement (nm).
/// `beta`: creep coefficient (dimensionless, typical 0.01-0.1).
/// `tau`: time constant (s).
/// `times`: array of time points (s).
/// Returns predicted creep displacement at each time.
pub fn logarithmic_creep(z0: f64, beta: f64, tau: f64, times: &[f64]) -> Vec<f64> {
    times
        .iter()
        .map(|&t| z0 * (1.0 + beta * (1.0 + t / tau).ln()))
        .collect()
}

/// Forward/backward scan hysteresis correction.
/// Aligns forward (trace) and backward (retrace) scan lines.
///
/// `forward`: forward scan heights.
/// `backward`: backward scan heights (same direction order).
/// Returns corrected forward scan (shifts forward to match backward).
pub fn hysteresis_correction(forward: &[f64], backward: &[f64]) -> Vec<f64> {
    let n = forward.len().min(backward.len());
    if n == 0 {
        return vec![];
    }

    // Compute average shift between forward and backward
    let shift: f64 = (0..n).map(|i| backward[i] - forward[i]).sum::<f64>() / n as f64;

    // Apply half the shift to forward to center both
    forward.iter().map(|&v| v + shift / 2.0).collect()
}

// ─── Helper Functions ───────────────────────────────────────────────

/// Solve 3x3 linear system Ax = b using Cramer's rule.
fn solve_3x3(a: &[[f64; 3]; 3], b: &[f64; 3]) -> Option<[f64; 3]> {
    let det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if det.abs() < 1e-30 {
        return None;
    }

    let x0 = (b[0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
        - a[0][1] * (b[1] * a[2][2] - a[1][2] * b[2])
        + a[0][2] * (b[1] * a[2][1] - a[1][1] * b[2]))
        / det;

    let x1 = (a[0][0] * (b[1] * a[2][2] - a[1][2] * b[2])
        - b[0] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
        + a[0][2] * (a[1][0] * b[2] - b[1] * a[2][0]))
        / det;

    let x2 = (a[0][0] * (a[1][1] * b[2] - b[1] * a[2][1])
        - a[0][1] * (a[1][0] * b[2] - b[1] * a[2][0])
        + b[0] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
        / det;

    Some([x0, x1, x2])
}

/// Least-squares polynomial fit of order `order` for evenly sampled data.
/// Returns coefficients [c0, c1, c2, ...] where z = c0 + c1*x + c2*x^2 + ...
fn polyfit(data: &[f64], order: usize) -> Vec<f64> {
    let n = data.len();
    let order = order.min(n.saturating_sub(1));
    let m = order + 1;

    // Build normal equations: A^T A x = A^T b
    // where A is the Vandermonde matrix
    let mut ata = vec![0.0; m * m];
    let mut atb = vec![0.0; m];

    for i in 0..n {
        let x = i as f64;
        let mut xp = vec![1.0; m];
        for j in 1..m {
            xp[j] = xp[j - 1] * x;
        }

        for r in 0..m {
            for c in 0..m {
                ata[r * m + c] += xp[r] * xp[c];
            }
            atb[r] += xp[r] * data[i];
        }
    }

    // Solve via Gaussian elimination with partial pivoting
    gauss_solve(&mut ata, &mut atb, m)
}

/// Gaussian elimination with partial pivoting.
fn gauss_solve(a: &mut [f64], b: &mut [f64], n: usize) -> Vec<f64> {
    // Forward elimination
    for col in 0..n {
        // Partial pivoting
        let mut max_row = col;
        let mut max_val = a[col * n + col].abs();
        for row in (col + 1)..n {
            let val = a[row * n + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        if max_row != col {
            for c in 0..n {
                let tmp = a[col * n + c];
                a[col * n + c] = a[max_row * n + c];
                a[max_row * n + c] = tmp;
            }
            let tmp = b[col];
            b[col] = b[max_row];
            b[max_row] = tmp;
        }

        let pivot = a[col * n + col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for c in col..n {
                a[row * n + c] -= factor * a[col * n + c];
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = b[i];
        for j in (i + 1)..n {
            sum -= a[i * n + j] * x[j];
        }
        let diag = a[i * n + i];
        if diag.abs() < 1e-30 {
            x[i] = 0.0;
        } else {
            x[i] = sum / diag;
        }
    }

    x
}

// ─── Tests ──────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ── CantileverConfig ──

    #[test]
    fn test_cantilever_config_new() {
        let c = CantileverConfig::new(42.0, 320_000.0, 500.0, 50.0);
        assert_eq!(c.spring_constant_nm, 42.0);
        assert_eq!(c.resonance_freq_hz, 320_000.0);
        assert_eq!(c.quality_factor, 500.0);
        assert_eq!(c.sensitivity_nm_per_v, 50.0);
    }

    #[test]
    fn test_cantilever_tapping_preset() {
        let c = CantileverConfig::tapping_mode_preset();
        assert!(c.spring_constant_nm > 10.0);
        assert!(c.resonance_freq_hz > 100_000.0);
        assert!(c.quality_factor > 100.0);
    }

    #[test]
    fn test_cantilever_contact_preset() {
        let c = CantileverConfig::contact_mode_preset();
        assert!(c.spring_constant_nm < 1.0);
        assert!(c.resonance_freq_hz > 10_000.0);
    }

    // ── AfmImage basic ──

    #[test]
    fn test_afm_image_new() {
        let data = vec![1.0, 2.0, 3.0, 4.0];
        let img = AfmImage::new(data, 2, 2, 1.0);
        assert_eq!(img.pixels_x, 2);
        assert_eq!(img.pixels_y, 2);
        assert_eq!(img.get(0, 0), 1.0);
        assert_eq!(img.get(1, 1), 4.0);
    }

    #[test]
    fn test_afm_image_set() {
        let data = vec![0.0; 4];
        let mut img = AfmImage::new(data, 2, 2, 1.0);
        img.set(1, 0, 5.5);
        assert_eq!(img.get(1, 0), 5.5);
    }

    // ── Roughness ──

    #[test]
    fn test_rms_roughness_flat() {
        let data = vec![5.0; 16];
        let img = AfmImage::new(data, 4, 4, 1.0);
        assert!(img.rms_roughness() < TOL);
    }

    #[test]
    fn test_rms_roughness_known() {
        // Values: [1, -1, 1, -1], mean=0, Rq = 1.0
        let data = vec![1.0, -1.0, 1.0, -1.0];
        let img = AfmImage::new(data, 2, 2, 1.0);
        assert!(approx_eq(img.rms_roughness(), 1.0, TOL));
    }

    #[test]
    fn test_ra_roughness_known() {
        // Values: [2, -2, 2, -2], mean=0, Ra = 2.0
        let data = vec![2.0, -2.0, 2.0, -2.0];
        let img = AfmImage::new(data, 2, 2, 1.0);
        assert!(approx_eq(img.ra_roughness(), 2.0, TOL));
    }

    #[test]
    fn test_peak_to_valley() {
        let data = vec![1.0, 5.0, -3.0, 2.0];
        let img = AfmImage::new(data, 2, 2, 1.0);
        assert!(approx_eq(img.peak_to_valley(), 8.0, TOL));
    }

    // ── Line flattening ──

    #[test]
    fn test_flatten_lines_removes_slope() {
        // Row with a linear ramp should become ~flat
        let data = vec![0.0, 1.0, 2.0, 3.0, 10.0, 11.0, 12.0, 13.0];
        let mut img = AfmImage::new(data, 4, 2, 1.0);
        img.flatten_lines();
        // After flattening, each row should have ~zero mean and ~zero slope
        for row in 0..2 {
            let mut sum = 0.0;
            for col in 0..4 {
                sum += img.get(col, row);
            }
            assert!(sum.abs() < 1e-10, "Row {} sum = {}", row, sum);
        }
    }

    #[test]
    fn test_flatten_lines_preserves_variation() {
        // Non-linear variation should remain
        let data = vec![0.0, 1.0, 0.0, 1.0]; // 1 row of 4
        let mut img = AfmImage::new(data, 4, 1, 1.0);
        img.flatten_lines();
        // Should still have some variation
        let rq = img.rms_roughness();
        assert!(rq > 0.0);
    }

    // ── Plane subtraction ──

    #[test]
    fn test_subtract_plane_tilted() {
        let nx = 4;
        let ny = 4;
        let mut data = vec![0.0; nx * ny];
        // z = 2*x + 3*y + 5
        for row in 0..ny {
            for col in 0..nx {
                data[row * nx + col] = 2.0 * col as f64 + 3.0 * row as f64 + 5.0;
            }
        }
        let mut img = AfmImage::new(data, nx, ny, 1.0);
        img.subtract_plane();
        // After subtraction, all values should be ~0
        for &z in &img.data {
            assert!(z.abs() < 1e-8, "z = {}", z);
        }
    }

    // ── Histogram ──

    #[test]
    fn test_height_histogram() {
        let data = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0];
        let img = AfmImage::new(data, 3, 3, 1.0);
        let (edges, counts) = img.height_histogram(4);
        assert_eq!(edges.len(), 4);
        assert_eq!(counts.len(), 4);
        assert_eq!(counts.iter().sum::<usize>(), 9);
    }

    // ── Polynomial line subtraction ──

    #[test]
    fn test_polynomial_line_subtract_linear() {
        // Linear background should be fully removed
        let data = vec![1.0, 3.0, 5.0, 7.0]; // y = 2x + 1
        let mut img = AfmImage::new(data, 4, 1, 1.0);
        img.polynomial_line_subtract(1);
        for &z in &img.data {
            assert!(z.abs() < 1e-8, "z = {}", z);
        }
    }

    #[test]
    fn test_polynomial_line_subtract_quadratic() {
        // Quadratic background: z = x^2
        let data = vec![0.0, 1.0, 4.0, 9.0, 16.0];
        let mut img = AfmImage::new(data, 5, 1, 1.0);
        img.polynomial_line_subtract(2);
        for &z in &img.data {
            assert!(z.abs() < 1e-6, "z = {}", z);
        }
    }

    // ── Median line correction ──

    #[test]
    fn test_median_line_correction() {
        // Two rows with different offsets
        let data = vec![10.0, 11.0, 12.0, 20.0, 21.0, 22.0];
        let mut img = AfmImage::new(data, 3, 2, 1.0);
        img.median_line_correction();
        // Row medians should now be equal
        let med0 = {
            let mut r: Vec<f64> = (0..3).map(|c| img.get(c, 0)).collect();
            r.sort_by(|a, b| a.partial_cmp(b).unwrap());
            r[1]
        };
        let med1 = {
            let mut r: Vec<f64> = (0..3).map(|c| img.get(c, 1)).collect();
            r.sort_by(|a, b| a.partial_cmp(b).unwrap());
            r[1]
        };
        assert!(approx_eq(med0, med1, 1e-10));
    }

    // ── Gaussian smooth ──

    #[test]
    fn test_gaussian_smooth_preserves_uniform() {
        let data = vec![5.0; 16];
        let mut img = AfmImage::new(data, 4, 4, 1.0);
        img.gaussian_smooth(1.0);
        for &z in &img.data {
            assert!(approx_eq(z, 5.0, 1e-10));
        }
    }

    #[test]
    fn test_gaussian_smooth_reduces_noise() {
        // Spike in the middle should be smoothed
        let mut data = vec![0.0; 25];
        data[12] = 100.0; // center of 5x5
        let mut img = AfmImage::new(data, 5, 5, 1.0);
        let peak_before = img.get(2, 2);
        img.gaussian_smooth(1.0);
        let peak_after = img.get(2, 2);
        assert!(peak_after < peak_before);
    }

    // ── Sobel edge detection ──

    #[test]
    fn test_sobel_edge_step() {
        // Left half = 0, right half = 10 → edge in middle
        let mut data = vec![0.0; 36];
        for row in 0..6 {
            for col in 3..6 {
                data[row * 6 + col] = 10.0;
            }
        }
        let img = AfmImage::new(data, 6, 6, 1.0);
        let edges = img.sobel_edge_detect();
        // Edge pixels around col=2-3 should have high gradient
        let edge_val = edges.get(3, 3);
        let flat_val = edges.get(0, 3);
        assert!(edge_val > flat_val);
    }

    // ── PSD ──

    #[test]
    fn test_psd_1d_length() {
        let data = vec![0.0; 32];
        let img = AfmImage::new(data, 8, 4, 2.0);
        let (freqs, psd) = img.psd_1d();
        assert_eq!(freqs.len(), 5); // nx/2 + 1
        assert_eq!(psd.len(), 5);
    }

    // ── Force curves ──

    #[test]
    fn test_force_curve_contact_point() {
        // Simulate approach: far away (small defl), then snap to contact
        let z: Vec<f64> = (0..20).map(|i| 100.0 - 5.0 * i as f64).collect();
        let mut defl = vec![0.0; 20];
        // Snap at index 10
        for i in 10..20 {
            defl[i] = -5.0 * (i as f64 - 10.0);
        }
        let fc = ForceCurve::new(z, defl, vec![], vec![]);
        let cp = fc.contact_point_index();
        // Contact point should be near index 10
        assert!(cp >= 9 && cp <= 12, "Contact point at {}", cp);
    }

    #[test]
    fn test_adhesion_force() {
        let fc = ForceCurve::new(
            vec![],
            vec![],
            vec![0.0, -10.0, -20.0, -5.0, 0.0],
            vec![0.0, -5.0, -15.0, -3.0, 0.0],
        );
        let k = 0.5; // N/m
        let f_adh = fc.adhesion_force(k);
        assert!(approx_eq(f_adh, 0.5 * 15.0, TOL));
    }

    // ── Hertz model ──

    #[test]
    fn test_hertz_force_and_modulus_roundtrip() {
        let e = 1e9; // 1 GPa
        let delta = 5.0; // nm
        let r = 20.0; // nm
        let f = hertz_force(e, delta, r);
        assert!(f > 0.0);
        let e_recovered = hertz_modulus(f, delta, r);
        assert!(approx_eq(e_recovered, e, e * 1e-6));
    }

    #[test]
    fn test_hertz_modulus_zero_indentation() {
        assert_eq!(hertz_modulus(1.0, 0.0, 20.0), 0.0);
    }

    #[test]
    fn test_hertz_force_scaling() {
        let e = 1e9;
        let r = 20.0;
        let f1 = hertz_force(e, 1.0, r);
        let f2 = hertz_force(e, 2.0, r);
        // F ~ delta^(3/2), so f2/f1 ~ 2^(3/2) = 2.828...
        let ratio = f2 / f1;
        assert!(approx_eq(ratio, 2.0_f64.powf(1.5), 1e-6));
    }

    // ── DMT/JKR models ──

    #[test]
    fn test_dmt_pull_off() {
        let r = 20.0; // nm
        let w = 0.05; // J/m^2
        let f = dmt_pull_off(r, w);
        // F = 2*pi*R*W = 2*pi*20e-9*0.05 = 6.283e-9 N = 6.283 nN
        let expected = 2.0 * PI * 20e-9 * 0.05 * 1e9;
        assert!(approx_eq(f, expected, 1e-3));
    }

    #[test]
    fn test_jkr_pull_off() {
        let r = 20.0;
        let w = 0.05;
        let f = jkr_pull_off(r, w);
        let expected = 1.5 * PI * 20e-9 * 0.05 * 1e9;
        assert!(approx_eq(f, expected, 1e-3));
    }

    #[test]
    fn test_dmt_greater_than_jkr() {
        // DMT pull-off (2*pi*R*W) > JKR pull-off (3/2*pi*R*W)
        let r = 30.0;
        let w = 0.03;
        assert!(dmt_pull_off(r, w) > jkr_pull_off(r, w));
    }

    // ── Tapping mode ──

    #[test]
    fn test_lock_in_detect_pure_sine() {
        let fs = 10000.0;
        let freq = 100.0;
        let amp = 5.0;
        let n = 1000;
        let signal: Vec<f64> = (0..n)
            .map(|i| amp * (2.0 * PI * freq * i as f64 / fs).cos())
            .collect();
        let result = lock_in_detect(&signal, fs, freq);
        assert!(approx_eq(result.amplitude, amp, 0.1));
        assert!(result.phase.abs() < 0.1); // cosine → phase near 0
    }

    #[test]
    fn test_lock_in_detect_sine_with_phase() {
        let fs = 10000.0;
        let freq = 100.0;
        let amp = 3.0;
        let phi = PI / 4.0;
        let n = 1000;
        let signal: Vec<f64> = (0..n)
            .map(|i| amp * (2.0 * PI * freq * i as f64 / fs + phi).cos())
            .collect();
        let result = lock_in_detect(&signal, fs, freq);
        assert!(approx_eq(result.amplitude, amp, 0.1));
    }

    #[test]
    fn test_amplitude_ratio() {
        assert!(approx_eq(amplitude_ratio(30.0, 50.0), 0.6, TOL));
        assert!(approx_eq(amplitude_ratio(0.0, 50.0), 0.0, TOL));
        assert!(approx_eq(amplitude_ratio(50.0, 0.0), 0.0, TOL));
    }

    #[test]
    fn test_energy_dissipation_zero_at_free() {
        // When A = A0 and phi = pi/2 (free oscillation), sin(pi/2)=1, A/A0=1 → E=0
        let e = energy_dissipation(42.0, 50.0, 50.0, 500.0, PI / 2.0);
        assert!(e.abs() < 1e-6);
    }

    // ── FM-AFM ──

    #[test]
    fn test_freq_shift_to_force_gradient() {
        // delta_f = -100 Hz, f0 = 300 kHz, k = 40 N/m
        let fg = freq_shift_to_force_gradient(-100.0, 300_000.0, 40.0);
        // dF/dz = -2*40*(-100)/300000 = 0.02667 N/m
        let expected = -2.0 * 40.0 * (-100.0) / 300_000.0;
        assert!(approx_eq(fg, expected, 1e-8));
    }

    #[test]
    fn test_sader_jarvis_force_length() {
        let z: Vec<f64> = (0..50).map(|i| 0.5 + 0.1 * i as f64).collect();
        let delta_f: Vec<f64> = z.iter().map(|&zi| -10.0 / (zi * zi)).collect();
        let force = sader_jarvis_force(&z, &delta_f, 300_000.0, 40.0, 10.0);
        assert_eq!(force.len(), 50);
    }

    #[test]
    fn test_sader_jarvis_force_attractive() {
        // Negative freq shift → attractive interaction → force should be negative (or decrease with z)
        let z: Vec<f64> = (0..30).map(|i| 1.0 + 0.5 * i as f64).collect();
        let delta_f: Vec<f64> = z.iter().map(|&zi| -50.0 / (zi * zi)).collect();
        let force = sader_jarvis_force(&z, &delta_f, 300_000.0, 40.0, 10.0);
        // Force at close range should be more negative than far
        assert!(force[0].abs() > force[20].abs());
    }

    // ── Calibration ──

    #[test]
    fn test_z_piezo_calibration() {
        let cal = z_piezo_calibration(50.0, 100.0);
        assert!(approx_eq(cal, 2.0, TOL));
    }

    #[test]
    fn test_sensitivity_calibration_linear() {
        let z = vec![0.0, 1.0, 2.0, 3.0, 4.0];
        let defl = vec![0.0, 0.5, 1.0, 1.5, 2.0];
        let sens = sensitivity_calibration(&z, &defl);
        assert!(approx_eq(sens, 0.5, 1e-10));
    }

    #[test]
    fn test_spring_constant_thermal() {
        // At 300K, variance = 0.01 nm^2 → k = kB*T / var
        let k = spring_constant_thermal(0.01, 300.0);
        let expected = 1.380649e-23 * 300.0 / (0.01 * 1e-18);
        assert!(approx_eq(k, expected, expected * 1e-6));
    }

    #[test]
    fn test_spring_constant_thermal_zero_variance() {
        assert_eq!(spring_constant_thermal(0.0, 300.0), 0.0);
    }

    // ── Creep and hysteresis ──

    #[test]
    fn test_logarithmic_creep() {
        let times = vec![0.0, 1.0, 10.0, 100.0];
        let result = logarithmic_creep(100.0, 0.05, 1.0, &times);
        assert_eq!(result.len(), 4);
        // At t=0: z = 100 * (1 + 0.05 * ln(1)) = 100
        assert!(approx_eq(result[0], 100.0, TOL));
        // Creep increases with time
        assert!(result[1] > result[0]);
        assert!(result[2] > result[1]);
        assert!(result[3] > result[2]);
    }

    #[test]
    fn test_hysteresis_correction() {
        let forward = vec![1.0, 2.0, 3.0, 4.0];
        let backward = vec![3.0, 4.0, 5.0, 6.0];
        let corrected = hysteresis_correction(&forward, &backward);
        // Average shift = 2.0, half shift = 1.0
        assert!(approx_eq(corrected[0], 2.0, TOL));
        assert!(approx_eq(corrected[3], 5.0, TOL));
    }

    #[test]
    fn test_hysteresis_correction_no_shift() {
        let data = vec![1.0, 2.0, 3.0];
        let corrected = hysteresis_correction(&data, &data);
        for (a, b) in corrected.iter().zip(data.iter()) {
            assert!(approx_eq(*a, *b, TOL));
        }
    }

    // ── Force curve indentation ──

    #[test]
    fn test_indentation_force_extraction() {
        let z: Vec<f64> = (0..20).map(|i| 100.0 - 5.0 * i as f64).collect();
        let mut defl = vec![0.0; 20];
        for i in 10..20 {
            defl[i] = -3.0 * (i as f64 - 10.0);
        }
        let fc = ForceCurve::new(z, defl, vec![], vec![]);
        let (indent, force) = fc.indentation_force(0.5, 10);
        assert!(!indent.is_empty());
        assert_eq!(indent.len(), force.len());
    }

    // ── Edge cases ──

    #[test]
    fn test_lock_in_empty_signal() {
        let result = lock_in_detect(&[], 1000.0, 100.0);
        assert_eq!(result.amplitude, 0.0);
    }

    #[test]
    fn test_polyfit_constant() {
        let data = vec![7.0; 10];
        let coeffs = polyfit(&data, 0);
        assert!(approx_eq(coeffs[0], 7.0, 1e-6));
    }

    #[test]
    fn test_solve_3x3_identity() {
        let a = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];
        let b = [3.0, 5.0, 7.0];
        let x = solve_3x3(&a, &b).unwrap();
        assert!(approx_eq(x[0], 3.0, TOL));
        assert!(approx_eq(x[1], 5.0, TOL));
        assert!(approx_eq(x[2], 7.0, TOL));
    }
}
