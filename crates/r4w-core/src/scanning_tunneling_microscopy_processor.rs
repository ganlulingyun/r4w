// trace:FR-STM | ai:claude
//! # Scanning Tunneling Microscopy Processor
//!
//! This module implements STM image and spectroscopy data processing including
//! topographic image analysis, scanning tunneling spectroscopy (STS) dI/dV curves,
//! and surface structure characterization at atomic resolution.
//!
//! ## Physics Background
//!
//! The scanning tunneling microscope exploits quantum mechanical tunneling:
//! a sharp metallic tip is brought within ~1 nm of a conducting surface, and
//! a bias voltage V produces a tunneling current I that depends exponentially
//! on tip-sample distance d:
//!
//!   I ~ V * exp(-2 * kappa * d)
//!
//! where kappa = sqrt(2*m*phi) / hbar, with phi the effective barrier height
//! (~4 eV for typical metals) and m the electron mass.
//!
//! The dI/dV signal is proportional to the local density of states (LDOS),
//! enabling electronic structure mapping at atomic resolution.

use std::f64::consts::PI;

// Physical constants
const ELECTRON_MASS_KG: f64 = 9.10938e-31;
const HBAR_JS: f64 = 1.054571817e-34; // reduced Planck constant (J*s)
const EV_TO_J: f64 = 1.602176634e-19; // eV to Joules

/// A 2D topographic height image z(x,y) from STM scanning.
#[derive(Clone, Debug)]
pub struct StmImage {
    pub width: usize,
    pub height: usize,
    pub data: Vec<f64>,
    pub pixel_size_nm: f64,
}

impl StmImage {
    /// Create a new STM image.
    ///
    /// `data` is row-major: data[y * width + x] = z(x, y) in nm.
    pub fn new(width: usize, height: usize, data: Vec<f64>, pixel_size_nm: f64) -> Self {
        assert_eq!(data.len(), width * height, "Data length must match width*height");
        assert!(pixel_size_nm > 0.0, "Pixel size must be positive");
        Self { width, height, data, pixel_size_nm }
    }

    /// Get height at pixel (x, y).
    pub fn height_at(&self, x: usize, y: usize) -> f64 {
        assert!(x < self.width && y < self.height, "Index out of bounds");
        self.data[y * self.width + x]
    }

    /// Minimum height in the image.
    pub fn min_height(&self) -> f64 {
        self.data.iter().cloned().fold(f64::INFINITY, f64::min)
    }

    /// Maximum height in the image.
    pub fn max_height(&self) -> f64 {
        self.data.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
    }

    /// Mean height of the image.
    pub fn mean_height(&self) -> f64 {
        let sum: f64 = self.data.iter().sum();
        sum / self.data.len() as f64
    }

    /// RMS roughness Sq = sqrt(mean((z - z_mean)^2)).
    pub fn rms_roughness(&self) -> f64 {
        let mean = self.mean_height();
        let n = self.data.len() as f64;
        let sum_sq: f64 = self.data.iter().map(|&z| (z - mean) * (z - mean)).sum();
        (sum_sq / n).sqrt()
    }

    /// Image size in nm along x.
    pub fn size_x_nm(&self) -> f64 {
        self.width as f64 * self.pixel_size_nm
    }

    /// Image size in nm along y.
    pub fn size_y_nm(&self) -> f64 {
        self.height as f64 * self.pixel_size_nm
    }
}

/// Image flattening: remove tilt and bow artifacts from STM images.
pub struct ImageFlattening;

impl ImageFlattening {
    /// Subtract a least-squares fitted plane z = a*x + b*y + c from the image.
    pub fn plane_subtraction(image: &StmImage) -> StmImage {
        let n = image.data.len() as f64;
        let w = image.width;
        let h = image.height;

        // Build sums for least-squares: z = a*x + b*y + c
        let mut sx = 0.0_f64;
        let mut sy = 0.0_f64;
        let mut sz = 0.0_f64;
        let mut sxx = 0.0_f64;
        let mut syy = 0.0_f64;
        let mut sxy = 0.0_f64;
        let mut sxz = 0.0_f64;
        let mut syz = 0.0_f64;

        for iy in 0..h {
            for ix in 0..w {
                let x = ix as f64;
                let y = iy as f64;
                let z = image.data[iy * w + ix];
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

        // Solve 3x3 system: [sxx sxy sx; sxy syy sy; sx sy n] * [a;b;c] = [sxz; syz; sz]
        // Using Cramer's rule
        let det = sxx * (syy * n - sy * sy)
                - sxy * (sxy * n - sy * sx)
                + sx * (sxy * sy - syy * sx);

        if det.abs() < 1e-30 {
            return image.clone();
        }

        let a = (sxz * (syy * n - sy * sy) - sxy * (syz * n - sy * sz) + sx * (syz * sy - syy * sz)) / det;
        let b = (sxx * (syz * n - sy * sz) - sxz * (sxy * n - sy * sx) + sx * (sxy * sz - syz * sx)) / det;
        let c = (sxx * (syy * sz - syz * sy) - sxy * (sxy * sz - syz * sx) + sxz * (sxy * sy - syy * sx)) / det;

        let mut new_data = vec![0.0; w * h];
        for iy in 0..h {
            for ix in 0..w {
                let plane_val = a * ix as f64 + b * iy as f64 + c;
                new_data[iy * w + ix] = image.data[iy * w + ix] - plane_val;
            }
        }

        StmImage::new(w, h, new_data, image.pixel_size_nm)
    }

    /// Line-by-line polynomial subtraction (order 0 = mean, 1 = linear, 2 = quadratic, etc.).
    pub fn line_by_line(image: &StmImage, order: usize) -> StmImage {
        let w = image.width;
        let h = image.height;
        let mut new_data = vec![0.0; w * h];

        for iy in 0..h {
            let line: Vec<f64> = (0..w).map(|ix| image.data[iy * w + ix]).collect();
            let coeffs = poly_fit_1d(&line, order);
            for ix in 0..w {
                let x = ix as f64;
                let mut val = 0.0;
                for (k, &ck) in coeffs.iter().enumerate() {
                    val += ck * x.powi(k as i32);
                }
                new_data[iy * w + ix] = line[ix] - val;
            }
        }

        StmImage::new(w, h, new_data, image.pixel_size_nm)
    }

    /// 2D polynomial background subtraction.
    pub fn polynomial_2d(image: &StmImage, order: usize) -> StmImage {
        let w = image.width;
        let h = image.height;
        let n = w * h;

        // Build monomial terms for 2D polynomial up to given order
        // Terms: x^i * y^j where i+j <= order
        let mut terms: Vec<(usize, usize)> = Vec::new();
        for total in 0..=order {
            for i in 0..=total {
                let j = total - i;
                terms.push((i, j));
            }
        }
        let num_terms = terms.len();

        if num_terms > n {
            return image.clone();
        }

        // Build normal equations A^T A c = A^T z
        let mut ata = vec![0.0; num_terms * num_terms];
        let mut atz = vec![0.0; num_terms];

        for iy in 0..h {
            for ix in 0..w {
                let x = ix as f64;
                let y = iy as f64;
                let z = image.data[iy * w + ix];

                let basis: Vec<f64> = terms.iter()
                    .map(|&(pi, pj)| x.powi(pi as i32) * y.powi(pj as i32))
                    .collect();

                for r in 0..num_terms {
                    for c in 0..num_terms {
                        ata[r * num_terms + c] += basis[r] * basis[c];
                    }
                    atz[r] += basis[r] * z;
                }
            }
        }

        // Solve via Gaussian elimination
        let coeffs = solve_linear_system(&ata, &atz, num_terms);

        let mut new_data = vec![0.0; n];
        for iy in 0..h {
            for ix in 0..w {
                let x = ix as f64;
                let y = iy as f64;
                let mut bg = 0.0;
                for (k, &(pi, pj)) in terms.iter().enumerate() {
                    bg += coeffs[k] * x.powi(pi as i32) * y.powi(pj as i32);
                }
                new_data[iy * w + ix] = image.data[iy * w + ix] - bg;
            }
        }

        StmImage::new(w, h, new_data, image.pixel_size_nm)
    }

    /// Median line correction: subtract the median of each scan line.
    pub fn median_line_correction(image: &StmImage) -> StmImage {
        let w = image.width;
        let h = image.height;
        let mut new_data = vec![0.0; w * h];

        for iy in 0..h {
            let mut line: Vec<f64> = (0..w).map(|ix| image.data[iy * w + ix]).collect();
            line.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let median = if w % 2 == 0 {
                (line[w / 2 - 1] + line[w / 2]) / 2.0
            } else {
                line[w / 2]
            };
            for ix in 0..w {
                new_data[iy * w + ix] = image.data[iy * w + ix] - median;
            }
        }

        StmImage::new(w, h, new_data, image.pixel_size_nm)
    }
}

/// Tunnel current physics: I(V, z) based on the WKB approximation.
pub struct TunnelCurrentModel;

impl TunnelCurrentModel {
    /// Decay constant kappa in 1/nm.
    ///
    /// kappa = sqrt(2 * m * phi) / hbar, converted to nm^-1.
    /// For phi in eV: kappa ~ 10.25 * sqrt(phi/eV) nm^-1.
    pub fn decay_constant(phi_ev: f64) -> f64 {
        let phi_j = phi_ev * EV_TO_J;
        let kappa_m = (2.0 * ELECTRON_MASS_KG * phi_j).sqrt() / HBAR_JS;
        kappa_m * 1e-9 // convert 1/m to 1/nm
    }

    /// Tunneling current (arbitrary units) via WKB: I ~ V * exp(-2*kappa*d).
    ///
    /// `v_bias`: bias voltage (V)
    /// `d_nm`: tip-sample distance (nm)
    /// `phi_ev`: effective barrier height (eV)
    pub fn current_wkb(v_bias: f64, d_nm: f64, phi_ev: f64) -> f64 {
        let kappa = Self::decay_constant(phi_ev);
        v_bias * (-2.0 * kappa * d_nm).exp()
    }

    /// Apparent barrier height (eV) from I(z) spectroscopy.
    ///
    /// phi = (hbar^2 / (8*m)) * (ln(I1/I2) / (z2 - z1))^2
    pub fn apparent_barrier_height(i1: f64, i2: f64, z1_nm: f64, z2_nm: f64) -> f64 {
        let dz = (z2_nm - z1_nm) * 1e-9; // convert to meters
        if dz.abs() < 1e-30 || i1 <= 0.0 || i2 <= 0.0 {
            return 0.0;
        }
        let ln_ratio = (i1 / i2).ln();
        let factor = HBAR_JS * HBAR_JS / (8.0 * ELECTRON_MASS_KG);
        let phi_j = factor * (ln_ratio / dz) * (ln_ratio / dz);
        phi_j / EV_TO_J
    }

    /// Differential conductance dI/dV (arbitrary units).
    ///
    /// Computed analytically from the WKB model: dI/dV = exp(-2*kappa*d).
    pub fn conductance(v_bias: f64, d_nm: f64, phi_ev: f64) -> f64 {
        let kappa = Self::decay_constant(phi_ev);
        // dI/dV of I = V * exp(-2*kappa*d) is just exp(-2*kappa*d)
        // (d and phi treated as constant for this simple model)
        let _ = v_bias; // V-dependence is in the full Simmons model
        (-2.0 * kappa * d_nm).exp()
    }
}

/// Scanning Tunneling Spectroscopy: I(V) curves and dI/dV analysis.
#[derive(Clone, Debug)]
pub struct StsSpectroscopy {
    pub bias_v: Vec<f64>,
    pub current_na: Vec<f64>,
}

impl StsSpectroscopy {
    /// Create new STS data from bias voltages and measured currents.
    pub fn new(bias_v: Vec<f64>, current_na: Vec<f64>) -> Self {
        assert_eq!(bias_v.len(), current_na.len(), "Bias and current arrays must match");
        assert!(bias_v.len() >= 2, "Need at least 2 data points");
        Self { bias_v, current_na }
    }

    /// Numerical dI/dV (proportional to LDOS) with optional Savitzky-Golay-like smoothing.
    ///
    /// `smoothing`: number of adjacent points for moving average (0 = no smoothing).
    /// Returns (V, dI/dV) pairs.
    pub fn didv(&self, smoothing: usize) -> Vec<(f64, f64)> {
        let n = self.bias_v.len();
        if n < 2 {
            return vec![];
        }

        // Central finite difference for dI/dV
        let mut raw_didv = vec![0.0; n];
        // Forward difference at start
        raw_didv[0] = (self.current_na[1] - self.current_na[0])
            / (self.bias_v[1] - self.bias_v[0]);
        // Backward difference at end
        raw_didv[n - 1] = (self.current_na[n - 1] - self.current_na[n - 2])
            / (self.bias_v[n - 1] - self.bias_v[n - 2]);
        // Central difference for interior
        for i in 1..n - 1 {
            raw_didv[i] = (self.current_na[i + 1] - self.current_na[i - 1])
                / (self.bias_v[i + 1] - self.bias_v[i - 1]);
        }

        // Apply smoothing via moving average
        let smoothed = if smoothing > 0 {
            moving_average(&raw_didv, smoothing)
        } else {
            raw_didv
        };

        self.bias_v.iter().zip(smoothed.iter())
            .map(|(&v, &d)| (v, d))
            .collect()
    }

    /// Normalized conductance (dI/dV) / (I/V).
    ///
    /// This removes the exponential background and highlights LDOS features.
    /// Returns (V, normalized) pairs.
    pub fn normalized_didv(&self) -> Vec<(f64, f64)> {
        let didv = self.didv(0);
        let mut result = Vec::with_capacity(didv.len());

        for (i, &(v, d)) in didv.iter().enumerate() {
            if v.abs() < 1e-12 {
                // At V=0, use limit: (dI/dV)/(I/V) -> 1 for ohmic contact
                result.push((v, 1.0));
            } else {
                let iv = self.current_na[i] / v;
                if iv.abs() < 1e-30 {
                    result.push((v, 0.0));
                } else {
                    result.push((v, d / iv));
                }
            }
        }

        result
    }

    /// Estimate band gap from dI/dV data.
    ///
    /// The gap is the voltage range around V=0 where dI/dV is near zero.
    /// We find the region where dI/dV < threshold * max(dI/dV).
    pub fn band_gap(didv: &[(f64, f64)]) -> f64 {
        if didv.is_empty() {
            return 0.0;
        }

        let max_didv = didv.iter()
            .map(|&(_, d)| d.abs())
            .fold(0.0_f64, f64::max);

        if max_didv < 1e-30 {
            return 0.0;
        }

        let threshold = 0.05 * max_didv; // 5% of max as threshold

        // Find leftmost voltage above threshold (conduction band edge)
        let mut v_left = 0.0_f64;
        let mut v_right = 0.0_f64;

        // Search from V=0 toward negative bias
        for &(v, d) in didv.iter().rev() {
            if v < 0.0 && d.abs() > threshold {
                v_left = v;
                break;
            }
        }

        // Search from V=0 toward positive bias
        for &(v, d) in didv.iter() {
            if v > 0.0 && d.abs() > threshold {
                v_right = v;
                break;
            }
        }

        (v_right - v_left).abs()
    }

    /// Estimate Fermi level position from V=0 crossing.
    ///
    /// Returns the voltage at which the dI/dV has a zero crossing nearest to V=0,
    /// or 0.0 if the Fermi level is at V=0 (as expected for metals).
    pub fn fermi_level_position(didv: &[(f64, f64)]) -> f64 {
        // For a metal, EF is at V=0. For semiconductors, look for
        // the midpoint of the gap.
        if didv.is_empty() {
            return 0.0;
        }

        let max_didv = didv.iter()
            .map(|&(_, d)| d.abs())
            .fold(0.0_f64, f64::max);

        if max_didv < 1e-30 {
            return 0.0;
        }

        let threshold = 0.05 * max_didv;

        // Find gap edges
        let mut left_edge = None;
        let mut right_edge = None;

        for &(v, d) in didv.iter() {
            if v < 0.0 && d.abs() > threshold {
                left_edge = Some(v);
            }
            if v > 0.0 && d.abs() > threshold && right_edge.is_none() {
                right_edge = Some(v);
            }
        }

        match (left_edge, right_edge) {
            (Some(l), Some(r)) => (l + r) / 2.0, // midpoint of gap
            _ => 0.0,
        }
    }
}

/// 2D Fourier analysis for periodic surface structures.
pub struct FourierAnalysis;

impl FourierAnalysis {
    /// Compute 2D power spectrum (|FFT|^2) of the image.
    ///
    /// Returns a 2D array (row-major) of power spectral density.
    pub fn fft_2d(image: &StmImage) -> Vec<Vec<f64>> {
        let w = image.width;
        let h = image.height;

        // Compute 2D DFT via row-then-column 1D DFTs
        // First: DFT of each row
        let mut real = vec![vec![0.0; w]; h];
        let mut imag = vec![vec![0.0; w]; h];

        for iy in 0..h {
            let row: Vec<f64> = (0..w).map(|ix| image.data[iy * w + ix]).collect();
            let (re, im) = dft_1d(&row);
            real[iy] = re;
            imag[iy] = im;
        }

        // Then: DFT of each column
        let mut real2 = vec![vec![0.0; w]; h];
        let mut imag2 = vec![vec![0.0; w]; h];

        for ix in 0..w {
            let col_re: Vec<f64> = (0..h).map(|iy| real[iy][ix]).collect();
            let col_im: Vec<f64> = (0..h).map(|iy| imag[iy][ix]).collect();
            let (re, im) = dft_1d_complex(&col_re, &col_im);
            for iy in 0..h {
                real2[iy][ix] = re[iy];
                imag2[iy][ix] = im[iy];
            }
        }

        // Power spectrum
        let mut power = vec![vec![0.0; w]; h];
        for iy in 0..h {
            for ix in 0..w {
                power[iy][ix] = real2[iy][ix] * real2[iy][ix] + imag2[iy][ix] * imag2[iy][ix];
            }
        }

        power
    }

    /// Extract lattice vectors from FFT peaks.
    ///
    /// Finds the two strongest non-DC peaks in the FFT and converts to real-space
    /// lattice vectors.
    pub fn lattice_vectors(fft_image: &[Vec<f64>]) -> ((f64, f64), (f64, f64)) {
        if fft_image.is_empty() || fft_image[0].is_empty() {
            return ((0.0, 0.0), (0.0, 0.0));
        }

        let h = fft_image.len();
        let w = fft_image[0].len();

        // Find peaks (exclude DC at (0,0) and restrict to first half due to symmetry)
        let mut peaks: Vec<(f64, usize, usize)> = Vec::new();

        for iy in 0..h {
            for ix in 0..w {
                if ix == 0 && iy == 0 {
                    continue; // skip DC
                }
                peaks.push((fft_image[iy][ix], ix, iy));
            }
        }

        peaks.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

        let peak1 = if peaks.is_empty() {
            (0, 0)
        } else {
            (peaks[0].1, peaks[0].2)
        };

        // Find second peak that's not the conjugate mirror
        let mut peak2 = (0, 0);
        for p in peaks.iter().skip(1) {
            let (_, px, py) = *p;
            // Skip if it's the conjugate mirror of peak1
            let mirror_x = if peak1.0 == 0 { 0 } else { w - peak1.0 };
            let mirror_y = if peak1.1 == 0 { 0 } else { h - peak1.1 };
            if px == mirror_x && py == mirror_y {
                continue;
            }
            peak2 = (px, py);
            break;
        }

        // Convert frequency-domain positions to real-space vectors
        // For a peak at (kx, ky), the real-space periodicity is (Lx/kx, Ly/ky)
        let v1 = (
            if peak1.0 > 0 { w as f64 / peak1.0 as f64 } else { 0.0 },
            if peak1.1 > 0 { h as f64 / peak1.1 as f64 } else { 0.0 },
        );
        let v2 = (
            if peak2.0 > 0 { w as f64 / peak2.0 as f64 } else { 0.0 },
            if peak2.1 > 0 { h as f64 / peak2.1 as f64 } else { 0.0 },
        );

        (v1, v2)
    }

    /// Compute lattice constant from an FFT peak position.
    ///
    /// `peak_position`: (kx, ky) in pixel coordinates in the FFT
    /// `image_size_nm`: physical size of the image in nm
    /// Returns lattice constant d = L / r, where r = sqrt(kx^2 + ky^2).
    pub fn lattice_constant(peak_position: (f64, f64), image_size_nm: f64) -> f64 {
        let r = (peak_position.0 * peak_position.0 + peak_position.1 * peak_position.1).sqrt();
        if r < 1e-12 {
            return 0.0;
        }
        image_size_nm / r
    }

    /// Bandpass filter in frequency domain.
    ///
    /// `f_min`, `f_max`: frequency bounds in cycles per image width.
    pub fn filter_frequency(image: &StmImage, f_min: f64, f_max: f64) -> StmImage {
        let w = image.width;
        let h = image.height;

        // Forward 2D DFT
        let mut real = vec![vec![0.0; w]; h];
        let mut imag = vec![vec![0.0; w]; h];

        for iy in 0..h {
            let row: Vec<f64> = (0..w).map(|ix| image.data[iy * w + ix]).collect();
            let (re, im) = dft_1d(&row);
            real[iy] = re;
            imag[iy] = im;
        }

        let mut real2 = vec![vec![0.0; w]; h];
        let mut imag2 = vec![vec![0.0; w]; h];

        for ix in 0..w {
            let col_re: Vec<f64> = (0..h).map(|iy| real[iy][ix]).collect();
            let col_im: Vec<f64> = (0..h).map(|iy| imag[iy][ix]).collect();
            let (re, im) = dft_1d_complex(&col_re, &col_im);
            for iy in 0..h {
                real2[iy][ix] = re[iy];
                imag2[iy][ix] = im[iy];
            }
        }

        // Apply bandpass mask
        for iy in 0..h {
            for ix in 0..w {
                let fx = if ix <= w / 2 { ix as f64 } else { (w - ix) as f64 };
                let fy = if iy <= h / 2 { iy as f64 } else { (h - iy) as f64 };
                let freq = (fx * fx + fy * fy).sqrt();
                if freq < f_min || freq > f_max {
                    real2[iy][ix] = 0.0;
                    imag2[iy][ix] = 0.0;
                }
            }
        }

        // Inverse 2D DFT (columns first, then rows)
        let mut real3 = vec![vec![0.0; w]; h];
        let mut imag3 = vec![vec![0.0; w]; h];

        for ix in 0..w {
            let col_re: Vec<f64> = (0..h).map(|iy| real2[iy][ix]).collect();
            let col_im: Vec<f64> = (0..h).map(|iy| imag2[iy][ix]).collect();
            let (re, im) = idft_1d_complex(&col_re, &col_im);
            for iy in 0..h {
                real3[iy][ix] = re[iy];
                imag3[iy][ix] = im[iy];
            }
        }

        let mut new_data = vec![0.0; w * h];
        for iy in 0..h {
            let row_re: Vec<f64> = (0..w).map(|ix| real3[iy][ix]).collect();
            let row_im: Vec<f64> = (0..w).map(|ix| imag3[iy][ix]).collect();
            let (re, _im) = idft_1d_complex(&row_re, &row_im);
            for ix in 0..w {
                new_data[iy * w + ix] = re[ix];
            }
        }

        StmImage::new(w, h, new_data, image.pixel_size_nm)
    }
}

/// A detected step edge on the surface.
#[derive(Clone, Debug)]
pub struct StepEdge {
    pub start: (usize, usize),
    pub end: (usize, usize),
    pub height_nm: f64,
}

/// Detect atomic step edges in STM images.
pub struct StepEdgeDetector;

impl StepEdgeDetector {
    /// Detect step edges by gradient thresholding.
    ///
    /// `threshold_nm`: minimum height difference per pixel to qualify as a step.
    pub fn detect_steps(image: &StmImage, threshold_nm: f64) -> Vec<StepEdge> {
        let w = image.width;
        let h = image.height;
        let mut edges = Vec::new();

        // Horizontal gradient detection
        for iy in 0..h {
            let mut in_step = false;
            let mut start_x = 0;
            let mut max_diff = 0.0_f64;

            for ix in 1..w {
                let diff = (image.data[iy * w + ix] - image.data[iy * w + ix - 1]).abs();
                if diff > threshold_nm {
                    if !in_step {
                        start_x = ix - 1;
                        in_step = true;
                        max_diff = diff;
                    } else if diff > max_diff {
                        max_diff = diff;
                    }
                } else if in_step {
                    edges.push(StepEdge {
                        start: (start_x, iy),
                        end: (ix, iy),
                        height_nm: max_diff,
                    });
                    in_step = false;
                }
            }

            if in_step {
                edges.push(StepEdge {
                    start: (start_x, iy),
                    end: (w - 1, iy),
                    height_nm: max_diff,
                });
            }
        }

        // Vertical gradient detection
        for ix in 0..w {
            let mut in_step = false;
            let mut start_y = 0;
            let mut max_diff = 0.0_f64;

            for iy in 1..h {
                let diff = (image.data[iy * w + ix] - image.data[(iy - 1) * w + ix]).abs();
                if diff > threshold_nm {
                    if !in_step {
                        start_y = iy - 1;
                        in_step = true;
                        max_diff = diff;
                    } else if diff > max_diff {
                        max_diff = diff;
                    }
                } else if in_step {
                    edges.push(StepEdge {
                        start: (ix, start_y),
                        end: (ix, iy),
                        height_nm: max_diff,
                    });
                    in_step = false;
                }
            }

            if in_step {
                edges.push(StepEdge {
                    start: (ix, start_y),
                    end: (ix, h - 1),
                    height_nm: max_diff,
                });
            }
        }

        edges
    }

    /// Histogram of step heights.
    ///
    /// Returns (bin_center, count) pairs.
    pub fn step_height_histogram(edges: &[StepEdge], bin_width: f64) -> Vec<(f64, usize)> {
        if edges.is_empty() || bin_width <= 0.0 {
            return vec![];
        }

        let max_h = edges.iter().map(|e| e.height_nm).fold(0.0_f64, f64::max);
        let num_bins = ((max_h / bin_width).ceil() as usize).max(1);
        let mut counts = vec![0usize; num_bins];

        for edge in edges {
            let bin = ((edge.height_nm / bin_width).floor() as usize).min(num_bins - 1);
            counts[bin] += 1;
        }

        counts.iter().enumerate()
            .map(|(i, &c)| ((i as f64 + 0.5) * bin_width, c))
            .collect()
    }

    /// Average terrace width along a given direction.
    ///
    /// `direction`: (dx, dy) unit vector for scan direction.
    pub fn terrace_width(image: &StmImage, direction: (f64, f64)) -> f64 {
        let w = image.width;
        let h = image.height;
        let len = (direction.0 * direction.0 + direction.1 * direction.1).sqrt();
        if len < 1e-12 {
            return 0.0;
        }
        let dx = direction.0 / len;
        let dy = direction.1 / len;

        // Extract profile along direction from center
        let cx = w as f64 / 2.0;
        let cy = h as f64 / 2.0;
        let max_steps = (w.max(h) as f64 * 1.5) as usize;

        let mut profile = Vec::new();
        for i in 0..max_steps {
            let t = i as f64 - max_steps as f64 / 2.0;
            let px = cx + dx * t;
            let py = cy + dy * t;
            let ix = px.round() as isize;
            let iy = py.round() as isize;
            if ix >= 0 && ix < w as isize && iy >= 0 && iy < h as isize {
                profile.push(image.data[iy as usize * w + ix as usize]);
            }
        }

        if profile.len() < 2 {
            return 0.0;
        }

        // Count zero crossings of the derivative (step locations)
        let mut step_positions = Vec::new();
        let rms = {
            let mean: f64 = profile.iter().sum::<f64>() / profile.len() as f64;
            let var: f64 = profile.iter().map(|&z| (z - mean) * (z - mean)).sum::<f64>() / profile.len() as f64;
            var.sqrt()
        };
        let threshold = rms * 0.5;

        for i in 1..profile.len() {
            let diff = (profile[i] - profile[i - 1]).abs();
            if diff > threshold {
                step_positions.push(i);
            }
        }

        if step_positions.len() < 2 {
            return profile.len() as f64 * image.pixel_size_nm;
        }

        // Average distance between steps
        let mut total_width = 0.0;
        for i in 1..step_positions.len() {
            total_width += (step_positions[i] - step_positions[i - 1]) as f64;
        }
        total_width / (step_positions.len() - 1) as f64 * image.pixel_size_nm
    }
}

/// Thermal/piezo drift correction for STM images.
pub struct DriftCorrection;

impl DriftCorrection {
    /// Estimate drift between two consecutive images via cross-correlation.
    ///
    /// Returns (dx, dy) offset in pixels.
    pub fn estimate_drift(image1: &StmImage, image2: &StmImage) -> (f64, f64) {
        assert_eq!(image1.width, image2.width);
        assert_eq!(image1.height, image2.height);

        let w = image1.width;
        let h = image1.height;

        // Cross-correlation with limited search range
        let max_shift = (w.min(h) / 4).max(1);
        let mut best_corr = f64::NEG_INFINITY;
        let mut best_dx = 0_isize;
        let mut best_dy = 0_isize;

        for dy in -(max_shift as isize)..=(max_shift as isize) {
            for dx in -(max_shift as isize)..=(max_shift as isize) {
                let mut corr = 0.0;
                let mut count = 0;

                for iy in 0..h {
                    let iy2 = iy as isize + dy;
                    if iy2 < 0 || iy2 >= h as isize {
                        continue;
                    }
                    for ix in 0..w {
                        let ix2 = ix as isize + dx;
                        if ix2 < 0 || ix2 >= w as isize {
                            continue;
                        }
                        corr += image1.data[iy * w + ix] * image2.data[iy2 as usize * w + ix2 as usize];
                        count += 1;
                    }
                }

                if count > 0 {
                    corr /= count as f64;
                }

                if corr > best_corr {
                    best_corr = corr;
                    best_dx = dx;
                    best_dy = dy;
                }
            }
        }

        (best_dx as f64, best_dy as f64)
    }

    /// Correct linear drift by shearing.
    ///
    /// `drift_x_nm_per_line`: x drift per scan line in nm.
    /// `drift_y_nm_per_line`: y drift per scan line in nm.
    pub fn correct_linear_drift(
        image: &StmImage,
        drift_x_nm_per_line: f64,
        drift_y_nm_per_line: f64,
    ) -> StmImage {
        let w = image.width;
        let h = image.height;
        let ps = image.pixel_size_nm;
        let mut new_data = vec![0.0; w * h];

        let drift_x_pix = drift_x_nm_per_line / ps;
        let drift_y_pix = drift_y_nm_per_line / ps;

        for iy in 0..h {
            for ix in 0..w {
                // Source coordinates with drift correction
                let src_x = ix as f64 + drift_x_pix * iy as f64;
                let src_y = iy as f64 + drift_y_pix * iy as f64;

                // Bilinear interpolation
                let x0 = src_x.floor() as isize;
                let y0 = src_y.floor() as isize;
                let fx = src_x - x0 as f64;
                let fy = src_y - y0 as f64;

                let val = bilinear_sample(image, x0, y0, fx, fy);
                new_data[iy * w + ix] = val;
            }
        }

        StmImage::new(w, h, new_data, ps)
    }

    /// Creep correction: compensate exponential piezo creep.
    ///
    /// Creep model: x_actual = x_nominal * (1 + gamma * ln(1 + t/tau))
    /// `decay_constant`: creep ratio gamma (typically 0.01-0.1).
    pub fn creep_correction(image: &StmImage, decay_constant: f64) -> StmImage {
        let w = image.width;
        let h = image.height;
        let mut new_data = vec![0.0; w * h];

        for iy in 0..h {
            for ix in 0..w {
                // Correct x position for creep
                let t_frac = ix as f64 / w as f64;
                let correction = 1.0 + decay_constant * (1.0 + t_frac).ln();
                let src_x = ix as f64 / correction;

                let x0 = src_x.floor() as isize;
                let fx = src_x - x0 as f64;

                let val = bilinear_sample(image, x0, iy as isize, fx, 0.0);
                new_data[iy * w + ix] = val;
            }
        }

        StmImage::new(w, h, new_data, image.pixel_size_nm)
    }
}

/// Statistical surface height distribution analysis.
pub struct HeightDistribution;

impl HeightDistribution {
    /// Height histogram of the image.
    pub fn histogram(image: &StmImage, num_bins: usize) -> Vec<(f64, usize)> {
        if num_bins == 0 || image.data.is_empty() {
            return vec![];
        }

        let min_z = image.min_height();
        let max_z = image.max_height();
        let range = max_z - min_z;

        if range < 1e-30 {
            return vec![(min_z, image.data.len())];
        }

        let bin_width = range / num_bins as f64;
        let mut counts = vec![0usize; num_bins];

        for &z in &image.data {
            let bin = ((z - min_z) / bin_width).floor() as usize;
            let bin = bin.min(num_bins - 1);
            counts[bin] += 1;
        }

        counts.iter().enumerate()
            .map(|(i, &c)| (min_z + (i as f64 + 0.5) * bin_width, c))
            .collect()
    }

    /// Bearing ratio / Abbott-Firestone curve.
    ///
    /// Returns (height, ratio) where ratio is the fraction of the surface
    /// above the given height level.
    pub fn bearing_ratio(image: &StmImage) -> Vec<(f64, f64)> {
        let n = image.data.len();
        let mut sorted: Vec<f64> = image.data.clone();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let num_points = 100.min(n);
        let mut result = Vec::with_capacity(num_points);

        for i in 0..num_points {
            let idx = (i * n) / num_points;
            let height = sorted[idx];
            let above = sorted.iter().filter(|&&z| z >= height).count();
            result.push((height, above as f64 / n as f64));
        }

        result
    }

    /// Skewness Ssk: measure of height distribution asymmetry.
    ///
    /// Ssk = (1/N) * sum((z - mean)^3) / Sq^3
    pub fn skewness(image: &StmImage) -> f64 {
        let mean = image.mean_height();
        let sq = image.rms_roughness();
        if sq < 1e-30 {
            return 0.0;
        }
        let n = image.data.len() as f64;
        let sum_cube: f64 = image.data.iter()
            .map(|&z| {
                let d = z - mean;
                d * d * d
            })
            .sum();
        sum_cube / (n * sq * sq * sq)
    }

    /// Kurtosis Sku: measure of height distribution peakedness.
    ///
    /// Sku = (1/N) * sum((z - mean)^4) / Sq^4
    /// Gaussian surface has Sku = 3.
    pub fn kurtosis(image: &StmImage) -> f64 {
        let mean = image.mean_height();
        let sq = image.rms_roughness();
        if sq < 1e-30 {
            return 0.0;
        }
        let n = image.data.len() as f64;
        let sq4 = sq * sq * sq * sq;
        let sum_fourth: f64 = image.data.iter()
            .map(|&z| {
                let d = z - mean;
                d * d * d * d
            })
            .sum();
        sum_fourth / (n * sq4)
    }

    /// Correlation length: distance at which autocorrelation drops to 1/e.
    pub fn correlation_length(image: &StmImage) -> f64 {
        let w = image.width;
        let h = image.height;
        let mean = image.mean_height();
        let var: f64 = image.data.iter().map(|&z| (z - mean) * (z - mean)).sum::<f64>()
            / image.data.len() as f64;

        if var < 1e-30 {
            return 0.0;
        }

        // 1D autocorrelation along x (averaged over rows)
        let max_lag = w / 2;
        let threshold = 1.0 / std::f64::consts::E;

        for lag in 1..max_lag {
            let mut corr = 0.0;
            let mut count = 0;

            for iy in 0..h {
                for ix in 0..(w - lag) {
                    let z1 = image.data[iy * w + ix] - mean;
                    let z2 = image.data[iy * w + ix + lag] - mean;
                    corr += z1 * z2;
                    count += 1;
                }
            }

            if count > 0 {
                corr /= count as f64;
            }

            let normalized = corr / var;
            if normalized < threshold {
                // Linear interpolation for sub-pixel accuracy
                let prev_lag = lag - 1;
                let mut prev_corr = 0.0;
                let mut prev_count = 0;
                if prev_lag == 0 {
                    prev_corr = var;
                    prev_count = 1;
                } else {
                    for iy in 0..h {
                        for ix in 0..(w - prev_lag) {
                            let z1 = image.data[iy * w + ix] - mean;
                            let z2 = image.data[iy * w + ix + prev_lag] - mean;
                            prev_corr += z1 * z2;
                            prev_count += 1;
                        }
                    }
                    if prev_count > 0 {
                        prev_corr /= prev_count as f64;
                    }
                }
                let prev_norm = prev_corr / var;

                if (prev_norm - normalized).abs() > 1e-30 {
                    let frac = (prev_norm - threshold) / (prev_norm - normalized);
                    return (prev_lag as f64 + frac) * image.pixel_size_nm;
                }

                return lag as f64 * image.pixel_size_nm;
            }
        }

        max_lag as f64 * image.pixel_size_nm
    }
}

/// Locate individual atoms in STM images.
pub struct AtomTracker;

impl AtomTracker {
    /// Find atom positions using local maximum detection.
    ///
    /// `expected_spacing_nm`: approximate interatomic distance for neighborhood size.
    /// Returns atom positions in nm.
    pub fn find_atoms(image: &StmImage, expected_spacing_nm: f64) -> Vec<(f64, f64)> {
        let w = image.width;
        let h = image.height;
        let ps = image.pixel_size_nm;

        // Neighborhood radius in pixels
        let radius = ((expected_spacing_nm / ps) / 2.0).ceil() as usize;
        let radius = radius.max(1);

        let mut atoms = Vec::new();

        for iy in radius..h.saturating_sub(radius) {
            for ix in radius..w.saturating_sub(radius) {
                let z = image.data[iy * w + ix];
                let mut is_max = true;

                'outer: for dy in -(radius as isize)..=(radius as isize) {
                    for dx in -(radius as isize)..=(radius as isize) {
                        if dx == 0 && dy == 0 {
                            continue;
                        }
                        let nx = (ix as isize + dx) as usize;
                        let ny = (iy as isize + dy) as usize;
                        if image.data[ny * w + nx] > z {
                            is_max = false;
                            break 'outer;
                        }
                    }
                }

                if is_max {
                    // Sub-pixel refinement via center-of-mass in 3x3 neighborhood
                    let mut sum_w = 0.0;
                    let mut sum_wx = 0.0;
                    let mut sum_wy = 0.0;
                    let min_z = image.min_height();

                    for dy in -1..=1_isize {
                        for dx in -1..=1_isize {
                            let nx = (ix as isize + dx) as usize;
                            let ny = (iy as isize + dy) as usize;
                            let weight = image.data[ny * w + nx] - min_z;
                            sum_w += weight;
                            sum_wx += weight * (ix as f64 + dx as f64);
                            sum_wy += weight * (iy as f64 + dy as f64);
                        }
                    }

                    if sum_w > 0.0 {
                        atoms.push((sum_wx / sum_w * ps, sum_wy / sum_w * ps));
                    } else {
                        atoms.push((ix as f64 * ps, iy as f64 * ps));
                    }
                }
            }
        }

        atoms
    }

    /// Average nearest-neighbor distance among atoms.
    pub fn nearest_neighbor_distance(atoms: &[(f64, f64)]) -> f64 {
        if atoms.len() < 2 {
            return 0.0;
        }

        let mut total = 0.0;
        for i in 0..atoms.len() {
            let mut min_dist = f64::INFINITY;
            for j in 0..atoms.len() {
                if i == j {
                    continue;
                }
                let dx = atoms[i].0 - atoms[j].0;
                let dy = atoms[i].1 - atoms[j].1;
                let dist = (dx * dx + dy * dy).sqrt();
                if dist < min_dist {
                    min_dist = dist;
                }
            }
            total += min_dist;
        }

        total / atoms.len() as f64
    }

    /// Coordination number for each atom (number of neighbors within cutoff).
    pub fn coordination_number(atoms: &[(f64, f64)], cutoff_nm: f64) -> Vec<usize> {
        atoms.iter().enumerate().map(|(i, &a)| {
            atoms.iter().enumerate()
                .filter(|(j, &b)| {
                    if *j == i { return false; }
                    let dx = a.0 - b.0;
                    let dy = a.1 - b.1;
                    (dx * dx + dy * dy).sqrt() <= cutoff_nm
                })
                .count()
        }).collect()
    }

    /// Approximate Voronoi cell area for a given atom.
    ///
    /// Uses Monte Carlo estimation within a bounding box.
    pub fn voronoi_area(atoms: &[(f64, f64)], index: usize) -> f64 {
        if atoms.is_empty() || index >= atoms.len() || atoms.len() < 2 {
            return 0.0;
        }

        // Bounding box
        let min_x = atoms.iter().map(|a| a.0).fold(f64::INFINITY, f64::min);
        let max_x = atoms.iter().map(|a| a.0).fold(f64::NEG_INFINITY, f64::max);
        let min_y = atoms.iter().map(|a| a.1).fold(f64::INFINITY, f64::min);
        let max_y = atoms.iter().map(|a| a.1).fold(f64::NEG_INFINITY, f64::max);

        let box_area = (max_x - min_x) * (max_y - min_y);
        if box_area < 1e-30 {
            return 0.0;
        }

        // Deterministic grid sampling
        let grid_n = 100;
        let mut count_total = 0;
        let mut count_cell = 0;

        for gy in 0..grid_n {
            for gx in 0..grid_n {
                let px = min_x + (gx as f64 + 0.5) * (max_x - min_x) / grid_n as f64;
                let py = min_y + (gy as f64 + 0.5) * (max_y - min_y) / grid_n as f64;

                // Find nearest atom
                let mut best_dist = f64::INFINITY;
                let mut best_idx = 0;
                for (k, &a) in atoms.iter().enumerate() {
                    let dx = px - a.0;
                    let dy = py - a.1;
                    let d = dx * dx + dy * dy;
                    if d < best_dist {
                        best_dist = d;
                        best_idx = k;
                    }
                }

                count_total += 1;
                if best_idx == index {
                    count_cell += 1;
                }
            }
        }

        if count_total == 0 {
            return 0.0;
        }

        box_area * count_cell as f64 / count_total as f64
    }
}

/// Piezo scanner calibration utilities.
pub struct PiezoCalibration;

impl PiezoCalibration {
    /// XY calibration scale factor from a known reference.
    pub fn calibrate_xy(measured_nm: f64, known_nm: f64) -> f64 {
        if measured_nm.abs() < 1e-30 {
            return 1.0;
        }
        known_nm / measured_nm
    }

    /// Z calibration scale factor from a known step height.
    pub fn calibrate_z(measured_nm: f64, known_step_nm: f64) -> f64 {
        if measured_nm.abs() < 1e-30 {
            return 1.0;
        }
        known_step_nm / measured_nm
    }

    /// Hysteresis correction: average of forward and backward scan.
    pub fn hysteresis_correction(forward: &[f64], backward: &[f64]) -> Vec<f64> {
        assert_eq!(forward.len(), backward.len(), "Forward and backward must match");
        forward.iter().zip(backward.iter())
            .map(|(&f, &b)| (f + b) / 2.0)
            .collect()
    }

    /// Polynomial nonlinearity correction.
    ///
    /// `coeffs`: polynomial coefficients [c0, c1, c2, ...] such that
    /// corrected = c0 + c1*x + c2*x^2 + ...
    pub fn nonlinearity_correction(position: f64, coeffs: &[f64]) -> f64 {
        let mut result = 0.0;
        for (k, &c) in coeffs.iter().enumerate() {
            result += c * position.powi(k as i32);
        }
        result
    }
}

// ============================================================
// Helper functions
// ============================================================

/// 1D polynomial fit via least squares (real-valued input).
fn poly_fit_1d(data: &[f64], order: usize) -> Vec<f64> {
    let n = data.len();
    let m = order + 1;

    if n < m {
        return vec![0.0; m];
    }

    // Build normal equations
    let mut ata = vec![0.0; m * m];
    let mut atz = vec![0.0; m];

    for i in 0..n {
        let x = i as f64;
        let z = data[i];
        for r in 0..m {
            for c in 0..m {
                ata[r * m + c] += x.powi((r + c) as i32);
            }
            atz[r] += x.powi(r as i32) * z;
        }
    }

    solve_linear_system(&ata, &atz, m)
}

/// Solve Ax = b via Gaussian elimination with partial pivoting.
fn solve_linear_system(a: &[f64], b: &[f64], n: usize) -> Vec<f64> {
    let mut aug = vec![0.0; n * (n + 1)];

    for r in 0..n {
        for c in 0..n {
            aug[r * (n + 1) + c] = a[r * n + c];
        }
        aug[r * (n + 1) + n] = b[r];
    }

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = aug[col * (n + 1) + col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            let val = aug[row * (n + 1) + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        // Swap rows
        if max_row != col {
            for c in 0..=(n) {
                let tmp = aug[col * (n + 1) + c];
                aug[col * (n + 1) + c] = aug[max_row * (n + 1) + c];
                aug[max_row * (n + 1) + c] = tmp;
            }
        }

        let pivot = aug[col * (n + 1) + col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        for row in (col + 1)..n {
            let factor = aug[row * (n + 1) + col] / pivot;
            for c in col..=n {
                aug[row * (n + 1) + c] -= factor * aug[col * (n + 1) + c];
            }
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let pivot = aug[row * (n + 1) + row];
        if pivot.abs() < 1e-30 {
            continue;
        }
        let mut sum = aug[row * (n + 1) + n];
        for c in (row + 1)..n {
            sum -= aug[row * (n + 1) + c] * x[c];
        }
        x[row] = sum / pivot;
    }

    x
}

/// Moving average filter.
fn moving_average(data: &[f64], window: usize) -> Vec<f64> {
    let n = data.len();
    if window == 0 {
        return data.to_vec();
    }

    let half = window / 2;
    let mut result = vec![0.0; n];

    for i in 0..n {
        let start = if i >= half { i - half } else { 0 };
        let end = (i + half + 1).min(n);
        let sum: f64 = data[start..end].iter().sum();
        result[i] = sum / (end - start) as f64;
    }

    result
}

/// 1D Discrete Fourier Transform (real input).
fn dft_1d(data: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let n = data.len();
    let mut real = vec![0.0; n];
    let mut imag = vec![0.0; n];

    for k in 0..n {
        for j in 0..n {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            real[k] += data[j] * angle.cos();
            imag[k] += data[j] * angle.sin();
        }
    }

    (real, imag)
}

/// 1D DFT of complex input.
fn dft_1d_complex(re_in: &[f64], im_in: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let n = re_in.len();
    let mut real = vec![0.0; n];
    let mut imag = vec![0.0; n];

    for k in 0..n {
        for j in 0..n {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            real[k] += re_in[j] * cos_a - im_in[j] * sin_a;
            imag[k] += re_in[j] * sin_a + im_in[j] * cos_a;
        }
    }

    (real, imag)
}

/// 1D Inverse DFT of complex input.
fn idft_1d_complex(re_in: &[f64], im_in: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let n = re_in.len();
    let mut real = vec![0.0; n];
    let mut imag = vec![0.0; n];

    for k in 0..n {
        for j in 0..n {
            let angle = 2.0 * PI * k as f64 * j as f64 / n as f64;
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            real[k] += re_in[j] * cos_a - im_in[j] * sin_a;
            imag[k] += re_in[j] * sin_a + im_in[j] * cos_a;
        }
        real[k] /= n as f64;
        imag[k] /= n as f64;
    }

    (real, imag)
}

/// Bilinear interpolation from an StmImage.
fn bilinear_sample(image: &StmImage, x0: isize, y0: isize, fx: f64, fy: f64) -> f64 {
    let w = image.width as isize;
    let h = image.height as isize;

    let clamp_x = |x: isize| x.max(0).min(w - 1) as usize;
    let clamp_y = |y: isize| y.max(0).min(h - 1) as usize;

    let v00 = image.data[clamp_y(y0) * image.width + clamp_x(x0)];
    let v10 = image.data[clamp_y(y0) * image.width + clamp_x(x0 + 1)];
    let v01 = image.data[clamp_y(y0 + 1) * image.width + clamp_x(x0)];
    let v11 = image.data[clamp_y(y0 + 1) * image.width + clamp_x(x0 + 1)];

    v00 * (1.0 - fx) * (1.0 - fy)
        + v10 * fx * (1.0 - fy)
        + v01 * (1.0 - fx) * fy
        + v11 * fx * fy
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn make_flat_image(w: usize, h: usize, z: f64) -> StmImage {
        StmImage::new(w, h, vec![z; w * h], 0.1)
    }

    fn make_tilted_image(w: usize, h: usize) -> StmImage {
        let mut data = vec![0.0; w * h];
        for iy in 0..h {
            for ix in 0..w {
                data[iy * w + ix] = 0.5 * ix as f64 + 0.3 * iy as f64 + 1.0;
            }
        }
        StmImage::new(w, h, data, 0.1)
    }

    fn make_sinusoidal_image(w: usize, h: usize, period_pixels: f64) -> StmImage {
        let mut data = vec![0.0; w * h];
        for iy in 0..h {
            for ix in 0..w {
                data[iy * w + ix] = (2.0 * PI * ix as f64 / period_pixels).sin()
                    + (2.0 * PI * iy as f64 / period_pixels).sin();
            }
        }
        StmImage::new(w, h, data, 0.1)
    }

    fn make_step_image(w: usize, h: usize) -> StmImage {
        let mut data = vec![0.0; w * h];
        for iy in 0..h {
            for ix in 0..w {
                data[iy * w + ix] = if ix >= w / 2 { 0.3 } else { 0.0 };
            }
        }
        StmImage::new(w, h, data, 0.1)
    }

    // ---- StmImage tests ----

    #[test]
    fn test_stm_image_new() {
        let img = make_flat_image(10, 10, 1.0);
        assert_eq!(img.width, 10);
        assert_eq!(img.height, 10);
        assert_eq!(img.pixel_size_nm, 0.1);
    }

    #[test]
    fn test_height_at() {
        let mut data = vec![0.0; 4];
        data[0] = 1.0;
        data[1] = 2.0;
        data[2] = 3.0;
        data[3] = 4.0;
        let img = StmImage::new(2, 2, data, 0.1);
        assert_eq!(img.height_at(0, 0), 1.0);
        assert_eq!(img.height_at(1, 0), 2.0);
        assert_eq!(img.height_at(0, 1), 3.0);
        assert_eq!(img.height_at(1, 1), 4.0);
    }

    #[test]
    fn test_min_max_height() {
        let data = vec![1.0, 5.0, 3.0, -2.0, 7.0, 0.0];
        let img = StmImage::new(3, 2, data, 0.1);
        assert_eq!(img.min_height(), -2.0);
        assert_eq!(img.max_height(), 7.0);
    }

    #[test]
    fn test_mean_height() {
        let data = vec![1.0, 2.0, 3.0, 4.0];
        let img = StmImage::new(2, 2, data, 0.1);
        assert!((img.mean_height() - 2.5).abs() < 1e-10);
    }

    #[test]
    fn test_rms_roughness_flat() {
        let img = make_flat_image(10, 10, 5.0);
        assert!(img.rms_roughness() < 1e-10);
    }

    #[test]
    fn test_rms_roughness_varied() {
        let data = vec![0.0, 2.0, 0.0, 2.0]; // mean=1, deviations +-1
        let img = StmImage::new(2, 2, data, 0.1);
        assert!((img.rms_roughness() - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_image_size_nm() {
        let img = StmImage::new(20, 30, vec![0.0; 600], 0.5);
        assert!((img.size_x_nm() - 10.0).abs() < 1e-10);
        assert!((img.size_y_nm() - 15.0).abs() < 1e-10);
    }

    // ---- ImageFlattening tests ----

    #[test]
    fn test_plane_subtraction_flat() {
        let img = make_flat_image(10, 10, 5.0);
        let flat = ImageFlattening::plane_subtraction(&img);
        for &z in &flat.data {
            assert!(z.abs() < 1e-8, "After plane subtraction of flat, z={}", z);
        }
    }

    #[test]
    fn test_plane_subtraction_tilted() {
        let img = make_tilted_image(10, 10);
        let flat = ImageFlattening::plane_subtraction(&img);
        let rms = flat.rms_roughness();
        assert!(rms < 1e-6, "RMS after plane subtraction should be ~0, got {}", rms);
    }

    #[test]
    fn test_line_by_line_order0() {
        // Order 0 = subtract mean of each line
        let mut data = vec![0.0; 20];
        for ix in 0..10 {
            data[ix] = 5.0; // row 0: mean=5
            data[10 + ix] = 10.0; // row 1: mean=10
        }
        let img = StmImage::new(10, 2, data, 0.1);
        let flat = ImageFlattening::line_by_line(&img, 0);
        for &z in &flat.data {
            assert!(z.abs() < 1e-8);
        }
    }

    #[test]
    fn test_line_by_line_order1() {
        let img = make_tilted_image(10, 10);
        let flat = ImageFlattening::line_by_line(&img, 1);
        let rms = flat.rms_roughness();
        assert!(rms < 1e-6, "RMS after line-by-line linear = {}", rms);
    }

    #[test]
    fn test_polynomial_2d_order1() {
        let img = make_tilted_image(10, 10);
        let flat = ImageFlattening::polynomial_2d(&img, 1);
        let rms = flat.rms_roughness();
        assert!(rms < 1e-5, "2D poly order 1 RMS = {}", rms);
    }

    #[test]
    fn test_median_line_correction() {
        let mut data = vec![0.0; 20];
        for ix in 0..10 {
            data[ix] = 3.0;
            data[10 + ix] = 7.0;
        }
        let img = StmImage::new(10, 2, data, 0.1);
        let corrected = ImageFlattening::median_line_correction(&img);
        for &z in &corrected.data {
            assert!(z.abs() < 1e-10);
        }
    }

    // ---- TunnelCurrentModel tests ----

    #[test]
    fn test_decay_constant() {
        let kappa = TunnelCurrentModel::decay_constant(4.0);
        // kappa = sqrt(2*m*4eV)/hbar ~ 10.25 nm^-1
        assert!((kappa - 10.25).abs() < 0.1,
            "kappa for 4 eV = {}, expected ~10.25", kappa);
    }

    #[test]
    fn test_decay_constant_1ev() {
        let kappa = TunnelCurrentModel::decay_constant(1.0);
        // kappa = sqrt(2*m*1eV)/hbar ~ 5.12 nm^-1
        assert!((kappa - 5.12).abs() < 0.1, "kappa for 1eV = {}", kappa);
    }

    #[test]
    fn test_current_wkb_exponential_decay() {
        let i1 = TunnelCurrentModel::current_wkb(1.0, 0.5, 4.0);
        let i2 = TunnelCurrentModel::current_wkb(1.0, 1.0, 4.0);
        // Current should decrease with distance
        assert!(i1 > i2, "I(0.5nm) should > I(1nm)");
        // Ratio should be exp(-2*kappa*0.5)
        let kappa = TunnelCurrentModel::decay_constant(4.0);
        let expected_ratio = (2.0 * kappa * 0.5).exp();
        let actual_ratio = i1 / i2;
        assert!((actual_ratio - expected_ratio).abs() / expected_ratio < 0.01);
    }

    #[test]
    fn test_current_wkb_proportional_to_v() {
        let i1 = TunnelCurrentModel::current_wkb(1.0, 0.5, 4.0);
        let i2 = TunnelCurrentModel::current_wkb(2.0, 0.5, 4.0);
        assert!((i2 / i1 - 2.0).abs() < 1e-10, "Current should be proportional to V");
    }

    #[test]
    fn test_apparent_barrier_height() {
        // Generate I(z) from model and recover phi
        let phi = 4.0;
        let z1 = 0.5;
        let z2 = 0.7;
        let i1 = TunnelCurrentModel::current_wkb(1.0, z1, phi);
        let i2 = TunnelCurrentModel::current_wkb(1.0, z2, phi);
        let phi_est = TunnelCurrentModel::apparent_barrier_height(i1, i2, z1, z2);
        assert!((phi_est - phi).abs() < 0.1,
            "Estimated phi = {}, expected {}", phi_est, phi);
    }

    #[test]
    fn test_conductance_positive() {
        let g = TunnelCurrentModel::conductance(1.0, 0.5, 4.0);
        assert!(g > 0.0, "Conductance must be positive");
    }

    #[test]
    fn test_conductance_decreases_with_distance() {
        let g1 = TunnelCurrentModel::conductance(1.0, 0.5, 4.0);
        let g2 = TunnelCurrentModel::conductance(1.0, 1.0, 4.0);
        assert!(g1 > g2, "Conductance should decrease with distance");
    }

    // ---- StsSpectroscopy tests ----

    #[test]
    fn test_sts_new() {
        let bias: Vec<f64> = (-10..=10).map(|i| i as f64 * 0.1).collect();
        let current: Vec<f64> = bias.iter().map(|&v| v * 0.5).collect();
        let sts = StsSpectroscopy::new(bias, current);
        assert_eq!(sts.bias_v.len(), 21);
    }

    #[test]
    fn test_didv_linear() {
        // I = 2*V => dI/dV = 2 everywhere
        let bias: Vec<f64> = (0..10).map(|i| i as f64 * 0.1).collect();
        let current: Vec<f64> = bias.iter().map(|&v| 2.0 * v).collect();
        let sts = StsSpectroscopy::new(bias, current);
        let didv = sts.didv(0);
        for &(_v, d) in &didv {
            assert!((d - 2.0).abs() < 0.1, "dI/dV should be ~2, got {}", d);
        }
    }

    #[test]
    fn test_didv_with_smoothing() {
        let bias: Vec<f64> = (0..20).map(|i| i as f64 * 0.1).collect();
        let current: Vec<f64> = bias.iter().map(|&v| v * v).collect();
        let sts = StsSpectroscopy::new(bias, current);
        let didv_raw = sts.didv(0);
        let didv_smooth = sts.didv(3);
        // Both should have same length
        assert_eq!(didv_raw.len(), didv_smooth.len());
    }

    #[test]
    fn test_normalized_didv() {
        let bias: Vec<f64> = (1..11).map(|i| i as f64 * 0.1).collect();
        let current: Vec<f64> = bias.iter().map(|&v| v).collect(); // I = V (ohmic)
        let sts = StsSpectroscopy::new(bias, current);
        let norm = sts.normalized_didv();
        // For I=V, dI/dV = 1, I/V = 1, normalized = 1
        for &(_v, n) in &norm {
            assert!((n - 1.0).abs() < 0.5, "Normalized dI/dV for ohmic = {}", n);
        }
    }

    #[test]
    fn test_band_gap_metal() {
        // Metal: dI/dV > 0 everywhere
        let bias: Vec<f64> = (-10..=10).map(|i| i as f64 * 0.1).collect();
        let didv: Vec<(f64, f64)> = bias.iter().map(|&v| (v, 1.0)).collect();
        let gap = StsSpectroscopy::band_gap(&didv);
        assert!(gap < 0.3, "Metal gap should be ~0, got {}", gap);
    }

    #[test]
    fn test_band_gap_semiconductor() {
        // Semiconductor: dI/dV = 0 for |V| < 0.5
        let bias: Vec<f64> = (-20..=20).map(|i| i as f64 * 0.1).collect();
        let didv: Vec<(f64, f64)> = bias.iter().map(|&v| {
            if v.abs() < 0.5 { (v, 0.0) } else { (v, v.abs()) }
        }).collect();
        let gap = StsSpectroscopy::band_gap(&didv);
        assert!((gap - 1.0).abs() < 0.3, "Gap should be ~1.0 V, got {}", gap);
    }

    #[test]
    fn test_fermi_level_metal() {
        let bias: Vec<f64> = (-10..=10).map(|i| i as f64 * 0.1).collect();
        let didv: Vec<(f64, f64)> = bias.iter().map(|&v| (v, 1.0)).collect();
        let ef = StsSpectroscopy::fermi_level_position(&didv);
        assert!(ef.abs() < 0.5, "Metal EF should be ~0, got {}", ef);
    }

    // ---- FourierAnalysis tests ----

    #[test]
    fn test_fft_2d_dc() {
        let img = make_flat_image(8, 8, 3.0);
        let ps = FourierAnalysis::fft_2d(&img);
        // DC component should dominate
        let dc = ps[0][0];
        let total: f64 = ps.iter().flat_map(|row| row.iter()).sum();
        assert!(dc / total > 0.99, "DC should dominate for flat image");
    }

    #[test]
    fn test_fft_2d_periodic() {
        // Sinusoidal with period=4 pixels => peak at k=2 in 8-pixel transform
        let img = make_sinusoidal_image(8, 8, 4.0);
        let ps = FourierAnalysis::fft_2d(&img);
        // Should have peaks at spatial frequency 2
        assert!(ps[0][2] > ps[0][1], "Peak should be at k=2");
    }

    #[test]
    fn test_lattice_constant() {
        let d = FourierAnalysis::lattice_constant((5.0, 0.0), 10.0);
        assert!((d - 2.0).abs() < 1e-10, "Lattice constant = {}", d);
    }

    #[test]
    fn test_lattice_constant_diagonal() {
        let d = FourierAnalysis::lattice_constant((3.0, 4.0), 50.0);
        assert!((d - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_filter_frequency() {
        let img = make_sinusoidal_image(16, 16, 4.0);
        let filtered = FourierAnalysis::filter_frequency(&img, 3.0, 5.0);
        // Filtered image should retain the signal
        let rms_orig = img.rms_roughness();
        let rms_filt = filtered.rms_roughness();
        assert!(rms_filt > 0.5 * rms_orig, "Filtered RMS should retain signal: {} vs {}", rms_filt, rms_orig);
    }

    #[test]
    fn test_filter_removes_low_freq() {
        let img = make_flat_image(8, 8, 5.0);
        let filtered = FourierAnalysis::filter_frequency(&img, 1.0, 100.0);
        // Removing DC from flat image should give zeros
        let rms = filtered.rms_roughness();
        assert!(rms < 1e-8, "High-pass on flat should give 0, got {}", rms);
    }

    // ---- StepEdgeDetector tests ----

    #[test]
    fn test_detect_steps() {
        let img = make_step_image(10, 10);
        let edges = StepEdgeDetector::detect_steps(&img, 0.1);
        assert!(!edges.is_empty(), "Should find step edges");
    }

    #[test]
    fn test_step_height() {
        let img = make_step_image(10, 10);
        let edges = StepEdgeDetector::detect_steps(&img, 0.1);
        // Step height should be ~0.3
        for edge in &edges {
            assert!((edge.height_nm - 0.3).abs() < 0.05,
                "Step height = {}, expected ~0.3", edge.height_nm);
        }
    }

    #[test]
    fn test_no_steps_flat() {
        let img = make_flat_image(10, 10, 1.0);
        let edges = StepEdgeDetector::detect_steps(&img, 0.1);
        assert!(edges.is_empty(), "Flat image should have no steps");
    }

    #[test]
    fn test_step_height_histogram() {
        let edges = vec![
            StepEdge { start: (0, 0), end: (1, 0), height_nm: 0.1 },
            StepEdge { start: (0, 0), end: (1, 0), height_nm: 0.15 },
            StepEdge { start: (0, 0), end: (1, 0), height_nm: 0.3 },
        ];
        let hist = StepEdgeDetector::step_height_histogram(&edges, 0.1);
        assert!(!hist.is_empty());
        let total_count: usize = hist.iter().map(|&(_, c)| c).sum();
        assert_eq!(total_count, 3);
    }

    #[test]
    fn test_terrace_width() {
        let img = make_step_image(20, 20);
        let tw = StepEdgeDetector::terrace_width(&img, (1.0, 0.0));
        assert!(tw > 0.0, "Terrace width should be positive");
    }

    // ---- DriftCorrection tests ----

    #[test]
    fn test_estimate_drift_no_drift() {
        // Use a non-periodic image (Gaussian bump) for unambiguous correlation
        let w = 16;
        let h = 16;
        let mut data = vec![0.0; w * h];
        for iy in 0..h {
            for ix in 0..w {
                let dx = ix as f64 - 8.0;
                let dy = iy as f64 - 8.0;
                data[iy * w + ix] = (-0.1 * (dx * dx + dy * dy)).exp();
            }
        }
        let img = StmImage::new(w, h, data, 0.1);
        let (dx, dy) = DriftCorrection::estimate_drift(&img, &img);
        assert!((dx).abs() < 1.5 && (dy).abs() < 1.5,
            "No drift expected: ({}, {})", dx, dy);
    }

    #[test]
    fn test_correct_linear_drift() {
        let img = make_sinusoidal_image(16, 16, 4.0);
        let corrected = DriftCorrection::correct_linear_drift(&img, 0.01, 0.0);
        assert_eq!(corrected.width, img.width);
        assert_eq!(corrected.height, img.height);
    }

    #[test]
    fn test_creep_correction() {
        let img = make_sinusoidal_image(16, 16, 4.0);
        let corrected = DriftCorrection::creep_correction(&img, 0.05);
        assert_eq!(corrected.data.len(), img.data.len());
    }

    // ---- HeightDistribution tests ----

    #[test]
    fn test_histogram_flat() {
        let img = make_flat_image(10, 10, 5.0);
        let hist = HeightDistribution::histogram(&img, 10);
        // All values in one bin
        let total: usize = hist.iter().map(|&(_, c)| c).sum();
        assert_eq!(total, 100);
    }

    #[test]
    fn test_histogram_bins() {
        let data: Vec<f64> = (0..100).map(|i| i as f64 / 100.0).collect();
        let img = StmImage::new(10, 10, data, 0.1);
        let hist = HeightDistribution::histogram(&img, 5);
        assert_eq!(hist.len(), 5);
        let total: usize = hist.iter().map(|&(_, c)| c).sum();
        assert_eq!(total, 100);
    }

    #[test]
    fn test_bearing_ratio() {
        let data: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let img = StmImage::new(10, 10, data, 0.1);
        let br = HeightDistribution::bearing_ratio(&img);
        assert!(!br.is_empty());
        // First entry should have ratio near 1, last near 0
        assert!(br.first().unwrap().1 > 0.5);
    }

    #[test]
    fn test_skewness_symmetric() {
        // Symmetric distribution => skewness ~ 0
        let data = vec![0.0, 1.0, 0.0, 1.0];
        let img = StmImage::new(2, 2, data, 0.1);
        let ssk = HeightDistribution::skewness(&img);
        assert!(ssk.abs() < 0.1, "Symmetric skewness = {}", ssk);
    }

    #[test]
    fn test_kurtosis_uniform() {
        // For uniform distribution, kurtosis should be < 3 (platykurtic)
        let data: Vec<f64> = (0..100).map(|i| i as f64 / 99.0).collect();
        let img = StmImage::new(10, 10, data, 0.1);
        let sku = HeightDistribution::kurtosis(&img);
        assert!(sku < 3.0, "Uniform kurtosis = {} (should be < 3)", sku);
    }

    #[test]
    fn test_correlation_length_periodic() {
        let img = make_sinusoidal_image(64, 64, 16.0);
        let cl = HeightDistribution::correlation_length(&img);
        assert!(cl > 0.0, "Correlation length should be positive");
    }

    #[test]
    fn test_correlation_length_uncorrelated() {
        // Random-like pattern: short correlation
        let mut data = vec![0.0; 64 * 64];
        let mut seed = 42u64;
        for d in &mut data {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            *d = (seed >> 33) as f64 / (1u64 << 31) as f64;
        }
        let img = StmImage::new(64, 64, data, 0.1);
        let cl = HeightDistribution::correlation_length(&img);
        // Random data should have short correlation
        assert!(cl < 5.0, "Random correlation length = {}", cl);
    }

    // ---- AtomTracker tests ----

    #[test]
    fn test_find_atoms_grid() {
        // Create a grid of Gaussian peaks
        let w = 40;
        let h = 40;
        let mut data = vec![0.0; w * h];
        let spacing = 10; // pixels
        let sigma = 1.5;

        for cy in (5..h).step_by(spacing) {
            for cx in (5..w).step_by(spacing) {
                for iy in 0..h {
                    for ix in 0..w {
                        let dx = ix as f64 - cx as f64;
                        let dy = iy as f64 - cy as f64;
                        data[iy * w + ix] += (-0.5 * (dx * dx + dy * dy) / (sigma * sigma)).exp();
                    }
                }
            }
        }

        let img = StmImage::new(w, h, data, 0.1);
        let atoms = AtomTracker::find_atoms(&img, 1.0);
        // We placed a ~4x4 grid
        assert!(atoms.len() >= 4, "Should find multiple atoms, found {}", atoms.len());
    }

    #[test]
    fn test_nearest_neighbor_distance() {
        let atoms = vec![(0.0, 0.0), (1.0, 0.0), (0.0, 1.0), (1.0, 1.0)];
        let nnd = AtomTracker::nearest_neighbor_distance(&atoms);
        assert!((nnd - 1.0).abs() < 0.01, "NND = {}", nnd);
    }

    #[test]
    fn test_nearest_neighbor_single() {
        let atoms = vec![(0.0, 0.0)];
        let nnd = AtomTracker::nearest_neighbor_distance(&atoms);
        assert_eq!(nnd, 0.0);
    }

    #[test]
    fn test_coordination_number_square() {
        let atoms = vec![
            (0.0, 0.0), (1.0, 0.0), (2.0, 0.0),
            (0.0, 1.0), (1.0, 1.0), (2.0, 1.0),
            (0.0, 2.0), (1.0, 2.0), (2.0, 2.0),
        ];
        let cn = AtomTracker::coordination_number(&atoms, 1.1);
        // Center atom (1,1) should have 4 neighbors
        assert_eq!(cn[4], 4, "Center coordination = {}", cn[4]);
        // Corner atom (0,0) should have 2 neighbors
        assert_eq!(cn[0], 2, "Corner coordination = {}", cn[0]);
    }

    #[test]
    fn test_voronoi_area() {
        let atoms = vec![(0.0, 0.0), (2.0, 0.0), (0.0, 2.0), (2.0, 2.0)];
        let area = AtomTracker::voronoi_area(&atoms, 0);
        // Each atom gets ~1/4 of the 2x2 box = 1.0 nm^2
        assert!((area - 1.0).abs() < 0.2, "Voronoi area = {}", area);
    }

    // ---- PiezoCalibration tests ----

    #[test]
    fn test_calibrate_xy() {
        let scale = PiezoCalibration::calibrate_xy(0.28, 0.246);
        // Si(111) lattice constant 0.384 nm => measured 0.28 => scale = 0.246/0.28
        assert!((scale - 0.246 / 0.28).abs() < 1e-10);
    }

    #[test]
    fn test_calibrate_z() {
        let scale = PiezoCalibration::calibrate_z(0.35, 0.335);
        assert!((scale - 0.335 / 0.35).abs() < 1e-10);
    }

    #[test]
    fn test_hysteresis_correction() {
        let fwd = vec![1.0, 2.0, 3.0, 4.0];
        let bwd = vec![1.2, 2.4, 3.6, 4.8];
        let corrected = PiezoCalibration::hysteresis_correction(&fwd, &bwd);
        assert_eq!(corrected.len(), 4);
        assert!((corrected[0] - 1.1).abs() < 1e-10);
        assert!((corrected[1] - 2.2).abs() < 1e-10);
    }

    #[test]
    fn test_nonlinearity_correction() {
        // Linear correction: c0=0, c1=1.1
        let corrected = PiezoCalibration::nonlinearity_correction(5.0, &[0.0, 1.1]);
        assert!((corrected - 5.5).abs() < 1e-10);
    }

    #[test]
    fn test_nonlinearity_quadratic() {
        // Quadratic: c0=0, c1=1, c2=0.01
        let corrected = PiezoCalibration::nonlinearity_correction(10.0, &[0.0, 1.0, 0.01]);
        assert!((corrected - 11.0).abs() < 1e-10);
    }

    // ---- Helper function tests ----

    #[test]
    fn test_moving_average() {
        let data = vec![0.0, 0.0, 10.0, 0.0, 0.0];
        let smoothed = moving_average(&data, 2);
        // Center point: average of [0, 10, 0] = 3.33
        assert!((smoothed[2] - 10.0 / 3.0).abs() < 0.1);
    }

    #[test]
    fn test_dft_1d_dc() {
        let data = vec![1.0, 1.0, 1.0, 1.0];
        let (re, im) = dft_1d(&data);
        assert!((re[0] - 4.0).abs() < 1e-10);
        assert!(im[0].abs() < 1e-10);
        // All other bins should be zero
        for k in 1..4 {
            assert!(re[k].abs() < 1e-10, "re[{}] = {}", k, re[k]);
            assert!(im[k].abs() < 1e-10, "im[{}] = {}", k, im[k]);
        }
    }

    #[test]
    fn test_dft_idft_roundtrip() {
        let data = vec![1.0, 2.0, 3.0, 4.0];
        let (re, im) = dft_1d(&data);
        let (recovered, im_out) = idft_1d_complex(&re, &im);
        for i in 0..4 {
            assert!((recovered[i] - data[i]).abs() < 1e-10,
                "Roundtrip fail at {}: {} vs {}", i, recovered[i], data[i]);
            assert!(im_out[i].abs() < 1e-10);
        }
    }

    #[test]
    fn test_solve_linear_system_2x2() {
        // 2x+3y=8, x+y=3 => x=1, y=2
        let a = vec![2.0, 3.0, 1.0, 1.0];
        let b = vec![8.0, 3.0];
        let x = solve_linear_system(&a, &b, 2);
        assert!((x[0] - 1.0).abs() < 1e-10);
        assert!((x[1] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_poly_fit_linear() {
        // y = 2x + 1 at x=0,1,2,3
        let data = vec![1.0, 3.0, 5.0, 7.0];
        let coeffs = poly_fit_1d(&data, 1);
        assert!((coeffs[0] - 1.0).abs() < 1e-8, "c0 = {}", coeffs[0]);
        assert!((coeffs[1] - 2.0).abs() < 1e-8, "c1 = {}", coeffs[1]);
    }

    #[test]
    fn test_bilinear_sample_center() {
        let img = StmImage::new(3, 3, vec![
            0.0, 1.0, 2.0,
            3.0, 4.0, 5.0,
            6.0, 7.0, 8.0,
        ], 0.1);
        let val = bilinear_sample(&img, 1, 1, 0.0, 0.0);
        assert!((val - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_bilinear_sample_interpolated() {
        let img = StmImage::new(2, 2, vec![0.0, 1.0, 0.0, 1.0], 0.1);
        let val = bilinear_sample(&img, 0, 0, 0.5, 0.0);
        assert!((val - 0.5).abs() < 1e-10);
    }

    // ---- Integration tests ----

    #[test]
    fn test_full_pipeline() {
        // Create tilted image, flatten, analyze
        let img = make_tilted_image(20, 20);
        let flat = ImageFlattening::plane_subtraction(&img);
        let rms = flat.rms_roughness();
        assert!(rms < 1e-4);

        let hist = HeightDistribution::histogram(&flat, 10);
        let total: usize = hist.iter().map(|&(_, c)| c).sum();
        assert_eq!(total, 400);
    }

    #[test]
    fn test_sts_pipeline() {
        // Simulate I(V) curve and extract dI/dV
        let n = 101;
        let bias: Vec<f64> = (0..n).map(|i| -1.0 + 2.0 * i as f64 / (n - 1) as f64).collect();
        let phi = 4.0;
        let d = 0.5;
        let current: Vec<f64> = bias.iter().map(|&v| TunnelCurrentModel::current_wkb(v, d, phi)).collect();
        let sts = StsSpectroscopy::new(bias, current);
        let didv = sts.didv(3);
        assert_eq!(didv.len(), n);
    }

    #[test]
    fn test_lattice_vectors_not_empty() {
        let img = make_sinusoidal_image(16, 16, 4.0);
        let ps = FourierAnalysis::fft_2d(&img);
        let (v1, v2) = FourierAnalysis::lattice_vectors(&ps);
        // At least one vector should be nonzero
        let mag1 = (v1.0 * v1.0 + v1.1 * v1.1).sqrt();
        let mag2 = (v2.0 * v2.0 + v2.1 * v2.1).sqrt();
        assert!(mag1 > 0.0 || mag2 > 0.0, "Should find lattice vectors");
    }

    #[test]
    fn test_kurtosis_gaussian_like() {
        // Approximate Gaussian samples
        let mut data = vec![0.0; 10000];
        let mut seed = 12345u64;
        for d in &mut data {
            // Box-Muller-like: sum of 12 uniforms - 6
            let mut sum = 0.0;
            for _ in 0..12 {
                seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                let u = (seed >> 33) as f64 / (1u64 << 31) as f64;
                sum += u;
            }
            *d = sum - 6.0;
        }
        let img = StmImage::new(100, 100, data, 0.1);
        let sku = HeightDistribution::kurtosis(&img);
        // Gaussian kurtosis should be ~3
        assert!((sku - 3.0).abs() < 0.5, "Gaussian kurtosis = {}", sku);
    }

    #[test]
    fn test_skewness_positive() {
        // Right-skewed: mostly small values with a few large ones
        let mut data = vec![0.0; 100];
        for i in 0..90 { data[i] = 0.0; }
        for i in 90..100 { data[i] = 10.0; }
        let img = StmImage::new(10, 10, data, 0.1);
        let ssk = HeightDistribution::skewness(&img);
        assert!(ssk > 0.0, "Should be positively skewed, got {}", ssk);
    }

    #[test]
    fn test_coordination_number_empty() {
        let cn = AtomTracker::coordination_number(&[], 1.0);
        assert!(cn.is_empty());
    }

    #[test]
    fn test_voronoi_area_single() {
        let atoms = vec![(1.0, 1.0)];
        let area = AtomTracker::voronoi_area(&atoms, 0);
        assert_eq!(area, 0.0);
    }

    #[test]
    fn test_calibrate_xy_zero() {
        let scale = PiezoCalibration::calibrate_xy(0.0, 1.0);
        assert_eq!(scale, 1.0);
    }

    #[test]
    fn test_step_height_histogram_empty() {
        let hist = StepEdgeDetector::step_height_histogram(&[], 0.1);
        assert!(hist.is_empty());
    }

    #[test]
    fn test_polynomial_2d_order0() {
        let img = make_flat_image(10, 10, 7.0);
        let flat = ImageFlattening::polynomial_2d(&img, 0);
        for &z in &flat.data {
            assert!(z.abs() < 1e-6, "Order 0 poly should remove mean");
        }
    }

    #[test]
    fn test_line_by_line_preserves_size() {
        let img = make_tilted_image(15, 20);
        let flat = ImageFlattening::line_by_line(&img, 2);
        assert_eq!(flat.width, 15);
        assert_eq!(flat.height, 20);
        assert_eq!(flat.data.len(), 300);
    }
}
