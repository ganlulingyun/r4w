//! # Stellar Interferometry Processor
//!
//! Signal processing for stellar optical/radio interferometry enabling
//! high-resolution astronomical imaging far beyond the diffraction limit
//! of individual telescopes.
//!
//! ## Overview
//!
//! Interferometry combines signals from multiple telescopes separated by
//! a baseline vector **B** to achieve angular resolution proportional to
//! lambda/B_max, yielding milliarcsecond resolution at both radio and
//! optical wavelengths.
//!
//! ## Key Concepts
//!
//! - **Baseline**: Separation vector between two antenna/telescope elements
//! - **UV Plane**: Spatial frequency domain sampled by projected baselines
//! - **Visibility**: Complex Fourier component of the sky brightness distribution
//! - **Van Cittert-Zernike Theorem**: V(u,v) = FT{I(l,m)}
//! - **Closure Quantities**: Observables immune to antenna-based phase/gain errors
//! - **CLEAN Algorithm**: Iterative deconvolution for image reconstruction
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::stellar_interferometry_processor::*;
//!
//! let baseline = Baseline::new(1000.0, 500.0, 0.0);
//! let freq_hz = 1.4e9; // 21 cm hydrogen line
//! let lambda = SPEED_OF_LIGHT / freq_hz;
//! let resolution = angular_resolution_rad(lambda, baseline.length());
//! // ~milliarcsecond resolution for km-scale baselines at radio wavelengths
//! assert!(resolution < 1e-6);
//! ```

use std::f64::consts::PI;

/// Speed of light in m/s.
pub const SPEED_OF_LIGHT: f64 = 299_792_458.0;

/// Conversion factor: radians to arcseconds.
pub const RAD_TO_ARCSEC: f64 = 206_264.806_247;

/// Conversion factor: radians to milliarcseconds.
pub const RAD_TO_MAS: f64 = 206_264_806.247;

// ---------------------------------------------------------------------------
// Baseline
// ---------------------------------------------------------------------------

/// Baseline vector between two telescopes in a local East-North-Up frame.
#[derive(Debug, Clone, Copy)]
pub struct Baseline {
    /// East component in metres.
    pub east_m: f64,
    /// North component in metres.
    pub north_m: f64,
    /// Up (vertical) component in metres.
    pub up_m: f64,
}

impl Baseline {
    /// Create a new baseline.
    pub fn new(east_m: f64, north_m: f64, up_m: f64) -> Self {
        Self { east_m, north_m, up_m }
    }

    /// Total baseline length |B|.
    pub fn length(&self) -> f64 {
        (self.east_m * self.east_m + self.north_m * self.north_m + self.up_m * self.up_m).sqrt()
    }

    /// Projected baseline length toward a source at given hour angle and
    /// declination (both in radians).
    ///
    /// B_proj = sqrt(u^2 + v^2) where (u,v) are the UV coordinates.
    pub fn projected_length(&self, hour_angle_rad: f64, dec_rad: f64) -> f64 {
        let (u, v) = self.uv_coords(hour_angle_rad, dec_rad);
        (u * u + v * v).sqrt()
    }

    /// Compute UV coordinates (in metres) for a source at the given hour
    /// angle and declination (both in radians).
    ///
    /// u =  B_e sin(HA) + B_n cos(HA)
    /// v = -B_e sin(dec) cos(HA) + B_n sin(dec) sin(HA) + B_up cos(dec)
    pub fn uv_coords(&self, hour_angle_rad: f64, dec_rad: f64) -> (f64, f64) {
        let (sin_ha, cos_ha) = (hour_angle_rad.sin(), hour_angle_rad.cos());
        let (sin_dec, cos_dec) = (dec_rad.sin(), dec_rad.cos());

        let u = self.east_m * sin_ha + self.north_m * cos_ha;
        let v = -self.east_m * sin_dec * cos_ha
            + self.north_m * sin_dec * sin_ha
            + self.up_m * cos_dec;
        (u, v)
    }

    /// UV coordinates expressed in wavelengths.
    pub fn uv_lambda(
        &self,
        hour_angle_rad: f64,
        dec_rad: f64,
        wavelength_m: f64,
    ) -> (f64, f64) {
        let (u, v) = self.uv_coords(hour_angle_rad, dec_rad);
        (u / wavelength_m, v / wavelength_m)
    }
}

// ---------------------------------------------------------------------------
// Visibility
// ---------------------------------------------------------------------------

/// A single visibility measurement in the UV plane.
#[derive(Debug, Clone, Copy)]
pub struct VisibilityData {
    /// U coordinate in wavelengths.
    pub u_lambda: f64,
    /// V coordinate in wavelengths.
    pub v_lambda: f64,
    /// Visibility amplitude (fringe contrast).
    pub amplitude: f64,
    /// Visibility phase in radians.
    pub phase: f64,
}

impl VisibilityData {
    pub fn new(u_lambda: f64, v_lambda: f64, amplitude: f64, phase: f64) -> Self {
        Self { u_lambda, v_lambda, amplitude, phase }
    }

    /// Real part of the complex visibility.
    pub fn real(&self) -> f64 {
        self.amplitude * self.phase.cos()
    }

    /// Imaginary part of the complex visibility.
    pub fn imag(&self) -> f64 {
        self.amplitude * self.phase.sin()
    }

    /// UV radius in wavelengths: sqrt(u^2 + v^2).
    pub fn uv_radius(&self) -> f64 {
        (self.u_lambda * self.u_lambda + self.v_lambda * self.v_lambda).sqrt()
    }
}

// ---------------------------------------------------------------------------
// Angular resolution
// ---------------------------------------------------------------------------

/// Diffraction-limited angular resolution in radians: theta = lambda / B_max.
pub fn angular_resolution_rad(wavelength_m: f64, max_baseline_m: f64) -> f64 {
    wavelength_m / max_baseline_m
}

/// Angular resolution in arcseconds.
pub fn angular_resolution_arcsec(wavelength_m: f64, max_baseline_m: f64) -> f64 {
    angular_resolution_rad(wavelength_m, max_baseline_m) * RAD_TO_ARCSEC
}

/// Angular resolution in milliarcseconds.
pub fn angular_resolution_mas(wavelength_m: f64, max_baseline_m: f64) -> f64 {
    angular_resolution_rad(wavelength_m, max_baseline_m) * RAD_TO_MAS
}

// ---------------------------------------------------------------------------
// Fringe pattern
// ---------------------------------------------------------------------------

/// Two-element interferometer fringe pattern:
///   I(theta) = I0 * (1 + V_amp * cos(2*pi*B*theta/lambda + phase))
///
/// Returns intensity at angular offset `theta_rad` from fringe center.
pub fn fringe_intensity(
    i0: f64,
    visibility_amplitude: f64,
    baseline_m: f64,
    wavelength_m: f64,
    theta_rad: f64,
    phase_offset: f64,
) -> f64 {
    let arg = 2.0 * PI * baseline_m * theta_rad / wavelength_m + phase_offset;
    i0 * (1.0 + visibility_amplitude * arg.cos())
}

/// Fringe visibility (contrast) from measured max/min intensities:
///   V = (I_max - I_min) / (I_max + I_min)
pub fn fringe_visibility(i_max: f64, i_min: f64) -> f64 {
    if (i_max + i_min).abs() < 1e-30 {
        return 0.0;
    }
    (i_max - i_min) / (i_max + i_min)
}

// ---------------------------------------------------------------------------
// Correlator
// ---------------------------------------------------------------------------

/// Cross-correlate two real-valued signals and return the lag spectrum R(tau).
///
/// `R[k] = (1/N) * sum_{n} x1[n] * x2[n+k]`  for lags k in \[-max_lag, max_lag\].
///
/// Returns a vector of length `2*max_lag + 1`.
pub fn cross_correlate(x1: &[f64], x2: &[f64], max_lag: usize) -> Vec<f64> {
    let n = x1.len().min(x2.len());
    let len = 2 * max_lag + 1;
    let mut result = vec![0.0; len];
    for (i, lag_val) in result.iter_mut().enumerate() {
        let lag = i as isize - max_lag as isize;
        let mut sum = 0.0;
        let mut count = 0u64;
        for j in 0..n {
            let k = j as isize + lag;
            if k >= 0 && (k as usize) < n {
                sum += x1[j] * x2[k as usize];
                count += 1;
            }
        }
        if count > 0 {
            *lag_val = sum / count as f64;
        }
    }
    result
}

/// Extract complex visibility from a correlation lag spectrum.
///
/// Finds the peak of |R(tau)| and returns (amplitude, phase_rad, peak_lag_index).
pub fn visibility_from_correlation(lag_spectrum: &[f64]) -> (f64, f64, usize) {
    if lag_spectrum.is_empty() {
        return (0.0, 0.0, 0);
    }
    let mut peak_idx = 0;
    let mut peak_val = lag_spectrum[0].abs();
    for (i, &v) in lag_spectrum.iter().enumerate() {
        if v.abs() > peak_val {
            peak_val = v.abs();
            peak_idx = i;
        }
    }
    let amplitude = peak_val;
    let phase = if lag_spectrum[peak_idx] >= 0.0 { 0.0 } else { PI };
    (amplitude, phase, peak_idx)
}

/// Bandwidth smearing factor: sinc(delta_nu * tau) where delta_nu is
/// channel bandwidth and tau is the geometric delay.
///
/// Returns the multiplicative decorrelation factor in [0, 1].
pub fn bandwidth_smearing(channel_bandwidth_hz: f64, geometric_delay_s: f64) -> f64 {
    let x = PI * channel_bandwidth_hz * geometric_delay_s;
    if x.abs() < 1e-12 {
        1.0
    } else {
        (x.sin() / x).abs()
    }
}

// ---------------------------------------------------------------------------
// UV coverage
// ---------------------------------------------------------------------------

/// Generate UV track points for Earth-rotation synthesis.
///
/// As the Earth rotates, the projected baseline traces an ellipse in the UV
/// plane. Returns a vector of (u, v) in metres for the given hour-angle range.
pub fn uv_track(
    baseline: &Baseline,
    dec_rad: f64,
    ha_start_rad: f64,
    ha_end_rad: f64,
    num_points: usize,
) -> Vec<(f64, f64)> {
    if num_points < 2 {
        return vec![baseline.uv_coords(ha_start_rad, dec_rad)];
    }
    let step = (ha_end_rad - ha_start_rad) / (num_points - 1) as f64;
    (0..num_points)
        .map(|i| {
            let ha = ha_start_rad + i as f64 * step;
            baseline.uv_coords(ha, dec_rad)
        })
        .collect()
}

/// Snapshot UV coverage for an array of baselines at a single hour angle.
///
/// Each baseline contributes two points: (u,v) and (-u,-v) (Hermitian symmetry).
pub fn snapshot_uv_coverage(
    baselines: &[Baseline],
    hour_angle_rad: f64,
    dec_rad: f64,
) -> Vec<(f64, f64)> {
    let mut points = Vec::with_capacity(baselines.len() * 2);
    for b in baselines {
        let (u, v) = b.uv_coords(hour_angle_rad, dec_rad);
        points.push((u, v));
        points.push((-u, -v));
    }
    points
}

/// Compute UV plane sampling density on a grid.
///
/// Returns a `grid_size x grid_size` array where each cell counts how many
/// visibility points fall within it. `uv_max` sets the extent of the grid
/// from -uv_max to +uv_max in both axes.
pub fn uv_sampling_density(
    uv_points: &[(f64, f64)],
    grid_size: usize,
    uv_max: f64,
) -> Vec<Vec<u32>> {
    let mut grid = vec![vec![0u32; grid_size]; grid_size];
    let scale = grid_size as f64 / (2.0 * uv_max);
    for &(u, v) in uv_points {
        let col = ((u + uv_max) * scale) as isize;
        let row = ((v + uv_max) * scale) as isize;
        if col >= 0 && (col as usize) < grid_size && row >= 0 && (row as usize) < grid_size {
            grid[row as usize][col as usize] += 1;
        }
    }
    grid
}

// ---------------------------------------------------------------------------
// Simple 2D DFT / inverse DFT for image reconstruction
// ---------------------------------------------------------------------------

/// Grid visibilities onto a 2D UV grid and compute the dirty image via
/// inverse DFT.
///
/// `visibilities`: slice of `VisibilityData`.
/// `image_size`: output image is `image_size x image_size` pixels.
/// `uv_max_lambda`: extent of UV grid (−uv_max .. +uv_max in each axis).
///
/// Returns (dirty_image, dirty_beam) each as row-major `Vec<f64>` of
/// length `image_size * image_size`.
pub fn make_dirty_image(
    visibilities: &[VisibilityData],
    image_size: usize,
    uv_max_lambda: f64,
) -> (Vec<f64>, Vec<f64>) {
    let n = image_size;
    let nn = n * n;

    // Grid visibilities (real and imaginary) and sampling function
    let mut grid_re = vec![0.0f64; nn];
    let mut grid_im = vec![0.0f64; nn];
    let mut grid_wt = vec![0.0f64; nn];

    let scale = n as f64 / (2.0 * uv_max_lambda);
    for vis in visibilities {
        let col = ((vis.u_lambda + uv_max_lambda) * scale).round() as isize;
        let row = ((vis.v_lambda + uv_max_lambda) * scale).round() as isize;
        if col >= 0 && (col as usize) < n && row >= 0 && (row as usize) < n {
            let idx = row as usize * n + col as usize;
            grid_re[idx] += vis.real();
            grid_im[idx] += vis.imag();
            grid_wt[idx] += 1.0;
        }
        // Hermitian conjugate
        let col2 = ((-vis.u_lambda + uv_max_lambda) * scale).round() as isize;
        let row2 = ((-vis.v_lambda + uv_max_lambda) * scale).round() as isize;
        if col2 >= 0 && (col2 as usize) < n && row2 >= 0 && (row2 as usize) < n {
            let idx2 = row2 as usize * n + col2 as usize;
            grid_re[idx2] += vis.real();
            grid_im[idx2] -= vis.imag(); // conjugate
            grid_wt[idx2] += 1.0;
        }
    }

    // Normalize gridded visibilities
    for i in 0..nn {
        if grid_wt[i] > 0.0 {
            grid_re[i] /= grid_wt[i];
            grid_im[i] /= grid_wt[i];
        }
    }

    // Dirty image = inverse 2D DFT of gridded visibility
    let dirty_image = inverse_dft_2d(&grid_re, &grid_im, n);

    // Dirty beam = inverse 2D DFT of sampling function (weight grid)
    let wt_im = vec![0.0f64; nn];
    // Normalize weights for beam: set sampled cells to 1
    let mut wt_re = vec![0.0f64; nn];
    for i in 0..nn {
        wt_re[i] = if grid_wt[i] > 0.0 { 1.0 } else { 0.0 };
    }
    let dirty_beam = inverse_dft_2d(&wt_re, &wt_im, n);

    (dirty_image, dirty_beam)
}

/// Simple 2D inverse DFT (not FFT – O(N^4) but correct for small grids).
///
/// Input: row-major real and imaginary grids of size `n x n`.
/// Output: row-major real image of size `n x n`.
fn inverse_dft_2d(re: &[f64], im: &[f64], n: usize) -> Vec<f64> {
    let nn = n * n;
    let mut image = vec![0.0f64; nn];
    let n_f = n as f64;
    for iy in 0..n {
        for ix in 0..n {
            let mut sum_re = 0.0;
            for ky in 0..n {
                for kx in 0..n {
                    let idx = ky * n + kx;
                    let phase =
                        2.0 * PI * (kx as f64 * ix as f64 / n_f + ky as f64 * iy as f64 / n_f);
                    let (sin_p, cos_p) = phase.sin_cos();
                    // IDFT: multiply by exp(+j*phase)
                    sum_re += re[idx] * cos_p - im[idx] * sin_p;
                }
            }
            image[iy * n + ix] = sum_re / (n_f * n_f);
        }
    }
    image
}

// ---------------------------------------------------------------------------
// CLEAN (Hogbom)
// ---------------------------------------------------------------------------

/// Result of the CLEAN deconvolution algorithm.
#[derive(Debug, Clone)]
pub struct CleanResult {
    /// Positions (row, col) and flux of each clean component.
    pub components: Vec<(usize, usize, f64)>,
    /// Residual image after CLEAN (row-major, n*n).
    pub residual: Vec<f64>,
    /// Restored image: clean components convolved with clean beam + residual.
    pub restored: Vec<f64>,
    /// Number of iterations performed.
    pub iterations: usize,
}

/// Run the Hogbom CLEAN algorithm.
///
/// * `dirty_image` – row-major n*n dirty image.
/// * `dirty_beam` – row-major n*n dirty beam (PSF).
/// * `n` – image dimension.
/// * `loop_gain` – fraction of peak subtracted per iteration (typically 0.1).
/// * `threshold` – stop when peak residual < threshold.
/// * `max_iter` – maximum number of iterations.
/// * `clean_beam_sigma` – sigma of Gaussian clean beam in pixels (0 = skip restore).
pub fn clean_hogbom(
    dirty_image: &[f64],
    dirty_beam: &[f64],
    n: usize,
    loop_gain: f64,
    threshold: f64,
    max_iter: usize,
    clean_beam_sigma: f64,
) -> CleanResult {
    let nn = n * n;
    let mut residual = dirty_image.to_vec();
    let mut components: Vec<(usize, usize, f64)> = Vec::new();

    // Find beam peak (should be at center) for normalization
    let beam_peak = dirty_beam.iter().cloned().fold(0.0f64, f64::max).max(1e-30);

    let center = n / 2;
    let mut iter = 0;
    while iter < max_iter {
        // Find peak in residual
        let (peak_idx, peak_val) = find_abs_peak(&residual);
        if peak_val.abs() < threshold {
            break;
        }
        let peak_row = peak_idx / n;
        let peak_col = peak_idx % n;

        let component_flux = loop_gain * peak_val;
        components.push((peak_row, peak_col, component_flux));

        // Subtract shifted and scaled dirty beam
        for by in 0..n {
            for bx in 0..n {
                let dy = by as isize - center as isize;
                let dx = bx as isize - center as isize;
                let ry = peak_row as isize + dy;
                let rx = peak_col as isize + dx;
                if ry >= 0 && (ry as usize) < n && rx >= 0 && (rx as usize) < n {
                    let beam_val = dirty_beam[by * n + bx] / beam_peak;
                    residual[ry as usize * n + rx as usize] -= component_flux * beam_val;
                }
            }
        }
        iter += 1;
    }

    // Restore: convolve components with clean beam + add residual
    let restored = if clean_beam_sigma > 0.0 {
        let mut restored = residual.clone();
        for &(row, col, flux) in &components {
            add_gaussian(&mut restored, n, row, col, flux, clean_beam_sigma);
        }
        restored
    } else {
        residual.clone()
    };

    CleanResult { components, residual, restored, iterations: iter }
}

/// Find the index and value of the absolute-maximum element.
fn find_abs_peak(data: &[f64]) -> (usize, f64) {
    let mut idx = 0;
    let mut val = 0.0f64;
    for (i, &v) in data.iter().enumerate() {
        if v.abs() > val.abs() {
            val = v;
            idx = i;
        }
    }
    (idx, val)
}

/// Add a Gaussian to an image at (row, col) with given flux and sigma.
fn add_gaussian(image: &mut [f64], n: usize, row: usize, col: usize, flux: f64, sigma: f64) {
    let radius = (3.0 * sigma).ceil() as isize;
    let two_sigma_sq = 2.0 * sigma * sigma;
    for dy in -radius..=radius {
        for dx in -radius..=radius {
            let ry = row as isize + dy;
            let rx = col as isize + dx;
            if ry >= 0 && (ry as usize) < n && rx >= 0 && (rx as usize) < n {
                let r2 = (dy * dy + dx * dx) as f64;
                let g = (-r2 / two_sigma_sq).exp();
                image[ry as usize * n + rx as usize] += flux * g;
            }
        }
    }
}

/// Fit a Gaussian to the central lobe of the dirty beam, returning sigma in pixels.
pub fn fit_clean_beam_sigma(dirty_beam: &[f64], n: usize) -> f64 {
    let center = n / 2;
    let peak = dirty_beam[center * n + center];
    if peak.abs() < 1e-30 {
        return 1.0;
    }
    // Find half-power radius along the row through center
    let mut half_r = 1.0;
    for dx in 1..n / 2 {
        let val = dirty_beam[center * n + center + dx] / peak;
        if val < 0.5 {
            // Interpolate
            let prev = dirty_beam[center * n + center + dx - 1] / peak;
            let frac = (prev - 0.5) / (prev - val);
            half_r = (dx - 1) as f64 + frac;
            break;
        }
    }
    // FWHM = 2 * half_r, sigma = FWHM / (2*sqrt(2*ln2))
    let fwhm = 2.0 * half_r;
    fwhm / (2.0 * (2.0_f64.ln() * 2.0).sqrt())
}

// ---------------------------------------------------------------------------
// Closure quantities
// ---------------------------------------------------------------------------

/// Closure phase for a triangle of baselines 1-2, 2-3, 3-1.
///
/// phi_123 = phi_12 + phi_23 + phi_31
///
/// The result is wrapped to [-pi, pi].
pub fn closure_phase(phi_12: f64, phi_23: f64, phi_31: f64) -> f64 {
    wrap_phase(phi_12 + phi_23 + phi_31)
}

/// Closure amplitude for a quadrilateral of baselines.
///
/// A_1234 = |V_12 * V_34| / |V_13 * V_24|
pub fn closure_amplitude(
    amp_12: f64,
    amp_34: f64,
    amp_13: f64,
    amp_24: f64,
) -> f64 {
    let denom = amp_13 * amp_24;
    if denom.abs() < 1e-30 {
        return 0.0;
    }
    (amp_12 * amp_34).abs() / denom.abs()
}

/// Wrap a phase angle to the range [-pi, pi].
pub fn wrap_phase(phase: f64) -> f64 {
    let mut p = phase % (2.0 * PI);
    if p > PI {
        p -= 2.0 * PI;
    }
    if p < -PI {
        p += 2.0 * PI;
    }
    p
}

// ---------------------------------------------------------------------------
// Source models
// ---------------------------------------------------------------------------

/// Point source visibility: constant amplitude, zero phase.
pub fn visibility_point_source(flux: f64) -> (f64, f64) {
    (flux, 0.0)
}

/// Uniform disk visibility:
///   V = flux * 2 * J1(pi * theta_d * q) / (pi * theta_d * q)
/// where q = sqrt(u^2 + v^2) in wavelengths and theta_d is the angular
/// diameter in radians.
pub fn visibility_uniform_disk(
    flux: f64,
    angular_diameter_rad: f64,
    u_lambda: f64,
    v_lambda: f64,
) -> (f64, f64) {
    let q = (u_lambda * u_lambda + v_lambda * v_lambda).sqrt();
    let x = PI * angular_diameter_rad * q;
    let amp = if x.abs() < 1e-12 {
        flux
    } else {
        flux * 2.0 * bessel_j1(x) / x
    };
    (amp.abs(), if amp < 0.0 { PI } else { 0.0 })
}

/// Gaussian source visibility:
///   V = flux * exp(-pi^2 * sigma^2 * (u^2 + v^2))
/// where sigma is the Gaussian width in radians.
pub fn visibility_gaussian(
    flux: f64,
    sigma_rad: f64,
    u_lambda: f64,
    v_lambda: f64,
) -> (f64, f64) {
    let q2 = u_lambda * u_lambda + v_lambda * v_lambda;
    let amp = flux * (-PI * PI * sigma_rad * sigma_rad * q2).exp();
    (amp, 0.0)
}

/// Binary source visibility:
///   V = (F1 + F2 * exp(-2*pi*j*(u*dx + v*dy))) / (F1 + F2)
/// where dx, dy are the angular separation in radians.
///
/// Returns (amplitude, phase).
pub fn visibility_binary(
    flux1: f64,
    flux2: f64,
    delta_ra_rad: f64,
    delta_dec_rad: f64,
    u_lambda: f64,
    v_lambda: f64,
) -> (f64, f64) {
    let total = flux1 + flux2;
    if total.abs() < 1e-30 {
        return (0.0, 0.0);
    }
    let arg = -2.0 * PI * (u_lambda * delta_ra_rad + v_lambda * delta_dec_rad);
    let re = flux1 + flux2 * arg.cos();
    let im = flux2 * arg.sin();
    let amp = (re * re + im * im).sqrt() / total;
    let phase = im.atan2(re);
    (amp, phase)
}

// ---------------------------------------------------------------------------
// Bessel J1
// ---------------------------------------------------------------------------

/// Bessel function of the first kind J1(x), computed from a polynomial
/// approximation (Abramowitz & Stegun).
pub fn bessel_j1(x: f64) -> f64 {
    let ax = x.abs();
    if ax < 8.0 {
        let y = x * x;
        let num = x
            * (72362614232.0
                + y * (-7895059235.0
                    + y * (242396853.1
                        + y * (-2972611.439
                            + y * (15704.4826 + y * (-30.16036606))))));
        let den = 144725228442.0
            + y * (2300535178.0
                + y * (18583304.74
                    + y * (99447.43394 + y * (376.9991397 + y))));
        num / den
    } else {
        let z = 8.0 / ax;
        let y = z * z;
        let xx = ax - 2.356194491;
        let p = 1.0
            + y * (0.183105e-2
                + y * (-0.3516396496e-4
                    + y * (0.2457520174e-5 + y * (-0.240337019e-6))));
        let q = 0.04687499995
            + y * (-0.2002690873e-3
                + y * (0.8449199096e-5
                    + y * (-0.88228987e-6 + y * 0.105787412e-6)));
        let ans = (0.636619772 / ax).sqrt() * (xx.cos() * p - z * xx.sin() * q);
        if x < 0.0 {
            -ans
        } else {
            ans
        }
    }
}

// ---------------------------------------------------------------------------
// Stellar Interferometry Processor
// ---------------------------------------------------------------------------

/// Configuration for the interferometry processor.
#[derive(Debug, Clone)]
pub struct InterferometryConfig {
    /// Observing wavelength in metres.
    pub wavelength_m: f64,
    /// Image size for reconstruction (pixels per side).
    pub image_size: usize,
    /// UV grid extent in wavelengths.
    pub uv_max_lambda: f64,
    /// CLEAN loop gain.
    pub clean_loop_gain: f64,
    /// CLEAN threshold (stop criterion).
    pub clean_threshold: f64,
    /// Maximum CLEAN iterations.
    pub clean_max_iter: usize,
}

impl Default for InterferometryConfig {
    fn default() -> Self {
        Self {
            wavelength_m: SPEED_OF_LIGHT / 1.4e9, // 21 cm
            image_size: 32,
            uv_max_lambda: 1000.0,
            clean_loop_gain: 0.1,
            clean_threshold: 0.01,
            clean_max_iter: 100,
        }
    }
}

/// Main processor combining visibility generation, UV coverage, and imaging.
pub struct StellarInterferometryProcessor {
    pub config: InterferometryConfig,
    pub baselines: Vec<Baseline>,
    pub visibilities: Vec<VisibilityData>,
}

impl StellarInterferometryProcessor {
    pub fn new(config: InterferometryConfig) -> Self {
        Self {
            config,
            baselines: Vec::new(),
            visibilities: Vec::new(),
        }
    }

    /// Add a baseline to the array.
    pub fn add_baseline(&mut self, baseline: Baseline) {
        self.baselines.push(baseline);
    }

    /// Generate all baselines for an array of antenna positions (East, North, Up).
    pub fn set_array(&mut self, positions: &[(f64, f64, f64)]) {
        self.baselines.clear();
        for i in 0..positions.len() {
            for j in (i + 1)..positions.len() {
                let b = Baseline::new(
                    positions[j].0 - positions[i].0,
                    positions[j].1 - positions[i].1,
                    positions[j].2 - positions[i].2,
                );
                self.baselines.push(b);
            }
        }
    }

    /// Number of baselines.
    pub fn num_baselines(&self) -> usize {
        self.baselines.len()
    }

    /// Observe a source at given hour angle and declination, generating visibilities
    /// from all baselines using a model visibility function.
    ///
    /// `model_fn(u_lambda, v_lambda) -> (amplitude, phase)`
    pub fn observe<F>(&mut self, hour_angle_rad: f64, dec_rad: f64, model_fn: F)
    where
        F: Fn(f64, f64) -> (f64, f64),
    {
        for b in &self.baselines {
            let (u, v) = b.uv_lambda(hour_angle_rad, dec_rad, self.config.wavelength_m);
            let (amp, phase) = model_fn(u, v);
            self.visibilities.push(VisibilityData::new(u, v, amp, phase));
        }
    }

    /// Observe over an hour-angle range (Earth-rotation synthesis).
    pub fn observe_synthesis<F>(
        &mut self,
        dec_rad: f64,
        ha_start: f64,
        ha_end: f64,
        num_snapshots: usize,
        model_fn: F,
    ) where
        F: Fn(f64, f64) -> (f64, f64),
    {
        if num_snapshots == 0 {
            return;
        }
        let step = if num_snapshots > 1 {
            (ha_end - ha_start) / (num_snapshots - 1) as f64
        } else {
            0.0
        };
        for i in 0..num_snapshots {
            let ha = ha_start + i as f64 * step;
            self.observe(ha, dec_rad, &model_fn);
        }
    }

    /// Compute angular resolution of the array.
    pub fn angular_resolution_rad(&self) -> f64 {
        let max_b = self
            .baselines
            .iter()
            .map(|b| b.length())
            .fold(0.0f64, f64::max);
        if max_b < 1e-12 {
            return f64::INFINITY;
        }
        angular_resolution_rad(self.config.wavelength_m, max_b)
    }

    /// Get all UV points (in wavelengths) from current visibilities.
    pub fn uv_points_lambda(&self) -> Vec<(f64, f64)> {
        self.visibilities.iter().map(|v| (v.u_lambda, v.v_lambda)).collect()
    }

    /// Run dirty image + CLEAN and return the `CleanResult`.
    pub fn image_clean(&self) -> CleanResult {
        let (dirty_image, dirty_beam) = make_dirty_image(
            &self.visibilities,
            self.config.image_size,
            self.config.uv_max_lambda,
        );
        let sigma = fit_clean_beam_sigma(&dirty_beam, self.config.image_size);
        clean_hogbom(
            &dirty_image,
            &dirty_beam,
            self.config.image_size,
            self.config.clean_loop_gain,
            self.config.clean_threshold,
            self.config.clean_max_iter,
            sigma,
        )
    }

    /// Clear all accumulated visibilities.
    pub fn clear_visibilities(&mut self) {
        self.visibilities.clear();
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;

    // --- Baseline ---

    #[test]
    fn test_baseline_length() {
        let b = Baseline::new(3.0, 4.0, 0.0);
        assert!((b.length() - 5.0).abs() < TOL);
    }

    #[test]
    fn test_baseline_length_3d() {
        let b = Baseline::new(1.0, 2.0, 2.0);
        assert!((b.length() - 3.0).abs() < TOL);
    }

    #[test]
    fn test_uv_coords_zero_ha() {
        // At HA=0, u = B_e*0 + B_n*1 = B_n, v depends on dec
        let b = Baseline::new(100.0, 200.0, 0.0);
        let (u, v) = b.uv_coords(0.0, 0.0);
        assert!((u - 200.0).abs() < TOL); // B_n * cos(0)
        assert!((v - 0.0).abs() < TOL); // -B_e*sin(0)*cos(0) + B_n*sin(0)*sin(0) + 0
    }

    #[test]
    fn test_uv_coords_ha_pi_half() {
        // At HA=pi/2: u = B_e*1 + B_n*0 = B_e
        let b = Baseline::new(100.0, 200.0, 0.0);
        let (u, _v) = b.uv_coords(PI / 2.0, 0.0);
        assert!((u - 100.0).abs() < TOL);
    }

    #[test]
    fn test_uv_lambda() {
        let b = Baseline::new(1000.0, 0.0, 0.0);
        let lambda = 0.21; // 21 cm
        let (u_l, _v_l) = b.uv_lambda(PI / 2.0, 0.0, lambda);
        // u = 1000 * sin(pi/2) = 1000, u_lambda = 1000/0.21
        assert!((u_l - 1000.0 / 0.21).abs() < 1.0);
    }

    #[test]
    fn test_projected_length() {
        let b = Baseline::new(100.0, 0.0, 0.0);
        // At HA=pi/2, dec=0: u=100, v=0 → proj = 100
        let p = b.projected_length(PI / 2.0, 0.0);
        assert!((p - 100.0).abs() < TOL);
    }

    // --- Visibility ---

    #[test]
    fn test_visibility_real_imag() {
        let v = VisibilityData::new(10.0, 20.0, 1.0, 0.0);
        assert!((v.real() - 1.0).abs() < TOL);
        assert!(v.imag().abs() < TOL);
    }

    #[test]
    fn test_visibility_uv_radius() {
        let v = VisibilityData::new(3.0, 4.0, 1.0, 0.0);
        assert!((v.uv_radius() - 5.0).abs() < TOL);
    }

    #[test]
    fn test_visibility_phase() {
        let v = VisibilityData::new(0.0, 0.0, 2.0, PI / 4.0);
        let re = v.real();
        let im = v.imag();
        assert!((re - 2.0 * (PI / 4.0).cos()).abs() < TOL);
        assert!((im - 2.0 * (PI / 4.0).sin()).abs() < TOL);
    }

    // --- Angular resolution ---

    #[test]
    fn test_angular_resolution_radio() {
        // 21 cm, 1 km baseline → ~0.21 mrad
        let res = angular_resolution_rad(0.21, 1000.0);
        assert!((res - 0.00021).abs() < 1e-6);
    }

    #[test]
    fn test_angular_resolution_optical() {
        // 500 nm, 100 m baseline → 5e-9 rad ≈ 1 mas
        let res = angular_resolution_mas(500e-9, 100.0);
        assert!(res < 2.0); // should be ~1 mas
        assert!(res > 0.5);
    }

    #[test]
    fn test_angular_resolution_arcsec() {
        let res = angular_resolution_arcsec(0.21, 1000.0);
        // 0.00021 rad * 206264.8 ≈ 43.3 arcsec
        assert!((res - 43.315).abs() < 1.0);
    }

    // --- Fringe ---

    #[test]
    fn test_fringe_intensity_center() {
        // At theta=0 with zero phase offset: I = I0*(1 + V)
        let i = fringe_intensity(1.0, 0.5, 100.0, 0.01, 0.0, 0.0);
        assert!((i - 1.5).abs() < TOL);
    }

    #[test]
    fn test_fringe_intensity_null() {
        // At theta where cos = -1: I = I0*(1 - V)
        let lambda = 1.0;
        let baseline = 1.0;
        // 2*pi*B*theta/lambda = pi → theta = 0.5
        let i = fringe_intensity(1.0, 1.0, baseline, lambda, 0.5, 0.0);
        assert!((i - 0.0).abs() < TOL); // I0*(1 + 1*cos(pi)) = 0
    }

    #[test]
    fn test_fringe_visibility() {
        let v = fringe_visibility(10.0, 6.0);
        assert!((v - 0.25).abs() < TOL);
    }

    #[test]
    fn test_fringe_visibility_full() {
        assert!((fringe_visibility(1.0, 0.0) - 1.0).abs() < TOL);
    }

    #[test]
    fn test_fringe_visibility_zero() {
        assert!((fringe_visibility(5.0, 5.0) - 0.0).abs() < TOL);
    }

    // --- Correlator ---

    #[test]
    fn test_cross_correlate_identical() {
        let x: Vec<f64> = (0..64).map(|i| (2.0 * PI * i as f64 / 16.0).sin()).collect();
        let r = cross_correlate(&x, &x, 4);
        assert_eq!(r.len(), 9);
        // Peak should be at lag 0 (center)
        let center = r[4];
        assert!(center > 0.0);
        assert!(center >= r[0]);
        assert!(center >= r[8]);
    }

    #[test]
    fn test_cross_correlate_shifted() {
        let n = 64;
        let x1: Vec<f64> = (0..n).map(|i| (2.0 * PI * i as f64 / 16.0).sin()).collect();
        let x2: Vec<f64> = (0..n).map(|i| (2.0 * PI * (i as f64 - 2.0) / 16.0).sin()).collect();
        let r = cross_correlate(&x1, &x2, 5);
        // Peak should be near lag=+2
        let peak_idx = r.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        let peak_lag = peak_idx as isize - 5;
        assert!((peak_lag - 2).abs() <= 1);
    }

    #[test]
    fn test_visibility_from_correlation() {
        let spectrum = vec![0.1, 0.3, 0.9, 0.3, 0.1];
        let (amp, phase, idx) = visibility_from_correlation(&spectrum);
        assert!((amp - 0.9).abs() < TOL);
        assert!((phase - 0.0).abs() < TOL);
        assert_eq!(idx, 2);
    }

    #[test]
    fn test_visibility_from_correlation_negative() {
        let spectrum = vec![0.1, -0.8, 0.2];
        let (amp, phase, idx) = visibility_from_correlation(&spectrum);
        assert!((amp - 0.8).abs() < TOL);
        assert!((phase - PI).abs() < TOL);
        assert_eq!(idx, 1);
    }

    #[test]
    fn test_bandwidth_smearing_no_delay() {
        let f = bandwidth_smearing(1e6, 0.0);
        assert!((f - 1.0).abs() < TOL);
    }

    #[test]
    fn test_bandwidth_smearing_half() {
        // sinc(x) = 0 at x = pi → bandwidth * delay = 1
        let f = bandwidth_smearing(1e6, 1e-6);
        assert!(f.abs() < 0.01); // sinc(pi) ≈ 0
    }

    // --- UV coverage ---

    #[test]
    fn test_uv_track_num_points() {
        let b = Baseline::new(1000.0, 500.0, 0.0);
        let track = uv_track(&b, 0.5, -PI, PI, 100);
        assert_eq!(track.len(), 100);
    }

    #[test]
    fn test_snapshot_uv_coverage_hermitian() {
        let baselines = vec![Baseline::new(100.0, 0.0, 0.0)];
        let pts = snapshot_uv_coverage(&baselines, 0.0, 0.0);
        assert_eq!(pts.len(), 2);
        // Hermitian: second point is negative of first
        assert!((pts[0].0 + pts[1].0).abs() < TOL);
        assert!((pts[0].1 + pts[1].1).abs() < TOL);
    }

    #[test]
    fn test_uv_sampling_density() {
        let pts = vec![(0.0, 0.0), (50.0, 50.0), (-50.0, -50.0)];
        let grid = uv_sampling_density(&pts, 10, 100.0);
        // Center cell should have the (0,0) point
        assert!(grid[5][5] > 0);
    }

    // --- Closure quantities ---

    #[test]
    fn test_closure_phase_zero() {
        // Three antennas seeing a point source → all phases zero
        let cp = closure_phase(0.0, 0.0, 0.0);
        assert!(cp.abs() < TOL);
    }

    #[test]
    fn test_closure_phase_cancellation() {
        // Atmospheric phases: phi_12 = a1-a2, phi_23 = a2-a3, phi_31 = a3-a1
        // Sum = 0 regardless of a1, a2, a3
        let a1 = 1.5;
        let a2 = -0.7;
        let a3 = 2.1;
        let cp = closure_phase(a1 - a2, a2 - a3, a3 - a1);
        assert!(cp.abs() < TOL);
    }

    #[test]
    fn test_closure_amplitude() {
        let ca = closure_amplitude(1.0, 1.0, 1.0, 1.0);
        assert!((ca - 1.0).abs() < TOL);
    }

    #[test]
    fn test_closure_amplitude_ratio() {
        let ca = closure_amplitude(2.0, 3.0, 1.0, 6.0);
        assert!((ca - 1.0).abs() < TOL); // 2*3 / (1*6) = 1.0
    }

    // --- Source models ---

    #[test]
    fn test_point_source_visibility() {
        let (amp, phase) = visibility_point_source(5.0);
        assert!((amp - 5.0).abs() < TOL);
        assert!(phase.abs() < TOL);
    }

    #[test]
    fn test_uniform_disk_zero_baseline() {
        // At zero baseline, V = flux
        let (amp, _phase) = visibility_uniform_disk(1.0, 0.001, 0.0, 0.0);
        assert!((amp - 1.0).abs() < TOL);
    }

    #[test]
    fn test_uniform_disk_first_null() {
        // First null of 2*J1(x)/x is at x ≈ 3.8317
        // pi * theta_d * q = 3.8317 → q = 3.8317 / (pi * theta_d)
        let theta_d = 0.01; // radians
        let q_null = 3.8317 / (PI * theta_d);
        let (amp, _) = visibility_uniform_disk(1.0, theta_d, q_null, 0.0);
        assert!(amp.abs() < 0.02); // near zero
    }

    #[test]
    fn test_gaussian_visibility_zero_baseline() {
        let (amp, phase) = visibility_gaussian(2.0, 0.001, 0.0, 0.0);
        assert!((amp - 2.0).abs() < TOL);
        assert!(phase.abs() < TOL);
    }

    #[test]
    fn test_gaussian_visibility_decay() {
        let sigma = 1e-6;
        let (a1, _) = visibility_gaussian(1.0, sigma, 100.0, 0.0);
        let (a2, _) = visibility_gaussian(1.0, sigma, 1000.0, 0.0);
        // Larger baseline → smaller visibility
        assert!(a1 > a2);
    }

    #[test]
    fn test_binary_visibility_zero_separation() {
        // Zero separation → same as point source
        let (amp, phase) = visibility_binary(1.0, 1.0, 0.0, 0.0, 100.0, 200.0);
        assert!((amp - 1.0).abs() < TOL);
        assert!(phase.abs() < TOL);
    }

    #[test]
    fn test_binary_visibility_null() {
        // Binary with equal flux, separation dx = lambda/(2*u) → null at u
        // V = (1 + exp(-2*pi*j*u*dx))/2 = 0 when 2*pi*u*dx = pi → u*dx = 0.5
        let dx = 0.5 / 100.0; // for u = 100
        let (amp, _) = visibility_binary(1.0, 1.0, dx, 0.0, 100.0, 0.0);
        assert!(amp.abs() < 0.05);
    }

    // --- Bessel J1 ---

    #[test]
    fn test_bessel_j1_zero() {
        assert!(bessel_j1(0.0).abs() < TOL);
    }

    #[test]
    fn test_bessel_j1_known_value() {
        // J1(1.0) ≈ 0.44005
        let j = bessel_j1(1.0);
        assert!((j - 0.44005).abs() < 0.001);
    }

    #[test]
    fn test_bessel_j1_large_arg() {
        // J1(10.0) ≈ 0.04348
        let j = bessel_j1(10.0);
        assert!((j - 0.04348).abs() < 0.001);
    }

    #[test]
    fn test_bessel_j1_negative() {
        // J1(-x) = -J1(x)
        let j_pos = bessel_j1(3.0);
        let j_neg = bessel_j1(-3.0);
        assert!((j_pos + j_neg).abs() < TOL);
    }

    // --- Wrap phase ---

    #[test]
    fn test_wrap_phase() {
        assert!((wrap_phase(0.0)).abs() < TOL);
        assert!((wrap_phase(3.0 * PI) - PI).abs() < TOL);
        assert!((wrap_phase(-3.0 * PI) + PI).abs() < TOL);
    }

    // --- CLEAN ---

    #[test]
    fn test_clean_point_source() {
        // Create a simple point source and verify CLEAN finds it
        let n = 16;
        let mut dirty_image = vec![0.0; n * n];
        dirty_image[8 * n + 8] = 1.0; // point source at center

        // Simple delta-function PSF
        let mut dirty_beam = vec![0.0; n * n];
        dirty_beam[8 * n + 8] = 1.0;

        let result = clean_hogbom(&dirty_image, &dirty_beam, n, 0.1, 0.01, 100, 0.0);
        assert!(!result.components.is_empty());
        // Total cleaned flux should be near 1.0
        let total_flux: f64 = result.components.iter().map(|c| c.2).sum();
        assert!((total_flux - 1.0).abs() < 0.1);
    }

    #[test]
    fn test_clean_iterations_bounded() {
        let n = 8;
        let dirty_image = vec![0.5; n * n]; // uniform
        let mut dirty_beam = vec![0.0; n * n];
        dirty_beam[4 * n + 4] = 1.0;
        let result = clean_hogbom(&dirty_image, &dirty_beam, n, 0.1, 0.01, 10, 0.0);
        assert!(result.iterations <= 10);
    }

    #[test]
    fn test_fit_clean_beam_sigma() {
        // Gaussian PSF with known width
        let n = 32;
        let center = n / 2;
        let sigma_true = 2.0;
        let mut beam = vec![0.0; n * n];
        for y in 0..n {
            for x in 0..n {
                let dy = y as f64 - center as f64;
                let dx = x as f64 - center as f64;
                beam[y * n + x] = (-(dx * dx + dy * dy) / (2.0 * sigma_true * sigma_true)).exp();
            }
        }
        let sigma_fit = fit_clean_beam_sigma(&beam, n);
        assert!((sigma_fit - sigma_true).abs() < 0.5);
    }

    // --- Processor integration ---

    #[test]
    fn test_processor_set_array() {
        let mut proc = StellarInterferometryProcessor::new(InterferometryConfig::default());
        let positions = vec![(0.0, 0.0, 0.0), (100.0, 0.0, 0.0), (0.0, 100.0, 0.0)];
        proc.set_array(&positions);
        // 3 antennas → 3 baselines
        assert_eq!(proc.num_baselines(), 3);
    }

    #[test]
    fn test_processor_observe_point_source() {
        let mut proc = StellarInterferometryProcessor::new(InterferometryConfig::default());
        proc.add_baseline(Baseline::new(1000.0, 0.0, 0.0));
        proc.observe(0.0, 0.5, |_u, _v| visibility_point_source(1.0));
        assert_eq!(proc.visibilities.len(), 1);
        assert!((proc.visibilities[0].amplitude - 1.0).abs() < TOL);
    }

    #[test]
    fn test_processor_earth_rotation_synthesis() {
        let mut proc = StellarInterferometryProcessor::new(InterferometryConfig::default());
        proc.add_baseline(Baseline::new(500.0, 300.0, 0.0));
        proc.observe_synthesis(0.5, -PI, PI, 24, |_u, _v| (1.0, 0.0));
        assert_eq!(proc.visibilities.len(), 24);
    }

    #[test]
    fn test_processor_angular_resolution() {
        let mut proc = StellarInterferometryProcessor::new(InterferometryConfig {
            wavelength_m: 0.21,
            ..Default::default()
        });
        proc.add_baseline(Baseline::new(1000.0, 0.0, 0.0));
        let res = proc.angular_resolution_rad();
        assert!((res - 0.00021).abs() < 1e-6);
    }

    #[test]
    fn test_processor_clear_visibilities() {
        let mut proc = StellarInterferometryProcessor::new(InterferometryConfig::default());
        proc.add_baseline(Baseline::new(100.0, 0.0, 0.0));
        proc.observe(0.0, 0.0, |_u, _v| (1.0, 0.0));
        assert_eq!(proc.visibilities.len(), 1);
        proc.clear_visibilities();
        assert_eq!(proc.visibilities.len(), 0);
    }

    #[test]
    fn test_processor_uv_points() {
        let mut proc = StellarInterferometryProcessor::new(InterferometryConfig::default());
        proc.add_baseline(Baseline::new(100.0, 200.0, 0.0));
        proc.observe(0.0, 0.0, |_u, _v| (1.0, 0.0));
        let pts = proc.uv_points_lambda();
        assert_eq!(pts.len(), 1);
    }

    #[test]
    fn test_processor_image_clean_runs() {
        let config = InterferometryConfig {
            image_size: 16,
            uv_max_lambda: 500.0,
            clean_max_iter: 10,
            ..Default::default()
        };
        let mut proc = StellarInterferometryProcessor::new(config);
        proc.add_baseline(Baseline::new(100.0, 0.0, 0.0));
        proc.add_baseline(Baseline::new(0.0, 100.0, 0.0));
        proc.observe_synthesis(0.5, -PI / 4.0, PI / 4.0, 8, |_u, _v| (1.0, 0.0));
        let result = proc.image_clean();
        assert_eq!(result.restored.len(), 16 * 16);
    }

    // --- Inverse DFT ---

    #[test]
    fn test_inverse_dft_dc() {
        // Constant DC in frequency domain → delta at origin in spatial domain
        let n = 4;
        let mut re = vec![0.0; n * n];
        re[0] = 1.0; // DC at (0,0)
        let im = vec![0.0; n * n];
        let image = inverse_dft_2d(&re, &im, n);
        // All pixels should be 1/(n*n)
        let expected = 1.0 / (n * n) as f64;
        for &v in &image {
            assert!((v - expected).abs() < TOL);
        }
    }
}
