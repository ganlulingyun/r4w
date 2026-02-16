//! Near-field Acoustic Holography (NAH) for sound source visualization.
//!
//! This module implements planar NAH using spatial Fourier transforms to
//! reconstruct sound fields at the source plane from microphone array
//! measurements. It supports forward and backward propagation with
//! regularization to handle evanescent wave amplification.
//!
//! # Theory
//!
//! Planar NAH is based on the angular spectrum method:
//!
//! ```text
//! P(kx, ky, z_s) = P(kx, ky, z_h) * G(kx, ky, dz)
//! ```
//!
//! where `G = exp(-j * kz * dz)` is the spatial propagator and
//! `kz = sqrt(k^2 - kx^2 - ky^2)`.
//!
//! For backward propagation (`z_s < z_h`), evanescent waves grow
//! exponentially, requiring regularization (Tikhonov, exponential filter,
//! or k-space cutoff).
//!
//! # Example
//!
//! ```rust
//! use r4w_core::acoustic_holography_processor::{
//!     MicArrayConfig, HologramData, propagate_planar, RegularizationMethod,
//! };
//!
//! let config = MicArrayConfig {
//!     num_mics_x: 8,
//!     num_mics_y: 8,
//!     spacing_m: 0.02,
//!     measurement_distance_m: 0.05,
//! };
//!
//! // Create a simple pressure field (point source-like)
//! let nx = config.num_mics_x;
//! let ny = config.num_mics_y;
//! let mut pressure = vec![(0.0, 0.0); nx * ny];
//! pressure[ny / 2 * nx + nx / 2] = (1.0, 0.0);
//!
//! let hologram = HologramData {
//!     pressure,
//!     nx,
//!     ny,
//!     frequency_hz: 1000.0,
//!     dx: config.spacing_m,
//!     dy: config.spacing_m,
//! };
//!
//! let result = propagate_planar(
//!     &hologram,
//!     -0.03,
//!     RegularizationMethod::Tikhonov(1e-3),
//! );
//! assert_eq!(result.pressure.len(), nx * ny);
//! ```

use std::f64::consts::PI;

// ─── Complex number helpers ───────────────────────────────────────────────

/// Complex number represented as (real, imaginary).
type Complex = (f64, f64);

#[inline]
fn c_add(a: Complex, b: Complex) -> Complex {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: Complex, b: Complex) -> Complex {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: Complex, b: Complex) -> Complex {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: Complex) -> Complex {
    (a.0, -a.1)
}

#[inline]
fn c_abs_sq(a: Complex) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn c_abs(a: Complex) -> f64 {
    c_abs_sq(a).sqrt()
}

#[inline]
fn c_scale(a: Complex, s: f64) -> Complex {
    (a.0 * s, a.1 * s)
}

#[inline]
fn c_exp(phase: f64) -> Complex {
    (phase.cos(), phase.sin())
}

#[inline]
fn c_div(a: Complex, b: Complex) -> Complex {
    let denom = c_abs_sq(b);
    if denom < 1e-30 {
        (0.0, 0.0)
    } else {
        ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
    }
}

// ─── FFT ──────────────────────────────────────────────────────────────────

/// In-place radix-2 Cooley-Tukey FFT. `data` length must be a power of 2.
/// `inverse` = true for IFFT.
fn fft_1d(data: &mut [Complex], inverse: bool) {
    let n = data.len();
    if n <= 1 {
        return;
    }
    assert!(n.is_power_of_two(), "FFT length must be power of 2");

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 0..n {
        if i < j {
            data.swap(i, j);
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Butterfly stages
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let wn = c_exp(angle);
        let mut start = 0;
        while start < n {
            let mut w: Complex = (1.0, 0.0);
            for k in 0..half {
                let u = data[start + k];
                let t = c_mul(w, data[start + k + half]);
                data[start + k] = c_add(u, t);
                data[start + k + half] = c_sub(u, t);
                w = c_mul(w, wn);
            }
            start += len;
        }
        len <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for x in data.iter_mut() {
            *x = c_scale(*x, inv_n);
        }
    }
}

/// 2D FFT on an `nx x ny` grid stored row-major. Sizes must be powers of 2.
fn fft_2d(data: &mut [Complex], nx: usize, ny: usize, inverse: bool) {
    assert_eq!(data.len(), nx * ny);

    // FFT along rows (y-direction)
    for row in 0..ny {
        let offset = row * nx;
        let mut row_buf: Vec<Complex> = data[offset..offset + nx].to_vec();
        fft_1d(&mut row_buf, inverse);
        data[offset..offset + nx].copy_from_slice(&row_buf);
    }

    // FFT along columns (x-direction)
    let mut col_buf = vec![(0.0, 0.0); ny];
    for col in 0..nx {
        for row in 0..ny {
            col_buf[row] = data[row * nx + col];
        }
        fft_1d(&mut col_buf, inverse);
        for row in 0..ny {
            data[row * nx + col] = col_buf[row];
        }
    }
}

/// Pad a 2D field to the next power of 2 in each dimension.
fn pad_to_pow2(data: &[Complex], nx: usize, ny: usize) -> (Vec<Complex>, usize, usize) {
    let nx2 = nx.next_power_of_two();
    let ny2 = ny.next_power_of_two();
    let mut padded = vec![(0.0, 0.0); nx2 * ny2];
    for row in 0..ny {
        for col in 0..nx {
            padded[row * nx2 + col] = data[row * nx + col];
        }
    }
    (padded, nx2, ny2)
}

/// Extract original-size data from padded grid.
fn unpad(data: &[Complex], nx_padded: usize, nx: usize, ny: usize) -> Vec<Complex> {
    let mut out = vec![(0.0, 0.0); nx * ny];
    for row in 0..ny {
        for col in 0..nx {
            out[row * nx + col] = data[row * nx_padded + col];
        }
    }
    out
}

// ─── Public types ─────────────────────────────────────────────────────────

/// Speed of sound in air (m/s).
pub const SPEED_OF_SOUND: f64 = 343.0;

/// Air density (kg/m^3).
pub const AIR_DENSITY: f64 = 1.225;

/// Microphone array configuration for holographic measurement.
#[derive(Debug, Clone)]
pub struct MicArrayConfig {
    /// Number of microphones along x-axis.
    pub num_mics_x: usize,
    /// Number of microphones along y-axis.
    pub num_mics_y: usize,
    /// Spacing between microphones (m).
    pub spacing_m: f64,
    /// Distance from source plane to measurement plane (m).
    pub measurement_distance_m: f64,
}

impl MicArrayConfig {
    /// Spatial Nyquist frequency: k_max = pi / spacing.
    pub fn k_nyquist(&self) -> f64 {
        PI / self.spacing_m
    }

    /// Maximum frequency resolvable without spatial aliasing.
    pub fn max_frequency_hz(&self) -> f64 {
        SPEED_OF_SOUND * self.k_nyquist() / (2.0 * PI)
    }

    /// Array aperture size (x, y) in metres.
    pub fn aperture(&self) -> (f64, f64) {
        (
            (self.num_mics_x as f64 - 1.0) * self.spacing_m,
            (self.num_mics_y as f64 - 1.0) * self.spacing_m,
        )
    }
}

/// 2D complex pressure field at a given plane.
#[derive(Debug, Clone)]
pub struct HologramData {
    /// Row-major complex pressure values (ny rows, nx columns).
    pub pressure: Vec<Complex>,
    /// Number of columns (x points).
    pub nx: usize,
    /// Number of rows (y points).
    pub ny: usize,
    /// Measurement frequency (Hz).
    pub frequency_hz: f64,
    /// Grid spacing in x (m).
    pub dx: f64,
    /// Grid spacing in y (m).
    pub dy: f64,
}

impl HologramData {
    /// Acoustic wavenumber k = 2*pi*f/c.
    pub fn wavenumber(&self) -> f64 {
        2.0 * PI * self.frequency_hz / SPEED_OF_SOUND
    }

    /// Pressure magnitude at each grid point.
    pub fn magnitude(&self) -> Vec<f64> {
        self.pressure.iter().map(|p| c_abs(*p)).collect()
    }

    /// Pressure phase at each grid point (radians).
    pub fn phase(&self) -> Vec<f64> {
        self.pressure.iter().map(|p| p.1.atan2(p.0)).collect()
    }

    /// Sound Pressure Level in dB re 20 uPa.
    pub fn spl_db(&self) -> Vec<f64> {
        let p_ref = 20e-6_f64;
        self.pressure
            .iter()
            .map(|p| {
                let mag = c_abs(*p);
                if mag < 1e-30 {
                    -300.0
                } else {
                    20.0 * (mag / p_ref).log10()
                }
            })
            .collect()
    }
}

/// Regularization method for backward propagation.
#[derive(Debug, Clone, Copy)]
pub enum RegularizationMethod {
    /// No regularization (forward propagation only).
    None,
    /// Exponential filter: W = exp(-alpha * max(0, kx^2 + ky^2 - k^2)).
    Exponential(f64),
    /// Tikhonov: W = |G|^2 / (|G|^2 + lambda).
    Tikhonov(f64),
    /// k-space cutoff at multiple of acoustic wavenumber.
    KSpaceCutoff(f64),
}

// ─── Wavenumber grid ──────────────────────────────────────────────────────

/// Generate wavenumber array for FFT output.
/// For N points with spacing dx, frequencies are [0, 1, ..., N/2-1, -N/2, ..., -1] * (2*pi/(N*dx)).
fn wavenumber_axis(n: usize, dx: f64) -> Vec<f64> {
    let dk = 2.0 * PI / (n as f64 * dx);
    (0..n)
        .map(|i| {
            if i <= n / 2 {
                i as f64 * dk
            } else {
                (i as f64 - n as f64) * dk
            }
        })
        .collect()
}

// ─── Planar NAH propagation ──────────────────────────────────────────────

/// Propagate a hologram by distance `dz` metres using the angular spectrum method.
///
/// `dz > 0` moves away from source (forward), `dz < 0` moves toward source (backward).
/// Backward propagation amplifies evanescent waves; use regularization.
pub fn propagate_planar(
    hologram: &HologramData,
    dz: f64,
    regularization: RegularizationMethod,
) -> HologramData {
    let k = hologram.wavenumber();
    let k_sq = k * k;

    let (mut padded, nx2, ny2) = pad_to_pow2(&hologram.pressure, hologram.nx, hologram.ny);

    // Forward 2D FFT
    fft_2d(&mut padded, nx2, ny2, false);

    let kx_axis = wavenumber_axis(nx2, hologram.dx);
    let ky_axis = wavenumber_axis(ny2, hologram.dy);

    // Apply propagator with regularization
    for iy in 0..ny2 {
        let ky = ky_axis[iy];
        for ix in 0..nx2 {
            let kx = kx_axis[ix];
            let kt_sq = kx * kx + ky * ky;

            let (propagator, reg_weight) = if kt_sq <= k_sq {
                // Propagating wave
                let kz = (k_sq - kt_sq).sqrt();
                (c_exp(-kz * dz), 1.0)
            } else {
                // Evanescent wave
                let kz_im = (kt_sq - k_sq).sqrt();
                // G = exp(-kz_im * |dz|) for forward, exp(kz_im * |dz|) for backward
                let g_mag = (-kz_im * dz.abs()).exp();
                let prop = if dz >= 0.0 {
                    // Forward: evanescent decays
                    (g_mag, 0.0)
                } else {
                    // Backward: evanescent grows
                    (1.0 / g_mag.max(1e-30), 0.0)
                };

                let w = match regularization {
                    RegularizationMethod::None => 1.0,
                    RegularizationMethod::Exponential(alpha) => {
                        (-alpha * (kt_sq - k_sq)).exp()
                    }
                    RegularizationMethod::Tikhonov(lambda) => {
                        let g_sq = c_abs_sq(prop);
                        g_sq / (g_sq + lambda)
                    }
                    RegularizationMethod::KSpaceCutoff(mult) => {
                        let k_cut = mult * k;
                        if kt_sq.sqrt() <= k_cut {
                            1.0
                        } else {
                            0.0
                        }
                    }
                };
                (prop, w)
            };

            let idx = iy * nx2 + ix;
            padded[idx] = c_scale(c_mul(padded[idx], propagator), reg_weight);
        }
    }

    // Inverse 2D FFT
    fft_2d(&mut padded, nx2, ny2, true);

    let pressure = unpad(&padded, nx2, hologram.nx, hologram.ny);

    HologramData {
        pressure,
        nx: hologram.nx,
        ny: hologram.ny,
        frequency_hz: hologram.frequency_hz,
        dx: hologram.dx,
        dy: hologram.dy,
    }
}

// ─── Sound field quantities ───────────────────────────────────────────────

/// Compute normal particle velocity from pressure hologram.
///
/// v_n = -1/(j * omega * rho) * dp/dz
///
/// In k-space: V_n(kx,ky) = kz / (omega * rho) * P(kx,ky)
pub fn compute_normal_velocity(
    hologram: &HologramData,
    regularization: RegularizationMethod,
) -> Vec<Complex> {
    let k = hologram.wavenumber();
    let k_sq = k * k;
    let omega = 2.0 * PI * hologram.frequency_hz;
    let rho = AIR_DENSITY;

    let (mut padded, nx2, ny2) = pad_to_pow2(&hologram.pressure, hologram.nx, hologram.ny);
    fft_2d(&mut padded, nx2, ny2, false);

    let kx_axis = wavenumber_axis(nx2, hologram.dx);
    let ky_axis = wavenumber_axis(ny2, hologram.dy);

    for iy in 0..ny2 {
        let ky = ky_axis[iy];
        for ix in 0..nx2 {
            let kx = kx_axis[ix];
            let kt_sq = kx * kx + ky * ky;

            let (vel_transfer, reg_weight) = if kt_sq <= k_sq {
                let kz = (k_sq - kt_sq).sqrt();
                // v_n = kz / (omega * rho) * P
                let scale = kz / (omega * rho);
                ((scale, 0.0), 1.0)
            } else {
                let kz_im = (kt_sq - k_sq).sqrt();
                // Evanescent: kz = j*kz_im, so v_n factor = j*kz_im/(omega*rho)
                let scale = kz_im / (omega * rho);
                let w = match regularization {
                    RegularizationMethod::None => 1.0,
                    RegularizationMethod::Exponential(alpha) => {
                        (-alpha * (kt_sq - k_sq)).exp()
                    }
                    RegularizationMethod::Tikhonov(lambda) => {
                        let g = scale;
                        g * g / (g * g + lambda)
                    }
                    RegularizationMethod::KSpaceCutoff(mult) => {
                        if kt_sq.sqrt() <= mult * k {
                            1.0
                        } else {
                            0.0
                        }
                    }
                };
                ((0.0, scale), w)
            };

            let idx = iy * nx2 + ix;
            padded[idx] = c_scale(c_mul(padded[idx], vel_transfer), reg_weight);
        }
    }

    fft_2d(&mut padded, nx2, ny2, true);
    unpad(&padded, nx2, hologram.nx, hologram.ny)
}

/// Compute sound intensity I = 0.5 * Re(p * conj(v_n)).
pub fn compute_sound_intensity(pressure: &[Complex], velocity: &[Complex]) -> Vec<f64> {
    pressure
        .iter()
        .zip(velocity.iter())
        .map(|(&p, &v)| 0.5 * c_mul(p, c_conj(v)).0)
        .collect()
}

/// Compute total radiated sound power W = sum(I_i * dA).
pub fn compute_sound_power(intensity: &[f64], dx: f64, dy: f64) -> f64 {
    let da = dx * dy;
    intensity.iter().sum::<f64>() * da
}

/// Compute acoustic impedance Z = p / v_n at each point.
pub fn compute_acoustic_impedance(pressure: &[Complex], velocity: &[Complex]) -> Vec<Complex> {
    pressure
        .iter()
        .zip(velocity.iter())
        .map(|(&p, &v)| c_div(p, v))
        .collect()
}

/// Compute radiation efficiency sigma = W / (rho * c * <v_n^2> * A).
pub fn compute_radiation_efficiency(
    power: f64,
    velocity: &[Complex],
    dx: f64,
    dy: f64,
) -> f64 {
    let n = velocity.len() as f64;
    if n < 1.0 {
        return 0.0;
    }
    let mean_v_sq = velocity.iter().map(|v| c_abs_sq(*v)).sum::<f64>() / n;
    let area = n * dx * dy;
    let denom = AIR_DENSITY * SPEED_OF_SOUND * mean_v_sq * area;
    if denom.abs() < 1e-30 {
        0.0
    } else {
        power / denom
    }
}

// ─── HELS (Helmholtz Equation Least Squares) ─────────────────────────────

/// Transfer matrix element for HELS spherical expansion.
///
/// Simplified 2D version using cylindrical wave functions:
/// T_{ij} = H_0^(2)(k * r_{ij}) where r_{ij} is distance from expansion
/// centre j to measurement point i.
///
/// We use a Hankel-function approximation: H_0^(2)(x) ~ sqrt(2/(pi*x)) * exp(-j*(x - pi/4))
fn hankel_h0_2_approx(x: f64) -> Complex {
    if x < 1e-10 {
        return (1e6, 0.0); // Singularity at origin
    }
    let amp = (2.0 / (PI * x)).sqrt();
    let phase = -(x - PI / 4.0);
    c_scale(c_exp(phase), amp)
}

/// Solve HELS reconstruction: p_meas = T * c, c = (T^H*T + lambda*I)^{-1} * T^H * p_meas.
///
/// `measurement_points`: (x, y) coordinates of microphones.
/// `expansion_centres`: (x, y) coordinates of expansion centres.
/// `pressures`: measured complex pressures.
/// `k`: wavenumber.
/// `lambda`: Tikhonov regularization parameter.
///
/// Returns coefficients `c`.
pub fn hels_solve(
    measurement_points: &[(f64, f64)],
    expansion_centres: &[(f64, f64)],
    pressures: &[Complex],
    k: f64,
    lambda: f64,
) -> Vec<Complex> {
    let m = measurement_points.len();
    let n = expansion_centres.len();
    assert_eq!(pressures.len(), m);

    // Build transfer matrix T (m x n)
    let mut t_matrix: Vec<Complex> = vec![(0.0, 0.0); m * n];
    for i in 0..m {
        for j in 0..n {
            let dx = measurement_points[i].0 - expansion_centres[j].0;
            let dy = measurement_points[i].1 - expansion_centres[j].1;
            let r = (dx * dx + dy * dy).sqrt();
            t_matrix[i * n + j] = hankel_h0_2_approx(k * r);
        }
    }

    // Compute T^H * T (n x n) and T^H * p (n x 1)
    let mut th_t: Vec<Complex> = vec![(0.0, 0.0); n * n];
    let mut th_p: Vec<Complex> = vec![(0.0, 0.0); n];

    for i in 0..n {
        for j in 0..n {
            let mut sum = (0.0, 0.0);
            for k_idx in 0..m {
                let t_ki = t_matrix[k_idx * n + i];
                let t_kj = t_matrix[k_idx * n + j];
                sum = c_add(sum, c_mul(c_conj(t_ki), t_kj));
            }
            th_t[i * n + j] = sum;
        }

        let mut sum = (0.0, 0.0);
        for k_idx in 0..m {
            let t_ki = t_matrix[k_idx * n + i];
            sum = c_add(sum, c_mul(c_conj(t_ki), pressures[k_idx]));
        }
        th_p[i] = sum;
    }

    // Add regularization: T^H*T + lambda*I
    for i in 0..n {
        th_t[i * n + i] = c_add(th_t[i * n + i], (lambda, 0.0));
    }

    // Solve (T^H*T + lambda*I) * c = T^H*p using Gaussian elimination
    solve_complex_system(&mut th_t, &mut th_p, n)
}

/// Solve A*x = b via Gaussian elimination with partial pivoting.
fn solve_complex_system(a: &mut [Complex], b: &mut [Complex], n: usize) -> Vec<Complex> {
    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = c_abs(a[col * n + col]);
        let mut max_row = col;
        for row in (col + 1)..n {
            let val = c_abs(a[row * n + col]);
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        // Swap rows
        if max_row != col {
            for k in 0..n {
                let tmp = a[col * n + k];
                a[col * n + k] = a[max_row * n + k];
                a[max_row * n + k] = tmp;
            }
            let tmp = b[col];
            b[col] = b[max_row];
            b[max_row] = tmp;
        }

        let pivot = a[col * n + col];
        if c_abs(pivot) < 1e-20 {
            continue; // Singular
        }

        // Eliminate below
        for row in (col + 1)..n {
            let factor = c_div(a[row * n + col], pivot);
            for k in col..n {
                let sub = c_mul(factor, a[col * n + k]);
                a[row * n + k] = c_sub(a[row * n + k], sub);
            }
            let sub = c_mul(factor, b[col]);
            b[row] = c_sub(b[row], sub);
        }
    }

    // Back substitution
    let mut x = vec![(0.0, 0.0); n];
    for i in (0..n).rev() {
        let mut sum = b[i];
        for j in (i + 1)..n {
            sum = c_sub(sum, c_mul(a[i * n + j], x[j]));
        }
        x[i] = c_div(sum, a[i * n + i]);
    }

    x
}

/// Reconstruct field at scan points using HELS coefficients.
pub fn hels_reconstruct(
    scan_points: &[(f64, f64)],
    expansion_centres: &[(f64, f64)],
    coefficients: &[Complex],
    k: f64,
) -> Vec<Complex> {
    let n_scan = scan_points.len();
    let n_exp = expansion_centres.len();

    let mut result = vec![(0.0, 0.0); n_scan];
    for i in 0..n_scan {
        let mut sum = (0.0, 0.0);
        for j in 0..n_exp {
            let dx = scan_points[i].0 - expansion_centres[j].0;
            let dy = scan_points[i].1 - expansion_centres[j].1;
            let r = (dx * dx + dy * dy).sqrt();
            let h = hankel_h0_2_approx(k * r);
            sum = c_add(sum, c_mul(h, coefficients[j]));
        }
        result[i] = sum;
    }
    result
}

// ─── Beamforming ──────────────────────────────────────────────────────────

/// Delay-and-sum beamforming power map.
///
/// `mic_positions`: (x, y, z) coordinates of microphones.
/// `scan_points`: (x, y, z) coordinates of scan grid.
/// `cross_spectral_matrix`: Hermitian CSM, stored row-major (n_mics x n_mics).
/// `k`: wavenumber.
///
/// Returns power at each scan point.
pub fn beamforming_power_map(
    mic_positions: &[(f64, f64, f64)],
    scan_points: &[(f64, f64, f64)],
    cross_spectral_matrix: &[Complex],
    k: f64,
) -> Vec<f64> {
    let n_mics = mic_positions.len();
    assert_eq!(cross_spectral_matrix.len(), n_mics * n_mics);

    scan_points
        .iter()
        .map(|scan| {
            // Steering vector: a_i = exp(-j*k*r_i) / r_i
            let steering: Vec<Complex> = mic_positions
                .iter()
                .map(|mic| {
                    let dx = scan.0 - mic.0;
                    let dy = scan.1 - mic.1;
                    let dz = scan.2 - mic.2;
                    let r = (dx * dx + dy * dy + dz * dz).sqrt().max(1e-10);
                    c_scale(c_exp(-k * r), 1.0 / r)
                })
                .collect();

            // P = |a^H * C * a|
            // First compute C * a
            let mut ca = vec![(0.0, 0.0); n_mics];
            for i in 0..n_mics {
                for j in 0..n_mics {
                    ca[i] = c_add(ca[i], c_mul(cross_spectral_matrix[i * n_mics + j], steering[j]));
                }
            }

            // Then a^H * (C * a)
            let mut result = (0.0, 0.0);
            for i in 0..n_mics {
                result = c_add(result, c_mul(c_conj(steering[i]), ca[i]));
            }

            c_abs(result)
        })
        .collect()
}

/// Build cross-spectral matrix from microphone data.
///
/// `mic_data`: n_mics vectors of complex frequency-domain data.
/// Returns Hermitian CSM (n_mics x n_mics).
pub fn build_cross_spectral_matrix(mic_data: &[Vec<Complex>]) -> Vec<Complex> {
    let n = mic_data.len();
    let mut csm = vec![(0.0, 0.0); n * n];
    for i in 0..n {
        for j in 0..n {
            let mut sum = (0.0, 0.0);
            let len = mic_data[i].len().min(mic_data[j].len());
            for s in 0..len {
                sum = c_add(sum, c_mul(mic_data[i][s], c_conj(mic_data[j][s])));
            }
            if len > 0 {
                csm[i * n + j] = c_scale(sum, 1.0 / len as f64);
            }
        }
    }
    csm
}

// ─── Source identification ────────────────────────────────────────────────

/// Detected sound source.
#[derive(Debug, Clone)]
pub struct DetectedSource {
    /// Grid index (ix, iy).
    pub index: (usize, usize),
    /// Position (x, y) in metres.
    pub position: (f64, f64),
    /// Intensity value at source location.
    pub intensity: f64,
    /// Source strength (estimated from velocity).
    pub strength: f64,
}

/// Detect sound sources from intensity map via peak finding.
///
/// `intensity`: ny x nx row-major intensity values.
/// `nx`, `ny`: grid dimensions.
/// `dx`, `dy`: grid spacing (m).
/// `threshold`: minimum intensity (fraction of peak) to qualify as source.
pub fn detect_sources(
    intensity: &[f64],
    nx: usize,
    ny: usize,
    dx: f64,
    dy: f64,
    threshold: f64,
) -> Vec<DetectedSource> {
    assert_eq!(intensity.len(), nx * ny);
    if intensity.is_empty() {
        return vec![];
    }

    let peak_intensity = intensity
        .iter()
        .cloned()
        .fold(f64::NEG_INFINITY, f64::max);

    if peak_intensity <= 0.0 {
        return vec![];
    }

    let abs_threshold = threshold * peak_intensity;
    let mut sources = Vec::new();

    for iy in 0..ny {
        for ix in 0..nx {
            let idx = iy * nx + ix;
            let val = intensity[idx];
            if val < abs_threshold {
                continue;
            }

            // Check if local maximum (8-connected neighborhood)
            let mut is_peak = true;
            for dy_off in [-1i32, 0, 1] {
                for dx_off in [-1i32, 0, 1] {
                    if dy_off == 0 && dx_off == 0 {
                        continue;
                    }
                    let nx_i = ix as i32 + dx_off;
                    let ny_i = iy as i32 + dy_off;
                    if nx_i >= 0 && nx_i < nx as i32 && ny_i >= 0 && ny_i < ny as i32 {
                        let neighbor_idx = ny_i as usize * nx + nx_i as usize;
                        if intensity[neighbor_idx] > val {
                            is_peak = false;
                            break;
                        }
                    }
                }
                if !is_peak {
                    break;
                }
            }

            if is_peak {
                sources.push(DetectedSource {
                    index: (ix, iy),
                    position: (ix as f64 * dx, iy as f64 * dy),
                    intensity: val,
                    strength: 0.0, // Will be filled from velocity
                });
            }
        }
    }

    // Sort by intensity, strongest first
    sources.sort_by(|a, b| b.intensity.partial_cmp(&a.intensity).unwrap());
    sources
}

/// Estimate source strengths from normal velocity field.
pub fn estimate_source_strengths(
    sources: &mut [DetectedSource],
    velocity: &[Complex],
    nx: usize,
) {
    for src in sources.iter_mut() {
        let idx = src.index.1 * nx + src.index.0;
        if idx < velocity.len() {
            src.strength = c_abs(velocity[idx]);
        }
    }
}

// ─── Utility ──────────────────────────────────────────────────────────────

/// Check measurement distance constraint. For evanescent wave capture,
/// z_h should be less than lambda/2.
pub fn check_measurement_distance(config: &MicArrayConfig, frequency_hz: f64) -> bool {
    let lambda = SPEED_OF_SOUND / frequency_hz;
    config.measurement_distance_m < lambda / 2.0
}

/// Generate a monopole point source field at the measurement plane.
///
/// Source at (sx, sy, sz), measurement plane at z = 0.
pub fn monopole_field(
    nx: usize,
    ny: usize,
    dx: f64,
    dy: f64,
    source_pos: (f64, f64, f64),
    frequency_hz: f64,
    amplitude: f64,
) -> Vec<Complex> {
    let k = 2.0 * PI * frequency_hz / SPEED_OF_SOUND;
    let mut field = vec![(0.0, 0.0); nx * ny];
    for iy in 0..ny {
        for ix in 0..nx {
            let x = ix as f64 * dx;
            let y = iy as f64 * dy;
            let rx = x - source_pos.0;
            let ry = y - source_pos.1;
            let rz = -source_pos.2; // Measurement at z=0
            let r = (rx * rx + ry * ry + rz * rz).sqrt().max(1e-10);
            // p = A * exp(-j*k*r) / (4*pi*r)
            let mag = amplitude / (4.0 * PI * r);
            field[iy * nx + ix] = c_scale(c_exp(-k * r), mag);
        }
    }
    field
}

/// Generate a dipole point source field.
pub fn dipole_field(
    nx: usize,
    ny: usize,
    dx: f64,
    dy: f64,
    source_pos: (f64, f64, f64),
    frequency_hz: f64,
    amplitude: f64,
) -> Vec<Complex> {
    let k = 2.0 * PI * frequency_hz / SPEED_OF_SOUND;
    let mut field = vec![(0.0, 0.0); nx * ny];
    for iy in 0..ny {
        for ix in 0..nx {
            let x = ix as f64 * dx;
            let y = iy as f64 * dy;
            let rx = x - source_pos.0;
            let ry = y - source_pos.1;
            let rz = -source_pos.2;
            let r = (rx * rx + ry * ry + rz * rz).sqrt().max(1e-10);
            // Dipole: dp/dn = -j*k*cos(theta) * A * exp(-j*k*r) / (4*pi*r)
            let cos_theta = rz / r;
            let mag = amplitude * k * cos_theta / (4.0 * PI * r);
            // Phase includes the -j factor: multiply by (0, -1)
            let base = c_scale(c_exp(-k * r), mag);
            field[iy * nx + ix] = c_mul((0.0, -1.0), base);
        }
    }
    field
}

// ─── Tests ────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    // ── Complex helpers ──

    #[test]
    fn test_complex_add() {
        let a = (1.0, 2.0);
        let b = (3.0, 4.0);
        let c = c_add(a, b);
        assert!((c.0 - 4.0).abs() < TOL);
        assert!((c.1 - 6.0).abs() < TOL);
    }

    #[test]
    fn test_complex_mul() {
        let a = (1.0, 2.0);
        let b = (3.0, 4.0);
        let c = c_mul(a, b);
        // (1+2j)(3+4j) = 3+4j+6j+8j^2 = -5+10j
        assert!((c.0 - (-5.0)).abs() < TOL);
        assert!((c.1 - 10.0).abs() < TOL);
    }

    #[test]
    fn test_complex_conj() {
        let a = (3.0, -4.0);
        let c = c_conj(a);
        assert!((c.0 - 3.0).abs() < TOL);
        assert!((c.1 - 4.0).abs() < TOL);
    }

    #[test]
    fn test_complex_abs() {
        let a = (3.0, 4.0);
        assert!((c_abs(a) - 5.0).abs() < TOL);
    }

    #[test]
    fn test_complex_div() {
        let a = (1.0, 0.0);
        let b = (0.0, 1.0);
        let c = c_div(a, b);
        // 1/(j) = -j
        assert!(c.0.abs() < TOL);
        assert!((c.1 - (-1.0)).abs() < TOL);
    }

    #[test]
    fn test_complex_exp_zero() {
        let c = c_exp(0.0);
        assert!((c.0 - 1.0).abs() < TOL);
        assert!(c.1.abs() < TOL);
    }

    #[test]
    fn test_complex_exp_pi() {
        let c = c_exp(PI);
        assert!((c.0 - (-1.0)).abs() < TOL);
        assert!(c.1.abs() < TOL);
    }

    // ── FFT ──

    #[test]
    fn test_fft_1d_impulse() {
        let mut data = vec![(0.0, 0.0); 4];
        data[0] = (1.0, 0.0);
        fft_1d(&mut data, false);
        // FFT of delta -> all ones
        for x in &data {
            assert!((x.0 - 1.0).abs() < TOL);
            assert!(x.1.abs() < TOL);
        }
    }

    #[test]
    fn test_fft_1d_roundtrip() {
        let original = vec![(1.0, 0.0), (2.0, 1.0), (3.0, -1.0), (0.5, 0.5)];
        let mut data = original.clone();
        fft_1d(&mut data, false);
        fft_1d(&mut data, true);
        for (a, b) in data.iter().zip(original.iter()) {
            assert!((a.0 - b.0).abs() < TOL);
            assert!((a.1 - b.1).abs() < TOL);
        }
    }

    #[test]
    fn test_fft_1d_dc() {
        // All ones -> DC only
        let mut data = vec![(1.0, 0.0); 8];
        fft_1d(&mut data, false);
        assert!((data[0].0 - 8.0).abs() < TOL);
        for i in 1..8 {
            assert!(c_abs(data[i]) < TOL);
        }
    }

    #[test]
    fn test_fft_2d_roundtrip() {
        let nx = 4;
        let ny = 4;
        let mut data: Vec<Complex> = (0..nx * ny)
            .map(|i| (i as f64, (i as f64 * 0.1)))
            .collect();
        let original = data.clone();
        fft_2d(&mut data, nx, ny, false);
        fft_2d(&mut data, nx, ny, true);
        for (a, b) in data.iter().zip(original.iter()) {
            assert!((a.0 - b.0).abs() < 1e-10);
            assert!((a.1 - b.1).abs() < 1e-10);
        }
    }

    #[test]
    fn test_fft_2d_impulse() {
        let nx = 4;
        let ny = 4;
        let mut data = vec![(0.0, 0.0); nx * ny];
        data[0] = (1.0, 0.0);
        fft_2d(&mut data, nx, ny, false);
        // All bins should be 1.0
        for x in &data {
            assert!((x.0 - 1.0).abs() < TOL);
            assert!(x.1.abs() < TOL);
        }
    }

    // ── Wavenumber grid ──

    #[test]
    fn test_wavenumber_axis() {
        let k = wavenumber_axis(8, 0.01);
        assert_eq!(k.len(), 8);
        assert!((k[0]).abs() < TOL); // DC = 0
        // k[4] should be Nyquist
        let k_nyq = PI / 0.01;
        assert!((k[4] - k_nyq).abs() < 1.0);
    }

    #[test]
    fn test_wavenumber_axis_symmetry() {
        let k = wavenumber_axis(8, 0.02);
        // k[1] = -k[7], k[2] = -k[6], k[3] = -k[5]
        assert!((k[1] + k[7]).abs() < TOL);
        assert!((k[2] + k[6]).abs() < TOL);
        assert!((k[3] + k[5]).abs() < TOL);
    }

    // ── MicArrayConfig ──

    #[test]
    fn test_mic_array_k_nyquist() {
        let config = MicArrayConfig {
            num_mics_x: 16,
            num_mics_y: 16,
            spacing_m: 0.01,
            measurement_distance_m: 0.02,
        };
        let k_nyq = config.k_nyquist();
        assert!((k_nyq - PI / 0.01).abs() < TOL);
    }

    #[test]
    fn test_mic_array_max_frequency() {
        let config = MicArrayConfig {
            num_mics_x: 16,
            num_mics_y: 16,
            spacing_m: 0.01,
            measurement_distance_m: 0.02,
        };
        // f_max = c * k_nyq / (2*pi) = 343 * pi/0.01 / (2*pi) = 343/(2*0.01) = 17150
        let f_max = config.max_frequency_hz();
        assert!((f_max - 17150.0).abs() < 1.0);
    }

    #[test]
    fn test_mic_array_aperture() {
        let config = MicArrayConfig {
            num_mics_x: 8,
            num_mics_y: 4,
            spacing_m: 0.02,
            measurement_distance_m: 0.05,
        };
        let (ax, ay) = config.aperture();
        assert!((ax - 0.14).abs() < TOL);
        assert!((ay - 0.06).abs() < TOL);
    }

    // ── HologramData ──

    #[test]
    fn test_hologram_wavenumber() {
        let h = HologramData {
            pressure: vec![(1.0, 0.0)],
            nx: 1,
            ny: 1,
            frequency_hz: 1000.0,
            dx: 0.01,
            dy: 0.01,
        };
        let k = h.wavenumber();
        let expected = 2.0 * PI * 1000.0 / SPEED_OF_SOUND;
        assert!((k - expected).abs() < TOL);
    }

    #[test]
    fn test_hologram_magnitude() {
        let h = HologramData {
            pressure: vec![(3.0, 4.0), (0.0, 1.0)],
            nx: 2,
            ny: 1,
            frequency_hz: 1000.0,
            dx: 0.01,
            dy: 0.01,
        };
        let mag = h.magnitude();
        assert!((mag[0] - 5.0).abs() < TOL);
        assert!((mag[1] - 1.0).abs() < TOL);
    }

    #[test]
    fn test_hologram_phase() {
        let h = HologramData {
            pressure: vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)],
            nx: 3,
            ny: 1,
            frequency_hz: 1000.0,
            dx: 0.01,
            dy: 0.01,
        };
        let ph = h.phase();
        assert!(ph[0].abs() < TOL); // 0 rad
        assert!((ph[1] - PI / 2.0).abs() < TOL); // pi/2
        assert!((ph[2] - PI).abs() < TOL); // pi
    }

    #[test]
    fn test_hologram_spl_db() {
        let p_ref = 20e-6;
        let h = HologramData {
            pressure: vec![(p_ref, 0.0)],
            nx: 1,
            ny: 1,
            frequency_hz: 1000.0,
            dx: 0.01,
            dy: 0.01,
        };
        let spl = h.spl_db();
        assert!(spl[0].abs() < 0.01); // 0 dB
    }

    // ── Propagation ──

    #[test]
    fn test_propagation_zero_distance() {
        let nx = 4;
        let ny = 4;
        let pressure: Vec<Complex> = (0..nx * ny).map(|i| (i as f64, 0.0)).collect();
        let h = HologramData {
            pressure: pressure.clone(),
            nx,
            ny,
            frequency_hz: 1000.0,
            dx: 0.01,
            dy: 0.01,
        };
        let result = propagate_planar(&h, 0.0, RegularizationMethod::None);
        for (a, b) in result.pressure.iter().zip(pressure.iter()) {
            assert!((a.0 - b.0).abs() < 1e-8);
            assert!((a.1 - b.1).abs() < 1e-8);
        }
    }

    #[test]
    fn test_propagation_preserves_size() {
        let nx = 8;
        let ny = 8;
        let pressure = vec![(1.0, 0.0); nx * ny];
        let h = HologramData {
            pressure,
            nx,
            ny,
            frequency_hz: 2000.0,
            dx: 0.01,
            dy: 0.01,
        };
        let fwd = propagate_planar(&h, 0.05, RegularizationMethod::None);
        assert_eq!(fwd.pressure.len(), nx * ny);
        assert_eq!(fwd.nx, nx);
        assert_eq!(fwd.ny, ny);
    }

    #[test]
    fn test_forward_backward_roundtrip() {
        let nx = 8;
        let ny = 8;
        let mut pressure = vec![(0.0, 0.0); nx * ny];
        // Simple pattern
        pressure[ny / 2 * nx + nx / 2] = (1.0, 0.0);
        let h = HologramData {
            pressure: pressure.clone(),
            nx,
            ny,
            frequency_hz: 1000.0,
            dx: 0.02,
            dy: 0.02,
        };
        let fwd = propagate_planar(&h, 0.05, RegularizationMethod::None);
        let back = propagate_planar(&fwd, -0.05, RegularizationMethod::Tikhonov(1e-6));
        // Roundtrip should approximately recover original (with regularization losses)
        let orig_energy: f64 = pressure.iter().map(|p| c_abs_sq(*p)).sum();
        let rec_energy: f64 = back.pressure.iter().map(|p| c_abs_sq(*p)).sum();
        // Energy should be non-zero and somewhat preserved
        assert!(rec_energy > 0.01 * orig_energy);
    }

    #[test]
    fn test_tikhonov_regularization() {
        let nx = 4;
        let ny = 4;
        let mut pressure = vec![(0.0, 0.0); nx * ny];
        pressure[0] = (1.0, 0.0);
        let h = HologramData {
            pressure,
            nx,
            ny,
            frequency_hz: 5000.0,
            dx: 0.01,
            dy: 0.01,
        };
        // Backward propagation with strong regularization should limit amplification
        let strong = propagate_planar(&h, -0.02, RegularizationMethod::Tikhonov(1.0));
        let weak = propagate_planar(&h, -0.02, RegularizationMethod::Tikhonov(1e-6));
        let strong_energy: f64 = strong.pressure.iter().map(|p| c_abs_sq(*p)).sum();
        let weak_energy: f64 = weak.pressure.iter().map(|p| c_abs_sq(*p)).sum();
        // Stronger regularization -> less energy amplification
        assert!(strong_energy <= weak_energy + 1e-6);
    }

    #[test]
    fn test_exponential_regularization() {
        let nx = 4;
        let ny = 4;
        let pressure = vec![(1.0, 0.0); nx * ny];
        let h = HologramData {
            pressure,
            nx,
            ny,
            frequency_hz: 2000.0,
            dx: 0.01,
            dy: 0.01,
        };
        let result = propagate_planar(&h, -0.01, RegularizationMethod::Exponential(0.01));
        assert_eq!(result.pressure.len(), nx * ny);
        // Should produce finite values
        for p in &result.pressure {
            assert!(p.0.is_finite());
            assert!(p.1.is_finite());
        }
    }

    #[test]
    fn test_kspace_cutoff_regularization() {
        let nx = 8;
        let ny = 8;
        let mut pressure = vec![(0.0, 0.0); nx * ny];
        pressure[0] = (1.0, 0.0);
        let h = HologramData {
            pressure,
            nx,
            ny,
            frequency_hz: 1000.0,
            dx: 0.01,
            dy: 0.01,
        };
        let result = propagate_planar(&h, -0.03, RegularizationMethod::KSpaceCutoff(1.5));
        for p in &result.pressure {
            assert!(p.0.is_finite());
            assert!(p.1.is_finite());
        }
    }

    // ── Sound field quantities ──

    #[test]
    fn test_normal_velocity_computation() {
        let nx = 4;
        let ny = 4;
        let pressure = vec![(1.0, 0.0); nx * ny];
        let h = HologramData {
            pressure,
            nx,
            ny,
            frequency_hz: 1000.0,
            dx: 0.02,
            dy: 0.02,
        };
        let vel = compute_normal_velocity(&h, RegularizationMethod::Tikhonov(1e-4));
        assert_eq!(vel.len(), nx * ny);
        for v in &vel {
            assert!(v.0.is_finite());
            assert!(v.1.is_finite());
        }
    }

    #[test]
    fn test_sound_intensity() {
        let p = vec![(1.0, 0.0), (0.0, 1.0)];
        let v = vec![(0.5, 0.0), (0.0, 0.5)];
        let intensity = compute_sound_intensity(&p, &v);
        assert_eq!(intensity.len(), 2);
        // I = 0.5 * Re(p * conj(v))
        // First: 0.5 * Re((1,0)*(0.5,0)) = 0.5 * 0.5 = 0.25
        assert!((intensity[0] - 0.25).abs() < TOL);
        // Second: 0.5 * Re((0,1)*(0,-0.5)) = 0.5 * Re(0*0 + 1*0.5, ...) = 0.5 * 0.5 = 0.25
        assert!((intensity[1] - 0.25).abs() < TOL);
    }

    #[test]
    fn test_sound_power() {
        let intensity = vec![1.0, 2.0, 3.0, 4.0];
        let power = compute_sound_power(&intensity, 0.01, 0.01);
        // W = sum * dA = 10.0 * 0.0001 = 0.001
        assert!((power - 0.001).abs() < TOL);
    }

    #[test]
    fn test_acoustic_impedance() {
        let p = vec![(1.0, 0.0)];
        let v = vec![(0.0, 1.0)];
        let z = compute_acoustic_impedance(&p, &v);
        // Z = (1,0)/(0,1) = (0,-1) = -j
        assert!(z[0].0.abs() < TOL);
        assert!((z[0].1 - (-1.0)).abs() < TOL);
    }

    #[test]
    fn test_radiation_efficiency() {
        let velocity = vec![(1.0, 0.0), (1.0, 0.0), (1.0, 0.0), (1.0, 0.0)];
        let power = 1.0;
        let sigma = compute_radiation_efficiency(power, &velocity, 0.01, 0.01);
        // sigma = W / (rho*c*<v^2>*A)
        // <v^2> = 1.0, A = 4*0.0001 = 0.0004
        // sigma = 1.0 / (1.225 * 343 * 1.0 * 0.0004)
        let expected = 1.0 / (AIR_DENSITY * SPEED_OF_SOUND * 1.0 * 0.0004);
        assert!((sigma - expected).abs() / expected < 0.01);
    }

    // ── HELS ──

    #[test]
    fn test_hels_solve_simple() {
        let mic_pts = vec![(0.0, 0.0), (0.1, 0.0), (0.0, 0.1), (0.1, 0.1)];
        let exp_centres = vec![(0.05, 0.05)];
        let k = 2.0 * PI * 500.0 / SPEED_OF_SOUND;

        // Generate "measured" pressures from known coefficient
        let true_coeff = vec![(1.0, 0.0)];
        let pressures = hels_reconstruct(&mic_pts, &exp_centres, &true_coeff, k);

        let recovered = hels_solve(&mic_pts, &exp_centres, &pressures, k, 1e-6);
        assert_eq!(recovered.len(), 1);
        // Should approximately recover the coefficient
        assert!((c_abs(c_sub(recovered[0], true_coeff[0])) / c_abs(true_coeff[0])) < 0.1);
    }

    #[test]
    fn test_hels_reconstruct() {
        let scan = vec![(0.0, 0.0), (0.1, 0.1)];
        let exp = vec![(0.05, 0.05)];
        let coeffs = vec![(1.0, 0.0)];
        let k = 10.0;
        let result = hels_reconstruct(&scan, &exp, &coeffs, k);
        assert_eq!(result.len(), 2);
        for p in &result {
            assert!(p.0.is_finite());
            assert!(p.1.is_finite());
        }
    }

    // ── Beamforming ──

    #[test]
    fn test_beamforming_single_source() {
        // 4 mics in a line, source directly in front of centre
        let mics: Vec<(f64, f64, f64)> = vec![
            (-0.03, 0.0, 0.0),
            (-0.01, 0.0, 0.0),
            (0.01, 0.0, 0.0),
            (0.03, 0.0, 0.0),
        ];
        let k = 2.0 * PI * 1000.0 / SPEED_OF_SOUND;

        // Generate data from a source at (0.0, 0.0, 0.1)
        let source: (f64, f64, f64) = (0.0, 0.0, 0.1);
        let mic_data: Vec<Vec<Complex>> = mics
            .iter()
            .map(|m| {
                let dx = source.0 - m.0;
                let dy = source.1 - m.1;
                let dz = source.2 - m.2;
                let r = (dx * dx + dy * dy + dz * dz).sqrt();
                vec![c_scale(c_exp(-k * r), 1.0 / r)]
            })
            .collect();

        let csm = build_cross_spectral_matrix(&mic_data);

        // Scan two points: one at the source, one offset
        let scan = vec![(0.0, 0.0, 0.1), (0.05, 0.0, 0.1)];
        let power = beamforming_power_map(&mics, &scan, &csm, k);

        assert_eq!(power.len(), 2);
        // Source location should have higher power
        assert!(power[0] > power[1]);
    }

    #[test]
    fn test_csm_hermitian() {
        let data = vec![
            vec![(1.0, 0.5), (0.3, -0.2)],
            vec![(0.5, -0.3), (0.7, 0.1)],
        ];
        let csm = build_cross_spectral_matrix(&data);
        // CSM should be Hermitian: csm[i,j] = conj(csm[j,i])
        let n = 2;
        for i in 0..n {
            for j in 0..n {
                let ij = csm[i * n + j];
                let ji = csm[j * n + i];
                assert!((ij.0 - ji.0).abs() < TOL);
                assert!((ij.1 + ji.1).abs() < TOL);
            }
        }
    }

    // ── Source detection ──

    #[test]
    fn test_detect_sources_single_peak() {
        let nx = 4;
        let ny = 4;
        let mut intensity = vec![0.0; nx * ny];
        intensity[1 * nx + 2] = 10.0; // Peak at (2, 1)
        let sources = detect_sources(&intensity, nx, ny, 0.01, 0.01, 0.5);
        assert_eq!(sources.len(), 1);
        assert_eq!(sources[0].index, (2, 1));
        assert!((sources[0].intensity - 10.0).abs() < TOL);
    }

    #[test]
    fn test_detect_sources_multiple_peaks() {
        let nx = 8;
        let ny = 8;
        let mut intensity = vec![0.0; nx * ny];
        intensity[1 * nx + 1] = 10.0;
        intensity[6 * nx + 6] = 8.0;
        let sources = detect_sources(&intensity, nx, ny, 0.01, 0.01, 0.5);
        assert_eq!(sources.len(), 2);
        // Sorted by intensity: strongest first
        assert_eq!(sources[0].index, (1, 1));
        assert_eq!(sources[1].index, (6, 6));
    }

    #[test]
    fn test_detect_sources_threshold() {
        let nx = 4;
        let ny = 4;
        let mut intensity = vec![0.0; nx * ny];
        intensity[0] = 10.0;
        intensity[nx * ny - 1] = 2.0; // Below 50% threshold
        let sources = detect_sources(&intensity, nx, ny, 0.01, 0.01, 0.5);
        assert_eq!(sources.len(), 1);
    }

    #[test]
    fn test_estimate_source_strengths() {
        let mut sources = vec![DetectedSource {
            index: (2, 3),
            position: (0.02, 0.03),
            intensity: 5.0,
            strength: 0.0,
        }];
        let nx = 4;
        let velocity = vec![(0.0, 0.0); nx * 4];
        // Velocity at index 3*4+2 = 14
        let mut vel = velocity;
        vel[14] = (3.0, 4.0); // |v| = 5
        estimate_source_strengths(&mut sources, &vel, nx);
        assert!((sources[0].strength - 5.0).abs() < TOL);
    }

    // ── Utility ──

    #[test]
    fn test_measurement_distance_check() {
        let config = MicArrayConfig {
            num_mics_x: 16,
            num_mics_y: 16,
            spacing_m: 0.01,
            measurement_distance_m: 0.05,
        };
        // At 1000 Hz, lambda = 0.343m, lambda/2 = 0.1715m. 0.05 < 0.1715 -> true
        assert!(check_measurement_distance(&config, 1000.0));
        // At 10000 Hz, lambda = 0.0343m, lambda/2 = 0.01715m. 0.05 > 0.01715 -> false
        assert!(!check_measurement_distance(&config, 10000.0));
    }

    #[test]
    fn test_monopole_field() {
        let nx = 8;
        let ny = 8;
        let field = monopole_field(nx, ny, 0.01, 0.01, (0.035, 0.035, 0.05), 1000.0, 1.0);
        assert_eq!(field.len(), nx * ny);
        // Field should be strongest near source projection
        let center_idx = 3 * nx + 3;
        let corner_idx = 0;
        assert!(c_abs(field[center_idx]) > c_abs(field[corner_idx]));
    }

    #[test]
    fn test_dipole_field() {
        let nx = 8;
        let ny = 8;
        let field = dipole_field(nx, ny, 0.01, 0.01, (0.035, 0.035, 0.05), 1000.0, 1.0);
        assert_eq!(field.len(), nx * ny);
        for p in &field {
            assert!(p.0.is_finite());
            assert!(p.1.is_finite());
        }
    }

    #[test]
    fn test_monopole_inverse_square_falloff() {
        // Pressure from monopole should decrease with distance
        let field_close = monopole_field(1, 1, 0.01, 0.01, (0.0, 0.0, 0.01), 1000.0, 1.0);
        let field_far = monopole_field(1, 1, 0.01, 0.01, (0.0, 0.0, 0.1), 1000.0, 1.0);
        assert!(c_abs(field_close[0]) > c_abs(field_far[0]));
    }

    // ── Padding ──

    #[test]
    fn test_pad_and_unpad_roundtrip() {
        let nx = 3;
        let ny = 5;
        let data: Vec<Complex> = (0..nx * ny).map(|i| (i as f64, -(i as f64) * 0.5)).collect();
        let (padded, nx2, ny2) = pad_to_pow2(&data, nx, ny);
        assert!(nx2.is_power_of_two() && nx2 >= nx);
        assert!(ny2.is_power_of_two() && ny2 >= ny);
        let recovered = unpad(&padded, nx2, nx, ny);
        for (a, b) in recovered.iter().zip(data.iter()) {
            assert!((a.0 - b.0).abs() < TOL);
            assert!((a.1 - b.1).abs() < TOL);
        }
    }

    // ── Gaussian elimination ──

    #[test]
    fn test_solve_complex_system_identity() {
        // I * x = b => x = b
        let mut a = vec![(1.0, 0.0), (0.0, 0.0), (0.0, 0.0), (1.0, 0.0)];
        let mut b = vec![(3.0, 1.0), (5.0, -2.0)];
        let x = solve_complex_system(&mut a, &mut b, 2);
        assert!((x[0].0 - 3.0).abs() < TOL);
        assert!((x[0].1 - 1.0).abs() < TOL);
        assert!((x[1].0 - 5.0).abs() < TOL);
        assert!((x[1].1 - (-2.0)).abs() < TOL);
    }
}
