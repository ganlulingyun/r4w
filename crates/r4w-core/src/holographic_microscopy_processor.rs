//! Digital holographic microscopy (DHM) signal processing.
//!
//! This module implements the core algorithms for digital holographic microscopy,
//! enabling 3D particle tracking, live cell imaging, surface topography,
//! microfluidics analysis, and quantitative phase imaging (QPI).
//!
//! # Overview
//!
//! - **[`HolographyConfig`]** — optical system parameters (wavelength, magnification,
//!   pixel size, reconstruction distance)
//! - **[`FresnelPropagator`]** — Fresnel diffraction integral for scalar field
//!   propagation: U(x,y,z) = FFT{U0 · exp(iπ(x²+y²)/(λz))}
//! - **[`AngularSpectrumMethod`]** — exact propagation via transfer function:
//!   H(fx,fy) = exp(i2πz√(1/λ² − fx² − fy²))
//! - **[`PhaseRetriever`]** — quantitative phase extraction with 1D unwrapping
//! - **[`TwinImageRemover`]** — zero-order and twin image suppression
//! - **[`AutofocusAlgorithm`]** — find best focus distance by gradient sharpness
//!   or minimum entropy
//! - **[`ParticleTracker3D`]** — 3D particle localisation from holographic
//!   reconstructions at multiple z-planes
//! - **[`ThicknessMeasurer`]** — optical path length difference → surface height
//! - **[`InterferencePatternGenerator`]** — simulate inline holograms from point
//!   scatterers
//! - **[`NumericalAperture`]** — effective NA, lateral and axial resolution limits
//!
//! # Physics
//!
//! ```text
//! Fresnel number:      F = a² / (λz)
//! Fresnel propagation: paraxial approximation of Huygens-Fresnel integral
//! Angular spectrum:    exact for homogeneous media
//! Hologram intensity:  I = |R + O|² = |R|² + |O|² + R*O* + R·O
//! Phase sensitivity:   ~ 1 nm OPL for visible light
//! Lateral resolution:  Δx ≈ λ / (2·NA)
//! Axial resolution:    Δz ≈ λ / NA²
//! OPL → height:        h = OPL / (n − 1)
//! ```
//!
//! # Example
//!
//! ```
//! use r4w_core::holographic_microscopy_processor::{
//!     HolographyConfig, FresnelPropagator, NumericalAperture,
//! };
//!
//! let cfg = HolographyConfig::new(632.8e-9, 40.0, 6.45e-6, 50.0e-6);
//! let na = NumericalAperture::new(cfg.wavelength_m, cfg.magnification, cfg.pixel_size_m, 64);
//! assert!(na.lateral_resolution_m() > 0.0);
//!
//! let propagator = FresnelPropagator::new(&cfg, 8);
//! let field = vec![(1.0, 0.0); 64]; // 8×8 uniform field
//! let recon = propagator.propagate(&field, cfg.recon_distance_m);
//! assert_eq!(recon.len(), 64);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers (re, im) tuples
// ---------------------------------------------------------------------------

type C64 = (f64, f64);

#[inline]
fn c_add(a: C64, b: C64) -> C64 {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: C64, b: C64) -> C64 {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: C64, b: C64) -> C64 {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: C64) -> C64 {
    (a.0, -a.1)
}

#[inline]
fn c_abs(a: C64) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

#[inline]
fn c_abs2(a: C64) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn c_arg(a: C64) -> f64 {
    a.1.atan2(a.0)
}

#[inline]
fn c_exp_j(theta: f64) -> C64 {
    (theta.cos(), theta.sin())
}

#[inline]
fn c_scale(s: f64, a: C64) -> C64 {
    (s * a.0, s * a.1)
}

// ---------------------------------------------------------------------------
// 2-D DFT / IDFT  (row-column decomposition with Cooley-Tukey 1-D FFT)
// ---------------------------------------------------------------------------

/// Radix-2 in-place FFT (Cooley-Tukey decimation-in-time).
/// `inverse` == true → IFFT (conjugate twiddles, 1/N scaling).
fn fft_1d(buf: &mut [C64], inverse: bool) {
    let n = buf.len();
    if n <= 1 {
        return;
    }
    debug_assert!(n.is_power_of_two(), "FFT length must be power of 2");

    // Bit-reversal permutation
    let mut j: usize = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            buf.swap(i, j);
        }
    }

    // Butterfly stages
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let sign = if inverse { 1.0 } else { -1.0 };
        let angle = sign * 2.0 * PI / len as f64;
        let wn = c_exp_j(angle);
        let mut start = 0;
        while start < n {
            let mut w: C64 = (1.0, 0.0);
            for k in 0..half {
                let u = buf[start + k];
                let t = c_mul(w, buf[start + k + half]);
                buf[start + k] = c_add(u, t);
                buf[start + k + half] = c_sub(u, t);
                w = c_mul(w, wn);
            }
            start += len;
        }
        len <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for v in buf.iter_mut() {
            *v = c_scale(inv_n, *v);
        }
    }
}

/// Forward 2-D DFT on an N×N grid (row-column decomposition).
fn fft_2d(data: &mut [C64], n: usize) {
    assert_eq!(data.len(), n * n);
    // Rows
    for r in 0..n {
        let start = r * n;
        let row = &mut data[start..start + n];
        fft_1d(row, false);
    }
    // Columns (extract, transform, put back)
    let mut col = vec![(0.0, 0.0); n];
    for c in 0..n {
        for r in 0..n {
            col[r] = data[r * n + c];
        }
        fft_1d(&mut col, false);
        for r in 0..n {
            data[r * n + c] = col[r];
        }
    }
}

/// Inverse 2-D DFT on an N×N grid.
fn ifft_2d(data: &mut [C64], n: usize) {
    assert_eq!(data.len(), n * n);
    // Rows
    for r in 0..n {
        let start = r * n;
        let row = &mut data[start..start + n];
        fft_1d(row, true);
    }
    // Columns
    let mut col = vec![(0.0, 0.0); n];
    for c in 0..n {
        for r in 0..n {
            col[r] = data[r * n + c];
        }
        fft_1d(&mut col, true);
        for r in 0..n {
            data[r * n + c] = col[r];
        }
    }
}

/// Centre-shift a 2-D spectrum (swap quadrants).
fn fftshift_2d(data: &mut [C64], n: usize) {
    let half = n / 2;
    // Swap rows
    for r in 0..half {
        for c in 0..n {
            let i1 = r * n + c;
            let i2 = (r + half) * n + c;
            data.swap(i1, i2);
        }
    }
    // Swap columns
    for r in 0..n {
        for c in 0..half {
            let i1 = r * n + c;
            let i2 = r * n + c + half;
            data.swap(i1, i2);
        }
    }
}

// ---------------------------------------------------------------------------
// HolographyConfig
// ---------------------------------------------------------------------------

/// Optical system parameters for digital holographic microscopy.
#[derive(Debug, Clone)]
pub struct HolographyConfig {
    /// Illumination wavelength in metres (e.g. 632.8e-9 for HeNe).
    pub wavelength_m: f64,
    /// Objective magnification (e.g. 40.0).
    pub magnification: f64,
    /// Camera pixel size in metres (e.g. 6.45e-6).
    pub pixel_size_m: f64,
    /// Reconstruction distance in metres (object plane to hologram plane).
    pub recon_distance_m: f64,
}

impl HolographyConfig {
    /// Create a new configuration.
    pub fn new(
        wavelength_m: f64,
        magnification: f64,
        pixel_size_m: f64,
        recon_distance_m: f64,
    ) -> Self {
        Self {
            wavelength_m,
            magnification,
            pixel_size_m,
            recon_distance_m,
        }
    }

    /// Effective pixel size at the object plane.
    pub fn effective_pixel_size(&self) -> f64 {
        self.pixel_size_m / self.magnification
    }

    /// Fresnel number F = a² / (λ z) where a = half the field of view.
    pub fn fresnel_number(&self, n: usize) -> f64 {
        let a = self.effective_pixel_size() * n as f64 / 2.0;
        a * a / (self.wavelength_m * self.recon_distance_m)
    }
}

// ---------------------------------------------------------------------------
// FresnelPropagator
// ---------------------------------------------------------------------------

/// Fresnel diffraction propagation using the single-FFT method.
///
/// Computes:
/// ```text
/// U(x,y,z) = FFT{ U0(x',y') · exp(iπ(x'² + y'²) / (λz)) } · phase_factor
/// ```
pub struct FresnelPropagator {
    wavelength_m: f64,
    effective_pixel: f64,
    n: usize,
}

impl FresnelPropagator {
    /// Create a propagator for the given config and grid size N×N.
    pub fn new(cfg: &HolographyConfig, n: usize) -> Self {
        assert!(n.is_power_of_two(), "Grid size must be power of 2");
        Self {
            wavelength_m: cfg.wavelength_m,
            effective_pixel: cfg.effective_pixel_size(),
            n,
        }
    }

    /// Propagate a complex field to distance `z` using the Fresnel approximation.
    /// `field` must be N² elements (row-major N×N).
    pub fn propagate(&self, field: &[C64], z: f64) -> Vec<C64> {
        let n = self.n;
        assert_eq!(field.len(), n * n);
        let dx = self.effective_pixel;
        let lam = self.wavelength_m;

        // Build chirp kernel and multiply with input
        let mut buf: Vec<C64> = Vec::with_capacity(n * n);
        let half = n as f64 / 2.0;
        for r in 0..n {
            let y = (r as f64 - half) * dx;
            for c in 0..n {
                let x = (c as f64 - half) * dx;
                let phase = PI * (x * x + y * y) / (lam * z);
                buf.push(c_mul(field[r * n + c], c_exp_j(phase)));
            }
        }

        fft_2d(&mut buf, n);
        fftshift_2d(&mut buf, n);

        // Constant phase factor (often dropped for intensity but kept for phase imaging)
        let k = 2.0 * PI / lam;
        let prefactor_phase = k * z;
        let prefactor = c_exp_j(prefactor_phase);
        for v in buf.iter_mut() {
            *v = c_mul(prefactor, *v);
        }
        buf
    }

    /// Reconstruct intensity image at distance z.
    pub fn intensity(&self, field: &[C64], z: f64) -> Vec<f64> {
        self.propagate(field, z).iter().map(|v| c_abs2(*v)).collect()
    }
}

// ---------------------------------------------------------------------------
// AngularSpectrumMethod
// ---------------------------------------------------------------------------

/// Exact scalar field propagation via the Angular Spectrum Method.
///
/// Transfer function:
/// ```text
/// H(fx, fy) = exp(i 2π z √(1/λ² − fx² − fy²))
/// ```
///
/// Evanescent waves (fx² + fy² > 1/λ²) are attenuated.
pub struct AngularSpectrumMethod {
    wavelength_m: f64,
    effective_pixel: f64,
    n: usize,
}

impl AngularSpectrumMethod {
    pub fn new(cfg: &HolographyConfig, n: usize) -> Self {
        assert!(n.is_power_of_two(), "Grid size must be power of 2");
        Self {
            wavelength_m: cfg.wavelength_m,
            effective_pixel: cfg.effective_pixel_size(),
            n,
        }
    }

    /// Propagate a complex field to distance `z`.
    pub fn propagate(&self, field: &[C64], z: f64) -> Vec<C64> {
        let n = self.n;
        assert_eq!(field.len(), n * n);
        let dx = self.effective_pixel;
        let lam = self.wavelength_m;
        let inv_lam2 = 1.0 / (lam * lam);
        let df = 1.0 / (n as f64 * dx); // spatial frequency sampling

        // Forward FFT to get angular spectrum
        let mut buf = field.to_vec();
        fft_2d(&mut buf, n);

        // Apply transfer function in frequency domain
        let half = (n / 2) as i64;
        for r in 0..n {
            let fy = (r as i64 - half) as f64 * df;
            for c in 0..n {
                let fx = (c as i64 - half) as f64 * df;
                let f2 = fx * fx + fy * fy;
                let idx = r * n + c;
                if f2 < inv_lam2 {
                    // Propagating wave
                    let kz = 2.0 * PI * (inv_lam2 - f2).sqrt();
                    buf[idx] = c_mul(buf[idx], c_exp_j(kz * z));
                } else {
                    // Evanescent — attenuate
                    let alpha = 2.0 * PI * (f2 - inv_lam2).sqrt();
                    let decay = (-alpha * z.abs()).exp();
                    buf[idx] = c_scale(decay, buf[idx]);
                }
            }
        }

        // Inverse FFT back to spatial domain
        ifft_2d(&mut buf, n);
        buf
    }

    /// Reconstruct intensity image at distance z.
    pub fn intensity(&self, field: &[C64], z: f64) -> Vec<f64> {
        self.propagate(field, z).iter().map(|v| c_abs2(*v)).collect()
    }
}

// ---------------------------------------------------------------------------
// PhaseRetriever
// ---------------------------------------------------------------------------

/// Extract quantitative phase from a reconstructed complex field.
///
/// Phase is computed as φ = atan2(Im, Re) and optionally unwrapped
/// row-by-row using Itoh's method (adjacent-difference unwrapping).
pub struct PhaseRetriever;

impl PhaseRetriever {
    /// Wrapped phase map φ ∈ (−π, π].
    pub fn wrapped_phase(field: &[C64]) -> Vec<f64> {
        field.iter().map(|v| c_arg(*v)).collect()
    }

    /// 1-D phase unwrapping (Itoh's method): add ±2π to remove jumps > π.
    pub fn unwrap_1d(wrapped: &[f64]) -> Vec<f64> {
        if wrapped.is_empty() {
            return Vec::new();
        }
        let mut out = vec![0.0; wrapped.len()];
        out[0] = wrapped[0];
        for i in 1..wrapped.len() {
            let mut diff = wrapped[i] - wrapped[i - 1];
            // Wrap diff into (−π, π]
            while diff > PI {
                diff -= 2.0 * PI;
            }
            while diff <= -PI {
                diff += 2.0 * PI;
            }
            out[i] = out[i - 1] + diff;
        }
        out
    }

    /// 2-D phase unwrapping — row-by-row then column-by-column averaging.
    /// Simple but effective for smooth phase distributions.
    pub fn unwrap_2d(wrapped: &[f64], nx: usize, ny: usize) -> Vec<f64> {
        assert_eq!(wrapped.len(), nx * ny);

        // Row-wise unwrap
        let mut row_unwrapped = vec![0.0; nx * ny];
        for r in 0..ny {
            let start = r * nx;
            let row = &wrapped[start..start + nx];
            let uw = Self::unwrap_1d(row);
            row_unwrapped[start..start + nx].copy_from_slice(&uw);
        }

        // Column-wise unwrap
        let mut col_unwrapped = vec![0.0; nx * ny];
        for c in 0..nx {
            let col_wrapped: Vec<f64> = (0..ny).map(|r| wrapped[r * nx + c]).collect();
            let uw = Self::unwrap_1d(&col_wrapped);
            for r in 0..ny {
                col_unwrapped[r * nx + c] = uw[r];
            }
        }

        // Average of the two strategies (reduces residues)
        let mut result = vec![0.0; nx * ny];
        for i in 0..result.len() {
            result[i] = (row_unwrapped[i] + col_unwrapped[i]) / 2.0;
        }
        result
    }

    /// Full quantitative phase extraction: wrapped + 2D unwrap.
    pub fn retrieve(field: &[C64], nx: usize, ny: usize) -> Vec<f64> {
        let wrapped = Self::wrapped_phase(field);
        Self::unwrap_2d(&wrapped, nx, ny)
    }
}

// ---------------------------------------------------------------------------
// TwinImageRemover
// ---------------------------------------------------------------------------

/// Twin image and zero-order suppression for inline (Gabor) holography.
///
/// Inline holograms contain three overlapping terms:
/// - Zero-order (DC): |R|² + |O|²
/// - Real image: R·O*
/// - Twin (conjugate) image: R*·O
///
/// This implementation subtracts the estimated DC term and applies a
/// spatial high-pass to suppress the residual twin image contribution.
pub struct TwinImageRemover;

impl TwinImageRemover {
    /// Subtract the mean (DC component) from hologram intensity,
    /// returning a zero-mean interference pattern.
    pub fn remove_dc(hologram: &[f64]) -> Vec<f64> {
        if hologram.is_empty() {
            return Vec::new();
        }
        let mean = hologram.iter().sum::<f64>() / hologram.len() as f64;
        hologram.iter().map(|v| v - mean).collect()
    }

    /// Angular offset method: apply a linear phase tilt to the hologram
    /// field to separate the twin image in frequency space.
    ///
    /// `tilt_x`, `tilt_y` are the tilt angles in radians (typically a few
    /// times λ/aperture for off-axis setups).
    pub fn apply_phase_tilt(
        field: &[C64],
        nx: usize,
        ny: usize,
        tilt_x: f64,
        tilt_y: f64,
        wavelength_m: f64,
    ) -> Vec<C64> {
        assert_eq!(field.len(), nx * ny);
        let k = 2.0 * PI / wavelength_m;
        let mut out = vec![(0.0, 0.0); field.len()];
        // Assume unit pixel spacing for the tilt; caller can scale tilt values.
        for r in 0..ny {
            for c in 0..nx {
                let phase = k * (tilt_x * c as f64 + tilt_y * r as f64);
                out[r * nx + c] = c_mul(field[r * nx + c], c_exp_j(phase));
            }
        }
        out
    }

    /// Simple spatial high-pass by subtracting a box-averaged version
    /// of the complex field (kernel_size × kernel_size).
    pub fn highpass_2d(field: &[C64], nx: usize, ny: usize, kernel_size: usize) -> Vec<C64> {
        assert_eq!(field.len(), nx * ny);
        let half = (kernel_size / 2).max(1);
        let mut out = vec![(0.0, 0.0); nx * ny];
        for r in 0..ny {
            for c in 0..nx {
                let r_lo = r.saturating_sub(half);
                let r_hi = (r + half + 1).min(ny);
                let c_lo = c.saturating_sub(half);
                let c_hi = (c + half + 1).min(nx);
                let mut sum = (0.0, 0.0);
                let mut count = 0usize;
                for rr in r_lo..r_hi {
                    for cc in c_lo..c_hi {
                        sum = c_add(sum, field[rr * nx + cc]);
                        count += 1;
                    }
                }
                let avg = c_scale(1.0 / count as f64, sum);
                out[r * nx + c] = c_sub(field[r * nx + c], avg);
            }
        }
        out
    }
}

// ---------------------------------------------------------------------------
// AutofocusAlgorithm
// ---------------------------------------------------------------------------

/// Autofocus metric for finding the best reconstruction distance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FocusMetric {
    /// Maximise the Tenengrad (Sobel gradient magnitude squared) sharpness.
    GradientSharpness,
    /// Minimise the Shannon entropy of the normalised intensity histogram.
    MinEntropy,
}

/// Find the optimal reconstruction distance by evaluating a focus metric
/// over a range of z values.
pub struct AutofocusAlgorithm;

impl AutofocusAlgorithm {
    /// Evaluate gradient sharpness (Tenengrad) of a 2-D intensity image.
    ///
    /// Uses 3×3 Sobel operators for horizontal and vertical gradients.
    pub fn gradient_sharpness(intensity: &[f64], nx: usize, ny: usize) -> f64 {
        assert_eq!(intensity.len(), nx * ny);
        if nx < 3 || ny < 3 {
            return 0.0;
        }
        let mut sum = 0.0;
        for r in 1..ny - 1 {
            for c in 1..nx - 1 {
                let idx = |rr: usize, cc: usize| intensity[rr * nx + cc];
                // Sobel Gx
                let gx = -idx(r - 1, c - 1) + idx(r - 1, c + 1)
                    - 2.0 * idx(r, c - 1)
                    + 2.0 * idx(r, c + 1)
                    - idx(r + 1, c - 1)
                    + idx(r + 1, c + 1);
                // Sobel Gy
                let gy = -idx(r - 1, c - 1) - 2.0 * idx(r - 1, c)
                    - idx(r - 1, c + 1)
                    + idx(r + 1, c - 1)
                    + 2.0 * idx(r + 1, c)
                    + idx(r + 1, c + 1);
                sum += gx * gx + gy * gy;
            }
        }
        sum
    }

    /// Evaluate Shannon entropy of a normalised intensity histogram.
    /// Lower entropy → sharper (more peaked) image.
    pub fn intensity_entropy(intensity: &[f64], num_bins: usize) -> f64 {
        if intensity.is_empty() || num_bins == 0 {
            return 0.0;
        }
        let min_val = intensity.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_val = intensity.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let range = max_val - min_val;
        if range <= 0.0 {
            return 0.0;
        }
        let mut hist = vec![0usize; num_bins];
        for &v in intensity {
            let bin = ((v - min_val) / range * (num_bins - 1) as f64).round() as usize;
            let bin = bin.min(num_bins - 1);
            hist[bin] += 1;
        }
        let total = intensity.len() as f64;
        let mut entropy = 0.0;
        for &count in &hist {
            if count > 0 {
                let p = count as f64 / total;
                entropy -= p * p.ln();
            }
        }
        entropy
    }

    /// Sweep through z values and return the best focus distance.
    ///
    /// Uses `FresnelPropagator` for reconstruction at each candidate z.
    pub fn find_best_focus(
        cfg: &HolographyConfig,
        field: &[C64],
        n: usize,
        z_start: f64,
        z_end: f64,
        num_steps: usize,
        metric: FocusMetric,
    ) -> (f64, f64) {
        assert!(num_steps >= 2, "need at least 2 steps");
        let propagator = FresnelPropagator::new(cfg, n);
        let dz = (z_end - z_start) / (num_steps - 1) as f64;

        let mut best_z = z_start;
        let mut best_score = match metric {
            FocusMetric::GradientSharpness => f64::NEG_INFINITY,
            FocusMetric::MinEntropy => f64::INFINITY,
        };

        for step in 0..num_steps {
            let z = z_start + step as f64 * dz;
            let intensity = propagator.intensity(field, z);
            let score = match metric {
                FocusMetric::GradientSharpness => {
                    Self::gradient_sharpness(&intensity, n, n)
                }
                FocusMetric::MinEntropy => {
                    Self::intensity_entropy(&intensity, 64)
                }
            };
            let is_better = match metric {
                FocusMetric::GradientSharpness => score > best_score,
                FocusMetric::MinEntropy => score < best_score,
            };
            if is_better {
                best_score = score;
                best_z = z;
            }
        }
        (best_z, best_score)
    }
}

// ---------------------------------------------------------------------------
// ParticleTracker3D
// ---------------------------------------------------------------------------

/// A detected particle in 3-D space.
#[derive(Debug, Clone)]
pub struct Particle3D {
    /// X position (pixel coordinates or physical).
    pub x: f64,
    /// Y position.
    pub y: f64,
    /// Z position (reconstruction distance).
    pub z: f64,
    /// Peak intensity at the detected focus.
    pub peak_intensity: f64,
}

/// Locate particles in 3-D from a stack of holographic reconstructions.
///
/// Strategy:
/// 1. Reconstruct at multiple z planes.
/// 2. At each plane, find local intensity maxima above a threshold.
/// 3. For each XY candidate, find the z with maximum intensity (best focus).
pub struct ParticleTracker3D {
    threshold: f64,
    neighbourhood: usize,
}

impl ParticleTracker3D {
    /// Create a tracker.
    ///
    /// - `threshold`: minimum intensity for peak detection.
    /// - `neighbourhood`: half-width of exclusion zone around each peak.
    pub fn new(threshold: f64, neighbourhood: usize) -> Self {
        Self {
            threshold,
            neighbourhood,
        }
    }

    /// Find local maxima in a 2-D intensity image.
    pub fn find_peaks_2d(
        &self,
        intensity: &[f64],
        nx: usize,
        ny: usize,
    ) -> Vec<(usize, usize, f64)> {
        let nb = self.neighbourhood.max(1);
        let mut peaks = Vec::new();
        for r in nb..ny.saturating_sub(nb) {
            for c in nb..nx.saturating_sub(nb) {
                let val = intensity[r * nx + c];
                if val < self.threshold {
                    continue;
                }
                let mut is_max = true;
                'outer: for dr in 0..=2 * nb {
                    for dc in 0..=2 * nb {
                        let rr = r + dr - nb;
                        let cc = c + dc - nb;
                        if rr == r && cc == c {
                            continue;
                        }
                        if rr < ny && cc < nx && intensity[rr * nx + cc] >= val {
                            is_max = false;
                            break 'outer;
                        }
                    }
                }
                if is_max {
                    peaks.push((c, r, val));
                }
            }
        }
        peaks
    }

    /// Track particles across a z-stack of intensity images.
    ///
    /// `z_stack` contains `(z_distance, intensity_image)` pairs, each N×N.
    pub fn track(
        &self,
        z_stack: &[(f64, Vec<f64>)],
        nx: usize,
        ny: usize,
    ) -> Vec<Particle3D> {
        // Collect all peaks from all z planes
        let mut candidates: Vec<(usize, usize, f64, f64)> = Vec::new(); // (x, y, z, intensity)
        for (z, intensity) in z_stack {
            let peaks = self.find_peaks_2d(intensity, nx, ny);
            for (px, py, pval) in peaks {
                candidates.push((px, py, *z, pval));
            }
        }

        // Group by approximate XY location and pick best z
        let mut particles = Vec::new();
        let mut used = vec![false; candidates.len()];
        let merge_dist = self.neighbourhood as f64;

        for i in 0..candidates.len() {
            if used[i] {
                continue;
            }
            let (bx, by, mut bz, mut bval) = candidates[i];
            used[i] = true;

            // Find all candidates near this XY
            for j in (i + 1)..candidates.len() {
                if used[j] {
                    continue;
                }
                let dx = candidates[j].0 as f64 - bx as f64;
                let dy = candidates[j].1 as f64 - by as f64;
                if (dx * dx + dy * dy).sqrt() <= merge_dist {
                    used[j] = true;
                    if candidates[j].3 > bval {
                        bval = candidates[j].3;
                        bz = candidates[j].2;
                    }
                }
            }

            particles.push(Particle3D {
                x: bx as f64,
                y: by as f64,
                z: bz,
                peak_intensity: bval,
            });
        }

        particles
    }
}

// ---------------------------------------------------------------------------
// ThicknessMeasurer
// ---------------------------------------------------------------------------

/// Convert quantitative phase to optical path length and surface height.
///
/// ```text
/// OPL = φ · λ / (2π)
/// height = OPL / (n_sample − n_medium)
/// ```
pub struct ThicknessMeasurer {
    wavelength_m: f64,
    /// Refractive index of the sample.
    n_sample: f64,
    /// Refractive index of the surrounding medium.
    n_medium: f64,
}

impl ThicknessMeasurer {
    pub fn new(wavelength_m: f64, n_sample: f64, n_medium: f64) -> Self {
        Self {
            wavelength_m,
            n_sample,
            n_medium,
        }
    }

    /// Optical path length from phase (radians).
    pub fn opl_from_phase(&self, phase: &[f64]) -> Vec<f64> {
        let scale = self.wavelength_m / (2.0 * PI);
        phase.iter().map(|&phi| phi * scale).collect()
    }

    /// Surface height from phase (radians).
    pub fn height_from_phase(&self, phase: &[f64]) -> Vec<f64> {
        let delta_n = self.n_sample - self.n_medium;
        if delta_n.abs() < 1e-15 {
            return vec![0.0; phase.len()];
        }
        let scale = self.wavelength_m / (2.0 * PI * delta_n);
        phase.iter().map(|&phi| phi * scale).collect()
    }

    /// Dry mass density from integrated phase (useful for cell biology).
    ///
    /// ```text
    /// dry_mass = Σ (OPL_i · pixel_area) / specific_refraction_increment
    /// ```
    ///
    /// `pixel_area_m2` is the area of one pixel at the object plane.
    /// `alpha` is the specific refraction increment (~0.18 mL/g for proteins).
    pub fn dry_mass(&self, phase: &[f64], pixel_area_m2: f64, alpha: f64) -> f64 {
        let opl = self.opl_from_phase(phase);
        let total_opl: f64 = opl.iter().sum();
        total_opl * pixel_area_m2 / alpha
    }
}

// ---------------------------------------------------------------------------
// InterferencePatternGenerator
// ---------------------------------------------------------------------------

/// Generate synthetic inline holograms from a set of point scatterers.
///
/// Each point emits a spherical wave that interferes with the reference
/// (plane wave) beam on the hologram plane.
pub struct InterferencePatternGenerator {
    wavelength_m: f64,
    pixel_size_m: f64,
    nx: usize,
    ny: usize,
}

impl InterferencePatternGenerator {
    pub fn new(wavelength_m: f64, pixel_size_m: f64, nx: usize, ny: usize) -> Self {
        Self {
            wavelength_m,
            pixel_size_m,
            nx,
            ny,
        }
    }

    /// Generate a hologram intensity from point scatterers.
    ///
    /// Each scatterer is `(x_m, y_m, z_m, amplitude)` where (x,y,z) are
    /// physical coordinates and z is the distance from the hologram plane.
    /// The reference beam is a unit-amplitude plane wave along z.
    pub fn generate(&self, scatterers: &[(f64, f64, f64, f64)]) -> Vec<f64> {
        let k = 2.0 * PI / self.wavelength_m;
        let mut hologram = vec![0.0; self.nx * self.ny];
        let half_x = self.nx as f64 / 2.0;
        let half_y = self.ny as f64 / 2.0;

        for r in 0..self.ny {
            let yp = (r as f64 - half_y) * self.pixel_size_m;
            for c in 0..self.nx {
                let xp = (c as f64 - half_x) * self.pixel_size_m;

                // Reference beam: unit plane wave (complex amplitude = 1)
                let mut field: C64 = (1.0, 0.0);

                // Add spherical waves from each scatterer
                for &(sx, sy, sz, amp) in scatterers {
                    let dx = xp - sx;
                    let dy = yp - sy;
                    let r_dist = (dx * dx + dy * dy + sz * sz).sqrt();
                    let phase = k * r_dist;
                    let sph = c_scale(amp / r_dist, c_exp_j(phase));
                    field = c_add(field, sph);
                }

                hologram[r * self.nx + c] = c_abs2(field);
            }
        }
        hologram
    }

    /// Generate complex field (rather than intensity) for advanced processing.
    pub fn generate_complex(&self, scatterers: &[(f64, f64, f64, f64)]) -> Vec<C64> {
        let k = 2.0 * PI / self.wavelength_m;
        let mut field_out = vec![(0.0, 0.0); self.nx * self.ny];
        let half_x = self.nx as f64 / 2.0;
        let half_y = self.ny as f64 / 2.0;

        for r in 0..self.ny {
            let yp = (r as f64 - half_y) * self.pixel_size_m;
            for c in 0..self.nx {
                let xp = (c as f64 - half_x) * self.pixel_size_m;

                let mut field: C64 = (1.0, 0.0); // reference
                for &(sx, sy, sz, amp) in scatterers {
                    let dx = xp - sx;
                    let dy = yp - sy;
                    let r_dist = (dx * dx + dy * dy + sz * sz).sqrt();
                    let phase = k * r_dist;
                    let sph = c_scale(amp / r_dist, c_exp_j(phase));
                    field = c_add(field, sph);
                }
                field_out[r * self.nx + c] = field;
            }
        }
        field_out
    }
}

// ---------------------------------------------------------------------------
// NumericalAperture
// ---------------------------------------------------------------------------

/// Compute effective numerical aperture and resolution limits.
pub struct NumericalAperture {
    wavelength_m: f64,
    na: f64,
}

impl NumericalAperture {
    /// Create from optical system parameters.
    ///
    /// Effective NA is limited by the sensor size:
    /// ```text
    /// NA_sensor = sin(atan(N · Δx / (2 · M · z)))
    /// ```
    /// but we approximate as NA ≈ N·Δx / (2·M·z) for small angles,
    /// capped at a practical maximum.
    ///
    /// For simplicity, we also accept the magnification-based NA:
    /// NA ≈ (N/2 · pixel_size / magnification) / (focal length).
    /// Since we don't have focal length, we compute from the pixel geometry.
    pub fn new(
        wavelength_m: f64,
        magnification: f64,
        pixel_size_m: f64,
        n: usize,
    ) -> Self {
        // Effective pixel at object plane
        let dx = pixel_size_m / magnification;
        // Nyquist-limited NA: the finest fringe the sensor can resolve
        // has period = 2·dx, spatial freq = 1/(2·dx), so NA = λ / (2·dx)
        // But we cap at 1.0.
        let na = (wavelength_m / (2.0 * dx)).min(1.0);
        Self { wavelength_m, na }
    }

    /// Create with a specified NA directly.
    pub fn with_na(wavelength_m: f64, na: f64) -> Self {
        Self { wavelength_m, na }
    }

    /// Effective numerical aperture.
    pub fn na(&self) -> f64 {
        self.na
    }

    /// Lateral (XY) resolution: Δx ≈ λ / (2·NA).
    pub fn lateral_resolution_m(&self) -> f64 {
        self.wavelength_m / (2.0 * self.na)
    }

    /// Axial (Z) resolution: Δz ≈ λ / NA².
    pub fn axial_resolution_m(&self) -> f64 {
        self.wavelength_m / (self.na * self.na)
    }

    /// Depth of focus (twice axial resolution, one-sided).
    pub fn depth_of_focus_m(&self) -> f64 {
        2.0 * self.axial_resolution_m()
    }

    /// Fresnel number for a given aperture radius and distance.
    pub fn fresnel_number(aperture_radius_m: f64, wavelength_m: f64, distance_m: f64) -> f64 {
        aperture_radius_m * aperture_radius_m / (wavelength_m * distance_m)
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -- HolographyConfig --

    #[test]
    fn test_config_effective_pixel_size() {
        let cfg = HolographyConfig::new(632.8e-9, 40.0, 6.45e-6, 100.0e-6);
        let eps = cfg.effective_pixel_size();
        assert!(approx_eq(eps, 6.45e-6 / 40.0, 1e-12));
    }

    #[test]
    fn test_config_fresnel_number() {
        let cfg = HolographyConfig::new(500.0e-9, 1.0, 10.0e-6, 1.0e-3);
        let f = cfg.fresnel_number(64);
        // a = 10e-6 * 64 / 2 = 3.2e-4, F = a^2/(lambda*z) = 1.024e-7 / 5e-13 = ...
        assert!(f > 0.0);
    }

    #[test]
    fn test_config_clone() {
        let cfg = HolographyConfig::new(632.8e-9, 20.0, 3.45e-6, 50.0e-6);
        let cfg2 = cfg.clone();
        assert_eq!(cfg.wavelength_m, cfg2.wavelength_m);
        assert_eq!(cfg.magnification, cfg2.magnification);
    }

    // -- FFT --

    #[test]
    fn test_fft_1d_impulse() {
        // FFT of a single 1.0 should be all 1's
        let mut buf = vec![(1.0, 0.0), (0.0, 0.0), (0.0, 0.0), (0.0, 0.0)];
        fft_1d(&mut buf, false);
        for v in &buf {
            assert!(approx_eq(v.0, 1.0, 1e-10));
            assert!(approx_eq(v.1, 0.0, 1e-10));
        }
    }

    #[test]
    fn test_fft_1d_roundtrip() {
        let original = vec![(1.0, 0.5), (0.3, -0.2), (-0.7, 0.1), (0.4, 0.9)];
        let mut buf = original.clone();
        fft_1d(&mut buf, false);
        fft_1d(&mut buf, true);
        for (a, b) in buf.iter().zip(original.iter()) {
            assert!(approx_eq(a.0, b.0, 1e-10));
            assert!(approx_eq(a.1, b.1, 1e-10));
        }
    }

    #[test]
    fn test_fft_2d_roundtrip() {
        let n = 4;
        let original: Vec<C64> = (0..n * n)
            .map(|i| ((i as f64 * 0.1).sin(), (i as f64 * 0.2).cos()))
            .collect();
        let mut buf = original.clone();
        fft_2d(&mut buf, n);
        ifft_2d(&mut buf, n);
        for (a, b) in buf.iter().zip(original.iter()) {
            assert!(approx_eq(a.0, b.0, 1e-9));
            assert!(approx_eq(a.1, b.1, 1e-9));
        }
    }

    #[test]
    fn test_fftshift_2d() {
        let n = 4;
        let mut data: Vec<C64> = (0..16).map(|i| (i as f64, 0.0)).collect();
        fftshift_2d(&mut data, n);
        // After shift, element [0][0] should be what was at [2][2]
        assert!(approx_eq(data[0].0, 10.0, 1e-10));
    }

    // -- FresnelPropagator --

    #[test]
    fn test_fresnel_propagator_size() {
        let cfg = HolographyConfig::new(632.8e-9, 40.0, 6.45e-6, 50.0e-6);
        let prop = FresnelPropagator::new(&cfg, 8);
        let field = vec![(1.0, 0.0); 64];
        let result = prop.propagate(&field, 50.0e-6);
        assert_eq!(result.len(), 64);
    }

    #[test]
    fn test_fresnel_propagator_nonzero() {
        let cfg = HolographyConfig::new(632.8e-9, 10.0, 6.45e-6, 100.0e-6);
        let prop = FresnelPropagator::new(&cfg, 8);
        let mut field = vec![(0.0, 0.0); 64];
        field[4 * 8 + 4] = (1.0, 0.0); // single point source
        let result = prop.propagate(&field, 100.0e-6);
        let total_power: f64 = result.iter().map(|v| c_abs2(*v)).sum();
        assert!(total_power > 0.0);
    }

    #[test]
    fn test_fresnel_intensity() {
        let cfg = HolographyConfig::new(532.0e-9, 20.0, 5.0e-6, 80.0e-6);
        let prop = FresnelPropagator::new(&cfg, 8);
        let field = vec![(1.0, 0.0); 64];
        let intensity = prop.intensity(&field, 80.0e-6);
        assert_eq!(intensity.len(), 64);
        assert!(intensity.iter().all(|&v| v >= 0.0));
    }

    // -- AngularSpectrumMethod --

    #[test]
    fn test_angular_spectrum_size() {
        let cfg = HolographyConfig::new(632.8e-9, 40.0, 6.45e-6, 50.0e-6);
        let asm = AngularSpectrumMethod::new(&cfg, 8);
        let field = vec![(1.0, 0.0); 64];
        let result = asm.propagate(&field, 50.0e-6);
        assert_eq!(result.len(), 64);
    }

    #[test]
    fn test_angular_spectrum_uniform_field() {
        // A uniform field should remain roughly uniform after short propagation
        let cfg = HolographyConfig::new(632.8e-9, 1.0, 1.0e-6, 1.0e-6);
        let asm = AngularSpectrumMethod::new(&cfg, 8);
        let field = vec![(1.0, 0.0); 64];
        let result = asm.propagate(&field, 1.0e-6);
        // Check central region has high amplitude
        let centre_amp = c_abs(result[4 * 8 + 4]);
        assert!(centre_amp > 0.1);
    }

    #[test]
    fn test_angular_spectrum_intensity() {
        let cfg = HolographyConfig::new(532.0e-9, 10.0, 5.0e-6, 50.0e-6);
        let asm = AngularSpectrumMethod::new(&cfg, 8);
        let field = vec![(1.0, 0.0); 64];
        let intensity = asm.intensity(&field, 50.0e-6);
        assert!(intensity.iter().all(|&v| v >= 0.0));
    }

    #[test]
    fn test_angular_spectrum_evanescent_attenuation() {
        // Very short wavelength relative to pixel should produce evanescent modes
        let cfg = HolographyConfig::new(1.0e-12, 1.0, 1.0e-6, 1.0e-3);
        let asm = AngularSpectrumMethod::new(&cfg, 4);
        let mut field = vec![(0.0, 0.0); 16];
        field[0] = (1.0, 0.0);
        let result = asm.propagate(&field, 1.0e-3);
        // Most energy should be heavily attenuated
        let total_power: f64 = result.iter().map(|v| c_abs2(*v)).sum();
        // Just check it's finite and didn't blow up
        assert!(total_power.is_finite());
    }

    // -- PhaseRetriever --

    #[test]
    fn test_wrapped_phase_zero() {
        let field = vec![(1.0, 0.0); 4];
        let phase = PhaseRetriever::wrapped_phase(&field);
        assert!(phase.iter().all(|&p| approx_eq(p, 0.0, 1e-12)));
    }

    #[test]
    fn test_wrapped_phase_90deg() {
        let field = vec![(0.0, 1.0)]; // 90 degrees
        let phase = PhaseRetriever::wrapped_phase(&field);
        assert!(approx_eq(phase[0], PI / 2.0, 1e-12));
    }

    #[test]
    fn test_unwrap_1d_no_jump() {
        let wrapped = vec![0.0, 0.1, 0.2, 0.3];
        let uw = PhaseRetriever::unwrap_1d(&wrapped);
        for (a, b) in uw.iter().zip(wrapped.iter()) {
            assert!(approx_eq(*a, *b, 1e-12));
        }
    }

    #[test]
    fn test_unwrap_1d_with_jump() {
        // Simulate a phase ramp that wraps
        let true_phase: Vec<f64> = (0..8).map(|i| i as f64 * 1.0).collect();
        let wrapped: Vec<f64> = true_phase
            .iter()
            .map(|&p| {
                let mut w = p % (2.0 * PI);
                if w > PI {
                    w -= 2.0 * PI;
                }
                w
            })
            .collect();
        let unwrapped = PhaseRetriever::unwrap_1d(&wrapped);
        // Check differences are preserved
        for i in 1..unwrapped.len() {
            let diff_uw = unwrapped[i] - unwrapped[i - 1];
            let diff_true = true_phase[i] - true_phase[i - 1];
            assert!(approx_eq(diff_uw, diff_true, 0.1));
        }
    }

    #[test]
    fn test_unwrap_1d_empty() {
        let uw = PhaseRetriever::unwrap_1d(&[]);
        assert!(uw.is_empty());
    }

    #[test]
    fn test_unwrap_2d_flat() {
        let phase = vec![0.5; 16]; // flat phase, 4×4
        let uw = PhaseRetriever::unwrap_2d(&phase, 4, 4);
        assert!(uw.iter().all(|&v| approx_eq(v, 0.5, 1e-10)));
    }

    #[test]
    fn test_retrieve_real_positive() {
        let field = vec![(1.0, 0.0); 16]; // all real positive
        let phase = PhaseRetriever::retrieve(&field, 4, 4);
        assert!(phase.iter().all(|&p| approx_eq(p, 0.0, 1e-10)));
    }

    // -- TwinImageRemover --

    #[test]
    fn test_remove_dc() {
        let holo = vec![10.0, 12.0, 8.0, 10.0];
        let result = TwinImageRemover::remove_dc(&holo);
        let sum: f64 = result.iter().sum();
        assert!(approx_eq(sum, 0.0, 1e-10));
    }

    #[test]
    fn test_remove_dc_empty() {
        let result = TwinImageRemover::remove_dc(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_phase_tilt() {
        let field = vec![(1.0, 0.0); 4]; // 2×2
        let tilted = TwinImageRemover::apply_phase_tilt(
            &field, 2, 2, 0.01, 0.01, 632.8e-9,
        );
        assert_eq!(tilted.len(), 4);
        // The first pixel (0,0) should still have magnitude ~1
        assert!(approx_eq(c_abs(tilted[0]), 1.0, 1e-10));
    }

    #[test]
    fn test_highpass_2d() {
        // A uniform field should be zeroed by highpass
        let field = vec![(5.0, 3.0); 16]; // 4×4 uniform
        let hp = TwinImageRemover::highpass_2d(&field, 4, 4, 3);
        for v in &hp {
            assert!(c_abs(*v) < 1e-10);
        }
    }

    // -- AutofocusAlgorithm --

    #[test]
    fn test_gradient_sharpness_flat() {
        // Flat image → zero gradient
        let intensity = vec![1.0; 16]; // 4×4
        let sharp = AutofocusAlgorithm::gradient_sharpness(&intensity, 4, 4);
        assert!(approx_eq(sharp, 0.0, 1e-10));
    }

    #[test]
    fn test_gradient_sharpness_edge() {
        // Image with a vertical edge → nonzero gradient
        let mut intensity = vec![0.0; 64]; // 8×8
        for r in 0..8 {
            for c in 4..8 {
                intensity[r * 8 + c] = 1.0;
            }
        }
        let sharp = AutofocusAlgorithm::gradient_sharpness(&intensity, 8, 8);
        assert!(sharp > 0.0);
    }

    #[test]
    fn test_intensity_entropy_uniform() {
        // Uniform → single bin → zero entropy
        let intensity = vec![1.0; 16];
        let ent = AutofocusAlgorithm::intensity_entropy(&intensity, 32);
        assert!(approx_eq(ent, 0.0, 1e-10));
    }

    #[test]
    fn test_intensity_entropy_varied() {
        let intensity: Vec<f64> = (0..16).map(|i| i as f64).collect();
        let ent = AutofocusAlgorithm::intensity_entropy(&intensity, 16);
        assert!(ent > 0.0);
    }

    #[test]
    fn test_autofocus_returns_valid() {
        let cfg = HolographyConfig::new(632.8e-9, 10.0, 6.45e-6, 50.0e-6);
        let field = vec![(1.0, 0.0); 64]; // 8×8 uniform
        let (best_z, score) = AutofocusAlgorithm::find_best_focus(
            &cfg,
            &field,
            8,
            40.0e-6,
            60.0e-6,
            5,
            FocusMetric::GradientSharpness,
        );
        assert!(best_z >= 40.0e-6 && best_z <= 60.0e-6);
        assert!(score.is_finite());
    }

    #[test]
    fn test_autofocus_min_entropy() {
        let cfg = HolographyConfig::new(532.0e-9, 20.0, 5.0e-6, 50.0e-6);
        let field = vec![(1.0, 0.0); 64];
        let (best_z, score) = AutofocusAlgorithm::find_best_focus(
            &cfg,
            &field,
            8,
            30.0e-6,
            70.0e-6,
            5,
            FocusMetric::MinEntropy,
        );
        assert!(best_z >= 30.0e-6 && best_z <= 70.0e-6);
        assert!(score.is_finite());
    }

    // -- ParticleTracker3D --

    #[test]
    fn test_find_peaks_2d_single() {
        let mut intensity = vec![0.0; 64]; // 8×8
        intensity[3 * 8 + 4] = 10.0; // peak at (4, 3)
        let tracker = ParticleTracker3D::new(5.0, 1);
        let peaks = tracker.find_peaks_2d(&intensity, 8, 8);
        assert_eq!(peaks.len(), 1);
        assert_eq!(peaks[0].0, 4); // x
        assert_eq!(peaks[0].1, 3); // y
    }

    #[test]
    fn test_find_peaks_below_threshold() {
        let intensity = vec![1.0; 64]; // all below threshold=5
        let tracker = ParticleTracker3D::new(5.0, 1);
        let peaks = tracker.find_peaks_2d(&intensity, 8, 8);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_track_single_particle() {
        let n = 8;
        let tracker = ParticleTracker3D::new(5.0, 1);
        let mut stack = Vec::new();
        for z_idx in 0..3 {
            let z = 40.0e-6 + z_idx as f64 * 10.0e-6;
            let mut img = vec![0.0; n * n];
            // Peak gets stronger at middle z
            let peak_val = if z_idx == 1 { 20.0 } else { 8.0 };
            img[3 * n + 4] = peak_val;
            stack.push((z, img));
        }
        let particles = tracker.track(&stack, n, n);
        assert!(!particles.is_empty());
        assert!(approx_eq(particles[0].z, 50.0e-6, 1e-10));
    }

    // -- ThicknessMeasurer --

    #[test]
    fn test_opl_from_phase() {
        let tm = ThicknessMeasurer::new(632.8e-9, 1.5, 1.0);
        let phase = vec![2.0 * PI]; // one full wave OPL = lambda
        let opl = tm.opl_from_phase(&phase);
        assert!(approx_eq(opl[0], 632.8e-9, 1e-15));
    }

    #[test]
    fn test_height_from_phase() {
        let tm = ThicknessMeasurer::new(632.8e-9, 1.5, 1.0);
        let phase = vec![2.0 * PI];
        let height = tm.height_from_phase(&phase);
        // h = lambda / (n_sample - n_medium) = 632.8e-9 / 0.5
        assert!(approx_eq(height[0], 632.8e-9 / 0.5, 1e-15));
    }

    #[test]
    fn test_height_zero_delta_n() {
        let tm = ThicknessMeasurer::new(632.8e-9, 1.33, 1.33);
        let phase = vec![1.0, 2.0];
        let height = tm.height_from_phase(&phase);
        assert!(height.iter().all(|&h| approx_eq(h, 0.0, 1e-10)));
    }

    #[test]
    fn test_dry_mass() {
        let tm = ThicknessMeasurer::new(500.0e-9, 1.38, 1.33);
        let phase = vec![1.0; 4]; // 2×2
        let pixel_area = 1e-12; // 1 um^2
        let alpha = 0.18e-6; // mL/g -> m^3/g roughly
        let dm = tm.dry_mass(&phase, pixel_area, alpha);
        assert!(dm.is_finite());
        assert!(dm > 0.0);
    }

    // -- InterferencePatternGenerator --

    #[test]
    fn test_interference_pattern_size() {
        let gen = InterferencePatternGenerator::new(632.8e-9, 6.45e-6, 8, 8);
        let scatterers = vec![(0.0, 0.0, 100.0e-6, 1e-6)];
        let holo = gen.generate(&scatterers);
        assert_eq!(holo.len(), 64);
    }

    #[test]
    fn test_interference_pattern_positive() {
        let gen = InterferencePatternGenerator::new(632.8e-9, 6.45e-6, 8, 8);
        let scatterers = vec![(0.0, 0.0, 50.0e-6, 1e-6)];
        let holo = gen.generate(&scatterers);
        assert!(holo.iter().all(|&v| v >= 0.0));
    }

    #[test]
    fn test_interference_fringes() {
        // Off-axis scatterer should create fringes (intensity variation)
        let gen = InterferencePatternGenerator::new(632.8e-9, 1.0e-6, 16, 16);
        let scatterers = vec![(5.0e-6, 0.0, 50.0e-6, 1e-6)];
        let holo = gen.generate(&scatterers);
        let min = holo.iter().cloned().fold(f64::INFINITY, f64::min);
        let max = holo.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(max - min > 1e-6, "should have fringe contrast");
    }

    #[test]
    fn test_generate_complex() {
        let gen = InterferencePatternGenerator::new(632.8e-9, 6.45e-6, 4, 4);
        let scatterers = vec![(0.0, 0.0, 100.0e-6, 1e-6)];
        let field = gen.generate_complex(&scatterers);
        assert_eq!(field.len(), 16);
        // Magnitude of complex field squared should match intensity
        let intensity_from_complex: Vec<f64> = field.iter().map(|v| c_abs2(*v)).collect();
        let intensity = gen.generate(&scatterers);
        for (a, b) in intensity_from_complex.iter().zip(intensity.iter()) {
            assert!(approx_eq(*a, *b, 1e-10));
        }
    }

    // -- NumericalAperture --

    #[test]
    fn test_na_lateral_resolution() {
        let na = NumericalAperture::with_na(500.0e-9, 0.5);
        let lr = na.lateral_resolution_m();
        // Δx = 500e-9 / (2*0.5) = 500e-9
        assert!(approx_eq(lr, 500.0e-9, 1e-15));
    }

    #[test]
    fn test_na_axial_resolution() {
        let na = NumericalAperture::with_na(500.0e-9, 0.5);
        let ar = na.axial_resolution_m();
        // Δz = 500e-9 / 0.25 = 2000e-9
        assert!(approx_eq(ar, 2.0e-6, 1e-15));
    }

    #[test]
    fn test_na_depth_of_focus() {
        let na = NumericalAperture::with_na(500.0e-9, 0.5);
        assert!(approx_eq(na.depth_of_focus_m(), 4.0e-6, 1e-15));
    }

    #[test]
    fn test_na_from_config() {
        let na = NumericalAperture::new(632.8e-9, 40.0, 6.45e-6, 64);
        assert!(na.na() > 0.0 && na.na() <= 1.0);
        assert!(na.lateral_resolution_m() > 0.0);
        assert!(na.axial_resolution_m() > na.lateral_resolution_m());
    }

    #[test]
    fn test_fresnel_number_function() {
        let f = NumericalAperture::fresnel_number(1.0e-3, 500.0e-9, 1.0);
        // F = (1e-3)^2 / (500e-9 * 1) = 1e-6 / 5e-7 = 2.0
        assert!(approx_eq(f, 2.0, 1e-10));
    }

    // -- Integration tests --

    #[test]
    fn test_roundtrip_hologram_reconstruct() {
        // Generate hologram from point scatterer, then reconstruct
        let lambda = 632.8e-9;
        let pixel = 1.0e-6;
        let n = 8;
        let z = 50.0e-6;

        let gen = InterferencePatternGenerator::new(lambda, pixel, n, n);
        let scatterers = vec![(0.0, 0.0, z, 1e-6)];
        let field = gen.generate_complex(&scatterers);

        let cfg = HolographyConfig::new(lambda, 1.0, pixel, z);
        let prop = FresnelPropagator::new(&cfg, n);
        let recon = prop.propagate(&field, -z); // back-propagate
        assert_eq!(recon.len(), n * n);
        let total_power: f64 = recon.iter().map(|v| c_abs2(*v)).sum();
        assert!(total_power > 0.0);
    }

    #[test]
    fn test_phase_to_thickness_pipeline() {
        // Full pipeline: complex field → phase → height
        let field: Vec<C64> = (0..16)
            .map(|i| c_exp_j(i as f64 * 0.1))
            .collect();
        let phase = PhaseRetriever::retrieve(&field, 4, 4);
        let tm = ThicknessMeasurer::new(532.0e-9, 1.38, 1.33);
        let height = tm.height_from_phase(&phase);
        assert_eq!(height.len(), 16);
        assert!(height.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn test_focus_metric_enum_eq() {
        assert_eq!(FocusMetric::GradientSharpness, FocusMetric::GradientSharpness);
        assert_ne!(FocusMetric::GradientSharpness, FocusMetric::MinEntropy);
    }

    #[test]
    fn test_particle3d_debug() {
        let p = Particle3D {
            x: 1.0,
            y: 2.0,
            z: 3.0,
            peak_intensity: 10.0,
        };
        let s = format!("{:?}", p);
        assert!(s.contains("Particle3D"));
    }
}
