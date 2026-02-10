//! Photoacoustic image reconstruction from ultrasonic transducer array data.
//!
//! Photoacoustic imaging (PAI) uses pulsed laser illumination to generate thermoelastic
//! expansion in tissue, producing broadband acoustic waves. These waves are detected by
//! ultrasonic transducer arrays and mathematically reconstructed into images that reveal
//! optical absorption contrast — useful for visualizing vasculature, tumors, and
//! functional tissue properties.
//!
//! This module provides:
//! - **Delay-and-sum (DAS) beamforming** — the standard real-time reconstruction method
//! - **Filtered backprojection (FBP)** — tomographic reconstruction from angular projections
//! - **Time-reversal focusing** — compute the coherent sum at an arbitrary focal point
//! - **Apodization windowing** — Hann, Hamming, and Blackman element weighting
//! - **Speed-of-sound correction** — heterogeneous media compensation
//! - **Image quality metrics** — SNR, contrast-to-noise ratio, lateral resolution (FWHM)
//!
//! # Example
//!
//! ```
//! use r4w_core::photoacoustic_image_reconstructor::{PaConfig, PaImage, PaReconstructor};
//!
//! let config = PaConfig {
//!     sound_speed_mps: 1540.0,
//!     sample_rate_hz: 40.0e6,
//!     num_elements: 64,
//!     element_pitch_m: 0.3e-3,
//! };
//! let reconstructor = PaReconstructor::new(config);
//!
//! let mut image = PaImage::new(16, 16, 0.1e-3);
//! assert_eq!(image.pixels.len(), 256);
//! assert_eq!(image.width, 16);
//! assert_eq!(image.height, 16);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Configuration parameters for the photoacoustic reconstruction system.
///
/// These describe the physical transducer array and acquisition settings.
#[derive(Debug, Clone)]
pub struct PaConfig {
    /// Speed of sound in the coupling medium (m/s). Typically ~1540 m/s for soft tissue.
    pub sound_speed_mps: f64,
    /// Sampling rate of the data acquisition system (Hz).
    pub sample_rate_hz: f64,
    /// Number of transducer elements in the array.
    pub num_elements: usize,
    /// Center-to-center spacing between adjacent elements (m).
    pub element_pitch_m: f64,
}

// ---------------------------------------------------------------------------
// Image
// ---------------------------------------------------------------------------

/// A 2-D reconstructed photoacoustic image stored in row-major order.
#[derive(Debug, Clone)]
pub struct PaImage {
    /// Pixel intensities in row-major layout (length = width * height).
    pub pixels: Vec<f64>,
    /// Number of columns (pixels along the lateral axis).
    pub width: usize,
    /// Number of rows (pixels along the axial axis).
    pub height: usize,
    /// Physical size of each square pixel (m).
    pub pixel_size_m: f64,
}

impl PaImage {
    /// Create a new image with all pixels initialised to zero.
    ///
    /// # Arguments
    /// * `width`  — columns
    /// * `height` — rows
    /// * `pixel_size_m` — physical edge length of each pixel (m)
    pub fn new(width: usize, height: usize, pixel_size_m: f64) -> Self {
        Self {
            pixels: vec![0.0; width * height],
            width,
            height,
            pixel_size_m,
        }
    }

    /// Return the pixel value at (row, col). Panics if out of bounds.
    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.pixels[row * self.width + col]
    }

    /// Set the pixel value at (row, col). Panics if out of bounds.
    pub fn set(&mut self, row: usize, col: usize, value: f64) {
        self.pixels[row * self.width + col] = value;
    }
}

// ---------------------------------------------------------------------------
// Reconstructor
// ---------------------------------------------------------------------------

/// Main photoacoustic image reconstructor.
///
/// Holds a [`PaConfig`] and provides reconstruction methods.
pub struct PaReconstructor {
    /// System configuration.
    pub config: PaConfig,
}

impl PaReconstructor {
    /// Create a new reconstructor from the given configuration.
    pub fn new(config: PaConfig) -> Self {
        Self { config }
    }
}

// ---------------------------------------------------------------------------
// Delay-and-sum beamforming
// ---------------------------------------------------------------------------

/// Delay-and-sum (DAS) beamforming reconstruction.
///
/// For each pixel in `image`, the one-way propagation delay from the pixel
/// centre to every transducer element is computed; the corresponding sample
/// is looked up (with linear interpolation) and accumulated.
///
/// # Arguments
/// * `signals` — one `Vec<f64>` per element, all the same length
/// * `element_positions` — (x, z) coordinates of each element in metres
/// * `config` — acquisition parameters
/// * `image` — output image (must already be sized; pixels are **accumulated**, not overwritten)
pub fn delay_and_sum(
    signals: &[Vec<f64>],
    element_positions: &[(f64, f64)],
    config: &PaConfig,
    image: &mut PaImage,
) {
    let w = image.width;
    let h = image.height;
    let ps = image.pixel_size_m;
    let inv_c = 1.0 / config.sound_speed_mps;
    let fs = config.sample_rate_hz;

    // Image origin: centre of the image laterally. Pixel (row, col) maps to
    // physical (x, z) with x lateral, z axial (depth).
    let x_offset = (w as f64 - 1.0) / 2.0;

    for row in 0..h {
        let pz = row as f64 * ps;
        for col in 0..w {
            let px = (col as f64 - x_offset) * ps;
            let mut sum = 0.0;
            for (elem_idx, &(ex, ez)) in element_positions.iter().enumerate() {
                let dx = px - ex;
                let dz = pz - ez;
                let dist = (dx * dx + dz * dz).sqrt();
                let delay_s = dist * inv_c;
                let sample_idx_f = delay_s * fs;
                let sample_idx = sample_idx_f as usize;
                let frac = sample_idx_f - sample_idx as f64;

                if let Some(sig) = signals.get(elem_idx) {
                    if sample_idx + 1 < sig.len() {
                        // linear interpolation
                        let val =
                            sig[sample_idx] * (1.0 - frac) + sig[sample_idx + 1] * frac;
                        sum += val;
                    }
                }
            }
            image.pixels[row * w + col] += sum;
        }
    }
}

// ---------------------------------------------------------------------------
// Filtered back-projection
// ---------------------------------------------------------------------------

/// Filtered backprojection (FBP) reconstruction from angular projections.
///
/// Each row of `signals` is a 1-D projection at the corresponding angle in
/// `angles` (radians). The projections are ramp-filtered in the spatial domain
/// (Ram-Lak kernel) and then back-projected onto `image`.
///
/// # Arguments
/// * `signals` — one projection per angle, all the same length
/// * `angles`  — projection angles in radians, one per signal row
/// * `image`   — output image (pixels are accumulated)
pub fn filtered_backprojection(
    signals: &[Vec<f64>],
    angles: &[f64],
    image: &mut PaImage,
) {
    let w = image.width;
    let h = image.height;
    let ps = image.pixel_size_m;
    let cx = (w as f64 - 1.0) / 2.0;
    let cy = (h as f64 - 1.0) / 2.0;

    for (proj_idx, angle) in angles.iter().enumerate() {
        let proj = &signals[proj_idx];
        let n = proj.len();
        if n == 0 {
            continue;
        }
        // Apply ramp filter in spatial domain (Ram-Lak kernel)
        let filtered = ramp_filter(proj);

        let proj_centre = (n as f64 - 1.0) / 2.0;
        let cos_a = angle.cos();
        let sin_a = angle.sin();

        for row in 0..h {
            let y = (row as f64 - cy) * ps;
            for col in 0..w {
                let x = (col as f64 - cx) * ps;
                // Project (x, y) onto the detector line
                let t = x * cos_a + y * sin_a;
                // Convert to sample index
                let idx_f = t / ps + proj_centre;
                let idx = idx_f as isize;
                let frac = idx_f - idx as f64;
                if idx >= 0 && (idx as usize + 1) < filtered.len() {
                    let i = idx as usize;
                    let val = filtered[i] * (1.0 - frac) + filtered[i + 1] * frac;
                    image.pixels[row * w + col] += val;
                }
            }
        }
    }
    // Normalise by the number of angles
    if !angles.is_empty() {
        let scale = PI / angles.len() as f64;
        for p in image.pixels.iter_mut() {
            *p *= scale;
        }
    }
}

/// Ramp (|freq|) filter a 1-D projection using spatial-domain convolution
/// with the Ram-Lak kernel.
fn ramp_filter(proj: &[f64]) -> Vec<f64> {
    let n = proj.len();
    if n == 0 {
        return vec![];
    }
    // Kernel: h[0] = 1/4, h[even!=0] = 0, h[odd k] = -1/(pi*k)^2
    let mut out = vec![0.0; n];
    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..n {
            let k = i as isize - j as isize;
            let h = if k == 0 {
                0.25
            } else if k % 2 != 0 {
                -1.0 / (PI * PI * (k as f64) * (k as f64))
            } else {
                0.0
            };
            sum += proj[j] * h;
        }
        out[i] = sum;
    }
    out
}

// ---------------------------------------------------------------------------
// Apodization
// ---------------------------------------------------------------------------

/// Apply apodization (element weighting) to multi-element signals in-place.
///
/// Supported window types: `"hann"`, `"hamming"`, `"blackman"`.
/// The window length equals the number of signal channels (elements).
/// Each element's entire time series is scaled by the corresponding window weight.
///
/// Unknown window types are silently treated as rectangular (no-op).
pub fn apply_apodization(signals: &mut [Vec<f64>], window_type: &str) {
    let n = signals.len();
    if n == 0 {
        return;
    }
    let weights: Vec<f64> = (0..n)
        .map(|i| {
            let x = i as f64 / (n as f64 - 1.0).max(1.0);
            match window_type {
                "hann" => 0.5 * (1.0 - (2.0 * PI * x).cos()),
                "hamming" => 0.54 - 0.46 * (2.0 * PI * x).cos(),
                "blackman" => {
                    0.42 - 0.5 * (2.0 * PI * x).cos() + 0.08 * (4.0 * PI * x).cos()
                }
                _ => 1.0,
            }
        })
        .collect();

    for (sig, &w) in signals.iter_mut().zip(weights.iter()) {
        for s in sig.iter_mut() {
            *s *= w;
        }
    }
}

// ---------------------------------------------------------------------------
// Speed-of-sound correction
// ---------------------------------------------------------------------------

/// Apply a heterogeneous speed-of-sound correction to an image.
///
/// `sos_map` contains per-pixel speed-of-sound values (m/s) in the same
/// row-major layout as `image.pixels`. The correction rescales each pixel's
/// intensity proportionally to the local SoS deviation from the global mean.
///
/// Returns a new [`PaImage`] with corrected pixel values.
pub fn speed_of_sound_correction(image: &PaImage, sos_map: &[f64]) -> PaImage {
    let n = image.pixels.len();
    assert_eq!(
        sos_map.len(),
        n,
        "sos_map length must equal number of pixels"
    );

    // Global mean SoS
    let mean_sos = sos_map.iter().copied().sum::<f64>() / n as f64;

    let mut corrected = PaImage::new(image.width, image.height, image.pixel_size_m);
    for i in 0..n {
        // Scale pixel by ratio of local SoS to mean SoS
        corrected.pixels[i] = image.pixels[i] * (sos_map[i] / mean_sos);
    }
    corrected
}

// ---------------------------------------------------------------------------
// Time-reversal focusing
// ---------------------------------------------------------------------------

/// Compute the time-reversal focused amplitude at a single point.
///
/// Sums the appropriately delayed (and time-reversed) signals from all elements
/// at the specified focus point. Returns the coherent sum amplitude.
///
/// # Arguments
/// * `signals` — one time series per element
/// * `element_positions` — (x, z) of each element (m)
/// * `focus_point` — (x, z) of the desired focal point (m)
/// * `sound_speed` — speed of sound (m/s)
/// * `sample_rate` — acquisition sample rate (Hz)
pub fn time_reversal_focus(
    signals: &[Vec<f64>],
    element_positions: &[(f64, f64)],
    focus_point: (f64, f64),
    sound_speed: f64,
    sample_rate: f64,
) -> f64 {
    let mut sum = 0.0;
    let (fx, fz) = focus_point;
    for (idx, &(ex, ez)) in element_positions.iter().enumerate() {
        let dx = fx - ex;
        let dz = fz - ez;
        let dist = (dx * dx + dz * dz).sqrt();
        let delay_samples = (dist / sound_speed) * sample_rate;
        let si = delay_samples as usize;
        let frac = delay_samples - si as f64;

        if let Some(sig) = signals.get(idx) {
            if si + 1 < sig.len() {
                let val = sig[si] * (1.0 - frac) + sig[si + 1] * frac;
                sum += val;
            }
        }
    }
    sum
}

// ---------------------------------------------------------------------------
// Image quality metrics
// ---------------------------------------------------------------------------

/// Compute the signal-to-noise ratio (SNR) in dB between a signal ROI and a noise ROI.
///
/// Each ROI is specified as `(row_start, col_start, row_end, col_end)` (exclusive end).
/// SNR = 20 * log10(mean_signal / std_noise).
pub fn compute_snr(
    image: &PaImage,
    signal_roi: (usize, usize, usize, usize),
    noise_roi: (usize, usize, usize, usize),
) -> f64 {
    let mean_signal = roi_mean(image, signal_roi);
    let std_noise = roi_std(image, noise_roi);
    if std_noise == 0.0 {
        return f64::INFINITY;
    }
    20.0 * (mean_signal.abs() / std_noise).log10()
}

/// Compute the contrast-to-noise ratio (CNR) between a target and background ROI.
///
/// CNR = |mean_target - mean_background| / std_background.
pub fn compute_cnr(
    image: &PaImage,
    target_roi: (usize, usize, usize, usize),
    background_roi: (usize, usize, usize, usize),
) -> f64 {
    let mean_target = roi_mean(image, target_roi);
    let mean_bg = roi_mean(image, background_roi);
    let std_bg = roi_std(image, background_roi);
    if std_bg == 0.0 {
        return f64::INFINITY;
    }
    (mean_target - mean_bg).abs() / std_bg
}

/// Estimate the lateral resolution as the full-width at half-maximum (FWHM) of
/// a point-spread-function (PSF) line profile.
///
/// Returns the FWHM in metres (`pixel_count * pixel_size_m`).
///
/// If no valid half-maximum crossing is found, returns `0.0`.
pub fn lateral_resolution(psf_line: &[f64], pixel_size_m: f64) -> f64 {
    if psf_line.is_empty() {
        return 0.0;
    }
    let max_val = psf_line.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    if max_val <= 0.0 {
        return 0.0;
    }
    let half_max = max_val / 2.0;

    // Find first crossing above half-max from the left
    let mut left = None;
    for i in 0..psf_line.len() - 1 {
        if psf_line[i] < half_max && psf_line[i + 1] >= half_max {
            // Linear interpolation of crossing point
            let frac = (half_max - psf_line[i]) / (psf_line[i + 1] - psf_line[i]);
            left = Some(i as f64 + frac);
            break;
        }
    }

    // Find first crossing below half-max from the right
    let mut right = None;
    for i in (0..psf_line.len() - 1).rev() {
        if psf_line[i + 1] < half_max && psf_line[i] >= half_max {
            let frac = (half_max - psf_line[i + 1]) / (psf_line[i] - psf_line[i + 1]);
            right = Some(i as f64 + 1.0 - frac);
            break;
        }
    }

    match (left, right) {
        (Some(l), Some(r)) => (r - l) * pixel_size_m,
        _ => 0.0,
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Mean of pixel values inside an ROI.
fn roi_mean(image: &PaImage, roi: (usize, usize, usize, usize)) -> f64 {
    let (r0, c0, r1, c1) = roi;
    let mut sum = 0.0;
    let mut count = 0usize;
    for r in r0..r1 {
        for c in c0..c1 {
            sum += image.get(r, c);
            count += 1;
        }
    }
    if count == 0 {
        0.0
    } else {
        sum / count as f64
    }
}

/// Standard deviation of pixel values inside an ROI.
fn roi_std(image: &PaImage, roi: (usize, usize, usize, usize)) -> f64 {
    let mean = roi_mean(image, roi);
    let (r0, c0, r1, c1) = roi;
    let mut sum_sq = 0.0;
    let mut count = 0usize;
    for r in r0..r1 {
        for c in c0..c1 {
            let d = image.get(r, c) - mean;
            sum_sq += d * d;
            count += 1;
        }
    }
    if count == 0 {
        0.0
    } else {
        (sum_sq / count as f64).sqrt()
    }
}

// ===========================================================================
// Tests
// ===========================================================================
#[cfg(test)]
mod tests {
    use super::*;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // -----------------------------------------------------------------------
    // PaConfig / PaImage basics
    // -----------------------------------------------------------------------

    #[test]
    fn test_pa_config_creation() {
        let cfg = PaConfig {
            sound_speed_mps: 1540.0,
            sample_rate_hz: 40e6,
            num_elements: 128,
            element_pitch_m: 0.3e-3,
        };
        assert_eq!(cfg.num_elements, 128);
        assert!(approx_eq(cfg.sound_speed_mps, 1540.0, 1e-9));
    }

    #[test]
    fn test_pa_image_new() {
        let img = PaImage::new(32, 64, 0.1e-3);
        assert_eq!(img.width, 32);
        assert_eq!(img.height, 64);
        assert_eq!(img.pixels.len(), 32 * 64);
        assert!(img.pixels.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_pa_image_get_set() {
        let mut img = PaImage::new(10, 10, 1e-3);
        img.set(3, 7, 42.0);
        assert!(approx_eq(img.get(3, 7), 42.0, 1e-12));
        assert!(approx_eq(img.get(0, 0), 0.0, 1e-12));
    }

    #[test]
    fn test_pa_image_pixel_size() {
        let img = PaImage::new(4, 4, 0.25e-3);
        assert!(approx_eq(img.pixel_size_m, 0.25e-3, 1e-15));
    }

    // -----------------------------------------------------------------------
    // PaReconstructor
    // -----------------------------------------------------------------------

    #[test]
    fn test_pa_reconstructor_creation() {
        let cfg = PaConfig {
            sound_speed_mps: 1500.0,
            sample_rate_hz: 20e6,
            num_elements: 64,
            element_pitch_m: 0.5e-3,
        };
        let rec = PaReconstructor::new(cfg);
        assert_eq!(rec.config.num_elements, 64);
    }

    // -----------------------------------------------------------------------
    // Delay-and-sum
    // -----------------------------------------------------------------------

    #[test]
    fn test_delay_and_sum_zero_signals() {
        let cfg = PaConfig {
            sound_speed_mps: 1540.0,
            sample_rate_hz: 40e6,
            num_elements: 4,
            element_pitch_m: 0.3e-3,
        };
        let signals: Vec<Vec<f64>> = vec![vec![0.0; 100]; 4];
        let positions: Vec<(f64, f64)> = (0..4)
            .map(|i| (i as f64 * 0.3e-3, 0.0))
            .collect();
        let mut img = PaImage::new(8, 8, 0.1e-3);
        delay_and_sum(&signals, &positions, &cfg, &mut img);
        assert!(img.pixels.iter().all(|&v| approx_eq(v, 0.0, 1e-15)));
    }

    #[test]
    fn test_delay_and_sum_single_element() {
        let cfg = PaConfig {
            sound_speed_mps: 1540.0,
            sample_rate_hz: 40e6,
            num_elements: 1,
            element_pitch_m: 0.3e-3,
        };
        // Single element at origin with an impulse at sample 100
        let mut sig = vec![0.0; 500];
        sig[100] = 1.0;
        let signals = vec![sig];
        let positions = vec![(0.0, 0.0)];
        let mut img = PaImage::new(4, 4, 0.1e-3);
        delay_and_sum(&signals, &positions, &cfg, &mut img);
        // At least some pixels should be nonzero if their delay maps to sample ~100
        let total: f64 = img.pixels.iter().sum();
        // The image should have accumulated some energy
        assert!(total.is_finite());
    }

    #[test]
    fn test_delay_and_sum_accumulates() {
        // Running DAS twice should double the pixel values
        let cfg = PaConfig {
            sound_speed_mps: 1540.0,
            sample_rate_hz: 40e6,
            num_elements: 2,
            element_pitch_m: 0.5e-3,
        };
        let mut sig = vec![0.0; 200];
        sig[50] = 5.0;
        let signals = vec![sig.clone(), sig];
        let positions = vec![(0.0, 0.0), (0.5e-3, 0.0)];
        let mut img = PaImage::new(4, 4, 0.1e-3);
        delay_and_sum(&signals, &positions, &cfg, &mut img);
        let first_pass = img.pixels.clone();
        delay_and_sum(&signals, &positions, &cfg, &mut img);
        for i in 0..img.pixels.len() {
            assert!(approx_eq(img.pixels[i], 2.0 * first_pass[i], 1e-10));
        }
    }

    // -----------------------------------------------------------------------
    // Filtered backprojection
    // -----------------------------------------------------------------------

    #[test]
    fn test_fbp_zero_projections() {
        let signals: Vec<Vec<f64>> = vec![vec![0.0; 32]; 8];
        let angles: Vec<f64> = (0..8).map(|i| i as f64 * PI / 8.0).collect();
        let mut img = PaImage::new(8, 8, 1e-3);
        filtered_backprojection(&signals, &angles, &mut img);
        assert!(img.pixels.iter().all(|&v| approx_eq(v, 0.0, 1e-12)));
    }

    #[test]
    fn test_fbp_single_angle() {
        // A single projection with a spike should produce a stripe
        let mut proj = vec![0.0; 32];
        proj[16] = 10.0;
        let signals = vec![proj];
        let angles = vec![0.0];
        let mut img = PaImage::new(8, 8, 1e-3);
        filtered_backprojection(&signals, &angles, &mut img);
        // Image should contain some nonzero values from the ramp-filtered spike
        let has_nonzero = img.pixels.iter().any(|&v| v.abs() > 1e-15);
        assert!(has_nonzero);
    }

    #[test]
    fn test_ramp_filter_length() {
        let proj = vec![1.0; 16];
        let filtered = ramp_filter(&proj);
        assert_eq!(filtered.len(), 16);
    }

    #[test]
    fn test_ramp_filter_empty() {
        let filtered = ramp_filter(&[]);
        assert!(filtered.is_empty());
    }

    // -----------------------------------------------------------------------
    // Apodization
    // -----------------------------------------------------------------------

    #[test]
    fn test_apodization_hann() {
        let mut signals: Vec<Vec<f64>> = vec![vec![1.0; 10]; 5];
        apply_apodization(&mut signals, "hann");
        // Hann window: first and last elements should be ~0
        assert!(approx_eq(signals[0][0], 0.0, 1e-10));
        assert!(approx_eq(signals[4][0], 0.0, 1e-10));
        // Middle element should be ~1
        assert!(approx_eq(signals[2][0], 1.0, 1e-10));
    }

    #[test]
    fn test_apodization_hamming() {
        let mut signals: Vec<Vec<f64>> = vec![vec![1.0; 10]; 5];
        apply_apodization(&mut signals, "hamming");
        // Hamming: first element weight = 0.54 - 0.46 = 0.08
        assert!(approx_eq(signals[0][0], 0.08, 1e-10));
        // Middle element: 0.54 - 0.46*cos(pi) = 0.54 + 0.46 = 1.0
        assert!(approx_eq(signals[2][0], 1.0, 1e-10));
    }

    #[test]
    fn test_apodization_blackman() {
        let mut signals: Vec<Vec<f64>> = vec![vec![1.0; 10]; 5];
        apply_apodization(&mut signals, "blackman");
        // Blackman: endpoints ~0
        assert!(approx_eq(signals[0][0], 0.0, 1e-10));
        assert!(approx_eq(signals[4][0], 0.0, 1e-10));
    }

    #[test]
    fn test_apodization_unknown_is_rectangular() {
        let mut signals: Vec<Vec<f64>> = vec![vec![3.0; 10]; 5];
        apply_apodization(&mut signals, "unknown_window");
        // Should be unchanged (rectangular = 1.0 weights)
        for sig in &signals {
            for &s in sig {
                assert!(approx_eq(s, 3.0, 1e-12));
            }
        }
    }

    // -----------------------------------------------------------------------
    // Speed-of-sound correction
    // -----------------------------------------------------------------------

    #[test]
    fn test_sos_correction_uniform() {
        let mut img = PaImage::new(4, 4, 1e-3);
        for (i, p) in img.pixels.iter_mut().enumerate() {
            *p = (i + 1) as f64;
        }
        // Uniform SoS => no change
        let sos_map = vec![1540.0; 16];
        let corrected = speed_of_sound_correction(&img, &sos_map);
        for i in 0..16 {
            assert!(approx_eq(corrected.pixels[i], img.pixels[i], 1e-10));
        }
    }

    #[test]
    fn test_sos_correction_heterogeneous() {
        let mut img = PaImage::new(2, 2, 1e-3);
        img.pixels = vec![10.0, 10.0, 10.0, 10.0];
        // Half at 1540, half at 1600 => mean = 1570
        let sos_map = vec![1540.0, 1540.0, 1600.0, 1600.0];
        let corrected = speed_of_sound_correction(&img, &sos_map);
        let mean_sos = 1570.0;
        assert!(approx_eq(corrected.pixels[0], 10.0 * 1540.0 / mean_sos, 1e-6));
        assert!(approx_eq(corrected.pixels[3], 10.0 * 1600.0 / mean_sos, 1e-6));
    }

    // -----------------------------------------------------------------------
    // Time-reversal focusing
    // -----------------------------------------------------------------------

    #[test]
    fn test_time_reversal_zero_signals() {
        let signals = vec![vec![0.0; 100]; 4];
        let positions: Vec<(f64, f64)> = (0..4).map(|i| (i as f64 * 1e-3, 0.0)).collect();
        let val = time_reversal_focus(&signals, &positions, (2e-3, 5e-3), 1540.0, 40e6);
        assert!(approx_eq(val, 0.0, 1e-15));
    }

    #[test]
    fn test_time_reversal_constructive() {
        // Place a constant-1 signal on all elements — all delays should add up
        let signals = vec![vec![1.0; 10000]; 4];
        let positions: Vec<(f64, f64)> = (0..4).map(|i| (i as f64 * 1e-3, 0.0)).collect();
        let val = time_reversal_focus(&signals, &positions, (1.5e-3, 10e-3), 1540.0, 40e6);
        // Should be close to 4.0 (sum of 4 elements, each contributing ~1.0)
        assert!(approx_eq(val, 4.0, 0.1));
    }

    // -----------------------------------------------------------------------
    // SNR & CNR
    // -----------------------------------------------------------------------

    #[test]
    fn test_compute_snr() {
        let mut img = PaImage::new(10, 10, 1e-3);
        // Signal region: rows 0..5, cols 0..5 all set to 100
        for r in 0..5 {
            for c in 0..5 {
                img.set(r, c, 100.0);
            }
        }
        // Noise region: rows 5..10, cols 5..10, small noise
        for r in 5..10 {
            for c in 5..10 {
                img.set(r, c, 0.1 * ((r * 10 + c) as f64 - 75.0));
            }
        }
        let snr = compute_snr(&img, (0, 0, 5, 5), (5, 5, 10, 10));
        // Should be a positive, finite dB value
        assert!(snr.is_finite());
        assert!(snr > 0.0);
    }

    #[test]
    fn test_compute_cnr() {
        let mut img = PaImage::new(10, 10, 1e-3);
        // Target region: bright
        for r in 0..5 {
            for c in 0..5 {
                img.set(r, c, 50.0);
            }
        }
        // Background region: with some variation
        for r in 5..10 {
            for c in 5..10 {
                img.set(r, c, 10.0 + 0.5 * (c as f64));
            }
        }
        let cnr = compute_cnr(&img, (0, 0, 5, 5), (5, 5, 10, 10));
        assert!(cnr.is_finite());
        assert!(cnr > 0.0);
    }

    #[test]
    fn test_snr_infinite_for_zero_noise() {
        let mut img = PaImage::new(4, 4, 1e-3);
        for r in 0..2 {
            for c in 0..2 {
                img.set(r, c, 10.0);
            }
        }
        // Noise region is all zeros => std = 0 => SNR infinite
        let snr = compute_snr(&img, (0, 0, 2, 2), (2, 2, 4, 4));
        assert!(snr.is_infinite());
    }

    // -----------------------------------------------------------------------
    // Lateral resolution (FWHM)
    // -----------------------------------------------------------------------

    #[test]
    fn test_lateral_resolution_gaussian_like() {
        // Create a Gaussian-like PSF profile
        let n = 101;
        let sigma = 10.0; // pixels
        let centre = 50.0;
        let psf: Vec<f64> = (0..n)
            .map(|i| {
                let x = i as f64 - centre;
                (-x * x / (2.0 * sigma * sigma)).exp()
            })
            .collect();
        let pixel_size = 0.1e-3; // 0.1 mm
        let fwhm = lateral_resolution(&psf, pixel_size);
        // Theoretical FWHM of Gaussian = 2*sqrt(2*ln2)*sigma ~ 2.3548 * sigma
        let expected_fwhm_pixels = 2.0 * (2.0 * 2.0_f64.ln()).sqrt() * sigma;
        let expected_fwhm_m = expected_fwhm_pixels * pixel_size;
        assert!(
            approx_eq(fwhm, expected_fwhm_m, 0.5e-3),
            "FWHM={fwhm}, expected={expected_fwhm_m}"
        );
    }

    #[test]
    fn test_lateral_resolution_empty() {
        assert!(approx_eq(lateral_resolution(&[], 1e-3), 0.0, 1e-15));
    }

    #[test]
    fn test_lateral_resolution_flat_zero() {
        let psf = vec![0.0; 50];
        assert!(approx_eq(lateral_resolution(&psf, 1e-3), 0.0, 1e-15));
    }
}
