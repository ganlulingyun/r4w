//! Synthetic Aperture Sonar (SAS) image formation for high-resolution
//! underwater acoustic imaging.
//!
//! This module provides the core algorithms needed to form focused SAS images
//! from raw ping data collected by a moving sonar platform. The processing
//! chain includes:
//!
//! - **Range compression** via matched filtering (pulse compression)
//! - **Motion compensation** with phase correction for platform motion errors
//! - **DPCA micronavigation** for sub-wavelength surge/sway estimation
//! - **Backprojection** cross-range focusing
//! - **Phase Gradient Autofocus (PGA)** for residual phase error removal
//! - **Image quality metrics**: resolution, PSLR, ISLR
//!
//! # Example
//!
//! ```
//! use r4w_core::synthetic_aperture_sonar::{SasConfig, SasProcessor};
//!
//! let config = SasConfig {
//!     sound_speed_mps: 1500.0,
//!     center_freq_hz: 100_000.0,
//!     bandwidth_hz: 20_000.0,
//!     ping_rate_hz: 10.0,
//!     platform_speed_mps: 2.0,
//!     sample_rate_hz: 60_000.0,
//! };
//!
//! let proc = SasProcessor::new(config);
//! assert!((proc.range_resolution() - 0.0375).abs() < 1e-6);
//! ```

use std::f64::consts::PI;

// ───────────────────────────────────────────────────────────────────────────
// Complex arithmetic helpers (using (f64, f64) tuples)
// ───────────────────────────────────────────────────────────────────────────

/// Multiply two complex numbers represented as `(re, im)` tuples.
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Conjugate of a complex number.
#[inline]
fn conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Magnitude squared.
#[inline]
fn mag_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Magnitude.
#[inline]
fn mag(a: (f64, f64)) -> f64 {
    mag_sq(a).sqrt()
}

/// Add two complex numbers.
#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}


/// Complex exponential: e^{j*theta}.
#[inline]
fn cexp(theta: f64) -> (f64, f64) {
    (theta.cos(), theta.sin())
}

/// Argument (phase angle) of a complex number.
#[inline]
fn carg(a: (f64, f64)) -> f64 {
    a.1.atan2(a.0)
}

// ───────────────────────────────────────────────────────────────────────────
// FFT (radix-2 Cooley-Tukey, in-place)
// ───────────────────────────────────────────────────────────────────────────

/// In-place radix-2 FFT. `inverse` = true for IFFT.
/// `data` length must be a power of two.
fn fft_inplace(data: &mut [(f64, f64)], inverse: bool) {
    let n = data.len();
    if n <= 1 {
        return;
    }
    assert!(n.is_power_of_two(), "FFT length must be a power of two");

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            data.swap(i, j);
        }
    }

    // Cooley-Tukey butterfly
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle_step = sign * 2.0 * PI / len as f64;
        let wn = cexp(angle_step);
        let mut start = 0;
        while start < n {
            let mut w: (f64, f64) = (1.0, 0.0);
            for k in 0..half {
                let u = data[start + k];
                let t = cmul(w, data[start + k + half]);
                data[start + k] = cadd(u, t);
                data[start + k + half] = (u.0 - t.0, u.1 - t.1);
                w = cmul(w, wn);
            }
            start += len;
        }
        len <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for sample in data.iter_mut() {
            sample.0 *= inv_n;
            sample.1 *= inv_n;
        }
    }
}

/// Forward FFT (returns new vector). Input is zero-padded to the next power
/// of two if necessary.
fn fft(data: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = data.len().next_power_of_two();
    let mut buf = vec![(0.0, 0.0); n];
    buf[..data.len()].copy_from_slice(data);
    fft_inplace(&mut buf, false);
    buf
}

/// Inverse FFT (returns new vector).
fn ifft(data: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = data.len().next_power_of_two();
    let mut buf = vec![(0.0, 0.0); n];
    buf[..data.len()].copy_from_slice(data);
    fft_inplace(&mut buf, true);
    buf
}

// ───────────────────────────────────────────────────────────────────────────
// Configuration
// ───────────────────────────────────────────────────────────────────────────

/// Configuration parameters for the SAS processor.
#[derive(Debug, Clone)]
pub struct SasConfig {
    /// Speed of sound in water (m/s). Typical ocean: 1500 m/s.
    pub sound_speed_mps: f64,
    /// Centre frequency of the sonar transmit waveform (Hz).
    pub center_freq_hz: f64,
    /// Bandwidth of the transmit chirp (Hz).
    pub bandwidth_hz: f64,
    /// Ping repetition rate (pings per second).
    pub ping_rate_hz: f64,
    /// Along-track platform speed (m/s).
    pub platform_speed_mps: f64,
    /// Receiver sample rate (Hz).
    pub sample_rate_hz: f64,
}

impl Default for SasConfig {
    fn default() -> Self {
        Self {
            sound_speed_mps: 1500.0,
            center_freq_hz: 100_000.0,
            bandwidth_hz: 20_000.0,
            ping_rate_hz: 10.0,
            platform_speed_mps: 2.0,
            sample_rate_hz: 60_000.0,
        }
    }
}

// ───────────────────────────────────────────────────────────────────────────
// SAS image
// ───────────────────────────────────────────────────────────────────────────

/// A focused 2D SAS image.
#[derive(Debug, Clone)]
pub struct SasImage {
    /// Image pixel magnitudes stored in row-major order (cross_range x range).
    pub pixels: Vec<f64>,
    /// Number of pixels in the cross-range (along-track) dimension.
    pub cross_range_pixels: usize,
    /// Number of pixels in the range dimension.
    pub range_pixels: usize,
    /// Cross-range extent of the image (m).
    pub cross_range_extent: f64,
    /// Range extent of the image (m).
    pub range_extent: f64,
}

impl SasImage {
    /// Create a new image initialised to zeros.
    pub fn new(
        cross_range_pixels: usize,
        range_pixels: usize,
        cross_range_extent: f64,
        range_extent: f64,
    ) -> Self {
        Self {
            pixels: vec![0.0; cross_range_pixels * range_pixels],
            cross_range_pixels,
            range_pixels,
            cross_range_extent,
            range_extent,
        }
    }

    /// Access pixel at `(cross_range_idx, range_idx)`.
    pub fn pixel(&self, cross_range_idx: usize, range_idx: usize) -> f64 {
        self.pixels[cross_range_idx * self.range_pixels + range_idx]
    }

    /// Mutable access to pixel at `(cross_range_idx, range_idx)`.
    pub fn pixel_mut(&mut self, cross_range_idx: usize, range_idx: usize) -> &mut f64 {
        &mut self.pixels[cross_range_idx * self.range_pixels + range_idx]
    }

    /// Peak pixel value.
    pub fn peak(&self) -> f64 {
        self.pixels
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// Return a normalised (0..1) copy of the image.
    pub fn normalised(&self) -> SasImage {
        let peak = self.peak();
        let pixels = if peak > 0.0 {
            self.pixels.iter().map(|&v| v / peak).collect()
        } else {
            self.pixels.clone()
        };
        SasImage {
            pixels,
            cross_range_pixels: self.cross_range_pixels,
            range_pixels: self.range_pixels,
            cross_range_extent: self.cross_range_extent,
            range_extent: self.range_extent,
        }
    }
}

// ───────────────────────────────────────────────────────────────────────────
// SAS Processor
// ───────────────────────────────────────────────────────────────────────────

/// Main synthetic aperture sonar processor.
///
/// Provides range compression, motion compensation, DPCA micro-navigation,
/// back-projection image formation, and auto-focus capabilities.
pub struct SasProcessor {
    /// Configuration parameters.
    config: SasConfig,
}

impl SasProcessor {
    /// Create a new SAS processor with the given configuration.
    pub fn new(config: SasConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &SasConfig {
        &self.config
    }

    /// Theoretical range resolution: `c / (2 * B)`.
    pub fn range_resolution(&self) -> f64 {
        self.config.sound_speed_mps / (2.0 * self.config.bandwidth_hz)
    }

    /// Wavelength at the centre frequency: `c / f_c`.
    pub fn wavelength(&self) -> f64 {
        self.config.sound_speed_mps / self.config.center_freq_hz
    }

    /// Theoretical cross-range resolution for a fully synthesised aperture
    /// of length `aperture_m`: `lambda * R / (2 * L)` where R is slant range
    /// and L is the synthetic aperture length.
    pub fn cross_range_resolution(&self, slant_range_m: f64, aperture_m: f64) -> f64 {
        self.wavelength() * slant_range_m / (2.0 * aperture_m)
    }

    /// Maximum unambiguous range: `c / (2 * ping_rate)`.
    pub fn max_unambiguous_range(&self) -> f64 {
        self.config.sound_speed_mps / (2.0 * self.config.ping_rate_hz)
    }

    /// Ping spacing along track: `v / ping_rate`.
    pub fn ping_spacing(&self) -> f64 {
        self.config.platform_speed_mps / self.config.ping_rate_hz
    }

    /// Generate a linear frequency modulated (LFM) chirp replica for matched
    /// filtering. Returns `n_samples` complex samples.
    pub fn generate_chirp_replica(&self, n_samples: usize) -> Vec<(f64, f64)> {
        let duration = n_samples as f64 / self.config.sample_rate_hz;
        let chirp_rate = self.config.bandwidth_hz / duration;
        (0..n_samples)
            .map(|i| {
                let t = i as f64 / self.config.sample_rate_hz;
                let phase = 2.0
                    * PI
                    * (self.config.center_freq_hz * t + 0.5 * chirp_rate * t * t);
                cexp(phase)
            })
            .collect()
    }
}

// ───────────────────────────────────────────────────────────────────────────
// Public free functions
// ───────────────────────────────────────────────────────────────────────────

/// Range-compress a single ping via matched filtering with the given replica.
///
/// Performs frequency-domain conjugate multiplication followed by inverse FFT
/// (i.e. circular cross-correlation). The output length equals the
/// zero-padded FFT length (next power of two of `ping.len() + replica.len() - 1`).
pub fn range_compress(ping: &[(f64, f64)], replica: &[(f64, f64)]) -> Vec<(f64, f64)> {
    if ping.is_empty() || replica.is_empty() {
        return vec![];
    }
    let out_len = (ping.len() + replica.len() - 1).next_power_of_two();
    let mut ping_buf = vec![(0.0, 0.0); out_len];
    let mut rep_buf = vec![(0.0, 0.0); out_len];
    ping_buf[..ping.len()].copy_from_slice(ping);
    rep_buf[..replica.len()].copy_from_slice(replica);

    fft_inplace(&mut ping_buf, false);
    fft_inplace(&mut rep_buf, false);

    // Multiply ping spectrum by conjugate of replica spectrum
    for i in 0..out_len {
        ping_buf[i] = cmul(ping_buf[i], conj(rep_buf[i]));
    }
    fft_inplace(&mut ping_buf, true);
    ping_buf
}

/// Apply motion compensation to a set of pings given measured platform
/// motion offsets.
///
/// Each offset is `(dx, dy, dz)` in metres representing the deviation of
/// the platform from its nominal straight-line track for the corresponding
/// ping. The correction applies a phase shift to each sample based on the
/// two-way path length change.
///
/// `config` provides the sound speed and centre frequency needed to convert
/// metric offsets into phase corrections.
pub fn motion_compensate(
    pings: &[Vec<(f64, f64)>],
    offsets: &[(f64, f64, f64)],
    config: &SasConfig,
) -> Vec<Vec<(f64, f64)>> {
    assert_eq!(
        pings.len(),
        offsets.len(),
        "Number of pings must equal number of offsets"
    );
    let wavelength = config.sound_speed_mps / config.center_freq_hz;
    pings
        .iter()
        .zip(offsets.iter())
        .map(|(ping, &(dx, dy, dz))| {
            // Total displacement magnitude
            let displacement = (dx * dx + dy * dy + dz * dz).sqrt();
            // Two-way phase error: 2 * displacement / wavelength * 2pi
            let phase_correction = -4.0 * PI * displacement / wavelength;
            let correction = cexp(phase_correction);
            ping.iter().map(|&s| cmul(s, correction)).collect()
        })
        .collect()
}

/// Displaced Phase Center Antenna (DPCA) micro-navigation.
///
/// Estimates the along-track (surge) and across-track (sway) displacement
/// between two consecutive pings using the overlap region of fore/aft
/// receiver arrays. `overlap` is the fractional overlap (0.0 to 1.0)
/// between the two pings' spatial coverage.
///
/// Returns `(surge, sway)` in metres.
pub fn dpca_micronavigation(
    ping_a: &[(f64, f64)],
    ping_b: &[(f64, f64)],
    overlap: f64,
) -> (f64, f64) {
    if ping_a.is_empty() || ping_b.is_empty() || overlap <= 0.0 {
        return (0.0, 0.0);
    }
    let overlap = overlap.clamp(0.0, 1.0);
    let n_overlap = ((ping_a.len().min(ping_b.len()) as f64) * overlap).round() as usize;
    let n_overlap = n_overlap.max(1);

    // Cross-correlate the overlap regions to find the peak shift
    let a_start = ping_a.len().saturating_sub(n_overlap);
    let overlap_a = &ping_a[a_start..];
    let overlap_b = &ping_b[..n_overlap.min(ping_b.len())];

    let common = overlap_a.len().min(overlap_b.len());
    if common == 0 {
        return (0.0, 0.0);
    }

    // Compute cross-correlation at zero lag and neighbouring lags
    // to sub-sample interpolation
    let max_lag = (common / 4).max(1).min(common - 1);
    let mut best_lag: isize = 0;
    let mut best_mag = 0.0f64;

    for lag in -(max_lag as isize)..=(max_lag as isize) {
        let mut sum = (0.0, 0.0);
        let mut count = 0usize;
        for i in 0..common {
            let j = i as isize + lag;
            if j >= 0 && (j as usize) < common {
                sum = cadd(sum, cmul(overlap_a[i], conj(overlap_b[j as usize])));
                count += 1;
            }
        }
        if count > 0 {
            let m = mag(sum);
            if m > best_mag {
                best_mag = m;
                best_lag = lag;
            }
        }
    }

    // Phase at best lag gives the sub-sample sway estimate
    let mut sum_at_best = (0.0, 0.0);
    for i in 0..common {
        let j = i as isize + best_lag;
        if j >= 0 && (j as usize) < common {
            sum_at_best = cadd(
                sum_at_best,
                cmul(overlap_a[i], conj(overlap_b[j as usize])),
            );
        }
    }

    let phase = carg(sum_at_best);
    // Surge is estimated from the lag offset (in sample spacings)
    let surge = best_lag as f64;
    // Sway is proportional to the residual phase
    let sway = phase;

    (surge, sway)
}

/// Back-projection image formation.
///
/// Focuses the range-compressed ping data onto a 2D image grid. Each pixel
/// is formed by coherently summing contributions from every ping, applying
/// the appropriate range delay and phase correction.
///
/// `config` provides the acoustic parameters. `image_size` is
/// `(cross_range_pixels, range_pixels)`.
pub fn backproject(
    pings: &[Vec<(f64, f64)>],
    image_size: (usize, usize),
    config: &SasConfig,
) -> SasImage {
    let (n_cr, n_rg) = image_size;
    if pings.is_empty() || n_cr == 0 || n_rg == 0 {
        return SasImage::new(n_cr, n_rg, 0.0, 0.0);
    }

    let n_pings = pings.len();
    let ping_spacing = config.platform_speed_mps / config.ping_rate_hz;
    let total_aperture = (n_pings as f64 - 1.0) * ping_spacing;
    let n_range_samples = pings.iter().map(|p| p.len()).max().unwrap_or(0);
    let range_extent = n_range_samples as f64 * config.sound_speed_mps
        / (2.0 * config.sample_rate_hz);
    let cross_range_extent = total_aperture;

    let mut image = SasImage::new(n_cr, n_rg, cross_range_extent, range_extent);
    let wavelength = config.sound_speed_mps / config.center_freq_hz;

    for cr_idx in 0..n_cr {
        let y_pixel = if n_cr > 1 {
            (cr_idx as f64 / (n_cr - 1) as f64 - 0.5) * cross_range_extent
        } else {
            0.0
        };

        for rg_idx in 0..n_rg {
            let r_pixel = if n_rg > 1 {
                (rg_idx as f64 / (n_rg - 1) as f64) * range_extent
            } else {
                0.0
            };

            let mut accum = (0.0f64, 0.0f64);

            for ping_idx in 0..n_pings {
                let y_platform = (ping_idx as f64 - (n_pings - 1) as f64 / 2.0) * ping_spacing;
                let dy = y_pixel - y_platform;
                let slant_range = (r_pixel * r_pixel + dy * dy).sqrt();

                // Convert slant range to sample index (two-way travel)
                let sample_idx_f =
                    slant_range * 2.0 * config.sample_rate_hz / config.sound_speed_mps;
                let sample_idx = sample_idx_f.round() as usize;

                if sample_idx < pings[ping_idx].len() {
                    // Phase correction for range difference
                    let phase = -4.0 * PI * slant_range / wavelength;
                    let correction = cexp(phase);
                    accum = cadd(accum, cmul(pings[ping_idx][sample_idx], correction));
                }
            }

            *image.pixel_mut(cr_idx, rg_idx) = mag(accum);
        }
    }

    image
}

/// Phase Gradient Autofocus (PGA).
///
/// Iteratively estimates and removes residual phase errors from a SAS image.
/// Works on the complex image data (here approximated from magnitudes by
/// assuming zero initial phase, then iterating).
///
/// `iterations` controls the number of PGA passes.
pub fn autofocus_pga(image: &SasImage, iterations: usize) -> SasImage {
    if image.pixels.is_empty() || iterations == 0 {
        return image.clone();
    }

    let n_cr = image.cross_range_pixels;
    let n_rg = image.range_pixels;

    // Build a complex image from magnitudes (initial phase = 0)
    let mut complex_image: Vec<(f64, f64)> = image.pixels.iter().map(|&v| (v, 0.0)).collect();

    for _iter in 0..iterations {
        // For each range bin, estimate the phase error across cross-range
        let mut phase_errors = vec![0.0f64; n_cr];

        for rg in 0..n_rg {
            // Extract the cross-range profile for this range bin
            let profile: Vec<(f64, f64)> = (0..n_cr)
                .map(|cr| complex_image[cr * n_rg + rg])
                .collect();

            // Window and shift to centre the dominant scatterer
            let max_idx = profile
                .iter()
                .enumerate()
                .max_by(|(_, a), (_, b)| mag_sq(**a).partial_cmp(&mag_sq(**b)).unwrap())
                .map(|(i, _)| i)
                .unwrap_or(0);

            // Circular shift so peak is at centre
            let shift = (n_cr / 2) as isize - max_idx as isize;
            let mut shifted = vec![(0.0, 0.0); n_cr];
            for i in 0..n_cr {
                let src = ((i as isize - shift).rem_euclid(n_cr as isize)) as usize;
                shifted[i] = profile[src];
            }

            // Phase gradient: d/dx arg(profile) estimated from
            // consecutive sample conjugate products
            for i in 1..n_cr {
                let prod = cmul(shifted[i], conj(shifted[i - 1]));
                let weight = mag_sq(shifted[i]);
                phase_errors[i] += carg(prod) * weight;
            }
        }

        // Integrate phase gradient to get phase error estimate
        let mut integrated_phase = vec![0.0f64; n_cr];
        for i in 1..n_cr {
            // Normalise by number of range bins
            let avg_gradient = phase_errors[i] / (n_rg as f64).max(1.0);
            integrated_phase[i] = integrated_phase[i - 1] + avg_gradient;
        }

        // Remove mean phase
        let mean_phase: f64 = integrated_phase.iter().sum::<f64>() / n_cr as f64;
        for p in integrated_phase.iter_mut() {
            *p -= mean_phase;
        }

        // Apply phase correction
        for cr in 0..n_cr {
            let correction = cexp(-integrated_phase[cr]);
            for rg in 0..n_rg {
                let idx = cr * n_rg + rg;
                complex_image[idx] = cmul(complex_image[idx], correction);
            }
        }
    }

    // Extract magnitudes
    let pixels: Vec<f64> = complex_image.iter().map(|&c| mag(c)).collect();

    SasImage {
        pixels,
        cross_range_pixels: n_cr,
        range_pixels: n_rg,
        cross_range_extent: image.cross_range_extent,
        range_extent: image.range_extent,
    }
}

/// Compute the -3 dB resolution from a point spread function (PSF).
///
/// The PSF is expected to be a 1D slice of the image through the peak of a
/// point target. Returns the width (in bins) at which the PSF drops 3 dB
/// below its peak.
pub fn compute_resolution(psf: &[f64]) -> f64 {
    if psf.is_empty() {
        return 0.0;
    }
    let peak = psf.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if peak <= 0.0 {
        return 0.0;
    }
    let threshold = peak / 2.0_f64.sqrt(); // -3 dB in linear (voltage) scale

    // Find the peak index
    let peak_idx = psf
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(i, _)| i)
        .unwrap_or(0);

    // Search left from peak
    let mut left = peak_idx as f64;
    for i in (0..peak_idx).rev() {
        if psf[i] < threshold {
            // Linear interpolation
            let frac = (threshold - psf[i]) / (psf[i + 1] - psf[i]);
            left = i as f64 + frac;
            break;
        }
    }

    // Search right from peak
    let mut right = peak_idx as f64;
    for i in (peak_idx + 1)..psf.len() {
        if psf[i] < threshold {
            let frac = (threshold - psf[i]) / (psf[i - 1] - psf[i]);
            right = i as f64 - frac;
            break;
        }
    }

    right - left
}

/// Peak Sidelobe Ratio (PSLR) in dB.
///
/// PSLR = 20 * log10(peak_sidelobe / mainlobe_peak).
/// The mainlobe is identified as the region around the peak that stays
/// above -3 dB.
pub fn peak_sidelobe_ratio(psf: &[f64]) -> f64 {
    if psf.len() < 3 {
        return f64::NEG_INFINITY;
    }
    let peak = psf.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if peak <= 0.0 {
        return f64::NEG_INFINITY;
    }
    let threshold = peak / 2.0_f64.sqrt(); // -3 dB

    let peak_idx = psf
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(i, _)| i)
        .unwrap_or(0);

    // Find mainlobe extent
    let mut ml_left = 0;
    for i in (0..peak_idx).rev() {
        if psf[i] < threshold {
            ml_left = i + 1;
            break;
        }
    }

    let mut ml_right = psf.len() - 1;
    for i in (peak_idx + 1)..psf.len() {
        if psf[i] < threshold {
            ml_right = i - 1;
            break;
        }
    }

    // Find peak sidelobe outside mainlobe
    let mut sidelobe_peak = 0.0f64;
    for (i, &v) in psf.iter().enumerate() {
        if i < ml_left || i > ml_right {
            sidelobe_peak = sidelobe_peak.max(v);
        }
    }

    if sidelobe_peak <= 0.0 {
        return f64::NEG_INFINITY;
    }

    20.0 * (sidelobe_peak / peak).log10()
}

/// Integrated Sidelobe Ratio (ISLR) in dB.
///
/// ISLR = 10 * log10(sidelobe_energy / mainlobe_energy).
/// Energy is computed as the sum of squared magnitudes.
pub fn integrated_sidelobe_ratio(psf: &[f64]) -> f64 {
    if psf.len() < 3 {
        return f64::NEG_INFINITY;
    }
    let peak = psf.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    if peak <= 0.0 {
        return f64::NEG_INFINITY;
    }
    let threshold = peak / 2.0_f64.sqrt();

    let peak_idx = psf
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .map(|(i, _)| i)
        .unwrap_or(0);

    let mut ml_left = 0;
    for i in (0..peak_idx).rev() {
        if psf[i] < threshold {
            ml_left = i + 1;
            break;
        }
    }

    let mut ml_right = psf.len() - 1;
    for i in (peak_idx + 1)..psf.len() {
        if psf[i] < threshold {
            ml_right = i - 1;
            break;
        }
    }

    let mut mainlobe_energy = 0.0f64;
    let mut sidelobe_energy = 0.0f64;
    for (i, &v) in psf.iter().enumerate() {
        let e = v * v;
        if i >= ml_left && i <= ml_right {
            mainlobe_energy += e;
        } else {
            sidelobe_energy += e;
        }
    }

    if mainlobe_energy <= 0.0 {
        return f64::NEG_INFINITY;
    }

    10.0 * (sidelobe_energy / mainlobe_energy).log10()
}

// ───────────────────────────────────────────────────────────────────────────
// Tests
// ───────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> SasConfig {
        SasConfig::default()
    }

    fn default_processor() -> SasProcessor {
        SasProcessor::new(default_config())
    }

    // -- SasConfig / SasProcessor basic tests --------------------------------

    #[test]
    fn test_range_resolution() {
        let proc = default_processor();
        // c / (2 * B) = 1500 / (2 * 20000) = 0.0375
        let expected = 1500.0 / (2.0 * 20_000.0);
        assert!((proc.range_resolution() - expected).abs() < 1e-10);
    }

    #[test]
    fn test_wavelength() {
        let proc = default_processor();
        // lambda = c / f = 1500 / 100_000 = 0.015
        let expected = 1500.0 / 100_000.0;
        assert!((proc.wavelength() - expected).abs() < 1e-10);
    }

    #[test]
    fn test_cross_range_resolution() {
        let proc = default_processor();
        // lambda * R / (2 * L)
        let lambda = proc.wavelength();
        let r = 50.0;
        let l = 10.0;
        let expected = lambda * r / (2.0 * l);
        assert!((proc.cross_range_resolution(r, l) - expected).abs() < 1e-10);
    }

    #[test]
    fn test_max_unambiguous_range() {
        let proc = default_processor();
        // c / (2 * PRF) = 1500 / (2 * 10) = 75
        assert!((proc.max_unambiguous_range() - 75.0).abs() < 1e-10);
    }

    #[test]
    fn test_ping_spacing() {
        let proc = default_processor();
        // v / PRF = 2 / 10 = 0.2
        assert!((proc.ping_spacing() - 0.2).abs() < 1e-10);
    }

    #[test]
    fn test_config_accessor() {
        let proc = default_processor();
        assert!((proc.config().sound_speed_mps - 1500.0).abs() < 1e-10);
    }

    #[test]
    fn test_generate_chirp_replica() {
        let proc = default_processor();
        let replica = proc.generate_chirp_replica(128);
        assert_eq!(replica.len(), 128);
        // Each sample should be unit magnitude (complex exponential)
        for &s in &replica {
            let m = mag(s);
            assert!((m - 1.0).abs() < 1e-10, "Chirp sample magnitude was {}", m);
        }
    }

    // -- Range compression tests --------------------------------------------

    #[test]
    fn test_range_compress_empty_inputs() {
        assert!(range_compress(&[], &[]).is_empty());
        assert!(range_compress(&[(1.0, 0.0)], &[]).is_empty());
        assert!(range_compress(&[], &[(1.0, 0.0)]).is_empty());
    }

    #[test]
    fn test_range_compress_identity() {
        // Cross-correlating a signal with itself should give a peak at lag 0
        let n = 64;
        let proc = default_processor();
        let signal = proc.generate_chirp_replica(n);
        let result = range_compress(&signal, &signal);

        // Find peak
        let peak_idx = result
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| mag_sq(**a).partial_cmp(&mag_sq(**b)).unwrap())
            .map(|(i, _)| i)
            .unwrap();

        // Peak should be at index 0 (autocorrelation peak for circular correlation)
        assert_eq!(peak_idx, 0);
    }

    #[test]
    fn test_range_compress_chirp_concentration() {
        // Matched filtering should concentrate energy
        let n = 64;
        let proc = default_processor();
        let chirp = proc.generate_chirp_replica(n);
        let result = range_compress(&chirp, &chirp);

        // Peak magnitude should be larger than average
        let peak_mag = result.iter().map(|s| mag(*s)).fold(0.0f64, f64::max);
        let avg_mag = result.iter().map(|s| mag(*s)).sum::<f64>() / result.len() as f64;
        assert!(
            peak_mag > avg_mag * 2.0,
            "peak={}, avg={}",
            peak_mag,
            avg_mag
        );
    }

    // -- Motion compensation tests ------------------------------------------

    #[test]
    fn test_motion_compensate_zero_offset() {
        let config = default_config();
        let pings = vec![vec![(1.0, 0.0); 16]; 4];
        let offsets = vec![(0.0, 0.0, 0.0); 4];
        let result = motion_compensate(&pings, &offsets, &config);
        assert_eq!(result.len(), 4);
        // Zero offsets should leave data unchanged
        for (orig, comp) in pings.iter().zip(result.iter()) {
            for (a, b) in orig.iter().zip(comp.iter()) {
                assert!((a.0 - b.0).abs() < 1e-10);
                assert!((a.1 - b.1).abs() < 1e-10);
            }
        }
    }

    #[test]
    fn test_motion_compensate_preserves_magnitude() {
        let config = default_config();
        let pings = vec![vec![(1.0, 0.5); 16]; 3];
        let offsets = vec![(0.01, 0.02, 0.0), (0.0, 0.0, 0.03), (-0.01, 0.0, 0.0)];
        let result = motion_compensate(&pings, &offsets, &config);
        // Motion compensation is a phase-only correction, magnitudes preserved
        for (orig, comp) in pings.iter().zip(result.iter()) {
            for (a, b) in orig.iter().zip(comp.iter()) {
                let ma = mag(*a);
                let mb = mag(*b);
                assert!(
                    (ma - mb).abs() < 1e-10,
                    "Magnitude changed: {} -> {}",
                    ma,
                    mb
                );
            }
        }
    }

    #[test]
    fn test_motion_compensate_applies_phase() {
        let config = default_config();
        let pings = vec![vec![(1.0, 0.0); 4]];
        let displacement = 0.005; // 5mm
        let offsets = vec![(displacement, 0.0, 0.0)];
        let result = motion_compensate(&pings, &offsets, &config);
        // Expected phase correction: -4pi * d / lambda
        let wavelength = config.sound_speed_mps / config.center_freq_hz;
        let expected_phase = -4.0 * PI * displacement / wavelength;
        let actual_phase = carg(result[0][0]);
        // Compare modulo 2*PI (phases wrap)
        let mut diff = (actual_phase - expected_phase) % (2.0 * PI);
        if diff > PI {
            diff -= 2.0 * PI;
        } else if diff < -PI {
            diff += 2.0 * PI;
        }
        assert!(
            diff.abs() < 1e-6,
            "Phase: expected {}, got {} (diff={})",
            expected_phase,
            actual_phase,
            diff
        );
    }

    // -- DPCA micronavigation tests -----------------------------------------

    #[test]
    fn test_dpca_empty_inputs() {
        assert_eq!(dpca_micronavigation(&[], &[(1.0, 0.0)], 0.5), (0.0, 0.0));
        assert_eq!(dpca_micronavigation(&[(1.0, 0.0)], &[], 0.5), (0.0, 0.0));
    }

    #[test]
    fn test_dpca_zero_overlap() {
        let a = vec![(1.0, 0.0); 32];
        let b = vec![(1.0, 0.0); 32];
        assert_eq!(dpca_micronavigation(&a, &b, 0.0), (0.0, 0.0));
    }

    #[test]
    fn test_dpca_identical_pings() {
        // Identical pings should give zero displacement
        let ping = vec![(1.0, 0.0); 64];
        let (surge, sway) = dpca_micronavigation(&ping, &ping, 0.5);
        assert!(
            surge.abs() < 1e-6,
            "Expected zero surge, got {}",
            surge
        );
        // Sway should also be near zero for identical signals
        assert!(
            sway.abs() < 1e-6,
            "Expected zero sway, got {}",
            sway
        );
    }

    // -- Backprojection tests -----------------------------------------------

    #[test]
    fn test_backproject_empty() {
        let config = default_config();
        let image = backproject(&[], (8, 8), &config);
        assert_eq!(image.cross_range_pixels, 8);
        assert_eq!(image.range_pixels, 8);
    }

    #[test]
    fn test_backproject_dimensions() {
        let config = default_config();
        let pings = vec![vec![(1.0, 0.0); 32]; 8];
        let image = backproject(&pings, (16, 32), &config);
        assert_eq!(image.cross_range_pixels, 16);
        assert_eq!(image.range_pixels, 32);
        assert_eq!(image.pixels.len(), 16 * 32);
    }

    #[test]
    fn test_backproject_nonzero_output() {
        let config = default_config();
        let pings = vec![vec![(1.0, 0.0); 64]; 8];
        let image = backproject(&pings, (8, 8), &config);
        // With constant input, image should have nonzero content
        let total: f64 = image.pixels.iter().sum();
        assert!(total > 0.0, "Backprojected image is all zeros");
    }

    // -- Autofocus PGA tests ------------------------------------------------

    #[test]
    fn test_autofocus_pga_zero_iterations() {
        let image = SasImage::new(4, 4, 1.0, 1.0);
        let result = autofocus_pga(&image, 0);
        assert_eq!(result.pixels, image.pixels);
    }

    #[test]
    fn test_autofocus_pga_preserves_dimensions() {
        let mut image = SasImage::new(8, 16, 2.0, 4.0);
        // Put some data in the image
        for (i, p) in image.pixels.iter_mut().enumerate() {
            *p = (i as f64 * 0.1).sin().abs();
        }
        let result = autofocus_pga(&image, 3);
        assert_eq!(result.cross_range_pixels, 8);
        assert_eq!(result.range_pixels, 16);
        assert_eq!(result.pixels.len(), 8 * 16);
    }

    #[test]
    fn test_autofocus_pga_empty_image() {
        let image = SasImage::new(0, 0, 0.0, 0.0);
        let result = autofocus_pga(&image, 5);
        assert!(result.pixels.is_empty());
    }

    // -- SasImage tests -----------------------------------------------------

    #[test]
    fn test_sas_image_pixel_access() {
        let mut image = SasImage::new(4, 8, 1.0, 2.0);
        *image.pixel_mut(2, 3) = 42.0;
        assert!((image.pixel(2, 3) - 42.0).abs() < 1e-10);
    }

    #[test]
    fn test_sas_image_peak() {
        let mut image = SasImage::new(4, 4, 1.0, 1.0);
        *image.pixel_mut(1, 2) = 10.0;
        *image.pixel_mut(3, 0) = 5.0;
        assert!((image.peak() - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_sas_image_normalised() {
        let mut image = SasImage::new(2, 2, 1.0, 1.0);
        *image.pixel_mut(0, 0) = 4.0;
        *image.pixel_mut(0, 1) = 2.0;
        *image.pixel_mut(1, 0) = 1.0;
        *image.pixel_mut(1, 1) = 0.0;
        let norm = image.normalised();
        assert!((norm.pixel(0, 0) - 1.0).abs() < 1e-10);
        assert!((norm.pixel(0, 1) - 0.5).abs() < 1e-10);
    }

    // -- Resolution / PSLR / ISLR tests ------------------------------------

    #[test]
    fn test_compute_resolution_sinc() {
        // Construct an approximate sinc-like PSF with a known -3dB width
        let n = 256;
        let mut psf = vec![0.0; n];
        let center = n / 2;
        for i in 0..n {
            let x = (i as f64 - center as f64) * 0.1;
            psf[i] = if x.abs() < 1e-12 {
                1.0
            } else {
                (PI * x).sin() / (PI * x)
            }
            .abs();
        }
        let res = compute_resolution(&psf);
        // Sinc -3dB width is ~0.886 / (bin_spacing) -> roughly 8.86 bins at 0.1 spacing
        assert!(res > 5.0, "Resolution too narrow: {}", res);
        assert!(res < 15.0, "Resolution too wide: {}", res);
    }

    #[test]
    fn test_compute_resolution_empty() {
        assert!((compute_resolution(&[]) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_peak_sidelobe_ratio_short_psf() {
        // Too short to have sidelobes
        assert!(peak_sidelobe_ratio(&[1.0]).is_infinite());
    }

    #[test]
    fn test_peak_sidelobe_ratio_known() {
        // PSF: a peak at centre, sidelobes at known level
        let n = 128;
        let center = n / 2;
        let mut psf = vec![0.0; n];
        // Mainlobe: 3 bins above threshold
        psf[center - 1] = 0.8;
        psf[center] = 1.0;
        psf[center + 1] = 0.8;
        // Sidelobes
        psf[center + 10] = 0.1;
        psf[center - 10] = 0.1;
        let pslr = peak_sidelobe_ratio(&psf);
        // PSLR = 20*log10(0.1/1.0) = -20 dB
        assert!(
            (pslr - (-20.0)).abs() < 1.0,
            "PSLR expected ~-20 dB, got {} dB",
            pslr
        );
    }

    #[test]
    fn test_integrated_sidelobe_ratio_no_sidelobes() {
        // All energy in mainlobe
        let psf = vec![0.0, 0.8, 1.0, 0.8, 0.0];
        let islr = integrated_sidelobe_ratio(&psf);
        // Sidelobe energy is zero or negligible
        assert!(islr < -20.0, "ISLR should be very low, got {} dB", islr);
    }

    #[test]
    fn test_integrated_sidelobe_ratio_known() {
        // Construct PSF with known energy distribution
        let n = 128;
        let center = n / 2;
        let mut psf = vec![0.0; n];
        psf[center] = 1.0;
        psf[center - 1] = 0.8;
        psf[center + 1] = 0.8;
        // Add distributed sidelobe energy
        for i in 0..n {
            if i < center - 1 || i > center + 1 {
                psf[i] = 0.05;
            }
        }
        let islr = integrated_sidelobe_ratio(&psf);
        // There should be finite sidelobe energy
        assert!(
            islr.is_finite(),
            "ISLR should be finite, got {}",
            islr
        );
        assert!(islr < 0.0, "ISLR should be negative (dB), got {}", islr);
    }

    // -- FFT helper tests ---------------------------------------------------

    #[test]
    fn test_fft_roundtrip() {
        let signal: Vec<(f64, f64)> = (0..16)
            .map(|i| {
                let t = i as f64 / 16.0;
                cexp(2.0 * PI * t * 3.0)
            })
            .collect();
        let spectrum = fft(&signal);
        let recovered = ifft(&spectrum);
        for (orig, rec) in signal.iter().zip(recovered.iter()) {
            assert!(
                (orig.0 - rec.0).abs() < 1e-10,
                "FFT roundtrip mismatch"
            );
            assert!(
                (orig.1 - rec.1).abs() < 1e-10,
                "FFT roundtrip mismatch"
            );
        }
    }
}
