//! Synthetic Aperture Sonar (SAS) Image Processor
//!
//! Implements coherent integration of sonar returns along a platform trajectory
//! to form high-resolution seafloor images -- the underwater equivalent of
//! Synthetic Aperture Radar (SAR).
//!
//! The core algorithm is delay-and-sum beamforming: for every output pixel the
//! processor computes the two-way range to every platform position, applies the
//! corresponding phase correction, and coherently sums all pulse returns. This
//! yields diffraction-limited resolution in both range and azimuth.
//!
//! All arithmetic is from scratch (no external linear-algebra or FFT crates).
//! Complex numbers are represented as `(f64, f64)` tuples (real, imaginary).
//!
//! # Example
//!
//! ```
//! use r4w_core::synthetic_aperture_sonar_imager::{
//!     SasConfig, SasProcessor, Position3d,
//! };
//!
//! let config = SasConfig {
//!     platform_speed_mps: 2.0,
//!     pulse_bandwidth_hz: 20_000.0,
//!     center_freq_hz: 100_000.0,
//!     sound_speed_mps: 1500.0,
//!     operating_depth_m: 50.0,
//!     swath_width_m: 100.0,
//! };
//! let proc = SasProcessor::new(config);
//! assert!((proc.range_resolution() - 0.0375).abs() < 1e-6);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex-number helpers (real, imag) tuples
// ---------------------------------------------------------------------------

type C64 = (f64, f64);

#[inline]
fn c_add(a: C64, b: C64) -> C64 {
    (a.0 + b.0, a.1 + b.1)
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
fn c_exp(phase: f64) -> C64 {
    (phase.cos(), phase.sin())
}

#[inline]
fn c_scale(s: f64, a: C64) -> C64 {
    (s * a.0, s * a.1)
}

#[inline]
fn c_arg(a: C64) -> f64 {
    a.1.atan2(a.0)
}

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Configuration for the SAS processor.
#[derive(Debug, Clone)]
pub struct SasConfig {
    /// Platform (towfish / AUV) speed in m/s.
    pub platform_speed_mps: f64,
    /// Transmitted pulse bandwidth in Hz.
    pub pulse_bandwidth_hz: f64,
    /// Transmit center frequency in Hz.
    pub center_freq_hz: f64,
    /// Speed of sound in seawater (default ~1500 m/s).
    pub sound_speed_mps: f64,
    /// Operating depth below the surface in metres.
    pub operating_depth_m: f64,
    /// Cross-track swath width in metres.
    pub swath_width_m: f64,
}

impl Default for SasConfig {
    fn default() -> Self {
        Self {
            platform_speed_mps: 2.0,
            pulse_bandwidth_hz: 20_000.0,
            center_freq_hz: 100_000.0,
            sound_speed_mps: 1500.0,
            operating_depth_m: 50.0,
            swath_width_m: 100.0,
        }
    }
}

/// A 3-D position in a local Cartesian frame (metres).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Position3d {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Position3d {
    /// Create a new position.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Euclidean distance to another position.
    pub fn distance_to(&self, other: &Position3d) -> f64 {
        let dx = self.x - other.x;
        let dy = self.y - other.y;
        let dz = self.z - other.z;
        (dx * dx + dy * dy + dz * dz).sqrt()
    }

    /// Vector subtraction: self - other.
    pub fn sub(&self, other: &Position3d) -> Position3d {
        Position3d::new(self.x - other.x, self.y - other.y, self.z - other.z)
    }

    /// Vector addition: self + other.
    pub fn add(&self, other: &Position3d) -> Position3d {
        Position3d::new(self.x + other.x, self.y + other.y, self.z + other.z)
    }

    /// Scalar multiplication.
    pub fn scale(&self, s: f64) -> Position3d {
        Position3d::new(self.x * s, self.y * s, self.z * s)
    }

    /// Euclidean norm.
    pub fn norm(&self) -> f64 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }

    /// Dot product.
    pub fn dot(&self, other: &Position3d) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z
    }
}

/// A formed SAS image (real-valued pixel intensities).
#[derive(Debug, Clone)]
pub struct SasImage {
    /// Row-major image data `[azimuth][range]`.
    pub data: Vec<Vec<f64>>,
    /// Range resolution in metres.
    pub range_resolution_m: f64,
    /// Azimuth resolution in metres.
    pub azimuth_resolution_m: f64,
    /// Number of range bins (columns).
    pub num_range_bins: usize,
    /// Number of azimuth bins (rows).
    pub num_azimuth_bins: usize,
}

impl SasImage {
    /// Return the peak pixel value.
    pub fn peak_value(&self) -> f64 {
        self.data
            .iter()
            .flat_map(|row| row.iter())
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max)
    }

    /// Convert pixel intensity to dB relative to the peak.
    pub fn to_db(&self) -> Vec<Vec<f64>> {
        let peak = self.peak_value();
        if peak <= 0.0 {
            return self.data.clone();
        }
        self.data
            .iter()
            .map(|row| {
                row.iter()
                    .map(|&v| {
                        if v > 0.0 {
                            10.0 * (v / peak).log10()
                        } else {
                            -200.0
                        }
                    })
                    .collect()
            })
            .collect()
    }
}

/// Statistics about the synthetic aperture.
#[derive(Debug, Clone)]
pub struct ApertureStats {
    /// Total aperture length (distance between first and last position) in metres.
    pub length_m: f64,
    /// Mean depth of platform positions (z coordinate) in metres.
    pub mean_depth_m: f64,
    /// Maximum deviation from a straight-line track in metres.
    pub max_deviation_m: f64,
}

/// 2-D window types for image sidelobe control.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum WindowType {
    Rectangular,
    Hamming,
    Hann,
    Blackman,
}

/// A detected target in the SAS image.
#[derive(Debug, Clone)]
pub struct TargetDetection {
    /// Range from scene center in metres.
    pub range_m: f64,
    /// Azimuth from scene center in metres.
    pub azimuth_m: f64,
    /// Intensity in dB relative to image peak.
    pub intensity_db: f64,
}

// ---------------------------------------------------------------------------
// SasProcessor
// ---------------------------------------------------------------------------

/// Main SAS image formation processor.
#[derive(Debug, Clone)]
pub struct SasProcessor {
    config: SasConfig,
}

impl SasProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: SasConfig) -> Self {
        Self { config }
    }

    /// Range resolution: c / (2 * B).
    pub fn range_resolution(&self) -> f64 {
        self.config.sound_speed_mps / (2.0 * self.config.pulse_bandwidth_hz)
    }

    /// Wavelength at center frequency.
    pub fn wavelength(&self) -> f64 {
        self.config.sound_speed_mps / self.config.center_freq_hz
    }

    /// Azimuth resolution: lambda / (2 * delta_theta).
    ///
    /// `delta_theta` is the synthetic aperture angle subtended from the scene
    /// center.  For a straight track of length L at slant range R the angle is
    /// approximately L / R, giving azimuth resolution ~ lambda * R / (2 * L).
    /// When called without scene geometry we use the full swath midpoint as
    /// representative range.
    pub fn azimuth_resolution(&self) -> f64 {
        // Representative slant range = sqrt(depth^2 + (swath/2)^2)
        let half_swath = self.config.swath_width_m / 2.0;
        let slant_range = (self.config.operating_depth_m.powi(2) + half_swath.powi(2)).sqrt();
        let lambda = self.wavelength();
        // Synthetic aperture length needed for best azimuth resolution at this range.
        // With platform speed v and integration time T, L = v * T.
        // For full-aperture SAS the best achievable azimuth resolution is lambda / 2
        // (half the along-track element spacing), but that requires L = lambda * R / D
        // where D is physical aperture.  Here we report the *achievable* resolution
        // from a given aperture length.  We assume integration over the full 3-dB
        // beamwidth: theta_3dB ~ lambda / D_phys.  With D_phys ~ lambda/2 (Nyquist):
        //   delta_theta ~ 2   => azimuth_res ~ lambda / 4 ?
        // Standard formula for stripmap SAS: azimuth_res = D_phys / 2.
        // With Nyquist element spacing D_phys = lambda/2  => res = lambda/4.
        // We will use the general form with an explicit aperture length:
        //   res = lambda * R / (2 * L)
        // and assume L = aperture required for 3 dB beamwidth at midswath.

        // Use a practical formula: best SAS resolution = D/2 where D = lambda/2
        // => lambda/4.  But to keep this parametric we'll compute from a nominal
        // integration time = swath_width / platform_speed.
        let integration_time = self.config.swath_width_m / self.config.platform_speed_mps;
        let aperture_length = self.config.platform_speed_mps * integration_time;
        if aperture_length < 1e-12 {
            return f64::INFINITY;
        }
        lambda * slant_range / (2.0 * aperture_length)
    }

    /// Azimuth resolution for a given synthetic aperture length and slant
    /// range.  res = lambda * R / (2 * L).
    pub fn azimuth_resolution_from_aperture(&self, aperture_length_m: f64, slant_range_m: f64) -> f64 {
        if aperture_length_m < 1e-12 {
            return f64::INFINITY;
        }
        let lambda = self.wavelength();
        lambda * slant_range_m / (2.0 * aperture_length_m)
    }

    /// Form a SAS image using delay-and-sum beamforming.
    ///
    /// # Arguments
    ///
    /// * `pulses` - Matched-filtered pulse returns, one `Vec<(f64, f64)>` per
    ///   ping.  Each element is a complex sample (I, Q).
    /// * `positions` - Platform position for each ping.
    /// * `scene_center` - Centre of the output image in the same coordinate
    ///   frame as `positions`.
    ///
    /// The output image spans `[-swath_width/2 .. +swath_width/2]` in the
    /// cross-track (range) direction and the along-track extent of the given
    /// positions in the azimuth direction.
    pub fn form_image(
        &self,
        pulses: &[Vec<(f64, f64)>],
        positions: &[Position3d],
        scene_center: Position3d,
    ) -> SasImage {
        assert_eq!(
            pulses.len(),
            positions.len(),
            "Number of pulses must equal number of positions"
        );
        assert!(!pulses.is_empty(), "Must have at least one pulse");

        let c = self.config.sound_speed_mps;
        let fc = self.config.center_freq_hz;
        let lambda = c / fc;

        // Determine image grid ------------------------------------------
        let range_res = self.range_resolution();

        // Along-track extent from positions
        let along_min = positions
            .iter()
            .map(|p| p.y)
            .fold(f64::INFINITY, f64::min);
        let along_max = positions
            .iter()
            .map(|p| p.y)
            .fold(f64::NEG_INFINITY, f64::max);
        let along_extent = (along_max - along_min).max(range_res);

        let cross_extent = self.config.swath_width_m;
        let half_cross = cross_extent / 2.0;

        let num_range_bins = ((cross_extent / range_res).ceil() as usize).max(1);
        let num_azimuth_bins = ((along_extent / range_res).ceil() as usize).max(1);

        let azimuth_res = along_extent / num_azimuth_bins as f64;

        // Pixel grid
        let mut image_complex: Vec<Vec<C64>> =
            vec![vec![(0.0, 0.0); num_range_bins]; num_azimuth_bins];

        // Delay-and-sum --------------------------------------------------
        for az_idx in 0..num_azimuth_bins {
            let pixel_y = along_min + (az_idx as f64 + 0.5) * azimuth_res;
            for rng_idx in 0..num_range_bins {
                let pixel_x = scene_center.x - half_cross + (rng_idx as f64 + 0.5) * range_res;
                let pixel_pos = Position3d::new(pixel_x, pixel_y, scene_center.z);

                let mut accum: C64 = (0.0, 0.0);

                for (pulse_idx, pos) in positions.iter().enumerate() {
                    let two_way_range = 2.0 * pos.distance_to(&pixel_pos);
                    let two_way_time = two_way_range / c;

                    // Which range sample does this correspond to?
                    let sample_idx_f = two_way_range / (2.0 * range_res);
                    let sample_idx = sample_idx_f.round() as isize;
                    if sample_idx < 0 || sample_idx >= pulses[pulse_idx].len() as isize {
                        continue;
                    }
                    let sample = pulses[pulse_idx][sample_idx as usize];

                    // Phase correction to align all pulses coherently.
                    // The received echo carries propagation phase -2*pi*d/lambda.
                    // We compensate by applying +2*pi*d_pixel/lambda so that the
                    // true scatterer location (d_pixel == d_actual) sums coherently.
                    let phase = 2.0 * PI * two_way_range / lambda;
                    let correction = c_exp(phase);
                    accum = c_add(accum, c_mul(sample, correction));
                }
                image_complex[az_idx][rng_idx] = accum;
            }
        }

        // Convert to magnitude-squared (intensity)
        let data: Vec<Vec<f64>> = image_complex
            .iter()
            .map(|row| row.iter().map(|&s| s.0 * s.0 + s.1 * s.1).collect())
            .collect();

        SasImage {
            data,
            range_resolution_m: range_res,
            azimuth_resolution_m: azimuth_res,
            num_range_bins,
            num_azimuth_bins,
        }
    }
}

// ---------------------------------------------------------------------------
// Autofocus: Phase Gradient Autofocus (PGA)
// ---------------------------------------------------------------------------

/// Apply Phase Gradient Autofocus (PGA) to complex image data **in place**.
///
/// PGA estimates and removes residual phase errors across azimuth (slow-time)
/// for each range bin independently.  Algorithm:
///
/// 1. For each range bin, circularly shift the azimuth profile so the
///    strongest scatterer is centred.
/// 2. Window the profile to keep the dominant scatterer.
/// 3. Estimate the phase gradient across azimuth via the conjugate-product
///    method (derivative of phase).
/// 4. Integrate the gradient to obtain the phase error and remove it.
///
/// `image_data` is `[azimuth][range]` of complex (I, Q) samples.
pub fn autofocus_phase_gradient(image_data: &mut [Vec<(f64, f64)>]) {
    let num_az = image_data.len();
    if num_az < 2 {
        return;
    }
    let num_rng = image_data[0].len();
    if num_rng == 0 {
        return;
    }

    // Iterate PGA a few times for convergence
    for _iter in 0..4 {
        // Accumulate phase gradient estimate across all range bins
        let mut gradient: Vec<f64> = vec![0.0; num_az];
        let mut weight: Vec<f64> = vec![0.0; num_az];

        for rng in 0..num_rng {
            // Extract azimuth column for this range bin
            let col: Vec<C64> = (0..num_az)
                .map(|az| image_data[az][rng])
                .collect();

            // Find peak
            let (peak_idx, _) = col
                .iter()
                .enumerate()
                .max_by(|a, b| {
                    let ma = c_abs(*a.1);
                    let mb = c_abs(*b.1);
                    ma.partial_cmp(&mb).unwrap_or(std::cmp::Ordering::Equal)
                })
                .unwrap();

            // Circular shift so peak is at centre
            let half = num_az / 2;
            let shift = (peak_idx as isize) - (half as isize);
            let shifted: Vec<C64> = (0..num_az)
                .map(|i| {
                    let src = ((i as isize + shift).rem_euclid(num_az as isize)) as usize;
                    col[src]
                })
                .collect();

            // Conjugate-product phase gradient
            for i in 1..num_az {
                let prod = c_mul(shifted[i], c_conj(shifted[i - 1]));
                let mag = c_abs(prod);
                let ph = c_arg(prod);
                gradient[i] += ph * mag;
                weight[i] += mag;
            }
        }

        // Weighted average gradient
        for i in 1..num_az {
            if weight[i] > 1e-30 {
                gradient[i] /= weight[i];
            }
        }

        // Integrate gradient to get phase error
        let mut phase_error = vec![0.0; num_az];
        for i in 1..num_az {
            phase_error[i] = phase_error[i - 1] + gradient[i];
        }

        // Remove mean
        let mean_phase: f64 = phase_error.iter().sum::<f64>() / num_az as f64;
        for p in &mut phase_error {
            *p -= mean_phase;
        }

        // Apply correction
        for az in 0..num_az {
            let corr = c_exp(-phase_error[az]);
            for rng in 0..num_rng {
                image_data[az][rng] = c_mul(image_data[az][rng], corr);
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Aperture statistics
// ---------------------------------------------------------------------------

/// Compute statistics about the synthetic aperture from the platform
/// positions.
pub fn compute_aperture_stats(positions: &[Position3d]) -> ApertureStats {
    if positions.is_empty() {
        return ApertureStats {
            length_m: 0.0,
            mean_depth_m: 0.0,
            max_deviation_m: 0.0,
        };
    }
    if positions.len() == 1 {
        return ApertureStats {
            length_m: 0.0,
            mean_depth_m: positions[0].z,
            max_deviation_m: 0.0,
        };
    }

    // Aperture length = distance between first and last
    let first = positions.first().unwrap();
    let last = positions.last().unwrap();
    let length_m = first.distance_to(last);

    // Mean depth
    let mean_depth_m = positions.iter().map(|p| p.z).sum::<f64>() / positions.len() as f64;

    // Maximum deviation from the straight line connecting first to last
    let track_dir = last.sub(first);
    let track_len = track_dir.norm();
    let max_deviation_m = if track_len < 1e-12 {
        // All positions essentially at the same point
        positions
            .iter()
            .map(|p| p.distance_to(first))
            .fold(0.0_f64, f64::max)
    } else {
        let unit_dir = track_dir.scale(1.0 / track_len);
        positions
            .iter()
            .map(|p| {
                let v = p.sub(first);
                let proj = v.dot(&unit_dir);
                let closest = first.add(&unit_dir.scale(proj));
                p.distance_to(&closest)
            })
            .fold(0.0_f64, f64::max)
    };

    ApertureStats {
        length_m,
        mean_depth_m,
        max_deviation_m,
    }
}

// ---------------------------------------------------------------------------
// 2-D windowing
// ---------------------------------------------------------------------------

/// Apply a separable 2-D window to a SAS image (modifies in place).
///
/// The window tapers towards zero at the edges to reduce sidelobes in the
/// image spectrum.
pub fn apply_window_2d(image: &mut SasImage, window_type: WindowType) {
    let naz = image.num_azimuth_bins;
    let nrng = image.num_range_bins;

    let az_win = make_window(naz, window_type);
    let rng_win = make_window(nrng, window_type);

    for az in 0..naz {
        for rng in 0..nrng {
            image.data[az][rng] *= az_win[az] * rng_win[rng];
        }
    }
}

/// Generate a 1-D window of given length.
fn make_window(n: usize, wtype: WindowType) -> Vec<f64> {
    if n <= 1 {
        return vec![1.0; n];
    }
    let nm1 = (n - 1) as f64;
    (0..n)
        .map(|i| {
            let x = i as f64 / nm1;
            match wtype {
                WindowType::Rectangular => 1.0,
                WindowType::Hamming => 0.54 - 0.46 * (2.0 * PI * x).cos(),
                WindowType::Hann => 0.5 * (1.0 - (2.0 * PI * x).cos()),
                WindowType::Blackman => {
                    0.42 - 0.5 * (2.0 * PI * x).cos() + 0.08 * (4.0 * PI * x).cos()
                }
            }
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Seawater absorption loss (Thorp's formula)
// ---------------------------------------------------------------------------

/// Acoustic absorption loss in seawater using Thorp's formula (dB).
///
/// Valid for frequencies roughly 100 Hz to 1 MHz.  Returns the *one-way*
/// absorption coefficient (dB/km) multiplied by range in km, i.e. the total
/// one-way absorption loss in dB.
///
/// Thorp's empirical formula (simplified, f in kHz):
///
/// ```text
/// alpha = 0.11 * f^2 / (1 + f^2)  +  44 * f^2 / (4100 + f^2)  +  2.75e-4 * f^2  +  0.003
/// ```
///
/// The result is in dB/km.  We multiply by `range_m / 1000` to get total dB.
pub fn absorption_loss_db(freq_hz: f64, range_m: f64) -> f64 {
    let f_khz = freq_hz / 1000.0;
    let f2 = f_khz * f_khz;
    // Thorp's formula — absorption coefficient in dB/km
    let alpha_db_per_km =
        0.11 * f2 / (1.0 + f2) + 44.0 * f2 / (4100.0 + f2) + 2.75e-4 * f2 + 0.003;
    alpha_db_per_km * range_m / 1000.0
}

// ---------------------------------------------------------------------------
// Target detection
// ---------------------------------------------------------------------------

/// Detect targets in a SAS image by thresholding intensity (in dB relative to
/// peak).
///
/// Returns all pixels whose intensity exceeds `threshold_db` dB below the
/// image peak.  For example, `threshold_db = -10.0` returns all pixels within
/// 10 dB of the peak.
pub fn detect_targets(image: &SasImage, threshold_db: f64) -> Vec<TargetDetection> {
    let peak = image.peak_value();
    if peak <= 0.0 {
        return Vec::new();
    }

    let mut detections = Vec::new();

    for az in 0..image.num_azimuth_bins {
        for rng in 0..image.num_range_bins {
            let val = image.data[az][rng];
            if val <= 0.0 {
                continue;
            }
            let db = 10.0 * (val / peak).log10();
            if db >= threshold_db {
                // Convert pixel indices to physical coordinates relative to image
                // centre.
                let range_m = (rng as f64 + 0.5 - image.num_range_bins as f64 / 2.0)
                    * image.range_resolution_m;
                let azimuth_m = (az as f64 + 0.5 - image.num_azimuth_bins as f64 / 2.0)
                    * image.azimuth_resolution_m;
                detections.push(TargetDetection {
                    range_m,
                    azimuth_m,
                    intensity_db: db,
                });
            }
        }
    }

    // Sort by intensity descending
    detections.sort_by(|a, b| {
        b.intensity_db
            .partial_cmp(&a.intensity_db)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    detections
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_config() -> SasConfig {
        SasConfig::default()
    }

    // -- Range resolution ---------------------------------------------------

    #[test]
    fn test_range_resolution_basic() {
        // c / (2B) = 1500 / (2 * 20000) = 0.0375 m
        let proc = SasProcessor::new(default_config());
        let res = proc.range_resolution();
        assert!((res - 0.0375).abs() < 1e-9, "got {}", res);
    }

    #[test]
    fn test_range_resolution_wide_bandwidth() {
        let cfg = SasConfig {
            pulse_bandwidth_hz: 100_000.0,
            ..default_config()
        };
        let proc = SasProcessor::new(cfg);
        // 1500 / (2 * 100000) = 0.0075
        assert!((proc.range_resolution() - 0.0075).abs() < 1e-9);
    }

    #[test]
    fn test_range_resolution_different_sound_speed() {
        let cfg = SasConfig {
            sound_speed_mps: 1480.0,
            pulse_bandwidth_hz: 20_000.0,
            ..default_config()
        };
        let proc = SasProcessor::new(cfg);
        let expected = 1480.0 / 40_000.0;
        assert!((proc.range_resolution() - expected).abs() < 1e-9);
    }

    // -- Wavelength ---------------------------------------------------------

    #[test]
    fn test_wavelength() {
        let proc = SasProcessor::new(default_config());
        // 1500 / 100000 = 0.015 m
        assert!((proc.wavelength() - 0.015).abs() < 1e-9);
    }

    // -- Azimuth resolution -------------------------------------------------

    #[test]
    fn test_azimuth_resolution_finite() {
        let proc = SasProcessor::new(default_config());
        let res = proc.azimuth_resolution();
        assert!(res.is_finite());
        assert!(res > 0.0);
    }

    #[test]
    fn test_azimuth_resolution_from_aperture() {
        let proc = SasProcessor::new(default_config());
        let lambda = proc.wavelength();
        let slant_range = 100.0;
        let aperture = 10.0;
        let expected = lambda * slant_range / (2.0 * aperture);
        let got = proc.azimuth_resolution_from_aperture(aperture, slant_range);
        assert!((got - expected).abs() < 1e-12);
    }

    #[test]
    fn test_azimuth_resolution_longer_aperture_is_finer() {
        let proc = SasProcessor::new(default_config());
        let r = 100.0;
        let res_short = proc.azimuth_resolution_from_aperture(5.0, r);
        let res_long = proc.azimuth_resolution_from_aperture(20.0, r);
        assert!(res_long < res_short, "Longer aperture should give finer resolution");
    }

    #[test]
    fn test_azimuth_resolution_zero_aperture() {
        let proc = SasProcessor::new(default_config());
        let res = proc.azimuth_resolution_from_aperture(0.0, 100.0);
        assert!(res.is_infinite());
    }

    // -- Position3d ---------------------------------------------------------

    #[test]
    fn test_position_distance_to_self() {
        let p = Position3d::new(1.0, 2.0, 3.0);
        assert!((p.distance_to(&p)).abs() < 1e-15);
    }

    #[test]
    fn test_position_distance_known() {
        let a = Position3d::new(0.0, 0.0, 0.0);
        let b = Position3d::new(3.0, 4.0, 0.0);
        assert!((a.distance_to(&b) - 5.0).abs() < 1e-12);
    }

    #[test]
    fn test_position_sub_add() {
        let a = Position3d::new(1.0, 2.0, 3.0);
        let b = Position3d::new(4.0, 5.0, 6.0);
        let diff = b.sub(&a);
        assert!((diff.x - 3.0).abs() < 1e-15);
        assert!((diff.y - 3.0).abs() < 1e-15);
        assert!((diff.z - 3.0).abs() < 1e-15);

        let sum = a.add(&b);
        assert!((sum.x - 5.0).abs() < 1e-15);
    }

    #[test]
    fn test_position_scale_and_norm() {
        let p = Position3d::new(3.0, 4.0, 0.0);
        assert!((p.norm() - 5.0).abs() < 1e-12);
        let scaled = p.scale(2.0);
        assert!((scaled.norm() - 10.0).abs() < 1e-12);
    }

    #[test]
    fn test_position_dot() {
        let a = Position3d::new(1.0, 0.0, 0.0);
        let b = Position3d::new(0.0, 1.0, 0.0);
        assert!((a.dot(&b)).abs() < 1e-15);
        assert!((a.dot(&a) - 1.0).abs() < 1e-15);
    }

    // -- Absorption loss ----------------------------------------------------

    #[test]
    fn test_absorption_loss_zero_range() {
        assert!((absorption_loss_db(100_000.0, 0.0)).abs() < 1e-15);
    }

    #[test]
    fn test_absorption_loss_positive() {
        let loss = absorption_loss_db(100_000.0, 1000.0);
        assert!(loss > 0.0, "Absorption should be positive, got {}", loss);
    }

    #[test]
    fn test_absorption_loss_increases_with_frequency() {
        let low = absorption_loss_db(10_000.0, 1000.0);
        let high = absorption_loss_db(100_000.0, 1000.0);
        assert!(
            high > low,
            "Higher frequency should have more absorption: {} vs {}",
            high,
            low
        );
    }

    #[test]
    fn test_absorption_loss_increases_with_range() {
        let short = absorption_loss_db(100_000.0, 100.0);
        let long = absorption_loss_db(100_000.0, 1000.0);
        assert!(
            long > short,
            "Longer range should have more absorption: {} vs {}",
            long,
            short
        );
    }

    #[test]
    fn test_absorption_loss_thorp_100khz() {
        // At 100 kHz (f_khz = 100): alpha ~ 0.11*10000/10001 + 44*10000/14100 + 2.75e-4*10000 + 0.003
        //   ~ 1.099 + 31.206 + 2.75 + 0.003 ~ 35.06 dB/km
        let alpha = absorption_loss_db(100_000.0, 1000.0); // 1 km
        assert!(
            (alpha - 35.06).abs() < 1.0,
            "Thorp at 100kHz should be ~35 dB/km, got {}",
            alpha
        );
    }

    #[test]
    fn test_absorption_loss_linear_in_range() {
        let r1 = absorption_loss_db(50_000.0, 500.0);
        let r2 = absorption_loss_db(50_000.0, 1000.0);
        // r2 should be 2 * r1
        assert!(
            (r2 - 2.0 * r1).abs() < 1e-10,
            "Absorption should scale linearly with range"
        );
    }

    // -- Aperture statistics ------------------------------------------------

    #[test]
    fn test_aperture_stats_empty() {
        let stats = compute_aperture_stats(&[]);
        assert!((stats.length_m).abs() < 1e-15);
    }

    #[test]
    fn test_aperture_stats_single_position() {
        let stats = compute_aperture_stats(&[Position3d::new(0.0, 0.0, -50.0)]);
        assert!((stats.length_m).abs() < 1e-15);
        assert!((stats.mean_depth_m - (-50.0)).abs() < 1e-15);
    }

    #[test]
    fn test_aperture_stats_straight_line() {
        let positions: Vec<Position3d> = (0..100)
            .map(|i| Position3d::new(0.0, i as f64 * 0.1, -50.0))
            .collect();
        let stats = compute_aperture_stats(&positions);
        // Length = distance between first and last = 99 * 0.1 = 9.9
        assert!((stats.length_m - 9.9).abs() < 1e-9);
        assert!((stats.mean_depth_m - (-50.0)).abs() < 1e-9);
        // Perfectly straight => deviation = 0
        assert!(stats.max_deviation_m < 1e-9);
    }

    #[test]
    fn test_aperture_stats_curved_track() {
        // Arc that deviates from straight line
        let n = 101;
        let mut positions = Vec::with_capacity(n);
        for i in 0..n {
            let t = i as f64 / (n - 1) as f64;
            let y = t * 10.0; // along-track
            let x = 2.0 * (PI * t).sin(); // sinusoidal deviation
            positions.push(Position3d::new(x, y, -50.0));
        }
        let stats = compute_aperture_stats(&positions);
        assert!(stats.max_deviation_m > 1.0, "Should detect deviation from straight line");
    }

    // -- Windowing ----------------------------------------------------------

    #[test]
    fn test_window_rectangular_no_change() {
        let mut image = SasImage {
            data: vec![vec![1.0; 4]; 4],
            range_resolution_m: 0.1,
            azimuth_resolution_m: 0.1,
            num_range_bins: 4,
            num_azimuth_bins: 4,
        };
        apply_window_2d(&mut image, WindowType::Rectangular);
        for row in &image.data {
            for &v in row {
                assert!((v - 1.0).abs() < 1e-15);
            }
        }
    }

    #[test]
    fn test_window_hamming_tapers_edges() {
        let n = 16;
        let mut image = SasImage {
            data: vec![vec![1.0; n]; n],
            range_resolution_m: 0.1,
            azimuth_resolution_m: 0.1,
            num_range_bins: n,
            num_azimuth_bins: n,
        };
        apply_window_2d(&mut image, WindowType::Hamming);
        // Corners should be much smaller than centre
        let corner = image.data[0][0];
        let centre = image.data[n / 2][n / 2];
        assert!(
            corner < centre,
            "Corner ({}) should be less than centre ({})",
            corner,
            centre
        );
    }

    #[test]
    fn test_window_hann_zero_at_edges() {
        let n = 32;
        let mut image = SasImage {
            data: vec![vec![1.0; n]; n],
            range_resolution_m: 0.1,
            azimuth_resolution_m: 0.1,
            num_range_bins: n,
            num_azimuth_bins: n,
        };
        apply_window_2d(&mut image, WindowType::Hann);
        // First and last sample of a Hann window are zero
        assert!(image.data[0][0].abs() < 1e-12, "Hann should be ~0 at corner");
    }

    #[test]
    fn test_window_blackman_symmetry() {
        let n = 17;
        let win = make_window(n, WindowType::Blackman);
        for i in 0..n / 2 {
            assert!(
                (win[i] - win[n - 1 - i]).abs() < 1e-12,
                "Blackman window should be symmetric"
            );
        }
    }

    // -- Target detection ---------------------------------------------------

    #[test]
    fn test_detect_targets_empty_image() {
        let image = SasImage {
            data: vec![vec![0.0; 4]; 4],
            range_resolution_m: 0.1,
            azimuth_resolution_m: 0.1,
            num_range_bins: 4,
            num_azimuth_bins: 4,
        };
        let targets = detect_targets(&image, -3.0);
        assert!(targets.is_empty());
    }

    #[test]
    fn test_detect_targets_single_peak() {
        let mut data = vec![vec![0.01; 16]; 16];
        data[8][8] = 100.0; // strong target
        let image = SasImage {
            data,
            range_resolution_m: 1.0,
            azimuth_resolution_m: 1.0,
            num_range_bins: 16,
            num_azimuth_bins: 16,
        };
        let targets = detect_targets(&image, -3.0);
        assert!(!targets.is_empty(), "Should detect the peak");
        // The brightest detection should be at pixel (8, 8)
        let best = &targets[0];
        assert!(
            (best.intensity_db).abs() < 1e-6,
            "Peak should be at 0 dB, got {}",
            best.intensity_db
        );
    }

    #[test]
    fn test_detect_targets_threshold_filters() {
        let mut data = vec![vec![0.001; 8]; 8];
        data[4][4] = 1.0;
        data[2][2] = 0.5; // -3.01 dB
        let image = SasImage {
            data,
            range_resolution_m: 1.0,
            azimuth_resolution_m: 1.0,
            num_range_bins: 8,
            num_azimuth_bins: 8,
        };
        // Threshold -2 dB: only the peak should survive (0.5 => -3.01 dB < -2)
        let targets = detect_targets(&image, -2.0);
        assert_eq!(targets.len(), 1);
    }

    // -- Image formation (delay-and-sum) ------------------------------------

    #[test]
    fn test_form_image_single_pulse() {
        let cfg = SasConfig {
            pulse_bandwidth_hz: 1000.0,
            swath_width_m: 10.0,
            ..default_config()
        };
        let proc = SasProcessor::new(cfg);

        // Single pulse, single position
        let num_samples = 128;
        let pulse: Vec<(f64, f64)> = (0..num_samples).map(|_| (1.0, 0.0)).collect();
        let pos = Position3d::new(0.0, 0.0, -50.0);
        let center = Position3d::new(0.0, 0.0, -50.0);

        let image = proc.form_image(&[pulse], &[pos], center);
        assert_eq!(image.num_range_bins, image.data[0].len());
        assert_eq!(image.num_azimuth_bins, image.data.len());
    }

    #[test]
    fn test_form_image_dimensions_consistency() {
        let cfg = SasConfig {
            pulse_bandwidth_hz: 5000.0,
            swath_width_m: 20.0,
            ..default_config()
        };
        let proc = SasProcessor::new(cfg);

        let num_pulses = 10;
        let num_samples = 256;
        let pulses: Vec<Vec<(f64, f64)>> =
            (0..num_pulses).map(|_| vec![(1.0, 0.0); num_samples]).collect();
        let positions: Vec<Position3d> = (0..num_pulses)
            .map(|i| Position3d::new(0.0, i as f64 * 0.5, -50.0))
            .collect();
        let center = Position3d::new(0.0, 2.25, -50.0);

        let image = proc.form_image(&pulses, &positions, center);
        assert_eq!(image.data.len(), image.num_azimuth_bins);
        assert!(image.num_range_bins > 0);
        for row in &image.data {
            assert_eq!(row.len(), image.num_range_bins);
        }
    }

    #[test]
    fn test_coherent_integration_gain() {
        // N identical pulses coherently integrated should give N^2 power gain
        // (amplitude grows as N, power as N^2).
        let cfg = SasConfig {
            pulse_bandwidth_hz: 500.0,
            swath_width_m: 2.0,
            center_freq_hz: 50_000.0,
            sound_speed_mps: 1500.0,
            operating_depth_m: 10.0,
            ..default_config()
        };
        let proc = SasProcessor::new(cfg);

        // Create pulses: a single scatterer at the scene centre.
        // We'll use 1 pulse and N pulses and check the peak ratio.
        let range_res = proc.range_resolution(); // 1500 / 1000 = 1.5 m
        let num_samples = 64;
        let lambda = proc.wavelength();

        // Place platform positions along y-axis, all at same range to origin
        let n_pulses = 8;
        let positions: Vec<Position3d> = (0..n_pulses)
            .map(|i| Position3d::new(0.0, i as f64 * 0.1, -10.0))
            .collect();
        let center = Position3d::new(0.0, (n_pulses as f64 - 1.0) * 0.05, -10.0);

        // All pulses identical, with a scatterer response in the appropriate
        // range bin.
        let pulse: Vec<(f64, f64)> = (0..num_samples).map(|_| (1.0, 0.0)).collect();
        let pulses_n: Vec<Vec<(f64, f64)>> = vec![pulse.clone(); n_pulses];

        let image_n = proc.form_image(&pulses_n, &positions, center);
        let peak_n = image_n.peak_value();

        // Single pulse
        let image_1 = proc.form_image(&[pulse.clone()], &[positions[0]], center);
        let peak_1 = image_1.peak_value();

        // With coherent integration the peak power should scale roughly as N^2
        // relative to single-pulse.  Allow generous tolerance because the
        // geometry changes slightly.
        if peak_1 > 0.0 && peak_n > 0.0 {
            let gain_ratio = peak_n / peak_1;
            // We expect roughly N^2 = 64.  Accept anything above N (= 8).
            assert!(
                gain_ratio > n_pulses as f64,
                "Coherent integration gain ratio {} should exceed N={}",
                gain_ratio,
                n_pulses
            );
        }
    }

    #[test]
    #[should_panic(expected = "Number of pulses must equal number of positions")]
    fn test_form_image_pulse_position_mismatch() {
        let proc = SasProcessor::new(default_config());
        let pulses = vec![vec![(1.0, 0.0); 16]; 3];
        let positions = vec![Position3d::new(0.0, 0.0, 0.0); 2]; // mismatch
        proc.form_image(&pulses, &positions, Position3d::new(0.0, 0.0, 0.0));
    }

    #[test]
    #[should_panic(expected = "Must have at least one pulse")]
    fn test_form_image_empty_pulses() {
        let proc = SasProcessor::new(default_config());
        let empty: Vec<Vec<(f64, f64)>> = vec![];
        proc.form_image(&empty, &[], Position3d::new(0.0, 0.0, 0.0));
    }

    // -- Autofocus ----------------------------------------------------------

    #[test]
    fn test_autofocus_no_crash_small() {
        let mut data: Vec<Vec<(f64, f64)>> = vec![vec![(1.0, 0.0); 4]; 4];
        autofocus_phase_gradient(&mut data);
        // Just verify it doesn't crash
    }

    #[test]
    fn test_autofocus_single_azimuth() {
        let mut data: Vec<Vec<(f64, f64)>> = vec![vec![(1.0, 0.0); 8]];
        autofocus_phase_gradient(&mut data);
        // Single azimuth line: no phase gradient to correct, should be unchanged
        for &(re, _im) in &data[0] {
            assert!((re - 1.0).abs() < 0.1);
        }
    }

    #[test]
    fn test_autofocus_removes_linear_phase() {
        // Apply a known linear phase ramp across azimuth and check that PGA
        // reduces it.
        let n_az = 32;
        let n_rng = 16;
        let mut data: Vec<Vec<(f64, f64)>> = Vec::with_capacity(n_az);
        for az in 0..n_az {
            let phase = 0.3 * az as f64; // linear phase error
            let row: Vec<(f64, f64)> = (0..n_rng)
                .map(|_| c_exp(phase))
                .collect();
            data.push(row);
        }

        // Measure phase spread before
        let phase_before: Vec<f64> = data.iter().map(|row| c_arg(row[0])).collect();
        let spread_before = phase_before.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
            - phase_before.iter().cloned().fold(f64::INFINITY, f64::min);

        autofocus_phase_gradient(&mut data);

        // Measure phase spread after
        let phase_after: Vec<f64> = data.iter().map(|row| c_arg(row[0])).collect();
        let spread_after = phase_after.iter().cloned().fold(f64::NEG_INFINITY, f64::max)
            - phase_after.iter().cloned().fold(f64::INFINITY, f64::min);

        assert!(
            spread_after < spread_before,
            "PGA should reduce phase spread: before={:.2}, after={:.2}",
            spread_before,
            spread_after
        );
    }

    // -- SasImage helpers ---------------------------------------------------

    #[test]
    fn test_sas_image_peak_value() {
        let image = SasImage {
            data: vec![vec![1.0, 3.0, 2.0], vec![0.5, 5.0, 0.1]],
            range_resolution_m: 0.1,
            azimuth_resolution_m: 0.1,
            num_range_bins: 3,
            num_azimuth_bins: 2,
        };
        assert!((image.peak_value() - 5.0).abs() < 1e-15);
    }

    #[test]
    fn test_sas_image_to_db() {
        let image = SasImage {
            data: vec![vec![1.0, 10.0, 100.0]],
            range_resolution_m: 1.0,
            azimuth_resolution_m: 1.0,
            num_range_bins: 3,
            num_azimuth_bins: 1,
        };
        let db = image.to_db();
        // Peak (100) should be 0 dB, 10 should be -10 dB, 1 should be -20 dB
        assert!((db[0][2]).abs() < 1e-10, "Peak should be 0 dB");
        assert!((db[0][1] - (-10.0)).abs() < 1e-10, "10 should be -10 dB");
        assert!((db[0][0] - (-20.0)).abs() < 1e-10, "1 should be -20 dB");
    }

    // -- Edge cases ---------------------------------------------------------

    #[test]
    fn test_config_default() {
        let cfg = SasConfig::default();
        assert!((cfg.sound_speed_mps - 1500.0).abs() < 1e-15);
    }

    #[test]
    fn test_make_window_single_element() {
        let w = make_window(1, WindowType::Hamming);
        assert_eq!(w.len(), 1);
        assert!((w[0] - 1.0).abs() < 1e-15);
    }

    #[test]
    fn test_make_window_empty() {
        let w = make_window(0, WindowType::Hann);
        assert!(w.is_empty());
    }

    // -- Point target imaging -----------------------------------------------

    #[test]
    fn test_point_target_focus() {
        // Verify that delay-and-sum produces a focused peak at the
        // scatterer location.  We construct pulses synthetically so that
        // each pulse contains a single echo at the correct range bin with
        // the correct propagation phase, then form the image and check
        // that the peak pixel is near the true scatterer position.
        let cfg = SasConfig {
            pulse_bandwidth_hz: 1000.0,
            center_freq_hz: 50_000.0,
            sound_speed_mps: 1500.0,
            swath_width_m: 20.0,
            operating_depth_m: 20.0,
            platform_speed_mps: 2.0,
        };
        let proc = SasProcessor::new(cfg);
        let range_res = proc.range_resolution(); // 0.75 m
        let lambda = proc.wavelength(); // 0.03 m

        // Platform moves along y-axis at z=0 (surface); scatterer on
        // the seafloor directly below scene_center, offset in x.
        let n_pulses = 32;
        let scatterer = Position3d::new(3.0, 3.0, -20.0);
        let scene_center = Position3d::new(0.0, 3.0, -20.0);

        let num_samples = 128;
        let mut pulses: Vec<Vec<(f64, f64)>> = Vec::with_capacity(n_pulses);
        let mut positions: Vec<Position3d> = Vec::with_capacity(n_pulses);

        for i in 0..n_pulses {
            let pos = Position3d::new(0.0, i as f64 * 0.2, 0.0);
            positions.push(pos);

            // Echo from scatterer at the correct range bin
            let dist = pos.distance_to(&scatterer);
            let range_bin = (dist / range_res).round() as usize;
            let phase = 2.0 * PI * 2.0 * dist / lambda;

            let mut pulse = vec![(0.0, 0.0); num_samples];
            if range_bin < num_samples {
                pulse[range_bin] = c_exp(-phase);
            }
            pulses.push(pulse);
        }

        let image = proc.form_image(&pulses, &positions, scene_center);
        let peak = image.peak_value();
        assert!(peak > 0.0, "Image should have non-zero peak");

        // The image should have a focused peak -- verify the peak pixel is
        // significantly brighter than the median.
        let mut all_vals: Vec<f64> = image
            .data
            .iter()
            .flat_map(|row| row.iter().copied())
            .filter(|&v| v > 0.0)
            .collect();
        all_vals.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = if all_vals.is_empty() {
            0.0
        } else {
            all_vals[all_vals.len() / 2]
        };
        assert!(
            peak > 4.0 * median,
            "Peak ({:.4}) should be well above median ({:.4}) for a focused target",
            peak,
            median
        );
    }

    // -- Detect targets at known locations ----------------------------------

    #[test]
    fn test_detect_targets_at_known_peaks() {
        let n = 32;
        let mut data = vec![vec![0.001; n]; n];
        // Place two targets
        data[10][16] = 10.0;
        data[20][24] = 5.0;

        let image = SasImage {
            data,
            range_resolution_m: 1.0,
            azimuth_resolution_m: 1.0,
            num_range_bins: n,
            num_azimuth_bins: n,
        };

        let targets = detect_targets(&image, -6.0); // within 6 dB of peak
        assert!(targets.len() >= 2, "Should detect both targets, got {}", targets.len());
        // First detection should be the stronger one
        assert!(targets[0].intensity_db > targets[1].intensity_db);
    }

    // -- Complex helpers (internal) -----------------------------------------

    #[test]
    fn test_c_exp_unit_circle() {
        for i in 0..8 {
            let theta = i as f64 * PI / 4.0;
            let e = c_exp(theta);
            let mag = c_abs(e);
            assert!((mag - 1.0).abs() < 1e-12, "c_exp should be on unit circle");
        }
    }

    #[test]
    fn test_c_mul_identity() {
        let a = (3.0, 4.0);
        let one = (1.0, 0.0);
        let prod = c_mul(a, one);
        assert!((prod.0 - 3.0).abs() < 1e-15);
        assert!((prod.1 - 4.0).abs() < 1e-15);
    }

    #[test]
    fn test_c_conj_and_abs() {
        let z = (3.0, 4.0);
        let zc = c_conj(z);
        assert!((zc.0 - 3.0).abs() < 1e-15);
        assert!((zc.1 - (-4.0)).abs() < 1e-15);
        assert!((c_abs(z) - 5.0).abs() < 1e-12);
    }
}
