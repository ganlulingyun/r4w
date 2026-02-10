//! # FMCW Automotive Radar Signal Processor
//!
//! This module processes Frequency-Modulated Continuous Wave (FMCW) radar signals
//! for automotive target detection, classification, and tracking. It implements the
//! full processing chain from raw beat-frequency chirps through range-Doppler map
//! generation, CFAR detection, rule-based target classification, and frame-to-frame
//! nearest-neighbor tracking.
//!
//! ## Overview
//!
//! Automotive FMCW radar (typically at 77 GHz) transmits a sequence of linear
//! frequency chirps. The received echo is mixed with the transmit signal to produce
//! a beat frequency proportional to target range. By stacking multiple chirps and
//! performing a second FFT across the slow-time dimension, Doppler (velocity)
//! information is extracted.
//!
//! ## Processing Chain
//!
//! ```text
//! Raw Chirps → Range FFT → Doppler FFT → CFAR Detection → Classification → Tracking
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::fmcw_automotive_processor::{
//!     FmcwAutomotiveProcessor, RadarConfig, TargetClass,
//! };
//!
//! // 77 GHz radar with 1 GHz BW, 60 us chirps, 40 MSPS
//! let config = RadarConfig {
//!     center_freq_ghz: 77.0,
//!     bandwidth_mhz: 1000.0,
//!     chirp_duration_us: 60.0,
//!     num_chirps: 64,
//!     sample_rate: 40_000_000.0,
//!     max_range_m: 200.0,
//!     cfar_guard_cells: 2,
//!     cfar_training_cells: 8,
//!     cfar_threshold_db: 12.0,
//! };
//!
//! let processor = FmcwAutomotiveProcessor::new(config);
//!
//! // Check radar resolution parameters
//! let range_res = processor.range_resolution();
//! assert!(range_res > 0.0 && range_res < 1.0); // sub-meter for 1 GHz BW
//!
//! let vel_res = processor.velocity_resolution();
//! assert!(vel_res > 0.0);
//!
//! // Generate synthetic chirps (single target at ~30 m, 10 m/s)
//! let target_range = 30.0_f64;
//! let target_vel = 10.0_f64;
//! let samples_per_chirp = (config.chirp_duration_us * 1e-6 * config.sample_rate) as usize;
//! let num_chirps = config.num_chirps;
//! let c = 299_792_458.0_f64;
//! let slope = config.bandwidth_mhz * 1e6 / (config.chirp_duration_us * 1e-6);
//! let beat_freq = 2.0 * target_range * slope / c;
//! let doppler_freq = 2.0 * target_vel * config.center_freq_ghz * 1e9 / c;
//!
//! let mut chirps: Vec<Vec<(f64, f64)>> = Vec::new();
//! for chirp_idx in 0..num_chirps {
//!     let mut samples = Vec::with_capacity(samples_per_chirp);
//!     for i in 0..samples_per_chirp {
//!         let t = i as f64 / config.sample_rate;
//!         let phase = 2.0 * std::f64::consts::PI
//!             * (beat_freq * t + doppler_freq * chirp_idx as f64
//!                 * config.chirp_duration_us * 1e-6);
//!         samples.push((phase.cos() * 0.5, phase.sin() * 0.5));
//!     }
//!     chirps.push(samples);
//! }
//!
//! let detections = processor.process_frame(&chirps);
//! // Should detect at least one target
//! assert!(!detections.is_empty(), "expected at least one detection");
//! ```

use std::f64::consts::PI;

// ─── Configuration ───────────────────────────────────────────────────────────

/// Radar configuration parameters for an automotive FMCW radar.
#[derive(Debug, Clone, Copy)]
pub struct RadarConfig {
    /// Center frequency in GHz (typically 77).
    pub center_freq_ghz: f64,
    /// Sweep bandwidth in MHz (1000–4000).
    pub bandwidth_mhz: f64,
    /// Single chirp duration in microseconds.
    pub chirp_duration_us: f64,
    /// Number of chirps per frame (slow-time dimension).
    pub num_chirps: usize,
    /// ADC sample rate in Hz.
    pub sample_rate: f64,
    /// Maximum unambiguous range in meters.
    pub max_range_m: f64,
    /// CFAR guard cells on each side of the cell under test.
    pub cfar_guard_cells: usize,
    /// CFAR training cells on each side (beyond guard cells).
    pub cfar_training_cells: usize,
    /// CFAR detection threshold above noise estimate (dB).
    pub cfar_threshold_db: f64,
}

impl Default for RadarConfig {
    fn default() -> Self {
        Self {
            center_freq_ghz: 77.0,
            bandwidth_mhz: 1000.0,
            chirp_duration_us: 60.0,
            num_chirps: 64,
            sample_rate: 40_000_000.0,
            max_range_m: 200.0,
            cfar_guard_cells: 2,
            cfar_training_cells: 8,
            cfar_threshold_db: 12.0,
        }
    }
}

// ─── Detection and Classification ────────────────────────────────────────────

/// Classification label for a detected target.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TargetClass {
    /// Car, truck, or other motor vehicle (RCS typically > 0 dBsm).
    Vehicle,
    /// Pedestrian (RCS typically −10 to 0 dBsm, low velocity).
    Pedestrian,
    /// Bicycle or e-scooter rider.
    Cyclist,
    /// Static clutter or noise (very low velocity).
    Clutter,
    /// Unclassified target.
    Unknown,
}

/// A single target detection from one radar frame.
#[derive(Debug, Clone)]
pub struct TargetDetection {
    /// Estimated range in meters.
    pub range_m: f64,
    /// Estimated radial velocity in m/s (positive = approaching).
    pub velocity_mps: f64,
    /// Estimated azimuth angle in degrees (stub: always 0 for single-antenna).
    pub angle_deg: f64,
    /// Radar cross section estimate in dBsm.
    pub rcs_dbsm: f64,
    /// Rule-based classification.
    pub classification: TargetClass,
    /// Range bin index.
    pub range_bin: usize,
    /// Doppler bin index.
    pub doppler_bin: usize,
    /// Detection power (linear).
    pub power: f64,
}

// ─── Tracked Object ──────────────────────────────────────────────────────────

/// A tracked object maintained across multiple frames.
#[derive(Debug, Clone)]
pub struct TrackedObject {
    /// Unique track identifier.
    pub track_id: u64,
    /// Most recent range estimate in meters.
    pub position: f64,
    /// Most recent velocity estimate in m/s.
    pub velocity: f64,
    /// Number of consecutive frames this track has been maintained.
    pub age: u32,
    /// Current classification.
    pub class: TargetClass,
}

// ─── Processor ───────────────────────────────────────────────────────────────

/// FMCW automotive radar signal processor.
///
/// Holds radar configuration, internal state for tracking, and provides the full
/// processing chain from raw chirp samples to tracked target lists.
pub struct FmcwAutomotiveProcessor {
    config: RadarConfig,
    tracks: Vec<TrackedObject>,
    next_track_id: u64,
    /// Association gate distance in meters for nearest-neighbor tracking.
    pub association_gate_m: f64,
    /// Maximum number of frames a track can go without an update before it is dropped.
    pub max_coast_frames: u32,
}

impl FmcwAutomotiveProcessor {
    /// Speed of light in m/s.
    const C: f64 = 299_792_458.0;

    /// Create a new processor with the given radar configuration.
    pub fn new(config: RadarConfig) -> Self {
        Self {
            config,
            tracks: Vec::new(),
            next_track_id: 1,
            association_gate_m: 10.0,
            max_coast_frames: 5,
        }
    }

    /// Return a reference to the current radar configuration.
    pub fn config(&self) -> &RadarConfig {
        &self.config
    }

    // ── Resolution helpers ───────────────────────────────────────────────

    /// Range resolution: `c / (2 * B)` where B is sweep bandwidth in Hz.
    pub fn range_resolution(&self) -> f64 {
        Self::C / (2.0 * self.config.bandwidth_mhz * 1e6)
    }

    /// Velocity resolution: `lambda / (2 * T_frame)` where T_frame = num_chirps * chirp_duration.
    pub fn velocity_resolution(&self) -> f64 {
        let lambda = Self::C / (self.config.center_freq_ghz * 1e9);
        let t_frame = self.config.num_chirps as f64 * self.config.chirp_duration_us * 1e-6;
        lambda / (2.0 * t_frame)
    }

    /// Maximum unambiguous velocity: `lambda / (4 * T_chirp)`.
    pub fn max_unambiguous_velocity(&self) -> f64 {
        let lambda = Self::C / (self.config.center_freq_ghz * 1e9);
        lambda / (4.0 * self.config.chirp_duration_us * 1e-6)
    }

    // ── Range FFT ────────────────────────────────────────────────────────

    /// Compute the range FFT (fast-time FFT) for a single chirp.
    ///
    /// Input: slice of complex IQ samples `(I, Q)` for one chirp.
    /// Output: power spectrum (magnitude squared) with length = next power of 2.
    pub fn range_fft(samples: &[(f64, f64)]) -> Vec<f64> {
        let n = samples.len().next_power_of_two();
        let mut re = vec![0.0; n];
        let mut im = vec![0.0; n];

        // Apply Hanning window and zero-pad
        for (i, &(r, q)) in samples.iter().enumerate() {
            let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / samples.len() as f64).cos());
            re[i] = r * w;
            im[i] = q * w;
        }

        fft_in_place(&mut re, &mut im, false);

        // Return magnitude squared (power)
        re.iter()
            .zip(im.iter())
            .map(|(&r, &i)| r * r + i * i)
            .collect()
    }

    /// Compute the range FFT for every chirp in a frame (complex output).
    ///
    /// Returns a 2-D array: `range_map[chirp_idx][range_bin]` as `(re, im)`.
    pub fn range_fft_frame(chirps: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
        chirps
            .iter()
            .map(|chirp| {
                let n = chirp.len().next_power_of_two();
                let mut re = vec![0.0; n];
                let mut im = vec![0.0; n];
                for (i, &(r, q)) in chirp.iter().enumerate() {
                    let w =
                        0.5 * (1.0 - (2.0 * PI * i as f64 / chirp.len() as f64).cos());
                    re[i] = r * w;
                    im[i] = q * w;
                }
                fft_in_place(&mut re, &mut im, false);
                re.into_iter().zip(im).collect()
            })
            .collect()
    }

    // ── Doppler FFT ──────────────────────────────────────────────────────

    /// Compute the Doppler FFT across chirps for each range bin.
    ///
    /// Input: `range_map[chirp_idx][range_bin]` (complex values from range FFT).
    /// Output: `rd_map[range_bin][doppler_bin]` power spectrum.
    pub fn doppler_fft(range_map: &[Vec<(f64, f64)>]) -> Vec<Vec<f64>> {
        if range_map.is_empty() {
            return Vec::new();
        }
        let num_range_bins = range_map[0].len();
        let num_chirps = range_map.len();
        let n_doppler = num_chirps.next_power_of_two();

        let mut rd_map = Vec::with_capacity(num_range_bins);

        for rbin in 0..num_range_bins {
            let mut re = vec![0.0; n_doppler];
            let mut im = vec![0.0; n_doppler];
            for (c, row) in range_map.iter().enumerate() {
                if rbin < row.len() {
                    let w = 0.5
                        * (1.0 - (2.0 * PI * c as f64 / num_chirps as f64).cos());
                    re[c] = row[rbin].0 * w;
                    im[c] = row[rbin].1 * w;
                }
            }
            fft_in_place(&mut re, &mut im, false);

            // FFT-shift so zero Doppler is at center
            let power: Vec<f64> = re
                .iter()
                .zip(im.iter())
                .map(|(&r, &i)| r * r + i * i)
                .collect();
            let half = power.len() / 2;
            let mut shifted = Vec::with_capacity(power.len());
            shifted.extend_from_slice(&power[half..]);
            shifted.extend_from_slice(&power[..half]);
            rd_map.push(shifted);
        }
        rd_map
    }

    // ── CFAR Detection ───────────────────────────────────────────────────

    /// Cell-Averaging CFAR detector on a 1-D power vector.
    ///
    /// Returns indices of cells that exceed the adaptive threshold.
    pub fn cfar_1d(
        power: &[f64],
        guard_cells: usize,
        training_cells: usize,
        threshold_db: f64,
    ) -> Vec<usize> {
        let n = power.len();
        let alpha = 10.0_f64.powf(threshold_db / 10.0);
        let window = guard_cells + training_cells;
        let mut detections = Vec::new();

        if n <= 2 * window {
            return detections;
        }

        for i in window..(n - window) {
            let mut sum = 0.0;
            let mut count = 0u32;
            for j in (i - window)..(i - guard_cells) {
                sum += power[j];
                count += 1;
            }
            for j in (i + guard_cells + 1)..=(i + window) {
                if j < n {
                    sum += power[j];
                    count += 1;
                }
            }
            if count == 0 {
                continue;
            }
            let noise_est = sum / count as f64;
            if power[i] > alpha * noise_est {
                detections.push(i);
            }
        }
        detections
    }

    /// 2-D CFAR over the range-Doppler map.
    ///
    /// Returns `(range_bin, doppler_bin, power)` tuples for each detection.
    pub fn cfar_2d(
        rd_map: &[Vec<f64>],
        guard_cells: usize,
        training_cells: usize,
        threshold_db: f64,
    ) -> Vec<(usize, usize, f64)> {
        let alpha = 10.0_f64.powf(threshold_db / 10.0);
        let window = guard_cells + training_cells;
        let num_range = rd_map.len();
        if num_range == 0 {
            return Vec::new();
        }
        let num_doppler = rd_map[0].len();
        let mut detections = Vec::new();

        for r in window..(num_range.saturating_sub(window)) {
            for d in window..(num_doppler.saturating_sub(window)) {
                let mut sum = 0.0;
                let mut count = 0u32;
                for rr in (r - window)..=(r + window) {
                    for dd in (d - window)..=(d + window) {
                        // Skip guard region and CUT
                        if rr.abs_diff(r) <= guard_cells && dd.abs_diff(d) <= guard_cells {
                            continue;
                        }
                        if rr < num_range && dd < num_doppler {
                            sum += rd_map[rr][dd];
                            count += 1;
                        }
                    }
                }
                if count == 0 {
                    continue;
                }
                let noise_est = sum / count as f64;
                let power = rd_map[r][d];
                if power > alpha * noise_est {
                    detections.push((r, d, power));
                }
            }
        }
        detections
    }

    // ── Classification ───────────────────────────────────────────────────

    /// Rule-based target classification from RCS and velocity.
    ///
    /// | Class      | RCS (dBsm) | Velocity (m/s) |
    /// |------------|------------|----------------|
    /// | Vehicle    | > 0        | > 2            |
    /// | Pedestrian | −15 to 5   | 0.3 to 3       |
    /// | Cyclist    | −10 to 5   | 2 to 15        |
    /// | Clutter    | any        | < 0.3          |
    /// | Unknown    | fallthrough|                |
    pub fn classify_target(rcs_dbsm: f64, velocity_abs: f64) -> TargetClass {
        if velocity_abs < 0.3 {
            return TargetClass::Clutter;
        }
        if rcs_dbsm > 0.0 && velocity_abs > 2.0 {
            return TargetClass::Vehicle;
        }
        if rcs_dbsm >= -15.0 && rcs_dbsm <= 5.0 && velocity_abs >= 0.3 && velocity_abs <= 3.0 {
            return TargetClass::Pedestrian;
        }
        if rcs_dbsm >= -10.0 && rcs_dbsm <= 5.0 && velocity_abs >= 2.0 && velocity_abs <= 15.0 {
            return TargetClass::Cyclist;
        }
        TargetClass::Unknown
    }

    // ── Full Frame Processing ────────────────────────────────────────────

    /// Process a complete radar frame (all chirps) and return target detections.
    ///
    /// `chirps` is indexed as `chirps[chirp_index][sample_index]` where each
    /// sample is an `(I, Q)` tuple.
    pub fn process_frame(&self, chirps: &[Vec<(f64, f64)>]) -> Vec<TargetDetection> {
        if chirps.is_empty() || chirps[0].is_empty() {
            return Vec::new();
        }

        // Step 1: Range FFT per chirp (complex output)
        let range_map = Self::range_fft_frame(chirps);

        // Step 2: Doppler FFT across chirps
        let rd_map = Self::doppler_fft(&range_map);

        // Step 3: CFAR detection on the range-Doppler map
        let raw_detections = Self::cfar_2d(
            &rd_map,
            self.config.cfar_guard_cells,
            self.config.cfar_training_cells,
            self.config.cfar_threshold_db,
        );

        // Step 4: Convert bin indices to physical units and classify
        let num_range_bins = rd_map.len();
        let num_doppler_bins = if num_range_bins > 0 { rd_map[0].len() } else { 0 };

        // Beat-frequency bin spacing = fs / N_fft_range.
        // Range = f_beat * c * T_chirp / (2 * B), so:
        //   range_per_bin = (fs / N_fft) * c * T_chirp / (2 * B)
        let n_fft_range = num_range_bins as f64;
        let slope = self.config.bandwidth_mhz * 1e6 / (self.config.chirp_duration_us * 1e-6);
        let range_per_bin = (self.config.sample_rate / n_fft_range) * Self::C / (2.0 * slope);

        let lambda = Self::C / (self.config.center_freq_ghz * 1e9);
        // Doppler bin spacing = fs_slow / N_fft_doppler where fs_slow = 1/T_chirp
        let doppler_per_bin =
            1.0 / (num_doppler_bins as f64 * self.config.chirp_duration_us * 1e-6);
        let vel_per_bin = lambda * doppler_per_bin / 2.0;

        raw_detections
            .into_iter()
            .filter_map(|(rbin, dbin, power)| {
                let range_m = rbin as f64 * range_per_bin;
                if range_m > self.config.max_range_m {
                    return None;
                }

                // Doppler is FFT-shifted: center = zero Doppler
                let doppler_signed = dbin as f64 - (num_doppler_bins as f64 / 2.0);
                let velocity_mps = doppler_signed * vel_per_bin;

                // Rough RCS estimate (power falls off as R^4, use relative dB)
                let rcs_dbsm = if range_m > 1.0 {
                    10.0 * power.log10() + 40.0 * range_m.log10() - 30.0
                } else {
                    10.0 * power.log10()
                };

                let classification =
                    Self::classify_target(rcs_dbsm, velocity_mps.abs());

                Some(TargetDetection {
                    range_m,
                    velocity_mps,
                    angle_deg: 0.0, // single-antenna: no angle estimation
                    rcs_dbsm,
                    classification,
                    range_bin: rbin,
                    doppler_bin: dbin,
                    power,
                })
            })
            .collect()
    }

    // ── Tracking ─────────────────────────────────────────────────────────

    /// Simple nearest-neighbor tracking across frames.
    ///
    /// Associates new detections with existing tracks using range distance.
    /// Creates new tracks for unassociated detections and drops stale tracks.
    /// Returns the updated list of tracked objects.
    pub fn track_targets(&mut self, detections: &[TargetDetection]) -> &[TrackedObject] {
        let mut used = vec![false; detections.len()];

        // Try to associate each existing track with the closest detection
        for track in &mut self.tracks {
            let mut best_idx: Option<usize> = None;
            let mut best_dist = f64::MAX;
            for (i, det) in detections.iter().enumerate() {
                if used[i] {
                    continue;
                }
                let dist = (track.position - det.range_m).abs();
                if dist < best_dist && dist < self.association_gate_m {
                    best_dist = dist;
                    best_idx = Some(i);
                }
            }
            if let Some(idx) = best_idx {
                used[idx] = true;
                track.position = detections[idx].range_m;
                track.velocity = detections[idx].velocity_mps;
                track.class = detections[idx].classification;
                track.age += 1;
            } else {
                // Coast: track not updated this frame
                track.age = track.age.saturating_sub(1);
            }
        }

        // Remove stale tracks
        self.tracks.retain(|t| t.age > 0);

        // Create new tracks for unassociated detections
        for (i, det) in detections.iter().enumerate() {
            if !used[i] {
                self.tracks.push(TrackedObject {
                    track_id: self.next_track_id,
                    position: det.range_m,
                    velocity: det.velocity_mps,
                    age: 1,
                    class: det.classification,
                });
                self.next_track_id += 1;
            }
        }

        &self.tracks
    }

    /// Return a snapshot of currently tracked objects.
    pub fn active_tracks(&self) -> &[TrackedObject] {
        &self.tracks
    }

    /// Reset all tracks (e.g., on mode change).
    pub fn reset_tracks(&mut self) {
        self.tracks.clear();
        self.next_track_id = 1;
    }
}

// ─── Radix-2 DIT FFT (std-only) ─────────────────────────────────────────────

/// In-place Cooley-Tukey radix-2 DIT FFT.
/// `inverse = true` performs the inverse FFT (with 1/N scaling).
fn fft_in_place(re: &mut [f64], im: &mut [f64], inverse: bool) {
    let n = re.len();
    assert!(n.is_power_of_two(), "FFT length must be a power of two");
    assert_eq!(re.len(), im.len());

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
            re.swap(i, j);
            im.swap(i, j);
        }
    }

    // Butterfly stages
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let wn_re = angle.cos();
        let wn_im = angle.sin();

        let mut start = 0;
        while start < n {
            let mut w_re = 1.0;
            let mut w_im = 0.0;
            for k in 0..half {
                let even = start + k;
                let odd = start + k + half;
                let t_re = w_re * re[odd] - w_im * im[odd];
                let t_im = w_re * im[odd] + w_im * re[odd];
                re[odd] = re[even] - t_re;
                im[odd] = im[even] - t_im;
                re[even] += t_re;
                im[even] += t_im;
                let new_w_re = w_re * wn_re - w_im * wn_im;
                let new_w_im = w_re * wn_im + w_im * wn_re;
                w_re = new_w_re;
                w_im = new_w_im;
            }
            start += len;
        }
        len *= 2;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for i in 0..n {
            re[i] *= inv_n;
            im[i] *= inv_n;
        }
    }
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_processor() -> FmcwAutomotiveProcessor {
        FmcwAutomotiveProcessor::new(RadarConfig::default())
    }

    /// Helper: create a config and synthetic chirps for a target at given range/velocity.
    fn make_synthetic_frame(
        config: &RadarConfig,
        target_range: f64,
        target_vel: f64,
    ) -> Vec<Vec<(f64, f64)>> {
        let samples_per_chirp =
            (config.chirp_duration_us * 1e-6 * config.sample_rate) as usize;
        let slope = config.bandwidth_mhz * 1e6 / (config.chirp_duration_us * 1e-6);
        let beat_freq = 2.0 * target_range * slope / FmcwAutomotiveProcessor::C;
        let doppler_freq =
            2.0 * target_vel * config.center_freq_ghz * 1e9 / FmcwAutomotiveProcessor::C;

        (0..config.num_chirps)
            .map(|ci| {
                (0..samples_per_chirp)
                    .map(|i| {
                        let t = i as f64 / config.sample_rate;
                        let phase = 2.0 * PI
                            * (beat_freq * t
                                + doppler_freq * ci as f64 * config.chirp_duration_us * 1e-6);
                        (phase.cos() * 0.5, phase.sin() * 0.5)
                    })
                    .collect()
            })
            .collect()
    }

    // --- Test 1: range resolution ---
    #[test]
    fn test_range_resolution() {
        let proc = default_processor();
        // B = 1000 MHz → range_res = c/(2*1e9) ≈ 0.15 m
        let res = proc.range_resolution();
        assert!((res - 0.1499).abs() < 0.001, "range res = {res}");
    }

    // --- Test 2: velocity resolution ---
    #[test]
    fn test_velocity_resolution() {
        let proc = default_processor();
        // lambda = c / 77e9 ≈ 3.893 mm, T_frame = 64 * 60e-6 = 3.84 ms
        // vel_res = lambda / (2 * T_frame) ≈ 0.507 m/s
        let res = proc.velocity_resolution();
        assert!(res > 0.3 && res < 1.0, "velocity res = {res}");
    }

    // --- Test 3: max unambiguous velocity ---
    #[test]
    fn test_max_unambiguous_velocity() {
        let proc = default_processor();
        // lambda / (4 * 60e-6) ≈ 16.2 m/s
        let v = proc.max_unambiguous_velocity();
        assert!(v > 10.0 && v < 25.0, "max unamb vel = {v}");
    }

    // --- Test 4: range FFT on a pure tone ---
    #[test]
    fn test_range_fft_pure_tone() {
        let n = 256;
        let fs = 40_000_000.0;
        let f_beat = 2_000_000.0; // 2 MHz beat frequency
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                let phase = 2.0 * PI * f_beat * t;
                (phase.cos(), phase.sin())
            })
            .collect();

        let power = FmcwAutomotiveProcessor::range_fft(&samples);
        assert_eq!(power.len(), 256); // 256 is already pow2

        // Peak should be near bin = f_beat / (fs/N) = 2e6 / (40e6/256) ≈ 12.8
        let peak_bin = power
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert!(
            (peak_bin as i32 - 13).unsigned_abs() <= 2,
            "peak at bin {peak_bin}, expected ~13"
        );
    }

    // --- Test 5: doppler FFT produces correct dimensions ---
    #[test]
    fn test_doppler_fft_dimensions() {
        // 16 chirps, 32 samples each → range bins = 32, doppler bins = 16
        let range_map: Vec<Vec<(f64, f64)>> = (0..16)
            .map(|_| (0..32).map(|_| (1.0, 0.0)).collect())
            .collect();

        let rd = FmcwAutomotiveProcessor::doppler_fft(&range_map);
        assert_eq!(rd.len(), 32, "should have 32 range bins");
        assert_eq!(rd[0].len(), 16, "should have 16 doppler bins");
    }

    // --- Test 6: CFAR 1-D detection ---
    #[test]
    fn test_cfar_1d_single_peak() {
        let mut power = vec![1.0; 64];
        power[30] = 100.0; // Strong target at bin 30

        let dets = FmcwAutomotiveProcessor::cfar_1d(&power, 2, 4, 10.0);
        assert!(
            dets.contains(&30),
            "CFAR should detect bin 30, got: {dets:?}"
        );
    }

    // --- Test 7: CFAR with no targets ---
    #[test]
    fn test_cfar_1d_no_targets() {
        let power = vec![1.0; 64]; // Uniform noise, no peaks
        let dets = FmcwAutomotiveProcessor::cfar_1d(&power, 2, 4, 6.0);
        assert!(dets.is_empty(), "uniform noise should yield no detections");
    }

    // --- Test 8: target classification ---
    #[test]
    fn test_classify_target_rules() {
        assert_eq!(
            FmcwAutomotiveProcessor::classify_target(10.0, 20.0),
            TargetClass::Vehicle
        );
        assert_eq!(
            FmcwAutomotiveProcessor::classify_target(-5.0, 1.5),
            TargetClass::Pedestrian
        );
        assert_eq!(
            FmcwAutomotiveProcessor::classify_target(-3.0, 8.0),
            TargetClass::Cyclist
        );
        assert_eq!(
            FmcwAutomotiveProcessor::classify_target(5.0, 0.1),
            TargetClass::Clutter
        );
        // Extreme values that don't match any rule
        assert_eq!(
            FmcwAutomotiveProcessor::classify_target(-20.0, 50.0),
            TargetClass::Unknown
        );
    }

    // --- Test 9: nearest-neighbor tracking creates and maintains tracks ---
    #[test]
    fn test_track_targets_basic() {
        let mut proc = default_processor();

        let det1 = TargetDetection {
            range_m: 50.0,
            velocity_mps: 10.0,
            angle_deg: 0.0,
            rcs_dbsm: 5.0,
            classification: TargetClass::Vehicle,
            range_bin: 0,
            doppler_bin: 0,
            power: 1.0,
        };

        // Frame 1: one detection → one new track
        let tracks = proc.track_targets(&[det1.clone()]);
        assert_eq!(tracks.len(), 1);
        assert_eq!(tracks[0].track_id, 1);
        assert!((tracks[0].position - 50.0).abs() < 1e-9);

        // Frame 2: same detection slightly moved → should associate
        let det2 = TargetDetection {
            range_m: 51.0,
            ..det1.clone()
        };
        let tracks = proc.track_targets(&[det2]);
        assert_eq!(tracks.len(), 1);
        assert!((tracks[0].position - 51.0).abs() < 1e-9);
        assert_eq!(tracks[0].age, 2);
    }

    // --- Test 10: stale tracks are dropped ---
    #[test]
    fn test_track_drops_stale() {
        let mut proc = default_processor();

        let det = TargetDetection {
            range_m: 50.0,
            velocity_mps: 10.0,
            angle_deg: 0.0,
            rcs_dbsm: 5.0,
            classification: TargetClass::Vehicle,
            range_bin: 0,
            doppler_bin: 0,
            power: 1.0,
        };

        // Create a track
        proc.track_targets(&[det]);
        assert_eq!(proc.active_tracks().len(), 1);

        // No detections → track coasts (age decrements, then removed at 0)
        proc.track_targets(&[]);
        assert!(
            proc.active_tracks().is_empty(),
            "track with age 1 should be removed after one miss"
        );
    }

    // --- Test 11: full process_frame with synthetic target ---
    #[test]
    fn test_process_frame_synthetic_target() {
        let config = RadarConfig {
            cfar_threshold_db: 10.0,
            ..RadarConfig::default()
        };
        let proc = FmcwAutomotiveProcessor::new(config);

        let target_range = 30.0; // meters
        let chirps = make_synthetic_frame(&config, target_range, 5.0);

        let detections = proc.process_frame(&chirps);
        assert!(
            !detections.is_empty(),
            "should detect the synthetic target"
        );

        // The closest detection to target_range should be reasonably accurate
        let best = detections
            .iter()
            .min_by(|a, b| {
                (a.range_m - target_range)
                    .abs()
                    .partial_cmp(&(b.range_m - target_range).abs())
                    .unwrap()
            })
            .unwrap();
        assert!(
            (best.range_m - target_range).abs() < 5.0,
            "range {:.1} m, expected ~{target_range} m",
            best.range_m
        );
    }

    // --- Test 12: FFT round-trip (forward then inverse) ---
    #[test]
    fn test_fft_roundtrip() {
        let n = 16;
        let mut re: Vec<f64> = (0..n).map(|i| (i as f64).sin()).collect();
        let mut im = vec![0.0; n];
        let orig_re = re.clone();

        fft_in_place(&mut re, &mut im, false);
        fft_in_place(&mut re, &mut im, true);

        for i in 0..n {
            assert!(
                (re[i] - orig_re[i]).abs() < 1e-10,
                "FFT roundtrip mismatch at {i}"
            );
            assert!(im[i].abs() < 1e-10);
        }
    }

    // --- Test 13: reset tracks clears state ---
    #[test]
    fn test_reset_tracks() {
        let mut proc = default_processor();
        let det = TargetDetection {
            range_m: 30.0,
            velocity_mps: 5.0,
            angle_deg: 0.0,
            rcs_dbsm: 2.0,
            classification: TargetClass::Vehicle,
            range_bin: 0,
            doppler_bin: 0,
            power: 1.0,
        };
        proc.track_targets(&[det]);
        assert!(!proc.active_tracks().is_empty());
        proc.reset_tracks();
        assert!(proc.active_tracks().is_empty());
    }
}
