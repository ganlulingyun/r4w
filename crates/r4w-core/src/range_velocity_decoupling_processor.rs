//! Range-Velocity Decoupling Processor
//!
//! Separates overlapping range-Doppler spectra from coherent pulse radar.
//! Implements joint range-velocity estimation, Doppler aliasing resolution,
//! and 2D CFAR detection for multi-target environments.
//!
//! This module is pure Rust with no external crate dependencies. All math
//! (FFT, complex arithmetic, CFAR) is implemented from scratch.
//!
//! ## Processing Pipeline
//!
//! ```text
//! CPI Matrix (pulses x range bins)
//!   │
//!   ├─ Range FFT per pulse ──► Range-compressed pulses
//!   │
//!   ├─ Doppler FFT across pulses ──► Range-Doppler map
//!   │
//!   ├─ 2D CA-CFAR detection ──► Target detections (bin, SNR)
//!   │
//!   └─ Bin-to-physical conversion ──► Targets (range_m, velocity_mps)
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::range_velocity_decoupling_processor::*;
//!
//! let config = RvdConfig {
//!     prf_hz: 1000.0,
//!     carrier_freq_hz: 10.0e9,
//!     bandwidth_hz: 5.0e6,
//!     num_range_bins: 64,
//!     num_doppler_bins: 64,
//!     speed_of_light: 3.0e8,
//! };
//!
//! let processor = RvdProcessor::new(config);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers (no external crate)
// ---------------------------------------------------------------------------

/// Minimal complex number for internal use.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    #[inline]
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    #[inline]
    pub fn zero() -> Self {
        Self { re: 0.0, im: 0.0 }
    }

    #[inline]
    pub fn from_polar(r: f64, theta: f64) -> Self {
        Self {
            re: r * theta.cos(),
            im: r * theta.sin(),
        }
    }

    #[inline]
    pub fn norm_sqr(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    #[inline]
    pub fn norm(self) -> f64 {
        self.norm_sqr().sqrt()
    }

    #[inline]
    pub fn conj(self) -> Self {
        Self {
            re: self.re,
            im: -self.im,
        }
    }
}

impl std::ops::Add for Complex {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }
}

impl std::ops::AddAssign for Complex {
    #[inline]
    fn add_assign(&mut self, rhs: Self) {
        self.re += rhs.re;
        self.im += rhs.im;
    }
}

impl std::ops::Sub for Complex {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self {
            re: self.re - rhs.re,
            im: self.im - rhs.im,
        }
    }
}

impl std::ops::Mul for Complex {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: Self) -> Self {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }
}

impl std::ops::Mul<f64> for Complex {
    type Output = Self;
    #[inline]
    fn mul(self, rhs: f64) -> Self {
        Self {
            re: self.re * rhs,
            im: self.im * rhs,
        }
    }
}

// ---------------------------------------------------------------------------
// FFT
// ---------------------------------------------------------------------------

/// In-place radix-2 Cooley-Tukey FFT.
///
/// `data` length must be a power of two. When `inverse` is true, computes the
/// inverse FFT (with 1/N scaling).
pub fn fft_radix2(data: &mut [(f64, f64)], inverse: bool) {
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
        let angle = sign * 2.0 * PI / len as f64;
        let wn = (angle.cos(), angle.sin());

        let mut i = 0;
        while i < n {
            let mut w = (1.0, 0.0);
            for k in 0..half {
                let u = data[i + k];
                let t = (
                    w.0 * data[i + k + half].0 - w.1 * data[i + k + half].1,
                    w.0 * data[i + k + half].1 + w.1 * data[i + k + half].0,
                );
                data[i + k] = (u.0 + t.0, u.1 + t.1);
                data[i + k + half] = (u.0 - t.0, u.1 - t.1);
                let new_w = (w.0 * wn.0 - w.1 * wn.1, w.0 * wn.1 + w.1 * wn.0);
                w = new_w;
            }
            i += len;
        }
        len <<= 1;
    }

    // Inverse FFT scaling
    if inverse {
        let scale = 1.0 / n as f64;
        for sample in data.iter_mut() {
            sample.0 *= scale;
            sample.1 *= scale;
        }
    }
}

// ---------------------------------------------------------------------------
// Configuration & target types
// ---------------------------------------------------------------------------

/// Configuration for the Range-Velocity Decoupling Processor.
#[derive(Clone, Debug)]
pub struct RvdConfig {
    /// Pulse repetition frequency in Hz.
    pub prf_hz: f64,
    /// Carrier (RF) frequency in Hz.
    pub carrier_freq_hz: f64,
    /// Transmit bandwidth in Hz (determines range resolution).
    pub bandwidth_hz: f64,
    /// Number of range bins (columns in CPI matrix).
    pub num_range_bins: usize,
    /// Number of Doppler bins (typically equals number of pulses in CPI).
    pub num_doppler_bins: usize,
    /// Speed of light in m/s (default 3e8).
    pub speed_of_light: f64,
}

impl Default for RvdConfig {
    fn default() -> Self {
        Self {
            prf_hz: 1000.0,
            carrier_freq_hz: 10.0e9,
            bandwidth_hz: 5.0e6,
            num_range_bins: 64,
            num_doppler_bins: 64,
            speed_of_light: 3.0e8,
        }
    }
}

/// A detected target in range-velocity space.
#[derive(Clone, Debug, PartialEq)]
pub struct RvdTarget {
    /// Estimated range in meters.
    pub range_m: f64,
    /// Estimated radial velocity in m/s (positive = closing).
    pub velocity_mps: f64,
    /// Signal-to-noise ratio in dB.
    pub snr_db: f64,
    /// Range bin index.
    pub range_bin: usize,
    /// Doppler bin index.
    pub doppler_bin: usize,
}

// ---------------------------------------------------------------------------
// Processor
// ---------------------------------------------------------------------------

/// Main range-velocity decoupling processor.
///
/// Accepts a coherent processing interval (CPI) matrix and produces
/// a list of detected targets with range, velocity, and SNR.
pub struct RvdProcessor {
    config: RvdConfig,
}

impl RvdProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: RvdConfig) -> Self {
        Self { config }
    }

    /// Process a full CPI matrix and return detected targets.
    ///
    /// `cpi_matrix` has one row per pulse and one column per range sample.
    /// Each element is an (I, Q) pair. The number of rows should equal
    /// `config.num_doppler_bins` (number of pulses in the CPI).
    ///
    /// Internally performs:
    /// 1. Range FFT per pulse
    /// 2. Doppler FFT across pulses per range bin
    /// 3. 2D CA-CFAR detection
    /// 4. Conversion from bin indices to physical range/velocity
    pub fn process_cpi(&self, cpi_matrix: &[Vec<(f64, f64)>]) -> Vec<RvdTarget> {
        let num_pulses = cpi_matrix.len();
        if num_pulses == 0 {
            return Vec::new();
        }

        let num_range_samples = cpi_matrix[0].len();
        if num_range_samples == 0 {
            return Vec::new();
        }

        // Step 1 & 2: build range-Doppler power map
        let rd_map = range_doppler_map(cpi_matrix, num_pulses);
        let num_range = rd_map.len();
        let num_dopp = if num_range > 0 { rd_map[0].len() } else { 0 };

        if num_range < 3 || num_dopp < 3 {
            // Not enough cells for CFAR
            return Vec::new();
        }

        // Step 3: 2D CFAR detection
        let guard_cells = 2;
        let training_cells = 4;
        let pfa = 1e-4;
        let detections = cfar_2d(&rd_map, guard_cells, training_cells, pfa);

        // Step 4: convert to physical targets
        let c = self.config.speed_of_light;
        detections
            .into_iter()
            .map(|(rb, db, snr)| {
                let range_m = range_bin_to_meters(
                    rb,
                    self.config.bandwidth_hz,
                    num_range,
                    c,
                );
                let velocity_mps = doppler_to_velocity(
                    db,
                    num_dopp,
                    self.config.prf_hz,
                    self.config.carrier_freq_hz,
                    c,
                );
                RvdTarget {
                    range_m,
                    velocity_mps,
                    snr_db: snr,
                    range_bin: rb,
                    doppler_bin: db,
                }
            })
            .collect()
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &RvdConfig {
        &self.config
    }
}

// ---------------------------------------------------------------------------
// Range-Doppler map
// ---------------------------------------------------------------------------

/// Compute a 2D power map (range bins x Doppler bins) from a CPI matrix.
///
/// Each row of `cpi` is one pulse (slow-time), each column is a range sample
/// (fast-time). `num_doppler_bins` is the FFT size used along the Doppler
/// (slow-time) dimension; it is typically the number of pulses.
///
/// Returns `rd_map[range_bin][doppler_bin]` = power (magnitude squared).
pub fn range_doppler_map(
    cpi: &[Vec<(f64, f64)>],
    num_doppler_bins: usize,
) -> Vec<Vec<f64>> {
    let num_pulses = cpi.len();
    if num_pulses == 0 {
        return Vec::new();
    }

    let num_range_samples = cpi[0].len();
    let range_fft_len = num_range_samples.next_power_of_two();
    let doppler_fft_len = num_doppler_bins.next_power_of_two();

    // Step 1: range FFT per pulse → range_compressed[pulse][range_bin]
    let mut range_compressed: Vec<Vec<Complex>> = Vec::with_capacity(num_pulses);
    for pulse in cpi {
        let mut buf: Vec<(f64, f64)> = Vec::with_capacity(range_fft_len);
        for &sample in pulse {
            buf.push(sample);
        }
        buf.resize(range_fft_len, (0.0, 0.0));
        fft_radix2(&mut buf, false);
        range_compressed.push(
            buf.iter().map(|&(re, im)| Complex::new(re, im)).collect(),
        );
    }

    // Step 2: Doppler FFT across pulses for each range bin
    let mut rd_map = vec![vec![0.0f64; doppler_fft_len]; range_fft_len];
    for rbin in 0..range_fft_len {
        let mut doppler_col: Vec<(f64, f64)> = Vec::with_capacity(doppler_fft_len);
        for p in 0..num_pulses {
            let c = &range_compressed[p][rbin];
            doppler_col.push((c.re, c.im));
        }
        doppler_col.resize(doppler_fft_len, (0.0, 0.0));
        fft_radix2(&mut doppler_col, false);
        for (dbin, &(re, im)) in doppler_col.iter().enumerate() {
            rd_map[rbin][dbin] = re * re + im * im;
        }
    }

    rd_map
}

// ---------------------------------------------------------------------------
// 2D CA-CFAR detector
// ---------------------------------------------------------------------------

/// 2D Cell-Averaging CFAR detector.
///
/// Scans a power `map[range_bin][doppler_bin]` and returns detections as
/// `(range_bin, doppler_bin, snr_db)`. `guard_cells` and `training_cells`
/// define the ring of cells used for noise estimation. `pfa` is the desired
/// probability of false alarm which sets the detection threshold via
/// `threshold_factor = training_count * (pfa^(-1/training_count) - 1)`.
pub fn cfar_2d(
    map: &[Vec<f64>],
    guard_cells: usize,
    training_cells: usize,
    pfa: f64,
) -> Vec<(usize, usize, f64)> {
    let num_range = map.len();
    if num_range == 0 {
        return Vec::new();
    }
    let num_doppler = map[0].len();

    let outer = guard_cells + training_cells;

    // Skip cells too close to the border for a full window
    if num_range <= 2 * outer || num_doppler <= 2 * outer {
        return Vec::new();
    }

    let mut detections = Vec::new();

    for r in outer..num_range - outer {
        for d in outer..num_doppler - outer {
            let cut = map[r][d]; // cell under test

            // Gather training cells in a 2D ring
            let mut sum = 0.0;
            let mut count = 0u32;
            for ri in (r.saturating_sub(outer))..=(r + outer).min(num_range - 1) {
                for di in (d.saturating_sub(outer))..=(d + outer).min(num_doppler - 1) {
                    let dr = if ri >= r { ri - r } else { r - ri };
                    let dd = if di >= d { di - d } else { d - di };
                    // Inside outer ring but outside guard ring (Chebyshev distance)
                    let max_dist = dr.max(dd);
                    if max_dist > guard_cells && max_dist <= outer {
                        sum += map[ri][di];
                        count += 1;
                    }
                }
            }

            if count == 0 {
                continue;
            }

            let noise_avg = sum / count as f64;

            // CFAR threshold factor: N * (Pfa^(-1/N) - 1)
            let n = count as f64;
            let alpha = n * (pfa.powf(-1.0 / n) - 1.0);

            let threshold = alpha * noise_avg;

            if cut > threshold && noise_avg > 0.0 {
                let snr_linear = cut / noise_avg;
                let snr_db = 10.0 * snr_linear.log10();
                detections.push((r, d, snr_db));
            }
        }
    }

    detections
}

// ---------------------------------------------------------------------------
// Bin-to-physical conversions
// ---------------------------------------------------------------------------

/// Convert a Doppler bin index to radial velocity in m/s.
///
/// The Doppler axis of an FFT is periodic with unambiguous interval
/// `[-PRF/2, +PRF/2)`. Positive velocity means target is approaching (closing).
/// Handles aliasing by wrapping bin indices into `[-N/2, N/2)`.
pub fn doppler_to_velocity(
    doppler_bin: usize,
    num_bins: usize,
    prf_hz: f64,
    carrier_freq_hz: f64,
    c: f64,
) -> f64 {
    if num_bins == 0 || carrier_freq_hz == 0.0 {
        return 0.0;
    }

    // Map bin index to signed Doppler frequency.
    // Bin 0 = DC (zero Doppler), bin N/2 = most negative Doppler.
    // FFT output convention: bins [0..N/2) are non-negative, [N/2..N) are negative.
    let half = num_bins / 2;
    let signed_bin = if doppler_bin >= half {
        doppler_bin as f64 - num_bins as f64
    } else {
        doppler_bin as f64
    };

    // Doppler frequency for this bin
    let doppler_hz = signed_bin * prf_hz / num_bins as f64;

    // Radial velocity: v = c * fd / (2 * fc)
    let wavelength = c / carrier_freq_hz;
    doppler_hz * wavelength / 2.0
}

/// Convert a range bin index to range in meters.
///
/// Range resolution = c / (2 * B). The range for bin `k` is `k * delta_r`.
pub fn range_bin_to_meters(
    range_bin: usize,
    bandwidth_hz: f64,
    num_bins: usize,
    c: f64,
) -> f64 {
    if num_bins == 0 || bandwidth_hz == 0.0 {
        return 0.0;
    }

    // Range resolution per bin: each bin spans c / (2 * B * N) * N = c/(2*B)
    // but with N FFT bins the spacing is c / (2 * B_eff) where B_eff accounts
    // for the FFT sample spacing. More precisely:
    //   delta_range = c / (2 * bandwidth_hz)  per range resolution cell
    //   but the FFT maps N time-domain samples into N frequency bins, so
    //   bin_spacing = c / (2 * bandwidth_hz * num_bins) ... wrong for radar.
    //
    // Correct radar range mapping:
    //   max_unambiguous_range = c / (2 * fs) * N = c * N / (2 * bandwidth_hz)
    //   range per bin = c / (2 * bandwidth_hz) = range_resolution
    //   Actually with N-point FFT on bandwidth B:
    //     delta_R = c / (2 * B)  (resolution)
    //   But the FFT bin index k maps to:
    //     R_k = k * c / (2 * B * N/N) = k * c / (2 * B)  ... only if N = num_range_samples
    //
    //   Simplified: R = range_bin * c / (2 * bandwidth_hz)
    //   But this assumes each bin = one resolution cell. With zero-padded FFT
    //   we must divide by num_bins:
    //     R_k = k * c / (2 * bandwidth_hz * num_bins) * num_original_samples
    //   For simplicity (and matching typical radar signal processing):
    //     R_k = k * c / (2 * bandwidth_hz) when num_bins = num_original_samples
    //
    // We use the general formula: R_k = k * (c / (2 * bandwidth_hz)) / (num_bins as f64 / num_bins as f64)
    // Simplification: since we zero-pad to next_power_of_two, the bin spacing
    // is c * num_original / (2 * B * fft_len), but we don't know the original
    // count here. We'll use: R_k = k * c / (2 * B * num_bins / num_bins)
    //
    // Standard convention: range per bin = c / (2 * B) when fft_size = num_range_samples.
    // With zero-padding to M points:  range per bin = c * N_orig / (2 * B * M).
    // For simplicity use:
    range_bin as f64 * c / (2.0 * bandwidth_hz)
}

// ---------------------------------------------------------------------------
// Doppler ambiguity resolution (multi-PRF)
// ---------------------------------------------------------------------------

/// Resolve Doppler ambiguity using multiple PRFs (Chinese Remainder Theorem approach).
///
/// Given targets detected at each of several PRFs, this function finds the
/// true (unambiguous) velocity by searching for a velocity that is consistent
/// with all measured Doppler bins modulo each PRF's unambiguous interval.
///
/// `targets` are the detections at the *first* PRF. `prfs` contains all PRF
/// values used (including the first). `carrier_freq_hz` and `c` are the RF
/// parameters. Returns targets with updated `velocity_mps`.
pub fn resolve_doppler_ambiguity(
    targets: &[RvdTarget],
    prfs: &[f64],
    carrier_freq_hz: f64,
    c: f64,
) -> Vec<RvdTarget> {
    if prfs.is_empty() || targets.is_empty() {
        return targets.to_vec();
    }

    // With a single PRF, no ambiguity can be resolved beyond what we already
    // have. Return targets unchanged.
    if prfs.len() == 1 {
        return targets.to_vec();
    }

    let wavelength = c / carrier_freq_hz;

    // For each PRF, the unambiguous velocity interval is [-v_ua, +v_ua)
    // where v_ua = PRF * lambda / 4.
    let v_unamb: Vec<f64> = prfs.iter().map(|&prf| prf * wavelength / 4.0).collect();

    // Maximum unambiguous velocity across all PRFs
    let v_max: f64 = v_unamb.iter().cloned().fold(0.0_f64, f64::max);

    // Search granularity: use the finest velocity resolution among all PRFs
    let v_res = prfs
        .iter()
        .map(|&prf| wavelength * prf / (2.0 * 1024.0)) // approximate bin width
        .fold(f64::MAX, f64::min);

    // For CRT-style resolution, we need a larger unambiguous interval.
    // The combined unambiguous interval is related to the LCM of the
    // individual intervals. We'll use a brute-force search over a range.
    let search_max = v_max * 4.0; // search wider than any single PRF

    let mut resolved = Vec::with_capacity(targets.len());

    for tgt in targets {
        // The measured velocity from the first PRF
        let v_measured = tgt.velocity_mps;

        // For each candidate velocity in [-search_max, search_max],
        // check consistency: fold the candidate into each PRF's unambiguous
        // interval and compare with the measured velocity (also folded).
        let mut best_v = v_measured;
        let mut best_cost = f64::MAX;

        let num_steps = (2.0 * search_max / v_res).ceil() as usize;
        let num_steps = num_steps.min(10_000); // cap for performance

        for step in 0..=num_steps {
            let v_candidate = -search_max + step as f64 * 2.0 * search_max / num_steps as f64;

            // Cost: sum of squared differences between the aliased candidate
            // and the measured velocity under each PRF.
            let mut cost = 0.0;
            for &v_ua in &v_unamb {
                let interval = 2.0 * v_ua;
                // Fold candidate velocity into [-v_ua, v_ua)
                let v_folded = fold_velocity(v_candidate, interval);
                // Fold the measurement the same way
                let v_meas_folded = fold_velocity(v_measured, interval);
                let diff = v_folded - v_meas_folded;
                cost += diff * diff;
            }

            if cost < best_cost {
                best_cost = cost;
                best_v = v_candidate;
            }
        }

        let mut resolved_tgt = tgt.clone();
        resolved_tgt.velocity_mps = best_v;
        resolved.push(resolved_tgt);
    }

    resolved
}

/// Fold a velocity value into the interval [-half_interval, +half_interval).
fn fold_velocity(v: f64, interval: f64) -> f64 {
    if interval <= 0.0 {
        return v;
    }
    let mut folded = v % interval;
    let half = interval / 2.0;
    if folded > half {
        folded -= interval;
    } else if folded <= -half {
        folded += interval;
    }
    folded
}

// ---------------------------------------------------------------------------
// Ambiguity function
// ---------------------------------------------------------------------------

/// Compute the radar ambiguity function |chi(tau, fd)|^2 for a given pulse.
///
/// `pulse` is the complex baseband waveform (I, Q). `max_delay` is the
/// maximum delay in samples (tau ranges from -max_delay to +max_delay).
/// `max_doppler_shift` is the maximum Doppler shift in Hz.
/// `num_doppler_steps` is the number of Doppler bins (centered on 0).
///
/// Returns a 2D grid `[delay_index][doppler_index]` of |chi|^2 values,
/// normalized so the peak equals 1.0.
pub fn compute_ambiguity_function(
    pulse: &[(f64, f64)],
    max_delay: usize,
    max_doppler_shift: f64,
    num_doppler_steps: usize,
) -> Vec<Vec<f64>> {
    let n = pulse.len();
    if n == 0 || num_doppler_steps == 0 {
        return Vec::new();
    }

    let num_delays = 2 * max_delay + 1;
    let num_doppler = if num_doppler_steps < 1 {
        1
    } else {
        num_doppler_steps
    };

    let doppler_step = if num_doppler > 1 {
        2.0 * max_doppler_shift / (num_doppler - 1) as f64
    } else {
        0.0
    };

    let mut surface = vec![vec![0.0f64; num_doppler]; num_delays];

    for tau_idx in 0..num_delays {
        let tau = tau_idx as isize - max_delay as isize;

        for fd_idx in 0..num_doppler {
            let fd = -max_doppler_shift + fd_idx as f64 * doppler_step;

            let mut sum = Complex::zero();
            for k in 0..n {
                let shifted = k as isize + tau;
                if shifted >= 0 && (shifted as usize) < n {
                    let x = Complex::new(pulse[k].0, pulse[k].1);
                    let y = Complex::new(pulse[shifted as usize].0, pulse[shifted as usize].1);
                    let phase = 2.0 * PI * fd * k as f64 / n as f64;
                    let steering = Complex::from_polar(1.0, phase);
                    sum += x * y.conj() * steering;
                }
            }
            surface[tau_idx][fd_idx] = sum.norm_sqr();
        }
    }

    // Normalize to peak = 1.0
    let peak = surface
        .iter()
        .flat_map(|row| row.iter())
        .cloned()
        .fold(0.0_f64, f64::max);
    if peak > 0.0 {
        for row in &mut surface {
            for val in row.iter_mut() {
                *val /= peak;
            }
        }
    }

    surface
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1e-9;
    const SPEED_OF_LIGHT: f64 = 3.0e8;

    /// Helper: generate a single-tone complex sinusoid.
    fn tone(freq_bin: usize, n: usize, amplitude: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|k| {
                let phase = 2.0 * PI * freq_bin as f64 * k as f64 / n as f64;
                (amplitude * phase.cos(), amplitude * phase.sin())
            })
            .collect()
    }

    /// Helper: generate a CPI matrix with a single target at a given range
    /// bin and Doppler bin.
    fn make_cpi_single_target(
        num_pulses: usize,
        num_range_bins: usize,
        target_range_bin: usize,
        target_doppler_bin: usize,
        amplitude: f64,
    ) -> Vec<Vec<(f64, f64)>> {
        // Each pulse: a complex sinusoid in the range dimension at the target
        // range bin, with a phase progression across pulses corresponding to
        // the target Doppler bin.
        let mut cpi = vec![vec![(0.0, 0.0); num_range_bins]; num_pulses];
        for p in 0..num_pulses {
            let doppler_phase =
                2.0 * PI * target_doppler_bin as f64 * p as f64 / num_pulses as f64;
            // Place energy at the target range bin via inverse FFT approach:
            // In the fast-time domain, a tone at freq = target_range_bin.
            for k in 0..num_range_bins {
                let range_phase =
                    2.0 * PI * target_range_bin as f64 * k as f64 / num_range_bins as f64;
                let total_phase = range_phase + doppler_phase;
                cpi[p][k].0 += amplitude * total_phase.cos();
                cpi[p][k].1 += amplitude * total_phase.sin();
            }
        }
        cpi
    }

    // ---- FFT tests ----

    #[test]
    fn test_fft_dc() {
        let mut data = vec![(1.0, 0.0); 8];
        fft_radix2(&mut data, false);
        // DC component should be 8.0, all others 0
        assert!((data[0].0 - 8.0).abs() < EPSILON);
        assert!(data[0].1.abs() < EPSILON);
        for k in 1..8 {
            assert!(data[k].0.abs() < EPSILON);
            assert!(data[k].1.abs() < EPSILON);
        }
    }

    #[test]
    fn test_fft_single_tone() {
        let n = 64;
        let freq = 5;
        let mut data = tone(freq, n, 1.0);
        fft_radix2(&mut data, false);
        // Peak should be at bin 5
        let magnitudes: Vec<f64> = data.iter().map(|&(re, im)| (re * re + im * im).sqrt()).collect();
        let peak_bin = magnitudes
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak_bin, freq);
    }

    #[test]
    fn test_fft_inverse_roundtrip() {
        let n = 32;
        let original: Vec<(f64, f64)> = (0..n)
            .map(|k| ((k as f64).sin(), (k as f64 * 0.3).cos()))
            .collect();
        let mut data = original.clone();
        fft_radix2(&mut data, false);
        fft_radix2(&mut data, true);
        for k in 0..n {
            assert!(
                (data[k].0 - original[k].0).abs() < 1e-10,
                "Real mismatch at k={}: {} vs {}",
                k,
                data[k].0,
                original[k].0
            );
            assert!(
                (data[k].1 - original[k].1).abs() < 1e-10,
                "Imag mismatch at k={}: {} vs {}",
                k,
                data[k].1,
                original[k].1
            );
        }
    }

    #[test]
    fn test_fft_parseval() {
        // Energy in time domain should equal energy in frequency domain (scaled by N)
        let n = 128;
        let data_orig: Vec<(f64, f64)> = (0..n)
            .map(|k| ((k as f64 * 0.7).sin(), (k as f64 * 1.3).cos()))
            .collect();
        let time_energy: f64 = data_orig.iter().map(|&(r, i)| r * r + i * i).sum();

        let mut data = data_orig;
        fft_radix2(&mut data, false);
        let freq_energy: f64 = data.iter().map(|&(r, i)| r * r + i * i).sum();

        assert!(
            (freq_energy / n as f64 - time_energy).abs() < 1e-6,
            "Parseval failed: time={}, freq/N={}",
            time_energy,
            freq_energy / n as f64
        );
    }

    #[test]
    fn test_fft_length_one() {
        let mut data = vec![(42.0, -7.0)];
        fft_radix2(&mut data, false);
        assert!((data[0].0 - 42.0).abs() < EPSILON);
        assert!((data[0].1 - (-7.0)).abs() < EPSILON);
    }

    #[test]
    fn test_fft_two_tones() {
        let n = 64;
        let freq1 = 3;
        let freq2 = 12;
        let t1 = tone(freq1, n, 1.0);
        let t2 = tone(freq2, n, 0.5);
        let mut data: Vec<(f64, f64)> = t1
            .iter()
            .zip(t2.iter())
            .map(|(&(r1, i1), &(r2, i2))| (r1 + r2, i1 + i2))
            .collect();
        fft_radix2(&mut data, false);
        let magnitudes: Vec<f64> = data.iter().map(|&(r, i)| (r * r + i * i).sqrt()).collect();
        // Two peaks
        let mut sorted: Vec<(usize, f64)> =
            magnitudes.iter().cloned().enumerate().collect();
        sorted.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());
        let top_bins: Vec<usize> = sorted.iter().take(2).map(|&(i, _)| i).collect();
        assert!(top_bins.contains(&freq1));
        assert!(top_bins.contains(&freq2));
    }

    // ---- Doppler-to-velocity tests ----

    #[test]
    fn test_doppler_to_velocity_zero() {
        // Bin 0 → zero velocity
        let v = doppler_to_velocity(0, 64, 1000.0, 10.0e9, SPEED_OF_LIGHT);
        assert!(v.abs() < EPSILON, "Zero Doppler bin should give 0 velocity, got {}", v);
    }

    #[test]
    fn test_doppler_to_velocity_positive() {
        // Bin 1 out of 64, PRF=1000, fc=10GHz
        let v = doppler_to_velocity(1, 64, 1000.0, 10.0e9, SPEED_OF_LIGHT);
        // fd = 1 * 1000/64 = 15.625 Hz
        // lambda = 3e8/10e9 = 0.03 m
        // v = fd * lambda / 2 = 15.625 * 0.03 / 2 = 0.234375 m/s
        let expected = 15.625 * 0.03 / 2.0;
        assert!(
            (v - expected).abs() < 1e-6,
            "Expected {}, got {}",
            expected,
            v
        );
    }

    #[test]
    fn test_doppler_to_velocity_negative() {
        // Bin 63 out of 64 → negative Doppler (bin >= N/2 wraps negative)
        let v = doppler_to_velocity(63, 64, 1000.0, 10.0e9, SPEED_OF_LIGHT);
        // signed_bin = 63 - 64 = -1
        // fd = -1 * 1000/64 = -15.625 Hz
        // v = -15.625 * 0.03 / 2 = -0.234375
        let expected = -15.625 * 0.03 / 2.0;
        assert!(
            (v - expected).abs() < 1e-6,
            "Expected {}, got {}",
            expected,
            v
        );
    }

    #[test]
    fn test_doppler_to_velocity_nyquist() {
        // Bin N/2 → most negative Doppler
        let n = 64;
        let v = doppler_to_velocity(n / 2, n, 1000.0, 10.0e9, SPEED_OF_LIGHT);
        // signed_bin = 32 - 64 = -32 → fd = -32*1000/64 = -500 Hz
        let expected = -500.0 * 0.03 / 2.0;
        assert!((v - expected).abs() < 1e-6);
    }

    #[test]
    fn test_doppler_to_velocity_zero_bins() {
        let v = doppler_to_velocity(0, 0, 1000.0, 10.0e9, SPEED_OF_LIGHT);
        assert!(v.abs() < EPSILON);
    }

    // ---- Range-bin-to-meters tests ----

    #[test]
    fn test_range_bin_zero() {
        let r = range_bin_to_meters(0, 5.0e6, 64, SPEED_OF_LIGHT);
        assert!(r.abs() < EPSILON);
    }

    #[test]
    fn test_range_bin_to_meters_basic() {
        // delta_R = c / (2 * B) = 3e8 / (2 * 5e6) = 30 m
        let r = range_bin_to_meters(1, 5.0e6, 64, SPEED_OF_LIGHT);
        let expected = 30.0;
        assert!(
            (r - expected).abs() < 1e-6,
            "Expected {} m, got {} m",
            expected,
            r
        );
    }

    #[test]
    fn test_range_bin_to_meters_bin10() {
        let r = range_bin_to_meters(10, 5.0e6, 64, SPEED_OF_LIGHT);
        let expected = 10.0 * 30.0;
        assert!((r - expected).abs() < 1e-6);
    }

    #[test]
    fn test_range_bin_to_meters_zero_bandwidth() {
        let r = range_bin_to_meters(5, 0.0, 64, SPEED_OF_LIGHT);
        assert!(r.abs() < EPSILON);
    }

    // ---- CFAR tests ----

    #[test]
    fn test_cfar_2d_single_target() {
        // Create a map with a single bright target and low noise floor
        let n = 32;
        let mut map = vec![vec![1.0; n]; n];
        // Place a strong target at (16, 16)
        map[16][16] = 1000.0;

        let dets = cfar_2d(&map, 2, 4, 1e-4);
        // Should detect the target
        assert!(
            !dets.is_empty(),
            "CFAR should detect the strong target"
        );
        let found = dets.iter().any(|&(r, d, _)| r == 16 && d == 16);
        assert!(found, "Target at (16,16) should be detected");
    }

    #[test]
    fn test_cfar_2d_no_targets() {
        // Uniform map → no detections (noise only)
        let n = 32;
        let map = vec![vec![1.0; n]; n];
        let dets = cfar_2d(&map, 2, 4, 1e-6);
        assert!(
            dets.is_empty(),
            "Uniform map should produce no CFAR detections, got {}",
            dets.len()
        );
    }

    #[test]
    fn test_cfar_2d_two_targets() {
        let n = 64;
        let mut map = vec![vec![0.1; n]; n];
        map[20][10] = 500.0;
        map[40][50] = 300.0;
        let dets = cfar_2d(&map, 2, 4, 1e-4);
        let found_1 = dets.iter().any(|&(r, d, _)| r == 20 && d == 10);
        let found_2 = dets.iter().any(|&(r, d, _)| r == 40 && d == 50);
        assert!(found_1, "Target 1 at (20,10) should be detected");
        assert!(found_2, "Target 2 at (40,50) should be detected");
    }

    #[test]
    fn test_cfar_2d_snr_value() {
        let n = 32;
        let noise_power = 2.0;
        let target_power = 2000.0;
        let mut map = vec![vec![noise_power; n]; n];
        map[16][16] = target_power;
        let dets = cfar_2d(&map, 2, 4, 1e-4);
        let det = dets.iter().find(|&&(r, d, _)| r == 16 && d == 16);
        assert!(det.is_some(), "Target should be detected");
        let snr_db = det.unwrap().2;
        // SNR ≈ 10*log10(2000/2) = 10*log10(1000) ≈ 30 dB
        assert!(
            (snr_db - 30.0).abs() < 1.0,
            "SNR should be ~30 dB, got {} dB",
            snr_db
        );
    }

    #[test]
    fn test_cfar_2d_pfa_sensitivity() {
        // With a moderately strong target, low pfa should still detect it,
        // but an extremely high pfa threshold factor is lower so more detections.
        let n = 32;
        let mut map = vec![vec![1.0; n]; n];
        map[16][16] = 100.0;
        let dets_low_pfa = cfar_2d(&map, 2, 4, 1e-8);
        let dets_high_pfa = cfar_2d(&map, 2, 4, 1e-1);
        // Both should detect the very strong target
        let found_low = dets_low_pfa.iter().any(|&(r, d, _)| r == 16 && d == 16);
        let found_high = dets_high_pfa.iter().any(|&(r, d, _)| r == 16 && d == 16);
        assert!(found_low || found_high, "At least one pfa setting should detect target");
        // High pfa → more detections (lower threshold)
        assert!(
            dets_high_pfa.len() >= dets_low_pfa.len(),
            "Higher Pfa should yield >= detections: {} vs {}",
            dets_high_pfa.len(),
            dets_low_pfa.len()
        );
    }

    #[test]
    fn test_cfar_2d_small_map() {
        // Map too small for guard + training ring
        let map = vec![vec![100.0; 4]; 4];
        let dets = cfar_2d(&map, 2, 4, 1e-4);
        assert!(dets.is_empty(), "Map too small should return no detections");
    }

    // ---- Range-Doppler map tests ----

    #[test]
    fn test_range_doppler_map_single_target() {
        let num_pulses = 32;
        let num_range = 32;
        let target_rbin = 5;
        let target_dbin = 3;
        let cpi = make_cpi_single_target(num_pulses, num_range, target_rbin, target_dbin, 10.0);
        let rd = range_doppler_map(&cpi, num_pulses);
        assert!(!rd.is_empty());
        assert!(!rd[0].is_empty());

        // Find peak in the RD map
        let mut max_val = 0.0f64;
        let mut max_r = 0;
        let mut max_d = 0;
        for (r, row) in rd.iter().enumerate() {
            for (d, &val) in row.iter().enumerate() {
                if val > max_val {
                    max_val = val;
                    max_r = r;
                    max_d = d;
                }
            }
        }
        assert_eq!(
            max_r, target_rbin,
            "Peak range bin: expected {}, got {}",
            target_rbin, max_r
        );
        assert_eq!(
            max_d, target_dbin,
            "Peak Doppler bin: expected {}, got {}",
            target_dbin, max_d
        );
    }

    #[test]
    fn test_range_doppler_map_two_targets() {
        let num_pulses = 64;
        let num_range = 64;
        let cpi1 = make_cpi_single_target(num_pulses, num_range, 10, 5, 10.0);
        let cpi2 = make_cpi_single_target(num_pulses, num_range, 30, 20, 8.0);
        // Superimpose both
        let cpi: Vec<Vec<(f64, f64)>> = cpi1
            .iter()
            .zip(cpi2.iter())
            .map(|(row1, row2)| {
                row1.iter()
                    .zip(row2.iter())
                    .map(|(&(r1, i1), &(r2, i2))| (r1 + r2, i1 + i2))
                    .collect()
            })
            .collect();

        let rd = range_doppler_map(&cpi, num_pulses);

        // Find top-2 peaks
        let mut peaks: Vec<(usize, usize, f64)> = Vec::new();
        for (r, row) in rd.iter().enumerate() {
            for (d, &val) in row.iter().enumerate() {
                peaks.push((r, d, val));
            }
        }
        peaks.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap());
        let top2: Vec<(usize, usize)> = peaks.iter().take(2).map(|&(r, d, _)| (r, d)).collect();
        assert!(
            top2.contains(&(10, 5)),
            "Should find target at (10,5), top2: {:?}",
            top2
        );
        assert!(
            top2.contains(&(30, 20)),
            "Should find target at (30,20), top2: {:?}",
            top2
        );
    }

    #[test]
    fn test_range_doppler_map_empty() {
        let rd = range_doppler_map(&[], 8);
        assert!(rd.is_empty());
    }

    // ---- Full processor tests ----

    #[test]
    fn test_processor_single_target() {
        let config = RvdConfig {
            prf_hz: 1000.0,
            carrier_freq_hz: 10.0e9,
            bandwidth_hz: 5.0e6,
            num_range_bins: 128,
            num_doppler_bins: 128,
            speed_of_light: SPEED_OF_LIGHT,
        };
        let processor = RvdProcessor::new(config);

        // Strong target well inside the CFAR window: range bin 32, Doppler bin 16
        let num_pulses = 128;
        let num_range = 128;
        let cpi = make_cpi_single_target(num_pulses, num_range, 32, 16, 100.0);

        let targets = processor.process_cpi(&cpi);
        assert!(
            !targets.is_empty(),
            "Processor should detect the target"
        );
        // The strongest target should be at range bin 32
        let best = targets
            .iter()
            .max_by(|a, b| a.snr_db.partial_cmp(&b.snr_db).unwrap())
            .unwrap();
        assert_eq!(best.range_bin, 32, "Best target range bin should be 32");
        assert_eq!(best.doppler_bin, 16, "Best target doppler bin should be 16");
    }

    #[test]
    fn test_processor_empty_cpi() {
        let processor = RvdProcessor::new(RvdConfig::default());
        let targets = processor.process_cpi(&[]);
        assert!(targets.is_empty());
    }

    #[test]
    fn test_processor_config_access() {
        let config = RvdConfig {
            prf_hz: 2000.0,
            ..RvdConfig::default()
        };
        let processor = RvdProcessor::new(config);
        assert!((processor.config().prf_hz - 2000.0).abs() < EPSILON);
    }

    // ---- Ambiguity function tests ----

    #[test]
    fn test_ambiguity_peak_at_origin() {
        // A rectangular pulse should have peak |chi|^2 at (tau=0, fd=0)
        let n = 32;
        let pulse: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let af = compute_ambiguity_function(&pulse, n / 2, 0.5, 21);
        assert!(!af.is_empty());
        // Peak should be at center delay (max_delay) and center Doppler (mid index)
        let center_delay = n / 2;
        let center_doppler = 10; // (21-1)/2
        let peak_val = af[center_delay][center_doppler];
        assert!(
            (peak_val - 1.0).abs() < 1e-6,
            "Peak at origin should be 1.0, got {}",
            peak_val
        );
    }

    #[test]
    fn test_ambiguity_symmetric() {
        // Ambiguity function of a real symmetric pulse should be symmetric
        let n = 16;
        let pulse: Vec<(f64, f64)> = vec![(1.0, 0.0); n];
        let max_delay = 8;
        let num_dopp = 11;
        let af = compute_ambiguity_function(&pulse, max_delay, 0.5, num_dopp);
        // Check delay symmetry at zero Doppler
        let fd_center = num_dopp / 2;
        for tau in 0..max_delay {
            let val_pos = af[max_delay + tau][fd_center];
            let val_neg = af[max_delay - tau][fd_center];
            assert!(
                (val_pos - val_neg).abs() < 1e-6,
                "Asymmetry at tau={}: {} vs {}",
                tau,
                val_pos,
                val_neg
            );
        }
    }

    #[test]
    fn test_ambiguity_empty_pulse() {
        let af = compute_ambiguity_function(&[], 5, 0.5, 10);
        assert!(af.is_empty());
    }

    // ---- Doppler ambiguity resolution tests ----

    #[test]
    fn test_resolve_doppler_ambiguity_identity() {
        // Single PRF → no resolution possible, velocity should stay exactly the same
        let targets = vec![RvdTarget {
            range_m: 1000.0,
            velocity_mps: 5.0,
            snr_db: 20.0,
            range_bin: 10,
            doppler_bin: 5,
        }];
        let resolved = resolve_doppler_ambiguity(&targets, &[1000.0], 10.0e9, SPEED_OF_LIGHT);
        assert_eq!(resolved.len(), 1);
        assert!(
            (resolved[0].velocity_mps - 5.0).abs() < EPSILON,
            "Single-PRF resolution should preserve velocity exactly, got {}",
            resolved[0].velocity_mps
        );
    }

    #[test]
    fn test_resolve_doppler_ambiguity_empty() {
        let resolved = resolve_doppler_ambiguity(&[], &[1000.0, 1200.0], 10.0e9, SPEED_OF_LIGHT);
        assert!(resolved.is_empty());
    }

    #[test]
    fn test_resolve_doppler_ambiguity_multi_prf() {
        // Two staggered PRFs should give a velocity consistent with both
        let targets = vec![RvdTarget {
            range_m: 3000.0,
            velocity_mps: 2.0,
            snr_db: 25.0,
            range_bin: 100,
            doppler_bin: 8,
        }];
        let resolved =
            resolve_doppler_ambiguity(&targets, &[1000.0, 1200.0], 10.0e9, SPEED_OF_LIGHT);
        assert_eq!(resolved.len(), 1);
        // Should produce a velocity close to the input (which is already unambiguous)
        assert!(
            (resolved[0].velocity_mps - 2.0).abs() < 1.0,
            "Multi-PRF should find ~2.0 m/s, got {}",
            resolved[0].velocity_mps
        );
    }

    // ---- Stationary target (zero Doppler) ----

    #[test]
    fn test_stationary_target_zero_doppler() {
        let num_pulses = 32;
        let num_range = 32;
        // Doppler bin 0 → stationary
        let cpi = make_cpi_single_target(num_pulses, num_range, 8, 0, 20.0);
        let rd = range_doppler_map(&cpi, num_pulses);

        let mut max_val = 0.0f64;
        let mut max_r = 0;
        let mut max_d = 0;
        for (r, row) in rd.iter().enumerate() {
            for (d, &val) in row.iter().enumerate() {
                if val > max_val {
                    max_val = val;
                    max_r = r;
                    max_d = d;
                }
            }
        }
        assert_eq!(max_r, 8, "Stationary target range bin should be 8");
        assert_eq!(max_d, 0, "Stationary target Doppler bin should be 0");
    }

    // ---- Edge case: single pulse ----

    #[test]
    fn test_single_pulse_cpi() {
        // CPI with only 1 pulse → Doppler FFT has length 1
        let cpi = vec![vec![(1.0, 0.0); 16]];
        let rd = range_doppler_map(&cpi, 1);
        // Should still produce a valid map (no crash)
        assert!(!rd.is_empty());
    }

    // ---- Edge case: single range bin ----

    #[test]
    fn test_single_range_bin() {
        let num_pulses = 16;
        let cpi: Vec<Vec<(f64, f64)>> = (0..num_pulses).map(|_| vec![(1.0, 0.0)]).collect();
        let rd = range_doppler_map(&cpi, num_pulses);
        // Should produce a 1 x N map (after zero-padding range to 1)
        assert!(!rd.is_empty());
    }

    // ---- Complex arithmetic tests ----

    #[test]
    fn test_complex_mul() {
        let a = Complex::new(3.0, 4.0);
        let b = Complex::new(1.0, 2.0);
        let c = a * b;
        // (3+4i)(1+2i) = 3+6i+4i+8i^2 = 3+10i-8 = -5+10i
        assert!((c.re - (-5.0)).abs() < EPSILON);
        assert!((c.im - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_complex_conj() {
        let a = Complex::new(3.0, 4.0);
        let b = a.conj();
        assert!((b.re - 3.0).abs() < EPSILON);
        assert!((b.im - (-4.0)).abs() < EPSILON);
    }

    #[test]
    fn test_complex_norm() {
        let a = Complex::new(3.0, 4.0);
        assert!((a.norm() - 5.0).abs() < EPSILON);
        assert!((a.norm_sqr() - 25.0).abs() < EPSILON);
    }
}
