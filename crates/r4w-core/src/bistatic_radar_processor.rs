//! Passive bistatic radar signal processing using non-cooperative illuminators.
//!
//! This module implements a passive radar processor that exploits transmissions of
//! opportunity (e.g., DVB-T, FM radio) to detect and locate targets without a
//! dedicated transmitter. The key processing stages are:
//!
//! 1. **Direct-path cancellation** — Remove the strong direct-path signal from the
//!    surveillance channel using ECA, LMS, or Batch-LMS adaptive filtering.
//! 2. **Cross-ambiguity function (CAF)** — Correlate the cleaned surveillance signal
//!    with time-delayed and Doppler-shifted copies of the reference signal to produce
//!    a range-Doppler map.
//! 3. **Target detection** — Apply 2D CFAR thresholding on the range-Doppler surface
//!    to extract bistatic detections.
//!
//! # Example
//!
//! ```
//! use r4w_core::bistatic_radar_processor::{BistaticProcessor, BistaticConfig, CancellationMethod};
//!
//! let config = BistaticConfig {
//!     sample_rate: 1000.0,
//!     num_range_bins: 32,
//!     num_doppler_bins: 16,
//!     cancellation_order: 5,
//!     integration_time_s: 0.032,
//!     cancellation_method: CancellationMethod::Eca,
//! };
//!
//! let processor = BistaticProcessor::new(config);
//!
//! // Create simple test signals — reference is a tone, surveillance has a delayed copy
//! let n = 32;
//! let reference: Vec<(f64, f64)> = (0..n).map(|i| {
//!     let phase = 2.0 * std::f64::consts::PI * 0.1 * i as f64;
//!     (phase.cos(), phase.sin())
//! }).collect();
//! let surveillance = reference.clone();
//!
//! // Cancel direct path
//! let cleaned = processor.cancel_direct_path(&surveillance, &reference);
//! assert_eq!(cleaned.len(), surveillance.len());
//!
//! // Compute cross-ambiguity surface
//! let cas = processor.cross_ambiguity(&surveillance, &reference);
//! assert_eq!(cas.range_doppler_map.len(), 32);
//!
//! // Detect targets above threshold
//! let detections = processor.detect_targets(&cas, 3.0);
//! // detections may or may not be empty depending on signal content
//! assert!(detections.len() <= 32 * 16);
//!
//! // Check resolutions
//! assert!(processor.range_resolution() > 0.0);
//! assert!(processor.doppler_resolution() > 0.0);
//! ```

use std::f64::consts::PI;

/// Configuration for the bistatic radar processor.
#[derive(Debug, Clone)]
pub struct BistaticConfig {
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Number of range bins (delay taps) in the cross-ambiguity surface.
    pub num_range_bins: usize,
    /// Number of Doppler bins in the cross-ambiguity surface.
    pub num_doppler_bins: usize,
    /// Filter order for direct-path cancellation (number of taps).
    pub cancellation_order: usize,
    /// Coherent integration time in seconds.
    pub integration_time_s: f64,
    /// Direct-path cancellation algorithm to use.
    pub cancellation_method: CancellationMethod,
}

/// Direct-path cancellation algorithm selection.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CancellationMethod {
    /// Extensive Cancellation Algorithm (batch least-squares projection).
    Eca,
    /// Least Mean Squares adaptive filter.
    Lms,
    /// Block / Batch LMS adaptive filter.
    BatchLms,
}

/// 2D range-Doppler surface produced by the cross-ambiguity function.
#[derive(Debug, Clone)]
pub struct CrossAmbiguitySurface {
    /// Range-Doppler map: `[range_bin][doppler_bin]` power in linear scale.
    pub range_doppler_map: Vec<Vec<f64>>,
    /// Range resolution in metres.
    pub range_resolution_m: f64,
    /// Doppler resolution in Hz.
    pub doppler_resolution_hz: f64,
}

/// A single bistatic target detection.
#[derive(Debug, Clone)]
pub struct BistaticDetection {
    /// Estimated range in metres (from range bin).
    pub range_m: f64,
    /// Estimated Doppler shift in Hz.
    pub doppler_hz: f64,
    /// Estimated signal-to-noise ratio in dB.
    pub snr_db: f64,
    /// Bistatic range in metres.
    pub bistatic_range_m: f64,
}

/// Main passive bistatic radar processor.
#[derive(Debug, Clone)]
pub struct BistaticProcessor {
    config: BistaticConfig,
}

// ---------------------------------------------------------------------------
// Complex arithmetic helpers (using (f64, f64) tuples)
// ---------------------------------------------------------------------------

#[inline]
fn cx_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn cx_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn cx_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn cx_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

#[inline]
fn cx_mag_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn cx_scale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

/// Euclidean distance between two 3D points.
fn dist3(a: (f64, f64, f64), b: (f64, f64, f64)) -> f64 {
    let dx = a.0 - b.0;
    let dy = a.1 - b.1;
    let dz = a.2 - b.2;
    (dx * dx + dy * dy + dz * dz).sqrt()
}

impl BistaticProcessor {
    /// Create a new processor from the given configuration.
    pub fn new(config: BistaticConfig) -> Self {
        Self { config }
    }

    /// Range resolution in metres: c / sample_rate.
    ///
    /// For bistatic geometry, the range cell spacing corresponds to one sample
    /// of delay, which maps to c / sample_rate metres of bistatic range.
    pub fn range_resolution(&self) -> f64 {
        const C: f64 = 299_792_458.0;
        C / self.config.sample_rate
    }

    /// Doppler resolution in Hz: 1 / integration_time_s.
    pub fn doppler_resolution(&self) -> f64 {
        1.0 / self.config.integration_time_s
    }

    /// Compute bistatic range for a given geometry.
    ///
    /// bistatic_range = |Tx - Target| + |Target - Rx| - |Tx - Rx|
    pub fn bistatic_range(
        &self,
        tx_pos: (f64, f64, f64),
        rx_pos: (f64, f64, f64),
        target_pos: (f64, f64, f64),
    ) -> f64 {
        let d_tx_target = dist3(tx_pos, target_pos);
        let d_target_rx = dist3(target_pos, rx_pos);
        let d_tx_rx = dist3(tx_pos, rx_pos);
        d_tx_target + d_target_rx - d_tx_rx
    }

    /// Cancel the direct-path and multipath clutter from the surveillance
    /// channel using the chosen cancellation method.
    ///
    /// Returns a new vector of the same length as `surveillance` with the
    /// direct-path contribution removed.
    pub fn cancel_direct_path(
        &self,
        surveillance: &[(f64, f64)],
        reference: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        match self.config.cancellation_method {
            CancellationMethod::Eca => self.cancel_eca(surveillance, reference),
            CancellationMethod::Lms => self.cancel_lms(surveillance, reference),
            CancellationMethod::BatchLms => self.cancel_batch_lms(surveillance, reference),
        }
    }

    /// ECA: Extensive Cancellation Algorithm.
    ///
    /// Build a matrix of delayed reference copies and solve for the
    /// least-squares weights, then subtract the reconstruction from the
    /// surveillance signal.
    fn cancel_eca(
        &self,
        surveillance: &[(f64, f64)],
        reference: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        let n = surveillance.len();
        let p = self.config.cancellation_order.min(n);

        if p == 0 || n == 0 {
            return surveillance.to_vec();
        }

        // Build R = X^H X (p x p) and rhs = X^H s (p x 1)
        // where X[:,k] = reference shifted by k samples (zero-padded).
        let mut r_mat = vec![(0.0_f64, 0.0_f64); p * p];
        let mut rhs = vec![(0.0_f64, 0.0_f64); p];

        for i in 0..p {
            // rhs[i] = sum_n conj(reference[n-i]) * surveillance[n]
            for n_idx in i..n {
                let ref_idx = n_idx - i;
                if ref_idx < reference.len() {
                    rhs[i] = cx_add(
                        rhs[i],
                        cx_mul(cx_conj(reference[ref_idx]), surveillance[n_idx]),
                    );
                }
            }
            // R[i][j] = sum_n conj(reference[n-i]) * reference[n-j]
            for j in 0..p {
                let mut acc = (0.0, 0.0);
                let start = i.max(j);
                for n_idx in start..n {
                    let ri = n_idx - i;
                    let rj = n_idx - j;
                    if ri < reference.len() && rj < reference.len() {
                        acc = cx_add(acc, cx_mul(cx_conj(reference[ri]), reference[rj]));
                    }
                }
                r_mat[i * p + j] = acc;
            }
        }

        // Solve R * w = rhs using Gauss elimination with partial pivoting.
        let weights = solve_complex_system(&mut r_mat, &mut rhs, p);

        // Reconstruct direct-path estimate and subtract.
        let mut output = surveillance.to_vec();
        for i in 0..n {
            let mut est = (0.0, 0.0);
            for k in 0..p {
                if i >= k && (i - k) < reference.len() {
                    est = cx_add(est, cx_mul(weights[k], reference[i - k]));
                }
            }
            output[i] = cx_sub(output[i], est);
        }
        output
    }

    /// LMS adaptive filter cancellation.
    fn cancel_lms(
        &self,
        surveillance: &[(f64, f64)],
        reference: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        let n = surveillance.len();
        let p = self.config.cancellation_order.min(n);
        let mu = 0.01; // step size

        let mut weights = vec![(0.0_f64, 0.0_f64); p];
        let mut output = vec![(0.0, 0.0); n];

        for i in 0..n {
            // Compute filter output y = w^H * x
            let mut y = (0.0, 0.0);
            for k in 0..p {
                if i >= k && (i - k) < reference.len() {
                    y = cx_add(y, cx_mul(cx_conj(weights[k]), reference[i - k]));
                }
            }

            // Error = surveillance - y
            let error = cx_sub(surveillance[i], y);
            output[i] = error;

            // Update weights: w += mu * x * conj(error)
            for k in 0..p {
                if i >= k && (i - k) < reference.len() {
                    let update = cx_scale(cx_mul(reference[i - k], cx_conj(error)), mu);
                    weights[k] = cx_add(weights[k], update);
                }
            }
        }
        output
    }

    /// Batch LMS: process in blocks, update weights once per block.
    fn cancel_batch_lms(
        &self,
        surveillance: &[(f64, f64)],
        reference: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        let n = surveillance.len();
        let p = self.config.cancellation_order.min(n);
        let mu = 0.05;
        let block_size = 64_usize.min(n);
        let num_blocks = (n + block_size - 1) / block_size;

        let mut weights = vec![(0.0_f64, 0.0_f64); p];
        let mut output = vec![(0.0, 0.0); n];

        for blk in 0..num_blocks {
            let start = blk * block_size;
            let end = (start + block_size).min(n);

            // Accumulate gradient over the block.
            let mut grad = vec![(0.0_f64, 0.0_f64); p];

            for i in start..end {
                let mut y = (0.0, 0.0);
                for k in 0..p {
                    if i >= k && (i - k) < reference.len() {
                        y = cx_add(y, cx_mul(cx_conj(weights[k]), reference[i - k]));
                    }
                }
                let error = cx_sub(surveillance[i], y);
                output[i] = error;

                for k in 0..p {
                    if i >= k && (i - k) < reference.len() {
                        grad[k] = cx_add(
                            grad[k],
                            cx_mul(reference[i - k], cx_conj(error)),
                        );
                    }
                }
            }

            let block_len = (end - start) as f64;
            for k in 0..p {
                weights[k] = cx_add(weights[k], cx_scale(grad[k], mu / block_len));
            }
        }
        output
    }

    /// Compute the cross-ambiguity surface between surveillance and reference signals.
    ///
    /// For each range bin (delay tau) and Doppler bin (frequency shift f_d),
    /// compute:
    ///   CAF(tau, f_d) = |sum_n surv[n] * conj(ref[n-tau]) * exp(-j*2*pi*f_d*n/fs)|^2
    pub fn cross_ambiguity(
        &self,
        surveillance: &[(f64, f64)],
        reference: &[(f64, f64)],
    ) -> CrossAmbiguitySurface {
        let n = surveillance.len().min(reference.len());
        let nr = self.config.num_range_bins;
        let nd = self.config.num_doppler_bins;

        let range_res = self.range_resolution();
        let doppler_res = self.doppler_resolution();

        let mut map = vec![vec![0.0_f64; nd]; nr];

        for r_bin in 0..nr {
            let delay = r_bin;
            for d_bin in 0..nd {
                // Doppler frequency: centre the bins around zero
                let f_d = (d_bin as f64 - nd as f64 / 2.0) * doppler_res;

                let mut acc = (0.0_f64, 0.0_f64);
                for i in delay..n {
                    let ref_idx = i - delay;
                    if ref_idx < reference.len() {
                        // Doppler steering: exp(-j * 2pi * f_d * i / fs)
                        let phase = -2.0 * PI * f_d * (i as f64) / self.config.sample_rate;
                        let steer = (phase.cos(), phase.sin());
                        let prod = cx_mul(surveillance[i], cx_conj(reference[ref_idx]));
                        acc = cx_add(acc, cx_mul(prod, steer));
                    }
                }
                map[r_bin][d_bin] = cx_mag_sq(acc);
            }
        }

        CrossAmbiguitySurface {
            range_doppler_map: map,
            range_resolution_m: range_res,
            doppler_resolution_hz: doppler_res,
        }
    }

    /// Detect targets from the cross-ambiguity surface using a simple
    /// cell-averaging CFAR (Constant False Alarm Rate) detector.
    ///
    /// `threshold_db` is the required SNR above the local noise floor for
    /// a detection to be declared.
    pub fn detect_targets(
        &self,
        cas: &CrossAmbiguitySurface,
        threshold_db: f64,
    ) -> Vec<BistaticDetection> {
        let nr = cas.range_doppler_map.len();
        if nr == 0 {
            return Vec::new();
        }
        let nd = cas.range_doppler_map[0].len();
        if nd == 0 {
            return Vec::new();
        }

        let threshold_linear = 10.0_f64.powf(threshold_db / 10.0);
        let guard_r: usize = 1;
        let guard_d: usize = 1;
        let train_r: usize = 2;
        let train_d: usize = 2;

        let mut detections = Vec::new();

        for r in 0..nr {
            for d in 0..nd {
                let cell_power = cas.range_doppler_map[r][d];
                if cell_power <= 0.0 {
                    continue;
                }

                // Compute average noise from training cells around (r, d),
                // excluding guard cells.
                let mut noise_sum = 0.0_f64;
                let mut count = 0_usize;

                let r_lo = if r >= guard_r + train_r {
                    r - guard_r - train_r
                } else {
                    0
                };
                let r_hi = (r + guard_r + train_r + 1).min(nr);
                let d_lo = if d >= guard_d + train_d {
                    d - guard_d - train_d
                } else {
                    0
                };
                let d_hi = (d + guard_d + train_d + 1).min(nd);

                for ri in r_lo..r_hi {
                    for di in d_lo..d_hi {
                        // Skip guard and CUT cells
                        let in_guard_r =
                            (ri as isize - r as isize).unsigned_abs() <= guard_r;
                        let in_guard_d =
                            (di as isize - d as isize).unsigned_abs() <= guard_d;
                        if in_guard_r && in_guard_d {
                            continue;
                        }
                        noise_sum += cas.range_doppler_map[ri][di];
                        count += 1;
                    }
                }

                if count == 0 {
                    continue;
                }

                let noise_avg = noise_sum / count as f64;
                if noise_avg <= 0.0 {
                    continue;
                }

                let snr_linear = cell_power / noise_avg;
                if snr_linear >= threshold_linear {
                    let snr_db_val = 10.0 * snr_linear.log10();
                    let range_m = r as f64 * cas.range_resolution_m;
                    let doppler_hz =
                        (d as f64 - nd as f64 / 2.0) * cas.doppler_resolution_hz;

                    detections.push(BistaticDetection {
                        range_m,
                        doppler_hz,
                        snr_db: snr_db_val,
                        bistatic_range_m: range_m,
                    });
                }
            }
        }

        detections
    }
}

/// Solve a complex linear system A*x = b using Gaussian elimination with
/// partial pivoting. Operates in-place on `a` and `b`. Returns the solution.
fn solve_complex_system(
    a: &mut [(f64, f64)],
    b: &mut [(f64, f64)],
    n: usize,
) -> Vec<(f64, f64)> {
    // Forward elimination with partial pivoting.
    for col in 0..n {
        // Find pivot.
        let mut max_mag = cx_mag_sq(a[col * n + col]);
        let mut max_row = col;
        for row in (col + 1)..n {
            let mag = cx_mag_sq(a[row * n + col]);
            if mag > max_mag {
                max_mag = mag;
                max_row = row;
            }
        }

        // Swap rows.
        if max_row != col {
            for k in 0..n {
                a.swap(col * n + k, max_row * n + k);
            }
            b.swap(col, max_row);
        }

        let pivot = a[col * n + col];
        let pivot_mag = cx_mag_sq(pivot);
        if pivot_mag < 1e-30 {
            continue; // Singular or near-singular
        }

        // Eliminate below.
        for row in (col + 1)..n {
            let factor = cx_mul(a[row * n + col], cx_conj(pivot));
            let factor = cx_scale(factor, 1.0 / pivot_mag);
            for k in col..n {
                let sub = cx_mul(factor, a[col * n + k]);
                a[row * n + k] = cx_sub(a[row * n + k], sub);
            }
            let sub = cx_mul(factor, b[col]);
            b[row] = cx_sub(b[row], sub);
        }
    }

    // Back substitution.
    let mut x = vec![(0.0, 0.0); n];
    for col in (0..n).rev() {
        let pivot = a[col * n + col];
        let pivot_mag = cx_mag_sq(pivot);
        if pivot_mag < 1e-30 {
            continue;
        }
        let mut sum = b[col];
        for k in (col + 1)..n {
            sum = cx_sub(sum, cx_mul(a[col * n + k], x[k]));
        }
        x[col] = cx_mul(sum, cx_conj(pivot));
        x[col] = cx_scale(x[col], 1.0 / pivot_mag);
    }
    x
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> BistaticConfig {
        BistaticConfig {
            sample_rate: 1000.0,
            num_range_bins: 16,
            num_doppler_bins: 16,
            cancellation_order: 5,
            integration_time_s: 0.016,
            cancellation_method: CancellationMethod::Eca,
        }
    }

    fn tone(n: usize, freq: f64, fs: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let phase = 2.0 * PI * freq * i as f64 / fs;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    fn power(sig: &[(f64, f64)]) -> f64 {
        sig.iter().map(|s| cx_mag_sq(*s)).sum::<f64>() / sig.len() as f64
    }

    // 1. Processor creation succeeds with valid config.
    #[test]
    fn test_new_processor() {
        let cfg = default_config();
        let proc = BistaticProcessor::new(cfg);
        assert_eq!(proc.config.sample_rate, 1000.0);
        assert_eq!(proc.config.num_range_bins, 16);
    }

    // 2. Range resolution is c / sample_rate.
    #[test]
    fn test_range_resolution() {
        let proc = BistaticProcessor::new(default_config());
        let expected = 299_792_458.0 / 1000.0;
        assert!((proc.range_resolution() - expected).abs() < 1.0);
    }

    // 3. Doppler resolution is 1 / integration_time.
    #[test]
    fn test_doppler_resolution() {
        let proc = BistaticProcessor::new(default_config());
        let expected = 1.0 / 0.016;
        assert!((proc.doppler_resolution() - expected).abs() < 1e-6);
    }

    // 4. Bistatic range for collocated tx/rx and distant target.
    #[test]
    fn test_bistatic_range_collocated() {
        let proc = BistaticProcessor::new(default_config());
        let tx = (0.0, 0.0, 0.0);
        let rx = (0.0, 0.0, 0.0);
        let target = (1000.0, 0.0, 0.0);
        let br = proc.bistatic_range(tx, rx, target);
        // |tx-target| + |target-rx| - |tx-rx| = 1000 + 1000 - 0 = 2000
        assert!((br - 2000.0).abs() < 1e-6);
    }

    // 5. Bistatic range for baseline geometry.
    #[test]
    fn test_bistatic_range_baseline() {
        let proc = BistaticProcessor::new(default_config());
        let tx = (0.0, 0.0, 0.0);
        let rx = (1000.0, 0.0, 0.0);
        let target = (500.0, 500.0, 0.0);
        let d_tx_t = dist3(tx, target);
        let d_t_rx = dist3(target, rx);
        let d_tx_rx = 1000.0;
        let expected = d_tx_t + d_t_rx - d_tx_rx;
        let br = proc.bistatic_range(tx, rx, target);
        assert!((br - expected).abs() < 1e-6);
    }

    // 6. ECA cancellation reduces direct-path power.
    #[test]
    fn test_eca_cancellation_reduces_power() {
        let cfg = BistaticConfig {
            cancellation_order: 10,
            cancellation_method: CancellationMethod::Eca,
            ..default_config()
        };
        let proc = BistaticProcessor::new(cfg);
        let n = 128;
        let reference = tone(n, 100.0, 1000.0);
        // Surveillance = scaled copy of reference (direct path only)
        let surveillance: Vec<(f64, f64)> =
            reference.iter().map(|s| cx_scale(*s, 3.0)).collect();
        let cleaned = proc.cancel_direct_path(&surveillance, &reference);
        let p_before = power(&surveillance);
        let p_after = power(&cleaned);
        // After cancellation, residual power should be much lower.
        assert!(
            p_after < p_before * 0.1,
            "ECA should reduce power: before={p_before:.4}, after={p_after:.4}"
        );
    }

    // 7. LMS cancellation reduces direct-path power.
    #[test]
    fn test_lms_cancellation_reduces_power() {
        let cfg = BistaticConfig {
            cancellation_order: 10,
            cancellation_method: CancellationMethod::Lms,
            ..default_config()
        };
        let proc = BistaticProcessor::new(cfg);
        let n = 256;
        let reference = tone(n, 50.0, 1000.0);
        let surveillance = reference.clone();
        let cleaned = proc.cancel_direct_path(&surveillance, &reference);
        let p_before = power(&surveillance);
        let p_after = power(&cleaned);
        assert!(
            p_after < p_before * 0.5,
            "LMS should reduce power: before={p_before:.4}, after={p_after:.4}"
        );
    }

    // 8. Batch LMS cancellation reduces direct-path power.
    #[test]
    fn test_batch_lms_cancellation() {
        let cfg = BistaticConfig {
            cancellation_order: 10,
            cancellation_method: CancellationMethod::BatchLms,
            ..default_config()
        };
        let proc = BistaticProcessor::new(cfg);
        let n = 256;
        let reference = tone(n, 50.0, 1000.0);
        let surveillance = reference.clone();
        let cleaned = proc.cancel_direct_path(&surveillance, &reference);
        let p_before = power(&surveillance);
        let p_after = power(&cleaned);
        assert!(
            p_after < p_before * 0.5,
            "BatchLMS should reduce power: before={p_before:.4}, after={p_after:.4}"
        );
    }

    // 9. Cross-ambiguity surface has correct dimensions.
    #[test]
    fn test_cas_dimensions() {
        let cfg = BistaticConfig {
            num_range_bins: 20,
            num_doppler_bins: 10,
            ..default_config()
        };
        let proc = BistaticProcessor::new(cfg);
        let n = 64;
        let sig = tone(n, 100.0, 1000.0);
        let cas = proc.cross_ambiguity(&sig, &sig);
        assert_eq!(cas.range_doppler_map.len(), 20);
        assert_eq!(cas.range_doppler_map[0].len(), 10);
    }

    // 10. Cross-ambiguity peak at zero delay and zero Doppler for identical signals.
    #[test]
    fn test_cas_peak_at_zero_delay() {
        let cfg = BistaticConfig {
            num_range_bins: 8,
            num_doppler_bins: 8,
            ..default_config()
        };
        let proc = BistaticProcessor::new(cfg);
        let n = 64;
        let sig = tone(n, 0.0, 1000.0); // DC tone — no Doppler
        let cas = proc.cross_ambiguity(&sig, &sig);

        // Zero delay = range bin 0, zero Doppler = middle bin (nd/2)
        let zero_delay_bin = 0;
        let zero_doppler_bin = 4; // nd/2

        let peak_val = cas.range_doppler_map[zero_delay_bin][zero_doppler_bin];

        // The peak at (0, 0) should be the maximum of the entire surface.
        let mut max_val = 0.0_f64;
        for r in &cas.range_doppler_map {
            for &v in r {
                if v > max_val {
                    max_val = v;
                }
            }
        }
        assert!(
            (peak_val - max_val).abs() < 1e-6,
            "Peak should be at zero delay/Doppler: peak={peak_val:.4}, max={max_val:.4}"
        );
    }

    // 11. Cross-ambiguity surface values are non-negative.
    #[test]
    fn test_cas_non_negative() {
        let proc = BistaticProcessor::new(default_config());
        let n = 32;
        let sig = tone(n, 100.0, 1000.0);
        let cas = proc.cross_ambiguity(&sig, &sig);
        for row in &cas.range_doppler_map {
            for &v in row {
                assert!(v >= 0.0, "CAS values should be non-negative: {v}");
            }
        }
    }

    // 12. Detection on a surface with a clear peak returns at least one detection.
    #[test]
    fn test_detect_targets_with_peak() {
        let cfg = BistaticConfig {
            num_range_bins: 8,
            num_doppler_bins: 8,
            ..default_config()
        };
        let proc = BistaticProcessor::new(cfg);

        // Build a synthetic surface with one dominant cell.
        let nr = 8;
        let nd = 8;
        let mut map = vec![vec![1.0; nd]; nr];
        map[3][4] = 1000.0; // Strong target

        let cas = CrossAmbiguitySurface {
            range_doppler_map: map,
            range_resolution_m: proc.range_resolution(),
            doppler_resolution_hz: proc.doppler_resolution(),
        };

        let dets = proc.detect_targets(&cas, 6.0);
        assert!(!dets.is_empty(), "Should detect the strong target");
        // The strongest detection should be near bin (3, 4).
        let best = dets
            .iter()
            .max_by(|a, b| a.snr_db.partial_cmp(&b.snr_db).unwrap())
            .unwrap();
        let expected_range = 3.0 * proc.range_resolution();
        assert!((best.range_m - expected_range).abs() < 1.0);
    }

    // 13. No detections below threshold on a flat surface.
    #[test]
    fn test_detect_no_false_alarms_flat() {
        let cfg = BistaticConfig {
            num_range_bins: 8,
            num_doppler_bins: 8,
            ..default_config()
        };
        let proc = BistaticProcessor::new(cfg);

        // Perfectly flat surface — no cell exceeds the average.
        let map = vec![vec![1.0; 8]; 8];
        let cas = CrossAmbiguitySurface {
            range_doppler_map: map,
            range_resolution_m: proc.range_resolution(),
            doppler_resolution_hz: proc.doppler_resolution(),
        };

        let dets = proc.detect_targets(&cas, 3.0);
        assert!(dets.is_empty(), "Flat surface should yield no detections");
    }

    // 14. Cancellation preserves signal length.
    #[test]
    fn test_cancellation_preserves_length() {
        let n = 100;
        let sig = tone(n, 50.0, 1000.0);
        for &method in &[
            CancellationMethod::Eca,
            CancellationMethod::Lms,
            CancellationMethod::BatchLms,
        ] {
            let cfg = BistaticConfig {
                cancellation_method: method,
                ..default_config()
            };
            let proc = BistaticProcessor::new(cfg);
            let out = proc.cancel_direct_path(&sig, &sig);
            assert_eq!(out.len(), n, "Method {:?} changed length", method);
        }
    }

    // 15. Empty input signals are handled gracefully.
    #[test]
    fn test_empty_inputs() {
        let proc = BistaticProcessor::new(default_config());
        let empty: Vec<(f64, f64)> = Vec::new();

        let cancelled = proc.cancel_direct_path(&empty, &empty);
        assert!(cancelled.is_empty());

        let cas = proc.cross_ambiguity(&empty, &empty);
        assert_eq!(cas.range_doppler_map.len(), proc.config.num_range_bins);

        let dets = proc.detect_targets(&cas, 3.0);
        assert!(dets.is_empty());
    }

    // 16. Bistatic range is zero when target is on the baseline.
    #[test]
    fn test_bistatic_range_on_baseline() {
        let proc = BistaticProcessor::new(default_config());
        let tx = (0.0, 0.0, 0.0);
        let rx = (100.0, 0.0, 0.0);
        let target = (50.0, 0.0, 0.0);
        let br = proc.bistatic_range(tx, rx, target);
        // |tx-target| + |target-rx| - |tx-rx| = 50 + 50 - 100 = 0
        assert!(
            br.abs() < 1e-10,
            "Bistatic range on baseline should be ~0, got {br}"
        );
    }

    // 17. CAS resolution fields match processor resolutions.
    #[test]
    fn test_cas_resolution_fields() {
        let proc = BistaticProcessor::new(default_config());
        let sig = tone(32, 10.0, 1000.0);
        let cas = proc.cross_ambiguity(&sig, &sig);
        assert!((cas.range_resolution_m - proc.range_resolution()).abs() < 1e-6);
        assert!(
            (cas.doppler_resolution_hz - proc.doppler_resolution()).abs() < 1e-6
        );
    }

    // 18. Detection struct fields are sensible.
    #[test]
    fn test_detection_fields() {
        let det = BistaticDetection {
            range_m: 1500.0,
            doppler_hz: 42.0,
            snr_db: 15.0,
            bistatic_range_m: 1500.0,
        };
        assert_eq!(det.range_m, 1500.0);
        assert_eq!(det.doppler_hz, 42.0);
        assert_eq!(det.snr_db, 15.0);
        assert_eq!(det.bistatic_range_m, 1500.0);
    }

    // 19. Complex arithmetic helpers are correct.
    #[test]
    fn test_complex_helpers() {
        let a = (3.0, 4.0);
        let b = (1.0, -2.0);

        let sum = cx_add(a, b);
        assert!((sum.0 - 4.0).abs() < 1e-12);
        assert!((sum.1 - 2.0).abs() < 1e-12);

        let diff = cx_sub(a, b);
        assert!((diff.0 - 2.0).abs() < 1e-12);
        assert!((diff.1 - 6.0).abs() < 1e-12);

        // (3+4j)(1-2j) = 3 -6j +4j -8j^2 = 11 - 2j
        let prod = cx_mul(a, b);
        assert!((prod.0 - 11.0).abs() < 1e-12);
        assert!((prod.1 - (-2.0)).abs() < 1e-12);

        let conj = cx_conj(a);
        assert!((conj.0 - 3.0).abs() < 1e-12);
        assert!((conj.1 - (-4.0)).abs() < 1e-12);

        let mag = cx_mag_sq(a);
        assert!((mag - 25.0).abs() < 1e-12);

        let scaled = cx_scale(a, 2.0);
        assert!((scaled.0 - 6.0).abs() < 1e-12);
        assert!((scaled.1 - 8.0).abs() < 1e-12);
    }
}
