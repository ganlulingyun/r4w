//! Magnetic Resonance (MR) Spectroscopy Signal Processor
//!
//! This module provides signal processing routines for NMR (Nuclear Magnetic
//! Resonance) and MRI (Magnetic Resonance Imaging) spectroscopy applications.
//! It includes free induction decay (FID) signal generation, relaxation time
//! estimation (T1 and T2), chemical shift computation, line broadening
//! (exponential and Gaussian apodization), phase correction (zero-order and
//! first-order), baseline correction, peak integration, water suppression, and
//! spectral quantification.
//!
//! All complex numbers are represented as `(f64, f64)` tuples where the first
//! element is the real part and the second is the imaginary part.
//!
//! # Example
//!
//! ```
//! use r4w_core::mr_spectroscopy_processor::{generate_fid, MrProcessor, MrConfig};
//!
//! let amplitudes = [1.0];
//! let frequencies = [100.0];
//! let t2_values = [0.05];
//! let dwell_time = 1.0 / 4096.0;
//! let num_points = 512;
//!
//! let fid = generate_fid(&amplitudes, &frequencies, &t2_values, dwell_time, num_points);
//! assert_eq!(fid.len(), num_points);
//!
//! let config = MrConfig {
//!     larmor_freq_hz: 127.74e6,
//!     spectral_width_hz: 4096.0,
//!     num_points: 512,
//!     dwell_time_s: 1.0 / 4096.0,
//! };
//! let processor = MrProcessor::new(config);
//! let spectrum = processor.fft(&fid);
//! assert_eq!(spectrum.real.len(), num_points);
//! ```

use std::f64::consts::PI;

/// Configuration for MR spectroscopy processing.
///
/// Contains the fundamental parameters needed for MR signal acquisition
/// and processing, including the Larmor frequency, spectral width, number
/// of acquisition points, and dwell time.
#[derive(Debug, Clone)]
pub struct MrConfig {
    /// Larmor frequency in Hz (e.g., 127.74 MHz for 3T 1H)
    pub larmor_freq_hz: f64,
    /// Spectral width (bandwidth) in Hz
    pub spectral_width_hz: f64,
    /// Number of acquired time-domain points
    pub num_points: usize,
    /// Dwell time (sampling interval) in seconds
    pub dwell_time_s: f64,
}

/// Represents a frequency-domain MR spectrum.
///
/// Stores the real and imaginary parts of the spectrum along with the
/// corresponding frequency axis in Hz.
#[derive(Debug, Clone)]
pub struct MrSpectrum {
    /// Real part of the spectrum at each frequency bin
    pub real: Vec<f64>,
    /// Imaginary part of the spectrum at each frequency bin
    pub imag: Vec<f64>,
    /// Frequency axis in Hz for each bin
    pub freq_axis_hz: Vec<f64>,
}

/// Main MR spectroscopy processor.
///
/// Provides FFT-based spectral processing with the configuration parameters
/// specified in [`MrConfig`].
#[derive(Debug, Clone)]
pub struct MrProcessor {
    /// Processor configuration
    pub config: MrConfig,
}

impl MrProcessor {
    /// Creates a new MR processor with the given configuration.
    pub fn new(config: MrConfig) -> Self {
        Self { config }
    }

    /// Performs an FFT on the given FID (time-domain) data and returns an [`MrSpectrum`].
    ///
    /// Uses a radix-2 DIT FFT algorithm. The input is zero-padded to the next
    /// power of two if necessary. The frequency axis is computed from the spectral
    /// width and centered around zero.
    pub fn fft(&self, fid: &[(f64, f64)]) -> MrSpectrum {
        let n = self.config.num_points;
        // Pad or truncate to n points
        let mut data: Vec<(f64, f64)> = Vec::with_capacity(n);
        for i in 0..n {
            if i < fid.len() {
                data.push(fid[i]);
            } else {
                data.push((0.0, 0.0));
            }
        }

        // Pad to next power of two for radix-2 FFT
        let fft_len = n.next_power_of_two();
        data.resize(fft_len, (0.0, 0.0));

        let result = fft_radix2(&data);

        // Take first n points and build spectrum
        let mut real = Vec::with_capacity(n);
        let mut imag = Vec::with_capacity(n);
        for i in 0..n {
            real.push(result[i].0);
            imag.push(result[i].1);
        }

        // Build frequency axis
        let freq_axis_hz = build_freq_axis(n, self.config.dwell_time_s);

        MrSpectrum {
            real,
            imag,
            freq_axis_hz,
        }
    }

    /// Computes the magnitude spectrum from an [`MrSpectrum`].
    pub fn magnitude_spectrum(&self, spectrum: &MrSpectrum) -> Vec<f64> {
        spectrum
            .real
            .iter()
            .zip(spectrum.imag.iter())
            .map(|(r, i)| (r * r + i * i).sqrt())
            .collect()
    }
}

/// Generates a multi-component free induction decay (FID) signal.
///
/// Each component is a damped complex exponential with the specified amplitude,
/// frequency offset, and T2 relaxation time.
///
/// # Arguments
///
/// * `amplitudes` - Amplitude of each spectral component
/// * `frequencies_hz` - Frequency offset in Hz for each component
/// * `t2_s` - T2 relaxation time in seconds for each component
/// * `dwell_time_s` - Sampling interval (dwell time) in seconds
/// * `num_points` - Number of time-domain points to generate
///
/// # Returns
///
/// A vector of `(f64, f64)` complex samples representing the FID.
pub fn generate_fid(
    amplitudes: &[f64],
    frequencies_hz: &[f64],
    t2_s: &[f64],
    dwell_time_s: f64,
    num_points: usize,
) -> Vec<(f64, f64)> {
    assert_eq!(amplitudes.len(), frequencies_hz.len());
    assert_eq!(amplitudes.len(), t2_s.len());

    let mut fid = vec![(0.0_f64, 0.0_f64); num_points];

    for ((&amp, &freq), &t2) in amplitudes.iter().zip(frequencies_hz.iter()).zip(t2_s.iter()) {
        for k in 0..num_points {
            let t = k as f64 * dwell_time_s;
            let decay = (-t / t2).exp();
            let phase = 2.0 * PI * freq * t;
            fid[k].0 += amp * decay * phase.cos();
            fid[k].1 += amp * decay * phase.sin();
        }
    }

    fid
}

/// Estimates T2 relaxation time from the magnitude envelope of an FID signal.
///
/// Performs a linear regression on the log of the FID magnitude to find the
/// exponential decay rate. Returns T2 in seconds.
///
/// # Arguments
///
/// * `fid_magnitude` - Magnitude (envelope) of the FID signal at each sample
/// * `dwell_time_s` - Sampling interval in seconds
///
/// # Returns
///
/// Estimated T2 in seconds.
pub fn estimate_t2(fid_magnitude: &[f64], dwell_time_s: f64) -> f64 {
    // Use linear regression on ln(magnitude) vs time
    // ln|S(t)| = ln(A) - t/T2
    // slope = -1/T2

    let n = fid_magnitude.len();
    if n < 2 {
        return 0.0;
    }

    let mut sum_t = 0.0;
    let mut sum_ln = 0.0;
    let mut sum_t2 = 0.0;
    let mut sum_t_ln = 0.0;
    let mut count = 0.0;

    for k in 0..n {
        let mag = fid_magnitude[k];
        if mag <= 0.0 {
            continue;
        }
        let t = k as f64 * dwell_time_s;
        let ln_mag = mag.ln();
        sum_t += t;
        sum_ln += ln_mag;
        sum_t2 += t * t;
        sum_t_ln += t * ln_mag;
        count += 1.0;
    }

    if count < 2.0 {
        return 0.0;
    }

    let slope = (count * sum_t_ln - sum_t * sum_ln) / (count * sum_t2 - sum_t * sum_t);

    if slope >= 0.0 {
        return f64::INFINITY;
    }

    -1.0 / slope
}

/// Estimates T1 relaxation time from inversion recovery data.
///
/// Fits the model `S(TI) = A * (1 - 2 * exp(-TI/T1))` using a simple
/// iterative search over candidate T1 values.
///
/// # Arguments
///
/// * `signal_vs_ti` - Slice of `(TI_seconds, signal_amplitude)` pairs
///
/// # Returns
///
/// Estimated T1 in seconds.
pub fn estimate_t1_inversion_recovery(signal_vs_ti: &[(f64, f64)]) -> f64 {
    if signal_vs_ti.is_empty() {
        return 0.0;
    }

    let mut sorted = signal_vs_ti.to_vec();
    sorted.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    let ti_max = sorted.last().unwrap().0;
    if ti_max <= 0.0 {
        return 0.0;
    }

    // Joint grid search over T1 and A (equilibrium amplitude).
    // The model is S(TI) = A * (1 - 2 * exp(-TI / T1)).
    // We search T1 on a grid then solve for the optimal A analytically.
    let mut best_t1 = ti_max / 2.0;
    let mut best_error = f64::MAX;

    let steps = 2000;
    let t1_min = ti_max * 0.01;
    let t1_max_search = ti_max * 10.0;

    // Helper: for a given T1 candidate, find optimal A via least-squares
    // S_i = A * f_i  =>  A = sum(S_i * f_i) / sum(f_i^2)
    let fit_and_error = |t1_cand: f64| -> (f64, f64) {
        let mut sum_sf = 0.0;
        let mut sum_ff = 0.0;
        for &(ti, sig) in signal_vs_ti {
            let f = 1.0 - 2.0 * (-ti / t1_cand).exp();
            sum_sf += sig * f;
            sum_ff += f * f;
        }
        if sum_ff < 1e-30 {
            return (f64::MAX, 0.0);
        }
        let a_opt = sum_sf / sum_ff;
        let error: f64 = signal_vs_ti
            .iter()
            .map(|&(ti, sig)| {
                let predicted = a_opt * (1.0 - 2.0 * (-ti / t1_cand).exp());
                (sig - predicted).powi(2)
            })
            .sum();
        (error, a_opt)
    };

    // Coarse search
    for i in 0..steps {
        let t1_candidate = t1_min + (t1_max_search - t1_min) * (i as f64 / steps as f64);
        let (error, _) = fit_and_error(t1_candidate);
        if error < best_error {
            best_error = error;
            best_t1 = t1_candidate;
        }
    }

    // Fine search around best candidate
    let step_size = (t1_max_search - t1_min) / steps as f64;
    let fine_min = (best_t1 - step_size * 2.0).max(t1_min);
    let fine_max = best_t1 + step_size * 2.0;

    for i in 0..steps {
        let t1_candidate = fine_min + (fine_max - fine_min) * (i as f64 / steps as f64);
        let (error, _) = fit_and_error(t1_candidate);
        if error < best_error {
            best_error = error;
            best_t1 = t1_candidate;
        }
    }

    best_t1
}

/// Computes the chemical shift in parts per million (ppm).
///
/// Chemical shift is defined as `(freq - reference_freq) / larmor_freq * 1e6`.
///
/// # Arguments
///
/// * `freq_hz` - Observed resonance frequency in Hz
/// * `reference_freq_hz` - Reference frequency (e.g., TMS) in Hz
/// * `larmor_freq_hz` - Larmor frequency of the nucleus in Hz
///
/// # Returns
///
/// Chemical shift in ppm.
pub fn chemical_shift_ppm(freq_hz: f64, reference_freq_hz: f64, larmor_freq_hz: f64) -> f64 {
    (freq_hz - reference_freq_hz) / larmor_freq_hz * 1e6
}

/// Applies exponential (Lorentzian) apodization to an FID signal.
///
/// Multiplies the FID by `exp(-pi * lb * t)` where `lb` is the line broadening
/// in Hz and `t` is the time. This increases the Lorentzian linewidth by `lb` Hz.
///
/// # Arguments
///
/// * `fid` - Mutable slice of complex FID samples to modify in-place
/// * `line_broadening_hz` - Additional line broadening in Hz
/// * `dwell_time_s` - Sampling interval in seconds
pub fn exponential_apodization(fid: &mut [(f64, f64)], line_broadening_hz: f64, dwell_time_s: f64) {
    for (k, sample) in fid.iter_mut().enumerate() {
        let t = k as f64 * dwell_time_s;
        let weight = (-PI * line_broadening_hz * t).exp();
        sample.0 *= weight;
        sample.1 *= weight;
    }
}

/// Applies Gaussian apodization to an FID signal.
///
/// Multiplies the FID by `exp(-pi^2 * sigma^2 * t^2)` where `sigma` is the
/// Gaussian width parameter in Hz. This produces a Gaussian line shape.
///
/// # Arguments
///
/// * `fid` - Mutable slice of complex FID samples to modify in-place
/// * `sigma_hz` - Gaussian broadening parameter in Hz
/// * `dwell_time_s` - Sampling interval in seconds
pub fn gaussian_apodization(fid: &mut [(f64, f64)], sigma_hz: f64, dwell_time_s: f64) {
    for (k, sample) in fid.iter_mut().enumerate() {
        let t = k as f64 * dwell_time_s;
        let weight = (-PI * PI * sigma_hz * sigma_hz * t * t).exp();
        sample.0 *= weight;
        sample.1 *= weight;
    }
}

/// Applies zero-order phase correction to a spectrum.
///
/// Rotates all points in the complex spectrum by a constant phase angle `phi0_rad`.
///
/// # Arguments
///
/// * `spectrum` - Mutable slice of complex spectral points
/// * `phi0_rad` - Zero-order phase correction angle in radians
pub fn phase_correct_zero(spectrum: &mut [(f64, f64)], phi0_rad: f64) {
    let cos_phi = phi0_rad.cos();
    let sin_phi = phi0_rad.sin();

    for sample in spectrum.iter_mut() {
        let re = sample.0;
        let im = sample.1;
        sample.0 = re * cos_phi - im * sin_phi;
        sample.1 = re * sin_phi + im * cos_phi;
    }
}

/// Applies first-order phase correction to a spectrum.
///
/// Applies a linearly varying phase across the spectrum. Point `k` is rotated
/// by `k * phi1_rad_per_point` radians.
///
/// # Arguments
///
/// * `spectrum` - Mutable slice of complex spectral points
/// * `phi1_rad_per_point` - Phase increment per spectral point in radians
pub fn phase_correct_first(spectrum: &mut [(f64, f64)], phi1_rad_per_point: f64) {
    for (k, sample) in spectrum.iter_mut().enumerate() {
        let angle = k as f64 * phi1_rad_per_point;
        let cos_a = angle.cos();
        let sin_a = angle.sin();
        let re = sample.0;
        let im = sample.1;
        sample.0 = re * cos_a - im * sin_a;
        sample.1 = re * sin_a + im * cos_a;
    }
}

/// Applies polynomial baseline correction to a real spectrum.
///
/// Fits a polynomial of the specified order to the spectrum using least-squares
/// regression and subtracts it, removing baseline drift artifacts.
///
/// # Arguments
///
/// * `spectrum` - Mutable slice of real spectral values to correct in-place
/// * `poly_order` - Order of the polynomial to fit (0 = constant, 1 = linear, etc.)
pub fn baseline_correct(spectrum: &mut [f64], poly_order: usize) {
    let n = spectrum.len();
    if n == 0 || poly_order >= n {
        return;
    }

    let order = poly_order + 1;

    // Build the normal equations: (X^T X) c = X^T y
    // Where X is the Vandermonde matrix
    let mut xtx = vec![0.0_f64; order * order];
    let mut xty = vec![0.0_f64; order];

    for i in 0..n {
        // Normalize x to [-1, 1] for numerical stability
        let x = if n > 1 {
            2.0 * (i as f64) / ((n - 1) as f64) - 1.0
        } else {
            0.0
        };
        let y = spectrum[i];

        let mut x_pow = vec![1.0_f64; order];
        for j in 1..order {
            x_pow[j] = x_pow[j - 1] * x;
        }

        for row in 0..order {
            for col in 0..order {
                xtx[row * order + col] += x_pow[row] * x_pow[col];
            }
            xty[row] += x_pow[row] * y;
        }
    }

    // Solve using Gaussian elimination with partial pivoting
    let coeffs = solve_linear_system(&mut xtx, &mut xty, order);

    // Subtract the baseline
    for i in 0..n {
        let x = if n > 1 {
            2.0 * (i as f64) / ((n - 1) as f64) - 1.0
        } else {
            0.0
        };
        let mut baseline = 0.0;
        let mut x_pow = 1.0;
        for c in &coeffs {
            baseline += c * x_pow;
            x_pow *= x;
        }
        spectrum[i] -= baseline;
    }
}

/// Integrates a peak in a real spectrum between the specified bin indices.
///
/// Computes the sum of spectral values from `start_bin` to `end_bin` (exclusive),
/// which approximates the area under the peak using rectangular integration.
///
/// # Arguments
///
/// * `spectrum` - Real spectral values
/// * `start_bin` - Starting bin index (inclusive)
/// * `end_bin` - Ending bin index (exclusive)
///
/// # Returns
///
/// The integrated peak area.
pub fn integrate_peak(spectrum: &[f64], start_bin: usize, end_bin: usize) -> f64 {
    if start_bin >= spectrum.len() || end_bin > spectrum.len() || start_bin >= end_bin {
        return 0.0;
    }
    spectrum[start_bin..end_bin].iter().sum()
}

/// Applies water suppression to an FID signal.
///
/// Removes a narrow band of frequencies centered at `water_freq_hz` with the
/// given bandwidth. This is analogous to HLSVD-based water suppression: the
/// water component is estimated by bandpass filtering and subtracted from the
/// FID in the time domain.
///
/// # Arguments
///
/// * `fid` - Mutable vector of complex FID samples
/// * `water_freq_hz` - Center frequency of the water peak in Hz
/// * `bandwidth_hz` - Width of the suppression band in Hz
/// * `dwell_time_s` - Sampling interval in seconds
pub fn water_suppress(
    fid: &mut Vec<(f64, f64)>,
    water_freq_hz: f64,
    bandwidth_hz: f64,
    dwell_time_s: f64,
) {
    let n = fid.len();
    if n == 0 {
        return;
    }

    // Estimate the water component via a time-domain bandpass approach:
    // Mix down to baseband, lowpass, mix back up, subtract.

    // Step 1: Mix the FID down by the water frequency
    let mut mixed: Vec<(f64, f64)> = Vec::with_capacity(n);
    for k in 0..n {
        let t = k as f64 * dwell_time_s;
        let phase = -2.0 * PI * water_freq_hz * t;
        let cos_p = phase.cos();
        let sin_p = phase.sin();
        let re = fid[k].0 * cos_p - fid[k].1 * sin_p;
        let im = fid[k].0 * sin_p + fid[k].1 * cos_p;
        mixed.push((re, im));
    }

    // Step 2: Apply a simple moving-average lowpass filter
    // The cutoff is approximately bandwidth_hz/2
    let filter_len = ((1.0 / (bandwidth_hz * dwell_time_s)) as usize).max(1).min(n);
    let mut filtered: Vec<(f64, f64)> = vec![(0.0, 0.0); n];

    for k in 0..n {
        let lo = if k >= filter_len / 2 {
            k - filter_len / 2
        } else {
            0
        };
        let hi = (k + filter_len / 2 + 1).min(n);
        let count = hi - lo;

        let mut s_re = 0.0;
        let mut s_im = 0.0;
        for j in lo..hi {
            s_re += mixed[j].0;
            s_im += mixed[j].1;
        }

        filtered[k] = (s_re / count as f64, s_im / count as f64);
    }

    // Step 3: Mix the filtered (water-only) estimate back up
    for k in 0..n {
        let t = k as f64 * dwell_time_s;
        let phase = 2.0 * PI * water_freq_hz * t;
        let cos_p = phase.cos();
        let sin_p = phase.sin();
        let water_re = filtered[k].0 * cos_p - filtered[k].1 * sin_p;
        let water_im = filtered[k].0 * sin_p + filtered[k].1 * cos_p;

        // Subtract water component
        fid[k].0 -= water_re;
        fid[k].1 -= water_im;
    }
}

// ---------------------------------------------------------------------------
// Internal helper functions
// ---------------------------------------------------------------------------

/// Builds a frequency axis in Hz for the given number of points and dwell time.
fn build_freq_axis(n: usize, dwell_time_s: f64) -> Vec<f64> {
    let sw = 1.0 / dwell_time_s; // spectral width
    let df = sw / n as f64;
    (0..n)
        .map(|k| {
            if k <= n / 2 {
                k as f64 * df
            } else {
                (k as f64 - n as f64) * df
            }
        })
        .collect()
}

/// In-place radix-2 decimation-in-time FFT.
fn fft_radix2(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    assert!(n.is_power_of_two(), "FFT length must be a power of two");

    let mut data = input.to_vec();

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

    // Cooley-Tukey butterfly
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle_step = -2.0 * PI / len as f64;

        for start in (0..n).step_by(len) {
            for k in 0..half {
                let angle = angle_step * k as f64;
                let w = (angle.cos(), angle.sin());

                let idx_even = start + k;
                let idx_odd = start + k + half;

                let even = data[idx_even];
                let odd = data[idx_odd];

                // Complex multiply: w * odd
                let t_re = w.0 * odd.0 - w.1 * odd.1;
                let t_im = w.0 * odd.1 + w.1 * odd.0;

                data[idx_even] = (even.0 + t_re, even.1 + t_im);
                data[idx_odd] = (even.0 - t_re, even.1 - t_im);
            }
        }

        len <<= 1;
    }

    data
}

/// Solves a linear system Ax = b using Gaussian elimination with partial pivoting.
/// `a` is the flattened n x n matrix, `b` is the right-hand side. Both are modified
/// in place. Returns the solution vector.
fn solve_linear_system(a: &mut [f64], b: &mut [f64], n: usize) -> Vec<f64> {
    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = a[col * n + col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            let val = a[row * n + col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        // Swap rows
        if max_row != col {
            for j in 0..n {
                let tmp = a[col * n + j];
                a[col * n + j] = a[max_row * n + j];
                a[max_row * n + j] = tmp;
            }
            let tmp = b[col];
            b[col] = b[max_row];
            b[max_row] = tmp;
        }

        let pivot = a[col * n + col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        // Eliminate below
        for row in (col + 1)..n {
            let factor = a[row * n + col] / pivot;
            for j in col..n {
                a[row * n + j] -= factor * a[col * n + j];
            }
            b[row] -= factor * b[col];
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for col in (0..n).rev() {
        let pivot = a[col * n + col];
        if pivot.abs() < 1e-30 {
            x[col] = 0.0;
            continue;
        }
        let mut sum = b[col];
        for j in (col + 1)..n {
            sum -= a[col * n + j] * x[j];
        }
        x[col] = sum / pivot;
    }

    x
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;

    #[test]
    fn test_generate_fid_single_component() {
        let fid = generate_fid(&[1.0], &[0.0], &[1.0], 0.001, 100);
        assert_eq!(fid.len(), 100);
        // At t=0, should be amplitude 1.0 + 0i
        assert!((fid[0].0 - 1.0).abs() < TOL);
        assert!(fid[0].1.abs() < TOL);
    }

    #[test]
    fn test_generate_fid_decay() {
        let t2 = 0.1;
        let dt = 0.001;
        let fid = generate_fid(&[1.0], &[0.0], &[t2], dt, 200);
        // At t = t2, magnitude should be ~1/e
        let idx = (t2 / dt) as usize;
        let mag = (fid[idx].0.powi(2) + fid[idx].1.powi(2)).sqrt();
        let expected = (-1.0_f64).exp(); // 1/e
        assert!((mag - expected).abs() < 0.01);
    }

    #[test]
    fn test_generate_fid_multi_component() {
        let fid = generate_fid(&[1.0, 0.5], &[100.0, -200.0], &[0.1, 0.05], 0.001, 50);
        assert_eq!(fid.len(), 50);
        // At t=0, all cos terms start at amplitude sum, sin terms at 0
        assert!((fid[0].0 - 1.5).abs() < TOL);
        assert!(fid[0].1.abs() < TOL);
    }

    #[test]
    fn test_generate_fid_frequency_oscillation() {
        let freq = 250.0;
        let dt = 0.0001;
        let fid = generate_fid(&[1.0], &[freq], &[10.0], dt, 1000);
        // At t = 1/(4*freq) = 0.001s => sample 10, phase = pi/2
        // real ~ cos(pi/2) ~ 0, imag ~ sin(pi/2) ~ 1 (modulo decay)
        let idx = 10;
        let t = idx as f64 * dt;
        let decay = (-t / 10.0).exp();
        assert!(fid[idx].0.abs() < 0.05);
        assert!((fid[idx].1 - decay).abs() < 0.05);
    }

    #[test]
    fn test_estimate_t2() {
        let t2_true = 0.05;
        let dt = 0.001;
        let n = 200;
        let fid = generate_fid(&[1.0], &[0.0], &[t2_true], dt, n);
        let magnitudes: Vec<f64> = fid.iter().map(|s| (s.0.powi(2) + s.1.powi(2)).sqrt()).collect();
        let t2_est = estimate_t2(&magnitudes, dt);
        assert!((t2_est - t2_true).abs() < 0.005, "T2 estimate {} vs true {}", t2_est, t2_true);
    }

    #[test]
    fn test_estimate_t2_short_signal() {
        let result = estimate_t2(&[1.0], 0.001);
        assert_eq!(result, 0.0); // Too few points
    }

    #[test]
    fn test_estimate_t1_inversion_recovery() {
        let t1_true = 1.0;
        let a = 5.0;
        // Generate IR curve: S(TI) = A * (1 - 2*exp(-TI/T1))
        let ti_values: Vec<f64> = (1..=20).map(|i| i as f64 * 0.1).collect();
        let data: Vec<(f64, f64)> = ti_values
            .iter()
            .map(|&ti| (ti, a * (1.0 - 2.0 * (-ti / t1_true).exp())))
            .collect();

        let t1_est = estimate_t1_inversion_recovery(&data);
        assert!(
            (t1_est - t1_true).abs() < 0.05,
            "T1 estimate {} vs true {}",
            t1_est,
            t1_true
        );
    }

    #[test]
    fn test_estimate_t1_empty() {
        let result = estimate_t1_inversion_recovery(&[]);
        assert_eq!(result, 0.0);
    }

    #[test]
    fn test_chemical_shift_ppm() {
        let larmor = 400e6; // 400 MHz
        let reference = 400e6;
        let freq = 400.001e6; // 1 kHz offset
        let ppm = chemical_shift_ppm(freq, reference, larmor);
        // 1000 / 400e6 * 1e6 = 2.5 ppm
        assert!((ppm - 2.5).abs() < TOL);
    }

    #[test]
    fn test_chemical_shift_negative() {
        let larmor = 400e6;
        let ppm = chemical_shift_ppm(399.999e6, 400e6, larmor);
        assert!((ppm - (-2.5)).abs() < TOL);
    }

    #[test]
    fn test_exponential_apodization() {
        let dt = 0.001;
        let lb = 10.0;
        let mut fid = vec![(1.0, 0.0); 100];
        exponential_apodization(&mut fid, lb, dt);

        // At t=0, weight = 1.0
        assert!((fid[0].0 - 1.0).abs() < TOL);
        // At t = 0.01s (k=10), weight = exp(-pi*10*0.01)
        let expected = (-PI * lb * 0.01).exp();
        assert!((fid[10].0 - expected).abs() < TOL);
    }

    #[test]
    fn test_gaussian_apodization() {
        let dt = 0.001;
        let sigma = 5.0;
        let mut fid = vec![(1.0, 0.0); 100];
        gaussian_apodization(&mut fid, sigma, dt);

        // At t=0, weight = 1.0
        assert!((fid[0].0 - 1.0).abs() < TOL);
        // At k=10, t=0.01s, weight = exp(-pi^2 * 25 * 0.0001)
        let expected = (-PI * PI * sigma * sigma * 0.01 * 0.01).exp();
        assert!((fid[10].0 - expected).abs() < TOL);
    }

    #[test]
    fn test_phase_correct_zero() {
        let mut spectrum = vec![(1.0, 0.0)];
        phase_correct_zero(&mut spectrum, PI / 2.0);
        // Rotating (1,0) by pi/2 gives (0,1)
        assert!(spectrum[0].0.abs() < TOL);
        assert!((spectrum[0].1 - 1.0).abs() < TOL);
    }

    #[test]
    fn test_phase_correct_zero_identity() {
        let mut spectrum = vec![(3.0, 4.0), (-1.0, 2.0)];
        let original = spectrum.clone();
        phase_correct_zero(&mut spectrum, 0.0);
        for (s, o) in spectrum.iter().zip(original.iter()) {
            assert!((s.0 - o.0).abs() < TOL);
            assert!((s.1 - o.1).abs() < TOL);
        }
    }

    #[test]
    fn test_phase_correct_first() {
        let phi1 = PI / 4.0;
        let mut spectrum = vec![(1.0, 0.0), (1.0, 0.0), (1.0, 0.0)];
        phase_correct_first(&mut spectrum, phi1);

        // Point 0: angle=0, no rotation
        assert!((spectrum[0].0 - 1.0).abs() < TOL);
        assert!(spectrum[0].1.abs() < TOL);

        // Point 1: angle=pi/4
        assert!((spectrum[1].0 - (PI / 4.0).cos()).abs() < TOL);
        assert!((spectrum[1].1 - (PI / 4.0).sin()).abs() < TOL);

        // Point 2: angle=pi/2
        assert!(spectrum[2].0.abs() < TOL);
        assert!((spectrum[2].1 - 1.0).abs() < TOL);
    }

    #[test]
    fn test_baseline_correct_constant() {
        let mut spectrum = vec![10.0; 50];
        baseline_correct(&mut spectrum, 0);
        // After removing constant baseline, all values should be near 0
        for val in &spectrum {
            assert!(val.abs() < TOL, "Expected ~0, got {}", val);
        }
    }

    #[test]
    fn test_baseline_correct_linear() {
        let n = 100;
        // Create a linear ramp plus a peak
        let mut spectrum: Vec<f64> = (0..n).map(|i| 2.0 * i as f64 + 5.0).collect();
        // Add a peak
        spectrum[50] += 100.0;

        baseline_correct(&mut spectrum, 1);

        // The peak should remain prominent, the linear baseline removed
        // Non-peak points should be near zero
        assert!(spectrum[0].abs() < 1.0);
        assert!(spectrum[50] > 90.0); // Peak still present
    }

    #[test]
    fn test_integrate_peak() {
        let spectrum = vec![0.0, 1.0, 2.0, 3.0, 2.0, 1.0, 0.0];
        let area = integrate_peak(&spectrum, 1, 6);
        assert!((area - 9.0).abs() < TOL);
    }

    #[test]
    fn test_integrate_peak_empty_range() {
        let spectrum = vec![1.0, 2.0, 3.0];
        assert_eq!(integrate_peak(&spectrum, 2, 2), 0.0);
        assert_eq!(integrate_peak(&spectrum, 3, 5), 0.0);
    }

    #[test]
    fn test_water_suppress() {
        let dt = 1.0 / 4096.0;
        let n = 4096;
        let water_freq = 0.0; // Water at center
        let metabolite_freq = 500.0;

        // Create FID with strong water and weak metabolite
        let mut fid = generate_fid(
            &[100.0, 1.0],
            &[water_freq, metabolite_freq],
            &[0.5, 0.05],
            dt,
            n,
        );

        let initial_power: f64 = fid.iter().map(|s| s.0.powi(2) + s.1.powi(2)).sum();

        water_suppress(&mut fid, water_freq, 50.0, dt);

        let final_power: f64 = fid.iter().map(|s| s.0.powi(2) + s.1.powi(2)).sum();

        // Water suppression should significantly reduce total signal power
        // since water is the dominant component
        assert!(
            final_power < initial_power * 0.5,
            "Water suppression did not reduce power enough: {} vs {}",
            final_power,
            initial_power
        );
    }

    #[test]
    fn test_mr_processor_fft_single_tone() {
        let freq = 100.0;
        let dt = 1.0 / 1024.0;
        let n = 1024;
        let fid = generate_fid(&[1.0], &[freq], &[100.0], dt, n);

        let config = MrConfig {
            larmor_freq_hz: 400e6,
            spectral_width_hz: 1.0 / dt,
            num_points: n,
            dwell_time_s: dt,
        };
        let processor = MrProcessor::new(config);
        let spectrum = processor.fft(&fid);

        assert_eq!(spectrum.real.len(), n);
        assert_eq!(spectrum.imag.len(), n);
        assert_eq!(spectrum.freq_axis_hz.len(), n);

        // Find the peak in the magnitude spectrum
        let mag = processor.magnitude_spectrum(&spectrum);
        let peak_bin = mag
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap()
            .0;

        // Peak should be near the expected frequency bin
        let expected_bin = (freq * n as f64 * dt) as usize;
        assert!(
            (peak_bin as i64 - expected_bin as i64).unsigned_abs() <= 2,
            "Peak at bin {} expected near {}",
            peak_bin,
            expected_bin
        );
    }

    #[test]
    fn test_mr_processor_magnitude_spectrum() {
        let config = MrConfig {
            larmor_freq_hz: 400e6,
            spectral_width_hz: 1000.0,
            num_points: 64,
            dwell_time_s: 0.001,
        };
        let processor = MrProcessor::new(config);

        let spectrum = MrSpectrum {
            real: vec![3.0; 64],
            imag: vec![4.0; 64],
            freq_axis_hz: vec![0.0; 64],
        };
        let mag = processor.magnitude_spectrum(&spectrum);
        for m in &mag {
            assert!((m - 5.0).abs() < TOL);
        }
    }

    #[test]
    fn test_fft_radix2_dc() {
        // All-ones signal => DC peak at bin 0
        let input = vec![(1.0, 0.0); 8];
        let result = fft_radix2(&input);
        assert!((result[0].0 - 8.0).abs() < TOL);
        assert!(result[0].1.abs() < TOL);
        // Other bins should be ~0
        for k in 1..8 {
            assert!(result[k].0.abs() < TOL, "Bin {} real not zero: {}", k, result[k].0);
            assert!(result[k].1.abs() < TOL, "Bin {} imag not zero: {}", k, result[k].1);
        }
    }

    #[test]
    fn test_baseline_correct_quadratic() {
        let n = 200;
        // Quadratic baseline + constant signal
        let mut spectrum: Vec<f64> = (0..n)
            .map(|i| {
                let x = i as f64 / n as f64;
                3.0 * x * x - 2.0 * x + 1.0
            })
            .collect();

        baseline_correct(&mut spectrum, 2);

        // After quadratic correction, values should be near zero
        let max_residual = spectrum.iter().map(|v| v.abs()).fold(0.0_f64, f64::max);
        assert!(
            max_residual < 0.1,
            "Quadratic baseline not fully removed, max residual: {}",
            max_residual
        );
    }

    #[test]
    fn test_exponential_apodization_complex() {
        let dt = 0.001;
        let lb = 5.0;
        let mut fid = vec![(1.0, 1.0); 50];
        exponential_apodization(&mut fid, lb, dt);

        // Both real and imaginary should be scaled equally
        for k in 0..50 {
            let t = k as f64 * dt;
            let w = (-PI * lb * t).exp();
            assert!((fid[k].0 - w).abs() < TOL);
            assert!((fid[k].1 - w).abs() < TOL);
        }
    }
}
