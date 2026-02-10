//! Adaptive RF interference mitigation through phased-array nulling, spectral excision,
//! and temporal blanking.
//!
//! This module provides algorithms for suppressing RF interference sources using
//! spatial (beamforming), spectral (notch filtering), and temporal (pulse blanking)
//! techniques. Key capabilities include:
//!
//! - **Sample Matrix Inversion (SMI)** beamforming for adaptive weight computation
//! - **Power Inversion** adaptive nulling that automatically steers nulls toward interferers
//! - **Spectral Excision** via FFT-domain notch filtering of narrowband interference
//! - **Temporal Blanking** for pulse interference removal
//! - **Spatial-Temporal Interference Cancellation** combining spatial and temporal methods
//! - **INR Estimation** for measuring interference-to-noise ratio
//! - **Null Depth Computation** for verifying interference suppression
//!
//! All complex numbers use `(f64, f64)` tuples representing `(real, imaginary)`.
//!
//! # Example
//!
//! ```
//! use r4w_core::rf_mitigation_adaptive_nulling::{
//!     AdaptiveNuller, steering_vector, spatial_covariance, smi_beamform,
//! };
//!
//! let nuller = AdaptiveNuller::new(4, 1e6, 0.03);
//!
//! // Create a steering vector for broadside (0 radians)
//! let sv = steering_vector(0.0, 4, 0.015, 0.03);
//! assert_eq!(sv.len(), 4);
//!
//! // Generate simple snapshots (identity-like covariance)
//! let snapshots: Vec<Vec<(f64, f64)>> = (0..100)
//!     .map(|i| {
//!         let phase = i as f64 * 0.1;
//!         sv.iter().map(|&(re, im)| {
//!             let cos_p = phase.cos();
//!             let sin_p = phase.sin();
//!             (re * cos_p - im * sin_p, re * sin_p + im * cos_p)
//!         }).collect()
//!     })
//!     .collect();
//!
//! let weights = smi_beamform(&snapshots, &sv);
//! assert_eq!(weights.len(), 4);
//! ```

use std::f64::consts::PI;

/// Result of adaptive nulling processing.
#[derive(Debug, Clone)]
pub struct NullResult {
    /// Output samples after interference suppression.
    pub output_samples: Vec<(f64, f64)>,
    /// Null depth in dB for each interferer direction.
    pub null_depths_db: Vec<f64>,
    /// SINR improvement in dB from the nulling process.
    pub sinr_improvement_db: f64,
}

/// Adaptive nuller for phased-array interference suppression.
///
/// Combines spatial, spectral, and temporal mitigation techniques to suppress
/// RF interference from multiple sources.
#[derive(Debug, Clone)]
pub struct AdaptiveNuller {
    /// Number of antenna elements in the array.
    pub num_elements: usize,
    /// Sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Wavelength in meters at the operating frequency.
    pub wavelength_m: f64,
}

impl AdaptiveNuller {
    /// Creates a new `AdaptiveNuller`.
    ///
    /// # Arguments
    ///
    /// * `num_elements` - Number of antenna elements in the uniform linear array
    /// * `sample_rate_hz` - Sample rate in Hz
    /// * `wavelength_m` - Operating wavelength in meters
    pub fn new(num_elements: usize, sample_rate_hz: f64, wavelength_m: f64) -> Self {
        Self {
            num_elements,
            sample_rate_hz,
            wavelength_m,
        }
    }

    /// Performs full adaptive nulling combining spatial and spectral techniques.
    ///
    /// Applies SMI beamforming to suppress spatial interferers, then applies
    /// spectral excision for any remaining narrowband interference.
    ///
    /// # Arguments
    ///
    /// * `snapshots` - Array snapshots, each a vector of complex samples per element
    /// * `desired_doa_rad` - Direction of arrival of the desired signal in radians
    /// * `interferer_doas_rad` - Directions of arrival of interferers in radians
    /// * `notch_freqs_hz` - Optional narrowband interference frequencies for spectral excision
    /// * `notch_bw_hz` - Bandwidth of spectral notches in Hz
    pub fn process(
        &self,
        snapshots: &[Vec<(f64, f64)>],
        desired_doa_rad: f64,
        interferer_doas_rad: &[f64],
        notch_freqs_hz: &[f64],
        notch_bw_hz: f64,
    ) -> NullResult {
        let element_spacing = self.wavelength_m / 2.0;

        // Compute steering vector for desired signal
        let steer = steering_vector(
            desired_doa_rad,
            self.num_elements,
            element_spacing,
            self.wavelength_m,
        );

        // Compute adaptive weights using SMI
        let weights = smi_beamform(snapshots, &steer);

        // Apply beamforming weights to each snapshot to produce output
        let mut output_samples: Vec<(f64, f64)> = snapshots
            .iter()
            .map(|snap| {
                // output = w^H * x (conjugate inner product)
                let mut re = 0.0;
                let mut im = 0.0;
                for (i, &(wr, wi)) in weights.iter().enumerate() {
                    if i < snap.len() {
                        let (xr, xi) = snap[i];
                        // conjugate of weight times sample
                        re += wr * xr + wi * xi;
                        im += wr * xi - wi * xr;
                    }
                }
                (re, im)
            })
            .collect();

        // Apply spectral excision if notch frequencies provided
        if !notch_freqs_hz.is_empty() {
            output_samples = spectral_excise(
                &output_samples,
                notch_freqs_hz,
                notch_bw_hz,
                self.sample_rate_hz,
            );
        }

        // Compute null depths for each interferer
        let null_depths_db: Vec<f64> = interferer_doas_rad
            .iter()
            .map(|&doa| {
                compute_null_depth(&weights, doa, self.wavelength_m, element_spacing)
            })
            .collect();

        // Estimate SINR improvement
        let inr = estimate_inr(snapshots, &steer);
        let sinr_improvement_db = 10.0 * (1.0 + inr).log10();

        NullResult {
            output_samples,
            null_depths_db,
            sinr_improvement_db,
        }
    }
}

// ---------------------------------------------------------------------------
// Complex arithmetic helpers
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

#[inline]
fn cx_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = cx_mag_sq(b);
    if denom < 1e-30 {
        (0.0, 0.0)
    } else {
        ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
    }
}

// ---------------------------------------------------------------------------
// Matrix operations for complex matrices stored as Vec<Vec<(f64,f64)>>
// ---------------------------------------------------------------------------

/// Inverts a complex matrix using Gauss-Jordan elimination.
fn mat_invert(mat: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
    let n = mat.len();
    // Build augmented matrix [A | I]
    let mut aug: Vec<Vec<(f64, f64)>> = vec![vec![(0.0, 0.0); 2 * n]; n];
    for i in 0..n {
        for j in 0..n {
            aug[i][j] = mat[i][j];
        }
        aug[i][n + i] = (1.0, 0.0);
    }

    for col in 0..n {
        // Partial pivoting: find row with largest magnitude in this column
        let mut max_mag = cx_mag_sq(aug[col][col]);
        let mut max_row = col;
        for row in (col + 1)..n {
            let mag = cx_mag_sq(aug[row][col]);
            if mag > max_mag {
                max_mag = mag;
                max_row = row;
            }
        }
        if max_mag < 1e-30 {
            // Singular or near-singular: add diagonal loading
            aug[col][col] = cx_add(aug[col][col], (1e-6, 0.0));
        }
        if max_row != col {
            aug.swap(col, max_row);
        }

        let pivot = aug[col][col];
        // Scale pivot row
        for j in 0..(2 * n) {
            aug[col][j] = cx_div(aug[col][j], pivot);
        }

        // Eliminate other rows
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            for j in 0..(2 * n) {
                let val = cx_mul(factor, aug[col][j]);
                aug[row][j] = cx_sub(aug[row][j], val);
            }
        }
    }

    // Extract right half as inverse
    let mut inv = vec![vec![(0.0, 0.0); n]; n];
    for i in 0..n {
        for j in 0..n {
            inv[i][j] = aug[i][n + j];
        }
    }
    inv
}

/// Matrix-vector multiply: y = A * x.
fn mat_vec_mul(mat: &[Vec<(f64, f64)>], vec_in: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = mat.len();
    let mut result = vec![(0.0, 0.0); n];
    for i in 0..n {
        for j in 0..mat[i].len().min(vec_in.len()) {
            result[i] = cx_add(result[i], cx_mul(mat[i][j], vec_in[j]));
        }
    }
    result
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/// Computes the spatial covariance matrix from array snapshots.
///
/// The covariance matrix R is estimated as:
/// `R = (1/K) * sum(x[k] * x[k]^H)` for k = 0..K-1
///
/// where K is the number of snapshots and x[k] is the element vector for snapshot k.
///
/// # Arguments
///
/// * `snapshots` - Array of snapshots; each snapshot is a vector of complex samples,
///   one per antenna element
///
/// # Returns
///
/// An N x N complex covariance matrix where N is the number of antenna elements.
pub fn spatial_covariance(snapshots: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
    if snapshots.is_empty() {
        return vec![];
    }
    let n = snapshots[0].len();
    let k = snapshots.len() as f64;
    let mut cov = vec![vec![(0.0, 0.0); n]; n];

    for snap in snapshots {
        for i in 0..n {
            for j in 0..n {
                // R += x * x^H  =>  R[i][j] += x[i] * conj(x[j])
                cov[i][j] = cx_add(cov[i][j], cx_mul(snap[i], cx_conj(snap[j])));
            }
        }
    }

    // Normalize
    for i in 0..n {
        for j in 0..n {
            cov[i][j] = cx_scale(cov[i][j], 1.0 / k);
        }
    }

    cov
}

/// Computes the array steering vector for a uniform linear array.
///
/// The steering vector for direction of arrival `doa_rad` is:
/// `a[k] = exp(-j * 2 * pi * k * d * sin(doa) / lambda)` for k = 0..N-1
///
/// # Arguments
///
/// * `doa_rad` - Direction of arrival in radians (0 = broadside)
/// * `num_elements` - Number of array elements
/// * `element_spacing_m` - Spacing between elements in meters
/// * `wavelength_m` - Operating wavelength in meters
///
/// # Returns
///
/// Vector of complex steering weights, one per element.
pub fn steering_vector(
    doa_rad: f64,
    num_elements: usize,
    element_spacing_m: f64,
    wavelength_m: f64,
) -> Vec<(f64, f64)> {
    let phase_inc = -2.0 * PI * element_spacing_m * doa_rad.sin() / wavelength_m;
    (0..num_elements)
        .map(|k| {
            let phase = phase_inc * k as f64;
            (phase.cos(), phase.sin())
        })
        .collect()
}

/// Performs Sample Matrix Inversion (SMI) beamforming.
///
/// Computes optimal weights using the Capon/MVDR criterion:
/// `w = R^{-1} * a / (a^H * R^{-1} * a)`
///
/// where R is the sample covariance matrix and a is the steering vector.
///
/// # Arguments
///
/// * `snapshots` - Array snapshots for covariance estimation
/// * `steering` - Steering vector toward the desired signal direction
///
/// # Returns
///
/// Vector of complex beamforming weights.
pub fn smi_beamform(
    snapshots: &[Vec<(f64, f64)>],
    steering: &[(f64, f64)],
) -> Vec<(f64, f64)> {
    if snapshots.is_empty() || steering.is_empty() {
        return vec![];
    }

    let n = steering.len();

    // Compute covariance matrix with diagonal loading for numerical stability
    let mut cov = spatial_covariance(snapshots);

    // Diagonal loading: R_loaded = R + sigma^2 * I
    let trace: f64 = (0..n).map(|i| cov[i][i].0).sum();
    let loading = trace * 1e-4 / n as f64;
    for i in 0..n {
        cov[i][i].0 += loading;
    }

    // Invert covariance
    let r_inv = mat_invert(&cov);

    // w_unnorm = R^{-1} * a
    let w_unnorm = mat_vec_mul(&r_inv, steering);

    // Normalise: denom = a^H * R^{-1} * a
    let mut denom = (0.0, 0.0);
    for i in 0..n {
        denom = cx_add(denom, cx_mul(cx_conj(steering[i]), w_unnorm[i]));
    }

    w_unnorm
        .iter()
        .map(|&w| cx_div(w, denom))
        .collect()
}

/// Performs Power Inversion adaptive nulling.
///
/// Uses the power inversion algorithm which minimizes total output power
/// while maintaining a constraint on the desired signal direction:
/// `w = R^{-1} * a(doa_desired)`
///
/// This naturally places nulls toward high-power interferers.
///
/// # Arguments
///
/// * `snapshots` - Array snapshots for covariance estimation
/// * `desired_doa_rad` - Direction of arrival of the desired signal in radians
///
/// # Returns
///
/// Vector of complex adaptive weights.
pub fn power_inversion_null(
    snapshots: &[Vec<(f64, f64)>],
    desired_doa_rad: f64,
) -> Vec<(f64, f64)> {
    if snapshots.is_empty() {
        return vec![];
    }

    let n = snapshots[0].len();
    let element_spacing = 0.5; // Half-wavelength spacing (normalised to wavelength)
    let wavelength = 1.0; // Normalised wavelength

    let steer = steering_vector(desired_doa_rad, n, element_spacing, wavelength);
    smi_beamform(snapshots, &steer)
}

/// Performs FFT-based spectral excision of narrowband interference.
///
/// Transforms the signal to the frequency domain, zeroes out bins around
/// the specified notch frequencies, then transforms back. This effectively
/// removes narrowband interferers from the signal.
///
/// # Arguments
///
/// * `samples` - Input complex samples
/// * `notch_freqs_hz` - Center frequencies of interference to excise in Hz
/// * `notch_bw_hz` - Bandwidth of each notch in Hz
/// * `sample_rate_hz` - Sample rate in Hz
///
/// # Returns
///
/// Filtered complex samples with interference removed.
pub fn spectral_excise(
    samples: &[(f64, f64)],
    notch_freqs_hz: &[f64],
    notch_bw_hz: f64,
    sample_rate_hz: f64,
) -> Vec<(f64, f64)> {
    if samples.is_empty() {
        return vec![];
    }

    let n = samples.len();

    // Pad to next power of 2 for efficient FFT
    let fft_size = n.next_power_of_two();

    // Forward FFT (DFT)
    let mut freq = dft_forward(samples, fft_size);

    // Excise bins around notch frequencies
    let bin_resolution = sample_rate_hz / fft_size as f64;
    let half_notch_bins = ((notch_bw_hz / 2.0) / bin_resolution).ceil() as usize;

    for &freq_hz in notch_freqs_hz {
        // Map frequency to bin index
        let mut normalized_freq = freq_hz / sample_rate_hz;
        // Handle negative frequencies by wrapping
        if normalized_freq < 0.0 {
            normalized_freq += 1.0;
        }
        let center_bin = (normalized_freq * fft_size as f64).round() as isize;

        // Zero out bins in the notch region with a smooth taper at edges
        for offset in -(half_notch_bins as isize)..=(half_notch_bins as isize) {
            let bin = ((center_bin + offset) % fft_size as isize + fft_size as isize) as usize
                % fft_size;
            // Apply Hann-shaped taper at edges for smoother transition
            let edge_dist = (offset.unsigned_abs() as f64) / (half_notch_bins as f64 + 1.0);
            if edge_dist < 0.8 {
                freq[bin] = (0.0, 0.0);
            } else {
                let taper = (edge_dist - 0.8) / 0.2;
                freq[bin] = cx_scale(freq[bin], taper);
            }
        }
    }

    // Inverse FFT
    let result = dft_inverse(&freq, fft_size);

    // Truncate to original length
    result[..n].to_vec()
}

/// Performs temporal blanking to remove pulse interference.
///
/// Samples whose magnitude exceeds the threshold are replaced with zeros.
/// This is effective against pulsed interference such as radar or ignition noise.
///
/// # Arguments
///
/// * `samples` - Input complex samples
/// * `threshold` - Power threshold; samples with magnitude squared exceeding this are blanked
///
/// # Returns
///
/// Samples with pulse interference blanked (replaced by zeros).
pub fn temporal_blank(samples: &[(f64, f64)], threshold: f64) -> Vec<(f64, f64)> {
    samples
        .iter()
        .map(|&s| {
            if cx_mag_sq(s) > threshold {
                (0.0, 0.0)
            } else {
                s
            }
        })
        .collect()
}

/// Estimates the interference-to-noise ratio (INR).
///
/// Computes INR by comparing the total received power to the noise floor,
/// estimated from the minimum eigenvalue of the covariance matrix (approximated
/// by the smallest diagonal element after whitening).
///
/// # Arguments
///
/// * `snapshots` - Array snapshots
/// * `steering` - Steering vector toward the desired signal
///
/// # Returns
///
/// Estimated INR in linear scale.
pub fn estimate_inr(snapshots: &[Vec<(f64, f64)>], steering: &[(f64, f64)]) -> f64 {
    if snapshots.is_empty() || steering.is_empty() {
        return 0.0;
    }

    let cov = spatial_covariance(snapshots);
    let n = cov.len();

    // Total power: trace of covariance matrix
    let total_power: f64 = (0..n).map(|i| cov[i][i].0).sum();

    // Noise floor estimate: minimum diagonal element of covariance matrix
    // For a well-conditioned array, this approximates the noise-only eigenvalue
    let noise_floor = (0..n)
        .map(|i| cov[i][i].0)
        .fold(f64::INFINITY, f64::min);

    // Total noise power across all elements
    let noise_total = noise_floor * n as f64;

    if noise_total < 1e-30 {
        return 0.0;
    }

    // INR = (total_power - noise_total) / noise_total
    // This captures all non-noise power (signal + interference) relative to noise.
    // In the presence of strong interference, this ratio will be dominated by interference.
    let excess_power = (total_power - noise_total).max(0.0);
    excess_power / noise_total
}

/// Computes the null depth in dB at a given interferer direction.
///
/// The null depth is the array response at the interferer direction relative
/// to the peak response:
/// `null_depth = 20 * log10(|w^H * a(doa_interferer)| / |w^H * a(doa_peak)|)`
///
/// More negative values indicate deeper nulls (better suppression).
///
/// # Arguments
///
/// * `weights` - Beamforming weight vector
/// * `interferer_doa_rad` - Direction of the interferer in radians
/// * `wavelength_m` - Operating wavelength in meters
/// * `element_spacing_m` - Element spacing in meters
///
/// # Returns
///
/// Null depth in dB (typically a negative number).
pub fn compute_null_depth(
    weights: &[(f64, f64)],
    interferer_doa_rad: f64,
    wavelength_m: f64,
    element_spacing_m: f64,
) -> f64 {
    let n = weights.len();
    let a_interf = steering_vector(interferer_doa_rad, n, element_spacing_m, wavelength_m);

    // Compute w^H * a(interferer)
    let mut response = (0.0, 0.0);
    for i in 0..n {
        response = cx_add(response, cx_mul(cx_conj(weights[i]), a_interf[i]));
    }
    let resp_mag = cx_mag_sq(response).sqrt();

    // Compute peak response (at broadside for reference, or use max over scan)
    // We normalize relative to the quiescent pattern peak
    // For a uniform weight vector, peak response = N
    // For adapted weights, compute response at broadside (0 rad)
    let a_broad = steering_vector(0.0, n, element_spacing_m, wavelength_m);
    let mut peak_response = (0.0, 0.0);
    for i in 0..n {
        peak_response = cx_add(peak_response, cx_mul(cx_conj(weights[i]), a_broad[i]));
    }
    let peak_mag = cx_mag_sq(peak_response).sqrt();

    if peak_mag < 1e-30 {
        return -100.0;
    }

    20.0 * (resp_mag / peak_mag).log10()
}

// ---------------------------------------------------------------------------
// DFT routines (no external FFT crate)
// ---------------------------------------------------------------------------

/// Forward DFT: time-domain to frequency-domain.
fn dft_forward(samples: &[(f64, f64)], fft_size: usize) -> Vec<(f64, f64)> {
    let n = fft_size;
    let mut result = vec![(0.0, 0.0); n];

    // Zero-pad if needed
    let mut padded = vec![(0.0, 0.0); n];
    for (i, &s) in samples.iter().enumerate() {
        if i < n {
            padded[i] = s;
        }
    }

    // Use Cooley-Tukey if power of 2, otherwise naive DFT
    if n.is_power_of_two() && n > 1 {
        fft_radix2(&padded, &mut result, false);
    } else {
        for k in 0..n {
            let mut sum = (0.0, 0.0);
            for (j, &s) in padded.iter().enumerate() {
                let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
                let twiddle = (angle.cos(), angle.sin());
                sum = cx_add(sum, cx_mul(s, twiddle));
            }
            result[k] = sum;
        }
    }

    result
}

/// Inverse DFT: frequency-domain to time-domain.
fn dft_inverse(freq: &[(f64, f64)], fft_size: usize) -> Vec<(f64, f64)> {
    let n = fft_size;
    let mut result = vec![(0.0, 0.0); n];

    if n.is_power_of_two() && n > 1 {
        fft_radix2(freq, &mut result, true);
        // Scale by 1/N
        for r in &mut result {
            *r = cx_scale(*r, 1.0 / n as f64);
        }
    } else {
        for k in 0..n {
            let mut sum = (0.0, 0.0);
            for (j, &s) in freq.iter().enumerate() {
                let angle = 2.0 * PI * k as f64 * j as f64 / n as f64;
                let twiddle = (angle.cos(), angle.sin());
                sum = cx_add(sum, cx_mul(s, twiddle));
            }
            result[k] = cx_scale(sum, 1.0 / n as f64);
        }
    }

    result
}

/// In-place Cooley-Tukey radix-2 FFT.
fn fft_radix2(input: &[(f64, f64)], output: &mut [(f64, f64)], inverse: bool) {
    let n = input.len();
    assert!(n.is_power_of_two());

    // Bit-reversal permutation
    let mut bits = 0u32;
    let mut temp = n;
    while temp > 1 {
        bits += 1;
        temp >>= 1;
    }

    for i in 0..n {
        let rev = bit_reverse(i as u32, bits) as usize;
        output[rev] = input[i];
    }

    // Butterfly stages
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut size = 2;
    while size <= n {
        let half = size / 2;
        let angle_step = sign * 2.0 * PI / size as f64;

        let mut k = 0;
        while k < n {
            for j in 0..half {
                let angle = angle_step * j as f64;
                let twiddle = (angle.cos(), angle.sin());
                let u = output[k + j];
                let t = cx_mul(twiddle, output[k + j + half]);
                output[k + j] = cx_add(u, t);
                output[k + j + half] = cx_sub(u, t);
            }
            k += size;
        }
        size *= 2;
    }
}

/// Reverses `bits` bits of value `x`.
fn bit_reverse(x: u32, bits: u32) -> u32 {
    let mut result = 0u32;
    let mut val = x;
    for _ in 0..bits {
        result = (result << 1) | (val & 1);
        val >>= 1;
    }
    result
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn cx_approx_eq(a: (f64, f64), b: (f64, f64), tol: f64) -> bool {
        approx_eq(a.0, b.0, tol) && approx_eq(a.1, b.1, tol)
    }

    /// Generate snapshots with a desired signal and optional interferer.
    fn generate_snapshots(
        num_elements: usize,
        num_snapshots: usize,
        desired_doa: f64,
        desired_power: f64,
        interferer_doas: &[f64],
        interferer_powers: &[f64],
        noise_power: f64,
    ) -> Vec<Vec<(f64, f64)>> {
        let element_spacing = 0.5;
        let wavelength = 1.0;
        let sv_desired = steering_vector(desired_doa, num_elements, element_spacing, wavelength);

        let sv_interfs: Vec<Vec<(f64, f64)>> = interferer_doas
            .iter()
            .map(|&doa| steering_vector(doa, num_elements, element_spacing, wavelength))
            .collect();

        // Simple PRNG (LCG) for reproducibility
        let mut seed: u64 = 12345;
        let mut next_gaussian = || -> f64 {
            // Box-Muller using LCG
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u1 = (seed >> 11) as f64 / (1u64 << 53) as f64;
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u2 = (seed >> 11) as f64 / (1u64 << 53) as f64;
            let u1 = u1.max(1e-15);
            (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
        };

        (0..num_snapshots)
            .map(|_| {
                // Desired signal
                let sig_re = next_gaussian() * (desired_power / 2.0).sqrt();
                let sig_im = next_gaussian() * (desired_power / 2.0).sqrt();
                let sig = (sig_re, sig_im);

                let mut snap: Vec<(f64, f64)> = sv_desired
                    .iter()
                    .map(|&sv| cx_mul(sig, sv))
                    .collect();

                // Add interferers
                for (idx, sv_int) in sv_interfs.iter().enumerate() {
                    let pwr = if idx < interferer_powers.len() {
                        interferer_powers[idx]
                    } else {
                        1.0
                    };
                    let int_re = next_gaussian() * (pwr / 2.0).sqrt();
                    let int_im = next_gaussian() * (pwr / 2.0).sqrt();
                    let int_sig = (int_re, int_im);
                    for (i, &sv) in sv_int.iter().enumerate() {
                        snap[i] = cx_add(snap[i], cx_mul(int_sig, sv));
                    }
                }

                // Add noise
                for s in &mut snap {
                    let nr = next_gaussian() * (noise_power / 2.0).sqrt();
                    let ni = next_gaussian() * (noise_power / 2.0).sqrt();
                    *s = cx_add(*s, (nr, ni));
                }

                snap
            })
            .collect()
    }

    #[test]
    fn test_steering_vector_broadside() {
        // At broadside (0 rad), all elements should have equal phase
        let sv = steering_vector(0.0, 4, 0.5, 1.0);
        assert_eq!(sv.len(), 4);
        for &(re, im) in &sv {
            assert!(approx_eq(re, 1.0, EPS));
            assert!(approx_eq(im, 0.0, EPS));
        }
    }

    #[test]
    fn test_steering_vector_unit_magnitude() {
        // Each element of the steering vector should have unit magnitude
        let sv = steering_vector(0.3, 8, 0.5, 1.0);
        for &s in &sv {
            let mag = cx_mag_sq(s).sqrt();
            assert!(approx_eq(mag, 1.0, EPS));
        }
    }

    #[test]
    fn test_steering_vector_symmetry() {
        // Steering vectors for +theta and -theta should be conjugates
        let sv_pos = steering_vector(0.5, 4, 0.5, 1.0);
        let sv_neg = steering_vector(-0.5, 4, 0.5, 1.0);
        for i in 0..4 {
            assert!(cx_approx_eq(sv_pos[i], cx_conj(sv_neg[i]), EPS));
        }
    }

    #[test]
    fn test_spatial_covariance_identity() {
        // For white noise (identity covariance), off-diagonals should be near zero
        let n = 4;
        let k = 1000;

        // Generate uncorrelated snapshots
        let mut seed: u64 = 42;
        let mut next_val = || -> f64 {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u1 = ((seed >> 11) as f64 / (1u64 << 53) as f64).max(1e-15);
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u2 = (seed >> 11) as f64 / (1u64 << 53) as f64;
            (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
        };

        let snapshots: Vec<Vec<(f64, f64)>> = (0..k)
            .map(|_| (0..n).map(|_| (next_val(), next_val())).collect())
            .collect();

        let cov = spatial_covariance(&snapshots);
        assert_eq!(cov.len(), n);

        // Diagonal should be approximately equal (all same power)
        let diag_avg: f64 = (0..n).map(|i| cov[i][i].0).sum::<f64>() / n as f64;
        for i in 0..n {
            assert!(
                approx_eq(cov[i][i].0, diag_avg, 0.3),
                "Diagonal elements should be similar"
            );
        }

        // Off-diagonal imaginary should be small relative to diagonal
        for i in 0..n {
            for j in 0..n {
                if i != j {
                    let off_mag = cx_mag_sq(cov[i][j]).sqrt();
                    assert!(
                        off_mag < diag_avg * 0.3,
                        "Off-diagonal [{i}][{j}] = {off_mag:.4} too large relative to diag {diag_avg:.4}"
                    );
                }
            }
        }
    }

    #[test]
    fn test_spatial_covariance_hermitian() {
        // Covariance matrix must be Hermitian: R[i][j] = conj(R[j][i])
        let snapshots = generate_snapshots(4, 50, 0.0, 1.0, &[0.5], &[10.0], 0.1);
        let cov = spatial_covariance(&snapshots);

        for i in 0..cov.len() {
            for j in 0..cov.len() {
                assert!(
                    cx_approx_eq(cov[i][j], cx_conj(cov[j][i]), 1e-10),
                    "Covariance not Hermitian at [{i}][{j}]"
                );
            }
        }
    }

    #[test]
    fn test_spatial_covariance_empty() {
        let cov = spatial_covariance(&[]);
        assert!(cov.is_empty());
    }

    #[test]
    fn test_smi_beamform_basic() {
        // SMI should produce valid weights
        let snapshots = generate_snapshots(4, 100, 0.0, 1.0, &[], &[], 0.1);
        let sv = steering_vector(0.0, 4, 0.5, 1.0);
        let weights = smi_beamform(&snapshots, &sv);

        assert_eq!(weights.len(), 4);
        // All weights should be finite
        for &(re, im) in &weights {
            assert!(re.is_finite());
            assert!(im.is_finite());
        }
    }

    #[test]
    fn test_smi_beamform_null_placement() {
        // With a strong interferer at 0.5 rad, the beamformer should place a null there
        let snapshots = generate_snapshots(8, 200, 0.0, 1.0, &[0.5], &[100.0], 0.01);
        let sv = steering_vector(0.0, 8, 0.5, 1.0);
        let weights = smi_beamform(&snapshots, &sv);

        // Response in desired direction should be stronger than in interferer direction
        let sv_desired = steering_vector(0.0, 8, 0.5, 1.0);
        let sv_interf = steering_vector(0.5, 8, 0.5, 1.0);

        let mut resp_desired = (0.0, 0.0);
        let mut resp_interf = (0.0, 0.0);
        for i in 0..8 {
            resp_desired = cx_add(resp_desired, cx_mul(cx_conj(weights[i]), sv_desired[i]));
            resp_interf = cx_add(resp_interf, cx_mul(cx_conj(weights[i]), sv_interf[i]));
        }

        let desired_mag = cx_mag_sq(resp_desired).sqrt();
        let interf_mag = cx_mag_sq(resp_interf).sqrt();

        // Interferer response should be significantly attenuated
        assert!(
            interf_mag < desired_mag,
            "Interferer response ({interf_mag:.4}) should be less than desired ({desired_mag:.4})"
        );
    }

    #[test]
    fn test_smi_beamform_empty() {
        let result = smi_beamform(&[], &[(1.0, 0.0)]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_power_inversion_null() {
        // Power inversion should return valid weights
        let snapshots = generate_snapshots(4, 100, 0.0, 1.0, &[0.7], &[50.0], 0.1);
        let weights = power_inversion_null(&snapshots, 0.0);

        assert_eq!(weights.len(), 4);
        for &(re, im) in &weights {
            assert!(re.is_finite());
            assert!(im.is_finite());
        }
    }

    #[test]
    fn test_power_inversion_empty() {
        let result = power_inversion_null(&[], 0.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_spectral_excise_passband_preservation() {
        // Generate a tone not at the notch frequency - it should survive
        let n = 256;
        let sample_rate = 1000.0;
        let tone_freq = 100.0; // Hz
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * tone_freq * t;
                (phase.cos(), phase.sin())
            })
            .collect();

        // Excise at 400 Hz (far from our 100 Hz tone)
        let result = spectral_excise(&samples, &[400.0], 20.0, sample_rate);

        assert_eq!(result.len(), n);

        // The tone energy should be mostly preserved
        let input_power: f64 = samples.iter().map(|s| cx_mag_sq(*s)).sum::<f64>() / n as f64;
        let output_power: f64 = result.iter().map(|s| cx_mag_sq(*s)).sum::<f64>() / n as f64;

        assert!(
            output_power > input_power * 0.7,
            "Passband power should be mostly preserved: input={input_power:.4}, output={output_power:.4}"
        );
    }

    #[test]
    fn test_spectral_excise_notch_suppression() {
        // Generate a tone at the notch frequency - it should be suppressed
        let n = 256;
        let sample_rate = 1000.0;
        let tone_freq = 200.0;
        let samples: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * tone_freq * t;
                (phase.cos(), phase.sin())
            })
            .collect();

        let result = spectral_excise(&samples, &[200.0], 30.0, sample_rate);

        let input_power: f64 = samples.iter().map(|s| cx_mag_sq(*s)).sum::<f64>() / n as f64;
        let output_power: f64 = result.iter().map(|s| cx_mag_sq(*s)).sum::<f64>() / n as f64;

        assert!(
            output_power < input_power * 0.3,
            "Notched tone should be significantly suppressed: input={input_power:.4}, output={output_power:.4}"
        );
    }

    #[test]
    fn test_spectral_excise_empty() {
        let result = spectral_excise(&[], &[100.0], 10.0, 1000.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_temporal_blank_below_threshold() {
        // All samples below threshold should pass through unchanged
        let samples = vec![(0.1, 0.1), (0.2, -0.1), (-0.1, 0.3)];
        let result = temporal_blank(&samples, 1.0);
        assert_eq!(result.len(), 3);
        for i in 0..3 {
            assert!(cx_approx_eq(result[i], samples[i], EPS));
        }
    }

    #[test]
    fn test_temporal_blank_above_threshold() {
        // Samples above threshold should be blanked to zero
        let samples = vec![(0.1, 0.1), (10.0, 10.0), (0.2, -0.1)];
        let result = temporal_blank(&samples, 1.0);
        assert!(cx_approx_eq(result[0], (0.1, 0.1), EPS));
        assert!(cx_approx_eq(result[1], (0.0, 0.0), EPS)); // blanked
        assert!(cx_approx_eq(result[2], (0.2, -0.1), EPS));
    }

    #[test]
    fn test_temporal_blank_empty() {
        let result = temporal_blank(&[], 1.0);
        assert!(result.is_empty());
    }

    #[test]
    fn test_estimate_inr_no_interference() {
        // With no interferers, INR should be near zero
        let snapshots = generate_snapshots(4, 200, 0.0, 1.0, &[], &[], 1.0);
        let sv = steering_vector(0.0, 4, 0.5, 1.0);
        let inr = estimate_inr(&snapshots, &sv);

        assert!(
            inr < 1.0,
            "INR with no interference should be small, got {inr:.4}"
        );
    }

    #[test]
    fn test_estimate_inr_with_interference() {
        // With a strong interferer, INR should be larger
        let snapshots =
            generate_snapshots(4, 200, 0.0, 1.0, &[0.5], &[100.0], 0.01);
        let sv = steering_vector(0.0, 4, 0.5, 1.0);
        let inr = estimate_inr(&snapshots, &sv);

        assert!(
            inr > 0.0,
            "INR with strong interference should be positive, got {inr:.4}"
        );
    }

    #[test]
    fn test_estimate_inr_empty() {
        let inr = estimate_inr(&[], &[(1.0, 0.0)]);
        assert!(approx_eq(inr, 0.0, EPS));
    }

    #[test]
    fn test_compute_null_depth_uniform_weights() {
        // Uniform weights should have a known pattern
        let n = 8;
        let weights: Vec<(f64, f64)> = vec![(1.0 / n as f64, 0.0); n];
        let depth = compute_null_depth(&weights, 0.0, 1.0, 0.5);
        // At broadside with uniform weights, depth relative to broadside peak = 0 dB
        assert!(
            approx_eq(depth, 0.0, 0.1),
            "Null depth at broadside should be ~0 dB, got {depth:.4}"
        );
    }

    #[test]
    fn test_compute_null_depth_at_null() {
        // For a ULA with uniform weights, first null is at arcsin(lambda/(N*d))
        let n = 8;
        let d = 0.5;
        let lambda = 1.0;
        let weights: Vec<(f64, f64)> = vec![(1.0 / n as f64, 0.0); n];

        // First null of uniform array at theta = arcsin(lambda / (N*d))
        let null_angle = (lambda / (n as f64 * d)).asin();
        let depth = compute_null_depth(&weights, null_angle, lambda, d);

        // Should be significantly below 0 dB (deep null)
        assert!(
            depth < -10.0,
            "Null depth at first null should be deep, got {depth:.2} dB"
        );
    }

    #[test]
    fn test_adaptive_nuller_new() {
        let nuller = AdaptiveNuller::new(8, 1e6, 0.03);
        assert_eq!(nuller.num_elements, 8);
        assert!(approx_eq(nuller.sample_rate_hz, 1e6, EPS));
        assert!(approx_eq(nuller.wavelength_m, 0.03, EPS));
    }

    #[test]
    fn test_adaptive_nuller_process() {
        let nuller = AdaptiveNuller::new(4, 1000.0, 1.0);

        let snapshots = generate_snapshots(4, 100, 0.0, 1.0, &[0.6], &[50.0], 0.1);

        let result = nuller.process(&snapshots, 0.0, &[0.6], &[], 0.0);

        assert_eq!(result.output_samples.len(), snapshots.len());
        assert_eq!(result.null_depths_db.len(), 1);
        assert!(result.sinr_improvement_db >= 0.0);
        assert!(result.null_depths_db[0] < 0.0, "Null depth should be negative (suppression)");
    }

    #[test]
    fn test_adaptive_nuller_with_spectral_excision() {
        let nuller = AdaptiveNuller::new(4, 1000.0, 1.0);

        let snapshots = generate_snapshots(4, 100, 0.0, 1.0, &[], &[], 0.1);

        let result = nuller.process(&snapshots, 0.0, &[], &[200.0], 30.0);

        assert_eq!(result.output_samples.len(), snapshots.len());
        assert!(result.null_depths_db.is_empty());
    }

    #[test]
    fn test_dft_roundtrip() {
        // Forward then inverse DFT should recover original signal
        let samples = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        let n = 4;
        let freq = dft_forward(&samples, n);
        let recovered = dft_inverse(&freq, n);

        for i in 0..n {
            assert!(
                cx_approx_eq(recovered[i], samples[i], 1e-10),
                "DFT roundtrip failed at index {i}: {:?} vs {:?}",
                recovered[i],
                samples[i]
            );
        }
    }

    #[test]
    fn test_full_pipeline_integration() {
        // End-to-end: generate snapshots with interference, null it, check improvement
        let n_elements = 8;
        let desired_doa = 0.0;
        let interf_doa = 0.7;
        let interf_power = 100.0;
        let noise_power = 0.01;

        let snapshots = generate_snapshots(
            n_elements,
            300,
            desired_doa,
            1.0,
            &[interf_doa],
            &[interf_power],
            noise_power,
        );

        // Beamform
        let sv = steering_vector(desired_doa, n_elements, 0.5, 1.0);
        let weights = smi_beamform(&snapshots, &sv);

        // Verify null at interferer
        let null_depth = compute_null_depth(&weights, interf_doa, 1.0, 0.5);
        assert!(
            null_depth < -5.0,
            "Should have meaningful null at interferer: got {null_depth:.2} dB"
        );

        // Verify signal passthrough at desired direction
        let desired_depth = compute_null_depth(&weights, desired_doa, 1.0, 0.5);
        assert!(
            desired_depth > null_depth,
            "Desired direction ({desired_depth:.2} dB) should have higher response than interferer ({null_depth:.2} dB)"
        );

        // Apply temporal blanking to the beamformed output
        let output: Vec<(f64, f64)> = snapshots
            .iter()
            .map(|snap| {
                let mut out = (0.0, 0.0);
                for (i, &(wr, wi)) in weights.iter().enumerate() {
                    if i < snap.len() {
                        let (xr, xi) = snap[i];
                        out = cx_add(out, (wr * xr + wi * xi, wr * xi - wi * xr));
                    }
                }
                out
            })
            .collect();

        let blanked = temporal_blank(&output, 1000.0);
        assert_eq!(blanked.len(), output.len());
    }
}
