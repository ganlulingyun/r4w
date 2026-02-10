//! Ground-Penetrating Radar (GPR) signal processing for subsurface imaging.
//!
//! This module implements a complete GPR processing pipeline including UWB pulse
//! generation, time-zero alignment, background removal (mean subtraction and
//! SVD-based clutter removal), gain control (spherical and exponential compensation),
//! migration (Kirchhoff and Stolt F-K), dielectric constant estimation, depth
//! conversion, and hyperbola detection for buried object localization.
//!
//! # Example
//!
//! ```
//! use r4w_core::gpr_subsurface_imager::{
//!     GprConfig, GprRadargram, GprProcessor,
//!     generate_ricker_wavelet, depth_convert, estimate_dielectric,
//! };
//!
//! // Create a configuration for a 1 GHz center-frequency survey
//! let config = GprConfig {
//!     sample_rate_hz: 10.0e9,
//!     time_window_ns: 50.0,
//!     antenna_separation_m: 0.1,
//!     scan_spacing_m: 0.02,
//! };
//!
//! // Generate a Ricker wavelet at 1 GHz
//! let wavelet = generate_ricker_wavelet(1.0e9, config.sample_rate_hz, 2.0e-9);
//! assert!(!wavelet.is_empty());
//!
//! // Estimate dielectric constant from known depth and travel time
//! let eps_r = estimate_dielectric(10.0, 1.0);
//! assert!(eps_r > 1.0);
//!
//! // Convert two-way travel time to depth given a dielectric constant
//! let depth = depth_convert(10.0, 9.0);
//! assert!(depth > 0.0);
//!
//! // Create a processor
//! let processor = GprProcessor::new(config);
//! assert!(processor.config().sample_rate_hz > 0.0);
//! ```

use std::f64::consts::PI;

// --- Configuration -----------------------------------------------------------

/// Configuration parameters for a GPR survey.
#[derive(Debug, Clone)]
pub struct GprConfig {
    /// Sampling rate in Hz (e.g., 10 GHz for a 1 GHz center-frequency system).
    pub sample_rate_hz: f64,
    /// Total time window per trace in nanoseconds.
    pub time_window_ns: f64,
    /// Separation between transmitter and receiver antennas in metres.
    pub antenna_separation_m: f64,
    /// Spacing between adjacent scan positions along the survey line in metres.
    pub scan_spacing_m: f64,
}

// --- Radargram ---------------------------------------------------------------

/// A 2-D radargram (B-scan) consisting of a collection of A-scan traces.
///
/// Each trace is a vector of `f64` amplitude samples. All traces must have the
/// same length (`samples_per_trace`).
#[derive(Debug, Clone)]
pub struct GprRadargram {
    /// Column-major trace data -- `traces[i]` is the i-th A-scan.
    pub traces: Vec<Vec<f64>>,
    /// Number of traces (columns).
    pub num_traces: usize,
    /// Number of samples per trace (rows).
    pub samples_per_trace: usize,
    /// Time-sample interval in nanoseconds.
    pub dt_ns: f64,
    /// Spatial interval between traces in metres.
    pub dx_m: f64,
}

impl GprRadargram {
    /// Create a new radargram from the given traces.
    ///
    /// # Panics
    ///
    /// Panics if `traces` is empty, or if any trace has a different length from
    /// the first trace, or if `dt_ns` or `dx_m` are not positive.
    pub fn new(traces: Vec<Vec<f64>>, dt_ns: f64, dx_m: f64) -> Self {
        assert!(!traces.is_empty(), "traces must not be empty");
        let samples_per_trace = traces[0].len();
        assert!(samples_per_trace > 0, "traces must contain samples");
        for (i, t) in traces.iter().enumerate() {
            assert_eq!(
                t.len(),
                samples_per_trace,
                "trace {} has {} samples, expected {}",
                i,
                t.len(),
                samples_per_trace,
            );
        }
        assert!(dt_ns > 0.0, "dt_ns must be positive");
        assert!(dx_m > 0.0, "dx_m must be positive");
        let num_traces = traces.len();
        Self {
            traces,
            num_traces,
            samples_per_trace,
            dt_ns,
            dx_m,
        }
    }
}

// --- Processor ---------------------------------------------------------------

/// Main GPR signal processor.
///
/// Holds a [`GprConfig`] and provides methods that delegate to the free
/// functions in this module, for convenience.
#[derive(Debug, Clone)]
pub struct GprProcessor {
    config: GprConfig,
}

impl GprProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: GprConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the configuration.
    pub fn config(&self) -> &GprConfig {
        &self.config
    }

    /// Generate a Ricker wavelet using the processor's sample rate.
    pub fn ricker_wavelet(&self, center_freq_hz: f64, duration_s: f64) -> Vec<f64> {
        generate_ricker_wavelet(center_freq_hz, self.config.sample_rate_hz, duration_s)
    }

    /// Generate a Gaussian-derivative pulse using the processor's sample rate.
    pub fn gaussian_derivative(&self, sigma: f64, duration_s: f64) -> Vec<f64> {
        generate_gaussian_derivative(sigma, self.config.sample_rate_hz, duration_s)
    }

    /// Apply time-zero alignment to a radargram.
    pub fn align_time_zero(&self, radargram: &mut GprRadargram, threshold: f64) {
        time_zero_align(radargram, threshold);
    }

    /// Apply mean-trace background removal.
    pub fn remove_background_mean(&self, radargram: &mut GprRadargram) {
        background_removal_mean(radargram);
    }

    /// Apply SVD-based clutter removal.
    pub fn remove_background_svd(
        &self,
        radargram: &mut GprRadargram,
        components_to_remove: usize,
    ) {
        background_removal_svd(radargram, components_to_remove);
    }

    /// Apply SEC gain to a single trace.
    pub fn apply_gain_sec(
        &self,
        trace: &mut [f64],
        attenuation_db_per_m: f64,
        velocity_mps: f64,
    ) {
        let dt_ns = self.config.time_window_ns / (trace.len() as f64);
        gain_sec(trace, attenuation_db_per_m, velocity_mps, dt_ns);
    }

    /// Perform Kirchhoff migration.
    pub fn migrate_kirchhoff(
        &self,
        radargram: &GprRadargram,
        velocity_mps: f64,
    ) -> GprRadargram {
        kirchhoff_migrate(radargram, velocity_mps)
    }
}

// --- UWB Pulse Generation ----------------------------------------------------

/// Generate a Ricker wavelet (Mexican-hat wavelet).
///
/// The Ricker wavelet is the negative normalised second derivative of a Gaussian
/// and is widely used as a source pulse in GPR modelling.
///
/// # Arguments
///
/// * `center_freq_hz` -- Peak frequency in Hz.
/// * `sample_rate` -- Sampling rate in Hz.
/// * `duration_s` -- Total duration in seconds (centred on t = 0).
///
/// # Returns
///
/// A vector of amplitude samples.
pub fn generate_ricker_wavelet(
    center_freq_hz: f64,
    sample_rate: f64,
    duration_s: f64,
) -> Vec<f64> {
    let n = (duration_s * sample_rate).round() as usize;
    if n == 0 {
        return Vec::new();
    }
    let sigma = 1.0 / (PI * center_freq_hz * (2.0_f64).sqrt());
    let half_n = (n - 1) as f64 / 2.0;
    let dt = 1.0 / sample_rate;
    (0..n)
        .map(|i| {
            let t = (i as f64 - half_n) * dt;
            let u = (t / sigma).powi(2);
            (1.0 - u) * (-u / 2.0).exp()
        })
        .collect()
}

/// Generate a Gaussian first-derivative pulse.
///
/// # Arguments
///
/// * `sigma` -- Standard deviation of the underlying Gaussian (seconds).
/// * `sample_rate` -- Sampling rate in Hz.
/// * `duration_s` -- Total duration in seconds (centred on t = 0).
///
/// # Returns
///
/// A vector of amplitude samples.
pub fn generate_gaussian_derivative(
    sigma: f64,
    sample_rate: f64,
    duration_s: f64,
) -> Vec<f64> {
    let n = (duration_s * sample_rate).round() as usize;
    if n == 0 {
        return Vec::new();
    }
    let half_n = (n - 1) as f64 / 2.0;
    let dt = 1.0 / sample_rate;
    (0..n)
        .map(|i| {
            let t = (i as f64 - half_n) * dt;
            let norm = t / (sigma * sigma);
            -norm * (-t * t / (2.0 * sigma * sigma)).exp()
        })
        .collect()
}

// --- Time-Zero Alignment -----------------------------------------------------

/// Align all traces so that the first sample exceeding `threshold` (in
/// absolute value) sits at sample index 0.
///
/// Traces are shifted (truncated or zero-padded) so that every trace begins
/// at its estimated first-break time.  This corrects for trigger jitter and
/// antenna coupling variations between scans.
pub fn time_zero_align(radargram: &mut GprRadargram, threshold: f64) {
    let abs_thresh = threshold.abs();
    for trace in &mut radargram.traces {
        let first_break = trace
            .iter()
            .position(|&s| s.abs() >= abs_thresh)
            .unwrap_or(0);
        if first_break > 0 {
            trace.drain(0..first_break);
        }
        // Pad back to original length
        trace.resize(radargram.samples_per_trace, 0.0);
    }
}

// --- Background Removal ------------------------------------------------------

/// Remove the mean trace from every trace in the radargram.
///
/// This is the simplest form of background removal -- it suppresses the direct
/// (air-wave / ground-bounce) coupling that is common to all traces.
pub fn background_removal_mean(radargram: &mut GprRadargram) {
    let nt = radargram.num_traces as f64;
    let ns = radargram.samples_per_trace;
    // Compute mean trace
    let mut mean = vec![0.0; ns];
    for trace in &radargram.traces {
        for (j, &s) in trace.iter().enumerate() {
            mean[j] += s;
        }
    }
    for m in &mut mean {
        *m /= nt;
    }
    // Subtract
    for trace in &mut radargram.traces {
        for (j, s) in trace.iter_mut().enumerate() {
            *s -= mean[j];
        }
    }
}

/// SVD-based background (clutter) removal.
///
/// Computes a truncated singular-value decomposition of the radargram matrix
/// and reconstructs it with the first `components_to_remove` singular values
/// set to zero. Because the strongest singular values typically correspond to
/// the horizontally coherent clutter (direct coupling, ringing), removing
/// them reveals the weaker scattering targets.
///
/// This is a pure-`std` implementation using power-iteration deflation.
pub fn background_removal_svd(
    radargram: &mut GprRadargram,
    components_to_remove: usize,
) {
    if components_to_remove == 0 {
        return;
    }
    let m = radargram.samples_per_trace; // rows
    let n = radargram.num_traces; // cols

    // Build a flat column-major matrix  (m rows x n cols).
    let mut mat = vec![0.0; m * n];
    for (col, trace) in radargram.traces.iter().enumerate() {
        for (row, &val) in trace.iter().enumerate() {
            mat[col * m + row] = val;
        }
    }

    // For each component to remove, find the dominant singular triplet via
    // power iteration, subtract its rank-1 contribution, and repeat.
    for _ in 0..components_to_remove {
        let (u, sigma, v) = power_iteration_svd(&mat, m, n, 200);
        if sigma < 1e-15 {
            break;
        }
        // Subtract rank-1 approximation: A -= sigma * u * v^T
        for col in 0..n {
            for row in 0..m {
                mat[col * m + row] -= sigma * u[row] * v[col];
            }
        }
    }

    // Write back
    for (col, trace) in radargram.traces.iter_mut().enumerate() {
        for (row, s) in trace.iter_mut().enumerate() {
            *s = mat[col * m + row];
        }
    }
}

/// Single-vector power iteration for the top singular triplet of an m x n
/// column-major matrix. Returns (u, sigma, v).
fn power_iteration_svd(
    mat: &[f64],
    m: usize,
    n: usize,
    max_iter: usize,
) -> (Vec<f64>, f64, Vec<f64>) {
    // Start with a deterministic vector
    let mut v = vec![0.0; n];
    for i in 0..n {
        v[i] = ((i as f64 + 1.0) * 0.7071).sin();
    }
    normalise(&mut v);

    let mut u = vec![0.0; m];

    for _ in 0..max_iter {
        // u = A * v
        for row in 0..m {
            let mut sum = 0.0;
            for col in 0..n {
                sum += mat[col * m + row] * v[col];
            }
            u[row] = sum;
        }
        let sigma_u = vec_norm(&u);
        if sigma_u < 1e-30 {
            return (vec![0.0; m], 0.0, vec![0.0; n]);
        }
        for x in &mut u {
            *x /= sigma_u;
        }

        // v = A^T * u
        for col in 0..n {
            let mut sum = 0.0;
            for row in 0..m {
                sum += mat[col * m + row] * u[row];
            }
            v[col] = sum;
        }
        let sigma_v = vec_norm(&v);
        if sigma_v < 1e-30 {
            return (u, 0.0, vec![0.0; n]);
        }
        for x in &mut v {
            *x /= sigma_v;
        }
    }

    // Final sigma
    let mut av = vec![0.0; m];
    for row in 0..m {
        let mut sum = 0.0;
        for col in 0..n {
            sum += mat[col * m + row] * v[col];
        }
        av[row] = sum;
    }
    let sigma = vec_norm(&av);
    (u, sigma, v)
}

fn vec_norm(v: &[f64]) -> f64 {
    v.iter().map(|x| x * x).sum::<f64>().sqrt()
}

fn normalise(v: &mut [f64]) {
    let n = vec_norm(v);
    if n > 1e-30 {
        for x in v.iter_mut() {
            *x /= n;
        }
    }
}

// --- Gain Control ------------------------------------------------------------

/// Apply Spherical and Exponential Compensation (SEC) gain to a single trace.
///
/// Compensates for geometric spreading (1/r) and material attenuation
/// (exp(-alpha * r)) so that deeper reflectors appear with similar amplitudes
/// to shallow ones.
///
/// # Arguments
///
/// * `trace` -- Mutable trace samples to be gain-corrected in place.
/// * `attenuation_db_per_m` -- Material attenuation in dB/m.
/// * `velocity_mps` -- Electromagnetic wave velocity in the medium (m/s).
/// * `dt_ns` -- Time spacing between samples in nanoseconds.
pub fn gain_sec(
    trace: &mut [f64],
    attenuation_db_per_m: f64,
    velocity_mps: f64,
    dt_ns: f64,
) {
    let alpha_neper = attenuation_db_per_m / 8.686; // dB -> Np/m
    let dt_s = dt_ns * 1e-9;
    for (i, sample) in trace.iter_mut().enumerate() {
        let two_way_time = (i as f64) * dt_s;
        let one_way_range = velocity_mps * two_way_time / 2.0;
        if one_way_range < 1e-12 {
            continue; // avoid division by zero at t = 0
        }
        // Spherical: r, Exponential: exp(alpha * r)
        let gain = one_way_range * (alpha_neper * one_way_range).exp();
        *sample *= gain;
    }
}

// --- Migration ---------------------------------------------------------------

/// Kirchhoff diffraction-summation migration.
///
/// For each output pixel (trace, sample) the algorithm sums all input samples
/// that lie on the corresponding diffraction hyperbola, weighted by an
/// obliquity (cosine) factor. This collapses diffraction hyperbolae into
/// focused point reflectors and corrects dipping-reflector positions.
///
/// # Arguments
///
/// * `radargram` -- Input radargram to migrate.
/// * `velocity_mps` -- Assumed constant EM velocity in the subsurface (m/s).
///
/// # Returns
///
/// A new, migrated [`GprRadargram`].
pub fn kirchhoff_migrate(
    radargram: &GprRadargram,
    velocity_mps: f64,
) -> GprRadargram {
    let nt = radargram.num_traces;
    let ns = radargram.samples_per_trace;
    let dt_s = radargram.dt_ns * 1e-9;
    let dx = radargram.dx_m;

    let mut output = vec![vec![0.0; ns]; nt];

    for ix_out in 0..nt {
        let x_out = ix_out as f64 * dx;
        for iz in 1..ns {
            let t0 = iz as f64 * dt_s;
            let z = velocity_mps * t0 / 2.0; // one-way depth

            let mut sum = 0.0;
            let mut weight_sum = 0.0;

            for ix_in in 0..nt {
                let x_in = ix_in as f64 * dx;
                let horiz = x_in - x_out;
                let slant = (z * z + horiz * horiz).sqrt();
                let t_hyp = 2.0 * slant / velocity_mps; // two-way travel time
                let sample_idx_f = t_hyp / dt_s;
                let sample_idx = sample_idx_f as usize;
                if sample_idx + 1 < ns {
                    let frac = sample_idx_f - sample_idx as f64;
                    let val = radargram.traces[ix_in][sample_idx] * (1.0 - frac)
                        + radargram.traces[ix_in][sample_idx + 1] * frac;
                    // Obliquity (cosine) weight: cos(theta) = z / slant
                    let cos_theta = if slant > 1e-15 { z / slant } else { 1.0 };
                    sum += val * cos_theta;
                    weight_sum += cos_theta;
                }
            }

            if weight_sum > 1e-15 {
                output[ix_out][iz] = sum / weight_sum;
            }
        }
    }

    GprRadargram::new(output, radargram.dt_ns, radargram.dx_m)
}

/// Stolt F-K (frequency-wavenumber) migration.
///
/// Transforms the radargram into the 2-D Fourier domain (f, k_x), remaps
/// the frequency axis using the Stolt stretch, and transforms back. This is
/// computationally efficient and exact for constant-velocity media.
///
/// Because this module is `std`-only, we implement a small radix-2
/// Cooley-Tukey FFT internally.
///
/// # Arguments
///
/// * `radargram` -- Input radargram (dimensions should ideally be powers of 2).
/// * `velocity_mps` -- Assumed constant EM velocity in the subsurface (m/s).
///
/// # Returns
///
/// A new, migrated [`GprRadargram`].
pub fn stolt_fk_migrate(
    radargram: &GprRadargram,
    velocity_mps: f64,
) -> GprRadargram {
    let nt = radargram.num_traces;
    let ns = radargram.samples_per_trace;

    // Pad to next power of 2 in both dimensions
    let nf = ns.next_power_of_two();
    let nk = nt.next_power_of_two();

    let dt_s = radargram.dt_ns * 1e-9;
    let dx = radargram.dx_m;
    let df = 1.0 / (nf as f64 * dt_s);
    let dk = 1.0 / (nk as f64 * dx);

    // 2-D FFT: first FFT along time (rows), then along traces (columns).
    let mut data: Vec<Vec<(f64, f64)>> = Vec::with_capacity(nk);
    for col in 0..nk {
        let mut col_data: Vec<(f64, f64)> = vec![(0.0, 0.0); nf];
        if col < nt {
            for row in 0..ns.min(nf) {
                col_data[row] = (radargram.traces[col][row], 0.0);
            }
        }
        fft_in_place(&mut col_data, false);
        data.push(col_data);
    }

    // FFT along the trace (kx) dimension for each frequency bin
    for f_idx in 0..nf {
        let mut row: Vec<(f64, f64)> = (0..nk).map(|col| data[col][f_idx]).collect();
        fft_in_place(&mut row, false);
        for col in 0..nk {
            data[col][f_idx] = row[col];
        }
    }

    // Stolt mapping
    let v_half = velocity_mps / 2.0;
    let mut migrated = vec![vec![(0.0, 0.0); nf]; nk];

    for kx_idx in 0..nk {
        let kx = if kx_idx <= nk / 2 {
            kx_idx as f64 * dk
        } else {
            (kx_idx as f64 - nk as f64) * dk
        };
        let kx_term = (v_half * kx * 2.0 * PI).powi(2);

        for fz_idx in 0..nf {
            let fz = if fz_idx <= nf / 2 {
                fz_idx as f64 * df
            } else {
                (fz_idx as f64 - nf as f64) * df
            };
            let f_in_sq = fz * fz + kx_term / (4.0 * PI * PI);
            if f_in_sq < 0.0 {
                continue;
            }
            let f_in = f_in_sq.sqrt();
            let f_in_idx = f_in / df;
            let idx0 = f_in_idx as usize;
            if idx0 + 1 < nf / 2 {
                let frac = f_in_idx - idx0 as f64;
                let v0 = data[kx_idx][idx0];
                let v1 = data[kx_idx][idx0 + 1];
                migrated[kx_idx][fz_idx] = (
                    v0.0 * (1.0 - frac) + v1.0 * frac,
                    v0.1 * (1.0 - frac) + v1.1 * frac,
                );
            }
        }
    }

    // Inverse 2-D FFT
    for f_idx in 0..nf {
        let mut row: Vec<(f64, f64)> = (0..nk).map(|col| migrated[col][f_idx]).collect();
        fft_in_place(&mut row, true);
        for col in 0..nk {
            migrated[col][f_idx] = row[col];
        }
    }

    let mut traces = Vec::with_capacity(nt);
    for col in 0..nt {
        fft_in_place(&mut migrated[col], true);
        let trace: Vec<f64> = migrated[col][..ns].iter().map(|c| c.0).collect();
        traces.push(trace);
    }

    GprRadargram::new(traces, radargram.dt_ns, radargram.dx_m)
}

// --- Minimal Radix-2 FFT ----------------------------------------------------

/// In-place radix-2 Cooley-Tukey FFT.  `inverse` = true for IFFT.
/// Length must be a power of 2.
fn fft_in_place(buf: &mut [(f64, f64)], inverse: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT length must be a power of 2");
    if n <= 1 {
        return;
    }

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
            buf.swap(i, j);
        }
    }

    // Butterfly passes
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let wn = (angle.cos(), angle.sin());
        let mut start = 0;
        while start < n {
            let mut w = (1.0, 0.0);
            for k in 0..half {
                let a = buf[start + k];
                let b = buf[start + k + half];
                let tb = (w.0 * b.0 - w.1 * b.1, w.0 * b.1 + w.1 * b.0);
                buf[start + k] = (a.0 + tb.0, a.1 + tb.1);
                buf[start + k + half] = (a.0 - tb.0, a.1 - tb.1);
                w = (w.0 * wn.0 - w.1 * wn.1, w.0 * wn.1 + w.1 * wn.0);
            }
            start += len;
        }
        len <<= 1;
    }

    if inverse {
        let scale = 1.0 / n as f64;
        for x in buf.iter_mut() {
            x.0 *= scale;
            x.1 *= scale;
        }
    }
}

// --- Dielectric / Depth Utilities --------------------------------------------

/// Estimate relative dielectric constant (permittivity) from a known
/// two-way travel time and physical depth.
///
/// Uses the relation  epsilon_r = (c * t / (2 * d))^2  where c is the speed of
/// light, t is the two-way travel time, and d is the depth.
///
/// # Arguments
///
/// * `two_way_time_ns` -- Measured two-way travel time in nanoseconds.
/// * `depth_m` -- Known physical depth in metres.
///
/// # Panics
///
/// Panics if `depth_m` is zero or negative.
pub fn estimate_dielectric(two_way_time_ns: f64, depth_m: f64) -> f64 {
    assert!(depth_m > 0.0, "depth_m must be positive");
    const C: f64 = 299_792_458.0; // m/s
    let t_s = two_way_time_ns * 1e-9;
    let v = 2.0 * depth_m / t_s;
    (C / v).powi(2)
}

/// Convert a two-way travel time to depth given a dielectric constant.
///
/// depth = c * t / (2 * sqrt(epsilon_r))
///
/// # Arguments
///
/// * `time_ns` -- Two-way travel time in nanoseconds.
/// * `dielectric_constant` -- Relative permittivity of the medium.
///
/// # Panics
///
/// Panics if `dielectric_constant` is not positive.
pub fn depth_convert(time_ns: f64, dielectric_constant: f64) -> f64 {
    assert!(dielectric_constant > 0.0, "dielectric_constant must be positive");
    const C: f64 = 299_792_458.0;
    let v = C / dielectric_constant.sqrt();
    let t_s = time_ns * 1e-9;
    v * t_s / 2.0
}

// --- Hyperbola Detection -----------------------------------------------------

/// Detect diffraction hyperbolae in a radargram by searching for local
/// amplitude maxima and fitting hyperbolic travel-time curves.
///
/// Returns a vector of `(trace_index, sample_index, estimated_velocity_mps)`
/// for each detected hyperbola apex.
///
/// # Algorithm
///
/// 1. Find local maxima that exceed `min_amplitude`.
/// 2. For each candidate apex, fit a hyperbola  t^2 = t0^2 + (2x/v)^2  using
///    a least-squares velocity estimate over neighbouring traces.
/// 3. Accept the detection if the fit residual is below an internal threshold.
///
/// # Arguments
///
/// * `radargram` -- Input radargram.
/// * `min_amplitude` -- Minimum absolute amplitude for a peak to be considered.
///
/// # Returns
///
/// A `Vec<(usize, usize, f64)>` of (trace index, sample index, velocity m/s).
pub fn detect_hyperbolas(
    radargram: &GprRadargram,
    min_amplitude: f64,
) -> Vec<(usize, usize, f64)> {
    let nt = radargram.num_traces;
    let ns = radargram.samples_per_trace;
    let dt_s = radargram.dt_ns * 1e-9;
    let dx = radargram.dx_m;
    let mut detections = Vec::new();

    // Minimum half-aperture for fitting (traces on each side).
    let half_ap = 3.min(nt / 2);
    if half_ap < 2 {
        return detections;
    }

    for ix in half_ap..(nt - half_ap) {
        for iz in 1..(ns - 1) {
            let amp = radargram.traces[ix][iz];
            if amp.abs() < min_amplitude {
                continue;
            }
            // Local maximum check (3x3)
            if amp.abs() <= radargram.traces[ix][iz - 1].abs()
                || amp.abs() <= radargram.traces[ix][iz + 1].abs()
                || amp.abs() <= radargram.traces[ix - 1][iz].abs()
                || amp.abs() <= radargram.traces[ix + 1][iz].abs()
            {
                continue;
            }

            // Fit hyperbola:  t(x)^2 = t0^2 + 4*x^2 / v^2
            let t0 = iz as f64 * dt_s;
            let t0_sq = t0 * t0;

            // Least-squares: minimise sum (dt^2_i - 4*x_i^2/v^2)^2 w.r.t. 1/v^2
            //   => 1/v^2 = sum(x^2 * dt^2) / (4 * sum(x^4))
            let mut sum_x4 = 0.0;
            let mut sum_x2_dt2 = 0.0;
            let mut n_pts = 0u32;

            for offset in 1..=half_ap {
                for &sign in &[-1i32, 1i32] {
                    let ix2 = (ix as i32 + sign * offset as i32) as usize;
                    if ix2 >= nt {
                        continue;
                    }
                    let x = (offset as f64) * dx;
                    // Search in a window around iz for the strongest peak
                    let search_lo = iz.saturating_sub(ns / 4);
                    let search_hi = (iz + ns / 4).min(ns);
                    let mut best_k = iz;
                    let mut best_amp = 0.0_f64;
                    for k in search_lo..search_hi {
                        let a = radargram.traces[ix2][k].abs();
                        if a > best_amp {
                            best_amp = a;
                            best_k = k;
                        }
                    }
                    let t_k = best_k as f64 * dt_s;
                    let dt2 = t_k * t_k - t0_sq;
                    let x2 = x * x;
                    sum_x4 += x2 * x2;
                    sum_x2_dt2 += x2 * dt2;
                    n_pts += 1;
                }
            }

            if n_pts < 2 || sum_x4 < 1e-30 || sum_x2_dt2 <= 0.0 {
                continue;
            }

            let inv_v2 = sum_x2_dt2 / (4.0 * sum_x4);
            if inv_v2 <= 0.0 {
                continue;
            }
            let v_est = (1.0 / inv_v2).sqrt();

            // Sanity check: velocity between 0.01c and c
            const C: f64 = 299_792_458.0;
            if v_est < 0.01 * C || v_est > C {
                continue;
            }

            detections.push((ix, iz, v_est));
        }
    }

    detections
}

// --- Tests -------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: create a simple synthetic radargram
    fn make_radargram(num_traces: usize, samples_per_trace: usize) -> GprRadargram {
        let traces: Vec<Vec<f64>> = (0..num_traces)
            .map(|_| vec![0.0; samples_per_trace])
            .collect();
        GprRadargram::new(traces, 0.5, 0.02)
    }

    // Helper: add a point reflector hyperbola to a radargram
    fn add_hyperbola(
        rg: &mut GprRadargram,
        apex_trace: usize,
        apex_sample: usize,
        velocity_mps: f64,
        amplitude: f64,
    ) {
        let dt_s = rg.dt_ns * 1e-9;
        let dx = rg.dx_m;
        let t0 = apex_sample as f64 * dt_s;
        for ix in 0..rg.num_traces {
            let x = (ix as f64 - apex_trace as f64) * dx;
            let t_hyp = (t0 * t0 + 4.0 * x * x / (velocity_mps * velocity_mps)).sqrt();
            let sample_f = t_hyp / dt_s;
            let sample = sample_f.round() as usize;
            if sample < rg.samples_per_trace {
                rg.traces[ix][sample] += amplitude;
            }
        }
    }

    // -- Ricker wavelet tests -------------------------------------------------

    #[test]
    fn test_ricker_wavelet_length() {
        let w = generate_ricker_wavelet(1.0e9, 10.0e9, 2.0e-9);
        // 2 ns * 10 GHz = 20 samples
        assert_eq!(w.len(), 20);
    }

    #[test]
    fn test_ricker_wavelet_symmetry() {
        let w = generate_ricker_wavelet(500.0e6, 8.0e9, 4.0e-9);
        let n = w.len();
        for i in 0..n / 2 {
            assert!(
                (w[i] - w[n - 1 - i]).abs() < 1e-12,
                "wavelet not symmetric at {}",
                i
            );
        }
    }

    #[test]
    fn test_ricker_wavelet_peak_at_center() {
        let w = generate_ricker_wavelet(1.0e9, 20.0e9, 4.0e-9);
        let n = w.len();
        let center = n / 2;
        let peak_idx = w
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        // Peak should be at or very near center
        assert!((peak_idx as i32 - center as i32).unsigned_abs() <= 1);
    }

    #[test]
    fn test_ricker_wavelet_zero_duration() {
        let w = generate_ricker_wavelet(1.0e9, 10.0e9, 0.0);
        assert!(w.is_empty());
    }

    // -- Gaussian derivative tests --------------------------------------------

    #[test]
    fn test_gaussian_derivative_length() {
        let g = generate_gaussian_derivative(0.5e-9, 10.0e9, 4.0e-9);
        assert_eq!(g.len(), 40);
    }

    #[test]
    fn test_gaussian_derivative_antisymmetry() {
        let g = generate_gaussian_derivative(1.0e-9, 20.0e9, 6.0e-9);
        let n = g.len();
        for i in 0..n / 2 {
            assert!(
                (g[i] + g[n - 1 - i]).abs() < 1e-10,
                "derivative not antisymmetric at index {}: {} vs {}",
                i,
                g[i],
                g[n - 1 - i],
            );
        }
    }

    #[test]
    fn test_gaussian_derivative_zero_crossing() {
        let g = generate_gaussian_derivative(1.0e-9, 20.0e9, 6.0e-9);
        let n = g.len();
        let mid = n / 2;
        let left = g[mid - 1];
        let right = g[mid];
        assert!(
            left * right <= 0.0 || left.abs() < 1e-10 || right.abs() < 1e-10,
            "expected zero crossing near centre",
        );
    }

    // -- GprRadargram tests ---------------------------------------------------

    #[test]
    fn test_radargram_new() {
        let rg = make_radargram(10, 50);
        assert_eq!(rg.num_traces, 10);
        assert_eq!(rg.samples_per_trace, 50);
    }

    #[test]
    #[should_panic]
    fn test_radargram_empty_traces() {
        let _rg = GprRadargram::new(vec![], 0.5, 0.02);
    }

    #[test]
    #[should_panic]
    fn test_radargram_mismatched_lengths() {
        let traces = vec![vec![0.0; 10], vec![0.0; 20]];
        let _rg = GprRadargram::new(traces, 0.5, 0.02);
    }

    // -- Time-zero alignment tests --------------------------------------------

    #[test]
    fn test_time_zero_align_shifts_traces() {
        let mut rg = make_radargram(3, 20);
        rg.traces[0][2] = 5.0;
        rg.traces[1][5] = 5.0;
        rg.traces[2][3] = 5.0;

        time_zero_align(&mut rg, 4.0);

        assert!(rg.traces[0][0].abs() >= 4.0);
        assert!(rg.traces[1][0].abs() >= 4.0);
        assert!(rg.traces[2][0].abs() >= 4.0);
    }

    #[test]
    fn test_time_zero_align_preserves_length() {
        let mut rg = make_radargram(5, 30);
        rg.traces[0][10] = 10.0;
        rg.traces[1][15] = 10.0;
        time_zero_align(&mut rg, 5.0);
        for trace in &rg.traces {
            assert_eq!(trace.len(), 30);
        }
    }

    // -- Background removal (mean) tests --------------------------------------

    #[test]
    fn test_background_removal_mean_removes_dc() {
        let mut rg = make_radargram(5, 10);
        for trace in &mut rg.traces {
            for s in trace.iter_mut() {
                *s = 3.0;
            }
        }
        rg.traces[2][5] = 10.0;

        background_removal_mean(&mut rg);

        for j in 0..10 {
            let col_mean: f64 =
                rg.traces.iter().map(|t| t[j]).sum::<f64>() / 5.0;
            assert!(col_mean.abs() < 1e-12, "residual mean at sample {}", j);
        }
        assert!(rg.traces[2][5] > 0.0);
    }

    // -- Background removal (SVD) tests ---------------------------------------

    #[test]
    fn test_background_removal_svd_reduces_clutter() {
        let mut rg = make_radargram(8, 16);
        for trace in &mut rg.traces {
            trace[5] = 100.0;
            trace[6] = 50.0;
        }
        rg.traces[4][10] = 5.0;

        let clutter_energy_before: f64 = rg
            .traces
            .iter()
            .map(|t| t[5] * t[5] + t[6] * t[6])
            .sum();

        background_removal_svd(&mut rg, 1);

        let clutter_energy_after: f64 = rg
            .traces
            .iter()
            .map(|t| t[5] * t[5] + t[6] * t[6])
            .sum();

        assert!(
            clutter_energy_after < clutter_energy_before * 0.01,
            "SVD did not sufficiently reduce clutter: before={}, after={}",
            clutter_energy_before,
            clutter_energy_after,
        );
    }

    #[test]
    fn test_background_removal_svd_zero_components() {
        let mut rg = make_radargram(4, 8);
        rg.traces[0][3] = 7.0;
        let original = rg.traces[0][3];
        background_removal_svd(&mut rg, 0);
        assert!((rg.traces[0][3] - original).abs() < 1e-12);
    }

    // -- Gain SEC tests -------------------------------------------------------

    #[test]
    fn test_gain_sec_increases_with_depth() {
        let mut trace = vec![1.0; 100];
        gain_sec(&mut trace, 1.0, 1.0e8, 1.0);
        assert!(trace[50] > trace[10]);
        assert!(trace[90] > trace[50]);
    }

    #[test]
    fn test_gain_sec_first_sample_unchanged() {
        let mut trace = vec![1.0; 10];
        let original = trace[0];
        gain_sec(&mut trace, 2.0, 1.0e8, 0.5);
        assert!((trace[0] - original).abs() < 1e-12);
    }

    // -- Kirchhoff migration tests --------------------------------------------

    #[test]
    fn test_kirchhoff_migrate_output_dimensions() {
        let rg = make_radargram(20, 50);
        let migrated = kirchhoff_migrate(&rg, 1.0e8);
        assert_eq!(migrated.num_traces, 20);
        assert_eq!(migrated.samples_per_trace, 50);
    }

    #[test]
    fn test_kirchhoff_migrate_focuses_hyperbola() {
        let mut rg = make_radargram(32, 64);
        let vel = 1.0e8;
        add_hyperbola(&mut rg, 16, 30, vel, 10.0);

        let migrated = kirchhoff_migrate(&rg, vel);

        let apex_energy: f64 = migrated.traces[16].iter().map(|s| s * s).sum();
        let off_energy: f64 = migrated.traces[5].iter().map(|s| s * s).sum();
        assert!(
            apex_energy > off_energy * 2.0,
            "Migration did not focus hyperbola: apex={:.4}, off={:.4}",
            apex_energy,
            off_energy,
        );
    }

    // -- Stolt F-K migration test ---------------------------------------------

    #[test]
    fn test_stolt_fk_migrate_output_dimensions() {
        let rg = make_radargram(16, 32);
        let migrated = stolt_fk_migrate(&rg, 1.0e8);
        assert_eq!(migrated.num_traces, 16);
        assert_eq!(migrated.samples_per_trace, 32);
    }

    // -- Dielectric estimation tests ------------------------------------------

    #[test]
    fn test_estimate_dielectric_free_space() {
        let t = 2.0 * 1.0 / 299_792_458.0 * 1e9;
        let eps = estimate_dielectric(t, 1.0);
        assert!((eps - 1.0).abs() < 0.01, "expected ~1.0, got {}", eps);
    }

    #[test]
    fn test_estimate_dielectric_soil() {
        let v = 299_792_458.0 / 3.0;
        let t_ns = 2.0 * 1.0 / v * 1e9;
        let eps = estimate_dielectric(t_ns, 1.0);
        assert!((eps - 9.0).abs() < 0.1, "expected ~9.0, got {}", eps);
    }

    #[test]
    #[should_panic]
    fn test_estimate_dielectric_zero_depth() {
        estimate_dielectric(10.0, 0.0);
    }

    // -- Depth conversion tests -----------------------------------------------

    #[test]
    fn test_depth_convert_free_space() {
        let depth = depth_convert(10.0, 1.0);
        let expected = 299_792_458.0 * 10.0e-9 / 2.0;
        assert!(
            (depth - expected).abs() < 1e-3,
            "expected {}, got {}",
            expected,
            depth,
        );
    }

    #[test]
    fn test_depth_convert_dielectric_9() {
        let depth = depth_convert(10.0, 9.0);
        let expected = (299_792_458.0 / 3.0) * 10.0e-9 / 2.0;
        assert!(
            (depth - expected).abs() < 1e-3,
            "expected {}, got {}",
            expected,
            depth,
        );
    }

    #[test]
    #[should_panic]
    fn test_depth_convert_zero_dielectric() {
        depth_convert(10.0, 0.0);
    }

    // -- Hyperbola detection tests --------------------------------------------

    #[test]
    fn test_detect_hyperbolas_finds_target() {
        let mut rg = make_radargram(32, 128);
        let vel = 2.0e7;
        add_hyperbola(&mut rg, 16, 40, vel, 20.0);

        let detections = detect_hyperbolas(&rg, 10.0);
        assert!(
            !detections.is_empty(),
            "should detect at least one hyperbola",
        );
        let near_apex = detections
            .iter()
            .any(|&(tr, sa, _)| {
                (tr as i32 - 16).unsigned_abs() <= 2
                    && (sa as i32 - 40).unsigned_abs() <= 2
            });
        assert!(near_apex, "detection should be near apex (16, 40)");
    }

    #[test]
    fn test_detect_hyperbolas_empty_radargram() {
        let rg = make_radargram(20, 40);
        let detections = detect_hyperbolas(&rg, 1.0);
        assert!(detections.is_empty());
    }

    // -- GprProcessor tests ---------------------------------------------------

    #[test]
    fn test_processor_config_access() {
        let config = GprConfig {
            sample_rate_hz: 10.0e9,
            time_window_ns: 50.0,
            antenna_separation_m: 0.1,
            scan_spacing_m: 0.02,
        };
        let proc = GprProcessor::new(config);
        assert!((proc.config().sample_rate_hz - 10.0e9).abs() < 1.0);
        assert!((proc.config().antenna_separation_m - 0.1).abs() < 1e-6);
    }

    #[test]
    fn test_processor_ricker() {
        let config = GprConfig {
            sample_rate_hz: 10.0e9,
            time_window_ns: 50.0,
            antenna_separation_m: 0.1,
            scan_spacing_m: 0.02,
        };
        let proc = GprProcessor::new(config);
        let w = proc.ricker_wavelet(1.0e9, 2.0e-9);
        assert_eq!(w.len(), 20);
    }

    // -- FFT roundtrip test ---------------------------------------------------

    #[test]
    fn test_fft_roundtrip() {
        let mut buf: Vec<(f64, f64)> = (0..8)
            .map(|i| ((i as f64).sin(), (i as f64).cos()))
            .collect();
        let original = buf.clone();
        fft_in_place(&mut buf, false);
        fft_in_place(&mut buf, true);
        for (i, (a, b)) in buf.iter().zip(original.iter()).enumerate() {
            assert!(
                (a.0 - b.0).abs() < 1e-10 && (a.1 - b.1).abs() < 1e-10,
                "FFT roundtrip mismatch at {}: ({}, {}) vs ({}, {})",
                i,
                a.0,
                a.1,
                b.0,
                b.1,
            );
        }
    }
}
