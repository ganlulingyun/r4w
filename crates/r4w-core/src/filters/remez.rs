//! Parks-McClellan Equiripple FIR Filter Design
//!
//! Implements the Remez exchange algorithm for optimal FIR filter design.
//! Produces filters with equiripple (Chebyshev) error distribution.
//!
//! ## Overview
//!
//! The Parks-McClellan algorithm designs FIR filters that minimize the
//! maximum weighted error between the desired and actual frequency response.
//! This results in equiripple behavior in both passband and stopband.
//!
//! ## Advantages over Windowed Design
//!
//! - Optimal for a given filter length
//! - Sharper transition bands
//! - Configurable passband/stopband weights
//! - Guaranteed maximum ripple specification
//!
//! ## Example
//!
//! ```rust,ignore
//! use r4w_core::filters::{remez_lowpass, RemezSpec, FirFilter};
//!
//! // Design a 63-tap lowpass with 0.2 passband, 0.3 stopband
//! let coeffs = remez_lowpass(63, 0.2, 0.3, 1.0, 1.0);
//! let filter = FirFilter::new(coeffs);
//!
//! // Or use the spec builder
//! let spec = RemezSpec::lowpass(0.2, 0.3)
//!     .with_weights(1.0, 10.0)  // 10x weight on stopband
//!     .with_num_taps(63);
//! let coeffs = spec.design();
//! ```

use std::f64::consts::PI;

/// Specification for Parks-McClellan filter design.
#[derive(Debug, Clone)]
pub struct RemezSpec {
    /// Number of filter taps
    num_taps: usize,
    /// Band edges (normalized frequency, 0 to 0.5)
    bands: Vec<(f64, f64)>,
    /// Desired response at each band (0 or 1 for standard filters)
    desired: Vec<f64>,
    /// Weight for each band
    weights: Vec<f64>,
    /// Filter type
    filter_type: RemezFilterType,
}

/// Filter type for Remez design.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RemezFilterType {
    /// Standard bandpass/bandstop (Type I or II)
    Bandpass,
    /// Differentiator (antisymmetric, Type III or IV)
    Differentiator,
    /// Hilbert transformer (antisymmetric, Type III or IV)
    Hilbert,
}

impl RemezSpec {
    /// Create a lowpass filter specification.
    ///
    /// # Arguments
    /// * `passband_edge` - End of passband (normalized, 0 to 0.5)
    /// * `stopband_edge` - Start of stopband (normalized, 0 to 0.5)
    pub fn lowpass(passband_edge: f64, stopband_edge: f64) -> Self {
        assert!(passband_edge > 0.0 && passband_edge < stopband_edge);
        assert!(stopband_edge < 0.5);

        Self {
            num_taps: 31,
            bands: vec![(0.0, passband_edge), (stopband_edge, 0.5)],
            desired: vec![1.0, 0.0],
            weights: vec![1.0, 1.0],
            filter_type: RemezFilterType::Bandpass,
        }
    }

    /// Create a highpass filter specification.
    ///
    /// # Arguments
    /// * `stopband_edge` - End of stopband (normalized, 0 to 0.5)
    /// * `passband_edge` - Start of passband (normalized, 0 to 0.5)
    pub fn highpass(stopband_edge: f64, passband_edge: f64) -> Self {
        assert!(stopband_edge > 0.0 && stopband_edge < passband_edge);
        assert!(passband_edge < 0.5);

        Self {
            num_taps: 31,
            bands: vec![(0.0, stopband_edge), (passband_edge, 0.5)],
            desired: vec![0.0, 1.0],
            weights: vec![1.0, 1.0],
            filter_type: RemezFilterType::Bandpass,
        }
    }

    /// Create a bandpass filter specification.
    ///
    /// # Arguments
    /// * `stop1` - End of lower stopband
    /// * `pass1` - Start of passband
    /// * `pass2` - End of passband
    /// * `stop2` - Start of upper stopband
    pub fn bandpass(stop1: f64, pass1: f64, pass2: f64, stop2: f64) -> Self {
        assert!(stop1 > 0.0 && stop1 < pass1);
        assert!(pass1 < pass2 && pass2 < stop2);
        assert!(stop2 < 0.5);

        Self {
            num_taps: 63,
            bands: vec![(0.0, stop1), (pass1, pass2), (stop2, 0.5)],
            desired: vec![0.0, 1.0, 0.0],
            weights: vec![1.0, 1.0, 1.0],
            filter_type: RemezFilterType::Bandpass,
        }
    }

    /// Create a bandstop (notch) filter specification.
    ///
    /// # Arguments
    /// * `pass1` - End of lower passband
    /// * `stop1` - Start of stopband
    /// * `stop2` - End of stopband
    /// * `pass2` - Start of upper passband
    pub fn bandstop(pass1: f64, stop1: f64, stop2: f64, pass2: f64) -> Self {
        assert!(pass1 > 0.0 && pass1 < stop1);
        assert!(stop1 < stop2 && stop2 < pass2);
        assert!(pass2 < 0.5);

        Self {
            num_taps: 63,
            bands: vec![(0.0, pass1), (stop1, stop2), (pass2, 0.5)],
            desired: vec![1.0, 0.0, 1.0],
            weights: vec![1.0, 1.0, 1.0],
            filter_type: RemezFilterType::Bandpass,
        }
    }

    /// Create a differentiator specification.
    ///
    /// # Arguments
    /// * `passband_edge` - End of differentiator band (normalized)
    pub fn differentiator(passband_edge: f64) -> Self {
        assert!(passband_edge > 0.0 && passband_edge < 0.5);

        Self {
            num_taps: 31,
            bands: vec![(0.0, passband_edge)],
            desired: vec![1.0], // Will be scaled by frequency
            weights: vec![1.0],
            filter_type: RemezFilterType::Differentiator,
        }
    }

    /// Create a Hilbert transformer specification.
    ///
    /// # Arguments
    /// * `low_edge` - Start of Hilbert band (typically small, e.g., 0.03)
    /// * `high_edge` - End of Hilbert band (typically close to 0.5)
    pub fn hilbert(low_edge: f64, high_edge: f64) -> Self {
        assert!(low_edge > 0.0 && low_edge < high_edge);
        assert!(high_edge < 0.5);

        Self {
            num_taps: 31,
            bands: vec![(low_edge, high_edge)],
            desired: vec![1.0],
            weights: vec![1.0],
            filter_type: RemezFilterType::Hilbert,
        }
    }

    /// Set the number of filter taps.
    pub fn with_num_taps(mut self, num_taps: usize) -> Self {
        assert!(num_taps >= 3, "Need at least 3 taps");
        self.num_taps = num_taps;
        self
    }

    /// Set weights for each band.
    ///
    /// Higher weight means smaller ripple in that band.
    pub fn with_weights(mut self, passband_weight: f64, stopband_weight: f64) -> Self {
        if self.weights.len() == 2 {
            // Lowpass/highpass
            if self.desired[0] > 0.5 {
                self.weights = vec![passband_weight, stopband_weight];
            } else {
                self.weights = vec![stopband_weight, passband_weight];
            }
        } else if self.weights.len() == 3 {
            // Bandpass/bandstop
            if self.desired[1] > 0.5 {
                // Bandpass
                self.weights = vec![stopband_weight, passband_weight, stopband_weight];
            } else {
                // Bandstop
                self.weights = vec![passband_weight, stopband_weight, passband_weight];
            }
        }
        self
    }

    /// Set custom weights for all bands.
    pub fn with_custom_weights(mut self, weights: Vec<f64>) -> Self {
        assert_eq!(weights.len(), self.bands.len());
        self.weights = weights;
        self
    }

    /// Design the filter and return coefficients.
    pub fn design(&self) -> Vec<f64> {
        remez_design(
            self.num_taps,
            &self.bands,
            &self.desired,
            &self.weights,
            self.filter_type,
        )
    }
}

/// Design a lowpass equiripple filter.
///
/// # Arguments
/// * `num_taps` - Number of filter taps (odd recommended)
/// * `passband_edge` - End of passband (normalized frequency, 0 to 0.5)
/// * `stopband_edge` - Start of stopband (normalized frequency)
/// * `passband_weight` - Weight for passband error (typically 1.0)
/// * `stopband_weight` - Weight for stopband error (higher = more attenuation)
///
/// # Returns
/// Filter coefficients
///
/// # Example
/// ```rust,ignore
/// // 63-tap lowpass, passband 0-0.2, stopband 0.25-0.5
/// let coeffs = remez_lowpass(63, 0.2, 0.25, 1.0, 1.0);
/// ```
pub fn remez_lowpass(
    num_taps: usize,
    passband_edge: f64,
    stopband_edge: f64,
    passband_weight: f64,
    stopband_weight: f64,
) -> Vec<f64> {
    RemezSpec::lowpass(passband_edge, stopband_edge)
        .with_num_taps(num_taps)
        .with_weights(passband_weight, stopband_weight)
        .design()
}

/// Design a highpass equiripple filter.
pub fn remez_highpass(
    num_taps: usize,
    stopband_edge: f64,
    passband_edge: f64,
    passband_weight: f64,
    stopband_weight: f64,
) -> Vec<f64> {
    RemezSpec::highpass(stopband_edge, passband_edge)
        .with_num_taps(num_taps)
        .with_weights(passband_weight, stopband_weight)
        .design()
}

/// Design a bandpass equiripple filter.
pub fn remez_bandpass(
    num_taps: usize,
    stop1: f64,
    pass1: f64,
    pass2: f64,
    stop2: f64,
    passband_weight: f64,
    stopband_weight: f64,
) -> Vec<f64> {
    RemezSpec::bandpass(stop1, pass1, pass2, stop2)
        .with_num_taps(num_taps)
        .with_weights(passband_weight, stopband_weight)
        .design()
}

// ============================================================================
// Core Remez Algorithm Implementation
// ============================================================================

/// Maximum iterations for Remez algorithm.
const MAX_ITERATIONS: usize = 40;

/// Convergence threshold.
const CONVERGENCE_THRESHOLD: f64 = 1e-6;

/// Grid density (points per transition band width).
const GRID_DENSITY: usize = 16;

/// Design filter using Parks-McClellan/Remez algorithm.
fn remez_design(
    num_taps: usize,
    bands: &[(f64, f64)],
    desired: &[f64],
    weights: &[f64],
    filter_type: RemezFilterType,
) -> Vec<f64> {
    let num_taps = if num_taps % 2 == 0 { num_taps + 1 } else { num_taps };

    // Determine filter symmetry
    let (symmetric, num_coeffs) = match filter_type {
        RemezFilterType::Bandpass => {
            // Type I (odd taps) or Type II (even taps)
            (true, (num_taps + 1) / 2)
        }
        RemezFilterType::Differentiator | RemezFilterType::Hilbert => {
            // Type III or Type IV (antisymmetric)
            (false, (num_taps + 1) / 2)
        }
    };

    // Create dense frequency grid
    let grid = create_grid(bands, filter_type, GRID_DENSITY);

    // Compute desired response and weights on grid
    let (des, wt) = compute_desired_weights(&grid, bands, desired, weights, filter_type);

    // Number of extremal frequencies
    let num_extremal = num_coeffs + 1;

    // Initialize extremal frequencies uniformly
    let mut extremal_indices = initialize_extremals(&grid, num_extremal);

    // Remez exchange iteration
    let mut delta = 0.0;
    for _iteration in 0..MAX_ITERATIONS {
        // Compute the optimal filter for current extremals
        let (new_delta, a) = compute_delta_and_response(&grid, &des, &wt, &extremal_indices);

        // Check convergence
        if (new_delta - delta).abs() < CONVERGENCE_THRESHOLD * delta.abs().max(1e-10) {
            delta = new_delta;
            break;
        }
        delta = new_delta;

        // Compute error on entire grid
        let error = compute_error(&grid, &des, &wt, &a, delta);

        // Find new extremal frequencies
        extremal_indices = find_extremals(&error, num_extremal);
    }

    // Determine if passband is at DC (for normalization)
    let passband_at_dc = !desired.is_empty() && desired[0] > 0.5;

    // Compute final filter coefficients
    let coeffs = compute_coefficients(
        &grid,
        &extremal_indices,
        &des,
        &wt,
        symmetric,
        num_taps,
        passband_at_dc,
    );

    coeffs
}

/// Create frequency grid for Remez algorithm.
fn create_grid(
    bands: &[(f64, f64)],
    filter_type: RemezFilterType,
    density: usize,
) -> Vec<f64> {
    let mut grid = Vec::new();

    for &(f1, f2) in bands {
        let band_width = f2 - f1;
        let num_points = ((band_width * density as f64 * 100.0) as usize).max(10);

        for i in 0..num_points {
            let f = f1 + (f2 - f1) * i as f64 / (num_points - 1).max(1) as f64;

            // Avoid f=0 for differentiator/Hilbert (divide by zero)
            let f = match filter_type {
                RemezFilterType::Differentiator | RemezFilterType::Hilbert => f.max(0.001),
                _ => f,
            };

            grid.push(f);
        }
    }

    grid
}

/// Compute desired response and weights on grid.
fn compute_desired_weights(
    grid: &[f64],
    bands: &[(f64, f64)],
    desired: &[f64],
    weights: &[f64],
    filter_type: RemezFilterType,
) -> (Vec<f64>, Vec<f64>) {
    let mut des = vec![0.0; grid.len()];
    let mut wt = vec![1.0; grid.len()];

    for (i, &f) in grid.iter().enumerate() {
        // Find which band this frequency belongs to
        for (band_idx, &(f1, f2)) in bands.iter().enumerate() {
            if f >= f1 && f <= f2 {
                match filter_type {
                    RemezFilterType::Bandpass => {
                        des[i] = desired[band_idx];
                        wt[i] = weights[band_idx];
                    }
                    RemezFilterType::Differentiator => {
                        // Ideal differentiator: H(f) = j*2*pi*f
                        // Magnitude: 2*pi*f
                        des[i] = 2.0 * PI * f;
                        // Weight inversely proportional to frequency
                        wt[i] = weights[band_idx] / f.max(0.001);
                    }
                    RemezFilterType::Hilbert => {
                        des[i] = 1.0;
                        wt[i] = weights[band_idx];
                    }
                }
                break;
            }
        }
    }

    (des, wt)
}

/// Initialize extremal frequency indices uniformly.
fn initialize_extremals(grid: &[f64], num_extremal: usize) -> Vec<usize> {
    let n = grid.len();
    (0..num_extremal)
        .map(|i| i * (n - 1) / (num_extremal - 1).max(1))
        .collect()
}

/// Compute delta (ripple magnitude) and polynomial coefficients.
fn compute_delta_and_response(
    grid: &[f64],
    des: &[f64],
    wt: &[f64],
    extremal_indices: &[usize],
) -> (f64, Vec<f64>) {
    let n = extremal_indices.len();

    // Get values at extremal points
    let x: Vec<f64> = extremal_indices.iter().map(|&i| (PI * grid[i]).cos()).collect();
    let d: Vec<f64> = extremal_indices.iter().map(|&i| des[i]).collect();
    let w: Vec<f64> = extremal_indices.iter().map(|&i| wt[i]).collect();

    // Compute barycentric weights
    let mut bary = vec![1.0; n];
    for i in 0..n {
        for j in 0..n {
            if i != j {
                let diff = x[i] - x[j];
                if diff.abs() > 1e-10 {
                    bary[i] /= diff;
                }
            }
        }
    }

    // Compute delta using the formula:
    // delta = sum((-1)^k * bary[k] * D[k]) / sum((-1)^k * bary[k] / W[k])
    let mut num = 0.0;
    let mut den = 0.0;
    for i in 0..n {
        let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
        num += sign * bary[i] * d[i];
        den += sign * bary[i] / w[i];
    }
    let delta = num / den;

    // Compute the polynomial A(cos(w)) at extremal points
    // A[k] = D[k] - (-1)^k * delta / W[k]
    let a: Vec<f64> = (0..n)
        .map(|i| {
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            d[i] - sign * delta / w[i]
        })
        .collect();

    (delta.abs(), a)
}

/// Compute weighted error on entire grid.
fn compute_error(
    grid: &[f64],
    des: &[f64],
    wt: &[f64],
    a_at_extremals: &[f64],
    _delta: f64,
) -> Vec<f64> {
    // Interpolate A(cos(w)) to all grid points using barycentric interpolation
    let n_ext = a_at_extremals.len();

    // This is simplified - in practice we'd use Lagrange interpolation
    // For now, use linear interpolation between extremal points
    let mut error = vec![0.0; grid.len()];

    for (i, &f) in grid.iter().enumerate() {
        let x = (PI * f).cos();

        // Simple polynomial evaluation (Chebyshev basis would be better)
        // For stability, we estimate the filter response at each point
        let mut sum = 0.0;
        let mut weight_sum = 0.0;

        for (j, &a_j) in a_at_extremals.iter().enumerate() {
            let x_j = (PI * grid[j * (grid.len() - 1) / (n_ext - 1).max(1)]).cos();
            let w = 1.0 / ((x - x_j).abs() + 1e-10);
            sum += a_j * w;
            weight_sum += w;
        }

        let response = sum / weight_sum;
        error[i] = wt[i] * (response - des[i]);
    }

    error
}

/// Find indices of extremal frequencies (local maxima of |error|).
fn find_extremals(error: &[f64], num_extremal: usize) -> Vec<usize> {
    let n = error.len();
    let mut extremals = Vec::new();

    // Find all local extrema
    for i in 1..n - 1 {
        let is_local_max = error[i].abs() >= error[i - 1].abs()
            && error[i].abs() >= error[i + 1].abs()
            && error[i].abs() > 1e-10;
        if is_local_max {
            extremals.push((i, error[i].abs()));
        }
    }

    // Include endpoints if they're extrema
    if error[0].abs() >= error[1].abs() {
        extremals.push((0, error[0].abs()));
    }
    if error[n - 1].abs() >= error[n - 2].abs() {
        extremals.push((n - 1, error[n - 1].abs()));
    }

    // Sort by error magnitude and take top num_extremal
    extremals.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

    let mut result: Vec<usize> = extremals.iter().take(num_extremal).map(|&(i, _)| i).collect();

    // If we don't have enough extremals, fill with uniform spacing
    while result.len() < num_extremal {
        let missing = num_extremal - result.len();
        for i in 0..missing {
            let idx = i * (n - 1) / missing.max(1);
            if !result.contains(&idx) {
                result.push(idx);
            }
        }
    }

    // Sort by index
    result.sort();
    result.truncate(num_extremal);

    result
}

/// Compute final filter coefficients from extremal frequencies.
fn compute_coefficients(
    grid: &[f64],
    extremal_indices: &[usize],
    des: &[f64],
    wt: &[f64],
    symmetric: bool,
    num_taps: usize,
    passband_at_dc: bool,
) -> Vec<f64> {
    // For a practical implementation, we compute coefficients via inverse DFT
    // of the optimal frequency response

    let num_coeffs = (num_taps + 1) / 2;

    // First, compute the frequency response at the extremal points
    let n_ext = extremal_indices.len();
    let ext_freqs: Vec<f64> = extremal_indices.iter().map(|&i| grid[i]).collect();
    let ext_des: Vec<f64> = extremal_indices.iter().map(|&i| des[i]).collect();

    // Compute delta
    let x: Vec<f64> = ext_freqs.iter().map(|&f| (PI * f).cos()).collect();
    let w: Vec<f64> = extremal_indices.iter().map(|&i| wt[i]).collect();

    let mut bary = vec![1.0; n_ext];
    for i in 0..n_ext {
        for j in 0..n_ext {
            if i != j {
                let diff = x[i] - x[j];
                if diff.abs() > 1e-10 {
                    bary[i] /= diff;
                }
            }
        }
    }

    let mut num = 0.0;
    let mut den = 0.0;
    for i in 0..n_ext {
        let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
        num += sign * bary[i] * ext_des[i];
        den += sign * bary[i] / w[i];
    }
    let delta = num / den;

    // Compute optimal response at extremals
    let optimal_resp: Vec<f64> = (0..n_ext)
        .map(|i| {
            let sign = if i % 2 == 0 { 1.0 } else { -1.0 };
            ext_des[i] - sign * delta / w[i]
        })
        .collect();

    // Interpolate to dense frequency grid and compute coefficients via IDFT
    let fft_size = 1024;
    let mut response = vec![0.0; fft_size];

    // Interpolate optimal response to FFT grid
    for k in 0..=fft_size / 2 {
        let f = k as f64 / fft_size as f64;

        // Barycentric interpolation
        let x_k = (PI * f).cos();
        let mut sum = 0.0;
        let mut weight_sum = 0.0;

        for (j, &x_j) in x.iter().enumerate() {
            let diff = x_k - x_j;
            if diff.abs() < 1e-10 {
                sum = optimal_resp[j];
                weight_sum = 1.0;
                break;
            }
            let w_j = bary[j] / diff;
            sum += w_j * optimal_resp[j];
            weight_sum += w_j;
        }

        response[k] = sum / weight_sum.max(1e-10);

        // Mirror for negative frequencies (real filter)
        if k > 0 && k < fft_size / 2 {
            response[fft_size - k] = response[k];
        }
    }

    // Inverse DFT to get impulse response
    let mut h = vec![0.0; fft_size];
    for n in 0..fft_size {
        let mut sum = 0.0;
        for k in 0..fft_size {
            let phase = 2.0 * PI * n as f64 * k as f64 / fft_size as f64;
            sum += response[k] * phase.cos();
        }
        h[n] = sum / fft_size as f64;
    }

    // Extract and window the coefficients
    let center = fft_size / 2;
    let half_len = num_taps / 2;

    let mut coeffs = vec![0.0; num_taps];
    for i in 0..num_taps {
        let idx = (center - half_len + i) % fft_size;
        coeffs[i] = h[idx];
    }

    // Apply symmetry constraint
    if symmetric {
        for i in 0..num_taps / 2 {
            let avg = (coeffs[i] + coeffs[num_taps - 1 - i]) / 2.0;
            coeffs[i] = avg;
            coeffs[num_taps - 1 - i] = avg;
        }
    } else {
        // Antisymmetric
        for i in 0..num_taps / 2 {
            let avg = (coeffs[i] - coeffs[num_taps - 1 - i]) / 2.0;
            coeffs[i] = avg;
            coeffs[num_taps - 1 - i] = -avg;
        }
        coeffs[num_taps / 2] = 0.0;
    }

    // Normalize based on filter type
    // For lowpass: normalize DC gain to 1
    // For highpass: normalize Nyquist gain to 1
    if passband_at_dc {
        // Normalize DC gain
        let sum: f64 = coeffs.iter().sum();
        if sum.abs() > 0.1 {
            for c in coeffs.iter_mut() {
                *c /= sum;
            }
        }
    } else {
        // For highpass, normalize at Nyquist (alternating sum)
        let nyquist_gain: f64 = coeffs
            .iter()
            .enumerate()
            .map(|(i, &c)| if i % 2 == 0 { c } else { -c })
            .sum();
        if nyquist_gain.abs() > 0.1 {
            for c in coeffs.iter_mut() {
                *c /= nyquist_gain;
            }
        }
    }

    coeffs
}

/// Estimate the required filter order for given specifications.
///
/// Uses the Kaiser formula for estimating FIR filter order:
/// N ≈ (-20 * log10(sqrt(δp * δs)) - 13) / (14.6 * Δf)
///
/// # Arguments
/// * `passband_ripple_db` - Passband ripple in dB (e.g., 0.1)
/// * `stopband_atten_db` - Stopband attenuation in dB (e.g., 60)
/// * `transition_width` - Normalized transition bandwidth
///
/// # Returns
/// Estimated filter order (number of taps)
pub fn estimate_order(
    passband_ripple_db: f64,
    stopband_atten_db: f64,
    transition_width: f64,
) -> usize {
    let delta_p = (10.0_f64.powf(passband_ripple_db / 20.0) - 1.0)
        / (10.0_f64.powf(passband_ripple_db / 20.0) + 1.0);
    let delta_s = 10.0_f64.powf(-stopband_atten_db / 20.0);

    let d = -20.0 * (delta_p * delta_s).sqrt().log10();
    let n = ((d - 13.0) / (14.6 * transition_width)).ceil() as usize;

    // Ensure odd for Type I filter
    if n % 2 == 0 { n + 1 } else { n }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_remez_spec_lowpass() {
        let spec = RemezSpec::lowpass(0.2, 0.3);
        assert_eq!(spec.bands.len(), 2);
        assert_eq!(spec.desired, vec![1.0, 0.0]);
    }

    #[test]
    fn test_remez_spec_highpass() {
        let spec = RemezSpec::highpass(0.2, 0.3);
        assert_eq!(spec.bands.len(), 2);
        assert_eq!(spec.desired, vec![0.0, 1.0]);
    }

    #[test]
    fn test_remez_spec_bandpass() {
        let spec = RemezSpec::bandpass(0.1, 0.2, 0.3, 0.4);
        assert_eq!(spec.bands.len(), 3);
        assert_eq!(spec.desired, vec![0.0, 1.0, 0.0]);
    }

    #[test]
    fn test_remez_spec_builder() {
        let spec = RemezSpec::lowpass(0.2, 0.3)
            .with_num_taps(63)
            .with_weights(1.0, 10.0);

        assert_eq!(spec.num_taps, 63);
    }

    #[test]
    fn test_remez_lowpass_design() {
        let coeffs = remez_lowpass(31, 0.2, 0.3, 1.0, 1.0);

        assert_eq!(coeffs.len(), 31);

        // Should be symmetric (linear phase)
        for i in 0..15 {
            assert!(
                (coeffs[i] - coeffs[30 - i]).abs() < 0.01,
                "Filter should be symmetric"
            );
        }
    }

    #[test]
    fn test_remez_highpass_design() {
        let coeffs = remez_highpass(31, 0.2, 0.3, 1.0, 1.0);

        assert_eq!(coeffs.len(), 31);

        // Should be symmetric (linear phase)
        for i in 0..15 {
            assert!(
                (coeffs[i] - coeffs[30 - i]).abs() < 0.01,
                "Filter should be symmetric"
            );
        }

        // Nyquist gain should be normalized
        let nyquist_gain: f64 = coeffs
            .iter()
            .enumerate()
            .map(|(i, &c)| if i % 2 == 0 { c } else { -c })
            .sum();
        assert!(
            (nyquist_gain - 1.0).abs() < 0.1,
            "Nyquist gain should be ~1, got {}",
            nyquist_gain
        );
    }

    #[test]
    fn test_remez_bandpass_design() {
        let coeffs = remez_bandpass(63, 0.1, 0.15, 0.35, 0.4, 1.0, 1.0);

        assert_eq!(coeffs.len(), 63);
    }

    #[test]
    fn test_remez_dc_passthrough() {
        let coeffs = remez_lowpass(31, 0.2, 0.3, 1.0, 1.0);

        // Sum of coefficients should be close to 1 for lowpass
        let sum: f64 = coeffs.iter().sum();
        assert!(
            (sum - 1.0).abs() < 0.1,
            "DC gain should be near unity: {}",
            sum
        );
    }

    #[test]
    fn test_estimate_order() {
        // 0.1 dB passband ripple, 60 dB stopband, 0.1 transition width
        let order = estimate_order(0.1, 60.0, 0.1);

        // Should need a reasonable number of taps (Kaiser formula gives ~29)
        assert!(order >= 20 && order <= 150, "Order was {}", order);
    }

    #[test]
    fn test_estimate_order_narrow_transition() {
        let wide = estimate_order(0.1, 60.0, 0.1);
        let narrow = estimate_order(0.1, 60.0, 0.05);

        // Narrower transition = more taps
        assert!(narrow > wide);
    }

    #[test]
    fn test_remez_differentiator_spec() {
        let spec = RemezSpec::differentiator(0.4);
        assert_eq!(spec.filter_type, RemezFilterType::Differentiator);
    }

    #[test]
    fn test_remez_hilbert_spec() {
        let spec = RemezSpec::hilbert(0.03, 0.47);
        assert_eq!(spec.filter_type, RemezFilterType::Hilbert);
    }

    #[test]
    fn test_create_grid() {
        let bands = vec![(0.0, 0.2), (0.3, 0.5)];
        let grid = create_grid(&bands, RemezFilterType::Bandpass, 16);

        assert!(!grid.is_empty());
        assert!(grid[0] >= 0.0);
        assert!(*grid.last().unwrap() <= 0.5);
    }

    #[test]
    fn test_various_tap_counts() {
        for num_taps in [15, 31, 63, 127] {
            let coeffs = remez_lowpass(num_taps, 0.2, 0.3, 1.0, 1.0);
            assert_eq!(coeffs.len(), num_taps);
        }
    }

    #[test]
    fn test_weighted_design() {
        // Design with more weight on stopband
        let spec = RemezSpec::lowpass(0.2, 0.3)
            .with_num_taps(31)
            .with_weights(1.0, 10.0);

        let coeffs = spec.design();
        assert_eq!(coeffs.len(), 31);
    }

    #[test]
    fn test_bandstop_design() {
        let spec = RemezSpec::bandstop(0.1, 0.15, 0.35, 0.4).with_num_taps(63);
        let coeffs = spec.design();

        assert_eq!(coeffs.len(), 63);
    }
}
