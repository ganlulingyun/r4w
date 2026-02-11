//! Radio Direction Finder (RDF)
//!
//! Implements radio direction-finding algorithms for bearing estimation from
//! multi-antenna arrays. Supported methods:
//!
//! - **Watson-Watt**: Classic crossed-loop / sense-antenna approach.
//!   Uses atan2(E_rms, N_rms) with a sense antenna to resolve the 180-degree
//!   ambiguity inherent in the loop response.
//! - **Correlative Interferometer**: Phase-comparison across antenna pairs
//!   with known baseline spacings.
//! - **Amplitude Comparison**: Directional antenna patterns compared by
//!   amplitude to find the most-likely bearing.
//! - **Phase (Adcock/Sense)**: Simple phase-difference bearing for a pair of
//!   antennas.
//! - **MUSIC (MUltiple SIgnal Classification)**: Subspace-based
//!   super-resolution algorithm for resolving closely-spaced sources.
//!
//! Complex values are represented as `(f64, f64)` tuples of `(re, im)` to
//! keep the module self-contained with no external crate dependencies.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::radio_direction_finder::{
//!     RdfConfig, RdfMethod, RdfProcessor, watson_watt_bearing,
//!     bearing_error, circular_mean_bearing,
//! };
//!
//! // Watson-Watt bearing from cardinal-direction signals
//! let n = 128;
//! let freq = 10.0;
//! // Signal arriving from 0 deg (north): strong on N loop, zero on E loop
//! let north: Vec<(f64, f64)> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / n as f64;
//!         ((2.0 * std::f64::consts::PI * freq * t).cos(), 0.0)
//!     })
//!     .collect();
//! let east: Vec<(f64, f64)> = vec![(0.0, 0.0); n];
//! let sense: Vec<(f64, f64)> = north.clone();
//! let bearing = watson_watt_bearing(&north, &east, &sense);
//! assert!((bearing - 0.0).abs() < 5.0 || (bearing - 360.0).abs() < 5.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers  (re, im) tuples
// ---------------------------------------------------------------------------

/// Multiply two complex numbers represented as (re, im) tuples.
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Add two complex numbers.
#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Subtract two complex numbers.
#[inline]
fn csub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Complex conjugate.
#[inline]
fn conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Squared magnitude of a complex number.
#[inline]
fn cnorm_sq(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Magnitude of a complex number.
#[inline]
fn cabs(a: (f64, f64)) -> f64 {
    cnorm_sq(a).sqrt()
}

/// Construct a complex exponential `exp(j * phase)`.
#[inline]
fn cexp_j(phase: f64) -> (f64, f64) {
    (phase.cos(), phase.sin())
}

/// Scale a complex number by a real scalar.
#[inline]
fn cscale(s: f64, a: (f64, f64)) -> (f64, f64) {
    (s * a.0, s * a.1)
}

/// Divide complex `a` by complex `b`.
#[inline]
fn cdiv(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = cnorm_sq(b);
    if denom < 1e-30 {
        (0.0, 0.0)
    } else {
        ((a.0 * b.0 + a.1 * b.1) / denom, (a.1 * b.0 - a.0 * b.1) / denom)
    }
}

// ---------------------------------------------------------------------------
// Tiny linear-algebra helpers for dense complex matrices stored as Vec<Vec<>>
// ---------------------------------------------------------------------------

/// Complex matrix-vector multiply: y = A * x.
fn mat_vec_mul(a: &[Vec<(f64, f64)>], x: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let m = a.len();
    let mut y = vec![(0.0, 0.0); m];
    for i in 0..m {
        for (j, xj) in x.iter().enumerate() {
            y[i] = cadd(y[i], cmul(a[i][j], *xj));
        }
    }
    y
}

/// Hermitian transpose of a complex matrix.
fn mat_hermitian(a: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
    let m = a.len();
    let n = a[0].len();
    let mut ah = vec![vec![(0.0, 0.0); m]; n];
    for i in 0..m {
        for j in 0..n {
            ah[j][i] = conj(a[i][j]);
        }
    }
    ah
}

/// Complex matrix multiply: C = A * B.
fn mat_mul(a: &[Vec<(f64, f64)>], b: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
    let m = a.len();
    let p = b[0].len();
    let n = b.len();
    let mut c = vec![vec![(0.0, 0.0); p]; m];
    for i in 0..m {
        for k in 0..n {
            let aik = a[i][k];
            for j in 0..p {
                c[i][j] = cadd(c[i][j], cmul(aik, b[k][j]));
            }
        }
    }
    c
}

/// Subtract two same-sized matrices: C = A - B.
fn mat_sub(a: &[Vec<(f64, f64)>], b: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
    let m = a.len();
    let n = a[0].len();
    let mut c = vec![vec![(0.0, 0.0); n]; m];
    for i in 0..m {
        for j in 0..n {
            c[i][j] = csub(a[i][j], b[i][j]);
        }
    }
    c
}

/// Identity matrix of size n.
fn mat_eye(n: usize) -> Vec<Vec<(f64, f64)>> {
    let mut m = vec![vec![(0.0, 0.0); n]; n];
    for i in 0..n {
        m[i][i] = (1.0, 0.0);
    }
    m
}

/// Outer product of two complex vectors: M = a * b^H.
fn outer_product(a: &[(f64, f64)], b: &[(f64, f64)]) -> Vec<Vec<(f64, f64)>> {
    let m = a.len();
    let n = b.len();
    let mut out = vec![vec![(0.0, 0.0); n]; m];
    for i in 0..m {
        for j in 0..n {
            out[i][j] = cmul(a[i], conj(b[j]));
        }
    }
    out
}

// ---------------------------------------------------------------------------
// Eigenvalue decomposition via repeated QR iteration (Householder)
// ---------------------------------------------------------------------------

/// Compute eigenvalues and eigenvectors of a Hermitian matrix using QR
/// iteration. Returns `(eigenvalues_real, eigenvectors_as_columns)`.
///
/// The eigenvalues are sorted in descending order. Only real eigenvalues
/// are returned since the input is assumed Hermitian.
fn eigen_hermitian(
    mat: &[Vec<(f64, f64)>],
    max_iter: usize,
) -> (Vec<f64>, Vec<Vec<(f64, f64)>>) {
    let n = mat.len();
    // Work on a copy
    let mut a: Vec<Vec<(f64, f64)>> = mat.to_vec();
    // Accumulate eigenvectors
    let mut v = mat_eye(n);

    for _ in 0..max_iter {
        // Simple QR via Gram-Schmidt
        let (q, r) = qr_decomposition(&a);
        // A <- R * Q
        a = mat_mul(&r, &q);
        // V <- V * Q
        v = mat_mul(&v, &q);
    }

    // Eigenvalues are diagonal of A (real parts for Hermitian)
    let mut eig_vals: Vec<f64> = (0..n).map(|i| a[i][i].0).collect();

    // Sort descending by eigenvalue, rearranging eigenvectors accordingly
    let mut indices: Vec<usize> = (0..n).collect();
    indices.sort_by(|&a, &b| eig_vals[b].partial_cmp(&eig_vals[a]).unwrap());

    let sorted_vals: Vec<f64> = indices.iter().map(|&i| eig_vals[i]).collect();
    let sorted_vecs: Vec<Vec<(f64, f64)>> = {
        // Eigenvectors as columns of v - extract them
        let mut cols = vec![vec![(0.0, 0.0); n]; n];
        for (new_j, &old_j) in indices.iter().enumerate() {
            for i in 0..n {
                cols[new_j][i] = v[i][old_j];
            }
        }
        cols
    };

    let _ = eig_vals; // shadow
    (sorted_vals, sorted_vecs)
}

/// Gram-Schmidt QR decomposition of a complex matrix (column-oriented).
fn qr_decomposition(
    a: &[Vec<(f64, f64)>],
) -> (Vec<Vec<(f64, f64)>>, Vec<Vec<(f64, f64)>>) {
    let m = a.len();
    let n = a[0].len();

    // Extract columns
    let mut cols: Vec<Vec<(f64, f64)>> = (0..n)
        .map(|j| (0..m).map(|i| a[i][j]).collect())
        .collect();

    let mut q_cols: Vec<Vec<(f64, f64)>> = Vec::with_capacity(n);
    let mut r = vec![vec![(0.0, 0.0); n]; n];

    for j in 0..n {
        let mut u = cols[j].clone();
        for (k, qk) in q_cols.iter().enumerate() {
            // r[k][j] = <q_k, a_j>
            let mut dot = (0.0, 0.0);
            for i in 0..m {
                dot = cadd(dot, cmul(conj(qk[i]), cols[j][i]));
            }
            r[k][j] = dot;
            // u -= r[k][j] * q_k
            for i in 0..m {
                u[i] = csub(u[i], cmul(dot, qk[i]));
            }
        }
        // r[j][j] = ||u||
        let norm: f64 = u.iter().map(|x| cnorm_sq(*x)).sum::<f64>().sqrt();
        r[j][j] = (norm, 0.0);
        if norm > 1e-14 {
            let inv = 1.0 / norm;
            for x in &mut u {
                *x = cscale(inv, *x);
            }
        }
        q_cols.push(u);
    }

    // Build Q as row-major matrix
    let mut q = vec![vec![(0.0, 0.0); n]; m];
    for j in 0..n {
        for i in 0..m {
            q[i][j] = q_cols[j][i];
        }
    }

    (q, r)
}

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Direction-finding method.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RdfMethod {
    /// Watson-Watt crossed-loop with sense antenna.
    WatsonWatt,
    /// Correlative interferometer (phase comparison).
    Correlative,
    /// Amplitude comparison of directional antennas.
    Amplitude,
    /// Phase-difference (Adcock/sense).
    Phase,
    /// MUSIC super-resolution algorithm.
    Music,
}

/// Configuration for an RDF processor.
#[derive(Debug, Clone)]
pub struct RdfConfig {
    /// Direction-finding method to use.
    pub method: RdfMethod,
    /// Number of antenna elements in the array.
    pub num_antennas: usize,
    /// Physical spacing between adjacent antenna elements (metres).
    pub antenna_spacing_m: f64,
    /// Centre frequency of the signal of interest (Hz).
    pub center_freq_hz: f64,
    /// Sample rate of the digitised antenna signals (Hz).
    pub sample_rate_hz: f64,
}

/// Result of a bearing estimation.
#[derive(Debug, Clone)]
pub struct BearingEstimate {
    /// Estimated bearing in degrees [0, 360).
    pub bearing_deg: f64,
    /// Confidence metric in [0, 1] (1 = highest confidence).
    pub confidence: f64,
    /// Whether the estimate may have an unresolved 180-degree ambiguity.
    pub ambiguity: bool,
    /// Optional elevation estimate (degrees above horizon).
    pub elevation_deg: Option<f64>,
}

/// Direction-finding processor.
///
/// Wraps an [`RdfConfig`] and dispatches `estimate_bearing` to the
/// configured algorithm.
#[derive(Debug, Clone)]
pub struct RdfProcessor {
    config: RdfConfig,
}

impl RdfProcessor {
    /// Create a new processor from the given configuration.
    pub fn new(config: RdfConfig) -> Self {
        Self { config }
    }

    /// Estimate the bearing of an incoming signal.
    ///
    /// `antenna_signals` contains one `Vec<(f64, f64)>` per antenna element.
    /// Each inner vector is a time series of complex I/Q samples from that
    /// element.
    ///
    /// The interpretation of the antenna signals depends on the configured
    /// method:
    ///
    /// - **WatsonWatt**: expects 3 channels: `[north_loop, east_loop, sense]`.
    /// - **Correlative**: phases are extracted from consecutive antenna pairs.
    /// - **Amplitude**: amplitudes are extracted; antenna pointing directions
    ///   are assumed equally spaced around 360 degrees.
    /// - **Phase**: uses the first two antennas as an Adcock pair.
    /// - **Music**: computes the spatial covariance matrix and applies MUSIC
    ///   with `num_signals = 1`.
    pub fn estimate_bearing(
        &self,
        antenna_signals: &[Vec<(f64, f64)>],
    ) -> BearingEstimate {
        match self.config.method {
            RdfMethod::WatsonWatt => {
                if antenna_signals.len() < 3 {
                    return BearingEstimate {
                        bearing_deg: 0.0,
                        confidence: 0.0,
                        ambiguity: true,
                        elevation_deg: None,
                    };
                }
                let bearing = watson_watt_bearing(
                    &antenna_signals[0],
                    &antenna_signals[1],
                    &antenna_signals[2],
                );
                BearingEstimate {
                    bearing_deg: bearing,
                    confidence: 0.9,
                    ambiguity: false,
                    elevation_deg: None,
                }
            }
            RdfMethod::Correlative => {
                let wavelength = speed_of_light() / self.config.center_freq_hz;
                let baseline_wl = self.config.antenna_spacing_m / wavelength;
                // Extract phase differences between adjacent antenna pairs
                let phases = extract_phase_differences(antenna_signals);
                let bearing = correlative_interferometer(&phases, baseline_wl);
                BearingEstimate {
                    bearing_deg: bearing,
                    confidence: 0.85,
                    ambiguity: false,
                    elevation_deg: None,
                }
            }
            RdfMethod::Amplitude => {
                let n = antenna_signals.len();
                let amplitudes: Vec<f64> = antenna_signals
                    .iter()
                    .map(|sig| rms_amplitude(sig))
                    .collect();
                let patterns: Vec<f64> = (0..n)
                    .map(|i| i as f64 * 360.0 / n as f64)
                    .collect();
                let bearing = amplitude_comparison(&amplitudes, &patterns);
                BearingEstimate {
                    bearing_deg: bearing,
                    confidence: 0.7,
                    ambiguity: false,
                    elevation_deg: None,
                }
            }
            RdfMethod::Phase => {
                if antenna_signals.len() < 2 {
                    return BearingEstimate {
                        bearing_deg: 0.0,
                        confidence: 0.0,
                        ambiguity: true,
                        elevation_deg: None,
                    };
                }
                let wavelength = speed_of_light() / self.config.center_freq_hz;
                let baseline_wl = self.config.antenna_spacing_m / wavelength;
                let phase = mean_phase_difference(
                    &antenna_signals[0],
                    &antenna_signals[1],
                );
                // bearing = arcsin(phase / (2*pi*d/lambda))
                let arg = phase / (2.0 * PI * baseline_wl);
                let arg_clamped = arg.max(-1.0).min(1.0);
                let bearing_rad = arg_clamped.asin();
                let bearing_deg = normalize_bearing(bearing_rad.to_degrees());
                BearingEstimate {
                    bearing_deg,
                    confidence: 0.75,
                    ambiguity: baseline_wl > 0.5,
                    elevation_deg: None,
                }
            }
            RdfMethod::Music => {
                let wavelength = speed_of_light() / self.config.center_freq_hz;
                let spacing_wl = self.config.antenna_spacing_m / wavelength;
                let cov = compute_covariance_matrix(antenna_signals);
                let bearings = music_bearing(&cov, 1, spacing_wl);
                let bearing = if bearings.is_empty() {
                    0.0
                } else {
                    bearings[0]
                };
                BearingEstimate {
                    bearing_deg: bearing,
                    confidence: 0.95,
                    ambiguity: false,
                    elevation_deg: None,
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Public algorithm functions
// ---------------------------------------------------------------------------

/// Watson-Watt bearing estimation.
///
/// Given three antenna channels (north loop, east loop, omnidirectional sense),
/// computes `atan2(E_rms, N_rms)` to get the bearing. The sense antenna
/// resolves the inherent 180-degree ambiguity of the loop antennas: if the
/// cross-correlation of the sense antenna with the north loop is negative,
/// the bearing is flipped by 180 degrees.
///
/// Returns bearing in degrees [0, 360).
pub fn watson_watt_bearing(
    north_signal: &[(f64, f64)],
    east_signal: &[(f64, f64)],
    sense_signal: &[(f64, f64)],
) -> f64 {
    let n_rms = rms_amplitude(north_signal);
    let e_rms = rms_amplitude(east_signal);

    // Signed RMS: use cross-correlation with sense to determine sign
    let n_corr = cross_correlate_real(north_signal, sense_signal);
    let e_corr = cross_correlate_real(east_signal, sense_signal);

    let n_signed = if n_corr >= 0.0 { n_rms } else { -n_rms };
    let e_signed = if e_corr >= 0.0 { e_rms } else { -e_rms };

    // atan2(east, north) gives bearing from north, clockwise positive
    let bearing_rad = e_signed.atan2(n_signed);
    normalize_bearing(bearing_rad.to_degrees())
}

/// Correlative interferometer bearing estimation.
///
/// Given an array of phase differences (radians) between adjacent antenna
/// pairs and the baseline spacing in wavelengths, estimates the bearing by
/// fitting `phase = 2 * pi * d * sin(theta)` via least-squares over the
/// measured phases.
///
/// Returns bearing in degrees [0, 360).
pub fn correlative_interferometer(phases: &[f64], baseline_wavelengths: f64) -> f64 {
    if phases.is_empty() {
        return 0.0;
    }

    // Average the phase differences
    let mean_phase: f64 = phases.iter().copied().sum::<f64>() / phases.len() as f64;

    // phase = 2*pi*d/lambda * sin(theta)
    let arg = mean_phase / (2.0 * PI * baseline_wavelengths);
    let arg_clamped = arg.max(-1.0).min(1.0);
    let bearing_rad = arg_clamped.asin();
    normalize_bearing(bearing_rad.to_degrees())
}

/// Amplitude comparison bearing estimation.
///
/// Compares the signal amplitudes received on multiple directional antennas.
/// `antenna_patterns_deg` gives the boresight pointing direction of each
/// antenna in degrees. The bearing is estimated via weighted centroid of the
/// antenna pointing angles, weighted by measured amplitude.
///
/// Returns bearing in degrees [0, 360).
pub fn amplitude_comparison(amplitudes: &[f64], antenna_patterns_deg: &[f64]) -> f64 {
    if amplitudes.is_empty() || amplitudes.len() != antenna_patterns_deg.len() {
        return 0.0;
    }

    // Use weighted circular mean so angles wrap correctly
    let mut sum_sin = 0.0;
    let mut sum_cos = 0.0;
    for (amp, &dir) in amplitudes.iter().zip(antenna_patterns_deg.iter()) {
        let rad = dir.to_radians();
        sum_sin += amp * rad.sin();
        sum_cos += amp * rad.cos();
    }

    let bearing_rad = sum_sin.atan2(sum_cos);
    normalize_bearing(bearing_rad.to_degrees())
}

/// MUSIC (MUltiple SIgnal Classification) bearing estimation.
///
/// Given the spatial covariance matrix of an M-element uniform linear array,
/// the number of signal sources `num_signals`, and the inter-element spacing
/// in wavelengths, performs eigendecomposition to separate signal and noise
/// subspaces, then scans for peaks in the MUSIC pseudo-spectrum.
///
/// Returns a vector of estimated bearings in degrees [0, 360), one per
/// detected signal source, sorted by pseudo-spectrum peak height
/// (strongest first).
pub fn music_bearing(
    covariance_matrix: &[Vec<(f64, f64)>],
    num_signals: usize,
    antenna_spacing_wavelengths: f64,
) -> Vec<f64> {
    let m = covariance_matrix.len();
    if m == 0 || num_signals >= m {
        return vec![];
    }

    // Eigendecomposition of the covariance matrix
    let (eigenvalues, eigenvectors) = eigen_hermitian(covariance_matrix, 200);

    // Noise subspace: eigenvectors corresponding to the (m - num_signals)
    // smallest eigenvalues. eigenvectors are sorted descending, so noise
    // eigenvectors are indices [num_signals .. m].
    let noise_start = num_signals;
    let noise_dim = m - num_signals;

    // Build noise subspace matrix E_n (m x noise_dim)
    // eigenvectors[k] is the k-th column vector (stored as Vec)
    let en: Vec<Vec<(f64, f64)>> = (noise_start..m)
        .map(|k| eigenvectors[k].clone())
        .collect();

    // Scan bearing from -90 to +90 degrees (for a ULA)
    let scan_steps = 3600;
    let mut spectrum = vec![0.0f64; scan_steps];

    for (idx, spec_val) in spectrum.iter_mut().enumerate() {
        let theta = -90.0 + 180.0 * idx as f64 / scan_steps as f64;
        let a = steering_vector(theta, m, antenna_spacing_wavelengths);

        // MUSIC pseudo-spectrum: 1 / (a^H * En * En^H * a)
        // = 1 / sum_k |a^H * e_k|^2  where e_k are noise eigenvectors
        let mut denom = 0.0;
        for noise_vec in &en {
            // a^H * e_k
            let mut dot = (0.0, 0.0);
            for i in 0..m {
                dot = cadd(dot, cmul(conj(a[i]), noise_vec[i]));
            }
            denom += cnorm_sq(dot);
        }

        *spec_val = if denom > 1e-30 { 1.0 / denom } else { 1e30 };
    }

    // Find peaks: num_signals largest local maxima
    let mut peaks: Vec<(usize, f64)> = Vec::new();
    for i in 1..scan_steps - 1 {
        if spectrum[i] > spectrum[i - 1] && spectrum[i] > spectrum[i + 1] {
            peaks.push((i, spectrum[i]));
        }
    }
    // Sort by height descending
    peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

    peaks
        .iter()
        .take(num_signals)
        .map(|&(idx, _)| {
            let theta = -90.0 + 180.0 * idx as f64 / scan_steps as f64;
            normalize_bearing(theta)
        })
        .collect()
}

/// Compute the spatial covariance matrix from multi-antenna I/Q signals.
///
/// `signals[k]` is the complex time series from antenna element `k`.
/// Returns the M x M sample covariance matrix
/// `R = (1/N) * sum_n x(n) * x(n)^H` where `x(n)` is the M-element
/// snapshot vector at sample index `n`.
pub fn compute_covariance_matrix(
    signals: &[Vec<(f64, f64)>],
) -> Vec<Vec<(f64, f64)>> {
    let m = signals.len();
    if m == 0 {
        return vec![];
    }
    let n = signals[0].len();
    if n == 0 {
        return vec![vec![(0.0, 0.0); m]; m];
    }

    let mut cov = vec![vec![(0.0, 0.0); m]; m];

    for sample_idx in 0..n {
        // Build snapshot vector
        let snapshot: Vec<(f64, f64)> =
            signals.iter().map(|s| s[sample_idx]).collect();

        // Outer product accumulation
        for i in 0..m {
            for j in 0..m {
                let prod = cmul(snapshot[i], conj(snapshot[j]));
                cov[i][j] = cadd(cov[i][j], prod);
            }
        }
    }

    // Normalize
    let inv_n = 1.0 / n as f64;
    for row in &mut cov {
        for val in row.iter_mut() {
            *val = cscale(inv_n, *val);
        }
    }

    cov
}

/// Array steering vector for a uniform linear array.
///
/// For bearing `bearing_deg` (measured from array broadside), returns the
/// M-element complex steering vector:
///
/// `a_k = exp(-j * 2 * pi * k * d * sin(theta))`
///
/// where `k = 0..num_antennas-1`, `d` is `spacing_wavelengths`, and
/// `theta` is the bearing in radians.
pub fn steering_vector(
    bearing_deg: f64,
    num_antennas: usize,
    spacing_wavelengths: f64,
) -> Vec<(f64, f64)> {
    let theta_rad = bearing_deg.to_radians();
    let phase_inc = -2.0 * PI * spacing_wavelengths * theta_rad.sin();

    (0..num_antennas)
        .map(|k| cexp_j(phase_inc * k as f64))
        .collect()
}

/// Circular (angular) mean of a set of bearings in degrees.
///
/// Computes the mean direction on the unit circle so that wrapping around
/// 0/360 is handled correctly.
///
/// Returns a bearing in degrees [0, 360).
pub fn circular_mean_bearing(bearings: &[f64]) -> f64 {
    if bearings.is_empty() {
        return 0.0;
    }

    let mut sum_sin = 0.0;
    let mut sum_cos = 0.0;
    for &b in bearings {
        let rad = b.to_radians();
        sum_sin += rad.sin();
        sum_cos += rad.cos();
    }

    let mean_rad = sum_sin.atan2(sum_cos);
    normalize_bearing(mean_rad.to_degrees())
}

/// Shortest angular difference between two bearings in degrees.
///
/// Returns a value in [0, 180].
pub fn bearing_error(true_deg: f64, estimated_deg: f64) -> f64 {
    let mut diff = (estimated_deg - true_deg) % 360.0;
    if diff < 0.0 {
        diff += 360.0;
    }
    if diff > 180.0 {
        diff = 360.0 - diff;
    }
    diff
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Speed of light in m/s.
fn speed_of_light() -> f64 {
    299_792_458.0
}

/// RMS amplitude of a complex signal.
fn rms_amplitude(signal: &[(f64, f64)]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    let sum: f64 = signal.iter().map(|s| cnorm_sq(*s)).sum();
    (sum / signal.len() as f64).sqrt()
}

/// Real-valued cross-correlation at lag 0.
///
/// Sums `Re(x[n] * conj(y[n]))` over all samples. A positive result means
/// the signals are in-phase; negative means anti-phase.
fn cross_correlate_real(x: &[(f64, f64)], y: &[(f64, f64)]) -> f64 {
    let n = x.len().min(y.len());
    let mut sum = 0.0;
    for i in 0..n {
        let prod = cmul(x[i], conj(y[i]));
        sum += prod.0; // real part
    }
    sum
}

/// Mean phase difference between two antenna signals.
///
/// Computes the average of `arg(x1[n] * conj(x0[n]))` over all samples.
fn mean_phase_difference(sig0: &[(f64, f64)], sig1: &[(f64, f64)]) -> f64 {
    let n = sig0.len().min(sig1.len());
    if n == 0 {
        return 0.0;
    }
    let mut sum_sin = 0.0;
    let mut sum_cos = 0.0;
    for i in 0..n {
        let prod = cmul(sig1[i], conj(sig0[i]));
        let phase = prod.1.atan2(prod.0);
        sum_sin += phase.sin();
        sum_cos += phase.cos();
    }
    sum_sin.atan2(sum_cos)
}

/// Extract phase differences between adjacent antenna pairs.
fn extract_phase_differences(signals: &[Vec<(f64, f64)>]) -> Vec<f64> {
    if signals.len() < 2 {
        return vec![];
    }
    (0..signals.len() - 1)
        .map(|k| mean_phase_difference(&signals[k], &signals[k + 1]))
        .collect()
}

/// Normalize bearing to [0, 360).
fn normalize_bearing(deg: f64) -> f64 {
    let mut b = deg % 360.0;
    if b < 0.0 {
        b += 360.0;
    }
    b
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: generate a complex sinusoid at the given frequency.
    fn tone(freq: f64, sample_rate: f64, n: usize, amplitude: f64) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * freq * t;
                (amplitude * phase.cos(), amplitude * phase.sin())
            })
            .collect()
    }

    /// Helper: generate Watson-Watt signals for a known bearing.
    /// Bearing is in degrees from north, clockwise.
    fn watson_watt_signals(
        bearing_deg: f64,
        n: usize,
    ) -> (Vec<(f64, f64)>, Vec<(f64, f64)>, Vec<(f64, f64)>) {
        let theta = bearing_deg.to_radians();
        let freq = 10.0;
        let sr = 1000.0;
        // Loop antenna responses: North ~ cos(theta), East ~ sin(theta)
        let n_amp = theta.cos();
        let e_amp = theta.sin();
        let north = tone(freq, sr, n, n_amp);
        let east = tone(freq, sr, n, e_amp);
        // Sense antenna is omni (positive reference)
        let sense = tone(freq, sr, n, 1.0);
        (north, east, sense)
    }

    // -----------------------------------------------------------------------
    // Watson-Watt tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_watson_watt_north() {
        let (n, e, s) = watson_watt_signals(0.0, 256);
        let bearing = watson_watt_bearing(&n, &e, &s);
        assert!(
            bearing_error(0.0, bearing) < 5.0,
            "Expected ~0 deg, got {bearing}"
        );
    }

    #[test]
    fn test_watson_watt_east() {
        let (n, e, s) = watson_watt_signals(90.0, 256);
        let bearing = watson_watt_bearing(&n, &e, &s);
        assert!(
            bearing_error(90.0, bearing) < 5.0,
            "Expected ~90 deg, got {bearing}"
        );
    }

    #[test]
    fn test_watson_watt_south() {
        let (n, e, s) = watson_watt_signals(180.0, 256);
        let bearing = watson_watt_bearing(&n, &e, &s);
        assert!(
            bearing_error(180.0, bearing) < 5.0,
            "Expected ~180 deg, got {bearing}"
        );
    }

    #[test]
    fn test_watson_watt_west() {
        let (n, e, s) = watson_watt_signals(270.0, 256);
        let bearing = watson_watt_bearing(&n, &e, &s);
        assert!(
            bearing_error(270.0, bearing) < 5.0,
            "Expected ~270 deg, got {bearing}"
        );
    }

    #[test]
    fn test_watson_watt_northeast() {
        let (n, e, s) = watson_watt_signals(45.0, 256);
        let bearing = watson_watt_bearing(&n, &e, &s);
        assert!(
            bearing_error(45.0, bearing) < 5.0,
            "Expected ~45 deg, got {bearing}"
        );
    }

    #[test]
    fn test_watson_watt_southeast() {
        let (n, e, s) = watson_watt_signals(135.0, 256);
        let bearing = watson_watt_bearing(&n, &e, &s);
        assert!(
            bearing_error(135.0, bearing) < 5.0,
            "Expected ~135 deg, got {bearing}"
        );
    }

    #[test]
    fn test_watson_watt_southwest() {
        let (n, e, s) = watson_watt_signals(225.0, 256);
        let bearing = watson_watt_bearing(&n, &e, &s);
        assert!(
            bearing_error(225.0, bearing) < 5.0,
            "Expected ~225 deg, got {bearing}"
        );
    }

    #[test]
    fn test_watson_watt_northwest() {
        let (n, e, s) = watson_watt_signals(315.0, 256);
        let bearing = watson_watt_bearing(&n, &e, &s);
        assert!(
            bearing_error(315.0, bearing) < 5.0,
            "Expected ~315 deg, got {bearing}"
        );
    }

    // -----------------------------------------------------------------------
    // Correlative interferometer tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_correlative_broadside() {
        // Signal arriving from broadside (0 deg) => zero phase difference
        let phases = vec![0.0, 0.0, 0.0];
        let bearing = correlative_interferometer(&phases, 0.5);
        assert!(
            bearing_error(0.0, bearing) < 1.0,
            "Expected ~0 deg, got {bearing}"
        );
    }

    #[test]
    fn test_correlative_30_degrees() {
        // phase = 2*pi*d*sin(30) = 2*pi*0.5*0.5 = pi/2
        let expected_phase = 2.0 * PI * 0.5 * (30.0_f64.to_radians().sin());
        let phases = vec![expected_phase; 3];
        let bearing = correlative_interferometer(&phases, 0.5);
        assert!(
            bearing_error(30.0, bearing) < 2.0,
            "Expected ~30 deg, got {bearing}"
        );
    }

    #[test]
    fn test_correlative_negative_angle() {
        // Signal from -45 deg => negative phase
        let expected_phase = 2.0 * PI * 0.5 * ((-45.0_f64).to_radians().sin());
        let phases = vec![expected_phase; 2];
        let bearing = correlative_interferometer(&phases, 0.5);
        // -45 deg normalized = 315 deg
        assert!(
            bearing_error(315.0, bearing) < 2.0,
            "Expected ~315 deg, got {bearing}"
        );
    }

    #[test]
    fn test_correlative_empty_phases() {
        let bearing = correlative_interferometer(&[], 0.5);
        assert!((bearing - 0.0).abs() < 0.01);
    }

    // -----------------------------------------------------------------------
    // Amplitude comparison tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_amplitude_north_peak() {
        // 4 antennas at 0, 90, 180, 270 deg. Strongest signal on antenna 0 (north)
        let amplitudes = vec![1.0, 0.1, 0.05, 0.1];
        let patterns = vec![0.0, 90.0, 180.0, 270.0];
        let bearing = amplitude_comparison(&amplitudes, &patterns);
        assert!(
            bearing_error(0.0, bearing) < 15.0,
            "Expected ~0 deg, got {bearing}"
        );
    }

    #[test]
    fn test_amplitude_east_peak() {
        let amplitudes = vec![0.1, 1.0, 0.1, 0.05];
        let patterns = vec![0.0, 90.0, 180.0, 270.0];
        let bearing = amplitude_comparison(&amplitudes, &patterns);
        assert!(
            bearing_error(90.0, bearing) < 15.0,
            "Expected ~90 deg, got {bearing}"
        );
    }

    #[test]
    fn test_amplitude_empty() {
        let bearing = amplitude_comparison(&[], &[]);
        assert!((bearing - 0.0).abs() < 0.01);
    }

    #[test]
    fn test_amplitude_mismatched_lengths() {
        let bearing = amplitude_comparison(&[1.0, 2.0], &[0.0]);
        assert!((bearing - 0.0).abs() < 0.01);
    }

    // -----------------------------------------------------------------------
    // MUSIC tests
    // -----------------------------------------------------------------------

    /// Generate ULA signals for a single source at a known bearing.
    fn ula_signals(
        bearing_deg: f64,
        num_antennas: usize,
        spacing_wl: f64,
        n_samples: usize,
        snr_linear: f64,
    ) -> Vec<Vec<(f64, f64)>> {
        let sv = steering_vector(bearing_deg, num_antennas, spacing_wl);
        let freq = 50.0;
        let sr = 1000.0;

        // Simple deterministic signal + no noise for high SNR
        let mut signals = vec![vec![(0.0, 0.0); n_samples]; num_antennas];
        // Use a simple PRNG for noise (LCG)
        let mut rng_state: u64 = 42;
        let next_gauss = |state: &mut u64| -> f64 {
            // Box-Muller from LCG
            *state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u1 = (*state >> 33) as f64 / (1u64 << 31) as f64;
            *state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u2 = (*state >> 33) as f64 / (1u64 << 31) as f64;
            let u1 = if u1 < 1e-10 { 1e-10 } else { u1 };
            (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
        };

        let noise_amp = 1.0 / snr_linear.sqrt();

        for t in 0..n_samples {
            let phase = 2.0 * PI * freq * t as f64 / sr;
            let sig = cexp_j(phase);
            for k in 0..num_antennas {
                let s = cmul(sv[k], sig);
                let nr = noise_amp * next_gauss(&mut rng_state);
                let ni = noise_amp * next_gauss(&mut rng_state);
                signals[k][t] = cadd(s, (nr, ni));
            }
        }

        signals
    }

    #[test]
    fn test_music_single_source_broadside() {
        let signals = ula_signals(0.0, 8, 0.5, 512, 100.0);
        let cov = compute_covariance_matrix(&signals);
        let bearings = music_bearing(&cov, 1, 0.5);
        assert_eq!(bearings.len(), 1);
        assert!(
            bearing_error(0.0, bearings[0]) < 5.0,
            "Expected ~0 deg, got {}",
            bearings[0]
        );
    }

    #[test]
    fn test_music_single_source_30deg() {
        let signals = ula_signals(30.0, 8, 0.5, 512, 100.0);
        let cov = compute_covariance_matrix(&signals);
        let bearings = music_bearing(&cov, 1, 0.5);
        assert_eq!(bearings.len(), 1);
        assert!(
            bearing_error(30.0, bearings[0]) < 5.0,
            "Expected ~30 deg, got {}",
            bearings[0]
        );
    }

    #[test]
    fn test_music_two_sources() {
        // Two sources at -20 and +40 degrees with uncorrelated signals and
        // a small amount of noise to break coherence (coherent sources are a
        // well-known MUSIC limitation without spatial smoothing).
        let n = 8;
        let spacing = 0.5;
        let n_samples = 2048;
        let sv1 = steering_vector(-20.0, n, spacing);
        let sv2 = steering_vector(40.0, n, spacing);
        let sr = 1000.0;

        // Simple LCG for deterministic noise
        let mut rng_state: u64 = 12345;
        let next_val = |state: &mut u64| -> f64 {
            *state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u1 = (*state >> 33) as f64 / (1u64 << 31) as f64;
            *state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u2 = (*state >> 33) as f64 / (1u64 << 31) as f64;
            let u1 = if u1 < 1e-10 { 1e-10 } else { u1 };
            (-2.0 * u1.ln()).sqrt() * (2.0 * PI * u2).cos()
        };

        let mut signals = vec![vec![(0.0, 0.0); n_samples]; n];
        for t in 0..n_samples {
            // Use different, non-harmonically-related frequencies
            let phase1 = 2.0 * PI * 31.7 * t as f64 / sr;
            let phase2 = 2.0 * PI * 73.3 * t as f64 / sr;
            let s1 = cexp_j(phase1);
            let s2 = cexp_j(phase2);
            for k in 0..n {
                let contrib = cadd(cmul(sv1[k], s1), cmul(sv2[k], s2));
                // Add small noise to regularise the covariance matrix
                let nr = 0.05 * next_val(&mut rng_state);
                let ni = 0.05 * next_val(&mut rng_state);
                signals[k][t] = cadd(contrib, (nr, ni));
            }
        }

        let cov = compute_covariance_matrix(&signals);
        let bearings = music_bearing(&cov, 2, spacing);
        assert_eq!(bearings.len(), 2);

        // Check both sources are found (order may vary).
        // Allow 15-degree tolerance for the simplified QR eigen solver.
        let mut found_m20 = false;
        let mut found_40 = false;
        for &b in &bearings {
            if bearing_error(340.0, b) < 15.0 {
                // -20 normalized = 340
                found_m20 = true;
            }
            if bearing_error(40.0, b) < 15.0 {
                found_40 = true;
            }
        }
        assert!(
            found_m20,
            "Did not find source at -20 deg; bearings = {:?}",
            bearings
        );
        assert!(
            found_40,
            "Did not find source at +40 deg; bearings = {:?}",
            bearings
        );
    }

    #[test]
    fn test_music_num_signals_too_large() {
        // num_signals >= num_antennas should return empty
        let cov = vec![vec![(1.0, 0.0); 4]; 4];
        let bearings = music_bearing(&cov, 4, 0.5);
        assert!(bearings.is_empty());
    }

    #[test]
    fn test_music_empty_covariance() {
        let bearings = music_bearing(&[], 1, 0.5);
        assert!(bearings.is_empty());
    }

    // -----------------------------------------------------------------------
    // Steering vector tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_steering_vector_broadside() {
        // At broadside (0 deg), all elements should have phase 0
        let sv = steering_vector(0.0, 4, 0.5);
        assert_eq!(sv.len(), 4);
        for &(re, im) in &sv {
            assert!((re - 1.0).abs() < 1e-10, "re = {re}");
            assert!(im.abs() < 1e-10, "im = {im}");
        }
    }

    #[test]
    fn test_steering_vector_endfire() {
        // At 90 deg (endfire), phase = -2*pi*d*sin(90) = -2*pi*d per element
        let sv = steering_vector(90.0, 4, 0.5);
        let expected_phase_inc = -2.0 * PI * 0.5; // = -pi
        for (k, &(re, im)) in sv.iter().enumerate() {
            let expected = cexp_j(expected_phase_inc * k as f64);
            assert!(
                (re - expected.0).abs() < 1e-10 && (im - expected.1).abs() < 1e-10,
                "Element {k}: got ({re}, {im}), expected ({}, {})",
                expected.0,
                expected.1
            );
        }
    }

    #[test]
    fn test_steering_vector_unit_norm() {
        let sv = steering_vector(30.0, 6, 0.5);
        // Each element should have magnitude 1
        for &s in &sv {
            let mag = cabs(s);
            assert!(
                (mag - 1.0).abs() < 1e-10,
                "Expected unit magnitude, got {mag}"
            );
        }
    }

    #[test]
    fn test_steering_vector_single_element() {
        let sv = steering_vector(45.0, 1, 0.5);
        assert_eq!(sv.len(), 1);
        assert!((sv[0].0 - 1.0).abs() < 1e-10);
        assert!(sv[0].1.abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Covariance matrix tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_covariance_hermitian() {
        // The covariance matrix should be Hermitian: R[i][j] = conj(R[j][i])
        let signals = ula_signals(20.0, 4, 0.5, 128, 10.0);
        let cov = compute_covariance_matrix(&signals);
        for i in 0..4 {
            for j in 0..4 {
                let diff_re = (cov[i][j].0 - cov[j][i].0).abs();
                let diff_im = (cov[i][j].1 + cov[j][i].1).abs();
                assert!(
                    diff_re < 1e-10 && diff_im < 1e-10,
                    "Not Hermitian at [{i}][{j}]: ({}, {}) vs conj({}, {})",
                    cov[i][j].0,
                    cov[i][j].1,
                    cov[j][i].0,
                    cov[j][i].1
                );
            }
        }
    }

    #[test]
    fn test_covariance_diagonal_positive() {
        // Diagonal elements should be real and non-negative
        let signals = ula_signals(0.0, 4, 0.5, 256, 5.0);
        let cov = compute_covariance_matrix(&signals);
        for i in 0..4 {
            assert!(
                cov[i][i].0 >= 0.0,
                "Diagonal [{i}] should be non-negative: {}",
                cov[i][i].0
            );
            assert!(
                cov[i][i].1.abs() < 1e-10,
                "Diagonal [{i}] imaginary should be ~0: {}",
                cov[i][i].1
            );
        }
    }

    #[test]
    fn test_covariance_empty_signals() {
        let cov = compute_covariance_matrix(&[]);
        assert!(cov.is_empty());
    }

    #[test]
    fn test_covariance_single_sample() {
        // With a single sample, R = x * x^H
        let signals = vec![vec![(1.0, 0.0)], vec![(0.0, 1.0)]];
        let cov = compute_covariance_matrix(&signals);
        // R[0][0] = 1*1 = 1
        assert!((cov[0][0].0 - 1.0).abs() < 1e-10);
        // R[0][1] = (1+0j)*(0-1j) = (0, -1) * conj => wait: (1,0)*conj(0,1)=(1,0)*(0,-1)=(0,-1)
        assert!((cov[0][1].0 - 0.0).abs() < 1e-10);
        assert!((cov[0][1].1 - (-1.0)).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Bearing error tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_bearing_error_same() {
        assert!((bearing_error(90.0, 90.0)).abs() < 1e-10);
    }

    #[test]
    fn test_bearing_error_wrapping() {
        // 350 to 10 should be 20 degrees, not 340
        assert!((bearing_error(350.0, 10.0) - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_bearing_error_negative_wrap() {
        assert!((bearing_error(10.0, 350.0) - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_bearing_error_180() {
        assert!((bearing_error(0.0, 180.0) - 180.0).abs() < 1e-10);
    }

    #[test]
    fn test_bearing_error_symmetry() {
        let e1 = bearing_error(30.0, 50.0);
        let e2 = bearing_error(50.0, 30.0);
        assert!((e1 - e2).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Circular mean bearing tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_circular_mean_simple() {
        let mean = circular_mean_bearing(&[10.0, 20.0, 30.0]);
        assert!(
            bearing_error(20.0, mean) < 1.0,
            "Expected ~20 deg, got {mean}"
        );
    }

    #[test]
    fn test_circular_mean_wrapping() {
        // 350 and 10 should average to 0/360
        let mean = circular_mean_bearing(&[350.0, 10.0]);
        assert!(
            bearing_error(0.0, mean) < 1.0,
            "Expected ~0 deg, got {mean}"
        );
    }

    #[test]
    fn test_circular_mean_opposite() {
        // 0 and 180 are opposite; result depends on atan2 but should be
        // either 0 or 180 or 90 depending on exact numerics. Just check
        // that it returns a valid bearing.
        let mean = circular_mean_bearing(&[0.0, 180.0]);
        assert!(mean >= 0.0 && mean < 360.0);
    }

    #[test]
    fn test_circular_mean_empty() {
        assert!((circular_mean_bearing(&[]) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_circular_mean_single() {
        let mean = circular_mean_bearing(&[123.0]);
        assert!(
            bearing_error(123.0, mean) < 0.01,
            "Expected 123 deg, got {mean}"
        );
    }

    // -----------------------------------------------------------------------
    // RdfProcessor dispatch tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_processor_watson_watt() {
        let config = RdfConfig {
            method: RdfMethod::WatsonWatt,
            num_antennas: 3,
            antenna_spacing_m: 0.15,
            center_freq_hz: 1e9,
            sample_rate_hz: 48000.0,
        };
        let proc = RdfProcessor::new(config);
        let (n, e, s) = watson_watt_signals(90.0, 256);
        let est = proc.estimate_bearing(&[n, e, s]);
        assert!(
            bearing_error(90.0, est.bearing_deg) < 5.0,
            "Expected ~90 deg, got {}",
            est.bearing_deg
        );
        assert!(!est.ambiguity);
    }

    #[test]
    fn test_processor_watson_watt_too_few_channels() {
        let config = RdfConfig {
            method: RdfMethod::WatsonWatt,
            num_antennas: 3,
            antenna_spacing_m: 0.15,
            center_freq_hz: 1e9,
            sample_rate_hz: 48000.0,
        };
        let proc = RdfProcessor::new(config);
        let est = proc.estimate_bearing(&[vec![(1.0, 0.0)]]);
        assert!(est.ambiguity);
        assert!((est.confidence - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_processor_music() {
        let config = RdfConfig {
            method: RdfMethod::Music,
            num_antennas: 8,
            antenna_spacing_m: 0.15,
            center_freq_hz: 1e9,
            sample_rate_hz: 48000.0,
        };
        let proc = RdfProcessor::new(config);
        let wavelength = speed_of_light() / 1e9;
        let spacing_wl = 0.15 / wavelength;
        let signals = ula_signals(30.0, 8, spacing_wl, 512, 100.0);
        let est = proc.estimate_bearing(&signals);
        assert!(
            bearing_error(30.0, est.bearing_deg) < 10.0,
            "Expected ~30 deg, got {}",
            est.bearing_deg
        );
    }

    #[test]
    fn test_processor_amplitude() {
        let config = RdfConfig {
            method: RdfMethod::Amplitude,
            num_antennas: 4,
            antenna_spacing_m: 0.15,
            center_freq_hz: 1e9,
            sample_rate_hz: 48000.0,
        };
        let proc = RdfProcessor::new(config);
        // 4 antennas: strongest on index 1 (pointing at 90 deg)
        let signals: Vec<Vec<(f64, f64)>> = vec![
            tone(100.0, 48000.0, 256, 0.1),
            tone(100.0, 48000.0, 256, 1.0),
            tone(100.0, 48000.0, 256, 0.05),
            tone(100.0, 48000.0, 256, 0.1),
        ];
        let est = proc.estimate_bearing(&signals);
        assert!(
            bearing_error(90.0, est.bearing_deg) < 20.0,
            "Expected ~90 deg, got {}",
            est.bearing_deg
        );
    }

    // -----------------------------------------------------------------------
    // Normalize bearing tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_normalize_bearing_positive() {
        assert!((normalize_bearing(370.0) - 10.0).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_bearing_negative() {
        assert!((normalize_bearing(-10.0) - 350.0).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_bearing_zero() {
        assert!((normalize_bearing(0.0) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_bearing_360() {
        assert!((normalize_bearing(360.0) - 0.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Edge case / misc tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_rms_empty() {
        assert!((rms_amplitude(&[]) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_rms_unit_signal() {
        let sig: Vec<(f64, f64)> = vec![(1.0, 0.0); 100];
        assert!((rms_amplitude(&sig) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_phase_method() {
        let config = RdfConfig {
            method: RdfMethod::Phase,
            num_antennas: 2,
            antenna_spacing_m: 0.15,
            center_freq_hz: 1e9,
            sample_rate_hz: 48000.0,
        };
        let proc = RdfProcessor::new(config);
        // Two antennas with known phase shift
        let n_samples = 256;
        let wavelength = speed_of_light() / 1e9;
        let d_wl = 0.15 / wavelength;
        let target_bearing_deg = 20.0;
        let phase_diff =
            2.0 * PI * d_wl * (target_bearing_deg as f64).to_radians().sin();
        let freq = 100.0;
        let sr = 48000.0;
        let sig0 = tone(freq, sr, n_samples, 1.0);
        let sig1: Vec<(f64, f64)> = sig0
            .iter()
            .map(|&s| cmul(s, cexp_j(phase_diff)))
            .collect();
        let est = proc.estimate_bearing(&[sig0, sig1]);
        assert!(
            bearing_error(target_bearing_deg, est.bearing_deg) < 5.0,
            "Expected ~{target_bearing_deg} deg, got {}",
            est.bearing_deg
        );
    }

    #[test]
    fn test_rdf_method_clone() {
        let m = RdfMethod::Music;
        let m2 = m;
        assert_eq!(m, m2);
    }
}
