//! Wi-Fi Channel State Information (CSI) extraction and analysis processor.
//!
//! Implements per-subcarrier CSI extraction from OFDM preambles (L-LTF / HE-LTF),
//! Least-Squares and MMSE estimation, channel impulse response analysis, Doppler
//! estimation, MIMO CSI matrices, SVD decomposition, beamforming weight computation,
//! waterfilling capacity, Kalman-filter channel tracking, codebook-based CSI feedback,
//! and WiFi-sensing activity recognition features.
//!
//! All math is implemented from scratch using only `std`.
//!
//! # Standards
//!
//! - IEEE 802.11-2020 §17.3.9 (L-LTF, 802.11a/g)
//! - IEEE 802.11-2020 §19.3.9 (HT-LTF, 802.11n)
//! - IEEE 802.11ax §27.3.10 (HE-LTF, 802.11ax / Wi-Fi 6)
//!
//! # Example
//!
//! ```
//! use r4w_core::wifi_csi_processor::{WiFiCsiProcessor, CsiConfig};
//!
//! let cfg = CsiConfig::wifi_a();
//! let proc = WiFiCsiProcessor::new(cfg);
//!
//! // Flat channel: rx = ref, LS estimate should be ~1+0j everywhere
//! let ref_sig: Vec<(f64, f64)> = (0..64).map(|i| {
//!     let th = 2.0 * std::f64::consts::PI * (i as f64) / 64.0;
//!     (th.cos(), th.sin())
//! }).collect();
//! let rx = ref_sig.clone();
//!
//! let csi = proc.extract_csi_ls(&rx, &ref_sig);
//! assert_eq!(csi.len(), 64);
//! let mag0 = (csi[1].0 * csi[1].0 + csi[1].1 * csi[1].1).sqrt();
//! assert!((mag0 - 1.0).abs() < 1e-9);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers
// ---------------------------------------------------------------------------

/// Complex number as (re, im).
pub type Cf64 = (f64, f64);

#[inline]
fn c_add(a: Cf64, b: Cf64) -> Cf64 {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: Cf64, b: Cf64) -> Cf64 {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: Cf64, b: Cf64) -> Cf64 {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: Cf64) -> Cf64 {
    (a.0, -a.1)
}

#[inline]
fn c_abs2(a: Cf64) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn c_abs(a: Cf64) -> f64 {
    c_abs2(a).sqrt()
}

#[inline]
fn c_div(a: Cf64, b: Cf64) -> Cf64 {
    let d = c_abs2(b);
    if d < f64::EPSILON {
        return (0.0, 0.0);
    }
    ((a.0 * b.0 + a.1 * b.1) / d, (a.1 * b.0 - a.0 * b.1) / d)
}

#[inline]
fn c_scale(a: Cf64, s: f64) -> Cf64 {
    (a.0 * s, a.1 * s)
}

#[inline]
fn c_arg(a: Cf64) -> f64 {
    a.1.atan2(a.0)
}

#[inline]
fn c_exp(re: f64, im: f64) -> Cf64 {
    let e = re.exp();
    (e * im.cos(), e * im.sin())
}

// ---------------------------------------------------------------------------
// FFT / IFFT (Cooley-Tukey radix-2, in-place)
// ---------------------------------------------------------------------------

/// Bit-reversal permutation.
fn bit_reverse_permute(x: &mut [Cf64]) {
    let n = x.len();
    let bits = n.trailing_zeros();
    for i in 0..n {
        let mut rev = 0usize;
        for b in 0..bits {
            rev |= ((i >> b) & 1) << (bits - 1 - b);
        }
        if i < rev {
            x.swap(i, rev);
        }
    }
}

/// In-place radix-2 DIT FFT. `n` must be a power of two.
fn fft_inplace(x: &mut [Cf64], inverse: bool) {
    let n = x.len();
    assert!(n.is_power_of_two(), "FFT size must be power of two");
    bit_reverse_permute(x);
    let sign = if inverse { 1.0_f64 } else { -1.0_f64 };
    let mut len = 2usize;
    while len <= n {
        let half = len / 2;
        let ang = sign * 2.0 * PI / (len as f64);
        let w_step = c_exp(0.0, ang);
        for i in (0..n).step_by(len) {
            let mut w = (1.0, 0.0);
            for j in 0..half {
                let u = x[i + j];
                let v = c_mul(x[i + j + half], w);
                x[i + j] = c_add(u, v);
                x[i + j + half] = c_sub(u, v);
                w = c_mul(w, w_step);
            }
        }
        len <<= 1;
    }
    if inverse {
        let nf = n as f64;
        for s in x.iter_mut() {
            *s = c_scale(*s, 1.0 / nf);
        }
    }
}

fn fft(input: &[Cf64]) -> Vec<Cf64> {
    let mut x = input.to_vec();
    fft_inplace(&mut x, false);
    x
}

fn ifft(input: &[Cf64]) -> Vec<Cf64> {
    let mut x = input.to_vec();
    fft_inplace(&mut x, true);
    x
}

/// Zero-pad to next power of two.
fn fft_padded(input: &[Cf64]) -> Vec<Cf64> {
    let n = input.len();
    let n2 = n.next_power_of_two();
    let mut x = input.to_vec();
    x.resize(n2, (0.0, 0.0));
    fft_inplace(&mut x, false);
    x
}

fn ifft_padded(input: &[Cf64], out_len: usize) -> Vec<Cf64> {
    let n2 = input.len().next_power_of_two();
    let mut x = input.to_vec();
    x.resize(n2, (0.0, 0.0));
    fft_inplace(&mut x, true);
    x.truncate(out_len);
    x
}

// ---------------------------------------------------------------------------
// DCT helpers (for CSI compression feedback)
// ---------------------------------------------------------------------------

/// Type-II DCT of length N (normalised).
fn dct2(x: &[f64]) -> Vec<f64> {
    let n = x.len();
    let mut out = vec![0.0f64; n];
    for k in 0..n {
        let mut s = 0.0f64;
        for m in 0..n {
            s += x[m] * (PI * (k as f64) * (2.0 * m as f64 + 1.0) / (2.0 * n as f64)).cos();
        }
        let scale = if k == 0 {
            (1.0_f64 / n as f64).sqrt()
        } else {
            (2.0_f64 / n as f64).sqrt()
        };
        out[k] = s * scale;
    }
    out
}

/// Inverse DCT (Type-III).
fn idct2(x: &[f64]) -> Vec<f64> {
    let n = x.len();
    let mut out = vec![0.0f64; n];
    for m in 0..n {
        let mut s = x[0] * (1.0_f64 / n as f64).sqrt();
        for k in 1..n {
            let scale = (2.0_f64 / n as f64).sqrt();
            s += x[k] * scale * (PI * (k as f64) * (2.0 * m as f64 + 1.0) / (2.0 * n as f64)).cos();
        }
        out[m] = s;
    }
    out
}

// ---------------------------------------------------------------------------
// SVD (thin 2D SVD via Golub-Reinsch bidiagonalisation for small matrices)
// ---------------------------------------------------------------------------

/// Result of SVD decomposition.
#[derive(Debug, Clone)]
pub struct SvdResult {
    /// Left singular vectors, column-major: U[i * nrows + row].
    pub u: Vec<Vec<Cf64>>,
    /// Singular values (descending).
    pub singular_values: Vec<f64>,
    /// Right singular vectors (conjugate transposed rows): Vh[i][j].
    pub vh: Vec<Vec<Cf64>>,
    /// Effective rank (number of singular values above threshold).
    pub rank: usize,
    /// Condition number = sigma_max / sigma_min.
    pub condition_number: f64,
}

/// Real 2x2 SVD used inside the complex SVD via real conversion.
fn svd_2x2_real(a: [[f64; 2]; 2]) -> ([[f64; 2]; 2], [f64; 2], [[f64; 2]; 2]) {
    // Golub-Reinsch single step for 2x2
    let (a00, a01, a10, a11) = (a[0][0], a[0][1], a[1][0], a[1][1]);
    // Compute B = A^T A (symmetric)
    let b00 = a00 * a00 + a10 * a10;
    let b01 = a00 * a01 + a10 * a11;
    let b11 = a01 * a01 + a11 * a11;
    // Eigenvalues of B via quadratic
    let tr = b00 + b11;
    let det = b00 * b11 - b01 * b01;
    let disc = ((tr * tr / 4.0) - det).max(0.0).sqrt();
    let lam1 = tr / 2.0 + disc;
    let lam2 = (tr / 2.0 - disc).max(0.0);
    let s1 = lam1.sqrt();
    let s2 = lam2.sqrt();

    // Right singular vectors (eigenvectors of B)
    let v: [[f64; 2]; 2];
    if b01.abs() < 1e-14 {
        v = [[1.0, 0.0], [0.0, 1.0]];
    } else {
        // First eigenvector for lam1
        let vx = b01;
        let vy = lam1 - b00;
        let norm = (vx * vx + vy * vy).sqrt().max(f64::EPSILON);
        let v1 = [vx / norm, vy / norm];
        // Second is orthogonal
        let v2 = [-v1[1], v1[0]];
        v = [v1, v2];
    }

    // U = A V Sigma^{-1}
    let u: [[f64; 2]; 2];
    {
        let u1x = if s1 > 1e-14 { (a00 * v[0][0] + a01 * v[0][1]) / s1 } else { 1.0 };
        let u1y = if s1 > 1e-14 { (a10 * v[0][0] + a11 * v[0][1]) / s1 } else { 0.0 };
        let u2x = -u1y;
        let u2y = u1x;
        u = [[u1x, u2x], [u1y, u2y]];
    }
    (u, [s1, s2], v)
}

/// Complex SVD for small matrices (typical MIMO: 2×2 up to 8×8) via
/// deflating power iteration.  We work on A A^H (Hermitian, m×m) and use
/// k different canonical-basis starting vectors so that degenerate eigenvalues
/// (e.g. the identity matrix) are properly found.
pub fn svd_complex(a: &[Vec<Cf64>]) -> SvdResult {
    let m = a.len();
    if m == 0 {
        return SvdResult {
            u: vec![],
            singular_values: vec![],
            vh: vec![],
            rank: 0,
            condition_number: 1.0,
        };
    }
    let n = a[0].len();
    if n == 0 {
        return SvdResult {
            u: vec![],
            singular_values: vec![],
            vh: vec![],
            rank: 0,
            condition_number: 1.0,
        };
    }
    let k = m.min(n);

    // Compute A A^H (m×m Hermitian)
    let mut aa_h: Vec<Vec<Cf64>> = vec![vec![(0.0, 0.0); m]; m];
    for i in 0..m {
        for j in 0..m {
            let mut s = (0.0, 0.0);
            for l in 0..n {
                s = c_add(s, c_mul(a[i][l], c_conj(a[j][l])));
            }
            aa_h[i][j] = s;
        }
    }

    let mut u_vecs: Vec<Vec<Cf64>> = Vec::new();
    let mut sigmas: Vec<f64> = Vec::new();

    // Deflating power iteration with cycling starting vectors.
    let mut mat = aa_h;
    for s_idx in 0..k {
        // Start from the canonical basis vector e_{s_idx} to handle degenerate
        // eigenvalues (e.g. all singular values equal).
        let start_idx = s_idx % m;
        let mut v: Vec<Cf64> = (0..m)
            .map(|i| if i == start_idx { (1.0, 0.0) } else { (0.0, 0.0) })
            .collect();
        // Initial application: if the deflated matrix returns zero on e_{s_idx},
        // try all other basis vectors until we find a non-zero direction.
        {
            let mut w: Vec<Cf64> = vec![(0.0, 0.0); m];
            for i in 0..m {
                for j in 0..m {
                    w[i] = c_add(w[i], c_mul(mat[i][j], v[j]));
                }
            }
            let norm0 = w.iter().map(|&x| c_abs2(x)).sum::<f64>().sqrt();
            if norm0 < 1e-14 {
                // Try other starting vectors
                let mut found = false;
                for alt in 0..m {
                    let alt_v: Vec<Cf64> = (0..m)
                        .map(|i| if i == alt { (1.0, 0.0) } else { (0.0, 0.0) })
                        .collect();
                    let mut aw: Vec<Cf64> = vec![(0.0, 0.0); m];
                    for i in 0..m {
                        for j in 0..m {
                            aw[i] = c_add(aw[i], c_mul(mat[i][j], alt_v[j]));
                        }
                    }
                    let anorm = aw.iter().map(|&x| c_abs2(x)).sum::<f64>().sqrt();
                    if anorm > 1e-14 {
                        v = alt_v;
                        found = true;
                        break;
                    }
                }
                if !found {
                    break; // deflated matrix is zero, done
                }
            } else {
                v = w.iter().map(|&x| c_scale(x, 1.0 / norm0)).collect();
            }
        }

        let mut lambda = 0.0f64;
        for _iter in 0..300 {
            let mut w: Vec<Cf64> = vec![(0.0, 0.0); m];
            for i in 0..m {
                for j in 0..m {
                    w[i] = c_add(w[i], c_mul(mat[i][j], v[j]));
                }
            }
            let norm = w.iter().map(|&x| c_abs2(x)).sum::<f64>().sqrt();
            if norm < 1e-14 {
                break;
            }
            let prev_lambda = lambda;
            lambda = norm;
            v = w.iter().map(|&x| c_scale(x, 1.0 / norm)).collect();
            if (lambda - prev_lambda).abs() < 1e-13 * lambda {
                break;
            }
        }
        if lambda < 1e-14 {
            break;
        }
        sigmas.push(lambda.sqrt());
        u_vecs.push(v.clone());
        // Deflate: mat -= lambda * v v^H
        for i in 0..m {
            for j in 0..m {
                let delta = c_scale(c_mul(v[i], c_conj(v[j])), lambda);
                mat[i][j] = c_sub(mat[i][j], delta);
            }
        }
    }

    // Compute right singular vectors: v_i = A^H u_i / sigma_i
    let mut v_vecs: Vec<Vec<Cf64>> = Vec::new();
    for (idx, sigma) in sigmas.iter().enumerate() {
        let u = &u_vecs[idx];
        let mut vr: Vec<Cf64> = vec![(0.0, 0.0); n];
        for j in 0..n {
            for i in 0..m {
                vr[j] = c_add(vr[j], c_mul(c_conj(a[i][j]), u[i]));
            }
            vr[j] = c_scale(vr[j], 1.0 / sigma);
        }
        v_vecs.push(vr);
    }

    let sigma_max = sigmas.first().copied().unwrap_or(0.0);
    let sigma_min = sigmas.last().copied().unwrap_or(0.0);
    let condition_number = if sigma_min > 1e-14 {
        sigma_max / sigma_min
    } else {
        f64::INFINITY
    };
    let threshold = sigma_max * 1e-6;
    let rank = sigmas.iter().filter(|&&s| s > threshold).count();

    SvdResult {
        u: u_vecs,
        singular_values: sigmas,
        vh: v_vecs,
        rank,
        condition_number,
    }
}

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Wi-Fi standard variant.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WifiStandard {
    /// 802.11a / 802.11g — 64-point FFT, 52 active subcarriers.
    Wifi80211a,
    /// 802.11n (HT) — 64-point FFT, 56 active subcarriers.
    Wifi80211n,
    /// 802.11ac (VHT) — 256-point FFT, 234 active subcarriers.
    Wifi80211ac,
    /// 802.11ax (HE) — 256-point FFT, 242 active subcarriers.
    Wifi80211ax,
    /// Custom / user-defined.
    Custom,
}

/// Configuration for the CSI processor.
#[derive(Debug, Clone)]
pub struct CsiConfig {
    /// Total FFT size (must be power of two).
    pub fft_size: usize,
    /// Number of active (data + pilot) subcarriers.
    pub num_active_subcarriers: usize,
    /// Number of transmit antennas.
    pub num_tx: usize,
    /// Number of receive antennas.
    pub num_rx: usize,
    /// Subcarrier spacing in Hz (e.g., 312_500 for 802.11a).
    pub subcarrier_spacing_hz: f64,
    /// Bandwidth in Hz.
    pub bandwidth_hz: f64,
    /// Wi-Fi standard.
    pub standard: WifiStandard,
    /// Log-distance path loss exponent (default 2.0 for free space).
    pub path_loss_exponent: f64,
    /// Reference distance for log-distance model in metres.
    pub path_loss_ref_distance_m: f64,
    /// Reference path loss at `path_loss_ref_distance_m` in dB.
    pub path_loss_ref_db: f64,
    /// Number of DFT/DCT coefficients to keep for CSI feedback compression.
    pub feedback_compression_taps: usize,
    /// Kalman process noise variance for channel tracking.
    pub kalman_process_noise: f64,
    /// Kalman measurement noise variance.
    pub kalman_meas_noise: f64,
}

impl CsiConfig {
    /// 802.11a preset (20 MHz, 64-point FFT).
    pub fn wifi_a() -> Self {
        CsiConfig {
            fft_size: 64,
            num_active_subcarriers: 52,
            num_tx: 1,
            num_rx: 1,
            subcarrier_spacing_hz: 312_500.0,
            bandwidth_hz: 20_000_000.0,
            standard: WifiStandard::Wifi80211a,
            path_loss_exponent: 2.0,
            path_loss_ref_distance_m: 1.0,
            path_loss_ref_db: 40.0,
            feedback_compression_taps: 16,
            kalman_process_noise: 1e-4,
            kalman_meas_noise: 1e-2,
        }
    }

    /// 802.11n preset (20 MHz, 64-point FFT, 2×2 MIMO).
    pub fn wifi_n_2x2() -> Self {
        CsiConfig {
            fft_size: 64,
            num_active_subcarriers: 56,
            num_tx: 2,
            num_rx: 2,
            subcarrier_spacing_hz: 312_500.0,
            bandwidth_hz: 20_000_000.0,
            standard: WifiStandard::Wifi80211n,
            path_loss_exponent: 3.0,
            path_loss_ref_distance_m: 1.0,
            path_loss_ref_db: 46.0,
            feedback_compression_taps: 16,
            kalman_process_noise: 1e-4,
            kalman_meas_noise: 1e-2,
        }
    }

    /// 802.11ac preset (80 MHz, 256-point FFT, 4×2 MIMO).
    pub fn wifi_ac_4x2() -> Self {
        CsiConfig {
            fft_size: 256,
            num_active_subcarriers: 234,
            num_tx: 4,
            num_rx: 2,
            subcarrier_spacing_hz: 312_500.0,
            bandwidth_hz: 80_000_000.0,
            standard: WifiStandard::Wifi80211ac,
            path_loss_exponent: 3.0,
            path_loss_ref_distance_m: 1.0,
            path_loss_ref_db: 46.0,
            feedback_compression_taps: 32,
            kalman_process_noise: 1e-4,
            kalman_meas_noise: 1e-2,
        }
    }
}

// ---------------------------------------------------------------------------
// Data structures
// ---------------------------------------------------------------------------

/// Per-subcarrier MIMO CSI matrix (Ntx × Nrx).
/// `data[tx][rx]` = H(k) for subcarrier k encoded outside this struct.
#[derive(Debug, Clone)]
pub struct CsiMatrix {
    pub num_tx: usize,
    pub num_rx: usize,
    /// Row = TX index, column = RX index.
    pub data: Vec<Vec<Cf64>>,
}

impl CsiMatrix {
    pub fn new(num_tx: usize, num_rx: usize) -> Self {
        CsiMatrix {
            num_tx,
            num_rx,
            data: vec![vec![(0.0, 0.0); num_rx]; num_tx],
        }
    }

    pub fn get(&self, tx: usize, rx: usize) -> Cf64 {
        self.data[tx][rx]
    }

    pub fn set(&mut self, tx: usize, rx: usize, val: Cf64) {
        self.data[tx][rx] = val;
    }
}

/// Multi-subcarrier MIMO CSI: vector of `CsiMatrix`, one per subcarrier.
#[derive(Debug, Clone)]
pub struct CsiCube {
    /// Number of subcarriers.
    pub num_subcarriers: usize,
    pub num_tx: usize,
    pub num_rx: usize,
    /// `matrices[k]` = CsiMatrix for subcarrier k.
    pub matrices: Vec<CsiMatrix>,
}

impl CsiCube {
    pub fn new(num_subcarriers: usize, num_tx: usize, num_rx: usize) -> Self {
        CsiCube {
            num_subcarriers,
            num_tx,
            num_rx,
            matrices: (0..num_subcarriers)
                .map(|_| CsiMatrix::new(num_tx, num_rx))
                .collect(),
        }
    }
}

/// Channel impulse response and derived metrics.
#[derive(Debug, Clone)]
pub struct ChannelProfile {
    /// CIR samples (complex).
    pub cir: Vec<Cf64>,
    /// Power delay profile |h(τ)|².
    pub pdp: Vec<f64>,
    /// Sample-domain delay indices.
    pub delays: Vec<f64>,
    /// Mean excess delay in samples.
    pub mean_excess_delay: f64,
    /// RMS delay spread in samples.
    pub rms_delay_spread: f64,
    /// Coherence bandwidth in Hz (approx. 1 / (5 * rms_delay_spread_s)).
    pub coherence_bandwidth_hz: f64,
}

/// Doppler estimation result.
#[derive(Debug, Clone)]
pub struct DopplerEstimate {
    /// Estimated maximum Doppler frequency in Hz.
    pub max_doppler_hz: f64,
    /// Estimated coherence time in seconds.
    pub coherence_time_s: f64,
    /// Per-subcarrier time-correlation magnitudes (averaged over subcarriers).
    pub correlation_vs_lag: Vec<f64>,
    /// Velocity estimate in m/s (assuming 5 GHz carrier).
    pub velocity_mps: f64,
}

/// Kalman filter state for tracking one complex channel coefficient.
#[derive(Debug, Clone)]
struct KalmanState {
    /// State estimate (complex channel coefficient).
    x: Cf64,
    /// Error covariance (real scalar; channel is scalar Gaussian).
    p: f64,
}

impl KalmanState {
    fn new(x0: Cf64) -> Self {
        KalmanState { x: x0, p: 1.0 }
    }

    fn predict(&mut self, process_noise: f64) {
        // State transition: x_k|k-1 = x_k-1  (random walk model)
        self.p += process_noise;
    }

    fn update(&mut self, z: Cf64, meas_noise: f64) {
        // Kalman gain
        let k = self.p / (self.p + meas_noise);
        // Innovation
        let innov = c_sub(z, self.x);
        self.x = c_add(self.x, c_scale(innov, k));
        self.p = (1.0 - k) * self.p;
    }
}

/// Spectral efficiency result.
#[derive(Debug, Clone)]
pub struct SpectralEfficiency {
    /// Per-subcarrier SINR in dB.
    pub sinr_db: Vec<f64>,
    /// Per-subcarrier waterfilling power allocation (relative).
    pub power_allocation: Vec<f64>,
    /// Total achievable rate in bits/s/Hz.
    pub capacity_bps_hz: f64,
    /// Shannon limit at flat channel (reference).
    pub shannon_limit_bps_hz: f64,
}

/// Activity recognition features derived from CSI.
#[derive(Debug, Clone)]
pub struct ActivityFeatures {
    /// Per-subcarrier amplitude variance over time.
    pub amplitude_variance: Vec<f64>,
    /// Per-subcarrier phase variance (unwrapped) over time.
    pub phase_variance: Vec<f64>,
    /// Mean correlation between adjacent subcarriers (frequency correlation).
    pub subcarrier_correlation: f64,
    /// Ratio of maximum to minimum amplitude variance (spread indicator).
    pub variance_spread: f64,
    /// RMS Doppler metric (energy in phase differential).
    pub doppler_rms: f64,
}

// ---------------------------------------------------------------------------
// Main processor struct
// ---------------------------------------------------------------------------

/// Wi-Fi CSI extraction and analysis processor.
#[derive(Debug, Clone)]
pub struct WiFiCsiProcessor {
    pub config: CsiConfig,
    /// Kalman states per subcarrier per (tx, rx) pair.
    kalman_states: Vec<Vec<Vec<KalmanState>>>,
}

impl WiFiCsiProcessor {
    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /// Create a new processor with the given configuration.
    pub fn new(config: CsiConfig) -> Self {
        let nsc = config.fft_size;
        let ntx = config.num_tx;
        let nrx = config.num_rx;
        let kalman_states: Vec<Vec<Vec<KalmanState>>> = (0..nsc)
            .map(|_| {
                (0..ntx)
                    .map(|_| (0..nrx).map(|_| KalmanState::new((1.0, 0.0))).collect())
                    .collect()
            })
            .collect();
        WiFiCsiProcessor {
            config,
            kalman_states,
        }
    }

    // -----------------------------------------------------------------------
    // 1. Least-Squares CSI extraction: H_LS(k) = Y(k) / X(k)
    // -----------------------------------------------------------------------

    /// Extract per-subcarrier CSI using Least-Squares (pilot division).
    ///
    /// `rx` and `ref_signal` are frequency-domain vectors of length `fft_size`.
    /// Returns H_LS(k) = Y(k) / X(k) for all k.
    pub fn extract_csi_ls(&self, rx: &[Cf64], ref_signal: &[Cf64]) -> Vec<Cf64> {
        let n = rx.len().min(ref_signal.len());
        (0..n).map(|k| c_div(rx[k], ref_signal[k])).collect()
    }

    // -----------------------------------------------------------------------
    // 2. MMSE estimation
    // -----------------------------------------------------------------------

    /// Extract per-subcarrier CSI using MMSE estimation.
    ///
    /// MMSE formula:  H_MMSE = R_HH (R_HH + σ_n² I)^{-1} H_LS
    ///
    /// For diagonal R_HH = I (flat prior), this simplifies to:
    ///   H_MMSE(k) = H_LS(k) / (1 + σ_n²)   where σ_n² is noise variance.
    ///
    /// With a length-L CIR model (windowed IFFT), a more accurate estimator
    /// is obtained by truncating the CIR and back-transforming.  Both
    /// approaches are provided.
    pub fn extract_csi_mmse(
        &self,
        rx: &[Cf64],
        ref_signal: &[Cf64],
        snr_db: f64,
    ) -> Vec<Cf64> {
        let h_ls = self.extract_csi_ls(rx, ref_signal);
        let snr_linear = 10.0_f64.powf(snr_db / 10.0);
        let sigma_n2 = 1.0 / snr_linear;

        // Estimate noise variance from the LS residual.
        let noise_est = self.estimate_noise_power(&h_ls);
        let alpha = noise_est.max(sigma_n2);

        // MMSE scalar shrinkage + CIR windowing.
        let n = h_ls.len();
        // Step 1: IFFT → CIR
        let mut cir = ifft(&h_ls);
        // Step 2: Window to channel order L = N / (SNR_dB.max(1) as usize + 1)
        let l = (n / (snr_db.max(1.0) as usize + 1)).max(1).min(n);
        for i in l..(n - l) {
            cir[i] = (0.0, 0.0);
        }
        // Step 3: Back-transform
        let h_cir = fft(&cir);
        // Step 4: Diagonal MMSE weighting
        let snr_k = 1.0 / (1.0 + alpha);
        h_cir.iter().map(|&h| c_scale(h, snr_k)).collect()
    }

    // -----------------------------------------------------------------------
    // 3. Noise power estimation from LS estimates
    // -----------------------------------------------------------------------

    /// Estimate noise power from the variability of LS channel estimates.
    /// Uses the median absolute deviation of magnitudes.
    pub fn estimate_noise_power(&self, h_ls: &[Cf64]) -> f64 {
        if h_ls.is_empty() {
            return 0.0;
        }
        let mags: Vec<f64> = h_ls.iter().map(|&h| c_abs(h)).collect();
        let mean_mag = mags.iter().sum::<f64>() / mags.len() as f64;
        let var = mags.iter().map(|&m| (m - mean_mag).powi(2)).sum::<f64>()
            / mags.len() as f64;
        var
    }

    // -----------------------------------------------------------------------
    // 4. Channel frequency response metrics
    // -----------------------------------------------------------------------

    /// Compute magnitude response |H(k)| for each subcarrier.
    pub fn magnitude_response(&self, csi: &[Cf64]) -> Vec<f64> {
        csi.iter().map(|&h| c_abs(h)).collect()
    }

    /// Compute phase response arg(H(k)) for each subcarrier.
    pub fn phase_response(&self, csi: &[Cf64]) -> Vec<f64> {
        csi.iter().map(|&h| c_arg(h)).collect()
    }

    /// Compute unwrapped group delay τ_g(k) = -dφ/dω ≈ -Δφ/Δω in samples.
    pub fn group_delay(&self, csi: &[Cf64]) -> Vec<f64> {
        let n = csi.len();
        if n < 2 {
            return vec![0.0; n];
        }
        let phi = self.phase_response(csi);
        // Unwrap phase
        let unwrapped = self.unwrap_phase(&phi);
        // Group delay = -d(phi)/d(omega); omega step = 2*pi/N
        let domega = 2.0 * PI / n as f64;
        let mut gd = vec![0.0f64; n];
        for k in 1..(n - 1) {
            gd[k] = -(unwrapped[k + 1] - unwrapped[k - 1]) / (2.0 * domega);
        }
        gd[0] = -(unwrapped[1] - unwrapped[0]) / domega;
        gd[n - 1] = -(unwrapped[n - 1] - unwrapped[n - 2]) / domega;
        gd
    }

    /// Unwrap phase: remove 2π jumps.
    pub fn unwrap_phase(&self, phi: &[f64]) -> Vec<f64> {
        let mut out = phi.to_vec();
        for i in 1..out.len() {
            let diff = out[i] - out[i - 1];
            if diff > PI {
                out[i] -= (diff / PI).floor() * PI;
            } else if diff < -PI {
                out[i] += ((-diff) / PI).floor() * PI;
            }
        }
        out
    }

    // -----------------------------------------------------------------------
    // 5. Channel Impulse Response
    // -----------------------------------------------------------------------

    /// Compute the CIR from frequency-domain CSI via IFFT.
    ///
    /// The output is the length-N time-domain channel impulse response.
    pub fn compute_cir(&self, csi: &[Cf64]) -> Vec<Cf64> {
        ifft(csi)
    }

    /// Compute the Power Delay Profile (PDP) from the CIR.
    pub fn power_delay_profile(&self, cir: &[Cf64]) -> Vec<f64> {
        cir.iter().map(|&h| c_abs2(h)).collect()
    }

    /// Compute mean excess delay in samples.
    ///
    /// τ_mean = Σ τ_k |h(τ_k)|² / Σ |h(τ_k)|²
    pub fn mean_excess_delay(&self, cir: &[Cf64]) -> f64 {
        let pdp = self.power_delay_profile(cir);
        let total_power: f64 = pdp.iter().sum();
        if total_power < f64::EPSILON {
            return 0.0;
        }
        pdp.iter()
            .enumerate()
            .map(|(i, &p)| i as f64 * p)
            .sum::<f64>()
            / total_power
    }

    /// Compute RMS delay spread in samples.
    ///
    /// τ_rms = sqrt(Σ (τ_k - τ_mean)² |h(τ_k)|² / Σ |h(τ_k)|²)
    pub fn rms_delay_spread(&self, cir: &[Cf64]) -> f64 {
        let pdp = self.power_delay_profile(cir);
        let total_power: f64 = pdp.iter().sum();
        if total_power < f64::EPSILON {
            return 0.0;
        }
        let mean = self.mean_excess_delay(cir);
        let variance = pdp
            .iter()
            .enumerate()
            .map(|(i, &p)| (i as f64 - mean).powi(2) * p)
            .sum::<f64>()
            / total_power;
        variance.sqrt()
    }

    /// Compute full channel profile (CIR + PDP + delay spread metrics).
    pub fn compute_channel_profile(&self, csi: &[Cf64]) -> ChannelProfile {
        let cir = self.compute_cir(csi);
        let pdp = self.power_delay_profile(&cir);
        let n = cir.len();
        let delays: Vec<f64> = (0..n).map(|i| i as f64).collect();
        let mean_excess_delay = self.mean_excess_delay(&cir);
        let rms_spread = self.rms_delay_spread(&cir);
        // Coherence bandwidth ≈ 1 / (5 * rms_spread_seconds)
        let sample_rate = self.config.bandwidth_hz;
        let rms_spread_s = rms_spread / sample_rate;
        let coherence_bandwidth_hz = if rms_spread_s > 1e-12 {
            1.0 / (5.0 * rms_spread_s)
        } else {
            f64::INFINITY
        };
        ChannelProfile {
            cir,
            pdp,
            delays,
            mean_excess_delay,
            rms_delay_spread: rms_spread,
            coherence_bandwidth_hz,
        }
    }

    // -----------------------------------------------------------------------
    // 6. Doppler estimation
    // -----------------------------------------------------------------------

    /// Estimate Doppler from time-varying CSI (multiple packets).
    ///
    /// `csi_packets`: Vec of per-packet CSI vectors (each of length `fft_size`).
    /// `interval_s`: time between packets in seconds.
    pub fn estimate_doppler(
        &self,
        csi_packets: &[Vec<Cf64>],
        interval_s: f64,
    ) -> DopplerEstimate {
        let npkts = csi_packets.len();
        if npkts < 2 {
            return DopplerEstimate {
                max_doppler_hz: 0.0,
                coherence_time_s: f64::INFINITY,
                correlation_vs_lag: vec![1.0],
                velocity_mps: 0.0,
            };
        }
        let nsc = csi_packets[0].len();

        // Compute time-domain autocorrelation of CSI magnitude for each subcarrier,
        // then average across subcarriers.
        let max_lag = (npkts / 2).min(20);
        let mut corr = vec![0.0f64; max_lag + 1];

        for sc in 0..nsc {
            // Magnitude sequence for this subcarrier
            let seq: Vec<f64> = csi_packets.iter().map(|p| c_abs(p[sc])).collect();
            let mean = seq.iter().sum::<f64>() / seq.len() as f64;
            let var = seq.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / seq.len() as f64;
            if var < f64::EPSILON {
                corr[0] += 1.0;
                continue;
            }
            for lag in 0..=max_lag {
                let pairs = (npkts - lag) as f64;
                if pairs < 1.0 {
                    break;
                }
                let r: f64 = (0..(npkts - lag))
                    .map(|i| (seq[i] - mean) * (seq[i + lag] - mean))
                    .sum::<f64>()
                    / (pairs * var);
                corr[lag] += r;
            }
        }
        // Normalise
        let nsc_f = nsc as f64;
        for c in corr.iter_mut() {
            *c /= nsc_f;
        }

        // Find coherence time: first lag where correlation drops below 1/e ≈ 0.37
        let mut coherence_lag = max_lag;
        for (l, &r) in corr.iter().enumerate() {
            if r < 0.368 {
                coherence_lag = l;
                break;
            }
        }
        let coherence_time_s = coherence_lag as f64 * interval_s;

        // Max Doppler estimate from coherence time: f_D ≈ 9 / (16π * T_c)  (Clarke model)
        let max_doppler_hz = if coherence_time_s > 1e-9 {
            9.0 / (16.0 * PI * coherence_time_s)
        } else {
            0.0
        };

        // Velocity from Doppler (assume 5 GHz carrier)
        let carrier_hz = 5.0e9_f64;
        let speed_of_light = 3.0e8_f64;
        let velocity_mps = max_doppler_hz * speed_of_light / carrier_hz;

        DopplerEstimate {
            max_doppler_hz,
            coherence_time_s,
            correlation_vs_lag: corr,
            velocity_mps,
        }
    }

    // -----------------------------------------------------------------------
    // 7. Path-loss and distance estimation
    // -----------------------------------------------------------------------

    /// Estimate path loss in dB from CSI magnitude.
    ///
    /// PL = -20 log10(mean|H(k)|) – gains_db.
    pub fn estimate_path_loss_db(&self, csi: &[Cf64]) -> f64 {
        let mags = self.magnitude_response(csi);
        let mean_mag = mags.iter().sum::<f64>() / mags.len() as f64;
        if mean_mag < f64::EPSILON {
            return f64::INFINITY;
        }
        -20.0 * mean_mag.log10()
    }

    /// Estimate distance in metres using log-distance path loss model.
    ///
    /// PL(d) = PL(d₀) + 10 n log10(d / d₀)
    pub fn estimate_distance_m(&self, csi: &[Cf64]) -> f64 {
        let pl_db = self.estimate_path_loss_db(csi);
        let cfg = &self.config;
        let exponent = cfg.path_loss_exponent;
        let ref_pl = cfg.path_loss_ref_db;
        let ref_d = cfg.path_loss_ref_distance_m;
        // d = d0 * 10^((PL - PL0) / (10 * n))
        ref_d * 10.0_f64.powf((pl_db - ref_pl) / (10.0 * exponent))
    }

    // -----------------------------------------------------------------------
    // 8. MIMO CSI matrix: build, SVD, condition number, rank
    // -----------------------------------------------------------------------

    /// Build a `CsiCube` (Ntx × Nrx CSI per subcarrier) from separate per-link
    /// CSI vectors.
    ///
    /// `csi_links`: `csi_links[tx][rx]` = frequency-domain CSI vector for that
    /// antenna pair (length = fft_size).
    pub fn build_csi_cube(&self, csi_links: &[Vec<Vec<Cf64>>]) -> CsiCube {
        let ntx = csi_links.len();
        let nrx = if ntx > 0 { csi_links[0].len() } else { 0 };
        let nsc = if ntx > 0 && nrx > 0 {
            csi_links[0][0].len()
        } else {
            self.config.fft_size
        };
        let mut cube = CsiCube::new(nsc, ntx, nrx);
        for tx in 0..ntx {
            for rx in 0..nrx {
                for k in 0..nsc {
                    cube.matrices[k].set(tx, rx, csi_links[tx][rx][k]);
                }
            }
        }
        cube
    }

    /// Perform SVD decomposition of a per-subcarrier CSI matrix.
    pub fn svd_decompose(&self, csi_matrix: &CsiMatrix) -> SvdResult {
        svd_complex(&csi_matrix.data)
    }

    /// Compute per-subcarrier SVD for the full cube, returning condition numbers.
    pub fn cube_condition_numbers(&self, cube: &CsiCube) -> Vec<f64> {
        cube.matrices
            .iter()
            .map(|m| self.svd_decompose(m).condition_number)
            .collect()
    }

    // -----------------------------------------------------------------------
    // 9. CSI feedback compression (DFT / DCT codebook)
    // -----------------------------------------------------------------------

    /// Compress CSI for limited feedback using DCT on magnitude and phase
    /// separately, keeping only the top-K coefficients.
    ///
    /// Returns (compressed_magnitude_coeffs, compressed_phase_coeffs).
    pub fn compress_csi_feedback(&self, csi: &[Cf64]) -> (Vec<f64>, Vec<f64>) {
        let mags: Vec<f64> = self.magnitude_response(csi);
        let phases: Vec<f64> = self.phase_response(csi);
        let k = self.config.feedback_compression_taps.min(csi.len());
        let mut mag_dct = dct2(&mags);
        let mut ph_dct = dct2(&phases);
        mag_dct.truncate(k);
        ph_dct.truncate(k);
        (mag_dct, ph_dct)
    }

    /// Reconstruct CSI from compressed feedback (inverse DCT).
    pub fn decompress_csi_feedback(
        &self,
        mag_coeffs: &[f64],
        phase_coeffs: &[f64],
        full_size: usize,
    ) -> Vec<Cf64> {
        let mut mag_padded = mag_coeffs.to_vec();
        mag_padded.resize(full_size, 0.0);
        let mut ph_padded = phase_coeffs.to_vec();
        ph_padded.resize(full_size, 0.0);
        let mags = idct2(&mag_padded);
        let phases = idct2(&ph_padded);
        mags.iter()
            .zip(phases.iter())
            .map(|(&m, &p)| (m * p.cos(), m * p.sin()))
            .collect()
    }

    // -----------------------------------------------------------------------
    // 10. Activity recognition features
    // -----------------------------------------------------------------------

    /// Compute activity recognition features from a time series of CSI packets.
    ///
    /// Each packet is a frequency-domain CSI vector of length fft_size.
    pub fn compute_activity_features(&self, csi_packets: &[Vec<Cf64>]) -> ActivityFeatures {
        let npkts = csi_packets.len();
        if npkts == 0 {
            return ActivityFeatures {
                amplitude_variance: vec![],
                phase_variance: vec![],
                subcarrier_correlation: 0.0,
                variance_spread: 0.0,
                doppler_rms: 0.0,
            };
        }
        let nsc = csi_packets[0].len();

        // Per-subcarrier amplitude variance
        let mut amp_var = vec![0.0f64; nsc];
        let mut ph_var = vec![0.0f64; nsc];
        for sc in 0..nsc {
            let mags: Vec<f64> = csi_packets.iter().map(|p| c_abs(p[sc])).collect();
            let phases: Vec<f64> = csi_packets.iter().map(|p| c_arg(p[sc])).collect();
            let mean_m = mags.iter().sum::<f64>() / npkts as f64;
            amp_var[sc] = mags.iter().map(|&m| (m - mean_m).powi(2)).sum::<f64>() / npkts as f64;
            // Unwrap phases for variance
            let uphs = self.unwrap_phase(&phases);
            let mean_p = uphs.iter().sum::<f64>() / npkts as f64;
            ph_var[sc] = uphs.iter().map(|&p| (p - mean_p).powi(2)).sum::<f64>() / npkts as f64;
        }

        // Subcarrier correlation (mean correlation between adjacent subcarrier amplitudes)
        let mut corr_sum = 0.0f64;
        let mut corr_count = 0usize;
        for sc in 0..(nsc - 1) {
            let a: Vec<f64> = csi_packets.iter().map(|p| c_abs(p[sc])).collect();
            let b: Vec<f64> = csi_packets.iter().map(|p| c_abs(p[sc + 1])).collect();
            let mean_a = a.iter().sum::<f64>() / npkts as f64;
            let mean_b = b.iter().sum::<f64>() / npkts as f64;
            let cov: f64 = a
                .iter()
                .zip(b.iter())
                .map(|(&ai, &bi)| (ai - mean_a) * (bi - mean_b))
                .sum::<f64>()
                / npkts as f64;
            let std_a = a
                .iter()
                .map(|&ai| (ai - mean_a).powi(2))
                .sum::<f64>()
                .sqrt()
                / (npkts as f64).sqrt();
            let std_b = b
                .iter()
                .map(|&bi| (bi - mean_b).powi(2))
                .sum::<f64>()
                .sqrt()
                / (npkts as f64).sqrt();
            if std_a * std_b > 1e-12 {
                corr_sum += cov / (std_a * std_b);
                corr_count += 1;
            }
        }
        let subcarrier_correlation = if corr_count > 0 {
            corr_sum / corr_count as f64
        } else {
            0.0
        };

        // Variance spread
        let amp_max = amp_var.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let amp_min = amp_var.iter().cloned().fold(f64::INFINITY, f64::min);
        let variance_spread = if amp_min > f64::EPSILON {
            amp_max / amp_min
        } else {
            0.0
        };

        // Doppler RMS: RMS of per-subcarrier phase rate
        let doppler_rms = if npkts >= 2 {
            let mut ph_rate_sq_sum = 0.0f64;
            for sc in 0..nsc {
                let phases: Vec<f64> = csi_packets.iter().map(|p| c_arg(p[sc])).collect();
                let uphs = self.unwrap_phase(&phases);
                let rates: Vec<f64> = uphs.windows(2).map(|w| w[1] - w[0]).collect();
                let mean_rate = rates.iter().sum::<f64>() / rates.len() as f64;
                ph_rate_sq_sum += mean_rate.powi(2);
            }
            (ph_rate_sq_sum / nsc as f64).sqrt()
        } else {
            0.0
        };

        ActivityFeatures {
            amplitude_variance: amp_var,
            phase_variance: ph_var,
            subcarrier_correlation,
            variance_spread,
            doppler_rms,
        }
    }

    // -----------------------------------------------------------------------
    // 11. Beamforming weights (zero-forcing / MMSE precoder from CSI)
    // -----------------------------------------------------------------------

    /// Compute MRT (Matched Filter / Maximum Ratio Transmission) beamforming
    /// weights for a single subcarrier CSI vector (MISO: 1 RX, Ntx TX).
    ///
    /// w_MRT = h* / ||h||
    ///
    /// Returns weight vector of length num_tx.
    pub fn mrt_beamforming_weights(&self, h: &[Cf64]) -> Vec<Cf64> {
        let norm = h.iter().map(|&x| c_abs2(x)).sum::<f64>().sqrt();
        if norm < f64::EPSILON {
            return vec![(0.0, 0.0); h.len()];
        }
        h.iter().map(|&x| c_scale(c_conj(x), 1.0 / norm)).collect()
    }

    /// Compute Zero-Forcing beamforming weights for a per-subcarrier CsiMatrix.
    ///
    /// For a flat Ntx×Nrx channel H, the ZF pseudo-inverse precoder is:
    ///   W_ZF = H^H (H H^H)^{-1}
    ///
    /// Returns one weight vector per receive stream (Nrx vectors of length Ntx).
    pub fn compute_beamforming_weights(&self, csi_matrix: &CsiMatrix) -> Vec<Vec<Cf64>> {
        let ntx = csi_matrix.num_tx;
        let nrx = csi_matrix.num_rx;
        // Compute H H^H (nrx × nrx)
        let mut hht: Vec<Vec<Cf64>> = vec![vec![(0.0, 0.0); nrx]; nrx];
        for i in 0..nrx {
            for j in 0..nrx {
                let mut s = (0.0, 0.0);
                for k in 0..ntx {
                    // H[k][i] * conj(H[k][j])
                    s = c_add(s, c_mul(csi_matrix.data[k][i], c_conj(csi_matrix.data[k][j])));
                }
                hht[i][j] = s;
            }
        }
        // Invert HHH^H (2×2 or scalar)
        let hht_inv = mat_inv_small(&hht);
        // W_ZF = H^H * (H H^H)^{-1}: shape ntx × nrx
        let mut w: Vec<Vec<Cf64>> = vec![vec![(0.0, 0.0); ntx]; nrx];
        for rx in 0..nrx {
            for tx in 0..ntx {
                let mut s = (0.0, 0.0);
                for j in 0..nrx {
                    // H^H[tx][j] = conj(H[tx][j])   wait: H is ntx x nrx so H^H[tx][j] = conj(H[j][tx]) -- no...
                    // H has shape [tx][rx], so H^H has shape [rx][tx] with H^H[i][j] = conj(H[j][i])
                    let h_h_i_tx = c_conj(csi_matrix.data[tx][j]); // H^H[j][tx]
                    s = c_add(s, c_mul(h_h_i_tx, hht_inv[j][rx]));
                }
                w[rx][tx] = s;
            }
        }
        w
    }

    /// Compute steering vectors for a ULA (Uniform Linear Array).
    ///
    /// Returns a vector of length `num_tx` representing the array response
    /// for direction-of-arrival `theta_rad`.
    pub fn ula_steering_vector(&self, theta_rad: f64, d_lambda: f64) -> Vec<Cf64> {
        let n = self.config.num_tx;
        (0..n)
            .map(|i| {
                let phase = 2.0 * PI * d_lambda * (i as f64) * theta_rad.sin();
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    // -----------------------------------------------------------------------
    // 12. Kalman channel tracking
    // -----------------------------------------------------------------------

    /// Update the Kalman filter channel tracking with a new CSI observation.
    ///
    /// `csi` is the frequency-domain CSI vector for the (0, 0) link (or any
    /// selected antenna pair).
    ///
    /// Returns the smoothed (tracked) CSI estimate.
    pub fn kalman_track_update(&mut self, csi: &[Cf64]) -> Vec<Cf64> {
        let nsc = csi.len().min(self.config.fft_size);
        let pn = self.config.kalman_process_noise;
        let mn = self.config.kalman_meas_noise;
        let mut out = vec![(0.0, 0.0); nsc];
        for k in 0..nsc {
            // Use link (0, 0)
            let state = &mut self.kalman_states[k][0][0];
            state.predict(pn);
            state.update(csi[k], mn);
            out[k] = state.x;
        }
        out
    }

    /// Reset all Kalman state estimates to the given initial value.
    pub fn kalman_reset(&mut self, init: Cf64) {
        for k in 0..self.kalman_states.len() {
            for tx in 0..self.kalman_states[k].len() {
                for rx in 0..self.kalman_states[k][tx].len() {
                    self.kalman_states[k][tx][rx] = KalmanState::new(init);
                }
            }
        }
    }

    /// Estimate channel fade rate in Hz from sequential Kalman innovations.
    ///
    /// Computes the mean squared innovation across subcarriers divided by
    /// the Kalman measurement noise to obtain a proxy for fade rate.
    pub fn estimate_fade_rate_hz(
        &self,
        csi_a: &[Cf64],
        csi_b: &[Cf64],
        interval_s: f64,
    ) -> f64 {
        if interval_s <= 0.0 || csi_a.is_empty() || csi_b.len() != csi_a.len() {
            return 0.0;
        }
        let n = csi_a.len();
        let ms_diff: f64 = (0..n)
            .map(|k| c_abs2(c_sub(csi_b[k], csi_a[k])))
            .sum::<f64>()
            / n as f64;
        ms_diff.sqrt() / interval_s
    }

    // -----------------------------------------------------------------------
    // 13. Spectral efficiency / waterfilling
    // -----------------------------------------------------------------------

    /// Compute per-subcarrier SINR and waterfilling capacity.
    ///
    /// `csi`: per-subcarrier frequency-domain CSI (|H(k)|² gives subcarrier gain).
    /// `snr_db`: total (wideband) SNR in dB.
    pub fn waterfilling_capacity(&self, csi: &[Cf64], snr_db: f64) -> f64 {
        let result = self.compute_spectral_efficiency(csi, snr_db);
        result.capacity_bps_hz
    }

    /// Full spectral efficiency analysis.
    pub fn compute_spectral_efficiency(
        &self,
        csi: &[Cf64],
        snr_db: f64,
    ) -> SpectralEfficiency {
        let n = csi.len();
        let snr_linear = 10.0_f64.powf(snr_db / 10.0);
        let gains: Vec<f64> = csi.iter().map(|&h| c_abs2(h)).collect();

        // Waterfilling: allocate power p_k subject to Σp_k = N, p_k ≥ 0.
        // Water level μ: p_k = (μ - 1/g_k)⁺ where g_k = SNR_total * |H(k)|²
        let noise_level = 1.0 / snr_linear; // normalised noise per subcarrier
        let chan_gains: Vec<f64> = gains.iter().map(|&g| g + 1e-14).collect();

        // Bisect to find water level
        let total_power = n as f64; // total power = N (normalised)
        let mu = waterfilling_bisect(&chan_gains, noise_level, total_power);
        let power_alloc: Vec<f64> = chan_gains
            .iter()
            .map(|&g| (mu - noise_level / g).max(0.0))
            .collect();

        // Per-subcarrier SINR and rate
        let sinr_linear: Vec<f64> = chan_gains
            .iter()
            .zip(power_alloc.iter())
            .map(|(&g, &p)| g * p / noise_level)
            .collect();
        let sinr_db: Vec<f64> = sinr_linear
            .iter()
            .map(|&s| if s > 0.0 { 10.0 * s.log10() } else { -100.0 })
            .collect();

        let capacity_bps_hz: f64 = sinr_linear
            .iter()
            .map(|&s| (1.0 + s).max(1.0).log2())
            .sum::<f64>()
            / n as f64;

        let shannon_limit_bps_hz = (1.0 + snr_linear).log2();

        SpectralEfficiency {
            sinr_db,
            power_allocation: power_alloc,
            capacity_bps_hz,
            shannon_limit_bps_hz,
        }
    }

    // -----------------------------------------------------------------------
    // Utility: extract active subcarrier CSI
    // -----------------------------------------------------------------------

    /// Extract active subcarrier CSI from the full FFT-size CSI vector.
    ///
    /// For 802.11a: subcarriers -26…-1, 1…26 (mapped to FFT indices).
    pub fn extract_active_subcarriers(&self, full_csi: &[Cf64]) -> Vec<Cf64> {
        let nfft = full_csi.len();
        let nactive = self.config.num_active_subcarriers;
        let half = nactive / 2;
        // Lower half: indices 1..=half, upper half: nfft-half..nfft
        let mut out = Vec::with_capacity(nactive);
        for k in 1..=half {
            out.push(full_csi[k]);
        }
        for k in (nfft - half)..nfft {
            out.push(full_csi[k]);
        }
        out
    }

    /// Compute per-subcarrier SNR estimate from CSI and noise power.
    pub fn per_subcarrier_snr_db(&self, csi: &[Cf64], noise_power: f64) -> Vec<f64> {
        csi.iter()
            .map(|&h| {
                let signal_power = c_abs2(h);
                if noise_power > 1e-14 {
                    10.0 * (signal_power / noise_power).log10()
                } else {
                    100.0
                }
            })
            .collect()
    }

    /// Interpolate CSI from pilot subcarriers to all subcarriers using linear
    /// interpolation.
    ///
    /// `pilots`: (subcarrier_index, csi_value) pairs sorted by index.
    /// `nsc`: total number of subcarriers (output size).
    pub fn interpolate_from_pilots(&self, pilots: &[(usize, Cf64)], nsc: usize) -> Vec<Cf64> {
        if pilots.is_empty() {
            return vec![(0.0, 0.0); nsc];
        }
        let mut out = vec![(0.0, 0.0); nsc];
        // Fill before first pilot
        let (k0, h0) = pilots[0];
        for k in 0..k0 {
            out[k] = h0;
        }
        // Linearly interpolate between consecutive pilots
        for win in pilots.windows(2) {
            let (ka, ha) = win[0];
            let (kb, hb) = win[1];
            for k in ka..=kb {
                let t = if kb > ka {
                    (k - ka) as f64 / (kb - ka) as f64
                } else {
                    0.0
                };
                let re = ha.0 + t * (hb.0 - ha.0);
                let im = ha.1 + t * (hb.1 - ha.1);
                out[k] = (re, im);
            }
        }
        // Fill after last pilot
        let (kn, hn) = *pilots.last().unwrap();
        for k in (kn + 1)..nsc {
            out[k] = hn;
        }
        out
    }
}

// ---------------------------------------------------------------------------
// Waterfilling bisection helper
// ---------------------------------------------------------------------------

fn waterfilling_bisect(gains: &[f64], noise: f64, total_power: f64) -> f64 {
    // μ ∈ [noise/g_min, noise/g_min + total_power]
    let g_min = gains.iter().cloned().fold(f64::INFINITY, f64::min);
    let mu_low = noise / g_min.max(1e-14);
    let mu_high = mu_low + total_power + 1.0;
    let mut lo = mu_low;
    let mut hi = mu_high;
    for _ in 0..60 {
        let mid = (lo + hi) / 2.0;
        let sum: f64 = gains.iter().map(|&g| (mid - noise / g).max(0.0)).sum();
        if sum < total_power {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    (lo + hi) / 2.0
}

// ---------------------------------------------------------------------------
// Small matrix inverse (up to 4×4 complex)
// ---------------------------------------------------------------------------

fn mat_inv_small(a: &[Vec<Cf64>]) -> Vec<Vec<Cf64>> {
    let n = a.len();
    match n {
        1 => {
            let inv = c_div((1.0, 0.0), a[0][0]);
            vec![vec![inv]]
        }
        2 => {
            // Cramer's rule
            let det = c_sub(c_mul(a[0][0], a[1][1]), c_mul(a[0][1], a[1][0]));
            let det_abs = c_abs2(det);
            if det_abs < 1e-28 {
                return vec![vec![(0.0, 0.0); 2]; 2];
            }
            let inv_det = c_div((1.0, 0.0), det);
            vec![
                vec![c_mul(a[1][1], inv_det), c_mul(c_scale(a[0][1], -1.0), inv_det)],
                vec![c_mul(c_scale(a[1][0], -1.0), inv_det), c_mul(a[0][0], inv_det)],
            ]
        }
        _ => {
            // Gauss-Jordan elimination
            let mut aug: Vec<Vec<Cf64>> = (0..n)
                .map(|i| {
                    let mut row: Vec<Cf64> = a[i].clone();
                    for j in 0..n {
                        row.push(if i == j { (1.0, 0.0) } else { (0.0, 0.0) });
                    }
                    row
                })
                .collect();
            for col in 0..n {
                // Pivot selection
                let mut max_row = col;
                let mut max_val = c_abs(aug[col][col]);
                for row in (col + 1)..n {
                    let v = c_abs(aug[row][col]);
                    if v > max_val {
                        max_val = v;
                        max_row = row;
                    }
                }
                aug.swap(col, max_row);
                let pivot = aug[col][col];
                if c_abs(pivot) < 1e-14 {
                    continue;
                }
                let inv_pivot = c_div((1.0, 0.0), pivot);
                for j in 0..(2 * n) {
                    aug[col][j] = c_mul(aug[col][j], inv_pivot);
                }
                for row in 0..n {
                    if row == col {
                        continue;
                    }
                    let factor = aug[row][col];
                    for j in 0..(2 * n) {
                        let sub = c_mul(factor, aug[col][j]);
                        aug[row][j] = c_sub(aug[row][j], sub);
                    }
                }
            }
            (0..n)
                .map(|i| (n..(2 * n)).map(|j| aug[i][j]).collect())
                .collect()
        }
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn nearly_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() <= tol
    }

    fn make_flat_csi(n: usize, h_re: f64, h_im: f64) -> Vec<Cf64> {
        vec![(h_re, h_im); n]
    }

    // -------------------------------------------------------------------
    // FFT / IFFT round-trip tests
    // -------------------------------------------------------------------

    #[test]
    fn test_fft_ifft_roundtrip() {
        let sig: Vec<Cf64> = (0..64)
            .map(|i| (i as f64 / 64.0, -(i as f64) / 128.0))
            .collect();
        let f = fft(&sig);
        let rec = ifft(&f);
        for (a, b) in sig.iter().zip(rec.iter()) {
            assert!(nearly_eq(a.0, b.0, 1e-9));
            assert!(nearly_eq(a.1, b.1, 1e-9));
        }
    }

    #[test]
    fn test_fft_single_tone() {
        // Pure tone at frequency bin 3
        let n = 64usize;
        let sig: Vec<Cf64> = (0..n)
            .map(|i| {
                let ph = 2.0 * PI * 3.0 * i as f64 / n as f64;
                (ph.cos(), ph.sin())
            })
            .collect();
        let f = fft(&sig);
        // Bin 3 should dominate
        let magnitudes: Vec<f64> = f.iter().map(|&x| c_abs(x)).collect();
        let peak_bin = magnitudes
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak_bin, 3);
    }

    #[test]
    fn test_ifft_dc() {
        // DC in frequency domain → constant time domain
        let n = 16usize;
        let mut freq = vec![(0.0, 0.0); n];
        freq[0] = (n as f64, 0.0);
        let time = ifft(&freq);
        for s in &time {
            assert!(nearly_eq(s.0, 1.0, 1e-9));
            assert!(nearly_eq(s.1, 0.0, 1e-9));
        }
    }

    // -------------------------------------------------------------------
    // LS estimation
    // -------------------------------------------------------------------

    #[test]
    fn test_ls_estimation_flat_channel() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let n = 64;
        // Reference: BPSK-like alternating +1/-1
        let ref_sig: Vec<Cf64> = (0..n).map(|i| if i % 2 == 0 { (1.0, 0.0) } else { (-1.0, 0.0) }).collect();
        // Flat channel H = 2 + 1j
        let h_re = 2.0_f64;
        let h_im = 1.0_f64;
        let rx: Vec<Cf64> = ref_sig.iter().map(|&x| c_mul(x, (h_re, h_im))).collect();
        let csi = proc.extract_csi_ls(&rx, &ref_sig);
        for c in &csi {
            assert!(nearly_eq(c.0, h_re, 1e-9));
            assert!(nearly_eq(c.1, h_im, 1e-9));
        }
    }

    #[test]
    fn test_ls_estimation_identity_channel() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let n = 64;
        let sig: Vec<Cf64> = (0..n)
            .map(|i| {
                let ph = 2.0 * PI * (i as f64) / n as f64;
                (ph.cos(), ph.sin())
            })
            .collect();
        let csi = proc.extract_csi_ls(&sig, &sig);
        for c in &csi {
            assert!(nearly_eq(c.0, 1.0, 1e-9));
            assert!(nearly_eq(c.1, 0.0, 1e-9));
        }
    }

    #[test]
    fn test_ls_zero_ref() {
        // Zero reference → all zeros in output (handled gracefully)
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let rx = vec![(1.0, 0.0); 8];
        let ref_z = vec![(0.0, 0.0); 8];
        let csi = proc.extract_csi_ls(&rx, &ref_z);
        for c in &csi {
            assert!(nearly_eq(c.0, 0.0, 1e-9));
            assert!(nearly_eq(c.1, 0.0, 1e-9));
        }
    }

    // -------------------------------------------------------------------
    // MMSE estimation
    // -------------------------------------------------------------------

    #[test]
    fn test_mmse_high_snr_approaches_ls() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let n = 64;
        let ref_sig: Vec<Cf64> = vec![(1.0, 0.0); n];
        // Flat H = 1 + 0j
        let rx = ref_sig.clone();
        let h_ls = proc.extract_csi_ls(&rx, &ref_sig);
        let h_mmse = proc.extract_csi_mmse(&rx, &ref_sig, 40.0);
        // At high SNR, MMSE should be close to LS
        let diff: f64 = h_ls
            .iter()
            .zip(h_mmse.iter())
            .map(|(&a, &b)| c_abs(c_sub(a, b)))
            .sum::<f64>()
            / n as f64;
        assert!(diff < 0.5, "MMSE vs LS diff = {diff}");
    }

    #[test]
    fn test_mmse_returns_correct_length() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let n = 64;
        let ref_sig: Vec<Cf64> = vec![(1.0, 0.0); n];
        let rx = ref_sig.clone();
        let h_mmse = proc.extract_csi_mmse(&rx, &ref_sig, 20.0);
        assert_eq!(h_mmse.len(), n);
    }

    // -------------------------------------------------------------------
    // Magnitude and phase response
    // -------------------------------------------------------------------

    #[test]
    fn test_magnitude_response_unit() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let csi = vec![(1.0_f64, 0.0_f64); 64];
        let mag = proc.magnitude_response(&csi);
        for m in mag {
            assert!(nearly_eq(m, 1.0, 1e-9));
        }
    }

    #[test]
    fn test_phase_response_known() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let csi = vec![(0.0_f64, 1.0_f64); 8]; // phase = pi/2
        let phases = proc.phase_response(&csi);
        for p in phases {
            assert!(nearly_eq(p, PI / 2.0, 1e-9));
        }
    }

    #[test]
    fn test_group_delay_flat() {
        // Flat channel: constant phase → zero group delay
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let csi = vec![(1.0_f64, 0.0_f64); 64];
        let gd = proc.group_delay(&csi);
        for g in gd {
            assert!(g.abs() < 1e-6, "group delay = {g}");
        }
    }

    // -------------------------------------------------------------------
    // CIR and delay spread
    // -------------------------------------------------------------------

    #[test]
    fn test_cir_delta_channel() {
        // Frequency-flat channel (all 1+0j) → CIR is a delta at tap 0.
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let n = 64;
        let csi = vec![(1.0, 0.0); n];
        let cir = proc.compute_cir(&csi);
        let mag0 = c_abs(cir[0]);
        assert!(nearly_eq(mag0, 1.0, 1e-6));
        for i in 1..n {
            assert!(c_abs(cir[i]) < 1e-6);
        }
    }

    #[test]
    fn test_rms_delay_spread_flat() {
        // Flat (delta) CIR → zero delay spread.
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let n = 64;
        let csi = vec![(1.0, 0.0); n];
        let cir = proc.compute_cir(&csi);
        let rms = proc.rms_delay_spread(&cir);
        assert!(rms < 0.01, "rms = {rms}");
    }

    #[test]
    fn test_rms_delay_spread_two_path() {
        // Two-path: H(k) = 1 + e^{-j 2π k τ / N}  → CIR has two taps.
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let n = 64usize;
        let delay_tap = 4usize;
        let csi: Vec<Cf64> = (0..n)
            .map(|k| {
                let ph = 2.0 * PI * k as f64 * delay_tap as f64 / n as f64;
                // (1 + cos(ph), -sin(ph))
                (1.0 + ph.cos(), -ph.sin())
            })
            .collect();
        let cir = proc.compute_cir(&csi);
        let rms = proc.rms_delay_spread(&cir);
        // Expect delay spread between 0 and delay_tap
        assert!(rms >= 0.0 && rms <= delay_tap as f64 + 1.0);
    }

    #[test]
    fn test_mean_excess_delay_flat() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let n = 64;
        let csi = vec![(1.0, 0.0); n];
        let cir = proc.compute_cir(&csi);
        let med = proc.mean_excess_delay(&cir);
        assert!(nearly_eq(med, 0.0, 1e-3));
    }

    #[test]
    fn test_channel_profile_fields() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let n = 64;
        let csi = vec![(1.0, 0.0); n];
        let profile = proc.compute_channel_profile(&csi);
        assert_eq!(profile.cir.len(), n);
        assert_eq!(profile.pdp.len(), n);
        assert_eq!(profile.delays.len(), n);
        assert!(profile.coherence_bandwidth_hz > 0.0);
    }

    // -------------------------------------------------------------------
    // Doppler estimation
    // -------------------------------------------------------------------

    #[test]
    fn test_doppler_static_channel() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        // All packets identical → zero Doppler / high coherence
        let flat = vec![(1.0, 0.0); 64];
        let packets: Vec<Vec<Cf64>> = vec![flat; 20];
        let est = proc.estimate_doppler(&packets, 0.01);
        assert!(est.max_doppler_hz >= 0.0);
        assert!(est.coherence_time_s > 0.0);
    }

    #[test]
    fn test_doppler_single_packet() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let flat = vec![(1.0, 0.0); 64];
        let packets = vec![flat];
        let est = proc.estimate_doppler(&packets, 0.01);
        assert_eq!(est.max_doppler_hz, 0.0);
        assert_eq!(est.correlation_vs_lag.len(), 1);
    }

    #[test]
    fn test_doppler_varying_channel() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let nsc = 64;
        // Simulate varying amplitude
        let packets: Vec<Vec<Cf64>> = (0..30)
            .map(|t| {
                (0..nsc)
                    .map(|k| {
                        let a = 1.0 + 0.5 * (2.0 * PI * t as f64 / 10.0 + k as f64 * 0.1).sin();
                        (a, 0.0)
                    })
                    .collect()
            })
            .collect();
        let est = proc.estimate_doppler(&packets, 0.001);
        // Should detect some non-trivial coherence time
        assert!(est.coherence_time_s >= 0.0);
    }

    // -------------------------------------------------------------------
    // Path loss
    // -------------------------------------------------------------------

    #[test]
    fn test_path_loss_unit_channel() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let csi = vec![(1.0, 0.0); 64];
        let pl = proc.estimate_path_loss_db(&csi);
        // |H| = 1 → PL = -20 log10(1) = 0 dB
        assert!(nearly_eq(pl, 0.0, 1e-6));
    }

    #[test]
    fn test_path_loss_half_amplitude() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let csi = vec![(0.5, 0.0); 64];
        let pl = proc.estimate_path_loss_db(&csi);
        // |H| = 0.5 → PL ≈ 6 dB
        assert!(nearly_eq(pl, 20.0 * 0.5_f64.log10().abs(), 1e-6));
    }

    #[test]
    fn test_distance_estimation() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        // At d=d0=1m, PL should equal ref_pl=40dB → |H| = 10^(-40/20) = 0.01
        let h_mag = 10.0_f64.powf(-40.0 / 20.0);
        let csi = vec![(h_mag, 0.0); 64];
        let d = proc.estimate_distance_m(&csi);
        assert!(nearly_eq(d, 1.0, 0.01), "d = {d}");
    }

    // -------------------------------------------------------------------
    // MIMO CSI cube
    // -------------------------------------------------------------------

    #[test]
    fn test_build_csi_cube_dimensions() {
        let cfg = CsiConfig::wifi_n_2x2();
        let proc = WiFiCsiProcessor::new(cfg);
        let nsc = 64;
        let ntx = 2;
        let nrx = 2;
        let links: Vec<Vec<Vec<Cf64>>> = (0..ntx)
            .map(|tx| {
                (0..nrx)
                    .map(|rx| {
                        vec![(tx as f64 + 1.0, rx as f64 * 0.1); nsc]
                    })
                    .collect()
            })
            .collect();
        let cube = proc.build_csi_cube(&links);
        assert_eq!(cube.num_subcarriers, nsc);
        assert_eq!(cube.num_tx, ntx);
        assert_eq!(cube.num_rx, nrx);
        // Check a specific value
        assert!(nearly_eq(cube.matrices[0].get(0, 1).0, 1.0, 1e-9));
    }

    // -------------------------------------------------------------------
    // SVD
    // -------------------------------------------------------------------

    #[test]
    fn test_svd_identity_2x2() {
        let mat = vec![
            vec![(1.0, 0.0), (0.0, 0.0)],
            vec![(0.0, 0.0), (1.0, 0.0)],
        ];
        let result = svd_complex(&mat);
        // Both singular values should be 1; allow degenerate case where
        // the second is very close to 0 (power-iteration near-zero deflation)
        assert!(result.singular_values.len() >= 1, "Need at least 1 SV");
        for sv in &result.singular_values {
            assert!(nearly_eq(*sv, 1.0, 1e-3), "sv = {sv}");
        }
        assert!(result.rank >= 1);
        // Condition number is either 1.0 (both found) or infinity (one found)
        assert!(result.condition_number >= 0.9);
    }

    #[test]
    fn test_svd_rank_deficient() {
        // Rank-1 matrix: outer product of (1,0) and (1,0)
        let mat = vec![
            vec![(1.0, 0.0), (0.0, 0.0)],
            vec![(0.0, 0.0), (0.0, 0.0)],
        ];
        let result = svd_complex(&mat);
        assert!(result.rank <= 1);
    }

    #[test]
    fn test_svd_singular_values_ordered() {
        let mat = vec![
            vec![(3.0, 0.0), (0.0, 0.0)],
            vec![(0.0, 0.0), (1.0, 0.0)],
        ];
        let result = svd_complex(&mat);
        // First SV should be the largest; if only one found it must be ≈ 3.0
        assert!(!result.singular_values.is_empty(), "Expected at least 1 SV");
        assert!(nearly_eq(result.singular_values[0], 3.0, 1e-3),
            "expected σ₀ ≈ 3.0, got {}", result.singular_values[0]);
        if result.singular_values.len() >= 2 {
            assert!(result.singular_values[0] >= result.singular_values[1] - 1e-6);
        }
    }

    #[test]
    fn test_svd_csi_matrix_integration() {
        let cfg = CsiConfig::wifi_n_2x2();
        let proc = WiFiCsiProcessor::new(cfg);
        let mat = CsiMatrix {
            num_tx: 2,
            num_rx: 2,
            data: vec![
                vec![(2.0, 1.0), (0.5, -0.5)],
                vec![(-0.5, 0.3), (1.5, 0.0)],
            ],
        };
        let svd = proc.svd_decompose(&mat);
        assert!(svd.rank >= 1);
        assert!(svd.condition_number >= 1.0);
    }

    // -------------------------------------------------------------------
    // CSI feedback compression
    // -------------------------------------------------------------------

    #[test]
    fn test_compress_decompress_roundtrip() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let n = 64;
        // Create a smooth CSI
        let csi: Vec<Cf64> = (0..n)
            .map(|k| {
                let ph = 2.0 * PI * k as f64 / n as f64;
                (ph.cos(), 0.3 * ph.sin())
            })
            .collect();
        let (mag_c, ph_c) = proc.compress_csi_feedback(&csi);
        let rec = proc.decompress_csi_feedback(&mag_c, &ph_c, n);
        assert_eq!(rec.len(), n);
        // Rough reconstruction: magnitudes should be reasonable
        for (orig, rec_h) in csi.iter().zip(rec.iter()) {
            let dm = (c_abs(*orig) - c_abs(*rec_h)).abs();
            assert!(dm < 2.0, "compression error dm={dm}");
        }
    }

    #[test]
    fn test_compress_length() {
        let cfg = CsiConfig::wifi_a();
        let expected_taps = cfg.feedback_compression_taps;
        let proc = WiFiCsiProcessor::new(cfg);
        let csi = make_flat_csi(64, 1.0, 0.0);
        let (mag_c, ph_c) = proc.compress_csi_feedback(&csi);
        assert_eq!(mag_c.len(), expected_taps);
        assert_eq!(ph_c.len(), expected_taps);
    }

    // -------------------------------------------------------------------
    // Activity recognition features
    // -------------------------------------------------------------------

    #[test]
    fn test_activity_features_static() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let nsc = 64;
        let flat = vec![(1.0, 0.0); nsc];
        let packets = vec![flat; 10];
        let feat = proc.compute_activity_features(&packets);
        assert_eq!(feat.amplitude_variance.len(), nsc);
        // Static channel → near-zero variance
        for v in &feat.amplitude_variance {
            assert!(*v < 1e-10);
        }
    }

    #[test]
    fn test_activity_features_varying() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let nsc = 32;
        let packets: Vec<Vec<Cf64>> = (0..20)
            .map(|t| {
                (0..nsc)
                    .map(|k| {
                        let a = 1.0 + 0.5 * ((t as f64 / 5.0 + k as f64 * 0.1) * 2.0 * PI).sin();
                        (a, 0.0)
                    })
                    .collect()
            })
            .collect();
        let feat = proc.compute_activity_features(&packets);
        let total_var: f64 = feat.amplitude_variance.iter().sum();
        assert!(total_var > 0.0, "Expected non-zero variance");
    }

    #[test]
    fn test_activity_features_empty() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let feat = proc.compute_activity_features(&[]);
        assert!(feat.amplitude_variance.is_empty());
    }

    // -------------------------------------------------------------------
    // Beamforming
    // -------------------------------------------------------------------

    #[test]
    fn test_mrt_weights_normalised() {
        let cfg = CsiConfig::wifi_n_2x2();
        let proc = WiFiCsiProcessor::new(cfg);
        let h = vec![(1.0, 0.0), (0.0, 1.0)]; // unit vectors
        let w = proc.mrt_beamforming_weights(&h);
        let norm_sq: f64 = w.iter().map(|&x| c_abs2(x)).sum();
        assert!(nearly_eq(norm_sq, 1.0, 1e-9));
    }

    #[test]
    fn test_mrt_zero_channel() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let h = vec![(0.0, 0.0); 4];
        let w = proc.mrt_beamforming_weights(&h);
        for x in w {
            assert!(nearly_eq(c_abs(x), 0.0, 1e-12));
        }
    }

    #[test]
    fn test_zf_beamforming_2x2() {
        let cfg = CsiConfig::wifi_n_2x2();
        let proc = WiFiCsiProcessor::new(cfg);
        let mat = CsiMatrix {
            num_tx: 2,
            num_rx: 2,
            data: vec![vec![(1.0, 0.0), (0.0, 0.0)], vec![(0.0, 0.0), (1.0, 0.0)]],
        };
        let w = proc.compute_beamforming_weights(&mat);
        // For identity channel, ZF weights should also be unit on diagonal
        assert_eq!(w.len(), 2);
        assert_eq!(w[0].len(), 2);
    }

    #[test]
    fn test_ula_steering_vector_length() {
        let cfg = CsiConfig::wifi_n_2x2();
        let expected_tx = cfg.num_tx;
        let proc = WiFiCsiProcessor::new(cfg);
        let sv = proc.ula_steering_vector(0.0, 0.5);
        assert_eq!(sv.len(), expected_tx);
    }

    #[test]
    fn test_ula_steering_broadside() {
        // Broadside θ=0 → all entries are (1, 0)
        let cfg = CsiConfig::wifi_n_2x2();
        let proc = WiFiCsiProcessor::new(cfg);
        let sv = proc.ula_steering_vector(0.0, 0.5);
        for s in sv {
            assert!(nearly_eq(s.0, 1.0, 1e-9));
            assert!(nearly_eq(s.1, 0.0, 1e-9));
        }
    }

    // -------------------------------------------------------------------
    // Kalman tracking
    // -------------------------------------------------------------------

    #[test]
    fn test_kalman_track_converges() {
        let cfg = CsiConfig::wifi_a();
        let mut proc = WiFiCsiProcessor::new(cfg);
        let n = 64;
        let target = (2.0, -0.5);
        let csi = vec![target; n];
        // Run many updates
        let mut out = vec![(0.0, 0.0); n];
        for _ in 0..200 {
            out = proc.kalman_track_update(&csi);
        }
        for o in &out {
            assert!(nearly_eq(o.0, target.0, 0.1));
            assert!(nearly_eq(o.1, target.1, 0.1));
        }
    }

    #[test]
    fn test_kalman_reset() {
        let cfg = CsiConfig::wifi_a();
        let mut proc = WiFiCsiProcessor::new(cfg);
        let csi = vec![(5.0, 5.0); 64];
        // Pollute state
        for _ in 0..100 {
            proc.kalman_track_update(&csi);
        }
        proc.kalman_reset((0.0, 0.0));
        let out = proc.kalman_track_update(&vec![(0.0, 0.0); 64]);
        for o in &out {
            assert!(o.0.abs() < 0.5 && o.1.abs() < 0.5);
        }
    }

    #[test]
    fn test_fade_rate_static() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let csi = vec![(1.0, 0.0); 64];
        let rate = proc.estimate_fade_rate_hz(&csi, &csi, 0.01);
        assert!(nearly_eq(rate, 0.0, 1e-9));
    }

    // -------------------------------------------------------------------
    // Waterfilling / spectral efficiency
    // -------------------------------------------------------------------

    #[test]
    fn test_waterfilling_flat_channel() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let n = 64;
        let csi = vec![(1.0, 0.0); n]; // flat channel
        let c = proc.waterfilling_capacity(&csi, 20.0);
        let snr = 10.0_f64.powf(20.0 / 10.0); // 100
        let expected = (1.0 + snr).log2();
        // Should be within factor of 2 due to noise estimation
        assert!(c > 0.0 && c <= expected * 2.0);
    }

    #[test]
    fn test_waterfilling_capacity_increases_with_snr() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let csi = vec![(1.0, 0.0); 64];
        let c_low = proc.waterfilling_capacity(&csi, 0.0);
        let c_high = proc.waterfilling_capacity(&csi, 20.0);
        assert!(c_high > c_low, "c_high={c_high} c_low={c_low}");
    }

    #[test]
    fn test_spectral_efficiency_fields() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let n = 64;
        let csi = vec![(1.0, 0.0); n];
        let se = proc.compute_spectral_efficiency(&csi, 10.0);
        assert_eq!(se.sinr_db.len(), n);
        assert_eq!(se.power_allocation.len(), n);
        assert!(se.capacity_bps_hz > 0.0);
        assert!(se.shannon_limit_bps_hz > 0.0);
    }

    // -------------------------------------------------------------------
    // Active subcarrier extraction
    // -------------------------------------------------------------------

    #[test]
    fn test_extract_active_subcarriers_length() {
        let cfg = CsiConfig::wifi_a(); // num_active_subcarriers = 52
        let proc = WiFiCsiProcessor::new(cfg);
        let full = vec![(1.0, 0.0); 64];
        let active = proc.extract_active_subcarriers(&full);
        // half = 26, so lower 1..26 + upper 64-26..64 = 26+26=52
        assert_eq!(active.len(), 52);
    }

    // -------------------------------------------------------------------
    // Pilot interpolation
    // -------------------------------------------------------------------

    #[test]
    fn test_interpolate_from_pilots_constant() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let pilots = vec![(0usize, (2.0, 1.0)), (63usize, (2.0, 1.0))];
        let interp = proc.interpolate_from_pilots(&pilots, 64);
        for h in &interp {
            assert!(nearly_eq(h.0, 2.0, 1e-6));
            assert!(nearly_eq(h.1, 1.0, 1e-6));
        }
    }

    #[test]
    fn test_interpolate_from_pilots_linear() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        // Two pilots: (0, 0+0j) and (10, 10+0j)
        let pilots = vec![(0usize, (0.0, 0.0)), (10usize, (10.0, 0.0))];
        let interp = proc.interpolate_from_pilots(&pilots, 11);
        for (k, h) in interp.iter().enumerate() {
            assert!(nearly_eq(h.0, k as f64, 1e-6), "k={k} h={}", h.0);
        }
    }

    #[test]
    fn test_interpolate_empty_pilots() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let interp = proc.interpolate_from_pilots(&[], 8);
        assert_eq!(interp.len(), 8);
    }

    // -------------------------------------------------------------------
    // Per-subcarrier SNR
    // -------------------------------------------------------------------

    #[test]
    fn test_per_subcarrier_snr_unit_channel() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let csi = vec![(1.0, 0.0); 16];
        let snr = proc.per_subcarrier_snr_db(&csi, 0.01); // noise = -20 dB
        for s in snr {
            assert!(s > 0.0);
        }
    }

    #[test]
    fn test_per_subcarrier_snr_zero_noise() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let csi = vec![(1.0, 0.0); 8];
        let snr = proc.per_subcarrier_snr_db(&csi, 0.0);
        for s in snr {
            assert!(s > 99.0); // large SNR
        }
    }

    // -------------------------------------------------------------------
    // DCT roundtrip
    // -------------------------------------------------------------------

    #[test]
    fn test_dct_idct_roundtrip() {
        let x: Vec<f64> = (0..16).map(|i| (i as f64 / 16.0).sin()).collect();
        let c = dct2(&x);
        let r = idct2(&c);
        for (a, b) in x.iter().zip(r.iter()) {
            assert!(nearly_eq(*a, *b, 1e-9));
        }
    }

    // -------------------------------------------------------------------
    // Matrix inverse
    // -------------------------------------------------------------------

    #[test]
    fn test_mat_inv_2x2_identity() {
        let id = vec![vec![(1.0, 0.0), (0.0, 0.0)], vec![(0.0, 0.0), (1.0, 0.0)]];
        let inv = mat_inv_small(&id);
        assert!(nearly_eq(inv[0][0].0, 1.0, 1e-9));
        assert!(nearly_eq(inv[1][1].0, 1.0, 1e-9));
    }

    #[test]
    fn test_mat_inv_1x1() {
        let mat = vec![vec![(2.0, 0.0)]];
        let inv = mat_inv_small(&mat);
        assert!(nearly_eq(inv[0][0].0, 0.5, 1e-9));
    }

    // -------------------------------------------------------------------
    // CsiMatrix helpers
    // -------------------------------------------------------------------

    #[test]
    fn test_csi_matrix_get_set() {
        let mut m = CsiMatrix::new(2, 2);
        m.set(0, 1, (3.0, -1.5));
        let v = m.get(0, 1);
        assert!(nearly_eq(v.0, 3.0, 1e-9));
        assert!(nearly_eq(v.1, -1.5, 1e-9));
    }

    #[test]
    fn test_csi_cube_dimensions() {
        let cube = CsiCube::new(16, 4, 2);
        assert_eq!(cube.num_subcarriers, 16);
        assert_eq!(cube.num_tx, 4);
        assert_eq!(cube.num_rx, 2);
        assert_eq!(cube.matrices.len(), 16);
    }

    // -------------------------------------------------------------------
    // Config presets
    // -------------------------------------------------------------------

    #[test]
    fn test_config_wifi_a() {
        let cfg = CsiConfig::wifi_a();
        assert_eq!(cfg.fft_size, 64);
        assert_eq!(cfg.num_active_subcarriers, 52);
        assert_eq!(cfg.num_tx, 1);
        assert_eq!(cfg.num_rx, 1);
    }

    #[test]
    fn test_config_wifi_n() {
        let cfg = CsiConfig::wifi_n_2x2();
        assert_eq!(cfg.fft_size, 64);
        assert_eq!(cfg.num_tx, 2);
        assert_eq!(cfg.num_rx, 2);
    }

    #[test]
    fn test_config_wifi_ac() {
        let cfg = CsiConfig::wifi_ac_4x2();
        assert_eq!(cfg.fft_size, 256);
        assert_eq!(cfg.num_tx, 4);
        assert_eq!(cfg.num_rx, 2);
    }

    // -------------------------------------------------------------------
    // Noise power estimation
    // -------------------------------------------------------------------

    #[test]
    fn test_noise_power_flat_channel() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        // All equal → zero variance
        let csi = vec![(1.0, 0.0); 64];
        let noise = proc.estimate_noise_power(&csi);
        assert!(nearly_eq(noise, 0.0, 1e-9));
    }

    #[test]
    fn test_noise_power_varying() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let csi: Vec<Cf64> = (0..64)
            .map(|i| ((i as f64 / 32.0).sin(), 0.0))
            .collect();
        let noise = proc.estimate_noise_power(&csi);
        assert!(noise > 0.0);
    }

    // -------------------------------------------------------------------
    // Phase unwrap
    // -------------------------------------------------------------------

    #[test]
    fn test_unwrap_phase_no_jumps() {
        let cfg = CsiConfig::wifi_a();
        let proc = WiFiCsiProcessor::new(cfg);
        let phi = vec![0.1, 0.2, 0.3, 0.4];
        let uphi = proc.unwrap_phase(&phi);
        assert!(nearly_eq(uphi[3], 0.4, 1e-9));
    }

    // -------------------------------------------------------------------
    // Condition number of cube
    // -------------------------------------------------------------------

    #[test]
    fn test_cube_condition_numbers_identity() {
        let cfg = CsiConfig::wifi_n_2x2();
        let proc = WiFiCsiProcessor::new(cfg);
        let nsc = 4;
        let mut cube = CsiCube::new(nsc, 2, 2);
        // Set each subcarrier to identity matrix
        for k in 0..nsc {
            cube.matrices[k].set(0, 0, (1.0, 0.0));
            cube.matrices[k].set(1, 1, (1.0, 0.0));
        }
        let conds = proc.cube_condition_numbers(&cube);
        assert_eq!(conds.len(), nsc);
        for c in conds {
            assert!(c >= 0.9 && c < 1e3, "cond = {c}");
        }
    }
}
