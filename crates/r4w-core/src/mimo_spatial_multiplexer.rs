//! MIMO Spatial Multiplexing
//!
//! Implements MIMO spatial multiplexing with precoding, detection, and capacity
//! analysis for multi-antenna wireless systems (2x2, 4x4, up to 8x8).
//!
//! Key capabilities:
//! - Zero-forcing (ZF) and MMSE linear detection
//! - SVD-based precoding via power iteration
//! - Shannon capacity calculation with waterfilling power allocation
//! - Rayleigh fading channel model with deterministic seeded RNG
//! - Complete complex matrix algebra (multiply, inverse, Hermitian, etc.)
//!
//! Complex values are represented as `(f64, f64)` tuples (real, imaginary) to
//! keep the module self-contained with no external dependencies.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::mimo_spatial_multiplexer::*;
//!
//! let config = MimoConfig {
//!     num_tx: 2,
//!     num_rx: 2,
//!     snr_db: 20.0,
//!     modulation: MimoModulation::Qpsk,
//! };
//! let processor = MimoProcessor::new(config);
//!
//! // Two spatial streams, each carrying one QPSK symbol
//! let streams = vec![
//!     vec![(0.707, 0.707)],
//!     vec![(-0.707, 0.707)],
//! ];
//!
//! let channel = MimoChannel::new_identity(2);
//! let tx = processor.transmit(&streams);
//! let rx = channel.apply(&tx);
//! let detected = processor.receive(&rx, &channel);
//!
//! // With an identity channel the detected symbols match the input
//! assert!((detected[0][0].0 - streams[0][0].0).abs() < 1e-6);
//! ```

// ---------------------------------------------------------------------------
// Complex arithmetic helpers
// ---------------------------------------------------------------------------

/// Complex conjugate: (a, b) -> (a, -b).
#[inline]
pub fn cx_conj(z: (f64, f64)) -> (f64, f64) {
    (z.0, -z.1)
}

/// Complex multiply: (a+bi)(c+di).
#[inline]
pub fn cx_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex add.
#[inline]
pub fn cx_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex subtract.
#[inline]
pub fn cx_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Complex magnitude squared |z|^2.
#[inline]
pub fn cx_mag_sq(z: (f64, f64)) -> f64 {
    z.0 * z.0 + z.1 * z.1
}

/// Complex magnitude |z|.
#[inline]
pub fn cx_mag(z: (f64, f64)) -> f64 {
    cx_mag_sq(z).sqrt()
}

/// Complex division: a / b.
#[inline]
pub fn cx_div(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    let denom = cx_mag_sq(b);
    if denom < 1e-30 {
        return (0.0, 0.0);
    }
    (
        (a.0 * b.0 + a.1 * b.1) / denom,
        (a.1 * b.0 - a.0 * b.1) / denom,
    )
}

/// Scale complex by real scalar.
#[inline]
pub fn cx_scale(z: (f64, f64), s: f64) -> (f64, f64) {
    (z.0 * s, z.1 * s)
}

// ---------------------------------------------------------------------------
// Simple deterministic PRNG (xoshiro256**)
// ---------------------------------------------------------------------------

struct Rng {
    s: [u64; 4],
}

impl Rng {
    fn new(seed: u64) -> Self {
        // SplitMix64 to expand seed into state
        let mut z = seed;
        let mut s = [0u64; 4];
        for slot in &mut s {
            z = z.wrapping_add(0x9e3779b97f4a7c15);
            let mut x = z;
            x = (x ^ (x >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
            x = (x ^ (x >> 27)).wrapping_mul(0x94d049bb133111eb);
            *slot = x ^ (x >> 31);
        }
        Rng { s }
    }

    fn next_u64(&mut self) -> u64 {
        let result = (self.s[1].wrapping_mul(5)).rotate_left(7).wrapping_mul(9);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        result
    }

    /// Uniform f64 in [0, 1).
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Standard normal via Box-Muller.
    fn next_gaussian(&mut self) -> (f64, f64) {
        loop {
            let u1 = self.next_f64();
            let u2 = self.next_f64();
            if u1 > 1e-30 {
                let r = (-2.0 * u1.ln()).sqrt();
                let theta = 2.0 * std::f64::consts::PI * u2;
                return (r * theta.cos(), r * theta.sin());
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Complex matrix operations
// ---------------------------------------------------------------------------

/// Multiply two complex matrices: C = A * B.
///
/// `a` is (m x k), `b` is (k x n), result is (m x n).
pub fn complex_matrix_multiply(
    a: &[Vec<(f64, f64)>],
    b: &[Vec<(f64, f64)>],
) -> Vec<Vec<(f64, f64)>> {
    let m = a.len();
    if m == 0 {
        return vec![];
    }
    let k = a[0].len();
    let n = if b.is_empty() { 0 } else { b[0].len() };
    assert!(b.len() == k, "Matrix dimension mismatch in multiply");

    let mut c = vec![vec![(0.0, 0.0); n]; m];
    for i in 0..m {
        for j in 0..n {
            let mut sum = (0.0, 0.0);
            for p in 0..k {
                sum = cx_add(sum, cx_mul(a[i][p], b[p][j]));
            }
            c[i][j] = sum;
        }
    }
    c
}

/// Hermitian (conjugate-transpose) of a complex matrix.
pub fn complex_matrix_hermitian(a: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
    let m = a.len();
    if m == 0 {
        return vec![];
    }
    let n = a[0].len();
    let mut h = vec![vec![(0.0, 0.0); m]; n];
    for i in 0..m {
        for j in 0..n {
            h[j][i] = cx_conj(a[i][j]);
        }
    }
    h
}

/// Invert a square complex matrix using Gauss-Jordan elimination.
///
/// Returns `None` if the matrix is singular (or near-singular).
pub fn complex_matrix_inverse(a: &[Vec<(f64, f64)>]) -> Option<Vec<Vec<(f64, f64)>>> {
    let n = a.len();
    if n == 0 {
        return Some(vec![]);
    }
    assert!(a.iter().all(|row| row.len() == n), "Matrix must be square");

    // Augmented matrix [A | I]
    let mut aug: Vec<Vec<(f64, f64)>> = Vec::with_capacity(n);
    for i in 0..n {
        let mut row = Vec::with_capacity(2 * n);
        row.extend_from_slice(&a[i]);
        for j in 0..n {
            row.push(if i == j { (1.0, 0.0) } else { (0.0, 0.0) });
        }
        aug.push(row);
    }

    for col in 0..n {
        // Partial pivot
        let mut max_mag = cx_mag_sq(aug[col][col]);
        let mut max_row = col;
        for row in (col + 1)..n {
            let mag = cx_mag_sq(aug[row][col]);
            if mag > max_mag {
                max_mag = mag;
                max_row = row;
            }
        }
        if max_mag < 1e-24 {
            return None; // Singular
        }
        if max_row != col {
            aug.swap(col, max_row);
        }

        // Scale pivot row
        let pivot = aug[col][col];
        let pivot_inv = cx_div((1.0, 0.0), pivot);
        for j in 0..(2 * n) {
            aug[col][j] = cx_mul(aug[col][j], pivot_inv);
        }

        // Eliminate other rows
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            for j in 0..(2 * n) {
                let scaled = cx_mul(factor, aug[col][j]);
                aug[row][j] = cx_sub(aug[row][j], scaled);
            }
        }
    }

    // Extract inverse from right half
    let inv: Vec<Vec<(f64, f64)>> = aug
        .into_iter()
        .map(|row| row[n..].to_vec())
        .collect();
    Some(inv)
}

/// Create an n x n complex identity matrix.
pub fn complex_identity(n: usize) -> Vec<Vec<(f64, f64)>> {
    let mut m = vec![vec![(0.0, 0.0); n]; n];
    for i in 0..n {
        m[i][i] = (1.0, 0.0);
    }
    m
}

/// Add two complex matrices element-wise.
pub fn complex_matrix_add(
    a: &[Vec<(f64, f64)>],
    b: &[Vec<(f64, f64)>],
) -> Vec<Vec<(f64, f64)>> {
    assert_eq!(a.len(), b.len());
    a.iter()
        .zip(b.iter())
        .map(|(ar, br)| {
            assert_eq!(ar.len(), br.len());
            ar.iter().zip(br.iter()).map(|(&x, &y)| cx_add(x, y)).collect()
        })
        .collect()
}

/// Scale a complex matrix by a real scalar.
pub fn complex_matrix_scale(a: &[Vec<(f64, f64)>], s: f64) -> Vec<Vec<(f64, f64)>> {
    a.iter()
        .map(|row| row.iter().map(|&z| cx_scale(z, s)).collect())
        .collect()
}

// ---------------------------------------------------------------------------
// Enums and configuration
// ---------------------------------------------------------------------------

/// Modulation scheme for MIMO spatial streams.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MimoModulation {
    Bpsk,
    Qpsk,
    Qam16,
}

impl MimoModulation {
    /// Bits per symbol for this modulation.
    pub fn bits_per_symbol(self) -> usize {
        match self {
            MimoModulation::Bpsk => 1,
            MimoModulation::Qpsk => 2,
            MimoModulation::Qam16 => 4,
        }
    }
}

/// Configuration for a MIMO system.
#[derive(Debug, Clone)]
pub struct MimoConfig {
    pub num_tx: usize,
    pub num_rx: usize,
    pub snr_db: f64,
    pub modulation: MimoModulation,
}

impl MimoConfig {
    /// Linear SNR from dB.
    pub fn snr_linear(&self) -> f64 {
        10.0_f64.powf(self.snr_db / 10.0)
    }
}

// ---------------------------------------------------------------------------
// MimoChannel
// ---------------------------------------------------------------------------

/// MIMO channel represented by a complex matrix H of size (num_rx x num_tx).
///
/// The channel model is: y = H * x + n
/// where x is the transmitted vector, y is received, and n is noise.
#[derive(Debug, Clone)]
pub struct MimoChannel {
    /// Channel matrix H, dimensions: num_rx rows x num_tx columns.
    pub matrix: Vec<Vec<(f64, f64)>>,
}

impl MimoChannel {
    /// Create an n x n identity channel (perfect, no fading).
    pub fn new_identity(n: usize) -> Self {
        MimoChannel {
            matrix: complex_identity(n),
        }
    }

    /// Create a Rayleigh fading channel with i.i.d. CN(0,1) entries.
    ///
    /// Uses a deterministic seed for reproducibility.
    pub fn new_rayleigh(num_rx: usize, num_tx: usize, seed: u64) -> Self {
        let mut rng = Rng::new(seed);
        let scale = 1.0 / (2.0_f64).sqrt(); // Each component ~ N(0, 0.5)
        let mut matrix = vec![vec![(0.0, 0.0); num_tx]; num_rx];
        for i in 0..num_rx {
            for j in 0..num_tx {
                let (g1, g2) = rng.next_gaussian();
                matrix[i][j] = (g1 * scale, g2 * scale);
            }
        }
        MimoChannel { matrix }
    }

    /// Number of receive antennas.
    pub fn num_rx(&self) -> usize {
        self.matrix.len()
    }

    /// Number of transmit antennas.
    pub fn num_tx(&self) -> usize {
        if self.matrix.is_empty() {
            0
        } else {
            self.matrix[0].len()
        }
    }

    /// Apply the channel: y[k] = H * x[k] for each time sample k.
    ///
    /// `tx` has `num_tx` rows, each of length `num_samples`.
    /// Returns `num_rx` rows, each of length `num_samples`.
    pub fn apply(&self, tx: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
        let num_tx = self.num_tx();
        let num_rx = self.num_rx();
        assert_eq!(tx.len(), num_tx, "TX streams must match num_tx");

        let num_samples = if tx.is_empty() { 0 } else { tx[0].len() };

        let mut rx = vec![vec![(0.0, 0.0); num_samples]; num_rx];
        for k in 0..num_samples {
            for i in 0..num_rx {
                let mut sum = (0.0, 0.0);
                for j in 0..num_tx {
                    sum = cx_add(sum, cx_mul(self.matrix[i][j], tx[j][k]));
                }
                rx[i][k] = sum;
            }
        }
        rx
    }
}

// ---------------------------------------------------------------------------
// MimoProcessor
// ---------------------------------------------------------------------------

/// MIMO spatial multiplexing processor.
///
/// Handles precoding (transmit side) and detection (receive side).
pub struct MimoProcessor {
    config: MimoConfig,
}

impl MimoProcessor {
    /// Create a new MIMO processor with the given configuration.
    pub fn new(config: MimoConfig) -> Self {
        assert!(config.num_tx >= 1 && config.num_tx <= 8, "num_tx must be 1..8");
        assert!(config.num_rx >= 1 && config.num_rx <= 8, "num_rx must be 1..8");
        MimoProcessor { config }
    }

    /// Transmit: apply precoding to data streams.
    ///
    /// `data_streams` has `num_tx` spatial streams, each containing a vector
    /// of complex symbols. Returns precoded antenna signals (same dimensions).
    ///
    /// Currently applies identity precoding (direct mapping). For SVD-based
    /// precoding, use `svd_precoding` to obtain the precoding matrix and
    /// apply it manually.
    pub fn transmit(&self, data_streams: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
        assert_eq!(
            data_streams.len(),
            self.config.num_tx,
            "Number of streams must match num_tx"
        );
        // Power normalization: scale by 1/sqrt(num_tx) to keep total power constant
        let scale = 1.0 / (self.config.num_tx as f64).sqrt();
        data_streams
            .iter()
            .map(|stream| stream.iter().map(|&s| cx_scale(s, scale)).collect())
            .collect()
    }

    /// Receive: detect and demultiplex using MMSE detection.
    ///
    /// `rx_signals` has `num_rx` rows (one per RX antenna).
    /// Returns `num_tx` detected spatial streams.
    pub fn receive(
        &self,
        rx_signals: &[Vec<(f64, f64)>],
        channel: &MimoChannel,
    ) -> Vec<Vec<(f64, f64)>> {
        assert_eq!(rx_signals.len(), self.config.num_rx);
        mmse_detector(rx_signals, channel, self.config.snr_linear())
    }

    /// Access the configuration.
    pub fn config(&self) -> &MimoConfig {
        &self.config
    }
}

// ---------------------------------------------------------------------------
// Detectors
// ---------------------------------------------------------------------------

/// Zero-Forcing detector: x_hat = (H^H * H)^{-1} * H^H * y
///
/// Each column of the input/output corresponds to one time sample.
/// `y` has `num_rx` rows, returns `num_tx` rows.
pub fn zf_detector(
    y: &[Vec<(f64, f64)>],
    h: &MimoChannel,
) -> Vec<Vec<(f64, f64)>> {
    let hh = complex_matrix_hermitian(&h.matrix); // num_tx x num_rx
    let hh_h = complex_matrix_multiply(&hh, &h.matrix); // num_tx x num_tx
    let hh_h_inv = complex_matrix_inverse(&hh_h)
        .expect("Channel matrix H^H*H is singular; ZF detection not possible");
    let w = complex_matrix_multiply(&hh_h_inv, &hh); // num_tx x num_rx (ZF weight)

    apply_weight_matrix(&w, y)
}

/// MMSE detector: x_hat = (H^H*H + sigma^2*I)^{-1} * H^H * y
///
/// `snr_linear` is the per-stream SNR (P/sigma^2). The noise variance is
/// computed as sigma^2 = 1/snr_linear assuming unit signal power.
pub fn mmse_detector(
    y: &[Vec<(f64, f64)>],
    h: &MimoChannel,
    snr_linear: f64,
) -> Vec<Vec<(f64, f64)>> {
    let num_tx = h.num_tx();
    let hh = complex_matrix_hermitian(&h.matrix);
    let hh_h = complex_matrix_multiply(&hh, &h.matrix);

    // Add sigma^2 * I
    let sigma2 = if snr_linear > 1e-30 {
        1.0 / snr_linear
    } else {
        0.0
    };
    let sigma2_i = complex_matrix_scale(&complex_identity(num_tx), sigma2);
    let regularized = complex_matrix_add(&hh_h, &sigma2_i);

    let reg_inv = complex_matrix_inverse(&regularized)
        .expect("MMSE regularized matrix is singular");
    let w = complex_matrix_multiply(&reg_inv, &hh);

    apply_weight_matrix(&w, y)
}

/// Apply a weight matrix W (num_tx x num_rx) to received signals y (num_rx rows).
/// Returns detected signals (num_tx rows).
fn apply_weight_matrix(
    w: &[Vec<(f64, f64)>],
    y: &[Vec<(f64, f64)>],
) -> Vec<Vec<(f64, f64)>> {
    let num_tx = w.len();
    let num_rx = if w.is_empty() { 0 } else { w[0].len() };
    let num_samples = if y.is_empty() { 0 } else { y[0].len() };

    assert_eq!(y.len(), num_rx, "Weight matrix columns must match RX antennas");

    let mut result = vec![vec![(0.0, 0.0); num_samples]; num_tx];
    for k in 0..num_samples {
        for i in 0..num_tx {
            let mut sum = (0.0, 0.0);
            for j in 0..num_rx {
                sum = cx_add(sum, cx_mul(w[i][j], y[j][k]));
            }
            result[i][k] = sum;
        }
    }
    result
}

// ---------------------------------------------------------------------------
// SVD precoding (power iteration method)
// ---------------------------------------------------------------------------

/// SVD decomposition of the channel matrix for precoding.
///
/// Returns (U, singular_values, V_hermitian) where H ≈ U * diag(s) * V^H.
///
/// Uses power iteration to find singular vectors one at a time (deflation).
/// This is a simplified approach suitable for small matrices (up to 8x8).
pub fn svd_precoding(
    channel: &MimoChannel,
) -> (Vec<Vec<(f64, f64)>>, Vec<f64>, Vec<Vec<(f64, f64)>>) {
    let h = &channel.matrix;
    let m = channel.num_rx();
    let n = channel.num_tx();
    let rank = m.min(n);

    let mut singular_values = Vec::with_capacity(rank);
    let mut u_vecs: Vec<Vec<(f64, f64)>> = Vec::with_capacity(rank);
    let mut v_vecs: Vec<Vec<(f64, f64)>> = Vec::with_capacity(rank);

    // Work on a copy that we deflate
    let mut h_residual: Vec<Vec<(f64, f64)>> = h.to_vec();

    let max_iter = 200;
    let tol = 1e-12;

    // Use a simple seeded RNG for initial vectors to avoid degenerate cases
    let mut rng = Rng::new(0xDEAD_BEEF_CAFE_1234);

    for sv_idx in 0..rank {
        // Power iteration on H^H * H to find dominant right singular vector
        let hh = complex_matrix_hermitian(&h_residual);
        let hh_h = complex_matrix_multiply(&hh, &h_residual); // n x n

        // Random initial vector to avoid degenerate eigenvectors
        let mut v: Vec<(f64, f64)> = (0..n)
            .map(|_| {
                let (g1, g2) = rng.next_gaussian();
                (g1, g2)
            })
            .collect();

        // Orthogonalize against previously found v-vectors
        for prev_v in &v_vecs {
            let proj = inner_product(prev_v, &v);
            for k in 0..n {
                v[k] = cx_sub(v[k], cx_mul(proj, prev_v[k]));
            }
        }
        normalize_vec(&mut v);

        for _iter in 0..max_iter {
            let v_old = v.clone();
            // v_new = H^H * H * v
            v = mat_vec_mul(&hh_h, &v);

            // Re-orthogonalize against previously found v-vectors
            for prev_v in &v_vecs {
                let proj = inner_product(prev_v, &v);
                for k in 0..n {
                    v[k] = cx_sub(v[k], cx_mul(proj, prev_v[k]));
                }
            }
            normalize_vec(&mut v);

            // Check convergence: |1 - |<v_new, v_old>||
            let dot = inner_product(&v, &v_old);
            if (cx_mag(dot) - 1.0).abs() < tol {
                break;
            }
        }

        let _ = sv_idx;

        // u = H * v / sigma
        let u_raw = mat_vec_mul(&h_residual, &v);
        let sigma = vec_norm(&u_raw);

        if sigma < 1e-14 {
            // Remaining singular values are effectively zero
            break;
        }

        let u: Vec<(f64, f64)> = u_raw.iter().map(|&z| cx_scale(z, 1.0 / sigma)).collect();

        singular_values.push(sigma);
        u_vecs.push(u.clone());
        v_vecs.push(v.clone());

        // Deflate: H_residual -= sigma * u * v^H
        for i in 0..m {
            for j in 0..n {
                let outer = cx_mul(u[i], cx_conj(v[j]));
                h_residual[i][j] = cx_sub(h_residual[i][j], cx_scale(outer, sigma));
            }
        }
    }

    // Build U (m x rank), S (rank), V^H (rank x n)
    let u_matrix: Vec<Vec<(f64, f64)>> = (0..m)
        .map(|i| u_vecs.iter().map(|col| col[i]).collect())
        .collect();

    let vh_matrix: Vec<Vec<(f64, f64)>> = v_vecs
        .iter()
        .map(|v| v.iter().map(|&z| cx_conj(z)).collect())
        .collect();

    (u_matrix, singular_values, vh_matrix)
}

/// Matrix-vector multiply: result = A * x.
fn mat_vec_mul(a: &[Vec<(f64, f64)>], x: &[(f64, f64)]) -> Vec<(f64, f64)> {
    a.iter()
        .map(|row| {
            row.iter()
                .zip(x.iter())
                .fold((0.0, 0.0), |acc, (&ai, &xi)| cx_add(acc, cx_mul(ai, xi)))
        })
        .collect()
}

/// Complex inner product: <a, b> = sum(conj(a_i) * b_i).
fn inner_product(a: &[(f64, f64)], b: &[(f64, f64)]) -> (f64, f64) {
    a.iter()
        .zip(b.iter())
        .fold((0.0, 0.0), |acc, (&ai, &bi)| {
            cx_add(acc, cx_mul(cx_conj(ai), bi))
        })
}

/// Euclidean norm of a complex vector.
fn vec_norm(v: &[(f64, f64)]) -> f64 {
    v.iter().map(|&z| cx_mag_sq(z)).sum::<f64>().sqrt()
}

/// Normalize a complex vector in-place.
fn normalize_vec(v: &mut [(f64, f64)]) {
    let n = vec_norm(v);
    if n > 1e-30 {
        for z in v.iter_mut() {
            *z = cx_scale(*z, 1.0 / n);
        }
    }
}

// ---------------------------------------------------------------------------
// Capacity and power allocation
// ---------------------------------------------------------------------------

/// MIMO Shannon capacity in bps/Hz: C = sum_i log2(1 + snr * lambda_i^2 / num_tx)
///
/// where lambda_i are the singular values of the channel matrix H.
///
/// Assumes equal power allocation across spatial streams and unit-normalized
/// total transmit power.
pub fn mimo_capacity_bps_hz(channel: &MimoChannel, snr_linear: f64) -> f64 {
    let (_, singular_values, _) = svd_precoding(channel);
    let num_tx = channel.num_tx() as f64;
    singular_values
        .iter()
        .map(|&sv| {
            let eigen = sv * sv;
            (1.0 + snr_linear * eigen / num_tx).log2()
        })
        .sum()
}

/// Waterfilling power allocation across spatial streams.
///
/// Given channel singular values and total power budget, allocates power
/// to maximize mutual information: P_i = max(0, mu - 1/lambda_i^2).
///
/// Returns a vector of per-stream power allocations that sum to `total_power`.
pub fn waterfilling_power_allocation(singular_values: &[f64], total_power: f64) -> Vec<f64> {
    let n = singular_values.len();
    if n == 0 {
        return vec![];
    }

    // Channel gains (eigenvalues of H^H H)
    let gains: Vec<f64> = singular_values.iter().map(|&sv| sv * sv).collect();

    // Inverse gains (noise floor per stream)
    let mut inv_gains: Vec<(usize, f64)> = gains
        .iter()
        .enumerate()
        .map(|(i, &g)| {
            if g > 1e-30 {
                (i, 1.0 / g)
            } else {
                (i, f64::INFINITY)
            }
        })
        .collect();

    // Sort by inverse gain (best channels first)
    inv_gains.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

    let mut powers = vec![0.0; n];
    let mut active = n;

    // Iterative waterfilling: remove channels that get negative power
    loop {
        let sum_inv: f64 = inv_gains.iter().take(active).map(|(_, ig)| ig).sum();
        let mu = (total_power + sum_inv) / active as f64;

        let mut all_positive = true;
        for k in 0..active {
            let p = mu - inv_gains[k].1;
            if p < 0.0 {
                active = k;
                all_positive = false;
                break;
            }
        }
        if all_positive {
            for k in 0..active {
                powers[inv_gains[k].0] = mu - inv_gains[k].1;
            }
            break;
        }
    }

    powers
}

/// Condition number of the channel matrix: ratio of largest to smallest
/// singular value. A large condition number indicates ill-conditioning
/// (one or more weak spatial streams).
pub fn condition_number(channel: &MimoChannel) -> f64 {
    let (_, sv, _) = svd_precoding(channel);
    if sv.is_empty() {
        return f64::INFINITY;
    }
    let max_sv = sv.iter().cloned().fold(0.0_f64, f64::max);
    let min_sv = sv.iter().cloned().fold(f64::INFINITY, f64::min);
    if min_sv < 1e-30 {
        f64::INFINITY
    } else {
        max_sv / min_sv
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    fn assert_complex_near(a: (f64, f64), b: (f64, f64), tol: f64, msg: &str) {
        let diff = cx_mag(cx_sub(a, b));
        assert!(
            diff < tol,
            "{}: expected ({:.6}, {:.6}), got ({:.6}, {:.6}), diff = {:.2e}",
            msg,
            b.0,
            b.1,
            a.0,
            a.1,
            diff
        );
    }

    // -----------------------------------------------------------------------
    // Complex arithmetic
    // -----------------------------------------------------------------------

    #[test]
    fn test_cx_conj() {
        assert_eq!(cx_conj((3.0, 4.0)), (3.0, -4.0));
        assert_eq!(cx_conj((0.0, 0.0)), (0.0, 0.0));
        assert_eq!(cx_conj((-1.0, -2.0)), (-1.0, 2.0));
    }

    #[test]
    fn test_cx_mul() {
        // (1+2i)(3+4i) = (3-8) + (4+6)i = -5 + 10i
        let result = cx_mul((1.0, 2.0), (3.0, 4.0));
        assert_complex_near(result, (-5.0, 10.0), EPSILON, "cx_mul");
    }

    #[test]
    fn test_cx_div() {
        // (1+2i)/(1+0i) = (1+2i)
        let result = cx_div((1.0, 2.0), (1.0, 0.0));
        assert_complex_near(result, (1.0, 2.0), EPSILON, "cx_div identity");

        // (1+0i)/(0+1i) = (0-1i)
        let result2 = cx_div((1.0, 0.0), (0.0, 1.0));
        assert_complex_near(result2, (0.0, -1.0), EPSILON, "cx_div by i");

        // Division by near-zero
        let result3 = cx_div((1.0, 0.0), (0.0, 0.0));
        assert_eq!(result3, (0.0, 0.0));
    }

    #[test]
    fn test_cx_mag() {
        assert!((cx_mag((3.0, 4.0)) - 5.0).abs() < EPSILON);
        assert!((cx_mag((0.0, 0.0))).abs() < EPSILON);
        assert!((cx_mag((1.0, 0.0)) - 1.0).abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Matrix operations
    // -----------------------------------------------------------------------

    #[test]
    fn test_matrix_multiply_identity() {
        let i2 = complex_identity(2);
        let a = vec![
            vec![(1.0, 2.0), (3.0, 4.0)],
            vec![(5.0, 6.0), (7.0, 8.0)],
        ];
        let result = complex_matrix_multiply(&a, &i2);
        for i in 0..2 {
            for j in 0..2 {
                assert_complex_near(result[i][j], a[i][j], EPSILON, "A*I = A");
            }
        }
    }

    #[test]
    fn test_matrix_multiply_2x2() {
        // [[1, 0], [0, 1]] * [[2, 3], [4, 5]] = [[2, 3], [4, 5]]
        let a = vec![
            vec![(1.0, 0.0), (0.0, 0.0)],
            vec![(0.0, 0.0), (1.0, 0.0)],
        ];
        let b = vec![
            vec![(2.0, 0.0), (3.0, 0.0)],
            vec![(4.0, 0.0), (5.0, 0.0)],
        ];
        let c = complex_matrix_multiply(&a, &b);
        assert_complex_near(c[0][0], (2.0, 0.0), EPSILON, "[0][0]");
        assert_complex_near(c[1][1], (5.0, 0.0), EPSILON, "[1][1]");
    }

    #[test]
    fn test_matrix_hermitian() {
        let a = vec![
            vec![(1.0, 2.0), (3.0, 4.0)],
            vec![(5.0, 6.0), (7.0, 8.0)],
        ];
        let h = complex_matrix_hermitian(&a);
        assert_eq!(h.len(), 2);
        assert_eq!(h[0].len(), 2);
        // h[0][0] = conj(a[0][0]) = (1, -2)
        assert_complex_near(h[0][0], (1.0, -2.0), EPSILON, "H[0][0]");
        // h[0][1] = conj(a[1][0]) = (5, -6)
        assert_complex_near(h[0][1], (5.0, -6.0), EPSILON, "H[0][1]");
        // h[1][0] = conj(a[0][1]) = (3, -4)
        assert_complex_near(h[1][0], (3.0, -4.0), EPSILON, "H[1][0]");
    }

    #[test]
    fn test_matrix_inverse_identity() {
        let i3 = complex_identity(3);
        let inv = complex_matrix_inverse(&i3).unwrap();
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { (1.0, 0.0) } else { (0.0, 0.0) };
                assert_complex_near(inv[i][j], expected, EPSILON, "inv(I) = I");
            }
        }
    }

    #[test]
    fn test_matrix_inverse_2x2() {
        // A = [[1, 2], [3, 4]] (real), inv(A) = [[-2, 1], [1.5, -0.5]]
        let a = vec![
            vec![(1.0, 0.0), (2.0, 0.0)],
            vec![(3.0, 0.0), (4.0, 0.0)],
        ];
        let inv = complex_matrix_inverse(&a).unwrap();
        assert_complex_near(inv[0][0], (-2.0, 0.0), EPSILON, "inv[0][0]");
        assert_complex_near(inv[0][1], (1.0, 0.0), EPSILON, "inv[0][1]");
        assert_complex_near(inv[1][0], (1.5, 0.0), EPSILON, "inv[1][0]");
        assert_complex_near(inv[1][1], (-0.5, 0.0), EPSILON, "inv[1][1]");
    }

    #[test]
    fn test_matrix_inverse_complex() {
        // A * inv(A) should be identity
        let a = vec![
            vec![(1.0, 1.0), (2.0, -1.0)],
            vec![(0.0, 1.0), (1.0, 1.0)],
        ];
        let inv = complex_matrix_inverse(&a).unwrap();
        let product = complex_matrix_multiply(&a, &inv);
        for i in 0..2 {
            for j in 0..2 {
                let expected = if i == j { (1.0, 0.0) } else { (0.0, 0.0) };
                assert_complex_near(product[i][j], expected, 1e-10, "A*inv(A)=I");
            }
        }
    }

    #[test]
    fn test_matrix_inverse_singular() {
        // Singular matrix: second row is 2x first
        let a = vec![
            vec![(1.0, 0.0), (2.0, 0.0)],
            vec![(2.0, 0.0), (4.0, 0.0)],
        ];
        assert!(complex_matrix_inverse(&a).is_none());
    }

    #[test]
    fn test_hermitian_of_product() {
        // (A*B)^H = B^H * A^H
        let a = vec![
            vec![(1.0, 2.0), (3.0, 0.0)],
            vec![(0.0, 1.0), (2.0, -1.0)],
        ];
        let b = vec![
            vec![(0.0, 1.0), (1.0, 1.0)],
            vec![(2.0, 0.0), (0.0, -1.0)],
        ];
        let ab = complex_matrix_multiply(&a, &b);
        let ab_h = complex_matrix_hermitian(&ab);
        let bh_ah = complex_matrix_multiply(
            &complex_matrix_hermitian(&b),
            &complex_matrix_hermitian(&a),
        );
        for i in 0..2 {
            for j in 0..2 {
                assert_complex_near(ab_h[i][j], bh_ah[i][j], EPSILON, "(AB)^H = B^H A^H");
            }
        }
    }

    // -----------------------------------------------------------------------
    // MimoChannel
    // -----------------------------------------------------------------------

    #[test]
    fn test_identity_channel_passthrough() {
        let ch = MimoChannel::new_identity(2);
        let tx = vec![
            vec![(1.0, 0.0), (0.0, 1.0)],
            vec![(0.5, 0.5), (-1.0, 0.0)],
        ];
        let rx = ch.apply(&tx);
        assert_eq!(rx.len(), 2);
        for i in 0..2 {
            for k in 0..2 {
                assert_complex_near(rx[i][k], tx[i][k], EPSILON, "identity passthrough");
            }
        }
    }

    #[test]
    fn test_rayleigh_channel_dimensions() {
        let ch = MimoChannel::new_rayleigh(4, 2, 42);
        assert_eq!(ch.num_rx(), 4);
        assert_eq!(ch.num_tx(), 2);
        assert_eq!(ch.matrix.len(), 4);
        assert_eq!(ch.matrix[0].len(), 2);
    }

    #[test]
    fn test_rayleigh_channel_deterministic() {
        let ch1 = MimoChannel::new_rayleigh(2, 2, 123);
        let ch2 = MimoChannel::new_rayleigh(2, 2, 123);
        for i in 0..2 {
            for j in 0..2 {
                assert_eq!(ch1.matrix[i][j], ch2.matrix[i][j]);
            }
        }
    }

    #[test]
    fn test_rayleigh_channel_different_seeds() {
        let ch1 = MimoChannel::new_rayleigh(2, 2, 1);
        let ch2 = MimoChannel::new_rayleigh(2, 2, 2);
        // Very unlikely to produce the same matrix
        let mut any_different = false;
        for i in 0..2 {
            for j in 0..2 {
                if (ch1.matrix[i][j].0 - ch2.matrix[i][j].0).abs() > 1e-10 {
                    any_different = true;
                }
            }
        }
        assert!(any_different, "Different seeds should produce different channels");
    }

    #[test]
    fn test_channel_apply_single_sample() {
        // 2x2 channel, single sample
        let ch = MimoChannel {
            matrix: vec![
                vec![(1.0, 0.0), (0.0, 0.0)],
                vec![(0.0, 0.0), (1.0, 0.0)],
            ],
        };
        let tx = vec![vec![(2.0, 3.0)], vec![(4.0, 5.0)]];
        let rx = ch.apply(&tx);
        assert_complex_near(rx[0][0], (2.0, 3.0), EPSILON, "rx[0]");
        assert_complex_near(rx[1][0], (4.0, 5.0), EPSILON, "rx[1]");
    }

    // -----------------------------------------------------------------------
    // MimoProcessor
    // -----------------------------------------------------------------------

    #[test]
    fn test_processor_transmit_power_normalization() {
        let config = MimoConfig {
            num_tx: 4,
            num_rx: 4,
            snr_db: 20.0,
            modulation: MimoModulation::Qpsk,
        };
        let proc = MimoProcessor::new(config);

        let streams: Vec<Vec<(f64, f64)>> = (0..4)
            .map(|_| vec![(1.0, 0.0)])
            .collect();

        let tx = proc.transmit(&streams);
        // Total power across antennas for one sample should be 1.0
        let total_power: f64 = tx.iter().map(|s| cx_mag_sq(s[0])).sum();
        assert!((total_power - 1.0).abs() < EPSILON, "Total TX power should be 1.0");
    }

    #[test]
    fn test_processor_roundtrip_identity_2x2() {
        let config = MimoConfig {
            num_tx: 2,
            num_rx: 2,
            snr_db: 40.0,
            modulation: MimoModulation::Qpsk,
        };
        let proc = MimoProcessor::new(config);

        let streams = vec![
            vec![(1.0, 0.0), (-1.0, 0.0)],
            vec![(0.0, 1.0), (0.0, -1.0)],
        ];

        let ch = MimoChannel::new_identity(2);
        let tx = proc.transmit(&streams);
        let rx = ch.apply(&tx);
        let detected = proc.receive(&rx, &ch);

        // With identity channel, detected should be proportional to input
        // The scaling is 1/sqrt(num_tx) from TX, and the MMSE detector
        // should recover the transmitted (scaled) symbols.
        let scale = 1.0 / (2.0_f64).sqrt();
        for i in 0..2 {
            for k in 0..2 {
                let expected = cx_scale(streams[i][k], scale);
                assert_complex_near(detected[i][k], expected, 1e-4, "roundtrip 2x2");
            }
        }
    }

    // -----------------------------------------------------------------------
    // ZF detector
    // -----------------------------------------------------------------------

    #[test]
    fn test_zf_detector_identity_channel() {
        let ch = MimoChannel::new_identity(2);
        // Transmit directly (no processor scaling)
        let tx = vec![vec![(1.0, 0.0)], vec![(0.0, 1.0)]];
        let rx = ch.apply(&tx);
        let detected = zf_detector(&rx, &ch);
        assert_complex_near(detected[0][0], (1.0, 0.0), EPSILON, "ZF stream 0");
        assert_complex_near(detected[1][0], (0.0, 1.0), EPSILON, "ZF stream 1");
    }

    #[test]
    fn test_zf_detector_scaled_channel() {
        // H = [[2, 0], [0, 3]]
        let ch = MimoChannel {
            matrix: vec![
                vec![(2.0, 0.0), (0.0, 0.0)],
                vec![(0.0, 0.0), (3.0, 0.0)],
            ],
        };
        let tx = vec![vec![(1.0, 0.0)], vec![(1.0, 0.0)]];
        let rx = ch.apply(&tx);
        // rx = [[2, 0], [0, 3]] * [[1], [1]] = [[2], [3]]
        assert_complex_near(rx[0][0], (2.0, 0.0), EPSILON, "rx[0]");
        assert_complex_near(rx[1][0], (3.0, 0.0), EPSILON, "rx[1]");

        let detected = zf_detector(&rx, &ch);
        assert_complex_near(detected[0][0], (1.0, 0.0), EPSILON, "ZF recovers x[0]");
        assert_complex_near(detected[1][0], (1.0, 0.0), EPSILON, "ZF recovers x[1]");
    }

    #[test]
    fn test_zf_detector_complex_channel() {
        let ch = MimoChannel {
            matrix: vec![
                vec![(1.0, 1.0), (0.5, -0.5)],
                vec![(0.0, 1.0), (1.0, 0.0)],
            ],
        };
        let x = vec![vec![(1.0, 0.0)], vec![(0.0, 1.0)]];
        let y = ch.apply(&x);
        let detected = zf_detector(&y, &ch);
        assert_complex_near(detected[0][0], (1.0, 0.0), 1e-8, "ZF complex ch stream 0");
        assert_complex_near(detected[1][0], (0.0, 1.0), 1e-8, "ZF complex ch stream 1");
    }

    // -----------------------------------------------------------------------
    // MMSE detector
    // -----------------------------------------------------------------------

    #[test]
    fn test_mmse_detector_high_snr() {
        // At very high SNR, MMSE ≈ ZF
        let ch = MimoChannel {
            matrix: vec![
                vec![(1.0, 0.5), (0.3, -0.2)],
                vec![(-0.1, 0.4), (0.8, 0.1)],
            ],
        };
        let x = vec![vec![(1.0, 0.0)], vec![(-1.0, 0.0)]];
        let y = ch.apply(&x);

        let zf_result = zf_detector(&y, &ch);
        let mmse_result = mmse_detector(&y, &ch, 1e6);

        for i in 0..2 {
            assert_complex_near(
                mmse_result[i][0],
                zf_result[i][0],
                1e-4,
                "MMSE ≈ ZF at high SNR",
            );
        }
    }

    #[test]
    fn test_mmse_detector_identity_channel() {
        let ch = MimoChannel::new_identity(2);
        let tx = vec![vec![(1.0, 0.0)], vec![(0.0, 1.0)]];
        let rx = ch.apply(&tx);
        let detected = mmse_detector(&rx, &ch, 100.0);
        // At SNR=100 (20 dB), MMSE should be very close to input
        // MMSE weight for identity H: (I + sigma^2 I)^{-1} = 1/(1+0.01) * I
        let expected_scale = 100.0 / 101.0;
        assert_complex_near(
            detected[0][0],
            (expected_scale, 0.0),
            EPSILON,
            "MMSE identity stream 0",
        );
        assert_complex_near(
            detected[1][0],
            (0.0, expected_scale),
            EPSILON,
            "MMSE identity stream 1",
        );
    }

    // -----------------------------------------------------------------------
    // SVD precoding
    // -----------------------------------------------------------------------

    #[test]
    fn test_svd_identity_channel() {
        let ch = MimoChannel::new_identity(2);
        let (_, sv, _) = svd_precoding(&ch);
        assert_eq!(sv.len(), 2);
        for &s in &sv {
            assert!((s - 1.0).abs() < 1e-6, "Identity singular values should be 1.0");
        }
    }

    #[test]
    fn test_svd_diagonal_channel() {
        let ch = MimoChannel {
            matrix: vec![
                vec![(3.0, 0.0), (0.0, 0.0)],
                vec![(0.0, 0.0), (5.0, 0.0)],
            ],
        };
        let (_, mut sv, _) = svd_precoding(&ch);
        sv.sort_by(|a, b| b.partial_cmp(a).unwrap());
        assert!((sv[0] - 5.0).abs() < 1e-6, "Largest SV should be 5.0");
        assert!((sv[1] - 3.0).abs() < 1e-6, "Smallest SV should be 3.0");
    }

    #[test]
    fn test_svd_reconstruction() {
        // H ≈ U * diag(s) * V^H
        let ch = MimoChannel::new_rayleigh(2, 2, 99);
        let (u, sv, vh) = svd_precoding(&ch);

        // Build diag(s) as a matrix
        let rank = sv.len();
        let mut s_mat = vec![vec![(0.0, 0.0); rank]; rank];
        for i in 0..rank {
            s_mat[i][i] = (sv[i], 0.0);
        }

        let us = complex_matrix_multiply(&u, &s_mat);
        let h_reconstructed = complex_matrix_multiply(&us, &vh);

        for i in 0..2 {
            for j in 0..2 {
                assert_complex_near(
                    h_reconstructed[i][j],
                    ch.matrix[i][j],
                    1e-6,
                    "SVD reconstruction",
                );
            }
        }
    }

    #[test]
    fn test_svd_4x4() {
        let ch = MimoChannel::new_rayleigh(4, 4, 77);
        let (u, sv, vh) = svd_precoding(&ch);
        assert_eq!(sv.len(), 4);
        // All singular values should be positive
        for &s in &sv {
            assert!(s >= 0.0, "Singular values must be non-negative");
        }
        // Reconstruction
        let mut s_mat = vec![vec![(0.0, 0.0); 4]; 4];
        for i in 0..4 {
            s_mat[i][i] = (sv[i], 0.0);
        }
        let h_rec = complex_matrix_multiply(
            &complex_matrix_multiply(&u, &s_mat),
            &vh,
        );
        for i in 0..4 {
            for j in 0..4 {
                assert_complex_near(h_rec[i][j], ch.matrix[i][j], 1e-5, "SVD 4x4 reconstruction");
            }
        }
    }

    // -----------------------------------------------------------------------
    // Capacity
    // -----------------------------------------------------------------------

    #[test]
    fn test_capacity_identity_channel() {
        let ch = MimoChannel::new_identity(2);
        let snr_linear = 10.0; // 10 dB
        let cap = mimo_capacity_bps_hz(&ch, snr_linear);
        // C = 2 * log2(1 + 10/2) = 2 * log2(6) ≈ 5.17
        let expected = 2.0 * (1.0 + 10.0 / 2.0_f64).log2();
        assert!(
            (cap - expected).abs() < 0.01,
            "Capacity: got {:.3}, expected {:.3}",
            cap,
            expected
        );
    }

    #[test]
    fn test_capacity_increases_with_snr() {
        let ch = MimoChannel::new_rayleigh(2, 2, 42);
        let cap_low = mimo_capacity_bps_hz(&ch, 1.0);
        let cap_high = mimo_capacity_bps_hz(&ch, 100.0);
        assert!(
            cap_high > cap_low,
            "Capacity should increase with SNR: {:.3} vs {:.3}",
            cap_low,
            cap_high
        );
    }

    #[test]
    fn test_capacity_increases_with_antennas() {
        let ch2 = MimoChannel::new_identity(2);
        let ch4 = MimoChannel::new_identity(4);
        let snr = 10.0;
        let cap2 = mimo_capacity_bps_hz(&ch2, snr);
        let cap4 = mimo_capacity_bps_hz(&ch4, snr);
        assert!(
            cap4 > cap2,
            "4x4 capacity should exceed 2x2: {:.3} vs {:.3}",
            cap2,
            cap4
        );
    }

    #[test]
    fn test_capacity_zero_snr() {
        let ch = MimoChannel::new_identity(2);
        let cap = mimo_capacity_bps_hz(&ch, 0.0);
        // log2(1 + 0) = 0
        assert!(cap.abs() < 0.01, "Capacity at 0 SNR should be ~0");
    }

    // -----------------------------------------------------------------------
    // Waterfilling
    // -----------------------------------------------------------------------

    #[test]
    fn test_waterfilling_equal_channels() {
        let sv = vec![2.0, 2.0, 2.0];
        let powers = waterfilling_power_allocation(&sv, 3.0);
        // Equal channels => equal power allocation
        for &p in &powers {
            assert!((p - 1.0).abs() < EPSILON, "Equal channels get equal power");
        }
    }

    #[test]
    fn test_waterfilling_power_sums_to_total() {
        let sv = vec![3.0, 1.0, 0.5];
        let total = 5.0;
        let powers = waterfilling_power_allocation(&sv, total);
        let sum: f64 = powers.iter().sum();
        assert!(
            (sum - total).abs() < 0.01,
            "Powers must sum to total: got {:.3}",
            sum
        );
    }

    #[test]
    fn test_waterfilling_weak_channel_gets_less() {
        let sv = vec![10.0, 0.1]; // One strong, one weak
        let powers = waterfilling_power_allocation(&sv, 2.0);
        assert!(
            powers[0] > powers[1],
            "Strong channel should get more power: {:.4} vs {:.4}",
            powers[0],
            powers[1]
        );
    }

    #[test]
    fn test_waterfilling_empty() {
        let powers = waterfilling_power_allocation(&[], 10.0);
        assert!(powers.is_empty());
    }

    #[test]
    fn test_waterfilling_single_stream() {
        let powers = waterfilling_power_allocation(&[5.0], 3.0);
        assert!((powers[0] - 3.0).abs() < EPSILON);
    }

    // -----------------------------------------------------------------------
    // Condition number
    // -----------------------------------------------------------------------

    #[test]
    fn test_condition_number_identity() {
        let ch = MimoChannel::new_identity(3);
        let cn = condition_number(&ch);
        assert!((cn - 1.0).abs() < 1e-6, "Identity condition number should be 1.0");
    }

    #[test]
    fn test_condition_number_diagonal() {
        let ch = MimoChannel {
            matrix: vec![
                vec![(4.0, 0.0), (0.0, 0.0)],
                vec![(0.0, 0.0), (2.0, 0.0)],
            ],
        };
        let cn = condition_number(&ch);
        assert!((cn - 2.0).abs() < 1e-4, "Condition number should be 4/2=2, got {:.4}", cn);
    }

    #[test]
    fn test_condition_number_ill_conditioned() {
        // Near-singular channel
        let ch = MimoChannel {
            matrix: vec![
                vec![(1.0, 0.0), (1.0, 0.0)],
                vec![(1.0, 0.0), (1.0 + 1e-8, 0.0)],
            ],
        };
        let cn = condition_number(&ch);
        assert!(cn > 1e6, "Near-singular channel should have large condition number");
    }

    // -----------------------------------------------------------------------
    // Modulation enum
    // -----------------------------------------------------------------------

    #[test]
    fn test_modulation_bits_per_symbol() {
        assert_eq!(MimoModulation::Bpsk.bits_per_symbol(), 1);
        assert_eq!(MimoModulation::Qpsk.bits_per_symbol(), 2);
        assert_eq!(MimoModulation::Qam16.bits_per_symbol(), 4);
    }

    // -----------------------------------------------------------------------
    // Edge cases and larger configurations
    // -----------------------------------------------------------------------

    #[test]
    fn test_1x1_mimo_is_siso() {
        let config = MimoConfig {
            num_tx: 1,
            num_rx: 1,
            snr_db: 60.0, // Very high SNR so MMSE bias is negligible
            modulation: MimoModulation::Bpsk,
        };
        let proc = MimoProcessor::new(config);
        let streams = vec![vec![(1.0, 0.0), (-1.0, 0.0), (1.0, 0.0)]];
        let ch = MimoChannel::new_identity(1);
        let tx = proc.transmit(&streams);
        let rx = ch.apply(&tx);
        let detected = proc.receive(&rx, &ch);

        assert_eq!(detected.len(), 1);
        assert_eq!(detected[0].len(), 3);
        for k in 0..3 {
            assert_complex_near(detected[0][k], tx[0][k], 1e-4, "1x1 SISO");
        }
    }

    #[test]
    fn test_4x4_zf_roundtrip() {
        let ch = MimoChannel::new_rayleigh(4, 4, 314);
        let x: Vec<Vec<(f64, f64)>> = (0..4)
            .map(|i| vec![((i as f64) * 0.5 - 0.75, 0.25)])
            .collect();
        let y = ch.apply(&x);
        let detected = zf_detector(&y, &ch);
        for i in 0..4 {
            assert_complex_near(detected[i][0], x[i][0], 1e-6, "4x4 ZF roundtrip");
        }
    }

    #[test]
    fn test_overdet_3rx_2tx_zf() {
        // Over-determined: more RX than TX (should work with pseudo-inverse)
        let ch = MimoChannel::new_rayleigh(3, 2, 555);
        let x = vec![vec![(1.0, 0.5)], vec![(-0.5, 1.0)]];
        let y = ch.apply(&x);
        let detected = zf_detector(&y, &ch);
        assert_eq!(detected.len(), 2);
        assert_complex_near(detected[0][0], (1.0, 0.5), 1e-6, "3x2 ZF stream 0");
        assert_complex_near(detected[1][0], (-0.5, 1.0), 1e-6, "3x2 ZF stream 1");
    }

    #[test]
    fn test_snr_config_linear() {
        let config = MimoConfig {
            num_tx: 2,
            num_rx: 2,
            snr_db: 10.0,
            modulation: MimoModulation::Qpsk,
        };
        assert!((config.snr_linear() - 10.0).abs() < 0.01);

        let config2 = MimoConfig {
            num_tx: 2,
            num_rx: 2,
            snr_db: 0.0,
            modulation: MimoModulation::Bpsk,
        };
        assert!((config2.snr_linear() - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_multiple_samples_zf() {
        let ch = MimoChannel::new_rayleigh(2, 2, 888);
        // 10 samples per stream
        let x: Vec<Vec<(f64, f64)>> = vec![
            (0..10).map(|k| ((k as f64) * 0.1, 0.0)).collect(),
            (0..10).map(|k| (0.0, (k as f64) * 0.1)).collect(),
        ];
        let y = ch.apply(&x);
        let detected = zf_detector(&y, &ch);
        for k in 0..10 {
            assert_complex_near(detected[0][k], x[0][k], 1e-8, "multi-sample ZF s0");
            assert_complex_near(detected[1][k], x[1][k], 1e-8, "multi-sample ZF s1");
        }
    }

    #[test]
    fn test_matrix_multiply_non_square() {
        // 2x3 * 3x1 = 2x1
        let a = vec![
            vec![(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)],
            vec![(4.0, 0.0), (5.0, 0.0), (6.0, 0.0)],
        ];
        let b = vec![
            vec![(1.0, 0.0)],
            vec![(0.0, 0.0)],
            vec![(-1.0, 0.0)],
        ];
        let c = complex_matrix_multiply(&a, &b);
        assert_eq!(c.len(), 2);
        assert_eq!(c[0].len(), 1);
        // Row 0: 1*1 + 2*0 + 3*(-1) = -2
        assert_complex_near(c[0][0], (-2.0, 0.0), EPSILON, "non-square [0][0]");
        // Row 1: 4*1 + 5*0 + 6*(-1) = -2
        assert_complex_near(c[1][0], (-2.0, 0.0), EPSILON, "non-square [1][0]");
    }

    #[test]
    fn test_matrix_inverse_3x3() {
        let a = vec![
            vec![(1.0, 0.0), (0.0, 0.0), (2.0, 0.0)],
            vec![(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)],
            vec![(3.0, 0.0), (0.0, 0.0), (1.0, 0.0)],
        ];
        let inv = complex_matrix_inverse(&a).unwrap();
        let product = complex_matrix_multiply(&a, &inv);
        for i in 0..3 {
            for j in 0..3 {
                let expected = if i == j { (1.0, 0.0) } else { (0.0, 0.0) };
                assert_complex_near(product[i][j], expected, 1e-10, "3x3 inv check");
            }
        }
    }
}
