//! MIMO Precoding — Spatial Multiplexing and Beamforming
//!
//! Implements MIMO precoding techniques for multi-antenna transmit processing.
//! Supports identity (open-loop spatial multiplexing), zero-forcing, MMSE,
//! matched-filter (conjugate beamforming), and DFT-based precoding. These are
//! essential building blocks for LTE/5G NR, Wi-Fi 802.11n/ac/ax, and any
//! multi-antenna SDR transmitter.
//!
//! Complex values are represented as `(f64, f64)` tuples `(re, im)` to keep
//! the module self-contained with no external crate dependencies.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::mimo_precoder::{MimoPrecoder, PrecodingType};
//!
//! // 2 TX antennas, 2 spatial streams, identity precoding
//! let precoder = MimoPrecoder::new(2, 2, PrecodingType::Identity);
//!
//! // Two streams, each carrying 3 symbols
//! let symbols = vec![
//!     vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)],
//!     vec![(0.0, -1.0), (1.0, 1.0), (-1.0, -1.0)],
//! ];
//!
//! let precoded = precoder.precode(&symbols);
//! assert_eq!(precoded.len(), 2); // 2 TX antennas
//! assert_eq!(precoded[0].len(), 3); // 3 time samples per antenna
//! ```

use std::f64::consts::PI;

/// Precoding type selection.
#[derive(Debug, Clone, PartialEq)]
pub enum PrecodingType {
    /// Identity precoding (open-loop spatial multiplexing, no CSI needed).
    Identity,
    /// Zero-forcing precoding: W = H^H (H H^H)^{-1}, nulls inter-stream interference.
    ZeroForcing,
    /// MMSE precoding: W = H^H (H H^H + sigma^2 I)^{-1}, trades interference for noise.
    MMSE {
        /// Noise variance (sigma^2).
        noise_var: f64,
    },
    /// Matched-filter / conjugate beamforming: W = H^H, maximises received SNR.
    MatchedFilter,
    /// DFT-based precoding (codebook-free, used in SC-FDMA / LTE uplink).
    DFT,
}

/// MIMO precoder that maps spatial streams to TX antenna ports.
///
/// The precoding matrix `W` has dimensions `num_tx x num_streams`. Given
/// `num_streams` input symbol vectors of length `N`, the output is `num_tx`
/// vectors of length `N` where:
///
/// ```text
/// output[tx][n] = sum_s W[tx][s] * symbols[s][n]
/// ```
#[derive(Debug, Clone)]
pub struct MimoPrecoder {
    /// Number of transmit antennas.
    pub num_tx: usize,
    /// Number of spatial streams.
    pub num_streams: usize,
    /// Precoding matrix W, dimensions [num_tx][num_streams], stored row-major.
    pub precoding_matrix: Vec<Vec<(f64, f64)>>,
}

impl MimoPrecoder {
    /// Create a new MIMO precoder.
    ///
    /// Builds the precoding matrix according to `precoding_type`. For types
    /// that require channel knowledge (`ZeroForcing`, `MMSE`, `MatchedFilter`),
    /// an identity-sized channel is assumed; call [`set_channel`] afterwards
    /// with the actual channel estimate.
    pub fn new(num_tx: usize, num_streams: usize, precoding_type: PrecodingType) -> Self {
        let precoding_matrix = match precoding_type {
            PrecodingType::Identity => identity_precoder(num_tx),
            PrecodingType::DFT => dft_precoder(num_tx),
            PrecodingType::MatchedFilter => {
                // Without a channel estimate, default to identity
                identity_precoder(num_tx)
            }
            PrecodingType::ZeroForcing => {
                // Without a channel estimate, default to identity
                identity_precoder(num_tx)
            }
            PrecodingType::MMSE { .. } => {
                // Without a channel estimate, default to identity
                identity_precoder(num_tx)
            }
        };

        // Trim or pad columns to match num_streams
        let precoding_matrix = precoding_matrix
            .into_iter()
            .map(|row| {
                let mut r = row;
                r.resize(num_streams, (0.0, 0.0));
                r
            })
            .collect::<Vec<_>>();

        // Ensure we have exactly num_tx rows
        let mut precoding_matrix = precoding_matrix;
        precoding_matrix.resize(num_tx, vec![(0.0, 0.0); num_streams]);

        MimoPrecoder {
            num_tx,
            num_streams,
            precoding_matrix,
        }
    }

    /// Apply precoding to input symbol streams.
    ///
    /// `symbols` has dimensions `[num_streams][N]` where `N` is the number of
    /// time-domain samples. Returns `[num_tx][N]` precoded output.
    ///
    /// Each output sample is computed as:
    /// ```text
    /// output[tx][n] = sum_{s=0}^{num_streams-1} W[tx][s] * symbols[s][n]
    /// ```
    pub fn precode(&self, symbols: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
        if symbols.is_empty() {
            return vec![vec![]; self.num_tx];
        }

        let n_samples = symbols[0].len();
        let mut output = vec![vec![(0.0, 0.0); n_samples]; self.num_tx];

        for tx in 0..self.num_tx {
            for n in 0..n_samples {
                let mut accum = (0.0, 0.0);
                for s in 0..self.num_streams.min(symbols.len()) {
                    if n < symbols[s].len() {
                        let prod = complex_mul(self.precoding_matrix[tx][s], symbols[s][n]);
                        accum.0 += prod.0;
                        accum.1 += prod.1;
                    }
                }
                output[tx][n] = accum;
            }
        }

        output
    }

    /// Recompute the precoding matrix from a channel estimate.
    ///
    /// `channel` has dimensions `[num_rx][num_tx]` representing the MIMO
    /// channel matrix H. The precoding matrix is recomputed according to the
    /// given `precoding_type`.
    pub fn set_channel(
        &mut self,
        channel: &[Vec<(f64, f64)>],
        precoding_type: PrecodingType,
    ) {
        let w = match precoding_type {
            PrecodingType::Identity => identity_precoder(self.num_tx),
            PrecodingType::DFT => dft_precoder(self.num_tx),
            PrecodingType::MatchedFilter => matched_filter_precoder(channel),
            PrecodingType::ZeroForcing => zero_forcing_precoder(channel),
            PrecodingType::MMSE { noise_var } => mmse_precoder(channel, noise_var),
        };

        // Fit to num_tx x num_streams
        self.precoding_matrix = w
            .into_iter()
            .map(|row| {
                let mut r = row;
                r.resize(self.num_streams, (0.0, 0.0));
                r
            })
            .collect();
        self.precoding_matrix
            .resize(self.num_tx, vec![(0.0, 0.0); self.num_streams]);
    }
}

// ---------------------------------------------------------------------------
// Precoder construction functions
// ---------------------------------------------------------------------------

/// Build an identity precoding matrix of size `num_tx x num_tx`.
pub fn identity_precoder(num_tx: usize) -> Vec<Vec<(f64, f64)>> {
    let mut m = vec![vec![(0.0, 0.0); num_tx]; num_tx];
    for i in 0..num_tx {
        m[i][i] = (1.0, 0.0);
    }
    m
}

/// Build a DFT precoding matrix of size `num_tx x num_tx`.
///
/// Entry `(k, n)` = `exp(-j 2 pi k n / N) / sqrt(N)`, which forms a unitary
/// matrix used for SC-FDMA-style precoding.
pub fn dft_precoder(num_tx: usize) -> Vec<Vec<(f64, f64)>> {
    let n = num_tx as f64;
    let scale = 1.0 / n.sqrt();
    let mut m = vec![vec![(0.0, 0.0); num_tx]; num_tx];
    for k in 0..num_tx {
        for j in 0..num_tx {
            let angle = -2.0 * PI * (k as f64) * (j as f64) / n;
            m[k][j] = (scale * angle.cos(), scale * angle.sin());
        }
    }
    m
}

/// Matched-filter (conjugate beamforming) precoder: W = H^H.
///
/// `channel` has dimensions `[num_rx][num_tx]`. Returns `[num_tx][num_rx]`.
pub fn matched_filter_precoder(channel: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
    matrix_hermitian(channel)
}

/// Zero-forcing precoder: W = H^H (H H^H)^{-1}.
///
/// `channel` has dimensions `[num_rx][num_tx]`.
/// Returns `[num_tx][num_rx]` (or `[num_tx][num_streams]` effectively).
fn zero_forcing_precoder(channel: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
    let h_h = matrix_hermitian(channel);
    let h_hh = matrix_mul_complex(channel, &h_h); // H H^H  [num_rx x num_rx]
    if let Some(inv) = matrix_inverse_complex(&h_hh) {
        matrix_mul_complex(&h_h, &inv) // H^H (H H^H)^{-1}
    } else {
        // Singular channel — fall back to matched filter
        h_h
    }
}

/// MMSE precoder: W = H^H (H H^H + sigma^2 I)^{-1}.
///
/// `channel` has dimensions `[num_rx][num_tx]`.
fn mmse_precoder(channel: &[Vec<(f64, f64)>], noise_var: f64) -> Vec<Vec<(f64, f64)>> {
    let h_h = matrix_hermitian(channel);
    let mut h_hh = matrix_mul_complex(channel, &h_h); // H H^H
    let nr = h_hh.len();
    // Add noise_var * I
    for i in 0..nr {
        if i < h_hh.len() && i < h_hh[i].len() {
            h_hh[i][i].0 += noise_var;
        }
    }
    if let Some(inv) = matrix_inverse_complex(&h_hh) {
        matrix_mul_complex(&h_h, &inv)
    } else {
        h_h
    }
}

// ---------------------------------------------------------------------------
// Linear algebra helpers (self-contained, no external crates)
// ---------------------------------------------------------------------------

/// Complex multiplication: `(a_re + j a_im)(b_re + j b_im)`.
#[inline]
pub fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex conjugate: `(re, -im)`.
#[inline]
pub fn complex_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Hermitian (conjugate transpose) of a complex matrix.
///
/// Input `[M][N]` → output `[N][M]` with each element conjugated.
pub fn matrix_hermitian(m: &[Vec<(f64, f64)>]) -> Vec<Vec<(f64, f64)>> {
    if m.is_empty() {
        return vec![];
    }
    let rows = m.len();
    let cols = m[0].len();
    let mut out = vec![vec![(0.0, 0.0); rows]; cols];
    for i in 0..rows {
        for j in 0..m[i].len() {
            out[j][i] = complex_conj(m[i][j]);
        }
    }
    out
}

/// Multiply two complex matrices: C = A * B.
///
/// A is `[M][K]`, B is `[K][N]`, result is `[M][N]`.
pub fn matrix_mul_complex(
    a: &[Vec<(f64, f64)>],
    b: &[Vec<(f64, f64)>],
) -> Vec<Vec<(f64, f64)>> {
    if a.is_empty() || b.is_empty() {
        return vec![];
    }
    let m = a.len();
    let k = a[0].len();
    let n = b[0].len();
    let mut c = vec![vec![(0.0, 0.0); n]; m];
    for i in 0..m {
        for j in 0..n {
            let mut sum = (0.0, 0.0);
            for p in 0..k.min(b.len()) {
                let prod = complex_mul(a[i][p], b[p][j]);
                sum.0 += prod.0;
                sum.1 += prod.1;
            }
            c[i][j] = sum;
        }
    }
    c
}

/// Invert a small complex square matrix using Gauss-Jordan elimination.
///
/// Returns `None` if the matrix is singular (or nearly so).
fn matrix_inverse_complex(m: &[Vec<(f64, f64)>]) -> Option<Vec<Vec<(f64, f64)>>> {
    let n = m.len();
    if n == 0 {
        return Some(vec![]);
    }
    // Check square
    for row in m {
        if row.len() != n {
            return None;
        }
    }

    // Build augmented matrix [M | I]
    let mut aug: Vec<Vec<(f64, f64)>> = Vec::with_capacity(n);
    for i in 0..n {
        let mut row = Vec::with_capacity(2 * n);
        row.extend_from_slice(&m[i]);
        for j in 0..n {
            row.push(if i == j { (1.0, 0.0) } else { (0.0, 0.0) });
        }
        aug.push(row);
    }

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot (largest magnitude)
        let mut max_mag = 0.0;
        let mut max_row = col;
        for row in col..n {
            let mag = aug[row][col].0 * aug[row][col].0 + aug[row][col].1 * aug[row][col].1;
            if mag > max_mag {
                max_mag = mag;
                max_row = row;
            }
        }
        if max_mag < 1e-30 {
            return None; // Singular
        }
        aug.swap(col, max_row);

        // Scale pivot row
        let pivot = aug[col][col];
        let pivot_mag_sq = pivot.0 * pivot.0 + pivot.1 * pivot.1;
        let pivot_inv = (pivot.0 / pivot_mag_sq, -pivot.1 / pivot_mag_sq);
        for j in 0..2 * n {
            aug[col][j] = complex_mul(aug[col][j], pivot_inv);
        }

        // Eliminate column in all other rows
        for row in 0..n {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            for j in 0..2 * n {
                let sub = complex_mul(factor, aug[col][j]);
                aug[row][j].0 -= sub.0;
                aug[row][j].1 -= sub.1;
            }
        }
    }

    // Extract inverse from right half
    let inv = aug
        .into_iter()
        .map(|row| row[n..].to_vec())
        .collect();
    Some(inv)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    fn approx_eq(a: (f64, f64), b: (f64, f64)) -> bool {
        (a.0 - b.0).abs() < EPS && (a.1 - b.1).abs() < EPS
    }

    #[test]
    fn test_identity_precoding() {
        let precoder = MimoPrecoder::new(2, 2, PrecodingType::Identity);
        let symbols = vec![
            vec![(1.0, 0.0), (0.0, 1.0)],
            vec![(0.0, -1.0), (1.0, 0.0)],
        ];
        let out = precoder.precode(&symbols);
        assert_eq!(out.len(), 2);
        // Identity: output should equal input
        assert!(approx_eq(out[0][0], (1.0, 0.0)));
        assert!(approx_eq(out[0][1], (0.0, 1.0)));
        assert!(approx_eq(out[1][0], (0.0, -1.0)));
        assert!(approx_eq(out[1][1], (1.0, 0.0)));
    }

    #[test]
    fn test_dft_precoding() {
        let precoder = MimoPrecoder::new(4, 4, PrecodingType::DFT);
        // DFT matrix should be unitary: W^H W = I
        let w = &precoder.precoding_matrix;
        let w_h = matrix_hermitian(w);
        let product = matrix_mul_complex(&w_h, w);
        for i in 0..4 {
            for j in 0..4 {
                let expected = if i == j { (1.0, 0.0) } else { (0.0, 0.0) };
                assert!(
                    approx_eq(product[i][j], expected),
                    "W^H W [{i}][{j}] = {:?}, expected {:?}",
                    product[i][j],
                    expected,
                );
            }
        }
    }

    #[test]
    fn test_matched_filter() {
        // 2x2 channel
        let channel = vec![
            vec![(1.0, 0.5), (0.3, -0.2)],
            vec![(0.7, 0.1), (0.9, -0.4)],
        ];
        let w = matched_filter_precoder(&channel);
        // W = H^H, so W[i][j] = conj(H[j][i])
        assert_eq!(w.len(), 2);
        assert_eq!(w[0].len(), 2);
        assert!(approx_eq(w[0][0], complex_conj(channel[0][0])));
        assert!(approx_eq(w[0][1], complex_conj(channel[1][0])));
        assert!(approx_eq(w[1][0], complex_conj(channel[0][1])));
        assert!(approx_eq(w[1][1], complex_conj(channel[1][1])));
    }

    #[test]
    fn test_precoding_dimensions() {
        // 4 TX, 2 streams
        let precoder = MimoPrecoder::new(4, 2, PrecodingType::Identity);
        assert_eq!(precoder.precoding_matrix.len(), 4);
        assert_eq!(precoder.precoding_matrix[0].len(), 2);

        let symbols = vec![
            vec![(1.0, 0.0); 10],
            vec![(0.0, 1.0); 10],
        ];
        let out = precoder.precode(&symbols);
        assert_eq!(out.len(), 4); // 4 TX outputs
        assert_eq!(out[0].len(), 10); // 10 samples each
    }

    #[test]
    fn test_single_stream() {
        // 4 TX, 1 stream — first TX gets the stream, others are zero
        let precoder = MimoPrecoder::new(4, 1, PrecodingType::Identity);
        let symbols = vec![vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0)]];
        let out = precoder.precode(&symbols);
        assert_eq!(out.len(), 4);
        // Only TX 0 should have signal
        assert!(approx_eq(out[0][0], (1.0, 0.0)));
        assert!(approx_eq(out[0][1], (0.0, 1.0)));
        // TX 1..3 should be zero
        for tx in 1..4 {
            for n in 0..3 {
                assert!(
                    approx_eq(out[tx][n], (0.0, 0.0)),
                    "TX {tx} sample {n} should be zero",
                );
            }
        }
    }

    #[test]
    fn test_full_rank() {
        // Full-rank 3x3 identity precoding
        let precoder = MimoPrecoder::new(3, 3, PrecodingType::Identity);
        let symbols = vec![
            vec![(1.0, 0.0)],
            vec![(0.0, 1.0)],
            vec![(-1.0, -1.0)],
        ];
        let out = precoder.precode(&symbols);
        assert!(approx_eq(out[0][0], (1.0, 0.0)));
        assert!(approx_eq(out[1][0], (0.0, 1.0)));
        assert!(approx_eq(out[2][0], (-1.0, -1.0)));
    }

    #[test]
    fn test_channel_update() {
        let mut precoder = MimoPrecoder::new(2, 2, PrecodingType::Identity);
        // Verify starts as identity
        assert!(approx_eq(precoder.precoding_matrix[0][0], (1.0, 0.0)));
        assert!(approx_eq(precoder.precoding_matrix[0][1], (0.0, 0.0)));

        // Update with a channel and matched filter
        let channel = vec![
            vec![(1.0, 0.0), (0.0, 1.0)],
            vec![(0.0, -1.0), (1.0, 0.0)],
        ];
        precoder.set_channel(&channel, PrecodingType::MatchedFilter);

        // After update, W = H^H
        assert!(approx_eq(precoder.precoding_matrix[0][0], (1.0, 0.0)));
        assert!(approx_eq(precoder.precoding_matrix[0][1], (0.0, 1.0))); // conj of (0,-1)
        assert!(approx_eq(precoder.precoding_matrix[1][0], (0.0, -1.0))); // conj of (0,1)
        assert!(approx_eq(precoder.precoding_matrix[1][1], (1.0, 0.0)));
    }

    #[test]
    fn test_matrix_hermitian() {
        let m = vec![
            vec![(1.0, 2.0), (3.0, 4.0)],
            vec![(5.0, 6.0), (7.0, 8.0)],
        ];
        let h = matrix_hermitian(&m);
        assert_eq!(h.len(), 2);
        assert_eq!(h[0].len(), 2);
        // H[0][0] = conj(M[0][0])
        assert!(approx_eq(h[0][0], (1.0, -2.0)));
        // H[0][1] = conj(M[1][0])
        assert!(approx_eq(h[0][1], (5.0, -6.0)));
        // H[1][0] = conj(M[0][1])
        assert!(approx_eq(h[1][0], (3.0, -4.0)));
        // H[1][1] = conj(M[1][1])
        assert!(approx_eq(h[1][1], (7.0, -8.0)));
    }

    #[test]
    fn test_matrix_mul() {
        // Multiply identity by arbitrary matrix
        let id = vec![
            vec![(1.0, 0.0), (0.0, 0.0)],
            vec![(0.0, 0.0), (1.0, 0.0)],
        ];
        let a = vec![
            vec![(2.0, 3.0), (4.0, -1.0)],
            vec![(0.5, 0.5), (1.0, 2.0)],
        ];
        let c = matrix_mul_complex(&id, &a);
        assert!(approx_eq(c[0][0], (2.0, 3.0)));
        assert!(approx_eq(c[0][1], (4.0, -1.0)));
        assert!(approx_eq(c[1][0], (0.5, 0.5)));
        assert!(approx_eq(c[1][1], (1.0, 2.0)));

        // Non-trivial: [1+j, 0] [1, 0] = [1+j, 0]
        //              [0, 1]   [0, 1]   [0,   1]
        let b = vec![
            vec![(1.0, 1.0), (0.0, 0.0)],
            vec![(0.0, 0.0), (1.0, 0.0)],
        ];
        let d = matrix_mul_complex(&b, &id);
        assert!(approx_eq(d[0][0], (1.0, 1.0)));
        assert!(approx_eq(d[1][1], (1.0, 0.0)));
    }

    #[test]
    fn test_empty_input() {
        let precoder = MimoPrecoder::new(2, 2, PrecodingType::Identity);
        let symbols: Vec<Vec<(f64, f64)>> = vec![];
        let out = precoder.precode(&symbols);
        assert_eq!(out.len(), 2);
        // Each TX output should be empty
        assert!(out[0].is_empty());
        assert!(out[1].is_empty());
    }
}
