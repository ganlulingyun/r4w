//! MIMO — Multi-Input Multi-Output space-time processing
//!
//! Implements Alamouti STBC (2x1 space-time block code), spatial
//! multiplexing with zero-forcing and MMSE demultiplexing, and
//! transmit beamforming. Essential for Wi-Fi, LTE/5G, and any
//! multi-antenna SDR system.
//! GNU Radio equivalent: `gr-mimo` (out-of-tree), 3GPP MIMO blocks.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::mimo::{AlamoutiEncoder, AlamoutiDecoder};
//! use num_complex::Complex64;
//!
//! let encoder = AlamoutiEncoder;
//! let symbols = vec![Complex64::new(1.0, 0.0), Complex64::new(0.0, 1.0)];
//! let encoded = encoder.encode(&symbols);
//! assert_eq!(encoded.len(), 2); // 2 time slots
//!
//! // Perfect channel
//! let h = [Complex64::new(1.0, 0.0), Complex64::new(1.0, 0.0)];
//! let decoder = AlamoutiDecoder;
//! let decoded = decoder.decode(&encoded, &h);
//! assert_eq!(decoded.len(), 2);
//! ```

use num_complex::Complex64;

/// Alamouti 2x1 Space-Time Block Code encoder.
///
/// Encodes pairs of symbols for 2 TX antennas over 2 time slots.
/// Provides full-rate, full-diversity for 2 TX, 1 RX.
#[derive(Debug, Clone, Copy)]
pub struct AlamoutiEncoder;

impl AlamoutiEncoder {
    /// Encode symbols into Alamouti STBC.
    ///
    /// Input: [s0, s1, s2, s3, ...]
    /// Output: [[tx0_t0, tx1_t0], [tx0_t1, tx1_t1], ...]
    /// where tx0_t0=s0, tx1_t0=s1, tx0_t1=-s1*, tx1_t1=s0*
    pub fn encode(&self, symbols: &[Complex64]) -> Vec<[Complex64; 2]> {
        let mut result = Vec::new();
        for pair in symbols.chunks(2) {
            if pair.len() < 2 {
                break;
            }
            let s0 = pair[0];
            let s1 = pair[1];
            // Time slot 0: [s0, s1]
            result.push([s0, s1]);
            // Time slot 1: [-s1*, s0*]
            result.push([-s1.conj(), s0.conj()]);
        }
        result
    }
}

/// Alamouti 2x1 STBC decoder.
#[derive(Debug, Clone, Copy)]
pub struct AlamoutiDecoder;

impl AlamoutiDecoder {
    /// Decode Alamouti STBC with known channel.
    ///
    /// `encoded`: pairs of [tx0, tx1] for each time slot
    /// `h`: channel coefficients [h0, h1] for 2 TX antennas
    pub fn decode(
        &self,
        encoded: &[[Complex64; 2]],
        h: &[Complex64; 2],
    ) -> Vec<Complex64> {
        let mut result = Vec::new();
        let h0 = h[0];
        let h1 = h[1];
        let norm = h0.norm_sqr() + h1.norm_sqr();

        for pair in encoded.chunks(2) {
            if pair.len() < 2 {
                break;
            }
            // Received: r0 = h0*s0 + h1*s1, r1 = -h0*s1* + h1*s0*
            let r0 = h0 * pair[0][0] + h1 * pair[0][1];
            let r1 = h0 * pair[1][0] + h1 * pair[1][1];

            // Alamouti combining
            let s0_hat = (h0.conj() * r0 + h1 * r1.conj()) / norm;
            let s1_hat = (h1.conj() * r0 - h0 * r1.conj()) / norm;

            result.push(s0_hat);
            result.push(s1_hat);
        }
        result
    }
}

/// Channel matrix for MIMO (Nr x Nt).
#[derive(Debug, Clone)]
pub struct ChannelMatrix {
    /// Number of receive antennas.
    pub nr: usize,
    /// Number of transmit antennas.
    pub nt: usize,
    /// Flat storage (row-major: nr rows, nt cols).
    pub data: Vec<Complex64>,
}

impl ChannelMatrix {
    /// Create from flat data.
    pub fn from_flat(nr: usize, nt: usize, data: Vec<Complex64>) -> Self {
        assert_eq!(data.len(), nr * nt);
        Self { nr, nt, data }
    }

    /// Create identity-like channel (nr x nt).
    pub fn identity(n: usize) -> Self {
        let mut data = vec![Complex64::new(0.0, 0.0); n * n];
        for i in 0..n {
            data[i * n + i] = Complex64::new(1.0, 0.0);
        }
        Self {
            nr: n,
            nt: n,
            data,
        }
    }

    /// Get element H[r][c].
    pub fn get(&self, r: usize, c: usize) -> Complex64 {
        self.data[r * self.nt + c]
    }

    /// Set element H[r][c].
    pub fn set(&mut self, r: usize, c: usize, val: Complex64) {
        self.data[r * self.nt + c] = val;
    }

    /// Compute H^H (conjugate transpose).
    pub fn hermitian(&self) -> ChannelMatrix {
        let mut data = vec![Complex64::new(0.0, 0.0); self.nt * self.nr];
        for r in 0..self.nr {
            for c in 0..self.nt {
                data[c * self.nr + r] = self.get(r, c).conj();
            }
        }
        ChannelMatrix {
            nr: self.nt,
            nt: self.nr,
            data,
        }
    }

    /// Matrix-vector multiply: H * x.
    pub fn mul_vec(&self, x: &[Complex64]) -> Vec<Complex64> {
        assert_eq!(x.len(), self.nt);
        let mut result = vec![Complex64::new(0.0, 0.0); self.nr];
        for r in 0..self.nr {
            for c in 0..self.nt {
                result[r] += self.get(r, c) * x[c];
            }
        }
        result
    }

    /// Matrix multiply: self * other.
    pub fn mul_mat(&self, other: &ChannelMatrix) -> ChannelMatrix {
        assert_eq!(self.nt, other.nr);
        let mut data = vec![Complex64::new(0.0, 0.0); self.nr * other.nt];
        for r in 0..self.nr {
            for c in 0..other.nt {
                for k in 0..self.nt {
                    data[r * other.nt + c] += self.get(r, k) * other.get(k, c);
                }
            }
        }
        ChannelMatrix {
            nr: self.nr,
            nt: other.nt,
            data,
        }
    }

    /// Frobenius norm.
    pub fn frobenius_norm(&self) -> f64 {
        self.data.iter().map(|x| x.norm_sqr()).sum::<f64>().sqrt()
    }

    /// Condition number approximation (max_diag / min_diag of H^H * H).
    pub fn condition_number(&self) -> f64 {
        let hh = self.hermitian().mul_mat(self);
        let n = hh.nr;
        let diags: Vec<f64> = (0..n).map(|i| hh.get(i, i).re).collect();
        let max_d = diags.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min_d = diags.iter().cloned().fold(f64::INFINITY, f64::min);
        if min_d > 1e-30 {
            (max_d / min_d).sqrt()
        } else {
            f64::INFINITY
        }
    }

    /// MIMO capacity: log2(det(I + SNR/Nt * H * H^H)) bits/s/Hz.
    pub fn capacity(&self, snr_linear: f64) -> f64 {
        let n = self.nr;
        let hhh = self.mul_mat(&self.hermitian());

        // For 2x2, compute det(I + alpha * M) directly
        if n == 2 {
            let alpha = snr_linear / self.nt as f64;
            let a = 1.0 + alpha * hhh.get(0, 0).re;
            let d = 1.0 + alpha * hhh.get(1, 1).re;
            let bc = alpha * alpha * (hhh.get(0, 1) * hhh.get(1, 0)).re;
            let det = a * d - bc;
            return det.max(1e-30).log2();
        }

        // For 1x1
        if n == 1 {
            return (1.0 + snr_linear * hhh.get(0, 0).re / self.nt as f64).log2();
        }

        // General: sum of log2(1 + snr/nt * eigenvalue) using trace approximation
        let alpha = snr_linear / self.nt as f64;
        let mut sum = 0.0;
        for i in 0..n {
            sum += (1.0 + alpha * hhh.get(i, i).re).log2();
        }
        sum
    }
}

/// Zero-Forcing spatial demultiplexer.
///
/// W_ZF = (H^H * H)^(-1) * H^H
pub fn zf_demux(rx: &[Complex64], h: &ChannelMatrix) -> Vec<Complex64> {
    let hh = h.hermitian();
    let hhh = hh.mul_mat(h);
    let n = hhh.nr;

    // Invert H^H * H (for small matrices)
    if n == 2 {
        let inv = invert_2x2(&hhh);
        let w = inv.mul_mat(&hh);
        return w.mul_vec(rx);
    }

    // Fallback: diagonal approximation
    let mut result = vec![Complex64::new(0.0, 0.0); n];
    let hh_rx = hh.mul_vec(rx);
    for i in 0..n {
        let diag = hhh.get(i, i);
        if diag.norm_sqr() > 1e-30 {
            result[i] = hh_rx[i] / diag;
        }
    }
    result
}

/// MMSE spatial demultiplexer.
///
/// W_MMSE = (H^H * H + (1/SNR) * I)^(-1) * H^H
pub fn mmse_demux(rx: &[Complex64], h: &ChannelMatrix, snr_linear: f64) -> Vec<Complex64> {
    let hh = h.hermitian();
    let mut hhh = hh.mul_mat(h);
    let n = hhh.nr;
    let noise = 1.0 / snr_linear.max(1e-30);

    // Add noise regularization to diagonal
    for i in 0..n {
        let val = hhh.get(i, i) + Complex64::new(noise, 0.0);
        hhh.set(i, i, val);
    }

    if n == 2 {
        let inv = invert_2x2(&hhh);
        let w = inv.mul_mat(&hh);
        return w.mul_vec(rx);
    }

    // Fallback: diagonal approximation
    let mut result = vec![Complex64::new(0.0, 0.0); n];
    let hh_rx = hh.mul_vec(rx);
    for i in 0..n {
        let diag = hhh.get(i, i);
        if diag.norm_sqr() > 1e-30 {
            result[i] = hh_rx[i] / diag;
        }
    }
    result
}

/// Maximum Ratio Transmission beamforming weights.
///
/// For MISO: w = h* / ||h|| (conjugate of channel, normalized).
pub fn mrt_weights(h: &[Complex64]) -> Vec<Complex64> {
    let norm: f64 = h.iter().map(|x| x.norm_sqr()).sum::<f64>().sqrt();
    if norm < 1e-30 {
        return vec![Complex64::new(0.0, 0.0); h.len()];
    }
    h.iter().map(|&x| x.conj() / norm).collect()
}

/// Invert a 2x2 complex matrix.
fn invert_2x2(m: &ChannelMatrix) -> ChannelMatrix {
    assert_eq!(m.nr, 2);
    assert_eq!(m.nt, 2);
    let a = m.get(0, 0);
    let b = m.get(0, 1);
    let c = m.get(1, 0);
    let d = m.get(1, 1);
    let det = a * d - b * c;
    let inv_det = if det.norm_sqr() > 1e-30 {
        Complex64::new(1.0, 0.0) / det
    } else {
        Complex64::new(0.0, 0.0)
    };
    ChannelMatrix::from_flat(
        2,
        2,
        vec![d * inv_det, -b * inv_det, -c * inv_det, a * inv_det],
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_alamouti_roundtrip() {
        let encoder = AlamoutiEncoder;
        let symbols = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 1.0),
            Complex64::new(-1.0, 0.0),
            Complex64::new(0.0, -1.0),
        ];

        let encoded = encoder.encode(&symbols);
        assert_eq!(encoded.len(), 4); // 2 pairs * 2 time slots

        let h = [Complex64::new(1.0, 0.0), Complex64::new(1.0, 0.0)];
        let decoder = AlamoutiDecoder;
        let decoded = decoder.decode(&encoded, &h);
        assert_eq!(decoded.len(), 4);

        for (orig, dec) in symbols.iter().zip(decoded.iter()) {
            assert!(
                (orig - dec).norm() < 1e-10,
                "Mismatch: {:?} vs {:?}",
                orig,
                dec
            );
        }
    }

    #[test]
    fn test_alamouti_with_channel() {
        let encoder = AlamoutiEncoder;
        let symbols = vec![Complex64::new(1.0, 1.0), Complex64::new(-1.0, 1.0)];
        let encoded = encoder.encode(&symbols);

        let h = [Complex64::new(0.8, 0.2), Complex64::new(0.6, -0.3)];
        let decoder = AlamoutiDecoder;
        let decoded = decoder.decode(&encoded, &h);

        for (orig, dec) in symbols.iter().zip(decoded.iter()) {
            assert!(
                (orig - dec).norm() < 1e-10,
                "Channel mismatch: {:?} vs {:?}",
                orig,
                dec
            );
        }
    }

    #[test]
    fn test_channel_matrix_identity() {
        let h = ChannelMatrix::identity(2);
        assert_eq!(h.get(0, 0), Complex64::new(1.0, 0.0));
        assert_eq!(h.get(0, 1), Complex64::new(0.0, 0.0));
        assert_eq!(h.get(1, 1), Complex64::new(1.0, 0.0));
    }

    #[test]
    fn test_condition_number_identity() {
        let h = ChannelMatrix::identity(2);
        let cn = h.condition_number();
        assert!((cn - 1.0).abs() < 1e-10, "Identity condition number should be 1.0, got {}", cn);
    }

    #[test]
    fn test_capacity_identity_channel() {
        let h = ChannelMatrix::identity(2);
        let snr_db = 10.0;
        let snr_lin = 10.0f64.powf(snr_db / 10.0);
        let cap = h.capacity(snr_lin);
        // 2x2 identity at 10dB: 2 * log2(1 + 10/2) = 2 * log2(6) ≈ 5.17 bits/s/Hz
        assert!(cap > 4.0, "Capacity should be > 4 bits/s/Hz, got {}", cap);
    }

    #[test]
    fn test_zf_demux_identity() {
        let h = ChannelMatrix::identity(2);
        let tx = vec![Complex64::new(1.0, 0.0), Complex64::new(0.0, 1.0)];
        let rx = h.mul_vec(&tx);
        let decoded = zf_demux(&rx, &h);

        for (orig, dec) in tx.iter().zip(decoded.iter()) {
            assert!(
                (orig - dec).norm() < 1e-10,
                "ZF mismatch: {:?} vs {:?}",
                orig,
                dec
            );
        }
    }

    #[test]
    fn test_mmse_demux_identity() {
        let h = ChannelMatrix::identity(2);
        let tx = vec![Complex64::new(1.0, 0.0), Complex64::new(0.0, 1.0)];
        let rx = h.mul_vec(&tx);
        let decoded = mmse_demux(&rx, &h, 100.0);

        for (orig, dec) in tx.iter().zip(decoded.iter()) {
            assert!(
                (orig - dec).norm() < 0.1,
                "MMSE mismatch: {:?} vs {:?}",
                orig,
                dec
            );
        }
    }

    #[test]
    fn test_mrt_weights() {
        let h = vec![Complex64::new(1.0, 0.0), Complex64::new(0.0, 1.0)];
        let w = mrt_weights(&h);
        assert_eq!(w.len(), 2);
        // Weights should be normalized
        let norm: f64 = w.iter().map(|x| x.norm_sqr()).sum::<f64>().sqrt();
        assert!((norm - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_hermitian() {
        let h = ChannelMatrix::from_flat(
            2,
            2,
            vec![
                Complex64::new(1.0, 2.0),
                Complex64::new(3.0, 4.0),
                Complex64::new(5.0, 6.0),
                Complex64::new(7.0, 8.0),
            ],
        );
        let hh = h.hermitian();
        assert_eq!(hh.nr, 2);
        assert_eq!(hh.nt, 2);
        assert_eq!(hh.get(0, 0), Complex64::new(1.0, -2.0));
        assert_eq!(hh.get(0, 1), Complex64::new(5.0, -6.0));
    }

    #[test]
    fn test_frobenius_norm() {
        let h = ChannelMatrix::identity(2);
        let norm = h.frobenius_norm();
        assert!((norm - 2.0f64.sqrt()).abs() < 1e-10);
    }

    #[test]
    fn test_mul_vec() {
        let h = ChannelMatrix::identity(2);
        let x = vec![Complex64::new(3.0, 4.0), Complex64::new(1.0, 2.0)];
        let y = h.mul_vec(&x);
        assert_eq!(y[0], x[0]);
        assert_eq!(y[1], x[1]);
    }
}
