//! Alamouti Space-Time Block Code (STBC)
//!
//! Implements the Alamouti scheme for 2-transmit-antenna diversity, supporting
//! both 2x1 MISO (one receive antenna) and 2x2 MIMO (two receive antennas)
//! configurations. The Alamouti code achieves full transmit diversity (order 2)
//! at rate 1 (no bandwidth penalty) using orthogonal space-time coding.
//!
//! The encoding matrix for each pair of symbols (s0, s1) over two time slots is:
//!
//! ```text
//!            Antenna 0      Antenna 1
//! Time 0:      s0              s1
//! Time 1:    -conj(s1)       conj(s0)
//! ```
//!
//! The decoder uses maximum-likelihood combining with channel estimates to
//! recover the original symbols with full diversity gain.
//!
//! Complex values are represented as `(f64, f64)` tuples (real, imaginary) to
//! keep the module self-contained with no external dependencies.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::alamouti_codec::{AlamoutiEncoder, AlamoutiDecoder};
//!
//! let encoder = AlamoutiEncoder::new();
//! let symbols = vec![(1.0, 0.0), (0.0, 1.0)];
//! let (ant0, ant1) = encoder.encode(&symbols);
//!
//! // Simulate flat channel with known gains
//! let h0 = vec![(1.0, 0.0); 2]; // channel from TX0 to RX
//! let h1 = vec![(1.0, 0.0); 2]; // channel from TX1 to RX
//!
//! // Received signal: r = h0 * ant0 + h1 * ant1 (no noise)
//! let rx: Vec<(f64, f64)> = ant0.iter().zip(ant1.iter())
//!     .zip(h0.iter().zip(h1.iter()))
//!     .map(|((a0, a1), (c0, c1))| complex_add(complex_mul(*c0, *a0), complex_mul(*c1, *a1)))
//!     .collect();
//!
//! use r4w_core::alamouti_codec::{complex_add, complex_mul};
//!
//! let decoder = AlamoutiDecoder::new();
//! let recovered = decoder.decode(&rx, &h0, &h1);
//! assert!((recovered[0].0 - 1.0).abs() < 1e-10);
//! assert!((recovered[1].1 - 1.0).abs() < 1e-10);
//! ```

// ---------------------------------------------------------------------------
// Complex arithmetic helpers
// ---------------------------------------------------------------------------

/// Compute the complex conjugate of `z`: (a, b) -> (a, -b).
#[inline]
pub fn complex_conj(z: (f64, f64)) -> (f64, f64) {
    (z.0, -z.1)
}

/// Multiply two complex numbers: (a+bi)(c+di) = (ac-bd, ad+bc).
#[inline]
pub fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Add two complex numbers element-wise.
#[inline]
pub fn complex_add(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Subtract two complex numbers element-wise: a - b.
#[inline]
fn complex_sub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Squared magnitude |z|^2 = a^2 + b^2.
#[inline]
fn complex_mag_sq(z: (f64, f64)) -> f64 {
    z.0 * z.0 + z.1 * z.1
}

/// Scale a complex number by a real scalar.
#[inline]
fn complex_scale(z: (f64, f64), s: f64) -> (f64, f64) {
    (z.0 * s, z.1 * s)
}

/// Negate a complex number.
#[inline]
fn complex_neg(z: (f64, f64)) -> (f64, f64) {
    (-z.0, -z.1)
}

// ---------------------------------------------------------------------------
// Alamouti Encoder (2 TX antennas)
// ---------------------------------------------------------------------------

/// Alamouti space-time block encoder for 2 transmit antennas.
///
/// Encodes pairs of complex symbols into two antenna streams over two time
/// slots, providing full-rate transmit diversity.
#[derive(Debug, Clone)]
pub struct AlamoutiEncoder {
    /// Number of transmit antennas (always 2 for Alamouti).
    pub num_tx_antennas: usize,
}

impl AlamoutiEncoder {
    /// Create a new Alamouti encoder with 2 TX antennas.
    pub fn new() -> Self {
        Self { num_tx_antennas: 2 }
    }

    /// Encode a slice of complex symbols into two antenna streams.
    ///
    /// Symbols are processed in pairs. If the input length is odd, a zero
    /// symbol is appended internally to complete the final pair.
    ///
    /// Returns `(antenna0_stream, antenna1_stream)` where each stream has
    /// the same length as the (possibly padded) input.
    ///
    /// # Encoding rule
    ///
    /// For each pair (s0, s1):
    /// - Time 0: antenna0 = s0,         antenna1 = s1
    /// - Time 1: antenna0 = -conj(s1),  antenna1 = conj(s0)
    pub fn encode(&self, symbols: &[(f64, f64)]) -> (Vec<(f64, f64)>, Vec<(f64, f64)>) {
        if symbols.is_empty() {
            return (Vec::new(), Vec::new());
        }

        // Pad to even length if necessary.
        let padded: Vec<(f64, f64)> = if symbols.len() % 2 != 0 {
            let mut v = symbols.to_vec();
            v.push((0.0, 0.0));
            v
        } else {
            symbols.to_vec()
        };

        let n = padded.len();
        let mut ant0 = Vec::with_capacity(n);
        let mut ant1 = Vec::with_capacity(n);

        for pair in padded.chunks_exact(2) {
            let s0 = pair[0];
            let s1 = pair[1];

            // Time slot 0
            ant0.push(s0);
            ant1.push(s1);

            // Time slot 1
            ant0.push(complex_neg(complex_conj(s1))); // -conj(s1)
            ant1.push(complex_conj(s0)); // conj(s0)
        }

        (ant0, ant1)
    }
}

impl Default for AlamoutiEncoder {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Alamouti Decoder — 2x1 MISO (single receive antenna)
// ---------------------------------------------------------------------------

/// Alamouti maximum-likelihood decoder for 2x1 MISO.
///
/// Given the received signal and channel estimates from both transmit
/// antennas, recovers the original symbol pairs using the orthogonality of
/// the Alamouti code.
#[derive(Debug, Clone)]
pub struct AlamoutiDecoder;

impl AlamoutiDecoder {
    /// Create a new 2x1 Alamouti decoder.
    pub fn new() -> Self {
        Self
    }

    /// Decode received samples using channel estimates.
    ///
    /// # Arguments
    ///
    /// * `rx`  - Received signal samples (must have even length).
    /// * `h0`  - Channel estimates from TX antenna 0 to the RX antenna,
    ///           one per time slot (same length as `rx`).
    /// * `h1`  - Channel estimates from TX antenna 1 to the RX antenna.
    ///
    /// # Decoding rule
    ///
    /// For each received pair (r0, r1):
    /// ```text
    /// s0_hat = (conj(h0) * r0 + h1 * conj(r1)) / (|h0|^2 + |h1|^2)
    /// s1_hat = (conj(h1) * r0 - h0 * conj(r1)) / (|h0|^2 + |h1|^2)
    /// ```
    ///
    /// # Panics
    ///
    /// Panics if `rx`, `h0`, and `h1` do not all have the same even length.
    pub fn decode(
        &self,
        rx: &[(f64, f64)],
        h0: &[(f64, f64)],
        h1: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        assert_eq!(rx.len(), h0.len());
        assert_eq!(rx.len(), h1.len());

        if rx.is_empty() {
            return Vec::new();
        }

        assert!(
            rx.len() % 2 == 0,
            "received sample count must be even for Alamouti decoding"
        );

        let num_pairs = rx.len() / 2;
        let mut decoded = Vec::with_capacity(rx.len());

        for i in 0..num_pairs {
            let r0 = rx[2 * i];
            let r1 = rx[2 * i + 1];
            let c0 = h0[2 * i]; // channel h0 at time 0
            let c1 = h1[2 * i]; // channel h1 at time 0

            let norm = complex_mag_sq(c0) + complex_mag_sq(c1);

            // s0_hat = conj(h0)*r0 + h1*conj(r1)
            let s0_hat = complex_add(
                complex_mul(complex_conj(c0), r0),
                complex_mul(c1, complex_conj(r1)),
            );

            // s1_hat = conj(h1)*r0 - h0*conj(r1)
            let s1_hat = complex_sub(
                complex_mul(complex_conj(c1), r0),
                complex_mul(c0, complex_conj(r1)),
            );

            if norm > 0.0 {
                decoded.push(complex_scale(s0_hat, 1.0 / norm));
                decoded.push(complex_scale(s1_hat, 1.0 / norm));
            } else {
                decoded.push((0.0, 0.0));
                decoded.push((0.0, 0.0));
            }
        }

        decoded
    }
}

impl Default for AlamoutiDecoder {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Alamouti2x2Decoder — 2x2 MIMO (two receive antennas)
// ---------------------------------------------------------------------------

/// Alamouti maximum-likelihood decoder for 2x2 MIMO.
///
/// Combines signals from two receive antennas for enhanced diversity gain
/// (order 4) using the same Alamouti orthogonal structure.
#[derive(Debug, Clone)]
pub struct Alamouti2x2Decoder;

impl Alamouti2x2Decoder {
    /// Create a new 2x2 Alamouti decoder.
    pub fn new() -> Self {
        Self
    }

    /// Decode received samples from two RX antennas.
    ///
    /// # Arguments
    ///
    /// * `rx0` - Received samples at RX antenna 0.
    /// * `rx1` - Received samples at RX antenna 1.
    /// * `h00` - Channel from TX0 to RX0.
    /// * `h01` - Channel from TX1 to RX0.
    /// * `h10` - Channel from TX0 to RX1.
    /// * `h11` - Channel from TX1 to RX1.
    ///
    /// All slices must have the same even length.
    ///
    /// # Decoding rule
    ///
    /// The 2x2 decoder extends the 2x1 combining by summing the ML
    /// statistics from both receive antennas:
    ///
    /// ```text
    /// s0_hat = (conj(h00)*r0_0 + h01*conj(r1_0) + conj(h10)*r0_1 + h11*conj(r1_1))
    ///          / (|h00|^2 + |h01|^2 + |h10|^2 + |h11|^2)
    /// ```
    pub fn decode(
        &self,
        rx0: &[(f64, f64)],
        rx1: &[(f64, f64)],
        h00: &[(f64, f64)],
        h01: &[(f64, f64)],
        h10: &[(f64, f64)],
        h11: &[(f64, f64)],
    ) -> Vec<(f64, f64)> {
        let n = rx0.len();
        assert_eq!(n, rx1.len());
        assert_eq!(n, h00.len());
        assert_eq!(n, h01.len());
        assert_eq!(n, h10.len());
        assert_eq!(n, h11.len());

        if n == 0 {
            return Vec::new();
        }

        assert!(
            n % 2 == 0,
            "received sample count must be even for Alamouti decoding"
        );

        let num_pairs = n / 2;
        let mut decoded = Vec::with_capacity(n);

        for i in 0..num_pairs {
            let r0_a = rx0[2 * i];
            let r1_a = rx0[2 * i + 1];
            let r0_b = rx1[2 * i];
            let r1_b = rx1[2 * i + 1];

            let c00 = h00[2 * i]; // TX0 -> RX0
            let c01 = h01[2 * i]; // TX1 -> RX0
            let c10 = h10[2 * i]; // TX0 -> RX1
            let c11 = h11[2 * i]; // TX1 -> RX1

            let norm = complex_mag_sq(c00)
                + complex_mag_sq(c01)
                + complex_mag_sq(c10)
                + complex_mag_sq(c11);

            // s0: combine from both RX antennas
            let s0_hat = complex_add(
                complex_add(
                    complex_mul(complex_conj(c00), r0_a),
                    complex_mul(c01, complex_conj(r1_a)),
                ),
                complex_add(
                    complex_mul(complex_conj(c10), r0_b),
                    complex_mul(c11, complex_conj(r1_b)),
                ),
            );

            // s1: combine from both RX antennas
            let s1_hat = complex_add(
                complex_sub(
                    complex_mul(complex_conj(c01), r0_a),
                    complex_mul(c00, complex_conj(r1_a)),
                ),
                complex_sub(
                    complex_mul(complex_conj(c11), r0_b),
                    complex_mul(c10, complex_conj(r1_b)),
                ),
            );

            if norm > 0.0 {
                decoded.push(complex_scale(s0_hat, 1.0 / norm));
                decoded.push(complex_scale(s1_hat, 1.0 / norm));
            } else {
                decoded.push((0.0, 0.0));
                decoded.push((0.0, 0.0));
            }
        }

        decoded
    }
}

impl Default for Alamouti2x2Decoder {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Diversity gain
// ---------------------------------------------------------------------------

/// Compute the theoretical BER improvement factor from Alamouti diversity.
///
/// For BPSK over Rayleigh fading, the average BER without diversity is
/// approximately `1 / (4 * SNR_linear)` at high SNR, and with L-branch MRC
/// diversity it scales as `C(2L-1, L) / (4*SNR)^L`, giving a dramatic BER
/// reduction.
///
/// This function returns the ratio `BER_no_diversity / BER_with_diversity`,
/// which represents the factor by which BER is improved through diversity
/// combining. Higher values mean more improvement.
///
/// # Arguments
///
/// * `snr_db` - Signal-to-noise ratio in decibels.
/// * `num_antennas` - Diversity order (total number of independent branches).
///
/// # Returns
///
/// The diversity gain factor (>= 1.0). Returns 1.0 for a single antenna.
pub fn diversity_gain(snr_db: f64, num_antennas: usize) -> f64 {
    if num_antennas <= 1 {
        return 1.0;
    }

    let snr_linear = 10.0_f64.powf(snr_db / 10.0);

    // High-SNR approximation for Rayleigh fading BER:
    //   BER_1 ~ 1 / (4 * gamma)
    //   BER_L ~ C(2L-1, L) / (4 * gamma)^L
    // Gain = BER_1 / BER_L = (4*gamma)^(L-1) / C(2L-1, L)
    let l = num_antennas as f64;
    let binom = binom_central(num_antennas);
    let gain = (4.0 * snr_linear).powf(l - 1.0) / binom;

    gain.max(1.0)
}

/// Central binomial-like coefficient C(2L-1, L) used in Rayleigh BER formula.
fn binom_central(l: usize) -> f64 {
    let n = 2 * l - 1;
    let mut result = 1.0_f64;
    for i in 0..l {
        result *= (n - i) as f64 / (i + 1) as f64;
    }
    result
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-10;

    fn assert_complex_eq(a: (f64, f64), b: (f64, f64), tol: f64) {
        assert!(
            (a.0 - b.0).abs() < tol && (a.1 - b.1).abs() < tol,
            "expected ({}, {}), got ({}, {})",
            b.0,
            b.1,
            a.0,
            a.1
        );
    }

    #[test]
    fn test_complex_helpers() {
        // Conjugate
        assert_eq!(complex_conj((3.0, 4.0)), (3.0, -4.0));
        assert_eq!(complex_conj((0.0, 0.0)), (0.0, 0.0));

        // Multiply: (1+2i)(3+4i) = (3-8) + (4+6)i = (-5, 10)
        assert_complex_eq(complex_mul((1.0, 2.0), (3.0, 4.0)), (-5.0, 10.0), EPS);

        // Multiply by conjugate: (a)(conj(a)) = |a|^2
        let a = (3.0, 4.0);
        let result = complex_mul(a, complex_conj(a));
        assert!((result.0 - 25.0).abs() < EPS);
        assert!(result.1.abs() < EPS);

        // Add
        assert_eq!(complex_add((1.0, 2.0), (3.0, 4.0)), (4.0, 6.0));
    }

    #[test]
    fn test_encode_basic() {
        let enc = AlamoutiEncoder::new();
        assert_eq!(enc.num_tx_antennas, 2);

        let symbols = vec![(1.0, 0.0), (0.0, 1.0)];
        let (ant0, ant1) = enc.encode(&symbols);

        assert_eq!(ant0.len(), 2);
        assert_eq!(ant1.len(), 2);

        // Time 0: ant0 = s0 = (1,0), ant1 = s1 = (0,1)
        assert_complex_eq(ant0[0], (1.0, 0.0), EPS);
        assert_complex_eq(ant1[0], (0.0, 1.0), EPS);

        // Time 1: ant0 = -conj(s1) = -(0,-1) = (0,1), ant1 = conj(s0) = (1,0)
        assert_complex_eq(ant0[1], (0.0, 1.0), EPS);
        assert_complex_eq(ant1[1], (1.0, 0.0), EPS);
    }

    #[test]
    fn test_encode_decode_roundtrip() {
        let enc = AlamoutiEncoder::new();
        let dec = AlamoutiDecoder::new();

        let symbols = vec![(1.0, 0.5), (-0.3, 0.7), (0.8, -0.2), (-1.0, -0.5)];
        let (ant0, ant1) = enc.encode(&symbols);

        // Perfect channel (identity, no fading)
        let h0: Vec<(f64, f64)> = vec![(1.0, 0.0); ant0.len()];
        let h1: Vec<(f64, f64)> = vec![(0.0, 0.0); ant0.len()]; // only TX0 reaches RX

        // Received = h0 * ant0 + h1 * ant1 = ant0 (since h0=1, h1=0)
        let rx: Vec<(f64, f64)> = ant0
            .iter()
            .zip(ant1.iter())
            .zip(h0.iter().zip(h1.iter()))
            .map(|((a0, a1), (c0, c1))| {
                complex_add(complex_mul(*c0, *a0), complex_mul(*c1, *a1))
            })
            .collect();

        let recovered = dec.decode(&rx, &h0, &h1);

        assert_eq!(recovered.len(), symbols.len());
        for (orig, rec) in symbols.iter().zip(recovered.iter()) {
            assert_complex_eq(*rec, *orig, EPS);
        }
    }

    #[test]
    fn test_orthogonality() {
        // The Alamouti code is orthogonal: the inner product of the two
        // antenna streams (across a symbol pair) should be zero.
        let enc = AlamoutiEncoder::new();
        let symbols = vec![(0.7, 0.3), (-0.5, 0.9)];
        let (ant0, ant1) = enc.encode(&symbols);

        // Inner product = sum of ant0[t] * conj(ant1[t])
        let mut inner = (0.0, 0.0);
        for t in 0..ant0.len() {
            inner = complex_add(inner, complex_mul(ant0[t], complex_conj(ant1[t])));
        }

        assert!(
            inner.0.abs() < EPS && inner.1.abs() < EPS,
            "Antenna streams should be orthogonal, got ({}, {})",
            inner.0,
            inner.1
        );
    }

    #[test]
    fn test_with_flat_channel() {
        let enc = AlamoutiEncoder::new();
        let dec = AlamoutiDecoder::new();

        let symbols = vec![(1.0, 0.0), (0.0, 1.0)];
        let (ant0, ant1) = enc.encode(&symbols);

        // Flat channel with both paths active
        let h0 = vec![(0.8, 0.1); ant0.len()];
        let h1 = vec![(0.5, -0.3); ant0.len()];

        let rx: Vec<(f64, f64)> = ant0
            .iter()
            .zip(ant1.iter())
            .zip(h0.iter().zip(h1.iter()))
            .map(|((a0, a1), (c0, c1))| {
                complex_add(complex_mul(*c0, *a0), complex_mul(*c1, *a1))
            })
            .collect();

        let recovered = dec.decode(&rx, &h0, &h1);

        assert_eq!(recovered.len(), 2);
        for (orig, rec) in symbols.iter().zip(recovered.iter()) {
            assert_complex_eq(*rec, *orig, EPS);
        }
    }

    #[test]
    fn test_with_fading_channel() {
        let enc = AlamoutiEncoder::new();
        let dec = AlamoutiDecoder::new();

        let symbols = vec![
            (0.707, 0.707),
            (-0.707, 0.707),
            (0.707, -0.707),
            (-0.707, -0.707),
        ];
        let (ant0, ant1) = enc.encode(&symbols);

        // Fading channel: different gains per symbol pair (but constant within a pair)
        let h0 = vec![
            (0.3, 0.5),
            (0.3, 0.5), // pair 0
            (0.9, -0.2),
            (0.9, -0.2), // pair 1
        ];
        let h1 = vec![
            (-0.4, 0.6),
            (-0.4, 0.6), // pair 0
            (0.1, 0.8),
            (0.1, 0.8), // pair 1
        ];

        let rx: Vec<(f64, f64)> = ant0
            .iter()
            .zip(ant1.iter())
            .zip(h0.iter().zip(h1.iter()))
            .map(|((a0, a1), (c0, c1))| {
                complex_add(complex_mul(*c0, *a0), complex_mul(*c1, *a1))
            })
            .collect();

        let recovered = dec.decode(&rx, &h0, &h1);

        assert_eq!(recovered.len(), 4);
        for (orig, rec) in symbols.iter().zip(recovered.iter()) {
            assert_complex_eq(*rec, *orig, 1e-9);
        }
    }

    #[test]
    fn test_diversity_gain() {
        // Single antenna: gain should be 1.0
        assert!((diversity_gain(10.0, 1) - 1.0).abs() < EPS);

        // 2 antennas at 10 dB: gain should be > 1
        let g2 = diversity_gain(10.0, 2);
        assert!(
            g2 > 1.0,
            "2-antenna diversity gain should be > 1, got {}",
            g2
        );

        // Higher SNR should yield greater diversity gain
        let g_low = diversity_gain(5.0, 2);
        let g_high = diversity_gain(20.0, 2);
        assert!(
            g_high > g_low,
            "diversity gain should increase with SNR: {} vs {}",
            g_low,
            g_high
        );

        // More antennas should give greater gain at same SNR
        let g3 = diversity_gain(10.0, 3);
        assert!(
            g3 > g2,
            "3-antenna gain ({}) should exceed 2-antenna gain ({})",
            g3,
            g2
        );
    }

    #[test]
    fn test_2x2_decode() {
        let enc = AlamoutiEncoder::new();
        let dec = Alamouti2x2Decoder::new();

        let symbols = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        let (ant0, ant1) = enc.encode(&symbols);

        // 2x2 channel matrix (constant per symbol pair)
        let h00 = vec![
            (0.6, 0.3),
            (0.6, 0.3),
            (0.2, -0.5),
            (0.2, -0.5),
        ];
        let h01 = vec![
            (-0.2, 0.7),
            (-0.2, 0.7),
            (0.8, 0.1),
            (0.8, 0.1),
        ];
        let h10 = vec![
            (0.4, -0.1),
            (0.4, -0.1),
            (-0.3, 0.6),
            (-0.3, 0.6),
        ];
        let h11 = vec![
            (0.5, 0.5),
            (0.5, 0.5),
            (0.7, -0.4),
            (0.7, -0.4),
        ];

        // Received at RX0 = h00*ant0 + h01*ant1
        let rx0: Vec<(f64, f64)> = ant0
            .iter()
            .zip(ant1.iter())
            .zip(h00.iter().zip(h01.iter()))
            .map(|((a0, a1), (c0, c1))| {
                complex_add(complex_mul(*c0, *a0), complex_mul(*c1, *a1))
            })
            .collect();

        // Received at RX1 = h10*ant0 + h11*ant1
        let rx1: Vec<(f64, f64)> = ant0
            .iter()
            .zip(ant1.iter())
            .zip(h10.iter().zip(h11.iter()))
            .map(|((a0, a1), (c0, c1))| {
                complex_add(complex_mul(*c0, *a0), complex_mul(*c1, *a1))
            })
            .collect();

        let recovered = dec.decode(&rx0, &rx1, &h00, &h01, &h10, &h11);

        assert_eq!(recovered.len(), 4);
        for (orig, rec) in symbols.iter().zip(recovered.iter()) {
            assert_complex_eq(*rec, *orig, 1e-9);
        }
    }

    #[test]
    fn test_single_symbol() {
        // Single symbol should be zero-padded to form a pair
        let enc = AlamoutiEncoder::new();
        let dec = AlamoutiDecoder::new();

        let symbols = vec![(1.0, 0.5)];
        let (ant0, ant1) = enc.encode(&symbols);

        // Padded to 2 symbols: (1.0, 0.5) and (0.0, 0.0)
        assert_eq!(ant0.len(), 2);
        assert_eq!(ant1.len(), 2);

        // Time 0: ant0 = s0 = (1, 0.5), ant1 = s1 = (0, 0)
        assert_complex_eq(ant0[0], (1.0, 0.5), EPS);
        assert_complex_eq(ant1[0], (0.0, 0.0), EPS);

        // Decode with identity channel
        let h0 = vec![(1.0, 0.0); 2];
        let h1 = vec![(0.0, 0.0); 2];
        let rx = ant0.clone();
        let recovered = dec.decode(&rx, &h0, &h1);

        assert_complex_eq(recovered[0], (1.0, 0.5), EPS);
        assert_complex_eq(recovered[1], (0.0, 0.0), EPS);
    }

    #[test]
    fn test_empty_input() {
        let enc = AlamoutiEncoder::new();
        let dec = AlamoutiDecoder::new();
        let dec2x2 = Alamouti2x2Decoder::new();

        let (ant0, ant1) = enc.encode(&[]);
        assert!(ant0.is_empty());
        assert!(ant1.is_empty());

        let recovered = dec.decode(&[], &[], &[]);
        assert!(recovered.is_empty());

        let recovered2 = dec2x2.decode(&[], &[], &[], &[], &[], &[]);
        assert!(recovered2.is_empty());
    }
}
