//! Orthogonal Space-Time Block Code (OSTBC) encoder/decoder for MIMO transmit diversity.
//!
//! Implements Alamouti 2x2 (rate 1) and Tarokh extensions for 3-4 transmit antennas.
//! OSTBC provides transmit diversity gain without requiring channel state information
//! at the transmitter, using orthogonal code matrices for linear ML decoding.
//!
//! # Example
//!
//! ```
//! use r4w_core::orthogonal_space_time_block_code::{
//!     OstbcEncoder, OstbcDecoder, OstbcScheme, OstbcConfig,
//! };
//!
//! // Create an Alamouti 2x2 encoder (rate 1, 2 TX antennas)
//! let encoder = OstbcEncoder::new(OstbcScheme::Alamouti2x2);
//! let symbols = vec![(1.0, 0.0), (0.0, 1.0)];
//! let encoded = encoder.encode_block(&symbols);
//! // encoded[time_slot][antenna]: 2 time slots, 2 antennas
//! assert_eq!(encoded.len(), 2);
//! assert_eq!(encoded[0].len(), 2);
//!
//! // Decode with perfect channel knowledge (identity channel)
//! let config = OstbcConfig {
//!     scheme: OstbcScheme::Alamouti2x2,
//!     num_rx_antennas: 1,
//! };
//! let decoder = OstbcDecoder::new(config);
//! // Channel: h[tx_ant] = (1,0) for all antennas (identity fading)
//! let channel = vec![
//!     vec![(1.0, 0.0), (1.0, 0.0)],
//!     vec![(1.0, 0.0), (1.0, 0.0)],
//! ];
//! // Simulate received signal: r[t][rx] = sum_tx h[tx]*encoded[t][tx]
//! let received: Vec<Vec<(f64, f64)>> = encoded.iter().enumerate().map(|(t, row)| {
//!     let mut r = (0.0, 0.0);
//!     for tx in 0..2 {
//!         let h = channel[t][tx];
//!         let s = row[tx];
//!         r.0 += h.0 * s.0 - h.1 * s.1;
//!         r.1 += h.0 * s.1 + h.1 * s.0;
//!     }
//!     vec![r]
//! }).collect();
//! let decoded = decoder.decode_ml(&received, &channel);
//! assert_eq!(decoded.len(), 2);
//! // With identity channel, decoded symbols match input
//! let tol = 1e-9;
//! assert!((decoded[0].0 - 1.0).abs() < tol);
//! assert!((decoded[0].1 - 0.0).abs() < tol);
//! assert!((decoded[1].0 - 0.0).abs() < tol);
//! assert!((decoded[1].1 - 1.0).abs() < tol);
//! ```

/// Complex number helper: conjugate of (re, im) -> (re, -im)
#[inline]
fn conj(c: (f64, f64)) -> (f64, f64) {
    (c.0, -c.1)
}

/// Complex multiplication: (a+bi)(c+di) = (ac-bd) + (ad+bc)i
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex addition
#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex subtraction
#[inline]
fn csub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Complex negation
#[inline]
fn cneg(c: (f64, f64)) -> (f64, f64) {
    (-c.0, -c.1)
}

/// Squared magnitude |c|^2
#[inline]
fn cmag2(c: (f64, f64)) -> f64 {
    c.0 * c.0 + c.1 * c.1
}

/// Scale complex by real
#[inline]
fn cscale(c: (f64, f64), s: f64) -> (f64, f64) {
    (c.0 * s, c.1 * s)
}

/// Zero complex constant
const CZERO: (f64, f64) = (0.0, 0.0);

/// OSTBC coding scheme variants.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OstbcScheme {
    /// Alamouti code: 2 TX antennas, rate 1, 2 time slots for 2 symbols.
    Alamouti2x2,
    /// Tarokh G3: 3 TX antennas, rate 3/4, 4 time slots for 3 symbols.
    Rate3_4_3Tx,
    /// Tarokh G4: 4 TX antennas, rate 3/4, 4 time slots for 3 symbols.
    Rate3_4_4Tx,
    /// 4 TX antennas, rate 1/2, 4 time slots for 2 symbols.
    Rate1_2_4Tx,
}

/// Configuration for OSTBC decoder.
#[derive(Debug, Clone)]
pub struct OstbcConfig {
    /// The OSTBC scheme to use.
    pub scheme: OstbcScheme,
    /// Number of receive antennas.
    pub num_rx_antennas: usize,
}

/// OSTBC encoder: maps input symbols to space-time coded blocks.
#[derive(Debug, Clone)]
pub struct OstbcEncoder {
    scheme: OstbcScheme,
}

impl OstbcEncoder {
    /// Create a new OSTBC encoder for the given scheme.
    pub fn new(scheme: OstbcScheme) -> Self {
        Self { scheme }
    }

    /// Returns the code rate (symbols per time slot per antenna usage).
    pub fn code_rate(&self) -> f64 {
        match self.scheme {
            OstbcScheme::Alamouti2x2 => 1.0,
            OstbcScheme::Rate3_4_3Tx => 0.75,
            OstbcScheme::Rate3_4_4Tx => 0.75,
            OstbcScheme::Rate1_2_4Tx => 0.5,
        }
    }

    /// Returns (symbols_in, time_slots) for one code block.
    pub fn block_size(&self) -> (usize, usize) {
        match self.scheme {
            OstbcScheme::Alamouti2x2 => (2, 2),
            OstbcScheme::Rate3_4_3Tx => (3, 4),
            OstbcScheme::Rate3_4_4Tx => (3, 4),
            OstbcScheme::Rate1_2_4Tx => (2, 4),
        }
    }

    /// Number of transmit antennas for this scheme.
    pub fn num_tx_antennas(&self) -> usize {
        match self.scheme {
            OstbcScheme::Alamouti2x2 => 2,
            OstbcScheme::Rate3_4_3Tx => 3,
            OstbcScheme::Rate3_4_4Tx => 4,
            OstbcScheme::Rate1_2_4Tx => 4,
        }
    }

    /// Encode a block of symbols into a space-time code matrix.
    ///
    /// Returns `Vec<Vec<(f64,f64)>>` where the outer index is time slot
    /// and the inner index is transmit antenna.
    ///
    /// # Panics
    ///
    /// Panics if `symbols.len()` does not match the required block input size.
    pub fn encode_block(&self, symbols: &[(f64, f64)]) -> Vec<Vec<(f64, f64)>> {
        let (syms_in, _) = self.block_size();
        assert_eq!(
            symbols.len(),
            syms_in,
            "Expected {} symbols for {:?}, got {}",
            syms_in,
            self.scheme,
            symbols.len()
        );

        match self.scheme {
            OstbcScheme::Alamouti2x2 => self.encode_alamouti(symbols),
            OstbcScheme::Rate3_4_3Tx => self.encode_g3(symbols),
            OstbcScheme::Rate3_4_4Tx => self.encode_g4(symbols),
            OstbcScheme::Rate1_2_4Tx => self.encode_rate_half_4tx(symbols),
        }
    }

    /// Alamouti 2x2 encoding:
    /// Time 0: [s1,  s2 ]
    /// Time 1: [-s2*, s1*]
    fn encode_alamouti(&self, s: &[(f64, f64)]) -> Vec<Vec<(f64, f64)>> {
        let s1 = s[0];
        let s2 = s[1];
        vec![
            vec![s1, s2],
            vec![cneg(conj(s2)), conj(s1)],
        ]
    }

    /// Tarokh G3 code for 3 TX antennas (rate 3/4, 4 time slots, 3 symbols):
    ///
    /// Time 0: [ s1,   s2,   s3  ]
    /// Time 1: [-s2*,  s1*,  0   ]
    /// Time 2: [-s3*,  0,    s1* ]
    /// Time 3: [ 0,   -s3*,  s2* ]
    fn encode_g3(&self, s: &[(f64, f64)]) -> Vec<Vec<(f64, f64)>> {
        let s1 = s[0];
        let s2 = s[1];
        let s3 = s[2];
        vec![
            vec![s1, s2, s3],
            vec![cneg(conj(s2)), conj(s1), CZERO],
            vec![cneg(conj(s3)), CZERO, conj(s1)],
            vec![CZERO, cneg(conj(s3)), conj(s2)],
        ]
    }

    /// Tarokh G4 code for 4 TX antennas (rate 3/4, 4 time slots, 3 symbols):
    ///
    /// Time 0: [ s1,    s2,    s3,    0   ]
    /// Time 1: [-s2*,   s1*,   0,     s3  ]
    /// Time 2: [-s3*,   0,     s1*,  -s2  ]
    /// Time 3: [ 0,    -s3*,   s2*,   s1  ]
    fn encode_g4(&self, s: &[(f64, f64)]) -> Vec<Vec<(f64, f64)>> {
        let s1 = s[0];
        let s2 = s[1];
        let s3 = s[2];
        vec![
            vec![s1, s2, s3, CZERO],
            vec![cneg(conj(s2)), conj(s1), CZERO, s3],
            vec![cneg(conj(s3)), CZERO, conj(s1), cneg(s2)],
            vec![CZERO, cneg(conj(s3)), conj(s2), s1],
        ]
    }

    /// Rate-1/2 code for 4 TX antennas (4 time slots, 2 symbols):
    ///
    /// Time 0: [ s1,    s2,    0,     0   ]
    /// Time 1: [-s2*,   s1*,   0,     0   ]
    /// Time 2: [ 0,     0,     s1,    s2  ]
    /// Time 3: [ 0,     0,    -s2*,   s1* ]
    fn encode_rate_half_4tx(&self, s: &[(f64, f64)]) -> Vec<Vec<(f64, f64)>> {
        let s1 = s[0];
        let s2 = s[1];
        vec![
            vec![s1, s2, CZERO, CZERO],
            vec![cneg(conj(s2)), conj(s1), CZERO, CZERO],
            vec![CZERO, CZERO, s1, s2],
            vec![CZERO, CZERO, cneg(conj(s2)), conj(s1)],
        ]
    }
}

/// OSTBC decoder: ML decoding with linear complexity using channel estimates.
#[derive(Debug, Clone)]
pub struct OstbcDecoder {
    config: OstbcConfig,
}

impl OstbcDecoder {
    /// Create a new OSTBC decoder.
    pub fn new(config: OstbcConfig) -> Self {
        Self { config }
    }

    /// Returns the code rate.
    pub fn code_rate(&self) -> f64 {
        match self.config.scheme {
            OstbcScheme::Alamouti2x2 => 1.0,
            OstbcScheme::Rate3_4_3Tx => 0.75,
            OstbcScheme::Rate3_4_4Tx => 0.75,
            OstbcScheme::Rate1_2_4Tx => 0.5,
        }
    }

    /// Returns (symbols_in, time_slots) for one code block.
    pub fn block_size(&self) -> (usize, usize) {
        match self.config.scheme {
            OstbcScheme::Alamouti2x2 => (2, 2),
            OstbcScheme::Rate3_4_3Tx => (3, 4),
            OstbcScheme::Rate3_4_4Tx => (3, 4),
            OstbcScheme::Rate1_2_4Tx => (2, 4),
        }
    }

    /// Maximum-likelihood decoding using linear combining.
    ///
    /// # Arguments
    /// - `received`: `received[time_slot][rx_antenna]` - received samples
    /// - `channel`: `channel[time_slot][tx_ant * num_rx + rx_ant]` - channel estimates
    ///
    /// # Returns
    /// Decoded symbols (one per input symbol of the code block).
    pub fn decode_ml(
        &self,
        received: &[Vec<(f64, f64)>],
        channel: &[Vec<(f64, f64)>],
    ) -> Vec<(f64, f64)> {
        match self.config.scheme {
            OstbcScheme::Alamouti2x2 => self.decode_alamouti(received, channel),
            OstbcScheme::Rate3_4_3Tx => self.decode_g3(received, channel),
            OstbcScheme::Rate3_4_4Tx => self.decode_g4(received, channel),
            OstbcScheme::Rate1_2_4Tx => self.decode_rate_half_4tx(received, channel),
        }
    }

    /// Diversity combining: same as decode_ml but returns the combined
    /// (soft) symbol estimates before hard decision.
    pub fn combine_diversity(
        &self,
        received: &[Vec<(f64, f64)>],
        channel: &[Vec<(f64, f64)>],
    ) -> Vec<(f64, f64)> {
        // For OSTBC, linear combining IS the ML decoder (no separate hard decision needed).
        self.decode_ml(received, channel)
    }

    /// Alamouti ML decoding for 2x2 (or 2xN_r).
    ///
    /// For each RX antenna j:
    ///   Time 0, RX j: r0_j = h0j*s1 + h1j*s2
    ///   Time 1, RX j: r1_j = h0j*(-s2*) + h1j*(s1*)
    ///
    /// Linear combining for s1:
    ///   s1_hat = sum_j [ conj(h0j)*r0_j + h1j*conj(r1_j) ]
    /// Linear combining for s2:
    ///   s2_hat = sum_j [ conj(h1j)*r0_j - h0j*conj(r1_j) ]
    /// Normalize by sum_j [ |h0j|^2 + |h1j|^2 ]
    fn decode_alamouti(
        &self,
        received: &[Vec<(f64, f64)>],
        channel: &[Vec<(f64, f64)>],
    ) -> Vec<(f64, f64)> {
        let num_rx = self.config.num_rx_antennas;
        let mut s1_hat = CZERO;
        let mut s2_hat = CZERO;
        let mut norm = 0.0;

        for j in 0..num_rx {
            // Channel from TX ant k to RX ant j at time 0
            // channel layout: channel[time][tx * num_rx + rx]
            let h0j = channel[0][0 * num_rx + j]; // TX0 -> RXj
            let h1j = channel[0][1 * num_rx + j]; // TX1 -> RXj

            let r0 = received[0][j];
            let r1 = received[1][j];

            // s1_hat += conj(h0j)*r0 + h1j*conj(r1)
            s1_hat = cadd(s1_hat, cadd(cmul(conj(h0j), r0), cmul(h1j, conj(r1))));
            // s2_hat += conj(h1j)*r0 - h0j*conj(r1)
            s2_hat = cadd(s2_hat, csub(cmul(conj(h1j), r0), cmul(h0j, conj(r1))));

            norm += cmag2(h0j) + cmag2(h1j);
        }

        if norm > 1e-30 {
            let inv = 1.0 / norm;
            vec![cscale(s1_hat, inv), cscale(s2_hat, inv)]
        } else {
            vec![CZERO, CZERO]
        }
    }

    /// Decode Tarokh G3 (3 TX, rate 3/4).
    ///
    /// The G3 code structure allows symbol-by-symbol ML decoding via linear combining.
    fn decode_g3(
        &self,
        received: &[Vec<(f64, f64)>],
        channel: &[Vec<(f64, f64)>],
    ) -> Vec<(f64, f64)> {
        let num_rx = self.config.num_rx_antennas;
        let mut s1_hat = CZERO;
        let mut s2_hat = CZERO;
        let mut s3_hat = CZERO;
        let mut norm = 0.0;

        for j in 0..num_rx {
            let h0 = channel[0][0 * num_rx + j]; // TX0
            let h1 = channel[0][1 * num_rx + j]; // TX1
            let h2 = channel[0][2 * num_rx + j]; // TX2

            let r0 = received[0][j];
            let r1 = received[1][j];
            let r2 = received[2][j];
            let r3 = received[3][j];

            // From the G3 code matrix structure:
            // r0 = h0*s1 + h1*s2 + h2*s3
            // r1 = -h0*s2* + h1*s1*
            // r2 = -h0*s3* + h2*s1*
            // r3 = -h1*s3* + h2*s2*
            //
            // Linear combining for s1:
            //   conj(h0)*r0 + h1*conj(r1) + h2*conj(r2)
            s1_hat = cadd(
                s1_hat,
                cadd(
                    cmul(conj(h0), r0),
                    cadd(cmul(h1, conj(r1)), cmul(h2, conj(r2))),
                ),
            );

            // Linear combining for s2:
            //   conj(h1)*r0 - h0*conj(r1) + h2*conj(r3)
            s2_hat = cadd(
                s2_hat,
                cadd(
                    csub(cmul(conj(h1), r0), cmul(h0, conj(r1))),
                    cmul(h2, conj(r3)),
                ),
            );

            // Linear combining for s3:
            //   conj(h2)*r0 - h0*conj(r2) - h1*conj(r3)
            s3_hat = cadd(
                s3_hat,
                csub(
                    csub(cmul(conj(h2), r0), cmul(h0, conj(r2))),
                    cmul(h1, conj(r3)),
                ),
            );

            norm += cmag2(h0) + cmag2(h1) + cmag2(h2);
        }

        if norm > 1e-30 {
            let inv = 1.0 / norm;
            vec![
                cscale(s1_hat, inv),
                cscale(s2_hat, inv),
                cscale(s3_hat, inv),
            ]
        } else {
            vec![CZERO, CZERO, CZERO]
        }
    }

    /// Decode Tarokh G4 (4 TX, rate 3/4).
    ///
    /// The G4 code has columns that are not purely conjugate-based in column 4,
    /// but the structure still allows linear combining for ML decoding.
    fn decode_g4(
        &self,
        received: &[Vec<(f64, f64)>],
        channel: &[Vec<(f64, f64)>],
    ) -> Vec<(f64, f64)> {
        let num_rx = self.config.num_rx_antennas;
        let mut s1_hat = CZERO;
        let mut s2_hat = CZERO;
        let mut s3_hat = CZERO;
        let mut norm = 0.0;

        for j in 0..num_rx {
            let h0 = channel[0][0 * num_rx + j];
            let h1 = channel[0][1 * num_rx + j];
            let h2 = channel[0][2 * num_rx + j];
            let h3 = channel[0][3 * num_rx + j];

            let r0 = received[0][j];
            let r1 = received[1][j];
            let r2 = received[2][j];
            let r3 = received[3][j];

            // G4 code matrix:
            // Time 0: [ s1,    s2,    s3,    0   ]
            // Time 1: [-s2*,   s1*,   0,     s3  ]
            // Time 2: [-s3*,   0,     s1*,  -s2  ]
            // Time 3: [ 0,    -s3*,   s2*,   s1  ]
            //
            // r0 = h0*s1 + h1*s2 + h2*s3
            // r1 = -h0*s2* + h1*s1* + h3*s3
            // r2 = -h0*s3* + h2*s1* - h3*s2
            // r3 = -h1*s3* + h2*s2* + h3*s1
            //
            // Linear combining for s1:
            //   conj(h0)*r0 + h1*conj(r1) + h2*conj(r2) + conj(h3)*r3
            s1_hat = cadd(
                s1_hat,
                cadd(
                    cadd(cmul(conj(h0), r0), cmul(h1, conj(r1))),
                    cadd(cmul(h2, conj(r2)), cmul(conj(h3), r3)),
                ),
            );

            // Linear combining for s2:
            //   conj(h1)*r0 - h0*conj(r1) - conj(h3)*r2 + h2*conj(r3)
            s2_hat = cadd(
                s2_hat,
                cadd(
                    csub(cmul(conj(h1), r0), cmul(h0, conj(r1))),
                    csub(cmul(h2, conj(r3)), cmul(conj(h3), r2)),
                ),
            );

            // Linear combining for s3:
            //   conj(h2)*r0 - h0*conj(r2) - h1*conj(r3) + conj(h3)*r1
            s3_hat = cadd(
                s3_hat,
                cadd(
                    csub(cmul(conj(h2), r0), cmul(h0, conj(r2))),
                    csub(cmul(conj(h3), r1), cmul(h1, conj(r3))),
                ),
            );

            norm += cmag2(h0) + cmag2(h1) + cmag2(h2) + cmag2(h3);
        }

        if norm > 1e-30 {
            let inv = 1.0 / norm;
            vec![
                cscale(s1_hat, inv),
                cscale(s2_hat, inv),
                cscale(s3_hat, inv),
            ]
        } else {
            vec![CZERO, CZERO, CZERO]
        }
    }

    /// Decode rate-1/2 4TX code.
    ///
    /// This is essentially two parallel Alamouti blocks on antenna pairs (0,1) and (2,3).
    fn decode_rate_half_4tx(
        &self,
        received: &[Vec<(f64, f64)>],
        channel: &[Vec<(f64, f64)>],
    ) -> Vec<(f64, f64)> {
        let num_rx = self.config.num_rx_antennas;
        let mut s1_hat = CZERO;
        let mut s2_hat = CZERO;
        let mut norm = 0.0;

        for j in 0..num_rx {
            // First Alamouti pair (antennas 0,1) in time slots 0,1
            let h0 = channel[0][0 * num_rx + j];
            let h1 = channel[0][1 * num_rx + j];

            let r0 = received[0][j];
            let r1 = received[1][j];

            s1_hat = cadd(s1_hat, cadd(cmul(conj(h0), r0), cmul(h1, conj(r1))));
            s2_hat = cadd(s2_hat, csub(cmul(conj(h1), r0), cmul(h0, conj(r1))));

            norm += cmag2(h0) + cmag2(h1);

            // Second Alamouti pair (antennas 2,3) in time slots 2,3
            let h2 = channel[0][2 * num_rx + j];
            let h3 = channel[0][3 * num_rx + j];

            let r2 = received[2][j];
            let r3 = received[3][j];

            s1_hat = cadd(s1_hat, cadd(cmul(conj(h2), r2), cmul(h3, conj(r3))));
            s2_hat = cadd(s2_hat, csub(cmul(conj(h3), r2), cmul(h2, conj(r3))));

            norm += cmag2(h2) + cmag2(h3);
        }

        if norm > 1e-30 {
            let inv = 1.0 / norm;
            vec![cscale(s1_hat, inv), cscale(s2_hat, inv)]
        } else {
            vec![CZERO, CZERO]
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-9;

    fn assert_complex_eq(a: (f64, f64), b: (f64, f64), tol: f64) {
        assert!(
            (a.0 - b.0).abs() < tol && (a.1 - b.1).abs() < tol,
            "Expected ({}, {}), got ({}, {})",
            b.0, b.1, a.0, a.1
        );
    }

    #[test]
    fn test_alamouti_encode_basic() {
        let enc = OstbcEncoder::new(OstbcScheme::Alamouti2x2);
        let s = vec![(1.0, 0.0), (0.0, 1.0)];
        let coded = enc.encode_block(&s);

        // Time 0: [s1, s2] = [(1,0), (0,1)]
        assert_complex_eq(coded[0][0], (1.0, 0.0), TOL);
        assert_complex_eq(coded[0][1], (0.0, 1.0), TOL);
        // Time 1: [-s2*, s1*] = [-(0,-1), (1,0)] = [(0,1), (1,0)]
        assert_complex_eq(coded[1][0], (0.0, 1.0), TOL);
        assert_complex_eq(coded[1][1], (1.0, 0.0), TOL);
    }

    #[test]
    fn test_alamouti_encode_real_symbols() {
        let enc = OstbcEncoder::new(OstbcScheme::Alamouti2x2);
        let s = vec![(1.0, 0.0), (-1.0, 0.0)];
        let coded = enc.encode_block(&s);

        assert_complex_eq(coded[0][0], (1.0, 0.0), TOL);
        assert_complex_eq(coded[0][1], (-1.0, 0.0), TOL);
        // -conj(-1,0) = -(−1,0) = (1,0), conj(1,0) = (1,0)
        assert_complex_eq(coded[1][0], (1.0, 0.0), TOL);
        assert_complex_eq(coded[1][1], (1.0, 0.0), TOL);
    }

    #[test]
    fn test_alamouti_roundtrip_identity_channel() {
        let enc = OstbcEncoder::new(OstbcScheme::Alamouti2x2);
        let config = OstbcConfig {
            scheme: OstbcScheme::Alamouti2x2,
            num_rx_antennas: 1,
        };
        let dec = OstbcDecoder::new(config);

        let symbols = vec![(0.7, -0.3), (-0.5, 0.8)];
        let coded = enc.encode_block(&symbols);

        // Identity channel: h0=1, h1=1 for single RX
        let channel = vec![
            vec![(1.0, 0.0), (1.0, 0.0)],
            vec![(1.0, 0.0), (1.0, 0.0)],
        ];

        // Simulate received = coded (since channel is identity on each TX->RX path)
        // r[t][rx=0] = sum over tx of h_tx * coded[t][tx]
        let mut received = vec![vec![CZERO; 1]; 2];
        for t in 0..2 {
            for tx in 0..2 {
                received[t][0] = cadd(received[t][0], cmul(channel[t][tx], coded[t][tx]));
            }
        }

        let decoded = dec.decode_ml(&received, &channel);
        assert_complex_eq(decoded[0], symbols[0], TOL);
        assert_complex_eq(decoded[1], symbols[1], TOL);
    }

    #[test]
    fn test_alamouti_roundtrip_complex_channel() {
        let enc = OstbcEncoder::new(OstbcScheme::Alamouti2x2);
        let config = OstbcConfig {
            scheme: OstbcScheme::Alamouti2x2,
            num_rx_antennas: 1,
        };
        let dec = OstbcDecoder::new(config);

        let symbols = vec![(1.0, 0.0), (0.0, 1.0)];
        let coded = enc.encode_block(&symbols);

        // Complex fading channel (constant across time slots for Alamouti)
        let h0 = (0.8, 0.3);
        let h1 = (-0.5, 0.6);
        let channel = vec![
            vec![h0, h1],
            vec![h0, h1],
        ];

        // Received signal
        let mut received = vec![vec![CZERO; 1]; 2];
        for t in 0..2 {
            for tx in 0..2 {
                received[t][0] = cadd(received[t][0], cmul(channel[t][tx], coded[t][tx]));
            }
        }

        let decoded = dec.decode_ml(&received, &channel);
        assert_complex_eq(decoded[0], symbols[0], 1e-6);
        assert_complex_eq(decoded[1], symbols[1], 1e-6);
    }

    #[test]
    fn test_alamouti_two_rx_antennas() {
        let enc = OstbcEncoder::new(OstbcScheme::Alamouti2x2);
        let config = OstbcConfig {
            scheme: OstbcScheme::Alamouti2x2,
            num_rx_antennas: 2,
        };
        let dec = OstbcDecoder::new(config);

        let symbols = vec![(0.5, 0.5), (-0.5, 0.5)];
        let coded = enc.encode_block(&symbols);

        // Two different channels for 2 RX antennas
        let h00 = (0.9, 0.1); // TX0->RX0
        let h01 = (0.3, -0.7); // TX0->RX1
        let h10 = (-0.4, 0.5); // TX1->RX0
        let h11 = (0.6, 0.2); // TX1->RX1

        // channel[time][tx * num_rx + rx]
        let channel = vec![
            vec![h00, h01, h10, h11],
            vec![h00, h01, h10, h11],
        ];

        // received[time][rx]
        let mut received = vec![vec![CZERO; 2]; 2];
        for t in 0..2 {
            for rx in 0..2 {
                for tx in 0..2 {
                    let h = channel[t][tx * 2 + rx];
                    received[t][rx] = cadd(received[t][rx], cmul(h, coded[t][tx]));
                }
            }
        }

        let decoded = dec.decode_ml(&received, &channel);
        assert_complex_eq(decoded[0], symbols[0], 1e-6);
        assert_complex_eq(decoded[1], symbols[1], 1e-6);
    }

    #[test]
    fn test_code_rates() {
        let enc_a = OstbcEncoder::new(OstbcScheme::Alamouti2x2);
        let enc_g3 = OstbcEncoder::new(OstbcScheme::Rate3_4_3Tx);
        let enc_g4 = OstbcEncoder::new(OstbcScheme::Rate3_4_4Tx);
        let enc_h = OstbcEncoder::new(OstbcScheme::Rate1_2_4Tx);

        assert!((enc_a.code_rate() - 1.0).abs() < TOL);
        assert!((enc_g3.code_rate() - 0.75).abs() < TOL);
        assert!((enc_g4.code_rate() - 0.75).abs() < TOL);
        assert!((enc_h.code_rate() - 0.5).abs() < TOL);
    }

    #[test]
    fn test_block_sizes() {
        let enc_a = OstbcEncoder::new(OstbcScheme::Alamouti2x2);
        let enc_g3 = OstbcEncoder::new(OstbcScheme::Rate3_4_3Tx);
        let enc_g4 = OstbcEncoder::new(OstbcScheme::Rate3_4_4Tx);
        let enc_h = OstbcEncoder::new(OstbcScheme::Rate1_2_4Tx);

        assert_eq!(enc_a.block_size(), (2, 2));
        assert_eq!(enc_g3.block_size(), (3, 4));
        assert_eq!(enc_g4.block_size(), (3, 4));
        assert_eq!(enc_h.block_size(), (2, 4));
    }

    #[test]
    fn test_num_tx_antennas() {
        assert_eq!(OstbcEncoder::new(OstbcScheme::Alamouti2x2).num_tx_antennas(), 2);
        assert_eq!(OstbcEncoder::new(OstbcScheme::Rate3_4_3Tx).num_tx_antennas(), 3);
        assert_eq!(OstbcEncoder::new(OstbcScheme::Rate3_4_4Tx).num_tx_antennas(), 4);
        assert_eq!(OstbcEncoder::new(OstbcScheme::Rate1_2_4Tx).num_tx_antennas(), 4);
    }

    #[test]
    fn test_g3_encode_structure() {
        let enc = OstbcEncoder::new(OstbcScheme::Rate3_4_3Tx);
        let s = vec![(1.0, 0.0), (0.0, 1.0), (1.0, 1.0)];
        let coded = enc.encode_block(&s);

        assert_eq!(coded.len(), 4); // 4 time slots
        assert_eq!(coded[0].len(), 3); // 3 TX antennas

        // Time 0: [s1, s2, s3]
        assert_complex_eq(coded[0][0], (1.0, 0.0), TOL);
        assert_complex_eq(coded[0][1], (0.0, 1.0), TOL);
        assert_complex_eq(coded[0][2], (1.0, 1.0), TOL);

        // Time 1: [-s2*, s1*, 0]
        assert_complex_eq(coded[1][0], (0.0, 1.0), TOL); // -conj(0,1) = -(0,-1) = (0,1)
        assert_complex_eq(coded[1][1], (1.0, 0.0), TOL); // conj(1,0) = (1,0)
        assert_complex_eq(coded[1][2], CZERO, TOL);
    }

    #[test]
    fn test_g3_roundtrip_identity_channel() {
        let enc = OstbcEncoder::new(OstbcScheme::Rate3_4_3Tx);
        let config = OstbcConfig {
            scheme: OstbcScheme::Rate3_4_3Tx,
            num_rx_antennas: 1,
        };
        let dec = OstbcDecoder::new(config);

        let symbols = vec![(0.7, -0.3), (-0.5, 0.8), (0.2, 0.6)];
        let coded = enc.encode_block(&symbols);

        let h0 = (1.0, 0.0);
        let h1 = (1.0, 0.0);
        let h2 = (1.0, 0.0);
        let channel = vec![
            vec![h0, h1, h2],
            vec![h0, h1, h2],
            vec![h0, h1, h2],
            vec![h0, h1, h2],
        ];

        let mut received = vec![vec![CZERO; 1]; 4];
        for t in 0..4 {
            for tx in 0..3 {
                received[t][0] = cadd(received[t][0], cmul(channel[t][tx], coded[t][tx]));
            }
        }

        let decoded = dec.decode_ml(&received, &channel);
        assert_complex_eq(decoded[0], symbols[0], 1e-6);
        assert_complex_eq(decoded[1], symbols[1], 1e-6);
        assert_complex_eq(decoded[2], symbols[2], 1e-6);
    }

    #[test]
    fn test_g3_roundtrip_fading_channel() {
        let enc = OstbcEncoder::new(OstbcScheme::Rate3_4_3Tx);
        let config = OstbcConfig {
            scheme: OstbcScheme::Rate3_4_3Tx,
            num_rx_antennas: 1,
        };
        let dec = OstbcDecoder::new(config);

        let symbols = vec![(1.0, 0.0), (0.0, -1.0), (0.5, 0.5)];
        let coded = enc.encode_block(&symbols);

        let h0 = (0.6, 0.2);
        let h1 = (-0.3, 0.8);
        let h2 = (0.4, -0.5);
        let channel = vec![
            vec![h0, h1, h2],
            vec![h0, h1, h2],
            vec![h0, h1, h2],
            vec![h0, h1, h2],
        ];

        let mut received = vec![vec![CZERO; 1]; 4];
        for t in 0..4 {
            for tx in 0..3 {
                received[t][0] = cadd(received[t][0], cmul(channel[t][tx], coded[t][tx]));
            }
        }

        let decoded = dec.decode_ml(&received, &channel);
        assert_complex_eq(decoded[0], symbols[0], 1e-6);
        assert_complex_eq(decoded[1], symbols[1], 1e-6);
        assert_complex_eq(decoded[2], symbols[2], 1e-6);
    }

    #[test]
    fn test_g4_encode_structure() {
        let enc = OstbcEncoder::new(OstbcScheme::Rate3_4_4Tx);
        let s = vec![(1.0, 0.0), (0.0, 1.0), (1.0, 1.0)];
        let coded = enc.encode_block(&s);

        assert_eq!(coded.len(), 4);
        assert_eq!(coded[0].len(), 4);

        // Time 0: [s1, s2, s3, 0]
        assert_complex_eq(coded[0][0], (1.0, 0.0), TOL);
        assert_complex_eq(coded[0][1], (0.0, 1.0), TOL);
        assert_complex_eq(coded[0][2], (1.0, 1.0), TOL);
        assert_complex_eq(coded[0][3], CZERO, TOL);
    }

    #[test]
    fn test_g4_roundtrip_fading_channel() {
        let enc = OstbcEncoder::new(OstbcScheme::Rate3_4_4Tx);
        let config = OstbcConfig {
            scheme: OstbcScheme::Rate3_4_4Tx,
            num_rx_antennas: 1,
        };
        let dec = OstbcDecoder::new(config);

        let symbols = vec![(0.3, -0.7), (0.9, 0.1), (-0.4, 0.6)];
        let coded = enc.encode_block(&symbols);

        let h0 = (0.5, 0.3);
        let h1 = (-0.2, 0.7);
        let h2 = (0.8, -0.1);
        let h3 = (0.1, 0.4);
        let channel = vec![
            vec![h0, h1, h2, h3],
            vec![h0, h1, h2, h3],
            vec![h0, h1, h2, h3],
            vec![h0, h1, h2, h3],
        ];

        let mut received = vec![vec![CZERO; 1]; 4];
        for t in 0..4 {
            for tx in 0..4 {
                received[t][0] = cadd(received[t][0], cmul(channel[t][tx], coded[t][tx]));
            }
        }

        let decoded = dec.decode_ml(&received, &channel);
        assert_complex_eq(decoded[0], symbols[0], 1e-6);
        assert_complex_eq(decoded[1], symbols[1], 1e-6);
        assert_complex_eq(decoded[2], symbols[2], 1e-6);
    }

    #[test]
    fn test_rate_half_4tx_roundtrip() {
        let enc = OstbcEncoder::new(OstbcScheme::Rate1_2_4Tx);
        let config = OstbcConfig {
            scheme: OstbcScheme::Rate1_2_4Tx,
            num_rx_antennas: 1,
        };
        let dec = OstbcDecoder::new(config);

        let symbols = vec![(1.0, 0.5), (-0.5, -0.3)];
        let coded = enc.encode_block(&symbols);

        assert_eq!(coded.len(), 4);
        assert_eq!(coded[0].len(), 4);

        let h0 = (0.7, -0.2);
        let h1 = (0.3, 0.6);
        let h2 = (-0.5, 0.4);
        let h3 = (0.8, 0.1);
        let channel = vec![
            vec![h0, h1, h2, h3],
            vec![h0, h1, h2, h3],
            vec![h0, h1, h2, h3],
            vec![h0, h1, h2, h3],
        ];

        let mut received = vec![vec![CZERO; 1]; 4];
        for t in 0..4 {
            for tx in 0..4 {
                received[t][0] = cadd(received[t][0], cmul(channel[t][tx], coded[t][tx]));
            }
        }

        let decoded = dec.decode_ml(&received, &channel);
        assert_complex_eq(decoded[0], symbols[0], 1e-6);
        assert_complex_eq(decoded[1], symbols[1], 1e-6);
    }

    #[test]
    fn test_combine_diversity_equals_decode_ml() {
        let enc = OstbcEncoder::new(OstbcScheme::Alamouti2x2);
        let config = OstbcConfig {
            scheme: OstbcScheme::Alamouti2x2,
            num_rx_antennas: 1,
        };
        let dec = OstbcDecoder::new(config);

        let symbols = vec![(0.3, 0.4), (-0.6, 0.1)];
        let coded = enc.encode_block(&symbols);

        let channel = vec![
            vec![(0.8, 0.3), (-0.5, 0.6)],
            vec![(0.8, 0.3), (-0.5, 0.6)],
        ];

        let mut received = vec![vec![CZERO; 1]; 2];
        for t in 0..2 {
            for tx in 0..2 {
                received[t][0] = cadd(received[t][0], cmul(channel[t][tx], coded[t][tx]));
            }
        }

        let ml = dec.decode_ml(&received, &channel);
        let cd = dec.combine_diversity(&received, &channel);
        assert_complex_eq(ml[0], cd[0], TOL);
        assert_complex_eq(ml[1], cd[1], TOL);
    }

    #[test]
    fn test_decoder_code_rate_and_block_size() {
        let config = OstbcConfig {
            scheme: OstbcScheme::Rate3_4_3Tx,
            num_rx_antennas: 2,
        };
        let dec = OstbcDecoder::new(config);
        assert!((dec.code_rate() - 0.75).abs() < TOL);
        assert_eq!(dec.block_size(), (3, 4));
    }

    #[test]
    #[should_panic(expected = "Expected 2 symbols")]
    fn test_alamouti_wrong_input_size() {
        let enc = OstbcEncoder::new(OstbcScheme::Alamouti2x2);
        enc.encode_block(&[(1.0, 0.0)]); // Should panic: need 2 symbols
    }

    #[test]
    fn test_complex_helpers() {
        assert_complex_eq(conj((3.0, 4.0)), (3.0, -4.0), TOL);
        assert_complex_eq(cmul((1.0, 2.0), (3.0, 4.0)), (-5.0, 10.0), TOL);
        assert_complex_eq(cadd((1.0, 2.0), (3.0, 4.0)), (4.0, 6.0), TOL);
        assert_complex_eq(csub((5.0, 7.0), (3.0, 4.0)), (2.0, 3.0), TOL);
        assert_complex_eq(cneg((3.0, -4.0)), (-3.0, 4.0), TOL);
        assert!((cmag2((3.0, 4.0)) - 25.0).abs() < TOL);
        assert_complex_eq(cscale((2.0, 3.0), 2.0), (4.0, 6.0), TOL);
    }
}
