//! Differential Encoder / Decoder
//!
//! Differential encoding resolves phase ambiguity in PSK systems by encoding
//! information in phase *transitions* rather than absolute phase values.
//! This means the receiver doesn't need an absolute phase reference.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::differential::{DiffEncoder, DiffDecoder};
//!
//! // DQPSK: 4-level differential encoding
//! let mut encoder = DiffEncoder::new(4);
//! let mut decoder = DiffDecoder::new(4);
//!
//! let data = vec![0, 1, 3, 2, 1, 0, 2, 3];
//! let encoded: Vec<u8> = data.iter().map(|&s| encoder.encode(s)).collect();
//! let decoded: Vec<u8> = encoded.iter().map(|&s| decoder.decode(s)).collect();
//! assert_eq!(data, decoded);
//! ```

use num_complex::Complex64;

/// Differential encoder.
///
/// Maps: `y[n] = (x[n] + y[n-1]) mod M`
///
/// where M is the constellation size (modulus).
#[derive(Debug, Clone)]
pub struct DiffEncoder {
    modulus: u8,
    prev: u8,
}

impl DiffEncoder {
    /// Create a new differential encoder with the given modulus.
    ///
    /// Common values: 2 (DBPSK), 4 (DQPSK), 8 (D8PSK).
    pub fn new(modulus: u8) -> Self {
        Self { modulus, prev: 0 }
    }

    /// Encode a single symbol.
    pub fn encode(&mut self, symbol: u8) -> u8 {
        self.prev = (symbol + self.prev) % self.modulus;
        self.prev
    }

    /// Encode a sequence of symbols.
    pub fn encode_symbols(&mut self, symbols: &[u8]) -> Vec<u8> {
        symbols.iter().map(|&s| self.encode(s)).collect()
    }

    /// Reset encoder state.
    pub fn reset(&mut self) {
        self.prev = 0;
    }
}

/// Differential decoder.
///
/// Maps: `y[n] = (x[n] - x[n-1] + M) mod M`
#[derive(Debug, Clone)]
pub struct DiffDecoder {
    modulus: u8,
    prev: u8,
}

impl DiffDecoder {
    /// Create a new differential decoder with the given modulus.
    pub fn new(modulus: u8) -> Self {
        Self { modulus, prev: 0 }
    }

    /// Decode a single symbol.
    pub fn decode(&mut self, symbol: u8) -> u8 {
        let out = (symbol + self.modulus - self.prev) % self.modulus;
        self.prev = symbol;
        out
    }

    /// Decode a sequence of symbols.
    pub fn decode_symbols(&mut self, symbols: &[u8]) -> Vec<u8> {
        symbols.iter().map(|&s| self.decode(s)).collect()
    }

    /// Reset decoder state.
    pub fn reset(&mut self) {
        self.prev = 0;
    }
}

/// Complex-domain differential encoder/decoder.
///
/// Works directly on constellation points for DPSK:
/// - Encode: `y[n] = x[n] * y[n-1]`
/// - Decode: `y[n] = x[n] * conj(x[n-1])`
#[derive(Debug, Clone)]
pub struct ComplexDiffEncoder {
    prev: Complex64,
}

impl ComplexDiffEncoder {
    /// Create a new complex differential encoder.
    pub fn new() -> Self {
        Self {
            prev: Complex64::new(1.0, 0.0),
        }
    }

    /// Encode a single complex symbol.
    pub fn encode(&mut self, symbol: Complex64) -> Complex64 {
        self.prev = symbol * self.prev;
        // Normalize to prevent drift
        let norm = self.prev.norm();
        if norm > 1e-10 {
            self.prev /= norm;
        }
        self.prev
    }

    /// Encode a sequence of complex symbols.
    pub fn encode_symbols(&mut self, symbols: &[Complex64]) -> Vec<Complex64> {
        symbols.iter().map(|&s| self.encode(s)).collect()
    }

    /// Reset to initial state.
    pub fn reset(&mut self) {
        self.prev = Complex64::new(1.0, 0.0);
    }
}

impl Default for ComplexDiffEncoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Complex-domain differential decoder.
#[derive(Debug, Clone)]
pub struct ComplexDiffDecoder {
    prev: Complex64,
}

impl ComplexDiffDecoder {
    /// Create a new complex differential decoder.
    pub fn new() -> Self {
        Self {
            prev: Complex64::new(1.0, 0.0),
        }
    }

    /// Decode a single complex symbol.
    pub fn decode(&mut self, symbol: Complex64) -> Complex64 {
        let decoded = symbol * self.prev.conj();
        self.prev = symbol;
        // Normalize
        let norm = decoded.norm();
        if norm > 1e-10 {
            decoded / norm
        } else {
            decoded
        }
    }

    /// Decode a sequence of complex symbols.
    pub fn decode_symbols(&mut self, symbols: &[Complex64]) -> Vec<Complex64> {
        symbols.iter().map(|&s| self.decode(s)).collect()
    }

    /// Reset to initial state.
    pub fn reset(&mut self) {
        self.prev = Complex64::new(1.0, 0.0);
    }
}

impl Default for ComplexDiffDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Generate DPSK constellation points for the given modulus.
pub fn dpsk_constellation(modulus: u8) -> Vec<Complex64> {
    (0..modulus)
        .map(|i| {
            let angle = 2.0 * std::f64::consts::PI * i as f64 / modulus as f64;
            Complex64::new(angle.cos(), angle.sin())
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_dbpsk_roundtrip() {
        let mut enc = DiffEncoder::new(2);
        let mut dec = DiffDecoder::new(2);

        let data = vec![0, 1, 1, 0, 1, 0, 0, 1, 1, 0];
        let encoded = enc.encode_symbols(&data);
        let decoded = dec.decode_symbols(&encoded);

        assert_eq!(data, decoded, "DBPSK roundtrip failed");
    }

    #[test]
    fn test_dqpsk_roundtrip() {
        let mut enc = DiffEncoder::new(4);
        let mut dec = DiffDecoder::new(4);

        let data = vec![0, 1, 3, 2, 1, 0, 2, 3, 1, 2];
        let encoded = enc.encode_symbols(&data);
        let decoded = dec.decode_symbols(&encoded);

        assert_eq!(data, decoded, "DQPSK roundtrip failed");
    }

    #[test]
    fn test_d8psk_roundtrip() {
        let mut enc = DiffEncoder::new(8);
        let mut dec = DiffDecoder::new(8);

        let data: Vec<u8> = (0..20).map(|i| (i % 8) as u8).collect();
        let encoded = enc.encode_symbols(&data);
        let decoded = dec.decode_symbols(&encoded);

        assert_eq!(data, decoded, "D8PSK roundtrip failed");
    }

    #[test]
    fn test_encoder_accumulates() {
        let mut enc = DiffEncoder::new(4);

        // Starting from 0:
        // Input: 1 → output: (1+0)%4 = 1
        // Input: 1 → output: (1+1)%4 = 2
        // Input: 1 → output: (1+2)%4 = 3
        // Input: 1 → output: (1+3)%4 = 0 (wraps)
        let out = enc.encode_symbols(&[1, 1, 1, 1]);
        assert_eq!(out, vec![1, 2, 3, 0]);
    }

    #[test]
    fn test_complex_dbpsk_roundtrip() {
        let mut enc = ComplexDiffEncoder::new();
        let mut dec = ComplexDiffDecoder::new();

        // BPSK symbols: +1 and -1
        let symbols = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
            Complex64::new(-1.0, 0.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(-1.0, 0.0),
        ];

        let encoded = enc.encode_symbols(&symbols);
        let decoded = dec.decode_symbols(&encoded);

        for (i, (orig, dec)) in symbols.iter().zip(decoded.iter()).enumerate() {
            assert!(
                (orig - dec).norm() < 0.01,
                "Complex DBPSK roundtrip failed at {i}: orig={orig:.3}, dec={dec:.3}"
            );
        }
    }

    #[test]
    fn test_complex_dqpsk_roundtrip() {
        let mut enc = ComplexDiffEncoder::new();
        let mut dec = ComplexDiffDecoder::new();

        let constellation = dpsk_constellation(4);
        let symbols: Vec<Complex64> = (0..10).map(|i| constellation[i % 4]).collect();

        let encoded = enc.encode_symbols(&symbols);
        let decoded = dec.decode_symbols(&encoded);

        for (i, (orig, dec)) in symbols.iter().zip(decoded.iter()).enumerate() {
            assert!(
                (orig - dec).norm() < 0.01,
                "Complex DQPSK roundtrip failed at {i}: orig={orig:.3}, dec={dec:.3}"
            );
        }
    }

    #[test]
    fn test_dpsk_constellation() {
        let c = dpsk_constellation(4);
        assert_eq!(c.len(), 4);
        // Should be at 0°, 90°, 180°, 270°
        assert!((c[0] - Complex64::new(1.0, 0.0)).norm() < 1e-10);
        assert!((c[1] - Complex64::new(0.0, 1.0)).norm() < 1e-10);
        assert!((c[2] - Complex64::new(-1.0, 0.0)).norm() < 1e-10);
        assert!((c[3] - Complex64::new(0.0, -1.0)).norm() < 1e-10);
    }

    #[test]
    fn test_reset() {
        let mut enc = DiffEncoder::new(4);
        let data = vec![1, 2, 3];

        let out1 = enc.encode_symbols(&data);
        enc.reset();
        let out2 = enc.encode_symbols(&data);

        assert_eq!(out1, out2, "Reset should produce identical output");
    }

    #[test]
    fn test_phase_ambiguity_resolution() {
        // Key property: differential encoding resolves 180° ambiguity
        let mut enc = DiffEncoder::new(2);
        let mut dec = DiffDecoder::new(2);

        let data = vec![0, 1, 0, 1, 1, 0];
        let encoded = enc.encode_symbols(&data);

        // Simulate 180° phase rotation: flip all encoded symbols
        let rotated: Vec<u8> = encoded.iter().map(|&s| (s + 1) % 2).collect();

        let decoded = dec.decode_symbols(&rotated);
        // After first symbol (which may be wrong), all subsequent are correct
        assert_eq!(
            &data[1..],
            &decoded[1..],
            "Differential should resolve phase ambiguity after first symbol"
        );
    }
}
