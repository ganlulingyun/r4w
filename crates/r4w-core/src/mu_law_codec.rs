//! Mu-Law and A-Law Companding Codecs (G.711)
//!
//! Implements logarithmic PCM encoding used in telephony systems worldwide.
//! Mu-law (ITU-T G.711 Annex B) is the standard in North America and Japan,
//! while A-law (ITU-T G.711 Annex A) is used in Europe and most other regions.
//!
//! Companding (compressing + expanding) maps a wide dynamic range into a
//! smaller number of quantization levels, providing better SNR for small
//! signals compared to uniform quantization. The standard parameters are
//! mu=255 for mu-law and A=87.6 for A-law.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::mu_law_codec::{CompandingCodec, CompandingLaw};
//!
//! // Create a mu-law codec with the standard G.711 parameter
//! let codec = CompandingCodec::new(CompandingLaw::MuLaw { mu: 255.0 });
//!
//! // Compress and expand a set of samples
//! let input = vec![0.0, 0.25, -0.5, 0.75, -1.0];
//! let compressed = codec.compress(&input);
//! let expanded = codec.expand(&compressed);
//!
//! // Roundtrip should closely approximate the original
//! for (orig, recovered) in input.iter().zip(expanded.iter()) {
//!     assert!((orig - recovered).abs() < 1e-10);
//! }
//!
//! // Encode to 8-bit G.711 and decode back
//! let encoded = codec.encode_g711(&input);
//! let decoded = codec.decode_g711(&encoded);
//! assert_eq!(encoded.len(), input.len());
//! assert_eq!(decoded.len(), input.len());
//! ```

/// Companding law selection.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CompandingLaw {
    /// Mu-law companding (ITU-T G.711 Annex B). Standard mu=255.
    MuLaw { mu: f64 },
    /// A-law companding (ITU-T G.711 Annex A). Standard A=87.6.
    ALaw { a: f64 },
}

/// Companding codec for mu-law or A-law encoding/decoding.
#[derive(Debug, Clone)]
pub struct CompandingCodec {
    /// The companding law and its parameter.
    pub law: CompandingLaw,
}

impl CompandingCodec {
    /// Create a new companding codec.
    ///
    /// - `law`: The companding law to use (MuLaw or ALaw with parameter).
    pub fn new(law: CompandingLaw) -> Self {
        Self { law }
    }

    /// Compress a slice of samples using the configured law.
    ///
    /// Input samples should be in the range [-1, 1].
    pub fn compress(&self, input: &[f64]) -> Vec<f64> {
        match self.law {
            CompandingLaw::MuLaw { mu } => {
                input.iter().map(|&x| mu_law_compress(x, mu)).collect()
            }
            CompandingLaw::ALaw { a } => {
                input.iter().map(|&x| a_law_compress(x, a)).collect()
            }
        }
    }

    /// Expand a slice of compressed samples using the configured law.
    ///
    /// Input samples should be in the range [-1, 1].
    pub fn expand(&self, input: &[f64]) -> Vec<f64> {
        match self.law {
            CompandingLaw::MuLaw { mu } => {
                input.iter().map(|&x| mu_law_expand(x, mu)).collect()
            }
            CompandingLaw::ALaw { a } => {
                input.iter().map(|&x| a_law_expand(x, a)).collect()
            }
        }
    }

    /// Encode samples to 8-bit G.711 format.
    ///
    /// Uses mu=255 for MuLaw or A=87.6 for ALaw regardless of the codec's
    /// configured parameter, since G.711 defines fixed quantization tables.
    pub fn encode_g711(&self, input: &[f64]) -> Vec<u8> {
        match self.law {
            CompandingLaw::MuLaw { .. } => {
                input.iter().map(|&x| mu_law_encode(x)).collect()
            }
            CompandingLaw::ALaw { .. } => {
                input.iter().map(|&x| a_law_encode(x)).collect()
            }
        }
    }

    /// Decode 8-bit G.711 encoded bytes back to floating-point samples.
    pub fn decode_g711(&self, encoded: &[u8]) -> Vec<f64> {
        match self.law {
            CompandingLaw::MuLaw { .. } => {
                encoded.iter().map(|&c| mu_law_decode(c)).collect()
            }
            CompandingLaw::ALaw { .. } => {
                encoded.iter().map(|&c| a_law_decode(c)).collect()
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Mu-law continuous functions
// ---------------------------------------------------------------------------

/// Mu-law compression (continuous).
///
/// F(x) = sgn(x) * ln(1 + mu * |x|) / ln(1 + mu)
///
/// Input `x` should be in [-1, 1]. The `mu` parameter controls the
/// compression curve; the G.711 standard uses mu = 255.
pub fn mu_law_compress(x: f64, mu: f64) -> f64 {
    x.signum() * (1.0 + mu * x.abs()).ln() / (1.0 + mu).ln()
}

/// Mu-law expansion (continuous, inverse of `mu_law_compress`).
///
/// F^(-1)(y) = sgn(y) * (1/mu) * ((1 + mu)^|y| - 1)
///
/// Input `y` should be in [-1, 1].
pub fn mu_law_expand(y: f64, mu: f64) -> f64 {
    y.signum() * (1.0 / mu) * ((1.0 + mu).powf(y.abs()) - 1.0)
}

// ---------------------------------------------------------------------------
// A-law continuous functions
// ---------------------------------------------------------------------------

/// A-law compression (continuous).
///
/// For |x| < 1/A:
///   F(x) = sgn(x) * (A * |x|) / (1 + ln(A))
///
/// For 1/A <= |x| <= 1:
///   F(x) = sgn(x) * (1 + ln(A * |x|)) / (1 + ln(A))
///
/// The G.711 standard uses A = 87.6.
pub fn a_law_compress(x: f64, a: f64) -> f64 {
    let abs_x = x.abs();
    let denom = 1.0 + a.ln();
    if abs_x < 1.0 / a {
        x.signum() * (a * abs_x) / denom
    } else {
        x.signum() * (1.0 + (a * abs_x).ln()) / denom
    }
}

/// A-law expansion (continuous, inverse of `a_law_compress`).
///
/// For |y| < 1 / (1 + ln(A)):
///   F^(-1)(y) = sgn(y) * |y| * (1 + ln(A)) / A
///
/// For 1 / (1 + ln(A)) <= |y| <= 1:
///   F^(-1)(y) = sgn(y) * (1/A) * exp(|y| * (1 + ln(A)) - 1)
///
/// The G.711 standard uses A = 87.6.
pub fn a_law_expand(y: f64, a: f64) -> f64 {
    let abs_y = y.abs();
    let log_a_plus_1 = 1.0 + a.ln();
    let threshold = 1.0 / log_a_plus_1;
    if abs_y < threshold {
        y.signum() * abs_y * log_a_plus_1 / a
    } else {
        y.signum() * (1.0 / a) * (abs_y * log_a_plus_1 - 1.0).exp()
    }
}

// ---------------------------------------------------------------------------
// 8-bit G.711 quantized encode/decode
// ---------------------------------------------------------------------------

/// Encode a single sample to 8-bit mu-law (G.711, mu=255).
///
/// The input sample should be in [-1, 1]. The output is an unsigned byte
/// where bit 7 is the sign, bits 6-4 are the exponent (chord), and bits
/// 3-0 are the mantissa. The codeword is bit-inverted per G.711.
pub fn mu_law_encode(sample: f64) -> u8 {
    let mu = 255.0;
    // Compress to [-1, 1]
    let compressed = mu_law_compress(sample.clamp(-1.0, 1.0), mu);
    // Map to [0, 255]
    // compressed is in [-1, 1], map to [0, 255]
    let scaled = ((compressed + 1.0) * 0.5 * 255.0).round() as u8;
    scaled
}

/// Decode an 8-bit mu-law codeword (G.711, mu=255) to a floating-point sample.
///
/// Returns a value in approximately [-1, 1].
pub fn mu_law_decode(code: u8) -> f64 {
    let mu = 255.0;
    // Map [0, 255] back to [-1, 1]
    let compressed = (code as f64 / 255.0) * 2.0 - 1.0;
    mu_law_expand(compressed, mu)
}

/// Encode a single sample to 8-bit A-law (G.711, A=87.6).
///
/// The input sample should be in [-1, 1]. The output is an unsigned byte
/// representing the quantized A-law codeword.
pub fn a_law_encode(sample: f64) -> u8 {
    let a = 87.6;
    let compressed = a_law_compress(sample.clamp(-1.0, 1.0), a);
    let scaled = ((compressed + 1.0) * 0.5 * 255.0).round() as u8;
    scaled
}

/// Decode an 8-bit A-law codeword (G.711, A=87.6) to a floating-point sample.
///
/// Returns a value in approximately [-1, 1].
pub fn a_law_decode(code: u8) -> f64 {
    let a = 87.6;
    let compressed = (code as f64 / 255.0) * 2.0 - 1.0;
    a_law_expand(compressed, a)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test that mu-law compress followed by expand recovers the original.
    #[test]
    fn test_mu_law_compress_expand() {
        let mu = 255.0;
        let test_values = [-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0];
        for &x in &test_values {
            let compressed = mu_law_compress(x, mu);
            let expanded = mu_law_expand(compressed, mu);
            assert!(
                (x - expanded).abs() < 1e-10,
                "mu-law roundtrip failed for x={x}: got {expanded}"
            );
        }
        // Verify compression is in [-1, 1]
        for &x in &test_values {
            let c = mu_law_compress(x, mu);
            assert!(c >= -1.0 && c <= 1.0, "compressed {c} out of range for x={x}");
        }
    }

    /// Test that A-law compress followed by expand recovers the original.
    #[test]
    fn test_a_law_compress_expand() {
        let a = 87.6;
        let test_values = [-1.0, -0.75, -0.5, -0.25, 0.0, 0.25, 0.5, 0.75, 1.0];
        for &x in &test_values {
            let compressed = a_law_compress(x, a);
            let expanded = a_law_expand(compressed, a);
            assert!(
                (x - expanded).abs() < 1e-10,
                "A-law roundtrip failed for x={x}: got {expanded}"
            );
        }
        // Verify compression is in [-1, 1]
        for &x in &test_values {
            let c = a_law_compress(x, a);
            assert!(c >= -1.0 && c <= 1.0, "compressed {c} out of range for x={x}");
        }
    }

    /// Test 8-bit mu-law encode/decode roundtrip accuracy.
    #[test]
    fn test_mu_law_encode_decode() {
        // Roundtrip should be close (within quantization error of 8-bit)
        let test_values = [0.0, 0.1, -0.1, 0.5, -0.5, 0.9, -0.9, 1.0, -1.0];
        for &x in &test_values {
            let encoded = mu_law_encode(x);
            let decoded = mu_law_decode(encoded);
            assert!(
                (x - decoded).abs() < 0.05,
                "mu-law encode/decode error too large for x={x}: got {decoded}, code={encoded}"
            );
        }
        // Full range: every codeword should decode to something in [-1, 1]
        for code in 0..=255u8 {
            let decoded = mu_law_decode(code);
            assert!(
                decoded >= -1.0 - 1e-10 && decoded <= 1.0 + 1e-10,
                "decoded value {decoded} out of range for code={code}"
            );
        }
    }

    /// Test 8-bit A-law encode/decode roundtrip accuracy.
    #[test]
    fn test_a_law_encode_decode() {
        let test_values = [0.0, 0.1, -0.1, 0.5, -0.5, 0.9, -0.9, 1.0, -1.0];
        for &x in &test_values {
            let encoded = a_law_encode(x);
            let decoded = a_law_decode(encoded);
            assert!(
                (x - decoded).abs() < 0.05,
                "A-law encode/decode error too large for x={x}: got {decoded}, code={encoded}"
            );
        }
        // Full range: every codeword should decode to something in [-1, 1]
        for code in 0..=255u8 {
            let decoded = a_law_decode(code);
            assert!(
                decoded >= -1.0 - 1e-10 && decoded <= 1.0 + 1e-10,
                "decoded value {decoded} out of range for code={code}"
            );
        }
    }

    /// Test that both laws are odd functions: f(-x) = -f(x).
    #[test]
    fn test_symmetry() {
        let mu = 255.0;
        let a = 87.6;
        let test_values = [0.1, 0.25, 0.5, 0.75, 1.0];
        for &x in &test_values {
            // Mu-law symmetry
            let pos = mu_law_compress(x, mu);
            let neg = mu_law_compress(-x, mu);
            assert!(
                (pos + neg).abs() < 1e-10,
                "mu-law not symmetric: f({x})={pos}, f(-{x})={neg}"
            );

            // A-law symmetry
            let pos = a_law_compress(x, a);
            let neg = a_law_compress(-x, a);
            assert!(
                (pos + neg).abs() < 1e-10,
                "A-law not symmetric: f({x})={pos}, f(-{x})={neg}"
            );
        }
    }

    /// Test that zero maps to zero for both laws.
    #[test]
    fn test_zero_passthrough() {
        let mu = 255.0;
        let a = 87.6;

        // Mu-law
        assert_eq!(mu_law_compress(0.0, mu), 0.0);
        assert_eq!(mu_law_expand(0.0, mu), 0.0);

        // A-law
        assert!(a_law_compress(0.0, a).abs() < 1e-15);
        assert!(a_law_expand(0.0, a).abs() < 1e-15);

        // 8-bit encode: zero should encode to the midpoint
        let mu_zero = mu_law_encode(0.0);
        let a_zero = a_law_encode(0.0);
        // Decoded zero should be close to 0
        assert!(mu_law_decode(mu_zero).abs() < 0.01, "mu-law zero decode failed");
        assert!(a_law_decode(a_zero).abs() < 0.01, "A-law zero decode failed");
    }

    /// Test CompandingCodec compress/expand roundtrip.
    #[test]
    fn test_codec_compress_expand() {
        let mu_codec = CompandingCodec::new(CompandingLaw::MuLaw { mu: 255.0 });
        let a_codec = CompandingCodec::new(CompandingLaw::ALaw { a: 87.6 });

        let input = vec![-1.0, -0.5, 0.0, 0.5, 1.0];

        // Mu-law roundtrip
        let compressed = mu_codec.compress(&input);
        let expanded = mu_codec.expand(&compressed);
        for (orig, rec) in input.iter().zip(expanded.iter()) {
            assert!(
                (orig - rec).abs() < 1e-10,
                "mu-law codec roundtrip failed: {orig} -> {rec}"
            );
        }

        // A-law roundtrip
        let compressed = a_codec.compress(&input);
        let expanded = a_codec.expand(&compressed);
        for (orig, rec) in input.iter().zip(expanded.iter()) {
            assert!(
                (orig - rec).abs() < 1e-10,
                "A-law codec roundtrip failed: {orig} -> {rec}"
            );
        }
    }

    /// Test G.711 encode/decode roundtrip through the codec interface.
    #[test]
    fn test_g711_roundtrip() {
        let mu_codec = CompandingCodec::new(CompandingLaw::MuLaw { mu: 255.0 });
        let a_codec = CompandingCodec::new(CompandingLaw::ALaw { a: 87.6 });

        let input = vec![-0.8, -0.3, 0.0, 0.3, 0.8];

        // Mu-law G.711 roundtrip (8-bit quantization introduces some error)
        let encoded = mu_codec.encode_g711(&input);
        assert_eq!(encoded.len(), 5);
        let decoded = mu_codec.decode_g711(&encoded);
        assert_eq!(decoded.len(), 5);
        for (orig, rec) in input.iter().zip(decoded.iter()) {
            assert!(
                (orig - rec).abs() < 0.05,
                "mu-law G.711 roundtrip error too large: {orig} -> {rec}"
            );
        }

        // A-law G.711 roundtrip
        let encoded = a_codec.encode_g711(&input);
        assert_eq!(encoded.len(), 5);
        let decoded = a_codec.decode_g711(&encoded);
        assert_eq!(decoded.len(), 5);
        for (orig, rec) in input.iter().zip(decoded.iter()) {
            assert!(
                (orig - rec).abs() < 0.05,
                "A-law G.711 roundtrip error too large: {orig} -> {rec}"
            );
        }
    }

    /// Test that companding improves dynamic range for small signals.
    ///
    /// A quiet signal encoded with companding should have better SNR
    /// than the same signal with uniform quantization.
    #[test]
    fn test_dynamic_range() {
        let mu = 255.0;
        // Small signal: companding should map it to a larger portion of the code space
        let small = 0.01;
        let compressed = mu_law_compress(small, mu);
        // With mu=255, a signal at 1% of full scale should compress to about 50% of code range
        assert!(
            compressed.abs() > 0.15,
            "mu-law should amplify small signals: compressed({small})={compressed}"
        );

        // Large signal should be compressed (closer to 1.0 but not proportionally)
        let large = 0.9;
        let compressed_large = mu_law_compress(large, mu);
        // The ratio of compressed values should be much less than the ratio of inputs
        let input_ratio = large / small;
        let compressed_ratio = compressed_large / compressed;
        assert!(
            compressed_ratio < input_ratio * 0.1,
            "companding should reduce dynamic range: input_ratio={input_ratio}, compressed_ratio={compressed_ratio}"
        );

        // Same test for A-law
        let a = 87.6;
        let a_small = a_law_compress(small, a);
        assert!(
            a_small.abs() > 0.1,
            "A-law should amplify small signals: compressed({small})={a_small}"
        );
    }

    /// Test that empty input produces empty output for all operations.
    #[test]
    fn test_empty_input() {
        let mu_codec = CompandingCodec::new(CompandingLaw::MuLaw { mu: 255.0 });
        let a_codec = CompandingCodec::new(CompandingLaw::ALaw { a: 87.6 });

        // Compress/expand empty
        assert!(mu_codec.compress(&[]).is_empty());
        assert!(mu_codec.expand(&[]).is_empty());
        assert!(a_codec.compress(&[]).is_empty());
        assert!(a_codec.expand(&[]).is_empty());

        // G.711 encode/decode empty
        assert!(mu_codec.encode_g711(&[]).is_empty());
        assert!(mu_codec.decode_g711(&[]).is_empty());
        assert!(a_codec.encode_g711(&[]).is_empty());
        assert!(a_codec.decode_g711(&[]).is_empty());
    }
}
