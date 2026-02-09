//! Interleaved Format Conversions — SDR hardware I/Q data type conversion
//!
//! Converts between interleaved integer formats (i16, i8, u8) and Complex64
//! for interfacing with SDR hardware. Most SDR devices (USRP, HackRF, RTL-SDR,
//! PlutoSDR) output interleaved integer I/Q pairs: [I₀, Q₀, I₁, Q₁, ...].
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::interleaved::{InterleavedShortToComplex, ComplexToInterleavedShort};
//! use num_complex::Complex64;
//!
//! // SDR hardware → Complex64
//! let raw_iq: Vec<i16> = vec![1000, -500, 2000, 1000];
//! let complex = InterleavedShortToComplex::convert(&raw_iq);
//! assert_eq!(complex.len(), 2); // 4 i16s → 2 complex samples
//!
//! // Complex64 → SDR hardware
//! let shorts = ComplexToInterleavedShort::convert(&complex);
//! assert_eq!(shorts.len(), 4);
//! ```

use num_complex::Complex64;

/// Convert interleaved i16 [I,Q,I,Q,...] to Complex64 with normalization.
pub struct InterleavedShortToComplex;

impl InterleavedShortToComplex {
    /// Convert interleaved i16 to Complex64, normalized to [-1, 1].
    pub fn convert(input: &[i16]) -> Vec<Complex64> {
        input.chunks_exact(2)
            .map(|pair| Complex64::new(
                pair[0] as f64 / 32768.0,
                pair[1] as f64 / 32768.0,
            ))
            .collect()
    }

    /// Convert with custom scale factor.
    pub fn convert_scaled(input: &[i16], scale: f64) -> Vec<Complex64> {
        input.chunks_exact(2)
            .map(|pair| Complex64::new(
                pair[0] as f64 * scale,
                pair[1] as f64 * scale,
            ))
            .collect()
    }

    /// Convert without normalization (raw integer values as f64).
    pub fn convert_raw(input: &[i16]) -> Vec<Complex64> {
        input.chunks_exact(2)
            .map(|pair| Complex64::new(pair[0] as f64, pair[1] as f64))
            .collect()
    }
}

/// Convert Complex64 to interleaved i16 [I,Q,I,Q,...].
pub struct ComplexToInterleavedShort;

impl ComplexToInterleavedShort {
    /// Convert Complex64 to interleaved i16, expecting input in [-1, 1].
    pub fn convert(input: &[Complex64]) -> Vec<i16> {
        let mut output = Vec::with_capacity(input.len() * 2);
        for &s in input {
            output.push((s.re * 32767.0).clamp(-32768.0, 32767.0) as i16);
            output.push((s.im * 32767.0).clamp(-32768.0, 32767.0) as i16);
        }
        output
    }

    /// Convert with custom scale factor.
    pub fn convert_scaled(input: &[Complex64], scale: f64) -> Vec<i16> {
        let mut output = Vec::with_capacity(input.len() * 2);
        for &s in input {
            output.push((s.re * scale).clamp(-32768.0, 32767.0) as i16);
            output.push((s.im * scale).clamp(-32768.0, 32767.0) as i16);
        }
        output
    }
}

/// Convert interleaved i8 [I,Q,I,Q,...] to Complex64 (HackRF format).
pub struct InterleavedCharToComplex;

impl InterleavedCharToComplex {
    /// Convert interleaved i8 to Complex64, normalized to [-1, 1].
    pub fn convert(input: &[i8]) -> Vec<Complex64> {
        input.chunks_exact(2)
            .map(|pair| Complex64::new(
                pair[0] as f64 / 128.0,
                pair[1] as f64 / 128.0,
            ))
            .collect()
    }

    /// Convert without normalization.
    pub fn convert_raw(input: &[i8]) -> Vec<Complex64> {
        input.chunks_exact(2)
            .map(|pair| Complex64::new(pair[0] as f64, pair[1] as f64))
            .collect()
    }
}

/// Convert Complex64 to interleaved i8.
pub struct ComplexToInterleavedChar;

impl ComplexToInterleavedChar {
    pub fn convert(input: &[Complex64]) -> Vec<i8> {
        let mut output = Vec::with_capacity(input.len() * 2);
        for &s in input {
            output.push((s.re * 127.0).clamp(-128.0, 127.0) as i8);
            output.push((s.im * 127.0).clamp(-128.0, 127.0) as i8);
        }
        output
    }
}

/// Convert interleaved u8 [I,Q,I,Q,...] to Complex64 (RTL-SDR format).
///
/// RTL-SDR outputs unsigned 8-bit with 128 as DC center.
pub struct InterleavedUCharToComplex;

impl InterleavedUCharToComplex {
    /// Convert RTL-SDR format (unsigned, centered at 128) to Complex64.
    pub fn convert(input: &[u8]) -> Vec<Complex64> {
        input.chunks_exact(2)
            .map(|pair| Complex64::new(
                (pair[0] as f64 - 128.0) / 128.0,
                (pair[1] as f64 - 128.0) / 128.0,
            ))
            .collect()
    }

    /// Convert without DC offset removal (raw 0-255 → 0.0-1.0).
    pub fn convert_raw(input: &[u8]) -> Vec<Complex64> {
        input.chunks_exact(2)
            .map(|pair| Complex64::new(
                pair[0] as f64 / 255.0,
                pair[1] as f64 / 255.0,
            ))
            .collect()
    }
}

/// Convert Complex64 to interleaved u8 (RTL-SDR compatible).
pub struct ComplexToInterleavedUChar;

impl ComplexToInterleavedUChar {
    /// Convert Complex64 (expected [-1, 1]) to unsigned 8-bit centered at 128.
    pub fn convert(input: &[Complex64]) -> Vec<u8> {
        let mut output = Vec::with_capacity(input.len() * 2);
        for &s in input {
            output.push((s.re * 128.0 + 128.0).clamp(0.0, 255.0) as u8);
            output.push((s.im * 128.0 + 128.0).clamp(0.0, 255.0) as u8);
        }
        output
    }
}

/// Convert interleaved f32 [I,Q,I,Q,...] to Complex64 (USRP/Ettus format).
pub struct InterleavedFloatToComplex;

impl InterleavedFloatToComplex {
    pub fn convert(input: &[f32]) -> Vec<Complex64> {
        input.chunks_exact(2)
            .map(|pair| Complex64::new(pair[0] as f64, pair[1] as f64))
            .collect()
    }
}

/// Convert Complex64 to interleaved f32.
pub struct ComplexToInterleavedFloat;

impl ComplexToInterleavedFloat {
    pub fn convert(input: &[Complex64]) -> Vec<f32> {
        let mut output = Vec::with_capacity(input.len() * 2);
        for &s in input {
            output.push(s.re as f32);
            output.push(s.im as f32);
        }
        output
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_short_to_complex() {
        let input = vec![16384_i16, -16384, 0, 32767];
        let output = InterleavedShortToComplex::convert(&input);
        assert_eq!(output.len(), 2);
        assert!((output[0].re - 0.5).abs() < 0.001);
        assert!((output[0].im + 0.5).abs() < 0.001);
        assert!((output[1].re).abs() < 0.001);
        assert!((output[1].im - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_complex_to_short() {
        let input = vec![
            Complex64::new(0.5, -0.5),
            Complex64::new(0.0, 1.0),
        ];
        let output = ComplexToInterleavedShort::convert(&input);
        assert_eq!(output.len(), 4);
        assert!((output[0] - 16383).abs() < 2); // 0.5 * 32767
        assert!((output[1] + 16383).abs() < 2);
    }

    #[test]
    fn test_short_roundtrip() {
        let original = vec![1000_i16, -2000, 3000, -4000, 0, 0];
        let complex = InterleavedShortToComplex::convert(&original);
        let back = ComplexToInterleavedShort::convert(&complex);
        for (a, b) in original.iter().zip(back.iter()) {
            assert!((*a as i32 - *b as i32).abs() <= 1, "Roundtrip mismatch: {} vs {}", a, b);
        }
    }

    #[test]
    fn test_char_to_complex() {
        let input = vec![127_i8, -128, 0, 64];
        let output = InterleavedCharToComplex::convert(&input);
        assert_eq!(output.len(), 2);
        assert!((output[0].re - (127.0 / 128.0)).abs() < 0.01);
        assert!((output[0].im + 1.0).abs() < 0.01);
    }

    #[test]
    fn test_complex_to_char() {
        let input = vec![Complex64::new(0.5, -0.5)];
        let output = ComplexToInterleavedChar::convert(&input);
        assert_eq!(output.len(), 2);
        assert!((output[0] as f64 - 63.5).abs() < 1.5);
        assert!((output[1] as f64 + 63.5).abs() < 1.5);
    }

    #[test]
    fn test_uchar_to_complex() {
        // RTL-SDR: 128 = DC center → 0.0
        let input = vec![128_u8, 128, 255, 0];
        let output = InterleavedUCharToComplex::convert(&input);
        assert_eq!(output.len(), 2);
        assert!(output[0].re.abs() < 0.01); // 128 → 0
        assert!(output[0].im.abs() < 0.01);
        assert!((output[1].re - (127.0 / 128.0)).abs() < 0.01); // 255 → ~1.0
        assert!((output[1].im + 1.0).abs() < 0.01); // 0 → -1.0
    }

    #[test]
    fn test_complex_to_uchar() {
        let input = vec![Complex64::new(0.0, 0.0)]; // DC center
        let output = ComplexToInterleavedUChar::convert(&input);
        assert_eq!(output.len(), 2);
        assert_eq!(output[0], 128); // 0.0 → 128
        assert_eq!(output[1], 128);
    }

    #[test]
    fn test_uchar_roundtrip() {
        let original = vec![100_u8, 200, 50, 150, 128, 128];
        let complex = InterleavedUCharToComplex::convert(&original);
        let back = ComplexToInterleavedUChar::convert(&complex);
        for (a, b) in original.iter().zip(back.iter()) {
            assert!((*a as i32 - *b as i32).abs() <= 1, "Roundtrip mismatch: {} vs {}", a, b);
        }
    }

    #[test]
    fn test_float_to_complex() {
        let input = vec![0.5_f32, -0.25, 1.0, 0.0];
        let output = InterleavedFloatToComplex::convert(&input);
        assert_eq!(output.len(), 2);
        assert!((output[0].re - 0.5).abs() < 1e-6);
        assert!((output[0].im + 0.25).abs() < 1e-6);
    }

    #[test]
    fn test_complex_to_float() {
        let input = vec![Complex64::new(0.5, -0.25)];
        let output = ComplexToInterleavedFloat::convert(&input);
        assert_eq!(output.len(), 2);
        assert!((output[0] - 0.5).abs() < 1e-6);
        assert!((output[1] + 0.25).abs() < 1e-6);
    }

    #[test]
    fn test_odd_length_ignored() {
        // Odd number of i16s — last sample ignored
        let input = vec![1000_i16, 2000, 3000];
        let output = InterleavedShortToComplex::convert(&input);
        assert_eq!(output.len(), 1); // Only one complete pair
    }

    #[test]
    fn test_empty_input() {
        assert!(InterleavedShortToComplex::convert(&[]).is_empty());
        assert!(InterleavedCharToComplex::convert(&[]).is_empty());
        assert!(InterleavedUCharToComplex::convert(&[]).is_empty());
        assert!(InterleavedFloatToComplex::convert(&[]).is_empty());
    }

    #[test]
    fn test_scaled_conversion() {
        let input = vec![10000_i16, -10000];
        let output = InterleavedShortToComplex::convert_scaled(&input, 0.001);
        assert!((output[0].re - 10.0).abs() < 0.01);
        assert!((output[0].im + 10.0).abs() < 0.01);
    }

    #[test]
    fn test_saturation() {
        // Values outside [-1, 1] should be clamped
        let input = vec![Complex64::new(2.0, -2.0)];
        let shorts = ComplexToInterleavedShort::convert(&input);
        assert_eq!(shorts[0], 32767); // Clamped to max
        assert!(shorts[1] <= -32767); // Clamped to min (-32768 from clamp)
    }
}
