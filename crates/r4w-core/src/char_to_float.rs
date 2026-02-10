//! # Char to Float / Short to Float Type Converters
//!
//! Standard SDR type conversion blocks with configurable scaling.
//! Converts between integer types (u8, i8, i16) and floating point
//! (f32, f64) with proper normalization for SDR pipelines.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::char_to_float::{uchar_to_float, float_to_uchar};
//!
//! // RTL-SDR u8 samples → normalized float [-1, 1]
//! let raw = vec![0u8, 128, 255];
//! let normalized = uchar_to_float(&raw);
//! assert!((normalized[1] - 0.0).abs() < 0.01); // 128 → ~0.0
//! ```

/// Convert unsigned char (u8) to f64, centered at 0.
///
/// Maps [0, 255] to approximately [-1.0, +1.0] by subtracting 127.5 and dividing by 127.5.
pub fn uchar_to_float(input: &[u8]) -> Vec<f64> {
    input
        .iter()
        .map(|&x| (x as f64 - 127.5) / 127.5)
        .collect()
}

/// Convert f64 back to unsigned char.
///
/// Maps [-1.0, +1.0] to [0, 255].
pub fn float_to_uchar(input: &[f64]) -> Vec<u8> {
    input
        .iter()
        .map(|&x| {
            let val = (x * 127.5 + 127.5).round();
            val.clamp(0.0, 255.0) as u8
        })
        .collect()
}

/// Convert signed char (i8) to f64.
///
/// Maps [-128, 127] to approximately [-1.0, +1.0].
pub fn char_to_float(input: &[i8]) -> Vec<f64> {
    input.iter().map(|&x| x as f64 / 128.0).collect()
}

/// Convert f64 to signed char.
pub fn float_to_char(input: &[f64]) -> Vec<i8> {
    input
        .iter()
        .map(|&x| {
            let val = (x * 128.0).round();
            val.clamp(-128.0, 127.0) as i8
        })
        .collect()
}

/// Convert signed short (i16) to f64.
///
/// Maps [-32768, 32767] to approximately [-1.0, +1.0].
pub fn short_to_float(input: &[i16]) -> Vec<f64> {
    input.iter().map(|&x| x as f64 / 32768.0).collect()
}

/// Convert f64 to signed short (i16).
pub fn float_to_short(input: &[f64]) -> Vec<i16> {
    input
        .iter()
        .map(|&x| {
            let val = (x * 32768.0).round();
            val.clamp(-32768.0, 32767.0) as i16
        })
        .collect()
}

/// Convert unsigned char to f32 (single precision).
pub fn uchar_to_float32(input: &[u8]) -> Vec<f32> {
    input
        .iter()
        .map(|&x| (x as f32 - 127.5) / 127.5)
        .collect()
}

/// Convert f32 to unsigned char.
pub fn float32_to_uchar(input: &[f32]) -> Vec<u8> {
    input
        .iter()
        .map(|&x| {
            let val = (x * 127.5 + 127.5).round();
            val.clamp(0.0, 255.0) as u8
        })
        .collect()
}

/// Convert short to f32 (single precision).
pub fn short_to_float32(input: &[i16]) -> Vec<f32> {
    input.iter().map(|&x| x as f32 / 32768.0).collect()
}

/// Convert f32 to short.
pub fn float32_to_short(input: &[f32]) -> Vec<i16> {
    input
        .iter()
        .map(|&x| {
            let val = (x * 32768.0).round();
            val.clamp(-32768.0, 32767.0) as i16
        })
        .collect()
}

/// Convert interleaved u8 IQ samples to complex f64.
pub fn uchar_iq_to_complex(input: &[u8]) -> Vec<(f64, f64)> {
    input
        .chunks_exact(2)
        .map(|c| {
            let i = (c[0] as f64 - 127.5) / 127.5;
            let q = (c[1] as f64 - 127.5) / 127.5;
            (i, q)
        })
        .collect()
}

/// Convert complex f64 to interleaved u8 IQ samples.
pub fn complex_to_uchar_iq(input: &[(f64, f64)]) -> Vec<u8> {
    let mut output = Vec::with_capacity(input.len() * 2);
    for &(i, q) in input {
        output.push((i * 127.5 + 127.5).round().clamp(0.0, 255.0) as u8);
        output.push((q * 127.5 + 127.5).round().clamp(0.0, 255.0) as u8);
    }
    output
}

/// Convert interleaved i16 IQ samples to complex f64.
pub fn short_iq_to_complex(input: &[i16]) -> Vec<(f64, f64)> {
    input
        .chunks_exact(2)
        .map(|c| {
            let i = c[0] as f64 / 32768.0;
            let q = c[1] as f64 / 32768.0;
            (i, q)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_uchar_to_float() {
        let input = vec![0u8, 128, 255];
        let output = uchar_to_float(&input);
        assert!((output[0] + 1.0).abs() < 0.01);
        assert!(output[1].abs() < 0.01);
        assert!((output[2] - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_float_to_uchar() {
        let input = vec![-1.0, 0.0, 1.0];
        let output = float_to_uchar(&input);
        assert_eq!(output[0], 0);
        assert_eq!(output[1], 128);
        assert_eq!(output[2], 255);
    }

    #[test]
    fn test_uchar_roundtrip() {
        let original = vec![0u8, 64, 128, 192, 255];
        let floats = uchar_to_float(&original);
        let restored = float_to_uchar(&floats);
        assert_eq!(original, restored);
    }

    #[test]
    fn test_char_to_float() {
        let input = vec![-128i8, 0, 127];
        let output = char_to_float(&input);
        assert!((output[0] + 1.0).abs() < 0.01);
        assert!(output[1].abs() < 1e-10);
        assert!((output[2] - 127.0 / 128.0).abs() < 1e-10);
    }

    #[test]
    fn test_short_to_float() {
        let input = vec![-32768i16, 0, 32767];
        let output = short_to_float(&input);
        assert!((output[0] + 1.0).abs() < 0.001);
        assert!(output[1].abs() < 1e-10);
        assert!((output[2] - 32767.0 / 32768.0).abs() < 1e-10);
    }

    #[test]
    fn test_short_roundtrip() {
        let original = vec![-32768i16, -1000, 0, 1000, 32767];
        let floats = short_to_float(&original);
        let restored = float_to_short(&floats);
        assert_eq!(original, restored);
    }

    #[test]
    fn test_uchar_iq() {
        let raw = vec![0u8, 255, 128, 128];
        let complex = uchar_iq_to_complex(&raw);
        assert_eq!(complex.len(), 2);
        assert!((complex[0].0 + 1.0).abs() < 0.01); // I = 0 → -1
        assert!((complex[0].1 - 1.0).abs() < 0.01); // Q = 255 → +1
    }

    #[test]
    fn test_complex_to_uchar_roundtrip() {
        let original = vec![(0.5, -0.5), (0.0, 0.0)];
        let raw = complex_to_uchar_iq(&original);
        let restored = uchar_iq_to_complex(&raw);
        for (o, r) in original.iter().zip(restored.iter()) {
            assert!((o.0 - r.0).abs() < 0.02);
            assert!((o.1 - r.1).abs() < 0.02);
        }
    }

    #[test]
    fn test_short_iq() {
        let raw = vec![0i16, 16384, -16384, 0];
        let complex = short_iq_to_complex(&raw);
        assert_eq!(complex.len(), 2);
        assert!(complex[0].0.abs() < 1e-10);
        assert!((complex[0].1 - 0.5).abs() < 0.001);
    }

    #[test]
    fn test_clipping() {
        let input = vec![2.0, -2.0]; // Out of range.
        let output = float_to_uchar(&input);
        assert_eq!(output[0], 255);
        assert_eq!(output[1], 0);
    }
}
