//! # Complex to Interleaved Converter
//!
//! Converts between complex IQ sample representation and interleaved
//! real-valued format. Commonly needed for interfacing with hardware
//! SDRs and file formats that use interleaved I/Q.
//!
//! ## Formats
//!
//! - Complex: `Vec<(f64, f64)>` — each element is (I, Q)
//! - Interleaved: `Vec<f64>` — [I0, Q0, I1, Q1, I2, Q2, ...]
//! - Split: Two `Vec<f64>` — separate I and Q channels
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::complex_to_interleaved::{complex_to_interleaved, interleaved_to_complex};
//!
//! let complex = vec![(1.0, 2.0), (3.0, 4.0)];
//! let interleaved = complex_to_interleaved(&complex);
//! assert_eq!(interleaved, vec![1.0, 2.0, 3.0, 4.0]);
//!
//! let back = interleaved_to_complex(&interleaved);
//! assert_eq!(back, complex);
//! ```

/// Convert complex samples to interleaved real format.
pub fn complex_to_interleaved(complex: &[(f64, f64)]) -> Vec<f64> {
    let mut output = Vec::with_capacity(complex.len() * 2);
    for &(i, q) in complex {
        output.push(i);
        output.push(q);
    }
    output
}

/// Convert interleaved real samples to complex format.
///
/// If the input has an odd number of samples, the last sample is treated
/// as I with Q=0.
pub fn interleaved_to_complex(interleaved: &[f64]) -> Vec<(f64, f64)> {
    let mut output = Vec::with_capacity((interleaved.len() + 1) / 2);
    let mut i = 0;
    while i + 1 < interleaved.len() {
        output.push((interleaved[i], interleaved[i + 1]));
        i += 2;
    }
    if i < interleaved.len() {
        output.push((interleaved[i], 0.0));
    }
    output
}

/// Split complex samples into separate I and Q channels.
pub fn complex_to_split(complex: &[(f64, f64)]) -> (Vec<f64>, Vec<f64>) {
    let mut i_chan = Vec::with_capacity(complex.len());
    let mut q_chan = Vec::with_capacity(complex.len());
    for &(i, q) in complex {
        i_chan.push(i);
        q_chan.push(q);
    }
    (i_chan, q_chan)
}

/// Merge separate I and Q channels into complex samples.
pub fn split_to_complex(i_chan: &[f64], q_chan: &[f64]) -> Vec<(f64, f64)> {
    let len = i_chan.len().min(q_chan.len());
    (0..len).map(|k| (i_chan[k], q_chan[k])).collect()
}

/// Convert interleaved f32 samples to complex f64.
pub fn interleaved_f32_to_complex(interleaved: &[f32]) -> Vec<(f64, f64)> {
    let mut output = Vec::with_capacity((interleaved.len() + 1) / 2);
    let mut i = 0;
    while i + 1 < interleaved.len() {
        output.push((interleaved[i] as f64, interleaved[i + 1] as f64));
        i += 2;
    }
    if i < interleaved.len() {
        output.push((interleaved[i] as f64, 0.0));
    }
    output
}

/// Convert complex f64 to interleaved f32.
pub fn complex_to_interleaved_f32(complex: &[(f64, f64)]) -> Vec<f32> {
    let mut output = Vec::with_capacity(complex.len() * 2);
    for &(i, q) in complex {
        output.push(i as f32);
        output.push(q as f32);
    }
    output
}

/// Convert interleaved i16 samples to complex f64 (normalized to ±1.0).
pub fn interleaved_i16_to_complex(interleaved: &[i16]) -> Vec<(f64, f64)> {
    let scale = 1.0 / 32767.0;
    let mut output = Vec::with_capacity((interleaved.len() + 1) / 2);
    let mut i = 0;
    while i + 1 < interleaved.len() {
        output.push((
            interleaved[i] as f64 * scale,
            interleaved[i + 1] as f64 * scale,
        ));
        i += 2;
    }
    if i < interleaved.len() {
        output.push((interleaved[i] as f64 * scale, 0.0));
    }
    output
}

/// Convert complex f64 to interleaved i16 (scaled from ±1.0).
pub fn complex_to_interleaved_i16(complex: &[(f64, f64)]) -> Vec<i16> {
    let scale = 32767.0;
    let mut output = Vec::with_capacity(complex.len() * 2);
    for &(i, q) in complex {
        output.push((i * scale).clamp(-32768.0, 32767.0) as i16);
        output.push((q * scale).clamp(-32768.0, 32767.0) as i16);
    }
    output
}

/// Convert interleaved u8 samples to complex f64 (centered at 0, normalized).
pub fn interleaved_u8_to_complex(interleaved: &[u8]) -> Vec<(f64, f64)> {
    let scale = 1.0 / 127.5;
    let mut output = Vec::with_capacity((interleaved.len() + 1) / 2);
    let mut i = 0;
    while i + 1 < interleaved.len() {
        output.push((
            (interleaved[i] as f64 - 127.5) * scale,
            (interleaved[i + 1] as f64 - 127.5) * scale,
        ));
        i += 2;
    }
    if i < interleaved.len() {
        output.push(((interleaved[i] as f64 - 127.5) * scale, 0.0));
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_complex_to_interleaved() {
        let complex = vec![(1.0, 2.0), (3.0, 4.0), (5.0, 6.0)];
        let interleaved = complex_to_interleaved(&complex);
        assert_eq!(interleaved, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
    }

    #[test]
    fn test_interleaved_to_complex() {
        let interleaved = vec![1.0, 2.0, 3.0, 4.0];
        let complex = interleaved_to_complex(&interleaved);
        assert_eq!(complex, vec![(1.0, 2.0), (3.0, 4.0)]);
    }

    #[test]
    fn test_roundtrip() {
        let original = vec![(0.1, -0.2), (0.3, 0.4), (-0.5, 0.6)];
        let interleaved = complex_to_interleaved(&original);
        let recovered = interleaved_to_complex(&interleaved);
        assert_eq!(recovered, original);
    }

    #[test]
    fn test_odd_interleaved() {
        let interleaved = vec![1.0, 2.0, 3.0]; // odd count
        let complex = interleaved_to_complex(&interleaved);
        assert_eq!(complex, vec![(1.0, 2.0), (3.0, 0.0)]);
    }

    #[test]
    fn test_split_and_merge() {
        let complex = vec![(1.0, 2.0), (3.0, 4.0)];
        let (i_ch, q_ch) = complex_to_split(&complex);
        assert_eq!(i_ch, vec![1.0, 3.0]);
        assert_eq!(q_ch, vec![2.0, 4.0]);
        let merged = split_to_complex(&i_ch, &q_ch);
        assert_eq!(merged, complex);
    }

    #[test]
    fn test_f32_conversion() {
        let complex = vec![(1.0, -1.0), (0.5, 0.5)];
        let f32_data = complex_to_interleaved_f32(&complex);
        assert_eq!(f32_data.len(), 4);
        let back = interleaved_f32_to_complex(&f32_data);
        for (a, b) in complex.iter().zip(back.iter()) {
            assert!((a.0 - b.0).abs() < 1e-6);
            assert!((a.1 - b.1).abs() < 1e-6);
        }
    }

    #[test]
    fn test_i16_conversion() {
        let complex = vec![(1.0, -1.0), (0.0, 0.5)];
        let i16_data = complex_to_interleaved_i16(&complex);
        let back = interleaved_i16_to_complex(&i16_data);
        for (a, b) in complex.iter().zip(back.iter()) {
            assert!((a.0 - b.0).abs() < 0.001);
            assert!((a.1 - b.1).abs() < 0.001);
        }
    }

    #[test]
    fn test_u8_conversion() {
        // 127 → ~0.0, 255 → ~1.0, 0 → ~-1.0
        let u8_data = vec![127, 127, 255, 0];
        let complex = interleaved_u8_to_complex(&u8_data);
        assert!(complex[0].0.abs() < 0.01); // ~0
        assert!(complex[0].1.abs() < 0.01);
        assert!((complex[1].0 - 1.0).abs() < 0.01); // ~1.0
        assert!((complex[1].1 - (-1.0)).abs() < 0.02); // ~-1.0
    }

    #[test]
    fn test_empty() {
        assert!(complex_to_interleaved(&[]).is_empty());
        assert!(interleaved_to_complex(&[]).is_empty());
    }

    #[test]
    fn test_split_unequal_lengths() {
        let i_ch = vec![1.0, 2.0, 3.0];
        let q_ch = vec![4.0, 5.0];
        let merged = split_to_complex(&i_ch, &q_ch);
        assert_eq!(merged.len(), 2); // min of the two lengths
    }
}
