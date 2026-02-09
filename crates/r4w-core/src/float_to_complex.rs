//! Float to Complex — Real/Imag ↔ Complex conversion
//!
//! Combines two real-valued streams (I and Q) into a complex stream,
//! or splits a complex stream into separate real and imaginary parts.
//! Essential for bridging real-valued DSP chains to complex baseband.
//! GNU Radio equivalent: `float_to_complex`, `complex_to_real`, `complex_to_imag`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::float_to_complex::{float_to_complex, complex_to_float};
//! use num_complex::Complex64;
//!
//! let i_data = vec![1.0, 2.0, 3.0];
//! let q_data = vec![4.0, 5.0, 6.0];
//! let complex = float_to_complex(&i_data, &q_data);
//! assert_eq!(complex[0], Complex64::new(1.0, 4.0));
//!
//! let (i_out, q_out) = complex_to_float(&complex);
//! assert_eq!(i_out, i_data);
//! assert_eq!(q_out, q_data);
//! ```

use num_complex::Complex64;

/// Combine two real streams (I and Q) into a complex stream.
///
/// Output length = min(i_data.len(), q_data.len()).
pub fn float_to_complex(i_data: &[f64], q_data: &[f64]) -> Vec<Complex64> {
    i_data
        .iter()
        .zip(q_data.iter())
        .map(|(&re, &im)| Complex64::new(re, im))
        .collect()
}

/// Combine a real stream with zero imaginary part.
///
/// Equivalent to `float_to_complex(data, &vec![0.0; data.len()])`.
pub fn float_to_complex_real(data: &[f64]) -> Vec<Complex64> {
    data.iter().map(|&re| Complex64::new(re, 0.0)).collect()
}

/// Split a complex stream into separate I and Q float streams.
pub fn complex_to_float(data: &[Complex64]) -> (Vec<f64>, Vec<f64>) {
    let i_data: Vec<f64> = data.iter().map(|c| c.re).collect();
    let q_data: Vec<f64> = data.iter().map(|c| c.im).collect();
    (i_data, q_data)
}

/// Extract the real (I) component of a complex stream.
pub fn complex_to_real(data: &[Complex64]) -> Vec<f64> {
    data.iter().map(|c| c.re).collect()
}

/// Extract the imaginary (Q) component of a complex stream.
pub fn complex_to_imag(data: &[Complex64]) -> Vec<f64> {
    data.iter().map(|c| c.im).collect()
}

/// Interleave I/Q floats into a flat buffer: [I0, Q0, I1, Q1, ...].
///
/// Common format for hardware interfaces (USRP, RTL-SDR).
pub fn float_to_interleaved(i_data: &[f64], q_data: &[f64]) -> Vec<f64> {
    let len = i_data.len().min(q_data.len());
    let mut out = Vec::with_capacity(len * 2);
    for k in 0..len {
        out.push(i_data[k]);
        out.push(q_data[k]);
    }
    out
}

/// Deinterleave a flat I/Q buffer into separate I and Q streams.
pub fn interleaved_to_float(data: &[f64]) -> (Vec<f64>, Vec<f64>) {
    let len = data.len() / 2;
    let mut i_data = Vec::with_capacity(len);
    let mut q_data = Vec::with_capacity(len);
    for chunk in data.chunks_exact(2) {
        i_data.push(chunk[0]);
        q_data.push(chunk[1]);
    }
    (i_data, q_data)
}

/// Convert interleaved I/Q floats directly to complex.
pub fn interleaved_to_complex(data: &[f64]) -> Vec<Complex64> {
    data.chunks_exact(2)
        .map(|c| Complex64::new(c[0], c[1]))
        .collect()
}

/// Convert complex to interleaved I/Q floats.
pub fn complex_to_interleaved(data: &[Complex64]) -> Vec<f64> {
    let mut out = Vec::with_capacity(data.len() * 2);
    for c in data {
        out.push(c.re);
        out.push(c.im);
    }
    out
}

/// Convert polar (magnitude, phase) pairs to complex.
pub fn polar_to_complex(mag: &[f64], phase: &[f64]) -> Vec<Complex64> {
    mag.iter()
        .zip(phase.iter())
        .map(|(&r, &theta)| Complex64::new(r * theta.cos(), r * theta.sin()))
        .collect()
}

/// Convert complex to polar (magnitude, phase) pairs.
pub fn complex_to_polar(data: &[Complex64]) -> (Vec<f64>, Vec<f64>) {
    let mag: Vec<f64> = data.iter().map(|c| c.norm()).collect();
    let phase: Vec<f64> = data.iter().map(|c| c.arg()).collect();
    (mag, phase)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_float_to_complex() {
        let i = vec![1.0, 2.0, 3.0];
        let q = vec![4.0, 5.0, 6.0];
        let c = float_to_complex(&i, &q);
        assert_eq!(c.len(), 3);
        assert_eq!(c[0], Complex64::new(1.0, 4.0));
        assert_eq!(c[2], Complex64::new(3.0, 6.0));
    }

    #[test]
    fn test_float_to_complex_real() {
        let data = vec![1.0, 2.0, 3.0];
        let c = float_to_complex_real(&data);
        assert_eq!(c[0], Complex64::new(1.0, 0.0));
        assert_eq!(c[1], Complex64::new(2.0, 0.0));
    }

    #[test]
    fn test_complex_to_float() {
        let c = vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, 4.0)];
        let (i, q) = complex_to_float(&c);
        assert_eq!(i, vec![1.0, 3.0]);
        assert_eq!(q, vec![2.0, 4.0]);
    }

    #[test]
    fn test_complex_to_real() {
        let c = vec![Complex64::new(5.0, 7.0)];
        assert_eq!(complex_to_real(&c), vec![5.0]);
    }

    #[test]
    fn test_complex_to_imag() {
        let c = vec![Complex64::new(5.0, 7.0)];
        assert_eq!(complex_to_imag(&c), vec![7.0]);
    }

    #[test]
    fn test_roundtrip() {
        let i = vec![1.0, 2.0, 3.0];
        let q = vec![4.0, 5.0, 6.0];
        let c = float_to_complex(&i, &q);
        let (i2, q2) = complex_to_float(&c);
        assert_eq!(i, i2);
        assert_eq!(q, q2);
    }

    #[test]
    fn test_unequal_lengths() {
        let i = vec![1.0, 2.0, 3.0];
        let q = vec![4.0, 5.0];
        let c = float_to_complex(&i, &q);
        assert_eq!(c.len(), 2); // Truncated to shorter
    }

    #[test]
    fn test_interleaved() {
        let i = vec![1.0, 2.0];
        let q = vec![3.0, 4.0];
        let interleaved = float_to_interleaved(&i, &q);
        assert_eq!(interleaved, vec![1.0, 3.0, 2.0, 4.0]);
    }

    #[test]
    fn test_deinterleave() {
        let data = vec![1.0, 3.0, 2.0, 4.0];
        let (i, q) = interleaved_to_float(&data);
        assert_eq!(i, vec![1.0, 2.0]);
        assert_eq!(q, vec![3.0, 4.0]);
    }

    #[test]
    fn test_interleaved_roundtrip() {
        let i = vec![1.0, 2.0, 3.0];
        let q = vec![4.0, 5.0, 6.0];
        let interleaved = float_to_interleaved(&i, &q);
        let (i2, q2) = interleaved_to_float(&interleaved);
        assert_eq!(i, i2);
        assert_eq!(q, q2);
    }

    #[test]
    fn test_interleaved_to_complex() {
        let data = vec![1.0, 2.0, 3.0, 4.0];
        let c = interleaved_to_complex(&data);
        assert_eq!(c[0], Complex64::new(1.0, 2.0));
        assert_eq!(c[1], Complex64::new(3.0, 4.0));
    }

    #[test]
    fn test_complex_to_interleaved() {
        let c = vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, 4.0)];
        let interleaved = complex_to_interleaved(&c);
        assert_eq!(interleaved, vec![1.0, 2.0, 3.0, 4.0]);
    }

    #[test]
    fn test_polar_to_complex() {
        let mag = vec![1.0, 2.0];
        let phase = vec![0.0, PI / 2.0];
        let c = polar_to_complex(&mag, &phase);
        assert!((c[0].re - 1.0).abs() < 1e-10);
        assert!(c[0].im.abs() < 1e-10);
        assert!(c[1].re.abs() < 1e-10);
        assert!((c[1].im - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_to_polar() {
        let c = vec![Complex64::new(3.0, 4.0)];
        let (mag, phase) = complex_to_polar(&c);
        assert!((mag[0] - 5.0).abs() < 1e-10);
        assert!((phase[0] - (4.0f64).atan2(3.0)).abs() < 1e-10);
    }

    #[test]
    fn test_polar_roundtrip() {
        let original = vec![Complex64::new(1.0, 1.0), Complex64::new(-2.0, 3.0)];
        let (mag, phase) = complex_to_polar(&original);
        let recovered = polar_to_complex(&mag, &phase);
        for (a, b) in original.iter().zip(recovered.iter()) {
            assert!((a.re - b.re).abs() < 1e-10);
            assert!((a.im - b.im).abs() < 1e-10);
        }
    }

    #[test]
    fn test_empty() {
        assert!(float_to_complex(&[], &[]).is_empty());
        let (i, q) = complex_to_float(&[]);
        assert!(i.is_empty());
        assert!(q.is_empty());
    }
}
