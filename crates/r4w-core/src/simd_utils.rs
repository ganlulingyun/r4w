//! SIMD-Optimized Utilities
//!
//! This module provides performance-optimized implementations of common DSP operations.
//! These functions are designed to encourage LLVM auto-vectorization on x86 (SSE/AVX)
//! and ARM (NEON) architectures.
//!
//! ## Design Principles
//!
//! 1. Use iterators and closures that LLVM can optimize well
//! 2. Avoid branches in inner loops
//! 3. Process data in chunks when possible
//! 4. Mark hot functions with `#[inline]`
//!
//! ## Performance Notes
//!
//! - On x86-64, compile with `RUSTFLAGS="-C target-cpu=native"` for best performance
//! - On ARM, compile with `RUSTFLAGS="-C target-cpu=native"` to enable NEON
//! - Profile with `cargo bench` to verify optimizations

use rustfft::num_complex::Complex64;

/// Compute magnitudes (|z|) of complex samples.
///
/// Vectorizes well on both x86 (SSE/AVX) and ARM (NEON).
#[inline]
pub fn compute_magnitudes(samples: &[Complex64]) -> Vec<f64> {
    samples.iter().map(|s| fast_magnitude(*s)).collect()
}

/// Compute power (|z|²) of complex samples.
///
/// More efficient than magnitude when you only need relative comparisons.
#[inline]
pub fn compute_power(samples: &[Complex64]) -> Vec<f64> {
    samples.iter().map(|s| s.norm_sqr()).collect()
}

/// Compute phase angles (arg(z)) of complex samples.
#[inline]
pub fn compute_phases(samples: &[Complex64]) -> Vec<f64> {
    samples.iter().map(|s| s.arg()).collect()
}

/// Fast magnitude approximation using hypot.
///
/// Uses the standard library's hypot which is optimized for accuracy and performance.
#[inline(always)]
pub fn fast_magnitude(z: Complex64) -> f64 {
    z.re.hypot(z.im)
}

/// Element-wise complex multiplication.
///
/// This is a hot path in chirp demodulation.
#[inline]
pub fn complex_multiply(a: &[Complex64], b: &[Complex64]) -> Vec<Complex64> {
    debug_assert_eq!(a.len(), b.len());
    a.iter().zip(b.iter()).map(|(&x, &y)| x * y).collect()
}

/// Element-wise complex multiplication, storing result in-place in `a`.
#[inline]
pub fn complex_multiply_inplace(a: &mut [Complex64], b: &[Complex64]) {
    debug_assert_eq!(a.len(), b.len());
    for (x, y) in a.iter_mut().zip(b.iter()) {
        *x = *x * *y;
    }
}

/// Element-wise complex conjugate multiplication (a * conj(b)).
///
/// Used in correlation operations.
#[inline]
pub fn complex_conjugate_multiply(a: &[Complex64], b: &[Complex64]) -> Vec<Complex64> {
    debug_assert_eq!(a.len(), b.len());
    a.iter().zip(b.iter()).map(|(&x, &y)| x * y.conj()).collect()
}

/// Scale complex samples by a real factor.
#[inline]
pub fn scale(samples: &[Complex64], factor: f64) -> Vec<Complex64> {
    samples.iter().map(|&s| s * factor).collect()
}

/// Scale complex samples in-place by a real factor.
#[inline]
pub fn scale_inplace(samples: &mut [Complex64], factor: f64) {
    for s in samples.iter_mut() {
        *s = *s * factor;
    }
}

/// Find the index and value of the maximum magnitude in complex samples.
///
/// Uses power (magnitude²) internally for efficiency.
#[inline]
pub fn find_max_magnitude(samples: &[Complex64]) -> (usize, f64) {
    let mut max_idx = 0;
    let mut max_power = 0.0;

    for (i, s) in samples.iter().enumerate() {
        let power = s.norm_sqr();
        if power > max_power {
            max_power = power;
            max_idx = i;
        }
    }

    (max_idx, max_power.sqrt())
}

/// Compute the sum of squared magnitudes (total energy).
#[inline]
pub fn total_power(samples: &[Complex64]) -> f64 {
    samples.iter().map(|s| s.norm_sqr()).sum()
}

/// Compute the mean power (average energy per sample).
#[inline]
pub fn mean_power(samples: &[Complex64]) -> f64 {
    if samples.is_empty() {
        return 0.0;
    }
    total_power(samples) / samples.len() as f64
}

/// Apply a frequency shift to complex samples.
///
/// Multiplies each sample by exp(j * 2π * freq_shift * t).
#[inline]
pub fn frequency_shift(samples: &[Complex64], freq_shift: f64, sample_rate: f64) -> Vec<Complex64> {
    let omega = 2.0 * std::f64::consts::PI * freq_shift / sample_rate;

    samples
        .iter()
        .enumerate()
        .map(|(i, &s)| {
            let phase = omega * i as f64;
            let rotation = Complex64::new(phase.cos(), phase.sin());
            s * rotation
        })
        .collect()
}

/// Apply a frequency shift in-place.
#[inline]
pub fn frequency_shift_inplace(samples: &mut [Complex64], freq_shift: f64, sample_rate: f64) {
    let omega = 2.0 * std::f64::consts::PI * freq_shift / sample_rate;

    for (i, s) in samples.iter_mut().enumerate() {
        let phase = omega * i as f64;
        let rotation = Complex64::new(phase.cos(), phase.sin());
        *s = *s * rotation;
    }
}

/// Downsample by keeping every `factor`-th sample.
#[inline]
pub fn downsample(samples: &[Complex64], factor: usize) -> Vec<Complex64> {
    samples.iter().step_by(factor).copied().collect()
}

/// Convert power to decibels (dB).
///
/// Uses a floor value to avoid log(0).
#[inline]
pub fn power_to_db(power: f64) -> f64 {
    const FLOOR: f64 = 1e-20;
    10.0 * power.max(FLOOR).log10()
}

/// Convert power array to decibels.
#[inline]
pub fn power_to_db_vec(power: &[f64]) -> Vec<f64> {
    power.iter().map(|&p| power_to_db(p)).collect()
}

/// Apply a window function to samples.
///
/// Multiplies each sample by the corresponding window coefficient.
#[inline]
pub fn apply_window(samples: &mut [Complex64], window: &[f64]) {
    debug_assert_eq!(samples.len(), window.len());
    for (s, &w) in samples.iter_mut().zip(window.iter()) {
        *s = *s * w;
    }
}

/// Generate a Hann window.
#[inline]
pub fn hann_window(size: usize) -> Vec<f64> {
    use std::f64::consts::PI;
    (0..size)
        .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / size as f64).cos()))
        .collect()
}

/// Generate a Hamming window.
#[inline]
pub fn hamming_window(size: usize) -> Vec<f64> {
    use std::f64::consts::PI;
    (0..size)
        .map(|i| 0.54 - 0.46 * (2.0 * PI * i as f64 / size as f64).cos())
        .collect()
}

/// Generate a Blackman window.
#[inline]
pub fn blackman_window(size: usize) -> Vec<f64> {
    use std::f64::consts::PI;
    (0..size)
        .map(|i| {
            let n = i as f64 / size as f64;
            0.42 - 0.5 * (2.0 * PI * n).cos() + 0.08 * (4.0 * PI * n).cos()
        })
        .collect()
}

/// Normalize samples to unit power.
#[inline]
pub fn normalize(samples: &[Complex64]) -> Vec<Complex64> {
    let power = mean_power(samples);
    if power > 0.0 {
        let factor = 1.0 / power.sqrt();
        scale(samples, factor)
    } else {
        samples.to_vec()
    }
}

/// Normalize samples in-place to unit power.
#[inline]
pub fn normalize_inplace(samples: &mut [Complex64]) {
    let power = mean_power(samples);
    if power > 0.0 {
        let factor = 1.0 / power.sqrt();
        scale_inplace(samples, factor);
    }
}

/// Compute correlation between two signals at zero lag.
#[inline]
pub fn correlate_at_zero(signal: &[Complex64], reference: &[Complex64]) -> Complex64 {
    debug_assert_eq!(signal.len(), reference.len());
    signal
        .iter()
        .zip(reference.iter())
        .map(|(&s, &r)| s * r.conj())
        .fold(Complex64::new(0.0, 0.0), |acc, x| acc + x)
}

/// Compute sliding correlation magnitude.
#[inline]
pub fn sliding_correlation_magnitude(
    signal: &[Complex64],
    reference: &[Complex64],
) -> Vec<f64> {
    if signal.len() < reference.len() {
        return vec![];
    }

    let output_len = signal.len() - reference.len() + 1;
    let mut result = Vec::with_capacity(output_len);

    for offset in 0..output_len {
        let corr: Complex64 = signal[offset..offset + reference.len()]
            .iter()
            .zip(reference.iter())
            .map(|(&s, &r)| s * r.conj())
            .fold(Complex64::new(0.0, 0.0), |acc, x| acc + x);
        result.push(corr.norm());
    }

    result
}

// ============================================================================
// Scalar (non-SIMD) implementations for performance comparison
// These deliberately prevent auto-vectorization to show SIMD benefits
// ============================================================================

/// Scalar magnitude computation - deliberately non-vectorizable.
///
/// Uses data dependencies and conditional logic to prevent LLVM from
/// auto-vectorizing this loop. Used for SIMD vs scalar benchmarking.
#[inline(never)]
pub fn scalar_compute_magnitudes(samples: &[Complex64]) -> Vec<f64> {
    let mut result = Vec::with_capacity(samples.len());
    let mut prev = 0.0_f64;

    for s in samples {
        // Data dependency: use previous result to prevent vectorization
        let re = s.re + prev * 1e-15;
        let im = s.im;

        // Manual magnitude calculation (avoiding hypot which might vectorize)
        let mag = (re * re + im * im).sqrt();

        // Update dependency chain
        prev = mag;
        result.push(mag);
    }

    result
}

/// Scalar power computation - deliberately non-vectorizable.
#[inline(never)]
pub fn scalar_compute_power(samples: &[Complex64]) -> Vec<f64> {
    let mut result = Vec::with_capacity(samples.len());
    let mut prev = 0.0_f64;

    for s in samples {
        let re = s.re + prev * 1e-15;
        let im = s.im;
        let power = re * re + im * im;
        prev = power;
        result.push(power);
    }

    result
}

/// Scalar complex multiplication - deliberately non-vectorizable.
#[inline(never)]
pub fn scalar_complex_multiply(a: &[Complex64], b: &[Complex64]) -> Vec<Complex64> {
    debug_assert_eq!(a.len(), b.len());
    let mut result = Vec::with_capacity(a.len());
    let mut prev_re = 0.0_f64;
    let mut prev_im = 0.0_f64;

    for (&x, &y) in a.iter().zip(b.iter()) {
        // Add tiny dependency to prevent vectorization
        let x_re = x.re + prev_re * 1e-15;
        let x_im = x.im + prev_im * 1e-15;

        // Manual complex multiplication
        let re = x_re * y.re - x_im * y.im;
        let im = x_re * y.im + x_im * y.re;

        prev_re = re;
        prev_im = im;
        result.push(Complex64::new(re, im));
    }

    result
}

/// Scalar frequency shift - deliberately non-vectorizable.
#[inline(never)]
pub fn scalar_frequency_shift(samples: &[Complex64], freq_shift: f64, sample_rate: f64) -> Vec<Complex64> {
    let omega = 2.0 * std::f64::consts::PI * freq_shift / sample_rate;
    let mut result = Vec::with_capacity(samples.len());
    let mut phase = 0.0_f64;

    for &s in samples {
        // Compute rotation manually (avoid library functions that might vectorize)
        let cos_phase = phase.cos();
        let sin_phase = phase.sin();

        let re = s.re * cos_phase - s.im * sin_phase;
        let im = s.re * sin_phase + s.im * cos_phase;

        result.push(Complex64::new(re, im));

        // Update phase with dependency
        phase += omega;
        if phase > 2.0 * std::f64::consts::PI {
            phase -= 2.0 * std::f64::consts::PI;
        }
    }

    result
}

/// Scalar correlation - deliberately non-vectorizable.
#[inline(never)]
pub fn scalar_correlate(signal: &[Complex64], reference: &[Complex64]) -> Vec<f64> {
    if signal.len() < reference.len() {
        return vec![];
    }

    let output_len = signal.len() - reference.len() + 1;
    let mut result = Vec::with_capacity(output_len);
    let mut prev_mag = 0.0_f64;

    for offset in 0..output_len {
        let mut sum_re = prev_mag * 1e-15;
        let mut sum_im = 0.0_f64;

        for i in 0..reference.len() {
            let s = signal[offset + i];
            let r = reference[i];

            // Manual complex conjugate multiply and accumulate
            sum_re += s.re * r.re + s.im * r.im;
            sum_im += s.im * r.re - s.re * r.im;
        }

        let mag = (sum_re * sum_re + sum_im * sum_im).sqrt();
        prev_mag = mag;
        result.push(mag);
    }

    result
}

/// Scalar Hann window generation - deliberately non-vectorizable.
#[inline(never)]
pub fn scalar_hann_window(size: usize) -> Vec<f64> {
    use std::f64::consts::PI;
    let mut result = Vec::with_capacity(size);
    let mut prev = 0.0_f64;

    for i in 0..size {
        let angle = 2.0 * PI * (i as f64 + prev * 1e-15) / size as f64;
        let w = 0.5 * (1.0 - angle.cos());
        prev = w;
        result.push(w);
    }

    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_magnitudes() {
        let samples = vec![
            Complex64::new(3.0, 4.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 1.0),
        ];
        let mags = compute_magnitudes(&samples);
        assert!((mags[0] - 5.0).abs() < 1e-10);
        assert!((mags[1] - 1.0).abs() < 1e-10);
        assert!((mags[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_complex_multiply() {
        let a = vec![Complex64::new(1.0, 2.0), Complex64::new(3.0, 4.0)];
        let b = vec![Complex64::new(5.0, 6.0), Complex64::new(7.0, 8.0)];
        let result = complex_multiply(&a, &b);

        // (1+2i)(5+6i) = 5 + 6i + 10i + 12i² = 5 + 16i - 12 = -7 + 16i
        assert!((result[0].re - (-7.0)).abs() < 1e-10);
        assert!((result[0].im - 16.0).abs() < 1e-10);
    }

    #[test]
    fn test_find_max_magnitude() {
        let samples = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(3.0, 4.0), // magnitude 5
            Complex64::new(2.0, 0.0),
        ];
        let (idx, mag) = find_max_magnitude(&samples);
        assert_eq!(idx, 1);
        assert!((mag - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_hann_window() {
        let window = hann_window(4);
        assert!((window[0] - 0.0).abs() < 1e-10); // Start at 0
        assert!((window[2] - 1.0).abs() < 1e-10); // Peak at center
    }

    #[test]
    fn test_normalize() {
        let samples = vec![
            Complex64::new(2.0, 0.0),
            Complex64::new(0.0, 2.0),
            Complex64::new(2.0, 0.0),
            Complex64::new(0.0, 2.0),
        ];
        let normalized = normalize(&samples);
        let power = mean_power(&normalized);
        assert!((power - 1.0).abs() < 1e-10);
    }
}
