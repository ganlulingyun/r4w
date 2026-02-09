//! Magnitude Squared — Complex to real power conversion
//!
//! Computes |x|² = re² + im² for each complex sample, converting
//! IQ data to instantaneous power. More efficient than computing
//! magnitude (avoids sqrt). Essential for power detection, energy
//! estimation, and envelope extraction.
//! GNU Radio equivalent: `complex_to_mag_squared`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::magnitude_squared::*;
//! use num_complex::Complex64;
//!
//! let samples = vec![Complex64::new(3.0, 4.0), Complex64::new(1.0, 0.0)];
//! let power = mag_squared(&samples);
//! assert!((power[0] - 25.0).abs() < 1e-10); // 3² + 4² = 25
//! assert!((power[1] - 1.0).abs() < 1e-10);
//! ```

use num_complex::Complex64;

/// Compute |x|² for each complex sample.
pub fn mag_squared(input: &[Complex64]) -> Vec<f64> {
    input.iter().map(|s| s.norm_sqr()).collect()
}

/// Compute |x| (magnitude) for each complex sample.
pub fn magnitude(input: &[Complex64]) -> Vec<f64> {
    input.iter().map(|s| s.norm()).collect()
}

/// Compute |x|² in-place, writing to a pre-allocated output.
pub fn mag_squared_into(input: &[Complex64], output: &mut [f64]) {
    for (out, s) in output.iter_mut().zip(input.iter()) {
        *out = s.norm_sqr();
    }
}

/// Compute 10*log10(|x|²) for each sample (power in dB).
pub fn mag_squared_db(input: &[Complex64]) -> Vec<f64> {
    input
        .iter()
        .map(|s| 10.0 * s.norm_sqr().max(1e-30).log10())
        .collect()
}

/// Compute average power: mean(|x|²).
pub fn avg_power(input: &[Complex64]) -> f64 {
    if input.is_empty() {
        return 0.0;
    }
    input.iter().map(|s| s.norm_sqr()).sum::<f64>() / input.len() as f64
}

/// Compute peak power: max(|x|²).
pub fn peak_power(input: &[Complex64]) -> f64 {
    input
        .iter()
        .map(|s| s.norm_sqr())
        .fold(0.0f64, f64::max)
}

/// Compute RMS amplitude: sqrt(mean(|x|²)).
pub fn rms_amplitude(input: &[Complex64]) -> f64 {
    avg_power(input).sqrt()
}

/// Compute power for real-valued signals: x².
pub fn power_real(input: &[f64]) -> Vec<f64> {
    input.iter().map(|&x| x * x).collect()
}

/// Average power for real signals.
pub fn avg_power_real(input: &[f64]) -> f64 {
    if input.is_empty() {
        return 0.0;
    }
    input.iter().map(|&x| x * x).sum::<f64>() / input.len() as f64
}

/// RMS for real signals.
pub fn rms_real(input: &[f64]) -> f64 {
    avg_power_real(input).sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_mag_squared_basic() {
        let samples = vec![
            Complex64::new(3.0, 4.0),
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 1.0),
        ];
        let power = mag_squared(&samples);
        assert!((power[0] - 25.0).abs() < 1e-10);
        assert!((power[1] - 1.0).abs() < 1e-10);
        assert!((power[2] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_magnitude() {
        let samples = vec![Complex64::new(3.0, 4.0)];
        let mag = magnitude(&samples);
        assert!((mag[0] - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_mag_squared_into() {
        let input = vec![Complex64::new(1.0, 1.0), Complex64::new(2.0, 0.0)];
        let mut output = vec![0.0; 2];
        mag_squared_into(&input, &mut output);
        assert!((output[0] - 2.0).abs() < 1e-10);
        assert!((output[1] - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_mag_squared_db() {
        let samples = vec![Complex64::new(10.0, 0.0)]; // |x|² = 100 → 20 dB
        let db = mag_squared_db(&samples);
        assert!((db[0] - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_avg_power() {
        let samples = vec![
            Complex64::new(1.0, 0.0), // |x|² = 1
            Complex64::new(0.0, 1.0), // |x|² = 1
        ];
        assert!((avg_power(&samples) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_peak_power() {
        let samples = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(3.0, 4.0),
            Complex64::new(0.0, 1.0),
        ];
        assert!((peak_power(&samples) - 25.0).abs() < 1e-10);
    }

    #[test]
    fn test_rms_amplitude() {
        let samples = vec![
            Complex64::new(1.0, 0.0),
            Complex64::new(0.0, 1.0),
        ];
        assert!((rms_amplitude(&samples) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_power_real() {
        let power = power_real(&[3.0, -4.0, 5.0]);
        assert_eq!(power, vec![9.0, 16.0, 25.0]);
    }

    #[test]
    fn test_avg_power_real() {
        assert!((avg_power_real(&[3.0, 4.0]) - 12.5).abs() < 1e-10); // (9+16)/2
    }

    #[test]
    fn test_rms_real() {
        // RMS of [3, 4] = sqrt((9+16)/2) = sqrt(12.5) ≈ 3.536
        let rms = rms_real(&[3.0, 4.0]);
        assert!((rms - 12.5f64.sqrt()).abs() < 1e-10);
    }

    #[test]
    fn test_empty() {
        assert!(mag_squared(&[]).is_empty());
        assert_eq!(avg_power(&[]), 0.0);
        assert_eq!(peak_power(&[]), 0.0);
        assert_eq!(avg_power_real(&[]), 0.0);
    }

    #[test]
    fn test_zero_signal() {
        let zero = vec![Complex64::new(0.0, 0.0); 10];
        assert_eq!(avg_power(&zero), 0.0);
        assert_eq!(rms_amplitude(&zero), 0.0);
    }

    #[test]
    fn test_unit_circle() {
        // Points on unit circle all have |x|² = 1
        let samples: Vec<Complex64> = (0..8)
            .map(|i| {
                let angle = 2.0 * std::f64::consts::PI * i as f64 / 8.0;
                Complex64::new(angle.cos(), angle.sin())
            })
            .collect();
        let power = mag_squared(&samples);
        for &p in &power {
            assert!((p - 1.0).abs() < 1e-10);
        }
    }
}
