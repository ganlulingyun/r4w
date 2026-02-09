//! NLog10 — Logarithmic scaling (linear to dB conversion)
//!
//! Computes `n * log10(|x|) + k` for converting linear power/amplitude
//! to decibel scale. Essential for spectrum display, signal level
//! metering, and log-domain processing.
//! GNU Radio equivalent: `nlog10_ff`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::nlog10::{nlog10, to_db, to_dbm};
//!
//! // 10 * log10(100) = 20 dB
//! let result = nlog10(&[100.0], 10.0, 0.0);
//! assert!((result[0] - 20.0).abs() < 1e-10);
//!
//! // Power to dB
//! assert!((to_db(0.01) - (-20.0)).abs() < 1e-10);
//! ```

/// Compute `n * log10(|x|) + k` for each sample.
///
/// Values <= 0 are clamped to a small positive number to avoid -inf.
pub fn nlog10(input: &[f64], n: f64, k: f64) -> Vec<f64> {
    input
        .iter()
        .map(|&x| n * x.abs().max(1e-30).log10() + k)
        .collect()
}

/// In-place `n * log10(|x|) + k`.
pub fn nlog10_inplace(data: &mut [f64], n: f64, k: f64) {
    for x in data.iter_mut() {
        *x = n * x.abs().max(1e-30).log10() + k;
    }
}

/// Convert linear power to dB: `10 * log10(x)`.
#[inline]
pub fn to_db(power: f64) -> f64 {
    10.0 * power.abs().max(1e-30).log10()
}

/// Convert dB back to linear power: `10^(db/10)`.
#[inline]
pub fn from_db(db: f64) -> f64 {
    10.0f64.powf(db / 10.0)
}

/// Convert linear amplitude to dB: `20 * log10(|x|)`.
#[inline]
pub fn amplitude_to_db(amplitude: f64) -> f64 {
    20.0 * amplitude.abs().max(1e-30).log10()
}

/// Convert dB back to linear amplitude: `10^(db/20)`.
#[inline]
pub fn db_to_amplitude(db: f64) -> f64 {
    10.0f64.powf(db / 20.0)
}

/// Convert linear power to dBm (relative to 1 mW): `10*log10(x) + 30`.
#[inline]
pub fn to_dbm(power_watts: f64) -> f64 {
    10.0 * power_watts.abs().max(1e-30).log10() + 30.0
}

/// Convert dBm to linear power (Watts): `10^((dbm-30)/10)`.
#[inline]
pub fn from_dbm(dbm: f64) -> f64 {
    10.0f64.powf((dbm - 30.0) / 10.0)
}

/// Batch convert power to dB.
pub fn power_to_db(input: &[f64]) -> Vec<f64> {
    nlog10(input, 10.0, 0.0)
}

/// Batch convert amplitude to dB.
pub fn amplitude_to_db_vec(input: &[f64]) -> Vec<f64> {
    nlog10(input, 20.0, 0.0)
}

/// Batch convert power to dBm.
pub fn power_to_dbm(input: &[f64]) -> Vec<f64> {
    nlog10(input, 10.0, 30.0)
}

/// Batch convert dB back to linear power.
pub fn db_to_power(input: &[f64]) -> Vec<f64> {
    input.iter().map(|&db| from_db(db)).collect()
}

/// Compute magnitude squared from I/Q and convert to dB.
///
/// `10 * log10(I² + Q²)`
pub fn iq_to_db(i: &[f64], q: &[f64]) -> Vec<f64> {
    i.iter()
        .zip(q.iter())
        .map(|(&re, &im)| {
            let power = re * re + im * im;
            10.0 * power.max(1e-30).log10()
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_nlog10_basic() {
        let result = nlog10(&[100.0], 10.0, 0.0);
        assert!((result[0] - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_nlog10_with_offset() {
        let result = nlog10(&[1.0], 10.0, 30.0);
        assert!((result[0] - 30.0).abs() < 1e-10); // 10*log10(1) + 30 = 30
    }

    #[test]
    fn test_nlog10_amplitude() {
        // 20*log10(10) = 20
        let result = nlog10(&[10.0], 20.0, 0.0);
        assert!((result[0] - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_to_db() {
        assert!((to_db(1.0) - 0.0).abs() < 1e-10);
        assert!((to_db(10.0) - 10.0).abs() < 1e-10);
        assert!((to_db(100.0) - 20.0).abs() < 1e-10);
        assert!((to_db(0.01) - (-20.0)).abs() < 1e-10);
    }

    #[test]
    fn test_from_db() {
        assert!((from_db(0.0) - 1.0).abs() < 1e-10);
        assert!((from_db(10.0) - 10.0).abs() < 1e-10);
        assert!((from_db(20.0) - 100.0).abs() < 1e-10);
        assert!((from_db(-20.0) - 0.01).abs() < 1e-10);
    }

    #[test]
    fn test_db_roundtrip() {
        for &val in &[0.001, 0.1, 1.0, 10.0, 1000.0] {
            let db = to_db(val);
            let recovered = from_db(db);
            assert!(
                (recovered - val).abs() / val < 1e-10,
                "roundtrip failed for {val}"
            );
        }
    }

    #[test]
    fn test_amplitude_to_db() {
        assert!((amplitude_to_db(1.0) - 0.0).abs() < 1e-10);
        assert!((amplitude_to_db(10.0) - 20.0).abs() < 1e-10);
        assert!((amplitude_to_db(0.1) - (-20.0)).abs() < 1e-10);
    }

    #[test]
    fn test_amplitude_db_roundtrip() {
        for &val in &[0.01, 0.5, 1.0, 3.0, 100.0] {
            let db = amplitude_to_db(val);
            let recovered = db_to_amplitude(db);
            assert!(
                (recovered - val).abs() / val < 1e-10,
                "amplitude roundtrip failed for {val}"
            );
        }
    }

    #[test]
    fn test_to_dbm() {
        // 1 Watt = 30 dBm
        assert!((to_dbm(1.0) - 30.0).abs() < 1e-10);
        // 1 mW = 0 dBm
        assert!((to_dbm(0.001) - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_from_dbm() {
        assert!((from_dbm(30.0) - 1.0).abs() < 1e-10);
        assert!((from_dbm(0.0) - 0.001).abs() < 1e-10);
    }

    #[test]
    fn test_dbm_roundtrip() {
        for &val in &[0.001, 0.01, 1.0, 10.0] {
            let dbm = to_dbm(val);
            let recovered = from_dbm(dbm);
            assert!(
                (recovered - val).abs() / val < 1e-10,
                "dBm roundtrip failed for {val}"
            );
        }
    }

    #[test]
    fn test_batch_power_to_db() {
        let out = power_to_db(&[1.0, 10.0, 100.0]);
        assert!((out[0] - 0.0).abs() < 1e-10);
        assert!((out[1] - 10.0).abs() < 1e-10);
        assert!((out[2] - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_iq_to_db() {
        // |1+0j|² = 1 → 0 dB
        let out = iq_to_db(&[1.0], &[0.0]);
        assert!((out[0] - 0.0).abs() < 1e-10);
        // |0+10j|² = 100 → 20 dB
        let out = iq_to_db(&[0.0], &[10.0]);
        assert!((out[0] - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_inplace() {
        let mut data = vec![100.0, 1000.0];
        nlog10_inplace(&mut data, 10.0, 0.0);
        assert!((data[0] - 20.0).abs() < 1e-10);
        assert!((data[1] - 30.0).abs() < 1e-10);
    }

    #[test]
    fn test_negative_input() {
        // Should use |x|
        let result = nlog10(&[-100.0], 10.0, 0.0);
        assert!((result[0] - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_zero_input() {
        // Should clamp to very small value, not -inf
        let result = nlog10(&[0.0], 10.0, 0.0);
        assert!(result[0].is_finite());
        assert!(result[0] < -200.0); // Very negative dB
    }

    #[test]
    fn test_empty() {
        assert!(nlog10(&[], 10.0, 0.0).is_empty());
    }
}
