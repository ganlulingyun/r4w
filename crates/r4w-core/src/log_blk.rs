//! # Logarithmic Operations Block
//!
//! Element-wise logarithmic operations for signal processing:
//! natural log, log10, log2, and dB conversions. Essential for
//! power spectrum display, signal level metering, and dynamic
//! range processing.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::log_blk::{log10_vec, to_db, from_db};
//!
//! let power = vec![1.0, 10.0, 100.0];
//! let db = to_db(&power);
//! assert!((db[0] - 0.0).abs() < 1e-10);
//! assert!((db[1] - 10.0).abs() < 1e-10);
//! assert!((db[2] - 20.0).abs() < 1e-10);
//! ```

/// Natural logarithm (ln) of each element.
pub fn ln_vec(input: &[f64]) -> Vec<f64> {
    input.iter().map(|&x| x.max(1e-300).ln()).collect()
}

/// Base-10 logarithm of each element.
pub fn log10_vec(input: &[f64]) -> Vec<f64> {
    input.iter().map(|&x| x.max(1e-300).log10()).collect()
}

/// Base-2 logarithm of each element.
pub fn log2_vec(input: &[f64]) -> Vec<f64> {
    input.iter().map(|&x| x.max(1e-300).log2()).collect()
}

/// Convert linear power values to dB: 10 * log10(x).
pub fn to_db(input: &[f64]) -> Vec<f64> {
    input.iter().map(|&x| 10.0 * x.max(1e-300).log10()).collect()
}

/// Convert dB values to linear power: 10^(x/10).
pub fn from_db(input: &[f64]) -> Vec<f64> {
    input.iter().map(|&x| 10.0_f64.powf(x / 10.0)).collect()
}

/// Convert linear amplitude to dB: 20 * log10(x).
pub fn amplitude_to_db(input: &[f64]) -> Vec<f64> {
    input
        .iter()
        .map(|&x| 20.0 * x.abs().max(1e-300).log10())
        .collect()
}

/// Convert dB to linear amplitude: 10^(x/20).
pub fn db_to_amplitude(input: &[f64]) -> Vec<f64> {
    input.iter().map(|&x| 10.0_f64.powf(x / 20.0)).collect()
}

/// Exponential (e^x) of each element.
pub fn exp_vec(input: &[f64]) -> Vec<f64> {
    input.iter().map(|&x| x.exp()).collect()
}

/// 10^x for each element.
pub fn pow10_vec(input: &[f64]) -> Vec<f64> {
    input.iter().map(|&x| 10.0_f64.powf(x)).collect()
}

/// Complex magnitude in dB: 10 * log10(re² + im²).
pub fn complex_to_db(input: &[(f64, f64)]) -> Vec<f64> {
    input
        .iter()
        .map(|&(re, im)| {
            let power = re * re + im * im;
            10.0 * power.max(1e-300).log10()
        })
        .collect()
}

/// Normalize to dB relative to peak.
pub fn normalize_db(input: &[f64]) -> Vec<f64> {
    let max_val = input.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    if max_val <= 1e-300 {
        return vec![0.0; input.len()];
    }
    input.iter().map(|&x| 10.0 * (x / max_val).max(1e-300).log10()).collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_log10() {
        let input = vec![1.0, 10.0, 100.0, 1000.0];
        let result = log10_vec(&input);
        assert!((result[0] - 0.0).abs() < 1e-10);
        assert!((result[1] - 1.0).abs() < 1e-10);
        assert!((result[2] - 2.0).abs() < 1e-10);
        assert!((result[3] - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_ln() {
        let input = vec![1.0, std::f64::consts::E];
        let result = ln_vec(&input);
        assert!((result[0] - 0.0).abs() < 1e-10);
        assert!((result[1] - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_log2() {
        let input = vec![1.0, 2.0, 4.0, 8.0];
        let result = log2_vec(&input);
        assert!((result[0] - 0.0).abs() < 1e-10);
        assert!((result[1] - 1.0).abs() < 1e-10);
        assert!((result[2] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_to_db() {
        let input = vec![1.0, 10.0, 100.0, 0.001];
        let db = to_db(&input);
        assert!((db[0] - 0.0).abs() < 1e-10);
        assert!((db[1] - 10.0).abs() < 1e-10);
        assert!((db[2] - 20.0).abs() < 1e-10);
        assert!((db[3] + 30.0).abs() < 1e-8);
    }

    #[test]
    fn test_from_db() {
        let db = vec![0.0, 10.0, 20.0, -30.0];
        let linear = from_db(&db);
        assert!((linear[0] - 1.0).abs() < 1e-10);
        assert!((linear[1] - 10.0).abs() < 1e-8);
        assert!((linear[2] - 100.0).abs() < 1e-6);
    }

    #[test]
    fn test_db_roundtrip() {
        let original = vec![0.5, 1.0, 2.0, 10.0];
        let db = to_db(&original);
        let restored = from_db(&db);
        for (o, r) in original.iter().zip(restored.iter()) {
            assert!((o - r).abs() < 1e-10, "o={} r={}", o, r);
        }
    }

    #[test]
    fn test_amplitude_db() {
        let amp = vec![1.0, 2.0, 10.0];
        let db = amplitude_to_db(&amp);
        assert!((db[0] - 0.0).abs() < 1e-10);
        assert!((db[1] - 6.0206).abs() < 0.001);
        assert!((db[2] - 20.0).abs() < 1e-10);
    }

    #[test]
    fn test_exp() {
        let input = vec![0.0, 1.0];
        let result = exp_vec(&input);
        assert!((result[0] - 1.0).abs() < 1e-10);
        assert!((result[1] - std::f64::consts::E).abs() < 1e-10);
    }

    #[test]
    fn test_complex_to_db() {
        let input = vec![(1.0, 0.0), (0.0, 1.0), (3.0, 4.0)];
        let db = complex_to_db(&input);
        assert!((db[0] - 0.0).abs() < 1e-10); // |1+0j|^2 = 1 → 0 dB
        assert!((db[1] - 0.0).abs() < 1e-10); // |0+1j|^2 = 1 → 0 dB
        assert!((db[2] - 10.0 * 25.0_f64.log10()).abs() < 1e-8); // |3+4j|^2 = 25 → 13.98 dB
    }

    #[test]
    fn test_normalize_db() {
        let input = vec![1.0, 10.0, 100.0];
        let norm = normalize_db(&input);
        assert!((norm[2] - 0.0).abs() < 1e-10); // Peak is 0 dB
        assert!((norm[1] + 10.0).abs() < 1e-8); // -10 dB
    }
}
