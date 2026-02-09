//! Peak-to-Average Power Ratio (PAPR) — Crest factor measurement
//!
//! Measures the ratio of peak power to average power in a signal,
//! expressed in dB. High PAPR is a key challenge in OFDM systems
//! where it reduces power amplifier efficiency. Also useful for
//! signal characterization and clipping analysis.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::peak_to_average::{papr_db, crest_factor};
//!
//! // Pure sine wave has PAPR = 3.01 dB
//! let sine: Vec<f64> = (0..1000)
//!     .map(|i| (2.0 * std::f64::consts::PI * i as f64 / 100.0).sin())
//!     .collect();
//! let papr = papr_db(&sine);
//! assert!((papr - 3.01).abs() < 0.1);
//!
//! // Crest factor = sqrt(PAPR_linear) for a sine = sqrt(2)
//! let cf = crest_factor(&sine);
//! assert!((cf - std::f64::consts::SQRT_2).abs() < 0.05);
//! ```

use num_complex::Complex64;

/// Compute PAPR in dB for a real signal.
///
/// PAPR = 10 * log10(peak_power / avg_power)
/// Returns 0.0 for zero-power signals.
pub fn papr_db(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    let avg_power = signal.iter().map(|&x| x * x).sum::<f64>() / signal.len() as f64;
    let peak_power = signal
        .iter()
        .map(|&x| x * x)
        .fold(0.0f64, f64::max);
    if avg_power < 1e-30 {
        return 0.0;
    }
    10.0 * (peak_power / avg_power).log10()
}

/// Compute PAPR in dB for a complex signal.
///
/// PAPR = 10 * log10(peak|x|² / avg|x|²)
pub fn papr_db_complex(signal: &[Complex64]) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    let avg_power = signal.iter().map(|s| s.norm_sqr()).sum::<f64>() / signal.len() as f64;
    let peak_power = signal
        .iter()
        .map(|s| s.norm_sqr())
        .fold(0.0f64, f64::max);
    if avg_power < 1e-30 {
        return 0.0;
    }
    10.0 * (peak_power / avg_power).log10()
}

/// Compute PAPR in linear scale for a real signal.
pub fn papr_linear(signal: &[f64]) -> f64 {
    if signal.is_empty() {
        return 1.0;
    }
    let avg_power = signal.iter().map(|&x| x * x).sum::<f64>() / signal.len() as f64;
    let peak_power = signal
        .iter()
        .map(|&x| x * x)
        .fold(0.0f64, f64::max);
    if avg_power < 1e-30 {
        return 1.0;
    }
    peak_power / avg_power
}

/// Crest factor: ratio of peak amplitude to RMS amplitude.
///
/// CF = peak / RMS = sqrt(PAPR_linear)
pub fn crest_factor(signal: &[f64]) -> f64 {
    papr_linear(signal).sqrt()
}

/// Crest factor for complex signal.
pub fn crest_factor_complex(signal: &[Complex64]) -> f64 {
    if signal.is_empty() {
        return 1.0;
    }
    let rms = (signal.iter().map(|s| s.norm_sqr()).sum::<f64>() / signal.len() as f64).sqrt();
    let peak = signal
        .iter()
        .map(|s| s.norm())
        .fold(0.0f64, f64::max);
    if rms < 1e-15 {
        return 1.0;
    }
    peak / rms
}

/// Streaming PAPR estimator with sliding window.
#[derive(Debug, Clone)]
pub struct PaprEstimator {
    /// Window of power values.
    buffer: Vec<f64>,
    /// Current write position.
    pos: usize,
    /// Window size.
    window_size: usize,
    /// Running sum of power.
    sum: f64,
    /// Running peak power.
    peak: f64,
    /// Count of samples in buffer.
    count: usize,
}

impl PaprEstimator {
    /// Create a PAPR estimator with given window size.
    pub fn new(window_size: usize) -> Self {
        let window_size = window_size.max(1);
        Self {
            buffer: vec![0.0; window_size],
            pos: 0,
            window_size,
            sum: 0.0,
            peak: 0.0,
            count: 0,
        }
    }

    /// Process a sample, returning current PAPR estimate in dB.
    pub fn process_sample(&mut self, x: f64) -> f64 {
        let power = x * x;
        if self.count >= self.window_size {
            self.sum -= self.buffer[self.pos];
        }
        self.buffer[self.pos] = power;
        self.sum += power;
        self.pos = (self.pos + 1) % self.window_size;
        if self.count < self.window_size {
            self.count += 1;
        }

        // Recompute peak (linear scan since we need exact peak after eviction)
        self.peak = self
            .buffer[..self.count.min(self.window_size)]
            .iter()
            .cloned()
            .fold(0.0f64, f64::max);

        let avg = self.sum / self.count as f64;
        if avg < 1e-30 {
            0.0
        } else {
            10.0 * (self.peak / avg).log10()
        }
    }

    /// Process a complex sample.
    pub fn process_sample_complex(&mut self, s: Complex64) -> f64 {
        self.process_sample(s.norm())
    }

    /// Process a block, returning PAPR for each sample.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.process_sample(x)).collect()
    }

    /// Get current PAPR in dB.
    pub fn papr_db(&self) -> f64 {
        let avg = if self.count > 0 {
            self.sum / self.count as f64
        } else {
            return 0.0;
        };
        if avg < 1e-30 {
            0.0
        } else {
            10.0 * (self.peak / avg).log10()
        }
    }

    /// Reset.
    pub fn reset(&mut self) {
        self.buffer.fill(0.0);
        self.pos = 0;
        self.sum = 0.0;
        self.peak = 0.0;
        self.count = 0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_sine_papr() {
        // Pure sine: PAPR = 3.01 dB
        let sine: Vec<f64> = (0..10000)
            .map(|i| (2.0 * PI * i as f64 / 100.0).sin())
            .collect();
        let papr = papr_db(&sine);
        assert!(
            (papr - 3.01).abs() < 0.1,
            "sine PAPR should be ~3.01 dB, got {papr}"
        );
    }

    #[test]
    fn test_constant_papr() {
        // Constant signal: PAPR = 0 dB
        let dc = vec![1.0; 1000];
        let papr = papr_db(&dc);
        assert!(papr.abs() < 0.01, "DC PAPR should be 0 dB, got {papr}");
    }

    #[test]
    fn test_complex_sine_papr() {
        let signal: Vec<Complex64> = (0..10000)
            .map(|i| {
                let t = 2.0 * PI * i as f64 / 100.0;
                Complex64::new(t.cos(), t.sin())
            })
            .collect();
        let papr = papr_db_complex(&signal);
        // Constant-envelope complex sinusoid: PAPR = 0 dB
        assert!(papr.abs() < 0.1, "CW PAPR should be ~0 dB, got {papr}");
    }

    #[test]
    fn test_crest_factor_sine() {
        let sine: Vec<f64> = (0..10000)
            .map(|i| (2.0 * PI * i as f64 / 100.0).sin())
            .collect();
        let cf = crest_factor(&sine);
        assert!(
            (cf - std::f64::consts::SQRT_2).abs() < 0.05,
            "sine CF should be sqrt(2), got {cf}"
        );
    }

    #[test]
    fn test_crest_factor_dc() {
        let dc = vec![3.0; 100];
        let cf = crest_factor(&dc);
        assert!((cf - 1.0).abs() < 0.01, "DC CF should be 1.0, got {cf}");
    }

    #[test]
    fn test_crest_factor_complex() {
        // CW: constant envelope, CF = 1.0
        let signal: Vec<Complex64> = (0..1000)
            .map(|i| {
                let t = 2.0 * PI * i as f64 / 100.0;
                Complex64::new(t.cos(), t.sin())
            })
            .collect();
        let cf = crest_factor_complex(&signal);
        assert!((cf - 1.0).abs() < 0.05, "CW CF should be ~1.0, got {cf}");
    }

    #[test]
    fn test_papr_linear() {
        let sine: Vec<f64> = (0..10000)
            .map(|i| (2.0 * PI * i as f64 / 100.0).sin())
            .collect();
        let papr = papr_linear(&sine);
        assert!(
            (papr - 2.0).abs() < 0.1,
            "sine PAPR linear should be ~2.0, got {papr}"
        );
    }

    #[test]
    fn test_empty_signal() {
        assert_eq!(papr_db(&[]), 0.0);
        assert_eq!(papr_db_complex(&[]), 0.0);
        assert_eq!(papr_linear(&[]), 1.0);
        assert_eq!(crest_factor(&[]), 1.0);
    }

    #[test]
    fn test_zero_signal() {
        assert_eq!(papr_db(&[0.0, 0.0, 0.0]), 0.0);
    }

    #[test]
    fn test_streaming_papr() {
        let mut est = PaprEstimator::new(100);
        let sine: Vec<f64> = (0..1000)
            .map(|i| (2.0 * PI * i as f64 / 20.0).sin())
            .collect();
        let results = est.process(&sine);
        assert_eq!(results.len(), 1000);
        // After enough samples, should approach ~3 dB
        let last = *results.last().unwrap();
        assert!(last > 2.0, "streaming PAPR should be >2 dB, got {last}");
    }

    #[test]
    fn test_streaming_reset() {
        let mut est = PaprEstimator::new(50);
        est.process_sample(1.0);
        est.reset();
        assert_eq!(est.papr_db(), 0.0);
    }

    #[test]
    fn test_high_papr_signal() {
        // Spike in noise: should have high PAPR
        let mut signal = vec![0.01; 1000];
        signal[500] = 10.0;
        let papr = papr_db(&signal);
        assert!(papr > 20.0, "spike signal should have high PAPR, got {papr}");
    }
}
