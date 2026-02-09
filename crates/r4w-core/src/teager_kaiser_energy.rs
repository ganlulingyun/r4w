//! Teager-Kaiser Energy Operator (TKEO)
//!
//! Computes instantaneous energy of a signal using the nonlinear
//! Teager-Kaiser operator: Ψ[x(n)] = x(n)² - x(n-1)·x(n+1).
//! Provides superior time resolution compared to squared envelope
//! for transient detection, AM/FM demodulation, and speech analysis.
//!
//! No direct GNU Radio equivalent (fundamental DSP building block).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::teager_kaiser_energy::{tkeo, tkeo_am_demod};
//!
//! let signal: Vec<f64> = (0..100)
//!     .map(|i| (i as f64 * 0.2).sin())
//!     .collect();
//! let energy = tkeo(&signal);
//! assert_eq!(energy.len(), signal.len() - 2);
//! assert!(energy.iter().all(|&e| e >= -0.1));
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Compute the Teager-Kaiser Energy Operator on a real signal.
///
/// Ψ[x(n)] = x(n)² - x(n-1)·x(n+1)
///
/// Output length = input length - 2 (needs one sample of context on each side).
pub fn tkeo(signal: &[f64]) -> Vec<f64> {
    if signal.len() < 3 {
        return vec![];
    }
    (1..signal.len() - 1)
        .map(|i| signal[i] * signal[i] - signal[i - 1] * signal[i + 1])
        .collect()
}

/// Compute TKEO on a complex signal.
///
/// For complex signals: Ψ[x(n)] = |x(n)|² - Re{x(n-1)·conj(x(n+1))}.
pub fn tkeo_complex(signal: &[Complex64]) -> Vec<f64> {
    if signal.len() < 3 {
        return vec![];
    }
    (1..signal.len() - 1)
        .map(|i| {
            let mag_sq = signal[i].norm_sqr();
            let cross = (signal[i - 1] * signal[i + 1].conj()).re;
            mag_sq - cross
        })
        .collect()
}

/// Demodulate an AM signal using TKEO.
///
/// For x(n) = a(n)·cos(ω(n)·n + φ), the instantaneous amplitude is:
/// a(n) ≈ sqrt(Ψ[x] / Ψ[cos(ω·n)])
///
/// Returns the amplitude envelope.
pub fn tkeo_am_demod(signal: &[f64]) -> Vec<f64> {
    let energy = tkeo(signal);
    energy.iter().map(|&e| e.abs().sqrt()).collect()
}

/// Demodulate an FM signal using TKEO.
///
/// For x(n) = cos(ω(n)·n + φ), the instantaneous frequency is:
/// ω(n) ≈ arccos(1 - Ψ[x'(n)] / (2·Ψ[x(n)]))
///
/// Returns normalized frequency (0..0.5).
pub fn tkeo_fm_demod(signal: &[f64]) -> Vec<f64> {
    // Compute signal derivative (first difference)
    let diff: Vec<f64> = signal.windows(2).map(|w| w[1] - w[0]).collect();

    let energy_x = tkeo(signal);
    let energy_dx = tkeo(&diff);

    // Need to align: energy_x is from indices 1..N-1, energy_dx from 1..N-2
    let len = energy_x.len().min(energy_dx.len());
    (0..len)
        .map(|i| {
            if energy_x[i].abs() < 1e-30 {
                0.0
            } else {
                let cos_arg = 1.0 - energy_dx[i] / (2.0 * energy_x[i]);
                let cos_arg = cos_arg.clamp(-1.0, 1.0);
                cos_arg.acos() / (2.0 * PI)
            }
        })
        .collect()
}

/// Streaming TKEO processor with 2-sample history buffer.
#[derive(Debug, Clone)]
pub struct TeagerKaiserOperator {
    prev2: f64,
    prev1: f64,
    count: usize,
}

impl TeagerKaiserOperator {
    /// Create a new TKEO processor.
    pub fn new() -> Self {
        Self {
            prev2: 0.0,
            prev1: 0.0,
            count: 0,
        }
    }

    /// Process a single sample, returning TKEO output (None for first 2 samples).
    pub fn process_sample(&mut self, sample: f64) -> Option<f64> {
        self.count += 1;
        if self.count < 3 {
            self.prev2 = self.prev1;
            self.prev1 = sample;
            return None;
        }
        let energy = self.prev1 * self.prev1 - self.prev2 * sample;
        self.prev2 = self.prev1;
        self.prev1 = sample;
        Some(energy)
    }

    /// Process a block of samples.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().filter_map(|&s| self.process_sample(s)).collect()
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.prev2 = 0.0;
        self.prev1 = 0.0;
        self.count = 0;
    }
}

impl Default for TeagerKaiserOperator {
    fn default() -> Self {
        Self::new()
    }
}

/// Detect transients (abrupt changes) using TKEO with threshold.
///
/// Returns indices where TKEO energy exceeds `threshold`.
pub fn detect_transients(signal: &[f64], threshold: f64) -> Vec<usize> {
    let energy = tkeo(signal);
    energy
        .iter()
        .enumerate()
        .filter(|(_, &e)| e > threshold)
        .map(|(i, _)| i + 1) // +1 to map back to original signal index
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_tkeo_pure_sine() {
        // For sin(ωn), TKEO ≈ sin²(ω) (constant energy)
        let omega = 0.3;
        let signal: Vec<f64> = (0..200).map(|i| (omega * i as f64).sin()).collect();
        let energy = tkeo(&signal);
        let expected = omega.sin().powi(2);
        // Skip edges
        for &e in &energy[10..energy.len() - 10] {
            assert!((e - expected).abs() < 0.01, "e={e} expected={expected}");
        }
    }

    #[test]
    fn test_tkeo_dc_signal() {
        // TKEO of DC = 0
        let signal = vec![3.0; 50];
        let energy = tkeo(&signal);
        for &e in &energy {
            assert!(e.abs() < 1e-10, "e={e}");
        }
    }

    #[test]
    fn test_tkeo_length() {
        let signal = vec![1.0; 10];
        let energy = tkeo(&signal);
        assert_eq!(energy.len(), 8);
    }

    #[test]
    fn test_tkeo_short_input() {
        assert!(tkeo(&[1.0, 2.0]).is_empty());
        assert!(tkeo(&[1.0]).is_empty());
    }

    #[test]
    fn test_tkeo_complex_sine() {
        let omega = 0.2;
        let signal: Vec<Complex64> = (0..200)
            .map(|i| {
                let phase = omega * i as f64;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();
        let energy = tkeo_complex(&signal);
        // For complex exponential e^{jωn}: |x|²=1, Re{x(n-1)·conj(x(n+1))}=cos(2ω)
        // TKEO = 1 - cos(2ω) = 2sin²(ω)
        let expected = 2.0 * omega.sin().powi(2);
        for &e in &energy[10..energy.len() - 10] {
            assert!((e - expected).abs() < 0.02, "e={e} expected={expected}");
        }
    }

    #[test]
    fn test_am_demod() {
        let fc = 0.3; // Carrier frequency
        let fm = 0.01; // Modulation frequency
        let signal: Vec<f64> = (0..500)
            .map(|i| {
                let t = i as f64;
                let envelope = 1.0 + 0.5 * (2.0 * PI * fm * t).sin();
                envelope * (2.0 * PI * fc * t).cos()
            })
            .collect();
        let envelope = tkeo_am_demod(&signal);
        assert!(!envelope.is_empty());
        // Envelope should be positive
        assert!(envelope.iter().all(|&e| e >= 0.0));
    }

    #[test]
    fn test_fm_demod() {
        let fc = 0.1; // Carrier frequency (normalized)
        let signal: Vec<f64> = (0..500)
            .map(|i| (2.0 * PI * fc * i as f64).cos())
            .collect();
        let freq = tkeo_fm_demod(&signal);
        assert!(!freq.is_empty());
        // Estimated frequency should be near fc
        let mid = &freq[20..freq.len() - 20];
        let avg: f64 = mid.iter().sum::<f64>() / mid.len() as f64;
        assert!((avg - fc).abs() < 0.02, "avg={avg} expected={fc}");
    }

    #[test]
    fn test_streaming_operator() {
        let signal: Vec<f64> = (0..50).map(|i| (i as f64 * 0.2).sin()).collect();
        let batch = tkeo(&signal);
        let mut op = TeagerKaiserOperator::new();
        let stream = op.process(&signal);
        assert_eq!(batch.len(), stream.len());
        for (b, s) in batch.iter().zip(stream.iter()) {
            assert!((b - s).abs() < 1e-10);
        }
    }

    #[test]
    fn test_transient_detection() {
        // Create signal with a step transient
        let mut signal = vec![0.0; 50];
        signal.extend(vec![1.0; 50]);
        let transients = detect_transients(&signal, 0.1);
        // Should detect transient near index 50
        assert!(!transients.is_empty());
        let near_50 = transients.iter().any(|&i| (i as i64 - 50).unsigned_abs() < 3);
        assert!(near_50, "transients={transients:?}");
    }

    #[test]
    fn test_reset() {
        let mut op = TeagerKaiserOperator::new();
        op.process(&[1.0, 2.0, 3.0, 4.0]);
        op.reset();
        assert_eq!(op.count, 0);
    }
}
