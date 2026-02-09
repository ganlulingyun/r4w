//! FM Pre-emphasis and De-emphasis Filters
//!
//! FM broadcast and two-way radio use pre-emphasis on transmit and de-emphasis
//! on receive to improve SNR at high audio frequencies.
//!
//! ## Standards
//!
//! | Region       | Time constant | Cutoff   |
//! |-------------|---------------|----------|
//! | US / Korea  | 75 µs         | 2122 Hz  |
//! | Europe / AU | 50 µs         | 3183 Hz  |
//!
//! ## Signal Flow
//!
//! ```text
//! TX: audio → [pre-emphasis (+6 dB/octave above f_c)] → FM modulator
//! RX: FM demod → [de-emphasis (-6 dB/octave above f_c)] → audio
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::fm_emphasis::{DeEmphasisFilter, PreEmphasisFilter, EmphasisStandard};
//!
//! // US FM broadcast (75 µs)
//! let mut deemph = DeEmphasisFilter::new(EmphasisStandard::Us75, 48000.0);
//! let mut preemph = PreEmphasisFilter::new(EmphasisStandard::Us75, 48000.0);
//!
//! let input = vec![1.0f64; 100];
//! let emphasized = preemph.process_block(&input);
//! let recovered = deemph.process_block(&emphasized);
//! // recovered ≈ input (after settling)
//! ```

use std::f64::consts::PI;

/// Emphasis time constant standard.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EmphasisStandard {
    /// US / Korea: 75 µs (2122 Hz cutoff)
    Us75,
    /// Europe / Australia: 50 µs (3183 Hz cutoff)
    Eu50,
    /// Custom time constant in microseconds
    Custom(u32),
}

impl EmphasisStandard {
    /// Get time constant in seconds.
    pub fn tau_seconds(&self) -> f64 {
        match self {
            Self::Us75 => 75e-6,
            Self::Eu50 => 50e-6,
            Self::Custom(us) => *us as f64 * 1e-6,
        }
    }

    /// Get -3 dB cutoff frequency in Hz.
    pub fn cutoff_hz(&self) -> f64 {
        1.0 / (2.0 * PI * self.tau_seconds())
    }
}

/// FM de-emphasis filter (1-pole IIR lowpass).
///
/// Transfer function: H(z) = (1 - α) / (1 - α·z^-1)
/// where α = exp(-1 / (τ·fs))
#[derive(Debug, Clone)]
pub struct DeEmphasisFilter {
    /// IIR coefficient
    alpha: f64,
    /// Previous output
    y1: f64,
}

impl DeEmphasisFilter {
    /// Create a new de-emphasis filter.
    pub fn new(standard: EmphasisStandard, sample_rate: f64) -> Self {
        let tau = standard.tau_seconds();
        let alpha = (-1.0 / (tau * sample_rate)).exp();
        Self { alpha, y1: 0.0 }
    }

    /// Create with custom time constant in seconds.
    pub fn with_tau(tau_seconds: f64, sample_rate: f64) -> Self {
        let alpha = (-1.0 / (tau_seconds * sample_rate)).exp();
        Self { alpha, y1: 0.0 }
    }

    /// Process a single sample.
    pub fn process(&mut self, input: f64) -> f64 {
        let y = (1.0 - self.alpha) * input + self.alpha * self.y1;
        self.y1 = y;
        y
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.process(x)).collect()
    }

    /// Process in-place.
    pub fn process_inplace(&mut self, samples: &mut [f64]) {
        for s in samples.iter_mut() {
            *s = self.process(*s);
        }
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.y1 = 0.0;
    }
}

/// FM pre-emphasis filter (1-pole IIR highpass / differentiator).
///
/// Transfer function: H(z) = (1 - α·z^-1) / (1 - α)
/// Inverse of de-emphasis: boosts high frequencies.
#[derive(Debug, Clone)]
pub struct PreEmphasisFilter {
    /// IIR coefficient
    alpha: f64,
    /// Previous input
    x1: f64,
}

impl PreEmphasisFilter {
    /// Create a new pre-emphasis filter.
    pub fn new(standard: EmphasisStandard, sample_rate: f64) -> Self {
        let tau = standard.tau_seconds();
        let alpha = (-1.0 / (tau * sample_rate)).exp();
        Self { alpha, x1: 0.0 }
    }

    /// Create with custom time constant in seconds.
    pub fn with_tau(tau_seconds: f64, sample_rate: f64) -> Self {
        let alpha = (-1.0 / (tau_seconds * sample_rate)).exp();
        Self { alpha, x1: 0.0 }
    }

    /// Process a single sample.
    pub fn process(&mut self, input: f64) -> f64 {
        let y = (input - self.alpha * self.x1) / (1.0 - self.alpha);
        self.x1 = input;
        y
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&x| self.process(x)).collect()
    }

    /// Process in-place.
    pub fn process_inplace(&mut self, samples: &mut [f64]) {
        for s in samples.iter_mut() {
            *s = self.process(*s);
        }
    }

    /// Reset filter state.
    pub fn reset(&mut self) {
        self.x1 = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_emphasis_standards() {
        let us = EmphasisStandard::Us75;
        assert!((us.tau_seconds() - 75e-6).abs() < 1e-10);
        assert!((us.cutoff_hz() - 2122.07).abs() < 1.0);

        let eu = EmphasisStandard::Eu50;
        assert!((eu.tau_seconds() - 50e-6).abs() < 1e-10);
        assert!((eu.cutoff_hz() - 3183.1).abs() < 1.0);
    }

    #[test]
    fn test_deemphasis_attenuates_high_freq() {
        let fs = 48000.0;
        let mut deemph = DeEmphasisFilter::new(EmphasisStandard::Us75, fs);

        // Generate high-frequency tone (10 kHz)
        let n = 4800;
        let input: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 10000.0 * i as f64 / fs).sin())
            .collect();

        let output = deemph.process_block(&input);

        // Measure output power (skip settling)
        let in_power: f64 = input[1000..].iter().map(|x| x * x).sum::<f64>() / (n - 1000) as f64;
        let out_power: f64 = output[1000..].iter().map(|x| x * x).sum::<f64>() / (n - 1000) as f64;

        // 10 kHz should be significantly attenuated (> 10 dB)
        let attenuation_db = 10.0 * (in_power / out_power).log10();
        assert!(
            attenuation_db > 8.0,
            "10 kHz should be attenuated by >8 dB: got {:.1} dB",
            attenuation_db
        );
    }

    #[test]
    fn test_deemphasis_passes_low_freq() {
        let fs = 48000.0;
        let mut deemph = DeEmphasisFilter::new(EmphasisStandard::Us75, fs);

        // Generate low-frequency tone (100 Hz)
        let n = 4800;
        let input: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 100.0 * i as f64 / fs).sin())
            .collect();

        let output = deemph.process_block(&input);

        // Low frequency should pass with minimal attenuation
        let in_power: f64 = input[1000..].iter().map(|x| x * x).sum::<f64>() / (n - 1000) as f64;
        let out_power: f64 = output[1000..].iter().map(|x| x * x).sum::<f64>() / (n - 1000) as f64;

        let attenuation_db = 10.0 * (in_power / out_power).log10();
        assert!(
            attenuation_db < 1.0,
            "100 Hz should pass with <1 dB loss: got {:.1} dB",
            attenuation_db
        );
    }

    #[test]
    fn test_preemphasis_boosts_high_freq() {
        let fs = 48000.0;
        let mut preemph = PreEmphasisFilter::new(EmphasisStandard::Us75, fs);

        // Generate high-frequency tone (10 kHz)
        let n = 4800;
        let input: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 10000.0 * i as f64 / fs).sin())
            .collect();

        let output = preemph.process_block(&input);

        // Measure power (skip settling)
        let in_power: f64 = input[1000..].iter().map(|x| x * x).sum::<f64>() / (n - 1000) as f64;
        let out_power: f64 = output[1000..].iter().map(|x| x * x).sum::<f64>() / (n - 1000) as f64;

        // 10 kHz should be boosted (> 8 dB)
        let boost_db = 10.0 * (out_power / in_power).log10();
        assert!(
            boost_db > 8.0,
            "10 kHz should be boosted by >8 dB: got {:.1} dB",
            boost_db
        );
    }

    #[test]
    fn test_emphasis_roundtrip() {
        let fs = 48000.0;
        let mut preemph = PreEmphasisFilter::new(EmphasisStandard::Us75, fs);
        let mut deemph = DeEmphasisFilter::new(EmphasisStandard::Us75, fs);

        // Generate mixed-frequency signal
        let n = 4800;
        let input: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                0.5 * (2.0 * PI * 300.0 * t).sin() + 0.5 * (2.0 * PI * 5000.0 * t).sin()
            })
            .collect();

        let emphasized = preemph.process_block(&input);
        let recovered = deemph.process_block(&emphasized);

        // After settling, recovered should match input closely
        let error: f64 = input[2000..]
            .iter()
            .zip(recovered[2000..].iter())
            .map(|(a, b)| (a - b).powi(2))
            .sum::<f64>()
            / (n - 2000) as f64;

        assert!(
            error < 0.01,
            "Roundtrip MSE should be small: got {:.6}",
            error
        );
    }

    #[test]
    fn test_deemphasis_reset() {
        let mut deemph = DeEmphasisFilter::new(EmphasisStandard::Us75, 48000.0);
        deemph.process(1.0);
        assert!(deemph.y1.abs() > 0.0);
        deemph.reset();
        assert!(deemph.y1.abs() < 1e-10);
    }

    #[test]
    fn test_custom_time_constant() {
        let custom = EmphasisStandard::Custom(100);
        assert!((custom.tau_seconds() - 100e-6).abs() < 1e-10);
    }

    #[test]
    fn test_eu50_standard() {
        let fs = 48000.0;
        let mut deemph_us = DeEmphasisFilter::new(EmphasisStandard::Us75, fs);
        let mut deemph_eu = DeEmphasisFilter::new(EmphasisStandard::Eu50, fs);

        // EU has higher cutoff → less attenuation at same frequency
        let input: Vec<f64> = (0..4800)
            .map(|i| (2.0 * PI * 5000.0 * i as f64 / fs).sin())
            .collect();

        let out_us = deemph_us.process_block(&input);
        let out_eu = deemph_eu.process_block(&input);

        let pow_us: f64 = out_us[1000..].iter().map(|x| x * x).sum::<f64>();
        let pow_eu: f64 = out_eu[1000..].iter().map(|x| x * x).sum::<f64>();

        // EU 50µs should have more power through at 5 kHz than US 75µs
        assert!(
            pow_eu > pow_us,
            "EU should attenuate less at 5 kHz: eu={:.4}, us={:.4}",
            pow_eu,
            pow_us
        );
    }
}
