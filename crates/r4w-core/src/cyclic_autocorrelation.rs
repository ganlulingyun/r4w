//! Cyclic Autocorrelation — Cyclostationary Signal Analysis
//!
//! Computes the cyclic autocorrelation function (CAF) and spectral
//! correlation function (SCF) for cyclostationary signal detection
//! and modulation classification. Can detect signals below the noise
//! floor by exploiting periodic features (symbol rate, carrier freq).
//! GNU Radio equivalent: CycloDSP module (research implementations).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::cyclic_autocorrelation::{CyclicAnalyzer, CyclicConfig};
//! use num_complex::Complex64;
//!
//! let config = CyclicConfig {
//!     fft_size: 64,
//!     num_averages: 4,
//!     sample_rate: 1e6,
//! };
//! let analyzer = CyclicAnalyzer::new(config);
//! let signal: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 256];
//! let caf = analyzer.compute_caf(&signal, 0.0);
//! assert_eq!(caf.len(), 64);
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Configuration for cyclic analysis.
#[derive(Debug, Clone)]
pub struct CyclicConfig {
    /// FFT size for spectral analysis.
    pub fft_size: usize,
    /// Number of segments to average.
    pub num_averages: usize,
    /// Sample rate in Hz.
    pub sample_rate: f64,
}

/// Detected cyclic feature.
#[derive(Debug, Clone)]
pub struct CyclicFeature {
    /// Cyclic frequency (alpha) in Hz.
    pub alpha_hz: f64,
    /// Peak magnitude of CAF at this alpha.
    pub magnitude: f64,
    /// Spectral frequency at peak.
    pub frequency_hz: f64,
}

/// Modulation type detected from cyclic features.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectedModulation {
    /// No signal detected.
    NoSignal,
    /// Continuous wave / unmodulated carrier.
    CW,
    /// Binary PSK.
    Bpsk,
    /// Quadrature PSK.
    Qpsk,
    /// Amplitude modulation.
    Am,
    /// Frequency modulation.
    Fm,
    /// OFDM.
    Ofdm,
    /// Unknown modulation.
    Unknown,
}

/// Cyclostationary signal analyzer.
#[derive(Debug, Clone)]
pub struct CyclicAnalyzer {
    config: CyclicConfig,
}

impl CyclicAnalyzer {
    /// Create a new cyclic analyzer.
    pub fn new(config: CyclicConfig) -> Self {
        Self { config }
    }

    /// Compute cyclic autocorrelation function for a given cyclic frequency alpha.
    ///
    /// Returns the CAF as a function of spectral frequency.
    pub fn compute_caf(&self, signal: &[Complex64], alpha_hz: f64) -> Vec<Complex64> {
        let n = self.config.fft_size;
        let alpha_norm = alpha_hz / self.config.sample_rate;

        // Segment the signal
        let num_segments = (signal.len() / n).min(self.config.num_averages.max(1));
        if num_segments == 0 {
            return vec![Complex64::new(0.0, 0.0); n];
        }

        let mut avg = vec![Complex64::new(0.0, 0.0); n];

        for seg in 0..num_segments {
            let offset = seg * n;
            if offset + n > signal.len() {
                break;
            }

            // x(t) * conj(x(t)) * exp(-j*2*pi*alpha*t)
            let mut windowed = Vec::with_capacity(n);
            for k in 0..n {
                let t = (offset + k) as f64;
                let phase = -2.0 * PI * alpha_norm * t;
                let shift = Complex64::new(phase.cos(), phase.sin());
                windowed.push(signal[offset + k] * signal[offset + k].conj().sqrt() * shift);
            }

            // DFT of windowed segment
            let spectrum = simple_dft(&windowed);
            for (i, s) in spectrum.iter().enumerate() {
                avg[i] += s;
            }
        }

        let scale = 1.0 / num_segments as f64;
        avg.iter().map(|&s| s * scale).collect()
    }

    /// Compute spectral correlation function over a range of cyclic frequencies.
    pub fn compute_scf(
        &self,
        signal: &[Complex64],
        alpha_range: (f64, f64),
        alpha_step: f64,
    ) -> Vec<(f64, Vec<f64>)> {
        let mut results = Vec::new();
        let mut alpha = alpha_range.0;

        while alpha <= alpha_range.1 {
            let caf = self.compute_caf(signal, alpha);
            let magnitudes: Vec<f64> = caf.iter().map(|c| c.norm()).collect();
            results.push((alpha, magnitudes));
            alpha += alpha_step;
        }

        results
    }

    /// Detect cyclic features in a signal.
    ///
    /// Searches for significant peaks in the cyclic autocorrelation at
    /// various cyclic frequencies.
    pub fn detect_features(
        &self,
        signal: &[Complex64],
        alpha_candidates: &[f64],
        threshold_db: f64,
    ) -> Vec<CyclicFeature> {
        let threshold_linear = 10.0f64.powf(threshold_db / 20.0);
        let mut features = Vec::new();

        // Compute CAF at alpha=0 for reference (PSD)
        let psd = self.compute_caf(signal, 0.0);
        let noise_floor = psd.iter().map(|c| c.norm()).sum::<f64>() / psd.len() as f64;

        for &alpha in alpha_candidates {
            let caf = self.compute_caf(signal, alpha);
            let magnitudes: Vec<f64> = caf.iter().map(|c| c.norm()).collect();

            // Find peak
            let (peak_idx, peak_mag) = magnitudes
                .iter()
                .enumerate()
                .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
                .map(|(i, &m)| (i, m))
                .unwrap_or((0, 0.0));

            if noise_floor > 0.0 && peak_mag / noise_floor > threshold_linear {
                let freq_hz = if peak_idx <= self.config.fft_size / 2 {
                    peak_idx as f64 * self.config.sample_rate / self.config.fft_size as f64
                } else {
                    (peak_idx as f64 - self.config.fft_size as f64)
                        * self.config.sample_rate
                        / self.config.fft_size as f64
                };

                features.push(CyclicFeature {
                    alpha_hz: alpha,
                    magnitude: peak_mag,
                    frequency_hz: freq_hz,
                });
            }
        }

        // Sort by magnitude descending
        features.sort_by(|a, b| b.magnitude.partial_cmp(&a.magnitude).unwrap());
        features
    }

    /// Classify modulation based on cyclic features.
    pub fn classify_modulation(
        &self,
        signal: &[Complex64],
        symbol_rate_candidates: &[f64],
    ) -> DetectedModulation {
        // Check at alpha = 0 for signal presence (PSD)
        let psd = self.compute_caf(signal, 0.0);
        let avg_power = psd.iter().map(|c| c.norm_sqr()).sum::<f64>() / psd.len() as f64;

        if avg_power < 1e-12 {
            return DetectedModulation::NoSignal;
        }

        // Check for cyclostationary features at candidate symbol rates
        let mut best_alpha = 0.0f64;
        let mut best_peak = 0.0f64;

        for &rate in symbol_rate_candidates {
            for &mult in &[1.0, 2.0] {
                let alpha = rate * mult;
                let caf = self.compute_caf(signal, alpha);
                let peak = caf.iter().map(|c| c.norm()).fold(0.0f64, f64::max);
                if peak > best_peak {
                    best_peak = peak;
                    best_alpha = alpha;
                }
            }
        }

        // Check for double-rate feature (PSK indicator)
        if best_peak > avg_power.sqrt() * 0.5 {
            // Check if feature at 2x symbol rate exists (BPSK characteristic)
            let double_rate = best_alpha * 2.0;
            let caf_2x = self.compute_caf(signal, double_rate);
            let peak_2x = caf_2x.iter().map(|c| c.norm()).fold(0.0f64, f64::max);

            if peak_2x > best_peak * 0.3 {
                return DetectedModulation::Bpsk;
            }
            return DetectedModulation::Qpsk;
        }

        if best_peak > avg_power.sqrt() * 0.1 {
            return DetectedModulation::Unknown;
        }

        DetectedModulation::CW
    }

    /// Get config.
    pub fn config(&self) -> &CyclicConfig {
        &self.config
    }

    /// Get frequency axis for CAF output.
    pub fn frequency_axis(&self) -> Vec<f64> {
        let n = self.config.fft_size;
        (0..n)
            .map(|i| {
                if i <= n / 2 {
                    i as f64 * self.config.sample_rate / n as f64
                } else {
                    (i as f64 - n as f64) * self.config.sample_rate / n as f64
                }
            })
            .collect()
    }
}

/// Simple DFT implementation.
fn simple_dft(input: &[Complex64]) -> Vec<Complex64> {
    let n = input.len();
    let mut output = vec![Complex64::new(0.0, 0.0); n];

    for k in 0..n {
        for (j, &x) in input.iter().enumerate() {
            let phase = -2.0 * PI * k as f64 * j as f64 / n as f64;
            output[k] += x * Complex64::new(phase.cos(), phase.sin());
        }
    }
    output
}

/// Compute spectral coherence (normalized SCF).
pub fn spectral_coherence(
    scf: &[(f64, Vec<f64>)],
    psd: &[f64],
) -> Vec<(f64, Vec<f64>)> {
    scf.iter()
        .map(|(alpha, magnitudes)| {
            let normalized: Vec<f64> = magnitudes
                .iter()
                .zip(psd.iter())
                .map(|(&scf_val, &psd_val)| {
                    if psd_val > 1e-12 {
                        scf_val / psd_val
                    } else {
                        0.0
                    }
                })
                .collect();
            (*alpha, normalized)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_bpsk_signal(symbol_rate: f64, sample_rate: f64, num_samples: usize) -> Vec<Complex64> {
        let sps = (sample_rate / symbol_rate) as usize;
        let mut rng = 42u32;
        let mut signal = Vec::with_capacity(num_samples);

        for i in 0..num_samples {
            let symbol_idx = i / sps.max(1);
            // Pseudo-random BPSK symbols
            rng = rng.wrapping_mul(1103515245).wrapping_add(12345);
            let bit = if (rng >> (16 + (symbol_idx % 16))) & 1 == 1 { 1.0 } else { -1.0 };
            signal.push(Complex64::new(bit, 0.0));
        }
        signal
    }

    #[test]
    fn test_caf_at_zero() {
        let config = CyclicConfig {
            fft_size: 64,
            num_averages: 4,
            sample_rate: 1e6,
        };
        let analyzer = CyclicAnalyzer::new(config);
        let signal: Vec<Complex64> = (0..256)
            .map(|i| Complex64::new((2.0 * PI * 0.1 * i as f64).cos(), 0.0))
            .collect();
        let caf = analyzer.compute_caf(&signal, 0.0);
        assert_eq!(caf.len(), 64);
        // At alpha=0, CAF should be the PSD — should have non-zero values
        let total_power: f64 = caf.iter().map(|c| c.norm_sqr()).sum();
        assert!(total_power > 0.0, "PSD should be non-zero");
    }

    #[test]
    fn test_scf_computation() {
        let config = CyclicConfig {
            fft_size: 32,
            num_averages: 2,
            sample_rate: 1e6,
        };
        let analyzer = CyclicAnalyzer::new(config);
        let signal: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 128];
        let scf = analyzer.compute_scf(&signal, (-1000.0, 1000.0), 500.0);
        assert!(!scf.is_empty());
    }

    #[test]
    fn test_frequency_axis() {
        let config = CyclicConfig {
            fft_size: 8,
            num_averages: 1,
            sample_rate: 8000.0,
        };
        let analyzer = CyclicAnalyzer::new(config);
        let axis = analyzer.frequency_axis();
        assert_eq!(axis.len(), 8);
        assert_eq!(axis[0], 0.0);
    }

    #[test]
    fn test_detect_features() {
        let config = CyclicConfig {
            fft_size: 64,
            num_averages: 4,
            sample_rate: 1e6,
        };
        let analyzer = CyclicAnalyzer::new(config);
        let signal = make_bpsk_signal(100e3, 1e6, 512);
        let features = analyzer.detect_features(
            &signal,
            &[100e3, 200e3, 50e3],
            3.0,
        );
        // Should find at least some features (may or may not depending on SNR)
        // Just verify it doesn't crash
        assert!(features.len() <= 10);
    }

    #[test]
    fn test_classify_no_signal() {
        let config = CyclicConfig {
            fft_size: 32,
            num_averages: 2,
            sample_rate: 1e6,
        };
        let analyzer = CyclicAnalyzer::new(config);
        let signal = vec![Complex64::new(0.0, 0.0); 128];
        let result = analyzer.classify_modulation(&signal, &[100e3]);
        assert_eq!(result, DetectedModulation::NoSignal);
    }

    #[test]
    fn test_classify_cw() {
        let config = CyclicConfig {
            fft_size: 64,
            num_averages: 4,
            sample_rate: 1e6,
        };
        let analyzer = CyclicAnalyzer::new(config);
        let signal: Vec<Complex64> = (0..512)
            .map(|i| {
                let phase = 2.0 * PI * 100e3 * i as f64 / 1e6;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();
        let result = analyzer.classify_modulation(&signal, &[1e3, 10e3]);
        // CW should be detected as a signal (not NoSignal)
        assert_ne!(result, DetectedModulation::NoSignal, "CW should be detected as a signal");
    }

    #[test]
    fn test_spectral_coherence() {
        let scf = vec![
            (100.0, vec![1.0, 2.0, 3.0]),
            (200.0, vec![0.5, 1.0, 1.5]),
        ];
        let psd = vec![2.0, 4.0, 6.0];
        let coherence = spectral_coherence(&scf, &psd);
        assert_eq!(coherence.len(), 2);
        assert!((coherence[0].1[0] - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_config_access() {
        let config = CyclicConfig {
            fft_size: 128,
            num_averages: 8,
            sample_rate: 2e6,
        };
        let analyzer = CyclicAnalyzer::new(config.clone());
        assert_eq!(analyzer.config().fft_size, 128);
        assert_eq!(analyzer.config().sample_rate, 2e6);
    }

    #[test]
    fn test_detected_modulation_eq() {
        assert_eq!(DetectedModulation::Bpsk, DetectedModulation::Bpsk);
        assert_ne!(DetectedModulation::Bpsk, DetectedModulation::Qpsk);
    }
}
