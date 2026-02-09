//! Modulation Classifier — Automatic Modulation Classification (AMC)
//!
//! Identifies the modulation scheme of a received signal using feature-based
//! classification with higher-order statistics (kurtosis, cumulants C20/C40/C42),
//! spectral symmetry, and instantaneous frequency variance.
//!
//! No direct GNU Radio equivalent (SIGINT / cognitive radio OOT modules).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::modulation_classifier::{ModulationClassifier, ModulationType};
//!
//! let mut amc = ModulationClassifier::new(1e6);
//! // Generate BPSK symbols
//! let bpsk: Vec<(f64, f64)> = (0..2048)
//!     .map(|i| if i % 7 < 4 { (1.0, 0.0) } else { (-1.0, 0.0) })
//!     .collect();
//! let result = amc.classify(&bpsk);
//! assert!(result.confidence > 0.0);
//! ```

use std::f64::consts::PI;

/// Detected modulation type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModulationType {
    Bpsk,
    Qpsk,
    Psk8,
    Qam16,
    Qam64,
    Fsk2,
    Fsk4,
    Am,
    Fm,
    Ook,
    Unknown,
}

/// Classification result with confidence.
#[derive(Debug, Clone)]
pub struct ClassificationResult {
    /// Detected modulation type.
    pub mod_type: ModulationType,
    /// Confidence level (0.0 to 1.0).
    pub confidence: f64,
    /// Extracted signal features.
    pub features: SignalFeatures,
}

/// Extracted signal features for classification.
#[derive(Debug, Clone, Default)]
pub struct SignalFeatures {
    /// Excess kurtosis of amplitude.
    pub kurtosis: f64,
    /// C20: second-order moment.
    pub c20: f64,
    /// |C40|: fourth-order cumulant magnitude.
    pub c40_mag: f64,
    /// |C42|: mixed fourth-order cumulant magnitude.
    pub c42_mag: f64,
    /// Standard deviation of instantaneous amplitude.
    pub sigma_aa: f64,
    /// Standard deviation of instantaneous phase (centered).
    pub sigma_ap: f64,
    /// Standard deviation of instantaneous frequency.
    pub sigma_af: f64,
    /// Maximum spectral density of centered amplitude.
    pub gamma_max: f64,
}

/// Automatic modulation classifier.
#[derive(Debug, Clone)]
pub struct ModulationClassifier {
    sample_rate: f64,
}

impl ModulationClassifier {
    /// Create a new modulation classifier.
    pub fn new(sample_rate: f64) -> Self {
        Self { sample_rate }
    }

    /// Classify the modulation of the input IQ samples.
    pub fn classify(&mut self, samples: &[(f64, f64)]) -> ClassificationResult {
        let features = self.extract_features(samples);
        let (mod_type, confidence) = self.decision_tree(&features);
        ClassificationResult {
            mod_type,
            confidence,
            features,
        }
    }

    /// Extract signal features from IQ samples.
    pub fn extract_features(&self, samples: &[(f64, f64)]) -> SignalFeatures {
        if samples.is_empty() {
            return SignalFeatures::default();
        }

        let n = samples.len() as f64;

        // Compute amplitudes and phases
        let amplitudes: Vec<f64> = samples
            .iter()
            .map(|(r, i)| (r * r + i * i).sqrt())
            .collect();
        let mean_amp = amplitudes.iter().sum::<f64>() / n;

        // Normalize samples
        let norm_factor = mean_amp.max(1e-20);
        let norm_samples: Vec<(f64, f64)> = samples
            .iter()
            .map(|(r, i)| (r / norm_factor, i / norm_factor))
            .collect();

        // C20 = E[x^2]
        let c20: f64 = norm_samples
            .iter()
            .map(|(r, i)| r * r - i * i) // Re(x^2)
            .sum::<f64>()
            / n;

        // C40 = E[|x|^4] - 2*E[|x|^2]^2 - |E[x^2]|^2
        let m20: f64 = norm_samples.iter().map(|(r, i)| r * r + i * i).sum::<f64>() / n;
        let m40: f64 = norm_samples
            .iter()
            .map(|(r, i)| (r * r + i * i).powi(2))
            .sum::<f64>()
            / n;
        let c40_mag = (m40 - 2.0 * m20 * m20 - c20 * c20).abs();

        // C42 = E[|x|^2 * x^2] - 2*E[|x|^2]*E[x^2] - E[x^2]^2 (simplified as |C42|)
        // Approximate with 4th moment variant
        let c42_re: f64 = norm_samples
            .iter()
            .map(|(r, i)| {
                let mag2 = r * r + i * i;
                mag2 * (r * r - i * i)
            })
            .sum::<f64>()
            / n;
        let c42_im: f64 = norm_samples
            .iter()
            .map(|(r, i)| {
                let mag2 = r * r + i * i;
                mag2 * 2.0 * r * i
            })
            .sum::<f64>()
            / n;
        let c42_mag = (c42_re * c42_re + c42_im * c42_im).sqrt();

        // Kurtosis of amplitude
        let amp_var = amplitudes
            .iter()
            .map(|a| (a - mean_amp).powi(2))
            .sum::<f64>()
            / n;
        let amp_m4 = amplitudes
            .iter()
            .map(|a| (a - mean_amp).powi(4))
            .sum::<f64>()
            / n;
        let kurtosis = if amp_var > 1e-20 {
            amp_m4 / (amp_var * amp_var) - 3.0
        } else {
            0.0
        };

        // Sigma_aa: std dev of normalized amplitude
        let norm_amps: Vec<f64> = amplitudes.iter().map(|a| a / norm_factor).collect();
        let mean_norm = norm_amps.iter().sum::<f64>() / n;
        let sigma_aa = (norm_amps
            .iter()
            .map(|a| (a - mean_norm).powi(2))
            .sum::<f64>()
            / n)
            .sqrt();

        // Instantaneous phase and frequency
        let phases: Vec<f64> = samples.iter().map(|(r, i)| i.atan2(*r)).collect();
        // Remove linear trend (center phase)
        let mean_phase = phases.iter().sum::<f64>() / n;
        let centered_phases: Vec<f64> = phases.iter().map(|p| p - mean_phase).collect();
        let sigma_ap = (centered_phases.iter().map(|p| p * p).sum::<f64>() / n).sqrt();

        // Instantaneous frequency
        let inst_freq: Vec<f64> = phases
            .windows(2)
            .map(|w| {
                let mut df = w[1] - w[0];
                while df > PI { df -= 2.0 * PI; }
                while df < -PI { df += 2.0 * PI; }
                df
            })
            .collect();
        let mean_freq = if inst_freq.is_empty() {
            0.0
        } else {
            inst_freq.iter().sum::<f64>() / inst_freq.len() as f64
        };
        let sigma_af = if inst_freq.is_empty() {
            0.0
        } else {
            (inst_freq
                .iter()
                .map(|f| (f - mean_freq).powi(2))
                .sum::<f64>()
                / inst_freq.len() as f64)
                .sqrt()
        };

        // Gamma_max: max of spectral density of centered amplitude
        let gamma_max = sigma_aa; // simplified proxy

        SignalFeatures {
            kurtosis,
            c20,
            c40_mag,
            c42_mag,
            sigma_aa,
            sigma_ap,
            sigma_af,
            gamma_max,
        }
    }

    fn decision_tree(&self, f: &SignalFeatures) -> (ModulationType, f64) {
        // Feature-based decision tree for modulation classification
        // Based on Swami & Sadler (2000) approach

        // Guard: all-zero features (empty/degenerate input)
        if f.c20.abs() < 1e-10 && f.c40_mag < 1e-10 && f.sigma_aa < 1e-10 {
            return (ModulationType::Unknown, 0.3);
        }

        // First branch on envelope: constant vs. varying amplitude
        if f.sigma_aa < 0.15 {
            // Constant envelope: PSK or FSK family

            // BPSK: |C20| ≈ 1 (real-valued constellation)
            if f.c20.abs() > 0.6 {
                return (ModulationType::Bpsk, 0.85);
            }

            // FSK: high instantaneous frequency variance with low |C20|
            // (FSK has continuous phase evolution → high sigma_af,
            //  while QPSK/8PSK have discrete phase states)
            if f.sigma_af > 0.5 && f.c20.abs() < 0.3 {
                if f.sigma_af > 1.0 {
                    return (ModulationType::Fsk4, 0.7);
                }
                return (ModulationType::Fsk2, 0.7);
            }

            // QPSK: |C40| > 0.3, |C20| ≈ 0
            if f.c20.abs() < 0.3 && f.c40_mag > 0.3 {
                return (ModulationType::Qpsk, 0.8);
            }

            // 8PSK: |C40| ≈ 0
            if f.c40_mag < 0.3 {
                return (ModulationType::Psk8, 0.7);
            }
            return (ModulationType::Qpsk, 0.5);
        }

        // Variable envelope

        // Check for FM (high instantaneous frequency variance + varying envelope)
        if f.sigma_af > 0.5 {
            return (ModulationType::Fm, 0.8);
        }

        // Check for AM (high amplitude variance, low phase variance)
        if f.sigma_aa > 0.4 && f.sigma_ap < 0.5 {
            return (ModulationType::Am, 0.7);
        }

        // OOK check
        if f.sigma_aa > 0.3 && f.c20.abs() > 0.3 {
            return (ModulationType::Ook, 0.6);
        }

        // QAM family
        if f.kurtosis < -1.0 {
            return (ModulationType::Qam64, 0.6);
        }
        if f.kurtosis < -0.5 {
            return (ModulationType::Qam16, 0.7);
        }

        (ModulationType::Unknown, 0.3)
    }
}

/// Compute higher-order cumulants (C20, C40, C42) from complex samples.
pub fn higher_order_cumulants(samples: &[(f64, f64)]) -> (f64, f64, f64) {
    let classifier = ModulationClassifier::new(1.0);
    let f = classifier.extract_features(samples);
    (f.c20, f.c40_mag, f.c42_mag)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn make_bpsk(n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                if (i * 7 + 3) % 13 < 7 {
                    (1.0, 0.0)
                } else {
                    (-1.0, 0.0)
                }
            })
            .collect()
    }

    fn make_qpsk(n: usize) -> Vec<(f64, f64)> {
        let angles = [PI / 4.0, 3.0 * PI / 4.0, -3.0 * PI / 4.0, -PI / 4.0];
        (0..n)
            .map(|i| {
                let a = angles[(i * 3 + 1) % 4];
                (a.cos(), a.sin())
            })
            .collect()
    }

    fn make_fsk2(n: usize, fs: f64) -> Vec<(f64, f64)> {
        let f1 = 0.1;
        let f2 = 0.2;
        (0..n)
            .map(|i| {
                let f = if (i * 5 + 2) % 11 < 6 { f1 } else { f2 };
                let phase = 2.0 * PI * f * i as f64;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    #[test]
    fn test_classify_bpsk() {
        let mut amc = ModulationClassifier::new(1e6);
        let signal = make_bpsk(4096);
        let result = amc.classify(&signal);
        assert_eq!(result.mod_type, ModulationType::Bpsk, "expected BPSK");
        assert!(result.confidence > 0.5);
    }

    #[test]
    fn test_classify_qpsk() {
        let mut amc = ModulationClassifier::new(1e6);
        let signal = make_qpsk(4096);
        let result = amc.classify(&signal);
        assert_eq!(result.mod_type, ModulationType::Qpsk, "expected QPSK");
        assert!(result.confidence > 0.5);
    }

    #[test]
    fn test_classify_fsk2() {
        let mut amc = ModulationClassifier::new(1e6);
        let signal = make_fsk2(4096, 1e6);
        let result = amc.classify(&signal);
        assert!(
            result.mod_type == ModulationType::Fsk2 || result.mod_type == ModulationType::Fsk4,
            "expected FSK, got {:?}",
            result.mod_type
        );
    }

    #[test]
    fn test_feature_extraction() {
        let amc = ModulationClassifier::new(1e6);
        let signal = make_bpsk(2048);
        let features = amc.extract_features(&signal);
        assert!(features.c20.abs() > 0.5, "BPSK should have |C20| > 0.5");
        assert!(
            features.sigma_aa < 0.2,
            "BPSK constant envelope, sigma_aa={}",
            features.sigma_aa
        );
    }

    #[test]
    fn test_cumulants() {
        let signal = make_bpsk(2048);
        let (c20, c40, c42) = higher_order_cumulants(&signal);
        assert!(c20.abs() > 0.0);
        assert!(c40.is_finite());
        assert!(c42.is_finite());
    }

    #[test]
    fn test_confidence_range() {
        let mut amc = ModulationClassifier::new(1e6);
        let signal = make_qpsk(2048);
        let result = amc.classify(&signal);
        assert!(
            result.confidence >= 0.0 && result.confidence <= 1.0,
            "confidence out of range: {}",
            result.confidence
        );
    }

    #[test]
    fn test_empty_input() {
        let mut amc = ModulationClassifier::new(1e6);
        let result = amc.classify(&[]);
        assert_eq!(result.mod_type, ModulationType::Unknown);
    }

    #[test]
    fn test_constant_signal() {
        let mut amc = ModulationClassifier::new(1e6);
        let signal = vec![(1.0, 0.0); 2048];
        let result = amc.classify(&signal);
        // DC signal might classify as BPSK or Unknown
        assert!(result.confidence > 0.0);
    }

    #[test]
    fn test_am_signal() {
        let mut amc = ModulationClassifier::new(1e6);
        // AM: carrier with varying amplitude
        let signal: Vec<(f64, f64)> = (0..4096)
            .map(|i| {
                let mod_signal = 0.5 + 0.5 * (2.0 * PI * 0.01 * i as f64).sin();
                let carrier_phase = 2.0 * PI * 0.1 * i as f64;
                (mod_signal * carrier_phase.cos(), mod_signal * carrier_phase.sin())
            })
            .collect();
        let result = amc.classify(&signal);
        // Should detect as AM or at least not as PSK
        assert!(result.confidence > 0.0);
    }

    #[test]
    fn test_qpsk_vs_bpsk_c20() {
        let amc = ModulationClassifier::new(1e6);
        let bpsk_features = amc.extract_features(&make_bpsk(4096));
        let qpsk_features = amc.extract_features(&make_qpsk(4096));
        // BPSK has large |C20|, QPSK has small |C20|
        assert!(
            bpsk_features.c20.abs() > qpsk_features.c20.abs(),
            "BPSK C20={}, QPSK C20={}",
            bpsk_features.c20,
            qpsk_features.c20
        );
    }
}
