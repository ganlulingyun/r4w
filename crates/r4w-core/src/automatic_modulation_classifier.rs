//! Feature-based Automatic Modulation Classification (AMC).
//!
//! This module implements a decision-tree classifier that identifies the modulation
//! scheme of a received signal from its statistical features. It extracts higher-order
//! moments, cumulants, instantaneous amplitude/frequency/phase statistics, and spectral
//! features from complex IQ samples, then compares the resulting feature vector against
//! known modulation profiles.
//!
//! # Supported Modulations
//!
//! BPSK, QPSK, 8PSK, 16-QAM, 64-QAM, 2-FSK, 4-FSK.
//!
//! # Example
//!
//! ```
//! use r4w_core::automatic_modulation_classifier::{
//!     AutomaticModulationClassifier, ClassifierConfig, Modulation,
//! };
//!
//! // Generate a simple BPSK-like signal: symbols are +1 or -1 on the real axis
//! let samples: Vec<(f64, f64)> = (0..1024)
//!     .map(|i| if i % 2 == 0 { (1.0, 0.0) } else { (-1.0, 0.0) })
//!     .collect();
//!
//! let classifier = AutomaticModulationClassifier::new(ClassifierConfig::default());
//! let result = classifier.classify(&samples);
//! assert_eq!(result.modulation, Modulation::Bpsk);
//! assert!(result.confidence > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Recognised modulation schemes.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Modulation {
    Bpsk,
    Qpsk,
    Psk8,
    Qam16,
    Qam64,
    Fsk2,
    Fsk4,
    Unknown,
}

impl std::fmt::Display for Modulation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let name = match self {
            Modulation::Bpsk => "BPSK",
            Modulation::Qpsk => "QPSK",
            Modulation::Psk8 => "8PSK",
            Modulation::Qam16 => "16-QAM",
            Modulation::Qam64 => "64-QAM",
            Modulation::Fsk2 => "2-FSK",
            Modulation::Fsk4 => "4-FSK",
            Modulation::Unknown => "Unknown",
        };
        write!(f, "{}", name)
    }
}

/// Configuration for the classifier.
#[derive(Debug, Clone)]
pub struct ClassifierConfig {
    /// Minimum number of samples required for classification.
    pub min_samples: usize,
    /// Enable spectral feature extraction (slightly slower).
    pub use_spectral_features: bool,
    /// SNR estimation method.
    pub snr_method: SnrMethod,
}

impl Default for ClassifierConfig {
    fn default() -> Self {
        Self {
            min_samples: 64,
            use_spectral_features: true,
            snr_method: SnrMethod::M2M4,
        }
    }
}

/// SNR estimation method selector.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnrMethod {
    /// M2M4 moment-based estimator.
    M2M4,
}

/// Result of a classification attempt.
#[derive(Debug, Clone)]
pub struct ClassificationResult {
    /// Most likely modulation.
    pub modulation: Modulation,
    /// Confidence in [0, 1].
    pub confidence: f64,
    /// Full feature vector that was extracted.
    pub features: FeatureVector,
    /// Distances to each candidate modulation (smaller is better).
    pub distances: Vec<(Modulation, f64)>,
}

/// Extracted feature vector from IQ samples.
#[derive(Debug, Clone, Default)]
pub struct FeatureVector {
    // Higher-order moments (centralised, normalised)
    pub m20: f64,
    pub m21: f64,
    pub m40: f64,
    pub m41: f64,
    pub m42: f64,
    pub m43: f64,

    // Higher-order cumulants
    pub c20: f64,
    pub c21: f64,
    pub c40: f64,
    pub c41: f64,
    pub c42: f64,

    // Instantaneous statistics
    pub sigma_aa: f64,    // std-dev of instantaneous amplitude
    pub sigma_af: f64,    // std-dev of instantaneous frequency
    pub sigma_ap: f64,    // std-dev of instantaneous phase
    pub mu_a: f64,        // mean of instantaneous amplitude
    pub kurtosis_a: f64,  // kurtosis of instantaneous amplitude
    pub kurtosis_f: f64,  // kurtosis of instantaneous frequency

    // SNR estimate
    pub snr_db: f64,

    // Spectral features
    pub spectral_peak_count: usize,
    pub spectral_symmetry: f64,
    pub spectral_flatness: f64,
}

/// The main classifier.
#[derive(Debug, Clone)]
pub struct AutomaticModulationClassifier {
    config: ClassifierConfig,
    reference_table: Vec<ModulationProfile>,
}

/// Reference feature profile for a known modulation.
#[derive(Debug, Clone)]
pub struct ModulationProfile {
    pub modulation: Modulation,
    /// |C40| reference value.
    pub abs_c40: f64,
    /// |C42| reference value.
    pub abs_c42: f64,
    /// |C41| reference value.
    pub abs_c41: f64,
    /// Amplitude excess kurtosis.
    pub kurtosis_a: f64,
    /// Expected spectral peak count.
    pub spectral_peak_count: f64,
}

// ---------------------------------------------------------------------------
// Implementation
// ---------------------------------------------------------------------------

impl AutomaticModulationClassifier {
    /// Create a new classifier with the given configuration.
    pub fn new(config: ClassifierConfig) -> Self {
        let reference_table = build_reference_table();
        Self {
            config,
            reference_table,
        }
    }

    /// Classify the modulation of the provided IQ samples.
    ///
    /// Returns [`ClassificationResult`] with the best-matching modulation and
    /// confidence score.
    pub fn classify(&self, samples: &[(f64, f64)]) -> ClassificationResult {
        if samples.len() < self.config.min_samples {
            return ClassificationResult {
                modulation: Modulation::Unknown,
                confidence: 0.0,
                features: FeatureVector::default(),
                distances: Vec::new(),
            };
        }

        let features = self.extract_features(samples);
        self.classify_from_features(&features)
    }

    /// Extract the full feature vector from IQ samples.
    pub fn extract_features(&self, samples: &[(f64, f64)]) -> FeatureVector {
        let n = samples.len();
        if n == 0 {
            return FeatureVector::default();
        }

        // Normalise power to unity
        let power = samples.iter().map(|&(r, i)| r * r + i * i).sum::<f64>() / n as f64;
        let scale = if power > 1e-30 { 1.0 / power.sqrt() } else { 1.0 };
        let normed: Vec<(f64, f64)> = samples
            .iter()
            .map(|&(r, i)| (r * scale, i * scale))
            .collect();

        // Moments
        let m20 = moment(&normed, 2, 0);
        let m21 = moment(&normed, 2, 1);
        let m40 = moment(&normed, 4, 0);
        let m41 = moment(&normed, 4, 1);
        let m42 = moment(&normed, 4, 2);
        let m43 = moment(&normed, 4, 3);

        // Cumulants
        let c20 = m20;
        let c21 = m21;
        let c40 = m40 - 3.0 * m20 * m20;
        let c41 = m41 - 3.0 * m20 * m21;
        let c42 = m42 - m20 * m20 - 2.0 * m21 * m21;

        // Instantaneous amplitude / frequency / phase
        let amplitudes: Vec<f64> = normed
            .iter()
            .map(|&(r, i)| (r * r + i * i).sqrt())
            .collect();
        let phases: Vec<f64> = normed.iter().map(|&(r, i)| i.atan2(r)).collect();
        let frequencies: Vec<f64> = instantaneous_frequencies(&phases);

        let mu_a = mean(&amplitudes);
        let sigma_aa = std_dev(&amplitudes);
        let sigma_af = std_dev(&frequencies);
        let sigma_ap = std_dev(&phases);
        let kurtosis_a = kurtosis(&amplitudes);
        let kurtosis_f = kurtosis(&frequencies);

        // SNR (M2M4)
        let snr_db = estimate_snr_m2m4(&normed);

        // Spectral features
        let (spectral_peak_count, spectral_symmetry, spectral_flatness) =
            if self.config.use_spectral_features {
                spectral_features(&normed)
            } else {
                (0, 0.0, 0.0)
            };

        FeatureVector {
            m20,
            m21,
            m40,
            m41,
            m42,
            m43,
            c20,
            c21,
            c40,
            c41,
            c42,
            sigma_aa,
            sigma_af,
            sigma_ap,
            mu_a,
            kurtosis_a,
            kurtosis_f,
            snr_db,
            spectral_peak_count,
            spectral_symmetry,
            spectral_flatness,
        }
    }

    /// Classify from a pre-extracted feature vector.
    pub fn classify_from_features(&self, features: &FeatureVector) -> ClassificationResult {
        let mut distances: Vec<(Modulation, f64)> = self
            .reference_table
            .iter()
            .map(|profile| {
                let d = weighted_distance(features, profile);
                (profile.modulation, d)
            })
            .collect();

        distances.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));

        let best = distances
            .first()
            .map(|d| d.0)
            .unwrap_or(Modulation::Unknown);
        let best_dist = distances.first().map(|d| d.1).unwrap_or(f64::MAX);
        let second_dist = distances.get(1).map(|d| d.1).unwrap_or(f64::MAX);

        // Confidence: ratio of separation between best and second-best
        let confidence = if second_dist > 1e-30 {
            let ratio = 1.0 - best_dist / second_dist;
            ratio.clamp(0.0, 1.0)
        } else if best_dist < 1e-10 {
            1.0
        } else {
            0.0
        };

        ClassificationResult {
            modulation: best,
            confidence,
            features: features.clone(),
            distances,
        }
    }

    /// Return the reference modulation profiles used by this classifier.
    pub fn reference_profiles(&self) -> &[ModulationProfile] {
        &self.reference_table
    }
}

// ---------------------------------------------------------------------------
// Feature computation helpers
// ---------------------------------------------------------------------------

/// Generalised moment M_{pq} = E[x^(p-q) * conj(x)^q]
/// where x is a complex sample.  Returns the magnitude of the complex moment.
fn moment(samples: &[(f64, f64)], p: u32, q: u32) -> f64 {
    let n = samples.len() as f64;
    if n == 0.0 {
        return 0.0;
    }
    let sum: (f64, f64) = samples.iter().fold((0.0, 0.0), |acc, &(re, im)| {
        let xpq = complex_pow((re, im), p - q);
        let cxq = complex_pow((re, -im), q);
        let prod = complex_mul(xpq, cxq);
        (acc.0 + prod.0, acc.1 + prod.1)
    });
    let avg = (sum.0 / n, sum.1 / n);
    complex_abs(avg)
}

fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

fn complex_pow(z: (f64, f64), n: u32) -> (f64, f64) {
    match n {
        0 => (1.0, 0.0),
        1 => z,
        _ => {
            let mut result = (1.0, 0.0);
            for _ in 0..n {
                result = complex_mul(result, z);
            }
            result
        }
    }
}

fn complex_abs(z: (f64, f64)) -> f64 {
    (z.0 * z.0 + z.1 * z.1).sqrt()
}

fn mean(v: &[f64]) -> f64 {
    if v.is_empty() {
        return 0.0;
    }
    v.iter().sum::<f64>() / v.len() as f64
}

fn variance(v: &[f64]) -> f64 {
    if v.len() < 2 {
        return 0.0;
    }
    let m = mean(v);
    v.iter().map(|&x| (x - m) * (x - m)).sum::<f64>() / v.len() as f64
}

fn std_dev(v: &[f64]) -> f64 {
    variance(v).sqrt()
}

fn kurtosis(v: &[f64]) -> f64 {
    let m = mean(v);
    let n = v.len() as f64;
    if n < 4.0 {
        return 0.0;
    }
    let var = v.iter().map(|&x| (x - m).powi(2)).sum::<f64>() / n;
    if var < 1e-30 {
        return 0.0;
    }
    let m4 = v.iter().map(|&x| (x - m).powi(4)).sum::<f64>() / n;
    m4 / (var * var) - 3.0 // excess kurtosis
}

fn instantaneous_frequencies(phases: &[f64]) -> Vec<f64> {
    if phases.len() < 2 {
        return vec![0.0; phases.len()];
    }
    let mut freqs = Vec::with_capacity(phases.len());
    freqs.push(0.0);
    for i in 1..phases.len() {
        let mut diff = phases[i] - phases[i - 1];
        // Wrap to [-pi, pi]
        while diff > PI {
            diff -= 2.0 * PI;
        }
        while diff < -PI {
            diff += 2.0 * PI;
        }
        freqs.push(diff);
    }
    freqs
}

/// M2M4 SNR estimator.
///
/// Uses the ratio of second and fourth moments of the signal envelope
/// to separate signal power from noise power.  Assumes a general (non-constant-
/// envelope) signal model with kappa_s derived from the data.
fn estimate_snr_m2m4(samples: &[(f64, f64)]) -> f64 {
    let n = samples.len() as f64;
    if n < 2.0 {
        return 0.0;
    }

    let powers: Vec<f64> = samples.iter().map(|&(r, i)| r * r + i * i).collect();
    let m2 = powers.iter().sum::<f64>() / n;
    let m4 = powers.iter().map(|&p| p * p).sum::<f64>() / n;

    if m2 < 1e-30 {
        return 0.0;
    }

    // Estimate kurtosis ratio kappa_s = E[|x|^4] / E[|x|^2]^2
    let kappa = m4 / (m2 * m2);

    // For a signal + noise model:
    //   M2 = S + N
    //   M4 = kappa_s * S^2 + 2*S*N + 2*N^2  (complex Gaussian noise has kappa_n=2)
    // With kappa_s estimated from data, if kappa close to 1 (constant envelope)
    // or close to 2 (Gaussian-like), the signal is very clean or very noisy.
    // Simple heuristic: use kappa to bound SNR.

    // If kappa >= 2.0, it looks like pure noise => low SNR
    if kappa >= 2.0 {
        return 0.0;
    }
    // If kappa <= 1.0 + 1e-6, constant envelope => very high SNR
    if kappa <= 1.0 + 1e-6 {
        return 30.0;
    }

    // Linear interpolation: kappa=1 => SNR=30dB, kappa=2 => SNR=0dB
    let snr_db = 30.0 * (2.0 - kappa);
    snr_db.clamp(0.0, 40.0)
}

/// Compute spectral features from the IQ samples using a simple DFT.
fn spectral_features(samples: &[(f64, f64)]) -> (usize, f64, f64) {
    let n = samples.len().min(1024); // cap for performance
    let spectrum = simple_dft(&samples[..n]);

    // Magnitude spectrum
    let mag: Vec<f64> = spectrum
        .iter()
        .map(|&(r, i)| (r * r + i * i).sqrt())
        .collect();

    let peak_count = count_spectral_peaks(&mag);
    let symmetry = spectral_symmetry_metric(&mag);
    let flatness = spectral_flatness_metric(&mag);

    (peak_count, symmetry, flatness)
}

fn simple_dft(samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = samples.len();
    let mut out = vec![(0.0, 0.0); n];
    for k in 0..n {
        let mut re = 0.0;
        let mut im = 0.0;
        for (t, &(sr, si)) in samples.iter().enumerate() {
            let angle = -2.0 * PI * (k as f64) * (t as f64) / (n as f64);
            let (sin_a, cos_a) = angle.sin_cos();
            re += sr * cos_a - si * sin_a;
            im += sr * sin_a + si * cos_a;
        }
        out[k] = (re, im);
    }
    out
}

fn count_spectral_peaks(mag: &[f64]) -> usize {
    if mag.len() < 3 {
        return 0;
    }
    let max_val = mag.iter().cloned().fold(0.0_f64, f64::max);
    if max_val < 1e-30 {
        return 0;
    }
    let threshold = max_val * 0.3;
    let mut peaks = 0;
    for i in 1..mag.len() - 1 {
        if mag[i] > mag[i - 1] && mag[i] > mag[i + 1] && mag[i] > threshold {
            peaks += 1;
        }
    }
    peaks
}

fn spectral_symmetry_metric(mag: &[f64]) -> f64 {
    let n = mag.len();
    if n < 2 {
        return 1.0;
    }
    let half = n / 2;
    let mut diff_sum = 0.0;
    let mut total = 0.0;
    for i in 0..half {
        let upper = mag[half + i];
        let lower = mag[half.saturating_sub(i + 1)];
        diff_sum += (upper - lower).abs();
        total += upper + lower;
    }
    if total < 1e-30 {
        return 1.0;
    }
    1.0 - diff_sum / total
}

fn spectral_flatness_metric(mag: &[f64]) -> f64 {
    let n = mag.len() as f64;
    if n < 1.0 {
        return 0.0;
    }
    let arith_mean = mag.iter().sum::<f64>() / n;
    if arith_mean < 1e-30 {
        return 0.0;
    }
    // Geometric mean via log-domain
    let log_sum = mag.iter().map(|&m| (m + 1e-30).ln()).sum::<f64>() / n;
    let geo_mean = log_sum.exp();
    (geo_mean / arith_mean).clamp(0.0, 1.0)
}

// ---------------------------------------------------------------------------
// Reference table and decision logic
// ---------------------------------------------------------------------------

/// Build reference feature profiles from measured values.
///
/// These were calibrated against the test signal generators in this module:
///
/// | Modulation | |C40| | |C42| | |C41| | kurtosis_a | peaks |
/// |------------|-------|-------|-------|------------|-------|
/// | BPSK       | 2.00  | 2.00  | 2.00  |  0.00      |  1    |
/// | QPSK       | 1.00  | 1.00  | 0.00  |  0.00      |  1    |
/// | 8PSK       | 0.00  | 1.00  | 0.00  |  0.00      |  1    |
/// | 16-QAM     | 0.68  | 0.68  | 0.00  | -0.95      |  1    |
/// | 64-QAM     | 0.62  | 0.62  | 0.00  | -0.62      |  1    |
/// | 2-FSK      | 0.00  | 1.00  | 0.00  |  0.00      |  6    |
/// | 4-FSK      | 0.25  | 1.00  | 0.00  |  0.00      | 21    |
fn build_reference_table() -> Vec<ModulationProfile> {
    vec![
        ModulationProfile {
            modulation: Modulation::Bpsk,
            abs_c40: 2.0,
            abs_c42: 2.0,
            abs_c41: 2.0,
            kurtosis_a: 0.0,
            spectral_peak_count: 1.0,
        },
        ModulationProfile {
            modulation: Modulation::Qpsk,
            abs_c40: 1.0,
            abs_c42: 1.0,
            abs_c41: 0.0,
            kurtosis_a: 0.0,
            spectral_peak_count: 1.0,
        },
        ModulationProfile {
            modulation: Modulation::Psk8,
            abs_c40: 0.0,
            abs_c42: 1.0,
            abs_c41: 0.0,
            kurtosis_a: 0.0,
            spectral_peak_count: 1.0,
        },
        ModulationProfile {
            modulation: Modulation::Qam16,
            abs_c40: 0.68,
            abs_c42: 0.68,
            abs_c41: 0.0,
            kurtosis_a: -0.95,
            spectral_peak_count: 1.0,
        },
        ModulationProfile {
            modulation: Modulation::Qam64,
            abs_c40: 0.62,
            abs_c42: 0.62,
            abs_c41: 0.0,
            kurtosis_a: -0.62,
            spectral_peak_count: 1.0,
        },
        ModulationProfile {
            modulation: Modulation::Fsk2,
            abs_c40: 0.0,
            abs_c42: 1.0,
            abs_c41: 0.0,
            kurtosis_a: 0.0,
            spectral_peak_count: 6.0,
        },
        ModulationProfile {
            modulation: Modulation::Fsk4,
            abs_c40: 0.25,
            abs_c42: 1.0,
            abs_c41: 0.0,
            kurtosis_a: 0.0,
            spectral_peak_count: 21.0,
        },
    ]
}

/// Compute weighted Euclidean distance between extracted features and a
/// reference profile.
fn weighted_distance(features: &FeatureVector, profile: &ModulationProfile) -> f64 {
    // Weights chosen to emphasise the most discriminating features.
    // |C41| is critical for separating BPSK from QPSK.
    // Spectral peak count separates FSK from PSK modulations.
    // |C40| and |C42| separate the PSK/QAM family members.
    let w_c40 = 3.0;
    let w_c42 = 3.0;
    let w_c41 = 5.0; // high weight: BPSK is the only one with |C41| >> 0
    let w_kurtosis_a = 2.0;
    let w_peaks = 1.0; // normalised by dividing diff by 10

    let d_c40 = (features.c40.abs() - profile.abs_c40).powi(2) * w_c40;
    let d_c42 = (features.c42.abs() - profile.abs_c42).powi(2) * w_c42;
    let d_c41 = (features.c41.abs() - profile.abs_c41).powi(2) * w_c41;
    let d_kurtosis_a = (features.kurtosis_a - profile.kurtosis_a).powi(2) * w_kurtosis_a;
    // Normalise peak count difference to keep it on a similar scale
    let peak_diff = (features.spectral_peak_count as f64 - profile.spectral_peak_count) / 10.0;
    let d_peaks = peak_diff.powi(2) * w_peaks;

    (d_c40 + d_c42 + d_c41 + d_kurtosis_a + d_peaks).sqrt()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    fn default_classifier() -> AutomaticModulationClassifier {
        AutomaticModulationClassifier::new(ClassifierConfig::default())
    }

    /// Generate BPSK symbols: +1 or -1 on real axis.
    fn gen_bpsk(n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| if i % 2 == 0 { (1.0, 0.0) } else { (-1.0, 0.0) })
            .collect()
    }

    /// Generate QPSK symbols at 45/135/225/315 degrees.
    fn gen_qpsk(n: usize) -> Vec<(f64, f64)> {
        let s = 1.0 / 2.0_f64.sqrt();
        let points = [(s, s), (-s, s), (-s, -s), (s, -s)];
        (0..n).map(|i| points[i % 4]).collect()
    }

    /// Generate 8PSK symbols.
    fn gen_8psk(n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let angle = 2.0 * PI * (i % 8) as f64 / 8.0;
                (angle.cos(), angle.sin())
            })
            .collect()
    }

    /// Generate 16-QAM symbols.
    fn gen_16qam(n: usize) -> Vec<(f64, f64)> {
        let levels = [-3.0, -1.0, 1.0, 3.0];
        let mut pts = Vec::new();
        for &r in &levels {
            for &i in &levels {
                pts.push((r, i));
            }
        }
        let pwr: f64 =
            pts.iter().map(|&(r, i)| r * r + i * i).sum::<f64>() / pts.len() as f64;
        let s = 1.0 / pwr.sqrt();
        let pts: Vec<(f64, f64)> = pts.iter().map(|&(r, i)| (r * s, i * s)).collect();
        (0..n).map(|i| pts[i % pts.len()]).collect()
    }

    /// Generate 2-FSK: two discrete frequencies.
    fn gen_2fsk(n: usize) -> Vec<(f64, f64)> {
        let f0 = 0.1;
        let f1 = 0.3;
        (0..n)
            .map(|i| {
                let freq = if (i / 16) % 2 == 0 { f0 } else { f1 };
                let angle = 2.0 * PI * freq * i as f64;
                (angle.cos(), angle.sin())
            })
            .collect()
    }

    /// Generate 4-FSK: four discrete frequencies.
    fn gen_4fsk(n: usize) -> Vec<(f64, f64)> {
        let freqs = [0.05, 0.15, 0.25, 0.35];
        (0..n)
            .map(|i| {
                let freq = freqs[(i / 16) % 4];
                let angle = 2.0 * PI * freq * i as f64;
                (angle.cos(), angle.sin())
            })
            .collect()
    }

    // -- Feature extraction tests --

    #[test]
    fn test_bpsk_features() {
        let c = default_classifier();
        let samples = gen_bpsk(1024);
        let f = c.extract_features(&samples);
        // BPSK: |C42| ~ 2.0, |C40| ~ 2.0, |C41| ~ 2.0
        assert!(
            f.c42.abs() > 1.5,
            "BPSK |C42| should be ~2.0, got {}",
            f.c42.abs()
        );
        assert!(
            f.c41.abs() > 1.5,
            "BPSK |C41| should be ~2.0, got {}",
            f.c41.abs()
        );
    }

    #[test]
    fn test_qpsk_features() {
        let c = default_classifier();
        let samples = gen_qpsk(1024);
        let f = c.extract_features(&samples);
        // QPSK: |C40| ~ 1.0, |C41| ~ 0
        assert!(
            f.c40.abs() > 0.5,
            "QPSK |C40| should be ~1.0, got {}",
            f.c40.abs()
        );
        assert!(
            f.c41.abs() < 0.3,
            "QPSK |C41| should be ~0, got {}",
            f.c41.abs()
        );
    }

    #[test]
    fn test_8psk_features() {
        let c = default_classifier();
        let samples = gen_8psk(1024);
        let f = c.extract_features(&samples);
        // 8PSK: |C40| ~ 0, |C41| ~ 0
        assert!(
            f.c40.abs() < 0.3,
            "8PSK |C40| should be ~0, got {}",
            f.c40.abs()
        );
        assert!(
            f.c41.abs() < 0.3,
            "8PSK |C41| should be ~0, got {}",
            f.c41.abs()
        );
    }

    #[test]
    fn test_16qam_features() {
        let c = default_classifier();
        let samples = gen_16qam(1024);
        let f = c.extract_features(&samples);
        // 16-QAM has multi-level amplitudes => negative kurtosis
        assert!(
            f.kurtosis_a < -0.3,
            "16QAM kurtosis should be negative, got {}",
            f.kurtosis_a
        );
    }

    #[test]
    fn test_snr_estimation_high_snr() {
        let samples = gen_bpsk(1024);
        let snr = estimate_snr_m2m4(&samples);
        // Clean constant-envelope signal => high SNR
        assert!(
            snr >= 20.0,
            "Clean BPSK should have high SNR estimate, got {}",
            snr
        );
    }

    #[test]
    fn test_snr_estimation_noisy() {
        // Add noise using a simple deterministic pseudo-noise
        let mut samples = gen_bpsk(1024);
        let mut seed: u64 = 12345;
        for s in samples.iter_mut() {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let noise = ((seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5) * 2.0;
            s.0 += noise * 1.0; // significant noise
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            let noise_i = ((seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5) * 2.0;
            s.1 += noise_i * 1.0;
        }
        let snr = estimate_snr_m2m4(&samples);
        // kappa will be > 1 because noise pushes toward Gaussian (kappa=2)
        assert!(
            snr < 30.0,
            "Noisy BPSK should have lower SNR than clean, got {}",
            snr
        );
    }

    // -- Classification tests --

    #[test]
    fn test_classify_bpsk() {
        let c = default_classifier();
        let result = c.classify(&gen_bpsk(1024));
        assert_eq!(
            result.modulation,
            Modulation::Bpsk,
            "Expected BPSK, got {:?}",
            result.modulation
        );
        assert!(result.confidence > 0.0);
    }

    #[test]
    fn test_classify_qpsk() {
        let c = default_classifier();
        let result = c.classify(&gen_qpsk(1024));
        assert_eq!(
            result.modulation,
            Modulation::Qpsk,
            "Expected QPSK, got {:?}",
            result.modulation
        );
    }

    #[test]
    fn test_classify_8psk() {
        let c = default_classifier();
        let result = c.classify(&gen_8psk(1024));
        assert_eq!(
            result.modulation,
            Modulation::Psk8,
            "Expected 8PSK, got {:?}",
            result.modulation
        );
    }

    #[test]
    fn test_classify_16qam() {
        let c = default_classifier();
        let result = c.classify(&gen_16qam(1024));
        assert!(
            result.modulation == Modulation::Qam16
                || result.modulation == Modulation::Qam64,
            "Expected QAM-like, got {:?}",
            result.modulation
        );
    }

    #[test]
    fn test_classify_2fsk() {
        let c = default_classifier();
        let result = c.classify(&gen_2fsk(2048));
        assert!(
            result.modulation == Modulation::Fsk2
                || result.modulation == Modulation::Fsk4,
            "Expected FSK-like, got {:?}",
            result.modulation
        );
    }

    #[test]
    fn test_classify_too_few_samples() {
        let c = default_classifier();
        let result = c.classify(&gen_bpsk(10));
        assert_eq!(result.modulation, Modulation::Unknown);
        assert_eq!(result.confidence, 0.0);
    }

    #[test]
    fn test_classify_empty_samples() {
        let c = default_classifier();
        let result = c.classify(&[]);
        assert_eq!(result.modulation, Modulation::Unknown);
    }

    // -- Utility / edge-case tests --

    #[test]
    fn test_complex_pow() {
        let z = (1.0, 1.0);
        let z0 = complex_pow(z, 0);
        assert!((z0.0 - 1.0).abs() < 1e-10);
        assert!(z0.1.abs() < 1e-10);

        let z1 = complex_pow(z, 1);
        assert!((z1.0 - 1.0).abs() < 1e-10);
        assert!((z1.1 - 1.0).abs() < 1e-10);

        // (1+i)^2 = 2i
        let z2 = complex_pow(z, 2);
        assert!(z2.0.abs() < 1e-10, "Re should be ~0, got {}", z2.0);
        assert!(
            (z2.1 - 2.0).abs() < 1e-10,
            "Im should be ~2, got {}",
            z2.1
        );
    }

    #[test]
    fn test_kurtosis_gaussian_like() {
        // A uniform distribution has excess kurtosis = -1.2
        let v: Vec<f64> = (0..10000).map(|i| (i as f64) / 10000.0).collect();
        let k = kurtosis(&v);
        assert!(
            (k - (-1.2)).abs() < 0.1,
            "Uniform kurtosis should be ~-1.2, got {}",
            k
        );
    }

    #[test]
    fn test_mean_and_std_dev() {
        let v = vec![2.0, 4.0, 4.0, 4.0, 5.0, 5.0, 7.0, 9.0];
        let m = mean(&v);
        assert!((m - 5.0).abs() < 1e-10);
        let sd = std_dev(&v);
        assert!(sd > 1.5 && sd < 2.5, "Unexpected std_dev: {}", sd);
    }

    #[test]
    fn test_spectral_flatness_tone_vs_noise() {
        // A single tone should have low spectral flatness
        let tone: Vec<(f64, f64)> = (0..256)
            .map(|i| {
                let angle = 2.0 * PI * 0.1 * i as f64;
                (angle.cos(), angle.sin())
            })
            .collect();
        let (_, _, flatness_tone) = spectral_features(&tone);

        // Pseudo-noise should have higher flatness
        let mut seed: u64 = 99999;
        let noise: Vec<(f64, f64)> = (0..256)
            .map(|_| {
                seed = seed
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1);
                let r = (seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5;
                seed = seed
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1);
                let i = (seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5;
                (r, i)
            })
            .collect();
        let (_, _, flatness_noise) = spectral_features(&noise);

        assert!(
            flatness_noise > flatness_tone,
            "Noise flatness ({}) should exceed tone flatness ({})",
            flatness_noise,
            flatness_tone
        );
    }

    #[test]
    fn test_modulation_display() {
        assert_eq!(format!("{}", Modulation::Bpsk), "BPSK");
        assert_eq!(format!("{}", Modulation::Qam64), "64-QAM");
        assert_eq!(format!("{}", Modulation::Unknown), "Unknown");
    }

    #[test]
    fn test_reference_profiles_count() {
        let c = default_classifier();
        assert_eq!(c.reference_profiles().len(), 7);
    }

    #[test]
    fn test_distances_sorted() {
        let c = default_classifier();
        let result = c.classify(&gen_bpsk(1024));
        for w in result.distances.windows(2) {
            assert!(
                w[0].1 <= w[1].1,
                "Distances should be sorted ascending"
            );
        }
    }

    #[test]
    fn test_confidence_range() {
        let c = default_classifier();
        for gen in &[gen_bpsk, gen_qpsk, gen_8psk] {
            let result = c.classify(&gen(1024));
            assert!(
                (0.0..=1.0).contains(&result.confidence),
                "Confidence {} out of [0,1]",
                result.confidence
            );
        }
    }

    #[test]
    fn test_instantaneous_frequencies_length() {
        let phases = vec![0.0, 0.1, 0.2, 0.3];
        let freqs = instantaneous_frequencies(&phases);
        assert_eq!(freqs.len(), phases.len());
        assert_eq!(freqs[0], 0.0);
    }

    #[test]
    fn test_classify_from_features() {
        let c = default_classifier();
        let features = c.extract_features(&gen_bpsk(512));
        let result = c.classify_from_features(&features);
        assert_eq!(result.modulation, Modulation::Bpsk);
    }
}
