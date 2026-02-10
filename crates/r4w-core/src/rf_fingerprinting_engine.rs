//! RF Fingerprinting Engine for Device Identification
//!
//! Extracts hardware impairment signatures from RF transmissions for device
//! identification and authentication. Each physical transmitter exhibits unique
//! analog front-end imperfections — IQ imbalance, carrier frequency offset drift,
//! power amplifier nonlinearity, and turn-on transient shape — that form a
//! repeatable "fingerprint" for that device.
//!
//! The engine provides:
//! - **Feature extraction**: IQ imbalance, CFO stability, transient shape, spectral signature
//! - **Fingerprint database**: Store and retrieve known device fingerprints
//! - **Device matching**: Euclidean and Mahalanobis distance metrics with confidence scoring
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::rf_fingerprinting_engine::{
//!     RfFingerprintEngine, FingerprintDatabase, DistanceMetric,
//! };
//!
//! // Generate a synthetic IQ burst (in practice, captured from an SDR)
//! let burst: Vec<(f64, f64)> = (0..1024)
//!     .map(|i| {
//!         let t = i as f64 / 1024.0;
//!         let amplitude = if i < 50 { i as f64 / 50.0 } else { 1.0 };
//!         (amplitude * (2.0 * std::f64::consts::PI * 10.0 * t).cos(),
//!          amplitude * (2.0 * std::f64::consts::PI * 10.0 * t).sin())
//!     })
//!     .collect();
//!
//! // Extract fingerprint
//! let engine = RfFingerprintEngine::new(1024.0);
//! let fp = engine.extract_fingerprint(&burst);
//! assert!(fp.iq_imbalance.0.is_finite());
//!
//! // Build a database and match
//! let mut db = FingerprintDatabase::new(DistanceMetric::Euclidean);
//! db.add_device("tx-alpha", fp.clone());
//! let result = db.match_device(&fp);
//! assert_eq!(result.device_id, "tx-alpha");
//! assert!(result.confidence > 0.9);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex helper functions (using (f64, f64) tuples)
// ---------------------------------------------------------------------------

#[inline]
fn cx_mag_sq(c: (f64, f64)) -> f64 {
    c.0 * c.0 + c.1 * c.1
}

#[inline]
fn cx_mag(c: (f64, f64)) -> f64 {
    cx_mag_sq(c).sqrt()
}

#[inline]
fn cx_conj(c: (f64, f64)) -> (f64, f64) {
    (c.0, -c.1)
}

#[inline]
fn cx_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// IQ imbalance measurement: (amplitude_imbalance_dB, phase_imbalance_rad).
///
/// Amplitude imbalance is the ratio of Q-arm gain to I-arm gain in dB.
/// Phase imbalance is the deviation from π/2 quadrature in radians.
pub type IqImbalance = (f64, f64);

/// Features extracted from the power-on (or power-off) transient portion.
#[derive(Debug, Clone, PartialEq)]
pub struct TransientFeatures {
    /// Duration of the transient in samples.
    pub duration_samples: usize,
    /// RMS energy of the transient segment.
    pub energy_rms: f64,
    /// Peak-to-average power ratio (dB) of the transient.
    pub papr_db: f64,
    /// Slope of the envelope rise (linear units per sample).
    pub rise_slope: f64,
}

/// Spectral shape features derived from the power spectral density.
#[derive(Debug, Clone, PartialEq)]
pub struct SpectralFeatures {
    /// Spectral centroid (normalized frequency, 0..1).
    pub centroid: f64,
    /// 3-dB bandwidth (normalized frequency).
    pub bandwidth: f64,
    /// Spectral skewness (third central moment of the PSD).
    pub skewness: f64,
}

/// Complete device fingerprint extracted from an RF burst.
#[derive(Debug, Clone, PartialEq)]
pub struct DeviceFingerprint {
    /// IQ imbalance: (amplitude_dB, phase_rad).
    pub iq_imbalance: IqImbalance,
    /// Standard deviation of instantaneous carrier frequency offset (Hz).
    pub freq_offset_std: f64,
    /// Turn-on transient shape features.
    pub transient_shape: TransientFeatures,
    /// Power spectral density shape features.
    pub spectral_features: SpectralFeatures,
}

/// Result returned by [`FingerprintDatabase::match_device`].
#[derive(Debug, Clone)]
pub struct MatchResult {
    /// Identifier of the best-matching device.
    pub device_id: String,
    /// Distance from the query to the matched fingerprint.
    pub distance: f64,
    /// Confidence score in [0, 1]. Higher is better.
    pub confidence: f64,
}

/// Distance metric used for fingerprint comparison.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DistanceMetric {
    /// Standard Euclidean distance across all feature dimensions.
    Euclidean,
    /// Mahalanobis distance, which accounts for per-dimension variance.
    /// If fewer than 2 fingerprints are stored, falls back to Euclidean.
    Mahalanobis,
}

// ---------------------------------------------------------------------------
// RfFingerprintEngine
// ---------------------------------------------------------------------------

/// Core engine for extracting RF fingerprints from IQ bursts.
///
/// The `sample_rate` field is used to convert frequency-domain quantities
/// (like CFO) from normalized units to Hz.
#[derive(Debug, Clone)]
pub struct RfFingerprintEngine {
    sample_rate: f64,
    /// Fraction of burst start used for transient detection (0..1).
    transient_fraction: f64,
    /// Threshold (relative to peak envelope) below which samples are
    /// considered part of the turn-on transient.
    transient_threshold: f64,
}

impl RfFingerprintEngine {
    /// Create a new engine with the given sample rate in Hz.
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            transient_fraction: 0.15,
            transient_threshold: 0.7,
        }
    }

    /// Override the transient detection parameters.
    pub fn with_transient_params(mut self, fraction: f64, threshold: f64) -> Self {
        self.transient_fraction = fraction.clamp(0.01, 0.5);
        self.transient_threshold = threshold.clamp(0.1, 0.99);
        self
    }

    /// Extract a complete [`DeviceFingerprint`] from an IQ burst.
    pub fn extract_fingerprint(&self, burst: &[(f64, f64)]) -> DeviceFingerprint {
        let iq_imbalance = self.estimate_iq_imbalance(burst);
        let freq_offset_std = self.estimate_cfo_stability(burst);
        let transient_shape = self.extract_transient(burst);
        let spectral_features = self.spectral_signature(burst);
        DeviceFingerprint {
            iq_imbalance,
            freq_offset_std,
            transient_shape,
            spectral_features,
        }
    }

    /// Estimate IQ amplitude and phase imbalance from the burst.
    ///
    /// Uses the second-order statistics method: computes the correlation
    /// between the signal and its conjugate to estimate the image component.
    /// Returns `(amplitude_imbalance_dB, phase_imbalance_rad)`.
    pub fn estimate_iq_imbalance(&self, burst: &[(f64, f64)]) -> IqImbalance {
        if burst.len() < 2 {
            return (0.0, 0.0);
        }

        // Compute E[z^2] and E[|z|^2] (second-order statistics).
        let n = burst.len() as f64;
        let mut sum_z2_re = 0.0;
        let mut sum_z2_im = 0.0;
        let mut sum_mag2 = 0.0;

        for &s in burst {
            let z2 = cx_mul(s, s);
            sum_z2_re += z2.0;
            sum_z2_im += z2.1;
            sum_mag2 += cx_mag_sq(s);
        }

        let cz2_re = sum_z2_re / n;
        let cz2_im = sum_z2_im / n;
        let rz = sum_mag2 / n;

        if rz < 1e-30 {
            return (0.0, 0.0);
        }

        // Image rejection ratio: |E[z^2]| / E[|z|^2]
        let conj_ratio = (cz2_re * cz2_re + cz2_im * cz2_im).sqrt() / rz;

        // Amplitude imbalance in dB (small-signal approximation: IRR ≈ (g-1)/(g+1))
        let g = (1.0 + conj_ratio) / (1.0 - conj_ratio + 1e-30);
        let amp_imbalance_db = 20.0 * g.abs().log10();

        // Phase imbalance from angle of E[z^2]
        let phase_imbalance = cz2_im.atan2(cz2_re) / 2.0;

        (amp_imbalance_db, phase_imbalance)
    }

    /// Estimate carrier frequency offset stability (standard deviation in Hz).
    ///
    /// Computes the instantaneous frequency from sample-to-sample phase
    /// differences, then returns the standard deviation of those estimates.
    pub fn estimate_cfo_stability(&self, burst: &[(f64, f64)]) -> f64 {
        if burst.len() < 3 {
            return 0.0;
        }

        // Instantaneous frequency via conjugate product: z[n] * conj(z[n-1])
        let mut inst_freqs: Vec<f64> = Vec::with_capacity(burst.len() - 1);
        for i in 1..burst.len() {
            let prod = cx_mul(burst[i], cx_conj(burst[i - 1]));
            let phase_diff = prod.1.atan2(prod.0);
            // Convert from radians/sample to Hz
            let freq_hz = phase_diff / (2.0 * PI) * self.sample_rate;
            inst_freqs.push(freq_hz);
        }

        // Standard deviation
        let n = inst_freqs.len() as f64;
        let mean = inst_freqs.iter().sum::<f64>() / n;
        let variance = inst_freqs.iter().map(|f| (f - mean).powi(2)).sum::<f64>() / n;
        variance.sqrt()
    }

    /// Extract the turn-on transient from the burst and compute shape features.
    ///
    /// The transient is identified as the initial portion of the burst where
    /// the envelope is below [`Self::transient_threshold`] times the steady-state
    /// peak envelope.
    pub fn extract_transient(&self, burst: &[(f64, f64)]) -> TransientFeatures {
        if burst.is_empty() {
            return TransientFeatures {
                duration_samples: 0,
                energy_rms: 0.0,
                papr_db: 0.0,
                rise_slope: 0.0,
            };
        }

        let search_len = ((burst.len() as f64 * self.transient_fraction).ceil() as usize)
            .max(1)
            .min(burst.len());

        // Compute envelope
        let envelope: Vec<f64> = burst.iter().map(|&s| cx_mag(s)).collect();

        // Peak envelope in the steady-state portion (after the search window)
        let ss_peak = envelope[search_len..]
            .iter()
            .copied()
            .fold(0.0_f64, f64::max)
            .max(envelope.iter().copied().fold(0.0_f64, f64::max) * 0.5);

        let threshold = ss_peak * self.transient_threshold;

        // Find where envelope first exceeds threshold
        let transient_end = envelope[..search_len]
            .iter()
            .position(|&e| e >= threshold)
            .unwrap_or(search_len)
            .max(1);

        let transient_samples = &burst[..transient_end];
        let transient_env = &envelope[..transient_end];

        // RMS energy
        let energy_rms = if transient_samples.is_empty() {
            0.0
        } else {
            let sum_sq: f64 = transient_samples.iter().map(|&s| cx_mag_sq(s)).sum();
            (sum_sq / transient_samples.len() as f64).sqrt()
        };

        // PAPR
        let peak_power = transient_env
            .iter()
            .copied()
            .fold(0.0_f64, f64::max)
            .powi(2);
        let avg_power = if transient_samples.is_empty() {
            1e-30
        } else {
            transient_samples
                .iter()
                .map(|&s| cx_mag_sq(s))
                .sum::<f64>()
                / transient_samples.len() as f64
        };
        let papr_db = 10.0 * (peak_power / avg_power.max(1e-30)).log10();

        // Rise slope: linear fit of envelope over the transient
        let rise_slope = if transient_end > 1 {
            let last = transient_env[transient_end - 1];
            let first = transient_env[0];
            (last - first) / (transient_end - 1) as f64
        } else {
            0.0
        };

        TransientFeatures {
            duration_samples: transient_end,
            energy_rms,
            papr_db,
            rise_slope,
        }
    }

    /// Compute spectral shape features: centroid, bandwidth, skewness.
    ///
    /// Uses a simple DFT (no external FFT crate) for moderate burst lengths.
    pub fn spectral_signature(&self, burst: &[(f64, f64)]) -> SpectralFeatures {
        if burst.is_empty() {
            return SpectralFeatures {
                centroid: 0.0,
                bandwidth: 0.0,
                skewness: 0.0,
            };
        }

        let n = burst.len();
        // Compute PSD via DFT magnitude squared (only positive frequencies)
        let num_bins = n / 2 + 1;
        let mut psd = vec![0.0_f64; num_bins];

        for k in 0..num_bins {
            let mut re = 0.0;
            let mut im = 0.0;
            for (i, &(sr, si)) in burst.iter().enumerate() {
                let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
                let (sin_a, cos_a) = angle.sin_cos();
                re += sr * cos_a - si * sin_a;
                im += sr * sin_a + si * cos_a;
            }
            psd[k] = re * re + im * im;
        }

        // Normalize PSD to a probability distribution
        let total: f64 = psd.iter().sum();
        if total < 1e-30 {
            return SpectralFeatures {
                centroid: 0.0,
                bandwidth: 0.0,
                skewness: 0.0,
            };
        }
        let psd_norm: Vec<f64> = psd.iter().map(|&p| p / total).collect();

        // Normalized frequency axis: 0..0.5 (Nyquist)
        let freq = |k: usize| -> f64 { k as f64 / n as f64 };

        // Centroid: sum(f * P(f))
        let centroid: f64 = (0..num_bins).map(|k| freq(k) * psd_norm[k]).sum();

        // Bandwidth: sqrt(sum((f - centroid)^2 * P(f)))
        let variance: f64 = (0..num_bins)
            .map(|k| {
                let diff = freq(k) - centroid;
                diff * diff * psd_norm[k]
            })
            .sum();
        let bandwidth = variance.sqrt();

        // Skewness: sum((f - centroid)^3 * P(f)) / bandwidth^3
        let skewness = if bandwidth > 1e-15 {
            let third_moment: f64 = (0..num_bins)
                .map(|k| {
                    let diff = freq(k) - centroid;
                    diff * diff * diff * psd_norm[k]
                })
                .sum();
            third_moment / (bandwidth.powi(3))
        } else {
            0.0
        };

        SpectralFeatures {
            centroid,
            bandwidth,
            skewness,
        }
    }
}

// ---------------------------------------------------------------------------
// Fingerprint vector conversion (for distance computation)
// ---------------------------------------------------------------------------

impl DeviceFingerprint {
    /// Convert the fingerprint into a flat feature vector for distance computation.
    fn to_feature_vec(&self) -> Vec<f64> {
        vec![
            self.iq_imbalance.0,
            self.iq_imbalance.1,
            self.freq_offset_std,
            self.transient_shape.duration_samples as f64,
            self.transient_shape.energy_rms,
            self.transient_shape.papr_db,
            self.transient_shape.rise_slope,
            self.spectral_features.centroid,
            self.spectral_features.bandwidth,
            self.spectral_features.skewness,
        ]
    }
}

// ---------------------------------------------------------------------------
// FingerprintDatabase
// ---------------------------------------------------------------------------

/// Database of known device fingerprints with matching capability.
#[derive(Debug, Clone)]
pub struct FingerprintDatabase {
    metric: DistanceMetric,
    devices: Vec<(String, DeviceFingerprint)>,
}

impl FingerprintDatabase {
    /// Create an empty database with the specified distance metric.
    pub fn new(metric: DistanceMetric) -> Self {
        Self {
            metric,
            devices: Vec::new(),
        }
    }

    /// Add a device fingerprint to the database.
    pub fn add_device(&mut self, device_id: &str, fingerprint: DeviceFingerprint) {
        self.devices.push((device_id.to_string(), fingerprint));
    }

    /// Number of devices in the database.
    pub fn len(&self) -> usize {
        self.devices.len()
    }

    /// Whether the database is empty.
    pub fn is_empty(&self) -> bool {
        self.devices.is_empty()
    }

    /// List all device IDs in the database.
    pub fn device_ids(&self) -> Vec<&str> {
        self.devices.iter().map(|(id, _)| id.as_str()).collect()
    }

    /// Match a query fingerprint against the database, returning the best match.
    ///
    /// # Panics
    ///
    /// Panics if the database is empty.
    pub fn match_device(&self, query: &DeviceFingerprint) -> MatchResult {
        assert!(!self.devices.is_empty(), "database is empty");

        let query_vec = query.to_feature_vec();

        // Compute per-dimension variance for Mahalanobis (if needed)
        let inv_variances = if self.metric == DistanceMetric::Mahalanobis && self.devices.len() >= 2
        {
            Some(self.compute_inv_variances())
        } else {
            None
        };

        let mut best_id = &self.devices[0].0;
        let mut best_dist = f64::MAX;

        for (id, fp) in &self.devices {
            let fv = fp.to_feature_vec();
            let dist = match &inv_variances {
                Some(iv) => mahalanobis_distance(&query_vec, &fv, iv),
                None => euclidean_distance(&query_vec, &fv),
            };
            if dist < best_dist {
                best_dist = dist;
                best_id = id;
            }
        }

        // Convert distance to confidence: conf = exp(-dist / scale)
        // Scale chosen so that distance == 0 -> confidence 1.0 and
        // a "typical" mismatch gives a lower score.
        let scale = self.estimate_scale();
        let confidence = (-best_dist / scale).exp().clamp(0.0, 1.0);

        MatchResult {
            device_id: best_id.clone(),
            distance: best_dist,
            confidence,
        }
    }

    /// Compute per-dimension inverse variances from the stored fingerprints.
    fn compute_inv_variances(&self) -> Vec<f64> {
        let vecs: Vec<Vec<f64>> = self.devices.iter().map(|(_, fp)| fp.to_feature_vec()).collect();
        let dim = vecs[0].len();
        let n = vecs.len() as f64;

        let mut inv_var = vec![1.0; dim];
        for d in 0..dim {
            let mean: f64 = vecs.iter().map(|v| v[d]).sum::<f64>() / n;
            let var: f64 = vecs.iter().map(|v| (v[d] - mean).powi(2)).sum::<f64>() / n;
            inv_var[d] = if var > 1e-30 { 1.0 / var } else { 1.0 };
        }
        inv_var
    }

    /// Estimate a reasonable distance scale for confidence mapping.
    fn estimate_scale(&self) -> f64 {
        if self.devices.len() < 2 {
            return 1.0;
        }
        // Use average pairwise distance as scale
        let vecs: Vec<Vec<f64>> = self.devices.iter().map(|(_, fp)| fp.to_feature_vec()).collect();
        let mut total = 0.0;
        let mut count = 0;
        for i in 0..vecs.len() {
            for j in (i + 1)..vecs.len() {
                total += euclidean_distance(&vecs[i], &vecs[j]);
                count += 1;
            }
        }
        if count > 0 {
            (total / count as f64).max(1e-10)
        } else {
            1.0
        }
    }
}

// ---------------------------------------------------------------------------
// Distance functions
// ---------------------------------------------------------------------------

/// Euclidean distance between two feature vectors.
fn euclidean_distance(a: &[f64], b: &[f64]) -> f64 {
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (x - y).powi(2))
        .sum::<f64>()
        .sqrt()
}

/// Mahalanobis distance given pre-computed inverse per-dimension variances.
fn mahalanobis_distance(a: &[f64], b: &[f64], inv_var: &[f64]) -> f64 {
    a.iter()
        .zip(b.iter())
        .zip(inv_var.iter())
        .map(|((x, y), iv)| (x - y).powi(2) * iv)
        .sum::<f64>()
        .sqrt()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a synthetic IQ burst with configurable impairments.
    fn make_burst(
        n: usize,
        freq: f64,
        amp_imbalance: f64,
        phase_error: f64,
        transient_len: usize,
    ) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                let envelope = if i < transient_len {
                    i as f64 / transient_len as f64
                } else {
                    1.0
                };
                let phase = 2.0 * PI * freq * t;
                let i_val = envelope * phase.cos();
                let q_val = envelope * amp_imbalance * (phase + phase_error).sin();
                (i_val, q_val)
            })
            .collect()
    }

    #[test]
    fn test_extract_fingerprint_runs() {
        let burst = make_burst(512, 10.0, 1.0, 0.0, 30);
        let engine = RfFingerprintEngine::new(1000.0);
        let fp = engine.extract_fingerprint(&burst);
        assert!(fp.iq_imbalance.0.is_finite());
        assert!(fp.iq_imbalance.1.is_finite());
        assert!(fp.freq_offset_std.is_finite());
        assert!(fp.spectral_features.centroid.is_finite());
    }

    #[test]
    fn test_iq_imbalance_no_impairment() {
        // Perfectly balanced IQ: expect near-zero imbalance
        let burst: Vec<(f64, f64)> = (0..1024)
            .map(|i| {
                let t = i as f64 / 1024.0;
                let phase = 2.0 * PI * 50.0 * t;
                (phase.cos(), phase.sin())
            })
            .collect();
        let engine = RfFingerprintEngine::new(1024.0);
        let (amp_db, _phase) = engine.estimate_iq_imbalance(&burst);
        // For a pure complex exponential, amplitude imbalance should be near 0 dB
        assert!(amp_db.abs() < 1.0, "amp imbalance {amp_db} dB too large");
    }

    #[test]
    fn test_iq_imbalance_with_gain_error() {
        // Q arm has 2x gain -> expect noticeable imbalance
        let burst: Vec<(f64, f64)> = (0..2048)
            .map(|i| {
                let t = i as f64 / 2048.0;
                let phase = 2.0 * PI * 25.0 * t;
                (phase.cos(), 2.0 * phase.sin())
            })
            .collect();
        let engine = RfFingerprintEngine::new(2048.0);
        let (amp_db, _) = engine.estimate_iq_imbalance(&burst);
        assert!(amp_db > 0.5, "expected measurable imbalance, got {amp_db} dB");
    }

    #[test]
    fn test_cfo_stability_constant_freq() {
        // Single-tone burst: instantaneous frequency should be very stable
        let burst: Vec<(f64, f64)> = (0..512)
            .map(|i| {
                let phase = 2.0 * PI * 100.0 * i as f64 / 1000.0;
                (phase.cos(), phase.sin())
            })
            .collect();
        let engine = RfFingerprintEngine::new(1000.0);
        let std_hz = engine.estimate_cfo_stability(&burst);
        assert!(
            std_hz < 1.0,
            "constant-freq signal should have near-zero CFO std, got {std_hz}"
        );
    }

    #[test]
    fn test_cfo_stability_noisy() {
        // Add phase noise -> higher CFO standard deviation
        let mut burst: Vec<(f64, f64)> = (0..512)
            .map(|i| {
                let phase = 2.0 * PI * 100.0 * i as f64 / 1000.0;
                (phase.cos(), phase.sin())
            })
            .collect();

        // Inject deterministic pseudo-random phase jitter
        for i in 0..burst.len() {
            let jitter = 0.3 * ((i as f64 * 7.3).sin() + (i as f64 * 13.1).cos());
            let (re, im) = burst[i];
            let mag = cx_mag((re, im));
            let phase = im.atan2(re) + jitter;
            burst[i] = (mag * phase.cos(), mag * phase.sin());
        }

        let engine = RfFingerprintEngine::new(1000.0);
        let std_hz = engine.estimate_cfo_stability(&burst);
        assert!(
            std_hz > 1.0,
            "noisy signal should have higher CFO std, got {std_hz}"
        );
    }

    #[test]
    fn test_extract_transient() {
        let burst = make_burst(512, 10.0, 1.0, 0.0, 50);
        let engine = RfFingerprintEngine::new(1000.0);
        let transient = engine.extract_transient(&burst);
        // Transient should be detected in roughly the first 50 samples
        assert!(
            transient.duration_samples > 0,
            "should detect a transient"
        );
        assert!(
            transient.duration_samples <= 80,
            "transient too long: {}",
            transient.duration_samples
        );
        assert!(transient.rise_slope > 0.0, "rise slope should be positive");
    }

    #[test]
    fn test_extract_transient_no_ramp() {
        // Burst with no transient (instant full power)
        let burst: Vec<(f64, f64)> = (0..256)
            .map(|i| {
                let t = i as f64 / 256.0;
                ((2.0 * PI * 10.0 * t).cos(), (2.0 * PI * 10.0 * t).sin())
            })
            .collect();
        let engine = RfFingerprintEngine::new(1000.0);
        let transient = engine.extract_transient(&burst);
        // With an instant-on signal, transient duration should be very short
        assert!(
            transient.duration_samples <= 5,
            "no-ramp signal has transient {} samples",
            transient.duration_samples
        );
    }

    #[test]
    fn test_spectral_signature_single_tone() {
        // Single tone -> spectral energy concentrated at one bin
        let n = 128;
        let burst: Vec<(f64, f64)> = (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                let phase = 2.0 * PI * 10.0 * t;
                (phase.cos(), phase.sin())
            })
            .collect();
        let engine = RfFingerprintEngine::new(n as f64);
        let sf = engine.spectral_signature(&burst);
        // Centroid should be near 10/128 ~ 0.078
        assert!(
            (sf.centroid - 10.0 / n as f64).abs() < 0.02,
            "centroid {} not near expected",
            sf.centroid
        );
        // Bandwidth should be small for a single tone
        assert!(
            sf.bandwidth < 0.05,
            "single tone bandwidth {} too wide",
            sf.bandwidth
        );
    }

    #[test]
    fn test_database_match_exact() {
        let burst = make_burst(512, 10.0, 1.0, 0.0, 30);
        let engine = RfFingerprintEngine::new(1000.0);
        let fp = engine.extract_fingerprint(&burst);

        let mut db = FingerprintDatabase::new(DistanceMetric::Euclidean);
        db.add_device("device-a", fp.clone());

        let result = db.match_device(&fp);
        assert_eq!(result.device_id, "device-a");
        assert!(
            result.distance < 1e-10,
            "exact match should have zero distance"
        );
        assert!(
            result.confidence > 0.99,
            "exact match confidence {} too low",
            result.confidence
        );
    }

    #[test]
    fn test_database_match_closest() {
        let engine = RfFingerprintEngine::new(1000.0);

        let fp_a = engine.extract_fingerprint(&make_burst(512, 10.0, 1.0, 0.0, 30));
        let fp_b = engine.extract_fingerprint(&make_burst(512, 50.0, 1.5, 0.2, 60));
        // Query is similar to device A
        let query = engine.extract_fingerprint(&make_burst(512, 10.0, 1.01, 0.001, 31));

        let mut db = FingerprintDatabase::new(DistanceMetric::Euclidean);
        db.add_device("dev-a", fp_a);
        db.add_device("dev-b", fp_b);

        let result = db.match_device(&query);
        assert_eq!(
            result.device_id, "dev-a",
            "should match dev-a, got {}",
            result.device_id
        );
    }

    #[test]
    fn test_database_mahalanobis() {
        let engine = RfFingerprintEngine::new(1000.0);

        let fp_a = engine.extract_fingerprint(&make_burst(512, 10.0, 1.0, 0.0, 30));
        let fp_b = engine.extract_fingerprint(&make_burst(512, 50.0, 1.5, 0.2, 60));
        let query = engine.extract_fingerprint(&make_burst(512, 10.0, 1.01, 0.001, 31));

        let mut db = FingerprintDatabase::new(DistanceMetric::Mahalanobis);
        db.add_device("dev-a", fp_a);
        db.add_device("dev-b", fp_b);

        let result = db.match_device(&query);
        assert_eq!(result.device_id, "dev-a");
        assert!(result.confidence > 0.0 && result.confidence <= 1.0);
    }

    #[test]
    fn test_empty_burst_handling() {
        let engine = RfFingerprintEngine::new(1000.0);
        let empty: Vec<(f64, f64)> = vec![];

        let iq = engine.estimate_iq_imbalance(&empty);
        assert_eq!(iq, (0.0, 0.0));

        let cfo = engine.estimate_cfo_stability(&empty);
        assert_eq!(cfo, 0.0);

        let trans = engine.extract_transient(&empty);
        assert_eq!(trans.duration_samples, 0);

        let spec = engine.spectral_signature(&empty);
        assert_eq!(spec.centroid, 0.0);
    }

    #[test]
    fn test_database_operations() {
        let mut db = FingerprintDatabase::new(DistanceMetric::Euclidean);
        assert!(db.is_empty());
        assert_eq!(db.len(), 0);

        let engine = RfFingerprintEngine::new(1000.0);
        let fp = engine.extract_fingerprint(&make_burst(256, 10.0, 1.0, 0.0, 20));
        db.add_device("alpha", fp);

        assert!(!db.is_empty());
        assert_eq!(db.len(), 1);
        assert_eq!(db.device_ids(), vec!["alpha"]);
    }
}
