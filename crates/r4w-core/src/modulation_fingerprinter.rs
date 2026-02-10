//! Statistical modulation signature extraction for automatic modulation classification (AMC).
//!
//! This module implements feature-vector-based modulation fingerprinting useful for
//! SIGINT and cognitive radio applications. It extracts higher-order cumulants,
//! instantaneous statistics, spectral features, zero-crossing rates, and constellation
//! shape metrics from raw IQ samples, then classifies against known modulation templates
//! using nearest-neighbour matching.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::modulation_fingerprinter::{ModulationFingerprinter, ModulationType};
//!
//! // Generate BPSK-like samples: symbols at +1 and -1 on the real axis
//! let mut samples: Vec<(f64, f64)> = Vec::new();
//! for i in 0..1024 {
//!     let sym = if (i / 8) % 2 == 0 { 1.0 } else { -1.0 };
//!     samples.push((sym, 0.0));
//! }
//!
//! let fp = ModulationFingerprinter::new();
//! let fingerprint = fp.extract(&samples);
//! let (modulation, confidence) = fp.classify(&fingerprint);
//! assert!(confidence > 0.0);
//! println!("Detected: {:?} (confidence {:.2})", modulation, confidence);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Modulation type labels for template matching.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum ModulationType {
    Bpsk,
    Qpsk,
    Psk8,
    Qam16,
    Qam64,
    Fsk2,
    Fsk4,
    Unknown,
}

/// A modulation fingerprint – a fixed-length feature vector extracted from IQ data.
#[derive(Debug, Clone)]
pub struct ModulationFingerprint {
    // Higher-order cumulants
    pub c20: f64,
    pub c21: f64,
    pub c40: f64,
    pub c41: f64,
    pub c42: f64,
    pub c60: f64,
    pub c61: f64,
    pub c62: f64,
    pub c63: f64,

    // Instantaneous statistics
    pub amplitude_variance: f64,
    pub phase_variance: f64,
    pub frequency_variance: f64,

    // Spectral features
    pub spectral_symmetry: f64,
    pub spectral_flatness: f64,
    pub peak_to_average_ratio: f64,

    // Zero-crossing rates
    pub phase_zero_crossing_rate: f64,
    pub amplitude_zero_crossing_rate: f64,

    // Constellation shape
    pub cluster_count_estimate: f64,
    pub cluster_spread: f64,
}

impl ModulationFingerprint {
    /// Return the feature vector as a fixed-order slice of f64 values.
    pub fn as_vec(&self) -> Vec<f64> {
        vec![
            self.c20,
            self.c21,
            self.c40,
            self.c41,
            self.c42,
            self.c60,
            self.c61,
            self.c62,
            self.c63,
            self.amplitude_variance,
            self.phase_variance,
            self.frequency_variance,
            self.spectral_symmetry,
            self.spectral_flatness,
            self.peak_to_average_ratio,
            self.phase_zero_crossing_rate,
            self.amplitude_zero_crossing_rate,
            self.cluster_count_estimate,
            self.cluster_spread,
        ]
    }

    /// Euclidean distance to another fingerprint.
    pub fn euclidean_distance(&self, other: &Self) -> f64 {
        let a = self.as_vec();
        let b = other.as_vec();
        a.iter()
            .zip(b.iter())
            .map(|(x, y)| (x - y) * (x - y))
            .sum::<f64>()
            .sqrt()
    }

    /// Weighted (Mahalanobis-like) distance using per-feature scale factors.
    /// `weights` must have the same length as the feature vector (19).
    /// Falls back to Euclidean distance if the length mismatches.
    pub fn mahalanobis_like_distance(&self, other: &Self, weights: &[f64]) -> f64 {
        let a = self.as_vec();
        let b = other.as_vec();
        if weights.len() != a.len() {
            return self.euclidean_distance(other);
        }
        a.iter()
            .zip(b.iter())
            .zip(weights.iter())
            .map(|((x, y), w)| w * (x - y) * (x - y))
            .sum::<f64>()
            .sqrt()
    }
}

// ---------------------------------------------------------------------------
// Internal helpers – complex arithmetic on (f64, f64)
// ---------------------------------------------------------------------------

#[inline]
fn c_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

#[inline]
fn c_abs(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

#[inline]
fn c_arg(a: (f64, f64)) -> f64 {
    a.1.atan2(a.0)
}

/// Raise a complex value to an integer power.
fn c_pow(a: (f64, f64), n: u32) -> (f64, f64) {
    let mut result = (1.0, 0.0);
    for _ in 0..n {
        result = c_mul(result, a);
    }
    result
}

/// Mean of complex values.
fn c_mean(s: &[(f64, f64)]) -> (f64, f64) {
    let n = s.len() as f64;
    if n == 0.0 {
        return (0.0, 0.0);
    }
    let sum = s.iter().fold((0.0, 0.0), |acc, &v| (acc.0 + v.0, acc.1 + v.1));
    (sum.0 / n, sum.1 / n)
}

/// Compute E[x^p * conj(x)^q] (mixed moment) over centred, normalised data.
fn moment_pq(data: &[(f64, f64)], p: u32, q: u32) -> (f64, f64) {
    let n = data.len() as f64;
    if n == 0.0 {
        return (0.0, 0.0);
    }
    let sum = data.iter().fold((0.0, 0.0), |acc, &x| {
        let term = c_mul(c_pow(x, p), c_pow(c_conj(x), q));
        (acc.0 + term.0, acc.1 + term.1)
    });
    (sum.0 / n, sum.1 / n)
}

// ---------------------------------------------------------------------------
// ModulationFingerprinter
// ---------------------------------------------------------------------------

/// Extracts statistical fingerprints from IQ sample buffers and classifies them
/// against known modulation templates.
pub struct ModulationFingerprinter {
    templates: Vec<(ModulationType, ModulationFingerprint)>,
    feature_weights: Vec<f64>,
}

impl ModulationFingerprinter {
    /// Create a new fingerprinter pre-loaded with modulation templates.
    pub fn new() -> Self {
        let templates = Self::build_templates();
        // Default weights: favour cumulants and constellation metrics
        let feature_weights = vec![
            2.0, // c20
            2.0, // c21
            3.0, // c40
            3.0, // c41
            3.0, // c42
            1.5, // c60
            1.5, // c61
            1.5, // c62
            1.5, // c63
            1.0, // amplitude_variance
            1.0, // phase_variance
            1.0, // frequency_variance
            0.5, // spectral_symmetry
            0.5, // spectral_flatness
            1.0, // peak_to_average_ratio
            0.5, // phase_zero_crossing_rate
            0.5, // amplitude_zero_crossing_rate
            2.0, // cluster_count_estimate
            1.5, // cluster_spread
        ];
        Self { templates, feature_weights }
    }

    /// Set custom per-feature weights for classification distance.
    pub fn set_weights(&mut self, weights: Vec<f64>) {
        self.feature_weights = weights;
    }

    // ----- Feature extraction -------------------------------------------------

    /// Extract a [`ModulationFingerprint`] from the supplied IQ samples.
    ///
    /// The samples are centred (mean removed) and power-normalised internally.
    pub fn extract(&self, samples: &[(f64, f64)]) -> ModulationFingerprint {
        if samples.is_empty() {
            return Self::zero_fingerprint();
        }

        // Centre the data
        let mean = c_mean(samples);
        let centred: Vec<(f64, f64)> = samples
            .iter()
            .map(|&(r, i)| (r - mean.0, i - mean.1))
            .collect();

        // Power-normalise
        let power: f64 =
            centred.iter().map(|&(r, i)| r * r + i * i).sum::<f64>() / centred.len() as f64;
        let scale = if power > 1e-30 { 1.0 / power.sqrt() } else { 1.0 };
        let normed: Vec<(f64, f64)> =
            centred.iter().map(|&(r, i)| (r * scale, i * scale)).collect();

        // --- Higher-order cumulants ---
        let m20 = moment_pq(&normed, 2, 0);
        let m21 = moment_pq(&normed, 1, 1); // E[|x|^2]
        let m40 = moment_pq(&normed, 4, 0);
        let m41 = moment_pq(&normed, 3, 1);
        let m42 = moment_pq(&normed, 2, 2);
        let m60 = moment_pq(&normed, 6, 0);
        let m61 = moment_pq(&normed, 5, 1);
        let m62 = moment_pq(&normed, 4, 2);
        let m63 = moment_pq(&normed, 3, 3);

        // 4th-order cumulants
        let c40_val = m40.0 - 3.0 * c_mul(m20, m20).0;
        let c41_val = m41.0 - 3.0 * c_mul(m20, m21).0;
        let c42_val = m42.0 - m21.0 * m21.0 - 2.0 * c_mul(m20, c_conj(m20)).0;

        // 6th-order cumulants (simplified: subtract leading Gaussian terms)
        let c60_val = m60.0 - 15.0 * c_mul(m20, m40).0 + 30.0 * c_pow(m20, 3).0;
        let c61_val = m61.0 - 5.0 * c_mul(m21, m40).0
            - 10.0 * c_mul(m20, m41).0
            + 30.0 * c_mul(c_pow(m20, 2), m21).0;
        let c62_val = m62.0 - 6.0 * c_mul(m20, m42).0
            - 8.0 * c_mul(m21, m41).0
            - c_mul(m40, c_conj(m20)).0
            + 6.0 * m21.0 * m21.0 * m20.0
            + 24.0 * c_mul(m20, c_mul(m21, c_conj(m20))).0;
        let c63_val = m63.0 - 9.0 * c_mul(m21, m42).0 + 12.0 * m21.0.powi(3)
            - 3.0 * c_mul(c_conj(m20), m41).0
            - 3.0 * c_mul(m20, c_conj(m41)).0
            + 18.0 * c_mul(m20, c_mul(c_conj(m20), m21)).0;

        // --- Instantaneous statistics ---
        let amplitudes: Vec<f64> = normed.iter().map(|&s| c_abs(s)).collect();
        let phases: Vec<f64> = normed.iter().map(|&s| c_arg(s)).collect();
        let amplitude_variance = variance(&amplitudes);
        let phase_variance = circular_variance(&phases);

        // Instantaneous frequency (phase differences)
        let freq: Vec<f64> = phases
            .windows(2)
            .map(|w| {
                let mut d = w[1] - w[0];
                // wrap to [-pi, pi]
                while d > PI { d -= 2.0 * PI; }
                while d < -PI { d += 2.0 * PI; }
                d
            })
            .collect();
        let frequency_variance = variance(&freq);

        // --- Spectral features ---
        let psd = power_spectrum(&normed);
        let spectral_symmetry = compute_spectral_symmetry(&psd);
        let spectral_flatness = compute_spectral_flatness(&psd);
        let peak_to_average_ratio = compute_par(&psd);

        // --- Zero-crossing rates ---
        let phase_zero_crossing_rate = zero_crossing_rate(&phases);
        let amplitude_zero_crossing_rate = {
            let amp_mean = mean_f64(&amplitudes);
            let amp_centred: Vec<f64> = amplitudes.iter().map(|&a| a - amp_mean).collect();
            zero_crossing_rate(&amp_centred)
        };

        // --- Constellation shape ---
        let (cluster_count_estimate, cluster_spread) =
            estimate_constellation_shape(&normed);

        ModulationFingerprint {
            c20: m20.0,
            c21: m21.0,
            c40: c40_val,
            c41: c41_val,
            c42: c42_val,
            c60: c60_val,
            c61: c61_val,
            c62: c62_val,
            c63: c63_val,
            amplitude_variance,
            phase_variance,
            frequency_variance,
            spectral_symmetry,
            spectral_flatness,
            peak_to_average_ratio,
            phase_zero_crossing_rate,
            amplitude_zero_crossing_rate,
            cluster_count_estimate,
            cluster_spread,
        }
    }

    // ----- Classification -----------------------------------------------------

    /// Classify a fingerprint against the built-in templates.
    ///
    /// Returns `(ModulationType, confidence)` where confidence is in `[0, 1]`.
    /// A confidence of 1.0 means a perfect match; values below ~0.3 are unreliable.
    pub fn classify(&self, fp: &ModulationFingerprint) -> (ModulationType, f64) {
        if self.templates.is_empty() {
            return (ModulationType::Unknown, 0.0);
        }

        let mut best_type = ModulationType::Unknown;
        let mut best_dist = f64::MAX;

        for (mod_type, template) in &self.templates {
            let d = fp.mahalanobis_like_distance(template, &self.feature_weights);
            if d < best_dist {
                best_dist = d;
                best_type = *mod_type;
            }
        }

        // Convert distance to a confidence score. The mapping is heuristic:
        // confidence = exp(-dist / scale).  Scale chosen so that a distance of
        // ~5 yields confidence ~0.5.
        let scale = 5.0;
        let confidence = (-best_dist / scale).exp();

        (best_type, confidence)
    }

    /// Return a reference to the template library.
    pub fn templates(&self) -> &[(ModulationType, ModulationFingerprint)] {
        &self.templates
    }

    // ----- Template construction (theoretical cumulant values) -----------------

    fn build_templates() -> Vec<(ModulationType, ModulationFingerprint)> {
        vec![
            (ModulationType::Bpsk, Self::template_bpsk()),
            (ModulationType::Qpsk, Self::template_qpsk()),
            (ModulationType::Psk8, Self::template_8psk()),
            (ModulationType::Qam16, Self::template_16qam()),
            (ModulationType::Qam64, Self::template_64qam()),
            (ModulationType::Fsk2, Self::template_fsk2()),
            (ModulationType::Fsk4, Self::template_fsk4()),
        ]
    }

    fn template_bpsk() -> ModulationFingerprint {
        ModulationFingerprint {
            c20: 1.0,
            c21: 1.0,
            c40: -2.0,
            c41: -2.0,
            c42: -2.0,
            c60: 16.0,
            c61: 16.0,
            c62: 16.0,
            c63: 16.0,
            amplitude_variance: 0.0,
            phase_variance: 1.0,
            frequency_variance: 0.5,
            spectral_symmetry: 1.0,
            spectral_flatness: 0.8,
            peak_to_average_ratio: 0.0,
            phase_zero_crossing_rate: 0.5,
            amplitude_zero_crossing_rate: 0.0,
            cluster_count_estimate: 2.0,
            cluster_spread: 0.0,
        }
    }

    fn template_qpsk() -> ModulationFingerprint {
        ModulationFingerprint {
            c20: 0.0,
            c21: 1.0,
            c40: 1.0,
            c41: 0.0,
            c42: -1.0,
            c60: 0.0,
            c61: 4.0,
            c62: 0.0,
            c63: -4.0,
            amplitude_variance: 0.0,
            phase_variance: 0.75,
            frequency_variance: 0.5,
            spectral_symmetry: 1.0,
            spectral_flatness: 0.8,
            peak_to_average_ratio: 0.0,
            phase_zero_crossing_rate: 0.5,
            amplitude_zero_crossing_rate: 0.0,
            cluster_count_estimate: 4.0,
            cluster_spread: 0.0,
        }
    }

    fn template_8psk() -> ModulationFingerprint {
        ModulationFingerprint {
            c20: 0.0,
            c21: 1.0,
            c40: 0.0,
            c41: 0.0,
            c42: -1.0,
            c60: 0.0,
            c61: 0.0,
            c62: 0.0,
            c63: 4.0,
            amplitude_variance: 0.0,
            phase_variance: 0.85,
            frequency_variance: 0.3,
            spectral_symmetry: 1.0,
            spectral_flatness: 0.8,
            peak_to_average_ratio: 0.0,
            phase_zero_crossing_rate: 0.4,
            amplitude_zero_crossing_rate: 0.0,
            cluster_count_estimate: 8.0,
            cluster_spread: 0.0,
        }
    }

    fn template_16qam() -> ModulationFingerprint {
        ModulationFingerprint {
            c20: 0.0,
            c21: 1.0,
            c40: -0.68,
            c41: 0.0,
            c42: -0.68,
            c60: 2.08,
            c61: 0.0,
            c62: 2.08,
            c63: 0.0,
            amplitude_variance: 0.2,
            phase_variance: 0.65,
            frequency_variance: 0.5,
            spectral_symmetry: 1.0,
            spectral_flatness: 0.8,
            peak_to_average_ratio: 2.6,
            phase_zero_crossing_rate: 0.5,
            amplitude_zero_crossing_rate: 0.3,
            cluster_count_estimate: 16.0,
            cluster_spread: 0.15,
        }
    }

    fn template_64qam() -> ModulationFingerprint {
        ModulationFingerprint {
            c20: 0.0,
            c21: 1.0,
            c40: -0.619,
            c41: 0.0,
            c42: -0.619,
            c60: 1.797,
            c61: 0.0,
            c62: 1.797,
            c63: 0.0,
            amplitude_variance: 0.18,
            phase_variance: 0.60,
            frequency_variance: 0.5,
            spectral_symmetry: 1.0,
            spectral_flatness: 0.8,
            peak_to_average_ratio: 3.7,
            phase_zero_crossing_rate: 0.5,
            amplitude_zero_crossing_rate: 0.25,
            cluster_count_estimate: 64.0,
            cluster_spread: 0.08,
        }
    }

    fn template_fsk2() -> ModulationFingerprint {
        ModulationFingerprint {
            c20: 0.0,
            c21: 1.0,
            c40: 0.0,
            c41: 0.0,
            c42: -1.0,
            c60: 0.0,
            c61: 0.0,
            c62: 0.0,
            c63: 4.0,
            amplitude_variance: 0.0,
            phase_variance: 0.9,
            frequency_variance: 0.8,
            spectral_symmetry: 1.0,
            spectral_flatness: 0.3,
            peak_to_average_ratio: 0.0,
            phase_zero_crossing_rate: 0.7,
            amplitude_zero_crossing_rate: 0.0,
            cluster_count_estimate: 2.0,
            cluster_spread: 0.1,
        }
    }

    fn template_fsk4() -> ModulationFingerprint {
        ModulationFingerprint {
            c20: 0.0,
            c21: 1.0,
            c40: 0.0,
            c41: 0.0,
            c42: -1.0,
            c60: 0.0,
            c61: 0.0,
            c62: 0.0,
            c63: 4.0,
            amplitude_variance: 0.0,
            phase_variance: 0.9,
            frequency_variance: 0.6,
            spectral_symmetry: 1.0,
            spectral_flatness: 0.25,
            peak_to_average_ratio: 0.0,
            phase_zero_crossing_rate: 0.6,
            amplitude_zero_crossing_rate: 0.0,
            cluster_count_estimate: 4.0,
            cluster_spread: 0.1,
        }
    }

    fn zero_fingerprint() -> ModulationFingerprint {
        ModulationFingerprint {
            c20: 0.0, c21: 0.0, c40: 0.0, c41: 0.0, c42: 0.0,
            c60: 0.0, c61: 0.0, c62: 0.0, c63: 0.0,
            amplitude_variance: 0.0, phase_variance: 0.0, frequency_variance: 0.0,
            spectral_symmetry: 0.0, spectral_flatness: 0.0, peak_to_average_ratio: 0.0,
            phase_zero_crossing_rate: 0.0, amplitude_zero_crossing_rate: 0.0,
            cluster_count_estimate: 0.0, cluster_spread: 0.0,
        }
    }
}

impl Default for ModulationFingerprinter {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Statistical helpers
// ---------------------------------------------------------------------------

fn mean_f64(v: &[f64]) -> f64 {
    if v.is_empty() { return 0.0; }
    v.iter().sum::<f64>() / v.len() as f64
}

fn variance(v: &[f64]) -> f64 {
    if v.len() < 2 { return 0.0; }
    let m = mean_f64(v);
    v.iter().map(|&x| (x - m) * (x - m)).sum::<f64>() / v.len() as f64
}

/// Circular variance for angular data (result in [0, 1]).
fn circular_variance(angles: &[f64]) -> f64 {
    if angles.is_empty() { return 0.0; }
    let n = angles.len() as f64;
    let c: f64 = angles.iter().map(|&a| a.cos()).sum::<f64>() / n;
    let s: f64 = angles.iter().map(|&a| a.sin()).sum::<f64>() / n;
    1.0 - (c * c + s * s).sqrt()
}

/// Simple power spectrum via DFT (no FFT library needed; O(N^2) but module is
/// self-contained).  We cap the DFT size at 256 bins for performance.
fn power_spectrum(data: &[(f64, f64)]) -> Vec<f64> {
    let n = data.len().min(256);
    let slice = &data[..n];
    let mut psd = Vec::with_capacity(n);
    for k in 0..n {
        let mut re = 0.0f64;
        let mut im = 0.0f64;
        for (m, &(dr, di)) in slice.iter().enumerate() {
            let angle = -2.0 * PI * (k as f64) * (m as f64) / (n as f64);
            let (s, c) = angle.sin_cos();
            re += dr * c - di * s;
            im += dr * s + di * c;
        }
        psd.push(re * re + im * im);
    }
    psd
}

fn compute_spectral_symmetry(psd: &[f64]) -> f64 {
    if psd.len() < 2 { return 1.0; }
    let n = psd.len();
    let half = n / 2;
    let mut sum_diff = 0.0f64;
    let mut sum_total = 0.0f64;
    for i in 1..half {
        let lower = psd[i];
        let upper = psd[n - i];
        sum_diff += (lower - upper).abs();
        sum_total += lower + upper;
    }
    if sum_total < 1e-30 { return 1.0; }
    1.0 - sum_diff / sum_total
}

fn compute_spectral_flatness(psd: &[f64]) -> f64 {
    if psd.is_empty() { return 0.0; }
    let n = psd.len() as f64;
    let eps = 1e-30;
    let log_mean = psd.iter().map(|&p| (p + eps).ln()).sum::<f64>() / n;
    let geo_mean = log_mean.exp();
    let arith_mean = psd.iter().sum::<f64>() / n;
    if arith_mean < eps { return 0.0; }
    (geo_mean / arith_mean).min(1.0).max(0.0)
}

fn compute_par(psd: &[f64]) -> f64 {
    if psd.is_empty() { return 0.0; }
    let avg = psd.iter().sum::<f64>() / psd.len() as f64;
    let peak = psd.iter().cloned().fold(0.0f64, f64::max);
    if avg < 1e-30 { return 0.0; }
    10.0 * (peak / avg).log10()
}

fn zero_crossing_rate(v: &[f64]) -> f64 {
    if v.len() < 2 { return 0.0; }
    let crossings = v.windows(2).filter(|w| w[0].signum() != w[1].signum()).count();
    crossings as f64 / (v.len() - 1) as f64
}

/// Estimate the number of clusters and average spread in the IQ plane.
///
/// Uses a simple histogram-of-phases approach for constant-envelope modulations
/// and k-means-like iteration for multi-amplitude modulations.
fn estimate_constellation_shape(data: &[(f64, f64)]) -> (f64, f64) {
    if data.is_empty() {
        return (0.0, 0.0);
    }

    // Check amplitude variation
    let amps: Vec<f64> = data.iter().map(|&s| c_abs(s)).collect();
    let amp_var = variance(&amps);

    // Use phase histogram for near-constant-envelope signals
    if amp_var < 0.05 {
        let num_bins = 64;
        let mut hist = vec![0u32; num_bins];
        for &s in data {
            let ph = c_arg(s); // [-pi, pi]
            let bin = ((ph + PI) / (2.0 * PI) * num_bins as f64).floor() as usize;
            let bin = bin.min(num_bins - 1);
            hist[bin] += 1;
        }
        // Count peaks: bins with count > mean
        let mean_count = data.len() as f64 / num_bins as f64;
        let threshold = mean_count * 1.5;
        let mut in_peak = false;
        let mut peaks = 0u32;
        for &count in &hist {
            if count as f64 > threshold {
                if !in_peak {
                    peaks += 1;
                    in_peak = true;
                }
            } else {
                in_peak = false;
            }
        }
        return (peaks as f64, amp_var);
    }

    // For multi-amplitude modulations: estimate using unique amplitude-phase
    // combinations via grid quantisation.
    let grid = 8; // 8x8 grid
    let mut occupied = std::collections::HashSet::new();
    for &s in data {
        let r = c_abs(s);
        let th = c_arg(s);
        let ri = ((r * grid as f64).round() as i32).max(0);
        let ti = ((th + PI) / (2.0 * PI) * grid as f64).round() as i32;
        occupied.insert((ri, ti));
    }
    let cluster_count = occupied.len() as f64;

    // Average distance of points to nearest grid centre as a spread metric
    let avg_amp = mean_f64(&amps);
    let spread = if avg_amp > 1e-30 { amp_var.sqrt() / avg_amp } else { 0.0 };

    (cluster_count, spread)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: generate N BPSK symbols (±1, 0).
    fn gen_bpsk(n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| if i % 2 == 0 { (1.0, 0.0) } else { (-1.0, 0.0) })
            .collect()
    }

    /// Helper: generate N QPSK symbols (±1/√2, ±1/√2).
    fn gen_qpsk(n: usize) -> Vec<(f64, f64)> {
        let s = 1.0 / 2.0f64.sqrt();
        let constellation = [(s, s), (-s, s), (-s, -s), (s, -s)];
        (0..n).map(|i| constellation[i % 4]).collect()
    }

    /// Helper: generate N 8PSK symbols.
    fn gen_8psk(n: usize) -> Vec<(f64, f64)> {
        (0..n)
            .map(|i| {
                let angle = 2.0 * PI * (i % 8) as f64 / 8.0;
                (angle.cos(), angle.sin())
            })
            .collect()
    }

    /// Helper: generate N 16-QAM symbols.
    fn gen_16qam(n: usize) -> Vec<(f64, f64)> {
        let levels = [-3.0, -1.0, 1.0, 3.0];
        let mut pts = Vec::new();
        for &i in &levels {
            for &q in &levels {
                pts.push((i, q));
            }
        }
        // Normalise power
        let pwr: f64 = pts.iter().map(|&(r, i)| r * r + i * i).sum::<f64>() / pts.len() as f64;
        let scale = 1.0 / pwr.sqrt();
        let pts: Vec<(f64, f64)> = pts.iter().map(|&(r, i)| (r * scale, i * scale)).collect();
        (0..n).map(|i| pts[i % 16]).collect()
    }

    /// Helper: generate N FSK2-like samples (two distinct tones).
    fn gen_fsk2(n: usize) -> Vec<(f64, f64)> {
        let samples_per_symbol = 16;
        (0..n)
            .map(|i| {
                let symbol = (i / samples_per_symbol) % 2;
                let freq = if symbol == 0 { 0.1 } else { 0.3 };
                let phase = 2.0 * PI * freq * i as f64;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    // ---- Test 1: fingerprint is non-zero for real data ----
    #[test]
    fn test_extract_produces_nonzero_features() {
        let fp = ModulationFingerprinter::new();
        let data = gen_bpsk(512);
        let print = fp.extract(&data);
        let v = print.as_vec();
        assert!(v.iter().any(|&x| x.abs() > 1e-10), "fingerprint should have non-zero features");
    }

    // ---- Test 2: empty input yields zero fingerprint ----
    #[test]
    fn test_extract_empty_input() {
        let fp = ModulationFingerprinter::new();
        let print = fp.extract(&[]);
        let v = print.as_vec();
        assert!(v.iter().all(|&x| x.abs() < 1e-15), "empty input should give zero fingerprint");
    }

    // ---- Test 3: BPSK classification ----
    #[test]
    fn test_classify_bpsk() {
        let fp = ModulationFingerprinter::new();
        let data = gen_bpsk(1024);
        let print = fp.extract(&data);
        let (mt, conf) = fp.classify(&print);
        assert_eq!(mt, ModulationType::Bpsk, "should classify as BPSK");
        assert!(conf > 0.0, "confidence should be positive");
    }

    // ---- Test 4: QPSK classification ----
    #[test]
    fn test_classify_qpsk() {
        let fp = ModulationFingerprinter::new();
        let data = gen_qpsk(1024);
        let print = fp.extract(&data);
        let (mt, _conf) = fp.classify(&print);
        assert_eq!(mt, ModulationType::Qpsk, "should classify as QPSK");
    }

    // ---- Test 5: 8PSK classification ----
    #[test]
    fn test_classify_8psk() {
        let fp = ModulationFingerprinter::new();
        let data = gen_8psk(1024);
        let print = fp.extract(&data);
        let (mt, _conf) = fp.classify(&print);
        assert_eq!(mt, ModulationType::Psk8, "should classify as 8PSK");
    }

    // ---- Test 6: Euclidean distance is zero for identical fingerprints ----
    #[test]
    fn test_euclidean_distance_zero_self() {
        let fp = ModulationFingerprinter::new();
        let data = gen_bpsk(256);
        let print = fp.extract(&data);
        let d = print.euclidean_distance(&print);
        assert!(d.abs() < 1e-12, "distance to self should be zero, got {}", d);
    }

    // ---- Test 7: Euclidean distance is symmetric ----
    #[test]
    fn test_euclidean_distance_symmetric() {
        let fp = ModulationFingerprinter::new();
        let a = fp.extract(&gen_bpsk(256));
        let b = fp.extract(&gen_qpsk(256));
        let d1 = a.euclidean_distance(&b);
        let d2 = b.euclidean_distance(&a);
        assert!((d1 - d2).abs() < 1e-12, "distance should be symmetric");
    }

    // ---- Test 8: different modulations have positive distance ----
    #[test]
    fn test_different_modulations_positive_distance() {
        let fp = ModulationFingerprinter::new();
        let a = fp.extract(&gen_bpsk(512));
        let b = fp.extract(&gen_qpsk(512));
        assert!(a.euclidean_distance(&b) > 0.1, "different modulations should have nonzero distance");
    }

    // ---- Test 9: Mahalanobis-like distance with uniform weights equals Euclidean ----
    #[test]
    fn test_mahalanobis_uniform_weights_equals_euclidean() {
        let fp = ModulationFingerprinter::new();
        let a = fp.extract(&gen_bpsk(256));
        let b = fp.extract(&gen_qpsk(256));
        let weights = vec![1.0; 19];
        let d_m = a.mahalanobis_like_distance(&b, &weights);
        let d_e = a.euclidean_distance(&b);
        assert!(
            (d_m - d_e).abs() < 1e-10,
            "uniform-weight Mahalanobis should equal Euclidean: {} vs {}",
            d_m, d_e
        );
    }

    // ---- Test 10: Mahalanobis-like with wrong weight length falls back to Euclidean ----
    #[test]
    fn test_mahalanobis_wrong_weight_length() {
        let fp = ModulationFingerprinter::new();
        let a = fp.extract(&gen_bpsk(256));
        let b = fp.extract(&gen_qpsk(256));
        let short_weights = vec![1.0; 5]; // wrong length
        let d_m = a.mahalanobis_like_distance(&b, &short_weights);
        let d_e = a.euclidean_distance(&b);
        assert!(
            (d_m - d_e).abs() < 1e-10,
            "wrong-length weights should fall back to Euclidean"
        );
    }

    // ---- Test 11: feature vector length is 19 ----
    #[test]
    fn test_feature_vector_length() {
        let fp = ModulationFingerprinter::new();
        let print = fp.extract(&gen_bpsk(128));
        assert_eq!(print.as_vec().len(), 19, "feature vector should have 19 elements");
    }

    // ---- Test 12: template library has 7 entries ----
    #[test]
    fn test_template_count() {
        let fp = ModulationFingerprinter::new();
        assert_eq!(fp.templates().len(), 7, "should have 7 modulation templates");
    }

    // ---- Test 13: confidence is in [0, 1] ----
    #[test]
    fn test_confidence_range() {
        let fp = ModulationFingerprinter::new();
        for data in &[gen_bpsk(256), gen_qpsk(256), gen_8psk(256)] {
            let print = fp.extract(data);
            let (_mt, conf) = fp.classify(&print);
            assert!(
                (0.0..=1.0).contains(&conf),
                "confidence {} should be in [0,1]",
                conf
            );
        }
    }

    // ---- Test 14: spectral flatness is in [0, 1] ----
    #[test]
    fn test_spectral_flatness_range() {
        let fp = ModulationFingerprinter::new();
        let print = fp.extract(&gen_bpsk(256));
        assert!(
            (0.0..=1.0).contains(&print.spectral_flatness),
            "spectral flatness {} should be in [0,1]",
            print.spectral_flatness
        );
    }

    // ---- Test 15: spectral symmetry is in [0, 1] for symmetric signal ----
    #[test]
    fn test_spectral_symmetry_range() {
        let fp = ModulationFingerprinter::new();
        let print = fp.extract(&gen_bpsk(256));
        assert!(
            print.spectral_symmetry >= 0.0 && print.spectral_symmetry <= 1.0 + 1e-10,
            "spectral symmetry {} should be in [0,1]",
            print.spectral_symmetry
        );
    }

    // ---- Test 16: zero-crossing rate is in [0, 1] ----
    #[test]
    fn test_zero_crossing_rate_range() {
        let fp = ModulationFingerprinter::new();
        let print = fp.extract(&gen_bpsk(512));
        assert!(
            (0.0..=1.0).contains(&print.phase_zero_crossing_rate),
            "phase ZCR {} should be in [0,1]",
            print.phase_zero_crossing_rate
        );
        assert!(
            (0.0..=1.0).contains(&print.amplitude_zero_crossing_rate),
            "amplitude ZCR {} should be in [0,1]",
            print.amplitude_zero_crossing_rate
        );
    }

    // ---- Test 17: cluster count estimate is positive for real signals ----
    #[test]
    fn test_cluster_count_positive() {
        let fp = ModulationFingerprinter::new();
        let print = fp.extract(&gen_qpsk(1024));
        assert!(
            print.cluster_count_estimate >= 1.0,
            "cluster count {} should be >= 1 for QPSK",
            print.cluster_count_estimate
        );
    }

    // ---- Test 18: 16-QAM has higher amplitude variance than BPSK ----
    #[test]
    fn test_amplitude_variance_ordering() {
        let fp = ModulationFingerprinter::new();
        let bpsk_print = fp.extract(&gen_bpsk(1024));
        let qam_print = fp.extract(&gen_16qam(1024));
        assert!(
            qam_print.amplitude_variance > bpsk_print.amplitude_variance,
            "16-QAM should have higher amplitude variance ({}) than BPSK ({})",
            qam_print.amplitude_variance,
            bpsk_print.amplitude_variance,
        );
    }

    // ---- Test 19: FSK2 has non-zero frequency variance ----
    #[test]
    fn test_fsk_frequency_variance() {
        let fp = ModulationFingerprinter::new();
        let data = gen_fsk2(1024);
        let print = fp.extract(&data);
        assert!(
            print.frequency_variance > 1e-6,
            "FSK2 should have non-zero frequency variance, got {}",
            print.frequency_variance
        );
    }

    // ---- Test 20: Default trait works ----
    #[test]
    fn test_default_trait() {
        let fp = ModulationFingerprinter::default();
        assert_eq!(fp.templates().len(), 7);
    }

    // ---- Test 21: set_weights changes classification behaviour ----
    #[test]
    fn test_set_weights() {
        let mut fp = ModulationFingerprinter::new();
        // Zero out all weights - distances should all be zero, first template wins
        fp.set_weights(vec![0.0; 19]);
        let print = fp.extract(&gen_bpsk(256));
        let (mt, _conf) = fp.classify(&print);
        // With zero weights, distance is 0 for all templates -> first template = BPSK
        assert_eq!(mt, ModulationType::Bpsk);
    }
}
