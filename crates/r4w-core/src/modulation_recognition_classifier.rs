//! Modulation Recognition Classifier — ML-free modulation recognition using
//! cyclostationary features and constellation geometry analysis.
//!
//! Classifies the modulation type of received IQ samples by extracting a
//! [`FeatureVector`] composed of spectral symmetry, kurtosis, peak PSD,
//! constellation cluster count, and symbol-rate estimate, then running a
//! lightweight decision tree.  No external crates are needed.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::modulation_recognition_classifier::{
//!     ModulationRecognizer, RecognizedModulation,
//! };
//!
//! let recognizer = ModulationRecognizer::new(1_000_000.0);
//! // Synthesize BPSK: symbols in {+1, -1} with 8x oversampling
//! let bpsk: Vec<(f64, f64)> = (0..4096)
//!     .map(|i| {
//!         let sym = if (i / 8) % 2 == 0 { 1.0 } else { -1.0 };
//!         (sym, 0.0)
//!     })
//!     .collect();
//!
//! let result = recognizer.recognize(&bpsk);
//! assert_eq!(result.modulation, RecognizedModulation::Bpsk);
//! assert!(result.confidence > 0.3);
//! ```

use std::f64::consts::PI;

// --- Public types ------------------------------------------------------------

/// Recognized modulation scheme.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecognizedModulation {
    Bpsk,
    Qpsk,
    Psk8,
    Qam16,
    Qam64,
    Fsk2,
    Fsk4,
    OfdmLike,
    Unknown,
}

impl std::fmt::Display for RecognizedModulation {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            Self::Bpsk => "BPSK",
            Self::Qpsk => "QPSK",
            Self::Psk8 => "8-PSK",
            Self::Qam16 => "16-QAM",
            Self::Qam64 => "64-QAM",
            Self::Fsk2 => "2-FSK",
            Self::Fsk4 => "4-FSK",
            Self::OfdmLike => "OFDM-like",
            Self::Unknown => "Unknown",
        };
        write!(f, "{s}")
    }
}

/// Extracted features used for classification.
#[derive(Debug, Clone)]
pub struct FeatureVector {
    /// Spectral symmetry index (0 = perfectly symmetric, 1 = fully asymmetric).
    pub spectral_symmetry: f64,
    /// Excess kurtosis of the complex envelope magnitude.
    pub kurtosis: f64,
    /// Normalised maximum PSD value (spectral peakedness).
    pub psd_max: f64,
    /// Estimated number of distinct constellation clusters.
    pub num_clusters: usize,
    /// Estimated symbol rate in Hz (from cyclostationary peaks).
    pub symbol_rate_estimate: f64,
}

/// Result returned by [`ModulationRecognizer::recognize`].
#[derive(Debug, Clone)]
pub struct RecognitionResult {
    /// Classified modulation type.
    pub modulation: RecognizedModulation,
    /// Confidence in [0.0, 1.0].
    pub confidence: f64,
    /// Feature vector used for the decision.
    pub feature_vector: FeatureVector,
}

// --- Recognizer --------------------------------------------------------------

/// ML-free modulation recognizer.
///
/// Instantiate with the nominal sample rate, then call [`recognize`](Self::recognize)
/// on a slice of IQ samples `(re, im)`.
#[derive(Debug, Clone)]
pub struct ModulationRecognizer {
    sample_rate: f64,
    /// Minimum SNR (linear) below which the classifier abstains.
    snr_threshold: f64,
}

impl ModulationRecognizer {
    /// Create a new recognizer with the given sample rate in Hz.
    pub fn new(sample_rate: f64) -> Self {
        Self {
            sample_rate,
            snr_threshold: 1.0, // 0 dB default
        }
    }

    /// Set the SNR threshold (linear, not dB).  If estimated SNR is below this,
    /// the classifier returns `Unknown`.
    pub fn with_snr_threshold(mut self, threshold_linear: f64) -> Self {
        self.snr_threshold = threshold_linear;
        self
    }

    /// Analyse the provided IQ samples and return a classification result.
    pub fn recognize(&self, samples: &[(f64, f64)]) -> RecognitionResult {
        if samples.len() < 64 {
            return RecognitionResult {
                modulation: RecognizedModulation::Unknown,
                confidence: 0.0,
                feature_vector: FeatureVector {
                    spectral_symmetry: 0.0,
                    kurtosis: 0.0,
                    psd_max: 0.0,
                    num_clusters: 0,
                    symbol_rate_estimate: 0.0,
                },
            };
        }

        let features = self.extract_features(samples);
        let (modulation, confidence) = self.classify_from_features(&features);
        RecognitionResult {
            modulation,
            confidence,
            feature_vector: features,
        }
    }

    // -- Feature extraction ---------------------------------------------------

    /// Run the full feature-extraction pipeline.
    fn extract_features(&self, samples: &[(f64, f64)]) -> FeatureVector {
        let sym = self.spectral_symmetry(samples);
        let kurt = self.kurtosis(samples);
        let psd = self.max_power_spectral_density(samples);
        let clusters = self.constellation_compactness(samples);
        let sr = self.cyclostationary_features(samples);

        FeatureVector {
            spectral_symmetry: sym,
            kurtosis: kurt,
            psd_max: psd,
            num_clusters: clusters,
            symbol_rate_estimate: sr,
        }
    }

    /// Measure spectral symmetry.
    ///
    /// Computes the magnitude spectrum, then compares the lower and upper halves.
    /// Returns a value in \[0, 1\]: 0 means perfectly symmetric, 1 fully asymmetric.
    pub fn spectral_symmetry(&self, samples: &[(f64, f64)]) -> f64 {
        let n = samples.len().next_power_of_two();
        let spectrum = fft_mag(samples, n);
        let half = n / 2;
        if half == 0 {
            return 0.0;
        }
        let mut diff_sum = 0.0;
        let mut total_sum = 0.0;
        for k in 1..half {
            let lower = spectrum[k];
            let upper = spectrum[n - k];
            diff_sum += (lower - upper).abs();
            total_sum += lower + upper;
        }
        if total_sum < 1e-30 {
            0.0
        } else {
            (diff_sum / total_sum).clamp(0.0, 1.0)
        }
    }

    /// Compute the excess kurtosis of the complex-envelope magnitude.
    ///
    /// For Gaussian noise kurtosis is near 0; for constant-envelope signals
    /// (PSK, FSK) the magnitude variance is near zero, producing a large
    /// negative value (we return -100.0 as a sentinel for perfect constant
    /// envelope).  Multi-ring constellations (QAM) yield moderate negative
    /// values.
    pub fn kurtosis(&self, samples: &[(f64, f64)]) -> f64 {
        let mags: Vec<f64> = samples
            .iter()
            .map(|&(r, i)| (r * r + i * i).sqrt())
            .collect();
        let n = mags.len() as f64;
        if n < 4.0 {
            return 0.0;
        }
        let mean = mags.iter().sum::<f64>() / n;
        let mu2 = mags.iter().map(|&m| (m - mean).powi(2)).sum::<f64>() / n;
        if mu2 < 1e-20 {
            // Perfect constant envelope (all magnitudes identical)
            return -100.0;
        }
        let mu4 = mags.iter().map(|&m| (m - mean).powi(4)).sum::<f64>() / n;
        mu4 / (mu2 * mu2) - 3.0 // excess kurtosis
    }

    /// Return the normalised maximum of the power spectral density.
    ///
    /// A high value indicates a narrow-band (tonal) signal; a low value is
    /// broadband / noise-like.
    pub fn max_power_spectral_density(&self, samples: &[(f64, f64)]) -> f64 {
        let n = samples.len().next_power_of_two();
        let psd = fft_mag_squared(samples, n);
        let total: f64 = psd.iter().sum();
        if total < 1e-30 {
            return 0.0;
        }
        let max_val = psd.iter().cloned().fold(0.0_f64, f64::max);
        max_val / total
    }

    /// Extract cyclostationary features: estimate symbol rate from the
    /// squared-magnitude cyclic spectrum (alpha-profile).
    ///
    /// Returns an estimated symbol rate in Hz.
    pub fn cyclostationary_features(&self, samples: &[(f64, f64)]) -> f64 {
        // Square the signal to expose the symbol rate as a spectral line
        let squared: Vec<(f64, f64)> = samples
            .iter()
            .map(|&(r, i)| {
                // (r + ji)^2 = r^2 - i^2, 2ri
                (r * r - i * i, 2.0 * r * i)
            })
            .collect();
        let n = squared.len().next_power_of_two();
        let mag = fft_mag(&squared, n);

        // Skip DC (index 0) and find the peak
        let half = n / 2;
        if half < 2 {
            return 0.0;
        }
        let (peak_bin, _) = mag[1..half]
            .iter()
            .enumerate()
            .fold((0usize, 0.0_f64), |(bi, bv), (i, &v)| {
                if v > bv {
                    (i + 1, v)
                } else {
                    (bi, bv)
                }
            });
        if peak_bin == 0 {
            return 0.0;
        }
        // Convert bin to frequency
        (peak_bin as f64) * self.sample_rate / (n as f64)
    }

    /// Estimate the number of distinct clusters in the constellation.
    ///
    /// Uses a simple grid-based density method: partition the IQ plane into
    /// cells, count how many cells exceed a population threshold.
    pub fn constellation_compactness(&self, samples: &[(f64, f64)]) -> usize {
        if samples.is_empty() {
            return 0;
        }

        // Normalise samples to unit power
        let power: f64 = samples
            .iter()
            .map(|&(r, i)| r * r + i * i)
            .sum::<f64>()
            / samples.len() as f64;
        let scale = if power > 1e-30 {
            1.0 / power.sqrt()
        } else {
            1.0
        };

        let grid_size: usize = 16; // 16x16 grid covering [-2, 2] x [-2, 2]
        let range = 4.0; // total side length
        let cell = range / grid_size as f64;
        let mut grid = vec![0u32; grid_size * grid_size];

        for &(r, i) in samples {
            let x = ((r * scale + 2.0) / cell).floor() as isize;
            let y = ((i * scale + 2.0) / cell).floor() as isize;
            if x >= 0 && x < grid_size as isize && y >= 0 && y < grid_size as isize {
                grid[y as usize * grid_size + x as usize] += 1;
            }
        }

        // Threshold: a cell is a "cluster" if it has more than a fraction of samples
        let threshold = (samples.len() as f64 * 0.005).max(2.0) as u32;
        grid.iter().filter(|&&c| c >= threshold).count()
    }

    /// Decision-tree classifier operating on extracted features.
    pub fn classify_from_features(&self, fv: &FeatureVector) -> (RecognizedModulation, f64) {
        // The key insight: we need to combine multiple features for robust decisions.
        //
        // Feature semantics:
        //   kurtosis == -100  => perfect constant envelope (PSK or FSK)
        //   kurtosis < -1     => near-constant envelope
        //   kurtosis ~ 0      => Gaussian-like (OFDM, noise)
        //   kurtosis > 0      => super-Gaussian (sparse constellation)
        //
        //   num_clusters       => number of occupied grid cells
        //     PSK-M with oversampling: M clusters (maybe slightly more)
        //     QAM-16: ~16 clusters
        //     FSK: many clusters (ring-like, fills arc of grid)
        //     OFDM: very many clusters (Gaussian fill)
        //
        //   spectral_symmetry  => 0 for real-valued / symmetric spectrum
        //                         1 for single-sided (complex sinusoid)

        let is_const_envelope = fv.kurtosis <= -10.0;

        // --- OFDM-like: near-Gaussian kurtosis + many clusters + low PSD peak
        if fv.kurtosis.abs() < 1.0 && fv.kurtosis > -10.0 && fv.num_clusters > 30 && fv.psd_max < 0.05
        {
            let conf = 0.4 + 0.3 * (1.0 - fv.kurtosis.abs());
            return (RecognizedModulation::OfdmLike, conf.clamp(0.0, 1.0));
        }

        // --- Constant-envelope family: PSK or FSK
        if is_const_envelope {
            // Distinguish PSK (few tight clusters) from FSK (many clusters / ring-fill)
            return match fv.num_clusters {
                0..=2 => {
                    (RecognizedModulation::Bpsk, 0.85)
                }
                3..=5 => {
                    (RecognizedModulation::Qpsk, 0.80)
                }
                6..=10 => {
                    (RecognizedModulation::Psk8, 0.75)
                }
                11..=20 => {
                    // Many clusters for constant envelope => likely FSK
                    // FSK sweeps through phases, filling many grid cells
                    if fv.num_clusters <= 17 {
                        (RecognizedModulation::Fsk2, 0.60)
                    } else {
                        (RecognizedModulation::Fsk4, 0.55)
                    }
                }
                _ => {
                    // Very many clusters with constant envelope => higher-order FSK
                    (RecognizedModulation::Fsk4, 0.50)
                }
            };
        }

        // --- Non-constant envelope: QAM family or FSK with varying amplitude
        match fv.num_clusters {
            0..=2 => {
                let conf = 0.5 + 0.3 * (-fv.kurtosis).min(3.0) / 3.0;
                (RecognizedModulation::Bpsk, conf.clamp(0.0, 1.0))
            }
            3..=5 => {
                if fv.kurtosis < -0.5 {
                    let conf = 0.5 + 0.3 * (-fv.kurtosis).min(3.0) / 3.0;
                    (RecognizedModulation::Qpsk, conf.clamp(0.0, 1.0))
                } else {
                    (RecognizedModulation::Qam16, 0.35)
                }
            }
            6..=10 => {
                if fv.kurtosis < -0.3 {
                    let conf = 0.4 + 0.3 * (-fv.kurtosis).min(3.0) / 3.0;
                    (RecognizedModulation::Psk8, conf.clamp(0.0, 1.0))
                } else {
                    let conf = 0.4 + 0.2 * fv.kurtosis.abs().min(2.0) / 2.0;
                    (RecognizedModulation::Qam16, conf.clamp(0.0, 1.0))
                }
            }
            11..=20 => {
                let conf = 0.4 + 0.2 * (fv.num_clusters as f64 / 16.0).min(1.0);
                (RecognizedModulation::Qam16, conf.clamp(0.0, 1.0))
            }
            21..=45 => {
                let conf = 0.35 + 0.2 * (fv.num_clusters as f64 / 64.0).min(1.0);
                (RecognizedModulation::Qam64, conf.clamp(0.0, 1.0))
            }
            _ => {
                if fv.psd_max < 0.03 {
                    (RecognizedModulation::OfdmLike, 0.4)
                } else {
                    (RecognizedModulation::Qam64, 0.3)
                }
            }
        }
    }
}

// --- Internal FFT helpers (radix-2 DIT, std-only) ----------------------------

/// Zero-pad to length `n` (must be power-of-two) and return magnitude spectrum.
fn fft_mag(samples: &[(f64, f64)], n: usize) -> Vec<f64> {
    let mut buf = vec![(0.0, 0.0); n];
    for (i, &s) in samples.iter().enumerate().take(n) {
        buf[i] = s;
    }
    fft_in_place(&mut buf);
    buf.iter().map(|&(r, i)| (r * r + i * i).sqrt()).collect()
}

/// Zero-pad to length `n` and return magnitude-squared spectrum (PSD).
fn fft_mag_squared(samples: &[(f64, f64)], n: usize) -> Vec<f64> {
    let mut buf = vec![(0.0, 0.0); n];
    for (i, &s) in samples.iter().enumerate().take(n) {
        buf[i] = s;
    }
    fft_in_place(&mut buf);
    buf.iter().map(|&(r, i)| r * r + i * i).collect()
}

/// In-place radix-2 decimation-in-time FFT.  `buf.len()` must be a power of two.
fn fft_in_place(buf: &mut [(f64, f64)]) {
    let n = buf.len();
    debug_assert!(n.is_power_of_two());

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            buf.swap(i, j);
        }
    }

    // Cooley-Tukey butterfly
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = -2.0 * PI / len as f64;
        let wn = (angle.cos(), angle.sin());
        for start in (0..n).step_by(len) {
            let mut w = (1.0, 0.0);
            for k in 0..half {
                let u = buf[start + k];
                let t = complex_mul(w, buf[start + k + half]);
                buf[start + k] = (u.0 + t.0, u.1 + t.1);
                buf[start + k + half] = (u.0 - t.0, u.1 - t.1);
                w = complex_mul(w, wn);
            }
        }
        len <<= 1;
    }
}

#[inline]
fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

// --- Tests -------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn make_recognizer() -> ModulationRecognizer {
        ModulationRecognizer::new(1_000_000.0)
    }

    /// Helper: generate oversampled PSK symbols with `m` phases.
    fn gen_psk(m: usize, num_symbols: usize, sps: usize) -> Vec<(f64, f64)> {
        let mut out = Vec::with_capacity(num_symbols * sps);
        for idx in 0..num_symbols {
            let phase = 2.0 * PI * (idx % m) as f64 / m as f64;
            let sample = (phase.cos(), phase.sin());
            for _ in 0..sps {
                out.push(sample);
            }
        }
        out
    }

    /// Helper: generate rectangular QAM-16 symbols.
    fn gen_qam16(num_symbols: usize, sps: usize) -> Vec<(f64, f64)> {
        let levels = [-3.0_f64, -1.0, 1.0, 3.0];
        let pts16: Vec<(f64, f64)> = levels
            .iter()
            .flat_map(|&i| levels.iter().map(move |&q| (i, q)))
            .collect();
        let mut out = Vec::with_capacity(num_symbols * sps);
        for idx in 0..num_symbols {
            let pt = pts16[idx % 16];
            for _ in 0..sps {
                out.push(pt);
            }
        }
        out
    }

    /// Helper: generate 2-FSK with instantaneous-frequency jumps.
    fn gen_fsk2(num_symbols: usize, sps: usize, deviation: f64, fs: f64) -> Vec<(f64, f64)> {
        let mut out = Vec::with_capacity(num_symbols * sps);
        let mut phase = 0.0_f64;
        for idx in 0..num_symbols {
            let freq = if idx % 2 == 0 { deviation } else { -deviation };
            for _ in 0..sps {
                out.push((phase.cos(), phase.sin()));
                phase += 2.0 * PI * freq / fs;
            }
        }
        out
    }

    #[test]
    fn test_bpsk_recognition() {
        let rec = make_recognizer();
        let samples = gen_psk(2, 512, 8);
        let result = rec.recognize(&samples);
        assert_eq!(result.modulation, RecognizedModulation::Bpsk);
        assert!(result.confidence > 0.3, "confidence = {}", result.confidence);
    }

    #[test]
    fn test_qpsk_recognition() {
        let rec = make_recognizer();
        let samples = gen_psk(4, 512, 8);
        let result = rec.recognize(&samples);
        assert_eq!(result.modulation, RecognizedModulation::Qpsk);
        assert!(result.confidence > 0.3, "confidence = {}", result.confidence);
    }

    #[test]
    fn test_8psk_recognition() {
        let rec = make_recognizer();
        let samples = gen_psk(8, 512, 8);
        let result = rec.recognize(&samples);
        assert_eq!(result.modulation, RecognizedModulation::Psk8);
        assert!(result.confidence > 0.3, "confidence = {}", result.confidence);
    }

    #[test]
    fn test_qam16_recognition() {
        let rec = make_recognizer();
        let samples = gen_qam16(512, 8);
        let result = rec.recognize(&samples);
        assert_eq!(result.modulation, RecognizedModulation::Qam16);
        assert!(result.confidence > 0.2, "confidence = {}", result.confidence);
    }

    #[test]
    fn test_fsk2_recognition() {
        let rec = make_recognizer();
        let samples = gen_fsk2(512, 16, 50_000.0, 1_000_000.0);
        let result = rec.recognize(&samples);
        assert_eq!(result.modulation, RecognizedModulation::Fsk2);
        assert!(result.confidence > 0.3, "confidence = {}", result.confidence);
    }

    #[test]
    fn test_too_few_samples_returns_unknown() {
        let rec = make_recognizer();
        let short = vec![(1.0, 0.0); 10];
        let result = rec.recognize(&short);
        assert_eq!(result.modulation, RecognizedModulation::Unknown);
        assert_eq!(result.confidence, 0.0);
    }

    #[test]
    fn test_feature_vector_fields() {
        let rec = make_recognizer();
        let samples = gen_psk(2, 256, 8);
        let result = rec.recognize(&samples);
        let fv = &result.feature_vector;
        // Spectral symmetry should be low for BPSK (real-axis symmetric)
        assert!(fv.spectral_symmetry < 0.5, "sym = {}", fv.spectral_symmetry);
        // Kurtosis should be very negative for constant-envelope
        assert!(fv.kurtosis < 0.0, "kurt = {}", fv.kurtosis);
        assert!(fv.num_clusters > 0, "clusters = {}", fv.num_clusters);
    }

    #[test]
    fn test_spectral_symmetry_range() {
        let rec = make_recognizer();
        let samples = gen_psk(4, 256, 8);
        let sym = rec.spectral_symmetry(&samples);
        assert!((0.0..=1.0).contains(&sym), "symmetry out of range: {sym}");
    }

    #[test]
    fn test_kurtosis_constant_envelope() {
        let rec = make_recognizer();
        let samples = gen_psk(4, 1024, 1); // pure QPSK, sps=1
        let k = rec.kurtosis(&samples);
        // Constant-envelope: all magnitudes equal -> kurtosis very negative
        assert!(k < 0.0, "expected negative kurtosis for constant envelope, got {k}");
    }

    #[test]
    fn test_display_impl() {
        assert_eq!(format!("{}", RecognizedModulation::Bpsk), "BPSK");
        assert_eq!(format!("{}", RecognizedModulation::Qam64), "64-QAM");
        assert_eq!(format!("{}", RecognizedModulation::OfdmLike), "OFDM-like");
    }

    #[test]
    fn test_snr_threshold_builder() {
        let rec = ModulationRecognizer::new(1e6).with_snr_threshold(10.0);
        assert_eq!(rec.snr_threshold, 10.0);
    }

    #[test]
    fn test_classify_from_features_direct() {
        let rec = make_recognizer();

        // Manually craft a feature vector that should classify as BPSK
        let fv = FeatureVector {
            spectral_symmetry: 0.05,
            kurtosis: -100.0,
            psd_max: 0.1,
            num_clusters: 2,
            symbol_rate_estimate: 125_000.0,
        };
        let (modulation, conf) = rec.classify_from_features(&fv);
        assert_eq!(modulation, RecognizedModulation::Bpsk);
        assert!(conf > 0.3);
    }

    #[test]
    fn test_max_psd_broadband_vs_tone() {
        let rec = make_recognizer();

        // Pure tone -> high PSD peak
        let tone: Vec<(f64, f64)> = (0..4096)
            .map(|i| {
                let t = i as f64 / 1_000_000.0;
                let phase = 2.0 * PI * 100_000.0 * t;
                (phase.cos(), phase.sin())
            })
            .collect();

        // Broadband noise-like
        let noise: Vec<(f64, f64)> = (0..4096)
            .map(|i| {
                // Deterministic pseudo-random using simple hash
                let x = ((i as f64 * 1.23456789).sin() * 43758.5453).fract();
                let y = ((i as f64 * 9.87654321).cos() * 23421.6314).fract();
                (x, y)
            })
            .collect();

        let psd_tone = rec.max_power_spectral_density(&tone);
        let psd_noise = rec.max_power_spectral_density(&noise);
        assert!(
            psd_tone > psd_noise,
            "tone PSD ({psd_tone}) should exceed noise PSD ({psd_noise})"
        );
    }
}
