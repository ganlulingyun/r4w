//! Zero-Crossing Rate Detector and Speech Activity Detection
//!
//! Computes the zero-crossing rate (ZCR) and related features for
//! signal classification. The ZCR measures how often a signal crosses
//! zero per unit time, useful for distinguishing voiced/unvoiced speech,
//! modulation classification, and transient detection.
//!
//! Includes voice activity detection (VAD) using combined energy + ZCR
//! features.
//!
//! No direct GNU Radio equivalent (signal classification feature).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::zero_crossing_detector::{zero_crossing_rate, ZcrAnalyzer};
//!
//! let signal: Vec<f64> = (0..256)
//!     .map(|i| (i as f64 * 0.5).sin())
//!     .collect();
//! let zcr = zero_crossing_rate(&signal);
//! assert!(zcr > 0.0 && zcr < 1.0);
//!
//! let mut analyzer = ZcrAnalyzer::new(64, 32);
//! let zcr_series = analyzer.process(&signal);
//! assert!(!zcr_series.is_empty());
//! ```

use std::f64::consts::PI;

/// Compute the zero-crossing rate of a signal.
///
/// ZCR = (1/N) · Σ |sign(x[n]) - sign(x[n-1])| / 2
///
/// Returns a value in [0, 1] where 0 = no crossings, 1 = crossing every sample.
pub fn zero_crossing_rate(signal: &[f64]) -> f64 {
    if signal.len() < 2 {
        return 0.0;
    }
    let count = signal
        .windows(2)
        .filter(|w| (w[0] >= 0.0) != (w[1] >= 0.0))
        .count();
    count as f64 / (signal.len() - 1) as f64
}

/// Count zero crossings in a signal.
pub fn zero_crossing_count(signal: &[f64]) -> usize {
    if signal.len() < 2 {
        return 0;
    }
    signal
        .windows(2)
        .filter(|w| (w[0] >= 0.0) != (w[1] >= 0.0))
        .count()
}

/// Estimate frequency from zero-crossing rate.
///
/// For a sinusoidal signal: f ≈ ZCR · fs / 2
///
/// `sample_rate`: sampling frequency in Hz.
/// Returns estimated frequency in Hz.
pub fn estimate_frequency_from_zcr(signal: &[f64], sample_rate: f64) -> f64 {
    zero_crossing_rate(signal) * sample_rate / 2.0
}

/// Sliding-window ZCR analyzer.
#[derive(Debug, Clone)]
pub struct ZcrAnalyzer {
    /// Window size in samples.
    window_size: usize,
    /// Hop size in samples.
    hop_size: usize,
}

impl ZcrAnalyzer {
    /// Create a new ZCR analyzer.
    pub fn new(window_size: usize, hop_size: usize) -> Self {
        Self {
            window_size: window_size.max(2),
            hop_size: hop_size.max(1),
        }
    }

    /// Process a signal, returning ZCR for each window.
    pub fn process(&self, signal: &[f64]) -> Vec<f64> {
        let mut results = Vec::new();
        let mut pos = 0;
        while pos + self.window_size <= signal.len() {
            let window = &signal[pos..pos + self.window_size];
            results.push(zero_crossing_rate(window));
            pos += self.hop_size;
        }
        results
    }

    /// Process returning both ZCR and short-time energy.
    pub fn process_with_energy(&self, signal: &[f64]) -> Vec<(f64, f64)> {
        let mut results = Vec::new();
        let mut pos = 0;
        while pos + self.window_size <= signal.len() {
            let window = &signal[pos..pos + self.window_size];
            let zcr = zero_crossing_rate(window);
            let energy = window.iter().map(|x| x * x).sum::<f64>() / self.window_size as f64;
            results.push((zcr, energy));
            pos += self.hop_size;
        }
        results
    }

    /// Get the window size.
    pub fn window_size(&self) -> usize {
        self.window_size
    }

    /// Get the hop size.
    pub fn hop_size(&self) -> usize {
        self.hop_size
    }
}

/// Voice Activity Detector using energy + ZCR features.
#[derive(Debug, Clone)]
pub struct VoiceActivityDetector {
    /// Window size for analysis.
    window_size: usize,
    /// Hop size for analysis.
    hop_size: usize,
    /// Energy threshold (relative to noise floor estimate).
    energy_threshold: f64,
    /// ZCR threshold (voiced speech typically < 0.3).
    zcr_threshold: f64,
    /// Hangover frames (keep active after last detection).
    hangover: usize,
    /// Noise floor estimate (running average of silent frames).
    noise_floor: f64,
    /// Adaptation rate for noise floor.
    adaptation_rate: f64,
}

impl VoiceActivityDetector {
    /// Create a new VAD.
    pub fn new(window_size: usize, hop_size: usize) -> Self {
        Self {
            window_size: window_size.max(2),
            hop_size: hop_size.max(1),
            energy_threshold: 3.0, // 3x noise floor
            zcr_threshold: 0.4,
            hangover: 5,
            noise_floor: 0.0,
            adaptation_rate: 0.01,
        }
    }

    /// Set the energy threshold multiplier.
    pub fn energy_threshold(mut self, threshold: f64) -> Self {
        self.energy_threshold = threshold;
        self
    }

    /// Set the ZCR threshold.
    pub fn zcr_threshold(mut self, threshold: f64) -> Self {
        self.zcr_threshold = threshold;
        self
    }

    /// Set hangover frames.
    pub fn hangover(mut self, frames: usize) -> Self {
        self.hangover = frames;
        self
    }

    /// Process a signal, returning per-frame voice activity decisions.
    ///
    /// Returns a vector of booleans (true = voice active).
    pub fn detect(&mut self, signal: &[f64]) -> Vec<bool> {
        let mut decisions = Vec::new();
        let mut hangover_count = 0;
        let mut pos = 0;

        // Initialize noise floor from first few frames
        if self.noise_floor == 0.0 && signal.len() >= self.window_size {
            let init_frames = 5.min(signal.len() / self.window_size);
            let mut total_energy = 0.0;
            for i in 0..init_frames {
                let start = i * self.window_size;
                let window = &signal[start..start + self.window_size];
                total_energy +=
                    window.iter().map(|x| x * x).sum::<f64>() / self.window_size as f64;
            }
            self.noise_floor = total_energy / init_frames.max(1) as f64;
        }

        while pos + self.window_size <= signal.len() {
            let window = &signal[pos..pos + self.window_size];
            let zcr = zero_crossing_rate(window);
            let energy = window.iter().map(|x| x * x).sum::<f64>() / self.window_size as f64;

            let energy_active = energy > self.energy_threshold * self.noise_floor.max(1e-20);
            let zcr_active = zcr < self.zcr_threshold; // Voiced speech has low ZCR

            let is_active = energy_active && zcr_active;

            if is_active {
                hangover_count = self.hangover;
            } else if hangover_count > 0 {
                hangover_count -= 1;
            }

            decisions.push(is_active || hangover_count > 0);

            // Adapt noise floor during silence
            if !is_active && hangover_count == 0 {
                self.noise_floor = (1.0 - self.adaptation_rate) * self.noise_floor
                    + self.adaptation_rate * energy;
            }

            pos += self.hop_size;
        }

        decisions
    }

    /// Reset the detector state.
    pub fn reset(&mut self) {
        self.noise_floor = 0.0;
    }

    /// Get the current noise floor estimate.
    pub fn noise_floor(&self) -> f64 {
        self.noise_floor
    }
}

/// Compute the spectral centroid of a signal frame.
///
/// The spectral centroid is the "center of mass" of the spectrum,
/// correlating with perceived brightness.
pub fn spectral_centroid(signal: &[f64], sample_rate: f64) -> f64 {
    let n = signal.len();
    if n == 0 {
        return 0.0;
    }

    // Compute magnitude spectrum (simple DFT for first half)
    let half = n / 2;
    let mut weighted_sum = 0.0;
    let mut magnitude_sum = 0.0;

    for k in 0..half {
        let freq = k as f64 * sample_rate / n as f64;
        let mut re = 0.0;
        let mut im = 0.0;
        for (j, &x) in signal.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            re += x * angle.cos();
            im += x * angle.sin();
        }
        let mag = (re * re + im * im).sqrt();
        weighted_sum += freq * mag;
        magnitude_sum += mag;
    }

    if magnitude_sum < 1e-30 {
        0.0
    } else {
        weighted_sum / magnitude_sum
    }
}

/// Compute the spectral flatness (Wiener entropy).
///
/// Measures how tone-like vs noise-like a signal is.
/// Returns a value in [0, 1] where 0 = pure tone, 1 = white noise.
pub fn spectral_flatness(signal: &[f64]) -> f64 {
    let n = signal.len();
    if n < 2 {
        return 0.0;
    }

    let half = n / 2;
    let mut magnitudes = Vec::with_capacity(half);

    for k in 1..half {
        let mut re = 0.0;
        let mut im = 0.0;
        for (j, &x) in signal.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            re += x * angle.cos();
            im += x * angle.sin();
        }
        let mag_sq = re * re + im * im;
        magnitudes.push(mag_sq.max(1e-30));
    }

    if magnitudes.is_empty() {
        return 0.0;
    }

    let m = magnitudes.len() as f64;
    let geometric_mean = (magnitudes.iter().map(|x| x.ln()).sum::<f64>() / m).exp();
    let arithmetic_mean = magnitudes.iter().sum::<f64>() / m;

    if arithmetic_mean < 1e-30 {
        0.0
    } else {
        (geometric_mean / arithmetic_mean).min(1.0)
    }
}

/// Classify modulation type based on ZCR and spectral features.
///
/// Simple classifier that distinguishes:
/// - FSK (high ZCR, moderate spectral flatness)
/// - PSK (moderate ZCR, low spectral flatness)
/// - AM (low ZCR, low spectral flatness)
/// - Noise (high ZCR, high spectral flatness)
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ModulationGuess {
    Fsk,
    Psk,
    Am,
    Noise,
    Unknown,
}

/// Classify a signal frame's modulation type.
pub fn classify_modulation(signal: &[f64]) -> ModulationGuess {
    if signal.len() < 16 {
        return ModulationGuess::Unknown;
    }

    let zcr = zero_crossing_rate(signal);
    let flatness = spectral_flatness(signal);

    if flatness > 0.5 {
        ModulationGuess::Noise
    } else if zcr > 0.35 {
        ModulationGuess::Fsk
    } else if zcr > 0.15 {
        ModulationGuess::Psk
    } else {
        ModulationGuess::Am
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_zcr_pure_sine() {
        // Sine at normalized frequency f has ZCR ≈ 2f
        let f = 0.1; // 10% of Nyquist
        let signal: Vec<f64> = (0..1000).map(|i| (2.0 * PI * f * i as f64).sin()).collect();
        let zcr = zero_crossing_rate(&signal);
        // ZCR should be near 2*f = 0.2
        assert!(
            (zcr - 2.0 * f).abs() < 0.05,
            "zcr={zcr}, expected≈{}",
            2.0 * f
        );
    }

    #[test]
    fn test_zcr_dc() {
        let signal = vec![1.0; 100];
        let zcr = zero_crossing_rate(&signal);
        assert_eq!(zcr, 0.0);
    }

    #[test]
    fn test_zcr_alternating() {
        // +1, -1, +1, -1, ... has ZCR = 1.0
        let signal: Vec<f64> = (0..100).map(|i| if i % 2 == 0 { 1.0 } else { -1.0 }).collect();
        let zcr = zero_crossing_rate(&signal);
        assert!((zcr - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_zero_crossing_count() {
        let signal = vec![1.0, -1.0, 1.0, -1.0, 1.0];
        assert_eq!(zero_crossing_count(&signal), 4);
    }

    #[test]
    fn test_frequency_estimation() {
        let f = 440.0; // Hz
        let fs = 16000.0;
        let signal: Vec<f64> = (0..8000)
            .map(|i| (2.0 * PI * f * i as f64 / fs).sin())
            .collect();
        let est = estimate_frequency_from_zcr(&signal, fs);
        assert!(
            (est - f).abs() < 50.0,
            "estimated={est}, expected={f}"
        );
    }

    #[test]
    fn test_zcr_analyzer() {
        let signal: Vec<f64> = (0..512).map(|i| (i as f64 * 0.1).sin()).collect();
        let analyzer = ZcrAnalyzer::new(64, 32);
        let zcrs = analyzer.process(&signal);
        assert!(!zcrs.is_empty());
        assert!(zcrs.iter().all(|&z| z >= 0.0 && z <= 1.0));
    }

    #[test]
    fn test_zcr_analyzer_with_energy() {
        let signal: Vec<f64> = (0..256).map(|i| (i as f64 * 0.1).sin()).collect();
        let analyzer = ZcrAnalyzer::new(64, 32);
        let results = analyzer.process_with_energy(&signal);
        assert!(!results.is_empty());
        for (zcr, energy) in &results {
            assert!(*zcr >= 0.0 && *zcr <= 1.0);
            assert!(*energy >= 0.0);
        }
    }

    #[test]
    fn test_vad_silence() {
        let signal = vec![0.001; 1024]; // Near-silence
        let mut vad = VoiceActivityDetector::new(64, 32);
        let decisions = vad.detect(&signal);
        // Should mostly be inactive
        let active_count = decisions.iter().filter(|&&d| d).count();
        assert!(
            active_count < decisions.len() / 2,
            "too many active: {}/{}", active_count, decisions.len()
        );
    }

    #[test]
    fn test_vad_active() {
        // Start with silence so noise floor is low, then add a loud sinusoid
        let mut signal = vec![0.0f64; 640]; // 5 frames of silence for noise floor estimation
        signal.extend((0..2048).map(|i| 0.5 * (2.0 * PI * 0.05 * i as f64).sin()));
        let mut vad = VoiceActivityDetector::new(128, 64)
            .energy_threshold(1.5);
        let decisions = vad.detect(&signal);
        let active_count = decisions.iter().filter(|&&d| d).count();
        // Should have some active frames in the sinusoidal portion
        assert!(active_count > 0, "no active frames detected");
    }

    #[test]
    fn test_spectral_centroid() {
        // Low-frequency sine should have low centroid
        let low: Vec<f64> = (0..256).map(|i| (2.0 * PI * 0.02 * i as f64).sin()).collect();
        let high: Vec<f64> = (0..256).map(|i| (2.0 * PI * 0.3 * i as f64).sin()).collect();
        let c_low = spectral_centroid(&low, 1000.0);
        let c_high = spectral_centroid(&high, 1000.0);
        assert!(c_low < c_high, "c_low={c_low}, c_high={c_high}");
    }

    #[test]
    fn test_spectral_flatness() {
        // Sinusoid has low flatness (tonal)
        let sine: Vec<f64> = (0..256).map(|i| (i as f64 * 0.1).sin()).collect();
        let flat = spectral_flatness(&sine);
        assert!(flat < 0.5, "flatness={flat}");
    }

    #[test]
    fn test_classify_modulation() {
        // Noise-like signal
        let noise: Vec<f64> = (0..256)
            .map(|i| {
                let seed = (i as u64).wrapping_mul(6364136223846793005).wrapping_add(1);
                (seed >> 33) as f64 / (1u64 << 31) as f64 - 0.5
            })
            .collect();
        let class = classify_modulation(&noise);
        // Should be noise or FSK (high ZCR)
        assert!(class == ModulationGuess::Noise || class == ModulationGuess::Fsk);
    }

    #[test]
    fn test_short_signal() {
        assert_eq!(zero_crossing_rate(&[1.0]), 0.0);
        assert_eq!(zero_crossing_rate(&[]), 0.0);
    }
}
