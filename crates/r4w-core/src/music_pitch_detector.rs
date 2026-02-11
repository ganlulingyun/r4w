//! Music Pitch Detector — Polyphonic and Monophonic Pitch Detection
//!
//! Implements pitch detection for music information retrieval using:
//! - **YIN algorithm**: Autocorrelation-based monophonic pitch detection with
//!   cumulative mean normalized difference for robust F0 estimation
//! - **Harmonic Product Spectrum (HPS)**: Downsample-and-multiply for polyphonic
//!   fundamental frequency detection from magnitude spectra
//! - **Chromagram extraction**: 12-bin pitch class profile mapping frequencies
//!   to semitone bins (C, C#, D, ..., B)
//! - **Onset detection**: Spectral flux method for note onset timing
//! - **A-weighting**: Perceptual frequency weighting curve (IEC 61672)
//!
//! All math is implemented from scratch with no external dependencies.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::music_pitch_detector::{PitchConfig, PitchDetector};
//!
//! let config = PitchConfig {
//!     sample_rate_hz: 44100.0,
//!     ..PitchConfig::default()
//! };
//! let detector = PitchDetector::new(config);
//!
//! // Generate a 440 Hz sine wave (A4)
//! let duration_samples = 4410; // 100 ms at 44.1 kHz
//! let audio: Vec<f64> = (0..duration_samples)
//!     .map(|i| (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 44100.0).sin())
//!     .collect();
//!
//! let frames = detector.detect_pitch(&audio);
//! assert!(!frames.is_empty());
//! // First detected pitch should be close to 440 Hz
//! let f = frames[0].frequency_hz;
//! assert!((f - 440.0).abs() < 10.0);
//! ```

use std::f64::consts::PI;

// ─── Constants ───────────────────────────────────────────────────────────────

const A4_FREQUENCY: f64 = 440.0;
const A4_MIDI_NOTE: u8 = 69;

/// Note names in chromatic order starting from C.
const NOTE_NAMES: [&str; 12] = [
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B",
];

/// Default YIN threshold for pitch detection.
const DEFAULT_YIN_THRESHOLD: f64 = 0.15;

// ─── Configuration ───────────────────────────────────────────────────────────

/// Configuration for the pitch detector.
#[derive(Debug, Clone)]
pub struct PitchConfig {
    /// Audio sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Minimum detectable frequency in Hz.
    pub min_frequency_hz: f64,
    /// Maximum detectable frequency in Hz.
    pub max_frequency_hz: f64,
    /// Analysis frame size in milliseconds.
    pub frame_size_ms: f64,
    /// Hop size between frames in milliseconds.
    pub hop_size_ms: f64,
}

impl Default for PitchConfig {
    fn default() -> Self {
        Self {
            sample_rate_hz: 44100.0,
            min_frequency_hz: 50.0,
            max_frequency_hz: 2000.0,
            frame_size_ms: 40.0,
            hop_size_ms: 10.0,
        }
    }
}

// ─── Pitch Frame ─────────────────────────────────────────────────────────────

/// A single pitch detection result for one analysis frame.
#[derive(Debug, Clone)]
pub struct PitchFrame {
    /// Detected fundamental frequency in Hz (0.0 if unvoiced/silent).
    pub frequency_hz: f64,
    /// Confidence of the detection in [0.0, 1.0].
    pub confidence: f64,
    /// Nearest MIDI note number (0-127).
    pub midi_note: u8,
    /// Note name with octave (e.g., "A4", "C#3").
    pub note_name: String,
    /// Deviation from the nearest equal-tempered pitch in cents.
    pub cents_deviation: f64,
}

// ─── Pitch Detector ──────────────────────────────────────────────────────────

/// Multi-frame pitch detector using the YIN algorithm.
///
/// Segments the input audio into overlapping frames and runs YIN pitch
/// estimation on each frame. Returns a sequence of [`PitchFrame`] results.
pub struct PitchDetector {
    config: PitchConfig,
}

impl PitchDetector {
    /// Create a new pitch detector with the given configuration.
    pub fn new(config: PitchConfig) -> Self {
        Self { config }
    }

    /// Detect pitch across all frames of the input audio buffer.
    ///
    /// The audio is segmented into overlapping frames of `frame_size_ms` with
    /// a hop of `hop_size_ms`. Each frame is analyzed with the YIN algorithm.
    pub fn detect_pitch(&self, audio: &[f64]) -> Vec<PitchFrame> {
        let frame_size = (self.config.frame_size_ms * self.config.sample_rate_hz / 1000.0) as usize;
        let hop_size = (self.config.hop_size_ms * self.config.sample_rate_hz / 1000.0) as usize;

        if frame_size == 0 || hop_size == 0 || audio.len() < frame_size {
            return Vec::new();
        }

        let mut frames = Vec::new();
        let mut offset = 0;

        while offset + frame_size <= audio.len() {
            let frame_data = &audio[offset..offset + frame_size];

            let result = yin_pitch(
                frame_data,
                self.config.sample_rate_hz,
                self.config.min_frequency_hz,
                self.config.max_frequency_hz,
                DEFAULT_YIN_THRESHOLD,
            );

            let pf = match result {
                Some((freq, conf)) => {
                    let (midi, cents) = cents_from_nearest(freq);
                    PitchFrame {
                        frequency_hz: freq,
                        confidence: conf,
                        midi_note: midi,
                        note_name: midi_to_note_name(midi),
                        cents_deviation: cents,
                    }
                }
                None => PitchFrame {
                    frequency_hz: 0.0,
                    confidence: 0.0,
                    midi_note: 0,
                    note_name: String::from("--"),
                    cents_deviation: 0.0,
                },
            };

            frames.push(pf);
            offset += hop_size;
        }

        frames
    }
}

// ─── YIN Algorithm ───────────────────────────────────────────────────────────

/// YIN pitch detection algorithm (de Cheveigné & Kawahara, 2002).
///
/// Estimates the fundamental frequency of a monophonic signal frame using
/// the cumulative mean normalized difference function.
///
/// Returns `Some((frequency_hz, confidence))` if a pitch is found below
/// the threshold, or `None` if the frame is unvoiced/silent.
///
/// # Arguments
/// * `frame` - Audio samples for one analysis frame
/// * `fs` - Sample rate in Hz
/// * `min_freq` - Minimum frequency to search (Hz)
/// * `max_freq` - Maximum frequency to search (Hz)
/// * `threshold` - CMNDF threshold in (0, 1); lower = stricter
pub fn yin_pitch(
    frame: &[f64],
    fs: f64,
    min_freq: f64,
    max_freq: f64,
    threshold: f64,
) -> Option<(f64, f64)> {
    if frame.is_empty() || fs <= 0.0 || min_freq <= 0.0 || max_freq <= min_freq {
        return None;
    }

    // Lag range corresponds to the frequency range
    let min_lag = (fs / max_freq).floor() as usize;
    let max_lag = (fs / min_freq).ceil() as usize;

    // Need at least max_lag samples in the second half for the difference function
    let max_lag = max_lag.min(frame.len() / 2);
    if min_lag >= max_lag {
        return None;
    }

    // Step 1: Difference function
    let d = difference_function(frame, max_lag);

    // Step 2: Cumulative mean normalized difference
    let cmndf = cumulative_mean_normalized_difference(&d);

    // Step 3: Absolute threshold — find the first valley below threshold
    let mut tau = min_lag;
    let mut best_tau = None;

    // Search for the first dip below threshold after min_lag
    while tau < cmndf.len() {
        if cmndf[tau] < threshold {
            // Walk to the local minimum
            while tau + 1 < cmndf.len() && cmndf[tau + 1] < cmndf[tau] {
                tau += 1;
            }
            best_tau = Some(tau);
            break;
        }
        tau += 1;
    }

    // If no value below threshold, find the global minimum in the search range
    let tau = match best_tau {
        Some(t) => t,
        None => {
            let mut min_val = f64::MAX;
            let mut min_idx = min_lag;
            for i in min_lag..cmndf.len() {
                if cmndf[i] < min_val {
                    min_val = cmndf[i];
                    min_idx = i;
                }
            }
            // Only accept if the minimum is reasonably low
            if min_val > 0.5 {
                return None;
            }
            min_idx
        }
    };

    // Step 4: Parabolic interpolation for sub-sample accuracy
    let refined_tau = parabolic_interpolation(&cmndf, tau);

    if refined_tau <= 0.0 {
        return None;
    }

    let frequency = fs / refined_tau;

    // Confidence is 1 - cmndf[tau], clamped to [0, 1]
    let confidence = (1.0 - cmndf[tau]).clamp(0.0, 1.0);

    Some((frequency, confidence))
}

/// Parabolic interpolation around a valley in the CMNDF for sub-sample
/// lag estimation.
///
/// Given index `tau` of a local minimum, fits a parabola through
/// `(tau-1, tau, tau+1)` and returns the interpolated minimum location.
fn parabolic_interpolation(data: &[f64], tau: usize) -> f64 {
    if tau == 0 || tau >= data.len() - 1 {
        return tau as f64;
    }

    let s0 = data[tau - 1];
    let s1 = data[tau];
    let s2 = data[tau + 1];

    let denominator = 2.0 * s1 - s0 - s2;
    if denominator.abs() < 1e-12 {
        return tau as f64;
    }

    let adjustment = (s0 - s2) / (2.0 * denominator);
    tau as f64 + adjustment
}

// ─── Difference Function ─────────────────────────────────────────────────────

/// Compute the YIN difference function.
///
/// ```text
/// d(tau) = sum_{j=0}^{W-1} (x[j] - x[j + tau])^2
/// ```
///
/// where W = `frame.len() / 2` (or `max_lag` if smaller).
///
/// # Arguments
/// * `signal` - Input audio frame
/// * `max_lag` - Maximum lag (tau) to compute
///
/// # Returns
/// Vector of length `max_lag + 1` containing d(0), d(1), ..., d(max_lag).
pub fn difference_function(signal: &[f64], max_lag: usize) -> Vec<f64> {
    let n = signal.len();
    let max_lag = max_lag.min(n / 2);
    let mut d = vec![0.0; max_lag + 1];

    // d(0) = 0 by definition
    for tau in 1..=max_lag {
        let mut sum = 0.0;
        let window = n - max_lag;
        for j in 0..window {
            let diff = signal[j] - signal[j + tau];
            sum += diff * diff;
        }
        d[tau] = sum;
    }

    d
}

// ─── Cumulative Mean Normalized Difference ───────────────────────────────────

/// Compute the Cumulative Mean Normalized Difference Function (CMNDF).
///
/// Normalizes the difference function so that dips are easier to detect:
/// ```text
/// d'(0) = 1
/// d'(tau) = d(tau) / ((1/tau) * sum_{j=1}^{tau} d(j))
/// ```
///
/// This normalization prevents the tendency of the raw difference function
/// to favor smaller lags.
///
/// # Arguments
/// * `d` - Raw difference function values from [`difference_function`]
///
/// # Returns
/// Normalized values of the same length.
pub fn cumulative_mean_normalized_difference(d: &[f64]) -> Vec<f64> {
    let n = d.len();
    if n == 0 {
        return Vec::new();
    }

    let mut cmndf = vec![0.0; n];
    cmndf[0] = 1.0; // By convention

    let mut running_sum = 0.0;
    for tau in 1..n {
        running_sum += d[tau];
        if running_sum.abs() < 1e-30 {
            cmndf[tau] = 1.0;
        } else {
            cmndf[tau] = d[tau] * tau as f64 / running_sum;
        }
    }

    cmndf
}

// ─── Harmonic Product Spectrum ───────────────────────────────────────────────

/// Harmonic Product Spectrum (HPS) for polyphonic fundamental frequency detection.
///
/// Downsamples the magnitude spectrum by integer factors 1, 2, ..., `num_harmonics`
/// and multiplies the results element-wise. The fundamental frequency produces a
/// peak where all harmonics align.
///
/// # Arguments
/// * `spectrum` - Magnitude spectrum (e.g., from FFT). Typically the first N/2+1
///   bins of |FFT(x)|.
/// * `num_harmonics` - Number of harmonic products (typically 3-5).
///
/// # Returns
/// HPS result vector. The length equals `spectrum.len() / num_harmonics`.
/// The index of the maximum corresponds to the fundamental frequency bin.
pub fn harmonic_product_spectrum(spectrum: &[f64], num_harmonics: usize) -> Vec<f64> {
    if spectrum.is_empty() || num_harmonics == 0 {
        return Vec::new();
    }

    let out_len = spectrum.len() / num_harmonics.max(1);
    if out_len == 0 {
        return Vec::new();
    }

    let mut hps = vec![1.0; out_len];

    for harmonic in 1..=num_harmonics {
        for i in 0..out_len {
            let idx = i * harmonic;
            if idx < spectrum.len() {
                hps[i] *= spectrum[idx];
            }
        }
    }

    hps
}

// ─── Chromagram ──────────────────────────────────────────────────────────────

/// Compute a chromagram (pitch class profile) from an audio signal.
///
/// Maps spectral energy into 12 pitch class bins (C, C#, D, ..., B) using
/// a simple DFT-based approach. Each frequency bin in the spectrum is assigned
/// to its nearest pitch class with energy accumulation.
///
/// # Arguments
/// * `signal` - Time-domain audio samples
/// * `fs` - Sample rate in Hz
/// * `num_bins` - Number of chroma bins (normally 12 for Western music)
///
/// # Returns
/// Vector of `num_bins` chroma energy values, normalized to [0, 1].
pub fn chromagram(signal: &[f64], fs: f64, num_bins: usize) -> Vec<f64> {
    if signal.is_empty() || fs <= 0.0 || num_bins == 0 {
        return vec![0.0; num_bins];
    }

    // Compute magnitude spectrum via DFT (real-valued input)
    let n = signal.len();
    let half = n / 2 + 1;
    let mut mag_spectrum = Vec::with_capacity(half);

    for k in 0..half {
        let mut re = 0.0;
        let mut im = 0.0;
        let freq_k = 2.0 * PI * k as f64 / n as f64;
        for (j, &s) in signal.iter().enumerate() {
            let angle = freq_k * j as f64;
            re += s * angle.cos();
            im -= s * angle.sin();
        }
        mag_spectrum.push((re * re + im * im).sqrt());
    }

    // Map each frequency bin to a chroma bin
    let mut chroma = vec![0.0; num_bins];

    for (k, &mag) in mag_spectrum.iter().enumerate() {
        if k == 0 {
            continue; // Skip DC
        }
        let freq = k as f64 * fs / n as f64;
        if freq < 20.0 || freq > 10000.0 {
            continue; // Ignore sub-audio and very high frequencies
        }

        // Map frequency to chroma bin
        // chroma index = round(12 * log2(freq / C0)) mod 12
        // C0 ~ 16.35 Hz
        let c0 = A4_FREQUENCY * 2.0_f64.powf(-(A4_MIDI_NOTE as f64) / 12.0);
        let semitone = 12.0 * (freq / c0).log2();
        let bin = ((semitone.round() as i64) % num_bins as i64 + num_bins as i64) as usize
            % num_bins;

        chroma[bin] += mag * mag; // Energy (magnitude squared)
    }

    // Normalize to [0, 1]
    let max_val = chroma.iter().cloned().fold(0.0_f64, f64::max);
    if max_val > 0.0 {
        for v in &mut chroma {
            *v /= max_val;
        }
    }

    chroma
}

// ─── MIDI / Note Conversions ─────────────────────────────────────────────────

/// Convert a frequency in Hz to the nearest MIDI note number.
///
/// Uses the standard formula: `round(69 + 12 * log2(freq / 440))`.
/// Clamps to [0, 127].
pub fn frequency_to_midi(freq_hz: f64) -> u8 {
    if freq_hz <= 0.0 {
        return 0;
    }
    let midi = (A4_MIDI_NOTE as f64 + 12.0 * (freq_hz / A4_FREQUENCY).log2()).round();
    midi.clamp(0.0, 127.0) as u8
}

/// Convert a MIDI note number to its equal-tempered frequency in Hz.
///
/// Uses the standard formula: `440 * 2^((midi - 69) / 12)`.
pub fn midi_to_frequency(midi_note: u8) -> f64 {
    A4_FREQUENCY * 2.0_f64.powf((midi_note as f64 - A4_MIDI_NOTE as f64) / 12.0)
}

/// Convert a MIDI note number to a human-readable note name with octave.
///
/// Examples: 69 -> "A4", 60 -> "C4", 48 -> "C3", 70 -> "A#4".
pub fn midi_to_note_name(midi_note: u8) -> String {
    let note_index = (midi_note % 12) as usize;
    let octave = (midi_note as i32 / 12) - 1; // MIDI octave convention: C4 = 60
    format!("{}{}", NOTE_NAMES[note_index], octave)
}

/// Find the nearest MIDI note and cents deviation for a given frequency.
///
/// Returns `(nearest_midi_note, cents_deviation)` where cents_deviation
/// is in the range [-50, +50).
pub fn cents_from_nearest(freq_hz: f64) -> (u8, f64) {
    if freq_hz <= 0.0 {
        return (0, 0.0);
    }

    let exact_midi = A4_MIDI_NOTE as f64 + 12.0 * (freq_hz / A4_FREQUENCY).log2();
    let nearest = exact_midi.round();
    let cents = (exact_midi - nearest) * 100.0;

    (nearest.clamp(0.0, 127.0) as u8, cents)
}

// ─── Onset Detection ─────────────────────────────────────────────────────────

/// Spectral flux onset detection.
///
/// Computes the half-wave rectified spectral flux between consecutive
/// short-time Fourier transform frames. Onset locations are at peaks
/// in the flux curve that exceed the adaptive threshold.
///
/// # Arguments
/// * `signal` - Time-domain audio
/// * `fs` - Sample rate in Hz
/// * `hop_size` - Hop between frames in samples
///
/// # Returns
/// Vector of sample indices where onsets are detected.
pub fn onset_detection(signal: &[f64], fs: f64, hop_size: usize) -> Vec<usize> {
    if signal.is_empty() || fs <= 0.0 || hop_size == 0 {
        return Vec::new();
    }

    // Use a frame size that is a power-of-2-like multiple of hop_size
    let frame_size = hop_size * 4;
    if signal.len() < frame_size {
        return Vec::new();
    }

    let num_bins = frame_size / 2 + 1;

    // Compute magnitude spectra for successive frames
    let mut spectra: Vec<Vec<f64>> = Vec::new();
    let mut offset = 0;

    while offset + frame_size <= signal.len() {
        let frame = &signal[offset..offset + frame_size];
        let mut mag = Vec::with_capacity(num_bins);

        for k in 0..num_bins {
            let mut re = 0.0;
            let mut im = 0.0;
            let freq_k = 2.0 * PI * k as f64 / frame_size as f64;
            for (j, &s) in frame.iter().enumerate() {
                // Apply Hann window
                let w = 0.5 * (1.0 - (2.0 * PI * j as f64 / (frame_size - 1) as f64).cos());
                let sample = s * w;
                let angle = freq_k * j as f64;
                re += sample * angle.cos();
                im -= sample * angle.sin();
            }
            mag.push((re * re + im * im).sqrt());
        }

        spectra.push(mag);
        offset += hop_size;
    }

    if spectra.len() < 2 {
        return Vec::new();
    }

    // Compute spectral flux (half-wave rectified)
    let mut flux = vec![0.0; spectra.len()];
    for i in 1..spectra.len() {
        let mut sf = 0.0;
        for k in 0..num_bins {
            let diff = spectra[i][k] - spectra[i - 1][k];
            if diff > 0.0 {
                sf += diff;
            }
        }
        flux[i] = sf;
    }

    // Adaptive threshold: local mean + constant factor
    let window = 10; // frames
    let multiplier = 1.5;
    let mut onsets = Vec::new();

    for i in 1..flux.len() - 1 {
        // Local mean
        let start = if i > window { i - window } else { 0 };
        let end = (i + window + 1).min(flux.len());
        let local_mean: f64 = flux[start..end].iter().sum::<f64>() / (end - start) as f64;

        let adaptive_thresh = local_mean * multiplier;

        // Peak picking: flux[i] is a local max and above threshold
        if flux[i] > flux[i - 1] && flux[i] > flux[i + 1] && flux[i] > adaptive_thresh {
            // Convert frame index to sample index
            onsets.push(i * hop_size);
        }
    }

    onsets
}

// ─── A-Weighting ─────────────────────────────────────────────────────────────

/// A-weighting filter magnitude at a given frequency (IEC 61672:2003).
///
/// Returns the A-weighting correction in dB. The A-weighting curve
/// approximates human hearing sensitivity and is defined as:
///
/// ```text
/// R_A(f) = 12194^2 * f^4 / ((f^2 + 20.6^2) * sqrt((f^2 + 107.7^2)(f^2 + 737.9^2)) * (f^2 + 12194^2))
/// A(f) = 20*log10(R_A(f)) + 2.00
/// ```
///
/// # Arguments
/// * `freq_hz` - Frequency in Hz (must be > 0)
///
/// # Returns
/// A-weighting value in dB. Returns `f64::NEG_INFINITY` for freq <= 0.
pub fn a_weighting_db(freq_hz: f64) -> f64 {
    if freq_hz <= 0.0 {
        return f64::NEG_INFINITY;
    }

    let f2 = freq_hz * freq_hz;

    let num = 12194.0_f64.powi(2) * f2 * f2;

    let d1 = f2 + 20.6_f64.powi(2);
    let d2 = f2 + 107.7_f64.powi(2);
    let d3 = f2 + 737.9_f64.powi(2);
    let d4 = f2 + 12194.0_f64.powi(2);

    let den = d1 * (d2 * d3).sqrt() * d4;

    if den <= 0.0 {
        return f64::NEG_INFINITY;
    }

    let ra = num / den;
    20.0 * ra.log10() + 2.00
}

// ─── Internal Helpers ────────────────────────────────────────────────────────

/// Generate a pure sine wave for testing.
fn generate_sine(frequency_hz: f64, sample_rate_hz: f64, num_samples: usize) -> Vec<f64> {
    (0..num_samples)
        .map(|i| (2.0 * PI * frequency_hz * i as f64 / sample_rate_hz).sin())
        .collect()
}

/// Simple magnitude spectrum via DFT (for test/internal use).
fn magnitude_spectrum(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    let half = n / 2 + 1;
    let mut mag = Vec::with_capacity(half);

    for k in 0..half {
        let mut re = 0.0;
        let mut im = 0.0;
        let freq_k = 2.0 * PI * k as f64 / n as f64;
        for (j, &s) in signal.iter().enumerate() {
            let angle = freq_k * j as f64;
            re += s * angle.cos();
            im -= s * angle.sin();
        }
        mag.push((re * re + im * im).sqrt());
    }

    mag
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // --- YIN on pure sine waves ---

    #[test]
    fn test_yin_a4_440hz() {
        let fs = 44100.0;
        let signal = generate_sine(440.0, fs, 4410);
        let result = yin_pitch(&signal, fs, 50.0, 2000.0, 0.2);
        assert!(result.is_some(), "YIN should detect A4");
        let (freq, conf) = result.unwrap();
        assert!(
            (freq - 440.0).abs() < 5.0,
            "Expected ~440 Hz, got {:.1}",
            freq
        );
        assert!(conf > 0.8, "Confidence should be high, got {:.3}", conf);
    }

    #[test]
    fn test_yin_c3_131hz() {
        let fs = 44100.0;
        // C3 = MIDI 48 ~ 130.81 Hz
        let freq_c3 = midi_to_frequency(48);
        let signal = generate_sine(freq_c3, fs, 8820); // 200 ms
        let result = yin_pitch(&signal, fs, 50.0, 2000.0, 0.2);
        assert!(result.is_some(), "YIN should detect C3");
        let (freq, conf) = result.unwrap();
        assert!(
            (freq - freq_c3).abs() < 3.0,
            "Expected ~{:.1} Hz, got {:.1}",
            freq_c3,
            freq
        );
        assert!(conf > 0.7, "Confidence should be reasonable, got {:.3}", conf);
    }

    #[test]
    fn test_yin_e5_659hz() {
        let fs = 44100.0;
        // E5 = MIDI 76 ~ 659.26 Hz
        let freq_e5 = midi_to_frequency(76);
        let signal = generate_sine(freq_e5, fs, 4410);
        let result = yin_pitch(&signal, fs, 50.0, 2000.0, 0.2);
        assert!(result.is_some(), "YIN should detect E5");
        let (freq, _) = result.unwrap();
        assert!(
            (freq - freq_e5).abs() < 5.0,
            "Expected ~{:.1} Hz, got {:.1}",
            freq_e5,
            freq
        );
    }

    #[test]
    fn test_yin_silence() {
        let fs = 44100.0;
        let silence = vec![0.0; 4410];
        let result = yin_pitch(&silence, fs, 50.0, 2000.0, 0.15);
        // Silence should either return None or very low confidence
        if let Some((_, conf)) = result {
            assert!(conf < 0.3, "Silence should have low confidence");
        }
    }

    #[test]
    fn test_yin_empty_input() {
        let result = yin_pitch(&[], 44100.0, 50.0, 2000.0, 0.15);
        assert!(result.is_none());
    }

    #[test]
    fn test_yin_invalid_params() {
        let signal = generate_sine(440.0, 44100.0, 4410);
        assert!(yin_pitch(&signal, 0.0, 50.0, 2000.0, 0.15).is_none());
        assert!(yin_pitch(&signal, 44100.0, -1.0, 2000.0, 0.15).is_none());
        assert!(yin_pitch(&signal, 44100.0, 2000.0, 50.0, 0.15).is_none());
    }

    // --- Difference function ---

    #[test]
    fn test_difference_function_zero_lag() {
        let signal = generate_sine(440.0, 44100.0, 1024);
        let d = difference_function(&signal, 200);
        assert_eq!(d[0], 0.0, "d(0) should always be 0");
    }

    #[test]
    fn test_difference_function_sine_periodicity() {
        let fs = 8000.0;
        let freq = 200.0; // Period = 40 samples
        let signal = generate_sine(freq, fs, 512);
        let period_samples = (fs / freq).round() as usize; // 40
        let d = difference_function(&signal, period_samples + 10);

        // d(tau) should have a minimum near tau = period_samples
        let d_at_period = d[period_samples];
        // It should be small (near-zero for a perfect sine)
        let d_at_half = d[period_samples / 2];
        assert!(
            d_at_period < d_at_half,
            "d at period ({:.2}) should be less than d at half-period ({:.2})",
            d_at_period,
            d_at_half
        );
    }

    #[test]
    fn test_difference_function_length() {
        let signal = vec![1.0; 100];
        let d = difference_function(&signal, 30);
        assert_eq!(d.len(), 31); // 0..=30
    }

    // --- CMNDF ---

    #[test]
    fn test_cmndf_first_element() {
        let d = vec![0.0, 10.0, 5.0, 8.0, 2.0];
        let cmndf = cumulative_mean_normalized_difference(&d);
        assert_eq!(cmndf[0], 1.0, "CMNDF[0] should be 1.0");
    }

    #[test]
    fn test_cmndf_empty() {
        let cmndf = cumulative_mean_normalized_difference(&[]);
        assert!(cmndf.is_empty());
    }

    #[test]
    fn test_cmndf_normalization() {
        // d = [0, 4, 4, 4, 4]
        // running sum at tau=1: 4, cmndf[1] = 4*1/4 = 1.0
        // running sum at tau=2: 8, cmndf[2] = 4*2/8 = 1.0
        let d = vec![0.0, 4.0, 4.0, 4.0, 4.0];
        let cmndf = cumulative_mean_normalized_difference(&d);
        assert!((cmndf[1] - 1.0).abs() < 1e-10);
        assert!((cmndf[2] - 1.0).abs() < 1e-10);
    }

    // --- MIDI conversions ---

    #[test]
    fn test_frequency_to_midi_a4() {
        assert_eq!(frequency_to_midi(440.0), 69);
    }

    #[test]
    fn test_frequency_to_midi_c4() {
        assert_eq!(frequency_to_midi(261.63), 60);
    }

    #[test]
    fn test_frequency_to_midi_low() {
        // Very low frequency should map to low MIDI number
        let midi = frequency_to_midi(27.5); // A0
        assert_eq!(midi, 21);
    }

    #[test]
    fn test_frequency_to_midi_zero() {
        assert_eq!(frequency_to_midi(0.0), 0);
    }

    #[test]
    fn test_frequency_to_midi_negative() {
        assert_eq!(frequency_to_midi(-100.0), 0);
    }

    #[test]
    fn test_midi_to_frequency_a4() {
        let freq = midi_to_frequency(69);
        assert!((freq - 440.0).abs() < 0.01);
    }

    #[test]
    fn test_midi_to_frequency_c4() {
        let freq = midi_to_frequency(60);
        assert!((freq - 261.63).abs() < 0.1);
    }

    #[test]
    fn test_midi_to_frequency_roundtrip() {
        for midi in 21..=108 {
            let freq = midi_to_frequency(midi);
            let back = frequency_to_midi(freq);
            assert_eq!(back, midi, "Roundtrip failed for MIDI {}", midi);
        }
    }

    // --- Note names ---

    #[test]
    fn test_midi_to_note_name_a4() {
        assert_eq!(midi_to_note_name(69), "A4");
    }

    #[test]
    fn test_midi_to_note_name_c4() {
        assert_eq!(midi_to_note_name(60), "C4");
    }

    #[test]
    fn test_midi_to_note_name_c_minus1() {
        // MIDI 0 = C-1
        assert_eq!(midi_to_note_name(0), "C-1");
    }

    #[test]
    fn test_midi_to_note_name_sharps() {
        assert_eq!(midi_to_note_name(70), "A#4");
        assert_eq!(midi_to_note_name(61), "C#4");
        assert_eq!(midi_to_note_name(66), "F#4");
    }

    // --- Cents deviation ---

    #[test]
    fn test_cents_exact_a4() {
        let (midi, cents) = cents_from_nearest(440.0);
        assert_eq!(midi, 69);
        assert!(cents.abs() < 0.1, "A4 should have ~0 cents deviation");
    }

    #[test]
    fn test_cents_sharp_a4() {
        // 10 cents sharp of A4
        // freq = 440 * 2^(10/1200) = 440 * 1.00578 ~ 442.55
        let freq = 440.0 * 2.0_f64.powf(10.0 / 1200.0);
        let (midi, cents) = cents_from_nearest(freq);
        assert_eq!(midi, 69);
        assert!(
            (cents - 10.0).abs() < 0.5,
            "Expected ~10 cents sharp, got {:.2}",
            cents
        );
    }

    #[test]
    fn test_cents_flat_a4() {
        // 25 cents flat of A4
        let freq = 440.0 * 2.0_f64.powf(-25.0 / 1200.0);
        let (midi, cents) = cents_from_nearest(freq);
        assert_eq!(midi, 69);
        assert!(
            (cents - (-25.0)).abs() < 0.5,
            "Expected ~-25 cents, got {:.2}",
            cents
        );
    }

    #[test]
    fn test_cents_zero_frequency() {
        let (midi, cents) = cents_from_nearest(0.0);
        assert_eq!(midi, 0);
        assert_eq!(cents, 0.0);
    }

    // --- HPS ---

    #[test]
    fn test_hps_fundamental_detection() {
        let fs = 8000.0;
        let n = 2048;
        let fundamental = 200.0;

        // Generate signal with harmonics
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                (2.0 * PI * fundamental * t).sin()
                    + 0.5 * (2.0 * PI * 2.0 * fundamental * t).sin()
                    + 0.25 * (2.0 * PI * 3.0 * fundamental * t).sin()
            })
            .collect();

        let spec = magnitude_spectrum(&signal);
        let hps = harmonic_product_spectrum(&spec, 3);

        // Find peak in HPS
        let peak_bin = hps
            .iter()
            .enumerate()
            .skip(1) // Skip DC
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap();

        let detected_freq = peak_bin as f64 * fs / n as f64;
        assert!(
            (detected_freq - fundamental).abs() < 20.0,
            "HPS should detect fundamental ~{:.0} Hz, got {:.1}",
            fundamental,
            detected_freq
        );
    }

    #[test]
    fn test_hps_empty() {
        assert!(harmonic_product_spectrum(&[], 3).is_empty());
    }

    #[test]
    fn test_hps_zero_harmonics() {
        assert!(harmonic_product_spectrum(&[1.0, 2.0, 3.0], 0).is_empty());
    }

    #[test]
    fn test_hps_single_harmonic() {
        let input = vec![0.0, 1.0, 2.0, 3.0, 4.0, 5.0];
        let hps = harmonic_product_spectrum(&input, 1);
        assert_eq!(hps.len(), 6);
        for (i, &v) in hps.iter().enumerate() {
            assert!(
                (v - input[i]).abs() < 1e-10,
                "Single harmonic HPS should equal input"
            );
        }
    }

    // --- Chromagram ---

    #[test]
    fn test_chromagram_a4() {
        let fs = 8000.0;
        let signal = generate_sine(440.0, fs, 2048);
        let chroma = chromagram(&signal, fs, 12);
        assert_eq!(chroma.len(), 12);

        // A should be the dominant chroma bin (index 9 = A)
        let a_bin = 9;
        let max_bin = chroma
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap();
        assert_eq!(
            max_bin, a_bin,
            "A4 should produce peak at A bin (9), got bin {}",
            max_bin
        );
    }

    #[test]
    fn test_chromagram_empty() {
        let chroma = chromagram(&[], 44100.0, 12);
        assert_eq!(chroma.len(), 12);
        assert!(chroma.iter().all(|&v| v == 0.0));
    }

    #[test]
    fn test_chromagram_normalization() {
        let fs = 8000.0;
        let signal = generate_sine(440.0, fs, 2048);
        let chroma = chromagram(&signal, fs, 12);
        let max_val = chroma.iter().cloned().fold(0.0_f64, f64::max);
        assert!(
            (max_val - 1.0).abs() < 1e-10 || max_val == 0.0,
            "Chromagram should be normalized to 1.0"
        );
    }

    // --- Onset detection ---

    #[test]
    fn test_onset_detection_two_tones() {
        let fs = 8000.0;
        // Silence, then a tone, then silence, then another tone
        let mut signal = vec![0.0; 2000];
        // First tone: 440 Hz from sample 2000 to 4000
        for i in 0..2000 {
            signal.push((2.0 * PI * 440.0 * i as f64 / fs).sin() * 0.9);
        }
        // Silence
        signal.extend(vec![0.0; 2000]);
        // Second tone: 880 Hz from sample 6000 to 8000
        for i in 0..2000 {
            signal.push((2.0 * PI * 880.0 * i as f64 / fs).sin() * 0.9);
        }
        signal.extend(vec![0.0; 1000]);

        let onsets = onset_detection(&signal, fs, 256);

        // Should detect at least 1 onset (ideally 2)
        assert!(
            !onsets.is_empty(),
            "Should detect at least one onset in two-tone signal"
        );
    }

    #[test]
    fn test_onset_detection_silence() {
        let signal = vec![0.0; 8000];
        let onsets = onset_detection(&signal, 8000.0, 256);
        assert!(onsets.is_empty(), "No onsets in silence");
    }

    #[test]
    fn test_onset_detection_empty() {
        assert!(onset_detection(&[], 8000.0, 256).is_empty());
    }

    // --- A-weighting ---

    #[test]
    fn test_a_weighting_1khz() {
        // A-weighting at 1 kHz should be approximately 0 dB (reference)
        let aw = a_weighting_db(1000.0);
        assert!(
            aw.abs() < 1.0,
            "A-weighting at 1 kHz should be ~0 dB, got {:.2}",
            aw
        );
    }

    #[test]
    fn test_a_weighting_low_freq() {
        // A-weighting heavily attenuates low frequencies
        let aw_50 = a_weighting_db(50.0);
        assert!(
            aw_50 < -25.0,
            "A-weighting at 50 Hz should be < -25 dB, got {:.2}",
            aw_50
        );
    }

    #[test]
    fn test_a_weighting_high_freq() {
        // A-weighting at 10 kHz should be slightly negative
        let aw = a_weighting_db(10000.0);
        assert!(
            aw < 0.0 && aw > -10.0,
            "A-weighting at 10 kHz should be between -10 and 0 dB, got {:.2}",
            aw
        );
    }

    #[test]
    fn test_a_weighting_zero() {
        assert!(a_weighting_db(0.0).is_infinite());
    }

    #[test]
    fn test_a_weighting_negative() {
        assert!(a_weighting_db(-100.0).is_infinite());
    }

    // --- PitchDetector ---

    #[test]
    fn test_pitch_detector_a4() {
        let config = PitchConfig {
            sample_rate_hz: 44100.0,
            ..PitchConfig::default()
        };
        let detector = PitchDetector::new(config);
        let audio = generate_sine(440.0, 44100.0, 8820); // 200 ms

        let frames = detector.detect_pitch(&audio);
        assert!(!frames.is_empty(), "Should produce at least one frame");

        // Check first frame
        let f = &frames[0];
        assert!(
            (f.frequency_hz - 440.0).abs() < 10.0,
            "Detected frequency {:.1} should be near 440",
            f.frequency_hz
        );
        assert_eq!(f.midi_note, 69);
        assert_eq!(f.note_name, "A4");
    }

    #[test]
    fn test_pitch_detector_too_short() {
        let config = PitchConfig {
            sample_rate_hz: 44100.0,
            frame_size_ms: 40.0,
            ..PitchConfig::default()
        };
        let detector = PitchDetector::new(config);
        // 10 ms of audio = 441 samples, frame_size = 1764 samples
        let audio = generate_sine(440.0, 44100.0, 441);

        let frames = detector.detect_pitch(&audio);
        assert!(
            frames.is_empty(),
            "Audio shorter than frame_size should yield no frames"
        );
    }

    #[test]
    fn test_pitch_detector_config_defaults() {
        let config = PitchConfig::default();
        assert_eq!(config.sample_rate_hz, 44100.0);
        assert_eq!(config.min_frequency_hz, 50.0);
        assert_eq!(config.max_frequency_hz, 2000.0);
        assert_eq!(config.frame_size_ms, 40.0);
        assert_eq!(config.hop_size_ms, 10.0);
    }

    // --- Edge cases ---

    #[test]
    fn test_hps_output_length() {
        let spec = vec![1.0; 100];
        let hps = harmonic_product_spectrum(&spec, 4);
        assert_eq!(hps.len(), 25); // 100 / 4
    }

    #[test]
    fn test_yin_very_short_signal() {
        let signal = vec![1.0, -1.0, 1.0, -1.0];
        let result = yin_pitch(&signal, 44100.0, 50.0, 2000.0, 0.15);
        // May or may not detect pitch, but shouldn't panic
        let _ = result;
    }

    #[test]
    fn test_generate_sine_basic() {
        let signal = generate_sine(100.0, 1000.0, 100);
        assert_eq!(signal.len(), 100);
        // First sample should be ~0 (sin(0))
        assert!(signal[0].abs() < 1e-10);
    }
}
