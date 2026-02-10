//! Real-time monophonic and polyphonic pitch tracking for musical signals.
//!
//! This module provides multiple pitch detection algorithms suitable for
//! musical signal analysis, including the YIN autocorrelation method,
//! harmonic product spectrum (HPS), and cepstral pitch estimation.
//! It also includes utilities for MIDI conversion, cent deviation,
//! onset detection, vibrato analysis, and tuning estimation.
//!
//! # Example
//!
//! ```
//! use r4w_core::music_pitch_tracker::{PitchTracker, freq_to_note_name, freq_to_midi, midi_to_freq};
//!
//! // Create a tracker for audio at 44100 Hz
//! let tracker = PitchTracker::new(44100.0, 80.0, 1000.0, 0.15);
//!
//! // Generate a pure 440 Hz sine wave (A4)
//! let sample_rate = 44100.0;
//! let frame: Vec<f64> = (0..2048)
//!     .map(|i| (2.0 * std::f64::consts::PI * 440.0 * i as f64 / sample_rate).sin())
//!     .collect();
//!
//! // Detect pitch using YIN
//! if let Some(result) = tracker.yin_pitch(&frame) {
//!     assert!((result.frequency_hz - 440.0).abs() < 5.0);
//!     assert_eq!(freq_to_note_name(440.0), "A4");
//! }
//!
//! // MIDI conversions
//! assert_eq!(freq_to_midi(440.0), 69.0);
//! assert!((midi_to_freq(69.0) - 440.0).abs() < 1e-10);
//! ```

use std::f64::consts::PI;

/// Result of a pitch detection operation.
#[derive(Debug, Clone)]
pub struct PitchResult {
    /// Detected fundamental frequency in Hz.
    pub frequency_hz: f64,
    /// Confidence value in `[0.0, 1.0]`, where 1.0 is highest confidence.
    pub confidence: f64,
    /// MIDI note number (fractional). A4 = 69.0.
    pub midi_note: f64,
    /// Deviation in cents from the nearest equal-temperament note.
    pub cent_deviation: f64,
    /// Note name string, e.g. `"A4"`, `"C#5"`.
    pub note_name: String,
}

/// Configuration and state for pitch tracking.
///
/// The `PitchTracker` holds parameters that control the range of detectable
/// pitches and the confidence threshold used by the YIN algorithm.
#[derive(Debug, Clone)]
pub struct PitchTracker {
    /// Sample rate of the audio signal in Hz.
    pub sample_rate_hz: f64,
    /// Minimum detectable frequency in Hz.
    pub min_freq_hz: f64,
    /// Maximum detectable frequency in Hz.
    pub max_freq_hz: f64,
    /// YIN absolute threshold (typical values: 0.10 to 0.20).
    pub threshold: f64,
}

impl PitchTracker {
    /// Create a new `PitchTracker` with the given parameters.
    ///
    /// # Arguments
    ///
    /// * `sample_rate_hz` - Sample rate of the input signal.
    /// * `min_freq_hz` - Lowest pitch to search for.
    /// * `max_freq_hz` - Highest pitch to search for.
    /// * `threshold` - YIN absolute threshold; lower values are stricter.
    pub fn new(sample_rate_hz: f64, min_freq_hz: f64, max_freq_hz: f64, threshold: f64) -> Self {
        Self {
            sample_rate_hz,
            min_freq_hz,
            max_freq_hz,
            threshold,
        }
    }

    /// Detect pitch using the YIN algorithm.
    ///
    /// The YIN algorithm computes a cumulative mean-normalized difference
    /// function and searches for the first dip below the configured threshold.
    /// Parabolic interpolation refines the estimate.
    ///
    /// Returns `None` if no pitch is detected with sufficient confidence.
    pub fn yin_pitch(&self, frame: &[f64]) -> Option<PitchResult> {
        let n = frame.len();
        if n < 4 {
            return None;
        }

        // Lag bounds from frequency bounds
        let tau_min = (self.sample_rate_hz / self.max_freq_hz).floor() as usize;
        let tau_max = (self.sample_rate_hz / self.min_freq_hz).ceil() as usize;
        let tau_max = tau_max.min(n / 2);

        if tau_min >= tau_max || tau_max < 2 {
            return None;
        }

        // Step 1: difference function d(tau)
        let mut diff = vec![0.0f64; tau_max + 1];
        for tau in 1..=tau_max {
            let mut sum = 0.0;
            for j in 0..(n - tau) {
                let delta = frame[j] - frame[j + tau];
                sum += delta * delta;
            }
            diff[tau] = sum;
        }

        // Step 2: cumulative mean normalized difference function d'(tau)
        let mut cmnd = vec![0.0f64; tau_max + 1];
        cmnd[0] = 1.0;
        let mut running_sum = 0.0;
        for tau in 1..=tau_max {
            running_sum += diff[tau];
            if running_sum > 0.0 {
                cmnd[tau] = diff[tau] * tau as f64 / running_sum;
            } else {
                cmnd[tau] = 1.0;
            }
        }

        // Step 3: absolute threshold -- find first tau where cmnd dips below threshold
        let mut best_tau = None;
        let search_start = tau_min.max(2);
        for tau in search_start..tau_max {
            if cmnd[tau] < self.threshold {
                // Find the local minimum in this valley
                let mut min_tau = tau;
                let mut min_val = cmnd[tau];
                let mut t = tau + 1;
                while t <= tau_max && cmnd[t] < cmnd[t - 1] {
                    if cmnd[t] < min_val {
                        min_val = cmnd[t];
                        min_tau = t;
                    }
                    t += 1;
                }
                best_tau = Some(min_tau);
                break;
            }
        }

        let best_tau = best_tau?;

        // Step 4: parabolic interpolation
        let tau_refined = if best_tau > 0 && best_tau < tau_max {
            let alpha = cmnd[best_tau - 1];
            let beta = cmnd[best_tau];
            let gamma = cmnd[best_tau + 1];
            let denom = 2.0 * (2.0 * beta - alpha - gamma);
            if denom.abs() > 1e-12 {
                best_tau as f64 + (alpha - gamma) / denom
            } else {
                best_tau as f64
            }
        } else {
            best_tau as f64
        };

        let freq = self.sample_rate_hz / tau_refined;
        let confidence = 1.0 - cmnd[best_tau].min(1.0);

        if freq < self.min_freq_hz || freq > self.max_freq_hz {
            return None;
        }

        Some(PitchResult {
            frequency_hz: freq,
            confidence,
            midi_note: freq_to_midi(freq),
            cent_deviation: cent_deviation(freq),
            note_name: freq_to_note_name(freq),
        })
    }

    /// Detect pitch using the Harmonic Product Spectrum method.
    ///
    /// Downsamples the magnitude spectrum by factors `1..=n_harmonics` and
    /// multiplies them together. The peak of the product gives the F0 estimate.
    ///
    /// Returns `None` if the frame is too short or the detected frequency
    /// is outside the configured range.
    pub fn harmonic_product_spectrum(&self, frame: &[f64], n_harmonics: usize) -> Option<f64> {
        harmonic_product_spectrum_with_rate(frame, n_harmonics, self.sample_rate_hz, self.min_freq_hz, self.max_freq_hz)
    }

    /// Detect pitch using cepstral analysis.
    ///
    /// Computes the real cepstrum and finds the peak in the expected quefrency
    /// range corresponding to `[min_freq_hz, max_freq_hz]`.
    ///
    /// Returns `None` if the frame is too short or no peak is found.
    pub fn cepstral_pitch(&self, frame: &[f64]) -> Option<f64> {
        cepstral_pitch_with_rate(frame, self.sample_rate_hz, self.min_freq_hz, self.max_freq_hz)
    }
}

// ---------------------------------------------------------------------------
// Standalone functions
// ---------------------------------------------------------------------------

/// Compute the Harmonic Product Spectrum pitch for a given frame.
///
/// Uses the configured sample rate and searches within `[min_freq_hz, max_freq_hz]`.
fn harmonic_product_spectrum_with_rate(
    frame: &[f64],
    n_harmonics: usize,
    sample_rate_hz: f64,
    min_freq_hz: f64,
    max_freq_hz: f64,
) -> Option<f64> {
    let n = frame.len();
    if n < 8 {
        return None;
    }
    let n_harmonics = n_harmonics.max(2);

    // FFT size: next power of 2
    let fft_size = n.next_power_of_two();

    // Compute magnitude spectrum via DFT (real input)
    let mag = magnitude_spectrum(frame, fft_size);
    let half = fft_size / 2;

    // Bin indices for frequency bounds
    let bin_min = ((min_freq_hz * fft_size as f64 / sample_rate_hz).ceil() as usize).max(1);
    let bin_max = ((max_freq_hz * fft_size as f64 / sample_rate_hz).floor() as usize).min(half - 1);

    if bin_min >= bin_max {
        return None;
    }

    // HPS: multiply downsampled spectra
    let mut hps = mag[bin_min..=bin_max].to_vec();
    for h in 2..=n_harmonics {
        for (i, val) in hps.iter_mut().enumerate() {
            let src_bin = (bin_min + i) * h;
            if src_bin < half {
                *val *= mag[src_bin];
            } else {
                *val = 0.0;
            }
        }
    }

    // Find peak
    let (peak_idx, _peak_val) = hps
        .iter()
        .enumerate()
        .fold((0, f64::NEG_INFINITY), |(bi, bv), (i, &v)| {
            if v > bv { (i, v) } else { (bi, bv) }
        });

    let freq = (bin_min + peak_idx) as f64 * sample_rate_hz / fft_size as f64;
    if freq >= min_freq_hz && freq <= max_freq_hz {
        Some(freq)
    } else {
        None
    }
}

/// Compute cepstral pitch estimation for a given frame.
fn cepstral_pitch_with_rate(
    frame: &[f64],
    sample_rate_hz: f64,
    min_freq_hz: f64,
    max_freq_hz: f64,
) -> Option<f64> {
    let n = frame.len();
    if n < 8 {
        return None;
    }

    let fft_size = n.next_power_of_two();

    // Compute full log-power spectrum via DFT, then real cepstrum via inverse DFT
    // We need the full N-point DFT for the cepstrum to work correctly.
    let mut log_power = vec![0.0f64; fft_size];
    for k in 0..fft_size {
        let mut re = 0.0;
        let mut im = 0.0;
        let len = n.min(fft_size);
        for j in 0..len {
            let angle = -2.0 * PI * k as f64 * j as f64 / fft_size as f64;
            re += frame[j] * angle.cos();
            im += frame[j] * angle.sin();
        }
        let mag_sq = re * re + im * im;
        log_power[k] = (mag_sq + 1e-20).ln();
    }

    // Inverse DFT of log-power -> real cepstrum (full N-point)
    let mut cepstrum = vec![0.0f64; fft_size];
    for q in 0..fft_size {
        let mut sum = 0.0;
        for k in 0..fft_size {
            let angle = 2.0 * PI * q as f64 * k as f64 / fft_size as f64;
            sum += log_power[k] * angle.cos();
        }
        cepstrum[q] = sum / fft_size as f64;
    }

    // Quefrency range from frequency bounds
    let q_min = (sample_rate_hz / max_freq_hz).floor() as usize;
    let q_max = (sample_rate_hz / min_freq_hz).ceil() as usize;
    let q_max = q_max.min(fft_size / 2);
    let q_min = q_min.max(1);

    if q_min >= q_max {
        return None;
    }

    // Find peak in quefrency range
    let mut best_q = q_min;
    let mut best_val = f64::NEG_INFINITY;
    for q in q_min..=q_max.min(fft_size - 1) {
        if cepstrum[q] > best_val {
            best_val = cepstrum[q];
            best_q = q;
        }
    }

    let freq = sample_rate_hz / best_q as f64;
    if freq >= min_freq_hz && freq <= max_freq_hz {
        Some(freq)
    } else {
        None
    }
}

/// Standalone harmonic product spectrum function (uses default 44100 Hz, 50-4000 Hz range).
pub fn harmonic_product_spectrum(frame: &[f64], n_harmonics: usize) -> Option<f64> {
    harmonic_product_spectrum_with_rate(frame, n_harmonics, 44100.0, 50.0, 4000.0)
}

/// Standalone cepstral pitch function (uses default 44100 Hz, 50-4000 Hz range).
pub fn cepstral_pitch(frame: &[f64]) -> Option<f64> {
    cepstral_pitch_with_rate(frame, 44100.0, 50.0, 4000.0)
}

/// Convert a frequency in Hz to a fractional MIDI note number.
///
/// A4 (440 Hz) maps to MIDI note 69.
///
/// # Example
///
/// ```
/// use r4w_core::music_pitch_tracker::freq_to_midi;
/// assert_eq!(freq_to_midi(440.0), 69.0);
/// ```
pub fn freq_to_midi(freq_hz: f64) -> f64 {
    69.0 + 12.0 * (freq_hz / 440.0).log2()
}

/// Convert a fractional MIDI note number to frequency in Hz.
///
/// MIDI note 69 maps to 440 Hz (A4).
///
/// # Example
///
/// ```
/// use r4w_core::music_pitch_tracker::midi_to_freq;
/// assert!((midi_to_freq(69.0) - 440.0).abs() < 1e-10);
/// ```
pub fn midi_to_freq(midi_note: f64) -> f64 {
    440.0 * 2.0_f64.powf((midi_note - 69.0) / 12.0)
}

/// Map a frequency to the nearest note name in scientific pitch notation.
///
/// Accidentals are represented with `#` (sharp).
///
/// # Example
///
/// ```
/// use r4w_core::music_pitch_tracker::freq_to_note_name;
/// assert_eq!(freq_to_note_name(440.0), "A4");
/// assert_eq!(freq_to_note_name(261.63), "C4");
/// ```
pub fn freq_to_note_name(freq_hz: f64) -> String {
    const NOTE_NAMES: [&str; 12] = [
        "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", "A#", "B",
    ];
    let midi = freq_to_midi(freq_hz).round() as i32;
    let note_index = ((midi % 12) + 12) % 12;
    let octave = (midi / 12) - 1;
    format!("{}{}", NOTE_NAMES[note_index as usize], octave)
}

/// Calculate the deviation in cents from the nearest equal-temperament note.
///
/// The result is in the range `(-50, 50]`.
///
/// # Example
///
/// ```
/// use r4w_core::music_pitch_tracker::cent_deviation;
/// assert!((cent_deviation(440.0)).abs() < 1e-10);
/// ```
pub fn cent_deviation(freq_hz: f64) -> f64 {
    let midi = freq_to_midi(freq_hz);
    let nearest = midi.round();
    (midi - nearest) * 100.0
}

/// Detect note onsets using spectral flux with half-wave rectification.
///
/// Returns a vector of sample indices where onsets are detected.
/// The `hop_size` parameter controls the analysis resolution.
///
/// # Arguments
///
/// * `signal` - The input audio signal.
/// * `hop_size` - Number of samples between analysis frames.
pub fn detect_onsets(signal: &[f64], hop_size: usize) -> Vec<usize> {
    if signal.len() < hop_size * 2 || hop_size == 0 {
        return Vec::new();
    }

    let fft_size = hop_size.next_power_of_two() * 2;
    let num_frames = (signal.len().saturating_sub(fft_size)) / hop_size + 1;

    if num_frames < 2 {
        return Vec::new();
    }

    // Compute magnitude spectrum for each frame
    let mut spectra: Vec<Vec<f64>> = Vec::with_capacity(num_frames);
    for i in 0..num_frames {
        let start = i * hop_size;
        let end = (start + fft_size).min(signal.len());
        let frame = &signal[start..end];
        spectra.push(magnitude_spectrum(frame, fft_size));
    }

    // Spectral flux with half-wave rectification
    let mut flux = vec![0.0f64; num_frames];
    for i in 1..num_frames {
        let mut sf = 0.0;
        let half = fft_size / 2;
        for k in 0..half.min(spectra[i].len()).min(spectra[i - 1].len()) {
            let diff = spectra[i][k] - spectra[i - 1][k];
            if diff > 0.0 {
                sf += diff;
            }
        }
        flux[i] = sf;
    }

    // Adaptive threshold: median + 1.5 * std_dev
    let mut sorted_flux = flux.clone();
    sorted_flux.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let median = sorted_flux[sorted_flux.len() / 2];
    let mean = flux.iter().sum::<f64>() / flux.len() as f64;
    let variance = flux.iter().map(|&v| (v - mean) * (v - mean)).sum::<f64>() / flux.len() as f64;
    let std_dev = variance.sqrt();
    let threshold = median + 1.5 * std_dev;

    // Peak pick above threshold
    let mut onsets = Vec::new();
    for i in 1..(num_frames - 1) {
        if flux[i] > threshold && flux[i] > flux[i - 1] && flux[i] >= flux[i + 1] {
            onsets.push(i * hop_size);
        }
    }

    onsets
}

/// Analyze vibrato characteristics from a pitch track.
///
/// Returns `(rate_hz, extent_cents)`:
/// - `rate_hz`: vibrato rate in Hz, estimated from autocorrelation of the pitch track.
/// - `extent_cents`: vibrato extent as peak deviation in cents.
///
/// # Arguments
///
/// * `pitch_track` - Sequence of pitch values in Hz (one per analysis frame).
/// * `sample_rate` - The rate at which pitch values are sampled (frames per second).
pub fn detect_vibrato(pitch_track: &[f64], sample_rate: f64) -> (f64, f64) {
    if pitch_track.len() < 4 || sample_rate <= 0.0 {
        return (0.0, 0.0);
    }

    // Convert pitch track to cent deviations from mean
    let mean_pitch = pitch_track.iter().sum::<f64>() / pitch_track.len() as f64;
    if mean_pitch <= 0.0 {
        return (0.0, 0.0);
    }

    let cent_track: Vec<f64> = pitch_track
        .iter()
        .map(|&f| 1200.0 * (f / mean_pitch).log2())
        .collect();

    // Vibrato extent: RMS of cent deviations * sqrt(2) for peak
    let rms = (cent_track.iter().map(|&c| c * c).sum::<f64>() / cent_track.len() as f64).sqrt();
    let extent_cents = rms * std::f64::consts::SQRT_2;

    // Vibrato rate via autocorrelation
    // Search for first peak after zero crossing in autocorrelation
    let n = cent_track.len();
    let max_lag = n / 2;

    // Subtract mean from cent track
    let cent_mean = cent_track.iter().sum::<f64>() / n as f64;
    let centered: Vec<f64> = cent_track.iter().map(|&c| c - cent_mean).collect();

    let mut acf = vec![0.0; max_lag];
    let acf0: f64 = centered.iter().map(|&x| x * x).sum();
    if acf0 < 1e-20 {
        return (0.0, 0.0);
    }

    for lag in 1..max_lag {
        let mut sum = 0.0;
        for i in 0..(n - lag) {
            sum += centered[i] * centered[i + lag];
        }
        acf[lag] = sum / acf0;
    }

    // Typical vibrato: 4-8 Hz. Search for first positive peak after zero crossing.
    // Lag range corresponding to 3-12 Hz vibrato
    let lag_min = (sample_rate / 12.0).floor() as usize;
    let lag_max = (sample_rate / 3.0).ceil() as usize;
    let lag_min = lag_min.max(1);
    let lag_max = lag_max.min(max_lag - 1);

    if lag_min >= lag_max {
        return (0.0, extent_cents);
    }

    let mut best_lag = lag_min;
    let mut best_val = f64::NEG_INFINITY;
    for lag in lag_min..=lag_max {
        if acf[lag] > best_val {
            best_val = acf[lag];
            best_lag = lag;
        }
    }

    let rate_hz = if best_val > 0.1 {
        sample_rate / best_lag as f64
    } else {
        0.0
    };

    (rate_hz, extent_cents)
}

/// Estimate the A4 reference tuning from a detected pitch.
///
/// Given a detected frequency presumed to be some note, this calculates what
/// A4 tuning would produce that frequency for the nearest MIDI note.
///
/// # Example
///
/// ```
/// use r4w_core::music_pitch_tracker::a4_tuning;
/// let tuning = a4_tuning(440.0);
/// assert!((tuning - 440.0).abs() < 0.01);
/// ```
pub fn a4_tuning(reference_hz: f64) -> f64 {
    let midi = freq_to_midi(reference_hz).round();
    // A4 tuning = reference_hz / 2^((midi - 69) / 12)
    reference_hz / 2.0_f64.powf((midi - 69.0) / 12.0)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Compute magnitude spectrum of a real signal using DFT.
/// Zero-pads to `fft_size` if needed.
fn magnitude_spectrum(signal: &[f64], fft_size: usize) -> Vec<f64> {
    let half = fft_size / 2 + 1;
    let mut mag = vec![0.0; half];

    for k in 0..half {
        let mut re = 0.0;
        let mut im = 0.0;
        let len = signal.len().min(fft_size);
        for n in 0..len {
            let angle = -2.0 * PI * k as f64 * n as f64 / fft_size as f64;
            re += signal[n] * angle.cos();
            im += signal[n] * angle.sin();
        }
        mag[k] = (re * re + im * im).sqrt();
    }
    mag
}

/// Compute an inverse DFT of a real-valued spectrum, returning the real part.
fn inverse_dft_real(spectrum: &[f64]) -> Vec<f64> {
    let n = spectrum.len();
    let mut result = vec![0.0; n];
    for k in 0..n {
        let mut sum = 0.0;
        for (m, &val) in spectrum.iter().enumerate() {
            let angle = 2.0 * PI * k as f64 * m as f64 / n as f64;
            sum += val * angle.cos();
        }
        result[k] = sum / n as f64;
    }
    result
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    /// Helper: generate a sine wave.
    fn sine_wave(freq: f64, sample_rate: f64, n_samples: usize) -> Vec<f64> {
        (0..n_samples)
            .map(|i| (2.0 * PI * freq * i as f64 / sample_rate).sin())
            .collect()
    }

    /// Helper: generate a sine wave with harmonics.
    fn harmonic_signal(fundamental: f64, sample_rate: f64, n_samples: usize, harmonics: &[(f64, f64)]) -> Vec<f64> {
        let mut signal = sine_wave(fundamental, sample_rate, n_samples);
        for &(harmonic_num, amplitude) in harmonics {
            for i in 0..n_samples {
                signal[i] += amplitude * (2.0 * PI * fundamental * harmonic_num * i as f64 / sample_rate).sin();
            }
        }
        signal
    }

    #[test]
    fn test_freq_to_midi_a4() {
        assert!((freq_to_midi(440.0) - 69.0).abs() < 1e-10);
    }

    #[test]
    fn test_freq_to_midi_c4() {
        // C4 = MIDI 60, freq ~261.63 Hz
        let c4_freq = midi_to_freq(60.0);
        assert!((freq_to_midi(c4_freq) - 60.0).abs() < 1e-10);
    }

    #[test]
    fn test_midi_to_freq_a4() {
        assert!((midi_to_freq(69.0) - 440.0).abs() < 1e-10);
    }

    #[test]
    fn test_midi_to_freq_c4() {
        let c4 = midi_to_freq(60.0);
        assert!((c4 - 261.6255653005986).abs() < 0.01);
    }

    #[test]
    fn test_midi_freq_roundtrip() {
        for midi in 21..=108 {
            let freq = midi_to_freq(midi as f64);
            let back = freq_to_midi(freq);
            assert!((back - midi as f64).abs() < 1e-10, "Roundtrip failed for MIDI {}", midi);
        }
    }

    #[test]
    fn test_freq_to_note_name_a4() {
        assert_eq!(freq_to_note_name(440.0), "A4");
    }

    #[test]
    fn test_freq_to_note_name_c4() {
        assert_eq!(freq_to_note_name(261.63), "C4");
    }

    #[test]
    fn test_freq_to_note_name_sharps() {
        // C#4 = MIDI 61
        let freq = midi_to_freq(61.0);
        assert_eq!(freq_to_note_name(freq), "C#4");
    }

    #[test]
    fn test_freq_to_note_name_various() {
        assert_eq!(freq_to_note_name(midi_to_freq(48.0)), "C3");
        assert_eq!(freq_to_note_name(midi_to_freq(72.0)), "C5");
        assert_eq!(freq_to_note_name(midi_to_freq(79.0)), "G5");
        assert_eq!(freq_to_note_name(midi_to_freq(64.0)), "E4");
    }

    #[test]
    fn test_cent_deviation_exact() {
        assert!((cent_deviation(440.0)).abs() < 1e-10);
        assert!((cent_deviation(midi_to_freq(60.0))).abs() < 1e-10);
    }

    #[test]
    fn test_cent_deviation_sharp() {
        // 10 cents sharp of A4
        let freq = 440.0 * 2.0_f64.powf(10.0 / 1200.0);
        assert!((cent_deviation(freq) - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_cent_deviation_flat() {
        // 20 cents flat of A4
        let freq = 440.0 * 2.0_f64.powf(-20.0 / 1200.0);
        assert!((cent_deviation(freq) - (-20.0)).abs() < 0.01);
    }

    #[test]
    fn test_a4_tuning_standard() {
        let tuning = a4_tuning(440.0);
        assert!((tuning - 440.0).abs() < 0.01);
    }

    #[test]
    fn test_a4_tuning_from_other_note() {
        // If we play a C4 at standard tuning, a4_tuning should return ~440
        let c4 = midi_to_freq(60.0); // 261.63 Hz
        let tuning = a4_tuning(c4);
        assert!((tuning - 440.0).abs() < 0.1, "Expected ~440, got {}", tuning);
    }

    #[test]
    fn test_a4_tuning_442() {
        // A4 played at 442 Hz -- tuning should be 442
        let tuning = a4_tuning(442.0);
        assert!((tuning - 442.0).abs() < 0.1, "Expected ~442, got {}", tuning);
    }

    #[test]
    fn test_yin_pitch_440hz() {
        let sr = 44100.0;
        let signal = sine_wave(440.0, sr, 4096);
        let tracker = PitchTracker::new(sr, 80.0, 1000.0, 0.20);
        let result = tracker.yin_pitch(&signal).expect("Should detect 440 Hz");
        assert!(
            (result.frequency_hz - 440.0).abs() < 5.0,
            "Expected ~440 Hz, got {} Hz",
            result.frequency_hz
        );
        assert!(result.confidence > 0.5, "Confidence should be high");
        assert_eq!(result.note_name, "A4");
    }

    #[test]
    fn test_yin_pitch_220hz() {
        let sr = 44100.0;
        let signal = sine_wave(220.0, sr, 4096);
        let tracker = PitchTracker::new(sr, 80.0, 1000.0, 0.20);
        let result = tracker.yin_pitch(&signal).expect("Should detect 220 Hz");
        assert!(
            (result.frequency_hz - 220.0).abs() < 5.0,
            "Expected ~220 Hz, got {} Hz",
            result.frequency_hz
        );
    }

    #[test]
    fn test_yin_pitch_with_harmonics() {
        let sr = 44100.0;
        let signal = harmonic_signal(330.0, sr, 4096, &[(2.0, 0.5), (3.0, 0.25)]);
        let tracker = PitchTracker::new(sr, 80.0, 1000.0, 0.20);
        let result = tracker.yin_pitch(&signal).expect("Should detect fundamental");
        assert!(
            (result.frequency_hz - 330.0).abs() < 10.0,
            "Expected ~330 Hz, got {} Hz",
            result.frequency_hz
        );
    }

    #[test]
    fn test_yin_silence_returns_none() {
        let signal = vec![0.0; 4096];
        let tracker = PitchTracker::new(44100.0, 80.0, 1000.0, 0.15);
        assert!(tracker.yin_pitch(&signal).is_none());
    }

    #[test]
    fn test_yin_short_frame_returns_none() {
        let signal = vec![1.0, 0.0, -1.0];
        let tracker = PitchTracker::new(44100.0, 80.0, 1000.0, 0.15);
        assert!(tracker.yin_pitch(&signal).is_none());
    }

    #[test]
    fn test_hps_440hz() {
        let sr = 44100.0;
        let signal = harmonic_signal(440.0, sr, 4096, &[(2.0, 0.6), (3.0, 0.3), (4.0, 0.15)]);
        let tracker = PitchTracker::new(sr, 80.0, 2000.0, 0.15);
        let result = tracker.harmonic_product_spectrum(&signal, 4);
        assert!(result.is_some(), "HPS should detect a pitch");
        let freq = result.unwrap();
        assert!(
            (freq - 440.0).abs() < 20.0,
            "Expected ~440 Hz, got {} Hz",
            freq
        );
    }

    #[test]
    fn test_cepstral_pitch_440hz() {
        let sr = 44100.0;
        let signal = harmonic_signal(440.0, sr, 4096, &[(2.0, 0.5), (3.0, 0.25)]);
        let tracker = PitchTracker::new(sr, 80.0, 2000.0, 0.15);
        let result = tracker.cepstral_pitch(&signal);
        assert!(result.is_some(), "Cepstral should detect a pitch");
        let freq = result.unwrap();
        assert!(
            (freq - 440.0).abs() < 20.0,
            "Expected ~440 Hz, got {} Hz",
            freq
        );
    }

    #[test]
    fn test_detect_onsets_simple() {
        let sr = 44100.0;
        let hop = 512;

        // Silence for 0.1s, then a tone, then silence, then another tone
        let silence_len = (sr * 0.1) as usize;
        let tone_len = (sr * 0.1) as usize;

        let mut signal = vec![0.0f64; silence_len];
        signal.extend(sine_wave(440.0, sr, tone_len));
        signal.extend(vec![0.0f64; silence_len]);
        signal.extend(sine_wave(880.0, sr, tone_len));
        signal.extend(vec![0.0f64; silence_len]);

        let onsets = detect_onsets(&signal, hop);
        // We expect at least 1 onset (depending on threshold)
        // The exact count depends on the adaptive threshold
        assert!(
            !onsets.is_empty() || signal.len() < hop * 4,
            "Should detect at least one onset (got {})",
            onsets.len()
        );
    }

    #[test]
    fn test_detect_onsets_empty() {
        let onsets = detect_onsets(&[], 512);
        assert!(onsets.is_empty());
    }

    #[test]
    fn test_detect_vibrato_synthetic() {
        // Synthesize a pitch track with 6 Hz vibrato, +/-50 cents
        let frames_per_sec = 100.0;
        let duration = 2.0; // seconds
        let n = (frames_per_sec * duration) as usize;
        let vibrato_rate = 6.0;
        let vibrato_extent_cents = 50.0;

        let base_freq = 440.0;
        let pitch_track: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / frames_per_sec;
                let cent_offset = vibrato_extent_cents * (2.0 * PI * vibrato_rate * t).sin();
                base_freq * 2.0_f64.powf(cent_offset / 1200.0)
            })
            .collect();

        let (rate, extent) = detect_vibrato(&pitch_track, frames_per_sec);
        assert!(
            (rate - vibrato_rate).abs() < 1.5,
            "Expected vibrato rate ~{} Hz, got {} Hz",
            vibrato_rate,
            rate
        );
        assert!(
            extent > 10.0,
            "Expected significant vibrato extent, got {} cents",
            extent
        );
    }

    #[test]
    fn test_detect_vibrato_no_vibrato() {
        // Constant pitch -- should have near-zero extent
        let pitch_track = vec![440.0; 200];
        let (rate, extent) = detect_vibrato(&pitch_track, 100.0);
        assert!(extent < 1.0, "Constant pitch should have ~0 extent, got {}", extent);
        // Rate is meaningless when extent is 0, so we don't check it strictly
        let _ = rate;
    }

    #[test]
    fn test_pitch_result_fields() {
        let sr = 44100.0;
        let signal = sine_wave(440.0, sr, 4096);
        let tracker = PitchTracker::new(sr, 80.0, 1000.0, 0.20);
        let result = tracker.yin_pitch(&signal).expect("Should detect pitch");
        // Check that all fields are populated
        assert!(result.frequency_hz > 0.0);
        assert!(result.confidence >= 0.0 && result.confidence <= 1.0);
        assert!(result.midi_note > 0.0);
        assert!(result.cent_deviation.abs() < 100.0);
        assert!(!result.note_name.is_empty());
    }

    #[test]
    fn test_pitch_tracker_new() {
        let t = PitchTracker::new(48000.0, 50.0, 2000.0, 0.10);
        assert_eq!(t.sample_rate_hz, 48000.0);
        assert_eq!(t.min_freq_hz, 50.0);
        assert_eq!(t.max_freq_hz, 2000.0);
        assert_eq!(t.threshold, 0.10);
    }
}
