// trace:FR-BATCH138 | ai:claude
//! # Marine Mammal Detector
//!
//! Passive Acoustic Monitoring (PAM) module for marine mammal detection and
//! classification. Used for environmental monitoring, naval operations, and
//! conservation research.
//!
//! ## Overview
//!
//! Marine mammals produce a variety of acoustic signals underwater:
//!
//! - **Echolocation clicks**: Short, broadband pulses used by odontocetes
//!   (dolphins, porpoises, beaked whales, sperm whales) for navigation and
//!   prey detection. Typical frequencies range from 10 kHz to 150+ kHz.
//!
//! - **Whistles**: Tonal, frequency-modulated signals produced by dolphins
//!   and some whales for communication. Frequencies typically 2-30 kHz.
//!
//! - **Whale songs**: Complex sequences of low-frequency calls by baleen
//!   whales (humpback, blue, fin). Frequencies range from 10 Hz to 4 kHz.
//!
//! ## Detection Pipeline
//!
//! 1. Signal conditioning (SPL calibration via hydrophone sensitivity)
//! 2. Energy detection (Teager-Kaiser Energy Operator for clicks)
//! 3. Spectral analysis (spectrogram for whistles and calls)
//! 4. Feature extraction (peak frequency, duration, inter-click interval)
//! 5. Species classification (frequency band and temporal pattern matching)
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::marine_mammal_detector::*;
//!
//! let detector = ClickDetector::new(96000.0, 10.0);
//! let signal = vec![0.0; 1000];
//! let energy = detector.teager_kaiser_energy(&signal);
//! assert_eq!(energy.len(), signal.len());
//! ```

use std::f64::consts::PI;

/// Configuration for passive acoustic monitoring.
#[derive(Debug, Clone)]
pub struct PamConfig {
    /// Sample rate in Hz (typically 96000-384000 for high-frequency clicks).
    pub sample_rate_hz: f64,
    /// Number of hydrophone channels.
    pub num_channels: usize,
    /// Hydrophone sensitivity in dB re 1V/uPa (typically around -170).
    pub hydrophone_sensitivity_db: f64,
    /// Detection threshold in dB above background noise.
    pub detection_threshold_db: f64,
}

impl PamConfig {
    /// Create a new PAM configuration.
    pub fn new(
        sample_rate_hz: f64,
        num_channels: usize,
        hydrophone_sensitivity_db: f64,
        detection_threshold_db: f64,
    ) -> Self {
        Self {
            sample_rate_hz,
            num_channels,
            hydrophone_sensitivity_db,
            detection_threshold_db,
        }
    }

    /// Default configuration for high-frequency click monitoring.
    pub fn click_monitoring() -> Self {
        Self {
            sample_rate_hz: 192000.0,
            num_channels: 1,
            hydrophone_sensitivity_db: -170.0,
            detection_threshold_db: 10.0,
        }
    }

    /// Default configuration for low-frequency whale call monitoring.
    pub fn whale_monitoring() -> Self {
        Self {
            sample_rate_hz: 2000.0,
            num_channels: 1,
            hydrophone_sensitivity_db: -170.0,
            detection_threshold_db: 6.0,
        }
    }
}

/// Marine mammal species for classification.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Species {
    /// Bottlenose dolphin (*Tursiops truncatus*). Clicks: 20-130 kHz, whistles: 2-20 kHz.
    BottlenoseDolphin,
    /// Harbor porpoise (*Phocoena phocoena*). Narrow-band HF clicks: 120-150 kHz.
    HarborPorpoise,
    /// Sperm whale (*Physeter macrocephalus*). Regular clicks: 2-30 kHz, codas.
    SpermWhale,
    /// Humpback whale (*Megaptera novaeangliae*). Songs: 80 Hz - 4 kHz.
    Humpback,
    /// Blue whale (*Balaenoptera musculus*). D-calls: 40-100 Hz, songs: 10-40 Hz.
    BlueWhale,
    /// Fin whale (*Balaenoptera physalus*). 20 Hz pulses, ~1 s duration.
    FinWhale,
    /// North Atlantic right whale (*Eubalaena glacialis*). Upcalls: 50-400 Hz.
    RightWhale,
    /// Beaked whale (Ziphiidae family). FM clicks: 20-80 kHz.
    BeakedWhale,
    /// Orca / Killer whale (*Orcinus orca*). Clicks: 12-25 kHz, whistles and pulsed calls.
    Orca,
    /// Unclassified or generic detection.
    Generic,
}

impl std::fmt::Display for Species {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Species::BottlenoseDolphin => write!(f, "Bottlenose Dolphin"),
            Species::HarborPorpoise => write!(f, "Harbor Porpoise"),
            Species::SpermWhale => write!(f, "Sperm Whale"),
            Species::Humpback => write!(f, "Humpback Whale"),
            Species::BlueWhale => write!(f, "Blue Whale"),
            Species::FinWhale => write!(f, "Fin Whale"),
            Species::RightWhale => write!(f, "Right Whale"),
            Species::BeakedWhale => write!(f, "Beaked Whale"),
            Species::Orca => write!(f, "Orca"),
            Species::Generic => write!(f, "Generic"),
        }
    }
}

/// Type of acoustic signal detected.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SignalType {
    /// Short broadband transient (echolocation click).
    Click,
    /// Tonal frequency-modulated signal (dolphin whistle).
    Whistle,
    /// Complex long-duration vocalization (baleen whale song).
    Song,
    /// Repetitive low-frequency pulse (fin whale 20 Hz pulse).
    Pulse,
    /// Low-frequency tonal call (blue whale moan).
    Moan,
}

/// A detected marine mammal acoustic event.
#[derive(Debug, Clone)]
pub struct Detection {
    /// Classified species.
    pub species: Species,
    /// Start sample index in the recording.
    pub start_sample: usize,
    /// End sample index in the recording.
    pub end_sample: usize,
    /// Peak frequency of the detection in Hz.
    pub peak_frequency_hz: f64,
    /// Received level in dB re 1 uPa (RMS).
    pub received_level_db: f64,
    /// Classification confidence (0.0 to 1.0).
    pub confidence: f64,
    /// Type of acoustic signal.
    pub signal_type: SignalType,
}

/// Click detector using Teager-Kaiser Energy Operator (TKEO).
///
/// The TKEO is particularly effective for detecting short transient signals
/// like echolocation clicks because it responds to both instantaneous
/// amplitude and instantaneous frequency. The operator is defined as:
///
/// `Psi[x(n)] = x(n)^2 - x(n-1) * x(n+1)`
///
/// This produces high output for narrowband or impulsive signals.
#[derive(Debug, Clone)]
pub struct ClickDetector {
    sample_rate: f64,
    threshold_db: f64,
}

impl ClickDetector {
    /// Create a new click detector.
    ///
    /// # Arguments
    /// * `sample_rate` - Sample rate in Hz
    /// * `threshold_db` - Detection threshold in dB above background energy
    pub fn new(sample_rate: f64, threshold_db: f64) -> Self {
        Self {
            sample_rate,
            threshold_db,
        }
    }

    /// Compute the Teager-Kaiser Energy Operator (TKEO) of a signal.
    ///
    /// TKEO: `Psi[x(n)] = x(n)^2 - x(n-1) * x(n+1)`
    ///
    /// The output has the same length as the input. The first and last samples
    /// are computed using boundary handling (replication of nearest sample).
    pub fn teager_kaiser_energy(&self, signal: &[f64]) -> Vec<f64> {
        let n = signal.len();
        if n < 3 {
            return signal.iter().map(|x| x * x).collect();
        }
        let mut energy = vec![0.0; n];
        // Boundary: first sample uses x[0] for x[-1]
        energy[0] = signal[0] * signal[0] - signal[0] * signal[1];
        for i in 1..n - 1 {
            energy[i] = signal[i] * signal[i] - signal[i - 1] * signal[i + 1];
        }
        // Boundary: last sample uses x[n-1] for x[n]
        energy[n - 1] =
            signal[n - 1] * signal[n - 1] - signal[n - 2] * signal[n - 1];
        // Ensure non-negative
        for e in &mut energy {
            if *e < 0.0 {
                *e = 0.0;
            }
        }
        energy
    }

    /// Detect clicks in a signal using energy thresholding.
    ///
    /// Returns a vector of `(sample_index, energy_value)` tuples for each
    /// detected click. Clicks closer than `min_gap_samples` apart are merged,
    /// keeping only the highest-energy peak.
    pub fn detect_clicks(
        &self,
        signal: &[f64],
        threshold: f64,
        min_gap_samples: usize,
    ) -> Vec<(usize, f64)> {
        let energy = self.teager_kaiser_energy(signal);
        let mut clicks: Vec<(usize, f64)> = Vec::new();

        // Find all samples above threshold
        let mut i = 0;
        while i < energy.len() {
            if energy[i] >= threshold {
                // Find the peak within this excursion
                let mut peak_idx = i;
                let mut peak_val = energy[i];
                let mut j = i + 1;
                while j < energy.len() && energy[j] >= threshold {
                    if energy[j] > peak_val {
                        peak_val = energy[j];
                        peak_idx = j;
                    }
                    j += 1;
                }
                clicks.push((peak_idx, peak_val));
                i = j;
            } else {
                i += 1;
            }
        }

        // Merge clicks that are too close together
        if min_gap_samples > 0 && clicks.len() > 1 {
            let mut merged: Vec<(usize, f64)> = Vec::new();
            merged.push(clicks[0]);
            for k in 1..clicks.len() {
                let last = merged.last_mut().unwrap();
                if clicks[k].0 - last.0 < min_gap_samples {
                    // Keep the higher-energy click
                    if clicks[k].1 > last.1 {
                        *last = clicks[k];
                    }
                } else {
                    merged.push(clicks[k]);
                }
            }
            return merged;
        }

        clicks
    }

    /// Calculate inter-click intervals (ICI) from click times.
    ///
    /// Returns intervals in milliseconds. ICI is a key feature for species
    /// classification:
    /// - Bottlenose dolphin: 20-100 ms
    /// - Harbor porpoise: 50-150 ms
    /// - Sperm whale: 0.5-2.0 s (regular clicks), ~35 ms (buzzes)
    pub fn click_ici(&self, click_times: &[usize], sample_rate: f64) -> Vec<f64> {
        if click_times.len() < 2 {
            return Vec::new();
        }
        click_times
            .windows(2)
            .map(|w| (w[1] - w[0]) as f64 / sample_rate * 1000.0)
            .collect()
    }

    /// Classify a click by its spectral characteristics.
    ///
    /// Uses peak frequency as the primary discriminant:
    /// - Harbor porpoise: 120-150 kHz (narrow-band high-frequency, NBHF)
    /// - Bottlenose dolphin: 20-130 kHz (broadband)
    /// - Beaked whale: 20-80 kHz (frequency-modulated upsweeps)
    /// - Sperm whale: 2-30 kHz (multipulse structure)
    /// - Orca: 12-25 kHz
    pub fn classify_click(&self, click: &[f64], sample_rate: f64) -> Species {
        let peak_freq = self.peak_frequency(click, sample_rate);
        let nyquist = sample_rate / 2.0;

        // Only classify if we can resolve the frequency
        if peak_freq > nyquist {
            return Species::Generic;
        }

        if peak_freq >= 120_000.0 && peak_freq <= 150_000.0 {
            Species::HarborPorpoise
        } else if peak_freq >= 80_000.0 && peak_freq < 120_000.0 {
            Species::BottlenoseDolphin
        } else if peak_freq >= 20_000.0 && peak_freq < 80_000.0 {
            // Could be beaked whale or dolphin; use bandwidth as tiebreaker
            let bw = self.click_bandwidth(click, sample_rate, 10.0);
            if bw < 15_000.0 {
                Species::BeakedWhale
            } else {
                Species::BottlenoseDolphin
            }
        } else if peak_freq >= 12_000.0 && peak_freq < 20_000.0 {
            Species::Orca
        } else if peak_freq >= 2_000.0 && peak_freq < 12_000.0 {
            Species::SpermWhale
        } else {
            Species::Generic
        }
    }

    /// Find the peak frequency of a signal using FFT-based power spectrum.
    ///
    /// Computes the magnitude spectrum and returns the frequency in Hz
    /// corresponding to the highest spectral peak.
    pub fn peak_frequency(&self, click: &[f64], sample_rate: f64) -> f64 {
        if click.is_empty() {
            return 0.0;
        }
        let spectrum = power_spectrum(click);
        let n = spectrum.len();
        if n == 0 {
            return 0.0;
        }
        let freq_resolution = sample_rate / (click.len() as f64);
        let mut max_idx = 0;
        let mut max_val = spectrum[0];
        // Only search up to Nyquist (first half of spectrum)
        let half = n / 2 + 1;
        for i in 1..half.min(n) {
            if spectrum[i] > max_val {
                max_val = spectrum[i];
                max_idx = i;
            }
        }
        max_idx as f64 * freq_resolution
    }

    /// Measure click duration in microseconds.
    ///
    /// Duration is measured as the time the signal envelope remains above
    /// `threshold` times the peak amplitude.
    pub fn click_duration_us(
        &self,
        click: &[f64],
        sample_rate: f64,
        threshold: f64,
    ) -> f64 {
        if click.is_empty() {
            return 0.0;
        }
        let peak = click.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
        if peak == 0.0 {
            return 0.0;
        }
        let abs_threshold = threshold * peak;
        let mut start = 0;
        let mut end = click.len() - 1;
        // Find first sample above threshold
        for (i, &s) in click.iter().enumerate() {
            if s.abs() >= abs_threshold {
                start = i;
                break;
            }
        }
        // Find last sample above threshold
        for i in (0..click.len()).rev() {
            if click[i].abs() >= abs_threshold {
                end = i;
                break;
            }
        }
        let duration_samples = if end >= start { end - start + 1 } else { 0 };
        duration_samples as f64 / sample_rate * 1_000_000.0
    }

    /// Calculate the spectral centroid frequency.
    ///
    /// The centroid is the "center of mass" of the spectrum:
    /// `f_c = sum(f_k * S_k) / sum(S_k)`
    ///
    /// # Arguments
    /// * `spectrum` - Power spectrum magnitudes
    /// * `freq_resolution` - Frequency spacing between bins in Hz
    pub fn centroid_frequency(&self, spectrum: &[f64], freq_resolution: f64) -> f64 {
        if spectrum.is_empty() {
            return 0.0;
        }
        let mut weighted_sum = 0.0;
        let mut total_power = 0.0;
        for (i, &s) in spectrum.iter().enumerate() {
            let freq = i as f64 * freq_resolution;
            weighted_sum += freq * s;
            total_power += s;
        }
        if total_power <= 0.0 {
            return 0.0;
        }
        weighted_sum / total_power
    }

    /// Estimate the -N dB bandwidth of a click.
    fn click_bandwidth(&self, click: &[f64], sample_rate: f64, db_down: f64) -> f64 {
        let spectrum = power_spectrum(click);
        let n = spectrum.len();
        if n == 0 {
            return 0.0;
        }
        let half = n / 2 + 1;
        let freq_resolution = sample_rate / (click.len() as f64);

        let max_val = spectrum[..half.min(n)]
            .iter()
            .cloned()
            .fold(0.0_f64, f64::max);
        if max_val <= 0.0 {
            return 0.0;
        }

        let threshold = max_val * 10.0_f64.powf(-db_down / 10.0);
        let mut low_idx = 0;
        let mut high_idx = 0;
        for i in 0..half.min(n) {
            if spectrum[i] >= threshold {
                low_idx = i;
                break;
            }
        }
        for i in (0..half.min(n)).rev() {
            if spectrum[i] >= threshold {
                high_idx = i;
                break;
            }
        }
        (high_idx as f64 - low_idx as f64) * freq_resolution
    }
}

/// Whistle detector for tonal frequency-modulated signals.
///
/// Dolphin whistles are typically 2-30 kHz with frequency modulation patterns
/// that are individually distinctive (signature whistles). Detection uses
/// spectrogram analysis followed by contour extraction.
#[derive(Debug, Clone)]
pub struct WhistleDetector {
    sample_rate: f64,
    min_freq_hz: f64,
    max_freq_hz: f64,
}

impl WhistleDetector {
    /// Create a new whistle detector.
    ///
    /// # Arguments
    /// * `sample_rate` - Sample rate in Hz
    /// * `min_freq_hz` - Minimum whistle frequency to detect
    /// * `max_freq_hz` - Maximum whistle frequency to detect
    pub fn new(sample_rate: f64, min_freq_hz: f64, max_freq_hz: f64) -> Self {
        Self {
            sample_rate,
            min_freq_hz,
            max_freq_hz,
        }
    }

    /// Compute a spectrogram of the input signal.
    ///
    /// Returns a 2D array where each row is the magnitude spectrum (in dB)
    /// of one time frame. Uses a Hann window for sidelobe suppression.
    ///
    /// # Arguments
    /// * `signal` - Input time-domain signal
    /// * `fft_size` - FFT length (determines frequency resolution)
    /// * `hop_size` - Samples between successive frames (determines time resolution)
    pub fn spectrogram(
        &self,
        signal: &[f64],
        fft_size: usize,
        hop_size: usize,
    ) -> Vec<Vec<f64>> {
        if signal.len() < fft_size || fft_size == 0 || hop_size == 0 {
            return Vec::new();
        }
        let num_frames = (signal.len() - fft_size) / hop_size + 1;
        let mut result = Vec::with_capacity(num_frames);

        // Pre-compute Hann window
        let window: Vec<f64> = (0..fft_size)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / fft_size as f64).cos()))
            .collect();

        for frame in 0..num_frames {
            let start = frame * hop_size;
            let end = start + fft_size;
            if end > signal.len() {
                break;
            }
            // Apply window
            let windowed: Vec<f64> = signal[start..end]
                .iter()
                .zip(window.iter())
                .map(|(&s, &w)| s * w)
                .collect();

            // Compute power spectrum
            let spec = power_spectrum(&windowed);
            // Convert to dB (only first half + DC)
            let half = fft_size / 2 + 1;
            let db_spec: Vec<f64> = spec[..half.min(spec.len())]
                .iter()
                .map(|&s| {
                    if s > 1e-30 {
                        10.0 * s.log10()
                    } else {
                        -300.0
                    }
                })
                .collect();
            result.push(db_spec);
        }

        result
    }

    /// Extract time-frequency contours from a spectrogram.
    ///
    /// A contour is a connected sequence of spectral peaks above the threshold.
    /// Each contour point is `(time_frame_index, frequency_hz)`.
    ///
    /// # Arguments
    /// * `spectrogram` - Output from `spectrogram()` (dB scale)
    /// * `threshold_db` - Minimum level in dB for peak detection
    pub fn contour_extraction(
        &self,
        spectrogram: &[Vec<f64>],
        threshold_db: f64,
    ) -> Vec<Vec<(usize, f64)>> {
        if spectrogram.is_empty() {
            return Vec::new();
        }
        let num_bins = spectrogram[0].len();
        if num_bins == 0 {
            return Vec::new();
        }
        let freq_resolution = self.sample_rate / ((num_bins - 1) as f64 * 2.0);
        let min_bin = (self.min_freq_hz / freq_resolution).ceil() as usize;
        let max_bin = ((self.max_freq_hz / freq_resolution).floor() as usize).min(num_bins - 1);

        let mut contours: Vec<Vec<(usize, f64)>> = Vec::new();
        let mut active_contour: Option<Vec<(usize, f64)>> = None;

        for (frame_idx, frame) in spectrogram.iter().enumerate() {
            // Find the peak bin in the frequency range of interest
            let mut peak_bin = min_bin;
            let mut peak_val = f64::NEG_INFINITY;
            for bin in min_bin..=max_bin.min(frame.len().saturating_sub(1)) {
                if frame[bin] > peak_val {
                    peak_val = frame[bin];
                    peak_bin = bin;
                }
            }

            if peak_val >= threshold_db {
                let freq = peak_bin as f64 * freq_resolution;
                if let Some(ref mut contour) = active_contour {
                    contour.push((frame_idx, freq));
                } else {
                    active_contour = Some(vec![(frame_idx, freq)]);
                }
            } else {
                // Break in contour
                if let Some(contour) = active_contour.take() {
                    if contour.len() >= 2 {
                        contours.push(contour);
                    }
                }
            }
        }
        // Don't forget trailing contour
        if let Some(contour) = active_contour.take() {
            if contour.len() >= 2 {
                contours.push(contour);
            }
        }

        contours
    }

    /// Calculate the duration of a whistle contour in seconds.
    ///
    /// # Arguments
    /// * `contour` - Time-frequency contour points
    /// * `hop_size` - Spectrogram hop size in samples
    /// * `sample_rate` - Sample rate in Hz
    pub fn whistle_duration(
        &self,
        contour: &[(usize, f64)],
        hop_size: usize,
        sample_rate: f64,
    ) -> f64 {
        if contour.len() < 2 {
            return 0.0;
        }
        let first_frame = contour[0].0;
        let last_frame = contour[contour.len() - 1].0;
        (last_frame - first_frame) as f64 * hop_size as f64 / sample_rate
    }

    /// Find the frequency range of a whistle contour.
    ///
    /// Returns `(min_frequency_hz, max_frequency_hz)`.
    pub fn frequency_range(&self, contour: &[(usize, f64)]) -> (f64, f64) {
        if contour.is_empty() {
            return (0.0, 0.0);
        }
        let min = contour.iter().map(|&(_, f)| f).fold(f64::INFINITY, f64::min);
        let max = contour
            .iter()
            .map(|&(_, f)| f)
            .fold(f64::NEG_INFINITY, f64::max);
        (min, max)
    }

    /// Classify a whistle by its frequency range and modulation pattern.
    ///
    /// Classification is based on known frequency ranges:
    /// - Bottlenose dolphin: 2-20 kHz, often with loops
    /// - Orca: 1-18 kHz, stereotyped clan-specific calls
    /// - Humpback: 0.08-4 kHz (not really "whistles" but tonal units)
    pub fn classify_whistle(&self, contour: &[(usize, f64)]) -> Species {
        if contour.is_empty() {
            return Species::Generic;
        }
        let (min_f, max_f) = self.frequency_range(contour);

        if max_f > 20_000.0 {
            // High frequency whistles are typically dolphins
            Species::BottlenoseDolphin
        } else if min_f >= 2_000.0 && max_f <= 20_000.0 {
            // Standard dolphin whistle range
            let bandwidth = max_f - min_f;
            if bandwidth > 8_000.0 {
                Species::BottlenoseDolphin
            } else {
                // Narrower whistles could be orca pulsed calls
                Species::Orca
            }
        } else if min_f < 2_000.0 && max_f <= 5_000.0 {
            Species::Humpback
        } else if min_f < 500.0 {
            Species::RightWhale
        } else {
            Species::Generic
        }
    }
}

/// Detector for low-frequency whale calls and songs.
///
/// Baleen whales (mysticetes) produce powerful low-frequency calls that
/// can propagate hundreds of kilometers through the deep sound channel.
/// Species-specific call types include:
///
/// - Blue whale D-calls: downswept from ~80 Hz to ~40 Hz over 1-3 seconds
/// - Fin whale 20 Hz pulses: ~1 second duration, ~10-25 s repetition
/// - Right whale upcalls: upsweeping from ~50 to ~400 Hz over ~1 second
#[derive(Debug, Clone)]
pub struct WhaleCallDetector {
    sample_rate: f64,
}

impl WhaleCallDetector {
    /// Create a new whale call detector.
    pub fn new(sample_rate: f64) -> Self {
        Self { sample_rate }
    }

    /// Detect low-frequency calls using band-pass filtered energy.
    ///
    /// Returns a list of `(start_sample, end_sample, dominant_frequency)` tuples.
    ///
    /// # Arguments
    /// * `signal` - Input signal
    /// * `min_freq_hz` - Lower band edge in Hz
    /// * `max_freq_hz` - Upper band edge in Hz
    /// * `min_duration_s` - Minimum call duration in seconds
    pub fn detect_low_frequency_call(
        &self,
        signal: &[f64],
        min_freq_hz: f64,
        max_freq_hz: f64,
        min_duration_s: f64,
    ) -> Vec<(usize, usize, f64)> {
        if signal.is_empty() {
            return Vec::new();
        }

        // Bandpass filter the signal
        let filtered = bandpass_filter(signal, min_freq_hz, max_freq_hz, self.sample_rate);

        // Compute envelope (moving RMS)
        let window_size = (self.sample_rate * 0.05).max(1.0) as usize; // 50 ms window
        let envelope = moving_rms(&filtered, window_size);

        // Threshold: 3x median level
        let mut sorted_env = envelope.clone();
        sorted_env.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median = if sorted_env.is_empty() {
            0.0
        } else {
            sorted_env[sorted_env.len() / 2]
        };
        let threshold = median * 3.0;

        let min_duration_samples = (min_duration_s * self.sample_rate) as usize;
        let mut detections = Vec::new();
        let mut in_detection = false;
        let mut det_start = 0;

        for (i, &e) in envelope.iter().enumerate() {
            if e >= threshold && !in_detection {
                in_detection = true;
                det_start = i;
            } else if e < threshold && in_detection {
                in_detection = false;
                if i - det_start >= min_duration_samples {
                    // Estimate dominant frequency
                    let segment = &filtered[det_start..i];
                    let freq = dominant_frequency(segment, self.sample_rate);
                    detections.push((det_start, i, freq));
                }
            }
        }
        // Handle detection that extends to end
        if in_detection && envelope.len() - det_start >= min_duration_samples {
            let segment = &filtered[det_start..envelope.len()];
            let freq = dominant_frequency(segment, self.sample_rate);
            detections.push((det_start, envelope.len(), freq));
        }

        detections
    }

    /// Detect blue whale D-calls (downswept ~80 Hz to ~40 Hz, 1-3 s).
    ///
    /// Uses a matched filter approach with a synthetic downsweep template.
    /// Returns a list of `(start_sample, end_sample)` for each detected call.
    pub fn blue_whale_d_call(
        &self,
        signal: &[f64],
        sample_rate: f64,
    ) -> Vec<(usize, usize)> {
        // Generate a D-call template: downsweep from 80 Hz to 40 Hz over 2 seconds
        let template = self.generate_downsweep_template(80.0, 40.0, 2.0, sample_rate);
        let corr = self.matched_filter(signal, &template);

        // Find peaks in correlation output
        let max_corr = corr.iter().cloned().fold(0.0_f64, f64::max);
        if max_corr <= 0.0 {
            return Vec::new();
        }
        let threshold = max_corr * 0.5;
        let call_len = template.len();
        let mut detections = Vec::new();

        let mut i = 0;
        while i < corr.len() {
            if corr[i] >= threshold {
                // Find peak in this cluster
                let mut peak_idx = i;
                let mut peak_val = corr[i];
                let mut j = i + 1;
                while j < corr.len() && corr[j] >= threshold * 0.5 {
                    if corr[j] > peak_val {
                        peak_val = corr[j];
                        peak_idx = j;
                    }
                    j += 1;
                }
                let start = peak_idx.saturating_sub(call_len / 2);
                let end = (peak_idx + call_len / 2).min(signal.len());
                detections.push((start, end));
                i = j;
            } else {
                i += 1;
            }
        }

        detections
    }

    /// Detect fin whale 20 Hz pulses.
    ///
    /// Fin whales produce characteristic ~20 Hz pulses that are approximately
    /// 1 second in duration, repeated every 10-25 seconds. These are among
    /// the loudest and most recognizable biological sounds in the ocean.
    pub fn fin_whale_pulse(
        &self,
        signal: &[f64],
        sample_rate: f64,
    ) -> Vec<(usize, usize)> {
        // Bandpass filter around 15-25 Hz
        let filtered = bandpass_filter(signal, 15.0, 25.0, sample_rate);

        // Envelope detection
        let window_size = (sample_rate * 0.1).max(1.0) as usize;
        let envelope = moving_rms(&filtered, window_size);

        // Threshold
        let mut sorted_env = envelope.clone();
        sorted_env.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median = if sorted_env.is_empty() {
            0.0
        } else {
            sorted_env[sorted_env.len() / 2]
        };
        let threshold = median * 4.0;

        let min_duration = (sample_rate * 0.5) as usize; // At least 0.5 s
        let max_duration = (sample_rate * 2.0) as usize; // At most 2.0 s
        let mut detections = Vec::new();
        let mut in_pulse = false;
        let mut pulse_start = 0;

        for (i, &e) in envelope.iter().enumerate() {
            if e >= threshold && !in_pulse {
                in_pulse = true;
                pulse_start = i;
            } else if e < threshold && in_pulse {
                in_pulse = false;
                let duration = i - pulse_start;
                if duration >= min_duration && duration <= max_duration {
                    detections.push((pulse_start, i));
                }
            }
        }

        detections
    }

    /// Cross-correlate a signal with a template (matched filter).
    ///
    /// Returns the normalized cross-correlation output. The peak location
    /// indicates where the template best matches the signal.
    pub fn matched_filter(&self, signal: &[f64], template: &[f64]) -> Vec<f64> {
        if signal.is_empty() || template.is_empty() || signal.len() < template.len() {
            return Vec::new();
        }
        let n = signal.len();
        let m = template.len();
        let out_len = n - m + 1;
        let mut result = vec![0.0; out_len];

        // Template energy for normalization
        let template_energy: f64 = template.iter().map(|x| x * x).sum();
        if template_energy <= 0.0 {
            return result;
        }
        let norm = template_energy.sqrt();

        for i in 0..out_len {
            let mut sum = 0.0;
            for j in 0..m {
                sum += signal[i + j] * template[j];
            }
            result[i] = sum / norm;
        }

        result
    }

    /// Generate a linear frequency downsweep template.
    ///
    /// Used for matched filtering of blue whale D-calls and similar
    /// downswept vocalizations.
    ///
    /// # Arguments
    /// * `start_freq` - Starting frequency in Hz (high)
    /// * `end_freq` - Ending frequency in Hz (low)
    /// * `duration_s` - Template duration in seconds
    /// * `sample_rate` - Sample rate in Hz
    pub fn generate_downsweep_template(
        &self,
        start_freq: f64,
        end_freq: f64,
        duration_s: f64,
        sample_rate: f64,
    ) -> Vec<f64> {
        let num_samples = (duration_s * sample_rate) as usize;
        if num_samples == 0 {
            return Vec::new();
        }
        let mut template = Vec::with_capacity(num_samples);
        for i in 0..num_samples {
            let t = i as f64 / sample_rate;
            let frac = t / duration_s;
            let freq = start_freq + (end_freq - start_freq) * frac;
            // Chirp: phase = 2 * pi * integral of f(t) dt
            // For linear sweep: phase = 2*pi * (f0*t + (f1-f0)/(2*T) * t^2)
            let phase =
                2.0 * PI * (start_freq * t + (end_freq - start_freq) / (2.0 * duration_s) * t * t);
            // Apply Hann window to taper edges
            let window =
                0.5 * (1.0 - (2.0 * PI * i as f64 / (num_samples as f64 - 1.0)).cos());
            template.push(phase.sin() * window);
            let _ = freq; // used in instantaneous frequency concept
        }
        template
    }
}

/// Acoustic metrics for sound level measurement.
///
/// Provides calibrated measurements following standard underwater acoustics
/// conventions. Reference pressure is 1 micropascal (uPa) for underwater
/// sound, compared to 20 uPa for in-air measurements.
pub struct AcousticMetrics;

impl AcousticMetrics {
    /// Calculate RMS Sound Pressure Level (SPL) in dB re 1 uPa.
    ///
    /// `SPL = 20 * log10(p_rms) - sensitivity_db`
    ///
    /// where `sensitivity_db` is the hydrophone sensitivity (dB re 1V/uPa,
    /// typically a negative number like -170).
    pub fn sound_pressure_level(signal: &[f64], sensitivity_db: f64) -> f64 {
        if signal.is_empty() {
            return f64::NEG_INFINITY;
        }
        let rms = (signal.iter().map(|x| x * x).sum::<f64>() / signal.len() as f64).sqrt();
        if rms <= 0.0 {
            return f64::NEG_INFINITY;
        }
        // Convert voltage to pressure using sensitivity
        // sensitivity_db = 20*log10(V/Pa), so Pa = V / 10^(sens/20)
        // SPL = 20*log10(Pa / 1e-6) = 20*log10(V) - sensitivity_db + 120
        // Here we treat signal values as raw ADC/voltage units
        20.0 * rms.log10() - sensitivity_db
    }

    /// Calculate peak Sound Pressure Level in dB re 1 uPa.
    pub fn peak_pressure_level(signal: &[f64], sensitivity_db: f64) -> f64 {
        if signal.is_empty() {
            return f64::NEG_INFINITY;
        }
        let peak = signal.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
        if peak <= 0.0 {
            return f64::NEG_INFINITY;
        }
        20.0 * peak.log10() - sensitivity_db
    }

    /// Calculate Sound Exposure Level (SEL) in dB re 1 uPa^2*s.
    ///
    /// SEL integrates sound energy over time:
    /// `SEL = 10 * log10(sum(p^2) / fs) - 2 * sensitivity_db`
    ///
    /// SEL is the standard metric for assessing cumulative noise exposure
    /// impacts on marine mammals (per NOAA/NMFS guidelines).
    pub fn sound_exposure_level(
        signal: &[f64],
        sensitivity_db: f64,
        sample_rate: f64,
    ) -> f64 {
        if signal.is_empty() || sample_rate <= 0.0 {
            return f64::NEG_INFINITY;
        }
        let sum_sq: f64 = signal.iter().map(|x| x * x).sum();
        let energy = sum_sq / sample_rate;
        if energy <= 0.0 {
            return f64::NEG_INFINITY;
        }
        10.0 * energy.log10() - 2.0 * sensitivity_db
    }

    /// Compute 1/3-octave band levels.
    ///
    /// Returns a vector of `(center_frequency_hz, band_level_db)` tuples
    /// for standard 1/3-octave bands within the signal bandwidth.
    pub fn third_octave_bands(signal: &[f64], sample_rate: f64) -> Vec<(f64, f64)> {
        if signal.is_empty() || sample_rate <= 0.0 {
            return Vec::new();
        }

        let spectrum = power_spectrum(signal);
        let n = signal.len();
        let freq_resolution = sample_rate / n as f64;
        let nyquist = sample_rate / 2.0;

        // Standard 1/3-octave center frequencies (ISO preferred)
        let center_freqs = [
            10.0, 12.5, 16.0, 20.0, 25.0, 31.5, 40.0, 50.0, 63.0, 80.0, 100.0,
            125.0, 160.0, 200.0, 250.0, 315.0, 400.0, 500.0, 630.0, 800.0,
            1000.0, 1250.0, 1600.0, 2000.0, 2500.0, 3150.0, 4000.0, 5000.0,
            6300.0, 8000.0, 10000.0, 12500.0, 16000.0, 20000.0, 25000.0,
            31500.0, 40000.0, 50000.0, 63000.0, 80000.0, 100000.0,
        ];

        let factor = 2.0_f64.powf(1.0 / 6.0); // Half 1/3-octave bandwidth
        let mut bands = Vec::new();

        for &fc in &center_freqs {
            if fc > nyquist {
                break;
            }
            let f_low = fc / factor;
            let f_high = fc * factor;
            if f_high > nyquist {
                continue;
            }

            let bin_low = (f_low / freq_resolution).ceil() as usize;
            let bin_high = (f_high / freq_resolution).floor() as usize;
            let half = n / 2 + 1;

            if bin_low >= half || bin_high >= half || bin_low > bin_high {
                continue;
            }

            let band_power: f64 = spectrum[bin_low..=bin_high].iter().sum();
            if band_power > 0.0 {
                let level_db = 10.0 * band_power.log10();
                bands.push((fc, level_db));
            }
        }

        bands
    }

    /// Estimate maximum detection range using simple spreading + absorption.
    ///
    /// Uses the sonar equation:
    /// `SL - TL = NL + DT`
    ///
    /// where `TL = N * log10(R) + alpha * R / 1000` (practical spreading).
    ///
    /// For simplicity this uses `TL = tl_coefficient * log10(R)`.
    ///
    /// # Arguments
    /// * `source_level_db` - Source level in dB re 1 uPa @ 1m
    /// * `noise_level_db` - Ambient noise level in dB re 1 uPa
    /// * `detection_threshold_db` - Required SNR for detection in dB
    /// * `tl_coefficient` - Transmission loss coefficient (15 = between
    ///   spherical and cylindrical, 20 = spherical spreading)
    ///
    /// Returns estimated detection range in meters.
    pub fn detection_range(
        source_level_db: f64,
        noise_level_db: f64,
        detection_threshold_db: f64,
        tl_coefficient: f64,
    ) -> f64 {
        if tl_coefficient <= 0.0 {
            return f64::INFINITY;
        }
        // SL - TL >= NL + DT
        // SL - N*log10(R) >= NL + DT
        // N*log10(R) <= SL - NL - DT
        // R <= 10^((SL - NL - DT) / N)
        let allowable_tl = source_level_db - noise_level_db - detection_threshold_db;
        if allowable_tl <= 0.0 {
            return 0.0;
        }
        10.0_f64.powf(allowable_tl / tl_coefficient)
    }
}

// ============================================================================
// Internal helper functions
// ============================================================================

/// Compute power spectrum |X(k)|^2 using DFT.
///
/// This is a straightforward O(N^2) DFT implementation for correctness.
/// For production use, a proper FFT library would be preferred.
fn power_spectrum(signal: &[f64]) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return Vec::new();
    }
    let mut spectrum = vec![0.0; n];
    for k in 0..n {
        let mut re = 0.0;
        let mut im = 0.0;
        for (i, &x) in signal.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
            re += x * angle.cos();
            im += x * angle.sin();
        }
        spectrum[k] = re * re + im * im;
    }
    spectrum
}

/// Simple bandpass filter using DFT zeroing.
///
/// Transforms to frequency domain, zeros out-of-band components,
/// and transforms back. Not phase-optimal but sufficient for detection.
fn bandpass_filter(signal: &[f64], low_hz: f64, high_hz: f64, sample_rate: f64) -> Vec<f64> {
    let n = signal.len();
    if n == 0 {
        return Vec::new();
    }
    let freq_resolution = sample_rate / n as f64;
    let low_bin = (low_hz / freq_resolution).floor() as usize;
    let high_bin = (high_hz / freq_resolution).ceil() as usize;

    // Forward DFT
    let mut re = vec![0.0; n];
    let mut im = vec![0.0; n];
    for k in 0..n {
        for (i, &x) in signal.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * i as f64 / n as f64;
            re[k] += x * angle.cos();
            im[k] += x * angle.sin();
        }
    }

    // Zero out-of-band bins (keep positive and corresponding negative freqs)
    for k in 0..n {
        let is_positive = k <= n / 2;
        let bin = if is_positive { k } else { n - k };
        if bin < low_bin || bin > high_bin {
            re[k] = 0.0;
            im[k] = 0.0;
        }
    }

    // Inverse DFT
    let mut output = vec![0.0; n];
    for i in 0..n {
        for k in 0..n {
            let angle = 2.0 * PI * k as f64 * i as f64 / n as f64;
            output[i] += re[k] * angle.cos() - im[k] * angle.sin();
        }
        output[i] /= n as f64;
    }

    output
}

/// Moving RMS (root mean square) envelope.
fn moving_rms(signal: &[f64], window_size: usize) -> Vec<f64> {
    let n = signal.len();
    if n == 0 || window_size == 0 {
        return Vec::new();
    }
    let w = window_size.min(n);
    let mut envelope = vec![0.0; n];
    let mut sum_sq = 0.0;

    // Initialize first window
    for i in 0..w {
        sum_sq += signal[i] * signal[i];
    }
    envelope[w / 2] = (sum_sq / w as f64).sqrt();

    // Slide
    for i in 1..=(n - w) {
        sum_sq -= signal[i - 1] * signal[i - 1];
        sum_sq += signal[i + w - 1] * signal[i + w - 1];
        if sum_sq < 0.0 {
            sum_sq = 0.0;
        }
        let center = i + w / 2;
        if center < n {
            envelope[center] = (sum_sq / w as f64).sqrt();
        }
    }

    envelope
}

/// Estimate dominant frequency of a signal segment.
fn dominant_frequency(signal: &[f64], sample_rate: f64) -> f64 {
    if signal.is_empty() {
        return 0.0;
    }
    let spectrum = power_spectrum(signal);
    let half = signal.len() / 2 + 1;
    let freq_resolution = sample_rate / signal.len() as f64;

    let mut max_idx = 0;
    let mut max_val = 0.0;
    for i in 1..half.min(spectrum.len()) {
        if spectrum[i] > max_val {
            max_val = spectrum[i];
            max_idx = i;
        }
    }
    max_idx as f64 * freq_resolution
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    // Helper to generate a sine wave
    fn sine_wave(freq: f64, sample_rate: f64, duration_s: f64, amplitude: f64) -> Vec<f64> {
        let n = (sample_rate * duration_s) as usize;
        (0..n)
            .map(|i| amplitude * (2.0 * PI * freq * i as f64 / sample_rate).sin())
            .collect()
    }

    // Helper to generate a click (Gaussian-windowed sinusoid)
    fn synthetic_click(
        center_freq: f64,
        sample_rate: f64,
        duration_us: f64,
        amplitude: f64,
    ) -> Vec<f64> {
        let n = (duration_us * 1e-6 * sample_rate) as usize;
        let n = n.max(16);
        let center = n as f64 / 2.0;
        let sigma = n as f64 / 6.0;
        (0..n)
            .map(|i| {
                let t = i as f64 - center;
                let gauss = (-0.5 * (t / sigma).powi(2)).exp();
                amplitude * gauss * (2.0 * PI * center_freq * i as f64 / sample_rate).sin()
            })
            .collect()
    }

    // ========================================================================
    // PamConfig tests
    // ========================================================================

    #[test]
    fn test_pam_config_new() {
        let cfg = PamConfig::new(192000.0, 2, -170.0, 10.0);
        assert_eq!(cfg.sample_rate_hz, 192000.0);
        assert_eq!(cfg.num_channels, 2);
        assert_eq!(cfg.hydrophone_sensitivity_db, -170.0);
        assert_eq!(cfg.detection_threshold_db, 10.0);
    }

    #[test]
    fn test_pam_config_click_monitoring() {
        let cfg = PamConfig::click_monitoring();
        assert!(cfg.sample_rate_hz >= 96000.0);
        assert_eq!(cfg.num_channels, 1);
    }

    #[test]
    fn test_pam_config_whale_monitoring() {
        let cfg = PamConfig::whale_monitoring();
        assert!(cfg.sample_rate_hz <= 10000.0);
    }

    // ========================================================================
    // Species and SignalType tests
    // ========================================================================

    #[test]
    fn test_species_display() {
        assert_eq!(format!("{}", Species::BottlenoseDolphin), "Bottlenose Dolphin");
        assert_eq!(format!("{}", Species::SpermWhale), "Sperm Whale");
        assert_eq!(format!("{}", Species::Generic), "Generic");
    }

    #[test]
    fn test_species_equality() {
        assert_eq!(Species::Humpback, Species::Humpback);
        assert_ne!(Species::Humpback, Species::BlueWhale);
    }

    #[test]
    fn test_signal_type_equality() {
        assert_eq!(SignalType::Click, SignalType::Click);
        assert_ne!(SignalType::Click, SignalType::Whistle);
    }

    // ========================================================================
    // TKEO tests
    // ========================================================================

    #[test]
    fn test_tkeo_detects_sharp_transients() {
        let sample_rate = 96000.0;
        let detector = ClickDetector::new(sample_rate, 10.0);

        // Quiet background with an embedded click
        let mut signal = vec![0.0; 1000];
        // Insert a sharp transient at sample 500
        signal[499] = 0.1;
        signal[500] = 1.0;
        signal[501] = -0.8;
        signal[502] = 0.1;

        let energy = detector.teager_kaiser_energy(&signal);
        assert_eq!(energy.len(), signal.len());

        // TKEO should have high energy at the transient
        let max_energy_idx = energy
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        // Peak should be near the transient (samples 499-502)
        assert!(
            max_energy_idx >= 498 && max_energy_idx <= 503,
            "Peak at {} not near transient",
            max_energy_idx
        );

        // Background energy should be near zero
        let bg_energy: f64 = energy[0..490].iter().sum::<f64>() / 490.0;
        assert!(bg_energy < energy[500] * 0.01, "Background energy too high");
    }

    #[test]
    fn test_tkeo_empty_signal() {
        let detector = ClickDetector::new(96000.0, 10.0);
        let result = detector.teager_kaiser_energy(&[]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_tkeo_short_signal() {
        let detector = ClickDetector::new(96000.0, 10.0);
        let result = detector.teager_kaiser_energy(&[1.0, 2.0]);
        assert_eq!(result.len(), 2);
    }

    #[test]
    fn test_tkeo_constant_signal() {
        let detector = ClickDetector::new(96000.0, 10.0);
        let signal = vec![1.0; 100];
        let energy = detector.teager_kaiser_energy(&signal);
        // For a constant signal, TKEO should be ~0 (x^2 - x*x = 0)
        for &e in &energy[1..energy.len() - 1] {
            assert!(e.abs() < 1e-10, "TKEO of constant should be ~0, got {}", e);
        }
    }

    // ========================================================================
    // Click detection tests
    // ========================================================================

    #[test]
    fn test_click_detection_finds_known_clicks() {
        let sample_rate = 96000.0;
        let detector = ClickDetector::new(sample_rate, 10.0);

        // Create signal with two distinct clicks
        let mut signal = vec![0.0; 5000];
        // Click 1 at sample 1000
        signal[1000] = 2.0;
        signal[1001] = -1.5;
        // Click 2 at sample 3000
        signal[3000] = 1.8;
        signal[3001] = -1.2;

        let clicks = detector.detect_clicks(&signal, 0.5, 100);
        assert!(
            clicks.len() >= 2,
            "Expected at least 2 clicks, got {}",
            clicks.len()
        );
    }

    #[test]
    fn test_click_detection_respects_min_gap() {
        let detector = ClickDetector::new(96000.0, 10.0);

        let mut signal = vec![0.0; 1000];
        // Two clicks very close together
        signal[100] = 2.0;
        signal[101] = -1.5;
        signal[110] = 1.8;
        signal[111] = -1.2;

        // With large min_gap, should merge to one
        let clicks = detector.detect_clicks(&signal, 0.5, 500);
        assert_eq!(clicks.len(), 1, "Close clicks should merge");
    }

    #[test]
    fn test_click_detection_empty_signal() {
        let detector = ClickDetector::new(96000.0, 10.0);
        let clicks = detector.detect_clicks(&[], 0.5, 100);
        assert!(clicks.is_empty());
    }

    // ========================================================================
    // ICI tests
    // ========================================================================

    #[test]
    fn test_ici_calculation() {
        let detector = ClickDetector::new(96000.0, 10.0);
        let sample_rate = 96000.0;

        // Clicks at 0 ms, 50 ms, 150 ms
        let click_times = vec![
            0,
            (0.050 * sample_rate) as usize,
            (0.150 * sample_rate) as usize,
        ];
        let ici = detector.click_ici(&click_times, sample_rate);
        assert_eq!(ici.len(), 2);
        assert!((ici[0] - 50.0).abs() < 0.1, "First ICI should be ~50 ms");
        assert!((ici[1] - 100.0).abs() < 0.1, "Second ICI should be ~100 ms");
    }

    #[test]
    fn test_ici_single_click() {
        let detector = ClickDetector::new(96000.0, 10.0);
        let ici = detector.click_ici(&[100], 96000.0);
        assert!(ici.is_empty());
    }

    // ========================================================================
    // Peak frequency tests
    // ========================================================================

    #[test]
    fn test_peak_frequency_of_tone() {
        let sample_rate = 96000.0;
        let detector = ClickDetector::new(sample_rate, 10.0);

        // 10 kHz sine wave
        let signal = sine_wave(10000.0, sample_rate, 0.01, 1.0);
        let peak_freq = detector.peak_frequency(&signal, sample_rate);

        // Should be close to 10 kHz (within frequency resolution)
        let freq_res = sample_rate / signal.len() as f64;
        assert!(
            (peak_freq - 10000.0).abs() < freq_res * 2.0,
            "Peak freq {} not near 10 kHz (resolution {})",
            peak_freq,
            freq_res
        );
    }

    #[test]
    fn test_peak_frequency_bandpass_signal() {
        let sample_rate = 96000.0;
        let detector = ClickDetector::new(sample_rate, 10.0);

        // Strong 20 kHz + weak 5 kHz
        let n = 960;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                1.0 * (2.0 * PI * 20000.0 * i as f64 / sample_rate).sin()
                    + 0.1 * (2.0 * PI * 5000.0 * i as f64 / sample_rate).sin()
            })
            .collect();

        let peak_freq = detector.peak_frequency(&signal, sample_rate);
        let freq_res = sample_rate / signal.len() as f64;
        assert!(
            (peak_freq - 20000.0).abs() < freq_res * 2.0,
            "Peak freq {} not near 20 kHz",
            peak_freq
        );
    }

    #[test]
    fn test_peak_frequency_empty() {
        let detector = ClickDetector::new(96000.0, 10.0);
        assert_eq!(detector.peak_frequency(&[], 96000.0), 0.0);
    }

    // ========================================================================
    // Click duration tests
    // ========================================================================

    #[test]
    fn test_click_duration_us() {
        let sample_rate = 192000.0;
        let detector = ClickDetector::new(sample_rate, 10.0);

        // Create a 100 us click
        let click = synthetic_click(50000.0, sample_rate, 100.0, 1.0);
        let duration = detector.click_duration_us(&click, sample_rate, 0.1);
        // Should be in the ballpark of 100 us
        assert!(
            duration > 50.0 && duration < 200.0,
            "Duration {} us not near 100 us",
            duration
        );
    }

    #[test]
    fn test_click_duration_empty() {
        let detector = ClickDetector::new(96000.0, 10.0);
        assert_eq!(detector.click_duration_us(&[], 96000.0, 0.1), 0.0);
    }

    // ========================================================================
    // Centroid frequency tests
    // ========================================================================

    #[test]
    fn test_centroid_frequency() {
        let detector = ClickDetector::new(96000.0, 10.0);

        // Spectrum with peak at bin 10 (let's say 100 Hz resolution)
        let mut spectrum = vec![0.0; 50];
        spectrum[10] = 100.0;
        spectrum[11] = 50.0;
        let centroid = detector.centroid_frequency(&spectrum, 100.0);
        // Centroid should be between 1000 and 1100 Hz
        assert!(
            centroid >= 1000.0 && centroid <= 1100.0,
            "Centroid {} not in expected range",
            centroid
        );
    }

    #[test]
    fn test_centroid_frequency_empty() {
        let detector = ClickDetector::new(96000.0, 10.0);
        assert_eq!(detector.centroid_frequency(&[], 100.0), 0.0);
    }

    // ========================================================================
    // Click classification tests
    // ========================================================================

    #[test]
    fn test_classify_click_harbor_porpoise() {
        let sample_rate = 384000.0; // Need high SR for 130 kHz
        let detector = ClickDetector::new(sample_rate, 10.0);

        let click = synthetic_click(130000.0, sample_rate, 50.0, 1.0);
        let species = detector.classify_click(&click, sample_rate);
        assert_eq!(species, Species::HarborPorpoise);
    }

    #[test]
    fn test_classify_click_sperm_whale() {
        let sample_rate = 96000.0;
        let detector = ClickDetector::new(sample_rate, 10.0);

        let click = synthetic_click(5000.0, sample_rate, 200.0, 1.0);
        let species = detector.classify_click(&click, sample_rate);
        assert_eq!(species, Species::SpermWhale);
    }

    // ========================================================================
    // Whistle detector tests
    // ========================================================================

    #[test]
    fn test_spectrogram_dimensions() {
        let sample_rate = 96000.0;
        let detector = WhistleDetector::new(sample_rate, 2000.0, 30000.0);

        let signal = sine_wave(10000.0, sample_rate, 0.1, 1.0);
        let fft_size = 256;
        let hop_size = 128;
        let spec = detector.spectrogram(&signal, fft_size, hop_size);

        let expected_frames = (signal.len() - fft_size) / hop_size + 1;
        assert_eq!(spec.len(), expected_frames);
        // Each frame should have fft_size/2 + 1 bins
        for frame in &spec {
            assert_eq!(frame.len(), fft_size / 2 + 1);
        }
    }

    #[test]
    fn test_spectrogram_empty_signal() {
        let detector = WhistleDetector::new(96000.0, 2000.0, 30000.0);
        let spec = detector.spectrogram(&[], 256, 128);
        assert!(spec.is_empty());
    }

    #[test]
    fn test_spectrogram_signal_too_short() {
        let detector = WhistleDetector::new(96000.0, 2000.0, 30000.0);
        let signal = vec![1.0; 100];
        let spec = detector.spectrogram(&signal, 256, 128);
        assert!(spec.is_empty());
    }

    #[test]
    fn test_whistle_contour_extraction() {
        let sample_rate = 96000.0;
        let detector = WhistleDetector::new(sample_rate, 5000.0, 25000.0);

        // Create a 10 kHz tone (should appear as a contour)
        let signal = sine_wave(10000.0, sample_rate, 0.05, 1.0);
        let fft_size = 256;
        let hop_size = 128;
        let spec = detector.spectrogram(&signal, fft_size, hop_size);
        let contours = detector.contour_extraction(&spec, -40.0);

        // Should find at least one contour
        assert!(
            !contours.is_empty(),
            "Should detect at least one whistle contour"
        );
    }

    #[test]
    fn test_whistle_duration() {
        let detector = WhistleDetector::new(96000.0, 2000.0, 30000.0);

        // Contour spanning frames 0 to 10 with hop_size 128 at 96 kHz
        let contour: Vec<(usize, f64)> = (0..11).map(|i| (i, 10000.0)).collect();
        let duration = detector.whistle_duration(&contour, 128, 96000.0);
        let expected = 10.0 * 128.0 / 96000.0;
        assert!(
            (duration - expected).abs() < 1e-6,
            "Duration {} != expected {}",
            duration,
            expected
        );
    }

    #[test]
    fn test_frequency_range() {
        let detector = WhistleDetector::new(96000.0, 2000.0, 30000.0);
        let contour = vec![(0, 5000.0), (1, 8000.0), (2, 15000.0), (3, 10000.0)];
        let (min_f, max_f) = detector.frequency_range(&contour);
        assert_eq!(min_f, 5000.0);
        assert_eq!(max_f, 15000.0);
    }

    #[test]
    fn test_frequency_range_empty() {
        let detector = WhistleDetector::new(96000.0, 2000.0, 30000.0);
        let (min_f, max_f) = detector.frequency_range(&[]);
        assert_eq!(min_f, 0.0);
        assert_eq!(max_f, 0.0);
    }

    #[test]
    fn test_classify_whistle_dolphin() {
        let detector = WhistleDetector::new(96000.0, 2000.0, 30000.0);
        let contour = vec![(0, 5000.0), (1, 15000.0), (2, 8000.0)];
        let species = detector.classify_whistle(&contour);
        assert_eq!(species, Species::BottlenoseDolphin);
    }

    #[test]
    fn test_classify_whistle_humpback() {
        let detector = WhistleDetector::new(96000.0, 100.0, 5000.0);
        let contour = vec![(0, 500.0), (1, 1500.0), (2, 3000.0)];
        let species = detector.classify_whistle(&contour);
        assert_eq!(species, Species::Humpback);
    }

    // ========================================================================
    // Whale call detector tests
    // ========================================================================

    #[test]
    fn test_matched_filter_peaks_at_template() {
        let detector = WhaleCallDetector::new(2000.0);
        let template = sine_wave(50.0, 2000.0, 0.1, 1.0);

        // Embed template in noise
        let mut signal = vec![0.0; 2000];
        let offset = 500;
        for (i, &t) in template.iter().enumerate() {
            if offset + i < signal.len() {
                signal[offset + i] = t;
            }
        }

        let corr = detector.matched_filter(&signal, &template);
        assert!(!corr.is_empty());

        // Peak should be near the offset
        let peak_idx = corr
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert!(
            (peak_idx as i64 - offset as i64).unsigned_abs() < 10,
            "Matched filter peak at {} not near template offset {}",
            peak_idx,
            offset
        );
    }

    #[test]
    fn test_matched_filter_empty() {
        let detector = WhaleCallDetector::new(2000.0);
        let result = detector.matched_filter(&[], &[1.0, 2.0]);
        assert!(result.is_empty());
    }

    #[test]
    fn test_downsweep_template_frequency_decreases() {
        let sample_rate = 2000.0;
        let detector = WhaleCallDetector::new(sample_rate);
        let template = detector.generate_downsweep_template(100.0, 50.0, 1.0, sample_rate);

        assert!(!template.is_empty());
        let n = template.len();
        assert_eq!(n, sample_rate as usize); // 1 second at 2000 Hz

        // Check that the template is a valid signal (not all zeros)
        let energy: f64 = template.iter().map(|x| x * x).sum();
        assert!(energy > 0.0, "Template should have non-zero energy");

        // Verify frequency content: first half should have higher frequency than second half
        // by comparing zero-crossing rates
        let first_half = &template[..n / 2];
        let second_half = &template[n / 2..];

        let count_crossings = |sig: &[f64]| -> usize {
            sig.windows(2)
                .filter(|w| (w[0] >= 0.0 && w[1] < 0.0) || (w[0] < 0.0 && w[1] >= 0.0))
                .count()
        };

        let crossings_first = count_crossings(first_half);
        let crossings_second = count_crossings(second_half);
        assert!(
            crossings_first > crossings_second,
            "First half crossings ({}) should exceed second half ({})",
            crossings_first,
            crossings_second
        );
    }

    #[test]
    fn test_blue_whale_d_call_template_in_band() {
        let sample_rate = 2000.0;
        let detector = WhaleCallDetector::new(sample_rate);
        let template = detector.generate_downsweep_template(80.0, 40.0, 2.0, sample_rate);

        // Verify the template frequency content is in the 40-80 Hz band
        let spectrum = power_spectrum(&template);
        let freq_resolution = sample_rate / template.len() as f64;
        let half = template.len() / 2 + 1;

        // Find peak frequency
        let mut max_idx = 0;
        let mut max_val = 0.0;
        for i in 1..half.min(spectrum.len()) {
            if spectrum[i] > max_val {
                max_val = spectrum[i];
                max_idx = i;
            }
        }
        let peak_freq = max_idx as f64 * freq_resolution;
        assert!(
            peak_freq >= 30.0 && peak_freq <= 90.0,
            "D-call template peak freq {} Hz not in 30-90 Hz band",
            peak_freq
        );
    }

    // ========================================================================
    // Acoustic metrics tests
    // ========================================================================

    #[test]
    fn test_spl_calculation() {
        // A sine wave with known amplitude
        let signal = sine_wave(1000.0, 48000.0, 0.1, 1.0);
        let rms = (signal.iter().map(|x| x * x).sum::<f64>() / signal.len() as f64).sqrt();
        let expected_spl = 20.0 * rms.log10() - (-170.0);

        let spl = AcousticMetrics::sound_pressure_level(&signal, -170.0);
        assert!(
            (spl - expected_spl).abs() < 0.1,
            "SPL {} != expected {}",
            spl,
            expected_spl
        );
    }

    #[test]
    fn test_spl_empty() {
        let spl = AcousticMetrics::sound_pressure_level(&[], -170.0);
        assert!(spl.is_infinite() && spl.is_sign_negative());
    }

    #[test]
    fn test_peak_pressure_level() {
        let signal = vec![0.0, 0.5, 1.0, -0.8, 0.3];
        let ppl = AcousticMetrics::peak_pressure_level(&signal, -170.0);
        let expected = 20.0 * 1.0_f64.log10() - (-170.0);
        assert!((ppl - expected).abs() < 0.01);
    }

    #[test]
    fn test_sound_exposure_level() {
        let signal = sine_wave(1000.0, 48000.0, 1.0, 1.0);
        let sel = AcousticMetrics::sound_exposure_level(&signal, -170.0, 48000.0);
        // SEL should be finite and positive
        assert!(sel.is_finite());
        assert!(sel > 0.0);
    }

    #[test]
    fn test_detection_range_increases_with_source_level() {
        let range1 = AcousticMetrics::detection_range(180.0, 100.0, 10.0, 20.0);
        let range2 = AcousticMetrics::detection_range(190.0, 100.0, 10.0, 20.0);
        assert!(
            range2 > range1,
            "Higher source level should give longer range: {} vs {}",
            range2,
            range1
        );
    }

    #[test]
    fn test_detection_range_decreases_with_noise() {
        let range1 = AcousticMetrics::detection_range(180.0, 90.0, 10.0, 20.0);
        let range2 = AcousticMetrics::detection_range(180.0, 100.0, 10.0, 20.0);
        assert!(
            range1 > range2,
            "Lower noise should give longer range: {} vs {}",
            range1,
            range2
        );
    }

    #[test]
    fn test_detection_range_zero_when_snr_insufficient() {
        // Source level below noise + threshold
        let range = AcousticMetrics::detection_range(100.0, 100.0, 10.0, 20.0);
        assert_eq!(range, 0.0);
    }

    #[test]
    fn test_third_octave_bands() {
        let sample_rate = 48000.0;
        let signal = sine_wave(1000.0, sample_rate, 0.1, 1.0);
        let bands = AcousticMetrics::third_octave_bands(&signal, sample_rate);

        // Should have some bands
        assert!(!bands.is_empty(), "Should compute at least some bands");

        // The 1000 Hz band should have the highest level
        let band_1k = bands.iter().find(|&&(f, _)| (f - 1000.0).abs() < 1.0);
        assert!(band_1k.is_some(), "Should have a 1000 Hz band");
    }

    // ========================================================================
    // Detection struct tests
    // ========================================================================

    #[test]
    fn test_detection_creation() {
        let det = Detection {
            species: Species::BottlenoseDolphin,
            start_sample: 1000,
            end_sample: 1050,
            peak_frequency_hz: 100000.0,
            received_level_db: 155.0,
            confidence: 0.85,
            signal_type: SignalType::Click,
        };
        assert_eq!(det.species, Species::BottlenoseDolphin);
        assert_eq!(det.signal_type, SignalType::Click);
        assert!(det.confidence > 0.0 && det.confidence <= 1.0);
    }

    // ========================================================================
    // Helper function tests
    // ========================================================================

    #[test]
    fn test_power_spectrum_tone() {
        let sample_rate = 1000.0;
        let signal = sine_wave(100.0, sample_rate, 0.1, 1.0);
        let spectrum = power_spectrum(&signal);

        assert_eq!(spectrum.len(), signal.len());

        // Peak should be at bin corresponding to 100 Hz
        let freq_res = sample_rate / signal.len() as f64;
        let expected_bin = (100.0 / freq_res).round() as usize;
        let half = signal.len() / 2 + 1;

        let peak_bin = spectrum[..half]
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert!(
            (peak_bin as i64 - expected_bin as i64).unsigned_abs() <= 1,
            "Peak bin {} not near expected {}",
            peak_bin,
            expected_bin
        );
    }

    #[test]
    fn test_bandpass_filter() {
        // Create signal with 50 Hz and 500 Hz components
        let sample_rate = 2000.0;
        let n = 2000;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (2.0 * PI * 50.0 * t).sin() + (2.0 * PI * 500.0 * t).sin()
            })
            .collect();

        // Bandpass around 500 Hz (pass 400-600 Hz)
        let filtered = bandpass_filter(&signal, 400.0, 600.0, sample_rate);

        // The 500 Hz component should be preserved, 50 Hz should be attenuated
        let spectrum_in = power_spectrum(&signal);
        let spectrum_out = power_spectrum(&filtered);
        let freq_res = sample_rate / n as f64;
        let bin_50 = (50.0 / freq_res).round() as usize;
        let bin_500 = (500.0 / freq_res).round() as usize;

        // After filtering, 500 Hz should be much stronger than 50 Hz
        let ratio = spectrum_out[bin_500] / (spectrum_out[bin_50] + 1e-30);
        assert!(
            ratio > 10.0,
            "500 Hz should dominate after bandpass (ratio {})",
            ratio
        );
    }

    #[test]
    fn test_moving_rms() {
        let signal = vec![1.0; 100];
        let rms = moving_rms(&signal, 10);
        assert_eq!(rms.len(), 100);
        // For a constant signal of 1.0, RMS should be 1.0
        // (at positions where the window is fully inside)
        for &r in &rms[5..95] {
            assert!(
                (r - 1.0).abs() < 0.01,
                "RMS of constant signal should be ~1.0, got {}",
                r
            );
        }
    }
}
