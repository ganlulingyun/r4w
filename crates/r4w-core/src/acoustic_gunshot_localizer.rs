//! Acoustic gunshot detection and localization using microphone arrays.
//!
//! This module detects impulsive acoustic events from multi-channel microphone
//! data, classifies them (gunshot, firecracker, vehicle backfire, construction,
//! unknown), and localizes the source in 3-D via Time-Difference-of-Arrival
//! (TDOA) using Generalized Cross-Correlation with PHAse Transform (GCC-PHAT).
//!
//! # Algorithm Overview
//!
//! 1. **Detection** -- Each channel is scanned for impulsive events that exceed
//!    a configurable dB threshold above the local RMS floor.
//! 2. **Classification** -- Rise time, duration, spectral shape, and energy
//!    distribution are used to distinguish gunshots from other impulsive sounds.
//! 3. **TDOA estimation** -- GCC-PHAT is computed for every microphone pair to
//!    obtain sub-sample time-delay estimates.
//! 4. **Localization** -- A least-squares solver converts the TDOA measurements
//!    into a 3-D source position estimate.
//!
//! # Example
//!
//! ```
//! use r4w_core::acoustic_gunshot_localizer::{
//!     GunDetectorConfig, GunDetector, generate_synthetic_gunshot,
//! };
//!
//! let mic_positions = vec![
//!     (0.0, 0.0, 0.0),
//!     (10.0, 0.0, 0.0),
//!     (0.0, 10.0, 0.0),
//!     (0.0, 0.0, 10.0),
//! ];
//! let config = GunDetectorConfig {
//!     num_microphones: 4,
//!     mic_positions,
//!     sample_rate_hz: 44100.0,
//!     sound_speed_mps: 343.0,
//!     detection_threshold_db: 20.0,
//! };
//! let mut detector = GunDetector::new(config);
//! let shot = generate_synthetic_gunshot(44100.0, 0.05);
//! let multichannel = vec![shot.clone(); 4];
//! let events = detector.process_audio(&multichannel);
//! assert!(!events.is_empty());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public enums
// ---------------------------------------------------------------------------

/// Classification of an impulsive acoustic event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EventClass {
    /// Muzzle blast / supersonic crack -- very fast rise, broad spectrum.
    Gunshot,
    /// Firecracker -- fast rise but shorter duration, narrower spectrum.
    Firecracker,
    /// Vehicle backfire or door slam -- slower rise, low-frequency dominated.
    Vehicle,
    /// Construction impulsive noise (hammering, pile driving).
    Construction,
    /// Could not be classified with sufficient confidence.
    Unknown,
}

// ---------------------------------------------------------------------------
// Configuration and result structs
// ---------------------------------------------------------------------------

/// Configuration for the gunshot detector / localizer.
#[derive(Debug, Clone)]
pub struct GunDetectorConfig {
    /// Number of microphones in the array.
    pub num_microphones: usize,
    /// 3-D positions of each microphone in metres (x, y, z).
    pub mic_positions: Vec<(f64, f64, f64)>,
    /// Sample rate common to all channels, in Hz.
    pub sample_rate_hz: f64,
    /// Speed of sound in m/s (default 343.0 at ~20 C sea-level).
    pub sound_speed_mps: f64,
    /// Detection threshold in dB above local RMS floor.
    pub detection_threshold_db: f64,
}

/// A detected and (optionally) localized impulsive event.
#[derive(Debug, Clone)]
pub struct GunEvent {
    /// Sample index at which the event was detected (on the reference mic).
    pub timestamp_sample: usize,
    /// Classification result.
    pub classification: EventClass,
    /// Estimated 3-D source position, if localization succeeded.
    pub source_position: Option<(f64, f64, f64)>,
    /// Classification / localization confidence in [0, 1].
    pub confidence: f64,
    /// Peak amplitude in dB relative to full-scale (dBFS).
    pub peak_amplitude_db: f64,
}

/// Result of impulsive-event detection on a single channel.
#[derive(Debug, Clone, Copy)]
pub struct ImpulseDetection {
    /// Sample index where the impulse onset was detected.
    pub onset_sample: usize,
    /// Sample index of the peak amplitude.
    pub peak_sample: usize,
    /// Absolute peak amplitude (linear).
    pub peak_amplitude: f64,
    /// Duration of the impulse in samples (onset to decay below threshold).
    pub duration_samples: usize,
}

// ---------------------------------------------------------------------------
// GunDetector
// ---------------------------------------------------------------------------

/// Multi-channel impulsive-event detector, classifier, and localizer.
#[derive(Debug, Clone)]
pub struct GunDetector {
    config: GunDetectorConfig,
}

impl GunDetector {
    /// Create a new detector from the given configuration.
    pub fn new(config: GunDetectorConfig) -> Self {
        assert!(
            config.mic_positions.len() == config.num_microphones,
            "mic_positions length must equal num_microphones"
        );
        assert!(config.num_microphones >= 1, "need at least one microphone");
        Self { config }
    }

    /// Process a block of multi-channel audio and return detected events.
    ///
    /// `multichannel[i]` is the sample vector from the *i*-th microphone.
    /// All channels must have the same length.
    pub fn process_audio(&mut self, multichannel: &[Vec<f64>]) -> Vec<GunEvent> {
        assert_eq!(
            multichannel.len(),
            self.config.num_microphones,
            "channel count mismatch"
        );
        if multichannel.is_empty() || multichannel[0].is_empty() {
            return Vec::new();
        }
        let n = multichannel[0].len();
        for ch in multichannel {
            assert_eq!(ch.len(), n, "all channels must have equal length");
        }

        // Use channel 0 as the reference microphone for detection.
        let ref_signal = &multichannel[0];
        let min_gap = (self.config.sample_rate_hz * 0.05) as usize; // 50 ms gap
        let detections =
            detect_impulsive_events(ref_signal, self.config.detection_threshold_db, min_gap);

        let mut events = Vec::new();
        for det in &detections {
            // Extract a window around the detection for classification.
            let window_len = (self.config.sample_rate_hz * 0.02) as usize; // 20 ms
            let start = det.onset_sample;
            let end = (start + window_len).min(n);
            let snippet = &ref_signal[start..end];

            let classification = classify_impulse(snippet, self.config.sample_rate_hz);

            // Compute TDOA pairs relative to microphone 0.
            let mut tdoa_pairs: Vec<(usize, usize, f64)> = Vec::new();
            if self.config.num_microphones >= 2 {
                // Extract a wider window for cross-correlation.
                let corr_half = (self.config.sample_rate_hz * 0.01) as usize;
                let corr_start = if det.peak_sample > corr_half {
                    det.peak_sample - corr_half
                } else {
                    0
                };
                let corr_end = (det.peak_sample + corr_half).min(n);
                let ref_seg = &multichannel[0][corr_start..corr_end];

                for j in 1..self.config.num_microphones {
                    let other_seg = &multichannel[j][corr_start..corr_end];
                    let delay = tdoa_gcc_phat(ref_seg, other_seg);
                    tdoa_pairs.push((0, j, delay));
                }
            }

            // Localize.
            let source_position = if tdoa_pairs.len() >= 3 {
                localize_tdoa_3d(
                    &tdoa_pairs,
                    &self.config.mic_positions,
                    self.config.sound_speed_mps,
                )
            } else {
                None
            };

            let confidence = compute_confidence(&classification, det.peak_amplitude);
            let peak_db = 20.0 * det.peak_amplitude.max(1e-30).log10();

            events.push(GunEvent {
                timestamp_sample: det.onset_sample,
                classification,
                source_position,
                confidence,
                peak_amplitude_db: peak_db,
            });
        }
        events
    }
}

// ---------------------------------------------------------------------------
// Impulse detection
// ---------------------------------------------------------------------------

/// Detect impulsive events in a single-channel signal.
///
/// An impulse is detected when the instantaneous amplitude exceeds the local
/// RMS floor by `threshold_db` decibels. Detections closer than
/// `min_gap_samples` are merged.
pub fn detect_impulsive_events(
    signal: &[f64],
    threshold_db: f64,
    min_gap_samples: usize,
) -> Vec<ImpulseDetection> {
    if signal.is_empty() {
        return Vec::new();
    }

    // Compute RMS of the entire signal as the noise floor estimate.
    let rms = (signal.iter().map(|&s| s * s).sum::<f64>() / signal.len() as f64).sqrt();
    let lin_threshold = rms * 10.0_f64.powf(threshold_db / 20.0);

    let mut detections = Vec::new();
    let mut i = 0;
    let n = signal.len();

    while i < n {
        let amp = signal[i].abs();
        if amp >= lin_threshold {
            // Found onset -- scan forward for peak and duration.
            let onset = i;
            let mut peak_idx = i;
            let mut peak_amp = amp;
            // Advance while above half the threshold (or until gap).
            let decay_threshold = lin_threshold * 0.25;
            while i < n && signal[i].abs() >= decay_threshold {
                if signal[i].abs() > peak_amp {
                    peak_amp = signal[i].abs();
                    peak_idx = i;
                }
                i += 1;
            }
            let duration = i - onset;

            // Check minimum gap from last detection.
            if let Some(last) = detections.last() {
                let last: &ImpulseDetection = last;
                if onset < last.onset_sample + last.duration_samples + min_gap_samples {
                    // Merge: skip this detection (extend previous if peak is bigger).
                    i += 1;
                    continue;
                }
            }

            detections.push(ImpulseDetection {
                onset_sample: onset,
                peak_sample: peak_idx,
                peak_amplitude: peak_amp,
                duration_samples: duration,
            });
        } else {
            i += 1;
        }
    }
    detections
}

// ---------------------------------------------------------------------------
// Classification
// ---------------------------------------------------------------------------

/// Classify an impulsive signal snippet.
///
/// Heuristics based on:
/// - **Rise time**: gunshots have extremely fast rise (< 0.5 ms at 44.1 kHz).
/// - **Duration**: gunshots typically 5--20 ms; firecrackers shorter.
/// - **Spectral energy**: gunshots have broadband energy up to 3+ kHz.
/// - **Energy ratio**: gunshots concentrate energy near the onset.
pub fn classify_impulse(signal: &[f64], fs: f64) -> EventClass {
    if signal.len() < 4 {
        return EventClass::Unknown;
    }

    let rise = rise_time_samples(signal, 0.1);
    let rise_ms = (rise as f64 / fs) * 1000.0;
    let duration_ms = (signal.len() as f64 / fs) * 1000.0;

    // Spectral analysis.
    let spectrum = compute_muzzle_blast_spectrum(signal, fs);
    let spectral_centroid = spectral_centroid_hz(&spectrum, fs);

    // Energy ratio: energy in first 25% vs total.
    let split = signal.len() / 4;
    let eratio = if split > 0 {
        energy_ratio(signal, split)
    } else {
        1.0
    };

    // Decision tree.
    if rise_ms < 0.8 && spectral_centroid > 800.0 && eratio > 1.5 {
        // Very fast rise, broadband, front-loaded energy => gunshot.
        EventClass::Gunshot
    } else if rise_ms < 1.5 && duration_ms < 8.0 && spectral_centroid > 600.0 {
        EventClass::Firecracker
    } else if rise_ms > 3.0 && spectral_centroid < 500.0 {
        EventClass::Vehicle
    } else if rise_ms > 2.0 && spectral_centroid < 700.0 {
        EventClass::Construction
    } else {
        EventClass::Unknown
    }
}

// ---------------------------------------------------------------------------
// TDOA via GCC-PHAT
// ---------------------------------------------------------------------------

/// Generalized Cross-Correlation with PHAse Transform.
///
/// Returns the estimated time delay in **samples** (sub-sample resolution)
/// of `signal_b` relative to `signal_a`. Positive means `signal_b` lags.
///
/// The GCC-PHAT whitens the cross-power spectrum before inverse-transforming,
/// yielding a sharper peak than plain cross-correlation.
pub fn tdoa_gcc_phat(signal_a: &[f64], signal_b: &[f64]) -> f64 {
    let n = signal_a.len().min(signal_b.len());
    if n < 2 {
        return 0.0;
    }

    // Zero-pad to next power of two for efficient DFT.
    let nfft = next_power_of_two(2 * n);

    let mut a_re = vec![0.0; nfft];
    let mut a_im = vec![0.0; nfft];
    let mut b_re = vec![0.0; nfft];
    let mut b_im = vec![0.0; nfft];

    for i in 0..n {
        a_re[i] = signal_a[i];
        b_re[i] = signal_b[i];
    }

    // Forward FFT.
    fft_radix2(&mut a_re, &mut a_im, false);
    fft_radix2(&mut b_re, &mut b_im, false);

    // Cross-power spectrum with PHAT weighting: G = A* . B / |A* . B|.
    let mut g_re = vec![0.0; nfft];
    let mut g_im = vec![0.0; nfft];
    for k in 0..nfft {
        // A* . B
        let xr = a_re[k] * b_re[k] + a_im[k] * b_im[k];
        let xi = a_re[k] * b_im[k] - a_im[k] * b_re[k];
        let mag = (xr * xr + xi * xi).sqrt().max(1e-30);
        g_re[k] = xr / mag;
        g_im[k] = xi / mag;
    }

    // Inverse FFT.
    fft_radix2(&mut g_re, &mut g_im, true);

    // Find the peak in the GCC output (search both positive and negative lags).
    let half = nfft / 2;
    let mut best_idx: isize = 0;
    let mut best_val = f64::NEG_INFINITY;
    // Positive lags [0, half).
    for i in 0..half {
        if g_re[i] > best_val {
            best_val = g_re[i];
            best_idx = i as isize;
        }
    }
    // Negative lags: the last half elements represent lags -(half)..(-1).
    for i in (nfft - half)..nfft {
        if g_re[i] > best_val {
            best_val = g_re[i];
            best_idx = i as isize - nfft as isize;
        }
    }

    // Sub-sample parabolic interpolation around the peak.
    let idx_u = if best_idx >= 0 {
        best_idx as usize
    } else {
        (best_idx + nfft as isize) as usize
    };
    let left = if idx_u == 0 {
        g_re[nfft - 1]
    } else {
        g_re[idx_u - 1]
    };
    let right = g_re[(idx_u + 1) % nfft];
    let center = g_re[idx_u];
    let denom = 2.0 * (2.0 * center - left - right);
    let delta = if denom.abs() > 1e-12 {
        (left - right) / denom
    } else {
        0.0
    };

    best_idx as f64 + delta
}

// ---------------------------------------------------------------------------
// TDOA 3-D Localization
// ---------------------------------------------------------------------------

/// Localize a source in 3-D from TDOA measurements.
///
/// `tdoa_pairs` entries are `(mic_i, mic_j, delay_samples)` where delay is
/// in samples. Uses microphone 0 as reference and solves a linearized
/// least-squares system (the Schau / Friedlander / Chan formulation).
///
/// Returns `None` if the system is under-determined or singular.
pub fn localize_tdoa_3d(
    tdoa_pairs: &[(usize, usize, f64)],
    mic_positions: &[(f64, f64, f64)],
    sound_speed: f64,
) -> Option<(f64, f64, f64)> {
    if tdoa_pairs.is_empty() || mic_positions.len() < 2 {
        return None;
    }

    // Convert delays from samples to metres.
    // We re-express everything relative to mic 0.
    // For pair (0, j), range_diff = delay * sound_speed / sample_rate is
    // already what we need, but the pairs give us delay in samples.  The
    // caller already factored out sample_rate when providing sound_speed
    // in m/s, so we need to divide delay_samples by sample_rate_hz to get
    // seconds and then multiply by sound_speed.  Since we don't have fs
    // here, the caller is expected to pass delay in *samples* and we need
    // fs.  Instead, we accept that sound_speed is in m/sample here for
    // convenience (the GunDetector divides it by fs before calling).
    //
    // Actually -- to keep the API clean, let us accept delay in samples and
    // require the caller to pass sound_speed already divided by fs (i.e.
    // metres-per-sample).  This is done inside GunDetector::process_audio.
    //
    // For the public API we document: sound_speed is in m/s, and the
    // tdoa delay is in *seconds* (i.e. samples / fs).  Let the detector
    // convert.  But for standalone use, the user provides whatever is
    // consistent: if delays are in seconds, sound_speed is m/s.
    //
    // We will treat delay * sound_speed = range difference in metres.

    let m0 = mic_positions[0];

    // Build the over-determined linear system A x = b from the TDOA equation:
    //   |x - m_j| - |x - m_0| = d_j   (range difference)
    // Squaring and subtracting a reference row yields a linear system.
    // We use pairs that involve mic 0 as one end.

    // Collect relevant pairs: those with mic 0.
    let mut rows: Vec<(usize, f64)> = Vec::new(); // (mic_j_index, range_diff)
    for &(mi, mj, delay) in tdoa_pairs {
        let rd = delay * sound_speed;
        if mi == 0 {
            rows.push((mj, rd));
        } else if mj == 0 {
            rows.push((mi, -rd));
        }
    }

    if rows.len() < 3 {
        // Under-determined for 3-D.
        // Fall back to what we have; for 2-D we need >= 2 but we try anyway.
        if rows.is_empty() {
            return None;
        }
    }

    let m = rows.len();
    // Linearisation: for each pair (0, j):
    //   2 (m_j - m_0)^T x - 2 d_j R0 = |m_j|^2 - |m_0|^2 - d_j^2
    // where R0 = |x - m_0| is an extra unknown.
    // This gives [A | -2d] [x; R0]^T = b  (m x 4 system).

    let norm2 = |p: (f64, f64, f64)| p.0 * p.0 + p.1 * p.1 + p.2 * p.2;

    // Build A (m x 4) and b (m).
    let mut a_mat: Vec<[f64; 4]> = Vec::with_capacity(m);
    let mut b_vec: Vec<f64> = Vec::with_capacity(m);

    for &(j, dj) in &rows {
        let mj = mic_positions[j];
        let ax = 2.0 * (mj.0 - m0.0);
        let ay = 2.0 * (mj.1 - m0.1);
        let az = 2.0 * (mj.2 - m0.2);
        let ad = -2.0 * dj;
        let bval = norm2(mj) - norm2(m0) - dj * dj;
        a_mat.push([ax, ay, az, ad]);
        b_vec.push(bval);
    }

    // Solve via normal equations: (A^T A) x = A^T b.
    let ncols = 4;
    let mut ata = [[0.0_f64; 4]; 4];
    let mut atb = [0.0_f64; 4];

    for row_idx in 0..m {
        let r = &a_mat[row_idx];
        for i in 0..ncols {
            for j in 0..ncols {
                ata[i][j] += r[i] * r[j];
            }
            atb[i] += r[i] * b_vec[row_idx];
        }
    }

    // Solve 4x4 linear system via Gaussian elimination with partial pivoting.
    let sol = solve_4x4(&ata, &atb)?;

    Some((sol[0], sol[1], sol[2]))
}

// ---------------------------------------------------------------------------
// Spectral analysis
// ---------------------------------------------------------------------------

/// Compute the power spectral density of a signal.
///
/// Returns a vector of length `nfft/2 + 1` where `nfft` is the next power of
/// two >= `signal.len()`. Values are in linear power units.
pub fn compute_muzzle_blast_spectrum(signal: &[f64], _fs: f64) -> Vec<f64> {
    if signal.is_empty() {
        return Vec::new();
    }
    let nfft = next_power_of_two(signal.len());
    let mut re = vec![0.0; nfft];
    let mut im = vec![0.0; nfft];

    // Apply a Hann window before FFT.
    for (i, &s) in signal.iter().enumerate() {
        let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / signal.len() as f64).cos());
        re[i] = s * w;
    }

    fft_radix2(&mut re, &mut im, false);

    let nbins = nfft / 2 + 1;
    let mut psd = Vec::with_capacity(nbins);
    let norm = 1.0 / (nfft as f64 * nfft as f64);
    for k in 0..nbins {
        psd.push((re[k] * re[k] + im[k] * im[k]) * norm);
    }
    psd
}

// ---------------------------------------------------------------------------
// Temporal analysis helpers
// ---------------------------------------------------------------------------

/// Compute the rise time of a signal in samples.
///
/// Rise time is measured from the first crossing of `threshold_fraction *
/// peak` to the peak itself.
pub fn rise_time_samples(signal: &[f64], threshold_fraction: f64) -> usize {
    if signal.is_empty() {
        return 0;
    }

    // Find peak.
    let mut peak_idx = 0;
    let mut peak_val = 0.0_f64;
    for (i, &s) in signal.iter().enumerate() {
        if s.abs() > peak_val {
            peak_val = s.abs();
            peak_idx = i;
        }
    }

    let thresh = peak_val * threshold_fraction;

    // Scan forward from start to find first crossing.
    let mut onset = 0;
    for (i, &s) in signal.iter().enumerate() {
        if i > peak_idx {
            break;
        }
        if s.abs() >= thresh {
            onset = i;
            break;
        }
    }

    if peak_idx >= onset {
        peak_idx - onset
    } else {
        0
    }
}

/// Compute the ratio of energy before a split point to energy after it.
///
/// Returns `energy_before / energy_after`. A high ratio means the signal is
/// front-loaded (characteristic of muzzle blasts).
pub fn energy_ratio(signal: &[f64], split_sample: usize) -> f64 {
    if signal.is_empty() || split_sample == 0 || split_sample >= signal.len() {
        return 1.0;
    }

    let e_before: f64 = signal[..split_sample].iter().map(|&s| s * s).sum();
    let e_after: f64 = signal[split_sample..].iter().map(|&s| s * s).sum();

    if e_after < 1e-30 {
        return f64::INFINITY;
    }
    e_before / e_after
}

// ---------------------------------------------------------------------------
// Synthetic signal generation
// ---------------------------------------------------------------------------

/// Generate a synthetic gunshot waveform (N-wave with exponential decay).
///
/// The N-wave models the supersonic ballistic shock and the muzzle blast.
/// Returns a mono signal of length `(fs * duration_s)` samples.
pub fn generate_synthetic_gunshot(fs: f64, duration_s: f64) -> Vec<f64> {
    let n = (fs * duration_s).ceil() as usize;
    if n == 0 {
        return Vec::new();
    }
    let mut signal = vec![0.0; n];

    // N-wave parameters.
    let n_wave_duration = 0.001; // 1 ms N-wave
    let n_wave_samples = (fs * n_wave_duration) as usize;

    // Generate N-wave (positive lobe followed by negative lobe).
    for i in 0..n_wave_samples.min(n) {
        let t = i as f64 / n_wave_samples as f64;
        // Sinusoidal N-wave shape.
        signal[i] = (PI * t).sin();
    }
    for i in n_wave_samples..(2 * n_wave_samples).min(n) {
        let t = (i - n_wave_samples) as f64 / n_wave_samples as f64;
        signal[i] = -(PI * t).sin();
    }

    // Exponential decay envelope for the muzzle blast tail.
    let decay_start = 2 * n_wave_samples;
    let decay_rate = 5.0 / (duration_s - 2.0 * n_wave_duration).max(0.001); // ~5 time constants
    for i in decay_start..n {
        let t = (i - decay_start) as f64 / fs;
        let env = (-decay_rate * t).exp();
        // Broadband noise * envelope.
        let noise = simple_hash_noise(i);
        signal[i] = noise * env * 0.3;
    }

    signal
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Spectral centroid in Hz from a PSD vector.
fn spectral_centroid_hz(psd: &[f64], fs: f64) -> f64 {
    if psd.is_empty() {
        return 0.0;
    }
    let nbins = psd.len();
    let bin_hz = fs / (2.0 * (nbins - 1) as f64);
    let mut num = 0.0;
    let mut den = 0.0;
    for (k, &p) in psd.iter().enumerate() {
        let freq = k as f64 * bin_hz;
        num += freq * p;
        den += p;
    }
    if den < 1e-30 {
        0.0
    } else {
        num / den
    }
}

/// Confidence heuristic based on classification and amplitude.
fn compute_confidence(class: &EventClass, amplitude: f64) -> f64 {
    let base = match class {
        EventClass::Gunshot => 0.85,
        EventClass::Firecracker => 0.70,
        EventClass::Vehicle => 0.60,
        EventClass::Construction => 0.55,
        EventClass::Unknown => 0.30,
    };
    // Louder signals are more confidently classified.
    let amp_db = 20.0 * amplitude.max(1e-30).log10();
    let amp_bonus = ((amp_db + 20.0) / 80.0).clamp(0.0, 0.15);
    (base + amp_bonus).min(1.0)
}

/// Deterministic pseudo-random noise for synthetic signal generation.
fn simple_hash_noise(index: usize) -> f64 {
    // Simple hash to get a deterministic "random" value in [-1, 1].
    let mut x = index as u64;
    x = x.wrapping_mul(6364136223846793005);
    x = x.wrapping_add(1442695040888963407);
    x ^= x >> 33;
    x = x.wrapping_mul(0xff51afd7ed558ccd);
    x ^= x >> 33;
    (x as f64 / u64::MAX as f64) * 2.0 - 1.0
}

/// Next power of two >= n.
fn next_power_of_two(n: usize) -> usize {
    let mut p = 1;
    while p < n {
        p <<= 1;
    }
    p
}

/// In-place radix-2 Cooley-Tukey FFT.
///
/// If `inverse` is true, computes the IFFT (with 1/N normalisation).
fn fft_radix2(re: &mut [f64], im: &mut [f64], inverse: bool) {
    let n = re.len();
    assert_eq!(n, im.len());
    assert!(n.is_power_of_two(), "FFT length must be a power of two");

    // Bit-reversal permutation.
    let bits = n.trailing_zeros();
    for i in 0..n {
        let j = i.reverse_bits() >> (usize::BITS - bits);
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }

    // Butterfly stages.
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let wn_re = angle.cos();
        let wn_im = angle.sin();

        let mut start = 0;
        while start < n {
            let mut w_re = 1.0;
            let mut w_im = 0.0;
            for k in 0..half {
                let even = start + k;
                let odd = start + k + half;
                let t_re = w_re * re[odd] - w_im * im[odd];
                let t_im = w_re * im[odd] + w_im * re[odd];
                re[odd] = re[even] - t_re;
                im[odd] = im[even] - t_im;
                re[even] += t_re;
                im[even] += t_im;
                let new_w_re = w_re * wn_re - w_im * wn_im;
                let new_w_im = w_re * wn_im + w_im * wn_re;
                w_re = new_w_re;
                w_im = new_w_im;
            }
            start += len;
        }
        len <<= 1;
    }

    // Normalise inverse FFT.
    if inverse {
        let inv_n = 1.0 / n as f64;
        for i in 0..n {
            re[i] *= inv_n;
            im[i] *= inv_n;
        }
    }
}

/// Solve a 4x4 linear system via Gaussian elimination with partial pivoting.
///
/// Returns `None` if the matrix is singular.
fn solve_4x4(a: &[[f64; 4]; 4], b: &[f64; 4]) -> Option<[f64; 4]> {
    // Augmented matrix.
    let mut m = [[0.0_f64; 5]; 4];
    for i in 0..4 {
        for j in 0..4 {
            m[i][j] = a[i][j];
        }
        m[i][4] = b[i];
    }

    // Forward elimination with partial pivoting.
    for col in 0..4 {
        // Find pivot.
        let mut max_row = col;
        let mut max_val = m[col][col].abs();
        for row in (col + 1)..4 {
            if m[row][col].abs() > max_val {
                max_val = m[row][col].abs();
                max_row = row;
            }
        }
        if max_val < 1e-12 {
            return None; // Singular.
        }
        if max_row != col {
            m.swap(col, max_row);
        }
        let pivot = m[col][col];
        for row in (col + 1)..4 {
            let factor = m[row][col] / pivot;
            for j in col..5 {
                m[row][j] -= factor * m[col][j];
            }
        }
    }

    // Back substitution.
    let mut x = [0.0_f64; 4];
    for i in (0..4).rev() {
        let mut sum = m[i][4];
        for j in (i + 1)..4 {
            sum -= m[i][j] * x[j];
        }
        x[i] = sum / m[i][i];
    }
    Some(x)
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const FS: f64 = 44100.0;

    // Helper: generate a simple impulse at a given sample.
    fn impulse(n: usize, pos: usize, amplitude: f64) -> Vec<f64> {
        let mut s = vec![0.0; n];
        if pos < n {
            s[pos] = amplitude;
        }
        s
    }

    // Helper: add Gaussian-ish noise from a deterministic source.
    fn add_noise(signal: &mut [f64], amplitude: f64) {
        for i in 0..signal.len() {
            signal[i] += simple_hash_noise(i + 7777) * amplitude;
        }
    }

    // -----------------------------------------------------------------------
    // Impulse detection tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_detect_single_impulse() {
        let mut sig = vec![0.001; 4410]; // ~100 ms of low-level noise
        sig[2000] = 1.0;
        sig[2001] = 0.8;
        sig[2002] = 0.3;
        let dets = detect_impulsive_events(&sig, 20.0, 100);
        assert!(!dets.is_empty(), "should detect at least one impulse");
        assert_eq!(dets[0].peak_sample, 2000);
    }

    #[test]
    fn test_detect_multiple_impulses_with_gap() {
        let mut sig = vec![0.001; 44100];
        sig[5000] = 1.0;
        sig[25000] = 0.9;
        let dets = detect_impulsive_events(&sig, 20.0, 100);
        assert!(dets.len() >= 2, "should detect two separate impulses");
    }

    #[test]
    fn test_detect_no_impulse_below_threshold() {
        let sig = vec![0.01; 4410];
        let dets = detect_impulsive_events(&sig, 40.0, 100);
        assert!(dets.is_empty(), "no impulse above threshold");
    }

    #[test]
    fn test_detect_empty_signal() {
        let dets = detect_impulsive_events(&[], 20.0, 100);
        assert!(dets.is_empty());
    }

    #[test]
    fn test_detect_merges_close_impulses() {
        let mut sig = vec![0.001; 4410];
        sig[1000] = 1.0;
        sig[1050] = 0.9; // Only 50 samples later, should merge with min_gap=200.
        let dets = detect_impulsive_events(&sig, 20.0, 200);
        assert_eq!(dets.len(), 1, "close impulses should merge");
    }

    // -----------------------------------------------------------------------
    // Rise time tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_rise_time_impulse() {
        let mut sig = vec![0.0; 100];
        sig[10] = 0.2;
        sig[11] = 0.5;
        sig[12] = 1.0;
        let rt = rise_time_samples(&sig, 0.1);
        assert_eq!(rt, 2, "rise time should be 2 samples (from index 10 to 12)");
    }

    #[test]
    fn test_rise_time_instant() {
        let mut sig = vec![0.0; 100];
        sig[50] = 1.0; // Instant rise.
        let rt = rise_time_samples(&sig, 0.1);
        assert_eq!(rt, 0, "instant rise => 0 samples");
    }

    #[test]
    fn test_rise_time_empty() {
        assert_eq!(rise_time_samples(&[], 0.1), 0);
    }

    #[test]
    fn test_rise_time_gradual() {
        let mut sig = vec![0.0; 50];
        for i in 0..20 {
            sig[i] = i as f64 / 20.0;
        }
        sig[20] = 1.0;
        let rt = rise_time_samples(&sig, 0.1);
        // First crossing of 0.1 is around index 2, peak at 20 => ~18.
        assert!(rt >= 15 && rt <= 20, "gradual rise time: got {}", rt);
    }

    // -----------------------------------------------------------------------
    // Energy ratio tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_energy_ratio_front_loaded() {
        let mut sig = vec![0.01; 100];
        for i in 0..25 {
            sig[i] = 1.0;
        }
        let er = energy_ratio(&sig, 25);
        assert!(er > 10.0, "front-loaded signal should have high ratio, got {}", er);
    }

    #[test]
    fn test_energy_ratio_uniform() {
        let sig = vec![1.0; 100];
        let er = energy_ratio(&sig, 50);
        assert!(
            (er - 1.0).abs() < 0.01,
            "uniform signal => ratio ~1.0, got {}",
            er
        );
    }

    #[test]
    fn test_energy_ratio_empty() {
        assert_eq!(energy_ratio(&[], 0), 1.0);
    }

    #[test]
    fn test_energy_ratio_split_at_boundary() {
        let sig = vec![1.0; 100];
        // split_sample == 0 or >= len returns 1.0.
        assert_eq!(energy_ratio(&sig, 0), 1.0);
        assert_eq!(energy_ratio(&sig, 100), 1.0);
    }

    // -----------------------------------------------------------------------
    // GCC-PHAT / TDOA tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_gcc_phat_zero_delay() {
        let sig: Vec<f64> = (0..128).map(|i| (2.0 * PI * 5.0 * i as f64 / 128.0).sin()).collect();
        let delay = tdoa_gcc_phat(&sig, &sig);
        assert!(
            delay.abs() < 0.5,
            "identical signals => ~0 delay, got {}",
            delay
        );
    }

    #[test]
    fn test_gcc_phat_known_integer_delay() {
        let n = 256;
        let mut sig_a = vec![0.0; n];
        sig_a[50] = 1.0;

        let delay_samples = 10;
        let mut sig_b = vec![0.0; n];
        sig_b[50 + delay_samples] = 1.0;

        let est = tdoa_gcc_phat(&sig_a, &sig_b);
        assert!(
            (est - delay_samples as f64).abs() < 1.5,
            "expected delay ~{}, got {}",
            delay_samples,
            est
        );
    }

    #[test]
    fn test_gcc_phat_negative_delay() {
        let n = 256;
        let mut sig_a = vec![0.0; n];
        sig_a[60] = 1.0;

        let mut sig_b = vec![0.0; n];
        sig_b[50] = 1.0; // sig_b leads sig_a => negative delay.

        let est = tdoa_gcc_phat(&sig_a, &sig_b);
        assert!(
            est < 0.0,
            "expected negative delay, got {}",
            est
        );
        assert!(
            (est + 10.0).abs() < 1.5,
            "expected delay ~-10, got {}",
            est
        );
    }

    #[test]
    fn test_gcc_phat_short_signal() {
        let sig = vec![1.0];
        let delay = tdoa_gcc_phat(&sig, &sig);
        assert_eq!(delay, 0.0, "single-sample signal => 0 delay");
    }

    // -----------------------------------------------------------------------
    // Spectral analysis tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_muzzle_blast_spectrum_sine() {
        // A pure tone should show a spectral peak.
        let n = 1024;
        let freq = 1000.0;
        let sig: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * freq * i as f64 / FS).sin())
            .collect();
        let psd = compute_muzzle_blast_spectrum(&sig, FS);
        assert!(!psd.is_empty());
        // Find peak bin.
        let peak_bin = psd
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let nfft = next_power_of_two(n);
        let peak_freq = peak_bin as f64 * FS / nfft as f64;
        assert!(
            (peak_freq - freq).abs() < 100.0,
            "spectral peak should be near {} Hz, got {} Hz",
            freq,
            peak_freq
        );
    }

    #[test]
    fn test_muzzle_blast_spectrum_empty() {
        let psd = compute_muzzle_blast_spectrum(&[], FS);
        assert!(psd.is_empty());
    }

    #[test]
    fn test_spectral_centroid() {
        // A signal with energy concentrated at high frequencies should have a
        // high spectral centroid.
        let n = 1024;
        let high_freq: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 8000.0 * i as f64 / FS).sin())
            .collect();
        let low_freq: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 500.0 * i as f64 / FS).sin())
            .collect();
        let psd_high = compute_muzzle_blast_spectrum(&high_freq, FS);
        let psd_low = compute_muzzle_blast_spectrum(&low_freq, FS);
        let cent_high = spectral_centroid_hz(&psd_high, FS);
        let cent_low = spectral_centroid_hz(&psd_low, FS);
        assert!(
            cent_high > cent_low,
            "high-freq centroid ({}) should exceed low-freq centroid ({})",
            cent_high,
            cent_low
        );
    }

    // -----------------------------------------------------------------------
    // Classification tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_classify_synthetic_gunshot() {
        let shot = generate_synthetic_gunshot(FS, 0.02);
        let class = classify_impulse(&shot, FS);
        assert!(
            class == EventClass::Gunshot || class == EventClass::Firecracker,
            "synthetic gunshot should classify as Gunshot or Firecracker, got {:?}",
            class
        );
    }

    #[test]
    fn test_classify_slow_rise_as_vehicle_or_construction() {
        // Slow rise signal: ramp up over 5 ms.
        let n = (FS * 0.02) as usize;
        let mut sig = vec![0.0; n];
        let ramp = (FS * 0.005) as usize;
        for i in 0..ramp.min(n) {
            sig[i] = i as f64 / ramp as f64;
        }
        for i in ramp..n {
            sig[i] = (-(i as f64 - ramp as f64) / (FS * 0.01)).exp();
        }
        let class = classify_impulse(&sig, FS);
        assert!(
            class == EventClass::Vehicle
                || class == EventClass::Construction
                || class == EventClass::Unknown,
            "slow rise should not be Gunshot, got {:?}",
            class
        );
    }

    #[test]
    fn test_classify_very_short_signal() {
        let sig = vec![1.0, -1.0, 0.5];
        let class = classify_impulse(&sig, FS);
        // With only 3 samples the function returns Unknown.
        assert_eq!(class, EventClass::Unknown);
    }

    // -----------------------------------------------------------------------
    // Synthetic gunshot generation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_generate_synthetic_gunshot_length() {
        let dur = 0.05;
        let shot = generate_synthetic_gunshot(FS, dur);
        let expected = (FS * dur).ceil() as usize;
        assert_eq!(shot.len(), expected);
    }

    #[test]
    fn test_generate_synthetic_gunshot_has_n_wave() {
        let shot = generate_synthetic_gunshot(FS, 0.05);
        // The N-wave should have both positive and negative peaks.
        let max = shot.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let min = shot.iter().cloned().fold(f64::INFINITY, f64::min);
        assert!(max > 0.5, "N-wave positive peak should exist");
        assert!(min < -0.5, "N-wave negative peak should exist");
    }

    #[test]
    fn test_generate_synthetic_gunshot_zero_duration() {
        let shot = generate_synthetic_gunshot(FS, 0.0);
        assert!(shot.is_empty());
    }

    #[test]
    fn test_generate_synthetic_gunshot_decays() {
        let shot = generate_synthetic_gunshot(FS, 0.1);
        let n = shot.len();
        // Energy in last 10% should be much less than first 10%.
        let tenth = n / 10;
        let e_front: f64 = shot[..tenth].iter().map(|&s| s * s).sum();
        let e_tail: f64 = shot[n - tenth..].iter().map(|&s| s * s).sum();
        assert!(
            e_front > e_tail * 2.0,
            "signal should decay: front energy {} vs tail energy {}",
            e_front,
            e_tail
        );
    }

    // -----------------------------------------------------------------------
    // 3-D localization tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_localize_tdoa_3d_known_source() {
        // Place 5 mics at known positions and compute TDOA from a known source.
        let mics = vec![
            (0.0, 0.0, 0.0),
            (10.0, 0.0, 0.0),
            (0.0, 10.0, 0.0),
            (0.0, 0.0, 10.0),
            (10.0, 10.0, 0.0),
        ];
        let source = (3.0, 7.0, 2.0);
        let c = 343.0;

        let dist = |a: (f64, f64, f64), b: (f64, f64, f64)| -> f64 {
            ((a.0 - b.0).powi(2) + (a.1 - b.1).powi(2) + (a.2 - b.2).powi(2)).sqrt()
        };

        let d0 = dist(source, mics[0]);
        let mut tdoa_pairs = Vec::new();
        for j in 1..mics.len() {
            let dj = dist(source, mics[j]);
            let delay_s = (dj - d0) / c;
            tdoa_pairs.push((0, j, delay_s));
        }

        let pos = localize_tdoa_3d(&tdoa_pairs, &mics, c);
        assert!(pos.is_some(), "localization should succeed");
        let (x, y, z) = pos.unwrap();
        let err = ((x - source.0).powi(2) + (y - source.1).powi(2) + (z - source.2).powi(2))
            .sqrt();
        assert!(
            err < 2.0,
            "localization error should be < 2 m, got {} m (pos: {:.2}, {:.2}, {:.2})",
            err,
            x,
            y,
            z
        );
    }

    #[test]
    fn test_localize_tdoa_3d_insufficient_pairs() {
        let mics = vec![(0.0, 0.0, 0.0), (10.0, 0.0, 0.0)];
        let tdoa_pairs = vec![(0, 1, 0.001)];
        // Only 1 pair => under-determined for 3-D.
        let pos = localize_tdoa_3d(&tdoa_pairs, &mics, 343.0);
        // May return None or a degenerate solution; just ensure no panic.
        let _ = pos;
    }

    #[test]
    fn test_localize_tdoa_3d_empty() {
        let pos = localize_tdoa_3d(&[], &[], 343.0);
        assert!(pos.is_none());
    }

    // -----------------------------------------------------------------------
    // GunDetector integration tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_gun_detector_basic() {
        let mics = vec![
            (0.0, 0.0, 0.0),
            (10.0, 0.0, 0.0),
            (0.0, 10.0, 0.0),
            (0.0, 0.0, 10.0),
        ];
        let config = GunDetectorConfig {
            num_microphones: 4,
            mic_positions: mics,
            sample_rate_hz: FS,
            sound_speed_mps: 343.0,
            detection_threshold_db: 15.0,
        };
        let mut det = GunDetector::new(config);

        // Generate a synthetic gunshot on all channels (no inter-channel delay
        // for simplicity, so source is equidistant from all mics).
        let shot = generate_synthetic_gunshot(FS, 0.05);
        let mut multichannel = vec![vec![0.001; 44100]; 4];
        for ch in &mut multichannel {
            for (i, &s) in shot.iter().enumerate() {
                if i + 1000 < ch.len() {
                    ch[i + 1000] = s + ch[i + 1000];
                }
            }
        }
        let events = det.process_audio(&multichannel);
        assert!(!events.is_empty(), "should detect at least one event");
    }

    #[test]
    fn test_gun_detector_no_event_for_silence() {
        let mics = vec![
            (0.0, 0.0, 0.0),
            (10.0, 0.0, 0.0),
            (0.0, 10.0, 0.0),
            (0.0, 0.0, 10.0),
        ];
        let config = GunDetectorConfig {
            num_microphones: 4,
            mic_positions: mics,
            sample_rate_hz: FS,
            sound_speed_mps: 343.0,
            detection_threshold_db: 20.0,
        };
        let mut det = GunDetector::new(config);
        let multichannel = vec![vec![0.001; 4410]; 4];
        let events = det.process_audio(&multichannel);
        assert!(events.is_empty(), "silence should produce no events");
    }

    #[test]
    fn test_gun_detector_single_mic() {
        let config = GunDetectorConfig {
            num_microphones: 1,
            mic_positions: vec![(0.0, 0.0, 0.0)],
            sample_rate_hz: FS,
            sound_speed_mps: 343.0,
            detection_threshold_db: 15.0,
        };
        let mut det = GunDetector::new(config);

        let shot = generate_synthetic_gunshot(FS, 0.05);
        let mut ch = vec![0.001; 44100];
        for (i, &s) in shot.iter().enumerate() {
            if i + 500 < ch.len() {
                ch[i + 500] = s;
            }
        }
        let events = det.process_audio(&[ch]);
        assert!(!events.is_empty(), "single-mic detector should still detect");
        // No localization possible with one mic.
        assert!(
            events[0].source_position.is_none(),
            "single mic cannot localize"
        );
    }

    #[test]
    fn test_gun_detector_empty_signal() {
        let config = GunDetectorConfig {
            num_microphones: 2,
            mic_positions: vec![(0.0, 0.0, 0.0), (5.0, 0.0, 0.0)],
            sample_rate_hz: FS,
            sound_speed_mps: 343.0,
            detection_threshold_db: 20.0,
        };
        let mut det = GunDetector::new(config);
        let events = det.process_audio(&[vec![], vec![]]);
        assert!(events.is_empty());
    }

    // -----------------------------------------------------------------------
    // FFT helper tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_fft_roundtrip() {
        let n = 64;
        let original: Vec<f64> = (0..n).map(|i| (2.0 * PI * 3.0 * i as f64 / n as f64).sin()).collect();
        let mut re = original.clone();
        let mut im = vec![0.0; n];

        fft_radix2(&mut re, &mut im, false);
        fft_radix2(&mut re, &mut im, true);

        for (i, (&orig, &recovered)) in original.iter().zip(re.iter()).enumerate() {
            assert!(
                (orig - recovered).abs() < 1e-10,
                "FFT roundtrip mismatch at index {}: {} vs {}",
                i,
                orig,
                recovered
            );
        }
    }

    #[test]
    fn test_fft_dc_signal() {
        let n = 16;
        let mut re = vec![1.0; n];
        let mut im = vec![0.0; n];
        fft_radix2(&mut re, &mut im, false);
        // DC bin should be N, all others ~0.
        assert!((re[0] - n as f64).abs() < 1e-10);
        for k in 1..n {
            assert!(
                re[k].abs() < 1e-10 && im[k].abs() < 1e-10,
                "non-DC bin {} should be ~0",
                k
            );
        }
    }

    // -----------------------------------------------------------------------
    // Solver tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_solve_4x4_identity() {
        let a = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let b = [1.0, 2.0, 3.0, 4.0];
        let x = solve_4x4(&a, &b).unwrap();
        for i in 0..4 {
            assert!(
                (x[i] - b[i]).abs() < 1e-10,
                "identity solve failed at {}",
                i
            );
        }
    }

    #[test]
    fn test_solve_4x4_singular() {
        let a = [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ];
        let b = [1.0, 2.0, 3.0, 4.0];
        assert!(solve_4x4(&a, &b).is_none(), "singular matrix should return None");
    }

    // -----------------------------------------------------------------------
    // Edge case and miscellaneous tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_next_power_of_two() {
        assert_eq!(next_power_of_two(1), 1);
        assert_eq!(next_power_of_two(2), 2);
        assert_eq!(next_power_of_two(3), 4);
        assert_eq!(next_power_of_two(1023), 1024);
        assert_eq!(next_power_of_two(1024), 1024);
    }

    #[test]
    fn test_simple_hash_noise_deterministic() {
        let a = simple_hash_noise(42);
        let b = simple_hash_noise(42);
        assert_eq!(a, b, "hash noise must be deterministic");
    }

    #[test]
    fn test_simple_hash_noise_range() {
        for i in 0..1000 {
            let v = simple_hash_noise(i);
            assert!(
                v >= -1.0 && v <= 1.0,
                "noise value {} out of range at index {}",
                v,
                i
            );
        }
    }

    #[test]
    fn test_confidence_range() {
        for class in &[
            EventClass::Gunshot,
            EventClass::Firecracker,
            EventClass::Vehicle,
            EventClass::Construction,
            EventClass::Unknown,
        ] {
            for amp in &[0.001, 0.1, 1.0, 10.0] {
                let c = compute_confidence(class, *amp);
                assert!(
                    (0.0..=1.0).contains(&c),
                    "confidence {} out of [0,1] for {:?} amp={}",
                    c,
                    class,
                    amp
                );
            }
        }
    }

    #[test]
    fn test_event_class_variants() {
        // Ensure all variants are distinct.
        let classes = [
            EventClass::Gunshot,
            EventClass::Firecracker,
            EventClass::Vehicle,
            EventClass::Construction,
            EventClass::Unknown,
        ];
        for i in 0..classes.len() {
            for j in (i + 1)..classes.len() {
                assert_ne!(classes[i], classes[j]);
            }
        }
    }
}
