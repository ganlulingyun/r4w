//! Voice Activity Detection (VAD) for speech processing and radio communications.
//!
//! This module provides real-time voice activity detection using a combination
//! of short-time energy (STE), zero crossing rate (ZCR), and spectral flatness
//! measure (SFM). It supports both fixed-threshold and adaptive-threshold
//! operation, with hangover logic to prevent clipping trailing syllables.
//!
//! # Features
//!
//! - **Short-time energy**: Detects voiced speech segments with high amplitude
//! - **Zero crossing rate**: Distinguishes voiced (low ZCR) from unvoiced (high ZCR) speech
//! - **Spectral flatness**: Differentiates tonal speech (low SFM) from broadband noise (high SFM)
//! - **Hangover**: Keeps speech state active for a configurable number of frames after energy drops
//! - **Adaptive VAD**: Automatically tracks noise floor and adjusts threshold
//!
//! # Example
//!
//! ```rust
//! use r4w_core::voice_activity_detector::{VoiceActivityDetector, VadConfig, VadState};
//!
//! let config = VadConfig {
//!     frame_size: 160,
//!     energy_threshold_db: -30.0,
//!     spectral_flatness_threshold: 0.5,
//!     hangover_frames: 5,
//!     sample_rate: 8000.0,
//! };
//! let mut vad = VoiceActivityDetector::new(config);
//!
//! // Process a frame of silence
//! let silence = vec![0.001; 160];
//! let state = vad.process_frame(&silence);
//! assert_eq!(state, VadState::Silent);
//! assert!(!vad.is_speech());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// VadState
// ---------------------------------------------------------------------------

/// The three-state result of voice activity detection.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum VadState {
    /// No speech detected; the frame is silence or background noise.
    Silent,
    /// Speech is actively detected in the current frame.
    Speech,
    /// Transitional state: speech has ended but hangover is still active,
    /// keeping the channel open to avoid clipping trailing syllables.
    Transition,
}

// ---------------------------------------------------------------------------
// VadConfig
// ---------------------------------------------------------------------------

/// Configuration parameters for the voice activity detector.
#[derive(Debug, Clone)]
pub struct VadConfig {
    /// Number of samples per frame.
    pub frame_size: usize,
    /// Energy threshold in dB; frames above this are considered potential speech.
    pub energy_threshold_db: f64,
    /// Spectral flatness threshold in [0, 1]. Frames with SFM below this
    /// value are more likely to contain tonal (speech) content. A value of
    /// 1.0 means flat/noise-like; 0.0 means purely tonal.
    pub spectral_flatness_threshold: f64,
    /// Number of frames to keep the speech state active after energy drops
    /// below the threshold (hangover / holdover).
    pub hangover_frames: usize,
    /// Sample rate in Hz. Used for documentation and factory functions.
    pub sample_rate: f64,
}

impl Default for VadConfig {
    fn default() -> Self {
        Self {
            frame_size: 160,
            energy_threshold_db: -30.0,
            spectral_flatness_threshold: 0.5,
            hangover_frames: 5,
            sample_rate: 8000.0,
        }
    }
}

// ---------------------------------------------------------------------------
// VoiceActivityDetector
// ---------------------------------------------------------------------------

/// A voice activity detector combining energy, ZCR, and spectral flatness.
#[derive(Debug, Clone)]
pub struct VoiceActivityDetector {
    config: VadConfig,
    /// Energy threshold in linear scale.
    energy_threshold_linear: f64,
    /// Current VAD state.
    state: VadState,
    /// Current hangover counter.
    hangover_count: usize,
    /// Most recent speech probability estimate.
    probability: f64,
}

impl VoiceActivityDetector {
    /// Create a new voice activity detector with the given configuration.
    pub fn new(config: VadConfig) -> Self {
        let energy_threshold_linear = 10.0_f64.powf(config.energy_threshold_db / 10.0);
        Self {
            config,
            energy_threshold_linear,
            state: VadState::Silent,
            hangover_count: 0,
            probability: 0.0,
        }
    }

    /// Analyze one frame of audio samples and return the VAD state.
    ///
    /// The decision is based on three features:
    /// 1. Short-time energy exceeds the configured dB threshold.
    /// 2. Spectral flatness is below the configured threshold (tonal content).
    /// 3. Hangover logic extends speech state for trailing syllables.
    pub fn process_frame(&mut self, frame: &[f64]) -> VadState {
        let energy = frame_energy_linear(frame);
        let sfm = spectral_flatness(frame);

        // Compute a probability-like score from the three features.
        // Energy contribution: sigmoid around the threshold.
        let energy_db = linear_to_db(energy);
        let energy_score = sigmoid(energy_db - self.config.energy_threshold_db, 0.5);

        // Spectral flatness contribution: lower SFM = more tonal = more likely speech.
        let sfm_score = 1.0 - sfm;

        // ZCR contribution: moderate ZCR is typical of speech; very high is noise.
        let zcr = zero_crossing_rate(frame);
        let zcr_score = if zcr < 0.5 { 1.0 - zcr } else { 0.5 - (zcr - 0.5) }.max(0.0);

        // Weighted combination.
        self.probability = 0.5 * energy_score + 0.3 * sfm_score + 0.2 * zcr_score;
        self.probability = self.probability.clamp(0.0, 1.0);

        // Primary decision: energy above threshold AND spectral flatness below threshold.
        let speech_detected = energy > self.energy_threshold_linear
            && sfm < self.config.spectral_flatness_threshold;

        if speech_detected {
            self.state = VadState::Speech;
            self.hangover_count = self.config.hangover_frames;
        } else if self.hangover_count > 0 {
            self.hangover_count -= 1;
            self.state = VadState::Transition;
        } else {
            self.state = VadState::Silent;
            // Decay probability toward zero during silence.
            self.probability *= 0.5;
        }

        self.state
    }

    /// Returns `true` if the detector is currently in the Speech or Transition state.
    pub fn is_speech(&self) -> bool {
        matches!(self.state, VadState::Speech | VadState::Transition)
    }

    /// Returns the estimated speech probability in [0.0, 1.0].
    ///
    /// Higher values indicate greater confidence that the current frame
    /// contains speech.
    pub fn speech_probability(&self) -> f64 {
        self.probability
    }

    /// Reset the detector to its initial state.
    pub fn reset(&mut self) {
        self.state = VadState::Silent;
        self.hangover_count = 0;
        self.probability = 0.0;
    }
}

// ---------------------------------------------------------------------------
// AdaptiveVad
// ---------------------------------------------------------------------------

/// An adaptive voice activity detector that automatically tracks the noise
/// floor and adjusts the energy threshold.
///
/// Uses an exponential moving average to estimate the background noise level,
/// then sets the decision threshold a fixed number of dB above it.
#[derive(Debug, Clone)]
pub struct AdaptiveVad {
    /// Inner fixed-threshold VAD.
    inner: VoiceActivityDetector,
    /// EMA smoothing factor for noise floor estimation.
    alpha: f64,
    /// Current estimated noise floor (linear power).
    noise_floor: f64,
    /// Margin in dB above the noise floor to place the decision threshold.
    margin_db: f64,
    /// Whether the noise floor has been initialized.
    initialized: bool,
}

impl AdaptiveVad {
    /// Create a new adaptive VAD with the given frame size and sample rate.
    ///
    /// Uses sensible defaults: 6 dB margin, 10-frame hangover, 0.5 SFM threshold.
    pub fn new(frame_size: usize, sample_rate: f64) -> Self {
        let config = VadConfig {
            frame_size,
            energy_threshold_db: -40.0, // Will be adapted.
            spectral_flatness_threshold: 0.5,
            hangover_frames: 10,
            sample_rate,
        };
        Self {
            inner: VoiceActivityDetector::new(config),
            alpha: 0.05,
            noise_floor: 0.0,
            margin_db: 6.0,
            initialized: false,
        }
    }

    /// Process one frame with adaptive threshold adjustment.
    ///
    /// During silence, the noise floor estimate is updated. The energy
    /// threshold is set to `noise_floor_db + margin_db`.
    pub fn process_frame(&mut self, frame: &[f64]) -> VadState {
        let energy = frame_energy_linear(frame);

        if !self.initialized {
            // Initialize noise floor with first frame.
            self.noise_floor = energy.max(1e-30);
            self.initialized = true;
            // Set threshold based on initial noise floor.
            let noise_db = linear_to_db(self.noise_floor);
            self.inner.config.energy_threshold_db = noise_db + self.margin_db;
            self.inner.energy_threshold_linear =
                10.0_f64.powf(self.inner.config.energy_threshold_db / 10.0);
        }

        let state = self.inner.process_frame(frame);

        // Only update noise floor during silence to avoid biasing it upward.
        if state == VadState::Silent {
            self.noise_floor = (1.0 - self.alpha) * self.noise_floor + self.alpha * energy;
            let noise_db = linear_to_db(self.noise_floor.max(1e-30));
            self.inner.config.energy_threshold_db = noise_db + self.margin_db;
            self.inner.energy_threshold_linear =
                10.0_f64.powf(self.inner.config.energy_threshold_db / 10.0);
        }

        state
    }

    /// Get the current noise floor estimate in dB.
    pub fn noise_floor_db(&self) -> f64 {
        linear_to_db(self.noise_floor.max(1e-30))
    }

    /// Get the current adaptive energy threshold in dB.
    pub fn threshold_db(&self) -> f64 {
        self.inner.config.energy_threshold_db
    }
}

// ---------------------------------------------------------------------------
// Factory functions
// ---------------------------------------------------------------------------

/// Create a VAD configured for telephone-quality audio (8 kHz, 20 ms frames).
///
/// Frame size = 160 samples (20 ms at 8 kHz). This is the standard frame
/// duration used in G.729 and similar telephone codecs.
pub fn telephone_vad() -> VoiceActivityDetector {
    VoiceActivityDetector::new(VadConfig {
        frame_size: 160,
        energy_threshold_db: -30.0,
        spectral_flatness_threshold: 0.5,
        hangover_frames: 5,
        sample_rate: 8000.0,
    })
}

/// Create a VAD configured for wideband audio (16 kHz, 20 ms frames).
///
/// Frame size = 320 samples (20 ms at 16 kHz). Suitable for wideband
/// speech codecs like AMR-WB.
pub fn wideband_vad() -> VoiceActivityDetector {
    VoiceActivityDetector::new(VadConfig {
        frame_size: 320,
        energy_threshold_db: -30.0,
        spectral_flatness_threshold: 0.5,
        hangover_frames: 5,
        sample_rate: 16000.0,
    })
}

// ---------------------------------------------------------------------------
// Public free functions
// ---------------------------------------------------------------------------

/// Compute the frame energy in dB.
///
/// Energy is defined as the mean of the squared samples:
/// `E = (1/N) * sum(x[n]^2)`.
/// The result is expressed in dB: `10 * log10(E)`.
///
/// Returns approximately -300 dB for a zero-energy frame.
pub fn frame_energy_db(frame: &[f64]) -> f64 {
    linear_to_db(frame_energy_linear(frame))
}

/// Compute the zero crossing rate of a frame, normalized to [0, 1].
///
/// ZCR counts the fraction of adjacent sample pairs that have different
/// signs. A value near 0 indicates a low-frequency or DC signal; a value
/// near 0.5 indicates a signal near Nyquist/2; a value near 1.0 indicates
/// very high frequency or heavily alternating content.
pub fn zero_crossing_rate(frame: &[f64]) -> f64 {
    if frame.len() < 2 {
        return 0.0;
    }
    let crossings = frame
        .windows(2)
        .filter(|w| (w[0] >= 0.0) != (w[1] >= 0.0))
        .count();
    crossings as f64 / (frame.len() - 1) as f64
}

/// Compute the spectral flatness measure (SFM) of a frame.
///
/// SFM is defined as the ratio of the geometric mean to the arithmetic
/// mean of the power spectrum magnitudes:
///
/// `SFM = geometric_mean(|X[k]|^2) / arithmetic_mean(|X[k]|^2)`
///
/// A value near 1.0 indicates a flat (noise-like) spectrum; a value near
/// 0.0 indicates a tonal (speech-like) spectrum with spectral peaks.
///
/// The power spectrum is computed via a brute-force DFT (no external
/// FFT crate required).
pub fn spectral_flatness(frame: &[f64]) -> f64 {
    if frame.is_empty() {
        return 0.0;
    }

    let n = frame.len();

    // Compute magnitude-squared spectrum via DFT.
    // We only need the first N/2+1 bins (real signal is symmetric).
    let num_bins = n / 2 + 1;
    let mut power_spectrum = Vec::with_capacity(num_bins);

    for k in 0..num_bins {
        let mut re = 0.0_f64;
        let mut im = 0.0_f64;
        for (i, &x) in frame.iter().enumerate() {
            let angle = -2.0 * PI * (k as f64) * (i as f64) / (n as f64);
            let (sin_a, cos_a) = angle.sin_cos();
            re += x * cos_a;
            im += x * sin_a;
        }
        let mag_sq = (re * re + im * im) / (n as f64 * n as f64);
        power_spectrum.push(mag_sq);
    }

    // Arithmetic mean.
    let arith_mean = power_spectrum.iter().sum::<f64>() / num_bins as f64;
    if arith_mean <= 0.0 {
        return 0.0;
    }

    // Geometric mean via log-domain to avoid overflow/underflow.
    // geo_mean = exp( (1/N) * sum(ln(x_k)) )
    // We clamp each value to avoid ln(0).
    let log_sum: f64 = power_spectrum
        .iter()
        .map(|&p| (p.max(1e-30)).ln())
        .sum::<f64>();
    let geo_mean = (log_sum / num_bins as f64).exp();

    (geo_mean / arith_mean).clamp(0.0, 1.0)
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Compute frame energy as mean of squared samples (linear scale).
fn frame_energy_linear(frame: &[f64]) -> f64 {
    if frame.is_empty() {
        return 0.0;
    }
    frame.iter().map(|&x| x * x).sum::<f64>() / frame.len() as f64
}

/// Convert linear power to dB. Returns -300 for zero/negative input.
fn linear_to_db(p: f64) -> f64 {
    if p <= 0.0 {
        -300.0
    } else {
        10.0 * p.log10()
    }
}

/// Sigmoid function for smooth thresholding.
fn sigmoid(x: f64, steepness: f64) -> f64 {
    1.0 / (1.0 + (-steepness * x).exp())
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a sine wave frame.
    fn sine_frame(n: usize, amplitude: f64, freq_hz: f64, sample_rate: f64) -> Vec<f64> {
        (0..n)
            .map(|i| amplitude * (2.0 * PI * freq_hz * i as f64 / sample_rate).sin())
            .collect()
    }

    /// Helper: generate a silence frame with very small amplitude.
    fn silence_frame(n: usize) -> Vec<f64> {
        vec![1e-6; n]
    }

    /// Helper: generate a pseudo-random noise frame using an LCG.
    fn noise_frame(n: usize, amplitude: f64, seed: u64) -> Vec<f64> {
        let mut state = seed;
        (0..n)
            .map(|_| {
                state = state
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                let val = (state >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0;
                val * amplitude
            })
            .collect()
    }

    #[test]
    fn test_construction_and_initial_state() {
        let config = VadConfig {
            frame_size: 160,
            energy_threshold_db: -30.0,
            spectral_flatness_threshold: 0.5,
            hangover_frames: 5,
            sample_rate: 8000.0,
        };
        let vad = VoiceActivityDetector::new(config);
        assert!(!vad.is_speech());
        assert_eq!(vad.speech_probability(), 0.0);
    }

    #[test]
    fn test_detect_speech() {
        // A strong sine tone has high energy and low spectral flatness.
        let config = VadConfig {
            frame_size: 256,
            energy_threshold_db: -30.0,
            spectral_flatness_threshold: 0.8,
            hangover_frames: 0,
            sample_rate: 8000.0,
        };
        let mut vad = VoiceActivityDetector::new(config);

        let speech = sine_frame(256, 0.8, 300.0, 8000.0);
        let state = vad.process_frame(&speech);
        assert_eq!(state, VadState::Speech, "Strong tone should be detected as speech");
        assert!(vad.is_speech());
        assert!(vad.speech_probability() > 0.3);
    }

    #[test]
    fn test_detect_silence() {
        let config = VadConfig {
            frame_size: 160,
            energy_threshold_db: -30.0,
            spectral_flatness_threshold: 0.5,
            hangover_frames: 0,
            sample_rate: 8000.0,
        };
        let mut vad = VoiceActivityDetector::new(config);

        let silence = silence_frame(160);
        let state = vad.process_frame(&silence);
        assert_eq!(state, VadState::Silent, "Low-energy frame should be silent");
        assert!(!vad.is_speech());
    }

    #[test]
    fn test_hangover_keeps_speech_active() {
        let config = VadConfig {
            frame_size: 256,
            energy_threshold_db: -30.0,
            spectral_flatness_threshold: 0.8,
            hangover_frames: 3,
            sample_rate: 8000.0,
        };
        let mut vad = VoiceActivityDetector::new(config);

        // First frame: speech
        let speech = sine_frame(256, 0.8, 300.0, 8000.0);
        let state = vad.process_frame(&speech);
        assert_eq!(state, VadState::Speech);

        // Next 3 frames: silence, but hangover should keep it in Transition.
        let silence = silence_frame(256);
        for i in 0..3 {
            let state = vad.process_frame(&silence);
            assert_eq!(
                state,
                VadState::Transition,
                "Hangover frame {} should be Transition",
                i
            );
            assert!(vad.is_speech(), "is_speech() should be true during hangover");
        }

        // Fourth silence frame: hangover exhausted, should go to Silent.
        let state = vad.process_frame(&silence);
        assert_eq!(state, VadState::Silent, "After hangover, should be Silent");
        assert!(!vad.is_speech());
    }

    #[test]
    fn test_frame_energy_db_computation() {
        // A frame of all 1.0 has energy = 1.0, which is 0 dB.
        let frame = vec![1.0; 100];
        let db = frame_energy_db(&frame);
        assert!(
            (db - 0.0).abs() < 1e-10,
            "Energy of all-ones should be 0 dB, got {}",
            db
        );

        // A frame of all 0.1 has energy = 0.01, which is -20 dB.
        let frame2 = vec![0.1; 100];
        let db2 = frame_energy_db(&frame2);
        assert!(
            (db2 - (-20.0)).abs() < 1e-10,
            "Energy of 0.1 amplitude should be -20 dB, got {}",
            db2
        );

        // Empty frame returns very low dB.
        let db3 = frame_energy_db(&[]);
        assert!(db3 < -200.0, "Empty frame should have very low dB, got {}", db3);
    }

    #[test]
    fn test_zero_crossing_rate_computation() {
        // A frame that alternates +1, -1 should have ZCR = 1.0.
        let alternating: Vec<f64> = (0..100).map(|i| if i % 2 == 0 { 1.0 } else { -1.0 }).collect();
        let zcr = zero_crossing_rate(&alternating);
        assert!(
            (zcr - 1.0).abs() < 1e-10,
            "Alternating signal should have ZCR=1.0, got {}",
            zcr
        );

        // A constant positive frame should have ZCR = 0.0.
        let constant = vec![1.0; 100];
        let zcr2 = zero_crossing_rate(&constant);
        assert!(
            (zcr2 - 0.0).abs() < 1e-10,
            "Constant signal should have ZCR=0.0, got {}",
            zcr2
        );

        // A sine wave at Nyquist/4 should have ZCR around 0.25.
        // (1000 Hz at 8000 Hz sample rate => period of 8 samples => ~2 crossings per 8 samples)
        let sine = sine_frame(800, 1.0, 1000.0, 8000.0);
        let zcr3 = zero_crossing_rate(&sine);
        assert!(
            (zcr3 - 0.25).abs() < 0.05,
            "1 kHz sine at 8 kHz should have ZCR near 0.25, got {}",
            zcr3
        );

        // Single sample or empty: ZCR = 0.
        assert_eq!(zero_crossing_rate(&[]), 0.0);
        assert_eq!(zero_crossing_rate(&[1.0]), 0.0);
    }

    #[test]
    fn test_spectral_flatness_computation() {
        // A pure sine wave should have low spectral flatness (near 0).
        let sine = sine_frame(256, 1.0, 500.0, 8000.0);
        let sfm_sine = spectral_flatness(&sine);
        assert!(
            sfm_sine < 0.3,
            "Pure sine should have low SFM, got {}",
            sfm_sine
        );

        // Pseudo-random noise should have higher spectral flatness.
        let noise = noise_frame(256, 1.0, 42);
        let sfm_noise = spectral_flatness(&noise);
        assert!(
            sfm_noise > sfm_sine,
            "Noise SFM ({}) should be greater than sine SFM ({})",
            sfm_noise,
            sfm_sine
        );

        // Empty frame.
        assert_eq!(spectral_flatness(&[]), 0.0);
    }

    #[test]
    fn test_adaptive_vad_threshold_adaptation() {
        let mut avad = AdaptiveVad::new(160, 8000.0);

        // Feed many silence frames to let the noise floor converge.
        let silence = silence_frame(160);
        for _ in 0..50 {
            avad.process_frame(&silence);
        }

        let noise_floor_1 = avad.noise_floor_db();
        let threshold_1 = avad.threshold_db();

        // Threshold should be margin_db above noise floor.
        assert!(
            (threshold_1 - noise_floor_1 - 6.0).abs() < 1.0,
            "Threshold should be ~6 dB above noise floor: threshold={}, noise_floor={}",
            threshold_1,
            noise_floor_1
        );

        // Now feed louder broadband noise to shift the noise floor upward.
        // Use pseudo-random noise (high SFM) so it's still classified as
        // silence by the energy+SFM criterion, but has higher energy than
        // the initial silence frames.
        let louder_noise = noise_frame(160, 0.01, 99);
        for _ in 0..100 {
            avad.process_frame(&louder_noise);
        }

        let noise_floor_2 = avad.noise_floor_db();
        assert!(
            noise_floor_2 > noise_floor_1,
            "Noise floor should rise with louder noise: before={}, after={}",
            noise_floor_1,
            noise_floor_2
        );
    }

    #[test]
    fn test_telephone_vad_factory() {
        let vad = telephone_vad();
        assert_eq!(vad.config.frame_size, 160, "Telephone frame size should be 160");
        assert!(
            (vad.config.sample_rate - 8000.0).abs() < 1e-10,
            "Telephone sample rate should be 8000 Hz"
        );
        // 160 samples at 8 kHz = 20 ms
        let frame_duration_ms = vad.config.frame_size as f64 / vad.config.sample_rate * 1000.0;
        assert!(
            (frame_duration_ms - 20.0).abs() < 1e-10,
            "Telephone frame duration should be 20 ms, got {} ms",
            frame_duration_ms
        );
    }

    #[test]
    fn test_wideband_vad_factory() {
        let vad = wideband_vad();
        assert_eq!(vad.config.frame_size, 320, "Wideband frame size should be 320");
        assert!(
            (vad.config.sample_rate - 16000.0).abs() < 1e-10,
            "Wideband sample rate should be 16000 Hz"
        );
        // 320 samples at 16 kHz = 20 ms
        let frame_duration_ms = vad.config.frame_size as f64 / vad.config.sample_rate * 1000.0;
        assert!(
            (frame_duration_ms - 20.0).abs() < 1e-10,
            "Wideband frame duration should be 20 ms, got {} ms",
            frame_duration_ms
        );
    }
}
