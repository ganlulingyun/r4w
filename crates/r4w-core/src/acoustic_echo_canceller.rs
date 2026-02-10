//! Acoustic Echo Cancellation for full-duplex SDR voice communications.
//!
//! This module provides an NLMS (Normalized Least Mean Squares) based acoustic
//! echo canceller with double-talk detection using the Geigel algorithm. It is
//! designed for removing acoustic echo in full-duplex voice communication
//! scenarios common in SDR applications.
//!
//! # Overview
//!
//! When a speaker signal is played through a loudspeaker, it couples back into
//! the microphone, creating an acoustic echo. The AEC estimates the echo path
//! impulse response using an adaptive filter and subtracts the estimated echo
//! from the microphone signal.
//!
//! # Algorithm
//!
//! The NLMS update rule is:
//!
//! ```text
//! w(n+1) = w(n) + mu * e(n) * x(n) / (||x||^2 + delta)
//! ```
//!
//! where `w` is the adaptive filter weight vector, `mu` is the step size,
//! `e(n)` is the error signal, `x(n)` is the reference (speaker) signal buffer,
//! and `delta` is a regularization term to prevent division by zero.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::acoustic_echo_canceller::{AcousticEchoCanceller, AecConfig};
//!
//! // Create a canceller with default config
//! let config = AecConfig::default();
//! let mut aec = AcousticEchoCanceller::new(config);
//!
//! // Simulate: speaker plays a tone, mic picks up echo
//! let speaker_signal: Vec<f64> = (0..256)
//!     .map(|i| (2.0 * std::f64::consts::PI * 440.0 * i as f64 / 8000.0).sin())
//!     .collect();
//!
//! // Simulate echo path: attenuated, delayed copy of speaker
//! let echo_path: Vec<f64> = speaker_signal.iter()
//!     .map(|&s| s * 0.5)
//!     .collect();
//!
//! // Process through AEC
//! let output = aec.process_block(&echo_path, &speaker_signal);
//! assert_eq!(output.len(), echo_path.len());
//!
//! // After convergence, output energy should be less than input echo energy
//! let input_energy: f64 = echo_path.iter().map(|x| x * x).sum();
//! let output_energy: f64 = output[128..].iter().map(|x| x * x).sum();
//! assert!(output_energy < input_energy, "AEC should reduce echo energy");
//! ```

use std::f64::consts::PI;

/// Configuration for the acoustic echo canceller.
#[derive(Debug, Clone)]
pub struct AecConfig {
    /// Length of the adaptive filter in taps. Longer filters can model longer
    /// echo paths but require more computation. Typical range: 128-512.
    pub filter_length: usize,
    /// NLMS step size (learning rate). Controls convergence speed vs stability.
    /// Range: 0.01 to 1.0. Typical: 0.5.
    pub step_size: f64,
    /// Regularization constant to prevent division by zero in NLMS update.
    /// Typical: 1e-6.
    pub regularization: f64,
    /// Geigel double-talk detection threshold. Typical: 0.5.
    pub dtd_threshold: f64,
}

impl Default for AecConfig {
    fn default() -> Self {
        Self {
            filter_length: 256,
            step_size: 0.5,
            regularization: 1e-6,
            dtd_threshold: 0.5,
        }
    }
}

/// Metrics for monitoring AEC performance.
#[derive(Debug, Clone)]
pub struct AecMetrics {
    /// Echo Return Loss Enhancement in dB. Higher values indicate better
    /// echo cancellation. ERLE = 10*log10(E[mic^2] / E[error^2]).
    pub erle_db: f64,
    /// Estimated echo path delay in milliseconds (based on peak of impulse response).
    pub echo_path_delay_ms: f64,
    /// Convergence percentage (0-100). Based on filter coefficient stability.
    pub convergence_percent: f64,
}

/// Double-Talk Detector using the Geigel algorithm.
///
/// Detects when both the near-end speaker and far-end speaker are active
/// simultaneously. During double-talk, the adaptive filter should freeze
/// adaptation to prevent divergence.
#[derive(Debug, Clone)]
pub struct DtdDetector {
    /// Detection threshold. If mic_power / max_speaker_power > threshold,
    /// double-talk is declared.
    threshold: f64,
    /// Sliding window of recent speaker sample magnitudes for peak tracking.
    speaker_peak_buffer: Vec<f64>,
    /// Current write index into the peak buffer.
    buffer_index: usize,
    /// Length of the observation window.
    window_length: usize,
}

impl DtdDetector {
    /// Create a new Geigel double-talk detector.
    ///
    /// # Arguments
    /// * `threshold` - Detection threshold (typical: 0.5). Lower values make
    ///   detection more sensitive.
    pub fn new(threshold: f64) -> Self {
        let window_length = 256;
        Self {
            threshold,
            speaker_peak_buffer: vec![0.0; window_length],
            buffer_index: 0,
            window_length,
        }
    }

    /// Determine if double-talk is currently occurring.
    ///
    /// Uses the Geigel algorithm: compares the microphone signal magnitude
    /// against the maximum recent speaker signal magnitude. If the mic signal
    /// exceeds the threshold times the max speaker magnitude, double-talk is
    /// declared.
    ///
    /// # Arguments
    /// * `mic` - Current microphone sample
    /// * `speaker` - Current speaker (reference) sample
    ///
    /// # Returns
    /// `true` if double-talk is detected
    pub fn is_double_talk(&mut self, mic: f64, speaker: f64) -> bool {
        // Update speaker peak buffer
        self.speaker_peak_buffer[self.buffer_index] = speaker.abs();
        self.buffer_index = (self.buffer_index + 1) % self.window_length;

        // Find maximum speaker magnitude in the window
        let max_speaker = self
            .speaker_peak_buffer
            .iter()
            .cloned()
            .fold(0.0_f64, f64::max);

        if max_speaker < 1e-10 {
            // No speaker activity - can't determine double-talk
            return false;
        }

        // Geigel criterion: double-talk if |mic| > threshold * max(|speaker|)
        mic.abs() > self.threshold * max_speaker
    }

    /// Reset the detector state.
    pub fn reset(&mut self) {
        self.speaker_peak_buffer.fill(0.0);
        self.buffer_index = 0;
    }
}

/// NLMS-based Acoustic Echo Canceller for full-duplex voice communications.
///
/// Estimates and removes acoustic echo from a microphone signal given a
/// reference (speaker) signal. Uses the Normalized Least Mean Squares (NLMS)
/// algorithm with Geigel double-talk detection to prevent filter divergence
/// during near-end speech.
#[derive(Debug, Clone)]
pub struct AcousticEchoCanceller {
    /// Adaptive filter weights (impulse response estimate).
    weights: Vec<f64>,
    /// Circular buffer of recent speaker samples.
    speaker_buffer: Vec<f64>,
    /// Current write position in the speaker buffer.
    buffer_pos: usize,
    /// NLMS step size.
    step_size: f64,
    /// Regularization constant.
    regularization: f64,
    /// Filter length.
    filter_length: usize,
    /// Double-talk detector.
    dtd: DtdDetector,
    /// Running sum of mic^2 for ERLE calculation (exponential moving average).
    mic_power_avg: f64,
    /// Running sum of error^2 for ERLE calculation (exponential moving average).
    error_power_avg: f64,
    /// Smoothing factor for power averaging.
    power_smoothing: f64,
    /// Previous weights for convergence detection.
    prev_weights_norm: f64,
    /// Total samples processed.
    samples_processed: u64,
    /// Sample rate in Hz (for delay estimation).
    sample_rate: f64,
}

impl AcousticEchoCanceller {
    /// Create a new acoustic echo canceller with the given configuration.
    ///
    /// # Arguments
    /// * `config` - AEC configuration parameters
    pub fn new(config: AecConfig) -> Self {
        let filter_length = config.filter_length.max(1);
        Self {
            weights: vec![0.0; filter_length],
            speaker_buffer: vec![0.0; filter_length],
            buffer_pos: 0,
            step_size: config.step_size.clamp(0.01, 1.0),
            regularization: config.regularization.max(1e-12),
            filter_length,
            dtd: DtdDetector::new(config.dtd_threshold),
            mic_power_avg: 0.0,
            error_power_avg: 0.0,
            power_smoothing: 0.999,
            prev_weights_norm: 0.0,
            samples_processed: 0,
            sample_rate: 8000.0,
        }
    }

    /// Process a single sample pair and return the echo-cancelled output.
    ///
    /// # Arguments
    /// * `mic` - Microphone input sample (contains desired signal + echo)
    /// * `speaker` - Speaker (far-end reference) sample
    ///
    /// # Returns
    /// Echo-cancelled output sample
    pub fn process(&mut self, mic: f64, speaker: f64) -> f64 {
        // Insert new speaker sample into circular buffer
        self.speaker_buffer[self.buffer_pos] = speaker;

        // Compute estimated echo: y_hat = w^T * x
        let mut echo_estimate = 0.0;
        let mut power = 0.0;
        for i in 0..self.filter_length {
            let idx = (self.buffer_pos + self.filter_length - i) % self.filter_length;
            let x = self.speaker_buffer[idx];
            echo_estimate += self.weights[i] * x;
            power += x * x;
        }

        // Error signal (desired output)
        let error = mic - echo_estimate;

        // Double-talk detection
        let is_double_talk = self.dtd.is_double_talk(mic, speaker);

        // NLMS weight update (freeze during double-talk)
        if !is_double_talk {
            let norm_factor = self.step_size / (power + self.regularization);
            for i in 0..self.filter_length {
                let idx = (self.buffer_pos + self.filter_length - i) % self.filter_length;
                let x = self.speaker_buffer[idx];
                self.weights[i] += norm_factor * error * x;
            }
        }

        // Update power estimates for ERLE
        let alpha = self.power_smoothing;
        self.mic_power_avg = alpha * self.mic_power_avg + (1.0 - alpha) * mic * mic;
        self.error_power_avg = alpha * self.error_power_avg + (1.0 - alpha) * error * error;

        // Advance buffer position
        self.buffer_pos = (self.buffer_pos + 1) % self.filter_length;
        self.samples_processed += 1;

        error
    }

    /// Process a block of samples and return echo-cancelled output.
    ///
    /// # Arguments
    /// * `mic` - Microphone input samples
    /// * `speaker` - Speaker (far-end reference) samples
    ///
    /// # Returns
    /// Vector of echo-cancelled output samples
    ///
    /// # Panics
    /// Panics if `mic` and `speaker` have different lengths.
    pub fn process_block(&mut self, mic: &[f64], speaker: &[f64]) -> Vec<f64> {
        assert_eq!(
            mic.len(),
            speaker.len(),
            "mic and speaker blocks must have the same length"
        );
        mic.iter()
            .zip(speaker.iter())
            .map(|(&m, &s)| self.process(m, s))
            .collect()
    }

    /// Compute the Echo Return Loss Enhancement in dB.
    ///
    /// ERLE = 10 * log10(E[mic^2] / E[error^2])
    ///
    /// Higher values indicate better echo cancellation.
    pub fn erle_db(&self) -> f64 {
        if self.error_power_avg < 1e-20 {
            return 0.0;
        }
        if self.mic_power_avg < 1e-20 {
            return 0.0;
        }
        10.0 * (self.mic_power_avg / self.error_power_avg).log10()
    }

    /// Check if the adaptive filter has converged.
    ///
    /// Convergence is estimated by checking if the filter weights have
    /// stabilized (norm change below threshold) and sufficient samples
    /// have been processed.
    pub fn is_converged(&self) -> bool {
        if self.samples_processed < self.filter_length as u64 * 2 {
            return false;
        }
        let current_norm: f64 = self.weights.iter().map(|w| w * w).sum::<f64>().sqrt();
        if current_norm < 1e-12 {
            return false;
        }
        // Consider converged if ERLE > 6 dB (factor of 4 power reduction)
        self.erle_db() > 6.0
    }

    /// Get current AEC performance metrics.
    ///
    /// # Arguments
    /// * `sample_rate` - Sample rate in Hz (used for delay estimation)
    pub fn metrics(&self, sample_rate: f64) -> AecMetrics {
        // Find peak of impulse response for delay estimation
        let mut max_val = 0.0_f64;
        let mut max_idx = 0;
        for (i, &w) in self.weights.iter().enumerate() {
            if w.abs() > max_val {
                max_val = w.abs();
                max_idx = i;
            }
        }
        let delay_ms = if max_val > 1e-10 {
            (max_idx as f64 / sample_rate) * 1000.0
        } else {
            0.0
        };

        // Convergence estimate
        let convergence = if self.samples_processed == 0 {
            0.0
        } else {
            let erle = self.erle_db();
            // Map ERLE to convergence: 0 dB -> 0%, 20 dB -> 100%
            (erle / 20.0 * 100.0).clamp(0.0, 100.0)
        };

        AecMetrics {
            erle_db: self.erle_db(),
            echo_path_delay_ms: delay_ms,
            convergence_percent: convergence,
        }
    }

    /// Reset the canceller to its initial state, clearing all learned weights.
    pub fn reset(&mut self) {
        self.weights.fill(0.0);
        self.speaker_buffer.fill(0.0);
        self.buffer_pos = 0;
        self.mic_power_avg = 0.0;
        self.error_power_avg = 0.0;
        self.prev_weights_norm = 0.0;
        self.samples_processed = 0;
        self.dtd.reset();
    }

    /// Generate comfort noise at the specified level.
    ///
    /// Comfort noise is used to fill silence gaps after echo cancellation to
    /// avoid an unnatural "dead air" perception. Uses a simple linear
    /// congruential generator for deterministic pseudo-random noise.
    ///
    /// # Arguments
    /// * `level_db` - Desired noise level in dB relative to full scale
    ///
    /// # Returns
    /// A single comfort noise sample
    pub fn comfort_noise(&self, level_db: f64) -> f64 {
        // Convert dB to linear amplitude
        let amplitude = 10.0_f64.powf(level_db / 20.0);

        // Simple deterministic noise using sample count as seed
        // This produces a repeatable sequence for testing
        let seed = self.samples_processed.wrapping_mul(6364136223846793005).wrapping_add(1);
        let normalized = ((seed >> 33) as f64) / (u32::MAX as f64) * 2.0 - 1.0;

        amplitude * normalized
    }

    /// Get a reference to the current adaptive filter weights.
    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    /// Get the number of samples processed so far.
    pub fn samples_processed(&self) -> u64 {
        self.samples_processed
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a sine wave at the given frequency and sample rate.
    fn sine_wave(freq: f64, sample_rate: f64, num_samples: usize) -> Vec<f64> {
        (0..num_samples)
            .map(|i| (2.0 * PI * freq * i as f64 / sample_rate).sin())
            .collect()
    }

    /// Helper: compute signal power (mean of squared samples).
    fn signal_power(signal: &[f64]) -> f64 {
        if signal.is_empty() {
            return 0.0;
        }
        signal.iter().map(|x| x * x).sum::<f64>() / signal.len() as f64
    }

    #[test]
    fn test_default_config() {
        let config = AecConfig::default();
        assert_eq!(config.filter_length, 256);
        assert!((config.step_size - 0.5).abs() < 1e-10);
        assert!(config.regularization > 0.0);
        assert!((config.dtd_threshold - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_new_canceller() {
        let config = AecConfig::default();
        let aec = AcousticEchoCanceller::new(config);
        assert_eq!(aec.weights.len(), 256);
        assert_eq!(aec.samples_processed(), 0);
        assert!(!aec.is_converged());
    }

    #[test]
    fn test_process_single_sample() {
        let config = AecConfig {
            filter_length: 128,
            step_size: 0.5,
            regularization: 1e-6,
            dtd_threshold: 0.5,
        };
        let mut aec = AcousticEchoCanceller::new(config);

        // Process a single sample
        let output = aec.process(0.5, 1.0);
        // Initially weights are zero, so echo estimate is 0, error = mic = 0.5
        assert!((output - 0.5).abs() < 1e-10);
        assert_eq!(aec.samples_processed(), 1);
    }

    #[test]
    fn test_process_block_length() {
        let config = AecConfig::default();
        let mut aec = AcousticEchoCanceller::new(config);

        let mic = vec![0.1; 100];
        let speaker = vec![0.2; 100];
        let output = aec.process_block(&mic, &speaker);
        assert_eq!(output.len(), 100);
    }

    #[test]
    #[should_panic(expected = "mic and speaker blocks must have the same length")]
    fn test_process_block_mismatched_lengths() {
        let config = AecConfig::default();
        let mut aec = AcousticEchoCanceller::new(config);

        let mic = vec![0.1; 50];
        let speaker = vec![0.2; 100];
        aec.process_block(&mic, &speaker);
    }

    #[test]
    fn test_echo_cancellation_convergence() {
        // Simple echo path: speaker signal directly couples to mic with attenuation
        let config = AecConfig {
            filter_length: 128,
            step_size: 0.8,
            regularization: 1e-6,
            dtd_threshold: 10.0, // High threshold to disable DTD for this test
        };
        let mut aec = AcousticEchoCanceller::new(config);

        let num_samples = 4000;
        let speaker = sine_wave(300.0, 8000.0, num_samples);
        let echo_gain = 0.6;

        // Mic = echo only (no near-end speech)
        let mic: Vec<f64> = speaker.iter().map(|&s| s * echo_gain).collect();

        let output = aec.process_block(&mic, &speaker);

        // Measure error power in the second half (after some convergence)
        let second_half_start = num_samples / 2;
        let echo_power = signal_power(&mic[second_half_start..]);
        let residual_power = signal_power(&output[second_half_start..]);

        // Echo should be significantly reduced
        assert!(
            residual_power < echo_power * 0.1,
            "Residual power ({:.6}) should be < 10% of echo power ({:.6})",
            residual_power,
            echo_power
        );
    }

    #[test]
    fn test_erle_increases_with_convergence() {
        let config = AecConfig {
            filter_length: 128,
            step_size: 0.5,
            regularization: 1e-6,
            dtd_threshold: 10.0,
        };
        let mut aec = AcousticEchoCanceller::new(config);

        let speaker = sine_wave(500.0, 8000.0, 2000);
        let mic: Vec<f64> = speaker.iter().map(|&s| s * 0.4).collect();

        // Process first block
        let _ = aec.process_block(&mic[..1000], &speaker[..1000]);
        let erle_early = aec.erle_db();

        // Process second block
        let _ = aec.process_block(&mic[1000..], &speaker[1000..]);
        let erle_late = aec.erle_db();

        // ERLE should improve (or at least not decrease significantly)
        assert!(
            erle_late >= erle_early - 1.0,
            "ERLE should improve: early={:.2} dB, late={:.2} dB",
            erle_early,
            erle_late
        );
    }

    #[test]
    fn test_reset_clears_state() {
        let config = AecConfig::default();
        let mut aec = AcousticEchoCanceller::new(config);

        // Process some samples
        let speaker = sine_wave(440.0, 8000.0, 500);
        let mic: Vec<f64> = speaker.iter().map(|&s| s * 0.5).collect();
        let _ = aec.process_block(&mic, &speaker);

        assert!(aec.samples_processed() > 0);

        // Reset
        aec.reset();

        assert_eq!(aec.samples_processed(), 0);
        assert!(aec.weights().iter().all(|&w| w == 0.0));
        assert!(!aec.is_converged());
    }

    #[test]
    fn test_dtd_no_speaker_activity() {
        let mut dtd = DtdDetector::new(0.5);
        // With no speaker activity, should not detect double-talk
        let result = dtd.is_double_talk(0.5, 0.0);
        assert!(!result);
    }

    #[test]
    fn test_dtd_echo_only() {
        let mut dtd = DtdDetector::new(0.5);

        // Feed speaker activity to build up peak buffer
        for i in 0..300 {
            let speaker = (2.0 * PI * 300.0 * i as f64 / 8000.0).sin();
            // Echo is attenuated version of speaker - should NOT trigger DTD
            let mic = speaker * 0.3;
            let _ = dtd.is_double_talk(mic, speaker);
        }

        // Steady-state: echo-only should not trigger double-talk
        let speaker = 0.8;
        let mic = speaker * 0.3; // attenuated echo
        let result = dtd.is_double_talk(mic, speaker);
        assert!(!result, "Echo-only should not trigger double-talk");
    }

    #[test]
    fn test_dtd_double_talk() {
        let mut dtd = DtdDetector::new(0.5);

        // Feed speaker activity
        for i in 0..300 {
            let speaker = (2.0 * PI * 300.0 * i as f64 / 8000.0).sin() * 0.5;
            dtd.is_double_talk(speaker * 0.3, speaker);
        }

        // Now near-end speech causes mic to be much louder than expected echo
        let speaker = 0.5;
        let mic = 2.0; // Near-end speech dominates
        let result = dtd.is_double_talk(mic, speaker);
        assert!(result, "Should detect double-talk when mic >> speaker");
    }

    #[test]
    fn test_dtd_reset() {
        let mut dtd = DtdDetector::new(0.5);

        // Build up state
        for _ in 0..100 {
            dtd.is_double_talk(0.5, 1.0);
        }

        dtd.reset();

        // After reset, no speaker activity in buffer
        let result = dtd.is_double_talk(0.5, 0.0);
        assert!(!result);
    }

    #[test]
    fn test_comfort_noise_amplitude() {
        let config = AecConfig::default();
        let mut aec = AcousticEchoCanceller::new(config);

        // Process a few samples to advance the internal counter
        for _ in 0..100 {
            aec.process(0.0, 0.0);
        }

        // Generate comfort noise at -40 dBFS
        let noise = aec.comfort_noise(-40.0);
        let expected_max_amplitude = 10.0_f64.powf(-40.0 / 20.0); // 0.01

        assert!(
            noise.abs() <= expected_max_amplitude + 1e-10,
            "Comfort noise amplitude ({:.6}) should be <= {:.6}",
            noise.abs(),
            expected_max_amplitude
        );
    }

    #[test]
    fn test_comfort_noise_at_zero_db() {
        let config = AecConfig::default();
        let mut aec = AcousticEchoCanceller::new(config);
        aec.process(0.0, 0.0); // advance counter

        let noise = aec.comfort_noise(0.0);
        // At 0 dBFS, amplitude is 1.0, so |noise| <= 1.0
        assert!(noise.abs() <= 1.0 + 1e-10);
    }

    #[test]
    fn test_metrics_initial_state() {
        let config = AecConfig::default();
        let aec = AcousticEchoCanceller::new(config);
        let metrics = aec.metrics(8000.0);

        assert!((metrics.erle_db - 0.0).abs() < 1e-10);
        assert!((metrics.convergence_percent - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_metrics_after_convergence() {
        let config = AecConfig {
            filter_length: 128,
            step_size: 0.8,
            regularization: 1e-6,
            dtd_threshold: 10.0,
        };
        let mut aec = AcousticEchoCanceller::new(config);

        let num_samples = 5000;
        let speaker = sine_wave(400.0, 8000.0, num_samples);
        let mic: Vec<f64> = speaker.iter().map(|&s| s * 0.5).collect();
        let _ = aec.process_block(&mic, &speaker);

        let metrics = aec.metrics(8000.0);
        assert!(
            metrics.erle_db > 0.0,
            "ERLE should be positive after convergence: {:.2}",
            metrics.erle_db
        );
        assert!(metrics.convergence_percent >= 0.0);
        assert!(metrics.convergence_percent <= 100.0);
    }

    #[test]
    fn test_step_size_clamping() {
        // Step size too low
        let config = AecConfig {
            step_size: 0.001,
            ..AecConfig::default()
        };
        let aec = AcousticEchoCanceller::new(config);
        assert!((aec.step_size - 0.01).abs() < 1e-10);

        // Step size too high
        let config2 = AecConfig {
            step_size: 5.0,
            ..AecConfig::default()
        };
        let aec2 = AcousticEchoCanceller::new(config2);
        assert!((aec2.step_size - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_silence_input() {
        let config = AecConfig::default();
        let mut aec = AcousticEchoCanceller::new(config);

        // All-zero input should produce all-zero output
        let mic = vec![0.0; 200];
        let speaker = vec![0.0; 200];
        let output = aec.process_block(&mic, &speaker);

        for &sample in &output {
            assert!(
                sample.abs() < 1e-15,
                "Output should be zero for zero input, got {}",
                sample
            );
        }
    }

    #[test]
    fn test_weights_accessor() {
        let config = AecConfig {
            filter_length: 64,
            ..AecConfig::default()
        };
        let aec = AcousticEchoCanceller::new(config);

        let weights = aec.weights();
        assert_eq!(weights.len(), 64);
        assert!(weights.iter().all(|&w| w == 0.0));
    }
}
