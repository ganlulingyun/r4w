//! Dynamic Range Compressor — Audio/RF Dynamics Processing
//!
//! Implements compressor, limiter, expander, and noise gate for
//! controlling signal dynamic range. Uses logarithmic gain computation
//! with configurable attack/release time constants, knee width,
//! and ratio. Supports both RMS and peak detection modes.
//!
//! Useful for audio processing, AGC-like behavior, and RF signal
//! conditioning in SDR receive chains.
//!
//! GNU Radio equivalent: `gr-audio` dynamic range blocks, `gr::analog::agc`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::dynamic_range_compressor::{DynamicsProcessor, DynamicsMode, DetectorType};
//!
//! let mut comp = DynamicsProcessor::new(
//!     DynamicsMode::Compressor { ratio: 4.0 },
//!     -20.0,  // threshold in dB
//!     DetectorType::Rms,
//!     48000.0,
//! );
//! comp.set_attack_ms(10.0);
//! comp.set_release_ms(100.0);
//!
//! let input: Vec<f64> = (0..480).map(|i| 0.8 * (i as f64 * 0.1).sin()).collect();
//! let output = comp.process(&input);
//! assert_eq!(output.len(), input.len());
//! ```

/// Dynamics processing mode.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DynamicsMode {
    /// Compressor: reduce gain above threshold.
    /// `ratio` = input_change_dB / output_change_dB (e.g., 4:1 means 4dB input → 1dB output).
    Compressor { ratio: f64 },
    /// Limiter: hard ceiling at threshold (infinite ratio).
    Limiter,
    /// Expander: reduce gain below threshold.
    /// `ratio` = output_change_dB / input_change_dB below threshold.
    Expander { ratio: f64 },
    /// Noise gate: mute below threshold with hysteresis.
    Gate { hysteresis_db: f64 },
}

/// Level detection method.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectorType {
    /// Peak detection (faster response).
    Peak,
    /// RMS detection (average power, smoother).
    Rms,
}

/// Dynamic range processor with attack/release envelope follower.
#[derive(Debug, Clone)]
pub struct DynamicsProcessor {
    mode: DynamicsMode,
    /// Threshold in dBFS.
    threshold_db: f64,
    /// Knee width in dB (0 = hard knee).
    knee_db: f64,
    /// Level detector type.
    detector: DetectorType,
    /// Sample rate in Hz.
    sample_rate: f64,
    /// Attack coefficient (0..1).
    attack_coeff: f64,
    /// Release coefficient (0..1).
    release_coeff: f64,
    /// Makeup gain in dB.
    makeup_gain_db: f64,
    /// Envelope follower state.
    envelope_db: f64,
    /// Gate state (for hysteresis).
    gate_open: bool,
    /// RMS accumulator.
    rms_sum: f64,
    /// RMS window size.
    rms_window: usize,
    /// RMS buffer.
    rms_buffer: Vec<f64>,
    /// RMS buffer position.
    rms_pos: usize,
}

impl DynamicsProcessor {
    /// Create a new dynamics processor.
    ///
    /// - `mode`: Processing mode (Compressor, Limiter, Expander, Gate)
    /// - `threshold_db`: Level threshold in dBFS
    /// - `detector`: Peak or RMS level detection
    /// - `sample_rate`: Audio/signal sample rate in Hz
    pub fn new(
        mode: DynamicsMode,
        threshold_db: f64,
        detector: DetectorType,
        sample_rate: f64,
    ) -> Self {
        let rms_window = (sample_rate * 0.01) as usize; // 10ms RMS window
        Self {
            mode,
            threshold_db,
            knee_db: 0.0,
            detector,
            sample_rate,
            attack_coeff: time_to_coeff(10.0, sample_rate),
            release_coeff: time_to_coeff(100.0, sample_rate),
            makeup_gain_db: 0.0,
            envelope_db: -120.0,
            gate_open: false,
            rms_sum: 0.0,
            rms_window: rms_window.max(1),
            rms_buffer: vec![0.0; rms_window.max(1)],
            rms_pos: 0,
        }
    }

    /// Set attack time in milliseconds.
    pub fn set_attack_ms(&mut self, ms: f64) {
        self.attack_coeff = time_to_coeff(ms, self.sample_rate);
    }

    /// Set release time in milliseconds.
    pub fn set_release_ms(&mut self, ms: f64) {
        self.release_coeff = time_to_coeff(ms, self.sample_rate);
    }

    /// Set soft knee width in dB (0 = hard knee).
    pub fn set_knee_db(&mut self, knee: f64) {
        self.knee_db = knee.max(0.0);
    }

    /// Set makeup gain in dB.
    pub fn set_makeup_gain_db(&mut self, gain: f64) {
        self.makeup_gain_db = gain;
    }

    /// Process a block of samples, returning gain-adjusted output.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let mut output = Vec::with_capacity(input.len());
        for &sample in input {
            output.push(self.process_sample(sample));
        }
        output
    }

    /// Process a single sample.
    pub fn process_sample(&mut self, sample: f64) -> f64 {
        let level_db = self.detect_level(sample);
        self.update_envelope(level_db);
        let gain_db = self.compute_gain(self.envelope_db) + self.makeup_gain_db;
        let gain_linear = db_to_linear(gain_db);
        sample * gain_linear
    }

    /// Detect instantaneous level in dB.
    fn detect_level(&mut self, sample: f64) -> f64 {
        match self.detector {
            DetectorType::Peak => {
                let abs = sample.abs();
                if abs < 1e-30 { -120.0 } else { 20.0 * abs.log10() }
            }
            DetectorType::Rms => {
                let sq = sample * sample;
                // Sliding window RMS
                self.rms_sum -= self.rms_buffer[self.rms_pos];
                self.rms_buffer[self.rms_pos] = sq;
                self.rms_sum += sq;
                self.rms_pos = (self.rms_pos + 1) % self.rms_window;
                let rms = (self.rms_sum / self.rms_window as f64).sqrt();
                if rms < 1e-30 { -120.0 } else { 20.0 * rms.log10() }
            }
        }
    }

    /// Update the smoothed envelope with attack/release.
    fn update_envelope(&mut self, level_db: f64) {
        let coeff = if level_db > self.envelope_db {
            self.attack_coeff
        } else {
            self.release_coeff
        };
        self.envelope_db = coeff * self.envelope_db + (1.0 - coeff) * level_db;
    }

    /// Compute gain reduction in dB based on mode and threshold.
    fn compute_gain(&mut self, level_db: f64) -> f64 {
        match self.mode {
            DynamicsMode::Compressor { ratio } => {
                self.compressor_gain(level_db, ratio)
            }
            DynamicsMode::Limiter => {
                self.compressor_gain(level_db, f64::INFINITY)
            }
            DynamicsMode::Expander { ratio } => {
                self.expander_gain(level_db, ratio)
            }
            DynamicsMode::Gate { hysteresis_db } => {
                self.gate_gain(level_db, hysteresis_db)
            }
        }
    }

    /// Compressor/limiter gain computation with optional soft knee.
    fn compressor_gain(&self, level_db: f64, ratio: f64) -> f64 {
        let t = self.threshold_db;
        let k = self.knee_db;

        if k < 0.01 {
            // Hard knee
            if level_db <= t {
                0.0
            } else if ratio.is_infinite() {
                t - level_db // Limiter: clamp to threshold
            } else {
                (1.0 / ratio - 1.0) * (level_db - t)
            }
        } else {
            // Soft knee
            let lower = t - k / 2.0;
            let upper = t + k / 2.0;
            if level_db <= lower {
                0.0
            } else if level_db >= upper {
                if ratio.is_infinite() {
                    t - level_db
                } else {
                    (1.0 / ratio - 1.0) * (level_db - t)
                }
            } else {
                // Quadratic interpolation in knee region
                let x = level_db - lower;
                (1.0 / ratio - 1.0) * x * x / (2.0 * k)
            }
        }
    }

    /// Expander gain computation.
    fn expander_gain(&self, level_db: f64, ratio: f64) -> f64 {
        if level_db >= self.threshold_db {
            0.0
        } else {
            (1.0 - ratio) * (self.threshold_db - level_db)
        }
    }

    /// Gate gain computation with hysteresis.
    fn gate_gain(&mut self, level_db: f64, hysteresis_db: f64) -> f64 {
        if self.gate_open {
            if level_db < self.threshold_db - hysteresis_db {
                self.gate_open = false;
                -120.0 // Mute
            } else {
                0.0
            }
        } else if level_db > self.threshold_db {
            self.gate_open = true;
            0.0
        } else {
            -120.0 // Mute
        }
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.envelope_db = -120.0;
        self.gate_open = false;
        self.rms_sum = 0.0;
        self.rms_buffer.fill(0.0);
        self.rms_pos = 0;
    }

    /// Get the current gain reduction in dB.
    pub fn gain_reduction_db(&self) -> f64 {
        -self.compute_gain_readonly(self.envelope_db)
    }

    /// Read-only gain computation (doesn't modify gate state).
    fn compute_gain_readonly(&self, level_db: f64) -> f64 {
        match self.mode {
            DynamicsMode::Compressor { ratio } => self.compressor_gain(level_db, ratio),
            DynamicsMode::Limiter => self.compressor_gain(level_db, f64::INFINITY),
            DynamicsMode::Expander { ratio } => self.expander_gain(level_db, ratio),
            DynamicsMode::Gate { hysteresis_db } => {
                if level_db > self.threshold_db { 0.0 } else { -120.0 }
            }
        }
    }

    /// Get the threshold in dB.
    pub fn threshold_db(&self) -> f64 {
        self.threshold_db
    }

    /// Set the threshold in dB.
    pub fn set_threshold_db(&mut self, db: f64) {
        self.threshold_db = db;
    }
}

/// Convert time constant (ms) to one-pole IIR coefficient.
fn time_to_coeff(ms: f64, sample_rate: f64) -> f64 {
    if ms <= 0.0 {
        return 0.0;
    }
    let samples = ms * sample_rate / 1000.0;
    (-1.0 / samples).exp()
}

/// Convert dB to linear amplitude.
fn db_to_linear(db: f64) -> f64 {
    10.0_f64.powf(db / 20.0)
}

/// Convert linear amplitude to dB.
pub fn linear_to_db(linear: f64) -> f64 {
    if linear < 1e-30 {
        -120.0
    } else {
        20.0 * linear.log10()
    }
}

/// Compute the static gain curve for a compressor (for visualization).
///
/// Returns (input_db, output_db) pairs from `db_min` to `db_max`.
pub fn compression_curve(
    threshold_db: f64,
    ratio: f64,
    knee_db: f64,
    db_min: f64,
    db_max: f64,
    num_points: usize,
) -> Vec<(f64, f64)> {
    let step = (db_max - db_min) / (num_points - 1).max(1) as f64;
    (0..num_points)
        .map(|i| {
            let input_db = db_min + i as f64 * step;
            let output_db = if knee_db < 0.01 {
                if input_db <= threshold_db {
                    input_db
                } else {
                    threshold_db + (input_db - threshold_db) / ratio
                }
            } else {
                let lower = threshold_db - knee_db / 2.0;
                let upper = threshold_db + knee_db / 2.0;
                if input_db <= lower {
                    input_db
                } else if input_db >= upper {
                    threshold_db + (input_db - threshold_db) / ratio
                } else {
                    let x = input_db - lower;
                    input_db + (1.0 / ratio - 1.0) * x * x / (2.0 * knee_db)
                }
            };
            (input_db, output_db)
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compressor_below_threshold() {
        let mut comp = DynamicsProcessor::new(
            DynamicsMode::Compressor { ratio: 4.0 },
            -10.0,
            DetectorType::Peak,
            48000.0,
        );
        // Very quiet signal should pass through unchanged
        let input = vec![0.01; 480]; // ~-40 dBFS
        let output = comp.process(&input);
        // Gain should be ~0 dB (no compression below threshold)
        let ratio = output.last().unwrap() / input.last().unwrap();
        assert!((ratio - 1.0).abs() < 0.5, "ratio={ratio}");
    }

    #[test]
    fn test_compressor_reduces_level() {
        let mut comp = DynamicsProcessor::new(
            DynamicsMode::Compressor { ratio: 10.0 },
            -20.0,
            DetectorType::Peak,
            48000.0,
        );
        comp.set_attack_ms(0.1); // Very fast attack
        // Loud signal at ~0 dBFS
        let input: Vec<f64> = (0..4800).map(|_| 0.9).collect();
        let output = comp.process(&input);
        // After attack transient, output should be quieter
        let tail_avg: f64 = output[2400..].iter().map(|x| x.abs()).sum::<f64>() / 2400.0;
        let input_avg: f64 = input[2400..].iter().map(|x| x.abs()).sum::<f64>() / 2400.0;
        assert!(tail_avg < input_avg, "tail={tail_avg} input={input_avg}");
    }

    #[test]
    fn test_limiter_ceiling() {
        let mut lim = DynamicsProcessor::new(
            DynamicsMode::Limiter,
            -6.0, // ~0.5 linear
            DetectorType::Peak,
            48000.0,
        );
        lim.set_attack_ms(0.01);
        let input: Vec<f64> = (0..4800).map(|_| 0.9).collect();
        let output = lim.process(&input);
        // After settling, output should be clamped near threshold
        let tail = &output[2400..];
        for &v in tail {
            assert!(v.abs() < 0.9, "v={v} exceeds limit after settling");
        }
    }

    #[test]
    fn test_gate_mutes_quiet() {
        let mut gate = DynamicsProcessor::new(
            DynamicsMode::Gate { hysteresis_db: 3.0 },
            -30.0,
            DetectorType::Peak,
            48000.0,
        );
        gate.set_attack_ms(0.01);
        gate.set_release_ms(0.01);
        // Very quiet signal
        let input = vec![0.001; 480]; // ~-60 dBFS
        let output = gate.process(&input);
        // Should be muted (near zero)
        let avg: f64 = output.iter().map(|x| x.abs()).sum::<f64>() / output.len() as f64;
        assert!(avg < 0.01, "avg={avg}");
    }

    #[test]
    fn test_expander_reduces_below_threshold() {
        let mut exp = DynamicsProcessor::new(
            DynamicsMode::Expander { ratio: 2.0 },
            -20.0,
            DetectorType::Peak,
            48000.0,
        );
        exp.set_attack_ms(0.01);
        // Signal below threshold
        let input: Vec<f64> = (0..4800).map(|_| 0.01).collect(); // ~-40 dBFS
        let output = exp.process(&input);
        // Should be reduced further
        let tail_avg: f64 = output[2400..].iter().map(|x| x.abs()).sum::<f64>() / 2400.0;
        assert!(tail_avg < 0.01);
    }

    #[test]
    fn test_time_to_coeff() {
        let c = time_to_coeff(10.0, 48000.0);
        assert!(c > 0.0 && c < 1.0);
        // Longer time = closer to 1.0
        let c_long = time_to_coeff(1000.0, 48000.0);
        assert!(c_long > c);
    }

    #[test]
    fn test_db_conversions() {
        assert!((db_to_linear(0.0) - 1.0).abs() < 1e-10);
        assert!((db_to_linear(-6.0) - 0.5012).abs() < 0.01);
        assert!((db_to_linear(-20.0) - 0.1).abs() < 1e-10);
        assert!((linear_to_db(1.0) - 0.0).abs() < 1e-10);
        assert!((linear_to_db(0.1) - (-20.0)).abs() < 1e-10);
    }

    #[test]
    fn test_compression_curve() {
        let curve = compression_curve(-20.0, 4.0, 0.0, -60.0, 0.0, 61);
        assert_eq!(curve.len(), 61);
        // Below threshold: output = input
        let (in_db, out_db) = curve[20]; // -40 dBFS
        assert!((in_db - (-40.0)).abs() < 0.1);
        assert!((out_db - (-40.0)).abs() < 0.1);
        // Above threshold: compressed
        let (in_db, out_db) = curve[60]; // 0 dBFS
        assert!((in_db - 0.0).abs() < 0.1);
        // At 4:1 ratio, 20dB over threshold → 5dB over threshold
        assert!((out_db - (-15.0)).abs() < 0.5, "out_db={out_db}");
    }

    #[test]
    fn test_soft_knee() {
        let hard = compression_curve(-20.0, 4.0, 0.0, -30.0, -10.0, 21);
        let soft = compression_curve(-20.0, 4.0, 10.0, -30.0, -10.0, 21);
        // Soft knee should differ from hard knee in the transition region
        let hard_mid = hard[10].1; // At threshold
        let soft_mid = soft[10].1;
        // Both should be similar at threshold but soft has smoother transition
        assert!((hard_mid - soft_mid).abs() < 2.0);
    }

    #[test]
    fn test_rms_detector() {
        let mut comp = DynamicsProcessor::new(
            DynamicsMode::Compressor { ratio: 4.0 },
            -20.0,
            DetectorType::Rms,
            48000.0,
        );
        let input: Vec<f64> = (0..4800).map(|i| 0.5 * (i as f64 * 0.1).sin()).collect();
        let output = comp.process(&input);
        assert_eq!(output.len(), input.len());
    }

    #[test]
    fn test_makeup_gain() {
        let mut comp = DynamicsProcessor::new(
            DynamicsMode::Compressor { ratio: 4.0 },
            -20.0,
            DetectorType::Peak,
            48000.0,
        );
        comp.set_makeup_gain_db(6.0);
        // Quiet signal with makeup gain should be amplified
        let input = vec![0.01; 480];
        let output = comp.process(&input);
        let ratio = output.last().unwrap() / input.last().unwrap();
        // 6 dB gain = ~2x
        assert!(ratio > 1.5, "ratio={ratio}");
    }

    #[test]
    fn test_reset() {
        let mut comp = DynamicsProcessor::new(
            DynamicsMode::Compressor { ratio: 4.0 },
            -20.0,
            DetectorType::Peak,
            48000.0,
        );
        comp.process(&[0.9; 480]);
        comp.reset();
        assert_eq!(comp.envelope_db, -120.0);
    }
}
