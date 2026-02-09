//! Power Squelch
//!
//! Gates signal output based on input power level. When the input power
//! drops below a threshold, the output is zeroed. Essential for PTT radio
//! receivers and burst-mode communications.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::squelch::{PowerSquelch, SquelchConfig};
//! use num_complex::Complex64;
//!
//! let mut sq = PowerSquelch::new(SquelchConfig {
//!     threshold_db: -20.0,  // Gate signals below -20 dBFS
//!     alpha: 0.5,           // Fast power tracking
//!     ramp_samples: 0,      // Instant gate transitions
//! });
//!
//! let loud = vec![Complex64::new(1.0, 0.0); 50];
//! let output = sq.process_block(&loud);
//! // After first sample (power converges), output passes through
//! assert!(output.last().unwrap().norm() > 0.5);
//! ```

use num_complex::Complex64;

/// Configuration for power squelch.
#[derive(Debug, Clone)]
pub struct SquelchConfig {
    /// Power threshold in dB (relative to full scale). Default: -40.0 dBFS
    pub threshold_db: f64,
    /// Smoothing factor for power estimate (0-1). Higher = faster. Default: 0.01
    pub alpha: f64,
    /// Ramp time in samples for gate on/off transitions. Default: 10
    pub ramp_samples: usize,
}

impl Default for SquelchConfig {
    fn default() -> Self {
        Self {
            threshold_db: -40.0,
            alpha: 0.01,
            ramp_samples: 10,
        }
    }
}

/// Power squelch gate.
///
/// Continuously estimates input power using a single-pole IIR filter.
/// When power exceeds the threshold, the gate opens (output = input).
/// When power drops below, the gate closes (output = zero).
///
/// The ramp feature provides smooth transitions to avoid clicks.
///
/// This is equivalent to GNU Radio's `pwr_squelch_cc`.
#[derive(Debug, Clone)]
pub struct PowerSquelch {
    config: SquelchConfig,
    /// Smoothed power estimate (linear)
    power_estimate: f64,
    /// Threshold in linear scale
    threshold_linear: f64,
    /// Current gate state (0.0 = closed, 1.0 = open)
    gate: f64,
    /// Ramp increment per sample
    ramp_inc: f64,
    /// Whether the gate is currently open
    is_open: bool,
}

impl PowerSquelch {
    /// Create a new power squelch.
    pub fn new(config: SquelchConfig) -> Self {
        let threshold_linear = 10.0f64.powf(config.threshold_db / 10.0);
        let ramp_inc = if config.ramp_samples > 0 {
            1.0 / config.ramp_samples as f64
        } else {
            1.0
        };
        Self {
            config,
            power_estimate: 0.0,
            threshold_linear,
            gate: 0.0,
            ramp_inc,
            is_open: false,
        }
    }

    /// Check if the squelch gate is currently open.
    pub fn is_open(&self) -> bool {
        self.is_open
    }

    /// Get the current estimated power in dB.
    pub fn power_db(&self) -> f64 {
        if self.power_estimate > 1e-30 {
            10.0 * self.power_estimate.log10()
        } else {
            -300.0
        }
    }

    /// Process a single sample.
    pub fn process(&mut self, input: Complex64) -> Complex64 {
        // Update power estimate (single-pole IIR)
        let inst_power = input.norm_sqr();
        self.power_estimate =
            (1.0 - self.config.alpha) * self.power_estimate + self.config.alpha * inst_power;

        // Update gate state
        self.is_open = self.power_estimate > self.threshold_linear;

        // Ramp gate up/down
        if self.is_open {
            self.gate = (self.gate + self.ramp_inc).min(1.0);
        } else {
            self.gate = (self.gate - self.ramp_inc).max(0.0);
        }

        input * self.gate
    }

    /// Process a block of samples.
    pub fn process_block(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Process samples in place.
    pub fn process_inplace(&mut self, samples: &mut [Complex64]) {
        for s in samples.iter_mut() {
            *s = self.process(*s);
        }
    }

    /// Reset squelch to initial state.
    pub fn reset(&mut self) {
        self.power_estimate = 0.0;
        self.gate = 0.0;
        self.is_open = false;
    }
}

impl crate::filters::Filter for PowerSquelch {
    fn process(&mut self, input: Complex64) -> Complex64 {
        PowerSquelch::process(self, input)
    }

    fn reset(&mut self) {
        PowerSquelch::reset(self);
    }

    fn group_delay(&self) -> f64 {
        0.0
    }

    fn order(&self) -> usize {
        0
    }
}

/// Standard EIA/TIA CTCSS tones (Hz).
/// 38 tones from 67.0 Hz to 250.3 Hz.
pub const CTCSS_TONES: &[f64] = &[
    67.0, 71.9, 74.4, 77.0, 79.7, 82.5, 85.4, 88.5, 91.5, 94.8,
    97.4, 100.0, 103.5, 107.2, 110.9, 114.8, 118.8, 123.0, 127.3, 131.8,
    136.5, 141.3, 146.2, 151.4, 156.7, 162.2, 167.9, 173.8, 179.9, 186.2,
    192.8, 203.5, 210.7, 218.1, 225.7, 233.6, 241.8, 250.3,
];

/// CTCSS tone squelch configuration.
#[derive(Debug, Clone)]
pub struct CtcssConfig {
    /// Required CTCSS tone frequency (Hz). Must be one of `CTCSS_TONES`.
    pub tone_freq: f64,
    /// Sample rate of the audio stream (Hz).
    pub sample_rate: f64,
    /// Detection threshold (tone power / noise floor ratio). Default: 4.0
    pub threshold: f64,
    /// Goertzel block size. Default: 0 (auto-calculated for ~50ms blocks).
    pub block_size: usize,
    /// Ramp time in samples for gate transitions. Default: 50
    pub ramp_samples: usize,
}

impl Default for CtcssConfig {
    fn default() -> Self {
        Self {
            tone_freq: 100.0,
            sample_rate: 48000.0,
            threshold: 4.0,
            block_size: 0,
            ramp_samples: 50,
        }
    }
}

/// CTCSS tone-coded squelch.
///
/// Opens the audio gate only when the correct sub-audible CTCSS tone is
/// detected. Uses a Goertzel filter for efficient single-frequency detection.
///
/// This is equivalent to GNU Radio's `ctcss_squelch_ff`.
///
/// ## Example
///
/// ```rust
/// use r4w_core::squelch::{CtcssSquelch, CtcssConfig};
///
/// let mut sq = CtcssSquelch::new(CtcssConfig {
///     tone_freq: 100.0,
///     sample_rate: 48000.0,
///     threshold: 4.0,
///     ..Default::default()
/// });
///
/// // Generate audio with 100 Hz CTCSS tone
/// let samples: Vec<f64> = (0..4800)
///     .map(|i| {
///         let t = i as f64 / 48000.0;
///         0.1 * (2.0 * std::f64::consts::PI * 100.0 * t).sin() + 0.5 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin()
///     })
///     .collect();
///
/// let output = sq.process_block(&samples);
/// ```
#[derive(Debug, Clone)]
pub struct CtcssSquelch {
    /// Required tone frequency
    tone_freq: f64,
    /// Sample rate
    sample_rate: f64,
    /// Detection threshold
    threshold: f64,
    /// Block size for Goertzel
    block_size: usize,
    /// Goertzel coefficient
    coeff: f64,
    /// Goertzel state
    s1: f64,
    s2: f64,
    /// Sample counter within current block
    count: usize,
    /// Current gate state
    gate: f64,
    /// Ramp increment
    ramp_inc: f64,
    /// Whether gate is open
    is_open: bool,
    /// Last detected tone power
    tone_power: f64,
    /// Average signal power
    avg_power: f64,
}

impl CtcssSquelch {
    /// Create a new CTCSS squelch.
    pub fn new(config: CtcssConfig) -> Self {
        let block_size = if config.block_size == 0 {
            // Auto: ~50ms blocks, rounded to nearest integer number of tone cycles
            let cycles = (config.tone_freq * 0.05).round().max(2.0);
            (cycles * config.sample_rate / config.tone_freq).round() as usize
        } else {
            config.block_size
        };

        let k = config.tone_freq / config.sample_rate * block_size as f64;
        let coeff = 2.0 * (2.0 * std::f64::consts::PI * k / block_size as f64).cos();

        let ramp_inc = if config.ramp_samples > 0 {
            1.0 / config.ramp_samples as f64
        } else {
            1.0
        };

        Self {
            tone_freq: config.tone_freq,
            sample_rate: config.sample_rate,
            threshold: config.threshold,
            block_size,
            coeff,
            s1: 0.0,
            s2: 0.0,
            count: 0,
            gate: 0.0,
            ramp_inc,
            is_open: false,
            tone_power: 0.0,
            avg_power: 0.0,
        }
    }

    /// Process a single audio sample.
    pub fn process(&mut self, input: f64) -> f64 {
        // Accumulate Goertzel state
        let s0 = input + self.coeff * self.s1 - self.s2;
        self.s2 = self.s1;
        self.s1 = s0;

        // Track average power
        self.avg_power = 0.99 * self.avg_power + 0.01 * input * input;

        self.count += 1;

        if self.count >= self.block_size {
            // Complete Goertzel: compute magnitude squared
            let power = self.s1 * self.s1 + self.s2 * self.s2
                - self.coeff * self.s1 * self.s2;
            self.tone_power = power / (self.block_size as f64 * self.block_size as f64);

            // Normalize by average power
            let ratio = if self.avg_power > 1e-20 {
                self.tone_power / self.avg_power
            } else {
                0.0
            };

            self.is_open = ratio > self.threshold;

            // Reset for next block
            self.s1 = 0.0;
            self.s2 = 0.0;
            self.count = 0;
        }

        // Ramp gate
        if self.is_open {
            self.gate = (self.gate + self.ramp_inc).min(1.0);
        } else {
            self.gate = (self.gate - self.ramp_inc).max(0.0);
        }

        input * self.gate
    }

    /// Process a block of audio samples.
    pub fn process_block(&mut self, input: &[f64]) -> Vec<f64> {
        input.iter().map(|&s| self.process(s)).collect()
    }

    /// Check if squelch is currently open.
    pub fn is_open(&self) -> bool {
        self.is_open
    }

    /// Get the detected tone power.
    pub fn tone_power(&self) -> f64 {
        self.tone_power
    }

    /// Get the required tone frequency.
    pub fn tone_freq(&self) -> f64 {
        self.tone_freq
    }

    /// Find the closest standard CTCSS tone to a given frequency.
    pub fn closest_tone(freq: f64) -> f64 {
        CTCSS_TONES
            .iter()
            .copied()
            .min_by(|a, b| (a - freq).abs().partial_cmp(&(b - freq).abs()).unwrap())
            .unwrap_or(100.0)
    }

    /// Reset to initial state.
    pub fn reset(&mut self) {
        self.s1 = 0.0;
        self.s2 = 0.0;
        self.count = 0;
        self.gate = 0.0;
        self.is_open = false;
        self.tone_power = 0.0;
        self.avg_power = 0.0;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_squelch_passes_loud_signal() {
        let mut sq = PowerSquelch::new(SquelchConfig {
            threshold_db: -20.0,
            alpha: 0.1,
            ramp_samples: 0, // instant gate
            ..Default::default()
        });

        // Strong signal (well above threshold)
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 100];
        let output = sq.process_block(&input);

        // After initial convergence, output should pass through
        let last = output.last().unwrap();
        assert!(
            last.norm() > 0.5,
            "Loud signal should pass: got {:.3}",
            last.norm()
        );
        assert!(sq.is_open());
    }

    #[test]
    fn test_squelch_blocks_quiet_signal() {
        let mut sq = PowerSquelch::new(SquelchConfig {
            threshold_db: -20.0,
            alpha: 0.1,
            ramp_samples: 0,
            ..Default::default()
        });

        // Very weak signal (below threshold)
        let input: Vec<Complex64> = vec![Complex64::new(0.001, 0.0); 200];
        let output = sq.process_block(&input);

        // Output should be gated
        let last = output.last().unwrap();
        assert!(
            last.norm() < 0.01,
            "Weak signal should be squelched: got {:.3}",
            last.norm()
        );
        assert!(!sq.is_open());
    }

    #[test]
    fn test_squelch_ramp() {
        let ramp = 20;
        let mut sq = PowerSquelch::new(SquelchConfig {
            threshold_db: -30.0,
            alpha: 1.0, // instant power tracking
            ramp_samples: ramp,
        });

        // Start with strong signal
        let input: Vec<Complex64> = vec![Complex64::new(1.0, 0.0); 50];
        let output = sq.process_block(&input);

        // First sample should not be at full amplitude due to ramp
        assert!(
            output[0].norm() < 0.2,
            "Ramp should start low: got {:.3}",
            output[0].norm()
        );
        // After ramp, should be at full amplitude
        assert!(
            output[ramp + 1].norm() > 0.9,
            "Should reach full amplitude after ramp"
        );
    }

    #[test]
    fn test_squelch_power_db() {
        let mut sq = PowerSquelch::new(SquelchConfig {
            alpha: 1.0, // instant tracking
            ..Default::default()
        });

        let _ = sq.process(Complex64::new(1.0, 0.0));
        assert!(
            (sq.power_db() - 0.0).abs() < 0.1,
            "Unit amplitude should be ~0 dBFS: got {:.1}",
            sq.power_db()
        );
    }

    #[test]
    fn test_squelch_reset() {
        let mut sq = PowerSquelch::new(SquelchConfig::default());
        let _ = sq.process_block(&vec![Complex64::new(1.0, 0.0); 100]);
        assert!(sq.is_open());

        sq.reset();
        assert!(!sq.is_open());
        assert!(sq.power_db() < -100.0);
    }

    #[test]
    fn test_squelch_filter_trait() {
        use crate::filters::Filter;
        let mut sq = PowerSquelch::new(SquelchConfig {
            alpha: 1.0,
            ramp_samples: 0,
            threshold_db: -30.0,
        });
        let output = Filter::process(&mut sq, Complex64::new(1.0, 0.0));
        assert!(output.norm() > 0.0);
    }

    // ===== CTCSS tests =====

    #[test]
    fn test_ctcss_tones_count() {
        assert_eq!(CTCSS_TONES.len(), 38);
        assert!((CTCSS_TONES[0] - 67.0).abs() < 0.1);
        assert!((CTCSS_TONES[37] - 250.3).abs() < 0.1);
    }

    #[test]
    fn test_ctcss_opens_with_correct_tone() {
        let fs = 48000.0;
        let tone = 100.0;
        let mut sq = CtcssSquelch::new(CtcssConfig {
            tone_freq: tone,
            sample_rate: fs,
            threshold: 0.5,
            ramp_samples: 0,
            ..Default::default()
        });

        // Audio with strong CTCSS tone (100 Hz sub-audible only, no other audio)
        // Need enough samples: ~50ms blocks at 48kHz = ~2400 samples/block
        // Need several blocks for power estimation to converge
        let n = 48000; // 1 second — several detection blocks
        let input: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                0.5 * (2.0 * std::f64::consts::PI * tone * t).sin()
            })
            .collect();

        let output = sq.process_block(&input);

        // After multiple detection blocks, squelch should open
        assert!(
            sq.is_open(),
            "CTCSS squelch should open with correct tone (tone_power={:.6}, avg_power={:.6})",
            sq.tone_power(),
            sq.avg_power
        );
        // Samples near the end should pass through (after gate opens)
        let tail_power: f64 = output[n - 1000..]
            .iter()
            .map(|x| x * x)
            .sum::<f64>()
            / 1000.0;
        assert!(
            tail_power > 0.01,
            "Audio should pass through when squelch is open: tail_power={:.6}",
            tail_power
        );
    }

    #[test]
    fn test_ctcss_closed_without_tone() {
        let fs = 48000.0;
        let mut sq = CtcssSquelch::new(CtcssConfig {
            tone_freq: 100.0,
            sample_rate: fs,
            threshold: 2.0,
            ramp_samples: 0,
            ..Default::default()
        });

        // Audio without any CTCSS tone (just 1 kHz tone)
        let n = 9600;
        let input: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                0.5 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin()
            })
            .collect();

        let _output = sq.process_block(&input);

        assert!(
            !sq.is_open(),
            "CTCSS squelch should stay closed without correct tone"
        );
    }

    #[test]
    fn test_ctcss_closed_with_wrong_tone() {
        let fs = 48000.0;
        let mut sq = CtcssSquelch::new(CtcssConfig {
            tone_freq: 100.0, // Looking for 100 Hz
            sample_rate: fs,
            threshold: 2.0,
            ramp_samples: 0,
            ..Default::default()
        });

        // Audio with wrong CTCSS tone (150 Hz instead of 100 Hz)
        let n = 9600;
        let input: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                0.15 * (2.0 * std::f64::consts::PI * 150.0 * t).sin()
                    + 0.5 * (2.0 * std::f64::consts::PI * 1000.0 * t).sin()
            })
            .collect();

        let _output = sq.process_block(&input);

        assert!(
            !sq.is_open(),
            "CTCSS squelch should stay closed with wrong tone"
        );
    }

    #[test]
    fn test_ctcss_closest_tone() {
        assert!((CtcssSquelch::closest_tone(99.5) - 100.0).abs() < 0.1);
        assert!((CtcssSquelch::closest_tone(67.5) - 67.0).abs() < 0.1);
        assert!((CtcssSquelch::closest_tone(250.0) - 250.3).abs() < 0.5);
    }

    #[test]
    fn test_ctcss_reset() {
        let mut sq = CtcssSquelch::new(CtcssConfig::default());

        // Process some signal
        let input: Vec<f64> = (0..4800)
            .map(|i| (2.0 * std::f64::consts::PI * 100.0 * i as f64 / 48000.0).sin())
            .collect();
        sq.process_block(&input);

        sq.reset();
        assert!(!sq.is_open());
        assert!(sq.tone_power() < 1e-10);
    }
}
