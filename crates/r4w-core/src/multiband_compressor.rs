//! Multi-Band Dynamic Range Compressor
//!
//! Splits an audio or RF signal into multiple frequency bands using
//! Linkwitz-Riley crossover filters, then applies independent dynamic range
//! compression to each band before summing the results. This allows
//! frequency-selective gain control—compressing loud bass without affecting
//! quiet treble, for example.
//!
//! Each band has its own threshold, ratio, attack/release times, makeup gain,
//! and detection mode (RMS or peak). Crossover filters are 4th-order
//! Linkwitz-Riley (cascaded 2nd-order Butterworth) for flat magnitude
//! response at crossover points.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::multiband_compressor::{
//!     MultibandCompressor, BandConfig, DetectionMode,
//! };
//!
//! let sample_rate = 48000.0;
//! let bands = vec![
//!     BandConfig {
//!         low_hz: 0.0,
//!         high_hz: 200.0,
//!         threshold_db: -20.0,
//!         ratio: 4.0,
//!         attack_ms: 10.0,
//!         release_ms: 100.0,
//!         makeup_gain_db: 2.0,
//!         detection: DetectionMode::Rms,
//!     },
//!     BandConfig {
//!         low_hz: 200.0,
//!         high_hz: 5000.0,
//!         threshold_db: -15.0,
//!         ratio: 3.0,
//!         attack_ms: 5.0,
//!         release_ms: 80.0,
//!         makeup_gain_db: 1.0,
//!         detection: DetectionMode::Peak,
//!     },
//!     BandConfig {
//!         low_hz: 5000.0,
//!         high_hz: 24000.0,
//!         threshold_db: -18.0,
//!         ratio: 2.5,
//!         attack_ms: 3.0,
//!         release_ms: 60.0,
//!         makeup_gain_db: 1.5,
//!         detection: DetectionMode::Rms,
//!     },
//! ];
//!
//! let mut compressor = MultibandCompressor::new(bands, sample_rate);
//! let input: Vec<f64> = (0..4800)
//!     .map(|i| 0.9 * (2.0 * std::f64::consts::PI * 440.0 * i as f64 / sample_rate).sin())
//!     .collect();
//! let output = compressor.process(&input);
//! assert_eq!(output.len(), input.len());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Level detection mode for a compressor band.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DetectionMode {
    /// Root-mean-square envelope follower.
    Rms,
    /// Peak (absolute value) envelope follower.
    Peak,
}

/// Configuration for a single frequency band.
#[derive(Debug, Clone)]
pub struct BandConfig {
    /// Lower edge frequency in Hz (0.0 for the lowest band).
    pub low_hz: f64,
    /// Upper edge frequency in Hz (Nyquist or above for the highest band).
    pub high_hz: f64,
    /// Compression threshold in dB (below 0 for typical usage).
    pub threshold_db: f64,
    /// Compression ratio (e.g. 4.0 means 4:1).
    pub ratio: f64,
    /// Attack time in milliseconds.
    pub attack_ms: f64,
    /// Release time in milliseconds.
    pub release_ms: f64,
    /// Makeup gain in dB applied after compression.
    pub makeup_gain_db: f64,
    /// Level detection mode.
    pub detection: DetectionMode,
}

/// Multi-band dynamic range compressor.
///
/// Splits the input into N frequency bands using Linkwitz-Riley crossover
/// filters, compresses each band independently, then sums the results.
pub struct MultibandCompressor {
    bands: Vec<BandState>,
    crossovers: Vec<LinkwitzRileyCrossover>,
    sample_rate: f64,
}

// ---------------------------------------------------------------------------
// Internal types
// ---------------------------------------------------------------------------

/// Per-band runtime state.
struct BandState {
    config: BandConfig,
    /// Smoothed envelope level (linear amplitude).
    envelope: f64,
    /// Current gain reduction in dB (positive = reduction).
    gain_reduction_db: f64,
    /// Attack coefficient (per-sample).
    attack_coeff: f64,
    /// Release coefficient (per-sample).
    release_coeff: f64,
    /// RMS accumulator (squared) for RMS mode.
    rms_sq: f64,
}

/// 4th-order Linkwitz-Riley crossover (two cascaded 2nd-order Butterworth sections).
/// Produces a low-pass and a high-pass output.
struct LinkwitzRileyCrossover {
    lp: [BiquadState; 2],
    hp: [BiquadState; 2],
    lp_coeffs: BiquadCoeffs,
    hp_coeffs: BiquadCoeffs,
}

/// Direct Form II transposed biquad state.
#[derive(Clone)]
struct BiquadState {
    z1: f64,
    z2: f64,
}

/// Biquad filter coefficients: H(z) = (b0 + b1*z^-1 + b2*z^-2) / (1 + a1*z^-1 + a2*z^-2)
#[derive(Clone)]
struct BiquadCoeffs {
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
}

// ---------------------------------------------------------------------------
// Biquad implementation
// ---------------------------------------------------------------------------

impl BiquadState {
    fn new() -> Self {
        Self { z1: 0.0, z2: 0.0 }
    }

    /// Process one sample through the biquad (Direct Form II transposed).
    fn process(&mut self, x: f64, c: &BiquadCoeffs) -> f64 {
        let y = c.b0 * x + self.z1;
        self.z1 = c.b1 * x - c.a1 * y + self.z2;
        self.z2 = c.b2 * x - c.a2 * y;
        y
    }
}

impl BiquadCoeffs {
    /// 2nd-order Butterworth low-pass via bilinear transform.
    fn butterworth_lp(freq_hz: f64, sample_rate: f64) -> Self {
        let wc = (PI * freq_hz / sample_rate).tan();
        let wc2 = wc * wc;
        let sqrt2_wc = std::f64::consts::SQRT_2 * wc;
        let norm = 1.0 / (1.0 + sqrt2_wc + wc2);

        Self {
            b0: wc2 * norm,
            b1: 2.0 * wc2 * norm,
            b2: wc2 * norm,
            a1: 2.0 * (wc2 - 1.0) * norm,
            a2: (1.0 - sqrt2_wc + wc2) * norm,
        }
    }

    /// 2nd-order Butterworth high-pass via bilinear transform.
    fn butterworth_hp(freq_hz: f64, sample_rate: f64) -> Self {
        let wc = (PI * freq_hz / sample_rate).tan();
        let wc2 = wc * wc;
        let sqrt2_wc = std::f64::consts::SQRT_2 * wc;
        let norm = 1.0 / (1.0 + sqrt2_wc + wc2);

        Self {
            b0: norm,
            b1: -2.0 * norm,
            b2: norm,
            a1: 2.0 * (wc2 - 1.0) * norm,
            a2: (1.0 - sqrt2_wc + wc2) * norm,
        }
    }
}

// ---------------------------------------------------------------------------
// Linkwitz-Riley crossover
// ---------------------------------------------------------------------------

impl LinkwitzRileyCrossover {
    /// Create a new LR4 crossover at the given frequency.
    fn new(freq_hz: f64, sample_rate: f64) -> Self {
        Self {
            lp: [BiquadState::new(), BiquadState::new()],
            hp: [BiquadState::new(), BiquadState::new()],
            lp_coeffs: BiquadCoeffs::butterworth_lp(freq_hz, sample_rate),
            hp_coeffs: BiquadCoeffs::butterworth_hp(freq_hz, sample_rate),
        }
    }

    /// Split one sample into (low, high).
    fn process(&mut self, x: f64) -> (f64, f64) {
        let lp_c = self.lp_coeffs.clone();
        let hp_c = self.hp_coeffs.clone();
        let lo_stage1 = self.lp[0].process(x, &lp_c);
        let lo = self.lp[1].process(lo_stage1, &lp_c);
        let hi_stage1 = self.hp[0].process(x, &hp_c);
        let hi = self.hp[1].process(hi_stage1, &hp_c);
        (lo, hi)
    }
}

// ---------------------------------------------------------------------------
// BandState helpers
// ---------------------------------------------------------------------------

impl BandState {
    fn new(config: BandConfig, sample_rate: f64) -> Self {
        let attack_coeff = time_constant(config.attack_ms, sample_rate);
        let release_coeff = time_constant(config.release_ms, sample_rate);
        Self {
            config,
            envelope: 0.0,
            gain_reduction_db: 0.0,
            attack_coeff,
            release_coeff,
            rms_sq: 0.0,
        }
    }

    /// Process a single band sample: detect level, compute gain, apply.
    fn compress(&mut self, x: f64) -> f64 {
        // --- level detection ---
        let level = match self.config.detection {
            DetectionMode::Peak => {
                let abs_x = x.abs();
                let coeff = if abs_x > self.envelope {
                    self.attack_coeff
                } else {
                    self.release_coeff
                };
                self.envelope = coeff * self.envelope + (1.0 - coeff) * abs_x;
                self.envelope
            }
            DetectionMode::Rms => {
                let x2 = x * x;
                let coeff = if x2 > self.rms_sq {
                    self.attack_coeff
                } else {
                    self.release_coeff
                };
                self.rms_sq = coeff * self.rms_sq + (1.0 - coeff) * x2;
                self.rms_sq.sqrt()
            }
        };

        // --- gain calculation in dB ---
        let level_db = linear_to_db(level);
        let overshoot = level_db - self.config.threshold_db;

        let gr = if overshoot > 0.0 {
            overshoot - overshoot / self.config.ratio
        } else {
            0.0
        };
        self.gain_reduction_db = gr;

        let gain_db = -gr + self.config.makeup_gain_db;
        let gain_linear = db_to_linear(gain_db);

        x * gain_linear
    }
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Convert a time in ms to a one-pole smoothing coefficient.
/// coefficient = exp(-1 / (time_s * sample_rate))
fn time_constant(time_ms: f64, sample_rate: f64) -> f64 {
    if time_ms <= 0.0 {
        return 0.0;
    }
    (-1.0 / (time_ms * 0.001 * sample_rate)).exp()
}

/// Convert linear amplitude to dB, clamped to -120 dB minimum.
fn linear_to_db(x: f64) -> f64 {
    if x <= 1e-6 {
        -120.0
    } else {
        20.0 * x.log10()
    }
}

/// Convert dB to linear amplitude.
fn db_to_linear(db: f64) -> f64 {
    10.0_f64.powf(db / 20.0)
}

// ---------------------------------------------------------------------------
// MultibandCompressor implementation
// ---------------------------------------------------------------------------

impl MultibandCompressor {
    /// Create a new multi-band compressor.
    ///
    /// `bands` must be sorted by frequency and contiguous. Crossover filters
    /// are inserted at each boundary between adjacent bands.
    ///
    /// # Panics
    ///
    /// Panics if `bands` is empty.
    pub fn new(bands: Vec<BandConfig>, sample_rate: f64) -> Self {
        assert!(!bands.is_empty(), "at least one band is required");

        // Build crossover filters at each band boundary.
        let mut crossovers = Vec::new();
        for i in 0..bands.len().saturating_sub(1) {
            let crossover_freq = bands[i].high_hz;
            crossovers.push(LinkwitzRileyCrossover::new(crossover_freq, sample_rate));
        }

        let band_states = bands
            .into_iter()
            .map(|c| BandState::new(c, sample_rate))
            .collect();

        Self {
            bands: band_states,
            crossovers,
            sample_rate,
        }
    }

    /// Process a block of mono samples, returning the compressed output.
    pub fn process(&mut self, input: &[f64]) -> Vec<f64> {
        let n_bands = self.bands.len();
        let n_samples = input.len();
        let mut output = vec![0.0; n_samples];

        if n_bands == 1 {
            // Single band: no crossover splitting needed.
            for (i, &x) in input.iter().enumerate() {
                output[i] = self.bands[0].compress(x);
            }
            return output;
        }

        // Multi-band: split with crossover tree, compress, sum.
        // For N bands we have N-1 crossover filters.
        // Strategy: cascade splits from low to high.
        //   crossover[0] splits into band[0] (low) and remainder (high)
        //   crossover[1] splits remainder into band[1] (low) and remainder (high)
        //   ... last remainder = band[N-1]
        for (i, &x) in input.iter().enumerate() {
            let mut remainder = x;
            for b in 0..n_bands - 1 {
                let (lo, hi) = self.crossovers[b].process(remainder);
                output[i] += self.bands[b].compress(lo);
                remainder = hi;
            }
            // Last band gets the final remainder.
            output[i] += self.bands[n_bands - 1].compress(remainder);
        }

        output
    }

    /// Return the current gain reduction (in dB, positive) for each band.
    pub fn gain_reduction_per_band(&self) -> Vec<f64> {
        self.bands.iter().map(|b| b.gain_reduction_db).collect()
    }

    /// Return the number of bands.
    pub fn num_bands(&self) -> usize {
        self.bands.len()
    }

    /// Return the sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate
    }

    /// Return an immutable reference to the band configurations.
    pub fn band_configs(&self) -> Vec<&BandConfig> {
        self.bands.iter().map(|b| &b.config).collect()
    }

    // ----- Presets -----------------------------------------------------------

    /// Preset for speech processing: 3 bands optimized for voice clarity.
    ///
    /// - Low band (0-300 Hz): gentle compression to reduce rumble
    /// - Mid band (300-3400 Hz): moderate compression for intelligibility
    /// - High band (3400-Nyquist): light compression for sibilance control
    pub fn preset_speech(sample_rate: f64) -> Self {
        let nyquist = sample_rate / 2.0;
        Self::new(
            vec![
                BandConfig {
                    low_hz: 0.0,
                    high_hz: 300.0,
                    threshold_db: -24.0,
                    ratio: 3.0,
                    attack_ms: 15.0,
                    release_ms: 150.0,
                    makeup_gain_db: 2.0,
                    detection: DetectionMode::Rms,
                },
                BandConfig {
                    low_hz: 300.0,
                    high_hz: 3400.0,
                    threshold_db: -18.0,
                    ratio: 4.0,
                    attack_ms: 8.0,
                    release_ms: 100.0,
                    makeup_gain_db: 3.0,
                    detection: DetectionMode::Rms,
                },
                BandConfig {
                    low_hz: 3400.0,
                    high_hz: nyquist,
                    threshold_db: -22.0,
                    ratio: 2.5,
                    attack_ms: 5.0,
                    release_ms: 80.0,
                    makeup_gain_db: 1.0,
                    detection: DetectionMode::Peak,
                },
            ],
            sample_rate,
        )
    }

    /// Preset for music mastering: 4 bands.
    ///
    /// - Sub (0-80 Hz): control low end
    /// - Low-mid (80-800 Hz): warmth
    /// - High-mid (800-6000 Hz): presence
    /// - High (6000-Nyquist): air / sparkle
    pub fn preset_music(sample_rate: f64) -> Self {
        let nyquist = sample_rate / 2.0;
        Self::new(
            vec![
                BandConfig {
                    low_hz: 0.0,
                    high_hz: 80.0,
                    threshold_db: -16.0,
                    ratio: 3.0,
                    attack_ms: 20.0,
                    release_ms: 200.0,
                    makeup_gain_db: 1.0,
                    detection: DetectionMode::Rms,
                },
                BandConfig {
                    low_hz: 80.0,
                    high_hz: 800.0,
                    threshold_db: -14.0,
                    ratio: 2.5,
                    attack_ms: 12.0,
                    release_ms: 120.0,
                    makeup_gain_db: 1.5,
                    detection: DetectionMode::Rms,
                },
                BandConfig {
                    low_hz: 800.0,
                    high_hz: 6000.0,
                    threshold_db: -12.0,
                    ratio: 3.0,
                    attack_ms: 8.0,
                    release_ms: 80.0,
                    makeup_gain_db: 2.0,
                    detection: DetectionMode::Peak,
                },
                BandConfig {
                    low_hz: 6000.0,
                    high_hz: nyquist,
                    threshold_db: -18.0,
                    ratio: 2.0,
                    attack_ms: 5.0,
                    release_ms: 60.0,
                    makeup_gain_db: 1.0,
                    detection: DetectionMode::Peak,
                },
            ],
            sample_rate,
        )
    }

    /// Preset for broadcast / loudness maximisation: 5 bands.
    pub fn preset_broadcast(sample_rate: f64) -> Self {
        let nyquist = sample_rate / 2.0;
        Self::new(
            vec![
                BandConfig {
                    low_hz: 0.0,
                    high_hz: 100.0,
                    threshold_db: -20.0,
                    ratio: 4.0,
                    attack_ms: 15.0,
                    release_ms: 200.0,
                    makeup_gain_db: 3.0,
                    detection: DetectionMode::Rms,
                },
                BandConfig {
                    low_hz: 100.0,
                    high_hz: 500.0,
                    threshold_db: -16.0,
                    ratio: 3.5,
                    attack_ms: 10.0,
                    release_ms: 150.0,
                    makeup_gain_db: 2.5,
                    detection: DetectionMode::Rms,
                },
                BandConfig {
                    low_hz: 500.0,
                    high_hz: 2000.0,
                    threshold_db: -14.0,
                    ratio: 3.0,
                    attack_ms: 8.0,
                    release_ms: 100.0,
                    makeup_gain_db: 2.0,
                    detection: DetectionMode::Rms,
                },
                BandConfig {
                    low_hz: 2000.0,
                    high_hz: 8000.0,
                    threshold_db: -16.0,
                    ratio: 3.0,
                    attack_ms: 5.0,
                    release_ms: 80.0,
                    makeup_gain_db: 2.0,
                    detection: DetectionMode::Peak,
                },
                BandConfig {
                    low_hz: 8000.0,
                    high_hz: nyquist,
                    threshold_db: -20.0,
                    ratio: 2.5,
                    attack_ms: 3.0,
                    release_ms: 60.0,
                    makeup_gain_db: 1.5,
                    detection: DetectionMode::Peak,
                },
            ],
            sample_rate,
        )
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const SR: f64 = 48000.0;

    fn sine_wave(freq: f64, amplitude: f64, n_samples: usize, sample_rate: f64) -> Vec<f64> {
        (0..n_samples)
            .map(|i| amplitude * (2.0 * PI * freq * i as f64 / sample_rate).sin())
            .collect()
    }

    fn rms(signal: &[f64]) -> f64 {
        (signal.iter().map(|x| x * x).sum::<f64>() / signal.len() as f64).sqrt()
    }

    fn single_band_config() -> Vec<BandConfig> {
        vec![BandConfig {
            low_hz: 0.0,
            high_hz: 24000.0,
            threshold_db: -20.0,
            ratio: 4.0,
            attack_ms: 5.0,
            release_ms: 50.0,
            makeup_gain_db: 0.0,
            detection: DetectionMode::Peak,
        }]
    }

    fn two_band_config() -> Vec<BandConfig> {
        vec![
            BandConfig {
                low_hz: 0.0,
                high_hz: 1000.0,
                threshold_db: -20.0,
                ratio: 4.0,
                attack_ms: 5.0,
                release_ms: 50.0,
                makeup_gain_db: 0.0,
                detection: DetectionMode::Rms,
            },
            BandConfig {
                low_hz: 1000.0,
                high_hz: 24000.0,
                threshold_db: -20.0,
                ratio: 4.0,
                attack_ms: 5.0,
                release_ms: 50.0,
                makeup_gain_db: 0.0,
                detection: DetectionMode::Rms,
            },
        ]
    }

    // 1. Basic construction and output length.
    #[test]
    fn test_output_length_matches_input() {
        let mut comp = MultibandCompressor::new(single_band_config(), SR);
        let input = sine_wave(440.0, 0.8, 4800, SR);
        let output = comp.process(&input);
        assert_eq!(output.len(), input.len());
    }

    // 2. Silent input produces silent output.
    #[test]
    fn test_silence_passthrough() {
        let mut comp = MultibandCompressor::new(single_band_config(), SR);
        let input = vec![0.0; 1000];
        let output = comp.process(&input);
        for &s in &output {
            assert!(s.abs() < 1e-10, "expected silence, got {}", s);
        }
    }

    // 3. Loud signal is compressed (RMS decreases relative to input).
    #[test]
    fn test_compression_reduces_loud_signal() {
        let mut comp = MultibandCompressor::new(single_band_config(), SR);
        let input = sine_wave(440.0, 0.9, 48000, SR);
        let output = comp.process(&input);
        // Skip attack transient (first 10%)
        let skip = 4800;
        let input_rms = rms(&input[skip..]);
        let output_rms = rms(&output[skip..]);
        assert!(
            output_rms < input_rms,
            "expected compression: output_rms={} < input_rms={}",
            output_rms,
            input_rms
        );
    }

    // 4. Quiet signal below threshold passes with minimal change (no compression).
    #[test]
    fn test_below_threshold_minimal_change() {
        let mut comp = MultibandCompressor::new(single_band_config(), SR);
        // -40 dB signal, threshold at -20 dB -> should not compress.
        let input = sine_wave(440.0, 0.01, 48000, SR);
        let output = comp.process(&input);
        let skip = 4800;
        let input_rms = rms(&input[skip..]);
        let output_rms = rms(&output[skip..]);
        let ratio = output_rms / input_rms;
        // Should be close to 1.0 (within 20% tolerance for filter transients).
        assert!(
            (ratio - 1.0).abs() < 0.2,
            "expected ~unity gain below threshold, ratio={}",
            ratio
        );
    }

    // 5. gain_reduction_per_band reports correct number of bands.
    #[test]
    fn test_gain_reduction_band_count() {
        let mut comp = MultibandCompressor::new(two_band_config(), SR);
        let input = sine_wave(440.0, 0.8, 4800, SR);
        let _ = comp.process(&input);
        let gr = comp.gain_reduction_per_band();
        assert_eq!(gr.len(), 2);
    }

    // 6. gain_reduction_per_band returns non-negative values.
    #[test]
    fn test_gain_reduction_nonnegative() {
        let mut comp = MultibandCompressor::new(two_band_config(), SR);
        let input = sine_wave(500.0, 0.9, 48000, SR);
        let _ = comp.process(&input);
        for (i, &gr) in comp.gain_reduction_per_band().iter().enumerate() {
            assert!(gr >= 0.0, "band {} gain reduction negative: {}", i, gr);
        }
    }

    // 7. Makeup gain increases output level.
    #[test]
    fn test_makeup_gain() {
        let bands_no_makeup = vec![BandConfig {
            low_hz: 0.0,
            high_hz: 24000.0,
            threshold_db: -10.0,
            ratio: 4.0,
            attack_ms: 5.0,
            release_ms: 50.0,
            makeup_gain_db: 0.0,
            detection: DetectionMode::Peak,
        }];
        let bands_with_makeup = vec![BandConfig {
            low_hz: 0.0,
            high_hz: 24000.0,
            threshold_db: -10.0,
            ratio: 4.0,
            attack_ms: 5.0,
            release_ms: 50.0,
            makeup_gain_db: 6.0,
            detection: DetectionMode::Peak,
        }];

        let input = sine_wave(440.0, 0.8, 48000, SR);
        let skip = 4800;

        let mut comp0 = MultibandCompressor::new(bands_no_makeup, SR);
        let out0 = comp0.process(&input);
        let rms0 = rms(&out0[skip..]);

        let mut comp1 = MultibandCompressor::new(bands_with_makeup, SR);
        let out1 = comp1.process(&input);
        let rms1 = rms(&out1[skip..]);

        assert!(
            rms1 > rms0,
            "makeup gain should increase output: rms_with={} > rms_without={}",
            rms1,
            rms0
        );
    }

    // 8. Preset speech creates 3 bands.
    #[test]
    fn test_preset_speech() {
        let comp = MultibandCompressor::preset_speech(SR);
        assert_eq!(comp.num_bands(), 3);
        assert_eq!(comp.sample_rate(), SR);
    }

    // 9. Preset music creates 4 bands.
    #[test]
    fn test_preset_music() {
        let comp = MultibandCompressor::preset_music(SR);
        assert_eq!(comp.num_bands(), 4);
    }

    // 10. Preset broadcast creates 5 bands.
    #[test]
    fn test_preset_broadcast() {
        let comp = MultibandCompressor::preset_broadcast(SR);
        assert_eq!(comp.num_bands(), 5);
    }

    // 11. Two-band crossover preserves signal energy (RMS check).
    #[test]
    fn test_crossover_reconstruction() {
        // With no compression (very high threshold), the total output energy
        // should be close to input energy. LR4 crossovers are power-complementary
        // (|LP|^2 + |HP|^2 = 1), so the summed output RMS should be close to input RMS
        // for frequencies away from the crossover point.
        let bands = vec![
            BandConfig {
                low_hz: 0.0,
                high_hz: 4000.0,
                threshold_db: 0.0,  // threshold at 0 dB -> practically no compression
                ratio: 1.0,        // 1:1 ratio = no compression
                attack_ms: 1.0,
                release_ms: 10.0,
                makeup_gain_db: 0.0,
                detection: DetectionMode::Peak,
            },
            BandConfig {
                low_hz: 4000.0,
                high_hz: 24000.0,
                threshold_db: 0.0,
                ratio: 1.0,
                attack_ms: 1.0,
                release_ms: 10.0,
                makeup_gain_db: 0.0,
                detection: DetectionMode::Peak,
            },
        ];

        let mut comp = MultibandCompressor::new(bands, SR);
        // Use a tone well below the crossover (100 Hz) so almost all energy is in band 0.
        let input = sine_wave(100.0, 0.5, 48000, SR);
        let output = comp.process(&input);

        // After filter settling, output RMS should be close to input RMS.
        let skip = 4800;
        let in_rms = rms(&input[skip..]);
        let out_rms = rms(&output[skip..]);
        let ratio = out_rms / in_rms;
        assert!(
            (ratio - 1.0).abs() < 0.15,
            "crossover reconstruction RMS ratio should be ~1.0, got {}",
            ratio
        );
    }

    // 12. Higher ratio produces more compression.
    #[test]
    fn test_higher_ratio_more_compression() {
        let make_band = |ratio: f64| {
            vec![BandConfig {
                low_hz: 0.0,
                high_hz: 24000.0,
                threshold_db: -20.0,
                ratio,
                attack_ms: 5.0,
                release_ms: 50.0,
                makeup_gain_db: 0.0,
                detection: DetectionMode::Peak,
            }]
        };

        let input = sine_wave(440.0, 0.9, 48000, SR);
        let skip = 4800;

        let mut comp_low = MultibandCompressor::new(make_band(2.0), SR);
        let out_low = comp_low.process(&input);
        let rms_low = rms(&out_low[skip..]);

        let mut comp_high = MultibandCompressor::new(make_band(8.0), SR);
        let out_high = comp_high.process(&input);
        let rms_high = rms(&out_high[skip..]);

        assert!(
            rms_high < rms_low,
            "higher ratio should compress more: rms_8:1={} < rms_2:1={}",
            rms_high,
            rms_low
        );
    }

    // 13. Peak vs RMS detection modes produce different envelopes.
    #[test]
    fn test_peak_vs_rms_detection() {
        let make_band = |det: DetectionMode| {
            vec![BandConfig {
                low_hz: 0.0,
                high_hz: 24000.0,
                threshold_db: -20.0,
                ratio: 4.0,
                attack_ms: 5.0,
                release_ms: 50.0,
                makeup_gain_db: 0.0,
                detection: det,
            }]
        };

        let input = sine_wave(440.0, 0.9, 48000, SR);
        let skip = 4800;

        let mut comp_peak = MultibandCompressor::new(make_band(DetectionMode::Peak), SR);
        let out_peak = comp_peak.process(&input);
        let rms_peak = rms(&out_peak[skip..]);

        let mut comp_rms = MultibandCompressor::new(make_band(DetectionMode::Rms), SR);
        let out_rms = comp_rms.process(&input);
        let rms_rms = rms(&out_rms[skip..]);

        // They should differ (peak detection yields higher envelope -> more compression).
        assert!(
            (rms_peak - rms_rms).abs() > 1e-4,
            "peak and rms should differ: peak_rms={}, rms_rms={}",
            rms_peak,
            rms_rms
        );
    }

    // 14. Utility: linear_to_db / db_to_linear roundtrip.
    #[test]
    fn test_db_roundtrip() {
        for &val in &[0.001, 0.1, 0.5, 1.0, 2.0] {
            let db = linear_to_db(val);
            let back = db_to_linear(db);
            assert!(
                (back - val).abs() < 1e-10,
                "roundtrip failed for {}: got {}",
                val,
                back
            );
        }
    }

    // 15. time_constant edge case: zero ms returns 0.0 coefficient.
    #[test]
    fn test_time_constant_zero() {
        assert_eq!(time_constant(0.0, 48000.0), 0.0);
        assert_eq!(time_constant(-1.0, 48000.0), 0.0);
    }
}
