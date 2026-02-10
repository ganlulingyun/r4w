//! Active sonar signal processing with matched filtering, Doppler compensation,
//! and underwater acoustic channel modeling.
//!
//! This module provides a complete active-sonar processing pipeline:
//!
//! - **LFM chirp ping generation** for transmit waveforms
//! - **Matched filtering** (pulse compression) for echo detection
//! - **Doppler shift** estimation for moving targets
//! - **Transmission loss** modeling (spherical spreading + absorption)
//! - **Sonar equation** evaluation (SE = SL − 2·TL + TS − NL − DI)
//! - **Reverberation level** estimation
//!
//! # Example
//!
//! ```
//! use r4w_core::sonar_processor::{SonarConfig, SonarProcessor};
//!
//! let cfg = SonarConfig {
//!     frequency_hz: 10_000.0,
//!     bandwidth_hz: 2_000.0,
//!     pulse_duration_s: 0.05,
//!     sample_rate: 48_000.0,
//!     sound_speed_mps: 1500.0,
//! };
//!
//! let proc = SonarProcessor::new(cfg);
//!
//! // Generate a transmit ping
//! let ping = proc.generate_ping();
//! assert!(!ping.is_empty());
//!
//! // Round-trip range for a 0.1 s delay
//! let range = proc.range_from_delay(0.1);
//! assert!((range - 75.0).abs() < 1e-6);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// Parameters that fully describe the sonar system.
#[derive(Debug, Clone)]
pub struct SonarConfig {
    /// Centre frequency of the transmit waveform (Hz).
    pub frequency_hz: f64,
    /// Sweep bandwidth of the LFM chirp (Hz).
    pub bandwidth_hz: f64,
    /// Duration of a single ping (seconds).
    pub pulse_duration_s: f64,
    /// ADC / DAC sample rate (samples per second).
    pub sample_rate: f64,
    /// Speed of sound in water (m/s).  Default 1 500 m/s.
    pub sound_speed_mps: f64,
}

impl Default for SonarConfig {
    fn default() -> Self {
        Self {
            frequency_hz: 10_000.0,
            bandwidth_hz: 2_000.0,
            pulse_duration_s: 0.05,
            sample_rate: 48_000.0,
            sound_speed_mps: 1500.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Sonar return descriptor
// ---------------------------------------------------------------------------

/// A single detected echo with estimated parameters.
#[derive(Debug, Clone)]
pub struct SonarReturn {
    /// Estimated range to the target (metres).
    pub range_m: f64,
    /// Estimated Doppler shift (Hz).
    pub doppler_hz: f64,
    /// Signal-to-noise ratio of the detection (dB).
    pub snr_db: f64,
    /// Bearing angle, if available (degrees).  Set to 0.0 when unknown.
    pub bearing_deg: f64,
}

// ---------------------------------------------------------------------------
// Processor
// ---------------------------------------------------------------------------

/// Active sonar signal processor.
pub struct SonarProcessor {
    cfg: SonarConfig,
}

impl SonarProcessor {
    /// Create a new processor from the given configuration.
    pub fn new(cfg: SonarConfig) -> Self {
        Self { cfg }
    }

    // -- Waveform generation ------------------------------------------------

    /// Generate a Linear Frequency Modulated (LFM) chirp ping.
    ///
    /// The chirp sweeps from `f_c − B/2` to `f_c + B/2` over
    /// `pulse_duration_s` seconds, sampled at `sample_rate`.
    /// Returns a vector of real-valued samples.
    pub fn generate_ping(&self) -> Vec<f64> {
        let n = (self.cfg.pulse_duration_s * self.cfg.sample_rate).round() as usize;
        let dt = 1.0 / self.cfg.sample_rate;
        let f0 = self.cfg.frequency_hz - self.cfg.bandwidth_hz / 2.0;
        let chirp_rate = self.cfg.bandwidth_hz / self.cfg.pulse_duration_s;

        (0..n)
            .map(|i| {
                let t = i as f64 * dt;
                let phase = 2.0 * PI * (f0 * t + 0.5 * chirp_rate * t * t);
                phase.sin()
            })
            .collect()
    }

    // -- Matched filter / pulse compression ---------------------------------

    /// Cross-correlate `received` with the reference `ping` (matched filter).
    ///
    /// Returns the magnitude-squared of the normalised correlation at each lag.
    /// A perfect match gives a value of 1.0.
    pub fn matched_filter(&self, received: &[f64], ping: &[f64]) -> Vec<f64> {
        if ping.is_empty() || received.is_empty() {
            return Vec::new();
        }

        let n_out = received.len().saturating_sub(ping.len()) + 1;
        if n_out == 0 {
            return Vec::new();
        }

        // Energy of the reference ping (for normalisation).
        let ping_energy: f64 = ping.iter().map(|x| x * x).sum();
        if ping_energy == 0.0 {
            return vec![0.0; n_out];
        }

        let mut output = Vec::with_capacity(n_out);
        for lag in 0..n_out {
            let mut corr: f64 = 0.0;
            let mut rx_energy: f64 = 0.0;
            for k in 0..ping.len() {
                let r = received[lag + k];
                corr += r * ping[k];
                rx_energy += r * r;
            }
            let denom = (ping_energy * rx_energy).sqrt();
            let val = if denom > 0.0 { corr / denom } else { 0.0 };
            output.push(val * val); // magnitude-squared
        }
        output
    }

    // -- Range / Doppler ---------------------------------------------------

    /// Convert a two-way propagation delay to one-way range.
    ///
    /// `R = c · t / 2`
    pub fn range_from_delay(&self, delay_s: f64) -> f64 {
        self.cfg.sound_speed_mps * delay_s / 2.0
    }

    /// Compute the Doppler frequency shift for a target moving at
    /// `velocity_mps` (positive = closing).
    ///
    /// `f_d = 2 · v · f_c / c`
    pub fn doppler_shift(&self, velocity_mps: f64) -> f64 {
        2.0 * velocity_mps * self.cfg.frequency_hz / self.cfg.sound_speed_mps
    }

    // -- Detection ---------------------------------------------------------

    /// Threshold detection on the matched-filter output.
    ///
    /// Any peak whose normalised correlation exceeds `threshold` (0–1 scale,
    /// compared against the magnitude-squared output of `matched_filter`) is
    /// reported as a [`SonarReturn`].  The bearing is set to 0.0 (unknown).
    pub fn detect_echoes(
        &self,
        mf_output: &[f64],
        threshold: f64,
    ) -> Vec<SonarReturn> {
        let noise_floor = estimate_noise_floor(mf_output);
        let dt = 1.0 / self.cfg.sample_rate;

        let mut detections = Vec::new();
        for (i, &val) in mf_output.iter().enumerate() {
            if val >= threshold {
                let delay = i as f64 * dt;
                let range = self.range_from_delay(delay);
                let snr_db = if noise_floor > 0.0 {
                    10.0 * (val / noise_floor).log10()
                } else {
                    f64::INFINITY
                };
                detections.push(SonarReturn {
                    range_m: range,
                    doppler_hz: 0.0,
                    snr_db,
                    bearing_deg: 0.0,
                });
            }
        }
        detections
    }

    // -- Propagation loss ---------------------------------------------------

    /// Total one-way transmission loss (dB) at the given range.
    ///
    /// `TL = 20·log10(r) + α·r / 1000`
    ///
    /// where α is the frequency-dependent absorption coefficient (dB/km).
    pub fn transmission_loss(&self, range_m: f64) -> f64 {
        if range_m <= 0.0 {
            return 0.0;
        }
        let spreading = 20.0 * range_m.log10();
        let alpha = self.absorption_coefficient(self.cfg.frequency_hz);
        let absorption = alpha * range_m / 1000.0;
        spreading + absorption
    }

    /// Frequency-dependent absorption coefficient (dB/km).
    ///
    /// Uses a simplified Francois-Garrison model valid for 1–100 kHz,
    /// temperature ~ 10 C, salinity ~ 35 ppt, depth ~ 0 m.
    ///
    /// The model captures the two dominant molecular relaxation processes
    /// (boric acid and magnesium sulphate) plus a viscous term:
    ///
    /// ```text
    /// alpha = A1*f1*f^2 / (f1^2 + f^2) + A2*f2*f^2 / (f2^2 + f^2) + A3*f^2
    /// ```
    pub fn absorption_coefficient(&self, freq_hz: f64) -> f64 {
        let f_khz = freq_hz / 1000.0;
        let f2 = f_khz * f_khz;

        // Boric acid relaxation (~1 kHz)
        let f1_r = 0.78; // relaxation freq (kHz)
        let a1 = 0.106;
        let boric = a1 * f1_r * f2 / (f1_r * f1_r + f2);

        // Magnesium sulphate relaxation (~42 kHz for T~10 C)
        let f2_r = 42.0;
        let a2 = 0.52;
        let mgso4 = a2 * f2_r * f2 / (f2_r * f2_r + f2);

        // Viscous absorption (pure water)
        let a3 = 0.000_49;
        let viscous = a3 * f2;

        boric + mgso4 + viscous
    }

    // -- Reverberation ------------------------------------------------------

    /// Rough estimate of volume reverberation level (dB re 1 uPa).
    ///
    /// `RL = SL - 2*TL + S_v + 10*log10(V)`
    ///
    /// where `S_v` is the volume scattering strength (dB), and `V` is the
    /// ensonified volume approximated from the pulse length and beam width.
    ///
    /// * `source_level_db` -- source level (dB re 1 uPa @ 1 m)
    /// * `range_m`         -- range to the scattering volume
    /// * `sv_db`           -- volume scattering strength
    /// * `beam_width_deg`  -- 3 dB beam width of the transducer
    pub fn reverberation_level(
        &self,
        source_level_db: f64,
        range_m: f64,
        sv_db: f64,
        beam_width_deg: f64,
    ) -> f64 {
        let tl = self.transmission_loss(range_m);
        // Ensonified volume ~ (range * beam_rad)^2 * c*tau/2
        let beam_rad = beam_width_deg.to_radians();
        let cross = range_m * beam_rad;
        let depth = self.cfg.sound_speed_mps * self.cfg.pulse_duration_s / 2.0;
        let volume = cross * cross * depth;
        let vol_db = if volume > 0.0 {
            10.0 * volume.log10()
        } else {
            0.0
        };
        source_level_db - 2.0 * tl + sv_db + vol_db
    }

    // -- Sonar equation -----------------------------------------------------

    /// Evaluate the active sonar equation and return the signal excess (dB).
    ///
    /// `SE = SL - 2*TL + TS - NL - DI`
    ///
    /// * `source_level_db`    -- SL (dB re 1 uPa @ 1 m)
    /// * `range_m`            -- one-way range; TL computed internally
    /// * `target_strength_db` -- TS (dB)
    /// * `noise_level_db`     -- NL (dB re 1 uPa)
    /// * `directivity_index_db` -- DI (dB)
    pub fn sonar_equation(
        &self,
        source_level_db: f64,
        range_m: f64,
        target_strength_db: f64,
        noise_level_db: f64,
        directivity_index_db: f64,
    ) -> f64 {
        let tl = self.transmission_loss(range_m);
        source_level_db - 2.0 * tl + target_strength_db - noise_level_db - directivity_index_db
    }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Estimate the noise floor as the median of the values.
fn estimate_noise_floor(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    let mut sorted: Vec<f64> = data.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    sorted[sorted.len() / 2]
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_proc() -> SonarProcessor {
        SonarProcessor::new(SonarConfig::default())
    }

    // 1. Ping generation produces the correct number of samples.
    #[test]
    fn test_generate_ping_length() {
        let p = default_proc();
        let ping = p.generate_ping();
        let expected = (p.cfg.pulse_duration_s * p.cfg.sample_rate).round() as usize;
        assert_eq!(ping.len(), expected);
    }

    // 2. Ping samples are bounded in [-1, 1].
    #[test]
    fn test_generate_ping_amplitude() {
        let p = default_proc();
        let ping = p.generate_ping();
        for &s in &ping {
            assert!(
                s >= -1.0 && s <= 1.0,
                "sample {} out of range",
                s
            );
        }
    }

    // 3. Range from delay: R = c * t / 2.
    #[test]
    fn test_range_from_delay() {
        let p = default_proc();
        let range = p.range_from_delay(0.1);
        assert!((range - 75.0).abs() < 1e-9, "got {}", range);
    }

    // 4. Doppler shift for a closing target.
    #[test]
    fn test_doppler_shift() {
        let p = default_proc();
        // 5 m/s closing -> f_d = 2 * 5 * 10000 / 1500 = 66.667 Hz
        let fd = p.doppler_shift(5.0);
        assert!((fd - 66.6667).abs() < 0.01, "got {}", fd);
    }

    // 5. Matched filter on an identical copy -> peak at lag 0.
    #[test]
    fn test_matched_filter_peak_at_zero() {
        let p = default_proc();
        let ping = p.generate_ping();
        let mf = p.matched_filter(&ping, &ping);
        assert!(!mf.is_empty());
        // The peak should be at index 0 (identical signals, no delay).
        let peak_idx = mf
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak_idx, 0);
    }

    // 6. Matched filter peak value for identical signals ~ 1.0.
    #[test]
    fn test_matched_filter_peak_value() {
        let p = default_proc();
        let ping = p.generate_ping();
        let mf = p.matched_filter(&ping, &ping);
        let peak = mf.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (peak - 1.0).abs() < 1e-6,
            "peak should be ~1.0, got {}",
            peak
        );
    }

    // 7. Matched filter with delayed echo -> peak at correct lag.
    #[test]
    fn test_matched_filter_delayed_echo() {
        let p = default_proc();
        let ping = p.generate_ping();
        let delay_samples = 200;
        let mut received = vec![0.0; delay_samples];
        received.extend_from_slice(&ping);
        let mf = p.matched_filter(&received, &ping);
        let peak_idx = mf
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        assert_eq!(peak_idx, delay_samples);
    }

    // 8. Transmission loss increases with range.
    #[test]
    fn test_transmission_loss_increases() {
        let p = default_proc();
        let tl1 = p.transmission_loss(100.0);
        let tl2 = p.transmission_loss(1000.0);
        let tl3 = p.transmission_loss(10_000.0);
        assert!(tl2 > tl1, "TL should increase: {} vs {}", tl2, tl1);
        assert!(tl3 > tl2, "TL should increase: {} vs {}", tl3, tl2);
    }

    // 9. Absorption coefficient is positive and increases with frequency.
    #[test]
    fn test_absorption_coefficient() {
        let p = default_proc();
        let a1 = p.absorption_coefficient(1_000.0);
        let a2 = p.absorption_coefficient(10_000.0);
        let a3 = p.absorption_coefficient(50_000.0);
        assert!(a1 > 0.0);
        assert!(a2 > a1, "absorption should increase: {} vs {}", a2, a1);
        assert!(a3 > a2, "absorption should increase: {} vs {}", a3, a2);
    }

    // 10. Sonar equation: higher source level -> higher signal excess.
    #[test]
    fn test_sonar_equation_source_level() {
        let p = default_proc();
        let se1 = p.sonar_equation(200.0, 1000.0, -10.0, 60.0, 20.0);
        let se2 = p.sonar_equation(220.0, 1000.0, -10.0, 60.0, 20.0);
        assert!(
            se2 > se1,
            "higher SL should give higher SE: {} vs {}",
            se2,
            se1
        );
        assert!(
            (se2 - se1 - 20.0).abs() < 1e-9,
            "difference should be 20 dB, got {}",
            se2 - se1
        );
    }

    // 11. Detect echoes returns results above threshold.
    #[test]
    fn test_detect_echoes() {
        let p = default_proc();
        let ping = p.generate_ping();
        let delay_samples = 500;
        let mut received = vec![0.0; delay_samples];
        received.extend_from_slice(&ping);
        received.extend(vec![0.0; 1000]);
        let mf = p.matched_filter(&received, &ping);
        let echoes = p.detect_echoes(&mf, 0.8);
        assert!(
            !echoes.is_empty(),
            "should detect at least one echo"
        );
        // The detected echo's range should correspond to the delay.
        let expected_range =
            p.range_from_delay(delay_samples as f64 / p.cfg.sample_rate);
        let first = &echoes[0];
        assert!(
            (first.range_m - expected_range).abs() < 1.0,
            "range mismatch: got {}, expected {}",
            first.range_m,
            expected_range
        );
    }

    // 12. Reverberation level is finite.
    #[test]
    fn test_reverberation_level() {
        let p = default_proc();
        let rl = p.reverberation_level(220.0, 1000.0, -80.0, 10.0);
        assert!(rl.is_finite(), "RL should be finite, got {}", rl);
    }

    // 13. Matched filter with empty inputs.
    #[test]
    fn test_matched_filter_empty() {
        let p = default_proc();
        assert!(p.matched_filter(&[], &[1.0, 2.0]).is_empty());
        assert!(p.matched_filter(&[1.0, 2.0], &[]).is_empty());
    }

    // 14. Transmission loss at zero range.
    #[test]
    fn test_transmission_loss_zero_range() {
        let p = default_proc();
        let tl = p.transmission_loss(0.0);
        assert_eq!(tl, 0.0);
    }
}
