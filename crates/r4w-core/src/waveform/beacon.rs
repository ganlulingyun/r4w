//! Emergency Locator Transmitter (ELT) Swept Tone Beacon
//!
//! Implements the 121.5 MHz / 243 MHz distress beacon signal as defined by
//! ICAO Annex 10 and ITU Radio Regulations. The signal is an AM carrier
//! modulated by a distinctive swept audio tone used for search and rescue.
//!
//! ## Signal Structure
//!
//! ```text
//! s(t) = A · [1 + m · sin(φ(t))] · cos(2π·f_c·t)
//!
//! where φ(t) = 2π · ∫ f_sweep(τ) dτ
//!       f_sweep(t) = f_high - (f_high - f_low) · (t mod T_sweep) / T_sweep  [downward]
//!       f_sweep(t) = f_low  + (f_high - f_low) · (t mod T_sweep) / T_sweep  [upward]
//! ```
//!
//! ## Parameters (ICAO/ITU)
//!
//! | Parameter | Value |
//! |-----------|-------|
//! | Carrier | 121.5 MHz (VHF) / 243.0 MHz (UHF) |
//! | Modulation | AM, >85% depth |
//! | Sweep range | 300 Hz – 1600 Hz |
//! | Sweep rate | 2–4 sweeps/second |
//! | Sweep direction | Down (ELT), Up (PLB), Either (EPIRB) |
//! | RF power | 20–350 mW typical |
//!
//! ## Beacon Types
//!
//! - **ELT** (Emergency Locator Transmitter) – aircraft, sweeps downward
//! - **EPIRB** (Emergency Position-Indicating Radio Beacon) – maritime
//! - **PLB** (Personal Locator Beacon) – sweeps upward

use super::{CommonParams, DemodResult, ModulationStage, VisualizationData, Waveform, WaveformInfo};
use crate::types::IQSample;
use std::f64::consts::PI;

/// Sweep direction for the audio tone
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SweepDirection {
    /// Downward sweep (1600 Hz → 300 Hz) — standard ELT
    Down,
    /// Upward sweep (300 Hz → 1600 Hz) — PLB
    Up,
}

/// Beacon type determines default sweep direction and naming
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum BeaconType {
    /// Emergency Locator Transmitter (aircraft) — 121.5 MHz, sweep down
    Elt,
    /// Emergency Position-Indicating Radio Beacon (maritime) — 121.5 MHz
    Epirb,
    /// Personal Locator Beacon — 121.5 MHz, sweep up
    Plb,
    /// Military guard frequency beacon — 243.0 MHz, sweep down
    Military,
}

/// 121.5 / 243.0 MHz Swept Tone Distress Beacon
// trace:FR-043 | ai:claude
#[derive(Debug, Clone)]
pub struct Beacon {
    common: CommonParams,
    /// Beacon type (ELT, EPIRB, PLB, Military)
    beacon_type: BeaconType,
    /// Carrier frequency in Hz
    carrier_freq: f64,
    /// Low end of sweep range (Hz)
    sweep_low_hz: f64,
    /// High end of sweep range (Hz)
    sweep_high_hz: f64,
    /// Number of sweeps per second
    sweep_rate: f64,
    /// Sweep direction
    sweep_direction: SweepDirection,
    /// AM modulation depth (0.0 to 1.0)
    modulation_depth: f64,
}

impl Beacon {
    /// Create a beacon with full parameter control
    pub fn new(
        sample_rate: f64,
        beacon_type: BeaconType,
        carrier_freq: f64,
        sweep_low_hz: f64,
        sweep_high_hz: f64,
        sweep_rate: f64,
        sweep_direction: SweepDirection,
        modulation_depth: f64,
    ) -> Self {
        Self {
            common: CommonParams {
                sample_rate,
                carrier_freq: 0.0,
                amplitude: 1.0,
            },
            beacon_type,
            carrier_freq,
            sweep_low_hz,
            sweep_high_hz,
            sweep_rate,
            sweep_direction,
            modulation_depth,
        }
    }

    /// Standard 121.5 MHz ELT beacon (aircraft distress)
    ///
    /// Downward sweep from 1600 Hz to 300 Hz, 3 sweeps/second, 85% modulation
    pub fn elt(sample_rate: f64) -> Self {
        Self::new(
            sample_rate,
            BeaconType::Elt,
            121_500_000.0,
            300.0,
            1600.0,
            3.0,
            SweepDirection::Down,
            0.85,
        )
    }

    /// Standard PLB beacon (personal locator)
    ///
    /// Upward sweep from 300 Hz to 1600 Hz, 3 sweeps/second
    pub fn plb(sample_rate: f64) -> Self {
        Self::new(
            sample_rate,
            BeaconType::Plb,
            121_500_000.0,
            300.0,
            1600.0,
            3.0,
            SweepDirection::Up,
            0.85,
        )
    }

    /// EPIRB beacon (maritime emergency)
    ///
    /// Downward sweep, 3 sweeps/second
    pub fn epirb(sample_rate: f64) -> Self {
        Self::new(
            sample_rate,
            BeaconType::Epirb,
            121_500_000.0,
            300.0,
            1600.0,
            3.0,
            SweepDirection::Down,
            0.85,
        )
    }

    /// 243.0 MHz military guard frequency beacon
    ///
    /// Same audio sweep as ELT but on the UHF military guard frequency
    pub fn military(sample_rate: f64) -> Self {
        Self::new(
            sample_rate,
            BeaconType::Military,
            243_000_000.0,
            300.0,
            1600.0,
            3.0,
            SweepDirection::Down,
            0.85,
        )
    }

    /// Generate baseband IQ samples for a given duration
    ///
    /// The output is a baseband representation: the carrier is at DC (0 Hz)
    /// and the AM envelope carries the swept audio tone.
    pub fn generate(&self, duration_s: f64) -> Vec<IQSample> {
        let num_samples = (self.common.sample_rate * duration_s) as usize;
        self.generate_samples(num_samples)
    }

    /// Generate a specific number of baseband IQ samples
    pub fn generate_samples(&self, num_samples: usize) -> Vec<IQSample> {
        let sr = self.common.sample_rate;
        let amp = self.common.amplitude;
        let m = self.modulation_depth;
        let sweep_period = 1.0 / self.sweep_rate;
        let f_low = self.sweep_low_hz;
        let f_high = self.sweep_high_hz;
        let sweep_bw = f_high - f_low;

        let mut samples = Vec::with_capacity(num_samples);
        let mut audio_phase = 0.0_f64;

        for n in 0..num_samples {
            let t = n as f64 / sr;

            // Position within current sweep (0.0 to 1.0)
            let sweep_pos = (t % sweep_period) / sweep_period;

            // Instantaneous audio frequency based on sweep direction
            let f_audio = match self.sweep_direction {
                SweepDirection::Down => f_high - sweep_bw * sweep_pos,
                SweepDirection::Up => f_low + sweep_bw * sweep_pos,
            };

            // Accumulate audio phase (more accurate than closed-form for
            // continuous phase across sweep boundaries)
            audio_phase += 2.0 * PI * f_audio / sr;

            // AM envelope: 1 + m * sin(audio_phase)
            let envelope = amp * (1.0 + m * audio_phase.sin());

            // Baseband IQ (carrier at DC)
            samples.push(IQSample::new(envelope, 0.0));
        }

        samples
    }

    /// Generate the audio sweep waveform only (no carrier)
    ///
    /// Returns the modulating audio signal as f64 samples, useful for
    /// analysis or playback through a speaker.
    pub fn generate_audio(&self, duration_s: f64) -> Vec<f64> {
        let num_samples = (self.common.sample_rate * duration_s) as usize;
        let sr = self.common.sample_rate;
        let sweep_period = 1.0 / self.sweep_rate;
        let f_low = self.sweep_low_hz;
        let f_high = self.sweep_high_hz;
        let sweep_bw = f_high - f_low;

        let mut audio = Vec::with_capacity(num_samples);
        let mut phase = 0.0_f64;

        for n in 0..num_samples {
            let t = n as f64 / sr;
            let sweep_pos = (t % sweep_period) / sweep_period;

            let f_audio = match self.sweep_direction {
                SweepDirection::Down => f_high - sweep_bw * sweep_pos,
                SweepDirection::Up => f_low + sweep_bw * sweep_pos,
            };

            phase += 2.0 * PI * f_audio / sr;
            audio.push(phase.sin());
        }

        audio
    }

    /// Get the instantaneous sweep frequency at time t
    pub fn sweep_frequency_at(&self, t: f64) -> f64 {
        let sweep_period = 1.0 / self.sweep_rate;
        let sweep_pos = (t % sweep_period) / sweep_period;
        let sweep_bw = self.sweep_high_hz - self.sweep_low_hz;

        match self.sweep_direction {
            SweepDirection::Down => self.sweep_high_hz - sweep_bw * sweep_pos,
            SweepDirection::Up => self.sweep_low_hz + sweep_bw * sweep_pos,
        }
    }
}

impl Waveform for Beacon {
    fn info(&self) -> WaveformInfo {
        match self.beacon_type {
            BeaconType::Elt => WaveformInfo {
                name: "ELT-121.5",
                full_name: "Emergency Locator Transmitter (121.5 MHz)",
                description: "AM swept-tone distress beacon for aircraft search and rescue",
                complexity: 2,
                bits_per_symbol: 0,
                carries_data: false,
                characteristics: &[
                    "AM modulation with >85% depth",
                    "Audio sweep 1600→300 Hz (downward)",
                    "3 sweeps per second",
                    "121.5 MHz VHF guard frequency",
                    "Continuous transmission until battery depleted",
                ],
                history: "The 121.5 MHz distress frequency was established by ICAO in 1948. \
                    ELTs became mandatory on aircraft in many countries following the 1972 \
                    disappearance of U.S. congressmen Hale Boggs and Nick Begich in Alaska. \
                    The distinctive warbling tone enables direction-finding by SAR teams.",
                modern_usage: "While Cospas-Sarsat satellite monitoring shifted to 406 MHz \
                    in 2009, the 121.5 MHz frequency remains actively monitored by ATC and \
                    is used for the final homing phase of search and rescue operations. \
                    Modern ELTs transmit on both 406 MHz (satellite alert) and 121.5 MHz (homing).",
            },
            BeaconType::Plb => WaveformInfo {
                name: "PLB-121.5",
                full_name: "Personal Locator Beacon (121.5 MHz)",
                description: "AM swept-tone personal distress beacon (upward sweep)",
                complexity: 2,
                bits_per_symbol: 0,
                carries_data: false,
                characteristics: &[
                    "AM modulation with >85% depth",
                    "Audio sweep 300→1600 Hz (upward)",
                    "3 sweeps per second",
                    "121.5 MHz VHF guard frequency",
                ],
                history: "Personal Locator Beacons evolved from maritime EPIRBs for land-based \
                    use. The upward sweep distinguishes PLBs from ELTs in audio.",
                modern_usage: "Used by hikers, sailors, and remote workers as a personal \
                    emergency signaling device.",
            },
            BeaconType::Epirb => WaveformInfo {
                name: "EPIRB-121.5",
                full_name: "Emergency Position-Indicating Radio Beacon (121.5 MHz)",
                description: "AM swept-tone maritime distress beacon",
                complexity: 2,
                bits_per_symbol: 0,
                carries_data: false,
                characteristics: &[
                    "AM modulation with >85% depth",
                    "Audio sweep 1600→300 Hz",
                    "3 sweeps per second",
                    "121.5 MHz VHF guard frequency",
                    "Auto-activation by water or G-force",
                ],
                history: "EPIRBs have been mandatory on SOLAS vessels since 1999. \
                    The 121.5 MHz signal serves as a homing beacon for nearby rescue craft.",
                modern_usage: "Modern EPIRBs pair 406 MHz satellite alerting with \
                    121.5 MHz homing signals and GPS position encoding.",
            },
            BeaconType::Military => WaveformInfo {
                name: "Beacon-243",
                full_name: "Military Guard Frequency Beacon (243.0 MHz)",
                description: "AM swept-tone distress beacon on UHF military guard frequency",
                complexity: 2,
                bits_per_symbol: 0,
                carries_data: false,
                characteristics: &[
                    "AM modulation with >85% depth",
                    "Audio sweep 1600→300 Hz (downward)",
                    "3 sweeps per second",
                    "243.0 MHz UHF military guard frequency",
                    "Second harmonic of 121.5 MHz",
                ],
                history: "243.0 MHz is the UHF military emergency frequency, the second harmonic \
                    of 121.5 MHz. This harmonic relationship allows simple frequency-doubler \
                    circuits to transmit on both frequencies simultaneously.",
                modern_usage: "Monitored by military aircraft and installations worldwide. \
                    Combat survival radios (e.g., HOOK3) transmit on 243 MHz for CSAR operations.",
            },
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, _data: &[u8]) -> Vec<IQSample> {
        // Beacon carries no data — generate one complete sweep cycle
        let sweep_period = 1.0 / self.sweep_rate;
        self.generate(sweep_period)
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let mut result = DemodResult::default();
        if samples.is_empty() {
            return result;
        }

        // Envelope detection (AM demod)
        let envelopes: Vec<f64> = samples.iter().map(|s| s.norm()).collect();

        // Mean and peak envelope
        let mean_env = envelopes.iter().sum::<f64>() / envelopes.len() as f64;
        let peak_env = envelopes.iter().cloned().fold(0.0_f64, f64::max);

        result.metadata.insert("mean_envelope".to_string(), mean_env);
        result.metadata.insert("peak_envelope".to_string(), peak_env);

        // Estimate modulation depth: m ≈ (max - min) / (max + min)
        let min_env = envelopes.iter().cloned().fold(f64::MAX, f64::min);
        if peak_env + min_env > 0.0 {
            let mod_depth = (peak_env - min_env) / (peak_env + min_env);
            result.metadata.insert("modulation_depth".to_string(), mod_depth);
        }

        // Estimate sweep rate from envelope periodicity
        // Count zero-crossings of (envelope - mean) for rough periodicity
        let mut crossings = 0;
        for i in 1..envelopes.len() {
            if (envelopes[i] - mean_env).signum() != (envelopes[i - 1] - mean_env).signum() {
                crossings += 1;
            }
        }
        let duration_s = samples.len() as f64 / self.common.sample_rate;
        if duration_s > 0.0 {
            // Each sweep produces many crossings from the audio, but the
            // envelope amplitude modulation gives ~2 crossings per audio cycle
            result.metadata.insert("duration_s".to_string(), duration_s);
            result.metadata.insert("envelope_crossings".to_string(), crossings as f64);
        }

        result
    }

    fn samples_per_symbol(&self) -> usize {
        // One "symbol" is one complete sweep cycle
        (self.common.sample_rate / self.sweep_rate).ceil() as usize
    }

    fn get_visualization(&self, _data: &[u8]) -> VisualizationData {
        // Generate two sweep cycles for visualization
        let duration = 2.0 / self.sweep_rate;
        let samples = self.generate(duration);

        // Constellation: AM traces a line on the real axis
        let n_points = 64;
        let constellation: Vec<IQSample> = (0..n_points)
            .map(|i| {
                let env = self.common.amplitude
                    * (1.0 + self.modulation_depth * (2.0 * PI * i as f64 / n_points as f64).sin());
                IQSample::new(env, 0.0)
            })
            .collect();

        VisualizationData {
            samples,
            constellation,
            constellation_labels: vec![
                "AM envelope varies along I axis".to_string(),
            ],
            spectrum: Vec::new(),
            description: format!(
                "{} beacon: {:.0}→{:.0} Hz sweep, {:.0} sweeps/sec, {:.0}% AM depth",
                match self.beacon_type {
                    BeaconType::Elt => "ELT",
                    BeaconType::Epirb => "EPIRB",
                    BeaconType::Plb => "PLB",
                    BeaconType::Military => "Military",
                },
                match self.sweep_direction {
                    SweepDirection::Down => self.sweep_high_hz,
                    SweepDirection::Up => self.sweep_low_hz,
                },
                match self.sweep_direction {
                    SweepDirection::Down => self.sweep_low_hz,
                    SweepDirection::Up => self.sweep_high_hz,
                },
                self.sweep_rate,
                self.modulation_depth * 100.0,
            ),
        }
    }

    fn get_modulation_stages(&self, _data: &[u8]) -> Vec<ModulationStage> {
        let sweep_period = 1.0 / self.sweep_rate;
        let num_samples = (self.common.sample_rate * sweep_period) as usize;

        // Stage 1: Generate audio sweep
        let audio = self.generate_audio(sweep_period);
        let audio_iq: Vec<IQSample> = audio.iter().map(|&a| IQSample::new(a, 0.0)).collect();

        // Stage 2: Apply AM envelope
        let modulated = self.generate(sweep_period);

        vec![
            ModulationStage {
                name: "Audio Sweep".to_string(),
                description: format!(
                    "Swept tone {:.0}→{:.0} Hz at {:.0} sweeps/sec",
                    match self.sweep_direction {
                        SweepDirection::Down => self.sweep_high_hz,
                        SweepDirection::Up => self.sweep_low_hz,
                    },
                    match self.sweep_direction {
                        SweepDirection::Down => self.sweep_low_hz,
                        SweepDirection::Up => self.sweep_high_hz,
                    },
                    self.sweep_rate,
                ),
                samples: Some(audio_iq[..num_samples.min(audio_iq.len())].to_vec()),
                input_bits: None,
                output_symbols: None,
                constellation: None,
            },
            ModulationStage {
                name: "AM Modulation".to_string(),
                description: format!(
                    "Amplitude modulation with {:.0}% depth on {:.1} MHz carrier",
                    self.modulation_depth * 100.0,
                    self.carrier_freq / 1e6,
                ),
                samples: Some(modulated),
                input_bits: None,
                output_symbols: None,
                constellation: None,
            },
        ]
    }

    fn generate_demo(&self, duration_ms: f64) -> Vec<IQSample> {
        self.generate(duration_ms / 1000.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_elt_creation() {
        let elt = Beacon::elt(48000.0);
        assert_eq!(elt.carrier_freq, 121_500_000.0);
        assert_eq!(elt.sweep_direction, SweepDirection::Down);
        assert_eq!(elt.beacon_type, BeaconType::Elt);
        assert!((elt.modulation_depth - 0.85).abs() < 1e-10);
        let info = elt.info();
        assert_eq!(info.name, "ELT-121.5");
        assert!(!info.carries_data);
    }

    #[test]
    fn test_plb_sweeps_up() {
        let plb = Beacon::plb(48000.0);
        assert_eq!(plb.sweep_direction, SweepDirection::Up);
        assert_eq!(plb.beacon_type, BeaconType::Plb);
    }

    #[test]
    fn test_military_243mhz() {
        let mil = Beacon::military(48000.0);
        assert_eq!(mil.carrier_freq, 243_000_000.0);
        assert_eq!(mil.beacon_type, BeaconType::Military);
        let info = mil.info();
        assert_eq!(info.name, "Beacon-243");
    }

    #[test]
    fn test_generate_correct_length() {
        let elt = Beacon::elt(48000.0);
        // 1 second at 48kHz = 48000 samples
        let samples = elt.generate(1.0);
        assert_eq!(samples.len(), 48000);

        // One sweep period (1/3 sec) = 16000 samples
        let sweep = elt.modulate(&[]);
        assert_eq!(sweep.len(), 16000);
    }

    #[test]
    fn test_am_envelope_range() {
        let elt = Beacon::elt(48000.0);
        let samples = elt.generate(1.0);

        // With m=0.85, envelope should be between A*(1-0.85)=0.15 and A*(1+0.85)=1.85
        for s in &samples {
            let env = s.re; // Baseband AM, all energy in I channel
            assert!(env >= 0.0, "Envelope should be non-negative, got {}", env);
            assert!(env <= 1.86, "Envelope should be <= 1.85, got {}", env);
        }

        // Q channel should be zero for baseband AM
        for s in &samples {
            assert!(s.im.abs() < 1e-10, "Q should be zero for baseband AM");
        }
    }

    #[test]
    fn test_sweep_frequency() {
        let elt = Beacon::elt(48000.0);

        // At t=0, downward sweep starts at 1600 Hz
        let f0 = elt.sweep_frequency_at(0.0);
        assert!((f0 - 1600.0).abs() < 1.0);

        // At mid-sweep (t = 1/6 sec for 3 Hz rate), should be ~950 Hz
        let f_mid = elt.sweep_frequency_at(1.0 / 6.0);
        assert!((f_mid - 950.0).abs() < 10.0);

        // PLB sweeps up: at t=0, starts at 300 Hz
        let plb = Beacon::plb(48000.0);
        let f0_plb = plb.sweep_frequency_at(0.0);
        assert!((f0_plb - 300.0).abs() < 1.0);
    }

    #[test]
    fn test_audio_generation() {
        let elt = Beacon::elt(48000.0);
        let audio = elt.generate_audio(1.0);
        assert_eq!(audio.len(), 48000);

        // Audio values should be in [-1, 1]
        for &s in &audio {
            assert!(s >= -1.001 && s <= 1.001, "Audio out of range: {}", s);
        }
    }

    #[test]
    fn test_demodulate_recovers_modulation() {
        let elt = Beacon::elt(48000.0);
        let samples = elt.generate(1.0);
        let result = elt.demodulate(&samples);

        let mod_depth = result.metadata.get("modulation_depth").unwrap();
        // Should be close to 0.85
        assert!(
            (*mod_depth - 0.85).abs() < 0.1,
            "Recovered modulation depth {} should be near 0.85",
            mod_depth
        );
    }

    #[test]
    fn test_samples_per_symbol() {
        let elt = Beacon::elt(48000.0);
        // One sweep at 3 Hz rate = 48000/3 = 16000 samples
        assert_eq!(elt.samples_per_symbol(), 16000);
    }

    #[test]
    fn test_visualization() {
        let elt = Beacon::elt(48000.0);
        let viz = elt.get_visualization(&[]);
        assert!(!viz.samples.is_empty());
        assert!(!viz.constellation.is_empty());
        assert!(viz.description.contains("ELT"));
    }

    #[test]
    fn test_modulation_stages() {
        let elt = Beacon::elt(48000.0);
        let stages = elt.get_modulation_stages(&[]);
        assert_eq!(stages.len(), 2);
        assert_eq!(stages[0].name, "Audio Sweep");
        assert_eq!(stages[1].name, "AM Modulation");
    }
}
