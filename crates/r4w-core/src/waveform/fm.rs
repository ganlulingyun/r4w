//! FM (Frequency Modulation) - Analog Audio Modulation
//!
//! True analog FM modulates an audio/baseband signal by varying the instantaneous frequency
//! of a carrier wave. This is used in FM broadcast radio and two-way radio communications.
//!
//! ## Mathematical Definition
//!
//! ```text
//! s(t) = A · cos(2π·fc·t + 2π·kf · ∫x(τ)dτ)
//! ```
//!
//! Where:
//! - A = carrier amplitude
//! - fc = carrier frequency
//! - kf = frequency deviation constant (Hz per unit amplitude)
//! - x(t) = baseband/audio signal
//! - ∫x(τ)dτ = integral of the modulating signal
//!
//! ## Key Parameters
//!
//! - **Frequency Deviation (Δf)**: Maximum frequency shift from carrier
//! - **Modulation Index (β)**: β = Δf / fm_max (ratio of deviation to max audio freq)
//! - **Bandwidth**: Approximately 2(Δf + fm_max) by Carson's rule
//!
//! ## Variants
//!
//! - **NBFM**: Narrowband FM (β < 1), ~15 kHz bandwidth, used in two-way radio
//! - **WBFM**: Wideband FM (β >> 1), ~200 kHz bandwidth, used in FM broadcast
//!
//! ## Note
//!
//! For digital frequency modulation (FSK), see the `fsk` module.
//! This module is specifically for analog audio/voice transmission.

use super::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};
use crate::types::IQSample;
use std::f64::consts::PI;

/// FM variant for different applications
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FmVariant {
    /// Wideband FM (broadcast radio, β >> 1)
    Wideband,
    /// Narrowband FM (two-way radio, β < 1)
    Narrowband,
}

/// FM (Frequency Modulation) for analog audio
#[derive(Debug, Clone)]
pub struct FM {
    /// Common waveform parameters
    common: CommonParams,
    /// Carrier frequency in Hz
    carrier_freq: f64,
    /// Maximum frequency deviation in Hz
    freq_deviation: f64,
    /// FM variant (wideband or narrowband)
    variant: FmVariant,
    /// Audio bandwidth in Hz (for information/visualization)
    audio_bandwidth: f64,
}

impl FM {
    /// Create a new FM modulator
    pub fn new(common: CommonParams, carrier_freq: f64, freq_deviation: f64) -> Self {
        Self {
            common,
            carrier_freq,
            freq_deviation,
            variant: FmVariant::Wideband,
            audio_bandwidth: 15000.0, // FM broadcast audio bandwidth
        }
    }

    /// Create FM broadcast configuration (WBFM)
    /// Standard FM broadcast: 75 kHz deviation, 15 kHz audio
    pub fn broadcast(sample_rate: f64, carrier_freq: f64) -> Self {
        Self::new(
            CommonParams {
                sample_rate,
                carrier_freq: 0.0,
                amplitude: 1.0,
            },
            carrier_freq,
            75000.0, // 75 kHz deviation for FM broadcast
        )
        .with_audio_bandwidth(15000.0)
    }

    /// Create narrowband FM configuration (NBFM)
    /// Two-way radio: 2.5-5 kHz deviation, 3 kHz audio
    pub fn narrowband(sample_rate: f64, carrier_freq: f64) -> Self {
        Self::new(
            CommonParams {
                sample_rate,
                carrier_freq: 0.0,
                amplitude: 1.0,
            },
            carrier_freq,
            2500.0, // 2.5 kHz deviation for NBFM
        )
        .with_variant(FmVariant::Narrowband)
        .with_audio_bandwidth(3000.0)
    }

    /// Set frequency deviation
    pub fn with_freq_deviation(mut self, deviation: f64) -> Self {
        self.freq_deviation = deviation;
        self
    }

    /// Set FM variant
    pub fn with_variant(mut self, variant: FmVariant) -> Self {
        self.variant = variant;
        self
    }

    /// Set audio bandwidth
    pub fn with_audio_bandwidth(mut self, bw: f64) -> Self {
        self.audio_bandwidth = bw;
        self
    }

    /// Get frequency deviation
    pub fn freq_deviation(&self) -> f64 {
        self.freq_deviation
    }

    /// Get variant
    pub fn variant(&self) -> FmVariant {
        self.variant
    }

    /// Get modulation index (beta)
    pub fn modulation_index(&self) -> f64 {
        if self.audio_bandwidth > 0.0 {
            self.freq_deviation / self.audio_bandwidth
        } else {
            0.0
        }
    }

    /// Get Carson's rule bandwidth estimate
    pub fn carson_bandwidth(&self) -> f64 {
        2.0 * (self.freq_deviation + self.audio_bandwidth)
    }

    /// Modulate audio samples onto carrier
    /// Input: audio samples normalized to -1.0 to 1.0
    /// Output: I/Q samples of modulated signal
    pub fn modulate_audio(&self, audio: &[f64]) -> Vec<IQSample> {
        let omega_c = 2.0 * PI * self.carrier_freq / self.common.sample_rate;
        let k_f = 2.0 * PI * self.freq_deviation / self.common.sample_rate;
        let amp = self.common.amplitude;

        // FM modulation: phase is integral of frequency deviation
        let mut phase_integral = 0.0;

        audio
            .iter()
            .enumerate()
            .map(|(n, &x)| {
                // Accumulate phase from audio (integral approximation)
                phase_integral += k_f * x;

                // Total phase: carrier + modulation
                let phase = omega_c * n as f64 + phase_integral;

                IQSample::new(amp * phase.cos(), amp * phase.sin())
            })
            .collect()
    }

    /// Demodulate FM signal to recover audio
    /// Uses phase difference detection
    pub fn demodulate_audio(&self, samples: &[IQSample]) -> Vec<f64> {
        if samples.len() < 2 {
            return Vec::new();
        }

        let k_f = 2.0 * PI * self.freq_deviation / self.common.sample_rate;

        // FM demodulation via phase difference
        // Instantaneous frequency = d(phase)/dt
        samples
            .windows(2)
            .map(|pair| {
                // Phase difference between consecutive samples
                let phase0 = pair[0].im.atan2(pair[0].re);
                let phase1 = pair[1].im.atan2(pair[1].re);

                // Unwrap phase difference to handle wrapping
                let mut phase_diff = phase1 - phase0;
                while phase_diff > PI {
                    phase_diff -= 2.0 * PI;
                }
                while phase_diff < -PI {
                    phase_diff += 2.0 * PI;
                }

                // Remove carrier contribution and scale to audio
                // The carrier contributes omega_c per sample
                let omega_c = 2.0 * PI * self.carrier_freq / self.common.sample_rate;
                let deviation = phase_diff - omega_c;

                // Scale back to audio amplitude
                deviation / k_f
            })
            .collect()
    }
}

impl Waveform for FM {
    fn info(&self) -> WaveformInfo {
        let (name, full_name) = match self.variant {
            FmVariant::Wideband => ("WBFM", "Wideband Frequency Modulation"),
            FmVariant::Narrowband => ("NBFM", "Narrowband Frequency Modulation"),
        };

        WaveformInfo {
            name,
            full_name,
            description: "Modulates audio by varying carrier frequency",
            complexity: 2,
            bits_per_symbol: 0, // Analog - no discrete symbols
            carries_data: false, // Carries audio, not digital data
            characteristics: &[
                "Instantaneous frequency varies with audio",
                "Constant envelope (power efficient)",
                "Better noise immunity than AM",
                "Wider bandwidth than AM",
                "Capture effect rejects weaker signals",
            ],
            history: "FM was developed by Edwin Armstrong in the 1930s as an improvement \
                over AM, offering better sound quality and noise immunity. Despite initial \
                resistance from the AM broadcasting establishment, FM became the standard \
                for high-fidelity audio broadcasting by the 1960s-70s.",
            modern_usage: "FM broadcast radio (88-108 MHz) remains dominant for music \
                and local programming worldwide. NBFM is standard for land mobile radio, \
                public safety, aviation, and amateur radio. FM's constant envelope makes \
                it ideal for power-efficient transmitters.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // For analog FM, interpret input as audio samples
        // Each byte is a signed audio sample (-128 to 127 mapped to -1.0 to 1.0)
        let audio: Vec<f64> = data.iter().map(|&b| (b as i8) as f64 / 128.0).collect();
        self.modulate_audio(&audio)
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let audio = self.demodulate_audio(samples);

        // Convert audio back to bytes
        let bits: Vec<u8> = audio
            .iter()
            .map(|&x| ((x * 128.0).clamp(-128.0, 127.0) as i8) as u8)
            .collect();

        DemodResult {
            bits,
            symbols: Vec::new(),
            ber_estimate: None,
            snr_estimate: None,
            metadata: std::collections::HashMap::new(),
        }
    }

    fn samples_per_symbol(&self) -> usize {
        // Analog - no discrete symbols
        // Return 1 as a placeholder
        1
    }

    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);

        // For FM, show instantaneous frequency via phase differences
        let freq_profile: Vec<f64> = if samples.len() > 1 {
            samples
                .windows(2)
                .map(|pair| {
                    let phase0 = pair[0].im.atan2(pair[0].re);
                    let phase1 = pair[1].im.atan2(pair[1].re);
                    let mut diff = phase1 - phase0;
                    while diff > PI {
                        diff -= 2.0 * PI;
                    }
                    while diff < -PI {
                        diff += 2.0 * PI;
                    }
                    diff * self.common.sample_rate / (2.0 * PI)
                })
                .collect()
        } else {
            Vec::new()
        };

        VisualizationData {
            samples,
            constellation: vec![
                IQSample::new(self.common.amplitude, 0.0),
            ],
            constellation_labels: vec!["Carrier".to_string()],
            spectrum: freq_profile,
            description: format!(
                "FM ({}): Δf={:.0} kHz, BW={:.0} kHz (Carson), β={:.1}",
                match self.variant {
                    FmVariant::Wideband => "WBFM",
                    FmVariant::Narrowband => "NBFM",
                },
                self.freq_deviation / 1000.0,
                self.carson_bandwidth() / 1000.0,
                self.modulation_index()
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_fm_modulation() {
        let fm = FM::broadcast(480000.0, 10000.0);

        // Simple sine wave audio at 1 kHz
        let audio: Vec<f64> = (0..4800)
            .map(|n| (2.0 * PI * 1000.0 * n as f64 / 480000.0).sin())
            .collect();

        let samples = fm.modulate_audio(&audio);
        assert_eq!(samples.len(), 4800);

        // FM signal should have constant envelope
        let envs: Vec<f64> = samples.iter().map(|s| s.norm()).collect();
        let min_env = envs.iter().cloned().fold(f64::MAX, f64::min);
        let max_env = envs.iter().cloned().fold(0.0, f64::max);

        // Envelope should be nearly constant (within 1%)
        assert!((max_env - min_env) / max_env < 0.01, "FM should have constant envelope");
    }

    #[test]
    fn test_fm_roundtrip() {
        let fm = FM::narrowband(48000.0, 1000.0);

        // Low frequency test audio
        let audio: Vec<f64> = (0..1000)
            .map(|n| (2.0 * PI * 100.0 * n as f64 / 48000.0).sin() * 0.5)
            .collect();

        let samples = fm.modulate_audio(&audio);
        let recovered = fm.demodulate_audio(&samples);

        // Check rough recovery (demod produces n-1 samples due to differentiation)
        assert_eq!(recovered.len(), audio.len() - 1);
    }

    #[test]
    fn test_fm_variants() {
        let wbfm = FM::broadcast(480000.0, 10000.0);
        assert_eq!(wbfm.info().name, "WBFM");
        assert_eq!(wbfm.freq_deviation(), 75000.0);

        let nbfm = FM::narrowband(48000.0, 1000.0);
        assert_eq!(nbfm.info().name, "NBFM");
        assert_eq!(nbfm.freq_deviation(), 2500.0);
    }

    #[test]
    fn test_fm_modulation_index() {
        // FM broadcast: 75 kHz deviation, 15 kHz audio => β = 5
        let fm = FM::broadcast(480000.0, 10000.0);
        assert!((fm.modulation_index() - 5.0).abs() < 0.01);

        // NBFM: 2.5 kHz deviation, 3 kHz audio => β ≈ 0.83
        let nbfm = FM::narrowband(48000.0, 1000.0);
        assert!((nbfm.modulation_index() - 0.833).abs() < 0.01);
    }

    #[test]
    fn test_carson_bandwidth() {
        // FM broadcast: 2(75 + 15) = 180 kHz
        let fm = FM::broadcast(480000.0, 10000.0);
        assert!((fm.carson_bandwidth() - 180000.0).abs() < 1.0);
    }
}
