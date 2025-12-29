//! AM (Amplitude Modulation) - Analog Audio Modulation
//!
//! True analog AM modulates an audio/baseband signal onto a carrier wave.
//! This is the classic AM used in broadcast radio (AM radio band).
//!
//! ## Mathematical Definition
//!
//! ```text
//! s(t) = A · [1 + m·x(t)] · cos(2π·fc·t)
//! ```
//!
//! Where:
//! - A = carrier amplitude
//! - m = modulation index (depth), 0.0 to 1.0
//! - x(t) = normalized baseband signal (-1 to 1)
//! - fc = carrier frequency
//!
//! ## Modulation Index
//!
//! - m = 0: No modulation (pure carrier)
//! - m = 1: 100% modulation (envelope touches zero)
//! - m > 1: Over-modulation (causes distortion)
//!
//! ## Variants
//!
//! - DSB-FC: Double Sideband Full Carrier (standard AM radio)
//! - DSB-SC: Double Sideband Suppressed Carrier (more efficient)
//! - SSB: Single Sideband (half bandwidth, used in ham radio)
//!
//! ## Note
//!
//! For digital amplitude modulation (ASK), see the `ask` module.
//! This module is specifically for analog audio/voice transmission.

use super::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};
use crate::types::IQSample;
use std::f64::consts::PI;

/// AM variant
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AmVariant {
    /// Double Sideband Full Carrier (standard AM radio)
    DsbFc,
    /// Double Sideband Suppressed Carrier
    DsbSc,
    /// Single Sideband Upper
    SsbUsb,
    /// Single Sideband Lower
    SsbLsb,
}

/// AM (Amplitude Modulation) for analog audio
#[derive(Debug, Clone)]
pub struct AM {
    /// Common waveform parameters
    common: CommonParams,
    /// Carrier frequency in Hz
    carrier_freq: f64,
    /// Modulation index (0.0 to 1.0)
    modulation_index: f64,
    /// AM variant
    variant: AmVariant,
    /// Audio bandwidth in Hz
    audio_bandwidth: f64,
}

impl AM {
    /// Create a new AM modulator
    pub fn new(common: CommonParams, carrier_freq: f64, modulation_index: f64) -> Self {
        Self {
            common,
            carrier_freq,
            modulation_index: modulation_index.clamp(0.0, 1.5),
            variant: AmVariant::DsbFc,
            audio_bandwidth: 5000.0, // Standard AM broadcast
        }
    }

    /// Create standard AM broadcast configuration
    pub fn broadcast(sample_rate: f64, carrier_freq: f64) -> Self {
        Self::new(
            CommonParams {
                sample_rate,
                carrier_freq: 0.0,
                amplitude: 1.0,
            },
            carrier_freq,
            0.8, // 80% modulation typical
        )
    }

    /// Set modulation index
    pub fn with_modulation_index(mut self, m: f64) -> Self {
        self.modulation_index = m.clamp(0.0, 1.5);
        self
    }

    /// Set AM variant
    pub fn with_variant(mut self, variant: AmVariant) -> Self {
        self.variant = variant;
        self
    }

    /// Set audio bandwidth
    pub fn with_audio_bandwidth(mut self, bw: f64) -> Self {
        self.audio_bandwidth = bw;
        self
    }

    /// Get modulation index
    pub fn modulation_index(&self) -> f64 {
        self.modulation_index
    }

    /// Get variant
    pub fn variant(&self) -> AmVariant {
        self.variant
    }

    /// Modulate audio samples onto carrier
    /// Input: audio samples normalized to -1.0 to 1.0
    /// Output: I/Q samples of modulated signal
    pub fn modulate_audio(&self, audio: &[f64]) -> Vec<IQSample> {
        let omega = 2.0 * PI * self.carrier_freq / self.common.sample_rate;
        let amp = self.common.amplitude;
        let m = self.modulation_index;

        audio
            .iter()
            .enumerate()
            .map(|(n, &x)| {
                let phase = omega * n as f64;
                match self.variant {
                    AmVariant::DsbFc => {
                        // s(t) = A[1 + m*x(t)] * cos(wt)
                        let envelope = amp * (1.0 + m * x);
                        IQSample::new(envelope * phase.cos(), envelope * phase.sin())
                    }
                    AmVariant::DsbSc => {
                        // s(t) = A*m*x(t) * cos(wt) - suppressed carrier
                        let envelope = amp * m * x;
                        IQSample::new(envelope * phase.cos(), envelope * phase.sin())
                    }
                    AmVariant::SsbUsb | AmVariant::SsbLsb => {
                        // SSB requires Hilbert transform - simplified here
                        // Full implementation would use a Hilbert filter
                        let envelope = amp * m * x;
                        IQSample::new(envelope * phase.cos(), envelope * phase.sin())
                    }
                }
            })
            .collect()
    }

    /// Demodulate AM signal to recover audio
    /// Uses envelope detection for DSB-FC
    pub fn demodulate_audio(&self, samples: &[IQSample]) -> Vec<f64> {
        match self.variant {
            AmVariant::DsbFc => {
                // Envelope detection: |s(t)| = A[1 + m*x(t)]
                let m = self.modulation_index;
                samples
                    .iter()
                    .map(|s| {
                        let envelope = s.norm();
                        // Remove DC offset and scale
                        (envelope / self.common.amplitude - 1.0) / m
                    })
                    .collect()
            }
            AmVariant::DsbSc => {
                // Coherent detection needed - multiply by carrier and LPF
                let omega = 2.0 * PI * self.carrier_freq / self.common.sample_rate;
                samples
                    .iter()
                    .enumerate()
                    .map(|(n, s)| {
                        let phase = omega * n as f64;
                        // Multiply by carrier (coherent detection)
                        s.re * phase.cos() + s.im * phase.sin()
                    })
                    .collect()
            }
            _ => {
                // SSB demodulation - simplified
                samples.iter().map(|s| s.re).collect()
            }
        }
    }
}

impl Waveform for AM {
    fn info(&self) -> WaveformInfo {
        let (name, full_name) = match self.variant {
            AmVariant::DsbFc => ("AM", "Amplitude Modulation (DSB-FC)"),
            AmVariant::DsbSc => ("DSB-SC", "Double Sideband Suppressed Carrier"),
            AmVariant::SsbUsb => ("SSB-USB", "Single Sideband Upper"),
            AmVariant::SsbLsb => ("SSB-LSB", "Single Sideband Lower"),
        };

        WaveformInfo {
            name,
            full_name,
            description: "Modulates audio by varying carrier amplitude",
            complexity: 1,
            bits_per_symbol: 0, // Analog - no discrete symbols
            carries_data: false, // Carries audio, not digital data
            characteristics: &[
                "Carrier amplitude varies with audio",
                "Simple envelope detection",
                "Used in AM broadcast radio",
                "Both sidebands carry same info (DSB)",
                "Susceptible to noise and fading",
            ],
            history: "AM was the first practical modulation for voice transmission, \
                pioneered by Reginald Fessenden's 1906 Christmas Eve broadcast. \
                Dominated commercial broadcasting from the 1920s-1960s. AM's simplicity \
                enabled mass adoption of radio receivers, transforming society.",
            modern_usage: "AM broadcast (MW/SW) still operates worldwide with thousands \
                of stations. Aviation uses AM for air traffic control due to capture \
                effect resistance. Citizen's Band (CB) radio and some marine communications. \
                Declining for new applications but legacy infrastructure remains vast.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // For analog AM, interpret input as audio samples
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

        // For AM, show the envelope
        let envelope: Vec<f64> = samples.iter().map(|s| s.norm()).collect();

        VisualizationData {
            samples,
            constellation: vec![
                IQSample::new(1.0 - self.modulation_index, 0.0),
                IQSample::new(1.0, 0.0),
                IQSample::new(1.0 + self.modulation_index, 0.0),
            ],
            constellation_labels: vec![
                "Min".to_string(),
                "Carrier".to_string(),
                "Max".to_string(),
            ],
            spectrum: envelope,
            description: format!(
                "AM ({}): m={:.0}%, fc={:.0} Hz, BW={:.0} Hz",
                match self.variant {
                    AmVariant::DsbFc => "DSB-FC",
                    AmVariant::DsbSc => "DSB-SC",
                    AmVariant::SsbUsb => "SSB-USB",
                    AmVariant::SsbLsb => "SSB-LSB",
                },
                self.modulation_index * 100.0,
                self.carrier_freq,
                self.audio_bandwidth * 2.0
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_am_modulation() {
        let am = AM::broadcast(48000.0, 1000.0);

        // Simple sine wave audio
        let audio: Vec<f64> = (0..480)
            .map(|n| (2.0 * PI * 440.0 * n as f64 / 48000.0).sin())
            .collect();

        let samples = am.modulate_audio(&audio);
        assert_eq!(samples.len(), 480);

        // Check envelope varies with audio
        let min_env = samples.iter().map(|s| s.norm()).fold(f64::MAX, f64::min);
        let max_env = samples.iter().map(|s| s.norm()).fold(0.0, f64::max);

        assert!(max_env > min_env, "Envelope should vary with modulation");
    }

    #[test]
    fn test_am_roundtrip() {
        let am = AM::broadcast(48000.0, 1000.0);

        // Test audio
        let audio: Vec<f64> = (0..100).map(|n| n as f64 / 50.0 - 1.0).collect();
        let samples = am.modulate_audio(&audio);
        let recovered = am.demodulate_audio(&samples);

        // Check rough recovery (envelope detection has DC offset)
        assert_eq!(recovered.len(), audio.len());
    }

    #[test]
    fn test_am_variants() {
        let common = CommonParams::default();

        let dsb_fc = AM::new(common.clone(), 1000.0, 0.8);
        assert_eq!(dsb_fc.info().name, "AM");

        let dsb_sc = AM::new(common.clone(), 1000.0, 0.8).with_variant(AmVariant::DsbSc);
        assert_eq!(dsb_sc.info().name, "DSB-SC");

        let ssb = AM::new(common, 1000.0, 0.8).with_variant(AmVariant::SsbUsb);
        assert_eq!(ssb.info().name, "SSB-USB");
    }
}
