//! CW (Continuous Wave) - The Simplest Waveform
//!
//! A CW signal is a pure sinusoidal tone at a constant frequency.
//! It's the building block for all other modulation schemes.
//!
//! ## Mathematical Definition
//!
//! ```text
//! s(t) = A · e^(j·2π·f·t) = A · (cos(2πft) + j·sin(2πft))
//! ```
//!
//! ## Uses
//!
//! - Morse code (on/off keying of CW)
//! - Carrier signal for modulation
//! - Calibration and testing
//! - Beacons
//!
//! ## Characteristics
//!
//! - Carries NO data by itself
//! - Appears as single spike in frequency domain
//! - Rotates at constant rate in I/Q plane

use super::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};
use crate::types::IQSample;
use std::f64::consts::PI;

/// Continuous Wave (pure tone) generator
#[derive(Debug, Clone)]
pub struct CW {
    /// Common waveform parameters
    common: CommonParams,
    /// Tone frequency relative to carrier (Hz)
    frequency: f64,
    /// Initial phase (radians)
    initial_phase: f64,
}

impl CW {
    /// Create a new CW generator
    ///
    /// # Arguments
    /// * `common` - Common parameters (sample rate, amplitude)
    /// * `frequency` - Tone frequency in Hz
    pub fn new(common: CommonParams, frequency: f64) -> Self {
        Self {
            common,
            frequency,
            initial_phase: 0.0,
        }
    }

    /// Create with specific initial phase
    pub fn with_phase(mut self, phase: f64) -> Self {
        self.initial_phase = phase;
        self
    }

    /// Set frequency
    pub fn set_frequency(&mut self, freq: f64) {
        self.frequency = freq;
    }

    /// Get frequency
    pub fn frequency(&self) -> f64 {
        self.frequency
    }

    /// Generate samples for a specific duration
    pub fn generate(&self, duration_seconds: f64) -> Vec<IQSample> {
        let num_samples = (self.common.sample_rate * duration_seconds) as usize;
        self.generate_samples(num_samples)
    }

    /// Generate a specific number of samples
    pub fn generate_samples(&self, num_samples: usize) -> Vec<IQSample> {
        let omega = 2.0 * PI * self.frequency / self.common.sample_rate;
        let amp = self.common.amplitude;

        (0..num_samples)
            .map(|n| {
                let phase = self.initial_phase + omega * n as f64;
                IQSample::new(amp * phase.cos(), amp * phase.sin())
            })
            .collect()
    }
}

impl Waveform for CW {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "CW",
            full_name: "Continuous Wave",
            description: "Pure sinusoidal tone - the simplest possible signal",
            complexity: 1,
            bits_per_symbol: 0,
            carries_data: false,
            characteristics: &[
                "Single frequency component",
                "Constant amplitude",
                "Rotates in I/Q plane",
                "Building block for all modulation",
            ],
            history: "CW dates to Marconi's first radio transmissions (1895). \
                Used for Morse code telegraphy, it was the dominant mode until voice \
                transmission became practical in the 1920s. Ham radio operators still \
                use CW for its efficiency and ability to get through noise.",
            modern_usage: "Still actively used by amateur radio operators worldwide. \
                CW remains popular for weak-signal work, contests, and emergency \
                communications due to its narrow bandwidth and human-decodable nature. \
                Also used as a reference signal in test equipment and calibration.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, _data: &[u8]) -> Vec<IQSample> {
        // CW doesn't encode data, generate 1ms of tone
        self.generate(0.001)
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        // CW carries no data, but we can estimate frequency and SNR
        let mut result = DemodResult::default();

        if samples.is_empty() {
            return result;
        }

        // Estimate signal power
        let power: f64 = samples.iter().map(|s| s.norm_sqr()).sum::<f64>() / samples.len() as f64;
        result.metadata.insert("power".to_string(), power);

        // Estimate frequency from phase rotation
        if samples.len() >= 2 {
            let mut phase_diffs = Vec::new();
            for i in 1..samples.len() {
                let phase_diff = (samples[i] * samples[i - 1].conj()).arg();
                phase_diffs.push(phase_diff);
            }
            let avg_phase_diff: f64 = phase_diffs.iter().sum::<f64>() / phase_diffs.len() as f64;
            let estimated_freq = avg_phase_diff * self.common.sample_rate / (2.0 * PI);
            result.metadata.insert("estimated_freq".to_string(), estimated_freq);
        }

        result
    }

    fn samples_per_symbol(&self) -> usize {
        // CW has no symbols, return samples for one cycle (minimum 1)
        if self.frequency <= 0.0 {
            return 1;
        }
        ((self.common.sample_rate / self.frequency).ceil() as usize).max(1)
    }

    fn get_visualization(&self, _data: &[u8]) -> VisualizationData {
        // Generate a few cycles for visualization
        let cycles = 4.0;
        let duration = cycles / self.frequency;
        let samples = self.generate(duration);

        // Constellation for CW is a circle
        let constellation: Vec<IQSample> = (0..32)
            .map(|i| {
                let angle = 2.0 * PI * i as f64 / 32.0;
                IQSample::new(
                    self.common.amplitude * angle.cos(),
                    self.common.amplitude * angle.sin(),
                )
            })
            .collect();

        VisualizationData {
            samples,
            constellation,
            constellation_labels: vec!["CW traces a circle as phase rotates".to_string()],
            spectrum: Vec::new(),
            description: format!(
                "CW tone at {} Hz - watch the I/Q point rotate",
                self.frequency
            ),
        }
    }

    fn generate_demo(&self, duration_ms: f64) -> Vec<IQSample> {
        self.generate(duration_ms / 1000.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cw_generation() {
        let common = CommonParams {
            sample_rate: 1000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let cw = CW::new(common, 100.0); // 100 Hz tone

        let samples = cw.generate(0.01); // 10ms = 10 samples at 1000 Hz
        assert_eq!(samples.len(), 10);

        // First sample should be (1, 0) with phase 0
        assert!((samples[0].re - 1.0).abs() < 1e-10);
        assert!(samples[0].im.abs() < 1e-10);

        // All samples should have magnitude ~1
        for s in &samples {
            assert!((s.norm() - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_cw_frequency_estimation() {
        let common = CommonParams {
            sample_rate: 10000.0,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let freq = 500.0;
        let cw = CW::new(common, freq);

        let samples = cw.generate(0.1); // 100ms
        let result = cw.demodulate(&samples);

        let estimated = result.metadata.get("estimated_freq").unwrap();
        assert!((estimated - freq).abs() < 1.0); // Within 1 Hz
    }
}
