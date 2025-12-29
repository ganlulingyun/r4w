//! Ultra-Wideband (UWB) Impulse Radio
//!
//! UWB Impulse Radio transmits very short pulses (typically < 1 ns) that
//! spread energy across a very wide bandwidth (> 500 MHz). This provides
//! excellent LPD/LPI characteristics and precise ranging capability.
//!
//! ## Key Characteristics
//!
//! - **Bandwidth**: > 500 MHz (FCC definition) or > 20% fractional bandwidth
//! - **Pulse Duration**: Sub-nanosecond to few nanoseconds
//! - **Power Spectral Density**: Very low (-41.3 dBm/MHz FCC limit)
//! - **Range Resolution**: ~cm level (due to wide bandwidth)
//!
//! ## Modulation Schemes
//!
//! - **OOK**: On-Off Keying (pulse present/absent)
//! - **PPM**: Pulse Position Modulation (early/late pulse)
//! - **BPSK**: Binary Phase Shift (pulse polarity)
//! - **TH-UWB**: Time-Hopping for multiple access
//!
//! ## LPD/LPI Properties
//!
//! With 500 MHz bandwidth and -41.3 dBm/MHz limit:
//! - Total power ≈ -14.3 dBm (37 µW)
//! - Appears as noise to narrowband receivers
//! - Processing gain proportional to bandwidth/data_rate
//!
//! ## Applications
//!
//! - Indoor positioning (Apple AirTag, Samsung SmartTag)
//! - Automotive (keyless entry, radar)
//! - Secure communications
//! - Through-wall imaging
//! - High-precision ranging

use super::{CommonParams, DemodResult, VisualizationData, Waveform, WaveformInfo};
use crate::types::IQSample;
use std::f64::consts::PI;

/// Unpack bytes to individual bits (MSB first)
fn bytes_to_bits(data: &[u8]) -> Vec<u8> {
    let mut bits = Vec::with_capacity(data.len() * 8);
    for byte in data {
        for i in (0..8).rev() {
            bits.push((byte >> i) & 1);
        }
    }
    bits
}

/// Pack individual bits to bytes (MSB first)
fn bits_to_bytes(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8)
        .map(|chunk| {
            chunk.iter()
                .enumerate()
                .fold(0u8, |acc, (i, &bit)| {
                    acc | ((bit & 1) << (7 - i))
                })
        })
        .collect()
}

/// Check if data is packed bytes (contains values > 1)
fn is_packed_bytes(data: &[u8]) -> bool {
    data.iter().any(|&b| b > 1)
}

/// UWB pulse shape
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PulseShape {
    /// Gaussian monocycle (first derivative of Gaussian)
    GaussianMonocycle,
    /// Gaussian doublet (second derivative of Gaussian)
    GaussianDoublet,
    /// Raised cosine pulse
    RaisedCosine,
    /// Rectangular pulse (for simulation)
    Rectangular,
}

/// UWB modulation scheme
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum UwbModulation {
    /// On-Off Keying: pulse present (1) or absent (0)
    Ook,
    /// Binary Phase Shift Keying: positive (0) or negative (1) pulse
    Bpsk,
    /// Pulse Position Modulation: early (0) or late (1) pulse
    Ppm {
        /// Time shift in samples for PPM
        shift_samples: usize,
    },
}

impl UwbModulation {
    /// Bits per symbol
    pub fn bits_per_symbol(&self) -> usize {
        1 // All basic UWB modulations are binary
    }
}

/// UWB Impulse Radio configuration
#[derive(Debug, Clone)]
pub struct UwbConfig {
    /// Pulse shape
    pub pulse_shape: PulseShape,
    /// Modulation scheme
    pub modulation: UwbModulation,
    /// Pulse duration (at -10 dB points) in seconds
    pub pulse_duration_s: f64,
    /// Pulse repetition interval in seconds
    pub pulse_interval_s: f64,
    /// Number of pulses per bit (for integration gain)
    pub pulses_per_bit: usize,
}

impl Default for UwbConfig {
    fn default() -> Self {
        Self {
            pulse_shape: PulseShape::GaussianMonocycle,
            modulation: UwbModulation::Bpsk,
            pulse_duration_s: 1e-9,        // 1 ns pulse
            pulse_interval_s: 100e-9,      // 100 ns between pulses
            pulses_per_bit: 1,
        }
    }
}

/// UWB Impulse Radio modulator/demodulator
#[derive(Debug, Clone)]
pub struct UwbIr {
    /// Common waveform parameters
    common: CommonParams,
    /// UWB configuration
    config: UwbConfig,
    /// Pre-computed pulse template
    pulse_template: Vec<f64>,
    /// Samples per pulse interval
    samples_per_interval: usize,
}

impl UwbIr {
    /// Create a new UWB Impulse Radio modulator
    pub fn new(common: CommonParams, config: UwbConfig) -> Self {
        // Ensure samples_per_interval is at least 1 to prevent division by zero
        let samples_per_interval =
            ((config.pulse_interval_s * common.sample_rate) as usize).max(1);

        // Generate pulse template
        let pulse_samples = (config.pulse_duration_s * common.sample_rate * 4.0) as usize;
        let pulse_samples = pulse_samples.max(8);
        let pulse_template = Self::generate_pulse(&config.pulse_shape, pulse_samples);

        Self {
            common,
            config,
            pulse_template,
            samples_per_interval,
        }
    }

    /// Create with default configuration
    pub fn default_config(sample_rate: f64) -> Self {
        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        Self::new(common, UwbConfig::default())
    }

    /// Create IEEE 802.15.4a-like UWB (simplified)
    pub fn ieee_802_15_4a(sample_rate: f64) -> Self {
        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let config = UwbConfig {
            pulse_shape: PulseShape::GaussianDoublet,
            modulation: UwbModulation::Bpsk,
            pulse_duration_s: 2e-9,        // ~2 ns pulse
            pulse_interval_s: 128e-9,      // 128 ns (Tc)
            pulses_per_bit: 16,            // Multiple pulses per bit
        };
        Self::new(common, config)
    }

    /// Generate pulse shape
    fn generate_pulse(shape: &PulseShape, num_samples: usize) -> Vec<f64> {
        let mut pulse = vec![0.0; num_samples];
        let center = num_samples as f64 / 2.0;
        let sigma = num_samples as f64 / 6.0; // Pulse width parameter

        match shape {
            PulseShape::GaussianMonocycle => {
                // First derivative of Gaussian: -t * exp(-t²/2σ²)
                for i in 0..num_samples {
                    let t = (i as f64 - center) / sigma;
                    pulse[i] = -t * (-t * t / 2.0).exp();
                }
            }
            PulseShape::GaussianDoublet => {
                // Second derivative of Gaussian: (t² - 1) * exp(-t²/2σ²)
                for i in 0..num_samples {
                    let t = (i as f64 - center) / sigma;
                    pulse[i] = (t * t - 1.0) * (-t * t / 2.0).exp();
                }
            }
            PulseShape::RaisedCosine => {
                for i in 0..num_samples {
                    let t = i as f64 / num_samples as f64;
                    pulse[i] = 0.5 * (1.0 - (2.0 * PI * t).cos());
                }
            }
            PulseShape::Rectangular => {
                let start = num_samples / 4;
                let end = 3 * num_samples / 4;
                for i in start..end {
                    pulse[i] = 1.0;
                }
            }
        }

        // Normalize energy
        let energy: f64 = pulse.iter().map(|&x| x * x).sum();
        if energy > 0.0 {
            let norm = energy.sqrt();
            for p in &mut pulse {
                *p /= norm;
            }
        }

        pulse
    }

    /// Get pulse bandwidth (approximate -10 dB bandwidth)
    pub fn bandwidth(&self) -> f64 {
        // Approximate: BW ≈ 1 / pulse_duration for Gaussian pulses
        1.0 / self.config.pulse_duration_s
    }

    /// Get data rate in bits/second
    pub fn data_rate(&self) -> f64 {
        1.0 / (self.config.pulse_interval_s * self.config.pulses_per_bit as f64)
    }

    /// Get processing gain in dB
    pub fn processing_gain_db(&self) -> f64 {
        // PG ≈ bandwidth / data_rate
        10.0 * (self.bandwidth() / self.data_rate()).log10()
    }

    /// Get pulse repetition frequency
    pub fn prf(&self) -> f64 {
        1.0 / self.config.pulse_interval_s
    }

    /// Modulate a single bit
    fn modulate_bit(&self, bit: u8) -> Vec<IQSample> {
        let mut samples = vec![IQSample::new(0.0, 0.0); self.samples_per_interval];
        let _pulse_len = self.pulse_template.len();

        match self.config.modulation {
            UwbModulation::Ook => {
                if bit == 1 {
                    // Place pulse at start of interval
                    for (i, &p) in self.pulse_template.iter().enumerate() {
                        if i < samples.len() {
                            samples[i] = IQSample::new(
                                p * self.common.amplitude,
                                0.0,
                            );
                        }
                    }
                }
                // bit == 0: no pulse (samples stay zero)
            }
            UwbModulation::Bpsk => {
                let polarity = if bit == 0 { 1.0 } else { -1.0 };
                for (i, &p) in self.pulse_template.iter().enumerate() {
                    if i < samples.len() {
                        samples[i] = IQSample::new(
                            p * polarity * self.common.amplitude,
                            0.0,
                        );
                    }
                }
            }
            UwbModulation::Ppm { shift_samples } => {
                let offset = if bit == 0 { 0 } else { shift_samples };
                for (i, &p) in self.pulse_template.iter().enumerate() {
                    let idx = i + offset;
                    if idx < samples.len() {
                        samples[idx] = IQSample::new(
                            p * self.common.amplitude,
                            0.0,
                        );
                    }
                }
            }
        }

        samples
    }

    /// Demodulate samples to recover a bit
    fn demodulate_bit(&self, samples: &[IQSample]) -> u8 {
        match self.config.modulation {
            UwbModulation::Ook => {
                // Energy detection
                let energy: f64 = samples.iter()
                    .take(self.pulse_template.len())
                    .map(|s| s.re * s.re + s.im * s.im)
                    .sum();
                if energy > 0.1 { 1 } else { 0 }
            }
            UwbModulation::Bpsk => {
                // Correlate with pulse template
                let correlation: f64 = samples.iter()
                    .zip(self.pulse_template.iter())
                    .map(|(s, &p)| s.re * p)
                    .sum();
                if correlation >= 0.0 { 0 } else { 1 }
            }
            UwbModulation::Ppm { shift_samples } => {
                // Compare energy in early vs late positions
                let early_energy: f64 = samples.iter()
                    .take(self.pulse_template.len())
                    .map(|s| s.re * s.re)
                    .sum();

                let late_start = shift_samples.min(samples.len());
                let late_energy: f64 = samples.iter()
                    .skip(late_start)
                    .take(self.pulse_template.len())
                    .map(|s| s.re * s.re)
                    .sum();

                if early_energy >= late_energy { 0 } else { 1 }
            }
        }
    }
}

impl Waveform for UwbIr {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "UWB-IR",
            full_name: "Ultra-Wideband Impulse Radio",
            description: "Sub-nanosecond pulses for LPD/LPI and precise ranging",
            complexity: 4,
            bits_per_symbol: self.config.modulation.bits_per_symbol() as u8,
            carries_data: true,
            characteristics: &[
                "Very short pulses (< 1 ns)",
                "Wide bandwidth (> 500 MHz)",
                "Very low power spectral density",
                "Excellent LPD/LPI properties",
                "Centimeter-level ranging accuracy",
                "Coexists with narrowband systems",
            ],
            history: "UWB research began in the 1960s at MIT Lincoln Labs. The FCC \
                authorized unlicensed UWB in 2002 with strict power limits (-41.3 dBm/MHz). \
                IEEE 802.15.4a added UWB PHY in 2007. Initially for communications, UWB \
                is now primarily used for positioning and ranging.",
            modern_usage: "Apple's U1 chip uses UWB for AirTag/AirDrop spatial awareness. \
                Samsung, Google, and BMW implement UWB for device finding and secure car keys. \
                IEEE 802.15.4z enhanced security and ranging. Industrial applications include \
                asset tracking and robot navigation. FiRa Consortium standardizes interoperability.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // Convert packed bytes to individual bits if needed
        let bits = if is_packed_bytes(data) {
            bytes_to_bits(data)
        } else {
            data.to_vec()
        };

        let mut samples = Vec::new();

        for &bit in &bits {
            // Optionally repeat pulses for integration gain
            for _ in 0..self.config.pulses_per_bit {
                let bit_samples = self.modulate_bit(bit & 1);
                samples.extend(bit_samples);
            }
        }

        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let mut result = DemodResult::default();
        let samples_per_bit = self.samples_per_interval * self.config.pulses_per_bit;

        // Collect individual bits first
        let mut individual_bits = Vec::new();

        for bit_samples in samples.chunks(samples_per_bit) {
            if bit_samples.len() < samples_per_bit {
                break;
            }

            // If multiple pulses per bit, integrate
            if self.config.pulses_per_bit > 1 {
                let mut integrated = vec![IQSample::new(0.0, 0.0); self.samples_per_interval];
                for pulse_idx in 0..self.config.pulses_per_bit {
                    let start = pulse_idx * self.samples_per_interval;
                    for (i, sample) in bit_samples.iter().skip(start).take(self.samples_per_interval).enumerate() {
                        integrated[i].re += sample.re;
                        integrated[i].im += sample.im;
                    }
                }
                let bit = self.demodulate_bit(&integrated);
                individual_bits.push(bit);
            } else {
                let bit = self.demodulate_bit(bit_samples);
                individual_bits.push(bit);
            }
        }

        // Pack individual bits into bytes for consistent output
        result.bits = bits_to_bytes(&individual_bits);

        result.metadata.insert("bandwidth_hz".to_string(), self.bandwidth());
        result.metadata.insert("processing_gain_db".to_string(), self.processing_gain_db());
        result.metadata.insert("prf_hz".to_string(), self.prf());
        result.metadata.insert("data_rate_bps".to_string(), self.data_rate());

        result
    }

    fn samples_per_symbol(&self) -> usize {
        self.samples_per_interval * self.config.pulses_per_bit
    }

    fn get_visualization(&self, data: &[u8]) -> VisualizationData {
        let samples = self.modulate(data);

        // For UWB, show the pulse shape as constellation-like visualization
        let pulse_viz: Vec<IQSample> = self.pulse_template.iter()
            .enumerate()
            .map(|(i, &p)| IQSample::new(i as f64 / self.pulse_template.len() as f64, p))
            .collect();

        VisualizationData {
            samples,
            constellation: pulse_viz,
            constellation_labels: vec!["Pulse Shape".into()],
            spectrum: Vec::new(),
            description: format!(
                "UWB-IR: {:.0} MHz BW, {:.1} dB gain, {:?}",
                self.bandwidth() / 1e6,
                self.processing_gain_db(),
                self.config.modulation
            ),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_uwb_basic() {
        let uwb = UwbIr::default_config(10e9); // 10 GHz sample rate for ns pulses

        assert!(uwb.bandwidth() > 500e6); // > 500 MHz
        // Default: 1 GHz BW, 10 Mbps => 20 dB processing gain
        assert!(uwb.processing_gain_db() >= 20.0);
    }

    #[test]
    fn test_uwb_roundtrip_bpsk() {
        let uwb = UwbIr::default_config(10e9);

        let data: Vec<u8> = vec![0, 1, 0, 1, 1, 0, 0, 1];
        let modulated = uwb.modulate(&data);
        let result = uwb.demodulate(&modulated);

        // Demodulate returns packed bytes, so compare against packed input
        let expected = bits_to_bytes(&data);
        assert_eq!(result.bits.len(), expected.len());
        assert_eq!(result.bits, expected);
    }

    #[test]
    fn test_uwb_ook() {
        let common = CommonParams {
            sample_rate: 10e9,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let config = UwbConfig {
            modulation: UwbModulation::Ook,
            ..Default::default()
        };
        let uwb = UwbIr::new(common, config);

        // Use 8 bits to get full byte
        let data: Vec<u8> = vec![1, 0, 1, 1, 0, 0, 1, 0];
        let modulated = uwb.modulate(&data);
        let result = uwb.demodulate(&modulated);

        // Demodulate returns packed bytes
        assert_eq!(result.bits, bits_to_bytes(&data));
    }

    #[test]
    fn test_uwb_ppm() {
        let common = CommonParams {
            sample_rate: 10e9,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };
        let config = UwbConfig {
            modulation: UwbModulation::Ppm { shift_samples: 50 },
            ..Default::default()
        };
        let uwb = UwbIr::new(common, config);

        // Use 8 bits to get full byte
        let data: Vec<u8> = vec![0, 1, 0, 1, 1, 0, 1, 0];
        let modulated = uwb.modulate(&data);
        let result = uwb.demodulate(&modulated);

        // Demodulate returns packed bytes
        assert_eq!(result.bits, bits_to_bytes(&data));
    }

    #[test]
    fn test_pulse_shapes() {
        for shape in [
            PulseShape::GaussianMonocycle,
            PulseShape::GaussianDoublet,
            PulseShape::RaisedCosine,
            PulseShape::Rectangular,
        ] {
            let pulse = UwbIr::generate_pulse(&shape, 64);

            // Pulse should be normalized
            let energy: f64 = pulse.iter().map(|&x| x * x).sum();
            assert!((energy - 1.0).abs() < 0.01, "Pulse energy should be ~1.0");
        }
    }

    #[test]
    fn test_ieee_802_15_4a() {
        let uwb = UwbIr::ieee_802_15_4a(10e9);

        assert_eq!(uwb.config.pulses_per_bit, 16);
        assert!(uwb.processing_gain_db() > 30.0); // Higher due to pulse integration
    }
}
