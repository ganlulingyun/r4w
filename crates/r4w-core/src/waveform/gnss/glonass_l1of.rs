//! GLONASS L1OF Waveform
//!
//! GLONASS uses FDMA (Frequency Division Multiple Access) instead of CDMA.
//! All satellites transmit the same 511-chip m-sequence spreading code,
//! separated by different carrier frequencies: f = 1602 + k × 0.5625 MHz
//! where k is the frequency channel number (-7 to +6).
//!
//! Educational comparison: FDMA (GLONASS) vs CDMA (GPS) multiple access.

use crate::types::IQSample;
use crate::waveform::{
    CommonParams, DemodResult, ModulationStage, Waveform, WaveformInfo,
};
use rustfft::num_complex::Complex64;

use super::prn::GlonassCodeGenerator;

/// GLONASS L1OF waveform
// trace:FR-036 | ai:claude
#[derive(Debug)]
pub struct GlonassL1of {
    common: CommonParams,
    /// Frequency channel (-7 to +6)
    frequency_channel: i8,
    /// Pre-generated code
    code: Vec<i8>,
    /// Samples per chip
    samples_per_chip: f64,
}

impl GlonassL1of {
    /// GLONASS chipping rate
    pub const CHIPPING_RATE: f64 = 511_000.0;
    /// Code length in chips
    pub const CODE_LENGTH: usize = 511;
    /// Navigation data rate
    pub const NAV_DATA_RATE: f64 = 50.0;
    /// Code periods per nav bit
    pub const CODE_PERIODS_PER_BIT: usize = 10;

    /// Create a new GLONASS L1OF waveform
    pub fn new(sample_rate: f64, frequency_channel: i8) -> Self {
        let mut gen = GlonassCodeGenerator::new(frequency_channel);
        let code = gen.generate_code();
        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };

        Self {
            common,
            frequency_channel,
            code,
            samples_per_chip: sample_rate / Self::CHIPPING_RATE,
        }
    }

    /// Create with default channel 0
    pub fn default_config(sample_rate: f64) -> Self {
        Self::new(sample_rate, 0)
    }
}

impl Waveform for GlonassL1of {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "GLONASS-L1OF",
            full_name: "GLONASS L1 Open Frequency",
            description: "GLONASS civilian signal using BPSK(0.5) with FDMA multiple access",
            complexity: 3,
            bits_per_symbol: 1,
            carries_data: true,
            characteristics: &[
                "BPSK(0.5) modulation",
                "511-chip m-sequence (all SVs share same code)",
                "0.511 Mchip/s chipping rate",
                "50 bps navigation data",
                "FDMA multiple access (14 frequency channels)",
                "1602 + k×0.5625 MHz carrier",
            ],
            history: "GLONASS has been operational since 1993. Unlike GPS which uses \
                      CDMA, GLONASS L1OF uses FDMA to separate satellite signals. \
                      This simplifies the receiver code tracking but requires wider \
                      RF bandwidth to receive all channels.",
            modern_usage: "Used alongside GPS for improved positioning. GLONASS CDMA \
                          signals (L3OC) are being added to new satellites for \
                          interoperability with GPS/Galileo.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // Convert bytes to nav bits
        let nav_bits: Vec<i8> = if data.is_empty() {
            vec![1; 10] // Default pattern
        } else {
            data.iter()
                .flat_map(|&byte| {
                    (0..8).rev().map(move |b| {
                        if (byte >> b) & 1 == 1 { -1i8 } else { 1i8 }
                    })
                })
                .collect()
        };

        let mut signal = Vec::new();

        for &nav_bit in &nav_bits {
            let nav_val = nav_bit as f64;
            for _period in 0..Self::CODE_PERIODS_PER_BIT {
                for chip_idx in 0..Self::CODE_LENGTH {
                    let chip_val = self.code[chip_idx] as f64;
                    let spread_val = nav_val * chip_val * self.common.amplitude;

                    let num_samples = ((chip_idx + 1) as f64 * self.samples_per_chip) as usize
                        - (chip_idx as f64 * self.samples_per_chip) as usize;
                    for _ in 0..num_samples {
                        signal.push(Complex64::new(spread_val, 0.0));
                    }
                }
            }
        }

        signal
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let mut result = DemodResult::default();
        if samples.len() < Self::CODE_LENGTH {
            return result;
        }

        // Simple despreading
        let samples_per_code = (Self::CODE_LENGTH as f64 * self.samples_per_chip) as usize;
        let mut nav_bit_accum = 0.0;
        let mut code_count = 0;
        let mut pos = 0;

        while pos + Self::CODE_LENGTH.min(samples_per_code) <= samples.len() {
            let mut corr = 0.0;
            for (i, &sample) in samples[pos..pos + Self::CODE_LENGTH.min(samples_per_code)].iter().enumerate() {
                let chip_idx = i % Self::CODE_LENGTH;
                corr += sample.re * self.code[chip_idx] as f64;
            }

            nav_bit_accum += corr;
            code_count += 1;

            if code_count >= Self::CODE_PERIODS_PER_BIT {
                result.bits.push(if nav_bit_accum >= 0.0 { 0 } else { 1 });
                nav_bit_accum = 0.0;
                code_count = 0;
            }

            pos += samples_per_code;
        }

        result
    }

    fn samples_per_symbol(&self) -> usize {
        (Self::CODE_PERIODS_PER_BIT as f64 * Self::CODE_LENGTH as f64 * self.samples_per_chip) as usize
    }

    fn get_modulation_stages(&self, data: &[u8]) -> Vec<ModulationStage> {
        vec![
            ModulationStage::new(
                "Navigation Data",
                format!("50 bps navigation data from {} input bytes", data.len()),
            ).with_input_bits(data.to_vec()),
            ModulationStage::new(
                "M-Sequence Spreading",
                format!("Spread with 511-chip m-sequence. All GLONASS SVs use the same code. \
                         Processing gain: {:.1} dB", 10.0 * 511.0_f64.log10()),
            ),
            ModulationStage::new(
                "FDMA Carrier Assignment",
                format!("Frequency channel k={}: f = 1602 + {}×0.5625 = {:.3} MHz",
                    self.frequency_channel, self.frequency_channel,
                    1602.0 + self.frequency_channel as f64 * 0.5625),
            ),
            ModulationStage::new(
                "BPSK(0.5) Modulation",
                "BPSK modulation at 0.511 Mchip/s (half the GPS chipping rate)".to_string(),
            ).with_samples(self.modulate(data)),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_glonass_info() {
        let wf = GlonassL1of::default_config(511_000.0);
        assert_eq!(wf.info().name, "GLONASS-L1OF");
    }

    #[test]
    fn test_glonass_modulate() {
        let wf = GlonassL1of::new(511_000.0, 0);
        let samples = wf.modulate(&[0xFF]);
        // 8 bits × 10 code periods × 511 chips × 1 sample/chip
        assert_eq!(samples.len(), 8 * 10 * 511);
    }
}
