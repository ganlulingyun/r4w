//! GPS L5 Waveform
//!
//! GPS L5 is a modernized civilian signal on 1176.45 MHz with:
//! - QPSK modulation (I5 data + Q5 pilot)
//! - 10.23 Mchip/s chipping rate (10× GPS L1 C/A)
//! - 10230-chip codes from 13-stage LFSRs
//! - Neumann-Hoffman secondary codes for synchronization
//!
//! Educational value: demonstrates wider bandwidth benefits and
//! dual-frequency ionospheric correction when combined with L1.

use crate::types::IQSample;
use crate::waveform::{
    CommonParams, DemodResult, ModulationStage, Waveform, WaveformInfo,
};
use rustfft::num_complex::Complex64;

use super::prn::GpsL5CodeGenerator;
use crate::spreading::PnSequence;

/// GPS L5 waveform
// trace:FR-036 | ai:claude
#[derive(Debug)]
pub struct GpsL5 {
    common: CommonParams,
    prn: u8,
    /// I5 (data) channel code
    code_i: Vec<i8>,
    /// Q5 (pilot) channel code
    code_q: Vec<i8>,
    samples_per_chip: f64,
}

impl GpsL5 {
    /// L5 chipping rate
    pub const CHIPPING_RATE: f64 = 10_230_000.0;
    /// Code length
    pub const CODE_LENGTH: usize = 10230;
    /// Navigation data rate
    pub const NAV_DATA_RATE: f64 = 50.0;
    /// Code periods per nav bit (I5)
    pub const CODE_PERIODS_PER_BIT: usize = 10;
    /// Carrier frequency
    pub const CARRIER_FREQ: f64 = 1_176_450_000.0;

    /// Create a new GPS L5 waveform
    pub fn new(sample_rate: f64, prn: u8) -> Self {
        let mut gen_i = GpsL5CodeGenerator::new_i5(prn);
        let code_i = gen_i.generate_sequence();
        let mut gen_q = GpsL5CodeGenerator::new_q5(prn);
        let code_q = gen_q.generate_sequence();

        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };

        Self {
            common,
            prn,
            code_i,
            code_q,
            samples_per_chip: sample_rate / Self::CHIPPING_RATE,
        }
    }

    /// Create with default PRN 1
    pub fn default_config(sample_rate: f64) -> Self {
        Self::new(sample_rate, 1)
    }
}

impl Waveform for GpsL5 {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "GPS-L5",
            full_name: "GPS L5 Safety of Life",
            description: "GPS modernized signal with QPSK, 10.23 Mchip/s, 10230-chip codes",
            complexity: 5,
            bits_per_symbol: 1,
            carries_data: true,
            characteristics: &[
                "QPSK modulation (I5 data + Q5 pilot)",
                "10230-chip codes (40.1 dB processing gain)",
                "10.23 Mchip/s chipping rate (10× L1 C/A)",
                "50 bps navigation data + Neumann-Hoffman secondary codes",
                "CDMA multiple access",
                "1176.45 MHz L5 carrier (ARNS band)",
            ],
            history: "GPS L5 was first demonstrated on the Block IIR-M satellite in 2005. \
                      Designed for safety-of-life applications requiring dual-frequency \
                      ionospheric correction. The wider bandwidth provides better multipath \
                      rejection and faster acquisition.",
            modern_usage: "Available on all GPS III satellites. Provides 10× better ranging \
                          precision than L1 C/A. Combined L1+L5 enables real-time ionospheric \
                          correction for sub-meter accuracy without augmentation.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        let nav_bits: Vec<i8> = if data.is_empty() {
            vec![1; 10]
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

        for (bit_idx, &nav_bit) in nav_bits.iter().enumerate() {
            let nav_val = nav_bit as f64;

            // Apply Neumann-Hoffman secondary code to I5
            let nh_i = GpsL5CodeGenerator::NH_I5[bit_idx % 10] as f64;

            for _period in 0..Self::CODE_PERIODS_PER_BIT {
                for chip_idx in 0..Self::CODE_LENGTH {
                    let num_samples = ((chip_idx + 1) as f64 * self.samples_per_chip) as usize
                        - (chip_idx as f64 * self.samples_per_chip) as usize;

                    let i5_val = nav_val * nh_i * self.code_i[chip_idx] as f64;
                    let q5_val = self.code_q[chip_idx] as f64; // Pilot (no data)

                    for _ in 0..num_samples {
                        signal.push(Complex64::new(
                            i5_val * self.common.amplitude / 2.0_f64.sqrt(),
                            q5_val * self.common.amplitude / 2.0_f64.sqrt(),
                        ));
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

        let samples_per_code = (Self::CODE_LENGTH as f64 * self.samples_per_chip) as usize;
        let mut nav_accum = 0.0;
        let mut code_count = 0;
        let mut pos = 0;

        while pos + Self::CODE_LENGTH.min(samples_per_code) <= samples.len() {
            let mut corr = 0.0;
            for (i, &sample) in samples[pos..pos + Self::CODE_LENGTH.min(samples_per_code)].iter().enumerate() {
                let chip_idx = i % Self::CODE_LENGTH;
                corr += sample.re * self.code_i[chip_idx] as f64;
            }

            nav_accum += corr;
            code_count += 1;

            if code_count >= Self::CODE_PERIODS_PER_BIT {
                result.bits.push(if nav_accum >= 0.0 { 0 } else { 1 });
                nav_accum = 0.0;
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
                "Navigation Data + NH Code",
                format!("50 bps nav data with 10-bit Neumann-Hoffman secondary code on I5. \
                         {} bytes input.", data.len()),
            ).with_input_bits(data.to_vec()),
            ModulationStage::new(
                "10230-Chip Code Spreading",
                format!("13-stage LFSR codes for PRN {}. Processing gain: {:.1} dB. \
                         10× longer than GPS L1 C/A.",
                    self.prn, 10.0 * 10230.0_f64.log10()),
            ),
            ModulationStage::new(
                "QPSK Modulation",
                "I5 (data) and Q5 (pilot) on orthogonal carriers. \
                 10.23 Mchip/s = 24 MHz null-to-null bandwidth.".to_string(),
            ).with_samples(self.modulate(data)),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gps_l5_info() {
        let wf = GpsL5::default_config(10_230_000.0);
        assert_eq!(wf.info().name, "GPS-L5");
    }

    #[test]
    fn test_gps_l5_modulate() {
        let wf = GpsL5::new(10_230_000.0, 1);
        let samples = wf.modulate(&[0xFF]);
        // 8 bits × 10 code periods × 10230 chips × 1 sample/chip
        assert_eq!(samples.len(), 8 * 10 * 10230);
    }

    #[test]
    fn test_gps_l5_qpsk() {
        let wf = GpsL5::new(10_230_000.0, 1);
        let samples = wf.modulate(&[0xAA]);
        // Both I and Q channels should have energy
        let i_energy: f64 = samples.iter().map(|s| s.re.powi(2)).sum();
        let q_energy: f64 = samples.iter().map(|s| s.im.powi(2)).sum();
        assert!(i_energy > 0.0, "I5 channel should have energy");
        assert!(q_energy > 0.0, "Q5 channel should have energy");
    }
}
