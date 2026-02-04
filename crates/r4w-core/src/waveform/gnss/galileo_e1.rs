//! Galileo E1 Waveform
//!
//! Galileo E1 uses CBOC(6,1,1/11) modulation with 4092-chip memory codes.
//! The signal has two channels:
//! - E1B (data): carries I/NAV navigation message at 250 bps
//! - E1C (pilot): data-free for improved tracking
//!
//! The 4× longer code compared to GPS C/A provides better cross-correlation.

use crate::types::IQSample;
use crate::waveform::{
    CommonParams, DemodResult, ModulationStage, Waveform, WaveformInfo,
};
use rustfft::num_complex::Complex64;

use super::boc::CbocGenerator;
use super::prn::GalileoE1CodeGenerator;
use crate::spreading::PnSequence;

/// Galileo E1 waveform
// trace:FR-036 | ai:claude
#[derive(Debug)]
pub struct GalileoE1 {
    common: CommonParams,
    prn: u8,
    /// E1B (data) code
    code_b: Vec<i8>,
    /// E1C (pilot) code
    code_c: Vec<i8>,
    /// CBOC generator for data channel
    cboc_b: CbocGenerator,
    /// CBOC generator for pilot channel
    cboc_c: CbocGenerator,
    samples_per_chip: f64,
}

impl GalileoE1 {
    /// Galileo E1 chipping rate
    pub const CHIPPING_RATE: f64 = 1_023_000.0;
    /// Code length
    pub const CODE_LENGTH: usize = 4092;
    /// Navigation data rate (I/NAV)
    pub const NAV_DATA_RATE: f64 = 250.0;
    /// Code periods per nav symbol
    pub const CODE_PERIODS_PER_SYMBOL: usize = 4;
    /// Carrier frequency (same as GPS L1)
    pub const CARRIER_FREQ: f64 = 1_575_420_000.0;

    /// Create a new Galileo E1 waveform
    pub fn new(sample_rate: f64, prn: u8) -> Self {
        let mut gen_b = GalileoE1CodeGenerator::new(prn);
        let code_b = gen_b.generate_sequence();

        // Pilot uses a different code (PRN + 50 offset for simulation)
        let mut gen_c = GalileoE1CodeGenerator::new(((prn - 1) % 50) + 1);
        // Shift to make it different from data code
        let mut code_c = gen_c.generate_sequence();
        code_c.rotate_right(prn as usize * 100 % 4092);

        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0,
            amplitude: 1.0,
        };

        Self {
            common,
            prn,
            code_b,
            code_c,
            cboc_b: CbocGenerator::e1b(),
            cboc_c: CbocGenerator::e1c(),
            samples_per_chip: sample_rate / Self::CHIPPING_RATE,
        }
    }

    /// Create with default PRN 1
    pub fn default_config(sample_rate: f64) -> Self {
        Self::new(sample_rate, 1)
    }
}

impl Waveform for GalileoE1 {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "Galileo-E1",
            full_name: "Galileo E1 Open Service",
            description: "Galileo civilian signal using CBOC(6,1,1/11) with 4092-chip memory codes",
            complexity: 5,
            bits_per_symbol: 1,
            carries_data: true,
            characteristics: &[
                "CBOC(6,1,1/11) modulation (split-spectrum BOC)",
                "4092-chip memory codes (36.1 dB processing gain)",
                "1.023 Mchip/s chipping rate",
                "250 bps I/NAV data (E1B) + pilot (E1C)",
                "CDMA multiple access",
                "1575.42 MHz E1 carrier (same as GPS L1)",
            ],
            history: "Galileo is the European GNSS, with E1 signals first transmitted \
                      from IOV satellites in 2012. CBOC modulation was jointly designed \
                      with GPS L1C for interoperability while maintaining spectral separation.",
            modern_usage: "Galileo E1 provides position accuracy comparable to GPS. The pilot \
                          channel enables longer coherent integration for weak signal \
                          acquisition. Open Service achieves sub-meter accuracy with SBAS.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // Convert bytes to nav bits
        let nav_bits: Vec<i8> = if data.is_empty() {
            vec![1; 4] // One symbol
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

            for _period in 0..Self::CODE_PERIODS_PER_SYMBOL {
                for chip_idx in 0..Self::CODE_LENGTH {
                    let num_samples = ((chip_idx + 1) as f64 * self.samples_per_chip) as usize
                        - (chip_idx as f64 * self.samples_per_chip) as usize;

                    for s in 0..num_samples {
                        let phase = chip_idx as f64 + s as f64 / num_samples.max(1) as f64;

                        // E1B (data) on I-channel
                        let e1b = nav_val * self.code_b[chip_idx] as f64
                            * self.cboc_b.subcarrier(phase);

                        // E1C (pilot) on Q-channel
                        let e1c = self.code_c[chip_idx] as f64
                            * self.cboc_c.subcarrier(phase);

                        signal.push(Complex64::new(
                            e1b * self.common.amplitude / 2.0_f64.sqrt(),
                            e1c * self.common.amplitude / 2.0_f64.sqrt(),
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

        // Despread E1B (data channel on I)
        let samples_per_code = (Self::CODE_LENGTH as f64 * self.samples_per_chip) as usize;
        let mut nav_accum = 0.0;
        let mut code_count = 0;
        let mut pos = 0;

        while pos + Self::CODE_LENGTH.min(samples_per_code) <= samples.len() {
            let mut corr = 0.0;
            for (i, &sample) in samples[pos..pos + Self::CODE_LENGTH.min(samples_per_code)].iter().enumerate() {
                let chip_idx = i % Self::CODE_LENGTH;
                corr += sample.re * self.code_b[chip_idx] as f64;
            }

            nav_accum += corr;
            code_count += 1;

            if code_count >= Self::CODE_PERIODS_PER_SYMBOL {
                result.bits.push(if nav_accum >= 0.0 { 0 } else { 1 });
                nav_accum = 0.0;
                code_count = 0;
            }

            pos += samples_per_code;
        }

        result
    }

    fn samples_per_symbol(&self) -> usize {
        (Self::CODE_PERIODS_PER_SYMBOL as f64 * Self::CODE_LENGTH as f64 * self.samples_per_chip) as usize
    }

    fn get_modulation_stages(&self, data: &[u8]) -> Vec<ModulationStage> {
        vec![
            ModulationStage::new(
                "I/NAV Data",
                format!("250 bps navigation data from {} bytes", data.len()),
            ).with_input_bits(data.to_vec()),
            ModulationStage::new(
                "Memory Code Spreading",
                format!("4092-chip memory codes for PRN {}. Processing gain: {:.1} dB",
                    self.prn, 10.0 * 4092.0_f64.log10()),
            ),
            ModulationStage::new(
                "CBOC(6,1,1/11) Subcarrier",
                "Composite BOC: √(10/11)×BOC(1,1) ± √(1/11)×BOC(6,1). \
                 Split-spectrum modulation for reduced interference with GPS C/A.".to_string(),
            ),
            ModulationStage::new(
                "E1B/E1C Multiplexing",
                "Data (E1B) on I-channel, pilot (E1C) on Q-channel for \
                 improved tracking and longer coherent integration.".to_string(),
            ).with_samples(self.modulate(data)),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_galileo_e1_info() {
        let wf = GalileoE1::default_config(1_023_000.0);
        assert_eq!(wf.info().name, "Galileo-E1");
    }

    #[test]
    fn test_galileo_e1_modulate() {
        let wf = GalileoE1::new(1_023_000.0, 1);
        let samples = wf.modulate(&[0xFF]);
        // 8 bits × 4 code periods × 4092 chips × 1 sample/chip
        assert_eq!(samples.len(), 8 * 4 * 4092);
    }

    #[test]
    fn test_galileo_e1_pilot_present() {
        let wf = GalileoE1::new(1_023_000.0, 1);
        let samples = wf.modulate(&[0xAA]);
        // Q channel (pilot) should have non-zero values
        let q_energy: f64 = samples.iter().map(|s| s.im.powi(2)).sum();
        assert!(q_energy > 0.0, "Pilot channel should have energy");
    }
}
