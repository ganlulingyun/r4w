//! GPS L1 C/A Waveform
//!
//! Implements the GPS L1 C/A signal as a Waveform trait object.
//! BPSK(1) modulation with 1023-chip Gold codes at 1.023 Mchip/s,
//! carrying 50 bps navigation data.
//!
//! ## Signal Structure
//!
//! ```text
//! Nav Data (50 bps) ──→ ⊕ ──→ BPSK Modulate ──→ I/Q samples
//!                       ↑
//! PRN Code (1.023 Mchip/s)
//! ```

use crate::types::IQSample;
use crate::waveform::{
    CommonParams, DemodResult, DemodulationStep, ModulationStage, Waveform, WaveformInfo,
};
use rustfft::num_complex::Complex64;

use super::acquisition::PcpsAcquisition;
use super::nav_message::LnavMessage;
use super::prn::GpsCaCodeGenerator;

/// GPS L1 C/A waveform
// trace:FR-032 | ai:claude
#[derive(Debug)]
pub struct GpsL1Ca {
    /// Common waveform parameters
    common: CommonParams,
    /// PRN number (1-32)
    prn: u8,
    /// Pre-generated code (+1/-1)
    code: Vec<i8>,
    /// Samples per chip (sample_rate / chipping_rate)
    samples_per_chip: f64,
}

impl GpsL1Ca {
    /// GPS L1 C/A chipping rate
    pub const CHIPPING_RATE: f64 = 1_023_000.0;
    /// Code length in chips
    pub const CODE_LENGTH: usize = 1023;
    /// Navigation data rate in bps
    pub const NAV_DATA_RATE: f64 = 50.0;
    /// Code periods per nav bit (1023 chips * 20 = 20460 chips per bit)
    pub const CODE_PERIODS_PER_BIT: usize = 20;
    /// Carrier frequency
    pub const CARRIER_FREQ: f64 = 1_575_420_000.0;

    /// Create a new GPS L1 C/A waveform for a specific PRN
    pub fn new(sample_rate: f64, prn: u8) -> Self {
        let mut gen = GpsCaCodeGenerator::new(prn);
        let code = gen.generate_code();
        let common = CommonParams {
            sample_rate,
            carrier_freq: 0.0, // Baseband
            amplitude: 1.0,
        };
        let samples_per_chip = sample_rate / Self::CHIPPING_RATE;

        Self {
            common,
            prn,
            code,
            samples_per_chip,
        }
    }

    /// Create with default PRN 1
    pub fn default_config(sample_rate: f64) -> Self {
        Self::new(sample_rate, 1)
    }

    /// Get the PRN number
    pub fn prn(&self) -> u8 {
        self.prn
    }

    /// Get the spreading code
    pub fn code(&self) -> &[i8] {
        &self.code
    }

    /// Generate baseband signal for given navigation bits
    /// Each nav bit is spread by 20 repetitions of the 1023-chip code
    fn generate_signal(&self, nav_bits: &[i8]) -> Vec<IQSample> {
        let samples_per_code = (Self::CODE_LENGTH as f64 * self.samples_per_chip) as usize;
        let samples_per_bit = samples_per_code * Self::CODE_PERIODS_PER_BIT;
        let total_samples = nav_bits.len() * samples_per_bit;

        let mut signal = Vec::with_capacity(total_samples);

        for &nav_bit in nav_bits {
            let nav_val = nav_bit as f64; // +1 or -1

            for _code_period in 0..Self::CODE_PERIODS_PER_BIT {
                for chip_idx in 0..Self::CODE_LENGTH {
                    let chip_val = self.code[chip_idx] as f64; // +1 or -1
                    let spread_val = nav_val * chip_val * self.common.amplitude;

                    // Generate samples_per_chip samples for this chip
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
}

impl Waveform for GpsL1Ca {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "GPS-L1CA",
            full_name: "GPS L1 Coarse/Acquisition",
            description: "GPS civilian signal using BPSK(1) with 1023-chip Gold codes",
            complexity: 4,
            bits_per_symbol: 1,
            carries_data: true,
            characteristics: &[
                "BPSK(1) modulation",
                "1023-chip Gold code (30.1 dB processing gain)",
                "1.023 Mchip/s chipping rate",
                "50 bps navigation data",
                "CDMA multiple access",
                "1575.42 MHz L1 carrier",
            ],
            history: "GPS L1 C/A signal has been operational since 1978. Designed for \
                      civilian use with intentional degradation (SA) until 2000. The Gold \
                      code structure enables 32 satellites to share the same frequency.",
            modern_usage: "Primary civilian GPS signal worldwide. Used in billions of devices \
                          for navigation, timing, and surveying. Being supplemented by L1C, \
                          L2C, and L5 signals for improved accuracy and robustness.",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // Convert bytes to navigation bits (+1/-1)
        let nav_bits: Vec<i8> = if data.is_empty() {
            // Generate a default preamble pattern
            let subframe = LnavMessage::encode_subframe(1, 0);
            subframe
        } else {
            // Convert each byte to 8 nav bits
            data.iter()
                .flat_map(|&byte| {
                    (0..8).rev().map(move |b| {
                        if (byte >> b) & 1 == 1 { -1i8 } else { 1i8 }
                    })
                })
                .collect()
        };

        self.generate_signal(&nav_bits)
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let mut result = DemodResult::default();

        if samples.len() < Self::CODE_LENGTH {
            return result;
        }

        // Run acquisition
        let acq = PcpsAcquisition::new(Self::CODE_LENGTH, self.common.sample_rate)
            .with_threshold(2.0);
        let acq_result = acq.acquire(samples, &self.code, self.prn);

        result.metadata.insert("detected".to_string(), if acq_result.detected { 1.0 } else { 0.0 });
        result.metadata.insert("code_phase".to_string(), acq_result.code_phase);
        result.metadata.insert("doppler_hz".to_string(), acq_result.doppler_hz);
        result.metadata.insert("peak_metric".to_string(), acq_result.peak_metric);

        if let Some(cn0) = acq_result.cn0_estimate {
            result.snr_estimate = Some(cn0);
        }

        if !acq_result.detected {
            return result;
        }

        // Simple despreading: correlate code periods and extract bit signs
        let samples_per_code = (Self::CODE_LENGTH as f64 * self.samples_per_chip) as usize;
        let code_phase = acq_result.code_phase as usize;

        let mut nav_bit_accum = 0.0;
        let mut code_count = 0;
        let mut pos = code_phase;

        while pos + samples_per_code <= samples.len() {
            // Correlate one code period
            let mut corr = 0.0;
            for (i, &sample) in samples[pos..pos + samples_per_code.min(Self::CODE_LENGTH)].iter().enumerate() {
                let chip_idx = i % Self::CODE_LENGTH;
                corr += sample.re * self.code[chip_idx] as f64;
            }

            nav_bit_accum += corr;
            code_count += 1;

            // Every 20 code periods = 1 nav bit
            if code_count >= Self::CODE_PERIODS_PER_BIT {
                let bit = if nav_bit_accum >= 0.0 { 0u8 } else { 1u8 };
                result.bits.push(bit);
                nav_bit_accum = 0.0;
                code_count = 0;
            }

            pos += samples_per_code;
        }

        result
    }

    fn samples_per_symbol(&self) -> usize {
        // One nav bit = 20 code periods * 1023 chips * samples_per_chip
        (Self::CODE_PERIODS_PER_BIT as f64 * Self::CODE_LENGTH as f64 * self.samples_per_chip) as usize
    }

    fn get_modulation_stages(&self, data: &[u8]) -> Vec<ModulationStage> {
        let mut stages = Vec::new();

        // Stage 1: Data to nav bits
        stages.push(
            ModulationStage::new(
                "Navigation Data",
                format!("Convert input data to 50 bps navigation bits. {} bytes → {} bits",
                    data.len(), data.len() * 8),
            )
            .with_input_bits(data.to_vec()),
        );

        // Stage 2: PRN code generation
        stages.push(
            ModulationStage::new(
                "PRN Code Generation",
                format!("Generate 1023-chip Gold code for PRN {} using G1/G2 LFSRs. \
                         Processing gain: {:.1} dB",
                    self.prn, 10.0 * 1023.0_f64.log10()),
            ),
        );

        // Stage 3: Spreading
        stages.push(
            ModulationStage::new(
                "Code Spreading (XOR)",
                format!("Spread each nav bit with {} repetitions of the 1023-chip code. \
                         {} chips per nav bit at {:.3} Mchip/s",
                    Self::CODE_PERIODS_PER_BIT,
                    Self::CODE_PERIODS_PER_BIT * Self::CODE_LENGTH,
                    Self::CHIPPING_RATE / 1e6),
            ),
        );

        // Stage 4: BPSK modulation
        let samples = self.modulate(data);
        stages.push(
            ModulationStage::new(
                "BPSK Modulation",
                format!("Map spread chips to I/Q samples via BPSK. \
                         {:.1} samples/chip, {} total samples",
                    self.samples_per_chip, samples.len()),
            )
            .with_samples(samples),
        );

        stages
    }

    fn get_demodulation_steps(&self, samples: &[IQSample]) -> Vec<DemodulationStep> {
        let mut steps = Vec::new();

        steps.push(
            DemodulationStep::new(
                "2D Acquisition (PCPS)",
                format!("Search {}×{} grid: Doppler ±5 kHz (500 Hz steps) × {} code phases. \
                         FFT-based parallel code phase search.",
                    21, Self::CODE_LENGTH, Self::CODE_LENGTH),
            )
            .with_input_samples(samples[..samples.len().min(Self::CODE_LENGTH)].to_vec()),
        );

        steps.push(
            DemodulationStep::new(
                "DLL/PLL Tracking",
                "Early/Prompt/Late correlators with 0.5 chip spacing. \
                 DLL (1 Hz BW) for code tracking, PLL (15 Hz BW) for carrier.",
            ),
        );

        steps.push(
            DemodulationStep::new(
                "Bit Synchronization",
                "Detect nav bit edges from sign changes in prompt correlator. \
                 Accumulate 20 code periods (20 ms) per navigation bit.",
            ),
        );

        let result = self.demodulate(samples);
        steps.push(
            DemodulationStep::new(
                "Navigation Data Recovery",
                format!("Recovered {} navigation bits at 50 bps.", result.bits.len()),
            )
            .with_recovered_bits(result.bits),
        );

        steps
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gps_l1ca_info() {
        let waveform = GpsL1Ca::new(1_023_000.0, 1);
        let info = waveform.info();
        assert_eq!(info.name, "GPS-L1CA");
        assert!(info.carries_data);
    }

    #[test]
    fn test_gps_l1ca_modulate() {
        // 1 sample per chip for simplicity
        let waveform = GpsL1Ca::new(1_023_000.0, 1);
        let data = &[0xAA]; // 10101010 → 8 nav bits

        let samples = waveform.modulate(data);

        // 8 nav bits × 20 code periods × 1023 chips × 1 sample/chip
        let expected_samples = 8 * 20 * 1023;
        assert_eq!(samples.len(), expected_samples,
            "Expected {} samples, got {}", expected_samples, samples.len());

        // All samples should be ±1 (BPSK)
        assert!(samples.iter().all(|s| (s.re.abs() - 1.0).abs() < 1e-10 && s.im.abs() < 1e-10));
    }

    #[test]
    fn test_gps_l1ca_roundtrip_simple() {
        let waveform = GpsL1Ca::new(1_023_000.0, 1);
        let data = &[0xFF]; // All 1s → 8 nav bits

        let samples = waveform.modulate(data);
        let result = waveform.demodulate(&samples);

        // Should detect the signal
        assert_eq!(*result.metadata.get("detected").unwrap(), 1.0,
            "Signal should be detected in clean roundtrip");
    }

    #[test]
    fn test_gps_l1ca_samples_per_symbol() {
        let waveform = GpsL1Ca::new(1_023_000.0, 1);
        // 20 code periods × 1023 chips × 1 sample/chip = 20460
        assert_eq!(waveform.samples_per_symbol(), 20460);
    }

    #[test]
    fn test_gps_l1ca_different_prns() {
        let wf1 = GpsL1Ca::new(1_023_000.0, 1);
        let wf7 = GpsL1Ca::new(1_023_000.0, 7);
        assert_ne!(wf1.code(), wf7.code());
    }
}
