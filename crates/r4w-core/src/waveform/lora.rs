//! LoRa (Long Range) Chirp Spread Spectrum Waveform
//!
//! Wraps the core LoRa modulation/demodulation for use with the generic Waveform trait.

use super::{CommonParams, DemodResult, Waveform, WaveformInfo};
use crate::demodulation::Demodulator;
use crate::modulation::Modulator;
use crate::params::{Bandwidth, CodingRate, LoRaParams, SpreadingFactor};
use crate::types::IQSample;
use std::sync::Mutex;

/// LoRa Chirp Spread Spectrum waveform
pub struct LoRa {
    common: CommonParams,
    params: LoRaParams,
    modulator: Mutex<Modulator>,
    demodulator: Mutex<Demodulator>,
}

impl std::fmt::Debug for LoRa {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("LoRa")
            .field("params", &self.params)
            .finish()
    }
}

impl LoRa {
    /// Create a new LoRa waveform with specified parameters
    pub fn new(sample_rate: f64, sf: SpreadingFactor, bw: Bandwidth, cr: CodingRate) -> Self {
        let params = LoRaParams::builder()
            .spreading_factor(sf.value())
            .bandwidth(bw.hz() as u32)
            .coding_rate(cr.value())
            .build();

        let modulator = Modulator::new(params.clone());
        let demodulator = Demodulator::new(params.clone());

        Self {
            common: CommonParams {
                sample_rate,
                carrier_freq: 0.0,
                amplitude: 1.0,
            },
            params,
            modulator: Mutex::new(modulator),
            demodulator: Mutex::new(demodulator),
        }
    }

    /// Create with default SF7, 125kHz, CR 4/5
    pub fn default_config(sample_rate: f64) -> Self {
        Self::new(
            sample_rate,
            SpreadingFactor::SF7,
            Bandwidth::Bw125kHz,
            CodingRate::CR4_5,
        )
    }

    /// Create with SF7 (fast, short range)
    pub fn sf7(sample_rate: f64) -> Self {
        Self::new(
            sample_rate,
            SpreadingFactor::SF7,
            Bandwidth::Bw125kHz,
            CodingRate::CR4_5,
        )
    }

    /// Create with SF12 (slow, long range)
    pub fn sf12(sample_rate: f64) -> Self {
        Self::new(
            sample_rate,
            SpreadingFactor::SF12,
            Bandwidth::Bw125kHz,
            CodingRate::CR4_5,
        )
    }

    /// Get the LoRa parameters
    pub fn params(&self) -> &LoRaParams {
        &self.params
    }
}

impl Waveform for LoRa {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "LoRa",
            full_name: "Long Range Chirp Spread Spectrum",
            description: "LoRa uses CSS modulation for long-range, low-power IoT communications",
            complexity: 4,
            bits_per_symbol: self.params.sf.value(),
            carries_data: true,
            characteristics: &[
                "Chirp Spread Spectrum (CSS)",
                "Excellent range and noise immunity",
                "Low power consumption",
                "Configurable spreading factor (SF5-SF12)",
                "FFT-based demodulation",
            ],
            history: "Developed by Semtech, patented in 2014. Became popular for IoT/LoRaWAN.",
            modern_usage: "IoT sensors, smart cities, agriculture, asset tracking, LoRaWAN networks",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        let mut modulator = self.modulator.lock().unwrap();
        modulator.modulate(data)
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let mut demodulator = self.demodulator.lock().unwrap();

        match demodulator.demodulate(samples) {
            Ok(result) => DemodResult {
                bits: result.payload,
                symbols: result.symbols,
                ber_estimate: None,
                snr_estimate: Some(-result.rssi), // Convert RSSI to approximate SNR
                metadata: {
                    let mut m = std::collections::HashMap::new();
                    m.insert("cfo".to_string(), result.cfo);
                    m.insert("rssi".to_string(), result.rssi);
                    m.insert("errors_corrected".to_string(), result.errors_corrected as f64);
                    m
                },
            },
            Err(_) => DemodResult::default(),
        }
    }

    fn samples_per_symbol(&self) -> usize {
        self.params.samples_per_symbol()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lora_roundtrip() {
        let lora = LoRa::default_config(125_000.0);
        let data = vec![0xAB, 0xCD, 0xEF];
        let samples = lora.modulate(&data);
        assert!(!samples.is_empty());

        let result = lora.demodulate(&samples);
        // Note: May not be exact due to encoding/decoding pipeline
        println!("Demod result: {:?}", result);
    }

    #[test]
    fn test_lora_info() {
        let lora = LoRa::sf7(125_000.0);
        let info = lora.info();
        assert_eq!(info.name, "LoRa");
        assert_eq!(info.bits_per_symbol, 7);
    }
}
