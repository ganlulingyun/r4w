//! # Exercise 80: Implementing the Waveform Trait
//!
//! Learn how to create custom waveforms that integrate with R4W.
//!
//! ## Topics
//! - The Waveform trait interface
//! - Modulator implementation
//! - Demodulator implementation
//! - Educational visualization hooks
//!
//! ## Run
//! ```bash
//! cargo run --example 80_waveform_trait
//! ```
//!
//! trace:FR-0095 | ai:claude

use r4w_core::waveform::{CommonParams, DemodResult, Waveform, WaveformInfo};
use r4w_core::IQSample;
use std::collections::HashMap;
use std::f64::consts::PI;

/// A simple On-Off Keying (OOK) waveform implementation
///
/// OOK is the simplest form of amplitude modulation:
/// - Bit 1: Carrier present
/// - Bit 0: Carrier absent
#[derive(Debug)]
struct OokWaveform {
    samples_per_bit: usize,
    carrier_freq: f64,
    common: CommonParams,
}

impl OokWaveform {
    fn new(bit_rate: f64, sample_rate: f64) -> Self {
        let samples_per_bit = (sample_rate / bit_rate) as usize;
        // Carrier at 1/4 of sample rate for good representation
        let carrier_freq = sample_rate / 4.0;

        OokWaveform {
            samples_per_bit,
            carrier_freq,
            common: CommonParams {
                sample_rate,
                carrier_freq,
                amplitude: 1.0,
            },
        }
    }

    #[allow(dead_code)]
    fn bit_rate(&self) -> f64 {
        self.common.sample_rate / self.samples_per_bit as f64
    }
}

impl Waveform for OokWaveform {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "OOK",
            full_name: "On-Off Keying",
            description: "Simplest form of amplitude modulation",
            complexity: 1,
            bits_per_symbol: 1,
            carries_data: true,
            characteristics: &["Simple", "Low complexity", "Used in RFID"],
            history: "One of the earliest modulation schemes",
            modern_usage: "RFID, remote controls, simple telemetry",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn samples_per_symbol(&self) -> usize {
        self.samples_per_bit
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        // Convert bytes to bits and modulate
        let mut samples = Vec::with_capacity(data.len() * 8 * self.samples_per_bit);
        let mut sample_idx = 0usize;

        for byte in data {
            for bit_pos in (0..8).rev() {
                let bit = (byte >> bit_pos) & 1 == 1;

                for _ in 0..self.samples_per_bit {
                    let t = sample_idx as f64 / self.common.sample_rate;
                    let phase = 2.0 * PI * self.carrier_freq * t;

                    if bit {
                        // Carrier on
                        samples.push(IQSample::new(phase.cos(), phase.sin()));
                    } else {
                        // Carrier off
                        samples.push(IQSample::new(0.0, 0.0));
                    }
                    sample_idx += 1;
                }
            }
        }

        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        // Envelope detection: calculate power in each bit period
        let mut bytes = Vec::new();
        let mut current_byte = 0u8;
        let mut bit_count = 0;

        for chunk in samples.chunks(self.samples_per_bit) {
            if chunk.len() < self.samples_per_bit / 2 {
                continue; // Skip incomplete bits
            }

            // Calculate average power
            let power: f64 = chunk.iter()
                .map(|s| s.re * s.re + s.im * s.im)
                .sum::<f64>() / chunk.len() as f64;

            // Threshold at 0.25 (half amplitude squared)
            let bit = power > 0.25;

            current_byte = (current_byte << 1) | (bit as u8);
            bit_count += 1;

            if bit_count == 8 {
                bytes.push(current_byte);
                current_byte = 0;
                bit_count = 0;
            }
        }

        DemodResult {
            bits: bytes,
            symbols: Vec::new(),
            ber_estimate: None,
            snr_estimate: None,
            metadata: HashMap::new(),
        }
    }
}

/// A simple Frequency Shift Keying (FSK) waveform
#[derive(Debug)]
struct FskWaveform {
    samples_per_bit: usize,
    freq_low: f64,
    freq_high: f64,
    common: CommonParams,
}

impl FskWaveform {
    fn new(bit_rate: f64, sample_rate: f64, deviation: f64) -> Self {
        let samples_per_bit = (sample_rate / bit_rate) as usize;
        let center_freq = sample_rate / 8.0;

        FskWaveform {
            samples_per_bit,
            freq_low: center_freq - deviation,
            freq_high: center_freq + deviation,
            common: CommonParams {
                sample_rate,
                carrier_freq: center_freq,
                amplitude: 1.0,
            },
        }
    }
}

impl Waveform for FskWaveform {
    fn info(&self) -> WaveformInfo {
        WaveformInfo {
            name: "2-FSK",
            full_name: "Binary Frequency Shift Keying",
            description: "Frequency changes to represent bits",
            complexity: 2,
            bits_per_symbol: 1,
            carries_data: true,
            characteristics: &["Constant envelope", "Robust", "Used in pagers"],
            history: "Developed in the 1920s for radio teletype",
            modern_usage: "Bluetooth, Zigbee, low-power IoT",
        }
    }

    fn common_params(&self) -> &CommonParams {
        &self.common
    }

    fn samples_per_symbol(&self) -> usize {
        self.samples_per_bit
    }

    fn modulate(&self, data: &[u8]) -> Vec<IQSample> {
        let mut samples = Vec::with_capacity(data.len() * 8 * self.samples_per_bit);
        let mut phase = 0.0f64;

        for byte in data {
            for bit_pos in (0..8).rev() {
                let bit = (byte >> bit_pos) & 1 == 1;
                let freq = if bit { self.freq_high } else { self.freq_low };
                let phase_inc = 2.0 * PI * freq / self.common.sample_rate;

                for _ in 0..self.samples_per_bit {
                    samples.push(IQSample::new(phase.cos(), phase.sin()));
                    phase += phase_inc;
                    // Keep phase bounded
                    if phase > 2.0 * PI {
                        phase -= 2.0 * PI;
                    }
                }
            }
        }

        samples
    }

    fn demodulate(&self, samples: &[IQSample]) -> DemodResult {
        let mut bytes = Vec::new();
        let mut current_byte = 0u8;
        let mut bit_count = 0;

        for chunk in samples.chunks(self.samples_per_bit) {
            if chunk.len() < self.samples_per_bit / 2 {
                continue;
            }

            // Correlate with both frequencies
            let power_low = correlate_freq(chunk, self.freq_low, self.common.sample_rate);
            let power_high = correlate_freq(chunk, self.freq_high, self.common.sample_rate);

            let bit = power_high > power_low;
            current_byte = (current_byte << 1) | (bit as u8);
            bit_count += 1;

            if bit_count == 8 {
                bytes.push(current_byte);
                current_byte = 0;
                bit_count = 0;
            }
        }

        DemodResult {
            bits: bytes,
            symbols: Vec::new(),
            ber_estimate: None,
            snr_estimate: None,
            metadata: HashMap::new(),
        }
    }
}

fn correlate_freq(samples: &[IQSample], freq: f64, sample_rate: f64) -> f64 {
    let phase_inc = 2.0 * PI * freq / sample_rate;
    let mut sum_re = 0.0f64;
    let mut sum_im = 0.0f64;

    for (i, s) in samples.iter().enumerate() {
        let phase = phase_inc * i as f64;
        // Multiply by conjugate of reference
        sum_re += s.re * phase.cos() + s.im * phase.sin();
        sum_im += s.im * phase.cos() - s.re * phase.sin();
    }

    (sum_re * sum_re + sum_im * sum_im).sqrt()
}

fn main() {
    println!("=== Waveform Trait Implementation Workshop ===\n");

    // Test message
    let message = "Hi";
    let data = message.as_bytes();

    println!("Test message: \"{}\"", message);
    println!("Bytes: {:02x?}\n", data);

    // Test OOK
    println!("=== Testing OOK Waveform ===\n");
    let ook = OokWaveform::new(1000.0, 8000.0);
    test_waveform(&ook, data);

    // Test FSK
    println!("\n=== Testing FSK Waveform ===\n");
    let fsk = FskWaveform::new(1000.0, 8000.0, 500.0);
    test_waveform(&fsk, data);

    println!("\n=== The Waveform Trait ===\n");
    print_waveform_trait_info();

    println!("\n=== Workshop Complete ===");
}

fn test_waveform(waveform: &dyn Waveform, data: &[u8]) {
    let info = waveform.info();
    println!("Waveform: {} ({})", info.name, info.full_name);
    println!("Bit rate: {} bps", waveform.common_params().sample_rate / waveform.samples_per_symbol() as f64);

    // Modulate
    let samples = waveform.modulate(data);
    println!("Modulated: {} bytes → {} samples", data.len(), samples.len());
    println!("Samples per bit: {}", waveform.samples_per_symbol());

    // Demodulate
    let result = waveform.demodulate(&samples);
    println!("Demodulated: {} samples → {} bytes", samples.len(), result.bits.len());

    // Check for errors
    let errors = data.iter()
        .zip(result.bits.iter())
        .filter(|(a, b)| a != b)
        .count();

    let total_bits = data.len() * 8;
    println!("Byte errors: {} / {}", errors, data.len());
    println!("BER: {:.2e}", errors as f64 * 8.0 / total_bits as f64);

    // Verify content
    if result.bits == data {
        println!("✓ Perfect decode!");
    } else {
        println!("✗ Decode mismatch");
        println!("  Expected: {:02x?}", data);
        println!("  Got:      {:02x?}", result.bits);
    }

    // Visualize first few samples
    println!("\nFirst 32 samples (amplitude):");
    for (i, s) in samples.iter().take(32).enumerate() {
        let amp = (s.re * s.re + s.im * s.im).sqrt();
        if i % 8 == 0 {
            print!("{:3}: ", i);
        }
        print!("{}", if amp > 0.5 { "█" } else { "░" });
        if i % 8 == 7 {
            println!();
        }
    }
    println!();
}

fn print_waveform_trait_info() {
    println!("The Waveform trait defines the interface for all modulations:");
    println!();
    println!("pub trait Waveform: Debug + Send + Sync {{");
    println!("    /// Get waveform metadata");
    println!("    fn info(&self) -> WaveformInfo;");
    println!();
    println!("    /// Get common parameters (sample rate, carrier freq, amplitude)");
    println!("    fn common_params(&self) -> &CommonParams;");
    println!();
    println!("    /// Convert bytes to IQ samples");
    println!("    fn modulate(&self, data: &[u8]) -> Vec<IQSample>;");
    println!();
    println!("    /// Convert IQ samples back to bytes");
    println!("    fn demodulate(&self, samples: &[IQSample]) -> DemodResult;");
    println!();
    println!("    /// Number of samples per symbol");
    println!("    fn samples_per_symbol(&self) -> usize;");
    println!("}}");
    println!();
    println!("Optional methods with default implementations:");
    println!("  get_visualization()     - Get visualization data for GUI");
    println!("  generate_demo()         - Generate demo signal");
    println!("  get_modulation_stages() - Step-by-step breakdown");
}
