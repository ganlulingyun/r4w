//! # LoRa Basics Example
//!
//! Demonstrates LoRa chirp spread spectrum modulation.
//!
//! Run with: cargo run --example lora_basics

use r4w_core::params::{Bandwidth, CodingRate, SpreadingFactor};
use r4w_core::waveform::{lora::LoRa, Waveform};

fn main() {
    println!("=== LoRa Chirp Spread Spectrum ===\n");

    let sample_rate = 125_000.0; // 125 kHz bandwidth

    // Compare different spreading factors using factory methods
    println!("--- Spreading Factor 7 (Fast) ---");
    demo_lora(LoRa::sf7(sample_rate));

    println!("--- Spreading Factor 12 (Long Range) ---");
    demo_lora(LoRa::sf12(sample_rate));

    // Custom configuration
    println!("--- Custom: SF9 @ 250kHz ---");
    let custom = LoRa::new(
        sample_rate * 2.0, // 250 kHz
        SpreadingFactor::SF9,
        Bandwidth::Bw250kHz,
        CodingRate::CR4_5,
    );
    demo_lora(custom);

    println!("=== LoRa Characteristics ===");
    println!("- Uses chirp spread spectrum (CSS) modulation");
    println!("- Higher SF = longer range but slower data rate");
    println!("- SF7: ~5.5 kbps, SF12: ~0.3 kbps");
    println!("- Orthogonal spreading factors allow simultaneous transmissions");
    println!("- Very robust in noisy environments (process gain)");
}

fn demo_lora(lora: LoRa) {
    let sample_rate = lora.common_params().sample_rate;
    let info = lora.info();

    println!("Name: {}", info.full_name);
    println!("Bits per symbol: {}", info.bits_per_symbol);
    println!("Samples per symbol: {}", lora.samples_per_symbol());

    // Calculate chirp duration
    let chirp_duration_ms = lora.samples_per_symbol() as f64 / sample_rate * 1000.0;
    println!("Chirp duration: {:.2} ms", chirp_duration_ms);

    // Calculate approximate data rate
    let symbols_per_second = sample_rate / lora.samples_per_symbol() as f64;
    let bits_per_second = symbols_per_second * info.bits_per_symbol as f64;
    println!("Data rate: ~{:.0} bps", bits_per_second);

    // Modulate a short message
    let message = b"Hi";
    let samples = lora.modulate(message);
    println!(
        "Message '{}' -> {} samples",
        String::from_utf8_lossy(message),
        samples.len()
    );

    // Demodulate
    let result = lora.demodulate(&samples);
    let success = result.bits.starts_with(message);
    println!(
        "Demodulation: {}",
        if success { "SUCCESS" } else { "FAILED" }
    );

    println!();
}
