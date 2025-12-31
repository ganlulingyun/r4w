//! # Basic Modulation Example
//!
//! Demonstrates basic PSK modulation and demodulation.
//!
//! Run with: cargo run --example basic_modulation

use r4w_core::waveform::{psk, CommonParams, Waveform};

fn main() {
    // Configure waveform parameters
    let sample_rate = 48000.0; // 48 kHz
    let symbol_rate = 4800.0; // 4800 baud

    let params = CommonParams {
        sample_rate,
        carrier_freq: 0.0, // Baseband
        amplitude: 1.0,
    };

    // Create QPSK modulator
    let qpsk = psk::PSK::new_qpsk(params, symbol_rate);

    println!("=== QPSK Modulation Example ===");
    println!("Sample rate: {} Hz", sample_rate);
    println!("Symbol rate: {} baud", symbol_rate);
    println!("Bits per symbol: {}", qpsk.info().bits_per_symbol);
    println!();

    // Data to transmit
    let message = b"Hello, R4W!";
    println!("Message: {:?}", String::from_utf8_lossy(message));
    println!("Message bytes: {:02X?}", message);
    println!();

    // Modulate
    let samples = qpsk.modulate(message);
    println!("Generated {} I/Q samples", samples.len());
    println!("Duration: {:.3} ms", samples.len() as f64 / sample_rate * 1000.0);
    println!();

    // Show first few samples
    println!("First 5 samples:");
    for (i, sample) in samples.iter().take(5).enumerate() {
        println!("  [{}] I={:.4}, Q={:.4}", i, sample.re, sample.im);
    }
    println!();

    // Demodulate (no noise - perfect channel)
    let result = qpsk.demodulate(&samples);
    println!("Demodulated {} bytes", result.bits.len());

    // Compare
    if result.bits.starts_with(message) {
        println!("SUCCESS: Message recovered correctly!");
    } else {
        println!("Result: {:02X?}", &result.bits[..result.bits.len().min(message.len())]);
    }
}
