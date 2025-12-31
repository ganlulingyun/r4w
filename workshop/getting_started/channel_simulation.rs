//! # Channel Simulation Example
//!
//! Demonstrates channel models: AWGN, Rayleigh fading, and multipath.
//!
//! Run with: cargo run --example channel_simulation

use r4w_core::waveform::{psk, CommonParams, Waveform};
use r4w_sim::channel::{Channel, ChannelConfig, ChannelModel};

fn main() {
    println!("=== Channel Simulation Example ===\n");

    let sample_rate = 48000.0;
    let symbol_rate = 2400.0;

    let params = CommonParams {
        sample_rate,
        carrier_freq: 0.0,
        amplitude: 1.0,
    };

    // Create QPSK modulator
    let qpsk = psk::PSK::new_qpsk(params, symbol_rate);

    // Test message
    let message = b"R4W Channel Test";
    println!("Original message: {:?}", String::from_utf8_lossy(message));

    // Modulate
    let clean_samples = qpsk.modulate(message);
    println!("Generated {} samples\n", clean_samples.len());

    // Test different channel models
    let test_cases = [
        ("Ideal (no noise)", ChannelConfig {
            model: ChannelModel::Ideal,
            ..Default::default()
        }),
        ("AWGN (SNR=20dB)", ChannelConfig {
            model: ChannelModel::Awgn,
            snr_db: 20.0,
            sample_rate,
            ..Default::default()
        }),
        ("AWGN (SNR=10dB)", ChannelConfig {
            model: ChannelModel::Awgn,
            snr_db: 10.0,
            sample_rate,
            ..Default::default()
        }),
        ("AWGN (SNR=5dB)", ChannelConfig {
            model: ChannelModel::Awgn,
            snr_db: 5.0,
            sample_rate,
            ..Default::default()
        }),
        ("AWGN + CFO (100Hz)", ChannelConfig {
            model: ChannelModel::AwgnWithCfo,
            snr_db: 15.0,
            cfo_hz: 100.0,
            sample_rate,
            ..Default::default()
        }),
        ("Rayleigh Fading", ChannelConfig {
            model: ChannelModel::Rayleigh,
            snr_db: 15.0,
            sample_rate,
            ..Default::default()
        }),
        ("Rician Fading (K=10)", ChannelConfig {
            model: ChannelModel::Rician,
            snr_db: 15.0,
            rician_k: 10.0,
            sample_rate,
            ..Default::default()
        }),
    ];

    println!("{:<25} {:>10} {:>12}", "Channel", "BER Est", "Match");
    println!("{}", "-".repeat(50));

    for (name, config) in test_cases {
        let mut channel = Channel::new(config);
        let noisy_samples = channel.apply(&clean_samples);

        // Demodulate
        let result = qpsk.demodulate(&noisy_samples);

        // Check if message recovered correctly
        let matches = result.bits.starts_with(message);
        let ber = result.ber_estimate.unwrap_or(0.0);

        println!(
            "{:<25} {:>9.4}% {:>12}",
            name,
            ber * 100.0,
            if matches { "OK" } else { "ERRORS" }
        );
    }

    println!("\n=== Channel Effects Explained ===");
    println!("- Ideal: No impairments, perfect recovery");
    println!("- AWGN: Thermal noise, BER increases as SNR decreases");
    println!("- CFO: Carrier frequency offset causes phase rotation");
    println!("- Rayleigh: No line-of-sight, severe fading");
    println!("- Rician: Line-of-sight + scattered paths, less severe");
}
