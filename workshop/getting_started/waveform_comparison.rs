//! # Waveform Comparison Example
//!
//! Compares different modulation schemes: PSK, QAM, FSK.
//!
//! Run with: cargo run --example waveform_comparison

use r4w_core::waveform::{fsk, psk, qam, CommonParams, Waveform};
use r4w_sim::channel::{Channel, ChannelConfig, ChannelModel};

fn main() {
    println!("=== Waveform Comparison ===\n");

    let sample_rate = 48000.0;
    let symbol_rate = 2400.0;

    let params = CommonParams {
        sample_rate,
        carrier_freq: 0.0,
        amplitude: 1.0,
    };

    // Create different waveforms
    let waveforms: Vec<(&str, Box<dyn Waveform>)> = vec![
        ("BPSK", Box::new(psk::PSK::new_bpsk(params.clone(), symbol_rate))),
        ("QPSK", Box::new(psk::PSK::new_qpsk(params.clone(), symbol_rate))),
        ("8-PSK", Box::new(psk::PSK::new_8psk(params.clone(), symbol_rate))),
        ("16-QAM", Box::new(qam::QAM::new_16qam(params.clone(), symbol_rate))),
        ("BFSK", Box::new(fsk::FSK::new_bfsk(params.clone(), symbol_rate / 2.0, 500.0))),
    ];

    // Test message
    let message = b"Test1234";
    let message_bits = message.len() * 8;

    println!("Test message: {:?} ({} bits)\n", String::from_utf8_lossy(message), message_bits);

    // Print header
    println!("{:<10} {:>6} {:>8} {:>10} {:>12}",
             "Waveform", "b/sym", "Samples", "Bandwidth", "Efficiency");
    println!("{}", "-".repeat(50));

    for (name, waveform) in &waveforms {
        let info = waveform.info();
        let samples = waveform.modulate(message);
        let samples_per_symbol = waveform.samples_per_symbol();

        // Estimate bandwidth (simplified)
        let bandwidth_hz = sample_rate / samples_per_symbol as f64;

        // Spectral efficiency (bits/s/Hz)
        let efficiency = info.bits_per_symbol as f64 * symbol_rate / bandwidth_hz;

        println!(
            "{:<10} {:>6} {:>8} {:>9.0}Hz {:>10.2} b/s/Hz",
            name,
            info.bits_per_symbol,
            samples.len(),
            bandwidth_hz,
            efficiency
        );
    }

    // BER comparison under AWGN
    println!("\n=== BER vs SNR Comparison ===\n");

    let snr_values = [20.0, 15.0, 10.0, 5.0];

    // Header
    print!("{:<10}", "Waveform");
    for snr in &snr_values {
        print!(" {:>8}dB", snr);
    }
    println!();
    println!("{}", "-".repeat(50));

    for (name, waveform) in &waveforms {
        print!("{:<10}", name);

        let clean_samples = waveform.modulate(message);

        for snr in &snr_values {
            let config = ChannelConfig {
                model: ChannelModel::Awgn,
                snr_db: *snr,
                sample_rate,
                ..Default::default()
            };

            let mut channel = Channel::new(config);
            let noisy_samples = channel.apply(&clean_samples);
            let result = waveform.demodulate(&noisy_samples);

            // Count bit errors
            let errors = result.bits.iter()
                .zip(message.iter())
                .map(|(a, b)| (*a ^ *b).count_ones())
                .sum::<u32>();

            let ber = errors as f64 / message_bits as f64;
            print!(" {:>8.1}%", ber * 100.0);
        }
        println!();
    }

    println!("\n=== Waveform Trade-offs ===");
    println!("- BPSK: Most robust, lowest data rate (1 bit/symbol)");
    println!("- QPSK: Good balance of robustness and throughput (2 bits/symbol)");
    println!("- 8-PSK: Higher throughput, more sensitive to noise (3 bits/symbol)");
    println!("- 16-QAM: High throughput, requires good SNR (4 bits/symbol)");
    println!("- FSK: Constant envelope, good for non-linear amplifiers");
}
