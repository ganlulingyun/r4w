//! # Exercise 20: Build a PSK Modem from Scratch
//!
//! Implement a complete PSK modulator/demodulator to understand every step.
//!
//! ## Topics
//! - Constellation mapping
//! - Pulse shaping
//! - Symbol timing recovery
//! - Carrier phase recovery
//! - Decision making
//!
//! ## Run
//! ```bash
//! cargo run --example 20_psk_modem
//! ```
//!
//! trace:FR-0095 | ai:claude

use r4w_core::IQSample;
use std::f64::consts::PI;

/// PSK modulation order
#[derive(Debug, Clone, Copy)]
enum PskOrder {
    Bpsk,   // 1 bit/symbol
    Qpsk,   // 2 bits/symbol
    Psk8,   // 3 bits/symbol
}

fn main() {
    println!("=== PSK Modem Workshop ===\n");

    // Test data
    let message = "Hello!";
    let bits: Vec<bool> = message.bytes()
        .flat_map(|b| (0..8).rev().map(move |i| (b >> i) & 1 == 1))
        .collect();

    println!("Message: \"{}\"", message);
    println!("Bits: {} total\n", bits.len());

    // Test each PSK order
    for order in [PskOrder::Bpsk, PskOrder::Qpsk, PskOrder::Psk8] {
        println!("=== {:?} ===\n", order);
        test_psk_modem(&bits, order);
        println!();
    }

    println!("=== Workshop Complete ===");
}

fn test_psk_modem(bits: &[bool], order: PskOrder) {
    let bits_per_symbol = match order {
        PskOrder::Bpsk => 1,
        PskOrder::Qpsk => 2,
        PskOrder::Psk8 => 3,
    };

    let samples_per_symbol = 8;
    let snr_db = 10.0;

    println!("Configuration:");
    println!("  Bits per symbol: {}", bits_per_symbol);
    println!("  Samples per symbol: {}", samples_per_symbol);
    println!("  SNR: {} dB", snr_db);

    // Step 1: Bit to symbol mapping
    println!("\n1. Constellation Mapping:");
    let symbols = bits_to_symbols(bits, order);
    println!("   {} bits → {} symbols", bits.len(), symbols.len());

    // Show constellation points
    let constellation = get_constellation(order);
    println!("   Constellation points:");
    for (i, p) in constellation.iter().enumerate() {
        println!("     {:0width$b} → ({:6.3}, {:6.3})",
                 i, p.re, p.im, width = bits_per_symbol);
    }

    // Step 2: Pulse shaping (simple rectangular for now)
    println!("\n2. Pulse Shaping:");
    let tx_samples = pulse_shape(&symbols, samples_per_symbol);
    println!("   {} symbols → {} samples", symbols.len(), tx_samples.len());

    // Step 3: Add channel impairments
    println!("\n3. Channel Simulation:");
    let rx_samples = add_channel_impairments(&tx_samples, snr_db);
    println!("   Added AWGN at {} dB SNR", snr_db);

    // Step 4: Symbol timing recovery (simplified - we know timing)
    println!("\n4. Symbol Timing Recovery:");
    let recovered_symbols = timing_recovery(&rx_samples, samples_per_symbol);
    println!("   Recovered {} symbols", recovered_symbols.len());

    // Step 5: Decision making
    println!("\n5. Decision Making:");
    let decided_symbols = make_decisions(&recovered_symbols, order);

    // Step 6: Symbol to bits
    let decoded_bits = symbols_to_bits(&decided_symbols, order);
    println!("   Decoded {} bits", decoded_bits.len());

    // Step 7: Calculate BER
    let errors = bits.iter().zip(decoded_bits.iter())
        .filter(|(a, b)| a != b)
        .count();
    let ber = errors as f64 / bits.len() as f64;
    println!("\n6. Results:");
    println!("   Bit errors: {} / {}", errors, bits.len());
    println!("   BER: {:.2e}", ber);

    // Decode message
    let decoded_bytes: Vec<u8> = decoded_bits.chunks(8)
        .filter(|c| c.len() == 8)
        .map(|chunk| chunk.iter().enumerate()
             .fold(0u8, |acc, (i, &b)| if b { acc | (1 << (7 - i)) } else { acc }))
        .collect();

    if let Ok(decoded_msg) = String::from_utf8(decoded_bytes) {
        println!("   Decoded: \"{}\"", decoded_msg);
    }
}

fn get_constellation(order: PskOrder) -> Vec<IQSample> {
    match order {
        PskOrder::Bpsk => vec![
            IQSample::new(-1.0, 0.0),  // 0
            IQSample::new(1.0, 0.0),   // 1
        ],
        PskOrder::Qpsk => {
            let s = 1.0 / 2.0_f64.sqrt();
            vec![
                IQSample::new(s, s),    // 00
                IQSample::new(-s, s),   // 01
                IQSample::new(-s, -s),  // 10
                IQSample::new(s, -s),   // 11
            ]
        },
        PskOrder::Psk8 => {
            (0..8).map(|i| {
                let angle = 2.0 * PI * i as f64 / 8.0 + PI / 8.0;
                IQSample::new(angle.cos(), angle.sin())
            }).collect()
        },
    }
}

fn bits_to_symbols(bits: &[bool], order: PskOrder) -> Vec<IQSample> {
    let bits_per_symbol = match order {
        PskOrder::Bpsk => 1,
        PskOrder::Qpsk => 2,
        PskOrder::Psk8 => 3,
    };

    let constellation = get_constellation(order);

    bits.chunks(bits_per_symbol)
        .filter(|chunk| chunk.len() == bits_per_symbol)
        .map(|chunk| {
            let index = chunk.iter()
                .enumerate()
                .fold(0usize, |acc, (i, &b)| {
                    if b { acc | (1 << (bits_per_symbol - 1 - i)) } else { acc }
                });
            constellation[index]
        })
        .collect()
}

fn symbols_to_bits(symbols: &[usize], order: PskOrder) -> Vec<bool> {
    let bits_per_symbol = match order {
        PskOrder::Bpsk => 1,
        PskOrder::Qpsk => 2,
        PskOrder::Psk8 => 3,
    };

    symbols.iter()
        .flat_map(|&s| {
            (0..bits_per_symbol).rev().map(move |i| (s >> i) & 1 == 1)
        })
        .collect()
}

fn pulse_shape(symbols: &[IQSample], samples_per_symbol: usize) -> Vec<IQSample> {
    // Simple rectangular pulse shaping
    symbols.iter()
        .flat_map(|&s| vec![s; samples_per_symbol])
        .collect()
}

fn add_channel_impairments(samples: &[IQSample], snr_db: f64) -> Vec<IQSample> {
    // Calculate noise power
    let signal_power: f64 = samples.iter()
        .map(|s| s.re * s.re + s.im * s.im)
        .sum::<f64>() / samples.len() as f64;

    let snr_linear = 10.0_f64.powf(snr_db / 10.0);
    let noise_power = signal_power / snr_linear;
    let noise_std = (noise_power / 2.0).sqrt(); // Divided by 2 for I and Q

    // Simple PRNG for noise
    let mut seed = 12345u64;
    let mut next_random = || {
        seed = seed.wrapping_mul(1103515245).wrapping_add(12345);
        // Box-Muller approximation
        let u1 = (seed as f64) / (u64::MAX as f64);
        seed = seed.wrapping_mul(1103515245).wrapping_add(12345);
        let u2 = (seed as f64) / (u64::MAX as f64);
        let r = (-2.0 * u1.max(1e-10).ln()).sqrt();
        let theta = 2.0 * PI * u2;
        (r * theta.cos(), r * theta.sin())
    };

    samples.iter()
        .map(|s| {
            let (n_i, n_q) = next_random();
            IQSample::new(
                s.re + n_i * noise_std,
                s.im + n_q * noise_std,
            )
        })
        .collect()
}

fn timing_recovery(samples: &[IQSample], samples_per_symbol: usize) -> Vec<IQSample> {
    // Simplified: sample at middle of each symbol period
    // Real implementation would use Gardner or Mueller-Muller timing recovery

    let offset = samples_per_symbol / 2;

    samples.chunks(samples_per_symbol)
        .filter(|chunk| chunk.len() == samples_per_symbol)
        .map(|chunk| chunk[offset])
        .collect()
}

fn make_decisions(symbols: &[IQSample], order: PskOrder) -> Vec<usize> {
    let constellation = get_constellation(order);

    symbols.iter()
        .map(|s| {
            // Find closest constellation point
            constellation.iter()
                .enumerate()
                .min_by(|(_, a), (_, b)| {
                    let dist_a = (s.re - a.re).powi(2) + (s.im - a.im).powi(2);
                    let dist_b = (s.re - b.re).powi(2) + (s.im - b.im).powi(2);
                    dist_a.partial_cmp(&dist_b).unwrap()
                })
                .map(|(i, _)| i)
                .unwrap_or(0)
        })
        .collect()
}
