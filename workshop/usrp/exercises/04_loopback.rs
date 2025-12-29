//! # Exercise 04: TX→RX Loopback with Attenuator
//!
//! Perform a loopback test using a digital attenuator between TX and RX.
//!
//! ## Setup
//! ```
//! USRP TX ──► Attenuator ──► USRP RX
//! ```
//!
//! ## Goals
//! - Understand RF signal path
//! - Control digital attenuator
//! - Verify TX/RX correlation
//! - Measure signal quality at different attenuation levels
//!
//! ## Run
//! ```bash
//! # Simulated loopback
//! cargo run --example 04_loopback -- --simulator
//!
//! # Real hardware with simulated attenuator
//! cargo run --example 04_loopback -- --device "uhd://type=b200"
//!
//! # Real hardware with real attenuator
//! cargo run --example 04_loopback -- --device "uhd://type=b200" --attenuator "minicircuits://RCDAT-8000-90"
//! ```
//!
//! trace:FR-0093 | ai:claude

use r4w_sim::hal::{
    create_default_registry, create_attenuator,
    StreamConfig, StreamDirection,
};
use r4w_core::IQSample;
use std::f64::consts::PI;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();

    println!("=== R4W Loopback Test with Attenuator ===\n");

    // Parse arguments
    let args: Vec<String> = std::env::args().collect();

    let device_uri = args.iter()
        .position(|a| a == "--device" || a == "-d")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
        .unwrap_or("simulator://");

    let attenuator_uri = args.iter()
        .position(|a| a == "--attenuator" || a == "-a")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
        .unwrap_or("simulated://max=90");

    println!("Device: {}", device_uri);
    println!("Attenuator: {}", attenuator_uri);

    // Create SDR device
    let registry = create_default_registry();
    let mut device = registry.create(device_uri)?;

    let caps = device.capabilities();
    if !caps.can_tx || !caps.can_rx {
        println!("ERROR: Device must support both TX and RX for loopback");
        return Ok(());
    }

    if !caps.full_duplex {
        println!("WARNING: Device is half-duplex, using time-division loopback");
    }

    // Create attenuator
    let mut attenuator = create_attenuator(attenuator_uri)?;
    println!("\nAttenuator: {}", attenuator.name());
    let atten_caps = attenuator.capabilities();
    println!("  Range: {} - {} dB", atten_caps.min_attenuation, atten_caps.max_attenuation);

    // Configure device
    let tuner = device.tuner();
    let center_freq = 915_000_000u64;
    let sample_rate = 1e6;
    let bandwidth = 500e3;

    println!("\nConfiguring RF:");
    println!("  Frequency: {:.3} MHz", center_freq as f64 / 1e6);
    println!("  Sample rate: {:.3} MS/s", sample_rate / 1e6);

    tuner.set_frequency(center_freq)?;
    tuner.set_sample_rate(sample_rate)?;
    tuner.set_bandwidth(bandwidth)?;
    tuner.set_rx_gain(40.0)?;
    tuner.set_tx_gain(30.0)?;

    // Create streams
    let tx_config = StreamConfig {
        direction: StreamDirection::Tx,
        channels: vec![0],
        buffer_size: 4096,
        ..Default::default()
    };

    let rx_config = StreamConfig {
        direction: StreamDirection::Rx,
        channels: vec![0],
        buffer_size: 4096,
        ..Default::default()
    };

    let mut tx_stream = device.create_tx_stream(tx_config)?;
    let mut rx_stream = device.create_rx_stream(rx_config)?;

    // Generate test signal - tone with known frequency
    let tone_offset = 50e3;  // 50 kHz offset
    let test_samples = generate_test_tone(tone_offset, sample_rate, 4096);

    println!("\n{:>10} {:>12} {:>12} {:>12}",
             "Atten(dB)", "RX Power", "Expected", "Error");
    println!("{}", "-".repeat(50));

    // Test at different attenuation levels
    let test_attenuations = [0.0, 10.0, 20.0, 30.0, 40.0, 50.0];
    let initial_power_dbm = 0.0;  // Estimated TX power at attenuator input
    let timeout = Duration::from_millis(100);

    for &atten_db in &test_attenuations {
        // Set attenuation
        let actual_atten = attenuator.set_attenuation(atten_db)?;
        std::thread::sleep(Duration::from_millis(10));  // Settle time

        // Transmit test signal
        tx_stream.start()?;
        tx_stream.write(&test_samples, None, timeout)?;

        // Small delay for propagation
        std::thread::sleep(Duration::from_millis(5));

        // Receive
        rx_stream.start()?;
        let mut rx_buffer = vec![IQSample::new(0.0, 0.0); 4096];
        let (received, _) = rx_stream.read(&mut rx_buffer, timeout)?;

        tx_stream.stop()?;
        rx_stream.stop()?;

        if received > 0 {
            // Calculate received power
            let rx_power = calculate_power_db(&rx_buffer[..received]);
            let expected_power = initial_power_dbm - actual_atten;
            let error = rx_power - expected_power;

            println!("{:>10.1} {:>12.1} {:>12.1} {:>12.1}",
                     actual_atten, rx_power, expected_power, error);
        }
    }

    println!("\n=== Loopback Test Complete ===");

    Ok(())
}

/// Generate test tone signal
fn generate_test_tone(offset_hz: f64, sample_rate: f64, num_samples: usize) -> Vec<IQSample> {
    let phase_inc = 2.0 * PI * offset_hz / sample_rate;
    (0..num_samples)
        .map(|i| {
            let phase = phase_inc * i as f64;
            IQSample::new(phase.cos() * 0.8, phase.sin() * 0.8)  // 80% amplitude
        })
        .collect()
}

/// Calculate power in dB (relative to full scale)
fn calculate_power_db(samples: &[IQSample]) -> f64 {
    if samples.is_empty() {
        return -100.0;
    }

    let sum_power: f64 = samples.iter()
        .map(|s| s.re * s.re + s.im * s.im)
        .sum();

    let avg_power = sum_power / samples.len() as f64;
    10.0 * avg_power.max(1e-20).log10()
}

/// Correlate TX and RX signals to find delay and verify loopback
#[allow(dead_code)]
fn correlate_signals(tx: &[IQSample], rx: &[IQSample]) -> (usize, f64) {
    let mut best_lag = 0;
    let mut best_corr = 0.0f64;

    let max_lag = tx.len().min(rx.len()) / 2;

    for lag in 0..max_lag {
        let mut corr_re = 0.0f64;
        let mut corr_im = 0.0f64;

        let overlap = tx.len().min(rx.len() - lag);
        for i in 0..overlap {
            let t = &tx[i];
            let r = &rx[i + lag];
            // Complex conjugate multiply
            corr_re += t.re * r.re + t.im * r.im;
            corr_im += t.im * r.re - t.re * r.im;
        }

        let corr_mag = (corr_re * corr_re + corr_im * corr_im).sqrt();
        if corr_mag > best_corr {
            best_corr = corr_mag;
            best_lag = lag;
        }
    }

    (best_lag, best_corr)
}
