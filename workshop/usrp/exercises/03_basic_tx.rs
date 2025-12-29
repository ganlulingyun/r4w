//! # Exercise 03: Basic Transmit
//!
//! Learn to transmit a test tone using the USRP.
//!
//! ## Goals
//! - Configure the TX chain
//! - Generate a test signal
//! - Transmit IQ samples
//!
//! ## IMPORTANT
//! Only transmit on frequencies you are licensed to use!
//! Use low power and a dummy load or loopback cable for testing.
//!
//! ## Run
//! ```bash
//! cargo run --example 03_basic_tx -- --simulator
//! cargo run --example 03_basic_tx -- --device "uhd://type=b200" --power low
//! ```
//!
//! trace:FR-0093 | ai:claude

use r4w_sim::hal::{
    create_default_registry, StreamConfig, StreamDirection,
};
use r4w_core::IQSample;
use std::f64::consts::PI;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();

    println!("=== R4W Basic TX Exercise ===\n");
    println!("WARNING: Only transmit on frequencies you are licensed for!");
    println!("    Use low power and appropriate filtering.\n");

    // Parse arguments
    let args: Vec<String> = std::env::args().collect();
    let device_uri = args.iter()
        .position(|a| a == "--device" || a == "-d")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
        .unwrap_or("simulator://");

    let low_power = args.iter().any(|a| a == "--power" || a == "low");

    println!("Connecting to: {}", device_uri);

    // Create device
    let registry = create_default_registry();
    let mut device = registry.create(device_uri)?;

    // Get capabilities
    let caps = device.capabilities();
    if !caps.can_tx {
        println!("ERROR: Device does not support TX");
        return Ok(());
    }

    println!("\nDevice Capabilities:");
    println!("  TX channels: {}", caps.tx_channels);
    println!("  Frequency range: {:.0} - {:.0} MHz",
             caps.min_frequency / 1e6,
             caps.max_frequency / 1e6);

    // Configure tuner
    let tuner = device.tuner();

    let center_freq = 915_000_000u64;  // 915 MHz ISM band
    let sample_rate = 1e6;             // 1 MS/s
    let bandwidth = 1e6;               // 1 MHz
    let tx_gain = if low_power { 10.0 } else { 30.0 };

    println!("\nConfiguring TX:");
    println!("  Center frequency: {:.3} MHz", center_freq as f64 / 1e6);
    println!("  Sample rate: {:.3} MS/s", sample_rate / 1e6);
    println!("  TX gain: {:.1} dB {}", tx_gain, if low_power { "(low power)" } else { "" });

    tuner.set_frequency(center_freq)?;
    tuner.set_sample_rate(sample_rate)?;
    tuner.set_bandwidth(bandwidth)?;
    tuner.set_tx_gain(tx_gain)?;

    // Create TX stream
    let config = StreamConfig {
        direction: StreamDirection::Tx,
        channels: vec![0],
        buffer_size: 8192,
        ..Default::default()
    };

    let mut stream = device.create_tx_stream(config)?;

    // Generate test signal - a simple tone at offset frequency
    let tone_offset = 100e3;  // 100 kHz offset from center
    let duration_secs = 2.0;
    let num_samples = (sample_rate * duration_secs) as usize;

    println!("\nGenerating test tone:");
    println!("  Frequency offset: {:.0} kHz", tone_offset / 1e3);
    println!("  Duration: {:.1} seconds", duration_secs);
    println!("  Total samples: {}", num_samples);

    let samples = generate_tone(tone_offset, sample_rate, num_samples);

    // Transmit in chunks
    println!("\nTransmitting...");
    stream.start()?;

    let chunk_size = 4096;
    let mut total_sent = 0;
    let timeout = Duration::from_millis(100);

    for chunk in samples.chunks(chunk_size) {
        let sent = stream.write(chunk, None, timeout)?;
        total_sent += sent;

        // Print progress
        let progress = (total_sent as f64 / num_samples as f64 * 100.0) as u32;
        print!("\rProgress: {}% ({}/{} samples)", progress, total_sent, num_samples);
    }
    println!();

    // Wait for buffer to drain
    std::thread::sleep(Duration::from_millis(100));
    stream.stop()?;

    println!("\nTransmission complete!");
    println!("  Samples sent: {}", total_sent);

    println!("\n=== Exercise Complete ===");

    Ok(())
}

/// Generate a complex sinusoid (tone) at the given offset frequency
fn generate_tone(offset_hz: f64, sample_rate: f64, num_samples: usize) -> Vec<IQSample> {
    let phase_increment = 2.0 * PI * offset_hz / sample_rate;

    (0..num_samples)
        .map(|i| {
            let phase = phase_increment * i as f64;
            IQSample::new(phase.cos(), phase.sin())
        })
        .collect()
}

/// Generate a chirp signal (frequency sweep)
#[allow(dead_code)]
fn generate_chirp(
    start_freq: f64,
    end_freq: f64,
    sample_rate: f64,
    duration_secs: f64,
) -> Vec<IQSample> {
    let num_samples = (sample_rate * duration_secs) as usize;
    let freq_rate = (end_freq - start_freq) / duration_secs;

    (0..num_samples)
        .map(|i| {
            let t = i as f64 / sample_rate;
            let phase = 2.0 * PI * (start_freq * t + 0.5 * freq_rate * t * t);
            IQSample::new(phase.cos(), phase.sin())
        })
        .collect()
}

/// Generate BPSK modulated data
#[allow(dead_code)]
fn generate_bpsk(data: &[bool], samples_per_symbol: usize) -> Vec<IQSample> {
    data.iter()
        .flat_map(|&bit| {
            let amplitude = if bit { 1.0 } else { -1.0 };
            vec![IQSample::new(amplitude, 0.0); samples_per_symbol]
        })
        .collect()
}
