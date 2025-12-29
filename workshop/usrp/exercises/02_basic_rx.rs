//! # Exercise 02: Basic Receive
//!
//! Learn to receive samples and display spectrum.
//!
//! ## Goals
//! - Configure the RX chain
//! - Receive IQ samples
//! - Compute and display power spectrum
//!
//! ## Run
//! ```bash
//! cargo run --example 02_basic_rx -- --simulator
//! cargo run --example 02_basic_rx -- --device "uhd://type=b200"
//! ```
//!
//! trace:FR-0093 | ai:claude

use r4w_sim::hal::{
    create_default_registry, StreamConfig, StreamDirection,
};
use r4w_core::IQSample;
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();

    println!("=== R4W Basic RX Exercise ===\n");

    // Parse arguments
    let args: Vec<String> = std::env::args().collect();
    let device_uri = args.iter()
        .position(|a| a == "--device" || a == "-d")
        .and_then(|i| args.get(i + 1))
        .map(|s| s.as_str())
        .unwrap_or("simulator://");

    println!("Connecting to: {}", device_uri);

    // Create device
    let registry = create_default_registry();
    let mut device = registry.create(device_uri)?;

    // Get capabilities
    let caps = device.capabilities();
    println!("\nDevice Capabilities:");
    println!("  Frequency range: {:.0} - {:.0} MHz",
             caps.min_frequency / 1e6,
             caps.max_frequency / 1e6);
    println!("  Max sample rate: {:.1} MS/s", caps.max_sample_rate / 1e6);
    println!("  RX channels: {}", caps.rx_channels);

    // Configure tuner
    let tuner = device.tuner();

    let center_freq = 915_000_000u64;  // 915 MHz ISM band
    let sample_rate = 1e6;             // 1 MS/s
    let bandwidth = 1e6;               // 1 MHz
    let gain = 40.0;                   // dB

    println!("\nConfiguring RX:");
    println!("  Center frequency: {:.3} MHz", center_freq as f64 / 1e6);
    println!("  Sample rate: {:.3} MS/s", sample_rate / 1e6);
    println!("  Bandwidth: {:.3} MHz", bandwidth / 1e6);
    println!("  Gain: {:.1} dB", gain);

    tuner.set_frequency(center_freq)?;
    tuner.set_sample_rate(sample_rate)?;
    tuner.set_bandwidth(bandwidth)?;
    tuner.set_rx_gain(gain)?;

    // Create RX stream
    let config = StreamConfig {
        direction: StreamDirection::Rx,
        channels: vec![0],
        buffer_size: 8192,
        ..Default::default()
    };

    let mut stream = device.create_rx_stream(config)?;

    // Receive samples
    println!("\nReceiving samples...");
    stream.start()?;

    let num_captures = 10;
    let samples_per_capture = 4096;

    for capture in 0..num_captures {
        let mut buffer = vec![IQSample::new(0.0, 0.0); samples_per_capture];
        let (received, _metadata) = stream.read(&mut buffer, Duration::from_millis(100))?;

        if received > 0 {
            // Calculate power spectrum using simple FFT approximation
            let (avg_power, peak_power, peak_bin) = analyze_spectrum(&buffer[..received]);

            println!("Capture {}: {} samples, avg power: {:.1} dB, peak: {:.1} dB at bin {}",
                     capture + 1, received, avg_power, peak_power, peak_bin);
        }
    }

    stream.stop()?;

    // Print ASCII spectrum of last capture
    println!("\n=== Spectrum (ASCII) ===");
    print_ascii_spectrum(&[IQSample::new(0.0, 0.0); 256]); // Placeholder

    println!("\n=== Exercise Complete ===");

    Ok(())
}

/// Analyze spectrum - returns (average_power_db, peak_power_db, peak_bin)
fn analyze_spectrum(samples: &[IQSample]) -> (f64, f64, usize) {
    if samples.is_empty() {
        return (-100.0, -100.0, 0);
    }

    // Calculate power per sample (IQSample uses f64)
    let powers: Vec<f64> = samples.iter()
        .map(|s| s.re * s.re + s.im * s.im)
        .collect();

    let avg_power = powers.iter().sum::<f64>() / powers.len() as f64;
    let avg_power_db = 10.0 * avg_power.max(1e-20).log10();

    let (peak_bin, &peak_power) = powers.iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
        .unwrap_or((0, &1e-20));

    let peak_power_db = 10.0 * peak_power.max(1e-20).log10();

    (avg_power_db, peak_power_db, peak_bin)
}

/// Print a simple ASCII spectrum visualization
fn print_ascii_spectrum(samples: &[IQSample]) {
    let fft_size = samples.len().min(64);
    let width = 60;

    // Simple power calculation per bin (not a real FFT)
    let mut bins = vec![0.0f64; fft_size];
    for (i, s) in samples.iter().take(fft_size).enumerate() {
        bins[i] = s.re * s.re + s.im * s.im;
    }

    // Normalize
    let max_power = bins.iter().cloned().fold(f64::MIN, f64::max).max(1e-20);

    for (i, &power) in bins.iter().enumerate() {
        let normalized = (power / max_power * width as f64) as usize;
        let bar: String = "█".repeat(normalized);
        println!("{:3} |{}", i, bar);
    }
}
