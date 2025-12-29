//! # Exercise 05: LoRa Transmission
//!
//! Transmit LoRa packets using the USRP.
//!
//! ## Goals
//! - Understand LoRa modulation parameters
//! - Generate LoRa waveforms using r4w-core
//! - Transmit packets with configurable SF/BW/CR
//!
//! ## Run
//! ```bash
//! cargo run --example 05_lora_tx -- --simulator
//! cargo run --example 05_lora_tx -- --device "uhd://type=b200" --sf 7 --bw 125
//! ```
//!
//! trace:FR-0093 | ai:claude

use r4w_sim::hal::{
    create_default_registry, StreamConfig, StreamDirection,
};
use r4w_core::{IQSample, WaveformFactory};
use std::time::Duration;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();

    println!("=== R4W LoRa TX Exercise ===\n");
    println!("WARNING: Only transmit on licensed frequencies!\n");

    // Parse arguments
    let args: Vec<String> = std::env::args().collect();

    let device_uri = get_arg(&args, &["--device", "-d"])
        .unwrap_or("simulator://".to_string());

    let spreading_factor: u8 = get_arg(&args, &["--sf"])
        .and_then(|s| s.parse().ok())
        .unwrap_or(7);

    let bandwidth: u32 = get_arg(&args, &["--bw"])
        .and_then(|s| s.parse().ok())
        .unwrap_or(125) * 1000;  // kHz to Hz

    let message = get_arg(&args, &["--message", "-m"])
        .unwrap_or("Hello R4W!".to_string());

    println!("Device: {}", device_uri);
    println!("LoRa Parameters:");
    println!("  SF: {}", spreading_factor);
    println!("  BW: {} kHz", bandwidth / 1000);
    println!("  Message: \"{}\"", message);

    // Create LoRa waveform using factory
    let sample_rate = bandwidth as f64 * 4.0;  // Oversample 4x
    let waveform_name = format!("LoRa-SF{}", spreading_factor);
    let lora = WaveformFactory::create(&waveform_name, sample_rate)
        .or_else(|| WaveformFactory::create("LoRa", sample_rate))
        .expect("Failed to create LoRa waveform");

    // Print waveform info
    let info = lora.info();
    println!("\nWaveform Info:");
    println!("  Name: {}", info.name);
    println!("  Bits per symbol: {}", info.bits_per_symbol);
    println!("  Samples per symbol: {}", lora.samples_per_symbol());

    // Create device
    let registry = create_default_registry();
    let mut device = registry.create(&device_uri)?;

    let caps = device.capabilities();
    if !caps.can_tx {
        println!("ERROR: Device does not support TX");
        return Ok(());
    }

    // Configure for LoRa
    let center_freq = 915_000_000u64;  // US ISM band

    let tuner = device.tuner();
    tuner.set_frequency(center_freq)?;
    tuner.set_sample_rate(sample_rate)?;
    tuner.set_bandwidth(bandwidth as f64 * 1.2)?;
    tuner.set_tx_gain(20.0)?;

    println!("\nRF Configuration:");
    println!("  Center freq: {:.3} MHz", center_freq as f64 / 1e6);
    println!("  Sample rate: {:.3} MS/s", sample_rate / 1e6);

    // Modulate message bytes directly
    let data = message.as_bytes();
    println!("\nEncoding:");
    println!("  Data bytes: {}", data.len());

    // Modulate - Waveform::modulate takes &[u8]
    let samples = lora.modulate(data);
    println!("  Modulated samples: {}", samples.len());

    // Create TX stream
    let config = StreamConfig {
        direction: StreamDirection::Tx,
        channels: vec![0],
        buffer_size: 8192,
        ..Default::default()
    };

    let mut stream = device.create_tx_stream(config)?;

    // Transmit
    println!("\nTransmitting {} packets...", 3);
    let timeout = Duration::from_millis(100);

    for pkt_num in 1..=3 {
        println!("  Packet {}/3", pkt_num);

        stream.start()?;

        // Send in chunks
        for chunk in samples.chunks(4096) {
            stream.write(chunk, None, timeout)?;
        }

        // Gap between packets
        let silence = vec![IQSample::new(0.0, 0.0); 4096];
        stream.write(&silence, None, timeout)?;

        std::thread::sleep(Duration::from_millis(100));
        stream.stop()?;

        std::thread::sleep(Duration::from_millis(500));
    }

    println!("\n=== Exercise Complete ===");

    Ok(())
}

fn get_arg(args: &[String], names: &[&str]) -> Option<String> {
    for name in names {
        if let Some(pos) = args.iter().position(|a| a == *name) {
            if pos + 1 < args.len() {
                return Some(args[pos + 1].clone());
            }
        }
    }
    None
}
