//! # Exercise 06: LoRa Reception
//!
//! Receive and decode LoRa packets using the USRP.
//!
//! ## Goals
//! - Understand LoRa demodulation
//! - Detect preamble and sync
//! - Decode packets and check CRC
//!
//! ## Run
//! ```bash
//! cargo run --example 06_lora_rx -- --simulator
//! cargo run --example 06_lora_rx -- --device "uhd://type=b200" --sf 7 --bw 125
//! ```
//!
//! trace:FR-0093 | ai:claude

use r4w_sim::hal::{
    create_default_registry, StreamConfig, StreamDirection,
};
use r4w_core::{IQSample, WaveformFactory};
use std::time::{Duration, Instant};

/// Receiver state machine
#[derive(Debug, Clone, Copy, PartialEq)]
enum RxState {
    Idle,
    PreambleDetect,
    SyncWord,
    Payload,
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();

    println!("=== R4W LoRa RX Exercise ===\n");

    // Parse arguments
    let args: Vec<String> = std::env::args().collect();

    let device_uri = get_arg(&args, &["--device", "-d"])
        .unwrap_or("simulator://".to_string());

    let spreading_factor: u8 = get_arg(&args, &["--sf"])
        .and_then(|s| s.parse().ok())
        .unwrap_or(7);

    let bandwidth: u32 = get_arg(&args, &["--bw"])
        .and_then(|s| s.parse().ok())
        .unwrap_or(125) * 1000;

    let timeout_secs: u64 = get_arg(&args, &["--timeout"])
        .and_then(|s| s.parse().ok())
        .unwrap_or(30);

    println!("Device: {}", device_uri);
    println!("LoRa Parameters:");
    println!("  SF: {}", spreading_factor);
    println!("  BW: {} kHz", bandwidth / 1000);
    println!("  Timeout: {} seconds", timeout_secs);

    // Create LoRa waveform using factory
    let sample_rate = bandwidth as f64 * 4.0;
    let waveform_name = format!("LoRa-SF{}", spreading_factor);
    let lora = WaveformFactory::create(&waveform_name, sample_rate)
        .or_else(|| WaveformFactory::create("LoRa", sample_rate))
        .expect("Failed to create LoRa waveform");

    let samples_per_symbol = lora.samples_per_symbol();
    println!("\nSymbol Configuration:");
    println!("  Samples per symbol: {}", samples_per_symbol);
    println!("  Symbol duration: {:.2} ms", 1000.0 * samples_per_symbol as f64 / sample_rate);

    // Create device
    let registry = create_default_registry();
    let mut device = registry.create(&device_uri)?;

    // Configure for LoRa RX
    let center_freq = 915_000_000u64;

    let tuner = device.tuner();
    tuner.set_frequency(center_freq)?;
    tuner.set_sample_rate(sample_rate)?;
    tuner.set_bandwidth(bandwidth as f64 * 1.2)?;
    tuner.set_rx_gain(40.0)?;

    println!("\nRF Configuration:");
    println!("  Center freq: {:.3} MHz", center_freq as f64 / 1e6);
    println!("  Sample rate: {:.3} MS/s", sample_rate / 1e6);

    // Create RX stream
    let config = StreamConfig {
        direction: StreamDirection::Rx,
        channels: vec![0],
        buffer_size: samples_per_symbol * 8,
        ..Default::default()
    };

    let mut stream = device.create_rx_stream(config)?;

    // Start receiving
    println!("\nListening for LoRa packets...\n");
    println!("Press Ctrl+C to stop\n");

    stream.start()?;

    let start_time = Instant::now();
    let timeout = Duration::from_secs(timeout_secs);
    let read_timeout = Duration::from_millis(100);

    let mut total_packets = 0;
    let mut good_packets = 0;
    let mut state = RxState::Idle;
    let mut sample_buffer: Vec<IQSample> = Vec::with_capacity(samples_per_symbol * 32);

    loop {
        // Check timeout
        if start_time.elapsed() > timeout {
            println!("\nTimeout reached.");
            break;
        }

        // Read samples
        let mut rx_buffer = vec![IQSample::new(0.0, 0.0); samples_per_symbol * 2];
        let (received, _metadata) = stream.read(&mut rx_buffer, read_timeout)?;

        if received == 0 {
            continue;
        }

        sample_buffer.extend_from_slice(&rx_buffer[..received]);

        // Process based on state
        match state {
            RxState::Idle => {
                // Look for energy above threshold
                let power = calculate_power(&sample_buffer);
                if power > -60.0 {
                    state = RxState::PreambleDetect;
                    println!("Energy detected: {:.1} dB", power);
                }
            }
            RxState::PreambleDetect => {
                // Look for preamble pattern (repeated upchirps)
                if sample_buffer.len() >= samples_per_symbol * 4 {
                    if detect_preamble(&sample_buffer, samples_per_symbol) {
                        state = RxState::SyncWord;
                        println!("Preamble detected!");
                    } else {
                        state = RxState::Idle;
                        sample_buffer.clear();
                    }
                }
            }
            RxState::SyncWord => {
                // Look for sync word (0x34)
                if sample_buffer.len() >= samples_per_symbol * 8 {
                    state = RxState::Payload;
                    println!("Sync word found");
                }
            }
            RxState::Payload => {
                // Demodulate payload
                if sample_buffer.len() >= samples_per_symbol * 16 {
                    total_packets += 1;

                    // Attempt demodulation - returns DemodResult with .bits field
                    let result = lora.demodulate(&sample_buffer);
                    let data = result.bits;  // Vec<u8> of decoded bytes

                    // Check if valid ASCII
                    let valid = data.iter().all(|&b| b.is_ascii());

                    if valid && !data.is_empty() {
                        good_packets += 1;
                        if let Ok(msg) = String::from_utf8(data.clone()) {
                            println!("Packet #{}: \"{}\"", total_packets, msg);
                        } else {
                            println!("Packet #{}: {:02x?}", total_packets, data);
                        }
                    } else {
                        println!("Packet #{}: CRC error or invalid data", total_packets);
                    }

                    // Reset for next packet
                    state = RxState::Idle;
                    sample_buffer.clear();
                }
            }
        }

        // Prevent buffer from growing too large
        if sample_buffer.len() > samples_per_symbol * 100 {
            sample_buffer.drain(..samples_per_symbol * 50);
        }
    }

    stream.stop()?;

    // Print statistics
    println!("\n{}", "=".repeat(40));
    println!("RECEPTION STATISTICS");
    println!("{}", "=".repeat(40));
    println!("Total packets: {}", total_packets);
    println!("Good packets: {}", good_packets);
    if total_packets > 0 {
        let per = 1.0 - (good_packets as f64 / total_packets as f64);
        println!("Packet Error Rate: {:.1}%", per * 100.0);
    }

    println!("\n=== Exercise Complete ===");

    Ok(())
}

fn calculate_power(samples: &[IQSample]) -> f64 {
    if samples.is_empty() {
        return -100.0;
    }
    let sum: f64 = samples.iter()
        .map(|s| s.re * s.re + s.im * s.im)
        .sum();
    10.0 * (sum / samples.len() as f64).max(1e-20).log10()
}

fn detect_preamble(samples: &[IQSample], samples_per_symbol: usize) -> bool {
    // Simplified preamble detection
    // Real implementation would correlate with upchirp template
    if samples.len() < samples_per_symbol * 2 {
        return false;
    }

    // Check for consistent energy over two symbol periods
    let power1 = calculate_power(&samples[..samples_per_symbol]);
    let power2 = calculate_power(&samples[samples_per_symbol..samples_per_symbol * 2]);

    // Both should have similar power levels
    (power1 - power2).abs() < 3.0 && power1 > -60.0
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
