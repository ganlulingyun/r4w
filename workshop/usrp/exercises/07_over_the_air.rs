//! # Exercise 07: Over-the-Air Link
//!
//! Establish a full OTA communication link between two USRPs.
//!
//! ## Setup
//! ```
//! USRP #1 (TX) ───────► OTA ───────► USRP #2 (RX)
//!     or
//! USRP (TX) ──► Attenuator ──► USRP (RX) [same device or two]
//! ```
//!
//! ## Goals
//! - Establish bidirectional communication
//! - Implement packet framing (preamble, sync, length, payload, CRC)
//! - Measure link quality (RSSI, SNR, PER)
//! - Test with varying channel conditions
//!
//! ## Run
//! ```bash
//! # TX side
//! cargo run --example 07_over_the_air -- --mode tx --device "uhd://serial=TX123"
//!
//! # RX side (different terminal)
//! cargo run --example 07_over_the_air -- --mode rx --device "uhd://serial=RX456"
//!
//! # Loopback (single device)
//! cargo run --example 07_over_the_air -- --mode loopback --device "uhd://type=b200"
//! ```
//!
//! trace:FR-0093 | ai:claude

use r4w_sim::hal::{
    create_default_registry, SdrDeviceExt, StreamConfig, StreamDirection,
};
use r4w_core::{IQSample, Waveform, WaveformFactory};
use std::time::{Duration, Instant};

/// Operating mode
#[derive(Debug, Clone, Copy)]
enum Mode {
    Tx,
    Rx,
    Loopback,
}

/// Packet structure
#[derive(Debug, Clone)]
struct Packet {
    sequence: u16,
    payload: Vec<u8>,
    crc: u16,
}

impl Packet {
    fn new(sequence: u16, payload: Vec<u8>) -> Self {
        let crc = calculate_crc(&payload);
        Packet { sequence, payload, crc }
    }

    fn to_bytes(&self) -> Vec<u8> {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(&self.sequence.to_be_bytes());
        bytes.push(self.payload.len() as u8);
        bytes.extend_from_slice(&self.payload);
        bytes.extend_from_slice(&self.crc.to_be_bytes());
        bytes
    }

    fn from_bytes(bytes: &[u8]) -> Option<Self> {
        if bytes.len() < 5 {
            return None;
        }
        let sequence = u16::from_be_bytes([bytes[0], bytes[1]]);
        let length = bytes[2] as usize;
        if bytes.len() < 5 + length {
            return None;
        }
        let payload = bytes[3..3 + length].to_vec();
        let crc = u16::from_be_bytes([bytes[3 + length], bytes[4 + length]]);

        // Verify CRC
        if calculate_crc(&payload) == crc {
            Some(Packet { sequence, payload, crc })
        } else {
            None
        }
    }
}

fn calculate_crc(data: &[u8]) -> u16 {
    // CRC-16-CCITT
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();

    println!("=== R4W Over-the-Air Link ===\n");

    // Parse arguments
    let args: Vec<String> = std::env::args().collect();

    let mode = match get_arg(&args, &["--mode", "-m"]).as_deref() {
        Some("tx") => Mode::Tx,
        Some("rx") => Mode::Rx,
        Some("loopback") | _ => Mode::Loopback,
    };

    let device_uri = get_arg(&args, &["--device", "-d"])
        .unwrap_or("simulator://".to_string());

    let spreading_factor: u8 = get_arg(&args, &["--sf"])
        .and_then(|s| s.parse().ok())
        .unwrap_or(7);

    let bandwidth: u32 = get_arg(&args, &["--bw"])
        .and_then(|s| s.parse().ok())
        .unwrap_or(125) * 1000;

    let num_packets: u32 = get_arg(&args, &["--packets", "-n"])
        .and_then(|s| s.parse().ok())
        .unwrap_or(10);

    println!("Mode: {:?}", mode);
    println!("Device: {}", device_uri);
    println!("SF: {}, BW: {} kHz", spreading_factor, bandwidth / 1000);
    println!("Packets to send/receive: {}", num_packets);

    // Create waveform and device
    let sample_rate = bandwidth as f64 * 4.0;
    let waveform_name = format!("LoRa-SF{}", spreading_factor);
    let lora = WaveformFactory::create(&waveform_name, sample_rate)
        .or_else(|| WaveformFactory::create("LoRa", sample_rate))
        .expect("Failed to create LoRa waveform");

    let registry = create_default_registry();
    let mut device = registry.create(&device_uri)?;

    let center_freq = 915_000_000u64;

    match mode {
        Mode::Tx => run_tx(&mut *device, &*lora, center_freq, sample_rate, bandwidth, num_packets)?,
        Mode::Rx => run_rx(&mut *device, &*lora, center_freq, sample_rate, bandwidth, num_packets)?,
        Mode::Loopback => run_loopback(&mut *device, &*lora, center_freq, sample_rate, bandwidth, num_packets)?,
    }

    Ok(())
}

fn run_tx(
    device: &mut dyn SdrDeviceExt,
    lora: &dyn Waveform,
    center_freq: u64,
    sample_rate: f64,
    bandwidth: u32,
    num_packets: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("\n=== TX Mode ===\n");

    let tuner = device.tuner();
    tuner.set_frequency(center_freq)?;
    tuner.set_sample_rate(sample_rate)?;
    tuner.set_bandwidth(bandwidth as f64 * 1.2)?;
    tuner.set_tx_gain(25.0)?;

    let config = StreamConfig {
        direction: StreamDirection::Tx,
        channels: vec![0],
        buffer_size: 8192,
        ..Default::default()
    };

    let mut stream = device.create_tx_stream(config)?;
    let timeout = Duration::from_millis(100);

    println!("Transmitting {} packets...\n", num_packets);

    for seq in 0..num_packets {
        let message = format!("Packet {:04}", seq);
        let packet = Packet::new(seq as u16, message.as_bytes().to_vec());
        let packet_bytes = packet.to_bytes();

        // Modulate - Waveform::modulate takes &[u8]
        let samples = lora.modulate(&packet_bytes);

        // Add preamble and silence
        let preamble = vec![IQSample::new(0.5, 0.0); 512];
        let silence = vec![IQSample::new(0.0, 0.0); 256];

        stream.start()?;
        stream.write(&preamble, None, timeout)?;
        stream.write(&samples, None, timeout)?;
        stream.write(&silence, None, timeout)?;
        stream.stop()?;

        println!("TX #{}: \"{}\" ({} samples)", seq, message, samples.len());

        std::thread::sleep(Duration::from_millis(500));
    }

    println!("\nTransmission complete.");
    Ok(())
}

fn run_rx(
    device: &mut dyn SdrDeviceExt,
    lora: &dyn Waveform,
    center_freq: u64,
    sample_rate: f64,
    bandwidth: u32,
    num_packets: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("\n=== RX Mode ===\n");

    let tuner = device.tuner();
    tuner.set_frequency(center_freq)?;
    tuner.set_sample_rate(sample_rate)?;
    tuner.set_bandwidth(bandwidth as f64 * 1.2)?;
    tuner.set_rx_gain(40.0)?;

    let samples_per_symbol = lora.samples_per_symbol();

    let config = StreamConfig {
        direction: StreamDirection::Rx,
        channels: vec![0],
        buffer_size: samples_per_symbol * 4,
        ..Default::default()
    };

    let mut stream = device.create_rx_stream(config)?;
    let read_timeout = Duration::from_millis(100);

    println!("Waiting for {} packets...\n", num_packets);

    let mut received = 0u32;
    let mut errors = 0u32;
    let start = Instant::now();
    let timeout = Duration::from_secs(60);

    stream.start()?;

    while received + errors < num_packets && start.elapsed() < timeout {
        let mut buffer = vec![IQSample::new(0.0, 0.0); samples_per_symbol * 16];
        let (count, _) = stream.read(&mut buffer, read_timeout)?;

        if count > 0 {
            // Demodulate - returns DemodResult
            let result = lora.demodulate(&buffer[..count]);
            let bytes = result.bits;  // Vec<u8>

            if let Some(packet) = Packet::from_bytes(&bytes) {
                received += 1;
                if let Ok(msg) = String::from_utf8(packet.payload.clone()) {
                    println!("RX #{}: \"{}\" (seq={})", received, msg, packet.sequence);
                }
            } else if !bytes.is_empty() {
                errors += 1;
                println!("RX error: CRC failure");
            }
        }
    }

    stream.stop()?;

    println!("\n{}", "=".repeat(40));
    println!("Received: {}/{}", received, num_packets);
    println!("Errors: {}", errors);
    println!("PER: {:.1}%", 100.0 * errors as f64 / (received + errors).max(1) as f64);

    Ok(())
}

fn run_loopback(
    device: &mut dyn SdrDeviceExt,
    lora: &dyn Waveform,
    center_freq: u64,
    sample_rate: f64,
    bandwidth: u32,
    num_packets: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    println!("\n=== Loopback Mode ===\n");

    let tuner = device.tuner();
    tuner.set_frequency(center_freq)?;
    tuner.set_sample_rate(sample_rate)?;
    tuner.set_bandwidth(bandwidth as f64)?;
    tuner.set_tx_gain(20.0)?;
    tuner.set_rx_gain(40.0)?;

    let samples_per_symbol = lora.samples_per_symbol();

    let tx_config = StreamConfig {
        direction: StreamDirection::Tx,
        channels: vec![0],
        buffer_size: 8192,
        ..Default::default()
    };

    let rx_config = StreamConfig {
        direction: StreamDirection::Rx,
        channels: vec![0],
        buffer_size: samples_per_symbol * 4,
        ..Default::default()
    };

    let mut tx_stream = device.create_tx_stream(tx_config)?;
    let mut rx_stream = device.create_rx_stream(rx_config)?;
    let timeout = Duration::from_millis(100);

    let mut successes = 0u32;
    let mut failures = 0u32;

    for seq in 0..num_packets {
        let message = format!("Test{:02}", seq);
        let packet = Packet::new(seq as u16, message.as_bytes().to_vec());
        let packet_bytes = packet.to_bytes();

        // Modulate
        let tx_samples = lora.modulate(&packet_bytes);

        // Transmit
        tx_stream.start()?;
        tx_stream.write(&tx_samples, None, timeout)?;

        // Receive
        rx_stream.start()?;
        std::thread::sleep(Duration::from_millis(10));

        let mut rx_buffer = vec![IQSample::new(0.0, 0.0); tx_samples.len() * 2];
        let (count, _) = rx_stream.read(&mut rx_buffer, timeout)?;

        tx_stream.stop()?;
        rx_stream.stop()?;

        if count > 0 {
            // Demodulate - returns DemodResult
            let result = lora.demodulate(&rx_buffer[..count]);
            let rx_bytes = result.bits;

            if let Some(rx_packet) = Packet::from_bytes(&rx_bytes) {
                if rx_packet.payload == packet.payload {
                    successes += 1;
                    println!("Packet {}: OK", seq);
                } else {
                    failures += 1;
                    println!("Packet {}: MISMATCH", seq);
                }
            } else {
                failures += 1;
                println!("Packet {}: CRC ERROR", seq);
            }
        } else {
            failures += 1;
            println!("Packet {}: NO RX DATA", seq);
        }

        std::thread::sleep(Duration::from_millis(100));
    }

    println!("\n{}", "=".repeat(40));
    println!("LOOPBACK RESULTS");
    println!("{}", "=".repeat(40));
    println!("Sent: {}", num_packets);
    println!("Received correctly: {}", successes);
    println!("Failures: {}", failures);
    println!("Success rate: {:.1}%", 100.0 * successes as f64 / num_packets as f64);

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
