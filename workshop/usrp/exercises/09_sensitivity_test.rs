//! # Exercise 09: Automated Sensitivity Testing
//!
//! Use a digital attenuator to perform automated sensitivity measurements.
//!
//! ## Setup
//! ```
//! USRP TX ──► Attenuator ──► USRP RX (or same USRP in loopback)
//! ```
//!
//! ## Goals
//! - Understand attenuator control
//! - Perform automated SNR sweeps
//! - Find receiver sensitivity threshold
//! - Generate test reports
//!
//! ## Run
//! ```bash
//! # With simulated attenuator (no hardware needed)
//! cargo run --example 09_sensitivity_test -- --simulator
//!
//! # With real attenuator
//! cargo run --example 09_sensitivity_test -- --attenuator "minicircuits://RCDAT-8000-90"
//! ```
//!
//! trace:FR-0094 | ai:claude

use r4w_sim::hal::{
    create_attenuator, AttenuatorTestHarness,
};
use std::time::Duration;

/// Simulated packet transmission result
#[allow(dead_code)]
struct PacketResult {
    success: bool,
    snr_estimate: f64,
}

/// Test results for a single attenuation level
#[derive(Debug, Clone)]
struct TestPoint {
    attenuation_db: f64,
    estimated_snr_db: f64,
    packets_sent: usize,
    packets_received: usize,
    per: f64, // Packet Error Rate
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logging
    tracing_subscriber::fmt::init();

    println!("=== R4W Sensitivity Test with Digital Attenuator ===\n");

    // Parse command line arguments
    let (_use_simulator, attenuator_uri) = parse_args();

    // Create attenuator
    let uri = attenuator_uri.unwrap_or_else(|| "simulated://max=90".to_string());
    println!("Creating attenuator: {}", uri);

    let attenuator = create_attenuator(&uri)?;

    println!("Attenuator: {}", attenuator.name());
    let caps = attenuator.capabilities();
    println!("  Range: {} - {} dB", caps.min_attenuation, caps.max_attenuation);
    println!("  Step size: {} dB", caps.step_size);
    println!();

    // Create test harness
    let mut harness = AttenuatorTestHarness::new(attenuator);

    // Set SNR estimation parameters
    harness.set_tx_power(0.0);      // 0 dBm TX power
    harness.set_noise_floor(-100.0); // -100 dBm noise floor
    harness.set_settle_time(Duration::from_millis(10));

    // Test parameters
    let sweep_start = 0.0;
    let sweep_end = 60.0;
    let sweep_step = 2.0;
    let packets_per_point = 100;
    let per_threshold = 0.10; // 10% PER = sensitivity threshold

    println!("Test Configuration:");
    println!("  Attenuation sweep: {} to {} dB (step {})", sweep_start, sweep_end, sweep_step);
    println!("  Packets per point: {}", packets_per_point);
    println!("  PER threshold: {}%", per_threshold * 100.0);
    println!();

    // Run the sweep
    println!("Running sensitivity sweep...\n");
    println!("{:>10} {:>10} {:>8} {:>8} {:>10}",
             "Atten(dB)", "SNR(dB)", "TX", "RX", "PER(%)");
    println!("{}", "-".repeat(50));

    let mut results: Vec<TestPoint> = Vec::new();
    let mut sensitivity_found = false;
    let mut sensitivity_atten = 0.0;
    let mut sensitivity_snr = 0.0;

    let sweep_values = AttenuatorTestHarness::generate_sweep(sweep_start, sweep_end, sweep_step);

    for atten in sweep_values {
        // Set attenuation
        let actual_atten = harness.set_attenuation(atten)?;
        let estimated_snr = harness.calculate_snr(actual_atten);

        // Simulate packet transmission
        // In a real test, this would use the USRP to TX/RX packets
        let mut packets_received = 0;
        for _ in 0..packets_per_point {
            let result = simulate_packet_transmission(estimated_snr);
            if result.success {
                packets_received += 1;
            }
        }

        let per = 1.0 - (packets_received as f64 / packets_per_point as f64);

        let point = TestPoint {
            attenuation_db: actual_atten,
            estimated_snr_db: estimated_snr,
            packets_sent: packets_per_point,
            packets_received,
            per,
        };

        println!("{:>10.1} {:>10.1} {:>8} {:>8} {:>10.1}",
                 point.attenuation_db,
                 point.estimated_snr_db,
                 point.packets_sent,
                 point.packets_received,
                 point.per * 100.0);

        results.push(point.clone());

        // Check if we've crossed the sensitivity threshold
        if !sensitivity_found && per >= per_threshold {
            sensitivity_found = true;
            sensitivity_atten = actual_atten;
            sensitivity_snr = estimated_snr;
            println!("\n*** Sensitivity threshold reached at {} dB attenuation ***\n", actual_atten);
        }

        // Optionally stop early once we're well past the threshold
        if per > 0.5 {
            println!("\n(Stopping early - PER > 50%)");
            break;
        }
    }

    // Print summary
    println!("\n{}", "=".repeat(50));
    println!("RESULTS SUMMARY");
    println!("{}", "=".repeat(50));

    if sensitivity_found {
        println!("\nSensitivity (at {}% PER):", per_threshold * 100.0);
        println!("  Attenuation: {:.1} dB", sensitivity_atten);
        println!("  Estimated SNR: {:.1} dB", sensitivity_snr);
        println!("  Estimated RX Power: {:.1} dBm", -sensitivity_atten);
    } else {
        println!("\nSensitivity threshold not reached in sweep range.");
        println!("Receiver may have better sensitivity than test range allows.");
    }

    // Find the attenuation where PER was 0
    let perfect_points: Vec<_> = results.iter().filter(|p| p.per == 0.0).collect();
    if let Some(last_perfect) = perfect_points.last() {
        println!("\nLast point with 0% PER:");
        println!("  Attenuation: {:.1} dB", last_perfect.attenuation_db);
        println!("  SNR: {:.1} dB", last_perfect.estimated_snr_db);
    }

    // Link margin calculation
    if sensitivity_found && !perfect_points.is_empty() {
        let operating_point = perfect_points[perfect_points.len() / 2]; // Mid-point of good range
        let margin = sensitivity_atten - operating_point.attenuation_db;
        println!("\nLink Margin (from mid-point to sensitivity):");
        println!("  {:.1} dB", margin);
    }

    println!("\n=== Test Complete ===");

    Ok(())
}

/// Simulate packet transmission at a given SNR
///
/// In a real implementation, this would:
/// 1. Modulate the packet with the waveform
/// 2. Transmit via USRP TX
/// 3. Receive via USRP RX
/// 4. Demodulate and check CRC
fn simulate_packet_transmission(snr_db: f64) -> PacketResult {
    // Simple model: probability of success based on SNR
    // This is a rough approximation - real results depend on waveform, coding, etc.

    let success_probability = if snr_db > 20.0 {
        1.0
    } else if snr_db > 10.0 {
        0.99
    } else if snr_db > 5.0 {
        0.95
    } else if snr_db > 0.0 {
        0.8
    } else if snr_db > -5.0 {
        0.5
    } else if snr_db > -10.0 {
        0.2
    } else {
        0.01
    };

    // Random success based on probability
    let success = rand::random::<f64>() < success_probability;

    PacketResult {
        success,
        snr_estimate: snr_db,
    }
}

/// Parse command line arguments
fn parse_args() -> (bool, Option<String>) {
    let args: Vec<String> = std::env::args().collect();
    let mut use_simulator = false;
    let mut attenuator_uri = None;

    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--simulator" | "-s" => use_simulator = true,
            "--attenuator" | "-a" => {
                if i + 1 < args.len() {
                    attenuator_uri = Some(args[i + 1].clone());
                    i += 1;
                }
            }
            "--help" | "-h" => {
                println!("Usage: 09_sensitivity_test [OPTIONS]");
                println!();
                println!("Options:");
                println!("  -s, --simulator           Use simulated attenuator");
                println!("  -a, --attenuator URI      Attenuator connection string");
                println!("  -h, --help                Show this help");
                println!();
                println!("Examples:");
                println!("  --attenuator simulated://max=90");
                println!("  --attenuator minicircuits://RCDAT-8000-90");
                println!("  --attenuator pe43711://spi:0:0");
                std::process::exit(0);
            }
            _ => {}
        }
        i += 1;
    }

    (use_simulator, attenuator_uri)
}

// Simple random number for simulation (no external dependency)
mod rand {
    use std::time::{SystemTime, UNIX_EPOCH};

    static mut SEED: u64 = 0;

    pub fn random<T: FromRandom>() -> T {
        T::from_random(next_u64())
    }

    fn next_u64() -> u64 {
        unsafe {
            if SEED == 0 {
                SEED = SystemTime::now()
                    .duration_since(UNIX_EPOCH)
                    .unwrap()
                    .as_nanos() as u64;
            }
            // Simple LCG
            SEED = SEED.wrapping_mul(6364136223846793005).wrapping_add(1);
            SEED
        }
    }

    pub trait FromRandom {
        fn from_random(val: u64) -> Self;
    }

    impl FromRandom for f64 {
        fn from_random(val: u64) -> Self {
            (val as f64) / (u64::MAX as f64)
        }
    }
}
