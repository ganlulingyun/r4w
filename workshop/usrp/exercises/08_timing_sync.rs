//! # Exercise 08: Timing Synchronization
//!
//! Learn about clock synchronization using PPS and GPSDO.
//!
//! ## Goals
//! - Understand time and frequency synchronization
//! - Configure external clock sources
//! - Synchronize multiple USRPs
//! - Measure timing accuracy
//!
//! ## Setup Options
//! 1. Internal clock (no external hardware needed)
//! 2. External 10 MHz reference
//! 3. GPS disciplined oscillator (GPSDO)
//! 4. PPS (Pulse Per Second) timing
//!
//! ## Run
//! ```bash
//! # Internal clock (default)
//! cargo run --example 08_timing_sync -- --simulator
//!
//! # External reference
//! cargo run --example 08_timing_sync -- --device "uhd://type=n210" --clock external
//!
//! # GPSDO
//! cargo run --example 08_timing_sync -- --device "uhd://type=n210" --clock gpsdo
//! ```
//!
//! trace:FR-0093 | ai:claude

use r4w_sim::hal::{
    create_default_registry,
    ClockControl, ClockSource,
};
use r4w_core::timing::{TimeSource, Timestamp};
use std::time::{Duration, Instant};

fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt::init();

    println!("=== R4W Timing Synchronization Exercise ===\n");

    // Parse arguments
    let args: Vec<String> = std::env::args().collect();

    let device_uri = get_arg(&args, &["--device", "-d"])
        .unwrap_or("simulator://".to_string());

    let clock_source = match get_arg(&args, &["--clock", "-c"]).as_deref() {
        Some("external") => ClockSource::External,
        Some("gpsdo") => ClockSource::Gpsdo,
        Some("mimo") => ClockSource::Mimo,
        _ => ClockSource::Internal,
    };

    let time_source = match get_arg(&args, &["--time", "-t"]).as_deref() {
        Some("external") => TimeSource::External,
        Some("gpsdo") | Some("gps") => TimeSource::Gps,
        Some("pps") => TimeSource::Pps,
        _ => TimeSource::Freerun,
    };

    println!("Device: {}", device_uri);
    println!("Clock source: {:?}", clock_source);
    println!("Time source: {:?}", time_source);

    // Create device
    let registry = create_default_registry();
    let mut device = registry.create(&device_uri)?;

    // Get clock control interface
    let clock = match device.clock() {
        Some(c) => c,
        None => {
            println!("ERROR: Device does not support clock control");
            return Ok(());
        }
    };

    // Display current status
    println!("\n=== Current Clock Status ===");
    println!("Clock source: {:?}", clock.clock_source());
    println!("Time source: {:?}", clock.time_source());
    if let Some(hwclock) = clock.hardware_clock() {
        println!("Hardware time: {:.6} s", hwclock.to_duration().as_secs_f64());
    }

    // Set clock source
    println!("\n=== Configuring Clock ===");

    match clock.set_clock_source(clock_source) {
        Ok(()) => println!("Clock source set to {:?}", clock_source),
        Err(e) => println!("Failed to set clock source: {} (using internal)", e),
    }

    match clock.set_time_source(time_source) {
        Ok(()) => println!("Time source set to {:?}", time_source),
        Err(e) => println!("Failed to set time source: {} (using freerun)", e),
    }

    // Wait for lock if using external reference
    if clock_source != ClockSource::Internal {
        println!("\nWaiting for reference lock...");
        let start = Instant::now();
        let timeout = Duration::from_secs(10);

        loop {
            if clock.is_locked() {
                println!("Reference locked!");
                break;
            }
            if start.elapsed() > timeout {
                println!("Warning: Reference lock timeout (continuing anyway)");
                break;
            }
            std::thread::sleep(Duration::from_millis(100));
            print!(".");
        }
        println!();
    }

    // If using PPS, wait for PPS lock
    if time_source == TimeSource::Pps || time_source == TimeSource::Gps {
        println!("\nWaiting for PPS lock...");
        let start = Instant::now();
        let timeout = Duration::from_secs(30);

        loop {
            // Check PPS lock via hardware clock
            let pps_locked = clock.hardware_clock()
                .map(|hw| hw.is_pps_locked())
                .unwrap_or(false);

            if pps_locked {
                println!("PPS locked!");
                break;
            }
            if start.elapsed() > timeout {
                println!("Warning: PPS lock timeout (continuing anyway)");
                break;
            }
            std::thread::sleep(Duration::from_millis(500));
            print!(".");
        }
        println!();
    }

    // Set initial time
    println!("\n=== Setting Time ===");

    // If using GPS, wait for GPS time
    if time_source == TimeSource::Gps {
        println!("Waiting for GPS time...");
        // In real implementation, would query GPS for current time
        std::thread::sleep(Duration::from_secs(2));
    }

    // Set time to 0 at next PPS edge (or immediately if no PPS)
    let pps_locked = clock.hardware_clock()
        .map(|hw| hw.is_pps_locked())
        .unwrap_or(false);

    // Create a zero timestamp (sample_rate doesn't matter for time setting)
    let zero_time = Timestamp::new(1e6);  // Use 1 MHz as placeholder sample rate

    if pps_locked {
        println!("Setting time to 0 at next PPS edge...");
        clock.set_time_at_pps(zero_time)?;
    } else {
        println!("Setting time to 0 now...");
        clock.set_time(zero_time)?;
    }

    // Monitor time progression
    println!("\n=== Time Monitoring ===");
    println!("Monitoring hardware time for 5 seconds...\n");

    let mut last_time = 0.0f64;
    let start = Instant::now();

    while start.elapsed() < Duration::from_secs(5) {
        if let Some(hwtime) = clock.hardware_clock() {
            let current = hwtime.to_duration().as_secs_f64();
            let drift = current - last_time - 0.1;  // Expected 100ms between samples

            if last_time > 0.0 {
                println!("HW Time: {:.6} s | Delta: {:.6} s | Drift: {:+.3} ms",
                         current, current - last_time, drift * 1000.0);
            } else {
                println!("HW Time: {:.6} s", current);
            }
            last_time = current;
        }
        std::thread::sleep(Duration::from_millis(100));
    }

    // Timing accuracy test
    println!("\n=== Timing Accuracy Test ===");
    test_timing_accuracy(clock)?;

    // Multi-device synchronization example
    println!("\n=== Multi-Device Sync (Conceptual) ===");
    print_multi_device_sync_info();

    println!("\n=== Exercise Complete ===");

    Ok(())
}

fn test_timing_accuracy(clock: &mut dyn ClockControl) -> Result<(), Box<dyn std::error::Error>> {
    let num_samples = 100;
    let expected_interval = Duration::from_millis(10);

    let mut intervals = Vec::new();
    let mut last_hw_time = None;

    for _ in 0..num_samples {
        if let Some(hwtime) = clock.hardware_clock() {
            let current_secs = hwtime.to_duration().as_secs_f64();
            if let Some(last) = last_hw_time {
                let interval = current_secs - last;
                intervals.push(interval);
            }
            last_hw_time = Some(current_secs);
        }
        std::thread::sleep(expected_interval);
    }

    if intervals.is_empty() {
        println!("Could not collect timing samples");
        return Ok(());
    }

    let mean: f64 = intervals.iter().sum::<f64>() / intervals.len() as f64;
    let variance: f64 = intervals.iter()
        .map(|&x| (x - mean).powi(2))
        .sum::<f64>() / intervals.len() as f64;
    let std_dev = variance.sqrt();
    let min = intervals.iter().cloned().fold(f64::INFINITY, f64::min);
    let max = intervals.iter().cloned().fold(f64::NEG_INFINITY, f64::max);

    println!("Samples: {}", intervals.len());
    println!("Expected interval: {:.3} ms", expected_interval.as_secs_f64() * 1000.0);
    println!("Mean interval: {:.3} ms", mean * 1000.0);
    println!("Std deviation: {:.3} ms", std_dev * 1000.0);
    println!("Min: {:.3} ms, Max: {:.3} ms", min * 1000.0, max * 1000.0);
    println!("Jitter (max-min): {:.3} ms", (max - min) * 1000.0);

    Ok(())
}

fn print_multi_device_sync_info() {
    println!("To synchronize multiple USRPs:");
    println!();
    println!("1. MIMO Cable (N-Series):");
    println!("   - Connect MIMO cable between devices");
    println!("   - Master: --clock internal --time pps");
    println!("   - Slave:  --clock mimo --time mimo");
    println!();
    println!("2. External 10 MHz + PPS:");
    println!("   - Distribute 10 MHz reference to all devices");
    println!("   - Distribute PPS signal to all devices");
    println!("   - All devices: --clock external --time pps");
    println!();
    println!("3. GPSDO:");
    println!("   - Install GPSDO in each device");
    println!("   - Wait for GPS lock on all devices");
    println!("   - All devices: --clock gpsdo --time gpsdo");
    println!();
    println!("4. Octoclock:");
    println!("   - Use Ettus Octoclock for 10 MHz and PPS distribution");
    println!("   - Supports up to 8 USRPs synchronized");
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
