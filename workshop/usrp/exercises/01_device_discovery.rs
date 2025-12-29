//! # Exercise 01: Device Discovery
//!
//! Learn how to discover and enumerate USRP devices on your system.
//!
//! ## Goals
//! - Understand the R4W driver registry
//! - Discover connected USRP devices
//! - Query device capabilities
//!
//! ## Run
//! ```bash
//! cargo run --example 01_device_discovery
//! ```
//!
//! trace:FR-0093 | ai:claude

use r4w_sim::hal::create_default_registry;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    // Initialize logging
    tracing_subscriber::fmt::init();

    println!("=== R4W USRP Device Discovery ===\n");

    // Create the driver registry with all built-in drivers
    let registry = create_default_registry();

    // List available drivers
    println!("Available drivers:");
    for driver in registry.list() {
        println!("  - {}", driver);
    }
    println!();

    // Discover devices across all drivers
    println!("Discovering devices...\n");

    let devices = registry.discover_all();

    if devices.is_empty() {
        println!("No devices found.");
        println!("\nTroubleshooting:");
        println!("  - For B200: Check USB connection, verify UHD is installed");
        println!("  - For N210: Check network connection, ping 192.168.10.2");
        println!("  - Run 'uhd_find_devices' to verify UHD can see the device");
        println!("\nTo test with the simulator:");
        println!("  cargo run --example 01_device_discovery -- --simulator");
    } else {
        println!("Found {} device(s):\n", devices.len());

        for (i, (driver_name, info)) in devices.iter().enumerate() {
            println!("Device {}:", i + 1);
            println!("  Driver: {}", driver_name);
            println!("  Type: {}", info.driver);
            println!("  Label: {}", info.label);
            println!("  Serial: {}", if info.serial.is_empty() { "N/A" } else { &info.serial });
            println!("  Address: {}", if info.address.is_empty() { "N/A" } else { &info.address });
            println!();
        }
    }

    // Example: Create a device connection
    // Uncomment to actually connect to a device

    /*
    println!("Connecting to first device...");

    // Using the simulator for testing
    let mut device = registry.create("simulator://")?;

    // Or connect to a real USRP:
    // let mut device = registry.create("uhd://type=b200")?;
    // let mut device = registry.create("uhd://addr=192.168.10.2")?;

    let caps = device.capabilities();
    println!("\nDevice capabilities:");
    println!("  Can TX: {}", caps.can_tx);
    println!("  Can RX: {}", caps.can_rx);
    println!("  Full Duplex: {}", caps.full_duplex);
    println!("  Frequency Range: {:.0} - {:.0} MHz",
             caps.min_frequency / 1e6,
             caps.max_frequency / 1e6);
    println!("  Max Sample Rate: {:.1} MS/s", caps.max_sample_rate / 1e6);
    println!("  TX Channels: {}", caps.tx_channels);
    println!("  RX Channels: {}", caps.rx_channels);
    */

    println!("=== Exercise Complete ===");

    Ok(())
}

// Helper function to parse command line arguments
#[allow(dead_code)]
fn parse_args() -> (bool, Option<String>) {
    let args: Vec<String> = std::env::args().collect();
    let mut use_simulator = false;
    let mut device_uri = None;

    for i in 1..args.len() {
        match args[i].as_str() {
            "--simulator" | "-s" => use_simulator = true,
            "--device" | "-d" => {
                if i + 1 < args.len() {
                    device_uri = Some(args[i + 1].clone());
                }
            }
            _ => {}
        }
    }

    (use_simulator, device_uri)
}
