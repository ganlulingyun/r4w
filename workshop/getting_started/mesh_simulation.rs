//! # Mesh Network Simulation Example
//!
//! Demonstrates mesh networking with Meshtastic protocol.
//!
//! Run with: cargo run --example mesh_simulation

use r4w_core::mesh::{MeshSimulator, SimConfig};

fn main() {
    println!("=== Mesh Network Simulation ===\n");

    // Configure the simulation
    let config = SimConfig::default()
        .with_node_count(5)
        .with_area(1000.0, 1000.0)  // 1km x 1km
        .with_seed(42)
        .with_verbose(false);

    println!("Simulation Configuration:");
    println!("  Nodes: {}", config.node_count);
    println!("  Area: {:.0}m x {:.0}m", config.area_width, config.area_height);
    println!("  Modem: {:?}", config.modem_preset);
    println!("  Region: {:?}", config.region);
    println!();

    // Create simulator
    let mut sim = MeshSimulator::new(config);

    // Print initial node positions
    println!("Node Positions:");
    for i in 0..5 {
        if let Some(pos) = sim.node_position(i) {
            println!("  Node {}: ({:.0}, {:.0})", i, pos.x, pos.y);
        }
    }
    println!();

    // Send a message from node 0 to node 4 (may require multi-hop)
    println!("=== Sending Message ===");
    println!("From: Node 0");
    println!("To: Node 4");
    println!("Message: \"Hello from the mesh!\"");
    println!();

    let sent = sim.send_message(0, "Hello from the mesh!", Some(4));
    if sent {
        println!("Message queued for transmission");
    } else {
        println!("Failed to queue message");
    }

    // Run simulation for 100 steps
    println!("\nRunning {} simulation steps...", 100);
    sim.run(100);

    // Get statistics
    let stats = sim.stats();

    println!("\n=== Simulation Results ===");
    println!("  Messages sent: {}", stats.messages_sent);
    println!("  Messages delivered: {}", stats.messages_delivered);
    println!("  Delivery rate: {:.1}%", stats.delivery_rate() * 100.0);
    println!("  Packets transmitted: {}", stats.packets_transmitted);
    println!("  Packets received: {}", stats.packets_received);
    println!("  Collisions: {}", stats.collisions);
    println!("  Packets lost: {}", stats.packets_lost);
    if stats.avg_hops > 0.0 {
        println!("  Average hops: {:.1}", stats.avg_hops);
    }
    if stats.avg_latency_ms > 0.0 {
        println!("  Average latency: {:.1} ms", stats.avg_latency_ms);
    }

    println!("\n=== Mesh Networking Concepts ===");
    println!("- Multi-hop: Messages relay through intermediate nodes");
    println!("- Flood routing: Broadcast with hop limit prevents loops");
    println!("- CSMA/CA: Carrier sensing prevents collisions");
    println!("- Path loss: Signal strength decreases with distance");
}
