//! SCHED_FIFO Jitter Measurement Benchmark (MF-012)
//!
//! Measures timing jitter when running with real-time scheduling.
//! Run with:
//!
//! ```bash
//! # Run with sudo for SCHED_FIFO support
//! sudo -E cargo bench -p r4w-core --bench rt_jitter_bench
//!
//! # Or without sudo (will fall back to normal scheduling)
//! cargo bench -p r4w-core --bench rt_jitter_bench
//! ```
//!
//! This benchmark measures:
//! - Timer precision and jitter
//! - Periodic task latency variation
//! - RT vs non-RT scheduling comparison
//! - CPU affinity impact on jitter

use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use r4w_core::rt::{LatencyHistogram, RtConfig, RtPriority, spawn_rt_thread};

/// Number of iterations for jitter measurement
const ITERATIONS: usize = 10000;

/// Target period in microseconds
const TARGET_PERIOD_US: u64 = 100; // 10 kHz

/// Run the jitter measurement and print results
fn main() {
    println!("=== SCHED_FIFO Jitter Benchmark (MF-012) ===\n");

    // Check if we have RT privileges
    let has_rt_privs = check_rt_privileges();
    if has_rt_privs {
        println!("✓ Running with real-time privileges (SCHED_FIFO available)");
    } else {
        println!("⚠ Running WITHOUT real-time privileges (run with sudo for best results)");
    }
    println!();

    // Run benchmarks
    println!("--- Normal Priority (baseline) ---");
    let normal_stats = measure_jitter(RtPriority::Normal, &[]);
    print_jitter_results("Normal", &normal_stats);
    println!();

    if has_rt_privs {
        println!("--- SCHED_FIFO Low Priority ---");
        let low_stats = measure_jitter(RtPriority::Low, &[]);
        print_jitter_results("SCHED_FIFO Low", &low_stats);
        println!();

        println!("--- SCHED_FIFO High Priority ---");
        let high_stats = measure_jitter(RtPriority::High, &[]);
        print_jitter_results("SCHED_FIFO High", &high_stats);
        println!();

        println!("--- SCHED_FIFO High + CPU Affinity ---");
        let affinity_stats = measure_jitter(RtPriority::High, &[0]);
        print_jitter_results("SCHED_FIFO + Affinity", &affinity_stats);
        println!();

        // Compare results
        println!("=== Jitter Comparison ===");
        println!(
            "Normal p99:      {:>8} ns",
            normal_stats.percentiles.p99_ns
        );
        println!(
            "SCHED_FIFO p99:  {:>8} ns  ({:.1}x improvement)",
            affinity_stats.percentiles.p99_ns,
            normal_stats.percentiles.p99_ns as f64 / affinity_stats.percentiles.p99_ns.max(1) as f64
        );
        println!();

        // Pass/fail criteria
        let p99_us = affinity_stats.percentiles.p99_ns / 1000;
        let target_us = TARGET_PERIOD_US;
        let jitter_pct = (p99_us as f64 / target_us as f64) * 100.0;

        println!("=== Jitter Assessment ===");
        println!("Target period: {}µs", target_us);
        println!("p99 jitter:    {}µs ({:.1}% of period)", p99_us, jitter_pct);

        if jitter_pct < 10.0 {
            println!("✓ PASS: Jitter < 10% of target period");
        } else if jitter_pct < 25.0 {
            println!("⚠ MARGINAL: Jitter 10-25% of target period");
        } else {
            println!("✗ FAIL: Jitter > 25% of target period");
        }
    } else {
        println!("(Skipping SCHED_FIFO tests - no privileges)");
    }
}

/// Check if we can use SCHED_FIFO
fn check_rt_privileges() -> bool {
    #[cfg(target_os = "linux")]
    {
        use std::mem;
        unsafe {
            let mut param: libc::sched_param = mem::zeroed();
            param.sched_priority = 1;

            // Try to set SCHED_FIFO on current thread
            let result = libc::sched_setscheduler(0, libc::SCHED_FIFO, &param);
            if result == 0 {
                // Restore normal scheduling
                param.sched_priority = 0;
                libc::sched_setscheduler(0, libc::SCHED_OTHER, &param);
                return true;
            }
        }
        false
    }

    #[cfg(not(target_os = "linux"))]
    false
}

/// Jitter statistics
struct JitterStats {
    histogram: LatencyHistogram,
    percentiles: r4w_core::rt::LatencyPercentiles,
    min_ns: u64,
    max_ns: u64,
    mean_ns: u64,
}

/// Measure jitter with specified RT configuration
fn measure_jitter(priority: RtPriority, cpu_affinity: &[usize]) -> JitterStats {
    let histogram = Arc::new(LatencyHistogram::new_us(10000, 1)); // 0-10ms, 1µs bins
    let hist_clone = histogram.clone();
    let stop = Arc::new(AtomicBool::new(false));
    let stop_clone = stop.clone();

    let affinity = cpu_affinity.to_vec();

    // Spawn the measurement thread
    let config = RtConfig::builder()
        .name("jitter_bench")
        .priority(priority)
        .cpu_affinity(&affinity)
        .lock_memory(priority.is_realtime())
        .build();

    let handle = spawn_rt_thread(config, move || {
        let target_ns = TARGET_PERIOD_US * 1000;

        // Warm up
        for _ in 0..100 {
            let start = Instant::now();
            busy_wait_ns(target_ns);
            let _ = start.elapsed();
        }

        // Measurement loop
        for i in 0..ITERATIONS {
            if stop_clone.load(Ordering::Relaxed) {
                break;
            }

            let start = Instant::now();

            // Simulate periodic work (target period)
            busy_wait_ns(target_ns / 2); // Do half the period of "work"

            // Measure actual elapsed time
            let elapsed = start.elapsed().as_nanos() as u64;

            // Record the jitter (deviation from target)
            let jitter = if elapsed > target_ns {
                elapsed - target_ns
            } else {
                target_ns - elapsed
            };
            hist_clone.record_ns(jitter);

            // Sleep for the other half (if needed)
            let remaining = target_ns.saturating_sub(elapsed);
            if remaining > 1000 {
                thread::sleep(Duration::from_nanos(remaining));
            }

            // Every 1000 iterations, check if we should stop
            if i % 1000 == 0 && i > 0 {
                if stop_clone.load(Ordering::Relaxed) {
                    break;
                }
            }
        }
    });

    // Wait for completion
    match handle {
        Ok(h) => {
            let _ = h.join();
        }
        Err(e) => {
            eprintln!("Failed to spawn RT thread: {}", e);
        }
    }

    stop.store(true, Ordering::Relaxed);

    let stats = histogram.statistics();
    JitterStats {
        histogram: LatencyHistogram::new_us(10000, 1), // Placeholder
        percentiles: stats.percentiles,
        min_ns: stats.min_ns,
        max_ns: stats.max_ns,
        mean_ns: stats.mean_ns,
    }
}

/// Busy-wait for specified nanoseconds (more accurate than sleep for short durations)
fn busy_wait_ns(target_ns: u64) {
    let start = Instant::now();
    while start.elapsed().as_nanos() < target_ns as u128 {
        std::hint::spin_loop();
    }
}

/// Print jitter results
fn print_jitter_results(name: &str, stats: &JitterStats) {
    println!("{} Jitter Statistics:", name);
    println!("  Min:      {:>8} ns ({:>5} µs)", stats.min_ns, stats.min_ns / 1000);
    println!("  Max:      {:>8} ns ({:>5} µs)", stats.max_ns, stats.max_ns / 1000);
    println!("  Mean:     {:>8} ns ({:>5} µs)", stats.mean_ns, stats.mean_ns / 1000);
    println!("  p50:      {:>8} ns ({:>5} µs)", stats.percentiles.p50_ns, stats.percentiles.p50_ns / 1000);
    println!("  p90:      {:>8} ns ({:>5} µs)", stats.percentiles.p90_ns, stats.percentiles.p90_ns / 1000);
    println!("  p99:      {:>8} ns ({:>5} µs)", stats.percentiles.p99_ns, stats.percentiles.p99_ns / 1000);
    println!("  p99.9:    {:>8} ns ({:>5} µs)", stats.percentiles.p999_ns, stats.percentiles.p999_ns / 1000);
}
