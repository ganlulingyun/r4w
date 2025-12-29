//! # Exercise 30: Symbol Timing Recovery
//!
//! Learn how to recover symbol timing from received signals.
//!
//! ## Topics
//! - Why timing recovery is needed
//! - Early-late gate timing
//! - Gardner timing error detector
//! - Mueller-Muller algorithm
//!
//! ## Run
//! ```bash
//! cargo run --example 30_symbol_timing
//! ```
//!
//! trace:FR-0095 | ai:claude

use std::f32::consts::PI;

fn main() {
    println!("=== Symbol Timing Recovery Workshop ===\n");

    // Part 1: The timing problem
    println!("== Part 1: The Timing Problem ==\n");
    explain_timing_problem();

    // Part 2: Oversampling
    println!("\n== Part 2: Oversampling ==\n");
    demonstrate_oversampling();

    // Part 3: Early-Late Gate
    println!("\n== Part 3: Early-Late Gate Detector ==\n");
    demonstrate_early_late();

    // Part 4: Mueller-Muller
    println!("\n== Part 4: Mueller-Muller Algorithm ==\n");
    demonstrate_mueller_muller();

    // Part 5: Timing recovery in action
    println!("\n== Part 5: Complete Timing Recovery ==\n");
    timing_recovery_demo();

    println!("\n=== Workshop Complete ===");
}

fn explain_timing_problem() {
    println!("Problem: We receive continuous samples but need to know");
    println!("         exactly WHEN to sample for best decision.");
    println!();
    println!("TX: Symbols at times 0, T, 2T, 3T, ...");
    println!("RX: Samples at unknown offset τ from symbol centers");
    println!();
    println!("If we sample at wrong time:");
    println!("  - Inter-Symbol Interference (ISI)");
    println!("  - Reduced noise margin");
    println!("  - Higher bit error rate");
    println!();
    println!("Example (BPSK, 4 samples/symbol):");
    println!();

    let samples_per_symbol = 4;
    let symbols = [1.0, -1.0, 1.0, 1.0, -1.0];

    // Perfect timing
    println!("Perfect timing (sample at center):");
    print!("  ");
    for &sym in &symbols {
        print!("{:6.2} ", sym);
    }
    println!();

    // Offset timing
    let offset = 1.5;  // 1.5 samples off
    println!("\nOffset timing (+1.5 samples):");
    print!("  ");
    for i in 0..symbols.len() {
        let idx = (i * samples_per_symbol) as f32 + offset;
        let left_idx = idx.floor() as usize;
        let right_idx = left_idx + 1;
        let frac = idx - left_idx as f32;

        // Linear interpolation between symbols
        let left_sym = symbols.get(left_idx / samples_per_symbol).copied().unwrap_or(0.0);
        let right_sym = symbols.get(right_idx / samples_per_symbol).copied().unwrap_or(0.0);
        let sampled = left_sym * (1.0 - frac) + right_sym * frac;

        print!("{:6.2} ", sampled);
    }
    println!("  (reduced margins)");
}

fn demonstrate_oversampling() {
    println!("Solution: Oversample and pick best sample point");
    println!();
    println!("TX symbol rate: 1000 symbols/sec");
    println!("RX sample rate: 4000 samples/sec (4x oversampling)");
    println!();

    let samples_per_symbol = 4;
    let symbols = [1.0f32, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0];

    // Generate oversampled signal (rectangular pulses)
    let mut samples = Vec::new();
    for &sym in &symbols {
        for _ in 0..samples_per_symbol {
            samples.push(sym);
        }
    }

    println!("Oversampled signal (4 samples per symbol):");
    for (i, chunk) in samples.chunks(samples_per_symbol).enumerate() {
        print!("Symbol {}: ", i);
        for &s in chunk {
            let bar = if s > 0.0 { "█" } else { "░" };
            print!("{}", bar);
        }
        println!(" ({:+.1})", chunk[samples_per_symbol / 2]);
    }

    println!("\nBest sample point: center of each symbol period");
    println!("Indices: 1, 5, 9, 13, ... (every {} samples)", samples_per_symbol);
}

fn demonstrate_early_late() {
    println!("Early-Late Gate Timing Error Detector");
    println!();
    println!("Idea: Compare samples before and after the decision point");
    println!("      If early > late: timing is too early, delay");
    println!("      If late > early: timing is too late, advance");
    println!();

    let samples_per_symbol = 8;

    // Generate test signal with known offset
    let symbols = [1.0, -1.0, 1.0, -1.0];
    let samples = generate_filtered_signal(&symbols, samples_per_symbol);

    // Test at different timing offsets
    println!("{:>8} {:>10} {:>10} {:>10} {:>12}",
             "Offset", "Early", "On-time", "Late", "Error");
    println!("{}", "-".repeat(55));

    for offset in -3..=3 {
        let (early, on_time, late, error) =
            early_late_error(&samples, samples_per_symbol, offset);
        println!("{:>8} {:>10.3} {:>10.3} {:>10.3} {:>12.3}",
                 offset, early, on_time, late, error);
    }

    println!("\nNote: Error = |early|² - |late|²");
    println!("      Positive error → sample later");
    println!("      Negative error → sample earlier");
}

fn generate_filtered_signal(symbols: &[f32], samples_per_symbol: usize) -> Vec<f32> {
    // Simple raised cosine-like pulse
    let mut samples = Vec::new();

    for &sym in symbols {
        for i in 0..samples_per_symbol {
            let t = (i as f32 / samples_per_symbol as f32 - 0.5) * 2.0 * PI;
            let pulse = (1.0 + t.cos()) / 2.0;  // Raised cosine window
            samples.push(sym * pulse);
        }
    }

    samples
}

fn early_late_error(samples: &[f32], sps: usize, offset: i32) -> (f32, f32, f32, f32) {
    let center = sps / 2;
    let early_idx = (center as i32 + offset - 1).max(0) as usize;
    let on_time_idx = (center as i32 + offset).max(0) as usize;
    let late_idx = (center as i32 + offset + 1).max(0) as usize;

    // Average over symbols
    let num_symbols = samples.len() / sps;
    let mut early_sum = 0.0;
    let mut on_time_sum = 0.0;
    let mut late_sum = 0.0;

    for sym in 0..num_symbols {
        let base = sym * sps;
        early_sum += samples.get(base + early_idx).copied().unwrap_or(0.0).abs();
        on_time_sum += samples.get(base + on_time_idx).copied().unwrap_or(0.0).abs();
        late_sum += samples.get(base + late_idx).copied().unwrap_or(0.0).abs();
    }

    let early = early_sum / num_symbols as f32;
    let on_time = on_time_sum / num_symbols as f32;
    let late = late_sum / num_symbols as f32;
    let error = early * early - late * late;

    (early, on_time, late, error)
}

fn demonstrate_mueller_muller() {
    println!("Mueller-Muller Timing Error Detector");
    println!();
    println!("Idea: Uses decision values to compute timing error");
    println!("      e(k) = x(k-1)·d(k) - x(k)·d(k-1)");
    println!();
    println!("Where:");
    println!("  x(k) = received sample at time k");
    println!("  d(k) = decision (detected symbol) at time k");
    println!();
    println!("Advantages over Early-Late:");
    println!("  - Works with 1 sample/symbol");
    println!("  - Better noise immunity");
    println!("  - Commonly used in digital receivers");
    println!();

    // Example computation
    let samples = [0.9, -0.95, 1.05, -0.85, 0.92];
    let decisions = [1.0, -1.0, 1.0, -1.0, 1.0];

    println!("Sample computation:");
    println!("{:>4} {:>10} {:>10} {:>10}", "k", "x(k)", "d(k)", "e(k)");
    println!("{}", "-".repeat(40));

    for k in 1..samples.len() {
        let error = samples[k - 1] * decisions[k] - samples[k] * decisions[k - 1];
        println!("{:>4} {:>10.3} {:>10.1} {:>10.3}", k, samples[k], decisions[k], error);
    }
}

fn timing_recovery_demo() {
    println!("Complete Timing Recovery Loop");
    println!();

    let samples_per_symbol = 4;
    let symbols: Vec<f32> = vec![1.0, -1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0, -1.0];

    // Add timing offset to received signal
    let timing_offset = 1.3;  // Fractional sample offset
    let rx_samples = generate_offset_signal(&symbols, samples_per_symbol, timing_offset);

    println!("True timing offset: {:.1} samples", timing_offset);
    println!("Symbols: {:?}", symbols);
    println!();

    // Run timing recovery
    let mut mu = 0.0f32;  // Timing estimate (fractional)
    let mut timing_errors = Vec::new();
    let mut recovered_symbols = Vec::new();
    let gain = 0.1;  // Loop gain

    let mut sample_idx = samples_per_symbol / 2;  // Start at expected center

    while sample_idx < rx_samples.len() - samples_per_symbol {
        // Interpolate at current timing estimate
        let interp_idx = sample_idx as f32 + mu;
        let sample = interpolate(&rx_samples, interp_idx);

        // Make decision
        let decision = if sample >= 0.0 { 1.0 } else { -1.0 };
        recovered_symbols.push(decision);

        // Compute timing error (simplified early-late)
        let early = interpolate(&rx_samples, interp_idx - 1.0).abs();
        let late = interpolate(&rx_samples, interp_idx + 1.0).abs();
        let error = early - late;
        timing_errors.push(mu);

        // Update timing estimate
        mu += gain * error;

        // Move to next symbol
        sample_idx += samples_per_symbol;
    }

    println!("Timing recovery convergence:");
    for (i, &err) in timing_errors.iter().enumerate() {
        let bar_len = ((err.abs() / 2.0) * 20.0) as usize;
        let bar = "█".repeat(bar_len.min(20));
        println!("Symbol {:2}: μ={:+.3} |{}", i, err, bar);
    }

    let correct = symbols.iter()
        .zip(recovered_symbols.iter())
        .filter(|(a, b)| (**a > 0.0) == (**b > 0.0))
        .count();

    println!("\nRecovered {} / {} symbols correctly",
             correct, recovered_symbols.len().min(symbols.len()));
}

fn generate_offset_signal(symbols: &[f32], sps: usize, offset: f32) -> Vec<f32> {
    let total_samples = symbols.len() * sps + (sps * 2);  // Extra for interpolation
    let mut samples = vec![0.0f32; total_samples];

    for (sym_idx, &sym) in symbols.iter().enumerate() {
        let center = (sym_idx * sps + sps / 2) as f32 + offset;
        // Simple pulse
        for i in 0..sps {
            let sample_pos = sym_idx * sps + i;
            if sample_pos < samples.len() {
                let t = sample_pos as f32 - center;
                let pulse = (-t * t / (sps as f32 / 2.0)).exp();
                samples[sample_pos] = sym * pulse;
            }
        }
    }

    samples
}

fn interpolate(samples: &[f32], idx: f32) -> f32 {
    let left = idx.floor() as usize;
    let right = left + 1;
    let frac = idx - left as f32;

    if right >= samples.len() {
        return samples.last().copied().unwrap_or(0.0);
    }

    samples[left] * (1.0 - frac) + samples[right] * frac
}
