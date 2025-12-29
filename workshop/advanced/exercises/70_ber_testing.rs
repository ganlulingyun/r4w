//! # Exercise 70: Bit Error Rate Testing
//!
//! Learn to measure and analyze BER - the fundamental performance metric.
//!
//! ## Topics
//! - BER measurement methodology
//! - Statistical confidence intervals
//! - BER vs SNR curves
//! - Comparing theoretical to measured
//!
//! ## Run
//! ```bash
//! cargo run --example 70_ber_testing
//! ```
//!
//! trace:FR-0095 | ai:claude

use r4w_core::IQSample;

fn main() {
    println!("=== BER Testing Workshop ===\n");

    // Part 1: Basic BER measurement
    println!("== Part 1: BER Measurement Basics ==\n");
    basic_ber_measurement();

    // Part 2: Statistical confidence
    println!("\n== Part 2: Statistical Confidence ==\n");
    statistical_confidence();

    // Part 3: BER vs SNR curve
    println!("\n== Part 3: BER vs SNR Curve ==\n");
    ber_vs_snr_curve();

    // Part 4: Theoretical vs measured
    println!("\n== Part 4: Theory vs Practice ==\n");
    theory_vs_practice();

    println!("\n=== Workshop Complete ===");
}

fn basic_ber_measurement() {
    println!("BER = Number of bit errors / Total bits transmitted");
    println!();

    // Generate random test pattern
    let num_bits = 10000;
    let bits = generate_prbs(num_bits, 0x1234);

    // Simulate different error rates
    let error_rates = [0.0, 0.001, 0.01, 0.1];

    println!("{:>12} {:>12} {:>12} {:>12}",
             "True Rate", "Errors", "Measured", "Difference");
    println!("{}", "-".repeat(52));

    for &rate in &error_rates {
        let errors = introduce_errors(&bits, rate);
        let measured_ber = errors as f64 / num_bits as f64;
        let diff = (measured_ber - rate as f64).abs();

        println!("{:>12.4} {:>12} {:>12.4} {:>12.6}",
                 rate, errors, measured_ber, diff);
    }
}

fn statistical_confidence() {
    println!("For reliable BER measurement, need sufficient bit errors.");
    println!();
    println!("Rule of thumb: Need ~100 errors for 10% confidence interval");
    println!();

    println!("{:>12} {:>15} {:>18}",
             "Target BER", "Min Bits", "95% CI Width");
    println!("{}", "-".repeat(48));

    let target_bers = [1e-3, 1e-4, 1e-5, 1e-6];
    let target_errors = 100;  // For ~10% accuracy

    for &ber in &target_bers {
        let min_bits = (target_errors as f64 / ber) as u64;
        // Wilson score interval approximation
        let ci_width = 2.0 * (ber * (1.0 - ber) / min_bits as f64).sqrt() * 1.96;

        println!("{:>12.0e} {:>15} {:>18.2e}",
                 ber, min_bits, ci_width);
    }

    println!();
    println!("Example: To measure BER=1e-6 with confidence, need >100M bits!");
}

fn ber_vs_snr_curve() {
    println!("Testing BPSK at different SNR levels:");
    println!();

    println!("{:>8} {:>12} {:>12} {:>12}",
             "Eb/N0", "Theoretical", "Measured", "Difference");
    println!("{}", "-".repeat(48));

    let bits_per_point = 100000;
    let snr_values = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0];

    for &snr_db in &snr_values {
        // Generate test data
        let bits = generate_prbs(bits_per_point, 42);

        // Modulate (BPSK)
        let symbols: Vec<IQSample> = bits.iter()
            .map(|&b| if b { IQSample::new(1.0, 0.0) } else { IQSample::new(-1.0, 0.0) })
            .collect();

        // Add noise
        let noisy = add_awgn(&symbols, snr_db);

        // Demodulate
        let decoded: Vec<bool> = noisy.iter()
            .map(|s| s.re > 0.0)
            .collect();

        // Count errors
        let errors = bits.iter().zip(decoded.iter())
            .filter(|(a, b)| a != b)
            .count();

        let measured_ber = errors as f64 / bits_per_point as f64;
        let theoretical_ber = bpsk_theoretical_ber(snr_db);
        let diff = (measured_ber - theoretical_ber).abs();

        println!("{:>8.1} {:>12.2e} {:>12.2e} {:>12.2e}",
                 snr_db, theoretical_ber, measured_ber, diff);
    }

    // ASCII plot
    println!("\nBER vs Eb/N0 (BPSK):");
    println!();
    print_ber_curve();
}

fn theory_vs_practice() {
    println!("Reasons for measured BER to differ from theory:");
    println!();
    println!("1. Implementation Loss");
    println!("   - Non-ideal filters");
    println!("   - Quantization effects");
    println!("   - Timing jitter");
    println!();
    println!("2. Channel Effects");
    println!("   - Non-AWGN noise (interference, phase noise)");
    println!("   - Multipath fading");
    println!("   - Doppler shifts");
    println!();
    println!("3. Synchronization Errors");
    println!("   - Carrier frequency offset");
    println!("   - Symbol timing errors");
    println!("   - Phase ambiguity");
    println!();
    println!("4. Measurement Errors");
    println!("   - Insufficient samples");
    println!("   - SNR estimation errors");
    println!("   - Test pattern issues");
}

fn generate_prbs(length: usize, seed: u64) -> Vec<bool> {
    let mut state = seed;
    (0..length)
        .map(|_| {
            state = state.wrapping_mul(1103515245).wrapping_add(12345);
            (state >> 16) & 1 == 1
        })
        .collect()
}

fn introduce_errors(bits: &[bool], error_rate: f32) -> usize {
    let mut seed = 0x5678u64;
    bits.iter()
        .filter(|_| {
            seed = seed.wrapping_mul(1103515245).wrapping_add(12345);
            (seed as f32 / u64::MAX as f32) < error_rate
        })
        .count()
}

fn add_awgn(symbols: &[IQSample], snr_db: f32) -> Vec<IQSample> {
    let snr_linear = 10.0_f64.powf(snr_db as f64 / 10.0);
    let noise_std = (1.0 / (2.0 * snr_linear)).sqrt();

    let mut seed = 0xABCDu64;
    let mut next_gaussian = || -> f64 {
        seed = seed.wrapping_mul(1103515245).wrapping_add(12345);
        let u1 = (seed as f64 / u64::MAX as f64).max(1e-10);
        seed = seed.wrapping_mul(1103515245).wrapping_add(12345);
        let u2 = seed as f64 / u64::MAX as f64;
        (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
    };

    symbols.iter()
        .map(|s| {
            IQSample::new(
                s.re + next_gaussian() * noise_std,
                s.im + next_gaussian() * noise_std,
            )
        })
        .collect()
}

fn bpsk_theoretical_ber(snr_db: f32) -> f64 {
    let snr_linear = 10.0_f64.powf(snr_db as f64 / 10.0);
    // BER = 0.5 * erfc(sqrt(Eb/N0))
    // Approximation of erfc
    0.5 * erfc((snr_linear).sqrt())
}

fn erfc(x: f64) -> f64 {
    // Approximation of complementary error function
    let t = 1.0 / (1.0 + 0.5 * x.abs());
    let tau = t * (-x * x - 1.26551223
        + t * (1.00002368
        + t * (0.37409196
        + t * (0.09678418
        + t * (-0.18628806
        + t * (0.27886807
        + t * (-1.13520398
        + t * (1.48851587
        + t * (-0.82215223
        + t * 0.17087277)))))))))
        .exp();

    if x >= 0.0 { tau } else { 2.0 - tau }
}

fn print_ber_curve() {
    let height = 12;
    let width = 40;

    // BER from 1e-6 to 0.5
    let min_log = -6.0;
    let max_log = -0.3;

    // Eb/N0 from 0 to 14 dB
    let snr_min = 0.0;
    let snr_max = 14.0;

    // Print Y axis labels
    for row in 0..height {
        let _log_ber = max_log - (max_log - min_log) * row as f32 / (height - 1) as f32;

        // Y axis label
        if row == 0 {
            print!("1e-1 |");
        } else if row == height / 2 {
            print!("1e-3 |");
        } else if row == height - 1 {
            print!("1e-6 |");
        } else {
            print!("     |");
        }

        // Plot points
        for col in 0..width {
            let snr_db = snr_min + (snr_max - snr_min) * col as f32 / (width - 1) as f32;
            let ber = bpsk_theoretical_ber(snr_db);
            let ber_log = ber.log10() as f32;

            // Check if this point is on the curve
            let curve_row = ((max_log - ber_log) / (max_log - min_log) * (height - 1) as f32) as i32;
            if curve_row == row as i32 {
                print!("*");
            } else {
                print!(" ");
            }
        }
        println!();
    }

    // X axis
    println!("     +{}+", "-".repeat(width));
    println!("      0              7              14");
    println!("                 Eb/N0 (dB)");
}
