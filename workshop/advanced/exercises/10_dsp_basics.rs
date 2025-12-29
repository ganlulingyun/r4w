//! # Exercise 10: DSP Fundamentals
//!
//! Master the core concepts of digital signal processing.
//!
//! ## Topics
//! - Complex numbers and IQ representation
//! - Sampling theory and Nyquist
//! - Time vs frequency domain
//! - Signal power and energy
//!
//! ## Run
//! ```bash
//! cargo run --example 10_dsp_basics
//! ```
//!
//! trace:FR-0095 | ai:claude

use r4w_core::IQSample;
use std::f64::consts::PI;

fn main() {
    println!("=== DSP Fundamentals Workshop ===\n");

    // 1. Complex Numbers and IQ
    println!("== Part 1: Complex Numbers and IQ ==\n");
    demonstrate_complex_numbers();

    // 2. Sampling Theory
    println!("\n== Part 2: Sampling Theory ==\n");
    demonstrate_sampling();

    // 3. Time vs Frequency Domain
    println!("\n== Part 3: Time vs Frequency Domain ==\n");
    demonstrate_domains();

    // 4. Power and Energy
    println!("\n== Part 4: Signal Power and Energy ==\n");
    demonstrate_power();

    println!("\n=== Workshop Complete ===");
}

fn demonstrate_complex_numbers() {
    // IQ samples represent complex numbers
    // I = In-phase (real), Q = Quadrature (imaginary)

    let sample = IQSample::new(0.707, 0.707);

    println!("IQ Sample: {} + {}j", sample.re, sample.im);

    // Magnitude (amplitude)
    let magnitude = (sample.re * sample.re + sample.im * sample.im).sqrt();
    println!("Magnitude: {:.4}", magnitude);

    // Phase (angle)
    let phase = sample.im.atan2(sample.re);
    let phase_deg = phase * 180.0 / PI;
    println!("Phase: {:.2}° ({:.4} radians)", phase_deg, phase);

    // Polar to rectangular conversion
    let r = 1.0;
    let theta = PI / 4.0; // 45 degrees
    let reconstructed = IQSample::new(r * theta.cos(), r * theta.sin());
    println!("\nPolar (r=1, θ=45°) → Rectangular: ({:.4}, {:.4})",
             reconstructed.re, reconstructed.im);

    // Complex multiplication (rotation)
    println!("\nComplex multiplication (rotation):");
    let a = IQSample::new(1.0, 0.0);  // 0 degrees
    let b = IQSample::new(0.707, 0.707);  // 45 degrees
    let c = complex_multiply(&a, &b);
    println!("  (1+0j) × (0.707+0.707j) = ({:.4}+{:.4}j)", c.re, c.im);

    // Complex conjugate
    let conj = IQSample::new(sample.re, -sample.im);
    println!("\nConjugate of ({:.3}+{:.3}j) = ({:.3}{:.3}j)",
             sample.re, sample.im, conj.re, conj.im);
}

fn demonstrate_sampling() {
    let signal_freq: f64 = 1000.0; // 1 kHz signal

    println!("Signal frequency: {} Hz", signal_freq);
    println!("Nyquist rate: {} Hz (minimum)", signal_freq * 2.0);

    // Show what happens at different sample rates
    let sample_rates: [f64; 4] = [500.0, 2000.0, 4000.0, 10000.0];

    for &fs in &sample_rates {
        let aliased = if fs < signal_freq * 2.0 {
            // Aliasing occurs
            let alias_freq = (signal_freq - fs).abs() % fs;
            format!("ALIASED to {:.0} Hz", alias_freq.min(fs - alias_freq))
        } else {
            "OK - properly sampled".to_string()
        };

        println!("  Fs = {} Hz: {}", fs, aliased);
    }

    // Generate samples at different rates
    println!("\nSampling 1 kHz tone at 8 kHz (8 samples/cycle):");
    let fs = 8000.0;
    let duration = 0.001; // 1 ms = 1 cycle
    let num_samples = (fs * duration) as usize;

    for i in 0..num_samples {
        let t = i as f64 / fs;
        let angle = 2.0 * PI * signal_freq * t;
        let sample = IQSample::new(angle.cos(), angle.sin());
        println!("  n={}: t={:.4}ms, phase={:6.1}°, I={:6.3}, Q={:6.3}",
                 i, t * 1000.0, angle * 180.0 / PI % 360.0, sample.re, sample.im);
    }
}

fn demonstrate_domains() {
    // Time domain: signal as amplitude vs time
    println!("Time Domain: Amplitude vs Time");
    println!("  - Each sample has a value at a specific time instant");
    println!("  - Easy to see: transients, envelope, timing");

    // Frequency domain: signal as power vs frequency
    println!("\nFrequency Domain: Power vs Frequency");
    println!("  - Shows which frequencies are present");
    println!("  - Easy to see: spectral content, bandwidth, interference");

    // Example: Sum of two tones
    let f1 = 1000.0; // 1 kHz
    let f2 = 3000.0; // 3 kHz
    let fs = 16000.0;
    let num_samples = 64;

    println!("\nExample: Sum of {} Hz and {} Hz tones", f1, f2);

    let mut samples: Vec<f64> = Vec::new();
    for i in 0..num_samples {
        let t = i as f64 / fs;
        let value = (2.0 * PI * f1 * t).cos() + 0.5 * (2.0 * PI * f2 * t).cos();
        samples.push(value);
    }

    // Show a few time-domain samples
    println!("\nTime domain (first 8 samples):");
    for (i, &s) in samples.iter().take(8).enumerate() {
        let bar = "█".repeat(((s + 1.5) * 10.0) as usize);
        println!("  n={}: {:6.3} |{}", i, s, bar);
    }

    // Compute simple DFT for a few bins
    println!("\nFrequency domain (magnitude at key frequencies):");
    let freq_bins = [0.0, 1000.0, 2000.0, 3000.0, 4000.0];
    for &freq in &freq_bins {
        let bin = (freq / fs * num_samples as f64) as usize;
        let mag = compute_dft_bin(&samples, bin);
        let bar = "█".repeat((mag * 5.0) as usize);
        println!("  {:5.0} Hz (bin {}): {:6.3} |{}", freq, bin, mag, bar);
    }
}

fn demonstrate_power() {
    // Generate test signals
    let fs = 10000.0;
    let num_samples = 1000;

    // 1. Unity amplitude sinusoid
    let sinusoid: Vec<IQSample> = (0..num_samples)
        .map(|i| {
            let phase = 2.0 * PI * 1000.0 * i as f64 / fs;
            IQSample::new(phase.cos(), phase.sin())
        })
        .collect();

    // 2. Half amplitude
    let half_amp: Vec<IQSample> = sinusoid.iter()
        .map(|s| IQSample::new(s.re * 0.5, s.im * 0.5))
        .collect();

    // 3. Noise-like signal
    let noisy: Vec<IQSample> = (0..num_samples)
        .map(|i| {
            let phase = 2.0 * PI * 1000.0 * i as f64 / fs;
            let noise = (i as f64 * 1.234).sin() * 0.3;
            IQSample::new(phase.cos() + noise, phase.sin() + noise)
        })
        .collect();

    println!("Power calculations:");

    let p1 = calculate_power(&sinusoid);
    println!("  Unity sinusoid:  {:.4} linear, {:.2} dB", p1, 10.0 * p1.log10());

    let p2 = calculate_power(&half_amp);
    println!("  Half amplitude:  {:.4} linear, {:.2} dB", p2, 10.0 * p2.log10());

    let p3 = calculate_power(&noisy);
    println!("  Noisy sinusoid:  {:.4} linear, {:.2} dB", p3, 10.0 * p3.log10());

    // Peak vs RMS
    println!("\nPeak vs RMS:");
    let peak = sinusoid.iter()
        .map(|s| (s.re * s.re + s.im * s.im).sqrt())
        .fold(0.0f64, f64::max);
    let rms = p1.sqrt();
    println!("  Peak amplitude: {:.4}", peak);
    println!("  RMS amplitude:  {:.4}", rms);
    println!("  Crest factor:   {:.4} ({:.2} dB)", peak / rms, 20.0 * (peak / rms).log10());

    // Energy
    println!("\nEnergy (power × time):");
    let duration = num_samples as f64 / fs;
    let energy = p1 * duration;
    println!("  Duration: {:.4} seconds", duration);
    println!("  Energy:   {:.6} joules (relative)", energy);
}

fn complex_multiply(a: &IQSample, b: &IQSample) -> IQSample {
    IQSample::new(
        a.re * b.re - a.im * b.im,
        a.re * b.im + a.im * b.re,
    )
}

fn compute_dft_bin(samples: &[f64], bin: usize) -> f64 {
    let n = samples.len();
    let mut re = 0.0f64;
    let mut im = 0.0f64;

    for (i, &s) in samples.iter().enumerate() {
        let angle = -2.0 * PI * bin as f64 * i as f64 / n as f64;
        re += s * angle.cos();
        im += s * angle.sin();
    }

    (re * re + im * im).sqrt() / n as f64
}

fn calculate_power(samples: &[IQSample]) -> f64 {
    samples.iter()
        .map(|s| s.re * s.re + s.im * s.im)
        .sum::<f64>() / samples.len() as f64
}
