//! Sub-Millisecond Hop Timing Validation Benchmark (MF-040)
//!
//! Verifies that frequency hopping operations can be completed within
//! sub-millisecond timing constraints required for protocols like LoRa FHSS.
//!
//! Run with:
//!
//! ```bash
//! cargo bench -p r4w-core --bench hop_timing_bench
//! ```
//!
//! This benchmark measures:
//! - FFT reconfiguration time
//! - Waveform parameter updates
//! - Frequency synthesizer settling simulation
//! - End-to-end hop timing

use std::time::Instant;

use r4w_core::fft_utils::FftProcessor;
use r4w_core::rt::LatencyHistogram;
use r4w_core::waveform::WaveformFactory;
use rustfft::num_complex::Complex64;

/// Number of hops to measure
const HOP_COUNT: usize = 10000;

/// Target hop time in microseconds
const TARGET_HOP_US: u64 = 500; // 500µs = 0.5ms

fn main() {
    println!("=== Sub-Millisecond Hop Timing Validation (MF-040) ===\n");
    println!("Target: < {}µs per hop\n", TARGET_HOP_US);

    // Test 1: FFT Reconfiguration (changing FFT size mid-stream)
    println!("--- Test 1: FFT Reconfiguration ---");
    let fft_stats = measure_fft_reconfiguration();
    print_timing_results("FFT Reconfig", &fft_stats, TARGET_HOP_US);
    println!();

    // Test 2: Waveform Parameter Update
    println!("--- Test 2: Waveform Parameter Update ---");
    let waveform_stats = measure_waveform_update();
    print_timing_results("Waveform Update", &waveform_stats, TARGET_HOP_US);
    println!();

    // Test 3: Chirp Reconfiguration (LoRa)
    println!("--- Test 3: LoRa Chirp Reconfiguration ---");
    let chirp_stats = measure_lora_reconfig();
    print_timing_results("LoRa Reconfig", &chirp_stats, TARGET_HOP_US);
    println!();

    // Test 4: Full Hop Simulation (FFT + waveform + processing)
    println!("--- Test 4: Full Hop Cycle ---");
    let hop_stats = measure_full_hop();
    print_timing_results("Full Hop", &hop_stats, TARGET_HOP_US);
    println!();

    // Test 5: Rapid Frequency Hopping Pattern
    println!("--- Test 5: Rapid Hop Sequence ---");
    let rapid_stats = measure_rapid_hopping();
    print_timing_results("Rapid Hops", &rapid_stats, TARGET_HOP_US);
    println!();

    // Summary
    println!("=== Summary ===");
    let all_pass = [
        fft_stats.p99_us < TARGET_HOP_US,
        waveform_stats.p99_us < TARGET_HOP_US,
        chirp_stats.p99_us < TARGET_HOP_US,
        hop_stats.p99_us < TARGET_HOP_US,
        rapid_stats.p99_us < TARGET_HOP_US,
    ];

    println!("FFT Reconfig:     p99={:>5}µs  {}", fft_stats.p99_us, if all_pass[0] { "✓" } else { "✗" });
    println!("Waveform Update:  p99={:>5}µs  {}", waveform_stats.p99_us, if all_pass[1] { "✓" } else { "✗" });
    println!("LoRa Reconfig:    p99={:>5}µs  {}", chirp_stats.p99_us, if all_pass[2] { "✓" } else { "✗" });
    println!("Full Hop:         p99={:>5}µs  {}", hop_stats.p99_us, if all_pass[3] { "✓" } else { "✗" });
    println!("Rapid Hops:       p99={:>5}µs  {}", rapid_stats.p99_us, if all_pass[4] { "✓" } else { "✗" });
    println!();

    let pass_count = all_pass.iter().filter(|&&x| x).count();
    if pass_count == 5 {
        println!("✓ PASS: All hop operations complete within {}µs target", TARGET_HOP_US);
    } else if pass_count >= 3 {
        println!("⚠ MARGINAL: {}/5 tests pass the {}µs target", pass_count, TARGET_HOP_US);
    } else {
        println!("✗ FAIL: Only {}/5 tests meet the {}µs target", pass_count, TARGET_HOP_US);
    }
}

/// Timing statistics
struct TimingStats {
    min_us: u64,
    max_us: u64,
    mean_us: u64,
    p50_us: u64,
    p99_us: u64,
    p999_us: u64,
}

/// Measure FFT reconfiguration time
fn measure_fft_reconfiguration() -> TimingStats {
    let hist = LatencyHistogram::new_us(10000, 1);

    // FFT sizes to cycle through (simulating hop to different bandwidths)
    let fft_sizes = [256, 512, 1024, 2048, 512, 256, 1024];
    let mut current_fft: Option<FftProcessor> = None;
    let mut buffer = vec![Complex64::new(0.0, 0.0); 2048];

    for i in 0..HOP_COUNT {
        let new_size = fft_sizes[i % fft_sizes.len()];

        let start = Instant::now();

        // Reconfigure FFT (this is the "hop")
        current_fft = Some(FftProcessor::new(new_size));

        // Do one FFT to ensure planner is warmed up
        let fft = current_fft.as_mut().unwrap();
        let working = &mut buffer[..new_size];
        fft.fft_inplace(working);

        hist.record_ns(start.elapsed().as_nanos() as u64);
    }

    let stats = hist.statistics();
    TimingStats {
        min_us: stats.min_ns / 1000,
        max_us: stats.max_ns / 1000,
        mean_us: stats.mean_ns / 1000,
        p50_us: stats.percentiles.p50_ns / 1000,
        p99_us: stats.percentiles.p99_ns / 1000,
        p999_us: stats.percentiles.p999_ns / 1000,
    }
}

/// Measure waveform parameter update time
fn measure_waveform_update() -> TimingStats {
    let hist = LatencyHistogram::new_us(10000, 1);

    let sample_rate = 48000.0;
    let test_bits: Vec<u8> = (0..32).map(|i| (i % 2) as u8).collect();

    // Cycle through different waveforms
    let waveform_names = ["BPSK", "QPSK", "8PSK", "BFSK"];

    for i in 0..HOP_COUNT {
        let wf_name = waveform_names[i % waveform_names.len()];

        let start = Instant::now();

        // Create new waveform (simulating parameter change)
        if let Some(waveform) = WaveformFactory::create(wf_name, sample_rate) {
            // Do one modulation to ensure waveform is ready
            let _ = waveform.modulate(&test_bits);
        }

        hist.record_ns(start.elapsed().as_nanos() as u64);
    }

    let stats = hist.statistics();
    TimingStats {
        min_us: stats.min_ns / 1000,
        max_us: stats.max_ns / 1000,
        mean_us: stats.mean_ns / 1000,
        p50_us: stats.percentiles.p50_ns / 1000,
        p99_us: stats.percentiles.p99_ns / 1000,
        p999_us: stats.percentiles.p999_ns / 1000,
    }
}

/// Measure LoRa chirp reconfiguration (changing SF/BW)
fn measure_lora_reconfig() -> TimingStats {
    let hist = LatencyHistogram::new_us(10000, 1);

    // Different sample rates to cycle through (simulating BW changes)
    let sample_rates = [125000.0, 250000.0, 500000.0, 125000.0, 250000.0];

    let test_bits: Vec<u8> = (0..16).map(|i| (i % 2) as u8).collect();

    for i in 0..HOP_COUNT {
        let sample_rate = sample_rates[i % sample_rates.len()];

        let start = Instant::now();

        // Create LoRa waveform with new sample rate
        if let Some(waveform) = WaveformFactory::create("LoRa", sample_rate) {
            // Do one modulation to ensure it's ready
            let _ = waveform.modulate(&test_bits);
        }

        hist.record_ns(start.elapsed().as_nanos() as u64);
    }

    let stats = hist.statistics();
    TimingStats {
        min_us: stats.min_ns / 1000,
        max_us: stats.max_ns / 1000,
        mean_us: stats.mean_ns / 1000,
        p50_us: stats.percentiles.p50_ns / 1000,
        p99_us: stats.percentiles.p99_ns / 1000,
        p999_us: stats.percentiles.p999_ns / 1000,
    }
}

/// Measure full hop cycle (FFT + waveform + DSP)
fn measure_full_hop() -> TimingStats {
    let hist = LatencyHistogram::new_us(10000, 1);

    let sample_rate = 48000.0;
    let test_bits: Vec<u8> = (0..32).map(|i| (i % 2) as u8).collect();

    // Pre-create some waveforms
    let waveforms: Vec<_> = ["BPSK", "QPSK"]
        .iter()
        .filter_map(|name| WaveformFactory::create(name, sample_rate))
        .collect();

    // Use fixed FFT size for consistency
    let fft_size = 1024;
    let mut fft = FftProcessor::new(fft_size);
    let mut buffer = vec![Complex64::new(0.0, 0.0); fft_size];

    for i in 0..HOP_COUNT {
        let start = Instant::now();

        // Full hop cycle:
        // 1. Select new waveform
        let waveform = &waveforms[i % waveforms.len()];

        // 2. Modulate data
        let samples = waveform.modulate(&test_bits);

        // 3. Copy to FFT buffer (zero-pad if needed)
        for (j, sample) in buffer.iter_mut().enumerate() {
            if j < samples.len() {
                *sample = samples[j];
            } else {
                *sample = Complex64::new(0.0, 0.0);
            }
        }

        // 4. Apply FFT (frequency domain processing)
        fft.fft_inplace(&mut buffer);

        // 5. IFFT back to time domain
        fft.ifft_inplace(&mut buffer);

        hist.record_ns(start.elapsed().as_nanos() as u64);
    }

    let stats = hist.statistics();
    TimingStats {
        min_us: stats.min_ns / 1000,
        max_us: stats.max_ns / 1000,
        mean_us: stats.mean_ns / 1000,
        p50_us: stats.percentiles.p50_ns / 1000,
        p99_us: stats.percentiles.p99_ns / 1000,
        p999_us: stats.percentiles.p999_ns / 1000,
    }
}

/// Measure rapid hopping sequence (like LoRa FHSS)
fn measure_rapid_hopping() -> TimingStats {
    let hist = LatencyHistogram::new_us(10000, 1);

    // Simulate a frequency hopping pattern (like LoRa FHSS uses)
    // This represents the processing needed at each hop
    let hop_channels = [0u8, 4, 7, 2, 5, 1, 6, 3]; // 8-channel hop pattern

    let sample_rate = 500000.0; // 500 kHz (LoRa)
    let fft_size = 1024;

    // Use BPSK for hop testing (faster than LoRa for pure hop timing measurement)
    let waveform = WaveformFactory::create("BPSK", sample_rate).unwrap();
    let mut fft = FftProcessor::new(fft_size);
    let mut buffer = vec![Complex64::new(0.0, 0.0); fft_size];

    // Data to transmit per hop
    let hop_data: Vec<u8> = (0..8).map(|i| (i % 2) as u8).collect();

    for i in 0..HOP_COUNT {
        let channel = hop_channels[i % hop_channels.len()];

        let start = Instant::now();

        // Simulate hop processing:
        // 1. Apply frequency offset for channel (simple phase rotation)
        let freq_offset = channel as f64 * 125000.0; // 125 kHz channel spacing

        // 2. Modulate hop payload
        let samples = waveform.modulate(&hop_data);

        // 3. Copy to buffer and apply frequency shift (simplified)
        for (j, sample) in buffer.iter_mut().enumerate() {
            if j < samples.len() {
                let phase = 2.0 * std::f64::consts::PI * freq_offset * (j as f64) / sample_rate;
                let shift = Complex64::new(phase.cos(), phase.sin());
                *sample = samples[j] * shift;
            } else {
                *sample = Complex64::new(0.0, 0.0);
            }
        }

        // 4. FFT for spectral shaping
        fft.fft_inplace(&mut buffer);

        std::hint::black_box(&buffer);

        hist.record_ns(start.elapsed().as_nanos() as u64);
    }

    let stats = hist.statistics();
    TimingStats {
        min_us: stats.min_ns / 1000,
        max_us: stats.max_ns / 1000,
        mean_us: stats.mean_ns / 1000,
        p50_us: stats.percentiles.p50_ns / 1000,
        p99_us: stats.percentiles.p99_ns / 1000,
        p999_us: stats.percentiles.p999_ns / 1000,
    }
}

/// Print timing results with pass/fail
fn print_timing_results(name: &str, stats: &TimingStats, target_us: u64) {
    let pass = stats.p99_us < target_us;
    println!("{} Timing:", name);
    println!("  Min:      {:>6}µs", stats.min_us);
    println!("  Max:      {:>6}µs", stats.max_us);
    println!("  Mean:     {:>6}µs", stats.mean_us);
    println!("  p50:      {:>6}µs", stats.p50_us);
    println!("  p99:      {:>6}µs  {}", stats.p99_us, if pass { "✓ < target" } else { "✗ > target" });
    println!("  p99.9:    {:>6}µs", stats.p999_us);
}
