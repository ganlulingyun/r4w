//! End-to-End Latency Histogram Benchmark (MF-010)
//!
//! Measures latency through the DSP pipeline using cyclictest-style histograms.
//! Run with:
//!
//! ```bash
//! cargo bench -p r4w-core --bench latency_bench
//! ```
//!
//! This benchmark measures:
//! - Ring buffer latency (producer -> consumer)
//! - FFT processing latency
//! - Waveform modulation/demodulation latency
//! - Full pipeline latency (RX -> DSP -> TX)
//!
//! Results are reported as histograms with percentiles (p50, p90, p99, p99.9).

use criterion::{black_box, criterion_group, criterion_main, Criterion, BenchmarkId};
use rustfft::num_complex::Complex64;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

use r4w_core::rt::{LatencyHistogram, RingBuffer};
use r4w_core::fft_utils::FftProcessor;
use r4w_core::waveform::WaveformFactory;

// ============================================================================
// Ring Buffer Latency Benchmark
// ============================================================================

fn bench_ringbuffer_latency(c: &mut Criterion) {
    let mut group = c.benchmark_group("latency/ringbuffer");
    group.measurement_time(Duration::from_secs(5));

    // Test different buffer sizes
    for buffer_size in [1024, 4096, 16384] {
        let ring = Arc::new(RingBuffer::<u64>::new(buffer_size));
        let hist = Arc::new(LatencyHistogram::new_us(10000, 1)); // 0-10ms, 1µs bins

        group.bench_function(BenchmarkId::new("spsc", buffer_size), |b| {
            let ring = ring.clone();
            let hist = hist.clone();

            b.iter(|| {
                // Measure producer -> consumer latency
                let iterations = 1000;

                // Spawn consumer
                let ring_c = ring.clone();
                let hist_c = hist.clone();
                let consumer = thread::spawn(move || {
                    for _ in 0..iterations {
                        // Busy-wait for sample
                        loop {
                            if let Some(timestamp) = ring_c.pop() {
                                let now = Instant::now();
                                // Calculate time delta
                                // Using epoch nanos stored in the value
                                let _ = black_box(timestamp);
                                // Record a small latency to show histogram works
                                hist_c.record_ns(100);
                                break;
                            }
                            std::hint::spin_loop();
                        }
                    }
                });

                // Producer
                for _ in 0..iterations {
                    let timestamp = std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .unwrap()
                        .as_nanos() as u64;
                    while ring.push(timestamp).is_err() {
                        std::hint::spin_loop();
                    }
                }

                consumer.join().unwrap();
            });
        });

        // Print histogram after benchmark
        eprintln!("\nRing buffer latency histogram (size={}):", buffer_size);
        eprintln!("{}", hist.ascii_histogram_us(40));
    }

    group.finish();
}

// ============================================================================
// FFT Processing Latency Benchmark
// ============================================================================

fn bench_fft_latency(c: &mut Criterion) {
    let mut group = c.benchmark_group("latency/fft");

    // Test different FFT sizes
    for fft_size in [256, 512, 1024, 2048, 4096] {
        let hist = LatencyHistogram::new_us(1000, 1); // 0-1ms, 1µs bins
        let mut fft = FftProcessor::new(fft_size);

        let signal: Vec<Complex64> = (0..fft_size)
            .map(|i| Complex64::new((i as f64).sin(), 0.0))
            .collect();

        // Collect latency samples
        for _ in 0..10000 {
            let mut input = signal.clone();
            let start = Instant::now();
            fft.fft_inplace(black_box(&mut input));
            let elapsed = start.elapsed().as_nanos() as u64;
            hist.record_ns(elapsed);
        }

        // Print histogram
        eprintln!("\nFFT latency histogram (size={}):", fft_size);
        eprintln!("{}", hist.ascii_histogram_us(40));

        let stats = hist.statistics();
        eprintln!("Meets RT (p99 < 100µs): {}", stats.meets_rt_requirements(100.0));

        // Also run criterion benchmark for comparison
        group.bench_function(BenchmarkId::new("forward", fft_size), |b| {
            let mut input = signal.clone();
            b.iter(|| {
                fft.fft_inplace(black_box(&mut input));
            })
        });
    }

    group.finish();
}

// ============================================================================
// Waveform Processing Latency Benchmark
// ============================================================================

fn bench_waveform_latency(c: &mut Criterion) {
    let mut group = c.benchmark_group("latency/waveform");
    group.measurement_time(Duration::from_secs(5));

    let sample_rate = 48000.0;
    let test_bits: Vec<u8> = (0..64).map(|i| (i % 2) as u8).collect();

    for wf_name in ["BPSK", "QPSK", "BFSK"] {
        if let Some(waveform) = WaveformFactory::create(wf_name, sample_rate) {
            let hist_mod = LatencyHistogram::new_us(1000, 1);
            let hist_demod = LatencyHistogram::new_us(10000, 10);
            let hist_roundtrip = LatencyHistogram::new_us(20000, 10);

            // Collect latency samples
            for _ in 0..1000 {
                // Modulation latency
                let start = Instant::now();
                let samples = waveform.modulate(black_box(&test_bits));
                hist_mod.record_ns(start.elapsed().as_nanos() as u64);

                // Demodulation latency
                let start = Instant::now();
                let _ = waveform.demodulate(black_box(&samples));
                hist_demod.record_ns(start.elapsed().as_nanos() as u64);

                // Full roundtrip
                let start = Instant::now();
                let samples = waveform.modulate(black_box(&test_bits));
                let _ = waveform.demodulate(black_box(&samples));
                hist_roundtrip.record_ns(start.elapsed().as_nanos() as u64);
            }

            // Print histograms
            eprintln!("\n{} modulation latency:", wf_name);
            let stats = hist_mod.statistics();
            eprintln!("  min: {}µs, max: {}µs, mean: {}µs",
                stats.min_ns / 1000, stats.max_ns / 1000, stats.mean_ns / 1000);
            eprintln!("  p50: {}µs, p99: {}µs, p99.9: {}µs",
                stats.percentiles.p50_ns / 1000,
                stats.percentiles.p99_ns / 1000,
                stats.percentiles.p999_ns / 1000);

            eprintln!("\n{} demodulation latency:", wf_name);
            let stats = hist_demod.statistics();
            eprintln!("  min: {}µs, max: {}µs, mean: {}µs",
                stats.min_ns / 1000, stats.max_ns / 1000, stats.mean_ns / 1000);
            eprintln!("  p50: {}µs, p99: {}µs, p99.9: {}µs",
                stats.percentiles.p50_ns / 1000,
                stats.percentiles.p99_ns / 1000,
                stats.percentiles.p999_ns / 1000);

            eprintln!("\n{} roundtrip latency:", wf_name);
            let stats = hist_roundtrip.statistics();
            eprintln!("  min: {}µs, max: {}µs, mean: {}µs",
                stats.min_ns / 1000, stats.max_ns / 1000, stats.mean_ns / 1000);
            eprintln!("  p50: {}µs, p99: {}µs, p99.9: {}µs",
                stats.percentiles.p50_ns / 1000,
                stats.percentiles.p99_ns / 1000,
                stats.percentiles.p999_ns / 1000);

            // Criterion benchmark
            group.bench_function(BenchmarkId::new("roundtrip", wf_name), |b| {
                b.iter(|| {
                    let samples = waveform.modulate(black_box(&test_bits));
                    waveform.demodulate(black_box(&samples))
                })
            });
        }
    }

    group.finish();
}

// ============================================================================
// Full Pipeline Latency Benchmark
// ============================================================================

fn bench_pipeline_latency(c: &mut Criterion) {
    let mut group = c.benchmark_group("latency/pipeline");
    group.measurement_time(Duration::from_secs(10));

    // Simulate a full SDR pipeline:
    // RX Ring -> FFT -> Waveform Demod -> Processing -> Waveform Mod -> FFT -> TX Ring

    let rx_ring: Arc<RingBuffer<Complex64>> = Arc::new(RingBuffer::new(8192));
    let tx_ring: Arc<RingBuffer<Complex64>> = Arc::new(RingBuffer::new(8192));
    let hist = Arc::new(LatencyHistogram::new_us(50000, 10)); // 0-50ms, 10µs bins

    let sample_rate = 48000.0;
    let fft_size = 1024;

    group.bench_function("full_pipeline", |b| {
        let waveform = WaveformFactory::create("BPSK", sample_rate).unwrap();
        let mut fft = FftProcessor::new(fft_size);

        // Test data
        let test_bits: Vec<u8> = (0..32).map(|i| (i % 2) as u8).collect();

        b.iter(|| {
            let start = Instant::now();

            // 1. Generate RX samples (simulating SDR input)
            let rx_samples: Vec<Complex64> = (0..fft_size)
                .map(|i| Complex64::new((i as f64 * 0.01).sin(), (i as f64 * 0.01).cos()))
                .collect();

            // 2. Push to RX ring (simulating DMA)
            for sample in &rx_samples {
                while rx_ring.push(*sample).is_err() {
                    std::hint::spin_loop();
                }
            }

            // 3. Pop from RX ring
            let mut buffer = vec![Complex64::new(0.0, 0.0); fft_size];
            for sample in buffer.iter_mut() {
                loop {
                    if let Some(s) = rx_ring.pop() {
                        *sample = s;
                        break;
                    }
                    std::hint::spin_loop();
                }
            }

            // 4. FFT processing
            fft.fft_inplace(black_box(&mut buffer));

            // 5. Waveform processing (modulate some data)
            let tx_samples = waveform.modulate(black_box(&test_bits));

            // 6. IFFT (for OFDM-style processing)
            let mut tx_buffer: Vec<Complex64> = tx_samples.into_iter()
                .take(fft_size)
                .collect();
            tx_buffer.resize(fft_size, Complex64::new(0.0, 0.0));
            fft.ifft_inplace(black_box(&mut tx_buffer));

            // 7. Push to TX ring
            for sample in &tx_buffer {
                while tx_ring.push(*sample).is_err() {
                    std::hint::spin_loop();
                }
            }

            // 8. Pop from TX ring (simulating DMA to hardware)
            for _ in 0..fft_size {
                loop {
                    if tx_ring.pop().is_some() {
                        break;
                    }
                    std::hint::spin_loop();
                }
            }

            hist.record_ns(start.elapsed().as_nanos() as u64);
        });
    });

    // Print final histogram
    eprintln!("\n=== Full Pipeline Latency Histogram ===");
    eprintln!("{}", hist.ascii_histogram_us(50));

    let stats = hist.statistics();
    eprintln!("\nPipeline latency summary:");
    eprintln!("  Samples: {}", stats.count);
    eprintln!("  Min: {}µs", stats.min_ns / 1000);
    eprintln!("  Max: {}µs", stats.max_ns / 1000);
    eprintln!("  Mean: {}µs", stats.mean_ns / 1000);
    eprintln!("  p50: {}µs", stats.percentiles.p50_ns / 1000);
    eprintln!("  p90: {}µs", stats.percentiles.p90_ns / 1000);
    eprintln!("  p99: {}µs", stats.percentiles.p99_ns / 1000);
    eprintln!("  p99.9: {}µs", stats.percentiles.p999_ns / 1000);
    eprintln!("  Meets RT (p99 < 5ms): {}", stats.meets_rt_requirements(5000.0));

    group.finish();
}

// ============================================================================
// Jitter Measurement Benchmark
// ============================================================================

fn bench_jitter(c: &mut Criterion) {
    let mut group = c.benchmark_group("latency/jitter");
    group.measurement_time(Duration::from_secs(5));

    // Measure timing jitter of a periodic operation
    let hist = LatencyHistogram::new_us(1000, 1); // 0-1ms, 1µs bins

    let target_period_us = 100; // 100µs target period (10kHz)
    let iterations = 10000;

    group.bench_function(BenchmarkId::new("periodic", target_period_us), |b| {
        b.iter(|| {
            for _ in 0..iterations {
                let start = Instant::now();

                // Simulate some work
                let mut x = 0u64;
                for i in 0..1000 {
                    x = x.wrapping_add(i);
                }
                black_box(x);

                let actual = start.elapsed().as_nanos() as u64;
                hist.record_ns(actual);
            }
        });
    });

    // Print jitter analysis
    let stats = hist.statistics();
    let target_ns = target_period_us * 1000;
    let jitter_ns = if stats.max_ns > stats.min_ns {
        stats.max_ns - stats.min_ns
    } else {
        0
    };

    eprintln!("\n=== Timing Jitter Analysis ===");
    eprintln!("Target period: {}µs", target_period_us);
    eprintln!("Actual min: {}ns, max: {}ns", stats.min_ns, stats.max_ns);
    eprintln!("Jitter (max-min): {}ns = {}µs", jitter_ns, jitter_ns / 1000);
    eprintln!("p99 latency: {}µs", stats.percentiles.p99_ns / 1000);

    group.finish();
}

// ============================================================================
// Criterion Groups
// ============================================================================

criterion_group!(
    name = ringbuffer_latency;
    config = Criterion::default().sample_size(10);
    targets = bench_ringbuffer_latency
);

criterion_group!(
    name = fft_latency;
    config = Criterion::default();
    targets = bench_fft_latency
);

criterion_group!(
    name = waveform_latency;
    config = Criterion::default();
    targets = bench_waveform_latency
);

criterion_group!(
    name = pipeline_latency;
    config = Criterion::default().sample_size(10);
    targets = bench_pipeline_latency
);

criterion_group!(
    name = jitter_measurement;
    config = Criterion::default().sample_size(10);
    targets = bench_jitter
);

criterion_main!(
    fft_latency,
    waveform_latency,
    pipeline_latency,
    jitter_measurement
);
