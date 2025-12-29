//! Parallel Processing Benchmarks
//!
//! Compares sequential vs parallel implementations to measure speedup.
//!
//! Run with: cargo bench -p r4w-core --features parallel --bench parallel_bench

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use rayon::prelude::*;
use rustfft::num_complex::Complex64;
use std::time::Duration;

use r4w_core::fft_utils::{FftProcessor, Spectrogram};
use r4w_core::parallel::{self, ParallelDemodulator, ParallelModulator, ParallelWaveformProcessor};
use r4w_core::prelude::*;
use r4w_core::waveform::CommonParams;

/// Benchmark batch modulation: sequential vs parallel
fn bench_batch_modulation(c: &mut Criterion) {
    let mut group = c.benchmark_group("batch_modulation");
    group.measurement_time(Duration::from_secs(10));

    let params = LoRaParams::builder()
        .spreading_factor(7)
        .bandwidth(125_000)
        .build();

    // Test with different batch sizes
    for batch_size in [1, 4, 16, 64, 256].iter() {
        let payloads: Vec<Vec<u8>> = (0..*batch_size)
            .map(|i| format!("Message number {}", i).into_bytes())
            .collect();
        let payload_refs: Vec<&[u8]> = payloads.iter().map(|v| v.as_slice()).collect();

        group.throughput(Throughput::Elements(*batch_size as u64));

        // Sequential
        group.bench_with_input(
            BenchmarkId::new("sequential", batch_size),
            &payload_refs,
            |b, payloads| {
                b.iter(|| {
                    payloads
                        .iter()
                        .map(|payload| {
                            let mut modulator = r4w_core::modulation::Modulator::new(params.clone());
                            modulator.modulate(black_box(payload))
                        })
                        .collect::<Vec<_>>()
                })
            },
        );

        // Parallel
        let parallel_mod = ParallelModulator::new(params.clone());
        group.bench_with_input(
            BenchmarkId::new("parallel", batch_size),
            &payload_refs,
            |b, payloads| {
                b.iter(|| parallel_mod.modulate_batch(black_box(payloads)))
            },
        );
    }

    group.finish();
}

/// Benchmark spectrogram computation: sequential vs parallel
fn bench_spectrogram(c: &mut Criterion) {
    let mut group = c.benchmark_group("spectrogram");
    group.measurement_time(Duration::from_secs(10));

    let sample_rate = 125000.0;
    let fft_size = 256;
    let hop_size = 64;

    // Test with different signal lengths
    for signal_length in [4096, 16384, 65536, 262144].iter() {
        // Generate test signal
        let signal: Vec<Complex64> = (0..*signal_length)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * std::f64::consts::PI * 1000.0 * t;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        group.throughput(Throughput::Elements(*signal_length as u64));

        // Sequential
        group.bench_with_input(
            BenchmarkId::new("sequential", signal_length),
            &signal,
            |b, signal| {
                b.iter(|| Spectrogram::compute(black_box(signal), fft_size, hop_size, sample_rate))
            },
        );

        // Parallel
        group.bench_with_input(
            BenchmarkId::new("parallel", signal_length),
            &signal,
            |b, signal| {
                b.iter(|| {
                    parallel::parallel_spectrogram(black_box(signal), fft_size, hop_size, sample_rate)
                })
            },
        );
    }

    group.finish();
}

/// Benchmark symbol demodulation: sequential vs parallel
fn bench_symbol_demodulation(c: &mut Criterion) {
    let mut group = c.benchmark_group("symbol_demodulation");
    group.measurement_time(Duration::from_secs(10));

    let params = LoRaParams::builder()
        .spreading_factor(7)
        .bandwidth(125_000)
        .build();

    // Generate test samples (multiple symbols)
    let mut modulator = r4w_core::modulation::Modulator::new(params.clone());

    for num_symbols in [4, 16, 64, 256].iter() {
        // Create a message that will produce approximately num_symbols symbols
        let message: Vec<u8> = (0..(*num_symbols / 2).max(1))
            .map(|i| (i % 256) as u8)
            .collect();
        let samples = modulator.symbols_only(&message);

        group.throughput(Throughput::Elements(*num_symbols as u64));

        let samples_per_symbol = params.samples_per_symbol();

        // Sequential
        group.bench_with_input(
            BenchmarkId::new("sequential", num_symbols),
            &samples,
            |b, samples| {
                b.iter(|| {
                    let num_syms = samples.len() / samples_per_symbol;
                    let mut demodulator = r4w_core::demodulation::Demodulator::new(params.clone());
                    demodulator.demodulate_symbols(black_box(samples), num_syms)
                })
            },
        );

        // Parallel
        let parallel_demod = ParallelDemodulator::new(params.clone());
        group.bench_with_input(
            BenchmarkId::new("parallel", num_symbols),
            &samples,
            |b, samples| {
                b.iter(|| parallel_demod.demodulate_symbols_parallel(black_box(samples)))
            },
        );
    }

    group.finish();
}

/// Benchmark AWGN channel simulation: sequential vs parallel
fn bench_awgn_channel(c: &mut Criterion) {
    let mut group = c.benchmark_group("awgn_channel");
    group.measurement_time(Duration::from_secs(10));

    let snr_db = 10.0;

    for batch_size in [1, 4, 16, 64].iter() {
        let signals: Vec<Vec<Complex64>> = (0..*batch_size)
            .map(|_| {
                (0..1024)
                    .map(|_| Complex64::new(1.0, 0.0))
                    .collect()
            })
            .collect();

        group.throughput(Throughput::Elements(*batch_size as u64));

        // Sequential
        group.bench_with_input(
            BenchmarkId::new("sequential", batch_size),
            &signals,
            |b, signals| {
                b.iter(|| {
                    signals
                        .iter()
                        .enumerate()
                        .map(|(i, signal)| {
                            // Simple sequential AWGN
                            let signal_power: f64 =
                                signal.iter().map(|s| s.norm_sqr()).sum::<f64>() / signal.len() as f64;
                            let snr_linear = 10.0_f64.powf(snr_db / 10.0);
                            let noise_power = signal_power / snr_linear;
                            let noise_std = (noise_power / 2.0).sqrt();

                            use rand::SeedableRng;
                            use rand_distr::{Distribution, Normal};
                            let mut rng = rand::rngs::StdRng::seed_from_u64(i as u64);
                            let normal = Normal::new(0.0, noise_std).unwrap();

                            signal
                                .iter()
                                .map(|&s| {
                                    let noise = Complex64::new(
                                        normal.sample(&mut rng),
                                        normal.sample(&mut rng),
                                    );
                                    s + noise
                                })
                                .collect::<Vec<_>>()
                        })
                        .collect::<Vec<_>>()
                })
            },
        );

        // Parallel
        group.bench_with_input(
            BenchmarkId::new("parallel", batch_size),
            &signals,
            |b, signals| {
                b.iter(|| parallel::channel::add_awgn_batch(black_box(signals), snr_db))
            },
        );
    }

    group.finish();
}

/// Benchmark waveform comparison: sequential vs parallel
fn bench_waveform_comparison(c: &mut Criterion) {
    let mut group = c.benchmark_group("waveform_comparison");
    group.measurement_time(Duration::from_secs(10));

    let sample_rate = 48000.0;
    let common_params = CommonParams {
        sample_rate,
        carrier_freq: 1000.0,
        amplitude: 1.0,
    };

    let waveforms = ["BPSK", "QPSK", "BFSK", "OOK", "16QAM"];
    let test_bits: Vec<u8> = vec![1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1];

    // Sequential
    group.bench_function("sequential", |b| {
        b.iter(|| {
            waveforms
                .iter()
                .filter_map(|&name| {
                    r4w_core::waveform::WaveformFactory::create(name, sample_rate).map(|waveform| {
                        let samples = waveform.modulate(&test_bits);
                        let result = waveform.demodulate(&samples);
                        (name.to_string(), samples, result)
                    })
                })
                .collect::<Vec<_>>()
        })
    });

    // Parallel
    let processor = ParallelWaveformProcessor::new(common_params);
    group.bench_function("parallel", |b| {
        b.iter(|| processor.compare_waveforms(black_box(&waveforms), black_box(&test_bits)))
    });

    group.finish();
}

/// Benchmark utility operations: sequential vs parallel
fn bench_parallel_utils(c: &mut Criterion) {
    let mut group = c.benchmark_group("parallel_utils");

    // Large sample buffer
    let samples: Vec<Complex64> = (0..1_000_000)
        .map(|i| Complex64::new((i as f64).sin(), (i as f64).cos()))
        .collect();

    group.throughput(Throughput::Elements(samples.len() as u64));

    // Magnitude computation
    group.bench_function("magnitude_sequential", |b| {
        b.iter(|| {
            samples
                .iter()
                .map(|s| s.norm())
                .collect::<Vec<_>>()
        })
    });

    group.bench_function("magnitude_parallel", |b| {
        b.iter(|| parallel::utils::parallel_magnitude(black_box(&samples)))
    });

    // Power computation
    group.bench_function("power_sequential", |b| {
        b.iter(|| {
            samples
                .iter()
                .map(|s| s.norm_sqr())
                .collect::<Vec<_>>()
        })
    });

    group.bench_function("power_parallel", |b| {
        b.iter(|| parallel::utils::parallel_power(black_box(&samples)))
    });

    group.finish();
}

criterion_group!(
    benches,
    bench_batch_modulation,
    bench_spectrogram,
    bench_symbol_demodulation,
    bench_awgn_channel,
    bench_waveform_comparison,
    bench_parallel_utils,
);
criterion_main!(benches);
