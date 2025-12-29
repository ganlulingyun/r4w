//! Comprehensive Benchmarks for SDR DSP Operations
//!
//! Run with: cargo bench -p r4w-core --bench dsp_bench

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use rustfft::num_complex::Complex64;
use r4w_core::fft_utils::{FftProcessor, Spectrogram};
use r4w_core::prelude::*;
use r4w_core::waveform::WaveformFactory;
use std::time::Duration;

// ============================================================================
// LoRa Chirp Generation Benchmarks
// ============================================================================

fn bench_chirp_generation(c: &mut Criterion) {
    let mut group = c.benchmark_group("chirp_generation");

    for sf in [7, 8, 9, 10, 11, 12].iter() {
        let params = LoRaParams::builder()
            .spreading_factor(*sf)
            .bandwidth(125_000)
            .build();

        let chirp_gen = ChirpGenerator::new(params.clone());
        let samples_per_symbol = params.samples_per_symbol();

        group.throughput(Throughput::Elements(samples_per_symbol as u64));

        group.bench_with_input(BenchmarkId::new("symbol_chirp", sf), sf, |b, _| {
            b.iter(|| chirp_gen.generate_symbol_chirp_fast(black_box(42)))
        });
    }

    group.finish();
}

fn bench_chirp_types(c: &mut Criterion) {
    let mut group = c.benchmark_group("chirp_types");

    let params = LoRaParams::builder()
        .spreading_factor(7)
        .bandwidth(125_000)
        .build();

    let chirp_gen = ChirpGenerator::new(params);

    group.bench_function("base_upchirp", |b| {
        b.iter(|| chirp_gen.base_upchirp())
    });

    group.bench_function("base_downchirp", |b| {
        b.iter(|| chirp_gen.base_downchirp())
    });

    group.finish();
}

// ============================================================================
// LoRa Modulation Benchmarks
// ============================================================================

fn bench_lora_modulation(c: &mut Criterion) {
    let mut group = c.benchmark_group("lora_modulation");
    group.measurement_time(Duration::from_secs(5));

    let payload = b"Hello, LoRa! This is a test message for benchmarking.";

    for sf in [7, 9, 12].iter() {
        let params = LoRaParams::builder()
            .spreading_factor(*sf)
            .bandwidth(125_000)
            .coding_rate(1)
            .build();

        let mut modulator = Modulator::new(params);

        group.bench_with_input(BenchmarkId::new("full_modulate", sf), sf, |b, _| {
            b.iter(|| modulator.modulate(black_box(payload)))
        });
    }

    group.finish();
}

fn bench_lora_demodulation(c: &mut Criterion) {
    let mut group = c.benchmark_group("lora_demodulation");
    group.measurement_time(Duration::from_secs(5));

    for sf in [7, 9, 12].iter() {
        let params = LoRaParams::builder()
            .spreading_factor(*sf)
            .bandwidth(125_000)
            .coding_rate(1)
            .build();

        let mut modulator = Modulator::new(params.clone());
        let samples = modulator.symbols_only(b"Test");
        let mut demodulator = Demodulator::new(params);

        group.bench_with_input(BenchmarkId::new("demod_symbol", sf), sf, |b, _| {
            b.iter(|| demodulator.demodulate_symbol(black_box(&samples)))
        });
    }

    group.finish();
}

// ============================================================================
// FFT Benchmarks
// ============================================================================

fn bench_fft(c: &mut Criterion) {
    let mut group = c.benchmark_group("fft");

    for size in [128, 256, 512, 1024, 2048, 4096].iter() {
        let signal: Vec<Complex64> = (0..*size)
            .map(|i| {
                let t = i as f64 / *size as f64;
                Complex64::new((2.0 * std::f64::consts::PI * t).cos(), 0.0)
            })
            .collect();

        let mut fft = FftProcessor::new(*size);

        group.throughput(Throughput::Elements(*size as u64));

        group.bench_with_input(BenchmarkId::new("forward", size), size, |b, _| {
            let mut input = signal.clone();
            b.iter(|| {
                fft.fft_inplace(black_box(&mut input));
            })
        });
    }

    group.finish();
}

fn bench_fft_power_spectrum(c: &mut Criterion) {
    let mut group = c.benchmark_group("fft_power_spectrum");

    for size in [256, 512, 1024].iter() {
        let spectrum: Vec<Complex64> = (0..*size)
            .map(|i| Complex64::new(i as f64, (i as f64).sin()))
            .collect();

        group.throughput(Throughput::Elements(*size as u64));

        group.bench_with_input(BenchmarkId::new("power_db", size), &spectrum, |b, spec| {
            b.iter(|| FftProcessor::power_spectrum_db(black_box(spec)))
        });
    }

    group.finish();
}

// ============================================================================
// Spectrogram Benchmarks
// ============================================================================

fn bench_spectrogram(c: &mut Criterion) {
    let mut group = c.benchmark_group("spectrogram");
    group.measurement_time(Duration::from_secs(5));

    let sample_rate = 125000.0;

    for signal_len in [8192, 32768, 131072].iter() {
        let signal: Vec<Complex64> = (0..*signal_len)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * std::f64::consts::PI * 1000.0 * t;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        group.throughput(Throughput::Elements(*signal_len as u64));

        group.bench_with_input(
            BenchmarkId::new("compute", signal_len),
            &signal,
            |b, signal| {
                b.iter(|| Spectrogram::compute(black_box(signal), 256, 64, sample_rate))
            },
        );
    }

    group.finish();
}

// ============================================================================
// Waveform Benchmarks (Multi-waveform framework)
// ============================================================================

fn bench_waveform_modulation(c: &mut Criterion) {
    let mut group = c.benchmark_group("waveform_modulation");
    group.measurement_time(Duration::from_secs(5));

    let sample_rate = 48000.0;
    let test_bits: Vec<u8> = vec![1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1];

    let waveforms = ["BPSK", "QPSK", "8PSK", "BFSK", "4FSK", "OOK", "16QAM", "DSSS"];

    for wf_name in waveforms.iter() {
        if let Some(waveform) = WaveformFactory::create(wf_name, sample_rate) {
            group.bench_function(*wf_name, |b| {
                b.iter(|| waveform.modulate(black_box(&test_bits)))
            });
        }
    }

    group.finish();
}

fn bench_waveform_demodulation(c: &mut Criterion) {
    let mut group = c.benchmark_group("waveform_demodulation");
    group.measurement_time(Duration::from_secs(5));

    let sample_rate = 48000.0;
    let test_bits: Vec<u8> = vec![1, 0, 1, 1, 0, 0, 1, 0, 1, 1, 1, 0, 0, 1, 0, 1];

    let waveforms = ["BPSK", "QPSK", "BFSK", "OOK", "16QAM"];

    for wf_name in waveforms.iter() {
        if let Some(waveform) = WaveformFactory::create(wf_name, sample_rate) {
            let samples = waveform.modulate(&test_bits);

            group.bench_function(*wf_name, |b| {
                b.iter(|| waveform.demodulate(black_box(&samples)))
            });
        }
    }

    group.finish();
}

fn bench_waveform_roundtrip(c: &mut Criterion) {
    let mut group = c.benchmark_group("waveform_roundtrip");
    group.measurement_time(Duration::from_secs(5));

    let sample_rate = 48000.0;
    let test_bits: Vec<u8> = (0..64).map(|i| (i % 2) as u8).collect();

    let waveforms = ["BPSK", "QPSK", "BFSK", "OOK"];

    for wf_name in waveforms.iter() {
        if let Some(waveform) = WaveformFactory::create(wf_name, sample_rate) {
            group.bench_function(*wf_name, |b| {
                b.iter(|| {
                    let samples = waveform.modulate(black_box(&test_bits));
                    waveform.demodulate(&samples)
                })
            });
        }
    }

    group.finish();
}

// ============================================================================
// Coding Benchmarks (Hamming, Gray, Interleaving, Whitening)
// ============================================================================

fn bench_coding(c: &mut Criterion) {
    use r4w_core::coding::{GrayCode, HammingCode, Interleaver};
    use r4w_core::params::CodingRate;
    use r4w_core::whitening::Whitening;

    let mut group = c.benchmark_group("coding");

    // Hamming encoding
    let hamming = HammingCode::new(CodingRate::CR4_5);
    let nibbles: Vec<u8> = (0u16..256).map(|i| (i % 16) as u8).collect();

    group.throughput(Throughput::Elements(nibbles.len() as u64));

    group.bench_function("hamming_encode", |b| {
        b.iter(|| {
            nibbles
                .iter()
                .map(|&n| hamming.encode(black_box(n)))
                .collect::<Vec<_>>()
        })
    });

    // Hamming decoding
    let encoded: Vec<u8> = nibbles.iter().map(|&n| hamming.encode(n)).collect();
    group.bench_function("hamming_decode", |b| {
        b.iter(|| {
            encoded
                .iter()
                .map(|&c| hamming.decode(black_box(c)))
                .collect::<Vec<_>>()
        })
    });

    // Gray coding
    let gray = GrayCode::new(7);
    let symbols: Vec<u16> = (0..1024).map(|i| (i % 128) as u16).collect();
    group.bench_function("gray_encode", |b| {
        b.iter(|| {
            symbols
                .iter()
                .map(|&s| gray.encode(black_box(s)))
                .collect::<Vec<_>>()
        })
    });

    group.bench_function("gray_decode", |b| {
        b.iter(|| {
            symbols
                .iter()
                .map(|&s| gray.decode(black_box(s)))
                .collect::<Vec<_>>()
        })
    });

    // Interleaving
    let interleaver = Interleaver::new(7, CodingRate::CR4_5);
    let codewords: Vec<u8> = (0..64).collect();

    group.bench_function("interleave", |b| {
        b.iter(|| interleaver.interleave(black_box(&codewords)))
    });

    // Whitening
    let mut whitening = Whitening::new();
    let data: Vec<u8> = (0u8..=255).collect();

    group.bench_function("whitening", |b| {
        b.iter(|| {
            whitening.reset();
            whitening.whiten(black_box(&data))
        })
    });

    group.finish();
}

// ============================================================================
// Complex Number Operations
// ============================================================================

fn bench_complex_ops(c: &mut Criterion) {
    let mut group = c.benchmark_group("complex_ops");

    let size = 100_000;
    let samples: Vec<Complex64> = (0..size)
        .map(|i| Complex64::new((i as f64).sin(), (i as f64).cos()))
        .collect();

    group.throughput(Throughput::Elements(size as u64));

    group.bench_function("magnitude", |b| {
        b.iter(|| {
            samples
                .iter()
                .map(|s| s.norm())
                .collect::<Vec<_>>()
        })
    });

    group.bench_function("phase", |b| {
        b.iter(|| {
            samples
                .iter()
                .map(|s| s.arg())
                .collect::<Vec<_>>()
        })
    });

    group.bench_function("power", |b| {
        b.iter(|| {
            samples
                .iter()
                .map(|s| s.norm_sqr())
                .collect::<Vec<_>>()
        })
    });

    let samples2: Vec<Complex64> = (0..size)
        .map(|i| Complex64::new((i as f64 * 0.5).cos(), (i as f64 * 0.5).sin()))
        .collect();

    group.bench_function("complex_multiply", |b| {
        b.iter(|| {
            samples
                .iter()
                .zip(samples2.iter())
                .map(|(a, b)| a * b)
                .collect::<Vec<_>>()
        })
    });

    group.finish();
}

// ============================================================================
// Peak Finding
// ============================================================================

fn bench_peak_finding(c: &mut Criterion) {
    let mut group = c.benchmark_group("peak_finding");

    for size in [128, 256, 512, 1024].iter() {
        let spectrum: Vec<Complex64> = (0..*size)
            .map(|i| {
                let center = *size / 2;
                let dist = (i as i64 - center as i64).abs() as f64;
                let mag = (-dist * dist / 100.0).exp();
                Complex64::new(mag, 0.0)
            })
            .collect();

        group.throughput(Throughput::Elements(*size as u64));

        group.bench_with_input(BenchmarkId::new("find_peak", size), &spectrum, |b, spec| {
            b.iter(|| FftProcessor::find_peak(black_box(spec)))
        });

        group.bench_with_input(
            BenchmarkId::new("find_peak_interpolated", size),
            &spectrum,
            |b, spec| {
                b.iter(|| FftProcessor::find_peak_interpolated(black_box(spec)))
            },
        );
    }

    group.finish();
}

// ============================================================================
// Criterion Groups
// ============================================================================

criterion_group!(
    name = chirp_benches;
    config = Criterion::default();
    targets = bench_chirp_generation, bench_chirp_types
);

criterion_group!(
    name = lora_benches;
    config = Criterion::default();
    targets = bench_lora_modulation, bench_lora_demodulation
);

criterion_group!(
    name = fft_benches;
    config = Criterion::default();
    targets = bench_fft, bench_fft_power_spectrum, bench_spectrogram
);

criterion_group!(
    name = waveform_benches;
    config = Criterion::default();
    targets = bench_waveform_modulation, bench_waveform_demodulation, bench_waveform_roundtrip
);

criterion_group!(
    name = coding_benches;
    config = Criterion::default();
    targets = bench_coding
);

// ============================================================================
// SIMD Utils Benchmarks
// ============================================================================

fn bench_simd_utils(c: &mut Criterion) {
    use r4w_core::simd_utils;

    let mut group = c.benchmark_group("simd_utils");

    let size = 100_000;
    let samples: Vec<Complex64> = (0..size)
        .map(|i| Complex64::new((i as f64).sin(), (i as f64).cos()))
        .collect();

    let samples2: Vec<Complex64> = (0..size)
        .map(|i| Complex64::new((i as f64 * 0.5).cos(), (i as f64 * 0.5).sin()))
        .collect();

    group.throughput(Throughput::Elements(size as u64));

    group.bench_function("compute_magnitudes", |b| {
        b.iter(|| simd_utils::compute_magnitudes(black_box(&samples)))
    });

    group.bench_function("compute_power", |b| {
        b.iter(|| simd_utils::compute_power(black_box(&samples)))
    });

    group.bench_function("complex_multiply", |b| {
        b.iter(|| simd_utils::complex_multiply(black_box(&samples), black_box(&samples2)))
    });

    group.bench_function("find_max_magnitude", |b| {
        b.iter(|| simd_utils::find_max_magnitude(black_box(&samples)))
    });

    group.bench_function("total_power", |b| {
        b.iter(|| simd_utils::total_power(black_box(&samples)))
    });

    group.bench_function("normalize", |b| {
        b.iter(|| simd_utils::normalize(black_box(&samples)))
    });

    // Windowing benchmark
    let window = simd_utils::hann_window(1024);
    let mut windowed_samples: Vec<Complex64> = (0..1024)
        .map(|i| Complex64::new(i as f64, 0.0))
        .collect();

    group.bench_function("apply_window_1024", |b| {
        b.iter(|| {
            let mut s = windowed_samples.clone();
            simd_utils::apply_window(black_box(&mut s), black_box(&window));
            s
        })
    });

    // Correlation benchmark
    let signal: Vec<Complex64> = (0..10000)
        .map(|i| Complex64::new((i as f64 * 0.1).sin(), 0.0))
        .collect();
    let reference: Vec<Complex64> = (0..128)
        .map(|i| Complex64::new((i as f64 * 0.1).sin(), 0.0))
        .collect();

    group.bench_function("sliding_correlation", |b| {
        b.iter(|| simd_utils::sliding_correlation_magnitude(black_box(&signal), black_box(&reference)))
    });

    group.finish();
}

criterion_group!(
    name = util_benches;
    config = Criterion::default();
    targets = bench_complex_ops, bench_peak_finding
);

criterion_group!(
    name = simd_benches;
    config = Criterion::default();
    targets = bench_simd_utils
);

criterion_main!(
    chirp_benches,
    lora_benches,
    fft_benches,
    waveform_benches,
    coding_benches,
    util_benches,
    simd_benches
);
