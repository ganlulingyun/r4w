//! Ring Buffer Throughput Benchmarks (MF-011)
//!
//! This benchmark suite proves the lock-free ring buffer can sustain
//! real-time streaming rates even under CPU stress.
//!
//! Run with:
//!   cargo bench -p r4w-core --bench ringbuffer_bench
//!
//! Key metrics:
//! - Throughput in samples/second
//! - Latency distribution
//! - Behavior under CPU stress

use criterion::{black_box, criterion_group, criterion_main, BenchmarkId, Criterion, Throughput};
use r4w_core::rt::{BufferPool, RingBuffer};
use r4w_core::types::IQSample;
use num_complex::Complex64;
use std::sync::Arc;
use std::thread;
use std::time::{Duration, Instant};

/// Test data: simulated I/Q samples
fn generate_test_samples(count: usize) -> Vec<IQSample> {
    (0..count)
        .map(|i| Complex64::new(i as f64 / 1000.0, (i as f64 / 1000.0).sin()))
        .collect()
}

/// Benchmark single-element push/pop operations
fn bench_single_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("ringbuffer_single");

    for capacity in [256, 1024, 4096, 16384] {
        group.throughput(Throughput::Elements(1));

        group.bench_with_input(
            BenchmarkId::new("push", capacity),
            &capacity,
            |b, &cap| {
                let rb: RingBuffer<IQSample> = RingBuffer::new(cap);
                let sample = Complex64::new(1.0, 2.0);
                b.iter(|| {
                    // Push until full, then pop all
                    while rb.push(black_box(sample)).is_ok() {}
                    while rb.pop().is_some() {}
                });
            },
        );

        group.bench_with_input(
            BenchmarkId::new("pop", capacity),
            &capacity,
            |b, &cap| {
                let rb: RingBuffer<IQSample> = RingBuffer::new(cap);
                let sample = Complex64::new(1.0, 2.0);
                // Pre-fill
                for _ in 0..cap {
                    let _ = rb.push(sample);
                }
                b.iter(|| {
                    let _ = black_box(rb.pop());
                    let _ = rb.push(sample);
                });
            },
        );
    }

    group.finish();
}

/// Benchmark batch (slice) operations for streaming
fn bench_batch_operations(c: &mut Criterion) {
    let mut group = c.benchmark_group("ringbuffer_batch");

    let chunk_sizes = [64, 256, 1024, 4096];
    let capacity = 65536; // 64K samples

    for chunk_size in chunk_sizes {
        group.throughput(Throughput::Elements(chunk_size as u64));

        let samples = generate_test_samples(chunk_size);
        let mut recv_buf = vec![Complex64::default(); chunk_size];

        group.bench_with_input(
            BenchmarkId::new("push_slice", chunk_size),
            &chunk_size,
            |b, &_| {
                let rb: RingBuffer<IQSample> = RingBuffer::new(capacity);
                b.iter(|| {
                    let pushed = rb.push_slice(black_box(&samples));
                    // Pop to prevent overflow
                    if rb.len() > capacity / 2 {
                        let mut dummy = vec![Complex64::default(); pushed];
                        rb.pop_slice(&mut dummy);
                    }
                    pushed
                });
            },
        );

        group.bench_with_input(
            BenchmarkId::new("pop_slice", chunk_size),
            &chunk_size,
            |b, &_| {
                let rb: RingBuffer<IQSample> = RingBuffer::new(capacity);
                // Pre-fill
                for _ in 0..capacity / chunk_size {
                    rb.push_slice(&samples);
                }
                b.iter(|| {
                    let popped = rb.pop_slice(black_box(&mut recv_buf));
                    // Refill to prevent underflow
                    if rb.len() < capacity / 2 {
                        rb.push_slice(&samples);
                    }
                    popped
                });
            },
        );
    }

    group.finish();
}

/// Benchmark real-time streaming throughput (producer/consumer pattern)
fn bench_streaming_throughput(c: &mut Criterion) {
    let mut group = c.benchmark_group("ringbuffer_streaming");
    group.measurement_time(Duration::from_secs(3));

    // Simulate different SDR sample rates
    // 2.4 MS/s = 2,400,000 samples/sec = typical RTL-SDR rate
    let chunk_sizes = [256, 1024, 4096]; // Samples per chunk
    let capacity = 131072; // 128K samples

    for chunk_size in chunk_sizes {
        group.throughput(Throughput::Elements(chunk_size as u64 * 1000));

        group.bench_with_input(
            BenchmarkId::new("producer_consumer", chunk_size),
            &chunk_size,
            |b, &chunk| {
                let rb = Arc::new(RingBuffer::<IQSample>::new(capacity));
                let samples = generate_test_samples(chunk);
                let mut recv_buf = vec![Complex64::default(); chunk];

                b.iter(|| {
                    // Simulate 1000 transfers (realistic burst)
                    for _ in 0..1000 {
                        rb.push_slice(&samples);
                        rb.pop_slice(&mut recv_buf);
                    }
                    recv_buf.len()
                });
            },
        );
    }

    group.finish();
}

/// Benchmark SPSC throughput with actual threads
fn bench_threaded_throughput(c: &mut Criterion) {
    let mut group = c.benchmark_group("ringbuffer_threaded");
    group.sample_size(20);
    group.measurement_time(Duration::from_secs(5));

    let capacity = 131072;
    let total_samples: u64 = 10_000_000;

    for chunk_size in [256, 1024, 4096] {
        group.throughput(Throughput::Elements(total_samples));

        group.bench_with_input(
            BenchmarkId::new("spsc_throughput", chunk_size),
            &chunk_size,
            |b, &chunk| {
                b.iter_custom(|iters| {
                    let mut total_duration = Duration::ZERO;

                    for _ in 0..iters {
                        let rb = Arc::new(RingBuffer::<IQSample>::new(capacity));
                        let rb_producer = Arc::clone(&rb);
                        let rb_consumer = Arc::clone(&rb);

                        let samples_to_send = total_samples as usize;
                        let samples = generate_test_samples(chunk);

                        let start = Instant::now();

                        let producer = thread::spawn(move || {
                            let mut sent = 0;
                            while sent < samples_to_send {
                                let to_push = (samples_to_send - sent).min(chunk);
                                let pushed = rb_producer.push_slice(&samples[..to_push]);
                                sent += pushed;
                                if pushed == 0 {
                                    std::hint::spin_loop();
                                }
                            }
                        });

                        let consumer = thread::spawn(move || {
                            let mut received = 0;
                            let mut buf = vec![Complex64::default(); chunk];
                            while received < samples_to_send {
                                let popped = rb_consumer.pop_slice(&mut buf);
                                received += popped;
                                if popped == 0 {
                                    std::hint::spin_loop();
                                }
                            }
                        });

                        producer.join().unwrap();
                        consumer.join().unwrap();

                        total_duration += start.elapsed();
                    }

                    total_duration
                });
            },
        );
    }

    group.finish();
}

/// Benchmark under CPU stress to prove RT performance
fn bench_under_stress(c: &mut Criterion) {
    let mut group = c.benchmark_group("ringbuffer_stress");
    group.sample_size(10);
    group.measurement_time(Duration::from_secs(5));

    let capacity = 65536;
    let chunk_size = 1024;
    let total_samples: u64 = 1_000_000;

    // Number of CPU stress threads
    for stress_threads in [0, 2, 4] {
        group.throughput(Throughput::Elements(total_samples));

        group.bench_with_input(
            BenchmarkId::new("with_stress_threads", stress_threads),
            &stress_threads,
            |b, &num_stress| {
                b.iter_custom(|iters| {
                    let mut total_duration = Duration::ZERO;

                    for _ in 0..iters {
                        // Start stress threads
                        let stop_flag = Arc::new(std::sync::atomic::AtomicBool::new(false));
                        let mut stress_handles = Vec::new();

                        for _ in 0..num_stress {
                            let flag = Arc::clone(&stop_flag);
                            stress_handles.push(thread::spawn(move || {
                                // CPU-intensive work: spin with some computation
                                let mut x = 0u64;
                                while !flag.load(std::sync::atomic::Ordering::Relaxed) {
                                    x = x.wrapping_mul(1103515245).wrapping_add(12345);
                                    std::hint::spin_loop();
                                }
                                black_box(x);
                            }));
                        }

                        let rb = Arc::new(RingBuffer::<IQSample>::new(capacity));
                        let rb_producer = Arc::clone(&rb);
                        let rb_consumer = Arc::clone(&rb);

                        let samples_to_send = total_samples as usize;
                        let samples = generate_test_samples(chunk_size);

                        let start = Instant::now();

                        let producer = thread::spawn(move || {
                            let mut sent = 0;
                            while sent < samples_to_send {
                                let to_push = (samples_to_send - sent).min(chunk_size);
                                let pushed = rb_producer.push_slice(&samples[..to_push]);
                                sent += pushed;
                                if pushed == 0 {
                                    std::hint::spin_loop();
                                }
                            }
                        });

                        let consumer = thread::spawn(move || {
                            let mut received = 0;
                            let mut buf = vec![Complex64::default(); chunk_size];
                            while received < samples_to_send {
                                let popped = rb_consumer.pop_slice(&mut buf);
                                received += popped;
                                if popped == 0 {
                                    std::hint::spin_loop();
                                }
                            }
                        });

                        producer.join().unwrap();
                        consumer.join().unwrap();

                        total_duration += start.elapsed();

                        // Stop stress threads
                        stop_flag.store(true, std::sync::atomic::Ordering::Relaxed);
                        for handle in stress_handles {
                            let _ = handle.join();
                        }
                    }

                    total_duration
                });
            },
        );
    }

    group.finish();
}

/// Benchmark buffer pool allocation speed
fn bench_buffer_pool(c: &mut Criterion) {
    let mut group = c.benchmark_group("buffer_pool");

    let buffer_sizes = [1024, 4096, 16384];
    let pool_size = 32;

    for buffer_size in buffer_sizes {
        group.throughput(Throughput::Elements(1));

        group.bench_with_input(
            BenchmarkId::new("acquire_release", buffer_size),
            &buffer_size,
            |b, &buf_size| {
                let pool: BufferPool<IQSample> = BufferPool::new(pool_size, buf_size);
                b.iter(|| {
                    let handle = pool.acquire().expect("pool exhausted");
                    black_box(&handle);
                    drop(handle);
                });
            },
        );

        group.bench_with_input(
            BenchmarkId::new("acquire_use_release", buffer_size),
            &buffer_size,
            |b, &buf_size| {
                let pool: BufferPool<IQSample> = BufferPool::new(pool_size, buf_size);
                b.iter(|| {
                    let mut handle = pool.acquire().expect("pool exhausted");
                    // Simulate using the buffer
                    for i in 0..buf_size {
                        handle[i] = Complex64::new(i as f64, 0.0);
                    }
                    black_box(&handle);
                    drop(handle);
                });
            },
        );
    }

    group.finish();
}

/// Print summary statistics useful for RT analysis
fn measure_latency_statistics() {
    println!("\n=== Ring Buffer Latency Statistics ===\n");

    let capacity = 65536;
    let chunk_size = 1024;
    let iterations = 10000;

    let rb = RingBuffer::<IQSample>::new(capacity);
    let samples = generate_test_samples(chunk_size);
    let mut recv_buf = vec![Complex64::default(); chunk_size];
    let mut push_times = Vec::with_capacity(iterations);
    let mut pop_times = Vec::with_capacity(iterations);

    // Warm up
    for _ in 0..100 {
        rb.push_slice(&samples);
        rb.pop_slice(&mut recv_buf);
    }

    // Measure
    for _ in 0..iterations {
        let start = Instant::now();
        rb.push_slice(&samples);
        push_times.push(start.elapsed().as_nanos() as u64);

        let start = Instant::now();
        rb.pop_slice(&mut recv_buf);
        pop_times.push(start.elapsed().as_nanos() as u64);
    }

    // Statistics
    push_times.sort();
    pop_times.sort();

    let push_avg: u64 = push_times.iter().sum::<u64>() / iterations as u64;
    let pop_avg: u64 = pop_times.iter().sum::<u64>() / iterations as u64;

    println!("Chunk size: {} samples ({} bytes)",
             chunk_size, chunk_size * std::mem::size_of::<IQSample>());
    println!();
    println!("Push latency (ns):");
    println!("  Mean:   {:>8}", push_avg);
    println!("  Median: {:>8}", push_times[iterations / 2]);
    println!("  p99:    {:>8}", push_times[iterations * 99 / 100]);
    println!("  p99.9:  {:>8}", push_times[iterations * 999 / 1000]);
    println!("  Max:    {:>8}", push_times[iterations - 1]);
    println!();
    println!("Pop latency (ns):");
    println!("  Mean:   {:>8}", pop_avg);
    println!("  Median: {:>8}", pop_times[iterations / 2]);
    println!("  p99:    {:>8}", pop_times[iterations * 99 / 100]);
    println!("  p99.9:  {:>8}", pop_times[iterations * 999 / 1000]);
    println!("  Max:    {:>8}", pop_times[iterations - 1]);
    println!();

    // Calculate throughput
    let total_samples = iterations * chunk_size;
    let total_time_ns: u64 = push_times.iter().sum::<u64>() + pop_times.iter().sum::<u64>();
    let throughput_samples_per_sec = (total_samples as f64 * 1_000_000_000.0) / total_time_ns as f64;
    println!("Throughput: {:.2} M samples/sec", throughput_samples_per_sec / 1_000_000.0);

    // Check if we can sustain common SDR rates
    let rtl_sdr_rate = 2_400_000.0; // 2.4 MS/s
    let usrp_b200_rate = 61_440_000.0; // 61.44 MS/s
    println!();
    println!("Can sustain RTL-SDR (2.4 MS/s): {}",
             if throughput_samples_per_sec > rtl_sdr_rate { "YES" } else { "NO" });
    println!("Can sustain USRP B200 (61.44 MS/s): {}",
             if throughput_samples_per_sec > usrp_b200_rate { "YES" } else { "NO" });
}

criterion_group!(
    benches,
    bench_single_operations,
    bench_batch_operations,
    bench_streaming_throughput,
    bench_threaded_throughput,
    bench_under_stress,
    bench_buffer_pool,
);

criterion_main!(benches);
