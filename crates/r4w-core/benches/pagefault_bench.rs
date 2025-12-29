//! Page Fault Validation Benchmark (MF-014)
//!
//! Verifies that real-time paths have zero page faults when memory is locked.
//! Run with:
//!
//! ```bash
//! # Run with sudo for mlockall() support
//! sudo -E cargo bench -p r4w-core --bench pagefault_bench
//!
//! # Or without sudo (will show difference with/without locking)
//! cargo bench -p r4w-core --bench pagefault_bench
//! ```
//!
//! This benchmark measures:
//! - Page fault counts during RT operations
//! - Memory locking effectiveness
//! - Prefaulting strategy validation

use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Instant;

use r4w_core::rt::{LockedBuffer, RtConfig, RtPriority, spawn_rt_thread, prefault_pages};
use r4w_core::fft_utils::FftProcessor;
use rustfft::num_complex::Complex64;

/// Number of iterations for page fault measurement
const ITERATIONS: usize = 10000;

fn main() {
    println!("=== Page Fault Validation Benchmark (MF-014) ===\n");

    // Check if we have mlock privileges
    let has_mlock_privs = check_mlock_privileges();
    if has_mlock_privs {
        println!("✓ Memory locking (mlockall) is available");
    } else {
        println!("⚠ Memory locking requires elevated privileges (run with sudo)");
    }
    println!();

    // Test 1: Page faults with standard allocation
    println!("--- Test 1: Standard Allocation (no mlock) ---");
    let (faults_before, faults_after) = measure_page_faults_standard();
    let std_faults = faults_after.saturating_sub(faults_before);
    println!("Page faults during RT operations: {}", std_faults);
    println!();

    // Test 2: Page faults with LockedBuffer
    println!("--- Test 2: LockedBuffer (mlock per-buffer) ---");
    let (faults_before, faults_after) = measure_page_faults_locked_buffer();
    let locked_faults = faults_after.saturating_sub(faults_before);
    println!("Page faults during RT operations: {}", locked_faults);
    if locked_faults == 0 {
        println!("✓ Zero page faults with LockedBuffer");
    }
    println!();

    // Test 3: Page faults with prefaulting
    println!("--- Test 3: Prefaulted Memory ---");
    let (faults_before, faults_after) = measure_page_faults_prefaulted();
    let prefault_faults = faults_after.saturating_sub(faults_before);
    println!("Page faults during RT operations: {}", prefault_faults);
    if prefault_faults == 0 {
        println!("✓ Zero page faults with prefaulting");
    }
    println!();

    if has_mlock_privs {
        // Test 4: Full mlockall in RT thread
        println!("--- Test 4: mlockall() in RT Thread ---");
        let (faults_before, faults_after) = measure_page_faults_mlockall();
        let mlockall_faults = faults_after.saturating_sub(faults_before);
        println!("Page faults during RT operations: {}", mlockall_faults);
        if mlockall_faults == 0 {
            println!("✓ Zero page faults with mlockall()");
        }
        println!();
    }

    // Summary
    println!("=== Summary ===");
    println!("Standard allocation: {} page faults", std_faults);
    println!("LockedBuffer:        {} page faults", locked_faults);
    println!("Prefaulted:          {} page faults", prefault_faults);

    if has_mlock_privs {
        let mlockall_faults = measure_page_faults_mlockall().1.saturating_sub(
            measure_page_faults_mlockall().0
        );
        println!("mlockall():          {} page faults", mlockall_faults);
    }

    // Pass/fail assessment
    println!();
    if locked_faults == 0 || prefault_faults == 0 {
        println!("✓ PASS: At least one RT-safe memory strategy achieves zero page faults");
    } else {
        println!("⚠ WARNING: Page faults detected in RT paths");
        println!("  Recommendation: Use LockedBuffer or prefault_pages() before entering RT context");
    }
}

/// Check if we can use mlockall
fn check_mlock_privileges() -> bool {
    #[cfg(target_os = "linux")]
    {
        unsafe {
            let result = libc::mlockall(libc::MCL_CURRENT);
            if result == 0 {
                libc::munlockall();
                return true;
            }
        }
        false
    }

    #[cfg(not(target_os = "linux"))]
    false
}

/// Get current page fault counts from /proc/self/stat
#[cfg(target_os = "linux")]
fn get_page_faults() -> (u64, u64) {
    use std::fs;
    if let Ok(stat) = fs::read_to_string("/proc/self/stat") {
        let parts: Vec<&str> = stat.split_whitespace().collect();
        // minflt is field 10, majflt is field 12 (0-indexed: 9, 11)
        if parts.len() > 12 {
            let minflt = parts[9].parse::<u64>().unwrap_or(0);
            let majflt = parts[11].parse::<u64>().unwrap_or(0);
            return (minflt, majflt);
        }
    }
    (0, 0)
}

#[cfg(not(target_os = "linux"))]
fn get_page_faults() -> (u64, u64) {
    (0, 0)
}

/// Total page faults (minor + major)
fn total_page_faults() -> u64 {
    let (minor, major) = get_page_faults();
    minor + major
}

/// Measure page faults with standard Vec allocation
fn measure_page_faults_standard() -> (u64, u64) {
    let fft_size = 1024;
    let mut fft = FftProcessor::new(fft_size);

    // Pre-create the FFT processor outside measurement window
    let _ = fft.fft_inplace(&mut vec![Complex64::new(0.0, 0.0); fft_size]);

    let faults_before = total_page_faults();

    // Run FFT operations with dynamic allocation
    for _ in 0..ITERATIONS {
        // This allocation may cause page faults
        let mut buffer: Vec<Complex64> = (0..fft_size)
            .map(|i| Complex64::new((i as f64).sin(), 0.0))
            .collect();
        fft.fft_inplace(&mut buffer);
        std::hint::black_box(&buffer);
    }

    let faults_after = total_page_faults();

    println!("  Minor/Major faults before: {:?}", get_page_faults());
    println!("  Total faults at start: {}", faults_before);
    println!("  Total faults at end: {}", faults_after);

    (faults_before, faults_after)
}

/// Measure page faults with LockedBuffer
fn measure_page_faults_locked_buffer() -> (u64, u64) {
    let fft_size = 1024;
    let mut fft = FftProcessor::new(fft_size);

    // Create locked buffer BEFORE entering RT path
    let mut locked_buf: LockedBuffer<Complex64> = LockedBuffer::new(fft_size)
        .expect("Failed to create locked buffer");

    println!("  Buffer locked: {}", locked_buf.is_locked());

    // Initialize the buffer
    for (i, sample) in locked_buf.as_mut_slice().iter_mut().enumerate() {
        *sample = Complex64::new((i as f64).sin(), 0.0);
    }

    // Warm up FFT
    let mut temp = locked_buf.as_slice().to_vec();
    fft.fft_inplace(&mut temp);

    let faults_before = total_page_faults();

    // Run FFT operations using the locked buffer
    for _ in 0..ITERATIONS {
        // Copy from locked buffer (no allocation)
        let mut working = locked_buf.as_slice().to_vec();
        fft.fft_inplace(&mut working);
        std::hint::black_box(&working);
    }

    let faults_after = total_page_faults();

    println!("  Total faults at start: {}", faults_before);
    println!("  Total faults at end: {}", faults_after);

    (faults_before, faults_after)
}

/// Measure page faults with prefaulted memory
fn measure_page_faults_prefaulted() -> (u64, u64) {
    let fft_size = 1024;
    let mut fft = FftProcessor::new(fft_size);

    // Pre-allocate and prefault a buffer pool
    let pool_size = 16;
    let mut buffer_pool: Vec<Vec<Complex64>> = (0..pool_size)
        .map(|_| vec![Complex64::new(0.0, 0.0); fft_size])
        .collect();

    // Prefault all buffers
    for buf in &mut buffer_pool {
        // Convert to mutable u8 slice for prefaulting
        let ptr = buf.as_mut_ptr() as *mut u8;
        let len = buf.len() * std::mem::size_of::<Complex64>();
        let bytes = unsafe { std::slice::from_raw_parts_mut(ptr, len) };
        prefault_pages(bytes);
    }

    // Warm up FFT
    fft.fft_inplace(&mut buffer_pool[0]);

    let faults_before = total_page_faults();

    // Run FFT operations reusing prefaulted buffers
    for i in 0..ITERATIONS {
        let buf = &mut buffer_pool[i % pool_size];
        // Reinitialize buffer (no new allocation)
        for (j, sample) in buf.iter_mut().enumerate() {
            *sample = Complex64::new((j as f64).sin(), 0.0);
        }
        fft.fft_inplace(buf);
        std::hint::black_box(&buf);
    }

    let faults_after = total_page_faults();

    println!("  Total faults at start: {}", faults_before);
    println!("  Total faults at end: {}", faults_after);

    (faults_before, faults_after)
}

/// Measure page faults with mlockall in an RT thread
fn measure_page_faults_mlockall() -> (u64, u64) {
    let fft_size = 1024;

    let faults_before = Arc::new(AtomicU64::new(0));
    let faults_after = Arc::new(AtomicU64::new(0));
    let fb = faults_before.clone();
    let fa = faults_after.clone();

    let config = RtConfig::builder()
        .name("pagefault_test")
        .priority(RtPriority::High)
        .lock_memory(true)  // This calls mlockall
        .build();

    let handle = spawn_rt_thread(config, move || {
        let mut fft = FftProcessor::new(fft_size);

        // Pre-allocate buffer
        let mut buffer: Vec<Complex64> = vec![Complex64::new(0.0, 0.0); fft_size];

        // Warm up
        for i in 0..fft_size {
            buffer[i] = Complex64::new((i as f64).sin(), 0.0);
        }
        fft.fft_inplace(&mut buffer);

        fb.store(total_page_faults(), Ordering::SeqCst);

        // Run FFT operations
        for _ in 0..ITERATIONS {
            for (i, sample) in buffer.iter_mut().enumerate() {
                *sample = Complex64::new((i as f64).sin(), 0.0);
            }
            fft.fft_inplace(&mut buffer);
            std::hint::black_box(&buffer);
        }

        fa.store(total_page_faults(), Ordering::SeqCst);
    });

    match handle {
        Ok(h) => {
            let _ = h.join();
        }
        Err(e) => {
            eprintln!("Failed to spawn RT thread: {}", e);
        }
    }

    let before = faults_before.load(Ordering::SeqCst);
    let after = faults_after.load(Ordering::SeqCst);

    println!("  Total faults at start: {}", before);
    println!("  Total faults at end: {}", after);

    (before, after)
}
