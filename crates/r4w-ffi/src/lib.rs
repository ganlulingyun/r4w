//! # R4W FFI - C/C++ Bindings for R4W DSP
//!
//! This crate provides a C-compatible API for R4W's digital signal processing
//! functions. It enables C and C++ programs to use R4W's high-performance
//! Rust DSP algorithms.
//!
//! ## Features
//!
//! - **Complex I/Q Operations**: Create, manipulate, and analyze complex samples
//! - **FFT Processing**: Forward and inverse FFT with peak detection
//! - **Chirp Generation**: Create chirp signals for LoRa and radar applications
//! - **Ring Buffer**: Lock-free SPSC queue for real-time streaming
//! - **Filters**: Pulse shaping and matched filtering
//!
//! ## Building
//!
//! ```bash
//! cargo build --release -p r4w-ffi
//! ```
//!
//! This generates:
//! - `target/release/libr4w.so` (Linux shared library)
//! - `target/release/libr4w.a` (Linux static library)
//! - `crates/r4w-ffi/include/r4w.h` (C header)
//!
//! ## Usage from C
//!
//! ```c
//! #include <r4w.h>
//!
//! int main() {
//!     // Create FFT processor
//!     r4w_fft_t* fft = r4w_fft_new(1024);
//!
//!     // Allocate buffer
//!     r4w_complex_t* buffer = malloc(1024 * sizeof(r4w_complex_t));
//!
//!     // Fill with signal...
//!
//!     // Process FFT
//!     r4w_fft_forward(fft, buffer, 1024);
//!
//!     // Find peak
//!     size_t bin;
//!     double magnitude, phase;
//!     r4w_fft_find_peak(buffer, 1024, &bin, &magnitude, &phase);
//!
//!     // Cleanup
//!     r4w_fft_free(fft);
//!     free(buffer);
//! }
//! ```

use std::ffi::c_char;
use std::ptr;
use std::slice;

use num_complex::Complex64;
use r4w_core::chirp::ChirpGenerator;
use r4w_core::fft_utils::FftProcessor;
use r4w_core::params::LoRaParams;
use r4w_core::rt::RingBuffer;

// =============================================================================
// Types
// =============================================================================

/// Complex I/Q sample (matches C's complex double or struct { double re, im; })
#[repr(C)]
#[derive(Debug, Clone, Copy, Default)]
pub struct R4wComplex {
    /// Real (In-phase) component
    pub re: f64,
    /// Imaginary (Quadrature) component
    pub im: f64,
}

impl From<Complex64> for R4wComplex {
    fn from(c: Complex64) -> Self {
        R4wComplex { re: c.re, im: c.im }
    }
}

impl From<R4wComplex> for Complex64 {
    fn from(c: R4wComplex) -> Self {
        Complex64::new(c.re, c.im)
    }
}

/// Error codes returned by R4W functions
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum R4wError {
    /// Success (no error)
    Ok = 0,
    /// Null pointer passed
    NullPointer = 1,
    /// Invalid size or capacity
    InvalidSize = 2,
    /// Buffer is full
    BufferFull = 3,
    /// Buffer is empty
    BufferEmpty = 4,
    /// Invalid parameter value
    InvalidParameter = 5,
    /// Memory allocation failed
    AllocationFailed = 6,
    /// Operation not supported
    NotSupported = 7,
}

// =============================================================================
// Version and Initialization
// =============================================================================

/// Get R4W library version string
///
/// Returns a null-terminated string like "0.1.0"
#[no_mangle]
pub extern "C" fn r4w_version() -> *const c_char {
    // Static string with null terminator
    b"0.1.0\0".as_ptr() as *const c_char
}

/// Initialize the R4W library
///
/// Call once at program start. Currently a no-op but reserved for future use.
#[no_mangle]
pub extern "C" fn r4w_init() -> R4wError {
    R4wError::Ok
}

// =============================================================================
// Complex Operations
// =============================================================================

/// Create a complex number from real and imaginary parts
#[no_mangle]
pub extern "C" fn r4w_complex_new(re: f64, im: f64) -> R4wComplex {
    R4wComplex { re, im }
}

/// Create a complex number from polar coordinates (magnitude, phase in radians)
#[no_mangle]
pub extern "C" fn r4w_complex_from_polar(magnitude: f64, phase: f64) -> R4wComplex {
    R4wComplex {
        re: magnitude * phase.cos(),
        im: magnitude * phase.sin(),
    }
}

/// Get magnitude of a complex number
#[no_mangle]
pub extern "C" fn r4w_complex_magnitude(c: R4wComplex) -> f64 {
    (c.re * c.re + c.im * c.im).sqrt()
}

/// Get phase (argument) of a complex number in radians
#[no_mangle]
pub extern "C" fn r4w_complex_phase(c: R4wComplex) -> f64 {
    c.im.atan2(c.re)
}

/// Get power (magnitude squared) of a complex number
#[no_mangle]
pub extern "C" fn r4w_complex_power(c: R4wComplex) -> f64 {
    c.re * c.re + c.im * c.im
}

/// Conjugate of a complex number
#[no_mangle]
pub extern "C" fn r4w_complex_conj(c: R4wComplex) -> R4wComplex {
    R4wComplex { re: c.re, im: -c.im }
}

/// Multiply two complex numbers
#[no_mangle]
pub extern "C" fn r4w_complex_mul(a: R4wComplex, b: R4wComplex) -> R4wComplex {
    R4wComplex {
        re: a.re * b.re - a.im * b.im,
        im: a.re * b.im + a.im * b.re,
    }
}

/// Add two complex numbers
#[no_mangle]
pub extern "C" fn r4w_complex_add(a: R4wComplex, b: R4wComplex) -> R4wComplex {
    R4wComplex {
        re: a.re + b.re,
        im: a.im + b.im,
    }
}

/// Compute average power of a buffer of samples
///
/// # Safety
/// `buffer` must point to at least `len` elements
#[no_mangle]
pub unsafe extern "C" fn r4w_complex_average_power(
    buffer: *const R4wComplex,
    len: usize,
) -> f64 {
    if buffer.is_null() || len == 0 {
        return 0.0;
    }

    let samples = slice::from_raw_parts(buffer, len);
    let sum: f64 = samples.iter().map(|s| s.re * s.re + s.im * s.im).sum();
    sum / len as f64
}

/// Normalize a buffer of samples to unit power (in-place)
///
/// # Safety
/// `buffer` must point to at least `len` elements
#[no_mangle]
pub unsafe extern "C" fn r4w_complex_normalize(
    buffer: *mut R4wComplex,
    len: usize,
) -> R4wError {
    if buffer.is_null() {
        return R4wError::NullPointer;
    }
    if len == 0 {
        return R4wError::InvalidSize;
    }

    let samples = slice::from_raw_parts_mut(buffer, len);
    let avg_power: f64 = samples.iter().map(|s| s.re * s.re + s.im * s.im).sum::<f64>() / len as f64;

    if avg_power > 0.0 {
        let scale = 1.0 / avg_power.sqrt();
        for s in samples.iter_mut() {
            s.re *= scale;
            s.im *= scale;
        }
    }

    R4wError::Ok
}

// =============================================================================
// FFT Processing
// =============================================================================

/// Opaque FFT processor handle
pub struct R4wFftProcessor {
    inner: FftProcessor,
}

/// Create a new FFT processor for the given size
///
/// Size should be a power of 2 for best performance.
/// Returns NULL on failure.
#[no_mangle]
pub extern "C" fn r4w_fft_new(size: usize) -> *mut R4wFftProcessor {
    if size < 2 {
        return ptr::null_mut();
    }

    let processor = R4wFftProcessor {
        inner: FftProcessor::new(size),
    };

    Box::into_raw(Box::new(processor))
}

/// Free an FFT processor
///
/// # Safety
/// `fft` must be a valid pointer returned by `r4w_fft_new`
#[no_mangle]
pub unsafe extern "C" fn r4w_fft_free(fft: *mut R4wFftProcessor) {
    if !fft.is_null() {
        drop(Box::from_raw(fft));
    }
}

/// Get the size of an FFT processor
#[no_mangle]
pub unsafe extern "C" fn r4w_fft_size(fft: *const R4wFftProcessor) -> usize {
    if fft.is_null() {
        return 0;
    }
    (*fft).inner.size()
}

/// Compute forward FFT in-place
///
/// # Safety
/// - `fft` must be valid
/// - `buffer` must point to at least `len` elements
/// - `len` must match the FFT size
#[no_mangle]
pub unsafe extern "C" fn r4w_fft_forward(
    fft: *mut R4wFftProcessor,
    buffer: *mut R4wComplex,
    len: usize,
) -> R4wError {
    if fft.is_null() || buffer.is_null() {
        return R4wError::NullPointer;
    }

    let processor = &mut (*fft).inner;
    if len != processor.size() {
        return R4wError::InvalidSize;
    }

    // Reinterpret R4wComplex as Complex64 (they have the same layout)
    let buf = slice::from_raw_parts_mut(buffer as *mut Complex64, len);
    processor.fft_inplace(buf);

    R4wError::Ok
}

/// Compute inverse FFT in-place (includes 1/N normalization)
///
/// # Safety
/// - `fft` must be valid
/// - `buffer` must point to at least `len` elements
/// - `len` must match the FFT size
#[no_mangle]
pub unsafe extern "C" fn r4w_fft_inverse(
    fft: *mut R4wFftProcessor,
    buffer: *mut R4wComplex,
    len: usize,
) -> R4wError {
    if fft.is_null() || buffer.is_null() {
        return R4wError::NullPointer;
    }

    let processor = &mut (*fft).inner;
    if len != processor.size() {
        return R4wError::InvalidSize;
    }

    let buf = slice::from_raw_parts_mut(buffer as *mut Complex64, len);
    processor.ifft_inplace(buf);

    R4wError::Ok
}

/// Find the peak in an FFT magnitude spectrum
///
/// Returns the bin index, magnitude, and phase of the maximum.
///
/// # Safety
/// `buffer` must point to at least `len` elements
#[no_mangle]
pub unsafe extern "C" fn r4w_fft_find_peak(
    buffer: *const R4wComplex,
    len: usize,
    out_bin: *mut usize,
    out_magnitude: *mut f64,
    out_phase: *mut f64,
) -> R4wError {
    if buffer.is_null() {
        return R4wError::NullPointer;
    }
    if len == 0 {
        return R4wError::InvalidSize;
    }

    let samples = slice::from_raw_parts(buffer as *const Complex64, len);
    let (bin, mag, phase) = FftProcessor::find_peak(samples);

    if !out_bin.is_null() {
        *out_bin = bin;
    }
    if !out_magnitude.is_null() {
        *out_magnitude = mag;
    }
    if !out_phase.is_null() {
        *out_phase = phase;
    }

    R4wError::Ok
}

/// Find peak with parabolic interpolation for sub-bin resolution
///
/// Returns interpolated bin index and magnitude.
///
/// # Safety
/// `buffer` must point to at least `len` elements
#[no_mangle]
pub unsafe extern "C" fn r4w_fft_find_peak_interpolated(
    buffer: *const R4wComplex,
    len: usize,
    out_bin: *mut f64,
    out_magnitude: *mut f64,
) -> R4wError {
    if buffer.is_null() {
        return R4wError::NullPointer;
    }
    if len == 0 {
        return R4wError::InvalidSize;
    }

    let samples = slice::from_raw_parts(buffer as *const Complex64, len);
    let (bin, mag) = FftProcessor::find_peak_interpolated(samples);

    if !out_bin.is_null() {
        *out_bin = bin;
    }
    if !out_magnitude.is_null() {
        *out_magnitude = mag;
    }

    R4wError::Ok
}

/// Compute magnitude spectrum
///
/// # Safety
/// - `input` must point to at least `len` elements
/// - `output` must point to at least `len` elements
#[no_mangle]
pub unsafe extern "C" fn r4w_fft_magnitude_spectrum(
    input: *const R4wComplex,
    output: *mut f64,
    len: usize,
) -> R4wError {
    if input.is_null() || output.is_null() {
        return R4wError::NullPointer;
    }
    if len == 0 {
        return R4wError::InvalidSize;
    }

    let inp = slice::from_raw_parts(input as *const Complex64, len);
    let out = slice::from_raw_parts_mut(output, len);

    for (i, &c) in inp.iter().enumerate() {
        out[i] = c.norm();
    }

    R4wError::Ok
}

/// Compute power spectrum in dB
///
/// # Safety
/// - `input` must point to at least `len` elements
/// - `output` must point to at least `len` elements
#[no_mangle]
pub unsafe extern "C" fn r4w_fft_power_spectrum_db(
    input: *const R4wComplex,
    output: *mut f64,
    len: usize,
) -> R4wError {
    if input.is_null() || output.is_null() {
        return R4wError::NullPointer;
    }
    if len == 0 {
        return R4wError::InvalidSize;
    }

    let inp = slice::from_raw_parts(input as *const Complex64, len);
    let out = slice::from_raw_parts_mut(output, len);

    for (i, &c) in inp.iter().enumerate() {
        let power = c.norm_sqr();
        out[i] = if power > 1e-20 {
            10.0 * power.log10()
        } else {
            -200.0
        };
    }

    R4wError::Ok
}

// =============================================================================
// Chirp Generation
// =============================================================================

/// Opaque chirp generator handle
pub struct R4wChirpGenerator {
    inner: ChirpGenerator,
}

/// Create a new chirp generator
///
/// # Arguments
/// * `spreading_factor` - LoRa spreading factor (5-12)
/// * `bandwidth` - Bandwidth in Hz (125000, 250000, or 500000)
/// * `oversample` - Oversampling factor (1-8, typically 1)
#[no_mangle]
pub extern "C" fn r4w_chirp_new(
    spreading_factor: u8,
    bandwidth: u32,
    oversample: usize,
) -> *mut R4wChirpGenerator {
    if !(5..=12).contains(&spreading_factor) {
        return ptr::null_mut();
    }
    if bandwidth == 0 || oversample == 0 {
        return ptr::null_mut();
    }

    let params = LoRaParams::builder()
        .spreading_factor(spreading_factor)
        .bandwidth(bandwidth)
        .oversample(oversample)
        .build();

    let generator = ChirpGenerator::new(params);
    Box::into_raw(Box::new(R4wChirpGenerator { inner: generator }))
}

/// Free a chirp generator
#[no_mangle]
pub unsafe extern "C" fn r4w_chirp_free(chirp: *mut R4wChirpGenerator) {
    if !chirp.is_null() {
        drop(Box::from_raw(chirp));
    }
}

/// Get samples per symbol for a chirp generator
#[no_mangle]
pub unsafe extern "C" fn r4w_chirp_samples_per_symbol(chirp: *const R4wChirpGenerator) -> usize {
    if chirp.is_null() {
        return 0;
    }
    (*chirp).inner.params().samples_per_symbol()
}

/// Get bandwidth in Hz
#[no_mangle]
pub unsafe extern "C" fn r4w_chirp_bandwidth(chirp: *const R4wChirpGenerator) -> f64 {
    if chirp.is_null() {
        return 0.0;
    }
    (*chirp).inner.params().bw.hz()
}

/// Generate an upchirp and store in buffer
///
/// # Safety
/// `buffer` must point to at least `r4w_chirp_samples_per_symbol()` elements
#[no_mangle]
pub unsafe extern "C" fn r4w_chirp_generate_upchirp(
    chirp: *const R4wChirpGenerator,
    buffer: *mut R4wComplex,
    len: usize,
) -> R4wError {
    if chirp.is_null() || buffer.is_null() {
        return R4wError::NullPointer;
    }

    let generator = &(*chirp).inner;
    let samples = generator.base_upchirp();
    let required = samples.len();

    if len < required {
        return R4wError::InvalidSize;
    }

    let out = slice::from_raw_parts_mut(buffer, required);

    for (i, &s) in samples.iter().enumerate() {
        out[i] = R4wComplex { re: s.re, im: s.im };
    }

    R4wError::Ok
}

/// Generate a downchirp and store in buffer
///
/// # Safety
/// `buffer` must point to at least `r4w_chirp_samples_per_symbol()` elements
#[no_mangle]
pub unsafe extern "C" fn r4w_chirp_generate_downchirp(
    chirp: *const R4wChirpGenerator,
    buffer: *mut R4wComplex,
    len: usize,
) -> R4wError {
    if chirp.is_null() || buffer.is_null() {
        return R4wError::NullPointer;
    }

    let generator = &(*chirp).inner;
    let samples = generator.base_downchirp();
    let required = samples.len();

    if len < required {
        return R4wError::InvalidSize;
    }

    let out = slice::from_raw_parts_mut(buffer, required);

    for (i, &s) in samples.iter().enumerate() {
        out[i] = R4wComplex { re: s.re, im: s.im };
    }

    R4wError::Ok
}

/// Generate a modulated chirp for a given symbol
///
/// # Safety
/// `buffer` must point to at least `r4w_chirp_samples_per_symbol()` elements
#[no_mangle]
pub unsafe extern "C" fn r4w_chirp_modulate_symbol(
    chirp: *const R4wChirpGenerator,
    symbol: u16,
    buffer: *mut R4wComplex,
    len: usize,
) -> R4wError {
    if chirp.is_null() || buffer.is_null() {
        return R4wError::NullPointer;
    }

    let generator = &(*chirp).inner;
    let required = generator.params().samples_per_symbol();

    if len < required {
        return R4wError::InvalidSize;
    }

    let samples = generator.generate_symbol_chirp_fast(symbol);
    let out = slice::from_raw_parts_mut(buffer, required);

    for (i, &s) in samples.iter().enumerate() {
        out[i] = R4wComplex { re: s.re, im: s.im };
    }

    R4wError::Ok
}

// =============================================================================
// Lock-Free Ring Buffer (for real-time streaming)
// =============================================================================

/// Opaque ring buffer handle
pub struct R4wRingBuffer {
    inner: RingBuffer<R4wComplex>,
}

/// Create a new ring buffer
///
/// Capacity is rounded up to the next power of 2.
#[no_mangle]
pub extern "C" fn r4w_ringbuffer_new(capacity: usize) -> *mut R4wRingBuffer {
    if capacity < 2 {
        return ptr::null_mut();
    }

    let rb = R4wRingBuffer {
        inner: RingBuffer::new(capacity),
    };

    Box::into_raw(Box::new(rb))
}

/// Free a ring buffer
#[no_mangle]
pub unsafe extern "C" fn r4w_ringbuffer_free(rb: *mut R4wRingBuffer) {
    if !rb.is_null() {
        drop(Box::from_raw(rb));
    }
}

/// Get ring buffer capacity
#[no_mangle]
pub unsafe extern "C" fn r4w_ringbuffer_capacity(rb: *const R4wRingBuffer) -> usize {
    if rb.is_null() {
        return 0;
    }
    (*rb).inner.capacity()
}

/// Get current number of elements in ring buffer
#[no_mangle]
pub unsafe extern "C" fn r4w_ringbuffer_len(rb: *const R4wRingBuffer) -> usize {
    if rb.is_null() {
        return 0;
    }
    (*rb).inner.len()
}

/// Check if ring buffer is empty
#[no_mangle]
pub unsafe extern "C" fn r4w_ringbuffer_is_empty(rb: *const R4wRingBuffer) -> bool {
    if rb.is_null() {
        return true;
    }
    (*rb).inner.is_empty()
}

/// Check if ring buffer is full
#[no_mangle]
pub unsafe extern "C" fn r4w_ringbuffer_is_full(rb: *const R4wRingBuffer) -> bool {
    if rb.is_null() {
        return false;
    }
    (*rb).inner.is_full()
}

/// Push a single sample to the ring buffer
///
/// Returns R4wError::BufferFull if the buffer is full.
#[no_mangle]
pub unsafe extern "C" fn r4w_ringbuffer_push(
    rb: *const R4wRingBuffer,
    sample: R4wComplex,
) -> R4wError {
    if rb.is_null() {
        return R4wError::NullPointer;
    }

    match (*rb).inner.push(sample) {
        Ok(()) => R4wError::Ok,
        Err(_) => R4wError::BufferFull,
    }
}

/// Pop a single sample from the ring buffer
///
/// Returns R4wError::BufferEmpty if the buffer is empty.
#[no_mangle]
pub unsafe extern "C" fn r4w_ringbuffer_pop(
    rb: *const R4wRingBuffer,
    out: *mut R4wComplex,
) -> R4wError {
    if rb.is_null() || out.is_null() {
        return R4wError::NullPointer;
    }

    match (*rb).inner.pop() {
        Some(sample) => {
            *out = sample;
            R4wError::Ok
        }
        None => R4wError::BufferEmpty,
    }
}

/// Push multiple samples to the ring buffer
///
/// Returns the number of samples actually pushed.
///
/// # Safety
/// `buffer` must point to at least `len` elements
#[no_mangle]
pub unsafe extern "C" fn r4w_ringbuffer_push_slice(
    rb: *const R4wRingBuffer,
    buffer: *const R4wComplex,
    len: usize,
) -> usize {
    if rb.is_null() || buffer.is_null() {
        return 0;
    }

    let samples = slice::from_raw_parts(buffer, len);
    (*rb).inner.push_slice(samples)
}

/// Pop multiple samples from the ring buffer
///
/// Returns the number of samples actually popped.
///
/// # Safety
/// `buffer` must point to at least `len` elements
#[no_mangle]
pub unsafe extern "C" fn r4w_ringbuffer_pop_slice(
    rb: *const R4wRingBuffer,
    buffer: *mut R4wComplex,
    len: usize,
) -> usize {
    if rb.is_null() || buffer.is_null() {
        return 0;
    }

    let out = slice::from_raw_parts_mut(buffer, len);
    (*rb).inner.pop_slice(out)
}

// =============================================================================
// Signal Generation Utilities
// =============================================================================

/// Generate a complex sinusoid (cisoid) at the given frequency
///
/// # Arguments
/// * `frequency` - Frequency in Hz
/// * `sample_rate` - Sample rate in Hz
/// * `buffer` - Output buffer
/// * `len` - Number of samples to generate
///
/// # Safety
/// `buffer` must point to at least `len` elements
#[no_mangle]
pub unsafe extern "C" fn r4w_generate_tone(
    frequency: f64,
    sample_rate: f64,
    buffer: *mut R4wComplex,
    len: usize,
) -> R4wError {
    if buffer.is_null() {
        return R4wError::NullPointer;
    }
    if len == 0 || sample_rate <= 0.0 {
        return R4wError::InvalidParameter;
    }

    let out = slice::from_raw_parts_mut(buffer, len);
    let phase_inc = 2.0 * std::f64::consts::PI * frequency / sample_rate;

    for (i, sample) in out.iter_mut().enumerate() {
        let phase = phase_inc * i as f64;
        *sample = R4wComplex {
            re: phase.cos(),
            im: phase.sin(),
        };
    }

    R4wError::Ok
}

/// Add AWGN (Additive White Gaussian Noise) to a signal
///
/// # Arguments
/// * `buffer` - Signal buffer (modified in-place)
/// * `len` - Number of samples
/// * `snr_db` - Signal-to-noise ratio in dB
///
/// # Safety
/// `buffer` must point to at least `len` elements
#[no_mangle]
pub unsafe extern "C" fn r4w_add_awgn(
    buffer: *mut R4wComplex,
    len: usize,
    snr_db: f64,
) -> R4wError {
    if buffer.is_null() {
        return R4wError::NullPointer;
    }
    if len == 0 {
        return R4wError::InvalidSize;
    }

    // Use a simple LCG for deterministic noise (not cryptographic quality)
    // For production, integrate with rand crate
    let mut seed: u64 = 12345;
    let lcg_next = |s: &mut u64| -> f64 {
        *s = s.wrapping_mul(6364136223846793005).wrapping_add(1);
        (*s as f64) / (u64::MAX as f64)
    };

    let samples = slice::from_raw_parts_mut(buffer, len);

    // Calculate signal power
    let signal_power: f64 = samples.iter().map(|s| s.re * s.re + s.im * s.im).sum::<f64>() / len as f64;

    // Calculate noise power from SNR
    let snr_linear = 10.0_f64.powf(snr_db / 10.0);
    let noise_power = signal_power / snr_linear;
    let noise_std = (noise_power / 2.0).sqrt(); // Divide by 2 for I and Q

    // Add noise using Box-Muller transform
    for i in (0..len).step_by(2) {
        let u1 = lcg_next(&mut seed).max(1e-10);
        let u2 = lcg_next(&mut seed);
        let mag = (-2.0 * u1.ln()).sqrt() * noise_std;
        let phase = 2.0 * std::f64::consts::PI * u2;

        samples[i].re += mag * phase.cos();
        samples[i].im += mag * phase.sin();

        if i + 1 < len {
            let u1 = lcg_next(&mut seed).max(1e-10);
            let u2 = lcg_next(&mut seed);
            let mag = (-2.0 * u1.ln()).sqrt() * noise_std;
            let phase = 2.0 * std::f64::consts::PI * u2;

            samples[i + 1].re += mag * phase.cos();
            samples[i + 1].im += mag * phase.sin();
        }
    }

    R4wError::Ok
}

/// Frequency shift a signal
///
/// # Arguments
/// * `buffer` - Signal buffer (modified in-place)
/// * `len` - Number of samples
/// * `frequency_offset` - Frequency shift in Hz
/// * `sample_rate` - Sample rate in Hz
///
/// # Safety
/// `buffer` must point to at least `len` elements
#[no_mangle]
pub unsafe extern "C" fn r4w_frequency_shift(
    buffer: *mut R4wComplex,
    len: usize,
    frequency_offset: f64,
    sample_rate: f64,
) -> R4wError {
    if buffer.is_null() {
        return R4wError::NullPointer;
    }
    if len == 0 || sample_rate <= 0.0 {
        return R4wError::InvalidParameter;
    }

    let samples = slice::from_raw_parts_mut(buffer, len);
    let phase_inc = 2.0 * std::f64::consts::PI * frequency_offset / sample_rate;

    for (i, sample) in samples.iter_mut().enumerate() {
        let phase = phase_inc * i as f64;
        let shift = R4wComplex {
            re: phase.cos(),
            im: phase.sin(),
        };
        // Complex multiplication
        let new = r4w_complex_mul(*sample, shift);
        *sample = new;
    }

    R4wError::Ok
}

// =============================================================================
// Waveform Modulation/Demodulation (MF-032)
// =============================================================================

use r4w_core::waveform::{psk::PSK, lora::LoRa, Waveform, CommonParams};
use r4w_core::params::{SpreadingFactor, Bandwidth, CodingRate};

/// Waveform type enumeration
#[repr(C)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum R4wWaveformType {
    /// Binary Phase Shift Keying
    Bpsk = 0,
    /// Quadrature Phase Shift Keying
    Qpsk = 1,
    /// LoRa Chirp Spread Spectrum
    LoRa = 2,
}

/// Opaque waveform handle
pub struct R4wWaveform {
    inner: Box<dyn Waveform>,
    waveform_type: R4wWaveformType,
}

/// Create a BPSK waveform
///
/// # Arguments
/// * `sample_rate` - Sample rate in Hz
/// * `symbol_rate` - Symbol rate in baud
#[no_mangle]
pub extern "C" fn r4w_waveform_bpsk_new(
    sample_rate: f64,
    symbol_rate: f64,
) -> *mut R4wWaveform {
    if sample_rate <= 0.0 || symbol_rate <= 0.0 {
        return ptr::null_mut();
    }

    let common = CommonParams {
        sample_rate,
        carrier_freq: 0.0,
        amplitude: 1.0,
    };

    let psk = PSK::new_bpsk(common, symbol_rate);
    Box::into_raw(Box::new(R4wWaveform {
        inner: Box::new(psk),
        waveform_type: R4wWaveformType::Bpsk,
    }))
}

/// Create a QPSK waveform
///
/// # Arguments
/// * `sample_rate` - Sample rate in Hz
/// * `symbol_rate` - Symbol rate in baud
#[no_mangle]
pub extern "C" fn r4w_waveform_qpsk_new(
    sample_rate: f64,
    symbol_rate: f64,
) -> *mut R4wWaveform {
    if sample_rate <= 0.0 || symbol_rate <= 0.0 {
        return ptr::null_mut();
    }

    let common = CommonParams {
        sample_rate,
        carrier_freq: 0.0,
        amplitude: 1.0,
    };

    let psk = PSK::new_qpsk(common, symbol_rate);
    Box::into_raw(Box::new(R4wWaveform {
        inner: Box::new(psk),
        waveform_type: R4wWaveformType::Qpsk,
    }))
}

/// Create a LoRa waveform
///
/// # Arguments
/// * `spreading_factor` - Spreading factor (5-12)
/// * `bandwidth` - Bandwidth in Hz (125000, 250000, 500000)
/// * `sample_rate` - Sample rate in Hz (typically 1MHz)
#[no_mangle]
pub extern "C" fn r4w_waveform_lora_new(
    spreading_factor: u8,
    bandwidth: u32,
    sample_rate: f64,
) -> *mut R4wWaveform {
    if sample_rate <= 0.0 {
        return ptr::null_mut();
    }

    // Convert to enum types
    let sf = match spreading_factor {
        5 => SpreadingFactor::SF5,
        6 => SpreadingFactor::SF6,
        7 => SpreadingFactor::SF7,
        8 => SpreadingFactor::SF8,
        9 => SpreadingFactor::SF9,
        10 => SpreadingFactor::SF10,
        11 => SpreadingFactor::SF11,
        12 => SpreadingFactor::SF12,
        _ => return ptr::null_mut(),
    };

    let bw = match bandwidth {
        125_000 => Bandwidth::Bw125kHz,
        250_000 => Bandwidth::Bw250kHz,
        500_000 => Bandwidth::Bw500kHz,
        _ => return ptr::null_mut(),
    };

    let lora = LoRa::new(sample_rate, sf, bw, CodingRate::CR4_5);
    Box::into_raw(Box::new(R4wWaveform {
        inner: Box::new(lora),
        waveform_type: R4wWaveformType::LoRa,
    }))
}

/// Free a waveform
#[no_mangle]
pub unsafe extern "C" fn r4w_waveform_free(waveform: *mut R4wWaveform) {
    if !waveform.is_null() {
        drop(Box::from_raw(waveform));
    }
}

/// Get waveform type
#[no_mangle]
pub unsafe extern "C" fn r4w_waveform_type(waveform: *const R4wWaveform) -> R4wWaveformType {
    if waveform.is_null() {
        return R4wWaveformType::Bpsk; // Default
    }
    (*waveform).waveform_type
}

/// Get samples per symbol
#[no_mangle]
pub unsafe extern "C" fn r4w_waveform_samples_per_symbol(waveform: *const R4wWaveform) -> usize {
    if waveform.is_null() {
        return 0;
    }
    (*waveform).inner.samples_per_symbol()
}

/// Get waveform name
///
/// Returns a pointer to a static string. Do not free.
#[no_mangle]
pub unsafe extern "C" fn r4w_waveform_name(waveform: *const R4wWaveform) -> *const c_char {
    if waveform.is_null() {
        return ptr::null();
    }
    let _info = (*waveform).inner.info();
    // Return a static string based on type (we can't return info.name directly as it's not 'static)
    match (*waveform).waveform_type {
        R4wWaveformType::Bpsk => b"BPSK\0".as_ptr() as *const c_char,
        R4wWaveformType::Qpsk => b"QPSK\0".as_ptr() as *const c_char,
        R4wWaveformType::LoRa => b"LoRa\0".as_ptr() as *const c_char,
    }
}

/// Modulate data into I/Q samples
///
/// # Arguments
/// * `waveform` - Waveform handle
/// * `data` - Input data bytes
/// * `data_len` - Number of data bytes
/// * `output` - Output buffer for I/Q samples
/// * `output_len` - Maximum output buffer size
/// * `samples_written` - Output: number of samples actually written
///
/// # Returns
/// * `R4wError::Ok` on success
/// * `R4wError::InvalidSize` if output buffer is too small
///
/// # Safety
/// * `data` must point to at least `data_len` bytes
/// * `output` must point to at least `output_len` elements
#[no_mangle]
pub unsafe extern "C" fn r4w_waveform_modulate(
    waveform: *const R4wWaveform,
    data: *const u8,
    data_len: usize,
    output: *mut R4wComplex,
    output_len: usize,
    samples_written: *mut usize,
) -> R4wError {
    if waveform.is_null() || data.is_null() || output.is_null() {
        return R4wError::NullPointer;
    }

    let input = slice::from_raw_parts(data, data_len);
    let samples = (*waveform).inner.modulate(input);

    if samples.len() > output_len {
        return R4wError::InvalidSize;
    }

    let out = slice::from_raw_parts_mut(output, samples.len());
    for (i, &s) in samples.iter().enumerate() {
        out[i] = R4wComplex { re: s.re, im: s.im };
    }

    if !samples_written.is_null() {
        *samples_written = samples.len();
    }

    R4wError::Ok
}

/// Get required output buffer size for modulation
///
/// Returns the maximum number of samples that will be produced for the given input size.
#[no_mangle]
pub unsafe extern "C" fn r4w_waveform_modulate_size(
    waveform: *const R4wWaveform,
    data_len: usize,
) -> usize {
    if waveform.is_null() {
        return 0;
    }

    // Estimate based on samples per symbol and bits per symbol
    let samples_per_symbol = (*waveform).inner.samples_per_symbol();
    let bits_per_symbol = match (*waveform).waveform_type {
        R4wWaveformType::Bpsk => 1,
        R4wWaveformType::Qpsk => 2,
        R4wWaveformType::LoRa => 1, // Variable, but 1 byte per symbol is conservative
    };

    // Add some padding for preamble/sync
    let symbols = (data_len * 8 + bits_per_symbol - 1) / bits_per_symbol;
    (symbols + 16) * samples_per_symbol // +16 for preamble
}

/// Demodulate I/Q samples back to data
///
/// # Arguments
/// * `waveform` - Waveform handle
/// * `samples` - Input I/Q samples
/// * `samples_len` - Number of input samples
/// * `output` - Output buffer for demodulated data
/// * `output_len` - Maximum output buffer size
/// * `bytes_written` - Output: number of bytes actually written
///
/// # Safety
/// * `samples` must point to at least `samples_len` elements
/// * `output` must point to at least `output_len` bytes
#[no_mangle]
pub unsafe extern "C" fn r4w_waveform_demodulate(
    waveform: *const R4wWaveform,
    samples: *const R4wComplex,
    samples_len: usize,
    output: *mut u8,
    output_len: usize,
    bytes_written: *mut usize,
) -> R4wError {
    if waveform.is_null() || samples.is_null() || output.is_null() {
        return R4wError::NullPointer;
    }
    if samples_len == 0 {
        if !bytes_written.is_null() {
            *bytes_written = 0;
        }
        return R4wError::Ok;
    }

    // Convert input samples
    let inp: Vec<Complex64> = slice::from_raw_parts(samples, samples_len)
        .iter()
        .map(|s| Complex64::new(s.re, s.im))
        .collect();

    let result = (*waveform).inner.demodulate(&inp);
    let bits = &result.bits;

    if bits.len() > output_len {
        return R4wError::InvalidSize;
    }

    let out = slice::from_raw_parts_mut(output, bits.len());
    out.copy_from_slice(bits);

    if !bytes_written.is_null() {
        *bytes_written = bits.len();
    }

    R4wError::Ok
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_complex_operations() {
        let a = r4w_complex_new(3.0, 4.0);
        assert!((r4w_complex_magnitude(a) - 5.0).abs() < 1e-10);
        assert!((r4w_complex_power(a) - 25.0).abs() < 1e-10);

        let b = r4w_complex_conj(a);
        assert_eq!(b.re, 3.0);
        assert_eq!(b.im, -4.0);
    }

    #[test]
    fn test_fft_roundtrip() {
        unsafe {
            let fft = r4w_fft_new(64);
            assert!(!fft.is_null());

            let mut buffer: Vec<R4wComplex> = (0..64)
                .map(|i| r4w_complex_new(i as f64, 0.0))
                .collect();

            let original: Vec<R4wComplex> = buffer.clone();

            // Forward then inverse should recover original
            assert_eq!(r4w_fft_forward(fft, buffer.as_mut_ptr(), 64), R4wError::Ok);
            assert_eq!(r4w_fft_inverse(fft, buffer.as_mut_ptr(), 64), R4wError::Ok);

            for (orig, recovered) in original.iter().zip(buffer.iter()) {
                assert!((orig.re - recovered.re).abs() < 1e-10);
                assert!((orig.im - recovered.im).abs() < 1e-10);
            }

            r4w_fft_free(fft);
        }
    }

    #[test]
    fn test_ringbuffer() {
        unsafe {
            let rb = r4w_ringbuffer_new(16);
            assert!(!rb.is_null());
            assert_eq!(r4w_ringbuffer_capacity(rb), 16);
            assert!(r4w_ringbuffer_is_empty(rb));

            // Push and pop
            let sample = r4w_complex_new(1.0, 2.0);
            assert_eq!(r4w_ringbuffer_push(rb, sample), R4wError::Ok);
            assert!(!r4w_ringbuffer_is_empty(rb));
            assert_eq!(r4w_ringbuffer_len(rb), 1);

            let mut out = R4wComplex::default();
            assert_eq!(r4w_ringbuffer_pop(rb, &mut out), R4wError::Ok);
            assert_eq!(out.re, 1.0);
            assert_eq!(out.im, 2.0);
            assert!(r4w_ringbuffer_is_empty(rb));

            r4w_ringbuffer_free(rb);
        }
    }

    #[test]
    fn test_chirp_generator() {
        unsafe {
            // SF=7, BW=125kHz, oversample=1
            let chirp = r4w_chirp_new(7, 125000, 1);
            assert!(!chirp.is_null());

            let samples_per_symbol = r4w_chirp_samples_per_symbol(chirp);
            assert!(samples_per_symbol > 0);
            assert_eq!(samples_per_symbol, 128); // 2^7 = 128

            let mut buffer = vec![R4wComplex::default(); samples_per_symbol];
            assert_eq!(
                r4w_chirp_generate_upchirp(chirp, buffer.as_mut_ptr(), samples_per_symbol),
                R4wError::Ok
            );

            // Verify chirp has unit magnitude (within tolerance)
            for sample in &buffer {
                let mag = r4w_complex_magnitude(*sample);
                assert!((mag - 1.0).abs() < 0.01);
            }

            r4w_chirp_free(chirp);
        }
    }

    #[test]
    fn test_waveform_bpsk() {
        unsafe {
            // Create BPSK waveform: 100kHz sample rate, 1kHz symbol rate
            let waveform = r4w_waveform_bpsk_new(100_000.0, 1000.0);
            assert!(!waveform.is_null());

            assert_eq!(r4w_waveform_type(waveform), R4wWaveformType::Bpsk);
            assert!(r4w_waveform_samples_per_symbol(waveform) > 0);

            // Test modulation
            let data = [0xAA, 0x55]; // Alternating bits pattern
            let max_size = r4w_waveform_modulate_size(waveform, data.len());
            assert!(max_size > 0);

            let mut samples = vec![R4wComplex::default(); max_size];
            let mut written = 0usize;
            assert_eq!(
                r4w_waveform_modulate(
                    waveform,
                    data.as_ptr(),
                    data.len(),
                    samples.as_mut_ptr(),
                    max_size,
                    &mut written
                ),
                R4wError::Ok
            );
            assert!(written > 0);

            r4w_waveform_free(waveform);
        }
    }

    #[test]
    fn test_waveform_qpsk() {
        unsafe {
            // Create QPSK waveform
            let waveform = r4w_waveform_qpsk_new(100_000.0, 1000.0);
            assert!(!waveform.is_null());
            assert_eq!(r4w_waveform_type(waveform), R4wWaveformType::Qpsk);

            r4w_waveform_free(waveform);
        }
    }

    #[test]
    fn test_waveform_lora() {
        unsafe {
            // Create LoRa waveform: SF=7, BW=125kHz, 1MHz sample rate
            let waveform = r4w_waveform_lora_new(7, 125_000, 1_000_000.0);
            assert!(!waveform.is_null());

            assert_eq!(r4w_waveform_type(waveform), R4wWaveformType::LoRa);

            // LoRa samples per symbol = 2^SF * oversample = 128 * 8 = 1024
            let sps = r4w_waveform_samples_per_symbol(waveform);
            assert!(sps >= 128); // At least 2^7

            r4w_waveform_free(waveform);
        }
    }
}
