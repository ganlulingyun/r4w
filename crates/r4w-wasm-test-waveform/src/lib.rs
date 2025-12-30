//! Simple test waveform for WASM sandbox validation.
//!
//! This module exports basic DSP functions that can be called from the sandbox.
//! Also demonstrates calling native DSP host functions for accelerated operations.

use std::alloc::{alloc as std_alloc, dealloc as std_dealloc, Layout};
use std::f32::consts::PI;

// =============================================================================
// Native DSP Host Function Imports
// =============================================================================
// These are provided by the r4w_dsp host module for accelerated DSP operations.

#[link(wasm_import_module = "r4w_dsp")]
extern "C" {
    /// Forward FFT: complex f32 input/output (interleaved re, im)
    fn fft(in_ptr: *const f32, out_ptr: *mut f32, len: i32);

    /// Inverse FFT: complex f32 input/output (interleaved re, im)
    fn ifft(in_ptr: *const f32, out_ptr: *mut f32, len: i32);

    /// Element-wise complex multiply: out = a * b
    fn complex_multiply(a_ptr: *const f32, b_ptr: *const f32, out_ptr: *mut f32, len: i32);

    /// Compute magnitudes of complex samples: out[i] = |in[i]|
    fn compute_magnitudes(in_ptr: *const f32, out_ptr: *mut f32, len: i32);

    /// Find index of peak magnitude in complex buffer
    fn find_peak(in_ptr: *const f32, len: i32) -> i32;

    /// Compute total power of complex samples
    fn total_power(in_ptr: *const f32, len: i32) -> f32;

    /// Generate Hann window of given length
    fn hann_window(out_ptr: *mut f32, len: i32);
}

/// Allocate memory for host to write into.
#[no_mangle]
pub extern "C" fn alloc(size: i32) -> i32 {
    let layout = Layout::from_size_align(size as usize, 8).unwrap();
    unsafe { std_alloc(layout) as i32 }
}

/// Free previously allocated memory.
#[no_mangle]
pub extern "C" fn dealloc(ptr: i32, size: i32) {
    let layout = Layout::from_size_align(size as usize, 8).unwrap();
    unsafe { std_dealloc(ptr as *mut u8, layout) };
}

/// Get the waveform name length.
#[no_mangle]
pub extern "C" fn waveform_name_len() -> i32 {
    b"TestBPSK".len() as i32
}

/// Write waveform name to the given pointer.
#[no_mangle]
pub extern "C" fn waveform_name(ptr: i32) {
    let name = b"TestBPSK";
    let dest = ptr as *mut u8;
    for (i, &byte) in name.iter().enumerate() {
        unsafe { *dest.add(i) = byte };
    }
}

/// Get bits per symbol for BPSK (always 1).
#[no_mangle]
pub extern "C" fn bits_per_symbol() -> i32 {
    1
}

/// Get sample rate.
#[no_mangle]
pub extern "C" fn sample_rate() -> i32 {
    48000
}

/// Get samples per symbol.
#[no_mangle]
pub extern "C" fn samples_per_symbol() -> i32 {
    48 // 1000 symbols/sec at 48kHz
}

/// Modulate a single bit to I/Q samples.
/// Returns pointer to 2 * samples_per_symbol f32 values (I, Q interleaved).
/// Caller must free with dealloc.
#[no_mangle]
pub extern "C" fn modulate_bit(bit: i32) -> i32 {
    let sps: usize = 48;
    let size = (sps * 2 * 4) as i32; // samples * (I + Q) * sizeof(f32)
    let ptr = alloc(size);

    // BPSK: bit 0 = phase 0, bit 1 = phase pi
    let phase = if bit != 0 { PI } else { 0.0 };

    let samples = ptr as *mut f32;
    for i in 0..sps {
        let t = i as f32 / 48000.0;
        let carrier_phase = 2.0 * PI * 1000.0 * t + phase;
        unsafe {
            // I component
            *samples.add(i * 2) = carrier_phase.cos();
            // Q component
            *samples.add(i * 2 + 1) = carrier_phase.sin();
        }
    }

    ptr
}

/// Compute magnitude of I/Q sample.
#[no_mangle]
pub extern "C" fn magnitude(i: f32, q: f32) -> f32 {
    (i * i + q * q).sqrt()
}

/// Compute phase of I/Q sample in radians.
#[no_mangle]
pub extern "C" fn phase(i: f32, q: f32) -> f32 {
    q.atan2(i)
}

/// Add two numbers (simple test function).
#[no_mangle]
pub extern "C" fn add(a: i32, b: i32) -> i32 {
    a + b
}

/// Multiply two numbers (simple test function).
#[no_mangle]
pub extern "C" fn multiply(a: i32, b: i32) -> i32 {
    a * b
}

/// Process a buffer of samples - apply gain.
/// input_ptr: pointer to f32 array
/// output_ptr: pointer to f32 array (preallocated)
/// len: number of f32 values
/// gain: multiplier
#[no_mangle]
pub extern "C" fn apply_gain(input_ptr: i32, output_ptr: i32, len: i32, gain: f32) {
    let input = input_ptr as *const f32;
    let output = output_ptr as *mut f32;

    for i in 0..len as usize {
        unsafe {
            *output.add(i) = *input.add(i) * gain;
        }
    }
}

/// Compute sum of f32 array.
#[no_mangle]
pub extern "C" fn sum_f32(ptr: i32, len: i32) -> f32 {
    let data = ptr as *const f32;
    let mut total = 0.0f32;
    for i in 0..len as usize {
        unsafe {
            total += *data.add(i);
        }
    }
    total
}

/// Compute energy (sum of squares) of f32 array.
#[no_mangle]
pub extern "C" fn energy_f32(ptr: i32, len: i32) -> f32 {
    let data = ptr as *const f32;
    let mut total = 0.0f32;
    for i in 0..len as usize {
        unsafe {
            let v = *data.add(i);
            total += v * v;
        }
    }
    total
}

/// Version number for ABI compatibility.
#[no_mangle]
pub extern "C" fn version() -> i32 {
    1
}

// =============================================================================
// DSP Host Function Wrappers (for testing)
// =============================================================================

/// Test FFT: perform forward FFT and return pointer to output buffer.
/// len: number of complex samples (buffer has 2*len floats)
/// Returns pointer to output buffer (caller must free with dealloc)
#[no_mangle]
pub extern "C" fn test_fft(input_ptr: i32, len: i32) -> i32 {
    let size = (len * 2 * 4) as i32; // complex samples * sizeof(f32)
    let output_ptr = alloc(size);

    unsafe {
        fft(input_ptr as *const f32, output_ptr as *mut f32, len);
    }

    output_ptr
}

/// Test IFFT: perform inverse FFT and return pointer to output buffer.
#[no_mangle]
pub extern "C" fn test_ifft(input_ptr: i32, len: i32) -> i32 {
    let size = (len * 2 * 4) as i32;
    let output_ptr = alloc(size);

    unsafe {
        ifft(input_ptr as *const f32, output_ptr as *mut f32, len);
    }

    output_ptr
}

/// Test complex multiply: multiply two complex buffers element-wise.
#[no_mangle]
pub extern "C" fn test_complex_multiply(a_ptr: i32, b_ptr: i32, len: i32) -> i32 {
    let size = (len * 2 * 4) as i32;
    let output_ptr = alloc(size);

    unsafe {
        complex_multiply(
            a_ptr as *const f32,
            b_ptr as *const f32,
            output_ptr as *mut f32,
            len,
        );
    }

    output_ptr
}

/// Test compute magnitudes: compute magnitude of each complex sample.
/// Returns pointer to f32 array with len elements.
#[no_mangle]
pub extern "C" fn test_compute_magnitudes(input_ptr: i32, len: i32) -> i32 {
    let size = (len * 4) as i32; // len f32 values
    let output_ptr = alloc(size);

    unsafe {
        compute_magnitudes(input_ptr as *const f32, output_ptr as *mut f32, len);
    }

    output_ptr
}

/// Test find_peak: find index of maximum magnitude in complex buffer.
#[no_mangle]
pub extern "C" fn test_find_peak(input_ptr: i32, len: i32) -> i32 {
    unsafe { find_peak(input_ptr as *const f32, len) }
}

/// Test total_power: compute total power of complex samples.
#[no_mangle]
pub extern "C" fn test_total_power(input_ptr: i32, len: i32) -> f32 {
    unsafe { total_power(input_ptr as *const f32, len) }
}

/// Test hann_window: generate Hann window.
/// Returns pointer to f32 array with len elements.
#[no_mangle]
pub extern "C" fn test_hann_window(len: i32) -> i32 {
    let size = (len * 4) as i32;
    let output_ptr = alloc(size);

    unsafe {
        hann_window(output_ptr as *mut f32, len);
    }

    output_ptr
}

/// FFT-based demodulation example: multiply by reference, FFT, find peak.
/// This demonstrates the hybrid architecture: WASM logic calling native DSP.
/// input_ptr: complex IQ samples (interleaved f32)
/// reference_ptr: reference signal to correlate with (interleaved f32)
/// len: number of complex samples
/// Returns: peak bin index
#[no_mangle]
pub extern "C" fn demodulate_fft(input_ptr: i32, reference_ptr: i32, len: i32) -> i32 {
    // Allocate temporary buffers
    let size = (len * 2 * 4) as i32;
    let mixed_ptr = alloc(size);
    let spectrum_ptr = alloc(size);

    unsafe {
        // Multiply input by reference (e.g., downchirp for LoRa)
        complex_multiply(
            input_ptr as *const f32,
            reference_ptr as *const f32,
            mixed_ptr as *mut f32,
            len,
        );

        // FFT to find correlation peak
        fft(mixed_ptr as *const f32, spectrum_ptr as *mut f32, len);

        // Find peak bin
        let peak = find_peak(spectrum_ptr as *const f32, len);

        // Free temporary buffers
        dealloc(mixed_ptr, size);
        dealloc(spectrum_ptr, size);

        peak
    }
}

// =============================================================================
// Pure WASM Implementations (for benchmarking against host functions)
// =============================================================================

/// Pure WASM complex multiply (for benchmarking).
/// out = a * b element-wise
#[no_mangle]
pub extern "C" fn pure_wasm_complex_multiply(a_ptr: i32, b_ptr: i32, out_ptr: i32, len: i32) {
    let a = a_ptr as *const f32;
    let b = b_ptr as *const f32;
    let out = out_ptr as *mut f32;

    for i in 0..len as usize {
        unsafe {
            let a_re = *a.add(i * 2);
            let a_im = *a.add(i * 2 + 1);
            let b_re = *b.add(i * 2);
            let b_im = *b.add(i * 2 + 1);
            // (a + bi)(c + di) = (ac - bd) + (ad + bc)i
            *out.add(i * 2) = a_re * b_re - a_im * b_im;
            *out.add(i * 2 + 1) = a_re * b_im + a_im * b_re;
        }
    }
}

/// Pure WASM compute magnitudes (for benchmarking).
#[no_mangle]
pub extern "C" fn pure_wasm_compute_magnitudes(in_ptr: i32, out_ptr: i32, len: i32) {
    let input = in_ptr as *const f32;
    let output = out_ptr as *mut f32;

    for i in 0..len as usize {
        unsafe {
            let re = *input.add(i * 2);
            let im = *input.add(i * 2 + 1);
            *output.add(i) = (re * re + im * im).sqrt();
        }
    }
}

/// Pure WASM find peak (for benchmarking).
#[no_mangle]
pub extern "C" fn pure_wasm_find_peak(in_ptr: i32, len: i32) -> i32 {
    let input = in_ptr as *const f32;
    let mut max_mag_sq = 0.0f32;
    let mut max_idx = 0i32;

    for i in 0..len as usize {
        unsafe {
            let re = *input.add(i * 2);
            let im = *input.add(i * 2 + 1);
            let mag_sq = re * re + im * im;
            if mag_sq > max_mag_sq {
                max_mag_sq = mag_sq;
                max_idx = i as i32;
            }
        }
    }

    max_idx
}

/// Pure WASM total power (for benchmarking).
#[no_mangle]
pub extern "C" fn pure_wasm_total_power(in_ptr: i32, len: i32) -> f32 {
    let input = in_ptr as *const f32;
    let mut total = 0.0f32;

    for i in 0..len as usize {
        unsafe {
            let re = *input.add(i * 2);
            let im = *input.add(i * 2 + 1);
            total += re * re + im * im;
        }
    }

    total
}

/// Pure WASM Hann window (for benchmarking).
#[no_mangle]
pub extern "C" fn pure_wasm_hann_window(out_ptr: i32, len: i32) {
    let output = out_ptr as *mut f32;
    let n = len as f32;

    for i in 0..len as usize {
        unsafe {
            let x = 2.0 * PI * (i as f32) / n;
            *output.add(i) = 0.5 * (1.0 - x.cos());
        }
    }
}

/// Pure WASM radix-2 Cooley-Tukey FFT (for benchmarking).
/// Input/output are interleaved f32 (re, im pairs).
/// len must be a power of 2.
#[no_mangle]
pub extern "C" fn pure_wasm_fft(in_ptr: i32, out_ptr: i32, len: i32) {
    let input = in_ptr as *const f32;
    let output = out_ptr as *mut f32;
    let n = len as usize;

    // Copy input to output (we'll work in-place on output)
    for i in 0..n {
        unsafe {
            *output.add(i * 2) = *input.add(i * 2);
            *output.add(i * 2 + 1) = *input.add(i * 2 + 1);
        }
    }

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 0..n {
        if i < j {
            unsafe {
                // Swap complex samples i and j
                let tmp_re = *output.add(i * 2);
                let tmp_im = *output.add(i * 2 + 1);
                *output.add(i * 2) = *output.add(j * 2);
                *output.add(i * 2 + 1) = *output.add(j * 2 + 1);
                *output.add(j * 2) = tmp_re;
                *output.add(j * 2 + 1) = tmp_im;
            }
        }
        let mut m = n >> 1;
        while m > 0 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Cooley-Tukey iterative FFT
    let mut mmax = 1usize;
    while mmax < n {
        let istep = mmax << 1;
        let theta = -PI / (mmax as f32); // -2*PI / istep
        let wpr = (theta.cos(), theta.sin()); // twiddle factor step

        let mut wr = (1.0f32, 0.0f32); // current twiddle factor

        for m in 0..mmax {
            for i in (m..n).step_by(istep) {
                let j = i + mmax;
                unsafe {
                    // Complex multiply: temp = wr * output[j]
                    let o_j_re = *output.add(j * 2);
                    let o_j_im = *output.add(j * 2 + 1);
                    let temp_re = wr.0 * o_j_re - wr.1 * o_j_im;
                    let temp_im = wr.0 * o_j_im + wr.1 * o_j_re;

                    let o_i_re = *output.add(i * 2);
                    let o_i_im = *output.add(i * 2 + 1);

                    // Butterfly
                    *output.add(j * 2) = o_i_re - temp_re;
                    *output.add(j * 2 + 1) = o_i_im - temp_im;
                    *output.add(i * 2) = o_i_re + temp_re;
                    *output.add(i * 2 + 1) = o_i_im + temp_im;
                }
            }
            // Update twiddle factor: wr = wr * wpr (complex multiply)
            let new_wr_re = wr.0 * wpr.0 - wr.1 * wpr.1;
            let new_wr_im = wr.0 * wpr.1 + wr.1 * wpr.0;
            wr = (new_wr_re, new_wr_im);
        }
        mmax = istep;
    }
}

/// Pure WASM demodulation (for benchmarking).
/// Uses pure WASM FFT, complex multiply, and find peak.
#[no_mangle]
pub extern "C" fn pure_wasm_demodulate_fft(input_ptr: i32, reference_ptr: i32, len: i32) -> i32 {
    let size = (len * 2 * 4) as i32;
    let mixed_ptr = alloc(size);
    let spectrum_ptr = alloc(size);

    // Mix with reference
    pure_wasm_complex_multiply(input_ptr, reference_ptr, mixed_ptr, len);

    // FFT
    pure_wasm_fft(mixed_ptr, spectrum_ptr, len);

    // Find peak
    let peak = pure_wasm_find_peak(spectrum_ptr, len);

    // Free buffers
    dealloc(mixed_ptr, size);
    dealloc(spectrum_ptr, size);

    peak
}
