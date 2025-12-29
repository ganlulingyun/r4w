//! FFT Utilities for LoRa Signal Processing
//!
//! This module provides FFT (Fast Fourier Transform) utilities optimized for
//! LoRa demodulation.
//!
//! ## Understanding FFT in LoRa
//!
//! The FFT is fundamental to LoRa demodulation because:
//!
//! 1. **Symbol Detection**: When we multiply a received chirp by a conjugate
//!    reference downchirp, we get a tone at a frequency proportional to the
//!    transmitted symbol. The FFT finds this peak.
//!
//! 2. **Synchronization**: FFT helps detect preamble patterns and estimate
//!    carrier frequency offset.
//!
//! ```text
//! Received Signal × Conjugate(Reference Downchirp) = Tone at symbol frequency
//!
//!     │ Received     │ Reference      │ Result:
//!     │   Chirp      │  Downchirp     │  Single Tone
//! f   │      /       │  \             │     |
//!     │    /         │    \           │     |
//!     │  /           │      \    =    │     |
//!     │/             │        \       │     |
//!     └──────────    └──────────      └─────┴───── f
//!                                          ^
//!                                     symbol freq
//! ```
//!
//! ## FFT Peak Finding
//!
//! For LoRa with SF=7, there are 128 possible symbols. After the chirp mixing
//! and FFT, the symbol index equals the FFT bin with the maximum magnitude.
//! With oversampling, we use interpolation for finer resolution.

use rustfft::{num_complex::Complex64, Fft, FftPlanner};
use std::fmt;
use std::sync::Arc;

use crate::types::IQSample;

/// FFT processor optimized for LoRa operations
pub struct FftProcessor {
    /// FFT size
    size: usize,
    /// Forward FFT instance
    fft_forward: Arc<dyn Fft<f64>>,
    /// Inverse FFT instance
    fft_inverse: Arc<dyn Fft<f64>>,
    /// Scratch buffer for FFT operations
    scratch: Vec<Complex64>,
}

impl fmt::Debug for FftProcessor {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("FftProcessor")
            .field("size", &self.size)
            .finish()
    }
}

impl FftProcessor {
    /// Create a new FFT processor for the given size
    pub fn new(size: usize) -> Self {
        let mut planner = FftPlanner::new();
        let fft_forward = planner.plan_fft_forward(size);
        let fft_inverse = planner.plan_fft_inverse(size);
        let scratch = vec![Complex64::new(0.0, 0.0); fft_forward.get_inplace_scratch_len()];

        Self {
            size,
            fft_forward,
            fft_inverse,
            scratch,
        }
    }

    /// Get the FFT size
    pub fn size(&self) -> usize {
        self.size
    }

    /// Compute the forward FFT in-place
    pub fn fft_inplace(&mut self, buffer: &mut [Complex64]) {
        assert_eq!(buffer.len(), self.size);
        self.fft_forward.process_with_scratch(buffer, &mut self.scratch);
    }

    /// Compute the forward FFT, returning a new buffer
    pub fn fft(&mut self, input: &[IQSample]) -> Vec<Complex64> {
        let mut buffer: Vec<Complex64> = input.to_vec();
        buffer.resize(self.size, Complex64::new(0.0, 0.0));
        self.fft_inplace(&mut buffer);
        buffer
    }

    /// Compute the inverse FFT in-place
    pub fn ifft_inplace(&mut self, buffer: &mut [Complex64]) {
        assert_eq!(buffer.len(), self.size);
        self.fft_inverse.process_with_scratch(buffer, &mut self.scratch);

        // Normalize by dividing by N
        let scale = 1.0 / self.size as f64;
        for sample in buffer.iter_mut() {
            *sample *= scale;
        }
    }

    /// Compute the inverse FFT, returning a new buffer
    pub fn ifft(&mut self, input: &[Complex64]) -> Vec<Complex64> {
        let mut buffer = input.to_vec();
        buffer.resize(self.size, Complex64::new(0.0, 0.0));
        self.ifft_inplace(&mut buffer);
        buffer
    }

    /// Find the peak in the FFT magnitude spectrum
    ///
    /// Returns (bin_index, magnitude, phase)
    pub fn find_peak(spectrum: &[Complex64]) -> (usize, f64, f64) {
        let mut max_idx = 0;
        let mut max_mag = 0.0;

        for (i, &sample) in spectrum.iter().enumerate() {
            let mag = sample.norm();
            if mag > max_mag {
                max_mag = mag;
                max_idx = i;
            }
        }

        let phase = spectrum[max_idx].arg();
        (max_idx, max_mag, phase)
    }

    /// Find peak with parabolic interpolation for sub-bin resolution
    ///
    /// This uses the method from the SDR-LoRa paper (Eq. 8):
    /// ```text
    /// ν̂ = k̂ + (1/2) * (Γ_{k̂-1} - Γ_{k̂+1}) / (Γ_{k̂-1} - 2*Γ_{k̂} + Γ_{k̂+1})
    /// ```
    ///
    /// Returns (interpolated_index, magnitude)
    pub fn find_peak_interpolated(spectrum: &[Complex64]) -> (f64, f64) {
        let n = spectrum.len();
        let (k, mag, _) = Self::find_peak(spectrum);

        if k == 0 || k == n - 1 {
            return (k as f64, mag);
        }

        // Get magnitudes of neighboring bins
        let gamma_prev = spectrum[(k + n - 1) % n].norm();
        let gamma_curr = spectrum[k].norm();
        let gamma_next = spectrum[(k + 1) % n].norm();

        // Parabolic interpolation
        let denom = gamma_prev - 2.0 * gamma_curr + gamma_next;
        if denom.abs() < 1e-10 {
            return (k as f64, mag);
        }

        let delta = 0.5 * (gamma_prev - gamma_next) / denom;
        let interpolated_idx = k as f64 + delta;

        // Interpolated magnitude
        let interpolated_mag = gamma_curr - 0.25 * (gamma_prev - gamma_next) * delta;

        (interpolated_idx, interpolated_mag)
    }

    /// Compute magnitude spectrum (for visualization)
    pub fn magnitude_spectrum(spectrum: &[Complex64]) -> Vec<f64> {
        spectrum.iter().map(|c| c.norm()).collect()
    }

    /// Compute power spectrum in dB (for visualization)
    pub fn power_spectrum_db(spectrum: &[Complex64]) -> Vec<f64> {
        spectrum
            .iter()
            .map(|c| {
                let power = c.norm_sqr();
                if power > 1e-20 {
                    10.0 * power.log10()
                } else {
                    -200.0 // Floor value
                }
            })
            .collect()
    }

    /// FFT shift - move zero frequency to center (for visualization)
    pub fn fft_shift<T: Clone>(spectrum: &[T]) -> Vec<T> {
        let n = spectrum.len();
        let mid = n / 2;
        let mut shifted = Vec::with_capacity(n);
        shifted.extend_from_slice(&spectrum[mid..]);
        shifted.extend_from_slice(&spectrum[..mid]);
        shifted
    }
}

/// Compute cross-correlation of two signals using FFT
///
/// Cross-correlation is useful for:
/// - Finding signal timing offset
/// - Matched filtering
/// - Pattern detection
pub fn cross_correlate(signal: &[IQSample], reference: &[IQSample]) -> Vec<Complex64> {
    let n = signal.len().max(reference.len()).next_power_of_two() * 2;
    let mut processor = FftProcessor::new(n);

    // Zero-pad both signals
    let mut sig_padded: Vec<Complex64> = signal.to_vec();
    sig_padded.resize(n, Complex64::new(0.0, 0.0));

    let mut ref_padded: Vec<Complex64> = reference.to_vec();
    ref_padded.resize(n, Complex64::new(0.0, 0.0));

    // FFT both signals
    processor.fft_inplace(&mut sig_padded);
    processor.fft_inplace(&mut ref_padded);

    // Multiply signal FFT by conjugate of reference FFT
    for i in 0..n {
        sig_padded[i] *= ref_padded[i].conj();
    }

    // Inverse FFT to get correlation
    processor.ifft_inplace(&mut sig_padded);

    sig_padded
}

/// Spectrogram computation for visualization
///
/// Computes a time-frequency representation of the signal using
/// short-time Fourier transform (STFT).
#[derive(Debug, Clone)]
pub struct Spectrogram {
    /// Time bins (center time of each frame)
    pub times: Vec<f64>,
    /// Frequency bins (Hz)
    pub frequencies: Vec<f64>,
    /// Power values in dB [time][frequency]
    pub power_db: Vec<Vec<f64>>,
    /// FFT size used
    pub fft_size: usize,
    /// Hop size in samples
    pub hop_size: usize,
    /// Sample rate
    pub sample_rate: f64,
}

impl Spectrogram {
    /// Compute spectrogram of a signal
    ///
    /// # Arguments
    /// * `signal` - Input I/Q samples
    /// * `fft_size` - FFT size for each frame
    /// * `hop_size` - Number of samples to advance between frames
    /// * `sample_rate` - Sample rate in Hz
    pub fn compute(
        signal: &[IQSample],
        fft_size: usize,
        hop_size: usize,
        sample_rate: f64,
    ) -> Self {
        let mut processor = FftProcessor::new(fft_size);
        let mut power_db = Vec::new();
        let mut times = Vec::new();

        // Generate Hann window
        let window: Vec<f64> = (0..fft_size)
            .map(|i| {
                0.5 * (1.0 - (2.0 * std::f64::consts::PI * i as f64 / fft_size as f64).cos())
            })
            .collect();

        let mut pos = 0;
        while pos + fft_size <= signal.len() {
            // Extract and window the frame
            let mut frame: Vec<Complex64> = signal[pos..pos + fft_size]
                .iter()
                .enumerate()
                .map(|(i, &s)| s * window[i])
                .collect();

            // Compute FFT
            processor.fft_inplace(&mut frame);

            // Compute power spectrum in dB
            let power = FftProcessor::power_spectrum_db(&frame);

            // FFT shift for display
            power_db.push(FftProcessor::fft_shift(&power));

            times.push((pos + fft_size / 2) as f64 / sample_rate);
            pos += hop_size;
        }

        // Compute frequency axis
        let freq_resolution = sample_rate / fft_size as f64;
        let frequencies: Vec<f64> = (0..fft_size)
            .map(|i| {
                let idx = if i < fft_size / 2 {
                    i as f64
                } else {
                    (i as i64 - fft_size as i64) as f64
                };
                idx * freq_resolution
            })
            .collect();
        let frequencies = FftProcessor::fft_shift(&frequencies);

        Self {
            times,
            frequencies,
            power_db,
            fft_size,
            hop_size,
            sample_rate,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    #[test]
    fn test_fft_single_tone() {
        let n = 128;
        let sample_rate = 128.0;
        let freq = 10.0; // 10 Hz tone

        // Generate a single tone
        let signal: Vec<Complex64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * freq * t;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let mut processor = FftProcessor::new(n);
        let spectrum = processor.fft(&signal);

        let (peak_bin, _, _) = FftProcessor::find_peak(&spectrum);

        // Peak should be at bin 10 (10 Hz with 1 Hz resolution)
        assert_eq!(peak_bin, 10);
    }

    #[test]
    fn test_fft_inverse_identity() {
        let n = 64;
        let signal: Vec<Complex64> = (0..n)
            .map(|i| Complex64::new(i as f64, (i * 2) as f64))
            .collect();

        let mut processor = FftProcessor::new(n);

        let mut buffer = signal.clone();
        processor.fft_inplace(&mut buffer);
        processor.ifft_inplace(&mut buffer);

        // Should recover original signal
        for (orig, recovered) in signal.iter().zip(buffer.iter()) {
            assert!((orig - recovered).norm() < 1e-10);
        }
    }

    #[test]
    fn test_peak_interpolation() {
        let n = 64;
        let sample_rate = 64.0;
        let freq = 10.3; // Non-integer frequency

        let signal: Vec<Complex64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                let phase = 2.0 * PI * freq * t;
                Complex64::new(phase.cos(), phase.sin())
            })
            .collect();

        let mut processor = FftProcessor::new(n);
        let spectrum = processor.fft(&signal);

        let (interpolated_idx, _) = FftProcessor::find_peak_interpolated(&spectrum);

        // Interpolated index should be close to the integer bin (10)
        // Parabolic interpolation on linear magnitude has limited accuracy
        // with spectral leakage. We verify it improves over the integer bin.
        assert!(interpolated_idx > 10.0 && interpolated_idx < 11.0,
                "Interpolated index {} should be between bins 10 and 11", interpolated_idx);
        // Should be closer to true frequency than integer bin
        assert!((interpolated_idx - 10.3).abs() < (10.0_f64 - 10.3).abs(),
                "Interpolation should improve accuracy");
    }
}
