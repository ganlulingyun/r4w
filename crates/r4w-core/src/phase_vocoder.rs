//! Phase Vocoder — Time-Stretch and Pitch-Shift
//!
//! STFT-based time-frequency processor for modifying signal timing
//! without affecting pitch (time-stretch) or pitch without affecting
//! timing (pitch-shift). Uses overlap-add synthesis with phase
//! propagation/locking for natural-sounding output.
//!
//! Key applications: playback speed change, pitch transposition,
//! Doppler compensation, audio forensics, waveform adaptation.
//!
//! No direct GNU Radio equivalent (audio/speech processing tool).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::phase_vocoder::PhaseVocoder;
//!
//! let signal: Vec<f64> = (0..1024)
//!     .map(|i| (i as f64 * 0.1).sin())
//!     .collect();
//! let pv = PhaseVocoder::new(256, 64);
//! let stretched = pv.time_stretch(&signal, 1.5);
//! // Stretched signal is ~1.5x longer
//! assert!(stretched.len() > signal.len());
//! ```

use std::f64::consts::PI;

/// Phase vocoder processor.
#[derive(Debug, Clone)]
pub struct PhaseVocoder {
    /// FFT/window size.
    fft_size: usize,
    /// Hop size (analysis step).
    hop_size: usize,
    /// Window function.
    window: Vec<f64>,
}

impl PhaseVocoder {
    /// Create a new phase vocoder.
    ///
    /// `fft_size`: window/FFT length (should be power of 2).
    /// `hop_size`: analysis hop size (typically fft_size/4).
    pub fn new(fft_size: usize, hop_size: usize) -> Self {
        let fft_size = fft_size.max(4);
        let hop_size = hop_size.max(1).min(fft_size);
        let window = hann_window(fft_size);
        Self {
            fft_size,
            hop_size,
            window,
        }
    }

    /// Time-stretch a signal by the given ratio.
    ///
    /// `ratio` > 1.0 makes the signal longer (slower).
    /// `ratio` < 1.0 makes it shorter (faster).
    /// Pitch is preserved.
    pub fn time_stretch(&self, signal: &[f64], ratio: f64) -> Vec<f64> {
        if signal.len() < self.fft_size || ratio <= 0.0 {
            return signal.to_vec();
        }

        let ratio = ratio.max(0.1).min(10.0);
        let syn_hop = self.hop_size;
        let ana_hop = (syn_hop as f64 / ratio).round() as usize;
        let ana_hop = ana_hop.max(1);

        // Analyze: STFT with analysis hop
        let frames = self.stft(signal, ana_hop);
        if frames.is_empty() {
            return signal.to_vec();
        }

        // Phase propagation
        let half = self.fft_size / 2 + 1;
        let mut phase_accum = vec![0.0; half];
        let mut prev_phase = vec![0.0; half];

        let mut synth_frames: Vec<(Vec<f64>, Vec<f64>)> = Vec::with_capacity(frames.len());

        for (idx, frame) in frames.iter().enumerate() {
            let mut magnitudes = Vec::with_capacity(half);
            let mut phases = Vec::with_capacity(half);

            for k in 0..half {
                let re = frame[k].0;
                let im = frame[k].1;
                magnitudes.push((re * re + im * im).sqrt());
                phases.push(im.atan2(re));
            }

            if idx == 0 {
                phase_accum = phases.clone();
            } else {
                for k in 0..half {
                    // Expected phase advance for this bin
                    let expected = 2.0 * PI * k as f64 * ana_hop as f64 / self.fft_size as f64;
                    // Phase difference
                    let mut dp = phases[k] - prev_phase[k] - expected;
                    // Wrap to [-π, π]
                    dp = dp - (dp / (2.0 * PI)).round() * 2.0 * PI;
                    // True instantaneous frequency deviation
                    let inst_freq = expected + dp;
                    // Accumulate phase for synthesis hop
                    phase_accum[k] += inst_freq * syn_hop as f64 / ana_hop as f64;
                }
            }

            prev_phase = phases;
            synth_frames.push((magnitudes, phase_accum.clone()));
        }

        // Synthesize: inverse STFT with synthesis hop
        self.istft(&synth_frames, syn_hop)
    }

    /// Pitch-shift a signal by the given ratio.
    ///
    /// `ratio` > 1.0 raises pitch. `ratio` < 1.0 lowers pitch.
    /// Duration is preserved.
    pub fn pitch_shift(&self, signal: &[f64], ratio: f64) -> Vec<f64> {
        if ratio <= 0.0 {
            return signal.to_vec();
        }
        // Pitch shift = time stretch by ratio, then resample to original length
        let stretched = self.time_stretch(signal, ratio);
        resample_linear(&stretched, signal.len())
    }

    /// Pitch-shift by semitones.
    pub fn pitch_shift_semitones(&self, signal: &[f64], semitones: f64) -> Vec<f64> {
        let ratio = 2.0_f64.powf(semitones / 12.0);
        self.pitch_shift(signal, ratio)
    }

    /// Compute the Short-Time Fourier Transform.
    ///
    /// Returns frames of (Re, Im) pairs for each frequency bin (half-spectrum).
    pub fn stft(&self, signal: &[f64], hop: usize) -> Vec<Vec<(f64, f64)>> {
        let n = signal.len();
        let half = self.fft_size / 2 + 1;
        let mut frames = Vec::new();

        let mut pos = 0;
        while pos + self.fft_size <= n {
            // Window the frame
            let windowed: Vec<f64> = (0..self.fft_size)
                .map(|i| signal[pos + i] * self.window[i])
                .collect();

            // FFT (real-to-complex, keep first half+1 bins)
            let spectrum = fft_real(&windowed);
            frames.push(spectrum[..half].to_vec());

            pos += hop;
        }

        frames
    }

    /// Inverse STFT: overlap-add synthesis from magnitude + phase frames.
    fn istft(&self, frames: &[(Vec<f64>, Vec<f64>)], hop: usize) -> Vec<f64> {
        if frames.is_empty() {
            return vec![];
        }

        let out_len = (frames.len() - 1) * hop + self.fft_size;
        let mut output = vec![0.0; out_len];
        let mut window_sum = vec![0.0; out_len];
        let half = self.fft_size / 2 + 1;

        for (idx, (magnitudes, phases)) in frames.iter().enumerate() {
            // Reconstruct full complex spectrum
            let mut spectrum = vec![(0.0, 0.0); self.fft_size];
            for k in 0..half {
                let re = magnitudes[k] * phases[k].cos();
                let im = magnitudes[k] * phases[k].sin();
                spectrum[k] = (re, im);
                if k > 0 && k < self.fft_size / 2 {
                    spectrum[self.fft_size - k] = (re, -im); // Conjugate symmetry
                }
            }

            // IFFT
            let frame = ifft_to_real(&spectrum);

            // Overlap-add with window
            let pos = idx * hop;
            for i in 0..self.fft_size {
                if pos + i < out_len {
                    output[pos + i] += frame[i] * self.window[i];
                    window_sum[pos + i] += self.window[i] * self.window[i];
                }
            }
        }

        // Normalize by window overlap
        for i in 0..out_len {
            if window_sum[i] > 1e-10 {
                output[i] /= window_sum[i];
            }
        }

        output
    }

    /// Get the FFT size.
    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    /// Get the hop size.
    pub fn hop_size(&self) -> usize {
        self.hop_size
    }
}

/// Compute the spectrogram (magnitude STFT) of a signal.
///
/// Returns a 2D array: `result[frame][bin]` = magnitude.
pub fn spectrogram(signal: &[f64], fft_size: usize, hop_size: usize) -> Vec<Vec<f64>> {
    let pv = PhaseVocoder::new(fft_size, hop_size);
    let frames = pv.stft(signal, hop_size);
    frames
        .iter()
        .map(|frame| {
            frame
                .iter()
                .map(|(re, im)| (re * re + im * im).sqrt())
                .collect()
        })
        .collect()
}

/// Robotize effect: zero all phases (removes temporal structure).
pub fn robotize(signal: &[f64], fft_size: usize, hop_size: usize) -> Vec<f64> {
    let pv = PhaseVocoder::new(fft_size, hop_size);
    let frames = pv.stft(signal, hop_size);
    let half = fft_size / 2 + 1;

    let synth_frames: Vec<(Vec<f64>, Vec<f64>)> = frames
        .iter()
        .map(|frame| {
            let mags: Vec<f64> = frame.iter().map(|(re, im)| (re * re + im * im).sqrt()).collect();
            let phases = vec![0.0; half];
            (mags, phases)
        })
        .collect();

    pv.istft(&synth_frames, hop_size)
}

/// Whisperize effect: randomize phases (creates whisper-like texture).
pub fn whisperize(signal: &[f64], fft_size: usize, hop_size: usize) -> Vec<f64> {
    let pv = PhaseVocoder::new(fft_size, hop_size);
    let frames = pv.stft(signal, hop_size);
    let half = fft_size / 2 + 1;

    let mut seed = 42u64;
    let synth_frames: Vec<(Vec<f64>, Vec<f64>)> = frames
        .iter()
        .map(|frame| {
            let mags: Vec<f64> = frame.iter().map(|(re, im)| (re * re + im * im).sqrt()).collect();
            let phases: Vec<f64> = (0..half)
                .map(|_| {
                    seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
                    (seed >> 33) as f64 / (1u64 << 31) as f64 * 2.0 * PI - PI
                })
                .collect();
            (mags, phases)
        })
        .collect();

    pv.istft(&synth_frames, hop_size)
}

// ---- Internal helpers ----

fn hann_window(size: usize) -> Vec<f64> {
    (0..size)
        .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / (size - 1).max(1) as f64).cos()))
        .collect()
}

fn resample_linear(signal: &[f64], target_len: usize) -> Vec<f64> {
    if signal.is_empty() || target_len == 0 {
        return vec![];
    }
    let ratio = (signal.len() - 1) as f64 / (target_len - 1).max(1) as f64;
    (0..target_len)
        .map(|i| {
            let pos = i as f64 * ratio;
            let idx = pos.floor() as usize;
            let frac = pos - idx as f64;
            if idx + 1 < signal.len() {
                signal[idx] * (1.0 - frac) + signal[idx + 1] * frac
            } else {
                signal[signal.len() - 1]
            }
        })
        .collect()
}

/// Real-to-complex FFT (returns full spectrum).
fn fft_real(signal: &[f64]) -> Vec<(f64, f64)> {
    let n = signal.len();
    let n_padded = n.next_power_of_two();
    let mut input: Vec<(f64, f64)> = signal.iter().map(|&x| (x, 0.0)).collect();
    input.resize(n_padded, (0.0, 0.0));
    fft_impl(&input)
}

fn ifft_to_real(spectrum: &[(f64, f64)]) -> Vec<f64> {
    let n = spectrum.len();
    let conj: Vec<(f64, f64)> = spectrum.iter().map(|&(re, im)| (re, -im)).collect();
    let result = fft_impl(&conj);
    let scale = 1.0 / n as f64;
    result.iter().map(|&(re, _)| re * scale).collect()
}

fn fft_impl(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    if n <= 1 {
        return input.to_vec();
    }

    if !n.is_power_of_two() {
        // DFT fallback
        return (0..n)
            .map(|k| {
                let mut re = 0.0;
                let mut im = 0.0;
                for (j, &(xr, xi)) in input.iter().enumerate() {
                    let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
                    let (sa, ca) = angle.sin_cos();
                    re += xr * ca - xi * sa;
                    im += xr * sa + xi * ca;
                }
                (re, im)
            })
            .collect();
    }

    let even: Vec<(f64, f64)> = input.iter().step_by(2).copied().collect();
    let odd: Vec<(f64, f64)> = input.iter().skip(1).step_by(2).copied().collect();

    let even_fft = fft_impl(&even);
    let odd_fft = fft_impl(&odd);

    let mut result = vec![(0.0, 0.0); n];
    for k in 0..n / 2 {
        let angle = -2.0 * PI * k as f64 / n as f64;
        let (tw_im, tw_re) = angle.sin_cos();
        let (or, oi) = odd_fft[k];
        let tr = tw_re * or - tw_im * oi;
        let ti = tw_re * oi + tw_im * or;
        result[k] = (even_fft[k].0 + tr, even_fft[k].1 + ti);
        result[k + n / 2] = (even_fft[k].0 - tr, even_fft[k].1 - ti);
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_time_stretch_longer() {
        let signal: Vec<f64> = (0..1024).map(|i| (i as f64 * 0.05).sin()).collect();
        let pv = PhaseVocoder::new(256, 64);
        let stretched = pv.time_stretch(&signal, 2.0);
        // Should be roughly 2x longer
        assert!(
            stretched.len() > (signal.len() as f64 * 1.5) as usize,
            "len={}, expected > {}",
            stretched.len(),
            signal.len() * 3 / 2
        );
    }

    #[test]
    fn test_time_stretch_shorter() {
        let signal: Vec<f64> = (0..2048).map(|i| (i as f64 * 0.05).sin()).collect();
        let pv = PhaseVocoder::new(256, 64);
        let compressed = pv.time_stretch(&signal, 0.5);
        // Should be roughly 0.5x length
        assert!(
            compressed.len() < signal.len(),
            "len={}, expected < {}",
            compressed.len(),
            signal.len()
        );
    }

    #[test]
    fn test_time_stretch_unity() {
        let signal: Vec<f64> = (0..1024).map(|i| (i as f64 * 0.05).sin()).collect();
        let pv = PhaseVocoder::new(256, 64);
        let same = pv.time_stretch(&signal, 1.0);
        // Should be approximately same length
        let ratio = same.len() as f64 / signal.len() as f64;
        assert!(
            (ratio - 1.0).abs() < 0.2,
            "ratio={ratio}"
        );
    }

    #[test]
    fn test_pitch_shift() {
        let signal: Vec<f64> = (0..2048).map(|i| (i as f64 * 0.1).sin()).collect();
        let pv = PhaseVocoder::new(256, 64);
        let shifted = pv.pitch_shift(&signal, 1.5);
        // Output should be same length as input
        assert_eq!(shifted.len(), signal.len());
    }

    #[test]
    fn test_pitch_shift_semitones() {
        let signal: Vec<f64> = (0..2048).map(|i| (i as f64 * 0.1).sin()).collect();
        let pv = PhaseVocoder::new(256, 64);
        let shifted = pv.pitch_shift_semitones(&signal, 7.0); // Perfect fifth
        assert_eq!(shifted.len(), signal.len());
    }

    #[test]
    fn test_stft_roundtrip() {
        let signal: Vec<f64> = (0..512).map(|i| (i as f64 * 0.1).sin()).collect();
        let pv = PhaseVocoder::new(128, 32);
        let frames = pv.stft(&signal, 32);
        assert!(!frames.is_empty());
        // Each frame should have fft_size/2 + 1 bins
        assert_eq!(frames[0].len(), 65);
    }

    #[test]
    fn test_spectrogram() {
        let signal: Vec<f64> = (0..1024).map(|i| (i as f64 * 0.1).sin()).collect();
        let spec = spectrogram(&signal, 256, 64);
        assert!(!spec.is_empty());
        // All magnitudes should be non-negative
        assert!(spec.iter().all(|frame| frame.iter().all(|&m| m >= 0.0)));
    }

    #[test]
    fn test_robotize() {
        let signal: Vec<f64> = (0..1024).map(|i| (i as f64 * 0.1).sin()).collect();
        let robot = robotize(&signal, 256, 64);
        assert!(!robot.is_empty());
    }

    #[test]
    fn test_whisperize() {
        let signal: Vec<f64> = (0..1024).map(|i| (i as f64 * 0.1).sin()).collect();
        let whisper = whisperize(&signal, 256, 64);
        assert!(!whisper.is_empty());
    }

    #[test]
    fn test_short_signal() {
        let signal = vec![1.0, 2.0, 3.0];
        let pv = PhaseVocoder::new(256, 64);
        let result = pv.time_stretch(&signal, 2.0);
        assert_eq!(result, signal); // Too short, returned as-is
    }

    #[test]
    fn test_resample_linear() {
        let signal: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let resampled = resample_linear(&signal, 50);
        assert_eq!(resampled.len(), 50);
        // First and last should match
        assert!((resampled[0] - 0.0).abs() < 0.01);
        assert!((resampled[49] - 99.0).abs() < 0.01);
    }
}
