//! Goertzel Algorithm — Efficient Single-Frequency DFT
//!
//! Computes the DFT at a single frequency bin using O(N) operations,
//! much more efficient than a full FFT when only 1-2 frequencies are needed.
//!
//! Applications: DTMF detection, pilot tone detection, selective level
//! measurement, frequency presence/absence detection.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::goertzel::{Goertzel, DtmfDetector};
//! use std::f64::consts::PI;
//!
//! // Detect a 1 kHz tone at 8 kHz sample rate
//! let mut goertzel = Goertzel::new(1000.0, 8000.0, 205);
//!
//! // Generate test signal
//! let signal: Vec<f64> = (0..205)
//!     .map(|i| (2.0 * PI * 1000.0 * i as f64 / 8000.0).sin())
//!     .collect();
//!
//! let magnitude = goertzel.process(&signal);
//! assert!(magnitude > 50.0); // Strong tone present
//! ```

use std::f64::consts::PI;

/// Goertzel algorithm for efficient single-frequency DFT computation.
///
/// The Goertzel computes one DFT bin using a second-order IIR filter:
/// ```text
/// s[n] = x[n] + 2*cos(2πk/N)*s[n-1] - s[n-2]
/// ```
/// After N samples, the DFT value is:
/// ```text
/// X[k] = s[N-1] - W_N^k * s[N-2]
/// ```
/// where `W_N = exp(-j*2π/N)` and `k = round(f_target * N / f_sample)`.
#[derive(Debug, Clone)]
pub struct Goertzel {
    /// Target frequency (Hz)
    target_freq: f64,
    /// Sample rate (Hz)
    sample_rate: f64,
    /// Block size (N)
    block_size: usize,
    /// Precomputed coefficient: 2*cos(2*pi*k/N)
    coeff: f64,
    /// Precomputed normalized frequency k/N
    k_over_n: f64,
}

impl Goertzel {
    /// Create a new Goertzel detector for the given frequency.
    pub fn new(target_freq: f64, sample_rate: f64, block_size: usize) -> Self {
        let k = target_freq * block_size as f64 / sample_rate;
        let k_over_n = k / block_size as f64;
        let coeff = 2.0 * (2.0 * PI * k_over_n).cos();

        Self {
            target_freq,
            sample_rate,
            block_size,
            coeff,
            k_over_n,
        }
    }

    /// Process a block of real-valued samples.
    ///
    /// Returns the magnitude of the DFT at the target frequency.
    pub fn process(&self, input: &[f64]) -> f64 {
        let result = self.process_complex(input);
        result.0
    }

    /// Process a block and return (magnitude, phase) at the target frequency.
    pub fn process_complex(&self, input: &[f64]) -> (f64, f64) {
        let mut s0 = 0.0;
        let mut s1 = 0.0;
        let mut s2;

        let n = input.len().min(self.block_size);
        for i in 0..n {
            s2 = s1;
            s1 = s0;
            s0 = input[i] + self.coeff * s1 - s2;
        }

        // Compute DFT result
        let w_re = (2.0 * PI * self.k_over_n).cos();
        let w_im = -(2.0 * PI * self.k_over_n).sin();

        let real = s0 - s1 * w_re;
        let imag = -s1 * w_im;

        let magnitude = (real * real + imag * imag).sqrt();
        let phase = imag.atan2(real);

        (magnitude, phase)
    }

    /// Process and return power (magnitude squared) — avoids sqrt.
    pub fn power(&self, input: &[f64]) -> f64 {
        let mut s0 = 0.0;
        let mut s1 = 0.0;
        let mut s2;

        let n = input.len().min(self.block_size);
        for i in 0..n {
            s2 = s1;
            s1 = s0;
            s0 = input[i] + self.coeff * s1 - s2;
        }

        // Power = s0^2 + s1^2 - coeff*s0*s1
        s0 * s0 + s1 * s1 - self.coeff * s0 * s1
    }

    /// Get target frequency.
    pub fn target_freq(&self) -> f64 {
        self.target_freq
    }

    /// Get block size.
    pub fn block_size(&self) -> usize {
        self.block_size
    }
}

/// Multi-frequency Goertzel detector.
///
/// Efficiently detects multiple frequencies simultaneously using
/// separate Goertzel filters sharing the same input data.
#[derive(Debug, Clone)]
pub struct MultiGoertzel {
    detectors: Vec<Goertzel>,
}

impl MultiGoertzel {
    /// Create a multi-frequency detector.
    pub fn new(frequencies: &[f64], sample_rate: f64, block_size: usize) -> Self {
        let detectors = frequencies
            .iter()
            .map(|&f| Goertzel::new(f, sample_rate, block_size))
            .collect();
        Self { detectors }
    }

    /// Process input and return magnitudes for all frequencies.
    pub fn process(&self, input: &[f64]) -> Vec<f64> {
        self.detectors.iter().map(|d| d.process(input)).collect()
    }

    /// Process and return (frequency, magnitude) pairs.
    pub fn detect(&self, input: &[f64]) -> Vec<(f64, f64)> {
        self.detectors
            .iter()
            .map(|d| (d.target_freq(), d.process(input)))
            .collect()
    }

    /// Find the strongest detected frequency.
    pub fn strongest(&self, input: &[f64]) -> Option<(f64, f64)> {
        self.detect(input)
            .into_iter()
            .max_by(|a, b| a.1.partial_cmp(&b.1).unwrap())
    }
}

/// DTMF (Dual-Tone Multi-Frequency) detector.
///
/// Detects standard telephone DTMF digits (0-9, *, #, A-D) by
/// identifying the presence of two specific tones from the DTMF
/// frequency matrix.
///
/// | Hz    | 1209 | 1336 | 1477 | 1633 |
/// |-------|------|------|------|------|
/// | 697   | 1    | 2    | 3    | A    |
/// | 770   | 4    | 5    | 6    | B    |
/// | 852   | 7    | 8    | 9    | C    |
/// | 941   | *    | 0    | #    | D    |
#[derive(Debug, Clone)]
pub struct DtmfDetector {
    row_detectors: MultiGoertzel,
    col_detectors: MultiGoertzel,
    /// Detection threshold (relative to average energy)
    threshold: f64,
}

/// Standard DTMF row frequencies.
pub const DTMF_ROW_FREQS: [f64; 4] = [697.0, 770.0, 852.0, 941.0];
/// Standard DTMF column frequencies.
pub const DTMF_COL_FREQS: [f64; 4] = [1209.0, 1336.0, 1477.0, 1633.0];
/// DTMF digit lookup table: [row][col].
pub const DTMF_DIGITS: [[char; 4]; 4] = [
    ['1', '2', '3', 'A'],
    ['4', '5', '6', 'B'],
    ['7', '8', '9', 'C'],
    ['*', '0', '#', 'D'],
];

impl DtmfDetector {
    /// Create a new DTMF detector.
    ///
    /// - `sample_rate`: audio sample rate in Hz (typically 8000)
    /// - `block_size`: number of samples per detection (typically 205 for 8kHz)
    pub fn new(sample_rate: f64, block_size: usize) -> Self {
        Self {
            row_detectors: MultiGoertzel::new(&DTMF_ROW_FREQS, sample_rate, block_size),
            col_detectors: MultiGoertzel::new(&DTMF_COL_FREQS, sample_rate, block_size),
            threshold: 2.0,
        }
    }

    /// Set detection threshold (default: 2.0x average energy).
    pub fn set_threshold(&mut self, threshold: f64) {
        self.threshold = threshold;
    }

    /// Detect DTMF digit in audio block.
    ///
    /// Returns `Some(char)` if a valid digit is detected, `None` otherwise.
    pub fn detect(&self, input: &[f64]) -> Option<char> {
        let row_mags = self.row_detectors.process(input);
        let col_mags = self.col_detectors.process(input);

        // Find strongest row and column
        let (row_idx, row_max) = row_mags
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())?;
        let (col_idx, col_max) = col_mags
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())?;

        // Check that both tones are above threshold
        let avg_energy = (row_mags.iter().sum::<f64>() + col_mags.iter().sum::<f64>()) / 8.0;
        let threshold = avg_energy * self.threshold;

        if *row_max > threshold && *col_max > threshold {
            Some(DTMF_DIGITS[row_idx][col_idx])
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn generate_tone(freq: f64, sample_rate: f64, num_samples: usize) -> Vec<f64> {
        (0..num_samples)
            .map(|i| (2.0 * PI * freq * i as f64 / sample_rate).sin())
            .collect()
    }

    fn generate_dual_tone(
        f1: f64,
        f2: f64,
        sample_rate: f64,
        num_samples: usize,
    ) -> Vec<f64> {
        (0..num_samples)
            .map(|i| {
                let t = i as f64 / sample_rate;
                (2.0 * PI * f1 * t).sin() + (2.0 * PI * f2 * t).sin()
            })
            .collect()
    }

    #[test]
    fn test_goertzel_detects_tone() {
        let sr = 8000.0;
        let n = 205;
        let goertzel = Goertzel::new(1000.0, sr, n);

        let signal = generate_tone(1000.0, sr, n);
        let mag = goertzel.process(&signal);
        assert!(mag > 50.0, "Should detect 1kHz tone: magnitude={mag:.1}");
    }

    #[test]
    fn test_goertzel_rejects_other_freq() {
        let sr = 8000.0;
        let n = 205;
        let goertzel = Goertzel::new(1000.0, sr, n);

        // Tone at 2000 Hz (different frequency)
        let signal = generate_tone(2000.0, sr, n);
        let mag_other = goertzel.process(&signal);

        // Tone at 1000 Hz (target)
        let signal_target = generate_tone(1000.0, sr, n);
        let mag_target = goertzel.process(&signal_target);

        assert!(
            mag_target > mag_other * 5.0,
            "Should strongly prefer target freq: target={mag_target:.1}, other={mag_other:.1}"
        );
    }

    #[test]
    fn test_goertzel_silence() {
        let sr = 8000.0;
        let n = 205;
        let goertzel = Goertzel::new(1000.0, sr, n);

        let silence = vec![0.0; n];
        let mag = goertzel.process(&silence);
        assert!(mag < 0.001, "Silence should have ~zero magnitude: {mag:.6}");
    }

    #[test]
    fn test_goertzel_phase() {
        let sr = 8000.0;
        let n = 205;
        let goertzel = Goertzel::new(1000.0, sr, n);

        // Cosine (zero phase) vs sine (pi/2 phase)
        let cosine: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 1000.0 * i as f64 / sr).cos())
            .collect();
        let (_, phase_cos) = goertzel.process_complex(&cosine);

        let sine: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 1000.0 * i as f64 / sr).sin())
            .collect();
        let (_, phase_sin) = goertzel.process_complex(&sine);

        // Phase difference should be ~pi/2
        let diff = (phase_sin - phase_cos + PI).rem_euclid(2.0 * PI) - PI;
        assert!(
            (diff.abs() - PI / 2.0).abs() < 0.3,
            "Phase difference should be ~90°: got {:.1}°",
            diff.to_degrees()
        );
    }

    #[test]
    fn test_goertzel_power() {
        let sr = 8000.0;
        let n = 205;
        let goertzel = Goertzel::new(1000.0, sr, n);

        let signal = generate_tone(1000.0, sr, n);
        let mag = goertzel.process(&signal);
        let power = goertzel.power(&signal);

        assert!(
            (power - mag * mag).abs() / (mag * mag).max(1e-20) < 0.01,
            "Power should equal magnitude squared"
        );
    }

    #[test]
    fn test_multi_goertzel() {
        let sr = 8000.0;
        let n = 205;
        let freqs = [697.0, 1209.0, 1500.0];
        let detector = MultiGoertzel::new(&freqs, sr, n);

        // Tone at 697 Hz
        let signal = generate_tone(697.0, sr, n);
        let mags = detector.process(&signal);

        // 697 Hz bin should be strongest
        assert!(
            mags[0] > mags[1] && mags[0] > mags[2],
            "697 Hz should be strongest: mags={mags:?}"
        );
    }

    #[test]
    fn test_multi_goertzel_strongest() {
        let sr = 8000.0;
        let n = 205;
        let freqs = [500.0, 1000.0, 2000.0];
        let detector = MultiGoertzel::new(&freqs, sr, n);

        let signal = generate_tone(1000.0, sr, n);
        let (strongest_freq, _) = detector.strongest(&signal).unwrap();
        assert!(
            (strongest_freq - 1000.0).abs() < 1.0,
            "Strongest should be 1000 Hz: got {strongest_freq}"
        );
    }

    #[test]
    fn test_dtmf_digit_5() {
        // Digit '5' = 770 Hz + 1336 Hz
        let sr = 8000.0;
        let n = 205;
        let detector = DtmfDetector::new(sr, n);

        let signal = generate_dual_tone(770.0, 1336.0, sr, n);
        let digit = detector.detect(&signal);
        assert_eq!(digit, Some('5'), "Should detect digit 5");
    }

    #[test]
    fn test_dtmf_digit_star() {
        // '*' = 941 Hz + 1209 Hz
        let sr = 8000.0;
        let n = 205;
        let detector = DtmfDetector::new(sr, n);

        let signal = generate_dual_tone(941.0, 1209.0, sr, n);
        let digit = detector.detect(&signal);
        assert_eq!(digit, Some('*'), "Should detect digit *");
    }

    #[test]
    fn test_dtmf_silence() {
        let sr = 8000.0;
        let n = 205;
        let detector = DtmfDetector::new(sr, n);

        let silence = vec![0.0; n];
        let digit = detector.detect(&silence);
        assert_eq!(digit, None, "Silence should not detect a digit");
    }

    #[test]
    fn test_dtmf_all_digits() {
        let sr = 8000.0;
        let n = 205;
        let detector = DtmfDetector::new(sr, n);

        // Test each digit
        for (row_idx, &row_freq) in DTMF_ROW_FREQS.iter().enumerate() {
            for (col_idx, &col_freq) in DTMF_COL_FREQS.iter().enumerate() {
                let expected = DTMF_DIGITS[row_idx][col_idx];
                let signal = generate_dual_tone(row_freq, col_freq, sr, n);
                let detected = detector.detect(&signal);
                assert_eq!(
                    detected,
                    Some(expected),
                    "Failed to detect digit '{expected}' ({row_freq}Hz + {col_freq}Hz)"
                );
            }
        }
    }
}
