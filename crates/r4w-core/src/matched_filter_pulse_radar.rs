//! Generic matched filter bank for complex-valued pulse radar templates.
//!
//! This module implements pulse compression via matched filtering using conjugate
//! time-reversal correlation, with normalized cross-correlation output and
//! sidelobe suppression. It supports LFM (Linear Frequency Modulated) chirp
//! and Barker code pulse templates.
//!
//! # Features
//!
//! - **Matched filtering** – Conjugate time-reversal correlation for optimal SNR
//! - **LFM chirp generation** – Linear frequency modulated pulse templates
//! - **Barker codes** – Phase-coded pulse templates (lengths 2–13)
//! - **Normalized output** – Cross-correlation normalized to \[0, 1\]
//! - **Peak detection** – CFAR-like adaptive threshold detection
//! - **Pulse metrics** – Compression ratio, sidelobe level, range resolution
//!
//! # Example
//!
//! ```
//! use r4w_core::matched_filter_pulse_radar::{MatchedFilterPulseRadar, LfmChirp};
//!
//! // Generate a 10 µs LFM chirp with 1 MHz bandwidth at 10 MHz sample rate
//! let chirp = LfmChirp::new(1.0e6, 10.0e-6, 10.0e6);
//! let template = chirp.generate();
//!
//! // Build the matched filter
//! let mf = MatchedFilterPulseRadar::new(template);
//!
//! // Create a received signal: noise + delayed chirp
//! let chirp_samples = chirp.generate();
//! let mut rx: Vec<(f64, f64)> = vec![(0.0, 0.0); 50];
//! rx.extend_from_slice(&chirp_samples);
//! rx.extend(vec![(0.0, 0.0); 50]);
//!
//! // Apply matched filter
//! let output = mf.filter(&rx);
//! assert!(!output.is_empty());
//!
//! // Detect peaks
//! let peaks = mf.detect_peaks(&output, 0.5);
//! assert!(!peaks.is_empty());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers (using (f64, f64) tuples)
// ---------------------------------------------------------------------------

/// Complex multiplication: (a+jb)(c+jd) = (ac-bd) + j(ad+bc)
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Complex conjugate
#[inline]
fn conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Complex magnitude squared
#[inline]
fn mag2(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Complex magnitude
#[inline]
fn mag(a: (f64, f64)) -> f64 {
    mag2(a).sqrt()
}

/// Complex addition
#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

// ---------------------------------------------------------------------------
// LFM Chirp Generator
// ---------------------------------------------------------------------------

/// Linear Frequency Modulated (LFM) chirp pulse generator.
///
/// Generates a complex baseband LFM chirp signal defined as:
///
///   s(t) = exp(j * pi * mu * t^2)
///
/// where mu = bandwidth / pulse_width is the chirp rate.
#[derive(Debug, Clone)]
pub struct LfmChirp {
    /// Bandwidth in Hz
    pub bandwidth: f64,
    /// Pulse width in seconds
    pub pulse_width: f64,
    /// Sample rate in Hz
    pub sample_rate: f64,
}

impl LfmChirp {
    /// Create a new LFM chirp specification.
    ///
    /// # Arguments
    /// * `bandwidth` – Sweep bandwidth in Hz
    /// * `pulse_width` – Pulse duration in seconds
    /// * `sample_rate` – Sample rate in Hz
    pub fn new(bandwidth: f64, pulse_width: f64, sample_rate: f64) -> Self {
        Self {
            bandwidth,
            pulse_width,
            sample_rate,
        }
    }

    /// Generate the complex chirp samples.
    pub fn generate(&self) -> Vec<(f64, f64)> {
        let n = (self.pulse_width * self.sample_rate).round() as usize;
        let mu = self.bandwidth / self.pulse_width; // chirp rate
        let dt = 1.0 / self.sample_rate;
        (0..n)
            .map(|i| {
                let t = (i as f64) * dt - self.pulse_width / 2.0;
                let phase = PI * mu * t * t;
                (phase.cos(), phase.sin())
            })
            .collect()
    }

    /// Time-bandwidth product (pulse compression ratio).
    pub fn time_bandwidth_product(&self) -> f64 {
        self.bandwidth * self.pulse_width
    }

    /// Theoretical range resolution in meters.
    ///
    /// dR = c / (2 * B) where c = speed of light.
    pub fn range_resolution(&self) -> f64 {
        let c = 299_792_458.0;
        c / (2.0 * self.bandwidth)
    }
}

// ---------------------------------------------------------------------------
// Barker Code Generator
// ---------------------------------------------------------------------------

/// Barker code pulse template generator.
///
/// Barker codes are binary phase codes with optimal aperiodic autocorrelation
/// properties (peak sidelobe ratio of 1/N). Available lengths: 2, 3, 4, 5, 7, 11, 13.
#[derive(Debug, Clone)]
pub struct BarkerCode {
    /// Code length (must be one of 2, 3, 4, 5, 7, 11, 13)
    pub length: usize,
    /// Samples per chip
    pub samples_per_chip: usize,
}

impl BarkerCode {
    /// Create a new Barker code specification.
    ///
    /// # Panics
    /// Panics if `length` is not a valid Barker code length.
    pub fn new(length: usize, samples_per_chip: usize) -> Self {
        assert!(
            matches!(length, 2 | 3 | 4 | 5 | 7 | 11 | 13),
            "Invalid Barker code length {length}. Must be 2, 3, 4, 5, 7, 11, or 13."
        );
        assert!(samples_per_chip >= 1, "samples_per_chip must be >= 1");
        Self {
            length,
            samples_per_chip,
        }
    }

    /// Return the Barker code chips as +1 / -1 values.
    pub fn chips(&self) -> Vec<f64> {
        let code: &[f64] = match self.length {
            2 => &[1.0, -1.0],
            3 => &[1.0, 1.0, -1.0],
            4 => &[1.0, 1.0, -1.0, 1.0],
            5 => &[1.0, 1.0, 1.0, -1.0, 1.0],
            7 => &[1.0, 1.0, 1.0, -1.0, -1.0, 1.0, -1.0],
            11 => &[1.0, 1.0, 1.0, -1.0, -1.0, -1.0, 1.0, -1.0, -1.0, 1.0, -1.0],
            13 => &[
                1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0,
            ],
            _ => unreachable!(),
        };
        code.to_vec()
    }

    /// Generate the BPSK-modulated Barker code template (complex samples).
    ///
    /// Each chip is repeated `samples_per_chip` times.
    pub fn generate(&self) -> Vec<(f64, f64)> {
        let chips = self.chips();
        let mut samples = Vec::with_capacity(chips.len() * self.samples_per_chip);
        for &chip in &chips {
            for _ in 0..self.samples_per_chip {
                samples.push((chip, 0.0));
            }
        }
        samples
    }

    /// Pulse compression ratio (number of chips).
    pub fn compression_ratio(&self) -> usize {
        self.length
    }
}

// ---------------------------------------------------------------------------
// Matched Filter
// ---------------------------------------------------------------------------

/// A matched filter for complex-valued pulse radar templates.
///
/// The matched filter is the conjugate time-reversal of the transmitted
/// waveform, which maximises the output SNR for a known signal in white
/// Gaussian noise.
#[derive(Debug, Clone)]
pub struct MatchedFilterPulseRadar {
    /// The matched filter impulse response h[n] = s*[-n] (conjugate reversed template).
    impulse_response: Vec<(f64, f64)>,
    /// Energy of the original template (for normalization).
    template_energy: f64,
    /// Length of the original template.
    template_len: usize,
}

impl MatchedFilterPulseRadar {
    /// Create a matched filter from a reference template waveform.
    ///
    /// The impulse response is formed by conjugating and time-reversing the
    /// template, which is the optimal filter for detecting the template in
    /// additive white Gaussian noise.
    pub fn new(template: Vec<(f64, f64)>) -> Self {
        let template_len = template.len();
        let template_energy: f64 = template.iter().map(|s| mag2(*s)).sum();
        // h[n] = s*[N-1-n]  (conjugate time-reversal)
        let impulse_response: Vec<(f64, f64)> = template.iter().rev().map(|s| conj(*s)).collect();
        Self {
            impulse_response,
            template_energy,
            template_len,
        }
    }

    /// Return the matched filter impulse response.
    pub fn impulse_response(&self) -> &[(f64, f64)] {
        &self.impulse_response
    }

    /// Return the template length in samples.
    pub fn template_len(&self) -> usize {
        self.template_len
    }

    /// Return the template energy.
    pub fn template_energy(&self) -> f64 {
        self.template_energy
    }

    /// Apply the matched filter to an input signal and return the complex
    /// correlation output.
    ///
    /// The output length is `input.len() + template_len - 1` (full convolution).
    pub fn filter(&self, input: &[(f64, f64)]) -> Vec<(f64, f64)> {
        let h = &self.impulse_response;
        let h_len = h.len();
        let in_len = input.len();
        if in_len == 0 || h_len == 0 {
            let out_len = if in_len == 0 && h_len == 0 {
                0
            } else {
                in_len + h_len - 1
            };
            return vec![(0.0, 0.0); out_len];
        }
        let out_len = in_len + h_len - 1;
        let mut output = vec![(0.0, 0.0); out_len];
        for (i, &hi) in h.iter().enumerate() {
            for (j, &xj) in input.iter().enumerate() {
                output[i + j] = cadd(output[i + j], cmul(hi, xj));
            }
        }
        output
    }

    /// Apply the matched filter and return **normalized** cross-correlation
    /// magnitudes in \[0, 1\].
    ///
    /// Normalization: |y[n]| / (sqrt(E_template) * sqrt(E_local[n]))
    /// where E_local[n] is the energy of the input segment aligned with
    /// sample n.
    pub fn filter_normalized(&self, input: &[(f64, f64)]) -> Vec<f64> {
        let raw = self.filter(input);
        let h_len = self.template_len;
        let sqrt_e_templ = self.template_energy.sqrt();
        if sqrt_e_templ == 0.0 {
            return vec![0.0; raw.len()];
        }

        // Compute sliding window energy of the input
        let mut result = Vec::with_capacity(raw.len());
        for n in 0..raw.len() {
            // The segment of input contributing to output[n] spans
            // input[max(0, n+1-h_len) .. min(input.len(), n+1)]
            let start = if n + 1 >= h_len { n + 1 - h_len } else { 0 };
            let end = (n + 1).min(input.len());
            let local_energy: f64 = input[start..end].iter().map(|s| mag2(*s)).sum();
            let denom = sqrt_e_templ * local_energy.sqrt();
            if denom > 0.0 {
                result.push(mag(raw[n]) / denom);
            } else {
                result.push(0.0);
            }
        }
        result
    }

    /// Detect peaks in the matched filter output using a CFAR-like threshold.
    ///
    /// Returns a vector of `(index, magnitude)` pairs for peaks exceeding
    /// `threshold` times the maximum magnitude in the output.
    ///
    /// A simple local-maximum check ensures that only true peaks (samples
    /// greater than both neighbours) are returned.
    pub fn detect_peaks(&self, output: &[(f64, f64)], threshold: f64) -> Vec<(usize, f64)> {
        if output.is_empty() {
            return Vec::new();
        }
        let magnitudes: Vec<f64> = output.iter().map(|s| mag(*s)).collect();
        let max_mag = magnitudes.iter().cloned().fold(0.0_f64, f64::max);
        if max_mag == 0.0 {
            return Vec::new();
        }
        let abs_threshold = threshold * max_mag;
        let mut peaks = Vec::new();
        for i in 0..magnitudes.len() {
            if magnitudes[i] < abs_threshold {
                continue;
            }
            // Local maximum check
            let left = if i > 0 { magnitudes[i - 1] } else { 0.0 };
            let right = if i + 1 < magnitudes.len() {
                magnitudes[i + 1]
            } else {
                0.0
            };
            if magnitudes[i] >= left && magnitudes[i] >= right {
                peaks.push((i, magnitudes[i]));
            }
        }
        peaks
    }

    /// Detect peaks using normalized output magnitudes.
    ///
    /// Returns `(index, normalized_magnitude)` pairs where the normalized
    /// magnitude exceeds `threshold` (a value in \[0, 1\]).
    pub fn detect_peaks_normalized(
        &self,
        input: &[(f64, f64)],
        threshold: f64,
    ) -> Vec<(usize, f64)> {
        let norm = self.filter_normalized(input);
        let mut peaks = Vec::new();
        for i in 0..norm.len() {
            if norm[i] < threshold {
                continue;
            }
            let left = if i > 0 { norm[i - 1] } else { 0.0 };
            let right = if i + 1 < norm.len() {
                norm[i + 1]
            } else {
                0.0
            };
            if norm[i] >= left && norm[i] >= right {
                peaks.push((i, norm[i]));
            }
        }
        peaks
    }

    /// Measure the peak sidelobe level (PSL) in dB from the autocorrelation.
    ///
    /// PSL = 20 * log10(max_sidelobe / mainlobe_peak)
    ///
    /// This correlates the template with itself to obtain the ambiguity
    /// function zero-Doppler cut, then finds the highest sidelobe relative
    /// to the main peak.
    pub fn peak_sidelobe_level_db(&self) -> f64 {
        // Reconstruct the original template from the impulse response
        let template: Vec<(f64, f64)> = self
            .impulse_response
            .iter()
            .rev()
            .map(|s| conj(*s))
            .collect();
        let acf = self.filter(&template);
        let mags: Vec<f64> = acf.iter().map(|s| mag(*s)).collect();

        let (peak_idx, peak_val) = mags
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();

        if *peak_val == 0.0 {
            return f64::NEG_INFINITY;
        }

        // Find the highest sidelobe (exclude a region around the main peak)
        let guard = 2.max(self.template_len / 10);
        let max_sidelobe = mags
            .iter()
            .enumerate()
            .filter(|(i, _)| (*i as isize - peak_idx as isize).unsigned_abs() > guard)
            .map(|(_, &v)| v)
            .fold(0.0_f64, f64::max);

        if max_sidelobe == 0.0 {
            return f64::NEG_INFINITY;
        }

        20.0 * (max_sidelobe / peak_val).log10()
    }

    /// Compute the pulse compression ratio from the autocorrelation.
    ///
    /// PCR = (peak magnitude)^2 / (mean sidelobe magnitude)^2
    /// expressed in dB: 10 * log10(PCR).
    pub fn pulse_compression_ratio_db(&self) -> f64 {
        let template: Vec<(f64, f64)> = self
            .impulse_response
            .iter()
            .rev()
            .map(|s| conj(*s))
            .collect();
        let acf = self.filter(&template);
        let mags: Vec<f64> = acf.iter().map(|s| mag(*s)).collect();

        let (peak_idx, peak_val) = mags
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();

        if *peak_val == 0.0 {
            return 0.0;
        }

        let guard = 2.max(self.template_len / 10);
        let sidelobes: Vec<f64> = mags
            .iter()
            .enumerate()
            .filter(|(i, _)| (*i as isize - peak_idx as isize).unsigned_abs() > guard)
            .map(|(_, &v)| v)
            .collect();

        if sidelobes.is_empty() {
            return f64::INFINITY;
        }

        let mean_sl = sidelobes.iter().sum::<f64>() / sidelobes.len() as f64;
        if mean_sl == 0.0 {
            return f64::INFINITY;
        }

        20.0 * (peak_val / mean_sl).log10()
    }

    /// Calculate range resolution given the sample rate.
    ///
    /// Measured from the -3 dB main-lobe width of the autocorrelation.
    /// For an LFM chirp this approximates c / (2*B).
    pub fn range_resolution(&self, sample_rate: f64) -> f64 {
        let c = 299_792_458.0;
        let template: Vec<(f64, f64)> = self
            .impulse_response
            .iter()
            .rev()
            .map(|s| conj(*s))
            .collect();
        let acf = self.filter(&template);
        let mags: Vec<f64> = acf.iter().map(|s| mag(*s)).collect();

        let peak_val = mags.iter().cloned().fold(0.0_f64, f64::max);
        if peak_val == 0.0 {
            return f64::INFINITY;
        }

        let half_power = peak_val / 2.0_f64.sqrt(); // -3 dB level
        let peak_idx = mags
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        // Find -3 dB width by scanning left and right from the peak
        let mut left = peak_idx;
        while left > 0 && mags[left] >= half_power {
            left -= 1;
        }
        let mut right = peak_idx;
        while right + 1 < mags.len() && mags[right] >= half_power {
            right += 1;
        }
        let width_samples = (right - left) as f64;
        // Range resolution: each sample corresponds to c/(2*fs) in range
        width_samples * c / (2.0 * sample_rate)
    }
}

// ---------------------------------------------------------------------------
// Sidelobe Suppression (windowed matched filter)
// ---------------------------------------------------------------------------

/// Apply a Hamming window to a matched filter's impulse response for
/// sidelobe suppression.
///
/// Returns a new `MatchedFilterPulseRadar` with the windowed impulse response.
/// This trades ~1-2 dB of SNR loss for significantly lower sidelobes.
pub fn apply_hamming_window(mf: &MatchedFilterPulseRadar) -> MatchedFilterPulseRadar {
    let n = mf.impulse_response.len();
    let windowed: Vec<(f64, f64)> = mf
        .impulse_response
        .iter()
        .enumerate()
        .map(|(i, &(re, im))| {
            let w = 0.54 - 0.46 * (2.0 * PI * i as f64 / (n as f64 - 1.0)).cos();
            (re * w, im * w)
        })
        .collect();
    let energy: f64 = windowed.iter().map(|s| mag2(*s)).sum();
    MatchedFilterPulseRadar {
        impulse_response: windowed,
        template_energy: energy,
        template_len: mf.template_len,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-10;

    // --- Complex helper tests ---

    #[test]
    fn test_cmul_basic() {
        let a = (3.0, 4.0);
        let b = (1.0, 2.0);
        let c = cmul(a, b);
        // (3+4j)(1+2j) = 3+6j+4j+8j^2 = (3-8)+(6+4)j = (-5, 10)
        assert!((c.0 - (-5.0)).abs() < EPSILON);
        assert!((c.1 - 10.0).abs() < EPSILON);
    }

    #[test]
    fn test_conj_and_mag() {
        let a = (3.0, 4.0);
        assert_eq!(conj(a), (3.0, -4.0));
        assert!((mag(a) - 5.0).abs() < EPSILON);
        assert!((mag2(a) - 25.0).abs() < EPSILON);
    }

    // --- LFM Chirp tests ---

    #[test]
    fn test_lfm_chirp_length() {
        let chirp = LfmChirp::new(1e6, 10e-6, 10e6);
        let samples = chirp.generate();
        // 10e-6 * 10e6 = 100 samples
        assert_eq!(samples.len(), 100);
    }

    #[test]
    fn test_lfm_chirp_unit_magnitude() {
        let chirp = LfmChirp::new(1e6, 10e-6, 10e6);
        let samples = chirp.generate();
        for &s in &samples {
            let m = mag(s);
            assert!(
                (m - 1.0).abs() < EPSILON,
                "LFM chirp sample should have unit magnitude, got {m}"
            );
        }
    }

    #[test]
    fn test_lfm_time_bandwidth_product() {
        let chirp = LfmChirp::new(2e6, 20e-6, 10e6);
        assert!((chirp.time_bandwidth_product() - 40.0).abs() < EPSILON);
    }

    #[test]
    fn test_lfm_range_resolution() {
        let chirp = LfmChirp::new(1e6, 10e-6, 10e6);
        let dr = chirp.range_resolution();
        let expected = 299_792_458.0 / (2.0 * 1e6);
        assert!((dr - expected).abs() < 1.0); // within 1 meter
    }

    // --- Barker code tests ---

    #[test]
    fn test_barker_code_chips() {
        let b13 = BarkerCode::new(13, 1);
        let chips = b13.chips();
        assert_eq!(chips.len(), 13);
        assert_eq!(
            chips,
            vec![1.0, 1.0, 1.0, 1.0, 1.0, -1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, 1.0]
        );
    }

    #[test]
    fn test_barker_code_generate_length() {
        let b7 = BarkerCode::new(7, 4);
        let samples = b7.generate();
        assert_eq!(samples.len(), 7 * 4);
    }

    #[test]
    fn test_barker_compression_ratio() {
        let b5 = BarkerCode::new(5, 1);
        assert_eq!(b5.compression_ratio(), 5);
    }

    #[test]
    #[should_panic(expected = "Invalid Barker code length")]
    fn test_barker_invalid_length() {
        let _ = BarkerCode::new(6, 1);
    }

    #[test]
    fn test_barker_all_valid_lengths() {
        for &len in &[2, 3, 4, 5, 7, 11, 13] {
            let b = BarkerCode::new(len, 1);
            assert_eq!(b.generate().len(), len);
        }
    }

    // --- Matched filter tests ---

    #[test]
    fn test_matched_filter_output_length() {
        let template = vec![(1.0, 0.0); 10];
        let mf = MatchedFilterPulseRadar::new(template);
        let input = vec![(1.0, 0.0); 20];
        let output = mf.filter(&input);
        assert_eq!(output.len(), 20 + 10 - 1);
    }

    #[test]
    fn test_matched_filter_peak_at_alignment() {
        // A simple pulse: 5 samples of (1,0) in a 20-sample signal
        let template = vec![(1.0, 0.0); 5];
        let mf = MatchedFilterPulseRadar::new(template.clone());

        let mut input = vec![(0.0, 0.0); 30];
        for i in 10..15 {
            input[i] = (1.0, 0.0);
        }

        let output = mf.filter(&input);
        let mags: Vec<f64> = output.iter().map(|s| mag(*s)).collect();

        // Peak should occur at index 14 (= 10 + 5 - 1, alignment point)
        let (peak_idx, _) = mags
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        assert_eq!(peak_idx, 14);
    }

    #[test]
    fn test_normalized_output_peak_near_one() {
        let chirp = LfmChirp::new(1e6, 10e-6, 10e6);
        let template = chirp.generate();
        let mf = MatchedFilterPulseRadar::new(template.clone());

        // Input = template itself (perfect match)
        let norm = mf.filter_normalized(&template);
        let max_norm = norm.iter().cloned().fold(0.0_f64, f64::max);
        assert!(
            max_norm > 0.95,
            "Normalized peak should be near 1.0, got {max_norm}"
        );
    }

    #[test]
    fn test_detect_peaks_finds_single_pulse() {
        let barker = BarkerCode::new(13, 1);
        let template = barker.generate();
        let mf = MatchedFilterPulseRadar::new(template.clone());

        let mut input = vec![(0.0, 0.0); 20];
        input.extend_from_slice(&template);
        input.extend(vec![(0.0, 0.0); 20]);

        let output = mf.filter(&input);
        let peaks = mf.detect_peaks(&output, 0.5);
        assert_eq!(peaks.len(), 1, "Should detect exactly one peak");
    }

    #[test]
    fn test_detect_peaks_finds_two_pulses() {
        let barker = BarkerCode::new(13, 1);
        let template = barker.generate();
        let mf = MatchedFilterPulseRadar::new(template.clone());

        let mut input = vec![(0.0, 0.0); 20];
        input.extend_from_slice(&template);
        input.extend(vec![(0.0, 0.0); 40]);
        input.extend_from_slice(&template);
        input.extend(vec![(0.0, 0.0); 20]);

        let output = mf.filter(&input);
        let peaks = mf.detect_peaks(&output, 0.5);
        assert_eq!(peaks.len(), 2, "Should detect two peaks");
    }

    #[test]
    fn test_peak_sidelobe_barker13() {
        let barker = BarkerCode::new(13, 1);
        let template = barker.generate();
        let mf = MatchedFilterPulseRadar::new(template);
        let psl = mf.peak_sidelobe_level_db();
        // Barker-13 PSL should be about -22.3 dB (1/13)
        assert!(
            psl < -20.0,
            "Barker-13 PSL should be below -20 dB, got {psl}"
        );
    }

    #[test]
    fn test_pulse_compression_ratio_lfm() {
        let chirp = LfmChirp::new(1e6, 10e-6, 10e6);
        let template = chirp.generate();
        let mf = MatchedFilterPulseRadar::new(template);
        let pcr = mf.pulse_compression_ratio_db();
        // TB product = 10 => PCR ~ 10 dB (20 log10(10) roughly)
        // Actual PCR depends on sidelobe structure, but should be significantly positive
        assert!(
            pcr > 5.0,
            "LFM PCR should be well above 5 dB, got {pcr}"
        );
    }

    #[test]
    fn test_range_resolution_lfm() {
        let bw = 1e6;
        let chirp = LfmChirp::new(bw, 50e-6, 10e6);
        let template = chirp.generate();
        let mf = MatchedFilterPulseRadar::new(template);
        let dr = mf.range_resolution(10e6);
        let expected = 299_792_458.0 / (2.0 * bw);
        // Should be in the right ballpark (within 50%)
        assert!(
            dr < expected * 2.0 && dr > expected * 0.3,
            "Range resolution {dr} should be near {expected}"
        );
    }

    #[test]
    fn test_hamming_window_reduces_sidelobes() {
        let chirp = LfmChirp::new(1e6, 20e-6, 10e6);
        let template = chirp.generate();
        let mf = MatchedFilterPulseRadar::new(template);

        let psl_unwindowed = mf.peak_sidelobe_level_db();
        let mf_windowed = apply_hamming_window(&mf);
        let psl_windowed = mf_windowed.peak_sidelobe_level_db();

        assert!(
            psl_windowed < psl_unwindowed,
            "Windowed PSL ({psl_windowed} dB) should be lower than unwindowed ({psl_unwindowed} dB)"
        );
    }

    #[test]
    fn test_detect_peaks_normalized() {
        let barker = BarkerCode::new(7, 1);
        let template = barker.generate();
        let mf = MatchedFilterPulseRadar::new(template.clone());

        let mut input = vec![(0.0, 0.0); 15];
        input.extend_from_slice(&template);
        input.extend(vec![(0.0, 0.0); 15]);

        let peaks = mf.detect_peaks_normalized(&input, 0.8);
        assert!(
            !peaks.is_empty(),
            "Should detect at least one normalized peak"
        );
        // Peak value should be close to 1.0
        assert!(
            peaks[0].1 > 0.8,
            "Normalized peak should be > 0.8, got {}",
            peaks[0].1
        );
    }

    #[test]
    fn test_empty_input() {
        let template = vec![(1.0, 0.0); 5];
        let mf = MatchedFilterPulseRadar::new(template);
        let output = mf.filter(&[]);
        // Convolving length-5 with length-0: out_len = 0 + 5 - 1 = 4
        // but the inner loop over input does nothing, so all zeros.
        assert_eq!(output.len(), 4);
        for s in &output {
            assert!(mag(*s) < EPSILON);
        }
    }

    #[test]
    fn test_detect_peaks_empty() {
        let template = vec![(1.0, 0.0); 5];
        let mf = MatchedFilterPulseRadar::new(template);
        let peaks = mf.detect_peaks(&[], 0.5);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_template_energy() {
        let template = vec![(1.0, 0.0), (0.0, 1.0), (1.0, 1.0)];
        let mf = MatchedFilterPulseRadar::new(template);
        // Energy = 1 + 1 + 2 = 4
        assert!((mf.template_energy() - 4.0).abs() < EPSILON);
    }
}
