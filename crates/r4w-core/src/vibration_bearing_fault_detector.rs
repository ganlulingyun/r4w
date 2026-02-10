//! Spectral kurtosis and envelope analysis for rolling-element bearing fault detection.
//!
//! This module provides tools for detecting and diagnosing bearing faults from
//! vibration signals. It includes:
//!
//! - **Bearing defect frequency calculation** (BPFO, BPFI, BSF, FTF) from
//!   shaft speed and bearing geometry.
//! - **Envelope analysis** via Hilbert transform (analytic signal) and envelope
//!   spectrum computation for fault frequency identification.
//! - **Spectral kurtosis** computation per frequency band and kurtogram for
//!   optimal filter band selection.
//! - **Vibration features**: RMS, crest factor, and kurtosis.
//! - **Fault severity estimation** based on harmonic amplitude ratios.
//!
//! # Example
//!
//! ```
//! use r4w_core::vibration_bearing_fault_detector::{BearingGeometry, BearingFaultDetector};
//!
//! let geometry = BearingGeometry {
//!     ball_count: 9,
//!     pitch_diameter: 46.0e-3,
//!     ball_diameter: 7.94e-3,
//!     contact_angle_deg: 0.0,
//! };
//! let detector = BearingFaultDetector::new(48000.0, geometry);
//!
//! // Calculate defect frequencies at 30 Hz shaft speed
//! let freqs = detector.defect_frequencies(30.0);
//! assert!(freqs.bpfo > 100.0);
//! assert!(freqs.bpfi > 100.0);
//!
//! // Compute RMS of a simple sine vibration signal
//! let signal: Vec<f64> = (0..4800)
//!     .map(|i| (2.0 * std::f64::consts::PI * 100.0 * i as f64 / 48000.0).sin())
//!     .collect();
//! let rms = detector.rms(&signal);
//! assert!((rms - 1.0 / 2.0_f64.sqrt()).abs() < 0.01);
//! ```

use std::f64::consts::PI;

// ─── Complex helpers using (f64, f64) tuples ───────────────────────────────

/// Multiply two complex numbers represented as `(re, im)` tuples.
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Add two complex numbers.
#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Subtract two complex numbers.
#[inline]
fn csub(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 - b.0, a.1 - b.1)
}

/// Magnitude of a complex number.
#[inline]
fn cmag(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

/// Complex exponential: e^{j*theta}.
#[inline]
fn cexp_j(theta: f64) -> (f64, f64) {
    (theta.cos(), theta.sin())
}

// ─── FFT (radix-2 Cooley–Tukey, in-place) ─────────────────────────────────

/// In-place radix-2 decimation-in-time FFT. `buf.len()` must be a power of 2.
fn fft_in_place(buf: &mut [(f64, f64)], inverse: bool) {
    let n = buf.len();
    assert!(n.is_power_of_two(), "FFT length must be a power of two");

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            buf.swap(i, j);
        }
    }

    // Butterfly stages
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let wn = cexp_j(angle);
        let mut start = 0;
        while start < n {
            let mut w: (f64, f64) = (1.0, 0.0);
            for k in 0..half {
                let u = buf[start + k];
                let t = cmul(w, buf[start + k + half]);
                buf[start + k] = cadd(u, t);
                buf[start + k + half] = csub(u, t);
                w = cmul(w, wn);
            }
            start += len;
        }
        len <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for x in buf.iter_mut() {
            x.0 *= inv_n;
            x.1 *= inv_n;
        }
    }
}

/// Forward FFT returning a new vector. Zero-pads to the next power of 2.
fn fft(signal: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = signal.len().next_power_of_two();
    let mut buf = vec![(0.0, 0.0); n];
    buf[..signal.len()].copy_from_slice(signal);
    fft_in_place(&mut buf, false);
    buf
}

/// Inverse FFT returning a new vector.
fn ifft(spectrum: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = spectrum.len().next_power_of_two();
    let mut buf = vec![(0.0, 0.0); n];
    buf[..spectrum.len()].copy_from_slice(spectrum);
    fft_in_place(&mut buf, true);
    buf
}

// ─── Bearing geometry and defect frequencies ───────────────────────────────

/// Physical geometry of a rolling-element bearing.
#[derive(Debug, Clone)]
pub struct BearingGeometry {
    /// Number of rolling elements (balls/rollers).
    pub ball_count: u32,
    /// Pitch diameter in metres (centre-to-centre across the bearing).
    pub pitch_diameter: f64,
    /// Ball (rolling element) diameter in metres.
    pub ball_diameter: f64,
    /// Contact angle in degrees.
    pub contact_angle_deg: f64,
}

/// Characteristic defect frequencies for a bearing at a given shaft speed.
#[derive(Debug, Clone)]
pub struct DefectFrequencies {
    /// Ball Pass Frequency – Outer race (Hz).
    pub bpfo: f64,
    /// Ball Pass Frequency – Inner race (Hz).
    pub bpfi: f64,
    /// Ball Spin Frequency (Hz).
    pub bsf: f64,
    /// Fundamental Train Frequency / cage frequency (Hz).
    pub ftf: f64,
    /// Shaft rotation frequency used for computation (Hz).
    pub shaft_freq: f64,
}

/// Result of envelope spectrum analysis.
#[derive(Debug, Clone)]
pub struct EnvelopeSpectrumResult {
    /// Frequency axis (Hz) for each bin.
    pub frequencies: Vec<f64>,
    /// Magnitude of the envelope spectrum (linear).
    pub magnitudes: Vec<f64>,
}

/// Spectral kurtosis result for a single frequency band.
#[derive(Debug, Clone)]
pub struct SpectralKurtosisResult {
    /// Centre frequencies of the analysis bands (Hz).
    pub band_centres: Vec<f64>,
    /// Bandwidth of each analysis band (Hz).
    pub bandwidth: f64,
    /// Spectral kurtosis value per band.
    pub sk_values: Vec<f64>,
}

/// One cell in the kurtogram grid.
#[derive(Debug, Clone)]
pub struct KurtogramCell {
    /// Centre frequency (Hz).
    pub centre_freq: f64,
    /// Bandwidth (Hz).
    pub bandwidth: f64,
    /// Spectral kurtosis value.
    pub sk: f64,
}

/// Kurtogram result: a grid of (centre_freq, bandwidth, SK) cells.
#[derive(Debug, Clone)]
pub struct KurtogramResult {
    /// All computed cells.
    pub cells: Vec<KurtogramCell>,
    /// The cell with the highest spectral kurtosis.
    pub optimal: KurtogramCell,
}

/// Vibration statistical features.
#[derive(Debug, Clone)]
pub struct VibrationFeatures {
    /// Root-mean-square amplitude.
    pub rms: f64,
    /// Crest factor (peak / RMS).
    pub crest_factor: f64,
    /// Kurtosis of the time-domain signal.
    pub kurtosis: f64,
}

/// Fault type identifiers.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultType {
    /// Outer race defect.
    OuterRace,
    /// Inner race defect.
    InnerRace,
    /// Rolling element (ball) defect.
    BallDefect,
    /// Cage defect.
    CageDefect,
}

/// Result of fault severity estimation.
#[derive(Debug, Clone)]
pub struct FaultSeverity {
    /// Detected fault type.
    pub fault_type: FaultType,
    /// Number of harmonics detected above the noise floor.
    pub harmonics_detected: usize,
    /// Sum of harmonic amplitudes normalised to the fundamental.
    pub harmonic_ratio: f64,
    /// Severity score in [0, 1]. Higher = worse.
    pub severity: f64,
}

// ─── Main detector ─────────────────────────────────────────────────────────

/// Bearing fault detector combining envelope analysis, spectral kurtosis,
/// kurtogram, and vibration features.
#[derive(Debug, Clone)]
pub struct BearingFaultDetector {
    /// Sample rate in Hz.
    pub sample_rate: f64,
    /// Bearing geometry.
    pub geometry: BearingGeometry,
    /// Number of harmonics to check during fault severity estimation.
    pub max_harmonics: usize,
}

impl BearingFaultDetector {
    /// Create a new detector.
    ///
    /// * `sample_rate` – sampling frequency of the vibration signal (Hz).
    /// * `geometry` – physical dimensions of the bearing under test.
    pub fn new(sample_rate: f64, geometry: BearingGeometry) -> Self {
        Self {
            sample_rate,
            geometry,
            max_harmonics: 5,
        }
    }

    /// Set the number of harmonics used in fault severity estimation.
    pub fn with_max_harmonics(mut self, n: usize) -> Self {
        self.max_harmonics = n;
        self
    }

    // ── Defect frequency calculation ───────────────────────────────────

    /// Compute characteristic defect frequencies for a given shaft speed.
    ///
    /// * `shaft_freq` – shaft rotational frequency in Hz.
    pub fn defect_frequencies(&self, shaft_freq: f64) -> DefectFrequencies {
        let n = self.geometry.ball_count as f64;
        let d = self.geometry.ball_diameter;
        let dp = self.geometry.pitch_diameter;
        let alpha = self.geometry.contact_angle_deg.to_radians();
        let cos_a = alpha.cos();
        let ratio = d / dp;

        let ftf = 0.5 * shaft_freq * (1.0 - ratio * cos_a);
        let bpfo = 0.5 * n * shaft_freq * (1.0 - ratio * cos_a);
        let bpfi = 0.5 * n * shaft_freq * (1.0 + ratio * cos_a);
        let bsf = 0.5 * (dp / d) * shaft_freq * (1.0 - (ratio * cos_a).powi(2));

        DefectFrequencies {
            bpfo,
            bpfi,
            bsf,
            ftf,
            shaft_freq,
        }
    }

    // ── Hilbert transform & envelope ───────────────────────────────────

    /// Compute the analytic signal via a frequency-domain Hilbert transform.
    ///
    /// Returns complex analytic signal as `Vec<(f64, f64)>`.
    pub fn analytic_signal(&self, signal: &[f64]) -> Vec<(f64, f64)> {
        if signal.is_empty() {
            return Vec::new();
        }
        let complex_signal: Vec<(f64, f64)> = signal.iter().map(|&x| (x, 0.0)).collect();
        let mut spectrum = fft(&complex_signal);
        let n = spectrum.len();

        // Zero-out negative frequencies and double positive frequencies.
        // DC and Nyquist stay as-is.
        if n >= 2 {
            for item in spectrum.iter_mut().take(n / 2).skip(1) {
                item.0 *= 2.0;
                item.1 *= 2.0;
            }
            for item in spectrum.iter_mut().take(n).skip(n / 2 + 1) {
                *item = (0.0, 0.0);
            }
        }

        let analytic = ifft(&spectrum);
        analytic[..signal.len()].to_vec()
    }

    /// Compute the envelope (instantaneous amplitude) of the signal.
    pub fn envelope(&self, signal: &[f64]) -> Vec<f64> {
        self.analytic_signal(signal)
            .iter()
            .map(|&c| cmag(c))
            .collect()
    }

    /// Compute the envelope spectrum: FFT of the envelope signal.
    ///
    /// Returns only the positive-frequency half with a frequency axis.
    pub fn envelope_spectrum(&self, signal: &[f64]) -> EnvelopeSpectrumResult {
        let env = self.envelope(signal);
        // Remove DC from envelope
        let mean: f64 = env.iter().sum::<f64>() / env.len() as f64;
        let env_ac: Vec<(f64, f64)> = env.iter().map(|&x| (x - mean, 0.0)).collect();

        let spectrum = fft(&env_ac);
        let n = spectrum.len();
        let half = n / 2 + 1;
        let df = self.sample_rate / n as f64;

        let frequencies: Vec<f64> = (0..half).map(|i| i as f64 * df).collect();
        let magnitudes: Vec<f64> = spectrum[..half]
            .iter()
            .map(|&c| cmag(c) / n as f64)
            .collect();

        EnvelopeSpectrumResult {
            frequencies,
            magnitudes,
        }
    }

    // ── Spectral kurtosis ──────────────────────────────────────────────

    /// Compute spectral kurtosis using the Short-Time Fourier Transform (STFT).
    ///
    /// * `signal` – time-domain vibration signal.
    /// * `window_len` – STFT window length (must be power of 2).
    /// * `overlap` – number of overlapping samples between consecutive windows.
    ///
    /// Returns spectral kurtosis for each frequency bin from 0 to Nyquist.
    pub fn spectral_kurtosis(
        &self,
        signal: &[f64],
        window_len: usize,
        overlap: usize,
    ) -> SpectralKurtosisResult {
        assert!(
            window_len.is_power_of_two(),
            "window_len must be a power of two"
        );
        assert!(overlap < window_len, "overlap must be less than window_len");
        let hop = window_len - overlap;
        let half = window_len / 2 + 1;

        // Hann window
        let window: Vec<f64> = (0..window_len)
            .map(|i| 0.5 * (1.0 - (2.0 * PI * i as f64 / window_len as f64).cos()))
            .collect();

        // Collect STFT power spectra
        let mut power_frames: Vec<Vec<f64>> = Vec::new();
        let mut pos = 0;
        while pos + window_len <= signal.len() {
            let windowed: Vec<(f64, f64)> = (0..window_len)
                .map(|i| (signal[pos + i] * window[i], 0.0))
                .collect();
            let spec = fft(&windowed);
            let power: Vec<f64> = spec[..half]
                .iter()
                .map(|&c| c.0 * c.0 + c.1 * c.1)
                .collect();
            power_frames.push(power);
            pos += hop;
        }

        let num_frames = power_frames.len();
        if num_frames < 2 {
            return SpectralKurtosisResult {
                band_centres: Vec::new(),
                bandwidth: self.sample_rate / window_len as f64,
                sk_values: Vec::new(),
            };
        }

        // SK(f) = (M / (M-1)) * ( <|X|^4> / <|X|^2>^2 ) - 2
        // where M = number of frames
        let m = num_frames as f64;
        let mut sk_values = vec![0.0; half];
        for bin in 0..half {
            let mut sum2 = 0.0;
            let mut sum4 = 0.0;
            for frame in &power_frames {
                let p = frame[bin]; // |X|^2
                sum2 += p;
                sum4 += p * p;
            }
            let mean2 = sum2 / m;
            if mean2 > 1e-30 {
                sk_values[bin] = (m / (m - 1.0)) * (sum4 / m) / (mean2 * mean2) - 2.0;
            }
        }

        let df = self.sample_rate / window_len as f64;
        let band_centres: Vec<f64> = (0..half).map(|i| i as f64 * df).collect();

        SpectralKurtosisResult {
            band_centres,
            bandwidth: df,
            sk_values,
        }
    }

    /// Compute a kurtogram by evaluating spectral kurtosis at multiple
    /// window lengths (resolutions).
    ///
    /// * `signal` – time-domain vibration signal.
    /// * `min_window` – smallest STFT window length (power of 2).
    /// * `max_window` – largest STFT window length (power of 2).
    ///
    /// Returns the full grid and the optimal (centre_freq, bandwidth) cell.
    pub fn kurtogram(
        &self,
        signal: &[f64],
        min_window: usize,
        max_window: usize,
    ) -> KurtogramResult {
        assert!(min_window.is_power_of_two());
        assert!(max_window.is_power_of_two());
        assert!(min_window <= max_window);

        let mut cells: Vec<KurtogramCell> = Vec::new();
        let mut best = KurtogramCell {
            centre_freq: 0.0,
            bandwidth: 0.0,
            sk: f64::NEG_INFINITY,
        };

        let mut wlen = min_window;
        while wlen <= max_window {
            let overlap = wlen / 2;
            let sk_result = self.spectral_kurtosis(signal, wlen, overlap);
            for (i, &sk) in sk_result.sk_values.iter().enumerate() {
                let cell = KurtogramCell {
                    centre_freq: sk_result.band_centres[i],
                    bandwidth: sk_result.bandwidth,
                    sk,
                };
                if sk > best.sk {
                    best = cell.clone();
                }
                cells.push(cell);
            }
            wlen <<= 1;
        }

        KurtogramResult { cells, optimal: best }
    }

    // ── Vibration features ─────────────────────────────────────────────

    /// Root-mean-square amplitude.
    pub fn rms(&self, signal: &[f64]) -> f64 {
        if signal.is_empty() {
            return 0.0;
        }
        let sum_sq: f64 = signal.iter().map(|x| x * x).sum();
        (sum_sq / signal.len() as f64).sqrt()
    }

    /// Crest factor: peak absolute value divided by RMS.
    pub fn crest_factor(&self, signal: &[f64]) -> f64 {
        let r = self.rms(signal);
        if r < 1e-30 {
            return 0.0;
        }
        let peak = signal.iter().map(|x| x.abs()).fold(0.0_f64, f64::max);
        peak / r
    }

    /// Kurtosis of the time-domain signal (excess kurtosis, Gaussian = 0).
    pub fn kurtosis(&self, signal: &[f64]) -> f64 {
        let n = signal.len() as f64;
        if n < 4.0 {
            return 0.0;
        }
        let mean = signal.iter().sum::<f64>() / n;
        let m2: f64 = signal.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / n;
        if m2 < 1e-30 {
            return 0.0;
        }
        let m4: f64 = signal.iter().map(|x| (x - mean).powi(4)).sum::<f64>() / n;
        m4 / (m2 * m2) - 3.0
    }

    /// Compute RMS, crest factor, and kurtosis together.
    pub fn vibration_features(&self, signal: &[f64]) -> VibrationFeatures {
        VibrationFeatures {
            rms: self.rms(signal),
            crest_factor: self.crest_factor(signal),
            kurtosis: self.kurtosis(signal),
        }
    }

    // ── Fault severity ─────────────────────────────────────────────────

    /// Estimate fault severity by searching the envelope spectrum for
    /// harmonics of a candidate defect frequency.
    ///
    /// * `signal` – time-domain vibration signal.
    /// * `shaft_freq` – shaft rotational frequency (Hz).
    /// * `fault_type` – which defect frequency to search for.
    /// * `noise_floor_percentile` – percentile of spectrum magnitudes used
    ///   as the noise floor (e.g. 0.5 for median).
    pub fn estimate_fault_severity(
        &self,
        signal: &[f64],
        shaft_freq: f64,
        fault_type: FaultType,
        noise_floor_percentile: f64,
    ) -> FaultSeverity {
        let defect = self.defect_frequencies(shaft_freq);
        let target_freq = match fault_type {
            FaultType::OuterRace => defect.bpfo,
            FaultType::InnerRace => defect.bpfi,
            FaultType::BallDefect => defect.bsf,
            FaultType::CageDefect => defect.ftf,
        };

        let es = self.envelope_spectrum(signal);
        if es.frequencies.is_empty() || es.magnitudes.is_empty() {
            return FaultSeverity {
                fault_type,
                harmonics_detected: 0,
                harmonic_ratio: 0.0,
                severity: 0.0,
            };
        }

        // Estimate noise floor
        let mut sorted_mags = es.magnitudes.clone();
        sorted_mags.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let idx = ((sorted_mags.len() as f64 * noise_floor_percentile).floor() as usize)
            .min(sorted_mags.len() - 1);
        let noise_floor = sorted_mags[idx];

        let df = if es.frequencies.len() > 1 {
            es.frequencies[1] - es.frequencies[0]
        } else {
            1.0
        };

        // Search for harmonics in the envelope spectrum
        let mut harmonics_detected = 0usize;
        let mut fundamental_amp = 0.0_f64;
        let mut harmonic_sum = 0.0_f64;

        for h in 1..=self.max_harmonics {
            let freq = target_freq * h as f64;
            if freq > self.sample_rate / 2.0 {
                break;
            }
            // Find the closest bin
            let bin = (freq / df).round() as usize;
            if bin >= es.magnitudes.len() {
                break;
            }
            // Search a small neighbourhood (+/-2 bins)
            let lo = bin.saturating_sub(2);
            let hi = (bin + 2).min(es.magnitudes.len() - 1);
            let peak_amp = es.magnitudes[lo..=hi]
                .iter()
                .cloned()
                .fold(0.0_f64, f64::max);

            if peak_amp > noise_floor * 3.0 {
                harmonics_detected += 1;
                if h == 1 {
                    fundamental_amp = peak_amp;
                }
                harmonic_sum += peak_amp;
            }
        }

        let harmonic_ratio = if fundamental_amp > 1e-30 {
            harmonic_sum / fundamental_amp
        } else {
            0.0
        };

        // Simple severity score: combine harmonic count and ratio
        let max_h = self.max_harmonics as f64;
        let severity = ((harmonics_detected as f64 / max_h) * 0.5
            + (harmonic_ratio / (max_h + 1.0)).min(1.0) * 0.5)
            .min(1.0);

        FaultSeverity {
            fault_type,
            harmonics_detected,
            harmonic_ratio,
            severity,
        }
    }

    /// Bandpass filter a signal around a centre frequency with given bandwidth,
    /// using a frequency-domain rectangular window (brick-wall filter).
    ///
    /// This is useful for extracting the optimal band found by the kurtogram.
    pub fn bandpass_filter(
        &self,
        signal: &[f64],
        centre_freq: f64,
        bandwidth: f64,
    ) -> Vec<f64> {
        if signal.is_empty() {
            return Vec::new();
        }
        let complex_signal: Vec<(f64, f64)> = signal.iter().map(|&x| (x, 0.0)).collect();
        let mut spectrum = fft(&complex_signal);
        let n = spectrum.len();
        let df = self.sample_rate / n as f64;

        let lo = (centre_freq - bandwidth / 2.0).max(0.0);
        let hi = centre_freq + bandwidth / 2.0;

        for i in 0..n {
            let freq = if i <= n / 2 {
                i as f64 * df
            } else {
                (i as f64 - n as f64) * df
            };
            let abs_freq = freq.abs();
            if abs_freq < lo || abs_freq > hi {
                spectrum[i] = (0.0, 0.0);
            }
        }

        let filtered = ifft(&spectrum);
        filtered[..signal.len()].iter().map(|c| c.0).collect()
    }
}

// ─── Tests ─────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const FS: f64 = 48000.0;

    fn test_geometry() -> BearingGeometry {
        // SKF 6205 (approx)
        BearingGeometry {
            ball_count: 9,
            pitch_diameter: 38.5e-3,
            ball_diameter: 7.94e-3,
            contact_angle_deg: 0.0,
        }
    }

    fn test_detector() -> BearingFaultDetector {
        BearingFaultDetector::new(FS, test_geometry())
    }

    fn sine_signal(freq: f64, fs: f64, n: usize, amplitude: f64) -> Vec<f64> {
        (0..n)
            .map(|i| amplitude * (2.0 * PI * freq * i as f64 / fs).sin())
            .collect()
    }

    // ── 1. Defect frequency formulas ───────────────────────────────────

    #[test]
    fn test_defect_frequencies_nonzero() {
        let det = test_detector();
        let f = det.defect_frequencies(25.0);
        assert!(f.bpfo > 0.0, "BPFO should be positive");
        assert!(f.bpfi > 0.0, "BPFI should be positive");
        assert!(f.bsf > 0.0, "BSF should be positive");
        assert!(f.ftf > 0.0, "FTF should be positive");
    }

    #[test]
    fn test_bpfo_bpfi_sum() {
        // BPFO + BPFI = n * f_shaft
        let det = test_detector();
        let shaft = 30.0;
        let f = det.defect_frequencies(shaft);
        let sum = f.bpfo + f.bpfi;
        let expected = det.geometry.ball_count as f64 * shaft;
        assert!(
            (sum - expected).abs() < 1e-6,
            "BPFO + BPFI should equal n * f_shaft: got {sum}, expected {expected}"
        );
    }

    #[test]
    fn test_ftf_less_than_shaft() {
        let det = test_detector();
        let f = det.defect_frequencies(60.0);
        assert!(
            f.ftf < 60.0,
            "Cage frequency must be less than shaft frequency"
        );
        assert!(f.ftf > 0.0);
    }

    #[test]
    fn test_defect_frequencies_scale_with_shaft_speed() {
        let det = test_detector();
        let f1 = det.defect_frequencies(10.0);
        let f2 = det.defect_frequencies(20.0);
        assert!(
            (f2.bpfo / f1.bpfo - 2.0).abs() < 1e-9,
            "BPFO should scale linearly"
        );
        assert!(
            (f2.bpfi / f1.bpfi - 2.0).abs() < 1e-9,
            "BPFI should scale linearly"
        );
    }

    #[test]
    fn test_contact_angle_effect() {
        let mut geom = test_geometry();
        geom.contact_angle_deg = 0.0;
        let det0 = BearingFaultDetector::new(FS, geom.clone());
        let f0 = det0.defect_frequencies(30.0);

        geom.contact_angle_deg = 15.0;
        let det15 = BearingFaultDetector::new(FS, geom);
        let f15 = det15.defect_frequencies(30.0);

        // Non-zero contact angle reduces cos(alpha), changes frequencies
        assert!(
            (f0.bpfo - f15.bpfo).abs() > 0.01,
            "Contact angle should affect BPFO"
        );
    }

    // ── 2. Hilbert transform / analytic signal ─────────────────────────

    #[test]
    fn test_analytic_signal_length() {
        let det = test_detector();
        let sig = sine_signal(100.0, FS, 1024, 1.0);
        let analytic = det.analytic_signal(&sig);
        assert_eq!(analytic.len(), sig.len());
    }

    #[test]
    fn test_analytic_signal_envelope_of_sine() {
        // Envelope of a pure sine should be approximately constant ~ amplitude.
        let det = test_detector();
        let amp = 3.0;
        let sig = sine_signal(500.0, FS, 4096, amp);
        let env = det.envelope(&sig);
        // Skip edges (transient artefact)
        let core = &env[256..env.len() - 256];
        let mean_env: f64 = core.iter().sum::<f64>() / core.len() as f64;
        assert!(
            (mean_env - amp).abs() < 0.1,
            "Mean envelope of a sine should be close to amplitude: got {mean_env}"
        );
    }

    #[test]
    fn test_analytic_signal_empty() {
        let det = test_detector();
        let analytic = det.analytic_signal(&[]);
        assert!(analytic.is_empty());
    }

    // ── 3. Envelope spectrum ───────────────────────────────────────────

    #[test]
    fn test_envelope_spectrum_dc_removed() {
        let det = test_detector();
        let sig = sine_signal(200.0, FS, 4096, 1.0);
        let es = det.envelope_spectrum(&sig);
        // DC bin should be near zero (we subtracted the mean)
        assert!(
            es.magnitudes[0] < 0.01,
            "DC should be removed from envelope spectrum"
        );
    }

    #[test]
    fn test_envelope_spectrum_detects_am_frequency() {
        // AM signal: carrier at 5kHz modulated at 120 Hz
        // Envelope should show a peak near 120 Hz.
        let det = test_detector();
        let n = 8192;
        let carrier = 5000.0;
        let mod_freq = 120.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / FS;
                (1.0 + 0.8 * (2.0 * PI * mod_freq * t).sin())
                    * (2.0 * PI * carrier * t).sin()
            })
            .collect();

        let es = det.envelope_spectrum(&signal);
        // Find the peak in the 50-500 Hz range
        let df = es.frequencies[1] - es.frequencies[0];
        let lo_bin = (50.0 / df) as usize;
        let hi_bin = (500.0 / df) as usize;
        let (peak_bin, _peak_val) = es.magnitudes[lo_bin..hi_bin]
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        let peak_freq = es.frequencies[lo_bin + peak_bin];
        assert!(
            (peak_freq - mod_freq).abs() < df * 2.0,
            "Envelope spectrum peak should be near modulation freq: got {peak_freq} Hz"
        );
    }

    // ── 4. Spectral kurtosis ───────────────────────────────────────────

    #[test]
    fn test_spectral_kurtosis_gaussian_noise() {
        // For noise-like signal, SK should be modest.
        let det = test_detector();
        let n = 16384;
        let noise: Vec<f64> = (0..n)
            .map(|i| {
                let x = (i as f64 * 0.61803398875).fract() * 2.0 - 1.0;
                x
            })
            .collect();
        let sk = det.spectral_kurtosis(&noise, 256, 128);
        let avg_sk: f64 = sk.sk_values.iter().sum::<f64>() / sk.sk_values.len() as f64;
        assert!(
            avg_sk.abs() < 2.0,
            "Average SK for noise-like signal should be near 0, got {avg_sk}"
        );
    }

    #[test]
    fn test_spectral_kurtosis_impulsive_signal() {
        // A signal with sparse impulses: only a few STFT frames contain energy,
        // which should produce high spectral kurtosis (non-stationary).
        let det = test_detector();
        let n = 16384;
        let mut signal = vec![0.0; n];
        // Place a few isolated impulses so most frames are silent
        signal[500] = 50.0;
        signal[4000] = 50.0;
        signal[9000] = 50.0;
        let sk = det.spectral_kurtosis(&signal, 256, 128);
        let max_sk = sk
            .sk_values
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        assert!(
            max_sk > 1.0,
            "Impulsive signal should produce high SK, got max {max_sk}"
        );
    }

    #[test]
    fn test_spectral_kurtosis_band_centres() {
        let det = test_detector();
        let sig = sine_signal(1000.0, FS, 4096, 1.0);
        let sk = det.spectral_kurtosis(&sig, 256, 128);
        let df = FS / 256.0;
        assert!((sk.bandwidth - df).abs() < 1e-6);
        assert_eq!(sk.band_centres.len(), 129); // 256/2 + 1
        assert!((sk.band_centres[0]).abs() < 1e-6); // DC
    }

    // ── 5. Kurtogram ──────────────────────────────────────────────────

    #[test]
    fn test_kurtogram_returns_cells() {
        let det = test_detector();
        let n = 8192;
        let mut signal = vec![0.0; n];
        for i in (0..n).step_by(160) {
            signal[i] = 5.0;
        }
        let kg = det.kurtogram(&signal, 64, 512);
        assert!(!kg.cells.is_empty(), "Kurtogram should produce cells");
        assert!(
            kg.optimal.sk > 0.0,
            "Optimal SK should be positive for impulsive signal"
        );
    }

    #[test]
    fn test_kurtogram_optimal_bandwidth_varies() {
        let det = test_detector();
        let n = 8192;
        let mut signal = vec![0.0; n];
        for i in (0..n).step_by(80) {
            signal[i] = 8.0;
        }
        let kg = det.kurtogram(&signal, 64, 256);
        // We should have cells from window lengths 64, 128, 256
        let bandwidths: Vec<f64> = kg.cells.iter().map(|c| c.bandwidth).collect();
        let unique_bw: std::collections::HashSet<u64> =
            bandwidths.iter().map(|b| (*b * 1000.0) as u64).collect();
        assert!(
            unique_bw.len() >= 2,
            "Kurtogram should span multiple bandwidths"
        );
    }

    // ── 6. Vibration features ──────────────────────────────────────────

    #[test]
    fn test_rms_sine() {
        let det = test_detector();
        let sig = sine_signal(100.0, FS, 48000, 1.0);
        let rms = det.rms(&sig);
        let expected = 1.0 / 2.0_f64.sqrt();
        assert!(
            (rms - expected).abs() < 0.001,
            "RMS of unit sine should be 1/sqrt(2), got {rms}"
        );
    }

    #[test]
    fn test_rms_empty() {
        let det = test_detector();
        assert_eq!(det.rms(&[]), 0.0);
    }

    #[test]
    fn test_crest_factor_sine() {
        let det = test_detector();
        let sig = sine_signal(100.0, FS, 48000, 1.0);
        let cf = det.crest_factor(&sig);
        let expected = 2.0_f64.sqrt(); // peak(1) / rms(1/sqrt(2))
        assert!(
            (cf - expected).abs() < 0.01,
            "Crest factor of sine should be sqrt(2), got {cf}"
        );
    }

    #[test]
    fn test_kurtosis_sine() {
        // Kurtosis of a pure sine wave is -1.5 (excess kurtosis).
        let det = test_detector();
        let sig = sine_signal(100.0, FS, 48000, 1.0);
        let k = det.kurtosis(&sig);
        assert!(
            (k - (-1.5)).abs() < 0.05,
            "Excess kurtosis of sine should be ~ -1.5, got {k}"
        );
    }

    #[test]
    fn test_vibration_features_struct() {
        let det = test_detector();
        let sig = sine_signal(200.0, FS, 48000, 2.0);
        let feats = det.vibration_features(&sig);
        assert!(feats.rms > 0.0);
        assert!(feats.crest_factor > 0.0);
        assert!(feats.kurtosis < 0.0); // sine has negative excess kurtosis
    }

    // ── 7. Fault severity ──────────────────────────────────────────────

    #[test]
    fn test_fault_severity_with_injected_fault() {
        // Create a signal with an AM component at the BPFO frequency.
        let det = test_detector();
        let shaft = 30.0;
        let defect = det.defect_frequencies(shaft);
        let bpfo = defect.bpfo;

        let n = 32768;
        let carrier = 4000.0;
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / FS;
                // Strong AM at BPFO and its 2nd harmonic
                let modulation = 1.0
                    + 0.5 * (2.0 * PI * bpfo * t).sin()
                    + 0.2 * (2.0 * PI * 2.0 * bpfo * t).sin();
                modulation * (2.0 * PI * carrier * t).sin()
            })
            .collect();

        let severity =
            det.estimate_fault_severity(&signal, shaft, FaultType::OuterRace, 0.5);
        assert!(
            severity.harmonics_detected >= 1,
            "Should detect at least one harmonic, got {}",
            severity.harmonics_detected
        );
        assert!(
            severity.severity > 0.0,
            "Severity should be > 0 for a faulty signal"
        );
    }

    #[test]
    fn test_fault_severity_healthy_signal() {
        // A constant-amplitude signal has a flat envelope, so no harmonic
        // peaks should appear in the envelope spectrum.
        let det = test_detector();
        let n = 16384;
        // Constant value -- envelope is constant, envelope spectrum is zero
        let signal = vec![1.0; n];

        let severity =
            det.estimate_fault_severity(&signal, 30.0, FaultType::InnerRace, 0.5);
        assert_eq!(
            severity.harmonics_detected, 0,
            "Constant signal should have no detected harmonics"
        );
        assert!(
            severity.severity < 0.01,
            "Constant signal should have near-zero severity, got {}",
            severity.severity
        );
    }

    // ── 8. Bandpass filter ─────────────────────────────────────────────

    #[test]
    fn test_bandpass_filter_isolates_tone() {
        let det = test_detector();
        let n = 4096;
        // Two tones: 500 Hz and 5000 Hz
        let signal: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / FS;
                (2.0 * PI * 500.0 * t).sin() + (2.0 * PI * 5000.0 * t).sin()
            })
            .collect();

        // Bandpass around 5000 Hz (+/- 500 Hz)
        let filtered = det.bandpass_filter(&signal, 5000.0, 1000.0);
        assert_eq!(filtered.len(), signal.len());

        // The 500 Hz component should be heavily attenuated
        let ref_500: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * 500.0 * i as f64 / FS).sin())
            .collect();
        let corr_500: f64 = filtered
            .iter()
            .zip(ref_500.iter())
            .map(|(a, b)| a * b)
            .sum::<f64>()
            / n as f64;
        assert!(
            corr_500.abs() < 0.1,
            "500 Hz should be attenuated, correlation = {corr_500}"
        );
    }

    #[test]
    fn test_bandpass_filter_empty() {
        let det = test_detector();
        assert!(det.bandpass_filter(&[], 1000.0, 500.0).is_empty());
    }

    // ── 9. Builder / configuration ─────────────────────────────────────

    #[test]
    fn test_with_max_harmonics() {
        let det = test_detector().with_max_harmonics(10);
        assert_eq!(det.max_harmonics, 10);
    }
}
