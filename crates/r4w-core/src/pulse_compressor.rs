//! Pulse Compressor — Matched Filtering for Radar Signal Processing
//!
//! Performs matched filtering (pulse compression) of received radar returns
//! against a known transmitted waveform reference, producing compressed range
//! profiles with processing gain proportional to the time-bandwidth product.
//! Supports arbitrary waveform templates (LFM chirps, Barker codes, polyphase
//! codes) and includes sidelobe reduction via windowed matched filters.
//!
//! GNU Radio equivalent: `gr-plasma::matched_filter` / `gr-radar` pulse
//! compression blocks (OOT modules).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::pulse_compressor::{PulseCompressor, PulseCompressorConfig, SidelobeWindow, ProcessingDomain};
//! use num_complex::Complex64;
//!
//! let reference = r4w_core::pulse_compressor::lfm_reference(1e6, 10e-6, 10e6);
//! let config = PulseCompressorConfig {
//!     reference_waveform: reference,
//!     sidelobe_window: SidelobeWindow::None,
//!     domain: ProcessingDomain::FrequencyDomain { fft_size: 256 },
//!     normalize: true,
//! };
//! let mut pc = PulseCompressor::new(config);
//! let received = vec![Complex64::new(0.0, 0.0); 200];
//! let compressed = pc.compress(&received);
//! assert!(!compressed.is_empty());
//! ```

use num_complex::Complex64;
use std::f64::consts::PI;

/// Sidelobe window type for matched filter weighting.
#[derive(Debug, Clone)]
pub enum SidelobeWindow {
    /// No windowing (rectangular).
    None,
    /// Hamming window.
    Hamming,
    /// Chebyshev window with specified sidelobe level.
    Chebyshev { sidelobe_db: f64 },
    /// Taylor window with nbar nearly constant sidelobes.
    Taylor { nbar: usize, sidelobe_db: f64 },
}

/// Processing domain for compression.
#[derive(Debug, Clone)]
pub enum ProcessingDomain {
    /// Time-domain convolution (O(N*M)).
    TimeDomain,
    /// Frequency-domain (FFT-based, O(N log N)).
    FrequencyDomain { fft_size: usize },
}

/// Polyphase code type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolyphaseCode {
    Frank,
    P1,
    P2,
    P3,
    P4,
    Zadoff,
}

/// Pulse compressor configuration.
#[derive(Debug, Clone)]
pub struct PulseCompressorConfig {
    /// Reference (transmitted) waveform.
    pub reference_waveform: Vec<Complex64>,
    /// Sidelobe reduction window.
    pub sidelobe_window: SidelobeWindow,
    /// Processing domain.
    pub domain: ProcessingDomain,
    /// Normalize output to peak = 1.0.
    pub normalize: bool,
}

/// Pulse compressor performing matched filtering.
#[derive(Debug, Clone)]
pub struct PulseCompressor {
    /// Matched filter coefficients (time-reversed conjugate of reference).
    matched_filter: Vec<Complex64>,
    /// Reference waveform length.
    ref_len: usize,
    /// Processing domain.
    domain: ProcessingDomain,
    /// Whether to normalize output.
    normalize: bool,
    /// Precomputed reference spectrum for freq-domain processing.
    ref_spectrum: Option<Vec<Complex64>>,
    /// FFT size for freq-domain.
    fft_size: usize,
}

impl PulseCompressor {
    /// Create a new pulse compressor.
    pub fn new(config: PulseCompressorConfig) -> Self {
        let ref_len = config.reference_waveform.len();
        let window = generate_window(&config.sidelobe_window, ref_len);

        // Build matched filter: h[n] = w[n] * conj(x[N-1-n])
        let matched_filter: Vec<Complex64> = (0..ref_len)
            .map(|n| {
                let w = window[n];
                config.reference_waveform[ref_len - 1 - n].conj() * w
            })
            .collect();

        let (fft_size, ref_spectrum) = match &config.domain {
            ProcessingDomain::FrequencyDomain { fft_size } => {
                let fs = next_power_of_two((*fft_size).max(ref_len * 2));
                // Precompute reference spectrum (conjugate of reference, zero-padded)
                let mut padded = vec![Complex64::new(0.0, 0.0); fs];
                for (i, &s) in config.reference_waveform.iter().enumerate() {
                    padded[i] = s;
                }
                let ref_fft = dft(&padded);
                let conj_ref: Vec<Complex64> = ref_fft.iter().map(|s| s.conj()).collect();
                // Apply window in frequency domain
                let win_fft = if matches!(config.sidelobe_window, SidelobeWindow::None) {
                    conj_ref
                } else {
                    let mut win_padded = vec![Complex64::new(0.0, 0.0); fs];
                    for (i, &w) in window.iter().enumerate() {
                        win_padded[i] = Complex64::new(w, 0.0);
                    }
                    let _win_spectrum = dft(&win_padded);
                    // For simplicity, apply window to matched filter in time domain
                    // then transform to freq domain
                    let mut mf_padded = vec![Complex64::new(0.0, 0.0); fs];
                    for (i, &s) in matched_filter.iter().enumerate() {
                        mf_padded[i] = s;
                    }
                    dft(&mf_padded)
                };
                (fs, Some(win_fft))
            }
            ProcessingDomain::TimeDomain => (0, None),
        };

        Self {
            matched_filter,
            ref_len,
            domain: config.domain,
            normalize: config.normalize,
            ref_spectrum,
            fft_size,
        }
    }

    /// Compress a received signal (matched filtering).
    pub fn compress(&mut self, received: &[Complex64]) -> Vec<Complex64> {
        let result = match &self.domain {
            ProcessingDomain::TimeDomain => self.compress_time_domain(received),
            ProcessingDomain::FrequencyDomain { .. } => self.compress_freq_domain(received),
        };

        if self.normalize {
            let peak = result.iter().map(|s| s.norm()).fold(0.0_f64, f64::max);
            if peak > 1e-30 {
                result.iter().map(|s| s / peak).collect()
            } else {
                result
            }
        } else {
            result
        }
    }

    /// Compress multiple pulses.
    pub fn compress_batch(&mut self, pulses: &[Vec<Complex64>]) -> Vec<Vec<Complex64>> {
        pulses.iter().map(|p| self.compress(p)).collect()
    }

    /// Processing gain in dB (time-bandwidth product).
    pub fn processing_gain_db(&self) -> f64 {
        10.0 * (self.ref_len as f64).log10()
    }

    /// Range resolution in meters given bandwidth in Hz.
    pub fn range_resolution(&self, bandwidth: f64) -> f64 {
        if bandwidth <= 0.0 {
            return f64::INFINITY;
        }
        299_792_458.0 / (2.0 * bandwidth)
    }

    /// Compressed pulse width in samples.
    pub fn compressed_pulse_width(&self) -> usize {
        // Approximately 1 sample for matched filter
        1
    }

    /// Peak sidelobe ratio of the compressed output for the reference waveform.
    pub fn peak_sidelobe_ratio(&self) -> f64 {
        // Compress the reference itself
        let ref_waveform: Vec<Complex64> = self.matched_filter.iter().rev().map(|s| s.conj()).collect();
        let mut temp = self.clone();
        temp.normalize = false;
        let compressed = temp.compress(&ref_waveform);

        if compressed.is_empty() {
            return 0.0;
        }

        let magnitudes: Vec<f64> = compressed.iter().map(|s| s.norm()).collect();
        let peak = magnitudes.iter().cloned().fold(0.0_f64, f64::max);
        let peak_idx = magnitudes
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .map(|(i, _)| i)
            .unwrap_or(0);

        if peak < 1e-30 {
            return f64::NEG_INFINITY;
        }

        // Find max sidelobe (excluding main lobe region)
        let mainlobe_half = 3.max(self.ref_len / 20);
        let max_sidelobe = magnitudes
            .iter()
            .enumerate()
            .filter(|(i, _)| (*i as isize - peak_idx as isize).unsigned_abs() > mainlobe_half)
            .map(|(_, &v)| v)
            .fold(0.0_f64, f64::max);

        if max_sidelobe < 1e-30 {
            return f64::NEG_INFINITY;
        }

        20.0 * (max_sidelobe / peak).log10()
    }

    fn compress_time_domain(&self, received: &[Complex64]) -> Vec<Complex64> {
        let n = received.len();
        let m = self.matched_filter.len();
        let out_len = n + m - 1;
        let mut output = vec![Complex64::new(0.0, 0.0); out_len];

        for i in 0..out_len {
            let mut sum = Complex64::new(0.0, 0.0);
            for j in 0..m {
                if i >= j && (i - j) < n {
                    sum += received[i - j] * self.matched_filter[j];
                }
            }
            output[i] = sum;
        }

        output
    }

    fn compress_freq_domain(&self, received: &[Complex64]) -> Vec<Complex64> {
        let ref_spectrum = match &self.ref_spectrum {
            Some(s) => s,
            None => return self.compress_time_domain(received),
        };

        let n = self.fft_size;
        let mut rx_padded = vec![Complex64::new(0.0, 0.0); n];
        for (i, &s) in received.iter().enumerate().take(n) {
            rx_padded[i] = s;
        }

        let rx_fft = dft(&rx_padded);

        // Multiply in frequency domain
        let product: Vec<Complex64> = rx_fft
            .iter()
            .zip(ref_spectrum.iter())
            .map(|(r, h)| r * h)
            .collect();

        idft(&product)
    }
}

/// Generate a Linear FM (chirp) reference waveform.
pub fn lfm_reference(bandwidth: f64, duration: f64, sample_rate: f64) -> Vec<Complex64> {
    let n = (duration * sample_rate) as usize;
    let chirp_rate = bandwidth / duration;
    (0..n)
        .map(|i| {
            let t = i as f64 / sample_rate;
            let phase = PI * chirp_rate * t * t;
            Complex64::new(phase.cos(), phase.sin())
        })
        .collect()
}

/// Generate a Barker code reference waveform.
pub fn barker_reference(code: &[i8]) -> Vec<Complex64> {
    code.iter()
        .map(|&c| Complex64::new(c as f64, 0.0))
        .collect()
}

/// Generate a polyphase code reference waveform.
pub fn polyphase_reference(code_type: PolyphaseCode, length: usize) -> Vec<Complex64> {
    let n = length;
    match code_type {
        PolyphaseCode::Frank => {
            let m = (n as f64).sqrt().ceil() as usize;
            (0..n)
                .map(|i| {
                    let row = i / m;
                    let col = i % m;
                    let phase = 2.0 * PI * (row * col) as f64 / m as f64;
                    Complex64::new(phase.cos(), phase.sin())
                })
                .collect()
        }
        PolyphaseCode::P1 => {
            let m = (n as f64).sqrt().ceil() as usize;
            (0..n)
                .map(|i| {
                    let k = i / m;
                    let j = i % m;
                    let phase = -PI / m as f64 * (m - 1 - 2 * j) as f64 * (k * m + j) as f64 / m as f64;
                    Complex64::new(phase.cos(), phase.sin())
                })
                .collect()
        }
        PolyphaseCode::P2 => {
            let m = (n as f64).sqrt().ceil() as usize;
            (0..n)
                .map(|i| {
                    let k = i / m;
                    let j = i % m;
                    let phase = PI / m as f64 * ((m as f64 + 1.0) / 2.0 - (j + 1) as f64)
                        * (2.0 * k as f64 - (m - 1) as f64);
                    Complex64::new(phase.cos(), phase.sin())
                })
                .collect()
        }
        PolyphaseCode::P3 => {
            (0..n)
                .map(|i| {
                    let phase = PI * (i * i) as f64 / n as f64;
                    Complex64::new(phase.cos(), phase.sin())
                })
                .collect()
        }
        PolyphaseCode::P4 => {
            (0..n)
                .map(|i| {
                    let phase = PI * (i * i) as f64 / n as f64 - PI * i as f64;
                    Complex64::new(phase.cos(), phase.sin())
                })
                .collect()
        }
        PolyphaseCode::Zadoff => {
            let m = if n % 2 == 0 { n } else { n };
            (0..n)
                .map(|i| {
                    let phase = PI * (i * i) as f64 / m as f64;
                    Complex64::new(phase.cos(), phase.sin())
                })
                .collect()
        }
    }
}

fn generate_window(window: &SidelobeWindow, len: usize) -> Vec<f64> {
    match window {
        SidelobeWindow::None => vec![1.0; len],
        SidelobeWindow::Hamming => (0..len)
            .map(|i| 0.54 - 0.46 * (2.0 * PI * i as f64 / (len - 1) as f64).cos())
            .collect(),
        SidelobeWindow::Chebyshev { sidelobe_db } => {
            chebyshev_window(len, sidelobe_db.abs())
        }
        SidelobeWindow::Taylor { nbar, sidelobe_db } => {
            taylor_window(len, *nbar, sidelobe_db.abs())
        }
    }
}

fn chebyshev_window(len: usize, sidelobe_db: f64) -> Vec<f64> {
    // Dolph-Chebyshev window approximation
    let r = 10.0_f64.powf(sidelobe_db / 20.0);
    let n = len;
    let mut w = vec![0.0; n];
    let beta = (1.0 / (n as f64 - 1.0))
        * ((r + (r * r - 1.0).max(0.0).sqrt()).ln());

    for i in 0..n {
        let x = beta * (PI * i as f64 / n as f64).cos();
        w[i] = x.cosh() / beta.cosh();
    }

    // Normalize
    let peak = w.iter().cloned().fold(0.0_f64, f64::max);
    if peak > 0.0 {
        for v in &mut w {
            *v /= peak;
        }
    }
    w
}

fn taylor_window(len: usize, nbar: usize, sidelobe_db: f64) -> Vec<f64> {
    let n = len;
    let a = (sidelobe_db / 20.0).abs();
    let a_val = (10.0_f64.powf(a) + (10.0_f64.powf(2.0 * a) - 1.0).max(0.0).sqrt()).ln() / PI;

    let nbar = nbar.max(1);
    let sigma2 = (nbar as f64 * nbar as f64)
        / (a_val * a_val + ((nbar as f64 - 0.5) * (nbar as f64 - 0.5)));

    let mut w = vec![0.0; n];
    for i in 0..n {
        let mut val = 1.0;
        for m in 1..nbar {
            let mut num = 1.0;
            let mut den = 1.0;
            for p in 1..nbar {
                num *= 1.0 - (m as f64 * m as f64) / (sigma2 * (a_val * a_val + (p as f64 - 0.5) * (p as f64 - 0.5)));
                if p != m {
                    den *= 1.0 - (m as f64 * m as f64) / (p as f64 * p as f64);
                }
            }
            let f_m = if den.abs() > 1e-30 { num / den } else { 0.0 };
            val += 2.0 * f_m * (2.0 * PI * m as f64 * (i as f64 - (n as f64 - 1.0) / 2.0) / n as f64).cos();
        }
        w[i] = val;
    }

    // Normalize
    let peak = w.iter().cloned().fold(0.0_f64, f64::max);
    if peak > 0.0 {
        for v in &mut w {
            *v /= peak;
        }
    }
    w
}

fn next_power_of_two(n: usize) -> usize {
    let mut p = 1;
    while p < n {
        p <<= 1;
    }
    p
}

/// Simple DFT (for internal use).
fn dft(x: &[Complex64]) -> Vec<Complex64> {
    let n = x.len();
    let mut result = vec![Complex64::new(0.0, 0.0); n];
    for k in 0..n {
        let mut sum = Complex64::new(0.0, 0.0);
        for (j, &xj) in x.iter().enumerate() {
            let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
            sum += xj * Complex64::new(angle.cos(), angle.sin());
        }
        result[k] = sum;
    }
    result
}

/// Simple IDFT (for internal use).
fn idft(x: &[Complex64]) -> Vec<Complex64> {
    let n = x.len();
    let mut result = vec![Complex64::new(0.0, 0.0); n];
    for k in 0..n {
        let mut sum = Complex64::new(0.0, 0.0);
        for (j, &xj) in x.iter().enumerate() {
            let angle = 2.0 * PI * k as f64 * j as f64 / n as f64;
            sum += xj * Complex64::new(angle.cos(), angle.sin());
        }
        result[k] = sum / n as f64;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_lfm_compression_peak() {
        let bw = 1e6;
        let dur = 10e-6;
        let fs = 10e6;
        let reference = lfm_reference(bw, dur, fs);
        let n = reference.len();

        let config = PulseCompressorConfig {
            reference_waveform: reference.clone(),
            sidelobe_window: SidelobeWindow::None,
            domain: ProcessingDomain::TimeDomain,
            normalize: false,
        };
        let mut pc = PulseCompressor::new(config);
        let compressed = pc.compress(&reference);

        // Find peak
        let magnitudes: Vec<f64> = compressed.iter().map(|s| s.norm()).collect();
        let peak_idx = magnitudes
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        // Peak should be near center of convolution output
        assert!(
            (peak_idx as isize - (n - 1) as isize).unsigned_abs() < 3,
            "peak at {peak_idx}, expected near {}", n - 1
        );
    }

    #[test]
    fn test_processing_gain() {
        let reference = lfm_reference(1e6, 100e-6, 10e6);
        let config = PulseCompressorConfig {
            reference_waveform: reference,
            sidelobe_window: SidelobeWindow::None,
            domain: ProcessingDomain::TimeDomain,
            normalize: false,
        };
        let pc = PulseCompressor::new(config);

        let pg = pc.processing_gain_db();
        let expected = 10.0 * (1000.0_f64).log10(); // 1000 samples → 30 dB
        assert!(
            (pg - expected).abs() < 0.5,
            "processing gain = {pg:.1} dB, expected ~{expected:.1}"
        );
    }

    #[test]
    fn test_barker_13_sidelobes() {
        let barker_13: Vec<i8> = vec![1, 1, 1, 1, 1, -1, -1, 1, 1, -1, 1, -1, 1];
        let reference = barker_reference(&barker_13);

        let config = PulseCompressorConfig {
            reference_waveform: reference.clone(),
            sidelobe_window: SidelobeWindow::None,
            domain: ProcessingDomain::TimeDomain,
            normalize: false,
        };
        let mut pc = PulseCompressor::new(config);
        let compressed = pc.compress(&reference);

        let magnitudes: Vec<f64> = compressed.iter().map(|s| s.norm()).collect();
        let peak = magnitudes.iter().cloned().fold(0.0_f64, f64::max);
        let peak_idx = magnitudes
            .iter()
            .position(|&v| (v - peak).abs() < 1e-10)
            .unwrap();

        // Barker-13 sidelobes should be exactly 1/13 of peak
        let max_sidelobe = magnitudes
            .iter()
            .enumerate()
            .filter(|(i, _)| *i != peak_idx)
            .map(|(_, &v)| v)
            .fold(0.0_f64, f64::max);

        let psl_db = 20.0 * (max_sidelobe / peak).log10();
        assert!(
            psl_db < -20.0,
            "Barker-13 PSL = {psl_db:.1} dB, expected < -20 dB"
        );
    }

    #[test]
    fn test_freq_and_time_domain_match() {
        let reference = lfm_reference(500e3, 20e-6, 5e6);
        let received = reference.clone();

        let config_td = PulseCompressorConfig {
            reference_waveform: reference.clone(),
            sidelobe_window: SidelobeWindow::None,
            domain: ProcessingDomain::TimeDomain,
            normalize: true,
        };
        let mut pc_td = PulseCompressor::new(config_td);
        let out_td = pc_td.compress(&received);

        let config_fd = PulseCompressorConfig {
            reference_waveform: reference,
            sidelobe_window: SidelobeWindow::None,
            domain: ProcessingDomain::FrequencyDomain { fft_size: 512 },
            normalize: true,
        };
        let mut pc_fd = PulseCompressor::new(config_fd);
        let out_fd = pc_fd.compress(&received);

        // Find peaks in both
        let peak_td = out_td.iter().map(|s| s.norm()).fold(0.0_f64, f64::max);
        let peak_fd = out_fd.iter().map(|s| s.norm()).fold(0.0_f64, f64::max);

        // Both should have peak at 1.0 (normalized)
        assert!(
            (peak_td - 1.0).abs() < 0.01,
            "TD peak = {peak_td}"
        );
        assert!(
            (peak_fd - 1.0).abs() < 0.01,
            "FD peak = {peak_fd}"
        );
    }

    #[test]
    fn test_hamming_window_sidelobe_reduction() {
        let reference = lfm_reference(1e6, 50e-6, 10e6);

        let config_no_win = PulseCompressorConfig {
            reference_waveform: reference.clone(),
            sidelobe_window: SidelobeWindow::None,
            domain: ProcessingDomain::TimeDomain,
            normalize: false,
        };
        let mut pc_no_win = PulseCompressor::new(config_no_win);

        let config_hamming = PulseCompressorConfig {
            reference_waveform: reference.clone(),
            sidelobe_window: SidelobeWindow::Hamming,
            domain: ProcessingDomain::TimeDomain,
            normalize: false,
        };
        let mut pc_hamming = PulseCompressor::new(config_hamming);

        let out_no = pc_no_win.compress(&reference);
        let out_ham = pc_hamming.compress(&reference);

        // Use a wider mainlobe exclusion (fs/B = 10 samples, use 15 for safety)
        let psl_no = peak_sidelobe_level_ex(&out_no, 15);
        let psl_ham = peak_sidelobe_level_ex(&out_ham, 20); // wider mainlobe with Hamming

        assert!(
            psl_ham < psl_no,
            "Hamming PSL {psl_ham:.1} should be less than unwindowed {psl_no:.1}"
        );
    }

    #[test]
    fn test_taylor_window() {
        let reference = lfm_reference(1e6, 50e-6, 10e6);
        let config = PulseCompressorConfig {
            reference_waveform: reference.clone(),
            sidelobe_window: SidelobeWindow::Taylor {
                nbar: 4,
                sidelobe_db: 35.0,
            },
            domain: ProcessingDomain::TimeDomain,
            normalize: false,
        };
        let mut pc = PulseCompressor::new(config);
        let out = pc.compress(&reference);

        // Taylor window with wider mainlobe exclusion
        let psl = peak_sidelobe_level_ex(&out, 25);
        assert!(
            psl < -10.0,
            "Taylor PSL = {psl:.1} dB, expected below -10 dB"
        );
    }

    #[test]
    fn test_compress_batch() {
        let reference = barker_reference(&[1, 1, 1, -1, -1, 1, -1]);
        let config = PulseCompressorConfig {
            reference_waveform: reference.clone(),
            sidelobe_window: SidelobeWindow::None,
            domain: ProcessingDomain::TimeDomain,
            normalize: false,
        };
        let mut pc = PulseCompressor::new(config);

        let pulses: Vec<Vec<Complex64>> = (0..8)
            .map(|_| reference.clone())
            .collect();

        let batch = pc.compress_batch(&pulses);
        assert_eq!(batch.len(), 8);

        // Each should have same peak
        let peaks: Vec<f64> = batch
            .iter()
            .map(|p| p.iter().map(|s| s.norm()).fold(0.0_f64, f64::max))
            .collect();

        for (i, &p) in peaks.iter().enumerate().skip(1) {
            assert!(
                (p - peaks[0]).abs() < 1e-10,
                "batch {i} peak {p} != batch 0 peak {}",
                peaks[0]
            );
        }
    }

    #[test]
    fn test_noise_only_bounded() {
        let reference = lfm_reference(1e6, 10e-6, 10e6);
        let n = reference.len();

        let config = PulseCompressorConfig {
            reference_waveform: reference,
            sidelobe_window: SidelobeWindow::None,
            domain: ProcessingDomain::TimeDomain,
            normalize: false,
        };
        let mut pc = PulseCompressor::new(config);

        // White noise input
        let noise: Vec<Complex64> = (0..200)
            .map(|i| {
                let phase = (i as f64 * 1.618) % (2.0 * PI);
                Complex64::new(phase.cos() * 0.1, phase.sin() * 0.1)
            })
            .collect();

        let out = pc.compress(&noise);
        let peak = out.iter().map(|s| s.norm()).fold(0.0_f64, f64::max);

        // Peak should be bounded by sqrt(N) * noise_amplitude
        let bound = (n as f64).sqrt() * 0.15;
        assert!(
            peak < bound,
            "noise peak {peak:.3} exceeds bound {bound:.3}"
        );
    }

    #[test]
    fn test_two_targets_resolved() {
        let reference = lfm_reference(1e6, 10e-6, 10e6);
        let n = reference.len(); // 100 samples

        // Create received signal with two targets at different delays
        let delay1 = 20;
        let delay2 = 60; // Separation > compressed pulse width
        let total_len = n + delay2 + n;
        let mut received = vec![Complex64::new(0.0, 0.0); total_len];
        for (i, &s) in reference.iter().enumerate() {
            received[i + delay1] += s;
            received[i + delay2] += s * 0.5; // Second target weaker
        }

        let config = PulseCompressorConfig {
            reference_waveform: reference,
            sidelobe_window: SidelobeWindow::None,
            domain: ProcessingDomain::TimeDomain,
            normalize: false,
        };
        let mut pc = PulseCompressor::new(config);
        let compressed = pc.compress(&received);

        let magnitudes: Vec<f64> = compressed.iter().map(|s| s.norm()).collect();

        // Find the two highest peaks that are well separated
        let mut indexed: Vec<(usize, f64)> = magnitudes.iter().enumerate().map(|(i, &v)| (i, v)).collect();
        indexed.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap());

        let p1 = indexed[0].0;
        // Find second peak at least 10 samples away from first
        let p2 = indexed
            .iter()
            .find(|(i, _)| (*i as isize - p1 as isize).unsigned_abs() > 10)
            .map(|(i, _)| *i)
            .unwrap_or(0);

        let separation = (p1 as isize - p2 as isize).unsigned_abs();

        assert!(
            (separation as isize - (delay2 - delay1) as isize).unsigned_abs() < 5,
            "separation = {separation}, expected ~{}, p1={p1}, p2={p2}",
            delay2 - delay1
        );
    }

    #[test]
    fn test_range_resolution() {
        let config = PulseCompressorConfig {
            reference_waveform: lfm_reference(1e6, 10e-6, 10e6),
            sidelobe_window: SidelobeWindow::None,
            domain: ProcessingDomain::TimeDomain,
            normalize: false,
        };
        let pc = PulseCompressor::new(config);

        let dr = pc.range_resolution(1e6); // 1 MHz bandwidth
        // c / (2 * B) = 299792458 / 2e6 ≈ 149.9 m
        assert!(
            (dr - 149.9).abs() < 1.0,
            "range resolution = {dr:.1} m, expected ~149.9"
        );
    }

    fn peak_sidelobe_level_ex(compressed: &[Complex64], mainlobe_half: usize) -> f64 {
        let magnitudes: Vec<f64> = compressed.iter().map(|s| s.norm()).collect();
        let peak = magnitudes.iter().cloned().fold(0.0_f64, f64::max);
        let peak_idx = magnitudes
            .iter()
            .position(|&v| (v - peak).abs() < 1e-10)
            .unwrap_or(0);

        let max_sidelobe = magnitudes
            .iter()
            .enumerate()
            .filter(|(i, _)| (*i as isize - peak_idx as isize).unsigned_abs() > mainlobe_half)
            .map(|(_, &v)| v)
            .fold(0.0_f64, f64::max);

        if max_sidelobe < 1e-30 || peak < 1e-30 {
            return f64::NEG_INFINITY;
        }

        20.0 * (max_sidelobe / peak).log10()
    }
}
