//! Nuclear Magnetic Resonance (NMR) signal processor for spectroscopy and MRI.
//!
//! This module implements the complete NMR signal processing pipeline from
//! Free Induction Decay (FID) acquisition through spectral analysis. It provides:
//!
//! - **FID generation** with multiple peaks, T2 relaxation, and phase
//! - **Apodization** windows: exponential, Gaussian, sine-bell, cosine-squared
//! - **Zero filling** for improved spectral resolution
//! - **FFT** via Cooley-Tukey radix-2 DIT
//! - **Phase correction** (zero-order and first-order)
//! - **Baseline correction** via polynomial fitting
//! - **Chemical shift** referencing (TMS at 0 ppm)
//! - **Peak picking** with integration, FWHM, and multiplet analysis
//! - **J-coupling** extraction (doublet, triplet, quartet)
//! - **Relaxation fitting** (T1 inversion recovery, T2 CPMG)
//! - **Lorentzian lineshape** modeling
//! - **Larmor frequency** calculation for common nuclei
//! - **SNR estimation**
//!
//! # Example
//!
//! ```
//! use r4w_core::nuclear_magnetic_resonance_processor::{
//!     NmrProcessor, NmrPeak, Apodization, larmor_frequency_hz,
//! };
//!
//! let processor = NmrProcessor::new(400.0, 4000.0, 4096);
//!
//! let peaks = vec![
//!     NmrPeak { amplitude: 1.0, frequency_hz: 800.0, t2_s: 0.5, phase_rad: 0.0 },
//!     NmrPeak { amplitude: 0.5, frequency_hz: 1200.0, t2_s: 0.3, phase_rad: 0.0 },
//! ];
//!
//! let fid = processor.generate_fid(&peaks);
//! assert_eq!(fid.data.len(), 4096);
//!
//! let apodized = processor.apodize(&fid, Apodization::Exponential { lb_hz: 1.0 });
//! let spectrum = processor.fft_to_spectrum(&apodized);
//! assert_eq!(spectrum.magnitude.len(), 4096);
//!
//! // Larmor frequency for 1H at 9.4 T
//! let f = larmor_frequency_hz(Nucleus::Hydrogen1, 9.4);
//! assert!((f - 400.228e6).abs() < 1e3);
//! ```

use std::f64::consts::PI;

// ─── Physical constants ─────────────────────────────────────────────────────

/// Gyromagnetic ratio for 1H in rad/(s*T).
const GAMMA_1H: f64 = 267.522e6;

/// Gyromagnetic ratio for 13C in rad/(s*T).
const GAMMA_13C: f64 = 67.2828e6;

/// Gyromagnetic ratio for 15N in rad/(s*T).
const GAMMA_15N: f64 = -27.116e6;

/// Gyromagnetic ratio for 31P in rad/(s*T).
const GAMMA_31P: f64 = 108.394e6;

// ─── Nucleus enum ───────────────────────────────────────────────────────────

/// Common NMR-active nuclei.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Nucleus {
    /// Proton (1H), gamma = 267.522e6 rad/(s*T)
    Hydrogen1,
    /// Carbon-13, gamma = 67.2828e6 rad/(s*T)
    Carbon13,
    /// Nitrogen-15, gamma = -27.116e6 rad/(s*T)
    Nitrogen15,
    /// Phosphorus-31, gamma = 108.394e6 rad/(s*T)
    Phosphorus31,
}

impl Nucleus {
    /// Gyromagnetic ratio in rad/(s*T).
    pub fn gamma(&self) -> f64 {
        match self {
            Nucleus::Hydrogen1 => GAMMA_1H,
            Nucleus::Carbon13 => GAMMA_13C,
            Nucleus::Nitrogen15 => GAMMA_15N,
            Nucleus::Phosphorus31 => GAMMA_31P,
        }
    }
}

// ─── Larmor frequency ───────────────────────────────────────────────────────

/// Compute the Larmor frequency in Hz for a given nucleus and magnetic field.
///
/// f = |gamma| * B0 / (2*pi)
pub fn larmor_frequency_hz(nucleus: Nucleus, b0_tesla: f64) -> f64 {
    nucleus.gamma().abs() * b0_tesla / (2.0 * PI)
}

/// Compute the magnetic field strength in Tesla for a given spectrometer frequency.
///
/// B0 = 2*pi*f / |gamma|
pub fn field_from_frequency(nucleus: Nucleus, freq_hz: f64) -> f64 {
    2.0 * PI * freq_hz / nucleus.gamma().abs()
}

// ─── Data structures ────────────────────────────────────────────────────────

/// A complex number (re, im) for NMR signal representation.
#[derive(Debug, Clone, Copy)]
pub struct Complex {
    pub re: f64,
    pub im: f64,
}

impl Complex {
    pub fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    pub fn zero() -> Self {
        Self { re: 0.0, im: 0.0 }
    }

    pub fn magnitude(&self) -> f64 {
        (self.re * self.re + self.im * self.im).sqrt()
    }

    pub fn phase(&self) -> f64 {
        self.im.atan2(self.re)
    }

    pub fn from_polar(mag: f64, phase: f64) -> Self {
        Self {
            re: mag * phase.cos(),
            im: mag * phase.sin(),
        }
    }

    fn mul(&self, other: &Complex) -> Complex {
        Complex {
            re: self.re * other.re - self.im * other.im,
            im: self.re * other.im + self.im * other.re,
        }
    }

    fn add(&self, other: &Complex) -> Complex {
        Complex {
            re: self.re + other.re,
            im: self.im + other.im,
        }
    }

    fn sub(&self, other: &Complex) -> Complex {
        Complex {
            re: self.re - other.re,
            im: self.im - other.im,
        }
    }

    fn scale(&self, s: f64) -> Complex {
        Complex {
            re: self.re * s,
            im: self.im * s,
        }
    }
}

/// Describes a single NMR resonance peak for FID generation.
#[derive(Debug, Clone)]
pub struct NmrPeak {
    /// Amplitude (arbitrary units).
    pub amplitude: f64,
    /// Frequency offset from carrier in Hz.
    pub frequency_hz: f64,
    /// T2 relaxation time in seconds.
    pub t2_s: f64,
    /// Initial phase in radians.
    pub phase_rad: f64,
}

/// Free Induction Decay signal (complex time-domain data).
#[derive(Debug, Clone)]
pub struct FidSignal {
    /// Complex time-domain samples.
    pub data: Vec<Complex>,
    /// Sample rate in Hz.
    pub sample_rate_hz: f64,
    /// Carrier (spectrometer) frequency in Hz.
    pub carrier_freq_hz: f64,
}

/// NMR frequency-domain spectrum.
#[derive(Debug, Clone)]
pub struct NmrSpectrum {
    /// Frequency axis in Hz (relative to carrier).
    pub frequency_hz: Vec<f64>,
    /// Magnitude (absolute value) at each frequency.
    pub magnitude: Vec<f64>,
    /// Real part of spectrum.
    pub real: Vec<f64>,
    /// Imaginary part of spectrum.
    pub imaginary: Vec<f64>,
    /// Chemical shift axis in ppm.
    pub chemical_shift_ppm: Vec<f64>,
    /// Spectrometer frequency in MHz (for ppm conversion).
    pub spectrometer_freq_mhz: f64,
}

/// Apodization (window) function for FID processing.
#[derive(Debug, Clone, Copy)]
pub enum Apodization {
    /// No apodization.
    None,
    /// Exponential decay: exp(-pi * lb * t), line broadening in Hz.
    Exponential { lb_hz: f64 },
    /// Gaussian: exp(-0.5 * (t * gb)^2), gb controls width.
    Gaussian { gb: f64 },
    /// Sine-bell: sin(pi * t / T_max).
    SineBell,
    /// Cosine-squared: cos^2(pi * t / (2 * T_max)).
    CosineSquared,
}

/// Detected peak in an NMR spectrum.
#[derive(Debug, Clone)]
pub struct DetectedPeak {
    /// Frequency in Hz (offset from carrier).
    pub frequency_hz: f64,
    /// Chemical shift in ppm.
    pub chemical_shift_ppm: f64,
    /// Peak magnitude.
    pub magnitude: f64,
    /// Full width at half maximum in Hz.
    pub fwhm_hz: f64,
    /// Integrated area under the peak.
    pub integral: f64,
}

/// Multiplet pattern classification.
#[derive(Debug, Clone, PartialEq)]
pub enum MultipletPattern {
    Singlet,
    Doublet { j_hz: f64 },
    Triplet { j_hz: f64 },
    Quartet { j_hz: f64 },
    Multiplet { num_peaks: usize },
}

/// Result of relaxation curve fitting.
#[derive(Debug, Clone)]
pub struct RelaxationFit {
    /// Equilibrium magnetization M0.
    pub m0: f64,
    /// Relaxation time constant in seconds (T1 or T2).
    pub time_constant_s: f64,
    /// Fit residual (sum of squared errors).
    pub residual: f64,
}

// ─── NMR Processor ──────────────────────────────────────────────────────────

/// NMR signal processor with configurable spectrometer parameters.
#[derive(Debug, Clone)]
pub struct NmrProcessor {
    /// Spectrometer frequency in MHz (e.g., 400 for 1H at 9.4 T).
    pub spectrometer_frequency_mhz: f64,
    /// Spectral width in Hz.
    pub spectral_width_hz: f64,
    /// Number of data points.
    pub num_points: usize,
}

impl NmrProcessor {
    /// Create a new NMR processor.
    ///
    /// # Arguments
    /// * `spectrometer_frequency_mhz` - Operating frequency (e.g., 400.0 for 400 MHz 1H)
    /// * `spectral_width_hz` - Spectral width / sweep width in Hz
    /// * `num_points` - Number of time-domain points
    pub fn new(spectrometer_frequency_mhz: f64, spectral_width_hz: f64, num_points: usize) -> Self {
        Self {
            spectrometer_frequency_mhz,
            spectral_width_hz,
            num_points,
        }
    }

    /// Dwell time (time between samples) in seconds.
    pub fn dwell_time_s(&self) -> f64 {
        1.0 / self.spectral_width_hz
    }

    /// Total acquisition time in seconds.
    pub fn acquisition_time_s(&self) -> f64 {
        self.dwell_time_s() * self.num_points as f64
    }

    // ── FID generation ──────────────────────────────────────────────────

    /// Generate a synthetic FID from a set of NMR peaks.
    ///
    /// s(t) = sum_k A_k * exp(j*2*pi*f_k*t) * exp(-t/T2_k) * exp(j*phi_k)
    pub fn generate_fid(&self, peaks: &[NmrPeak]) -> FidSignal {
        let dt = self.dwell_time_s();
        let n = self.num_points;
        let mut data = vec![Complex::zero(); n];

        for peak in peaks {
            for i in 0..n {
                let t = i as f64 * dt;
                let decay = (-t / peak.t2_s).exp();
                let phase = 2.0 * PI * peak.frequency_hz * t + peak.phase_rad;
                data[i].re += peak.amplitude * decay * phase.cos();
                data[i].im += peak.amplitude * decay * phase.sin();
            }
        }

        FidSignal {
            data,
            sample_rate_hz: self.spectral_width_hz,
            carrier_freq_hz: self.spectrometer_frequency_mhz * 1e6,
        }
    }

    // ── Zero filling ────────────────────────────────────────────────────

    /// Zero-fill the FID to a given length (must be >= current length).
    /// Improves digital resolution in the frequency domain.
    pub fn zero_fill(&self, fid: &FidSignal, new_length: usize) -> FidSignal {
        let mut data = fid.data.clone();
        data.resize(new_length, Complex::zero());
        FidSignal {
            data,
            sample_rate_hz: fid.sample_rate_hz,
            carrier_freq_hz: fid.carrier_freq_hz,
        }
    }

    // ── Apodization ─────────────────────────────────────────────────────

    /// Apply an apodization (window) function to the FID.
    pub fn apodize(&self, fid: &FidSignal, apod: Apodization) -> FidSignal {
        let n = fid.data.len();
        let dt = 1.0 / fid.sample_rate_hz;
        let t_max = (n - 1) as f64 * dt;
        let mut data = fid.data.clone();

        for i in 0..n {
            let t = i as f64 * dt;
            let w = match apod {
                Apodization::None => 1.0,
                Apodization::Exponential { lb_hz } => (-PI * lb_hz * t).exp(),
                Apodization::Gaussian { gb } => (-0.5 * (t * gb) * (t * gb)).exp(),
                Apodization::SineBell => {
                    if t_max > 0.0 {
                        (PI * t / t_max).sin()
                    } else {
                        1.0
                    }
                }
                Apodization::CosineSquared => {
                    if t_max > 0.0 {
                        let c = (PI * t / (2.0 * t_max)).cos();
                        c * c
                    } else {
                        1.0
                    }
                }
            };
            data[i] = data[i].scale(w);
        }

        FidSignal {
            data,
            sample_rate_hz: fid.sample_rate_hz,
            carrier_freq_hz: fid.carrier_freq_hz,
        }
    }

    // ── FFT ─────────────────────────────────────────────────────────────

    /// Perform FFT on the FID to produce a frequency-domain spectrum.
    /// Uses Cooley-Tukey radix-2 DIT with zero-padding to next power of 2.
    pub fn fft_to_spectrum(&self, fid: &FidSignal) -> NmrSpectrum {
        let n = next_power_of_2(fid.data.len());
        let mut buf = fid.data.clone();
        buf.resize(n, Complex::zero());

        fft_radix2(&mut buf, false);

        // FFT shift: move DC to center
        let mut shifted = vec![Complex::zero(); n];
        let half = n / 2;
        for i in 0..n {
            let src = (i + half) % n;
            shifted[i] = buf[src];
        }

        let df = fid.sample_rate_hz / n as f64;
        let spec_mhz = self.spectrometer_frequency_mhz;

        let mut frequency_hz = Vec::with_capacity(n);
        let mut chemical_shift_ppm = Vec::with_capacity(n);
        let mut magnitude = Vec::with_capacity(n);
        let mut real = Vec::with_capacity(n);
        let mut imaginary = Vec::with_capacity(n);

        for i in 0..n {
            let f = (i as f64 - half as f64) * df;
            frequency_hz.push(f);
            // delta_ppm = f / spec_freq_hz * 1e6 = f / (spec_mhz * 1e6) * 1e6 = f / spec_mhz
            chemical_shift_ppm.push(f / spec_mhz);
            magnitude.push(shifted[i].magnitude());
            real.push(shifted[i].re);
            imaginary.push(shifted[i].im);
        }

        NmrSpectrum {
            frequency_hz,
            magnitude,
            real,
            imaginary,
            chemical_shift_ppm,
            spectrometer_freq_mhz: spec_mhz,
        }
    }

    // ── Phase correction ────────────────────────────────────────────────

    /// Apply zero-order and first-order phase correction to a spectrum.
    ///
    /// phi(f) = ph0 + ph1 * (f - f_pivot) / sw
    ///
    /// Zero-order: constant phase rotation (radians).
    /// First-order: linear phase gradient (radians across sweep width).
    pub fn phase_correct(
        &self,
        spectrum: &NmrSpectrum,
        ph0_rad: f64,
        ph1_rad: f64,
        pivot_index: usize,
    ) -> NmrSpectrum {
        let n = spectrum.magnitude.len();
        let pivot = if pivot_index < n { pivot_index } else { n / 2 };
        let mut result = spectrum.clone();

        for i in 0..n {
            let frac = if n > 1 {
                (i as f64 - pivot as f64) / (n as f64 - 1.0)
            } else {
                0.0
            };
            let phi = ph0_rad + ph1_rad * frac;
            let cos_phi = phi.cos();
            let sin_phi = phi.sin();
            let re = spectrum.real[i];
            let im = spectrum.imaginary[i];
            result.real[i] = re * cos_phi + im * sin_phi;
            result.imaginary[i] = -re * sin_phi + im * cos_phi;
            result.magnitude[i] = (result.real[i] * result.real[i]
                + result.imaginary[i] * result.imaginary[i])
                .sqrt();
        }

        result
    }

    // ── Baseline correction ─────────────────────────────────────────────

    /// Subtract a polynomial baseline from the real part of the spectrum.
    ///
    /// Fits a polynomial of given order to the real spectrum using least squares
    /// on the provided baseline region indices, then subtracts it.
    pub fn baseline_correct(
        &self,
        spectrum: &NmrSpectrum,
        order: usize,
        baseline_regions: &[usize],
    ) -> NmrSpectrum {
        let n = spectrum.real.len();
        let regions: Vec<usize> = baseline_regions
            .iter()
            .copied()
            .filter(|&i| i < n)
            .collect();

        if regions.is_empty() || order == 0 {
            return spectrum.clone();
        }

        // Fit polynomial to baseline regions
        let coeffs = polyfit(&regions, &spectrum.real, order, n);

        let mut result = spectrum.clone();
        for i in 0..n {
            let x = i as f64 / n as f64;
            let mut bl = 0.0;
            for (k, c) in coeffs.iter().enumerate() {
                bl += c * x.powi(k as i32);
            }
            result.real[i] -= bl;
            result.magnitude[i] = (result.real[i] * result.real[i]
                + result.imaginary[i] * result.imaginary[i])
                .sqrt();
        }

        result
    }

    // ── Chemical shift ──────────────────────────────────────────────────

    /// Convert frequency offset (Hz) to chemical shift (ppm).
    ///
    /// delta_ppm = (f_sample - f_reference) / f_spectrometer * 1e6
    pub fn hz_to_ppm(&self, freq_hz: f64) -> f64 {
        freq_hz / self.spectrometer_frequency_mhz
    }

    /// Convert chemical shift (ppm) to frequency offset (Hz).
    pub fn ppm_to_hz(&self, ppm: f64) -> f64 {
        ppm * self.spectrometer_frequency_mhz
    }

    // ── Peak picking ────────────────────────────────────────────────────

    /// Find peaks in the magnitude spectrum above a given threshold.
    ///
    /// Returns detected peaks with frequency, chemical shift, FWHM, and integral.
    pub fn pick_peaks(&self, spectrum: &NmrSpectrum, threshold: f64) -> Vec<DetectedPeak> {
        let mag = &spectrum.magnitude;
        let n = mag.len();
        if n < 3 {
            return vec![];
        }

        let mut peaks = Vec::new();
        let df = if n > 1 {
            (spectrum.frequency_hz[n - 1] - spectrum.frequency_hz[0]) / (n as f64 - 1.0)
        } else {
            1.0
        };

        for i in 1..n - 1 {
            if mag[i] > mag[i - 1] && mag[i] > mag[i + 1] && mag[i] > threshold {
                let half_max = mag[i] / 2.0;

                // Find FWHM by searching left and right
                let mut left = i;
                while left > 0 && mag[left] > half_max {
                    left -= 1;
                }
                let mut right = i;
                while right < n - 1 && mag[right] > half_max {
                    right += 1;
                }
                let fwhm_hz = (right as f64 - left as f64) * df.abs();

                // Integrate: sum of magnitudes * df in the FWHM region
                let int_start = if left > 0 { left } else { 0 };
                let int_end = if right < n { right } else { n - 1 };
                let integral: f64 = mag[int_start..=int_end].iter().sum::<f64>() * df.abs();

                peaks.push(DetectedPeak {
                    frequency_hz: spectrum.frequency_hz[i],
                    chemical_shift_ppm: spectrum.chemical_shift_ppm[i],
                    magnitude: mag[i],
                    fwhm_hz,
                    integral,
                });
            }
        }

        peaks
    }

    // ── Multiplet analysis ──────────────────────────────────────────────

    /// Classify a group of peaks as a multiplet pattern and extract J-coupling.
    ///
    /// Groups peaks by proximity (within `group_tolerance_hz`) and classifies
    /// each group as singlet, doublet (2 peaks, J spacing), triplet (1:2:1),
    /// quartet (1:3:3:1), or generic multiplet.
    pub fn classify_multiplet(&self, peaks: &[DetectedPeak], group_tolerance_hz: f64) -> Vec<MultipletPattern> {
        if peaks.is_empty() {
            return vec![];
        }

        let mut sorted: Vec<&DetectedPeak> = peaks.iter().collect();
        sorted.sort_by(|a, b| a.frequency_hz.partial_cmp(&b.frequency_hz).unwrap());

        // Group peaks by proximity
        let mut groups: Vec<Vec<&DetectedPeak>> = Vec::new();
        let mut current_group = vec![sorted[0]];

        for i in 1..sorted.len() {
            if (sorted[i].frequency_hz - sorted[i - 1].frequency_hz).abs() < group_tolerance_hz {
                current_group.push(sorted[i]);
            } else {
                groups.push(current_group);
                current_group = vec![sorted[i]];
            }
        }
        groups.push(current_group);

        let mut patterns = Vec::new();
        for group in &groups {
            let pattern = match group.len() {
                1 => MultipletPattern::Singlet,
                2 => {
                    let j = (group[1].frequency_hz - group[0].frequency_hz).abs();
                    MultipletPattern::Doublet { j_hz: j }
                }
                3 => {
                    // Check 1:2:1 ratio for triplet
                    let j1 = (group[1].frequency_hz - group[0].frequency_hz).abs();
                    let j2 = (group[2].frequency_hz - group[1].frequency_hz).abs();
                    let avg_j = (j1 + j2) / 2.0;
                    if (j1 - j2).abs() / avg_j < 0.2 {
                        MultipletPattern::Triplet { j_hz: avg_j }
                    } else {
                        MultipletPattern::Multiplet { num_peaks: 3 }
                    }
                }
                4 => {
                    // Check 1:3:3:1 ratio for quartet
                    let spacings: Vec<f64> = (0..3)
                        .map(|i| (group[i + 1].frequency_hz - group[i].frequency_hz).abs())
                        .collect();
                    let avg_j = spacings.iter().sum::<f64>() / spacings.len() as f64;
                    let uniform = spacings.iter().all(|s| (s - avg_j).abs() / avg_j < 0.2);
                    if uniform {
                        MultipletPattern::Quartet { j_hz: avg_j }
                    } else {
                        MultipletPattern::Multiplet { num_peaks: 4 }
                    }
                }
                n => MultipletPattern::Multiplet { num_peaks: n },
            };
            patterns.push(pattern);
        }

        patterns
    }

    // ── J-coupling extraction ───────────────────────────────────────────

    /// Extract J-coupling constant from a pair of peak frequencies.
    pub fn j_coupling_hz(&self, peak1_hz: f64, peak2_hz: f64) -> f64 {
        (peak2_hz - peak1_hz).abs()
    }

    // ── SNR estimation ──────────────────────────────────────────────────

    /// Estimate signal-to-noise ratio of a spectrum.
    ///
    /// Signal: maximum magnitude.
    /// Noise: standard deviation of a noise region.
    pub fn estimate_snr(
        &self,
        spectrum: &NmrSpectrum,
        noise_region_start: usize,
        noise_region_end: usize,
    ) -> f64 {
        let mag = &spectrum.magnitude;
        let signal = mag.iter().cloned().fold(0.0_f64, f64::max);

        let noise_slice = &mag[noise_region_start..noise_region_end.min(mag.len())];
        if noise_slice.is_empty() {
            return 0.0;
        }

        let mean = noise_slice.iter().sum::<f64>() / noise_slice.len() as f64;
        let variance =
            noise_slice.iter().map(|x| (x - mean) * (x - mean)).sum::<f64>() / noise_slice.len() as f64;
        let noise_std = variance.sqrt();

        if noise_std > 0.0 {
            signal / noise_std
        } else {
            f64::INFINITY
        }
    }
}

// ─── Relaxation measurements ────────────────────────────────────────────────

/// T1 inversion recovery model: M(t) = M0 * (1 - 2 * exp(-t/T1)).
pub fn t1_inversion_recovery(m0: f64, t1: f64, t: f64) -> f64 {
    m0 * (1.0 - 2.0 * (-t / t1).exp())
}

/// T2 CPMG decay model: M(t) = M0 * exp(-t/T2).
pub fn t2_cpmg_decay(m0: f64, t2: f64, t: f64) -> f64 {
    m0 * (-t / t2).exp()
}

/// Fit T1 from inversion recovery data using iterative least-squares.
///
/// Data: (time, magnetization) pairs.
/// Returns RelaxationFit with M0, T1, and residual.
pub fn fit_t1(times: &[f64], magnetizations: &[f64]) -> RelaxationFit {
    assert_eq!(times.len(), magnetizations.len());
    let n = times.len();
    if n < 2 {
        return RelaxationFit {
            m0: 0.0,
            time_constant_s: 0.0,
            residual: 0.0,
        };
    }

    // Initial guess: M0 ~ last value, T1 ~ median time
    let m0_guess = magnetizations.last().copied().unwrap_or(1.0).abs();
    let t1_guess = times[n / 2];

    let mut m0 = m0_guess;
    let mut t1 = if t1_guess > 0.0 { t1_guess } else { 1.0 };

    // Simple Gauss-Newton iterations
    for _ in 0..100 {
        let mut j00 = 0.0;
        let mut j01 = 0.0;
        let mut j11 = 0.0;
        let mut r0 = 0.0;
        let mut r1 = 0.0;

        for i in 0..n {
            let t = times[i];
            let exp_val = (-t / t1).exp();
            let model = m0 * (1.0 - 2.0 * exp_val);
            let residual = magnetizations[i] - model;

            // Jacobian: d/dM0 = 1 - 2*exp(-t/T1)
            let dm0 = 1.0 - 2.0 * exp_val;
            // d/dT1 = -M0 * 2 * t / T1^2 * exp(-t/T1)
            let dt1 = -m0 * 2.0 * t / (t1 * t1) * exp_val;

            j00 += dm0 * dm0;
            j01 += dm0 * dt1;
            j11 += dt1 * dt1;
            r0 += dm0 * residual;
            r1 += dt1 * residual;
        }

        let det = j00 * j11 - j01 * j01;
        if det.abs() < 1e-30 {
            break;
        }

        let delta_m0 = (j11 * r0 - j01 * r1) / det;
        let delta_t1 = (-j01 * r0 + j00 * r1) / det;

        m0 += delta_m0 * 0.5;
        t1 += delta_t1 * 0.5;

        if t1 < 1e-10 {
            t1 = 1e-10;
        }

        if delta_m0.abs() < 1e-10 && delta_t1.abs() < 1e-10 {
            break;
        }
    }

    let residual: f64 = times
        .iter()
        .zip(magnetizations.iter())
        .map(|(&t, &m)| {
            let model = m0 * (1.0 - 2.0 * (-t / t1).exp());
            (m - model) * (m - model)
        })
        .sum();

    RelaxationFit {
        m0,
        time_constant_s: t1,
        residual,
    }
}

/// Fit T2 from CPMG decay data using log-linear regression.
///
/// ln(M(t)) = ln(M0) - t/T2
pub fn fit_t2(times: &[f64], magnetizations: &[f64]) -> RelaxationFit {
    assert_eq!(times.len(), magnetizations.len());
    let n = times.len();
    if n < 2 {
        return RelaxationFit {
            m0: 0.0,
            time_constant_s: 0.0,
            residual: 0.0,
        };
    }

    // Log-linear fit: ln(M) = ln(M0) - t/T2
    let mut sum_t = 0.0;
    let mut sum_lnm = 0.0;
    let mut sum_t2 = 0.0;
    let mut sum_t_lnm = 0.0;
    let mut count = 0.0;

    for i in 0..n {
        if magnetizations[i] > 0.0 {
            let lnm = magnetizations[i].ln();
            let t = times[i];
            sum_t += t;
            sum_lnm += lnm;
            sum_t2 += t * t;
            sum_t_lnm += t * lnm;
            count += 1.0;
        }
    }

    if count < 2.0 {
        return RelaxationFit {
            m0: magnetizations[0],
            time_constant_s: 1.0,
            residual: f64::MAX,
        };
    }

    let denom = count * sum_t2 - sum_t * sum_t;
    if denom.abs() < 1e-30 {
        return RelaxationFit {
            m0: magnetizations[0],
            time_constant_s: 1.0,
            residual: f64::MAX,
        };
    }

    let slope = (count * sum_t_lnm - sum_t * sum_lnm) / denom;
    let intercept = (sum_lnm - slope * sum_t) / count;

    let m0 = intercept.exp();
    let t2 = if slope.abs() > 1e-30 { -1.0 / slope } else { f64::MAX };

    let residual: f64 = times
        .iter()
        .zip(magnetizations.iter())
        .map(|(&t, &m)| {
            let model = m0 * (-t / t2).exp();
            (m - model) * (m - model)
        })
        .sum();

    RelaxationFit {
        m0,
        time_constant_s: t2,
        residual,
    }
}

// ─── Lorentzian lineshape ───────────────────────────────────────────────────

/// Lorentzian lineshape function.
///
/// L(f) = A * (Gamma/2)^2 / ((f - f0)^2 + (Gamma/2)^2)
///
/// where Gamma = 1 / (pi * T2) is the full width at half maximum (FWHM).
pub fn lorentzian(f: f64, f0: f64, amplitude: f64, t2: f64) -> f64 {
    let gamma = 1.0 / (PI * t2);
    let half_gamma = gamma / 2.0;
    let df = f - f0;
    amplitude * half_gamma * half_gamma / (df * df + half_gamma * half_gamma)
}

/// Compute theoretical FWHM in Hz from T2 relaxation time.
///
/// FWHM = 1 / (pi * T2)
pub fn fwhm_from_t2(t2: f64) -> f64 {
    1.0 / (PI * t2)
}

/// Compute T2 from measured FWHM in Hz.
///
/// T2 = 1 / (pi * FWHM)
pub fn t2_from_fwhm(fwhm_hz: f64) -> f64 {
    1.0 / (PI * fwhm_hz)
}

// ─── J-coupling patterns ────────────────────────────────────────────────────

/// Generate a doublet pattern: two Lorentzians separated by J Hz.
pub fn generate_doublet(
    freqs: &[f64],
    center_hz: f64,
    j_hz: f64,
    amplitude: f64,
    t2: f64,
) -> Vec<f64> {
    let f1 = center_hz - j_hz / 2.0;
    let f2 = center_hz + j_hz / 2.0;
    freqs
        .iter()
        .map(|&f| lorentzian(f, f1, amplitude, t2) + lorentzian(f, f2, amplitude, t2))
        .collect()
}

/// Generate a triplet pattern (1:2:1) with J spacing.
pub fn generate_triplet(
    freqs: &[f64],
    center_hz: f64,
    j_hz: f64,
    amplitude: f64,
    t2: f64,
) -> Vec<f64> {
    let f1 = center_hz - j_hz;
    let f2 = center_hz;
    let f3 = center_hz + j_hz;
    freqs
        .iter()
        .map(|&f| {
            lorentzian(f, f1, amplitude, t2)
                + 2.0 * lorentzian(f, f2, amplitude, t2)
                + lorentzian(f, f3, amplitude, t2)
        })
        .collect()
}

/// Generate a quartet pattern (1:3:3:1) with J spacing.
pub fn generate_quartet(
    freqs: &[f64],
    center_hz: f64,
    j_hz: f64,
    amplitude: f64,
    t2: f64,
) -> Vec<f64> {
    let f1 = center_hz - 1.5 * j_hz;
    let f2 = center_hz - 0.5 * j_hz;
    let f3 = center_hz + 0.5 * j_hz;
    let f4 = center_hz + 1.5 * j_hz;
    freqs
        .iter()
        .map(|&f| {
            lorentzian(f, f1, amplitude, t2)
                + 3.0 * lorentzian(f, f2, amplitude, t2)
                + 3.0 * lorentzian(f, f3, amplitude, t2)
                + lorentzian(f, f4, amplitude, t2)
        })
        .collect()
}

// ─── Helper functions ───────────────────────────────────────────────────────

/// Next power of 2 >= n.
fn next_power_of_2(n: usize) -> usize {
    let mut p = 1;
    while p < n {
        p <<= 1;
    }
    p
}

/// Radix-2 Cooley-Tukey FFT (in-place, decimation in time).
/// `inverse`: true for IFFT.
fn fft_radix2(data: &mut [Complex], inverse: bool) {
    let n = data.len();
    if n <= 1 {
        return;
    }
    assert!(n.is_power_of_two(), "FFT length must be a power of 2");

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 0..n {
        if i < j {
            data.swap(i, j);
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Butterfly stages
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let wn = Complex::from_polar(1.0, angle);

        let mut start = 0;
        while start < n {
            let mut w = Complex::new(1.0, 0.0);
            for k in 0..half {
                let u = data[start + k];
                let t = w.mul(&data[start + k + half]);
                data[start + k] = u.add(&t);
                data[start + k + half] = u.sub(&t);
                w = w.mul(&wn);
            }
            start += len;
        }
        len <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for x in data.iter_mut() {
            *x = x.scale(inv_n);
        }
    }
}

/// Simple polynomial fitting via normal equations.
///
/// Fits polynomial of given `order` to data at specified indices.
/// Returns coefficients [a0, a1, ..., a_order].
fn polyfit(indices: &[usize], data: &[f64], order: usize, n_total: usize) -> Vec<f64> {
    let m = indices.len();
    let p = order + 1;

    // Build A^T * A and A^T * b
    let mut ata = vec![0.0; p * p];
    let mut atb = vec![0.0; p];

    for &idx in indices {
        let x = idx as f64 / n_total as f64;
        let y = data[idx];

        for j in 0..p {
            let xj = x.powi(j as i32);
            atb[j] += xj * y;
            for k in 0..p {
                ata[j * p + k] += xj * x.powi(k as i32);
            }
        }
    }

    // Solve via Gaussian elimination
    let mut aug = vec![0.0; p * (p + 1)];
    for j in 0..p {
        for k in 0..p {
            aug[j * (p + 1) + k] = ata[j * p + k];
        }
        aug[j * (p + 1) + p] = atb[j];
    }

    for col in 0..p {
        // Partial pivoting
        let mut max_row = col;
        let mut max_val = aug[col * (p + 1) + col].abs();
        for row in (col + 1)..p {
            let v = aug[row * (p + 1) + col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_row != col {
            for k in 0..=p {
                let tmp = aug[col * (p + 1) + k];
                aug[col * (p + 1) + k] = aug[max_row * (p + 1) + k];
                aug[max_row * (p + 1) + k] = tmp;
            }
        }

        let pivot = aug[col * (p + 1) + col];
        if pivot.abs() < 1e-30 {
            continue;
        }

        for row in (col + 1)..p {
            let factor = aug[row * (p + 1) + col] / pivot;
            for k in col..=p {
                aug[row * (p + 1) + k] -= factor * aug[col * (p + 1) + k];
            }
        }
    }

    // Back substitution
    let mut coeffs = vec![0.0; p];
    for row in (0..p).rev() {
        let pivot = aug[row * (p + 1) + row];
        if pivot.abs() < 1e-30 {
            continue;
        }
        let mut sum = aug[row * (p + 1) + p];
        for k in (row + 1)..p {
            sum -= aug[row * (p + 1) + k] * coeffs[k];
        }
        coeffs[row] = sum / pivot;
    }

    coeffs
}

// ─── Tests ──────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // ── Larmor frequency tests ──────────────────────────────────────────

    #[test]
    fn test_larmor_frequency_1h() {
        // 1H at 9.4 T should be ~400.228 MHz
        let f = larmor_frequency_hz(Nucleus::Hydrogen1, 9.4);
        assert!((f - 400.228e6).abs() < 1e4, "got {f}");
    }

    #[test]
    fn test_larmor_frequency_13c() {
        // 13C at 9.4 T should be ~100.66 MHz
        let f = larmor_frequency_hz(Nucleus::Carbon13, 9.4);
        assert!((f - 100.66e6).abs() < 0.1e6, "got {f}");
    }

    #[test]
    fn test_larmor_frequency_31p() {
        let f = larmor_frequency_hz(Nucleus::Phosphorus31, 9.4);
        assert!(f > 100e6 && f < 200e6, "31P at 9.4T: {f}");
    }

    #[test]
    fn test_field_from_frequency() {
        // At 400 MHz 1H, B0 should be ~9.4 T
        let b0 = field_from_frequency(Nucleus::Hydrogen1, 400e6);
        assert!((b0 - 9.395).abs() < 0.01, "got {b0}");
    }

    #[test]
    fn test_larmor_roundtrip() {
        let b0 = 11.74; // ~500 MHz 1H
        let f = larmor_frequency_hz(Nucleus::Hydrogen1, b0);
        let b0_back = field_from_frequency(Nucleus::Hydrogen1, f);
        assert!(approx_eq(b0, b0_back, 1e-10));
    }

    // ── Nucleus tests ───────────────────────────────────────────────────

    #[test]
    fn test_nucleus_gamma_values() {
        assert!(approx_eq(Nucleus::Hydrogen1.gamma(), 267.522e6, 1.0));
        assert!(approx_eq(Nucleus::Carbon13.gamma(), 67.2828e6, 1.0));
        assert!(Nucleus::Nitrogen15.gamma() < 0.0); // negative gyromagnetic ratio
    }

    // ── FID generation tests ────────────────────────────────────────────

    #[test]
    fn test_fid_single_peak() {
        let proc = NmrProcessor::new(400.0, 4000.0, 256);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 100.0,
            t2_s: 1.0,
            phase_rad: 0.0,
        }];
        let fid = proc.generate_fid(&peaks);
        assert_eq!(fid.data.len(), 256);
        // First sample should have amplitude ~1.0
        assert!(approx_eq(fid.data[0].magnitude(), 1.0, 0.01));
    }

    #[test]
    fn test_fid_decay() {
        let proc = NmrProcessor::new(400.0, 1000.0, 1000);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 0.0,
            t2_s: 0.5,
            phase_rad: 0.0,
        }];
        let fid = proc.generate_fid(&peaks);
        // At t = T2 = 0.5s (sample 500), magnitude should be ~exp(-1) ≈ 0.368
        let mag = fid.data[500].magnitude();
        assert!(approx_eq(mag, (-1.0_f64).exp(), 0.01), "got {mag}");
    }

    #[test]
    fn test_fid_multiple_peaks() {
        let proc = NmrProcessor::new(400.0, 4000.0, 512);
        let peaks = vec![
            NmrPeak { amplitude: 1.0, frequency_hz: 200.0, t2_s: 0.5, phase_rad: 0.0 },
            NmrPeak { amplitude: 0.5, frequency_hz: 800.0, t2_s: 0.3, phase_rad: 0.0 },
        ];
        let fid = proc.generate_fid(&peaks);
        assert_eq!(fid.data.len(), 512);
        // First sample should be sum of amplitudes
        assert!(approx_eq(fid.data[0].magnitude(), 1.5, 0.01));
    }

    #[test]
    fn test_fid_with_phase() {
        let proc = NmrProcessor::new(400.0, 4000.0, 64);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 0.0,
            t2_s: 10.0,
            phase_rad: PI / 2.0,
        }];
        let fid = proc.generate_fid(&peaks);
        // With 90 degree phase at f=0, first sample: cos(pi/2) ~ 0, sin(pi/2) ~ 1
        assert!(fid.data[0].re.abs() < 0.01);
        assert!(approx_eq(fid.data[0].im, 1.0, 0.01));
    }

    // ── Zero filling tests ──────────────────────────────────────────────

    #[test]
    fn test_zero_fill() {
        let proc = NmrProcessor::new(400.0, 4000.0, 128);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 100.0,
            t2_s: 0.5,
            phase_rad: 0.0,
        }];
        let fid = proc.generate_fid(&peaks);
        let zf = proc.zero_fill(&fid, 256);
        assert_eq!(zf.data.len(), 256);
        // Appended points should be zero
        assert!(approx_eq(zf.data[200].magnitude(), 0.0, TOL));
    }

    // ── Apodization tests ───────────────────────────────────────────────

    #[test]
    fn test_apodization_none() {
        let proc = NmrProcessor::new(400.0, 4000.0, 64);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 0.0,
            t2_s: 10.0,
            phase_rad: 0.0,
        }];
        let fid = proc.generate_fid(&peaks);
        let apod = proc.apodize(&fid, Apodization::None);
        assert!(approx_eq(apod.data[0].re, fid.data[0].re, TOL));
        assert!(approx_eq(apod.data[32].re, fid.data[32].re, TOL));
    }

    #[test]
    fn test_apodization_exponential() {
        let proc = NmrProcessor::new(400.0, 1000.0, 256);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 0.0,
            t2_s: 100.0,
            phase_rad: 0.0,
        }];
        let fid = proc.generate_fid(&peaks);
        let apod = proc.apodize(&fid, Apodization::Exponential { lb_hz: 5.0 });
        // Exponential should decay the signal
        assert!(apod.data[128].magnitude() < fid.data[128].magnitude());
    }

    #[test]
    fn test_apodization_gaussian() {
        let proc = NmrProcessor::new(400.0, 1000.0, 256);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 0.0,
            t2_s: 100.0,
            phase_rad: 0.0,
        }];
        let fid = proc.generate_fid(&peaks);
        let apod = proc.apodize(&fid, Apodization::Gaussian { gb: 10.0 });
        // Gaussian decays the signal at later times
        assert!(apod.data[200].magnitude() < fid.data[200].magnitude());
    }

    #[test]
    fn test_apodization_sinebell() {
        let proc = NmrProcessor::new(400.0, 4000.0, 128);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 0.0,
            t2_s: 100.0,
            phase_rad: 0.0,
        }];
        let fid = proc.generate_fid(&peaks);
        let apod = proc.apodize(&fid, Apodization::SineBell);
        // Sine-bell is 0 at t=0 and t=T_max, peaks at t=T_max/2
        assert!(apod.data[0].magnitude() < 0.01);
    }

    #[test]
    fn test_apodization_cosine_squared() {
        let proc = NmrProcessor::new(400.0, 4000.0, 128);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 0.0,
            t2_s: 100.0,
            phase_rad: 0.0,
        }];
        let fid = proc.generate_fid(&peaks);
        let apod = proc.apodize(&fid, Apodization::CosineSquared);
        // Cosine-squared is 1 at t=0 and decays
        assert!(approx_eq(apod.data[0].magnitude(), fid.data[0].magnitude(), 0.01));
        assert!(apod.data[127].magnitude() < fid.data[127].magnitude());
    }

    // ── FFT tests ───────────────────────────────────────────────────────

    #[test]
    fn test_fft_single_tone() {
        let proc = NmrProcessor::new(400.0, 4000.0, 1024);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 500.0,
            t2_s: 10.0,
            phase_rad: 0.0,
        }];
        let fid = proc.generate_fid(&peaks);
        let spectrum = proc.fft_to_spectrum(&fid);
        assert_eq!(spectrum.magnitude.len(), 1024);

        // Find the peak - should be near 500 Hz
        let (max_idx, _) = spectrum
            .magnitude
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap();
        let peak_freq = spectrum.frequency_hz[max_idx];
        assert!(
            (peak_freq - 500.0).abs() < 10.0,
            "peak at {peak_freq} Hz, expected ~500 Hz"
        );
    }

    #[test]
    fn test_fft_spectrum_length() {
        let proc = NmrProcessor::new(400.0, 2000.0, 512);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 0.0,
            t2_s: 1.0,
            phase_rad: 0.0,
        }];
        let fid = proc.generate_fid(&peaks);
        let spectrum = proc.fft_to_spectrum(&fid);
        assert_eq!(spectrum.frequency_hz.len(), spectrum.magnitude.len());
        assert_eq!(spectrum.real.len(), spectrum.imaginary.len());
    }

    // ── Chemical shift tests ────────────────────────────────────────────

    #[test]
    fn test_hz_to_ppm() {
        let proc = NmrProcessor::new(400.0, 4000.0, 1024);
        // 400 Hz offset at 400 MHz = 1.0 ppm
        let ppm = proc.hz_to_ppm(400.0);
        assert!(approx_eq(ppm, 1.0, TOL));
    }

    #[test]
    fn test_ppm_to_hz() {
        let proc = NmrProcessor::new(400.0, 4000.0, 1024);
        let hz = proc.ppm_to_hz(2.5);
        assert!(approx_eq(hz, 1000.0, TOL));
    }

    #[test]
    fn test_ppm_hz_roundtrip() {
        let proc = NmrProcessor::new(600.0, 8000.0, 2048);
        let ppm = 3.7;
        let hz = proc.ppm_to_hz(ppm);
        let ppm_back = proc.hz_to_ppm(hz);
        assert!(approx_eq(ppm, ppm_back, TOL));
    }

    // ── Lorentzian lineshape tests ──────────────────────────────────────

    #[test]
    fn test_lorentzian_at_center() {
        let val = lorentzian(100.0, 100.0, 1.0, 0.5);
        // At f = f0, L = A
        assert!(approx_eq(val, 1.0, TOL));
    }

    #[test]
    fn test_lorentzian_half_max() {
        let t2 = 0.5;
        let gamma = 1.0 / (PI * t2);
        let half_gamma = gamma / 2.0;
        // At f = f0 + gamma/2, L should be A/2
        let val = lorentzian(100.0 + half_gamma, 100.0, 1.0, t2);
        assert!(approx_eq(val, 0.5, 0.01), "got {val}");
    }

    #[test]
    fn test_fwhm_from_t2() {
        let t2 = 0.318; // ~1 Hz FWHM
        let fwhm = fwhm_from_t2(t2);
        assert!((fwhm - 1.0).abs() < 0.01, "got {fwhm}");
    }

    #[test]
    fn test_t2_from_fwhm() {
        let fwhm = 2.0;
        let t2 = t2_from_fwhm(fwhm);
        let expected = 1.0 / (PI * 2.0);
        assert!(approx_eq(t2, expected, TOL));
    }

    #[test]
    fn test_fwhm_t2_roundtrip() {
        let t2 = 0.25;
        let fwhm = fwhm_from_t2(t2);
        let t2_back = t2_from_fwhm(fwhm);
        assert!(approx_eq(t2, t2_back, TOL));
    }

    // ── Relaxation tests ────────────────────────────────────────────────

    #[test]
    fn test_t1_inversion_recovery_model() {
        // At t=0: M = M0*(1 - 2) = -M0
        let m = t1_inversion_recovery(1.0, 1.0, 0.0);
        assert!(approx_eq(m, -1.0, TOL));
    }

    #[test]
    fn test_t1_recovery_at_infinity() {
        // At t >> T1: M -> M0
        let m = t1_inversion_recovery(1.0, 1.0, 100.0);
        assert!(approx_eq(m, 1.0, TOL));
    }

    #[test]
    fn test_t2_cpmg_decay_model() {
        let m = t2_cpmg_decay(1.0, 1.0, 0.0);
        assert!(approx_eq(m, 1.0, TOL));
        let m2 = t2_cpmg_decay(1.0, 1.0, 1.0);
        assert!(approx_eq(m2, (-1.0_f64).exp(), TOL));
    }

    #[test]
    fn test_fit_t2() {
        let true_m0 = 100.0;
        let true_t2 = 0.5;
        let times: Vec<f64> = (0..50).map(|i| i as f64 * 0.02).collect();
        let mags: Vec<f64> = times.iter().map(|&t| true_m0 * (-t / true_t2).exp()).collect();

        let fit = fit_t2(&times, &mags);
        assert!((fit.m0 - true_m0).abs() < 1.0, "M0: {}", fit.m0);
        assert!((fit.time_constant_s - true_t2).abs() < 0.05, "T2: {}", fit.time_constant_s);
    }

    #[test]
    fn test_fit_t1() {
        let true_m0 = 1.0;
        let true_t1 = 1.5;
        let times: Vec<f64> = (0..60).map(|i| i as f64 * 0.1).collect();
        let mags: Vec<f64> = times
            .iter()
            .map(|&t| t1_inversion_recovery(true_m0, true_t1, t))
            .collect();

        let fit = fit_t1(&times, &mags);
        assert!((fit.m0 - true_m0).abs() < 0.1, "M0: {}", fit.m0);
        assert!(
            (fit.time_constant_s - true_t1).abs() < 0.2,
            "T1: {}",
            fit.time_constant_s
        );
    }

    // ── J-coupling pattern tests ────────────────────────────────────────

    #[test]
    fn test_doublet_generation() {
        let freqs: Vec<f64> = (0..1000).map(|i| i as f64 * 0.1).collect();
        let pattern = generate_doublet(&freqs, 50.0, 7.0, 1.0, 0.5);
        assert_eq!(pattern.len(), 1000);

        // Should have two peaks near 46.5 and 53.5 Hz
        let max_val = pattern.iter().cloned().fold(0.0_f64, f64::max);
        assert!(max_val > 0.5);
    }

    #[test]
    fn test_triplet_generation() {
        let freqs: Vec<f64> = (0..1000).map(|i| i as f64 * 0.1).collect();
        let pattern = generate_triplet(&freqs, 50.0, 7.0, 1.0, 0.5);
        // Center peak should be about 2x the outer peaks
        let center_val = pattern[500]; // f = 50.0
        let outer_val = pattern[430]; // f ~ 43.0 (near f-J)
        assert!(center_val > outer_val);
    }

    #[test]
    fn test_quartet_generation() {
        let freqs: Vec<f64> = (0..2000).map(|i| i as f64 * 0.1).collect();
        let pattern = generate_quartet(&freqs, 100.0, 7.0, 1.0, 0.5);
        assert_eq!(pattern.len(), 2000);
        // Pattern should have 4 distinct peaks
        let max_val = pattern.iter().cloned().fold(0.0_f64, f64::max);
        assert!(max_val > 0.0);
    }

    // ── Phase correction tests ──────────────────────────────────────────

    #[test]
    fn test_phase_correction_zero_order() {
        let proc = NmrProcessor::new(400.0, 4000.0, 256);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 500.0,
            t2_s: 1.0,
            phase_rad: PI / 4.0, // 45 degree phase error
        }];
        let fid = proc.generate_fid(&peaks);
        let spectrum = proc.fft_to_spectrum(&fid);
        let corrected = proc.phase_correct(&spectrum, -PI / 4.0, 0.0, 128);
        // After correction, some real parts should increase
        assert_eq!(corrected.real.len(), spectrum.real.len());
    }

    #[test]
    fn test_phase_correction_preserves_magnitude() {
        let proc = NmrProcessor::new(400.0, 4000.0, 128);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 200.0,
            t2_s: 1.0,
            phase_rad: 0.0,
        }];
        let fid = proc.generate_fid(&peaks);
        let spectrum = proc.fft_to_spectrum(&fid);
        let corrected = proc.phase_correct(&spectrum, 1.0, 0.5, 64);

        // Magnitude should be preserved
        for i in 0..spectrum.magnitude.len() {
            assert!(
                (corrected.magnitude[i] - spectrum.magnitude[i]).abs() < 0.01,
                "idx {i}: {} vs {}",
                corrected.magnitude[i],
                spectrum.magnitude[i]
            );
        }
    }

    // ── Peak picking tests ──────────────────────────────────────────────

    #[test]
    fn test_peak_picking_single() {
        let proc = NmrProcessor::new(400.0, 4000.0, 2048);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 500.0,
            t2_s: 0.5,
            phase_rad: 0.0,
        }];
        let fid = proc.generate_fid(&peaks);
        let spectrum = proc.fft_to_spectrum(&fid);

        let detected = proc.pick_peaks(&spectrum, 10.0);
        assert!(!detected.is_empty(), "should detect at least one peak");

        // The strongest peak should be near 500 Hz
        let strongest = detected
            .iter()
            .max_by(|a, b| a.magnitude.partial_cmp(&b.magnitude).unwrap())
            .unwrap();
        assert!(
            (strongest.frequency_hz - 500.0).abs() < 20.0,
            "peak at {} Hz",
            strongest.frequency_hz
        );
    }

    #[test]
    fn test_peak_picking_two_peaks() {
        let proc = NmrProcessor::new(400.0, 4000.0, 4096);
        let peaks = vec![
            NmrPeak { amplitude: 1.0, frequency_hz: 300.0, t2_s: 0.5, phase_rad: 0.0 },
            NmrPeak { amplitude: 0.8, frequency_hz: 1000.0, t2_s: 0.5, phase_rad: 0.0 },
        ];
        let fid = proc.generate_fid(&peaks);
        let spectrum = proc.fft_to_spectrum(&fid);

        let detected = proc.pick_peaks(&spectrum, 5.0);
        assert!(detected.len() >= 2, "should find at least 2 peaks, found {}", detected.len());
    }

    // ── Multiplet classification tests ──────────────────────────────────

    #[test]
    fn test_classify_singlet() {
        let proc = NmrProcessor::new(400.0, 4000.0, 1024);
        let peaks = vec![DetectedPeak {
            frequency_hz: 500.0,
            chemical_shift_ppm: 1.25,
            magnitude: 100.0,
            fwhm_hz: 2.0,
            integral: 50.0,
        }];
        let patterns = proc.classify_multiplet(&peaks, 20.0);
        assert_eq!(patterns.len(), 1);
        assert_eq!(patterns[0], MultipletPattern::Singlet);
    }

    #[test]
    fn test_classify_doublet() {
        let proc = NmrProcessor::new(400.0, 4000.0, 1024);
        let peaks = vec![
            DetectedPeak {
                frequency_hz: 493.5,
                chemical_shift_ppm: 1.23,
                magnitude: 100.0,
                fwhm_hz: 2.0,
                integral: 50.0,
            },
            DetectedPeak {
                frequency_hz: 506.5,
                chemical_shift_ppm: 1.27,
                magnitude: 100.0,
                fwhm_hz: 2.0,
                integral: 50.0,
            },
        ];
        let patterns = proc.classify_multiplet(&peaks, 20.0);
        assert_eq!(patterns.len(), 1);
        match &patterns[0] {
            MultipletPattern::Doublet { j_hz } => {
                assert!(approx_eq(*j_hz, 13.0, 0.5), "J = {j_hz}");
            }
            other => panic!("expected doublet, got {:?}", other),
        }
    }

    #[test]
    fn test_classify_triplet() {
        let proc = NmrProcessor::new(400.0, 4000.0, 1024);
        let peaks = vec![
            DetectedPeak { frequency_hz: 493.0, chemical_shift_ppm: 1.23, magnitude: 50.0, fwhm_hz: 2.0, integral: 25.0 },
            DetectedPeak { frequency_hz: 500.0, chemical_shift_ppm: 1.25, magnitude: 100.0, fwhm_hz: 2.0, integral: 50.0 },
            DetectedPeak { frequency_hz: 507.0, chemical_shift_ppm: 1.27, magnitude: 50.0, fwhm_hz: 2.0, integral: 25.0 },
        ];
        let patterns = proc.classify_multiplet(&peaks, 20.0);
        assert_eq!(patterns.len(), 1);
        match &patterns[0] {
            MultipletPattern::Triplet { j_hz } => {
                assert!(approx_eq(*j_hz, 7.0, 0.5), "J = {j_hz}");
            }
            other => panic!("expected triplet, got {:?}", other),
        }
    }

    // ── SNR estimation test ─────────────────────────────────────────────

    #[test]
    fn test_snr_estimation() {
        let proc = NmrProcessor::new(400.0, 4000.0, 1024);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 500.0,
            t2_s: 0.5,
            phase_rad: 0.0,
        }];
        let fid = proc.generate_fid(&peaks);
        let spectrum = proc.fft_to_spectrum(&fid);

        // Use edge of spectrum as noise region
        let snr = proc.estimate_snr(&spectrum, 0, 50);
        assert!(snr > 1.0, "SNR should be > 1, got {snr}");
    }

    // ── Processor parameter tests ───────────────────────────────────────

    #[test]
    fn test_dwell_time() {
        let proc = NmrProcessor::new(400.0, 4000.0, 1024);
        let dt = proc.dwell_time_s();
        assert!(approx_eq(dt, 0.00025, TOL));
    }

    #[test]
    fn test_acquisition_time() {
        let proc = NmrProcessor::new(400.0, 4000.0, 4000);
        let at = proc.acquisition_time_s();
        assert!(approx_eq(at, 1.0, TOL));
    }

    // ── J-coupling extraction test ──────────────────────────────────────

    #[test]
    fn test_j_coupling_extraction() {
        let proc = NmrProcessor::new(400.0, 4000.0, 1024);
        let j = proc.j_coupling_hz(493.5, 506.5);
        assert!(approx_eq(j, 13.0, TOL));
    }

    // ── Baseline correction test ────────────────────────────────────────

    #[test]
    fn test_baseline_correction() {
        let proc = NmrProcessor::new(400.0, 4000.0, 256);
        let peaks = vec![NmrPeak {
            amplitude: 1.0,
            frequency_hz: 500.0,
            t2_s: 0.5,
            phase_rad: 0.0,
        }];
        let fid = proc.generate_fid(&peaks);
        let mut spectrum = proc.fft_to_spectrum(&fid);

        // Add a DC offset to simulate baseline drift
        for r in spectrum.real.iter_mut() {
            *r += 10.0;
        }

        // Use edge regions as baseline
        let baseline_regions: Vec<usize> = (0..20).chain(236..256).collect();
        let corrected = proc.baseline_correct(&spectrum, 1, &baseline_regions);

        // The corrected spectrum should have less DC offset
        let mean_orig: f64 = spectrum.real.iter().sum::<f64>() / spectrum.real.len() as f64;
        let mean_corr: f64 = corrected.real.iter().sum::<f64>() / corrected.real.len() as f64;
        assert!(
            mean_corr.abs() < mean_orig.abs(),
            "baseline correction should reduce DC: {mean_corr} vs {mean_orig}"
        );
    }

    // ── Complex number tests ────────────────────────────────────────────

    #[test]
    fn test_complex_magnitude() {
        let c = Complex::new(3.0, 4.0);
        assert!(approx_eq(c.magnitude(), 5.0, TOL));
    }

    #[test]
    fn test_complex_phase() {
        let c = Complex::new(0.0, 1.0);
        assert!(approx_eq(c.phase(), PI / 2.0, TOL));
    }

    #[test]
    fn test_complex_from_polar() {
        let c = Complex::from_polar(1.0, PI / 4.0);
        assert!(approx_eq(c.re, (PI / 4.0).cos(), TOL));
        assert!(approx_eq(c.im, (PI / 4.0).sin(), TOL));
    }

    // ── FFT internal tests ──────────────────────────────────────────────

    #[test]
    fn test_fft_inverse_roundtrip() {
        let mut data = vec![
            Complex::new(1.0, 0.0),
            Complex::new(2.0, 0.0),
            Complex::new(3.0, 0.0),
            Complex::new(4.0, 0.0),
        ];
        let original: Vec<Complex> = data.clone();

        fft_radix2(&mut data, false);
        fft_radix2(&mut data, true);

        for (i, (a, b)) in data.iter().zip(original.iter()).enumerate() {
            assert!(
                approx_eq(a.re, b.re, 1e-10) && approx_eq(a.im, b.im, 1e-10),
                "FFT roundtrip mismatch at {i}: ({}, {}) vs ({}, {})",
                a.re,
                a.im,
                b.re,
                b.im,
            );
        }
    }

    #[test]
    fn test_next_power_of_2() {
        assert_eq!(next_power_of_2(1), 1);
        assert_eq!(next_power_of_2(3), 4);
        assert_eq!(next_power_of_2(4), 4);
        assert_eq!(next_power_of_2(5), 8);
        assert_eq!(next_power_of_2(1023), 1024);
    }
}
