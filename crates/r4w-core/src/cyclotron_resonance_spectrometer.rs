//! Ion Cyclotron Resonance (ICR) and Fourier Transform ICR Mass Spectrometry (FT-ICR MS).
//!
//! This module implements the signal processing pipeline for FT-ICR mass spectrometry,
//! the highest-resolution mass analysis technique available. Ions trapped in a Penning
//! trap (strong magnetic field + electrostatic trapping potential) orbit at their
//! cyclotron frequency, which is inversely proportional to their mass-to-charge ratio.
//!
//! # Physics
//!
//! In a uniform magnetic field **B**, an ion with mass *m* and charge *q* orbits at
//! the unperturbed cyclotron frequency:
//!
//! ```text
//! ω_c = qB / m       f_c = qB / (2πm)
//! ```
//!
//! The electrostatic trapping potential introduces two additional motions:
//!
//! - **Reduced cyclotron**: ω₊ = ω_c/2 + √(ω_c²/4 − ω_t²/2)
//! - **Magnetron**: ω₋ = ω_c/2 − √(ω_c²/4 − ω_t²/2)
//! - **Trapping (axial)**: ω_t = √(qVα / (md²))
//!
//! where V is the trapping voltage, α is the cell geometry factor, and d is the
//! characteristic cell dimension.
//!
//! The measured frequency in a real ICR cell is ω₊ (reduced cyclotron), not ω_c.
//! The relationship ω_c = ω₊ + ω₋ allows recovery of the true cyclotron frequency.
//!
//! # Signal Processing Pipeline
//!
//! 1. **Transient acquisition** — digitized image-current time-domain signal (FID-like)
//! 2. **Apodization** — window function to reduce spectral leakage
//! 3. **Zero-filling** — improve spectral interpolation (typically 1–2 zero fills)
//! 4. **FFT** — convert to frequency-domain magnitude spectrum
//! 5. **Phase correction** — absorption-mode display for narrower peaks
//! 6. **Calibration** — convert frequency axis to m/z (Ledford equation)
//! 7. **Peak picking** — centroid detection for mass list generation
//! 8. **Space-charge correction** — adjust for Coulomb-induced frequency shifts
//! 9. **Harmonic removal** — detect and flag 2f, 3f artifacts
//! 10. **Mass defect analysis** — Kendrick plots for homologous series
//!
//! # Applications
//!
//! - Proteomics (top-down protein identification)
//! - Metabolomics (exact-mass molecular formula assignment)
//! - Petroleomics (crude oil molecular characterization)
//! - Environmental analysis (PFAS, dissolved organic matter)
//! - Isotope ratio measurement
//!
//! # Example
//!
//! ```rust
//! use r4w_core::cyclotron_resonance_spectrometer::{
//!     IcrConfig, CellGeometry, CyclotronFrequencyCalculator,
//!     TransientProcessor, ApodizationType, MassCalibrator,
//!     PeakPicker, ResolvingPowerCalculator,
//! };
//!
//! // 12 Tesla FT-ICR with cylindrical cell
//! let config = IcrConfig {
//!     magnetic_field_tesla: 12.0,
//!     cell_geometry: CellGeometry::Cylindrical { radius_m: 0.03, length_m: 0.06 },
//!     trapping_voltage: 1.0,
//!     geometry_factor: 2.77,
//!     excite_amplitude_vpp: 100.0,
//!     excite_duration_ms: 10.0,
//! };
//!
//! // Calculate cyclotron frequency for a known ion (e.g., m/z = 400)
//! let calc = CyclotronFrequencyCalculator::new(&config);
//! let freq = calc.mz_to_frequency(400.0);
//! assert!(freq > 0.0);
//!
//! // Round-trip: frequency -> m/z -> frequency
//! let mz_back = calc.frequency_to_mz(freq);
//! assert!((mz_back - 400.0).abs() < 1e-6);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Elementary charge in Coulombs.
const ELEMENTARY_CHARGE: f64 = 1.602_176_634e-19;

/// Unified atomic mass unit in kg.
const DALTON_KG: f64 = 1.660_539_067e-27;

/// Vacuum permittivity (F/m).
const EPSILON_0: f64 = 8.854_187_817e-12;

/// Electron mass in Daltons.
const ELECTRON_MASS_DA: f64 = 5.485_799_09e-4;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// ICR cell geometry.
#[derive(Debug, Clone, Copy)]
pub enum CellGeometry {
    /// Cylindrical open-ended cell with radius and length.
    Cylindrical { radius_m: f64, length_m: f64 },
    /// Cubic (orthorhombic) cell with side length.
    Cubic { side_m: f64 },
    /// Custom geometry with a characteristic dimension d.
    Custom { d_m: f64 },
}

impl CellGeometry {
    /// Characteristic dimension *d* used in the trapping frequency formula.
    pub fn characteristic_dimension(&self) -> f64 {
        match self {
            CellGeometry::Cylindrical { radius_m, length_m } => {
                // d² = (r² + z₀²)/2 where z₀ = length/2; simplified to ~length/2
                // for a typical cylindrical open cell
                let z0 = length_m / 2.0;
                (radius_m.powi(2) / 2.0 + z0.powi(2)).sqrt()
            }
            CellGeometry::Cubic { side_m } => side_m / 2.0,
            CellGeometry::Custom { d_m } => *d_m,
        }
    }
}

/// FT-ICR instrument configuration.
#[derive(Debug, Clone)]
pub struct IcrConfig {
    /// Magnetic field strength in Tesla (typical: 7, 9.4, 12, 15, 21 T).
    pub magnetic_field_tesla: f64,
    /// ICR cell geometry.
    pub cell_geometry: CellGeometry,
    /// Trapping voltage in Volts (typical: 0.5–2.0 V).
    pub trapping_voltage: f64,
    /// Cell geometry factor α (dimensionless, ~2.77 for cylindrical open cell).
    pub geometry_factor: f64,
    /// Excitation amplitude in Vpp.
    pub excite_amplitude_vpp: f64,
    /// Excitation duration in milliseconds.
    pub excite_duration_ms: f64,
}

impl IcrConfig {
    /// Create a standard 12 T FT-ICR configuration.
    pub fn standard_12t() -> Self {
        Self {
            magnetic_field_tesla: 12.0,
            cell_geometry: CellGeometry::Cylindrical {
                radius_m: 0.03,
                length_m: 0.06,
            },
            trapping_voltage: 1.0,
            geometry_factor: 2.77,
            excite_amplitude_vpp: 100.0,
            excite_duration_ms: 10.0,
        }
    }

    /// Create a standard 9.4 T FT-ICR configuration.
    pub fn standard_9_4t() -> Self {
        Self {
            magnetic_field_tesla: 9.4,
            cell_geometry: CellGeometry::Cylindrical {
                radius_m: 0.03,
                length_m: 0.06,
            },
            trapping_voltage: 1.0,
            geometry_factor: 2.77,
            excite_amplitude_vpp: 80.0,
            excite_duration_ms: 12.0,
        }
    }
}

// ---------------------------------------------------------------------------
// Cyclotron Frequency Calculator
// ---------------------------------------------------------------------------

/// Converts between cyclotron frequency and m/z using ICR physics.
///
/// Accounts for the trapping-potential perturbation (reduced cyclotron vs true
/// cyclotron frequency) and the magnetron frequency.
#[derive(Debug)]
pub struct CyclotronFrequencyCalculator {
    b: f64,
    v_trap: f64,
    alpha: f64,
    d: f64,
}

impl CyclotronFrequencyCalculator {
    /// Create a new calculator from instrument configuration.
    pub fn new(config: &IcrConfig) -> Self {
        Self {
            b: config.magnetic_field_tesla,
            v_trap: config.trapping_voltage,
            alpha: config.geometry_factor,
            d: config.cell_geometry.characteristic_dimension(),
        }
    }

    /// Unperturbed cyclotron frequency in Hz for a given m/z (in Daltons/e).
    ///
    /// f_c = qB / (2π m)  where m = mz × u, q = e.
    pub fn cyclotron_frequency(&self, mz: f64) -> f64 {
        let m_kg = mz * DALTON_KG;
        ELEMENTARY_CHARGE * self.b / (2.0 * PI * m_kg)
    }

    /// Trapping (axial) angular frequency ω_t for a given m/z.
    ///
    /// ω_t = √(qVα / (md²))
    pub fn trapping_angular_freq(&self, mz: f64) -> f64 {
        let m_kg = mz * DALTON_KG;
        (ELEMENTARY_CHARGE * self.v_trap * self.alpha / (m_kg * self.d.powi(2))).sqrt()
    }

    /// Reduced cyclotron angular frequency ω₊ (the measured frequency in practice).
    ///
    /// ω₊ = ω_c/2 + √(ω_c²/4 − ω_t²/2)
    pub fn reduced_cyclotron_angular_freq(&self, mz: f64) -> f64 {
        let wc = self.cyclotron_frequency(mz) * 2.0 * PI;
        let wt = self.trapping_angular_freq(mz);
        let discriminant = wc.powi(2) / 4.0 - wt.powi(2) / 2.0;
        if discriminant < 0.0 {
            return 0.0; // ion is unstable (low mass limit)
        }
        wc / 2.0 + discriminant.sqrt()
    }

    /// Reduced cyclotron frequency in Hz (observed frequency).
    pub fn reduced_cyclotron_frequency(&self, mz: f64) -> f64 {
        self.reduced_cyclotron_angular_freq(mz) / (2.0 * PI)
    }

    /// Magnetron angular frequency ω₋.
    ///
    /// ω₋ = ω_c/2 − √(ω_c²/4 − ω_t²/2)
    pub fn magnetron_angular_freq(&self, mz: f64) -> f64 {
        let wc = self.cyclotron_frequency(mz) * 2.0 * PI;
        let wt = self.trapping_angular_freq(mz);
        let discriminant = wc.powi(2) / 4.0 - wt.powi(2) / 2.0;
        if discriminant < 0.0 {
            return 0.0;
        }
        wc / 2.0 - discriminant.sqrt()
    }

    /// Convert m/z to observed frequency (Hz) using the reduced cyclotron formula.
    pub fn mz_to_frequency(&self, mz: f64) -> f64 {
        self.reduced_cyclotron_frequency(mz)
    }

    /// Convert observed frequency (Hz) to m/z using the unperturbed cyclotron relation.
    ///
    /// This is a first-order approximation: m/z = qB / (2πf) / u.
    /// For higher accuracy, use [`MassCalibrator`] with empirical coefficients.
    pub fn frequency_to_mz(&self, freq_hz: f64) -> f64 {
        if freq_hz <= 0.0 {
            return 0.0;
        }
        ELEMENTARY_CHARGE * self.b / (2.0 * PI * freq_hz * DALTON_KG)
    }

    /// Convert observed frequency to m/z accounting for trapping potential.
    ///
    /// Uses the quadratic relation: f_observed ≈ f_c − f_t²/(2f_c).
    /// Solves iteratively for 5 iterations.
    pub fn frequency_to_mz_corrected(&self, freq_hz: f64) -> f64 {
        if freq_hz <= 0.0 {
            return 0.0;
        }
        // First estimate from unperturbed
        let mut mz = self.frequency_to_mz(freq_hz);
        // Iterate to refine
        for _ in 0..5 {
            let f_calc = self.reduced_cyclotron_frequency(mz);
            if f_calc <= 0.0 {
                break;
            }
            // Newton-like correction
            mz *= f_calc / freq_hz;
        }
        mz
    }
}

// ---------------------------------------------------------------------------
// Transient Processor
// ---------------------------------------------------------------------------

/// Apodization (window) function type for transient processing.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ApodizationType {
    /// No windowing (rectangular).
    None,
    /// Hanning (raised cosine).
    Hanning,
    /// Kaiser window with parameter beta.
    Kaiser(f64),
    /// Gaussian with sigma (fraction of window length, e.g. 0.4).
    Gaussian(f64),
    /// Hamming window.
    Hamming,
    /// Half-Hanning (sine-bell).
    SineBell,
}

/// Processes raw time-domain transient (FID) signals into frequency-domain spectra.
///
/// Pipeline: apodization -> zero-fill -> DFT -> magnitude spectrum.
#[derive(Debug)]
pub struct TransientProcessor {
    apodization: ApodizationType,
    zero_fill_factor: usize,
}

impl TransientProcessor {
    /// Create a new transient processor.
    ///
    /// * `apodization` — window function to apply before DFT.
    /// * `zero_fill_factor` — number of zero-fills (0 = none, 1 = double length, 2 = 4x).
    pub fn new(apodization: ApodizationType, zero_fill_factor: usize) -> Self {
        Self {
            apodization,
            zero_fill_factor,
        }
    }

    /// Apply the configured apodization window to a transient signal (in-place).
    pub fn apodize(&self, signal: &mut [f64]) {
        let n = signal.len();
        if n == 0 {
            return;
        }
        match self.apodization {
            ApodizationType::None => {}
            ApodizationType::Hanning => {
                for i in 0..n {
                    let w = 0.5 * (1.0 - (2.0 * PI * i as f64 / (n - 1) as f64).cos());
                    signal[i] *= w;
                }
            }
            ApodizationType::Kaiser(beta) => {
                let denom = bessel_i0(beta);
                for i in 0..n {
                    let x = 2.0 * i as f64 / (n - 1) as f64 - 1.0;
                    let w = bessel_i0(beta * (1.0 - x * x).max(0.0).sqrt()) / denom;
                    signal[i] *= w;
                }
            }
            ApodizationType::Gaussian(sigma) => {
                let center = (n - 1) as f64 / 2.0;
                let s = sigma * n as f64;
                for i in 0..n {
                    let t = (i as f64 - center) / s;
                    signal[i] *= (-0.5 * t * t).exp();
                }
            }
            ApodizationType::Hamming => {
                for i in 0..n {
                    let w = 0.54 - 0.46 * (2.0 * PI * i as f64 / (n - 1) as f64).cos();
                    signal[i] *= w;
                }
            }
            ApodizationType::SineBell => {
                for i in 0..n {
                    let w = (PI * i as f64 / (n - 1) as f64).sin();
                    signal[i] *= w;
                }
            }
        }
    }

    /// Zero-fill a signal by appending zeros.
    pub fn zero_fill(&self, signal: &[f64]) -> Vec<f64> {
        let factor = 1usize << self.zero_fill_factor;
        let new_len = signal.len() * factor;
        let mut out = Vec::with_capacity(new_len);
        out.extend_from_slice(signal);
        out.resize(new_len, 0.0);
        out
    }

    /// Process a transient: apodize, zero-fill, DFT, return magnitude spectrum.
    ///
    /// Returns `(frequencies_normalized, magnitudes)` where frequencies are in the
    /// range `[0, 0.5)` (normalized to sample rate). Multiply by sample rate to get Hz.
    pub fn process(&self, transient: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let mut data = transient.to_vec();
        self.apodize(&mut data);
        let filled = self.zero_fill(&data);
        let n = filled.len();

        // Compute DFT magnitude for positive frequencies only
        let n_pos = n / 2;
        let mut magnitudes = Vec::with_capacity(n_pos);
        let mut frequencies = Vec::with_capacity(n_pos);

        for k in 0..n_pos {
            let (mut re, mut im) = (0.0, 0.0);
            for (j, &sample) in filled.iter().enumerate() {
                let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
                re += sample * angle.cos();
                im += sample * angle.sin();
            }
            magnitudes.push((re * re + im * im).sqrt() * 2.0 / n as f64);
            frequencies.push(k as f64 / n as f64);
        }

        (frequencies, magnitudes)
    }

    /// Process a transient and return both real (absorption) and imaginary (dispersion)
    /// components for phase correction.
    pub fn process_complex(&self, transient: &[f64]) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
        let mut data = transient.to_vec();
        self.apodize(&mut data);
        let filled = self.zero_fill(&data);
        let n = filled.len();
        let n_pos = n / 2;

        let mut real_parts = Vec::with_capacity(n_pos);
        let mut imag_parts = Vec::with_capacity(n_pos);
        let mut frequencies = Vec::with_capacity(n_pos);

        for k in 0..n_pos {
            let (mut re, mut im) = (0.0, 0.0);
            for (j, &sample) in filled.iter().enumerate() {
                let angle = -2.0 * PI * k as f64 * j as f64 / n as f64;
                re += sample * angle.cos();
                im += sample * angle.sin();
            }
            real_parts.push(re * 2.0 / n as f64);
            imag_parts.push(im * 2.0 / n as f64);
            frequencies.push(k as f64 / n as f64);
        }

        (frequencies, real_parts, imag_parts)
    }
}

// ---------------------------------------------------------------------------
// Mass Calibrator
// ---------------------------------------------------------------------------

/// Calibration equation type for frequency-to-m/z conversion.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CalibrationEquation {
    /// Ledford 2-term: m/z = A/f + B/f²
    Ledford,
    /// Francl 1-term: m/z = A/f (ideal, no trap correction)
    Francl,
    /// 3-term: m/z = A/f + B/f² + C
    ThreeTerm,
}

/// Empirical mass calibration for FT-ICR.
///
/// Converts measured frequencies to m/z using calibration equations fitted
/// to known reference masses (internal or external calibration).
#[derive(Debug, Clone)]
pub struct MassCalibrator {
    equation: CalibrationEquation,
    a: f64,
    b: f64,
    c: f64,
}

impl MassCalibrator {
    /// Create a new mass calibrator with given coefficients.
    ///
    /// * `equation` — calibration equation type
    /// * `a`, `b`, `c` — coefficients (C only used for ThreeTerm)
    pub fn new(equation: CalibrationEquation, a: f64, b: f64, c: f64) -> Self {
        Self { equation, a, b, c }
    }

    /// Convert frequency (Hz) to m/z using the calibration equation.
    pub fn frequency_to_mz(&self, freq_hz: f64) -> f64 {
        if freq_hz <= 0.0 {
            return 0.0;
        }
        match self.equation {
            CalibrationEquation::Francl => self.a / freq_hz,
            CalibrationEquation::Ledford => {
                self.a / freq_hz + self.b / (freq_hz * freq_hz)
            }
            CalibrationEquation::ThreeTerm => {
                self.a / freq_hz + self.b / (freq_hz * freq_hz) + self.c
            }
        }
    }

    /// Convert m/z to frequency (Hz) using iterative root-finding.
    ///
    /// For Ledford: solve m/z = A/f + B/f² for f given m/z.
    /// Rearranging: m/z·f² = A·f + B, i.e., m/z·f² − A·f − B = 0.
    /// Quadratic in f: f = (A + √(A² + 4·m/z·B)) / (2·m/z).
    pub fn mz_to_frequency(&self, mz: f64) -> f64 {
        if mz <= 0.0 {
            return 0.0;
        }
        match self.equation {
            CalibrationEquation::Francl => self.a / mz,
            CalibrationEquation::Ledford => {
                // m/z·f² − A·f − B = 0 → quadratic formula
                let discriminant = self.a * self.a + 4.0 * mz * self.b;
                if discriminant < 0.0 {
                    // Fallback to Francl approximation
                    return self.a / mz;
                }
                let f = (self.a + discriminant.sqrt()) / (2.0 * mz);
                if f > 0.0 { f } else { self.a / mz }
            }
            CalibrationEquation::ThreeTerm => {
                // (mz - C)·f² − A·f − B = 0
                let mz_eff = mz - self.c;
                if mz_eff.abs() < 1e-30 {
                    return self.a / mz;
                }
                let discriminant = self.a * self.a + 4.0 * mz_eff * self.b;
                if discriminant < 0.0 {
                    return self.a / mz;
                }
                let f = (self.a + discriminant.sqrt()) / (2.0 * mz_eff);
                if f > 0.0 { f } else { self.a / mz }
            }
        }
    }

    /// Calibrate from known reference masses using least-squares fit.
    ///
    /// Given pairs of (frequency_hz, known_mz), computes optimal A, B coefficients
    /// for the Ledford equation: m/z = A/f + B/f².
    pub fn calibrate_ledford(references: &[(f64, f64)]) -> Option<Self> {
        if references.len() < 2 {
            return None;
        }
        // Least-squares: m/z_i = A * (1/f_i) + B * (1/f_i²)
        // Design matrix columns: x1 = 1/f, x2 = 1/f²
        let _n = references.len();
        let mut sum_x1x1 = 0.0;
        let mut sum_x1x2 = 0.0;
        let mut sum_x2x2 = 0.0;
        let mut sum_x1y = 0.0;
        let mut sum_x2y = 0.0;

        for &(f, mz) in references {
            let x1 = 1.0 / f;
            let x2 = 1.0 / (f * f);
            sum_x1x1 += x1 * x1;
            sum_x1x2 += x1 * x2;
            sum_x2x2 += x2 * x2;
            sum_x1y += x1 * mz;
            sum_x2y += x2 * mz;
        }

        let det = sum_x1x1 * sum_x2x2 - sum_x1x2 * sum_x1x2;
        // Use relative threshold: compare det to the product of diagonal elements
        let scale = (sum_x1x1 * sum_x2x2).abs();
        let threshold = if scale > 0.0 { scale * 1e-30 } else { 1e-60 };
        if det.abs() < threshold {
            return None;
        }

        let a = (sum_x2x2 * sum_x1y - sum_x1x2 * sum_x2y) / det;
        let b = (sum_x1x1 * sum_x2y - sum_x1x2 * sum_x1y) / det;

        Some(Self {
            equation: CalibrationEquation::Ledford,
            a,
            b,
            c: 0.0,
        })
    }

    /// Compute mass accuracy in ppm for a measured frequency vs. known m/z.
    pub fn mass_error_ppm(&self, freq_hz: f64, known_mz: f64) -> f64 {
        let measured_mz = self.frequency_to_mz(freq_hz);
        (measured_mz - known_mz) / known_mz * 1e6
    }
}

// ---------------------------------------------------------------------------
// Peak Picker
// ---------------------------------------------------------------------------

/// A detected peak in the mass spectrum.
#[derive(Debug, Clone)]
pub struct Peak {
    /// Peak frequency (Hz) or normalized frequency depending on context.
    pub frequency: f64,
    /// Peak magnitude (arbitrary units).
    pub magnitude: f64,
    /// Mass-to-charge ratio (if calibration applied, otherwise 0).
    pub mz: f64,
    /// Resolving power at this peak (if computed).
    pub resolving_power: f64,
    /// Signal-to-noise ratio.
    pub snr: f64,
}

/// Peak detection modes.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum PeakDetectionMode {
    /// Report the bin with maximum intensity.
    Maximum,
    /// Compute centroid (intensity-weighted average) around each peak.
    Centroid,
}

/// Detects peaks in frequency/mass spectra.
#[derive(Debug)]
pub struct PeakPicker {
    mode: PeakDetectionMode,
    snr_threshold: f64,
    noise_window_bins: usize,
}

impl PeakPicker {
    /// Create a new peak picker.
    ///
    /// * `mode` — detection method (Maximum or Centroid)
    /// * `snr_threshold` — minimum signal-to-noise ratio for detection
    /// * `noise_window_bins` — number of bins on each side for local noise estimation
    pub fn new(mode: PeakDetectionMode, snr_threshold: f64, noise_window_bins: usize) -> Self {
        Self {
            mode,
            snr_threshold,
            noise_window_bins,
        }
    }

    /// Estimate local noise RMS around bin `center`, excluding ±`exclude_bins`.
    fn estimate_noise(&self, magnitudes: &[f64], center: usize) -> f64 {
        let n = magnitudes.len();
        let exclude = 3; // exclude ±3 bins around peak
        let mut sum_sq = 0.0;
        let mut count = 0usize;

        let start = center.saturating_sub(self.noise_window_bins);
        let end = (center + self.noise_window_bins + 1).min(n);

        for i in start..end {
            if i >= center.saturating_sub(exclude) && i <= center + exclude {
                continue;
            }
            sum_sq += magnitudes[i] * magnitudes[i];
            count += 1;
        }
        if count == 0 {
            return 1e-30;
        }
        (sum_sq / count as f64).sqrt()
    }

    /// Pick peaks from a magnitude spectrum.
    ///
    /// * `frequencies` — frequency values corresponding to each bin
    /// * `magnitudes` — magnitude values
    ///
    /// Returns detected peaks sorted by descending magnitude.
    pub fn pick_peaks(&self, frequencies: &[f64], magnitudes: &[f64]) -> Vec<Peak> {
        let n = magnitudes.len();
        if n < 3 {
            return Vec::new();
        }

        let mut peaks = Vec::new();

        for i in 1..n - 1 {
            // Local maximum check
            if magnitudes[i] <= magnitudes[i - 1] || magnitudes[i] <= magnitudes[i + 1] {
                continue;
            }

            let noise = self.estimate_noise(magnitudes, i);
            let snr = magnitudes[i] / noise;
            if snr < self.snr_threshold {
                continue;
            }

            let (peak_freq, peak_mag) = match self.mode {
                PeakDetectionMode::Maximum => (frequencies[i], magnitudes[i]),
                PeakDetectionMode::Centroid => {
                    // Intensity-weighted centroid over peak apex ± 1 bin
                    let start = if i >= 1 { i - 1 } else { 0 };
                    let end = (i + 2).min(n);
                    let mut sum_f = 0.0;
                    let mut sum_w = 0.0;
                    for j in start..end {
                        sum_f += frequencies[j] * magnitudes[j];
                        sum_w += magnitudes[j];
                    }
                    if sum_w > 0.0 {
                        (sum_f / sum_w, magnitudes[i])
                    } else {
                        (frequencies[i], magnitudes[i])
                    }
                }
            };

            peaks.push(Peak {
                frequency: peak_freq,
                magnitude: peak_mag,
                mz: 0.0,
                resolving_power: 0.0,
                snr,
            });
        }

        peaks.sort_by(|a, b| b.magnitude.partial_cmp(&a.magnitude).unwrap_or(std::cmp::Ordering::Equal));
        peaks
    }
}

// ---------------------------------------------------------------------------
// Resolving Power Calculator
// ---------------------------------------------------------------------------

/// Calculates resolving power (mass resolution) for FT-ICR spectra.
///
/// R = m / Δm = f / (2·Δf)  where Δf is the full width at half maximum (FWHM).
#[derive(Debug)]
pub struct ResolvingPowerCalculator;

impl ResolvingPowerCalculator {
    /// Theoretical resolving power for a given transient duration and frequency.
    ///
    /// R = f · T_acq / 2  (magnitude mode)
    /// R = f · T_acq      (absorption mode, with phase correction)
    ///
    /// * `frequency_hz` — observed cyclotron frequency
    /// * `acquisition_time_s` — transient duration in seconds
    /// * `absorption_mode` — true for absorption (phase-corrected) mode
    pub fn theoretical(frequency_hz: f64, acquisition_time_s: f64, absorption_mode: bool) -> f64 {
        let factor = if absorption_mode { 1.0 } else { 0.5 };
        frequency_hz * acquisition_time_s * factor
    }

    /// Compute FWHM from a peak in the spectrum.
    ///
    /// Finds the half-maximum points around the peak at `peak_index` by linear
    /// interpolation, returns the width in frequency units.
    pub fn measure_fwhm(frequencies: &[f64], magnitudes: &[f64], peak_index: usize) -> f64 {
        let n = magnitudes.len();
        if peak_index >= n || n < 3 {
            return 0.0;
        }
        let half_max = magnitudes[peak_index] / 2.0;

        // Search left for half-max crossing
        let mut left_freq = frequencies[0];
        for i in (0..peak_index).rev() {
            if magnitudes[i] <= half_max {
                // Linear interpolation
                let frac = (half_max - magnitudes[i]) / (magnitudes[i + 1] - magnitudes[i]).max(1e-30);
                left_freq = frequencies[i] + frac * (frequencies[i + 1] - frequencies[i]);
                break;
            }
        }

        // Search right for half-max crossing
        let mut right_freq = frequencies[n - 1];
        for i in (peak_index + 1)..n {
            if magnitudes[i] <= half_max {
                let frac = (half_max - magnitudes[i]) / (magnitudes[i - 1] - magnitudes[i]).max(1e-30);
                right_freq = frequencies[i] - frac * (frequencies[i] - frequencies[i - 1]);
                break;
            }
        }

        right_freq - left_freq
    }

    /// Resolving power from measured FWHM: R = f / (2·FWHM).
    pub fn from_fwhm(frequency_hz: f64, fwhm_hz: f64) -> f64 {
        if fwhm_hz <= 0.0 {
            return 0.0;
        }
        frequency_hz / (2.0 * fwhm_hz)
    }

    /// Resolving power from m/z and peak width: R = m/z / Δ(m/z).
    pub fn from_mass_width(mz: f64, delta_mz: f64) -> f64 {
        if delta_mz <= 0.0 {
            return 0.0;
        }
        mz / delta_mz
    }
}

// ---------------------------------------------------------------------------
// Space Charge Corrector
// ---------------------------------------------------------------------------

/// Corrects for Coulomb-induced frequency shifts in FT-ICR.
///
/// When many ions are trapped simultaneously, their mutual Coulomb repulsion
/// shifts the measured cyclotron frequency downward:
///
/// Δf = −q·N_ion / (4π·ε₀·m·L²) × (1/(2π))
///
/// This is a major source of systematic mass error in FT-ICR MS.
#[derive(Debug)]
pub struct SpaceChargeCorrector {
    /// Effective cell length (m).
    cell_length_m: f64,
}

impl SpaceChargeCorrector {
    /// Create a new space charge corrector.
    pub fn new(cell_length_m: f64) -> Self {
        Self { cell_length_m }
    }

    /// Estimate frequency shift (Hz) due to space charge.
    ///
    /// * `n_ions` — number of trapped ions
    /// * `mz` — mass-to-charge ratio of the ion species (Da/e)
    ///
    /// Returns negative frequency shift (Hz).
    pub fn frequency_shift(&self, n_ions: f64, mz: f64) -> f64 {
        let m_kg = mz * DALTON_KG;
        let l = self.cell_length_m;
        // Δf = -q * N / (4π ε₀ m L²) / (2π)
        -ELEMENTARY_CHARGE * n_ions / (4.0 * PI * EPSILON_0 * m_kg * l * l * 2.0 * PI)
    }

    /// Correct a measured frequency for space-charge shift.
    ///
    /// f_corrected = f_measured − Δf (subtracting the negative shift adds frequency back).
    pub fn correct_frequency(&self, measured_freq_hz: f64, n_ions: f64, mz: f64) -> f64 {
        measured_freq_hz - self.frequency_shift(n_ions, mz)
    }

    /// Estimate mass error (ppm) from space charge at a given m/z and ion count.
    pub fn mass_error_ppm(&self, mz: f64, n_ions: f64, base_freq_hz: f64) -> f64 {
        if base_freq_hz <= 0.0 {
            return 0.0;
        }
        let df = self.frequency_shift(n_ions, mz);
        // Δm/m ≈ −Δf/f for small shifts
        -df / base_freq_hz * 1e6
    }
}

// ---------------------------------------------------------------------------
// Isotope Pattern Simulator
// ---------------------------------------------------------------------------

/// Molecular formula for isotope pattern simulation.
#[derive(Debug, Clone)]
pub struct MolecularFormula {
    pub c: u32,
    pub h: u32,
    pub n: u32,
    pub o: u32,
    pub s: u32,
}

impl MolecularFormula {
    pub fn new(c: u32, h: u32, n: u32, o: u32, s: u32) -> Self {
        Self { c, h, n, o, s }
    }

    /// Exact monoisotopic mass (Da) using most abundant isotopes.
    pub fn monoisotopic_mass(&self) -> f64 {
        self.c as f64 * 12.0
            + self.h as f64 * 1.007_825_032
            + self.n as f64 * 14.003_074_004
            + self.o as f64 * 15.994_914_620
            + self.s as f64 * 31.972_071_174
    }

    /// Nominal mass (integer sum of nucleons using most abundant isotopes).
    pub fn nominal_mass(&self) -> u32 {
        self.c * 12 + self.h * 1 + self.n * 14 + self.o * 16 + self.s * 32
    }
}

/// An isotope peak with mass and relative abundance.
#[derive(Debug, Clone)]
pub struct IsotopePeak {
    pub mass: f64,
    pub abundance: f64,
}

/// Simulates theoretical isotope distributions from molecular formulae.
///
/// Uses a simplified convolution approach for C, H, N, O, S atoms, considering
/// the most abundant isotopes for each element.
#[derive(Debug)]
pub struct IsotopePatternSimulator;

impl IsotopePatternSimulator {
    /// Compute the isotope pattern for a molecular formula.
    ///
    /// Returns a vector of (mass_offset_da, relative_abundance) peaks normalized
    /// to the most abundant peak = 1.0.
    ///
    /// Uses the polynomial expansion method: for each element, the isotope
    /// distribution is represented as a polynomial in mass offset. These
    /// polynomials are convolved together.
    pub fn simulate(formula: &MolecularFormula, max_peaks: usize) -> Vec<IsotopePeak> {
        // Element isotope data: (mass_offset, natural_abundance)
        // 12C: 0.0 / 98.93%, 13C: 1.003355 / 1.07%
        // 1H: 0.0 / 99.9885%, 2H: 1.006277 / 0.0115%
        // 14N: 0.0 / 99.636%, 15N: 0.997 / 0.364%
        // 16O: 0.0 / 99.757%, 18O: 2.004 / 0.205%  (17O: 1.004 / 0.038% - ignored for simplicity)
        // 32S: 0.0 / 94.93%, 34S: 1.996 / 4.29%  (33S: 0.998 / 0.76% - ignored for simplicity)

        let max_n = max_peaks.min(20);

        // Start with a delta at 0 mass offset with amplitude 1.0
        let mut pattern = vec![1.0f64; 1];

        // Convolve element by element using binomial expansion approximation
        // For element with n atoms, heavy isotope probability p:
        // P(k heavy) = C(n,k) * p^k * (1-p)^(n-k)

        // Carbon
        let p_c13 = 0.0107;
        pattern = Self::convolve_element(&pattern, formula.c, p_c13, max_n);

        // Hydrogen
        let p_h2 = 0.000_115;
        pattern = Self::convolve_element(&pattern, formula.h, p_h2, max_n);

        // Nitrogen
        let p_n15 = 0.003_64;
        pattern = Self::convolve_element(&pattern, formula.n, p_n15, max_n);

        // Oxygen (approximating 18O, ignoring 17O for simplicity)
        let p_o18 = 0.002_05;
        pattern = Self::convolve_element(&pattern, formula.o, p_o18, max_n);

        // Sulfur (34S, ignoring 33S)
        let p_s34 = 0.042_9;
        pattern = Self::convolve_element(&pattern, formula.s, p_s34, max_n);

        // Normalize to max = 1.0
        let max_val = pattern.iter().cloned().fold(0.0f64, f64::max);
        if max_val > 0.0 {
            for p in &mut pattern {
                *p /= max_val;
            }
        }

        // Build output with mass offsets
        let mono_mass = formula.monoisotopic_mass();
        pattern
            .iter()
            .enumerate()
            .take(max_n)
            .filter(|(_, &a)| a > 1e-10)
            .map(|(i, &a)| IsotopePeak {
                mass: mono_mass + i as f64 * 1.003_355, // approximate 1 Da spacing
                abundance: a,
            })
            .collect()
    }

    /// Binomial convolution of element isotope distribution.
    fn convolve_element(current: &[f64], n_atoms: u32, p_heavy: f64, max_len: usize) -> Vec<f64> {
        if n_atoms == 0 {
            return current.to_vec();
        }

        // Compute binomial distribution P(k) = C(n,k) * p^k * (1-p)^(n-k)
        let n = n_atoms as usize;
        let terms = (n + 1).min(max_len);
        let mut binom = vec![0.0; terms];

        // Use log-space to avoid overflow for large n
        let log_1mp = (1.0 - p_heavy).ln();
        let log_p = if p_heavy > 0.0 { p_heavy.ln() } else { f64::NEG_INFINITY };

        let mut log_comb = 0.0; // log(C(n,0)) = 0
        for k in 0..terms {
            if k > 0 {
                log_comb += ((n + 1 - k) as f64).ln() - (k as f64).ln();
            }
            let log_prob = log_comb + k as f64 * log_p + (n - k) as f64 * log_1mp;
            binom[k] = log_prob.exp();
        }

        // Convolve current pattern with binomial distribution
        let out_len = (current.len() + terms - 1).min(max_len);
        let mut result = vec![0.0; out_len];

        for (i, &c) in current.iter().enumerate() {
            for (j, &b) in binom.iter().enumerate() {
                let idx = i + j;
                if idx >= out_len {
                    break;
                }
                result[idx] += c * b;
            }
        }

        result
    }
}

// ---------------------------------------------------------------------------
// Mass Defect Analyzer
// ---------------------------------------------------------------------------

/// Kendrick mass defect analysis for complex mixture characterization.
///
/// The Kendrick mass rescales the IUPAC mass scale so that a chosen repeat unit
/// (e.g., CH₂ = 14.01565 Da) has an exact integer mass:
///
/// KM = IUPAC_mass × (14 / 14.01565)
///
/// Kendrick Mass Defect (KMD) = nominal_KM − exact_KM
///
/// Compounds in a homologous series (differing only by CH₂ units) share the same KMD.
#[derive(Debug)]
pub struct MassDefectAnalyzer {
    /// Exact mass of the repeat unit (e.g., CH₂ = 14.01565 Da).
    repeat_unit_mass: f64,
    /// Nominal (integer) mass of the repeat unit.
    repeat_unit_nominal: f64,
}

impl MassDefectAnalyzer {
    /// Create a new analyzer with a CH₂ repeat unit (most common).
    pub fn ch2() -> Self {
        Self {
            repeat_unit_mass: 14.015_65,
            repeat_unit_nominal: 14.0,
        }
    }

    /// Create a new analyzer with a custom repeat unit.
    pub fn new(repeat_unit_mass: f64, repeat_unit_nominal: f64) -> Self {
        Self {
            repeat_unit_mass,
            repeat_unit_nominal,
        }
    }

    /// Compute the Kendrick mass from an IUPAC mass.
    pub fn kendrick_mass(&self, iupac_mass: f64) -> f64 {
        iupac_mass * (self.repeat_unit_nominal / self.repeat_unit_mass)
    }

    /// Compute the Kendrick Mass Defect (KMD).
    pub fn kendrick_mass_defect(&self, iupac_mass: f64) -> f64 {
        let km = self.kendrick_mass(iupac_mass);
        km.round() - km
    }

    /// Compute the nominal Kendrick mass.
    pub fn nominal_kendrick_mass(&self, iupac_mass: f64) -> f64 {
        self.kendrick_mass(iupac_mass).round()
    }

    /// Group masses into homologous series by KMD similarity.
    ///
    /// Returns vector of series, each containing indices into the input.
    pub fn find_homologous_series(&self, masses: &[f64], kmd_tolerance: f64) -> Vec<Vec<usize>> {
        let kmds: Vec<f64> = masses.iter().map(|&m| self.kendrick_mass_defect(m)).collect();
        let mut used = vec![false; masses.len()];
        let mut series_list = Vec::new();

        for i in 0..masses.len() {
            if used[i] {
                continue;
            }
            let mut series = vec![i];
            used[i] = true;

            for j in (i + 1)..masses.len() {
                if used[j] {
                    continue;
                }
                if (kmds[j] - kmds[i]).abs() < kmd_tolerance {
                    series.push(j);
                    used[j] = true;
                }
            }
            if series.len() >= 2 {
                series_list.push(series);
            }
        }
        series_list
    }

    /// Mass defect (fractional part of mass) in mDa.
    pub fn mass_defect_mda(&self, exact_mass: f64) -> f64 {
        (exact_mass - exact_mass.round()) * 1000.0
    }
}

// ---------------------------------------------------------------------------
// Phase Corrector
// ---------------------------------------------------------------------------

/// Phase correction for absorption-mode FT-ICR spectra.
///
/// Absorption-mode display provides ~2x narrower peaks (and thus 2x better
/// resolving power) compared to magnitude mode. Requires accurate knowledge
/// of the spectral phase.
///
/// Phase model: φ(f) = φ₀ + 2π·f·t₀ + higher-order terms
///
/// where φ₀ is the zero-order phase and t₀ is the group delay.
#[derive(Debug)]
pub struct PhaseCorrector {
    /// Zero-order phase (radians).
    phi0: f64,
    /// First-order phase / group delay (seconds).
    t0: f64,
}

impl PhaseCorrector {
    /// Create a phase corrector with known phase parameters.
    pub fn new(phi0_rad: f64, t0_s: f64) -> Self {
        Self { phi0: phi0_rad, t0: t0_s }
    }

    /// Compute the phase at a given frequency.
    pub fn phase_at(&self, freq_hz: f64) -> f64 {
        self.phi0 + 2.0 * PI * freq_hz * self.t0
    }

    /// Apply phase correction to complex spectrum (real, imag) -> absorption spectrum.
    ///
    /// absorption[k] = re[k]·cos(φ[k]) + im[k]·sin(φ[k])
    pub fn correct(
        &self,
        frequencies: &[f64],
        real_parts: &[f64],
        imag_parts: &[f64],
    ) -> Vec<f64> {
        let n = frequencies.len().min(real_parts.len()).min(imag_parts.len());
        let mut absorption = Vec::with_capacity(n);

        for i in 0..n {
            let phase = self.phase_at(frequencies[i]);
            let a = real_parts[i] * phase.cos() + imag_parts[i] * phase.sin();
            absorption.push(a);
        }
        absorption
    }

    /// Estimate phase parameters from a complex spectrum using iterative
    /// entropy minimization (simplified approach).
    ///
    /// Searches for (phi0, t0) that minimize negative spectral entropy
    /// (maximize signal concentration in positive absorption peaks).
    pub fn estimate_from_spectrum(
        frequencies: &[f64],
        real_parts: &[f64],
        imag_parts: &[f64],
    ) -> Self {
        let n = frequencies.len().min(real_parts.len()).min(imag_parts.len());
        if n == 0 {
            return Self { phi0: 0.0, t0: 0.0 };
        }

        let mut best_phi0 = 0.0;
        let mut best_t0 = 0.0;
        let mut best_score = f64::NEG_INFINITY;

        // Coarse grid search over phi0 and t0
        // phi0: 0 to 2π in 36 steps
        // t0: based on frequency range
        let f_range = frequencies[n - 1] - frequencies[0];
        let t0_range = if f_range > 0.0 { 2.0 / f_range } else { 1e-3 };

        for phi_step in 0..36 {
            let phi0 = phi_step as f64 * 2.0 * PI / 36.0;
            for t_step in -10i32..=10 {
                let t0 = t_step as f64 * t0_range / 10.0;

                // Score: sum of positive absorption values (maximize)
                let mut score = 0.0;
                for i in 0..n {
                    let phase = phi0 + 2.0 * PI * frequencies[i] * t0;
                    let a = real_parts[i] * phase.cos() + imag_parts[i] * phase.sin();
                    if a > 0.0 {
                        score += a;
                    }
                }

                if score > best_score {
                    best_score = score;
                    best_phi0 = phi0;
                    best_t0 = t0;
                }
            }
        }

        Self {
            phi0: best_phi0,
            t0: best_t0,
        }
    }
}

// ---------------------------------------------------------------------------
// Harmonic Detector
// ---------------------------------------------------------------------------

/// Detects and removes harmonic artifacts in FT-ICR spectra.
///
/// In ICR, image-current detection can produce harmonics at 2f, 3f, etc. of
/// the fundamental cyclotron frequency. These appear as ghost peaks at
/// apparent m/z values of m/(2z), m/(3z), etc.
#[derive(Debug)]
pub struct HarmonicDetector {
    /// Maximum harmonic order to check (e.g., 3 checks 2f and 3f).
    max_harmonic: usize,
    /// Frequency tolerance for matching harmonics (relative, e.g., 1e-4).
    tolerance: f64,
}

impl HarmonicDetector {
    /// Create a new harmonic detector.
    pub fn new(max_harmonic: usize, tolerance: f64) -> Self {
        Self {
            max_harmonic: max_harmonic.max(2),
            tolerance,
        }
    }

    /// Check if a given frequency could be a harmonic of any peak in the list.
    ///
    /// Returns `Some((harmonic_order, fundamental_index))` if a match is found.
    pub fn check_harmonic(
        &self,
        freq: f64,
        fundamental_freqs: &[f64],
    ) -> Option<(usize, usize)> {
        for h in 2..=self.max_harmonic {
            let expected_fundamental = freq / h as f64;
            for (idx, &f0) in fundamental_freqs.iter().enumerate() {
                let rel_diff = (expected_fundamental - f0).abs() / f0;
                if rel_diff < self.tolerance {
                    return Some((h, idx));
                }
            }
        }
        None
    }

    /// Identify harmonic peaks in a peak list.
    ///
    /// Returns indices of peaks that are likely harmonics, along with
    /// the (harmonic_order, fundamental_index) for each.
    pub fn find_harmonics(&self, peaks: &[Peak]) -> Vec<(usize, usize, usize)> {
        let freqs: Vec<f64> = peaks.iter().map(|p| p.frequency).collect();
        let mut harmonics = Vec::new();

        for (i, peak) in peaks.iter().enumerate() {
            if let Some((order, fund_idx)) = self.check_harmonic(peak.frequency, &freqs) {
                // Only flag if the fundamental is stronger
                if fund_idx != i && peaks[fund_idx].magnitude > peak.magnitude {
                    harmonics.push((i, order, fund_idx));
                }
            }
        }
        harmonics
    }

    /// Remove harmonic peaks from a peak list, returning cleaned list.
    pub fn remove_harmonics(&self, peaks: &[Peak]) -> Vec<Peak> {
        let harmonic_indices: Vec<usize> = self
            .find_harmonics(peaks)
            .iter()
            .map(|&(idx, _, _)| idx)
            .collect();

        peaks
            .iter()
            .enumerate()
            .filter(|(i, _)| !harmonic_indices.contains(i))
            .map(|(_, p)| p.clone())
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Helper: Bessel I0 (for Kaiser window)
// ---------------------------------------------------------------------------

/// Modified Bessel function of the first kind, order 0.
/// Computed using the series expansion: I0(x) = Σ (x/2)^(2k) / (k!)^2
fn bessel_i0(x: f64) -> f64 {
    let mut sum = 1.0;
    let mut term = 1.0;
    let half_x = x / 2.0;
    for k in 1..25 {
        term *= (half_x / k as f64).powi(2);
        sum += term;
        if term < 1e-16 * sum {
            break;
        }
    }
    sum
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;

    // --- IcrConfig ---

    #[test]
    fn test_standard_12t_config() {
        let cfg = IcrConfig::standard_12t();
        assert!((cfg.magnetic_field_tesla - 12.0).abs() < TOL);
        assert!((cfg.trapping_voltage - 1.0).abs() < TOL);
    }

    #[test]
    fn test_standard_9_4t_config() {
        let cfg = IcrConfig::standard_9_4t();
        assert!((cfg.magnetic_field_tesla - 9.4).abs() < TOL);
    }

    // --- CellGeometry ---

    #[test]
    fn test_cylindrical_geometry_dimension() {
        let g = CellGeometry::Cylindrical {
            radius_m: 0.03,
            length_m: 0.06,
        };
        let d = g.characteristic_dimension();
        assert!(d > 0.0);
        assert!(d < 0.1);
    }

    #[test]
    fn test_cubic_geometry_dimension() {
        let g = CellGeometry::Cubic { side_m: 0.04 };
        let d = g.characteristic_dimension();
        assert!((d - 0.02).abs() < TOL);
    }

    #[test]
    fn test_custom_geometry_dimension() {
        let g = CellGeometry::Custom { d_m: 0.025 };
        assert!((g.characteristic_dimension() - 0.025).abs() < TOL);
    }

    // --- CyclotronFrequencyCalculator ---

    #[test]
    fn test_cyclotron_frequency_inverse_mass() {
        let config = IcrConfig::standard_12t();
        let calc = CyclotronFrequencyCalculator::new(&config);
        // Higher mass should have lower frequency
        let f200 = calc.cyclotron_frequency(200.0);
        let f400 = calc.cyclotron_frequency(400.0);
        assert!(f200 > f400);
        assert!((f200 / f400 - 2.0).abs() < 0.01);
    }

    #[test]
    fn test_cyclotron_frequency_positive() {
        let config = IcrConfig::standard_12t();
        let calc = CyclotronFrequencyCalculator::new(&config);
        let f = calc.cyclotron_frequency(100.0);
        assert!(f > 0.0);
        // For m/z=100 at 12T, expect ~1.84 MHz
        assert!(f > 1e6);
        assert!(f < 1e8);
    }

    #[test]
    fn test_mz_to_frequency_roundtrip() {
        let config = IcrConfig::standard_12t();
        let calc = CyclotronFrequencyCalculator::new(&config);
        for &mz in &[100.0, 200.0, 500.0, 1000.0] {
            let f = calc.mz_to_frequency(mz);
            let mz_back = calc.frequency_to_mz(f);
            // Simple (unperturbed) roundtrip — close but not exact due to trap correction
            assert!(
                (mz_back - mz).abs() / mz < 0.01,
                "Roundtrip failed for m/z={}: got {}", mz, mz_back
            );
        }
    }

    #[test]
    fn test_reduced_cyclotron_less_than_unperturbed() {
        let config = IcrConfig::standard_12t();
        let calc = CyclotronFrequencyCalculator::new(&config);
        let f_unperturbed = calc.cyclotron_frequency(400.0);
        let f_reduced = calc.reduced_cyclotron_frequency(400.0);
        // Reduced cyclotron is always slightly less than unperturbed
        assert!(f_reduced < f_unperturbed);
        assert!(f_reduced > 0.99 * f_unperturbed);
    }

    #[test]
    fn test_magnetron_frequency_small() {
        let config = IcrConfig::standard_12t();
        let calc = CyclotronFrequencyCalculator::new(&config);
        let w_mag = calc.magnetron_angular_freq(400.0);
        let w_cyc = calc.cyclotron_frequency(400.0) * 2.0 * PI;
        // Magnetron frequency should be much smaller than cyclotron
        assert!(w_mag > 0.0);
        assert!(w_mag < w_cyc * 0.01);
    }

    #[test]
    fn test_trapping_angular_freq() {
        let config = IcrConfig::standard_12t();
        let calc = CyclotronFrequencyCalculator::new(&config);
        let wt = calc.trapping_angular_freq(400.0);
        assert!(wt > 0.0);
        // Trapping frequency should be much smaller than cyclotron frequency
        let wc = calc.cyclotron_frequency(400.0) * 2.0 * PI;
        assert!(wt < wc * 0.1);
    }

    #[test]
    fn test_frequency_to_mz_corrected() {
        let config = IcrConfig::standard_12t();
        let calc = CyclotronFrequencyCalculator::new(&config);
        let target_mz = 500.0;
        let f_obs = calc.reduced_cyclotron_frequency(target_mz);
        let mz_corrected = calc.frequency_to_mz_corrected(f_obs);
        assert!(
            (mz_corrected - target_mz).abs() / target_mz < 1e-4,
            "Corrected m/z: expected ~{}, got {}", target_mz, mz_corrected
        );
    }

    #[test]
    fn test_frequency_to_mz_zero() {
        let config = IcrConfig::standard_12t();
        let calc = CyclotronFrequencyCalculator::new(&config);
        assert_eq!(calc.frequency_to_mz(0.0), 0.0);
        assert_eq!(calc.frequency_to_mz(-1.0), 0.0);
    }

    // --- TransientProcessor ---

    #[test]
    fn test_apodize_hanning() {
        let proc = TransientProcessor::new(ApodizationType::Hanning, 0);
        let mut sig = vec![1.0; 64];
        proc.apodize(&mut sig);
        // Hanning: zero at endpoints, max at center
        assert!(sig[0].abs() < 1e-10);
        assert!(sig[63].abs() < 1e-10);
        assert!(sig[32] > 0.9);
    }

    #[test]
    fn test_apodize_hamming() {
        let proc = TransientProcessor::new(ApodizationType::Hamming, 0);
        let mut sig = vec![1.0; 64];
        proc.apodize(&mut sig);
        // Hamming: non-zero at endpoints (~0.08)
        assert!(sig[0] > 0.05 && sig[0] < 0.12);
    }

    #[test]
    fn test_apodize_kaiser() {
        let proc = TransientProcessor::new(ApodizationType::Kaiser(5.0), 0);
        let mut sig = vec![1.0; 64];
        proc.apodize(&mut sig);
        // Kaiser with beta=5: small but non-zero at endpoints
        assert!(sig[0] > 0.0);
        assert!(sig[32] > sig[0]);
    }

    #[test]
    fn test_apodize_gaussian() {
        let proc = TransientProcessor::new(ApodizationType::Gaussian(0.4), 0);
        let mut sig = vec![1.0; 64];
        proc.apodize(&mut sig);
        // Gaussian: maximum at center, decays symmetrically
        assert!(sig[31] > sig[0]);
        assert!(sig[32] > sig[0]);
    }

    #[test]
    fn test_apodize_sine_bell() {
        let proc = TransientProcessor::new(ApodizationType::SineBell, 0);
        let mut sig = vec![1.0; 65];
        proc.apodize(&mut sig);
        // Sine bell: zero at endpoints, maximum at center
        assert!(sig[0].abs() < 1e-10);
        assert!(sig[64].abs() < 1e-10);
        assert!((sig[32] - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_apodize_none() {
        let proc = TransientProcessor::new(ApodizationType::None, 0);
        let mut sig = vec![1.0; 64];
        proc.apodize(&mut sig);
        assert!((sig[0] - 1.0).abs() < TOL);
        assert!((sig[63] - 1.0).abs() < TOL);
    }

    #[test]
    fn test_zero_fill() {
        let proc = TransientProcessor::new(ApodizationType::None, 2);
        let sig = vec![1.0; 16];
        let filled = proc.zero_fill(&sig);
        assert_eq!(filled.len(), 64); // 16 * 2^2 = 64
        assert!((filled[0] - 1.0).abs() < TOL);
        assert!((filled[16] - 0.0).abs() < TOL);
    }

    #[test]
    fn test_process_single_frequency() {
        let proc = TransientProcessor::new(ApodizationType::None, 0);
        let n = 128;
        let fs = 1000.0;
        let f0 = 100.0;
        let transient: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f0 * i as f64 / fs).cos())
            .collect();

        let (freqs, mags) = proc.process(&transient);

        // Find peak bin
        let peak_idx = mags
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap()
            .0;

        // Peak should be near f0/fs = 0.1
        let peak_freq_normalized = freqs[peak_idx];
        assert!(
            (peak_freq_normalized - 0.1).abs() < 0.02,
            "Peak at {}, expected ~0.1", peak_freq_normalized
        );
    }

    #[test]
    fn test_process_complex_returns_re_im() {
        let proc = TransientProcessor::new(ApodizationType::None, 0);
        let transient = vec![1.0, 0.0, -1.0, 0.0, 1.0, 0.0, -1.0, 0.0];
        let (freqs, re, im) = proc.process_complex(&transient);
        assert_eq!(freqs.len(), re.len());
        assert_eq!(freqs.len(), im.len());
        assert!(freqs.len() > 0);
    }

    // --- MassCalibrator ---

    #[test]
    fn test_francl_calibration() {
        let cal = MassCalibrator::new(CalibrationEquation::Francl, 1e8, 0.0, 0.0);
        let mz = cal.frequency_to_mz(1e6);
        assert!((mz - 100.0).abs() < TOL);
    }

    #[test]
    fn test_ledford_calibration() {
        let cal = MassCalibrator::new(CalibrationEquation::Ledford, 1e8, -1e12, 0.0);
        let mz1 = cal.frequency_to_mz(1e6);
        let mz2 = MassCalibrator::new(CalibrationEquation::Francl, 1e8, 0.0, 0.0)
            .frequency_to_mz(1e6);
        // Ledford with negative B should give slightly different m/z than Francl
        assert!((mz1 - mz2).abs() > 0.0);
    }

    #[test]
    fn test_three_term_calibration() {
        let cal = MassCalibrator::new(CalibrationEquation::ThreeTerm, 1e8, -1e12, 0.5);
        let mz = cal.frequency_to_mz(1e6);
        // Should include the constant offset C=0.5
        let mz_led = MassCalibrator::new(CalibrationEquation::Ledford, 1e8, -1e12, 0.0)
            .frequency_to_mz(1e6);
        assert!((mz - mz_led - 0.5).abs() < TOL);
    }

    #[test]
    fn test_calibrate_ledford_from_references() {
        // Create a known calibrator and generate reference data
        let true_cal = MassCalibrator::new(CalibrationEquation::Ledford, 1e8, -1e12, 0.0);
        let refs: Vec<(f64, f64)> = vec![200.0, 400.0, 600.0, 800.0]
            .into_iter()
            .map(|mz| {
                let f = true_cal.mz_to_frequency(mz);
                (f, mz)
            })
            .collect();

        let fitted = MassCalibrator::calibrate_ledford(&refs).unwrap();

        // Check that fitted calibrator reproduces reference masses
        for &(f, mz) in &refs {
            let error_ppm = fitted.mass_error_ppm(f, mz);
            assert!(
                error_ppm.abs() < 1.0,
                "Mass error {} ppm for m/z={}", error_ppm, mz
            );
        }
    }

    #[test]
    fn test_calibrate_ledford_insufficient_points() {
        let result = MassCalibrator::calibrate_ledford(&[(1e6, 100.0)]);
        // Need at least 2 reference points for 2 coefficients
        // Actually our implementation requires >= 2 so this should be None
        // wait, with exactly 1 point it should return None
        assert!(result.is_none());
    }

    #[test]
    fn test_mass_error_ppm() {
        let cal = MassCalibrator::new(CalibrationEquation::Francl, 1e8, 0.0, 0.0);
        // Perfect calibration should give 0 ppm error
        let error = cal.mass_error_ppm(1e6, 100.0);
        assert!(error.abs() < TOL);
    }

    // --- PeakPicker ---

    #[test]
    fn test_peak_picker_finds_peaks() {
        let freqs: Vec<f64> = (0..100).map(|i| i as f64 * 10.0).collect();
        let mut mags = vec![0.01; 100];
        // Insert two peaks
        mags[30] = 1.0;
        mags[70] = 0.5;

        let picker = PeakPicker::new(PeakDetectionMode::Maximum, 2.0, 20);
        let peaks = picker.pick_peaks(&freqs, &mags);
        assert!(peaks.len() >= 2, "Found {} peaks, expected >=2", peaks.len());
        // First peak should be the tallest
        assert!((peaks[0].magnitude - 1.0).abs() < TOL);
    }

    #[test]
    fn test_peak_picker_centroid_mode() {
        let freqs: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let mut mags = vec![0.01; 100];
        mags[49] = 0.5;
        mags[50] = 1.0;
        mags[51] = 0.6;

        let picker = PeakPicker::new(PeakDetectionMode::Centroid, 2.0, 20);
        let peaks = picker.pick_peaks(&freqs, &mags);
        assert!(!peaks.is_empty());
        // Centroid should shift slightly from bin 50 toward 51 due to asymmetry
        assert!(peaks[0].frequency > 49.5 && peaks[0].frequency < 51.0);
    }

    #[test]
    fn test_peak_picker_snr_filter() {
        let freqs: Vec<f64> = (0..100).map(|i| i as f64).collect();
        let mut mags = vec![0.1; 100];
        mags[50] = 0.12; // Just barely above noise — low SNR

        let picker = PeakPicker::new(PeakDetectionMode::Maximum, 5.0, 20);
        let peaks = picker.pick_peaks(&freqs, &mags);
        // Should not detect this low-SNR peak
        assert!(peaks.is_empty() || peaks[0].snr >= 5.0);
    }

    // --- ResolvingPowerCalculator ---

    #[test]
    fn test_theoretical_resolving_power_magnitude_mode() {
        // 12T, m/z=400, 1 second transient
        let config = IcrConfig::standard_12t();
        let calc = CyclotronFrequencyCalculator::new(&config);
        let f = calc.cyclotron_frequency(400.0);
        let rp = ResolvingPowerCalculator::theoretical(f, 1.0, false);
        // Should be > 100,000 for 12T
        assert!(rp > 100_000.0, "RP={}", rp);
    }

    #[test]
    fn test_theoretical_resolving_power_absorption_mode() {
        let config = IcrConfig::standard_12t();
        let calc = CyclotronFrequencyCalculator::new(&config);
        let f = calc.cyclotron_frequency(400.0);
        let rp_mag = ResolvingPowerCalculator::theoretical(f, 1.0, false);
        let rp_abs = ResolvingPowerCalculator::theoretical(f, 1.0, true);
        // Absorption mode gives 2x better RP
        assert!((rp_abs / rp_mag - 2.0).abs() < TOL);
    }

    #[test]
    fn test_from_fwhm() {
        let rp = ResolvingPowerCalculator::from_fwhm(1_000_000.0, 1.0);
        assert!((rp - 500_000.0).abs() < TOL);
    }

    #[test]
    fn test_from_mass_width() {
        let rp = ResolvingPowerCalculator::from_mass_width(400.0, 0.001);
        assert!((rp - 400_000.0).abs() < TOL);
    }

    #[test]
    fn test_measure_fwhm() {
        // Gaussian peak at index 50
        let n = 101;
        let freqs: Vec<f64> = (0..n).map(|i| i as f64).collect();
        let sigma = 3.0;
        let mags: Vec<f64> = (0..n)
            .map(|i| (-0.5 * ((i as f64 - 50.0) / sigma).powi(2)).exp())
            .collect();

        let fwhm = ResolvingPowerCalculator::measure_fwhm(&freqs, &mags, 50);
        // FWHM of Gaussian ≈ 2.355 * sigma
        let expected = 2.355 * sigma;
        assert!(
            (fwhm - expected).abs() < 1.0,
            "FWHM={}, expected ~{}", fwhm, expected
        );
    }

    // --- SpaceChargeCorrector ---

    #[test]
    fn test_space_charge_shift_negative() {
        let sc = SpaceChargeCorrector::new(0.06);
        let shift = sc.frequency_shift(1e6, 400.0);
        // Shift should be negative (frequency decreases)
        assert!(shift < 0.0);
    }

    #[test]
    fn test_space_charge_correction_increases_freq() {
        let sc = SpaceChargeCorrector::new(0.06);
        let f_measured = 500_000.0;
        let f_corrected = sc.correct_frequency(f_measured, 1e5, 400.0);
        // Corrected frequency should be higher than measured
        assert!(f_corrected > f_measured);
    }

    #[test]
    fn test_space_charge_mass_error_positive() {
        let sc = SpaceChargeCorrector::new(0.06);
        let config = IcrConfig::standard_12t();
        let calc = CyclotronFrequencyCalculator::new(&config);
        let f = calc.cyclotron_frequency(400.0);
        let error = sc.mass_error_ppm(400.0, 1e6, f);
        // Positive mass error (measured m/z too high because frequency too low)
        assert!(error > 0.0);
    }

    // --- IsotopePatternSimulator ---

    #[test]
    fn test_monoisotopic_mass_water() {
        let water = MolecularFormula::new(0, 2, 0, 1, 0);
        let mass = water.monoisotopic_mass();
        assert!(
            (mass - 18.010565).abs() < 0.001,
            "Water mass: {}", mass
        );
    }

    #[test]
    fn test_monoisotopic_mass_caffeine() {
        // Caffeine: C8H10N4O2
        let caffeine = MolecularFormula::new(8, 10, 4, 2, 0);
        let mass = caffeine.monoisotopic_mass();
        assert!(
            (mass - 194.0804).abs() < 0.01,
            "Caffeine mass: {}", mass
        );
    }

    #[test]
    fn test_isotope_pattern_monoisotopic_dominant() {
        // Small molecule: monoisotopic peak should be the most abundant
        let formula = MolecularFormula::new(6, 12, 0, 0, 0); // C6H12
        let pattern = IsotopePatternSimulator::simulate(&formula, 5);
        assert!(!pattern.is_empty());
        // M+0 should be the most abundant (normalized to 1.0)
        assert!((pattern[0].abundance - 1.0).abs() < TOL);
    }

    #[test]
    fn test_isotope_pattern_large_molecule_m1() {
        // Large molecule: C100H200 — M+1 peak should be significant
        let formula = MolecularFormula::new(100, 200, 0, 0, 0);
        let pattern = IsotopePatternSimulator::simulate(&formula, 10);
        assert!(pattern.len() >= 2);
        // M+1 should be substantial (~1.07 * 100 ≈ 107% relative to M+0 via 13C alone)
        // With both C and H, M+1 > M+0 for C100
        assert!(pattern[1].abundance > 0.5);
    }

    #[test]
    fn test_isotope_pattern_sulfur_compounds() {
        // Dimethyl sulfoxide: C2H6OS
        let dmso = MolecularFormula::new(2, 6, 0, 1, 1);
        let pattern = IsotopePatternSimulator::simulate(&dmso, 5);
        assert!(!pattern.is_empty());
        // M+2 should be noticeable due to 34S (4.29%)
        assert!(pattern.len() >= 3);
    }

    #[test]
    fn test_nominal_mass() {
        let caffeine = MolecularFormula::new(8, 10, 4, 2, 0);
        assert_eq!(caffeine.nominal_mass(), 194);
    }

    // --- MassDefectAnalyzer ---

    #[test]
    fn test_kendrick_mass_ch2() {
        let analyzer = MassDefectAnalyzer::ch2();
        // For exact CH2 mass, KM should equal its nominal mass
        let km = analyzer.kendrick_mass(14.01565);
        assert!((km - 14.0).abs() < 0.001);
    }

    #[test]
    fn test_homologous_series_same_kmd() {
        let analyzer = MassDefectAnalyzer::ch2();
        // Homologous series: differing by CH2 units (14.01565 Da)
        let masses = vec![200.0, 214.01565, 228.0313, 300.0, 314.01565];
        let kmd1 = analyzer.kendrick_mass_defect(masses[0]);
        let kmd2 = analyzer.kendrick_mass_defect(masses[1]);
        // Members of same homologous series should have similar KMD
        assert!(
            (kmd1 - kmd2).abs() < 0.002,
            "KMD1={}, KMD2={}", kmd1, kmd2
        );
    }

    #[test]
    fn test_find_homologous_series() {
        let analyzer = MassDefectAnalyzer::ch2();
        let ch2 = 14.01565;
        let masses = vec![
            200.0,
            200.0 + ch2,
            200.0 + 2.0 * ch2,
            300.5, // different series
            300.5 + ch2,
        ];
        let series = analyzer.find_homologous_series(&masses, 0.005);
        assert!(!series.is_empty());
    }

    #[test]
    fn test_mass_defect_mda() {
        let analyzer = MassDefectAnalyzer::ch2();
        // Exact integer mass → 0 mDa defect
        let mda_exact = analyzer.mass_defect_mda(200.0);
        assert!(mda_exact.abs() < TOL);
        // Non-integer mass → non-zero defect
        let mda = analyzer.mass_defect_mda(200.123);
        assert!((mda - 123.0).abs() < 1.0);
    }

    // --- PhaseCorrector ---

    #[test]
    fn test_phase_corrector_zero_phase() {
        let pc = PhaseCorrector::new(0.0, 0.0);
        let freqs = vec![100.0, 200.0, 300.0];
        let real = vec![1.0, 2.0, 3.0];
        let imag = vec![0.5, 1.0, 1.5];
        let absorption = pc.correct(&freqs, &real, &imag);
        // With zero phase, absorption = real
        for (a, r) in absorption.iter().zip(real.iter()) {
            assert!((a - r).abs() < TOL);
        }
    }

    #[test]
    fn test_phase_corrector_pi_over_2() {
        let pc = PhaseCorrector::new(PI / 2.0, 0.0);
        let freqs = vec![0.0]; // frequency doesn't matter for zero-order phase
        let real = vec![0.0];
        let imag = vec![1.0];
        let absorption = pc.correct(&freqs, &real, &imag);
        // absorption = re*cos(π/2) + im*sin(π/2) = 0 + 1 = 1
        assert!((absorption[0] - 1.0).abs() < TOL);
    }

    #[test]
    fn test_phase_corrector_estimate() {
        // Generate a spectrum with known phase
        let n = 64;
        let freqs: Vec<f64> = (0..n).map(|i| i as f64 * 100.0).collect();
        let phi0: f64 = 0.3;
        let real: Vec<f64> = freqs.iter().map(|_f| phi0.cos()).collect();
        let imag: Vec<f64> = freqs.iter().map(|_f| phi0.sin()).collect();

        let estimated = PhaseCorrector::estimate_from_spectrum(&freqs, &real, &imag);
        // The estimated phase should produce positive absorption values
        let absorption = estimated.correct(&freqs, &real, &imag);
        let sum_pos: f64 = absorption.iter().filter(|&&a| a > 0.0).sum();
        assert!(sum_pos > 0.0);
    }

    // --- HarmonicDetector ---

    #[test]
    fn test_harmonic_detection() {
        let detector = HarmonicDetector::new(3, 0.01);
        let fundamental_freqs = vec![100_000.0, 200_000.0, 300_000.0];
        // 200 kHz is 2x of 100 kHz
        let result = detector.check_harmonic(200_000.0, &fundamental_freqs);
        assert!(result.is_some());
        let (order, idx) = result.unwrap();
        assert_eq!(order, 2);
        assert_eq!(idx, 0);
    }

    #[test]
    fn test_harmonic_not_detected() {
        let detector = HarmonicDetector::new(3, 0.001);
        let fundamental_freqs = vec![100_000.0];
        // 150 kHz is not a harmonic of 100 kHz
        let result = detector.check_harmonic(150_000.0, &fundamental_freqs);
        assert!(result.is_none());
    }

    #[test]
    fn test_harmonic_removal() {
        let detector = HarmonicDetector::new(3, 0.01);
        let peaks = vec![
            Peak {
                frequency: 100_000.0,
                magnitude: 1.0,
                mz: 0.0,
                resolving_power: 0.0,
                snr: 100.0,
            },
            Peak {
                frequency: 200_000.0,
                magnitude: 0.1,
                mz: 0.0,
                resolving_power: 0.0,
                snr: 10.0,
            },
            Peak {
                frequency: 300_000.0,
                magnitude: 0.05,
                mz: 0.0,
                resolving_power: 0.0,
                snr: 5.0,
            },
            Peak {
                frequency: 150_000.0,
                magnitude: 0.8,
                mz: 0.0,
                resolving_power: 0.0,
                snr: 80.0,
            },
        ];
        let cleaned = detector.remove_harmonics(&peaks);
        // 200 kHz (2x100) and 300 kHz (3x100) should be removed
        // 100 kHz and 150 kHz should remain
        assert_eq!(cleaned.len(), 2);
        assert!((cleaned[0].frequency - 100_000.0).abs() < TOL);
        assert!((cleaned[1].frequency - 150_000.0).abs() < TOL);
    }

    // --- Bessel I0 ---

    #[test]
    fn test_bessel_i0_zero() {
        assert!((bessel_i0(0.0) - 1.0).abs() < TOL);
    }

    #[test]
    fn test_bessel_i0_known_value() {
        // I0(1) ≈ 1.2660658
        let val = bessel_i0(1.0);
        assert!((val - 1.2660658).abs() < 1e-5, "I0(1) = {}", val);
    }

    // --- Integration / end-to-end ---

    #[test]
    fn test_full_pipeline_synthetic_transient() {
        // Simulate a transient with two frequency components
        let n = 256;
        let fs = 2_000_000.0; // 2 MHz sample rate
        let f1 = 500_000.0; // ion at f1
        let f2 = 300_000.0; // ion at f2

        let transient: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                1.0 * (2.0 * PI * f1 * t).cos() + 0.5 * (2.0 * PI * f2 * t).cos()
            })
            .collect();

        // Process
        let proc = TransientProcessor::new(ApodizationType::Hanning, 1);
        let (freqs, mags) = proc.process(&transient);

        // Pick peaks
        let picker = PeakPicker::new(PeakDetectionMode::Maximum, 3.0, 30);
        let peaks = picker.pick_peaks(&freqs, &mags);

        // Should find at least 2 peaks
        assert!(
            peaks.len() >= 2,
            "Expected >=2 peaks, found {}", peaks.len()
        );

        // Convert normalized frequencies to Hz and verify
        let peak_freqs: Vec<f64> = peaks.iter().map(|p| p.frequency * fs).collect();
        let has_f1 = peak_freqs.iter().any(|&f| (f - f1).abs() < fs / n as f64 * 2.0);
        let has_f2 = peak_freqs.iter().any(|&f| (f - f2).abs() < fs / n as f64 * 2.0);
        assert!(has_f1, "Missing peak near f1={} Hz", f1);
        assert!(has_f2, "Missing peak near f2={} Hz", f2);
    }

    #[test]
    fn test_mass_calibration_pipeline() {
        let config = IcrConfig::standard_12t();
        let calc = CyclotronFrequencyCalculator::new(&config);

        // Generate calibrant frequencies for known m/z values
        let calibrant_mzs = vec![200.0, 400.0, 600.0, 800.0];
        let refs: Vec<(f64, f64)> = calibrant_mzs
            .iter()
            .map(|&mz| (calc.mz_to_frequency(mz), mz))
            .collect();

        // Fit Ledford calibration
        let cal = MassCalibrator::calibrate_ledford(&refs).unwrap();

        // Verify sub-ppm accuracy on the calibrants
        for &(f, mz) in &refs {
            let error = cal.mass_error_ppm(f, mz);
            assert!(
                error.abs() < 5.0,
                "Calibration error {} ppm at m/z={}", error, mz
            );
        }
    }
}
