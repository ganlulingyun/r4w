//! Ion Mobility Spectrometry (IMS) Signal Processing
//!
//! Implements signal processing and chemical identification for Ion Mobility
//! Spectrometry (IMS) — the primary technology in airport explosives detectors,
//! military chemical warfare agent (CWA) sensors, and law enforcement narcotics
//! screeners.
//!
//! ## Principles
//!
//! Ions drift through a tube under an applied electric field. Different ions
//! have different drift velocities depending on their mobility `K`:
//!
//! - `v_d = K * E` where E is the electric field (V/m)
//! - Drift time `t_d = L^2 / (K * V)` where L is tube length, V is drift voltage
//! - Reduced mobility `K0 = K * (P / P0) * (T0 / T)` normalises to STP
//!
//! Peak width is governed by diffusion broadening (Mason-Schamp equation)
//! and the gate pulse width.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::ion_mobility_spectrometer::{ImsConfig, ImsProcessor, IonPolarity};
//!
//! let config = ImsConfig::default();
//! let mut proc = ImsProcessor::new(config.clone());
//!
//! // Synthesise a simple spectrum with RIP and one analyte peak
//! let n = 500;
//! let dt_step = 30.0 / n as f64; // 0..30 ms
//! let drift_times: Vec<f64> = (0..n).map(|i| i as f64 * dt_step).collect();
//! let intensities: Vec<f64> = drift_times.iter().map(|&t| {
//!     1000.0 * (-((t - 6.5f64).powi(2)) / (2.0 * 0.15f64.powi(2))).exp()
//!     + 400.0 * (-((t - 12.0f64).powi(2)) / (2.0 * 0.2f64.powi(2))).exp()
//! }).collect();
//!
//! let spectrum = proc.process_spectrum(&drift_times, &intensities, IonPolarity::Positive);
//! assert!(spectrum.peaks.len() >= 2);
//! ```

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Boltzmann constant (J/K)
const K_BOLTZ: f64 = 1.380649e-23;

/// Elementary charge (C)
const ELEM_CHARGE: f64 = 1.602176634e-19;

/// Standard pressure (Pa)
const P0: f64 = 101325.0;

/// Standard temperature (K)
const T0: f64 = 273.15;

/// ln(2)
const LN2: f64 = 0.693147180559945;

/// sqrt(2 * pi)  -- hand-computed to avoid pulling in extra deps
const SQRT_2PI: f64 = 2.5066282746310002;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// IMS instrument configuration.
#[derive(Debug, Clone)]
pub struct ImsConfig {
    /// Drift tube length in metres (typical: 0.05 – 0.15 m).
    pub drift_tube_length_m: f64,
    /// Electric field in V/m (typical: 200 – 400 V/cm = 20000 – 40000 V/m).
    pub electric_field_vm: f64,
    /// Ambient temperature in kelvin (typical: 323 – 523 K).
    pub temperature_k: f64,
    /// Ambient pressure in pascal (typical: ~101325 Pa).
    pub pressure_pa: f64,
    /// Ion gate pulse width in microseconds (typical: 100 – 300 us).
    pub gate_pulse_width_us: f64,
    /// Peak detection threshold (fraction of max intensity, 0..1).
    pub peak_threshold: f64,
    /// Smoothing window size (samples). 0 = no smoothing.
    pub smoothing_window: usize,
    /// K0 match tolerance (cm²/V·s) for identification.
    pub k0_tolerance: f64,
}

impl Default for ImsConfig {
    fn default() -> Self {
        Self {
            drift_tube_length_m: 0.10,
            electric_field_vm: 25000.0, // 250 V/cm
            temperature_k: 373.15,      // 100 °C
            pressure_pa: P0,
            gate_pulse_width_us: 200.0,
            peak_threshold: 0.05,
            smoothing_window: 5,
            k0_tolerance: 0.08,
        }
    }
}

// ---------------------------------------------------------------------------
// Ion polarity
// ---------------------------------------------------------------------------

/// Ionisation polarity mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IonPolarity {
    /// Positive-ion mode (most CWA, narcotics).
    Positive,
    /// Negative-ion mode (most explosives).
    Negative,
}

// ---------------------------------------------------------------------------
// Alarm classes
// ---------------------------------------------------------------------------

/// Threat alarm category.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlarmClass {
    /// Chemical Warfare Agent.
    Cwa,
    /// Toxic Industrial Chemical.
    Tic,
    /// Explosive.
    Explosive,
    /// Narcotic.
    Narcotic,
}

// ---------------------------------------------------------------------------
// Known analyte database
// ---------------------------------------------------------------------------

/// An entry in the reduced-mobility lookup database.
#[derive(Debug, Clone)]
pub struct AnalyteEntry {
    /// Human-readable name.
    pub name: &'static str,
    /// Reduced mobility K0 in cm²/(V·s).
    pub k0: f64,
    /// Which ion polarity this is measured in.
    pub polarity: IonPolarity,
    /// Alarm class if detected.
    pub alarm_class: AlarmClass,
}

/// Built-in analyte database.
pub fn default_analyte_db() -> Vec<AnalyteEntry> {
    vec![
        // Positive-ion mode
        AnalyteEntry {
            name: "RIP+ (H3O+·(H2O)n)",
            k0: 2.22,
            polarity: IonPolarity::Positive,
            alarm_class: AlarmClass::Tic, // not really an alarm, but RIP marker
        },
        AnalyteEntry {
            name: "GB (Sarin)",
            k0: 1.61,
            polarity: IonPolarity::Positive,
            alarm_class: AlarmClass::Cwa,
        },
        AnalyteEntry {
            name: "VX",
            k0: 1.22,
            polarity: IonPolarity::Positive,
            alarm_class: AlarmClass::Cwa,
        },
        AnalyteEntry {
            name: "HD (Mustard)",
            k0: 1.48,
            polarity: IonPolarity::Positive,
            alarm_class: AlarmClass::Cwa,
        },
        AnalyteEntry {
            name: "Cocaine",
            k0: 1.16,
            polarity: IonPolarity::Positive,
            alarm_class: AlarmClass::Narcotic,
        },
        AnalyteEntry {
            name: "Heroin",
            k0: 1.04,
            polarity: IonPolarity::Positive,
            alarm_class: AlarmClass::Narcotic,
        },
        AnalyteEntry {
            name: "Methamphetamine",
            k0: 1.63,
            polarity: IonPolarity::Positive,
            alarm_class: AlarmClass::Narcotic,
        },
        AnalyteEntry {
            name: "MDMA",
            k0: 1.44,
            polarity: IonPolarity::Positive,
            alarm_class: AlarmClass::Narcotic,
        },
        // Negative-ion mode
        AnalyteEntry {
            name: "RIP- (O2-·(H2O)n)",
            k0: 2.18,
            polarity: IonPolarity::Negative,
            alarm_class: AlarmClass::Tic,
        },
        AnalyteEntry {
            name: "TNT",
            k0: 1.54,
            polarity: IonPolarity::Negative,
            alarm_class: AlarmClass::Explosive,
        },
        AnalyteEntry {
            name: "RDX",
            k0: 1.39,
            polarity: IonPolarity::Negative,
            alarm_class: AlarmClass::Explosive,
        },
        AnalyteEntry {
            name: "PETN",
            k0: 1.24,
            polarity: IonPolarity::Negative,
            alarm_class: AlarmClass::Explosive,
        },
        AnalyteEntry {
            name: "HMX",
            k0: 1.32,
            polarity: IonPolarity::Negative,
            alarm_class: AlarmClass::Explosive,
        },
        AnalyteEntry {
            name: "NG (Nitroglycerin)",
            k0: 1.46,
            polarity: IonPolarity::Negative,
            alarm_class: AlarmClass::Explosive,
        },
        AnalyteEntry {
            name: "EGDN",
            k0: 1.58,
            polarity: IonPolarity::Negative,
            alarm_class: AlarmClass::Explosive,
        },
        AnalyteEntry {
            name: "Chlorine (Cl2)",
            k0: 2.01,
            polarity: IonPolarity::Negative,
            alarm_class: AlarmClass::Tic,
        },
    ]
}

// ---------------------------------------------------------------------------
// Spectrum and peak types
// ---------------------------------------------------------------------------

/// A detected peak in an IMS spectrum.
#[derive(Debug, Clone)]
pub struct ImsPeak {
    /// Drift time of peak centre (ms).
    pub drift_time_ms: f64,
    /// Peak intensity (arbitrary units).
    pub height: f64,
    /// Full Width at Half Maximum (ms).
    pub fwhm_ms: f64,
    /// Peak area (intensity × ms).
    pub area: f64,
    /// Resolving power R = t_d / FWHM.
    pub resolving_power: f64,
    /// Reduced mobility K0 (cm²/V·s), computed from drift time and config.
    pub k0: f64,
    /// Identified analyte name, if any.
    pub identification: Option<String>,
    /// Alarm class, if identified.
    pub alarm_class: Option<AlarmClass>,
}

/// Processed IMS spectrum with peaks and alarms.
#[derive(Debug, Clone)]
pub struct ImsSpectrum {
    /// Drift times (ms) — same length as `intensities`.
    pub drift_time_ms: Vec<f64>,
    /// Processed intensities (after baseline/smoothing).
    pub intensities: Vec<f64>,
    /// Detected peaks.
    pub peaks: Vec<ImsPeak>,
    /// Active alarms.
    pub alarms: Vec<ImsAlarm>,
    /// Ion polarity used.
    pub polarity: IonPolarity,
}

/// An alarm raised by the IMS processor.
#[derive(Debug, Clone)]
pub struct ImsAlarm {
    /// Which substance triggered it.
    pub substance: String,
    /// Alarm class.
    pub class: AlarmClass,
    /// Confidence (0..1).
    pub confidence: f64,
    /// Measured drift time (ms).
    pub drift_time_ms: f64,
    /// Measured K0 (cm²/V·s).
    pub k0: f64,
}

// ---------------------------------------------------------------------------
// Core physics helpers (pub for testing)
// ---------------------------------------------------------------------------

/// Compute ion mobility K (cm²/V·s) from drift velocity (m/s) and electric
/// field (V/m).  Result converted to cm²/V·s (×1e4).
pub fn ion_mobility(drift_velocity_ms: f64, electric_field_vm: f64) -> f64 {
    (drift_velocity_ms / electric_field_vm) * 1e4
}

/// Reduced mobility K0 = K * (P/P0) * (T0/T).
pub fn reduced_mobility(k: f64, pressure_pa: f64, temperature_k: f64) -> f64 {
    k * (pressure_pa / P0) * (T0 / temperature_k)
}

/// Actual mobility K from reduced mobility K0.
pub fn actual_mobility(k0: f64, pressure_pa: f64, temperature_k: f64) -> f64 {
    k0 * (P0 / pressure_pa) * (temperature_k / T0)
}

/// Drift time in milliseconds: t_d = L² / (K_actual * V) where V = E*L,
/// so t_d = L / (K_actual * E).  K_actual in cm²/V·s needs conversion to
/// m²/V·s (÷1e4).
pub fn drift_time_ms(
    tube_length_m: f64,
    electric_field_vm: f64,
    k_actual_cm2: f64,
) -> f64 {
    let k_m2 = k_actual_cm2 * 1e-4;
    let t_s = tube_length_m / (k_m2 * electric_field_vm);
    t_s * 1e3 // seconds → milliseconds
}

/// K0 from measured drift time (ms) and instrument geometry.
pub fn k0_from_drift_time(
    td_ms: f64,
    tube_length_m: f64,
    electric_field_vm: f64,
    pressure_pa: f64,
    temperature_k: f64,
) -> f64 {
    let td_s = td_ms * 1e-3;
    let k_m2 = tube_length_m / (td_s * electric_field_vm);
    let k_cm2 = k_m2 * 1e4;
    reduced_mobility(k_cm2, pressure_pa, temperature_k)
}

/// Diffusion-limited FWHM (ms).
/// W_diff = t_d * sqrt(16 * k_B * T * ln(2) / (e * V))
/// where V = E * L is the drift voltage.
pub fn diffusion_fwhm_ms(
    td_ms: f64,
    temperature_k: f64,
    electric_field_vm: f64,
    tube_length_m: f64,
) -> f64 {
    let v_drift = electric_field_vm * tube_length_m; // drift voltage
    let factor = (16.0 * K_BOLTZ * temperature_k * LN2 / (ELEM_CHARGE * v_drift)).sqrt();
    td_ms * factor
}

/// Diffusion-limited resolving power.
/// R_diff = sqrt(e * V / (16 * k_B * T * ln(2)))
pub fn diffusion_resolving_power(
    temperature_k: f64,
    electric_field_vm: f64,
    tube_length_m: f64,
) -> f64 {
    let v_drift = electric_field_vm * tube_length_m;
    (ELEM_CHARGE * v_drift / (16.0 * K_BOLTZ * temperature_k * LN2)).sqrt()
}

/// Total FWHM including gate pulse broadening (ms).
/// W_total = sqrt(W_diff² + W_gate²)
pub fn total_fwhm_ms(diffusion_fwhm: f64, gate_pulse_width_ms: f64) -> f64 {
    (diffusion_fwhm * diffusion_fwhm + gate_pulse_width_ms * gate_pulse_width_ms).sqrt()
}

/// Number density N (molecules/m³) from ideal gas law N = P / (k_B * T).
pub fn number_density(pressure_pa: f64, temperature_k: f64) -> f64 {
    pressure_pa / (K_BOLTZ * temperature_k)
}

/// Simplified Mason-Schamp: K0 = (3e / (16 * N)) * sqrt(2*pi / (mu * k_B * T))
/// * (1 + alpha) / Omega.
/// Here `mu` is the reduced mass (kg), `omega_m2` is the collision cross-section
/// (m²), and `alpha` is the correction factor (typically small, often set to 0).
pub fn mason_schamp_k0(
    temperature_k: f64,
    pressure_pa: f64,
    reduced_mass_kg: f64,
    omega_m2: f64,
    alpha: f64,
) -> f64 {
    let n = number_density(pressure_pa, temperature_k);
    let prefactor = 3.0 * ELEM_CHARGE / (16.0 * n);
    let thermal = (2.0 * std::f64::consts::PI / (reduced_mass_kg * K_BOLTZ * temperature_k)).sqrt();
    let k0_m2 = prefactor * thermal * (1.0 + alpha) / omega_m2;
    k0_m2 * 1e4 // m²/V·s → cm²/V·s
}

// ---------------------------------------------------------------------------
// Signal processing helpers
// ---------------------------------------------------------------------------

/// Boxcar (moving-average) smoothing.  Returns a vector the same length as
/// `data`; edges are handled by shrinking the window.
fn boxcar_smooth(data: &[f64], window: usize) -> Vec<f64> {
    if window <= 1 || data.is_empty() {
        return data.to_vec();
    }
    let n = data.len();
    let half = window / 2;
    let mut out = vec![0.0; n];
    for i in 0..n {
        let lo = if i >= half { i - half } else { 0 };
        let hi = if i + half < n { i + half } else { n - 1 };
        let count = (hi - lo + 1) as f64;
        let sum: f64 = data[lo..=hi].iter().sum();
        out[i] = sum / count;
    }
    out
}

/// Rolling-minimum baseline estimation.
fn rolling_minimum(data: &[f64], window: usize) -> Vec<f64> {
    if window <= 1 || data.is_empty() {
        return data.to_vec();
    }
    let n = data.len();
    let half = window / 2;
    let mut baseline = vec![0.0; n];
    for i in 0..n {
        let lo = if i >= half { i - half } else { 0 };
        let hi = if i + half < n { i + half } else { n - 1 };
        let mut min_val = f64::MAX;
        for j in lo..=hi {
            if data[j] < min_val {
                min_val = data[j];
            }
        }
        baseline[i] = min_val;
    }
    baseline
}

/// Subtract a reference (background) spectrum from a signal spectrum.
/// Both must have the same length.
pub fn background_subtract(signal: &[f64], background: &[f64]) -> Vec<f64> {
    signal
        .iter()
        .zip(background.iter())
        .map(|(&s, &b)| (s - b).max(0.0))
        .collect()
}

/// Average multiple spectra element-wise.
pub fn average_spectra(spectra: &[Vec<f64>]) -> Vec<f64> {
    if spectra.is_empty() {
        return Vec::new();
    }
    let n = spectra[0].len();
    let count = spectra.len() as f64;
    let mut avg = vec![0.0; n];
    for spectrum in spectra {
        for (i, &v) in spectrum.iter().enumerate().take(n) {
            avg[i] += v;
        }
    }
    for v in &mut avg {
        *v /= count;
    }
    avg
}

/// Gaussian function: A * exp(-(x-mu)^2 / (2*sigma^2)).
fn gaussian(x: f64, amplitude: f64, mu: f64, sigma: f64) -> f64 {
    amplitude * (-((x - mu).powi(2)) / (2.0 * sigma * sigma)).exp()
}

/// Simple Gaussian peak fitting around a local maximum.
/// Returns (amplitude, centre, sigma).
fn fit_gaussian_peak(times: &[f64], intensities: &[f64], peak_idx: usize) -> (f64, f64, f64) {
    let amplitude = intensities[peak_idx];
    let centre = times[peak_idx];

    // Estimate sigma from half-max width
    let half_max = amplitude * 0.5;
    let n = times.len();

    // Search left for half-max crossing
    let mut left_t = centre;
    for i in (0..peak_idx).rev() {
        if intensities[i] <= half_max {
            // Linear interpolation
            let frac = if (intensities[i + 1] - intensities[i]).abs() > 1e-30 {
                (half_max - intensities[i]) / (intensities[i + 1] - intensities[i])
            } else {
                0.5
            };
            left_t = times[i] + frac * (times[i + 1] - times[i]);
            break;
        }
    }

    // Search right for half-max crossing
    let mut right_t = centre;
    for i in (peak_idx + 1)..n {
        if intensities[i] <= half_max {
            let frac = if (intensities[i - 1] - intensities[i]).abs() > 1e-30 {
                (half_max - intensities[i]) / (intensities[i - 1] - intensities[i])
            } else {
                0.5
            };
            right_t = times[i] - frac * (times[i] - times[i - 1]);
            break;
        }
    }

    let fwhm = (right_t - left_t).abs();
    let sigma = if fwhm > 1e-15 {
        fwhm / (2.0 * (2.0 * LN2).sqrt())
    } else {
        // Fallback: use spacing
        if n > 1 {
            (times[1] - times[0]).abs()
        } else {
            1e-6
        }
    };

    (amplitude, centre, sigma)
}

/// Peak area via trapezoidal integration over ±3σ around the peak centre.
fn peak_area(times: &[f64], intensities: &[f64], centre: f64, sigma: f64) -> f64 {
    let lo = centre - 3.0 * sigma;
    let hi = centre + 3.0 * sigma;
    let mut area = 0.0;
    for i in 1..times.len() {
        if times[i - 1] >= lo && times[i] <= hi {
            let dt = times[i] - times[i - 1];
            area += 0.5 * (intensities[i - 1] + intensities[i]) * dt;
        }
    }
    area
}

/// Estimate detection limit from signal and noise.
/// SNR = peak_height / noise_rms.
/// Detection limit is the concentration at which SNR = 3 (3-sigma criterion).
pub fn detection_limit_snr(peak_height: f64, noise_rms: f64) -> f64 {
    if peak_height.abs() < 1e-30 {
        return f64::INFINITY;
    }
    let snr = peak_height / noise_rms;
    if snr < 1e-30 {
        return f64::INFINITY;
    }
    3.0 / snr
}

/// RMS of a signal slice (used for noise estimation in baseline regions).
pub fn rms(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    let sum_sq: f64 = data.iter().map(|&x| x * x).sum();
    (sum_sq / data.len() as f64).sqrt()
}

// ---------------------------------------------------------------------------
// Multi-peak deconvolution
// ---------------------------------------------------------------------------

/// Result of multi-peak Gaussian deconvolution.
#[derive(Debug, Clone)]
pub struct DeconvolutionResult {
    /// Fitted Gaussian parameters: (amplitude, centre_ms, sigma_ms).
    pub components: Vec<(f64, f64, f64)>,
    /// Residual RMS after subtracting all fitted Gaussians.
    pub residual_rms: f64,
}

/// Simple iterative Gaussian deconvolution for overlapping IMS peaks.
///
/// 1. Find the strongest peak in the residual.
/// 2. Fit a Gaussian and subtract it.
/// 3. Repeat until no peak exceeds `min_height` or `max_peaks` reached.
pub fn deconvolve_peaks(
    times: &[f64],
    intensities: &[f64],
    min_height: f64,
    max_peaks: usize,
) -> DeconvolutionResult {
    let n = times.len();
    if n < 3 {
        return DeconvolutionResult {
            components: Vec::new(),
            residual_rms: rms(intensities),
        };
    }

    let mut residual = intensities.to_vec();
    let mut components = Vec::new();

    for _ in 0..max_peaks {
        // Find maximum in residual
        let mut max_idx = 0;
        let mut max_val = f64::NEG_INFINITY;
        for (i, &v) in residual.iter().enumerate() {
            if v > max_val {
                max_val = v;
                max_idx = i;
            }
        }

        if max_val < min_height {
            break;
        }

        // Fit Gaussian around this peak
        let (amp, mu, sigma) = fit_gaussian_peak(times, &residual, max_idx);
        if sigma < 1e-15 {
            break;
        }

        // Subtract fitted Gaussian from residual
        for (i, t) in times.iter().enumerate() {
            residual[i] -= gaussian(*t, amp, mu, sigma);
            if residual[i] < 0.0 {
                residual[i] = 0.0;
            }
        }

        components.push((amp, mu, sigma));
    }

    let residual_rms = rms(&residual);
    DeconvolutionResult {
        components,
        residual_rms,
    }
}

// ---------------------------------------------------------------------------
// IMS Processor
// ---------------------------------------------------------------------------

/// Main IMS signal processor.
#[derive(Debug, Clone)]
pub struct ImsProcessor {
    config: ImsConfig,
    analyte_db: Vec<AnalyteEntry>,
    /// Optional clean-air background spectrum for subtraction.
    background: Option<Vec<f64>>,
    /// Buffer for spectrum averaging.
    avg_buffer: Vec<Vec<f64>>,
    /// Number of spectra to average (1 = no averaging).
    avg_count: usize,
}

impl ImsProcessor {
    /// Create a new IMS processor with the given configuration.
    pub fn new(config: ImsConfig) -> Self {
        Self {
            config,
            analyte_db: default_analyte_db(),
            background: None,
            avg_buffer: Vec::new(),
            avg_count: 1,
        }
    }

    /// Set a custom analyte database.
    pub fn set_analyte_db(&mut self, db: Vec<AnalyteEntry>) {
        self.analyte_db = db;
    }

    /// Set the clean-air background spectrum.
    pub fn set_background(&mut self, bg: Vec<f64>) {
        self.background = Some(bg);
    }

    /// Clear the background spectrum.
    pub fn clear_background(&mut self) {
        self.background = None;
    }

    /// Set the number of spectra to average before processing.
    pub fn set_averaging(&mut self, count: usize) {
        self.avg_count = count.max(1);
        self.avg_buffer.clear();
    }

    /// Expected drift time (ms) for a given reduced mobility K0 (cm²/V·s).
    pub fn expected_drift_time(&self, k0: f64) -> f64 {
        let k_actual = actual_mobility(k0, self.config.pressure_pa, self.config.temperature_k);
        drift_time_ms(
            self.config.drift_tube_length_m,
            self.config.electric_field_vm,
            k_actual,
        )
    }

    /// Theoretical diffusion-limited resolving power for this configuration.
    pub fn theoretical_resolving_power(&self) -> f64 {
        diffusion_resolving_power(
            self.config.temperature_k,
            self.config.electric_field_vm,
            self.config.drift_tube_length_m,
        )
    }

    /// Process a raw IMS spectrum.
    ///
    /// `drift_times` and `raw_intensities` must have the same length.
    pub fn process_spectrum(
        &mut self,
        drift_times: &[f64],
        raw_intensities: &[f64],
        polarity: IonPolarity,
    ) -> ImsSpectrum {
        let n = drift_times.len();
        assert_eq!(n, raw_intensities.len(), "drift_times and intensities must match");

        // --- Averaging ---
        let intensities = if self.avg_count > 1 {
            self.avg_buffer.push(raw_intensities.to_vec());
            if self.avg_buffer.len() > self.avg_count {
                self.avg_buffer.remove(0);
            }
            average_spectra(&self.avg_buffer)
        } else {
            raw_intensities.to_vec()
        };

        // --- Background subtraction ---
        let intensities = if let Some(ref bg) = self.background {
            if bg.len() == intensities.len() {
                background_subtract(&intensities, bg)
            } else {
                intensities
            }
        } else {
            intensities
        };

        // --- Baseline subtraction (rolling minimum) ---
        let baseline_window = n / 10; // ~10% of spectrum width
        let baseline_window = baseline_window.max(5);
        let baseline = rolling_minimum(&intensities, baseline_window);
        let mut proc_data: Vec<f64> = intensities
            .iter()
            .zip(baseline.iter())
            .map(|(&s, &b)| (s - b).max(0.0))
            .collect();

        // --- Smoothing ---
        if self.config.smoothing_window > 1 {
            proc_data = boxcar_smooth(&proc_data, self.config.smoothing_window);
        }

        // --- Peak detection ---
        let max_intensity = proc_data.iter().cloned().fold(0.0_f64, f64::max);
        let threshold = max_intensity * self.config.peak_threshold;

        let mut peak_indices = Vec::new();
        for i in 1..n.saturating_sub(1) {
            if proc_data[i] > threshold
                && proc_data[i] >= proc_data[i - 1]
                && proc_data[i] >= proc_data[i + 1]
            {
                // Avoid double-counting: skip if too close to the previous peak
                let too_close = peak_indices.last().map_or(false, |&prev: &usize| i - prev < 3);
                if !too_close {
                    peak_indices.push(i);
                }
            }
        }

        // --- Fit peaks ---
        let gate_fwhm_ms = self.config.gate_pulse_width_us * 1e-3; // us → ms
        let mut peaks = Vec::new();
        for &idx in &peak_indices {
            let (amp, centre, sigma) = fit_gaussian_peak(drift_times, &proc_data, idx);
            let fwhm = sigma * 2.0 * (2.0 * LN2).sqrt();
            let area = peak_area(drift_times, &proc_data, centre, sigma);
            let resolving_power = if fwhm > 1e-12 { centre / fwhm } else { 0.0 };

            let k0 = k0_from_drift_time(
                centre,
                self.config.drift_tube_length_m,
                self.config.electric_field_vm,
                self.config.pressure_pa,
                self.config.temperature_k,
            );

            // --- Identification (closest match within tolerance) ---
            let mut identification = None;
            let mut alarm_class = None;
            let mut best_delta = f64::MAX;
            for entry in &self.analyte_db {
                if entry.polarity == polarity {
                    let delta = (entry.k0 - k0).abs();
                    if delta < self.config.k0_tolerance && delta < best_delta {
                        best_delta = delta;
                        identification = Some(entry.name.to_string());
                        alarm_class = Some(entry.alarm_class);
                    }
                }
            }

            // Theoretical FWHM for comparison
            let _diff_fwhm = diffusion_fwhm_ms(
                centre,
                self.config.temperature_k,
                self.config.electric_field_vm,
                self.config.drift_tube_length_m,
            );
            let _total_theory_fwhm = total_fwhm_ms(_diff_fwhm, gate_fwhm_ms);

            peaks.push(ImsPeak {
                drift_time_ms: centre,
                height: amp,
                fwhm_ms: fwhm,
                area,
                resolving_power,
                k0,
                identification,
                alarm_class,
            });
        }

        // --- Alarm generation ---
        let alarms: Vec<ImsAlarm> = peaks
            .iter()
            .filter(|p| p.alarm_class.is_some() && p.alarm_class != Some(AlarmClass::Tic))
            .map(|p| {
                // Confidence based on closeness of K0 match
                let best_match = self
                    .analyte_db
                    .iter()
                    .filter(|e| e.polarity == polarity)
                    .min_by(|a, b| {
                        (a.k0 - p.k0)
                            .abs()
                            .partial_cmp(&(b.k0 - p.k0).abs())
                            .unwrap_or(std::cmp::Ordering::Equal)
                    });
                let confidence = if let Some(entry) = best_match {
                    let delta = (entry.k0 - p.k0).abs();
                    (1.0 - delta / self.config.k0_tolerance).max(0.0)
                } else {
                    0.0
                };

                ImsAlarm {
                    substance: p.identification.clone().unwrap_or_default(),
                    class: p.alarm_class.unwrap(),
                    confidence,
                    drift_time_ms: p.drift_time_ms,
                    k0: p.k0,
                }
            })
            .collect();

        ImsSpectrum {
            drift_time_ms: drift_times.to_vec(),
            intensities: proc_data,
            peaks,
            alarms,
            polarity,
        }
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    // Helper: generate a Gaussian peak spectrum
    fn make_spectrum(
        n: usize,
        t_max_ms: f64,
        peaks: &[(f64, f64, f64)], // (centre_ms, amplitude, sigma_ms)
    ) -> (Vec<f64>, Vec<f64>) {
        let dt = t_max_ms / n as f64;
        let times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        let mut intensities = vec![0.0; n];
        for &(centre, amp, sigma) in peaks {
            for (i, t) in times.iter().enumerate() {
                intensities[i] += gaussian(*t, amp, sigma, centre);
                // Note: gaussian(x, A, mu, sigma) — pass (time, amp, centre, sigma)
            }
        }
        (times, intensities)
    }

    // Corrected helper that uses gaussian() properly
    fn make_spectrum_correct(
        n: usize,
        t_max_ms: f64,
        peaks: &[(f64, f64, f64)], // (centre_ms, amplitude, sigma_ms)
    ) -> (Vec<f64>, Vec<f64>) {
        let dt = t_max_ms / n as f64;
        let times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        let mut intensities = vec![0.0; n];
        for &(centre, amp, sigma) in peaks {
            for (i, t) in times.iter().enumerate() {
                intensities[i] += amp * (-((t - centre).powi(2)) / (2.0 * sigma * sigma)).exp();
            }
        }
        (times, intensities)
    }

    // -----------------------------------------------------------------------
    // 1. Ion mobility basics
    // -----------------------------------------------------------------------

    #[test]
    fn test_ion_mobility_basic() {
        // v_d = 10 m/s, E = 20000 V/m → K = 10/20000 = 5e-4 m²/Vs = 5.0 cm²/Vs (×1e4)
        // But wait: ion_mobility returns (v/E)*1e4
        let k = ion_mobility(10.0, 20000.0);
        assert!((k - 5.0).abs() < 1e-6, "K = {}", k);
    }

    #[test]
    fn test_reduced_mobility_at_stp() {
        // At STP (P0, T0), K0 should equal K
        let k = 2.22;
        let k0 = reduced_mobility(k, P0, T0);
        assert!((k0 - k).abs() < 1e-10, "K0 = {}", k0);
    }

    #[test]
    fn test_reduced_mobility_correction() {
        // At 2*P0 and T0, K0 = K * 2
        let k = 1.0;
        let k0 = reduced_mobility(k, 2.0 * P0, T0);
        assert!((k0 - 2.0).abs() < 1e-10, "K0 = {}", k0);

        // At P0 and 2*T0, K0 = K * 0.5
        let k0 = reduced_mobility(k, P0, 2.0 * T0);
        assert!((k0 - 0.5).abs() < 1e-10, "K0 = {}", k0);
    }

    #[test]
    fn test_actual_mobility_roundtrip() {
        let k0 = 1.54; // TNT
        let p = 95000.0;
        let t = 400.0;
        let k = actual_mobility(k0, p, t);
        let k0_back = reduced_mobility(k, p, t);
        assert!((k0_back - k0).abs() < 1e-10, "roundtrip failed: {} vs {}", k0_back, k0);
    }

    // -----------------------------------------------------------------------
    // 2. Drift time
    // -----------------------------------------------------------------------

    #[test]
    fn test_drift_time_rip_positive() {
        // RIP+ K0=2.22, standard instrument
        let config = ImsConfig::default();
        let k_actual = actual_mobility(2.22, config.pressure_pa, config.temperature_k);
        let td = drift_time_ms(config.drift_tube_length_m, config.electric_field_vm, k_actual);
        // Should be a few ms
        assert!(td > 0.5 && td < 20.0, "drift time = {} ms", td);
    }

    #[test]
    fn test_drift_time_tnt() {
        let config = ImsConfig::default();
        let k_actual = actual_mobility(1.54, config.pressure_pa, config.temperature_k);
        let td = drift_time_ms(config.drift_tube_length_m, config.electric_field_vm, k_actual);
        // TNT should drift slower than RIP (higher drift time)
        let k_rip = actual_mobility(2.22, config.pressure_pa, config.temperature_k);
        let td_rip = drift_time_ms(config.drift_tube_length_m, config.electric_field_vm, k_rip);
        assert!(td > td_rip, "TNT td={} should be > RIP td={}", td, td_rip);
    }

    #[test]
    fn test_k0_from_drift_time_roundtrip() {
        let config = ImsConfig::default();
        let k0_orig = 1.54;
        let k_actual = actual_mobility(k0_orig, config.pressure_pa, config.temperature_k);
        let td = drift_time_ms(config.drift_tube_length_m, config.electric_field_vm, k_actual);
        let k0_back = k0_from_drift_time(
            td,
            config.drift_tube_length_m,
            config.electric_field_vm,
            config.pressure_pa,
            config.temperature_k,
        );
        assert!(
            (k0_back - k0_orig).abs() < 1e-8,
            "roundtrip: {} vs {}",
            k0_back,
            k0_orig
        );
    }

    // -----------------------------------------------------------------------
    // 3. Diffusion broadening
    // -----------------------------------------------------------------------

    #[test]
    fn test_diffusion_fwhm_positive() {
        let config = ImsConfig::default();
        let td = 8.0; // ms
        let fwhm = diffusion_fwhm_ms(
            td,
            config.temperature_k,
            config.electric_field_vm,
            config.drift_tube_length_m,
        );
        assert!(fwhm > 0.0, "FWHM must be positive");
        assert!(fwhm < td, "FWHM {} must be < drift time {}", fwhm, td);
    }

    #[test]
    fn test_diffusion_resolving_power() {
        let config = ImsConfig::default();
        let r = diffusion_resolving_power(
            config.temperature_k,
            config.electric_field_vm,
            config.drift_tube_length_m,
        );
        // Typical IMS resolving power: 20 – 200
        assert!(r > 10.0, "R = {} too low", r);
        assert!(r < 500.0, "R = {} too high", r);
    }

    #[test]
    fn test_total_fwhm_dominance() {
        // When gate pulse is much wider than diffusion, it dominates
        let diff = 0.1; // ms
        let gate = 1.0; // ms
        let total = total_fwhm_ms(diff, gate);
        assert!((total - gate).abs() < 0.01, "total={}, should ≈ gate={}", total, gate);

        // When diffusion dominates
        let diff2 = 1.0;
        let gate2 = 0.01;
        let total2 = total_fwhm_ms(diff2, gate2);
        assert!(
            (total2 - diff2).abs() < 0.01,
            "total={}, should ≈ diff={}",
            total2,
            diff2
        );
    }

    // -----------------------------------------------------------------------
    // 4. Number density and Mason-Schamp
    // -----------------------------------------------------------------------

    #[test]
    fn test_number_density_stp() {
        let n = number_density(P0, T0);
        // Loschmidt constant ≈ 2.687e25 /m³
        assert!((n - 2.687e25).abs() / 2.687e25 < 0.01, "N = {}", n);
    }

    #[test]
    fn test_mason_schamp_reasonable() {
        // Rough check: ion with mass ~100 amu in N2 buffer gas at STP
        // Ion mass ~100 amu, buffer gas mass ~28 amu → reduced mass ≈ 100*28/(100+28) amu
        let m_ion = 100.0 * 1.6605e-27;
        let m_gas = 28.0 * 1.6605e-27;
        let mu = m_ion * m_gas / (m_ion + m_gas);
        // Collision cross-section for medium-sized organic ion ≈ 1.5e-18 m² (150 Å²)
        let omega = 1.5e-18;
        let k0 = mason_schamp_k0(T0, P0, mu, omega, 0.0);
        // Should be in the range 0.5 – 5 cm²/Vs (typical IMS range)
        assert!(k0 > 0.1 && k0 < 10.0, "K0 = {} cm²/Vs", k0);
    }

    #[test]
    fn test_mason_schamp_cross_section_inverse() {
        // Doubling cross-section should halve K0
        let mu = 14.0 * 1.6605e-27;
        let omega1 = 1.0e-19;
        let omega2 = 2.0e-19;
        let k0_1 = mason_schamp_k0(T0, P0, mu, omega1, 0.0);
        let k0_2 = mason_schamp_k0(T0, P0, mu, omega2, 0.0);
        assert!(
            ((k0_1 / k0_2) - 2.0).abs() < 1e-6,
            "ratio = {}",
            k0_1 / k0_2
        );
    }

    // -----------------------------------------------------------------------
    // 5. Signal processing helpers
    // -----------------------------------------------------------------------

    #[test]
    fn test_boxcar_smooth_identity() {
        let data = vec![1.0, 2.0, 3.0, 4.0, 5.0];
        let smoothed = boxcar_smooth(&data, 1);
        assert_eq!(smoothed, data);
    }

    #[test]
    fn test_boxcar_smooth_flat() {
        let data = vec![5.0; 20];
        let smoothed = boxcar_smooth(&data, 5);
        for v in &smoothed {
            assert!((v - 5.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_boxcar_smooth_reduces_noise() {
        // Step function with noise
        let mut data = vec![0.0; 50];
        for i in 25..50 {
            data[i] = 10.0;
        }
        data[30] = 15.0; // spike
        let smoothed = boxcar_smooth(&data, 5);
        // The spike should be reduced
        assert!(smoothed[30] < 15.0);
    }

    #[test]
    fn test_rolling_minimum() {
        let data = vec![5.0, 3.0, 8.0, 2.0, 7.0, 1.0, 6.0];
        let baseline = rolling_minimum(&data, 3);
        // Each element should be ≤ the original
        for (i, (&b, &d)) in baseline.iter().zip(data.iter()).enumerate() {
            assert!(b <= d, "baseline[{}]={} > data[{}]={}", i, b, i, d);
        }
    }

    #[test]
    fn test_background_subtract() {
        let signal = vec![10.0, 20.0, 15.0, 5.0];
        let bg = vec![5.0, 8.0, 20.0, 3.0];
        let result = background_subtract(&signal, &bg);
        assert!((result[0] - 5.0).abs() < 1e-10);
        assert!((result[1] - 12.0).abs() < 1e-10);
        assert!((result[2] - 0.0).abs() < 1e-10); // clamped to 0
        assert!((result[3] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_average_spectra() {
        let s1 = vec![10.0, 20.0, 30.0];
        let s2 = vec![20.0, 40.0, 60.0];
        let avg = average_spectra(&[s1, s2]);
        assert!((avg[0] - 15.0).abs() < 1e-10);
        assert!((avg[1] - 30.0).abs() < 1e-10);
        assert!((avg[2] - 45.0).abs() < 1e-10);
    }

    #[test]
    fn test_average_spectra_empty() {
        let avg = average_spectra(&[]);
        assert!(avg.is_empty());
    }

    #[test]
    fn test_rms_calculation() {
        let data = vec![3.0, 4.0]; // rms = sqrt((9+16)/2) = sqrt(12.5)
        let r = rms(&data);
        assert!((r - (12.5_f64).sqrt()).abs() < 1e-10);
    }

    #[test]
    fn test_rms_empty() {
        assert!((rms(&[]) - 0.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // 6. Peak detection and identification
    // -----------------------------------------------------------------------

    #[test]
    fn test_single_peak_detection() {
        let (times, intensities) = make_spectrum_correct(500, 30.0, &[(10.0, 1000.0, 0.3)]);
        let config = ImsConfig {
            smoothing_window: 0,
            ..ImsConfig::default()
        };
        let mut proc = ImsProcessor::new(config);
        let spectrum = proc.process_spectrum(&times, &intensities, IonPolarity::Positive);
        assert!(
            spectrum.peaks.len() >= 1,
            "expected >= 1 peak, got {}",
            spectrum.peaks.len()
        );
        // Peak should be near t=10 ms
        let best = &spectrum.peaks[0];
        assert!(
            (best.drift_time_ms - 10.0).abs() < 1.0,
            "peak at {} ms, expected ~10",
            best.drift_time_ms
        );
    }

    #[test]
    fn test_two_peak_detection() {
        let (times, intensities) =
            make_spectrum_correct(1000, 30.0, &[(8.0, 1000.0, 0.3), (15.0, 600.0, 0.4)]);
        let config = ImsConfig {
            smoothing_window: 0,
            ..ImsConfig::default()
        };
        let mut proc = ImsProcessor::new(config);
        let spectrum = proc.process_spectrum(&times, &intensities, IonPolarity::Negative);
        assert!(
            spectrum.peaks.len() >= 2,
            "expected >=2 peaks, got {}",
            spectrum.peaks.len()
        );
    }

    #[test]
    fn test_peak_fwhm_reasonable() {
        let sigma = 0.3;
        let expected_fwhm = sigma * 2.0 * (2.0 * LN2).sqrt();
        let (times, intensities) = make_spectrum_correct(2000, 30.0, &[(10.0, 1000.0, sigma)]);
        let config = ImsConfig {
            smoothing_window: 0,
            ..ImsConfig::default()
        };
        let mut proc = ImsProcessor::new(config);
        let spectrum = proc.process_spectrum(&times, &intensities, IonPolarity::Positive);
        assert!(!spectrum.peaks.is_empty());
        let fwhm = spectrum.peaks[0].fwhm_ms;
        assert!(
            (fwhm - expected_fwhm).abs() < 0.15,
            "FWHM = {}, expected ~{}",
            fwhm,
            expected_fwhm
        );
    }

    #[test]
    fn test_resolving_power() {
        let sigma = 0.2;
        let centre = 12.0;
        let expected_fwhm = sigma * 2.0 * (2.0 * LN2).sqrt();
        let expected_r = centre / expected_fwhm;
        let (times, intensities) = make_spectrum_correct(2000, 30.0, &[(centre, 1000.0, sigma)]);
        let config = ImsConfig {
            smoothing_window: 0,
            ..ImsConfig::default()
        };
        let mut proc = ImsProcessor::new(config);
        let spectrum = proc.process_spectrum(&times, &intensities, IonPolarity::Positive);
        assert!(!spectrum.peaks.is_empty());
        let r = spectrum.peaks[0].resolving_power;
        assert!(
            (r - expected_r).abs() / expected_r < 0.2,
            "R = {}, expected ~{}",
            r,
            expected_r
        );
    }

    #[test]
    fn test_peak_area_positive() {
        let (times, intensities) = make_spectrum_correct(2000, 30.0, &[(10.0, 500.0, 0.3)]);
        let config = ImsConfig {
            smoothing_window: 0,
            ..ImsConfig::default()
        };
        let mut proc = ImsProcessor::new(config);
        let spectrum = proc.process_spectrum(&times, &intensities, IonPolarity::Positive);
        assert!(!spectrum.peaks.is_empty());
        assert!(
            spectrum.peaks[0].area > 0.0,
            "area = {}",
            spectrum.peaks[0].area
        );
    }

    // -----------------------------------------------------------------------
    // 7. Identification and alarms
    // -----------------------------------------------------------------------

    #[test]
    fn test_tnt_identification() {
        // Place a peak at the drift time expected for TNT (K0=1.54) in negative mode
        let config = ImsConfig::default();
        let mut proc = ImsProcessor::new(config.clone());
        let td_tnt = proc.expected_drift_time(1.54);

        let (times, intensities) =
            make_spectrum_correct(2000, 30.0, &[(td_tnt, 800.0, 0.25)]);
        let spectrum = proc.process_spectrum(&times, &intensities, IonPolarity::Negative);
        let tnt_peak = spectrum.peaks.iter().find(|p| {
            p.identification
                .as_ref()
                .map_or(false, |name| name.contains("TNT"))
        });
        assert!(tnt_peak.is_some(), "TNT should be identified");
        assert_eq!(tnt_peak.unwrap().alarm_class, Some(AlarmClass::Explosive));
    }

    #[test]
    fn test_cocaine_identification() {
        let config = ImsConfig::default();
        let mut proc = ImsProcessor::new(config);
        let td_cocaine = proc.expected_drift_time(1.16);

        let (times, intensities) =
            make_spectrum_correct(2000, 30.0, &[(td_cocaine, 600.0, 0.3)]);
        let spectrum = proc.process_spectrum(&times, &intensities, IonPolarity::Positive);
        let cocaine_peak = spectrum.peaks.iter().find(|p| {
            p.identification
                .as_ref()
                .map_or(false, |name| name.contains("Cocaine"))
        });
        assert!(cocaine_peak.is_some(), "Cocaine should be identified");
        assert_eq!(cocaine_peak.unwrap().alarm_class, Some(AlarmClass::Narcotic));
    }

    #[test]
    fn test_alarm_generation_explosive() {
        let config = ImsConfig::default();
        let mut proc = ImsProcessor::new(config);
        let td_rdx = proc.expected_drift_time(1.39);

        let (times, intensities) =
            make_spectrum_correct(2000, 30.0, &[(td_rdx, 700.0, 0.25)]);
        let spectrum = proc.process_spectrum(&times, &intensities, IonPolarity::Negative);
        assert!(
            !spectrum.alarms.is_empty(),
            "should raise an alarm for RDX"
        );
        assert_eq!(spectrum.alarms[0].class, AlarmClass::Explosive);
        assert!(
            spectrum.alarms[0].confidence > 0.5,
            "confidence = {}",
            spectrum.alarms[0].confidence
        );
    }

    #[test]
    fn test_alarm_generation_cwa() {
        let config = ImsConfig::default();
        let mut proc = ImsProcessor::new(config);
        let td_gb = proc.expected_drift_time(1.61);

        let (times, intensities) =
            make_spectrum_correct(2000, 30.0, &[(td_gb, 500.0, 0.25)]);
        let spectrum = proc.process_spectrum(&times, &intensities, IonPolarity::Positive);
        let cwa_alarm = spectrum.alarms.iter().find(|a| a.class == AlarmClass::Cwa);
        assert!(cwa_alarm.is_some(), "should raise CWA alarm for GB/Sarin");
    }

    #[test]
    fn test_no_alarm_for_rip() {
        // RIP is always present and should NOT raise a threat alarm
        let config = ImsConfig::default();
        let mut proc = ImsProcessor::new(config);
        let td_rip = proc.expected_drift_time(2.22);

        let (times, intensities) =
            make_spectrum_correct(2000, 30.0, &[(td_rip, 2000.0, 0.2)]);
        let spectrum = proc.process_spectrum(&times, &intensities, IonPolarity::Positive);
        // RIP is classified as Tic, which is filtered out of alarms
        assert!(
            spectrum.alarms.is_empty(),
            "RIP should not trigger an alarm, got {} alarms",
            spectrum.alarms.len()
        );
    }

    #[test]
    fn test_dual_polarity() {
        // TNT should only be found in negative mode
        let config = ImsConfig::default();
        let mut proc = ImsProcessor::new(config);
        let td_tnt = proc.expected_drift_time(1.54);

        let (times, intensities) =
            make_spectrum_correct(2000, 30.0, &[(td_tnt, 800.0, 0.25)]);

        let spec_neg = proc.process_spectrum(&times, &intensities, IonPolarity::Negative);
        let spec_pos = proc.process_spectrum(&times, &intensities, IonPolarity::Positive);

        let tnt_neg = spec_neg
            .peaks
            .iter()
            .any(|p| p.identification.as_ref().map_or(false, |n| n.contains("TNT")));
        let tnt_pos = spec_pos
            .peaks
            .iter()
            .any(|p| p.identification.as_ref().map_or(false, |n| n.contains("TNT")));

        assert!(tnt_neg, "TNT should be found in negative mode");
        assert!(!tnt_pos, "TNT should NOT be found in positive mode");
    }

    // -----------------------------------------------------------------------
    // 8. Background subtraction
    // -----------------------------------------------------------------------

    #[test]
    fn test_background_subtraction_cleans_rip() {
        let config = ImsConfig::default();
        let mut proc = ImsProcessor::new(config);

        // Background: just RIP
        let td_rip = proc.expected_drift_time(2.22);
        let (times, bg) = make_spectrum_correct(2000, 30.0, &[(td_rip, 2000.0, 0.2)]);

        // Signal: RIP + TNT
        let td_tnt = proc.expected_drift_time(1.54);
        let (_, signal) = make_spectrum_correct(
            2000,
            30.0,
            &[(td_rip, 2000.0, 0.2), (td_tnt, 500.0, 0.25)],
        );

        proc.set_background(bg);
        let spectrum = proc.process_spectrum(&times, &signal, IonPolarity::Negative);

        // After background subtraction, the RIP peak should be diminished
        // and the TNT peak should remain
        let has_tnt = spectrum
            .peaks
            .iter()
            .any(|p| p.identification.as_ref().map_or(false, |n| n.contains("TNT")));
        assert!(has_tnt, "TNT peak should survive background subtraction");
    }

    // -----------------------------------------------------------------------
    // 9. Spectrum averaging
    // -----------------------------------------------------------------------

    #[test]
    fn test_spectrum_averaging() {
        let mut config = ImsConfig::default();
        config.smoothing_window = 0;
        let mut proc = ImsProcessor::new(config);
        proc.set_averaging(3);

        let n = 200;
        let dt = 30.0 / n as f64;
        let times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();

        // Feed 3 identical spectra
        let intensities = vec![100.0; n];
        for _ in 0..3 {
            let _ = proc.process_spectrum(&times, &intensities, IonPolarity::Positive);
        }
        // The averaged spectrum should still be ~100
        let spectrum = proc.process_spectrum(&times, &intensities, IonPolarity::Positive);
        let mean: f64 = spectrum.intensities.iter().sum::<f64>() / spectrum.intensities.len() as f64;
        // After baseline subtraction, values near edges may differ, but interior should be low
        // (flat signal → baseline ≈ signal → subtracted ≈ 0)
        // This is expected behaviour: a flat signal has no peaks
        assert!(spectrum.peaks.is_empty(), "flat signal should have no peaks");
    }

    // -----------------------------------------------------------------------
    // 10. Detection limit
    // -----------------------------------------------------------------------

    #[test]
    fn test_detection_limit_snr() {
        // Peak=100, noise_rms=10 → SNR=10 → detection limit fraction = 3/10 = 0.3
        let dl = detection_limit_snr(100.0, 10.0);
        assert!((dl - 0.3).abs() < 1e-10, "dl = {}", dl);
    }

    #[test]
    fn test_detection_limit_zero_peak() {
        let dl = detection_limit_snr(0.0, 10.0);
        assert!(dl.is_infinite());
    }

    // -----------------------------------------------------------------------
    // 11. Deconvolution
    // -----------------------------------------------------------------------

    #[test]
    fn test_deconvolution_single_peak() {
        let (times, intensities) = make_spectrum_correct(1000, 30.0, &[(10.0, 500.0, 0.3)]);
        // Use a threshold high enough to avoid picking up fitting residuals
        let result = deconvolve_peaks(&times, &intensities, 50.0, 5);
        assert_eq!(result.components.len(), 1, "found {} components", result.components.len());
        assert!((result.components[0].1 - 10.0).abs() < 0.5);
    }

    #[test]
    fn test_deconvolution_two_overlapping_peaks() {
        // Two close peaks that partially overlap
        let (times, intensities) = make_spectrum_correct(
            2000,
            30.0,
            &[(10.0, 500.0, 0.3), (11.0, 400.0, 0.3)],
        );
        let result = deconvolve_peaks(&times, &intensities, 50.0, 5);
        assert!(
            result.components.len() >= 2,
            "should find at least 2 components, got {}",
            result.components.len()
        );
    }

    #[test]
    fn test_deconvolution_residual() {
        let (times, intensities) = make_spectrum_correct(1000, 30.0, &[(10.0, 500.0, 0.3)]);
        let result = deconvolve_peaks(&times, &intensities, 10.0, 5);
        // Residual should be much smaller than original peak
        assert!(
            result.residual_rms < 50.0,
            "residual_rms = {}",
            result.residual_rms
        );
    }

    // -----------------------------------------------------------------------
    // 12. Gaussian fitting
    // -----------------------------------------------------------------------

    #[test]
    fn test_gaussian_function() {
        let val = gaussian(5.0, 100.0, 5.0, 1.0);
        assert!((val - 100.0).abs() < 1e-10, "peak should be 100 at centre");

        let val2 = gaussian(5.0 + 1.0, 100.0, 5.0, 1.0);
        let expected = 100.0 * (-0.5_f64).exp();
        assert!(
            (val2 - expected).abs() < 1e-8,
            "1σ from centre: {} vs {}",
            val2,
            expected
        );
    }

    #[test]
    fn test_fit_gaussian_peak_accuracy() {
        let n = 1000;
        let dt = 20.0 / n as f64;
        let times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        let sigma = 0.5;
        let centre = 10.0;
        let amp = 200.0;
        let intensities: Vec<f64> = times
            .iter()
            .map(|&t| gaussian(t, amp, centre, sigma))
            .collect();

        // Find peak index
        let peak_idx = intensities
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        let (fit_amp, fit_centre, fit_sigma) = fit_gaussian_peak(&times, &intensities, peak_idx);
        assert!(
            (fit_amp - amp).abs() < 5.0,
            "amp: {} vs {}",
            fit_amp,
            amp
        );
        assert!(
            (fit_centre - centre).abs() < 0.1,
            "centre: {} vs {}",
            fit_centre,
            centre
        );
        assert!(
            (fit_sigma - sigma).abs() < 0.1,
            "sigma: {} vs {}",
            fit_sigma,
            sigma
        );
    }

    // -----------------------------------------------------------------------
    // 13. Processor configuration
    // -----------------------------------------------------------------------

    #[test]
    fn test_processor_expected_drift_time() {
        let config = ImsConfig::default();
        let proc = ImsProcessor::new(config);
        // RIP+ K0=2.22 should have a reasonable drift time
        let td = proc.expected_drift_time(2.22);
        assert!(td > 0.0 && td < 50.0, "td = {} ms", td);
    }

    #[test]
    fn test_processor_theoretical_resolving_power() {
        let config = ImsConfig::default();
        let proc = ImsProcessor::new(config);
        let r = proc.theoretical_resolving_power();
        assert!(r > 10.0 && r < 500.0, "R = {}", r);
    }

    #[test]
    fn test_default_analyte_db_entries() {
        let db = default_analyte_db();
        assert!(db.len() >= 10, "should have >=10 entries, got {}", db.len());

        // Check specific entries exist
        let has_tnt = db.iter().any(|e| e.name.contains("TNT"));
        let has_cocaine = db.iter().any(|e| e.name.contains("Cocaine"));
        let has_gb = db.iter().any(|e| e.name.contains("GB"));
        assert!(has_tnt);
        assert!(has_cocaine);
        assert!(has_gb);
    }

    #[test]
    fn test_custom_analyte_db() {
        let mut proc = ImsProcessor::new(ImsConfig::default());
        let custom_db = vec![AnalyteEntry {
            name: "TestSubstance",
            k0: 1.80,
            polarity: IonPolarity::Positive,
            alarm_class: AlarmClass::Tic,
        }];
        proc.set_analyte_db(custom_db);
        let td = proc.expected_drift_time(1.80);
        let (times, intensities) =
            make_spectrum_correct(2000, 30.0, &[(td, 500.0, 0.25)]);
        let spectrum = proc.process_spectrum(&times, &intensities, IonPolarity::Positive);
        let found = spectrum.peaks.iter().any(|p| {
            p.identification
                .as_ref()
                .map_or(false, |n| n.contains("TestSubstance"))
        });
        assert!(found, "custom substance should be identified");
    }

    // -----------------------------------------------------------------------
    // 14. Edge cases
    // -----------------------------------------------------------------------

    #[test]
    fn test_empty_spectrum() {
        let mut proc = ImsProcessor::new(ImsConfig::default());
        let times: Vec<f64> = Vec::new();
        let intensities: Vec<f64> = Vec::new();
        // Should not panic
        let spectrum = proc.process_spectrum(&times, &intensities, IonPolarity::Positive);
        assert!(spectrum.peaks.is_empty());
    }

    #[test]
    fn test_flat_spectrum_no_peaks() {
        let n = 500;
        let dt = 30.0 / n as f64;
        let times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        let intensities = vec![100.0; n];
        let mut proc = ImsProcessor::new(ImsConfig::default());
        let spectrum = proc.process_spectrum(&times, &intensities, IonPolarity::Positive);
        // Flat spectrum: after baseline subtraction, should have ~0 and no peaks
        assert!(
            spectrum.peaks.is_empty(),
            "flat spectrum should not have peaks"
        );
    }

    #[test]
    fn test_high_temperature_shorter_drift_time() {
        // Higher temperature → higher actual K → shorter drift time
        let config_cold = ImsConfig {
            temperature_k: 300.0,
            ..ImsConfig::default()
        };
        let config_hot = ImsConfig {
            temperature_k: 500.0,
            ..ImsConfig::default()
        };
        let proc_cold = ImsProcessor::new(config_cold);
        let proc_hot = ImsProcessor::new(config_hot);

        let td_cold = proc_cold.expected_drift_time(1.54);
        let td_hot = proc_hot.expected_drift_time(1.54);
        assert!(
            td_hot < td_cold,
            "hot drift time {} should be < cold {}",
            td_hot,
            td_cold
        );
    }

    #[test]
    fn test_high_pressure_longer_drift_time() {
        // Higher pressure → lower actual K → longer drift time
        let config_lo = ImsConfig {
            pressure_pa: 80000.0,
            ..ImsConfig::default()
        };
        let config_hi = ImsConfig {
            pressure_pa: 120000.0,
            ..ImsConfig::default()
        };
        let proc_lo = ImsProcessor::new(config_lo);
        let proc_hi = ImsProcessor::new(config_hi);

        let td_lo = proc_lo.expected_drift_time(1.54);
        let td_hi = proc_hi.expected_drift_time(1.54);
        assert!(
            td_hi > td_lo,
            "high-P drift time {} should be > low-P {}",
            td_hi,
            td_lo
        );
    }
}
