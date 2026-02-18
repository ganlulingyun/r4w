//! Ion Mobility Spectrometry (IMS) Signal Processing
//!
//! Implements signal processing for gas-phase ion separation and identification.
//! IMS separates ions by their mobility in an electric field through a drift gas,
//! producing a spectrum of drift time vs. intensity (plasmagram).
//!
//! This module uses traditional IMS units (cm, V/cm, Torr) and provides:
//!
//! - **Drift time to reduced mobility** conversion with temperature/pressure normalization
//! - **Mason-Schamp equation** for collision cross-section (CCS) computation
//! - **Peak detection** with Gaussian fitting and resolving power measurement
//! - **Baseline correction** via rolling minimum or polynomial subtraction
//! - **Spectrum smoothing** (moving average and Savitzky-Golay)
//! - **Reactant Ion Peak (RIP)** identification and characterization
//! - **Analyte database** with K0 values for explosives, CWAs, and narcotics
//! - **Fourier Transform IMS (FT-IMS)** twin-gate simulation with FFT
//! - **Multi-peak deconvolution** using iterative Gaussian fitting
//! - **SNR estimation** and alarm threshold comparison
//! - **CCS database** for calibration
//!
//! # Example
//!
//! ```rust
//! use r4w_core::ion_mobility_spectrometry_processor::{
//!     ImsConfig, ImsProcessor, ImsSpectrum, DriftGas,
//! };
//!
//! let config = ImsConfig {
//!     drift_tube_length_cm: 10.0,
//!     electric_field_vcm: 250.0,
//!     drift_gas: DriftGas::N2,
//!     temperature_k: 373.15,
//!     pressure_torr: 760.0,
//!     gate_pulse_width_us: 200.0,
//! };
//!
//! let proc = ImsProcessor::new(config);
//!
//! // Convert a measured drift time to reduced mobility
//! let k0 = proc.drift_time_to_mobility(8.5);
//! assert!(k0 > 0.5 && k0 < 5.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Boltzmann constant (J/K)
const K_BOLTZ: f64 = 1.380649e-23;

/// Elementary charge (C)
const ELEM_CHARGE: f64 = 1.602176634e-19;

/// Avogadro's number (1/mol)
const AVOGADRO: f64 = 6.02214076e23;

/// Standard temperature for reduced mobility (K)
const T_STD: f64 = 273.15;

/// Standard pressure for reduced mobility (Torr)
const P_STD: f64 = 760.0;

/// Atomic mass unit (kg)
const AMU: f64 = 1.66053906660e-27;

/// ln(2)
const LN2: f64 = 0.693_147_180_559_945_3;

/// sqrt(2 * ln(2)) -- for FWHM/sigma conversion: FWHM = 2 * sqrt(2*ln(2)) * sigma
const SQRT_2LN2: f64 = 1.177_410_022_515_475;

// ---------------------------------------------------------------------------
// Drift gas
// ---------------------------------------------------------------------------

/// Drift gas species with associated physical properties.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DriftGas {
    /// Nitrogen (N2) -- most common drift gas
    N2,
    /// Helium (He) -- high-resolution IMS
    He,
    /// Air (approx. 78% N2 + 21% O2 + 1% Ar)
    Air,
    /// Carbon dioxide (CO2) -- sometimes used for negative-mode selectivity
    CO2,
}

impl DriftGas {
    /// Molecular weight in atomic mass units (g/mol).
    pub fn molecular_weight(&self) -> f64 {
        match self {
            DriftGas::N2 => 28.014,
            DriftGas::He => 4.0026,
            DriftGas::Air => 28.97,
            DriftGas::CO2 => 44.01,
        }
    }

    /// Molecular weight in kg.
    pub fn mass_kg(&self) -> f64 {
        self.molecular_weight() * AMU
    }

    /// Polarizability in angstrom^3 (10^-30 m^3).
    pub fn polarizability_a3(&self) -> f64 {
        match self {
            DriftGas::N2 => 1.740,
            DriftGas::He => 0.205,
            DriftGas::Air => 1.710,
            DriftGas::CO2 => 2.911,
        }
    }

    /// Polarizability in SI units (m^3).
    pub fn polarizability_m3(&self) -> f64 {
        self.polarizability_a3() * 1e-30
    }
}

// ---------------------------------------------------------------------------
// IMS configuration
// ---------------------------------------------------------------------------

/// Configuration for the IMS instrument in traditional IMS units.
#[derive(Debug, Clone)]
pub struct ImsConfig {
    /// Drift tube length in centimetres (typical: 5 -- 15 cm).
    pub drift_tube_length_cm: f64,
    /// Electric field strength in V/cm (typical: 200 -- 400 V/cm).
    pub electric_field_vcm: f64,
    /// Drift gas species.
    pub drift_gas: DriftGas,
    /// Drift tube temperature in kelvin (typical: 323 -- 523 K).
    pub temperature_k: f64,
    /// Drift tube pressure in Torr (typical: 700 -- 800 Torr).
    pub pressure_torr: f64,
    /// Ion gate pulse width in microseconds (typical: 100 -- 300 us).
    pub gate_pulse_width_us: f64,
}

impl Default for ImsConfig {
    fn default() -> Self {
        Self {
            drift_tube_length_cm: 10.0,
            electric_field_vcm: 250.0,
            drift_gas: DriftGas::N2,
            temperature_k: 373.15,       // 100 deg C
            pressure_torr: 760.0,        // 1 atm
            gate_pulse_width_us: 200.0,
        }
    }
}

// ---------------------------------------------------------------------------
// IMS spectrum
// ---------------------------------------------------------------------------

/// A raw or processed IMS spectrum (plasmagram).
#[derive(Debug, Clone)]
pub struct ImsSpectrum {
    /// Drift times in milliseconds.
    pub drift_times: Vec<f64>,
    /// Intensity values (arbitrary units).
    pub intensities: Vec<f64>,
}

impl ImsSpectrum {
    /// Create a new spectrum from matched drift-time and intensity vectors.
    pub fn new(drift_times: Vec<f64>, intensities: Vec<f64>) -> Self {
        assert_eq!(
            drift_times.len(),
            intensities.len(),
            "drift_times and intensities must have equal length"
        );
        Self {
            drift_times,
            intensities,
        }
    }

    /// Number of data points.
    pub fn len(&self) -> usize {
        self.drift_times.len()
    }

    /// Whether the spectrum is empty.
    pub fn is_empty(&self) -> bool {
        self.drift_times.is_empty()
    }
}

// ---------------------------------------------------------------------------
// Peak result
// ---------------------------------------------------------------------------

/// A detected peak in an IMS spectrum.
#[derive(Debug, Clone)]
pub struct ImsPeak {
    /// Drift time of peak centre (ms).
    pub drift_time_ms: f64,
    /// Peak intensity (arbitrary units).
    pub intensity: f64,
    /// Full width at half maximum (ms).
    pub fwhm_ms: f64,
    /// Resolving power R = td / FWHM.
    pub resolving_power: f64,
    /// Reduced ion mobility K0 (cm^2 / V*s).
    pub k0: f64,
    /// Gaussian sigma parameter (ms).
    pub sigma_ms: f64,
    /// Peak area (intensity * ms).
    pub area: f64,
}

// ---------------------------------------------------------------------------
// Analyte database
// ---------------------------------------------------------------------------

/// A known analyte with its reduced mobility value.
#[derive(Debug, Clone)]
pub struct AnalyteRecord {
    /// Human-readable name.
    pub name: &'static str,
    /// Reduced mobility K0 in cm^2/(V*s).
    pub k0: f64,
    /// Category (explosive, CWA, narcotic, general).
    pub category: &'static str,
    /// Ion polarity mode (positive or negative).
    pub positive_mode: bool,
}

/// CCS calibrant entry.
#[derive(Debug, Clone)]
pub struct CcsRecord {
    /// Human-readable name.
    pub name: &'static str,
    /// Collision cross-section in angstrom^2.
    pub ccs_a2: f64,
    /// Reduced mobility K0 in cm^2/(V*s).
    pub k0: f64,
    /// Molecular weight in amu (Da).
    pub mw_da: f64,
}

/// Build the default analyte K0 database.
pub fn default_analyte_database() -> Vec<AnalyteRecord> {
    vec![
        // Explosives (negative mode)
        AnalyteRecord { name: "TNT", k0: 1.54, category: "explosive", positive_mode: false },
        AnalyteRecord { name: "RDX", k0: 1.39, category: "explosive", positive_mode: false },
        AnalyteRecord { name: "PETN", k0: 1.24, category: "explosive", positive_mode: false },
        AnalyteRecord { name: "HMX", k0: 1.32, category: "explosive", positive_mode: false },
        AnalyteRecord { name: "Nitroglycerin", k0: 1.46, category: "explosive", positive_mode: false },
        AnalyteRecord { name: "EGDN", k0: 1.58, category: "explosive", positive_mode: false },
        AnalyteRecord { name: "DNT", k0: 1.62, category: "explosive", positive_mode: false },
        AnalyteRecord { name: "TATP", k0: 1.36, category: "explosive", positive_mode: true },
        // CWAs (positive mode)
        AnalyteRecord { name: "GA (Tabun)", k0: 1.46, category: "CWA", positive_mode: true },
        AnalyteRecord { name: "GB (Sarin)", k0: 1.61, category: "CWA", positive_mode: true },
        AnalyteRecord { name: "GD (Soman)", k0: 1.34, category: "CWA", positive_mode: true },
        AnalyteRecord { name: "VX", k0: 1.22, category: "CWA", positive_mode: true },
        AnalyteRecord { name: "HD (Mustard)", k0: 1.48, category: "CWA", positive_mode: true },
        AnalyteRecord { name: "Lewisite", k0: 1.42, category: "CWA", positive_mode: true },
        // Narcotics (positive mode)
        AnalyteRecord { name: "Cocaine", k0: 1.16, category: "narcotic", positive_mode: true },
        AnalyteRecord { name: "Methamphetamine", k0: 1.63, category: "narcotic", positive_mode: true },
        AnalyteRecord { name: "Heroin", k0: 1.04, category: "narcotic", positive_mode: true },
        AnalyteRecord { name: "MDMA", k0: 1.44, category: "narcotic", positive_mode: true },
        AnalyteRecord { name: "THC", k0: 1.28, category: "narcotic", positive_mode: true },
        AnalyteRecord { name: "Fentanyl", k0: 1.08, category: "narcotic", positive_mode: true },
        // RIP markers
        AnalyteRecord { name: "RIP+ (H3O+)", k0: 2.22, category: "RIP", positive_mode: true },
        AnalyteRecord { name: "RIP- (O2-)", k0: 2.18, category: "RIP", positive_mode: false },
    ]
}

/// Build a CCS calibrant database with common ions.
pub fn collision_cross_section_database() -> Vec<CcsRecord> {
    vec![
        // Tetraalkylammonium salts -- common CCS calibrants
        CcsRecord { name: "TMA+ (tetramethylammonium)", ccs_a2: 130.0, k0: 2.01, mw_da: 74.1 },
        CcsRecord { name: "TEA+ (tetraethylammonium)", ccs_a2: 164.0, k0: 1.66, mw_da: 130.2 },
        CcsRecord { name: "TPA+ (tetrapropylammonium)", ccs_a2: 208.0, k0: 1.38, mw_da: 186.3 },
        CcsRecord { name: "TBA+ (tetrabutylammonium)", ccs_a2: 252.0, k0: 1.15, mw_da: 242.5 },
        CcsRecord { name: "THA+ (tetrahexylammonium)", ccs_a2: 332.0, k0: 0.92, mw_da: 354.6 },
        // Small ions
        CcsRecord { name: "K+", ccs_a2: 65.0, k0: 2.65, mw_da: 39.1 },
        CcsRecord { name: "Na+", ccs_a2: 55.0, k0: 2.80, mw_da: 23.0 },
        CcsRecord { name: "NH4+", ccs_a2: 70.0, k0: 2.52, mw_da: 18.0 },
        // Drug-like molecules
        CcsRecord { name: "Caffeine [M+H]+", ccs_a2: 145.0, k0: 1.58, mw_da: 195.2 },
        CcsRecord { name: "Cocaine [M+H]+", ccs_a2: 180.0, k0: 1.16, mw_da: 303.4 },
    ]
}

// ---------------------------------------------------------------------------
// Alarm result
// ---------------------------------------------------------------------------

/// Result of comparing a peak against detection thresholds.
#[derive(Debug, Clone)]
pub struct AlarmResult {
    /// Name of the matched analyte.
    pub analyte: String,
    /// Measured K0 (cm^2/V*s).
    pub measured_k0: f64,
    /// Expected K0 from database.
    pub expected_k0: f64,
    /// K0 match error (absolute).
    pub k0_error: f64,
    /// Peak intensity.
    pub peak_intensity: f64,
    /// Whether the alarm threshold was exceeded.
    pub triggered: bool,
}

// ---------------------------------------------------------------------------
// Gaussian helper
// ---------------------------------------------------------------------------

/// Evaluate a Gaussian function: A * exp(-(x-mu)^2 / (2*sigma^2))
fn gaussian(x: f64, amplitude: f64, mu: f64, sigma: f64) -> f64 {
    if sigma.abs() < 1e-30 {
        return 0.0;
    }
    amplitude * (-((x - mu).powi(2)) / (2.0 * sigma * sigma)).exp()
}

// ---------------------------------------------------------------------------
// IMS Processor
// ---------------------------------------------------------------------------

/// Main IMS signal processor.
///
/// Converts between drift times and mobilities, detects peaks,
/// identifies analytes from a K0 database, and supports FT-IMS simulation.
#[derive(Debug, Clone)]
pub struct ImsProcessor {
    config: ImsConfig,
    analyte_db: Vec<AnalyteRecord>,
}

impl ImsProcessor {
    /// Create a new IMS processor with the given configuration.
    pub fn new(config: ImsConfig) -> Self {
        Self {
            analyte_db: default_analyte_database(),
            config,
        }
    }

    /// Access the current configuration.
    pub fn config(&self) -> &ImsConfig {
        &self.config
    }

    /// Set a custom analyte database.
    pub fn set_analyte_db(&mut self, db: Vec<AnalyteRecord>) {
        self.analyte_db = db;
    }

    // -------------------------------------------------------------------
    // Core mobility conversions
    // -------------------------------------------------------------------

    /// Convert drift time (ms) to reduced ion mobility K0 (cm^2 / V*s).
    ///
    /// K0 = (L^2 / (V * td)) * (T_std / T) * (P / P_std)
    ///
    /// where L is tube length (cm), V = E * L is drift voltage (V),
    /// td is drift time (s), T_std = 273.15 K, P_std = 760 Torr.
    pub fn drift_time_to_mobility(&self, td_ms: f64) -> f64 {
        if td_ms.abs() < 1e-30 {
            return 0.0;
        }
        let l_cm = self.config.drift_tube_length_cm;
        let v_volts = self.config.electric_field_vcm * l_cm; // drift voltage
        let td_s = td_ms * 1e-3;

        let k = (l_cm * l_cm) / (v_volts * td_s); // cm^2/(V*s) in raw (non-reduced)
        // Reduce to standard conditions
        k * (T_STD / self.config.temperature_k) * (self.config.pressure_torr / P_STD)
    }

    /// Convert reduced mobility K0 (cm^2/V*s) to expected drift time (ms).
    pub fn mobility_to_drift_time(&self, k0: f64) -> f64 {
        if k0.abs() < 1e-30 {
            return 0.0;
        }
        // K_actual = K0 * (T / T_std) * (P_std / P)
        let k_actual = k0 * (self.config.temperature_k / T_STD) * (P_STD / self.config.pressure_torr);
        let l_cm = self.config.drift_tube_length_cm;
        let v_volts = self.config.electric_field_vcm * l_cm;

        let td_s = (l_cm * l_cm) / (k_actual * v_volts);
        td_s * 1e3 // seconds to ms
    }

    /// Convert reduced mobility K0 to collision cross-section (CCS) in angstrom^2
    /// using the Mason-Schamp equation.
    ///
    /// CCS = (3*e / (16*N)) * sqrt(2*PI / (mu * kB * T)) * (1/K)
    ///
    /// where mu is the reduced mass of ion and buffer gas, N is number density,
    /// and K is the actual (non-reduced) mobility.
    pub fn mobility_to_ccs(&self, k0: f64, ion_mass_amu: f64) -> f64 {
        if k0.abs() < 1e-30 || ion_mass_amu <= 0.0 {
            return 0.0;
        }

        let gas_mass_amu = self.config.drift_gas.molecular_weight();
        let mu_amu = (ion_mass_amu * gas_mass_amu) / (ion_mass_amu + gas_mass_amu);
        let mu_kg = mu_amu * AMU;

        // Number density at standard conditions (for K0)
        // N at STP: P_std / (kB * T_std), but P must be in Pa
        // 760 Torr = 101325 Pa, T_std = 273.15 K
        let p_pa = P_STD * 101325.0 / 760.0; // = 101325 Pa
        let n = p_pa / (K_BOLTZ * T_STD); // number density at STP

        // K0 is in cm^2/(V*s) -> convert to m^2/(V*s)
        let k0_m2 = k0 * 1e-4;

        let prefactor = (3.0 * ELEM_CHARGE) / (16.0 * n);
        let thermal = (2.0 * PI / (mu_kg * K_BOLTZ * T_STD)).sqrt();

        let ccs_m2 = prefactor * thermal / k0_m2;
        // Convert m^2 to angstrom^2 (1 m = 1e10 A)
        ccs_m2 * 1e20
    }

    /// Convert CCS (angstrom^2) back to reduced mobility K0 (cm^2/V*s).
    pub fn ccs_to_mobility(&self, ccs_a2: f64, ion_mass_amu: f64) -> f64 {
        if ccs_a2 <= 0.0 || ion_mass_amu <= 0.0 {
            return 0.0;
        }

        let gas_mass_amu = self.config.drift_gas.molecular_weight();
        let mu_amu = (ion_mass_amu * gas_mass_amu) / (ion_mass_amu + gas_mass_amu);
        let mu_kg = mu_amu * AMU;

        let p_pa = 101325.0;
        let n = p_pa / (K_BOLTZ * T_STD);

        let ccs_m2 = ccs_a2 * 1e-20;

        let prefactor = (3.0 * ELEM_CHARGE) / (16.0 * n);
        let thermal = (2.0 * PI / (mu_kg * K_BOLTZ * T_STD)).sqrt();

        let k0_m2 = prefactor * thermal / ccs_m2;
        k0_m2 * 1e4 // m^2/(V*s) -> cm^2/(V*s)
    }

    // -------------------------------------------------------------------
    // Peak detection
    // -------------------------------------------------------------------

    /// Detect peaks in an IMS spectrum.
    ///
    /// Finds local maxima above `min_intensity` with Gaussian fitting
    /// to determine FWHM, resolving power, and K0.
    pub fn peak_detect(&self, spectrum: &ImsSpectrum, min_intensity: f64) -> Vec<ImsPeak> {
        let n = spectrum.len();
        if n < 3 {
            return Vec::new();
        }

        let times = &spectrum.drift_times;
        let data = &spectrum.intensities;

        let mut peaks = Vec::new();

        for i in 1..n - 1 {
            if data[i] > min_intensity
                && data[i] >= data[i - 1]
                && data[i] >= data[i + 1]
            {
                // Avoid double-counting nearby samples
                let too_close = peaks.last().map_or(false, |p: &ImsPeak| {
                    (times[i] - p.drift_time_ms).abs() < 0.1
                });
                if too_close {
                    continue;
                }

                let (amp, centre, sigma) = self.fit_gaussian_at(times, data, i);
                let fwhm = sigma * 2.0 * SQRT_2LN2;
                let rp = if fwhm > 1e-12 { centre / fwhm } else { 0.0 };
                let k0 = self.drift_time_to_mobility(centre);
                let area = self.peak_area(times, data, centre, sigma);

                peaks.push(ImsPeak {
                    drift_time_ms: centre,
                    intensity: amp,
                    fwhm_ms: fwhm,
                    resolving_power: rp,
                    k0,
                    sigma_ms: sigma,
                    area,
                });
            }
        }

        peaks
    }

    /// Compute resolving power for a given drift time and FWHM.
    pub fn resolving_power(td_ms: f64, fwhm_ms: f64) -> f64 {
        if fwhm_ms.abs() < 1e-15 {
            return 0.0;
        }
        td_ms / fwhm_ms
    }

    // -------------------------------------------------------------------
    // Spectrum generation and manipulation
    // -------------------------------------------------------------------

    /// Generate a Gaussian IMS peak for simulation.
    ///
    /// Returns an `ImsSpectrum` containing a single Gaussian peak at `centre_ms`
    /// with the given amplitude and sigma.
    pub fn generate_gaussian_peak(
        &self,
        n_points: usize,
        t_max_ms: f64,
        centre_ms: f64,
        amplitude: f64,
        sigma_ms: f64,
    ) -> ImsSpectrum {
        let dt = t_max_ms / n_points as f64;
        let drift_times: Vec<f64> = (0..n_points).map(|i| i as f64 * dt).collect();
        let intensities: Vec<f64> = drift_times
            .iter()
            .map(|&t| gaussian(t, amplitude, centre_ms, sigma_ms))
            .collect();
        ImsSpectrum { drift_times, intensities }
    }

    /// Generate a multi-peak synthetic spectrum.
    pub fn generate_multi_peak(
        &self,
        n_points: usize,
        t_max_ms: f64,
        peaks: &[(f64, f64, f64)], // (centre_ms, amplitude, sigma_ms)
    ) -> ImsSpectrum {
        let dt = t_max_ms / n_points as f64;
        let drift_times: Vec<f64> = (0..n_points).map(|i| i as f64 * dt).collect();
        let mut intensities = vec![0.0; n_points];
        for &(centre, amp, sigma) in peaks {
            for (i, &t) in drift_times.iter().enumerate() {
                intensities[i] += gaussian(t, amp, centre, sigma);
            }
        }
        ImsSpectrum { drift_times, intensities }
    }

    /// Baseline correction using rolling minimum subtraction.
    ///
    /// A sliding window finds the local minimum, which is then subtracted
    /// from the spectrum. Effective for slowly varying baselines.
    pub fn baseline_correction(&self, spectrum: &ImsSpectrum, window: usize) -> ImsSpectrum {
        let n = spectrum.len();
        if n == 0 || window <= 1 {
            return spectrum.clone();
        }
        let half = window / 2;
        let data = &spectrum.intensities;
        let mut baseline = vec![0.0; n];

        for i in 0..n {
            let lo = if i >= half { i - half } else { 0 };
            let hi = (i + half).min(n - 1);
            let mut min_val = f64::MAX;
            for j in lo..=hi {
                if data[j] < min_val {
                    min_val = data[j];
                }
            }
            baseline[i] = min_val;
        }

        let corrected: Vec<f64> = data
            .iter()
            .zip(baseline.iter())
            .map(|(&s, &b)| (s - b).max(0.0))
            .collect();

        ImsSpectrum {
            drift_times: spectrum.drift_times.clone(),
            intensities: corrected,
        }
    }

    /// Baseline correction using polynomial fit subtraction.
    ///
    /// Fits a polynomial of degree `order` to the spectrum and subtracts it.
    /// Order 1 = linear, 2 = quadratic, etc.
    pub fn baseline_correction_poly(&self, spectrum: &ImsSpectrum, order: usize) -> ImsSpectrum {
        let n = spectrum.len();
        if n < order + 1 {
            return spectrum.clone();
        }

        let coeffs = poly_fit(&spectrum.drift_times, &spectrum.intensities, order);
        let baseline: Vec<f64> = spectrum
            .drift_times
            .iter()
            .map(|&t| poly_eval(&coeffs, t))
            .collect();

        let corrected: Vec<f64> = spectrum
            .intensities
            .iter()
            .zip(baseline.iter())
            .map(|(&s, &b)| (s - b).max(0.0))
            .collect();

        ImsSpectrum {
            drift_times: spectrum.drift_times.clone(),
            intensities: corrected,
        }
    }

    /// Normalize spectrum so that the maximum intensity equals 1.0.
    pub fn normalize_spectrum(&self, spectrum: &ImsSpectrum) -> ImsSpectrum {
        let max_val = spectrum
            .intensities
            .iter()
            .cloned()
            .fold(0.0_f64, f64::max);
        if max_val < 1e-30 {
            return spectrum.clone();
        }
        let normalized: Vec<f64> = spectrum.intensities.iter().map(|&v| v / max_val).collect();
        ImsSpectrum {
            drift_times: spectrum.drift_times.clone(),
            intensities: normalized,
        }
    }

    /// Smooth a spectrum using a moving average filter.
    pub fn smooth_spectrum(&self, spectrum: &ImsSpectrum, window: usize) -> ImsSpectrum {
        if window <= 1 || spectrum.is_empty() {
            return spectrum.clone();
        }
        let n = spectrum.len();
        let half = window / 2;
        let data = &spectrum.intensities;
        let mut smoothed = vec![0.0; n];

        for i in 0..n {
            let lo = if i >= half { i - half } else { 0 };
            let hi = (i + half).min(n - 1);
            let count = (hi - lo + 1) as f64;
            let sum: f64 = data[lo..=hi].iter().sum();
            smoothed[i] = sum / count;
        }

        ImsSpectrum {
            drift_times: spectrum.drift_times.clone(),
            intensities: smoothed,
        }
    }

    /// Smooth a spectrum using a Savitzky-Golay filter.
    ///
    /// `half_width` gives a window of 2*half_width+1 points.
    /// `poly_order` is the polynomial degree (must be < window size).
    pub fn smooth_spectrum_sg(
        &self,
        spectrum: &ImsSpectrum,
        half_width: usize,
        poly_order: usize,
    ) -> ImsSpectrum {
        let n = spectrum.len();
        let window = 2 * half_width + 1;
        if n < window || poly_order >= window || half_width == 0 {
            return spectrum.clone();
        }

        let coeffs = sg_coefficients(half_width, poly_order);
        let data = &spectrum.intensities;
        let mut smoothed = vec![0.0; n];

        for i in 0..n {
            if i >= half_width && i + half_width < n {
                let mut sum = 0.0;
                for (j, &c) in coeffs.iter().enumerate() {
                    sum += c * data[i + j - half_width];
                }
                smoothed[i] = sum;
            } else {
                smoothed[i] = data[i];
            }
        }

        ImsSpectrum {
            drift_times: spectrum.drift_times.clone(),
            intensities: smoothed,
        }
    }

    // -------------------------------------------------------------------
    // RIP characterization
    // -------------------------------------------------------------------

    /// Identify and characterize the Reactant Ion Peak (RIP).
    ///
    /// The RIP is the dominant peak at the shortest drift time, produced by
    /// the background reactant ions. In positive mode this is typically
    /// H3O+(H2O)n with K0 ~ 2.22 cm^2/(V*s).
    ///
    /// Returns the index into the peaks vector and K0 of the RIP, or None.
    pub fn reactant_ion_peak(&self, peaks: &[ImsPeak]) -> Option<(usize, f64)> {
        if peaks.is_empty() {
            return None;
        }
        // The RIP is typically the first (shortest drift time) and largest peak
        let rip_idx = peaks
            .iter()
            .enumerate()
            .min_by(|a, b| {
                a.1.drift_time_ms
                    .partial_cmp(&b.1.drift_time_ms)
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
            .map(|(i, _)| i)?;

        Some((rip_idx, peaks[rip_idx].k0))
    }

    // -------------------------------------------------------------------
    // Analyte lookup
    // -------------------------------------------------------------------

    /// Look up an analyte by K0 value within a tolerance.
    ///
    /// Returns the best-matching analyte record, if any.
    pub fn reduced_mobility_lookup(&self, k0: f64, tolerance: f64, positive_mode: bool) -> Option<&AnalyteRecord> {
        self.analyte_db
            .iter()
            .filter(|a| a.positive_mode == positive_mode)
            .filter(|a| (a.k0 - k0).abs() < tolerance)
            .min_by(|a, b| {
                (a.k0 - k0)
                    .abs()
                    .partial_cmp(&(b.k0 - k0).abs())
                    .unwrap_or(std::cmp::Ordering::Equal)
            })
    }

    // -------------------------------------------------------------------
    // Temperature and pressure corrections
    // -------------------------------------------------------------------

    /// Correct a reduced mobility K0 for a change in temperature.
    ///
    /// If K0 was measured at `t_measured_k`, this returns the corrected K0
    /// as if measured at `t_target_k`. For a true reduced mobility this is
    /// an identity (K0 is already normalized), but for drift-gas-dependent
    /// effects, a sqrt(T) correction may apply.
    pub fn temperature_correction(&self, k0: f64, t_measured_k: f64, t_target_k: f64) -> f64 {
        // The reduced mobility includes (T_std/T) normalization, but the
        // Mason-Schamp equation shows K0 has a residual sqrt(T) dependence
        // through the thermal velocity. This correction accounts for that.
        k0 * (t_target_k / t_measured_k).sqrt()
    }

    /// Correct a reduced mobility K0 for a change in pressure.
    ///
    /// Reduced mobility already normalizes for pressure, but this accounts
    /// for non-ideal gas effects at extreme pressures through a linear correction.
    pub fn pressure_correction(&self, k0: f64, p_measured_torr: f64, p_target_torr: f64) -> f64 {
        // For ideal gas the reduced mobility is already pressure-independent.
        // At high pressures, clustering effects cause K0 to decrease slightly.
        // Simple linear model: K0_corr = K0 * (1 + alpha * (P_target - P_measured))
        // with alpha ~ -1e-5 per Torr (small correction).
        let alpha = -1.0e-5;
        k0 * (1.0 + alpha * (p_target_torr - p_measured_torr))
    }

    // -------------------------------------------------------------------
    // FT-IMS twin gate simulation
    // -------------------------------------------------------------------

    /// Simulate Fourier Transform IMS (FT-IMS) with square-wave twin gating.
    ///
    /// In FT-IMS, both the entrance and exit gates are modulated with the
    /// same square wave at varying frequencies. An FFT of the detector
    /// signal vs. gate frequency reveals the drift-time spectrum with
    /// improved duty cycle (Fellgett advantage).
    ///
    /// * `gate_frequencies` -- list of gate modulation frequencies (Hz)
    /// * `k0_ions` -- list of (K0, relative_abundance) for ions in the mixture
    ///
    /// Returns the reconstructed drift-time spectrum.
    pub fn twin_gate_mode(
        &self,
        gate_frequencies: &[f64],
        k0_ions: &[(f64, f64)],
    ) -> ImsSpectrum {
        let n_freq = gate_frequencies.len();
        if n_freq == 0 || k0_ions.is_empty() {
            return ImsSpectrum {
                drift_times: Vec::new(),
                intensities: Vec::new(),
            };
        }

        // Compute the detector signal at each gate frequency.
        // For each ion with drift time td, the transmission through the twin
        // gates at frequency f is T(f) = sinc^2(f * td), but for a square
        // wave it's the Fourier coefficient magnitude.
        // Simplified model: signal(f) = sum_ions [ abundance * cos(2*pi*f*td) ]
        let mut signal = vec![0.0; n_freq];

        for (i, &freq) in gate_frequencies.iter().enumerate() {
            for &(k0, abundance) in k0_ions {
                let td_s = self.mobility_to_drift_time(k0) * 1e-3; // ms -> s
                signal[i] += abundance * (2.0 * PI * freq * td_s).cos();
            }
        }

        // Perform a real DFT to recover the drift-time domain spectrum
        let n_out = n_freq;
        let mut drift_spectrum = vec![0.0; n_out];

        // Frequency step determines the drift-time resolution
        let f_min = gate_frequencies.iter().cloned().fold(f64::MAX, f64::min);
        let f_max = gate_frequencies.iter().cloned().fold(0.0_f64, f64::max);
        let df = if n_freq > 1 {
            (f_max - f_min) / (n_freq - 1) as f64
        } else {
            1.0
        };

        // DFT: discrete cosine transform (signal is real and even)
        let t_max_ms = if df > 0.0 { 1000.0 / df } else { 30.0 };
        let dt_ms = t_max_ms / n_out as f64;

        for k in 0..n_out {
            let t_ms = k as f64 * dt_ms;
            let t_s = t_ms * 1e-3;
            let mut sum = 0.0;
            for (i, &freq) in gate_frequencies.iter().enumerate() {
                sum += signal[i] * (2.0 * PI * freq * t_s).cos();
            }
            drift_spectrum[k] = sum.abs() / n_freq as f64;
        }

        let drift_times: Vec<f64> = (0..n_out).map(|k| k as f64 * dt_ms).collect();

        ImsSpectrum {
            drift_times,
            intensities: drift_spectrum,
        }
    }

    // -------------------------------------------------------------------
    // Multi-peak Gaussian fitting
    // -------------------------------------------------------------------

    /// Deconvolve overlapping peaks using iterative Gaussian subtraction.
    ///
    /// Finds the strongest peak, fits a Gaussian, subtracts it from the
    /// residual, and repeats until no peak exceeds `min_height` or
    /// `max_peaks` is reached.
    ///
    /// Returns fitted Gaussian parameters (centre_ms, amplitude, sigma_ms)
    /// for each component.
    pub fn multi_peak_fit(
        &self,
        spectrum: &ImsSpectrum,
        min_height: f64,
        max_peaks: usize,
    ) -> Vec<(f64, f64, f64)> {
        let n = spectrum.len();
        if n < 3 {
            return Vec::new();
        }

        let times = &spectrum.drift_times;
        let mut residual = spectrum.intensities.clone();
        let mut components = Vec::new();

        for _ in 0..max_peaks {
            // Find maximum in residual
            let (max_idx, max_val) = residual
                .iter()
                .enumerate()
                .fold((0, f64::NEG_INFINITY), |(bi, bv), (i, &v)| {
                    if v > bv { (i, v) } else { (bi, bv) }
                });

            if max_val < min_height {
                break;
            }

            let (amp, centre, sigma) = self.fit_gaussian_at(times, &residual, max_idx);
            if sigma < 1e-15 {
                break;
            }

            // Subtract fitted Gaussian
            for (i, &t) in times.iter().enumerate() {
                residual[i] -= gaussian(t, amp, centre, sigma);
                if residual[i] < 0.0 {
                    residual[i] = 0.0;
                }
            }

            components.push((centre, amp, sigma));
        }

        components
    }

    // -------------------------------------------------------------------
    // SNR estimation
    // -------------------------------------------------------------------

    /// Estimate signal-to-noise ratio for detected peaks.
    ///
    /// Noise is estimated from the baseline region (first 10% and last 10%
    /// of the spectrum where no peaks are expected).
    pub fn snr_estimate(&self, spectrum: &ImsSpectrum, peaks: &[ImsPeak]) -> Vec<f64> {
        let n = spectrum.len();
        if n < 10 {
            return peaks.iter().map(|_| 0.0).collect();
        }

        // Estimate noise RMS from edges of the spectrum
        let edge = n / 10;
        let noise_region: Vec<f64> = spectrum.intensities[..edge]
            .iter()
            .chain(spectrum.intensities[n - edge..].iter())
            .cloned()
            .collect();

        let noise_rms = rms(&noise_region);
        if noise_rms < 1e-30 {
            return peaks.iter().map(|p| if p.intensity > 0.0 { f64::INFINITY } else { 0.0 }).collect();
        }

        peaks.iter().map(|p| p.intensity / noise_rms).collect()
    }

    // -------------------------------------------------------------------
    // Alarm threshold
    // -------------------------------------------------------------------

    /// Compare peak K0 values against the analyte database and intensity thresholds.
    ///
    /// `k0_tolerance` is the maximum K0 difference for a match (cm^2/V*s).
    /// `intensity_threshold` is the minimum peak intensity to trigger an alarm.
    pub fn alarm_threshold(
        &self,
        peaks: &[ImsPeak],
        k0_tolerance: f64,
        intensity_threshold: f64,
        positive_mode: bool,
    ) -> Vec<AlarmResult> {
        let mut results = Vec::new();

        for peak in peaks {
            if let Some(record) = self.reduced_mobility_lookup(peak.k0, k0_tolerance, positive_mode) {
                let triggered = peak.intensity >= intensity_threshold;
                results.push(AlarmResult {
                    analyte: record.name.to_string(),
                    measured_k0: peak.k0,
                    expected_k0: record.k0,
                    k0_error: (peak.k0 - record.k0).abs(),
                    peak_intensity: peak.intensity,
                    triggered,
                });
            }
        }

        results
    }

    // -------------------------------------------------------------------
    // Internal helpers
    // -------------------------------------------------------------------

    /// Fit a Gaussian around a local maximum index.
    /// Returns (amplitude, centre, sigma).
    fn fit_gaussian_at(&self, times: &[f64], data: &[f64], peak_idx: usize) -> (f64, f64, f64) {
        let amplitude = data[peak_idx];
        let centre = times[peak_idx];
        let n = times.len();

        let half_max = amplitude * 0.5;

        // Search left for half-max crossing
        let mut left_t = centre;
        for i in (0..peak_idx).rev() {
            if data[i] <= half_max {
                let denom = data[i + 1] - data[i];
                let frac = if denom.abs() > 1e-30 {
                    (half_max - data[i]) / denom
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
            if data[i] <= half_max {
                let denom = data[i - 1] - data[i];
                let frac = if denom.abs() > 1e-30 {
                    (half_max - data[i]) / denom
                } else {
                    0.5
                };
                right_t = times[i] - frac * (times[i] - times[i - 1]);
                break;
            }
        }

        let fwhm = (right_t - left_t).abs();
        let sigma = if fwhm > 1e-15 {
            fwhm / (2.0 * SQRT_2LN2)
        } else if n > 1 {
            (times[1] - times[0]).abs()
        } else {
            1e-6
        };

        (amplitude, centre, sigma)
    }

    /// Peak area via trapezoidal integration over +/- 3*sigma.
    fn peak_area(&self, times: &[f64], data: &[f64], centre: f64, sigma: f64) -> f64 {
        let lo = centre - 3.0 * sigma;
        let hi = centre + 3.0 * sigma;
        let mut area = 0.0;
        for i in 1..times.len() {
            if times[i - 1] >= lo && times[i] <= hi {
                let dt = times[i] - times[i - 1];
                area += 0.5 * (data[i - 1] + data[i]) * dt;
            }
        }
        area
    }
}

// ---------------------------------------------------------------------------
// Free-standing helper functions
// ---------------------------------------------------------------------------

/// RMS of a data slice.
fn rms(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    let sum_sq: f64 = data.iter().map(|&x| x * x).sum();
    (sum_sq / data.len() as f64).sqrt()
}

/// Least-squares polynomial fit of degree `order`.
/// Returns coefficients [a0, a1, ..., a_order] where y = a0 + a1*x + a2*x^2 + ...
fn poly_fit(x: &[f64], y: &[f64], order: usize) -> Vec<f64> {
    let n = x.len();
    let p = order + 1;
    if n < p {
        return vec![0.0; p];
    }

    // Build normal equations: (X^T X) a = X^T y
    // X[i][j] = x[i]^j
    let mut xtx = vec![vec![0.0; p]; p];
    let mut xty = vec![0.0; p];

    for i in 0..n {
        let mut xi_pow = vec![1.0; p];
        for j in 1..p {
            xi_pow[j] = xi_pow[j - 1] * x[i];
        }
        for j in 0..p {
            xty[j] += xi_pow[j] * y[i];
            for k in 0..p {
                xtx[j][k] += xi_pow[j] * xi_pow[k];
            }
        }
    }

    // Solve via Gaussian elimination with partial pivoting
    gauss_solve(&mut xtx, &mut xty)
}

/// Solve a linear system Ax = b via Gaussian elimination with partial pivoting.
fn gauss_solve(a: &mut [Vec<f64>], b: &mut [f64]) -> Vec<f64> {
    let n = b.len();
    // Forward elimination
    for col in 0..n {
        // Partial pivoting
        let mut max_row = col;
        let mut max_val = a[col][col].abs();
        for row in (col + 1)..n {
            if a[row][col].abs() > max_val {
                max_val = a[row][col].abs();
                max_row = row;
            }
        }
        if max_row != col {
            a.swap(col, max_row);
            b.swap(col, max_row);
        }
        let pivot = a[col][col];
        if pivot.abs() < 1e-30 {
            continue;
        }
        for row in (col + 1)..n {
            let factor = a[row][col] / pivot;
            for k in col..n {
                a[row][k] -= factor * a[col][k];
            }
            b[row] -= factor * b[col];
        }
    }
    // Back substitution
    let mut x = vec![0.0; n];
    for col in (0..n).rev() {
        if a[col][col].abs() < 1e-30 {
            continue;
        }
        let mut sum = b[col];
        for k in (col + 1)..n {
            sum -= a[col][k] * x[k];
        }
        x[col] = sum / a[col][col];
    }
    x
}

/// Evaluate polynomial with coefficients [a0, a1, ...] at x.
fn poly_eval(coeffs: &[f64], x: f64) -> f64 {
    let mut result = 0.0;
    let mut x_pow = 1.0;
    for &c in coeffs {
        result += c * x_pow;
        x_pow *= x;
    }
    result
}

/// Compute Savitzky-Golay smoothing coefficients.
///
/// Returns coefficients for a window of 2*half_width+1 samples
/// with polynomial of degree `poly_order`.
fn sg_coefficients(half_width: usize, poly_order: usize) -> Vec<f64> {
    let m = half_width as i64;
    let window = (2 * m + 1) as usize;
    let p = poly_order + 1;

    // Build Vandermonde matrix J[i][k] = i^k for i in -m..=m
    let mut j_mat = vec![vec![0.0; p]; window];
    for (idx, row) in j_mat.iter_mut().enumerate() {
        let i = idx as i64 - m;
        let mut val = 1.0;
        for k in 0..p {
            row[k] = val;
            val *= i as f64;
        }
    }

    // Compute (J^T J)
    let mut jtj = vec![vec![0.0; p]; p];
    for row in 0..p {
        for col in 0..p {
            let mut sum = 0.0;
            for i in 0..window {
                sum += j_mat[i][row] * j_mat[i][col];
            }
            jtj[row][col] = sum;
        }
    }

    // Invert (J^T J) via Gauss-Jordan
    let mut inv = vec![vec![0.0; p]; p];
    for i in 0..p {
        inv[i][i] = 1.0;
    }

    let mut mat = jtj;
    for col in 0..p {
        let mut max_row = col;
        let mut max_val = mat[col][col].abs();
        for row in (col + 1)..p {
            if mat[row][col].abs() > max_val {
                max_val = mat[row][col].abs();
                max_row = row;
            }
        }
        mat.swap(col, max_row);
        inv.swap(col, max_row);

        let pivot = mat[col][col];
        if pivot.abs() < 1e-30 {
            continue;
        }
        for k in 0..p {
            mat[col][k] /= pivot;
            inv[col][k] /= pivot;
        }
        for row in 0..p {
            if row == col {
                continue;
            }
            let factor = mat[row][col];
            for k in 0..p {
                mat[row][k] -= factor * mat[col][k];
                inv[row][k] -= factor * inv[col][k];
            }
        }
    }

    // Smoothing coefficients for the 0th derivative:
    // c = (J^T J)^-1 * J^T, take the first row (0th derivative)
    // c[i] = sum_k inv[0][k] * J[i][k]
    let mut coeffs = vec![0.0; window];
    for i in 0..window {
        let mut sum = 0.0;
        for k in 0..p {
            sum += inv[0][k] * j_mat[i][k];
        }
        coeffs[i] = sum;
    }

    coeffs
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    fn default_proc() -> ImsProcessor {
        ImsProcessor::new(ImsConfig::default())
    }

    /// Generate a synthetic spectrum with Gaussian peaks.
    fn make_spectrum(
        n: usize,
        t_max_ms: f64,
        peaks: &[(f64, f64, f64)], // (centre_ms, amplitude, sigma_ms)
    ) -> ImsSpectrum {
        let dt = t_max_ms / n as f64;
        let drift_times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        let mut intensities = vec![0.0; n];
        for &(centre, amp, sigma) in peaks {
            for (i, &t) in drift_times.iter().enumerate() {
                intensities[i] += gaussian(t, amp, centre, sigma);
            }
        }
        ImsSpectrum { drift_times, intensities }
    }

    // -----------------------------------------------------------------------
    // 1. DriftGas properties
    // -----------------------------------------------------------------------

    #[test]
    fn test_drift_gas_molecular_weights() {
        assert!((DriftGas::N2.molecular_weight() - 28.014).abs() < 0.01);
        assert!((DriftGas::He.molecular_weight() - 4.003).abs() < 0.01);
        assert!((DriftGas::Air.molecular_weight() - 28.97).abs() < 0.01);
        assert!((DriftGas::CO2.molecular_weight() - 44.01).abs() < 0.01);
    }

    #[test]
    fn test_drift_gas_mass_kg() {
        let m = DriftGas::N2.mass_kg();
        // 28 amu ~ 4.65e-26 kg
        assert!(m > 4.0e-26 && m < 5.0e-26, "N2 mass = {}", m);
    }

    #[test]
    fn test_drift_gas_polarizability() {
        assert!(DriftGas::He.polarizability_a3() < DriftGas::N2.polarizability_a3());
        assert!(DriftGas::CO2.polarizability_a3() > DriftGas::N2.polarizability_a3());
        let p_m3 = DriftGas::N2.polarizability_m3();
        assert!(p_m3 > 1e-31 && p_m3 < 1e-29, "polarizability = {}", p_m3);
    }

    // -----------------------------------------------------------------------
    // 2. ImsConfig defaults
    // -----------------------------------------------------------------------

    #[test]
    fn test_config_defaults() {
        let cfg = ImsConfig::default();
        assert!((cfg.drift_tube_length_cm - 10.0).abs() < 1e-10);
        assert!((cfg.electric_field_vcm - 250.0).abs() < 1e-10);
        assert_eq!(cfg.drift_gas, DriftGas::N2);
        assert!((cfg.temperature_k - 373.15).abs() < 0.01);
        assert!((cfg.pressure_torr - 760.0).abs() < 0.01);
        assert!((cfg.gate_pulse_width_us - 200.0).abs() < 0.01);
    }

    // -----------------------------------------------------------------------
    // 3. Drift time <-> mobility conversions
    // -----------------------------------------------------------------------

    #[test]
    fn test_drift_time_to_mobility_rip() {
        let proc = default_proc();
        // For RIP+ with K0=2.22, compute expected drift time then round-trip
        let expected_td = proc.mobility_to_drift_time(2.22);
        let k0_back = proc.drift_time_to_mobility(expected_td);
        assert!(
            (k0_back - 2.22).abs() < 1e-6,
            "round-trip: {} vs 2.22",
            k0_back
        );
    }

    #[test]
    fn test_drift_time_to_mobility_tnt() {
        let proc = default_proc();
        let expected_td = proc.mobility_to_drift_time(1.54);
        let k0_back = proc.drift_time_to_mobility(expected_td);
        assert!(
            (k0_back - 1.54).abs() < 1e-6,
            "round-trip TNT: {} vs 1.54",
            k0_back
        );
    }

    #[test]
    fn test_mobility_to_drift_time_ordering() {
        let proc = default_proc();
        let td_rip = proc.mobility_to_drift_time(2.22);
        let td_tnt = proc.mobility_to_drift_time(1.54);
        let td_cocaine = proc.mobility_to_drift_time(1.16);
        // Higher K0 -> shorter drift time
        assert!(td_rip < td_tnt, "RIP should be faster than TNT");
        assert!(td_tnt < td_cocaine, "TNT should be faster than cocaine");
    }

    #[test]
    fn test_drift_time_realistic_range() {
        let proc = default_proc();
        // Typical drift times are 5-25 ms
        let td = proc.mobility_to_drift_time(1.54);
        assert!(td > 2.0 && td < 30.0, "drift time = {} ms", td);
    }

    #[test]
    fn test_drift_time_to_mobility_zero() {
        let proc = default_proc();
        let k0 = proc.drift_time_to_mobility(0.0);
        assert!((k0 - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_mobility_to_drift_time_zero() {
        let proc = default_proc();
        let td = proc.mobility_to_drift_time(0.0);
        assert!((td - 0.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // 4. Mason-Schamp CCS
    // -----------------------------------------------------------------------

    #[test]
    fn test_mobility_to_ccs_cocaine() {
        let proc = default_proc();
        // Cocaine [M+H]+: K0 ~ 1.16, MW ~ 303.4 Da, CCS ~ 180 A^2
        let ccs = proc.mobility_to_ccs(1.16, 303.4);
        assert!(
            ccs > 100.0 && ccs < 300.0,
            "cocaine CCS = {} A^2 (expected ~180)",
            ccs
        );
    }

    #[test]
    fn test_ccs_roundtrip() {
        let proc = default_proc();
        let k0 = 1.38;
        let mw = 186.3;
        let ccs = proc.mobility_to_ccs(k0, mw);
        let k0_back = proc.ccs_to_mobility(ccs, mw);
        assert!(
            (k0_back - k0).abs() < 0.01,
            "CCS round-trip: {} vs {}",
            k0_back,
            k0
        );
    }

    #[test]
    fn test_ccs_larger_ion_larger_ccs() {
        let proc = default_proc();
        // Larger molecules should have larger CCS at similar K0
        // Actually, larger molecules have lower K0 AND larger CCS
        let ccs_small = proc.mobility_to_ccs(2.01, 74.1);  // TMA+
        let ccs_large = proc.mobility_to_ccs(1.15, 242.5); // TBA+
        assert!(
            ccs_large > ccs_small,
            "TBA+ CCS {} should be > TMA+ CCS {}",
            ccs_large,
            ccs_small
        );
    }

    #[test]
    fn test_ccs_zero_inputs() {
        let proc = default_proc();
        assert!((proc.mobility_to_ccs(0.0, 100.0) - 0.0).abs() < 1e-10);
        assert!((proc.mobility_to_ccs(1.5, 0.0) - 0.0).abs() < 1e-10);
        assert!((proc.ccs_to_mobility(0.0, 100.0) - 0.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // 5. Peak detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_peak_detect_single() {
        let proc = default_proc();
        let spectrum = make_spectrum(1000, 30.0, &[(10.0, 500.0, 0.3)]);
        let peaks = proc.peak_detect(&spectrum, 10.0);
        assert!(!peaks.is_empty(), "should detect at least one peak");
        assert!(
            (peaks[0].drift_time_ms - 10.0).abs() < 0.5,
            "peak at {} ms, expected ~10.0",
            peaks[0].drift_time_ms
        );
    }

    #[test]
    fn test_peak_detect_two_peaks() {
        let proc = default_proc();
        let spectrum = make_spectrum(2000, 30.0, &[(8.0, 1000.0, 0.3), (15.0, 600.0, 0.4)]);
        let peaks = proc.peak_detect(&spectrum, 10.0);
        assert!(
            peaks.len() >= 2,
            "should detect >= 2 peaks, got {}",
            peaks.len()
        );
    }

    #[test]
    fn test_peak_detect_empty() {
        let proc = default_proc();
        let spectrum = ImsSpectrum::new(Vec::new(), Vec::new());
        let peaks = proc.peak_detect(&spectrum, 10.0);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_peak_detect_flat() {
        let proc = default_proc();
        let n = 500;
        let dt = 30.0 / n as f64;
        let drift_times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        let intensities = vec![100.0; n];
        let spectrum = ImsSpectrum::new(drift_times, intensities);
        // Flat signal: all internal points satisfy >= neighbors, so many
        // "peaks" may be detected. With the min-distance filter they get
        // spaced out. The important thing is no crash. A flat signal with
        // a very high threshold should yield zero peaks.
        let peaks_high_thresh = proc.peak_detect(&spectrum, 200.0);
        assert!(peaks_high_thresh.is_empty(), "no peaks above 200 in a flat-100 signal");
    }

    #[test]
    fn test_peak_fwhm_accuracy() {
        let proc = default_proc();
        let sigma = 0.3;
        let expected_fwhm = sigma * 2.0 * SQRT_2LN2;
        let spectrum = make_spectrum(2000, 30.0, &[(10.0, 1000.0, sigma)]);
        let peaks = proc.peak_detect(&spectrum, 10.0);
        assert!(!peaks.is_empty());
        let measured_fwhm = peaks[0].fwhm_ms;
        assert!(
            (measured_fwhm - expected_fwhm).abs() < 0.15,
            "FWHM: {} vs expected {}",
            measured_fwhm,
            expected_fwhm
        );
    }

    #[test]
    fn test_resolving_power_calculation() {
        let rp = ImsProcessor::resolving_power(12.0, 0.3);
        assert!((rp - 40.0).abs() < 0.01, "R = {}", rp);
    }

    #[test]
    fn test_resolving_power_typical_range() {
        let proc = default_proc();
        let spectrum = make_spectrum(2000, 30.0, &[(10.0, 1000.0, 0.15)]);
        let peaks = proc.peak_detect(&spectrum, 10.0);
        assert!(!peaks.is_empty());
        let rp = peaks[0].resolving_power;
        // Typical IMS resolving power: 30-100
        assert!(rp > 10.0 && rp < 200.0, "R = {}", rp);
    }

    #[test]
    fn test_peak_area_positive() {
        let proc = default_proc();
        let spectrum = make_spectrum(2000, 30.0, &[(10.0, 500.0, 0.3)]);
        let peaks = proc.peak_detect(&spectrum, 10.0);
        assert!(!peaks.is_empty());
        assert!(peaks[0].area > 0.0, "area = {}", peaks[0].area);
    }

    #[test]
    fn test_peak_k0_computed() {
        let proc = default_proc();
        let td = proc.mobility_to_drift_time(1.54);
        let spectrum = make_spectrum(2000, 30.0, &[(td, 500.0, 0.25)]);
        let peaks = proc.peak_detect(&spectrum, 10.0);
        assert!(!peaks.is_empty());
        assert!(
            (peaks[0].k0 - 1.54).abs() < 0.1,
            "K0 = {}, expected ~1.54",
            peaks[0].k0
        );
    }

    // -----------------------------------------------------------------------
    // 6. Spectrum generation
    // -----------------------------------------------------------------------

    #[test]
    fn test_generate_gaussian_peak() {
        let proc = default_proc();
        let spectrum = proc.generate_gaussian_peak(1000, 30.0, 10.0, 500.0, 0.3);
        assert_eq!(spectrum.len(), 1000);
        let max_val = spectrum.intensities.iter().cloned().fold(0.0_f64, f64::max);
        assert!(
            (max_val - 500.0).abs() < 5.0,
            "max = {}, expected ~500",
            max_val
        );
    }

    #[test]
    fn test_generate_multi_peak() {
        let proc = default_proc();
        let spectrum = proc.generate_multi_peak(
            1000,
            30.0,
            &[(8.0, 1000.0, 0.3), (15.0, 600.0, 0.4)],
        );
        assert_eq!(spectrum.len(), 1000);
        let max_val = spectrum.intensities.iter().cloned().fold(0.0_f64, f64::max);
        assert!(max_val > 900.0, "max = {}", max_val);
    }

    // -----------------------------------------------------------------------
    // 7. Baseline correction
    // -----------------------------------------------------------------------

    #[test]
    fn test_baseline_correction_rolling_min() {
        let proc = default_proc();
        let n = 500;
        let dt = 30.0 / n as f64;
        let drift_times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        // Linear baseline + peak
        let mut intensities: Vec<f64> = (0..n).map(|i| 50.0 + 0.5 * i as f64).collect();
        for (i, &t) in drift_times.iter().enumerate() {
            intensities[i] += gaussian(t, 500.0, 10.0, 0.3);
        }
        let spectrum = ImsSpectrum::new(drift_times, intensities);
        let corrected = proc.baseline_correction(&spectrum, 50);
        // The baseline region should be close to 0
        let baseline_val = corrected.intensities[0];
        assert!(baseline_val < 30.0, "baseline[0] = {}", baseline_val);
    }

    #[test]
    fn test_baseline_correction_poly() {
        let proc = default_proc();
        let n = 500;
        let dt = 30.0 / n as f64;
        let drift_times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        // Quadratic baseline + peak
        let mut intensities: Vec<f64> = drift_times.iter().map(|&t| 10.0 + 0.1 * t * t).collect();
        for (i, &t) in drift_times.iter().enumerate() {
            intensities[i] += gaussian(t, 500.0, 15.0, 0.3);
        }
        let spectrum = ImsSpectrum::new(drift_times, intensities);
        let corrected = proc.baseline_correction_poly(&spectrum, 2);
        // After removing quadratic baseline, peak should remain
        let max_val = corrected.intensities.iter().cloned().fold(0.0_f64, f64::max);
        assert!(max_val > 100.0, "max after poly baseline = {}", max_val);
    }

    // -----------------------------------------------------------------------
    // 8. Normalization
    // -----------------------------------------------------------------------

    #[test]
    fn test_normalize_spectrum() {
        let proc = default_proc();
        let spectrum = make_spectrum(500, 30.0, &[(10.0, 750.0, 0.3)]);
        let normalized = proc.normalize_spectrum(&spectrum);
        let max_val = normalized.intensities.iter().cloned().fold(0.0_f64, f64::max);
        assert!(
            (max_val - 1.0).abs() < 1e-10,
            "max after normalization = {}",
            max_val
        );
    }

    #[test]
    fn test_normalize_zero_spectrum() {
        let proc = default_proc();
        let spectrum = ImsSpectrum::new(vec![0.0, 1.0, 2.0], vec![0.0, 0.0, 0.0]);
        let normalized = proc.normalize_spectrum(&spectrum);
        assert_eq!(normalized.intensities, vec![0.0, 0.0, 0.0]);
    }

    // -----------------------------------------------------------------------
    // 9. Smoothing
    // -----------------------------------------------------------------------

    #[test]
    fn test_smooth_spectrum_moving_average() {
        let proc = default_proc();
        // Noisy signal
        let n = 200;
        let dt = 30.0 / n as f64;
        let drift_times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        let mut intensities: Vec<f64> = drift_times
            .iter()
            .map(|&t| gaussian(t, 500.0, 10.0, 0.5))
            .collect();
        // Add noise
        for i in 0..n {
            intensities[i] += (i as f64 * 7.3).sin() * 20.0;
        }
        let spectrum = ImsSpectrum::new(drift_times, intensities);
        let smoothed = proc.smooth_spectrum(&spectrum, 5);
        // Smoothed variance should be less
        let var_orig: f64 = spectrum.intensities.windows(2).map(|w| (w[1] - w[0]).powi(2)).sum::<f64>();
        let var_smooth: f64 = smoothed.intensities.windows(2).map(|w| (w[1] - w[0]).powi(2)).sum::<f64>();
        assert!(var_smooth < var_orig, "smoothing should reduce variance");
    }

    #[test]
    fn test_smooth_spectrum_identity() {
        let proc = default_proc();
        let spectrum = make_spectrum(100, 30.0, &[(10.0, 500.0, 0.3)]);
        let smoothed = proc.smooth_spectrum(&spectrum, 1);
        // Window=1 should return original
        for (a, b) in spectrum.intensities.iter().zip(smoothed.intensities.iter()) {
            assert!((a - b).abs() < 1e-10);
        }
    }

    #[test]
    fn test_smooth_spectrum_sg() {
        let proc = default_proc();
        let n = 200;
        let dt = 30.0 / n as f64;
        let drift_times: Vec<f64> = (0..n).map(|i| i as f64 * dt).collect();
        let mut intensities: Vec<f64> = drift_times
            .iter()
            .map(|&t| gaussian(t, 500.0, 10.0, 0.5))
            .collect();
        for i in 0..n {
            intensities[i] += (i as f64 * 3.7).sin() * 15.0;
        }
        let spectrum = ImsSpectrum::new(drift_times, intensities);
        let smoothed = proc.smooth_spectrum_sg(&spectrum, 3, 2);
        let var_orig: f64 = spectrum.intensities.windows(2).map(|w| (w[1] - w[0]).powi(2)).sum::<f64>();
        let var_smooth: f64 = smoothed.intensities.windows(2).map(|w| (w[1] - w[0]).powi(2)).sum::<f64>();
        assert!(var_smooth < var_orig, "SG smoothing should reduce variance");
    }

    // -----------------------------------------------------------------------
    // 10. RIP characterization
    // -----------------------------------------------------------------------

    #[test]
    fn test_reactant_ion_peak() {
        let proc = default_proc();
        let td_rip = proc.mobility_to_drift_time(2.22);
        let td_analyte = proc.mobility_to_drift_time(1.54);
        let spectrum = make_spectrum(
            2000,
            30.0,
            &[(td_rip, 2000.0, 0.2), (td_analyte, 500.0, 0.25)],
        );
        let peaks = proc.peak_detect(&spectrum, 10.0);
        let rip = proc.reactant_ion_peak(&peaks);
        assert!(rip.is_some());
        let (idx, k0) = rip.unwrap();
        // RIP should be the peak with shortest drift time
        assert_eq!(idx, 0, "RIP should be the first peak");
        assert!(
            (k0 - 2.22).abs() < 0.15,
            "RIP K0 = {}, expected ~2.22",
            k0
        );
    }

    #[test]
    fn test_reactant_ion_peak_empty() {
        let proc = default_proc();
        let rip = proc.reactant_ion_peak(&[]);
        assert!(rip.is_none());
    }

    // -----------------------------------------------------------------------
    // 11. Analyte lookup
    // -----------------------------------------------------------------------

    #[test]
    fn test_reduced_mobility_lookup_tnt() {
        let proc = default_proc();
        let record = proc.reduced_mobility_lookup(1.54, 0.1, false);
        assert!(record.is_some());
        assert_eq!(record.unwrap().name, "TNT");
    }

    #[test]
    fn test_reduced_mobility_lookup_cocaine() {
        let proc = default_proc();
        let record = proc.reduced_mobility_lookup(1.16, 0.1, true);
        assert!(record.is_some());
        assert_eq!(record.unwrap().name, "Cocaine");
    }

    #[test]
    fn test_reduced_mobility_lookup_gb() {
        let proc = default_proc();
        let record = proc.reduced_mobility_lookup(1.61, 0.05, true);
        assert!(record.is_some());
        assert!(record.unwrap().name.contains("GB"));
    }

    #[test]
    fn test_reduced_mobility_lookup_no_match() {
        let proc = default_proc();
        let record = proc.reduced_mobility_lookup(5.0, 0.01, true);
        assert!(record.is_none());
    }

    #[test]
    fn test_reduced_mobility_lookup_polarity_filter() {
        let proc = default_proc();
        // TNT is negative mode only
        let pos = proc.reduced_mobility_lookup(1.54, 0.1, true);
        let neg = proc.reduced_mobility_lookup(1.54, 0.1, false);
        assert!(neg.is_some(), "TNT should match in negative mode");
        // In positive mode, might match GA (1.46) or methamphetamine (1.63) but not TNT
        if let Some(r) = pos {
            assert_ne!(r.name, "TNT", "TNT should not match in positive mode");
        }
    }

    // -----------------------------------------------------------------------
    // 12. Temperature and pressure corrections
    // -----------------------------------------------------------------------

    #[test]
    fn test_temperature_correction_identity() {
        let proc = default_proc();
        let k0 = 1.54;
        let corrected = proc.temperature_correction(k0, 373.15, 373.15);
        assert!((corrected - k0).abs() < 1e-10);
    }

    #[test]
    fn test_temperature_correction_higher_temp() {
        let proc = default_proc();
        let k0 = 1.54;
        let corrected = proc.temperature_correction(k0, 373.15, 500.0);
        // sqrt(500/373.15) > 1 so corrected > k0
        assert!(corrected > k0, "corrected = {}, should be > {}", corrected, k0);
    }

    #[test]
    fn test_pressure_correction_identity() {
        let proc = default_proc();
        let k0 = 1.54;
        let corrected = proc.pressure_correction(k0, 760.0, 760.0);
        assert!((corrected - k0).abs() < 1e-10);
    }

    #[test]
    fn test_pressure_correction_higher_pressure() {
        let proc = default_proc();
        let k0 = 1.54;
        let corrected = proc.pressure_correction(k0, 760.0, 800.0);
        // alpha is negative, so higher pressure -> slightly lower K0
        assert!(corrected < k0, "corrected = {}, should be < {}", corrected, k0);
    }

    // -----------------------------------------------------------------------
    // 13. CCS database
    // -----------------------------------------------------------------------

    #[test]
    fn test_ccs_database_entries() {
        let db = collision_cross_section_database();
        assert!(db.len() >= 8, "CCS database should have >= 8 entries, got {}", db.len());
    }

    #[test]
    fn test_ccs_database_values_reasonable() {
        let db = collision_cross_section_database();
        for entry in &db {
            assert!(entry.ccs_a2 > 20.0 && entry.ccs_a2 < 500.0,
                    "{}: CCS = {} A^2", entry.name, entry.ccs_a2);
            assert!(entry.k0 > 0.5 && entry.k0 < 5.0,
                    "{}: K0 = {}", entry.name, entry.k0);
            assert!(entry.mw_da > 10.0,
                    "{}: MW = {}", entry.name, entry.mw_da);
        }
    }

    // -----------------------------------------------------------------------
    // 14. Multi-peak fit (deconvolution)
    // -----------------------------------------------------------------------

    #[test]
    fn test_multi_peak_fit_single() {
        let proc = default_proc();
        let spectrum = make_spectrum(1000, 30.0, &[(10.0, 500.0, 0.3)]);
        let components = proc.multi_peak_fit(&spectrum, 50.0, 5);
        assert_eq!(components.len(), 1);
        assert!(
            (components[0].0 - 10.0).abs() < 0.5,
            "centre = {}",
            components[0].0
        );
    }

    #[test]
    fn test_multi_peak_fit_two_overlapping() {
        let proc = default_proc();
        let spectrum = make_spectrum(
            2000,
            30.0,
            &[(10.0, 500.0, 0.3), (11.0, 400.0, 0.3)],
        );
        let components = proc.multi_peak_fit(&spectrum, 30.0, 5);
        assert!(
            components.len() >= 2,
            "should find >= 2 components, got {}",
            components.len()
        );
    }

    #[test]
    fn test_multi_peak_fit_max_peaks_limit() {
        let proc = default_proc();
        let spectrum = make_spectrum(
            2000,
            30.0,
            &[(5.0, 500.0, 0.3), (10.0, 400.0, 0.3), (15.0, 300.0, 0.3), (20.0, 200.0, 0.3)],
        );
        let components = proc.multi_peak_fit(&spectrum, 10.0, 2);
        assert!(components.len() <= 2, "should respect max_peaks=2");
    }

    // -----------------------------------------------------------------------
    // 15. SNR estimation
    // -----------------------------------------------------------------------

    #[test]
    fn test_snr_estimate() {
        let proc = default_proc();
        let spectrum = make_spectrum(1000, 30.0, &[(15.0, 500.0, 0.3)]);
        let peaks = proc.peak_detect(&spectrum, 10.0);
        let snrs = proc.snr_estimate(&spectrum, &peaks);
        assert_eq!(snrs.len(), peaks.len());
        // Peak is in the middle, edges are ~0, so SNR should be high
        for &snr in &snrs {
            assert!(snr > 10.0, "SNR = {} should be high", snr);
        }
    }

    #[test]
    fn test_snr_estimate_short_spectrum() {
        let proc = default_proc();
        let spectrum = ImsSpectrum::new(vec![1.0, 2.0, 3.0], vec![10.0, 20.0, 10.0]);
        let peaks = vec![ImsPeak {
            drift_time_ms: 2.0,
            intensity: 20.0,
            fwhm_ms: 0.5,
            resolving_power: 4.0,
            k0: 1.5,
            sigma_ms: 0.2,
            area: 5.0,
        }];
        let snrs = proc.snr_estimate(&spectrum, &peaks);
        assert_eq!(snrs.len(), 1);
        // Too short for proper noise estimation, returns 0
        assert!((snrs[0] - 0.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // 16. Alarm threshold
    // -----------------------------------------------------------------------

    #[test]
    fn test_alarm_threshold_triggered() {
        let proc = default_proc();
        let td_tnt = proc.mobility_to_drift_time(1.54);
        let spectrum = make_spectrum(2000, 30.0, &[(td_tnt, 500.0, 0.25)]);
        let peaks = proc.peak_detect(&spectrum, 10.0);
        let alarms = proc.alarm_threshold(&peaks, 0.1, 100.0, false);
        assert!(!alarms.is_empty(), "should raise alarm for TNT");
        assert!(alarms[0].triggered, "intensity should exceed threshold");
        assert_eq!(alarms[0].analyte, "TNT");
    }

    #[test]
    fn test_alarm_threshold_not_triggered() {
        let proc = default_proc();
        let td_tnt = proc.mobility_to_drift_time(1.54);
        let spectrum = make_spectrum(2000, 30.0, &[(td_tnt, 50.0, 0.25)]);
        let peaks = proc.peak_detect(&spectrum, 5.0);
        let alarms = proc.alarm_threshold(&peaks, 0.1, 1000.0, false);
        if !alarms.is_empty() {
            assert!(!alarms[0].triggered, "intensity should be below threshold");
        }
    }

    #[test]
    fn test_alarm_threshold_unknown_peak() {
        let proc = default_proc();
        // Peak at K0 ~ 3.0, no match in database
        let td = proc.mobility_to_drift_time(3.0);
        let spectrum = make_spectrum(2000, 30.0, &[(td, 500.0, 0.25)]);
        let peaks = proc.peak_detect(&spectrum, 10.0);
        let alarms = proc.alarm_threshold(&peaks, 0.05, 100.0, true);
        assert!(alarms.is_empty(), "unknown peak should not trigger alarm");
    }

    // -----------------------------------------------------------------------
    // 17. FT-IMS twin gate
    // -----------------------------------------------------------------------

    #[test]
    fn test_twin_gate_mode_basic() {
        let proc = default_proc();
        let n_freq = 128;
        let gate_frequencies: Vec<f64> = (0..n_freq).map(|i| 10.0 + i as f64 * 50.0).collect();
        let k0_ions = vec![(2.22, 1.0), (1.54, 0.5)];
        let spectrum = proc.twin_gate_mode(&gate_frequencies, &k0_ions);
        assert!(!spectrum.is_empty(), "should produce a spectrum");
        assert_eq!(spectrum.len(), n_freq);
    }

    #[test]
    fn test_twin_gate_mode_empty() {
        let proc = default_proc();
        let spectrum = proc.twin_gate_mode(&[], &[(1.5, 1.0)]);
        assert!(spectrum.is_empty());
    }

    // -----------------------------------------------------------------------
    // 18. ImsSpectrum
    // -----------------------------------------------------------------------

    #[test]
    fn test_ims_spectrum_new() {
        let spectrum = ImsSpectrum::new(vec![1.0, 2.0, 3.0], vec![10.0, 20.0, 30.0]);
        assert_eq!(spectrum.len(), 3);
        assert!(!spectrum.is_empty());
    }

    #[test]
    #[should_panic(expected = "drift_times and intensities must have equal length")]
    fn test_ims_spectrum_mismatched_lengths() {
        ImsSpectrum::new(vec![1.0, 2.0], vec![10.0]);
    }

    // -----------------------------------------------------------------------
    // 19. Analyte database
    // -----------------------------------------------------------------------

    #[test]
    fn test_default_analyte_database_coverage() {
        let db = default_analyte_database();
        assert!(db.len() >= 20, "should have >= 20 entries, got {}", db.len());
        let has_explosive = db.iter().any(|a| a.category == "explosive");
        let has_cwa = db.iter().any(|a| a.category == "CWA");
        let has_narcotic = db.iter().any(|a| a.category == "narcotic");
        let has_rip = db.iter().any(|a| a.category == "RIP");
        assert!(has_explosive);
        assert!(has_cwa);
        assert!(has_narcotic);
        assert!(has_rip);
    }

    #[test]
    fn test_analyte_k0_values_realistic() {
        let db = default_analyte_database();
        for entry in &db {
            assert!(
                entry.k0 > 0.5 && entry.k0 < 5.0,
                "{}: K0 = {} out of range",
                entry.name,
                entry.k0
            );
        }
    }

    // -----------------------------------------------------------------------
    // 20. Custom analyte database
    // -----------------------------------------------------------------------

    #[test]
    fn test_custom_analyte_db() {
        let mut proc = default_proc();
        proc.set_analyte_db(vec![AnalyteRecord {
            name: "CustomAgent",
            k0: 1.80,
            category: "test",
            positive_mode: true,
        }]);
        let record = proc.reduced_mobility_lookup(1.80, 0.05, true);
        assert!(record.is_some());
        assert_eq!(record.unwrap().name, "CustomAgent");
        // Original DB should be gone
        let tnt = proc.reduced_mobility_lookup(1.54, 0.05, false);
        assert!(tnt.is_none());
    }

    // -----------------------------------------------------------------------
    // 21. Polynomial fit helpers
    // -----------------------------------------------------------------------

    #[test]
    fn test_poly_fit_linear() {
        // y = 2 + 3*x
        let x: Vec<f64> = (0..10).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|&xi| 2.0 + 3.0 * xi).collect();
        let coeffs = poly_fit(&x, &y, 1);
        assert!((coeffs[0] - 2.0).abs() < 1e-6, "a0 = {}", coeffs[0]);
        assert!((coeffs[1] - 3.0).abs() < 1e-6, "a1 = {}", coeffs[1]);
    }

    #[test]
    fn test_poly_fit_quadratic() {
        // y = 1 + 0.5*x + 0.1*x^2
        let x: Vec<f64> = (0..20).map(|i| i as f64).collect();
        let y: Vec<f64> = x.iter().map(|&xi| 1.0 + 0.5 * xi + 0.1 * xi * xi).collect();
        let coeffs = poly_fit(&x, &y, 2);
        assert!((coeffs[0] - 1.0).abs() < 1e-4, "a0 = {}", coeffs[0]);
        assert!((coeffs[1] - 0.5).abs() < 1e-4, "a1 = {}", coeffs[1]);
        assert!((coeffs[2] - 0.1).abs() < 1e-4, "a2 = {}", coeffs[2]);
    }

    #[test]
    fn test_poly_eval() {
        let coeffs = vec![2.0, 3.0, 0.5]; // 2 + 3x + 0.5x^2
        let val = poly_eval(&coeffs, 4.0);
        let expected = 2.0 + 3.0 * 4.0 + 0.5 * 16.0;
        assert!((val - expected).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // 22. SG coefficients
    // -----------------------------------------------------------------------

    #[test]
    fn test_sg_coefficients_sum_to_one() {
        let coeffs = sg_coefficients(3, 2);
        let sum: f64 = coeffs.iter().sum();
        assert!(
            (sum - 1.0).abs() < 1e-10,
            "SG smoothing coefficients should sum to 1.0, got {}",
            sum
        );
    }

    #[test]
    fn test_sg_coefficients_symmetric() {
        let coeffs = sg_coefficients(3, 2);
        let n = coeffs.len();
        for i in 0..n / 2 {
            assert!(
                (coeffs[i] - coeffs[n - 1 - i]).abs() < 1e-10,
                "coefficients should be symmetric: [{}]={} vs [{}]={}",
                i,
                coeffs[i],
                n - 1 - i,
                coeffs[n - 1 - i]
            );
        }
    }

    // -----------------------------------------------------------------------
    // 23. Gaussian helper
    // -----------------------------------------------------------------------

    #[test]
    fn test_gaussian_peak_value() {
        let val = gaussian(5.0, 100.0, 5.0, 1.0);
        assert!((val - 100.0).abs() < 1e-10);
    }

    #[test]
    fn test_gaussian_one_sigma() {
        let val = gaussian(6.0, 100.0, 5.0, 1.0);
        let expected = 100.0 * (-0.5_f64).exp();
        assert!((val - expected).abs() < 1e-8);
    }

    #[test]
    fn test_gaussian_zero_sigma() {
        let val = gaussian(5.0, 100.0, 5.0, 0.0);
        assert!((val - 0.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // 24. RMS helper
    // -----------------------------------------------------------------------

    #[test]
    fn test_rms_values() {
        let data = vec![3.0, 4.0];
        let r = rms(&data);
        let expected = (12.5_f64).sqrt();
        assert!((r - expected).abs() < 1e-10);
    }

    #[test]
    fn test_rms_empty() {
        assert!((rms(&[]) - 0.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // 25. Different drift gas effects
    // -----------------------------------------------------------------------

    #[test]
    fn test_helium_drift_gas_faster() {
        // He has lower mass -> ions drift faster -> shorter drift times for same K0
        // But K0 is normalized to STP of the specific gas. In practice,
        // the CCS changes with gas. Test that CCS computation differs.
        let proc_n2 = ImsProcessor::new(ImsConfig {
            drift_gas: DriftGas::N2,
            ..ImsConfig::default()
        });
        let proc_he = ImsProcessor::new(ImsConfig {
            drift_gas: DriftGas::He,
            ..ImsConfig::default()
        });

        let ccs_n2 = proc_n2.mobility_to_ccs(1.5, 200.0);
        let ccs_he = proc_he.mobility_to_ccs(1.5, 200.0);
        // Different gases give different CCS for same K0
        assert!(
            (ccs_n2 - ccs_he).abs() > 1.0,
            "CCS should differ between gases: N2={}, He={}",
            ccs_n2,
            ccs_he
        );
    }

    // -----------------------------------------------------------------------
    // 26. Integration: full processing pipeline
    // -----------------------------------------------------------------------

    #[test]
    fn test_full_pipeline_rip_plus_explosive() {
        let proc = default_proc();
        let td_rip = proc.mobility_to_drift_time(2.22);
        let td_rdx = proc.mobility_to_drift_time(1.39);

        let spectrum = make_spectrum(
            2000,
            30.0,
            &[(td_rip, 2000.0, 0.2), (td_rdx, 400.0, 0.25)],
        );

        // Smooth
        let smoothed = proc.smooth_spectrum(&spectrum, 3);
        // Detect peaks
        let peaks = proc.peak_detect(&smoothed, 20.0);
        assert!(peaks.len() >= 2, "should find RIP + RDX");

        // Identify RIP
        let rip = proc.reactant_ion_peak(&peaks);
        assert!(rip.is_some());

        // Check alarm
        let alarms = proc.alarm_threshold(&peaks, 0.1, 50.0, false);
        let rdx_alarm = alarms.iter().find(|a| a.analyte == "RDX");
        assert!(rdx_alarm.is_some(), "should alarm on RDX");
        assert!(rdx_alarm.unwrap().triggered);
    }

    // -----------------------------------------------------------------------
    // 27. Drift tube geometry effects
    // -----------------------------------------------------------------------

    #[test]
    fn test_longer_tube_longer_drift_time() {
        let proc_short = ImsProcessor::new(ImsConfig {
            drift_tube_length_cm: 5.0,
            ..ImsConfig::default()
        });
        let proc_long = ImsProcessor::new(ImsConfig {
            drift_tube_length_cm: 15.0,
            ..ImsConfig::default()
        });

        let td_short = proc_short.mobility_to_drift_time(1.54);
        let td_long = proc_long.mobility_to_drift_time(1.54);
        assert!(td_long > td_short, "longer tube = longer drift time");
    }

    #[test]
    fn test_higher_field_shorter_drift_time() {
        let proc_lo = ImsProcessor::new(ImsConfig {
            electric_field_vcm: 200.0,
            ..ImsConfig::default()
        });
        let proc_hi = ImsProcessor::new(ImsConfig {
            electric_field_vcm: 400.0,
            ..ImsConfig::default()
        });

        let td_lo = proc_lo.mobility_to_drift_time(1.54);
        let td_hi = proc_hi.mobility_to_drift_time(1.54);
        assert!(td_hi < td_lo, "higher field = shorter drift time");
    }
}
