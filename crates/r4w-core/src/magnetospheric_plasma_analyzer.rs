//! Magnetospheric plasma analysis for space weather monitoring.
//!
//! This module implements time-series spectral analysis of magnetospheric ion cyclotron
//! and Alfven wave modes. It covers gyrofrequency computation, wave mode identification,
//! polarization analysis, and geomagnetic index estimation from magnetic field data.
//!
//! # Physics Background
//!
//! In the magnetosphere, charged particles gyrate around magnetic field lines at
//! species-dependent cyclotron frequencies. Plasma waves propagate in characteristic
//! modes determined by the magnetic field strength, particle densities, and temperatures:
//!
//! - **Ion cyclotron waves (ICW)**: left-hand polarized electromagnetic waves near the
//!   ion gyrofrequency, important for energy transfer between ring current ions and
//!   the thermal plasma.
//! - **Alfven waves**: low-frequency MHD waves propagating along field lines at the
//!   Alfven speed v_A = B / sqrt(mu_0 * rho). They carry energy through the
//!   magnetosphere and couple to the ionosphere.
//! - **Compressional modes**: oscillations in which the magnetic field magnitude
//!   varies (field-aligned compression).
//! - **Transverse modes**: oscillations perpendicular to the background field with
//!   approximately constant |B|.
//!
//! # Processing Pipeline
//!
//! 1. Compute plasma parameters (gyrofrequencies, plasma frequency, Alfven speed)
//! 2. Perform FFT-based spectral analysis on magnetic field time series
//! 3. Identify wave modes by frequency and polarization
//! 4. Separate compressional and transverse components
//! 5. Estimate geomagnetic indices (Kp, Dst) from perturbation statistics
//!
//! # Example
//!
//! ```
//! use r4w_core::magnetospheric_plasma_analyzer::{
//!     PlasmaParameters, GyrofrequencyCalculator, IonSpecies,
//!     cyclotron_frequency, alfven_speed, plasma_frequency,
//! };
//!
//! let params = PlasmaParameters {
//!     electron_density_m3: 1e7,
//!     ion_density_m3: 1e7,
//!     electron_temperature_ev: 100.0,
//!     ion_temperature_ev: 1000.0,
//!     magnetic_field_t: 100e-9, // 100 nT
//! };
//!
//! let calc = GyrofrequencyCalculator::new(params.magnetic_field_t);
//! let f_proton = calc.ion_gyrofrequency(IonSpecies::Proton);
//! assert!(f_proton > 0.0);
//!
//! let v_a = alfven_speed(params.magnetic_field_t, params.ion_density_m3, IonSpecies::Proton);
//! assert!(v_a > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants (SI units)
// ---------------------------------------------------------------------------

/// Elementary charge (C).
pub const ELECTRON_CHARGE: f64 = 1.602_176_634e-19;

/// Electron mass (kg).
pub const ELECTRON_MASS: f64 = 9.109_383_7015e-31;

/// Proton mass (kg).
pub const PROTON_MASS: f64 = 1.672_621_923_69e-27;

/// Atomic mass unit (kg).
pub const AMU: f64 = 1.660_539_066_60e-27;

/// Vacuum permeability mu_0 (H/m).
pub const MU_0: f64 = 1.256_637_062_12e-6;

/// Vacuum permittivity epsilon_0 (F/m).
pub const EPSILON_0: f64 = 8.854_187_8128e-12;

/// Boltzmann constant (J/K).
pub const BOLTZMANN_K: f64 = 1.380_649e-23;

/// Speed of light (m/s).
pub const SPEED_OF_LIGHT: f64 = 2.997_924_58e8;

/// Conversion factor: 1 eV in Joules.
pub const EV_TO_JOULE: f64 = 1.602_176_634e-19;

// ---------------------------------------------------------------------------
// Ion species
// ---------------------------------------------------------------------------

/// Ion species commonly found in the magnetosphere.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IonSpecies {
    /// Proton (H+), mass = 1 AMU.
    Proton,
    /// Helium ion (He+), mass = 4 AMU.
    HeliumPlus,
    /// Oxygen ion (O+), mass = 16 AMU.
    OxygenPlus,
    /// Custom species with mass in kg and charge number Z.
    Custom {
        /// Mass in kg.
        mass_kg: f64,
        /// Charge number (e.g., 1 for singly ionized).
        charge_number: f64,
    },
}

impl IonSpecies {
    /// Return the ion mass in kg.
    pub fn mass_kg(&self) -> f64 {
        match self {
            IonSpecies::Proton => PROTON_MASS,
            IonSpecies::HeliumPlus => 4.0 * AMU,
            IonSpecies::OxygenPlus => 16.0 * AMU,
            IonSpecies::Custom { mass_kg, .. } => *mass_kg,
        }
    }

    /// Return the charge number Z (number of elementary charges).
    pub fn charge_number(&self) -> f64 {
        match self {
            IonSpecies::Proton => 1.0,
            IonSpecies::HeliumPlus => 1.0,
            IonSpecies::OxygenPlus => 1.0,
            IonSpecies::Custom { charge_number, .. } => *charge_number,
        }
    }
}

// ---------------------------------------------------------------------------
// Plasma parameters
// ---------------------------------------------------------------------------

/// Core plasma parameters for the magnetospheric environment.
#[derive(Debug, Clone, Copy)]
pub struct PlasmaParameters {
    /// Electron number density in m^-3.
    pub electron_density_m3: f64,
    /// Ion number density in m^-3 (dominant species).
    pub ion_density_m3: f64,
    /// Electron temperature in eV.
    pub electron_temperature_ev: f64,
    /// Ion temperature in eV.
    pub ion_temperature_ev: f64,
    /// Background magnetic field magnitude in Tesla.
    pub magnetic_field_t: f64,
}

impl Default for PlasmaParameters {
    /// Default: typical inner magnetosphere (L~4) values.
    fn default() -> Self {
        Self {
            electron_density_m3: 1e7,
            ion_density_m3: 1e7,
            electron_temperature_ev: 100.0,
            ion_temperature_ev: 1000.0,
            magnetic_field_t: 200e-9, // 200 nT
        }
    }
}

// ---------------------------------------------------------------------------
// Helper / standalone functions
// ---------------------------------------------------------------------------

/// Compute the cyclotron (gyro) frequency in Hz for a particle with given charge
/// number and mass in a magnetic field of strength `b_tesla`.
///
/// f_c = |q| * B / (2 * pi * m)
pub fn cyclotron_frequency(b_tesla: f64, charge_number: f64, mass_kg: f64) -> f64 {
    (charge_number * ELECTRON_CHARGE * b_tesla.abs()) / (2.0 * PI * mass_kg)
}

/// Compute the angular cyclotron frequency omega_c = |q| * B / m (rad/s).
pub fn cyclotron_angular_frequency(b_tesla: f64, charge_number: f64, mass_kg: f64) -> f64 {
    (charge_number * ELECTRON_CHARGE * b_tesla.abs()) / mass_kg
}

/// Compute the electron plasma frequency in Hz.
///
/// f_pe = (1 / 2pi) * sqrt(n_e * e^2 / (epsilon_0 * m_e))
pub fn plasma_frequency(electron_density_m3: f64) -> f64 {
    let omega_pe_sq =
        electron_density_m3 * ELECTRON_CHARGE * ELECTRON_CHARGE / (EPSILON_0 * ELECTRON_MASS);
    omega_pe_sq.sqrt() / (2.0 * PI)
}

/// Compute the ion plasma frequency in Hz for a given species.
///
/// f_pi = (1 / 2pi) * sqrt(n_i * (Z*e)^2 / (epsilon_0 * m_i))
pub fn ion_plasma_frequency(ion_density_m3: f64, species: IonSpecies) -> f64 {
    let q = species.charge_number() * ELECTRON_CHARGE;
    let omega_pi_sq = ion_density_m3 * q * q / (EPSILON_0 * species.mass_kg());
    omega_pi_sq.sqrt() / (2.0 * PI)
}

/// Compute the Alfven speed in m/s.
///
/// v_A = B / sqrt(mu_0 * n_i * m_i)
pub fn alfven_speed(b_tesla: f64, ion_density_m3: f64, species: IonSpecies) -> f64 {
    let rho = ion_density_m3 * species.mass_kg();
    b_tesla.abs() / (MU_0 * rho).sqrt()
}

/// Evaluate the cold-plasma dispersion relation for parallel propagation (simplified).
///
/// Returns the squared refractive index n^2 for the left-hand (L) and right-hand (R)
/// circularly polarized modes at angular frequency `omega` (rad/s).
///
/// n^2 = 1 - omega_pe^2 / (omega * (omega +/- omega_ce))
///         - omega_pi^2 / (omega * (omega -/+ omega_ci))
///
/// Returns `(n2_left, n2_right)`.
pub fn dispersion_relation(
    omega: f64,
    electron_density_m3: f64,
    ion_density_m3: f64,
    b_tesla: f64,
    species: IonSpecies,
) -> (f64, f64) {
    let omega_pe_sq =
        electron_density_m3 * ELECTRON_CHARGE * ELECTRON_CHARGE / (EPSILON_0 * ELECTRON_MASS);
    let omega_ce =
        ELECTRON_CHARGE * b_tesla.abs() / ELECTRON_MASS;
    let q_ion = species.charge_number() * ELECTRON_CHARGE;
    let omega_pi_sq = ion_density_m3 * q_ion * q_ion / (EPSILON_0 * species.mass_kg());
    let omega_ci = q_ion * b_tesla.abs() / species.mass_kg();

    // Left-hand circular polarization (ion cyclotron branch):
    //   n^2_L = 1 - omega_pe^2 / (omega*(omega - omega_ce)) - omega_pi^2 / (omega*(omega + omega_ci))
    // Note: standard convention for L-mode uses (omega - omega_ce) for electrons
    // and (omega + omega_ci) for ions, which resonates at omega = omega_ci.
    let n2_left = 1.0
        - omega_pe_sq / (omega * (omega - omega_ce))
        - omega_pi_sq / (omega * (omega + omega_ci));

    // Right-hand circular polarization (electron cyclotron / whistler branch):
    //   n^2_R = 1 - omega_pe^2 / (omega*(omega + omega_ce)) - omega_pi^2 / (omega*(omega - omega_ci))
    let n2_right = 1.0
        - omega_pe_sq / (omega * (omega + omega_ce))
        - omega_pi_sq / (omega * (omega - omega_ci));

    (n2_left, n2_right)
}

/// Compute thermal (ion sound) speed: c_s = sqrt(k_B * T_i / m_i).
pub fn thermal_speed(temperature_ev: f64, species: IonSpecies) -> f64 {
    let t_joule = temperature_ev * EV_TO_JOULE;
    (t_joule / species.mass_kg()).sqrt()
}

/// Compute the Debye length: lambda_D = sqrt(epsilon_0 * k_B * T_e / (n_e * e^2)).
pub fn debye_length(electron_temperature_ev: f64, electron_density_m3: f64) -> f64 {
    let t_joule = electron_temperature_ev * EV_TO_JOULE;
    (EPSILON_0 * t_joule / (electron_density_m3 * ELECTRON_CHARGE * ELECTRON_CHARGE)).sqrt()
}

/// Compute the electron gyroradius (Larmor radius) in metres.
///
/// r_L = m_e * v_perp / (e * B), using thermal v_perp = sqrt(k_B * T_e / m_e).
pub fn electron_larmor_radius(electron_temperature_ev: f64, b_tesla: f64) -> f64 {
    let t_joule = electron_temperature_ev * EV_TO_JOULE;
    let v_perp = (t_joule / ELECTRON_MASS).sqrt();
    ELECTRON_MASS * v_perp / (ELECTRON_CHARGE * b_tesla.abs())
}

/// Compute the ion gyroradius (Larmor radius) in metres.
pub fn ion_larmor_radius(ion_temperature_ev: f64, b_tesla: f64, species: IonSpecies) -> f64 {
    let t_joule = ion_temperature_ev * EV_TO_JOULE;
    let v_perp = (t_joule / species.mass_kg()).sqrt();
    let q = species.charge_number() * ELECTRON_CHARGE;
    species.mass_kg() * v_perp / (q * b_tesla.abs())
}

// ---------------------------------------------------------------------------
// Gyrofrequency calculator
// ---------------------------------------------------------------------------

/// Computes cyclotron (gyro) frequencies for electrons and various ion species.
#[derive(Debug, Clone)]
pub struct GyrofrequencyCalculator {
    /// Background magnetic field in Tesla.
    b_tesla: f64,
}

impl GyrofrequencyCalculator {
    /// Create a new calculator for the given magnetic field magnitude.
    pub fn new(b_tesla: f64) -> Self {
        Self { b_tesla }
    }

    /// Electron cyclotron frequency in Hz.
    pub fn electron_gyrofrequency(&self) -> f64 {
        cyclotron_frequency(self.b_tesla, 1.0, ELECTRON_MASS)
    }

    /// Ion cyclotron frequency in Hz for the given species.
    pub fn ion_gyrofrequency(&self, species: IonSpecies) -> f64 {
        cyclotron_frequency(self.b_tesla, species.charge_number(), species.mass_kg())
    }

    /// Lower hybrid frequency (Hz), approximately:
    ///   f_LH = sqrt(f_ce * f_ci) for a hydrogen plasma (simplified).
    pub fn lower_hybrid_frequency(&self, species: IonSpecies) -> f64 {
        let f_ce = self.electron_gyrofrequency();
        let f_ci = self.ion_gyrofrequency(species);
        (f_ce * f_ci).sqrt()
    }

    /// Return the magnetic field strength in Tesla.
    pub fn magnetic_field(&self) -> f64 {
        self.b_tesla
    }
}

// ---------------------------------------------------------------------------
// Simple in-place FFT (radix-2 Cooley-Tukey, for internal use)
// ---------------------------------------------------------------------------

/// Compute a radix-2 FFT in place. `data` contains interleaved [re, im] pairs.
/// `n` must be a power of 2. `inverse` = true for IFFT.
fn fft_in_place(data: &mut [f64], n: usize, inverse: bool) {
    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 0..n {
        if i < j {
            data.swap(2 * i, 2 * j);
            data.swap(2 * i + 1, 2 * j + 1);
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Cooley-Tukey butterfly
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / len as f64;
        let wn_re = angle.cos();
        let wn_im = angle.sin();
        let mut start = 0;
        while start < n {
            let mut w_re = 1.0;
            let mut w_im = 0.0;
            for k in 0..half {
                let even = start + k;
                let odd = start + k + half;
                let tre = data[2 * odd] * w_re - data[2 * odd + 1] * w_im;
                let tim = data[2 * odd] * w_im + data[2 * odd + 1] * w_re;
                data[2 * odd] = data[2 * even] - tre;
                data[2 * odd + 1] = data[2 * even + 1] - tim;
                data[2 * even] += tre;
                data[2 * even + 1] += tim;
                let new_w_re = w_re * wn_re - w_im * wn_im;
                let new_w_im = w_re * wn_im + w_im * wn_re;
                w_re = new_w_re;
                w_im = new_w_im;
            }
            start += len;
        }
        len <<= 1;
    }

    if inverse {
        let inv_n = 1.0 / n as f64;
        for val in data.iter_mut() {
            *val *= inv_n;
        }
    }
}

/// Return the next power of two >= n.
fn next_power_of_two(n: usize) -> usize {
    let mut p = 1;
    while p < n {
        p <<= 1;
    }
    p
}

/// Compute the power spectral density of a real-valued signal.
/// Returns `(frequencies_hz, psd)` where `psd` is in units^2/Hz.
/// Only the non-negative frequency bins (0..N/2+1) are returned.
fn compute_psd(signal: &[f64], sample_rate_hz: f64) -> (Vec<f64>, Vec<f64>) {
    let n = next_power_of_two(signal.len());
    let mut data = vec![0.0; 2 * n]; // interleaved re/im
    for (i, &s) in signal.iter().enumerate() {
        data[2 * i] = s;
    }
    fft_in_place(&mut data, n, false);

    let n_out = n / 2 + 1;
    let df = sample_rate_hz / n as f64;
    let norm = 1.0 / (sample_rate_hz * n as f64);
    let mut freq = Vec::with_capacity(n_out);
    let mut psd = Vec::with_capacity(n_out);
    for k in 0..n_out {
        freq.push(k as f64 * df);
        let re = data[2 * k];
        let im = data[2 * k + 1];
        let power = (re * re + im * im) * norm;
        // Double one-sided spectrum (except DC and Nyquist)
        let scale = if k == 0 || k == n / 2 { 1.0 } else { 2.0 };
        psd.push(power * scale);
    }
    (freq, psd)
}

/// Compute the complex FFT of a real signal and return (re, im) pairs per bin.
/// Only bins 0..N/2+1 are returned.
fn compute_fft_complex(signal: &[f64], n: usize) -> Vec<(f64, f64)> {
    let mut data = vec![0.0; 2 * n];
    for (i, &s) in signal.iter().enumerate().take(n) {
        data[2 * i] = s;
    }
    fft_in_place(&mut data, n, false);
    let n_out = n / 2 + 1;
    let mut result = Vec::with_capacity(n_out);
    for k in 0..n_out {
        result.push((data[2 * k], data[2 * k + 1]));
    }
    result
}

// ---------------------------------------------------------------------------
// Polarization
// ---------------------------------------------------------------------------

/// Polarization handedness of a wave.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Polarization {
    /// Left-hand circular polarization (ion cyclotron sense).
    LeftHand,
    /// Right-hand circular polarization (electron cyclotron sense).
    RightHand,
    /// Linear polarization (degenerate case).
    Linear,
}

/// Result of polarization analysis at a single frequency bin.
#[derive(Debug, Clone, Copy)]
pub struct PolarizationResult {
    /// Frequency in Hz.
    pub frequency_hz: f64,
    /// Ellipticity: +1 = pure LH circular, -1 = pure RH circular, 0 = linear.
    pub ellipticity: f64,
    /// Degree of polarization (0..1).
    pub degree_of_polarization: f64,
    /// Detected handedness.
    pub polarization: Polarization,
    /// Power spectral density at this bin.
    pub power: f64,
}

// ---------------------------------------------------------------------------
// Polarization analyzer
// ---------------------------------------------------------------------------

/// Analyzes the polarization of magnetic field oscillations from two orthogonal
/// transverse components (e.g., Bx and By perpendicular to the background field).
///
/// Uses the spectral matrix method: for each frequency bin, the 2x2 cross-spectral
/// matrix is formed from Bx and By, and the ellipticity is derived from the
/// imaginary part of the cross-spectrum.
#[derive(Debug, Clone)]
pub struct PolarizationAnalyzer {
    /// Sample rate in Hz.
    sample_rate_hz: f64,
}

impl PolarizationAnalyzer {
    /// Create a new polarization analyzer.
    pub fn new(sample_rate_hz: f64) -> Self {
        Self { sample_rate_hz }
    }

    /// Analyze polarization from two transverse magnetic field components.
    ///
    /// `bx` and `by` are time series of equal length representing two orthogonal
    /// components perpendicular to the background magnetic field direction.
    ///
    /// Returns a vector of `PolarizationResult` for each frequency bin.
    pub fn analyze(&self, bx: &[f64], by: &[f64]) -> Vec<PolarizationResult> {
        assert_eq!(bx.len(), by.len(), "bx and by must have the same length");
        let n = next_power_of_two(bx.len());
        let fft_bx = compute_fft_complex(bx, n);
        let fft_by = compute_fft_complex(by, n);

        let df = self.sample_rate_hz / n as f64;
        let n_out = fft_bx.len();
        let mut results = Vec::with_capacity(n_out);

        for k in 0..n_out {
            let (bx_re, bx_im) = fft_bx[k];
            let (by_re, by_im) = fft_by[k];

            // Auto-spectra
            let sxx = bx_re * bx_re + bx_im * bx_im;
            let syy = by_re * by_re + by_im * by_im;

            // Cross-spectrum Sxy = Bx * conj(By)
            let sxy_re = bx_re * by_re + bx_im * by_im;
            let sxy_im = bx_im * by_re - bx_re * by_im;

            let total_power = sxx + syy;
            let freq = k as f64 * df;

            if total_power < 1e-30 {
                results.push(PolarizationResult {
                    frequency_hz: freq,
                    ellipticity: 0.0,
                    degree_of_polarization: 0.0,
                    polarization: Polarization::Linear,
                    power: 0.0,
                });
                continue;
            }

            // Ellipticity from the imaginary part of the cross-spectrum:
            //   tan(beta) = Im(Sxy) / sqrt(Sxx * Syy)
            // where beta is the ellipticity angle. Positive => LH, negative => RH.
            let denom = (sxx * syy).sqrt();
            let sin_2beta = if denom > 1e-30 {
                sxy_im / denom
            } else {
                0.0
            };
            // Clamp to [-1, 1] for numerical safety
            let sin_2beta = sin_2beta.clamp(-1.0, 1.0);
            // ellipticity in [-1, 1]: +1 = pure LH circular, -1 = pure RH circular
            let ellipticity = sin_2beta;

            // Degree of polarization:
            //   dop = sqrt((Sxx - Syy)^2 + 4*|Sxy|^2) / (Sxx + Syy)
            let sxy_mag_sq = sxy_re * sxy_re + sxy_im * sxy_im;
            let dop = ((sxx - syy) * (sxx - syy) + 4.0 * sxy_mag_sq).sqrt() / total_power;
            let dop = dop.clamp(0.0, 1.0);

            let polarization = if ellipticity.abs() < 0.1 {
                Polarization::Linear
            } else if ellipticity > 0.0 {
                Polarization::LeftHand
            } else {
                Polarization::RightHand
            };

            results.push(PolarizationResult {
                frequency_hz: freq,
                ellipticity,
                degree_of_polarization: dop,
                polarization,
                power: total_power,
            });
        }

        results
    }

    /// Return the sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate_hz
    }
}

// ---------------------------------------------------------------------------
// Wave mode identification
// ---------------------------------------------------------------------------

/// Identified wave mode from spectral analysis.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaveMode {
    /// Electromagnetic ion cyclotron (EMIC) wave.
    IonCyclotron,
    /// Shear (transverse) Alfven wave.
    ShearAlfven,
    /// Compressional (fast magnetosonic) wave.
    Compressional,
    /// Whistler-mode wave.
    Whistler,
    /// Pi2 pulsation (substorm onset indicator), period 40-150 s.
    Pi2Pulsation,
    /// Pc5 pulsation, period 150-600 s.
    Pc5Pulsation,
    /// Unclassified wave.
    Unknown,
}

/// A detected wave event with associated parameters.
#[derive(Debug, Clone)]
pub struct WaveDetection {
    /// Identified mode.
    pub mode: WaveMode,
    /// Peak frequency in Hz.
    pub peak_frequency_hz: f64,
    /// Peak power spectral density.
    pub peak_power: f64,
    /// Polarization at the peak frequency.
    pub polarization: Polarization,
    /// Ellipticity at the peak.
    pub ellipticity: f64,
    /// Frequency band lower bound (Hz).
    pub band_low_hz: f64,
    /// Frequency band upper bound (Hz).
    pub band_high_hz: f64,
}

// ---------------------------------------------------------------------------
// Alfven wave detector
// ---------------------------------------------------------------------------

/// Detects Alfven wave modes and electromagnetic ion cyclotron (EMIC) waves
/// from magnetic field time series.
///
/// This detector analyzes three-component magnetometer data (Bx, By, Bz)
/// where Bz is along the background field direction. It computes the PSD,
/// identifies spectral peaks, and classifies them based on frequency relative
/// to the ion cyclotron frequency and the observed polarization.
#[derive(Debug, Clone)]
pub struct AlfvenWaveDetector {
    /// Sample rate of the magnetometer data in Hz.
    sample_rate_hz: f64,
    /// Background magnetic field in Tesla (for gyrofrequency reference).
    b0_tesla: f64,
    /// Detection threshold: minimum PSD relative to noise floor to declare a peak.
    threshold_db: f64,
    /// Dominant ion species.
    species: IonSpecies,
}

impl AlfvenWaveDetector {
    /// Create a new Alfven wave detector.
    ///
    /// - `sample_rate_hz`: sample rate of the time series
    /// - `b0_tesla`: background magnetic field magnitude
    /// - `threshold_db`: minimum power above noise floor (dB) to detect a peak
    /// - `species`: dominant ion species for gyrofrequency reference
    pub fn new(
        sample_rate_hz: f64,
        b0_tesla: f64,
        threshold_db: f64,
        species: IonSpecies,
    ) -> Self {
        Self {
            sample_rate_hz,
            b0_tesla,
            threshold_db,
            species,
        }
    }

    /// Detect wave modes from three-component magnetic field data.
    ///
    /// `bx`, `by`: transverse components; `bz`: field-aligned component.
    /// All arrays must have the same length.
    pub fn detect(&self, bx: &[f64], by: &[f64], bz: &[f64]) -> Vec<WaveDetection> {
        assert_eq!(bx.len(), by.len());
        assert_eq!(bx.len(), bz.len());

        let f_ci = cyclotron_frequency(self.b0_tesla, self.species.charge_number(), self.species.mass_kg());

        // PSD of transverse and compressional components
        let (freqs_t, psd_bx) = compute_psd(bx, self.sample_rate_hz);
        let (_, psd_by) = compute_psd(by, self.sample_rate_hz);
        let (_, psd_bz) = compute_psd(bz, self.sample_rate_hz);

        // Total transverse PSD
        let psd_trans: Vec<f64> = psd_bx.iter().zip(psd_by.iter()).map(|(a, b)| a + b).collect();

        // Noise floor estimate: median of the PSD
        let noise_floor_trans = median_value(&psd_trans);
        let noise_floor_comp = median_value(&psd_bz);

        let threshold_linear = 10.0_f64.powf(self.threshold_db / 10.0);

        // Polarization analysis
        let pol_analyzer = PolarizationAnalyzer::new(self.sample_rate_hz);
        let pol_results = pol_analyzer.analyze(bx, by);

        let mut detections = Vec::new();

        // Find peaks in transverse PSD
        let trans_peaks = find_spectral_peaks(&freqs_t, &psd_trans, noise_floor_trans * threshold_linear);
        for &(peak_idx, peak_freq, peak_power) in &trans_peaks {
            let pol = if peak_idx < pol_results.len() {
                &pol_results[peak_idx]
            } else {
                continue;
            };

            let mode = classify_transverse_mode(peak_freq, f_ci, pol.ellipticity);
            let (bw_low, bw_high) = estimate_peak_bandwidth(&freqs_t, &psd_trans, peak_idx, noise_floor_trans * threshold_linear);

            detections.push(WaveDetection {
                mode,
                peak_frequency_hz: peak_freq,
                peak_power,
                polarization: pol.polarization,
                ellipticity: pol.ellipticity,
                band_low_hz: bw_low,
                band_high_hz: bw_high,
            });
        }

        // Find peaks in compressional (Bz) PSD
        let comp_peaks = find_spectral_peaks(&freqs_t, &psd_bz, noise_floor_comp * threshold_linear);
        for &(peak_idx, peak_freq, peak_power) in &comp_peaks {
            // Check if this is dominantly compressional (Bz power > transverse)
            let trans_power_at_peak = if peak_idx < psd_trans.len() {
                psd_trans[peak_idx]
            } else {
                0.0
            };

            if peak_power > trans_power_at_peak {
                let mode = classify_compressional_mode(peak_freq, f_ci);
                let (bw_low, bw_high) = estimate_peak_bandwidth(&freqs_t, &psd_bz, peak_idx, noise_floor_comp * threshold_linear);
                detections.push(WaveDetection {
                    mode,
                    peak_frequency_hz: peak_freq,
                    peak_power,
                    polarization: Polarization::Linear,
                    ellipticity: 0.0,
                    band_low_hz: bw_low,
                    band_high_hz: bw_high,
                });
            }
        }

        detections
    }

    /// Return the ion cyclotron frequency for the configured species and field.
    pub fn ion_cyclotron_frequency(&self) -> f64 {
        cyclotron_frequency(self.b0_tesla, self.species.charge_number(), self.species.mass_kg())
    }
}

/// Classify a transverse peak based on frequency relative to f_ci and polarization.
fn classify_transverse_mode(freq_hz: f64, f_ci: f64, ellipticity: f64) -> WaveMode {
    let period_s = if freq_hz > 0.0 { 1.0 / freq_hz } else { f64::INFINITY };

    // Pi2 pulsation: period 40-150 s (freq ~0.0067 - 0.025 Hz)
    if period_s >= 40.0 && period_s <= 150.0 {
        return WaveMode::Pi2Pulsation;
    }
    // Pc5 pulsation: period 150-600 s
    if period_s >= 150.0 && period_s <= 600.0 {
        return WaveMode::Pc5Pulsation;
    }

    // EMIC: frequency near but below f_ci, typically LH polarized
    if freq_hz > 0.1 * f_ci && freq_hz < 1.0 * f_ci && ellipticity > 0.1 {
        return WaveMode::IonCyclotron;
    }

    // Shear Alfven: frequency well below f_ci
    if freq_hz < 0.1 * f_ci {
        return WaveMode::ShearAlfven;
    }

    // Whistler: frequency above f_ci, RH polarized
    if freq_hz > f_ci && ellipticity < -0.1 {
        return WaveMode::Whistler;
    }

    WaveMode::Unknown
}

/// Classify a compressional peak.
fn classify_compressional_mode(freq_hz: f64, f_ci: f64) -> WaveMode {
    let period_s = if freq_hz > 0.0 { 1.0 / freq_hz } else { f64::INFINITY };

    if period_s >= 40.0 && period_s <= 150.0 {
        return WaveMode::Pi2Pulsation;
    }
    if period_s >= 150.0 && period_s <= 600.0 {
        return WaveMode::Pc5Pulsation;
    }

    // Compressional (fast mode) for frequencies below f_ci
    if freq_hz < f_ci {
        return WaveMode::Compressional;
    }

    WaveMode::Unknown
}

/// Find spectral peaks above a given threshold.
/// Returns a vector of (bin_index, frequency_hz, power).
fn find_spectral_peaks(freqs: &[f64], psd: &[f64], threshold: f64) -> Vec<(usize, f64, f64)> {
    let mut peaks = Vec::new();
    let n = psd.len();
    if n < 3 {
        return peaks;
    }
    for i in 1..n - 1 {
        if psd[i] > threshold && psd[i] > psd[i - 1] && psd[i] > psd[i + 1] {
            peaks.push((i, freqs[i], psd[i]));
        }
    }
    peaks
}

/// Estimate the bandwidth of a spectral peak (indices where power drops below threshold).
fn estimate_peak_bandwidth(
    freqs: &[f64],
    psd: &[f64],
    peak_idx: usize,
    threshold: f64,
) -> (f64, f64) {
    let mut low = peak_idx;
    while low > 0 && psd[low - 1] > threshold {
        low -= 1;
    }
    let mut high = peak_idx;
    while high + 1 < psd.len() && psd[high + 1] > threshold {
        high += 1;
    }
    (freqs[low], freqs[high])
}

/// Compute the median of a slice.
fn median_value(data: &[f64]) -> f64 {
    if data.is_empty() {
        return 0.0;
    }
    let mut sorted: Vec<f64> = data.to_vec();
    sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mid = sorted.len() / 2;
    if sorted.len() % 2 == 0 {
        (sorted[mid - 1] + sorted[mid]) / 2.0
    } else {
        sorted[mid]
    }
}

// ---------------------------------------------------------------------------
// Wave mode separator
// ---------------------------------------------------------------------------

/// Result of compressional/transverse mode separation.
#[derive(Debug, Clone)]
pub struct ModeSeparation {
    /// Compressional (field-aligned) power per frequency bin.
    pub compressional_psd: Vec<f64>,
    /// Transverse power per frequency bin.
    pub transverse_psd: Vec<f64>,
    /// Frequency axis in Hz.
    pub frequencies_hz: Vec<f64>,
    /// Ratio of compressional to total power per bin (0..1).
    pub compressional_ratio: Vec<f64>,
}

/// Separates compressional and transverse wave modes from three-component
/// magnetic field data.
///
/// Compressional power comes from the field-aligned component (Bz),
/// while transverse power is the sum of the two perpendicular components (Bx, By).
#[derive(Debug, Clone)]
pub struct WaveModeSeparator {
    /// Sample rate in Hz.
    sample_rate_hz: f64,
}

impl WaveModeSeparator {
    /// Create a new mode separator.
    pub fn new(sample_rate_hz: f64) -> Self {
        Self { sample_rate_hz }
    }

    /// Separate compressional and transverse modes.
    ///
    /// `bx`, `by`: transverse components; `bz`: field-aligned component.
    pub fn separate(&self, bx: &[f64], by: &[f64], bz: &[f64]) -> ModeSeparation {
        assert_eq!(bx.len(), by.len());
        assert_eq!(bx.len(), bz.len());

        let (freqs, psd_bx) = compute_psd(bx, self.sample_rate_hz);
        let (_, psd_by) = compute_psd(by, self.sample_rate_hz);
        let (_, psd_bz) = compute_psd(bz, self.sample_rate_hz);

        let n = freqs.len();
        let mut transverse_psd = Vec::with_capacity(n);
        let mut compressional_ratio = Vec::with_capacity(n);

        for i in 0..n {
            let trans = psd_bx[i] + psd_by[i];
            let total = trans + psd_bz[i];
            transverse_psd.push(trans);
            compressional_ratio.push(if total > 1e-30 { psd_bz[i] / total } else { 0.0 });
        }

        ModeSeparation {
            compressional_psd: psd_bz,
            transverse_psd,
            frequencies_hz: freqs,
            compressional_ratio,
        }
    }

    /// Return the sample rate.
    pub fn sample_rate(&self) -> f64 {
        self.sample_rate_hz
    }
}

// ---------------------------------------------------------------------------
// Geomagnetic index estimator
// ---------------------------------------------------------------------------

/// Estimated geomagnetic indices.
#[derive(Debug, Clone, Copy)]
pub struct GeomagneticIndices {
    /// Estimated Kp index (0.0 - 9.0).
    pub kp: f64,
    /// Estimated Dst index in nT (typically negative during storms).
    pub dst_nt: f64,
    /// Peak-to-peak perturbation in the horizontal component (nT).
    pub delta_h_nt: f64,
    /// RMS perturbation (nT).
    pub rms_perturbation_nt: f64,
}

/// Estimates geomagnetic activity indices from magnetometer data.
///
/// The Kp index is estimated from the range (max-min) of the horizontal
/// magnetic field perturbation over a 3-hour interval, mapped through
/// the standard quasi-logarithmic K-scale.
///
/// The Dst index is estimated from the mean depression of the horizontal
/// component relative to a quiet-day baseline.
#[derive(Debug, Clone)]
pub struct GeomagneticIndexEstimator {
    /// Geomagnetic latitude of the station (degrees). Used to scale K thresholds.
    geomag_latitude_deg: f64,
    /// K-index lower-bound thresholds in nT for values 0 through 9.
    k_thresholds_nt: [f64; 10],
}

/// Default K-index thresholds for a mid-latitude station (~50 deg geomag latitude).
const DEFAULT_K_THRESHOLDS: [f64; 10] = [
    0.0, 5.0, 10.0, 20.0, 40.0, 70.0, 120.0, 200.0, 330.0, 500.0,
];

impl GeomagneticIndexEstimator {
    /// Create a new estimator for a station at the given geomagnetic latitude.
    ///
    /// K thresholds are scaled by `cos(latitude)` relative to 50 deg reference.
    pub fn new(geomag_latitude_deg: f64) -> Self {
        let ref_cos = (50.0_f64.to_radians()).cos();
        let station_cos = geomag_latitude_deg.to_radians().cos();
        let scale = if station_cos.abs() > 1e-6 {
            ref_cos / station_cos
        } else {
            1.0
        };
        let mut thresholds = DEFAULT_K_THRESHOLDS;
        for t in thresholds.iter_mut() {
            *t *= scale;
        }
        Self {
            geomag_latitude_deg,
            k_thresholds_nt: thresholds,
        }
    }

    /// Create with custom K thresholds.
    pub fn with_thresholds(geomag_latitude_deg: f64, thresholds: [f64; 10]) -> Self {
        Self {
            geomag_latitude_deg,
            k_thresholds_nt: thresholds,
        }
    }

    /// Estimate geomagnetic indices from horizontal-component perturbation data.
    ///
    /// `delta_h` is the perturbation of the horizontal magnetic field component
    /// (in nT) after removing the quiet-day baseline. It represents a single
    /// 3-hour interval for K-index estimation.
    ///
    /// `baseline_h_nt` is the quiet-day baseline value (mean H during quiet time).
    /// It is used for Dst estimation: Dst ~ mean(delta_h) / cos(latitude).
    pub fn estimate(&self, delta_h: &[f64], baseline_h_nt: f64) -> GeomagneticIndices {
        if delta_h.is_empty() {
            return GeomagneticIndices {
                kp: 0.0,
                dst_nt: 0.0,
                delta_h_nt: 0.0,
                rms_perturbation_nt: 0.0,
            };
        }

        // Peak-to-peak range
        let mut min_val = f64::INFINITY;
        let mut max_val = f64::NEG_INFINITY;
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        for &v in delta_h {
            if v < min_val {
                min_val = v;
            }
            if v > max_val {
                max_val = v;
            }
            sum += v;
            sum_sq += v * v;
        }
        let n = delta_h.len() as f64;
        let range = max_val - min_val;
        let mean = sum / n;
        let rms = (sum_sq / n).sqrt();

        // K-index from range
        let mut k = 0u8;
        for i in (0..10).rev() {
            if range >= self.k_thresholds_nt[i] {
                k = i as u8;
                break;
            }
        }

        // Kp as a continuous value: interpolate within the K bin
        let kp = if (k as usize) < 9 {
            let low = self.k_thresholds_nt[k as usize];
            let high = self.k_thresholds_nt[k as usize + 1];
            let frac = if (high - low).abs() > 1e-10 {
                ((range - low) / (high - low)).clamp(0.0, 1.0)
            } else {
                0.0
            };
            k as f64 + frac
        } else {
            9.0
        };

        // Dst estimation: mean perturbation corrected for latitude
        // Dst = mean(delta_H) / cos(geomag_lat)
        // (simplified single-station estimate)
        let cos_lat = self.geomag_latitude_deg.to_radians().cos();
        let _baseline = baseline_h_nt; // reserved for more complex models
        let dst = if cos_lat.abs() > 1e-6 {
            mean / cos_lat
        } else {
            mean
        };

        GeomagneticIndices {
            kp,
            dst_nt: dst,
            delta_h_nt: range,
            rms_perturbation_nt: rms,
        }
    }

    /// Return the station geomagnetic latitude.
    pub fn latitude(&self) -> f64 {
        self.geomag_latitude_deg
    }

    /// Return the K-index thresholds.
    pub fn k_thresholds(&self) -> &[f64; 10] {
        &self.k_thresholds_nt
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const TOLERANCE: f64 = 1e-6;

    // -- Ion species tests --

    #[test]
    fn test_proton_mass() {
        let species = IonSpecies::Proton;
        assert!((species.mass_kg() - PROTON_MASS).abs() < 1e-40);
        assert!((species.charge_number() - 1.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_helium_mass() {
        let species = IonSpecies::HeliumPlus;
        let expected = 4.0 * AMU;
        assert!((species.mass_kg() - expected).abs() / expected < 1e-6);
    }

    #[test]
    fn test_oxygen_mass() {
        let species = IonSpecies::OxygenPlus;
        let expected = 16.0 * AMU;
        assert!((species.mass_kg() - expected).abs() / expected < 1e-6);
    }

    #[test]
    fn test_custom_species() {
        let species = IonSpecies::Custom {
            mass_kg: 2.0 * PROTON_MASS,
            charge_number: 1.0,
        };
        assert!((species.mass_kg() - 2.0 * PROTON_MASS).abs() < 1e-40);
        assert!((species.charge_number() - 1.0).abs() < TOLERANCE);
    }

    // -- Cyclotron frequency tests --

    #[test]
    fn test_electron_cyclotron_frequency() {
        // f_ce = e * B / (2*pi*m_e)
        let b = 100e-9; // 100 nT
        let f_ce = cyclotron_frequency(b, 1.0, ELECTRON_MASS);
        let expected = ELECTRON_CHARGE * b / (2.0 * PI * ELECTRON_MASS);
        assert!((f_ce - expected).abs() / expected < 1e-10);
        // Should be around 2.8 kHz for 100 nT
        assert!(f_ce > 2000.0 && f_ce < 3500.0);
    }

    #[test]
    fn test_proton_cyclotron_frequency() {
        let b = 100e-9;
        let f_ci = cyclotron_frequency(b, 1.0, PROTON_MASS);
        let expected = ELECTRON_CHARGE * b / (2.0 * PI * PROTON_MASS);
        assert!((f_ci - expected).abs() / expected < 1e-10);
        // Proton gyrofrequency at 100 nT ~ 1.5 Hz
        assert!(f_ci > 1.0 && f_ci < 2.0);
    }

    #[test]
    fn test_gyrofrequency_calculator() {
        let b = 200e-9;
        let calc = GyrofrequencyCalculator::new(b);
        let f_ce = calc.electron_gyrofrequency();
        let f_ci = calc.ion_gyrofrequency(IonSpecies::Proton);
        // Electron gyrofreq >> ion gyrofreq
        assert!(f_ce > f_ci * 1000.0);
        assert!((calc.magnetic_field() - b).abs() < 1e-20);
    }

    #[test]
    fn test_lower_hybrid_frequency() {
        let b = 200e-9;
        let calc = GyrofrequencyCalculator::new(b);
        let f_lh = calc.lower_hybrid_frequency(IonSpecies::Proton);
        let f_ce = calc.electron_gyrofrequency();
        let f_ci = calc.ion_gyrofrequency(IonSpecies::Proton);
        let expected = (f_ce * f_ci).sqrt();
        assert!((f_lh - expected).abs() / expected < 1e-10);
        // Should be between ion and electron gyrofrequencies
        assert!(f_lh > f_ci);
        assert!(f_lh < f_ce);
    }

    #[test]
    fn test_cyclotron_angular_frequency() {
        let b = 100e-9;
        let omega = cyclotron_angular_frequency(b, 1.0, PROTON_MASS);
        let f = cyclotron_frequency(b, 1.0, PROTON_MASS);
        assert!((omega - 2.0 * PI * f).abs() / omega < 1e-10);
    }

    // -- Plasma frequency tests --

    #[test]
    fn test_plasma_frequency() {
        let ne = 1e7; // 10^7 m^-3 (typical magnetosphere)
        let f_pe = plasma_frequency(ne);
        // f_pe ~ 9 * sqrt(ne) ~ 9 * sqrt(1e7) ~ 28.5 kHz ... wait, more precisely:
        // f_pe = (1/2pi) * sqrt(ne * e^2 / (eps0 * me)) ~ 898 kHz for 1e7
        // Actually for ne=1e7: f_pe ~ 898 Hz... Let me compute:
        // omega_pe^2 = 1e7 * (1.6e-19)^2 / (8.85e-12 * 9.1e-31) = 1e7 * 2.56e-38 / 8.06e-42
        // = 1e7 * 3175 = 3.175e10
        // omega_pe = 178266 rad/s => f_pe = 28373 Hz
        // So ~28 kHz
        assert!(f_pe > 20_000.0 && f_pe < 40_000.0);
    }

    #[test]
    fn test_ion_plasma_frequency() {
        let ni = 1e7;
        let f_pi = ion_plasma_frequency(ni, IonSpecies::Proton);
        let f_pe = plasma_frequency(ni);
        // Ion plasma frequency << electron plasma frequency by sqrt(me/mp) factor
        let ratio = f_pe / f_pi;
        let expected_ratio = (PROTON_MASS / ELECTRON_MASS).sqrt();
        assert!((ratio - expected_ratio).abs() / expected_ratio < 1e-3);
    }

    // -- Alfven speed tests --

    #[test]
    fn test_alfven_speed() {
        let b = 100e-9; // 100 nT
        let ni = 1e7;   // 10^7 m^-3
        let v_a = alfven_speed(b, ni, IonSpecies::Proton);
        // v_A = B / sqrt(mu0 * ni * mp)
        let expected = b / (MU_0 * ni * PROTON_MASS).sqrt();
        assert!((v_a - expected).abs() / expected < 1e-10);
        // Typical magnetospheric Alfven speed: ~10^5 - 10^7 m/s
        assert!(v_a > 1e4);
    }

    #[test]
    fn test_alfven_speed_heavier_ion() {
        let b = 100e-9;
        let ni = 1e7;
        let v_h = alfven_speed(b, ni, IonSpecies::Proton);
        let v_o = alfven_speed(b, ni, IonSpecies::OxygenPlus);
        // Heavier ion => lower Alfven speed (v_A ~ 1/sqrt(m_i))
        assert!(v_o < v_h);
        let ratio = v_h / v_o;
        let expected_ratio = (16.0_f64 * AMU / PROTON_MASS).sqrt();
        assert!((ratio - expected_ratio).abs() / expected_ratio < 1e-3);
    }

    // -- Thermal speed and Debye length --

    #[test]
    fn test_thermal_speed() {
        let v = thermal_speed(1000.0, IonSpecies::Proton);
        // v_th = sqrt(kT/m) with T in eV => T_J = 1000 * 1.6e-19 = 1.6e-16
        // v_th = sqrt(1.6e-16 / 1.67e-27) = sqrt(9.58e10) ~ 3.1e5 m/s
        assert!(v > 2e5 && v < 5e5);
    }

    #[test]
    fn test_debye_length() {
        let lambda = debye_length(100.0, 1e7);
        // lambda_D = sqrt(eps0 * kT / (ne * e^2))
        // with T_J = 100 * 1.6e-19 = 1.6e-17
        // = sqrt(8.85e-12 * 1.6e-17 / (1e7 * 2.56e-38))
        // = sqrt(1.416e-28 / 2.56e-31) = sqrt(553) ~ 23.5 m
        assert!(lambda > 10.0 && lambda < 100.0);
    }

    #[test]
    fn test_electron_larmor_radius() {
        let r = electron_larmor_radius(100.0, 100e-9);
        // Should be on the order of km for 100 eV electrons in 100 nT field
        assert!(r > 100.0); // > 100 m
    }

    #[test]
    fn test_ion_larmor_radius() {
        let r = ion_larmor_radius(1000.0, 100e-9, IonSpecies::Proton);
        // Ion Larmor radius >> electron Larmor radius
        let r_e = electron_larmor_radius(1000.0, 100e-9);
        assert!(r > r_e);
    }

    // -- Dispersion relation test --

    #[test]
    fn test_dispersion_relation_high_frequency() {
        let b = 100e-9;
        let ne = 1e7;
        let ni = 1e7;
        // At very high frequency (well above all characteristic frequencies),
        // n^2 should approach 1.
        let omega = 2.0 * PI * 1e9; // 1 GHz
        let (n2_l, n2_r) = dispersion_relation(omega, ne, ni, b, IonSpecies::Proton);
        assert!((n2_l - 1.0).abs() < 0.01);
        assert!((n2_r - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_dispersion_relation_l_r_differ() {
        let b = 100e-9;
        let ne = 1e7;
        let ni = 1e7;
        // At intermediate frequency, L and R modes should differ
        let f_ce = cyclotron_frequency(b, 1.0, ELECTRON_MASS);
        let omega = 2.0 * PI * f_ce * 0.5; // Half the electron cyclotron freq
        let (n2_l, n2_r) = dispersion_relation(omega, ne, ni, b, IonSpecies::Proton);
        assert!((n2_l - n2_r).abs() > 0.001);
    }

    // -- Plasma parameters default --

    #[test]
    fn test_plasma_parameters_default() {
        let p = PlasmaParameters::default();
        assert!(p.electron_density_m3 > 0.0);
        assert!(p.magnetic_field_t > 0.0);
        assert!(p.electron_temperature_ev > 0.0);
    }

    // -- FFT internal tests --

    #[test]
    fn test_fft_dc_signal() {
        // A DC signal should produce power only at bin 0
        let n = 64;
        let signal = vec![1.0; n];
        let (freqs, psd) = compute_psd(&signal, 100.0);
        assert_eq!(freqs.len(), n / 2 + 1);
        // DC bin should have dominant power
        let max_idx = psd.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        assert_eq!(max_idx, 0);
    }

    #[test]
    fn test_fft_sine_signal() {
        let n = 256;
        let fs = 256.0;
        let f_tone = 32.0; // Hz
        let signal: Vec<f64> = (0..n)
            .map(|i| (2.0 * PI * f_tone * i as f64 / fs).sin())
            .collect();
        let (freqs, psd) = compute_psd(&signal, fs);
        // Find the peak frequency
        let max_idx = psd.iter().enumerate().skip(1).max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        let peak_freq = freqs[max_idx];
        assert!((peak_freq - f_tone).abs() < fs / n as f64 + 0.01);
    }

    #[test]
    fn test_next_power_of_two() {
        assert_eq!(next_power_of_two(1), 1);
        assert_eq!(next_power_of_two(2), 2);
        assert_eq!(next_power_of_two(3), 4);
        assert_eq!(next_power_of_two(5), 8);
        assert_eq!(next_power_of_two(256), 256);
        assert_eq!(next_power_of_two(257), 512);
    }

    // -- Polarization analyzer tests --

    #[test]
    fn test_polarization_lh_circular() {
        // Generate a left-hand circularly polarized signal:
        // Bx = cos(2pi*f*t), By = sin(2pi*f*t)  [LH convention with our cross-spectrum sign]
        let n = 256;
        let fs = 256.0;
        let f = 16.0;
        let bx: Vec<f64> = (0..n).map(|i| (2.0 * PI * f * i as f64 / fs).cos()).collect();
        let by: Vec<f64> = (0..n).map(|i| (2.0 * PI * f * i as f64 / fs).sin()).collect();

        let analyzer = PolarizationAnalyzer::new(fs);
        let results = analyzer.analyze(&bx, &by);

        // Find the bin closest to 16 Hz
        let target_bin = (f / (fs / next_power_of_two(n) as f64)).round() as usize;
        let r = &results[target_bin];
        assert!(r.power > 0.0);
        assert!(r.degree_of_polarization > 0.5);
        // Should be LH (positive ellipticity)
        assert!(r.ellipticity > 0.0, "Expected positive ellipticity for LH, got {}", r.ellipticity);
    }

    #[test]
    fn test_polarization_rh_circular() {
        // RH circular: Bx = cos(2pi*f*t), By = -sin(2pi*f*t)
        let n = 256;
        let fs = 256.0;
        let f = 16.0;
        let bx: Vec<f64> = (0..n).map(|i| (2.0 * PI * f * i as f64 / fs).cos()).collect();
        let by: Vec<f64> = (0..n).map(|i| -(2.0 * PI * f * i as f64 / fs).sin()).collect();

        let analyzer = PolarizationAnalyzer::new(fs);
        let results = analyzer.analyze(&bx, &by);

        let target_bin = (f / (fs / next_power_of_two(n) as f64)).round() as usize;
        let r = &results[target_bin];
        assert!(r.power > 0.0);
        assert!(r.ellipticity < 0.0, "Expected negative ellipticity for RH, got {}", r.ellipticity);
    }

    #[test]
    fn test_polarization_linear() {
        // Linear: Bx = cos(2pi*f*t), By = 0
        let n = 256;
        let fs = 256.0;
        let f = 16.0;
        let bx: Vec<f64> = (0..n).map(|i| (2.0 * PI * f * i as f64 / fs).cos()).collect();
        let by = vec![0.0; n];

        let analyzer = PolarizationAnalyzer::new(fs);
        let results = analyzer.analyze(&bx, &by);
        assert_eq!(analyzer.sample_rate(), fs);

        let target_bin = (f / (fs / next_power_of_two(n) as f64)).round() as usize;
        let r = &results[target_bin];
        assert!(r.ellipticity.abs() < 0.2, "Expected near-zero ellipticity for linear, got {}", r.ellipticity);
    }

    // -- Wave mode separator test --

    #[test]
    fn test_wave_mode_separator_pure_compressional() {
        // Signal only in Bz (compressional)
        let n = 256;
        let fs = 256.0;
        let bx = vec![0.0; n];
        let by = vec![0.0; n];
        let bz: Vec<f64> = (0..n).map(|i| (2.0 * PI * 8.0 * i as f64 / fs).sin()).collect();

        let sep = WaveModeSeparator::new(fs);
        let result = sep.separate(&bx, &by, &bz);
        assert_eq!(sep.sample_rate(), fs);

        // Compressional ratio should be ~1.0 at the signal frequency
        let max_comp_ratio = result.compressional_ratio.iter().cloned()
            .fold(0.0_f64, f64::max);
        assert!(max_comp_ratio > 0.9, "Expected high compressional ratio, got {}", max_comp_ratio);
    }

    #[test]
    fn test_wave_mode_separator_pure_transverse() {
        // Signal only in Bx (transverse)
        let n = 256;
        let fs = 256.0;
        let bx: Vec<f64> = (0..n).map(|i| (2.0 * PI * 8.0 * i as f64 / fs).sin()).collect();
        let by = vec![0.0; n];
        let bz = vec![0.0; n];

        let sep = WaveModeSeparator::new(fs);
        let result = sep.separate(&bx, &by, &bz);

        // Transverse power should dominate: compressional ratio should be ~0 at signal frequency
        let peak_idx = result.transverse_psd.iter().enumerate().skip(1)
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        assert!(result.compressional_ratio[peak_idx] < 0.1);
    }

    // -- Geomagnetic index estimator tests --

    #[test]
    fn test_kp_quiet() {
        let estimator = GeomagneticIndexEstimator::new(50.0);
        // Small perturbation => low Kp
        let delta_h: Vec<f64> = (0..1000).map(|i| 2.0 * (i as f64 * 0.01).sin()).collect();
        let result = estimator.estimate(&delta_h, 20000.0);
        // Range ~ 4 nT => K=0 (threshold at 5 nT for K=1)
        assert!(result.kp < 2.0, "Expected low Kp for quiet data, got {}", result.kp);
    }

    #[test]
    fn test_kp_storm() {
        let estimator = GeomagneticIndexEstimator::new(50.0);
        // Large perturbation => high Kp
        let delta_h: Vec<f64> = (0..1000).map(|i| 250.0 * (i as f64 * 0.01).sin()).collect();
        let result = estimator.estimate(&delta_h, 20000.0);
        // Range ~ 500 nT => K=8 or 9
        assert!(result.kp > 7.0, "Expected high Kp for storm data, got {}", result.kp);
    }

    #[test]
    fn test_dst_negative_during_storm() {
        let estimator = GeomagneticIndexEstimator::new(30.0);
        // Negative mean perturbation => negative Dst (main phase of storm)
        let delta_h = vec![-100.0; 1000];
        let result = estimator.estimate(&delta_h, 20000.0);
        assert!(result.dst_nt < 0.0, "Dst should be negative during storm main phase");
    }

    #[test]
    fn test_geomag_empty_data() {
        let estimator = GeomagneticIndexEstimator::new(50.0);
        let result = estimator.estimate(&[], 20000.0);
        assert!((result.kp - 0.0).abs() < TOLERANCE);
        assert!((result.dst_nt - 0.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_estimator_thresholds() {
        let estimator = GeomagneticIndexEstimator::new(50.0);
        let thresholds = estimator.k_thresholds();
        // Thresholds should be monotonically increasing
        for i in 1..10 {
            assert!(thresholds[i] >= thresholds[i - 1]);
        }
        assert!((estimator.latitude() - 50.0).abs() < TOLERANCE);
    }

    // -- Alfven wave detector test --

    #[test]
    fn test_alfven_detector_no_signal() {
        let det = AlfvenWaveDetector::new(10.0, 100e-9, 10.0, IonSpecies::Proton);
        let n = 128;
        let bx = vec![0.0; n];
        let by = vec![0.0; n];
        let bz = vec![0.0; n];
        let detections = det.detect(&bx, &by, &bz);
        assert!(detections.is_empty(), "Should detect no waves in zero signal");
    }

    #[test]
    fn test_alfven_detector_with_tone() {
        // Inject a transverse tone well below the ion cyclotron frequency
        let fs = 10.0; // 10 Hz sample rate
        let b0 = 100e-9;
        let n = 256;
        let f_ci = cyclotron_frequency(b0, 1.0, PROTON_MASS);
        // Use a frequency well below f_ci (f_ci ~ 1.5 Hz for 100 nT)
        // At fs=10 Hz, we can resolve frequencies up to 5 Hz
        let f_tone = 0.2; // 0.2 Hz, well below f_ci
        let amplitude = 1.0;
        let bx: Vec<f64> = (0..n).map(|i| amplitude * (2.0 * PI * f_tone * i as f64 / fs).cos()).collect();
        let by: Vec<f64> = (0..n).map(|i| amplitude * (2.0 * PI * f_tone * i as f64 / fs).sin()).collect();
        let bz = vec![0.0; n];

        let det = AlfvenWaveDetector::new(fs, b0, 6.0, IonSpecies::Proton);
        assert!((det.ion_cyclotron_frequency() - f_ci).abs() / f_ci < 1e-10);
        let detections = det.detect(&bx, &by, &bz);
        // Should detect at least one wave
        assert!(!detections.is_empty(), "Should detect a wave from the injected tone");
        // The detected frequency should be near f_tone
        let first = &detections[0];
        assert!((first.peak_frequency_hz - f_tone).abs() < 0.5,
            "Detected frequency {} should be near {}", first.peak_frequency_hz, f_tone);
    }

    // -- Median helper test --

    #[test]
    fn test_median_odd() {
        assert!((median_value(&[3.0, 1.0, 2.0]) - 2.0).abs() < TOLERANCE);
    }

    #[test]
    fn test_median_even() {
        assert!((median_value(&[4.0, 1.0, 3.0, 2.0]) - 2.5).abs() < TOLERANCE);
    }

    #[test]
    fn test_median_empty() {
        assert!((median_value(&[]) - 0.0).abs() < TOLERANCE);
    }
}
