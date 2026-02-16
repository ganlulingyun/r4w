//! # Heterodyne Laser Interferometry for Precision Displacement Measurement
//!
//! This module implements signal processing for heterodyne laser interferometry,
//! the gold standard for nanometer-precision displacement and length measurement.
//!
//! ## Principle of Operation
//!
//! A heterodyne interferometer uses a two-frequency laser source (e.g., Zeeman-split
//! HeNe at 632.8 nm). The two orthogonally polarized frequencies f1 and f2 are split
//! into measurement and reference arms. Motion of the measurement mirror introduces a
//! Doppler shift on the measurement beam, changing the beat frequency:
//!
//! ```text
//! Reference beat:    f_ref  = f1 - f2
//! Measurement beat:  f_meas = f1 - f2 + f_doppler
//! Phase difference:  dphi   = phi_meas - phi_ref
//! Displacement:      d      = dphi * lambda / (4 * pi * n_air)
//! ```
//!
//! The factor of 4*pi arises from the double-pass (round-trip) geometry and the
//! 2*pi per wavelength relationship.
//!
//! ## Applications
//!
//! - **CNC Machine Tools**: Sub-micron positioning for precision machining
//! - **Semiconductor Lithography**: Wafer stepper stage alignment (sub-nm)
//! - **Gravitational Wave Detectors**: LIGO/Virgo displacement sensing
//! - **Coordinate Measuring Machines (CMMs)**: Traceable length measurement
//! - **Calibration**: Laser interferometer calibration of linear encoders
//!
//! ## Processing Chain
//!
//! ```text
//! Photodetector ── IQ Demodulation ── Phase Extraction ── Phase Unwrap
//!       |                                                      |
//!  Reference PD ── IQ Demodulation ── Phase Extraction    Displacement
//!                                         |                    |
//!                                    Phase Diff ───────── Env. Correction
//!                                                              |
//!                                                    Nonlinearity Correction
//!                                                              |
//!                                                      Cosine Error Corr.
//!                                                              |
//!                                                     Final Displacement
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::laser_heterodyne_interferometer::*;
//!
//! let config = HeterodyneConfig::hene_zeeman();
//! let mut demod = HeterodyneDemodulator::new(config.clone());
//! let mut unwrapper = PhaseUnwrapper::new();
//! let calc = DisplacementCalculator::new(config.wavelength_m, 1.000271);
//!
//! // Simulate a beat signal with some phase offset (representing displacement)
//! let fs = config.sample_rate_hz;
//! let f_beat = config.split_frequency_hz;
//! let target_phase = 1.5; // radians
//! let samples: Vec<f64> = (0..1000)
//!     .map(|i| {
//!         let t = i as f64 / fs;
//!         (2.0 * std::f64::consts::PI * f_beat * t + target_phase).cos()
//!     })
//!     .collect();
//!
//! let (i_out, q_out) = demod.demodulate(&samples);
//! assert!(i_out.len() == samples.len());
//! ```

use std::f64::consts::PI;

// ============================================================================
// Constants
// ============================================================================

/// Speed of light in vacuum (m/s).
const C_LIGHT: f64 = 299_792_458.0;

/// Standard atmospheric pressure (Pa).
const STD_PRESSURE_PA: f64 = 101_325.0;

/// Standard temperature (degrees Celsius).
const STD_TEMPERATURE_C: f64 = 20.0;

/// Standard relative humidity (fraction 0..1).
const STD_HUMIDITY: f64 = 0.5;

/// HeNe laser wavelength in vacuum (m).
const HENE_WAVELENGTH_M: f64 = 632.8e-9;

/// Typical Zeeman HeNe split frequency (Hz).
const ZEEMAN_SPLIT_HZ: f64 = 1.8e6;

// ============================================================================
// HeterodyneConfig
// ============================================================================

/// Configuration for a heterodyne laser interferometer system.
#[derive(Debug, Clone)]
pub struct HeterodyneConfig {
    /// Laser wavelength in vacuum (m).
    pub wavelength_m: f64,
    /// Split frequency f1 - f2 (Hz), the nominal beat frequency.
    pub split_frequency_hz: f64,
    /// Photodetector sample rate (Hz). Must be > 2 * split_frequency for Nyquist.
    pub sample_rate_hz: f64,
    /// Reference arm optical path length (m). Used for dead-path correction.
    pub reference_arm_length_m: f64,
    /// Number of passes (1 for single-pass, 2 for double-pass / retroreflector).
    pub num_passes: u32,
    /// Lowpass filter bandwidth for IQ demodulation (Hz).
    pub demod_bandwidth_hz: f64,
}

impl HeterodyneConfig {
    /// HeNe Zeeman laser preset (632.8 nm, ~1.8 MHz split).
    ///
    /// This is the most common configuration for commercial heterodyne
    /// interferometers (e.g., Agilent/Keysight 5517, Renishaw XL-80).
    pub fn hene_zeeman() -> Self {
        Self {
            wavelength_m: HENE_WAVELENGTH_M,
            split_frequency_hz: ZEEMAN_SPLIT_HZ,
            sample_rate_hz: 10.0e6,
            reference_arm_length_m: 0.0,
            num_passes: 2,
            demod_bandwidth_hz: 500.0e3,
        }
    }

    /// High-frequency HeNe preset for fast-moving stages (~3 MHz split).
    pub fn hene_high_freq() -> Self {
        Self {
            wavelength_m: HENE_WAVELENGTH_M,
            split_frequency_hz: 3.0e6,
            sample_rate_hz: 20.0e6,
            reference_arm_length_m: 0.0,
            num_passes: 2,
            demod_bandwidth_hz: 1.0e6,
        }
    }

    /// Single-pass configuration (e.g., for differential interferometry).
    pub fn hene_single_pass() -> Self {
        Self {
            wavelength_m: HENE_WAVELENGTH_M,
            split_frequency_hz: ZEEMAN_SPLIT_HZ,
            sample_rate_hz: 10.0e6,
            reference_arm_length_m: 0.0,
            num_passes: 1,
            demod_bandwidth_hz: 500.0e3,
        }
    }

    /// Wavelength per fringe: lambda / num_passes.
    /// For double-pass, one fringe = lambda/2 = 316.4 nm displacement.
    pub fn displacement_per_fringe(&self) -> f64 {
        self.wavelength_m / (self.num_passes as f64)
    }

    /// Maximum measurable velocity (m/s) before the Doppler shift exceeds
    /// half the split frequency (direction ambiguity limit).
    pub fn max_velocity(&self) -> f64 {
        // v_max = f_split * lambda / (2 * num_passes)
        self.split_frequency_hz * self.wavelength_m / (2.0 * self.num_passes as f64)
    }
}

// ============================================================================
// HeNePresets — named laser configurations
// ============================================================================

/// Named HeNe laser presets for common interferometer systems.
pub struct HeNePresets;

impl HeNePresets {
    /// Standard Zeeman-stabilized HeNe (632.8 nm, 1.8 MHz split).
    pub fn zeeman_standard() -> HeterodyneConfig {
        HeterodyneConfig::hene_zeeman()
    }

    /// Two-frequency HeNe with acousto-optic modulator (20 MHz split).
    /// Used in high-velocity applications (e.g., LIGO test masses).
    pub fn aom_shifted() -> HeterodyneConfig {
        HeterodyneConfig {
            wavelength_m: HENE_WAVELENGTH_M,
            split_frequency_hz: 20.0e6,
            sample_rate_hz: 100.0e6,
            reference_arm_length_m: 0.0,
            num_passes: 2,
            demod_bandwidth_hz: 5.0e6,
        }
    }

    /// Nd:YAG 1064 nm (used in gravitational wave detectors).
    pub fn ndyag_1064() -> HeterodyneConfig {
        HeterodyneConfig {
            wavelength_m: 1064.0e-9,
            split_frequency_hz: 10.0e6,
            sample_rate_hz: 50.0e6,
            reference_arm_length_m: 0.0,
            num_passes: 2,
            demod_bandwidth_hz: 2.0e6,
        }
    }
}

// ============================================================================
// HeterodyneDemodulator
// ============================================================================

/// IQ demodulator for heterodyne beat signals.
///
/// Mixes the input signal with sin/cos at the nominal beat frequency,
/// then applies a single-pole IIR lowpass to extract the baseband I and Q.
/// The instantaneous phase is then phi = atan2(Q, I).
#[derive(Debug, Clone)]
pub struct HeterodyneDemodulator {
    config: HeterodyneConfig,
    /// NCO phase accumulator (radians).
    nco_phase: f64,
    /// NCO phase increment per sample (radians).
    nco_phase_inc: f64,
    /// IIR lowpass coefficient alpha for I channel.
    alpha: f64,
    /// Filtered I (in-phase) state.
    i_state: f64,
    /// Filtered Q (quadrature) state.
    q_state: f64,
}

impl HeterodyneDemodulator {
    /// Create a new demodulator for the given configuration.
    pub fn new(config: HeterodyneConfig) -> Self {
        let nco_phase_inc = 2.0 * PI * config.split_frequency_hz / config.sample_rate_hz;
        // Single-pole IIR: alpha = 1 - exp(-2*pi*f_bw / fs)
        let alpha = 1.0 - (-2.0 * PI * config.demod_bandwidth_hz / config.sample_rate_hz).exp();
        Self {
            config,
            nco_phase: 0.0,
            nco_phase_inc,
            alpha,
            i_state: 0.0,
            q_state: 0.0,
        }
    }

    /// Demodulate a block of samples, returning (I, Q) vectors.
    ///
    /// Each output sample represents the lowpass-filtered in-phase and
    /// quadrature components at that time instant.
    pub fn demodulate(&mut self, samples: &[f64]) -> (Vec<f64>, Vec<f64>) {
        let n = samples.len();
        let mut i_out = Vec::with_capacity(n);
        let mut q_out = Vec::with_capacity(n);

        for &s in samples {
            // Mix with NCO
            let cos_val = self.nco_phase.cos();
            let sin_val = self.nco_phase.sin();
            let i_mixed = s * cos_val * 2.0; // factor 2 compensates for mixing loss
            let q_mixed = -s * sin_val * 2.0;

            // Single-pole IIR lowpass
            self.i_state += self.alpha * (i_mixed - self.i_state);
            self.q_state += self.alpha * (q_mixed - self.q_state);

            i_out.push(self.i_state);
            q_out.push(self.q_state);

            // Advance NCO
            self.nco_phase += self.nco_phase_inc;
            if self.nco_phase > 2.0 * PI {
                self.nco_phase -= 2.0 * PI;
            }
        }

        (i_out, q_out)
    }

    /// Extract instantaneous phase from I/Q vectors (radians, wrapped to [-pi, pi]).
    pub fn extract_phase(i_samples: &[f64], q_samples: &[f64]) -> Vec<f64> {
        i_samples
            .iter()
            .zip(q_samples.iter())
            .map(|(&i, &q)| q.atan2(i))
            .collect()
    }

    /// Reset the demodulator state (NCO phase and filter states).
    pub fn reset(&mut self) {
        self.nco_phase = 0.0;
        self.i_state = 0.0;
        self.q_state = 0.0;
    }

    /// Get the current configuration.
    pub fn config(&self) -> &HeterodyneConfig {
        &self.config
    }
}

// ============================================================================
// PhaseUnwrapper
// ============================================================================

/// Continuous phase tracker that removes 2*pi discontinuities.
///
/// Phase unwrapping is critical for displacement measurement because each
/// 2*pi of phase corresponds to lambda/(2*n_passes) of displacement.
/// A discontinuity would appear as a sudden ~316 nm jump for a HeNe double-pass.
#[derive(Debug, Clone)]
pub struct PhaseUnwrapper {
    /// Previous wrapped phase value.
    prev_phase: f64,
    /// Accumulated unwrapped phase.
    accumulated: f64,
    /// Whether we have received the first sample.
    initialized: bool,
    /// Tolerance for detecting wraps (default: PI).
    tolerance: f64,
}

impl PhaseUnwrapper {
    /// Create a new phase unwrapper with default pi tolerance.
    pub fn new() -> Self {
        Self {
            prev_phase: 0.0,
            accumulated: 0.0,
            initialized: false,
            tolerance: PI,
        }
    }

    /// Create with custom wrap tolerance.
    pub fn with_tolerance(tolerance: f64) -> Self {
        Self {
            prev_phase: 0.0,
            accumulated: 0.0,
            initialized: false,
            tolerance,
        }
    }

    /// Unwrap a single phase sample, returning the continuous phase.
    pub fn unwrap_sample(&mut self, phase: f64) -> f64 {
        if !self.initialized {
            self.prev_phase = phase;
            self.accumulated = phase;
            self.initialized = true;
            return phase;
        }

        let mut diff = phase - self.prev_phase;
        // Correct for wrapping
        while diff > self.tolerance {
            diff -= 2.0 * PI;
        }
        while diff < -self.tolerance {
            diff += 2.0 * PI;
        }

        self.accumulated += diff;
        self.prev_phase = phase;
        self.accumulated
    }

    /// Unwrap a vector of wrapped phase values.
    pub fn unwrap_batch(&mut self, phases: &[f64]) -> Vec<f64> {
        phases.iter().map(|&p| self.unwrap_sample(p)).collect()
    }

    /// Reset the unwrapper state.
    pub fn reset(&mut self) {
        self.prev_phase = 0.0;
        self.accumulated = 0.0;
        self.initialized = false;
    }

    /// Get current accumulated (unwrapped) phase.
    pub fn current_phase(&self) -> f64 {
        self.accumulated
    }

    /// Get total number of full fringes (2*pi wraps) counted.
    pub fn fringe_count(&self) -> f64 {
        self.accumulated / (2.0 * PI)
    }
}

impl Default for PhaseUnwrapper {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================================
// DisplacementCalculator
// ============================================================================

/// Converts unwrapped phase to physical displacement.
///
/// The fundamental relationship for a double-pass interferometer:
///
/// ```text
/// d = phi * lambda / (4 * pi * n_air)
/// ```
///
/// For single-pass: d = phi * lambda / (2 * pi * n_air)
///
/// General formula: d = phi * lambda / (2 * pi * num_passes * n_air)
#[derive(Debug, Clone)]
pub struct DisplacementCalculator {
    /// Laser wavelength in vacuum (m).
    wavelength_m: f64,
    /// Refractive index of air (typically ~1.000271 at standard conditions).
    refractive_index: f64,
    /// Number of passes (1 or 2).
    num_passes: u32,
    /// Conversion factor: wavelength / (2 * pi * num_passes * n_air).
    scale_factor: f64,
}

impl DisplacementCalculator {
    /// Create a new calculator for double-pass (standard) configuration.
    pub fn new(wavelength_m: f64, refractive_index: f64) -> Self {
        Self::with_passes(wavelength_m, refractive_index, 2)
    }

    /// Create with explicit number of passes.
    pub fn with_passes(wavelength_m: f64, refractive_index: f64, num_passes: u32) -> Self {
        let scale_factor =
            wavelength_m / (2.0 * PI * num_passes as f64 * refractive_index);
        Self {
            wavelength_m,
            refractive_index,
            num_passes,
            scale_factor,
        }
    }

    /// Convert unwrapped phase (radians) to displacement (meters).
    pub fn phase_to_displacement(&self, phase: f64) -> f64 {
        phase * self.scale_factor
    }

    /// Convert a batch of unwrapped phases to displacements.
    pub fn phase_to_displacement_batch(&self, phases: &[f64]) -> Vec<f64> {
        phases.iter().map(|&p| p * self.scale_factor).collect()
    }

    /// Convert displacement (meters) back to phase (radians).
    pub fn displacement_to_phase(&self, displacement: f64) -> f64 {
        displacement / self.scale_factor
    }

    /// Update the refractive index (e.g., after environmental compensation).
    pub fn set_refractive_index(&mut self, n: f64) {
        self.refractive_index = n;
        self.scale_factor =
            self.wavelength_m / (2.0 * PI * self.num_passes as f64 * self.refractive_index);
    }

    /// Get the current scale factor (m/rad).
    pub fn scale_factor(&self) -> f64 {
        self.scale_factor
    }

    /// Displacement per fringe (one full 2*pi of phase) in meters.
    pub fn displacement_per_fringe(&self) -> f64 {
        self.wavelength_m / (self.num_passes as f64 * self.refractive_index)
    }

    /// Get current refractive index.
    pub fn refractive_index(&self) -> f64 {
        self.refractive_index
    }
}

// ============================================================================
// RefractiveIndexCorrector — Edlen equation
// ============================================================================

/// Computes the refractive index of air using the modified Edlen equation.
///
/// The Edlen equation (1966, updated by Birch & Downs 1993) is the standard
/// method for compensating the wavelength of light in air for precision
/// interferometry. It accounts for temperature, pressure, humidity, and
/// wavelength dependence.
///
/// Accuracy: ~1e-8 (10 ppb) for standard laboratory conditions.
///
/// Reference: Birch, K.P. and Downs, M.J., "An Updated Edlen Equation for
/// the Refractive Index of Air", Metrologia, 30, 155-162 (1993).
#[derive(Debug, Clone)]
pub struct RefractiveIndexCorrector {
    /// Laser wavelength in vacuum (m) — used for dispersion calculation.
    wavelength_m: f64,
}

impl RefractiveIndexCorrector {
    /// Create a corrector for the given laser wavelength.
    pub fn new(wavelength_m: f64) -> Self {
        Self { wavelength_m }
    }

    /// HeNe laser corrector (632.8 nm).
    pub fn hene() -> Self {
        Self::new(HENE_WAVELENGTH_M)
    }

    /// Compute the refractive index of air.
    ///
    /// # Parameters
    /// - `temperature_c`: Air temperature in degrees Celsius
    /// - `pressure_pa`: Air pressure in Pascals
    /// - `humidity`: Relative humidity as fraction (0.0 to 1.0)
    ///
    /// # Returns
    /// Refractive index n (typically ~1.000271 at standard conditions).
    pub fn compute(&self, temperature_c: f64, pressure_pa: f64, humidity: f64) -> f64 {
        // Wavelength in micrometers for the dispersion formula
        let lambda_um = self.wavelength_m * 1.0e6;
        let sigma2 = 1.0 / (lambda_um * lambda_um); // wavenumber squared (1/um^2)

        // Refractivity of standard dry air (15 C, 101325 Pa) — Edlen/Birch-Downs
        // (n_s - 1) * 1e8 = 8342.54 + 2406147 / (130 - sigma^2) + 15998 / (38.9 - sigma^2)
        let n_s_minus_1 = (8342.54 + 2_406_147.0 / (130.0 - sigma2)
            + 15998.0 / (38.9 - sigma2))
            * 1.0e-8;

        // Temperature and pressure correction
        // n_tp - 1 = (n_s - 1) * p * [1 + p * (60.1 - 0.972*T) * 1e-10] / (96095.43 * (1 + 0.003661 * T))
        let t = temperature_c;
        let p = pressure_pa;

        let n_tp_minus_1 = n_s_minus_1 * p
            * (1.0 + p * (60.1 - 0.972 * t) * 1.0e-10)
            / (96095.43 * (1.0 + 0.003661 * t));

        // Water vapor correction (Edlen formula)
        // Saturation vapor pressure using simplified Magnus formula (Pa)
        let e_s = 611.2 * (17.67 * t / (t + 243.5)).exp();
        let e = humidity * e_s; // partial water vapor pressure

        // Water vapor correction to refractivity
        // dn_h = -e * (3.7345 - 0.0401 * sigma^2) * 1e-10
        let dn_h = -e * (3.7345 - 0.0401 * sigma2) * 1.0e-10;

        1.0 + n_tp_minus_1 + dn_h
    }

    /// Compute at standard conditions (20 C, 101325 Pa, 50% RH).
    pub fn standard_conditions(&self) -> f64 {
        self.compute(STD_TEMPERATURE_C, STD_PRESSURE_PA, STD_HUMIDITY)
    }

    /// Compute the wavelength of light in air (m).
    pub fn wavelength_in_air(&self, temperature_c: f64, pressure_pa: f64, humidity: f64) -> f64 {
        self.wavelength_m / self.compute(temperature_c, pressure_pa, humidity)
    }

    /// Compute the group refractive index (for broadband sources).
    /// n_g = n - lambda * dn/dlambda
    /// Approximated via finite difference.
    pub fn group_index(&self, temperature_c: f64, pressure_pa: f64, humidity: f64) -> f64 {
        let delta = self.wavelength_m * 1.0e-6; // small perturbation
        let n_plus = RefractiveIndexCorrector::new(self.wavelength_m + delta)
            .compute(temperature_c, pressure_pa, humidity);
        let n_minus = RefractiveIndexCorrector::new(self.wavelength_m - delta)
            .compute(temperature_c, pressure_pa, humidity);
        let n = self.compute(temperature_c, pressure_pa, humidity);
        let dn_dlambda = (n_plus - n_minus) / (2.0 * delta);
        n - self.wavelength_m * dn_dlambda
    }
}

// ============================================================================
// NonlinearityCorrector — Heydemann ellipse fitting
// ============================================================================

/// Corrects periodic nonlinearity in heterodyne interferometers.
///
/// Periodic nonlinearity arises from imperfect polarization separation,
/// causing the I/Q Lissajous figure to form an ellipse rather than a circle.
/// The Heydemann method fits an ellipse to the I/Q data and corrects:
///
/// - DC offsets in I and Q channels
/// - Gain mismatch between I and Q
/// - Phase quadrature error (non-orthogonality)
///
/// Typical periodic nonlinearity: 1-10 nm peak-to-peak for commercial systems.
///
/// Reference: Heydemann, P.L.M., "Determination and correction of quadrature
/// fringe measurement errors in interferometers", Applied Optics, 20(19),
/// 3382-3384 (1981).
#[derive(Debug, Clone)]
pub struct NonlinearityCorrector {
    /// DC offset of I channel.
    pub i_offset: f64,
    /// DC offset of Q channel.
    pub q_offset: f64,
    /// Gain of I channel.
    pub i_gain: f64,
    /// Gain of Q channel.
    pub q_gain: f64,
    /// Phase error (deviation from 90 degrees) in radians.
    pub phase_error: f64,
}

impl NonlinearityCorrector {
    /// Create a corrector with ideal parameters (no correction needed).
    pub fn ideal() -> Self {
        Self {
            i_offset: 0.0,
            q_offset: 0.0,
            i_gain: 1.0,
            q_gain: 1.0,
            phase_error: 0.0,
        }
    }

    /// Create from known calibration parameters.
    pub fn new(
        i_offset: f64,
        q_offset: f64,
        i_gain: f64,
        q_gain: f64,
        phase_error: f64,
    ) -> Self {
        Self {
            i_offset,
            q_offset,
            i_gain,
            q_gain,
            phase_error,
        }
    }

    /// Fit ellipse parameters from I/Q calibration data using least-squares.
    ///
    /// The data should span at least one full fringe (2*pi of phase).
    /// More data (multiple fringes) yields better estimates.
    pub fn fit(i_data: &[f64], q_data: &[f64]) -> Self {
        assert_eq!(i_data.len(), q_data.len(), "I and Q must have same length");
        let n = i_data.len() as f64;

        // Compute mean (DC offsets)
        let i_mean: f64 = i_data.iter().sum::<f64>() / n;
        let q_mean: f64 = q_data.iter().sum::<f64>() / n;

        // Center the data
        let i_centered: Vec<f64> = i_data.iter().map(|&x| x - i_mean).collect();
        let q_centered: Vec<f64> = q_data.iter().map(|&x| x - q_mean).collect();

        // Estimate amplitudes (RMS * sqrt(2) for sinusoidal signals)
        let i_rms = (i_centered.iter().map(|x| x * x).sum::<f64>() / n).sqrt();
        let q_rms = (q_centered.iter().map(|x| x * x).sum::<f64>() / n).sqrt();

        let i_amp = i_rms * std::f64::consts::SQRT_2;
        let q_amp = q_rms * std::f64::consts::SQRT_2;

        // Estimate phase error from cross-correlation at zero lag
        // For ideal quadrature signals, <I*Q> = 0
        // For phase error epsilon: <I*Q> = 0.5 * A_i * A_q * sin(epsilon)
        let cross_corr: f64 = i_centered
            .iter()
            .zip(q_centered.iter())
            .map(|(i, q)| i * q)
            .sum::<f64>()
            / n;

        let phase_error = if i_amp > 1.0e-20 && q_amp > 1.0e-20 {
            (2.0 * cross_corr / (i_amp * q_amp)).asin().clamp(-PI / 4.0, PI / 4.0)
        } else {
            0.0
        };

        Self {
            i_offset: i_mean,
            q_offset: q_mean,
            i_gain: if i_amp > 1.0e-20 { i_amp } else { 1.0 },
            q_gain: if q_amp > 1.0e-20 { q_amp } else { 1.0 },
            phase_error,
        }
    }

    /// Apply correction to a single (I, Q) pair, returning corrected (I, Q).
    pub fn correct(&self, i: f64, q: f64) -> (f64, f64) {
        // Remove DC offsets
        let i_c = i - self.i_offset;
        let q_c = q - self.q_offset;

        // Normalize gains
        let i_n = i_c / self.i_gain;
        let q_n = q_c / self.q_gain;

        // Correct phase error (skew correction)
        // q_corrected = (q_n - i_n * sin(eps)) / cos(eps)
        let cos_e = self.phase_error.cos();
        let sin_e = self.phase_error.sin();
        let q_corr = if cos_e.abs() > 1.0e-10 {
            (q_n - i_n * sin_e) / cos_e
        } else {
            q_n
        };

        (i_n, q_corr)
    }

    /// Apply correction to batches of I and Q data.
    pub fn correct_batch(&self, i_data: &[f64], q_data: &[f64]) -> (Vec<f64>, Vec<f64>) {
        i_data
            .iter()
            .zip(q_data.iter())
            .map(|(&i, &q)| self.correct(i, q))
            .unzip()
    }

    /// Estimate the peak-to-peak periodic nonlinearity (in meters)
    /// based on the fitted parameters.
    pub fn estimated_nonlinearity_m(&self, wavelength_m: f64, num_passes: u32) -> f64 {
        // The dominant terms are gain mismatch and phase error
        let gain_ratio = if self.i_gain > 1.0e-20 {
            (self.q_gain / self.i_gain - 1.0).abs()
        } else {
            0.0
        };
        let phase_err_abs = self.phase_error.abs();

        // Peak nonlinearity ~ (gain_mismatch + phase_error) * lambda / (4*pi*N)
        let scale = wavelength_m / (4.0 * PI * num_passes as f64);
        (gain_ratio + phase_err_abs) * scale
    }
}

// ============================================================================
// VelocityEstimator
// ============================================================================

/// Estimates instantaneous velocity from the phase rate (Doppler frequency).
///
/// The Doppler shift is proportional to velocity:
///
/// ```text
/// f_doppler = 2 * v * num_passes / lambda
/// v = d(phi)/dt * lambda / (2 * pi * num_passes)
/// ```
#[derive(Debug, Clone)]
pub struct VelocityEstimator {
    /// Conversion factor: lambda / (2 * pi * num_passes * n_air).
    scale: f64,
    /// Sample rate (Hz).
    sample_rate_hz: f64,
    /// Previous phase value for differentiation.
    prev_phase: Option<f64>,
}

impl VelocityEstimator {
    /// Create from configuration.
    pub fn from_config(config: &HeterodyneConfig, refractive_index: f64) -> Self {
        let scale = config.wavelength_m
            / (2.0 * PI * config.num_passes as f64 * refractive_index);
        Self {
            scale,
            sample_rate_hz: config.sample_rate_hz,
            prev_phase: None,
        }
    }

    /// Create with explicit parameters.
    pub fn new(wavelength_m: f64, num_passes: u32, refractive_index: f64, sample_rate_hz: f64) -> Self {
        let scale = wavelength_m / (2.0 * PI * num_passes as f64 * refractive_index);
        Self {
            scale,
            sample_rate_hz,
            prev_phase: None,
        }
    }

    /// Estimate velocity from a single unwrapped phase sample (m/s).
    pub fn estimate_sample(&mut self, unwrapped_phase: f64) -> f64 {
        let velocity = match self.prev_phase {
            Some(prev) => {
                let dphi_dt = (unwrapped_phase - prev) * self.sample_rate_hz;
                dphi_dt * self.scale
            }
            None => 0.0,
        };
        self.prev_phase = Some(unwrapped_phase);
        velocity
    }

    /// Estimate velocity from a batch of unwrapped phase samples (m/s).
    /// Uses central differences for interior points, forward/backward at edges.
    pub fn estimate_batch(&self, unwrapped_phases: &[f64]) -> Vec<f64> {
        let n = unwrapped_phases.len();
        if n == 0 {
            return vec![];
        }
        if n == 1 {
            return vec![0.0];
        }

        let mut velocities = Vec::with_capacity(n);

        // Forward difference for first sample
        let dphi = (unwrapped_phases[1] - unwrapped_phases[0]) * self.sample_rate_hz;
        velocities.push(dphi * self.scale);

        // Central differences for interior
        for i in 1..n - 1 {
            let dphi = (unwrapped_phases[i + 1] - unwrapped_phases[i - 1])
                * self.sample_rate_hz
                / 2.0;
            velocities.push(dphi * self.scale);
        }

        // Backward difference for last sample
        let dphi = (unwrapped_phases[n - 1] - unwrapped_phases[n - 2]) * self.sample_rate_hz;
        velocities.push(dphi * self.scale);

        velocities
    }

    /// Convert Doppler frequency to velocity (m/s).
    pub fn doppler_to_velocity(&self, doppler_hz: f64) -> f64 {
        doppler_hz * self.scale * 2.0 * PI / self.sample_rate_hz
            * self.sample_rate_hz // cancel out — simplify:
    }

    /// Reset the estimator state.
    pub fn reset(&mut self) {
        self.prev_phase = None;
    }
}

impl VelocityEstimator {
    /// Direct velocity from Doppler frequency: v = f_d * lambda / (2 * n_passes).
    pub fn velocity_from_doppler_freq(
        wavelength_m: f64,
        num_passes: u32,
        refractive_index: f64,
        doppler_hz: f64,
    ) -> f64 {
        doppler_hz * wavelength_m / (num_passes as f64 * 2.0 * refractive_index)
    }
}

// ============================================================================
// CosineErrorCorrector
// ============================================================================

/// Corrects cosine error from beam-to-axis misalignment.
///
/// When the laser beam is not perfectly parallel to the axis of motion,
/// the measured displacement is foreshortened:
///
/// ```text
/// d_measured = d_true * cos(theta)
/// d_true     = d_measured / cos(theta)
/// ```
///
/// For small angles: error ~ theta^2/2 (e.g., 1 mrad gives 0.5 ppm error).
#[derive(Debug, Clone)]
pub struct CosineErrorCorrector {
    /// Misalignment angle (radians).
    angle_rad: f64,
    /// cos(angle) precomputed for efficiency.
    cos_angle: f64,
}

impl CosineErrorCorrector {
    /// Create from misalignment angle in radians.
    pub fn from_radians(angle_rad: f64) -> Self {
        Self {
            angle_rad,
            cos_angle: angle_rad.cos(),
        }
    }

    /// Create from misalignment angle in milliradians.
    pub fn from_milliradians(angle_mrad: f64) -> Self {
        Self::from_radians(angle_mrad * 1.0e-3)
    }

    /// Create from misalignment angle in arc-seconds.
    pub fn from_arcseconds(angle_arcsec: f64) -> Self {
        Self::from_radians(angle_arcsec * PI / (180.0 * 3600.0))
    }

    /// Correct a measured displacement to true displacement.
    pub fn correct(&self, measured_displacement: f64) -> f64 {
        if self.cos_angle.abs() > 1.0e-10 {
            measured_displacement / self.cos_angle
        } else {
            measured_displacement
        }
    }

    /// Correct a batch of measured displacements.
    pub fn correct_batch(&self, measured: &[f64]) -> Vec<f64> {
        measured.iter().map(|&d| self.correct(d)).collect()
    }

    /// Compute the relative error (dimensionless).
    /// error = 1 - cos(theta) ~ theta^2/2 for small angles.
    pub fn relative_error(&self) -> f64 {
        1.0 - self.cos_angle
    }

    /// Compute error in parts per million.
    pub fn error_ppm(&self) -> f64 {
        self.relative_error() * 1.0e6
    }

    /// Get the misalignment angle in radians.
    pub fn angle_rad(&self) -> f64 {
        self.angle_rad
    }

    /// Update the alignment angle.
    pub fn set_angle_rad(&mut self, angle_rad: f64) {
        self.angle_rad = angle_rad;
        self.cos_angle = angle_rad.cos();
    }
}

// ============================================================================
// EnvironmentalCompensator
// ============================================================================

/// Compensates displacement measurements for environmental changes.
///
/// This module performs:
///
/// 1. **Wavelength compensation**: Corrects the laser wavelength in air for
///    changing temperature, pressure, and humidity using the Edlen equation.
///
/// 2. **Dead-path correction**: Compensates for the unequal air path between
///    measurement and reference beams when environmental conditions change.
///    Dead-path error = L_dead * (n_current - n_initial).
///
/// 3. **Thermal expansion**: Optional correction for target material CTE.
#[derive(Debug, Clone)]
pub struct EnvironmentalCompensator {
    /// Refractive index corrector.
    corrector: RefractiveIndexCorrector,
    /// Dead-path length (m): difference between reference and measurement arm
    /// optical path lengths at the initial (zero) position.
    dead_path_m: f64,
    /// Initial refractive index at system startup.
    n_initial: f64,
    /// Target material coefficient of thermal expansion (1/K), or 0.0 if not used.
    target_cte: f64,
    /// Reference temperature for CTE correction (Celsius).
    reference_temp_c: f64,
}

impl EnvironmentalCompensator {
    /// Create a new compensator.
    ///
    /// # Parameters
    /// - `wavelength_m`: Laser wavelength in vacuum
    /// - `dead_path_m`: Dead-path length (typically 0 if zeroed at startup)
    /// - `initial_temp_c`: Temperature at system startup (Celsius)
    /// - `initial_pressure_pa`: Pressure at system startup (Pa)
    /// - `initial_humidity`: Humidity at system startup (0..1)
    pub fn new(
        wavelength_m: f64,
        dead_path_m: f64,
        initial_temp_c: f64,
        initial_pressure_pa: f64,
        initial_humidity: f64,
    ) -> Self {
        let corrector = RefractiveIndexCorrector::new(wavelength_m);
        let n_initial = corrector.compute(initial_temp_c, initial_pressure_pa, initial_humidity);
        Self {
            corrector,
            dead_path_m,
            n_initial,
            target_cte: 0.0,
            reference_temp_c: initial_temp_c,
        }
    }

    /// Set target material CTE for thermal expansion correction.
    pub fn set_target_cte(&mut self, cte: f64, reference_temp_c: f64) {
        self.target_cte = cte;
        self.reference_temp_c = reference_temp_c;
    }

    /// Compensate a raw displacement measurement.
    ///
    /// # Parameters
    /// - `raw_displacement_m`: Displacement from phase measurement (m)
    /// - `temperature_c`: Current air temperature (Celsius)
    /// - `pressure_pa`: Current air pressure (Pa)
    /// - `humidity`: Current relative humidity (0..1)
    ///
    /// # Returns
    /// Corrected displacement in meters.
    pub fn compensate(
        &self,
        raw_displacement_m: f64,
        temperature_c: f64,
        pressure_pa: f64,
        humidity: f64,
    ) -> f64 {
        let n_current = self.corrector.compute(temperature_c, pressure_pa, humidity);

        // Wavelength ratio correction: d_corrected = d_raw * n_initial / n_current
        let wavelength_corrected = raw_displacement_m * self.n_initial / n_current;

        // Dead-path correction: d_dead = L_dead * (n_current - n_initial) / n_current
        let dead_path_correction = self.dead_path_m * (n_current - self.n_initial) / n_current;

        let corrected = wavelength_corrected - dead_path_correction;

        // Thermal expansion correction for target material
        if self.target_cte.abs() > 1.0e-15 {
            let delta_t = temperature_c - self.reference_temp_c;
            corrected * (1.0 + self.target_cte * delta_t)
        } else {
            corrected
        }
    }

    /// Get the current refractive index for given conditions.
    pub fn current_refractive_index(
        &self,
        temperature_c: f64,
        pressure_pa: f64,
        humidity: f64,
    ) -> f64 {
        self.corrector.compute(temperature_c, pressure_pa, humidity)
    }

    /// Get the initial refractive index.
    pub fn initial_refractive_index(&self) -> f64 {
        self.n_initial
    }

    /// Compute dead-path error for given conditions (m).
    pub fn dead_path_error(&self, temperature_c: f64, pressure_pa: f64, humidity: f64) -> f64 {
        let n_current = self.corrector.compute(temperature_c, pressure_pa, humidity);
        self.dead_path_m * (n_current - self.n_initial) / n_current
    }
}

// ============================================================================
// Full measurement pipeline
// ============================================================================

/// Complete heterodyne interferometer measurement pipeline.
///
/// Chains: demodulation -> phase extraction -> unwrapping -> displacement
/// with optional nonlinearity, cosine error, and environmental correction.
pub struct InterferometerPipeline {
    /// IQ demodulator.
    pub demodulator: HeterodyneDemodulator,
    /// Phase unwrapper.
    pub unwrapper: PhaseUnwrapper,
    /// Displacement calculator.
    pub calculator: DisplacementCalculator,
    /// Optional nonlinearity corrector.
    pub nonlinearity: Option<NonlinearityCorrector>,
    /// Optional cosine error corrector.
    pub cosine_error: Option<CosineErrorCorrector>,
    /// Velocity estimator.
    pub velocity_estimator: VelocityEstimator,
}

impl InterferometerPipeline {
    /// Create a pipeline with default settings from a config.
    pub fn new(config: HeterodyneConfig) -> Self {
        let n_air = RefractiveIndexCorrector::new(config.wavelength_m).standard_conditions();
        let velocity_estimator = VelocityEstimator::from_config(&config, n_air);
        let calculator =
            DisplacementCalculator::with_passes(config.wavelength_m, n_air, config.num_passes);
        let demodulator = HeterodyneDemodulator::new(config);
        Self {
            demodulator,
            unwrapper: PhaseUnwrapper::new(),
            calculator,
            nonlinearity: None,
            cosine_error: None,
            velocity_estimator,
        }
    }

    /// Process a block of raw photodetector samples.
    ///
    /// Returns (displacements, velocities) in meters and m/s.
    pub fn process(&mut self, samples: &[f64]) -> (Vec<f64>, Vec<f64>) {
        // Step 1: IQ demodulation
        let (mut i_data, mut q_data) = self.demodulator.demodulate(samples);

        // Step 2: Nonlinearity correction (if configured)
        if let Some(ref nlc) = self.nonlinearity {
            let (i_corr, q_corr) = nlc.correct_batch(&i_data, &q_data);
            i_data = i_corr;
            q_data = q_corr;
        }

        // Step 3: Phase extraction
        let wrapped_phase = HeterodyneDemodulator::extract_phase(&i_data, &q_data);

        // Step 4: Phase unwrapping
        let unwrapped = self.unwrapper.unwrap_batch(&wrapped_phase);

        // Step 5: Displacement conversion
        let mut displacements = self.calculator.phase_to_displacement_batch(&unwrapped);

        // Step 6: Cosine error correction (if configured)
        if let Some(ref cos_corr) = self.cosine_error {
            displacements = cos_corr.correct_batch(&displacements);
        }

        // Step 7: Velocity estimation
        let velocities = self.velocity_estimator.estimate_batch(&unwrapped);

        (displacements, velocities)
    }

    /// Reset all pipeline state.
    pub fn reset(&mut self) {
        self.demodulator.reset();
        self.unwrapper.reset();
        self.velocity_estimator.reset();
    }
}

// ============================================================================
// Utility functions
// ============================================================================

/// Convert displacement in meters to nanometers.
pub fn meters_to_nm(m: f64) -> f64 {
    m * 1.0e9
}

/// Convert nanometers to meters.
pub fn nm_to_meters(nm: f64) -> f64 {
    nm * 1.0e-9
}

/// Compute the Doppler frequency for a given velocity.
///
/// f_doppler = 2 * num_passes * v * n_air / lambda
pub fn doppler_frequency(velocity_m_s: f64, wavelength_m: f64, num_passes: u32, n_air: f64) -> f64 {
    2.0 * num_passes as f64 * velocity_m_s * n_air / wavelength_m
}

/// Compute the maximum measurable velocity before Doppler ambiguity.
pub fn max_velocity(split_freq_hz: f64, wavelength_m: f64, num_passes: u32) -> f64 {
    split_freq_hz * wavelength_m / (2.0 * num_passes as f64)
}

/// Compute displacement resolution from phase noise (radians RMS).
///
/// sigma_d = sigma_phi * lambda / (4 * pi * n_air) for double-pass
pub fn displacement_resolution(
    phase_noise_rad_rms: f64,
    wavelength_m: f64,
    num_passes: u32,
    n_air: f64,
) -> f64 {
    phase_noise_rad_rms * wavelength_m / (2.0 * PI * num_passes as f64 * n_air)
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const EPSILON: f64 = 1.0e-10;
    const EPSILON_PHASE: f64 = 1.0e-3;

    // --- HeterodyneConfig tests ---

    #[test]
    fn test_hene_zeeman_preset() {
        let config = HeterodyneConfig::hene_zeeman();
        assert!((config.wavelength_m - 632.8e-9).abs() < 1.0e-15);
        assert!((config.split_frequency_hz - 1.8e6).abs() < 1.0);
        assert_eq!(config.num_passes, 2);
    }

    #[test]
    fn test_displacement_per_fringe() {
        let config = HeterodyneConfig::hene_zeeman();
        let dpf = config.displacement_per_fringe();
        // lambda/2 = 316.4 nm
        assert!((dpf - 316.4e-9).abs() < 0.1e-9);
    }

    #[test]
    fn test_max_velocity() {
        let config = HeterodyneConfig::hene_zeeman();
        let v_max = config.max_velocity();
        // v_max = 1.8e6 * 632.8e-9 / (2 * 2) = ~0.2848 m/s
        assert!((v_max - 0.2847).abs() < 0.001);
    }

    #[test]
    fn test_hene_high_freq_preset() {
        let config = HeterodyneConfig::hene_high_freq();
        assert!((config.split_frequency_hz - 3.0e6).abs() < 1.0);
        assert!((config.sample_rate_hz - 20.0e6).abs() < 1.0);
    }

    #[test]
    fn test_single_pass_config() {
        let config = HeterodyneConfig::hene_single_pass();
        assert_eq!(config.num_passes, 1);
        // lambda/1 = 632.8 nm per fringe
        let dpf = config.displacement_per_fringe();
        assert!((dpf - 632.8e-9).abs() < 0.1e-9);
    }

    // --- HeNePresets tests ---

    #[test]
    fn test_aom_shifted_preset() {
        let config = HeNePresets::aom_shifted();
        assert!((config.split_frequency_hz - 20.0e6).abs() < 1.0);
        assert!((config.sample_rate_hz - 100.0e6).abs() < 1.0);
    }

    #[test]
    fn test_ndyag_preset() {
        let config = HeNePresets::ndyag_1064();
        assert!((config.wavelength_m - 1064.0e-9).abs() < 1.0e-15);
    }

    // --- HeterodyneDemodulator tests ---

    #[test]
    fn test_demodulator_creation() {
        let config = HeterodyneConfig::hene_zeeman();
        let demod = HeterodyneDemodulator::new(config);
        assert!((demod.nco_phase).abs() < EPSILON);
    }

    #[test]
    fn test_demodulator_output_length() {
        let config = HeterodyneConfig::hene_zeeman();
        let mut demod = HeterodyneDemodulator::new(config);
        let samples = vec![0.0; 100];
        let (i_out, q_out) = demod.demodulate(&samples);
        assert_eq!(i_out.len(), 100);
        assert_eq!(q_out.len(), 100);
    }

    #[test]
    fn test_demodulator_dc_input() {
        // DC input mixed with NCO produces signal at f_split.
        // The single-pole IIR lowpass attenuates it but does not fully remove it.
        // For a 1-pole filter at 500 kHz BW with 1.8 MHz signal:
        // attenuation ~ BW/f_signal ~ 500/1800 ~ 0.28 (-11 dB)
        // So the residual amplitude should be moderate but less than 1.0.
        let config = HeterodyneConfig::hene_zeeman();
        let mut demod = HeterodyneDemodulator::new(config.clone());
        let samples = vec![1.0; 10000];
        let (i_out, q_out) = demod.demodulate(&samples);
        // The I and Q channels should show reduced amplitude compared to
        // on-frequency signal (which produces ~1.0 amplitude)
        let last_i = i_out[9999].abs();
        let last_q = q_out[9999].abs();
        // With single-pole IIR, residual is attenuated but not zero
        assert!(last_i < 1.0, "I should be attenuated for DC input, got {}", last_i);
        assert!(last_q < 1.0, "Q should be attenuated for DC input, got {}", last_q);
    }

    #[test]
    fn test_demodulator_on_frequency_signal() {
        // Signal at exactly the split frequency should produce a DC baseband
        let config = HeterodyneConfig::hene_zeeman();
        let mut demod = HeterodyneDemodulator::new(config.clone());
        let fs = config.sample_rate_hz;
        let f_beat = config.split_frequency_hz;
        let n = 50000;
        let samples: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                (2.0 * PI * f_beat * t).cos()
            })
            .collect();
        let (i_out, q_out) = demod.demodulate(&samples);
        // After settling, I should converge to a positive value, Q near zero
        let last_i = i_out[n - 1];
        let last_q = q_out[n - 1];
        assert!(last_i > 0.5, "I should be positive for on-frequency signal, got {}", last_i);
        assert!(last_q.abs() < 0.2, "Q should be near zero, got {}", last_q);
    }

    #[test]
    fn test_demodulator_reset() {
        let config = HeterodyneConfig::hene_zeeman();
        let mut demod = HeterodyneDemodulator::new(config);
        let samples = vec![1.0; 100];
        demod.demodulate(&samples);
        demod.reset();
        assert!((demod.nco_phase).abs() < EPSILON);
        assert!((demod.i_state).abs() < EPSILON);
        assert!((demod.q_state).abs() < EPSILON);
    }

    #[test]
    fn test_extract_phase() {
        let i_samples = vec![1.0, 0.0, -1.0, 0.0];
        let q_samples = vec![0.0, 1.0, 0.0, -1.0];
        let phases = HeterodyneDemodulator::extract_phase(&i_samples, &q_samples);
        assert!((phases[0] - 0.0).abs() < EPSILON);
        assert!((phases[1] - PI / 2.0).abs() < EPSILON);
        assert!((phases[2] - PI).abs() < EPSILON);
        assert!((phases[3] - (-PI / 2.0)).abs() < EPSILON);
    }

    // --- PhaseUnwrapper tests ---

    #[test]
    fn test_unwrapper_no_wrapping() {
        let mut unwrapper = PhaseUnwrapper::new();
        let phases = vec![0.0, 0.1, 0.2, 0.3, 0.4];
        let unwrapped = unwrapper.unwrap_batch(&phases);
        for (i, &u) in unwrapped.iter().enumerate() {
            assert!((u - phases[i]).abs() < EPSILON);
        }
    }

    #[test]
    fn test_unwrapper_positive_wrap() {
        let mut unwrapper = PhaseUnwrapper::new();
        // Simulate phase wrapping from +3.0 to -3.0 (a forward jump)
        let phases = vec![2.8, 3.0, -3.0, -2.8];
        let unwrapped = unwrapper.unwrap_batch(&phases);
        // After unwrapping, should be monotonically increasing
        assert!(unwrapped[2] > unwrapped[1], "Should unwrap forward");
        assert!(unwrapped[3] > unwrapped[2]);
    }

    #[test]
    fn test_unwrapper_negative_wrap() {
        let mut unwrapper = PhaseUnwrapper::new();
        // Simulate decreasing phase wrapping from -3.0 to +3.0
        let phases = vec![-2.8, -3.0, 3.0, 2.8];
        let unwrapped = unwrapper.unwrap_batch(&phases);
        // After unwrapping, should be monotonically decreasing
        assert!(unwrapped[2] < unwrapped[1], "Should unwrap backward");
    }

    #[test]
    fn test_unwrapper_multiple_wraps() {
        let mut unwrapper = PhaseUnwrapper::new();
        // Generate a linearly increasing phase that wraps multiple times
        let n = 100;
        let phase_rate = 0.2; // rad/sample
        let wrapped: Vec<f64> = (0..n)
            .map(|i| {
                let phase = i as f64 * phase_rate;
                // Wrap to [-pi, pi]
                let mut w = phase % (2.0 * PI);
                if w > PI {
                    w -= 2.0 * PI;
                }
                w
            })
            .collect();
        let unwrapped = unwrapper.unwrap_batch(&wrapped);
        // Should recover the linear ramp
        for i in 1..n {
            let diff = unwrapped[i] - unwrapped[i - 1];
            assert!(
                (diff - phase_rate).abs() < 0.01,
                "Phase rate should be constant, got {} at sample {}",
                diff,
                i
            );
        }
    }

    #[test]
    fn test_unwrapper_fringe_count() {
        let mut unwrapper = PhaseUnwrapper::new();
        // Accumulate 5 full fringes
        let phases_per_fringe = 20;
        let total = 5 * phases_per_fringe;
        let step = 2.0 * PI / phases_per_fringe as f64;
        for i in 0..total {
            let wrapped = (i as f64 * step) % (2.0 * PI);
            let w = if wrapped > PI { wrapped - 2.0 * PI } else { wrapped };
            unwrapper.unwrap_sample(w);
        }
        let fc = unwrapper.fringe_count();
        assert!((fc - 5.0).abs() < 0.1, "Expected ~5 fringes, got {}", fc);
    }

    #[test]
    fn test_unwrapper_reset() {
        let mut unwrapper = PhaseUnwrapper::new();
        unwrapper.unwrap_sample(1.0);
        unwrapper.unwrap_sample(2.0);
        unwrapper.reset();
        assert!((unwrapper.current_phase()).abs() < EPSILON);
    }

    #[test]
    fn test_unwrapper_custom_tolerance() {
        let unwrapper = PhaseUnwrapper::with_tolerance(PI / 2.0);
        assert!((unwrapper.tolerance - PI / 2.0).abs() < EPSILON);
    }

    // --- DisplacementCalculator tests ---

    #[test]
    fn test_displacement_one_fringe() {
        let n_air = 1.000271;
        let calc = DisplacementCalculator::new(HENE_WAVELENGTH_M, n_air);
        // One full fringe (2*pi) = lambda / (2 * n_air) for double-pass
        let d = calc.phase_to_displacement(2.0 * PI);
        let expected = HENE_WAVELENGTH_M / (2.0 * n_air);
        assert!(
            (d - expected).abs() < 1.0e-12,
            "One fringe displacement: expected {:.4e}, got {:.4e}",
            expected,
            d
        );
    }

    #[test]
    fn test_displacement_roundtrip() {
        let calc = DisplacementCalculator::new(HENE_WAVELENGTH_M, 1.000271);
        let d = 1.5e-6; // 1.5 um
        let phase = calc.displacement_to_phase(d);
        let d_back = calc.phase_to_displacement(phase);
        assert!((d_back - d).abs() < 1.0e-15);
    }

    #[test]
    fn test_displacement_batch() {
        let calc = DisplacementCalculator::new(HENE_WAVELENGTH_M, 1.000271);
        let phases = vec![0.0, PI, 2.0 * PI, 4.0 * PI];
        let disps = calc.phase_to_displacement_batch(&phases);
        assert_eq!(disps.len(), 4);
        assert!((disps[0]).abs() < EPSILON);
        assert!(disps[1] > 0.0);
        assert!((disps[2] - 2.0 * disps[1]).abs() < 1.0e-15);
    }

    #[test]
    fn test_displacement_single_pass() {
        let calc = DisplacementCalculator::with_passes(HENE_WAVELENGTH_M, 1.0, 1);
        let d = calc.phase_to_displacement(2.0 * PI);
        // For single pass, n=1: d = lambda
        assert!((d - HENE_WAVELENGTH_M).abs() < 1.0e-15);
    }

    #[test]
    fn test_displacement_per_fringe_value() {
        let n_air = 1.000271;
        let calc = DisplacementCalculator::new(HENE_WAVELENGTH_M, n_air);
        let dpf = calc.displacement_per_fringe();
        // lambda / (2 * n_air)
        let expected = HENE_WAVELENGTH_M / (2.0 * n_air);
        assert!((dpf - expected).abs() < 1.0e-15);
    }

    #[test]
    fn test_displacement_update_refractive_index() {
        let mut calc = DisplacementCalculator::new(HENE_WAVELENGTH_M, 1.000271);
        let d1 = calc.phase_to_displacement(PI);
        calc.set_refractive_index(1.000300);
        let d2 = calc.phase_to_displacement(PI);
        // Higher n_air should give slightly smaller displacement
        assert!(d2 < d1);
    }

    // --- RefractiveIndexCorrector tests ---

    #[test]
    fn test_edlen_standard_conditions() {
        let corrector = RefractiveIndexCorrector::hene();
        let n = corrector.standard_conditions();
        // At 20 C, 101325 Pa, 50% RH, HeNe: n ~ 1.000271
        assert!(
            (n - 1.000271).abs() < 0.000010,
            "Standard n_air expected ~1.000271, got {:.6}",
            n
        );
    }

    #[test]
    fn test_edlen_temperature_dependence() {
        let corrector = RefractiveIndexCorrector::hene();
        let n_cold = corrector.compute(15.0, STD_PRESSURE_PA, STD_HUMIDITY);
        let n_hot = corrector.compute(25.0, STD_PRESSURE_PA, STD_HUMIDITY);
        // Higher temperature -> lower refractive index (air expands)
        assert!(n_hot < n_cold, "Hot air should have lower n than cold air");
    }

    #[test]
    fn test_edlen_pressure_dependence() {
        let corrector = RefractiveIndexCorrector::hene();
        let n_low = corrector.compute(STD_TEMPERATURE_C, 95000.0, STD_HUMIDITY);
        let n_high = corrector.compute(STD_TEMPERATURE_C, 105000.0, STD_HUMIDITY);
        // Higher pressure -> higher refractive index (denser air)
        assert!(n_high > n_low, "Higher pressure should give higher n");
    }

    #[test]
    fn test_edlen_humidity_dependence() {
        let corrector = RefractiveIndexCorrector::hene();
        let n_dry = corrector.compute(STD_TEMPERATURE_C, STD_PRESSURE_PA, 0.0);
        let n_humid = corrector.compute(STD_TEMPERATURE_C, STD_PRESSURE_PA, 1.0);
        // Higher humidity -> slightly lower refractive index (water vapor is less dense)
        assert!(
            n_humid < n_dry,
            "Humid air should have lower n: dry={:.8}, humid={:.8}",
            n_dry,
            n_humid
        );
    }

    #[test]
    fn test_edlen_vacuum() {
        let corrector = RefractiveIndexCorrector::hene();
        let n_vacuum = corrector.compute(20.0, 0.0, 0.0);
        // At zero pressure, n should be ~1.0
        assert!(
            (n_vacuum - 1.0).abs() < 1.0e-6,
            "Vacuum n should be ~1.0, got {}",
            n_vacuum
        );
    }

    #[test]
    fn test_wavelength_in_air() {
        let corrector = RefractiveIndexCorrector::hene();
        let lambda_air = corrector.wavelength_in_air(STD_TEMPERATURE_C, STD_PRESSURE_PA, STD_HUMIDITY);
        // Should be slightly less than vacuum wavelength
        assert!(lambda_air < HENE_WAVELENGTH_M);
        // Difference is ~171 pm (n_air ~ 1.000271, so delta = lambda * 2.71e-4 ~ 1.71e-13)
        assert!((lambda_air - HENE_WAVELENGTH_M).abs() < 1.0e-9);
    }

    #[test]
    fn test_group_refractive_index() {
        let corrector = RefractiveIndexCorrector::hene();
        let n_group = corrector.group_index(STD_TEMPERATURE_C, STD_PRESSURE_PA, STD_HUMIDITY);
        let n_phase = corrector.standard_conditions();
        // Group index is slightly higher than phase index for normal dispersion
        assert!(
            n_group > n_phase,
            "Group index should exceed phase index: n_g={:.8}, n_p={:.8}",
            n_group,
            n_phase
        );
    }

    // --- NonlinearityCorrector tests ---

    #[test]
    fn test_ideal_corrector_passthrough() {
        let nlc = NonlinearityCorrector::ideal();
        let (i_corr, q_corr) = nlc.correct(1.0, 0.5);
        assert!((i_corr - 1.0).abs() < EPSILON);
        assert!((q_corr - 0.5).abs() < EPSILON);
    }

    #[test]
    fn test_dc_offset_removal() {
        let nlc = NonlinearityCorrector::new(0.1, -0.2, 1.0, 1.0, 0.0);
        let (i_corr, q_corr) = nlc.correct(1.1, 0.3);
        assert!((i_corr - 1.0).abs() < EPSILON);
        assert!((q_corr - 0.5).abs() < EPSILON);
    }

    #[test]
    fn test_gain_normalization() {
        let nlc = NonlinearityCorrector::new(0.0, 0.0, 2.0, 0.5, 0.0);
        let (i_corr, q_corr) = nlc.correct(2.0, 0.5);
        assert!((i_corr - 1.0).abs() < EPSILON);
        assert!((q_corr - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_ellipse_fit_ideal() {
        // Generate ideal circular I/Q data
        let n = 1000;
        let i_data: Vec<f64> = (0..n).map(|i| (2.0 * PI * i as f64 / n as f64).cos()).collect();
        let q_data: Vec<f64> = (0..n).map(|i| (2.0 * PI * i as f64 / n as f64).sin()).collect();
        let nlc = NonlinearityCorrector::fit(&i_data, &q_data);
        assert!(nlc.i_offset.abs() < 0.01, "I offset should be ~0, got {}", nlc.i_offset);
        assert!(nlc.q_offset.abs() < 0.01, "Q offset should be ~0, got {}", nlc.q_offset);
        assert!((nlc.i_gain - 1.0).abs() < 0.05, "I gain should be ~1, got {}", nlc.i_gain);
        assert!((nlc.q_gain - 1.0).abs() < 0.05, "Q gain should be ~1, got {}", nlc.q_gain);
        assert!(nlc.phase_error.abs() < 0.05, "Phase error should be ~0, got {}", nlc.phase_error);
    }

    #[test]
    fn test_ellipse_fit_with_offset() {
        let n = 1000;
        let dc_i = 0.3;
        let dc_q = -0.2;
        let i_data: Vec<f64> = (0..n)
            .map(|i| dc_i + (2.0 * PI * i as f64 / n as f64).cos())
            .collect();
        let q_data: Vec<f64> = (0..n)
            .map(|i| dc_q + (2.0 * PI * i as f64 / n as f64).sin())
            .collect();
        let nlc = NonlinearityCorrector::fit(&i_data, &q_data);
        assert!(
            (nlc.i_offset - dc_i).abs() < 0.01,
            "Expected I offset {}, got {}",
            dc_i,
            nlc.i_offset
        );
        assert!(
            (nlc.q_offset - dc_q).abs() < 0.01,
            "Expected Q offset {}, got {}",
            dc_q,
            nlc.q_offset
        );
    }

    #[test]
    fn test_nonlinearity_batch_correction() {
        let nlc = NonlinearityCorrector::new(0.1, 0.2, 1.0, 1.0, 0.0);
        let i_data = vec![1.1, 0.1, -0.9];
        let q_data = vec![0.2, 1.2, 0.2];
        let (i_corr, q_corr) = nlc.correct_batch(&i_data, &q_data);
        assert_eq!(i_corr.len(), 3);
        assert_eq!(q_corr.len(), 3);
        assert!((i_corr[0] - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_estimated_nonlinearity() {
        let nlc = NonlinearityCorrector::new(0.0, 0.0, 1.0, 1.1, 0.05);
        let nm = nlc.estimated_nonlinearity_m(HENE_WAVELENGTH_M, 2);
        // Should be a few nanometers
        assert!(nm > 0.0);
        assert!(nm < 100.0e-9, "Expected < 100 nm, got {:.2e}", nm);
    }

    // --- VelocityEstimator tests ---

    #[test]
    fn test_velocity_stationary() {
        let config = HeterodyneConfig::hene_zeeman();
        let mut ve = VelocityEstimator::from_config(&config, 1.000271);
        // Constant phase -> zero velocity
        let v1 = ve.estimate_sample(1.0);
        let v2 = ve.estimate_sample(1.0);
        assert!((v1).abs() < EPSILON); // First sample, no previous
        assert!((v2).abs() < EPSILON); // No phase change
    }

    #[test]
    fn test_velocity_constant_motion() {
        let config = HeterodyneConfig::hene_zeeman();
        let n_air = 1.000271;
        let ve = VelocityEstimator::from_config(&config, n_air);
        // Linear phase ramp = constant velocity
        let phase_rate = 2.0 * PI * 1000.0; // 1000 Hz Doppler
        let n = 100;
        let phases: Vec<f64> = (0..n)
            .map(|i| i as f64 * phase_rate / config.sample_rate_hz)
            .collect();
        let velocities = ve.estimate_batch(&phases);
        // Check interior points (central difference should be accurate)
        for i in 1..n - 1 {
            let expected_v = phase_rate * config.wavelength_m
                / (2.0 * PI * config.num_passes as f64 * n_air);
            assert!(
                (velocities[i] - expected_v).abs() < expected_v * 0.01,
                "Velocity at {} should be {:.6e}, got {:.6e}",
                i,
                expected_v,
                velocities[i]
            );
        }
    }

    #[test]
    fn test_velocity_from_doppler_freq() {
        let v = VelocityEstimator::velocity_from_doppler_freq(HENE_WAVELENGTH_M, 2, 1.0, 1000.0);
        // v = f_d * lambda / (2 * 2 * 1.0) = 1000 * 632.8e-9 / 4
        let expected = 1000.0 * HENE_WAVELENGTH_M / 4.0;
        assert!((v - expected).abs() < 1.0e-15);
    }

    #[test]
    fn test_velocity_batch_empty() {
        let config = HeterodyneConfig::hene_zeeman();
        let ve = VelocityEstimator::from_config(&config, 1.000271);
        let v = ve.estimate_batch(&[]);
        assert!(v.is_empty());
    }

    #[test]
    fn test_velocity_batch_single() {
        let config = HeterodyneConfig::hene_zeeman();
        let ve = VelocityEstimator::from_config(&config, 1.000271);
        let v = ve.estimate_batch(&[1.0]);
        assert_eq!(v.len(), 1);
        assert!((v[0]).abs() < EPSILON);
    }

    // --- CosineErrorCorrector tests ---

    #[test]
    fn test_cosine_error_zero_angle() {
        let corr = CosineErrorCorrector::from_radians(0.0);
        let d = corr.correct(1.0e-6);
        assert!((d - 1.0e-6).abs() < EPSILON);
    }

    #[test]
    fn test_cosine_error_small_angle() {
        // 1 mrad misalignment -> 0.5 ppm error
        let corr = CosineErrorCorrector::from_milliradians(1.0);
        let error_ppm = corr.error_ppm();
        assert!(
            (error_ppm - 0.5).abs() < 0.01,
            "Expected ~0.5 ppm, got {:.4}",
            error_ppm
        );
    }

    #[test]
    fn test_cosine_error_correction() {
        let theta = 0.01; // 10 mrad
        let corr = CosineErrorCorrector::from_radians(theta);
        let measured = 1.0; // 1 m measured
        let true_d = corr.correct(measured);
        // true = measured / cos(theta) > measured
        assert!(true_d > measured);
        assert!((true_d - measured / theta.cos()).abs() < EPSILON);
    }

    #[test]
    fn test_cosine_error_from_arcseconds() {
        let corr = CosineErrorCorrector::from_arcseconds(206.265);
        // 206.265 arcsec = 1 mrad
        assert!((corr.angle_rad() - 1.0e-3).abs() < 1.0e-6);
    }

    #[test]
    fn test_cosine_error_batch() {
        let corr = CosineErrorCorrector::from_milliradians(1.0);
        let measured = vec![1.0e-6, 2.0e-6, 3.0e-6];
        let corrected = corr.correct_batch(&measured);
        assert_eq!(corrected.len(), 3);
        for (m, c) in measured.iter().zip(corrected.iter()) {
            assert!(c >= m);
        }
    }

    #[test]
    fn test_cosine_error_set_angle() {
        let mut corr = CosineErrorCorrector::from_radians(0.0);
        assert!((corr.relative_error()).abs() < EPSILON);
        corr.set_angle_rad(0.01);
        assert!(corr.relative_error() > 0.0);
    }

    // --- EnvironmentalCompensator tests ---

    #[test]
    fn test_env_compensator_no_change() {
        let comp = EnvironmentalCompensator::new(
            HENE_WAVELENGTH_M,
            0.0,
            STD_TEMPERATURE_C,
            STD_PRESSURE_PA,
            STD_HUMIDITY,
        );
        // If conditions haven't changed, compensation should be identity
        let d = comp.compensate(1.0e-6, STD_TEMPERATURE_C, STD_PRESSURE_PA, STD_HUMIDITY);
        assert!(
            (d - 1.0e-6).abs() < 1.0e-12,
            "No-change compensation should be identity, got {:.4e}",
            d
        );
    }

    #[test]
    fn test_env_compensator_temperature_change() {
        let comp = EnvironmentalCompensator::new(
            HENE_WAVELENGTH_M,
            0.0,
            20.0,
            STD_PRESSURE_PA,
            STD_HUMIDITY,
        );
        let d_same = comp.compensate(1.0e-3, 20.0, STD_PRESSURE_PA, STD_HUMIDITY);
        let d_hot = comp.compensate(1.0e-3, 25.0, STD_PRESSURE_PA, STD_HUMIDITY);
        // Temperature change should cause a measurable correction
        assert!(
            (d_same - d_hot).abs() > 1.0e-9,
            "5 C change should produce > 1 nm correction on 1 mm"
        );
    }

    #[test]
    fn test_env_compensator_dead_path() {
        let comp = EnvironmentalCompensator::new(
            HENE_WAVELENGTH_M,
            0.1, // 100 mm dead path
            20.0,
            STD_PRESSURE_PA,
            STD_HUMIDITY,
        );
        let error = comp.dead_path_error(25.0, STD_PRESSURE_PA, STD_HUMIDITY);
        // 100 mm dead path * ~5 ppm/K * 5 K ~ 2.5 um
        assert!(
            error.abs() > 1.0e-7,
            "Dead path error should be significant: {:.4e}",
            error
        );
    }

    #[test]
    fn test_env_compensator_cte() {
        let mut comp = EnvironmentalCompensator::new(
            HENE_WAVELENGTH_M,
            0.0,
            20.0,
            STD_PRESSURE_PA,
            STD_HUMIDITY,
        );
        // Steel CTE ~ 12e-6 /K
        comp.set_target_cte(12.0e-6, 20.0);
        let d_ref = comp.compensate(1.0, 20.0, STD_PRESSURE_PA, STD_HUMIDITY);
        let d_hot = comp.compensate(1.0, 25.0, STD_PRESSURE_PA, STD_HUMIDITY);
        // 5 K * 12e-6 /K * 1 m = 60 um expansion
        let expansion = (d_hot - d_ref).abs();
        assert!(
            expansion > 50.0e-6,
            "CTE correction should add ~60 um, got {:.4e}",
            expansion
        );
    }

    #[test]
    fn test_env_compensator_refractive_index() {
        let comp = EnvironmentalCompensator::new(
            HENE_WAVELENGTH_M,
            0.0,
            20.0,
            STD_PRESSURE_PA,
            0.5,
        );
        let n = comp.current_refractive_index(20.0, STD_PRESSURE_PA, 0.5);
        assert!((n - comp.initial_refractive_index()).abs() < 1.0e-10);
    }

    // --- InterferometerPipeline tests ---

    #[test]
    fn test_pipeline_creation() {
        let config = HeterodyneConfig::hene_zeeman();
        let pipeline = InterferometerPipeline::new(config);
        assert!(pipeline.nonlinearity.is_none());
        assert!(pipeline.cosine_error.is_none());
    }

    #[test]
    fn test_pipeline_process() {
        let config = HeterodyneConfig::hene_zeeman();
        let mut pipeline = InterferometerPipeline::new(config.clone());
        let n = 1000;
        let fs = config.sample_rate_hz;
        let f_beat = config.split_frequency_hz;
        let samples: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / fs;
                (2.0 * PI * f_beat * t).cos()
            })
            .collect();
        let (displacements, velocities) = pipeline.process(&samples);
        assert_eq!(displacements.len(), n);
        assert_eq!(velocities.len(), n);
    }

    #[test]
    fn test_pipeline_reset() {
        let config = HeterodyneConfig::hene_zeeman();
        let mut pipeline = InterferometerPipeline::new(config.clone());
        let samples = vec![1.0; 100];
        pipeline.process(&samples);
        pipeline.reset();
        // After reset, phase should be zero
        assert!((pipeline.unwrapper.current_phase()).abs() < EPSILON);
    }

    // --- Utility function tests ---

    #[test]
    fn test_meters_to_nm() {
        assert!((meters_to_nm(1.0e-9) - 1.0).abs() < EPSILON);
        assert!((meters_to_nm(632.8e-9) - 632.8).abs() < EPSILON);
    }

    #[test]
    fn test_nm_to_meters() {
        assert!((nm_to_meters(1.0) - 1.0e-9).abs() < EPSILON);
    }

    #[test]
    fn test_doppler_frequency_fn() {
        // 1 m/s velocity, HeNe double-pass
        let f_d = doppler_frequency(1.0, HENE_WAVELENGTH_M, 2, 1.0);
        // f_d = 2 * 2 * 1.0 / 632.8e-9 = ~6.32 MHz
        assert!((f_d - 4.0 / HENE_WAVELENGTH_M).abs() < 1.0);
    }

    #[test]
    fn test_max_velocity_fn() {
        let v = max_velocity(1.8e6, HENE_WAVELENGTH_M, 2);
        assert!((v - 0.2847).abs() < 0.001);
    }

    #[test]
    fn test_displacement_resolution_fn() {
        // 0.001 rad RMS phase noise
        let sigma_d = displacement_resolution(0.001, HENE_WAVELENGTH_M, 2, 1.000271);
        // ~0.05 nm
        let sigma_nm = meters_to_nm(sigma_d);
        assert!(
            sigma_nm < 1.0,
            "0.001 rad phase noise should give sub-nm resolution, got {:.4} nm",
            sigma_nm
        );
    }
}
