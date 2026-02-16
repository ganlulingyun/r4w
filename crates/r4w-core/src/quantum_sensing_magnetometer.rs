//! # Quantum Sensing Magnetometer for NV-Center Diamond Magnetometry
//!
//! This module implements signal processing for nitrogen-vacancy (NV) center
//! diamond quantum magnetometry, covering the full pipeline from ODMR spectral
//! analysis through vector field reconstruction and sensitivity estimation.
//!
//! ## Background
//!
//! The nitrogen-vacancy center in diamond is a point defect consisting of a
//! substitutional nitrogen atom adjacent to a lattice vacancy. Its ground state
//! is a spin-1 triplet with a zero-field splitting D = 2.87 GHz between the
//! m_s = 0 and m_s = +/-1 sublevels. An external magnetic field B along the
//! NV axis lifts the degeneracy via the Zeeman effect:
//!
//! ```text
//!   f_+ = D + gamma_e * B_NV
//!   f_- = D - gamma_e * B_NV
//! ```
//!
//! where gamma_e / (2*pi) = 28.025 GHz/T is the electron gyromagnetic ratio.
//!
//! Optically Detected Magnetic Resonance (ODMR) exploits spin-dependent
//! fluorescence: the m_s = 0 state fluoresces more brightly than m_s = +/-1,
//! producing dips in photoluminescence when microwave frequencies match the
//! spin transition energies.
//!
//! ## Applications
//!
//! - **Brain imaging (MEG)**: Non-cryogenic magnetoencephalography
//! - **Single-molecule NMR**: Nanoscale nuclear magnetic resonance
//! - **Geological surveys**: Field mapping without SQUID cooling
//! - **Materials characterisation**: Magnetic domain imaging
//! - **Navigation**: Dead-reckoning via Earth's field measurement
//!
//! ## Components
//!
//! - [`NvCenterConfig`] -- NV axis orientation, zero-field splitting, gyromagnetic ratio
//! - [`OdmrProcessor`] -- Lorentzian dip fitting for ODMR spectroscopy
//! - [`MagneticFieldExtractor`] -- Extract B_NV from ODMR splitting
//! - [`RamseyInterferometry`] -- T2* coherence measurement
//! - [`SpinEchoProcessor`] -- Hahn echo T2 measurement
//! - [`DynamicalDecoupling`] -- CPMG/XY-8 enhanced coherence and AC magnetometry
//! - [`AcMagnetometer`] -- Lock-in detection for AC field measurement
//! - [`VectorMagnetometer`] -- 3D field reconstruction from 4 NV orientations
//! - [`SensitivityCalculator`] -- Shot-noise-limited sensitivity estimation
//! - [`TemperatureCompensator`] -- Thermal drift correction for D(T)
//! - [`NoiseSpectrumAnalyzer`] -- Magnetic noise spectrum from relaxometry
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::quantum_sensing_magnetometer::*;
//!
//! // Configure NV center along [111] direction
//! let config = NvCenterConfig::default();
//!
//! // Extract field from ODMR splitting of 100 MHz
//! let extractor = MagneticFieldExtractor::new(&config);
//! let b_field = extractor.from_splitting_hz(100.0e6);
//! assert!((b_field - 1.784e-3).abs() < 1.0e-5); // ~1.784 mT
//!
//! // Estimate sensitivity
//! let calc = SensitivityCalculator::new(0.03, 1e6, 1.0e-6);
//! let eta = calc.sensitivity_t_per_sqrt_hz(&config);
//! assert!(eta > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Zero-field splitting of the NV ground state triplet (Hz).
pub const D_ZFS_HZ: f64 = 2.87e9;

/// Electron gyromagnetic ratio divided by 2*pi (Hz/T).
pub const GAMMA_E_HZ_PER_T: f64 = 28.025e9;

/// Temperature coefficient of the zero-field splitting (Hz/K).
/// dD/dT ~ -74 kHz/K near room temperature.
pub const DD_DT_HZ_PER_K: f64 = -74.0e3;

/// Reference temperature for D_ZFS_HZ (K).
pub const T_REF_K: f64 = 300.0;

/// Speed of light in vacuum (m/s) -- used in some calculations.
const _C: f64 = 299_792_458.0;

// ---------------------------------------------------------------------------
// NV center orientations in diamond lattice
// ---------------------------------------------------------------------------

/// The four NV-axis orientations in the diamond lattice (unit vectors).
/// These correspond to the [111], [-1-11], [-11-1], and [1-1-1] directions.
pub const NV_ORIENTATIONS: [[f64; 3]; 4] = [
    [0.577_350_269_189_626, 0.577_350_269_189_626, 0.577_350_269_189_626],   // [1, 1, 1] / sqrt(3)
    [-0.577_350_269_189_626, -0.577_350_269_189_626, 0.577_350_269_189_626],  // [-1,-1, 1] / sqrt(3)
    [-0.577_350_269_189_626, 0.577_350_269_189_626, -0.577_350_269_189_626],  // [-1, 1,-1] / sqrt(3)
    [0.577_350_269_189_626, -0.577_350_269_189_626, -0.577_350_269_189_626],  // [ 1,-1,-1] / sqrt(3)
];

// ---------------------------------------------------------------------------
// NvCenterConfig
// ---------------------------------------------------------------------------

/// Configuration for a single NV center (or ensemble aligned along one axis).
#[derive(Debug, Clone)]
pub struct NvCenterConfig {
    /// NV axis unit vector (normalised).
    pub axis: [f64; 3],
    /// Zero-field splitting (Hz). Default: 2.87 GHz.
    pub d_zfs_hz: f64,
    /// Electron gyromagnetic ratio / (2*pi) (Hz/T). Default: 28.025 GHz/T.
    pub gamma_e_hz_per_t: f64,
}

impl Default for NvCenterConfig {
    fn default() -> Self {
        Self {
            axis: NV_ORIENTATIONS[0],
            d_zfs_hz: D_ZFS_HZ,
            gamma_e_hz_per_t: GAMMA_E_HZ_PER_T,
        }
    }
}

impl NvCenterConfig {
    /// Create a config with a specific NV axis orientation (will be normalised).
    pub fn with_axis(axis: [f64; 3]) -> Self {
        let len = (axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]).sqrt();
        let norm = if len > 1e-15 {
            [axis[0] / len, axis[1] / len, axis[2] / len]
        } else {
            NV_ORIENTATIONS[0]
        };
        Self {
            axis: norm,
            ..Default::default()
        }
    }

    /// Create a config for the i-th crystallographic NV orientation (0..3).
    pub fn orientation(index: usize) -> Self {
        Self {
            axis: NV_ORIENTATIONS[index % 4],
            ..Default::default()
        }
    }

    /// Project a 3D magnetic field vector onto this NV axis.
    /// Returns the signed projection B_NV = B . n_NV.
    pub fn project_field(&self, b_vec: &[f64; 3]) -> f64 {
        b_vec[0] * self.axis[0] + b_vec[1] * self.axis[1] + b_vec[2] * self.axis[2]
    }

    /// Compute the two ODMR transition frequencies for a given field projection.
    /// Returns (f_minus, f_plus) in Hz.
    pub fn odmr_frequencies(&self, b_nv_tesla: f64) -> (f64, f64) {
        let shift = self.gamma_e_hz_per_t * b_nv_tesla.abs();
        (self.d_zfs_hz - shift, self.d_zfs_hz + shift)
    }
}

// ---------------------------------------------------------------------------
// Lorentzian fitting for ODMR
// ---------------------------------------------------------------------------

/// A single Lorentzian dip parameterisation.
#[derive(Debug, Clone, Copy)]
pub struct LorentzianDip {
    /// Centre frequency (Hz).
    pub center_hz: f64,
    /// Full-width at half-maximum (Hz).
    pub fwhm_hz: f64,
    /// Dip depth (contrast), 0..1.
    pub depth: f64,
}

impl LorentzianDip {
    /// Evaluate the Lorentzian dip at frequency f.
    /// Returns the dip factor (1 - depth * L(f)), where L(f) is normalised.
    pub fn evaluate(&self, f_hz: f64) -> f64 {
        let half_gamma = self.fwhm_hz / 2.0;
        let df = f_hz - self.center_hz;
        let lorentz = (half_gamma * half_gamma) / (df * df + half_gamma * half_gamma);
        1.0 - self.depth * lorentz
    }
}

/// Optically Detected Magnetic Resonance processor.
///
/// Fits one or two Lorentzian dips to an ODMR fluorescence spectrum to
/// extract transition frequencies.
#[derive(Debug, Clone)]
pub struct OdmrProcessor {
    /// Frequency axis (Hz).
    pub freq_hz: Vec<f64>,
    /// Expected ODMR linewidth (Hz) for initial guess.
    pub expected_linewidth_hz: f64,
    /// Minimum contrast to accept a dip.
    pub min_contrast: f64,
}

impl OdmrProcessor {
    /// Create a new ODMR processor.
    ///
    /// * `freq_start_hz` - Start of sweep range (Hz).
    /// * `freq_stop_hz` - End of sweep range (Hz).
    /// * `num_points` - Number of frequency points.
    /// * `expected_linewidth_hz` - Expected FWHM of ODMR dips (Hz).
    pub fn new(
        freq_start_hz: f64,
        freq_stop_hz: f64,
        num_points: usize,
        expected_linewidth_hz: f64,
    ) -> Self {
        let step = if num_points > 1 {
            (freq_stop_hz - freq_start_hz) / (num_points - 1) as f64
        } else {
            0.0
        };
        let freq_hz: Vec<f64> = (0..num_points)
            .map(|i| freq_start_hz + i as f64 * step)
            .collect();
        Self {
            freq_hz,
            expected_linewidth_hz,
            min_contrast: 0.005,
        }
    }

    /// Generate a synthetic ODMR spectrum for testing.
    ///
    /// Returns fluorescence values (0..1) with Lorentzian dips.
    pub fn generate_spectrum(&self, dips: &[LorentzianDip]) -> Vec<f64> {
        self.freq_hz
            .iter()
            .map(|&f| {
                let mut val = 1.0;
                for dip in dips {
                    val *= dip.evaluate(f);
                }
                val
            })
            .collect()
    }

    /// Find dip positions in an ODMR spectrum by locating local minima.
    ///
    /// Returns a vector of `LorentzianDip` structs fitted to each detected dip.
    pub fn find_dips(&self, spectrum: &[f64]) -> Vec<LorentzianDip> {
        if spectrum.len() != self.freq_hz.len() || spectrum.len() < 3 {
            return Vec::new();
        }

        let mut dips = Vec::new();

        // Find local minima
        for i in 1..spectrum.len() - 1 {
            if spectrum[i] < spectrum[i - 1] && spectrum[i] < spectrum[i + 1] {
                // Parabolic interpolation for sub-bin centre
                let y_l = spectrum[i - 1];
                let y_c = spectrum[i];
                let y_r = spectrum[i + 1];
                let denom = 2.0 * (y_l - 2.0 * y_c + y_r);
                let offset = if denom.abs() > 1e-30 {
                    (y_l - y_r) / denom
                } else {
                    0.0
                };
                let step = if self.freq_hz.len() > 1 {
                    self.freq_hz[1] - self.freq_hz[0]
                } else {
                    1.0
                };
                let center = self.freq_hz[i] + offset * step;

                // Estimate depth: baseline is ~1.0 for normalised spectrum
                let depth = 1.0 - y_c;
                if depth < self.min_contrast {
                    continue;
                }

                // Estimate FWHM: find half-depth crossings
                let half_level = 1.0 - depth / 2.0;
                let mut left_f = self.freq_hz[i];
                for j in (0..i).rev() {
                    if spectrum[j] >= half_level {
                        // Linear interpolate
                        let frac = (half_level - spectrum[j + 1])
                            / (spectrum[j] - spectrum[j + 1]);
                        left_f = self.freq_hz[j + 1]
                            + frac * (self.freq_hz[j] - self.freq_hz[j + 1]);
                        break;
                    }
                }
                let mut right_f = self.freq_hz[i];
                for j in (i + 1)..spectrum.len() {
                    if spectrum[j] >= half_level {
                        let frac = (half_level - spectrum[j - 1])
                            / (spectrum[j] - spectrum[j - 1]);
                        right_f = self.freq_hz[j - 1]
                            + frac * (self.freq_hz[j] - self.freq_hz[j - 1]);
                        break;
                    }
                }
                let fwhm = (right_f - left_f).max(self.expected_linewidth_hz * 0.1);

                dips.push(LorentzianDip {
                    center_hz: center,
                    fwhm_hz: fwhm,
                    depth,
                });
            }
        }

        dips
    }

    /// Extract the splitting (Hz) between two dips.
    /// Returns None if fewer than 2 dips are found.
    pub fn extract_splitting(&self, spectrum: &[f64]) -> Option<f64> {
        let dips = self.find_dips(spectrum);
        if dips.len() >= 2 {
            Some((dips[1].center_hz - dips[0].center_hz).abs())
        } else {
            None
        }
    }
}

// ---------------------------------------------------------------------------
// MagneticFieldExtractor
// ---------------------------------------------------------------------------

/// Extracts the magnetic field projection from ODMR frequency splitting.
///
/// The splitting between the m_s = -1 and m_s = +1 transitions is:
///     delta_f = 2 * gamma_e * B_NV
/// hence:
///     B_NV = delta_f / (2 * gamma_e)
#[derive(Debug, Clone)]
pub struct MagneticFieldExtractor {
    /// Gyromagnetic ratio / (2*pi) (Hz/T).
    pub gamma_e: f64,
}

impl MagneticFieldExtractor {
    /// Create from an NV center config.
    pub fn new(config: &NvCenterConfig) -> Self {
        Self {
            gamma_e: config.gamma_e_hz_per_t,
        }
    }

    /// Compute B_NV (Tesla) from frequency splitting (Hz).
    pub fn from_splitting_hz(&self, splitting_hz: f64) -> f64 {
        splitting_hz / (2.0 * self.gamma_e)
    }

    /// Compute B_NV (Tesla) from the two ODMR dip frequencies (Hz).
    pub fn from_dip_frequencies(&self, f_minus: f64, f_plus: f64) -> f64 {
        let splitting = (f_plus - f_minus).abs();
        self.from_splitting_hz(splitting)
    }

    /// Compute the expected splitting (Hz) for a given B_NV (Tesla).
    pub fn expected_splitting_hz(&self, b_nv_tesla: f64) -> f64 {
        2.0 * self.gamma_e * b_nv_tesla.abs()
    }
}

// ---------------------------------------------------------------------------
// Ramsey Interferometry
// ---------------------------------------------------------------------------

/// Ramsey interferometry processor for T2* (free-induction-decay) measurement.
///
/// The Ramsey fringe signal is:
///     P(tau) = (1 + C * cos(2*pi*delta*tau) * exp(-(tau/T2*)^n)) / 2
///
/// where:
/// - C is the fringe contrast
/// - delta is the detuning frequency (Hz)
/// - T2* is the inhomogeneous dephasing time
/// - n is the decay exponent (1 = exponential, 2 = Gaussian)
#[derive(Debug, Clone)]
pub struct RamseyInterferometry {
    /// Fringe contrast (0..1).
    pub contrast: f64,
    /// Detuning from resonance (Hz).
    pub detuning_hz: f64,
    /// Dephasing time T2* (s).
    pub t2_star_s: f64,
    /// Decay exponent (1 = exponential, 2 = Gaussian).
    pub decay_exponent: f64,
}

impl RamseyInterferometry {
    /// Create with typical NV ensemble parameters.
    pub fn new(contrast: f64, detuning_hz: f64, t2_star_s: f64) -> Self {
        Self {
            contrast,
            detuning_hz,
            t2_star_s,
            decay_exponent: 2.0, // Gaussian is typical for NV ensembles
        }
    }

    /// Evaluate the Ramsey signal P(tau) at free-precession time tau (seconds).
    pub fn signal(&self, tau_s: f64) -> f64 {
        let phase = 2.0 * PI * self.detuning_hz * tau_s;
        let decay = (-(tau_s / self.t2_star_s).powf(self.decay_exponent)).exp();
        0.5 * (1.0 + self.contrast * phase.cos() * decay)
    }

    /// Generate a Ramsey fringe curve over a range of tau values.
    pub fn generate_curve(&self, tau_values: &[f64]) -> Vec<f64> {
        tau_values.iter().map(|&t| self.signal(t)).collect()
    }

    /// Fit T2* from measured Ramsey data using envelope extraction.
    ///
    /// Uses the absolute value of oscillation envelope to estimate decay.
    /// Returns estimated T2* in seconds.
    pub fn fit_t2_star(&self, tau_values: &[f64], measurements: &[f64]) -> Option<f64> {
        if tau_values.len() != measurements.len() || tau_values.len() < 4 {
            return None;
        }

        // Extract envelope by computing |P(tau) - 0.5| * 2 / C
        // Then fit log(envelope) = -(tau/T2*)^n
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xx = 0.0;
        let mut sum_xy = 0.0;
        let mut count = 0.0;

        for (i, (&tau, &p)) in tau_values.iter().zip(measurements.iter()).enumerate() {
            if i == 0 {
                continue;
            }
            let envelope = ((p - 0.5) * 2.0).abs();
            if envelope < 0.01 || envelope > 1.5 {
                continue;
            }
            // For Gaussian decay (n=2): log(envelope/C) = -(tau/T2*)^2
            // Linear fit: log(envelope) vs tau^2
            let x = tau * tau;
            let y = envelope.ln();
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
            count += 1.0;
        }

        if count < 2.0 {
            return None;
        }

        // Linear regression: y = a + b*x, where b = -1/T2*^2
        let b = (count * sum_xy - sum_x * sum_y) / (count * sum_xx - sum_x * sum_x);
        if b >= 0.0 {
            return None; // No decay detected
        }
        let t2_star = (-1.0 / b).sqrt();
        Some(t2_star)
    }
}

// ---------------------------------------------------------------------------
// Spin Echo (Hahn Echo) Processor
// ---------------------------------------------------------------------------

/// Hahn spin echo processor for T2 coherence time measurement.
///
/// The spin echo sequence is: pi/2 -- tau -- pi -- tau -- echo
/// The echo amplitude decays as:
///     S(2*tau) = S_0 * exp(-(2*tau / T2)^n)
///
/// The pi pulse refocuses static inhomogeneities, extending coherence
/// beyond T2* to T2 (which can be orders of magnitude longer).
#[derive(Debug, Clone)]
pub struct SpinEchoProcessor {
    /// Initial echo amplitude (normalised).
    pub s0: f64,
    /// T2 coherence time (s).
    pub t2_s: f64,
    /// Decay exponent (typically 1 for single NV, ~1-3 for ensemble).
    pub decay_exponent: f64,
}

impl SpinEchoProcessor {
    /// Create with given parameters.
    pub fn new(s0: f64, t2_s: f64, decay_exponent: f64) -> Self {
        Self {
            s0,
            t2_s,
            decay_exponent,
        }
    }

    /// Typical single NV diamond parameters.
    pub fn single_nv() -> Self {
        Self::new(1.0, 1.0e-3, 1.0) // T2 ~ 1 ms, exponential
    }

    /// Typical NV ensemble parameters.
    pub fn ensemble() -> Self {
        Self::new(1.0, 10.0e-6, 2.0) // T2 ~ 10 us, Gaussian
    }

    /// Echo amplitude at total evolution time t_total = 2*tau.
    pub fn echo_amplitude(&self, t_total_s: f64) -> f64 {
        self.s0 * (-(t_total_s / self.t2_s).powf(self.decay_exponent)).exp()
    }

    /// Generate echo decay curve.
    pub fn generate_decay(&self, t_total_values: &[f64]) -> Vec<f64> {
        t_total_values
            .iter()
            .map(|&t| self.echo_amplitude(t))
            .collect()
    }

    /// Fit T2 from echo amplitude data using least-squares on log envelope.
    pub fn fit_t2(&self, t_total_values: &[f64], amplitudes: &[f64]) -> Option<f64> {
        if t_total_values.len() != amplitudes.len() || t_total_values.len() < 3 {
            return None;
        }

        // For exponential decay (n=1): log(S) = log(S0) - t/T2
        // Linear fit: log(S) vs t
        let n = self.decay_exponent;
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xx = 0.0;
        let mut sum_xy = 0.0;
        let mut count = 0.0;

        for (&t, &a) in t_total_values.iter().zip(amplitudes.iter()) {
            if a <= 0.0 {
                continue;
            }
            let x = t.powf(n);
            let y = a.ln();
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
            count += 1.0;
        }

        if count < 2.0 {
            return None;
        }

        let b = (count * sum_xy - sum_x * sum_y) / (count * sum_xx - sum_x * sum_x);
        if b >= 0.0 {
            return None;
        }
        // b = -1/T2^n => T2 = (-1/b)^(1/n)
        let t2 = (-1.0 / b).powf(1.0 / n);
        Some(t2)
    }
}

// ---------------------------------------------------------------------------
// Dynamical Decoupling
// ---------------------------------------------------------------------------

/// Dynamical decoupling pulse sequence type.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DdSequence {
    /// Carr-Purcell-Meiboom-Gill: equally-spaced pi pulses along Y.
    Cpmg,
    /// XY-4 sequence: pi_X, pi_Y, pi_X, pi_Y -- first-order robustness.
    Xy4,
    /// XY-8 sequence: pi_X, pi_Y, pi_X, pi_Y, pi_Y, pi_X, pi_Y, pi_X.
    Xy8,
    /// XY-16 sequence: double XY-8 with inverted phases.
    Xy16,
}

/// Dynamical decoupling processor for enhanced T2 and AC magnetometry.
///
/// Dynamical decoupling extends the coherence time by applying periodic
/// pi-pulses that refocus quasi-static noise. For AC magnetometry, the
/// pulse spacing tau is set to half the AC field period, making the
/// sequence a narrow-band filter centred at f_ac = 1/(2*tau).
#[derive(Debug, Clone)]
pub struct DynamicalDecoupling {
    /// Sequence type.
    pub sequence: DdSequence,
    /// Number of pi pulses (N).
    pub num_pulses: usize,
    /// Total sensing time (s).
    pub total_time_s: f64,
    /// Extended T2 under DD (s).
    pub t2_dd_s: f64,
    /// Contrast.
    pub contrast: f64,
}

impl DynamicalDecoupling {
    /// Create a new DD configuration.
    pub fn new(sequence: DdSequence, num_pulses: usize, total_time_s: f64) -> Self {
        Self {
            sequence,
            num_pulses,
            total_time_s,
            t2_dd_s: total_time_s * 2.0, // Placeholder; set by user
            contrast: 1.0,
        }
    }

    /// Inter-pulse spacing tau = T_total / N.
    pub fn tau_s(&self) -> f64 {
        if self.num_pulses == 0 {
            self.total_time_s
        } else {
            self.total_time_s / self.num_pulses as f64
        }
    }

    /// AC sensitivity frequency: f_ac = 1 / (2 * tau) = N / (2 * T_total).
    pub fn ac_frequency_hz(&self) -> f64 {
        if self.total_time_s <= 0.0 || self.num_pulses == 0 {
            return 0.0;
        }
        self.num_pulses as f64 / (2.0 * self.total_time_s)
    }

    /// Filter function |W(f)|^2 for the DD sequence.
    /// Normalised to peak of 1 at the AC sensitivity frequency.
    pub fn filter_function(&self, f_hz: f64) -> f64 {
        if self.num_pulses == 0 || self.total_time_s <= 0.0 {
            return 0.0;
        }
        let n = self.num_pulses;
        let tau = self.tau_s();
        let omega_tau = 2.0 * PI * f_hz * tau;

        // CPMG filter: |W(f)|^2 = sin^2(N*pi*f*tau) / cos^2(pi*f*tau)
        // normalised version
        match self.sequence {
            DdSequence::Cpmg | DdSequence::Xy4 | DdSequence::Xy8 | DdSequence::Xy16 => {
                let sin_half = (omega_tau / 2.0).sin();
                if sin_half.abs() < 1e-15 {
                    return 0.0;
                }
                let numerator = (n as f64 * omega_tau / 2.0).sin();
                let raw = (numerator / (n as f64 * sin_half)).powi(2);
                // The actual DD filter peaks when f = 1/(2*tau)
                // For CPMG: W(f) = 8*sin^2(pi*f*tau) * sin^2(N*pi*f*tau) / (pi*f*tau)^2
                // Simplified normalised form:
                let half_tau = PI * f_hz * tau;
                let cos_val = half_tau.cos();
                if cos_val.abs() < 1e-15 {
                    // At resonance f = 1/(2*tau), cos(pi/2) = 0
                    return 1.0;
                }
                let sin_n = (n as f64 * half_tau).sin();
                let val = (sin_n / cos_val).powi(2) / (n as f64).powi(2);
                val.min(1.0)
            }
        }
    }

    /// Signal amplitude under DD with AC magnetic field at frequency f_ac.
    ///
    /// S = 0.5 * (1 + C * cos(2*gamma_e * B_ac * T * W(f_ac)) * exp(-(T/T2_DD)^n))
    pub fn signal(
        &self,
        b_ac_tesla: f64,
        f_ac_hz: f64,
        config: &NvCenterConfig,
    ) -> f64 {
        let w = self.filter_function(f_ac_hz).sqrt();
        let phase = 2.0 * PI * config.gamma_e_hz_per_t * b_ac_tesla * self.total_time_s * w;
        let decay = (-(self.total_time_s / self.t2_dd_s)).exp();
        0.5 * (1.0 + self.contrast * phase.cos() * decay)
    }

    /// Number of basic pulse blocks for each sequence type.
    pub fn block_size(&self) -> usize {
        match self.sequence {
            DdSequence::Cpmg => 1,
            DdSequence::Xy4 => 4,
            DdSequence::Xy8 => 8,
            DdSequence::Xy16 => 16,
        }
    }

    /// Pulse phases (radians) for one block of the sequence.
    pub fn pulse_phases(&self) -> Vec<f64> {
        match self.sequence {
            DdSequence::Cpmg => vec![PI / 2.0], // Y-axis pi pulses
            DdSequence::Xy4 => vec![0.0, PI / 2.0, 0.0, PI / 2.0], // X, Y, X, Y
            DdSequence::Xy8 => vec![
                0.0,
                PI / 2.0,
                0.0,
                PI / 2.0,
                PI / 2.0,
                0.0,
                PI / 2.0,
                0.0,
            ],
            DdSequence::Xy16 => {
                let mut phases = vec![
                    0.0,
                    PI / 2.0,
                    0.0,
                    PI / 2.0,
                    PI / 2.0,
                    0.0,
                    PI / 2.0,
                    0.0,
                ];
                // Second half: inverted phases
                let second_half: Vec<f64> = phases.iter().map(|&p| p + PI).collect();
                phases.extend(second_half);
                phases
            }
        }
    }
}

// ---------------------------------------------------------------------------
// AC Magnetometer (lock-in detection)
// ---------------------------------------------------------------------------

/// AC magnetometer using lock-in detection.
///
/// Demodulates a fluorescence signal at a known modulation frequency to
/// extract the amplitude and phase of an AC magnetic field.
#[derive(Debug, Clone)]
pub struct AcMagnetometer {
    /// Reference frequency (Hz).
    pub reference_freq_hz: f64,
    /// Low-pass filter time constant (s).
    pub time_constant_s: f64,
    /// Accumulated in-phase (I) component.
    i_accum: f64,
    /// Accumulated quadrature (Q) component.
    q_accum: f64,
    /// Number of accumulated samples.
    sample_count: usize,
}

impl AcMagnetometer {
    /// Create a new lock-in AC magnetometer.
    pub fn new(reference_freq_hz: f64, time_constant_s: f64) -> Self {
        Self {
            reference_freq_hz,
            time_constant_s,
            i_accum: 0.0,
            q_accum: 0.0,
            sample_count: 0,
        }
    }

    /// Reset the accumulated state.
    pub fn reset(&mut self) {
        self.i_accum = 0.0;
        self.q_accum = 0.0;
        self.sample_count = 0;
    }

    /// Process a batch of samples with known sample rate.
    ///
    /// Returns (amplitude, phase_rad).
    pub fn process(&mut self, samples: &[f64], sample_rate_hz: f64) -> (f64, f64) {
        for (i, &s) in samples.iter().enumerate() {
            let t = (self.sample_count + i) as f64 / sample_rate_hz;
            let ref_phase = 2.0 * PI * self.reference_freq_hz * t;
            self.i_accum += s * ref_phase.cos();
            self.q_accum += s * ref_phase.sin();
        }
        self.sample_count += samples.len();
        self.result()
    }

    /// Get the current demodulated result: (amplitude, phase_rad).
    pub fn result(&self) -> (f64, f64) {
        if self.sample_count == 0 {
            return (0.0, 0.0);
        }
        let n = self.sample_count as f64;
        let i_avg = self.i_accum / n * 2.0; // Factor of 2 for single-sideband
        let q_avg = self.q_accum / n * 2.0;
        let amplitude = (i_avg * i_avg + q_avg * q_avg).sqrt();
        let phase = q_avg.atan2(i_avg);
        (amplitude, phase)
    }

    /// Single-shot demodulation of a complete signal.
    pub fn demodulate(signal: &[f64], reference_freq_hz: f64, sample_rate_hz: f64) -> (f64, f64) {
        let mut mag = AcMagnetometer::new(reference_freq_hz, 1.0);
        mag.process(signal, sample_rate_hz)
    }
}

// ---------------------------------------------------------------------------
// Vector Magnetometer
// ---------------------------------------------------------------------------

/// Reconstructs the full 3D magnetic field vector from measurements along
/// four NV crystallographic orientations.
///
/// Each NV axis i measures B_i = B . n_i. With four non-coplanar axes
/// the system is overdetermined (4 equations, 3 unknowns) and we solve
/// via least-squares: B = (A^T A)^{-1} A^T b.
#[derive(Debug, Clone)]
pub struct VectorMagnetometer {
    /// NV axis directions (unit vectors).
    axes: [[f64; 3]; 4],
    /// Pre-computed (A^T A)^{-1} A^T matrix (3x4).
    pseudo_inverse: [[f64; 4]; 3],
}

impl VectorMagnetometer {
    /// Create with the standard diamond NV orientations.
    pub fn new() -> Self {
        Self::from_axes(NV_ORIENTATIONS)
    }

    /// Create with custom NV axis orientations.
    pub fn from_axes(axes: [[f64; 3]; 4]) -> Self {
        let pinv = Self::compute_pseudo_inverse(&axes);
        Self {
            axes,
            pseudo_inverse: pinv,
        }
    }

    /// Compute the 3x4 pseudo-inverse of the 4x3 axis matrix.
    fn compute_pseudo_inverse(axes: &[[f64; 3]; 4]) -> [[f64; 4]; 3] {
        // A^T A (3x3)
        let mut ata = [[0.0f64; 3]; 3];
        for i in 0..3 {
            for j in 0..3 {
                for k in 0..4 {
                    ata[i][j] += axes[k][i] * axes[k][j];
                }
            }
        }

        // Invert 3x3 matrix
        let inv = Self::invert_3x3(&ata);

        // (A^T A)^{-1} A^T (3x4)
        let mut pinv = [[0.0f64; 4]; 3];
        for i in 0..3 {
            for j in 0..4 {
                for k in 0..3 {
                    pinv[i][j] += inv[i][k] * axes[j][k];
                }
            }
        }

        pinv
    }

    /// Invert a 3x3 matrix using the adjugate method.
    fn invert_3x3(m: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
        let det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

        let inv_det = if det.abs() > 1e-30 { 1.0 / det } else { 0.0 };

        let mut inv = [[0.0f64; 3]; 3];
        inv[0][0] = (m[1][1] * m[2][2] - m[1][2] * m[2][1]) * inv_det;
        inv[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * inv_det;
        inv[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * inv_det;
        inv[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * inv_det;
        inv[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * inv_det;
        inv[1][2] = (m[0][2] * m[1][0] - m[0][0] * m[1][2]) * inv_det;
        inv[2][0] = (m[1][0] * m[2][1] - m[1][1] * m[2][0]) * inv_det;
        inv[2][1] = (m[0][1] * m[2][0] - m[0][0] * m[2][1]) * inv_det;
        inv[2][2] = (m[0][0] * m[1][1] - m[0][1] * m[1][0]) * inv_det;

        inv
    }

    /// Reconstruct the 3D field vector from four NV-axis projections (Tesla).
    ///
    /// `projections[i]` is B . n_i for the i-th NV orientation.
    pub fn reconstruct(&self, projections: &[f64; 4]) -> [f64; 3] {
        let mut b = [0.0f64; 3];
        for i in 0..3 {
            for j in 0..4 {
                b[i] += self.pseudo_inverse[i][j] * projections[j];
            }
        }
        b
    }

    /// Compute the field magnitude from a reconstructed vector.
    pub fn field_magnitude(b: &[f64; 3]) -> f64 {
        (b[0] * b[0] + b[1] * b[1] + b[2] * b[2]).sqrt()
    }

    /// Compute projections from a known field vector (forward model).
    pub fn project(&self, b_vec: &[f64; 3]) -> [f64; 4] {
        let mut proj = [0.0f64; 4];
        for i in 0..4 {
            proj[i] = b_vec[0] * self.axes[i][0]
                + b_vec[1] * self.axes[i][1]
                + b_vec[2] * self.axes[i][2];
        }
        proj
    }
}

impl Default for VectorMagnetometer {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// SensitivityCalculator
// ---------------------------------------------------------------------------

/// Shot-noise-limited DC magnetometry sensitivity calculator.
///
/// The sensitivity (minimum detectable field per root-Hz) is:
///     eta = 1 / (gamma_e * C * sqrt(R * T2* * t_meas))
///
/// where:
/// - gamma_e is the electron gyromagnetic ratio (Hz/T)
/// - C is the ODMR contrast
/// - R is the photon detection rate (photons/s)
/// - T2* is the dephasing time (s)
/// - t_meas is the total measurement time (s)
///
/// For a single measurement (t_meas -> inf), the sensitivity per sqrt(Hz) is:
///     eta = 1 / (gamma_e * C * sqrt(R * T2*))
#[derive(Debug, Clone)]
pub struct SensitivityCalculator {
    /// ODMR contrast (0..1).
    pub contrast: f64,
    /// Photon detection rate (photons/s).
    pub photon_rate: f64,
    /// Dephasing time T2* (s).
    pub t2_star_s: f64,
}

impl SensitivityCalculator {
    /// Create a new sensitivity calculator.
    pub fn new(contrast: f64, photon_rate: f64, t2_star_s: f64) -> Self {
        Self {
            contrast,
            photon_rate,
            t2_star_s,
        }
    }

    /// Typical single NV diamond parameters.
    pub fn single_nv() -> Self {
        Self::new(0.30, 1e5, 1.0e-6)
    }

    /// Typical NV ensemble parameters.
    pub fn ensemble() -> Self {
        Self::new(0.03, 1e10, 1.0e-6)
    }

    /// Sensitivity in T/sqrt(Hz) for DC magnetometry.
    pub fn sensitivity_t_per_sqrt_hz(&self, config: &NvCenterConfig) -> f64 {
        let denom = config.gamma_e_hz_per_t
            * self.contrast
            * (self.photon_rate * self.t2_star_s).sqrt();
        if denom.abs() < 1e-30 {
            return f64::INFINITY;
        }
        1.0 / denom
    }

    /// Sensitivity in T/sqrt(Hz) using Ramsey interrogation.
    /// eta = (h / (g_e * mu_B)) * 1 / (C * sqrt(N * T2*))
    /// Simplified: eta = 1 / (2*pi * gamma_e * C * sqrt(R * T2*))
    pub fn ramsey_sensitivity_t_per_sqrt_hz(&self, config: &NvCenterConfig) -> f64 {
        let denom = 2.0
            * PI
            * config.gamma_e_hz_per_t
            * self.contrast
            * (self.photon_rate * self.t2_star_s).sqrt();
        if denom.abs() < 1e-30 {
            return f64::INFINITY;
        }
        1.0 / denom
    }

    /// Sensitivity for AC magnetometry with dynamical decoupling.
    /// Enhanced by sqrt(T2_DD / T2*) factor.
    pub fn ac_sensitivity_t_per_sqrt_hz(
        &self,
        config: &NvCenterConfig,
        t2_dd_s: f64,
    ) -> f64 {
        let dc_sens = self.sensitivity_t_per_sqrt_hz(config);
        dc_sens * (self.t2_star_s / t2_dd_s).sqrt()
    }

    /// Convert T/sqrt(Hz) to fT/sqrt(Hz) (femto-Tesla).
    pub fn to_ft_per_sqrt_hz(eta_t: f64) -> f64 {
        eta_t * 1e15
    }

    /// Convert T/sqrt(Hz) to pT/sqrt(Hz) (pico-Tesla).
    pub fn to_pt_per_sqrt_hz(eta_t: f64) -> f64 {
        eta_t * 1e12
    }

    /// Convert T/sqrt(Hz) to nT/sqrt(Hz) (nano-Tesla).
    pub fn to_nt_per_sqrt_hz(eta_t: f64) -> f64 {
        eta_t * 1e9
    }
}

// ---------------------------------------------------------------------------
// TemperatureCompensator
// ---------------------------------------------------------------------------

/// Temperature compensation for NV zero-field splitting drift.
///
/// The zero-field splitting has a temperature dependence:
///     D(T) = D_0 + dD/dT * (T - T_0)
///
/// where dD/dT ~ -74 kHz/K near room temperature (T_0 = 300 K).
#[derive(Debug, Clone)]
pub struct TemperatureCompensator {
    /// Reference zero-field splitting (Hz).
    pub d0_hz: f64,
    /// Reference temperature (K).
    pub t0_k: f64,
    /// Temperature coefficient (Hz/K), typically ~ -74 kHz/K.
    pub dd_dt_hz_per_k: f64,
}

impl Default for TemperatureCompensator {
    fn default() -> Self {
        Self {
            d0_hz: D_ZFS_HZ,
            t0_k: T_REF_K,
            dd_dt_hz_per_k: DD_DT_HZ_PER_K,
        }
    }
}

impl TemperatureCompensator {
    /// Create with custom parameters.
    pub fn new(d0_hz: f64, t0_k: f64, dd_dt_hz_per_k: f64) -> Self {
        Self {
            d0_hz,
            t0_k,
            dd_dt_hz_per_k,
        }
    }

    /// Compute D(T) at a given temperature (K).
    pub fn d_at_temperature(&self, temp_k: f64) -> f64 {
        self.d0_hz + self.dd_dt_hz_per_k * (temp_k - self.t0_k)
    }

    /// Compute the temperature shift in Hz for a given delta-T.
    pub fn frequency_shift(&self, delta_t_k: f64) -> f64 {
        self.dd_dt_hz_per_k * delta_t_k
    }

    /// Correct an observed D frequency to the reference temperature.
    pub fn correct_to_reference(&self, d_observed_hz: f64, current_temp_k: f64) -> f64 {
        d_observed_hz - self.dd_dt_hz_per_k * (current_temp_k - self.t0_k)
    }

    /// Estimate temperature from observed D frequency.
    pub fn estimate_temperature(&self, d_observed_hz: f64) -> f64 {
        if self.dd_dt_hz_per_k.abs() < 1e-30 {
            return self.t0_k;
        }
        self.t0_k + (d_observed_hz - self.d0_hz) / self.dd_dt_hz_per_k
    }

    /// Correct ODMR splitting for temperature.
    ///
    /// Given measured splitting and temperature, returns the field-only splitting
    /// by removing the temperature-dependent D shift.
    pub fn correct_splitting(
        &self,
        measured_splitting_hz: f64,
        _current_temp_k: f64,
    ) -> f64 {
        // The splitting is f+ - f- = 2*gamma_e*B, independent of D.
        // Temperature shifts D, which shifts both f+ and f- equally,
        // so the splitting itself is temperature-independent.
        measured_splitting_hz
    }
}

// ---------------------------------------------------------------------------
// Noise Spectrum Analyzer (relaxometry)
// ---------------------------------------------------------------------------

/// Magnetic noise spectral density analyser based on NV relaxometry.
///
/// NV relaxation rates are related to the magnetic noise power spectral
/// density at specific frequencies:
///
///     1/T1 = 3*gamma_e^2 * S_B(f = D)
///     1/T2 = 1/(2*T1) + gamma_e^2 * S_B(f ~ 0) + ...
///
/// By measuring T1 at different magnetic biases (shifting D), one can
/// map out the noise spectrum S_B(f).
#[derive(Debug, Clone)]
pub struct NoiseSpectrumAnalyzer {
    /// Measured (frequency_Hz, T1_s) data points.
    data_points: Vec<(f64, f64)>,
    /// Gyromagnetic ratio (Hz/T).
    gamma_e: f64,
}

impl NoiseSpectrumAnalyzer {
    /// Create a new analyser.
    pub fn new(gamma_e: f64) -> Self {
        Self {
            data_points: Vec::new(),
            gamma_e,
        }
    }

    /// Create with default NV parameters.
    pub fn default_nv() -> Self {
        Self::new(GAMMA_E_HZ_PER_T)
    }

    /// Add a relaxation measurement at a given probe frequency.
    pub fn add_measurement(&mut self, probe_freq_hz: f64, t1_s: f64) {
        self.data_points.push((probe_freq_hz, t1_s));
    }

    /// Compute the magnetic noise spectral density at each measured frequency.
    ///
    /// S_B(f) = 1 / (3 * gamma_e^2 * T1) in units of T^2/Hz.
    pub fn compute_spectrum(&self) -> Vec<(f64, f64)> {
        let factor = 3.0 * self.gamma_e * self.gamma_e;
        self.data_points
            .iter()
            .map(|&(f, t1)| {
                let s_b = if t1 > 0.0 && factor > 0.0 {
                    1.0 / (factor * t1)
                } else {
                    0.0
                };
                (f, s_b)
            })
            .collect()
    }

    /// Compute the noise spectral density at a single frequency from T1.
    pub fn noise_psd_from_t1(t1_s: f64, gamma_e: f64) -> f64 {
        if t1_s <= 0.0 {
            return 0.0;
        }
        1.0 / (3.0 * gamma_e * gamma_e * t1_s)
    }

    /// Compute T1 from a known noise spectral density.
    pub fn t1_from_noise_psd(s_b_t2_per_hz: f64, gamma_e: f64) -> f64 {
        if s_b_t2_per_hz <= 0.0 {
            return f64::INFINITY;
        }
        1.0 / (3.0 * gamma_e * gamma_e * s_b_t2_per_hz)
    }

    /// Fit a 1/f^alpha noise model to the measured spectrum.
    /// Returns (amplitude, alpha) where S(f) = A / f^alpha.
    pub fn fit_power_law(&self) -> Option<(f64, f64)> {
        let spectrum = self.compute_spectrum();
        if spectrum.len() < 2 {
            return None;
        }

        // Linear regression in log-log space: log(S) = log(A) - alpha * log(f)
        let mut sum_x = 0.0;
        let mut sum_y = 0.0;
        let mut sum_xx = 0.0;
        let mut sum_xy = 0.0;
        let mut count = 0.0;

        for &(f, s) in &spectrum {
            if f <= 0.0 || s <= 0.0 {
                continue;
            }
            let x = f.ln();
            let y = s.ln();
            sum_x += x;
            sum_y += y;
            sum_xx += x * x;
            sum_xy += x * y;
            count += 1.0;
        }

        if count < 2.0 {
            return None;
        }

        let denom = count * sum_xx - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return None;
        }

        let slope = (count * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / count;

        let alpha = -slope; // S(f) = A / f^alpha => log(S) = log(A) - alpha*log(f)
        let amplitude = intercept.exp();

        Some((amplitude, alpha))
    }

    /// Number of data points.
    pub fn num_points(&self) -> usize {
        self.data_points.len()
    }

    /// Clear all measurements.
    pub fn clear(&mut self) {
        self.data_points.clear();
    }
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Compute the dot product of two 3D vectors.
pub fn dot3(a: &[f64; 3], b: &[f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

/// Compute the magnitude of a 3D vector.
pub fn mag3(v: &[f64; 3]) -> f64 {
    dot3(v, v).sqrt()
}

/// Normalise a 3D vector in-place. Returns the original magnitude.
pub fn normalise3(v: &mut [f64; 3]) -> f64 {
    let m = mag3(v);
    if m > 1e-15 {
        v[0] /= m;
        v[1] /= m;
        v[2] /= m;
    }
    m
}

/// Convert magnetic field in Tesla to Gauss (1 T = 10_000 G).
pub fn tesla_to_gauss(tesla: f64) -> f64 {
    tesla * 1e4
}

/// Convert magnetic field in Gauss to Tesla.
pub fn gauss_to_tesla(gauss: f64) -> f64 {
    gauss * 1e-4
}

/// Convert ODMR splitting (Hz) to field (Tesla).
pub fn splitting_to_field(splitting_hz: f64) -> f64 {
    splitting_hz / (2.0 * GAMMA_E_HZ_PER_T)
}

/// Convert field (Tesla) to ODMR splitting (Hz).
pub fn field_to_splitting(field_tesla: f64) -> f64 {
    2.0 * GAMMA_E_HZ_PER_T * field_tesla.abs()
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-6;
    const TOL_PERCENT: f64 = 0.05; // 5% relative tolerance

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    fn relative_eq(a: f64, b: f64, rel_tol: f64) -> bool {
        if b.abs() < 1e-30 {
            a.abs() < 1e-20
        } else {
            ((a - b) / b).abs() < rel_tol
        }
    }

    // --- Physical constants ---

    #[test]
    fn test_physical_constants() {
        assert!(approx_eq(D_ZFS_HZ, 2.87e9, 1e6));
        assert!(approx_eq(GAMMA_E_HZ_PER_T, 28.025e9, 1e6));
        assert!(approx_eq(DD_DT_HZ_PER_K, -74.0e3, 1.0));
        assert!(approx_eq(T_REF_K, 300.0, 0.1));
    }

    // --- NV orientations ---

    #[test]
    fn test_nv_orientations_unit_vectors() {
        for orient in &NV_ORIENTATIONS {
            let len = mag3(orient);
            assert!(
                approx_eq(len, 1.0, 1e-12),
                "NV orientation should be unit vector, got len = {}",
                len
            );
        }
    }

    #[test]
    fn test_nv_orientations_tetrahedral_symmetry() {
        // All pairs should have the same absolute dot product (tetrahedral: -1/3)
        for i in 0..4 {
            for j in (i + 1)..4 {
                let d = dot3(&NV_ORIENTATIONS[i], &NV_ORIENTATIONS[j]);
                assert!(
                    approx_eq(d.abs(), 1.0 / 3.0, 1e-10),
                    "NV pair ({},{}) dot = {}, expected +-1/3",
                    i,
                    j,
                    d
                );
            }
        }
    }

    // --- NvCenterConfig ---

    #[test]
    fn test_nv_config_default() {
        let c = NvCenterConfig::default();
        assert!(approx_eq(c.d_zfs_hz, D_ZFS_HZ, 1.0));
        assert!(approx_eq(c.gamma_e_hz_per_t, GAMMA_E_HZ_PER_T, 1.0));
        assert!(approx_eq(mag3(&c.axis), 1.0, 1e-12));
    }

    #[test]
    fn test_nv_config_with_axis_normalisation() {
        let c = NvCenterConfig::with_axis([3.0, 4.0, 0.0]);
        assert!(approx_eq(mag3(&c.axis), 1.0, 1e-12));
        assert!(approx_eq(c.axis[0], 0.6, 1e-10));
        assert!(approx_eq(c.axis[1], 0.8, 1e-10));
    }

    #[test]
    fn test_nv_config_project_field() {
        let c = NvCenterConfig::with_axis([1.0, 0.0, 0.0]);
        let b = [0.001, 0.002, 0.003]; // 1 mT along x
        let proj = c.project_field(&b);
        assert!(approx_eq(proj, 0.001, 1e-10));
    }

    #[test]
    fn test_nv_config_odmr_frequencies() {
        let c = NvCenterConfig::default();
        let b_nv = 1e-3; // 1 mT
        let (f_minus, f_plus) = c.odmr_frequencies(b_nv);
        let expected_shift = GAMMA_E_HZ_PER_T * 1e-3;
        assert!(approx_eq(f_minus, D_ZFS_HZ - expected_shift, 1e3));
        assert!(approx_eq(f_plus, D_ZFS_HZ + expected_shift, 1e3));
        assert!(f_plus > f_minus);
    }

    #[test]
    fn test_nv_config_odmr_zero_field() {
        let c = NvCenterConfig::default();
        let (f_minus, f_plus) = c.odmr_frequencies(0.0);
        assert!(approx_eq(f_minus, D_ZFS_HZ, 1.0));
        assert!(approx_eq(f_plus, D_ZFS_HZ, 1.0));
    }

    // --- Lorentzian dip ---

    #[test]
    fn test_lorentzian_dip_at_center() {
        let dip = LorentzianDip {
            center_hz: 2.87e9,
            fwhm_hz: 10.0e6,
            depth: 0.10,
        };
        let val = dip.evaluate(2.87e9);
        assert!(approx_eq(val, 0.90, 1e-10), "At centre: 1 - depth = 0.9");
    }

    #[test]
    fn test_lorentzian_dip_far_from_center() {
        let dip = LorentzianDip {
            center_hz: 2.87e9,
            fwhm_hz: 10.0e6,
            depth: 0.10,
        };
        let val = dip.evaluate(2.87e9 + 1e9);
        assert!(val > 0.999, "Far from centre should be close to 1.0, got {}", val);
    }

    #[test]
    fn test_lorentzian_dip_at_half_max() {
        let dip = LorentzianDip {
            center_hz: 2.87e9,
            fwhm_hz: 10.0e6,
            depth: 0.10,
        };
        // At f = center +/- FWHM/2, the Lorentzian drops to 0.5
        let val = dip.evaluate(2.87e9 + 5.0e6);
        // Expected: 1 - 0.1 * 0.5 = 0.95
        assert!(approx_eq(val, 0.95, 1e-6));
    }

    // --- ODMR Processor ---

    #[test]
    fn test_odmr_generate_spectrum() {
        let proc = OdmrProcessor::new(2.85e9, 2.89e9, 1000, 10.0e6);
        let dips = vec![
            LorentzianDip {
                center_hz: 2.842e9,
                fwhm_hz: 10.0e6,
                depth: 0.05,
            },
            LorentzianDip {
                center_hz: 2.898e9,
                fwhm_hz: 10.0e6,
                depth: 0.05,
            },
        ];
        let spectrum = proc.generate_spectrum(&dips);
        assert_eq!(spectrum.len(), 1000);
        // All values should be in (0, 1]
        for &v in &spectrum {
            assert!(v > 0.0 && v <= 1.0, "Spectrum value {} out of range", v);
        }
    }

    #[test]
    fn test_odmr_find_dips() {
        let proc = OdmrProcessor::new(2.80e9, 2.94e9, 2000, 10.0e6);
        let dips = vec![
            LorentzianDip {
                center_hz: 2.842e9,
                fwhm_hz: 10.0e6,
                depth: 0.10,
            },
            LorentzianDip {
                center_hz: 2.898e9,
                fwhm_hz: 10.0e6,
                depth: 0.10,
            },
        ];
        let spectrum = proc.generate_spectrum(&dips);
        let found = proc.find_dips(&spectrum);
        assert_eq!(found.len(), 2, "Should find exactly 2 dips");
        // Check centres are close to expected
        assert!(
            relative_eq(found[0].center_hz, 2.842e9, 0.01),
            "First dip centre: {} vs expected {}",
            found[0].center_hz,
            2.842e9
        );
        assert!(
            relative_eq(found[1].center_hz, 2.898e9, 0.01),
            "Second dip centre: {} vs expected {}",
            found[1].center_hz,
            2.898e9
        );
    }

    #[test]
    fn test_odmr_extract_splitting() {
        let proc = OdmrProcessor::new(2.80e9, 2.94e9, 2000, 10.0e6);
        let b_nv = 1e-3; // 1 mT
        let expected_splitting = 2.0 * GAMMA_E_HZ_PER_T * b_nv;
        let f_minus = D_ZFS_HZ - GAMMA_E_HZ_PER_T * b_nv;
        let f_plus = D_ZFS_HZ + GAMMA_E_HZ_PER_T * b_nv;
        let dips = vec![
            LorentzianDip {
                center_hz: f_minus,
                fwhm_hz: 10.0e6,
                depth: 0.10,
            },
            LorentzianDip {
                center_hz: f_plus,
                fwhm_hz: 10.0e6,
                depth: 0.10,
            },
        ];
        let spectrum = proc.generate_spectrum(&dips);
        let splitting = proc.extract_splitting(&spectrum).unwrap();
        assert!(
            relative_eq(splitting, expected_splitting, 0.02),
            "Splitting: {} vs expected {}",
            splitting,
            expected_splitting
        );
    }

    // --- Magnetic Field Extractor ---

    #[test]
    fn test_field_extractor_from_splitting() {
        let config = NvCenterConfig::default();
        let ext = MagneticFieldExtractor::new(&config);

        // 100 MHz splitting -> B = 100e6 / (2 * 28.025e9) ~ 1.784 mT
        let b = ext.from_splitting_hz(100.0e6);
        let expected = 100.0e6 / (2.0 * GAMMA_E_HZ_PER_T);
        assert!(approx_eq(b, expected, 1e-10));
    }

    #[test]
    fn test_field_extractor_from_dip_frequencies() {
        let config = NvCenterConfig::default();
        let ext = MagneticFieldExtractor::new(&config);
        let b = ext.from_dip_frequencies(2.842e9, 2.898e9);
        let expected = (2.898e9 - 2.842e9) / (2.0 * GAMMA_E_HZ_PER_T);
        assert!(approx_eq(b, expected, 1e-10));
    }

    #[test]
    fn test_field_extractor_roundtrip() {
        let config = NvCenterConfig::default();
        let ext = MagneticFieldExtractor::new(&config);
        let b_original = 5e-3; // 5 mT
        let splitting = ext.expected_splitting_hz(b_original);
        let b_recovered = ext.from_splitting_hz(splitting);
        assert!(approx_eq(b_recovered, b_original, 1e-12));
    }

    // --- Ramsey Interferometry ---

    #[test]
    fn test_ramsey_signal_at_zero() {
        let r = RamseyInterferometry::new(1.0, 1e6, 1e-6);
        let p = r.signal(0.0);
        // At tau=0: P = (1 + C*cos(0)*exp(0))/2 = (1+1)/2 = 1.0
        assert!(approx_eq(p, 1.0, 1e-10));
    }

    #[test]
    fn test_ramsey_signal_oscillation() {
        let r = RamseyInterferometry::new(1.0, 1e6, 1e-3); // long T2* so decay is negligible
        // At tau = 1/(2*detuning), cos(pi) = -1
        let tau_half = 0.5 / 1e6;
        let p = r.signal(tau_half);
        assert!(approx_eq(p, 0.0, 0.01), "P at half period = {}", p);
    }

    #[test]
    fn test_ramsey_generate_curve() {
        let r = RamseyInterferometry::new(1.0, 1e6, 1e-6);
        let taus: Vec<f64> = (0..100).map(|i| i as f64 * 1e-8).collect();
        let curve = r.generate_curve(&taus);
        assert_eq!(curve.len(), 100);
        // All values between 0 and 1
        for &v in &curve {
            assert!(v >= -0.01 && v <= 1.01, "Ramsey value {} out of range", v);
        }
    }

    #[test]
    fn test_ramsey_fit_t2_star() {
        let t2_star = 2.0e-6;
        let r = RamseyInterferometry::new(0.8, 5.0e6, t2_star);
        let taus: Vec<f64> = (0..200).map(|i| i as f64 * 5e-8).collect();
        let measurements = r.generate_curve(&taus);
        let fitted = r.fit_t2_star(&taus, &measurements);
        assert!(fitted.is_some(), "Should be able to fit T2*");
        let t2_fit = fitted.unwrap();
        assert!(
            relative_eq(t2_fit, t2_star, 0.3),
            "Fitted T2*: {:.3e} vs actual {:.3e}",
            t2_fit,
            t2_star
        );
    }

    // --- Spin Echo ---

    #[test]
    fn test_spin_echo_at_zero() {
        let se = SpinEchoProcessor::new(1.0, 1e-3, 1.0);
        let amp = se.echo_amplitude(0.0);
        assert!(approx_eq(amp, 1.0, 1e-10));
    }

    #[test]
    fn test_spin_echo_exponential_decay() {
        let t2 = 1e-3;
        let se = SpinEchoProcessor::new(1.0, t2, 1.0);
        // At t = T2, amplitude = exp(-1)
        let amp = se.echo_amplitude(t2);
        assert!(approx_eq(amp, (-1.0_f64).exp(), 1e-10));
    }

    #[test]
    fn test_spin_echo_gaussian_decay() {
        let t2 = 1e-3;
        let se = SpinEchoProcessor::new(1.0, t2, 2.0);
        // At t = T2, amplitude = exp(-1)
        let amp = se.echo_amplitude(t2);
        assert!(approx_eq(amp, (-1.0_f64).exp(), 1e-10));
    }

    #[test]
    fn test_spin_echo_fit_t2() {
        let t2 = 5e-4;
        let se = SpinEchoProcessor::new(1.0, t2, 1.0);
        let times: Vec<f64> = (1..50).map(|i| i as f64 * 2e-5).collect();
        let amps = se.generate_decay(&times);
        let fitted = se.fit_t2(&times, &amps);
        assert!(fitted.is_some());
        let t2_fit = fitted.unwrap();
        assert!(
            relative_eq(t2_fit, t2, 0.05),
            "Fitted T2: {:.3e} vs actual {:.3e}",
            t2_fit,
            t2
        );
    }

    #[test]
    fn test_spin_echo_presets() {
        let single = SpinEchoProcessor::single_nv();
        assert!(approx_eq(single.t2_s, 1e-3, 1e-6));
        let ensemble = SpinEchoProcessor::ensemble();
        assert!(approx_eq(ensemble.t2_s, 10e-6, 1e-9));
    }

    // --- Dynamical Decoupling ---

    #[test]
    fn test_dd_tau_spacing() {
        let dd = DynamicalDecoupling::new(DdSequence::Cpmg, 100, 1e-3);
        assert!(approx_eq(dd.tau_s(), 1e-5, 1e-15));
    }

    #[test]
    fn test_dd_ac_frequency() {
        let dd = DynamicalDecoupling::new(DdSequence::Cpmg, 100, 1e-3);
        // f_ac = N / (2*T) = 100 / (2 * 1e-3) = 50 kHz
        assert!(approx_eq(dd.ac_frequency_hz(), 50e3, 1.0));
    }

    #[test]
    fn test_dd_block_size() {
        assert_eq!(
            DynamicalDecoupling::new(DdSequence::Cpmg, 1, 1.0).block_size(),
            1
        );
        assert_eq!(
            DynamicalDecoupling::new(DdSequence::Xy4, 4, 1.0).block_size(),
            4
        );
        assert_eq!(
            DynamicalDecoupling::new(DdSequence::Xy8, 8, 1.0).block_size(),
            8
        );
        assert_eq!(
            DynamicalDecoupling::new(DdSequence::Xy16, 16, 1.0).block_size(),
            16
        );
    }

    #[test]
    fn test_dd_pulse_phases() {
        let dd = DynamicalDecoupling::new(DdSequence::Xy8, 8, 1.0);
        let phases = dd.pulse_phases();
        assert_eq!(phases.len(), 8);
    }

    #[test]
    fn test_dd_xy16_phases() {
        let dd = DynamicalDecoupling::new(DdSequence::Xy16, 16, 1.0);
        let phases = dd.pulse_phases();
        assert_eq!(phases.len(), 16);
    }

    // --- AC Magnetometer ---

    #[test]
    fn test_ac_mag_pure_sine() {
        let f_ref = 1000.0; // 1 kHz
        let fs = 100_000.0; // 100 kHz sample rate
        let n = 10_000; // 100 ms of data
        let amplitude = 0.5;

        let signal: Vec<f64> = (0..n)
            .map(|i| amplitude * (2.0 * PI * f_ref * i as f64 / fs).cos())
            .collect();

        let (amp, _phase) = AcMagnetometer::demodulate(&signal, f_ref, fs);
        assert!(
            relative_eq(amp, amplitude, 0.05),
            "Demodulated amplitude: {} vs expected {}",
            amp,
            amplitude
        );
    }

    #[test]
    fn test_ac_mag_reset() {
        let mut mag = AcMagnetometer::new(1000.0, 1.0);
        let signal = vec![0.1; 100];
        let _ = mag.process(&signal, 10000.0);
        assert!(mag.sample_count > 0);
        mag.reset();
        assert_eq!(mag.sample_count, 0);
        assert!(approx_eq(mag.i_accum, 0.0, 1e-15));
    }

    // --- Vector Magnetometer ---

    #[test]
    fn test_vector_mag_roundtrip() {
        let vm = VectorMagnetometer::new();
        let b = [1e-4, 2e-4, 3e-4]; // 100/200/300 uT
        let proj = vm.project(&b);
        let b_recon = vm.reconstruct(&proj);
        for i in 0..3 {
            assert!(
                approx_eq(b_recon[i], b[i], 1e-12),
                "Component {} mismatch: {} vs {}",
                i,
                b_recon[i],
                b[i]
            );
        }
    }

    #[test]
    fn test_vector_mag_field_magnitude() {
        let b = [3e-4, 4e-4, 0.0];
        let mag = VectorMagnetometer::field_magnitude(&b);
        assert!(approx_eq(mag, 5e-4, 1e-12));
    }

    #[test]
    fn test_vector_mag_z_field_only() {
        let vm = VectorMagnetometer::new();
        let b = [0.0, 0.0, 50e-6]; // 50 uT along z (Earth's field)
        let proj = vm.project(&b);
        let b_recon = vm.reconstruct(&proj);
        assert!(approx_eq(b_recon[0], 0.0, 1e-12));
        assert!(approx_eq(b_recon[1], 0.0, 1e-12));
        assert!(approx_eq(b_recon[2], 50e-6, 1e-12));
    }

    #[test]
    fn test_vector_mag_default() {
        let vm = VectorMagnetometer::default();
        let b = [1e-3, 0.0, 0.0];
        let proj = vm.project(&b);
        let b_recon = vm.reconstruct(&proj);
        assert!(approx_eq(b_recon[0], 1e-3, 1e-12));
    }

    // --- Sensitivity Calculator ---

    #[test]
    fn test_sensitivity_positive() {
        let config = NvCenterConfig::default();
        let calc = SensitivityCalculator::new(0.03, 1e6, 1e-6);
        let eta = calc.sensitivity_t_per_sqrt_hz(&config);
        assert!(eta > 0.0);
        assert!(eta.is_finite());
    }

    #[test]
    fn test_sensitivity_typical_ensemble() {
        let config = NvCenterConfig::default();
        let calc = SensitivityCalculator::ensemble();
        let eta = calc.sensitivity_t_per_sqrt_hz(&config);
        let eta_nt = SensitivityCalculator::to_nt_per_sqrt_hz(eta);
        // Typical ensemble: 1-100 nT/sqrt(Hz)
        assert!(
            eta_nt > 0.001 && eta_nt < 1000.0,
            "Ensemble sensitivity: {} nT/sqrt(Hz)",
            eta_nt
        );
    }

    #[test]
    fn test_sensitivity_ac_better_than_dc() {
        let config = NvCenterConfig::default();
        let calc = SensitivityCalculator::new(0.03, 1e6, 1e-6);
        let dc = calc.sensitivity_t_per_sqrt_hz(&config);
        let ac = calc.ac_sensitivity_t_per_sqrt_hz(&config, 1e-3);
        // AC with T2_DD >> T2* should be much better (smaller eta)
        assert!(ac < dc, "AC sensitivity {} should be better than DC {}", ac, dc);
    }

    #[test]
    fn test_sensitivity_unit_conversions() {
        let eta = 1e-12; // 1 pT/sqrt(Hz)
        assert!(approx_eq(SensitivityCalculator::to_pt_per_sqrt_hz(eta), 1.0, 1e-6));
        assert!(approx_eq(SensitivityCalculator::to_nt_per_sqrt_hz(eta), 1e-3, 1e-9));
        assert!(approx_eq(SensitivityCalculator::to_ft_per_sqrt_hz(eta), 1e3, 1e-3));
    }

    #[test]
    fn test_sensitivity_ramsey() {
        let config = NvCenterConfig::default();
        let calc = SensitivityCalculator::new(0.1, 1e8, 5e-6);
        let eta = calc.ramsey_sensitivity_t_per_sqrt_hz(&config);
        assert!(eta > 0.0 && eta.is_finite());
    }

    // --- Temperature Compensator ---

    #[test]
    fn test_temp_comp_at_reference() {
        let tc = TemperatureCompensator::default();
        let d = tc.d_at_temperature(300.0);
        assert!(approx_eq(d, D_ZFS_HZ, 1.0));
    }

    #[test]
    fn test_temp_comp_above_reference() {
        let tc = TemperatureCompensator::default();
        // +10 K -> D shifts by -740 kHz
        let d = tc.d_at_temperature(310.0);
        assert!(approx_eq(d, D_ZFS_HZ - 740e3, 1.0));
    }

    #[test]
    fn test_temp_comp_correct_to_reference() {
        let tc = TemperatureCompensator::default();
        let d_observed = D_ZFS_HZ - 740e3; // at 310 K
        let d_corrected = tc.correct_to_reference(d_observed, 310.0);
        assert!(approx_eq(d_corrected, D_ZFS_HZ, 1.0));
    }

    #[test]
    fn test_temp_comp_estimate_temperature() {
        let tc = TemperatureCompensator::default();
        let d_observed = D_ZFS_HZ - 1.48e6; // 20 K above ref
        let t_est = tc.estimate_temperature(d_observed);
        assert!(approx_eq(t_est, 320.0, 0.1));
    }

    #[test]
    fn test_temp_comp_splitting_independence() {
        let tc = TemperatureCompensator::default();
        let splitting = 56.05e6; // from 1 mT field
        // Splitting is independent of temperature
        let corrected = tc.correct_splitting(splitting, 350.0);
        assert!(approx_eq(corrected, splitting, 1e-6));
    }

    // --- Noise Spectrum Analyzer ---

    #[test]
    fn test_noise_spectrum_from_t1() {
        let s = NoiseSpectrumAnalyzer::noise_psd_from_t1(1e-3, GAMMA_E_HZ_PER_T);
        assert!(s > 0.0 && s.is_finite());
    }

    #[test]
    fn test_noise_spectrum_t1_roundtrip() {
        let t1 = 5e-3;
        let s = NoiseSpectrumAnalyzer::noise_psd_from_t1(t1, GAMMA_E_HZ_PER_T);
        let t1_back = NoiseSpectrumAnalyzer::t1_from_noise_psd(s, GAMMA_E_HZ_PER_T);
        assert!(approx_eq(t1_back, t1, 1e-15));
    }

    #[test]
    fn test_noise_spectrum_compute() {
        let mut nsa = NoiseSpectrumAnalyzer::default_nv();
        nsa.add_measurement(1e6, 1e-3);
        nsa.add_measurement(2e6, 2e-3);
        nsa.add_measurement(5e6, 5e-3);
        let spectrum = nsa.compute_spectrum();
        assert_eq!(spectrum.len(), 3);
        // Higher T1 -> lower noise
        assert!(spectrum[0].1 > spectrum[2].1);
    }

    #[test]
    fn test_noise_spectrum_fit_power_law() {
        let mut nsa = NoiseSpectrumAnalyzer::default_nv();
        // Create 1/f noise: S(f) = A/f, so T1(f) = 1/(3*gamma^2*A/f) = f/(3*gamma^2*A)
        let a_noise = 1e-20;
        let factor = 3.0 * GAMMA_E_HZ_PER_T * GAMMA_E_HZ_PER_T;
        for i in 1..20 {
            let f = i as f64 * 1e6;
            let t1 = f / (factor * a_noise);
            nsa.add_measurement(f, t1);
        }
        let result = nsa.fit_power_law();
        assert!(result.is_some());
        let (amplitude, alpha) = result.unwrap();
        // Should find alpha ~ 1 for 1/f noise
        assert!(
            relative_eq(alpha, 1.0, 0.1),
            "Alpha: {} (expected ~1.0)",
            alpha
        );
        assert!(amplitude > 0.0);
    }

    #[test]
    fn test_noise_spectrum_clear() {
        let mut nsa = NoiseSpectrumAnalyzer::default_nv();
        nsa.add_measurement(1e6, 1e-3);
        assert_eq!(nsa.num_points(), 1);
        nsa.clear();
        assert_eq!(nsa.num_points(), 0);
    }

    // --- Utility functions ---

    #[test]
    fn test_splitting_to_field_roundtrip() {
        let b = 2.5e-3; // 2.5 mT
        let splitting = field_to_splitting(b);
        let b_back = splitting_to_field(splitting);
        assert!(approx_eq(b_back, b, 1e-15));
    }

    #[test]
    fn test_tesla_gauss_conversion() {
        assert!(approx_eq(tesla_to_gauss(1.0), 10000.0, 0.01));
        assert!(approx_eq(gauss_to_tesla(10000.0), 1.0, 1e-10));
    }

    #[test]
    fn test_dot3_orthogonal() {
        let a = [1.0, 0.0, 0.0];
        let b = [0.0, 1.0, 0.0];
        assert!(approx_eq(dot3(&a, &b), 0.0, 1e-15));
    }

    #[test]
    fn test_mag3() {
        let v = [3.0, 4.0, 0.0];
        assert!(approx_eq(mag3(&v), 5.0, 1e-12));
    }

    #[test]
    fn test_normalise3() {
        let mut v = [3.0, 4.0, 0.0];
        let orig_mag = normalise3(&mut v);
        assert!(approx_eq(orig_mag, 5.0, 1e-12));
        assert!(approx_eq(mag3(&v), 1.0, 1e-12));
    }

    // --- Integration: full ODMR-to-field pipeline ---

    #[test]
    fn test_full_odmr_pipeline() {
        let config = NvCenterConfig::default();
        let b_true = 2e-3; // 2 mT

        // 1. Generate expected ODMR frequencies
        let (f_minus, f_plus) = config.odmr_frequencies(b_true);

        // 2. Create and run ODMR processor
        let proc = OdmrProcessor::new(
            f_minus - 50e6,
            f_plus + 50e6,
            5000,
            10e6,
        );
        let dips = vec![
            LorentzianDip {
                center_hz: f_minus,
                fwhm_hz: 10e6,
                depth: 0.08,
            },
            LorentzianDip {
                center_hz: f_plus,
                fwhm_hz: 10e6,
                depth: 0.08,
            },
        ];
        let spectrum = proc.generate_spectrum(&dips);

        // 3. Extract splitting
        let splitting = proc.extract_splitting(&spectrum).unwrap();

        // 4. Convert to field
        let ext = MagneticFieldExtractor::new(&config);
        let b_meas = ext.from_splitting_hz(splitting);

        // Should be within 5% of true value
        assert!(
            relative_eq(b_meas, b_true, TOL_PERCENT),
            "Measured B: {:.6e} T vs true {:.6e} T",
            b_meas,
            b_true
        );
    }

    #[test]
    fn test_vector_field_reconstruction_pipeline() {
        let vm = VectorMagnetometer::new();
        let configs: Vec<NvCenterConfig> = (0..4).map(|i| NvCenterConfig::orientation(i)).collect();

        // True field: 50 uT in arbitrary direction
        let b_true = [30e-6, -20e-6, 40e-6];

        // Simulate per-axis projections (what each NV orientation measures)
        let mut projections = [0.0f64; 4];
        for i in 0..4 {
            projections[i] = configs[i].project_field(&b_true);
        }

        // Reconstruct
        let b_recon = vm.reconstruct(&projections);
        for i in 0..3 {
            assert!(
                approx_eq(b_recon[i], b_true[i], 1e-12),
                "Component {}: {:.6e} vs {:.6e}",
                i,
                b_recon[i],
                b_true[i]
            );
        }

        // Check magnitude
        let mag_true = VectorMagnetometer::field_magnitude(&b_true);
        let mag_recon = VectorMagnetometer::field_magnitude(&b_recon);
        assert!(approx_eq(mag_recon, mag_true, 1e-12));
    }
}
