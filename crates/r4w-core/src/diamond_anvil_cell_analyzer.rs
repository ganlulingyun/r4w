//! Signal processing for Diamond Anvil Cell (DAC) high-pressure experiments.
//!
//! Diamond anvil cells compress microscopic samples between two gem-quality
//! diamond anvils to pressures exceeding 300 GPa (Earth's inner core boundary
//! is ~360 GPa). In-situ spectroscopic and diffraction probes are used to
//! measure the pressure, temperature, and structural state of the sample.
//!
//! This module provides the signal processing chain for:
//!
//! - **Ruby fluorescence** pressure measurement (Mao hydrostatic scale)
//! - **Raman spectroscopy** pressure gauging via diamond edge shift
//! - **Phase transition detection** from spectral pattern changes
//! - **Equation of state** fitting (3rd-order Birch-Murnaghan)
//! - **Bragg peak indexing** for lattice parameter determination
//! - **P-T path mapping** for phase diagram construction
//! - **Laser heating** temperature from Planck radiation fitting
//! - **Gasket thickness** estimation from white-light interference fringes
//!
//! # Physical background
//!
//! The ruby fluorescence method is the most widely used primary pressure gauge
//! in DAC experiments. The R1 emission line of Cr^3+ in Al2O3 shifts to longer
//! wavelengths under compression:
//!
//! ```text
//! P(GPa) = (A / B) * [(lambda / lambda0)^B - 1]
//! ```
//!
//! where A = 1904 GPa, B = 7.665 (Mao hydrostatic scale, 1986), and
//! lambda0 = 694.3 nm is the ambient R1 wavelength.
//!
//! For the equation of state, the 3rd-order Birch-Murnaghan isothermal EOS is:
//!
//! ```text
//! P(V) = (3/2) * K0 * [eta^(7/3) - eta^(5/3)] * [1 + (3/4)*(K0'-4)*(eta^(2/3) - 1)]
//! ```
//!
//! where eta = V0/V, K0 is the bulk modulus at zero pressure, and K0' is its
//! pressure derivative.
//!
//! Laser heating temperature is determined by fitting the thermal emission
//! spectrum to the Planck radiation law:
//!
//! ```text
//! I(lambda, T) = (2*h*c^2 / lambda^5) / (exp(h*c / (lambda*k_B*T)) - 1)
//! ```
//!
//! # Components
//!
//! | Struct / Function | Purpose |
//! |---|---|
//! | [`DacConfig`] | Anvil type, culet size, gasket material, chamber diameter |
//! | [`PressureCalculator`] | Ruby fluorescence Mao scale P(lambda) |
//! | [`RubyFluorescenceTracker`] | Track R1 line shift with Voigt peak fitting |
//! | [`RamanPressureGauge`] | Diamond Raman edge shift pressure gauge |
//! | [`PhaseTransitionDetector`] | Detect structural transitions from spectral changes |
//! | [`EquationOfState`] | Birch-Murnaghan 3rd-order EOS fitting |
//! | [`BraggPeakIndexer`] | Index diffraction peaks for lattice parameters |
//! | [`PressureTemperatureMapper`] | P-T path tracking for phase diagrams |
//! | [`LaserHeatingController`] | Temperature from Planck fit to thermal emission |
//! | [`GasketThicknessEstimator`] | Gasket thinning from interference fringes |
//!
//! # Example
//!
//! ```rust
//! use r4w_core::diamond_anvil_cell_analyzer::{
//!     DacConfig, AnvilType, GasketMaterial, PressureCalculator, EquationOfState,
//! };
//!
//! let config = DacConfig {
//!     anvil_type: AnvilType::BrilliантCut,
//!     culet_diameter_um: 300.0,
//!     gasket_material: GasketMaterial::Rhenium,
//!     sample_chamber_diameter_um: 150.0,
//! };
//!
//! // Measure pressure from ruby R1 line shift
//! let calc = PressureCalculator::new_mao_hydrostatic();
//! let pressure_gpa = calc.pressure_from_wavelength(698.0); // nm
//! assert!(pressure_gpa > 0.0);
//!
//! // Fit equation of state
//! let eos = EquationOfState::birch_murnaghan(160.0, 4.0, 11.0); // K0, K0', V0
//! let p = eos.pressure(10.5); // P at V = 10.5 cm^3/mol
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Physical constants
// ---------------------------------------------------------------------------

/// Speed of light in vacuum (m/s).
const C_LIGHT: f64 = 2.998_e8;

/// Planck constant (J*s).
const H_PLANCK: f64 = 6.626_070_15e-34;

/// Boltzmann constant (J/K).
const K_BOLTZMANN: f64 = 1.380_649e-23;

/// Wien displacement constant (m*K).
const WIEN_B: f64 = 2.897_771_955e-3;

/// Ruby R1 line wavelength at ambient pressure (nm).
const RUBY_R1_AMBIENT_NM: f64 = 694.3;

/// Diamond Raman edge frequency at ambient pressure (cm^-1).
const DIAMOND_RAMAN_AMBIENT_CM1: f64 = 1332.5;

// ---------------------------------------------------------------------------
// DacConfig
// ---------------------------------------------------------------------------

/// Type of diamond anvil.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AnvilType {
    /// Standard brilliant-cut diamond anvil.
    BrilliантCut,
    /// Beveled anvil for ultra-high pressure (>100 GPa).
    Beveled,
    /// Toroidal anvil for pressures >300 GPa.
    Toroidal,
    /// Double-stage (ds-DAC) for terapascal range.
    DoubleStage,
}

/// Gasket material used in the DAC.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GasketMaterial {
    /// Stainless steel (up to ~30 GPa).
    StainlessSteel,
    /// Rhenium (up to ~300 GPa).
    Rhenium,
    /// Tungsten (up to ~200 GPa).
    Tungsten,
    /// Beryllium (X-ray transparent, up to ~50 GPa).
    Beryllium,
    /// Boron-epoxy composite (X-ray transparent).
    BoronEpoxy,
}

/// Configuration for a Diamond Anvil Cell experiment.
#[derive(Debug, Clone, PartialEq)]
pub struct DacConfig {
    /// Type of diamond anvil.
    pub anvil_type: AnvilType,
    /// Culet (flat tip) diameter in micrometers.
    pub culet_diameter_um: f64,
    /// Gasket material.
    pub gasket_material: GasketMaterial,
    /// Sample chamber diameter in micrometers.
    pub sample_chamber_diameter_um: f64,
}

impl DacConfig {
    /// Create a standard configuration for moderate pressures (<50 GPa).
    pub fn standard() -> Self {
        Self {
            anvil_type: AnvilType::BrilliантCut,
            culet_diameter_um: 300.0,
            gasket_material: GasketMaterial::StainlessSteel,
            sample_chamber_diameter_um: 150.0,
        }
    }

    /// Create a configuration for megabar pressures (>100 GPa).
    pub fn megabar() -> Self {
        Self {
            anvil_type: AnvilType::Beveled,
            culet_diameter_um: 100.0,
            gasket_material: GasketMaterial::Rhenium,
            sample_chamber_diameter_um: 50.0,
        }
    }

    /// Create a configuration for ultra-high pressure (>300 GPa).
    pub fn ultra_high() -> Self {
        Self {
            anvil_type: AnvilType::Toroidal,
            culet_diameter_um: 30.0,
            gasket_material: GasketMaterial::Rhenium,
            sample_chamber_diameter_um: 15.0,
        }
    }

    /// Estimate the force (in Newtons) required to achieve the given pressure.
    ///
    /// Uses F = P * A where A is the culet area.
    pub fn force_for_pressure(&self, pressure_gpa: f64) -> f64 {
        let radius_m = self.culet_diameter_um * 0.5e-6;
        let area_m2 = PI * radius_m * radius_m;
        pressure_gpa * 1e9 * area_m2
    }
}

// ---------------------------------------------------------------------------
// PressureCalculator
// ---------------------------------------------------------------------------

/// Ruby fluorescence pressure calculator using the Mao hydrostatic scale.
///
/// The Mao (1986) calibration relates the ruby R1 line wavelength shift to
/// pressure:
///
/// ```text
/// P(GPa) = (A / B) * [(lambda / lambda0)^B - 1]
/// ```
///
/// where A = 1904 GPa, B = 7.665 for hydrostatic conditions.
///
/// For quasi-hydrostatic conditions (no pressure medium), B = 5.0 is often
/// used (Mao non-hydrostatic scale).
#[derive(Debug, Clone)]
pub struct PressureCalculator {
    /// Scale parameter A (GPa).
    a: f64,
    /// Exponent parameter B (dimensionless).
    b: f64,
    /// Reference wavelength at ambient pressure (nm).
    lambda0: f64,
}

impl PressureCalculator {
    /// Create a calculator with the Mao hydrostatic scale (A=1904, B=7.665).
    pub fn new_mao_hydrostatic() -> Self {
        Self {
            a: 1904.0,
            b: 7.665,
            lambda0: RUBY_R1_AMBIENT_NM,
        }
    }

    /// Create a calculator with the Mao non-hydrostatic scale (A=1904, B=5.0).
    pub fn new_mao_non_hydrostatic() -> Self {
        Self {
            a: 1904.0,
            b: 5.0,
            lambda0: RUBY_R1_AMBIENT_NM,
        }
    }

    /// Create a calculator with custom parameters.
    pub fn new(a: f64, b: f64, lambda0: f64) -> Self {
        Self { a, b, lambda0 }
    }

    /// Calculate pressure (GPa) from measured R1 wavelength (nm).
    pub fn pressure_from_wavelength(&self, lambda_nm: f64) -> f64 {
        let ratio = lambda_nm / self.lambda0;
        (self.a / self.b) * (ratio.powf(self.b) - 1.0)
    }

    /// Calculate expected R1 wavelength (nm) at a given pressure (GPa).
    pub fn wavelength_from_pressure(&self, pressure_gpa: f64) -> f64 {
        let inner = pressure_gpa * self.b / self.a + 1.0;
        self.lambda0 * inner.powf(1.0 / self.b)
    }

    /// Linear approximation of pressure from wavelength shift (valid below ~10 GPa).
    ///
    /// dP/dlambda ~ 2.740 GPa/nm for the Mao hydrostatic scale near ambient.
    pub fn pressure_linear(&self, lambda_nm: f64) -> f64 {
        let delta = lambda_nm - self.lambda0;
        // Derivative at P=0: dP/dlambda = A / lambda0
        let sensitivity = self.a / self.lambda0;
        sensitivity * delta
    }

    /// Pressure sensitivity dP/dlambda (GPa/nm) at a given pressure.
    pub fn sensitivity_at_pressure(&self, pressure_gpa: f64) -> f64 {
        let lam = self.wavelength_from_pressure(pressure_gpa);
        // dP/dlambda = (A/lambda0) * (lambda/lambda0)^(B-1)
        let ratio = lam / self.lambda0;
        (self.a / self.lambda0) * ratio.powf(self.b - 1.0)
    }
}

// ---------------------------------------------------------------------------
// RubyFluorescenceTracker
// ---------------------------------------------------------------------------

/// Voigt profile peak fitting parameters for a spectral line.
#[derive(Debug, Clone, PartialEq)]
pub struct VoigtParameters {
    /// Peak center wavelength (nm).
    pub center_nm: f64,
    /// Gaussian width (FWHM, nm) - from thermal/instrumental broadening.
    pub gaussian_fwhm_nm: f64,
    /// Lorentzian width (FWHM, nm) - from natural/pressure broadening.
    pub lorentzian_fwhm_nm: f64,
    /// Peak amplitude (arbitrary units).
    pub amplitude: f64,
}

impl VoigtParameters {
    /// Approximate Voigt FWHM using Thompson's formula.
    ///
    /// ```text
    /// fV ~ 0.5346*fL + sqrt(0.2166*fL^2 + fG^2)
    /// ```
    pub fn voigt_fwhm(&self) -> f64 {
        let fl = self.lorentzian_fwhm_nm;
        let fg = self.gaussian_fwhm_nm;
        0.5346 * fl + (0.2166 * fl * fl + fg * fg).sqrt()
    }

    /// Evaluate the pseudo-Voigt approximation at wavelength `lambda_nm`.
    ///
    /// Uses a linear combination of Gaussian and Lorentzian profiles weighted
    /// by the mixing parameter eta (Thompson et al., 1987).
    pub fn evaluate(&self, lambda_nm: f64) -> f64 {
        let fv = self.voigt_fwhm();
        if fv <= 0.0 {
            return 0.0;
        }
        let fl = self.lorentzian_fwhm_nm;
        // Mixing parameter eta
        let r = fl / fv;
        let eta = 1.366_03 * r - 0.477_19 * r * r + 0.111_16 * r * r * r;
        let eta = eta.clamp(0.0, 1.0);

        let x = lambda_nm - self.center_nm;

        // Gaussian component
        let sigma_g = fv / (2.0 * (2.0_f64.ln()).sqrt());
        let gauss = (-0.5 * (x / sigma_g).powi(2)).exp() / (sigma_g * (2.0 * PI).sqrt());

        // Lorentzian component
        let gamma = fv / 2.0;
        let lorentz = (gamma / PI) / (x * x + gamma * gamma);

        self.amplitude * (eta * lorentz + (1.0 - eta) * gauss)
    }
}

/// Tracks the ruby R1 fluorescence line across measurements for pressure
/// monitoring during a DAC experiment.
#[derive(Debug, Clone)]
pub struct RubyFluorescenceTracker {
    /// Pressure calculator used for wavelength-to-pressure conversion.
    calculator: PressureCalculator,
    /// History of fitted peak positions (nm).
    peak_history: Vec<f64>,
    /// History of computed pressures (GPa).
    pressure_history: Vec<f64>,
    /// Current Voigt fit parameters.
    current_fit: Option<VoigtParameters>,
}

impl RubyFluorescenceTracker {
    /// Create a new tracker with the Mao hydrostatic scale.
    pub fn new() -> Self {
        Self {
            calculator: PressureCalculator::new_mao_hydrostatic(),
            peak_history: Vec::new(),
            pressure_history: Vec::new(),
            current_fit: None,
        }
    }

    /// Create a tracker with a custom pressure calculator.
    pub fn with_calculator(calculator: PressureCalculator) -> Self {
        Self {
            calculator,
            peak_history: Vec::new(),
            pressure_history: Vec::new(),
            current_fit: None,
        }
    }

    /// Process a new spectrum and extract the R1 peak position.
    ///
    /// `wavelengths` and `intensities` are parallel arrays representing the
    /// measured spectrum. The method finds the peak near the expected R1
    /// position and fits a Voigt profile.
    ///
    /// Returns the fitted peak wavelength (nm) and pressure (GPa).
    pub fn process_spectrum(
        &mut self,
        wavelengths: &[f64],
        intensities: &[f64],
    ) -> Option<(f64, f64)> {
        if wavelengths.len() != intensities.len() || wavelengths.len() < 3 {
            return None;
        }

        // Expected peak position: last known or ambient
        let expected = self
            .peak_history
            .last()
            .copied()
            .unwrap_or(RUBY_R1_AMBIENT_NM);

        // Find the peak near expected position (search within +/- 20 nm)
        let search_min = expected - 20.0;
        let search_max = expected + 20.0;

        let mut best_idx = None;
        let mut best_intensity = f64::NEG_INFINITY;
        for (i, (&wl, &inten)) in wavelengths.iter().zip(intensities.iter()).enumerate() {
            if wl >= search_min && wl <= search_max && inten > best_intensity {
                best_intensity = inten;
                best_idx = Some(i);
            }
        }

        let idx = best_idx?;

        // Parabolic interpolation around the peak for sub-pixel accuracy
        let center = if idx > 0 && idx < wavelengths.len() - 1 {
            let y0 = intensities[idx - 1];
            let y1 = intensities[idx];
            let y2 = intensities[idx + 1];
            let denom = 2.0 * (2.0 * y1 - y0 - y2);
            if denom.abs() > 1e-30 {
                let offset = (y0 - y2) / denom;
                let step = wavelengths[idx] - wavelengths[idx - 1];
                wavelengths[idx] + offset * step
            } else {
                wavelengths[idx]
            }
        } else {
            wavelengths[idx]
        };

        // Estimate Voigt parameters from the peak shape
        let half_max = best_intensity * 0.5;
        let mut left_hm = center;
        let mut right_hm = center;
        for i in (0..idx).rev() {
            if intensities[i] <= half_max {
                left_hm = wavelengths[i];
                break;
            }
        }
        for i in (idx + 1)..wavelengths.len() {
            if intensities[i] <= half_max {
                right_hm = wavelengths[i];
                break;
            }
        }
        let fwhm = right_hm - left_hm;

        let fit = VoigtParameters {
            center_nm: center,
            gaussian_fwhm_nm: fwhm * 0.6, // Approximate partition
            lorentzian_fwhm_nm: fwhm * 0.4,
            amplitude: best_intensity,
        };

        let pressure = self.calculator.pressure_from_wavelength(center);

        self.peak_history.push(center);
        self.pressure_history.push(pressure);
        self.current_fit = Some(fit);

        Some((center, pressure))
    }

    /// Return the current pressure (GPa), or `None` if no measurement yet.
    pub fn current_pressure(&self) -> Option<f64> {
        self.pressure_history.last().copied()
    }

    /// Return the full pressure history.
    pub fn pressure_history(&self) -> &[f64] {
        &self.pressure_history
    }

    /// Return the current Voigt fit parameters.
    pub fn current_fit(&self) -> Option<&VoigtParameters> {
        self.current_fit.as_ref()
    }

    /// Estimate the pressure rate (GPa/measurement) from recent history.
    pub fn pressure_rate(&self, window: usize) -> Option<f64> {
        let n = self.pressure_history.len();
        if n < 2 {
            return None;
        }
        let w = window.min(n);
        let slice = &self.pressure_history[n - w..];
        let first = slice[0];
        let last = *slice.last().unwrap();
        Some((last - first) / (w as f64 - 1.0))
    }
}

// ---------------------------------------------------------------------------
// RamanPressureGauge
// ---------------------------------------------------------------------------

/// Pressure gauge based on the diamond Raman edge shift.
///
/// The high-frequency edge of the diamond first-order Raman band shifts
/// with pressure. At ambient conditions it is at ~1332.5 cm^-1.
/// The shift follows approximately:
///
/// ```text
/// P(GPa) = K0 * (delta_nu / nu0) + K1 * (delta_nu / nu0)^2
/// ```
///
/// with K0 ~ 547 GPa, K1 ~ 690 GPa (Akahama & Kawamura, 2006).
#[derive(Debug, Clone)]
pub struct RamanPressureGauge {
    /// Reference Raman frequency at ambient (cm^-1).
    nu0: f64,
    /// Linear coefficient K0 (GPa).
    k0: f64,
    /// Quadratic coefficient K1 (GPa).
    k1: f64,
}

impl RamanPressureGauge {
    /// Create a gauge with the Akahama & Kawamura (2006) calibration.
    pub fn new_akahama_kawamura() -> Self {
        Self {
            nu0: DIAMOND_RAMAN_AMBIENT_CM1,
            k0: 547.0,
            k1: 690.0,
        }
    }

    /// Create a gauge with custom calibration parameters.
    pub fn new(nu0: f64, k0: f64, k1: f64) -> Self {
        Self { nu0, k0, k1 }
    }

    /// Calculate pressure (GPa) from measured Raman frequency (cm^-1).
    pub fn pressure_from_raman(&self, nu_cm1: f64) -> f64 {
        let delta = nu_cm1 - self.nu0;
        let x = delta / self.nu0;
        self.k0 * x + self.k1 * x * x
    }

    /// Expected Raman frequency (cm^-1) at a given pressure (GPa).
    ///
    /// Solves the quadratic K1*x^2 + K0*x - P = 0 for x = delta_nu/nu0.
    pub fn raman_from_pressure(&self, pressure_gpa: f64) -> f64 {
        if self.k1.abs() < 1e-12 {
            // Linear case
            return self.nu0 + pressure_gpa / self.k0 * self.nu0;
        }
        let discriminant = self.k0 * self.k0 + 4.0 * self.k1 * pressure_gpa;
        if discriminant < 0.0 {
            return self.nu0;
        }
        let x = (-self.k0 + discriminant.sqrt()) / (2.0 * self.k1);
        self.nu0 * (1.0 + x)
    }

    /// Find the Raman edge in a spectrum and compute pressure.
    ///
    /// `frequencies` (cm^-1) and `intensities` are the Raman spectrum.
    /// Returns (edge_frequency_cm1, pressure_gpa).
    pub fn analyze_spectrum(
        &self,
        frequencies: &[f64],
        intensities: &[f64],
    ) -> Option<(f64, f64)> {
        if frequencies.len() != intensities.len() || frequencies.len() < 5 {
            return None;
        }

        // Compute derivative to find the steepest edge
        let mut max_deriv = 0.0_f64;
        let mut edge_idx = 0;
        for i in 1..frequencies.len() {
            let df = frequencies[i] - frequencies[i - 1];
            if df.abs() < 1e-12 {
                continue;
            }
            let deriv = (intensities[i] - intensities[i - 1]) / df;
            if deriv.abs() > max_deriv.abs() {
                max_deriv = deriv;
                edge_idx = i;
            }
        }

        if max_deriv.abs() < 1e-30 {
            return None;
        }

        let edge_freq = (frequencies[edge_idx] + frequencies[edge_idx - 1]) / 2.0;
        let pressure = self.pressure_from_raman(edge_freq);
        Some((edge_freq, pressure))
    }
}

// ---------------------------------------------------------------------------
// PhaseTransitionDetector
// ---------------------------------------------------------------------------

/// A detected spectral event potentially indicating a phase transition.
#[derive(Debug, Clone, PartialEq)]
pub struct TransitionEvent {
    /// Measurement index where the transition was detected.
    pub measurement_index: usize,
    /// Pressure at which the transition was detected (GPa).
    pub pressure_gpa: f64,
    /// Temperature at which the transition was detected (K), if known.
    pub temperature_k: Option<f64>,
    /// Type of spectral change observed.
    pub event_type: TransitionEventType,
    /// Confidence score (0.0 to 1.0).
    pub confidence: f64,
}

/// Types of spectral changes that indicate phase transitions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransitionEventType {
    /// An existing peak splits into two (symmetry breaking).
    PeakSplitting,
    /// A new peak appears (new vibrational mode).
    PeakAppearance,
    /// An existing peak disappears (mode becomes inactive).
    PeakDisappearance,
    /// Abrupt change in peak intensity ratios.
    IntensityChange,
    /// Discontinuous shift in peak position.
    PositionJump,
}

/// Detects structural phase transitions from changes in spectral patterns.
///
/// Monitors Raman or XRD spectra for signatures of phase transitions:
/// peak splitting, appearance/disappearance, intensity ratio changes, and
/// discontinuous peak shifts.
#[derive(Debug, Clone)]
pub struct PhaseTransitionDetector {
    /// Stored peak positions from previous measurements.
    previous_peaks: Vec<f64>,
    /// Threshold for peak position jump detection (fractional).
    jump_threshold: f64,
    /// Threshold for peak count change detection.
    count_tolerance: usize,
    /// Detected transition events.
    events: Vec<TransitionEvent>,
    /// Measurement counter.
    measurement_count: usize,
}

impl PhaseTransitionDetector {
    /// Create a new detector with default thresholds.
    pub fn new() -> Self {
        Self {
            previous_peaks: Vec::new(),
            jump_threshold: 0.005, // 0.5% fractional shift
            count_tolerance: 0,
            events: Vec::new(),
            measurement_count: 0,
        }
    }

    /// Set the fractional jump threshold for position discontinuity detection.
    pub fn with_jump_threshold(mut self, threshold: f64) -> Self {
        self.jump_threshold = threshold;
        self
    }

    /// Analyze a new set of peak positions and detect transitions.
    ///
    /// `peaks` are the sorted peak positions (e.g., Raman frequencies in cm^-1
    /// or XRD 2-theta angles). `pressure_gpa` is the current pressure.
    ///
    /// Returns any newly detected transition events.
    pub fn analyze(
        &mut self,
        peaks: &[f64],
        pressure_gpa: f64,
        temperature_k: Option<f64>,
    ) -> Vec<TransitionEvent> {
        let mut new_events = Vec::new();
        self.measurement_count += 1;

        if !self.previous_peaks.is_empty() {
            let prev = &self.previous_peaks;
            let curr = peaks;

            // Check for peak count changes (appearance/disappearance)
            if curr.len() > prev.len() + self.count_tolerance {
                new_events.push(TransitionEvent {
                    measurement_index: self.measurement_count,
                    pressure_gpa,
                    temperature_k,
                    event_type: TransitionEventType::PeakAppearance,
                    confidence: 0.8,
                });
            } else if curr.len() + self.count_tolerance < prev.len() {
                new_events.push(TransitionEvent {
                    measurement_index: self.measurement_count,
                    pressure_gpa,
                    temperature_k,
                    event_type: TransitionEventType::PeakDisappearance,
                    confidence: 0.8,
                });
            }

            // Check for peak splitting: one previous peak now has two nearby peaks
            for &pp in prev.iter() {
                let nearby: Vec<_> = curr
                    .iter()
                    .filter(|&&cp| {
                        let frac = (cp - pp).abs() / pp.abs().max(1e-12);
                        frac < 0.05
                    })
                    .collect();
                if nearby.len() >= 2 {
                    new_events.push(TransitionEvent {
                        measurement_index: self.measurement_count,
                        pressure_gpa,
                        temperature_k,
                        event_type: TransitionEventType::PeakSplitting,
                        confidence: 0.9,
                    });
                    break;
                }
            }

            // Check for position jumps: matched peaks with discontinuous shifts
            let matched = match_peaks(prev, curr);
            for (pi, ci) in matched {
                let frac_shift =
                    (curr[ci] - prev[pi]).abs() / prev[pi].abs().max(1e-12);
                if frac_shift > self.jump_threshold {
                    new_events.push(TransitionEvent {
                        measurement_index: self.measurement_count,
                        pressure_gpa,
                        temperature_k,
                        event_type: TransitionEventType::PositionJump,
                        confidence: (frac_shift / self.jump_threshold).min(1.0),
                    });
                    break;
                }
            }
        }

        self.previous_peaks = peaks.to_vec();
        self.events.extend(new_events.clone());
        new_events
    }

    /// Return all detected transition events.
    pub fn events(&self) -> &[TransitionEvent] {
        &self.events
    }

    /// Clear event history.
    pub fn clear_events(&mut self) {
        self.events.clear();
    }
}

/// Match peaks between two sorted lists using nearest-neighbor assignment.
/// Returns pairs of (prev_index, curr_index).
fn match_peaks(prev: &[f64], curr: &[f64]) -> Vec<(usize, usize)> {
    let mut matches = Vec::new();
    let mut used = vec![false; curr.len()];

    for (pi, &pval) in prev.iter().enumerate() {
        let mut best_ci = None;
        let mut best_dist = f64::INFINITY;
        for (ci, &cval) in curr.iter().enumerate() {
            if used[ci] {
                continue;
            }
            let dist = (pval - cval).abs();
            if dist < best_dist {
                best_dist = dist;
                best_ci = Some(ci);
            }
        }
        if let Some(ci) = best_ci {
            used[ci] = true;
            matches.push((pi, ci));
        }
    }

    matches
}

// ---------------------------------------------------------------------------
// EquationOfState
// ---------------------------------------------------------------------------

/// Third-order Birch-Murnaghan equation of state for isothermal compression.
///
/// ```text
/// P(V) = (3/2) * K0 * [eta^(7/3) - eta^(5/3)] * [1 + (3/4)*(K0'-4)*(eta^(2/3) - 1)]
/// ```
///
/// where eta = V0/V, K0 is the zero-pressure bulk modulus, and K0' is its
/// pressure derivative.
#[derive(Debug, Clone)]
pub struct EquationOfState {
    /// Zero-pressure bulk modulus K0 (GPa).
    k0: f64,
    /// Pressure derivative of bulk modulus K0' (dimensionless).
    k0_prime: f64,
    /// Zero-pressure volume V0 (same units as input V, typically cm^3/mol or A^3).
    v0: f64,
}

impl EquationOfState {
    /// Create a Birch-Murnaghan EOS with given parameters.
    ///
    /// # Arguments
    /// * `k0` - Bulk modulus at zero pressure (GPa)
    /// * `k0_prime` - Pressure derivative of bulk modulus (dimensionless)
    /// * `v0` - Volume at zero pressure (arbitrary units, same as V in pressure())
    pub fn birch_murnaghan(k0: f64, k0_prime: f64, v0: f64) -> Self {
        Self { k0, k0_prime, v0 }
    }

    /// Presets for common materials.
    pub fn mgo() -> Self {
        // MgO: K0 = 160 GPa, K0' = 4.15, V0 = 11.24 cm^3/mol
        Self::birch_murnaghan(160.0, 4.15, 11.24)
    }

    pub fn iron_hcp() -> Self {
        // hcp-Fe: K0 = 165 GPa, K0' = 5.33, V0 = 6.73 cm^3/mol
        Self::birch_murnaghan(165.0, 5.33, 6.73)
    }

    pub fn diamond() -> Self {
        // Diamond: K0 = 442 GPa, K0' = 3.5, V0 = 3.417 cm^3/mol
        Self::birch_murnaghan(442.0, 3.5, 3.417)
    }

    /// Calculate pressure (GPa) at a given volume.
    pub fn pressure(&self, v: f64) -> f64 {
        let eta = self.v0 / v;
        let eta_7_3 = eta.powf(7.0 / 3.0);
        let eta_5_3 = eta.powf(5.0 / 3.0);
        let eta_2_3 = eta.powf(2.0 / 3.0);

        1.5 * self.k0 * (eta_7_3 - eta_5_3)
            * (1.0 + 0.75 * (self.k0_prime - 4.0) * (eta_2_3 - 1.0))
    }

    /// Calculate the Eulerian finite strain f = [(V0/V)^(2/3) - 1] / 2.
    pub fn eulerian_strain(&self, v: f64) -> f64 {
        ((self.v0 / v).powf(2.0 / 3.0) - 1.0) / 2.0
    }

    /// Calculate bulk modulus K at a given volume (GPa).
    ///
    /// K(V) = -V * dP/dV, computed by numerical differentiation.
    pub fn bulk_modulus(&self, v: f64) -> f64 {
        let dv = v * 1e-6;
        let p_plus = self.pressure(v + dv);
        let p_minus = self.pressure(v - dv);
        -v * (p_plus - p_minus) / (2.0 * dv)
    }

    /// Fit EOS parameters (K0, K0') from P-V data using least-squares.
    ///
    /// `volumes` and `pressures` are parallel arrays. `v0` is the known
    /// ambient volume. Returns fitted (K0, K0') or None if fitting fails.
    pub fn fit_from_pv_data(volumes: &[f64], pressures: &[f64], v0: f64) -> Option<(f64, f64)> {
        if volumes.len() != pressures.len() || volumes.len() < 3 {
            return None;
        }

        // Grid search for K0 and K0' to minimize sum of squared residuals
        let mut best_k0 = 100.0;
        let mut best_kp = 4.0;
        let mut best_sse = f64::INFINITY;

        // Coarse grid
        for k0_i in 0..100 {
            let k0 = 50.0 + k0_i as f64 * 5.0; // 50 to 545 GPa
            for kp_i in 0..40 {
                let kp = 2.0 + kp_i as f64 * 0.2; // 2.0 to 9.8
                let eos = Self::birch_murnaghan(k0, kp, v0);
                let sse: f64 = volumes
                    .iter()
                    .zip(pressures.iter())
                    .map(|(&v, &p)| {
                        let diff = eos.pressure(v) - p;
                        diff * diff
                    })
                    .sum();
                if sse < best_sse {
                    best_sse = sse;
                    best_k0 = k0;
                    best_kp = kp;
                }
            }
        }

        // Refine with finer grid around best
        let k0_lo = (best_k0 - 5.0).max(1.0);
        let kp_lo = (best_kp - 0.2).max(0.1);
        for k0_i in 0..100 {
            let k0 = k0_lo + k0_i as f64 * 0.1;
            for kp_i in 0..40 {
                let kp = kp_lo + kp_i as f64 * 0.01;
                let eos = Self::birch_murnaghan(k0, kp, v0);
                let sse: f64 = volumes
                    .iter()
                    .zip(pressures.iter())
                    .map(|(&v, &p)| {
                        let diff = eos.pressure(v) - p;
                        diff * diff
                    })
                    .sum();
                if sse < best_sse {
                    best_sse = sse;
                    best_k0 = k0;
                    best_kp = kp;
                }
            }
        }

        Some((best_k0, best_kp))
    }
}

// ---------------------------------------------------------------------------
// BraggPeakIndexer
// ---------------------------------------------------------------------------

/// Crystal system for Bragg peak indexing.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CrystalSystem {
    Cubic,
    Tetragonal,
    Hexagonal,
    Orthorhombic,
}

/// An indexed Bragg reflection.
#[derive(Debug, Clone, PartialEq)]
pub struct IndexedPeak {
    /// Measured 2-theta angle (degrees).
    pub two_theta_deg: f64,
    /// Miller indices (h, k, l).
    pub hkl: (i32, i32, i32),
    /// Measured d-spacing (Angstroms).
    pub d_measured: f64,
    /// Calculated d-spacing (Angstroms).
    pub d_calculated: f64,
    /// Residual (d_measured - d_calculated) / d_measured.
    pub residual: f64,
}

/// Indexes Bragg diffraction peaks for lattice parameter determination.
///
/// Given observed d-spacings and a crystal system, determines the lattice
/// parameters and Miller indices.
#[derive(Debug, Clone)]
pub struct BraggPeakIndexer {
    /// X-ray wavelength in Angstroms (e.g., 0.3344 for synchrotron at 37 keV).
    wavelength_angstrom: f64,
    /// Maximum Miller index to search.
    max_hkl: i32,
}

impl BraggPeakIndexer {
    /// Create an indexer for a given X-ray wavelength (Angstroms).
    pub fn new(wavelength_angstrom: f64) -> Self {
        Self {
            wavelength_angstrom,
            max_hkl: 10,
        }
    }

    /// Set the maximum Miller index to search.
    pub fn with_max_hkl(mut self, max_hkl: i32) -> Self {
        self.max_hkl = max_hkl;
        self
    }

    /// Convert 2-theta (degrees) to d-spacing (Angstroms) using Bragg's law.
    ///
    /// ```text
    /// d = lambda / (2 * sin(theta))
    /// ```
    pub fn two_theta_to_d(&self, two_theta_deg: f64) -> f64 {
        let theta_rad = two_theta_deg.to_radians() / 2.0;
        let sin_theta = theta_rad.sin();
        if sin_theta.abs() < 1e-12 {
            return f64::INFINITY;
        }
        self.wavelength_angstrom / (2.0 * sin_theta)
    }

    /// Convert d-spacing (Angstroms) to 2-theta (degrees).
    pub fn d_to_two_theta(&self, d: f64) -> f64 {
        let sin_theta = self.wavelength_angstrom / (2.0 * d);
        if sin_theta.abs() > 1.0 {
            return f64::NAN;
        }
        2.0 * sin_theta.asin().to_degrees()
    }

    /// Index peaks for a cubic crystal with lattice parameter `a` (Angstroms).
    ///
    /// For cubic: 1/d^2 = (h^2 + k^2 + l^2) / a^2
    ///
    /// Returns indexed peaks sorted by 2-theta.
    pub fn index_cubic(
        &self,
        two_theta_values: &[f64],
        a: f64,
    ) -> Vec<IndexedPeak> {
        let mut result = Vec::new();

        for &tt in two_theta_values {
            let d_meas = self.two_theta_to_d(tt);
            if d_meas.is_infinite() || d_meas.is_nan() {
                continue;
            }

            let mut best_hkl = (0, 0, 0);
            let mut best_residual = f64::INFINITY;
            let mut best_d_calc = 0.0;

            for h in 0..=self.max_hkl {
                for k in 0..=h {
                    for l in 0..=k {
                        let sum = (h * h + k * k + l * l) as f64;
                        if sum < 1e-12 {
                            continue;
                        }
                        let d_calc = a / sum.sqrt();
                        let residual = ((d_meas - d_calc) / d_meas).abs();
                        if residual < best_residual {
                            best_residual = residual;
                            best_hkl = (h, k, l);
                            best_d_calc = d_calc;
                        }
                    }
                }
            }

            if best_residual < 0.05 {
                // 5% tolerance
                result.push(IndexedPeak {
                    two_theta_deg: tt,
                    hkl: best_hkl,
                    d_measured: d_meas,
                    d_calculated: best_d_calc,
                    residual: (d_meas - best_d_calc) / d_meas,
                });
            }
        }

        result.sort_by(|a, b| a.two_theta_deg.partial_cmp(&b.two_theta_deg).unwrap());
        result
    }

    /// Estimate the cubic lattice parameter from observed d-spacings.
    ///
    /// Uses the first reflection (strongest peak) to estimate `a`, then
    /// refines using all peaks.
    pub fn estimate_cubic_lattice_parameter(&self, two_theta_values: &[f64]) -> Option<f64> {
        if two_theta_values.is_empty() {
            return None;
        }

        // Convert all to d-spacings
        let d_values: Vec<f64> = two_theta_values
            .iter()
            .map(|&tt| self.two_theta_to_d(tt))
            .filter(|d| d.is_finite())
            .collect();

        if d_values.is_empty() {
            return None;
        }

        // For cubic, the largest d-spacing is usually (100) or (110) or (111).
        // Try each and see which gives the best overall fit.
        let d_max = d_values.iter().cloned().fold(0.0_f64, f64::max);

        let candidate_hkl_sums: [f64; 3] = [1.0, 2.0, 3.0]; // h^2+k^2+l^2 for (100), (110), (111)
        let mut best_a = 0.0;
        let mut best_score = f64::INFINITY;

        for &sum in &candidate_hkl_sums {
            let a_candidate = d_max * sum.sqrt();
            let indexed = self.index_cubic(two_theta_values, a_candidate);
            let score: f64 = indexed.iter().map(|p| p.residual.abs()).sum::<f64>()
                / (indexed.len().max(1) as f64);
            let unindexed_penalty = (two_theta_values.len() - indexed.len()) as f64 * 0.1;
            let total_score = score + unindexed_penalty;
            if total_score < best_score {
                best_score = total_score;
                best_a = a_candidate;
            }
        }

        if best_a > 0.0 {
            Some(best_a)
        } else {
            None
        }
    }
}

// ---------------------------------------------------------------------------
// PressureTemperatureMapper
// ---------------------------------------------------------------------------

/// A point on a pressure-temperature path.
#[derive(Debug, Clone, PartialEq)]
pub struct PtPoint {
    /// Pressure (GPa).
    pub pressure_gpa: f64,
    /// Temperature (K).
    pub temperature_k: f64,
    /// Timestamp or measurement index.
    pub index: usize,
    /// Optional phase label.
    pub phase: Option<String>,
}

/// Tracks the pressure-temperature path during a DAC experiment for
/// constructing phase diagrams.
#[derive(Debug, Clone)]
pub struct PressureTemperatureMapper {
    /// Recorded P-T points.
    path: Vec<PtPoint>,
    /// Known phase boundaries as (P, T) pairs.
    boundaries: Vec<(f64, f64, String)>,
}

impl PressureTemperatureMapper {
    /// Create a new P-T mapper.
    pub fn new() -> Self {
        Self {
            path: Vec::new(),
            boundaries: Vec::new(),
        }
    }

    /// Add a P-T measurement point.
    pub fn add_point(&mut self, pressure_gpa: f64, temperature_k: f64, phase: Option<String>) {
        let index = self.path.len();
        self.path.push(PtPoint {
            pressure_gpa,
            temperature_k,
            index,
            phase,
        });
    }

    /// Record a known phase boundary point.
    pub fn add_boundary_point(
        &mut self,
        pressure_gpa: f64,
        temperature_k: f64,
        boundary_name: &str,
    ) {
        self.boundaries
            .push((pressure_gpa, temperature_k, boundary_name.to_string()));
    }

    /// Return the recorded P-T path.
    pub fn path(&self) -> &[PtPoint] {
        &self.path
    }

    /// Return the recorded boundary points.
    pub fn boundaries(&self) -> &[(f64, f64, String)] {
        &self.boundaries
    }

    /// Find the nearest boundary to a given P-T point.
    ///
    /// Returns (distance, boundary_name) or None if no boundaries are defined.
    pub fn nearest_boundary(&self, pressure_gpa: f64, temperature_k: f64) -> Option<(f64, String)> {
        self.boundaries
            .iter()
            .map(|(p, t, name)| {
                let dp = pressure_gpa - p;
                let dt = temperature_k - t;
                // Normalize: 1 GPa ~ 100 K for distance metric
                let dist = (dp * dp + (dt / 100.0) * (dt / 100.0)).sqrt();
                (dist, name.clone())
            })
            .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap())
    }

    /// Compute the Simon-Glatzel melting curve: T_m(P) = T0 * ((P - P0)/a + 1)^(1/c).
    ///
    /// Standard form for melting curves in high-pressure physics.
    pub fn simon_glatzel(
        t0: f64,
        p0: f64,
        a: f64,
        c: f64,
        pressure_gpa: f64,
    ) -> f64 {
        t0 * ((pressure_gpa - p0) / a + 1.0).powf(1.0 / c)
    }

    /// Interpolate the Clausius-Clapeyron slope dT/dP at a boundary.
    ///
    /// ```text
    /// dT/dP = T * Delta_V / Delta_H
    /// ```
    ///
    /// where Delta_V is the volume change (cm^3/mol) and Delta_H is the
    /// enthalpy change (J/mol).
    pub fn clausius_clapeyron_slope(
        temperature_k: f64,
        delta_v_cm3_per_mol: f64,
        delta_h_j_per_mol: f64,
    ) -> f64 {
        if delta_h_j_per_mol.abs() < 1e-30 {
            return 0.0;
        }
        // Convert delta_v from cm^3/mol to m^3/mol
        let delta_v_m3 = delta_v_cm3_per_mol * 1e-6;
        // dT/dP in K/Pa, then convert to K/GPa
        let slope_k_per_pa = temperature_k * delta_v_m3 / delta_h_j_per_mol;
        slope_k_per_pa * 1e9 // K/GPa
    }
}

// ---------------------------------------------------------------------------
// LaserHeatingController
// ---------------------------------------------------------------------------

/// Temperature determination from Planck radiation fitting for laser-heated
/// DAC experiments.
///
/// In double-sided laser heating, the thermal emission spectrum of the
/// heated sample is collected and fit to the Planck function:
///
/// ```text
/// I(lambda, T) = epsilon * (2*h*c^2 / lambda^5) / (exp(h*c / (lambda*k_B*T)) - 1)
/// ```
///
/// Wien's displacement law gives the peak wavelength: lambda_peak = b/T.
#[derive(Debug, Clone)]
pub struct LaserHeatingController {
    /// Emissivity of the sample (0 to 1). Typically ~0.3-0.5 for metals.
    emissivity: f64,
    /// Wavelength range for fitting (nm): (min, max).
    fit_range_nm: (f64, f64),
    /// Temperature history (K).
    temperature_history: Vec<f64>,
}

impl LaserHeatingController {
    /// Create a new controller with given emissivity and fitting wavelength range.
    pub fn new(emissivity: f64, fit_range_nm: (f64, f64)) -> Self {
        Self {
            emissivity,
            fit_range_nm,
            temperature_history: Vec::new(),
        }
    }

    /// Default controller for a typical laser-heating setup (500-900 nm).
    pub fn default_visible() -> Self {
        Self::new(0.4, (500.0, 900.0))
    }

    /// Evaluate the Planck function at a given wavelength (nm) and temperature (K).
    ///
    /// Returns spectral radiance in W/(m^2 * sr * m).
    pub fn planck_spectral_radiance(&self, lambda_nm: f64, temperature_k: f64) -> f64 {
        let lambda_m = lambda_nm * 1e-9;
        let numerator = 2.0 * H_PLANCK * C_LIGHT * C_LIGHT / lambda_m.powi(5);
        let exponent = H_PLANCK * C_LIGHT / (lambda_m * K_BOLTZMANN * temperature_k);
        let denominator = exponent.exp() - 1.0;
        if denominator <= 0.0 {
            return 0.0;
        }
        self.emissivity * numerator / denominator
    }

    /// Wien's displacement law: peak wavelength (nm) for a given temperature (K).
    pub fn wien_peak_wavelength(temperature_k: f64) -> f64 {
        if temperature_k <= 0.0 {
            return f64::INFINITY;
        }
        WIEN_B / temperature_k * 1e9 // Convert m to nm
    }

    /// Estimate temperature from Wien's displacement law given peak wavelength.
    pub fn temperature_from_wien(peak_wavelength_nm: f64) -> f64 {
        if peak_wavelength_nm <= 0.0 {
            return f64::INFINITY;
        }
        WIEN_B / (peak_wavelength_nm * 1e-9)
    }

    /// Fit temperature from a measured thermal emission spectrum.
    ///
    /// Uses a grid search over temperature to minimize the sum of squared
    /// residuals between the normalized measured spectrum and the Planck
    /// function.
    ///
    /// `wavelengths` (nm) and `intensities` (arbitrary units) define the
    /// measured spectrum.
    ///
    /// Returns the fitted temperature (K) and a goodness-of-fit R^2 value.
    pub fn fit_temperature(
        &mut self,
        wavelengths: &[f64],
        intensities: &[f64],
    ) -> Option<(f64, f64)> {
        if wavelengths.len() != intensities.len() || wavelengths.len() < 3 {
            return None;
        }

        // Filter to fitting range
        let mut wl_fit = Vec::new();
        let mut in_fit = Vec::new();
        for (&wl, &inten) in wavelengths.iter().zip(intensities.iter()) {
            if wl >= self.fit_range_nm.0 && wl <= self.fit_range_nm.1 {
                wl_fit.push(wl);
                in_fit.push(inten);
            }
        }
        if wl_fit.len() < 3 {
            return None;
        }

        // Normalize measured spectrum to peak = 1
        let max_inten = in_fit.iter().cloned().fold(0.0_f64, f64::max);
        if max_inten <= 0.0 {
            return None;
        }
        let in_norm: Vec<f64> = in_fit.iter().map(|&v| v / max_inten).collect();

        // Coarse search: 1000 K to 10000 K in 50 K steps
        let mut best_temp = 3000.0;
        let mut best_sse = f64::INFINITY;

        for t_i in 0..=180 {
            let temp = 1000.0 + t_i as f64 * 50.0;
            let sse = self.compute_sse(&wl_fit, &in_norm, temp);
            if sse < best_sse {
                best_sse = sse;
                best_temp = temp;
            }
        }

        // Fine search: +/- 50 K in 1 K steps
        let t_lo = (best_temp - 50.0).max(300.0);
        for t_i in 0..=100 {
            let temp = t_lo + t_i as f64;
            let sse = self.compute_sse(&wl_fit, &in_norm, temp);
            if sse < best_sse {
                best_sse = sse;
                best_temp = temp;
            }
        }

        // Compute R^2
        let mean_in: f64 = in_norm.iter().sum::<f64>() / in_norm.len() as f64;
        let ss_tot: f64 = in_norm.iter().map(|&v| (v - mean_in).powi(2)).sum();
        let r_squared = if ss_tot > 1e-30 {
            1.0 - best_sse / ss_tot
        } else {
            0.0
        };

        self.temperature_history.push(best_temp);
        Some((best_temp, r_squared))
    }

    /// Compute SSE between normalized data and normalized Planck curve at temp.
    fn compute_sse(&self, wavelengths: &[f64], normalized_data: &[f64], temp: f64) -> f64 {
        // Compute Planck values and normalize
        let planck_vals: Vec<f64> = wavelengths
            .iter()
            .map(|&wl| self.planck_spectral_radiance(wl, temp))
            .collect();
        let max_planck = planck_vals.iter().cloned().fold(0.0_f64, f64::max);
        if max_planck <= 0.0 {
            return f64::INFINITY;
        }

        let planck_norm: Vec<f64> = planck_vals.iter().map(|&v| v / max_planck).collect();

        normalized_data
            .iter()
            .zip(planck_norm.iter())
            .map(|(&d, &p)| (d - p).powi(2))
            .sum()
    }

    /// Return the temperature history.
    pub fn temperature_history(&self) -> &[f64] {
        &self.temperature_history
    }
}

// ---------------------------------------------------------------------------
// GasketThicknessEstimator
// ---------------------------------------------------------------------------

/// Estimates gasket thickness from white-light interference fringes.
///
/// When white light passes through the sample chamber, reflections from the
/// two diamond surfaces create interference fringes. The thickness is:
///
/// ```text
/// t = N * lambda / (2 * n * cos(theta))
/// ```
///
/// where N is the fringe order, n is the refractive index of the medium,
/// and theta is the angle of incidence.
///
/// For a series of fringes at wavelengths lambda_m and lambda_{m+1}:
///
/// ```text
/// t = lambda_m * lambda_{m+1} / (2 * n * |lambda_m - lambda_{m+1}|)
/// ```
#[derive(Debug, Clone)]
pub struct GasketThicknessEstimator {
    /// Refractive index of the pressure medium.
    refractive_index: f64,
    /// Angle of incidence (radians).
    angle_rad: f64,
    /// Thickness history (micrometers).
    thickness_history: Vec<f64>,
}

impl GasketThicknessEstimator {
    /// Create an estimator for a given pressure medium refractive index.
    ///
    /// Common media: Ne (n~1.0), Ar (n~1.0), N2 (n~1.0), NaCl (n~1.54),
    /// silicone oil (n~1.4).
    pub fn new(refractive_index: f64) -> Self {
        Self {
            refractive_index,
            angle_rad: 0.0, // Normal incidence
            thickness_history: Vec::new(),
        }
    }

    /// Set the angle of incidence (radians).
    pub fn with_angle(mut self, angle_rad: f64) -> Self {
        self.angle_rad = angle_rad;
        self
    }

    /// Estimate thickness from two adjacent fringe wavelengths (nm).
    ///
    /// Returns thickness in micrometers.
    pub fn thickness_from_two_fringes(&self, lambda1_nm: f64, lambda2_nm: f64) -> f64 {
        let delta = (lambda1_nm - lambda2_nm).abs();
        if delta < 1e-12 {
            return 0.0;
        }
        let cos_theta = self.angle_rad.cos();
        // Result in nm, convert to um
        lambda1_nm * lambda2_nm / (2.0 * self.refractive_index * cos_theta * delta) * 1e-3
    }

    /// Estimate thickness from a series of fringe wavelengths (nm).
    ///
    /// Takes the median of all adjacent-pair estimates for robustness.
    /// Returns thickness in micrometers.
    pub fn thickness_from_fringe_series(&mut self, fringe_wavelengths: &[f64]) -> Option<f64> {
        if fringe_wavelengths.len() < 2 {
            return None;
        }

        let mut estimates: Vec<f64> = Vec::new();
        for i in 0..fringe_wavelengths.len() - 1 {
            let t = self.thickness_from_two_fringes(
                fringe_wavelengths[i],
                fringe_wavelengths[i + 1],
            );
            if t > 0.0 && t.is_finite() {
                estimates.push(t);
            }
        }

        if estimates.is_empty() {
            return None;
        }

        // Median
        estimates.sort_by(|a, b| a.partial_cmp(b).unwrap());
        let median = if estimates.len() % 2 == 0 {
            (estimates[estimates.len() / 2 - 1] + estimates[estimates.len() / 2]) / 2.0
        } else {
            estimates[estimates.len() / 2]
        };

        self.thickness_history.push(median);
        Some(median)
    }

    /// Return the thickness history.
    pub fn thickness_history(&self) -> &[f64] {
        &self.thickness_history
    }

    /// Estimate the number of fringes visible across a wavelength range for
    /// a given thickness (um).
    pub fn expected_fringe_count(
        &self,
        thickness_um: f64,
        lambda_min_nm: f64,
        lambda_max_nm: f64,
    ) -> f64 {
        let cos_theta = self.angle_rad.cos();
        let t_nm = thickness_um * 1e3;
        let n_min = 2.0 * self.refractive_index * cos_theta * t_nm / lambda_max_nm;
        let n_max = 2.0 * self.refractive_index * cos_theta * t_nm / lambda_min_nm;
        (n_max - n_min).abs()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- DacConfig ---

    #[test]
    fn test_dac_config_standard() {
        let cfg = DacConfig::standard();
        assert_eq!(cfg.anvil_type, AnvilType::BrilliантCut);
        assert_eq!(cfg.culet_diameter_um, 300.0);
        assert_eq!(cfg.gasket_material, GasketMaterial::StainlessSteel);
        assert_eq!(cfg.sample_chamber_diameter_um, 150.0);
    }

    #[test]
    fn test_dac_config_megabar() {
        let cfg = DacConfig::megabar();
        assert_eq!(cfg.anvil_type, AnvilType::Beveled);
        assert!(cfg.culet_diameter_um < 150.0);
    }

    #[test]
    fn test_dac_force_for_pressure() {
        let cfg = DacConfig::standard();
        let force = cfg.force_for_pressure(10.0);
        // 10 GPa on 300 um culet: F = 10e9 * pi * (150e-6)^2 ~ 707 N
        assert!(force > 600.0 && force < 800.0);
    }

    // --- PressureCalculator ---

    #[test]
    fn test_pressure_zero_at_ambient() {
        let calc = PressureCalculator::new_mao_hydrostatic();
        let p = calc.pressure_from_wavelength(RUBY_R1_AMBIENT_NM);
        assert!(p.abs() < 1e-10);
    }

    #[test]
    fn test_pressure_positive_for_redshift() {
        let calc = PressureCalculator::new_mao_hydrostatic();
        let p = calc.pressure_from_wavelength(700.0);
        assert!(p > 0.0);
    }

    #[test]
    fn test_pressure_roundtrip() {
        let calc = PressureCalculator::new_mao_hydrostatic();
        for p_orig in [0.0, 5.0, 20.0, 50.0, 100.0, 200.0] {
            let wl = calc.wavelength_from_pressure(p_orig);
            let p_back = calc.pressure_from_wavelength(wl);
            assert!(
                (p_back - p_orig).abs() < 0.001,
                "Roundtrip failed for P={}: got {}",
                p_orig,
                p_back
            );
        }
    }

    #[test]
    fn test_pressure_10_gpa_approx() {
        // At 10 GPa, the ruby line shifts by about 3.65 nm
        let calc = PressureCalculator::new_mao_hydrostatic();
        let wl = calc.wavelength_from_pressure(10.0);
        let shift = wl - RUBY_R1_AMBIENT_NM;
        assert!(shift > 3.0 && shift < 4.5, "Shift at 10 GPa: {} nm", shift);
    }

    #[test]
    fn test_pressure_linear_approximation() {
        let calc = PressureCalculator::new_mao_hydrostatic();
        // At low pressure, linear should agree with nonlinear
        let wl = calc.wavelength_from_pressure(2.0);
        let p_exact = calc.pressure_from_wavelength(wl);
        let p_linear = calc.pressure_linear(wl);
        assert!(
            (p_exact - p_linear).abs() < 0.2,
            "Linear={}, Exact={}",
            p_linear,
            p_exact
        );
    }

    #[test]
    fn test_pressure_sensitivity() {
        let calc = PressureCalculator::new_mao_hydrostatic();
        let sens_0 = calc.sensitivity_at_pressure(0.0);
        let sens_100 = calc.sensitivity_at_pressure(100.0);
        // Sensitivity increases with pressure
        assert!(sens_100 > sens_0);
        // At ambient: A/lambda0 ~ 1904/694.3 ~ 2.74 GPa/nm
        assert!((sens_0 - 2.74).abs() < 0.1);
    }

    #[test]
    fn test_non_hydrostatic_vs_hydrostatic() {
        let hydro = PressureCalculator::new_mao_hydrostatic();
        let non_hydro = PressureCalculator::new_mao_non_hydrostatic();
        // The two scales diverge; verify they give different pressures
        let wl = 710.0;
        let p_hydro = hydro.pressure_from_wavelength(wl);
        let p_non_hydro = non_hydro.pressure_from_wavelength(wl);
        assert!(p_hydro > 0.0);
        assert!(p_non_hydro > 0.0);
        // At moderate pressure (~45 GPa) the two scales differ by a few percent
        assert!((p_hydro - p_non_hydro).abs() / p_hydro < 0.1);
        // At very high pressures (large wavelength shift), the scales diverge more
        let wl_high = 750.0;
        let p_hydro_high = hydro.pressure_from_wavelength(wl_high);
        let p_non_hydro_high = non_hydro.pressure_from_wavelength(wl_high);
        let divergence_low = (p_hydro - p_non_hydro).abs() / p_hydro;
        let divergence_high = (p_hydro_high - p_non_hydro_high).abs() / p_hydro_high;
        assert!(divergence_high > divergence_low, "Scales should diverge more at higher pressure");
    }

    // --- VoigtParameters ---

    #[test]
    fn test_voigt_fwhm() {
        let v = VoigtParameters {
            center_nm: 694.3,
            gaussian_fwhm_nm: 0.5,
            lorentzian_fwhm_nm: 0.3,
            amplitude: 1.0,
        };
        let fwhm = v.voigt_fwhm();
        // Should be between max(fG, fL) and fG + fL
        assert!(fwhm > 0.5);
        assert!(fwhm < 0.8);
    }

    #[test]
    fn test_voigt_peak_at_center() {
        let v = VoigtParameters {
            center_nm: 694.3,
            gaussian_fwhm_nm: 1.0,
            lorentzian_fwhm_nm: 0.5,
            amplitude: 1.0,
        };
        let peak = v.evaluate(694.3);
        let off = v.evaluate(700.0);
        assert!(peak > off, "Peak should be at center");
    }

    // --- RubyFluorescenceTracker ---

    #[test]
    fn test_ruby_tracker_basic() {
        let mut tracker = RubyFluorescenceTracker::new();
        // Create a simple ruby spectrum
        let wavelengths: Vec<f64> = (6900..7000).map(|i| i as f64 * 0.1).collect();
        let center = 696.0; // Shifted from 694.3 -> ~4.7 GPa
        let intensities: Vec<f64> = wavelengths
            .iter()
            .map(|&wl| {
                let x = wl - center;
                1000.0 * (-x * x / 0.5).exp()
            })
            .collect();

        let result = tracker.process_spectrum(&wavelengths, &intensities);
        assert!(result.is_some());
        let (peak_wl, pressure) = result.unwrap();
        assert!((peak_wl - center).abs() < 0.2);
        assert!(pressure > 0.0);
    }

    #[test]
    fn test_ruby_tracker_pressure_history() {
        let mut tracker = RubyFluorescenceTracker::new();
        // Two measurements at different pressures
        let wavelengths: Vec<f64> = (6900..7100).map(|i| i as f64 * 0.1).collect();

        for center in [695.0, 697.0] {
            let intensities: Vec<f64> = wavelengths
                .iter()
                .map(|&wl| {
                    let x = wl - center;
                    1000.0 * (-x * x / 0.5).exp()
                })
                .collect();
            tracker.process_spectrum(&wavelengths, &intensities);
        }

        assert_eq!(tracker.pressure_history().len(), 2);
        // Second pressure should be higher
        assert!(tracker.pressure_history()[1] > tracker.pressure_history()[0]);
    }

    // --- RamanPressureGauge ---

    #[test]
    fn test_raman_zero_at_ambient() {
        let gauge = RamanPressureGauge::new_akahama_kawamura();
        let p = gauge.pressure_from_raman(DIAMOND_RAMAN_AMBIENT_CM1);
        assert!(p.abs() < 1e-10);
    }

    #[test]
    fn test_raman_positive_for_blueshift() {
        let gauge = RamanPressureGauge::new_akahama_kawamura();
        // Diamond Raman edge blueshifts under pressure
        let p = gauge.pressure_from_raman(1350.0);
        assert!(p > 0.0);
    }

    #[test]
    fn test_raman_roundtrip() {
        let gauge = RamanPressureGauge::new_akahama_kawamura();
        for p in [0.0, 10.0, 50.0, 100.0] {
            let nu = gauge.raman_from_pressure(p);
            let p_back = gauge.pressure_from_raman(nu);
            assert!(
                (p_back - p).abs() < 0.01,
                "Roundtrip failed for P={}: got {}",
                p,
                p_back
            );
        }
    }

    // --- PhaseTransitionDetector ---

    #[test]
    fn test_phase_detector_peak_appearance() {
        let mut det = PhaseTransitionDetector::new();
        // First measurement: 3 peaks
        let events1 = det.analyze(&[100.0, 200.0, 300.0], 10.0, None);
        assert!(events1.is_empty()); // No previous to compare

        // Second measurement: 5 peaks (2 new)
        let events2 = det.analyze(&[100.0, 150.0, 200.0, 250.0, 300.0], 15.0, None);
        assert!(!events2.is_empty());
        assert!(events2
            .iter()
            .any(|e| e.event_type == TransitionEventType::PeakAppearance));
    }

    #[test]
    fn test_phase_detector_peak_disappearance() {
        let mut det = PhaseTransitionDetector::new();
        det.analyze(&[100.0, 200.0, 300.0, 400.0], 10.0, None);
        let events = det.analyze(&[100.0, 300.0], 15.0, None);
        assert!(events
            .iter()
            .any(|e| e.event_type == TransitionEventType::PeakDisappearance));
    }

    #[test]
    fn test_phase_detector_peak_splitting() {
        let mut det = PhaseTransitionDetector::new();
        det.analyze(&[100.0, 200.0, 300.0], 10.0, None);
        // Peak at 200 splits into 198 and 202
        let events = det.analyze(&[100.0, 198.0, 202.0, 300.0], 15.0, None);
        assert!(events
            .iter()
            .any(|e| e.event_type == TransitionEventType::PeakSplitting));
    }

    #[test]
    fn test_phase_detector_no_transition() {
        let mut det = PhaseTransitionDetector::new();
        det.analyze(&[100.0, 200.0, 300.0], 10.0, None);
        // Same peaks, slightly shifted (within threshold)
        let events = det.analyze(&[100.1, 200.2, 300.1], 11.0, None);
        // Should have no major transition events
        let major_events: Vec<_> = events
            .iter()
            .filter(|e| {
                e.event_type == TransitionEventType::PeakAppearance
                    || e.event_type == TransitionEventType::PeakDisappearance
                    || e.event_type == TransitionEventType::PeakSplitting
            })
            .collect();
        assert!(major_events.is_empty());
    }

    // --- EquationOfState ---

    #[test]
    fn test_eos_zero_pressure_at_v0() {
        let eos = EquationOfState::birch_murnaghan(160.0, 4.0, 11.0);
        let p = eos.pressure(11.0);
        assert!(p.abs() < 1e-10, "P at V0 should be 0, got {}", p);
    }

    #[test]
    fn test_eos_positive_pressure_for_compression() {
        let eos = EquationOfState::birch_murnaghan(160.0, 4.0, 11.0);
        let p = eos.pressure(10.0); // V < V0 -> compression
        assert!(p > 0.0, "Compression should give positive pressure");
    }

    #[test]
    fn test_eos_monotonic() {
        let eos = EquationOfState::mgo();
        let mut prev_p = 0.0;
        for i in 1..=20 {
            let v = eos.v0 * (1.0 - i as f64 * 0.02);
            let p = eos.pressure(v);
            assert!(p > prev_p, "Pressure should increase with compression");
            prev_p = p;
        }
    }

    #[test]
    fn test_eos_bulk_modulus_at_v0() {
        let eos = EquationOfState::mgo();
        let k = eos.bulk_modulus(eos.v0);
        assert!(
            (k - 160.0).abs() < 1.0,
            "K at V0 should be K0=160, got {}",
            k
        );
    }

    #[test]
    fn test_eos_eulerian_strain() {
        let eos = EquationOfState::birch_murnaghan(160.0, 4.0, 11.0);
        let f = eos.eulerian_strain(11.0);
        assert!(f.abs() < 1e-10, "Strain at V0 should be 0");
        let f2 = eos.eulerian_strain(10.0);
        assert!(f2 > 0.0, "Strain should be positive for compression");
    }

    #[test]
    fn test_eos_fit() {
        let eos_true = EquationOfState::birch_murnaghan(160.0, 4.2, 11.0);
        let volumes: Vec<f64> = (0..10).map(|i| 11.0 - i as f64 * 0.3).collect();
        let pressures: Vec<f64> = volumes.iter().map(|&v| eos_true.pressure(v)).collect();

        let fit = EquationOfState::fit_from_pv_data(&volumes, &pressures, 11.0);
        assert!(fit.is_some());
        let (k0, kp) = fit.unwrap();
        assert!(
            (k0 - 160.0).abs() < 5.0,
            "Fitted K0={} should be near 160",
            k0
        );
        assert!(
            (kp - 4.2).abs() < 0.5,
            "Fitted K0'={} should be near 4.2",
            kp
        );
    }

    #[test]
    fn test_eos_presets() {
        let mgo = EquationOfState::mgo();
        assert!((mgo.k0 - 160.0).abs() < 0.1);

        let fe = EquationOfState::iron_hcp();
        assert!((fe.k0 - 165.0).abs() < 0.1);

        let dia = EquationOfState::diamond();
        assert!(dia.k0 > 400.0); // Diamond is very stiff
    }

    // --- BraggPeakIndexer ---

    #[test]
    fn test_bragg_d_spacing() {
        let indexer = BraggPeakIndexer::new(0.3344); // Synchrotron
        let d = indexer.two_theta_to_d(10.0);
        assert!(d > 0.0 && d.is_finite());
        let tt_back = indexer.d_to_two_theta(d);
        assert!((tt_back - 10.0).abs() < 1e-6);
    }

    #[test]
    fn test_bragg_cubic_indexing() {
        let indexer = BraggPeakIndexer::new(0.3344);
        let a = 4.0; // 4 Angstrom cubic cell

        // Generate expected 2-theta for (100), (110), (111), (200)
        let hkl_sums = [1.0, 2.0, 3.0, 4.0];
        let two_thetas: Vec<f64> = hkl_sums
            .iter()
            .map(|&s: &f64| {
                let d = a / s.sqrt();
                indexer.d_to_two_theta(d)
            })
            .filter(|tt| tt.is_finite())
            .collect();

        let indexed = indexer.index_cubic(&two_thetas, a);
        assert!(!indexed.is_empty());
        // First peak should be (1,0,0)
        assert_eq!(indexed[0].hkl, (1, 0, 0));
    }

    #[test]
    fn test_bragg_lattice_parameter_estimate() {
        let indexer = BraggPeakIndexer::new(0.3344);
        let a_true = 5.0;

        let hkl_sums = [1.0, 2.0, 3.0, 4.0, 5.0];
        let two_thetas: Vec<f64> = hkl_sums
            .iter()
            .map(|&s: &f64| {
                let d = a_true / s.sqrt();
                indexer.d_to_two_theta(d)
            })
            .filter(|tt| tt.is_finite())
            .collect();

        let a_est = indexer.estimate_cubic_lattice_parameter(&two_thetas);
        assert!(a_est.is_some());
        let a = a_est.unwrap();
        assert!(
            (a - a_true).abs() < 0.5,
            "Estimated a={} should be near {}",
            a,
            a_true
        );
    }

    // --- PressureTemperatureMapper ---

    #[test]
    fn test_pt_mapper_add_points() {
        let mut mapper = PressureTemperatureMapper::new();
        mapper.add_point(10.0, 300.0, Some("alpha".into()));
        mapper.add_point(20.0, 500.0, Some("beta".into()));
        assert_eq!(mapper.path().len(), 2);
        assert_eq!(mapper.path()[0].phase.as_deref(), Some("alpha"));
    }

    #[test]
    fn test_pt_nearest_boundary() {
        let mut mapper = PressureTemperatureMapper::new();
        mapper.add_boundary_point(10.0, 1000.0, "alpha-beta");
        mapper.add_boundary_point(50.0, 2000.0, "beta-gamma");

        let (dist, name) = mapper.nearest_boundary(10.5, 1050.0).unwrap();
        assert_eq!(name, "alpha-beta");
        assert!(dist < 2.0);
    }

    #[test]
    fn test_simon_glatzel() {
        // Iron melting: T0 ~ 1811 K, P0 = 0, a ~ 26.0, c ~ 1.76
        let tm = PressureTemperatureMapper::simon_glatzel(1811.0, 0.0, 26.0, 1.76, 100.0);
        assert!(tm > 3000.0, "Iron melting at 100 GPa should be >3000 K: {}", tm);
        assert!(tm < 6000.0, "Iron melting at 100 GPa should be <6000 K: {}", tm);
    }

    #[test]
    fn test_clausius_clapeyron() {
        let slope =
            PressureTemperatureMapper::clausius_clapeyron_slope(1000.0, 0.5, 10000.0);
        // dT/dP = T * Delta_V / Delta_H in K/GPa
        assert!(slope > 0.0);
    }

    // --- LaserHeatingController ---

    #[test]
    fn test_wien_peak_wavelength() {
        // Sun surface ~5778 K -> peak ~502 nm
        let wl = LaserHeatingController::wien_peak_wavelength(5778.0);
        assert!(wl > 480.0 && wl < 520.0, "Wien peak at 5778 K: {} nm", wl);
    }

    #[test]
    fn test_temperature_from_wien() {
        let temp = LaserHeatingController::temperature_from_wien(500.0);
        // 500 nm -> ~5796 K
        assert!(temp > 5500.0 && temp < 6100.0, "T from 500 nm: {} K", temp);
    }

    #[test]
    fn test_planck_radiance() {
        let ctrl = LaserHeatingController::default_visible();
        // At 3000 K, visible range should have nonzero radiance
        let radiance = ctrl.planck_spectral_radiance(700.0, 3000.0);
        assert!(radiance > 0.0);
        // At higher temperature, same wavelength should be brighter
        let radiance_hot = ctrl.planck_spectral_radiance(700.0, 5000.0);
        assert!(radiance_hot > radiance);
    }

    #[test]
    fn test_laser_heating_fit() {
        let mut ctrl = LaserHeatingController::new(0.5, (500.0, 900.0));
        let temp_true = 3000.0;

        // Generate synthetic Planck spectrum
        let wavelengths: Vec<f64> = (500..=900).map(|i| i as f64).collect();
        let intensities: Vec<f64> = wavelengths
            .iter()
            .map(|&wl| ctrl.planck_spectral_radiance(wl, temp_true))
            .collect();

        let result = ctrl.fit_temperature(&wavelengths, &intensities);
        assert!(result.is_some());
        let (t_fit, r2) = result.unwrap();
        assert!(
            (t_fit - temp_true).abs() < 100.0,
            "Fitted T={} should be near {}",
            t_fit,
            temp_true
        );
        assert!(r2 > 0.99, "R^2={} should be near 1.0", r2);
    }

    // --- GasketThicknessEstimator ---

    #[test]
    fn test_gasket_two_fringes() {
        let est = GasketThicknessEstimator::new(1.0); // n=1 for gas
        // Two fringes at 600 nm and 500 nm
        let t = est.thickness_from_two_fringes(600.0, 500.0);
        // t = 600 * 500 / (2 * 1 * 100) = 1500 nm = 1.5 um
        assert!(
            (t - 1.5).abs() < 0.01,
            "Thickness should be 1.5 um, got {}",
            t
        );
    }

    #[test]
    fn test_gasket_fringe_series() {
        let mut est = GasketThicknessEstimator::new(1.0);
        // For a 10 um thick chamber with n=1, fringes separated by ~18 nm near 600 nm
        // Actually compute exact fringe positions for 10 um = 10000 nm
        // Fringe condition: 2*n*t = m*lambda -> lambda = 2*t/m
        let t_nm = 10000.0;
        let fringes: Vec<f64> = (15..=25)
            .map(|m| 2.0 * t_nm / m as f64)
            .collect();

        let thickness = est.thickness_from_fringe_series(&fringes);
        assert!(thickness.is_some());
        let t_um = thickness.unwrap();
        assert!(
            (t_um - 10.0).abs() < 0.5,
            "Thickness should be ~10 um, got {}",
            t_um
        );
    }

    #[test]
    fn test_gasket_expected_fringes() {
        let est = GasketThicknessEstimator::new(1.0);
        let count = est.expected_fringe_count(10.0, 500.0, 700.0);
        assert!(count > 0.0);
    }

    #[test]
    fn test_gasket_with_angle() {
        let est_normal = GasketThicknessEstimator::new(1.0);
        let est_angle = GasketThicknessEstimator::new(1.0).with_angle(0.1);
        let t_normal = est_normal.thickness_from_two_fringes(600.0, 500.0);
        let t_angle = est_angle.thickness_from_two_fringes(600.0, 500.0);
        // Off-axis measurement gives slightly larger apparent thickness
        assert!(t_angle > t_normal);
    }

    // --- Integration / cross-component tests ---

    #[test]
    fn test_ruby_pressure_with_eos() {
        // Use ruby gauge to measure pressure, then compute volume from EOS
        let calc = PressureCalculator::new_mao_hydrostatic();
        let eos = EquationOfState::mgo();

        let wl = 700.0; // Some shifted ruby line
        let p = calc.pressure_from_wavelength(wl);
        assert!(p > 0.0);

        // Find volume at this pressure using bisection
        let mut v_lo = eos.v0 * 0.5;
        let mut v_hi = eos.v0;
        for _ in 0..100 {
            let v_mid = (v_lo + v_hi) / 2.0;
            let p_mid = eos.pressure(v_mid);
            if p_mid < p {
                v_hi = v_mid;
            } else {
                v_lo = v_mid;
            }
        }
        let v_at_p = (v_lo + v_hi) / 2.0;
        assert!(v_at_p < eos.v0, "Volume should decrease under pressure");
    }

    #[test]
    fn test_pt_path_with_laser_heating() {
        let mut mapper = PressureTemperatureMapper::new();
        let mut ctrl = LaserHeatingController::default_visible();

        // Simulate a compression+heating experiment
        let calc = PressureCalculator::new_mao_hydrostatic();
        for i in 0..5 {
            let wl = 695.0 + i as f64; // Increasing wavelength -> increasing pressure
            let p = calc.pressure_from_wavelength(wl);
            let t = 300.0 + i as f64 * 500.0; // Increasing temperature
            mapper.add_point(p, t, None);
        }

        assert_eq!(mapper.path().len(), 5);
        // Pressure should increase along the path
        let pressures: Vec<f64> = mapper.path().iter().map(|pt| pt.pressure_gpa).collect();
        for i in 1..pressures.len() {
            assert!(pressures[i] > pressures[i - 1]);
        }
    }

    #[test]
    fn test_match_peaks_basic() {
        let prev = vec![100.0, 200.0, 300.0];
        let curr = vec![101.0, 199.0, 301.0];
        let matches = match_peaks(&prev, &curr);
        assert_eq!(matches.len(), 3);
    }
}
