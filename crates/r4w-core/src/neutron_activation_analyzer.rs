// trace:FR-NAA | ai:claude
//! # Neutron Activation Analysis (NAA) Signal Processor
//!
//! Implements signal processing for Neutron Activation Analysis, a highly
//! sensitive nuclear analytical technique for elemental and isotopic
//! determination. Applications include trace element quantification, nuclear
//! forensics, archaeological artifact provenance, environmental monitoring,
//! and industrial quality control.
//!
//! Key capabilities:
//! - **Activation calculation** using the fundamental NAA equation
//! - **Decay correction** for counting losses during finite measurement time
//! - **Peak area fitting** with Gaussian + linear background for gamma spectra
//! - **Comparator method** concentration determination (sample vs standard)
//! - **Spectral interference** correction for overlapping gamma lines
//! - **Self-shielding** correction for neutron flux attenuation in samples
//! - **Coincidence summing** correction for cascade gamma-ray summing
//! - **Isotope database** with gamma energies and half-lives for common NAA nuclides
//! - **Pulse shape discrimination** for neutron/gamma event separation
//!
//! All math is implemented from scratch using only the standard library.
//!
//! # Example
//!
//! ```
//! use r4w_core::neutron_activation_analyzer::{
//!     NaaConfig, ActivationCalculator, DecayCorrector, IsotopeDatabase,
//! };
//!
//! let config = NaaConfig {
//!     neutron_flux: 1e13,          // n/cm^2/s (research reactor)
//!     irradiation_time: 3600.0,    // 1 hour
//!     decay_time: 86400.0,         // 1 day cooldown
//!     counting_time: 3600.0,       // 1 hour counting
//!     detector_efficiency: 0.01,   // 1% at 1332 keV
//! };
//!
//! let db = IsotopeDatabase::new();
//! let na24 = db.lookup("Na-24").unwrap();
//!
//! // Calculate induced activity for 1 microgram of sodium
//! let calc = ActivationCalculator::new(&config);
//! let n_atoms = 1e-6 * 6.022e23 / 22.99; // 1 ug Na
//! let activity = calc.activity(n_atoms, na24.thermal_cross_section_barn, na24.half_life_s);
//! assert!(activity > 0.0);
//!
//! // Correct measured counts for decay during counting
//! let corrector = DecayCorrector::new(na24.half_life_s);
//! let true_counts = corrector.correct_counting_loss(1000.0, config.counting_time);
//! assert!(true_counts >= 1000.0);
//! ```

use std::f64::consts::PI;

/// Natural logarithm of 2.
const LN2: f64 = 0.693_147_180_559_945_3;

/// Avogadro's number (atoms/mol).
const AVOGADRO: f64 = 6.022_140_76e23;

// ---------------------------------------------------------------------------
// NaaConfig
// ---------------------------------------------------------------------------

/// Configuration for a Neutron Activation Analysis measurement.
#[derive(Debug, Clone)]
pub struct NaaConfig {
    /// Neutron flux in n/cm^2/s. Typical: 1e11 (TRIGA) to 1e14 (HFR).
    pub neutron_flux: f64,
    /// Irradiation time in seconds.
    pub irradiation_time: f64,
    /// Decay (cooling) time in seconds after irradiation ends.
    pub decay_time: f64,
    /// Counting (measurement) time in seconds.
    pub counting_time: f64,
    /// Absolute detector efficiency at the gamma energy of interest (0..1).
    pub detector_efficiency: f64,
}

impl Default for NaaConfig {
    fn default() -> Self {
        Self {
            neutron_flux: 1e13,
            irradiation_time: 3600.0,
            decay_time: 7200.0,
            counting_time: 3600.0,
            detector_efficiency: 0.01,
        }
    }
}

// ---------------------------------------------------------------------------
// ActivationCalculator
// ---------------------------------------------------------------------------

/// Calculates induced radioactivity from neutron activation.
///
/// The fundamental NAA equation is:
///
/// `A = N * sigma * phi * S * D * C`
///
/// where:
/// - N = number of target atoms
/// - sigma = neutron capture cross section (cm^2)
/// - phi = neutron flux (n/cm^2/s)
/// - S = saturation factor = 1 - exp(-lambda * t_irr)
/// - D = decay factor = exp(-lambda * t_d)
/// - C = counting factor = (1 - exp(-lambda * t_c)) / (lambda * t_c)
#[derive(Debug, Clone)]
pub struct ActivationCalculator {
    /// Neutron flux (n/cm^2/s).
    pub flux: f64,
    /// Irradiation time (s).
    pub t_irr: f64,
    /// Decay time (s).
    pub t_d: f64,
    /// Counting time (s).
    pub t_c: f64,
}

impl ActivationCalculator {
    /// Create a new calculator from an NAA configuration.
    pub fn new(config: &NaaConfig) -> Self {
        Self {
            flux: config.neutron_flux,
            t_irr: config.irradiation_time,
            t_d: config.decay_time,
            t_c: config.counting_time,
        }
    }

    /// Compute the decay constant lambda = ln(2) / t_half.
    pub fn decay_constant(half_life_s: f64) -> f64 {
        LN2 / half_life_s
    }

    /// Saturation factor: S = 1 - exp(-lambda * t_irr).
    pub fn saturation_factor(&self, half_life_s: f64) -> f64 {
        let lambda = Self::decay_constant(half_life_s);
        1.0 - (-lambda * self.t_irr).exp()
    }

    /// Decay factor: D = exp(-lambda * t_d).
    pub fn decay_factor(&self, half_life_s: f64) -> f64 {
        let lambda = Self::decay_constant(half_life_s);
        (-lambda * self.t_d).exp()
    }

    /// Counting factor: C = (1 - exp(-lambda * t_c)) / (lambda * t_c).
    ///
    /// For very long half-lives (lambda*t_c << 1), C approaches 1.
    pub fn counting_factor(&self, half_life_s: f64) -> f64 {
        let lambda = Self::decay_constant(half_life_s);
        let x = lambda * self.t_c;
        if x < 1e-10 {
            // Taylor expansion: (1 - exp(-x))/x ≈ 1 - x/2
            1.0 - x / 2.0
        } else {
            (1.0 - (-x).exp()) / x
        }
    }

    /// Calculate the induced activity at the start of counting in Bq.
    ///
    /// A = N * sigma * phi * S * D
    ///
    /// - `n_atoms`: number of target atoms
    /// - `cross_section_barn`: thermal neutron cross section in barns (1 barn = 1e-24 cm^2)
    /// - `half_life_s`: product nuclide half-life in seconds
    pub fn activity(&self, n_atoms: f64, cross_section_barn: f64, half_life_s: f64) -> f64 {
        let sigma_cm2 = cross_section_barn * 1e-24;
        let s = self.saturation_factor(half_life_s);
        let d = self.decay_factor(half_life_s);
        n_atoms * sigma_cm2 * self.flux * s * d
    }

    /// Calculate expected total counts in the detector during counting time.
    ///
    /// Counts = A * eps * C * t_c
    ///
    /// where eps is detector efficiency and C is the counting factor.
    pub fn expected_counts(
        &self,
        n_atoms: f64,
        cross_section_barn: f64,
        half_life_s: f64,
        detector_efficiency: f64,
    ) -> f64 {
        let a = self.activity(n_atoms, cross_section_barn, half_life_s);
        let c = self.counting_factor(half_life_s);
        a * detector_efficiency * c * self.t_c
    }

    /// Compute number of target atoms from mass and atomic weight.
    ///
    /// N = (mass_g / atomic_weight) * N_A * abundance
    pub fn atoms_from_mass(mass_g: f64, atomic_weight: f64, isotopic_abundance: f64) -> f64 {
        (mass_g / atomic_weight) * AVOGADRO * isotopic_abundance
    }
}

// ---------------------------------------------------------------------------
// DecayCorrector
// ---------------------------------------------------------------------------

/// Corrects for radioactive decay effects in NAA measurements.
///
/// During a finite counting time, the sample activity decreases, so the
/// measured count rate underestimates the true activity at the start of
/// counting.
///
/// C_true = C_meas * (lambda * t_c) / (1 - exp(-lambda * t_c))
#[derive(Debug, Clone)]
pub struct DecayCorrector {
    /// Half-life in seconds.
    pub half_life_s: f64,
    /// Decay constant lambda = ln(2)/t_half.
    pub lambda: f64,
}

impl DecayCorrector {
    /// Create a new decay corrector for a given half-life.
    pub fn new(half_life_s: f64) -> Self {
        Self {
            half_life_s,
            lambda: LN2 / half_life_s,
        }
    }

    /// Correct measured counts for decay during counting.
    ///
    /// Returns the true count value at the start of the counting interval.
    pub fn correct_counting_loss(&self, measured_counts: f64, counting_time_s: f64) -> f64 {
        let x = self.lambda * counting_time_s;
        if x < 1e-10 {
            // For very long half-lives, negligible correction
            measured_counts * (1.0 + x / 2.0)
        } else {
            measured_counts * x / (1.0 - (-x).exp())
        }
    }

    /// Correct for decay between end of irradiation and start of counting.
    ///
    /// A_eoi = A_measured * exp(lambda * t_d)
    pub fn correct_decay_time(&self, activity: f64, decay_time_s: f64) -> f64 {
        activity * (self.lambda * decay_time_s).exp()
    }

    /// Calculate remaining activity after a given time.
    ///
    /// A(t) = A0 * exp(-lambda * t)
    pub fn remaining_activity(&self, initial_activity: f64, time_s: f64) -> f64 {
        initial_activity * (-self.lambda * time_s).exp()
    }

    /// Number of half-lives elapsed in a given time.
    pub fn half_lives_elapsed(&self, time_s: f64) -> f64 {
        time_s / self.half_life_s
    }
}

// ---------------------------------------------------------------------------
// PeakAreaCalculator
// ---------------------------------------------------------------------------

/// Gaussian peak + linear background fitting for gamma-ray spectroscopy.
///
/// Model: I(E) = A * exp(-(E - E0)^2 / (2 * sigma^2)) + a + b * E
///
/// - A: peak amplitude
/// - E0: centroid energy
/// - sigma: Gaussian width (FWHM = 2.355 * sigma)
/// - a, b: linear background coefficients
#[derive(Debug, Clone)]
pub struct PeakAreaCalculator;

/// Result of a Gaussian peak fit.
#[derive(Debug, Clone)]
pub struct PeakFitResult {
    /// Centroid energy.
    pub centroid: f64,
    /// Gaussian sigma (standard deviation).
    pub sigma: f64,
    /// Peak amplitude above background.
    pub amplitude: f64,
    /// Net peak area (integrated counts above background).
    pub net_area: f64,
    /// FWHM = 2.355 * sigma.
    pub fwhm: f64,
    /// Background intercept.
    pub background_a: f64,
    /// Background slope.
    pub background_b: f64,
    /// Statistical uncertainty (1-sigma) on net area.
    pub area_uncertainty: f64,
}

impl PeakAreaCalculator {
    /// Evaluate the Gaussian + linear background model at energy E.
    pub fn model(e: f64, amplitude: f64, centroid: f64, sigma: f64, bg_a: f64, bg_b: f64) -> f64 {
        let gauss = amplitude * (-((e - centroid).powi(2)) / (2.0 * sigma * sigma)).exp();
        gauss + bg_a + bg_b * e
    }

    /// Fit a Gaussian peak to spectral data in the region [e_low, e_high].
    ///
    /// Uses iterative refinement: estimate background from edges, find peak,
    /// then refine centroid and sigma via moment analysis.
    ///
    /// `energies` and `counts` must have the same length.
    pub fn fit_peak(energies: &[f64], counts: &[f64], e_low: f64, e_high: f64) -> Option<PeakFitResult> {
        if energies.len() != counts.len() || energies.len() < 5 {
            return None;
        }

        // Select region of interest
        let mut roi_e = Vec::new();
        let mut roi_c = Vec::new();
        for (i, &e) in energies.iter().enumerate() {
            if e >= e_low && e <= e_high {
                roi_e.push(e);
                roi_c.push(counts[i]);
            }
        }

        if roi_e.len() < 5 {
            return None;
        }

        // Estimate linear background from first and last 20% of ROI
        let n_bg = (roi_e.len() / 5).max(2);
        let (bg_low_e, bg_low_c) = mean_pair(&roi_e[..n_bg], &roi_c[..n_bg]);
        let (bg_high_e, bg_high_c) = mean_pair(
            &roi_e[roi_e.len() - n_bg..],
            &roi_c[roi_e.len() - n_bg..],
        );

        let bg_b = if (bg_high_e - bg_low_e).abs() > 1e-12 {
            (bg_high_c - bg_low_c) / (bg_high_e - bg_low_e)
        } else {
            0.0
        };
        let bg_a = bg_low_c - bg_b * bg_low_e;

        // Subtract background and find peak
        let net: Vec<f64> = roi_e
            .iter()
            .zip(roi_c.iter())
            .map(|(&e, &c)| (c - bg_a - bg_b * e).max(0.0))
            .collect();

        // Find maximum (amplitude estimate)
        let mut max_idx = 0;
        let mut max_val = 0.0_f64;
        for (i, &v) in net.iter().enumerate() {
            if v > max_val {
                max_val = v;
                max_idx = i;
            }
        }

        if max_val < 1.0 {
            return None;
        }

        // Moment analysis for centroid and sigma
        let total_net: f64 = net.iter().sum();
        if total_net <= 0.0 {
            return None;
        }

        let centroid: f64 = roi_e
            .iter()
            .zip(net.iter())
            .map(|(&e, &n)| e * n)
            .sum::<f64>()
            / total_net;

        let variance: f64 = roi_e
            .iter()
            .zip(net.iter())
            .map(|(&e, &n)| n * (e - centroid).powi(2))
            .sum::<f64>()
            / total_net;

        let sigma = variance.sqrt();
        if sigma < 1e-12 {
            return None;
        }

        let fwhm = 2.355 * sigma;

        // Amplitude from max_val (refined)
        let amplitude = max_val;

        // Net area = integral of Gaussian = A * sigma * sqrt(2*pi)
        let net_area = amplitude * sigma * (2.0 * PI).sqrt();

        // Statistical uncertainty: sqrt(gross counts) in peak region
        // Estimate gross counts in +/- 2*sigma
        let mut gross_counts = 0.0;
        let mut bg_counts = 0.0;
        for (&e, &c) in roi_e.iter().zip(roi_c.iter()) {
            if (e - centroid).abs() <= 2.0 * sigma {
                gross_counts += c;
                bg_counts += bg_a + bg_b * e;
            }
        }
        let area_uncertainty = (gross_counts + bg_counts).sqrt();

        Some(PeakFitResult {
            centroid,
            sigma,
            amplitude,
            net_area,
            fwhm,
            background_a: bg_a,
            background_b: bg_b,
            area_uncertainty,
        })
    }

    /// Generate a synthetic gamma-ray spectrum with Gaussian peaks on a
    /// linear background.
    ///
    /// Returns (energies, counts) vectors.
    pub fn generate_spectrum(
        peaks: &[(f64, f64, f64)], // (energy, amplitude, sigma)
        e_min: f64,
        e_max: f64,
        n_bins: usize,
        bg_a: f64,
        bg_b: f64,
    ) -> (Vec<f64>, Vec<f64>) {
        let de = (e_max - e_min) / n_bins as f64;
        let energies: Vec<f64> = (0..n_bins).map(|i| e_min + (i as f64 + 0.5) * de).collect();
        let counts: Vec<f64> = energies
            .iter()
            .map(|&e| {
                let bg = bg_a + bg_b * e;
                let peak_sum: f64 = peaks
                    .iter()
                    .map(|&(e0, a, s)| a * (-((e - e0).powi(2)) / (2.0 * s * s)).exp())
                    .sum();
                (bg + peak_sum).max(0.0)
            })
            .collect();
        (energies, counts)
    }
}

/// Compute the mean of paired (energy, counts) slices.
fn mean_pair(energies: &[f64], counts: &[f64]) -> (f64, f64) {
    let n = energies.len() as f64;
    let me = energies.iter().sum::<f64>() / n;
    let mc = counts.iter().sum::<f64>() / n;
    (me, mc)
}

// ---------------------------------------------------------------------------
// ConcentrationCalculator
// ---------------------------------------------------------------------------

/// Comparator method for elemental concentration determination.
///
/// The ratio of sample to standard concentrations is:
///
/// C_sample / C_std = (A_sample / A_std) * (m_std / m_sample) * (D_std / D_sample)
///
/// where D corrections account for different decay times.
#[derive(Debug, Clone)]
pub struct ConcentrationCalculator {
    /// Half-life of the indicator isotope in seconds.
    pub half_life_s: f64,
}

/// Input data for a comparator measurement.
#[derive(Debug, Clone)]
pub struct ComparatorInput {
    /// Net peak area (counts) for the sample.
    pub sample_counts: f64,
    /// Net peak area (counts) for the standard.
    pub standard_counts: f64,
    /// Mass of the sample in grams.
    pub sample_mass_g: f64,
    /// Mass of the standard element content in micrograms.
    pub standard_mass_ug: f64,
    /// Decay time from end of irradiation to midpoint of sample counting (s).
    pub sample_decay_time_s: f64,
    /// Decay time from end of irradiation to midpoint of standard counting (s).
    pub standard_decay_time_s: f64,
    /// Sample counting live time (s).
    pub sample_count_time_s: f64,
    /// Standard counting live time (s).
    pub standard_count_time_s: f64,
}

/// Result of a concentration determination.
#[derive(Debug, Clone)]
pub struct ConcentrationResult {
    /// Element concentration in ug/g (ppm).
    pub concentration_ppm: f64,
    /// Concentration in mg/kg (same as ppm, for clarity).
    pub concentration_mg_per_kg: f64,
    /// Decay correction factor applied (D_std / D_sample).
    pub decay_correction_factor: f64,
    /// Counting correction factor (C_std / C_sample).
    pub counting_correction_factor: f64,
}

impl ConcentrationCalculator {
    /// Create a new concentration calculator.
    pub fn new(half_life_s: f64) -> Self {
        Self { half_life_s }
    }

    /// Calculate elemental concentration using the comparator method.
    pub fn comparator_method(&self, input: &ComparatorInput) -> ConcentrationResult {
        let lambda = LN2 / self.half_life_s;

        // Decay corrections: exp(lambda * t_d) to project back to end of irradiation
        let d_sample = (-lambda * input.sample_decay_time_s).exp();
        let d_standard = (-lambda * input.standard_decay_time_s).exp();
        let decay_correction = d_standard / d_sample;

        // Counting loss corrections
        let x_sample = lambda * input.sample_count_time_s;
        let x_standard = lambda * input.standard_count_time_s;

        let c_sample = if x_sample < 1e-10 {
            1.0
        } else {
            (1.0 - (-x_sample).exp()) / x_sample
        };
        let c_standard = if x_standard < 1e-10 {
            1.0
        } else {
            (1.0 - (-x_standard).exp()) / x_standard
        };
        let counting_correction = c_standard / c_sample;

        // Concentration: C_sample = C_std * (A_sample/A_std) * (m_std/m_sample) * corrections
        // C_std is given as standard_mass_ug (total element mass in standard)
        // Result in ug per gram of sample
        let conc_ppm = (input.sample_counts / input.standard_counts)
            * (input.standard_mass_ug / input.sample_mass_g)
            * decay_correction
            * counting_correction;

        ConcentrationResult {
            concentration_ppm: conc_ppm,
            concentration_mg_per_kg: conc_ppm,
            decay_correction_factor: decay_correction,
            counting_correction_factor: counting_correction,
        }
    }

    /// Calculate concentration from k0-standardization method (single comparator).
    ///
    /// C_a = (N_p / (eps * t_c)) * (M_a / (N_A * theta * sigma0 * phi * S * D * C))
    ///       * (1 / (k0 * m))
    ///
    /// Simplified: uses the k0 factor to relate to a monitor (e.g., Au).
    pub fn k0_method(
        &self,
        sample_counts: f64,
        sample_mass_g: f64,
        k0_factor: f64,
        monitor_counts: f64,
        monitor_mass_g: f64,
        monitor_half_life_s: f64,
        sample_decay_time_s: f64,
        monitor_decay_time_s: f64,
        counting_time_s: f64,
    ) -> f64 {
        let lambda_s = LN2 / self.half_life_s;
        let lambda_m = LN2 / monitor_half_life_s;

        // Decay corrections
        let d_s = (-lambda_s * sample_decay_time_s).exp();
        let d_m = (-lambda_m * monitor_decay_time_s).exp();

        // Counting corrections
        let x_s = lambda_s * counting_time_s;
        let x_m = lambda_m * counting_time_s;
        let c_s = if x_s < 1e-10 { 1.0 } else { (1.0 - (-x_s).exp()) / x_s };
        let c_m = if x_m < 1e-10 { 1.0 } else { (1.0 - (-x_m).exp()) / x_m };

        // Concentration (ug/g)
        let ratio = (sample_counts / (d_s * c_s)) / (monitor_counts / (d_m * c_m));
        ratio * (monitor_mass_g / sample_mass_g) / k0_factor * 1e6
    }
}

// ---------------------------------------------------------------------------
// InterferenceCorrector
// ---------------------------------------------------------------------------

/// Corrects for spectral interferences from overlapping gamma-ray lines.
///
/// In NAA, multiple isotopes may produce gamma rays at similar energies
/// (within detector resolution). The corrector subtracts interfering
/// contributions using known interference factors.
#[derive(Debug, Clone)]
pub struct InterferenceCorrector {
    /// List of interference entries: (interfering_isotope_name, correction_factor).
    /// The correction factor is the ratio of the interfering peak area
    /// to the primary peak area of the interferent.
    corrections: Vec<InterferenceEntry>,
}

/// A single spectral interference entry.
#[derive(Debug, Clone)]
pub struct InterferenceEntry {
    /// Name of the interfering isotope.
    pub isotope: String,
    /// Gamma energy of the primary line of the interferent (keV).
    pub primary_energy_kev: f64,
    /// Gamma energy of the interfering line (keV).
    pub interfering_energy_kev: f64,
    /// Interference factor: ratio of interference peak to primary peak intensity.
    pub interference_factor: f64,
}

impl InterferenceCorrector {
    /// Create a new interference corrector with no entries.
    pub fn new() -> Self {
        Self {
            corrections: Vec::new(),
        }
    }

    /// Add an interference correction entry.
    pub fn add_interference(
        &mut self,
        isotope: &str,
        primary_energy_kev: f64,
        interfering_energy_kev: f64,
        interference_factor: f64,
    ) {
        self.corrections.push(InterferenceEntry {
            isotope: isotope.to_string(),
            primary_energy_kev,
            interfering_energy_kev,
            interference_factor,
        });
    }

    /// Correct a measured peak area by subtracting known interferences.
    ///
    /// `interferent_areas` maps isotope names to their measured primary peak areas.
    /// The corrected area is: A_corrected = A_measured - sum(f_i * A_interferent_i)
    pub fn correct_area(
        &self,
        measured_area: f64,
        target_energy_kev: f64,
        interferent_areas: &[(&str, f64)],
        energy_tolerance_kev: f64,
    ) -> f64 {
        let mut correction = 0.0;
        for entry in &self.corrections {
            if (entry.interfering_energy_kev - target_energy_kev).abs() <= energy_tolerance_kev {
                // Find the measured area of this interferent's primary peak
                for &(name, area) in interferent_areas {
                    if name == entry.isotope {
                        correction += entry.interference_factor * area;
                    }
                }
            }
        }
        (measured_area - correction).max(0.0)
    }

    /// Return the number of registered interferences.
    pub fn num_interferences(&self) -> usize {
        self.corrections.len()
    }
}

impl Default for InterferenceCorrector {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// SelfShieldingCorrector
// ---------------------------------------------------------------------------

/// Corrects for neutron self-shielding in the sample.
///
/// When the sample has a significant macroscopic cross section, neutrons
/// are attenuated as they traverse the material. For a slab geometry:
///
/// f = (1 - exp(-Sigma * t)) / (Sigma * t)
///
/// where Sigma is the macroscopic cross section (cm^-1) and t is thickness (cm).
#[derive(Debug, Clone)]
pub struct SelfShieldingCorrector;

impl SelfShieldingCorrector {
    /// Compute the self-shielding factor for a slab of thickness t (cm).
    ///
    /// `macro_cross_section` is Sigma in cm^-1 (= N * sigma_micro).
    /// Returns f in (0, 1], where 1 = no shielding, 0 = total shielding.
    pub fn slab_factor(macro_cross_section: f64, thickness_cm: f64) -> f64 {
        let x = macro_cross_section * thickness_cm;
        if x < 1e-10 {
            1.0 - x / 2.0
        } else {
            (1.0 - (-x).exp()) / x
        }
    }

    /// Self-shielding factor for a cylindrical sample.
    ///
    /// Approximate: f ≈ 1 / (1 + Sigma * R / 2) for small Sigma*R.
    /// For more accurate results, uses the rational approximation.
    pub fn cylinder_factor(macro_cross_section: f64, radius_cm: f64) -> f64 {
        let x = macro_cross_section * radius_cm;
        if x < 1e-10 {
            1.0 - x / 2.0
        } else if x < 2.0 {
            // Rational approximation for moderate shielding
            1.0 / (1.0 + x / 2.0 + x * x / 12.0)
        } else {
            // Stronger shielding: numerical integration approximation
            // Using 2/(x*sqrt(pi)) * (1 - exp(-x)) for large x
            2.0 / (x * PI.sqrt()) * (1.0 - (-x).exp())
        }
    }

    /// Self-shielding factor for a spherical sample.
    ///
    /// f = 3/(2*x^2) * (1 - (2/x^2)*(1 - (1+x)*exp(-x)))
    /// where x = Sigma * diameter = 2 * Sigma * R
    pub fn sphere_factor(macro_cross_section: f64, radius_cm: f64) -> f64 {
        let x = 2.0 * macro_cross_section * radius_cm;
        if x < 1e-8 {
            1.0 - x / 4.0
        } else {
            let inner = 1.0 - (1.0 + x) * (-x).exp();
            3.0 / (2.0 * x * x) * (2.0 * inner - x * (-x).exp()) + (3.0 / x) * (1.0 - (2.0 / (x * x)) * inner)
        }
    }

    /// Compute macroscopic cross section from elemental data.
    ///
    /// Sigma = (rho * N_A / M) * sigma_micro
    ///
    /// - `density_g_per_cm3`: material density
    /// - `atomic_weight`: average atomic weight (g/mol)
    /// - `micro_cross_section_barn`: microscopic cross section in barns
    pub fn macroscopic_cross_section(
        density_g_per_cm3: f64,
        atomic_weight: f64,
        micro_cross_section_barn: f64,
    ) -> f64 {
        let sigma_cm2 = micro_cross_section_barn * 1e-24;
        (density_g_per_cm3 * AVOGADRO / atomic_weight) * sigma_cm2
    }
}

// ---------------------------------------------------------------------------
// CoincidenceSumCorrector
// ---------------------------------------------------------------------------

/// Corrects for cascade (true) coincidence summing effects.
///
/// When a nuclide emits two or more gamma rays in rapid succession
/// (cascade), they may be detected simultaneously in close geometry,
/// leading to sum peaks and loss from full-energy peaks.
///
/// The correction factor for a single gamma ray in a two-gamma cascade:
///
/// C_coinc = 1 / (1 - eps_total(E2))
///
/// where eps_total is the total detection efficiency for the coincident gamma.
#[derive(Debug, Clone)]
pub struct CoincidenceSumCorrector {
    /// Cascade gamma-ray pairs: (E1_keV, E2_keV, branching_ratio).
    cascades: Vec<CascadeEntry>,
}

/// A cascade gamma-ray pair.
#[derive(Debug, Clone)]
pub struct CascadeEntry {
    /// Energy of the first gamma ray (keV).
    pub energy1_kev: f64,
    /// Energy of the second gamma ray (keV).
    pub energy2_kev: f64,
    /// Branching ratio for this cascade path.
    pub branching_ratio: f64,
}

impl CoincidenceSumCorrector {
    /// Create a new coincidence sum corrector.
    pub fn new() -> Self {
        Self {
            cascades: Vec::new(),
        }
    }

    /// Add a cascade pair.
    pub fn add_cascade(&mut self, e1_kev: f64, e2_kev: f64, branching_ratio: f64) {
        self.cascades.push(CascadeEntry {
            energy1_kev: e1_kev,
            energy2_kev: e2_kev,
            branching_ratio,
        });
    }

    /// Calculate coincidence summing correction factor for a gamma line at `target_energy_kev`.
    ///
    /// `total_efficiency_fn` maps energy (keV) to total detection efficiency.
    ///
    /// Returns the multiplicative correction factor (>= 1 for summing-out losses).
    pub fn correction_factor<F>(&self, target_energy_kev: f64, total_efficiency_fn: F) -> f64
    where
        F: Fn(f64) -> f64,
    {
        let mut sum_out = 0.0;
        let mut sum_in = 0.0;

        for cascade in &self.cascades {
            if (cascade.energy1_kev - target_energy_kev).abs() < 0.1 {
                // This gamma is part of a cascade; coincident gamma is energy2
                let eps_total = total_efficiency_fn(cascade.energy2_kev);
                sum_out += cascade.branching_ratio * eps_total;
            } else if (cascade.energy2_kev - target_energy_kev).abs() < 0.1 {
                // Same but roles reversed
                let eps_total = total_efficiency_fn(cascade.energy1_kev);
                sum_out += cascade.branching_ratio * eps_total;
            }

            // Sum-in: if E1+E2 ≈ target, counts add to the sum peak
            let e_sum = cascade.energy1_kev + cascade.energy2_kev;
            if (e_sum - target_energy_kev).abs() < 0.1 {
                let eps_peak1 = total_efficiency_fn(cascade.energy1_kev);
                let eps_peak2 = total_efficiency_fn(cascade.energy2_kev);
                sum_in += cascade.branching_ratio * eps_peak1 * eps_peak2;
            }
        }

        // Net correction: true counts / measured counts
        // measured = true * (1 - sum_out) + sum_in contributions
        let denom = 1.0 - sum_out + sum_in;
        if denom > 0.01 {
            1.0 / denom
        } else {
            1.0 // Avoid unrealistic corrections
        }
    }

    /// Number of registered cascades.
    pub fn num_cascades(&self) -> usize {
        self.cascades.len()
    }
}

impl Default for CoincidenceSumCorrector {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// IsotopeDatabase
// ---------------------------------------------------------------------------

/// Information about an isotope commonly measured in NAA.
#[derive(Debug, Clone)]
pub struct IsotopeInfo {
    /// Isotope name (e.g., "Na-24").
    pub name: &'static str,
    /// Target element.
    pub element: &'static str,
    /// Target isotope (before activation).
    pub target_isotope: &'static str,
    /// Product isotope (after activation).
    pub product_isotope: &'static str,
    /// Half-life of the product in seconds.
    pub half_life_s: f64,
    /// Primary gamma-ray energy in keV.
    pub gamma_energy_kev: f64,
    /// Gamma-ray branching ratio (emission probability).
    pub gamma_branching_ratio: f64,
    /// Thermal neutron capture cross section in barns.
    pub thermal_cross_section_barn: f64,
}

/// Database of isotopes commonly used in Neutron Activation Analysis.
#[derive(Debug, Clone)]
pub struct IsotopeDatabase {
    isotopes: Vec<IsotopeInfo>,
}

impl IsotopeDatabase {
    /// Create a new database populated with common NAA isotopes.
    pub fn new() -> Self {
        let isotopes = vec![
            IsotopeInfo {
                name: "Na-24",
                element: "Na",
                target_isotope: "Na-23",
                product_isotope: "Na-24",
                half_life_s: 14.96 * 3600.0, // 14.96 hours
                gamma_energy_kev: 1368.6,
                gamma_branching_ratio: 1.0,
                thermal_cross_section_barn: 0.530,
            },
            IsotopeInfo {
                name: "Mn-56",
                element: "Mn",
                target_isotope: "Mn-55",
                product_isotope: "Mn-56",
                half_life_s: 2.579 * 3600.0, // 2.579 hours
                gamma_energy_kev: 846.8,
                gamma_branching_ratio: 0.989,
                thermal_cross_section_barn: 13.3,
            },
            IsotopeInfo {
                name: "Au-198",
                element: "Au",
                target_isotope: "Au-197",
                product_isotope: "Au-198",
                half_life_s: 2.696 * 86400.0, // 2.696 days
                gamma_energy_kev: 411.8,
                gamma_branching_ratio: 0.9562,
                thermal_cross_section_barn: 98.65,
            },
            IsotopeInfo {
                name: "Co-60",
                element: "Co",
                target_isotope: "Co-59",
                product_isotope: "Co-60",
                half_life_s: 5.271 * 365.25 * 86400.0, // 5.271 years
                gamma_energy_kev: 1332.5,
                gamma_branching_ratio: 0.9998,
                thermal_cross_section_barn: 37.18,
            },
            IsotopeInfo {
                name: "Sc-46",
                element: "Sc",
                target_isotope: "Sc-45",
                product_isotope: "Sc-46",
                half_life_s: 83.79 * 86400.0, // 83.79 days
                gamma_energy_kev: 889.3,
                gamma_branching_ratio: 0.9998,
                thermal_cross_section_barn: 27.2,
            },
            IsotopeInfo {
                name: "Cr-51",
                element: "Cr",
                target_isotope: "Cr-50",
                product_isotope: "Cr-51",
                half_life_s: 27.70 * 86400.0, // 27.70 days
                gamma_energy_kev: 320.1,
                gamma_branching_ratio: 0.0983,
                thermal_cross_section_barn: 15.8,
            },
            IsotopeInfo {
                name: "Fe-59",
                element: "Fe",
                target_isotope: "Fe-58",
                product_isotope: "Fe-59",
                half_life_s: 44.50 * 86400.0, // 44.50 days
                gamma_energy_kev: 1099.2,
                gamma_branching_ratio: 0.565,
                thermal_cross_section_barn: 1.28,
            },
            IsotopeInfo {
                name: "Zn-65",
                element: "Zn",
                target_isotope: "Zn-64",
                product_isotope: "Zn-65",
                half_life_s: 244.1 * 86400.0, // 244.1 days
                gamma_energy_kev: 1115.5,
                gamma_branching_ratio: 0.506,
                thermal_cross_section_barn: 0.76,
            },
            IsotopeInfo {
                name: "As-76",
                element: "As",
                target_isotope: "As-75",
                product_isotope: "As-76",
                half_life_s: 26.24 * 3600.0, // 26.24 hours
                gamma_energy_kev: 559.1,
                gamma_branching_ratio: 0.45,
                thermal_cross_section_barn: 4.23,
            },
            IsotopeInfo {
                name: "Br-82",
                element: "Br",
                target_isotope: "Br-81",
                product_isotope: "Br-82",
                half_life_s: 35.28 * 3600.0, // 35.28 hours
                gamma_energy_kev: 776.5,
                gamma_branching_ratio: 0.835,
                thermal_cross_section_barn: 2.36,
            },
            IsotopeInfo {
                name: "La-140",
                element: "La",
                target_isotope: "La-139",
                product_isotope: "La-140",
                half_life_s: 40.27 * 3600.0, // 40.27 hours = 1.678 days
                gamma_energy_kev: 1596.2,
                gamma_branching_ratio: 0.954,
                thermal_cross_section_barn: 8.93,
            },
            IsotopeInfo {
                name: "Sm-153",
                element: "Sm",
                target_isotope: "Sm-152",
                product_isotope: "Sm-153",
                half_life_s: 46.50 * 3600.0, // 46.50 hours
                gamma_energy_kev: 103.2,
                gamma_branching_ratio: 0.283,
                thermal_cross_section_barn: 206.0,
            },
            IsotopeInfo {
                name: "Eu-152",
                element: "Eu",
                target_isotope: "Eu-151",
                product_isotope: "Eu-152",
                half_life_s: 13.52 * 365.25 * 86400.0, // 13.52 years
                gamma_energy_kev: 1408.0,
                gamma_branching_ratio: 0.210,
                thermal_cross_section_barn: 5900.0,
            },
            IsotopeInfo {
                name: "Hf-181",
                element: "Hf",
                target_isotope: "Hf-180",
                product_isotope: "Hf-181",
                half_life_s: 42.39 * 86400.0, // 42.39 days
                gamma_energy_kev: 482.2,
                gamma_branching_ratio: 0.856,
                thermal_cross_section_barn: 13.04,
            },
            IsotopeInfo {
                name: "W-187",
                element: "W",
                target_isotope: "W-186",
                product_isotope: "W-187",
                half_life_s: 23.72 * 3600.0, // 23.72 hours
                gamma_energy_kev: 685.8,
                gamma_branching_ratio: 0.332,
                thermal_cross_section_barn: 37.9,
            },
        ];

        Self { isotopes }
    }

    /// Look up an isotope by name (e.g., "Na-24", "Co-60").
    pub fn lookup(&self, name: &str) -> Option<&IsotopeInfo> {
        self.isotopes.iter().find(|iso| iso.name == name)
    }

    /// Look up isotopes by element symbol (e.g., "Na", "Co").
    pub fn lookup_by_element(&self, element: &str) -> Vec<&IsotopeInfo> {
        self.isotopes.iter().filter(|iso| iso.element == element).collect()
    }

    /// Find isotopes with gamma energy near a given value (within tolerance).
    pub fn find_by_energy(&self, energy_kev: f64, tolerance_kev: f64) -> Vec<&IsotopeInfo> {
        self.isotopes
            .iter()
            .filter(|iso| (iso.gamma_energy_kev - energy_kev).abs() <= tolerance_kev)
            .collect()
    }

    /// Return all isotopes in the database.
    pub fn all(&self) -> &[IsotopeInfo] {
        &self.isotopes
    }

    /// Number of isotopes in the database.
    pub fn len(&self) -> usize {
        self.isotopes.len()
    }

    /// Check if the database is empty.
    pub fn is_empty(&self) -> bool {
        self.isotopes.is_empty()
    }
}

impl Default for IsotopeDatabase {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// PulseShapeDiscriminator
// ---------------------------------------------------------------------------

/// Pulse shape discrimination (PSD) for separating neutron and gamma events.
///
/// Neutrons produce recoil protons in organic scintillators, which have a
/// slower decay component compared to gamma-ray interactions. The PSD
/// parameter is computed as:
///
/// PSD = Q_tail / Q_total
///
/// where Q_tail is the charge in the tail portion and Q_total is the total
/// pulse charge (charge comparison method).
#[derive(Debug, Clone)]
pub struct PulseShapeDiscriminator {
    /// Start of the tail integration window as a fraction of total pulse length.
    pub tail_start_fraction: f64,
    /// PSD threshold: above = neutron, below = gamma.
    pub psd_threshold: f64,
    /// Minimum pulse amplitude to consider (noise rejection).
    pub min_amplitude: f64,
}

/// Classification result for a single pulse.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ParticleType {
    /// Gamma-ray interaction (fast decay, low PSD ratio).
    Gamma,
    /// Neutron interaction (slow tail, high PSD ratio).
    Neutron,
    /// Below amplitude threshold, indeterminate.
    Noise,
}

/// PSD analysis result for a set of pulses.
#[derive(Debug, Clone)]
pub struct PsdResult {
    /// Number of gamma events.
    pub gamma_count: usize,
    /// Number of neutron events.
    pub neutron_count: usize,
    /// Number of noise events.
    pub noise_count: usize,
    /// PSD values for each pulse.
    pub psd_values: Vec<f64>,
    /// Classifications for each pulse.
    pub classifications: Vec<ParticleType>,
    /// Figure of merit: FOM = peak_separation / (fwhm_gamma + fwhm_neutron).
    pub figure_of_merit: f64,
}

impl PulseShapeDiscriminator {
    /// Create a new PSD analyzer.
    ///
    /// `tail_start_fraction`: where the tail starts (0.2-0.4 typical).
    /// `psd_threshold`: PSD ratio boundary (0.2-0.4 typical).
    /// `min_amplitude`: noise rejection threshold.
    pub fn new(tail_start_fraction: f64, psd_threshold: f64, min_amplitude: f64) -> Self {
        Self {
            tail_start_fraction,
            psd_threshold,
            min_amplitude,
        }
    }

    /// Compute PSD ratio for a single pulse waveform.
    ///
    /// Returns Q_tail / Q_total using the charge comparison method.
    pub fn compute_psd(&self, pulse: &[f64]) -> f64 {
        if pulse.is_empty() {
            return 0.0;
        }
        let tail_start = (pulse.len() as f64 * self.tail_start_fraction) as usize;
        let q_total: f64 = pulse.iter().map(|&x| x.abs()).sum();
        if q_total < 1e-12 {
            return 0.0;
        }
        let q_tail: f64 = pulse[tail_start..].iter().map(|&x| x.abs()).sum();
        q_tail / q_total
    }

    /// Classify a single pulse.
    pub fn classify(&self, pulse: &[f64]) -> ParticleType {
        let peak = pulse.iter().cloned().fold(0.0_f64, f64::max);
        if peak < self.min_amplitude {
            return ParticleType::Noise;
        }
        let psd = self.compute_psd(pulse);
        if psd > self.psd_threshold {
            ParticleType::Neutron
        } else {
            ParticleType::Gamma
        }
    }

    /// Analyze a batch of pulses.
    pub fn analyze(&self, pulses: &[Vec<f64>]) -> PsdResult {
        let mut gamma_count = 0;
        let mut neutron_count = 0;
        let mut noise_count = 0;
        let mut psd_values = Vec::with_capacity(pulses.len());
        let mut classifications = Vec::with_capacity(pulses.len());

        for pulse in pulses {
            let psd = self.compute_psd(pulse);
            let class = self.classify(pulse);
            psd_values.push(psd);
            classifications.push(class);
            match class {
                ParticleType::Gamma => gamma_count += 1,
                ParticleType::Neutron => neutron_count += 1,
                ParticleType::Noise => noise_count += 1,
            }
        }

        // Figure of merit: separation / (FWHM_g + FWHM_n)
        let fom = compute_figure_of_merit(&psd_values, &classifications, self.psd_threshold);

        PsdResult {
            gamma_count,
            neutron_count,
            noise_count,
            psd_values,
            classifications,
            figure_of_merit: fom,
        }
    }

    /// Generate a synthetic pulse with specified decay characteristics.
    ///
    /// `amplitude`: peak amplitude
    /// `fast_decay`: fast component decay constant (samples)
    /// `slow_fraction`: fraction of energy in slow component (0 for gamma, ~0.3 for neutron)
    /// `slow_decay`: slow component decay constant (samples)
    /// `n_samples`: total pulse length
    pub fn generate_pulse(
        amplitude: f64,
        fast_decay: f64,
        slow_fraction: f64,
        slow_decay: f64,
        n_samples: usize,
    ) -> Vec<f64> {
        let fast_fraction = 1.0 - slow_fraction;
        (0..n_samples)
            .map(|i| {
                let t = i as f64;
                amplitude
                    * (fast_fraction * (-t / fast_decay).exp()
                        + slow_fraction * (-t / slow_decay).exp())
            })
            .collect()
    }
}

/// Compute the PSD figure of merit from classified events.
fn compute_figure_of_merit(
    psd_values: &[f64],
    classifications: &[ParticleType],
    _threshold: f64,
) -> f64 {
    let gamma_psds: Vec<f64> = psd_values
        .iter()
        .zip(classifications.iter())
        .filter_map(|(&v, &c)| if c == ParticleType::Gamma { Some(v) } else { None })
        .collect();

    let neutron_psds: Vec<f64> = psd_values
        .iter()
        .zip(classifications.iter())
        .filter_map(|(&v, &c)| if c == ParticleType::Neutron { Some(v) } else { None })
        .collect();

    if gamma_psds.is_empty() || neutron_psds.is_empty() {
        return 0.0;
    }

    let mean_g = gamma_psds.iter().sum::<f64>() / gamma_psds.len() as f64;
    let mean_n = neutron_psds.iter().sum::<f64>() / neutron_psds.len() as f64;

    let var_g = gamma_psds.iter().map(|&v| (v - mean_g).powi(2)).sum::<f64>()
        / gamma_psds.len() as f64;
    let var_n = neutron_psds.iter().map(|&v| (v - mean_n).powi(2)).sum::<f64>()
        / neutron_psds.len() as f64;

    let fwhm_g = 2.355 * var_g.sqrt();
    let fwhm_n = 2.355 * var_n.sqrt();

    let separation = (mean_n - mean_g).abs();
    let denom = fwhm_g + fwhm_n;
    if denom > 1e-12 {
        separation / denom
    } else {
        0.0
    }
}

// ---------------------------------------------------------------------------
// DetectorEfficiency
// ---------------------------------------------------------------------------

/// Detector efficiency model using a power-law approximation.
///
/// For semiconductor detectors (HPGe), efficiency varies as:
///
/// eps(E) = a * E^b
///
/// where E is in keV, and a, b are calibration parameters.
#[derive(Debug, Clone)]
pub struct DetectorEfficiency {
    /// Coefficient 'a' in eps = a * E^b.
    pub coefficient: f64,
    /// Exponent 'b' in eps = a * E^b (typically negative, ~-0.5 to -1).
    pub exponent: f64,
}

impl DetectorEfficiency {
    /// Create a new efficiency model with given parameters.
    pub fn new(coefficient: f64, exponent: f64) -> Self {
        Self {
            coefficient,
            exponent,
        }
    }

    /// Typical HPGe detector (relative efficiency ~30% at 1332 keV).
    pub fn typical_hpge() -> Self {
        // Calibrated: ~1% at 100 keV, ~0.3% at 1332 keV
        Self {
            coefficient: 0.1,
            exponent: -0.7,
        }
    }

    /// Calculate absolute efficiency at a given gamma energy.
    pub fn efficiency_at(&self, energy_kev: f64) -> f64 {
        if energy_kev <= 0.0 {
            return 0.0;
        }
        (self.coefficient * energy_kev.powf(self.exponent)).min(1.0).max(0.0)
    }

    /// Calculate FWHM energy resolution for a semiconductor detector.
    ///
    /// FWHM(E) = k * sqrt(E) where k is a detector-dependent constant.
    /// For HPGe: FWHM ≈ 1.8-2.0 keV at 1332 keV → k ≈ 0.054
    pub fn fwhm_kev(energy_kev: f64, resolution_constant: f64) -> f64 {
        resolution_constant * energy_kev.sqrt()
    }
}

// ---------------------------------------------------------------------------
// NaaProcessor — high-level analysis pipeline
// ---------------------------------------------------------------------------

/// High-level NAA processing pipeline that combines all correction steps.
#[derive(Debug, Clone)]
pub struct NaaProcessor {
    /// Measurement configuration.
    pub config: NaaConfig,
    /// Isotope database.
    pub database: IsotopeDatabase,
    /// Detector efficiency model.
    pub efficiency: DetectorEfficiency,
}

/// Result of a complete NAA analysis for one element.
#[derive(Debug, Clone)]
pub struct NaaAnalysisResult {
    /// Isotope used for quantification.
    pub isotope_name: String,
    /// Measured net peak area (counts).
    pub net_peak_area: f64,
    /// Decay-corrected peak area.
    pub corrected_peak_area: f64,
    /// Detector efficiency at the gamma energy.
    pub efficiency: f64,
    /// Estimated activity at end of irradiation (Bq).
    pub activity_bq: f64,
    /// Concentration if standard data is available (ppm).
    pub concentration_ppm: Option<f64>,
}

impl NaaProcessor {
    /// Create a new NAA processor.
    pub fn new(config: NaaConfig, efficiency: DetectorEfficiency) -> Self {
        Self {
            config,
            database: IsotopeDatabase::new(),
            efficiency,
        }
    }

    /// Analyze a single peak for a known isotope.
    ///
    /// Returns the corrected activity and optionally the concentration
    /// if standard data is provided.
    pub fn analyze_peak(
        &self,
        isotope_name: &str,
        net_peak_area: f64,
        standard_data: Option<(&ComparatorInput,)>,
    ) -> Option<NaaAnalysisResult> {
        let isotope = self.database.lookup(isotope_name)?;

        // Decay correction
        let corrector = DecayCorrector::new(isotope.half_life_s);
        let corrected_area = corrector.correct_counting_loss(net_peak_area, self.config.counting_time);

        // Efficiency at this energy
        let eps = self.efficiency.efficiency_at(isotope.gamma_energy_kev);

        // Activity at start of counting
        let activity_at_count = if eps > 0.0 && self.config.counting_time > 0.0 {
            corrected_area / (eps * self.config.counting_time)
        } else {
            0.0
        };

        // Project back to end of irradiation
        let activity_eoi = corrector.correct_decay_time(activity_at_count, self.config.decay_time);

        // Concentration via comparator if standard data provided
        let concentration_ppm = standard_data.map(|(input,)| {
            let calc = ConcentrationCalculator::new(isotope.half_life_s);
            let result = calc.comparator_method(input);
            result.concentration_ppm
        });

        Some(NaaAnalysisResult {
            isotope_name: isotope_name.to_string(),
            net_peak_area,
            corrected_peak_area: corrected_area,
            efficiency: eps,
            activity_bq: activity_eoi,
            concentration_ppm,
        })
    }

    /// Identify possible isotopes from a list of peak energies.
    pub fn identify_peaks(&self, peak_energies_kev: &[f64], tolerance_kev: f64) -> Vec<Vec<&IsotopeInfo>> {
        peak_energies_kev
            .iter()
            .map(|&e| self.database.find_by_energy(e, tolerance_kev))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-6;

    // -- NaaConfig tests --

    #[test]
    fn test_naa_config_default() {
        let config = NaaConfig::default();
        assert_eq!(config.neutron_flux, 1e13);
        assert_eq!(config.irradiation_time, 3600.0);
        assert_eq!(config.decay_time, 7200.0);
        assert_eq!(config.counting_time, 3600.0);
        assert_eq!(config.detector_efficiency, 0.01);
    }

    // -- ActivationCalculator tests --

    #[test]
    fn test_decay_constant_na24() {
        let t_half = 14.96 * 3600.0; // 14.96 hours in seconds
        let lambda = ActivationCalculator::decay_constant(t_half);
        let expected = LN2 / t_half;
        assert!((lambda - expected).abs() < EPSILON);
    }

    #[test]
    fn test_saturation_factor_short_irradiation() {
        // For t_irr << t_half, S ≈ lambda * t_irr (small)
        let config = NaaConfig {
            irradiation_time: 60.0, // 1 minute
            ..NaaConfig::default()
        };
        let calc = ActivationCalculator::new(&config);
        let t_half = 5.271 * 365.25 * 86400.0; // Co-60: 5.271 years
        let s = calc.saturation_factor(t_half);
        assert!(s > 0.0 && s < 0.001); // Very small for short irradiation of long-lived isotope
    }

    #[test]
    fn test_saturation_factor_long_irradiation() {
        // For t_irr >> t_half, S approaches 1 (saturation)
        let config = NaaConfig {
            irradiation_time: 100.0 * 14.96 * 3600.0, // 100 half-lives of Na-24
            ..NaaConfig::default()
        };
        let calc = ActivationCalculator::new(&config);
        let t_half = 14.96 * 3600.0;
        let s = calc.saturation_factor(t_half);
        assert!((s - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_decay_factor_zero_time() {
        let config = NaaConfig {
            decay_time: 0.0,
            ..NaaConfig::default()
        };
        let calc = ActivationCalculator::new(&config);
        let d = calc.decay_factor(14.96 * 3600.0);
        assert!((d - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_decay_factor_one_half_life() {
        let t_half = 14.96 * 3600.0;
        let config = NaaConfig {
            decay_time: t_half,
            ..NaaConfig::default()
        };
        let calc = ActivationCalculator::new(&config);
        let d = calc.decay_factor(t_half);
        assert!((d - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_counting_factor_long_halflife() {
        // For very long half-life, C ≈ 1
        let config = NaaConfig::default();
        let calc = ActivationCalculator::new(&config);
        let t_half = 5.271 * 365.25 * 86400.0; // Co-60
        let c = calc.counting_factor(t_half);
        assert!((c - 1.0).abs() < 1e-4);
    }

    #[test]
    fn test_counting_factor_short_halflife() {
        // For short half-life, C < 1
        let config = NaaConfig {
            counting_time: 3600.0,
            ..NaaConfig::default()
        };
        let calc = ActivationCalculator::new(&config);
        let t_half = 3600.0; // 1 hour
        let c = calc.counting_factor(t_half);
        assert!(c > 0.0 && c < 1.0);
        // Exact: (1 - exp(-ln2)) / ln2 = 0.5 / ln2 ≈ 0.7213
        assert!((c - 0.5 / LN2).abs() < 1e-6);
    }

    #[test]
    fn test_activity_positive() {
        let config = NaaConfig::default();
        let calc = ActivationCalculator::new(&config);
        let n_atoms = 1e18;
        let sigma = 0.530; // Na-23 cross section in barns
        let t_half = 14.96 * 3600.0;
        let a = calc.activity(n_atoms, sigma, t_half);
        assert!(a > 0.0);
    }

    #[test]
    fn test_expected_counts_positive() {
        let config = NaaConfig::default();
        let calc = ActivationCalculator::new(&config);
        let n_atoms = 1e18;
        let counts = calc.expected_counts(n_atoms, 0.530, 14.96 * 3600.0, 0.01);
        assert!(counts > 0.0);
    }

    #[test]
    fn test_atoms_from_mass() {
        // 1 gram of Na-23 (100% abundant, AW=22.99)
        let n = ActivationCalculator::atoms_from_mass(1.0, 22.99, 1.0);
        let expected = AVOGADRO / 22.99;
        assert!((n - expected).abs() / expected < 1e-6);
    }

    #[test]
    fn test_atoms_partial_abundance() {
        // 50% abundance
        let n_full = ActivationCalculator::atoms_from_mass(1.0, 50.0, 1.0);
        let n_half = ActivationCalculator::atoms_from_mass(1.0, 50.0, 0.5);
        assert!((n_half - n_full * 0.5).abs() < 1.0);
    }

    // -- DecayCorrector tests --

    #[test]
    fn test_decay_corrector_creation() {
        let dc = DecayCorrector::new(3600.0);
        assert_eq!(dc.half_life_s, 3600.0);
        assert!((dc.lambda - LN2 / 3600.0).abs() < EPSILON);
    }

    #[test]
    fn test_counting_loss_correction_identity() {
        // For very long half-life, correction ≈ 1
        let dc = DecayCorrector::new(1e12);
        let corrected = dc.correct_counting_loss(1000.0, 3600.0);
        assert!((corrected - 1000.0).abs() < 0.1);
    }

    #[test]
    fn test_counting_loss_correction_short() {
        // For short half-life, corrected > measured
        let dc = DecayCorrector::new(3600.0);
        let corrected = dc.correct_counting_loss(1000.0, 3600.0);
        assert!(corrected > 1000.0);
    }

    #[test]
    fn test_remaining_activity_half_life() {
        let dc = DecayCorrector::new(3600.0);
        let remaining = dc.remaining_activity(1000.0, 3600.0);
        assert!((remaining - 500.0).abs() < 0.1);
    }

    #[test]
    fn test_remaining_activity_zero_time() {
        let dc = DecayCorrector::new(3600.0);
        let remaining = dc.remaining_activity(1000.0, 0.0);
        assert!((remaining - 1000.0).abs() < EPSILON);
    }

    #[test]
    fn test_correct_decay_time() {
        let dc = DecayCorrector::new(3600.0);
        // After one half-life, activity is halved; correcting back should double
        let measured_activity = 500.0;
        let original = dc.correct_decay_time(measured_activity, 3600.0);
        assert!((original - 1000.0).abs() < 0.1);
    }

    #[test]
    fn test_half_lives_elapsed() {
        let dc = DecayCorrector::new(3600.0);
        assert!((dc.half_lives_elapsed(7200.0) - 2.0).abs() < EPSILON);
        assert!((dc.half_lives_elapsed(3600.0) - 1.0).abs() < EPSILON);
    }

    // -- PeakAreaCalculator tests --

    #[test]
    fn test_model_evaluation() {
        // At centroid, Gaussian peak should equal amplitude + background
        let val = PeakAreaCalculator::model(500.0, 1000.0, 500.0, 2.0, 10.0, 0.01);
        let expected = 1000.0 + 10.0 + 0.01 * 500.0;
        assert!((val - expected).abs() < EPSILON);
    }

    #[test]
    fn test_model_away_from_peak() {
        // Far from centroid, only background remains
        let val = PeakAreaCalculator::model(0.0, 1000.0, 500.0, 2.0, 10.0, 0.01);
        let bg = 10.0 + 0.01 * 0.0;
        assert!((val - bg).abs() < 1e-3);
    }

    #[test]
    fn test_peak_fitting_synthetic() {
        let peaks = vec![(500.0, 1000.0, 3.0)];
        let (energies, counts) =
            PeakAreaCalculator::generate_spectrum(&peaks, 400.0, 600.0, 200, 50.0, 0.0);

        let result = PeakAreaCalculator::fit_peak(&energies, &counts, 470.0, 530.0);
        assert!(result.is_some());
        let fit = result.unwrap();
        assert!((fit.centroid - 500.0).abs() < 2.0);
        assert!(fit.net_area > 0.0);
        assert!(fit.fwhm > 0.0);
    }

    #[test]
    fn test_peak_fitting_too_few_points() {
        let result = PeakAreaCalculator::fit_peak(&[1.0, 2.0], &[10.0, 20.0], 0.0, 3.0);
        assert!(result.is_none());
    }

    #[test]
    fn test_generate_spectrum_background() {
        let (energies, counts) =
            PeakAreaCalculator::generate_spectrum(&[], 0.0, 1000.0, 100, 100.0, 0.0);
        assert_eq!(energies.len(), 100);
        // All counts should be ~100 (flat background)
        for &c in &counts {
            assert!((c - 100.0).abs() < 1.0);
        }
    }

    // -- ConcentrationCalculator tests --

    #[test]
    fn test_comparator_equal_conditions() {
        // Same decay time, same counting time: concentration scales with count ratio
        let calc = ConcentrationCalculator::new(14.96 * 3600.0);
        let input = ComparatorInput {
            sample_counts: 5000.0,
            standard_counts: 10000.0,
            sample_mass_g: 0.5,
            standard_mass_ug: 100.0,
            sample_decay_time_s: 86400.0,
            standard_decay_time_s: 86400.0,
            sample_count_time_s: 3600.0,
            standard_count_time_s: 3600.0,
        };
        let result = calc.comparator_method(&input);
        // C = (5000/10000) * (100/0.5) * 1.0 * 1.0 = 100 ppm
        assert!((result.concentration_ppm - 100.0).abs() < EPSILON);
        assert!((result.decay_correction_factor - 1.0).abs() < EPSILON);
        assert!((result.counting_correction_factor - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_comparator_different_decay() {
        let calc = ConcentrationCalculator::new(14.96 * 3600.0);
        let input = ComparatorInput {
            sample_counts: 5000.0,
            standard_counts: 5000.0,
            sample_mass_g: 1.0,
            standard_mass_ug: 100.0,
            sample_decay_time_s: 86400.0,
            standard_decay_time_s: 43200.0, // Half the decay time
            sample_count_time_s: 3600.0,
            standard_count_time_s: 3600.0,
        };
        let result = calc.comparator_method(&input);
        // Standard decayed less (shorter decay time), so D_standard > D_sample
        // Decay correction factor = D_standard / D_sample > 1
        assert!(result.decay_correction_factor > 1.0);
    }

    #[test]
    fn test_k0_method_positive() {
        let calc = ConcentrationCalculator::new(14.96 * 3600.0);
        let conc = calc.k0_method(
            10000.0,     // sample counts
            0.5,         // sample mass
            1.5,         // k0 factor
            5000.0,      // monitor counts
            0.001,       // monitor mass (1 mg Au)
            2.696 * 86400.0, // Au-198 half-life
            86400.0,     // sample decay time
            86400.0,     // monitor decay time
            3600.0,      // counting time
        );
        assert!(conc > 0.0);
    }

    // -- InterferenceCorrector tests --

    #[test]
    fn test_no_interference() {
        let corrector = InterferenceCorrector::new();
        let corrected = corrector.correct_area(1000.0, 500.0, &[], 2.0);
        assert!((corrected - 1000.0).abs() < EPSILON);
    }

    #[test]
    fn test_with_interference() {
        let mut corrector = InterferenceCorrector::new();
        corrector.add_interference("Fe-59", 1099.2, 1100.0, 0.1);
        // Fe-59 primary at 1099.2, interfering with target at 1100.0
        let interferents = vec![("Fe-59", 5000.0)]; // Fe-59 has 5000 counts at primary
        let corrected = corrector.correct_area(1500.0, 1100.0, &interferents, 2.0);
        // Correction: 1500 - 0.1 * 5000 = 1000
        assert!((corrected - 1000.0).abs() < EPSILON);
    }

    #[test]
    fn test_interference_floor_at_zero() {
        let mut corrector = InterferenceCorrector::new();
        corrector.add_interference("Fe-59", 1099.2, 1100.0, 0.5);
        let interferents = vec![("Fe-59", 5000.0)];
        // Correction: 1000 - 0.5 * 5000 = -1500, floored to 0
        let corrected = corrector.correct_area(1000.0, 1100.0, &interferents, 2.0);
        assert!((corrected - 0.0).abs() < EPSILON);
    }

    #[test]
    fn test_interference_default() {
        let corrector = InterferenceCorrector::default();
        assert_eq!(corrector.num_interferences(), 0);
    }

    // -- SelfShieldingCorrector tests --

    #[test]
    fn test_slab_no_shielding() {
        // Zero cross section = no shielding
        let f = SelfShieldingCorrector::slab_factor(0.0, 1.0);
        assert!((f - 1.0).abs() < 1e-8);
    }

    #[test]
    fn test_slab_moderate_shielding() {
        // f = (1 - exp(-1)) / 1 = 0.6321
        let f = SelfShieldingCorrector::slab_factor(1.0, 1.0);
        let expected = (1.0 - (-1.0_f64).exp()) / 1.0;
        assert!((f - expected).abs() < EPSILON);
    }

    #[test]
    fn test_slab_strong_shielding() {
        let f = SelfShieldingCorrector::slab_factor(10.0, 1.0);
        assert!(f < 0.2); // Strong shielding
        assert!(f > 0.0);
    }

    #[test]
    fn test_cylinder_no_shielding() {
        let f = SelfShieldingCorrector::cylinder_factor(0.0, 1.0);
        assert!((f - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_cylinder_moderate() {
        let f = SelfShieldingCorrector::cylinder_factor(0.5, 1.0);
        assert!(f > 0.0 && f < 1.0);
    }

    #[test]
    fn test_macroscopic_cross_section() {
        // Gold: rho=19.3 g/cm3, AW=197, sigma=98.65 barn
        let sigma_macro = SelfShieldingCorrector::macroscopic_cross_section(19.3, 197.0, 98.65);
        // Expected: (19.3 * 6.022e23 / 197) * 98.65e-24 ≈ 5.82 cm^-1
        assert!(sigma_macro > 5.0 && sigma_macro < 7.0);
    }

    // -- CoincidenceSumCorrector tests --

    #[test]
    fn test_no_cascades() {
        let corrector = CoincidenceSumCorrector::new();
        let factor = corrector.correction_factor(1332.5, |_| 0.01);
        assert!((factor - 1.0).abs() < EPSILON);
    }

    #[test]
    fn test_cascade_correction() {
        let mut corrector = CoincidenceSumCorrector::new();
        // Co-60: 1173.2 keV and 1332.5 keV in cascade
        corrector.add_cascade(1173.2, 1332.5, 1.0);

        // Correction for 1173.2 keV line
        let factor = corrector.correction_factor(1173.2, |_| 0.05);
        // factor = 1 / (1 - 1.0 * 0.05) = 1/0.95 ≈ 1.053
        assert!((factor - 1.0 / 0.95).abs() < 0.01);
    }

    #[test]
    fn test_coincidence_default() {
        let corrector = CoincidenceSumCorrector::default();
        assert_eq!(corrector.num_cascades(), 0);
    }

    // -- IsotopeDatabase tests --

    #[test]
    fn test_database_creation() {
        let db = IsotopeDatabase::new();
        assert!(db.len() >= 10);
        assert!(!db.is_empty());
    }

    #[test]
    fn test_lookup_na24() {
        let db = IsotopeDatabase::new();
        let iso = db.lookup("Na-24").unwrap();
        assert_eq!(iso.element, "Na");
        assert!((iso.gamma_energy_kev - 1368.6).abs() < 0.1);
        assert!((iso.half_life_s - 14.96 * 3600.0).abs() < 1.0);
    }

    #[test]
    fn test_lookup_co60() {
        let db = IsotopeDatabase::new();
        let iso = db.lookup("Co-60").unwrap();
        assert_eq!(iso.element, "Co");
        assert!((iso.gamma_energy_kev - 1332.5).abs() < 0.1);
        assert!(iso.thermal_cross_section_barn > 30.0);
    }

    #[test]
    fn test_lookup_nonexistent() {
        let db = IsotopeDatabase::new();
        assert!(db.lookup("Unobtainium-99").is_none());
    }

    #[test]
    fn test_lookup_by_element() {
        let db = IsotopeDatabase::new();
        let na_isotopes = db.lookup_by_element("Na");
        assert!(!na_isotopes.is_empty());
        assert_eq!(na_isotopes[0].element, "Na");
    }

    #[test]
    fn test_find_by_energy() {
        let db = IsotopeDatabase::new();
        // Na-24: 1368.6 keV
        let matches = db.find_by_energy(1368.6, 5.0);
        assert!(!matches.is_empty());
        assert!(matches.iter().any(|iso| iso.name == "Na-24"));
    }

    #[test]
    fn test_database_default() {
        let db = IsotopeDatabase::default();
        assert!(db.len() > 0);
    }

    // -- PulseShapeDiscriminator tests --

    #[test]
    fn test_psd_gamma_pulse() {
        let psd = PulseShapeDiscriminator::new(0.3, 0.25, 0.1);
        // Gamma: fast decay only
        let pulse = PulseShapeDiscriminator::generate_pulse(1.0, 5.0, 0.0, 50.0, 100);
        let ratio = psd.compute_psd(&pulse);
        assert!(ratio < 0.25); // Should be classified as gamma
    }

    #[test]
    fn test_psd_neutron_pulse() {
        let psd = PulseShapeDiscriminator::new(0.3, 0.25, 0.1);
        // Neutron: significant slow component
        let pulse = PulseShapeDiscriminator::generate_pulse(1.0, 5.0, 0.4, 50.0, 100);
        let ratio = psd.compute_psd(&pulse);
        assert!(ratio > 0.25); // Should be classified as neutron
    }

    #[test]
    fn test_psd_classify_noise() {
        let psd = PulseShapeDiscriminator::new(0.3, 0.25, 0.5);
        let pulse = vec![0.01; 100]; // Very low amplitude
        assert_eq!(psd.classify(&pulse), ParticleType::Noise);
    }

    #[test]
    fn test_psd_classify_gamma() {
        let psd = PulseShapeDiscriminator::new(0.3, 0.25, 0.1);
        let pulse = PulseShapeDiscriminator::generate_pulse(1.0, 5.0, 0.0, 50.0, 100);
        assert_eq!(psd.classify(&pulse), ParticleType::Gamma);
    }

    #[test]
    fn test_psd_classify_neutron() {
        let psd = PulseShapeDiscriminator::new(0.3, 0.25, 0.1);
        let pulse = PulseShapeDiscriminator::generate_pulse(1.0, 5.0, 0.4, 50.0, 100);
        assert_eq!(psd.classify(&pulse), ParticleType::Neutron);
    }

    #[test]
    fn test_psd_batch_analysis() {
        let psd = PulseShapeDiscriminator::new(0.3, 0.25, 0.1);
        let gamma = PulseShapeDiscriminator::generate_pulse(1.0, 5.0, 0.0, 50.0, 100);
        let neutron = PulseShapeDiscriminator::generate_pulse(1.0, 5.0, 0.4, 50.0, 100);
        let noise = vec![0.01; 100];

        let pulses = vec![gamma, neutron.clone(), neutron, noise];
        let result = psd.analyze(&pulses);
        assert_eq!(result.gamma_count, 1);
        assert_eq!(result.neutron_count, 2);
        assert_eq!(result.noise_count, 1);
        assert_eq!(result.psd_values.len(), 4);
    }

    #[test]
    fn test_psd_empty_pulse() {
        let psd = PulseShapeDiscriminator::new(0.3, 0.25, 0.1);
        let ratio = psd.compute_psd(&[]);
        assert!((ratio - 0.0).abs() < EPSILON);
    }

    // -- DetectorEfficiency tests --

    #[test]
    fn test_efficiency_typical_hpge() {
        let eff = DetectorEfficiency::typical_hpge();
        let e100 = eff.efficiency_at(100.0);
        let e1000 = eff.efficiency_at(1000.0);
        // Efficiency should decrease with energy
        assert!(e100 > e1000);
        assert!(e100 > 0.0 && e100 <= 1.0);
    }

    #[test]
    fn test_efficiency_zero_energy() {
        let eff = DetectorEfficiency::typical_hpge();
        assert_eq!(eff.efficiency_at(0.0), 0.0);
        assert_eq!(eff.efficiency_at(-100.0), 0.0);
    }

    #[test]
    fn test_fwhm_increases_with_energy() {
        let fwhm_low = DetectorEfficiency::fwhm_kev(100.0, 0.054);
        let fwhm_high = DetectorEfficiency::fwhm_kev(1000.0, 0.054);
        assert!(fwhm_high > fwhm_low);
    }

    // -- NaaProcessor tests --

    #[test]
    fn test_processor_analyze_peak() {
        let config = NaaConfig::default();
        let eff = DetectorEfficiency::typical_hpge();
        let proc = NaaProcessor::new(config, eff);
        let result = proc.analyze_peak("Na-24", 10000.0, None);
        assert!(result.is_some());
        let r = result.unwrap();
        assert_eq!(r.isotope_name, "Na-24");
        assert!(r.corrected_peak_area >= r.net_peak_area);
        assert!(r.activity_bq > 0.0);
        assert!(r.concentration_ppm.is_none());
    }

    #[test]
    fn test_processor_unknown_isotope() {
        let config = NaaConfig::default();
        let eff = DetectorEfficiency::typical_hpge();
        let proc = NaaProcessor::new(config, eff);
        let result = proc.analyze_peak("Fake-99", 1000.0, None);
        assert!(result.is_none());
    }

    #[test]
    fn test_identify_peaks() {
        let config = NaaConfig::default();
        let eff = DetectorEfficiency::typical_hpge();
        let proc = NaaProcessor::new(config, eff);
        let identifications = proc.identify_peaks(&[1368.6, 846.8], 5.0);
        assert_eq!(identifications.len(), 2);
        assert!(!identifications[0].is_empty()); // Na-24
        assert!(!identifications[1].is_empty()); // Mn-56
    }

    // -- Sphere self-shielding --

    #[test]
    fn test_sphere_no_shielding() {
        let f = SelfShieldingCorrector::sphere_factor(0.0, 1.0);
        assert!((f - 1.0).abs() < 1e-4);
    }

    // -- Integration: full NAA workflow --

    #[test]
    fn test_full_naa_workflow() {
        // Simulate a complete NAA measurement for sodium in a geological sample
        let config = NaaConfig {
            neutron_flux: 1e13,
            irradiation_time: 3600.0,
            decay_time: 86400.0,
            counting_time: 3600.0,
            detector_efficiency: 0.01,
        };

        let db = IsotopeDatabase::new();
        let na24 = db.lookup("Na-24").unwrap();

        // 1. Calculate expected activity for 1 mg Na in 1 g sample
        let calc = ActivationCalculator::new(&config);
        let n_atoms = ActivationCalculator::atoms_from_mass(1e-3, 22.99, 1.0);
        let activity = calc.activity(n_atoms, na24.thermal_cross_section_barn, na24.half_life_s);
        assert!(activity > 0.0);

        // 2. Expected counts
        let counts = calc.expected_counts(n_atoms, na24.thermal_cross_section_barn, na24.half_life_s, 0.01);
        assert!(counts > 0.0);

        // 3. Decay correction
        let corrector = DecayCorrector::new(na24.half_life_s);
        let corrected = corrector.correct_counting_loss(counts, config.counting_time);
        assert!(corrected >= counts);

        // 4. Self-shielding (thin sample, negligible)
        let f = SelfShieldingCorrector::slab_factor(0.01, 0.1);
        assert!(f > 0.99);
    }
}
