//! # Auger Electron Spectroscopy (AES) Analyzer
//!
//! Implements AES data analysis for surface elemental composition using Auger
//! electron kinetic energy spectra, including derivative spectra, quantification,
//! and depth profiling.
//!
//! ## Physics
//!
//! The Auger process involves three steps:
//! 1. A core hole is created (e.g., by electron bombardment)
//! 2. An outer electron fills the core hole
//! 3. The released energy ejects an Auger electron with KE = E_A - E_B - E_C
//!
//! KLL notation means: core hole in K shell, L electrons involved in transition.
//! Surface sensitivity arises from the short inelastic mean free path (IMFP) of
//! 0.5-5 nm depending on element and kinetic energy.
//!
//! Peak-to-peak height in the dN/dE derivative spectrum is proportional to
//! elemental concentration at the surface.

use std::f64::consts::PI;

// ─────────────────────────────────── AesSpectrum ─────────────────────────────

/// Kinetic energy spectrum N(E) or E*N(E).
#[derive(Debug, Clone)]
pub struct AesSpectrum {
    /// Kinetic energy axis in eV.
    pub kinetic_energy_ev: Vec<f64>,
    /// Counts (intensity) at each energy.
    pub counts: Vec<f64>,
}

impl AesSpectrum {
    /// Create a new AES spectrum from energy and count vectors.
    ///
    /// # Panics
    /// Panics if the vectors have different lengths or are empty.
    pub fn new(kinetic_energy_ev: Vec<f64>, counts: Vec<f64>) -> Self {
        assert!(
            !kinetic_energy_ev.is_empty(),
            "Energy vector must not be empty"
        );
        assert_eq!(
            kinetic_energy_ev.len(),
            counts.len(),
            "Energy and counts must have the same length"
        );
        Self {
            kinetic_energy_ev,
            counts,
        }
    }

    /// Return the (min, max) energy range in eV.
    pub fn energy_range(&self) -> (f64, f64) {
        let mut min = self.kinetic_energy_ev[0];
        let mut max = self.kinetic_energy_ev[0];
        for &e in &self.kinetic_energy_ev {
            if e < min {
                min = e;
            }
            if e > max {
                max = e;
            }
        }
        (min, max)
    }

    /// Return E*N(E) spectrum (multiply counts by energy).
    pub fn multiply_by_energy(&self) -> AesSpectrum {
        let counts: Vec<f64> = self
            .kinetic_energy_ev
            .iter()
            .zip(self.counts.iter())
            .map(|(e, c)| e * c)
            .collect();
        AesSpectrum {
            kinetic_energy_ev: self.kinetic_energy_ev.clone(),
            counts,
        }
    }

    /// Savitzky-Golay-style smoothing using a moving average of given window size.
    /// Window must be odd and >= 3. If even, it is incremented by 1.
    pub fn smooth(&self, mut window: usize) -> AesSpectrum {
        if window < 3 {
            window = 3;
        }
        if window % 2 == 0 {
            window += 1;
        }
        let half = window / 2;
        let n = self.counts.len();
        let mut smoothed = vec![0.0; n];
        for i in 0..n {
            let lo = if i >= half { i - half } else { 0 };
            let hi = if i + half < n { i + half } else { n - 1 };
            let mut sum = 0.0;
            let mut count = 0.0;
            for j in lo..=hi {
                sum += self.counts[j];
                count += 1.0;
            }
            smoothed[i] = sum / count;
        }
        AesSpectrum {
            kinetic_energy_ev: self.kinetic_energy_ev.clone(),
            counts: smoothed,
        }
    }

    /// Number of data points.
    pub fn len(&self) -> usize {
        self.kinetic_energy_ev.len()
    }

    /// Whether the spectrum is empty.
    pub fn is_empty(&self) -> bool {
        self.kinetic_energy_ev.is_empty()
    }
}

// ─────────────────────────────── AugerPeak ───────────────────────────────────

/// A detected Auger peak.
#[derive(Debug, Clone)]
pub struct AugerPeak {
    /// Kinetic energy of the peak in eV.
    pub kinetic_energy_ev: f64,
    /// Peak-to-peak height in derivative spectrum.
    pub peak_to_peak: f64,
    /// Identified element (if any).
    pub element: Option<String>,
    /// Auger transition label (e.g., "KLL", "LMM").
    pub transition: Option<String>,
}

// ─────────────────────────────── DerivativeSpectrum ──────────────────────────

/// Derivative spectrum computation and peak analysis.
pub struct DerivativeSpectrum;

impl DerivativeSpectrum {
    /// Compute numerical derivative dN/dE using central differences.
    ///
    /// `point_spacing` controls the number of points used for the finite
    /// difference (must be >= 1). A spacing of 1 uses adjacent points.
    pub fn compute(spectrum: &AesSpectrum, point_spacing: usize) -> AesSpectrum {
        let n = spectrum.counts.len();
        let sp = if point_spacing < 1 { 1 } else { point_spacing };
        let mut derivative = vec![0.0; n];

        for i in 0..n {
            let lo = if i >= sp { i - sp } else { 0 };
            let hi = if i + sp < n { i + sp } else { n - 1 };
            let de = spectrum.kinetic_energy_ev[hi] - spectrum.kinetic_energy_ev[lo];
            if de.abs() > 1e-15 {
                derivative[i] = (spectrum.counts[hi] - spectrum.counts[lo]) / de;
            }
        }

        AesSpectrum {
            kinetic_energy_ev: spectrum.kinetic_energy_ev.clone(),
            counts: derivative,
        }
    }

    /// Measure peak-to-peak height in a derivative spectrum within an energy window.
    pub fn peak_to_peak_height(
        derivative: &AesSpectrum,
        energy_min: f64,
        energy_max: f64,
    ) -> f64 {
        let mut max_val = f64::NEG_INFINITY;
        let mut min_val = f64::INFINITY;

        for (i, &e) in derivative.kinetic_energy_ev.iter().enumerate() {
            if e >= energy_min && e <= energy_max {
                if derivative.counts[i] > max_val {
                    max_val = derivative.counts[i];
                }
                if derivative.counts[i] < min_val {
                    min_val = derivative.counts[i];
                }
            }
        }

        if max_val == f64::NEG_INFINITY || min_val == f64::INFINITY {
            return 0.0;
        }
        max_val - min_val
    }

    /// Find peaks in a derivative spectrum above a threshold.
    ///
    /// A peak is identified as a zero-crossing in the derivative (from positive
    /// to negative), which corresponds to a local maximum in the original dN/dE.
    /// The peak-to-peak is measured within a local window around each crossing.
    pub fn find_peaks(derivative: &AesSpectrum, threshold: f64) -> Vec<AugerPeak> {
        let n = derivative.counts.len();
        let mut peaks = Vec::new();

        // Look for positive-to-negative zero-crossings
        for i in 1..n {
            if derivative.counts[i - 1] > 0.0 && derivative.counts[i] <= 0.0 {
                // Interpolate the crossing energy
                let frac = if (derivative.counts[i - 1] - derivative.counts[i]).abs() > 1e-15 {
                    derivative.counts[i - 1]
                        / (derivative.counts[i - 1] - derivative.counts[i])
                } else {
                    0.5
                };
                let energy = derivative.kinetic_energy_ev[i - 1]
                    + frac
                        * (derivative.kinetic_energy_ev[i]
                            - derivative.kinetic_energy_ev[i - 1]);

                // Measure local peak-to-peak within a window
                let window = 10.min(n / 4).max(2);
                let lo = if i >= window { i - window } else { 0 };
                let hi = if i + window < n { i + window } else { n - 1 };
                let mut local_max = f64::NEG_INFINITY;
                let mut local_min = f64::INFINITY;
                for j in lo..=hi {
                    if derivative.counts[j] > local_max {
                        local_max = derivative.counts[j];
                    }
                    if derivative.counts[j] < local_min {
                        local_min = derivative.counts[j];
                    }
                }
                let p2p = local_max - local_min;

                if p2p >= threshold {
                    // Try to identify the element
                    let matches = ElementIdentifier::identify(energy, 10.0);
                    let (element, transition) = if let Some(m) = matches.first() {
                        (Some(m.element.clone()), Some(m.transition.clone()))
                    } else {
                        (None, None)
                    };

                    peaks.push(AugerPeak {
                        kinetic_energy_ev: energy,
                        peak_to_peak: p2p,
                        element,
                        transition,
                    });
                }
            }
        }

        peaks
    }
}

// ─────────────────────────────── ElementMatch ────────────────────────────────

/// A candidate element match from the Auger transition database.
#[derive(Debug, Clone)]
pub struct ElementMatch {
    /// Element symbol.
    pub element: String,
    /// Transition label (e.g., "KLL").
    pub transition: String,
    /// Reference kinetic energy in eV.
    pub reference_energy_ev: f64,
    /// Difference from measured energy in eV.
    pub delta_ev: f64,
}

// ─────────────────────────── Auger transition database ───────────────────────

/// An entry in the Auger transition database.
struct AugerTransition {
    element: &'static str,
    transition: &'static str,
    energy_ev: f64,
}

const AUGER_DATABASE: &[AugerTransition] = &[
    AugerTransition { element: "C",  transition: "KLL",  energy_ev: 272.0 },
    AugerTransition { element: "N",  transition: "KLL",  energy_ev: 379.0 },
    AugerTransition { element: "O",  transition: "KLL",  energy_ev: 510.0 },
    AugerTransition { element: "P",  transition: "LMM",  energy_ev: 120.0 },
    AugerTransition { element: "S",  transition: "LMM",  energy_ev: 152.0 },
    AugerTransition { element: "Si", transition: "LMM",  energy_ev: 92.0 },
    AugerTransition { element: "Ti", transition: "LMM",  energy_ev: 418.0 },
    AugerTransition { element: "Cr", transition: "LMM",  energy_ev: 529.0 },
    AugerTransition { element: "Fe", transition: "LMM",  energy_ev: 703.0 },
    AugerTransition { element: "Ni", transition: "LMM",  energy_ev: 848.0 },
    AugerTransition { element: "Cu", transition: "LMM",  energy_ev: 920.0 },
    AugerTransition { element: "Zn", transition: "LMM",  energy_ev: 994.0 },
    AugerTransition { element: "Ag", transition: "MNN",  energy_ev: 351.0 },
    AugerTransition { element: "Al", transition: "KLL",  energy_ev: 1396.0 },
    AugerTransition { element: "Au", transition: "MNN",  energy_ev: 2024.0 },
    AugerTransition { element: "Na", transition: "KLL",  energy_ev: 990.0 },
    AugerTransition { element: "Mg", transition: "KLL",  energy_ev: 1186.0 },
    AugerTransition { element: "Ca", transition: "LMM",  energy_ev: 291.0 },
    AugerTransition { element: "Mn", transition: "LMM",  energy_ev: 589.0 },
    AugerTransition { element: "Co", transition: "LMM",  energy_ev: 775.0 },
    AugerTransition { element: "Ga", transition: "LMM",  energy_ev: 1070.0 },
    AugerTransition { element: "Ge", transition: "LMM",  energy_ev: 1147.0 },
    AugerTransition { element: "As", transition: "LMM",  energy_ev: 1228.0 },
    AugerTransition { element: "Se", transition: "LMM",  energy_ev: 1315.0 },
    AugerTransition { element: "Mo", transition: "MNN",  energy_ev: 186.0 },
    AugerTransition { element: "W",  transition: "MNN",  energy_ev: 1736.0 },
    AugerTransition { element: "Pt", transition: "MNN",  energy_ev: 1967.0 },
    AugerTransition { element: "Pd", transition: "MNN",  energy_ev: 330.0 },
    AugerTransition { element: "Sn", transition: "MNN",  energy_ev: 430.0 },
    AugerTransition { element: "Ta", transition: "MNN",  energy_ev: 1680.0 },
];

// ─────────────────────────────── ElementIdentifier ───────────────────────────

/// Identify elements from Auger kinetic energies using a built-in database.
pub struct ElementIdentifier;

impl ElementIdentifier {
    /// Find candidate elements matching a measured kinetic energy within tolerance.
    pub fn identify(kinetic_energy_ev: f64, tolerance_ev: f64) -> Vec<ElementMatch> {
        let mut matches = Vec::new();
        for t in AUGER_DATABASE {
            let delta = (kinetic_energy_ev - t.energy_ev).abs();
            if delta <= tolerance_ev {
                matches.push(ElementMatch {
                    element: t.element.to_string(),
                    transition: t.transition.to_string(),
                    reference_energy_ev: t.energy_ev,
                    delta_ev: delta,
                });
            }
        }
        // Sort by closest match
        matches.sort_by(|a, b| a.delta_ev.partial_cmp(&b.delta_ev).unwrap());
        matches
    }

    /// Look up the Auger energy for a specific element and transition.
    pub fn auger_energy(element: &str, transition: &str) -> Option<f64> {
        for t in AUGER_DATABASE {
            if t.element == element && t.transition == transition {
                return Some(t.energy_ev);
            }
        }
        None
    }

    /// Return all known transitions for a given element.
    pub fn all_transitions(element: &str) -> Vec<(String, f64)> {
        let mut result = Vec::new();
        for t in AUGER_DATABASE {
            if t.element == element {
                result.push((t.transition.to_string(), t.energy_ev));
            }
        }
        result
    }
}

// ─────────────────────────────── Quantification ──────────────────────────────

/// Atomic composition quantification from Auger peak intensities.
pub struct Quantification;

/// Relative sensitivity factors (relative to Ag MNN = 1.0).
const SENSITIVITY_FACTORS: &[(&str, f64)] = &[
    ("C",  0.14),
    ("N",  0.25),
    ("O",  0.40),
    ("Si", 0.35),
    ("Ti", 0.42),
    ("Cr", 0.30),
    ("Fe", 0.21),
    ("Ni", 0.27),
    ("Cu", 0.22),
    ("Zn", 0.25),
    ("Ag", 1.00),
    ("Al", 0.09),
    ("Au", 0.60),
    ("S",  0.62),
    ("P",  0.39),
    ("Na", 0.05),
    ("Mg", 0.07),
    ("Ca", 0.38),
    ("Mn", 0.28),
    ("Co", 0.24),
    ("Ga", 0.35),
    ("Ge", 0.32),
    ("As", 0.30),
    ("Se", 0.33),
    ("Mo", 0.50),
    ("W",  0.45),
    ("Pt", 0.55),
    ("Pd", 0.90),
    ("Sn", 0.55),
    ("Ta", 0.40),
];

impl Quantification {
    /// Calculate atomic percent from peak heights and sensitivity factors.
    ///
    /// atomic_i = (I_i / S_i) / sum(I_j / S_j) * 100
    pub fn atomic_percent(peak_heights: &[f64], sensitivity_factors: &[f64]) -> Vec<f64> {
        assert_eq!(peak_heights.len(), sensitivity_factors.len());
        let ratios: Vec<f64> = peak_heights
            .iter()
            .zip(sensitivity_factors.iter())
            .map(|(&h, &s)| if s.abs() > 1e-15 { h / s } else { 0.0 })
            .collect();
        let total: f64 = ratios.iter().sum();
        if total.abs() < 1e-15 {
            return vec![0.0; peak_heights.len()];
        }
        ratios.iter().map(|r| r / total * 100.0).collect()
    }

    /// Look up the relative sensitivity factor for an element.
    pub fn sensitivity_factor(element: &str) -> f64 {
        for &(el, sf) in SENSITIVITY_FACTORS {
            if el == element {
                return sf;
            }
        }
        1.0 // default if unknown
    }

    /// Apply backscatter correction factors to a composition estimate.
    ///
    /// Uses a simplified matrix correction where heavier elements have
    /// higher backscatter contributions. The correction is:
    /// C_corr_i = C_raw_i * r_i / sum(C_raw_j * r_j)
    /// where r is an approximate backscatter factor ~ 1 + 0.005 * Z.
    pub fn matrix_correction(composition: &[f64], elements: &[&str]) -> Vec<f64> {
        assert_eq!(composition.len(), elements.len());
        let factors: Vec<f64> = elements
            .iter()
            .map(|el| {
                let z = approximate_atomic_number(el);
                1.0 + 0.005 * z as f64
            })
            .collect();
        let weighted: Vec<f64> = composition
            .iter()
            .zip(factors.iter())
            .map(|(&c, &f)| c * f)
            .collect();
        let total: f64 = weighted.iter().sum();
        if total.abs() < 1e-15 {
            return vec![0.0; composition.len()];
        }
        weighted.iter().map(|w| w / total * 100.0).collect()
    }

    /// Estimate detection limit in atomic percent.
    ///
    /// detection_limit = 3 * noise_level / sensitivity (at% for 3-sigma)
    pub fn detection_limit(noise_level: f64, sensitivity: f64) -> f64 {
        if sensitivity.abs() < 1e-15 {
            return f64::INFINITY;
        }
        3.0 * noise_level / sensitivity
    }
}

/// Approximate atomic number from element symbol.
fn approximate_atomic_number(element: &str) -> u32 {
    match element {
        "H" => 1, "He" => 2, "Li" => 3, "Be" => 4, "B" => 5,
        "C" => 6, "N" => 7, "O" => 8, "F" => 9, "Ne" => 10,
        "Na" => 11, "Mg" => 12, "Al" => 13, "Si" => 14, "P" => 15,
        "S" => 16, "Cl" => 17, "Ar" => 18, "K" => 19, "Ca" => 20,
        "Ti" => 22, "Cr" => 24, "Mn" => 25, "Fe" => 26, "Co" => 27,
        "Ni" => 28, "Cu" => 29, "Zn" => 30, "Ga" => 31, "Ge" => 32,
        "As" => 33, "Se" => 34, "Mo" => 42, "Pd" => 46, "Ag" => 47,
        "Sn" => 50, "Ta" => 73, "W" => 74, "Pt" => 78, "Au" => 79,
        _ => 30, // default mid-range
    }
}

// ─────────────────────────── AugerParameterCalculator ────────────────────────

/// Modified Auger parameter calculation and Wagner plot analysis.
pub struct AugerParameterCalculator;

impl AugerParameterCalculator {
    /// Compute the modified Auger parameter alpha'.
    ///
    /// alpha' = KE_Auger + BE_XPS (in eV)
    ///
    /// This is independent of charging and is used to distinguish chemical states.
    pub fn auger_parameter(auger_ke: f64, photoelectron_be: f64) -> f64 {
        auger_ke + photoelectron_be
    }

    /// Compute extra-atomic relaxation energy from Auger parameter shift.
    ///
    /// Delta_R_ea = (alpha'_compound - alpha'_metal) / 2
    pub fn extra_atomic_relaxation(alpha_metal: f64, alpha_compound: f64) -> f64 {
        (alpha_compound - alpha_metal) / 2.0
    }

    /// Wagner plot coordinates: (x = BE_XPS, y = KE_Auger).
    pub fn wagner_plot_position(auger_ke: f64, xps_be: f64) -> (f64, f64) {
        (xps_be, auger_ke)
    }
}

// ─────────────────────────────── DepthProfile ────────────────────────────────

/// Sputter depth profiling analysis.
#[derive(Debug, Clone)]
pub struct DepthProfile {
    /// Sputter times in minutes.
    pub sputter_times_min: Vec<f64>,
    /// Composition at each time: compositions[time_idx][element_idx] in at%.
    pub compositions: Vec<Vec<f64>>,
}

impl DepthProfile {
    /// Create a new depth profile.
    pub fn new(sputter_times_min: Vec<f64>, compositions: Vec<Vec<f64>>) -> Self {
        assert_eq!(sputter_times_min.len(), compositions.len());
        Self {
            sputter_times_min,
            compositions,
        }
    }

    /// Convert sputter time to depth using a constant sputter rate.
    pub fn depth_from_rate(time_min: f64, rate_nm_per_min: f64) -> f64 {
        time_min * rate_nm_per_min
    }

    /// Find the interface position (depth where profile crosses threshold).
    ///
    /// Uses linear interpolation between adjacent data points.
    pub fn interface_position(profile: &[f64], depths: &[f64], threshold: f64) -> f64 {
        assert_eq!(profile.len(), depths.len());
        for i in 1..profile.len() {
            let (p0, p1) = (profile[i - 1], profile[i]);
            // Check for crossing in either direction
            if (p0 >= threshold && p1 < threshold) || (p0 < threshold && p1 >= threshold) {
                let frac = (threshold - p0) / (p1 - p0);
                return depths[i - 1] + frac * (depths[i] - depths[i - 1]);
            }
        }
        // If no crossing found, return last depth
        *depths.last().unwrap_or(&0.0)
    }

    /// Layer thickness using the 50% criterion (distance between 50% rise and fall).
    pub fn layer_thickness(profile: &[f64], depths: &[f64]) -> f64 {
        assert_eq!(profile.len(), depths.len());
        let max_val = profile.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let threshold = max_val * 0.5;

        let mut first_cross = None;
        let mut last_cross = None;

        for i in 1..profile.len() {
            let (p0, p1) = (profile[i - 1], profile[i]);
            if (p0 < threshold && p1 >= threshold) || (p0 >= threshold && p1 < threshold) {
                let frac = (threshold - p0) / (p1 - p0);
                let d = depths[i - 1] + frac * (depths[i] - depths[i - 1]);
                if first_cross.is_none() {
                    first_cross = Some(d);
                }
                last_cross = Some(d);
            }
        }

        match (first_cross, last_cross) {
            (Some(f), Some(l)) if (l - f).abs() > 1e-15 => (l - f).abs(),
            _ => 0.0,
        }
    }

    /// Mixing length: distance between 16% and 84% of max (one standard deviation).
    pub fn mixing_length(profile: &[f64], depths: &[f64]) -> f64 {
        assert_eq!(profile.len(), depths.len());
        let max_val = profile.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
        let low_thresh = max_val * 0.16;
        let high_thresh = max_val * 0.84;

        let d_low = Self::interface_position(profile, depths, low_thresh);
        let d_high = Self::interface_position(profile, depths, high_thresh);
        (d_high - d_low).abs()
    }
}

// ─────────────────────────── BackgroundSubtraction ───────────────────────────

/// Background subtraction methods for AES spectra.
pub struct BackgroundSubtraction;

impl BackgroundSubtraction {
    /// Linear background subtraction between two energy endpoints.
    pub fn linear_background(
        spectrum: &AesSpectrum,
        e_start: f64,
        e_end: f64,
    ) -> Vec<f64> {
        let n = spectrum.kinetic_energy_ev.len();
        // Find indices closest to e_start and e_end
        let i_start = Self::closest_index(&spectrum.kinetic_energy_ev, e_start);
        let i_end = Self::closest_index(&spectrum.kinetic_energy_ev, e_end);
        let c_start = spectrum.counts[i_start];
        let c_end = spectrum.counts[i_end];
        let e_s = spectrum.kinetic_energy_ev[i_start];
        let e_e = spectrum.kinetic_energy_ev[i_end];
        let de = e_e - e_s;

        let mut bg = vec![0.0; n];
        for i in 0..n {
            let e = spectrum.kinetic_energy_ev[i];
            if de.abs() > 1e-15 {
                let frac = (e - e_s) / de;
                bg[i] = c_start + frac * (c_end - c_start);
            } else {
                bg[i] = c_start;
            }
        }
        bg
    }

    /// Shirley (iterative) background subtraction.
    ///
    /// The Shirley background at energy E is proportional to the integrated
    /// signal above it: B(E) = k * integral(E to E_end) [S(E') - B(E')] dE'.
    pub fn shirley_background(
        spectrum: &AesSpectrum,
        e_start: f64,
        e_end: f64,
    ) -> Vec<f64> {
        let n = spectrum.counts.len();
        let i_start = Self::closest_index(&spectrum.kinetic_energy_ev, e_start);
        let i_end = Self::closest_index(&spectrum.kinetic_energy_ev, e_end);

        // Ensure i_start < i_end
        let (il, ih) = if i_start < i_end {
            (i_start, i_end)
        } else {
            (i_end, i_start)
        };

        let c_low = spectrum.counts[il];
        let c_high = spectrum.counts[ih];

        let mut bg = vec![0.0; n];
        // Initialize with linear background
        for i in 0..n {
            bg[i] = c_low;
        }

        // Iterate Shirley procedure
        let max_iter = 50;
        for _ in 0..max_iter {
            // Compute cumulative integral from high energy to low
            let mut integral = vec![0.0; n];
            for i in (il..ih).rev() {
                let ds = (spectrum.counts[i] - bg[i]).max(0.0);
                let de = if i + 1 < n {
                    (spectrum.kinetic_energy_ev[i + 1] - spectrum.kinetic_energy_ev[i]).abs()
                } else {
                    1.0
                };
                integral[i] = integral.get(i + 1).copied().unwrap_or(0.0) + ds * de;
            }

            let total_integral = integral[il];
            if total_integral.abs() < 1e-15 {
                break;
            }

            let k = (c_high - c_low) / total_integral;
            for i in il..=ih {
                bg[i] = c_low + k * integral[i];
            }
        }

        // Extend background outside the range
        for i in 0..il {
            bg[i] = bg[il];
        }
        for i in (ih + 1)..n {
            bg[i] = bg[ih];
        }

        bg
    }

    /// Subtract a background from a spectrum.
    pub fn subtract(spectrum: &AesSpectrum, background: &[f64]) -> AesSpectrum {
        assert_eq!(spectrum.counts.len(), background.len());
        let counts: Vec<f64> = spectrum
            .counts
            .iter()
            .zip(background.iter())
            .map(|(s, b)| s - b)
            .collect();
        AesSpectrum {
            kinetic_energy_ev: spectrum.kinetic_energy_ev.clone(),
            counts,
        }
    }

    fn closest_index(energies: &[f64], target: f64) -> usize {
        let mut best = 0;
        let mut best_dist = (energies[0] - target).abs();
        for (i, &e) in energies.iter().enumerate() {
            let d = (e - target).abs();
            if d < best_dist {
                best_dist = d;
                best = i;
            }
        }
        best
    }
}

// ─────────────────────────────── EscapeDepth ─────────────────────────────────

/// Material type for IMFP calculation.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MaterialType {
    /// Metallic element or alloy.
    Metal,
    /// Metal oxide or inorganic compound.
    Oxide,
    /// Organic material (polymer, biological).
    Organic,
}

/// Inelastic mean free path and escape depth calculations.
pub struct EscapeDepth;

impl EscapeDepth {
    /// Seah-Dench formula for IMFP in nm.
    ///
    /// For metals:   lambda = 538 / E^2 + 0.41 * (a * E)^0.5
    /// For oxides:   lambda = 2170 / E^2 + 0.72 * (a * E)^0.5
    /// For organics: lambda = 49 / E^2 + 0.11 * E^0.5
    ///
    /// where E is kinetic energy in eV and a is the monolayer thickness in nm.
    /// Default monolayer thickness: 0.25 nm (metals), 0.30 nm (oxides).
    pub fn imfp_seah_dench(kinetic_energy_ev: f64, material_type: MaterialType) -> f64 {
        let e = kinetic_energy_ev;
        if e <= 0.0 {
            return 0.0;
        }
        match material_type {
            MaterialType::Metal => {
                let a = 0.25; // nm monolayer thickness
                538.0 / (e * e) + 0.41 * (a * e).sqrt()
            }
            MaterialType::Oxide => {
                let a = 0.30;
                2170.0 / (e * e) + 0.72 * (a * e).sqrt()
            }
            MaterialType::Organic => {
                49.0 / (e * e) + 0.11 * e.sqrt()
            }
        }
    }

    /// Sampling depth = 3 * IMFP (captures ~95% of signal).
    pub fn sampling_depth(imfp: f64) -> f64 {
        3.0 * imfp
    }

    /// Overlayer attenuation factor.
    ///
    /// I/I0 = exp(-d / (lambda * cos(theta)))
    pub fn overlayer_attenuation(thickness_nm: f64, imfp_nm: f64, angle_deg: f64) -> f64 {
        if imfp_nm.abs() < 1e-15 {
            return 0.0;
        }
        let cos_theta = (angle_deg * PI / 180.0).cos();
        if cos_theta.abs() < 1e-15 {
            return 0.0;
        }
        (-thickness_nm / (imfp_nm * cos_theta)).exp()
    }
}

// ─────────────────────────────── AesSimulator ────────────────────────────────

/// Synthetic AES spectrum and depth profile generation.
pub struct AesSimulator;

impl AesSimulator {
    /// Simulate an AES spectrum with Gaussian peaks on a background.
    ///
    /// Each peak is specified as (center_energy_ev, height, fwhm_ev).
    /// Adds Gaussian noise with specified level.
    pub fn simulate_spectrum(
        peaks: &[(f64, f64, f64)],
        noise_level: f64,
    ) -> AesSpectrum {
        // Determine energy range from peaks
        let mut e_min = f64::INFINITY;
        let mut e_max = f64::NEG_INFINITY;
        for &(center, _, fwhm) in peaks {
            if center - 3.0 * fwhm < e_min {
                e_min = center - 3.0 * fwhm;
            }
            if center + 3.0 * fwhm > e_max {
                e_max = center + 3.0 * fwhm;
            }
        }
        if e_min > e_max {
            e_min = 0.0;
            e_max = 2000.0;
        }
        // Pad range
        e_min = (e_min - 50.0).max(0.0);
        e_max += 50.0;

        let num_points = 1000;
        let step = (e_max - e_min) / (num_points - 1) as f64;
        let energies: Vec<f64> = (0..num_points).map(|i| e_min + i as f64 * step).collect();

        let mut counts = vec![0.0; num_points];

        // Add Gaussian peaks
        for &(center, height, fwhm) in peaks {
            let sigma = fwhm / (2.0 * (2.0_f64.ln()).sqrt() * 2.0);
            for (i, &e) in energies.iter().enumerate() {
                let arg = (e - center) / sigma;
                counts[i] += height * (-0.5 * arg * arg).exp();
            }
        }

        // Add noise using a simple LCG PRNG
        let mut seed: u64 = 42;
        for c in counts.iter_mut() {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let u = (seed >> 33) as f64 / (1u64 << 31) as f64; // 0..1
            // Box-Muller for Gaussian noise
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let v = (seed >> 33) as f64 / (1u64 << 31) as f64;
            let u_clamped = u.max(1e-10).min(1.0 - 1e-10);
            let gauss = (-2.0 * u_clamped.ln()).sqrt() * (2.0 * PI * v).cos();
            *c += noise_level * gauss;
        }

        AesSpectrum::new(energies, counts)
    }

    /// Add a rising secondary electron background to a spectrum.
    ///
    /// Models the characteristic N(E) background that rises toward low energy.
    pub fn add_secondary_background(spectrum: &AesSpectrum, intensity: f64) -> AesSpectrum {
        let (e_min, e_max) = spectrum.energy_range();
        let range = e_max - e_min;
        let counts: Vec<f64> = spectrum
            .kinetic_energy_ev
            .iter()
            .zip(spectrum.counts.iter())
            .map(|(&e, &c)| {
                // Background ~ intensity / (1 + (E/50)^2) for secondary electron cascade
                let normalized = (e - e_min) / range;
                let bg = intensity * (1.0 - normalized).powi(2);
                c + bg
            })
            .collect();
        AesSpectrum {
            kinetic_energy_ev: spectrum.kinetic_energy_ev.clone(),
            counts,
        }
    }

    /// Simulate a depth profile from a layer structure.
    ///
    /// Each layer is defined by (thickness_nm, composition_vec).
    /// The sputter rate converts time to depth.
    pub fn simulate_depth_profile(
        layers: &[Layer],
        rate_nm_per_min: f64,
        num_points: usize,
    ) -> DepthProfile {
        // Total thickness
        let total_thickness: f64 = layers.iter().map(|l| l.thickness_nm).sum();
        let total_time = total_thickness / rate_nm_per_min;
        let dt = total_time / (num_points - 1).max(1) as f64;

        let num_elements = if let Some(first) = layers.first() {
            first.composition.len()
        } else {
            return DepthProfile::new(vec![0.0], vec![vec![]]);
        };

        let mut times = Vec::with_capacity(num_points);
        let mut compositions = Vec::with_capacity(num_points);

        // Mixing parameter (broadens interfaces)
        let mixing_sigma = 1.0; // nm

        for i in 0..num_points {
            let t = i as f64 * dt;
            let depth = t * rate_nm_per_min;
            times.push(t);

            let mut comp = vec![0.0; num_elements];
            let mut boundary = 0.0;

            for (layer_idx, layer) in layers.iter().enumerate() {
                let next_boundary = boundary + layer.thickness_nm;

                // Use error function-like profile for mixing
                let contribution = if layer_idx == 0 {
                    // First layer: 1 - erf((depth - boundary_top) / sigma)
                    0.5 * erfc_approx((depth - next_boundary) / (mixing_sigma * 2.0_f64.sqrt()))
                } else if layer_idx == layers.len() - 1 {
                    // Last layer (substrate)
                    0.5 * erfc_approx((boundary - depth) / (mixing_sigma * 2.0_f64.sqrt()))
                } else {
                    // Middle layer: difference of two error functions
                    let lower = 0.5 * erfc_approx((boundary - depth) / (mixing_sigma * 2.0_f64.sqrt()));
                    let upper = 0.5 * erfc_approx((depth - next_boundary) / (mixing_sigma * 2.0_f64.sqrt()));
                    lower * upper
                };

                for (j, &c) in layer.composition.iter().enumerate() {
                    if j < num_elements {
                        comp[j] += c * contribution;
                    }
                }

                boundary = next_boundary;
            }

            compositions.push(comp);
        }

        DepthProfile::new(times, compositions)
    }
}

/// A layer for depth profile simulation.
#[derive(Debug, Clone)]
pub struct Layer {
    /// Layer thickness in nm.
    pub thickness_nm: f64,
    /// Composition in atomic percent for each element.
    pub composition: Vec<f64>,
}

/// Complementary error function approximation.
fn erfc_approx(x: f64) -> f64 {
    // Abramowitz and Stegun approximation 7.1.26
    let t = 1.0 / (1.0 + 0.3275911 * x.abs());
    let poly = t
        * (0.254829592
            + t * (-0.284496736
                + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    let result = poly * (-x * x).exp();
    if x >= 0.0 {
        result
    } else {
        2.0 - result
    }
}

// ─────────────────────────────── ScanningAuger ───────────────────────────────

/// Result of grain boundary analysis from an AES line scan.
#[derive(Debug, Clone)]
pub struct GrainBoundaryResult {
    /// Position of peak segregation (index along line).
    pub peak_position: usize,
    /// Peak intensity at the grain boundary.
    pub peak_intensity: f64,
    /// Background intensity (average of grain interior).
    pub background_intensity: f64,
    /// Enrichment factor (peak / background).
    pub enrichment_factor: f64,
    /// Full width at half maximum of the segregation profile (in points).
    pub fwhm_points: usize,
}

/// Scanning Auger Microscopy (SAM) analysis.
pub struct ScanningAuger;

impl ScanningAuger {
    /// Generate an element map from a grid of spectra.
    ///
    /// For each spectrum in the grid, extracts the peak height at the given
    /// element energy (within tolerance) to build a 2D intensity map.
    pub fn element_map(
        spectra_grid: &[Vec<AesSpectrum>],
        element_energy: f64,
        tolerance: f64,
    ) -> Vec<Vec<f64>> {
        spectra_grid
            .iter()
            .map(|row| {
                row.iter()
                    .map(|spec| Self::extract_peak_intensity(spec, element_energy, tolerance))
                    .collect()
            })
            .collect()
    }

    /// Extract peak intensity along a line scan.
    pub fn line_scan(
        spectra: &[AesSpectrum],
        element_energy: f64,
    ) -> Vec<f64> {
        spectra
            .iter()
            .map(|spec| Self::extract_peak_intensity(spec, element_energy, 10.0))
            .collect()
    }

    /// Analyze grain boundary segregation from a line scan profile.
    pub fn grain_boundary_analysis(line_data: &[f64]) -> GrainBoundaryResult {
        let n = line_data.len();
        if n == 0 {
            return GrainBoundaryResult {
                peak_position: 0,
                peak_intensity: 0.0,
                background_intensity: 0.0,
                enrichment_factor: 0.0,
                fwhm_points: 0,
            };
        }

        // Find peak position
        let mut peak_pos = 0;
        let mut peak_val = line_data[0];
        for (i, &v) in line_data.iter().enumerate() {
            if v > peak_val {
                peak_val = v;
                peak_pos = i;
            }
        }

        // Background is average of first and last quarters
        let quarter = n / 4;
        let bg_left: f64 = if quarter > 0 {
            line_data[..quarter].iter().sum::<f64>() / quarter as f64
        } else {
            line_data[0]
        };
        let bg_right: f64 = if quarter > 0 {
            line_data[(n - quarter)..].iter().sum::<f64>() / quarter as f64
        } else {
            line_data[n - 1]
        };
        let background = (bg_left + bg_right) / 2.0;

        // FWHM
        let half_max = background + (peak_val - background) / 2.0;
        let mut left_hw = peak_pos;
        let mut right_hw = peak_pos;
        for i in (0..peak_pos).rev() {
            if line_data[i] < half_max {
                left_hw = i;
                break;
            }
        }
        for i in peak_pos..n {
            if line_data[i] < half_max {
                right_hw = i;
                break;
            }
        }
        let fwhm = if right_hw > left_hw {
            right_hw - left_hw
        } else {
            1
        };

        let enrichment = if background.abs() > 1e-15 {
            peak_val / background
        } else {
            0.0
        };

        GrainBoundaryResult {
            peak_position: peak_pos,
            peak_intensity: peak_val,
            background_intensity: background,
            enrichment_factor: enrichment,
            fwhm_points: fwhm,
        }
    }

    /// Extract the maximum intensity near a target energy.
    fn extract_peak_intensity(spectrum: &AesSpectrum, energy: f64, tolerance: f64) -> f64 {
        let mut max_intensity = 0.0_f64;
        for (i, &e) in spectrum.kinetic_energy_ev.iter().enumerate() {
            if (e - energy).abs() <= tolerance {
                if spectrum.counts[i] > max_intensity {
                    max_intensity = spectrum.counts[i];
                }
            }
        }
        max_intensity
    }
}

// ═══════════════════════════════════ Tests ════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    // Helper to create a simple energy axis
    fn energy_axis(start: f64, end: f64, n: usize) -> Vec<f64> {
        let step = (end - start) / (n - 1) as f64;
        (0..n).map(|i| start + i as f64 * step).collect()
    }

    fn gaussian_peak(energies: &[f64], center: f64, height: f64, fwhm: f64) -> Vec<f64> {
        let sigma = fwhm / (2.0 * (2.0_f64.ln()).sqrt() * 2.0);
        energies
            .iter()
            .map(|&e| {
                let arg = (e - center) / sigma;
                height * (-0.5 * arg * arg).exp()
            })
            .collect()
    }

    // ── AesSpectrum tests ──

    #[test]
    fn test_spectrum_new() {
        let spec = AesSpectrum::new(vec![100.0, 200.0, 300.0], vec![1.0, 2.0, 3.0]);
        assert_eq!(spec.len(), 3);
        assert!(!spec.is_empty());
    }

    #[test]
    #[should_panic]
    fn test_spectrum_new_empty() {
        AesSpectrum::new(vec![], vec![]);
    }

    #[test]
    #[should_panic]
    fn test_spectrum_new_mismatch() {
        AesSpectrum::new(vec![1.0, 2.0], vec![1.0]);
    }

    #[test]
    fn test_energy_range() {
        let spec = AesSpectrum::new(vec![50.0, 100.0, 200.0, 150.0], vec![1.0; 4]);
        let (min, max) = spec.energy_range();
        assert!((min - 50.0).abs() < 1e-10);
        assert!((max - 200.0).abs() < 1e-10);
    }

    #[test]
    fn test_multiply_by_energy() {
        let spec = AesSpectrum::new(vec![100.0, 200.0, 300.0], vec![1.0, 2.0, 3.0]);
        let ene = spec.multiply_by_energy();
        assert!((ene.counts[0] - 100.0).abs() < 1e-10);
        assert!((ene.counts[1] - 400.0).abs() < 1e-10);
        assert!((ene.counts[2] - 900.0).abs() < 1e-10);
    }

    #[test]
    fn test_smooth_preserves_length() {
        let energies = energy_axis(0.0, 1000.0, 100);
        let counts = vec![1.0; 100];
        let spec = AesSpectrum::new(energies, counts);
        let smoothed = spec.smooth(5);
        assert_eq!(smoothed.len(), spec.len());
    }

    #[test]
    fn test_smooth_reduces_noise() {
        let energies = energy_axis(0.0, 100.0, 101);
        // Noisy signal: constant 10.0 with +/- 5 noise
        let counts: Vec<f64> = energies
            .iter()
            .enumerate()
            .map(|(i, _)| 10.0 + if i % 2 == 0 { 5.0 } else { -5.0 })
            .collect();
        let spec = AesSpectrum::new(energies, counts);
        let smoothed = spec.smooth(5);
        // Middle values should be closer to 10.0
        let mid = smoothed.len() / 2;
        assert!((smoothed.counts[mid] - 10.0).abs() < 5.0);
    }

    #[test]
    fn test_smooth_minimum_window() {
        let spec = AesSpectrum::new(vec![1.0, 2.0, 3.0, 4.0, 5.0], vec![1.0, 3.0, 5.0, 3.0, 1.0]);
        let smoothed = spec.smooth(1); // Should be clamped to 3
        assert_eq!(smoothed.len(), 5);
    }

    #[test]
    fn test_smooth_even_window() {
        let spec = AesSpectrum::new(vec![1.0, 2.0, 3.0, 4.0, 5.0], vec![1.0, 3.0, 5.0, 3.0, 1.0]);
        let smoothed = spec.smooth(4); // Should become 5
        assert_eq!(smoothed.len(), 5);
    }

    // ── DerivativeSpectrum tests ──

    #[test]
    fn test_derivative_of_linear() {
        // dN/dE of a line with slope 2 should be ~2 everywhere
        let energies = energy_axis(0.0, 100.0, 101);
        let counts: Vec<f64> = energies.iter().map(|&e| 2.0 * e).collect();
        let spec = AesSpectrum::new(energies, counts);
        let deriv = DerivativeSpectrum::compute(&spec, 1);
        // Interior points should be close to 2.0
        for i in 2..deriv.len() - 2 {
            assert!(
                (deriv.counts[i] - 2.0).abs() < 0.1,
                "derivative at {} was {}",
                i,
                deriv.counts[i]
            );
        }
    }

    #[test]
    fn test_derivative_peak_detection() {
        let energies = energy_axis(200.0, 350.0, 500);
        let counts = gaussian_peak(&energies, 272.0, 100.0, 15.0);
        let spec = AesSpectrum::new(energies, counts);
        let deriv = DerivativeSpectrum::compute(&spec, 2);
        // Peak-to-peak in the derivative region around 272 eV
        let p2p = DerivativeSpectrum::peak_to_peak_height(&deriv, 255.0, 290.0);
        assert!(p2p > 0.0, "peak-to-peak should be positive");
    }

    #[test]
    fn test_peak_to_peak_empty_range() {
        let spec = AesSpectrum::new(vec![100.0, 200.0], vec![1.0, 2.0]);
        let p2p = DerivativeSpectrum::peak_to_peak_height(&spec, 300.0, 400.0);
        assert!((p2p - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_find_peaks_with_carbon() {
        let energies = energy_axis(200.0, 350.0, 500);
        let counts = gaussian_peak(&energies, 272.0, 100.0, 15.0);
        let spec = AesSpectrum::new(energies, counts);
        let deriv = DerivativeSpectrum::compute(&spec, 2);
        let peaks = DerivativeSpectrum::find_peaks(&deriv, 0.1);
        // Should find at least one peak near 272 eV
        assert!(!peaks.is_empty(), "should find at least one peak");
        let carbon_peak = peaks
            .iter()
            .find(|p| (p.kinetic_energy_ev - 272.0).abs() < 15.0);
        assert!(carbon_peak.is_some(), "should find C KLL near 272 eV");
    }

    #[test]
    fn test_find_peaks_identifies_element() {
        let energies = energy_axis(480.0, 540.0, 200);
        let counts = gaussian_peak(&energies, 510.0, 200.0, 10.0);
        let spec = AesSpectrum::new(energies, counts);
        let deriv = DerivativeSpectrum::compute(&spec, 2);
        let peaks = DerivativeSpectrum::find_peaks(&deriv, 0.01);
        let oxygen = peaks.iter().find(|p| {
            p.element.as_deref() == Some("O")
        });
        assert!(oxygen.is_some(), "should identify oxygen near 510 eV");
    }

    // ── ElementIdentifier tests ──

    #[test]
    fn test_identify_carbon() {
        let matches = ElementIdentifier::identify(272.0, 5.0);
        assert!(!matches.is_empty());
        assert_eq!(matches[0].element, "C");
        assert_eq!(matches[0].transition, "KLL");
    }

    #[test]
    fn test_identify_oxygen() {
        let matches = ElementIdentifier::identify(510.0, 5.0);
        assert!(!matches.is_empty());
        assert_eq!(matches[0].element, "O");
    }

    #[test]
    fn test_identify_silicon() {
        let matches = ElementIdentifier::identify(92.0, 3.0);
        assert!(!matches.is_empty());
        assert_eq!(matches[0].element, "Si");
    }

    #[test]
    fn test_identify_iron() {
        let matches = ElementIdentifier::identify(703.0, 5.0);
        assert!(!matches.is_empty());
        assert_eq!(matches[0].element, "Fe");
    }

    #[test]
    fn test_identify_no_match() {
        let matches = ElementIdentifier::identify(1500.0, 1.0);
        // No major transitions at exactly 1500 eV within 1 eV tolerance
        // (Al KLL is at 1396)
        assert!(matches.is_empty());
    }

    #[test]
    fn test_identify_within_tolerance() {
        let matches = ElementIdentifier::identify(275.0, 5.0);
        assert!(matches.iter().any(|m| m.element == "C"));
    }

    #[test]
    fn test_identify_sorted_by_delta() {
        let matches = ElementIdentifier::identify(273.0, 20.0);
        if matches.len() >= 2 {
            assert!(matches[0].delta_ev <= matches[1].delta_ev);
        }
    }

    #[test]
    fn test_auger_energy_lookup() {
        assert!((ElementIdentifier::auger_energy("C", "KLL").unwrap() - 272.0).abs() < 0.1);
        assert!((ElementIdentifier::auger_energy("Fe", "LMM").unwrap() - 703.0).abs() < 0.1);
        assert!(ElementIdentifier::auger_energy("Xx", "ZZZ").is_none());
    }

    #[test]
    fn test_all_transitions() {
        let trans = ElementIdentifier::all_transitions("C");
        assert_eq!(trans.len(), 1);
        assert_eq!(trans[0].0, "KLL");
        assert!((trans[0].1 - 272.0).abs() < 0.1);
    }

    #[test]
    fn test_all_transitions_unknown() {
        let trans = ElementIdentifier::all_transitions("Xx");
        assert!(trans.is_empty());
    }

    // ── Quantification tests ──

    #[test]
    fn test_atomic_percent_equal() {
        let heights = vec![10.0, 10.0];
        let sf = vec![1.0, 1.0];
        let result = Quantification::atomic_percent(&heights, &sf);
        assert!((result[0] - 50.0).abs() < 0.01);
        assert!((result[1] - 50.0).abs() < 0.01);
    }

    #[test]
    fn test_atomic_percent_with_sensitivity() {
        let heights = vec![10.0, 10.0];
        let sf = vec![0.5, 1.0]; // first element needs more signal per atom
        let result = Quantification::atomic_percent(&heights, &sf);
        // 10/0.5 = 20, 10/1.0 = 10, total = 30
        assert!((result[0] - 66.67).abs() < 0.1);
        assert!((result[1] - 33.33).abs() < 0.1);
    }

    #[test]
    fn test_atomic_percent_sums_to_100() {
        let heights = vec![5.0, 15.0, 10.0];
        let sf = vec![0.3, 0.5, 0.8];
        let result = Quantification::atomic_percent(&heights, &sf);
        let sum: f64 = result.iter().sum();
        assert!((sum - 100.0).abs() < 0.01);
    }

    #[test]
    fn test_atomic_percent_zero_signals() {
        let heights = vec![0.0, 0.0];
        let sf = vec![1.0, 1.0];
        let result = Quantification::atomic_percent(&heights, &sf);
        assert!((result[0]).abs() < 0.01);
    }

    #[test]
    fn test_sensitivity_factor_known() {
        assert!((Quantification::sensitivity_factor("C") - 0.14).abs() < 0.001);
        assert!((Quantification::sensitivity_factor("Ag") - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_sensitivity_factor_unknown() {
        assert!((Quantification::sensitivity_factor("Xx") - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_matrix_correction() {
        let comp = vec![50.0, 50.0];
        let elements = vec!["C", "Fe"];
        let corrected = Quantification::matrix_correction(&comp, &elements);
        let sum: f64 = corrected.iter().sum();
        assert!((sum - 100.0).abs() < 0.01);
        // Fe (Z=26) should have slightly higher corrected fraction due to backscatter
        assert!(corrected[1] > corrected[0]);
    }

    #[test]
    fn test_detection_limit() {
        let dl = Quantification::detection_limit(0.1, 0.3);
        assert!(dl > 0.0);
        assert!(dl < 10.0); // should be ~ 1 at%
    }

    #[test]
    fn test_detection_limit_zero_sensitivity() {
        let dl = Quantification::detection_limit(0.1, 0.0);
        assert!(dl.is_infinite());
    }

    // ── AugerParameterCalculator tests ──

    #[test]
    fn test_auger_parameter() {
        let alpha = AugerParameterCalculator::auger_parameter(920.0, 335.0);
        assert!((alpha - 1255.0).abs() < 0.01);
    }

    #[test]
    fn test_extra_atomic_relaxation() {
        let r = AugerParameterCalculator::extra_atomic_relaxation(1255.0, 1251.0);
        assert!((r - (-2.0)).abs() < 0.01);
    }

    #[test]
    fn test_wagner_plot_position() {
        let (x, y) = AugerParameterCalculator::wagner_plot_position(920.0, 335.0);
        assert!((x - 335.0).abs() < 0.01);
        assert!((y - 920.0).abs() < 0.01);
    }

    // ── DepthProfile tests ──

    #[test]
    fn test_depth_from_rate() {
        let d = DepthProfile::depth_from_rate(10.0, 2.0);
        assert!((d - 20.0).abs() < 0.01);
    }

    #[test]
    fn test_interface_position() {
        // Sigmoid-like profile
        let depths: Vec<f64> = (0..100).map(|i| i as f64 * 0.1).collect();
        let profile: Vec<f64> = depths
            .iter()
            .map(|&d| 100.0 / (1.0 + ((d - 5.0) * 3.0).exp()))
            .collect();
        let pos = DepthProfile::interface_position(&profile, &depths, 50.0);
        assert!((pos - 5.0).abs() < 0.5, "interface at ~5 nm, got {}", pos);
    }

    #[test]
    fn test_layer_thickness() {
        // Trapezoidal profile: 0->100->100->0 over 0-10 nm
        let depths: Vec<f64> = (0..100).map(|i| i as f64 * 0.1).collect();
        let profile: Vec<f64> = depths
            .iter()
            .map(|&d| {
                if d < 2.0 {
                    50.0 * d
                } else if d < 8.0 {
                    100.0
                } else if d < 10.0 {
                    100.0 - 50.0 * (d - 8.0)
                } else {
                    0.0
                }
            })
            .collect();
        let thickness = DepthProfile::layer_thickness(&profile, &depths);
        assert!(thickness > 5.0 && thickness < 10.0, "thickness = {}", thickness);
    }

    #[test]
    fn test_mixing_length() {
        let depths: Vec<f64> = (0..200).map(|i| i as f64 * 0.05).collect();
        let profile: Vec<f64> = depths
            .iter()
            .map(|&d| 100.0 / (1.0 + ((d - 5.0) * 2.0).exp()))
            .collect();
        let ml = DepthProfile::mixing_length(&profile, &depths);
        assert!(ml > 0.0 && ml < 5.0, "mixing_length = {}", ml);
    }

    #[test]
    fn test_depth_profile_new() {
        let dp = DepthProfile::new(vec![0.0, 1.0, 2.0], vec![vec![50.0, 50.0]; 3]);
        assert_eq!(dp.sputter_times_min.len(), 3);
        assert_eq!(dp.compositions.len(), 3);
    }

    // ── BackgroundSubtraction tests ──

    #[test]
    fn test_linear_background() {
        let spec = AesSpectrum::new(
            vec![100.0, 200.0, 300.0, 400.0, 500.0],
            vec![10.0, 20.0, 30.0, 20.0, 10.0],
        );
        let bg = BackgroundSubtraction::linear_background(&spec, 100.0, 500.0);
        assert_eq!(bg.len(), 5);
        assert!((bg[0] - 10.0).abs() < 0.01);
        assert!((bg[4] - 10.0).abs() < 0.01);
    }

    #[test]
    fn test_shirley_background() {
        let energies = energy_axis(100.0, 500.0, 200);
        let mut counts: Vec<f64> = energies.iter().map(|_| 5.0).collect();
        // Add a peak
        for (i, &e) in energies.iter().enumerate() {
            let arg = (e - 300.0) / 30.0;
            counts[i] += 50.0 * (-0.5 * arg * arg).exp();
        }
        let spec = AesSpectrum::new(energies, counts);
        let bg = BackgroundSubtraction::shirley_background(&spec, 100.0, 500.0);
        assert_eq!(bg.len(), 200);
        // Shirley background should be between the endpoint values
        for &b in &bg {
            assert!(b >= 0.0, "background should be non-negative");
        }
    }

    #[test]
    fn test_subtract_background() {
        let spec = AesSpectrum::new(vec![1.0, 2.0, 3.0], vec![10.0, 20.0, 15.0]);
        let bg = vec![5.0, 5.0, 5.0];
        let result = BackgroundSubtraction::subtract(&spec, &bg);
        assert!((result.counts[0] - 5.0).abs() < 0.01);
        assert!((result.counts[1] - 15.0).abs() < 0.01);
        assert!((result.counts[2] - 10.0).abs() < 0.01);
    }

    // ── EscapeDepth tests ──

    #[test]
    fn test_imfp_metal() {
        let lambda = EscapeDepth::imfp_seah_dench(500.0, MaterialType::Metal);
        assert!(lambda > 0.0 && lambda < 10.0, "IMFP = {} nm", lambda);
    }

    #[test]
    fn test_imfp_oxide() {
        let lambda = EscapeDepth::imfp_seah_dench(500.0, MaterialType::Oxide);
        assert!(lambda > 0.0 && lambda < 15.0, "IMFP = {} nm", lambda);
    }

    #[test]
    fn test_imfp_organic() {
        let lambda = EscapeDepth::imfp_seah_dench(500.0, MaterialType::Organic);
        assert!(lambda > 0.0, "IMFP should be positive");
    }

    #[test]
    fn test_imfp_zero_energy() {
        let lambda = EscapeDepth::imfp_seah_dench(0.0, MaterialType::Metal);
        assert!((lambda - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_imfp_increases_with_energy() {
        let l1 = EscapeDepth::imfp_seah_dench(100.0, MaterialType::Metal);
        let l2 = EscapeDepth::imfp_seah_dench(1000.0, MaterialType::Metal);
        assert!(l2 > l1, "IMFP should increase with energy");
    }

    #[test]
    fn test_sampling_depth() {
        let lambda = 1.5;
        let d = EscapeDepth::sampling_depth(lambda);
        assert!((d - 4.5).abs() < 0.01);
    }

    #[test]
    fn test_overlayer_attenuation_zero_thickness() {
        let atten = EscapeDepth::overlayer_attenuation(0.0, 1.0, 0.0);
        assert!((atten - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_overlayer_attenuation_thick() {
        let atten = EscapeDepth::overlayer_attenuation(10.0, 1.0, 0.0);
        assert!(atten < 0.001, "thick overlayer should attenuate strongly");
    }

    #[test]
    fn test_overlayer_attenuation_angle_dependence() {
        let a0 = EscapeDepth::overlayer_attenuation(2.0, 1.5, 0.0);
        let a60 = EscapeDepth::overlayer_attenuation(2.0, 1.5, 60.0);
        assert!(a60 < a0, "grazing angle should attenuate more");
    }

    // ── AesSimulator tests ──

    #[test]
    fn test_simulate_spectrum_single_peak() {
        let spec = AesSimulator::simulate_spectrum(&[(272.0, 100.0, 10.0)], 0.0);
        assert!(!spec.is_empty());
        // Find the maximum
        let max_idx = spec
            .counts
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .unwrap()
            .0;
        let peak_energy = spec.kinetic_energy_ev[max_idx];
        assert!((peak_energy - 272.0).abs() < 5.0);
    }

    #[test]
    fn test_simulate_spectrum_with_noise() {
        let spec = AesSimulator::simulate_spectrum(&[(500.0, 100.0, 15.0)], 5.0);
        assert!(spec.len() > 100);
    }

    #[test]
    fn test_simulate_multiple_peaks() {
        let peaks = vec![(272.0, 100.0, 10.0), (510.0, 80.0, 12.0)];
        let spec = AesSimulator::simulate_spectrum(&peaks, 0.0);
        assert!(spec.len() > 0);
    }

    #[test]
    fn test_add_secondary_background() {
        let spec = AesSpectrum::new(
            vec![100.0, 200.0, 300.0, 400.0, 500.0],
            vec![10.0; 5],
        );
        let with_bg = AesSimulator::add_secondary_background(&spec, 50.0);
        // Background should be higher at low energies
        assert!(with_bg.counts[0] > with_bg.counts[4]);
    }

    #[test]
    fn test_simulate_depth_profile() {
        let layers = vec![
            Layer {
                thickness_nm: 5.0,
                composition: vec![100.0, 0.0],
            },
            Layer {
                thickness_nm: 50.0,
                composition: vec![0.0, 100.0],
            },
        ];
        let dp = AesSimulator::simulate_depth_profile(&layers, 1.0, 50);
        assert_eq!(dp.sputter_times_min.len(), 50);
        assert_eq!(dp.compositions.len(), 50);
        // First point should be mostly first layer
        assert!(dp.compositions[0][0] > dp.compositions[0][1]);
    }

    // ── ScanningAuger tests ──

    #[test]
    fn test_element_map() {
        let spec1 = AesSpectrum::new(vec![270.0, 272.0, 274.0], vec![5.0, 100.0, 5.0]);
        let spec2 = AesSpectrum::new(vec![270.0, 272.0, 274.0], vec![5.0, 50.0, 5.0]);
        let grid = vec![vec![spec1.clone(), spec2.clone()], vec![spec2, spec1]];
        let map = ScanningAuger::element_map(&grid, 272.0, 3.0);
        assert_eq!(map.len(), 2);
        assert_eq!(map[0].len(), 2);
        assert!((map[0][0] - 100.0).abs() < 0.01);
        assert!((map[0][1] - 50.0).abs() < 0.01);
    }

    #[test]
    fn test_line_scan() {
        let specs: Vec<AesSpectrum> = (0..10)
            .map(|i| {
                AesSpectrum::new(
                    vec![270.0, 272.0, 274.0],
                    vec![5.0, 10.0 * (i + 1) as f64, 5.0],
                )
            })
            .collect();
        let scan = ScanningAuger::line_scan(&specs, 272.0);
        assert_eq!(scan.len(), 10);
        assert!(scan[9] > scan[0]);
    }

    #[test]
    fn test_grain_boundary_analysis() {
        // Gaussian-shaped segregation profile
        let n = 50;
        let data: Vec<f64> = (0..n)
            .map(|i| {
                let x = i as f64 - 25.0;
                1.0 + 10.0 * (-x * x / 18.0).exp()
            })
            .collect();
        let result = ScanningAuger::grain_boundary_analysis(&data);
        assert_eq!(result.peak_position, 25);
        assert!(result.peak_intensity > 10.0);
        assert!(result.enrichment_factor > 5.0);
        assert!(result.fwhm_points > 0);
    }

    #[test]
    fn test_grain_boundary_empty() {
        let result = ScanningAuger::grain_boundary_analysis(&[]);
        assert_eq!(result.peak_position, 0);
        assert!((result.peak_intensity - 0.0).abs() < 1e-10);
    }

    // ── erfc approximation test ──

    #[test]
    fn test_erfc_zero() {
        let val = erfc_approx(0.0);
        assert!((val - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_erfc_large_positive() {
        let val = erfc_approx(3.0);
        assert!(val < 0.001);
    }

    #[test]
    fn test_erfc_large_negative() {
        let val = erfc_approx(-3.0);
        assert!((val - 2.0).abs() < 0.01);
    }

    // ── Additional integration tests ──

    #[test]
    fn test_full_workflow_carbon_on_silicon() {
        // Simulate AES spectrum of carbon contamination on silicon
        let peaks = vec![
            (272.0, 80.0, 10.0),  // C KLL
            (92.0, 150.0, 8.0),   // Si LMM
        ];
        let spec = AesSimulator::simulate_spectrum(&peaks, 1.0);
        let with_bg = AesSimulator::add_secondary_background(&spec, 20.0);
        let smoothed = with_bg.smooth(5);

        // Compute derivative
        let deriv = DerivativeSpectrum::compute(&smoothed, 3);
        assert_eq!(deriv.len(), smoothed.len());

        // Quantify
        let c_sf = Quantification::sensitivity_factor("C");
        let si_sf = Quantification::sensitivity_factor("Si");
        let heights = vec![80.0, 150.0];
        let sfs = vec![c_sf, si_sf];
        let comp = Quantification::atomic_percent(&heights, &sfs);
        let sum: f64 = comp.iter().sum();
        assert!((sum - 100.0).abs() < 0.01);
    }

    #[test]
    fn test_full_workflow_stainless_steel() {
        // Fe, Cr, Ni as in stainless steel
        let fe_sf = Quantification::sensitivity_factor("Fe");
        let cr_sf = Quantification::sensitivity_factor("Cr");
        let ni_sf = Quantification::sensitivity_factor("Ni");

        // Typical 304 SS: 70% Fe, 18% Cr, 12% Ni
        // Raw peak heights proportional to concentration * sensitivity
        let heights = vec![70.0 * fe_sf, 18.0 * cr_sf, 12.0 * ni_sf];
        let sfs = vec![fe_sf, cr_sf, ni_sf];
        let comp = Quantification::atomic_percent(&heights, &sfs);
        assert!((comp[0] - 70.0).abs() < 0.1);
        assert!((comp[1] - 18.0).abs() < 0.1);
        assert!((comp[2] - 12.0).abs() < 0.1);
    }

    #[test]
    fn test_identify_copper() {
        let matches = ElementIdentifier::identify(920.0, 5.0);
        assert!(matches.iter().any(|m| m.element == "Cu"));
    }

    #[test]
    fn test_identify_nickel() {
        let matches = ElementIdentifier::identify(848.0, 5.0);
        assert!(matches.iter().any(|m| m.element == "Ni"));
    }

    #[test]
    fn test_identify_silver() {
        let matches = ElementIdentifier::identify(351.0, 5.0);
        assert!(matches.iter().any(|m| m.element == "Ag"));
    }

    #[test]
    fn test_identify_gold() {
        let matches = ElementIdentifier::identify(2024.0, 5.0);
        assert!(matches.iter().any(|m| m.element == "Au"));
    }

    #[test]
    fn test_depth_profile_interface_rising() {
        // Rising profile
        let depths: Vec<f64> = (0..50).map(|i| i as f64).collect();
        let profile: Vec<f64> = depths.iter().map(|&d| d * 2.0).collect();
        let pos = DepthProfile::interface_position(&profile, &depths, 50.0);
        assert!((pos - 25.0).abs() < 1.0);
    }

    #[test]
    fn test_approximate_atomic_number() {
        assert_eq!(approximate_atomic_number("C"), 6);
        assert_eq!(approximate_atomic_number("Fe"), 26);
        assert_eq!(approximate_atomic_number("Au"), 79);
        assert_eq!(approximate_atomic_number("Unknown"), 30); // default
    }

    #[test]
    fn test_auger_parameter_cu_metal() {
        // Cu metal: KE(LMM) ~ 920 eV, BE(2p3/2) ~ 932.7 eV
        let alpha = AugerParameterCalculator::auger_parameter(920.0, 932.7);
        assert!((alpha - 1852.7).abs() < 0.1);
    }

    #[test]
    fn test_overlayer_model() {
        // 1 nm C on Si, IMFP of Si LMM through C
        let imfp = EscapeDepth::imfp_seah_dench(92.0, MaterialType::Organic);
        let atten = EscapeDepth::overlayer_attenuation(1.0, imfp, 0.0);
        assert!(atten > 0.0 && atten < 1.0);
    }

    #[test]
    fn test_linear_bg_endpoints_match() {
        let spec = AesSpectrum::new(
            vec![100.0, 150.0, 200.0, 250.0, 300.0],
            vec![20.0, 40.0, 60.0, 40.0, 20.0],
        );
        let bg = BackgroundSubtraction::linear_background(&spec, 100.0, 300.0);
        assert!((bg[0] - 20.0).abs() < 0.01);
        assert!((bg[4] - 20.0).abs() < 0.01);
    }

    #[test]
    fn test_derivative_spacing() {
        let energies = energy_axis(0.0, 100.0, 101);
        let counts: Vec<f64> = energies.iter().map(|&e| e * e).collect();
        let spec = AesSpectrum::new(energies, counts);
        // Derivative of x^2 is 2x
        let d1 = DerivativeSpectrum::compute(&spec, 1);
        let d5 = DerivativeSpectrum::compute(&spec, 5);
        // Both should approximate 2x, but d5 is smoother
        let mid = 50;
        assert!((d1.counts[mid] - 100.0).abs() < 5.0);
        assert!((d5.counts[mid] - 100.0).abs() < 5.0);
    }

    #[test]
    fn test_multiple_element_identification() {
        // Near Ca LMM (291) and C KLL (272) - within 20 eV tolerance
        let matches = ElementIdentifier::identify(280.0, 15.0);
        assert!(matches.iter().any(|m| m.element == "C" || m.element == "Ca"));
    }

    #[test]
    fn test_depth_profile_no_crossing() {
        // Profile that never crosses threshold
        let depths = vec![0.0, 1.0, 2.0, 3.0];
        let profile = vec![10.0, 10.0, 10.0, 10.0];
        let pos = DepthProfile::interface_position(&profile, &depths, 50.0);
        assert!((pos - 3.0).abs() < 0.01); // returns last depth
    }
}
