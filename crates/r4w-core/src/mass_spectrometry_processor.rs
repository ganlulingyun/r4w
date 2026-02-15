// trace:FR-MASS-SPEC | ai:claude
//! # Mass Spectrometry Signal Processor
//!
//! Implements mass spectrometry signal processing for analytical chemistry.
//! Mass spectrometers measure mass-to-charge ratios (m/z) of ions, producing
//! spectra used for molecular identification, quantification, and structural
//! elucidation.
//!
//! ## Key Processing Steps
//!
//! 1. **Baseline correction** - Remove background signal drift
//! 2. **Noise estimation** - Determine noise floor from spectral data
//! 3. **Smoothing** - Savitzky-Golay-like polynomial smoothing
//! 4. **Peak detection** - Find peaks above SNR threshold
//! 5. **Centroiding** - Compute intensity-weighted m/z for profile peaks
//! 6. **Isotope pattern analysis** - Charge state determination and deisotoping
//! 7. **Molecular formula assignment** - Match observed masses to candidates
//! 8. **Quantification** - Peak area integration, calibration, LOD
//!
//! ## Atomic Masses (Da)
//!
//! | Element | Monoisotopic | Average  |
//! |---------|-------------|----------|
//! | C-12    | 12.000000   | 12.011   |
//! | C-13    | 13.003355   | -        |
//! | H       | 1.007830    | 1.00794  |
//! | N       | 14.003074   | 14.007   |
//! | O       | 15.994915   | 15.999   |
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::mass_spectrometry_processor::*;
//!
//! let config = MassSpecConfig {
//!     mass_range: (100.0, 2000.0),
//!     resolution: 10000.0,
//!     ionization_mode: IonizationMode::EsiPositive,
//!     mass_accuracy_ppm: 5.0,
//! };
//! let processor = MassSpecProcessor::new(config);
//!
//! // Detect peaks in a spectrum
//! let spectrum: Vec<(f64, f64)> = vec![
//!     (500.0, 100.0), (500.1, 500.0), (500.2, 1000.0),
//!     (500.3, 500.0), (500.4, 100.0),
//! ];
//! let peaks = processor.detect_peaks(&spectrum, 3.0);
//! ```

// ── Atomic masses (monoisotopic, in Daltons) ────────────────────────────────

/// Monoisotopic mass of Carbon-12 (Da).
pub const MASS_C12: f64 = 12.0;

/// Monoisotopic mass of Carbon-13 (Da).
pub const MASS_C13: f64 = 13.003355;

/// Monoisotopic mass of Hydrogen (Da).
pub const MASS_H: f64 = 1.007830;

/// Monoisotopic mass of Nitrogen-14 (Da).
pub const MASS_N: f64 = 14.003074;

/// Monoisotopic mass of Oxygen-16 (Da).
pub const MASS_O: f64 = 15.994915;

/// Average atomic mass of Carbon (Da).
pub const AVG_MASS_C: f64 = 12.011;

/// Average atomic mass of Hydrogen (Da).
pub const AVG_MASS_H: f64 = 1.00794;

/// Average atomic mass of Nitrogen (Da).
pub const AVG_MASS_N: f64 = 14.007;

/// Average atomic mass of Oxygen (Da).
pub const AVG_MASS_O: f64 = 15.999;

/// Mass difference between C-13 and C-12 (Da).
pub const C13_C12_DELTA: f64 = MASS_C13 - MASS_C12;

// ── Enums ───────────────────────────────────────────────────────────────────

/// Ionization mode used by the mass spectrometer.
///
/// The ionization mode affects the type of ions produced and thus
/// which adducts to expect in the spectrum.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum IonizationMode {
    /// Electrospray ionization, positive mode. Produces [M+H]+, [M+Na]+, etc.
    EsiPositive,
    /// Electrospray ionization, negative mode. Produces [M-H]-, [M+Cl]-, etc.
    EsiNegative,
    /// Matrix-Assisted Laser Desorption/Ionization. Common for proteins/polymers.
    Maldi,
    /// Electron Ionization (70 eV). Produces extensive fragmentation; used in GC-MS.
    Ei,
}

// ── Configuration ───────────────────────────────────────────────────────────

/// Configuration for the mass spectrometry processor.
///
/// # Fields
///
/// * `mass_range` - Observable m/z range in Daltons, e.g. (50.0, 2000.0).
/// * `resolution` - Mass resolving power m/Δm (FWHM definition).
/// * `ionization_mode` - The ionization technique used.
/// * `mass_accuracy_ppm` - Expected mass accuracy in parts-per-million.
#[derive(Debug, Clone)]
pub struct MassSpecConfig {
    /// Observable m/z range (min, max) in Daltons.
    pub mass_range: (f64, f64),
    /// Mass resolution (m / Δm at FWHM).
    pub resolution: f64,
    /// Ionization mode.
    pub ionization_mode: IonizationMode,
    /// Expected mass accuracy in ppm.
    pub mass_accuracy_ppm: f64,
}

// ── Peak ────────────────────────────────────────────────────────────────────

/// A detected peak in a mass spectrum.
///
/// Represents a single ion species at a particular m/z with measured
/// intensity, width, signal-to-noise ratio, and optional charge state.
#[derive(Debug, Clone)]
pub struct Peak {
    /// Mass-to-charge ratio (m/z) in Daltons.
    pub mz: f64,
    /// Signal intensity (arbitrary units).
    pub intensity: f64,
    /// Peak width at half maximum in Daltons.
    pub width_da: f64,
    /// Signal-to-noise ratio.
    pub snr: f64,
    /// Charge state if determined (e.g., 1 for singly charged, 2 for doubly).
    pub charge_state: Option<u8>,
}

// ── MassSpecProcessor ───────────────────────────────────────────────────────

/// Main mass spectrometry signal processor.
///
/// Provides baseline correction, noise estimation, smoothing, peak detection,
/// and centroiding operations on mass spectral data.
///
/// Spectra are represented as slices of `(m/z, intensity)` tuples sorted
/// by ascending m/z.
pub struct MassSpecProcessor {
    config: MassSpecConfig,
}

impl MassSpecProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: MassSpecConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &MassSpecConfig {
        &self.config
    }

    /// Detect peaks in a spectrum above the given SNR threshold.
    ///
    /// A peak is a local maximum whose intensity exceeds `snr_threshold`
    /// times the estimated noise level. Peak width is estimated from the
    /// FWHM of the local profile.
    ///
    /// # Arguments
    ///
    /// * `spectrum` - Sorted slice of (m/z, intensity) pairs.
    /// * `snr_threshold` - Minimum SNR for a peak to be reported.
    ///
    /// # Returns
    ///
    /// Vector of detected [`Peak`]s sorted by descending intensity.
    pub fn detect_peaks(&self, spectrum: &[(f64, f64)], snr_threshold: f64) -> Vec<Peak> {
        if spectrum.len() < 3 {
            return Vec::new();
        }

        let noise = self.noise_estimate(spectrum, 0.25);
        if noise <= 0.0 {
            return Vec::new();
        }

        let mut peaks = Vec::new();

        for i in 1..spectrum.len() - 1 {
            let (mz, intensity) = spectrum[i];
            let prev_int = spectrum[i - 1].1;
            let next_int = spectrum[i + 1].1;

            // Local maximum check
            if intensity > prev_int && intensity > next_int {
                let snr = intensity / noise;
                if snr >= snr_threshold {
                    // Estimate FWHM
                    let half_max = intensity / 2.0;
                    let width = self.estimate_fwhm(spectrum, i, half_max);

                    peaks.push(Peak {
                        mz,
                        intensity,
                        width_da: width,
                        snr,
                        charge_state: None,
                    });
                }
            }
        }

        // Sort by descending intensity
        peaks.sort_by(|a, b| b.intensity.partial_cmp(&a.intensity).unwrap_or(std::cmp::Ordering::Equal));
        peaks
    }

    /// Estimate FWHM (Full Width at Half Maximum) for a peak at index `peak_idx`.
    ///
    /// Searches left and right from the peak to find where intensity drops
    /// below `half_max`, then interpolates to get the exact crossing positions.
    fn estimate_fwhm(&self, spectrum: &[(f64, f64)], peak_idx: usize, half_max: f64) -> f64 {
        let peak_mz = spectrum[peak_idx].0;

        // Search left for half-max crossing
        let mut left_mz = spectrum[peak_idx].0;
        for j in (0..peak_idx).rev() {
            if spectrum[j].1 <= half_max {
                // Linear interpolation between j and j+1
                let frac = if (spectrum[j + 1].1 - spectrum[j].1).abs() > 1e-30 {
                    (half_max - spectrum[j].1) / (spectrum[j + 1].1 - spectrum[j].1)
                } else {
                    0.5
                };
                left_mz = spectrum[j].0 + frac * (spectrum[j + 1].0 - spectrum[j].0);
                break;
            }
            left_mz = spectrum[j].0;
        }

        // Search right for half-max crossing
        let mut right_mz = spectrum[peak_idx].0;
        for j in (peak_idx + 1)..spectrum.len() {
            if spectrum[j].1 <= half_max {
                let frac = if (spectrum[j - 1].1 - spectrum[j].1).abs() > 1e-30 {
                    (half_max - spectrum[j].1) / (spectrum[j - 1].1 - spectrum[j].1)
                } else {
                    0.5
                };
                right_mz = spectrum[j].0 - frac * (spectrum[j].0 - spectrum[j - 1].0);
                break;
            }
            right_mz = spectrum[j].0;
        }

        let width = right_mz - left_mz;
        if width > 0.0 {
            width
        } else {
            // Fallback: use resolution-based width
            peak_mz / self.config.resolution
        }
    }

    /// Compute the centroid of a peak region in profile-mode data.
    ///
    /// Returns the intensity-weighted average m/z and the total (summed)
    /// intensity across the specified region.
    ///
    /// # Arguments
    ///
    /// * `profile_mz` - m/z values of the profile spectrum.
    /// * `profile_intensity` - Intensity values of the profile spectrum.
    /// * `peak_region` - `(start, end)` index range (exclusive end) defining the peak.
    ///
    /// # Returns
    ///
    /// `(centroid_mz, total_intensity)` tuple.
    ///
    /// # Panics
    ///
    /// Returns `(0.0, 0.0)` if the region is empty or total intensity is zero.
    pub fn centroid(
        &self,
        profile_mz: &[f64],
        profile_intensity: &[f64],
        peak_region: (usize, usize),
    ) -> (f64, f64) {
        let (start, end) = peak_region;
        if start >= end || start >= profile_mz.len() || end > profile_mz.len() {
            return (0.0, 0.0);
        }

        let mut weighted_sum = 0.0;
        let mut total_intensity = 0.0;

        for i in start..end {
            weighted_sum += profile_mz[i] * profile_intensity[i];
            total_intensity += profile_intensity[i];
        }

        if total_intensity <= 0.0 {
            return (0.0, 0.0);
        }

        (weighted_sum / total_intensity, total_intensity)
    }

    /// Apply baseline correction using a rolling minimum filter.
    ///
    /// Estimates the baseline as the local minimum within a sliding window
    /// of `window_size` points, then subtracts it from each data point.
    /// Negative intensities are clamped to zero.
    ///
    /// # Arguments
    ///
    /// * `spectrum` - Mutable spectrum to correct in place.
    /// * `window_size` - Number of points in the rolling window (should be odd).
    pub fn baseline_correction(&self, spectrum: &mut Vec<(f64, f64)>, window_size: usize) {
        if spectrum.is_empty() || window_size == 0 {
            return;
        }

        let n = spectrum.len();
        let half = window_size / 2;

        // Compute baseline as local minimum in each window
        let mut baseline = vec![0.0; n];
        for i in 0..n {
            let start = if i >= half { i - half } else { 0 };
            let end = if i + half < n { i + half + 1 } else { n };

            let mut min_val = f64::MAX;
            for j in start..end {
                if spectrum[j].1 < min_val {
                    min_val = spectrum[j].1;
                }
            }
            baseline[i] = min_val;
        }

        // Subtract baseline and clamp
        for i in 0..n {
            spectrum[i].1 = (spectrum[i].1 - baseline[i]).max(0.0);
        }
    }

    /// Estimate the noise level of a spectrum using a percentile-based method.
    ///
    /// Sorts all intensities and returns the value at the given percentile.
    /// This is robust against high-intensity peaks biasing the estimate.
    ///
    /// # Arguments
    ///
    /// * `spectrum` - Spectrum data.
    /// * `percentile` - Percentile to use (0.0 to 1.0), e.g., 0.25 for Q1.
    ///
    /// # Returns
    ///
    /// Estimated noise level (intensity units). Always non-negative.
    pub fn noise_estimate(&self, spectrum: &[(f64, f64)], percentile: f64) -> f64 {
        if spectrum.is_empty() {
            return 0.0;
        }

        let mut intensities: Vec<f64> = spectrum.iter().map(|(_, int)| *int).collect();
        intensities.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));

        let idx = ((intensities.len() as f64 - 1.0) * percentile.clamp(0.0, 1.0)) as usize;
        let idx = idx.min(intensities.len() - 1);
        intensities[idx].max(0.0)
    }

    /// Apply Savitzky-Golay-like polynomial smoothing to a spectrum.
    ///
    /// Uses a simplified moving-average approach with quadratic weighting
    /// (triangle window) to smooth intensity values while preserving peak
    /// positions.
    ///
    /// # Arguments
    ///
    /// * `spectrum` - Mutable spectrum to smooth in place.
    /// * `window` - Smoothing window half-width. Actual window is `2*window + 1`.
    pub fn smooth_spectrum(&self, spectrum: &mut Vec<(f64, f64)>, window: usize) {
        if spectrum.is_empty() || window == 0 {
            return;
        }

        let n = spectrum.len();
        let intensities: Vec<f64> = spectrum.iter().map(|(_, int)| *int).collect();

        for i in 0..n {
            let start = if i >= window { i - window } else { 0 };
            let end = if i + window < n { i + window + 1 } else { n };

            let mut weighted_sum = 0.0;
            let mut weight_total = 0.0;

            for j in start..end {
                // Triangle (Bartlett) weighting: higher weight near center
                let dist = (j as f64 - i as f64).abs();
                let w = 1.0 - dist / (window as f64 + 1.0);
                weighted_sum += intensities[j] * w;
                weight_total += w;
            }

            if weight_total > 0.0 {
                spectrum[i].1 = weighted_sum / weight_total;
            }
        }
    }
}

// ── IsotopePatternAnalyzer ──────────────────────────────────────────────────

/// Analyzes isotope distributions for charge state determination and deisotoping.
///
/// Isotope patterns arise from the natural abundance of heavy isotopes
/// (e.g., C-13 at 1.1%). For multiply-charged ions, isotope peaks are
/// spaced at Δm/z = ~1.003/z Da apart.
pub struct IsotopePatternAnalyzer;

impl IsotopePatternAnalyzer {
    /// Generate a simplified theoretical isotope pattern based on the averagine model.
    ///
    /// The averagine model approximates the average amino acid as C4.9384 H7.7583
    /// N1.3577 O1.4773 S0.0417. For a given molecular formula string (simplified
    /// format like "C6H12O6"), the method computes approximate isotope envelope
    /// intensities using a binomial approximation for C-13.
    ///
    /// # Arguments
    ///
    /// * `formula` - Chemical formula string (supports C, H, N, O with counts).
    /// * `num_peaks` - Number of isotope peaks to generate.
    ///
    /// # Returns
    ///
    /// Vector of `(mass_offset, relative_intensity)` pairs where the
    /// monoisotopic peak (M+0) has relative intensity of 1.0.
    pub fn theoretical_isotope_pattern(formula: &str, num_peaks: usize) -> Vec<(f64, f64)> {
        if num_peaks == 0 {
            return Vec::new();
        }

        // Parse the formula to count carbons (main contributor to isotope pattern)
        let num_carbons = Self::parse_element_count(formula, 'C');

        // C-13 natural abundance
        let p_c13 = 0.0107;
        let n = num_carbons as f64;

        // Binomial approximation: P(k) = C(n,k) * p^k * (1-p)^(n-k)
        let mut pattern = Vec::with_capacity(num_peaks);
        let mut max_intensity = 0.0_f64;

        for k in 0..num_peaks {
            let intensity = binomial_prob(num_carbons, k, p_c13);
            if intensity > max_intensity {
                max_intensity = intensity;
            }
            let mass_offset = k as f64 * C13_C12_DELTA;
            pattern.push((mass_offset, intensity));
        }

        // Normalize to monoisotopic = 1.0 (the M+0 peak)
        let mono_int = if !pattern.is_empty() && pattern[0].1 > 0.0 {
            pattern[0].1
        } else {
            1.0
        };
        for p in &mut pattern {
            p.1 /= mono_int;
        }

        pattern
    }

    /// Determine the charge state from the spacing between adjacent peaks.
    ///
    /// For multiply-charged ions, isotope peaks are separated by
    /// approximately 1.003355 / z Da. This method examines consecutive
    /// peak spacings and estimates the most likely charge state.
    ///
    /// # Arguments
    ///
    /// * `peaks` - Slice of detected peaks sorted by m/z.
    ///
    /// # Returns
    ///
    /// `Some(z)` if a consistent charge state is found, `None` otherwise.
    pub fn charge_state_from_spacing(peaks: &[Peak]) -> Option<u8> {
        if peaks.len() < 2 {
            return None;
        }

        // Sort peaks by m/z
        let mut sorted: Vec<&Peak> = peaks.iter().collect();
        sorted.sort_by(|a, b| a.mz.partial_cmp(&b.mz).unwrap_or(std::cmp::Ordering::Equal));

        // Collect spacings between consecutive peaks
        let mut spacings = Vec::new();
        for i in 1..sorted.len() {
            let spacing = sorted[i].mz - sorted[i - 1].mz;
            if spacing > 0.01 {
                spacings.push(spacing);
            }
        }

        if spacings.is_empty() {
            return None;
        }

        // Median spacing
        spacings.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median_spacing = spacings[spacings.len() / 2];

        // Charge state z = C13_C12_DELTA / spacing
        // Try z = 1 through 10
        let mut best_z = 1u8;
        let mut best_error = f64::MAX;

        for z in 1..=10u8 {
            let expected = C13_C12_DELTA / z as f64;
            let error = (median_spacing - expected).abs();
            if error < best_error {
                best_error = error;
                best_z = z;
            }
        }

        // Accept if error is within 20% of expected spacing
        let expected = C13_C12_DELTA / best_z as f64;
        if best_error < expected * 0.2 {
            Some(best_z)
        } else {
            None
        }
    }

    /// Remove isotope peaks from a peak list, keeping only monoisotopic peaks.
    ///
    /// For each peak, checks if there is a heavier peak at approximately
    /// +1.003/z Da. If so, the heavier peak is considered an isotope peak
    /// and is removed. The charge state of the monoisotopic peak is updated.
    ///
    /// # Arguments
    ///
    /// * `peaks` - Mutable peak list to deisotope in place.
    pub fn deisotope(peaks: &mut Vec<Peak>) {
        if peaks.len() < 2 {
            return;
        }

        // Sort by m/z ascending
        peaks.sort_by(|a, b| a.mz.partial_cmp(&b.mz).unwrap_or(std::cmp::Ordering::Equal));

        let mut is_isotope = vec![false; peaks.len()];

        for i in 0..peaks.len() {
            if is_isotope[i] {
                continue;
            }

            // Try charge states z = 1 through 5
            for z in 1..=5u8 {
                let expected_spacing = C13_C12_DELTA / z as f64;
                let tolerance = expected_spacing * 0.15;

                // Look for isotope peaks at M+1, M+2, etc.
                let mut found_isotope = false;
                for k in 1..=5 {
                    let expected_mz = peaks[i].mz + k as f64 * expected_spacing;

                    for j in (i + 1)..peaks.len() {
                        if peaks[j].mz > expected_mz + tolerance {
                            break;
                        }
                        if (peaks[j].mz - expected_mz).abs() < tolerance {
                            // Check that isotope peak is less intense (for small molecules)
                            // or comparable (for larger molecules)
                            if peaks[j].intensity <= peaks[i].intensity * 2.0 {
                                is_isotope[j] = true;
                                found_isotope = true;
                            }
                        }
                    }
                }

                if found_isotope {
                    peaks[i].charge_state = Some(z);
                    break;
                }
            }
        }

        // Remove isotope peaks (iterate in reverse to preserve indices)
        let mut idx = 0;
        peaks.retain(|_| {
            let keep = !is_isotope[idx];
            idx += 1;
            keep
        });
    }

    /// Compute average molecular mass from element counts and average masses.
    ///
    /// # Arguments
    ///
    /// * `formula_weights` - Slice of `(average_atomic_mass, count)` pairs.
    ///
    /// # Returns
    ///
    /// Sum of (mass * count) for all elements.
    pub fn average_mass(formula_weights: &[(f64, usize)]) -> f64 {
        formula_weights.iter().map(|(mass, count)| mass * *count as f64).sum()
    }

    /// Compute monoisotopic mass from element counts and monoisotopic masses.
    ///
    /// The monoisotopic mass uses the lightest stable isotope of each element.
    ///
    /// # Arguments
    ///
    /// * `formula_weights` - Slice of `(monoisotopic_mass, count)` pairs.
    ///
    /// # Returns
    ///
    /// Sum of (mass * count) for all elements.
    pub fn monoisotopic_mass(formula_weights: &[(f64, usize)]) -> f64 {
        formula_weights.iter().map(|(mass, count)| mass * *count as f64).sum()
    }

    /// Parse the count of a specific element from a simple formula string.
    ///
    /// Handles formats like "C6H12O6", "C2H6", "H2O". If no count follows
    /// the element letter, assumes 1.
    fn parse_element_count(formula: &str, element: char) -> usize {
        let chars: Vec<char> = formula.chars().collect();
        let mut i = 0;
        while i < chars.len() {
            if chars[i] == element {
                // Check next chars aren't lowercase (would be different element)
                if i + 1 < chars.len() && chars[i + 1].is_lowercase() {
                    i += 1;
                    continue;
                }
                // Parse following digits
                let mut num_str = String::new();
                let mut j = i + 1;
                while j < chars.len() && chars[j].is_ascii_digit() {
                    num_str.push(chars[j]);
                    j += 1;
                }
                return if num_str.is_empty() {
                    1
                } else {
                    num_str.parse().unwrap_or(1)
                };
            }
            i += 1;
        }
        0
    }
}

// ── MolecularFormulaAssigner ────────────────────────────────────────────────

/// Assigns molecular formulas to observed masses based on mass accuracy.
///
/// Provides tools for computing mass errors, mass defects, and Kendrick
/// mass analysis for homologous series identification.
pub struct MolecularFormulaAssigner;

impl MolecularFormulaAssigner {
    /// Compute the mass error in parts-per-million (ppm).
    ///
    /// ppm = (observed - theoretical) / theoretical * 1e6
    ///
    /// # Arguments
    ///
    /// * `theoretical_mz` - Expected exact mass (Da).
    /// * `observed_mz` - Measured mass (Da).
    ///
    /// # Returns
    ///
    /// Mass error in ppm. Positive means observed > theoretical.
    pub fn ppm_error(theoretical_mz: f64, observed_mz: f64) -> f64 {
        if theoretical_mz.abs() < 1e-30 {
            return 0.0;
        }
        (observed_mz - theoretical_mz) / theoretical_mz * 1e6
    }

    /// Search candidate formulas and return those within the given ppm tolerance.
    ///
    /// # Arguments
    ///
    /// * `observed_mz` - Measured m/z value.
    /// * `tolerance_ppm` - Maximum absolute ppm error allowed.
    /// * `candidates` - Slice of `(name, exact_mass)` pairs to search.
    ///
    /// # Returns
    ///
    /// Vector of `(name, exact_mass, ppm_error)` for matching candidates,
    /// sorted by ascending absolute ppm error.
    pub fn assign_formula(
        observed_mz: f64,
        tolerance_ppm: f64,
        candidates: &[(String, f64)],
    ) -> Vec<(String, f64, f64)> {
        let mut matches: Vec<(String, f64, f64)> = candidates
            .iter()
            .filter_map(|(name, mass)| {
                let ppm = Self::ppm_error(*mass, observed_mz);
                if ppm.abs() <= tolerance_ppm {
                    Some((name.clone(), *mass, ppm))
                } else {
                    None
                }
            })
            .collect();

        matches.sort_by(|a, b| {
            a.2.abs()
                .partial_cmp(&b.2.abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        matches
    }

    /// Compute the mass defect (fractional part of the mass).
    ///
    /// Mass defect is the deviation from the nearest integer mass and is
    /// useful for identifying compound classes (e.g., lipids vs peptides).
    ///
    /// # Arguments
    ///
    /// * `mass` - Molecular mass in Da.
    ///
    /// # Returns
    ///
    /// Fractional part of the mass (always in [0, 1)).
    pub fn mass_defect(mass: f64) -> f64 {
        mass - mass.floor()
    }

    /// Compute the Kendrick mass for homologous series analysis.
    ///
    /// Kendrick mass rescales the mass so that a chosen repeat unit
    /// has an exact integer mass, revealing homologous series as
    /// horizontal lines in Kendrick mass vs nominal mass plots.
    ///
    /// Kendrick mass = observed_mass * (nominal_mass / base_unit_mass)
    ///
    /// For CH2 series: base_unit_mass = 14.01565, nominal_mass = 14.
    ///
    /// # Arguments
    ///
    /// * `mass` - Observed mass in Da.
    /// * `base_unit_mass` - Exact mass of the repeat unit (e.g., 14.01565 for CH2).
    /// * `nominal_mass` - Integer mass of the repeat unit (e.g., 14 for CH2).
    ///
    /// # Returns
    ///
    /// Kendrick mass (Da).
    pub fn kendrick_mass(mass: f64, base_unit_mass: f64, nominal_mass: f64) -> f64 {
        if base_unit_mass.abs() < 1e-30 {
            return mass;
        }
        mass * (nominal_mass / base_unit_mass)
    }
}

// ── Quantifier ──────────────────────────────────────────────────────────────

/// Quantification tools for mass spectrometry.
///
/// Provides peak area integration, calibration curve fitting, limit of
/// detection calculation, and extracted ion chromatogram generation.
pub struct Quantifier;

impl Quantifier {
    /// Integrate peak area within a mass window using the trapezoidal rule.
    ///
    /// # Arguments
    ///
    /// * `spectrum` - Sorted (m/z, intensity) data.
    /// * `mz_center` - Center m/z for integration.
    /// * `window_da` - Total width of the integration window in Da.
    ///
    /// # Returns
    ///
    /// Integrated area (intensity * Da units).
    pub fn peak_area(spectrum: &[(f64, f64)], mz_center: f64, window_da: f64) -> f64 {
        let half = window_da / 2.0;
        let low = mz_center - half;
        let high = mz_center + half;

        let mut area = 0.0;
        let mut prev: Option<(f64, f64)> = None;

        for &(mz, intensity) in spectrum {
            if mz < low {
                prev = Some((mz, intensity));
                continue;
            }
            if mz > high {
                // Add trapezoidal area for last segment crossing boundary
                if let Some((pm, pi)) = prev {
                    if pm >= low {
                        area += (mz.min(high) - pm) * (pi + intensity) / 2.0;
                    }
                }
                break;
            }

            if let Some((pm, pi)) = prev {
                if pm >= low {
                    area += (mz - pm) * (pi + intensity) / 2.0;
                } else {
                    // Interpolate into the window
                    let frac = if (mz - pm).abs() > 1e-30 {
                        (low - pm) / (mz - pm)
                    } else {
                        0.0
                    };
                    let interp_int = pi + frac * (intensity - pi);
                    area += (mz - low) * (interp_int + intensity) / 2.0;
                }
            }
            prev = Some((mz, intensity));
        }

        area.max(0.0)
    }

    /// Fit a linear calibration curve to concentration vs response data.
    ///
    /// Uses ordinary least-squares regression: response = slope * concentration + intercept.
    ///
    /// # Arguments
    ///
    /// * `concentrations` - Known analyte concentrations.
    /// * `responses` - Measured signal responses.
    ///
    /// # Returns
    ///
    /// `(slope, intercept)` of the best-fit line. Returns `(0.0, 0.0)` if
    /// fewer than 2 points or degenerate data.
    pub fn calibration_curve(concentrations: &[f64], responses: &[f64]) -> (f64, f64) {
        let n = concentrations.len().min(responses.len());
        if n < 2 {
            return (0.0, 0.0);
        }

        let n_f = n as f64;
        let sum_x: f64 = concentrations[..n].iter().sum();
        let sum_y: f64 = responses[..n].iter().sum();
        let sum_xy: f64 = concentrations[..n]
            .iter()
            .zip(responses[..n].iter())
            .map(|(x, y)| x * y)
            .sum();
        let sum_x2: f64 = concentrations[..n].iter().map(|x| x * x).sum();

        let denom = n_f * sum_x2 - sum_x * sum_x;
        if denom.abs() < 1e-30 {
            return (0.0, 0.0);
        }

        let slope = (n_f * sum_xy - sum_x * sum_y) / denom;
        let intercept = (sum_y - slope * sum_x) / n_f;

        (slope, intercept)
    }

    /// Calculate the limit of detection (LOD) from blank measurements.
    ///
    /// LOD = mean(blank) + 3 * std(blank)
    ///
    /// This is the signal level below which a true analyte signal cannot
    /// be reliably distinguished from noise.
    ///
    /// # Arguments
    ///
    /// * `blank_signals` - Measured signals from blank (analyte-free) samples.
    ///
    /// # Returns
    ///
    /// Limit of detection value. Returns 0.0 if no data provided.
    pub fn limit_of_detection(blank_signals: &[f64]) -> f64 {
        if blank_signals.is_empty() {
            return 0.0;
        }

        let n = blank_signals.len() as f64;
        let mean: f64 = blank_signals.iter().sum::<f64>() / n;

        if blank_signals.len() < 2 {
            return mean;
        }

        let variance: f64 = blank_signals.iter().map(|x| (x - mean).powi(2)).sum::<f64>() / (n - 1.0);
        let std_dev = variance.sqrt();

        mean + 3.0 * std_dev
    }

    /// Extract an ion chromatogram (XIC) from a series of spectra.
    ///
    /// For each spectrum in the time series, finds the maximum intensity
    /// within `tolerance_da` of the target m/z. This produces a time trace
    /// of a specific ion's abundance.
    ///
    /// # Arguments
    ///
    /// * `spectra` - Slice of spectra (one per time point), each sorted by m/z.
    /// * `target_mz` - Target m/z value to extract.
    /// * `tolerance_da` - Mass tolerance window in Da.
    ///
    /// # Returns
    ///
    /// Vector of intensities, one per input spectrum.
    pub fn extract_ion_chromatogram(
        spectra: &[Vec<(f64, f64)>],
        target_mz: f64,
        tolerance_da: f64,
    ) -> Vec<f64> {
        spectra
            .iter()
            .map(|spectrum| {
                let mut max_int = 0.0_f64;
                for &(mz, intensity) in spectrum.iter() {
                    if (mz - target_mz).abs() <= tolerance_da && intensity > max_int {
                        max_int = intensity;
                    }
                }
                max_int
            })
            .collect()
    }
}

// ── Helper functions ────────────────────────────────────────────────────────

/// Compute binomial probability P(X=k) for n trials with success probability p.
///
/// Uses the log-space computation to avoid overflow for large n:
/// log P = log(C(n,k)) + k*log(p) + (n-k)*log(1-p)
fn binomial_prob(n: usize, k: usize, p: f64) -> f64 {
    if k > n {
        return 0.0;
    }
    if p <= 0.0 {
        return if k == 0 { 1.0 } else { 0.0 };
    }
    if p >= 1.0 {
        return if k == n { 1.0 } else { 0.0 };
    }

    // log C(n,k) = sum(log(n-i+1) - log(i)) for i=1..k
    let mut log_coeff = 0.0;
    for i in 1..=k {
        log_coeff += ((n - i + 1) as f64).ln() - (i as f64).ln();
    }

    let log_prob = log_coeff + k as f64 * p.ln() + (n - k) as f64 * (1.0 - p).ln();
    log_prob.exp()
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> MassSpecConfig {
        MassSpecConfig {
            mass_range: (50.0, 2000.0),
            resolution: 10000.0,
            ionization_mode: IonizationMode::EsiPositive,
            mass_accuracy_ppm: 5.0,
        }
    }

    fn make_gaussian_peak(center: f64, height: f64, sigma: f64, num_points: usize) -> Vec<(f64, f64)> {
        let mut spectrum = Vec::with_capacity(num_points);
        let start = center - 4.0 * sigma;
        let step = 8.0 * sigma / (num_points as f64 - 1.0);
        for i in 0..num_points {
            let mz = start + i as f64 * step;
            let intensity = height * (-(mz - center).powi(2) / (2.0 * sigma * sigma)).exp();
            spectrum.push((mz, intensity));
        }
        spectrum
    }

    // ── MassSpecProcessor tests ─────────────────────────────────────────

    #[test]
    fn test_centroid_at_peak_center() {
        let proc = MassSpecProcessor::new(default_config());
        let mz = vec![100.0, 100.1, 100.2, 100.3, 100.4];
        let intensity = vec![10.0, 50.0, 100.0, 50.0, 10.0];
        let (centroid_mz, total_int) = proc.centroid(&mz, &intensity, (0, 5));
        // Symmetric peak should centroid at center
        assert!((centroid_mz - 100.2).abs() < 0.001, "centroid={centroid_mz}");
        assert!((total_int - 220.0).abs() < 0.001, "total={total_int}");
    }

    #[test]
    fn test_centroid_asymmetric_peak() {
        let proc = MassSpecProcessor::new(default_config());
        let mz = vec![500.0, 500.1, 500.2, 500.3];
        let intensity = vec![0.0, 100.0, 200.0, 0.0];
        let (centroid_mz, total_int) = proc.centroid(&mz, &intensity, (0, 4));
        // Weighted toward 500.2
        assert!(centroid_mz > 500.15, "centroid should be right of center: {centroid_mz}");
        assert!((total_int - 300.0).abs() < 0.001);
    }

    #[test]
    fn test_centroid_empty_region() {
        let proc = MassSpecProcessor::new(default_config());
        let mz = vec![100.0, 100.1];
        let intensity = vec![50.0, 50.0];
        let (c, t) = proc.centroid(&mz, &intensity, (5, 10));
        assert_eq!(c, 0.0);
        assert_eq!(t, 0.0);
    }

    #[test]
    fn test_noise_estimate_positive() {
        let proc = MassSpecProcessor::new(default_config());
        let spectrum: Vec<(f64, f64)> = (0..100)
            .map(|i| (i as f64, 10.0 + (i as f64 * 0.1)))
            .collect();
        let noise = proc.noise_estimate(&spectrum, 0.25);
        assert!(noise > 0.0, "noise must be positive: {noise}");
    }

    #[test]
    fn test_noise_estimate_empty() {
        let proc = MassSpecProcessor::new(default_config());
        assert_eq!(proc.noise_estimate(&[], 0.5), 0.0);
    }

    #[test]
    fn test_noise_estimate_percentile_ordering() {
        let proc = MassSpecProcessor::new(default_config());
        let spectrum: Vec<(f64, f64)> = (0..100)
            .map(|i| (i as f64, i as f64))
            .collect();
        let q25 = proc.noise_estimate(&spectrum, 0.25);
        let q75 = proc.noise_estimate(&spectrum, 0.75);
        assert!(q75 > q25, "q75={q75} should exceed q25={q25}");
    }

    #[test]
    fn test_detect_peaks_single_peak() {
        let proc = MassSpecProcessor::new(default_config());
        let mut spectrum = make_gaussian_peak(500.0, 1000.0, 0.05, 101);
        // Add noise floor
        for s in &mut spectrum {
            s.1 += 5.0;
        }
        let peaks = proc.detect_peaks(&spectrum, 3.0);
        assert!(!peaks.is_empty(), "should find at least one peak");
        assert!((peaks[0].mz - 500.0).abs() < 0.1, "peak near 500 Da: {}", peaks[0].mz);
    }

    #[test]
    fn test_detect_peaks_multiple() {
        let proc = MassSpecProcessor::new(default_config());
        let mut spectrum = Vec::new();
        // Two well-separated peaks
        for p in make_gaussian_peak(300.0, 500.0, 0.05, 51) {
            spectrum.push(p);
        }
        for p in make_gaussian_peak(600.0, 800.0, 0.05, 51) {
            spectrum.push(p);
        }
        spectrum.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

        let peaks = proc.detect_peaks(&spectrum, 2.0);
        assert!(peaks.len() >= 2, "should find 2 peaks, found {}", peaks.len());
    }

    #[test]
    fn test_detect_peaks_empty() {
        let proc = MassSpecProcessor::new(default_config());
        let peaks = proc.detect_peaks(&[], 3.0);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_detect_peaks_below_threshold() {
        let proc = MassSpecProcessor::new(default_config());
        // Very low SNR - all signal is noise-level
        let spectrum: Vec<(f64, f64)> = (0..50)
            .map(|i| (i as f64, 10.0 + (i % 2) as f64))
            .collect();
        let peaks = proc.detect_peaks(&spectrum, 100.0);
        assert!(peaks.is_empty(), "no peaks should exceed SNR=100");
    }

    #[test]
    fn test_baseline_correction_flattens() {
        let proc = MassSpecProcessor::new(default_config());
        // Spectrum with rising baseline
        let mut spectrum: Vec<(f64, f64)> = (0..100)
            .map(|i| (i as f64, 50.0 + i as f64 * 2.0))
            .collect();
        // Add a peak at index 50
        spectrum[50].1 += 500.0;

        let original_baseline_end = spectrum[99].1;
        proc.baseline_correction(&mut spectrum, 21);

        // After correction, the end should be much lower than original
        assert!(
            spectrum[99].1 < original_baseline_end * 0.5,
            "baseline should be reduced: {} vs {}",
            spectrum[99].1,
            original_baseline_end
        );
        // Peak should still be present
        let peak_int = spectrum[50].1;
        assert!(peak_int > 100.0, "peak should survive correction: {peak_int}");
    }

    #[test]
    fn test_baseline_correction_empty() {
        let proc = MassSpecProcessor::new(default_config());
        let mut spectrum: Vec<(f64, f64)> = Vec::new();
        proc.baseline_correction(&mut spectrum, 5);
        assert!(spectrum.is_empty());
    }

    #[test]
    fn test_smooth_spectrum() {
        let proc = MassSpecProcessor::new(default_config());
        let mut spectrum: Vec<(f64, f64)> = (0..100)
            .map(|i| {
                let noise = if i % 2 == 0 { 5.0 } else { -5.0 };
                (i as f64, 100.0 + noise)
            })
            .collect();

        proc.smooth_spectrum(&mut spectrum, 3);

        // After smoothing, variance should be reduced
        let mean: f64 = spectrum.iter().map(|(_, int)| *int).sum::<f64>() / spectrum.len() as f64;
        let variance: f64 = spectrum.iter().map(|(_, int)| (int - mean).powi(2)).sum::<f64>()
            / spectrum.len() as f64;
        assert!(variance < 25.0, "smoothing should reduce variance: {variance}");
    }

    #[test]
    fn test_smooth_spectrum_empty() {
        let proc = MassSpecProcessor::new(default_config());
        let mut spectrum: Vec<(f64, f64)> = Vec::new();
        proc.smooth_spectrum(&mut spectrum, 3);
        assert!(spectrum.is_empty());
    }

    // ── IsotopePatternAnalyzer tests ────────────────────────────────────

    #[test]
    fn test_charge_state_z2_from_half_da_spacing() {
        let peaks = vec![
            Peak { mz: 500.0, intensity: 1000.0, width_da: 0.05, snr: 100.0, charge_state: None },
            Peak { mz: 500.0 + C13_C12_DELTA / 2.0, intensity: 500.0, width_da: 0.05, snr: 50.0, charge_state: None },
        ];
        let z = IsotopePatternAnalyzer::charge_state_from_spacing(&peaks);
        assert_eq!(z, Some(2), "spacing of ~0.5 Da implies z=2");
    }

    #[test]
    fn test_charge_state_z1_from_full_da_spacing() {
        let peaks = vec![
            Peak { mz: 300.0, intensity: 1000.0, width_da: 0.05, snr: 100.0, charge_state: None },
            Peak { mz: 300.0 + C13_C12_DELTA, intensity: 400.0, width_da: 0.05, snr: 40.0, charge_state: None },
        ];
        let z = IsotopePatternAnalyzer::charge_state_from_spacing(&peaks);
        assert_eq!(z, Some(1), "spacing of ~1.003 Da implies z=1");
    }

    #[test]
    fn test_charge_state_z3() {
        let spacing = C13_C12_DELTA / 3.0;
        let peaks = vec![
            Peak { mz: 800.0, intensity: 1000.0, width_da: 0.02, snr: 100.0, charge_state: None },
            Peak { mz: 800.0 + spacing, intensity: 700.0, width_da: 0.02, snr: 70.0, charge_state: None },
            Peak { mz: 800.0 + 2.0 * spacing, intensity: 300.0, width_da: 0.02, snr: 30.0, charge_state: None },
        ];
        let z = IsotopePatternAnalyzer::charge_state_from_spacing(&peaks);
        assert_eq!(z, Some(3));
    }

    #[test]
    fn test_charge_state_insufficient_peaks() {
        let peaks = vec![
            Peak { mz: 500.0, intensity: 1000.0, width_da: 0.05, snr: 100.0, charge_state: None },
        ];
        assert_eq!(IsotopePatternAnalyzer::charge_state_from_spacing(&peaks), None);
    }

    #[test]
    fn test_deisotope_removes_isotope_peaks() {
        let mut peaks = vec![
            Peak { mz: 400.0, intensity: 1000.0, width_da: 0.05, snr: 100.0, charge_state: None },
            Peak { mz: 400.0 + C13_C12_DELTA, intensity: 300.0, width_da: 0.05, snr: 30.0, charge_state: None },
            Peak { mz: 400.0 + 2.0 * C13_C12_DELTA, intensity: 50.0, width_da: 0.05, snr: 5.0, charge_state: None },
            Peak { mz: 700.0, intensity: 800.0, width_da: 0.05, snr: 80.0, charge_state: None },
        ];

        IsotopePatternAnalyzer::deisotope(&mut peaks);

        // Should keep only monoisotopic peaks
        assert_eq!(peaks.len(), 2, "should have 2 monoisotopic peaks, got {}", peaks.len());
        assert!((peaks[0].mz - 400.0).abs() < 0.01);
        assert!((peaks[1].mz - 700.0).abs() < 0.01);
        assert_eq!(peaks[0].charge_state, Some(1));
    }

    #[test]
    fn test_deisotope_empty() {
        let mut peaks = Vec::new();
        IsotopePatternAnalyzer::deisotope(&mut peaks);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_theoretical_isotope_pattern() {
        // C6H12O6 (glucose) - 6 carbons
        let pattern = IsotopePatternAnalyzer::theoretical_isotope_pattern("C6H12O6", 4);
        assert_eq!(pattern.len(), 4);
        // M+0 should be normalized to 1.0
        assert!((pattern[0].1 - 1.0).abs() < 1e-6, "M+0 = {}", pattern[0].1);
        // M+1 should be ~6.5% of M+0 for 6 carbons (6 * 1.07%)
        assert!(pattern[1].1 > 0.03 && pattern[1].1 < 0.15, "M+1 = {}", pattern[1].1);
        // Mass offset should be approximately C13_C12_DELTA
        assert!((pattern[1].0 - C13_C12_DELTA).abs() < 0.001);
    }

    #[test]
    fn test_theoretical_isotope_pattern_no_carbon() {
        let pattern = IsotopePatternAnalyzer::theoretical_isotope_pattern("H2O", 3);
        // No carbons means M+0 = 1.0, M+1 ~ 0, etc.
        assert_eq!(pattern.len(), 3);
        assert!((pattern[0].1 - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_average_mass() {
        // Water H2O: 2*1.00794 + 1*15.999 = 18.01488
        let mass = IsotopePatternAnalyzer::average_mass(&[(AVG_MASS_H, 2), (AVG_MASS_O, 1)]);
        assert!((mass - 18.01488).abs() < 0.001, "H2O avg mass = {mass}");
    }

    #[test]
    fn test_monoisotopic_mass() {
        // Water H2O: 2*1.00783 + 1*15.994915 = 18.010495
        let mass = IsotopePatternAnalyzer::monoisotopic_mass(&[(MASS_H, 2), (MASS_O, 1)]);
        assert!((mass - 18.01049).abs() < 0.001, "H2O mono mass = {mass}");
    }

    #[test]
    fn test_monoisotopic_vs_average_mass() {
        // Average mass should be slightly higher than monoisotopic for most molecules
        let mono = IsotopePatternAnalyzer::monoisotopic_mass(&[(MASS_C12, 6), (MASS_H, 12), (MASS_O, 6)]);
        let avg = IsotopePatternAnalyzer::average_mass(&[(AVG_MASS_C, 6), (AVG_MASS_H, 12), (AVG_MASS_O, 6)]);
        assert!(avg > mono, "average ({avg}) should exceed monoisotopic ({mono})");
    }

    #[test]
    fn test_parse_element_count() {
        assert_eq!(IsotopePatternAnalyzer::parse_element_count("C6H12O6", 'C'), 6);
        assert_eq!(IsotopePatternAnalyzer::parse_element_count("C6H12O6", 'H'), 12);
        assert_eq!(IsotopePatternAnalyzer::parse_element_count("C6H12O6", 'O'), 6);
        assert_eq!(IsotopePatternAnalyzer::parse_element_count("H2O", 'C'), 0);
        assert_eq!(IsotopePatternAnalyzer::parse_element_count("CH4", 'C'), 1);
        assert_eq!(IsotopePatternAnalyzer::parse_element_count("CH4", 'H'), 4);
    }

    // ── MolecularFormulaAssigner tests ──────────────────────────────────

    #[test]
    fn test_ppm_error_exact_mass() {
        let ppm = MolecularFormulaAssigner::ppm_error(500.0, 500.0);
        assert!((ppm).abs() < 1e-10, "exact mass should give 0 ppm: {ppm}");
    }

    #[test]
    fn test_ppm_error_offset() {
        // 1 ppm at 1000 Da = 0.001 Da
        let ppm = MolecularFormulaAssigner::ppm_error(1000.0, 1000.001);
        assert!((ppm - 1.0).abs() < 0.01, "ppm = {ppm}");
    }

    #[test]
    fn test_ppm_error_negative() {
        let ppm = MolecularFormulaAssigner::ppm_error(500.0, 499.999);
        assert!(ppm < 0.0, "observed < theoretical should give negative ppm");
    }

    #[test]
    fn test_assign_formula() {
        let candidates = vec![
            ("Glucose".to_string(), 180.0634),
            ("Caffeine".to_string(), 194.0804),
            ("Aspirin".to_string(), 180.0423),
        ];

        let matches = MolecularFormulaAssigner::assign_formula(180.0634, 5.0, &candidates);
        assert!(!matches.is_empty(), "should find at least glucose");
        assert_eq!(matches[0].0, "Glucose");
        assert!(matches[0].2.abs() < 1.0, "best match ppm = {}", matches[0].2);
    }

    #[test]
    fn test_assign_formula_no_match() {
        let candidates = vec![("Water".to_string(), 18.0105)];
        let matches = MolecularFormulaAssigner::assign_formula(500.0, 5.0, &candidates);
        assert!(matches.is_empty(), "no candidate near 500 Da");
    }

    #[test]
    fn test_mass_defect_less_than_one() {
        let defect = MolecularFormulaAssigner::mass_defect(180.0634);
        assert!(defect >= 0.0 && defect < 1.0, "mass defect = {defect}");
        assert!((defect - 0.0634).abs() < 0.001);
    }

    #[test]
    fn test_mass_defect_integer_mass() {
        let defect = MolecularFormulaAssigner::mass_defect(200.0);
        assert!(defect.abs() < 1e-10, "integer mass has zero defect: {defect}");
    }

    #[test]
    fn test_kendrick_mass_ch2() {
        // CH2 base: exact = 14.01565, nominal = 14
        let mass = 200.156;
        let km = MolecularFormulaAssigner::kendrick_mass(mass, 14.01565, 14.0);
        // Kendrick mass should be slightly less than IUPAC mass
        assert!(km < mass, "Kendrick mass ({km}) should be < IUPAC mass ({mass})");
        assert!((km - mass).abs() < 1.0, "difference should be small");
    }

    #[test]
    fn test_kendrick_mass_zero_base() {
        let km = MolecularFormulaAssigner::kendrick_mass(500.0, 0.0, 14.0);
        assert_eq!(km, 500.0, "zero base should return original mass");
    }

    // ── Quantifier tests ────────────────────────────────────────────────

    #[test]
    fn test_peak_area_proportional_to_intensity() {
        // Peak with double height should give double area
        let spectrum1: Vec<(f64, f64)> = (0..101)
            .map(|i| {
                let mz = 499.0 + i as f64 * 0.02;
                let intensity = 100.0 * (-(mz - 500.0).powi(2) / (2.0 * 0.1 * 0.1)).exp();
                (mz, intensity)
            })
            .collect();

        let spectrum2: Vec<(f64, f64)> = spectrum1
            .iter()
            .map(|(mz, int)| (*mz, int * 2.0))
            .collect();

        let area1 = Quantifier::peak_area(&spectrum1, 500.0, 0.5);
        let area2 = Quantifier::peak_area(&spectrum2, 500.0, 0.5);

        assert!(area1 > 0.0, "area should be positive: {area1}");
        assert!((area2 / area1 - 2.0).abs() < 0.01, "ratio = {}", area2 / area1);
    }

    #[test]
    fn test_peak_area_empty() {
        assert_eq!(Quantifier::peak_area(&[], 500.0, 1.0), 0.0);
    }

    #[test]
    fn test_calibration_curve_roundtrip() {
        let concentrations = vec![0.0, 10.0, 20.0, 30.0, 40.0];
        let responses = vec![5.0, 15.0, 25.0, 35.0, 45.0]; // slope=1, intercept=5

        let (slope, intercept) = Quantifier::calibration_curve(&concentrations, &responses);
        assert!((slope - 1.0).abs() < 0.01, "slope = {slope}");
        assert!((intercept - 5.0).abs() < 0.01, "intercept = {intercept}");
    }

    #[test]
    fn test_calibration_curve_insufficient_data() {
        let (slope, intercept) = Quantifier::calibration_curve(&[1.0], &[2.0]);
        assert_eq!(slope, 0.0);
        assert_eq!(intercept, 0.0);
    }

    #[test]
    fn test_lod_greater_than_blank_mean() {
        let blanks = vec![10.0, 12.0, 11.0, 9.0, 10.5, 11.5, 10.0, 11.0];
        let lod = Quantifier::limit_of_detection(&blanks);
        let mean: f64 = blanks.iter().sum::<f64>() / blanks.len() as f64;
        assert!(lod > mean, "LOD ({lod}) should exceed blank mean ({mean})");
    }

    #[test]
    fn test_lod_empty() {
        assert_eq!(Quantifier::limit_of_detection(&[]), 0.0);
    }

    #[test]
    fn test_lod_single_value() {
        let lod = Quantifier::limit_of_detection(&[5.0]);
        assert!((lod - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_extract_ion_chromatogram() {
        let spectra = vec![
            vec![(499.0, 10.0), (500.0, 100.0), (501.0, 20.0)],
            vec![(499.0, 15.0), (500.0, 200.0), (501.0, 25.0)],
            vec![(499.0, 8.0), (500.0, 50.0), (501.0, 12.0)],
        ];

        let xic = Quantifier::extract_ion_chromatogram(&spectra, 500.0, 0.1);
        assert_eq!(xic.len(), 3);
        assert!((xic[0] - 100.0).abs() < 0.01);
        assert!((xic[1] - 200.0).abs() < 0.01);
        assert!((xic[2] - 50.0).abs() < 0.01);
    }

    #[test]
    fn test_extract_ion_chromatogram_no_match() {
        let spectra = vec![
            vec![(100.0, 500.0), (200.0, 300.0)],
        ];
        let xic = Quantifier::extract_ion_chromatogram(&spectra, 500.0, 0.1);
        assert_eq!(xic.len(), 1);
        assert_eq!(xic[0], 0.0);
    }

    // ── Helper function tests ───────────────────────────────────────────

    #[test]
    fn test_binomial_prob_sum_to_one() {
        let n = 20;
        let p = 0.0107;
        let total: f64 = (0..=n).map(|k| binomial_prob(n, k, p)).sum();
        assert!((total - 1.0).abs() < 1e-6, "binomial probs should sum to 1: {total}");
    }

    #[test]
    fn test_binomial_prob_edge_cases() {
        assert!((binomial_prob(10, 0, 0.0) - 1.0).abs() < 1e-10);
        assert!((binomial_prob(10, 10, 1.0) - 1.0).abs() < 1e-10);
        assert_eq!(binomial_prob(5, 6, 0.5), 0.0); // k > n
    }

    // ── Config and enum tests ───────────────────────────────────────────

    #[test]
    fn test_config_creation() {
        let config = MassSpecConfig {
            mass_range: (100.0, 2000.0),
            resolution: 50000.0,
            ionization_mode: IonizationMode::Maldi,
            mass_accuracy_ppm: 2.0,
        };
        assert_eq!(config.mass_range, (100.0, 2000.0));
        assert_eq!(config.resolution, 50000.0);
        assert_eq!(config.ionization_mode, IonizationMode::Maldi);
    }

    #[test]
    fn test_ionization_mode_variants() {
        let modes = [
            IonizationMode::EsiPositive,
            IonizationMode::EsiNegative,
            IonizationMode::Maldi,
            IonizationMode::Ei,
        ];
        // All variants should be distinct
        for i in 0..modes.len() {
            for j in (i + 1)..modes.len() {
                assert_ne!(modes[i], modes[j]);
            }
        }
    }

    #[test]
    fn test_peak_struct() {
        let peak = Peak {
            mz: 523.456,
            intensity: 1234.5,
            width_da: 0.05,
            snr: 50.0,
            charge_state: Some(2),
        };
        assert!((peak.mz - 523.456).abs() < 1e-10);
        assert_eq!(peak.charge_state, Some(2));
    }

    #[test]
    fn test_processor_config_reference() {
        let config = default_config();
        let proc = MassSpecProcessor::new(config.clone());
        assert_eq!(proc.config().resolution, 10000.0);
    }

    #[test]
    fn test_atomic_mass_constants() {
        // Sanity check atomic masses
        assert!(MASS_C12 == 12.0);
        assert!(MASS_C13 > MASS_C12);
        assert!(MASS_H > 1.0 && MASS_H < 1.01);
        assert!(MASS_N > 14.0 && MASS_N < 14.01);
        assert!(MASS_O > 15.99 && MASS_O < 16.0);
        assert!((C13_C12_DELTA - 1.003355).abs() < 0.0001);
    }
}
