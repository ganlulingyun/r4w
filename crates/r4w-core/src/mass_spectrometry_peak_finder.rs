//! # Mass Spectrometry Peak Finder
//!
//! Mass spectrum analysis for peak detection, isotope pattern matching,
//! molecular formula determination, and fragmentation analysis from
//! mass-to-charge (m/z) spectra.
//!
//! ## Components
//!
//! - **MassSpectrum** - m/z intensity data container with normalization and filtering
//! - **PeakDetector** - Local maxima detection with prominence, FWHM, S/N estimation
//! - **IsotopePattern** - Theoretical isotope distribution calculation and matching
//! - **MassAccuracy** - ppm/Da error, mass defect, Kendrick mass, RDBE
//! - **FormulaGenerator** - Molecular formula determination from measured m/z
//! - **FragmentAnalyzer** - Neutral loss detection and mass difference networks
//! - **Calibrant** - Linear and quadratic mass calibration
//! - **ChargeStateDeconvolution** - ESI charge state envelope detection
//! - **SpectrumComparison** - Dot product and weighted spectral matching
//! - **ChromatogramIntegrator** - XIC/TIC extraction and peak integration

/// Proton mass in Daltons (used for ESI charge state calculations).
const PROTON_MASS: f64 = 1.007_276_47;

// ─── Atomic masses (monoisotopic, most abundant isotope) ────────────────────

const MASS_12C: f64 = 12.0;
const MASS_13C: f64 = 13.003_354_835;
const MASS_1H: f64 = 1.007_825_032;
const MASS_2H: f64 = 2.014_101_778;
const MASS_14N: f64 = 14.003_074_004;
const MASS_15N: f64 = 15.000_108_898;
const MASS_16O: f64 = 15.994_914_620;
const MASS_17O: f64 = 16.999_131_757;
const MASS_18O: f64 = 17.999_161_001;
const MASS_32S: f64 = 31.972_071_174;
const MASS_33S: f64 = 32.971_458_910;
const MASS_34S: f64 = 33.967_867_012;
const MASS_31P: f64 = 30.973_761_998;
const MASS_35CL: f64 = 34.968_852_68;
const MASS_37CL: f64 = 36.965_902_60;
const MASS_79BR: f64 = 78.918_337_1;
const MASS_81BR: f64 = 80.916_291_0;

// ─── Isotope abundances (fractional) ────────────────────────────────────────

const ABUND_12C: f64 = 0.9893;
const ABUND_13C: f64 = 0.0107;
const ABUND_1H: f64 = 0.999_885;
const ABUND_2H: f64 = 0.000_115;
const ABUND_14N: f64 = 0.996_36;
const ABUND_15N: f64 = 0.003_64;
const ABUND_16O: f64 = 0.997_57;
const ABUND_17O: f64 = 0.000_38;
const ABUND_18O: f64 = 0.002_05;
const ABUND_32S: f64 = 0.9493;
const ABUND_33S: f64 = 0.0076;
const ABUND_34S: f64 = 0.0429;
const ABUND_35CL: f64 = 0.7576;
const ABUND_37CL: f64 = 0.2424;
const ABUND_79BR: f64 = 0.5069;
const ABUND_81BR: f64 = 0.4931;

// ─── Average atomic masses ──────────────────────────────────────────────────

const AVG_MASS_C: f64 = MASS_12C * ABUND_12C + MASS_13C * ABUND_13C;
const AVG_MASS_H: f64 = MASS_1H * ABUND_1H + MASS_2H * ABUND_2H;
const AVG_MASS_N: f64 = MASS_14N * ABUND_14N + MASS_15N * ABUND_15N;
const AVG_MASS_O: f64 = MASS_16O * ABUND_16O + MASS_17O * ABUND_17O + MASS_18O * ABUND_18O;
const AVG_MASS_S: f64 = MASS_32S * ABUND_32S + MASS_33S * ABUND_33S + MASS_34S * ABUND_34S;
const AVG_MASS_P: f64 = MASS_31P; // monoisotopic (100%)
const AVG_MASS_CL: f64 = MASS_35CL * ABUND_35CL + MASS_37CL * ABUND_37CL;
const AVG_MASS_BR: f64 = MASS_79BR * ABUND_79BR + MASS_81BR * ABUND_81BR;

// ─── Common neutral losses ──────────────────────────────────────────────────

const LOSS_H2O: f64 = 18.010_565;
const LOSS_CO: f64 = 27.994_915;
const LOSS_CO2: f64 = 43.989_829;
const LOSS_NH3: f64 = 17.026_549;
const LOSS_HCL: f64 = 35.976_678;
const LOSS_CH3OH: f64 = 32.026_215;
const LOSS_HCOOH: f64 = 46.005_479;
const LOSS_CH2O: f64 = 30.010_565;
const LOSS_SO3: f64 = 79.956_815;
const LOSS_HPO3: f64 = 79.966_331;

/// Known neutral losses with identifiers.
const KNOWN_LOSSES: &[(&str, f64)] = &[
    ("H2O", LOSS_H2O),
    ("NH3", LOSS_NH3),
    ("CO", LOSS_CO),
    ("CH2O", LOSS_CH2O),
    ("CH3OH", LOSS_CH3OH),
    ("HCl", LOSS_HCL),
    ("CO2", LOSS_CO2),
    ("HCOOH", LOSS_HCOOH),
    ("SO3", LOSS_SO3),
    ("HPO3", LOSS_HPO3),
];

// ═══════════════════════════════════════════════════════════════════════════
// MassSpectrum
// ═══════════════════════════════════════════════════════════════════════════

/// A mass spectrum: paired vectors of m/z values and intensities.
#[derive(Debug, Clone)]
pub struct MassSpectrum {
    /// Mass-to-charge ratios.
    pub mz_values: Vec<f64>,
    /// Intensities corresponding to each m/z.
    pub intensities: Vec<f64>,
}

impl MassSpectrum {
    /// Create a new mass spectrum from m/z and intensity vectors.
    ///
    /// # Panics
    /// Panics if the two vectors have different lengths.
    pub fn new(mz_values: Vec<f64>, intensities: Vec<f64>) -> Self {
        assert_eq!(
            mz_values.len(),
            intensities.len(),
            "m/z and intensity vectors must have equal length"
        );
        Self {
            mz_values,
            intensities,
        }
    }

    /// Number of data points.
    pub fn len(&self) -> usize {
        self.mz_values.len()
    }

    /// True if empty.
    pub fn is_empty(&self) -> bool {
        self.mz_values.is_empty()
    }

    /// Return the base peak (highest intensity) as (m/z, intensity).
    /// Returns (0.0, 0.0) if spectrum is empty.
    pub fn base_peak(&self) -> (f64, f64) {
        if self.intensities.is_empty() {
            return (0.0, 0.0);
        }
        let mut best_idx = 0;
        let mut best_int = self.intensities[0];
        for (i, &v) in self.intensities.iter().enumerate().skip(1) {
            if v > best_int {
                best_int = v;
                best_idx = i;
            }
        }
        (self.mz_values[best_idx], best_int)
    }

    /// Normalize all intensities relative to the base peak (0..100%).
    pub fn normalize_to_base_peak(&self) -> MassSpectrum {
        let (_, bp) = self.base_peak();
        if bp == 0.0 {
            return self.clone();
        }
        let normed: Vec<f64> = self.intensities.iter().map(|&v| v / bp * 100.0).collect();
        MassSpectrum::new(self.mz_values.clone(), normed)
    }

    /// Keep only peaks with relative intensity >= `min_percent` of the base peak.
    pub fn filter_by_intensity(&self, min_percent: f64) -> MassSpectrum {
        let (_, bp) = self.base_peak();
        if bp == 0.0 {
            return self.clone();
        }
        let threshold = bp * min_percent / 100.0;
        let mut mz = Vec::new();
        let mut ints = Vec::new();
        for (i, &v) in self.intensities.iter().enumerate() {
            if v >= threshold {
                mz.push(self.mz_values[i]);
                ints.push(v);
            }
        }
        MassSpectrum::new(mz, ints)
    }

    /// Return the m/z range (min, max).
    pub fn mz_range(&self) -> (f64, f64) {
        if self.mz_values.is_empty() {
            return (0.0, 0.0);
        }
        let mut lo = self.mz_values[0];
        let mut hi = self.mz_values[0];
        for &v in &self.mz_values[1..] {
            if v < lo {
                lo = v;
            }
            if v > hi {
                hi = v;
            }
        }
        (lo, hi)
    }

    /// Total Ion Current: sum of all intensities.
    pub fn total_ion_current(&self) -> f64 {
        self.intensities.iter().sum()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// PeakDetector
// ═══════════════════════════════════════════════════════════════════════════

/// A detected mass spectrum peak.
#[derive(Debug, Clone)]
pub struct MsPeak {
    /// Centroid m/z of the peak.
    pub mz: f64,
    /// Peak intensity.
    pub intensity: f64,
    /// Full width at half maximum in Da.
    pub fwhm: f64,
    /// Approximate area under the peak (trapezoidal).
    pub area: f64,
    /// Signal-to-noise ratio.
    pub sn_ratio: f64,
}

/// Peak detector for mass spectra.
#[derive(Debug, Clone)]
pub struct PeakDetector {
    /// Minimum absolute intensity to consider a peak.
    pub min_intensity: f64,
    /// Minimum prominence (intensity above surrounding valleys).
    pub min_prominence: f64,
}

impl PeakDetector {
    /// Create a new peak detector.
    pub fn new(min_intensity: f64, min_prominence: f64) -> Self {
        Self {
            min_intensity,
            min_prominence,
        }
    }

    /// Detect peaks in a mass spectrum.
    pub fn detect(&self, spectrum: &MassSpectrum) -> Vec<MsPeak> {
        let n = spectrum.len();
        if n < 3 {
            return Vec::new();
        }
        let mz = &spectrum.mz_values;
        let ints = &spectrum.intensities;

        // Estimate noise from the lowest 25% of intensities.
        let noise = self.estimate_noise(ints);

        let mut peaks = Vec::new();
        for i in 1..n - 1 {
            // local maximum check
            if ints[i] > ints[i - 1] && ints[i] > ints[i + 1] && ints[i] >= self.min_intensity {
                // prominence: height above highest of left/right minima
                let left_min = find_left_min(ints, i);
                let right_min = find_right_min(ints, i);
                let valley = if left_min > right_min {
                    left_min
                } else {
                    right_min
                };
                let prominence = ints[i] - valley;
                if prominence < self.min_prominence {
                    continue;
                }

                // centroid in neighbourhood
                let centroid_mz = self.centroid_local(mz, ints, i);

                // FWHM
                let half_max = ints[i] / 2.0;
                let fwhm = estimate_fwhm(mz, ints, i, half_max);

                // area (trapezoidal around peak)
                let area = estimate_peak_area(mz, ints, i);

                let sn = if noise > 0.0 {
                    ints[i] / noise
                } else {
                    f64::INFINITY
                };

                peaks.push(MsPeak {
                    mz: centroid_mz,
                    intensity: ints[i],
                    fwhm,
                    area,
                    sn_ratio: sn,
                });
            }
        }
        peaks
    }

    /// Compute the intensity-weighted centroid m/z in a given range.
    pub fn centroid(&self, spectrum: &MassSpectrum, mz_start: f64, mz_end: f64) -> f64 {
        let mut sum_wi = 0.0;
        let mut sum_w = 0.0;
        for (i, &m) in spectrum.mz_values.iter().enumerate() {
            if m >= mz_start && m <= mz_end {
                let w = spectrum.intensities[i];
                sum_wi += w * m;
                sum_w += w;
            }
        }
        if sum_w > 0.0 {
            sum_wi / sum_w
        } else {
            (mz_start + mz_end) / 2.0
        }
    }

    /// Estimate noise level in a given m/z region.
    pub fn noise_level(&self, spectrum: &MassSpectrum, mz_region: (f64, f64)) -> f64 {
        let mut vals = Vec::new();
        for (i, &m) in spectrum.mz_values.iter().enumerate() {
            if m >= mz_region.0 && m <= mz_region.1 {
                vals.push(spectrum.intensities[i]);
            }
        }
        if vals.is_empty() {
            return 0.0;
        }
        vals.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        // median of the lower half
        let quarter = vals.len() / 4;
        if quarter == 0 {
            vals[0]
        } else {
            vals[..quarter].iter().sum::<f64>() / quarter as f64
        }
    }

    // Internal: centroid around index i.
    fn centroid_local(&self, mz: &[f64], ints: &[f64], idx: usize) -> f64 {
        let lo = if idx >= 2 { idx - 2 } else { 0 };
        let hi = if idx + 2 < mz.len() { idx + 2 } else { mz.len() - 1 };
        let mut sw = 0.0;
        let mut swm = 0.0;
        for j in lo..=hi {
            sw += ints[j];
            swm += ints[j] * mz[j];
        }
        if sw > 0.0 {
            swm / sw
        } else {
            mz[idx]
        }
    }

    // Internal: noise estimate from intensity vector.
    fn estimate_noise(&self, ints: &[f64]) -> f64 {
        if ints.is_empty() {
            return 0.0;
        }
        let mut sorted: Vec<f64> = ints.to_vec();
        sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let quarter = sorted.len() / 4;
        if quarter == 0 {
            sorted[0]
        } else {
            sorted[..quarter].iter().sum::<f64>() / quarter as f64
        }
    }
}

/// Find the minimum intensity going left from index `idx`.
fn find_left_min(ints: &[f64], idx: usize) -> f64 {
    let mut min_val = ints[idx];
    let mut j = idx;
    while j > 0 {
        j -= 1;
        if ints[j] < min_val {
            min_val = ints[j];
        }
        // stop at next local max
        if j > 0 && ints[j] > ints[j - 1] && (j + 1 >= ints.len() || ints[j] > ints[j + 1]) {
            break;
        }
    }
    min_val
}

/// Find the minimum intensity going right from index `idx`.
fn find_right_min(ints: &[f64], idx: usize) -> f64 {
    let mut min_val = ints[idx];
    let mut j = idx;
    while j + 1 < ints.len() {
        j += 1;
        if ints[j] < min_val {
            min_val = ints[j];
        }
        if j + 1 < ints.len()
            && ints[j] > ints[j + 1]
            && (j == 0 || ints[j] > ints[j - 1])
        {
            break;
        }
    }
    min_val
}

/// Estimate FWHM by linear interpolation on either side of peak at `idx`.
fn estimate_fwhm(mz: &[f64], ints: &[f64], idx: usize, half_max: f64) -> f64 {
    // left side
    let mut left_mz = mz[idx];
    for j in (0..idx).rev() {
        if ints[j] <= half_max {
            // linear interpolation
            let frac = (half_max - ints[j]) / (ints[j + 1] - ints[j]).max(1e-30);
            left_mz = mz[j] + frac * (mz[j + 1] - mz[j]);
            break;
        }
    }
    // right side
    let mut right_mz = mz[idx];
    for j in (idx + 1)..ints.len() {
        if ints[j] <= half_max {
            let frac = (half_max - ints[j]) / (ints[j - 1] - ints[j]).max(1e-30);
            right_mz = mz[j] - frac * (mz[j] - mz[j - 1]);
            break;
        }
    }
    (right_mz - left_mz).abs()
}

/// Estimate peak area using trapezoidal integration around the peak index.
fn estimate_peak_area(mz: &[f64], ints: &[f64], idx: usize) -> f64 {
    let lo = if idx >= 3 { idx - 3 } else { 0 };
    let hi = if idx + 3 < mz.len() {
        idx + 3
    } else {
        mz.len() - 1
    };
    let mut area = 0.0;
    for j in lo..hi {
        area += (ints[j] + ints[j + 1]) * (mz[j + 1] - mz[j]) / 2.0;
    }
    area
}

// ═══════════════════════════════════════════════════════════════════════════
// MolecularFormula & IsotopePattern
// ═══════════════════════════════════════════════════════════════════════════

/// Molecular formula with atom counts.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct MolecularFormula {
    pub c: u32,
    pub h: u32,
    pub n: u32,
    pub o: u32,
    pub s: u32,
    pub p: u32,
    pub cl: u32,
    pub br: u32,
}

impl MolecularFormula {
    /// Total number of atoms.
    pub fn total_atoms(&self) -> u32 {
        self.c + self.h + self.n + self.o + self.s + self.p + self.cl + self.br
    }

    /// Nominal (integer) mass using most abundant isotope masses rounded.
    pub fn nominal_mass(&self) -> u32 {
        self.c * 12
            + self.h * 1
            + self.n * 14
            + self.o * 16
            + self.s * 32
            + self.p * 31
            + self.cl * 35
            + self.br * 79
    }
}

impl std::fmt::Display for MolecularFormula {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let mut s = String::new();
        if self.c > 0 {
            s.push('C');
            if self.c > 1 {
                s.push_str(&self.c.to_string());
            }
        }
        if self.h > 0 {
            s.push('H');
            if self.h > 1 {
                s.push_str(&self.h.to_string());
            }
        }
        if self.n > 0 {
            s.push('N');
            if self.n > 1 {
                s.push_str(&self.n.to_string());
            }
        }
        if self.o > 0 {
            s.push('O');
            if self.o > 1 {
                s.push_str(&self.o.to_string());
            }
        }
        if self.s > 0 {
            s.push('S');
            if self.s > 1 {
                s.push_str(&self.s.to_string());
            }
        }
        if self.p > 0 {
            s.push('P');
            if self.p > 1 {
                s.push_str(&self.p.to_string());
            }
        }
        if self.cl > 0 {
            s.push_str("Cl");
            if self.cl > 1 {
                s.push_str(&self.cl.to_string());
            }
        }
        if self.br > 0 {
            s.push_str("Br");
            if self.br > 1 {
                s.push_str(&self.br.to_string());
            }
        }
        write!(f, "{}", s)
    }
}

/// Isotope pattern calculator.
pub struct IsotopePattern;

impl IsotopePattern {
    /// Calculate the theoretical isotope distribution for a molecular formula
    /// at a given charge state. Returns (m/z, relative_intensity) pairs
    /// normalized so the highest peak is 1.0.
    pub fn calculate(formula: &MolecularFormula, charge: i32) -> Vec<(f64, f64)> {
        // We use the polynomial multiplication (convolution) approach.
        // Start with a unit pattern [1.0] and convolve element by element.
        let mut pattern: Vec<f64> = vec![1.0];

        // Carbon: 2 isotopes
        for _ in 0..formula.c {
            pattern = convolve_isotopes(&pattern, &[ABUND_12C, ABUND_13C]);
        }
        // Hydrogen: 2 isotopes
        for _ in 0..formula.h {
            pattern = convolve_isotopes(&pattern, &[ABUND_1H, ABUND_2H]);
        }
        // Nitrogen: 2 isotopes
        for _ in 0..formula.n {
            pattern = convolve_isotopes(&pattern, &[ABUND_14N, ABUND_15N]);
        }
        // Oxygen: 3 isotopes
        for _ in 0..formula.o {
            pattern = convolve_isotopes(&pattern, &[ABUND_16O, ABUND_17O, ABUND_18O]);
        }
        // Sulfur: 3 isotopes (32S, 33S, 34S)
        for _ in 0..formula.s {
            pattern = convolve_isotopes(&pattern, &[ABUND_32S, ABUND_33S, ABUND_34S]);
        }
        // Phosphorus: monoisotopic
        // (no convolution needed, just adds fixed mass)
        // Chlorine: 2 isotopes (35Cl, 37Cl) - note 2 Da spacing
        for _ in 0..formula.cl {
            pattern = convolve_isotopes_spacing(&pattern, &[ABUND_35CL, ABUND_37CL], 2);
        }
        // Bromine: 2 isotopes (79Br, 81Br) - 2 Da spacing
        for _ in 0..formula.br {
            pattern = convolve_isotopes_spacing(&pattern, &[ABUND_79BR, ABUND_81BR], 2);
        }

        // Normalize to max = 1.0
        let max_int = pattern
            .iter()
            .cloned()
            .fold(0.0_f64, f64::max);
        if max_int > 0.0 {
            for v in &mut pattern {
                *v /= max_int;
            }
        }

        // Calculate m/z values: monoisotopic mass + i * 1.003355 (average neutron mass difference)
        // divided by charge.
        let mono = Self::monoisotopic_mass(formula);
        let abs_charge = if charge == 0 { 1 } else { charge.unsigned_abs() };
        let neutron_delta = MASS_13C - MASS_12C; // ~1.003355 Da

        pattern
            .iter()
            .enumerate()
            .filter(|(_, &v)| v > 1e-6)
            .map(|(i, &v)| {
                let neutral = mono + i as f64 * neutron_delta;
                let mz = if charge > 0 {
                    (neutral + abs_charge as f64 * PROTON_MASS) / abs_charge as f64
                } else if charge < 0 {
                    (neutral - abs_charge as f64 * PROTON_MASS) / abs_charge as f64
                } else {
                    neutral
                };
                (mz, v)
            })
            .collect()
    }

    /// Monoisotopic mass of the formula (lightest isotope of each element).
    pub fn monoisotopic_mass(formula: &MolecularFormula) -> f64 {
        formula.c as f64 * MASS_12C
            + formula.h as f64 * MASS_1H
            + formula.n as f64 * MASS_14N
            + formula.o as f64 * MASS_16O
            + formula.s as f64 * MASS_32S
            + formula.p as f64 * MASS_31P
            + formula.cl as f64 * MASS_35CL
            + formula.br as f64 * MASS_79BR
    }

    /// Average mass (weighted by natural isotope abundances).
    pub fn average_mass(formula: &MolecularFormula) -> f64 {
        formula.c as f64 * AVG_MASS_C
            + formula.h as f64 * AVG_MASS_H
            + formula.n as f64 * AVG_MASS_N
            + formula.o as f64 * AVG_MASS_O
            + formula.s as f64 * AVG_MASS_S
            + formula.p as f64 * AVG_MASS_P
            + formula.cl as f64 * AVG_MASS_CL
            + formula.br as f64 * AVG_MASS_BR
    }

    /// Match an observed pattern against a theoretical one using cosine similarity.
    /// Both patterns are Vec<(m/z, intensity)>. Returns score in [0, 1].
    pub fn match_pattern(
        observed: &[(f64, f64)],
        theoretical: &[(f64, f64)],
        tolerance_da: f64,
    ) -> f64 {
        if observed.is_empty() || theoretical.is_empty() {
            return 0.0;
        }
        // Build matched pairs
        let mut dot = 0.0;
        let mut mag_a = 0.0;
        let mut mag_b = 0.0;

        for &(mz_t, int_t) in theoretical {
            mag_b += int_t * int_t;
            // find closest observed peak
            let mut best_int = 0.0;
            for &(mz_o, int_o) in observed {
                if (mz_o - mz_t).abs() <= tolerance_da && int_o > best_int {
                    best_int = int_o;
                }
            }
            dot += int_t * best_int;
        }

        for &(_, int_o) in observed {
            mag_a += int_o * int_o;
        }

        let denom = mag_a.sqrt() * mag_b.sqrt();
        if denom > 0.0 {
            dot / denom
        } else {
            0.0
        }
    }
}

/// Convolve isotope pattern with a single-element isotope distribution (1 Da spacing).
fn convolve_isotopes(pattern: &[f64], element: &[f64]) -> Vec<f64> {
    let new_len = pattern.len() + element.len() - 1;
    let mut result = vec![0.0; new_len];
    for (i, &p) in pattern.iter().enumerate() {
        for (j, &e) in element.iter().enumerate() {
            result[i + j] += p * e;
        }
    }
    result
}

/// Convolve with spacing > 1 Da between isotopes (e.g. Cl, Br at 2 Da).
fn convolve_isotopes_spacing(pattern: &[f64], element: &[f64], spacing: usize) -> Vec<f64> {
    // Expand element with zeros: e.g. [a, b] with spacing=2 => [a, 0, b]
    let expanded_len = 1 + (element.len() - 1) * spacing;
    let mut expanded = vec![0.0; expanded_len];
    for (i, &e) in element.iter().enumerate() {
        expanded[i * spacing] = e;
    }
    convolve_isotopes(pattern, &expanded)
}

// ═══════════════════════════════════════════════════════════════════════════
// MassAccuracy
// ═══════════════════════════════════════════════════════════════════════════

/// Mass accuracy calculations.
pub struct MassAccuracy;

impl MassAccuracy {
    /// Parts-per-million error: (measured - theoretical) / theoretical * 1e6.
    pub fn ppm_error(measured: f64, theoretical: f64) -> f64 {
        if theoretical == 0.0 {
            return 0.0;
        }
        (measured - theoretical) / theoretical * 1e6
    }

    /// Absolute mass error in Daltons.
    pub fn da_error(measured: f64, theoretical: f64) -> f64 {
        (measured - theoretical).abs()
    }

    /// Mass defect: the fractional part of the nominal mass.
    /// For example, mass 180.0634 has defect 0.0634.
    pub fn mass_defect(mass: f64) -> f64 {
        mass - mass.round()
    }

    /// Kendrick mass: KM = mass * (round(base_unit_mass) / base_unit_mass).
    /// Default base unit for hydrocarbon analysis: CH2 = 14.01565.
    pub fn kendrick_mass(mass: f64, base_unit_mass: f64) -> f64 {
        if base_unit_mass == 0.0 {
            return mass;
        }
        mass * (base_unit_mass.round() / base_unit_mass)
    }

    /// Kendrick mass defect.
    pub fn kendrick_mass_defect(mass: f64, base_unit_mass: f64) -> f64 {
        let km = Self::kendrick_mass(mass, base_unit_mass);
        km.round() - km
    }

    /// Ring and Double Bond Equivalence (degrees of unsaturation).
    /// RDBE = C - H/2 + N/2 + 1
    pub fn ring_double_bond_equivalence(c: u32, h: u32, n: u32) -> f64 {
        c as f64 - h as f64 / 2.0 + n as f64 / 2.0 + 1.0
    }

    /// Nitrogen rule: an even-electron ion with an even nominal mass
    /// has an even number of nitrogen atoms, and vice versa.
    /// Returns true if the formula satisfies the nitrogen rule.
    pub fn nitrogen_rule_satisfied(formula: &MolecularFormula) -> bool {
        let nominal = formula.nominal_mass();
        let n_even = formula.n % 2 == 0;
        let mass_even = nominal % 2 == 0;
        // For M+H ions: even mass ⟹ even N, odd mass ⟹ odd N
        n_even == mass_even
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// FormulaGenerator
// ═══════════════════════════════════════════════════════════════════════════

/// Constraints for molecular formula generation.
#[derive(Debug, Clone)]
pub struct FormulaConstraints {
    pub c_max: u32,
    pub h_max: u32,
    pub n_max: u32,
    pub o_max: u32,
    pub s_max: u32,
    pub p_max: u32,
    pub cl_max: u32,
    pub br_max: u32,
}

impl Default for FormulaConstraints {
    fn default() -> Self {
        Self {
            c_max: 40,
            h_max: 80,
            n_max: 10,
            o_max: 20,
            s_max: 4,
            p_max: 4,
            cl_max: 4,
            br_max: 4,
        }
    }
}

/// A candidate molecular formula with quality metrics.
#[derive(Debug, Clone)]
pub struct FormulaCandidate {
    pub formula: MolecularFormula,
    pub mass_error_ppm: f64,
    pub rdbe: f64,
}

/// Molecular formula generator from measured m/z.
pub struct FormulaGenerator;

impl FormulaGenerator {
    /// Generate candidate molecular formulas for a measured m/z value.
    /// Considers protonated species: M_neutral = z * m/z - z * proton_mass.
    pub fn generate(
        measured_mz: f64,
        tolerance_ppm: f64,
        charge: i32,
        constraints: &FormulaConstraints,
    ) -> Vec<FormulaCandidate> {
        let abs_z = if charge == 0 { 1 } else { charge.unsigned_abs() } as f64;
        // Neutral mass from m/z
        let neutral_mass = if charge > 0 {
            measured_mz * abs_z - abs_z * PROTON_MASS
        } else if charge < 0 {
            measured_mz * abs_z + abs_z * PROTON_MASS
        } else {
            measured_mz
        };

        let tol_da = neutral_mass * tolerance_ppm / 1e6;
        let mass_lo = neutral_mass - tol_da;
        let mass_hi = neutral_mass + tol_da;

        let mut candidates = Vec::new();

        // Brute-force enumeration with mass-based pruning.
        // Start with heaviest elements first for better pruning.
        for br in 0..=constraints.br_max {
            let m_br = br as f64 * MASS_79BR;
            if m_br > mass_hi {
                break;
            }
            for cl in 0..=constraints.cl_max {
                let m_cl = m_br + cl as f64 * MASS_35CL;
                if m_cl > mass_hi {
                    break;
                }
                for s in 0..=constraints.s_max {
                    let m_s = m_cl + s as f64 * MASS_32S;
                    if m_s > mass_hi {
                        break;
                    }
                    for p in 0..=constraints.p_max {
                        let m_p = m_s + p as f64 * MASS_31P;
                        if m_p > mass_hi {
                            break;
                        }
                        for o in 0..=constraints.o_max {
                            let m_o = m_p + o as f64 * MASS_16O;
                            if m_o > mass_hi {
                                break;
                            }
                            for n in 0..=constraints.n_max {
                                let m_n = m_o + n as f64 * MASS_14N;
                                if m_n > mass_hi {
                                    break;
                                }
                                for c in 0..=constraints.c_max {
                                    let m_c = m_n + c as f64 * MASS_12C;
                                    if m_c > mass_hi {
                                        break;
                                    }
                                    // Remaining mass must be filled by hydrogen
                                    let remaining = neutral_mass - m_c;
                                    if remaining < 0.0 {
                                        break;
                                    }
                                    let h_approx = remaining / MASS_1H;
                                    let h = h_approx.round() as u32;
                                    if h > constraints.h_max {
                                        continue;
                                    }
                                    let calc_mass = m_c + h as f64 * MASS_1H;
                                    if calc_mass < mass_lo || calc_mass > mass_hi {
                                        continue;
                                    }

                                    let formula = MolecularFormula {
                                        c,
                                        h,
                                        n,
                                        o,
                                        s,
                                        p,
                                        cl,
                                        br,
                                    };

                                    // Senior's theorem: RDBE must be >= 0
                                    let rdbe =
                                        MassAccuracy::ring_double_bond_equivalence(c, h, n);
                                    if rdbe < -0.5 {
                                        continue;
                                    }

                                    // Hydrogen rule: H <= 2C + N + 2 + P (simplified)
                                    let h_max_rule = 2 * c + n + 2 + p;
                                    if h > h_max_rule {
                                        continue;
                                    }

                                    let ppm_err = MassAccuracy::ppm_error(calc_mass, neutral_mass);

                                    candidates.push(FormulaCandidate {
                                        formula,
                                        mass_error_ppm: ppm_err,
                                        rdbe,
                                    });
                                }
                            }
                        }
                    }
                }
            }
        }

        // Sort by absolute ppm error
        candidates.sort_by(|a, b| {
            a.mass_error_ppm
                .abs()
                .partial_cmp(&b.mass_error_ppm.abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        });

        candidates
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// FragmentAnalyzer
// ═══════════════════════════════════════════════════════════════════════════

/// A detected neutral loss.
#[derive(Debug, Clone)]
pub struct NeutralLoss {
    /// m/z of the fragment peak.
    pub fragment_mz: f64,
    /// Mass of the loss.
    pub loss_mass: f64,
    /// Identified loss formula (empty if unidentified).
    pub loss_formula: String,
}

/// Fragment analyzer for MS/MS spectra.
pub struct FragmentAnalyzer;

impl FragmentAnalyzer {
    /// Identify neutral losses between a precursor m/z and observed fragments.
    pub fn neutral_losses(precursor_mz: f64, fragments: &[f64]) -> Vec<NeutralLoss> {
        let mut losses = Vec::new();
        for &frag in fragments {
            let diff = precursor_mz - frag;
            if diff <= 0.0 {
                continue;
            }
            let mut formula = String::new();
            for &(name, mass) in KNOWN_LOSSES {
                if (diff - mass).abs() < 0.01 {
                    formula = name.to_string();
                    break;
                }
            }
            losses.push(NeutralLoss {
                fragment_mz: frag,
                loss_mass: diff,
                loss_formula: formula,
            });
        }
        losses
    }

    /// Build a mass difference network: for every pair of peaks within tolerance,
    /// return (index_i, index_j, mass_difference).
    pub fn mass_difference_network(
        peaks: &[f64],
        tolerance_da: f64,
    ) -> Vec<(usize, usize, f64)> {
        let mut edges = Vec::new();
        for i in 0..peaks.len() {
            for j in (i + 1)..peaks.len() {
                let diff = (peaks[j] - peaks[i]).abs();
                // Check if this difference is close to a known neutral loss
                for &(_, mass) in KNOWN_LOSSES {
                    if (diff - mass).abs() <= tolerance_da {
                        edges.push((i, j, diff));
                        break;
                    }
                }
            }
        }
        edges
    }

    /// Identify potential loss from a mass difference.
    pub fn identify_loss(mass_diff: f64, tolerance_da: f64) -> Option<&'static str> {
        for &(name, mass) in KNOWN_LOSSES {
            if (mass_diff - mass).abs() <= tolerance_da {
                return Some(name);
            }
        }
        None
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Calibrant
// ═══════════════════════════════════════════════════════════════════════════

/// Result of mass calibration.
#[derive(Debug, Clone)]
pub struct CalibrationResult {
    /// Calibration type.
    pub cal_type: CalibrationType,
    /// Polynomial coefficients [a0, a1, ...] such that
    /// m/z_true = a0 + a1 * m/z_meas + a2 * m/z_meas^2 + ...
    pub coefficients: Vec<f64>,
    /// RMS residual error after calibration.
    pub rms_residual: f64,
    /// Individual residuals.
    pub residuals: Vec<f64>,
}

/// Type of calibration.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum CalibrationType {
    Linear,
    Quadratic,
}

/// Mass calibration using reference m/z values.
pub struct Calibrant;

impl Calibrant {
    /// Linear calibration: m/z_true = a + b * m/z_measured.
    /// Uses least-squares regression.
    pub fn calibrate_linear(measured: &[f64], reference: &[f64]) -> CalibrationResult {
        assert_eq!(measured.len(), reference.len());
        assert!(measured.len() >= 2, "Need at least 2 calibration points");

        let n = measured.len() as f64;
        let sum_x: f64 = measured.iter().sum();
        let sum_y: f64 = reference.iter().sum();
        let sum_xx: f64 = measured.iter().map(|&x| x * x).sum();
        let sum_xy: f64 = measured
            .iter()
            .zip(reference.iter())
            .map(|(&x, &y)| x * y)
            .sum();

        let denom = n * sum_xx - sum_x * sum_x;
        let b = if denom.abs() > 1e-30 {
            (n * sum_xy - sum_x * sum_y) / denom
        } else {
            1.0
        };
        let a = (sum_y - b * sum_x) / n;

        let residuals: Vec<f64> = measured
            .iter()
            .zip(reference.iter())
            .map(|(&m, &r)| (a + b * m) - r)
            .collect();
        let rms = (residuals.iter().map(|r| r * r).sum::<f64>() / n).sqrt();

        CalibrationResult {
            cal_type: CalibrationType::Linear,
            coefficients: vec![a, b],
            rms_residual: rms,
            residuals,
        }
    }

    /// Quadratic calibration: m/z_true = a + b*x + c*x^2.
    /// Uses normal equations for polynomial least squares.
    pub fn calibrate_quadratic(measured: &[f64], reference: &[f64]) -> CalibrationResult {
        assert_eq!(measured.len(), reference.len());
        assert!(
            measured.len() >= 3,
            "Need at least 3 calibration points for quadratic"
        );

        let n = measured.len();
        // Build normal equations: A^T A x = A^T b
        // A = [[1, x_i, x_i^2], ...]
        let mut ata = [[0.0_f64; 3]; 3];
        let mut atb = [0.0_f64; 3];

        for i in 0..n {
            let x = measured[i];
            let y = reference[i];
            let row = [1.0, x, x * x];
            for r in 0..3 {
                for c in 0..3 {
                    ata[r][c] += row[r] * row[c];
                }
                atb[r] += row[r] * y;
            }
        }

        // Solve 3x3 system via Cramer's rule
        let coeffs = solve_3x3(&ata, &atb);

        let residuals: Vec<f64> = measured
            .iter()
            .zip(reference.iter())
            .map(|(&m, &r)| {
                let fitted = coeffs[0] + coeffs[1] * m + coeffs[2] * m * m;
                fitted - r
            })
            .collect();
        let rms = (residuals.iter().map(|r| r * r).sum::<f64>() / n as f64).sqrt();

        CalibrationResult {
            cal_type: CalibrationType::Quadratic,
            coefficients: coeffs.to_vec(),
            rms_residual: rms,
            residuals,
        }
    }

    /// Apply a calibration to a spectrum.
    pub fn apply_calibration(
        spectrum: &MassSpectrum,
        cal: &CalibrationResult,
    ) -> MassSpectrum {
        let new_mz: Vec<f64> = spectrum
            .mz_values
            .iter()
            .map(|&m| apply_poly(&cal.coefficients, m))
            .collect();
        MassSpectrum::new(new_mz, spectrum.intensities.clone())
    }

    /// Get residual errors in ppm after calibration.
    pub fn residual_errors_ppm(
        measured: &[f64],
        reference: &[f64],
        cal: &CalibrationResult,
    ) -> Vec<f64> {
        measured
            .iter()
            .zip(reference.iter())
            .map(|(&m, &r)| {
                let corrected = apply_poly(&cal.coefficients, m);
                MassAccuracy::ppm_error(corrected, r)
            })
            .collect()
    }
}

/// Apply polynomial: c[0] + c[1]*x + c[2]*x^2 + ...
fn apply_poly(coeffs: &[f64], x: f64) -> f64 {
    let mut val = 0.0;
    let mut xn = 1.0;
    for &c in coeffs {
        val += c * xn;
        xn *= x;
    }
    val
}

/// Solve a 3x3 linear system using Cramer's rule.
fn solve_3x3(a: &[[f64; 3]; 3], b: &[f64; 3]) -> [f64; 3] {
    let det = |m: &[[f64; 3]; 3]| -> f64 {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };

    let d = det(a);
    if d.abs() < 1e-30 {
        return [0.0, 1.0, 0.0]; // fallback: identity-ish
    }

    let mut a0 = *a;
    for i in 0..3 {
        a0[i][0] = b[i];
    }
    let d0 = det(&a0);

    let mut a1 = *a;
    for i in 0..3 {
        a1[i][1] = b[i];
    }
    let d1 = det(&a1);

    let mut a2 = *a;
    for i in 0..3 {
        a2[i][2] = b[i];
    }
    let d2 = det(&a2);

    [d0 / d, d1 / d, d2 / d]
}

// ═══════════════════════════════════════════════════════════════════════════
// ChargeStateDeconvolution
// ═══════════════════════════════════════════════════════════════════════════

/// A deconvolved charge envelope.
#[derive(Debug, Clone)]
pub struct ChargeEnvelope {
    /// Neutral (deconvolved) monoisotopic mass.
    pub neutral_mass: f64,
    /// Charge states observed: (charge, m/z).
    pub charge_states: Vec<(i32, f64)>,
}

/// ESI charge state deconvolution.
pub struct ChargeStateDeconvolution;

impl ChargeStateDeconvolution {
    /// Deconvolve neutral mass from m/z and charge: M = z * (m/z - proton_mass).
    pub fn deconvolve_mass(mz: f64, z: i32) -> f64 {
        let abs_z = z.unsigned_abs() as f64;
        mz * abs_z - abs_z * PROTON_MASS
    }

    /// Estimate charge state from spacing between two adjacent charge state peaks.
    /// z = 1 / (mz_lower - mz_higher) approximately, for consecutive z, z+1.
    pub fn charge_from_spacing(mz1: f64, mz2: f64) -> i32 {
        let diff = (mz1 - mz2).abs();
        if diff < 0.01 {
            return 0;
        }
        // For consecutive charge states z and z+1:
        // mz1 = (M + z*p)/z, mz2 = (M + (z+1)*p)/(z+1)
        // delta = mz1 - mz2 ≈ M / (z*(z+1)) for large z, or more precisely:
        // z ≈ mz2 / (mz1 - mz2) when mz1 > mz2
        let (hi, lo) = if mz1 > mz2 {
            (mz1, mz2)
        } else {
            (mz2, mz1)
        };
        let z_est = lo / (hi - lo);
        z_est.round() as i32
    }

    /// Find charge state envelopes from a list of peak m/z values.
    /// Groups peaks that correspond to the same neutral mass at different charges.
    pub fn find_charge_states(
        peaks: &[f64],
        tolerance_da: f64,
    ) -> Vec<ChargeEnvelope> {
        let mut envelopes: Vec<ChargeEnvelope> = Vec::new();
        let mut used = vec![false; peaks.len()];

        // Try all pairs of peaks as potential adjacent charge states
        for i in 0..peaks.len() {
            if used[i] {
                continue;
            }
            for j in (i + 1)..peaks.len() {
                if used[j] {
                    continue;
                }
                let z = Self::charge_from_spacing(peaks[i], peaks[j]);
                if z < 1 || z > 100 {
                    continue;
                }

                // Check if these could be z and z+1
                let (hi_mz, lo_mz) = if peaks[i] > peaks[j] {
                    (peaks[i], peaks[j])
                } else {
                    (peaks[j], peaks[i])
                };
                let z_hi = z; // lower charge = higher m/z
                let z_lo = z + 1;

                let mass1 = Self::deconvolve_mass(hi_mz, z_hi);
                let mass2 = Self::deconvolve_mass(lo_mz, z_lo);

                if (mass1 - mass2).abs() <= tolerance_da {
                    let avg_mass = (mass1 + mass2) / 2.0;

                    // Try to find more charge states
                    let mut charge_states = vec![(z_hi, hi_mz), (z_lo, lo_mz)];
                    used[i] = true;
                    used[j] = true;

                    for zz in 1..=100_i32 {
                        if zz == z_hi || zz == z_lo {
                            continue;
                        }
                        let expected_mz =
                            (avg_mass + zz as f64 * PROTON_MASS) / zz as f64;
                        // Find a matching peak
                        for (k, &pk) in peaks.iter().enumerate() {
                            if !used[k] && (pk - expected_mz).abs() <= tolerance_da {
                                charge_states.push((zz, pk));
                                used[k] = true;
                                break;
                            }
                        }
                    }

                    if charge_states.len() >= 2 {
                        charge_states.sort_by_key(|&(z, _)| z);
                        envelopes.push(ChargeEnvelope {
                            neutral_mass: avg_mass,
                            charge_states,
                        });
                    }
                }
            }
        }

        envelopes
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// SpectrumComparison
// ═══════════════════════════════════════════════════════════════════════════

/// Quality tiers for spectral matching scores.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MatchQuality {
    Excellent,
    Good,
    Fair,
    Poor,
}

/// Spectrum comparison utilities.
pub struct SpectrumComparison;

impl SpectrumComparison {
    /// Cosine similarity (dot product) between two spectra.
    /// Peaks are matched within `tolerance_da`.
    /// Returns a score in [0, 1].
    pub fn dot_product(
        spec_a: &MassSpectrum,
        spec_b: &MassSpectrum,
        tolerance_da: f64,
    ) -> f64 {
        if spec_a.is_empty() || spec_b.is_empty() {
            return 0.0;
        }

        let mut dot = 0.0;
        let mut mag_a_sq = 0.0;
        let mut mag_b_sq = 0.0;

        // For each peak in A, find the best match in B
        let mut matched_b = vec![false; spec_b.len()];

        for i in 0..spec_a.len() {
            let ia = spec_a.intensities[i];
            mag_a_sq += ia * ia;

            let mut best_j = None;
            let mut best_diff = f64::MAX;
            for j in 0..spec_b.len() {
                let diff = (spec_a.mz_values[i] - spec_b.mz_values[j]).abs();
                if diff <= tolerance_da && diff < best_diff && !matched_b[j] {
                    best_diff = diff;
                    best_j = Some(j);
                }
            }
            if let Some(j) = best_j {
                dot += ia * spec_b.intensities[j];
                matched_b[j] = true;
            }
        }

        for j in 0..spec_b.len() {
            mag_b_sq += spec_b.intensities[j] * spec_b.intensities[j];
        }

        let denom = mag_a_sq.sqrt() * mag_b_sq.sqrt();
        if denom > 0.0 {
            dot / denom
        } else {
            0.0
        }
    }

    /// Reverse dot product: only considers peaks present in the library spectrum (spec_b).
    /// Useful when the query has extra peaks not in the library.
    pub fn reverse_dot_product(
        spec_a: &MassSpectrum,
        spec_b: &MassSpectrum,
        tolerance_da: f64,
    ) -> f64 {
        if spec_a.is_empty() || spec_b.is_empty() {
            return 0.0;
        }

        let mut dot = 0.0;
        let mut mag_matched_a_sq = 0.0;
        let mut mag_b_sq = 0.0;

        for j in 0..spec_b.len() {
            let ib = spec_b.intensities[j];
            mag_b_sq += ib * ib;

            let mut best_int = 0.0;
            for i in 0..spec_a.len() {
                let diff = (spec_a.mz_values[i] - spec_b.mz_values[j]).abs();
                if diff <= tolerance_da && spec_a.intensities[i] > best_int {
                    best_int = spec_a.intensities[i];
                }
            }
            dot += ib * best_int;
            mag_matched_a_sq += best_int * best_int;
        }

        let denom = mag_matched_a_sq.sqrt() * mag_b_sq.sqrt();
        if denom > 0.0 {
            dot / denom
        } else {
            0.0
        }
    }

    /// Classify a dot-product score into quality tiers (score on 0-1000 scale).
    pub fn match_factor(score: f64) -> MatchQuality {
        let s1000 = score * 1000.0;
        if s1000 > 900.0 {
            MatchQuality::Excellent
        } else if s1000 > 700.0 {
            MatchQuality::Good
        } else if s1000 > 500.0 {
            MatchQuality::Fair
        } else {
            MatchQuality::Poor
        }
    }

    /// Weighted dot product: intensities are weighted by m/z^mz_weight * intensity^int_weight.
    pub fn weighted_dot_product(
        spec_a: &MassSpectrum,
        spec_b: &MassSpectrum,
        mz_weight: f64,
        int_weight: f64,
        tolerance_da: f64,
    ) -> f64 {
        if spec_a.is_empty() || spec_b.is_empty() {
            return 0.0;
        }

        // Weight function: w = mz^mz_weight * intensity^int_weight
        let weight = |mz: f64, int: f64| -> f64 { mz.powf(mz_weight) * int.powf(int_weight) };

        let wa: Vec<f64> = spec_a
            .mz_values
            .iter()
            .zip(spec_a.intensities.iter())
            .map(|(&m, &i)| weight(m, i))
            .collect();

        let wb: Vec<f64> = spec_b
            .mz_values
            .iter()
            .zip(spec_b.intensities.iter())
            .map(|(&m, &i)| weight(m, i))
            .collect();

        let mut dot = 0.0;
        let mut mag_a_sq = 0.0;
        let mut mag_b_sq = 0.0;

        let mut matched_b = vec![false; spec_b.len()];

        for i in 0..spec_a.len() {
            mag_a_sq += wa[i] * wa[i];

            let mut best_j = None;
            let mut best_diff = f64::MAX;
            for j in 0..spec_b.len() {
                let diff = (spec_a.mz_values[i] - spec_b.mz_values[j]).abs();
                if diff <= tolerance_da && diff < best_diff && !matched_b[j] {
                    best_diff = diff;
                    best_j = Some(j);
                }
            }
            if let Some(j) = best_j {
                dot += wa[i] * wb[j];
                matched_b[j] = true;
            }
        }

        for j in 0..spec_b.len() {
            mag_b_sq += wb[j] * wb[j];
        }

        let denom = mag_a_sq.sqrt() * mag_b_sq.sqrt();
        if denom > 0.0 {
            dot / denom
        } else {
            0.0
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// ChromatogramIntegrator
// ═══════════════════════════════════════════════════════════════════════════

/// Chromatogram integrator for LC-MS / GC-MS data.
pub struct ChromatogramIntegrator;

impl ChromatogramIntegrator {
    /// Extract Ion Chromatogram (XIC): intensity of a target m/z over retention time.
    /// `spectra` is a list of (retention_time, spectrum) pairs.
    pub fn extract_ion_chromatogram(
        spectra: &[(f64, MassSpectrum)],
        target_mz: f64,
        tolerance_da: f64,
    ) -> Vec<(f64, f64)> {
        spectra
            .iter()
            .map(|(rt, spec)| {
                let mut best_int = 0.0;
                for (i, &m) in spec.mz_values.iter().enumerate() {
                    if (m - target_mz).abs() <= tolerance_da {
                        if spec.intensities[i] > best_int {
                            best_int = spec.intensities[i];
                        }
                    }
                }
                (*rt, best_int)
            })
            .collect()
    }

    /// Total Ion Chromatogram (TIC): sum of all intensities at each time point.
    pub fn total_ion_chromatogram(spectra: &[(f64, MassSpectrum)]) -> Vec<(f64, f64)> {
        spectra
            .iter()
            .map(|(rt, spec)| (*rt, spec.total_ion_current()))
            .collect()
    }

    /// Integrate a chromatographic peak using trapezoidal rule.
    /// `chromatogram` is a list of (time, intensity) pairs.
    pub fn integrate_peak(
        chromatogram: &[(f64, f64)],
        t_start: f64,
        t_end: f64,
    ) -> f64 {
        let mut area = 0.0;
        for w in chromatogram.windows(2) {
            let (t0, y0) = w[0];
            let (t1, y1) = w[1];
            // Check overlap with integration window
            let a = t0.max(t_start);
            let b = t1.min(t_end);
            if b > a {
                // Interpolate y at boundaries if needed
                let frac_a = if t1 > t0 {
                    (a - t0) / (t1 - t0)
                } else {
                    0.0
                };
                let frac_b = if t1 > t0 {
                    (b - t0) / (t1 - t0)
                } else {
                    1.0
                };
                let ya = y0 + frac_a * (y1 - y0);
                let yb = y0 + frac_b * (y1 - y0);
                area += (ya + yb) * (b - a) / 2.0;
            }
        }
        area
    }

    /// Base Peak Chromatogram: highest intensity peak at each time point.
    pub fn base_peak_chromatogram(spectra: &[(f64, MassSpectrum)]) -> Vec<(f64, f64)> {
        spectra
            .iter()
            .map(|(rt, spec)| {
                let (_, bp_int) = spec.base_peak();
                (*rt, bp_int)
            })
            .collect()
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// Tests
// ═══════════════════════════════════════════════════════════════════════════

#[cfg(test)]
mod tests {
    use super::*;

    // ─── MassSpectrum tests ─────────────────────────────────────────────

    #[test]
    fn test_mass_spectrum_new() {
        let spec = MassSpectrum::new(vec![100.0, 200.0, 300.0], vec![50.0, 100.0, 25.0]);
        assert_eq!(spec.len(), 3);
        assert!(!spec.is_empty());
    }

    #[test]
    fn test_mass_spectrum_empty() {
        let spec = MassSpectrum::new(vec![], vec![]);
        assert!(spec.is_empty());
        assert_eq!(spec.base_peak(), (0.0, 0.0));
        assert_eq!(spec.mz_range(), (0.0, 0.0));
        assert_eq!(spec.total_ion_current(), 0.0);
    }

    #[test]
    fn test_base_peak() {
        let spec = MassSpectrum::new(vec![100.0, 200.0, 300.0], vec![50.0, 100.0, 25.0]);
        assert_eq!(spec.base_peak(), (200.0, 100.0));
    }

    #[test]
    fn test_normalize_to_base_peak() {
        let spec = MassSpectrum::new(vec![100.0, 200.0, 300.0], vec![50.0, 100.0, 25.0]);
        let normed = spec.normalize_to_base_peak();
        assert!((normed.intensities[0] - 50.0).abs() < 1e-10);
        assert!((normed.intensities[1] - 100.0).abs() < 1e-10);
        assert!((normed.intensities[2] - 25.0).abs() < 1e-10);
    }

    #[test]
    fn test_filter_by_intensity() {
        let spec = MassSpectrum::new(
            vec![100.0, 200.0, 300.0, 400.0],
            vec![10.0, 100.0, 5.0, 50.0],
        );
        let filtered = spec.filter_by_intensity(10.0);
        // 10% of 100 = 10; keep peaks >= 10
        assert_eq!(filtered.len(), 3); // 10, 100, 50
    }

    #[test]
    fn test_mz_range() {
        let spec = MassSpectrum::new(vec![150.0, 100.0, 300.0, 200.0], vec![1.0; 4]);
        assert_eq!(spec.mz_range(), (100.0, 300.0));
    }

    #[test]
    fn test_total_ion_current() {
        let spec = MassSpectrum::new(vec![100.0, 200.0], vec![50.0, 75.0]);
        assert!((spec.total_ion_current() - 125.0).abs() < 1e-10);
    }

    // ─── PeakDetector tests ─────────────────────────────────────────────

    #[test]
    fn test_peak_detector_simple() {
        // Gaussian-like peak centered at m/z 500
        let mz: Vec<f64> = (0..100).map(|i| 450.0 + i as f64).collect();
        let ints: Vec<f64> = mz
            .iter()
            .map(|&m| {
                let x = (m - 500.0) / 2.0;
                1000.0 * (-x * x / 2.0).exp()
            })
            .collect();
        let spec = MassSpectrum::new(mz, ints);
        let det = PeakDetector::new(10.0, 5.0);
        let peaks = det.detect(&spec);
        assert!(!peaks.is_empty());
        // The detected peak should be near 500.0
        assert!((peaks[0].mz - 500.0).abs() < 1.0);
        assert!(peaks[0].intensity > 900.0);
    }

    #[test]
    fn test_peak_detector_multiple() {
        // Two well-separated peaks
        let mz: Vec<f64> = (0..200).map(|i| 100.0 + i as f64).collect();
        let ints: Vec<f64> = mz
            .iter()
            .map(|&m| {
                let g1 = 1000.0 * (-((m - 150.0) / 2.0).powi(2) / 2.0).exp();
                let g2 = 500.0 * (-((m - 250.0) / 3.0).powi(2) / 2.0).exp();
                g1 + g2
            })
            .collect();
        let spec = MassSpectrum::new(mz, ints);
        let det = PeakDetector::new(10.0, 5.0);
        let peaks = det.detect(&spec);
        assert!(peaks.len() >= 2);
    }

    #[test]
    fn test_peak_detector_no_peaks() {
        let spec = MassSpectrum::new(vec![100.0, 200.0], vec![1.0, 1.0]);
        let det = PeakDetector::new(10.0, 5.0);
        let peaks = det.detect(&spec);
        assert!(peaks.is_empty());
    }

    #[test]
    fn test_centroid_calculation() {
        let spec = MassSpectrum::new(
            vec![99.0, 100.0, 101.0],
            vec![25.0, 100.0, 75.0],
        );
        let det = PeakDetector::new(1.0, 0.1);
        let c = det.centroid(&spec, 98.0, 102.0);
        // Weighted average: (25*99 + 100*100 + 75*101) / (25+100+75) = 100.25
        assert!((c - 100.25).abs() < 0.01);
    }

    #[test]
    fn test_noise_level() {
        let spec = MassSpectrum::new(
            vec![100.0, 200.0, 300.0, 400.0, 500.0, 600.0, 700.0, 800.0],
            vec![1.0, 2.0, 3.0, 1000.0, 2.5, 1.5, 3.5, 900.0],
        );
        let det = PeakDetector::new(1.0, 0.1);
        let noise = det.noise_level(&spec, (100.0, 800.0));
        // Lowest 25% of 8 values = 2 values: 1.0, 1.5
        assert!(noise < 10.0);
    }

    #[test]
    fn test_peak_fwhm() {
        // Perfect Gaussian peak, FWHM ≈ 2.355 * sigma
        let sigma = 1.0;
        let mz: Vec<f64> = (0..100).map(|i| 495.0 + i as f64 * 0.1).collect();
        let ints: Vec<f64> = mz
            .iter()
            .map(|&m| {
                let x = (m - 500.0) / sigma;
                1000.0 * (-x * x / 2.0).exp()
            })
            .collect();
        let spec = MassSpectrum::new(mz, ints);
        let det = PeakDetector::new(1.0, 1.0);
        let peaks = det.detect(&spec);
        assert!(!peaks.is_empty());
        let expected_fwhm = 2.355 * sigma;
        assert!(
            (peaks[0].fwhm - expected_fwhm).abs() < 0.5,
            "FWHM {} expected ~{}",
            peaks[0].fwhm,
            expected_fwhm
        );
    }

    #[test]
    fn test_peak_sn_ratio() {
        let mz: Vec<f64> = (0..100).map(|i| 450.0 + i as f64).collect();
        let ints: Vec<f64> = mz
            .iter()
            .map(|&m| {
                let signal = 1000.0 * (-((m - 500.0) / 2.0).powi(2) / 2.0).exp();
                signal + 1.0 // add baseline noise
            })
            .collect();
        let spec = MassSpectrum::new(mz, ints);
        let det = PeakDetector::new(5.0, 5.0);
        let peaks = det.detect(&spec);
        assert!(!peaks.is_empty());
        assert!(peaks[0].sn_ratio > 10.0);
    }

    // ─── IsotopePattern tests ───────────────────────────────────────────

    #[test]
    fn test_monoisotopic_mass_water() {
        let h2o = MolecularFormula {
            h: 2,
            o: 1,
            ..Default::default()
        };
        let mono = IsotopePattern::monoisotopic_mass(&h2o);
        assert!((mono - 18.010565).abs() < 0.001);
    }

    #[test]
    fn test_monoisotopic_mass_glucose() {
        // C6H12O6
        let glucose = MolecularFormula {
            c: 6,
            h: 12,
            o: 6,
            ..Default::default()
        };
        let mono = IsotopePattern::monoisotopic_mass(&glucose);
        assert!((mono - 180.063388).abs() < 0.001);
    }

    #[test]
    fn test_average_mass() {
        let h2o = MolecularFormula {
            h: 2,
            o: 1,
            ..Default::default()
        };
        let avg = IsotopePattern::average_mass(&h2o);
        assert!((avg - 18.015).abs() < 0.01);
    }

    #[test]
    fn test_isotope_pattern_carbon() {
        // Pure carbon C1: should give M (98.93%) and M+1 (1.07%)
        let c1 = MolecularFormula {
            c: 1,
            ..Default::default()
        };
        let pattern = IsotopePattern::calculate(&c1, 0);
        assert!(pattern.len() >= 2);
        assert!((pattern[0].1 - 1.0).abs() < 0.01); // M normalized to 1.0
        assert!((pattern[1].1 - ABUND_13C / ABUND_12C).abs() < 0.01);
    }

    #[test]
    fn test_isotope_pattern_c10() {
        // C10: M+1 peak should be approximately 10 * 1.07% = 10.7% of M
        let c10 = MolecularFormula {
            c: 10,
            ..Default::default()
        };
        let pattern = IsotopePattern::calculate(&c10, 0);
        assert!(pattern.len() >= 2);
        // M+1 relative intensity ≈ 0.107
        assert!(
            (pattern[1].1 - 0.107).abs() < 0.02,
            "M+1 = {}",
            pattern[1].1
        );
    }

    #[test]
    fn test_isotope_pattern_chlorine() {
        // Cl: 75.76% 35Cl, 24.24% 37Cl → M and M+2 pattern
        let formula = MolecularFormula {
            cl: 1,
            ..Default::default()
        };
        let pattern = IsotopePattern::calculate(&formula, 0);
        // Should have peaks at M and M+2
        assert!(!pattern.is_empty());
        // Find the M+2 peak (about 2 Da higher)
        let m0 = pattern[0].0;
        let m2_peaks: Vec<_> = pattern
            .iter()
            .filter(|(mz, _)| (*mz - m0 - 2.0 * (MASS_13C - MASS_12C)).abs() < 0.5)
            .collect();
        assert!(!m2_peaks.is_empty());
    }

    #[test]
    fn test_isotope_pattern_charged() {
        // CH4 at charge +1: m/z = (16.031 + 1.00728) / 1 = 17.038
        let ch4 = MolecularFormula {
            c: 1,
            h: 4,
            ..Default::default()
        };
        let pattern = IsotopePattern::calculate(&ch4, 1);
        assert!(!pattern.is_empty());
        let expected_mz = IsotopePattern::monoisotopic_mass(&ch4) + PROTON_MASS;
        assert!(
            (pattern[0].0 - expected_mz).abs() < 0.01,
            "got {} expected {}",
            pattern[0].0,
            expected_mz
        );
    }

    #[test]
    fn test_isotope_match_perfect() {
        let formula = MolecularFormula {
            c: 6,
            h: 12,
            o: 6,
            ..Default::default()
        };
        let theo = IsotopePattern::calculate(&formula, 0);
        // Perfect match against itself
        let score = IsotopePattern::match_pattern(&theo, &theo, 0.1);
        assert!(score > 0.99, "Self-match score = {}", score);
    }

    #[test]
    fn test_isotope_match_poor() {
        let obs = vec![(100.0, 1.0), (101.0, 0.5)];
        let theo = vec![(200.0, 1.0), (201.0, 0.1)];
        let score = IsotopePattern::match_pattern(&obs, &theo, 0.1);
        assert!(score < 0.1, "Mismatch score = {}", score);
    }

    // ─── MassAccuracy tests ─────────────────────────────────────────────

    #[test]
    fn test_ppm_error() {
        let ppm = MassAccuracy::ppm_error(500.001, 500.0);
        assert!((ppm - 2.0).abs() < 0.1);
    }

    #[test]
    fn test_ppm_error_negative() {
        let ppm = MassAccuracy::ppm_error(499.999, 500.0);
        assert!(ppm < 0.0);
    }

    #[test]
    fn test_da_error() {
        assert!((MassAccuracy::da_error(500.001, 500.0) - 0.001).abs() < 1e-10);
    }

    #[test]
    fn test_mass_defect() {
        let defect = MassAccuracy::mass_defect(180.063);
        assert!((defect - 0.063).abs() < 0.001);
    }

    #[test]
    fn test_kendrick_mass() {
        // CH2 exact mass = 14.01565
        let km = MassAccuracy::kendrick_mass(200.0, 14.015_65);
        let expected = 200.0 * (14.0 / 14.015_65);
        assert!((km - expected).abs() < 0.01);
    }

    #[test]
    fn test_kendrick_mass_defect() {
        let kmd = MassAccuracy::kendrick_mass_defect(200.156, 14.015_65);
        // Should be a small number
        assert!(kmd.abs() < 1.0);
    }

    #[test]
    fn test_rdbe_benzene() {
        // Benzene C6H6: RDBE = 6 - 6/2 + 0/2 + 1 = 4
        let rdbe = MassAccuracy::ring_double_bond_equivalence(6, 6, 0);
        assert!((rdbe - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_rdbe_methane() {
        // CH4: RDBE = 1 - 4/2 + 0/2 + 1 = 0
        let rdbe = MassAccuracy::ring_double_bond_equivalence(1, 4, 0);
        assert!((rdbe - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_rdbe_pyridine() {
        // C5H5N: RDBE = 5 - 5/2 + 1/2 + 1 = 4
        let rdbe = MassAccuracy::ring_double_bond_equivalence(5, 5, 1);
        assert!((rdbe - 4.0).abs() < 1e-10);
    }

    #[test]
    fn test_nitrogen_rule_even() {
        // C6H12O6 (glucose): even nominal mass (180), 0 N (even) → satisfied
        let glucose = MolecularFormula {
            c: 6,
            h: 12,
            o: 6,
            ..Default::default()
        };
        assert!(MassAccuracy::nitrogen_rule_satisfied(&glucose));
    }

    #[test]
    fn test_nitrogen_rule_odd() {
        // C5H5N (pyridine): nominal mass = 79 (odd), 1 N (odd) → satisfied
        let pyridine = MolecularFormula {
            c: 5,
            h: 5,
            n: 1,
            ..Default::default()
        };
        assert!(MassAccuracy::nitrogen_rule_satisfied(&pyridine));
    }

    // ─── FormulaGenerator tests ─────────────────────────────────────────

    #[test]
    fn test_formula_generator_water() {
        let constraints = FormulaConstraints {
            c_max: 5,
            h_max: 20,
            n_max: 2,
            o_max: 5,
            s_max: 0,
            p_max: 0,
            cl_max: 0,
            br_max: 0,
        };
        let candidates = FormulaGenerator::generate(18.010565, 10.0, 0, &constraints);
        // Should find H2O
        let found = candidates
            .iter()
            .any(|c| c.formula.h == 2 && c.formula.o == 1 && c.formula.c == 0);
        assert!(found, "Should find H2O among candidates");
    }

    #[test]
    fn test_formula_generator_methane() {
        let constraints = FormulaConstraints {
            c_max: 5,
            h_max: 20,
            n_max: 2,
            o_max: 5,
            s_max: 0,
            p_max: 0,
            cl_max: 0,
            br_max: 0,
        };
        // CH4 monoisotopic mass = 16.031300
        let mono = IsotopePattern::monoisotopic_mass(&MolecularFormula {
            c: 1,
            h: 4,
            ..Default::default()
        });
        let candidates = FormulaGenerator::generate(mono, 5.0, 0, &constraints);
        let found = candidates
            .iter()
            .any(|c| c.formula.c == 1 && c.formula.h == 4);
        assert!(found, "Should find CH4");
    }

    #[test]
    fn test_formula_generator_protonated() {
        let constraints = FormulaConstraints {
            c_max: 10,
            h_max: 30,
            n_max: 2,
            o_max: 10,
            s_max: 0,
            p_max: 0,
            cl_max: 0,
            br_max: 0,
        };
        // Glucose [M+H]+: m/z = 180.063388 + 1.007276 = 181.070664
        let mz = 181.070664;
        let candidates = FormulaGenerator::generate(mz, 5.0, 1, &constraints);
        let found = candidates
            .iter()
            .any(|c| c.formula.c == 6 && c.formula.h == 12 && c.formula.o == 6);
        assert!(found, "Should find C6H12O6 for protonated glucose");
    }

    #[test]
    fn test_formula_rdbe_filter() {
        let constraints = FormulaConstraints {
            c_max: 5,
            h_max: 20,
            n_max: 2,
            o_max: 5,
            ..Default::default()
        };
        let candidates = FormulaGenerator::generate(18.010565, 10.0, 0, &constraints);
        // All candidates should have RDBE >= -0.5
        for c in &candidates {
            assert!(c.rdbe >= -0.5, "RDBE {} < -0.5 for {}", c.rdbe, c.formula);
        }
    }

    // ─── FragmentAnalyzer tests ─────────────────────────────────────────

    #[test]
    fn test_neutral_loss_h2o() {
        let losses = FragmentAnalyzer::neutral_losses(200.0, &[181.989]);
        assert!(!losses.is_empty());
        assert_eq!(losses[0].loss_formula, "H2O");
    }

    #[test]
    fn test_neutral_loss_co2() {
        let losses = FragmentAnalyzer::neutral_losses(200.0, &[156.01]);
        let co2_loss = losses.iter().find(|l| l.loss_formula == "CO2");
        assert!(co2_loss.is_some());
    }

    #[test]
    fn test_neutral_loss_nh3() {
        let losses = FragmentAnalyzer::neutral_losses(200.0, &[182.973]);
        let nh3_loss = losses.iter().find(|l| l.loss_formula == "NH3");
        assert!(nh3_loss.is_some());
    }

    #[test]
    fn test_mass_difference_network() {
        let peaks = vec![100.0, 118.011, 143.990]; // diffs: ~18.011 (H2O), ~43.990 (CO2)
        let edges = FragmentAnalyzer::mass_difference_network(&peaks, 0.02);
        assert!(!edges.is_empty());
    }

    #[test]
    fn test_identify_loss() {
        assert_eq!(
            FragmentAnalyzer::identify_loss(18.011, 0.01),
            Some("H2O")
        );
        assert_eq!(
            FragmentAnalyzer::identify_loss(27.995, 0.01),
            Some("CO")
        );
        assert!(FragmentAnalyzer::identify_loss(99.999, 0.01).is_none());
    }

    // ─── Calibrant tests ────────────────────────────────────────────────

    #[test]
    fn test_linear_calibration() {
        // Perfect linear relationship: true = 1.001 * measured + 0.05
        let measured = vec![100.0, 200.0, 300.0, 400.0, 500.0];
        let reference: Vec<f64> = measured.iter().map(|&m| 1.001 * m + 0.05).collect();
        let cal = Calibrant::calibrate_linear(&measured, &reference);
        assert!((cal.coefficients[0] - 0.05).abs() < 0.01);
        assert!((cal.coefficients[1] - 1.001).abs() < 0.001);
        assert!(cal.rms_residual < 0.01);
    }

    #[test]
    fn test_quadratic_calibration() {
        let measured = vec![100.0, 200.0, 300.0, 400.0, 500.0];
        let reference: Vec<f64> = measured
            .iter()
            .map(|&m| 0.1 + 1.0 * m + 0.0001 * m * m)
            .collect();
        let cal = Calibrant::calibrate_quadratic(&measured, &reference);
        assert!(cal.rms_residual < 0.01);
    }

    #[test]
    fn test_apply_calibration() {
        let measured = vec![100.0, 200.0, 300.0, 400.0];
        let reference: Vec<f64> = measured.iter().map(|&m| 1.001 * m + 0.1).collect();
        let cal = Calibrant::calibrate_linear(&measured, &reference);

        let spec = MassSpectrum::new(vec![150.0, 250.0], vec![1.0, 1.0]);
        let corrected = Calibrant::apply_calibration(&spec, &cal);
        // 150 should map to ~1.001 * 150 + 0.1 = 150.25
        assert!((corrected.mz_values[0] - (1.001 * 150.0 + 0.1)).abs() < 0.01);
    }

    #[test]
    fn test_residual_errors_ppm() {
        let measured = vec![100.0, 200.0, 300.0, 400.0];
        let reference = vec![100.1, 200.2, 300.3, 400.4];
        let cal = Calibrant::calibrate_linear(&measured, &reference);
        let ppm_errors = Calibrant::residual_errors_ppm(&measured, &reference, &cal);
        // After calibration, residuals should be small
        for &e in &ppm_errors {
            assert!(e.abs() < 50.0, "Residual ppm too large: {}", e);
        }
    }

    // ─── ChargeStateDeconvolution tests ─────────────────────────────────

    #[test]
    fn test_deconvolve_mass() {
        // [M+2H]2+ at m/z 500: M = 2 * (500 - 1.00728) = 997.985
        let m = ChargeStateDeconvolution::deconvolve_mass(500.0, 2);
        assert!((m - 997.985).abs() < 0.01);
    }

    #[test]
    fn test_deconvolve_mass_singly_charged() {
        // [M+H]+: M = m/z - 1.00728
        let m = ChargeStateDeconvolution::deconvolve_mass(181.07, 1);
        assert!((m - 180.063).abs() < 0.01);
    }

    #[test]
    fn test_charge_from_spacing() {
        // For a protein at ~10000 Da:
        // z=10: m/z = (10000 + 10*1.00728)/10 = 1001.0073
        // z=11: m/z = (10000 + 11*1.00728)/11 = 910.0073
        // spacing ≈ 91.0 → z ≈ 910/91 ≈ 10
        let mz_10 = (10000.0 + 10.0 * PROTON_MASS) / 10.0;
        let mz_11 = (10000.0 + 11.0 * PROTON_MASS) / 11.0;
        let z = ChargeStateDeconvolution::charge_from_spacing(mz_10, mz_11);
        assert!(
            (z - 10).abs() <= 1,
            "Expected z~10, got {}",
            z
        );
    }

    #[test]
    fn test_find_charge_states() {
        // Generate charge state series for a 5000 Da protein
        let mass = 5000.0;
        let peaks: Vec<f64> = (3..=8)
            .map(|z| (mass + z as f64 * PROTON_MASS) / z as f64)
            .collect();
        let envelopes = ChargeStateDeconvolution::find_charge_states(&peaks, 1.0);
        assert!(
            !envelopes.is_empty(),
            "Should find at least one charge envelope"
        );
        // The neutral mass should be close to 5000
        assert!(
            (envelopes[0].neutral_mass - mass).abs() < 2.0,
            "Neutral mass {} expected ~{}",
            envelopes[0].neutral_mass,
            mass
        );
    }

    // ─── SpectrumComparison tests ───────────────────────────────────────

    #[test]
    fn test_dot_product_identical() {
        let spec = MassSpectrum::new(vec![100.0, 200.0, 300.0], vec![50.0, 100.0, 25.0]);
        let score = SpectrumComparison::dot_product(&spec, &spec, 0.5);
        assert!(
            (score - 1.0).abs() < 0.01,
            "Self-comparison = {}",
            score
        );
    }

    #[test]
    fn test_dot_product_orthogonal() {
        let a = MassSpectrum::new(vec![100.0, 200.0], vec![100.0, 50.0]);
        let b = MassSpectrum::new(vec![300.0, 400.0], vec![100.0, 50.0]);
        let score = SpectrumComparison::dot_product(&a, &b, 0.5);
        assert!(score < 0.01, "Orthogonal spectra should score ~0");
    }

    #[test]
    fn test_reverse_dot_product() {
        // A has extra peaks not in B
        let a = MassSpectrum::new(
            vec![100.0, 200.0, 300.0, 400.0],
            vec![50.0, 100.0, 25.0, 80.0],
        );
        let b = MassSpectrum::new(vec![100.0, 200.0], vec![50.0, 100.0]);
        let rev = SpectrumComparison::reverse_dot_product(&a, &b, 0.5);
        assert!(rev > 0.9, "Reverse dot product = {}", rev);
    }

    #[test]
    fn test_match_factor() {
        assert_eq!(
            SpectrumComparison::match_factor(0.95),
            MatchQuality::Excellent
        );
        assert_eq!(
            SpectrumComparison::match_factor(0.75),
            MatchQuality::Good
        );
        assert_eq!(
            SpectrumComparison::match_factor(0.55),
            MatchQuality::Fair
        );
        assert_eq!(
            SpectrumComparison::match_factor(0.3),
            MatchQuality::Poor
        );
    }

    #[test]
    fn test_weighted_dot_product() {
        let a = MassSpectrum::new(vec![100.0, 200.0, 300.0], vec![50.0, 100.0, 25.0]);
        let score =
            SpectrumComparison::weighted_dot_product(&a, &a, 0.5, 1.0, 0.5);
        assert!(score > 0.99, "Self-weighted = {}", score);
    }

    // ─── ChromatogramIntegrator tests ───────────────────────────────────

    #[test]
    fn test_extract_ion_chromatogram() {
        let spectra = vec![
            (1.0, MassSpectrum::new(vec![100.0, 200.0], vec![50.0, 10.0])),
            (2.0, MassSpectrum::new(vec![100.0, 200.0], vec![100.0, 20.0])),
            (3.0, MassSpectrum::new(vec![100.0, 200.0], vec![30.0, 5.0])),
        ];
        let xic = ChromatogramIntegrator::extract_ion_chromatogram(&spectra, 100.0, 0.5);
        assert_eq!(xic.len(), 3);
        assert!((xic[0].1 - 50.0).abs() < 1e-10);
        assert!((xic[1].1 - 100.0).abs() < 1e-10);
        assert!((xic[2].1 - 30.0).abs() < 1e-10);
    }

    #[test]
    fn test_total_ion_chromatogram() {
        let spectra = vec![
            (1.0, MassSpectrum::new(vec![100.0, 200.0], vec![50.0, 10.0])),
            (2.0, MassSpectrum::new(vec![100.0, 200.0], vec![100.0, 20.0])),
        ];
        let tic = ChromatogramIntegrator::total_ion_chromatogram(&spectra);
        assert_eq!(tic.len(), 2);
        assert!((tic[0].1 - 60.0).abs() < 1e-10);
        assert!((tic[1].1 - 120.0).abs() < 1e-10);
    }

    #[test]
    fn test_integrate_peak_trapezoidal() {
        // Rectangle: height 100, width 2
        let chrom = vec![(0.0, 0.0), (1.0, 100.0), (2.0, 100.0), (3.0, 0.0)];
        let area = ChromatogramIntegrator::integrate_peak(&chrom, 1.0, 2.0);
        assert!((area - 100.0).abs() < 1.0);
    }

    #[test]
    fn test_integrate_peak_triangle() {
        let chrom = vec![(0.0, 0.0), (1.0, 100.0), (2.0, 0.0)];
        let area = ChromatogramIntegrator::integrate_peak(&chrom, 0.0, 2.0);
        // Triangle area = 0.5 * 2 * 100 = 100
        assert!((area - 100.0).abs() < 1.0);
    }

    #[test]
    fn test_base_peak_chromatogram() {
        let spectra = vec![
            (1.0, MassSpectrum::new(vec![100.0, 200.0], vec![50.0, 80.0])),
            (2.0, MassSpectrum::new(vec![100.0, 200.0], vec![120.0, 30.0])),
        ];
        let bpc = ChromatogramIntegrator::base_peak_chromatogram(&spectra);
        assert!((bpc[0].1 - 80.0).abs() < 1e-10);
        assert!((bpc[1].1 - 120.0).abs() < 1e-10);
    }

    // ─── MolecularFormula tests ─────────────────────────────────────────

    #[test]
    fn test_formula_display() {
        let f = MolecularFormula {
            c: 6,
            h: 12,
            o: 6,
            ..Default::default()
        };
        assert_eq!(format!("{}", f), "C6H12O6");
    }

    #[test]
    fn test_formula_display_single_atom() {
        let f = MolecularFormula {
            c: 1,
            h: 4,
            ..Default::default()
        };
        assert_eq!(format!("{}", f), "CH4");
    }

    #[test]
    fn test_formula_nominal_mass() {
        let glucose = MolecularFormula {
            c: 6,
            h: 12,
            o: 6,
            ..Default::default()
        };
        assert_eq!(glucose.nominal_mass(), 180);
    }

    #[test]
    fn test_formula_total_atoms() {
        let glucose = MolecularFormula {
            c: 6,
            h: 12,
            o: 6,
            ..Default::default()
        };
        assert_eq!(glucose.total_atoms(), 24);
    }

    // ─── Edge cases & additional tests ──────────────────────────────────

    #[test]
    fn test_ppm_error_zero_theoretical() {
        assert_eq!(MassAccuracy::ppm_error(100.0, 0.0), 0.0);
    }

    #[test]
    fn test_kendrick_mass_zero_base() {
        assert_eq!(MassAccuracy::kendrick_mass(200.0, 0.0), 200.0);
    }

    #[test]
    fn test_convolve_isotopes_single() {
        let result = convolve_isotopes(&[1.0], &[0.99, 0.01]);
        assert_eq!(result.len(), 2);
        assert!((result[0] - 0.99).abs() < 1e-10);
        assert!((result[1] - 0.01).abs() < 1e-10);
    }

    #[test]
    fn test_convolve_isotopes_spacing() {
        let result = convolve_isotopes_spacing(&[1.0], &[0.75, 0.25], 2);
        // Should be [0.75, 0.0, 0.25]
        assert_eq!(result.len(), 3);
        assert!((result[0] - 0.75).abs() < 1e-10);
        assert!((result[1] - 0.0).abs() < 1e-10);
        assert!((result[2] - 0.25).abs() < 1e-10);
    }

    #[test]
    fn test_apply_poly() {
        // 1 + 2x + 3x^2 at x=2 = 1 + 4 + 12 = 17
        assert!((apply_poly(&[1.0, 2.0, 3.0], 2.0) - 17.0).abs() < 1e-10);
    }

    #[test]
    fn test_spectrum_comparison_empty() {
        let empty = MassSpectrum::new(vec![], vec![]);
        let non_empty = MassSpectrum::new(vec![100.0], vec![50.0]);
        assert_eq!(SpectrumComparison::dot_product(&empty, &non_empty, 0.5), 0.0);
    }

    #[test]
    fn test_fragment_no_positive_losses() {
        let losses = FragmentAnalyzer::neutral_losses(100.0, &[200.0, 300.0]);
        // All fragments are heavier than precursor → no neutral losses
        assert!(losses.is_empty());
    }

    #[test]
    fn test_isotope_pattern_bromine() {
        // Br: 50.69% 79Br, 49.31% 81Br → near 1:1 M, M+2
        let formula = MolecularFormula {
            br: 1,
            ..Default::default()
        };
        let pattern = IsotopePattern::calculate(&formula, 0);
        assert!(pattern.len() >= 2);
        // The two peaks should be nearly equal intensity
        let max_int = pattern.iter().map(|p| p.1).fold(0.0_f64, f64::max);
        let min_int = pattern
            .iter()
            .filter(|p| p.1 > 0.01)
            .map(|p| p.1)
            .fold(f64::MAX, f64::min);
        assert!(
            min_int / max_int > 0.9,
            "Br isotope ratio should be ~1:1"
        );
    }

    #[test]
    fn test_formula_with_sulfur() {
        let f = MolecularFormula {
            c: 2,
            h: 6,
            s: 1,
            ..Default::default()
        };
        // Dimethyl sulfide, monoisotopic mass
        let mono = IsotopePattern::monoisotopic_mass(&f);
        assert!((mono - 62.019).abs() < 0.01);
    }

    #[test]
    fn test_formula_with_phosphorus() {
        let f = MolecularFormula {
            h: 3,
            o: 4,
            p: 1,
            ..Default::default()
        };
        // Phosphoric acid H3PO4
        let mono = IsotopePattern::monoisotopic_mass(&f);
        assert!((mono - 97.977).abs() < 0.01);
    }

    #[test]
    fn test_calibration_identity() {
        // If measured == reference, calibration should be identity-like
        let vals = vec![100.0, 200.0, 300.0, 400.0];
        let cal = Calibrant::calibrate_linear(&vals, &vals);
        assert!((cal.coefficients[0]).abs() < 0.01);
        assert!((cal.coefficients[1] - 1.0).abs() < 0.001);
    }

    #[test]
    fn test_deconvolve_mass_high_charge() {
        // z=50, m/z = 200 → M = 50*(200 - 1.00728) = 9963.64
        let m = ChargeStateDeconvolution::deconvolve_mass(200.0, 50);
        let expected = 50.0 * (200.0 - PROTON_MASS);
        assert!((m - expected).abs() < 0.01);
    }

    #[test]
    fn test_xic_no_match() {
        let spectra = vec![
            (1.0, MassSpectrum::new(vec![100.0], vec![50.0])),
        ];
        let xic = ChromatogramIntegrator::extract_ion_chromatogram(&spectra, 999.0, 0.5);
        assert_eq!(xic.len(), 1);
        assert!((xic[0].1 - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_integrate_empty_range() {
        let chrom = vec![(0.0, 100.0), (1.0, 100.0)];
        // t_start > t_end → no area
        let area = ChromatogramIntegrator::integrate_peak(&chrom, 2.0, 3.0);
        assert!((area - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_formula_display_with_halogen() {
        let f = MolecularFormula {
            c: 1,
            h: 3,
            cl: 1,
            ..Default::default()
        };
        assert_eq!(format!("{}", f), "CH3Cl");
    }

    #[test]
    fn test_mass_spectrum_single_point() {
        let spec = MassSpectrum::new(vec![500.0], vec![1000.0]);
        assert_eq!(spec.base_peak(), (500.0, 1000.0));
        assert_eq!(spec.mz_range(), (500.0, 500.0));
        assert!((spec.total_ion_current() - 1000.0).abs() < 1e-10);
    }

    #[test]
    fn test_peak_area_positive() {
        let mz: Vec<f64> = (0..50).map(|i| 490.0 + i as f64 * 0.5).collect();
        let ints: Vec<f64> = mz
            .iter()
            .map(|&m| {
                let x = (m - 500.0) / 1.0;
                1000.0 * (-x * x / 2.0).exp()
            })
            .collect();
        let spec = MassSpectrum::new(mz, ints);
        let det = PeakDetector::new(1.0, 1.0);
        let peaks = det.detect(&spec);
        assert!(!peaks.is_empty());
        assert!(peaks[0].area > 0.0, "Peak area should be positive");
    }
}
