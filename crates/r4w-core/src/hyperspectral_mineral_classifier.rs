//! Hyperspectral mineral identification via spectral unmixing and absorption feature detection.
//!
//! This module provides tools for geological remote sensing in the Visible/Near-Infrared
//! (VNIR, 0.4-1.0 um) and Short-Wave Infrared (SWIR, 1.0-2.5 um) spectral regions.
//! Minerals exhibit diagnostic absorption features caused by electronic transitions
//! (Fe2+/Fe3+ in VNIR) and vibrational overtones (OH, CO3, H2O in SWIR) that enable
//! remote identification from hyperspectral imagery.
//!
//! # Key Components
//!
//! - [`SpectralBand`] - Wavelength/reflectance pair for building spectral signatures
//! - [`SpectralSignature`] - Complete spectral signature with named mineral identity
//! - [`MineralLibrary`] - Reference spectra database for common rock-forming minerals
//! - [`AbsorptionFeatureDetector`] - Identifies absorption bands by position, depth, and width
//! - [`ContinuumRemoval`] - Hull quotient continuum removal for absorption analysis
//! - [`SpectralUnmixer`] - Linear and constrained least-squares abundance estimation
//! - [`SpectralAngleMapper`] - SAM classification against reference endmembers
//!
//! # Absorption Feature Detection
//!
//! Minerals are identified by their characteristic absorption wavelengths:
//! - **Calcite/Dolomite**: ~2.335 um (CO3 vibrational overtone)
//! - **Kaolinite**: ~2.205 um (Al-OH bond)
//! - **Muscovite**: ~2.200 um (Al-OH bond)
//! - **Montmorillonite**: ~2.210 um (Al-OH), ~1.900 um (H2O)
//! - **Goethite/Hematite**: ~0.900 um (Fe3+ crystal field transition)
//! - **Chlorite**: ~2.350 um (Mg-OH bond)
//!
//! # Example
//!
//! ```
//! use r4w_core::hyperspectral_mineral_classifier::{
//!     SpectralSignature, MineralLibrary, SpectralAngleMapper,
//!     spectral_angle, normalize_spectrum,
//! };
//!
//! let library = MineralLibrary::default();
//! let quartz = library.get("Quartz").unwrap();
//!
//! // SAM classification of a measured spectrum
//! let sam = SpectralAngleMapper::new(library.all_signatures().to_vec());
//! let (name, angle) = sam.classify(&quartz.reflectances);
//! assert_eq!(name, "Quartz");
//! assert!(angle < 0.01);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Spectral Band and Signature Types
// ---------------------------------------------------------------------------

/// A single spectral measurement at a specific wavelength.
///
/// Represents one band in a hyperspectral dataset with wavelength in micrometers
/// and reflectance as a dimensionless ratio (0.0 to 1.0).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SpectralBand {
    /// Center wavelength of the band in micrometers (um).
    pub wavelength_um: f64,
    /// Reflectance value, typically in range [0.0, 1.0].
    pub reflectance: f64,
}

impl SpectralBand {
    /// Create a new spectral band measurement.
    pub fn new(wavelength_um: f64, reflectance: f64) -> Self {
        Self {
            wavelength_um,
            reflectance,
        }
    }
}

/// A complete spectral signature for a mineral or material.
///
/// Contains the mineral name, an ordered set of wavelengths (in um),
/// and corresponding reflectance values. Wavelengths must be monotonically
/// increasing.
#[derive(Debug, Clone)]
pub struct SpectralSignature {
    /// Name of the mineral or material.
    pub name: String,
    /// Center wavelengths in micrometers, sorted in ascending order.
    pub wavelengths: Vec<f64>,
    /// Reflectance values corresponding to each wavelength.
    pub reflectances: Vec<f64>,
}

impl SpectralSignature {
    /// Create a new spectral signature from wavelengths and reflectances.
    ///
    /// # Panics
    ///
    /// Panics if `wavelengths` and `reflectances` have different lengths or are empty.
    pub fn new(name: &str, wavelengths: Vec<f64>, reflectances: Vec<f64>) -> Self {
        assert_eq!(
            wavelengths.len(),
            reflectances.len(),
            "wavelengths and reflectances must have the same length"
        );
        assert!(!wavelengths.is_empty(), "signature must have at least one band");
        Self {
            name: name.to_string(),
            wavelengths,
            reflectances,
        }
    }

    /// Create a spectral signature from a slice of [`SpectralBand`] values.
    pub fn from_bands(name: &str, bands: &[SpectralBand]) -> Self {
        let wavelengths: Vec<f64> = bands.iter().map(|b| b.wavelength_um).collect();
        let reflectances: Vec<f64> = bands.iter().map(|b| b.reflectance).collect();
        Self::new(name, wavelengths, reflectances)
    }

    /// Number of spectral bands in this signature.
    pub fn num_bands(&self) -> usize {
        self.wavelengths.len()
    }

    /// Get the spectral band at a given index.
    pub fn band(&self, index: usize) -> SpectralBand {
        SpectralBand {
            wavelength_um: self.wavelengths[index],
            reflectance: self.reflectances[index],
        }
    }
}

// ---------------------------------------------------------------------------
// Absorption Feature
// ---------------------------------------------------------------------------

/// A detected absorption feature in a spectral signature.
///
/// Absorption features are local minima in reflectance caused by specific
/// molecular vibrational or electronic transitions. They are characterized
/// by their center wavelength, depth relative to the continuum, and full
/// width at half maximum (FWHM).
#[derive(Debug, Clone, PartialEq)]
pub struct AbsorptionFeature {
    /// Center wavelength of the absorption in micrometers.
    pub center_wavelength_um: f64,
    /// Depth of the absorption (0.0 = no absorption, 1.0 = total absorption).
    /// Computed as 1 - (reflectance_min / continuum_at_min).
    pub depth: f64,
    /// Full width at half maximum depth in micrometers.
    pub fwhm_um: f64,
    /// Index of the absorption minimum in the spectrum.
    pub min_index: usize,
}

// ---------------------------------------------------------------------------
// Absorption Feature Detector
// ---------------------------------------------------------------------------

/// Detects absorption features in spectral signatures.
///
/// Uses a local-minimum search with configurable depth threshold and minimum
/// width to identify diagnostic absorption bands. Operates on continuum-removed
/// spectra for accurate depth measurement.
///
/// # Algorithm
///
/// 1. Apply continuum removal to normalize the spectrum
/// 2. Search for local minima in the continuum-removed spectrum
/// 3. Filter by minimum depth threshold
/// 4. Compute FWHM for each qualifying absorption
#[derive(Debug, Clone)]
pub struct AbsorptionFeatureDetector {
    /// Minimum absorption depth to report (0.0 to 1.0).
    pub min_depth: f64,
    /// Minimum number of bands for a valid absorption feature.
    pub min_width_bands: usize,
}

impl AbsorptionFeatureDetector {
    /// Create a new detector with the given minimum depth threshold.
    ///
    /// # Arguments
    ///
    /// * `min_depth` - Minimum absorption depth to report (e.g., 0.02 for 2%)
    pub fn new(min_depth: f64) -> Self {
        Self {
            min_depth,
            min_width_bands: 3,
        }
    }

    /// Create a detector with custom minimum depth and width.
    pub fn with_min_width(min_depth: f64, min_width_bands: usize) -> Self {
        Self {
            min_depth,
            min_width_bands,
        }
    }

    /// Detect absorption features in the given spectral signature.
    ///
    /// Returns a vector of [`AbsorptionFeature`] sorted by center wavelength.
    pub fn detect(&self, signature: &SpectralSignature) -> Vec<AbsorptionFeature> {
        let n = signature.num_bands();
        if n < 3 {
            return Vec::new();
        }

        // Compute continuum removal
        let cr = ContinuumRemoval::compute(&signature.wavelengths, &signature.reflectances);

        let mut features = Vec::new();

        // Find local minima in the continuum-removed spectrum
        for i in 1..n - 1 {
            if cr[i] < cr[i - 1] && cr[i] < cr[i + 1] {
                let depth = 1.0 - cr[i];
                if depth >= self.min_depth {
                    // Compute FWHM: find half-depth level on each side
                    let half_level = 1.0 - depth / 2.0;

                    // Search left for half-depth crossing
                    let mut left_wl = signature.wavelengths[i];
                    for j in (0..i).rev() {
                        if cr[j] >= half_level {
                            // Linear interpolation between j and j+1
                            let frac = (half_level - cr[j + 1]) / (cr[j] - cr[j + 1]);
                            left_wl = signature.wavelengths[j + 1]
                                + frac * (signature.wavelengths[j] - signature.wavelengths[j + 1]);
                            break;
                        }
                    }

                    // Search right for half-depth crossing
                    let mut right_wl = signature.wavelengths[i];
                    for j in (i + 1)..n {
                        if cr[j] >= half_level {
                            // Linear interpolation between j-1 and j
                            let frac = (half_level - cr[j - 1]) / (cr[j] - cr[j - 1]);
                            right_wl = signature.wavelengths[j - 1]
                                + frac * (signature.wavelengths[j] - signature.wavelengths[j - 1]);
                            break;
                        }
                    }

                    let fwhm = right_wl - left_wl;
                    if fwhm > 0.0 {
                        features.push(AbsorptionFeature {
                            center_wavelength_um: signature.wavelengths[i],
                            depth,
                            fwhm_um: fwhm,
                            min_index: i,
                        });
                    }
                }
            }
        }

        features
    }

    /// Detect absorption features from raw wavelength/reflectance arrays.
    pub fn detect_raw(&self, wavelengths: &[f64], reflectances: &[f64]) -> Vec<AbsorptionFeature> {
        let sig = SpectralSignature::new("unknown", wavelengths.to_vec(), reflectances.to_vec());
        self.detect(&sig)
    }
}

// ---------------------------------------------------------------------------
// Continuum Removal
// ---------------------------------------------------------------------------

/// Continuum removal for absorption feature analysis.
///
/// The continuum is the convex hull of the reflectance spectrum viewed from above.
/// Continuum removal normalizes the spectrum by dividing each reflectance value
/// by the corresponding continuum value, producing values in [0, 1] where
/// absorption features appear as troughs below 1.0.
///
/// This is the standard technique described by Clark & Roush (1984) for
/// quantitative comparison of absorption features.
pub struct ContinuumRemoval;

impl ContinuumRemoval {
    /// Compute the convex hull continuum of a reflectance spectrum.
    ///
    /// Returns the continuum values at each wavelength position.
    pub fn continuum(wavelengths: &[f64], reflectances: &[f64]) -> Vec<f64> {
        let n = wavelengths.len();
        if n == 0 {
            return Vec::new();
        }
        if n == 1 {
            return vec![reflectances[0]];
        }

        // Build upper convex hull (viewing reflectance from above)
        // Using monotone chain algorithm on (wavelength, reflectance) points
        let mut hull_indices: Vec<usize> = Vec::new();

        for i in 0..n {
            while hull_indices.len() >= 2 {
                let a = hull_indices[hull_indices.len() - 2];
                let b = hull_indices[hull_indices.len() - 1];
                // Cross product to check if we make a left turn (concave from above)
                let cross = (wavelengths[b] - wavelengths[a])
                    * (reflectances[i] - reflectances[a])
                    - (reflectances[b] - reflectances[a])
                        * (wavelengths[i] - wavelengths[a]);
                if cross >= 0.0 {
                    // Point b is below the line a-i, so it's not on the upper hull
                    hull_indices.pop();
                } else {
                    break;
                }
            }
            hull_indices.push(i);
        }

        // Interpolate continuum at every wavelength
        let mut continuum = vec![0.0; n];
        let mut seg = 0;
        for i in 0..n {
            // Advance to the correct hull segment
            while seg + 1 < hull_indices.len() - 1
                && wavelengths[i] > wavelengths[hull_indices[seg + 1]]
            {
                seg += 1;
            }
            let a = hull_indices[seg];
            let b = hull_indices[seg + 1.min(hull_indices.len() - 1)];
            if a == b {
                continuum[i] = reflectances[a];
            } else {
                let t = (wavelengths[i] - wavelengths[a]) / (wavelengths[b] - wavelengths[a]);
                continuum[i] = reflectances[a] + t * (reflectances[b] - reflectances[a]);
            }
        }

        continuum
    }

    /// Compute the continuum-removed spectrum (hull quotient).
    ///
    /// Each value is `reflectance / continuum`, producing values in [0, 1].
    /// Absorption features appear as values less than 1.0.
    pub fn compute(wavelengths: &[f64], reflectances: &[f64]) -> Vec<f64> {
        let continuum = Self::continuum(wavelengths, reflectances);
        reflectances
            .iter()
            .zip(continuum.iter())
            .map(|(&r, &c)| {
                if c.abs() < 1e-15 {
                    1.0
                } else {
                    r / c
                }
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Helper Functions
// ---------------------------------------------------------------------------

/// Compute the spectral angle (in radians) between two spectral vectors.
///
/// The Spectral Angle Mapper (SAM) metric measures the angle between two
/// vectors in n-dimensional spectral space. It is insensitive to illumination
/// differences (brightness scaling) and ranges from 0 (identical) to pi/2
/// (orthogonal).
///
/// # Formula
///
/// `SAM = arccos( (a . b) / (|a| * |b|) )`
pub fn spectral_angle(a: &[f64], b: &[f64]) -> f64 {
    assert_eq!(a.len(), b.len(), "spectral vectors must have equal length");
    let dot: f64 = a.iter().zip(b.iter()).map(|(x, y)| x * y).sum();
    let norm_a: f64 = a.iter().map(|x| x * x).sum::<f64>().sqrt();
    let norm_b: f64 = b.iter().map(|x| x * x).sum::<f64>().sqrt();
    if norm_a < 1e-15 || norm_b < 1e-15 {
        return PI / 2.0;
    }
    let cos_angle = (dot / (norm_a * norm_b)).clamp(-1.0, 1.0);
    cos_angle.acos()
}

/// Compute the Euclidean distance between two spectral vectors.
///
/// Unlike SAM, Euclidean distance is affected by overall brightness
/// (albedo) differences between spectra.
pub fn euclidean_distance(a: &[f64], b: &[f64]) -> f64 {
    assert_eq!(a.len(), b.len(), "spectral vectors must have equal length");
    a.iter()
        .zip(b.iter())
        .map(|(x, y)| (x - y) * (x - y))
        .sum::<f64>()
        .sqrt()
}

/// Normalize a spectrum to unit length (L2 normalization).
///
/// Returns a new vector where `|result| = 1.0`. This removes brightness
/// (albedo) effects, leaving only the spectral shape.
pub fn normalize_spectrum(spectrum: &[f64]) -> Vec<f64> {
    let norm: f64 = spectrum.iter().map(|x| x * x).sum::<f64>().sqrt();
    if norm < 1e-15 {
        return vec![0.0; spectrum.len()];
    }
    spectrum.iter().map(|x| x / norm).collect()
}

/// Resample a spectrum from one set of wavelengths to another using
/// linear interpolation.
///
/// Wavelengths outside the source range are extrapolated by clamping to
/// the nearest endpoint value.
///
/// # Arguments
///
/// * `src_wavelengths` - Source wavelength grid (must be sorted ascending)
/// * `src_reflectances` - Reflectance values at source wavelengths
/// * `dst_wavelengths` - Target wavelength grid (must be sorted ascending)
pub fn resample_spectrum(
    src_wavelengths: &[f64],
    src_reflectances: &[f64],
    dst_wavelengths: &[f64],
) -> Vec<f64> {
    assert_eq!(
        src_wavelengths.len(),
        src_reflectances.len(),
        "source arrays must have equal length"
    );
    let n = src_wavelengths.len();
    if n == 0 {
        return vec![0.0; dst_wavelengths.len()];
    }
    if n == 1 {
        return vec![src_reflectances[0]; dst_wavelengths.len()];
    }

    dst_wavelengths
        .iter()
        .map(|&wl| {
            if wl <= src_wavelengths[0] {
                return src_reflectances[0];
            }
            if wl >= src_wavelengths[n - 1] {
                return src_reflectances[n - 1];
            }
            // Binary search for the enclosing interval
            let mut lo = 0;
            let mut hi = n - 1;
            while hi - lo > 1 {
                let mid = (lo + hi) / 2;
                if src_wavelengths[mid] <= wl {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            let t = (wl - src_wavelengths[lo]) / (src_wavelengths[hi] - src_wavelengths[lo]);
            src_reflectances[lo] + t * (src_reflectances[hi] - src_reflectances[lo])
        })
        .collect()
}

/// Compute the spectral correlation coefficient between two spectra.
///
/// Returns the Pearson correlation coefficient in [-1, 1].
pub fn spectral_correlation(a: &[f64], b: &[f64]) -> f64 {
    assert_eq!(a.len(), b.len(), "spectral vectors must have equal length");
    let n = a.len() as f64;
    if n < 2.0 {
        return 0.0;
    }
    let mean_a: f64 = a.iter().sum::<f64>() / n;
    let mean_b: f64 = b.iter().sum::<f64>() / n;
    let mut cov = 0.0;
    let mut var_a = 0.0;
    let mut var_b = 0.0;
    for (x, y) in a.iter().zip(b.iter()) {
        let da = x - mean_a;
        let db = y - mean_b;
        cov += da * db;
        var_a += da * da;
        var_b += db * db;
    }
    if var_a < 1e-15 || var_b < 1e-15 {
        return 0.0;
    }
    cov / (var_a.sqrt() * var_b.sqrt())
}

// ---------------------------------------------------------------------------
// Mineral Library
// ---------------------------------------------------------------------------

/// Reference spectral library for common rock-forming and alteration minerals.
///
/// Contains simplified diagnostic spectra in the VNIR-SWIR range (0.4-2.5 um)
/// derived from published spectral libraries (USGS, JPL, ASTER). Each spectrum
/// is sampled at a standard set of wavelengths covering the key diagnostic
/// absorption regions.
///
/// # Built-in Minerals
///
/// - **Quartz** - featureless, high reflectance (no absorption in VNIR/SWIR)
/// - **Feldspar (Orthoclase)** - broad weak feature near 2.1 um
/// - **Calcite** - strong 2.335 um CO3 absorption
/// - **Dolomite** - 2.320 um CO3 absorption (shifted from calcite)
/// - **Kaolinite** - doublet at 2.165/2.205 um (Al-OH)
/// - **Muscovite** - 2.200 um Al-OH absorption
/// - **Montmorillonite** - 1.900 um H2O + 2.210 um Al-OH
/// - **Goethite** - broad 0.900 um Fe3+ absorption
/// - **Hematite** - 0.860 um Fe3+ absorption
/// - **Chlorite** - 2.350 um Mg-OH absorption
/// - **Gypsum** - 1.750 um and 2.210 um water/sulfate absorptions
/// - **Olivine** - broad 1.050 um Fe2+ absorption
#[derive(Debug, Clone)]
pub struct MineralLibrary {
    signatures: Vec<SpectralSignature>,
}

/// Standard wavelength grid for the mineral library (10 diagnostic bands, um).
const LIBRARY_WAVELENGTHS: [f64; 10] = [
    0.500, // VNIR blue-green
    0.700, // VNIR red
    0.860, // VNIR near-IR (Fe3+ absorption region)
    1.050, // SWIR (Fe2+/olivine region)
    1.400, // SWIR (water absorption)
    1.750, // SWIR (gypsum, other hydrates)
    1.900, // SWIR (H2O molecular water)
    2.200, // SWIR (Al-OH region)
    2.335, // SWIR (CO3 region)
    2.500, // SWIR upper limit
];

impl Default for MineralLibrary {
    /// Create the default mineral library with built-in reference spectra.
    fn default() -> Self {
        let wl = LIBRARY_WAVELENGTHS.to_vec();
        let signatures = vec![
            // Quartz: essentially featureless, high reflectance
            SpectralSignature::new(
                "Quartz",
                wl.clone(),
                vec![0.70, 0.78, 0.80, 0.82, 0.80, 0.82, 0.83, 0.84, 0.84, 0.83],
            ),
            // Feldspar (orthoclase): slight decrease in SWIR
            SpectralSignature::new(
                "Feldspar",
                wl.clone(),
                vec![0.55, 0.65, 0.70, 0.72, 0.68, 0.72, 0.70, 0.60, 0.65, 0.64],
            ),
            // Calcite: strong CO3 absorption at 2.335 um
            SpectralSignature::new(
                "Calcite",
                wl.clone(),
                vec![0.60, 0.72, 0.78, 0.80, 0.75, 0.80, 0.82, 0.80, 0.40, 0.70],
            ),
            // Dolomite: CO3 absorption shifted to ~2.320 um
            SpectralSignature::new(
                "Dolomite",
                wl.clone(),
                vec![0.55, 0.68, 0.75, 0.78, 0.72, 0.78, 0.80, 0.78, 0.42, 0.68],
            ),
            // Kaolinite: Al-OH doublet near 2.165/2.205 um
            SpectralSignature::new(
                "Kaolinite",
                wl.clone(),
                vec![0.65, 0.75, 0.80, 0.82, 0.70, 0.82, 0.78, 0.42, 0.72, 0.70],
            ),
            // Muscovite: Al-OH at 2.200 um
            SpectralSignature::new(
                "Muscovite",
                wl.clone(),
                vec![0.40, 0.55, 0.62, 0.65, 0.58, 0.68, 0.65, 0.35, 0.62, 0.60],
            ),
            // Montmorillonite: H2O at 1.900 + Al-OH at 2.210
            SpectralSignature::new(
                "Montmorillonite",
                wl.clone(),
                vec![0.50, 0.62, 0.68, 0.70, 0.55, 0.72, 0.38, 0.40, 0.65, 0.62],
            ),
            // Goethite: broad Fe3+ at 0.900 um
            SpectralSignature::new(
                "Goethite",
                wl.clone(),
                vec![0.15, 0.35, 0.22, 0.42, 0.48, 0.55, 0.58, 0.60, 0.60, 0.58],
            ),
            // Hematite: Fe3+ at 0.860 um
            SpectralSignature::new(
                "Hematite",
                wl.clone(),
                vec![0.08, 0.20, 0.12, 0.35, 0.42, 0.50, 0.52, 0.55, 0.55, 0.53],
            ),
            // Chlorite: Mg-OH at 2.350 um
            SpectralSignature::new(
                "Chlorite",
                wl.clone(),
                vec![0.10, 0.20, 0.28, 0.32, 0.28, 0.38, 0.35, 0.38, 0.18, 0.30],
            ),
            // Gypsum: water/sulfate at 1.750 and 2.210
            SpectralSignature::new(
                "Gypsum",
                wl.clone(),
                vec![0.75, 0.82, 0.85, 0.86, 0.70, 0.50, 0.60, 0.48, 0.78, 0.75],
            ),
            // Olivine: broad Fe2+ at 1.050 um
            SpectralSignature::new(
                "Olivine",
                wl.clone(),
                vec![0.12, 0.22, 0.25, 0.10, 0.30, 0.40, 0.42, 0.45, 0.45, 0.43],
            ),
        ];
        Self { signatures }
    }
}

impl MineralLibrary {
    /// Create an empty mineral library.
    pub fn new() -> Self {
        Self {
            signatures: Vec::new(),
        }
    }

    /// Add a spectral signature to the library.
    pub fn add(&mut self, signature: SpectralSignature) {
        self.signatures.push(signature);
    }

    /// Look up a mineral by name (case-sensitive).
    pub fn get(&self, name: &str) -> Option<&SpectralSignature> {
        self.signatures.iter().find(|s| s.name == name)
    }

    /// Return all signatures in the library.
    pub fn all_signatures(&self) -> &[SpectralSignature] {
        &self.signatures
    }

    /// Number of minerals in the library.
    pub fn len(&self) -> usize {
        self.signatures.len()
    }

    /// Whether the library is empty.
    pub fn is_empty(&self) -> bool {
        self.signatures.is_empty()
    }

    /// Get the standard library wavelength grid.
    pub fn standard_wavelengths() -> &'static [f64] {
        &LIBRARY_WAVELENGTHS
    }
}

// ---------------------------------------------------------------------------
// Spectral Angle Mapper
// ---------------------------------------------------------------------------

/// Spectral Angle Mapper (SAM) classifier.
///
/// Classifies unknown spectra by computing the spectral angle to each reference
/// endmember and selecting the closest match. SAM is insensitive to illumination
/// (albedo) variations because it only measures the angle between vectors, not
/// their magnitude.
///
/// # Algorithm
///
/// For each reference endmember `r` and unknown pixel `p`:
///
/// `angle = arccos( (p . r) / (|p| * |r|) )`
///
/// The endmember with the smallest angle is the best match.
#[derive(Debug, Clone)]
pub struct SpectralAngleMapper {
    /// Reference endmember signatures.
    endmembers: Vec<SpectralSignature>,
    /// Maximum angle (radians) to accept a classification. Above this, the
    /// pixel is labeled "Unknown".
    pub max_angle: f64,
}

impl SpectralAngleMapper {
    /// Create a new SAM classifier with the given endmember signatures.
    pub fn new(endmembers: Vec<SpectralSignature>) -> Self {
        Self {
            endmembers,
            max_angle: PI / 4.0, // 45 degrees default threshold
        }
    }

    /// Create a SAM classifier with a custom maximum angle threshold.
    pub fn with_threshold(endmembers: Vec<SpectralSignature>, max_angle_rad: f64) -> Self {
        Self {
            endmembers,
            max_angle: max_angle_rad,
        }
    }

    /// Classify an unknown spectrum against all endmembers.
    ///
    /// Returns `(mineral_name, angle_radians)` for the best match.
    /// If the best angle exceeds `max_angle`, returns `("Unknown", angle)`.
    pub fn classify(&self, spectrum: &[f64]) -> (String, f64) {
        let mut best_name = "Unknown".to_string();
        let mut best_angle = f64::MAX;

        for em in &self.endmembers {
            if em.reflectances.len() != spectrum.len() {
                continue;
            }
            let angle = spectral_angle(&em.reflectances, spectrum);
            if angle < best_angle {
                best_angle = angle;
                best_name = em.name.clone();
            }
        }

        if best_angle > self.max_angle {
            ("Unknown".to_string(), best_angle)
        } else {
            (best_name, best_angle)
        }
    }

    /// Classify a spectrum and return angles to all endmembers, sorted by angle.
    pub fn classify_all(&self, spectrum: &[f64]) -> Vec<(String, f64)> {
        let mut results: Vec<(String, f64)> = self
            .endmembers
            .iter()
            .filter(|em| em.reflectances.len() == spectrum.len())
            .map(|em| {
                let angle = spectral_angle(&em.reflectances, spectrum);
                (em.name.clone(), angle)
            })
            .collect();
        results.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap_or(std::cmp::Ordering::Equal));
        results
    }
}

// ---------------------------------------------------------------------------
// Spectral Unmixer
// ---------------------------------------------------------------------------

/// Linear spectral unmixing for sub-pixel mineral abundance estimation.
///
/// Decomposes a measured spectrum into a weighted sum of endmember spectra
/// (reference minerals). The weights (abundances) represent the fractional
/// area of each mineral within the pixel.
///
/// # Mixing Model
///
/// `pixel = sum_i (abundance_i * endmember_i) + residual`
///
/// Subject to:
/// - **ANC** (Abundance Non-negativity): `abundance_i >= 0`
/// - **ASC** (Abundance Sum-to-one): `sum(abundance_i) = 1`
///
/// # Algorithm
///
/// Unconstrained: Ordinary Least Squares (OLS) via normal equations
/// `a = (E^T E)^(-1) E^T p`
///
/// Constrained: Iterative projection onto the simplex (FCLS).
#[derive(Debug, Clone)]
pub struct SpectralUnmixer {
    /// Endmember spectra matrix (num_bands x num_endmembers, column-major).
    endmember_matrix: Vec<Vec<f64>>,
    /// Endmember names.
    names: Vec<String>,
    /// Number of spectral bands.
    num_bands: usize,
    /// Number of endmembers.
    num_endmembers: usize,
    /// Whether to enforce non-negativity constraint.
    pub non_negative: bool,
    /// Whether to enforce sum-to-one constraint.
    pub sum_to_one: bool,
}

/// Result of spectral unmixing for a single pixel.
#[derive(Debug, Clone)]
pub struct UnmixingResult {
    /// Fractional abundances for each endmember.
    pub abundances: Vec<f64>,
    /// Endmember names corresponding to each abundance.
    pub names: Vec<String>,
    /// Root-mean-square reconstruction error.
    pub rmse: f64,
}

impl SpectralUnmixer {
    /// Create a new spectral unmixer from endmember signatures.
    ///
    /// All signatures must have the same number of bands.
    pub fn new(endmembers: &[SpectralSignature]) -> Self {
        assert!(!endmembers.is_empty(), "need at least one endmember");
        let num_bands = endmembers[0].num_bands();
        for em in endmembers {
            assert_eq!(
                em.num_bands(),
                num_bands,
                "all endmembers must have the same number of bands"
            );
        }

        let num_endmembers = endmembers.len();
        let endmember_matrix: Vec<Vec<f64>> =
            endmembers.iter().map(|em| em.reflectances.clone()).collect();
        let names: Vec<String> = endmembers.iter().map(|em| em.name.clone()).collect();

        Self {
            endmember_matrix,
            names,
            num_bands,
            num_endmembers,
            non_negative: true,
            sum_to_one: true,
        }
    }

    /// Unmix a single pixel spectrum.
    ///
    /// Returns the estimated abundances and reconstruction error.
    pub fn unmix(&self, pixel: &[f64]) -> UnmixingResult {
        assert_eq!(
            pixel.len(),
            self.num_bands,
            "pixel must have {} bands",
            self.num_bands
        );

        // Build E^T E (num_endmembers x num_endmembers)
        let m = self.num_endmembers;
        let mut ete = vec![vec![0.0; m]; m];
        for i in 0..m {
            for j in 0..m {
                let mut dot = 0.0;
                for k in 0..self.num_bands {
                    dot += self.endmember_matrix[i][k] * self.endmember_matrix[j][k];
                }
                ete[i][j] = dot;
            }
        }

        // Build E^T p (num_endmembers x 1)
        let mut etp = vec![0.0; m];
        for i in 0..m {
            let mut dot = 0.0;
            for k in 0..self.num_bands {
                dot += self.endmember_matrix[i][k] * pixel[k];
            }
            etp[i] = dot;
        }

        // Solve (E^T E) a = E^T p via Gaussian elimination
        let mut abundances = solve_linear_system(&ete, &etp);

        // Apply constraints
        if self.non_negative {
            for a in abundances.iter_mut() {
                if *a < 0.0 {
                    *a = 0.0;
                }
            }
        }

        if self.sum_to_one {
            let sum: f64 = abundances.iter().sum();
            if sum > 1e-15 {
                for a in abundances.iter_mut() {
                    *a /= sum;
                }
            }
        }

        // Compute reconstruction error
        let rmse = self.reconstruction_error(pixel, &abundances);

        UnmixingResult {
            abundances,
            names: self.names.clone(),
            rmse,
        }
    }

    /// Compute the RMSE between the original pixel and the reconstructed spectrum.
    fn reconstruction_error(&self, pixel: &[f64], abundances: &[f64]) -> f64 {
        let mut sum_sq = 0.0;
        for k in 0..self.num_bands {
            let mut reconstructed = 0.0;
            for i in 0..self.num_endmembers {
                reconstructed += abundances[i] * self.endmember_matrix[i][k];
            }
            let err = pixel[k] - reconstructed;
            sum_sq += err * err;
        }
        (sum_sq / self.num_bands as f64).sqrt()
    }
}

/// Solve a linear system Ax = b via Gaussian elimination with partial pivoting.
///
/// Returns the solution vector x. For singular or near-singular systems,
/// returns a zero vector.
fn solve_linear_system(a: &[Vec<f64>], b: &[f64]) -> Vec<f64> {
    let n = b.len();
    if n == 0 {
        return Vec::new();
    }

    // Augmented matrix [A | b]
    let mut aug: Vec<Vec<f64>> = Vec::with_capacity(n);
    for i in 0..n {
        let mut row = a[i].clone();
        row.push(b[i]);
        aug.push(row);
    }

    // Forward elimination with partial pivoting
    for col in 0..n {
        // Find pivot
        let mut max_val = aug[col][col].abs();
        let mut max_row = col;
        for row in (col + 1)..n {
            let val = aug[row][col].abs();
            if val > max_val {
                max_val = val;
                max_row = row;
            }
        }

        if max_val < 1e-12 {
            // Singular or near-singular
            return vec![0.0; n];
        }

        // Swap rows
        if max_row != col {
            aug.swap(col, max_row);
        }

        // Eliminate below
        let pivot = aug[col][col];
        for row in (col + 1)..n {
            let factor = aug[row][col] / pivot;
            for j in col..=n {
                let val = aug[col][j];
                aug[row][j] -= factor * val;
            }
        }
    }

    // Back substitution
    let mut x = vec![0.0; n];
    for i in (0..n).rev() {
        let mut sum = aug[i][n];
        for j in (i + 1)..n {
            sum -= aug[i][j] * x[j];
        }
        if aug[i][i].abs() < 1e-12 {
            x[i] = 0.0;
        } else {
            x[i] = sum / aug[i][i];
        }
    }

    x
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- SpectralBand tests ---

    #[test]
    fn test_spectral_band_new() {
        let band = SpectralBand::new(2.200, 0.42);
        assert!((band.wavelength_um - 2.200).abs() < 1e-10);
        assert!((band.reflectance - 0.42).abs() < 1e-10);
    }

    // --- SpectralSignature tests ---

    #[test]
    fn test_spectral_signature_from_bands() {
        let bands = vec![
            SpectralBand::new(0.5, 0.6),
            SpectralBand::new(1.0, 0.7),
            SpectralBand::new(2.0, 0.8),
        ];
        let sig = SpectralSignature::from_bands("Test", &bands);
        assert_eq!(sig.name, "Test");
        assert_eq!(sig.num_bands(), 3);
        assert!((sig.wavelengths[1] - 1.0).abs() < 1e-10);
        assert!((sig.reflectances[2] - 0.8).abs() < 1e-10);
    }

    #[test]
    fn test_spectral_signature_band_accessor() {
        let sig = SpectralSignature::new(
            "X",
            vec![0.5, 1.0, 1.5],
            vec![0.3, 0.5, 0.7],
        );
        let b = sig.band(1);
        assert!((b.wavelength_um - 1.0).abs() < 1e-10);
        assert!((b.reflectance - 0.5).abs() < 1e-10);
    }

    #[test]
    #[should_panic(expected = "wavelengths and reflectances must have the same length")]
    fn test_spectral_signature_length_mismatch() {
        SpectralSignature::new("Bad", vec![0.5, 1.0], vec![0.3]);
    }

    // --- Helper function tests ---

    #[test]
    fn test_spectral_angle_identical() {
        let a = vec![1.0, 2.0, 3.0];
        let angle = spectral_angle(&a, &a);
        assert!(angle.abs() < 1e-10);
    }

    #[test]
    fn test_spectral_angle_orthogonal() {
        let a = vec![1.0, 0.0, 0.0];
        let b = vec![0.0, 1.0, 0.0];
        let angle = spectral_angle(&a, &b);
        assert!((angle - PI / 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_spectral_angle_scaled() {
        // SAM should be insensitive to scaling
        let a = vec![1.0, 2.0, 3.0];
        let b = vec![10.0, 20.0, 30.0];
        let angle = spectral_angle(&a, &b);
        assert!(angle.abs() < 1e-10);
    }

    #[test]
    fn test_euclidean_distance_zero() {
        let a = vec![0.5, 0.6, 0.7];
        let d = euclidean_distance(&a, &a);
        assert!(d.abs() < 1e-10);
    }

    #[test]
    fn test_euclidean_distance_known() {
        let a = vec![0.0, 0.0];
        let b = vec![3.0, 4.0];
        let d = euclidean_distance(&a, &b);
        assert!((d - 5.0).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_spectrum() {
        let s = vec![3.0, 4.0];
        let n = normalize_spectrum(&s);
        assert!((n[0] - 0.6).abs() < 1e-10);
        assert!((n[1] - 0.8).abs() < 1e-10);
        // Unit length
        let mag: f64 = n.iter().map(|x| x * x).sum::<f64>().sqrt();
        assert!((mag - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_normalize_zero_spectrum() {
        let s = vec![0.0, 0.0, 0.0];
        let n = normalize_spectrum(&s);
        assert!(n.iter().all(|&x| x == 0.0));
    }

    #[test]
    fn test_resample_spectrum_identity() {
        let wl = vec![0.5, 1.0, 1.5, 2.0];
        let refl = vec![0.2, 0.5, 0.8, 0.6];
        let resampled = resample_spectrum(&wl, &refl, &wl);
        for (a, b) in refl.iter().zip(resampled.iter()) {
            assert!((a - b).abs() < 1e-10);
        }
    }

    #[test]
    fn test_resample_spectrum_interpolation() {
        let wl = vec![0.0, 1.0, 2.0];
        let refl = vec![0.0, 1.0, 0.0];
        let dst = vec![0.5, 1.5];
        let resampled = resample_spectrum(&wl, &refl, &dst);
        assert!((resampled[0] - 0.5).abs() < 1e-10);
        assert!((resampled[1] - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_resample_spectrum_extrapolation() {
        let wl = vec![1.0, 2.0];
        let refl = vec![0.5, 0.8];
        let dst = vec![0.0, 3.0];
        let resampled = resample_spectrum(&wl, &refl, &dst);
        // Clamp to endpoints
        assert!((resampled[0] - 0.5).abs() < 1e-10);
        assert!((resampled[1] - 0.8).abs() < 1e-10);
    }

    #[test]
    fn test_spectral_correlation_perfect() {
        let a = vec![1.0, 2.0, 3.0, 4.0];
        let b = vec![2.0, 4.0, 6.0, 8.0];
        let r = spectral_correlation(&a, &b);
        assert!((r - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_spectral_correlation_negative() {
        let a = vec![1.0, 2.0, 3.0, 4.0];
        let b = vec![4.0, 3.0, 2.0, 1.0];
        let r = spectral_correlation(&a, &b);
        assert!((r - (-1.0)).abs() < 1e-10);
    }

    // --- MineralLibrary tests ---

    #[test]
    fn test_mineral_library_default() {
        let lib = MineralLibrary::default();
        assert_eq!(lib.len(), 12);
        assert!(!lib.is_empty());
    }

    #[test]
    fn test_mineral_library_lookup() {
        let lib = MineralLibrary::default();
        let quartz = lib.get("Quartz");
        assert!(quartz.is_some());
        assert_eq!(quartz.unwrap().num_bands(), 10);
    }

    #[test]
    fn test_mineral_library_lookup_missing() {
        let lib = MineralLibrary::default();
        assert!(lib.get("Unobtanium").is_none());
    }

    #[test]
    fn test_mineral_library_add_custom() {
        let mut lib = MineralLibrary::new();
        assert!(lib.is_empty());
        lib.add(SpectralSignature::new(
            "Custom",
            vec![0.5, 1.0],
            vec![0.3, 0.7],
        ));
        assert_eq!(lib.len(), 1);
        assert!(lib.get("Custom").is_some());
    }

    // --- Continuum Removal tests ---

    #[test]
    fn test_continuum_removal_flat() {
        // Flat spectrum -> continuum-removed = all 1.0
        let wl = vec![0.5, 1.0, 1.5, 2.0, 2.5];
        let refl = vec![0.5, 0.5, 0.5, 0.5, 0.5];
        let cr = ContinuumRemoval::compute(&wl, &refl);
        for v in &cr {
            assert!((v - 1.0).abs() < 1e-10);
        }
    }

    #[test]
    fn test_continuum_removal_absorption() {
        // Spectrum with absorption dip
        let wl = vec![0.5, 1.0, 1.5, 2.0, 2.5];
        let refl = vec![0.8, 0.8, 0.4, 0.8, 0.8];
        let cr = ContinuumRemoval::compute(&wl, &refl);
        // Endpoints should be 1.0 (on the continuum)
        assert!((cr[0] - 1.0).abs() < 1e-10);
        assert!((cr[4] - 1.0).abs() < 1e-10);
        // Middle should be < 1.0 (absorption)
        assert!(cr[2] < 1.0);
        assert!((cr[2] - 0.5).abs() < 1e-10); // 0.4 / 0.8 = 0.5
    }

    #[test]
    fn test_continuum_values_above_reflectance() {
        let wl = vec![0.5, 1.0, 1.5, 2.0, 2.5];
        let refl = vec![0.8, 0.7, 0.4, 0.6, 0.8];
        let cont = ContinuumRemoval::continuum(&wl, &refl);
        // Continuum should be >= reflectance everywhere
        for (c, r) in cont.iter().zip(refl.iter()) {
            assert!(*c >= *r - 1e-10, "continuum {} < reflectance {}", c, r);
        }
    }

    // --- AbsorptionFeatureDetector tests ---

    #[test]
    fn test_detect_single_absorption() {
        let wl = vec![0.5, 1.0, 1.5, 2.0, 2.5];
        let refl = vec![0.8, 0.8, 0.3, 0.8, 0.8];
        let sig = SpectralSignature::new("Test", wl, refl);
        let detector = AbsorptionFeatureDetector::new(0.01);
        let features = detector.detect(&sig);
        assert!(!features.is_empty(), "should detect at least one absorption");
        assert!((features[0].center_wavelength_um - 1.5).abs() < 0.01);
        assert!(features[0].depth > 0.5);
    }

    #[test]
    fn test_detect_no_absorption_flat() {
        let wl = vec![0.5, 1.0, 1.5, 2.0, 2.5];
        let refl = vec![0.5, 0.5, 0.5, 0.5, 0.5];
        let sig = SpectralSignature::new("Flat", wl, refl);
        let detector = AbsorptionFeatureDetector::new(0.01);
        let features = detector.detect(&sig);
        assert!(features.is_empty(), "flat spectrum should have no absorptions");
    }

    #[test]
    fn test_detect_depth_threshold() {
        let wl = vec![0.5, 1.0, 1.5, 2.0, 2.5];
        let refl = vec![0.8, 0.8, 0.78, 0.8, 0.8]; // very shallow dip
        let sig = SpectralSignature::new("Shallow", wl, refl);

        // High threshold: should miss it
        let detector_high = AbsorptionFeatureDetector::new(0.1);
        assert!(detector_high.detect(&sig).is_empty());

        // Low threshold: should find it
        let detector_low = AbsorptionFeatureDetector::new(0.001);
        assert!(!detector_low.detect(&sig).is_empty());
    }

    #[test]
    fn test_detect_raw_interface() {
        let wl = vec![0.5, 1.0, 1.5, 2.0, 2.5];
        let refl = vec![0.8, 0.8, 0.3, 0.8, 0.8];
        let detector = AbsorptionFeatureDetector::new(0.01);
        let features = detector.detect_raw(&wl, &refl);
        assert!(!features.is_empty());
    }

    // --- SpectralAngleMapper tests ---

    #[test]
    fn test_sam_classify_exact_match() {
        let lib = MineralLibrary::default();
        let sam = SpectralAngleMapper::new(lib.all_signatures().to_vec());
        let quartz = lib.get("Quartz").unwrap();
        let (name, angle) = sam.classify(&quartz.reflectances);
        assert_eq!(name, "Quartz");
        assert!(angle < 0.01);
    }

    #[test]
    fn test_sam_classify_scaled_spectrum() {
        // SAM should classify correctly even with brightness scaling
        let lib = MineralLibrary::default();
        let sam = SpectralAngleMapper::new(lib.all_signatures().to_vec());
        let calcite = lib.get("Calcite").unwrap();
        let scaled: Vec<f64> = calcite.reflectances.iter().map(|r| r * 2.5).collect();
        let (name, angle) = sam.classify(&scaled);
        assert_eq!(name, "Calcite");
        assert!(angle < 0.01);
    }

    #[test]
    fn test_sam_classify_all_ranking() {
        let lib = MineralLibrary::default();
        let sam = SpectralAngleMapper::new(lib.all_signatures().to_vec());
        let kaolinite = lib.get("Kaolinite").unwrap();
        let results = sam.classify_all(&kaolinite.reflectances);
        // First result should be Kaolinite with angle ~0
        assert_eq!(results[0].0, "Kaolinite");
        assert!(results[0].1 < 0.01);
        // Results should be sorted by angle
        for w in results.windows(2) {
            assert!(w[0].1 <= w[1].1);
        }
    }

    #[test]
    fn test_sam_threshold_rejection() {
        let lib = MineralLibrary::default();
        let sam = SpectralAngleMapper::with_threshold(lib.all_signatures().to_vec(), 0.001);
        // Random spectrum unlikely to match anything within 0.001 rad
        let noise = vec![0.1, 0.9, 0.2, 0.8, 0.3, 0.7, 0.4, 0.6, 0.5, 0.5];
        let (name, _) = sam.classify(&noise);
        assert_eq!(name, "Unknown");
    }

    // --- SpectralUnmixer tests ---

    #[test]
    fn test_unmixer_pure_endmember() {
        let lib = MineralLibrary::default();
        let quartz = lib.get("Quartz").unwrap();
        let calcite = lib.get("Calcite").unwrap();

        let unmixer = SpectralUnmixer::new(&[quartz.clone(), calcite.clone()]);
        let result = unmixer.unmix(&quartz.reflectances);

        // Should be ~100% quartz, ~0% calcite
        assert!(result.abundances[0] > 0.8, "quartz abundance should dominate: {}", result.abundances[0]);
        assert!(result.abundances[1] < 0.2, "calcite abundance should be small: {}", result.abundances[1]);
    }

    #[test]
    fn test_unmixer_mixed_spectrum() {
        let lib = MineralLibrary::default();
        let quartz = lib.get("Quartz").unwrap();
        let calcite = lib.get("Calcite").unwrap();

        // Create a 50/50 linear mixture
        let mixed: Vec<f64> = quartz
            .reflectances
            .iter()
            .zip(calcite.reflectances.iter())
            .map(|(q, c)| 0.5 * q + 0.5 * c)
            .collect();

        let unmixer = SpectralUnmixer::new(&[quartz.clone(), calcite.clone()]);
        let result = unmixer.unmix(&mixed);

        // Should recover approximately 50/50
        assert!(
            (result.abundances[0] - 0.5).abs() < 0.15,
            "quartz abundance should be ~0.5: {}",
            result.abundances[0]
        );
        assert!(
            (result.abundances[1] - 0.5).abs() < 0.15,
            "calcite abundance should be ~0.5: {}",
            result.abundances[1]
        );
    }

    #[test]
    fn test_unmixer_sum_to_one() {
        let lib = MineralLibrary::default();
        let quartz = lib.get("Quartz").unwrap();
        let calcite = lib.get("Calcite").unwrap();
        let kaolinite = lib.get("Kaolinite").unwrap();

        let unmixer = SpectralUnmixer::new(&[quartz.clone(), calcite.clone(), kaolinite.clone()]);
        let result = unmixer.unmix(&quartz.reflectances);

        let sum: f64 = result.abundances.iter().sum();
        assert!(
            (sum - 1.0).abs() < 1e-10,
            "abundances should sum to 1.0: {}",
            sum
        );
    }

    #[test]
    fn test_unmixer_non_negative() {
        let lib = MineralLibrary::default();
        let quartz = lib.get("Quartz").unwrap();
        let calcite = lib.get("Calcite").unwrap();

        let unmixer = SpectralUnmixer::new(&[quartz.clone(), calcite.clone()]);
        let result = unmixer.unmix(&quartz.reflectances);

        for a in &result.abundances {
            assert!(*a >= 0.0, "abundance should be non-negative: {}", a);
        }
    }

    #[test]
    fn test_unmixer_rmse_perfect() {
        let lib = MineralLibrary::default();
        let quartz = lib.get("Quartz").unwrap();
        let calcite = lib.get("Calcite").unwrap();

        // Perfect mixture should have very low RMSE
        let mixed: Vec<f64> = quartz
            .reflectances
            .iter()
            .zip(calcite.reflectances.iter())
            .map(|(q, c)| 0.5 * q + 0.5 * c)
            .collect();

        let unmixer = SpectralUnmixer::new(&[quartz.clone(), calcite.clone()]);
        let result = unmixer.unmix(&mixed);
        assert!(result.rmse < 0.05, "RMSE should be low for perfect mixture: {}", result.rmse);
    }

    // --- Integration tests ---

    #[test]
    fn test_mineral_library_wavelengths() {
        let wl = MineralLibrary::standard_wavelengths();
        assert_eq!(wl.len(), 10);
        // Should be sorted ascending
        for w in wl.windows(2) {
            assert!(w[0] < w[1]);
        }
        // Should span VNIR-SWIR
        assert!(wl[0] >= 0.4 && wl[0] <= 0.6);
        assert!(wl[wl.len() - 1] >= 2.4 && wl[wl.len() - 1] <= 2.6);
    }

    #[test]
    fn test_all_library_minerals_self_classify() {
        let lib = MineralLibrary::default();
        let sam = SpectralAngleMapper::new(lib.all_signatures().to_vec());
        for sig in lib.all_signatures() {
            let (name, angle) = sam.classify(&sig.reflectances);
            assert_eq!(
                name, sig.name,
                "mineral {} was misclassified as {}",
                sig.name, name
            );
            assert!(
                angle < 0.01,
                "self-classification angle for {} should be ~0: {}",
                sig.name, angle
            );
        }
    }

    #[test]
    fn test_continuum_removal_on_library_minerals() {
        let lib = MineralLibrary::default();
        for sig in lib.all_signatures() {
            let cr = ContinuumRemoval::compute(&sig.wavelengths, &sig.reflectances);
            assert_eq!(cr.len(), sig.num_bands());
            // All values should be in [0, 1] (with small tolerance for floating point)
            for (i, &v) in cr.iter().enumerate() {
                assert!(
                    v <= 1.0 + 1e-10 && v >= 0.0 - 1e-10,
                    "CR value at band {} for {} is out of range: {}",
                    i,
                    sig.name,
                    v
                );
            }
        }
    }

    #[test]
    fn test_solve_linear_system_2x2() {
        // x + 2y = 5, 3x + 4y = 11 => x=1, y=2
        let a = vec![vec![1.0, 2.0], vec![3.0, 4.0]];
        let b = vec![5.0, 11.0];
        let x = solve_linear_system(&a, &b);
        assert!((x[0] - 1.0).abs() < 1e-10);
        assert!((x[1] - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_absorption_feature_fwhm() {
        // Gaussian-like absorption for predictable FWHM
        let n = 21;
        let center = 2.0;
        let sigma = 0.1;
        let wl: Vec<f64> = (0..n).map(|i| 1.5 + i as f64 * 0.05).collect();
        let refl: Vec<f64> = wl
            .iter()
            .map(|&w| {
                let d = (w - center) / sigma;
                0.8 - 0.5 * (-0.5 * d * d).exp()
            })
            .collect();

        let sig = SpectralSignature::new("GaussAbs", wl, refl);
        let detector = AbsorptionFeatureDetector::new(0.01);
        let features = detector.detect(&sig);
        assert!(!features.is_empty(), "should detect the Gaussian absorption");

        // Center should be near 2.0 um
        let f = &features[0];
        assert!((f.center_wavelength_um - 2.0).abs() < 0.06);
        // FWHM of a Gaussian with sigma=0.1 is 2*sqrt(2*ln2)*sigma ~ 0.2355
        assert!(f.fwhm_um > 0.1 && f.fwhm_um < 0.5);
    }
}
