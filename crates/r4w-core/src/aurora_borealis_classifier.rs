//! # Aurora Borealis Classifier
//!
//! Auroral emission classification from all-sky camera imagery and photometer data.
//!
//! Aurora borealis (northern lights) and aurora australis (southern lights) are caused
//! by charged particles from the magnetosphere exciting atmospheric gases, producing
//! characteristic spectral emissions at specific wavelengths. The dominant emissions are:
//!
//! - **Green line (557.7 nm)**: Forbidden [O I] transition, emission altitude ~90-150 km
//! - **Red line (630.0 nm)**: Forbidden [O I] transition, emission altitude ~150-400 km
//! - **Blue line (427.8 nm)**: N2+ first negative band, emission altitude ~90-120 km
//! - **Purple line (391.4 nm)**: N2+ first negative band, emission altitude ~80-100 km
//! - **H-beta (486.1 nm)**: Hydrogen Balmer beta, proton aurora at ~100-200 km
//!
//! ## Aurora Types
//!
//! Classification of aurora morphology is important for understanding
//! magnetosphere-ionosphere coupling and space weather:
//!
//! - **Discrete arcs**: Sharp, well-defined curtain-like structures aligned with magnetic field
//! - **Diffuse bands**: Broad, structureless emission regions in the equatorward auroral zone
//! - **Corona**: Radial structure seen when aurora is directly overhead
//! - **Pulsating aurora**: Quasi-periodic on/off modulation at 2-20 Hz
//! - **Proton aurora**: Diffuse emission from energetic proton precipitation (H-beta)
//! - **Stable Auroral Red (SAR) arcs**: Sub-visual 630.0 nm emission at mid-latitudes
//! - **Substorm aurora**: Dynamic brightening and poleward expansion during substorms
//!
//! ## References
//!
//! - Akasofu, S.-I. (1964). The Development of the Auroral Substorm.
//! - Vallance Jones, A. (1974). Aurora. Springer.
//! - Partamies et al. (2017). Automatic Classification of Auroral Forms.

use std::f64::consts::PI;

/// Configuration for aurora observation and classification.
#[derive(Debug, Clone)]
pub struct AuroraConfig {
    /// Photometer sampling rate in Hz.
    pub sample_rate_hz: f64,
    /// All-sky camera image width in pixels.
    pub image_width: usize,
    /// All-sky camera image height in pixels.
    pub image_height: usize,
    /// Observed wavelengths in nanometers (e.g., [557.7, 630.0, 427.8]).
    pub wavelengths_nm: Vec<f64>,
    /// Geographic latitude of observation site in degrees.
    pub latitude_deg: f64,
    /// Geographic longitude of observation site in degrees.
    pub longitude_deg: f64,
    /// Magnetic latitude of observation site in degrees (corrected geomagnetic).
    pub magnetic_latitude_deg: f64,
}

/// Morphological classification of auroral forms.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AuroraType {
    /// Discrete arc: sharp, curtain-like structure aligned with magnetic east-west.
    DiscreteArc,
    /// Diffuse band: broad, structureless emission region.
    DiffuseBand,
    /// Corona: radial pattern seen when aurora is at zenith.
    Corona,
    /// Pulsating aurora: quasi-periodic on/off modulation (2-20 Hz).
    PulsatingAurora,
    /// Proton aurora: diffuse emission from energetic proton precipitation.
    ProtonAurora,
    /// Stable Auroral Red (SAR) arc: sub-visual 630.0 nm emission at mid-latitudes.
    StableAuroralRedArc,
    /// Substorm aurora: dynamic brightening and poleward expansion.
    Substorm,
}

/// Characteristic auroral emission lines.
///
/// Each emission line corresponds to a specific atomic or molecular transition
/// in the upper atmosphere, excited by precipitating charged particles.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EmissionLine {
    /// Green line at 557.7 nm: [O I] forbidden transition (1S -> 1D).
    /// Emission altitude ~90-150 km. Lifetime ~0.7 s.
    GreenLine,
    /// Red line at 630.0 nm: [O I] forbidden transition (1D -> 3P).
    /// Emission altitude ~150-400 km. Lifetime ~110 s (metastable).
    RedLine,
    /// Blue line at 427.8 nm: N2+ first negative band (0,1).
    /// Emission altitude ~90-120 km. Lifetime ~60 ns.
    BlueLine,
    /// Purple line at 391.4 nm: N2+ first negative band (0,0).
    /// Emission altitude ~80-100 km. Lifetime ~60 ns.
    PurpleLine,
    /// Hydrogen-beta at 486.1 nm: Balmer series, proton aurora.
    /// Emission altitude ~100-200 km. Lifetime ~10 ns.
    HydrogenBeta,
}

impl EmissionLine {
    /// Returns the wavelength of this emission line in nanometers.
    pub fn wavelength_nm(&self) -> f64 {
        match self {
            EmissionLine::GreenLine => 557.7,
            EmissionLine::RedLine => 630.0,
            EmissionLine::BlueLine => 427.8,
            EmissionLine::PurpleLine => 391.4,
            EmissionLine::HydrogenBeta => 486.1,
        }
    }
}

/// Classifier for aurora type based on all-sky camera imagery and photometer data.
///
/// Uses spatial pattern analysis, temporal modulation detection, and emission
/// line ratios to determine auroral morphological type.
pub struct AuroraClassifier {
    config: AuroraConfig,
}

impl AuroraClassifier {
    /// Creates a new `AuroraClassifier` with the given configuration.
    ///
    /// # Arguments
    /// * `config` - Observation site and instrument configuration.
    pub fn new(config: AuroraConfig) -> Self {
        Self { config }
    }

    /// Returns a reference to the configuration.
    pub fn config(&self) -> &AuroraConfig {
        &self.config
    }

    /// Classifies aurora type based on spatial intensity patterns in a 2D image.
    ///
    /// Analysis heuristics:
    /// - **DiscreteArc**: High peak-to-mean ratio, strong east-west elongation
    /// - **DiffuseBand**: Low variance, broad spatial extent
    /// - **Corona**: Radial gradient from center
    /// - **PulsatingAurora**: (requires temporal data; flagged if spatial pattern is patchy)
    /// - **Substorm**: Very high peak intensity with asymmetric poleward extent
    ///
    /// # Arguments
    /// * `image` - 2D image as row-major Vec of row Vecs (each inner Vec is one row).
    ///
    /// # Returns
    /// The classified `AuroraType`.
    pub fn classify_by_intensity_pattern(&self, image: &[Vec<f64>]) -> AuroraType {
        if image.is_empty() || image[0].is_empty() {
            return AuroraType::DiffuseBand;
        }

        let rows = image.len();
        let cols = image[0].len();

        // Compute basic statistics
        let mut sum = 0.0;
        let mut max_val = f64::NEG_INFINITY;
        let mut count = 0usize;
        for row in image {
            for &v in row {
                sum += v;
                if v > max_val {
                    max_val = v;
                }
                count += 1;
            }
        }
        let mean = if count > 0 { sum / count as f64 } else { 0.0 };

        // Compute variance
        let mut var_sum = 0.0;
        for row in image {
            for &v in row {
                let d = v - mean;
                var_sum += d * d;
            }
        }
        let variance = if count > 1 {
            var_sum / (count - 1) as f64
        } else {
            0.0
        };
        let std_dev = variance.sqrt();

        // Peak-to-mean ratio
        let peak_to_mean = if mean > 1e-12 { max_val / mean } else { 1.0 };

        // Check for arc-like structure: count bright pixels above threshold
        let threshold = mean + 2.0 * std_dev;
        let mut bright_count = 0usize;
        let mut bright_row_min = rows;
        let mut bright_row_max = 0usize;
        let mut bright_col_min = cols;
        let mut bright_col_max = 0usize;

        for (r, row) in image.iter().enumerate() {
            for (c, &v) in row.iter().enumerate() {
                if v > threshold {
                    bright_count += 1;
                    if r < bright_row_min {
                        bright_row_min = r;
                    }
                    if r > bright_row_max {
                        bright_row_max = r;
                    }
                    if c < bright_col_min {
                        bright_col_min = c;
                    }
                    if c > bright_col_max {
                        bright_col_max = c;
                    }
                }
            }
        }

        let row_extent = if bright_row_max >= bright_row_min {
            bright_row_max - bright_row_min + 1
        } else {
            0
        };
        let col_extent = if bright_col_max >= bright_col_min {
            bright_col_max - bright_col_min + 1
        } else {
            0
        };

        // Elongation: ratio of longer to shorter extent
        let elongation = if row_extent > 0 && col_extent > 0 {
            let longer = row_extent.max(col_extent) as f64;
            let shorter = row_extent.min(col_extent) as f64;
            longer / shorter
        } else {
            1.0
        };

        // Check for radial pattern from center (corona)
        let center_r = rows / 2;
        let center_c = cols / 2;
        let mut radial_score = 0.0;
        let mut radial_count = 0usize;
        for (r, row) in image.iter().enumerate() {
            for (c, &v) in row.iter().enumerate() {
                if v > threshold {
                    let dr = r as f64 - center_r as f64;
                    let dc = c as f64 - center_c as f64;
                    let dist = (dr * dr + dc * dc).sqrt();
                    if dist > 1.0 {
                        radial_score += v / dist;
                        radial_count += 1;
                    }
                }
            }
        }
        let avg_radial = if radial_count > 0 {
            radial_score / radial_count as f64
        } else {
            0.0
        };

        // Check for patchy structure (pulsating aurora indicator)
        // Count distinct bright clusters
        let bright_fraction = bright_count as f64 / count as f64;

        // Classification logic
        if peak_to_mean > 8.0 && row_extent > rows / 3 {
            // Very bright, large poleward extent -> substorm
            AuroraType::Substorm
        } else if elongation > 3.0 && peak_to_mean > 3.0 {
            // Elongated bright structure -> discrete arc
            AuroraType::DiscreteArc
        } else if avg_radial > mean * 0.5 && bright_fraction > 0.1 && bright_fraction < 0.5 {
            // Radial pattern from center -> corona
            AuroraType::Corona
        } else if bright_fraction > 0.01 && bright_fraction < 0.15 && std_dev > mean * 0.8 {
            // Patchy, high variance -> pulsating
            AuroraType::PulsatingAurora
        } else if std_dev < mean * 0.3 && bright_fraction > 0.3 {
            // Low variance, widespread -> diffuse band
            AuroraType::DiffuseBand
        } else {
            // Default: diffuse band for unstructured patterns
            AuroraType::DiffuseBand
        }
    }

    /// Detects linear arc-like structures in an all-sky camera image.
    ///
    /// Scans for contiguous bright rows that extend across a significant
    /// portion of the image width, characteristic of discrete auroral arcs.
    ///
    /// # Arguments
    /// * `image` - 2D image as row-major Vec of row Vecs.
    /// * `threshold` - Minimum intensity to consider a pixel "bright".
    ///
    /// # Returns
    /// Vector of (start_row, start_col, end_row, end_col) bounding boxes for detected arcs.
    pub fn detect_arc(
        &self,
        image: &[Vec<f64>],
        threshold: f64,
    ) -> Vec<(usize, usize, usize, usize)> {
        if image.is_empty() {
            return Vec::new();
        }
        let cols = image[0].len();
        let min_width = cols / 4; // Arc must span at least 1/4 of image width

        let mut arcs = Vec::new();
        let mut in_arc = false;
        let mut arc_start_row = 0usize;
        let mut arc_col_min = cols;
        let mut arc_col_max = 0usize;

        for (r, row) in image.iter().enumerate() {
            // Count bright pixels in this row and find their extent
            let mut row_col_min = cols;
            let mut row_col_max = 0usize;
            let mut bright_in_row = 0usize;

            for (c, &v) in row.iter().enumerate() {
                if v > threshold {
                    bright_in_row += 1;
                    if c < row_col_min {
                        row_col_min = c;
                    }
                    if c > row_col_max {
                        row_col_max = c;
                    }
                }
            }

            let row_extent = if row_col_max >= row_col_min {
                row_col_max - row_col_min + 1
            } else {
                0
            };

            let is_arc_row = bright_in_row > 0 && row_extent >= min_width;

            if is_arc_row {
                if !in_arc {
                    in_arc = true;
                    arc_start_row = r;
                    arc_col_min = row_col_min;
                    arc_col_max = row_col_max;
                } else {
                    if row_col_min < arc_col_min {
                        arc_col_min = row_col_min;
                    }
                    if row_col_max > arc_col_max {
                        arc_col_max = row_col_max;
                    }
                }
            } else if in_arc {
                arcs.push((arc_start_row, arc_col_min, r - 1, arc_col_max));
                in_arc = false;
                arc_col_min = cols;
                arc_col_max = 0;
            }
        }

        if in_arc {
            arcs.push((
                arc_start_row,
                arc_col_min,
                image.len() - 1,
                arc_col_max,
            ));
        }

        arcs
    }

    /// Detects quasi-periodic pulsation in a photometer time series.
    ///
    /// Pulsating aurora exhibits periodic intensity modulation at frequencies
    /// typically between 2-20 Hz. Uses autocorrelation to find the dominant
    /// modulation frequency.
    ///
    /// # Arguments
    /// * `time_series` - Photometer intensity samples.
    /// * `sample_rate` - Sampling rate in Hz.
    ///
    /// # Returns
    /// The detected pulsation frequency in Hz, or `None` if no periodic modulation found.
    pub fn detect_pulsation(&self, time_series: &[f64], sample_rate: f64) -> Option<f64> {
        if time_series.len() < 4 || sample_rate <= 0.0 {
            return None;
        }

        // Remove mean
        let n = time_series.len();
        let mean = time_series.iter().sum::<f64>() / n as f64;
        let centered: Vec<f64> = time_series.iter().map(|&x| x - mean).collect();

        // Compute autocorrelation for lags corresponding to 2-20 Hz
        let min_lag = (sample_rate / 20.0).ceil() as usize; // 20 Hz -> shortest period
        let max_lag = (sample_rate / 2.0).floor() as usize; // 2 Hz -> longest period

        if max_lag >= n || min_lag >= max_lag {
            return None;
        }

        // Normalization: autocorrelation at lag 0
        let r0: f64 = centered.iter().map(|&x| x * x).sum();
        if r0 < 1e-12 {
            return None;
        }

        let mut best_lag = 0usize;
        let mut best_corr = f64::NEG_INFINITY;

        for lag in min_lag..=max_lag.min(n - 1) {
            let mut corr = 0.0;
            for i in 0..(n - lag) {
                corr += centered[i] * centered[i + lag];
            }
            let normalized = corr / r0;

            if normalized > best_corr {
                best_corr = normalized;
                best_lag = lag;
            }
        }

        // Require a minimum correlation coefficient for detection
        if best_corr > 0.2 && best_lag > 0 {
            Some(sample_rate / best_lag as f64)
        } else {
            None
        }
    }

    /// Computes the I(557.7)/I(630.0) emission line intensity ratio.
    ///
    /// This ratio is a key diagnostic for auroral emission altitude:
    /// - High ratio (>2): Emission dominated by low-altitude green line (~100 km)
    /// - Low ratio (<0.5): Emission dominated by high-altitude red line (~250 km)
    /// - Ratio ~1: Mixed altitude emission (~150 km)
    ///
    /// # Arguments
    /// * `green_intensity` - Measured intensity at 557.7 nm.
    /// * `red_intensity` - Measured intensity at 630.0 nm.
    ///
    /// # Returns
    /// The I(557.7)/I(630.0) ratio. Returns `f64::INFINITY` if red_intensity is zero.
    pub fn emission_ratio_557_630(green_intensity: f64, red_intensity: f64) -> f64 {
        if red_intensity.abs() < 1e-15 {
            return f64::INFINITY;
        }
        green_intensity / red_intensity
    }

    /// Estimates emission altitude from the I(557.7)/I(630.0) ratio.
    ///
    /// Uses an empirical mapping:
    /// - Ratio >= 10: ~100 km (energetic electron precipitation, mostly green)
    /// - Ratio ~1: ~170 km (mixed emission)
    /// - Ratio <= 0.1: ~250 km (soft electron precipitation, mostly red)
    ///
    /// The relationship is approximately log-linear between ratio and altitude.
    ///
    /// # Arguments
    /// * `ratio_557_630` - The I(557.7)/I(630.0) intensity ratio.
    ///
    /// # Returns
    /// Estimated emission altitude in km.
    pub fn estimate_emission_altitude_km(ratio_557_630: f64) -> f64 {
        // Empirical log-linear model:
        // altitude = 170 - 70 * log10(ratio)
        // Clamped to physical range [90, 300] km
        if ratio_557_630 <= 0.0 {
            return 300.0;
        }
        let altitude = 170.0 - 70.0 * ratio_557_630.log10();
        altitude.clamp(90.0, 300.0)
    }

    /// Extracts a keogram (time-latitude plot) from a sequence of all-sky images.
    ///
    /// A keogram is formed by extracting a single column (typically the magnetic
    /// meridian) from each successive image, stacking them to form a
    /// time-vs-latitude display.
    ///
    /// # Arguments
    /// * `images` - Sequence of 2D images (each image is Vec<Vec<f64>>).
    /// * `column` - Column index to extract from each image.
    ///
    /// # Returns
    /// 2D Vec where rows are time steps and columns are spatial (latitude) pixels.
    /// Each row is the extracted column from the corresponding image.
    pub fn keogram_extract(
        &self,
        images: &[Vec<Vec<f64>>],
        column: usize,
    ) -> Vec<Vec<f64>> {
        let mut keogram = Vec::with_capacity(images.len());
        for image in images {
            let mut col_data = Vec::with_capacity(image.len());
            for row in image {
                if column < row.len() {
                    col_data.push(row[column]);
                } else {
                    col_data.push(0.0);
                }
            }
            keogram.push(col_data);
        }
        keogram
    }

    /// Extracts an east-west luminosity profile from a single image row.
    ///
    /// # Arguments
    /// * `image` - 2D image as row-major Vec of row Vecs.
    /// * `row` - Row index to extract.
    ///
    /// # Returns
    /// Vector of intensity values along the specified row.
    pub fn luminosity_profile(&self, image: &[Vec<f64>], row: usize) -> Vec<f64> {
        if row < image.len() {
            image[row].clone()
        } else {
            Vec::new()
        }
    }
}

/// Spectral analyzer for identifying auroral emission lines.
///
/// Identifies emission lines from measured wavelengths and provides
/// physical properties (emission height, excitation energy, radiative lifetime).
pub struct SpectralAnalyzer {
    /// Observed wavelengths in nm.
    wavelengths: Vec<f64>,
}

impl SpectralAnalyzer {
    /// Creates a new `SpectralAnalyzer` with the given observed wavelengths.
    ///
    /// # Arguments
    /// * `wavelengths` - Observed wavelengths in nanometers.
    pub fn new(wavelengths: Vec<f64>) -> Self {
        Self { wavelengths }
    }

    /// Returns a reference to the stored wavelengths.
    pub fn wavelengths(&self) -> &[f64] {
        &self.wavelengths
    }

    /// Identifies an emission line from a measured wavelength.
    ///
    /// # Arguments
    /// * `wavelength_nm` - Measured wavelength in nanometers.
    /// * `tolerance_nm` - Matching tolerance in nanometers.
    ///
    /// # Returns
    /// The matching `EmissionLine`, or `None` if no known line matches.
    pub fn identify_emission(
        &self,
        wavelength_nm: f64,
        tolerance_nm: f64,
    ) -> Option<EmissionLine> {
        let lines = [
            EmissionLine::GreenLine,
            EmissionLine::RedLine,
            EmissionLine::BlueLine,
            EmissionLine::PurpleLine,
            EmissionLine::HydrogenBeta,
        ];

        for line in &lines {
            if (wavelength_nm - line.wavelength_nm()).abs() <= tolerance_nm {
                return Some(*line);
            }
        }
        None
    }

    /// Returns the typical emission altitude for a given emission line.
    ///
    /// # Arguments
    /// * `line` - The emission line.
    ///
    /// # Returns
    /// Typical emission altitude in km.
    pub fn emission_height(&self, line: &EmissionLine) -> f64 {
        match line {
            EmissionLine::GreenLine => 110.0,    // 90-150 km, peak ~110 km
            EmissionLine::RedLine => 250.0,      // 150-400 km, peak ~250 km
            EmissionLine::BlueLine => 100.0,     // 90-120 km, peak ~100 km
            EmissionLine::PurpleLine => 90.0,    // 80-100 km, peak ~90 km
            EmissionLine::HydrogenBeta => 120.0, // 100-200 km, peak ~120 km
        }
    }

    /// Returns the excitation energy of the upper state in electron-volts.
    ///
    /// # Arguments
    /// * `line` - The emission line.
    ///
    /// # Returns
    /// Excitation energy in eV.
    pub fn excitation_energy_ev(&self, line: &EmissionLine) -> f64 {
        match line {
            EmissionLine::GreenLine => 4.19,    // O(1S) state
            EmissionLine::RedLine => 1.97,      // O(1D) state
            EmissionLine::BlueLine => 3.17,     // N2+ B state (0,1) band
            EmissionLine::PurpleLine => 3.17,   // N2+ B state (0,0) band
            EmissionLine::HydrogenBeta => 12.75, // H n=4 state
        }
    }

    /// Returns the radiative lifetime of the upper state in seconds.
    ///
    /// The 630.0 nm red line is notably metastable with a ~110 second lifetime,
    /// which is why it only appears at high altitudes where collisional quenching
    /// is low.
    ///
    /// # Arguments
    /// * `line` - The emission line.
    ///
    /// # Returns
    /// Radiative lifetime in seconds.
    pub fn lifetime_s(&self, line: &EmissionLine) -> f64 {
        match line {
            EmissionLine::GreenLine => 0.7,       // ~0.7 s
            EmissionLine::RedLine => 110.0,       // ~110 s (metastable)
            EmissionLine::BlueLine => 6.0e-8,     // ~60 ns
            EmissionLine::PurpleLine => 6.0e-8,   // ~60 ns
            EmissionLine::HydrogenBeta => 1.0e-8, // ~10 ns
        }
    }

    /// Determines whether the emission line is a forbidden transition.
    ///
    /// Forbidden lines violate electric dipole selection rules and have long
    /// radiative lifetimes. The [O I] green (557.7 nm) and red (630.0 nm) lines
    /// are both forbidden transitions of atomic oxygen.
    ///
    /// # Arguments
    /// * `line` - The emission line.
    ///
    /// # Returns
    /// `true` if the transition is forbidden, `false` otherwise.
    pub fn is_forbidden_transition(&self, line: &EmissionLine) -> bool {
        matches!(line, EmissionLine::GreenLine | EmissionLine::RedLine)
    }

    /// Converts a wavelength in nanometers to an approximate RGB color.
    ///
    /// Uses a piecewise linear approximation of the CIE color matching functions
    /// for visible wavelengths (380-780 nm).
    ///
    /// # Arguments
    /// * `wavelength_nm` - Wavelength in nanometers.
    ///
    /// # Returns
    /// (R, G, B) tuple with values in 0-255.
    pub fn color_from_wavelength(wavelength_nm: f64) -> (u8, u8, u8) {
        let wl = wavelength_nm;

        let (r, g, b) = if wl < 380.0 || wl > 780.0 {
            (0.0, 0.0, 0.0)
        } else if wl < 440.0 {
            let t = (wl - 380.0) / (440.0 - 380.0);
            (-(wl - 440.0) / (440.0 - 380.0), 0.0, 1.0 - (1.0 - t) * 0.0 + t * 0.0)
            // Violet to blue
        } else if wl < 490.0 {
            (0.0, (wl - 440.0) / (490.0 - 440.0), 1.0)
        } else if wl < 510.0 {
            (0.0, 1.0, -(wl - 510.0) / (510.0 - 490.0))
        } else if wl < 580.0 {
            ((wl - 510.0) / (580.0 - 510.0), 1.0, 0.0)
        } else if wl < 645.0 {
            (1.0, -(wl - 645.0) / (645.0 - 580.0), 0.0)
        } else {
            (1.0, 0.0, 0.0)
        };

        // Intensity correction at edges of visible spectrum
        let factor = if wl < 380.0 || wl > 780.0 {
            0.0
        } else if wl < 420.0 {
            0.3 + 0.7 * (wl - 380.0) / (420.0 - 380.0)
        } else if wl > 700.0 {
            0.3 + 0.7 * (780.0 - wl) / (780.0 - 700.0)
        } else {
            1.0
        };

        let r_byte = (r * factor * 255.0).round().clamp(0.0, 255.0) as u8;
        let g_byte = (g * factor * 255.0).round().clamp(0.0, 255.0) as u8;
        let b_byte = (b * factor * 255.0).round().clamp(0.0, 255.0) as u8;

        (r_byte, g_byte, b_byte)
    }
}

/// Monitors magnetic activity indices relevant to auroral dynamics.
///
/// Provides Kp-index mapping to auroral oval latitude, substorm onset
/// detection from magnetometer data, and auroral electrojet indices.
pub struct MagneticActivityMonitor;

impl MagneticActivityMonitor {
    /// Estimates the equatorward boundary of the auroral oval from the Kp index.
    ///
    /// Empirical relationship (Feldstein & Starkov, 1967):
    /// The auroral oval expands equatorward with increasing geomagnetic activity.
    ///
    /// # Arguments
    /// * `kp` - Kp index (0.0 to 9.0).
    ///
    /// # Returns
    /// Equatorward boundary latitude in degrees (corrected geomagnetic).
    pub fn kp_to_aurora_oval_latitude(kp: f64) -> f64 {
        // Empirical model: latitude = 67 - 1.89 * Kp
        // Kp=0 -> ~67°, Kp=9 -> ~50°
        let kp_clamped = kp.clamp(0.0, 9.0);
        67.0 - 1.89 * kp_clamped
    }

    /// Detects substorm onset from magnetometer H-component data.
    ///
    /// A substorm onset is characterized by a sudden negative excursion
    /// (negative bay) in the horizontal magnetic field component, typically
    /// >100 nT decrease within a few minutes.
    ///
    /// # Arguments
    /// * `magnetometer_h` - H-component magnetometer readings (nT).
    /// * `sample_rate` - Sampling rate in Hz.
    ///
    /// # Returns
    /// Sample index of detected onset, or `None` if no substorm detected.
    pub fn substorm_onset_detect(
        magnetometer_h: &[f64],
        sample_rate: f64,
    ) -> Option<usize> {
        if magnetometer_h.len() < 2 || sample_rate <= 0.0 {
            return None;
        }

        // Look for rapid negative change exceeding threshold
        // Window: ~60 seconds worth of samples
        let window = (60.0 * sample_rate).round() as usize;
        let window = window.max(2).min(magnetometer_h.len());

        let threshold_nt = -100.0; // Negative bay threshold

        for i in window..magnetometer_h.len() {
            // Compare current value to the value 'window' samples ago
            let delta = magnetometer_h[i] - magnetometer_h[i - window];
            if delta < threshold_nt {
                return Some(i);
            }
        }

        None
    }

    /// Computes auroral electrojet indices (AU, AL) from H-component variations.
    ///
    /// AU (Auroral Upper) is the maximum positive deviation, representing
    /// the eastward electrojet. AL (Auroral Lower) is the maximum negative
    /// deviation, representing the westward electrojet.
    ///
    /// # Arguments
    /// * `h_variations` - H-component deviations from quiet baseline (nT).
    ///
    /// # Returns
    /// Tuple of (AU, AL) in nT.
    pub fn auroral_electrojet_index(h_variations: &[f64]) -> (f64, f64) {
        if h_variations.is_empty() {
            return (0.0, 0.0);
        }

        let au = h_variations
            .iter()
            .copied()
            .fold(f64::NEG_INFINITY, f64::max);
        let al = h_variations
            .iter()
            .copied()
            .fold(f64::INFINITY, f64::min);

        (au, al)
    }

    /// Detects the expansion phase onset in a keogram.
    ///
    /// The substorm expansion phase is characterized by a sudden poleward
    /// movement of the auroral luminosity. Detected by finding the time index
    /// where the poleward boundary of bright emission moves significantly.
    ///
    /// # Arguments
    /// * `keogram` - 2D keogram (rows = time, columns = latitude pixels).
    ///
    /// # Returns
    /// Time index of expansion onset, or `None`.
    pub fn expansion_phase_detect(keogram: &[Vec<f64>]) -> Option<usize> {
        if keogram.len() < 3 {
            return None;
        }

        // For each time step, find the poleward boundary (lowest row index
        // above a brightness threshold)
        let mut boundaries = Vec::with_capacity(keogram.len());

        for row in keogram {
            // Compute mean and threshold
            let n = row.len();
            if n == 0 {
                boundaries.push(n);
                continue;
            }
            let mean = row.iter().sum::<f64>() / n as f64;
            let threshold = mean + row.iter().map(|&v| (v - mean).abs()).sum::<f64>() / n as f64;

            // Find first bright pixel (poleward boundary)
            let boundary = row
                .iter()
                .position(|&v| v > threshold)
                .unwrap_or(n);
            boundaries.push(boundary);
        }

        // Look for sudden poleward movement (boundary index decreases sharply)
        for i in 1..boundaries.len() {
            if boundaries[i - 1] > 0
                && boundaries[i] < boundaries[i - 1]
                && (boundaries[i - 1] - boundaries[i]) > keogram[0].len() / 10
            {
                return Some(i);
            }
        }

        None
    }
}

/// Processor for all-sky camera images.
///
/// Provides calibration (flat-field, star removal), geometric corrections
/// (zenith angle, van Rhijn), and coordinate mapping (pixel to azimuth/elevation).
pub struct AllSkyCameraProcessor;

impl AllSkyCameraProcessor {
    /// Applies flat-field correction to an all-sky camera image.
    ///
    /// Divides each pixel by the corresponding flat-field value to normalize
    /// non-uniform detector response and optical vignetting.
    ///
    /// # Arguments
    /// * `image` - Image to correct (modified in place).
    /// * `flat` - Flat-field reference image (same dimensions).
    pub fn flat_field_correction(image: &mut Vec<Vec<f64>>, flat: &[Vec<f64>]) {
        for (r, row) in image.iter_mut().enumerate() {
            if r < flat.len() {
                for (c, pixel) in row.iter_mut().enumerate() {
                    if c < flat[r].len() && flat[r][c].abs() > 1e-10 {
                        *pixel /= flat[r][c];
                    }
                }
            }
        }
    }

    /// Removes point sources (stars) from an all-sky camera image.
    ///
    /// Stars appear as isolated bright pixels. This function identifies pixels
    /// significantly brighter than their local neighborhood and replaces them
    /// with the local median.
    ///
    /// # Arguments
    /// * `image` - Image to process (modified in place).
    /// * `threshold` - Star detection threshold (multiplier over local median).
    pub fn star_removal(image: &mut Vec<Vec<f64>>, threshold: f64) {
        let rows = image.len();
        if rows == 0 {
            return;
        }
        let cols = image[0].len();

        // First pass: identify star pixels
        let mut star_mask = vec![vec![false; cols]; rows];

        for r in 1..rows.saturating_sub(1) {
            for c in 1..cols.saturating_sub(1) {
                // Collect 3x3 neighborhood (excluding center)
                let mut neighbors = Vec::with_capacity(8);
                for dr in 0..3usize {
                    for dc in 0..3usize {
                        let nr = r + dr - 1;
                        let nc = c + dc - 1;
                        if (nr != r || nc != c) && nr < rows && nc < cols {
                            neighbors.push(image[nr][nc]);
                        }
                    }
                }

                if neighbors.is_empty() {
                    continue;
                }

                neighbors.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                let median = neighbors[neighbors.len() / 2];

                if median > 0.0 && image[r][c] > median * threshold {
                    star_mask[r][c] = true;
                }
            }
        }

        // Second pass: replace star pixels with local median
        for r in 1..rows.saturating_sub(1) {
            for c in 1..cols.saturating_sub(1) {
                if star_mask[r][c] {
                    let mut neighbors = Vec::with_capacity(8);
                    for dr in 0..3usize {
                        for dc in 0..3usize {
                            let nr = r + dr - 1;
                            let nc = c + dc - 1;
                            if (nr != r || nc != c) && nr < rows && nc < cols && !star_mask[nr][nc]
                            {
                                neighbors.push(image[nr][nc]);
                            }
                        }
                    }

                    if !neighbors.is_empty() {
                        neighbors
                            .sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
                        image[r][c] = neighbors[neighbors.len() / 2];
                    }
                }
            }
        }
    }

    /// Applies zenith angle (van Rhijn) correction for optical path length.
    ///
    /// In an all-sky camera, the optical path through the emitting layer is
    /// longer at lower elevations. The van Rhijn factor corrects for this:
    /// `F(z) = 1 / sqrt(1 - (R_E / (R_E + h))^2 * sin^2(z))`
    ///
    /// For simplicity, we approximate with `1/cos(z)` where z is the zenith
    /// angle computed from pixel distance to center.
    ///
    /// # Arguments
    /// * `image` - Image to correct (modified in place).
    /// * `center` - (row, col) of the image center (zenith).
    pub fn zenith_angle_correction(image: &mut Vec<Vec<f64>>, center: (usize, usize)) {
        let rows = image.len();
        if rows == 0 {
            return;
        }
        let cols = image[0].len();

        // Effective radius is half the smaller dimension
        let radius = (rows.min(cols) as f64) / 2.0;
        if radius < 1.0 {
            return;
        }

        for r in 0..rows {
            for c in 0..cols {
                let dr = r as f64 - center.0 as f64;
                let dc = c as f64 - center.1 as f64;
                let dist = (dr * dr + dc * dc).sqrt();

                // Zenith angle proportional to pixel distance from center
                let zenith_angle = (PI / 2.0) * (dist / radius).min(1.0);

                // Van Rhijn correction: divide by cos(zenith_angle)
                let cos_z = zenith_angle.cos();
                if cos_z > 0.05 {
                    // Avoid divide-by-near-zero at horizon
                    image[r][c] /= cos_z;
                }
            }
        }
    }

    /// Computes the geographic azimuth for a pixel in an all-sky image.
    ///
    /// Assumes equidistant projection where pixel distance from center is
    /// proportional to zenith angle.
    ///
    /// # Arguments
    /// * `pixel` - (row, col) of the pixel.
    /// * `center` - (row, col) of the image center (zenith).
    /// * `radius` - Radius of the all-sky image in pixels.
    ///
    /// # Returns
    /// Azimuth in degrees (0=North, 90=East, 180=South, 270=West).
    pub fn azimuth_at_pixel(
        pixel: (usize, usize),
        center: (usize, usize),
        _radius: f64,
    ) -> f64 {
        let dr = pixel.0 as f64 - center.0 as f64;
        let dc = pixel.1 as f64 - center.1 as f64;

        // In image coordinates: row increases downward (south), col increases right (east)
        // Azimuth measured clockwise from north
        let azimuth_rad = dc.atan2(-dr); // -dr because north is up (decreasing row)
        let mut azimuth_deg = azimuth_rad.to_degrees();
        if azimuth_deg < 0.0 {
            azimuth_deg += 360.0;
        }
        azimuth_deg
    }

    /// Computes the elevation angle for a pixel in an all-sky image.
    ///
    /// Assumes equidistant projection: pixel distance from center is proportional
    /// to (90 - elevation).
    ///
    /// # Arguments
    /// * `pixel` - (row, col) of the pixel.
    /// * `center` - (row, col) of the image center (zenith).
    /// * `radius` - Radius of the all-sky image in pixels.
    ///
    /// # Returns
    /// Elevation angle in degrees (90=zenith, 0=horizon).
    pub fn elevation_at_pixel(
        pixel: (usize, usize),
        center: (usize, usize),
        radius: f64,
    ) -> f64 {
        let dr = pixel.0 as f64 - center.0 as f64;
        let dc = pixel.1 as f64 - center.1 as f64;
        let dist = (dr * dr + dc * dc).sqrt();

        let zenith_angle_deg = 90.0 * (dist / radius).min(1.0);
        90.0 - zenith_angle_deg
    }
}

// ============================================================
// Helper functions
// ============================================================

/// Computes approximate Magnetic Local Time (MLT) from UT and longitude.
///
/// MLT approximation (for a dipole field):
/// `MLT = UT + longitude/15`
///
/// This is a rough approximation; precise MLT requires the actual
/// subsolar point and magnetic dipole tilt.
///
/// # Arguments
/// * `ut_hours` - Universal Time in decimal hours (0.0-24.0).
/// * `longitude_deg` - Geographic longitude in degrees (east positive).
///
/// # Returns
/// Approximate MLT in hours (0.0-24.0).
pub fn magnetic_local_time(ut_hours: f64, longitude_deg: f64) -> f64 {
    let mlt = ut_hours + longitude_deg / 15.0;
    // Normalize to [0, 24)
    ((mlt % 24.0) + 24.0) % 24.0
}

/// Computes the McIlwain L-shell parameter from magnetic latitude.
///
/// `L = 1 / cos^2(lambda_m)`
///
/// The L-shell describes the set of magnetic field lines that cross
/// the equatorial plane at L Earth radii. The auroral zone is typically
/// at L ~ 5-7.
///
/// # Arguments
/// * `magnetic_latitude_deg` - Corrected geomagnetic latitude in degrees.
///
/// # Returns
/// L-shell value (dimensionless, in Earth radii).
pub fn l_shell(magnetic_latitude_deg: f64) -> f64 {
    let lambda_rad = magnetic_latitude_deg.to_radians();
    let cos_lambda = lambda_rad.cos();
    if cos_lambda.abs() < 1e-10 {
        return f64::INFINITY;
    }
    1.0 / (cos_lambda * cos_lambda)
}

/// Computes the invariant latitude from the L-shell parameter.
///
/// `lambda_m = arccos(1/sqrt(L))`
///
/// This is the inverse of `l_shell()`.
///
/// # Arguments
/// * `l_shell_val` - L-shell value.
///
/// # Returns
/// Invariant latitude in degrees.
pub fn invariant_latitude(l_shell_val: f64) -> f64 {
    if l_shell_val <= 0.0 {
        return 0.0;
    }
    let cos_lambda = (1.0 / l_shell_val).sqrt();
    let cos_lambda = cos_lambda.clamp(-1.0, 1.0);
    cos_lambda.acos().to_degrees()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> AuroraConfig {
        AuroraConfig {
            sample_rate_hz: 100.0,
            image_width: 256,
            image_height: 256,
            wavelengths_nm: vec![557.7, 630.0, 427.8],
            latitude_deg: 69.66,   // Tromso, Norway
            longitude_deg: 18.94,
            magnetic_latitude_deg: 66.5,
        }
    }

    // -------------------------------------------------------
    // Emission line identification
    // -------------------------------------------------------

    #[test]
    fn test_green_line_identification() {
        let sa = SpectralAnalyzer::new(vec![557.7]);
        let line = sa.identify_emission(557.7, 1.0);
        assert_eq!(line, Some(EmissionLine::GreenLine));
    }

    #[test]
    fn test_red_line_identification() {
        let sa = SpectralAnalyzer::new(vec![630.0]);
        let line = sa.identify_emission(630.0, 1.0);
        assert_eq!(line, Some(EmissionLine::RedLine));
    }

    #[test]
    fn test_blue_line_identification() {
        let sa = SpectralAnalyzer::new(vec![427.8]);
        let line = sa.identify_emission(427.8, 0.5);
        assert_eq!(line, Some(EmissionLine::BlueLine));
    }

    #[test]
    fn test_purple_line_identification() {
        let sa = SpectralAnalyzer::new(vec![391.4]);
        let line = sa.identify_emission(391.4, 0.5);
        assert_eq!(line, Some(EmissionLine::PurpleLine));
    }

    #[test]
    fn test_hydrogen_beta_identification() {
        let sa = SpectralAnalyzer::new(vec![486.1]);
        let line = sa.identify_emission(486.1, 0.5);
        assert_eq!(line, Some(EmissionLine::HydrogenBeta));
    }

    #[test]
    fn test_emission_line_with_tolerance() {
        let sa = SpectralAnalyzer::new(vec![]);
        // Slightly off-wavelength
        assert_eq!(sa.identify_emission(558.0, 0.5), Some(EmissionLine::GreenLine));
        assert_eq!(sa.identify_emission(558.5, 0.5), None); // Too far
    }

    #[test]
    fn test_no_match_wavelength() {
        let sa = SpectralAnalyzer::new(vec![]);
        assert_eq!(sa.identify_emission(500.0, 0.5), None);
    }

    // -------------------------------------------------------
    // Emission altitude estimation
    // -------------------------------------------------------

    #[test]
    fn test_high_ratio_low_altitude() {
        // High I(557.7)/I(630.0) ratio -> low altitude (~100 km)
        let alt = AuroraClassifier::estimate_emission_altitude_km(10.0);
        assert!(alt < 120.0, "Expected altitude < 120 km for ratio=10, got {}", alt);
        assert!(alt >= 90.0, "Expected altitude >= 90 km, got {}", alt);
    }

    #[test]
    fn test_low_ratio_high_altitude() {
        // Low I(557.7)/I(630.0) ratio -> high altitude (~250 km)
        let alt = AuroraClassifier::estimate_emission_altitude_km(0.1);
        assert!(alt > 200.0, "Expected altitude > 200 km for ratio=0.1, got {}", alt);
    }

    #[test]
    fn test_unity_ratio_intermediate_altitude() {
        let alt = AuroraClassifier::estimate_emission_altitude_km(1.0);
        // ratio=1 -> log10(1)=0 -> altitude=170
        assert!((alt - 170.0).abs() < 1.0, "Expected ~170 km for ratio=1, got {}", alt);
    }

    #[test]
    fn test_emission_ratio_calculation() {
        let ratio = AuroraClassifier::emission_ratio_557_630(100.0, 50.0);
        assert!((ratio - 2.0).abs() < 1e-10);
    }

    #[test]
    fn test_emission_ratio_zero_red() {
        let ratio = AuroraClassifier::emission_ratio_557_630(100.0, 0.0);
        assert!(ratio.is_infinite());
    }

    // -------------------------------------------------------
    // Forbidden transitions
    // -------------------------------------------------------

    #[test]
    fn test_forbidden_green_line() {
        let sa = SpectralAnalyzer::new(vec![]);
        assert!(sa.is_forbidden_transition(&EmissionLine::GreenLine));
    }

    #[test]
    fn test_forbidden_red_line() {
        let sa = SpectralAnalyzer::new(vec![]);
        assert!(sa.is_forbidden_transition(&EmissionLine::RedLine));
    }

    #[test]
    fn test_allowed_blue_line() {
        let sa = SpectralAnalyzer::new(vec![]);
        assert!(!sa.is_forbidden_transition(&EmissionLine::BlueLine));
    }

    #[test]
    fn test_allowed_purple_line() {
        let sa = SpectralAnalyzer::new(vec![]);
        assert!(!sa.is_forbidden_transition(&EmissionLine::PurpleLine));
    }

    // -------------------------------------------------------
    // Radiative lifetime
    // -------------------------------------------------------

    #[test]
    fn test_red_line_long_lifetime() {
        let sa = SpectralAnalyzer::new(vec![]);
        let lifetime = sa.lifetime_s(&EmissionLine::RedLine);
        assert!(
            (lifetime - 110.0).abs() < 5.0,
            "Expected ~110 s for 630.0 nm, got {}",
            lifetime
        );
    }

    #[test]
    fn test_green_line_lifetime() {
        let sa = SpectralAnalyzer::new(vec![]);
        let lifetime = sa.lifetime_s(&EmissionLine::GreenLine);
        assert!(
            (lifetime - 0.7).abs() < 0.1,
            "Expected ~0.7 s for 557.7 nm, got {}",
            lifetime
        );
    }

    #[test]
    fn test_blue_line_short_lifetime() {
        let sa = SpectralAnalyzer::new(vec![]);
        let lifetime = sa.lifetime_s(&EmissionLine::BlueLine);
        assert!(lifetime < 1e-6, "Expected nanosecond lifetime for N2+, got {}", lifetime);
    }

    // -------------------------------------------------------
    // Emission height
    // -------------------------------------------------------

    #[test]
    fn test_green_line_height() {
        let sa = SpectralAnalyzer::new(vec![]);
        let h = sa.emission_height(&EmissionLine::GreenLine);
        assert!((h - 110.0).abs() < 1.0);
    }

    #[test]
    fn test_red_line_height() {
        let sa = SpectralAnalyzer::new(vec![]);
        let h = sa.emission_height(&EmissionLine::RedLine);
        assert!((h - 250.0).abs() < 1.0);
    }

    // -------------------------------------------------------
    // Kp index and auroral oval
    // -------------------------------------------------------

    #[test]
    fn test_kp_0_oval_latitude() {
        let lat = MagneticActivityMonitor::kp_to_aurora_oval_latitude(0.0);
        assert!(
            (lat - 67.0).abs() < 1.0,
            "Expected ~67° for Kp=0, got {}",
            lat
        );
    }

    #[test]
    fn test_kp_9_oval_latitude() {
        let lat = MagneticActivityMonitor::kp_to_aurora_oval_latitude(9.0);
        assert!(
            (lat - 50.0).abs() < 1.0,
            "Expected ~50° for Kp=9, got {}",
            lat
        );
    }

    #[test]
    fn test_kp_intermediate_oval() {
        let lat = MagneticActivityMonitor::kp_to_aurora_oval_latitude(5.0);
        // 67 - 1.89*5 = 67 - 9.45 = 57.55
        assert!(lat > 55.0 && lat < 60.0, "Expected ~57.5° for Kp=5, got {}", lat);
    }

    // -------------------------------------------------------
    // L-shell
    // -------------------------------------------------------

    #[test]
    fn test_l_shell_auroral_zone() {
        // At 67° magnetic latitude, L = 1/cos²(67°) ≈ 1/0.1529 ≈ 6.54
        // But commonly cited as ~5.6 for the auroral zone
        // cos(67°) ≈ 0.3907, cos²(67°) ≈ 0.1527, 1/0.1527 ≈ 6.55
        // The "typical auroral zone L~5.6" corresponds to ~65° magnetic latitude
        let l = l_shell(65.0);
        assert!(
            (l - 5.6).abs() < 0.2,
            "Expected L~5.6 at 65°, got {}",
            l
        );
    }

    #[test]
    fn test_l_shell_equator() {
        let l = l_shell(0.0);
        assert!((l - 1.0).abs() < 1e-10, "Expected L=1 at equator, got {}", l);
    }

    #[test]
    fn test_l_shell_invariant_roundtrip() {
        let lat_deg = 65.0;
        let l = l_shell(lat_deg);
        let recovered = invariant_latitude(l);
        assert!(
            (recovered - lat_deg).abs() < 0.01,
            "Roundtrip failed: {} -> L={} -> {}",
            lat_deg,
            l,
            recovered
        );
    }

    #[test]
    fn test_invariant_latitude() {
        let lat = invariant_latitude(5.6);
        assert!(
            (lat - 65.0).abs() < 1.0,
            "Expected ~65° for L=5.6, got {}",
            lat
        );
    }

    // -------------------------------------------------------
    // Magnetic Local Time
    // -------------------------------------------------------

    #[test]
    fn test_mlt_at_greenwich_midnight() {
        let mlt = magnetic_local_time(0.0, 0.0);
        assert!((mlt - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_mlt_at_east_longitude() {
        // 90°E at 0 UT -> MLT = 0 + 90/15 = 6 hours
        let mlt = magnetic_local_time(0.0, 90.0);
        assert!((mlt - 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_mlt_wraps_around() {
        // 12 UT at 270°E (= -90°W) -> MLT = 12 + 270/15 = 12 + 18 = 30 -> 6
        let mlt = magnetic_local_time(12.0, 270.0);
        assert!((mlt - 6.0).abs() < 1e-10);
    }

    #[test]
    fn test_mlt_negative_longitude() {
        // 12 UT at -90° -> MLT = 12 - 6 = 6
        let mlt = magnetic_local_time(12.0, -90.0);
        assert!((mlt - 6.0).abs() < 1e-10);
    }

    // -------------------------------------------------------
    // Pulsating aurora detection
    // -------------------------------------------------------

    #[test]
    fn test_detect_pulsation_known_frequency() {
        let config = default_config();
        let classifier = AuroraClassifier::new(config);
        let sample_rate = 100.0;
        let freq = 5.0; // 5 Hz pulsation
        let n = 500;

        // Create sinusoidal modulation at 5 Hz
        let time_series: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / sample_rate;
                1.0 + 0.8 * (2.0 * PI * freq * t).sin()
            })
            .collect();

        let detected = classifier.detect_pulsation(&time_series, sample_rate);
        assert!(detected.is_some(), "Should detect pulsation");
        let detected_freq = detected.unwrap();
        assert!(
            (detected_freq - freq).abs() < 0.5,
            "Expected ~5 Hz, got {}",
            detected_freq
        );
    }

    #[test]
    fn test_detect_pulsation_no_modulation() {
        let config = default_config();
        let classifier = AuroraClassifier::new(config);
        let time_series = vec![1.0; 500];
        let result = classifier.detect_pulsation(&time_series, 100.0);
        assert!(result.is_none(), "Should not detect pulsation in constant signal");
    }

    // -------------------------------------------------------
    // Keogram
    // -------------------------------------------------------

    #[test]
    fn test_keogram_extraction_dimensions() {
        let config = default_config();
        let classifier = AuroraClassifier::new(config);

        // 10 images, each 5x8
        let images: Vec<Vec<Vec<f64>>> = (0..10)
            .map(|_| vec![vec![1.0; 8]; 5])
            .collect();

        let keogram = classifier.keogram_extract(&images, 3);
        assert_eq!(keogram.len(), 10, "Should have 10 time steps");
        assert_eq!(keogram[0].len(), 5, "Each column should have 5 latitude pixels");
    }

    #[test]
    fn test_keogram_values() {
        let config = default_config();
        let classifier = AuroraClassifier::new(config);

        // Image where each pixel has value = row * 10 + col
        let image: Vec<Vec<f64>> = (0..4)
            .map(|r| (0..6).map(|c| (r * 10 + c) as f64).collect())
            .collect();
        let images = vec![image];

        let keogram = classifier.keogram_extract(&images, 2);
        assert_eq!(keogram.len(), 1);
        // Column 2: values are 0*10+2=2, 1*10+2=12, 2*10+2=22, 3*10+2=32
        assert_eq!(keogram[0], vec![2.0, 12.0, 22.0, 32.0]);
    }

    // -------------------------------------------------------
    // Star removal
    // -------------------------------------------------------

    #[test]
    fn test_star_removal_reduces_peaks() {
        let mut image = vec![vec![10.0; 20]; 20];
        // Insert "stars" (isolated bright pixels)
        image[5][5] = 200.0;
        image[10][15] = 300.0;

        let peak_before = image.iter().flat_map(|r| r.iter()).cloned().fold(0.0_f64, f64::max);

        AllSkyCameraProcessor::star_removal(&mut image, 3.0);

        let peak_after = image.iter().flat_map(|r| r.iter()).cloned().fold(0.0_f64, f64::max);
        assert!(
            peak_after < peak_before,
            "Star removal should reduce peak: before={}, after={}",
            peak_before,
            peak_after
        );
    }

    #[test]
    fn test_star_removal_preserves_background() {
        let mut image = vec![vec![10.0; 20]; 20];
        // No stars - image should remain unchanged
        let original = image.clone();

        AllSkyCameraProcessor::star_removal(&mut image, 5.0);

        for (r, row) in image.iter().enumerate() {
            for (c, &v) in row.iter().enumerate() {
                assert!(
                    (v - original[r][c]).abs() < 1e-10,
                    "Uniform image should not change at ({}, {})",
                    r,
                    c
                );
            }
        }
    }

    // -------------------------------------------------------
    // Flat field correction
    // -------------------------------------------------------

    #[test]
    fn test_flat_field_correction_normalizes() {
        // Image with vignetting: center bright, edges dim
        let mut image = vec![
            vec![0.5, 0.7, 0.5],
            vec![0.7, 1.0, 0.7],
            vec![0.5, 0.7, 0.5],
        ];

        // Flat field represents the non-uniformity
        let flat = vec![
            vec![0.5, 0.7, 0.5],
            vec![0.7, 1.0, 0.7],
            vec![0.5, 0.7, 0.5],
        ];

        AllSkyCameraProcessor::flat_field_correction(&mut image, &flat);

        // After correction, all pixels should be ~1.0
        for row in &image {
            for &v in row {
                assert!(
                    (v - 1.0).abs() < 1e-10,
                    "Expected ~1.0 after flat-field, got {}",
                    v
                );
            }
        }
    }

    // -------------------------------------------------------
    // Zenith angle correction
    // -------------------------------------------------------

    #[test]
    fn test_zenith_correction_center_unchanged() {
        let mut image = vec![vec![1.0; 5]; 5];
        let center = (2, 2);
        AllSkyCameraProcessor::zenith_angle_correction(&mut image, center);

        // Center pixel should remain ~1.0 (zenith angle = 0)
        assert!(
            (image[2][2] - 1.0).abs() < 0.01,
            "Center pixel should be ~1.0 after zenith correction, got {}",
            image[2][2]
        );
    }

    #[test]
    fn test_zenith_correction_increases_edge_values() {
        let mut image = vec![vec![1.0; 11]; 11];
        let center = (5, 5);
        let edge_before = image[0][5];
        AllSkyCameraProcessor::zenith_angle_correction(&mut image, center);
        let edge_after = image[0][5];

        assert!(
            edge_after > edge_before,
            "Edge pixel should increase after zenith correction: {} -> {}",
            edge_before,
            edge_after
        );
    }

    // -------------------------------------------------------
    // Pixel coordinate mapping
    // -------------------------------------------------------

    #[test]
    fn test_azimuth_north() {
        // Pixel directly above center (north in image coords)
        let az = AllSkyCameraProcessor::azimuth_at_pixel((0, 50), (50, 50), 50.0);
        assert!(
            (az - 0.0).abs() < 1.0 || (az - 360.0).abs() < 1.0,
            "Expected ~0° azimuth for north pixel, got {}",
            az
        );
    }

    #[test]
    fn test_azimuth_east() {
        // Pixel directly right of center (east)
        let az = AllSkyCameraProcessor::azimuth_at_pixel((50, 100), (50, 50), 50.0);
        assert!(
            (az - 90.0).abs() < 1.0,
            "Expected ~90° azimuth for east pixel, got {}",
            az
        );
    }

    #[test]
    fn test_elevation_at_zenith() {
        let el = AllSkyCameraProcessor::elevation_at_pixel((50, 50), (50, 50), 50.0);
        assert!(
            (el - 90.0).abs() < 0.1,
            "Expected 90° at zenith, got {}",
            el
        );
    }

    #[test]
    fn test_elevation_at_horizon() {
        let el = AllSkyCameraProcessor::elevation_at_pixel((0, 50), (50, 50), 50.0);
        assert!(
            (el - 0.0).abs() < 1.0,
            "Expected ~0° at horizon, got {}",
            el
        );
    }

    // -------------------------------------------------------
    // Substorm onset detection
    // -------------------------------------------------------

    #[test]
    fn test_substorm_onset_detection() {
        let sample_rate = 1.0; // 1 Hz
        let n = 200;
        let mut h_data = vec![0.0; n];

        // Simulate a negative bay starting at sample 120
        for i in 120..n {
            h_data[i] = -2.0 * (i - 120) as f64; // -2 nT per sample
        }

        let onset = MagneticActivityMonitor::substorm_onset_detect(&h_data, sample_rate);
        assert!(onset.is_some(), "Should detect substorm onset");
    }

    #[test]
    fn test_no_substorm_in_quiet() {
        let h_data = vec![0.0; 200];
        let onset = MagneticActivityMonitor::substorm_onset_detect(&h_data, 1.0);
        assert!(onset.is_none(), "Should not detect substorm in quiet data");
    }

    // -------------------------------------------------------
    // Auroral electrojet index
    // -------------------------------------------------------

    #[test]
    fn test_electrojet_index() {
        let h_variations = vec![-200.0, -100.0, 50.0, 100.0, 150.0];
        let (au, al) = MagneticActivityMonitor::auroral_electrojet_index(&h_variations);
        assert!((au - 150.0).abs() < 1e-10);
        assert!((al - (-200.0)).abs() < 1e-10);
    }

    // -------------------------------------------------------
    // Color from wavelength
    // -------------------------------------------------------

    #[test]
    fn test_color_green_line() {
        let (r, g, b) = SpectralAnalyzer::color_from_wavelength(557.7);
        // Green line should have significant green component
        assert!(g > r, "Green line should be mostly green: R={} G={} B={}", r, g, b);
        assert!(g > b, "Green line should be mostly green: R={} G={} B={}", r, g, b);
    }

    #[test]
    fn test_color_red_line() {
        let (r, g, _b) = SpectralAnalyzer::color_from_wavelength(630.0);
        // Red line should have significant red component
        assert!(r > g, "Red line should be mostly red: R={} G={}", r, g);
    }

    #[test]
    fn test_color_out_of_visible_range() {
        let (r, g, b) = SpectralAnalyzer::color_from_wavelength(100.0);
        assert_eq!((r, g, b), (0, 0, 0), "UV should map to black");
    }

    // -------------------------------------------------------
    // Arc detection
    // -------------------------------------------------------

    #[test]
    fn test_detect_arc_in_image() {
        let config = default_config();
        let classifier = AuroraClassifier::new(config);

        // Create image with bright horizontal band (arc)
        let mut image = vec![vec![0.0; 40]; 30];
        for c in 0..40 {
            image[10][c] = 100.0;
            image[11][c] = 100.0;
        }

        let arcs = classifier.detect_arc(&image, 50.0);
        assert!(!arcs.is_empty(), "Should detect the arc");
        let (sr, _sc, er, _ec) = arcs[0];
        assert!(sr <= 11 && er >= 10, "Arc should span rows 10-11");
    }

    #[test]
    fn test_detect_no_arc_in_uniform() {
        let config = default_config();
        let classifier = AuroraClassifier::new(config);

        let image = vec![vec![1.0; 40]; 30];
        let arcs = classifier.detect_arc(&image, 50.0);
        assert!(arcs.is_empty(), "Should not detect arcs in uniform image");
    }

    // -------------------------------------------------------
    // Luminosity profile
    // -------------------------------------------------------

    #[test]
    fn test_luminosity_profile() {
        let config = default_config();
        let classifier = AuroraClassifier::new(config);
        let image = vec![
            vec![1.0, 2.0, 3.0],
            vec![4.0, 5.0, 6.0],
            vec![7.0, 8.0, 9.0],
        ];

        let profile = classifier.luminosity_profile(&image, 1);
        assert_eq!(profile, vec![4.0, 5.0, 6.0]);
    }

    #[test]
    fn test_luminosity_profile_out_of_bounds() {
        let config = default_config();
        let classifier = AuroraClassifier::new(config);
        let image = vec![vec![1.0; 3]; 3];
        let profile = classifier.luminosity_profile(&image, 10);
        assert!(profile.is_empty());
    }

    // -------------------------------------------------------
    // Classification
    // -------------------------------------------------------

    #[test]
    fn test_classify_discrete_arc() {
        let config = default_config();
        let classifier = AuroraClassifier::new(config);

        // Create image with thin bright east-west arc
        let mut image = vec![vec![1.0; 100]; 50];
        for c in 0..100 {
            image[25][c] = 50.0;
            image[26][c] = 50.0;
        }

        let result = classifier.classify_by_intensity_pattern(&image);
        assert_eq!(result, AuroraType::DiscreteArc);
    }

    #[test]
    fn test_classify_diffuse_band() {
        let config = default_config();
        let classifier = AuroraClassifier::new(config);

        // Uniform low-variance image
        let image = vec![vec![10.0; 50]; 50];

        let result = classifier.classify_by_intensity_pattern(&image);
        assert_eq!(result, AuroraType::DiffuseBand);
    }

    // -------------------------------------------------------
    // Excitation energy
    // -------------------------------------------------------

    #[test]
    fn test_excitation_energy() {
        let sa = SpectralAnalyzer::new(vec![]);
        let e = sa.excitation_energy_ev(&EmissionLine::GreenLine);
        assert!((e - 4.19).abs() < 0.1, "Expected ~4.19 eV for green line, got {}", e);

        let e2 = sa.excitation_energy_ev(&EmissionLine::HydrogenBeta);
        assert!(e2 > 10.0, "H-beta excitation should be > 10 eV, got {}", e2);
    }

    // -------------------------------------------------------
    // EmissionLine wavelength method
    // -------------------------------------------------------

    #[test]
    fn test_emission_line_wavelengths() {
        assert!((EmissionLine::GreenLine.wavelength_nm() - 557.7).abs() < 0.01);
        assert!((EmissionLine::RedLine.wavelength_nm() - 630.0).abs() < 0.01);
        assert!((EmissionLine::BlueLine.wavelength_nm() - 427.8).abs() < 0.01);
        assert!((EmissionLine::PurpleLine.wavelength_nm() - 391.4).abs() < 0.01);
        assert!((EmissionLine::HydrogenBeta.wavelength_nm() - 486.1).abs() < 0.01);
    }
}
