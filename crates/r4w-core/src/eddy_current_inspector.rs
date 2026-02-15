//! # Eddy Current Nondestructive Testing (NDT) Signal Processing
//!
//! This module implements eddy current inspection signal processing for
//! nondestructive testing of conductive materials. Eddy current testing uses
//! electromagnetic induction to detect surface and near-surface defects.
//!
//! ## Principle of Operation
//!
//! An alternating current flowing through a probe coil generates a primary
//! magnetic field. When placed near a conductive test piece, this field induces
//! circulating eddy currents in the material. These eddy currents generate
//! their own secondary magnetic field that opposes the primary field, altering
//! the coil's impedance. Defects (cracks, inclusions, corrosion) disrupt the
//! eddy current flow, causing measurable impedance changes.
//!
//! ## Key Concepts
//!
//! - **Skin depth**: The depth at which eddy current density falls to 1/e of the
//!   surface value. Given by `delta = 1 / sqrt(pi * f * mu * sigma)`.
//! - **Impedance plane**: A 2D plot of resistance vs. reactance that reveals
//!   defect signatures, lift-off variations, and material property changes.
//! - **Lift-off**: The gap between probe and test surface; causes impedance
//!   changes that must be compensated.
//! - **Multi-frequency**: Using multiple excitation frequencies to separate
//!   defect signals from lift-off and support-structure effects.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::eddy_current_inspector::*;
//!
//! let config = EddyCurrentConfig {
//!     frequency_hz: 100_000.0,
//!     coil_diameter_mm: 5.0,
//!     conductivity_ms_m: 58.0e6,
//!     permeability: 1.0,
//!     lift_off_mm: 0.1,
//!     material: MaterialType::Aluminum,
//! };
//!
//! let processor = EddyCurrentProcessor::new(config);
//! let depth = EddyCurrentProcessor::skin_depth(100_000.0, 58.0e6, 1.0);
//! assert!(depth > 0.0);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// Type of conductive material under test.
///
/// Each variant carries default conductivity and relative permeability values
/// used when explicit values are not provided.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MaterialType {
    /// 6061-T6 aluminum alloy (~25.4 MS/m, mu_r ~ 1.0)
    Aluminum,
    /// Annealed copper (~58.0 MS/m, mu_r ~ 1.0)
    Copper,
    /// Low-carbon steel (~6.99 MS/m, mu_r ~ 200)
    Steel,
    /// Ti-6Al-4V titanium alloy (~0.58 MS/m, mu_r ~ 1.0)
    Titanium,
    /// Inconel 718 nickel superalloy (~0.80 MS/m, mu_r ~ 1.0)
    Inconel,
}

impl MaterialType {
    /// Default electrical conductivity in S/m for the material.
    pub fn default_conductivity_sm(&self) -> f64 {
        match self {
            MaterialType::Aluminum => 25.4e6,
            MaterialType::Copper => 58.0e6,
            MaterialType::Steel => 6.99e6,
            MaterialType::Titanium => 0.58e6,
            MaterialType::Inconel => 0.80e6,
        }
    }

    /// Default relative magnetic permeability for the material.
    pub fn default_permeability(&self) -> f64 {
        match self {
            MaterialType::Steel => 200.0,
            _ => 1.0,
        }
    }
}

/// Classification of detected defect type based on impedance plane analysis.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DefectType {
    /// A crack originating at the surface (strong, high-phase signal).
    SurfaceCrack,
    /// A crack below the surface (weaker, lower-phase signal).
    SubsurfaceCrack,
    /// A non-metallic inclusion (moderate amplitude, distinctive phase).
    Inclusion,
    /// Material thinning due to corrosion (broad, low-amplitude signal).
    Corrosion,
    /// No defect detected.
    None,
}

// ---------------------------------------------------------------------------
// Configuration and parameter structs
// ---------------------------------------------------------------------------

/// Configuration for the eddy current inspection system.
///
/// Defines the excitation frequency, probe geometry, and material properties
/// of the test piece.
#[derive(Debug, Clone)]
pub struct EddyCurrentConfig {
    /// Excitation frequency in Hz (typical range: 100 Hz - 6 MHz).
    pub frequency_hz: f64,
    /// Probe coil outer diameter in millimetres.
    pub coil_diameter_mm: f64,
    /// Electrical conductivity of the test piece in S/m.
    pub conductivity_ms_m: f64,
    /// Relative magnetic permeability of the test piece.
    pub permeability: f64,
    /// Stand-off distance between probe and surface in mm.
    pub lift_off_mm: f64,
    /// Material type of the test piece.
    pub material: MaterialType,
}

/// A single impedance measurement at a given frequency.
///
/// The impedance is represented in both rectangular (R + jX) and polar
/// (magnitude, phase) forms.
#[derive(Debug, Clone, Copy)]
pub struct ImpedanceReading {
    /// Real part of impedance (resistance) in ohms.
    pub resistance: f64,
    /// Imaginary part of impedance (reactance) in ohms.
    pub reactance: f64,
    /// Magnitude of impedance in ohms: `sqrt(R^2 + X^2)`.
    pub magnitude: f64,
    /// Phase angle in degrees: `atan2(X, R) * 180 / pi`.
    pub phase_deg: f64,
    /// Excitation frequency at which this reading was taken.
    pub frequency_hz: f64,
}

impl ImpedanceReading {
    /// Create a new `ImpedanceReading` from rectangular components.
    ///
    /// Magnitude and phase are computed automatically.
    pub fn from_rectangular(resistance: f64, reactance: f64, frequency_hz: f64) -> Self {
        let magnitude = (resistance * resistance + reactance * reactance).sqrt();
        let phase_deg = reactance.atan2(resistance) * 180.0 / PI;
        Self {
            resistance,
            reactance,
            magnitude,
            phase_deg,
            frequency_hz,
        }
    }

    /// Create a new `ImpedanceReading` from polar components.
    ///
    /// Resistance and reactance are computed from magnitude and phase.
    pub fn from_polar(magnitude: f64, phase_deg: f64, frequency_hz: f64) -> Self {
        let phase_rad = phase_deg * PI / 180.0;
        let resistance = magnitude * phase_rad.cos();
        let reactance = magnitude * phase_rad.sin();
        Self {
            resistance,
            reactance,
            magnitude,
            phase_deg,
            frequency_hz,
        }
    }
}

/// Result of impedance plane defect analysis.
///
/// Contains the amplitude and phase of the detected indication along with
/// classification and estimated depth.
#[derive(Debug, Clone)]
pub struct DefectIndicator {
    /// Peak amplitude of the defect signal (change in impedance magnitude).
    pub amplitude: f64,
    /// Phase angle of the defect signal in degrees.
    pub phase_deg: f64,
    /// Classification of the defect type.
    pub defect_type: DefectType,
    /// Estimated defect depth below the surface in mm.
    pub estimated_depth_mm: f64,
}

/// Physical parameters of the eddy current probe coil.
#[derive(Debug, Clone)]
pub struct CoilParams {
    /// Number of wire turns in the coil.
    pub num_turns: usize,
    /// Inner radius of the coil winding in mm.
    pub inner_radius_mm: f64,
    /// Outer radius of the coil winding in mm.
    pub outer_radius_mm: f64,
    /// Height (length) of the coil winding in mm.
    pub height_mm: f64,
    /// Diameter of the winding wire in mm.
    pub wire_diameter_mm: f64,
}

// ---------------------------------------------------------------------------
// EddyCurrentProcessor
// ---------------------------------------------------------------------------

/// Core eddy current signal processor.
///
/// Provides methods for computing skin depth, converting I/Q probe signals
/// to impedance readings, analysing the impedance plane for defects, and
/// compensating for lift-off effects.
pub struct EddyCurrentProcessor {
    config: EddyCurrentConfig,
}

impl EddyCurrentProcessor {
    /// Create a new processor with the given configuration.
    pub fn new(config: EddyCurrentConfig) -> Self {
        Self { config }
    }

    /// Return a reference to the current configuration.
    pub fn config(&self) -> &EddyCurrentConfig {
        &self.config
    }

    /// Compute the standard skin depth (depth of penetration) in metres.
    ///
    /// The skin depth is the distance below the surface at which the eddy
    /// current density drops to `1/e` (~37%) of its surface value:
    ///
    /// ```text
    /// delta = 1 / sqrt(pi * f * mu_0 * mu_r * sigma)
    /// ```
    ///
    /// # Arguments
    /// - `frequency_hz` - excitation frequency in Hz
    /// - `conductivity_sm` - electrical conductivity in S/m
    /// - `permeability` - relative magnetic permeability (dimensionless)
    ///
    /// # Returns
    /// Skin depth in metres.
    pub fn skin_depth(frequency_hz: f64, conductivity_sm: f64, permeability: f64) -> f64 {
        let mu_0: f64 = 4.0 * PI * 1e-7; // permeability of free space (H/m)
        let denom = PI * frequency_hz * mu_0 * permeability * conductivity_sm;
        if denom <= 0.0 {
            return f64::INFINITY;
        }
        1.0 / denom.sqrt()
    }

    /// Standard depth of penetration — an alias for [`skin_depth`](Self::skin_depth)
    /// returning the result in metres.
    ///
    /// The standard depth of penetration (SDP) equals one skin depth and
    /// represents the effective inspection depth at the given frequency.
    pub fn standard_depth_of_penetration(
        frequency_hz: f64,
        conductivity_sm: f64,
        permeability: f64,
    ) -> f64 {
        Self::skin_depth(frequency_hz, conductivity_sm, permeability)
    }

    /// Convert in-phase (I) and quadrature (Q) probe signals to impedance readings.
    ///
    /// Each pair `(I[n], Q[n])` is treated as the real and imaginary part of
    /// the coil impedance at the configured frequency.
    ///
    /// # Panics
    /// Panics if `i_signal` and `q_signal` have different lengths.
    pub fn impedance_from_iq(
        &self,
        i_signal: &[f64],
        q_signal: &[f64],
    ) -> Vec<ImpedanceReading> {
        assert_eq!(
            i_signal.len(),
            q_signal.len(),
            "I and Q signals must have the same length"
        );
        i_signal
            .iter()
            .zip(q_signal.iter())
            .map(|(&i, &q)| {
                ImpedanceReading::from_rectangular(i, q, self.config.frequency_hz)
            })
            .collect()
    }

    /// Analyse a sequence of impedance readings for defect indications.
    ///
    /// The analysis computes the peak deviation from the mean impedance and
    /// classifies the defect based on amplitude and phase angle thresholds.
    ///
    /// # Classification rules
    /// - **SurfaceCrack**: phase angle > 60 deg and high amplitude
    /// - **SubsurfaceCrack**: phase angle 20..60 deg and moderate amplitude
    /// - **Inclusion**: phase angle 10..20 deg
    /// - **Corrosion**: phase angle < 10 deg with low amplitude
    /// - **None**: amplitude below noise threshold
    pub fn impedance_plane_analysis(&self, readings: &[ImpedanceReading]) -> DefectIndicator {
        if readings.is_empty() {
            return DefectIndicator {
                amplitude: 0.0,
                phase_deg: 0.0,
                defect_type: DefectType::None,
                estimated_depth_mm: 0.0,
            };
        }

        // Compute mean impedance
        let n = readings.len() as f64;
        let mean_r: f64 = readings.iter().map(|r| r.resistance).sum::<f64>() / n;
        let mean_x: f64 = readings.iter().map(|r| r.reactance).sum::<f64>() / n;

        // Find the reading with maximum deviation from the mean
        let mut max_amp = 0.0_f64;
        let mut max_phase = 0.0_f64;
        for r in readings {
            let dr = r.resistance - mean_r;
            let dx = r.reactance - mean_x;
            let amp = (dr * dr + dx * dx).sqrt();
            if amp > max_amp {
                max_amp = amp;
                max_phase = dx.atan2(dr) * 180.0 / PI;
            }
        }

        // Compute noise floor (standard deviation of amplitude)
        let mean_amp: f64 = readings
            .iter()
            .map(|r| {
                let dr = r.resistance - mean_r;
                let dx = r.reactance - mean_x;
                (dr * dr + dx * dx).sqrt()
            })
            .sum::<f64>()
            / n;
        let noise_threshold = mean_amp * 3.0;

        // Estimate skin depth for depth calculation
        let delta = Self::skin_depth(
            self.config.frequency_hz,
            self.config.conductivity_ms_m,
            self.config.permeability,
        );
        let delta_mm = delta * 1000.0;

        // Classify defect
        let abs_phase = max_phase.abs();
        let (defect_type, estimated_depth_mm) = if max_amp < noise_threshold || max_amp < 1e-12 {
            (DefectType::None, 0.0)
        } else if abs_phase > 60.0 {
            (DefectType::SurfaceCrack, 0.0)
        } else if abs_phase > 20.0 {
            // Subsurface: estimate depth from phase lag
            let depth = abs_phase / 180.0 * PI * delta_mm;
            (DefectType::SubsurfaceCrack, depth)
        } else if abs_phase > 10.0 {
            let depth = abs_phase / 180.0 * PI * delta_mm;
            (DefectType::Inclusion, depth)
        } else {
            (DefectType::Corrosion, delta_mm * 0.5)
        };

        DefectIndicator {
            amplitude: max_amp,
            phase_deg: max_phase,
            defect_type,
            estimated_depth_mm,
        }
    }

    /// Compensate for lift-off by rotating impedance readings in the impedance plane.
    ///
    /// Lift-off variations produce impedance changes along a characteristic
    /// angle. By rotating all readings by the negative of this angle, the
    /// lift-off component is projected onto one axis and can be ignored.
    ///
    /// # Arguments
    /// - `readings` - mutable slice of impedance readings to rotate in-place
    /// - `lift_off_angle_deg` - the lift-off direction angle in degrees
    pub fn lift_off_compensation(
        readings: &mut [ImpedanceReading],
        lift_off_angle_deg: f64,
    ) {
        let theta = -lift_off_angle_deg * PI / 180.0;
        let cos_t = theta.cos();
        let sin_t = theta.sin();

        for r in readings.iter_mut() {
            let new_r = r.resistance * cos_t - r.reactance * sin_t;
            let new_x = r.resistance * sin_t + r.reactance * cos_t;
            r.resistance = new_r;
            r.reactance = new_x;
            r.magnitude = (new_r * new_r + new_x * new_x).sqrt();
            r.phase_deg = new_x.atan2(new_r) * 180.0 / PI;
        }
    }

    /// Compute the phase lag angle (in degrees) for a defect at the given depth.
    ///
    /// The phase of the eddy current signal shifts by approximately
    /// `depth / delta` radians (one radian per skin depth). This function
    /// returns the result in degrees.
    ///
    /// # Arguments
    /// - `depth_mm` - depth of the defect below the surface in mm
    /// - `skin_depth_mm` - skin depth at the operating frequency in mm
    ///
    /// # Returns
    /// Phase lag in degrees.
    pub fn phase_angle_at_defect(depth_mm: f64, skin_depth_mm: f64) -> f64 {
        if skin_depth_mm <= 0.0 {
            return 0.0;
        }
        (depth_mm / skin_depth_mm) * (180.0 / PI)
    }
}

// ---------------------------------------------------------------------------
// ConductivityMeter
// ---------------------------------------------------------------------------

/// Conductivity measurement and conversion utilities.
///
/// Provides conversions between IACS (International Annealed Copper Standard)
/// percent and SI conductivity (S/m), temperature corrections, and impedance-
/// based conductivity estimation.
pub struct ConductivityMeter;

/// Reference conductivity of annealed copper: 58.0 MS/m = 100% IACS.
const IACS_REFERENCE_SM: f64 = 58.0e6;

impl ConductivityMeter {
    /// Convert IACS percentage to conductivity in S/m.
    ///
    /// 100% IACS = 58.0 MS/m (annealed copper at 20 deg C).
    pub fn iacs_to_sm(iacs_percent: f64) -> f64 {
        iacs_percent / 100.0 * IACS_REFERENCE_SM
    }

    /// Convert conductivity in S/m to IACS percentage.
    pub fn sm_to_iacs(conductivity_sm: f64) -> f64 {
        conductivity_sm / IACS_REFERENCE_SM * 100.0
    }

    /// Estimate material conductivity from an impedance reading and coil parameters.
    ///
    /// Uses a simplified model where the change in coil resistance due to eddy
    /// currents is proportional to `sigma * f * A_coil`:
    ///
    /// ```text
    /// sigma ~ R_measured / (k * f * N^2 * A)
    /// ```
    ///
    /// where `k` is a geometry-dependent constant (here approximated as
    /// `mu_0 * pi`), `N` is the number of turns, and `A` is the mean coil area.
    pub fn conductivity_from_impedance(
        impedance: &ImpedanceReading,
        coil_params: &CoilParams,
    ) -> f64 {
        let mu_0: f64 = 4.0 * PI * 1e-7;
        let mean_radius_m =
            (coil_params.inner_radius_mm + coil_params.outer_radius_mm) / 2.0 * 1e-3;
        let area = PI * mean_radius_m * mean_radius_m;
        let n = coil_params.num_turns as f64;
        let k = mu_0 * PI;
        let denom = k * impedance.frequency_hz * n * n * area;
        if denom.abs() < 1e-30 {
            return 0.0;
        }
        // The resistive component change is proportional to conductivity
        impedance.resistance.abs() / denom
    }

    /// Apply a linear temperature correction to a conductivity value.
    ///
    /// ```text
    /// sigma_corrected = sigma / (1 + alpha * (T - T_ref))
    /// ```
    ///
    /// # Arguments
    /// - `conductivity` - measured conductivity in S/m
    /// - `temp_c` - measurement temperature in degrees Celsius
    /// - `ref_temp_c` - reference temperature in degrees Celsius (typically 20)
    /// - `temp_coeff` - linear temperature coefficient (1/K), e.g. 0.004 for copper
    pub fn temperature_correction(
        conductivity: f64,
        temp_c: f64,
        ref_temp_c: f64,
        temp_coeff: f64,
    ) -> f64 {
        conductivity / (1.0 + temp_coeff * (temp_c - ref_temp_c))
    }
}

// ---------------------------------------------------------------------------
// MultiFrequencyProcessor
// ---------------------------------------------------------------------------

/// Multi-frequency eddy current processor.
///
/// Using multiple excitation frequencies allows separation of defect signals
/// from lift-off and support-structure artefacts, and improves depth resolution.
pub struct MultiFrequencyProcessor {
    /// Excitation frequencies in Hz.
    frequencies: Vec<f64>,
}

impl MultiFrequencyProcessor {
    /// Create a new multi-frequency processor with the given set of frequencies.
    pub fn new(frequencies: Vec<f64>) -> Self {
        Self { frequencies }
    }

    /// Return the configured excitation frequencies.
    pub fn frequencies(&self) -> &[f64] {
        &self.frequencies
    }

    /// Combine readings from multiple frequencies into a single composite set.
    ///
    /// Performs a weighted average where each frequency's readings are weighted
    /// by the inverse of the skin depth squared, giving higher weight to
    /// shallower (higher-frequency) inspections.
    ///
    /// All inner vectors must have the same length.
    pub fn combine_frequencies(
        &self,
        readings: &[Vec<ImpedanceReading>],
    ) -> Vec<ImpedanceReading> {
        if readings.is_empty() {
            return Vec::new();
        }
        let n_samples = readings[0].len();
        let n_freqs = readings.len();

        // Compute weights from frequencies (higher freq -> smaller skin depth -> higher weight)
        let weights: Vec<f64> = self
            .frequencies
            .iter()
            .take(n_freqs)
            .map(|&f| f.sqrt()) // weight proportional to sqrt(f)
            .collect();
        let total_weight: f64 = weights.iter().sum();

        let mut combined = Vec::with_capacity(n_samples);
        for i in 0..n_samples {
            let mut r_sum = 0.0;
            let mut x_sum = 0.0;
            for (j, freq_readings) in readings.iter().enumerate() {
                if i < freq_readings.len() {
                    let w = weights.get(j).copied().unwrap_or(1.0);
                    r_sum += freq_readings[i].resistance * w;
                    x_sum += freq_readings[i].reactance * w;
                }
            }
            if total_weight > 0.0 {
                r_sum /= total_weight;
                x_sum /= total_weight;
            }
            // Use the first frequency as reference for the combined reading
            combined.push(ImpedanceReading::from_rectangular(
                r_sum,
                x_sum,
                self.frequencies[0],
            ));
        }
        combined
    }

    /// Suppress lift-off variation using multi-frequency mixing.
    ///
    /// Subtracts a scaled version of the low-frequency readings (which are
    /// dominated by lift-off) from the high-frequency readings (which are
    /// more sensitive to surface defects). The scaling factor is chosen so
    /// that the lift-off component cancels.
    ///
    /// Requires at least two frequency sets. The first is assumed to be the
    /// lowest frequency, the last the highest.
    pub fn suppress_lift_off_variation(
        &self,
        multi_freq: &[Vec<ImpedanceReading>],
    ) -> Vec<ImpedanceReading> {
        if multi_freq.len() < 2 {
            return multi_freq.first().cloned().unwrap_or_default();
        }

        let low = &multi_freq[0];
        let high = &multi_freq[multi_freq.len() - 1];
        let n = low.len().min(high.len());

        // Compute scaling factor from mean magnitude ratio
        let mean_low: f64 = low.iter().take(n).map(|r| r.magnitude).sum::<f64>() / n as f64;
        let mean_high: f64 = high.iter().take(n).map(|r| r.magnitude).sum::<f64>() / n as f64;
        let scale = if mean_low.abs() > 1e-30 {
            mean_high / mean_low
        } else {
            1.0
        };

        let freq_hz = high.first().map(|r| r.frequency_hz).unwrap_or(0.0);

        (0..n)
            .map(|i| {
                let r = high[i].resistance - scale * low[i].resistance;
                let x = high[i].reactance - scale * low[i].reactance;
                ImpedanceReading::from_rectangular(r, x, freq_hz)
            })
            .collect()
    }
}

// ---------------------------------------------------------------------------
// ScanProcessor
// ---------------------------------------------------------------------------

/// Processes raster-scan eddy current data into 2D images and defect maps.
pub struct ScanProcessor;

impl ScanProcessor {
    /// Generate a 2D C-scan amplitude image from a grid of impedance readings.
    ///
    /// Each row of `scan_data` represents one scan line. The output image
    /// contains the impedance magnitude at each grid point.
    pub fn c_scan_image(scan_data: &[Vec<ImpedanceReading>]) -> Vec<Vec<f64>> {
        scan_data
            .iter()
            .map(|row| row.iter().map(|r| r.magnitude).collect())
            .collect()
    }

    /// Identify pixel locations where the C-scan amplitude exceeds a threshold.
    ///
    /// Returns a list of `(row, col)` indices of pixels above the threshold,
    /// representing potential defect locations.
    pub fn threshold_defect_map(image: &[Vec<f64>], threshold: f64) -> Vec<(usize, usize)> {
        let mut defects = Vec::new();
        for (row_idx, row) in image.iter().enumerate() {
            for (col_idx, &val) in row.iter().enumerate() {
                if val > threshold {
                    defects.push((row_idx, col_idx));
                }
            }
        }
        defects
    }

    /// Estimate the length of a defect from a line of impedance readings.
    ///
    /// The defect length is approximated by counting the number of consecutive
    /// readings whose magnitude exceeds the mean plus two standard deviations,
    /// multiplied by the inter-sample spacing (assumed to be 1 mm if not
    /// otherwise specified).
    ///
    /// Returns the estimated defect length in mm (assuming 1 mm step size).
    pub fn defect_sizing(readings: &[ImpedanceReading]) -> f64 {
        if readings.is_empty() {
            return 0.0;
        }

        let n = readings.len() as f64;
        let magnitudes: Vec<f64> = readings.iter().map(|r| r.magnitude).collect();

        let mean = magnitudes.iter().sum::<f64>() / n;
        let variance = magnitudes.iter().map(|&m| (m - mean) * (m - mean)).sum::<f64>() / n;
        let std_dev = variance.sqrt();
        let threshold = mean + 2.0 * std_dev;

        // Find the longest consecutive run above threshold
        let mut max_run = 0_usize;
        let mut current_run = 0_usize;
        for &mag in &magnitudes {
            if mag > threshold {
                current_run += 1;
                if current_run > max_run {
                    max_run = current_run;
                }
            } else {
                current_run = 0;
            }
        }

        max_run as f64
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -----------------------------------------------------------------------
    // Skin depth tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_skin_depth_aluminum_1khz() {
        // Aluminum: sigma ~ 25.4 MS/m, mu_r = 1.0, f = 1 kHz
        // delta = 1 / sqrt(pi * 1e3 * 4*pi*1e-7 * 1.0 * 25.4e6)
        // delta ~ 3.16 mm (approximately; the exact value depends on the conductivity used)
        let delta = EddyCurrentProcessor::skin_depth(1_000.0, 25.4e6, 1.0);
        let delta_mm = delta * 1000.0;
        // Accept range 2.0 - 4.0 mm (aluminum at 1 kHz is typically ~2.6-3.2 mm)
        assert!(
            delta_mm > 2.0 && delta_mm < 4.0,
            "Skin depth in Al at 1 kHz should be ~2.6-3.2 mm, got {} mm",
            delta_mm
        );
    }

    #[test]
    fn test_skin_depth_copper_1khz() {
        // Copper 58.0 MS/m at 1 kHz
        let delta = EddyCurrentProcessor::skin_depth(1_000.0, 58.0e6, 1.0);
        let delta_mm = delta * 1000.0;
        // Copper at 1 kHz: delta ~ 2.09 mm
        assert!(
            delta_mm > 1.5 && delta_mm < 2.5,
            "Skin depth in Cu at 1 kHz should be ~2.1 mm, got {} mm",
            delta_mm
        );
    }

    #[test]
    fn test_skin_depth_decreases_with_frequency() {
        let sigma = 25.4e6;
        let mu_r = 1.0;
        let d1 = EddyCurrentProcessor::skin_depth(1_000.0, sigma, mu_r);
        let d2 = EddyCurrentProcessor::skin_depth(10_000.0, sigma, mu_r);
        let d3 = EddyCurrentProcessor::skin_depth(100_000.0, sigma, mu_r);
        assert!(d1 > d2, "skin depth should decrease with frequency");
        assert!(d2 > d3, "skin depth should decrease with frequency");
    }

    #[test]
    fn test_skin_depth_increases_with_lower_conductivity() {
        let freq = 10_000.0;
        let mu_r = 1.0;
        let d_cu = EddyCurrentProcessor::skin_depth(freq, 58.0e6, mu_r);
        let d_ti = EddyCurrentProcessor::skin_depth(freq, 0.58e6, mu_r);
        assert!(
            d_ti > d_cu,
            "titanium (lower sigma) should have deeper penetration"
        );
    }

    #[test]
    fn test_skin_depth_zero_frequency() {
        let d = EddyCurrentProcessor::skin_depth(0.0, 58.0e6, 1.0);
        assert!(d.is_infinite(), "zero frequency should give infinite skin depth");
    }

    #[test]
    fn test_standard_depth_equals_skin_depth() {
        let d1 = EddyCurrentProcessor::skin_depth(100_000.0, 25.4e6, 1.0);
        let d2 =
            EddyCurrentProcessor::standard_depth_of_penetration(100_000.0, 25.4e6, 1.0);
        assert!((d1 - d2).abs() < 1e-15, "SDP should equal skin depth");
    }

    #[test]
    fn test_skin_depth_frequency_scaling() {
        // Skin depth should scale as 1/sqrt(f)
        let sigma = 25.4e6;
        let mu_r = 1.0;
        let d1 = EddyCurrentProcessor::skin_depth(1_000.0, sigma, mu_r);
        let d4 = EddyCurrentProcessor::skin_depth(4_000.0, sigma, mu_r);
        // d1 / d4 should be sqrt(4) = 2
        let ratio = d1 / d4;
        assert!(
            (ratio - 2.0).abs() < 0.01,
            "skin depth should scale as 1/sqrt(f), ratio = {}",
            ratio
        );
    }

    // -----------------------------------------------------------------------
    // IACS conversion tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_iacs_roundtrip_copper() {
        // Copper = 100% IACS = 58.0 MS/m
        let sm = ConductivityMeter::iacs_to_sm(100.0);
        assert!(
            (sm - 58.0e6).abs() < 1.0,
            "100% IACS should be 58.0e6 S/m, got {}",
            sm
        );
        let iacs = ConductivityMeter::sm_to_iacs(58.0e6);
        assert!(
            (iacs - 100.0).abs() < 1e-10,
            "58.0e6 S/m should be 100% IACS, got {}",
            iacs
        );
    }

    #[test]
    fn test_iacs_roundtrip_aluminum() {
        let iacs_al = ConductivityMeter::sm_to_iacs(25.4e6);
        let sm_back = ConductivityMeter::iacs_to_sm(iacs_al);
        assert!(
            (sm_back - 25.4e6).abs() < 1.0,
            "IACS roundtrip should be lossless"
        );
    }

    #[test]
    fn test_iacs_zero() {
        assert_eq!(ConductivityMeter::iacs_to_sm(0.0), 0.0);
        assert_eq!(ConductivityMeter::sm_to_iacs(0.0), 0.0);
    }

    // -----------------------------------------------------------------------
    // ImpedanceReading tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_impedance_magnitude() {
        let z = ImpedanceReading::from_rectangular(3.0, 4.0, 1000.0);
        assert!(
            (z.magnitude - 5.0).abs() < 1e-12,
            "sqrt(3^2 + 4^2) = 5, got {}",
            z.magnitude
        );
    }

    #[test]
    fn test_impedance_phase() {
        let z = ImpedanceReading::from_rectangular(1.0, 1.0, 1000.0);
        assert!(
            (z.phase_deg - 45.0).abs() < 1e-10,
            "atan2(1,1) = 45 deg, got {}",
            z.phase_deg
        );
    }

    #[test]
    fn test_impedance_from_polar() {
        let z = ImpedanceReading::from_polar(5.0, 53.1301, 1000.0);
        assert!(
            (z.resistance - 3.0).abs() < 0.01,
            "R = 5*cos(53.13) ~ 3, got {}",
            z.resistance
        );
        assert!(
            (z.reactance - 4.0).abs() < 0.01,
            "X = 5*sin(53.13) ~ 4, got {}",
            z.reactance
        );
    }

    #[test]
    fn test_impedance_purely_reactive() {
        let z = ImpedanceReading::from_rectangular(0.0, 10.0, 1000.0);
        assert!((z.phase_deg - 90.0).abs() < 1e-10);
        assert!((z.magnitude - 10.0).abs() < 1e-12);
    }

    #[test]
    fn test_impedance_purely_resistive() {
        let z = ImpedanceReading::from_rectangular(10.0, 0.0, 1000.0);
        assert!(z.phase_deg.abs() < 1e-10);
        assert!((z.magnitude - 10.0).abs() < 1e-12);
    }

    // -----------------------------------------------------------------------
    // Impedance from IQ tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_impedance_from_iq_basic() {
        let config = EddyCurrentConfig {
            frequency_hz: 100_000.0,
            coil_diameter_mm: 5.0,
            conductivity_ms_m: 25.4e6,
            permeability: 1.0,
            lift_off_mm: 0.1,
            material: MaterialType::Aluminum,
        };
        let proc = EddyCurrentProcessor::new(config);

        let i_sig = vec![1.0, 2.0, 3.0];
        let q_sig = vec![0.5, 1.0, 1.5];
        let readings = proc.impedance_from_iq(&i_sig, &q_sig);

        assert_eq!(readings.len(), 3);
        assert!((readings[0].resistance - 1.0).abs() < 1e-12);
        assert!((readings[0].reactance - 0.5).abs() < 1e-12);
    }

    #[test]
    #[should_panic(expected = "I and Q signals must have the same length")]
    fn test_impedance_from_iq_mismatched_lengths() {
        let config = EddyCurrentConfig {
            frequency_hz: 100_000.0,
            coil_diameter_mm: 5.0,
            conductivity_ms_m: 25.4e6,
            permeability: 1.0,
            lift_off_mm: 0.1,
            material: MaterialType::Aluminum,
        };
        let proc = EddyCurrentProcessor::new(config);
        proc.impedance_from_iq(&[1.0, 2.0], &[1.0]);
    }

    // -----------------------------------------------------------------------
    // Lift-off compensation tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_lift_off_compensation_rotates_phase() {
        let mut readings = vec![
            ImpedanceReading::from_rectangular(1.0, 0.0, 1000.0),
        ];
        let original_mag = readings[0].magnitude;

        // Rotate by 90 degrees: (1, 0) -> (0, 1)
        EddyCurrentProcessor::lift_off_compensation(&mut readings, 90.0);
        assert!(
            (readings[0].resistance - 0.0).abs() < 1e-10,
            "R after 90 deg rotation: {}",
            readings[0].resistance
        );
        assert!(
            (readings[0].reactance - (-1.0)).abs() < 1e-10,
            "X after 90 deg rotation should be -1: {}",
            readings[0].reactance
        );
        // Magnitude should be preserved
        assert!(
            (readings[0].magnitude - original_mag).abs() < 1e-10,
            "magnitude should be preserved after rotation"
        );
    }

    #[test]
    fn test_lift_off_compensation_45_degrees() {
        let mut readings = vec![
            ImpedanceReading::from_rectangular(1.0, 1.0, 1000.0),
        ];
        // Rotating by 45 deg should put (1,1) onto the real axis
        EddyCurrentProcessor::lift_off_compensation(&mut readings, 45.0);
        let expected_r = (2.0_f64).sqrt();
        assert!(
            (readings[0].resistance - expected_r).abs() < 1e-10,
            "after 45 deg compensation, R ~ sqrt(2), got {}",
            readings[0].resistance
        );
        assert!(
            readings[0].reactance.abs() < 1e-10,
            "after 45 deg compensation, X ~ 0, got {}",
            readings[0].reactance
        );
    }

    #[test]
    fn test_lift_off_compensation_zero_angle() {
        let mut readings = vec![
            ImpedanceReading::from_rectangular(3.0, 4.0, 1000.0),
        ];
        EddyCurrentProcessor::lift_off_compensation(&mut readings, 0.0);
        assert!((readings[0].resistance - 3.0).abs() < 1e-10);
        assert!((readings[0].reactance - 4.0).abs() < 1e-10);
    }

    // -----------------------------------------------------------------------
    // Phase angle at defect tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_phase_lag_increases_with_depth() {
        let delta_mm = 2.0;
        let p1 = EddyCurrentProcessor::phase_angle_at_defect(0.5, delta_mm);
        let p2 = EddyCurrentProcessor::phase_angle_at_defect(1.0, delta_mm);
        let p3 = EddyCurrentProcessor::phase_angle_at_defect(2.0, delta_mm);
        assert!(p2 > p1, "phase should increase with depth");
        assert!(p3 > p2, "phase should increase with depth");
    }

    #[test]
    fn test_phase_lag_at_one_skin_depth() {
        // At depth = delta, phase lag = 1 radian = 57.296 deg
        let p = EddyCurrentProcessor::phase_angle_at_defect(1.0, 1.0);
        assert!(
            (p - 57.2958).abs() < 0.01,
            "phase at 1 delta should be ~57.3 deg, got {}",
            p
        );
    }

    #[test]
    fn test_phase_lag_at_zero_depth() {
        let p = EddyCurrentProcessor::phase_angle_at_defect(0.0, 2.0);
        assert!(p.abs() < 1e-15, "phase at surface should be 0");
    }

    #[test]
    fn test_phase_lag_zero_skin_depth() {
        let p = EddyCurrentProcessor::phase_angle_at_defect(1.0, 0.0);
        assert_eq!(p, 0.0, "should handle zero skin depth");
    }

    // -----------------------------------------------------------------------
    // Temperature correction tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_temperature_correction_at_reference() {
        let sigma = 58.0e6;
        let corrected = ConductivityMeter::temperature_correction(sigma, 20.0, 20.0, 0.004);
        assert!(
            (corrected - sigma).abs() < 1.0,
            "no correction at reference temperature"
        );
    }

    #[test]
    fn test_temperature_correction_increases_at_lower_temp() {
        // Lower temperature -> higher conductivity
        let sigma_measured = 58.0e6;
        let corrected = ConductivityMeter::temperature_correction(
            sigma_measured, 10.0, 20.0, 0.004,
        );
        // At 10 C (below ref 20 C): divisor = 1 + 0.004 * (10-20) = 0.96
        // corrected = 58e6 / 0.96 > 58e6
        assert!(
            corrected > sigma_measured,
            "conductivity corrected to higher temp ref should be higher when measured at lower temp"
        );
    }

    #[test]
    fn test_temperature_correction_decreases_at_higher_temp() {
        let sigma_measured = 58.0e6;
        let corrected = ConductivityMeter::temperature_correction(
            sigma_measured, 30.0, 20.0, 0.004,
        );
        assert!(
            corrected < sigma_measured,
            "conductivity should decrease when corrected from higher temp"
        );
    }

    // -----------------------------------------------------------------------
    // Impedance plane analysis tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_impedance_plane_analysis_no_defect() {
        let config = EddyCurrentConfig {
            frequency_hz: 100_000.0,
            coil_diameter_mm: 5.0,
            conductivity_ms_m: 25.4e6,
            permeability: 1.0,
            lift_off_mm: 0.1,
            material: MaterialType::Aluminum,
        };
        let proc = EddyCurrentProcessor::new(config);

        // All identical readings -> no deviation -> no defect
        let readings: Vec<ImpedanceReading> = (0..50)
            .map(|_| ImpedanceReading::from_rectangular(10.0, 5.0, 100_000.0))
            .collect();
        let result = proc.impedance_plane_analysis(&readings);
        assert_eq!(result.defect_type, DefectType::None);
        assert!(result.amplitude < 1e-10);
    }

    #[test]
    fn test_impedance_plane_analysis_surface_crack() {
        let config = EddyCurrentConfig {
            frequency_hz: 100_000.0,
            coil_diameter_mm: 5.0,
            conductivity_ms_m: 25.4e6,
            permeability: 1.0,
            lift_off_mm: 0.1,
            material: MaterialType::Aluminum,
        };
        let proc = EddyCurrentProcessor::new(config);

        // Many baseline readings + one large high-phase deviation
        let mut readings: Vec<ImpedanceReading> = (0..50)
            .map(|_| ImpedanceReading::from_rectangular(10.0, 5.0, 100_000.0))
            .collect();
        // Add a surface crack signature: large R change + large X change at high phase
        readings.push(ImpedanceReading::from_rectangular(10.0 + 2.0, 5.0 + 20.0, 100_000.0));

        let result = proc.impedance_plane_analysis(&readings);
        assert!(
            result.amplitude > 1.0,
            "defect should have significant amplitude"
        );
    }

    #[test]
    fn test_impedance_plane_analysis_empty() {
        let config = EddyCurrentConfig {
            frequency_hz: 100_000.0,
            coil_diameter_mm: 5.0,
            conductivity_ms_m: 25.4e6,
            permeability: 1.0,
            lift_off_mm: 0.1,
            material: MaterialType::Aluminum,
        };
        let proc = EddyCurrentProcessor::new(config);
        let result = proc.impedance_plane_analysis(&[]);
        assert_eq!(result.defect_type, DefectType::None);
    }

    // -----------------------------------------------------------------------
    // Conductivity from impedance tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_conductivity_from_impedance_positive() {
        let coil = CoilParams {
            num_turns: 100,
            inner_radius_mm: 2.0,
            outer_radius_mm: 5.0,
            height_mm: 3.0,
            wire_diameter_mm: 0.2,
        };
        let z = ImpedanceReading::from_rectangular(50.0, 30.0, 100_000.0);
        let sigma = ConductivityMeter::conductivity_from_impedance(&z, &coil);
        assert!(sigma > 0.0, "conductivity should be positive");
    }

    #[test]
    fn test_conductivity_from_impedance_scales_with_resistance() {
        let coil = CoilParams {
            num_turns: 100,
            inner_radius_mm: 2.0,
            outer_radius_mm: 5.0,
            height_mm: 3.0,
            wire_diameter_mm: 0.2,
        };
        let z1 = ImpedanceReading::from_rectangular(50.0, 30.0, 100_000.0);
        let z2 = ImpedanceReading::from_rectangular(100.0, 30.0, 100_000.0);
        let s1 = ConductivityMeter::conductivity_from_impedance(&z1, &coil);
        let s2 = ConductivityMeter::conductivity_from_impedance(&z2, &coil);
        assert!(
            s2 > s1,
            "higher resistance change should indicate higher conductivity"
        );
    }

    // -----------------------------------------------------------------------
    // Multi-frequency tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_multi_frequency_combine() {
        let mfp = MultiFrequencyProcessor::new(vec![10_000.0, 100_000.0]);
        let low: Vec<ImpedanceReading> = (0..5)
            .map(|i| ImpedanceReading::from_rectangular(i as f64, 0.0, 10_000.0))
            .collect();
        let high: Vec<ImpedanceReading> = (0..5)
            .map(|i| ImpedanceReading::from_rectangular(i as f64 * 2.0, 0.0, 100_000.0))
            .collect();
        let combined = mfp.combine_frequencies(&[low, high]);
        assert_eq!(combined.len(), 5);
    }

    #[test]
    fn test_multi_frequency_combine_empty() {
        let mfp = MultiFrequencyProcessor::new(vec![10_000.0]);
        let combined = mfp.combine_frequencies(&[]);
        assert!(combined.is_empty());
    }

    #[test]
    fn test_suppress_lift_off_variation() {
        let mfp = MultiFrequencyProcessor::new(vec![10_000.0, 100_000.0]);
        // Low freq: baseline + lift-off
        let low: Vec<ImpedanceReading> = (0..10)
            .map(|i| {
                let lift_off = (i as f64 * 0.1).sin() * 2.0; // lift-off variation
                ImpedanceReading::from_rectangular(10.0 + lift_off, 5.0 + lift_off, 10_000.0)
            })
            .collect();
        // High freq: baseline + lift-off + defect
        let high: Vec<ImpedanceReading> = (0..10)
            .map(|i| {
                let lift_off = (i as f64 * 0.1).sin() * 2.0;
                let defect = if i == 5 { 5.0 } else { 0.0 };
                ImpedanceReading::from_rectangular(
                    10.0 + lift_off + defect,
                    5.0 + lift_off,
                    100_000.0,
                )
            })
            .collect();
        let result = mfp.suppress_lift_off_variation(&[low, high]);
        assert_eq!(result.len(), 10);
    }

    #[test]
    fn test_suppress_lift_off_single_freq() {
        let mfp = MultiFrequencyProcessor::new(vec![10_000.0]);
        let readings: Vec<ImpedanceReading> = (0..5)
            .map(|_| ImpedanceReading::from_rectangular(10.0, 5.0, 10_000.0))
            .collect();
        let result = mfp.suppress_lift_off_variation(&[readings.clone()]);
        assert_eq!(result.len(), 5);
    }

    // -----------------------------------------------------------------------
    // C-scan image tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_c_scan_image_dimensions() {
        let scan: Vec<Vec<ImpedanceReading>> = (0..10)
            .map(|_| {
                (0..20)
                    .map(|_| ImpedanceReading::from_rectangular(10.0, 5.0, 100_000.0))
                    .collect()
            })
            .collect();
        let image = ScanProcessor::c_scan_image(&scan);
        assert_eq!(image.len(), 10, "image should have 10 rows");
        assert_eq!(image[0].len(), 20, "image should have 20 columns");
    }

    #[test]
    fn test_c_scan_image_values() {
        let scan = vec![vec![
            ImpedanceReading::from_rectangular(3.0, 4.0, 1000.0),
        ]];
        let image = ScanProcessor::c_scan_image(&scan);
        assert!(
            (image[0][0] - 5.0).abs() < 1e-12,
            "magnitude of (3,4) should be 5"
        );
    }

    #[test]
    fn test_threshold_defect_map() {
        let image = vec![
            vec![1.0, 2.0, 3.0],
            vec![4.0, 10.0, 4.0],
            vec![1.0, 2.0, 1.0],
        ];
        let defects = ScanProcessor::threshold_defect_map(&image, 5.0);
        assert_eq!(defects.len(), 1);
        assert_eq!(defects[0], (1, 1));
    }

    #[test]
    fn test_threshold_defect_map_none() {
        let image = vec![vec![1.0, 2.0], vec![3.0, 4.0]];
        let defects = ScanProcessor::threshold_defect_map(&image, 10.0);
        assert!(defects.is_empty());
    }

    #[test]
    fn test_threshold_defect_map_all() {
        let image = vec![vec![10.0, 20.0]];
        let defects = ScanProcessor::threshold_defect_map(&image, 5.0);
        assert_eq!(defects.len(), 2);
    }

    // -----------------------------------------------------------------------
    // Defect sizing tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_defect_sizing_uniform() {
        // Uniform readings -> no defect -> size = 0
        let readings: Vec<ImpedanceReading> = (0..20)
            .map(|_| ImpedanceReading::from_rectangular(10.0, 5.0, 100_000.0))
            .collect();
        let size = ScanProcessor::defect_sizing(&readings);
        assert!(size < 1e-10, "no defect should give zero size");
    }

    #[test]
    fn test_defect_sizing_with_defect() {
        // Background + a 5-sample wide defect with higher magnitude
        let mut readings: Vec<ImpedanceReading> = (0..50)
            .map(|_| ImpedanceReading::from_rectangular(10.0, 5.0, 100_000.0))
            .collect();
        // Insert a defect from index 20 to 24 (5 samples)
        for i in 20..25 {
            readings[i] = ImpedanceReading::from_rectangular(10.0 + 50.0, 5.0 + 50.0, 100_000.0);
        }
        let size = ScanProcessor::defect_sizing(&readings);
        assert!(size >= 3.0, "defect size should be at least 3 mm, got {}", size);
    }

    #[test]
    fn test_defect_sizing_empty() {
        assert_eq!(ScanProcessor::defect_sizing(&[]), 0.0);
    }

    // -----------------------------------------------------------------------
    // Material type tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_material_default_conductivity() {
        assert!(MaterialType::Copper.default_conductivity_sm() > MaterialType::Aluminum.default_conductivity_sm());
        assert!(MaterialType::Aluminum.default_conductivity_sm() > MaterialType::Steel.default_conductivity_sm());
        assert!(MaterialType::Steel.default_conductivity_sm() > MaterialType::Titanium.default_conductivity_sm());
    }

    #[test]
    fn test_material_permeability() {
        assert_eq!(MaterialType::Aluminum.default_permeability(), 1.0);
        assert!(MaterialType::Steel.default_permeability() > 1.0);
    }

    // -----------------------------------------------------------------------
    // CoilParams tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_coil_params_creation() {
        let coil = CoilParams {
            num_turns: 200,
            inner_radius_mm: 1.0,
            outer_radius_mm: 3.0,
            height_mm: 5.0,
            wire_diameter_mm: 0.1,
        };
        assert_eq!(coil.num_turns, 200);
        assert!((coil.inner_radius_mm - 1.0).abs() < 1e-15);
    }

    // -----------------------------------------------------------------------
    // Integration-style tests
    // -----------------------------------------------------------------------

    #[test]
    fn test_full_inspection_workflow() {
        // Configure for aluminum tube inspection at 100 kHz
        let config = EddyCurrentConfig {
            frequency_hz: 100_000.0,
            coil_diameter_mm: 10.0,
            conductivity_ms_m: 25.4e6,
            permeability: 1.0,
            lift_off_mm: 0.2,
            material: MaterialType::Aluminum,
        };
        let proc = EddyCurrentProcessor::new(config);

        // Generate simulated probe data: baseline with a defect at sample 25
        let n = 50;
        let mut i_signal = vec![10.0; n];
        let mut q_signal = vec![5.0; n];

        // Inject a defect signal
        for k in 23..28 {
            i_signal[k] += 3.0;
            q_signal[k] += 8.0;
        }

        let readings = proc.impedance_from_iq(&i_signal, &q_signal);
        assert_eq!(readings.len(), n);

        let indicator = proc.impedance_plane_analysis(&readings);
        assert!(
            indicator.amplitude > 0.1,
            "defect should be detected"
        );
    }

    #[test]
    fn test_multi_frequency_workflow() {
        let mfp = MultiFrequencyProcessor::new(vec![10_000.0, 50_000.0, 200_000.0]);
        assert_eq!(mfp.frequencies().len(), 3);

        let readings: Vec<Vec<ImpedanceReading>> = mfp
            .frequencies()
            .iter()
            .map(|&f| {
                (0..20)
                    .map(|i| {
                        ImpedanceReading::from_rectangular(
                            10.0 + (i as f64 * 0.1).sin(),
                            5.0 + (i as f64 * 0.1).cos(),
                            f,
                        )
                    })
                    .collect()
            })
            .collect();

        let combined = mfp.combine_frequencies(&readings);
        assert_eq!(combined.len(), 20);
        let suppressed = mfp.suppress_lift_off_variation(&readings);
        assert_eq!(suppressed.len(), 20);
    }
}
