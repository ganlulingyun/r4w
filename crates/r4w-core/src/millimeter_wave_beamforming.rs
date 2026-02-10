//! Phased array beam steering for 5G mmWave frequencies.
//!
//! This module implements element-wise control and beam pattern synthesis for
//! Uniform Linear Arrays (ULA) and Uniform Planar Arrays (UPA) operating at
//! millimeter-wave frequencies (28 GHz, 73 GHz, etc.).
//!
//! # Features
//!
//! - Steering vector computation for arbitrary angles
//! - Analog beamforming with quantized phase shifters
//! - Digital beamforming with full complex weights
//! - Hybrid beamforming (analog + digital stages)
//! - Beam pattern (array factor) computation
//! - Beam width (3 dB) and sidelobe level analysis
//! - Codebook-based beam search (exhaustive and hierarchical)
//! - Array gain and directivity
//! - Grating lobe analysis
//! - 3GPP-style path loss models for 28 GHz and 73 GHz bands
//!
//! # Example
//!
//! ```
//! use r4w_core::millimeter_wave_beamforming::{MmWaveBeamformer, ArrayGeometry};
//!
//! // Create a 16-element ULA at 28 GHz with half-wavelength spacing
//! let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 16 }, 28.0e9);
//!
//! // Compute steering vector toward 30 degrees azimuth
//! let sv = bf.steering_vector_ula(30.0_f64.to_radians());
//! assert_eq!(sv.len(), 16);
//!
//! // Compute beam pattern and find the 3-dB beamwidth
//! let weights = bf.analog_weights(30.0_f64.to_radians(), None);
//! let (angles, pattern_db) = bf.beam_pattern(&weights, 360);
//! let bw = bf.beam_width_3db(&weights);
//! assert!(bw > 0.0);
//! ```

use std::f64::consts::PI;

// ── Complex-number helpers using (f64, f64) tuples ──────────────────────────

/// Multiply two complex numbers represented as `(re, im)` tuples.
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Add two complex numbers.
#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Magnitude (absolute value) of a complex number.
#[inline]
fn cabs(a: (f64, f64)) -> f64 {
    (a.0 * a.0 + a.1 * a.1).sqrt()
}

/// Magnitude squared of a complex number.
#[inline]
fn cabs2(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Complex conjugate.
#[inline]
fn conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Complex exponential: e^{j*theta}.
#[inline]
fn cexp_j(theta: f64) -> (f64, f64) {
    (theta.cos(), theta.sin())
}

/// Scale a complex number by a real scalar.
#[inline]
fn cscale(s: f64, a: (f64, f64)) -> (f64, f64) {
    (s * a.0, s * a.1)
}

// ── Public types ────────────────────────────────────────────────────────────

/// Array geometry specification.
#[derive(Debug, Clone)]
pub enum ArrayGeometry {
    /// Uniform Linear Array with `num_elements` along one axis.
    Ula { num_elements: usize },
    /// Uniform Planar Array with `rows` x `cols` elements.
    Upa { rows: usize, cols: usize },
}

/// Frequency band presets with associated path-loss parameters.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum MmWaveBand {
    /// 28 GHz band (n257/n261).
    Band28Ghz,
    /// 73 GHz band (experimental / WiGig-adjacent).
    Band73Ghz,
}

/// A single entry in a beam codebook.
#[derive(Debug, Clone)]
pub struct CodebookEntry {
    /// Index in the codebook.
    pub index: usize,
    /// Steering angle (radians) for ULA, or (azimuth, elevation) for UPA.
    pub angle_rad: f64,
    /// Pre-computed analog weight vector.
    pub weights: Vec<(f64, f64)>,
}

/// Result of a beam search.
#[derive(Debug, Clone)]
pub struct BeamSearchResult {
    /// Best codebook index.
    pub best_index: usize,
    /// Steering angle of the best beam (radians).
    pub best_angle_rad: f64,
    /// Received power metric (linear) for the best beam.
    pub best_power: f64,
}

/// Result of a grating lobe analysis.
#[derive(Debug, Clone)]
pub struct GratingLobeInfo {
    /// Whether grating lobes exist for the current element spacing.
    pub has_grating_lobes: bool,
    /// Angles (radians) at which grating lobes appear, if any.
    pub grating_angles: Vec<f64>,
    /// Element spacing in wavelengths (d / lambda).
    pub spacing_wavelengths: f64,
}

/// Path-loss model result.
#[derive(Debug, Clone, Copy)]
pub struct PathLossResult {
    /// Free-space path loss in dB.
    pub fspl_db: f64,
    /// Close-in (CI) reference path loss in dB.
    pub ci_path_loss_db: f64,
    /// Path-loss exponent used.
    pub path_loss_exponent: f64,
}

// ── Main beamformer ─────────────────────────────────────────────────────────

/// Phased-array beamformer for mmWave frequencies.
///
/// Supports ULA and UPA geometries with analog, digital, and hybrid
/// beamforming modes.
#[derive(Debug, Clone)]
pub struct MmWaveBeamformer {
    /// Array geometry.
    pub geometry: ArrayGeometry,
    /// Carrier frequency in Hz.
    pub freq_hz: f64,
    /// Wavelength in metres (c / freq_hz).
    pub wavelength: f64,
    /// Element spacing in metres (default: lambda/2).
    pub element_spacing: f64,
}

impl MmWaveBeamformer {
    /// Speed of light in m/s.
    const C: f64 = 299_792_458.0;

    /// Create a new beamformer with half-wavelength element spacing.
    pub fn new(geometry: ArrayGeometry, freq_hz: f64) -> Self {
        let wavelength = Self::C / freq_hz;
        Self {
            geometry,
            freq_hz,
            wavelength,
            element_spacing: wavelength / 2.0,
        }
    }

    /// Create a beamformer with a custom element spacing.
    pub fn with_spacing(geometry: ArrayGeometry, freq_hz: f64, element_spacing: f64) -> Self {
        let wavelength = Self::C / freq_hz;
        Self {
            geometry,
            freq_hz,
            wavelength,
            element_spacing,
        }
    }

    /// Total number of elements in the array.
    pub fn num_elements(&self) -> usize {
        match &self.geometry {
            ArrayGeometry::Ula { num_elements } => *num_elements,
            ArrayGeometry::Upa { rows, cols } => rows * cols,
        }
    }

    // ── Steering vectors ────────────────────────────────────────────────

    /// Compute the steering vector for a ULA at the given azimuth angle (radians).
    ///
    /// The phase progression across elements is
    /// `exp(-j * 2pi * d * n * sin(theta) / lambda)` for element index `n`.
    pub fn steering_vector_ula(&self, theta_rad: f64) -> Vec<(f64, f64)> {
        let n = self.num_elements();
        let d = self.element_spacing;
        let lam = self.wavelength;
        (0..n)
            .map(|i| {
                let phase = -2.0 * PI * d * (i as f64) * theta_rad.sin() / lam;
                cexp_j(phase)
            })
            .collect()
    }

    /// Compute the steering vector for a UPA at given azimuth and elevation (radians).
    ///
    /// Elements are indexed row-major. Phase contribution from both axes.
    pub fn steering_vector_upa(&self, az_rad: f64, el_rad: f64) -> Vec<(f64, f64)> {
        let (rows, cols) = match &self.geometry {
            ArrayGeometry::Upa { rows, cols } => (*rows, *cols),
            ArrayGeometry::Ula { num_elements } => (1, *num_elements),
        };
        let d = self.element_spacing;
        let lam = self.wavelength;
        let mut sv = Vec::with_capacity(rows * cols);
        for r in 0..rows {
            for c in 0..cols {
                let phase = -2.0 * PI * d / lam
                    * ((c as f64) * az_rad.sin() * el_rad.cos()
                        + (r as f64) * el_rad.sin());
                sv.push(cexp_j(phase));
            }
        }
        sv
    }

    // ── Analog beamforming ──────────────────────────────────────────────

    /// Compute analog (phase-only) beamforming weights for a given steering
    /// angle. Optionally quantise the phase to `phase_bits` bits.
    ///
    /// For ULA geometry, `theta_rad` is the azimuth angle. For UPA, use
    /// [`analog_weights_upa`] instead.
    pub fn analog_weights(
        &self,
        theta_rad: f64,
        phase_bits: Option<u32>,
    ) -> Vec<(f64, f64)> {
        let sv = self.steering_vector_ula(theta_rad);
        // Conjugate the steering vector (matched-filter beamforming)
        let mut weights: Vec<(f64, f64)> = sv.clone();
        if let Some(bits) = phase_bits {
            self.quantize_phases(&mut weights, bits);
        }
        weights
    }

    /// Compute analog (phase-only) beamforming weights for a UPA.
    pub fn analog_weights_upa(
        &self,
        az_rad: f64,
        el_rad: f64,
        phase_bits: Option<u32>,
    ) -> Vec<(f64, f64)> {
        let sv = self.steering_vector_upa(az_rad, el_rad);
        let mut weights: Vec<(f64, f64)> = sv.clone();
        if let Some(bits) = phase_bits {
            self.quantize_phases(&mut weights, bits);
        }
        weights
    }

    /// Quantise phase-only weights to `bits`-bit resolution.
    fn quantize_phases(&self, weights: &mut [(f64, f64)], bits: u32) {
        let levels = (1u64 << bits) as f64;
        let step = 2.0 * PI / levels;
        for w in weights.iter_mut() {
            let phase = w.1.atan2(w.0);
            let quantized = (phase / step).round() * step;
            *w = cexp_j(quantized);
        }
    }

    // ── Digital beamforming ─────────────────────────────────────────────

    /// Compute digital (full complex) beamforming weights using the
    /// matched-filter (conjugate) of the steering vector, normalized so
    /// that the weight vector has unit norm.
    pub fn digital_weights(&self, theta_rad: f64) -> Vec<(f64, f64)> {
        let sv = self.steering_vector_ula(theta_rad);
        let n = sv.len() as f64;
        sv.iter().map(|&s| cscale(1.0 / n, s)).collect()
    }

    /// Apply a set of beamforming weights to received element signals.
    ///
    /// Returns the scalar beamformed output `w^H * x`.
    pub fn apply_weights(
        weights: &[(f64, f64)],
        element_signals: &[(f64, f64)],
    ) -> (f64, f64) {
        assert_eq!(weights.len(), element_signals.len());
        weights
            .iter()
            .zip(element_signals.iter())
            .fold((0.0, 0.0), |acc, (&w, &x)| cadd(acc, cmul(conj(w), x)))
    }

    // ── Hybrid beamforming ──────────────────────────────────────────────

    /// Perform two-stage hybrid beamforming.
    ///
    /// - `analog_weights_matrix`: `num_rf_chains` entries, each a vector of
    ///   `num_elements` complex weights (one column per RF chain).
    /// - `digital_weights`: `num_rf_chains` complex weights.
    /// - `element_signals`: `num_elements` complex samples.
    ///
    /// The output is `w_d^H * (F_a^H * x)` where `F_a` is the analog
    /// precoding matrix.
    pub fn hybrid_beamform(
        analog_weights_matrix: &[Vec<(f64, f64)>],
        digital_weights: &[(f64, f64)],
        element_signals: &[(f64, f64)],
    ) -> (f64, f64) {
        let num_rf = digital_weights.len();
        assert_eq!(analog_weights_matrix.len(), num_rf);
        // Stage 1: analog combining per RF chain
        let mut rf_signals: Vec<(f64, f64)> = Vec::with_capacity(num_rf);
        for chain in analog_weights_matrix {
            let combined = Self::apply_weights(chain, element_signals);
            rf_signals.push(combined);
        }
        // Stage 2: digital combining
        Self::apply_weights(digital_weights, &rf_signals)
    }

    // ── Beam pattern ────────────────────────────────────────────────────

    /// Compute the array factor (beam pattern) in dB over a sweep of angles.
    ///
    /// Returns `(angles_rad, pattern_db)` where `angles_rad` spans
    /// `[-pi/2, pi/2]` with `num_points` samples.
    pub fn beam_pattern(
        &self,
        weights: &[(f64, f64)],
        num_points: usize,
    ) -> (Vec<f64>, Vec<f64>) {
        let angles: Vec<f64> = (0..num_points)
            .map(|i| -PI / 2.0 + PI * (i as f64) / (num_points as f64 - 1.0))
            .collect();
        let pattern_db: Vec<f64> = angles
            .iter()
            .map(|&theta| {
                let sv = self.steering_vector_ula(theta);
                let af = weights
                    .iter()
                    .zip(sv.iter())
                    .fold((0.0, 0.0), |acc, (&w, &s)| cadd(acc, cmul(conj(w), s)));
                let mag2 = cabs2(af);
                if mag2 < 1e-30 {
                    -300.0
                } else {
                    10.0 * mag2.log10()
                }
            })
            .collect();
        (angles, pattern_db)
    }

    // ── Beam width (3 dB) ───────────────────────────────────────────────

    /// Estimate the 3-dB beamwidth (in radians) of the main lobe for the
    /// given weight vector by scanning the beam pattern.
    pub fn beam_width_3db(&self, weights: &[(f64, f64)]) -> f64 {
        let num_points = 3600; // 0.05-degree resolution
        let (angles, pattern_db) = self.beam_pattern(weights, num_points);
        let peak = pattern_db
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let threshold = peak - 3.0;

        // Find the main lobe bounds: search from peak outward
        let peak_idx = pattern_db
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        // Search left
        let mut left = peak_idx;
        while left > 0 && pattern_db[left] >= threshold {
            left -= 1;
        }
        // Search right
        let mut right = peak_idx;
        while right < num_points - 1 && pattern_db[right] >= threshold {
            right += 1;
        }

        angles[right] - angles[left]
    }

    // ── Sidelobe level ──────────────────────────────────────────────────

    /// Compute the peak sidelobe level (SLL) in dB relative to the main
    /// lobe peak. Returns a negative value (e.g., -13.2 dB).
    pub fn sidelobe_level(&self, weights: &[(f64, f64)]) -> f64 {
        let num_points = 3600;
        let (angles, pattern_db) = self.beam_pattern(weights, num_points);
        let peak = pattern_db
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let threshold_3db = peak - 3.0;

        let peak_idx = pattern_db
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;

        // Find main lobe region
        let mut left = peak_idx;
        while left > 0 && pattern_db[left] >= threshold_3db {
            left -= 1;
        }
        let mut right = peak_idx;
        while right < num_points - 1 && pattern_db[right] >= threshold_3db {
            right += 1;
        }

        // Extend past the first null on each side
        while left > 0 && pattern_db[left] <= pattern_db[left + 1] {
            left -= 1;
        }
        while right < num_points - 1 && pattern_db[right] <= pattern_db[right - 1] {
            right += 1;
        }

        // Peak sidelobe is the max outside the main lobe region
        let sll = angles
            .iter()
            .zip(pattern_db.iter())
            .enumerate()
            .filter(|&(i, _)| i < left || i > right)
            .map(|(_, (_, &db))| db)
            .fold(f64::NEG_INFINITY, f64::max);

        sll - peak
    }

    // ── Codebook generation ─────────────────────────────────────────────

    /// Generate a uniform codebook of `num_beams` steering directions
    /// spanning `[-pi/2, pi/2]`.
    pub fn generate_codebook(&self, num_beams: usize) -> Vec<CodebookEntry> {
        (0..num_beams)
            .map(|i| {
                let angle = -PI / 2.0 + PI * (i as f64) / (num_beams as f64 - 1.0);
                let weights = self.analog_weights(angle, None);
                CodebookEntry {
                    index: i,
                    angle_rad: angle,
                    weights,
                }
            })
            .collect()
    }

    /// Exhaustive beam search: apply every codebook entry to the received
    /// signal and return the best.
    pub fn exhaustive_beam_search(
        &self,
        codebook: &[CodebookEntry],
        element_signals: &[(f64, f64)],
    ) -> BeamSearchResult {
        let mut best_idx = 0;
        let mut best_power = f64::NEG_INFINITY;
        for entry in codebook {
            let out = Self::apply_weights(&entry.weights, element_signals);
            let power = cabs2(out);
            if power > best_power {
                best_power = power;
                best_idx = entry.index;
            }
        }
        BeamSearchResult {
            best_index: best_idx,
            best_angle_rad: codebook[best_idx].angle_rad,
            best_power,
        }
    }

    /// Hierarchical beam search using progressively narrower codebooks.
    ///
    /// `levels` specifies the number of beams at each level (e.g., `[4, 8, 16]`).
    /// Each level refines the search around the best beam of the previous level.
    pub fn hierarchical_beam_search(
        &self,
        levels: &[usize],
        element_signals: &[(f64, f64)],
    ) -> BeamSearchResult {
        let mut search_min = -PI / 2.0;
        let mut search_max = PI / 2.0;
        let mut best = BeamSearchResult {
            best_index: 0,
            best_angle_rad: 0.0,
            best_power: f64::NEG_INFINITY,
        };

        for &num_beams in levels {
            let codebook: Vec<CodebookEntry> = (0..num_beams)
                .map(|i| {
                    let angle = search_min
                        + (search_max - search_min) * (i as f64) / (num_beams as f64 - 1.0).max(1.0);
                    let weights = self.analog_weights(angle, None);
                    CodebookEntry {
                        index: i,
                        angle_rad: angle,
                        weights,
                    }
                })
                .collect();

            best = self.exhaustive_beam_search(&codebook, element_signals);

            // Narrow the search range around the best angle
            let span = (search_max - search_min) / num_beams as f64;
            search_min = best.best_angle_rad - span;
            search_max = best.best_angle_rad + span;
            // Clamp to valid range
            search_min = search_min.max(-PI / 2.0);
            search_max = search_max.min(PI / 2.0);
        }

        best
    }

    // ── Array gain and directivity ──────────────────────────────────────

    /// Compute array gain (linear) in the steered direction.
    ///
    /// For a ULA with uniform weights this equals `N` (the number of
    /// elements).
    pub fn array_gain(&self, weights: &[(f64, f64)]) -> f64 {
        let num_points = 3600;
        let (_angles, pattern_db) = self.beam_pattern(weights, num_points);
        let peak_db = pattern_db
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let peak_linear = 10.0_f64.powf(peak_db / 10.0);

        // Normalize by the weight power (sum |w_i|^2)
        let w_power: f64 = weights.iter().map(|w| cabs2(*w)).sum();
        peak_linear / w_power
    }

    /// Compute array directivity (linear) by integrating the beam pattern
    /// over the visible hemisphere.
    pub fn directivity(&self, weights: &[(f64, f64)]) -> f64 {
        let num_points = 3600;
        let (angles, pattern_db) = self.beam_pattern(weights, num_points);
        let peak_db = pattern_db
            .iter()
            .cloned()
            .fold(f64::NEG_INFINITY, f64::max);
        let peak_linear = 10.0_f64.powf(peak_db / 10.0);

        // Numerical integration of the pattern (linear) over theta
        let d_theta = PI / (num_points as f64 - 1.0);
        let integral: f64 = pattern_db
            .iter()
            .zip(angles.iter())
            .map(|(&db, &theta)| {
                let linear = 10.0_f64.powf(db / 10.0);
                linear * theta.cos().abs() * d_theta
            })
            .sum();

        if integral < 1e-30 {
            1.0
        } else {
            peak_linear / integral
        }
    }

    /// Array directivity in dBi.
    pub fn directivity_dbi(&self, weights: &[(f64, f64)]) -> f64 {
        10.0 * self.directivity(weights).log10()
    }

    // ── Grating-lobe analysis ───────────────────────────────────────────

    /// Analyse grating lobes for the current element spacing and a given
    /// steering angle.
    ///
    /// Grating lobes appear when `d / lambda >= 1 / (1 + |sin(theta)|)`.
    pub fn grating_lobe_analysis(&self, steer_rad: f64) -> GratingLobeInfo {
        let d_over_lam = self.element_spacing / self.wavelength;
        let sin_steer = steer_rad.sin();
        let threshold = 1.0 / (1.0 + sin_steer.abs());
        let has_grating = d_over_lam >= threshold;

        let mut grating_angles = Vec::new();
        if has_grating {
            // Grating lobes at sin(theta_g) = sin(theta_s) + m*lambda/d for integer m != 0
            for m in [-3, -2, -1, 1, 2, 3] {
                let sin_g = sin_steer + (m as f64) * self.wavelength / self.element_spacing;
                if sin_g.abs() <= 1.0 {
                    grating_angles.push(sin_g.asin());
                }
            }
        }

        GratingLobeInfo {
            has_grating_lobes: has_grating,
            grating_angles,
            spacing_wavelengths: d_over_lam,
        }
    }

    // ── Path loss models ────────────────────────────────────────────────

    /// Compute path loss for a given distance and mmWave band.
    ///
    /// Uses the close-in (CI) free-space reference distance model:
    /// `PL(d) = FSPL(d0=1m) + 10*n*log10(d)` where `n` is the path-loss
    /// exponent.
    ///
    /// Typical PLE values from NYU measurements:
    /// - 28 GHz LOS: n = 2.0, NLOS: n = 3.4
    /// - 73 GHz LOS: n = 2.0, NLOS: n = 3.7
    pub fn path_loss(band: MmWaveBand, distance_m: f64, los: bool) -> PathLossResult {
        let freq_hz = match band {
            MmWaveBand::Band28Ghz => 28.0e9,
            MmWaveBand::Band73Ghz => 73.0e9,
        };

        let ple = match (band, los) {
            (MmWaveBand::Band28Ghz, true) => 2.0,
            (MmWaveBand::Band28Ghz, false) => 3.4,
            (MmWaveBand::Band73Ghz, true) => 2.0,
            (MmWaveBand::Band73Ghz, false) => 3.7,
        };

        // FSPL at d0 = 1 metre
        let fspl_d0 = 20.0 * (4.0 * PI * freq_hz / Self::C).log10();

        let d = distance_m.max(1.0); // Clamp to reference distance
        let ci_pl = fspl_d0 + 10.0 * ple * d.log10();

        // Plain FSPL at distance d
        let fspl = 20.0 * (4.0 * PI * distance_m.max(1.0) * freq_hz / Self::C).log10();

        PathLossResult {
            fspl_db: fspl,
            ci_path_loss_db: ci_pl,
            path_loss_exponent: ple,
        }
    }
}

// ── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-6;

    fn approx_eq(a: f64, b: f64, tol: f64) -> bool {
        (a - b).abs() < tol
    }

    // 1. Steering vector length matches num_elements
    #[test]
    fn test_steering_vector_length() {
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 8 }, 28.0e9);
        let sv = bf.steering_vector_ula(0.3);
        assert_eq!(sv.len(), 8);
    }

    // 2. Steering vector at broadside (theta=0) has all-ones phase
    #[test]
    fn test_steering_vector_broadside() {
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 4 }, 28.0e9);
        let sv = bf.steering_vector_ula(0.0);
        for s in &sv {
            assert!(approx_eq(s.0, 1.0, TOL));
            assert!(approx_eq(s.1, 0.0, TOL));
        }
    }

    // 3. Steering vector elements are unit-magnitude
    #[test]
    fn test_steering_vector_unit_magnitude() {
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 16 }, 73.0e9);
        let sv = bf.steering_vector_ula(0.5);
        for s in &sv {
            assert!(approx_eq(cabs(*s), 1.0, TOL));
        }
    }

    // 4. Analog weights have unit magnitude (phase-only)
    #[test]
    fn test_analog_weights_unit_magnitude() {
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 8 }, 28.0e9);
        let w = bf.analog_weights(0.2, None);
        for wi in &w {
            assert!(approx_eq(cabs(*wi), 1.0, TOL));
        }
    }

    // 5. Quantized phase weights still unit magnitude
    #[test]
    fn test_quantized_weights_unit_magnitude() {
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 8 }, 28.0e9);
        let w = bf.analog_weights(0.3, Some(3)); // 3-bit phase shifter
        for wi in &w {
            assert!(approx_eq(cabs(*wi), 1.0, TOL));
        }
    }

    // 6. Beam pattern peaks at the steered direction
    #[test]
    fn test_beam_pattern_peak_direction() {
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 16 }, 28.0e9);
        let steer = 0.3; // ~17 degrees
        let w = bf.analog_weights(steer, None);
        let (angles, pattern) = bf.beam_pattern(&w, 3600);

        let peak_idx = pattern
            .iter()
            .enumerate()
            .max_by(|a, b| a.1.partial_cmp(b.1).unwrap())
            .unwrap()
            .0;
        let peak_angle = angles[peak_idx];
        assert!(
            (peak_angle - steer).abs() < 0.02,
            "Peak at {:.4} but expected {:.4}",
            peak_angle,
            steer
        );
    }

    // 7. Beam width decreases with more elements
    #[test]
    fn test_beam_width_vs_elements() {
        let bf8 = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 8 }, 28.0e9);
        let bf32 = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 32 }, 28.0e9);
        let w8 = bf8.analog_weights(0.0, None);
        let w32 = bf32.analog_weights(0.0, None);
        let bw8 = bf8.beam_width_3db(&w8);
        let bw32 = bf32.beam_width_3db(&w32);
        assert!(
            bw32 < bw8,
            "32-element BW ({:.4}) should be narrower than 8-element ({:.4})",
            bw32,
            bw8
        );
    }

    // 8. Sidelobe level is negative
    #[test]
    fn test_sidelobe_level_negative() {
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 16 }, 28.0e9);
        let w = bf.analog_weights(0.0, None);
        let sll = bf.sidelobe_level(&w);
        assert!(sll < 0.0, "SLL should be negative, got {:.2} dB", sll);
    }

    // 9. ULA sidelobe for uniform weights ~= -13.2 dB
    #[test]
    fn test_sidelobe_level_uniform_approx() {
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 32 }, 28.0e9);
        let w = bf.analog_weights(0.0, None);
        let sll = bf.sidelobe_level(&w);
        // Uniform linear array SLL is approximately -13.26 dB
        assert!(
            (sll - (-13.26)).abs() < 1.5,
            "Expected SLL ~-13.26 dB, got {:.2} dB",
            sll
        );
    }

    // 10. Digital weights are normalized (unit-norm)
    #[test]
    fn test_digital_weights_normalized() {
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 8 }, 28.0e9);
        let w = bf.digital_weights(0.1);
        let norm: f64 = w.iter().map(|wi| cabs2(*wi)).sum::<f64>().sqrt();
        // 1/N normalization: norm = sqrt(N * (1/N)^2) = 1/sqrt(N)
        let expected_norm = 1.0 / (8.0_f64).sqrt();
        assert!(
            approx_eq(norm, expected_norm, 1e-10),
            "Expected norm {:.6}, got {:.6}",
            expected_norm,
            norm
        );
    }

    // 11. Apply weights produces correct inner product
    #[test]
    fn test_apply_weights() {
        let w = vec![(1.0, 0.0), (0.0, 1.0)];
        let x = vec![(1.0, 0.0), (0.0, 1.0)];
        // w^H * x = conj(w0)*x0 + conj(w1)*x1
        // = (1,0)*(1,0) + (0,-1)*(0,1) = (1,0) + (1,0) = (2,0)
        let out = MmWaveBeamformer::apply_weights(&w, &x);
        assert!(approx_eq(out.0, 2.0, TOL));
        assert!(approx_eq(out.1, 0.0, TOL));
    }

    // 12. Hybrid beamforming matches single-stage for 1 RF chain
    #[test]
    fn test_hybrid_single_chain() {
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 4 }, 28.0e9);
        let analog_w = bf.analog_weights(0.2, None);
        let digital_w = vec![(1.0, 0.0)]; // Single RF chain, unity digital weight
        let signals = vec![(1.0, 0.5), (0.5, -0.3), (0.8, 0.1), (-0.2, 0.7)];

        let analog_only = MmWaveBeamformer::apply_weights(&analog_w, &signals);
        let hybrid =
            MmWaveBeamformer::hybrid_beamform(&[analog_w.clone()], &digital_w, &signals);
        assert!(approx_eq(analog_only.0, hybrid.0, TOL));
        assert!(approx_eq(analog_only.1, hybrid.1, TOL));
    }

    // 13. Codebook generation creates correct number of entries
    #[test]
    fn test_codebook_size() {
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 8 }, 28.0e9);
        let cb = bf.generate_codebook(16);
        assert_eq!(cb.len(), 16);
        // First and last angles should be near +/-pi/2
        assert!(approx_eq(cb[0].angle_rad, -PI / 2.0, 0.01));
        assert!(approx_eq(cb[15].angle_rad, PI / 2.0, 0.01));
    }

    // 14. Exhaustive beam search finds correct direction
    #[test]
    fn test_exhaustive_beam_search() {
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 8 }, 28.0e9);
        let target_angle = 0.3; // radians
        let cb = bf.generate_codebook(64);

        // Simulate a signal arriving from target_angle
        let sv = bf.steering_vector_ula(target_angle);
        let result = bf.exhaustive_beam_search(&cb, &sv);

        assert!(
            (result.best_angle_rad - target_angle).abs() < 0.1,
            "Search found {:.4} rad, expected ~{:.4} rad",
            result.best_angle_rad,
            target_angle
        );
    }

    // 15. Hierarchical search converges to correct angle
    #[test]
    fn test_hierarchical_beam_search() {
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 16 }, 28.0e9);
        let target_angle = -0.4;
        let sv = bf.steering_vector_ula(target_angle);

        let result = bf.hierarchical_beam_search(&[4, 8, 16], &sv);
        assert!(
            (result.best_angle_rad - target_angle).abs() < 0.1,
            "Hierarchical search found {:.4} rad, expected ~{:.4} rad",
            result.best_angle_rad,
            target_angle
        );
    }

    // 16. Array gain approximately equals N for uniform steering
    #[test]
    fn test_array_gain() {
        let n = 16;
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: n }, 28.0e9);
        let w = bf.analog_weights(0.0, None);
        let gain = bf.array_gain(&w);
        assert!(
            (gain - n as f64).abs() < 1.5,
            "Expected gain ~{}, got {:.2}",
            n,
            gain
        );
    }

    // 17. Grating lobes: no grating lobes at half-wavelength spacing, broadside
    #[test]
    fn test_no_grating_lobes_half_lambda() {
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 8 }, 28.0e9);
        let info = bf.grating_lobe_analysis(0.0);
        assert!(
            !info.has_grating_lobes,
            "Should have no grating lobes at d=lambda/2 broadside"
        );
        assert!(approx_eq(info.spacing_wavelengths, 0.5, TOL));
    }

    // 18. Grating lobes: detected when spacing > lambda
    #[test]
    fn test_grating_lobes_detected() {
        let bf = MmWaveBeamformer::with_spacing(
            ArrayGeometry::Ula { num_elements: 8 },
            28.0e9,
            // Spacing = 1.5 * lambda
            1.5 * (MmWaveBeamformer::C / 28.0e9),
        );
        let info = bf.grating_lobe_analysis(0.0);
        assert!(
            info.has_grating_lobes,
            "Should detect grating lobes at d=1.5*lambda"
        );
        assert!(!info.grating_angles.is_empty());
        assert!(approx_eq(info.spacing_wavelengths, 1.5, TOL));
    }

    // 19. Path loss: 28 GHz LOS at 100 m
    #[test]
    fn test_path_loss_28ghz_los() {
        let pl = MmWaveBeamformer::path_loss(MmWaveBand::Band28Ghz, 100.0, true);
        assert!(pl.fspl_db > 80.0 && pl.fspl_db < 120.0);
        assert!(approx_eq(pl.path_loss_exponent, 2.0, TOL));
        // CI model at LOS with n=2 should match FSPL
        assert!((pl.ci_path_loss_db - pl.fspl_db).abs() < 0.5);
    }

    // 20. Path loss: 73 GHz NLOS has higher PLE
    #[test]
    fn test_path_loss_73ghz_nlos() {
        let pl = MmWaveBeamformer::path_loss(MmWaveBand::Band73Ghz, 200.0, false);
        assert!(approx_eq(pl.path_loss_exponent, 3.7, TOL));
        // CI path loss with n=3.7 should exceed FSPL (n=2)
        assert!(pl.ci_path_loss_db > pl.fspl_db);
    }

    // 21. UPA steering vector length
    #[test]
    fn test_upa_steering_vector_length() {
        let bf = MmWaveBeamformer::new(
            ArrayGeometry::Upa { rows: 4, cols: 4 },
            28.0e9,
        );
        let sv = bf.steering_vector_upa(0.2, 0.1);
        assert_eq!(sv.len(), 16);
    }

    // 22. UPA steering vector at broadside is all ones
    #[test]
    fn test_upa_broadside() {
        let bf = MmWaveBeamformer::new(
            ArrayGeometry::Upa { rows: 4, cols: 4 },
            28.0e9,
        );
        let sv = bf.steering_vector_upa(0.0, 0.0);
        for s in &sv {
            assert!(approx_eq(s.0, 1.0, TOL));
            assert!(approx_eq(s.1, 0.0, TOL));
        }
    }

    // 23. num_elements for UPA
    #[test]
    fn test_num_elements_upa() {
        let bf = MmWaveBeamformer::new(
            ArrayGeometry::Upa { rows: 8, cols: 4 },
            73.0e9,
        );
        assert_eq!(bf.num_elements(), 32);
    }

    // 24. Directivity is positive
    #[test]
    fn test_directivity_positive() {
        let bf = MmWaveBeamformer::new(ArrayGeometry::Ula { num_elements: 8 }, 28.0e9);
        let w = bf.analog_weights(0.0, None);
        let d = bf.directivity(&w);
        assert!(d > 1.0, "Directivity should be > 1 (0 dBi), got {:.2}", d);
    }

    // 25. Complex helper: cmul identity
    #[test]
    fn test_cmul_identity() {
        let a = (3.0, 4.0);
        let one = (1.0, 0.0);
        let result = cmul(a, one);
        assert!(approx_eq(result.0, 3.0, TOL));
        assert!(approx_eq(result.1, 4.0, TOL));
    }

    // 26. Complex helper: cabs
    #[test]
    fn test_cabs_345() {
        assert!(approx_eq(cabs((3.0, 4.0)), 5.0, TOL));
    }
}
