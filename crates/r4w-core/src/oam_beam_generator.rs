//! # OAM Beam Generator
//!
//! Generate orbital angular momentum (OAM) beams for spatial multiplexing
//! and vortex radio using a Uniform Circular Array (UCA).
//!
//! OAM beams carry a helical phase front described by exp(jℓφ), where ℓ is the
//! topological charge (mode number) and φ is the azimuthal angle. Different OAM
//! modes are orthogonal and can be used to multiplex independent data streams
//! on the same frequency, increasing spectral efficiency.
//!
//! ## Overview
//!
//! A UCA with N elements can generate OAM modes ℓ in the range
//! `[-(N-1)/2, N/2]`. Each mode applies a progressive phase shift
//! `φ_n = ℓ · 2π·n/N` to the n-th element. Mode orthogonality is guaranteed
//! by the discrete Fourier transform relationship:
//!
//! ```text
//! Σ_{n=0}^{N-1} exp(j·(ℓ₁ - ℓ₂)·2π·n/N) = N · δ(ℓ₁ - ℓ₂ mod N)
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::oam_beam_generator::{OamBeamGenerator, OamConfig};
//!
//! // Create an 8-element UCA with radius = 0.5 wavelengths
//! let config = OamConfig {
//!     num_elements: 8,
//!     radius_wavelengths: 0.5,
//!     max_mode: 3,
//! };
//! let gen = OamBeamGenerator::new(config);
//!
//! // Generate spiral phase plate weights for mode ℓ = 1
//! let weights = gen.spiral_phase_plate(1);
//! assert_eq!(weights.len(), 8);
//!
//! // Generate per-element signals for a simple input
//! let signal = vec![(1.0, 0.0); 4];
//! let per_element = gen.generate_mode(1, &signal);
//! assert_eq!(per_element.len(), 8); // one signal per element
//! assert_eq!(per_element[0].len(), 4); // same length as input
//!
//! // Verify mode orthogonality
//! let orth = gen.mode_orthogonality(1, 2);
//! assert!(orth.abs() < 1e-10, "Distinct OAM modes should be orthogonal");
//!
//! let self_corr = gen.mode_orthogonality(1, 1);
//! assert!((self_corr - 1.0).abs() < 1e-10, "Same mode should have correlation 1.0");
//! ```

use std::f64::consts::PI;

// --------------------------------------------------------------------------
// Complex arithmetic helpers (using (f64, f64) tuples)
// --------------------------------------------------------------------------

/// Multiply two complex numbers represented as (re, im) tuples.
#[inline]
fn cmul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

/// Add two complex numbers.
#[inline]
fn cadd(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 + b.0, a.1 + b.1)
}

/// Complex conjugate.
#[inline]
fn conj(a: (f64, f64)) -> (f64, f64) {
    (a.0, -a.1)
}

/// Magnitude squared of a complex number.
#[inline]
fn cmag2(a: (f64, f64)) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

/// Scale a complex number by a real scalar.
#[inline]
fn cscale(a: (f64, f64), s: f64) -> (f64, f64) {
    (a.0 * s, a.1 * s)
}

/// Create a complex exponential: exp(jθ) = (cos θ, sin θ).
#[inline]
fn cexp_j(theta: f64) -> (f64, f64) {
    (theta.cos(), theta.sin())
}

// --------------------------------------------------------------------------
// Configuration
// --------------------------------------------------------------------------

/// Configuration for the OAM beam generator.
#[derive(Debug, Clone)]
pub struct OamConfig {
    /// Number of elements in the Uniform Circular Array.
    pub num_elements: usize,
    /// Radius of the UCA in wavelengths.
    pub radius_wavelengths: f64,
    /// Maximum absolute OAM mode number to support.
    /// Modes from `-max_mode` to `+max_mode` will be available.
    pub max_mode: i32,
}

// --------------------------------------------------------------------------
// OAM Mode
// --------------------------------------------------------------------------

/// Describes a single OAM mode with its topological charge and per-element weights.
#[derive(Debug, Clone)]
pub struct OamMode {
    /// Topological charge (integer ℓ). Positive = left-handed helix,
    /// negative = right-handed helix, zero = plane wave.
    pub topological_charge: i32,
    /// Per-element complex weights for exciting this mode on the UCA.
    /// Length equals `num_elements`.
    pub element_weights: Vec<(f64, f64)>,
}

// --------------------------------------------------------------------------
// OAM Beam Generator
// --------------------------------------------------------------------------

/// Main OAM beam generation engine using a Uniform Circular Array (UCA).
///
/// Generates spiral phase plate weights, produces per-element signals for
/// individual OAM modes, and supports mode orthogonality analysis.
#[derive(Debug, Clone)]
pub struct OamBeamGenerator {
    config: OamConfig,
}

impl OamBeamGenerator {
    /// Create a new OAM beam generator with the given configuration.
    ///
    /// # Panics
    ///
    /// Panics if `num_elements` is zero or `max_mode` is negative.
    pub fn new(config: OamConfig) -> Self {
        assert!(config.num_elements > 0, "UCA must have at least 1 element");
        assert!(config.max_mode >= 0, "max_mode must be non-negative");
        Self { config }
    }

    /// Return the spiral phase plate weights for the given OAM mode ℓ.
    ///
    /// For a UCA with N elements, element n receives phase weight:
    ///
    /// ```text
    /// w_n = (1/√N) · exp(j · ℓ · 2π · n / N)
    /// ```
    ///
    /// The `1/√N` normalization ensures unit total power.
    pub fn spiral_phase_plate(&self, mode: i32) -> Vec<(f64, f64)> {
        let n = self.config.num_elements;
        let norm = 1.0 / (n as f64).sqrt();
        (0..n)
            .map(|i| {
                let phase = (mode as f64) * 2.0 * PI * (i as f64) / (n as f64);
                cscale(cexp_j(phase), norm)
            })
            .collect()
    }

    /// Generate per-element signals for a single OAM mode.
    ///
    /// Each element's output is the input `signal` multiplied by that element's
    /// spiral phase plate weight. Returns a `Vec` of length `num_elements`,
    /// where each inner `Vec` has the same length as `signal`.
    pub fn generate_mode(&self, mode: i32, signal: &[(f64, f64)]) -> Vec<Vec<(f64, f64)>> {
        let weights = self.spiral_phase_plate(mode);
        weights
            .iter()
            .map(|&w| signal.iter().map(|&s| cmul(w, s)).collect())
            .collect()
    }

    /// Multiplex multiple data streams onto orthogonal OAM modes.
    ///
    /// `streams` is a slice of signal references, one per mode. The modes used
    /// are `0, 1, 2, ...` up to `streams.len() - 1`. All streams must have the
    /// same length.
    ///
    /// Returns per-element composite signals (length = `num_elements`).
    ///
    /// # Panics
    ///
    /// Panics if streams have different lengths or if there are more streams
    /// than the array can support.
    pub fn multiplex(&self, streams: &[&[(f64, f64)]]) -> Vec<Vec<(f64, f64)>> {
        assert!(!streams.is_empty(), "Need at least one stream to multiplex");
        let sig_len = streams[0].len();
        for (i, s) in streams.iter().enumerate() {
            assert_eq!(
                s.len(),
                sig_len,
                "Stream {} has length {} but expected {}",
                i,
                s.len(),
                sig_len
            );
        }
        assert!(
            streams.len() <= self.config.num_elements,
            "Cannot multiplex {} streams on {} elements",
            streams.len(),
            self.config.num_elements
        );

        let n = self.config.num_elements;
        // Initialize per-element output buffers
        let mut output: Vec<Vec<(f64, f64)>> = vec![vec![(0.0, 0.0); sig_len]; n];

        // Assign modes 0, 1, 2, ... to successive streams
        for (mode_idx, stream) in streams.iter().enumerate() {
            let weights = self.spiral_phase_plate(mode_idx as i32);
            for (elem, &w) in weights.iter().enumerate() {
                for (t, &s) in stream.iter().enumerate() {
                    output[elem][t] = cadd(output[elem][t], cmul(w, s));
                }
            }
        }

        output
    }

    /// Demultiplex per-element received signals into individual OAM mode streams.
    ///
    /// Performs spatial DFT by correlating each element's signal with the
    /// conjugate mode weights. Extracts modes `0, 1, 2, ...` up to
    /// `num_modes - 1` where `num_modes = num_elements`.
    ///
    /// `element_signals` must have length `num_elements`, and all inner signals
    /// must have the same length.
    ///
    /// Returns one stream per mode (length = `num_elements`).
    pub fn demultiplex(&self, element_signals: &[&[(f64, f64)]]) -> Vec<Vec<(f64, f64)>> {
        let n = self.config.num_elements;
        assert_eq!(
            element_signals.len(),
            n,
            "Expected {} element signals, got {}",
            n,
            element_signals.len()
        );

        let sig_len = element_signals[0].len();
        for (i, s) in element_signals.iter().enumerate() {
            assert_eq!(
                s.len(),
                sig_len,
                "Element signal {} has length {} but expected {}",
                i,
                s.len(),
                sig_len
            );
        }

        let mut modes: Vec<Vec<(f64, f64)>> = vec![vec![(0.0, 0.0); sig_len]; n];

        for mode in 0..n {
            // Conjugate weights for this mode (matched filter / spatial DFT)
            let weights = self.spiral_phase_plate(mode as i32);
            let conj_weights: Vec<(f64, f64)> = weights.iter().map(|&w| conj(w)).collect();

            for (elem, &cw) in conj_weights.iter().enumerate() {
                for (t, &s) in element_signals[elem].iter().enumerate() {
                    modes[mode][t] = cadd(modes[mode][t], cmul(cw, s));
                }
            }
        }

        modes
    }

    /// Return the maximum OAM mode number supported by this configuration.
    ///
    /// For an N-element UCA, modes `[-(N-1)/2, N/2]` are unambiguous.
    /// This returns `min(max_mode, N/2)`.
    pub fn max_supported_mode(&self) -> i32 {
        let nyquist_limit = (self.config.num_elements as i32) / 2;
        self.config.max_mode.min(nyquist_limit)
    }

    /// Compute the normalized orthogonality between two OAM modes.
    ///
    /// Returns the magnitude of the inner product of the two mode weight
    /// vectors, normalized to `[0, 1]`. A value near 0 indicates orthogonal
    /// modes; 1 indicates identical modes.
    ///
    /// Mathematically:
    ///
    /// ```text
    /// ρ = |Σ_{n=0}^{N-1} w_a[n]* · w_b[n]|
    ///   = |(1/N) · Σ exp(j·(ℓ_b - ℓ_a)·2π·n/N)|
    /// ```
    ///
    /// which equals 1 when `(ℓ_b - ℓ_a) mod N == 0`, and 0 otherwise.
    pub fn mode_orthogonality(&self, mode_a: i32, mode_b: i32) -> f64 {
        let wa = self.spiral_phase_plate(mode_a);
        let wb = self.spiral_phase_plate(mode_b);

        let mut sum = (0.0, 0.0);
        for (&a, &b) in wa.iter().zip(wb.iter()) {
            sum = cadd(sum, cmul(conj(a), b));
        }

        cmag2(sum).sqrt()
    }

    /// Return the number of array elements.
    pub fn num_elements(&self) -> usize {
        self.config.num_elements
    }

    /// Return the UCA radius in wavelengths.
    pub fn radius_wavelengths(&self) -> f64 {
        self.config.radius_wavelengths
    }

    /// Build an `OamMode` struct for the given topological charge.
    pub fn build_mode(&self, topological_charge: i32) -> OamMode {
        OamMode {
            topological_charge,
            element_weights: self.spiral_phase_plate(topological_charge),
        }
    }
}

// --------------------------------------------------------------------------
// OAM Multiplexer
// --------------------------------------------------------------------------

/// High-level multiplexer / demultiplexer for OAM spatial multiplexing.
///
/// Wraps `OamBeamGenerator` and assigns a fixed set of OAM modes to
/// independent data streams.
#[derive(Debug, Clone)]
pub struct OamMultiplexer {
    generator: OamBeamGenerator,
    /// OAM modes assigned to each stream (in order).
    modes: Vec<i32>,
}

impl OamMultiplexer {
    /// Create a new multiplexer using the given modes.
    ///
    /// # Panics
    ///
    /// Panics if more modes are requested than the array supports.
    pub fn new(config: OamConfig, modes: Vec<i32>) -> Self {
        let generator = OamBeamGenerator::new(config);
        assert!(
            modes.len() <= generator.num_elements(),
            "Cannot use {} modes on {} elements",
            modes.len(),
            generator.num_elements()
        );
        Self { generator, modes }
    }

    /// Multiplex multiple data streams onto the configured OAM modes.
    ///
    /// `streams[i]` is transmitted on `self.modes[i]`. All streams must have
    /// the same length. Returns per-element composite signals.
    pub fn multiplex(&self, streams: &[&[(f64, f64)]]) -> Vec<Vec<(f64, f64)>> {
        assert_eq!(
            streams.len(),
            self.modes.len(),
            "Expected {} streams for {} modes",
            self.modes.len(),
            streams.len()
        );
        assert!(!streams.is_empty(), "Need at least one stream");

        let sig_len = streams[0].len();
        for (i, s) in streams.iter().enumerate() {
            assert_eq!(s.len(), sig_len, "Stream {} length mismatch", i);
        }

        let n = self.generator.num_elements();
        let mut output: Vec<Vec<(f64, f64)>> = vec![vec![(0.0, 0.0); sig_len]; n];

        for (stream, &mode) in streams.iter().zip(self.modes.iter()) {
            let weights = self.generator.spiral_phase_plate(mode);
            for (elem, &w) in weights.iter().enumerate() {
                for (t, &s) in stream.iter().enumerate() {
                    output[elem][t] = cadd(output[elem][t], cmul(w, s));
                }
            }
        }

        output
    }

    /// Demultiplex per-element signals into individual mode streams.
    ///
    /// Returns one stream per configured mode, in the same order as
    /// `self.modes`.
    pub fn demultiplex(&self, element_signals: &[&[(f64, f64)]]) -> Vec<Vec<(f64, f64)>> {
        let n = self.generator.num_elements();
        assert_eq!(
            element_signals.len(),
            n,
            "Expected {} element signals",
            n
        );

        let sig_len = element_signals[0].len();
        let mut result: Vec<Vec<(f64, f64)>> = Vec::with_capacity(self.modes.len());

        for &mode in &self.modes {
            let weights = self.generator.spiral_phase_plate(mode);
            let conj_weights: Vec<(f64, f64)> = weights.iter().map(|&w| conj(w)).collect();

            let mut stream = vec![(0.0, 0.0); sig_len];
            for (elem, &cw) in conj_weights.iter().enumerate() {
                for (t, &s) in element_signals[elem].iter().enumerate() {
                    stream[t] = cadd(stream[t], cmul(cw, s));
                }
            }
            result.push(stream);
        }

        result
    }

    /// Return the configured modes.
    pub fn modes(&self) -> &[i32] {
        &self.modes
    }

    /// Return a reference to the underlying generator.
    pub fn generator(&self) -> &OamBeamGenerator {
        &self.generator
    }
}

// ==========================================================================
// Tests
// ==========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-10;

    fn default_config() -> OamConfig {
        OamConfig {
            num_elements: 8,
            radius_wavelengths: 0.5,
            max_mode: 3,
        }
    }

    #[test]
    fn test_spiral_phase_plate_mode_zero() {
        let gen = OamBeamGenerator::new(default_config());
        let weights = gen.spiral_phase_plate(0);
        // Mode 0 should have uniform phase (plane wave)
        let norm = 1.0 / (8.0_f64).sqrt();
        for w in &weights {
            assert!((w.0 - norm).abs() < EPS, "Real part should be 1/sqrt(N)");
            assert!(w.1.abs() < EPS, "Imag part should be ~0 for mode 0");
        }
    }

    #[test]
    fn test_spiral_phase_plate_length() {
        let gen = OamBeamGenerator::new(default_config());
        for mode in -3..=3 {
            let weights = gen.spiral_phase_plate(mode);
            assert_eq!(weights.len(), 8);
        }
    }

    #[test]
    fn test_spiral_phase_plate_unit_power() {
        let gen = OamBeamGenerator::new(default_config());
        for mode in -3..=3 {
            let weights = gen.spiral_phase_plate(mode);
            let total_power: f64 = weights.iter().map(|w| cmag2(*w)).sum();
            assert!(
                (total_power - 1.0).abs() < EPS,
                "Total power for mode {} should be 1.0, got {}",
                mode,
                total_power
            );
        }
    }

    #[test]
    fn test_mode_orthogonality_same_mode() {
        let gen = OamBeamGenerator::new(default_config());
        for mode in -3..=3 {
            let orth = gen.mode_orthogonality(mode, mode);
            assert!(
                (orth - 1.0).abs() < EPS,
                "Self-correlation for mode {} should be 1.0, got {}",
                mode,
                orth
            );
        }
    }

    #[test]
    fn test_mode_orthogonality_different_modes() {
        let gen = OamBeamGenerator::new(default_config());
        // For an 8-element array, modes that differ by a non-multiple of 8
        // should be orthogonal.
        for a in 0..4 {
            for b in (a + 1)..4 {
                let orth = gen.mode_orthogonality(a, b);
                assert!(
                    orth.abs() < EPS,
                    "Modes {} and {} should be orthogonal, got {}",
                    a,
                    b,
                    orth
                );
            }
        }
    }

    #[test]
    fn test_mode_orthogonality_aliased_modes() {
        // Modes ℓ and ℓ+N are aliases on an N-element array
        let gen = OamBeamGenerator::new(default_config());
        let orth = gen.mode_orthogonality(1, 9); // 9 = 1 + 8
        assert!(
            (orth - 1.0).abs() < EPS,
            "Mode 1 and 9 should be aliased on 8-element array, got {}",
            orth
        );
    }

    #[test]
    fn test_generate_mode_dimensions() {
        let gen = OamBeamGenerator::new(default_config());
        let signal = vec![(1.0, 0.0); 16];
        let per_elem = gen.generate_mode(2, &signal);
        assert_eq!(per_elem.len(), 8, "Should have 8 element signals");
        for elem_sig in &per_elem {
            assert_eq!(elem_sig.len(), 16, "Each element signal should match input length");
        }
    }

    #[test]
    fn test_generate_mode_zero_is_uniform() {
        let gen = OamBeamGenerator::new(default_config());
        let signal = vec![(1.0, 0.5)];
        let per_elem = gen.generate_mode(0, &signal);
        // Mode 0 weights are all identical, so all elements should get the same output
        let first = per_elem[0][0];
        for elem_sig in &per_elem {
            assert!((elem_sig[0].0 - first.0).abs() < EPS);
            assert!((elem_sig[0].1 - first.1).abs() < EPS);
        }
    }

    #[test]
    fn test_multiplex_single_stream() {
        let gen = OamBeamGenerator::new(default_config());
        let signal = vec![(1.0, 0.0); 4];
        let streams: Vec<&[(f64, f64)]> = vec![&signal];
        let output = gen.multiplex(&streams);
        assert_eq!(output.len(), 8);
        assert_eq!(output[0].len(), 4);
    }

    #[test]
    fn test_roundtrip_multiplex_demultiplex() {
        let gen = OamBeamGenerator::new(default_config());
        let sig_len = 8;

        // Create two distinct test signals
        let stream0: Vec<(f64, f64)> = (0..sig_len).map(|i| ((i as f64) * 0.1, 0.0)).collect();
        let stream1: Vec<(f64, f64)> = (0..sig_len).map(|i| (0.0, (i as f64) * 0.2)).collect();

        let streams: Vec<&[(f64, f64)]> = vec![&stream0, &stream1];
        let muxed = gen.multiplex(&streams);

        // Demultiplex
        let elem_refs: Vec<&[(f64, f64)]> = muxed.iter().map(|v| v.as_slice()).collect();
        let demuxed = gen.demultiplex(&elem_refs);

        // With 1/sqrt(N) normalization: mux applies 1/sqrt(N), demux sums N terms
        // each with 1/sqrt(N) conjugate. Net gain = N * (1/sqrt(N))^2 = 1.
        for t in 0..sig_len {
            assert!(
                (demuxed[0][t].0 - stream0[t].0).abs() < EPS,
                "Mode 0 real mismatch at t={}: got {} expected {}",
                t,
                demuxed[0][t].0,
                stream0[t].0
            );
            assert!(
                (demuxed[0][t].1 - stream0[t].1).abs() < EPS,
                "Mode 0 imag mismatch at t={}",
                t
            );
        }
        for t in 0..sig_len {
            assert!(
                (demuxed[1][t].0 - stream1[t].0).abs() < EPS,
                "Mode 1 real mismatch at t={}",
                t
            );
            assert!(
                (demuxed[1][t].1 - stream1[t].1).abs() < EPS,
                "Mode 1 imag mismatch at t={}: got {} expected {}",
                t,
                demuxed[1][t].1,
                stream1[t].1
            );
        }
    }

    #[test]
    fn test_max_supported_mode() {
        let config = OamConfig {
            num_elements: 8,
            radius_wavelengths: 0.5,
            max_mode: 10, // more than array supports
        };
        let gen = OamBeamGenerator::new(config);
        assert_eq!(gen.max_supported_mode(), 4); // 8/2 = 4

        let config2 = OamConfig {
            num_elements: 8,
            radius_wavelengths: 0.5,
            max_mode: 2, // less than array supports
        };
        let gen2 = OamBeamGenerator::new(config2);
        assert_eq!(gen2.max_supported_mode(), 2);
    }

    #[test]
    fn test_build_mode() {
        let gen = OamBeamGenerator::new(default_config());
        let mode = gen.build_mode(2);
        assert_eq!(mode.topological_charge, 2);
        assert_eq!(mode.element_weights.len(), 8);
        // Should match spiral_phase_plate
        let expected = gen.spiral_phase_plate(2);
        for (a, b) in mode.element_weights.iter().zip(expected.iter()) {
            assert!((a.0 - b.0).abs() < EPS);
            assert!((a.1 - b.1).abs() < EPS);
        }
    }

    #[test]
    fn test_negative_mode_weights() {
        let gen = OamBeamGenerator::new(default_config());
        let pos_weights = gen.spiral_phase_plate(2);
        let neg_weights = gen.spiral_phase_plate(-2);
        // Negative mode should be the conjugate of the positive mode
        for (p, n) in pos_weights.iter().zip(neg_weights.iter()) {
            assert!((p.0 - n.0).abs() < EPS, "Real parts should match");
            assert!((p.1 + n.1).abs() < EPS, "Imag parts should be negated");
        }
    }

    #[test]
    fn test_oam_multiplexer_roundtrip() {
        let config = OamConfig {
            num_elements: 8,
            radius_wavelengths: 0.5,
            max_mode: 3,
        };
        let modes = vec![0, 1, -1, 2];
        let muxer = OamMultiplexer::new(config, modes.clone());

        let sig_len = 16;
        let s0: Vec<(f64, f64)> = (0..sig_len).map(|i| (1.0 + i as f64 * 0.01, 0.0)).collect();
        let s1: Vec<(f64, f64)> = (0..sig_len).map(|_| (0.0, 1.0)).collect();
        let s2: Vec<(f64, f64)> = (0..sig_len).map(|i| (-(i as f64) * 0.05, 0.3)).collect();
        let s3: Vec<(f64, f64)> = (0..sig_len).map(|i| (0.5, (i as f64) * 0.02)).collect();

        let streams: Vec<&[(f64, f64)]> = vec![&s0, &s1, &s2, &s3];
        let muxed = muxer.multiplex(&streams);

        let elem_refs: Vec<&[(f64, f64)]> = muxed.iter().map(|v| v.as_slice()).collect();
        let demuxed = muxer.demultiplex(&elem_refs);

        assert_eq!(demuxed.len(), 4);
        // Check stream 0 recovery
        for t in 0..sig_len {
            assert!(
                (demuxed[0][t].0 - s0[t].0).abs() < 1e-9,
                "Muxer stream 0 real mismatch at t={}",
                t
            );
            assert!(
                (demuxed[0][t].1 - s0[t].1).abs() < 1e-9,
                "Muxer stream 0 imag mismatch at t={}",
                t
            );
        }
        // Check stream 1 recovery
        for t in 0..sig_len {
            assert!(
                (demuxed[1][t].0 - s1[t].0).abs() < 1e-9,
                "Muxer stream 1 real mismatch at t={}",
                t
            );
            assert!(
                (demuxed[1][t].1 - s1[t].1).abs() < 1e-9,
                "Muxer stream 1 imag mismatch at t={}",
                t
            );
        }
    }

    #[test]
    fn test_multiplexer_modes_accessor() {
        let config = OamConfig {
            num_elements: 4,
            radius_wavelengths: 0.3,
            max_mode: 1,
        };
        let modes = vec![0, 1];
        let muxer = OamMultiplexer::new(config, modes.clone());
        assert_eq!(muxer.modes(), &[0, 1]);
    }

    #[test]
    fn test_small_array_two_elements() {
        // Minimal 2-element array: supports modes 0 and 1 (aliased with -1)
        let config = OamConfig {
            num_elements: 2,
            radius_wavelengths: 0.25,
            max_mode: 1,
        };
        let gen = OamBeamGenerator::new(config);

        // Mode 0 and mode 1 should be orthogonal
        let orth = gen.mode_orthogonality(0, 1);
        assert!(orth.abs() < EPS, "Modes 0 and 1 on 2-element array should be orthogonal");

        // Roundtrip
        let s0 = vec![(1.0, 0.0); 4];
        let s1 = vec![(0.0, 1.0); 4];
        let streams: Vec<&[(f64, f64)]> = vec![&s0, &s1];
        let muxed = gen.multiplex(&streams);
        let elem_refs: Vec<&[(f64, f64)]> = muxed.iter().map(|v| v.as_slice()).collect();
        let demuxed = gen.demultiplex(&elem_refs);
        for t in 0..4 {
            assert!((demuxed[0][t].0 - 1.0).abs() < EPS);
            assert!((demuxed[1][t].1 - 1.0).abs() < EPS);
        }
    }

    #[test]
    fn test_empty_signal() {
        let gen = OamBeamGenerator::new(default_config());
        let signal: Vec<(f64, f64)> = vec![];
        let per_elem = gen.generate_mode(1, &signal);
        assert_eq!(per_elem.len(), 8);
        for elem_sig in &per_elem {
            assert!(elem_sig.is_empty());
        }
    }

    #[test]
    fn test_progressive_phase_mode_one() {
        // For mode ℓ=1, element n should have phase 2π·n/N
        let config = OamConfig {
            num_elements: 4,
            radius_wavelengths: 0.5,
            max_mode: 2,
        };
        let gen = OamBeamGenerator::new(config);
        let weights = gen.spiral_phase_plate(1);
        let norm = 1.0 / 2.0; // 1/sqrt(4)

        // Element 0: phase = 0 => (norm, 0)
        assert!((weights[0].0 - norm).abs() < EPS);
        assert!(weights[0].1.abs() < EPS);

        // Element 1: phase = π/2 => (0, norm)
        assert!(weights[1].0.abs() < EPS);
        assert!((weights[1].1 - norm).abs() < EPS);

        // Element 2: phase = π => (-norm, 0)
        assert!((weights[2].0 + norm).abs() < EPS);
        assert!(weights[2].1.abs() < EPS);

        // Element 3: phase = 3π/2 => (0, -norm)
        assert!(weights[3].0.abs() < EPS);
        assert!((weights[3].1 + norm).abs() < EPS);
    }
}
