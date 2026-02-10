//! # Bispectrum Analyzer
//!
//! Third-order polyspectral (bispectrum) analysis for non-Gaussian signal
//! characterization, phase coupling detection, and modulation recognition.
//!
//! The bispectrum is defined as:
//!
//! ```text
//! B(f1, f2) = E[ X(f1) * X(f2) * conj(X(f1 + f2)) ]
//! ```
//!
//! where `X(f)` is the DFT of the signal. Unlike the power spectrum, the
//! bispectrum retains phase information and can detect nonlinear (quadratic)
//! phase coupling between frequency components.
//!
//! The **bicoherence** normalizes the bispectrum magnitude to `[0, 1]`:
//!
//! ```text
//! b²(f1, f2) = |B(f1, f2)|² / ( E[|X(f1) X(f2)|²] * E[|X(f1+f2)|²] )
//! ```
//!
//! A bicoherence near 1.0 indicates strong quadratic phase coupling; a value
//! near 0.0 is consistent with a Gaussian (linear) process.
//!
//! # Example
//!
//! ```rust
//! use r4w_core::bispectrum_analyzer::BispectrumAnalyzer;
//!
//! // Create a simple test signal (128 samples of a tone + harmonic)
//! let n = 128;
//! let signal: Vec<f64> = (0..n)
//!     .map(|i| {
//!         let t = i as f64 / n as f64;
//!         (2.0 * std::f64::consts::PI * 4.0 * t).sin()
//!             + 0.5 * (2.0 * std::f64::consts::PI * 8.0 * t).sin()
//!     })
//!     .collect();
//!
//! let analyzer = BispectrumAnalyzer::new(n, 1);
//! let result = analyzer.compute(&signal);
//!
//! // Bispectrum matrix is n x n of complex tuples
//! assert_eq!(result.bispectrum.len(), n);
//! assert_eq!(result.bispectrum[0].len(), n);
//!
//! // Bicoherence values lie in [0, 1]
//! let bic = analyzer.bicoherence(&signal);
//! for row in &bic {
//!     for &val in row {
//!         assert!(val >= 0.0 && val <= 1.0 + 1e-9);
//!     }
//! }
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Complex arithmetic helpers using (f64, f64) tuples
// ---------------------------------------------------------------------------

type C64 = (f64, f64);

const CZERO: C64 = (0.0, 0.0);

#[inline]
fn c_add(a: C64, b: C64) -> C64 {
    (a.0 + b.0, a.1 + b.1)
}

#[inline]
fn c_sub(a: C64, b: C64) -> C64 {
    (a.0 - b.0, a.1 - b.1)
}

#[inline]
fn c_mul(a: C64, b: C64) -> C64 {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

#[inline]
fn c_conj(a: C64) -> C64 {
    (a.0, -a.1)
}

#[inline]
fn c_abs2(a: C64) -> f64 {
    a.0 * a.0 + a.1 * a.1
}

#[inline]
fn c_abs(a: C64) -> f64 {
    c_abs2(a).sqrt()
}

#[inline]
fn c_scale(a: C64, s: f64) -> C64 {
    (a.0 * s, a.1 * s)
}

#[inline]
fn c_from_polar(mag: f64, phase: f64) -> C64 {
    (mag * phase.cos(), mag * phase.sin())
}

// ---------------------------------------------------------------------------
// DFT helper
// ---------------------------------------------------------------------------

/// Compute the Discrete Fourier Transform of a real-valued signal.
///
/// Returns a vector of `N` complex frequency-domain samples.
pub fn dft_real(signal: &[f64]) -> Vec<C64> {
    let n = signal.len();
    let mut out = vec![CZERO; n];
    for k in 0..n {
        let mut sum = CZERO;
        for (t, &x) in signal.iter().enumerate() {
            let angle = -2.0 * PI * (k as f64) * (t as f64) / (n as f64);
            sum = c_add(sum, c_mul((x, 0.0), c_from_polar(1.0, angle)));
        }
        out[k] = sum;
    }
    out
}

/// Compute the DFT of a complex-valued signal.
pub fn dft_complex(signal: &[C64]) -> Vec<C64> {
    let n = signal.len();
    let mut out = vec![CZERO; n];
    for k in 0..n {
        let mut sum = CZERO;
        for (t, &x) in signal.iter().enumerate() {
            let angle = -2.0 * PI * (k as f64) * (t as f64) / (n as f64);
            sum = c_add(sum, c_mul(x, c_from_polar(1.0, angle)));
        }
        out[k] = sum;
    }
    out
}

// ---------------------------------------------------------------------------
// Result types
// ---------------------------------------------------------------------------

/// Result of a bispectrum computation.
#[derive(Debug, Clone)]
pub struct BispectrumResult {
    /// 2-D bispectrum matrix `B(f1, f2)` of size `fft_size x fft_size`.
    pub bispectrum: Vec<Vec<C64>>,
    /// FFT size used.
    pub fft_size: usize,
    /// Number of segments averaged.
    pub num_segments: usize,
}

/// A detected phase-coupling event.
#[derive(Debug, Clone, PartialEq)]
pub struct PhaseCoupling {
    /// Frequency bin index f1.
    pub f1: usize,
    /// Frequency bin index f2.
    pub f2: usize,
    /// Bicoherence value at `(f1, f2)`.
    pub bicoherence: f64,
    /// Bispectrum phase (radians) at `(f1, f2)`.
    pub phase: f64,
}

/// Summary of a Gaussianity test.
#[derive(Debug, Clone, PartialEq)]
pub struct GaussianityResult {
    /// Mean bicoherence across the principal domain.
    pub mean_bicoherence: f64,
    /// Maximum bicoherence in the principal domain.
    pub max_bicoherence: f64,
    /// Whether the signal appears Gaussian at the given threshold.
    pub is_gaussian: bool,
    /// Threshold used for the test.
    pub threshold: f64,
}

// ---------------------------------------------------------------------------
// BispectrumAnalyzer
// ---------------------------------------------------------------------------

/// Configurable bispectrum / bicoherence analyzer.
///
/// Splits the input signal into overlapping segments, computes the DFT of each,
/// and accumulates the bispectrum estimate via ensemble averaging.
#[derive(Debug, Clone)]
pub struct BispectrumAnalyzer {
    fft_size: usize,
    num_segments: usize,
}

impl BispectrumAnalyzer {
    /// Create a new analyzer.
    ///
    /// * `fft_size` - Number of samples per DFT segment (must be >= 4).
    /// * `num_segments` - Number of segments to average. If the signal is
    ///   shorter than `fft_size * num_segments`, segments are created with
    ///   50% overlap (or the signal is zero-padded for a single segment).
    ///
    /// # Panics
    ///
    /// Panics if `fft_size < 4` or `num_segments == 0`.
    pub fn new(fft_size: usize, num_segments: usize) -> Self {
        assert!(fft_size >= 4, "fft_size must be >= 4");
        assert!(num_segments >= 1, "num_segments must be >= 1");
        Self {
            fft_size,
            num_segments,
        }
    }

    /// Return the configured FFT size.
    pub fn fft_size(&self) -> usize {
        self.fft_size
    }

    /// Return the configured number of segments.
    pub fn num_segments(&self) -> usize {
        self.num_segments
    }

    // ------------------------------------------------------------------
    // Segment extraction
    // ------------------------------------------------------------------

    /// Extract segments from the signal. Uses 50% overlap when there
    /// are multiple segments. Zero-pads if the signal is too short.
    fn extract_segments(&self, signal: &[f64]) -> Vec<Vec<f64>> {
        let n = self.fft_size;
        let hop = if self.num_segments > 1 { n / 2 } else { n };
        let mut segments = Vec::with_capacity(self.num_segments);

        for i in 0..self.num_segments {
            let start = i * hop;
            let mut seg = vec![0.0; n];
            for j in 0..n {
                if start + j < signal.len() {
                    seg[j] = signal[start + j];
                }
            }
            segments.push(seg);
        }
        segments
    }

    // ------------------------------------------------------------------
    // Core bispectrum computation
    // ------------------------------------------------------------------

    /// Compute the bispectrum of a real-valued signal.
    ///
    /// Returns a [`BispectrumResult`] containing the 2-D bispectrum matrix
    /// `B(f1, f2)` averaged over segments.
    pub fn compute(&self, signal: &[f64]) -> BispectrumResult {
        let n = self.fft_size;
        let segments = self.extract_segments(signal);
        let actual_segs = segments.len();

        // Accumulate bispectrum
        let mut bispec = vec![vec![CZERO; n]; n];

        for seg in &segments {
            let x = dft_real(seg);
            for f1 in 0..n {
                for f2 in 0..n {
                    let f3 = (f1 + f2) % n; // wrap-around for circular DFT
                    let triple = c_mul(c_mul(x[f1], x[f2]), c_conj(x[f3]));
                    bispec[f1][f2] = c_add(bispec[f1][f2], triple);
                }
            }
        }

        // Average
        let inv = 1.0 / actual_segs as f64;
        for row in &mut bispec {
            for val in row.iter_mut() {
                *val = c_scale(*val, inv);
            }
        }

        BispectrumResult {
            bispectrum: bispec,
            fft_size: n,
            num_segments: actual_segs,
        }
    }

    /// Compute the bicoherence (normalized bispectrum magnitude) matrix.
    ///
    /// Returns an `fft_size x fft_size` matrix of values in `[0, 1]`.
    pub fn bicoherence(&self, signal: &[f64]) -> Vec<Vec<f64>> {
        let n = self.fft_size;
        let segments = self.extract_segments(signal);
        let actual_segs = segments.len();

        // Accumulators
        let mut bispec = vec![vec![CZERO; n]; n];
        let mut denom_a = vec![vec![0.0f64; n]; n]; // E[|X(f1)X(f2)|^2]
        let mut denom_b = vec![vec![0.0f64; n]; n]; // E[|X(f1+f2)|^2]

        for seg in &segments {
            let x = dft_real(seg);
            for f1 in 0..n {
                for f2 in 0..n {
                    let f3 = (f1 + f2) % n;
                    let prod12 = c_mul(x[f1], x[f2]);
                    let triple = c_mul(prod12, c_conj(x[f3]));
                    bispec[f1][f2] = c_add(bispec[f1][f2], triple);
                    denom_a[f1][f2] += c_abs2(prod12);
                    denom_b[f1][f2] += c_abs2(x[f3]);
                }
            }
        }

        let inv = 1.0 / actual_segs as f64;
        let mut bic = vec![vec![0.0f64; n]; n];

        for f1 in 0..n {
            for f2 in 0..n {
                let num = c_abs2(bispec[f1][f2]) * inv * inv;
                let den = (denom_a[f1][f2] * inv) * (denom_b[f1][f2] * inv);
                if den > 1e-30 {
                    let val = (num / den).sqrt().min(1.0);
                    bic[f1][f2] = val;
                }
            }
        }

        bic
    }

    /// Detect phase-coupled frequency pairs whose bicoherence exceeds a
    /// threshold.
    ///
    /// Only the **principal domain** (f1 >= 0, f2 >= 0, f1 >= f2,
    /// f1 + f2 < N/2) is scanned to avoid redundant detections.
    pub fn detect_phase_coupling(
        &self,
        signal: &[f64],
        threshold: f64,
    ) -> Vec<PhaseCoupling> {
        let n = self.fft_size;
        let bic = self.bicoherence(signal);
        let result = self.compute(signal);
        let mut couplings = Vec::new();

        for f1 in 0..n / 2 {
            for f2 in 0..=f1 {
                if f1 + f2 >= n / 2 {
                    continue;
                }
                if bic[f1][f2] > threshold {
                    let phase = result.bispectrum[f1][f2].1.atan2(result.bispectrum[f1][f2].0);
                    couplings.push(PhaseCoupling {
                        f1,
                        f2,
                        bicoherence: bic[f1][f2],
                        phase,
                    });
                }
            }
        }

        // Sort by descending bicoherence
        couplings.sort_by(|a, b| b.bicoherence.partial_cmp(&a.bicoherence).unwrap());
        couplings
    }

    /// Perform a Gaussianity test based on bicoherence statistics.
    ///
    /// A Gaussian (linear) process has zero bispectrum, so the bicoherence
    /// should be small everywhere. If the mean bicoherence in the principal
    /// domain exceeds `threshold`, the signal is classified as non-Gaussian.
    pub fn gaussianity_test(
        &self,
        signal: &[f64],
        threshold: f64,
    ) -> GaussianityResult {
        let n = self.fft_size;
        let bic = self.bicoherence(signal);

        let mut sum = 0.0;
        let mut max_val = 0.0f64;
        let mut count = 0usize;

        for f1 in 1..n / 2 {
            for f2 in 1..=f1 {
                if f1 + f2 >= n / 2 {
                    continue;
                }
                let v = bic[f1][f2];
                sum += v;
                if v > max_val {
                    max_val = v;
                }
                count += 1;
            }
        }

        let mean = if count > 0 { sum / count as f64 } else { 0.0 };

        GaussianityResult {
            mean_bicoherence: mean,
            max_bicoherence: max_val,
            is_gaussian: mean <= threshold,
            threshold,
        }
    }

    /// Extract the diagonal slice `B(f, f)` from the bispectrum.
    pub fn diagonal_slice(&self, result: &BispectrumResult) -> Vec<C64> {
        let n = result.fft_size;
        (0..n).map(|f| result.bispectrum[f][f]).collect()
    }

    /// Extract a 1-D slice at a fixed `f2`: `B(f1, f2_fixed)` for all `f1`.
    pub fn slice_at_f2(&self, result: &BispectrumResult, f2: usize) -> Vec<C64> {
        let n = result.fft_size;
        assert!(f2 < n, "f2 out of range");
        (0..n).map(|f1| result.bispectrum[f1][f2]).collect()
    }

    /// Extract a 1-D slice at a fixed `f1`: `B(f1_fixed, f2)` for all `f2`.
    pub fn slice_at_f1(&self, result: &BispectrumResult, f1: usize) -> Vec<C64> {
        let n = result.fft_size;
        assert!(f1 < n, "f1 out of range");
        result.bispectrum[f1].clone()
    }

    /// Bispectrum magnitude matrix: `|B(f1, f2)|`.
    pub fn magnitude_matrix(&self, result: &BispectrumResult) -> Vec<Vec<f64>> {
        result
            .bispectrum
            .iter()
            .map(|row| row.iter().map(|&v| c_abs(v)).collect())
            .collect()
    }

    /// Bispectrum phase matrix: `angle(B(f1, f2))` in radians.
    pub fn phase_matrix(&self, result: &BispectrumResult) -> Vec<Vec<f64>> {
        result
            .bispectrum
            .iter()
            .map(|row| row.iter().map(|v| v.1.atan2(v.0)).collect())
            .collect()
    }
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    /// Helper: generate a pure sinusoid of `freq` cycles per `n` samples.
    fn tone(n: usize, freq: f64, amplitude: f64) -> Vec<f64> {
        (0..n)
            .map(|i| amplitude * (2.0 * PI * freq * i as f64 / n as f64).sin())
            .collect()
    }

    /// Helper: generate white Gaussian-ish noise via a simple LCG + Box-Muller.
    fn noise(n: usize, seed: u64) -> Vec<f64> {
        let mut state = seed;
        let mut out = Vec::with_capacity(n);
        for _ in 0..n / 2 + 1 {
            // LCG
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u1 = (state >> 33) as f64 / (1u64 << 31) as f64;
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let u2 = (state >> 33) as f64 / (1u64 << 31) as f64;
            let u1 = u1.max(1e-10); // avoid log(0)
            let r = (-2.0 * u1.ln()).sqrt();
            let theta = 2.0 * PI * u2;
            out.push(r * theta.cos());
            out.push(r * theta.sin());
        }
        out.truncate(n);
        out
    }

    // -----------------------------------------------------------------------
    // Construction / basic properties
    // -----------------------------------------------------------------------

    #[test]
    fn test_new_basic() {
        let a = BispectrumAnalyzer::new(16, 1);
        assert_eq!(a.fft_size(), 16);
        assert_eq!(a.num_segments(), 1);
    }

    #[test]
    #[should_panic(expected = "fft_size must be >= 4")]
    fn test_new_fft_too_small() {
        BispectrumAnalyzer::new(2, 1);
    }

    #[test]
    #[should_panic(expected = "num_segments must be >= 1")]
    fn test_new_zero_segments() {
        BispectrumAnalyzer::new(16, 0);
    }

    // -----------------------------------------------------------------------
    // DFT helpers
    // -----------------------------------------------------------------------

    #[test]
    fn test_dft_dc_component() {
        // Constant signal of 3.0 -> DFT bin 0 should be N * 3.0
        let sig = vec![3.0; 8];
        let x = dft_real(&sig);
        assert!((x[0].0 - 24.0).abs() < 1e-9);
        assert!(x[0].1.abs() < 1e-9);
        // All other bins should be ~0
        for k in 1..8 {
            assert!(c_abs(x[k]) < 1e-9, "bin {} non-zero: {:?}", k, x[k]);
        }
    }

    #[test]
    fn test_dft_single_tone() {
        let n = 16;
        let sig = tone(n, 3.0, 1.0);
        let x = dft_real(&sig);
        // Energy should concentrate at bin 3 and bin N-3=13
        let mag3 = c_abs(x[3]);
        let mag13 = c_abs(x[13]);
        assert!(mag3 > 5.0, "bin 3 magnitude too small: {}", mag3);
        assert!((mag3 - mag13).abs() < 1e-9);
    }

    #[test]
    fn test_dft_complex_signal() {
        let sig: Vec<C64> = vec![(1.0, 0.0), (0.0, 1.0), (-1.0, 0.0), (0.0, -1.0)];
        let x = dft_complex(&sig);
        // This is a single-frequency complex exponential at bin 1
        assert!(c_abs(x[1]) > 3.5);
        assert!(c_abs(x[0]) < 1e-9);
        assert!(c_abs(x[2]) < 1e-9);
        assert!(c_abs(x[3]) < 1e-9);
    }

    // -----------------------------------------------------------------------
    // Bispectrum shape
    // -----------------------------------------------------------------------

    #[test]
    fn test_bispectrum_result_shape() {
        let n = 16;
        let sig = tone(n, 2.0, 1.0);
        let analyzer = BispectrumAnalyzer::new(n, 1);
        let result = analyzer.compute(&sig);
        assert_eq!(result.bispectrum.len(), n);
        assert_eq!(result.bispectrum[0].len(), n);
        assert_eq!(result.fft_size, n);
        assert_eq!(result.num_segments, 1);
    }

    // -----------------------------------------------------------------------
    // Bicoherence range
    // -----------------------------------------------------------------------

    #[test]
    fn test_bicoherence_range() {
        let n = 16;
        let sig = tone(n, 2.0, 1.0);
        let analyzer = BispectrumAnalyzer::new(n, 1);
        let bic = analyzer.bicoherence(&sig);
        for f1 in 0..n {
            for f2 in 0..n {
                assert!(
                    bic[f1][f2] >= 0.0 && bic[f1][f2] <= 1.0 + 1e-9,
                    "bicoherence out of range at ({}, {}): {}",
                    f1,
                    f2,
                    bic[f1][f2]
                );
            }
        }
    }

    // -----------------------------------------------------------------------
    // Deterministic single-segment: bispectrum of a pure tone
    // -----------------------------------------------------------------------

    #[test]
    fn test_coupled_harmonics_bispectrum_nonzero() {
        // Signal with quadratic phase coupling: components at bins 2, 3, and 5
        // where f1=2, f2=3, f1+f2=5. Phase-locked: phi_5 = phi_2 + phi_3.
        let n = 32;
        let sig: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                let phi2 = 2.0 * PI * 2.0 * t;
                let phi3 = 2.0 * PI * 3.0 * t;
                phi2.sin() + phi3.sin() + (phi2 + phi3).sin()
            })
            .collect();
        let analyzer = BispectrumAnalyzer::new(n, 1);
        let result = analyzer.compute(&sig);
        // B(2, 3) = X(2)*X(3)*conj(X(5)) should be nonzero due to coupling.
        let b_2_3 = result.bispectrum[2][3];
        assert!(
            c_abs(b_2_3) > 1.0,
            "Expected nonzero bispectrum at (2,3): {:?}",
            b_2_3
        );
    }

    // -----------------------------------------------------------------------
    // Multi-segment averaging
    // -----------------------------------------------------------------------

    #[test]
    fn test_multi_segment_reduces_variance() {
        let n = 32;
        let long_signal: Vec<f64> = (0..n * 10)
            .map(|i| {
                let t = i as f64 / n as f64;
                (2.0 * PI * 3.0 * t).sin() + 0.2 * noise(1, i as u64 + 42)[0]
            })
            .collect();

        let a1 = BispectrumAnalyzer::new(n, 1);
        let a4 = BispectrumAnalyzer::new(n, 4);

        let r1 = a1.compute(&long_signal);
        let r4 = a4.compute(&long_signal);

        // Both should produce valid matrices
        assert_eq!(r1.bispectrum.len(), n);
        assert_eq!(r4.bispectrum.len(), n);
        assert_eq!(r4.num_segments, 4);
    }

    // -----------------------------------------------------------------------
    // Phase coupling detection
    // -----------------------------------------------------------------------

    #[test]
    fn test_phase_coupling_detection_returns_results() {
        let n = 32;
        // Signal with quadratic phase coupling: f1=3, f2=3, f1+f2=6
        let sig: Vec<f64> = (0..n)
            .map(|i| {
                let t = i as f64 / n as f64;
                let ph1 = 2.0 * PI * 3.0 * t;
                let ph2 = 2.0 * PI * 3.0 * t;
                ph1.sin() + ph2.sin() + (ph1 + ph2).sin()
            })
            .collect();

        let analyzer = BispectrumAnalyzer::new(n, 1);
        let couplings = analyzer.detect_phase_coupling(&sig, 0.01);

        // Should find at least some coupling
        assert!(
            !couplings.is_empty(),
            "Expected to detect phase coupling in a coupled signal"
        );

        // Couplings should be sorted by descending bicoherence
        for w in couplings.windows(2) {
            assert!(w[0].bicoherence >= w[1].bicoherence);
        }
    }

    #[test]
    fn test_phase_coupling_struct_fields() {
        let pc = PhaseCoupling {
            f1: 3,
            f2: 5,
            bicoherence: 0.85,
            phase: 1.23,
        };
        assert_eq!(pc.f1, 3);
        assert_eq!(pc.f2, 5);
        assert!((pc.bicoherence - 0.85).abs() < 1e-12);
        assert!((pc.phase - 1.23).abs() < 1e-12);
    }

    // -----------------------------------------------------------------------
    // Gaussianity test
    // -----------------------------------------------------------------------

    #[test]
    fn test_gaussianity_on_noise() {
        let n = 32;
        let sig = noise(n, 12345);
        let analyzer = BispectrumAnalyzer::new(n, 1);
        let result = analyzer.gaussianity_test(&sig, 0.95);

        assert!(result.mean_bicoherence >= 0.0);
        assert!(result.max_bicoherence >= result.mean_bicoherence);
        assert_eq!(result.threshold, 0.95);
    }

    #[test]
    fn test_gaussianity_result_fields() {
        let gr = GaussianityResult {
            mean_bicoherence: 0.1,
            max_bicoherence: 0.4,
            is_gaussian: true,
            threshold: 0.5,
        };
        assert!(gr.is_gaussian);
        assert_eq!(gr.threshold, 0.5);
    }

    // -----------------------------------------------------------------------
    // Diagonal and 1-D slice extraction
    // -----------------------------------------------------------------------

    #[test]
    fn test_diagonal_slice() {
        let n = 16;
        let sig = tone(n, 2.0, 1.0);
        let analyzer = BispectrumAnalyzer::new(n, 1);
        let result = analyzer.compute(&sig);
        let diag = analyzer.diagonal_slice(&result);
        assert_eq!(diag.len(), n);
        for k in 0..n {
            assert_eq!(diag[k], result.bispectrum[k][k]);
        }
    }

    #[test]
    fn test_slice_at_f2() {
        let n = 16;
        let sig = tone(n, 2.0, 1.0);
        let analyzer = BispectrumAnalyzer::new(n, 1);
        let result = analyzer.compute(&sig);
        let slice = analyzer.slice_at_f2(&result, 3);
        assert_eq!(slice.len(), n);
        for f1 in 0..n {
            assert_eq!(slice[f1], result.bispectrum[f1][3]);
        }
    }

    #[test]
    fn test_slice_at_f1() {
        let n = 16;
        let sig = tone(n, 2.0, 1.0);
        let analyzer = BispectrumAnalyzer::new(n, 1);
        let result = analyzer.compute(&sig);
        let slice = analyzer.slice_at_f1(&result, 5);
        assert_eq!(slice.len(), n);
        for f2 in 0..n {
            assert_eq!(slice[f2], result.bispectrum[5][f2]);
        }
    }

    #[test]
    #[should_panic(expected = "f2 out of range")]
    fn test_slice_at_f2_out_of_range() {
        let analyzer = BispectrumAnalyzer::new(8, 1);
        let sig = vec![0.0; 8];
        let result = analyzer.compute(&sig);
        analyzer.slice_at_f2(&result, 8);
    }

    // -----------------------------------------------------------------------
    // Magnitude and phase matrices
    // -----------------------------------------------------------------------

    #[test]
    fn test_magnitude_matrix_non_negative() {
        let n = 16;
        let sig = tone(n, 2.0, 1.0);
        let analyzer = BispectrumAnalyzer::new(n, 1);
        let result = analyzer.compute(&sig);
        let mag = analyzer.magnitude_matrix(&result);
        assert_eq!(mag.len(), n);
        for row in &mag {
            assert_eq!(row.len(), n);
            for &v in row {
                assert!(v >= 0.0);
            }
        }
    }

    #[test]
    fn test_phase_matrix_range() {
        let n = 16;
        let sig = tone(n, 2.0, 1.0);
        let analyzer = BispectrumAnalyzer::new(n, 1);
        let result = analyzer.compute(&sig);
        let phase = analyzer.phase_matrix(&result);
        assert_eq!(phase.len(), n);
        for row in &phase {
            for &v in row {
                assert!(v >= -PI && v <= PI, "phase out of [-pi, pi]: {}", v);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Zero signal
    // -----------------------------------------------------------------------

    #[test]
    fn test_zero_signal() {
        let n = 16;
        let sig = vec![0.0; n];
        let analyzer = BispectrumAnalyzer::new(n, 1);
        let result = analyzer.compute(&sig);
        for row in &result.bispectrum {
            for &v in row {
                assert!(c_abs(v) < 1e-15, "expected zero, got {:?}", v);
            }
        }
        let bic = analyzer.bicoherence(&sig);
        for row in &bic {
            for &v in row {
                assert!(v.abs() < 1e-15, "expected zero bicoherence, got {}", v);
            }
        }
    }

    // -----------------------------------------------------------------------
    // Complex helper arithmetic
    // -----------------------------------------------------------------------

    #[test]
    fn test_complex_helpers() {
        let a: C64 = (3.0, 4.0);
        let b: C64 = (1.0, -2.0);

        // add
        let s = c_add(a, b);
        assert!((s.0 - 4.0).abs() < 1e-12);
        assert!((s.1 - 2.0).abs() < 1e-12);

        // sub
        let d = c_sub(a, b);
        assert!((d.0 - 2.0).abs() < 1e-12);
        assert!((d.1 - 6.0).abs() < 1e-12);

        // mul: (3+4i)(1-2i) = 3-6i+4i-8i^2 = 3-6i+4i+8 = 11-2i
        let m = c_mul(a, b);
        assert!((m.0 - 11.0).abs() < 1e-12);
        assert!((m.1 - (-2.0)).abs() < 1e-12);

        // conj
        let c = c_conj(a);
        assert!((c.0 - 3.0).abs() < 1e-12);
        assert!((c.1 - (-4.0)).abs() < 1e-12);

        // abs2
        assert!((c_abs2(a) - 25.0).abs() < 1e-12);

        // abs
        assert!((c_abs(a) - 5.0).abs() < 1e-12);

        // scale
        let sc = c_scale(a, 2.0);
        assert!((sc.0 - 6.0).abs() < 1e-12);
        assert!((sc.1 - 8.0).abs() < 1e-12);

        // from_polar
        let p = c_from_polar(5.0, 0.0);
        assert!((p.0 - 5.0).abs() < 1e-12);
        assert!(p.1.abs() < 1e-12);
    }
}
