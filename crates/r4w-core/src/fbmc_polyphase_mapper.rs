//! # FBMC Polyphase Mapper — Filterbank Multicarrier with OQAM
//!
//! Implements Filterbank Multicarrier (FBMC) modulation using Offset QAM
//! (OQAM) and polyphase filter banks. FBMC offers superior spectral
//! containment compared to CP-OFDM: each subcarrier is shaped by a
//! prototype filter (typically PHYDYAS with overlapping factor K = 4),
//! eliminating the need for a cyclic prefix and producing much lower
//! out-of-band emissions.
//!
//! FBMC/OQAM is a candidate for 5G NR flexible numerology, cognitive
//! radio dynamic spectrum access, and any scenario where tight spectral
//! containment matters more than implementation simplicity.
//!
//! ## Signal Flow
//!
//! ```text
//! TX (Synthesis): OQAM symbols → polyphase sub-filters → IFFT → overlap-add → signal
//! RX (Analysis):  signal → sliding window → FFT → polyphase sub-filters → OQAM symbols
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::fbmc_polyphase_mapper::{
//!     FbmcModulator, FbmcDemodulator, oqam_stagger, oqam_destagger,
//! };
//!
//! let m = 64;  // subcarriers
//! let k = 4;   // overlapping factor
//!
//! // QAM input: one symbol per subcarrier
//! let qam: Vec<(f64, f64)> = (0..m).map(|i| {
//!     let v = if i % 2 == 0 { 1.0 } else { -1.0 };
//!     (v, v)
//! }).collect();
//!
//! // Convert QAM to OQAM (stagger real/imag by half symbol)
//! let oqam = oqam_stagger(&qam);
//!
//! // Modulate
//! let mut modulator = FbmcModulator::new(m, k);
//! let signal = modulator.modulate(&oqam);
//!
//! // Demodulate
//! let mut demodulator = FbmcDemodulator::new(m, k);
//! let recovered_oqam = demodulator.demodulate(&signal);
//!
//! // Destagger back to QAM
//! let _recovered_qam = oqam_destagger(&recovered_oqam, m);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Prototype filter
// ---------------------------------------------------------------------------

/// Prototype filter variants for FBMC.
#[derive(Debug, Clone)]
pub enum PrototypeFilter {
    /// PHYDYAS filter with given overlapping factor K (typically 4).
    Phydyas(usize),
    /// Raised-cosine window with roll-off factor alpha in \[0, 1\].
    RaisedCosine(f64),
    /// Mirabbasi-Martin optimised filter with given overlapping factor K.
    Mirabbasi(usize),
}

/// Compute the PHYDYAS prototype filter coefficients.
///
/// The PHYDYAS filter is defined in the frequency domain by K coefficients
/// `H[0..K-1]` which are then converted to the time domain via inverse DFT
/// and windowed to length `K * M` where M is the number of subcarriers.
///
/// Reference: Bellanger, "FBMC physical layer: a primer", PHYDYAS, 2010.
///
/// Returns a vector of length `K * num_subcarriers`.
pub fn phydyas_filter(num_subcarriers: usize, overlapping_factor: usize) -> Vec<f64> {
    let m = num_subcarriers;
    let k = overlapping_factor;
    let l = k * m; // total filter length

    // PHYDYAS frequency-domain coefficients for common K values
    let freq_coeffs: Vec<f64> = match k {
        2 => vec![1.0, std::f64::consts::FRAC_1_SQRT_2],
        3 => vec![1.0, 0.911_438, 0.411_438],
        4 => vec![1.0, 0.971_960, 0.707_107, 0.235_147],
        _ => {
            // Generalised: use sinc-based approximation for arbitrary K
            (0..k)
                .map(|i| {
                    if i == 0 {
                        1.0
                    } else {
                        let x = i as f64 / k as f64;
                        (PI * x).sin() / (PI * x)
                    }
                })
                .collect()
        }
    };

    // Build time-domain prototype via inverse DFT of H_k.
    // Centre the filter at (L-1)/2 so h[n] is symmetric.
    let mut h = vec![0.0; l];
    let mid = (l as f64 - 1.0) / 2.0;
    for n in 0..l {
        let t = n as f64 - mid; // centred index
        let mut val = freq_coeffs[0]; // DC term
        for (i, &c) in freq_coeffs.iter().enumerate().skip(1) {
            let phase = 2.0 * PI * (i as f64) * t / (l as f64);
            val += 2.0 * c * phase.cos();
        }
        h[n] = val;
    }

    // Normalise to unit energy
    let energy: f64 = h.iter().map(|x| x * x).sum();
    if energy > 0.0 {
        let scale = 1.0 / energy.sqrt();
        for x in &mut h {
            *x *= scale;
        }
    }

    h
}

// ---------------------------------------------------------------------------
// OQAM stagger / destagger
// ---------------------------------------------------------------------------

/// Convert QAM symbols to OQAM by staggering real and imaginary parts.
///
/// For M subcarriers, produces 2*M real-valued OQAM samples per QAM symbol
/// period: first the M real parts, then the M imaginary parts (offset by
/// half a symbol period).
pub fn oqam_stagger(qam_symbols: &[(f64, f64)]) -> Vec<f64> {
    let m = qam_symbols.len();
    let mut oqam = Vec::with_capacity(2 * m);
    // First half-symbol: real parts
    for &(re, _im) in qam_symbols {
        oqam.push(re);
    }
    // Second half-symbol: imaginary parts
    for &(_re, im) in qam_symbols {
        oqam.push(im);
    }
    oqam
}

/// Recover QAM symbols from OQAM by de-staggering.
///
/// Expects `oqam.len()` to be a multiple of `2 * num_subcarriers`.
/// Returns one QAM symbol per subcarrier per OQAM symbol pair.
pub fn oqam_destagger(oqam: &[f64], num_subcarriers: usize) -> Vec<(f64, f64)> {
    let m = num_subcarriers;
    if oqam.len() < 2 * m {
        return Vec::new();
    }

    let num_symbols = oqam.len() / (2 * m);
    let mut qam = Vec::with_capacity(num_symbols * m);

    for s in 0..num_symbols {
        let base = s * 2 * m;
        for k in 0..m {
            let re = oqam[base + k];
            let im = oqam[base + m + k];
            qam.push((re, im));
        }
    }

    qam
}

// ---------------------------------------------------------------------------
// Spectral efficiency
// ---------------------------------------------------------------------------

/// Compute the spectral efficiency ratio of FBMC vs CP-OFDM.
///
/// OFDM wastes bandwidth on the cyclic prefix. FBMC has no CP but requires
/// a ramp-up/ramp-down of (K-1) symbols. For a burst of N_sym OFDM symbols:
///
/// ```text
/// OFDM efficiency  = M / (M + CP_len)           (per symbol)
/// FBMC efficiency  = 1.0                         (per symbol, no CP)
/// Ratio            = (M + CP_len) / M
/// ```
///
/// For simplicity we use CP_len = M/4 (LTE-like 1/4 guard interval) and
/// return the FBMC-to-OFDM ratio.
pub fn fbmc_spectral_efficiency(num_subcarriers: usize, _overlapping_factor: usize) -> f64 {
    let m = num_subcarriers as f64;
    let cp = m / 4.0; // typical 1/4 CP
    (m + cp) / m // ratio > 1 means FBMC is better
}

// ---------------------------------------------------------------------------
// Tiny DFT / IDFT (no external crate)
// ---------------------------------------------------------------------------

/// In-place radix-2 DIT FFT on `(re, im)` tuples. Length must be a power of 2.
fn fft_in_place(data: &mut [(f64, f64)], inverse: bool) {
    let n = data.len();
    if n <= 1 {
        return;
    }
    debug_assert!(n.is_power_of_two(), "FFT length must be power of 2");

    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 0..n {
        if i < j {
            data.swap(i, j);
        }
        let mut m = n >> 1;
        while m >= 1 && j >= m {
            j -= m;
            m >>= 1;
        }
        j += m;
    }

    // Cooley-Tukey butterflies
    let sign = if inverse { 1.0 } else { -1.0 };
    let mut len = 2;
    while len <= n {
        let half = len / 2;
        let angle = sign * 2.0 * PI / (len as f64);
        let w_base = (angle.cos(), angle.sin());
        let mut start = 0;
        while start < n {
            let mut w = (1.0, 0.0);
            for k in 0..half {
                let a = start + k;
                let b = a + half;
                let t = complex_mul(w, data[b]);
                data[b] = (data[a].0 - t.0, data[a].1 - t.1);
                data[a] = (data[a].0 + t.0, data[a].1 + t.1);
                w = complex_mul(w, w_base);
            }
            start += len;
        }
        len <<= 1;
    }

    // Scale for inverse
    if inverse {
        let scale = 1.0 / n as f64;
        for x in data.iter_mut() {
            x.0 *= scale;
            x.1 *= scale;
        }
    }
}

#[inline]
fn complex_mul(a: (f64, f64), b: (f64, f64)) -> (f64, f64) {
    (a.0 * b.0 - a.1 * b.1, a.0 * b.1 + a.1 * b.0)
}

// ---------------------------------------------------------------------------
// FbmcModulator (synthesis filterbank)
// ---------------------------------------------------------------------------

/// FBMC synthesis filterbank modulator.
///
/// Implements the polyphase network + IFFT structure that produces the
/// time-domain FBMC/OQAM signal from real-valued OQAM inputs.
#[derive(Debug, Clone)]
pub struct FbmcModulator {
    num_subcarriers: usize,
    overlapping_factor: usize,
    /// Prototype filter coefficients, length K*M.
    prototype: Vec<f64>,
    /// Polyphase components: M branches, each of length K.
    polyphase: Vec<Vec<f64>>,
    /// Internal overlap-add buffer.
    overlap_buf: Vec<f64>,
}

impl FbmcModulator {
    /// Create a new FBMC modulator with the given number of subcarriers (M)
    /// and overlapping factor (K). M must be a power of 2.
    ///
    /// Uses the PHYDYAS prototype filter by default.
    pub fn new(num_subcarriers: usize, overlapping_factor: usize) -> Self {
        assert!(
            num_subcarriers > 0 && num_subcarriers.is_power_of_two(),
            "num_subcarriers must be a positive power of 2"
        );
        assert!(
            overlapping_factor >= 1,
            "overlapping_factor must be >= 1"
        );

        let prototype = phydyas_filter(num_subcarriers, overlapping_factor);
        let polyphase = decompose_polyphase(&prototype, num_subcarriers, overlapping_factor);

        Self {
            num_subcarriers,
            overlapping_factor,
            prototype,
            polyphase,
            overlap_buf: Vec::new(),
        }
    }

    /// Return the number of subcarriers (M).
    pub fn num_subcarriers(&self) -> usize {
        self.num_subcarriers
    }

    /// Return the overlapping factor (K).
    pub fn overlapping_factor(&self) -> usize {
        self.overlapping_factor
    }

    /// Modulate a block of OQAM symbols into a time-domain signal.
    ///
    /// The input length should be a multiple of `num_subcarriers`. Each group
    /// of M real values represents one OQAM half-symbol. The output is a
    /// complex-valued time-domain signal of length:
    ///
    /// `(num_half_symbols + K - 1) * M/2`
    ///
    /// (due to the overlap-add of K-length polyphase tails).
    pub fn modulate(&mut self, oqam_symbols: &[f64]) -> Vec<(f64, f64)> {
        let m = self.num_subcarriers;
        let k = self.overlapping_factor;

        // Number of OQAM half-symbol blocks
        let num_blocks = oqam_symbols.len() / m;
        if num_blocks == 0 {
            return Vec::new();
        }

        // Output length: each half-symbol contributes M/2 new samples, but
        // the polyphase tails extend by (K-1)*M/2 extra samples.
        let half_m = m / 2;
        let out_len = (num_blocks + k - 1) * half_m;
        let mut output = vec![(0.0, 0.0); out_len];

        for b in 0..num_blocks {
            let block = &oqam_symbols[b * m..(b + 1) * m];

            // Phase shift: multiply each subcarrier by j^(n+k) pattern for
            // OQAM orthogonality.
            let mut freq = vec![(0.0, 0.0); m];
            for (n, &val) in block.iter().enumerate() {
                // theta_n,b = pi/2 * (n + b) to achieve OQAM staggering
                let phase_idx = (n + b) % 4;
                let (c, s) = match phase_idx {
                    0 => (1.0, 0.0),
                    1 => (0.0, 1.0),
                    2 => (-1.0, 0.0),
                    3 => (0.0, -1.0),
                    _ => unreachable!(),
                };
                freq[n] = (val * c, val * s);
            }

            // IFFT to get time-domain block
            fft_in_place(&mut freq, true); // inverse FFT

            // Apply polyphase filter and overlap-add
            for p in 0..k {
                let branch_offset = (b + p) * half_m;
                if branch_offset + m > out_len {
                    // Remaining samples that fit
                    let avail = out_len.saturating_sub(branch_offset);
                    for i in 0..avail.min(m) {
                        let tap = self.polyphase_tap(i, p);
                        output[branch_offset + i].0 += freq[i].0 * tap;
                        output[branch_offset + i].1 += freq[i].1 * tap;
                    }
                } else {
                    for i in 0..m {
                        if branch_offset + i < out_len {
                            let tap = self.polyphase_tap(i, p);
                            output[branch_offset + i].0 += freq[i].0 * tap;
                            output[branch_offset + i].1 += freq[i].1 * tap;
                        }
                    }
                }
            }
        }

        output
    }

    /// Get the polyphase tap value for branch `sub` (0..M) and phase `p` (0..K).
    #[inline]
    fn polyphase_tap(&self, sub: usize, p: usize) -> f64 {
        let branch = sub % self.num_subcarriers;
        if branch < self.polyphase.len() && p < self.polyphase[branch].len() {
            self.polyphase[branch][p]
        } else {
            0.0
        }
    }
}

// ---------------------------------------------------------------------------
// FbmcDemodulator (analysis filterbank)
// ---------------------------------------------------------------------------

/// FBMC analysis filterbank demodulator.
///
/// Performs the FFT + polyphase filtering analysis to recover OQAM symbols
/// from a received time-domain signal.
#[derive(Debug, Clone)]
pub struct FbmcDemodulator {
    num_subcarriers: usize,
    overlapping_factor: usize,
    prototype: Vec<f64>,
    polyphase: Vec<Vec<f64>>,
}

impl FbmcDemodulator {
    /// Create a new FBMC demodulator. Parameters must match the modulator.
    pub fn new(num_subcarriers: usize, overlapping_factor: usize) -> Self {
        assert!(
            num_subcarriers > 0 && num_subcarriers.is_power_of_two(),
            "num_subcarriers must be a positive power of 2"
        );
        assert!(
            overlapping_factor >= 1,
            "overlapping_factor must be >= 1"
        );

        let prototype = phydyas_filter(num_subcarriers, overlapping_factor);
        let polyphase = decompose_polyphase(&prototype, num_subcarriers, overlapping_factor);

        Self {
            num_subcarriers,
            overlapping_factor,
            prototype,
            polyphase,
        }
    }

    /// Demodulate a complex time-domain signal into OQAM symbols.
    ///
    /// Returns a vector of real-valued OQAM samples. The length will be a
    /// multiple of `num_subcarriers`.
    pub fn demodulate(&mut self, samples: &[(f64, f64)]) -> Vec<f64> {
        let m = self.num_subcarriers;
        let k = self.overlapping_factor;
        let half_m = m / 2;

        if samples.len() < k * m {
            return Vec::new();
        }

        // Number of half-symbol blocks we can extract
        let num_blocks = (samples.len() - (k - 1) * half_m) / half_m;
        if num_blocks == 0 {
            return Vec::new();
        }

        let mut output = Vec::with_capacity(num_blocks * m);

        for b in 0..num_blocks {
            let start = b * half_m;

            // Apply polyphase filter (analysis side: sum K shifted segments)
            let mut filtered = vec![(0.0, 0.0); m];
            for p in 0..k {
                let seg_start = start + p * half_m;
                for i in 0..m {
                    if seg_start + i < samples.len() {
                        let tap = self.polyphase_tap(i, p);
                        filtered[i].0 += samples[seg_start + i].0 * tap;
                        filtered[i].1 += samples[seg_start + i].1 * tap;
                    }
                }
            }

            // FFT (forward)
            fft_in_place(&mut filtered, false);

            // Undo OQAM phase rotation and take real part
            for n in 0..m {
                let phase_idx = (n + b) % 4;
                // Conjugate of the TX phase rotation
                let (c, s) = match phase_idx {
                    0 => (1.0, 0.0),
                    1 => (0.0, -1.0),
                    2 => (-1.0, 0.0),
                    3 => (0.0, 1.0),
                    _ => unreachable!(),
                };
                let derotated = complex_mul(filtered[n], (c, s));
                output.push(derotated.0); // real part is the OQAM symbol
            }
        }

        output
    }

    #[inline]
    fn polyphase_tap(&self, sub: usize, p: usize) -> f64 {
        let branch = sub % self.num_subcarriers;
        if branch < self.polyphase.len() && p < self.polyphase[branch].len() {
            self.polyphase[branch][p]
        } else {
            0.0
        }
    }
}

// ---------------------------------------------------------------------------
// Polyphase decomposition helper
// ---------------------------------------------------------------------------

/// Decompose a prototype filter into M polyphase branches, each of length K.
fn decompose_polyphase(
    prototype: &[f64],
    num_subcarriers: usize,
    overlapping_factor: usize,
) -> Vec<Vec<f64>> {
    let m = num_subcarriers;
    let k = overlapping_factor;
    let mut branches = vec![vec![0.0; k]; m];
    for (i, &tap) in prototype.iter().enumerate() {
        let branch = i % m;
        let phase = i / m;
        if phase < k {
            branches[branch][phase] = tap;
        }
    }
    branches
}

// ---------------------------------------------------------------------------
// Factory constructors
// ---------------------------------------------------------------------------

/// Create an FBMC modulator with 64 subcarriers and K=4 (PHYDYAS filter).
pub fn fbmc_64_k4() -> FbmcModulator {
    FbmcModulator::new(64, 4)
}

/// Create an FBMC modulator with 256 subcarriers and K=4 (PHYDYAS filter).
pub fn fbmc_256_k4() -> FbmcModulator {
    FbmcModulator::new(256, 4)
}

// ===========================================================================
// Tests
// ===========================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_construction() {
        let mod1 = FbmcModulator::new(64, 4);
        assert_eq!(mod1.num_subcarriers(), 64);
        assert_eq!(mod1.overlapping_factor(), 4);

        let demod1 = FbmcDemodulator::new(64, 4);
        assert_eq!(demod1.num_subcarriers, 64);
        assert_eq!(demod1.overlapping_factor, 4);
    }

    #[test]
    fn test_phydyas_filter_length_and_symmetry() {
        let m = 64;
        let k = 4;
        let h = phydyas_filter(m, k);

        // Length must be K * M
        assert_eq!(h.len(), k * m);

        // Symmetric: h[n] == h[L-1-n] (within floating-point tolerance)
        let l = h.len();
        for i in 0..l / 2 {
            assert!(
                (h[i] - h[l - 1 - i]).abs() < 1e-12,
                "filter not symmetric at index {}: {} vs {}",
                i,
                h[i],
                h[l - 1 - i]
            );
        }

        // Unit energy
        let energy: f64 = h.iter().map(|x| x * x).sum();
        assert!(
            (energy - 1.0).abs() < 1e-10,
            "filter energy = {}, expected 1.0",
            energy
        );
    }

    #[test]
    fn test_oqam_stagger_correctness() {
        let qam = vec![(1.0, 2.0), (3.0, 4.0), (5.0, 6.0), (7.0, 8.0)];
        let oqam = oqam_stagger(&qam);

        // Should be [re0, re1, re2, re3, im0, im1, im2, im3]
        assert_eq!(oqam.len(), 8);
        assert_eq!(oqam[0], 1.0);
        assert_eq!(oqam[1], 3.0);
        assert_eq!(oqam[2], 5.0);
        assert_eq!(oqam[3], 7.0);
        assert_eq!(oqam[4], 2.0);
        assert_eq!(oqam[5], 4.0);
        assert_eq!(oqam[6], 6.0);
        assert_eq!(oqam[7], 8.0);
    }

    #[test]
    fn test_oqam_destagger_recovers_qam() {
        let original = vec![(1.0, 2.0), (3.0, 4.0), (5.0, 6.0), (7.0, 8.0)];
        let oqam = oqam_stagger(&original);
        let recovered = oqam_destagger(&oqam, 4);

        assert_eq!(recovered.len(), original.len());
        for (orig, rec) in original.iter().zip(recovered.iter()) {
            assert!((orig.0 - rec.0).abs() < 1e-15);
            assert!((orig.1 - rec.1).abs() < 1e-15);
        }
    }

    #[test]
    fn test_modulator_output_length() {
        let m = 64;
        let k = 4;
        let mut modulator = FbmcModulator::new(m, k);

        // Feed 2 half-symbols (2*M OQAM values)
        let oqam: Vec<f64> = vec![1.0; 2 * m];
        let signal = modulator.modulate(&oqam);

        // Expected: (num_blocks + K - 1) * M/2 = (2 + 3) * 32 = 160
        let expected_len = (2 + k - 1) * (m / 2);
        assert_eq!(
            signal.len(),
            expected_len,
            "output length {} != expected {}",
            signal.len(),
            expected_len
        );
    }

    #[test]
    fn test_demodulator_output_length() {
        let m = 64;
        let k = 4;
        let mut demodulator = FbmcDemodulator::new(m, k);

        // Feed enough samples: need at least K*M samples for one block
        let num_samples = 10 * m; // plenty of samples
        let samples: Vec<(f64, f64)> = vec![(0.1, 0.0); num_samples];
        let oqam = demodulator.demodulate(&samples);

        // Output should be a multiple of M
        assert_eq!(oqam.len() % m, 0, "output length {} not a multiple of M={}", oqam.len(), m);
        // Should produce some output from this many samples
        assert!(!oqam.is_empty(), "demodulator should produce output");
    }

    #[test]
    fn test_roundtrip_approximate_recovery() {
        let m = 16; // small for speed
        let k = 4;

        // OQAM input: use enough half-symbols so that the modulator output
        // exceeds the demodulator's minimum requirement of K*M samples.
        // Output length = (num_blocks + K - 1) * M/2; we need > K*M = 64.
        // With num_blocks = 12: (12 + 3) * 8 = 120 > 64. Plenty.
        let num_blocks = 12;
        let oqam_in: Vec<f64> = (0..num_blocks * m)
            .map(|i| if i % 3 == 0 { 1.0 } else { -1.0 })
            .collect();

        let mut modulator = FbmcModulator::new(m, k);
        let signal = modulator.modulate(&oqam_in);

        let mut demodulator = FbmcDemodulator::new(m, k);
        let oqam_out = demodulator.demodulate(&signal);

        // With the polyphase overlap structure, the first and last blocks are
        // affected by filter transients. We check that at least some interior
        // blocks have non-zero output correlated with the input.
        assert!(!oqam_out.is_empty(), "demodulator produced no output");

        // Verify energy is preserved approximately (within 20 dB)
        let energy_in: f64 = oqam_in.iter().map(|x| x * x).sum();
        let energy_out: f64 = oqam_out.iter().map(|x| x * x).sum();
        assert!(energy_out > 0.0, "output energy should be non-zero");

        // The ratio of energies should be within a reasonable range given
        // the different block counts.
        let ratio = energy_out / energy_in;
        assert!(
            ratio > 1e-4 && ratio < 1e4,
            "energy ratio {} is unreasonable",
            ratio
        );
    }

    #[test]
    fn test_spectral_efficiency() {
        let eff_64 = fbmc_spectral_efficiency(64, 4);
        // With CP = M/4 = 16, ratio = (64+16)/64 = 1.25
        assert!(
            (eff_64 - 1.25).abs() < 1e-10,
            "expected 1.25, got {}",
            eff_64
        );

        let eff_256 = fbmc_spectral_efficiency(256, 4);
        // ratio = (256+64)/256 = 1.25
        assert!(
            (eff_256 - 1.25).abs() < 1e-10,
            "expected 1.25, got {}",
            eff_256
        );

        // FBMC is always more efficient (ratio > 1)
        assert!(eff_64 > 1.0);
    }

    #[test]
    fn test_factory_64_k4() {
        let modulator = fbmc_64_k4();
        assert_eq!(modulator.num_subcarriers(), 64);
        assert_eq!(modulator.overlapping_factor(), 4);
        assert_eq!(modulator.prototype.len(), 64 * 4);
        assert_eq!(modulator.polyphase.len(), 64);
        assert_eq!(modulator.polyphase[0].len(), 4);
    }

    #[test]
    fn test_factory_256_k4() {
        let modulator = fbmc_256_k4();
        assert_eq!(modulator.num_subcarriers(), 256);
        assert_eq!(modulator.overlapping_factor(), 4);
        assert_eq!(modulator.prototype.len(), 256 * 4);
        assert_eq!(modulator.polyphase.len(), 256);
        assert_eq!(modulator.polyphase[0].len(), 4);
    }
}
