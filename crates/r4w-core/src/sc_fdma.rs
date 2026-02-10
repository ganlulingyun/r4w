//! # SC-FDMA (Single Carrier Frequency Division Multiple Access)
//!
//! SC-FDMA is the uplink multiple access scheme used in LTE (4G). It is
//! essentially DFT-precoded OFDM: the transmitter applies a DFT to a block
//! of time-domain symbols before mapping them onto OFDM subcarriers. This
//! DFT-spreading step preserves the single-carrier structure of the signal,
//! yielding significantly lower Peak-to-Average Power Ratio (PAPR) compared
//! to plain OFDM -- a critical advantage for battery-powered user equipment.
//!
//! ## Signal Flow
//!
//! ```text
//! TX: symbols ─→ DFT(M) ─→ subcarrier mapping ─→ IFFT(N) ─→ add CP ─→ signal
//! RX: signal  ─→ remove CP ─→ FFT(N) ─→ extract subcarriers ─→ IDFT(M) ─→ symbols
//! ```
//!
//! Where M = number of allocated subcarriers, N = IFFT/FFT size.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::sc_fdma::{ScFdmaModulator, ScFdmaDemodulator};
//!
//! let fft_size = 64;
//! let num_sc = 12;
//! let offset = 4;
//! let cp_len = 16;
//!
//! let modulator = ScFdmaModulator::new(fft_size, num_sc, offset, cp_len);
//! let demodulator = ScFdmaDemodulator::new(fft_size, num_sc, offset, cp_len);
//!
//! // QPSK symbols (I, Q)
//! let symbols: Vec<(f64, f64)> = (0..num_sc)
//!     .map(|i| if i % 2 == 0 { (0.707, 0.707) } else { (-0.707, 0.707) })
//!     .collect();
//!
//! let signal = modulator.modulate(&symbols);
//! let recovered = demodulator.demodulate(&signal);
//!
//! // Recovered symbols approximate the originals
//! for (orig, rec) in symbols.iter().zip(recovered.iter()) {
//!     assert!((orig.0 - rec.0).abs() < 1e-9);
//!     assert!((orig.1 - rec.1).abs() < 1e-9);
//! }
//! ```

use std::f64::consts::PI;

/// SC-FDMA modulator (DFT-spread OFDM transmitter).
///
/// Implements the LTE uplink transmit chain:
/// DFT precoding -> subcarrier mapping -> IFFT -> cyclic prefix insertion.
#[derive(Debug, Clone)]
pub struct ScFdmaModulator {
    /// Total IFFT size (number of subcarriers including unoccupied).
    pub fft_size: usize,
    /// Number of allocated subcarriers (M, the DFT precoding size).
    pub num_subcarriers: usize,
    /// Starting index for subcarrier mapping within the IFFT bin array.
    pub subcarrier_offset: usize,
    /// Length of the cyclic prefix in samples.
    pub cp_length: usize,
}

impl ScFdmaModulator {
    /// Create a new SC-FDMA modulator.
    ///
    /// # Arguments
    ///
    /// * `fft_size` - Total IFFT size (N). Must be >= `num_subcarriers + subcarrier_offset`.
    /// * `num_subcarriers` - Number of allocated subcarriers (M).
    /// * `subcarrier_offset` - Starting subcarrier index for mapping.
    /// * `cp_length` - Cyclic prefix length in samples.
    pub fn new(
        fft_size: usize,
        num_subcarriers: usize,
        subcarrier_offset: usize,
        cp_length: usize,
    ) -> Self {
        assert!(
            num_subcarriers + subcarrier_offset <= fft_size,
            "subcarrier allocation exceeds FFT size"
        );
        Self {
            fft_size,
            num_subcarriers,
            subcarrier_offset,
            cp_length,
        }
    }

    /// Modulate a block of symbols using SC-FDMA.
    ///
    /// Steps:
    /// 1. DFT-precode the input symbols (M-point DFT)
    /// 2. Map precoded symbols to allocated subcarriers in an N-point grid
    /// 3. N-point IFFT to produce time-domain samples
    /// 4. Prepend cyclic prefix (last `cp_length` samples copied to front)
    ///
    /// # Arguments
    ///
    /// * `symbols` - Input symbols as (I, Q) tuples. Length must equal `num_subcarriers`.
    ///
    /// # Returns
    ///
    /// Time-domain signal with cyclic prefix: length = `fft_size + cp_length`.
    pub fn modulate(&self, symbols: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if symbols.is_empty() {
            return Vec::new();
        }
        assert_eq!(
            symbols.len(),
            self.num_subcarriers,
            "symbol count must equal num_subcarriers"
        );

        // Step 1: DFT precoding (M-point DFT)
        let precoded = dft_precoding(symbols);

        // Step 2: Map to subcarriers (zero-pad N-point grid)
        let mut freq_grid = vec![(0.0, 0.0); self.fft_size];
        for (i, &sym) in precoded.iter().enumerate() {
            freq_grid[self.subcarrier_offset + i] = sym;
        }

        // Step 3: IFFT
        let time_domain = simple_ifft(&freq_grid);

        // Step 4: Cyclic prefix (copy tail to front)
        let mut output = Vec::with_capacity(self.cp_length + self.fft_size);
        let cp_start = self.fft_size - self.cp_length;
        output.extend_from_slice(&time_domain[cp_start..]);
        output.extend_from_slice(&time_domain);

        output
    }
}

/// SC-FDMA demodulator (DFT-spread OFDM receiver).
///
/// Implements the LTE uplink receive chain:
/// cyclic prefix removal -> FFT -> subcarrier extraction -> IDFT decoding.
#[derive(Debug, Clone)]
pub struct ScFdmaDemodulator {
    /// Total FFT size.
    pub fft_size: usize,
    /// Number of allocated subcarriers (M).
    pub num_subcarriers: usize,
    /// Starting index for subcarrier extraction.
    pub subcarrier_offset: usize,
    /// Length of the cyclic prefix in samples.
    pub cp_length: usize,
}

impl ScFdmaDemodulator {
    /// Create a new SC-FDMA demodulator.
    ///
    /// Parameters must match the modulator configuration.
    pub fn new(
        fft_size: usize,
        num_subcarriers: usize,
        subcarrier_offset: usize,
        cp_length: usize,
    ) -> Self {
        assert!(
            num_subcarriers + subcarrier_offset <= fft_size,
            "subcarrier allocation exceeds FFT size"
        );
        Self {
            fft_size,
            num_subcarriers,
            subcarrier_offset,
            cp_length,
        }
    }

    /// Demodulate an SC-FDMA signal block.
    ///
    /// Steps:
    /// 1. Remove cyclic prefix
    /// 2. N-point FFT to recover frequency-domain grid
    /// 3. Extract allocated subcarriers
    /// 4. IDFT decoding (M-point IDFT) to recover original symbols
    ///
    /// # Arguments
    ///
    /// * `signal` - Received time-domain signal with CP. Length must equal
    ///   `fft_size + cp_length`.
    ///
    /// # Returns
    ///
    /// Recovered symbols as (I, Q) tuples. Length = `num_subcarriers`.
    pub fn demodulate(&self, signal: &[(f64, f64)]) -> Vec<(f64, f64)> {
        if signal.is_empty() {
            return Vec::new();
        }
        assert_eq!(
            signal.len(),
            self.fft_size + self.cp_length,
            "signal length must equal fft_size + cp_length"
        );

        // Step 1: Remove cyclic prefix
        let without_cp = &signal[self.cp_length..];

        // Step 2: FFT
        let freq_domain = simple_fft(without_cp);

        // Step 3: Extract allocated subcarriers
        let extracted: Vec<(f64, f64)> = freq_domain
            [self.subcarrier_offset..self.subcarrier_offset + self.num_subcarriers]
            .to_vec();

        // Step 4: IDFT decoding
        idft_decoding(&extracted)
    }
}

/// DFT precoding step (M-point DFT).
///
/// Transforms time-domain symbols into frequency-domain representation.
/// This is the key step that gives SC-FDMA its single-carrier (low PAPR)
/// property.
///
/// # Arguments
///
/// * `symbols` - Input symbols in time domain.
///
/// # Returns
///
/// DFT-precoded symbols in frequency domain.
pub fn dft_precoding(symbols: &[(f64, f64)]) -> Vec<(f64, f64)> {
    simple_fft(symbols)
}

/// IDFT decoding step (M-point IDFT).
///
/// Inverse of DFT precoding. Converts frequency-domain subcarrier values
/// back to time-domain symbols.
///
/// # Arguments
///
/// * `symbols` - Frequency-domain symbols extracted from subcarriers.
///
/// # Returns
///
/// Recovered time-domain symbols.
pub fn idft_decoding(symbols: &[(f64, f64)]) -> Vec<(f64, f64)> {
    simple_ifft(symbols)
}

/// Compute the Discrete Fourier Transform (DFT) of a complex sequence.
///
/// This is an O(N^2) direct implementation, suitable for the small block
/// sizes typical in SC-FDMA subcarrier allocation (e.g., 12, 24, 48).
///
/// X[k] = sum_{n=0}^{N-1} x[n] * exp(-j * 2pi * k * n / N)
///
/// # Arguments
///
/// * `input` - Complex input samples as (real, imag) tuples.
///
/// # Returns
///
/// DFT output of the same length.
pub fn simple_fft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    if n == 0 {
        return Vec::new();
    }

    let mut output = vec![(0.0, 0.0); n];
    for k in 0..n {
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        for (idx, &(re, im)) in input.iter().enumerate() {
            let angle = -2.0 * PI * (k as f64) * (idx as f64) / (n as f64);
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            // (re + j*im) * (cos_a + j*sin_a)
            sum_re += re * cos_a - im * sin_a;
            sum_im += re * sin_a + im * cos_a;
        }
        output[k] = (sum_re, sum_im);
    }
    output
}

/// Compute the Inverse Discrete Fourier Transform (IDFT) of a complex sequence.
///
/// This is an O(N^2) direct implementation.
///
/// x[n] = (1/N) * sum_{k=0}^{N-1} X[k] * exp(+j * 2pi * k * n / N)
///
/// # Arguments
///
/// * `input` - Complex frequency-domain samples as (real, imag) tuples.
///
/// # Returns
///
/// IDFT output of the same length.
pub fn simple_ifft(input: &[(f64, f64)]) -> Vec<(f64, f64)> {
    let n = input.len();
    if n == 0 {
        return Vec::new();
    }

    let mut output = vec![(0.0, 0.0); n];
    let inv_n = 1.0 / n as f64;
    for k in 0..n {
        let mut sum_re = 0.0;
        let mut sum_im = 0.0;
        for (idx, &(re, im)) in input.iter().enumerate() {
            let angle = 2.0 * PI * (k as f64) * (idx as f64) / (n as f64);
            let cos_a = angle.cos();
            let sin_a = angle.sin();
            sum_re += re * cos_a - im * sin_a;
            sum_im += re * sin_a + im * cos_a;
        }
        output[k] = (sum_re * inv_n, sum_im * inv_n);
    }
    output
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPS: f64 = 1e-9;

    fn approx_eq(a: (f64, f64), b: (f64, f64), tol: f64) -> bool {
        (a.0 - b.0).abs() < tol && (a.1 - b.1).abs() < tol
    }

    #[test]
    fn test_dft_precoding_roundtrip() {
        // DFT followed by IDFT should recover original symbols.
        let symbols: Vec<(f64, f64)> = vec![
            (1.0, 0.0),
            (0.0, 1.0),
            (-1.0, 0.0),
            (0.0, -1.0),
            (0.5, 0.5),
            (-0.5, 0.5),
        ];
        let precoded = dft_precoding(&symbols);
        let recovered = idft_decoding(&precoded);

        assert_eq!(recovered.len(), symbols.len());
        for (orig, rec) in symbols.iter().zip(recovered.iter()) {
            assert!(
                approx_eq(*orig, *rec, EPS),
                "mismatch: {:?} vs {:?}",
                orig,
                rec
            );
        }
    }

    #[test]
    fn test_modulate_demodulate() {
        // Full modulate -> demodulate roundtrip should recover symbols.
        let fft_size = 64;
        let num_sc = 12;
        let offset = 10;
        let cp_len = 16;

        let modulator = ScFdmaModulator::new(fft_size, num_sc, offset, cp_len);
        let demodulator = ScFdmaDemodulator::new(fft_size, num_sc, offset, cp_len);

        let symbols: Vec<(f64, f64)> = (0..num_sc)
            .map(|i| {
                let angle = 2.0 * PI * i as f64 / num_sc as f64;
                (angle.cos(), angle.sin())
            })
            .collect();

        let signal = modulator.modulate(&symbols);
        let recovered = demodulator.demodulate(&signal);

        assert_eq!(recovered.len(), num_sc);
        for (orig, rec) in symbols.iter().zip(recovered.iter()) {
            assert!(
                approx_eq(*orig, *rec, EPS),
                "mismatch: {:?} vs {:?}",
                orig,
                rec
            );
        }
    }

    #[test]
    fn test_cp_insertion() {
        // The cyclic prefix should be a copy of the last cp_length samples
        // of the IFFT output (i.e., signal[cp_len..cp_len+fft_size] tail
        // should match signal[0..cp_len]).
        let fft_size = 32;
        let num_sc = 8;
        let offset = 0;
        let cp_len = 8;

        let modulator = ScFdmaModulator::new(fft_size, num_sc, offset, cp_len);

        let symbols: Vec<(f64, f64)> = (0..num_sc)
            .map(|i| (i as f64 * 0.1, i as f64 * -0.1))
            .collect();

        let signal = modulator.modulate(&symbols);
        assert_eq!(signal.len(), fft_size + cp_len);

        // CP (first cp_len samples) should equal the last cp_len of the body
        let body = &signal[cp_len..];
        let cp = &signal[..cp_len];
        let body_tail = &body[fft_size - cp_len..];

        for (i, (c, t)) in cp.iter().zip(body_tail.iter()).enumerate() {
            assert!(
                approx_eq(*c, *t, EPS),
                "CP mismatch at index {}: {:?} vs {:?}",
                i,
                c,
                t
            );
        }
    }

    #[test]
    fn test_subcarrier_mapping() {
        // After DFT precoding, symbols should land at the correct subcarrier
        // positions in the frequency grid. We verify by doing a partial
        // modulate and checking the FFT output.
        let fft_size = 64;
        let num_sc = 4;
        let offset = 20;
        let cp_len = 8;

        let modulator = ScFdmaModulator::new(fft_size, num_sc, offset, cp_len);

        let symbols: Vec<(f64, f64)> = vec![
            (1.0, 0.0),
            (0.0, 1.0),
            (-1.0, 0.0),
            (0.0, -1.0),
        ];

        let signal = modulator.modulate(&symbols);

        // Demodulate: remove CP, FFT, check subcarrier positions
        let without_cp = &signal[cp_len..];
        let freq = simple_fft(without_cp);

        // Subcarriers outside the allocation should be (near) zero
        for (k, val) in freq.iter().enumerate() {
            if k < offset || k >= offset + num_sc {
                assert!(
                    val.0.abs() < EPS && val.1.abs() < EPS,
                    "non-zero energy at unallocated subcarrier {}: {:?}",
                    k,
                    val
                );
            }
        }

        // Subcarriers inside the allocation should have non-trivial energy
        let precoded = dft_precoding(&symbols);
        for (i, expected) in precoded.iter().enumerate() {
            let actual = freq[offset + i];
            assert!(
                approx_eq(actual, *expected, EPS),
                "subcarrier {} mismatch: {:?} vs {:?}",
                offset + i,
                actual,
                expected
            );
        }
    }

    #[test]
    fn test_papr_lower_than_ofdm() {
        // SC-FDMA should have lower PAPR than plain OFDM for the same symbols.
        // Plain OFDM: place symbols directly on subcarriers (no DFT precoding).
        let fft_size = 64;
        let num_sc = 12;
        let offset = 4;
        let cp_len = 16;

        // Random-ish QPSK-like symbols
        let symbols: Vec<(f64, f64)> = (0..num_sc)
            .map(|i| {
                let s = if i % 4 == 0 {
                    (0.707, 0.707)
                } else if i % 4 == 1 {
                    (-0.707, 0.707)
                } else if i % 4 == 2 {
                    (-0.707, -0.707)
                } else {
                    (0.707, -0.707)
                };
                s
            })
            .collect();

        // SC-FDMA signal (DFT-precoded)
        let sc_fdma_mod = ScFdmaModulator::new(fft_size, num_sc, offset, cp_len);
        let sc_fdma_signal = sc_fdma_mod.modulate(&symbols);
        let sc_fdma_papr = compute_papr(&sc_fdma_signal);

        // Plain OFDM signal (no DFT precoding)
        let mut freq_grid = vec![(0.0, 0.0); fft_size];
        for (i, &sym) in symbols.iter().enumerate() {
            freq_grid[offset + i] = sym;
        }
        let ofdm_time = simple_ifft(&freq_grid);
        let mut ofdm_signal = Vec::with_capacity(cp_len + fft_size);
        ofdm_signal.extend_from_slice(&ofdm_time[fft_size - cp_len..]);
        ofdm_signal.extend_from_slice(&ofdm_time);
        let ofdm_papr = compute_papr(&ofdm_signal);

        assert!(
            sc_fdma_papr < ofdm_papr,
            "SC-FDMA PAPR ({:.2} dB) should be lower than OFDM PAPR ({:.2} dB)",
            sc_fdma_papr,
            ofdm_papr
        );
    }

    #[test]
    fn test_fft_ifft_roundtrip() {
        // FFT followed by IFFT should recover the original signal.
        let input: Vec<(f64, f64)> = vec![
            (1.0, 2.0),
            (3.0, -1.0),
            (-2.0, 0.5),
            (0.0, 4.0),
            (2.5, -3.0),
            (-1.0, 1.0),
            (0.0, 0.0),
            (1.5, 2.5),
        ];

        let freq = simple_fft(&input);
        let recovered = simple_ifft(&freq);

        assert_eq!(recovered.len(), input.len());
        for (orig, rec) in input.iter().zip(recovered.iter()) {
            assert!(
                approx_eq(*orig, *rec, EPS),
                "FFT/IFFT roundtrip mismatch: {:?} vs {:?}",
                orig,
                rec
            );
        }
    }

    #[test]
    fn test_single_subcarrier() {
        // With a single subcarrier, SC-FDMA reduces to a simple tone.
        let fft_size = 16;
        let num_sc = 1;
        let offset = 5;
        let cp_len = 4;

        let modulator = ScFdmaModulator::new(fft_size, num_sc, offset, cp_len);
        let demodulator = ScFdmaDemodulator::new(fft_size, num_sc, offset, cp_len);

        let symbols = vec![(1.0, 0.0)];
        let signal = modulator.modulate(&symbols);
        assert_eq!(signal.len(), fft_size + cp_len);

        let recovered = demodulator.demodulate(&signal);
        assert_eq!(recovered.len(), 1);
        assert!(
            approx_eq(recovered[0], (1.0, 0.0), EPS),
            "single subcarrier mismatch: {:?}",
            recovered[0]
        );
    }

    #[test]
    fn test_full_allocation() {
        // All subcarriers allocated (num_subcarriers == fft_size).
        let fft_size = 16;
        let num_sc = 16;
        let offset = 0;
        let cp_len = 4;

        let modulator = ScFdmaModulator::new(fft_size, num_sc, offset, cp_len);
        let demodulator = ScFdmaDemodulator::new(fft_size, num_sc, offset, cp_len);

        let symbols: Vec<(f64, f64)> = (0..num_sc)
            .map(|i| ((i as f64).sin(), (i as f64).cos()))
            .collect();

        let signal = modulator.modulate(&symbols);
        assert_eq!(signal.len(), fft_size + cp_len);

        let recovered = demodulator.demodulate(&signal);
        assert_eq!(recovered.len(), num_sc);
        for (orig, rec) in symbols.iter().zip(recovered.iter()) {
            assert!(
                approx_eq(*orig, *rec, EPS),
                "full allocation mismatch: {:?} vs {:?}",
                orig,
                rec
            );
        }
    }

    #[test]
    fn test_symbol_preservation() {
        // Different symbol constellations should be preserved through
        // modulate/demodulate.
        let fft_size = 32;
        let num_sc = 6;
        let offset = 2;
        let cp_len = 8;

        let modulator = ScFdmaModulator::new(fft_size, num_sc, offset, cp_len);
        let demodulator = ScFdmaDemodulator::new(fft_size, num_sc, offset, cp_len);

        // Test with various symbol amplitudes and phases
        let test_cases: Vec<Vec<(f64, f64)>> = vec![
            // All same symbol
            vec![(1.0, 0.0); 6],
            // Alternating
            vec![
                (1.0, 0.0),
                (-1.0, 0.0),
                (1.0, 0.0),
                (-1.0, 0.0),
                (1.0, 0.0),
                (-1.0, 0.0),
            ],
            // 16-QAM-like
            vec![
                (3.0, 3.0),
                (3.0, 1.0),
                (1.0, 3.0),
                (1.0, 1.0),
                (-1.0, -1.0),
                (-3.0, -3.0),
            ],
        ];

        for symbols in &test_cases {
            let signal = modulator.modulate(symbols);
            let recovered = demodulator.demodulate(&signal);
            for (orig, rec) in symbols.iter().zip(recovered.iter()) {
                assert!(
                    approx_eq(*orig, *rec, EPS),
                    "symbol preservation failed: {:?} vs {:?}",
                    orig,
                    rec
                );
            }
        }
    }

    #[test]
    fn test_empty_input() {
        let modulator = ScFdmaModulator::new(64, 12, 4, 16);
        let demodulator = ScFdmaDemodulator::new(64, 12, 4, 16);

        let signal = modulator.modulate(&[]);
        assert!(signal.is_empty());

        let symbols = demodulator.demodulate(&[]);
        assert!(symbols.is_empty());
    }

    /// Compute PAPR in dB for a complex signal.
    fn compute_papr(signal: &[(f64, f64)]) -> f64 {
        if signal.is_empty() {
            return 0.0;
        }
        let powers: Vec<f64> = signal
            .iter()
            .map(|&(re, im)| re * re + im * im)
            .collect();
        let peak = powers.iter().cloned().fold(0.0_f64, f64::max);
        let avg = powers.iter().sum::<f64>() / powers.len() as f64;
        if avg < 1e-30 {
            return 0.0;
        }
        10.0 * (peak / avg).log10()
    }
}
