//! Constellation Encoder — Gray-coded bit-to-symbol mapping for PSK/QAM
//!
//! Maps bit sequences to constellation symbols with Gray coding for
//! BPSK, QPSK, 8PSK, 16QAM, and 64QAM modulation schemes. This is
//! the TX-side counterpart to [`crate::constellation_demapper`].
//!
//! Supports configurable symbol energy normalization (unit average energy
//! by default) and batch encoding of bit vectors to symbol vectors.
//!
//! Uses `(f64, f64)` for complex I/Q representation (no external crates).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::constellation_encoder::{ConstellationEncoder, Modulation};
//!
//! let enc = ConstellationEncoder::new(Modulation::Qpsk);
//! assert_eq!(enc.bits_per_symbol(), 2);
//!
//! // Encode 4 bits into 2 QPSK symbols
//! let symbols = enc.encode(&[false, false, true, true]);
//! assert_eq!(symbols.len(), 2);
//!
//! // First symbol is (1/sqrt2, 1/sqrt2) for bits [0,0]
//! let tol = 1e-9;
//! let s = 1.0_f64 / 2.0_f64.sqrt();
//! assert!((symbols[0].0 - s).abs() < tol);
//! assert!((symbols[0].1 - s).abs() < tol);
//! ```

use std::f64::consts::PI;

/// Supported modulation schemes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Modulation {
    /// Binary Phase-Shift Keying (1 bit/symbol)
    Bpsk,
    /// Quadrature Phase-Shift Keying (2 bits/symbol)
    Qpsk,
    /// 8-ary Phase-Shift Keying (3 bits/symbol)
    Psk8,
    /// 16-ary Quadrature Amplitude Modulation (4 bits/symbol)
    Qam16,
    /// 64-ary Quadrature Amplitude Modulation (6 bits/symbol)
    Qam64,
}

/// A constellation encoder that maps bit groups to I/Q symbols.
///
/// The symbol table is pre-computed at construction time with Gray-coded
/// ordering and optional energy normalization.
#[derive(Debug, Clone)]
pub struct ConstellationEncoder {
    modulation: Modulation,
    bits_per_symbol: usize,
    /// Lookup table indexed by Gray-coded integer -> (re, im).
    table: Vec<(f64, f64)>,
    /// Whether average symbol energy is normalized to 1.0.
    normalized: bool,
}

impl ConstellationEncoder {
    /// Create a new encoder for the given modulation with unit-energy normalization.
    pub fn new(modulation: Modulation) -> Self {
        Self::with_normalization(modulation, true)
    }

    /// Create a new encoder with explicit normalization control.
    ///
    /// When `normalize` is true, symbols are scaled so the average energy
    /// across the constellation is 1.0. When false, raw grid/circle
    /// coordinates are used.
    pub fn with_normalization(modulation: Modulation, normalize: bool) -> Self {
        let bits_per_symbol = modulation.bits_per_symbol();
        let table = build_table(modulation, normalize);
        Self {
            modulation,
            bits_per_symbol,
            table,
            normalized: normalize,
        }
    }

    /// Number of bits consumed per output symbol.
    pub fn bits_per_symbol(&self) -> usize {
        self.bits_per_symbol
    }

    /// The modulation scheme in use.
    pub fn modulation(&self) -> Modulation {
        self.modulation
    }

    /// Whether symbol energy normalization is enabled.
    pub fn is_normalized(&self) -> bool {
        self.normalized
    }

    /// Return a reference to the full symbol lookup table.
    ///
    /// The table is indexed by the integer formed from each bit group
    /// (MSB first). For example, bits `[true, false]` maps to index 2.
    pub fn symbol_table(&self) -> &[(f64, f64)] {
        &self.table
    }

    /// Encode a bit slice into constellation symbols.
    ///
    /// The input length must be a multiple of `bits_per_symbol()`.
    /// Returns one `(re, im)` symbol per bit group.
    ///
    /// # Panics
    ///
    /// Panics if `bits.len()` is not a multiple of `bits_per_symbol`.
    pub fn encode(&self, bits: &[bool]) -> Vec<(f64, f64)> {
        let bps = self.bits_per_symbol;
        assert!(
            bits.len() % bps == 0,
            "Input length {} is not a multiple of bits_per_symbol {}",
            bits.len(),
            bps
        );
        let n_symbols = bits.len() / bps;
        let mut out = Vec::with_capacity(n_symbols);
        for i in 0..n_symbols {
            let chunk = &bits[i * bps..(i + 1) * bps];
            let idx = bits_to_index(chunk);
            out.push(self.table[idx]);
        }
        out
    }

    /// Encode a single group of bits into one symbol.
    ///
    /// # Panics
    ///
    /// Panics if `bits.len() != bits_per_symbol()`.
    pub fn encode_symbol(&self, bits: &[bool]) -> (f64, f64) {
        assert_eq!(
            bits.len(),
            self.bits_per_symbol,
            "Expected {} bits, got {}",
            self.bits_per_symbol,
            bits.len()
        );
        let idx = bits_to_index(bits);
        self.table[idx]
    }
}

impl Modulation {
    /// Number of bits per symbol for this modulation.
    pub fn bits_per_symbol(self) -> usize {
        match self {
            Modulation::Bpsk => 1,
            Modulation::Qpsk => 2,
            Modulation::Psk8 => 3,
            Modulation::Qam16 => 4,
            Modulation::Qam64 => 6,
        }
    }

    /// Number of constellation points (2^bits_per_symbol).
    pub fn order(self) -> usize {
        1 << self.bits_per_symbol()
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

/// Convert a bit slice (MSB first) to an integer index.
fn bits_to_index(bits: &[bool]) -> usize {
    let mut idx = 0usize;
    for &b in bits {
        idx = (idx << 1) | (b as usize);
    }
    idx
}

/// Standard binary-to-Gray code conversion.
fn binary_to_gray(n: usize) -> usize {
    n ^ (n >> 1)
}

/// Build the constellation lookup table for the given modulation.
///
/// The table is indexed by the *natural binary* integer formed from
/// the input bit group. Internally, Gray coding maps that index to
/// the appropriate constellation point.
fn build_table(modulation: Modulation, normalize: bool) -> Vec<(f64, f64)> {
    let order = modulation.order();
    let mut table = vec![(0.0, 0.0); order];

    match modulation {
        Modulation::Bpsk => {
            // Gray: 0 -> +1, 1 -> -1
            table[0] = (1.0, 0.0);
            table[1] = (-1.0, 0.0);
            // BPSK already has unit energy; normalization is a no-op.
        }
        Modulation::Qpsk => {
            // 4 points on the unit circle at 45 deg, 135 deg, 225 deg, 315 deg.
            // Gray mapping: index -> gray -> quadrant.
            //   00 -> 0 -> (+,+)
            //   01 -> 1 -> (+,-)
            //   10 -> 3 -> (-,-)
            //   11 -> 2 -> (-,+)
            let s = 1.0; // will normalize below
            let raw = [(s, s), (s, -s), (-s, -s), (-s, s)]; // gray order 0,1,2,3
            for idx in 0..order {
                let g = binary_to_gray(idx);
                table[idx] = raw[g];
            }
            if normalize {
                // Each point has energy 2*s^2; scale so avg energy = 1.
                let scale = 1.0 / (2.0_f64).sqrt();
                for pt in &mut table {
                    pt.0 *= scale;
                    pt.1 *= scale;
                }
            }
        }
        Modulation::Psk8 => {
            // 8 equally-spaced points on the unit circle.
            // Gray coding: index -> gray -> angle = gray * 2*pi/8.
            for idx in 0..order {
                let g = binary_to_gray(idx);
                let angle = (g as f64) * 2.0 * PI / 8.0;
                table[idx] = (angle.cos(), angle.sin());
            }
            // Points are on the unit circle; already unit energy.
        }
        Modulation::Qam16 => {
            build_qam_table(&mut table, 4, normalize);
        }
        Modulation::Qam64 => {
            build_qam_table(&mut table, 8, normalize);
        }
    }

    table
}

/// Build a square QAM table with `side x side` points (side = 4 for 16QAM,
/// 8 for 64QAM). Uses independent Gray coding on I and Q axes.
fn build_qam_table(table: &mut [(f64, f64)], side: usize, normalize: bool) {
    let order = side * side;
    let bits_per_axis = match side {
        4 => 2,
        8 => 3,
        _ => panic!("unsupported QAM side {side}"),
    };

    // Build 1-D Gray-coded PAM levels for one axis.
    // Natural index i -> Gray(i) -> PAM level = 2*Gray(i) - (side-1).
    // This gives levels like -3, -1, +1, +3 for side=4.
    let mut pam_levels = vec![0.0_f64; side];
    for i in 0..side {
        let g = binary_to_gray(i);
        pam_levels[i] = 2.0 * (g as f64) - (side as f64 - 1.0);
    }

    // For each (I_bits, Q_bits) combination, the overall bit index is
    // (I_index << bits_per_axis) | Q_index, i.e., I bits are MSBs.
    for i_idx in 0..side {
        for q_idx in 0..side {
            let bit_idx = (i_idx << bits_per_axis) | q_idx;
            if bit_idx < order {
                table[bit_idx] = (pam_levels[i_idx], pam_levels[q_idx]);
            }
        }
    }

    if normalize {
        // Average energy = mean of (re^2 + im^2) over all points.
        let avg_energy: f64 = table[..order]
            .iter()
            .map(|(re, im)| re * re + im * im)
            .sum::<f64>()
            / order as f64;
        let scale = 1.0 / avg_energy.sqrt();
        for pt in table[..order].iter_mut() {
            pt.0 *= scale;
            pt.1 *= scale;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const TOL: f64 = 1e-9;

    fn approx_eq(a: f64, b: f64) -> bool {
        (a - b).abs() < TOL
    }

    fn energy(pt: (f64, f64)) -> f64 {
        pt.0 * pt.0 + pt.1 * pt.1
    }

    // ------------------------------------------------------------------
    // 1. bits_per_symbol accessor
    // ------------------------------------------------------------------
    #[test]
    fn test_bits_per_symbol() {
        assert_eq!(ConstellationEncoder::new(Modulation::Bpsk).bits_per_symbol(), 1);
        assert_eq!(ConstellationEncoder::new(Modulation::Qpsk).bits_per_symbol(), 2);
        assert_eq!(ConstellationEncoder::new(Modulation::Psk8).bits_per_symbol(), 3);
        assert_eq!(ConstellationEncoder::new(Modulation::Qam16).bits_per_symbol(), 4);
        assert_eq!(ConstellationEncoder::new(Modulation::Qam64).bits_per_symbol(), 6);
    }

    // ------------------------------------------------------------------
    // 2. Modulation::order
    // ------------------------------------------------------------------
    #[test]
    fn test_modulation_order() {
        assert_eq!(Modulation::Bpsk.order(), 2);
        assert_eq!(Modulation::Qpsk.order(), 4);
        assert_eq!(Modulation::Psk8.order(), 8);
        assert_eq!(Modulation::Qam16.order(), 16);
        assert_eq!(Modulation::Qam64.order(), 64);
    }

    // ------------------------------------------------------------------
    // 3. BPSK encoding
    // ------------------------------------------------------------------
    #[test]
    fn test_bpsk_encoding() {
        let enc = ConstellationEncoder::new(Modulation::Bpsk);
        let syms = enc.encode(&[false, true, false]);
        assert_eq!(syms.len(), 3);
        assert!(approx_eq(syms[0].0, 1.0) && approx_eq(syms[0].1, 0.0));
        assert!(approx_eq(syms[1].0, -1.0) && approx_eq(syms[1].1, 0.0));
        assert!(approx_eq(syms[2].0, 1.0) && approx_eq(syms[2].1, 0.0));
    }

    // ------------------------------------------------------------------
    // 4. QPSK encoding and Gray coding
    // ------------------------------------------------------------------
    #[test]
    fn test_qpsk_gray_coded() {
        let enc = ConstellationEncoder::new(Modulation::Qpsk);
        let s = 1.0 / 2.0_f64.sqrt();

        // bits 00 -> Gray(0)=0 -> (+s, +s)
        let sym = enc.encode_symbol(&[false, false]);
        assert!(approx_eq(sym.0, s) && approx_eq(sym.1, s));

        // bits 01 -> Gray(1)=1 -> (+s, -s)
        let sym = enc.encode_symbol(&[false, true]);
        assert!(approx_eq(sym.0, s) && approx_eq(sym.1, -s));

        // bits 11 -> Gray(3)=2 -> (-s, -s)
        let sym = enc.encode_symbol(&[true, true]);
        assert!(approx_eq(sym.0, -s) && approx_eq(sym.1, -s));

        // bits 10 -> Gray(2)=3 -> (-s, +s)
        let sym = enc.encode_symbol(&[true, false]);
        assert!(approx_eq(sym.0, -s) && approx_eq(sym.1, s));
    }

    // ------------------------------------------------------------------
    // 5. QPSK unit energy
    // ------------------------------------------------------------------
    #[test]
    fn test_qpsk_unit_energy() {
        let enc = ConstellationEncoder::new(Modulation::Qpsk);
        let avg: f64 = enc.symbol_table().iter().map(|&p| energy(p)).sum::<f64>()
            / enc.symbol_table().len() as f64;
        assert!(approx_eq(avg, 1.0));
    }

    // ------------------------------------------------------------------
    // 6. 8PSK unit energy and unique points
    // ------------------------------------------------------------------
    #[test]
    fn test_8psk_unit_energy_and_uniqueness() {
        let enc = ConstellationEncoder::new(Modulation::Psk8);
        let tbl = enc.symbol_table();
        assert_eq!(tbl.len(), 8);

        // All points on unit circle
        for &pt in tbl {
            assert!(approx_eq(energy(pt), 1.0));
        }

        // All points are distinct
        for i in 0..tbl.len() {
            for j in (i + 1)..tbl.len() {
                let d = ((tbl[i].0 - tbl[j].0).powi(2) + (tbl[i].1 - tbl[j].1).powi(2)).sqrt();
                assert!(d > 0.01, "points {i} and {j} are too close: {d}");
            }
        }
    }

    // ------------------------------------------------------------------
    // 7. 16QAM unit average energy
    // ------------------------------------------------------------------
    #[test]
    fn test_16qam_unit_avg_energy() {
        let enc = ConstellationEncoder::new(Modulation::Qam16);
        let tbl = enc.symbol_table();
        assert_eq!(tbl.len(), 16);
        let avg: f64 = tbl.iter().map(|&p| energy(p)).sum::<f64>() / 16.0;
        assert!(
            approx_eq(avg, 1.0),
            "16QAM average energy = {avg}, expected 1.0"
        );
    }

    // ------------------------------------------------------------------
    // 8. 64QAM unit average energy
    // ------------------------------------------------------------------
    #[test]
    fn test_64qam_unit_avg_energy() {
        let enc = ConstellationEncoder::new(Modulation::Qam64);
        let tbl = enc.symbol_table();
        assert_eq!(tbl.len(), 64);
        let avg: f64 = tbl.iter().map(|&p| energy(p)).sum::<f64>() / 64.0;
        assert!(
            approx_eq(avg, 1.0),
            "64QAM average energy = {avg}, expected 1.0"
        );
    }

    // ------------------------------------------------------------------
    // 9. QAM without normalization gives raw grid
    // ------------------------------------------------------------------
    #[test]
    fn test_16qam_unnormalized() {
        let enc = ConstellationEncoder::with_normalization(Modulation::Qam16, false);
        assert!(!enc.is_normalized());
        let tbl = enc.symbol_table();
        // Raw PAM levels for side=4 Gray-coded: indices 0..4 map through
        // Gray to levels {-3, -1, +1, +3}. Average energy != 1.
        let avg: f64 = tbl.iter().map(|&p| energy(p)).sum::<f64>() / 16.0;
        // Raw avg energy for {-3,-1,+1,+3}^2 grid = 10.0
        assert!(
            approx_eq(avg, 10.0),
            "raw 16QAM avg energy = {avg}, expected 10.0"
        );
    }

    // ------------------------------------------------------------------
    // 10. Gray code property: adjacent indices differ by 1 bit
    // ------------------------------------------------------------------
    #[test]
    fn test_gray_code_adjacency() {
        // For each axis of 16QAM, consecutive Gray codes differ by 1 bit.
        for n in 0..3usize {
            let g1 = binary_to_gray(n);
            let g2 = binary_to_gray(n + 1);
            let diff = g1 ^ g2;
            assert_eq!(
                diff.count_ones(),
                1,
                "Gray({n})={g1} and Gray({})={g2} differ by {} bits",
                n + 1,
                diff.count_ones()
            );
        }
    }

    // ------------------------------------------------------------------
    // 11. Batch encode length validation
    // ------------------------------------------------------------------
    #[test]
    #[should_panic(expected = "not a multiple")]
    fn test_encode_panics_on_bad_length() {
        let enc = ConstellationEncoder::new(Modulation::Qpsk);
        let _ = enc.encode(&[false, true, false]); // 3 bits, not multiple of 2
    }

    // ------------------------------------------------------------------
    // 12. Roundtrip: encode all possible bit patterns
    // ------------------------------------------------------------------
    #[test]
    fn test_all_patterns_unique() {
        // For each modulation, encoding all 2^bps patterns must yield
        // the same set of points as the symbol table (one-to-one).
        for modulation in &[
            Modulation::Bpsk,
            Modulation::Qpsk,
            Modulation::Psk8,
            Modulation::Qam16,
            Modulation::Qam64,
        ] {
            let enc = ConstellationEncoder::new(*modulation);
            let bps = enc.bits_per_symbol();
            let order = modulation.order();
            let mut symbols = Vec::with_capacity(order);
            for idx in 0..order {
                let bits: Vec<bool> = (0..bps)
                    .rev()
                    .map(|bit| (idx >> bit) & 1 == 1)
                    .collect();
                symbols.push(enc.encode_symbol(&bits));
            }
            // Each table entry must appear exactly once.
            for (i, &expected) in enc.symbol_table().iter().enumerate() {
                assert!(
                    approx_eq(symbols[i].0, expected.0) && approx_eq(symbols[i].1, expected.1),
                    "{:?} index {i}: got {:?}, expected {:?}",
                    modulation,
                    symbols[i],
                    expected
                );
            }
        }
    }

    // ------------------------------------------------------------------
    // 13. encode_symbol consistency with encode
    // ------------------------------------------------------------------
    #[test]
    fn test_encode_symbol_matches_batch() {
        let enc = ConstellationEncoder::new(Modulation::Qam16);
        let bits = vec![
            true, false, true, true, // symbol 0
            false, false, true, false, // symbol 1
        ];
        let batch = enc.encode(&bits);
        let s0 = enc.encode_symbol(&bits[0..4]);
        let s1 = enc.encode_symbol(&bits[4..8]);
        assert!(approx_eq(batch[0].0, s0.0) && approx_eq(batch[0].1, s0.1));
        assert!(approx_eq(batch[1].0, s1.0) && approx_eq(batch[1].1, s1.1));
    }
}
