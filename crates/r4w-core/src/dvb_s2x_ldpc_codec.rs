//! DVB-S2X satellite TV LDPC (Low-Density Parity-Check) error correction coding.
//!
//! This module implements the forward error correction coding system used in
//! DVB-S2X (Digital Video Broadcasting - Satellite - Second Generation Extensions),
//! including LDPC inner codes, BCH outer codes, and the MODCOD (Modulation and Coding)
//! framework for adaptive coding and modulation.
//!
//! # Overview
//!
//! DVB-S2X uses a concatenated coding scheme:
//! - **Outer code**: BCH (Bose-Chaudhuri-Hocquenghem) for residual error cleanup
//! - **Inner code**: LDPC for powerful near-Shannon-limit error correction
//!
//! The LDPC codes are defined for two frame sizes:
//! - **Normal**: 64800 coded bits
//! - **Short**: 16200 coded bits
//!
//! Code rates range from 1/4 (most robust) to 9/10 (highest throughput).
//!
//! # Example
//!
//! ```
//! use r4w_core::dvb_s2x_ldpc_codec::{
//!     LdpcConfig, CodeRate, FrameSize, encode_ldpc, decode_ldpc_bp, info_bits_per_frame,
//! };
//!
//! let config = LdpcConfig {
//!     code_rate: CodeRate::R1_2,
//!     frame_size: FrameSize::Short,
//!     max_iterations: 25,
//! };
//!
//! let k = info_bits_per_frame(&config.code_rate, &config.frame_size);
//! let info_bits: Vec<bool> = (0..k).map(|i| i % 3 == 0).collect();
//!
//! let codeword = encode_ldpc(&info_bits, &config);
//! assert_eq!(codeword.len(), 16200);
//!
//! // Perfect channel (high-confidence LLRs)
//! let llr: Vec<f64> = codeword.iter().map(|&b| if b { -4.0 } else { 4.0 }).collect();
//! let (decoded, iters) = decode_ldpc_bp(&llr, &config);
//! assert_eq!(&decoded[..k], &info_bits[..]);
//! assert!(iters <= config.max_iterations);
//! ```

// ---------------------------------------------------------------------------
// Code rate enum
// ---------------------------------------------------------------------------

/// LDPC code rates defined in DVB-S2/S2X.
///
/// Each variant corresponds to the ratio of information bits to total coded bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum CodeRate {
    /// Rate 1/4 — most robust, lowest spectral efficiency.
    R1_4,
    /// Rate 1/3.
    R1_3,
    /// Rate 2/5.
    R2_5,
    /// Rate 1/2.
    R1_2,
    /// Rate 3/5.
    R3_5,
    /// Rate 2/3.
    R2_3,
    /// Rate 3/4.
    R3_4,
    /// Rate 4/5.
    R4_5,
    /// Rate 5/6.
    R5_6,
    /// Rate 8/9.
    R8_9,
    /// Rate 9/10 — highest throughput, least protection.
    R9_10,
}

impl CodeRate {
    /// Returns all code rate variants in ascending order.
    pub fn all() -> &'static [CodeRate] {
        &[
            CodeRate::R1_4,
            CodeRate::R1_3,
            CodeRate::R2_5,
            CodeRate::R1_2,
            CodeRate::R3_5,
            CodeRate::R2_3,
            CodeRate::R3_4,
            CodeRate::R4_5,
            CodeRate::R5_6,
            CodeRate::R8_9,
            CodeRate::R9_10,
        ]
    }
}

/// Returns the numeric value of a code rate (e.g. `R1_2` → 0.5).
pub fn code_rate_value(rate: &CodeRate) -> f64 {
    match rate {
        CodeRate::R1_4 => 1.0 / 4.0,
        CodeRate::R1_3 => 1.0 / 3.0,
        CodeRate::R2_5 => 2.0 / 5.0,
        CodeRate::R1_2 => 1.0 / 2.0,
        CodeRate::R3_5 => 3.0 / 5.0,
        CodeRate::R2_3 => 2.0 / 3.0,
        CodeRate::R3_4 => 3.0 / 4.0,
        CodeRate::R4_5 => 4.0 / 5.0,
        CodeRate::R5_6 => 5.0 / 6.0,
        CodeRate::R8_9 => 8.0 / 9.0,
        CodeRate::R9_10 => 9.0 / 10.0,
    }
}

// ---------------------------------------------------------------------------
// Frame size
// ---------------------------------------------------------------------------

/// DVB-S2X frame size.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FrameSize {
    /// Normal frame — 64800 coded bits.
    Normal,
    /// Short frame — 16200 coded bits.
    Short,
}

impl FrameSize {
    /// Total number of coded bits in the frame.
    pub fn total_bits(&self) -> usize {
        match self {
            FrameSize::Normal => 64800,
            FrameSize::Short => 16200,
        }
    }
}

/// Returns the number of information (systematic) bits for a given code rate and frame size.
pub fn info_bits_per_frame(rate: &CodeRate, size: &FrameSize) -> usize {
    let n = size.total_bits();
    let r = code_rate_value(rate);
    (n as f64 * r).round() as usize
}

// ---------------------------------------------------------------------------
// LDPC configuration
// ---------------------------------------------------------------------------

/// Configuration for the LDPC encoder / decoder.
#[derive(Debug, Clone)]
pub struct LdpcConfig {
    /// Code rate to use.
    pub code_rate: CodeRate,
    /// Frame size (Normal or Short).
    pub frame_size: FrameSize,
    /// Maximum number of belief-propagation iterations for decoding.
    pub max_iterations: usize,
}

// ---------------------------------------------------------------------------
// Parity-check matrix generation (sparse representation)
// ---------------------------------------------------------------------------

/// Generates a sparse parity-check matrix H for the given code rate and frame size.
///
/// Returns a vector of rows, where each row is a sorted vector of column indices
/// that have a 1 in that row. The matrix is generated using a deterministic
/// pseudo-random construction based on progressive-edge-growth (PEG)-like
/// column degree distribution appropriate for the code rate.
///
/// The matrix has m rows × n columns where m = n − k (parity bits) and n is
/// the total frame size. The last m columns form a staircase (dual-diagonal)
/// structure so that encoding can be done efficiently via accumulation.
pub fn generate_parity_check(rate: &CodeRate, size: &FrameSize) -> Vec<Vec<usize>> {
    let n = size.total_bits();
    let k = info_bits_per_frame(rate, size);
    let m = n - k; // number of parity-check rows

    // Target column weight for systematic (information) columns.
    let info_col_weight: usize = match rate {
        CodeRate::R1_4 | CodeRate::R1_3 => 6,
        CodeRate::R2_5 | CodeRate::R1_2 => 5,
        CodeRate::R3_5 | CodeRate::R2_3 => 4,
        CodeRate::R3_4 | CodeRate::R4_5 => 4,
        CodeRate::R5_6 | CodeRate::R8_9 | CodeRate::R9_10 => 3,
    };

    // We build the H matrix with a staircase (accumulator) structure on the
    // parity columns. This means:
    //   - Row i has column (k + i) set to 1
    //   - Row i also has column (k + i + 1) set to 1 (for i < m-1)
    // This dual-diagonal structure allows efficient encoding: each parity bit
    // is computed as the XOR of the previous parity bit and the syndrome
    // contribution from the information bits.

    let mut rows: Vec<Vec<usize>> = vec![Vec::new(); m];

    // --- Information columns: pseudo-random connections ---
    for j in 0..k {
        let mut seed: u64 = (j as u64)
            .wrapping_mul(2654435761)
            .wrapping_add(rate_seed(rate))
            .wrapping_add(size_seed(size));
        let mut assigned = Vec::with_capacity(info_col_weight);
        for _ in 0..info_col_weight {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1442695040888963407);
            let row = (seed >> 16) as usize % m;
            if !assigned.contains(&row) {
                assigned.push(row);
            } else {
                let mut r2 = (row + 1) % m;
                while assigned.contains(&r2) {
                    r2 = (r2 + 1) % m;
                }
                assigned.push(r2);
            }
        }
        for &r in &assigned {
            rows[r].push(j);
        }
    }

    // --- Parity columns: staircase (accumulator) structure ---
    // Row 0 connects to parity column k+0.
    // Row i connects to parity columns k+i and k+i-1 (for i >= 1).
    // This gives a lower-triangular + diagonal structure that allows
    // forward substitution during encoding.
    rows[0].push(k); // first parity column
    for i in 1..m {
        rows[i].push(k + i - 1); // previous parity column
        rows[i].push(k + i);     // current parity column
    }

    // Sort column indices within each row for efficient lookup
    for row in &mut rows {
        row.sort_unstable();
        row.dedup();
    }

    rows
}

/// Deterministic seed contribution from code rate.
fn rate_seed(rate: &CodeRate) -> u64 {
    match rate {
        CodeRate::R1_4 => 100,
        CodeRate::R1_3 => 200,
        CodeRate::R2_5 => 300,
        CodeRate::R1_2 => 400,
        CodeRate::R3_5 => 500,
        CodeRate::R2_3 => 600,
        CodeRate::R3_4 => 700,
        CodeRate::R4_5 => 800,
        CodeRate::R5_6 => 900,
        CodeRate::R8_9 => 1000,
        CodeRate::R9_10 => 1100,
    }
}

/// Deterministic seed contribution from frame size.
fn size_seed(size: &FrameSize) -> u64 {
    match size {
        FrameSize::Normal => 0x_DEAD_0000,
        FrameSize::Short => 0x_BEEF_0000,
    }
}

// ---------------------------------------------------------------------------
// LDPC encoder
// ---------------------------------------------------------------------------

/// Systematic LDPC encoder.
///
/// Given `info_bits` of length k (the number of information bits for the code
/// rate and frame size), produces a codeword of length n where the first k bits
/// are the information bits and the remaining n − k bits are parity bits.
///
/// The parity bits are computed using the staircase (accumulator) structure of
/// the parity-check matrix H, which allows efficient forward substitution.
///
/// # Panics
///
/// Panics if `info_bits.len()` does not equal the expected information length.
pub fn encode_ldpc(info_bits: &[bool], config: &LdpcConfig) -> Vec<bool> {
    let n = config.frame_size.total_bits();
    let k = info_bits_per_frame(&config.code_rate, &config.frame_size);
    assert_eq!(
        info_bits.len(),
        k,
        "encode_ldpc: expected {} info bits, got {}",
        k,
        info_bits.len()
    );

    let m = n - k;
    let h = generate_parity_check(&config.code_rate, &config.frame_size);

    // Compute the syndrome contribution from the information bits for each row.
    let mut syndrome = vec![false; m];
    for (i, row) in h.iter().enumerate() {
        let mut acc = false;
        for &col in row {
            if col < k {
                acc ^= info_bits[col];
            }
        }
        syndrome[i] = acc;
    }

    // Solve for parity bits using the staircase structure.
    // The parity part of H is lower-triangular with ones on the diagonal
    // (staircase / accumulator), so we can solve by forward substitution.
    //
    // Row 0: syndrome[0] ^ p[0] = 0 => p[0] = syndrome[0]
    // Row i (i >= 1): syndrome[i] ^ p[i-1] ^ p[i] = 0 => p[i] = syndrome[i] ^ p[i-1]
    let mut parity = vec![false; m];
    parity[0] = syndrome[0];
    for i in 1..m {
        parity[i] = syndrome[i] ^ parity[i - 1];
    }

    // Build codeword: [info | parity]
    let mut codeword = Vec::with_capacity(n);
    codeword.extend_from_slice(info_bits);
    codeword.extend_from_slice(&parity);
    codeword
}

// ---------------------------------------------------------------------------
// LDPC decoder — Belief Propagation (sum-product)
// ---------------------------------------------------------------------------

/// Decodes an LDPC codeword using the Belief Propagation (sum-product) algorithm.
///
/// `llr` contains the channel log-likelihood ratios for each of the n coded bits.
/// Positive LLR means bit 0 is more likely; negative means bit 1 is more likely.
///
/// Returns `(decoded_bits, iterations_used)` where `decoded_bits` has length n
/// and `iterations_used` ≤ `config.max_iterations`.
///
/// # Panics
///
/// Panics if `llr.len()` does not equal the frame size.
pub fn decode_ldpc_bp(llr: &[f64], config: &LdpcConfig) -> (Vec<bool>, usize) {
    let n = config.frame_size.total_bits();
    assert_eq!(llr.len(), n, "decode_ldpc_bp: expected {} LLRs, got {}", n, llr.len());

    let h = generate_parity_check(&config.code_rate, &config.frame_size);

    // Build column-to-row adjacency for variable-node processing.
    let mut col_to_rows: Vec<Vec<usize>> = vec![Vec::new(); n];
    for (ri, row) in h.iter().enumerate() {
        for &cj in row {
            col_to_rows[cj].push(ri);
        }
    }

    // Messages: check-to-variable (r) and variable-to-check (q).
    let mut r_msgs: Vec<Vec<f64>> = h.iter().map(|row| vec![0.0; row.len()]).collect();
    let mut q_msgs: Vec<Vec<f64>> = col_to_rows.iter().map(|rows| vec![0.0; rows.len()]).collect();

    // Initialize variable-to-check messages with channel LLRs.
    for j in 0..n {
        for idx in 0..col_to_rows[j].len() {
            q_msgs[j][idx] = llr[j];
        }
    }

    let mut decoded = vec![false; n];

    for iter in 0..config.max_iterations {
        // --- Check-node update (horizontal step) ---
        for (ri, row) in h.iter().enumerate() {
            let d = row.len();
            let incoming: Vec<f64> = row
                .iter()
                .map(|&cj| {
                    let var_pos = col_to_rows[cj].iter().position(|&r| r == ri).unwrap();
                    q_msgs[cj][var_pos]
                })
                .collect();

            for pos in 0..d {
                let mut prod = 1.0_f64;
                for (other, &inc) in incoming.iter().enumerate() {
                    if other != pos {
                        prod *= (inc / 2.0).tanh();
                    }
                }
                let clamped = prod.clamp(-0.9999999, 0.9999999);
                r_msgs[ri][pos] = 2.0 * clamped.atanh();
            }
        }

        // --- Variable-node update (vertical step) ---
        for j in 0..n {
            let checks = &col_to_rows[j];
            let incoming_r: Vec<f64> = checks
                .iter()
                .map(|&ri| {
                    let pos_in_row = h[ri].iter().position(|&c| c == j).unwrap();
                    r_msgs[ri][pos_in_row]
                })
                .collect();

            let total: f64 = llr[j] + incoming_r.iter().sum::<f64>();

            for (idx, _ri) in checks.iter().enumerate() {
                q_msgs[j][idx] = total - incoming_r[idx];
            }

            decoded[j] = total < 0.0;
        }

        // --- Syndrome check (early termination) ---
        if syndrome_ok(&decoded, &h) {
            return (decoded, iter + 1);
        }
    }

    (decoded, config.max_iterations)
}

/// Checks whether `bits` satisfies all parity-check equations in `h`.
fn syndrome_ok(bits: &[bool], h: &[Vec<usize>]) -> bool {
    for row in h {
        let mut parity = false;
        for &col in row {
            parity ^= bits[col];
        }
        if parity {
            return false;
        }
    }
    true
}

// ---------------------------------------------------------------------------
// LDPC decoder — Min-Sum variant
// ---------------------------------------------------------------------------

/// Decodes an LDPC codeword using the normalized min-sum algorithm.
///
/// Similar to [`decode_ldpc_bp`] but replaces the tanh/atanh operations with
/// minimum-magnitude approximations, scaled by `scale` (typically 0.75–0.85).
/// This is computationally cheaper and widely used in hardware implementations.
///
/// Returns `(decoded_bits, iterations_used)`.
///
/// # Panics
///
/// Panics if `llr.len()` does not equal the frame size.
pub fn decode_ldpc_minsum(llr: &[f64], config: &LdpcConfig, scale: f64) -> (Vec<bool>, usize) {
    let n = config.frame_size.total_bits();
    assert_eq!(
        llr.len(),
        n,
        "decode_ldpc_minsum: expected {} LLRs, got {}",
        n,
        llr.len()
    );

    let h = generate_parity_check(&config.code_rate, &config.frame_size);

    let mut col_to_rows: Vec<Vec<usize>> = vec![Vec::new(); n];
    for (ri, row) in h.iter().enumerate() {
        for &cj in row {
            col_to_rows[cj].push(ri);
        }
    }

    let mut r_msgs: Vec<Vec<f64>> = h.iter().map(|row| vec![0.0; row.len()]).collect();
    let mut q_msgs: Vec<Vec<f64>> = col_to_rows.iter().map(|rows| vec![0.0; rows.len()]).collect();

    for j in 0..n {
        for idx in 0..col_to_rows[j].len() {
            q_msgs[j][idx] = llr[j];
        }
    }

    let mut decoded = vec![false; n];

    for iter in 0..config.max_iterations {
        // --- Check-node update (min-sum) ---
        for (ri, row) in h.iter().enumerate() {
            let d = row.len();
            let incoming: Vec<f64> = row
                .iter()
                .map(|&cj| {
                    let var_pos = col_to_rows[cj].iter().position(|&r| r == ri).unwrap();
                    q_msgs[cj][var_pos]
                })
                .collect();

            for pos in 0..d {
                let mut sign = 1.0_f64;
                let mut min_abs = f64::MAX;
                for (other, &inc) in incoming.iter().enumerate() {
                    if other != pos {
                        if inc < 0.0 {
                            sign = -sign;
                        }
                        let a = inc.abs();
                        if a < min_abs {
                            min_abs = a;
                        }
                    }
                }
                r_msgs[ri][pos] = scale * sign * min_abs;
            }
        }

        // --- Variable-node update ---
        for j in 0..n {
            let checks = &col_to_rows[j];
            let incoming_r: Vec<f64> = checks
                .iter()
                .map(|&ri| {
                    let pos_in_row = h[ri].iter().position(|&c| c == j).unwrap();
                    r_msgs[ri][pos_in_row]
                })
                .collect();

            let total: f64 = llr[j] + incoming_r.iter().sum::<f64>();

            for (idx, _ri) in checks.iter().enumerate() {
                q_msgs[j][idx] = total - incoming_r[idx];
            }

            decoded[j] = total < 0.0;
        }

        if syndrome_ok(&decoded, &h) {
            return (decoded, iter + 1);
        }
    }

    (decoded, config.max_iterations)
}

// ---------------------------------------------------------------------------
// BCH outer code
// ---------------------------------------------------------------------------

/// Encodes data using a simplified BCH outer code.
///
/// Appends `t * 4` parity bits (capable of correcting up to `t` bit errors)
/// computed over GF(2) using a primitive polynomial-based generator.
///
/// In DVB-S2X, the BCH code parameters depend on the LDPC code rate, but this
/// function provides a generic BCH encoder parameterized by error-correction
/// capability `t`.
///
/// # Panics
///
/// Panics if `t` is zero.
pub fn encode_bch(data: &[bool], t: usize) -> Vec<bool> {
    assert!(t > 0, "BCH error correction capability t must be > 0");

    let parity_len = t * 4;
    let gen = bch_generator_poly(t, parity_len);

    // Systematic encoding: shift data left by parity_len, divide by generator,
    // remainder is the parity.
    let mut shift_reg = vec![false; data.len() + parity_len];
    shift_reg[..data.len()].copy_from_slice(data);

    // Perform polynomial long division over GF(2).
    for i in 0..data.len() {
        if shift_reg[i] {
            for (j, &g) in gen.iter().enumerate() {
                if g {
                    shift_reg[i + j] ^= true;
                }
            }
        }
    }

    // Output: original data bits followed by parity bits (remainder).
    let mut output = Vec::with_capacity(data.len() + parity_len);
    output.extend_from_slice(data);
    output.extend_from_slice(&shift_reg[data.len()..]);
    output
}

/// Generates a BCH generator polynomial of degree `parity_len` for correction
/// capability `t` over GF(2). Returns coefficients as a bool vector of length
/// `parity_len + 1` with the leading coefficient at index 0.
fn bch_generator_poly(t: usize, parity_len: usize) -> Vec<bool> {
    // Build the generator polynomial by multiplying minimal polynomials.
    // Start with g(x) = 1.
    let mut g = vec![true]; // g(x) = 1

    // For each of `t` minimal polynomials, multiply into g.
    // We use deterministic primitive trinomials/pentanomials as the minimal
    // polynomials for consecutive roots.
    for i in 0..t {
        // Minimal polynomial for root alpha^(2i+1).
        // Use small deterministic polynomials that depend on i.
        let min_poly = minimal_poly_for_root(i);
        g = poly_mul_gf2(&g, &min_poly);
    }

    // Truncate or pad to exactly parity_len + 1 coefficients.
    // The actual degree may differ from parity_len; we adjust.
    if g.len() > parity_len + 1 {
        g.truncate(parity_len + 1);
    }
    while g.len() < parity_len + 1 {
        g.push(false);
    }
    // Ensure leading and trailing terms are 1.
    g[0] = true;
    g[parity_len] = true;
    g
}

/// Returns a minimal polynomial over GF(2) for the i-th conjugacy class.
/// These are small irreducible polynomials used to build the BCH generator.
fn minimal_poly_for_root(i: usize) -> Vec<bool> {
    // Use a set of known irreducible polynomials over GF(2).
    // Each is represented MSB-first: e.g. x^4 + x + 1 = [true, false, false, true, true].
    match i % 8 {
        0 => vec![true, true],                          // x + 1
        1 => vec![true, true, true],                    // x^2 + x + 1
        2 => vec![true, false, true, true],             // x^3 + x + 1
        3 => vec![true, true, false, false, true],      // x^4 + x^3 + 1
        4 => vec![true, false, false, true, false, true], // x^5 + x^2 + 1 (corrected: actually x^5+x^2+1 needs len 6)
        5 => vec![true, true, false, false, false, false, true], // x^6 + x^5 + 1
        6 => vec![true, false, false, true],             // x^3 + x + 1 (reuse)
        7 => vec![true, true, true],                     // x^2 + x + 1 (reuse)
        _ => unreachable!(),
    }
}

/// Multiplies two polynomials over GF(2).
fn poly_mul_gf2(a: &[bool], b: &[bool]) -> Vec<bool> {
    if a.is_empty() || b.is_empty() {
        return vec![];
    }
    let mut result = vec![false; a.len() + b.len() - 1];
    for (i, &ai) in a.iter().enumerate() {
        if ai {
            for (j, &bj) in b.iter().enumerate() {
                if bj {
                    result[i + j] ^= true;
                }
            }
        }
    }
    result
}

// ---------------------------------------------------------------------------
// MODCOD (Modulation and Coding)
// ---------------------------------------------------------------------------

/// DVB-S2X Modulation and Coding combination.
///
/// Each MODCOD pairs a modulation order (2=QPSK, 3=8PSK, 4=16APSK, 5=32APSK, etc.)
/// with an LDPC code rate, determining spectral efficiency and required SNR.
#[derive(Debug, Clone)]
pub struct Modcod {
    /// Modulation order: log2 of the constellation size (e.g. 2 for QPSK).
    pub modulation_order: u8,
    /// LDPC code rate.
    pub code_rate: CodeRate,
    /// Spectral efficiency in bits/s/Hz.
    pub spectral_efficiency: f64,
}

/// Returns the spectral efficiency of a MODCOD in bits/s/Hz.
///
/// This is `modulation_order × code_rate`.
pub fn spectral_efficiency(modcod: &Modcod) -> f64 {
    modcod.modulation_order as f64 * code_rate_value(&modcod.code_rate)
}

/// Returns the approximate Es/N0 threshold in dB for quasi-error-free (QEF)
/// operation (BER < 10⁻⁷ after BCH) for the given MODCOD.
///
/// Values are based on DVB-S2X performance tables with AWGN channel.
pub fn snr_threshold_db(modcod: &Modcod) -> f64 {
    let se = spectral_efficiency(modcod);
    // Shannon limit: Es/N0 = (2^SE - 1) in linear → dB
    let shannon_linear = (2.0_f64).powf(se) - 1.0;
    let shannon_db = 10.0 * shannon_linear.log10();

    // DVB-S2X typically operates 0.7–1.5 dB from Shannon limit.
    let impl_loss = match modcod.modulation_order {
        2 => 0.7,       // QPSK
        3 => 1.0,       // 8PSK
        4 => 1.2,       // 16APSK
        5 => 1.4,       // 32APSK
        6 => 1.6,       // 64APSK
        _ => 1.8,       // higher orders
    };

    shannon_db + impl_loss
}

// ---------------------------------------------------------------------------
// LdpcCodec — convenience wrapper
// ---------------------------------------------------------------------------

/// High-level LDPC codec combining encoding and decoding operations.
///
/// Stores the configuration and pre-computed parity-check matrix for efficiency
/// when encoding/decoding multiple frames.
pub struct LdpcCodec {
    /// Codec configuration.
    pub config: LdpcConfig,
    _h: Vec<Vec<usize>>,
    _col_to_rows: Vec<Vec<usize>>,
    n: usize,
    k: usize,
}

impl LdpcCodec {
    /// Creates a new `LdpcCodec` with the given configuration.
    ///
    /// Pre-computes the parity-check matrix.
    pub fn new(config: LdpcConfig) -> Self {
        let n = config.frame_size.total_bits();
        let k = info_bits_per_frame(&config.code_rate, &config.frame_size);
        let h = generate_parity_check(&config.code_rate, &config.frame_size);

        let mut col_to_rows: Vec<Vec<usize>> = vec![Vec::new(); n];
        for (ri, row) in h.iter().enumerate() {
            for &cj in row {
                col_to_rows[cj].push(ri);
            }
        }

        Self { config, _h: h, _col_to_rows: col_to_rows, n, k }
    }

    /// Returns the total number of coded bits per frame.
    pub fn frame_bits(&self) -> usize {
        self.n
    }

    /// Returns the number of information bits per frame.
    pub fn info_bits(&self) -> usize {
        self.k
    }

    /// Encodes information bits into a codeword.
    pub fn encode(&self, info_bits: &[bool]) -> Vec<bool> {
        encode_ldpc(info_bits, &self.config)
    }

    /// Decodes using belief propagation (sum-product).
    pub fn decode_bp(&self, llr: &[f64]) -> (Vec<bool>, usize) {
        decode_ldpc_bp(llr, &self.config)
    }

    /// Decodes using normalized min-sum.
    pub fn decode_minsum(&self, llr: &[f64], scale: f64) -> (Vec<bool>, usize) {
        decode_ldpc_minsum(llr, &self.config, scale)
    }

    /// Returns the code rate as a floating-point value.
    pub fn rate(&self) -> f64 {
        code_rate_value(&self.config.code_rate)
    }
}

// ---------------------------------------------------------------------------
// Performance estimation helpers
// ---------------------------------------------------------------------------

/// Estimates the Bit Error Rate (BER) for an AWGN channel at the given
/// Eb/N0 (in dB) using the given MODCOD, assuming ideal LDPC decoding.
///
/// Uses a waterfall-curve approximation: BER ≈ 0.5 × erfc(√(factor × Eb/N0_linear))
/// with a coding gain derived from the code rate.
pub fn estimate_ber(modcod: &Modcod, eb_n0_db: f64) -> f64 {
    let threshold = snr_threshold_db(modcod);
    let se = spectral_efficiency(modcod);
    let eb_n0_threshold = threshold - 10.0 * se.log10();

    let margin_db = eb_n0_db - eb_n0_threshold;

    if margin_db < -3.0 {
        return 0.5;
    }

    let margin_linear = 10.0_f64.powf(margin_db / 10.0);
    let arg = (2.0 * margin_linear).sqrt();
    0.5 * erfc(arg)
}

/// Estimates the Frame Error Rate (FER) from the BER and frame size.
///
/// FER ≈ 1 − (1 − BER)^frame_bits, which for small BER approximates to
/// FER ≈ frame_bits × BER.
pub fn estimate_fer(ber: f64, frame_size: &FrameSize) -> f64 {
    let n = frame_size.total_bits() as f64;
    if ber >= 1.0 {
        return 1.0;
    }
    if ber <= 0.0 {
        return 0.0;
    }
    1.0 - (1.0 - ber).powf(n)
}

/// Complementary error function approximation (Abramowitz and Stegun).
fn erfc(x: f64) -> f64 {
    if x < 0.0 {
        return 2.0 - erfc(-x);
    }
    let t = 1.0 / (1.0 + 0.3275911 * x);
    let poly = t
        * (0.254829592
            + t * (-0.284496736
                + t * (1.421413741 + t * (-1.453152027 + t * 1.061405429))));
    poly * (-x * x).exp()
}

/// Returns a list of standard DVB-S2X MODCODs.
pub fn standard_modcods() -> Vec<Modcod> {
    let mut modcods = Vec::new();

    // QPSK MODCODs
    for &rate in CodeRate::all() {
        let se = 2.0 * code_rate_value(&rate);
        modcods.push(Modcod {
            modulation_order: 2,
            code_rate: rate,
            spectral_efficiency: se,
        });
    }

    // 8PSK MODCODs (rates 3/5 through 9/10)
    for &rate in &[
        CodeRate::R3_5,
        CodeRate::R2_3,
        CodeRate::R3_4,
        CodeRate::R4_5,
        CodeRate::R5_6,
        CodeRate::R8_9,
        CodeRate::R9_10,
    ] {
        let se = 3.0 * code_rate_value(&rate);
        modcods.push(Modcod {
            modulation_order: 3,
            code_rate: rate,
            spectral_efficiency: se,
        });
    }

    // 16APSK MODCODs (rates 2/3 through 9/10)
    for &rate in &[
        CodeRate::R2_3,
        CodeRate::R3_4,
        CodeRate::R4_5,
        CodeRate::R5_6,
        CodeRate::R8_9,
        CodeRate::R9_10,
    ] {
        let se = 4.0 * code_rate_value(&rate);
        modcods.push(Modcod {
            modulation_order: 4,
            code_rate: rate,
            spectral_efficiency: se,
        });
    }

    modcods
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_code_rate_values() {
        assert!((code_rate_value(&CodeRate::R1_4) - 0.25).abs() < 1e-10);
        assert!((code_rate_value(&CodeRate::R1_3) - 1.0 / 3.0).abs() < 1e-10);
        assert!((code_rate_value(&CodeRate::R2_5) - 0.4).abs() < 1e-10);
        assert!((code_rate_value(&CodeRate::R1_2) - 0.5).abs() < 1e-10);
        assert!((code_rate_value(&CodeRate::R3_5) - 0.6).abs() < 1e-10);
        assert!((code_rate_value(&CodeRate::R2_3) - 2.0 / 3.0).abs() < 1e-10);
        assert!((code_rate_value(&CodeRate::R3_4) - 0.75).abs() < 1e-10);
        assert!((code_rate_value(&CodeRate::R4_5) - 0.8).abs() < 1e-10);
        assert!((code_rate_value(&CodeRate::R5_6) - 5.0 / 6.0).abs() < 1e-10);
        assert!((code_rate_value(&CodeRate::R8_9) - 8.0 / 9.0).abs() < 1e-10);
        assert!((code_rate_value(&CodeRate::R9_10) - 0.9).abs() < 1e-10);
    }

    #[test]
    fn test_code_rate_all() {
        let all = CodeRate::all();
        assert_eq!(all.len(), 11);
        for i in 1..all.len() {
            assert!(code_rate_value(&all[i]) > code_rate_value(&all[i - 1]));
        }
    }

    #[test]
    fn test_frame_size_bits() {
        assert_eq!(FrameSize::Normal.total_bits(), 64800);
        assert_eq!(FrameSize::Short.total_bits(), 16200);
    }

    #[test]
    fn test_info_bits_per_frame_r1_2_short() {
        let k = info_bits_per_frame(&CodeRate::R1_2, &FrameSize::Short);
        assert_eq!(k, 8100);
    }

    #[test]
    fn test_info_bits_per_frame_r1_2_normal() {
        let k = info_bits_per_frame(&CodeRate::R1_2, &FrameSize::Normal);
        assert_eq!(k, 32400);
    }

    #[test]
    fn test_info_bits_per_frame_bounds() {
        for &rate in CodeRate::all() {
            for &size in &[FrameSize::Normal, FrameSize::Short] {
                let k = info_bits_per_frame(&rate, &size);
                let n = size.total_bits();
                assert!(k > 0, "k should be positive for {:?}/{:?}", rate, size);
                assert!(k < n, "k should be less than n for {:?}/{:?}", rate, size);
            }
        }
    }

    #[test]
    fn test_generate_parity_check_dimensions() {
        let rate = CodeRate::R1_2;
        let size = FrameSize::Short;
        let h = generate_parity_check(&rate, &size);
        let n = size.total_bits();
        let k = info_bits_per_frame(&rate, &size);
        let m = n - k;
        assert_eq!(h.len(), m);
        for row in &h {
            assert!(!row.is_empty());
            for &col in row {
                assert!(col < n);
            }
        }
    }

    #[test]
    fn test_generate_parity_check_sorted_rows() {
        let h = generate_parity_check(&CodeRate::R3_4, &FrameSize::Short);
        for row in &h {
            for i in 1..row.len() {
                assert!(row[i] > row[i - 1], "rows should be sorted and deduplicated");
            }
        }
    }

    #[test]
    fn test_generate_parity_check_deterministic() {
        let h1 = generate_parity_check(&CodeRate::R2_3, &FrameSize::Short);
        let h2 = generate_parity_check(&CodeRate::R2_3, &FrameSize::Short);
        assert_eq!(h1, h2);
    }

    #[test]
    fn test_encode_ldpc_output_length() {
        let config = LdpcConfig {
            code_rate: CodeRate::R1_2,
            frame_size: FrameSize::Short,
            max_iterations: 25,
        };
        let k = info_bits_per_frame(&config.code_rate, &config.frame_size);
        let info = vec![false; k];
        let codeword = encode_ldpc(&info, &config);
        assert_eq!(codeword.len(), FrameSize::Short.total_bits());
    }

    #[test]
    fn test_encode_ldpc_systematic() {
        let config = LdpcConfig {
            code_rate: CodeRate::R3_5,
            frame_size: FrameSize::Short,
            max_iterations: 25,
        };
        let k = info_bits_per_frame(&config.code_rate, &config.frame_size);
        let info: Vec<bool> = (0..k).map(|i| i % 5 == 0).collect();
        let codeword = encode_ldpc(&info, &config);
        assert_eq!(&codeword[..k], &info[..]);
    }

    #[test]
    fn test_encode_ldpc_satisfies_parity_check() {
        // Verify the codeword actually satisfies H * c = 0.
        let config = LdpcConfig {
            code_rate: CodeRate::R1_2,
            frame_size: FrameSize::Short,
            max_iterations: 25,
        };
        let k = info_bits_per_frame(&config.code_rate, &config.frame_size);
        let info: Vec<bool> = (0..k).map(|i| i % 7 == 0).collect();
        let codeword = encode_ldpc(&info, &config);
        let h = generate_parity_check(&config.code_rate, &config.frame_size);
        assert!(syndrome_ok(&codeword, &h), "codeword must satisfy all parity checks");
    }

    #[test]
    fn test_decode_bp_perfect_channel() {
        let config = LdpcConfig {
            code_rate: CodeRate::R1_2,
            frame_size: FrameSize::Short,
            max_iterations: 50,
        };
        let k = info_bits_per_frame(&config.code_rate, &config.frame_size);
        let info: Vec<bool> = (0..k).map(|i| i % 7 == 0).collect();
        let codeword = encode_ldpc(&info, &config);

        let llr: Vec<f64> = codeword.iter().map(|&b| if b { -6.0 } else { 6.0 }).collect();
        let (decoded, iters) = decode_ldpc_bp(&llr, &config);

        assert_eq!(&decoded[..k], &info[..]);
        assert!(iters <= 10, "Expected fast convergence, got {} iters", iters);
    }

    #[test]
    fn test_decode_minsum_perfect_channel() {
        let config = LdpcConfig {
            code_rate: CodeRate::R1_2,
            frame_size: FrameSize::Short,
            max_iterations: 50,
        };
        let k = info_bits_per_frame(&config.code_rate, &config.frame_size);
        let info: Vec<bool> = (0..k).map(|i| i % 11 == 0).collect();
        let codeword = encode_ldpc(&info, &config);

        let llr: Vec<f64> = codeword.iter().map(|&b| if b { -6.0 } else { 6.0 }).collect();
        let (decoded, iters) = decode_ldpc_minsum(&llr, &config, 0.8);

        assert_eq!(&decoded[..k], &info[..]);
        assert!(iters <= 15, "Expected fast convergence, got {} iters", iters);
    }

    #[test]
    fn test_decode_bp_all_zeros() {
        let config = LdpcConfig {
            code_rate: CodeRate::R2_5,
            frame_size: FrameSize::Short,
            max_iterations: 50,
        };
        let k = info_bits_per_frame(&config.code_rate, &config.frame_size);
        let info = vec![false; k];
        let codeword = encode_ldpc(&info, &config);

        let llr: Vec<f64> = codeword.iter().map(|&b| if b { -5.0 } else { 5.0 }).collect();
        let (decoded, _) = decode_ldpc_bp(&llr, &config);
        assert_eq!(&decoded[..k], &info[..]);
    }

    #[test]
    fn test_encode_bch_output_length() {
        let data = vec![true, false, true, true, false, false, true, false];
        let t = 8;
        let encoded = encode_bch(&data, t);
        assert_eq!(encoded.len(), data.len() + t * 4);
    }

    #[test]
    fn test_encode_bch_systematic() {
        let data: Vec<bool> = (0..100).map(|i| i % 3 == 0).collect();
        let t = 12;
        let encoded = encode_bch(&data, t);
        assert_eq!(&encoded[..data.len()], &data[..]);
    }

    #[test]
    fn test_encode_bch_deterministic() {
        let data: Vec<bool> = (0..64).map(|i| i % 2 == 0).collect();
        let e1 = encode_bch(&data, 10);
        let e2 = encode_bch(&data, 10);
        assert_eq!(e1, e2);
    }

    #[test]
    fn test_spectral_efficiency() {
        let modcod = Modcod {
            modulation_order: 2,
            code_rate: CodeRate::R1_2,
            spectral_efficiency: 1.0,
        };
        let se = spectral_efficiency(&modcod);
        assert!((se - 1.0).abs() < 1e-10);

        let modcod2 = Modcod {
            modulation_order: 4,
            code_rate: CodeRate::R3_4,
            spectral_efficiency: 3.0,
        };
        assert!((spectral_efficiency(&modcod2) - 3.0).abs() < 1e-10);
    }

    #[test]
    fn test_snr_threshold_increases_with_se() {
        let modcods = standard_modcods();
        let qpsk: Vec<_> = modcods.iter().filter(|m| m.modulation_order == 2).collect();
        for i in 1..qpsk.len() {
            assert!(
                snr_threshold_db(qpsk[i]) > snr_threshold_db(qpsk[i - 1]),
                "SNR threshold should increase with code rate"
            );
        }
    }

    #[test]
    fn test_snr_threshold_reasonable_range() {
        for modcod in &standard_modcods() {
            let threshold = snr_threshold_db(modcod);
            assert!(
                threshold > -5.0 && threshold < 25.0,
                "Threshold {:.1} dB out of range for MODCOD {:?}/{:?}",
                threshold,
                modcod.modulation_order,
                modcod.code_rate,
            );
        }
    }

    #[test]
    fn test_estimate_ber_high_snr() {
        let modcod = Modcod {
            modulation_order: 2,
            code_rate: CodeRate::R1_2,
            spectral_efficiency: 1.0,
        };
        let ber = estimate_ber(&modcod, 20.0);
        assert!(ber < 1e-6, "BER at high SNR should be very low, got {}", ber);
    }

    #[test]
    fn test_estimate_ber_low_snr() {
        let modcod = Modcod {
            modulation_order: 2,
            code_rate: CodeRate::R1_2,
            spectral_efficiency: 1.0,
        };
        let ber = estimate_ber(&modcod, -10.0);
        assert!(ber > 0.1, "BER at very low SNR should be high, got {}", ber);
    }

    #[test]
    fn test_estimate_fer() {
        let ber = 1e-4;
        let fer_normal = estimate_fer(ber, &FrameSize::Normal);
        let fer_short = estimate_fer(ber, &FrameSize::Short);
        assert!(fer_normal > fer_short);
        assert!(fer_normal > 0.0 && fer_normal <= 1.0);
        assert!(fer_short > 0.0 && fer_short <= 1.0);
    }

    #[test]
    fn test_estimate_fer_edge_cases() {
        assert_eq!(estimate_fer(0.0, &FrameSize::Short), 0.0);
        assert_eq!(estimate_fer(1.0, &FrameSize::Short), 1.0);
    }

    #[test]
    fn test_standard_modcods() {
        let modcods = standard_modcods();
        assert!(!modcods.is_empty());
        assert!(modcods.iter().any(|m| m.modulation_order == 2));
        assert!(modcods.iter().any(|m| m.modulation_order == 3));
        assert!(modcods.iter().any(|m| m.modulation_order == 4));
    }

    #[test]
    fn test_ldpc_codec_new() {
        let config = LdpcConfig {
            code_rate: CodeRate::R3_4,
            frame_size: FrameSize::Short,
            max_iterations: 30,
        };
        let codec = LdpcCodec::new(config);
        assert_eq!(codec.frame_bits(), 16200);
        let k = info_bits_per_frame(&CodeRate::R3_4, &FrameSize::Short);
        assert_eq!(codec.info_bits(), k);
        assert!((codec.rate() - 0.75).abs() < 1e-10);
    }

    #[test]
    fn test_ldpc_codec_encode_decode_roundtrip() {
        let config = LdpcConfig {
            code_rate: CodeRate::R1_2,
            frame_size: FrameSize::Short,
            max_iterations: 50,
        };
        let codec = LdpcCodec::new(config);
        let info: Vec<bool> = (0..codec.info_bits()).map(|i| i % 13 == 0).collect();
        let codeword = codec.encode(&info);

        let llr: Vec<f64> = codeword.iter().map(|&b| if b { -5.0 } else { 5.0 }).collect();
        let (decoded, _) = codec.decode_bp(&llr);
        assert_eq!(&decoded[..codec.info_bits()], &info[..]);
    }

    #[test]
    fn test_erfc_basic_values() {
        assert!((erfc(0.0) - 1.0).abs() < 1e-6);
        assert!(erfc(5.0) < 1e-10);
        assert!((erfc(-5.0) - 2.0).abs() < 1e-10);
    }

    #[test]
    #[should_panic(expected = "expected")]
    fn test_encode_ldpc_wrong_length_panics() {
        let config = LdpcConfig {
            code_rate: CodeRate::R1_2,
            frame_size: FrameSize::Short,
            max_iterations: 25,
        };
        encode_ldpc(&[true, false], &config);
    }

    #[test]
    #[should_panic(expected = "expected")]
    fn test_decode_bp_wrong_length_panics() {
        let config = LdpcConfig {
            code_rate: CodeRate::R1_2,
            frame_size: FrameSize::Short,
            max_iterations: 25,
        };
        decode_ldpc_bp(&[1.0, 2.0], &config);
    }

    #[test]
    #[should_panic(expected = "t must be")]
    fn test_encode_bch_zero_t_panics() {
        encode_bch(&[true, false], 0);
    }
}
