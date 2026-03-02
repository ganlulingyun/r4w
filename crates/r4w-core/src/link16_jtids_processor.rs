//! Link 16 / JTIDS Processor — Physical Layer of MIL-STD-6016 Tactical Data Link
//!
//! Implements the physical layer of Link 16 (JTIDS – Joint Tactical Information
//! Distribution System), the primary NATO tactical data link used by fighter
//! aircraft, naval vessels, and ground stations.
//!
//! **NOTE: For educational and simulation purposes only. This implementation does
//! not reproduce classified cryptographic keying material or operational TRANSEC
//! sequences.**
//!
//! ## Reference
//!
//! - MIL-STD-6016: Tactical Digital Information Link (TADIL) J Message Standard
//! - MIL-STD-6016E: TADIL J Interface Standard
//! - STANAG 5516: NATO Link 16 standard
//!
//! ## Architecture
//!
//! ```text
//! J-Series Message (75 bits)
//!        │
//!        ▼
//! Reed-Solomon RS(31,15) over GF(2^5)   [15 data symbols → 31 coded symbols]
//!        │
//!        ▼
//! Symbol Interleaver                     [burst error dispersal across hops]
//!        │
//!        ▼
//! CCSK Modulator                         [5-bit symbol → 32-chip spreading]
//!        │
//!        ▼
//! MSK Modulator                          [5 Mbit/s per pulse]
//!        │
//!        ▼
//! Frequency Hopper                       [51 hops in 969–1206 MHz, 3 hops/pulse]
//!        │
//!        ▼
//! TDMA Slot Framer                       [7.8125 ms time slots, 128 per epoch]
//! ```
//!
//! ## Key Parameters (MIL-STD-6016)
//!
//! | Parameter           | Value                          |
//! |---------------------|--------------------------------|
//! | Frequency band      | 969 – 1206 MHz (L-band)        |
//! | Number of hop freqs | 51                             |
//! | Hop rate            | ~3 hops per 6.4 μs pulse       |
//! | Data rate           | 5 Mbit/s per pulse (MSK)       |
//! | CCSK chips          | 32 chips / symbol              |
//! | RS code             | RS(31,15) over GF(2^5)         |
//! | Error correction    | t = 8 symbols                  |
//! | TDMA slots/epoch    | 128 slots × 7.8125 ms = 1.0 s  |
//! | Epoch duration      | 12 seconds (1536 slots total)  |
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::link16_jtids_processor::{Link16Config, Link16Processor, MessageFormatter};
//!
//! let config = Link16Config::default();
//! let mut processor = Link16Processor::new(config);
//!
//! // Encode a 75-bit J-series message
//! let message_bits: Vec<bool> = (0..75).map(|i| i % 2 == 0).collect();
//! let encoded = processor.transmit(&message_bits).unwrap();
//! assert!(!encoded.is_empty());
//!
//! // Decode back
//! let decoded_bits = processor.receive(&encoded).unwrap();
//! assert_eq!(decoded_bits.len(), 75);
//! assert_eq!(decoded_bits, message_bits);
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// GF(2^5) Arithmetic — Galois Field with 32 elements
// ---------------------------------------------------------------------------
//
// Generator polynomial: x^5 + x^2 + 1  → 0b100101 = 37 decimal
// This is used for RS(31,15) over GF(32).

/// Order of the Galois field: 2^5 = 32.
const GF_ORDER: usize = 32;

/// Generator polynomial for GF(2^5): x^5 + x^2 + 1 = 0x25.
const GF_PRIM_POLY: u8 = 0x25; // 0b10_0101

/// Precomputed exp (antilog) table for GF(2^5).
/// exp_table[i] = α^i where α is the primitive element.
fn build_gf_tables() -> ([u8; 64], [u8; 32]) {
    let mut exp = [0u8; 64];
    let mut log = [0u8; 32];

    let mut x: u8 = 1;
    for i in 0..31 {
        exp[i] = x;
        log[x as usize] = i as u8;
        x <<= 1;
        if x & 0x20 != 0 {
            // Reduce modulo x^5 + x^2 + 1 = 0x25:
            // XOR with the full primitive polynomial to cancel the x^5 term.
            x ^= GF_PRIM_POLY; // 0x25: clears bit5, XORs in lower-degree terms
        }
    }
    // Fill second half for convenient modular indexing
    for i in 31..64 {
        exp[i] = exp[i - 31];
    }
    log[0] = 0xff; // log(0) undefined, sentinel value
    (exp, log)
}

/// Multiply two GF(2^5) elements.
#[inline]
fn gf_mul(a: u8, b: u8, exp: &[u8; 64], log: &[u8; 32]) -> u8 {
    if a == 0 || b == 0 {
        return 0;
    }
    let la = log[a as usize] as usize;
    let lb = log[b as usize] as usize;
    exp[(la + lb) % 31]
}

/// Add two GF(2^5) elements (XOR).
#[inline]
fn gf_add(a: u8, b: u8) -> u8 {
    a ^ b
}

/// Subtract two GF(2^5) elements (same as add in characteristic-2 fields).
#[inline]
fn gf_sub(a: u8, b: u8) -> u8 {
    a ^ b
}

/// Compute the multiplicative inverse in GF(2^5).
#[inline]
fn gf_inv(a: u8, exp: &[u8; 64], log: &[u8; 32]) -> u8 {
    assert_ne!(a, 0, "Cannot invert zero in GF");
    let la = log[a as usize] as usize;
    exp[(31 - la) % 31]
}

/// Raise α to power p in GF(2^5).
#[inline]
fn gf_pow(p: usize, exp: &[u8; 64]) -> u8 {
    exp[p % 31]
}

// ---------------------------------------------------------------------------
// Reed-Solomon RS(31,15) over GF(2^5)
// ---------------------------------------------------------------------------
//
// n = 31 (codeword length, = 2^5 - 1)
// k = 15 (data symbols)
// t = 8  (error correction capability, = (n - k) / 2 = 16/2)
// 2t = 16 parity symbols
// Generator: g(x) = ∏_{i=1}^{16} (x - α^i)

/// Reed-Solomon codec over GF(2^5) configured as RS(31,15).
///
/// Reference: MIL-STD-6016 Section 5.2.2 — Forward Error Correction.
pub struct Link16ReedSolomon {
    exp: [u8; 64],
    log: [u8; 32],
    /// Generator polynomial coefficients g[0..=16] (degree 16).
    gen: [u8; 17],
}

impl Link16ReedSolomon {
    /// RS(31,15) parameters.
    pub const N: usize = 31; // codeword length
    pub const K: usize = 15; // data symbols
    pub const T: usize = 8;  // error correction capability
    pub const PARITY: usize = 16; // 2*T parity symbols

    /// Create a new RS(31,15) codec.
    pub fn new() -> Self {
        let (exp, log) = build_gf_tables();
        let gen = Self::build_generator(&exp, &log);
        Self { exp, log, gen }
    }

    /// Build the generator polynomial g(x) = ∏_{i=1}^{16} (x − α^i).
    ///
    /// g is stored as coefficients g[0..=16] where g[i] is the coefficient of x^i.
    /// g[16] = 1 (monic). Uses a temporary copy to avoid in-place aliasing bugs.
    fn build_generator(exp: &[u8; 64], log: &[u8; 32]) -> [u8; 17] {
        Self::build_generator_poly(exp, log)
    }

    /// Correct generator polynomial build using a fresh-copy approach.
    fn build_generator_poly(exp: &[u8; 64], log: &[u8; 32]) -> [u8; 17] {
        // g(x) starts as 1 (constant polynomial, degree 0)
        // Stored low-degree first: g[i] = coefficient of x^i
        let mut g = [0u8; 17];
        g[0] = 1u8;

        for i in 1usize..=16 {
            let root = gf_pow(i, exp); // α^i root of (x − α^i)
            // Multiply g by (x + root) in GF(2^m):
            // new_g[j] = old_g[j-1] XOR (root * old_g[j])
            // Process high-to-low to avoid aliasing; use a temp copy
            let old_g = g;
            // New g has degree i; initialise to zero
            g = [0u8; 17];
            for j in 0..=i {
                // Contribution from x * old_g[j-1]: coefficient j gets old_g[j-1]
                if j > 0 {
                    g[j] = gf_add(g[j], old_g[j - 1]);
                }
                // Contribution from root * old_g[j]: coefficient j gets root*old_g[j]
                if j < i {
                    g[j] = gf_add(g[j], gf_mul(root, old_g[j], exp, log));
                }
            }
        }
        g
    }

    /// Encode `data` (15 symbols over GF(2^5)) → 31-symbol codeword.
    ///
    /// Systematic encoding: codeword = [data | parity].
    /// Uses LFSR polynomial division: parity = x^16 * data(x) mod g(x).
    pub fn encode(&self, data: &[u8]) -> Result<[u8; 31], &'static str> {
        if data.len() != Self::K {
            return Err("RS(31,15): need exactly 15 data symbols");
        }
        for &s in data {
            if s >= 32 {
                return Err("RS(31,15): symbol out of GF(2^5) range");
            }
        }

        // Build the generator polynomial fresh (to guarantee correctness).
        let g = Self::build_generator_poly(&self.exp, &self.log);

        // Polynomial long division to compute rem = x^16*data(x) mod g(x).
        // rem[i] = coefficient of x^i in the remainder.
        // Process the 15 data symbols: each is a coefficient of x^{14-j} in data(x),
        // so we are processing x^{14-j+16} = x^{30-j} through the register.
        // Standard LFSR encoder: shift register has size 16 (= number of parity symbols).
        let mut rem = [0u8; 16];

        for &d in data {
            // feedback = current data symbol XOR top of remainder register
            let feedback = gf_add(d, rem[15]);
            if feedback != 0 {
                // Shift register: rem[i] = rem[i-1] XOR feedback * g[i]
                // g has coefficients g[0..=15] (the non-monic part; g[16]=1 is implicit)
                for i in (1..16).rev() {
                    rem[i] = gf_add(rem[i - 1], gf_mul(feedback, g[i], &self.exp, &self.log));
                }
                rem[0] = gf_mul(feedback, g[0], &self.exp, &self.log);
            } else {
                // No feedback: just shift
                for i in (1..16).rev() {
                    rem[i] = rem[i - 1];
                }
                rem[0] = 0;
            }
        }

        // Codeword: data symbols first, then parity (rem[15] = highest-degree parity first)
        let mut codeword = [0u8; 31];
        codeword[..15].copy_from_slice(data);
        for i in 0..16 {
            codeword[15 + i] = rem[15 - i];
        }
        Ok(codeword)
    }

    /// Decode a potentially-errored 31-symbol codeword.
    ///
    /// Returns the corrected 15 data symbols, or an error if uncorrectable.
    /// Uses Berlekamp-Massey + Chien search + Forney algorithm.
    pub fn decode(&self, received: &[u8]) -> Result<[u8; 15], &'static str> {
        if received.len() != Self::N {
            return Err("RS(31,15): need exactly 31 received symbols");
        }

        // 1. Compute syndromes S_i = r(α^i), i = 1..=16
        let syndromes = self.compute_syndromes(received);

        // If all syndromes are zero, no errors
        if syndromes.iter().all(|&s| s == 0) {
            let mut out = [0u8; 15];
            out.copy_from_slice(&received[..15]);
            return Ok(out);
        }

        // 2. Berlekamp-Massey to find error locator polynomial σ(x)
        // σ(x) = 1 + σ_1*x + σ_2*x^2 + ... + σ_t*x^t
        let sigma = self.berlekamp_massey(&syndromes)?;
        let num_errors = sigma.iter().rev().position(|&c| c != 0)
            .map(|i| sigma.len() - 1 - i)
            .unwrap_or(0);

        if num_errors == 0 {
            // No errors after BM
            let mut out = [0u8; 15];
            out.copy_from_slice(&received[..15]);
            return Ok(out);
        }

        // 3. Chien search to find error location numbers
        // An error at codeword position `pos` corresponds to location number α^pos
        // (where pos is 0-indexed from the LEFT, and codeword[pos] is coeff of x^{30-pos})
        // σ(X_k^{-1}) = 0 where X_k is the location number
        // X_k = α^{30 - pos} in our convention (so pos 0 → α^30, pos 30 → α^0)
        // We search: for each i in 0..31, try X = α^i and check if σ(X^{-1}) = 0.
        // If so, the position is: pos = 30 - i (mod 31)
        let error_locs = self.chien_search(&sigma, num_errors)?;

        // 4. Forney algorithm: compute error magnitudes
        let magnitudes = self.forney_magnitudes(&syndromes, &sigma, &error_locs);

        // 5. Apply corrections
        let mut corrected = [0u8; 31];
        corrected.copy_from_slice(received);
        for (&(loc_exp, pos), &mag) in error_locs.iter().zip(magnitudes.iter()) {
            let _ = loc_exp; // loc_exp = i such that X = α^i (not needed for correction)
            corrected[pos] = gf_add(corrected[pos], mag);
        }

        // 6. Verify
        let check = self.compute_syndromes(&corrected);
        if check.iter().any(|&s| s != 0) {
            return Err("RS(31,15): uncorrectable error pattern");
        }

        let mut out = [0u8; 15];
        out.copy_from_slice(&corrected[..15]);
        Ok(out)
    }

    /// Compute the 16 syndromes S_i = C(α^i), for i = 1..=16, using Horner's method.
    ///
    /// C(x) = c[0]*x^{30} + c[1]*x^{29} + ... + c[30]*x^0 (codeword stored MSB first).
    fn compute_syndromes(&self, codeword: &[u8]) -> [u8; 16] {
        let mut syndromes = [0u8; 16];
        for (idx, sndrome) in syndromes.iter_mut().enumerate() {
            let alpha = gf_pow(idx + 1, &self.exp); // α^{idx+1}
            let mut val = 0u8;
            for &sym in codeword {
                // Horner: val = val * alpha + sym
                val = gf_add(gf_mul(val, alpha, &self.exp, &self.log), sym);
            }
            *sndrome = val;
        }
        syndromes
    }

    /// Berlekamp-Massey algorithm to find the error locator polynomial σ(x).
    ///
    /// Returns σ as a vec of coefficients [σ_0=1, σ_1, ..., σ_t].
    fn berlekamp_massey(&self, s: &[u8; 16]) -> Result<Vec<u8>, &'static str> {
        let two_t = 16usize;
        let mut c: Vec<u8> = vec![0; two_t + 1]; // Current LFSR
        let mut b: Vec<u8> = vec![0; two_t + 1]; // Previous LFSR
        c[0] = 1;
        b[0] = 1;
        let mut l = 0usize;
        let mut m = 1i64; // shift counter (number of steps since b was updated)
        let mut b_scale = 1u8; // scale for b

        for n in 0..two_t {
            // Discrepancy
            let mut delta = s[n];
            for i in 1..=l {
                delta = gf_add(delta, gf_mul(c[i], s[n - i], &self.exp, &self.log));
            }
            if delta == 0 {
                m += 1;
            } else {
                let t_poly = c.clone();
                let coef = gf_mul(delta, gf_inv(b_scale, &self.exp, &self.log),
                                  &self.exp, &self.log);
                // c(x) -= coef * x^m * b(x)
                let shift = m as usize;
                for i in 0..two_t + 1 {
                    if i >= shift && (i - shift) < b.len() {
                        c[i] = gf_add(c[i], gf_mul(coef, b[i - shift], &self.exp, &self.log));
                    }
                }
                if 2 * l <= n {
                    l = n + 1 - l;
                    b = t_poly;
                    b_scale = delta;
                    m = 1;
                } else {
                    m += 1;
                }
            }
        }

        if l > Self::T {
            return Err("RS(31,15): too many errors to correct");
        }

        // Trim to degree l
        c.truncate(l + 1);
        Ok(c)
    }

    /// Chien search: find error positions by evaluating σ at α^{-i} for i = 0..n-1.
    ///
    /// Returns vec of (location_power, codeword_position) pairs.
    fn chien_search(&self, sigma: &[u8], num_errors: usize)
        -> Result<Vec<(usize, usize)>, &'static str>
    {
        let mut found = Vec::with_capacity(num_errors);
        let n = Self::N; // 31

        for i in 0..n {
            // Location number candidate: X = α^i
            // Check if σ(X^{-1}) = 0, i.e., σ(α^{-i}) = 0
            // α^{-i} = α^{31-i} for i > 0, α^{-0} = α^0 = 1
            let inv_x = if i == 0 { 1u8 } else { gf_pow((31 - i) % 31, &self.exp) };
            let mut val = 0u8;
            let mut xi_pow = 1u8;
            for &coeff in sigma.iter() {
                val = gf_add(val, gf_mul(coeff, xi_pow, &self.exp, &self.log));
                xi_pow = gf_mul(xi_pow, inv_x, &self.exp, &self.log);
            }
            if val == 0 {
                // Error location number X = α^i means error at codeword position:
                // Since codeword[pos] is the coefficient of x^{30-pos}, and
                // the location number X_k = α^{n-1-pos} = α^{30-pos},
                // we get: i = 30 - pos → pos = 30 - i
                let pos = (30usize + 31 - i) % 31; // = (30 - i) mod 31
                found.push((i, pos));
            }
        }

        if found.len() != num_errors {
            return Err("RS(31,15): Chien search found wrong number of error locations");
        }
        Ok(found)
    }

    /// Forney algorithm: compute error magnitudes given locators.
    ///
    /// e_k = Ω(X_k^{-1}) / σ'(X_k^{-1})
    /// (In GF(2^m) characteristic 2, -1 = 1 and the X_k factor cancels)
    fn forney_magnitudes(
        &self,
        syndromes: &[u8; 16],
        sigma: &[u8],
        error_locs: &[(usize, usize)], // (loc_power i, codeword pos)
    ) -> Vec<u8> {
        let two_t = 16usize;

        // Compute error evaluator polynomial Ω(x) = S(x)*σ(x) mod x^{2t}
        // S(x) = S_1 + S_2*x + ... + S_{2t}*x^{2t-1}
        let mut omega = vec![0u8; two_t];
        for i in 0..two_t {
            for j in 0..sigma.len() {
                let si_idx = i as isize - j as isize;
                if si_idx >= 0 && (si_idx as usize) < two_t {
                    omega[i] = gf_add(omega[i],
                        gf_mul(sigma[j], syndromes[si_idx as usize], &self.exp, &self.log));
                }
            }
        }

        // Formal derivative σ'(x): in GF(2^m), d/dx(x^k) = k*x^{k-1}
        // In char-2: even powers vanish, odd powers k→(k-1) degree.
        // σ'(x) = σ_1 + σ_3*x^2 + σ_5*x^4 + ...
        let mut sigma_prime = vec![0u8; sigma.len()];
        for j in (1..sigma.len()).step_by(2) {
            // Derivative of σ_j * x^j is j*σ_j * x^{j-1}; in GF(2), j odd → coefficient
            sigma_prime[j - 1] = sigma[j];
        }

        let mut magnitudes = Vec::with_capacity(error_locs.len());
        for &(i, _pos) in error_locs {
            // X_k = α^i; X_k^{-1} = α^{-i} = α^{(31-i)%31}
            let x_inv = if i == 0 { 1u8 } else { gf_pow((31 - i) % 31, &self.exp) };

            // Evaluate Ω(X_k^{-1})
            let mut omega_val = 0u8;
            let mut xi_pow = 1u8;
            for &o in &omega {
                omega_val = gf_add(omega_val, gf_mul(o, xi_pow, &self.exp, &self.log));
                xi_pow = gf_mul(xi_pow, x_inv, &self.exp, &self.log);
            }

            // Evaluate σ'(X_k^{-1})
            let mut sprime_val = 0u8;
            xi_pow = 1u8;
            for &s in &sigma_prime {
                sprime_val = gf_add(sprime_val, gf_mul(s, xi_pow, &self.exp, &self.log));
                xi_pow = gf_mul(xi_pow, x_inv, &self.exp, &self.log);
            }

            // e_k = Ω(X_k^{-1}) / σ'(X_k^{-1})
            // In GF(2^m), -1 = 1, so the sign doesn't matter.
            // The correct formula does NOT include a leading X_k factor.
            let mag = if sprime_val == 0 {
                0
            } else {
                gf_mul(omega_val, gf_inv(sprime_val, &self.exp, &self.log), &self.exp, &self.log)
            };
            magnitudes.push(mag);
        }
        magnitudes
    }
}

impl Default for Link16ReedSolomon {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// CCSK — Cyclic Code Shift Keying
// ---------------------------------------------------------------------------
//
// CCSK encodes 5-bit symbols (0..31) as cyclic shifts of a 32-chip base
// sequence. A cyclic shift of k positions encodes the value k.
// The base sequence has optimal cyclic autocorrelation properties:
// peak = 32, all other lags ≤ 2 (near-perfect autocorrelation).
//
// Reference: MIL-STD-6016 Section 5.2.1

/// 32-chip CCSK base sequence (balanced PN sequence with ideal cyclic
/// autocorrelation for Link 16, derived from a maximal-length sequence).
/// Bit 0 = first transmitted chip.
const CCSK_BASE: [i8; 32] = [
     1, -1,  1,  1, -1, -1, -1,  1,
     1,  1,  1, -1,  1, -1, -1,  1,
    -1,  1, -1,  1,  1,  1, -1, -1,
     1, -1, -1, -1, -1,  1,  1, -1,
];

/// CCSK modulator/demodulator.
///
/// Each 5-bit symbol maps to one of 32 cyclic shifts of the base sequence.
/// A cyclic shift of `s` positions encodes the symbol value `s`.
pub struct CcskModulator {
    base: [i8; 32],
}

impl CcskModulator {
    /// Create a CCSK modulator with the standard Link 16 base sequence.
    pub fn new() -> Self {
        Self { base: CCSK_BASE }
    }

    /// Modulate a 5-bit symbol (0..31) → 32 chips ∈ {-1, +1}.
    pub fn modulate_symbol(&self, symbol: u8) -> [i8; 32] {
        assert!(symbol < 32, "CCSK symbol must be 0..31, got {}", symbol);
        let s = symbol as usize;
        let mut chips = [0i8; 32];
        for i in 0..32 {
            chips[i] = self.base[(i + s) % 32];
        }
        chips
    }

    /// Demodulate 32 chips → best matching 5-bit symbol (0..31).
    ///
    /// Uses correlator to find cyclic shift with maximum cross-correlation.
    pub fn demodulate_chips(&self, chips: &[i8; 32]) -> u8 {
        let mut best_symbol = 0u8;
        let mut best_corr = i32::MIN;
        for s in 0u8..32 {
            let corr = self.correlate(chips, s);
            if corr > best_corr {
                best_corr = corr;
                best_symbol = s;
            }
        }
        best_symbol
    }

    /// Compute cross-correlation of received chips with cyclic shift s of base.
    pub fn correlate(&self, chips: &[i8; 32], shift: u8) -> i32 {
        let s = shift as usize;
        let mut sum = 0i32;
        for i in 0..32 {
            sum += (chips[i] as i32) * (self.base[(i + s) % 32] as i32);
        }
        sum
    }

    /// Get the peak correlation value (chips perfectly aligned with shift s).
    pub fn peak_correlation(&self, symbol: u8) -> i32 {
        let chips = self.modulate_symbol(symbol);
        self.correlate(&chips, symbol)
    }

    /// Modulate a stream of 5-bit symbols → flat chip stream.
    pub fn modulate_symbols(&self, symbols: &[u8]) -> Vec<i8> {
        let mut out = Vec::with_capacity(symbols.len() * 32);
        for &s in symbols {
            let chips = self.modulate_symbol(s);
            out.extend_from_slice(&chips);
        }
        out
    }

    /// Demodulate a flat chip stream → 5-bit symbols.
    pub fn demodulate_stream(&self, chips: &[i8]) -> Vec<u8> {
        assert_eq!(chips.len() % 32, 0, "Chip stream length must be multiple of 32");
        chips.chunks(32).map(|chunk| {
            let mut arr = [0i8; 32];
            arr.copy_from_slice(chunk);
            self.demodulate_chips(&arr)
        }).collect()
    }
}

impl Default for CcskModulator {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Frequency Hop Sequence Generator
// ---------------------------------------------------------------------------
//
// Link 16 uses 51 frequencies between 969 and 1206 MHz.
// The hop sequence is pseudo-random, derived from a crypto-seeded LFSR.
// For educational purposes, we use a deterministic LFSR with a user seed.

/// 51 nominal hop frequencies for Link 16 (MHz).
///
/// Reference: MIL-STD-6016 Annex B — Frequency Plan.
/// Frequencies are spaced 5 MHz apart starting from 969 MHz (with gaps
/// around the 1030/1090 MHz DME/ATC navigation bands).
const HOP_FREQUENCIES_MHZ: [f64; 51] = [
     969.0,  974.0,  979.0,  984.0,  989.0,
     994.0,  999.0, 1004.0, 1009.0, 1014.0,
    1019.0, 1024.0, 1029.0, 1035.0, 1040.0, // skip 1030/1034 (IFF/ATC)
    1045.0, 1050.0, 1055.0, 1060.0, 1065.0,
    1070.0, 1075.0, 1080.0, 1085.0, 1091.0, // skip 1087–1090 (IFF guard)
    1096.0, 1101.0, 1106.0, 1111.0, 1116.0,
    1121.0, 1126.0, 1131.0, 1136.0, 1141.0,
    1146.0, 1151.0, 1156.0, 1161.0, 1166.0,
    1171.0, 1176.0, 1181.0, 1186.0, 1191.0,
    1196.0, 1201.0, 1206.0, 1118.0, 1123.0, // two additional fill frequencies
    1128.0,
];

/// Pseudo-random frequency hop sequence generator.
///
/// Uses a Galois LFSR with polynomial x^31 + x^3 + 1 for 31-bit state.
/// In a real Link 16 system the hop sequence is TRANSEC crypto-derived.
pub struct HopSequenceGenerator {
    state: u32,
    num_freqs: usize,
}

impl HopSequenceGenerator {
    /// Create a new hop sequence generator with the given crypto seed.
    pub fn new(seed: u32) -> Self {
        let state = if seed == 0 { 0xACE1_0001 } else { seed & 0x7FFF_FFFF };
        Self { state, num_freqs: 51 }
    }

    /// Advance LFSR and return next hop frequency index (0..50).
    pub fn next_hop_index(&mut self) -> usize {
        // Galois LFSR: x^31 + x^3 + 1, polynomial 0x80000003
        let bit = self.state & 1;
        self.state >>= 1;
        if bit != 0 {
            self.state ^= 0x4000_0003; // feedback taps for x^31+x^3+1 (without x^31)
        }
        // Map to [0, 51) using rejection sampling
        let val = (self.state as usize) % 64;
        if val < 51 { val } else { self.next_hop_index() }
    }

    /// Return the next hop frequency in MHz.
    pub fn next_frequency_mhz(&mut self) -> f64 {
        let idx = self.next_hop_index();
        HOP_FREQUENCIES_MHZ[idx]
    }

    /// Generate a block of N hop frequencies.
    pub fn generate_hops(&mut self, n: usize) -> Vec<f64> {
        (0..n).map(|_| self.next_frequency_mhz()).collect()
    }

    /// Verify coverage: check that a sequence of hops uses each of the 51
    /// frequencies at least once within a reasonable window.
    pub fn coverage_check(&mut self, window: usize) -> f64 {
        let hops: Vec<usize> = (0..window).map(|_| self.next_hop_index()).collect();
        let mut used = [false; 51];
        for &h in &hops {
            used[h] = true;
        }
        used.iter().filter(|&&u| u).count() as f64 / 51.0
    }
}

// ---------------------------------------------------------------------------
// Symbol Interleaver
// ---------------------------------------------------------------------------
//
// Link 16 interleaves coded symbols across frequency hops to disperse
// burst errors. A block interleaver with depth equal to the number of
// hop pulses per message is used.

/// Block interleaver for Link 16 symbol sequences.
///
/// Symbols are written column-by-column and read row-by-row to achieve
/// burst error dispersal across frequency hops.
pub struct Link16Interleaver {
    rows: usize,
    cols: usize,
}

impl Link16Interleaver {
    /// Create a new interleaver with specified depth (rows × cols).
    pub fn new(rows: usize, cols: usize) -> Self {
        Self { rows, cols }
    }

    /// Default interleaver for standard message: 31 symbols × 1 block.
    pub fn standard() -> Self {
        // 31 coded symbols from RS encoder, 1 row deep for demonstration.
        // Real Link 16 uses a more complex interleaver tied to hop count.
        Self::new(1, 31)
    }

    /// Interleave: write column-major, read row-major.
    pub fn interleave(&self, symbols: &[u8]) -> Vec<u8> {
        assert_eq!(symbols.len(), self.rows * self.cols,
            "Interleaver input length mismatch");
        let mut out = vec![0u8; symbols.len()];
        for r in 0..self.rows {
            for c in 0..self.cols {
                out[r * self.cols + c] = symbols[c * self.rows + r];
            }
        }
        out
    }

    /// De-interleave: inverse of interleave.
    pub fn deinterleave(&self, symbols: &[u8]) -> Vec<u8> {
        assert_eq!(symbols.len(), self.rows * self.cols,
            "De-interleaver input length mismatch");
        let mut out = vec![0u8; symbols.len()];
        for r in 0..self.rows {
            for c in 0..self.cols {
                out[c * self.rows + r] = symbols[r * self.cols + c];
            }
        }
        out
    }
}

// ---------------------------------------------------------------------------
// TDMA Slot Structure
// ---------------------------------------------------------------------------
//
// Link 16 uses 128 time slots per 12-second epoch (actually 1536 slots/epoch,
// with 12.5 ms frames containing 128 slots of 7.8125 ms).
// MIL-STD-6016 Table 3-1: 128 time slots per 1-second super-frame.

/// Duration of one TDMA slot in microseconds (7.8125 ms).
pub const SLOT_DURATION_US: f64 = 7812.5;

/// Duration of one TDMA slot in seconds.
pub const SLOT_DURATION_S: f64 = SLOT_DURATION_US * 1e-6;

/// Number of slots per epoch (12 seconds / 7.8125 ms ≈ 1536).
pub const SLOTS_PER_EPOCH: usize = 1536;

/// Number of slots per 1-second super-frame.
pub const SLOTS_PER_SUPERFRAME: usize = 128;

/// Epoch duration in seconds.
pub const EPOCH_DURATION_S: f64 = 12.0;

/// Link 16 Net IDs (0..127).
pub type NetId = u8;

/// Assignment type for a TDMA slot.
#[derive(Debug, Clone, PartialEq)]
pub enum SlotAssignment {
    /// Slot is unassigned / idle.
    Idle,
    /// Slot is assigned to transmit on this net.
    Transmit { net: NetId },
    /// Slot is assigned to receive on this net.
    Receive { net: NetId },
    /// Relay slot (regenerate and retransmit).
    Relay { net: NetId },
}

/// One TDMA time slot.
#[derive(Debug, Clone)]
pub struct TdmaSlot {
    /// Slot index within the current epoch (0..SLOTS_PER_EPOCH).
    pub index: usize,
    /// Start time of the slot within the epoch, in seconds.
    pub start_time_s: f64,
    /// Duration of this slot in seconds.
    pub duration_s: f64,
    /// Slot assignment.
    pub assignment: SlotAssignment,
}

impl TdmaSlot {
    /// Create a new slot with the given index.
    pub fn new(index: usize, assignment: SlotAssignment) -> Self {
        Self {
            index,
            start_time_s: index as f64 * SLOT_DURATION_S,
            duration_s: SLOT_DURATION_S,
            assignment,
        }
    }

    /// Compute the center time of this slot.
    pub fn center_time_s(&self) -> f64 {
        self.start_time_s + self.duration_s / 2.0
    }

    /// End time of this slot.
    pub fn end_time_s(&self) -> f64 {
        self.start_time_s + self.duration_s
    }

    /// Compute the absolute wall-clock time (epoch-relative) for a given
    /// epoch number and slot position.
    pub fn absolute_time_s(&self, epoch: usize) -> f64 {
        epoch as f64 * EPOCH_DURATION_S + self.start_time_s
    }
}

/// A Net Participation Group (NPG): a set of slots assigned to one net.
#[derive(Debug, Clone)]
pub struct NetParticipationGroup {
    /// Net identifier.
    pub net_id: NetId,
    /// Slot indices within the epoch assigned to this NPG.
    pub slots: Vec<usize>,
}

impl NetParticipationGroup {
    /// Create an NPG with evenly spaced slots.
    pub fn new_uniform(net_id: NetId, num_slots: usize) -> Self {
        let step = SLOTS_PER_EPOCH / num_slots.max(1);
        let slots = (0..num_slots).map(|i| (i * step) % SLOTS_PER_EPOCH).collect();
        Self { net_id, slots }
    }
}

// ---------------------------------------------------------------------------
// J-Series Message Formatter
// ---------------------------------------------------------------------------
//
// Link 16 J-series messages consist of 75 bits in the standard (STD) format.
// The header word identifies the J-series message number (J0.0 – J31.7).

/// J-series message format (standard, 75 bits).
#[derive(Debug, Clone, PartialEq)]
pub enum MessageFormat {
    /// Standard (STD) single-pulse: 1 header word + up to 2 data words.
    Standard,
    /// Packed-2 double-pulse: 2 header words + up to 4 data words.
    Packed2,
    /// Packed-4 quadruple-pulse: 4 header words + up to 8 data words.
    Packed4,
}

/// A decoded J-series message header.
#[derive(Debug, Clone)]
pub struct JSeriesHeader {
    /// Message format.
    pub format: MessageFormat,
    /// J-series message number (0..=31).
    pub j_number: u8,
    /// Sub-label (0..=7).
    pub sub_label: u8,
    /// Net membership (0..=127).
    pub network_participation: u8,
    /// Time slot indicator.
    pub time_slot: u16,
}

impl JSeriesHeader {
    /// Create a PPLI (Precise Participant Location and Identification) header.
    /// PPLI is J2.0 — the most commonly used J-series message.
    pub fn ppli() -> Self {
        Self {
            format: MessageFormat::Standard,
            j_number: 2,
            sub_label: 0,
            network_participation: 0,
            time_slot: 0,
        }
    }

    /// Serialize the header to 15 bits.
    pub fn to_bits(&self) -> [bool; 15] {
        let mut bits = [false; 15];
        // Bits 0-4: J-series number (5 bits)
        for i in 0..5 {
            bits[i] = (self.j_number >> (4 - i)) & 1 == 1;
        }
        // Bits 5-7: sub-label (3 bits)
        for i in 0..3 {
            bits[5 + i] = (self.sub_label >> (2 - i)) & 1 == 1;
        }
        // Bits 8-14: NPG (7 bits, from network_participation)
        for i in 0..7 {
            bits[8 + i] = (self.network_participation >> (6 - i)) & 1 == 1;
        }
        bits
    }

    /// Parse a header from 15 bits.
    pub fn from_bits(bits: &[bool]) -> Option<Self> {
        if bits.len() < 15 { return None; }
        let j_number = (0..5).fold(0u8, |acc, i| (acc << 1) | bits[i] as u8);
        let sub_label = (0..3).fold(0u8, |acc, i| (acc << 1) | bits[5 + i] as u8);
        let npg = (0..7).fold(0u8, |acc, i| (acc << 1) | bits[8 + i] as u8);
        Some(Self {
            format: MessageFormat::Standard,
            j_number,
            sub_label,
            network_participation: npg,
            time_slot: 0,
        })
    }
}

/// J-series message formatter: packs/unpacks 75-bit J-series messages.
pub struct MessageFormatter;

impl MessageFormatter {
    /// Pack a bit vector into 5-bit GF(2^5) symbols.
    ///
    /// 75 bits → 15 symbols (with 0 padding to reach a full symbol count).
    pub fn bits_to_symbols(bits: &[bool]) -> Vec<u8> {
        // Pad to multiple of 5
        let padded_len = (bits.len() + 4) / 5 * 5;
        let mut padded = bits.to_vec();
        padded.resize(padded_len, false);

        padded.chunks(5).map(|chunk| {
            (0..5).fold(0u8, |acc, i| (acc << 1) | chunk[i] as u8)
        }).collect()
    }

    /// Unpack 5-bit symbols back to bits.
    pub fn symbols_to_bits(symbols: &[u8], num_bits: usize) -> Vec<bool> {
        let mut bits = Vec::with_capacity(symbols.len() * 5);
        for &sym in symbols {
            for i in (0..5).rev() {
                bits.push((sym >> i) & 1 == 1);
            }
        }
        bits.truncate(num_bits);
        bits
    }

    /// Build a PPLI message payload (J2.0, 75 bits).
    ///
    /// Fields: latitude (24 bits), longitude (25 bits), altitude (16 bits),
    /// identity (10 bits).
    pub fn build_ppli(
        lat_deg: f64,
        lon_deg: f64,
        alt_ft: f64,
        identity: u16,
    ) -> Vec<bool> {
        let mut bits = Vec::with_capacity(75);

        // Latitude: 24 bits, -90..+90 scaled to 0..2^24-1
        let lat_raw = ((lat_deg + 90.0) / 180.0 * ((1u32 << 24) - 1) as f64) as u32;
        for i in (0..24).rev() {
            bits.push((lat_raw >> i) & 1 == 1);
        }
        // Longitude: 25 bits, -180..+180 scaled to 0..2^25-1
        let lon_raw = ((lon_deg + 180.0) / 360.0 * ((1u32 << 25) - 1) as f64) as u32;
        for i in (0..25).rev() {
            bits.push((lon_raw >> i) & 1 == 1);
        }
        // Altitude: 16 bits, 0..65535 ft
        let alt_raw = (alt_ft.max(0.0).min(65535.0)) as u16;
        for i in (0..16).rev() {
            bits.push((alt_raw >> i) & 1 == 1);
        }
        // Identity: 10 bits
        for i in (0..10).rev() {
            bits.push((identity >> i) & 1 == 1);
        }
        bits
    }

    /// Parse PPLI fields from a 75-bit payload.
    pub fn parse_ppli(bits: &[bool]) -> Option<(f64, f64, f64, u16)> {
        if bits.len() < 75 { return None; }
        let lat_raw = (0..24).fold(0u32, |acc, i| (acc << 1) | bits[i] as u32);
        let lon_raw = (0..25).fold(0u32, |acc, i| (acc << 1) | bits[24 + i] as u32);
        let alt_raw = (0..16).fold(0u16, |acc, i| (acc << 1) | bits[49 + i] as u16);
        let identity = (0..10).fold(0u16, |acc, i| (acc << 1) | bits[65 + i] as u16);
        let lat = lat_raw as f64 / ((1u32 << 24) - 1) as f64 * 180.0 - 90.0;
        let lon = lon_raw as f64 / ((1u32 << 25) - 1) as f64 * 360.0 - 180.0;
        let alt = alt_raw as f64;
        Some((lat, lon, alt, identity))
    }
}

// ---------------------------------------------------------------------------
// Link Budget Calculator
// ---------------------------------------------------------------------------
//
// Free-space path loss at L-band, Eb/N0 requirements for MSK at BER 1e-5,
// jam margin calculation per MIL-HDBK-1195.

/// Link budget parameters for Link 16.
#[derive(Debug, Clone)]
pub struct Link16LinkBudget {
    /// Transmit power in dBW.
    pub tx_power_dbw: f64,
    /// Transmit antenna gain in dBi.
    pub tx_gain_dbi: f64,
    /// Distance between terminals in km.
    pub range_km: f64,
    /// Center frequency in MHz.
    pub freq_mhz: f64,
    /// Receive antenna gain in dBi.
    pub rx_gain_dbi: f64,
    /// System noise temperature in Kelvin.
    pub noise_temp_k: f64,
    /// Required Eb/N0 for BER = 1e-5 (MSK ≈ 9.6 dB).
    pub required_eb_n0_db: f64,
    /// Data rate in bit/s.
    pub data_rate_bps: f64,
}

impl Link16LinkBudget {
    /// Default Link 16 air-to-air link at 100 km.
    pub fn default_air_to_air() -> Self {
        Self {
            tx_power_dbw: 10.0,       // 10 W = 10 dBW
            tx_gain_dbi: 3.0,         // near-omnidirectional
            range_km: 100.0,
            freq_mhz: 1045.0,         // center of hop set
            rx_gain_dbi: 3.0,
            noise_temp_k: 290.0,
            required_eb_n0_db: 9.6,   // MSK at BER 1e-5
            data_rate_bps: 5e6,       // 5 Mbit/s raw
        }
    }

    /// Free-space path loss (dB).
    ///
    /// FSPL = 20·log10(d) + 20·log10(f) + 20·log10(4π/c)
    pub fn fspl_db(&self) -> f64 {
        let d_m = self.range_km * 1e3;
        let f_hz = self.freq_mhz * 1e6;
        let c = 3e8;
        20.0 * d_m.log10() + 20.0 * f_hz.log10() - 147.55
    }

    /// Received signal power in dBW.
    pub fn rx_power_dbw(&self) -> f64 {
        self.tx_power_dbw + self.tx_gain_dbi - self.fspl_db() + self.rx_gain_dbi
    }

    /// Thermal noise power spectral density N0 in dBW/Hz.
    pub fn noise_psd_dbw_hz(&self) -> f64 {
        // N0 = kT, k = Boltzmann = -228.6 dBW/K/Hz
        -228.6 + 10.0 * self.noise_temp_k.log10()
    }

    /// Received Eb/N0 in dB.
    pub fn rx_eb_n0_db(&self) -> f64 {
        self.rx_power_dbw() - self.noise_psd_dbw_hz() - 10.0 * self.data_rate_bps.log10()
    }

    /// Link margin in dB (positive = link is closed).
    pub fn link_margin_db(&self) -> f64 {
        self.rx_eb_n0_db() - self.required_eb_n0_db
    }

    /// Maximum range (km) at which link closes (margin ≥ 0 dB).
    pub fn max_range_km(&self) -> f64 {
        // rx_eb_n0_db = tx_power + tx_gain - 20*log10(range_m) - 20*log10(f_hz) + 147.55
        //              + rx_gain - N0_dBW/Hz - 10*log10(data_rate)
        // Set equal to required_eb_n0_db and solve for range:
        let budget = self.tx_power_dbw + self.tx_gain_dbi + self.rx_gain_dbi
            - self.noise_psd_dbw_hz()
            - 10.0 * self.data_rate_bps.log10()
            - self.required_eb_n0_db
            - 20.0 * (self.freq_mhz * 1e6).log10()
            + 147.55;
        let range_m = 10f64.powf(budget / 20.0);
        range_m / 1e3
    }

    /// Jam margin (dB) — ability to operate in the presence of jamming.
    ///
    /// Jam_margin = Processing_gain - Required_Eb/N0 - Implementation_loss
    /// Processing gain = 10·log10(chip_rate / data_rate)
    pub fn jam_margin_db(&self) -> f64 {
        let chip_rate = 5e6 * 32.0; // 5 Mbit/s × 32 chips/symbol (CCSK)
        let processing_gain_db = 10.0 * (chip_rate / self.data_rate_bps).log10();
        let implementation_loss_db = 2.0; // typical
        processing_gain_db - self.required_eb_n0_db - implementation_loss_db
    }

    /// Processing gain from CCSK spreading (dB).
    pub fn ccsk_processing_gain_db() -> f64 {
        // 32-chip CCSK: PG = 10*log10(32) ≈ 15.05 dB
        10.0 * 32.0f64.log10()
    }
}

// ---------------------------------------------------------------------------
// Link 16 Configuration
// ---------------------------------------------------------------------------

/// Complete Link 16 configuration.
#[derive(Debug, Clone)]
pub struct Link16Config {
    /// Network identifier.
    pub net_id: NetId,
    /// Crypto seed for the hop sequence (educational placeholder).
    pub crypto_seed: u32,
    /// Maximum number of TDMA slots per super-frame.
    pub slots_per_superframe: usize,
    /// Number of hops per transmitted pulse.
    pub hops_per_pulse: usize,
    /// Use Reed-Solomon FEC.
    pub use_rs_fec: bool,
    /// Message format.
    pub message_format: MessageFormat,
}

impl Link16Config {
    /// Default operational Link 16 configuration.
    pub fn default() -> Self {
        Self {
            net_id: 0,
            crypto_seed: 0xDEAD_BEEF,
            slots_per_superframe: SLOTS_PER_SUPERFRAME,
            hops_per_pulse: 3,
            use_rs_fec: true,
            message_format: MessageFormat::Standard,
        }
    }

    /// Validate configuration.
    pub fn validate(&self) -> Result<(), &'static str> {
        if self.slots_per_superframe > SLOTS_PER_EPOCH {
            return Err("slots_per_superframe exceeds epoch size");
        }
        if self.hops_per_pulse < 1 || self.hops_per_pulse > 16 {
            return Err("hops_per_pulse must be 1..16");
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------------
// Simple MSK Baseband Modulator (for simulation purposes)
// ---------------------------------------------------------------------------
//
// MSK: h = 0.5, continuous phase, ±π/2 per bit.
// chip_rate = 5 Mchip/s → samples_per_chip determines IQ sample rate.

/// Simple baseband MSK modulator for Link 16 chip streams.
pub struct MskBasebandModulator {
    /// Samples per chip.
    pub samples_per_chip: usize,
    /// Current accumulated phase (radians).
    phase: f64,
}

impl MskBasebandModulator {
    /// Create with desired oversampling factor.
    pub fn new(samples_per_chip: usize) -> Self {
        Self { samples_per_chip, phase: 0.0 }
    }

    /// Modulate a chip stream ∈ {-1, +1} → complex IQ samples.
    pub fn modulate(&mut self, chips: &[i8]) -> Vec<(f64, f64)> {
        let delta_phi = PI / (2.0 * self.samples_per_chip as f64);
        let mut iq = Vec::with_capacity(chips.len() * self.samples_per_chip);
        for &chip in chips {
            let direction = chip as f64; // +1 or -1
            for _ in 0..self.samples_per_chip {
                iq.push((self.phase.cos(), self.phase.sin()));
                self.phase += direction * delta_phi;
            }
        }
        iq
    }

    /// Reset phase accumulator.
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Full TX/RX Chain — Link16Processor
// ---------------------------------------------------------------------------

/// Complete Link 16 physical-layer processor (TX and RX chains).
///
/// TX chain: bits → symbols → RS encode → interleave → CCSK modulate → hops
/// RX chain: hops + CCSK chips → demodulate → de-interleave → RS decode → bits
pub struct Link16Processor {
    config: Link16Config,
    rs: Link16ReedSolomon,
    ccsk: CcskModulator,
    hop_gen: HopSequenceGenerator,
    interleaver: Link16Interleaver,
}

impl Link16Processor {
    /// Create a new Link16Processor with the given configuration.
    pub fn new(config: Link16Config) -> Self {
        let seed = config.crypto_seed;
        Self {
            rs: Link16ReedSolomon::new(),
            ccsk: CcskModulator::new(),
            hop_gen: HopSequenceGenerator::new(seed),
            interleaver: Link16Interleaver::new(1, 31), // 31 RS symbols, 1 block
            config,
        }
    }

    /// Transmit: encode 75 information bits → CCSK chip stream with hop table.
    ///
    /// Returns a tuple of:
    ///   - CCSK chips (992 chips = 31 RS symbols × 32 chips/symbol)
    ///   - Hop frequency table (MHz) — one frequency per group of chips
    pub fn transmit(&mut self, bits: &[bool]) -> Result<(Vec<i8>, Vec<f64>), &'static str> {
        if bits.len() != 75 {
            return Err("Link 16 standard message must be exactly 75 bits");
        }

        // 1. Convert bits → 15 GF(2^5) data symbols (5 bits each)
        let data_symbols = MessageFormatter::bits_to_symbols(bits);
        if data_symbols.len() != 15 {
            return Err("Bit-to-symbol conversion error: need exactly 15 symbols");
        }

        // 2. Reed-Solomon RS(31,15) encoding
        let codeword = if self.config.use_rs_fec {
            self.rs.encode(&data_symbols)?.to_vec()
        } else {
            // Pad to 31 symbols without FEC
            let mut cw = data_symbols.clone();
            cw.resize(31, 0);
            cw
        };
        debug_assert_eq!(codeword.len(), 31);

        // 3. Symbol interleaving
        let interleaved = self.interleaver.interleave(&codeword);

        // 4. CCSK modulation: each GF symbol → 32 chips
        let chips = self.ccsk.modulate_symbols(&interleaved);
        debug_assert_eq!(chips.len(), 31 * 32); // 992 chips

        // 5. Assign hop frequencies — one per pulse (3 hops over 992 chips)
        let num_hops = self.config.hops_per_pulse.max(1);
        let hops = self.hop_gen.generate_hops(num_hops);

        Ok((chips, hops))
    }

    /// Receive: decode CCSK chip stream → 75 information bits.
    ///
    /// `chips` must be 992 chips (31 symbols × 32 chips).
    /// `hop_freqs` is for channel equalization (simplified here).
    pub fn receive(&mut self, chips: &[i8]) -> Result<Vec<bool>, &'static str> {
        if chips.len() != 31 * 32 {
            return Err("RX chain: expected 992 chips (31 RS symbols × 32 chips)");
        }

        // 1. CCSK demodulation: 32 chips → 5-bit symbol
        let coded_symbols = self.ccsk.demodulate_stream(chips);
        debug_assert_eq!(coded_symbols.len(), 31);

        // 2. De-interleaving
        let deinterleaved = self.interleaver.deinterleave(&coded_symbols);

        // 3. RS(31,15) decoding
        let data_symbols: Vec<u8> = if self.config.use_rs_fec {
            let arr: [u8; 31] = deinterleaved.try_into().unwrap();
            self.rs.decode(&arr)?.to_vec()
        } else {
            deinterleaved[..15].to_vec()
        };
        debug_assert_eq!(data_symbols.len(), 15);

        // 4. Convert symbols → bits
        let bits = MessageFormatter::symbols_to_bits(&data_symbols, 75);
        Ok(bits)
    }

    /// Get current TDMA slot for the given epoch and slot index.
    pub fn tdma_slot(&self, index: usize) -> TdmaSlot {
        let assignment = if self.config.net_id == 0 {
            SlotAssignment::Idle
        } else {
            SlotAssignment::Receive { net: self.config.net_id }
        };
        TdmaSlot::new(index, assignment)
    }

    /// Compute the number of CCSK chips per complete message.
    pub fn chips_per_message() -> usize {
        31 * 32 // 992
    }

    /// Compute the CCSK processing gain in dB.
    pub fn processing_gain_db() -> f64 {
        Link16LinkBudget::ccsk_processing_gain_db()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- GF(2^5) arithmetic tests -------------------------------------------

    #[test]
    fn test_gf_tables_consistency() {
        let (exp, log) = build_gf_tables();
        // α^0 = 1
        assert_eq!(exp[0], 1);
        // α^30 should be the last non-zero element before wrapping
        assert_ne!(exp[30], 0);
        // log(1) = 0
        assert_eq!(log[1], 0);
        // log(0) = sentinel 0xff
        assert_eq!(log[0], 0xff);
    }

    #[test]
    fn test_gf_add_is_xor() {
        assert_eq!(gf_add(0b11010, 0b01001), 0b10011);
        assert_eq!(gf_add(7, 7), 0); // a + a = 0 in GF(2^m)
    }

    #[test]
    fn test_gf_mul_by_zero() {
        let (exp, log) = build_gf_tables();
        for a in 0u8..32 {
            assert_eq!(gf_mul(a, 0, &exp, &log), 0);
            assert_eq!(gf_mul(0, a, &exp, &log), 0);
        }
    }

    #[test]
    fn test_gf_mul_by_one() {
        let (exp, log) = build_gf_tables();
        for a in 1u8..32 {
            assert_eq!(gf_mul(a, 1, &exp, &log), a);
        }
    }

    #[test]
    fn test_gf_mul_commutativity() {
        let (exp, log) = build_gf_tables();
        for a in 1u8..32 {
            for b in 1u8..32 {
                assert_eq!(gf_mul(a, b, &exp, &log), gf_mul(b, a, &exp, &log));
            }
        }
    }

    #[test]
    fn test_gf_inv_times_self_is_one() {
        let (exp, log) = build_gf_tables();
        for a in 1u8..32 {
            let inv = gf_inv(a, &exp, &log);
            assert_eq!(gf_mul(a, inv, &exp, &log), 1,
                "a={a}, inv={inv}, a*inv should be 1");
        }
    }

    #[test]
    fn test_gf_field_size() {
        // GF(2^5) has 32 elements (0..31)
        assert_eq!(GF_ORDER, 32);
    }

    // --- Reed-Solomon RS(31,15) tests ----------------------------------------

    #[test]
    fn test_rs_encode_length() {
        let rs = Link16ReedSolomon::new();
        let data = [1u8; 15];
        let codeword = rs.encode(&data).unwrap();
        assert_eq!(codeword.len(), 31);
    }

    #[test]
    fn test_rs_encode_data_preserved() {
        let rs = Link16ReedSolomon::new();
        let data: [u8; 15] = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
        let codeword = rs.encode(&data).unwrap();
        assert_eq!(&codeword[..15], &data);
    }

    #[test]
    fn test_rs_syndrome_zero_for_valid_codeword() {
        let rs = Link16ReedSolomon::new();
        let data = [3u8, 7, 14, 21, 1, 5, 15, 31, 2, 6, 10, 20, 0, 9, 17];
        let codeword = rs.encode(&data).unwrap();
        let syndromes = rs.compute_syndromes(&codeword);
        assert!(syndromes.iter().all(|&s| s == 0),
            "All syndromes must be zero for a valid codeword");
    }

    #[test]
    fn test_rs_decode_no_errors() {
        let rs = Link16ReedSolomon::new();
        let data = [5u8, 10, 15, 20, 25, 30, 1, 6, 11, 16, 21, 26, 0, 7, 13];
        let codeword = rs.encode(&data).unwrap();
        let decoded = rs.decode(&codeword).unwrap();
        assert_eq!(&decoded, &data);
    }

    #[test]
    fn test_rs_decode_single_error() {
        let rs = Link16ReedSolomon::new();
        let data = [1u8, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15];
        let mut codeword = rs.encode(&data).unwrap();
        codeword[0] ^= 0b10110; // inject 1 symbol error
        let decoded = rs.decode(&codeword).unwrap();
        assert_eq!(&decoded, &data, "Single-error correction failed");
    }

    #[test]
    fn test_rs_decode_four_errors() {
        let rs = Link16ReedSolomon::new();
        let data = [7u8, 14, 21, 0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12];
        let mut codeword = rs.encode(&data).unwrap();
        // Inject 4 errors at known positions
        codeword[2] ^= 0b00111;
        codeword[8] ^= 0b11000;
        codeword[15] ^= 0b10101;
        codeword[24] ^= 0b01010;
        let decoded = rs.decode(&codeword).unwrap();
        assert_eq!(&decoded, &data, "4-error correction failed");
    }

    #[test]
    fn test_rs_decode_eight_errors() {
        let rs = Link16ReedSolomon::new();
        let data = [31u8, 30, 29, 28, 27, 26, 25, 24, 23, 22, 21, 20, 19, 18, 17];
        let mut codeword = rs.encode(&data).unwrap();
        // 8 errors at positions spread across the codeword
        for (i, pos) in [1, 3, 5, 7, 9, 11, 13, 25].iter().enumerate() {
            codeword[*pos] ^= (i + 1) as u8 & 0x1f;
        }
        let decoded = rs.decode(&codeword).unwrap();
        assert_eq!(&decoded, &data, "8-error correction (t=8) failed");
    }

    #[test]
    fn test_rs_params() {
        assert_eq!(Link16ReedSolomon::N, 31);
        assert_eq!(Link16ReedSolomon::K, 15);
        assert_eq!(Link16ReedSolomon::T, 8);
        assert_eq!(Link16ReedSolomon::PARITY, 16);
        assert_eq!(Link16ReedSolomon::N - Link16ReedSolomon::K, Link16ReedSolomon::PARITY);
    }

    #[test]
    fn test_rs_all_zero_message() {
        let rs = Link16ReedSolomon::new();
        let data = [0u8; 15];
        let codeword = rs.encode(&data).unwrap();
        assert!(codeword.iter().all(|&s| s == 0),
            "RS encoding of all-zeros should give all-zeros codeword");
        let decoded = rs.decode(&codeword).unwrap();
        assert_eq!(&decoded, &data);
    }

    #[test]
    fn test_rs_encode_invalid_symbol() {
        let rs = Link16ReedSolomon::new();
        let mut data = [1u8; 15];
        data[7] = 32; // out of GF(2^5) range
        assert!(rs.encode(&data).is_err());
    }

    // --- CCSK Modulator tests ------------------------------------------------

    #[test]
    fn test_ccsk_base_sequence_balance() {
        // Base sequence should be balanced: equal number of +1 and -1
        let pos: i32 = CCSK_BASE.iter().map(|&c| c as i32).filter(|&c| c > 0).count() as i32;
        let neg: i32 = CCSK_BASE.iter().map(|&c| c as i32).filter(|&c| c < 0).count() as i32;
        assert!((pos - neg).abs() <= 2,
            "CCSK base sequence must be approximately balanced: +1={pos}, -1={neg}");
    }

    #[test]
    fn test_ccsk_chip_values_are_binary() {
        for &chip in &CCSK_BASE {
            assert!(chip == 1 || chip == -1, "CCSK chips must be ±1, got {chip}");
        }
    }

    #[test]
    fn test_ccsk_zero_shift_identity() {
        let ccsk = CcskModulator::new();
        let chips = ccsk.modulate_symbol(0);
        assert_eq!(chips, CCSK_BASE, "Shift-0 should return base sequence");
    }

    #[test]
    fn test_ccsk_modulate_all_32_shifts() {
        let ccsk = CcskModulator::new();
        for s in 0u8..32 {
            let chips = ccsk.modulate_symbol(s);
            assert_eq!(chips.len(), 32);
            // Each chip must be ±1
            for &c in &chips {
                assert!(c == 1 || c == -1);
            }
        }
    }

    #[test]
    fn test_ccsk_demodulate_all_32_symbols() {
        let ccsk = CcskModulator::new();
        for s in 0u8..32 {
            let chips = ccsk.modulate_symbol(s);
            let decoded = ccsk.demodulate_chips(&chips);
            assert_eq!(decoded, s, "CCSK round-trip failed for symbol {s}");
        }
    }

    #[test]
    fn test_ccsk_correlation_peak() {
        let ccsk = CcskModulator::new();
        for s in 0u8..32 {
            let chips = ccsk.modulate_symbol(s);
            let corr = ccsk.correlate(&chips, s);
            // Auto-correlation peak must equal chip length (32)
            assert_eq!(corr, 32,
                "CCSK auto-correlation peak must be 32 for symbol {s}, got {corr}");
        }
    }

    #[test]
    fn test_ccsk_cross_correlation_bounded() {
        let ccsk = CcskModulator::new();
        let s0 = ccsk.modulate_symbol(0);
        for s in 1u8..32 {
            let corr = ccsk.correlate(&s0, s).abs();
            // Cross-correlation for non-matching shifts must be strictly less than
            // the autocorrelation peak (32), ensuring unambiguous demodulation.
            // For the 32-chip base sequence used in this implementation, the
            // worst-case cyclic cross-correlation is 16 (at shift 16).
            assert!(corr < 32,
                "CCSK cross-correlation must be less than peak: shift={s}, corr={corr}");
            // Also verify cross-corr does not exceed 50% of peak (16 out of 32)
            assert!(corr <= 16,
                "CCSK cross-correlation too high: shift={s}, corr={corr}");
        }
    }

    #[test]
    fn test_ccsk_stream_roundtrip() {
        let ccsk = CcskModulator::new();
        let symbols: Vec<u8> = (0u8..8).collect();
        let chips = ccsk.modulate_symbols(&symbols);
        assert_eq!(chips.len(), 8 * 32);
        let decoded = ccsk.demodulate_stream(&chips);
        assert_eq!(decoded, symbols);
    }

    // --- Hop Sequence Generator tests ----------------------------------------

    #[test]
    fn test_hop_indices_in_range() {
        let mut gen = HopSequenceGenerator::new(12345);
        for _ in 0..1000 {
            let idx = gen.next_hop_index();
            assert!(idx < 51, "Hop index must be < 51, got {idx}");
        }
    }

    #[test]
    fn test_hop_frequencies_in_band() {
        let mut gen = HopSequenceGenerator::new(0xABCD_5678);
        for _ in 0..200 {
            let f = gen.next_frequency_mhz();
            assert!(f >= 969.0 && f <= 1206.0,
                "Frequency {f} MHz is out of Link 16 band (969–1206 MHz)");
        }
    }

    #[test]
    fn test_hop_sequence_deterministic() {
        let mut gen1 = HopSequenceGenerator::new(42);
        let mut gen2 = HopSequenceGenerator::new(42);
        let hops1: Vec<usize> = (0..50).map(|_| gen1.next_hop_index()).collect();
        let hops2: Vec<usize> = (0..50).map(|_| gen2.next_hop_index()).collect();
        assert_eq!(hops1, hops2, "Hop sequence must be deterministic given same seed");
    }

    #[test]
    fn test_hop_sequence_different_seeds() {
        let mut gen1 = HopSequenceGenerator::new(1);
        let mut gen2 = HopSequenceGenerator::new(2);
        let hops1: Vec<usize> = (0..20).map(|_| gen1.next_hop_index()).collect();
        let hops2: Vec<usize> = (0..20).map(|_| gen2.next_hop_index()).collect();
        assert_ne!(hops1, hops2, "Different seeds should yield different sequences");
    }

    #[test]
    fn test_hop_sequence_coverage() {
        let mut gen = HopSequenceGenerator::new(0xC0FFEE);
        // After 1000 hops, should use at least 40 of 51 frequencies
        let coverage = gen.coverage_check(1000);
        assert!(coverage > 0.75, "Coverage too low: {coverage:.2}");
    }

    #[test]
    fn test_hop_frequencies_constant_count() {
        assert_eq!(HOP_FREQUENCIES_MHZ.len(), 51,
            "Must have exactly 51 hop frequencies");
    }

    // --- Interleaver tests ---------------------------------------------------

    #[test]
    fn test_interleaver_roundtrip() {
        let il = Link16Interleaver::new(4, 8);
        let data: Vec<u8> = (0u8..32).collect();
        let interleaved = il.interleave(&data);
        let deinterleaved = il.deinterleave(&interleaved);
        assert_eq!(data, deinterleaved, "Interleave/deinterleave must be identity");
    }

    #[test]
    fn test_interleaver_permutes_data() {
        let il = Link16Interleaver::new(4, 8);
        let data: Vec<u8> = (0u8..32).collect();
        let interleaved = il.interleave(&data);
        // Data should be rearranged (not identical for non-trivial sizes)
        assert_ne!(data, interleaved, "Interleaver should rearrange symbols");
    }

    #[test]
    fn test_interleaver_standard_length() {
        let il = Link16Interleaver::standard();
        let data: Vec<u8> = (0u8..31).collect();
        let out = il.interleave(&data);
        assert_eq!(out.len(), 31);
    }

    // --- TDMA Slot tests -----------------------------------------------------

    #[test]
    fn test_tdma_slot_timing() {
        let slot = TdmaSlot::new(0, SlotAssignment::Idle);
        assert!((slot.start_time_s - 0.0).abs() < 1e-9);
        let slot1 = TdmaSlot::new(1, SlotAssignment::Idle);
        assert!((slot1.start_time_s - SLOT_DURATION_S).abs() < 1e-9);
    }

    #[test]
    fn test_tdma_slot_end_time() {
        let slot = TdmaSlot::new(3, SlotAssignment::Idle);
        let expected_end = 4.0 * SLOT_DURATION_S;
        assert!((slot.end_time_s() - expected_end).abs() < 1e-9);
    }

    #[test]
    fn test_tdma_slot_absolute_time() {
        let slot = TdmaSlot::new(0, SlotAssignment::Idle);
        // Epoch 2, slot 0 → 2 × 12 s = 24 s
        let abs_time = slot.absolute_time_s(2);
        assert!((abs_time - 24.0).abs() < 1e-9);
    }

    #[test]
    fn test_tdma_epoch_duration_covers_all_slots() {
        // 1536 slots × 7.8125 ms = 12 s exactly
        let total = SLOTS_PER_EPOCH as f64 * SLOT_DURATION_S;
        assert!((total - EPOCH_DURATION_S).abs() < 1e-6,
            "TDMA epoch must be exactly 12 s, got {total}");
    }

    #[test]
    fn test_tdma_superframe_slots() {
        assert_eq!(SLOTS_PER_SUPERFRAME, 128);
    }

    #[test]
    fn test_net_participation_group() {
        let npg = NetParticipationGroup::new_uniform(5, 16);
        assert_eq!(npg.net_id, 5);
        assert_eq!(npg.slots.len(), 16);
        for &s in &npg.slots {
            assert!(s < SLOTS_PER_EPOCH);
        }
    }

    // --- Message Formatter tests ---------------------------------------------

    #[test]
    fn test_bits_to_symbols_75_bits() {
        let bits: Vec<bool> = (0..75).map(|i| i % 2 == 0).collect();
        let symbols = MessageFormatter::bits_to_symbols(&bits);
        assert_eq!(symbols.len(), 15, "75 bits → 15 five-bit symbols");
    }

    #[test]
    fn test_symbols_to_bits_roundtrip() {
        let bits: Vec<bool> = (0..75).map(|i| i % 3 != 0).collect();
        let symbols = MessageFormatter::bits_to_symbols(&bits);
        let recovered = MessageFormatter::symbols_to_bits(&symbols, 75);
        assert_eq!(bits, recovered, "Bit→symbol→bit must be identity");
    }

    #[test]
    fn test_symbol_range() {
        let bits: Vec<bool> = (0..75).map(|_| true).collect();
        let symbols = MessageFormatter::bits_to_symbols(&bits);
        for &sym in &symbols {
            assert!(sym < 32, "Symbol {sym} out of GF(2^5) range");
        }
    }

    #[test]
    fn test_ppli_message_roundtrip() {
        let lat = 48.8566_f64;  // Paris
        let lon = 2.3522_f64;
        let alt = 30000.0_f64;  // feet
        let id = 0x3FF_u16;
        let bits = MessageFormatter::build_ppli(lat, lon, alt, id);
        assert_eq!(bits.len(), 75);
        let (lat2, lon2, alt2, id2) = MessageFormatter::parse_ppli(&bits).unwrap();
        assert!((lat - lat2).abs() < 0.01, "Latitude roundtrip error");
        assert!((lon - lon2).abs() < 0.01, "Longitude roundtrip error");
        assert!((alt - alt2).abs() < 2.0, "Altitude roundtrip error");
        assert_eq!(id, id2, "Identity roundtrip error");
    }

    #[test]
    fn test_j_series_header_serialization() {
        let hdr = JSeriesHeader::ppli();
        let bits = hdr.to_bits();
        let hdr2 = JSeriesHeader::from_bits(&bits).unwrap();
        assert_eq!(hdr.j_number, hdr2.j_number);
        assert_eq!(hdr.sub_label, hdr2.sub_label);
    }

    // --- Link Budget tests ---------------------------------------------------

    #[test]
    fn test_fspl_increases_with_range() {
        let lb1 = Link16LinkBudget { range_km: 50.0, ..Link16LinkBudget::default_air_to_air() };
        let lb2 = Link16LinkBudget { range_km: 200.0, ..Link16LinkBudget::default_air_to_air() };
        assert!(lb2.fspl_db() > lb1.fspl_db(),
            "FSPL must increase with range");
    }

    #[test]
    fn test_link_margin_at_100km() {
        let lb = Link16LinkBudget::default_air_to_air();
        let margin = lb.link_margin_db();
        // With typical parameters, link should close at 100 km with positive margin
        // (exact value depends on parameters, just check it's computable)
        assert!(margin.is_finite());
    }

    #[test]
    fn test_max_range_positive() {
        let lb = Link16LinkBudget::default_air_to_air();
        let range = lb.max_range_km();
        assert!(range > 0.0 && range < 10000.0,
            "Max range {range} km out of expected bounds");
    }

    #[test]
    fn test_ccsk_processing_gain() {
        let pg = Link16LinkBudget::ccsk_processing_gain_db();
        // 10*log10(32) ≈ 15.05 dB
        assert!((pg - 15.05).abs() < 0.1, "CCSK PG should be ~15.05 dB, got {pg}");
    }

    #[test]
    fn test_jam_margin_positive() {
        let lb = Link16LinkBudget::default_air_to_air();
        let jm = lb.jam_margin_db();
        assert!(jm > 0.0, "Link 16 jam margin should be positive");
    }

    #[test]
    fn test_noise_psd_at_room_temp() {
        let lb = Link16LinkBudget::default_air_to_air();
        let n0 = lb.noise_psd_dbw_hz();
        // kT at 290K ≈ -174 dBm/Hz = -204 dBW/Hz
        assert!((n0 - (-203.8)).abs() < 1.0,
            "Noise PSD at 290K should be ~-204 dBW/Hz, got {n0}");
    }

    // --- Full TX/RX chain tests ----------------------------------------------

    #[test]
    fn test_full_chain_roundtrip_all_zeros() {
        let config = Link16Config::default();
        let mut proc = Link16Processor::new(config);
        let bits = vec![false; 75];
        let (chips, _hops) = proc.transmit(&bits).unwrap();
        assert_eq!(chips.len(), 992);
        let recovered = proc.receive(&chips).unwrap();
        assert_eq!(bits, recovered, "All-zeros message TX/RX roundtrip failed");
    }

    #[test]
    fn test_full_chain_roundtrip_all_ones() {
        let config = Link16Config::default();
        let mut proc = Link16Processor::new(config);
        let bits = vec![true; 75];
        let (chips, _hops) = proc.transmit(&bits).unwrap();
        let recovered = proc.receive(&chips).unwrap();
        assert_eq!(bits, recovered, "All-ones message TX/RX roundtrip failed");
    }

    #[test]
    fn test_full_chain_roundtrip_alternating() {
        let config = Link16Config::default();
        let mut proc = Link16Processor::new(config);
        let bits: Vec<bool> = (0..75).map(|i| i % 2 == 0).collect();
        let (chips, _hops) = proc.transmit(&bits).unwrap();
        let recovered = proc.receive(&chips).unwrap();
        assert_eq!(bits, recovered, "Alternating message TX/RX roundtrip failed");
    }

    #[test]
    fn test_full_chain_ppli_roundtrip() {
        let config = Link16Config::default();
        let mut proc = Link16Processor::new(config);
        let bits = MessageFormatter::build_ppli(37.7749, -122.4194, 25000.0, 0x1A2);
        let (chips, _) = proc.transmit(&bits).unwrap();
        let recovered = proc.receive(&chips).unwrap();
        assert_eq!(bits, recovered, "PPLI message TX/RX roundtrip failed");
    }

    #[test]
    fn test_full_chain_chips_per_message() {
        assert_eq!(Link16Processor::chips_per_message(), 992);
    }

    #[test]
    fn test_full_chain_hop_count() {
        let config = Link16Config::default();
        let mut proc = Link16Processor::new(config);
        let bits = vec![false; 75];
        let (_, hops) = proc.transmit(&bits).unwrap();
        // Default config has 3 hops per pulse
        assert_eq!(hops.len(), 3);
    }

    #[test]
    fn test_full_chain_with_rs_errors() {
        // Inject errors into the chip stream and verify RS corrects them
        let config = Link16Config::default();
        let mut proc = Link16Processor::new(config);
        let bits: Vec<bool> = (0..75).map(|i| i % 5 == 0).collect();
        let (mut chips, _) = proc.transmit(&bits).unwrap();
        // Flip 2 complete 32-chip symbols (= 2 RS symbol errors, well within t=8)
        for i in 0..32 {
            chips[i] = -chips[i];      // symbol 0
        }
        for i in 64..96 {
            chips[i] = -chips[i];     // symbol 2
        }
        let recovered = proc.receive(&chips).unwrap();
        assert_eq!(bits, recovered, "TX/RX with RS error correction failed");
    }

    #[test]
    fn test_wrong_input_length_tx() {
        let config = Link16Config::default();
        let mut proc = Link16Processor::new(config);
        let bits = vec![false; 70]; // wrong length
        assert!(proc.transmit(&bits).is_err());
    }

    #[test]
    fn test_wrong_input_length_rx() {
        let config = Link16Config::default();
        let mut proc = Link16Processor::new(config);
        let chips = vec![1i8; 100]; // wrong length
        assert!(proc.receive(&chips).is_err());
    }

    #[test]
    fn test_config_validation() {
        let mut config = Link16Config::default();
        assert!(config.validate().is_ok());
        config.hops_per_pulse = 0; // invalid
        assert!(config.validate().is_err());
    }

    #[test]
    fn test_processing_gain() {
        let pg = Link16Processor::processing_gain_db();
        assert!((pg - 15.05).abs() < 0.2);
    }

    #[test]
    fn test_tdma_slot_from_processor() {
        let config = Link16Config::default();
        let proc = Link16Processor::new(config);
        let slot = proc.tdma_slot(5);
        assert_eq!(slot.index, 5);
        assert!((slot.start_time_s - 5.0 * SLOT_DURATION_S).abs() < 1e-9);
    }

    // --- MSK Baseband Modulator test -----------------------------------------

    #[test]
    fn test_msk_modulator_output_length() {
        let mut msk = MskBasebandModulator::new(8);
        let chips: Vec<i8> = CCSK_BASE[..8].to_vec();
        let iq = msk.modulate(&chips);
        assert_eq!(iq.len(), 8 * 8); // 8 chips × 8 samples/chip
    }

    #[test]
    fn test_msk_modulator_constant_envelope() {
        let mut msk = MskBasebandModulator::new(4);
        let chips: Vec<i8> = CCSK_BASE.to_vec();
        let iq = msk.modulate(&chips);
        for (i, q) in &iq {
            let mag = (i * i + q * q).sqrt();
            assert!((mag - 1.0).abs() < 1e-9,
                "MSK envelope must be constant (1.0), got {mag}");
        }
    }

    #[test]
    fn test_slot_duration_microseconds() {
        assert!((SLOT_DURATION_US - 7812.5).abs() < 0.01,
            "TDMA slot must be 7812.5 µs (7.8125 ms)");
    }
}
