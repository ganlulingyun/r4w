//! Reed-Solomon error correction codes over GF(2^8).
//!
//! Systematic Reed-Solomon encoder and decoder using the Berlekamp-Massey
//! algorithm, Chien search, and Forney algorithm. The field uses primitive
//! polynomial x^8 + x^4 + x^3 + x^2 + 1 (0x11D), which is standard for
//! DVB-S/S2, CCSDS (space telemetry), QR codes, and Data Matrix barcodes.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::reed_solomon::{ReedSolomonEncoder, ReedSolomonDecoder};
//!
//! let enc = ReedSolomonEncoder::new(255, 223);
//! let dec = ReedSolomonDecoder::new(255, 223);
//!
//! let data: Vec<u8> = (0..223).map(|i| i as u8).collect();
//! let codeword = enc.encode(&data);
//! assert_eq!(codeword.len(), 255);
//!
//! // Corrupt 5 symbols
//! let mut received = codeword;
//! for i in 0..5 {
//!     received[i * 10] ^= 0xAA;
//! }
//!
//! let errors = dec.decode(&mut received).unwrap();
//! assert_eq!(errors, 5);
//! assert_eq!(&received[..223], &(0..223).map(|i| i as u8).collect::<Vec<_>>()[..]);
//! ```

use std::fmt;

// ---------------------------------------------------------------------------
// GF(2^8) arithmetic with primitive polynomial 0x11D
// ---------------------------------------------------------------------------

const PRIM_POLY: u16 = 0x11D;
const GF_ORDER: usize = 255; // 2^8 - 1

/// Exponential table extended to 512 entries for modular-free lookup.
static GF_EXP: [u8; 512] = {
    let mut t = [0u8; 512];
    let mut v: u16 = 1;
    let mut i = 0;
    while i < 512 {
        t[i] = v as u8;
        v <<= 1;
        if v & 0x100 != 0 {
            v ^= PRIM_POLY;
        }
        i += 1;
    }
    t
};

/// Logarithm table. `GF_LOG[0]` is unused (set to 0).
static GF_LOG: [u8; 256] = {
    let mut t = [0u8; 256];
    let mut i = 0;
    while i < GF_ORDER {
        t[GF_EXP[i] as usize] = i as u8;
        i += 1;
    }
    t
};

#[inline(always)]
fn gf_add(a: u8, b: u8) -> u8 {
    a ^ b
}

#[inline(always)]
fn gf_mul(a: u8, b: u8) -> u8 {
    if a == 0 || b == 0 {
        0
    } else {
        GF_EXP[GF_LOG[a as usize] as usize + GF_LOG[b as usize] as usize]
    }
}

#[inline]
fn gf_div(a: u8, b: u8) -> u8 {
    if a == 0 {
        0
    } else {
        assert_ne!(b, 0, "GF division by zero");
        GF_EXP[(GF_LOG[a as usize] as usize + GF_ORDER - GF_LOG[b as usize] as usize) % GF_ORDER]
    }
}

#[inline]
fn gf_inv(a: u8) -> u8 {
    assert_ne!(a, 0, "GF inverse of zero");
    GF_EXP[GF_ORDER - GF_LOG[a as usize] as usize]
}

#[inline]
fn gf_pow(n: usize) -> u8 {
    GF_EXP[n % GF_ORDER]
}

// ---------------------------------------------------------------------------
// Polynomial operations -- ascending order: p[i] = coefficient of x^i
// ---------------------------------------------------------------------------

fn poly_eval(p: &[u8], x: u8) -> u8 {
    let mut acc: u8 = 0;
    for &c in p.iter().rev() {
        acc = gf_add(gf_mul(acc, x), c);
    }
    acc
}

fn poly_mul(a: &[u8], b: &[u8]) -> Vec<u8> {
    if a.is_empty() || b.is_empty() {
        return vec![];
    }
    let mut out = vec![0u8; a.len() + b.len() - 1];
    for (i, &ai) in a.iter().enumerate() {
        if ai != 0 {
            for (j, &bj) in b.iter().enumerate() {
                out[i + j] ^= gf_mul(ai, bj);
            }
        }
    }
    out
}

fn poly_add(a: &[u8], b: &[u8]) -> Vec<u8> {
    let len = a.len().max(b.len());
    let mut out = vec![0u8; len];
    for (i, &v) in a.iter().enumerate() {
        out[i] ^= v;
    }
    for (i, &v) in b.iter().enumerate() {
        out[i] ^= v;
    }
    out
}

fn poly_scale(p: &[u8], s: u8) -> Vec<u8> {
    p.iter().map(|&c| gf_mul(c, s)).collect()
}

/// Formal derivative in GF(2^m): odd-degree terms survive, even vanish.
fn poly_deriv(p: &[u8]) -> Vec<u8> {
    if p.len() <= 1 {
        return vec![0];
    }
    let mut d = Vec::with_capacity(p.len() - 1);
    for i in 1..p.len() {
        if i & 1 == 1 {
            d.push(p[i]);
        } else {
            d.push(0);
        }
    }
    while d.len() > 1 && *d.last().unwrap() == 0 {
        d.pop();
    }
    d
}

// ---------------------------------------------------------------------------
// Error type
// ---------------------------------------------------------------------------

/// Reed-Solomon decoding error.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RsError {
    /// More errors than the code can correct.
    TooManyErrors,
    /// Input length does not match the code's `n`.
    InvalidLength { expected: usize, got: usize },
}

impl fmt::Display for RsError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            RsError::TooManyErrors => write!(f, "too many errors to correct"),
            RsError::InvalidLength { expected, got } => {
                write!(f, "expected length {}, got {}", expected, got)
            }
        }
    }
}

impl std::error::Error for RsError {}

// ---------------------------------------------------------------------------
// Generator polynomial
// ---------------------------------------------------------------------------

/// g(x) = prod_{i=1}^{nsym} (x - alpha^i).
/// Ascending order: g[0] is constant, g[nsym] = 1.
fn build_generator(nsym: usize) -> Vec<u8> {
    let mut g = vec![gf_pow(1), 1]; // (x + alpha^1)
    for i in 2..=nsym {
        g = poly_mul(&g, &[gf_pow(i), 1]);
    }
    g
}

// ---------------------------------------------------------------------------
// Encoder
// ---------------------------------------------------------------------------

/// Reed-Solomon systematic encoder.
///
/// Codeword = `[data_0 .. data_{k-1}, parity_0 .. parity_{nsym-1}]`.
#[derive(Clone)]
pub struct ReedSolomonEncoder {
    n: usize,
    k: usize,
    nsym: usize,
    gen: Vec<u8>, // ascending order
}

impl ReedSolomonEncoder {
    /// Create an RS(n, k) encoder.
    pub fn new(n: usize, k: usize) -> Self {
        assert!(n <= 255 && k < n && k > 0, "invalid RS({},{}) parameters", n, k);
        let nsym = n - k;
        Self { n, k, nsym, gen: build_generator(nsym) }
    }

    /// Maximum correctable errors t = (n-k)/2.
    pub fn max_errors(&self) -> usize {
        self.nsym / 2
    }

    /// Maximum correctable erasures = n-k.
    pub fn max_erasures(&self) -> usize {
        self.nsym
    }

    /// Encode `data` (length `k`) into systematic codeword of length `n`.
    pub fn encode(&self, data: &[u8]) -> Vec<u8> {
        assert_eq!(data.len(), self.k, "data length must be k={}", self.k);

        // Standard LFSR-based systematic encoding:
        // We treat data[0] as the coefficient of x^{n-1} in the message
        // polynomial. We divide M(x)*x^nsym by g(x); the remainder is the
        // parity, and the codeword is [data | parity].
        //
        // Using a feedback shift register approach.
        let mut feedback = vec![0u8; self.nsym];

        for i in 0..self.k {
            let d = gf_add(data[i], feedback[self.nsym - 1]);
            // Shift and feed back
            for j in (1..self.nsym).rev() {
                feedback[j] = gf_add(feedback[j - 1], gf_mul(d, self.gen[j]));
            }
            feedback[0] = gf_mul(d, self.gen[0]);
        }

        let mut codeword = Vec::with_capacity(self.n);
        codeword.extend_from_slice(data);
        // Parity bytes: feedback[nsym-1] is the highest-power remainder coeff,
        // which follows right after data.
        for j in (0..self.nsym).rev() {
            codeword.push(feedback[j]);
        }
        codeword
    }
}

impl fmt::Debug for ReedSolomonEncoder {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "ReedSolomonEncoder(RS({},{}))", self.n, self.k)
    }
}

// ---------------------------------------------------------------------------
// Decoder
// ---------------------------------------------------------------------------

/// Reed-Solomon decoder using Berlekamp-Massey, Chien search, and Forney.
///
/// Corrects up to t = (n-k)/2 symbol errors in-place.
#[derive(Clone)]
pub struct ReedSolomonDecoder {
    n: usize,
    #[allow(dead_code)]
    k: usize,
    nsym: usize,
}

impl ReedSolomonDecoder {
    /// Create an RS(n, k) decoder.
    pub fn new(n: usize, k: usize) -> Self {
        assert!(n <= 255 && k < n && k > 0, "invalid RS({},{}) parameters", n, k);
        Self { n, k, nsym: n - k }
    }

    /// Maximum correctable symbol errors.
    pub fn max_errors(&self) -> usize {
        self.nsym / 2
    }

    /// Maximum correctable erasures.
    pub fn max_erasures(&self) -> usize {
        self.nsym
    }

    /// Decode `received` in-place. Returns the number of corrected errors.
    pub fn decode(&self, received: &mut [u8]) -> Result<usize, RsError> {
        if received.len() != self.n {
            return Err(RsError::InvalidLength {
                expected: self.n,
                got: received.len(),
            });
        }

        // --- Syndromes ---
        // The codeword polynomial is c(x) = c[0]*x^{n-1} + c[1]*x^{n-2} + ... + c[n-1].
        // Syndromes: S_j = r(alpha^j) for j = 1..nsym.
        // We store them as synd[j] = S_{j+1} for convenience (0-indexed).
        let mut synd = vec![0u8; self.nsym];
        for j in 0..self.nsym {
            let a = gf_pow(j + 1);
            let mut val: u8 = 0;
            for &ri in received.iter() {
                val = gf_add(gf_mul(val, a), ri);
            }
            synd[j] = val;
        }

        if synd.iter().all(|&s| s == 0) {
            return Ok(0);
        }

        // --- Berlekamp-Massey ---
        // Find error locator sigma(x) in ascending coefficients, sigma[0]=1.
        let sigma = {
            let mut c_poly = vec![1u8]; // current
            let mut b_poly = vec![1u8]; // previous best
            let mut l: usize = 0;
            let mut delta_b: u8 = 1;
            let mut m: usize = 1;

            for step in 0..self.nsym {
                // Discrepancy
                let mut delta: u8 = synd[step];
                for i in 1..c_poly.len() {
                    if step >= i {
                        delta ^= gf_mul(c_poly[i], synd[step - i]);
                    }
                }

                if delta == 0 {
                    m += 1;
                } else if 2 * l <= step {
                    let factor = gf_div(delta, delta_b);
                    let mut xm_b = vec![0u8; m];
                    xm_b.extend(poly_scale(&b_poly, factor));
                    let t_poly = poly_add(&c_poly, &xm_b);
                    b_poly = c_poly;
                    c_poly = t_poly;
                    l = step + 1 - l;
                    delta_b = delta;
                    m = 1;
                } else {
                    let factor = gf_div(delta, delta_b);
                    let mut xm_b = vec![0u8; m];
                    xm_b.extend(poly_scale(&b_poly, factor));
                    c_poly = poly_add(&c_poly, &xm_b);
                    m += 1;
                }
            }
            // Trim trailing zeros
            while c_poly.len() > 1 && *c_poly.last().unwrap() == 0 {
                c_poly.pop();
            }
            c_poly
        };

        let num_errors = sigma.len() - 1;
        if num_errors == 0 || num_errors > self.nsym / 2 {
            return Err(RsError::TooManyErrors);
        }

        // --- Chien search ---
        // sigma(X_j^{-1}) = 0  where X_j = alpha^{e_j} and e_j is the
        // "power index" of the error position.
        //
        // Our codeword polynomial has r[0] as the coeff of x^{n-1}, so
        // array position `pos` corresponds to power index `n-1-pos`.
        // We need X_j = alpha^{n-1-pos}, so X_j^{-1} = alpha^{-(n-1-pos)}
        //   = alpha^{GF_ORDER - n + 1 + pos}.
        //
        // We test all positions 0..n:
        let mut err_pos = Vec::with_capacity(num_errors);
        let mut err_x_inv = Vec::with_capacity(num_errors);

        for pos in 0..self.n {
            let x_inv = gf_pow(pos + GF_ORDER - (self.n - 1));
            if poly_eval(&sigma, x_inv) == 0 {
                err_pos.push(pos);
                err_x_inv.push(x_inv);
            }
        }

        if err_pos.len() != num_errors {
            return Err(RsError::TooManyErrors);
        }

        // --- Forney algorithm ---
        // Omega(x) = S(x) * Sigma(x) mod x^{nsym}
        // where S(x) = S_1 + S_2*x + ... + S_{nsym}*x^{nsym-1}  (ascending)
        //
        // For FCR (first consecutive root) = 1, the Forney formula is:
        //   e_j = X_j^{1-FCR} * Omega(X_j^{-1}) / Sigma'(X_j^{-1})
        //       = Omega(X_j^{-1}) / Sigma'(X_j^{-1})
        // (since X_j^0 = 1, and -1 = 1 in characteristic 2).

        let s_poly: Vec<u8> = synd.clone();

        let omega_full = poly_mul(&s_poly, &sigma);
        let omega: Vec<u8> = omega_full[..omega_full.len().min(self.nsym)].to_vec();

        let sigma_prime = poly_deriv(&sigma);

        for (idx, &pos) in err_pos.iter().enumerate() {
            let x_inv = err_x_inv[idx];

            let omega_val = poly_eval(&omega, x_inv);
            let sigma_p_val = poly_eval(&sigma_prime, x_inv);

            if sigma_p_val == 0 {
                return Err(RsError::TooManyErrors);
            }

            let magnitude = gf_div(omega_val, sigma_p_val);
            received[pos] ^= magnitude;
        }

        Ok(err_pos.len())
    }
}

impl fmt::Debug for ReedSolomonDecoder {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "ReedSolomonDecoder(RS({},{}))", self.n, self.k)
    }
}

// ---------------------------------------------------------------------------
// Presets
// ---------------------------------------------------------------------------

/// RS(255,223) encoder/decoder pair (DVB-S / CCSDS standard).
/// Corrects up to 16 symbol errors per codeword.
pub fn rs_255_223() -> (ReedSolomonEncoder, ReedSolomonDecoder) {
    (
        ReedSolomonEncoder::new(255, 223),
        ReedSolomonDecoder::new(255, 223),
    )
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gf_arithmetic() {
        assert_eq!(gf_add(0x53, 0xCA), 0x53 ^ 0xCA);
        assert_eq!(gf_add(0, 0xFF), 0xFF);
        assert_eq!(gf_add(0xAB, 0xAB), 0);
        assert_eq!(gf_mul(1, 0x53), 0x53);
        assert_eq!(gf_mul(0x53, 1), 0x53);
        assert_eq!(gf_mul(0, 0x53), 0);
        assert_eq!(gf_mul(0x12, 0x34), gf_mul(0x34, 0x12));
        for a in 1..=255u8 {
            assert_eq!(gf_mul(a, gf_inv(a)), 1, "inverse failed for {}", a);
        }
        assert_eq!(gf_div(gf_mul(0x53, 0xCA), 0xCA), 0x53);
        assert_eq!(gf_pow(0), 1);
        assert_eq!(gf_pow(255), 1);
    }

    #[test]
    fn test_encode_decode_roundtrip() {
        let enc = ReedSolomonEncoder::new(15, 9);
        let dec = ReedSolomonDecoder::new(15, 9);
        let data = vec![1, 2, 3, 4, 5, 6, 7, 8, 9];
        let codeword = enc.encode(&data);
        assert_eq!(codeword.len(), 15);
        assert_eq!(&codeword[..9], &data[..]);
        let mut received = codeword;
        let errors = dec.decode(&mut received).unwrap();
        assert_eq!(errors, 0);
        assert_eq!(&received[..9], &data[..]);
    }

    #[test]
    fn test_single_error_correction() {
        let enc = ReedSolomonEncoder::new(15, 9);
        let dec = ReedSolomonDecoder::new(15, 9);
        let data = vec![10, 20, 30, 40, 50, 60, 70, 80, 90];
        let codeword = enc.encode(&data);
        for pos in 0..15 {
            let mut received = codeword.clone();
            received[pos] ^= 0x55;
            let errors = dec.decode(&mut received).unwrap();
            assert_eq!(errors, 1, "failed at position {}", pos);
            assert_eq!(&received[..9], &data[..], "data mismatch at position {}", pos);
        }
    }

    #[test]
    fn test_multiple_error_correction() {
        let enc = ReedSolomonEncoder::new(15, 9);
        let dec = ReedSolomonDecoder::new(15, 9);
        assert_eq!(dec.max_errors(), 3);
        let data = vec![0xDE, 0xAD, 0xBE, 0xEF, 0xCA, 0xFE, 0xBA, 0xBE, 0x42];
        let codeword = enc.encode(&data);
        let mut received = codeword;
        received[0] ^= 0xFF;
        received[7] ^= 0xAA;
        received[14] ^= 0x33;
        let errors = dec.decode(&mut received).unwrap();
        assert_eq!(errors, 3);
        assert_eq!(&received[..9], &data[..]);
    }

    #[test]
    fn test_max_errors_correction() {
        let enc = ReedSolomonEncoder::new(255, 223);
        let dec = ReedSolomonDecoder::new(255, 223);
        assert_eq!(enc.max_errors(), 16);
        assert_eq!(dec.max_errors(), 16);
        assert_eq!(enc.max_erasures(), 32);
        let data: Vec<u8> = (0..223).map(|i| i as u8).collect();
        let codeword = enc.encode(&data);
        let mut received = codeword;
        for i in 0..16 {
            received[i * 15] ^= ((i + 1) as u8) | 0x80;
        }
        let errors = dec.decode(&mut received).unwrap();
        assert_eq!(errors, 16);
        assert_eq!(&received[..223], &data[..]);
    }

    #[test]
    fn test_too_many_errors() {
        let enc = ReedSolomonEncoder::new(15, 9);
        let dec = ReedSolomonDecoder::new(15, 9);
        let data = vec![1, 2, 3, 4, 5, 6, 7, 8, 9];
        let codeword = enc.encode(&data);
        let mut received = codeword;
        received[0] ^= 0xFF;
        received[3] ^= 0xAA;
        received[7] ^= 0x55;
        received[11] ^= 0x33;
        assert!(dec.decode(&mut received).is_err());
    }

    #[test]
    fn test_all_zero_data() {
        let enc = ReedSolomonEncoder::new(15, 9);
        let dec = ReedSolomonDecoder::new(15, 9);
        let data = vec![0u8; 9];
        let codeword = enc.encode(&data);
        assert!(codeword.iter().all(|&b| b == 0));
        let mut received = codeword;
        assert_eq!(dec.decode(&mut received).unwrap(), 0);
    }

    #[test]
    fn test_preset_rs_255_223() {
        let (enc, dec) = rs_255_223();
        assert_eq!(enc.max_errors(), 16);
        assert_eq!(dec.max_errors(), 16);
        assert_eq!(enc.max_erasures(), 32);
        assert_eq!(dec.max_erasures(), 32);
        let data: Vec<u8> = (0..223).map(|i| (i * 7 + 3) as u8).collect();
        let codeword = enc.encode(&data);
        assert_eq!(codeword.len(), 255);
        assert_eq!(&codeword[..223], &data[..]);
        let mut received = codeword;
        received[100] ^= 0xBB;
        received[200] ^= 0xCC;
        let errors = dec.decode(&mut received).unwrap();
        assert_eq!(errors, 2);
        assert_eq!(&received[..223], &data[..]);
    }

    #[test]
    fn test_systematic_encoding() {
        let enc = ReedSolomonEncoder::new(31, 19);
        let data: Vec<u8> = (100..119).collect();
        let codeword = enc.encode(&data);
        assert_eq!(codeword.len(), 31);
        assert_eq!(&codeword[..19], &data[..], "systematic: data must appear first");
    }

    #[test]
    fn test_invalid_length() {
        let dec = ReedSolomonDecoder::new(15, 9);
        let mut short = vec![0u8; 10];
        assert_eq!(
            dec.decode(&mut short),
            Err(RsError::InvalidLength { expected: 15, got: 10 })
        );
    }

    #[test]
    fn test_various_error_patterns() {
        let enc = ReedSolomonEncoder::new(31, 19);
        let dec = ReedSolomonDecoder::new(31, 19);
        let t = dec.max_errors(); // 6
        let data: Vec<u8> = (0..19).map(|i| (i * 13 + 7) as u8).collect();
        for num_errors in 1..=t {
            let codeword = enc.encode(&data);
            let mut received = codeword;
            for e in 0..num_errors {
                received[e * 4] ^= 0xAA;
            }
            let corrected = dec.decode(&mut received).unwrap();
            assert_eq!(corrected, num_errors, "wrong count for {} errors", num_errors);
            assert_eq!(&received[..19], &data[..], "data mismatch for {} errors", num_errors);
        }
    }
}
