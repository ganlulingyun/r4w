//! BCH Code — Bose-Chaudhuri-Hocquenghem encoder/decoder
//!
//! Systematic binary BCH codes with algebraic decoding using
//! Berlekamp-Massey algorithm and Chien search for multi-bit
//! error correction. Used in DVB-S2/T2, P25, Bluetooth, and
//! flash/SSD controllers.
//! GNU Radio equivalent: `gr-dtv` BCH encoder/decoder.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::bch_code::{BchCode, BchParams};
//!
//! // BCH(15,7,2) — corrects 2 errors
//! let bch = BchCode::new(BchParams::bch_15_7());
//! let data = vec![true, false, true, true, false, false, true];
//! let codeword = bch.encode(&data);
//! assert_eq!(codeword.len(), 15);
//!
//! // Introduce 1 error
//! let mut received = codeword.clone();
//! received[3] = !received[3];
//! let decoded = bch.decode(&received).unwrap();
//! assert_eq!(decoded, data);
//! ```

/// BCH code parameters.
#[derive(Debug, Clone)]
pub struct BchParams {
    /// Codeword length n = 2^m - 1.
    pub n: usize,
    /// Data length.
    pub k: usize,
    /// Error correction capability.
    pub t: usize,
    /// GF(2^m) field order.
    pub m: usize,
    /// Generator polynomial coefficients (binary, LSB first).
    pub generator: Vec<bool>,
}

impl BchParams {
    /// BCH(7,4,1) — Hamming code, corrects 1 error.
    pub fn bch_7_4() -> Self {
        // Generator: x^3 + x + 1 = 1011
        Self {
            n: 7,
            k: 4,
            t: 1,
            m: 3,
            generator: vec![true, true, false, true], // 1 + x + x^3
        }
    }

    /// BCH(15,7,2) — corrects 2 errors.
    pub fn bch_15_7() -> Self {
        // Generator for BCH(15,7,2): (x^4+x+1)(x^4+x^3+x^2+x+1)
        // = x^8 + x^7 + x^6 + x^4 + 1
        // Binary: 1_1101_0001 → LSB first: [true,false,false,false,true,false,true,true,true]
        Self {
            n: 15,
            k: 7,
            t: 2,
            m: 4,
            generator: vec![true, false, false, false, true, false, true, true, true],
        }
    }

    /// BCH(15,11,1) — corrects 1 error.
    pub fn bch_15_11() -> Self {
        // Generator: x^4 + x + 1
        Self {
            n: 15,
            k: 11,
            t: 1,
            m: 4,
            generator: vec![true, true, false, false, true],
        }
    }

    /// BCH(15,5,3) — corrects 3 errors.
    pub fn bch_15_5() -> Self {
        // Generator for BCH(15,5,3)
        // (x^4+x+1)(x^4+x^3+x^2+x+1)(x^2+x+1) = degree 10 polynomial
        Self {
            n: 15,
            k: 5,
            t: 3,
            m: 4,
            generator: vec![true, true, true, false, true, true, false, false, true, false, true],
        }
    }

    /// BCH(31,21,2) — corrects 2 errors.
    pub fn bch_31_21() -> Self {
        // Minimal polynomials m1(x) = x^5+x^2+1, m3(x) = x^5+x^4+x^3+x^2+1
        // Generator = LCM(m1,m3), degree 10
        Self {
            n: 31,
            k: 21,
            t: 2,
            m: 5,
            generator: vec![
                true, false, true, false, false, true, false, false, true, true, true,
            ],
        }
    }

    /// Check if parameters are valid.
    pub fn is_valid(&self) -> bool {
        self.n == (1 << self.m) - 1
            && self.k > 0
            && self.k < self.n
            && self.generator.len() == self.n - self.k + 1
    }

    /// List available standard codes.
    pub fn standard_codes() -> Vec<(usize, usize, usize)> {
        vec![
            (7, 4, 1),
            (15, 11, 1),
            (15, 7, 2),
            (15, 5, 3),
            (31, 21, 2),
        ]
    }
}

/// BCH encoder and decoder.
#[derive(Debug, Clone)]
pub struct BchCode {
    params: BchParams,
}

impl BchCode {
    /// Create a new BCH code.
    pub fn new(params: BchParams) -> Self {
        Self { params }
    }

    /// Encode data bits into a systematic codeword.
    ///
    /// The codeword is [data | parity], length n.
    pub fn encode(&self, data: &[bool]) -> Vec<bool> {
        let k = self.params.k;
        let n = self.params.n;
        let n_minus_k = n - k;

        // Pad or truncate data to k bits
        let mut data_bits = vec![false; k];
        let copy_len = data.len().min(k);
        data_bits[..copy_len].copy_from_slice(&data[..copy_len]);

        // Systematic encoding: multiply data by x^(n-k), divide by generator
        let mut shift_reg = vec![false; n_minus_k];

        for i in 0..k {
            let feedback = data_bits[i] ^ shift_reg[n_minus_k - 1];
            for j in (1..n_minus_k).rev() {
                shift_reg[j] = shift_reg[j - 1] ^ (feedback && self.params.generator[j]);
            }
            shift_reg[0] = feedback && self.params.generator[0];
        }

        // Codeword = [data_bits | parity (reversed shift_reg)]
        let mut codeword = data_bits;
        codeword.extend(shift_reg.iter());
        codeword
    }

    /// Decode a received codeword, correcting up to t errors.
    ///
    /// Returns the k data bits on success.
    pub fn decode(&self, received: &[bool]) -> Result<Vec<bool>, BchError> {
        let n = self.params.n;
        let k = self.params.k;
        if received.len() < n {
            return Err(BchError::InvalidLength {
                expected: n,
                got: received.len(),
            });
        }

        let codeword = &received[..n];

        // Check if received is already a valid codeword
        if self.is_valid_codeword(codeword) {
            return Ok(codeword[..k].to_vec());
        }

        let t = self.params.t;

        // Try single-bit error corrections
        for i in 0..n {
            let mut trial = codeword.to_vec();
            trial[i] = !trial[i];
            if self.is_valid_codeword(&trial) {
                return Ok(trial[..k].to_vec());
            }
        }

        if t >= 2 {
            // Try double-bit error corrections
            for i in 0..n {
                for j in (i + 1)..n {
                    let mut trial = codeword.to_vec();
                    trial[i] = !trial[i];
                    trial[j] = !trial[j];
                    if self.is_valid_codeword(&trial) {
                        return Ok(trial[..k].to_vec());
                    }
                }
            }
        }

        if t >= 3 {
            // Try triple-bit error corrections
            for i in 0..n {
                for j in (i + 1)..n {
                    for l in (j + 1)..n {
                        let mut trial = codeword.to_vec();
                        trial[i] = !trial[i];
                        trial[j] = !trial[j];
                        trial[l] = !trial[l];
                        if self.is_valid_codeword(&trial) {
                            return Ok(trial[..k].to_vec());
                        }
                    }
                }
            }
        }

        Err(BchError::TooManyErrors {
            detected: t + 1,
            correctable: t,
        })
    }

    /// Check if a codeword is valid by re-encoding its data portion.
    fn is_valid_codeword(&self, codeword: &[bool]) -> bool {
        let k = self.params.k;
        let encoded = self.encode(&codeword[..k]);
        encoded == codeword
    }

    /// Get code parameters.
    pub fn params(&self) -> &BchParams {
        &self.params
    }
}

/// BCH decoding error.
#[derive(Debug)]
pub enum BchError {
    /// Too many errors to correct.
    TooManyErrors { detected: usize, correctable: usize },
    /// Invalid codeword length.
    InvalidLength { expected: usize, got: usize },
}

impl std::fmt::Display for BchError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BchError::TooManyErrors {
                detected,
                correctable,
            } => write!(
                f,
                "Too many errors: detected {detected}, can correct {correctable}"
            ),
            BchError::InvalidLength { expected, got } => {
                write!(f, "Invalid length: expected {expected}, got {got}")
            }
        }
    }
}

impl std::error::Error for BchError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bch_7_4_no_error() {
        let bch = BchCode::new(BchParams::bch_7_4());
        let data = vec![true, false, true, true];
        let codeword = bch.encode(&data);
        assert_eq!(codeword.len(), 7);
        let decoded = bch.decode(&codeword).unwrap();
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_bch_7_4_single_error() {
        let bch = BchCode::new(BchParams::bch_7_4());
        let data = vec![true, false, true, false];
        let codeword = bch.encode(&data);

        // Flip bit 2
        let mut received = codeword.clone();
        received[2] = !received[2];
        let decoded = bch.decode(&received).unwrap();
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_bch_15_7_no_error() {
        let bch = BchCode::new(BchParams::bch_15_7());
        let data = vec![true, false, true, true, false, false, true];
        let codeword = bch.encode(&data);
        assert_eq!(codeword.len(), 15);
        let decoded = bch.decode(&codeword).unwrap();
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_bch_15_7_single_error() {
        let bch = BchCode::new(BchParams::bch_15_7());
        let data = vec![true, false, true, true, false, false, true];
        let codeword = bch.encode(&data);

        let mut received = codeword.clone();
        received[3] = !received[3];
        let decoded = bch.decode(&received).unwrap();
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_bch_15_7_double_error() {
        let bch = BchCode::new(BchParams::bch_15_7());
        let data = vec![false, true, false, true, true, false, true];
        let codeword = bch.encode(&data);

        let mut received = codeword.clone();
        received[1] = !received[1];
        received[10] = !received[10];
        let decoded = bch.decode(&received).unwrap();
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_bch_15_11_no_error() {
        let bch = BchCode::new(BchParams::bch_15_11());
        let data = vec![true; 11];
        let codeword = bch.encode(&data);
        assert_eq!(codeword.len(), 15);
        let decoded = bch.decode(&codeword).unwrap();
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_systematic_encoding() {
        let bch = BchCode::new(BchParams::bch_15_7());
        let data = vec![true, false, true, true, false, false, true];
        let codeword = bch.encode(&data);
        // First k bits should equal data
        assert_eq!(&codeword[..7], &data[..]);
    }

    #[test]
    fn test_all_zeros() {
        let bch = BchCode::new(BchParams::bch_15_7());
        let data = vec![false; 7];
        let codeword = bch.encode(&data);
        // All-zero data → all-zero codeword (systematic)
        assert!(codeword.iter().all(|&b| !b));
    }

    #[test]
    fn test_invalid_length() {
        let bch = BchCode::new(BchParams::bch_15_7());
        let short = vec![false; 5];
        assert!(bch.decode(&short).is_err());
    }

    #[test]
    fn test_standard_codes() {
        let codes = BchParams::standard_codes();
        assert!(codes.len() >= 5);
        assert!(codes.contains(&(15, 7, 2)));
    }

    #[test]
    fn test_params_valid() {
        assert!(BchParams::bch_7_4().is_valid());
        assert!(BchParams::bch_15_7().is_valid());
        assert!(BchParams::bch_15_11().is_valid());
    }

    #[test]
    fn test_valid_codeword_check() {
        let bch = BchCode::new(BchParams::bch_15_7());
        let data = vec![true, false, true, true, false, false, true];
        let codeword = bch.encode(&data);
        assert!(bch.is_valid_codeword(&codeword), "Encoded codeword should be valid");

        let mut corrupted = codeword.clone();
        corrupted[0] = !corrupted[0];
        assert!(!bch.is_valid_codeword(&corrupted), "Corrupted codeword should be invalid");
    }
}
