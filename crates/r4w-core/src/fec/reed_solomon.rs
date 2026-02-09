//! Reed-Solomon Error-Correcting Code
//!
//! Systematic RS encoder and decoder over GF(2^8). Corrects up to `t = (n-k)/2`
//! symbol errors per block.
//!
//! ## Standard Configurations
//!
//! - RS(255,223) t=16 — CCSDS standard
//! - RS(255,239) t=8 — DVB, ATSC
//! - RS(255,249) t=3 — lightweight protection
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::fec::reed_solomon::{ReedSolomon, RsConfig};
//!
//! // CCSDS RS(255,223) corrects up to 16 symbol errors
//! let rs = ReedSolomon::new(RsConfig::ccsds());
//!
//! let mut message = vec![0u8; 223];
//! message[0..5].copy_from_slice(&[0x48, 0x65, 0x6C, 0x6C, 0x6F]); // "Hello" at start
//! let encoded = rs.encode(&message);
//!
//! // Introduce errors
//! let mut received = encoded.clone();
//! received[0] ^= 0xFF;
//! received[3] ^= 0xAB;
//!
//! let decoded = rs.decode(&received).expect("should correct 2 errors");
//! assert_eq!(&decoded[..5], &[0x48, 0x65, 0x6C, 0x6C, 0x6F]);
//! ```

/// GF(2^8) field element type.
type Gf = u8;

/// Primitive polynomial for GF(2^8): x^8 + x^4 + x^3 + x^2 + 1 = 0x11D
const PRIM_POLY: u16 = 0x11D;

/// GF(2^8) arithmetic tables.
#[derive(Clone)]
struct GfTables {
    exp: [Gf; 512], // exp[i] = alpha^i, doubled for easy mod
    log: [u16; 256], // log[x] = i where alpha^i = x (log[0] undefined)
}

impl GfTables {
    fn new() -> Self {
        let mut exp = [0u8; 512];
        let mut log = [0u16; 256];

        let mut x: u16 = 1;
        for i in 0..255u16 {
            exp[i as usize] = x as u8;
            log[x as usize] = i;
            x <<= 1;
            if x & 0x100 != 0 {
                x ^= PRIM_POLY;
            }
        }
        // Extend exp table for easy modular reduction
        for i in 255..512 {
            exp[i] = exp[i - 255];
        }

        Self { exp, log }
    }

    fn mul(&self, a: Gf, b: Gf) -> Gf {
        if a == 0 || b == 0 {
            return 0;
        }
        let idx = self.log[a as usize] as usize + self.log[b as usize] as usize;
        self.exp[idx]
    }

    fn div(&self, a: Gf, b: Gf) -> Gf {
        assert!(b != 0, "GF division by zero");
        if a == 0 {
            return 0;
        }
        let idx = (self.log[a as usize] as usize + 255 - self.log[b as usize] as usize) % 255;
        self.exp[idx]
    }

    fn pow(&self, a: Gf, p: usize) -> Gf {
        if a == 0 {
            return 0;
        }
        let idx = (self.log[a as usize] as usize * p) % 255;
        self.exp[idx]
    }

    fn inv(&self, a: Gf) -> Gf {
        assert!(a != 0, "GF inverse of zero");
        self.exp[255 - self.log[a as usize] as usize]
    }

    /// Evaluate polynomial at point x in GF(2^8).
    /// poly[0] is the highest-degree coefficient.
    fn poly_eval(&self, poly: &[Gf], x: Gf) -> Gf {
        let mut result: Gf = 0;
        for &coeff in poly {
            result = self.mul(result, x) ^ coeff;
        }
        result
    }
}

impl std::fmt::Debug for GfTables {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GfTables").finish()
    }
}

/// Configuration for Reed-Solomon codec.
#[derive(Debug, Clone)]
pub struct RsConfig {
    /// Total codeword length (n), typically 255 for GF(2^8)
    pub n: usize,
    /// Message length (k)
    pub k: usize,
    /// First consecutive root (fcr), typically 0 or 1
    pub fcr: usize,
    /// Primitive element (generator root), typically 2
    pub prim: usize,
}

impl RsConfig {
    /// CCSDS RS(255,223) t=16. Used in space communications.
    pub fn ccsds() -> Self {
        Self {
            n: 255,
            k: 223,
            fcr: 1,
            prim: 1,
        }
    }

    /// DVB RS(255,239) t=8. Used in digital video broadcasting.
    pub fn dvb() -> Self {
        Self {
            n: 255,
            k: 239,
            fcr: 0,
            prim: 1,
        }
    }

    /// Lightweight RS(255,249) t=3. Minimal overhead.
    pub fn lightweight() -> Self {
        Self {
            n: 255,
            k: 249,
            fcr: 1,
            prim: 1,
        }
    }

    /// Custom RS(n,k) configuration.
    pub fn custom(n: usize, k: usize) -> Self {
        assert!(n <= 255, "n must be <= 255 for GF(2^8)");
        assert!(k < n, "k must be < n");
        assert!((n - k) % 2 == 0, "n-k must be even");
        Self {
            n,
            k,
            fcr: 1,
            prim: 1,
        }
    }

    /// Error correction capability: t = (n-k)/2 symbols.
    pub fn t(&self) -> usize {
        (self.n - self.k) / 2
    }

    /// Number of parity symbols: 2t.
    pub fn parity_len(&self) -> usize {
        self.n - self.k
    }
}

/// Reed-Solomon encoder and decoder.
#[derive(Debug, Clone)]
pub struct ReedSolomon {
    config: RsConfig,
    gf: GfTables,
    /// Generator polynomial coefficients (highest degree first)
    generator: Vec<Gf>,
}

impl ReedSolomon {
    /// Create a new Reed-Solomon codec.
    pub fn new(config: RsConfig) -> Self {
        let gf = GfTables::new();
        let generator = Self::build_generator(&gf, &config);
        Self {
            config,
            gf,
            generator,
        }
    }

    /// Build the generator polynomial: prod_{i=0}^{2t-1} (x - alpha^(fcr + prim*i))
    fn build_generator(gf: &GfTables, config: &RsConfig) -> Vec<Gf> {
        let nsym = config.parity_len();
        // Start with g(x) = 1
        let mut gen = vec![0u8; nsym + 1];
        gen[nsym] = 1;

        for i in 0..nsym {
            let root = gf.exp[(config.fcr + config.prim * i) % 255];
            // Multiply gen by (x - root)
            let mut new_gen = vec![0u8; nsym + 1];
            for j in 0..=nsym {
                if gen[j] != 0 {
                    if j > 0 {
                        new_gen[j - 1] ^= gen[j]; // x * gen[j]
                    }
                    new_gen[j] ^= gf.mul(gen[j], root); // -root * gen[j], but in GF(2) minus is plus
                }
            }
            gen = new_gen;
        }

        gen
    }

    /// Encode a message, returning the full codeword (message + parity).
    ///
    /// Input message can be shorter than k; it will be zero-padded on the left.
    /// The output is always n bytes long.
    pub fn encode(&self, message: &[u8]) -> Vec<u8> {
        assert!(
            message.len() <= self.config.k,
            "Message too long: {} > {}",
            message.len(),
            self.config.k
        );

        let nsym = self.config.parity_len();

        // Systematic encoding: codeword = [message | parity]
        // Parity = message_poly * x^nsym mod generator
        let mut codeword = vec![0u8; self.config.n];

        // Copy message (left-padded with zeros if shorter than k)
        let offset = self.config.k - message.len();
        codeword[offset..self.config.k].copy_from_slice(message);

        // Compute parity by polynomial long division
        let mut remainder = vec![0u8; nsym];
        for i in 0..self.config.k {
            let feedback = codeword[i] ^ remainder[0];
            // Shift remainder left
            for j in 0..nsym - 1 {
                remainder[j] = remainder[j + 1];
            }
            remainder[nsym - 1] = 0;
            // Subtract feedback * generator
            if feedback != 0 {
                for j in 0..nsym {
                    remainder[j] ^= self.gf.mul(feedback, self.generator[j + 1]);
                }
            }
        }

        // Append parity
        codeword[self.config.k..].copy_from_slice(&remainder);
        codeword
    }

    /// Decode a received codeword, correcting up to t symbol errors.
    ///
    /// Returns the corrected message (k bytes) or an error if uncorrectable.
    pub fn decode(&self, received: &[u8]) -> Result<Vec<u8>, RsError> {
        if received.len() != self.config.n {
            return Err(RsError::InvalidLength {
                got: received.len(),
                expected: self.config.n,
            });
        }

        let mut codeword = received.to_vec();
        let nsym = self.config.parity_len();

        // Step 1: Compute syndromes
        let syndromes = self.compute_syndromes(&codeword);

        // Check if all syndromes are zero (no errors)
        if syndromes.iter().all(|&s| s == 0) {
            return Ok(codeword[..self.config.k].to_vec());
        }

        // Step 2: Berlekamp-Massey to find error locator polynomial
        let sigma = self.berlekamp_massey(&syndromes)?;

        // Step 3: Chien search to find error locations
        let error_positions = self.chien_search(&sigma)?;

        if error_positions.len() > self.config.t() {
            return Err(RsError::TooManyErrors {
                found: error_positions.len(),
                max: self.config.t(),
            });
        }

        // Step 4: Forney algorithm to find error magnitudes
        let error_magnitudes = self.forney(&syndromes, &sigma, &error_positions)?;

        // Step 5: Apply corrections
        for (&pos, &mag) in error_positions.iter().zip(error_magnitudes.iter()) {
            codeword[pos] ^= mag;
        }

        // Verify correction by re-checking syndromes
        let check = self.compute_syndromes(&codeword);
        if !check.iter().all(|&s| s == 0) {
            return Err(RsError::CorrectionFailed);
        }

        Ok(codeword[..self.config.k].to_vec())
    }

    /// Compute syndromes S_i = received(alpha^(fcr+i)) for i in 0..2t.
    fn compute_syndromes(&self, received: &[u8]) -> Vec<Gf> {
        let nsym = self.config.parity_len();
        let mut syndromes = vec![0u8; nsym];

        for i in 0..nsym {
            let root = self.gf.exp[(self.config.fcr + self.config.prim * i) % 255];
            syndromes[i] = self.gf.poly_eval(received, root);
        }

        syndromes
    }

    /// Berlekamp-Massey algorithm to find the error locator polynomial.
    ///
    /// Returns polynomial sigma where sigma[i] is the coefficient of x^i.
    /// sigma[0] = 1 always.
    fn berlekamp_massey(&self, syndromes: &[Gf]) -> Result<Vec<Gf>, RsError> {
        let nsym = syndromes.len();

        // C(x) = current error locator polynomial
        let mut c = vec![0u8; nsym + 1];
        c[0] = 1;
        // B(x) = previous error locator polynomial
        let mut b = vec![0u8; nsym + 1];
        b[0] = 1;

        let mut l: usize = 0; // number of errors found
        let mut m: usize = 1; // shift counter
        let mut bb: Gf = 1; // previous discrepancy

        for n in 0..nsym {
            // Compute discrepancy d = S[n] + sum_{i=1}^{L} C[i]*S[n-i]
            let mut d: Gf = syndromes[n];
            for i in 1..=l {
                d ^= self.gf.mul(c[i], syndromes[n - i]);
            }

            if d == 0 {
                m += 1;
            } else if 2 * l <= n {
                // Save C as T for later
                let t = c.clone();
                // C(x) = C(x) - (d/bb) * x^m * B(x)
                let coeff = self.gf.div(d, bb);
                for i in 0..=nsym {
                    if i + m <= nsym {
                        c[i + m] ^= self.gf.mul(coeff, b[i]);
                    }
                }
                l = n + 1 - l;
                b = t;
                bb = d;
                m = 1;
            } else {
                // C(x) = C(x) - (d/bb) * x^m * B(x)
                let coeff = self.gf.div(d, bb);
                for i in 0..=nsym {
                    if i + m <= nsym {
                        c[i + m] ^= self.gf.mul(coeff, b[i]);
                    }
                }
                m += 1;
            }
        }

        // Trim to actual degree
        let degree = c.iter().rposition(|&x| x != 0).unwrap_or(0);
        c.truncate(degree + 1);

        if c.len() - 1 > self.config.t() {
            return Err(RsError::TooManyErrors {
                found: c.len() - 1,
                max: self.config.t(),
            });
        }

        Ok(c)
    }

    /// Chien search: find roots of the error locator polynomial.
    ///
    /// Tests sigma(alpha^i) for i = 0..n-1. If sigma(alpha^i) = 0, then
    /// X_j^(-1) = alpha^i, so X_j = alpha^((255-i)%255). Since our codeword
    /// stores byte[0] as the x^(n-1) coefficient, the byte position is
    /// (n-1) - ((255-i)%255).
    fn chien_search(&self, sigma: &[Gf]) -> Result<Vec<usize>, RsError> {
        let num_errors = sigma.len() - 1;
        let mut positions = Vec::with_capacity(num_errors);

        for i in 0..self.config.n {
            let x = self.gf.exp[i % 255];
            let val = self.eval_poly(sigma, x);
            if val == 0 {
                // RS "power" position: p = (255 - i) % 255
                let p = (255 - i) % 255;
                if p < self.config.n {
                    positions.push(self.config.n - 1 - p);
                }
            }
        }

        if positions.len() != num_errors {
            return Err(RsError::ChienSearchFailed {
                found: positions.len(),
                expected: num_errors,
            });
        }

        positions.sort();
        Ok(positions)
    }

    /// Forney algorithm: compute error magnitudes at known error positions.
    fn forney(
        &self,
        syndromes: &[Gf],
        sigma: &[Gf],
        positions: &[usize],
    ) -> Result<Vec<Gf>, RsError> {
        let nsym = self.config.parity_len();

        // Compute error evaluator: Omega(x) = S(x) * sigma(x) mod x^nsym
        let mut omega = vec![0u8; nsym];
        for i in 0..nsym {
            for j in 0..sigma.len() {
                if j <= i {
                    omega[i] ^= self.gf.mul(sigma[j], syndromes[i - j]);
                }
            }
        }

        let mut magnitudes = Vec::with_capacity(positions.len());

        for &pos in positions {
            // RS power position: p = n-1-pos
            let p = self.config.n - 1 - pos;
            // X_j^(-1) = alpha^((255-p) % 255) = alpha^i where i was the Chien root
            let x_inv = self.gf.exp[(255 - p) % 255];

            // Evaluate Omega at X_j^(-1)
            let omega_val = self.eval_poly(&omega, x_inv);

            // Evaluate formal derivative sigma'(X_j^(-1))
            // In GF(2^m): d/dx x^k = x^(k-1) for odd k, 0 for even k
            // sigma'(x) = sigma[1] + sigma[3]*x^2 + sigma[5]*x^4 + ...
            let mut sigma_prime: Gf = 0;
            for k in (1..sigma.len()).step_by(2) {
                sigma_prime ^= self.gf.mul(sigma[k], self.gf.pow(x_inv, k - 1));
            }

            if sigma_prime == 0 {
                return Err(RsError::CorrectionFailed);
            }

            // Error magnitude: e_j = X_j^(1-fcr) * Omega(X_j^(-1)) / sigma'(X_j^(-1))
            // X_j = alpha^p
            let x_j_factor = if self.config.fcr == 1 {
                1u8 // X_j^(1-1) = X_j^0 = 1
            } else if self.config.fcr == 0 {
                self.gf.exp[p % 255] // X_j^(1-0) = X_j = alpha^p
            } else {
                // General: X_j^(1-fcr) = alpha^(p * (1-fcr) mod 255)
                let exp = ((p as i64) * (1 - self.config.fcr as i64)).rem_euclid(255) as usize;
                self.gf.exp[exp]
            };

            let magnitude =
                self.gf.div(self.gf.mul(x_j_factor, omega_val), sigma_prime);
            magnitudes.push(magnitude);
        }

        Ok(magnitudes)
    }

    /// Evaluate polynomial at a point. poly[0] is the constant term.
    fn eval_poly(&self, poly: &[Gf], x: Gf) -> Gf {
        let mut result: Gf = 0;
        let mut x_pow: Gf = 1;
        for &coeff in poly {
            result ^= self.gf.mul(coeff, x_pow);
            x_pow = self.gf.mul(x_pow, x);
        }
        result
    }

    /// Get configuration.
    pub fn config(&self) -> &RsConfig {
        &self.config
    }
}

/// Errors that can occur during RS decoding.
#[derive(Debug, Clone, PartialEq)]
pub enum RsError {
    /// Received word has wrong length.
    InvalidLength { got: usize, expected: usize },
    /// More errors than the code can correct.
    TooManyErrors { found: usize, max: usize },
    /// Chien search didn't find expected number of roots.
    ChienSearchFailed { found: usize, expected: usize },
    /// Error correction produced inconsistent result.
    CorrectionFailed,
}

impl std::fmt::Display for RsError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RsError::InvalidLength { got, expected } => {
                write!(f, "invalid codeword length: got {got}, expected {expected}")
            }
            RsError::TooManyErrors { found, max } => {
                write!(f, "too many errors: {found} > {max}")
            }
            RsError::ChienSearchFailed { found, expected } => {
                write!(
                    f,
                    "Chien search found {found} roots, expected {expected}"
                )
            }
            RsError::CorrectionFailed => write!(f, "error correction produced inconsistent result"),
        }
    }
}

impl std::error::Error for RsError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gf_tables_basic() {
        let gf = GfTables::new();
        // alpha^0 = 1
        assert_eq!(gf.exp[0], 1);
        // alpha^1 = 2
        assert_eq!(gf.exp[1], 2);
        // alpha^8 should wrap (primitive polynomial)
        assert_eq!(gf.exp[8], 0x1D); // x^8 mod (x^8+x^4+x^3+x^2+1) = x^4+x^3+x^2+1 = 29
        // alpha^255 = 1 (field order)
        assert_eq!(gf.exp[255], 1);
    }

    #[test]
    fn test_gf_multiply() {
        let gf = GfTables::new();
        // 0 * anything = 0
        assert_eq!(gf.mul(0, 42), 0);
        assert_eq!(gf.mul(42, 0), 0);
        // 1 * x = x
        assert_eq!(gf.mul(1, 42), 42);
        // Commutativity
        assert_eq!(gf.mul(3, 7), gf.mul(7, 3));
    }

    #[test]
    fn test_gf_inverse() {
        let gf = GfTables::new();
        // x * x^(-1) = 1
        for x in 1..=255u8 {
            let inv = gf.inv(x);
            assert_eq!(gf.mul(x, inv), 1, "inv({x}) = {inv} failed");
        }
    }

    #[test]
    fn test_rs_encode_no_errors() {
        let rs = ReedSolomon::new(RsConfig::ccsds());
        let message = vec![0x48, 0x65, 0x6C, 0x6C, 0x6F]; // "Hello"

        // Pad to k=223
        let mut padded = vec![0u8; 223];
        padded[218..223].copy_from_slice(&message);

        let encoded = rs.encode(&padded);
        assert_eq!(encoded.len(), 255);

        // No errors — decode should return message
        let decoded = rs.decode(&encoded).expect("no-error decode should succeed");
        assert_eq!(decoded, padded);
    }

    #[test]
    fn test_rs_correct_single_error() {
        let rs = ReedSolomon::new(RsConfig::dvb()); // t=8

        let mut message = vec![0u8; 239];
        for i in 0..message.len() {
            message[i] = (i & 0xFF) as u8;
        }

        let encoded = rs.encode(&message);

        // Introduce 1 error
        let mut received = encoded.clone();
        received[10] ^= 0xFF;

        let decoded = rs.decode(&received).expect("should correct 1 error");
        assert_eq!(decoded, message);
    }

    #[test]
    fn test_rs_correct_multiple_errors() {
        let rs = ReedSolomon::new(RsConfig::dvb()); // t=8

        let mut message = vec![0u8; 239];
        for i in 0..message.len() {
            message[i] = ((i * 7 + 3) & 0xFF) as u8;
        }

        let encoded = rs.encode(&message);

        // Introduce t errors (maximum correctable)
        let mut received = encoded.clone();
        for i in 0..8 {
            received[i * 30] ^= ((i + 1) * 0x11) as u8;
        }

        let decoded = rs.decode(&received).expect("should correct 8 errors");
        assert_eq!(decoded, message);
    }

    #[test]
    fn test_rs_detect_uncorrectable() {
        let rs = ReedSolomon::new(RsConfig::lightweight()); // t=3

        let message = vec![42u8; 249];
        let encoded = rs.encode(&message);

        // Introduce t+1 = 4 errors (uncorrectable)
        let mut received = encoded.clone();
        received[0] ^= 0xFF;
        received[1] ^= 0xFE;
        received[2] ^= 0xFD;
        received[3] ^= 0xFC;

        let result = rs.decode(&received);
        assert!(result.is_err(), "should detect uncorrectable errors");
    }

    #[test]
    fn test_rs_config_t() {
        assert_eq!(RsConfig::ccsds().t(), 16);
        assert_eq!(RsConfig::dvb().t(), 8);
        assert_eq!(RsConfig::lightweight().t(), 3);
        assert_eq!(RsConfig::custom(255, 223).t(), 16);
    }

    #[test]
    fn test_rs_encode_length() {
        let rs = ReedSolomon::new(RsConfig::ccsds());
        let message = vec![0u8; 223];
        let encoded = rs.encode(&message);
        assert_eq!(encoded.len(), 255);
        assert_eq!(encoded[..223], message[..]);
    }

    #[test]
    fn test_rs_invalid_length() {
        let rs = ReedSolomon::new(RsConfig::ccsds());
        let short = vec![0u8; 100];
        let result = rs.decode(&short);
        assert!(matches!(result, Err(RsError::InvalidLength { .. })));
    }
}
