//! Fountain Code — Rateless Erasure Codes (Luby Transform)
//!
//! Implements LT (Luby Transform) fountain codes, a class of rateless erasure
//! codes that generate a theoretically unlimited stream of encoded symbols from
//! K source symbols. Any K*(1+epsilon) received symbols suffice to recover the
//! original data, regardless of which specific symbols arrive. Ideal for
//! broadcast/multicast channels, deep-space links, and reliable file transfer.
//!
//! No direct GNU Radio equivalent.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::fountain_code::{LtEncoder, LtDecoder, LtConfig, DegreeDistribution};
//!
//! let config = LtConfig {
//!     num_source_symbols: 8,
//!     symbol_size_bytes: 4,
//!     degree_distribution: DegreeDistribution::RobustSoliton { c: 0.1, delta: 0.5 },
//!     prng_seed: 42,
//! };
//!
//! let mut encoder = LtEncoder::new(config.clone());
//! encoder.set_source_data(&[1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
//!                           13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24,
//!                           25, 26, 27, 28, 29, 30, 31, 32]);
//!
//! let mut decoder = LtDecoder::new(config);
//! for _ in 0..20 {
//!     let sym = encoder.generate_symbol();
//!     decoder.add_symbol(&sym);
//! }
//! assert!(decoder.recovery_fraction() > 0.0);
//! ```

/// LT code configuration.
#[derive(Debug, Clone)]
pub struct LtConfig {
    /// Number of source symbols (K).
    pub num_source_symbols: usize,
    /// Size of each symbol in bytes.
    pub symbol_size_bytes: usize,
    /// Degree distribution.
    pub degree_distribution: DegreeDistribution,
    /// PRNG seed for reproducible encoding.
    pub prng_seed: u64,
}

/// Degree distribution for LT codes.
#[derive(Debug, Clone)]
pub enum DegreeDistribution {
    /// Ideal Soliton distribution.
    IdealSoliton,
    /// Robust Soliton distribution with parameters c and delta.
    RobustSoliton { c: f64, delta: f64 },
    /// Custom CDF.
    Custom(Vec<f64>),
}

/// Decode status returned when adding symbols.
#[derive(Debug, Clone, PartialEq)]
pub enum DecodeStatus {
    /// Need more encoded symbols.
    NeedMore,
    /// Successfully decoded all source symbols.
    Decoded,
    /// Partial decoding progress.
    PartialProgress { decoded: usize, total: usize },
}

/// An encoded (output) symbol.
#[derive(Debug, Clone)]
pub struct EncodedSymbol {
    /// Unique symbol identifier.
    pub id: u32,
    /// Encoded data (XOR of selected source symbols).
    pub data: Vec<u8>,
    /// Degree of this symbol.
    pub degree: usize,
    /// Indices of source symbols XOR'd together.
    pub neighbors: Vec<usize>,
}

/// LT encoder.
#[derive(Debug, Clone)]
pub struct LtEncoder {
    config: LtConfig,
    /// Source data split into symbols.
    source_symbols: Vec<Vec<u8>>,
    /// CDF for degree sampling.
    degree_cdf: Vec<f64>,
    /// PRNG state.
    rng_state: u64,
    /// Next symbol ID.
    next_id: u32,
}

impl LtEncoder {
    /// Create a new LT encoder.
    pub fn new(config: LtConfig) -> Self {
        let k = config.num_source_symbols;
        let degree_cdf = build_cdf(&config.degree_distribution, k);

        Self {
            source_symbols: vec![vec![0u8; config.symbol_size_bytes]; k],
            degree_cdf,
            rng_state: config.prng_seed,
            next_id: 0,
            config,
        }
    }

    /// Set the source data to encode.
    pub fn set_source_data(&mut self, data: &[u8]) {
        let sym_size = self.config.symbol_size_bytes;
        let k = self.config.num_source_symbols;
        self.source_symbols = Vec::with_capacity(k);

        for i in 0..k {
            let start = i * sym_size;
            let end = (start + sym_size).min(data.len());
            let mut sym = vec![0u8; sym_size];
            if start < data.len() {
                let copy_len = end - start;
                sym[..copy_len].copy_from_slice(&data[start..end]);
            }
            self.source_symbols.push(sym);
        }
    }

    /// Generate one encoded symbol.
    pub fn generate_symbol(&mut self) -> EncodedSymbol {
        let id = self.next_id;
        self.next_id += 1;

        // Sample degree from distribution
        let degree = self.sample_degree();
        let degree = degree.min(self.config.num_source_symbols).max(1);

        // Select neighbors
        let neighbors = self.select_neighbors(degree);

        // XOR source symbols
        let sym_size = self.config.symbol_size_bytes;
        let mut data = vec![0u8; sym_size];
        for &idx in &neighbors {
            for j in 0..sym_size {
                data[j] ^= self.source_symbols[idx][j];
            }
        }

        EncodedSymbol {
            id,
            data,
            degree,
            neighbors,
        }
    }

    /// Generate N encoded symbols.
    pub fn generate_n_symbols(&mut self, n: usize) -> Vec<EncodedSymbol> {
        (0..n).map(|_| self.generate_symbol()).collect()
    }

    /// Expected overhead factor.
    pub fn overhead_factor(&self) -> f64 {
        let k = self.config.num_source_symbols as f64;
        match &self.config.degree_distribution {
            DegreeDistribution::IdealSoliton => 1.0 + (k.ln() / k.sqrt()),
            DegreeDistribution::RobustSoliton { c, delta } => {
                let r = *c * (k / *delta).ln() * k.sqrt();
                1.0 + r / k + (k.ln() * k.ln()) / k
            }
            DegreeDistribution::Custom(_) => 1.1,
        }
    }

    fn sample_degree(&mut self) -> usize {
        let u = self.next_random_f64();
        for (i, &cdf_val) in self.degree_cdf.iter().enumerate() {
            if u <= cdf_val {
                return i + 1; // degrees are 1-indexed
            }
        }
        self.config.num_source_symbols
    }

    fn select_neighbors(&mut self, degree: usize) -> Vec<usize> {
        let k = self.config.num_source_symbols;
        let mut neighbors = Vec::with_capacity(degree);

        // Fisher-Yates partial shuffle
        let mut indices: Vec<usize> = (0..k).collect();
        for i in 0..degree.min(k) {
            let j = i + (self.next_random_u64() as usize % (k - i));
            indices.swap(i, j);
            neighbors.push(indices[i]);
        }

        neighbors
    }

    fn next_random_u64(&mut self) -> u64 {
        // xorshift64
        self.rng_state ^= self.rng_state << 13;
        self.rng_state ^= self.rng_state >> 7;
        self.rng_state ^= self.rng_state << 17;
        self.rng_state
    }

    fn next_random_f64(&mut self) -> f64 {
        (self.next_random_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

/// LT decoder using belief propagation (peeling).
#[derive(Debug, Clone)]
pub struct LtDecoder {
    config: LtConfig,
    /// Decoded source symbols (None if not yet decoded).
    source_symbols: Vec<Option<Vec<u8>>>,
    /// Buffered encoded symbols (remaining neighbors and data).
    encoded_buffer: Vec<BufferedSymbol>,
    /// Number of decoded source symbols.
    num_decoded: usize,
}

#[derive(Debug, Clone)]
struct BufferedSymbol {
    data: Vec<u8>,
    remaining_neighbors: Vec<usize>,
}

impl LtDecoder {
    /// Create a new LT decoder.
    pub fn new(config: LtConfig) -> Self {
        let k = config.num_source_symbols;
        Self {
            source_symbols: vec![None; k],
            encoded_buffer: Vec::new(),
            num_decoded: 0,
            config,
        }
    }

    /// Add an encoded symbol and attempt belief propagation.
    pub fn add_symbol(&mut self, symbol: &EncodedSymbol) -> DecodeStatus {
        // Remove already-decoded neighbors from the symbol
        let mut data = symbol.data.clone();
        let mut remaining: Vec<usize> = Vec::new();

        for &idx in &symbol.neighbors {
            if idx < self.config.num_source_symbols {
                if let Some(ref decoded) = self.source_symbols[idx] {
                    // XOR out already-decoded symbol
                    for j in 0..data.len().min(decoded.len()) {
                        data[j] ^= decoded[j];
                    }
                } else {
                    remaining.push(idx);
                }
            }
        }

        if remaining.len() == 1 {
            // Degree-1: directly decode this source symbol
            let idx = remaining[0];
            self.source_symbols[idx] = Some(data);
            self.num_decoded += 1;
            self.propagate();
        } else if !remaining.is_empty() {
            self.encoded_buffer.push(BufferedSymbol {
                data,
                remaining_neighbors: remaining,
            });
        }

        if self.is_complete() {
            DecodeStatus::Decoded
        } else if self.num_decoded > 0 {
            DecodeStatus::PartialProgress {
                decoded: self.num_decoded,
                total: self.config.num_source_symbols,
            }
        } else {
            DecodeStatus::NeedMore
        }
    }

    /// Try to decode all source data.
    pub fn decode(&self) -> Option<Vec<u8>> {
        if !self.is_complete() {
            return None;
        }

        let mut data = Vec::with_capacity(
            self.config.num_source_symbols * self.config.symbol_size_bytes,
        );
        for sym in &self.source_symbols {
            if let Some(ref s) = sym {
                data.extend_from_slice(s);
            } else {
                return None;
            }
        }
        Some(data)
    }

    /// Check if all source symbols have been decoded.
    pub fn is_complete(&self) -> bool {
        self.num_decoded == self.config.num_source_symbols
    }

    /// Number of encoded symbols received.
    pub fn num_received(&self) -> usize {
        self.encoded_buffer.len() + self.num_decoded
    }

    /// Number of decoded source symbols.
    pub fn num_decoded_sources(&self) -> usize {
        self.num_decoded
    }

    /// Fraction of source symbols decoded.
    pub fn recovery_fraction(&self) -> f64 {
        self.num_decoded as f64 / self.config.num_source_symbols as f64
    }

    /// Reset decoder state.
    pub fn reset(&mut self) {
        self.source_symbols = vec![None; self.config.num_source_symbols];
        self.encoded_buffer.clear();
        self.num_decoded = 0;
    }

    /// Propagate newly decoded symbols through buffered encoded symbols.
    fn propagate(&mut self) {
        let mut changed = true;
        while changed {
            changed = false;
            let mut new_decoded: Vec<(usize, Vec<u8>)> = Vec::new();

            for sym in &mut self.encoded_buffer {
                // Remove decoded neighbors
                let mut i = 0;
                while i < sym.remaining_neighbors.len() {
                    let idx = sym.remaining_neighbors[i];
                    if let Some(ref decoded) = self.source_symbols[idx] {
                        for j in 0..sym.data.len().min(decoded.len()) {
                            sym.data[j] ^= decoded[j];
                        }
                        sym.remaining_neighbors.swap_remove(i);
                    } else {
                        i += 1;
                    }
                }

                if sym.remaining_neighbors.len() == 1 {
                    let idx = sym.remaining_neighbors[0];
                    if self.source_symbols[idx].is_none() {
                        new_decoded.push((idx, sym.data.clone()));
                    }
                    sym.remaining_neighbors.clear();
                }
            }

            for (idx, data) in new_decoded {
                if self.source_symbols[idx].is_none() {
                    self.source_symbols[idx] = Some(data);
                    self.num_decoded += 1;
                    changed = true;
                }
            }

            // Remove fully processed symbols
            self.encoded_buffer.retain(|s| !s.remaining_neighbors.is_empty());
        }
    }
}

/// Compute the Ideal Soliton distribution.
pub fn ideal_soliton_distribution(k: usize) -> Vec<f64> {
    let mut pmf = vec![0.0; k];
    pmf[0] = 1.0 / k as f64; // d=1
    for d in 2..=k {
        pmf[d - 1] = 1.0 / (d as f64 * (d as f64 - 1.0));
    }
    pmf
}

/// Compute the Robust Soliton distribution.
pub fn robust_soliton_distribution(k: usize, c: f64, delta: f64) -> Vec<f64> {
    let rho = ideal_soliton_distribution(k);
    let r = c * (k as f64 / delta).ln() * (k as f64).sqrt();
    let k_over_r = (k as f64 / r).floor() as usize;

    let mut tau = vec![0.0; k];
    for d in 1..=k {
        if d < k_over_r && k_over_r > 0 {
            tau[d - 1] = r / (d as f64 * k as f64);
        } else if d == k_over_r {
            tau[d - 1] = r * (r / delta).ln() / k as f64;
        }
    }

    // Combine and normalize
    let mut mu: Vec<f64> = rho.iter().zip(tau.iter()).map(|(r, t)| r + t).collect();
    let z: f64 = mu.iter().sum();
    if z > 0.0 {
        for v in &mut mu {
            *v /= z;
        }
    }

    mu
}

fn build_cdf(dist: &DegreeDistribution, k: usize) -> Vec<f64> {
    let pmf = match dist {
        DegreeDistribution::IdealSoliton => ideal_soliton_distribution(k),
        DegreeDistribution::RobustSoliton { c, delta } => {
            robust_soliton_distribution(k, *c, *delta)
        }
        DegreeDistribution::Custom(pmf) => pmf.clone(),
    };

    let mut cdf = Vec::with_capacity(pmf.len());
    let mut cumsum = 0.0;
    for p in &pmf {
        cumsum += p;
        cdf.push(cumsum);
    }

    // Ensure last value is 1.0
    if let Some(last) = cdf.last_mut() {
        *last = 1.0;
    }

    cdf
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ideal_soliton_sums_to_one() {
        let pmf = ideal_soliton_distribution(100);
        let sum: f64 = pmf.iter().sum();
        assert!(
            (sum - 1.0).abs() < 1e-10,
            "ideal soliton sum = {sum}"
        );
        assert!(
            (pmf[0] - 0.01).abs() < 1e-10,
            "rho(1) = {}, expected 0.01",
            pmf[0]
        );
    }

    #[test]
    fn test_robust_soliton_sums_to_one() {
        let pmf = robust_soliton_distribution(1000, 0.1, 0.5);
        let sum: f64 = pmf.iter().sum();
        assert!(
            (sum - 1.0).abs() < 1e-10,
            "robust soliton sum = {sum}"
        );
    }

    #[test]
    fn test_encode_decode_no_erasure() {
        let k = 32;
        let sym_size = 8;
        let config = LtConfig {
            num_source_symbols: k,
            symbol_size_bytes: sym_size,
            degree_distribution: DegreeDistribution::RobustSoliton { c: 0.1, delta: 0.5 },
            prng_seed: 42,
        };

        let source_data: Vec<u8> = (0..k * sym_size).map(|i| (i % 256) as u8).collect();

        let mut encoder = LtEncoder::new(config.clone());
        encoder.set_source_data(&source_data);

        let mut decoder = LtDecoder::new(config);

        // Generate enough symbols
        for _ in 0..k * 3 {
            let sym = encoder.generate_symbol();
            decoder.add_symbol(&sym);
            if decoder.is_complete() {
                break;
            }
        }

        assert!(decoder.is_complete(), "failed to decode");
        let decoded = decoder.decode().unwrap();
        assert_eq!(decoded, source_data);
    }

    #[test]
    fn test_decode_with_overhead() {
        let k = 16;
        let sym_size = 4;
        let config = LtConfig {
            num_source_symbols: k,
            symbol_size_bytes: sym_size,
            degree_distribution: DegreeDistribution::RobustSoliton { c: 0.2, delta: 0.5 },
            prng_seed: 123,
        };

        let source_data: Vec<u8> = (0..k * sym_size).map(|i| (i * 7 % 256) as u8).collect();

        let mut encoder = LtEncoder::new(config.clone());
        encoder.set_source_data(&source_data);

        let mut decoder = LtDecoder::new(config);

        // Generate symbols with 50% overhead
        let symbols = encoder.generate_n_symbols(k * 3);
        for sym in &symbols {
            decoder.add_symbol(sym);
            if decoder.is_complete() {
                break;
            }
        }

        assert!(decoder.is_complete());
        assert_eq!(decoder.decode().unwrap(), source_data);
    }

    #[test]
    fn test_is_complete_transitions() {
        let config = LtConfig {
            num_source_symbols: 4,
            symbol_size_bytes: 2,
            degree_distribution: DegreeDistribution::RobustSoliton { c: 0.3, delta: 0.5 },
            prng_seed: 99,
        };

        let source_data = vec![1u8, 2, 3, 4, 5, 6, 7, 8];
        let mut encoder = LtEncoder::new(config.clone());
        encoder.set_source_data(&source_data);

        let mut decoder = LtDecoder::new(config);
        assert!(!decoder.is_complete());

        for _ in 0..100 {
            let sym = encoder.generate_symbol();
            decoder.add_symbol(&sym);
            if decoder.is_complete() {
                break;
            }
        }

        assert!(decoder.is_complete());
    }

    #[test]
    fn test_need_more_with_few_symbols() {
        let config = LtConfig {
            num_source_symbols: 100,
            symbol_size_bytes: 4,
            degree_distribution: DegreeDistribution::RobustSoliton { c: 0.1, delta: 0.5 },
            prng_seed: 42,
        };

        let mut encoder = LtEncoder::new(config.clone());
        encoder.set_source_data(&vec![0u8; 400]);

        let mut decoder = LtDecoder::new(config);
        let sym = encoder.generate_symbol();
        let status = decoder.add_symbol(&sym);

        assert!(!decoder.is_complete());
        assert!(
            matches!(status, DecodeStatus::NeedMore | DecodeStatus::PartialProgress { .. }),
            "status = {status:?}"
        );
    }

    #[test]
    fn test_deterministic_prng() {
        let config = LtConfig {
            num_source_symbols: 10,
            symbol_size_bytes: 4,
            degree_distribution: DegreeDistribution::RobustSoliton { c: 0.1, delta: 0.5 },
            prng_seed: 42,
        };

        let data = vec![1u8; 40];
        let mut enc1 = LtEncoder::new(config.clone());
        enc1.set_source_data(&data);
        let sym1 = enc1.generate_symbol();

        let mut enc2 = LtEncoder::new(config);
        enc2.set_source_data(&data);
        let sym2 = enc2.generate_symbol();

        assert_eq!(sym1.neighbors, sym2.neighbors, "same seed should give same neighbors");
        assert_eq!(sym1.data, sym2.data);
    }

    #[test]
    fn test_degree_one_direct_decode() {
        let config = LtConfig {
            num_source_symbols: 4,
            symbol_size_bytes: 2,
            degree_distribution: DegreeDistribution::Custom(vec![1.0, 0.0, 0.0, 0.0]),
            prng_seed: 42,
        };

        let data = vec![0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11, 0x22];
        let mut encoder = LtEncoder::new(config.clone());
        encoder.set_source_data(&data);

        // With degree always 1, each symbol copies exactly one source
        let mut decoder = LtDecoder::new(config);
        for _ in 0..20 {
            let sym = encoder.generate_symbol();
            assert_eq!(sym.degree, 1);
            decoder.add_symbol(&sym);
            if decoder.is_complete() {
                break;
            }
        }

        assert!(decoder.is_complete());
        assert_eq!(decoder.decode().unwrap(), data);
    }

    #[test]
    fn test_recovery_fraction_monotonic() {
        let config = LtConfig {
            num_source_symbols: 20,
            symbol_size_bytes: 4,
            degree_distribution: DegreeDistribution::RobustSoliton { c: 0.2, delta: 0.5 },
            prng_seed: 77,
        };

        let data: Vec<u8> = (0..80).map(|i| i as u8).collect();
        let mut encoder = LtEncoder::new(config.clone());
        encoder.set_source_data(&data);

        let mut decoder = LtDecoder::new(config);
        let mut prev_frac = 0.0;

        for _ in 0..100 {
            let sym = encoder.generate_symbol();
            decoder.add_symbol(&sym);
            let frac = decoder.recovery_fraction();
            assert!(
                frac >= prev_frac,
                "fraction decreased: {prev_frac} -> {frac}"
            );
            prev_frac = frac;
            if decoder.is_complete() {
                break;
            }
        }
    }
}
