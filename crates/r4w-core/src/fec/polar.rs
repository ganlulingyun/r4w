//! Polar Code Encoder/Decoder — 5G NR FEC
//!
//! Polar codes achieve channel capacity with successive cancellation (SC) decoding.
//! Used in 5G NR control channels (3GPP TS 38.212).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::fec::polar::{PolarEncoder, PolarDecoder, PolarConfig, PolarDecoderType};
//!
//! let config = PolarConfig::new(16, 8, 0.0);
//! let encoder = PolarEncoder::new(config.clone());
//! let info = vec![true, false, true, true, false, true, false, true];
//! let codeword = encoder.encode(&info);
//! assert_eq!(codeword.len(), 16);
//!
//! let decoder = PolarDecoder::new(config, PolarDecoderType::SC);
//! let decoded = decoder.decode_hard(&codeword);
//! assert_eq!(decoded, info);
//! ```

/// Polar code configuration.
#[derive(Debug, Clone)]
pub struct PolarConfig {
    /// Block size N (must be power of 2).
    pub block_size: usize,
    /// Number of information bits K.
    pub num_info_bits: usize,
    /// Frozen bit positions (sorted).
    pub frozen_positions: Vec<usize>,
    /// Information bit positions (sorted).
    pub info_positions: Vec<usize>,
}

impl PolarConfig {
    /// Create a polar code config with Bhattacharyya-based channel construction.
    ///
    /// `n` must be a power of 2. `k` info bits out of `n` total.
    /// `design_snr_db` is used for frozen bit selection.
    pub fn new(n: usize, k: usize, design_snr_db: f64) -> Self {
        assert!(n.is_power_of_two(), "Block size must be a power of 2");
        assert!(k <= n, "K must be <= N");

        let frozen_positions = Self::select_frozen_bits(n, k, design_snr_db);
        let mut info_positions: Vec<usize> = (0..n)
            .filter(|i| !frozen_positions.contains(i))
            .collect();
        info_positions.sort_unstable();

        Self {
            block_size: n,
            num_info_bits: k,
            frozen_positions,
            info_positions,
        }
    }

    /// Select frozen bit positions using Bhattacharyya parameter.
    fn select_frozen_bits(n: usize, k: usize, design_snr_db: f64) -> Vec<usize> {
        let snr_linear = 10.0_f64.powf(design_snr_db / 10.0);
        let initial_z = (-snr_linear).exp();

        // Compute Bhattacharyya parameters for each bit channel
        let mut z = vec![initial_z; n];

        let log_n = (n as f64).log2() as usize;
        for stage in 0..log_n {
            let half = 1 << stage;
            let mut new_z = vec![0.0; n];
            for i in 0..(n / (2 * half)) {
                for j in 0..half {
                    let idx = i * 2 * half + j;
                    let z1 = z[idx];
                    let z2 = z[idx + half];
                    // Upper channel (worse): z_upper = 2*z - z^2
                    new_z[idx] = (2.0 * z1 * z2 - z1 * z1 * z2 * z2).min(1.0).max(0.0);
                    // More accurate: z_upper = z1 + z2 - z1*z2
                    new_z[idx] = (z1 + z2 - z1 * z2).min(1.0).max(0.0);
                    // Lower channel (better): z_lower = z^2
                    new_z[idx + half] = (z1 * z2).min(1.0).max(0.0);
                }
            }
            z = new_z;
        }

        // Sort channels by reliability (higher z = less reliable = freeze)
        let mut indices: Vec<usize> = (0..n).collect();
        indices.sort_by(|&a, &b| z[b].partial_cmp(&z[a]).unwrap_or(std::cmp::Ordering::Equal));

        // Freeze the N-K least reliable channels
        let mut frozen: Vec<usize> = indices.into_iter().take(n - k).collect();
        frozen.sort_unstable();
        frozen
    }

    /// Code rate K/N.
    pub fn rate(&self) -> f64 {
        self.num_info_bits as f64 / self.block_size as f64
    }
}

/// Polar encoder.
#[derive(Debug, Clone)]
pub struct PolarEncoder {
    config: PolarConfig,
}

impl PolarEncoder {
    pub fn new(config: PolarConfig) -> Self {
        Self { config }
    }

    /// Encode K information bits into N codeword bits.
    pub fn encode(&self, info_bits: &[bool]) -> Vec<bool> {
        assert_eq!(info_bits.len(), self.config.num_info_bits);
        let n = self.config.block_size;

        // Place info bits and frozen bits (zeros)
        let mut u = vec![false; n];
        for (idx, &pos) in self.config.info_positions.iter().enumerate() {
            u[pos] = info_bits[idx];
        }

        // Polar transform: x = u * F^(⊗n) (Kronecker product of F = [[1,0],[1,1]])
        Self::polar_transform(&mut u);
        u
    }

    /// Apply the polar transform in-place (butterfly structure).
    fn polar_transform(bits: &mut [bool]) {
        let n = bits.len();
        let mut half = 1;
        while half < n {
            for i in (0..n).step_by(2 * half) {
                for j in 0..half {
                    bits[i + j] ^= bits[i + j + half];
                }
            }
            half <<= 1;
        }
    }

    pub fn config(&self) -> &PolarConfig {
        &self.config
    }
}

/// Polar decoder type.
#[derive(Debug, Clone)]
pub enum PolarDecoderType {
    /// Successive Cancellation.
    SC,
    /// SC List with given list size.
    SCL { list_size: usize },
}

/// Polar decoder.
#[derive(Debug, Clone)]
pub struct PolarDecoder {
    config: PolarConfig,
    decoder_type: PolarDecoderType,
}

impl PolarDecoder {
    pub fn new(config: PolarConfig, decoder_type: PolarDecoderType) -> Self {
        Self { config, decoder_type }
    }

    /// Decode from hard bits using SC algorithm.
    pub fn decode_hard(&self, received: &[bool]) -> Vec<bool> {
        // Convert to LLRs: 0 → +1.0, 1 → -1.0
        let llrs: Vec<f64> = received.iter().map(|&b| if b { -1.0 } else { 1.0 }).collect();
        self.decode_soft(&llrs)
    }

    /// Decode from soft LLR values using SC (or SCL).
    pub fn decode_soft(&self, llrs: &[f64]) -> Vec<bool> {
        assert_eq!(llrs.len(), self.config.block_size);

        match &self.decoder_type {
            PolarDecoderType::SC => self.decode_sc(llrs),
            PolarDecoderType::SCL { list_size } => self.decode_scl(llrs, *list_size),
        }
    }

    /// Successive Cancellation decoding.
    fn decode_sc(&self, llrs: &[f64]) -> Vec<bool> {
        let n = self.config.block_size;
        let log_n = (n as f64).log2() as usize;

        // LLR storage: log_n+1 layers, each with n values
        let mut l: Vec<Vec<f64>> = (0..=log_n).map(|_| vec![0.0; n]).collect();
        // Bit storage
        let mut bits: Vec<Vec<bool>> = (0..=log_n).map(|_| vec![false; n]).collect();

        // Initialize channel LLRs
        l[0] = llrs.to_vec();

        let mut info_bits = Vec::with_capacity(self.config.num_info_bits);

        for i in 0..n {
            // Navigate through the tree to get LLR for bit i
            self.sc_navigate(&mut l, &bits, i, n, log_n);

            let llr_i = l[log_n][i];

            if self.config.frozen_positions.contains(&i) {
                bits[log_n][i] = false; // frozen bit = 0
            } else {
                let decision = llr_i < 0.0;
                bits[log_n][i] = decision;
                info_bits.push(decision);
            }

            // Update partial sums
            self.sc_update_bits(&mut bits, i, n, log_n);
        }

        info_bits
    }

    fn sc_navigate(&self, l: &mut Vec<Vec<f64>>, bits: &[Vec<bool>], bit_idx: usize, n: usize, log_n: usize) {
        let mut size = n;
        let mut stage = 0;

        // Process from top (channel) to bottom (bit) of the tree
        for s in 0..log_n {
            size /= 2;
            let block = bit_idx / (2 * size);
            let pos_in_block = bit_idx % (2 * size);

            if pos_in_block < size {
                // Left child: f function (min-sum approximation)
                for j in 0..size {
                    let idx = block * 2 * size + j;
                    let la = l[s][idx];
                    let lb = l[s][idx + size];
                    // f(a, b) = sign(a)*sign(b)*min(|a|,|b|)
                    l[s + 1][idx] = Self::f_func(la, lb);
                }
            } else {
                // Right child: g function
                for j in 0..size {
                    let idx = block * 2 * size + j;
                    let la = l[s][idx];
                    let lb = l[s][idx + size];
                    let u_s = bits[s + 1][idx];
                    l[s + 1][idx + size] = Self::g_func(la, lb, u_s);
                }
            }
            stage = s + 1;
        }
        let _ = stage;
    }

    fn sc_update_bits(&self, bits: &mut Vec<Vec<bool>>, bit_idx: usize, n: usize, log_n: usize) {
        let mut size = 1;
        for s in (0..log_n).rev() {
            let block = bit_idx / (2 * size);
            let pos_in_block = bit_idx % (2 * size);

            if pos_in_block == 2 * size - 1 {
                // Both children available, update parent
                for j in 0..size {
                    let idx = block * 2 * size + j;
                    bits[s][idx] = bits[s + 1][idx] ^ bits[s + 1][idx + size];
                    bits[s][idx + size] = bits[s + 1][idx + size];
                }
            }
            size *= 2;
        }
    }

    /// Min-sum f function: f(a,b) = sign(a)*sign(b)*min(|a|,|b|)
    fn f_func(a: f64, b: f64) -> f64 {
        a.signum() * b.signum() * a.abs().min(b.abs())
    }

    /// g function: g(a,b,u) = b + (1-2u)*a
    fn g_func(a: f64, b: f64, u: bool) -> f64 {
        if u { b - a } else { b + a }
    }

    /// SC List decoding (simplified — falls back to SC for list_size=1).
    fn decode_scl(&self, llrs: &[f64], list_size: usize) -> Vec<bool> {
        if list_size <= 1 {
            return self.decode_sc(llrs);
        }
        // For now, use SC as baseline (full SCL is complex)
        // A proper SCL would maintain `list_size` candidate paths
        self.decode_sc(llrs)
    }

    pub fn config(&self) -> &PolarConfig {
        &self.config
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_polar_config_creation() {
        let config = PolarConfig::new(8, 4, 0.0);
        assert_eq!(config.block_size, 8);
        assert_eq!(config.num_info_bits, 4);
        assert_eq!(config.frozen_positions.len(), 4);
        assert_eq!(config.info_positions.len(), 4);
    }

    #[test]
    fn test_polar_encode_length() {
        let config = PolarConfig::new(16, 8, 0.0);
        let encoder = PolarEncoder::new(config);
        let info = vec![true, false, true, true, false, true, false, true];
        let codeword = encoder.encode(&info);
        assert_eq!(codeword.len(), 16);
    }

    #[test]
    fn test_polar_roundtrip_n8_k4() {
        let config = PolarConfig::new(8, 4, 0.0);
        let encoder = PolarEncoder::new(config.clone());
        let decoder = PolarDecoder::new(config, PolarDecoderType::SC);

        let info = vec![true, false, true, false];
        let codeword = encoder.encode(&info);
        let decoded = decoder.decode_hard(&codeword);
        assert_eq!(decoded, info, "Roundtrip should recover info bits");
    }

    #[test]
    fn test_polar_roundtrip_n16_k8() {
        let config = PolarConfig::new(16, 8, 0.0);
        let encoder = PolarEncoder::new(config.clone());
        let decoder = PolarDecoder::new(config, PolarDecoderType::SC);

        let info = vec![true, false, true, true, false, true, false, true];
        let codeword = encoder.encode(&info);
        let decoded = decoder.decode_hard(&codeword);
        assert_eq!(decoded, info);
    }

    #[test]
    fn test_polar_all_zeros() {
        let config = PolarConfig::new(8, 4, 0.0);
        let encoder = PolarEncoder::new(config.clone());
        let decoder = PolarDecoder::new(config, PolarDecoderType::SC);

        let info = vec![false; 4];
        let codeword = encoder.encode(&info);
        let decoded = decoder.decode_hard(&codeword);
        assert_eq!(decoded, info);
    }

    #[test]
    fn test_polar_all_ones() {
        let config = PolarConfig::new(8, 4, 0.0);
        let encoder = PolarEncoder::new(config.clone());
        let decoder = PolarDecoder::new(config, PolarDecoderType::SC);

        let info = vec![true; 4];
        let codeword = encoder.encode(&info);
        let decoded = decoder.decode_hard(&codeword);
        assert_eq!(decoded, info);
    }

    #[test]
    fn test_polar_soft_decode() {
        let config = PolarConfig::new(8, 4, 0.0);
        let encoder = PolarEncoder::new(config.clone());
        let decoder = PolarDecoder::new(config, PolarDecoderType::SC);

        let info = vec![true, false, true, false];
        let codeword = encoder.encode(&info);

        // Convert to soft LLRs (perfect channel)
        let llrs: Vec<f64> = codeword.iter().map(|&b| if b { -5.0 } else { 5.0 }).collect();
        let decoded = decoder.decode_soft(&llrs);
        assert_eq!(decoded, info);
    }

    #[test]
    fn test_polar_rate() {
        let config = PolarConfig::new(16, 8, 0.0);
        assert!((config.rate() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn test_polar_frozen_vs_info_partition() {
        let config = PolarConfig::new(32, 16, 1.0);
        // Frozen + info should cover all positions
        let mut all: Vec<usize> = config.frozen_positions.iter().chain(config.info_positions.iter()).copied().collect();
        all.sort_unstable();
        all.dedup();
        assert_eq!(all.len(), 32);
    }

    #[test]
    fn test_polar_n32_k16() {
        let config = PolarConfig::new(32, 16, 1.0);
        let encoder = PolarEncoder::new(config.clone());
        let decoder = PolarDecoder::new(config, PolarDecoderType::SC);

        let info: Vec<bool> = (0..16).map(|i| i % 3 == 0).collect();
        let codeword = encoder.encode(&info);
        let decoded = decoder.decode_hard(&codeword);
        assert_eq!(decoded, info);
    }

    #[test]
    fn test_polar_transform_identity() {
        // All zeros should stay all zeros
        let mut bits = vec![false; 8];
        PolarEncoder::polar_transform(&mut bits);
        assert!(bits.iter().all(|&b| !b));
    }

    #[test]
    fn test_scl_fallback() {
        let config = PolarConfig::new(8, 4, 0.0);
        let encoder = PolarEncoder::new(config.clone());
        let decoder = PolarDecoder::new(config, PolarDecoderType::SCL { list_size: 4 });

        let info = vec![true, false, true, false];
        let codeword = encoder.encode(&info);
        let decoded = decoder.decode_hard(&codeword);
        assert_eq!(decoded, info);
    }
}
