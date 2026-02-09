//! Random Source — Configurable random data generator
//!
//! Generates random samples with selectable distribution: uniform,
//! Gaussian, binary (0/1), or bipolar (+1/-1). Used for noise
//! generation, Monte Carlo simulation, test pattern creation,
//! and dithering.
//! GNU Radio equivalent: `random_source_b/s/i/f/c`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::random_source::RandomSource;
//!
//! let mut src = RandomSource::uniform(42);
//! let samples = src.generate_f64(100);
//! assert_eq!(samples.len(), 100);
//! // All in [0, 1)
//! assert!(samples.iter().all(|&x| x >= 0.0 && x < 1.0));
//! ```

/// Pseudo-random number generator (xoshiro256**).
#[derive(Debug, Clone)]
struct Rng {
    s: [u64; 4],
}

impl Rng {
    fn new(seed: u64) -> Self {
        // SplitMix64 to expand seed into state
        let mut state = seed;
        let mut s = [0u64; 4];
        for slot in &mut s {
            state = state.wrapping_add(0x9e3779b97f4a7c15);
            let mut z = state;
            z = (z ^ (z >> 30)).wrapping_mul(0xbf58476d1ce4e5b9);
            z = (z ^ (z >> 27)).wrapping_mul(0x94d049bb133111eb);
            *slot = z ^ (z >> 31);
        }
        Self { s }
    }

    #[inline]
    fn next_u64(&mut self) -> u64 {
        let result = (self.s[1].wrapping_mul(5)).rotate_left(7).wrapping_mul(9);
        let t = self.s[1] << 17;
        self.s[2] ^= self.s[0];
        self.s[3] ^= self.s[1];
        self.s[1] ^= self.s[2];
        self.s[0] ^= self.s[3];
        self.s[2] ^= t;
        self.s[3] = self.s[3].rotate_left(45);
        result
    }

    /// Uniform f64 in [0, 1).
    #[inline]
    fn next_f64(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 * (1.0 / (1u64 << 53) as f64)
    }

    /// Gaussian via Box-Muller.
    fn next_gaussian(&mut self) -> f64 {
        loop {
            let u1 = self.next_f64();
            let u2 = self.next_f64();
            if u1 > 1e-30 {
                return (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos();
            }
        }
    }
}

/// Distribution type for the random source.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Distribution {
    /// Uniform [0, 1).
    Uniform,
    /// Gaussian (mean=0, std=1).
    Gaussian,
    /// Uniform over [lo, hi).
    UniformRange { lo: f64, hi: f64 },
}

/// Random data source block.
#[derive(Debug, Clone)]
pub struct RandomSource {
    rng: Rng,
    distribution: Distribution,
}

impl RandomSource {
    /// Create with uniform [0, 1) distribution.
    pub fn uniform(seed: u64) -> Self {
        Self {
            rng: Rng::new(seed),
            distribution: Distribution::Uniform,
        }
    }

    /// Create with Gaussian (mean=0, std=1) distribution.
    pub fn gaussian(seed: u64) -> Self {
        Self {
            rng: Rng::new(seed),
            distribution: Distribution::Gaussian,
        }
    }

    /// Create with uniform range [lo, hi).
    pub fn uniform_range(seed: u64, lo: f64, hi: f64) -> Self {
        Self {
            rng: Rng::new(seed),
            distribution: Distribution::UniformRange { lo, hi },
        }
    }

    /// Generate one f64 sample.
    #[inline]
    pub fn next_f64(&mut self) -> f64 {
        match self.distribution {
            Distribution::Uniform => self.rng.next_f64(),
            Distribution::Gaussian => self.rng.next_gaussian(),
            Distribution::UniformRange { lo, hi } => lo + (hi - lo) * self.rng.next_f64(),
        }
    }

    /// Generate N f64 samples.
    pub fn generate_f64(&mut self, n: usize) -> Vec<f64> {
        (0..n).map(|_| self.next_f64()).collect()
    }

    /// Generate N random bytes.
    pub fn generate_bytes(&mut self, n: usize) -> Vec<u8> {
        let mut output = Vec::with_capacity(n);
        let mut remaining = n;
        while remaining >= 8 {
            let val = self.rng.next_u64();
            output.extend_from_slice(&val.to_le_bytes());
            remaining -= 8;
        }
        if remaining > 0 {
            let val = self.rng.next_u64();
            output.extend_from_slice(&val.to_le_bytes()[..remaining]);
        }
        output
    }

    /// Generate N random bits (0 or 1).
    pub fn generate_bits(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| (self.rng.next_u64() & 1) as u8).collect()
    }

    /// Generate N random booleans.
    pub fn generate_bool(&mut self, n: usize) -> Vec<bool> {
        (0..n).map(|_| self.rng.next_u64() & 1 != 0).collect()
    }

    /// Generate N bipolar samples (+1.0 or -1.0).
    pub fn generate_bipolar(&mut self, n: usize) -> Vec<f64> {
        (0..n)
            .map(|_| {
                if self.rng.next_u64() & 1 != 0 {
                    1.0
                } else {
                    -1.0
                }
            })
            .collect()
    }

    /// Get distribution type.
    pub fn distribution(&self) -> Distribution {
        self.distribution
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_uniform_range() {
        let mut src = RandomSource::uniform(12345);
        let samples = src.generate_f64(1000);
        for &s in &samples {
            assert!(s >= 0.0 && s < 1.0, "uniform sample out of range: {s}");
        }
    }

    #[test]
    fn test_gaussian_statistics() {
        let mut src = RandomSource::gaussian(42);
        let samples = src.generate_f64(10000);
        let mean: f64 = samples.iter().sum::<f64>() / samples.len() as f64;
        let variance: f64 =
            samples.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / samples.len() as f64;
        assert!(mean.abs() < 0.1, "Gaussian mean should be ~0, got {mean}");
        assert!(
            (variance - 1.0).abs() < 0.2,
            "Gaussian variance should be ~1, got {variance}"
        );
    }

    #[test]
    fn test_uniform_range_custom() {
        let mut src = RandomSource::uniform_range(99, -5.0, 5.0);
        let samples = src.generate_f64(1000);
        for &s in &samples {
            assert!(s >= -5.0 && s < 5.0, "range sample out of bounds: {s}");
        }
    }

    #[test]
    fn test_generate_bytes() {
        let mut src = RandomSource::uniform(1);
        let bytes = src.generate_bytes(100);
        assert_eq!(bytes.len(), 100);
        // Should have some variation
        let unique: std::collections::HashSet<u8> = bytes.iter().cloned().collect();
        assert!(unique.len() > 10, "bytes should have variety");
    }

    #[test]
    fn test_generate_bits() {
        let mut src = RandomSource::uniform(42);
        let bits = src.generate_bits(1000);
        assert_eq!(bits.len(), 1000);
        for &b in &bits {
            assert!(b <= 1);
        }
        let ones: usize = bits.iter().map(|&b| b as usize).sum();
        assert!(ones > 400 && ones < 600, "bits should be ~balanced, got {ones} ones");
    }

    #[test]
    fn test_generate_bool() {
        let mut src = RandomSource::uniform(7);
        let bools = src.generate_bool(100);
        assert_eq!(bools.len(), 100);
    }

    #[test]
    fn test_generate_bipolar() {
        let mut src = RandomSource::uniform(13);
        let samples = src.generate_bipolar(100);
        for &s in &samples {
            assert!(s == 1.0 || s == -1.0);
        }
    }

    #[test]
    fn test_deterministic() {
        let mut src1 = RandomSource::uniform(42);
        let mut src2 = RandomSource::uniform(42);
        assert_eq!(src1.generate_f64(50), src2.generate_f64(50));
    }

    #[test]
    fn test_different_seeds() {
        let mut src1 = RandomSource::uniform(1);
        let mut src2 = RandomSource::uniform(2);
        assert_ne!(src1.generate_f64(10), src2.generate_f64(10));
    }

    #[test]
    fn test_distribution_accessor() {
        let src = RandomSource::gaussian(0);
        assert_eq!(src.distribution(), Distribution::Gaussian);
    }

    #[test]
    fn test_empty_generate() {
        let mut src = RandomSource::uniform(0);
        assert!(src.generate_f64(0).is_empty());
        assert!(src.generate_bits(0).is_empty());
        assert!(src.generate_bytes(0).is_empty());
    }

    #[test]
    fn test_bytes_odd_length() {
        let mut src = RandomSource::uniform(5);
        assert_eq!(src.generate_bytes(3).len(), 3);
        assert_eq!(src.generate_bytes(7).len(), 7);
    }
}
