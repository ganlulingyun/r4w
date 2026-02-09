//! Random PDU Generator — Test Traffic Generation
//!
//! Generates Protocol Data Units (PDUs) with random payloads for
//! testing packet-based systems, stress-testing protocols, and
//! simulating bursty traffic. Supports configurable PDU sizes,
//! inter-arrival time distributions, and metadata tagging.
//! GNU Radio equivalent: `gr::blocks::random_pdu_generator`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::random_pdu_gen::{RandomPduGen, PduConfig};
//!
//! let config = PduConfig {
//!     min_size: 10,
//!     max_size: 100,
//!     seed: 42,
//! };
//! let mut gen = RandomPduGen::new(config);
//! let pdu = gen.generate();
//! assert!(pdu.payload.len() >= 10 && pdu.payload.len() <= 100);
//! ```

/// PDU generator configuration.
#[derive(Debug, Clone)]
pub struct PduConfig {
    /// Minimum PDU payload size in bytes.
    pub min_size: usize,
    /// Maximum PDU payload size in bytes.
    pub max_size: usize,
    /// Random seed for reproducibility.
    pub seed: u64,
}

/// Inter-arrival time distribution.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ArrivalDistribution {
    /// Uniform random in [min, max] ms.
    Uniform { min_ms: f64, max_ms: f64 },
    /// Poisson arrivals with given mean rate (PDUs/sec).
    Poisson { rate: f64 },
    /// Fixed interval in ms.
    Fixed { interval_ms: f64 },
}

/// Generated PDU with metadata.
#[derive(Debug, Clone)]
pub struct GeneratedPdu {
    /// Payload bytes.
    pub payload: Vec<u8>,
    /// Sequence number.
    pub seq_num: u64,
    /// Timestamp offset from start (seconds).
    pub timestamp_s: f64,
    /// Metadata key-value pairs.
    pub metadata: Vec<(String, String)>,
}

/// Traffic statistics.
#[derive(Debug, Clone, Default)]
pub struct TrafficStats {
    /// Total PDUs generated.
    pub total_pdus: u64,
    /// Total bytes generated.
    pub total_bytes: u64,
    /// Average PDU size.
    pub avg_size: f64,
    /// Min PDU size observed.
    pub min_size: usize,
    /// Max PDU size observed.
    pub max_size: usize,
    /// Average inter-arrival time (ms).
    pub avg_iat_ms: f64,
}

/// Random PDU generator.
#[derive(Debug, Clone)]
pub struct RandomPduGen {
    config: PduConfig,
    distribution: ArrivalDistribution,
    rng_state: u64,
    seq_num: u64,
    current_time_s: f64,
    stats: TrafficStats,
    /// Optional metadata template to attach to every PDU.
    metadata_template: Vec<(String, String)>,
}

impl RandomPduGen {
    /// Create a new PDU generator.
    pub fn new(config: PduConfig) -> Self {
        let rng_state = config.seed;
        Self {
            config,
            distribution: ArrivalDistribution::Fixed { interval_ms: 100.0 },
            rng_state,
            seq_num: 0,
            current_time_s: 0.0,
            stats: TrafficStats::default(),
            metadata_template: Vec::new(),
        }
    }

    /// Set inter-arrival time distribution.
    pub fn set_distribution(&mut self, dist: ArrivalDistribution) {
        self.distribution = dist;
    }

    /// Add metadata template (attached to every generated PDU).
    pub fn add_metadata(&mut self, key: &str, value: &str) {
        self.metadata_template
            .push((key.to_string(), value.to_string()));
    }

    /// Generate a single PDU.
    pub fn generate(&mut self) -> GeneratedPdu {
        // Determine payload size
        let size = if self.config.min_size == self.config.max_size {
            self.config.min_size
        } else {
            let range = self.config.max_size - self.config.min_size + 1;
            self.config.min_size + (self.next_rand() as usize % range)
        };

        // Generate random payload
        let payload: Vec<u8> = (0..size).map(|_| (self.next_rand() & 0xFF) as u8).collect();

        // Advance time
        let iat_s = self.next_interval_s();
        self.current_time_s += iat_s;

        let seq = self.seq_num;
        self.seq_num += 1;

        // Update stats
        self.stats.total_pdus += 1;
        self.stats.total_bytes += size as u64;
        self.stats.avg_size =
            self.stats.total_bytes as f64 / self.stats.total_pdus as f64;
        if self.stats.total_pdus == 1 || size < self.stats.min_size {
            self.stats.min_size = size;
        }
        if size > self.stats.max_size {
            self.stats.max_size = size;
        }
        if self.stats.total_pdus > 1 {
            self.stats.avg_iat_ms = self.current_time_s * 1000.0
                / (self.stats.total_pdus - 1) as f64;
        }

        let mut metadata = self.metadata_template.clone();
        metadata.push(("seq".to_string(), seq.to_string()));

        GeneratedPdu {
            payload,
            seq_num: seq,
            timestamp_s: self.current_time_s,
            metadata,
        }
    }

    /// Generate a batch of PDUs.
    pub fn generate_batch(&mut self, count: usize) -> Vec<GeneratedPdu> {
        (0..count).map(|_| self.generate()).collect()
    }

    /// Generate PDUs for a given time duration.
    pub fn generate_for_duration(&mut self, duration_s: f64) -> Vec<GeneratedPdu> {
        let start = self.current_time_s;
        let mut pdus = Vec::new();

        loop {
            let pdu = self.generate();
            if pdu.timestamp_s - start > duration_s {
                break;
            }
            pdus.push(pdu);
            // Safety limit
            if pdus.len() > 100_000 {
                break;
            }
        }

        pdus
    }

    /// Get traffic statistics.
    pub fn stats(&self) -> &TrafficStats {
        &self.stats
    }

    /// Reset generator state.
    pub fn reset(&mut self) {
        self.rng_state = self.config.seed;
        self.seq_num = 0;
        self.current_time_s = 0.0;
        self.stats = TrafficStats::default();
    }

    /// Get config.
    pub fn config(&self) -> &PduConfig {
        &self.config
    }

    /// Simple LCG random number generator.
    fn next_rand(&mut self) -> u64 {
        self.rng_state = self.rng_state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        self.rng_state >> 32
    }

    /// Generate next inter-arrival time in seconds.
    fn next_interval_s(&mut self) -> f64 {
        match self.distribution {
            ArrivalDistribution::Fixed { interval_ms } => interval_ms / 1000.0,
            ArrivalDistribution::Uniform { min_ms, max_ms } => {
                let u = (self.next_rand() as f64) / (u32::MAX as f64);
                let ms = min_ms + u * (max_ms - min_ms);
                ms / 1000.0
            }
            ArrivalDistribution::Poisson { rate } => {
                // Exponential inter-arrival: -ln(U) / rate
                let u = (self.next_rand() as f64 + 1.0) / (u32::MAX as f64 + 1.0);
                -u.ln() / rate
            }
        }
    }
}

/// Generate a fixed-pattern PDU (for testing).
pub fn fixed_pattern_pdu(size: usize, pattern: u8) -> GeneratedPdu {
    GeneratedPdu {
        payload: vec![pattern; size],
        seq_num: 0,
        timestamp_s: 0.0,
        metadata: Vec::new(),
    }
}

/// Generate a counting pattern PDU (0, 1, 2, ..., 255, 0, 1, ...).
pub fn counting_pattern_pdu(size: usize) -> GeneratedPdu {
    GeneratedPdu {
        payload: (0..size).map(|i| (i % 256) as u8).collect(),
        seq_num: 0,
        timestamp_s: 0.0,
        metadata: Vec::new(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_generate_pdu() {
        let config = PduConfig {
            min_size: 10,
            max_size: 100,
            seed: 42,
        };
        let mut gen = RandomPduGen::new(config);
        let pdu = gen.generate();
        assert!(pdu.payload.len() >= 10 && pdu.payload.len() <= 100);
        assert_eq!(pdu.seq_num, 0);
    }

    #[test]
    fn test_fixed_size() {
        let config = PduConfig {
            min_size: 50,
            max_size: 50,
            seed: 1,
        };
        let mut gen = RandomPduGen::new(config);
        for _ in 0..10 {
            let pdu = gen.generate();
            assert_eq!(pdu.payload.len(), 50);
        }
    }

    #[test]
    fn test_sequence_numbers() {
        let config = PduConfig {
            min_size: 10,
            max_size: 10,
            seed: 42,
        };
        let mut gen = RandomPduGen::new(config);
        for i in 0..5 {
            let pdu = gen.generate();
            assert_eq!(pdu.seq_num, i);
        }
    }

    #[test]
    fn test_reproducibility() {
        let config = PduConfig {
            min_size: 10,
            max_size: 100,
            seed: 12345,
        };
        let mut gen1 = RandomPduGen::new(config.clone());
        let mut gen2 = RandomPduGen::new(config);
        for _ in 0..10 {
            let p1 = gen1.generate();
            let p2 = gen2.generate();
            assert_eq!(p1.payload, p2.payload);
        }
    }

    #[test]
    fn test_batch_generation() {
        let config = PduConfig {
            min_size: 5,
            max_size: 20,
            seed: 42,
        };
        let mut gen = RandomPduGen::new(config);
        let batch = gen.generate_batch(50);
        assert_eq!(batch.len(), 50);
        assert_eq!(gen.stats().total_pdus, 50);
    }

    #[test]
    fn test_poisson_distribution() {
        let config = PduConfig {
            min_size: 10,
            max_size: 10,
            seed: 42,
        };
        let mut gen = RandomPduGen::new(config);
        gen.set_distribution(ArrivalDistribution::Poisson { rate: 100.0 });
        let pdus = gen.generate_batch(100);
        // Inter-arrival times should vary
        let iats: Vec<f64> = pdus
            .windows(2)
            .map(|w| w[1].timestamp_s - w[0].timestamp_s)
            .collect();
        let min_iat = iats.iter().cloned().fold(f64::INFINITY, f64::min);
        let max_iat = iats.iter().cloned().fold(0.0f64, f64::max);
        assert!(max_iat > min_iat, "Poisson IATs should vary");
    }

    #[test]
    fn test_uniform_distribution() {
        let config = PduConfig {
            min_size: 10,
            max_size: 10,
            seed: 42,
        };
        let mut gen = RandomPduGen::new(config);
        gen.set_distribution(ArrivalDistribution::Uniform {
            min_ms: 10.0,
            max_ms: 100.0,
        });
        let pdus = gen.generate_batch(20);
        for w in pdus.windows(2) {
            let iat = w[1].timestamp_s - w[0].timestamp_s;
            assert!(iat >= 0.01 && iat <= 0.1, "IAT out of range: {}", iat);
        }
    }

    #[test]
    fn test_metadata() {
        let config = PduConfig {
            min_size: 10,
            max_size: 10,
            seed: 42,
        };
        let mut gen = RandomPduGen::new(config);
        gen.add_metadata("protocol", "test");
        let pdu = gen.generate();
        assert!(pdu.metadata.iter().any(|(k, v)| k == "protocol" && v == "test"));
        assert!(pdu.metadata.iter().any(|(k, _)| k == "seq"));
    }

    #[test]
    fn test_statistics() {
        let config = PduConfig {
            min_size: 10,
            max_size: 100,
            seed: 42,
        };
        let mut gen = RandomPduGen::new(config);
        gen.generate_batch(100);
        let stats = gen.stats();
        assert_eq!(stats.total_pdus, 100);
        assert!(stats.avg_size >= 10.0 && stats.avg_size <= 100.0);
        assert!(stats.min_size >= 10);
        assert!(stats.max_size <= 100);
    }

    #[test]
    fn test_reset() {
        let config = PduConfig {
            min_size: 10,
            max_size: 10,
            seed: 42,
        };
        let mut gen = RandomPduGen::new(config);
        let p1 = gen.generate();
        gen.reset();
        let p2 = gen.generate();
        assert_eq!(p1.payload, p2.payload);
        assert_eq!(p2.seq_num, 0);
    }

    #[test]
    fn test_fixed_pattern() {
        let pdu = fixed_pattern_pdu(16, 0xAA);
        assert_eq!(pdu.payload.len(), 16);
        assert!(pdu.payload.iter().all(|&b| b == 0xAA));
    }

    #[test]
    fn test_counting_pattern() {
        let pdu = counting_pattern_pdu(260);
        assert_eq!(pdu.payload.len(), 260);
        assert_eq!(pdu.payload[0], 0);
        assert_eq!(pdu.payload[255], 255);
        assert_eq!(pdu.payload[256], 0);
    }
}
