//! High-throughput parallel CRC computation with slicing-by-4 acceleration.
//!
//! This module provides a generic CRC engine supporting CRC-8, CRC-16-CCITT, CRC-32,
//! CRC-32C (Castagnoli), and custom CRC configurations. It uses precomputed lookup
//! tables for fast byte-at-a-time processing, and a slicing-by-4 technique for
//! additional throughput on larger payloads.
//!
//! # Example
//!
//! ```
//! use r4w_core::cyclic_redundancy_check_parallel::{ParallelCrc, CrcAlgorithm};
//!
//! // Compute CRC-32 of a known test vector
//! let crc = ParallelCrc::new(CrcAlgorithm::Crc32);
//! let result = crc.compute(b"123456789");
//! assert_eq!(result, 0xCBF43926);
//!
//! // Verify a CRC
//! assert!(crc.verify(b"123456789", 0xCBF43926));
//!
//! // Streaming / incremental computation
//! let mut crc_stream = ParallelCrc::new(CrcAlgorithm::Crc32);
//! crc_stream.update(b"1234");
//! crc_stream.update(b"56789");
//! assert_eq!(crc_stream.finalize(), 0xCBF43926);
//! ```

/// Supported CRC algorithm presets.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum CrcAlgorithm {
    /// CRC-8 (polynomial 0x07, used in ATM HEC, etc.)
    Crc8,
    /// CRC-16-CCITT (polynomial 0x1021, init 0xFFFF, used in X.25/HDLC)
    Crc16Ccitt,
    /// CRC-32 (ISO 3309 / ITU-T V.42 / Ethernet / PKZIP)
    Crc32,
    /// CRC-32C Castagnoli (iSCSI, SCTP, SSE4.2 instruction)
    Crc32c,
    /// User-supplied custom configuration
    Custom,
}

/// Configuration for a CRC algorithm.
#[derive(Debug, Clone)]
pub struct CrcConfig {
    /// CRC polynomial (without the implicit leading 1 bit).
    pub polynomial: u64,
    /// Initial register value before processing any data.
    pub initial_value: u64,
    /// Value XORed with the final register to produce the CRC.
    pub final_xor: u64,
    /// If true, each input byte is bit-reflected (LSB-first) before processing.
    pub reflect_input: bool,
    /// If true, the final CRC value is bit-reflected before XOR.
    pub reflect_output: bool,
    /// CRC width in bits (8, 16, 32, etc.).
    pub width: u8,
}

impl CrcConfig {
    /// Return the configuration for a named algorithm preset.
    pub fn for_algorithm(algo: &CrcAlgorithm) -> Self {
        match algo {
            CrcAlgorithm::Crc8 => CrcConfig {
                polynomial: 0x07,
                initial_value: 0x00,
                final_xor: 0x00,
                reflect_input: false,
                reflect_output: false,
                width: 8,
            },
            CrcAlgorithm::Crc16Ccitt => CrcConfig {
                polynomial: 0x1021,
                initial_value: 0xFFFF,
                final_xor: 0x0000,
                reflect_input: false,
                reflect_output: false,
                width: 16,
            },
            CrcAlgorithm::Crc32 => CrcConfig {
                polynomial: 0x04C11DB7,
                initial_value: 0xFFFFFFFF,
                final_xor: 0xFFFFFFFF,
                reflect_input: true,
                reflect_output: true,
                width: 32,
            },
            CrcAlgorithm::Crc32c => CrcConfig {
                polynomial: 0x1EDC6F41,
                initial_value: 0xFFFFFFFF,
                final_xor: 0xFFFFFFFF,
                reflect_input: true,
                reflect_output: true,
                width: 32,
            },
            CrcAlgorithm::Custom => CrcConfig {
                polynomial: 0,
                initial_value: 0,
                final_xor: 0,
                reflect_input: false,
                reflect_output: false,
                width: 8,
            },
        }
    }
}

/// Reflect (reverse) the bottom `width` bits of `value`.
fn reflect_bits(value: u64, width: u8) -> u64 {
    let mut reflected: u64 = 0;
    for i in 0..width as u64 {
        if value & (1 << i) != 0 {
            reflected |= 1 << ((width as u64 - 1) - i);
        }
    }
    reflected
}

/// Generic CRC computation engine with precomputed tables.
///
/// Supports standard byte-at-a-time computation and an accelerated slicing-by-4
/// variant that processes 4 bytes per iteration for higher throughput.
pub struct ParallelCrc {
    config: CrcConfig,
    algorithm: CrcAlgorithm,
    /// Standard 256-entry lookup table (table index 0).
    /// For slicing-by-4, tables[0..4] are each 256 entries.
    tables: Vec<Vec<u64>>,
    /// Running CRC register for incremental (streaming) computation.
    register: u64,
}

impl ParallelCrc {
    // ------------------------------------------------------------------ constructors

    /// Create a new CRC engine for a named algorithm preset.
    pub fn new(algorithm: CrcAlgorithm) -> Self {
        let config = CrcConfig::for_algorithm(&algorithm);
        let tables = Self::build_tables(&config);
        let register = config.initial_value;
        Self {
            config,
            algorithm,
            tables,
            register,
        }
    }

    /// Create a CRC engine from a custom configuration.
    pub fn custom(config: CrcConfig) -> Self {
        let tables = Self::build_tables(&config);
        let register = config.initial_value;
        Self {
            config,
            algorithm: CrcAlgorithm::Custom,
            tables,
            register,
        }
    }

    // ------------------------------------------------------------------ table generation

    /// Build 4 lookup tables (each 256 entries) for slicing-by-4.
    /// `tables[0]` is the standard byte-at-a-time table.
    fn build_tables(config: &CrcConfig) -> Vec<Vec<u64>> {
        let width = config.width;
        let poly = config.polynomial;

        // Table 0 - standard CRC table
        let mut t0 = vec![0u64; 256];
        for i in 0u64..256 {
            let mut crc = if config.reflect_input {
                reflect_bits(i, 8)
            } else {
                i
            };

            // Shift the byte to the MSB position of the CRC register
            crc <<= width.saturating_sub(8);

            for _bit in 0..8 {
                if crc & (1u64 << (width - 1)) != 0 {
                    crc = (crc << 1) ^ poly;
                } else {
                    crc <<= 1;
                }
            }

            // Mask to register width
            let mask = if width >= 64 {
                u64::MAX
            } else {
                (1u64 << width) - 1
            };
            crc &= mask;

            if config.reflect_input {
                crc = reflect_bits(crc, width);
            }

            t0[i as usize] = crc;
        }

        // Tables 1-3 for slicing-by-4
        let mask = if width >= 64 {
            u64::MAX
        } else {
            (1u64 << width) - 1
        };

        let mut tables = vec![t0.clone()];

        for _slice in 1..4 {
            let prev = tables.last().unwrap().clone();
            let mut tn = vec![0u64; 256];
            for i in 0..256 {
                let entry = prev[i];
                if config.reflect_input {
                    // For reflected algorithms, the register shifts right
                    let index = (entry & 0xFF) as usize;
                    tn[i] = (entry >> 8) ^ t0[index];
                } else {
                    // For non-reflected algorithms, the register shifts left
                    let index_nr = ((entry >> (width - 8)) & 0xFF) as usize;
                    tn[i] = ((entry << 8) & mask) ^ t0[index_nr];
                }
            }
            tables.push(tn);
        }

        tables
    }

    // ------------------------------------------------------------------ one-shot computation

    /// Compute the CRC of `data` using the standard byte-at-a-time table.
    pub fn compute(&self, data: &[u8]) -> u64 {
        let mask = self.register_mask();
        let mut crc = self.config.initial_value & mask;

        if self.config.reflect_input {
            // Reflected algorithm: register shifts right
            for &byte in data {
                let index = ((crc ^ (byte as u64)) & 0xFF) as usize;
                crc = (crc >> 8) ^ self.tables[0][index];
            }
        } else {
            // Normal algorithm: register shifts left
            let shift = self.config.width.saturating_sub(8);
            for &byte in data {
                let index = (((crc >> shift) ^ (byte as u64)) & 0xFF) as usize;
                crc = ((crc << 8) & mask) ^ self.tables[0][index];
            }
        }

        if self.config.reflect_output != self.config.reflect_input {
            crc = reflect_bits(crc, self.config.width);
        }

        (crc ^ self.config.final_xor) & mask
    }

    /// Compute the CRC of `data` using slicing-by-4 for higher throughput.
    ///
    /// Falls back to standard computation for payloads shorter than 4 bytes.
    pub fn compute_sliced(&self, data: &[u8]) -> u64 {
        if data.len() < 4 {
            return self.compute(data);
        }

        let mask = self.register_mask();
        let mut crc = self.config.initial_value & mask;

        let chunks = data.len() / 4;
        let remainder = data.len() % 4;

        if self.config.reflect_input {
            // Reflected (LSB-first) slicing-by-4
            for chunk_idx in 0..chunks {
                let base = chunk_idx * 4;
                let b0 = data[base] as u64;
                let b1 = data[base + 1] as u64;
                let b2 = data[base + 2] as u64;
                let b3 = data[base + 3] as u64;

                if self.config.width <= 8 {
                    let i0 = ((crc ^ b0) & 0xFF) as usize;
                    crc = self.tables[0][i0];
                    let i1 = ((crc ^ b1) & 0xFF) as usize;
                    crc = self.tables[0][i1];
                    let i2 = ((crc ^ b2) & 0xFF) as usize;
                    crc = self.tables[0][i2];
                    let i3 = ((crc ^ b3) & 0xFF) as usize;
                    crc = self.tables[0][i3];
                } else {
                    let i0 = ((crc ^ b0) & 0xFF) as usize;
                    let i1 = (((crc >> 8) ^ b1) & 0xFF) as usize;
                    let i2 = (((crc >> 16) ^ b2) & 0xFF) as usize;
                    let i3 = (((crc >> 24) ^ b3) & 0xFF) as usize;
                    crc = self.tables[3][i0]
                        ^ self.tables[2][i1]
                        ^ self.tables[1][i2]
                        ^ self.tables[0][i3];
                }
            }

            // Process remaining bytes
            let tail = &data[chunks * 4..chunks * 4 + remainder];
            for &byte in tail {
                let index = ((crc ^ (byte as u64)) & 0xFF) as usize;
                crc = (crc >> 8) ^ self.tables[0][index];
            }
        } else {
            // Normal (MSB-first) - fall back to standard for correctness
            let shift = self.config.width.saturating_sub(8);
            for &byte in data {
                let index = (((crc >> shift) ^ (byte as u64)) & 0xFF) as usize;
                crc = ((crc << 8) & mask) ^ self.tables[0][index];
            }
        }

        if self.config.reflect_output != self.config.reflect_input {
            crc = reflect_bits(crc, self.config.width);
        }

        (crc ^ self.config.final_xor) & mask
    }

    // ------------------------------------------------------------------ verification

    /// Verify that `data` produces the `expected` CRC value.
    pub fn verify(&self, data: &[u8], expected: u64) -> bool {
        self.compute(data) == expected
    }

    // ------------------------------------------------------------------ streaming / incremental

    /// Feed additional data into the running CRC computation.
    pub fn update(&mut self, data: &[u8]) {
        let mask = self.register_mask();

        if self.config.reflect_input {
            for &byte in data {
                let index = ((self.register ^ (byte as u64)) & 0xFF) as usize;
                self.register = (self.register >> 8) ^ self.tables[0][index];
            }
        } else {
            let shift = self.config.width.saturating_sub(8);
            for &byte in data {
                let index = (((self.register >> shift) ^ (byte as u64)) & 0xFF) as usize;
                self.register = ((self.register << 8) & mask) ^ self.tables[0][index];
            }
        }
    }

    /// Finalize the streaming CRC and return the result.
    ///
    /// This does **not** reset the internal register; call [`reset`](Self::reset)
    /// before starting a new computation.
    pub fn finalize(&self) -> u64 {
        let mask = self.register_mask();
        let mut crc = self.register;

        if self.config.reflect_output != self.config.reflect_input {
            crc = reflect_bits(crc, self.config.width);
        }

        (crc ^ self.config.final_xor) & mask
    }

    /// Reset the internal register to the initial value for a new computation.
    pub fn reset(&mut self) {
        self.register = self.config.initial_value & self.register_mask();
    }

    // ------------------------------------------------------------------ accessors

    /// Return the CRC polynomial for this engine.
    pub fn polynomial(&self) -> u64 {
        self.config.polynomial
    }

    /// Return the CRC algorithm enum variant.
    pub fn algorithm(&self) -> &CrcAlgorithm {
        &self.algorithm
    }

    /// Return the CRC width in bits.
    pub fn width(&self) -> u8 {
        self.config.width
    }

    // ------------------------------------------------------------------ helpers

    /// Bit mask covering the register width.
    fn register_mask(&self) -> u64 {
        if self.config.width >= 64 {
            u64::MAX
        } else {
            (1u64 << self.config.width) - 1
        }
    }
}

// ======================================================================== tests

#[cfg(test)]
mod tests {
    use super::*;

    // ---- CRC-32 known-answer tests ----

    #[test]
    fn crc32_check_value() {
        let crc = ParallelCrc::new(CrcAlgorithm::Crc32);
        assert_eq!(crc.compute(b"123456789"), 0xCBF43926);
    }

    #[test]
    fn crc32_sliced_check_value() {
        let crc = ParallelCrc::new(CrcAlgorithm::Crc32);
        assert_eq!(crc.compute_sliced(b"123456789"), 0xCBF43926);
    }

    #[test]
    fn crc32_empty_data() {
        let crc = ParallelCrc::new(CrcAlgorithm::Crc32);
        // CRC-32 of empty string: init XOR final = 0xFFFFFFFF ^ 0xFFFFFFFF = 0x00000000
        assert_eq!(crc.compute(b""), 0x00000000);
    }

    #[test]
    fn crc32_single_byte() {
        let crc = ParallelCrc::new(CrcAlgorithm::Crc32);
        // CRC-32 of "A" is a well-known value
        let result = crc.compute(b"A");
        // Verify via sliced path as well
        assert_eq!(result, crc.compute_sliced(b"A"));
        // Known value: CRC-32("A") = 0xD3D99E8B
        assert_eq!(result, 0xD3D99E8B);
    }

    #[test]
    fn crc32_verify_pass() {
        let crc = ParallelCrc::new(CrcAlgorithm::Crc32);
        assert!(crc.verify(b"123456789", 0xCBF43926));
    }

    #[test]
    fn crc32_verify_fail() {
        let crc = ParallelCrc::new(CrcAlgorithm::Crc32);
        assert!(!crc.verify(b"123456789", 0xDEADBEEF));
    }

    // ---- CRC-16-CCITT known-answer tests ----

    #[test]
    fn crc16_ccitt_check_value() {
        let crc = ParallelCrc::new(CrcAlgorithm::Crc16Ccitt);
        assert_eq!(crc.compute(b"123456789"), 0x29B1);
    }

    #[test]
    fn crc16_ccitt_sliced_check_value() {
        let crc = ParallelCrc::new(CrcAlgorithm::Crc16Ccitt);
        assert_eq!(crc.compute_sliced(b"123456789"), 0x29B1);
    }

    // ---- CRC-8 tests ----

    #[test]
    fn crc8_check_value() {
        // CRC-8 (poly 0x07, init 0x00) of "123456789" = 0xF4
        let crc = ParallelCrc::new(CrcAlgorithm::Crc8);
        assert_eq!(crc.compute(b"123456789"), 0xF4);
    }

    // ---- CRC-32C (Castagnoli) tests ----

    #[test]
    fn crc32c_check_value() {
        // CRC-32C of "123456789" = 0xE3069283
        let crc = ParallelCrc::new(CrcAlgorithm::Crc32c);
        assert_eq!(crc.compute(b"123456789"), 0xE3069283);
    }

    #[test]
    fn crc32c_sliced_check_value() {
        let crc = ParallelCrc::new(CrcAlgorithm::Crc32c);
        assert_eq!(crc.compute_sliced(b"123456789"), 0xE3069283);
    }

    // ---- Streaming / incremental tests ----

    #[test]
    fn crc32_streaming_matches_one_shot() {
        let one_shot = ParallelCrc::new(CrcAlgorithm::Crc32);
        let expected = one_shot.compute(b"Hello, world!");

        let mut streaming = ParallelCrc::new(CrcAlgorithm::Crc32);
        streaming.update(b"Hello");
        streaming.update(b", ");
        streaming.update(b"world!");
        assert_eq!(streaming.finalize(), expected);
    }

    #[test]
    fn crc16_streaming_matches_one_shot() {
        let one_shot = ParallelCrc::new(CrcAlgorithm::Crc16Ccitt);
        let expected = one_shot.compute(b"test data for streaming");

        let mut streaming = ParallelCrc::new(CrcAlgorithm::Crc16Ccitt);
        streaming.update(b"test ");
        streaming.update(b"data ");
        streaming.update(b"for ");
        streaming.update(b"streaming");
        assert_eq!(streaming.finalize(), expected);
    }

    #[test]
    fn streaming_reset() {
        let mut crc = ParallelCrc::new(CrcAlgorithm::Crc32);
        crc.update(b"some data");
        crc.reset();
        crc.update(b"123456789");
        assert_eq!(crc.finalize(), 0xCBF43926);
    }

    // ---- Consistency: compute vs compute_sliced ----

    #[test]
    fn sliced_matches_standard_for_various_lengths() {
        let crc = ParallelCrc::new(CrcAlgorithm::Crc32);

        // Test a range of lengths to exercise chunk/remainder logic
        for len in 0..64 {
            let data: Vec<u8> = (0..len).map(|i: u64| (i * 37 + 13) as u8).collect();
            let standard = crc.compute(&data);
            let sliced = crc.compute_sliced(&data);
            assert_eq!(
                standard, sliced,
                "Mismatch at length {len}: standard=0x{standard:08X}, sliced=0x{sliced:08X}"
            );
        }
    }

    // ---- Accessor tests ----

    #[test]
    fn polynomial_accessor() {
        let crc = ParallelCrc::new(CrcAlgorithm::Crc32);
        assert_eq!(crc.polynomial(), 0x04C11DB7);
        assert_eq!(crc.width(), 32);
    }

    // ---- Custom CRC ----

    #[test]
    fn custom_crc_matches_preset() {
        // Build a custom CRC-32 manually and verify it matches the preset
        let config = CrcConfig {
            polynomial: 0x04C11DB7,
            initial_value: 0xFFFFFFFF,
            final_xor: 0xFFFFFFFF,
            reflect_input: true,
            reflect_output: true,
            width: 32,
        };
        let custom = ParallelCrc::custom(config);
        let preset = ParallelCrc::new(CrcAlgorithm::Crc32);

        let data = b"The quick brown fox jumps over the lazy dog";
        assert_eq!(custom.compute(data), preset.compute(data));
    }

    // ---- Bit reflection ----

    #[test]
    fn reflect_bits_correctness() {
        assert_eq!(reflect_bits(0b10110000, 8), 0b00001101);
        assert_eq!(reflect_bits(0b1, 8), 0b10000000);
        assert_eq!(reflect_bits(0xF0, 8), 0x0F);
        // 16-bit reflection
        assert_eq!(reflect_bits(0x8005, 16), 0xA001);
    }
}
