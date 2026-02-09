//! Access Code Detector (Correlate Access Code)
//!
//! Bit-level pattern detector that searches a binary stream for a specified
//! access code (sync word). Supports configurable Hamming distance threshold
//! for robustness against bit errors. Essential for packet synchronization
//! in digital communications.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::access_code_detector::{AccessCodeDetector, AccessCodeDetectorConfig};
//!
//! let config = AccessCodeDetectorConfig {
//!     access_code: vec![true, true, true, false, false, true, false, true], // 0xE5
//!     threshold: 1, // Allow 1 bit error
//! };
//! let mut detector = AccessCodeDetector::new(config);
//!
//! // Feed bits; returns detection when access code is found
//! let bits = vec![false; 20]; // random preamble
//! let detections = detector.process_block(&bits);
//! assert!(detections.is_empty()); // no match in zeros
//! ```

/// Configuration for the access code detector.
#[derive(Debug, Clone)]
pub struct AccessCodeDetectorConfig {
    /// The bit pattern to search for.
    pub access_code: Vec<bool>,
    /// Maximum allowed Hamming distance (bit errors). 0 = exact match only.
    pub threshold: usize,
}

/// Information about a detected access code match.
#[derive(Debug, Clone)]
pub struct AccessCodeDetection {
    /// Absolute bit position where the match ends (inclusive).
    pub bit_offset: u64,
    /// Number of bit errors in the match (Hamming distance).
    pub hamming_distance: usize,
}

/// Bit-level access code detector using a shift register.
#[derive(Debug, Clone)]
pub struct AccessCodeDetector {
    /// Access code as a bitmask (MSB = first bit)
    code_bits: u64,
    /// Mask for the valid bits (e.g., for 8-bit code: 0xFF)
    mask: u64,
    /// Code length in bits
    code_len: usize,
    /// Maximum allowed Hamming distance
    threshold: usize,
    /// Shift register
    shift_reg: u64,
    /// Total bits processed
    bit_count: u64,
    /// Whether we have enough bits in the register
    primed: bool,
}

impl AccessCodeDetector {
    /// Create a new detector from configuration.
    ///
    /// # Panics
    /// Panics if access_code is empty or longer than 64 bits.
    pub fn new(config: AccessCodeDetectorConfig) -> Self {
        let len = config.access_code.len();
        assert!(len > 0, "Access code must not be empty");
        assert!(len <= 64, "Access code must be 64 bits or shorter");

        let mut code_bits: u64 = 0;
        for &bit in &config.access_code {
            code_bits = (code_bits << 1) | (bit as u64);
        }

        let mask = if len == 64 { u64::MAX } else { (1u64 << len) - 1 };

        Self {
            code_bits,
            mask,
            code_len: len,
            threshold: config.threshold,
            shift_reg: 0,
            bit_count: 0,
            primed: false,
        }
    }

    /// Create a detector for a common 32-bit Barker-like sync word (0x1ACFFC1D).
    /// Used in various satellite and communications protocols.
    pub fn barker_32() -> Self {
        let code: u32 = 0x1ACFFC1D;
        let bits: Vec<bool> = (0..32).rev().map(|i| (code >> i) & 1 == 1).collect();
        Self::new(AccessCodeDetectorConfig {
            access_code: bits,
            threshold: 0,
        })
    }

    /// Create a detector from a hex string (e.g., "E5" for 0b11100101).
    pub fn from_hex(hex: &str, threshold: usize) -> Self {
        let bytes = (0..hex.len())
            .step_by(2)
            .map(|i| u8::from_str_radix(&hex[i..i + 2], 16).expect("Invalid hex"))
            .collect::<Vec<u8>>();

        let mut bits = Vec::new();
        for byte in &bytes {
            for j in (0..8).rev() {
                bits.push((byte >> j) & 1 == 1);
            }
        }

        Self::new(AccessCodeDetectorConfig {
            access_code: bits,
            threshold,
        })
    }

    /// Process a single bit. Returns Some(detection) if the access code is found.
    pub fn process_bit(&mut self, bit: bool) -> Option<AccessCodeDetection> {
        self.shift_reg = ((self.shift_reg << 1) | (bit as u64)) & self.mask;
        self.bit_count += 1;

        if !self.primed {
            if self.bit_count >= self.code_len as u64 {
                self.primed = true;
            } else {
                return None;
            }
        }

        let xor = self.shift_reg ^ self.code_bits;
        let hamming = xor.count_ones() as usize;

        if hamming <= self.threshold {
            Some(AccessCodeDetection {
                bit_offset: self.bit_count - 1,
                hamming_distance: hamming,
            })
        } else {
            None
        }
    }

    /// Process a block of bits. Returns all detections found.
    pub fn process_block(&mut self, bits: &[bool]) -> Vec<AccessCodeDetection> {
        let mut detections = Vec::new();
        for &bit in bits {
            if let Some(det) = self.process_bit(bit) {
                detections.push(det);
            }
        }
        detections
    }

    /// Process packed bytes (MSB first). Returns all detections found.
    pub fn process_bytes(&mut self, bytes: &[u8]) -> Vec<AccessCodeDetection> {
        let mut detections = Vec::new();
        for &byte in bytes {
            for j in (0..8).rev() {
                let bit = (byte >> j) & 1 == 1;
                if let Some(det) = self.process_bit(bit) {
                    detections.push(det);
                }
            }
        }
        detections
    }

    /// Get the code length in bits.
    pub fn code_len(&self) -> usize {
        self.code_len
    }

    /// Get total bits processed.
    pub fn bits_processed(&self) -> u64 {
        self.bit_count
    }

    /// Reset internal state.
    pub fn reset(&mut self) {
        self.shift_reg = 0;
        self.bit_count = 0;
        self.primed = false;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bits_from_str(s: &str) -> Vec<bool> {
        s.chars().map(|c| c == '1').collect()
    }

    #[test]
    fn test_exact_match() {
        let config = AccessCodeDetectorConfig {
            access_code: bits_from_str("11100101"),
            threshold: 0,
        };
        let mut det = AccessCodeDetector::new(config);

        // Feed the exact code
        let bits = bits_from_str("11100101");
        let detections = det.process_block(&bits);
        assert_eq!(detections.len(), 1);
        assert_eq!(detections[0].hamming_distance, 0);
        assert_eq!(detections[0].bit_offset, 7); // 0-indexed, last bit
    }

    #[test]
    fn test_code_in_stream() {
        let config = AccessCodeDetectorConfig {
            access_code: bits_from_str("1010"),
            threshold: 0,
        };
        let mut det = AccessCodeDetector::new(config);

        // Preamble + code + trailer
        let bits = bits_from_str("000010100000");
        let detections = det.process_block(&bits);
        assert_eq!(detections.len(), 1);
        assert_eq!(detections[0].bit_offset, 7); // code ends at position 7
    }

    #[test]
    fn test_threshold_tolerance() {
        let config = AccessCodeDetectorConfig {
            access_code: bits_from_str("11110000"),
            threshold: 2, // Allow up to 2 bit errors
        };
        let mut det = AccessCodeDetector::new(config);

        // 1 bit error: 11110010 (bit 6 flipped)
        let bits = bits_from_str("11110010");
        let detections = det.process_block(&bits);
        assert_eq!(detections.len(), 1);
        assert_eq!(detections[0].hamming_distance, 1);
    }

    #[test]
    fn test_no_false_positive() {
        let config = AccessCodeDetectorConfig {
            access_code: bits_from_str("11111111"),
            threshold: 0,
        };
        let mut det = AccessCodeDetector::new(config);

        // All zeros — should never match
        let bits = vec![false; 100];
        let detections = det.process_block(&bits);
        assert!(detections.is_empty());
    }

    #[test]
    fn test_multiple_detections() {
        let config = AccessCodeDetectorConfig {
            access_code: bits_from_str("101"),
            threshold: 0,
        };
        let mut det = AccessCodeDetector::new(config);

        // Two occurrences: positions 2-4 and 10-12
        let bits = bits_from_str("0010100001010");
        let detections = det.process_block(&bits);
        assert!(detections.len() >= 2, "Should find at least 2: {:?}", detections);
    }

    #[test]
    fn test_from_hex() {
        let det = AccessCodeDetector::from_hex("E5", 0);
        assert_eq!(det.code_len(), 8);
        // 0xE5 = 11100101
        assert_eq!(det.code_bits, 0xE5);
    }

    #[test]
    fn test_barker_32() {
        let det = AccessCodeDetector::barker_32();
        assert_eq!(det.code_len(), 32);
        assert_eq!(det.code_bits, 0x1ACFFC1D);
    }

    #[test]
    fn test_process_bytes() {
        let config = AccessCodeDetectorConfig {
            access_code: bits_from_str("11100101"), // 0xE5
            threshold: 0,
        };
        let mut det = AccessCodeDetector::new(config);

        // Byte 0x00, then 0xE5
        let bytes = &[0x00u8, 0xE5];
        let detections = det.process_bytes(bytes);
        assert_eq!(detections.len(), 1);
    }

    #[test]
    fn test_reset() {
        let config = AccessCodeDetectorConfig {
            access_code: bits_from_str("1111"),
            threshold: 0,
        };
        let mut det = AccessCodeDetector::new(config);

        // Feed partial code
        det.process_block(&bits_from_str("111"));
        assert_eq!(det.bits_processed(), 3);

        det.reset();
        assert_eq!(det.bits_processed(), 0);

        // After reset, the partial should not cause a detection
        let detections = det.process_block(&bits_from_str("10000"));
        assert!(detections.is_empty());
    }

    #[test]
    fn test_threshold_too_high_matches_everything() {
        let config = AccessCodeDetectorConfig {
            access_code: bits_from_str("1010"),
            threshold: 4, // threshold >= code_len → matches everything
        };
        let mut det = AccessCodeDetector::new(config);

        let bits = vec![false; 10];
        let detections = det.process_block(&bits);
        // Once primed (after 4 bits), every bit should match
        assert_eq!(detections.len(), 7); // 10 - 4 + 1 = 7
    }

    #[test]
    fn test_single_bit_code() {
        let config = AccessCodeDetectorConfig {
            access_code: vec![true],
            threshold: 0,
        };
        let mut det = AccessCodeDetector::new(config);

        let bits = bits_from_str("01010");
        let detections = det.process_block(&bits);
        assert_eq!(detections.len(), 2); // two '1' bits
    }
}
