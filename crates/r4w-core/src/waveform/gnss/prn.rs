//! GNSS PRN Code Generators
//!
//! Implements spreading code generators for multiple GNSS constellations:
//!
//! - **GPS C/A**: Gold codes from G1/G2 LFSRs (1023 chips)
//! - **GLONASS L1OF**: Single 9-stage LFSR m-sequence (511 chips)
//! - **Galileo E1**: Memory codes (4092 chips per PRN)
//! - **GPS L5**: 13-stage LFSR codes (10230 chips)
//!
//! ## GPS C/A Code Structure
//!
//! ```text
//! G1 LFSR (10-bit): x^10 + x^3 + 1
//! ┌─┬─┬─┬─┬─┬─┬─┬─┬─┬──┐
//! │1│2│3│4│5│6│7│8│9│10│──→ output
//! └─┴─┴─┴─┴─┴─┴─┴─┴─┴──┘
//!        ↑              ↑
//!        └──── XOR ─────┘ (taps 3, 10)
//!
//! G2 LFSR (10-bit): x^10 + x^9 + x^8 + x^6 + x^3 + x^2 + 1
//! ┌─┬─┬─┬─┬─┬─┬─┬─┬─┬──┐
//! │1│2│3│4│5│6│7│8│9│10│
//! └─┴─┴─┴─┴─┴─┴─┴─┴─┴──┘
//!   ↑ ↑    ↑     ↑ ↑  ↑
//!   └─┴──XOR─────┴─┴──┘ (taps 2, 3, 6, 8, 9, 10)
//!
//! C/A code = G1 ⊕ G2(tap_a) ⊕ G2(tap_b)
//! ```

use crate::spreading::{Lfsr, PnSequence};

/// GPS C/A code tap assignments for PRN 1-32 (IS-GPS-200)
/// Each entry is (G2_tap_1, G2_tap_2) where taps are 1-based positions
const GPS_CA_TAPS: [(u8, u8); 32] = [
    (2, 6),   // PRN 1
    (3, 7),   // PRN 2
    (4, 8),   // PRN 3
    (5, 9),   // PRN 4
    (1, 9),   // PRN 5
    (2, 10),  // PRN 6
    (1, 8),   // PRN 7
    (2, 9),   // PRN 8
    (3, 10),  // PRN 9
    (2, 3),   // PRN 10
    (3, 4),   // PRN 11
    (5, 6),   // PRN 12
    (6, 7),   // PRN 13
    (7, 8),   // PRN 14
    (8, 9),   // PRN 15
    (9, 10),  // PRN 16
    (1, 4),   // PRN 17
    (2, 5),   // PRN 18
    (3, 6),   // PRN 19
    (4, 7),   // PRN 20
    (5, 8),   // PRN 21
    (6, 9),   // PRN 22
    (1, 3),   // PRN 23
    (4, 6),   // PRN 24
    (5, 7),   // PRN 25
    (6, 8),   // PRN 26
    (7, 9),   // PRN 27
    (8, 10),  // PRN 28
    (1, 6),   // PRN 29
    (2, 7),   // PRN 30
    (3, 8),   // PRN 31
    (4, 9),   // PRN 32
];

/// GPS C/A code generator
///
/// Generates the Gold code for a specific GPS satellite PRN using two
/// 10-stage LFSRs (G1 and G2) per IS-GPS-200.
#[derive(Debug, Clone)]
pub struct GpsCaCodeGenerator {
    /// G1 LFSR: polynomial x^10 + x^3 + 1
    g1: Lfsr,
    /// G2 LFSR: polynomial x^10 + x^9 + x^8 + x^6 + x^3 + x^2 + 1
    g2: Lfsr,
    /// PRN number (1-32)
    prn: u8,
    /// G2 output tap positions for this PRN
    tap_a: u8,
    tap_b: u8,
    /// Current chip position
    position: usize,
}

impl GpsCaCodeGenerator {
    /// Create a new GPS C/A code generator for the specified PRN
    pub fn new(prn: u8) -> Self {
        assert!(prn >= 1 && prn <= 32, "GPS PRN must be 1-32, got {}", prn);
        let (tap_a, tap_b) = GPS_CA_TAPS[(prn - 1) as usize];

        // G1: x^10 + x^3 + 1 → feedback taps at positions 3 and 10
        // Polynomial representation: bits 3 and 10 set = 0x204
        let g1 = Lfsr::new(10, 0x204, 0x3FF); // All ones initial state

        // G2: x^10 + x^9 + x^8 + x^6 + x^3 + x^2 + 1
        // Polynomial: bits 2, 3, 6, 8, 9, 10 = 0x2E4 + 0x100 + 0x004 ...
        // = 0x3A6 → positions 2,3,6,8,9,10
        let g2 = Lfsr::new(10, 0x3A6, 0x3FF); // All ones initial state

        Self {
            g1,
            g2,
            prn,
            tap_a,
            tap_b,
            position: 0,
        }
    }

    /// Get the PRN number
    pub fn prn(&self) -> u8 {
        self.prn
    }

    /// Generate the complete 1023-chip C/A code as +1/-1 values
    pub fn generate_code(&mut self) -> Vec<i8> {
        self.reset();
        (0..1023).map(|_| self.next_chip()).collect()
    }

    /// Generate the complete code as 0/1 bits
    pub fn generate_code_bits(&mut self) -> Vec<u8> {
        self.reset();
        (0..1023).map(|_| {
            let g1_out = self.g1.clock();
            let g2_tap = self.g2.tap_output(self.tap_a) ^ self.g2.tap_output(self.tap_b);
            self.g2.clock();
            self.position = (self.position + 1) % 1023;
            g1_out ^ g2_tap
        }).collect()
    }
}

impl PnSequence for GpsCaCodeGenerator {
    fn next_chip(&mut self) -> i8 {
        // G1 output is MSB (position 10)
        let g1_out = self.g1.clock();

        // G2 output is XOR of two specific taps (before clocking)
        let g2_tap = self.g2.tap_output(self.tap_a) ^ self.g2.tap_output(self.tap_b);
        self.g2.clock();

        let bit = g1_out ^ g2_tap;
        self.position = (self.position + 1) % 1023;

        // Map: 0 → +1, 1 → -1
        if bit == 0 { 1 } else { -1 }
    }

    fn reset(&mut self) {
        self.g1 = Lfsr::new(10, 0x204, 0x3FF);
        self.g2 = Lfsr::new(10, 0x3A6, 0x3FF);
        self.position = 0;
    }

    fn length(&self) -> usize {
        1023
    }
}

/// GLONASS L1OF code generator
///
/// Single 9-stage LFSR producing a 511-chip m-sequence.
/// All GLONASS satellites use the same code (FDMA separates signals).
/// Polynomial: 1 + x^5 + x^9
#[derive(Debug, Clone)]
pub struct GlonassCodeGenerator {
    /// 9-stage LFSR
    lfsr: Lfsr,
    /// GLONASS frequency channel (-7 to +6)
    frequency_channel: i8,
    /// Current position
    position: usize,
}

impl GlonassCodeGenerator {
    /// Create a new GLONASS code generator
    pub fn new(frequency_channel: i8) -> Self {
        assert!(frequency_channel >= -7 && frequency_channel <= 6,
                "GLONASS frequency channel must be -7 to +6");
        // Polynomial: x^9 + x^5 + 1 = taps at positions 5 and 9 = 0x110
        let lfsr = Lfsr::new(9, 0x110, 0x1FF); // All ones initial state
        Self {
            lfsr,
            frequency_channel,
            position: 0,
        }
    }

    /// Get the frequency channel
    pub fn frequency_channel(&self) -> i8 {
        self.frequency_channel
    }

    /// Get the carrier frequency for this channel in Hz
    pub fn carrier_frequency_hz(&self) -> f64 {
        1_602_000_000.0 + self.frequency_channel as f64 * 562_500.0
    }

    /// Generate the complete 511-chip code
    pub fn generate_code(&mut self) -> Vec<i8> {
        self.reset();
        (0..511).map(|_| self.next_chip()).collect()
    }
}

impl PnSequence for GlonassCodeGenerator {
    fn next_chip(&mut self) -> i8 {
        let bit = self.lfsr.clock();
        self.position = (self.position + 1) % 511;
        if bit == 0 { 1 } else { -1 }
    }

    fn reset(&mut self) {
        self.lfsr = Lfsr::new(9, 0x110, 0x1FF);
        self.position = 0;
    }

    fn length(&self) -> usize {
        511
    }
}

/// Galileo E1 code generator
///
/// Uses memory codes (4092 chips per PRN). The codes are generated from
/// a 14-stage LFSR and then truncated/stored. For simplicity, we generate
/// deterministic codes based on the PRN using a seed-based approach matching
/// the Galileo ICD structure.
#[derive(Debug, Clone)]
pub struct GalileoE1CodeGenerator {
    /// Pre-computed code for this PRN
    code: Vec<i8>,
    /// PRN number (1-50)
    prn: u8,
    /// Current position in the code
    position: usize,
}

impl GalileoE1CodeGenerator {
    /// Create a new Galileo E1 code generator
    pub fn new(prn: u8) -> Self {
        assert!(prn >= 1 && prn <= 50, "Galileo PRN must be 1-50");
        let code = Self::generate_memory_code(prn);
        Self {
            code,
            prn,
            position: 0,
        }
    }

    /// Get the PRN number
    pub fn prn(&self) -> u8 {
        self.prn
    }

    /// Generate memory code for a given PRN
    /// Uses a deterministic LFSR-based generation matching Galileo ICD structure.
    /// The actual Galileo codes are stored in the ICD; this generates codes with
    /// the same statistical properties for educational/simulation purposes.
    fn generate_memory_code(prn: u8) -> Vec<i8> {
        // Use two 14-stage LFSRs with PRN-specific initial states
        // This produces codes with good cross-correlation properties
        let seed_a = 0x3FFF_u32; // All ones
        let seed_b = (prn as u32 * 0x1234 + 0x5678) & 0x3FFF;

        let mut lfsr_a = Lfsr::new(14, 0x2015, seed_a.max(1)); // x^14 + x^13 + x^5 + x^3 + 1
        let mut lfsr_b = Lfsr::new(14, 0x2E57, seed_b.max(1)); // x^14 + x^12 + x^10 + x^7 + x^5 + x^3 + 1

        (0..4092).map(|_| {
            let bit = lfsr_a.clock() ^ lfsr_b.clock();
            if bit == 0 { 1 } else { -1 }
        }).collect()
    }
}

impl PnSequence for GalileoE1CodeGenerator {
    fn next_chip(&mut self) -> i8 {
        let chip = self.code[self.position];
        self.position = (self.position + 1) % 4092;
        chip
    }

    fn reset(&mut self) {
        self.position = 0;
    }

    fn length(&self) -> usize {
        4092
    }
}

/// GPS L5 code generator
///
/// Uses two 13-stage LFSRs to produce 10230-chip codes.
/// I5 and Q5 channels have independent codes with Neumann-Hoffman secondary codes.
#[derive(Debug, Clone)]
pub struct GpsL5CodeGenerator {
    /// Pre-computed code
    code: Vec<i8>,
    /// PRN number
    prn: u8,
    /// Channel (I5 or Q5)
    _is_q_channel: bool,
    /// Current position
    position: usize,
}

impl GpsL5CodeGenerator {
    /// Create a GPS L5 I-channel code generator
    pub fn new_i5(prn: u8) -> Self {
        assert!(prn >= 1 && prn <= 32, "GPS L5 PRN must be 1-32");
        let code = Self::generate_l5_code(prn, false);
        Self { code, prn, _is_q_channel: false, position: 0 }
    }

    /// Create a GPS L5 Q-channel code generator
    pub fn new_q5(prn: u8) -> Self {
        assert!(prn >= 1 && prn <= 32, "GPS L5 PRN must be 1-32");
        let code = Self::generate_l5_code(prn, true);
        Self { code, prn, _is_q_channel: true, position: 0 }
    }

    /// Get the PRN number
    pub fn prn(&self) -> u8 {
        self.prn
    }

    /// Neumann-Hoffman secondary code for I5 channel (10 bits)
    pub const NH_I5: [i8; 10] = [1, 1, 1, 1, -1, -1, 1, -1, 1, -1];

    /// Neumann-Hoffman secondary code for Q5 channel (20 bits)
    pub const NH_Q5: [i8; 20] = [
        1, 1, 1, 1, 1, -1, 1, 1, -1, -1,
        1, -1, 1, -1, 1, -1, -1, -1, 1, 1,
    ];

    /// Generate L5 code using two 13-stage LFSRs
    fn generate_l5_code(prn: u8, q_channel: bool) -> Vec<i8> {
        // XA LFSR: x^13 + x^12 + x^10 + x^9 + 1
        let xa_poly = 0x1E01_u32; // taps 9, 10, 12, 13
        // XB LFSR: different polynomial for I and Q channels
        let xb_poly = if q_channel {
            0x1B4F_u32 // Q channel polynomial
        } else {
            0x1AE3_u32 // I channel polynomial
        };

        let xa_init = 0x1FFF_u32; // All ones
        // PRN-specific initial state for XB
        let xb_init = ((prn as u32 * 0x2468 + if q_channel { 0xACE0 } else { 0x1357 }) & 0x1FFF).max(1);

        let mut xa = Lfsr::new(13, xa_poly, xa_init);
        let mut xb = Lfsr::new(13, xb_poly, xb_init);

        (0..10230).map(|_| {
            let bit = xa.clock() ^ xb.clock();
            if bit == 0 { 1 } else { -1 }
        }).collect()
    }
}

impl PnSequence for GpsL5CodeGenerator {
    fn next_chip(&mut self) -> i8 {
        let chip = self.code[self.position];
        self.position = (self.position + 1) % 10230;
        chip
    }

    fn reset(&mut self) {
        self.position = 0;
    }

    fn length(&self) -> usize {
        10230
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::spreading::{autocorrelation, cross_correlation, max_cross_correlation};

    #[test]
    fn test_gps_ca_code_length() {
        let mut gen = GpsCaCodeGenerator::new(1);
        let code = gen.generate_code();
        assert_eq!(code.len(), 1023);
    }

    #[test]
    fn test_gps_ca_code_values() {
        let mut gen = GpsCaCodeGenerator::new(1);
        let code = gen.generate_code();
        // All chips should be +1 or -1
        assert!(code.iter().all(|&c| c == 1 || c == -1));
    }

    #[test]
    fn test_gps_ca_autocorrelation_peak() {
        let mut gen = GpsCaCodeGenerator::new(1);
        let code = gen.generate_code();

        // Peak at lag 0 should be 1023
        let peak = autocorrelation(&code, 0);
        assert_eq!(peak, 1023);
    }

    #[test]
    fn test_gps_ca_cross_correlation_bounded() {
        let mut gen1 = GpsCaCodeGenerator::new(1);
        let mut gen7 = GpsCaCodeGenerator::new(7);
        let code1 = gen1.generate_code();
        let code7 = gen7.generate_code();

        // Gold code cross-correlation should be bounded
        // For n=10: t(n) = 2^((10+2)/2) + 1 = 65
        let max_xcorr = max_cross_correlation(&code1, &code7);
        assert!(max_xcorr <= 65, "Cross-correlation {} exceeds bound 65", max_xcorr);
    }

    #[test]
    fn test_gps_ca_different_prns_different() {
        let mut gen1 = GpsCaCodeGenerator::new(1);
        let mut gen2 = GpsCaCodeGenerator::new(2);
        let code1 = gen1.generate_code();
        let code2 = gen2.generate_code();
        assert_ne!(code1, code2);
    }

    #[test]
    fn test_gps_ca_deterministic() {
        let mut gen1a = GpsCaCodeGenerator::new(5);
        let mut gen1b = GpsCaCodeGenerator::new(5);
        let code_a = gen1a.generate_code();
        let code_b = gen1b.generate_code();
        assert_eq!(code_a, code_b);
    }

    #[test]
    fn test_glonass_code_length() {
        let mut gen = GlonassCodeGenerator::new(0);
        let code = gen.generate_code();
        assert_eq!(code.len(), 511);
    }

    #[test]
    fn test_glonass_carrier_frequency() {
        let gen = GlonassCodeGenerator::new(0);
        assert_eq!(gen.carrier_frequency_hz(), 1_602_000_000.0);

        let gen_pos = GlonassCodeGenerator::new(6);
        assert!((gen_pos.carrier_frequency_hz() - 1_605_375_000.0).abs() < 1.0);
    }

    #[test]
    fn test_galileo_e1_code_length() {
        let mut gen = GalileoE1CodeGenerator::new(1);
        assert_eq!(gen.length(), 4092);
        let code = gen.generate_sequence();
        assert_eq!(code.len(), 4092);
    }

    #[test]
    fn test_galileo_e1_different_prns() {
        let mut gen1 = GalileoE1CodeGenerator::new(1);
        let mut gen2 = GalileoE1CodeGenerator::new(2);
        let code1 = gen1.generate_sequence();
        let code2 = gen2.generate_sequence();
        assert_ne!(code1, code2);
    }

    #[test]
    fn test_gps_l5_code_length() {
        let gen = GpsL5CodeGenerator::new_i5(1);
        assert_eq!(gen.length(), 10230);
    }

    #[test]
    fn test_gps_l5_iq_different() {
        let mut gen_i = GpsL5CodeGenerator::new_i5(1);
        let mut gen_q = GpsL5CodeGenerator::new_q5(1);
        let code_i = gen_i.generate_sequence();
        let code_q = gen_q.generate_sequence();
        assert_ne!(code_i, code_q);
    }
}
