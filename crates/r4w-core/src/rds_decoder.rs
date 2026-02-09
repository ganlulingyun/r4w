//! RDS Decoder — FM Radio Data System (RDS/RBDS) decoder
//!
//! Decodes the 57 kHz BPSK subcarrier in FM broadcast signals carrying
//! station name (PS), radio text (RT), program type (PTY), clock/time,
//! and traffic announcements per IEC 62106 / NRSC-4-B (RBDS).
//! GNU Radio equivalent: `gr-rds` (out-of-tree module).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::rds_decoder::{RdsDecoder, RdsEvent};
//!
//! let mut decoder = RdsDecoder::new();
//! assert_eq!(decoder.program_service(), "        "); // 8 spaces initially
//! ```

/// RDS group (4 blocks of 16 bits each + check words).
#[derive(Debug, Clone)]
pub struct RdsGroup {
    /// PI code (block A).
    pub pi_code: u16,
    /// Group type (0-15).
    pub group_type: u8,
    /// Version (A or B).
    pub version: RdsVersion,
    /// Traffic program flag.
    pub tp: bool,
    /// Program type code (0-31).
    pub pty: u8,
    /// Raw blocks (A, B, C/C', D).
    pub blocks: [u16; 4],
}

/// RDS group version.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum RdsVersion {
    A,
    B,
}

/// Decoded RDS events.
#[derive(Debug, Clone)]
pub enum RdsEvent {
    /// Program Service name update (8 characters).
    ProgramService(String),
    /// Radio Text update (up to 64 characters).
    RadioText(String),
    /// Program Type.
    ProgramType(u8, String),
    /// Clock/Time (MJD, hour, minute, offset).
    ClockTime {
        mjd: u32,
        hour: u8,
        minute: u8,
        offset: i8,
    },
    /// Traffic Announcement active.
    TrafficAnnouncement(bool),
    /// Alternative Frequency list.
    AlternativeFreq(Vec<f64>),
}

/// RDS sync/framing detector.
#[derive(Debug, Clone)]
pub struct RdsSyncDetector {
    bit_buffer: Vec<bool>,
    synced: bool,
    block_count: usize,
}

impl RdsSyncDetector {
    /// Create new sync detector.
    pub fn new() -> Self {
        Self {
            bit_buffer: Vec::new(),
            synced: false,
            block_count: 0,
        }
    }

    /// Feed bits and extract complete groups.
    pub fn feed_bits(&mut self, bits: &[bool]) -> Vec<RdsGroup> {
        self.bit_buffer.extend_from_slice(bits);
        let mut groups = Vec::new();

        // Each group = 4 blocks * 26 bits = 104 bits
        while self.bit_buffer.len() >= 104 {
            if !self.synced {
                // Search for sync by checking offset word A
                if check_syndrome(&self.bit_buffer[..26]) == OFFSET_A {
                    self.synced = true;
                } else {
                    self.bit_buffer.drain(..1);
                    continue;
                }
            }

            if self.bit_buffer.len() < 104 {
                break;
            }

            // Extract 4 blocks
            let mut blocks = [0u16; 4];
            let mut valid = true;
            let offsets = [OFFSET_A, OFFSET_B, OFFSET_C, OFFSET_D];

            for (bi, &expected_offset) in offsets.iter().enumerate() {
                let start = bi * 26;
                let block_bits = &self.bit_buffer[start..start + 26];
                let syndrome = check_syndrome(block_bits);

                // Allow offset C' for version B groups
                if syndrome != expected_offset
                    && !(bi == 2 && syndrome == OFFSET_C_PRIME)
                {
                    // Try error correction
                    if let Some(corrected) = try_error_correct(block_bits, expected_offset) {
                        blocks[bi] = corrected;
                    } else {
                        valid = false;
                        break;
                    }
                } else {
                    blocks[bi] = bits_to_u16(&block_bits[..16]);
                }
            }

            self.bit_buffer.drain(..104);

            if !valid {
                self.synced = false;
                continue;
            }

            self.block_count += 1;

            // Parse group header from block B
            let b = blocks[1];
            let group_type = ((b >> 12) & 0xF) as u8;
            let version = if (b >> 11) & 1 == 0 {
                RdsVersion::A
            } else {
                RdsVersion::B
            };
            let tp = (b >> 10) & 1 == 1;
            let pty = ((b >> 5) & 0x1F) as u8;

            groups.push(RdsGroup {
                pi_code: blocks[0],
                group_type,
                version,
                tp,
                pty,
                blocks,
            });
        }

        groups
    }

    /// Whether sync has been acquired.
    pub fn is_synced(&self) -> bool {
        self.synced
    }

    /// Total groups decoded.
    pub fn block_count(&self) -> usize {
        self.block_count
    }
}

impl Default for RdsSyncDetector {
    fn default() -> Self {
        Self::new()
    }
}

/// RDS decoder with accumulated state.
#[derive(Debug, Clone)]
pub struct RdsDecoder {
    sync: RdsSyncDetector,
    /// Program Service name (8 chars).
    ps_name: [u8; 8],
    /// Radio Text (64 chars).
    radio_text: [u8; 64],
    /// Radio Text A/B flag for detecting changes.
    rt_ab: bool,
    /// PI code.
    pi_code: u16,
    /// PTY code.
    pty: u8,
    /// Events generated.
    events: Vec<RdsEvent>,
}

impl RdsDecoder {
    /// Create new RDS decoder.
    pub fn new() -> Self {
        Self {
            sync: RdsSyncDetector::new(),
            ps_name: [b' '; 8],
            radio_text: [b' '; 64],
            rt_ab: false,
            pi_code: 0,
            pty: 0,
            events: Vec::new(),
        }
    }

    /// Feed demodulated bits.
    pub fn feed_bits(&mut self, bits: &[bool]) -> Vec<RdsEvent> {
        let groups = self.sync.feed_bits(bits);
        let mut events = Vec::new();

        for group in &groups {
            self.pi_code = group.pi_code;
            self.pty = group.pty;

            match group.group_type {
                0 => {
                    // Group 0A/0B: Program Service
                    let segment = (group.blocks[1] & 0x3) as usize;
                    let d = group.blocks[3];
                    let c1 = ((d >> 8) & 0xFF) as u8;
                    let c2 = (d & 0xFF) as u8;
                    let idx = segment * 2;
                    if idx + 1 < 8 {
                        self.ps_name[idx] = if c1 >= 0x20 && c1 < 0x7F { c1 } else { b' ' };
                        self.ps_name[idx + 1] = if c2 >= 0x20 && c2 < 0x7F { c2 } else { b' ' };
                    }
                    events.push(RdsEvent::ProgramService(self.program_service().to_string()));
                }
                2 => {
                    // Group 2A: Radio Text
                    let ab_flag = (group.blocks[1] >> 4) & 1 == 1;
                    if ab_flag != self.rt_ab {
                        self.radio_text = [b' '; 64];
                        self.rt_ab = ab_flag;
                    }
                    let segment = (group.blocks[1] & 0xF) as usize;
                    if group.version == RdsVersion::A {
                        let idx = segment * 4;
                        let c = group.blocks[2];
                        let d = group.blocks[3];
                        if idx + 3 < 64 {
                            self.radio_text[idx] = ((c >> 8) & 0xFF) as u8;
                            self.radio_text[idx + 1] = (c & 0xFF) as u8;
                            self.radio_text[idx + 2] = ((d >> 8) & 0xFF) as u8;
                            self.radio_text[idx + 3] = (d & 0xFF) as u8;
                        }
                    }
                    events.push(RdsEvent::RadioText(self.radio_text().to_string()));
                }
                4 if group.version == RdsVersion::A => {
                    // Group 4A: Clock/Time
                    let c = group.blocks[2];
                    let d = group.blocks[3];
                    let mjd = ((group.blocks[1] as u32 & 0x3) << 15) | ((c as u32) >> 1);
                    let hour = (((c & 1) << 4) | ((d >> 12) & 0xF)) as u8;
                    let minute = ((d >> 6) & 0x3F) as u8;
                    let offset = (d & 0x3F) as i8;
                    let offset = if d & 0x20 != 0 { -offset } else { offset };

                    events.push(RdsEvent::ClockTime {
                        mjd,
                        hour,
                        minute,
                        offset,
                    });
                }
                _ => {}
            }
        }

        self.events.extend(events.clone());
        events
    }

    /// Get current Program Service name.
    pub fn program_service(&self) -> &str {
        std::str::from_utf8(&self.ps_name).unwrap_or("        ")
    }

    /// Get current Radio Text.
    pub fn radio_text(&self) -> String {
        String::from_utf8_lossy(&self.radio_text).trim_end().to_string()
    }

    /// Get PI code.
    pub fn pi_code(&self) -> u16 {
        self.pi_code
    }

    /// Get PTY code.
    pub fn pty(&self) -> u8 {
        self.pty
    }

    /// Reset decoder state.
    pub fn reset(&mut self) {
        self.sync = RdsSyncDetector::new();
        self.ps_name = [b' '; 8];
        self.radio_text = [b' '; 64];
        self.pi_code = 0;
        self.pty = 0;
        self.events.clear();
    }
}

impl Default for RdsDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// RDS offset words for block synchronization.
const OFFSET_A: u16 = 0x0FC;
const OFFSET_B: u16 = 0x198;
const OFFSET_C: u16 = 0x168;
const OFFSET_C_PRIME: u16 = 0x350;
const OFFSET_D: u16 = 0x1B4;

/// RDS generator polynomial: x^10 + x^8 + x^7 + x^5 + x^4 + x^3 + 1
const RDS_POLY: u16 = 0x1B9; // 110111001 in binary (degree 10 CRC)

/// Compute syndrome of a 26-bit block (16 data + 10 check).
fn check_syndrome(bits: &[bool]) -> u16 {
    if bits.len() < 26 {
        return 0xFFFF;
    }
    let mut reg: u16 = 0;
    for i in 0..26 {
        let feedback = ((reg >> 9) & 1) ^ if bits[i] { 1 } else { 0 };
        reg = (reg << 1) & 0x3FF;
        if feedback == 1 {
            reg ^= RDS_POLY;
        }
    }
    reg
}

/// Try to correct single-bit errors in a 26-bit block.
fn try_error_correct(bits: &[bool], expected_offset: u16) -> Option<u16> {
    // Try flipping each bit
    let mut test_bits = bits[..26].to_vec();
    for i in 0..26 {
        test_bits[i] = !test_bits[i];
        if check_syndrome(&test_bits) == expected_offset {
            return Some(bits_to_u16(&test_bits[..16]));
        }
        test_bits[i] = !test_bits[i]; // Restore
    }
    None
}

/// Convert 16 bits (MSB first) to u16.
fn bits_to_u16(bits: &[bool]) -> u16 {
    let mut val: u16 = 0;
    for (i, &b) in bits.iter().enumerate().take(16) {
        if b {
            val |= 1 << (15 - i);
        }
    }
    val
}

/// Get PTY name string.
pub fn pty_name(pty: u8) -> &'static str {
    match pty {
        0 => "No programme type",
        1 => "News",
        2 => "Information",
        3 => "Sports",
        4 => "Talk",
        5 => "Rock",
        6 => "Classic Rock",
        7 => "Adult Hits",
        8 => "Soft Rock",
        9 => "Top 40",
        10 => "Country",
        11 => "Oldies",
        12 => "Soft",
        13 => "Nostalgia",
        14 => "Jazz",
        15 => "Classical",
        16 => "R&B",
        17 => "Soft R&B",
        18 => "Language",
        19 => "Religious Music",
        20 => "Religious Talk",
        21 => "Personality",
        22 => "Public",
        23 => "College",
        24 => "Spanish Talk",
        25 => "Spanish Music",
        26 => "Hip Hop",
        27..=28 => "Unassigned",
        29 => "Weather",
        30 => "Emergency Test",
        31 => "Emergency",
        _ => "Unknown",
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_bits_to_u16() {
        let bits = [true; 16];
        assert_eq!(bits_to_u16(&bits), 0xFFFF);

        let bits = [false; 16];
        assert_eq!(bits_to_u16(&bits), 0x0000);

        // 0xA5A5 = 1010_0101_1010_0101
        let bits = [
            true, false, true, false, false, true, false, true, true, false, true, false, false,
            true, false, true,
        ];
        assert_eq!(bits_to_u16(&bits), 0xA5A5);
    }

    #[test]
    fn test_pty_names() {
        assert_eq!(pty_name(0), "No programme type");
        assert_eq!(pty_name(1), "News");
        assert_eq!(pty_name(5), "Rock");
        assert_eq!(pty_name(15), "Classical");
        assert_eq!(pty_name(31), "Emergency");
    }

    #[test]
    fn test_decoder_creation() {
        let decoder = RdsDecoder::new();
        assert_eq!(decoder.program_service(), "        ");
        assert_eq!(decoder.pi_code(), 0);
        assert_eq!(decoder.pty(), 0);
    }

    #[test]
    fn test_decoder_reset() {
        let mut decoder = RdsDecoder::new();
        decoder.pi_code = 0x1234;
        decoder.pty = 5;
        decoder.reset();
        assert_eq!(decoder.pi_code(), 0);
        assert_eq!(decoder.pty(), 0);
    }

    #[test]
    fn test_sync_detector_creation() {
        let sync = RdsSyncDetector::new();
        assert!(!sync.is_synced());
        assert_eq!(sync.block_count(), 0);
    }

    #[test]
    fn test_syndrome_computation() {
        // All zeros should give a specific syndrome
        let bits = [false; 26];
        let syn = check_syndrome(&bits);
        // Syndrome of all-zero block should be 0 (valid codeword for offset 0)
        assert_eq!(syn, 0);
    }

    #[test]
    fn test_syndrome_short_input() {
        let bits = [false; 10];
        assert_eq!(check_syndrome(&bits), 0xFFFF);
    }

    #[test]
    fn test_default_impls() {
        let _decoder = RdsDecoder::default();
        let _sync = RdsSyncDetector::default();
    }

    #[test]
    fn test_rds_version_eq() {
        assert_eq!(RdsVersion::A, RdsVersion::A);
        assert_ne!(RdsVersion::A, RdsVersion::B);
    }

    #[test]
    fn test_radio_text_initial() {
        let decoder = RdsDecoder::new();
        let rt = decoder.radio_text();
        assert!(rt.is_empty() || rt.chars().all(|c| c == ' '));
    }
}
