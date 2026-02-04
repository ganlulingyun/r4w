//! GPS LNAV Navigation Message
//!
//! The GPS L1 C/A navigation message (LNAV) is transmitted at 50 bps,
//! organized into 5 subframes of 300 bits each (6 seconds per subframe,
//! 30 seconds per complete frame).
//!
//! ## Frame Structure
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────┐
//! │                    LNAV Frame (30s)                      │
//! ├──────────┬──────────┬──────────┬──────────┬──────────────┤
//! │ SF 1     │ SF 2     │ SF 3     │ SF 4     │ SF 5         │
//! │ Clock    │ Ephemeris│ Ephemeris│ Almanac  │ Almanac      │
//! │ (6s)     │ (6s)     │ (6s)     │ (6s)     │ (6s)         │
//! └──────────┴──────────┴──────────┴──────────┴──────────────┘
//!
//! Each subframe: 10 words × 30 bits = 300 bits
//! Word 1: TLM (Telemetry)
//! Word 2: HOW (Handover Word) - contains TOW
//! ```

use super::types::{NavFrame, SubframeClock};

/// GPS LNAV message encoder/decoder
// trace:FR-035 | ai:claude
#[derive(Debug, Clone)]
pub struct LnavMessage {
    /// Current subframe being assembled
    current_bits: Vec<i8>,
    /// Completed subframes
    subframes: Vec<NavFrame>,
    /// Previous two bits for parity computation (D29* and D30*)
    _d29_star: bool,
    _d30_star: bool,
}

impl LnavMessage {
    /// Create a new LNAV message decoder
    pub fn new() -> Self {
        Self {
            current_bits: Vec::new(),
            subframes: Vec::new(),
            _d29_star: false,
            _d30_star: false,
        }
    }

    /// Add a navigation bit (+1 or -1) to the message
    /// Returns Some(NavFrame) when a complete subframe is assembled
    pub fn add_bit(&mut self, bit: i8) -> Option<NavFrame> {
        self.current_bits.push(bit);

        if self.current_bits.len() >= 300 {
            let frame = self.try_decode_subframe();
            self.current_bits.clear();
            return frame;
        }
        None
    }

    /// Try to decode a complete 300-bit subframe
    fn try_decode_subframe(&self) -> Option<NavFrame> {
        if self.current_bits.len() < 300 {
            return None;
        }

        // Convert +1/-1 to 0/1 bits
        let bits: Vec<u8> = self.current_bits.iter()
            .map(|&b| if b > 0 { 0 } else { 1 })
            .collect();

        // Extract 10 words of 30 bits each
        let mut words = [0u32; 10];
        for w in 0..10 {
            let mut word = 0u32;
            for b in 0..30 {
                word = (word << 1) | bits[w * 30 + b] as u32;
            }
            words[w] = word;
        }

        // Check TLM preamble (first 8 bits of word 1)
        let preamble = (words[0] >> 22) & 0xFF;
        if preamble != 0x8B && preamble != 0x74 {
            // Preamble is 10001011 or inverted (01110100)
            return None;
        }

        // Extract subframe ID from HOW (word 2, bits 20-22)
        let subframe_id = ((words[1] >> 8) & 0x07) as u8;
        if subframe_id < 1 || subframe_id > 5 {
            return None;
        }

        // Extract TOW from HOW (word 2, bits 1-17, truncated TOW count)
        let tow = ((words[1] >> 13) & 0x1FFFF) * 6; // TOW in seconds

        // Verify parity on all words
        let parity_ok = self.check_parity_all(&words);

        Some(NavFrame {
            subframe_id,
            tow,
            words,
            parity_ok,
        })
    }

    /// Check parity on all 10 words
    fn check_parity_all(&self, words: &[u32; 10]) -> bool {
        let mut d29 = false;
        let mut d30 = false;

        for &word in words.iter() {
            if !Self::check_word_parity(word, d29, d30) {
                return false;
            }
            d29 = (word >> 1) & 1 == 1;
            d30 = word & 1 == 1;
        }
        true
    }

    /// Check parity of a single 30-bit word
    /// GPS parity uses (24,30) Hamming code with bits D29* and D30* from previous word
    fn check_word_parity(word: u32, d29_star: bool, d30_star: bool) -> bool {
        // Data bits are XORed with D30* if D30* is set
        let data = if d30_star {
            (word >> 6) ^ 0xFFFFFF
        } else {
            word >> 6
        };

        // GPS parity equations (IS-GPS-200 Table 20-XIV)
        let parity_bits = word & 0x3F;

        let d25 = Self::parity_check(data, 0xBB1F34, d29_star);
        let d26 = Self::parity_check(data, 0x5D8F9A, d30_star);
        let d27 = Self::parity_check(data, 0xAEC7CD, d29_star);
        let d28 = Self::parity_check(data, 0x5763E6, d30_star);
        let d29 = Self::parity_check(data, 0x6BB1F3, d30_star);
        let d30 = Self::parity_check(data, 0x8B7A89, d29_star);

        let computed = (d25 as u32) << 5 | (d26 as u32) << 4 | (d27 as u32) << 3
            | (d28 as u32) << 2 | (d29 as u32) << 1 | d30 as u32;

        computed == parity_bits
    }

    /// Compute one parity bit
    fn parity_check(data: u32, mask: u32, d_star: bool) -> bool {
        let count = (data & mask).count_ones();
        let parity = (count & 1) == 1;
        parity ^ d_star
    }

    /// Encode a subframe into 300 bits (for signal generation)
    ///
    /// # Arguments
    /// * `subframe_id` - Subframe number (1-5)
    /// * `tow` - Time of week in seconds
    pub fn encode_subframe(subframe_id: u8, tow: u32) -> Vec<i8> {
        let mut words = [0u32; 10];

        // Word 1: TLM
        let preamble = 0x8Bu32;
        words[0] = preamble << 22; // TLM preamble + reserved + integrity

        // Word 2: HOW (Handover Word)
        let tow_count = (tow / 6) & 0x1FFFF;
        words[1] = (tow_count << 13) | ((subframe_id as u32 & 0x07) << 8);

        // Words 3-10: Fill with subframe-specific data
        match subframe_id {
            1 => {
                // GPS week, SV accuracy, health, clock parameters
                words[2] = 0x0800_0000; // WN placeholder, URA, health
            }
            2 | 3 => {
                // Ephemeris parameters (placeholder)
                for w in &mut words[2..10] {
                    *w = 0;
                }
            }
            4 | 5 => {
                // Almanac data (placeholder)
                for w in &mut words[2..10] {
                    *w = 0;
                }
            }
            _ => {}
        }

        // Add parity bits to all words
        let mut d29_star = false;
        let mut d30_star = false;
        for word in &mut words {
            *word = Self::add_parity(*word, d29_star, d30_star);
            d29_star = (*word >> 1) & 1 == 1;
            d30_star = *word & 1 == 1;
        }

        // Convert to +1/-1 bits
        let mut bits = Vec::with_capacity(300);
        for &word in &words {
            for b in (0..30).rev() {
                bits.push(if (word >> b) & 1 == 0 { 1 } else { -1 });
            }
        }
        bits
    }

    /// Add parity bits to a 30-bit word (data in bits 30-7, parity in bits 6-1)
    fn add_parity(word: u32, d29_star: bool, d30_star: bool) -> u32 {
        let data = word >> 6;
        let d25 = Self::parity_check(data, 0xBB1F34, d29_star);
        let d26 = Self::parity_check(data, 0x5D8F9A, d30_star);
        let d27 = Self::parity_check(data, 0xAEC7CD, d29_star);
        let d28 = Self::parity_check(data, 0x5763E6, d30_star);
        let d29 = Self::parity_check(data, 0x6BB1F3, d30_star);
        let d30 = Self::parity_check(data, 0x8B7A89, d29_star);

        let parity = (d25 as u32) << 5 | (d26 as u32) << 4 | (d27 as u32) << 3
            | (d28 as u32) << 2 | (d29 as u32) << 1 | d30 as u32;

        (word & 0xFFFFFFC0) | parity
    }

    /// Decode subframe 1 clock parameters
    pub fn decode_subframe_clock(frame: &NavFrame) -> Option<SubframeClock> {
        if frame.subframe_id != 1 {
            return None;
        }

        // Word 3: WN (bits 1-10), SV accuracy (bits 13-16), SV health (bits 17-22)
        let week_number = ((frame.words[2] >> 20) & 0x3FF) as u16;
        let sv_accuracy = ((frame.words[2] >> 14) & 0x0F) as u8;
        let sv_health = ((frame.words[2] >> 8) & 0x3F) as u8;

        // Word 7: Tgd (bits 17-24)
        let tgd_raw = ((frame.words[6] >> 6) & 0xFF) as i8;
        let tgd = tgd_raw as f64 * 2.0_f64.powi(-31);

        // Word 9: af2 (bits 1-8), af1 (bits 9-24)
        let af2_raw = ((frame.words[8] >> 22) & 0xFF) as i8;
        let af1_raw = ((frame.words[8] >> 6) & 0xFFFF) as i16;
        let af2 = af2_raw as f64 * 2.0_f64.powi(-55);
        let af1 = af1_raw as f64 * 2.0_f64.powi(-43);

        // Word 10: af0 (bits 1-22)
        let af0_raw = ((frame.words[9] >> 8) & 0x3FFFFF) as i32;
        let af0 = af0_raw as f64 * 2.0_f64.powi(-31);

        Some(SubframeClock {
            week_number,
            sv_accuracy,
            sv_health,
            af0,
            af1,
            af2,
            tgd,
        })
    }

    /// Get completed subframes
    pub fn subframes(&self) -> &[NavFrame] {
        &self.subframes
    }
}

impl Default for LnavMessage {
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_encode_subframe() {
        let bits = LnavMessage::encode_subframe(1, 0);
        assert_eq!(bits.len(), 300);
        // All bits should be +1 or -1
        assert!(bits.iter().all(|&b| b == 1 || b == -1));
    }

    #[test]
    fn test_encode_all_subframes() {
        for sf in 1..=5 {
            let bits = LnavMessage::encode_subframe(sf, 100 * sf as u32);
            assert_eq!(bits.len(), 300, "Subframe {} should be 300 bits", sf);
        }
    }

    #[test]
    fn test_nav_message_bit_accumulation() {
        let mut nav = LnavMessage::new();
        // Feed in bits one at a time
        for i in 0..299 {
            let result = nav.add_bit(if i % 2 == 0 { 1 } else { -1 });
            assert!(result.is_none(), "Should not produce frame before 300 bits");
        }
        // 300th bit should trigger decode attempt
        let _result = nav.add_bit(1);
        // May or may not decode depending on preamble match
    }
}
