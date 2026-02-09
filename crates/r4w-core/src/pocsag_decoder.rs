//! POCSAG Decoder — Pager protocol decoder
//!
//! Decodes POCSAG (Post Office Code Standardization Advisory Group)
//! paging messages from FSK-demodulated bitstreams. Extracts numeric,
//! alphanumeric, and tone-only pages. Uses BCH(31,21) error correction.
//! Operates at 512/1200/2400 baud on VHF frequencies.
//! GNU Radio equivalent: `gr-pager` / `multimon-ng`.
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::pocsag_decoder::{PocsagDecoder, PocsagMessage};
//!
//! let mut decoder = PocsagDecoder::new();
//! // In practice, feed FSK-demodulated bits
//! let messages = decoder.decode_batch(&[0x7CD215D8, 0x7A89C197, 0x7AAAAAAA]);
//! // Messages would be extracted from valid codewords
//! ```

/// POCSAG sync word: 0x7CD215D8.
pub const POCSAG_SYNC: u32 = 0x7CD215D8;

/// POCSAG idle codeword: 0x7A89C197.
pub const POCSAG_IDLE: u32 = 0x7A89C197;

/// POCSAG message content.
#[derive(Debug, Clone, PartialEq)]
pub enum PocsagContent {
    /// Numeric message (digits 0-9, space, hyphen, brackets, etc.).
    Numeric(String),
    /// Alphanumeric message (7-bit ASCII).
    Alphanumeric(String),
    /// Tone-only alert (no message content).
    ToneOnly,
}

/// Decoded POCSAG message.
#[derive(Debug, Clone)]
pub struct PocsagMessage {
    /// 21-bit address (cap code).
    pub address: u32,
    /// 2-bit function code (0-3).
    pub function: u8,
    /// Message content.
    pub content: PocsagContent,
    /// Batch number where this message was found.
    pub batch_number: usize,
}

/// POCSAG decoder error.
#[derive(Debug)]
pub enum PocsagError {
    /// BCH correction failed.
    BchError,
    /// Invalid codeword.
    InvalidCodeword,
    /// Sync not found.
    NoSync,
}

impl std::fmt::Display for PocsagError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            PocsagError::BchError => write!(f, "BCH correction failed"),
            PocsagError::InvalidCodeword => write!(f, "Invalid codeword"),
            PocsagError::NoSync => write!(f, "Sync word not found"),
        }
    }
}

impl std::error::Error for PocsagError {}

/// POCSAG decoder.
#[derive(Debug, Clone)]
pub struct PocsagDecoder {
    /// Accumulated bits.
    bit_buffer: Vec<bool>,
    /// Messages decoded so far.
    messages: Vec<PocsagMessage>,
    /// Current batch number.
    batch_count: usize,
}

impl PocsagDecoder {
    /// Create a new POCSAG decoder.
    pub fn new() -> Self {
        Self {
            bit_buffer: Vec::new(),
            messages: Vec::new(),
            batch_count: 0,
        }
    }

    /// Feed raw bits from FSK demodulator.
    pub fn feed_bits(&mut self, bits: &[bool]) -> Vec<PocsagMessage> {
        self.bit_buffer.extend_from_slice(bits);
        let mut new_messages = Vec::new();

        // Look for sync word
        while self.bit_buffer.len() >= 32 {
            if let Some(sync_pos) = self.find_sync() {
                // Remove bits before sync
                if sync_pos > 0 {
                    self.bit_buffer.drain(..sync_pos);
                }

                // Need sync (32) + 16 codewords (16 * 32 = 512) = 544 bits for a batch
                if self.bit_buffer.len() < 544 {
                    break;
                }

                // Skip sync word
                let batch_bits: Vec<bool> = self.bit_buffer.drain(..544).collect();

                // Process 8 frames (16 codewords)
                for frame in 0..8 {
                    let cw_offset = 32 + frame * 64; // Skip sync, each frame is 2 codewords
                    let cw1 = bits_to_u32(&batch_bits[cw_offset..cw_offset + 32]);
                    let cw2 = bits_to_u32(&batch_bits[cw_offset + 32..cw_offset + 64]);

                    for (idx, cw) in [cw1, cw2].iter().enumerate() {
                        if *cw == POCSAG_IDLE {
                            continue;
                        }
                        if let Ok(corrected) = bch_correct(*cw) {
                            if corrected & 0x80000000 == 0 {
                                // Address codeword (bit 31 = 0)
                                let addr = ((corrected >> 10) & 0x1FFFFF) << 3
                                    | (frame * 2 + idx) as u32 & 0x7;
                                let func = ((corrected >> 11) & 0x3) as u8;

                                new_messages.push(PocsagMessage {
                                    address: addr,
                                    function: func,
                                    content: PocsagContent::ToneOnly,
                                    batch_number: self.batch_count,
                                });
                            }
                            // Message codeword (bit 31 = 1) — data follows address
                        }
                    }
                }

                self.batch_count += 1;
            } else {
                // No sync found, drop oldest bit
                self.bit_buffer.drain(..1);
            }
        }

        self.messages.extend(new_messages.clone());
        new_messages
    }

    /// Decode a batch of pre-extracted 32-bit codewords.
    ///
    /// The first codeword should be the sync word.
    pub fn decode_batch(&mut self, codewords: &[u32]) -> Vec<PocsagMessage> {
        let mut messages = Vec::new();

        let start = if !codewords.is_empty() && codewords[0] == POCSAG_SYNC {
            1 // Skip sync
        } else {
            0
        };

        let mut pending_address: Option<(u32, u8, usize)> = None;
        let mut message_bits: Vec<bool> = Vec::new();

        for (idx, &cw) in codewords[start..].iter().enumerate() {
            if cw == POCSAG_IDLE {
                // Flush pending message
                if let Some((addr, func, batch)) = pending_address.take() {
                    let content = decode_message_content(func, &message_bits);
                    messages.push(PocsagMessage {
                        address: addr,
                        function: func,
                        content,
                        batch_number: batch,
                    });
                    message_bits.clear();
                }
                continue;
            }

            if let Ok(corrected) = bch_correct(cw) {
                if corrected & 0x80000000 == 0 {
                    // Address codeword
                    if let Some((addr, func, batch)) = pending_address.take() {
                        let content = decode_message_content(func, &message_bits);
                        messages.push(PocsagMessage {
                            address: addr,
                            function: func,
                            content,
                            batch_number: batch,
                        });
                        message_bits.clear();
                    }

                    let frame = idx / 2;
                    let addr = ((corrected >> 10) & 0x1FFFFF) << 3
                        | (frame * 2 + idx % 2) as u32 & 0x7;
                    let func = ((corrected >> 11) & 0x3) as u8;
                    pending_address = Some((addr, func, self.batch_count));
                } else {
                    // Message codeword — extract 20 data bits
                    for bit in (1..21).rev() {
                        message_bits.push((corrected >> (bit + 10)) & 1 == 1);
                    }
                }
            }
        }

        // Flush last pending
        if let Some((addr, func, batch)) = pending_address {
            let content = decode_message_content(func, &message_bits);
            messages.push(PocsagMessage {
                address: addr,
                function: func,
                content,
                batch_number: batch,
            });
        }

        self.batch_count += 1;
        self.messages.extend(messages.clone());
        messages
    }

    /// Get all decoded messages.
    pub fn messages(&self) -> &[PocsagMessage] {
        &self.messages
    }

    /// Total batches processed.
    pub fn batch_count(&self) -> usize {
        self.batch_count
    }

    /// Reset the decoder.
    pub fn reset(&mut self) {
        self.bit_buffer.clear();
        self.messages.clear();
        self.batch_count = 0;
    }

    /// Find sync word position in bit buffer.
    fn find_sync(&self) -> Option<usize> {
        if self.bit_buffer.len() < 32 {
            return None;
        }
        for i in 0..=(self.bit_buffer.len() - 32) {
            let word = bits_to_u32(&self.bit_buffer[i..i + 32]);
            if word == POCSAG_SYNC {
                return Some(i);
            }
        }
        None
    }
}

impl Default for PocsagDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// BCH(31,21) error correction for POCSAG codeword.
///
/// Corrects up to 2 bit errors in the 31-bit codeword.
/// Returns the corrected 32-bit value (including even parity bit).
pub fn bch_correct(codeword: u32) -> Result<u32, PocsagError> {
    // Check even parity (bit 0)
    let _parity = codeword.count_ones() % 2;

    // For simplicity, check if it's a valid codeword by syndrome check
    let syndrome = compute_bch_syndrome(codeword);
    if syndrome == 0 {
        return Ok(codeword);
    }

    // Try single-bit correction
    for i in 1..32 {
        let trial = codeword ^ (1 << i);
        if compute_bch_syndrome(trial) == 0 {
            return Ok(trial);
        }
    }

    // Try double-bit correction
    for i in 1..32 {
        for j in (i + 1)..32 {
            let trial = codeword ^ (1 << i) ^ (1 << j);
            if compute_bch_syndrome(trial) == 0 {
                return Ok(trial);
            }
        }
    }

    Err(PocsagError::BchError)
}

/// Compute BCH(31,21) syndrome.
fn compute_bch_syndrome(codeword: u32) -> u32 {
    // Generator polynomial for BCH(31,21,2): x^10 + x^9 + x^8 + x^6 + x^5 + x^3 + 1
    let generator: u32 = 0b11101101001; // 0x769
    let mut data = codeword >> 1; // Remove parity bit

    // Polynomial division
    for i in (10..31).rev() {
        if data & (1 << i) != 0 {
            data ^= generator << (i - 10);
        }
    }

    data & 0x3FF // 10-bit syndrome
}

/// Convert bool slice to u32 (MSB first).
fn bits_to_u32(bits: &[bool]) -> u32 {
    let mut val: u32 = 0;
    for (i, &bit) in bits.iter().enumerate().take(32) {
        if bit {
            val |= 1 << (31 - i);
        }
    }
    val
}

/// Decode POCSAG numeric characters from bits.
fn decode_numeric(bits: &[bool]) -> String {
    let numeric_table = [
        '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', '*', 'U', ' ', '-', ')', '(',
    ];
    let mut result = String::new();
    for chunk in bits.chunks(4) {
        let val = chunk
            .iter()
            .enumerate()
            .fold(0usize, |acc, (i, &b)| acc | if b { 1 << (3 - i) } else { 0 });
        if val < numeric_table.len() {
            result.push(numeric_table[val]);
        }
    }
    result
}

/// Decode POCSAG alphanumeric characters from bits (7-bit ASCII).
fn decode_alpha(bits: &[bool]) -> String {
    let mut result = String::new();
    for chunk in bits.chunks(7) {
        if chunk.len() < 7 {
            break;
        }
        let val: u8 = chunk
            .iter()
            .enumerate()
            .fold(0u8, |acc, (i, &b)| acc | if b { 1 << i } else { 0 }); // LSB first
        if val >= 32 && val < 127 {
            result.push(val as char);
        } else if val == 0 {
            break; // Null terminator
        }
    }
    result
}

/// Decode message content based on function code.
fn decode_message_content(function: u8, bits: &[bool]) -> PocsagContent {
    if bits.is_empty() {
        return PocsagContent::ToneOnly;
    }
    match function {
        0 => PocsagContent::Numeric(decode_numeric(bits)),
        3 => PocsagContent::Alphanumeric(decode_alpha(bits)),
        _ => {
            // Try alpha first, fall back to numeric
            let alpha = decode_alpha(bits);
            if alpha.is_empty() {
                PocsagContent::Numeric(decode_numeric(bits))
            } else {
                PocsagContent::Alphanumeric(alpha)
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_sync_word() {
        assert_eq!(POCSAG_SYNC, 0x7CD215D8);
    }

    #[test]
    fn test_idle_word() {
        assert_eq!(POCSAG_IDLE, 0x7A89C197);
    }

    #[test]
    fn test_bch_syndrome_idle() {
        // Idle word should have zero syndrome
        let syn = compute_bch_syndrome(POCSAG_IDLE);
        assert_eq!(syn, 0, "Idle codeword should have zero syndrome");
    }

    #[test]
    fn test_bch_correct_valid() {
        let result = bch_correct(POCSAG_IDLE);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), POCSAG_IDLE);
    }

    #[test]
    fn test_bch_correct_single_error() {
        let corrupted = POCSAG_IDLE ^ (1 << 15); // Flip bit 15
        let result = bch_correct(corrupted);
        assert!(result.is_ok());
        assert_eq!(result.unwrap(), POCSAG_IDLE);
    }

    #[test]
    fn test_decode_numeric() {
        // "123" = 0001 0010 0011
        let bits = vec![
            false, false, false, true, // 1
            false, false, true, false, // 2
            false, false, true, true,  // 3
        ];
        let result = decode_numeric(&bits);
        assert_eq!(result, "123");
    }

    #[test]
    fn test_decode_alpha() {
        // 'A' = 0x41 = 1000001 (LSB first: 1000010)
        let bits = vec![true, false, false, false, false, false, true]; // A in LSB-first
        let result = decode_alpha(&bits);
        assert_eq!(result, "A");
    }

    #[test]
    fn test_decoder_creation() {
        let decoder = PocsagDecoder::new();
        assert_eq!(decoder.batch_count(), 0);
        assert!(decoder.messages().is_empty());
    }

    #[test]
    fn test_decode_batch_idle_only() {
        let mut decoder = PocsagDecoder::new();
        let codewords = vec![POCSAG_SYNC, POCSAG_IDLE, POCSAG_IDLE];
        let messages = decoder.decode_batch(&codewords);
        assert!(messages.is_empty()); // No actual messages
    }

    #[test]
    fn test_bits_to_u32() {
        let bits = vec![true; 32]; // All ones
        assert_eq!(bits_to_u32(&bits), 0xFFFFFFFF);

        let bits = vec![false; 32]; // All zeros
        assert_eq!(bits_to_u32(&bits), 0x00000000);
    }

    #[test]
    fn test_message_content_toneonly() {
        let content = decode_message_content(0, &[]);
        assert_eq!(content, PocsagContent::ToneOnly);
    }

    #[test]
    fn test_reset() {
        let mut decoder = PocsagDecoder::new();
        decoder.batch_count = 5;
        decoder.reset();
        assert_eq!(decoder.batch_count(), 0);
    }

    #[test]
    fn test_default() {
        let decoder = PocsagDecoder::default();
        assert_eq!(decoder.batch_count(), 0);
    }
}
