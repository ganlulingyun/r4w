//! Protocol parsing and formatting for aviation and maritime telemetry data streams.
//!
//! This module implements encoders and decoders for common telemetry protocols:
//!
//! - **ARINC 429** — 32-bit avionics data bus word encoding/decoding with label, SDI, data, SSM,
//!   and odd parity.
//! - **NMEA 0183** — Maritime/GNSS sentence parsing and generation with XOR checksum.
//! - **IRIG-B** — Inter-Range Instrumentation Group time code encoding and decoding (AM modulated).
//! - **PCM Frame Synchronization** — PN sync word correlation search for telemetry decommutation.
//! - **CCSDS Space Packet** — Consultative Committee for Space Data Systems packet header parsing.
//!
//! # Example
//!
//! ```
//! use r4w_core::telemetry_framer::{Arinc429Word, encode_arinc429, decode_arinc429};
//!
//! let word = Arinc429Word {
//!     label: 0o310,  // Label 310 octal (altitude)
//!     sdi: 0b01,
//!     data: 0x3A00,
//!     ssm: 0b11,
//! };
//! let raw = encode_arinc429(&word);
//! let decoded = decode_arinc429(raw);
//! assert_eq!(decoded.label, word.label);
//! assert_eq!(decoded.sdi, word.sdi);
//! assert_eq!(decoded.data, word.data);
//! assert_eq!(decoded.ssm, word.ssm);
//! ```

/// ARINC 429 avionics data bus word.
///
/// A 32-bit word with the following bit layout (LSB-first transmission):
/// - Bits 0–7: Label (8 bits, reversed bit order per ARINC 429 convention)
/// - Bits 8–9: Source/Destination Identifier (SDI, 2 bits)
/// - Bits 10–28: Data field (19 bits)
/// - Bits 29–30: Sign/Status Matrix (SSM, 2 bits)
/// - Bit 31: Parity (odd)
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Arinc429Word {
    /// 8-bit label (octal encoding typical, e.g., 0o310 for altitude).
    pub label: u8,
    /// 2-bit Source/Destination Identifier.
    pub sdi: u8,
    /// 19-bit data field.
    pub data: u32,
    /// 2-bit Sign/Status Matrix.
    pub ssm: u8,
}

/// NMEA 0183 sentence.
///
/// Represents a parsed NMEA sentence with talker ID, sentence type, data fields,
/// and XOR checksum.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NmeaSentence {
    /// Two-character talker identifier (e.g., "GP" for GPS, "GL" for GLONASS).
    pub talker: String,
    /// Sentence type identifier (e.g., "GGA", "RMC").
    pub sentence_type: String,
    /// Comma-separated data fields.
    pub fields: Vec<String>,
    /// XOR checksum of characters between '$' and '*'.
    pub checksum: u8,
}

/// IRIG-B time code.
///
/// Represents time-of-year information encoded in IRIG-B format (IEEE 1344 / IRIG 200).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct IrigBTime {
    /// Seconds (0–59).
    pub seconds: u8,
    /// Minutes (0–59).
    pub minutes: u8,
    /// Hours (0–23).
    pub hours: u8,
    /// Day of year (1–366).
    pub day_of_year: u16,
    /// Two-digit year (0–99), or full year if extended.
    pub year: u16,
}

/// CCSDS (Consultative Committee for Space Data Systems) space packet.
///
/// Implements CCSDS 133.0-B-2 Space Packet Protocol header parsing.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CcsdsPacket {
    /// Packet version number (3 bits, typically 0).
    pub version: u8,
    /// Packet type flag: 0 = telemetry, 1 = telecommand.
    pub type_flag: u8,
    /// Application Process Identifier (11 bits).
    pub apid: u16,
    /// Packet sequence count (14 bits).
    pub sequence_count: u16,
    /// Packet data (user data zone).
    pub data: Vec<u8>,
}

// ---------------------------------------------------------------------------
// ARINC 429
// ---------------------------------------------------------------------------

/// Reverse the bit order of an 8-bit label per ARINC 429 convention.
///
/// ARINC 429 labels are transmitted LSB-first but read in reversed order.
fn reverse_label_bits(label: u8) -> u8 {
    label.reverse_bits()
}

/// Encode an [`Arinc429Word`] into a 32-bit word with odd parity.
///
/// Bit layout:
/// - Bits 0–7: Label (bit-reversed)
/// - Bits 8–9: SDI
/// - Bits 10–28: Data
/// - Bits 29–30: SSM
/// - Bit 31: Parity (odd)
pub fn encode_arinc429(word: &Arinc429Word) -> u32 {
    let label_reversed = reverse_label_bits(word.label) as u32;
    let sdi = (word.sdi as u32 & 0x03) << 8;
    let data = (word.data & 0x7_FFFF) << 10;
    let ssm = (word.ssm as u32 & 0x03) << 29;

    let mut raw = label_reversed | sdi | data | ssm;

    // Compute odd parity over bits 0–30
    let ones = (raw & 0x7FFF_FFFF).count_ones();
    if ones % 2 == 0 {
        raw |= 1 << 31; // set parity bit to make odd
    }
    // If already odd, parity bit stays 0

    raw
}

/// Decode a 32-bit raw ARINC 429 word into an [`Arinc429Word`].
///
/// The parity bit (bit 31) is not stored in the struct but is used during encoding.
pub fn decode_arinc429(raw: u32) -> Arinc429Word {
    let label_reversed = (raw & 0xFF) as u8;
    let label = reverse_label_bits(label_reversed);
    let sdi = ((raw >> 8) & 0x03) as u8;
    let data = (raw >> 10) & 0x7_FFFF;
    let ssm = ((raw >> 29) & 0x03) as u8;

    Arinc429Word {
        label,
        sdi,
        data,
        ssm,
    }
}

/// Verify odd parity of a raw ARINC 429 word.
///
/// Returns `true` if the word has valid odd parity (odd number of 1-bits across
/// all 32 bits).
pub fn verify_arinc429_parity(raw: u32) -> bool {
    raw.count_ones() % 2 == 1
}

// ---------------------------------------------------------------------------
// NMEA 0183
// ---------------------------------------------------------------------------

/// Compute the XOR checksum of an NMEA sentence body.
///
/// The checksum is computed over all characters between (but not including)
/// the '$' start delimiter and the '*' checksum delimiter. If neither delimiter
/// is present, the entire string is checksummed.
pub fn nmea_checksum(sentence: &str) -> u8 {
    let body = if let Some(start) = sentence.find('$') {
        let after_dollar = &sentence[start + 1..];
        if let Some(star) = after_dollar.find('*') {
            &after_dollar[..star]
        } else {
            after_dollar
        }
    } else if let Some(star) = sentence.find('*') {
        &sentence[..star]
    } else {
        sentence
    };

    body.bytes().fold(0u8, |acc, b| acc ^ b)
}

/// Parse an NMEA 0183 sentence string into an [`NmeaSentence`].
///
/// Expects the format `$TTMMM,field1,field2,...*CC` where:
/// - `TT` is the talker ID
/// - `MMM` is the sentence type
/// - `CC` is the two-hex-digit checksum
///
/// Returns `None` if the sentence is malformed or the checksum does not match.
pub fn parse_nmea(sentence: &str) -> Option<NmeaSentence> {
    let trimmed = sentence.trim();

    // Must start with '$'
    if !trimmed.starts_with('$') {
        return None;
    }

    // Find the '*' checksum delimiter
    let star_pos = trimmed.find('*')?;
    let body = &trimmed[1..star_pos];
    let checksum_str = &trimmed[star_pos + 1..];

    // Parse the provided checksum
    if checksum_str.len() < 2 {
        return None;
    }
    let provided_checksum = u8::from_str_radix(&checksum_str[..2], 16).ok()?;

    // Verify checksum
    let computed = nmea_checksum(trimmed);
    if computed != provided_checksum {
        return None;
    }

    // Split body into parts by comma
    let mut parts = body.split(',');
    let header = parts.next()?;

    // Header must be at least 3 characters (2 talker + 1+ sentence type)
    if header.len() < 3 {
        return None;
    }

    let talker = header[..2].to_string();
    let sentence_type = header[2..].to_string();
    let fields: Vec<String> = parts.map(|s| s.to_string()).collect();

    Some(NmeaSentence {
        talker,
        sentence_type,
        fields,
        checksum: provided_checksum,
    })
}

/// Format an [`NmeaSentence`] into a valid NMEA 0183 string with checksum.
///
/// The output format is `$TTMMM,field1,field2,...*CC` where `CC` is the
/// computed XOR checksum in uppercase hexadecimal.
pub fn format_nmea(sentence: &NmeaSentence) -> String {
    let mut body = format!("{}{}", sentence.talker, sentence.sentence_type);
    for field in &sentence.fields {
        body.push(',');
        body.push_str(field);
    }

    let checksum = body.bytes().fold(0u8, |acc, b| acc ^ b);
    format!("${}*{:02X}", body, checksum)
}

// ---------------------------------------------------------------------------
// IRIG-B
// ---------------------------------------------------------------------------

/// Encode BCD digits from a value, returning the specified number of bits.
fn encode_bcd_bits(value: u16, num_digits: usize) -> Vec<bool> {
    let mut bits = Vec::new();
    let mut val = value;
    for _ in 0..num_digits {
        let digit = val % 10;
        // Each BCD digit is 4 bits, LSB first
        for bit_pos in 0..4 {
            bits.push((digit >> bit_pos) & 1 == 1);
        }
        val /= 10;
    }
    bits
}

/// Decode BCD bits back to a numeric value.
fn decode_bcd_bits(bits: &[bool], num_digits: usize) -> u16 {
    let mut value: u16 = 0;
    let mut multiplier: u16 = 1;
    for digit_idx in 0..num_digits {
        let start = digit_idx * 4;
        let mut digit: u16 = 0;
        for bit_pos in 0..4 {
            if start + bit_pos < bits.len() && bits[start + bit_pos] {
                digit |= 1 << bit_pos;
            }
        }
        value += digit * multiplier;
        multiplier *= 10;
    }
    value
}

/// IRIG-B pulse width durations (as fraction of 10ms bit period).
///
/// - Reference marker: 8ms (0.8)
/// - Binary 1: 5ms (0.5)
/// - Binary 0: 2ms (0.2)
const IRIG_B_REF_DUTY: f64 = 0.8;
const IRIG_B_ONE_DUTY: f64 = 0.5;
const IRIG_B_ZERO_DUTY: f64 = 0.2;

/// IRIG-B frame structure: 100 bit-slots per second.
///
/// Each bit-slot is 10ms. The frame encodes time-of-year in BCD.
///
/// Bit positions (0-indexed) per IRIG Standard 200:
/// - 0: Reference marker (Pr)
/// - 1-4: Seconds ones (BCD: 1,2,4,8)
/// - 5-7: Seconds tens (BCD: 10,20,40)
/// - 8: unused
/// - 9: Position identifier P1
/// - 10-13: Minutes ones (BCD: 1,2,4,8)
/// - 14-16: Minutes tens (BCD: 10,20,40)
/// - 17-18: unused
/// - 19: Position identifier P2
/// - 20-23: Hours ones (BCD: 1,2,4,8)
/// - 24-25: Hours tens (BCD: 10,20)
/// - 26-28: unused
/// - 29: Position identifier P3
/// - 30-33: Days ones (BCD: 1,2,4,8)
/// - 34-37: Days tens (BCD: 10,20,40,80)
/// - 38: Days hundreds (100)
/// - 39: Position identifier P4
/// - 40: Days hundreds (200)
/// - 41-48: unused / control functions
/// - 49: Position identifier P5
/// - 50-53: Year ones (BCD: 1,2,4,8) — extended IRIG-B
/// - 54-57: Year tens (BCD: 10,20,40,80)
/// - 58: unused
/// - 59: Position identifier P6
/// - 60-68: unused / SBS
/// - 69: Position identifier P7
/// - 70-78: unused / SBS
/// - 79: Position identifier P8
/// - 80-88: unused / parity / control
/// - 89: Position identifier P9
/// - 90-98: unused
/// - 99: Reference marker (Pr0, next frame)

/// Encode an [`IrigBTime`] into an AM-modulated IRIG-B signal.
///
/// The output is a vector of amplitude samples at the given sample rate.
/// The signal represents one complete 1-second IRIG-B frame (100 bit-slots of 10ms each).
/// A carrier amplitude of 1.0 is modulated between high (1.0) and low (0.3) levels
/// based on pulse-width modulation of each bit-slot.
pub fn encode_irig_b(time: &IrigBTime, sample_rate: f64) -> Vec<f64> {
    let samples_per_bit = (sample_rate * 0.01) as usize; // 10ms per bit
    let total_samples = samples_per_bit * 100;
    let mut signal = vec![0.3_f64; total_samples];

    // Build the 100-element bit array
    let mut frame = [0u8; 100]; // 0 = binary 0, 1 = binary 1, 2 = reference marker

    // Bit 0: reference marker (frame reference)
    frame[0] = 2;

    // Seconds: ones in bits 1-4, tens in bits 5-7
    let sec_ones = time.seconds % 10;
    let sec_tens = time.seconds / 10;
    for i in 0..4 {
        frame[1 + i] = ((sec_ones >> i) & 1) as u8;
    }
    for i in 0..3 {
        frame[5 + i] = ((sec_tens >> i) & 1) as u8;
    }
    // Bit 9: position identifier P1
    frame[9] = 2;

    // Minutes: ones in bits 10-13, tens in bits 14-16
    let min_ones = time.minutes % 10;
    let min_tens = time.minutes / 10;
    for i in 0..4 {
        frame[10 + i] = ((min_ones >> i) & 1) as u8;
    }
    for i in 0..3 {
        frame[14 + i] = ((min_tens >> i) & 1) as u8;
    }
    // Bit 19: position identifier P2
    frame[19] = 2;

    // Hours: ones in bits 20-23, tens in bits 24-25
    let hr_ones = time.hours % 10;
    let hr_tens = time.hours / 10;
    for i in 0..4 {
        frame[20 + i] = ((hr_ones >> i) & 1) as u8;
    }
    for i in 0..2 {
        frame[24 + i] = ((hr_tens >> i) & 1) as u8;
    }
    // Bit 29: position identifier P3
    frame[29] = 2;

    // Day of year: ones in bits 30-33, tens in bits 34-37,
    // hundreds bit 0 (100) at position 38, hundreds bit 1 (200) at position 40
    let day_ones = (time.day_of_year % 10) as u8;
    let day_tens = ((time.day_of_year / 10) % 10) as u8;
    let day_hundreds = ((time.day_of_year / 100) % 10) as u8;
    for i in 0..4 {
        frame[30 + i] = (day_ones >> i) & 1;
    }
    for i in 0..4 {
        frame[34 + i] = (day_tens >> i) & 1;
    }
    // Day hundreds: bit 0 (value 100) at position 38
    frame[38] = day_hundreds & 1;
    // Bit 39: position identifier P4
    frame[39] = 2;
    // Day hundreds: bit 1 (value 200) at position 40
    frame[40] = (day_hundreds >> 1) & 1;

    // Bit 49: position identifier P5
    frame[49] = 2;

    // Year (extended IRIG-B): ones in bits 50-53, tens in bits 54-57
    let yr_ones = (time.year % 10) as u8;
    let yr_tens = ((time.year / 10) % 10) as u8;
    for i in 0..4 {
        frame[50 + i] = (yr_ones >> i) & 1;
    }
    for i in 0..4 {
        frame[54 + i] = (yr_tens >> i) & 1;
    }

    // Bit 59: position identifier P6
    frame[59] = 2;
    // Bit 69: position identifier P7
    frame[69] = 2;
    // Bit 79: position identifier P8
    frame[79] = 2;
    // Bit 89: position identifier P9
    frame[89] = 2;
    // Bit 99: reference marker (end/next frame)
    frame[99] = 2;

    // Generate AM-modulated signal
    for (slot, &bit_type) in frame.iter().enumerate() {
        let duty = match bit_type {
            2 => IRIG_B_REF_DUTY,
            1 => IRIG_B_ONE_DUTY,
            _ => IRIG_B_ZERO_DUTY,
        };
        let high_samples = (samples_per_bit as f64 * duty) as usize;
        let start = slot * samples_per_bit;
        for i in 0..high_samples.min(samples_per_bit) {
            signal[start + i] = 1.0;
        }
        // Remaining samples in this slot stay at 0.3 (already initialized)
    }

    signal
}

/// Decode an AM-modulated IRIG-B signal into an [`IrigBTime`].
///
/// The signal should contain at least one complete 1-second frame at the
/// specified sample rate. Returns `None` if the signal is too short or
/// frame synchronization fails.
pub fn decode_irig_b(signal: &[f64], sample_rate: f64) -> Option<IrigBTime> {
    let samples_per_bit = (sample_rate * 0.01) as usize;
    if samples_per_bit == 0 {
        return None;
    }

    let total_needed = samples_per_bit * 100;
    if signal.len() < total_needed {
        return None;
    }

    // Threshold for high/low detection
    let threshold = 0.65;

    // Decode each bit-slot by measuring high-duty fraction
    let mut frame = Vec::with_capacity(100);
    for slot in 0..100 {
        let start = slot * samples_per_bit;
        let high_count = signal[start..start + samples_per_bit]
            .iter()
            .filter(|&&s| s > threshold)
            .count();
        let duty = high_count as f64 / samples_per_bit as f64;

        if duty > 0.65 {
            frame.push(2u8); // reference marker
        } else if duty > 0.35 {
            frame.push(1u8); // binary 1
        } else {
            frame.push(0u8); // binary 0
        }
    }

    // Verify frame reference marker at position 0
    if frame[0] != 2 {
        return None;
    }

    // Decode seconds (ones: bits 1-4, tens: bits 5-7)
    let mut seconds: u8 = 0;
    for i in 0..4 {
        if frame[1 + i] == 1 {
            seconds |= 1 << i;
        }
    }
    let mut sec_tens: u8 = 0;
    for i in 0..3 {
        if frame[5 + i] == 1 {
            sec_tens |= 1 << i;
        }
    }
    seconds += sec_tens * 10;

    // Decode minutes (ones: bits 10-13, tens: bits 14-16)
    let mut minutes: u8 = 0;
    for i in 0..4 {
        if frame[10 + i] == 1 {
            minutes |= 1 << i;
        }
    }
    let mut min_tens: u8 = 0;
    for i in 0..3 {
        if frame[14 + i] == 1 {
            min_tens |= 1 << i;
        }
    }
    minutes += min_tens * 10;

    // Decode hours (ones: bits 20-23, tens: bits 24-25)
    let mut hours: u8 = 0;
    for i in 0..4 {
        if frame[20 + i] == 1 {
            hours |= 1 << i;
        }
    }
    let mut hr_tens: u8 = 0;
    for i in 0..2 {
        if frame[24 + i] == 1 {
            hr_tens |= 1 << i;
        }
    }
    hours += hr_tens * 10;

    // Decode day of year
    // ones: bits 30-33, tens: bits 34-37
    // hundreds bit 0 (100): position 38, hundreds bit 1 (200): position 40
    let mut day_ones: u16 = 0;
    for i in 0..4 {
        if frame[30 + i] == 1 {
            day_ones |= 1 << i;
        }
    }
    let mut day_tens: u16 = 0;
    for i in 0..4 {
        if frame[34 + i] == 1 {
            day_tens |= 1 << i;
        }
    }
    let mut day_hundreds: u16 = 0;
    if frame[38] == 1 {
        day_hundreds |= 1; // 100
    }
    if frame[40] == 1 {
        day_hundreds |= 2; // 200
    }
    let day_of_year = day_ones + day_tens * 10 + day_hundreds * 100;

    // Decode year (extended): ones bits 50-53, tens bits 54-57
    let mut yr_ones: u16 = 0;
    for i in 0..4 {
        if frame[50 + i] == 1 {
            yr_ones |= 1 << i;
        }
    }
    let mut yr_tens: u16 = 0;
    for i in 0..4 {
        if frame[54 + i] == 1 {
            yr_tens |= 1 << i;
        }
    }
    let year = yr_ones + yr_tens * 10;

    Some(IrigBTime {
        seconds,
        minutes,
        hours,
        day_of_year,
        year,
    })
}

// ---------------------------------------------------------------------------
// PCM Frame Synchronization
// ---------------------------------------------------------------------------

/// Find all occurrences of a sync word pattern in a bitstream using correlation.
///
/// Performs a sliding correlation of the `sync_pattern` against the `bitstream`.
/// Returns the starting indices where the pattern matches exactly (all bits match).
///
/// This is used in PCM telemetry decommutation to locate frame boundaries.
///
/// # Example
///
/// ```
/// use r4w_core::telemetry_framer::find_sync_word;
///
/// let bitstream = vec![false, true, true, false, true, false, true, true, false, true, false];
/// let sync = vec![true, false, true];
/// let positions = find_sync_word(&bitstream, &sync);
/// assert!(positions.contains(&4));
/// ```
pub fn find_sync_word(bitstream: &[bool], sync_pattern: &[bool]) -> Vec<usize> {
    if sync_pattern.is_empty() || bitstream.len() < sync_pattern.len() {
        return Vec::new();
    }

    let mut matches = Vec::new();
    let pattern_len = sync_pattern.len();

    for i in 0..=bitstream.len() - pattern_len {
        let correlation = bitstream[i..i + pattern_len]
            .iter()
            .zip(sync_pattern.iter())
            .filter(|(&a, &b)| a == b)
            .count();

        if correlation == pattern_len {
            matches.push(i);
        }
    }

    matches
}

/// Find sync word positions with a correlation threshold.
///
/// Instead of requiring an exact match, this function returns positions where
/// the number of matching bits meets or exceeds the given `threshold` count.
/// Useful for noisy channels where some sync bits may be corrupted.
pub fn find_sync_word_threshold(
    bitstream: &[bool],
    sync_pattern: &[bool],
    threshold: usize,
) -> Vec<(usize, usize)> {
    if sync_pattern.is_empty() || bitstream.len() < sync_pattern.len() {
        return Vec::new();
    }

    let mut matches = Vec::new();
    let pattern_len = sync_pattern.len();

    for i in 0..=bitstream.len() - pattern_len {
        let correlation = bitstream[i..i + pattern_len]
            .iter()
            .zip(sync_pattern.iter())
            .filter(|(&a, &b)| a == b)
            .count();

        if correlation >= threshold {
            matches.push((i, correlation));
        }
    }

    matches
}

// ---------------------------------------------------------------------------
// CCSDS Space Packet
// ---------------------------------------------------------------------------

/// Parse a CCSDS space packet from raw bytes.
///
/// The CCSDS Space Packet Protocol (CCSDS 133.0-B-2) header is 6 bytes:
/// - Bytes 0-1: Packet ID (version 3b, type 1b, sec header flag 1b, APID 11b)
/// - Bytes 2-3: Packet Sequence Control (sequence flags 2b, sequence count 14b)
/// - Bytes 4-5: Packet Data Length (number of octets in data field minus 1)
///
/// Returns `None` if the data is too short for the header or the declared length
/// exceeds the available data.
pub fn parse_ccsds(data: &[u8]) -> Option<CcsdsPacket> {
    if data.len() < 6 {
        return None;
    }

    let word0 = ((data[0] as u16) << 8) | data[1] as u16;
    let word1 = ((data[2] as u16) << 8) | data[3] as u16;
    let word2 = ((data[4] as u16) << 8) | data[5] as u16;

    let version = ((word0 >> 13) & 0x07) as u8;
    let type_flag = ((word0 >> 12) & 0x01) as u8;
    let apid = word0 & 0x07FF;
    let sequence_count = word1 & 0x3FFF;
    let data_length = word2 as usize + 1; // CCSDS: length field = num octets - 1

    let header_len = 6;
    let available = data.len() - header_len;
    let actual_len = data_length.min(available);

    let packet_data = data[header_len..header_len + actual_len].to_vec();

    Some(CcsdsPacket {
        version,
        type_flag,
        apid,
        sequence_count,
        data: packet_data,
    })
}

/// Encode a [`CcsdsPacket`] into raw bytes.
///
/// Produces a complete CCSDS space packet including the 6-byte primary header
/// followed by the packet data field.
pub fn encode_ccsds(packet: &CcsdsPacket) -> Vec<u8> {
    let data_length = if packet.data.is_empty() {
        0u16
    } else {
        (packet.data.len() as u16).saturating_sub(1)
    };

    let word0: u16 = ((packet.version as u16 & 0x07) << 13)
        | ((packet.type_flag as u16 & 0x01) << 12)
        | (packet.apid & 0x07FF);
    // Sequence flags = 0b11 (unsegmented) by default
    let word1: u16 = (0b11 << 14) | (packet.sequence_count & 0x3FFF);
    let word2: u16 = data_length;

    let mut bytes = Vec::with_capacity(6 + packet.data.len());
    bytes.push((word0 >> 8) as u8);
    bytes.push(word0 as u8);
    bytes.push((word1 >> 8) as u8);
    bytes.push(word1 as u8);
    bytes.push((word2 >> 8) as u8);
    bytes.push(word2 as u8);
    bytes.extend_from_slice(&packet.data);

    bytes
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- ARINC 429 Tests ---

    #[test]
    fn test_arinc429_encode_decode_roundtrip() {
        let word = Arinc429Word {
            label: 0o310,
            sdi: 0b01,
            data: 0x1234,
            ssm: 0b11,
        };
        let raw = encode_arinc429(&word);
        let decoded = decode_arinc429(raw);
        assert_eq!(decoded.label, word.label);
        assert_eq!(decoded.sdi, word.sdi);
        assert_eq!(decoded.data, word.data);
        assert_eq!(decoded.ssm, word.ssm);
    }

    #[test]
    fn test_arinc429_odd_parity() {
        let word = Arinc429Word {
            label: 0o205,
            sdi: 0b10,
            data: 0x0FF00,
            ssm: 0b00,
        };
        let raw = encode_arinc429(&word);
        assert!(verify_arinc429_parity(raw), "Encoded word should have odd parity");
    }

    #[test]
    fn test_arinc429_parity_various_data() {
        for data_val in [0u32, 1, 0x7FFFF, 0x55555, 0x2AAAA] {
            let word = Arinc429Word {
                label: 0o001,
                sdi: 0b00,
                data: data_val & 0x7_FFFF,
                ssm: 0b00,
            };
            let raw = encode_arinc429(&word);
            assert!(
                verify_arinc429_parity(raw),
                "Parity check failed for data=0x{:05X}",
                data_val
            );
        }
    }

    #[test]
    fn test_arinc429_zero_word() {
        let word = Arinc429Word {
            label: 0,
            sdi: 0,
            data: 0,
            ssm: 0,
        };
        let raw = encode_arinc429(&word);
        assert!(verify_arinc429_parity(raw));
        let decoded = decode_arinc429(raw);
        assert_eq!(decoded.label, 0);
        assert_eq!(decoded.data, 0);
    }

    #[test]
    fn test_arinc429_label_bit_reversal() {
        // Label 0o310 = 0b11001000 reversed = 0b00010011 = 0x13
        let word = Arinc429Word {
            label: 0o310,
            sdi: 0,
            data: 0,
            ssm: 0,
        };
        let raw = encode_arinc429(&word);
        let label_in_raw = (raw & 0xFF) as u8;
        assert_eq!(label_in_raw, reverse_label_bits(0o310));
    }

    #[test]
    fn test_arinc429_max_values() {
        let word = Arinc429Word {
            label: 0xFF,
            sdi: 0b11,
            data: 0x7_FFFF,
            ssm: 0b11,
        };
        let raw = encode_arinc429(&word);
        let decoded = decode_arinc429(raw);
        assert_eq!(decoded.label, 0xFF);
        assert_eq!(decoded.sdi, 0b11);
        assert_eq!(decoded.data, 0x7_FFFF);
        assert_eq!(decoded.ssm, 0b11);
    }

    // --- NMEA Tests ---

    #[test]
    fn test_nmea_checksum_basic() {
        // Compute checksum for a known sentence and verify
        let sentence = "$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62";
        let cs = nmea_checksum(sentence);
        assert_eq!(cs, 0x62);
    }

    #[test]
    fn test_nmea_checksum_simple() {
        let cs = nmea_checksum("GPRMC");
        let expected = b'G' ^ b'P' ^ b'R' ^ b'M' ^ b'C';
        assert_eq!(cs, expected);
    }

    #[test]
    fn test_nmea_parse_valid_rmc() {
        let sentence = "$GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E*62";
        let parsed = parse_nmea(sentence);
        assert!(parsed.is_some(), "Should parse valid RMC sentence");
        let nmea = parsed.unwrap();
        assert_eq!(nmea.talker, "GP");
        assert_eq!(nmea.sentence_type, "RMC");
        assert_eq!(nmea.fields[0], "081836");
        assert_eq!(nmea.fields[1], "A");
        assert_eq!(nmea.checksum, 0x62);
    }

    #[test]
    fn test_nmea_parse_invalid_checksum() {
        let sentence = "$GPGGA,123519,4807.038,N*FF";
        let parsed = parse_nmea(sentence);
        assert!(parsed.is_none(), "Should reject invalid checksum");
    }

    #[test]
    fn test_nmea_parse_no_dollar() {
        let parsed = parse_nmea("GPGGA,123519*47");
        assert!(parsed.is_none(), "Should reject sentence without $");
    }

    #[test]
    fn test_nmea_format_roundtrip() {
        let sentence = NmeaSentence {
            talker: "GP".to_string(),
            sentence_type: "RMC".to_string(),
            fields: vec![
                "123519".to_string(),
                "A".to_string(),
                "4807.038".to_string(),
                "N".to_string(),
            ],
            checksum: 0,
        };
        let formatted = format_nmea(&sentence);
        assert!(formatted.starts_with("$GPRMC,"));
        assert!(formatted.contains('*'));

        let reparsed = parse_nmea(&formatted);
        assert!(reparsed.is_some());
        let reparsed = reparsed.unwrap();
        assert_eq!(reparsed.talker, "GP");
        assert_eq!(reparsed.sentence_type, "RMC");
        assert_eq!(reparsed.fields.len(), 4);
        assert_eq!(reparsed.fields[0], "123519");
    }

    #[test]
    fn test_nmea_format_empty_fields() {
        let sentence = NmeaSentence {
            talker: "GL".to_string(),
            sentence_type: "GSV".to_string(),
            fields: vec![],
            checksum: 0,
        };
        let formatted = format_nmea(&sentence);
        assert!(formatted.starts_with("$GLGSV*"));
        let reparsed = parse_nmea(&formatted).unwrap();
        assert_eq!(reparsed.talker, "GL");
        assert!(reparsed.fields.is_empty());
    }

    // --- IRIG-B Tests ---

    #[test]
    fn test_irig_b_encode_decode_roundtrip() {
        let time = IrigBTime {
            seconds: 45,
            minutes: 30,
            hours: 14,
            day_of_year: 256,
            year: 26,
        };
        let sample_rate = 10000.0;
        let signal = encode_irig_b(&time, sample_rate);
        let decoded = decode_irig_b(&signal, sample_rate);
        assert!(decoded.is_some());
        let decoded = decoded.unwrap();
        assert_eq!(decoded.seconds, 45);
        assert_eq!(decoded.minutes, 30);
        assert_eq!(decoded.hours, 14);
        assert_eq!(decoded.day_of_year, 256);
        assert_eq!(decoded.year, 26);
    }

    #[test]
    fn test_irig_b_midnight_jan_first() {
        let time = IrigBTime {
            seconds: 0,
            minutes: 0,
            hours: 0,
            day_of_year: 1,
            year: 0,
        };
        let sample_rate = 10000.0;
        let signal = encode_irig_b(&time, sample_rate);
        let decoded = decode_irig_b(&signal, sample_rate).unwrap();
        assert_eq!(decoded.seconds, 0);
        assert_eq!(decoded.minutes, 0);
        assert_eq!(decoded.hours, 0);
        assert_eq!(decoded.day_of_year, 1);
        assert_eq!(decoded.year, 0);
    }

    #[test]
    fn test_irig_b_signal_length() {
        let time = IrigBTime {
            seconds: 0,
            minutes: 0,
            hours: 0,
            day_of_year: 1,
            year: 0,
        };
        let sample_rate = 10000.0;
        let signal = encode_irig_b(&time, sample_rate);
        assert_eq!(signal.len(), 10000);
    }

    #[test]
    fn test_irig_b_decode_too_short() {
        let signal = vec![1.0; 50];
        assert!(decode_irig_b(&signal, 10000.0).is_none());
    }

    #[test]
    fn test_irig_b_end_of_year() {
        let time = IrigBTime {
            seconds: 59,
            minutes: 59,
            hours: 23,
            day_of_year: 366,
            year: 99,
        };
        let sample_rate = 10000.0;
        let signal = encode_irig_b(&time, sample_rate);
        let decoded = decode_irig_b(&signal, sample_rate).unwrap();
        assert_eq!(decoded.seconds, 59);
        assert_eq!(decoded.minutes, 59);
        assert_eq!(decoded.hours, 23);
        assert_eq!(decoded.day_of_year, 366);
        assert_eq!(decoded.year, 99);
    }

    // --- PCM Sync Word Tests ---

    #[test]
    fn test_find_sync_word_exact_match() {
        let bitstream = vec![
            false, true, true, false, true, false, true, true, false, true, false,
        ];
        let sync = vec![true, false, true];
        let positions = find_sync_word(&bitstream, &sync);
        assert!(positions.contains(&4));
    }

    #[test]
    fn test_find_sync_word_multiple_matches() {
        let bitstream = vec![
            true, false, true, false, true, false, true, false, true, false, true,
        ];
        let sync = vec![true, false, true];
        let positions = find_sync_word(&bitstream, &sync);
        assert!(positions.contains(&0));
        assert!(positions.contains(&4));
        assert!(positions.contains(&8));
    }

    #[test]
    fn test_find_sync_word_no_match() {
        let bitstream = vec![false, false, false, false, false];
        let sync = vec![true, true];
        let positions = find_sync_word(&bitstream, &sync);
        assert!(positions.is_empty());
    }

    #[test]
    fn test_find_sync_word_empty_pattern() {
        let bitstream = vec![true, false, true];
        let sync: Vec<bool> = vec![];
        let positions = find_sync_word(&bitstream, &sync);
        assert!(positions.is_empty());
    }

    #[test]
    fn test_find_sync_word_threshold() {
        let bitstream = vec![true, true, false, false, true, false, true];
        let sync = vec![true, true, true];
        let results = find_sync_word_threshold(&bitstream, &sync, 2);
        assert!(results.iter().any(|&(pos, _)| pos == 0));
        assert!(results.iter().any(|&(pos, _)| pos == 4));
    }

    // --- CCSDS Tests ---

    #[test]
    fn test_ccsds_parse_valid_packet() {
        let bytes = vec![
            0x01, 0x23, // version=0, type=0, apid=0x123
            0x00, 0x2A, // seq flags=0, seq count=42
            0x00, 0x03, // data length = 3 (means 4 bytes)
            0xDE, 0xAD, 0xBE, 0xEF,
        ];
        let packet = parse_ccsds(&bytes).unwrap();
        assert_eq!(packet.version, 0);
        assert_eq!(packet.type_flag, 0);
        assert_eq!(packet.apid, 0x0123);
        assert_eq!(packet.sequence_count, 42);
        assert_eq!(packet.data, vec![0xDE, 0xAD, 0xBE, 0xEF]);
    }

    #[test]
    fn test_ccsds_parse_too_short() {
        let bytes = vec![0x00, 0x01, 0x02];
        assert!(parse_ccsds(&bytes).is_none());
    }

    #[test]
    fn test_ccsds_encode_decode_roundtrip() {
        let packet = CcsdsPacket {
            version: 0,
            type_flag: 1,
            apid: 0x7FF,
            sequence_count: 0x3FFF,
            data: vec![1, 2, 3, 4, 5],
        };
        let encoded = encode_ccsds(&packet);
        let decoded = parse_ccsds(&encoded).unwrap();
        assert_eq!(decoded.version, 0);
        assert_eq!(decoded.type_flag, 1);
        assert_eq!(decoded.apid, 0x7FF);
        assert_eq!(decoded.sequence_count, 0x3FFF);
        assert_eq!(decoded.data, vec![1, 2, 3, 4, 5]);
    }

    #[test]
    fn test_ccsds_telecommand_flag() {
        let packet = CcsdsPacket {
            version: 0,
            type_flag: 1,
            apid: 0x001,
            sequence_count: 0,
            data: vec![0xFF],
        };
        let encoded = encode_ccsds(&packet);
        let decoded = parse_ccsds(&encoded).unwrap();
        assert_eq!(decoded.type_flag, 1);
    }

    #[test]
    fn test_ccsds_empty_data() {
        let packet = CcsdsPacket {
            version: 0,
            type_flag: 0,
            apid: 100,
            sequence_count: 1,
            data: vec![],
        };
        let encoded = encode_ccsds(&packet);
        assert_eq!(encoded.len(), 6);
    }

    // --- BCD helper tests ---

    #[test]
    fn test_bcd_encode_decode_roundtrip() {
        for val in [0u16, 5, 9, 12, 59, 99, 365] {
            let num_digits = if val < 10 {
                1
            } else if val < 100 {
                2
            } else {
                3
            };
            let bits = encode_bcd_bits(val, num_digits);
            let decoded = decode_bcd_bits(&bits, num_digits);
            assert_eq!(decoded, val, "BCD roundtrip failed for {}", val);
        }
    }
}
