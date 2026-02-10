//! DVB-S2 satellite TV frame synchronization and header parsing per ETSI EN 302 307.
//!
//! This module implements the physical-layer deframing for DVB-S2, including:
//!
//! - Start-of-Frame (SOF) detection using the 26-symbol Zadoff-Chu-like pattern
//! - PLHEADER decoding via Reed-Muller (64,7) code
//! - MODCOD identification and frame-length computation
//! - Pilot block detection and stripping
//! - State-machine based frame synchronization (Searching -> Locked -> Tracking)
//!
//! Complex samples are represented as `(f64, f64)` tuples `(re, im)`.
//!
//! # Example
//!
//! ```
//! use r4w_core::dvb_s2_deframer::{DvbS2Deframer, DeframerState};
//!
//! let mut deframer = DvbS2Deframer::new();
//! assert_eq!(deframer.state(), DeframerState::Searching);
//! assert_eq!(deframer.frames_found(), 0);
//! ```

/// The 26-bit SOF code word from ETSI EN 302 307 Table 6.
const SOF_CODE: u32 = 0x18D2E82;

/// Number of SOF symbols.
const SOF_LEN: usize = 26;

/// Number of PLHEADER symbols (SOF + PLS code).
const PLHEADER_SYMBOLS: usize = 90;

/// Number of coded PLS symbols after the SOF (Reed-Muller (64,7) encoded).
const PLS_CODE_LEN: usize = 64;

/// Pilot block length in symbols.
const PILOT_BLOCK_LEN: usize = 36;

/// Number of symbols per slot.
const SLOT_SYMBOLS: usize = 90;

/// Number of slots between pilot blocks.
const PILOT_INTERVAL_SLOTS: usize = 16;

/// Correlation threshold for SOF detection (normalised).
const SOF_THRESHOLD: f64 = 0.65;

/// Number of consecutive frames needed to transition from Searching to Locked.
const LOCK_COUNT: usize = 3;

// ---------------------------------------------------------------------------
// Public types
// ---------------------------------------------------------------------------

/// Deframer state machine states.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DeframerState {
    /// Correlating every incoming symbol against the SOF pattern.
    Searching,
    /// SOF has been detected at least `LOCK_COUNT` consecutive times at the expected position.
    Locked,
    /// Frame boundary is tracked; we predict the next SOF location.
    Tracking,
}

/// DVB-S2 frame type.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameType {
    /// Normal FECFRAME (64 800 bits).
    Normal,
    /// Short FECFRAME (16 200 bits).
    Short,
}

/// Modulation and coding scheme identifiers (MODCOD 0-28 subset).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[allow(non_camel_case_types)]
pub enum ModCod {
    DummyFrame,
    QPSK_1_4,
    QPSK_1_3,
    QPSK_2_5,
    QPSK_1_2,
    QPSK_3_5,
    QPSK_2_3,
    QPSK_3_4,
    QPSK_4_5,
    QPSK_5_6,
    QPSK_8_9,
    QPSK_9_10,
    PSK8_3_5,
    PSK8_2_3,
    PSK8_3_4,
    PSK8_5_6,
    PSK8_8_9,
    PSK8_9_10,
    APSK16_2_3,
    APSK16_3_4,
    APSK16_4_5,
    APSK16_5_6,
    APSK16_8_9,
    APSK16_9_10,
    APSK32_3_4,
    APSK32_4_5,
    APSK32_5_6,
    APSK32_8_9,
    APSK32_9_10,
}

impl ModCod {
    /// Return the MODCOD index (0..=28).
    pub fn index(self) -> u8 {
        match self {
            Self::DummyFrame => 0,
            Self::QPSK_1_4 => 1,
            Self::QPSK_1_3 => 2,
            Self::QPSK_2_5 => 3,
            Self::QPSK_1_2 => 4,
            Self::QPSK_3_5 => 5,
            Self::QPSK_2_3 => 6,
            Self::QPSK_3_4 => 7,
            Self::QPSK_4_5 => 8,
            Self::QPSK_5_6 => 9,
            Self::QPSK_8_9 => 10,
            Self::QPSK_9_10 => 11,
            Self::PSK8_3_5 => 12,
            Self::PSK8_2_3 => 13,
            Self::PSK8_3_4 => 14,
            Self::PSK8_5_6 => 15,
            Self::PSK8_8_9 => 16,
            Self::PSK8_9_10 => 17,
            Self::APSK16_2_3 => 18,
            Self::APSK16_3_4 => 19,
            Self::APSK16_4_5 => 20,
            Self::APSK16_5_6 => 21,
            Self::APSK16_8_9 => 22,
            Self::APSK16_9_10 => 23,
            Self::APSK32_3_4 => 24,
            Self::APSK32_4_5 => 25,
            Self::APSK32_5_6 => 26,
            Self::APSK32_8_9 => 27,
            Self::APSK32_9_10 => 28,
        }
    }

    /// Create from raw MODCOD index.
    pub fn from_index(idx: u8) -> Option<Self> {
        match idx {
            0 => Some(Self::DummyFrame),
            1 => Some(Self::QPSK_1_4),
            2 => Some(Self::QPSK_1_3),
            3 => Some(Self::QPSK_2_5),
            4 => Some(Self::QPSK_1_2),
            5 => Some(Self::QPSK_3_5),
            6 => Some(Self::QPSK_2_3),
            7 => Some(Self::QPSK_3_4),
            8 => Some(Self::QPSK_4_5),
            9 => Some(Self::QPSK_5_6),
            10 => Some(Self::QPSK_8_9),
            11 => Some(Self::QPSK_9_10),
            12 => Some(Self::PSK8_3_5),
            13 => Some(Self::PSK8_2_3),
            14 => Some(Self::PSK8_3_4),
            15 => Some(Self::PSK8_5_6),
            16 => Some(Self::PSK8_8_9),
            17 => Some(Self::PSK8_9_10),
            18 => Some(Self::APSK16_2_3),
            19 => Some(Self::APSK16_3_4),
            20 => Some(Self::APSK16_4_5),
            21 => Some(Self::APSK16_5_6),
            22 => Some(Self::APSK16_8_9),
            23 => Some(Self::APSK16_9_10),
            24 => Some(Self::APSK32_3_4),
            25 => Some(Self::APSK32_4_5),
            26 => Some(Self::APSK32_5_6),
            27 => Some(Self::APSK32_8_9),
            28 => Some(Self::APSK32_9_10),
            _ => None,
        }
    }

    /// Bits per constellation symbol for this modcod.
    pub fn bits_per_symbol(self) -> usize {
        match self.index() {
            0..=11 => 2,  // QPSK
            12..=17 => 3, // 8PSK
            18..=23 => 4, // 16APSK
            24..=28 => 5, // 32APSK
            _ => 2,
        }
    }
}

/// Decoded physical-layer header.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PlHeader {
    /// Modulation and coding index.
    pub modcod: ModCod,
    /// Frame type (Normal or Short).
    pub frame_type: FrameType,
    /// Whether pilot symbols are inserted.
    pub has_pilots: bool,
}

/// A fully deframed DVB-S2 physical-layer frame.
#[derive(Debug, Clone)]
pub struct FrameOutput {
    /// Decoded PLHEADER.
    pub header: PlHeader,
    /// Payload symbols (with pilot blocks removed if present).
    pub payload_symbols: Vec<(f64, f64)>,
    /// Sequential frame number since lock.
    pub frame_number: u64,
}

// ---------------------------------------------------------------------------
// Reed-Muller (64,7) generator matrix rows (ETSI EN 302 307 Table 5a)
// ---------------------------------------------------------------------------

/// Generator matrix for the (64,7) first-order Reed-Muller code, stored as
/// 64-bit masks.  Row *i* is `G[i]`; bit *j* of `G[i]` indicates the *j*-th
/// coded bit.
const RM_GENERATOR: [u64; 7] = [
    0b1010101010101010101010101010101010101010101010101010101010101010,
    0b0110011001100110011001100110011001100110011001100110011001100110,
    0b0001111000011110000111100001111000011110000111100001111000011110,
    0b0000000111111110000000011111111000000001111111100000000111111110,
    0b0000000000000001111111111111111000000000000000011111111111111110,
    0b0000000000000000000000000000000111111111111111111111111111111110,
    0b1111111111111111111111111111111111111111111111111111111111111111,
];

// ---------------------------------------------------------------------------
// Helper functions
// ---------------------------------------------------------------------------

/// Map a single SOF bit (0/1) to a pi/2-BPSK symbol.
/// Even-index symbols: 0 -> (+1,0), 1 -> (-1,0).
/// Odd-index symbols:  0 -> (0,+1), 1 -> (0,-1).
fn sof_bit_to_symbol(bit: u8, index: usize) -> (f64, f64) {
    let sign = if bit == 0 { 1.0 } else { -1.0 };
    if index % 2 == 0 {
        (sign, 0.0)
    } else {
        (0.0, sign)
    }
}

/// Generate the 26 reference SOF symbols.
fn sof_reference() -> [(f64, f64); SOF_LEN] {
    let mut syms = [(0.0, 0.0); SOF_LEN];
    for i in 0..SOF_LEN {
        let bit = ((SOF_CODE >> (SOF_LEN - 1 - i)) & 1) as u8;
        syms[i] = sof_bit_to_symbol(bit, i);
    }
    syms
}

/// Complex dot product: sum of a_i * conj(b_i).
fn complex_dot(a: &[(f64, f64)], b: &[(f64, f64)]) -> (f64, f64) {
    let mut re = 0.0;
    let mut im = 0.0;
    for (ai, bi) in a.iter().zip(b.iter()) {
        re += ai.0 * bi.0 + ai.1 * bi.1;
        im += ai.1 * bi.0 - ai.0 * bi.1;
    }
    (re, im)
}

/// Magnitude of a complex number.
fn complex_mag(c: (f64, f64)) -> f64 {
    (c.0 * c.0 + c.1 * c.1).sqrt()
}

/// Energy of a slice of complex symbols.
fn energy(syms: &[(f64, f64)]) -> f64 {
    syms.iter().map(|s| s.0 * s.0 + s.1 * s.1).sum()
}

/// Encode 7 information bits using the (64,7) Reed-Muller code.
/// Returns a 64-bit codeword.
fn rm_encode(info: u8) -> u64 {
    let mut codeword: u64 = 0;
    for i in 0..7 {
        if (info >> (6 - i)) & 1 == 1 {
            codeword ^= RM_GENERATOR[i];
        }
    }
    codeword
}

/// Decode a 64-bit hard-decision vector to the nearest (64,7) RM codeword.
/// Uses exhaustive search over 128 codewords (fast enough for 7 info bits).
/// Returns `(info_bits_u8, min_distance)`.
fn rm_decode(received: u64) -> (u8, u32) {
    let mut best_info: u8 = 0;
    let mut best_dist: u32 = 64;
    for candidate in 0u8..128 {
        let cw = rm_encode(candidate);
        let dist = (cw ^ received).count_ones();
        if dist < best_dist {
            best_dist = dist;
            best_info = candidate;
            if dist == 0 {
                break;
            }
        }
    }
    (best_info, best_dist)
}

/// Convert hard-decision PLS symbols (pi/2-BPSK) to a 64-bit vector.
fn pls_symbols_to_bits(syms: &[(f64, f64)]) -> u64 {
    debug_assert!(syms.len() == PLS_CODE_LEN);
    let mut bits: u64 = 0;
    for (i, s) in syms.iter().enumerate() {
        let val = if i % 2 == 0 { s.0 } else { s.1 };
        let bit = if val < 0.0 { 1u64 } else { 0u64 };
        bits = (bits << 1) | bit;
    }
    bits
}

/// Extract MODCOD, frame type and pilots flag from the 7-bit PLS code field.
///
/// Bits b6..b0 (MSB first):
///   - b6..b2 = MODCOD (0..31)
///   - b1     = frame type (0 = Normal, 1 = Short)
///   - b0     = pilots (0 = off, 1 = on)
fn pls_to_header(info: u8) -> Option<PlHeader> {
    let modcod_idx = (info >> 2) & 0x1F;
    let frame_type = if (info >> 1) & 1 == 1 {
        FrameType::Short
    } else {
        FrameType::Normal
    };
    let has_pilots = (info & 1) == 1;
    let modcod = ModCod::from_index(modcod_idx)?;
    Some(PlHeader {
        modcod,
        frame_type,
        has_pilots,
    })
}

// ---------------------------------------------------------------------------
// Frame length computation (ETSI EN 302 307 Table 12)
// ---------------------------------------------------------------------------

/// Total number of payload symbols for a given MODCOD and frame type.
fn payload_symbols_count(modcod: ModCod, frame_type: FrameType) -> usize {
    let fec_bits: usize = match frame_type {
        FrameType::Normal => 64_800,
        FrameType::Short => 16_200,
    };
    let bps = modcod.bits_per_symbol();
    (fec_bits + bps - 1) / bps
}

/// Number of pilot blocks inserted in the frame (0 if pilots are off).
fn pilot_block_count(payload_syms: usize, has_pilots: bool) -> usize {
    if !has_pilots {
        return 0;
    }
    let segment = PILOT_INTERVAL_SLOTS * SLOT_SYMBOLS; // 1440
    if payload_syms == 0 {
        return 0;
    }
    (payload_syms - 1) / segment
}

/// Total PLFRAME length in symbols (PLHEADER + payload + pilot blocks).
pub fn frame_length(header: &PlHeader) -> usize {
    let data_syms = payload_symbols_count(header.modcod, header.frame_type);
    let pilots = pilot_block_count(data_syms, header.has_pilots);
    PLHEADER_SYMBOLS + data_syms + pilots * PILOT_BLOCK_LEN
}

// ---------------------------------------------------------------------------
// DvbS2Deframer
// ---------------------------------------------------------------------------

/// DVB-S2 physical-layer deframer.
///
/// Feed complex symbols via [`process_symbols`](Self::process_symbols) and
/// collect decoded [`FrameOutput`] values.
pub struct DvbS2Deframer {
    state: DeframerState,
    /// Internal symbol buffer.
    buffer: Vec<(f64, f64)>,
    /// Reference SOF symbols.
    sof_ref: [(f64, f64); SOF_LEN],
    /// Number of consecutive SOF detections at predicted positions.
    consecutive_locks: usize,
    /// Total frames decoded since creation.
    frames_found: u64,
    /// Expected distance (in symbols) to the next SOF from the current buffer start.
    predicted_frame_len: Option<usize>,
    /// Running frame counter (reset on lock loss).
    frame_counter: u64,
    /// Total symbols processed.
    symbols_processed: u64,
}

impl DvbS2Deframer {
    /// Create a new deframer in the [`Searching`](DeframerState::Searching) state.
    pub fn new() -> Self {
        Self {
            state: DeframerState::Searching,
            buffer: Vec::with_capacity(65536),
            sof_ref: sof_reference(),
            consecutive_locks: 0,
            frames_found: 0,
            predicted_frame_len: None,
            frame_counter: 0,
            symbols_processed: 0,
        }
    }

    /// Current deframer state.
    pub fn state(&self) -> DeframerState {
        self.state
    }

    /// Number of frames successfully decoded.
    pub fn frames_found(&self) -> u64 {
        self.frames_found
    }

    /// Total symbols ingested.
    pub fn symbols_processed(&self) -> u64 {
        self.symbols_processed
    }

    /// Number of consecutive on-time SOF detections.
    pub fn consecutive_locks(&self) -> usize {
        self.consecutive_locks
    }

    /// Reset the deframer to the initial searching state.
    pub fn reset(&mut self) {
        self.state = DeframerState::Searching;
        self.buffer.clear();
        self.consecutive_locks = 0;
        self.predicted_frame_len = None;
        self.frame_counter = 0;
    }

    // ------------------------------------------------------------------
    // SOF detection
    // ------------------------------------------------------------------

    /// Detect the start-of-frame pattern at position `pos` in the buffer.
    /// Returns the normalised correlation magnitude (0.0 - 1.0).
    pub fn detect_sof(&self, pos: usize) -> f64 {
        if pos + SOF_LEN > self.buffer.len() {
            return 0.0;
        }
        let window = &self.buffer[pos..pos + SOF_LEN];
        let dot = complex_dot(window, &self.sof_ref);
        let e_window = energy(window);
        let e_ref = energy(&self.sof_ref);
        let denom = (e_window * e_ref).sqrt();
        if denom < 1e-12 {
            return 0.0;
        }
        complex_mag(dot) / denom
    }

    /// Scan the buffer for the best SOF correlation above threshold.
    /// Returns `Some(position)` or `None`.
    fn search_sof(&self) -> Option<usize> {
        if self.buffer.len() < SOF_LEN {
            return None;
        }
        let mut best_pos = 0;
        let mut best_corr = 0.0f64;
        let search_end = self.buffer.len() - SOF_LEN;
        for pos in 0..=search_end {
            let corr = self.detect_sof(pos);
            if corr > best_corr {
                best_corr = corr;
                best_pos = pos;
            }
        }
        if best_corr >= SOF_THRESHOLD {
            Some(best_pos)
        } else {
            None
        }
    }

    // ------------------------------------------------------------------
    // PLHEADER decoding
    // ------------------------------------------------------------------

    /// Decode the PLHEADER starting at `pos` in the buffer.
    /// Returns `Some(PlHeader)` on success.
    pub fn decode_plheader(&self, pos: usize) -> Option<PlHeader> {
        let pls_start = pos + SOF_LEN;
        let pls_end = pls_start + PLS_CODE_LEN;
        if pls_end > self.buffer.len() {
            return None;
        }
        let pls_syms = &self.buffer[pls_start..pls_end];
        let received = pls_symbols_to_bits(pls_syms);
        let (info, _dist) = rm_decode(received);
        pls_to_header(info)
    }

    // ------------------------------------------------------------------
    // Pilot stripping
    // ------------------------------------------------------------------

    /// Remove pilot blocks from a payload symbol slice.
    fn strip_pilots(payload: &[(f64, f64)], has_pilots: bool) -> Vec<(f64, f64)> {
        if !has_pilots {
            return payload.to_vec();
        }
        let segment = PILOT_INTERVAL_SLOTS * SLOT_SYMBOLS; // 1440
        let mut out = Vec::with_capacity(payload.len());
        let mut i = 0;
        let mut syms_since_pilot = 0usize;
        while i < payload.len() {
            if syms_since_pilot == segment && i + PILOT_BLOCK_LEN <= payload.len() {
                // Skip pilot block
                i += PILOT_BLOCK_LEN;
                syms_since_pilot = 0;
            } else {
                out.push(payload[i]);
                i += 1;
                syms_since_pilot += 1;
            }
        }
        out
    }

    // ------------------------------------------------------------------
    // Main processing entry point
    // ------------------------------------------------------------------

    /// Feed new symbols into the deframer and return any complete frames.
    pub fn process_symbols(&mut self, symbols: &[(f64, f64)]) -> Vec<FrameOutput> {
        self.symbols_processed += symbols.len() as u64;
        self.buffer.extend_from_slice(symbols);

        let mut frames = Vec::new();

        loop {
            match self.state {
                DeframerState::Searching => {
                    if let Some(pos) = self.search_sof() {
                        // Discard everything before the SOF.
                        if pos > 0 {
                            self.buffer.drain(..pos);
                        }
                        // Try to decode header.
                        if let Some(hdr) = self.decode_plheader(0) {
                            let flen = frame_length(&hdr);
                            self.predicted_frame_len = Some(flen);
                            self.consecutive_locks += 1;

                            if self.buffer.len() >= flen {
                                let payload_start = PLHEADER_SYMBOLS;
                                let payload_end = flen;
                                let raw_payload =
                                    self.buffer[payload_start..payload_end].to_vec();
                                let payload =
                                    Self::strip_pilots(&raw_payload, hdr.has_pilots);
                                self.frame_counter += 1;
                                self.frames_found += 1;
                                frames.push(FrameOutput {
                                    header: hdr,
                                    payload_symbols: payload,
                                    frame_number: self.frame_counter,
                                });
                                self.buffer.drain(..flen);

                                if self.consecutive_locks >= LOCK_COUNT {
                                    self.state = DeframerState::Locked;
                                }
                                continue;
                            }
                        }
                        break; // Not enough data
                    } else {
                        // Keep last SOF_LEN-1 symbols for overlap.
                        let keep = SOF_LEN.saturating_sub(1);
                        if self.buffer.len() > keep {
                            let drain = self.buffer.len() - keep;
                            self.buffer.drain(..drain);
                        }
                        break;
                    }
                }
                DeframerState::Locked | DeframerState::Tracking => {
                    // Need at least a full PLHEADER to attempt decode.
                    if self.buffer.len() < PLHEADER_SYMBOLS {
                        break;
                    }
                    // We expect the next SOF at position 0.
                    let corr = self.detect_sof(0);
                    if corr >= SOF_THRESHOLD {
                        if let Some(hdr) = self.decode_plheader(0) {
                            let flen = frame_length(&hdr);
                            self.predicted_frame_len = Some(flen);

                            if self.buffer.len() >= flen {
                                let raw_payload =
                                    self.buffer[PLHEADER_SYMBOLS..flen].to_vec();
                                let payload =
                                    Self::strip_pilots(&raw_payload, hdr.has_pilots);
                                self.frame_counter += 1;
                                self.frames_found += 1;
                                frames.push(FrameOutput {
                                    header: hdr,
                                    payload_symbols: payload,
                                    frame_number: self.frame_counter,
                                });
                                self.buffer.drain(..flen);
                                self.consecutive_locks += 1;
                                self.state = DeframerState::Tracking;
                                continue;
                            }
                            break; // need more symbols
                        }
                    }
                    // SOF not found at expected position - lose lock.
                    self.consecutive_locks = 0;
                    self.state = DeframerState::Searching;
                    continue;
                }
            }
        }

        frames
    }
}

impl Default for DvbS2Deframer {
    fn default() -> Self {
        Self::new()
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -- helpers --

    /// Build a PLHEADER symbol sequence (SOF + PLS code) for a given PlHeader.
    fn build_plheader_symbols(hdr: &PlHeader) -> Vec<(f64, f64)> {
        let sof = sof_reference();
        let mut syms: Vec<(f64, f64)> = sof.to_vec();

        let info: u8 = (hdr.modcod.index() << 2)
            | (if hdr.frame_type == FrameType::Short { 2 } else { 0 })
            | (if hdr.has_pilots { 1 } else { 0 });
        let codeword = rm_encode(info);
        for i in 0..PLS_CODE_LEN {
            let bit = ((codeword >> (PLS_CODE_LEN - 1 - i)) & 1) as u8;
            syms.push(sof_bit_to_symbol(bit, i));
        }
        syms
    }

    /// Build a complete PLFRAME (header + dummy payload + pilots if needed).
    fn build_frame(hdr: &PlHeader) -> Vec<(f64, f64)> {
        let mut syms = build_plheader_symbols(hdr);
        let flen = frame_length(hdr);
        while syms.len() < flen {
            let phase = (syms.len() as f64) * 0.3;
            syms.push((phase.cos(), phase.sin()));
        }
        syms.truncate(flen);
        syms
    }

    // -- tests --

    #[test]
    fn test_sof_reference_length() {
        let sof = sof_reference();
        assert_eq!(sof.len(), SOF_LEN);
        for s in &sof {
            let e = s.0 * s.0 + s.1 * s.1;
            assert!((e - 1.0).abs() < 1e-12, "SOF symbol energy = {}", e);
        }
    }

    #[test]
    fn test_sof_detection_perfect() {
        let sof = sof_reference();
        let mut deframer = DvbS2Deframer::new();
        deframer.buffer.extend_from_slice(&sof);
        let corr = deframer.detect_sof(0);
        assert!(
            (corr - 1.0).abs() < 1e-9,
            "Perfect SOF correlation should be 1.0, got {}",
            corr
        );
    }

    #[test]
    fn test_sof_detection_offset() {
        let sof = sof_reference();
        let mut deframer = DvbS2Deframer::new();
        for i in 0..50 {
            let phase = i as f64 * 1.7;
            deframer.buffer.push((phase.cos() * 0.3, phase.sin() * 0.3));
        }
        deframer.buffer.extend_from_slice(&sof);
        let pos = deframer.search_sof();
        assert_eq!(pos, Some(50));
    }

    #[test]
    fn test_rm_encode_decode_roundtrip() {
        for info in 0..128u8 {
            let cw = rm_encode(info);
            let (decoded, dist) = rm_decode(cw);
            assert_eq!(decoded, info, "RM roundtrip failed for info={}", info);
            assert_eq!(dist, 0);
        }
    }

    #[test]
    fn test_rm_decode_with_errors() {
        let info: u8 = 42;
        let cw = rm_encode(info);
        // Flip 15 bits -- the (64,7) RM code has min distance 32,
        // so it can correct up to 15 errors.
        let corrupted = cw ^ 0x7FFF;
        let (decoded, _) = rm_decode(corrupted);
        assert_eq!(decoded, info, "RM should correct 15 errors");
    }

    #[test]
    fn test_modcod_roundtrip() {
        for idx in 0..=28u8 {
            let mc = ModCod::from_index(idx).unwrap();
            assert_eq!(mc.index(), idx);
        }
        assert!(ModCod::from_index(29).is_none());
    }

    #[test]
    fn test_pls_to_header() {
        // QPSK 1/2, Normal, no pilots => MODCOD=4, type=0, pilots=0
        // info = 4 << 2 | 0 | 0 = 16
        let hdr = pls_to_header(16).unwrap();
        assert_eq!(hdr.modcod, ModCod::QPSK_1_2);
        assert_eq!(hdr.frame_type, FrameType::Normal);
        assert!(!hdr.has_pilots);

        // 8PSK 3/5, Short, pilots => MODCOD=12, type=1, pilots=1
        // info = 12 << 2 | 2 | 1 = 51
        let hdr2 = pls_to_header(51).unwrap();
        assert_eq!(hdr2.modcod, ModCod::PSK8_3_5);
        assert_eq!(hdr2.frame_type, FrameType::Short);
        assert!(hdr2.has_pilots);
    }

    #[test]
    fn test_frame_length_qpsk_normal() {
        let hdr = PlHeader {
            modcod: ModCod::QPSK_1_2,
            frame_type: FrameType::Normal,
            has_pilots: false,
        };
        let flen = frame_length(&hdr);
        // 64800 / 2 = 32400 payload symbols, 0 pilot blocks
        // total = 90 + 32400 = 32490
        assert_eq!(flen, 32490);
    }

    #[test]
    fn test_frame_length_with_pilots() {
        let hdr = PlHeader {
            modcod: ModCod::QPSK_1_2,
            frame_type: FrameType::Normal,
            has_pilots: true,
        };
        let flen = frame_length(&hdr);
        // payload = 32400 symbols
        // pilot blocks = (32400 - 1) / 1440 = 22
        // total = 90 + 32400 + 22 * 36 = 90 + 32400 + 792 = 33282
        assert_eq!(flen, 33282);
    }

    #[test]
    fn test_frame_length_short_frame() {
        let hdr = PlHeader {
            modcod: ModCod::QPSK_3_4,
            frame_type: FrameType::Short,
            has_pilots: false,
        };
        let flen = frame_length(&hdr);
        // 16200 / 2 = 8100 payload, no pilots
        // total = 90 + 8100 = 8190
        assert_eq!(flen, 8190);
    }

    #[test]
    fn test_plheader_decode_synthetic() {
        let hdr = PlHeader {
            modcod: ModCod::QPSK_3_5,
            frame_type: FrameType::Normal,
            has_pilots: false,
        };
        let syms = build_plheader_symbols(&hdr);
        let mut deframer = DvbS2Deframer::new();
        deframer.buffer = syms;
        let decoded = deframer.decode_plheader(0).unwrap();
        assert_eq!(decoded, hdr);
    }

    #[test]
    fn test_process_single_frame() {
        let hdr = PlHeader {
            modcod: ModCod::QPSK_1_2,
            frame_type: FrameType::Short,
            has_pilots: false,
        };
        let frame = build_frame(&hdr);
        let mut deframer = DvbS2Deframer::new();
        let output = deframer.process_symbols(&frame);
        assert_eq!(output.len(), 1);
        assert_eq!(output[0].header, hdr);
        assert_eq!(output[0].frame_number, 1);
        assert_eq!(deframer.frames_found(), 1);
    }

    #[test]
    fn test_state_transitions_to_locked() {
        let hdr = PlHeader {
            modcod: ModCod::QPSK_1_4,
            frame_type: FrameType::Short,
            has_pilots: false,
        };
        let mut deframer = DvbS2Deframer::new();
        assert_eq!(deframer.state(), DeframerState::Searching);

        for i in 0..LOCK_COUNT {
            let frame = build_frame(&hdr);
            let _ = deframer.process_symbols(&frame);
            if i + 1 >= LOCK_COUNT {
                assert!(
                    deframer.state() == DeframerState::Locked
                        || deframer.state() == DeframerState::Tracking,
                    "Should be Locked/Tracking after {} frames, got {:?}",
                    i + 1,
                    deframer.state()
                );
            }
        }
        assert_eq!(deframer.frames_found(), LOCK_COUNT as u64);
    }

    #[test]
    fn test_reset() {
        let mut deframer = DvbS2Deframer::new();
        let hdr = PlHeader {
            modcod: ModCod::QPSK_1_3,
            frame_type: FrameType::Short,
            has_pilots: false,
        };
        let frame = build_frame(&hdr);
        let _ = deframer.process_symbols(&frame);
        assert!(deframer.frames_found() > 0);
        deframer.reset();
        assert_eq!(deframer.state(), DeframerState::Searching);
        assert_eq!(deframer.consecutive_locks(), 0);
    }

    #[test]
    fn test_bits_per_symbol() {
        assert_eq!(ModCod::QPSK_1_2.bits_per_symbol(), 2);
        assert_eq!(ModCod::PSK8_3_5.bits_per_symbol(), 3);
        assert_eq!(ModCod::APSK16_2_3.bits_per_symbol(), 4);
        assert_eq!(ModCod::APSK32_3_4.bits_per_symbol(), 5);
    }

    #[test]
    fn test_pilot_block_count() {
        assert_eq!(pilot_block_count(32400, true), 22);
        assert_eq!(pilot_block_count(32400, false), 0);
        assert_eq!(pilot_block_count(100, true), 0);
        assert_eq!(pilot_block_count(1440, true), 0);
        assert_eq!(pilot_block_count(1441, true), 1);
    }

    #[test]
    fn test_default_trait() {
        let d = DvbS2Deframer::default();
        assert_eq!(d.state(), DeframerState::Searching);
        assert_eq!(d.frames_found(), 0);
    }
}
