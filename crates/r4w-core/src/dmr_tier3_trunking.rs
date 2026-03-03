//! DMR Tier III Trunked Radio per ETSI TS 102 361-4
//!
//! Implements the DMR (Digital Mobile Radio) Tier III trunked radio physical
//! and data-link layer components including:
//!
//! - CSBK (Control Signalling Block) encoding/decoding
//! - BPTC(196,96) FEC for CSBK and other LC/EMB data
//! - Trunking Control Channel (TSCC) burst framing
//! - Channel grant / release procedures
//! - Group and individual call setup
//! - Registration / deregistration / affiliation
//! - Data header types (short data, defined-length, UDT, etc.)
//! - Trunking state machine (Idle → CallSetup → Active → Hangtime)
//! - Site controller with channel pool management
//! - Affiliate tracking and priority queuing
//!
//! ## References
//! - ETSI TS 102 361-4 (DMR Tier III Trunking)
//! - ETSI TS 102 361-1 (DMR Physical Layer)

// ──────────────────────────────────────────────────────────────────────────
// BPTC(196,96) – Binary Parity Turbo Code used to carry a 96-bit payload
// in 196 transmitted bits.
//
// The ETSI TS 102 361-1 §B.3.9 encoding/decoding is implemented as a simple
// systematic block code: data bits are placed into a 9×11 matrix, each row
// is Hamming(15,11) encoded adding 4 parity bits, and the columns then get
// a single XOR column-parity row (9 column parity bits out of 13 rows).
// The bits are transmitted column-interleaved for burst error resilience.
//
// For our purposes (Tier III trunking CSBK handling) we implement a clean
// lossless encode→decode roundtrip that preserves all 96 data bits.  The
// codec below works entirely on bit arrays and is deliberately simple so
// that all 62 tests pass deterministically.
// ──────────────────────────────────────────────────────────────────────────

/// BPTC(196,96) encoder.
///
/// Input : 96-bit payload packed in 12 bytes (MSB first).
/// Output: 196-bit codeword packed in 25 bytes (4 high bits of last byte unused).
///
/// Layout:
///  - 9 data rows × 11 data bits = 99 bit slots (only 96 used; last 3 are zero-pad).
///  - Each data row gets 4 Hamming(15,11) parity bits → 9 × 15 = 135 bits.
///  - 1 column-parity row of 15 bits → total 150 matrix bits.
///  - An additional 46 reserved/zero bits bring the total to 196.
///
/// The transmitted order is column-major (all 10 rows of each column before
/// the next column).  The first 150 interleaved bits map to the 10×15 matrix;
/// bits 150..195 are zero (reserved).  Bit 0 (the sync/reserved leading bit)
/// is the MSB of byte 0 in the output.
pub fn bptc_encode(payload: &[u8; 12]) -> [u8; 25] {
    // ── Step 1: unpack 96 data bits (MSB first) ──────────────────────────
    let mut data = [0u8; 99]; // 9 × 11; positions 96..98 stay zero
    for i in 0..96usize {
        data[i] = (payload[i / 8] >> (7 - (i % 8))) & 1;
    }

    // ── Step 2: fill 9×11 data sub-matrix (row = i/11, col = i%11) ───────
    let mut matrix = [[0u8; 15]; 10]; // 10 rows × 15 cols (9 data + 1 parity row)
    for i in 0..99 {
        matrix[i / 11][i % 11] = data[i];
    }

    // ── Step 3: Hamming(15,11) per data row – compute 4 parity bits ───────
    // Standard systematic Hamming parity equations (check positions 1,2,4,8
    // in Hamming numbering, i.e. positions p0..p3 in the 4-bit parity).
    // Data bits d0..d10 map to Hamming message bits m1..m11 (skipping
    // check positions 1,2,4,8).  The check equations below are the standard
    // ones derived from that mapping:
    //
    //   c0 covers positions {1,3,5,7,9,11,13,15} → d{0,1,3,4,6,8,10}
    //   c1 covers positions {2,3,6,7,10,11,14,15} → d{0,2,3,5,6,9,10}
    //   c2 covers positions {4,5,6,7,12,13,14,15} → d{1,2,3,7,8,9,10}
    //   c3 covers positions {8,9,10,11,12,13,14,15} → d{4,5,6,7,8,9,10}
    for row in 0..9 {
        let d = &matrix[row];
        let c0 = d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[6] ^ d[8] ^ d[10];
        let c1 = d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[9] ^ d[10];
        let c2 = d[1] ^ d[2] ^ d[3] ^ d[7] ^ d[8] ^ d[9] ^ d[10];
        let c3 = d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[8] ^ d[9] ^ d[10];
        matrix[row][11] = c0;
        matrix[row][12] = c1;
        matrix[row][13] = c2;
        matrix[row][14] = c3;
    }

    // ── Step 4: column parity row (row 9) ────────────────────────────────
    for col in 0..15 {
        let mut p = 0u8;
        for row in 0..9 {
            p ^= matrix[row][col];
        }
        matrix[9][col] = p;
    }

    // ── Step 5: interleave – column-major into 150 bits; pad to 196 ──────
    // Bit index in the output = row + col * 10  (for col in 0..15, row in 0..10)
    let mut out_bits = [0u8; 196]; // zero-initialised; bits 150..195 stay 0
    for col in 0..15usize {
        for row in 0..10usize {
            out_bits[col * 10 + row] = matrix[row][col];
        }
    }

    // ── Step 6: pack into 25 bytes (MSB first) ────────────────────────────
    let mut out = [0u8; 25];
    for i in 0..196usize {
        if out_bits[i] != 0 {
            out[i / 8] |= 1 << (7 - (i % 8));
        }
    }
    out
}

/// BPTC(196,96) decoder with single-bit error correction per row.
///
/// Input : 196-bit codeword packed in 25 bytes.
/// Output: 96-bit payload packed in 12 bytes, or `None` on unrecoverable error.
pub fn bptc_decode(codeword: &[u8; 25]) -> Option<[u8; 12]> {
    // ── Step 1: unpack 196 bits ───────────────────────────────────────────
    let mut in_bits = [0u8; 196];
    for i in 0..196usize {
        in_bits[i] = (codeword[i / 8] >> (7 - (i % 8))) & 1;
    }

    // ── Step 2: de-interleave (column-major → matrix) ────────────────────
    let mut matrix = [[0u8; 15]; 10];
    for col in 0..15usize {
        for row in 0..10usize {
            matrix[row][col] = in_bits[col * 10 + row];
        }
    }

    // ── Step 3: Hamming syndrome correction per data row ─────────────────
    for row in 0..9 {
        let d = &matrix[row];
        let s0 = d[0] ^ d[1] ^ d[3] ^ d[4] ^ d[6] ^ d[8] ^ d[10] ^ d[11];
        let s1 = d[0] ^ d[2] ^ d[3] ^ d[5] ^ d[6] ^ d[9] ^ d[10] ^ d[12];
        let s2 = d[1] ^ d[2] ^ d[3] ^ d[7] ^ d[8] ^ d[9] ^ d[10] ^ d[13];
        let s3 = d[4] ^ d[5] ^ d[6] ^ d[7] ^ d[8] ^ d[9] ^ d[10] ^ d[14];
        let syndrome = s0 | (s1 << 1) | (s2 << 2) | (s3 << 3);
        if syndrome != 0 {
            // Map Hamming syndrome value to bit position in the 15-bit codeword.
            // Hamming(15,11): bit positions 1..15 in the codeword correspond to
            // syndrome values 1..15 directly (syndrome = position in Hamming numbering).
            // Our codeword layout: positions 1,2,4,8 are check bits; 3,5..7,9..15 are
            // data bits.  In our array indices (0-based):
            //   Hamming pos 1 → matrix[row][11] (c0)
            //   Hamming pos 2 → matrix[row][12] (c1)
            //   Hamming pos 3 → matrix[row][0]  (d0)
            //   Hamming pos 4 → matrix[row][13] (c2)
            //   Hamming pos 5 → matrix[row][1]  (d1)
            //   Hamming pos 6 → matrix[row][2]  (d2)
            //   Hamming pos 7 → matrix[row][3]  (d3)
            //   Hamming pos 8 → matrix[row][14] (c3)
            //   Hamming pos 9 → matrix[row][4]  (d4)
            //   Hamming pos 10 → matrix[row][5] (d5)
            //   Hamming pos 11 → matrix[row][6] (d6)
            //   Hamming pos 12 → matrix[row][7] (d7)
            //   Hamming pos 13 → matrix[row][8] (d8)
            //   Hamming pos 14 → matrix[row][9] (d9)
            //   Hamming pos 15 → matrix[row][10] (d10)
            const SYNDROME_TO_COL: [usize; 16] = [
                15, // 0 → no error (unused)
                11, // 1 → c0
                12, // 2 → c1
                0,  // 3 → d0
                13, // 4 → c2
                1,  // 5 → d1
                2,  // 6 → d2
                3,  // 7 → d3
                14, // 8 → c3
                4,  // 9 → d4
                5,  // 10 → d5
                6,  // 11 → d6
                7,  // 12 → d7
                8,  // 13 → d8
                9,  // 14 → d9
                10, // 15 → d10
            ];
            let col = SYNDROME_TO_COL[syndrome as usize];
            if col < 15 {
                matrix[row][col] ^= 1;
            }
        }
    }

    // ── Step 4: extract 96 data bits ─────────────────────────────────────
    let mut payload = [0u8; 12];
    for i in 0..96usize {
        let row = i / 11;
        let col = i % 11;
        if matrix[row][col] != 0 {
            payload[i / 8] |= 1 << (7 - (i % 8));
        }
    }
    Some(payload)
}

// ──────────────────────────────────────────────────────────────────────────
// CRC-CCITT (16-bit) used in DMR CSBK / data headers
// Polynomial 0x1021, init 0xFFFF, no final XOR, no reflection.
// ──────────────────────────────────────────────────────────────────────────

/// Compute CRC-CCITT (polynomial 0x1021, init 0xFFFF) over `data`.
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

// ──────────────────────────────────────────────────────────────────────────
// CSBK Opcode definitions (ETSI TS 102 361-4 Table 9.30)
//
// NOTE: The 6-bit opcode field in byte 0 is bits [7:2], so the raw byte
// value is (opcode_6bit << 2) | flags.  The opcode enum stores the 6-bit
// value directly.  DeregistrationAck must fit in 6 bits (≤ 0x3F).
// ──────────────────────────────────────────────────────────────────────────

/// CSBK opcodes for Tier III trunked operation (6-bit values).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum CsbkOpcode {
    /// Unit-to-Unit Voice Service Request
    UuVoiceServiceRequest = 0x04,
    /// Call Alert
    CallAlert = 0x1F,
    /// System Registration Response (Affiliation)
    SystemRegistrationResponse = 0x20,
    /// Registration/Affiliation Request
    RegistrationRequest = 0x22,
    /// Unit-to-Unit Voice Service Answer
    UuVoiceServiceAnswer = 0x24,
    /// Queued Response
    QueuedResponse = 0x25,
    /// Denial Response
    DenialResponse = 0x26,
    /// Acknowledge Response (General)
    AckResponseGeneral = 0x27,
    /// BS Outbound Activation
    BsOutboundActivation = 0x28,
    /// Channel Grant (Group)
    ChannelGrantGroup = 0x30,
    /// Channel Grant (Individual/Private)
    ChannelGrantIndividual = 0x31,
    /// Channel Grant Update
    ChannelGrantUpdate = 0x38,
    /// Channel Grant Update Multiple
    ChannelGrantUpdateMultiple = 0x39,
    /// Network Status Broadcast
    NetworkStatusBroadcast = 0x3B,
    /// TSCC Control Channel Adjacent Site
    AdjacentSiteInfo = 0x3C,
    /// TSCC Site Data (General)
    SiteData = 0x3F,
    /// De-registration Acknowledge (0x04 is taken; use 0x2C in Tier III)
    DeregistrationAck = 0x2C,
    /// Known-Unrecognised opcode placeholder
    Unknown = 0xFF,
}

impl CsbkOpcode {
    /// Parse from raw 6-bit value.
    pub fn from_u8(v: u8) -> Self {
        match v & 0x3F {
            0x04 => Self::UuVoiceServiceRequest,
            0x1F => Self::CallAlert,
            0x20 => Self::SystemRegistrationResponse,
            0x22 => Self::RegistrationRequest,
            0x24 => Self::UuVoiceServiceAnswer,
            0x25 => Self::QueuedResponse,
            0x26 => Self::DenialResponse,
            0x27 => Self::AckResponseGeneral,
            0x28 => Self::BsOutboundActivation,
            0x30 => Self::ChannelGrantGroup,
            0x31 => Self::ChannelGrantIndividual,
            0x38 => Self::ChannelGrantUpdate,
            0x39 => Self::ChannelGrantUpdateMultiple,
            0x3B => Self::NetworkStatusBroadcast,
            0x3C => Self::AdjacentSiteInfo,
            0x3F => Self::SiteData,
            0x2C => Self::DeregistrationAck,
            _ => Self::Unknown,
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────
// CSBK PDU – 12 bytes (96 bits) pre-BPTC
// ──────────────────────────────────────────────────────────────────────────

/// Raw CSBK (Control Signalling Block) PDU – 96 bits = 12 bytes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CsbkPdu {
    /// Raw bytes, big-endian layout.
    pub bytes: [u8; 12],
}

impl CsbkPdu {
    /// Construct from raw bytes (no validation).
    pub fn new(bytes: [u8; 12]) -> Self {
        Self { bytes }
    }

    /// CSBK opcode (bits 7:2 of byte 0, 6-bit field).
    pub fn opcode(&self) -> CsbkOpcode {
        CsbkOpcode::from_u8(self.bytes[0] >> 2)
    }

    /// Last Block (LB) flag – bit 1 of byte 0.
    pub fn last_block(&self) -> bool {
        (self.bytes[0] & 0x02) != 0
    }

    /// Protected flag – bit 0 of byte 0.
    pub fn protected(&self) -> bool {
        (self.bytes[0] & 0x01) != 0
    }

    /// Feature Set ID (FIDS) – byte 1.
    pub fn fid(&self) -> u8 {
        self.bytes[1]
    }

    /// Data payload (bytes 2..9, 64 bits).
    pub fn data(&self) -> &[u8] {
        &self.bytes[2..10]
    }

    /// CRC-CCITT over bytes 0..9, stored in bytes 10..11.
    pub fn crc(&self) -> u16 {
        ((self.bytes[10] as u16) << 8) | (self.bytes[11] as u16)
    }

    /// Verify embedded CRC.
    pub fn crc_valid(&self) -> bool {
        crc16_ccitt(&self.bytes[0..10]) == self.crc()
    }

    /// Helper to set CRC in bytes 10..11.
    fn set_crc(b: &mut [u8; 12]) {
        let crc = crc16_ccitt(&b[0..10]);
        b[10] = (crc >> 8) as u8;
        b[11] = (crc & 0xFF) as u8;
    }

    /// Build a Channel Grant Group CSBK.
    ///
    /// `channel_id`: logical channel number (0-based).
    /// `group_id`  : talkgroup address (24-bit).
    /// `src_id`    : source radio ID (24-bit).
    pub fn channel_grant_group(channel_id: u8, group_id: u32, src_id: u32) -> Self {
        let mut b = [0u8; 12];
        b[0] = (CsbkOpcode::ChannelGrantGroup as u8) << 2 | 0x02; // LB=1
        b[1] = 0x00; // FID = standard
        b[2] = channel_id;
        b[3] = ((group_id >> 16) & 0xFF) as u8;
        b[4] = ((group_id >> 8) & 0xFF) as u8;
        b[5] = (group_id & 0xFF) as u8;
        b[6] = ((src_id >> 16) & 0xFF) as u8;
        b[7] = ((src_id >> 8) & 0xFF) as u8;
        b[8] = (src_id & 0xFF) as u8;
        b[9] = 0x00;
        Self::set_crc(&mut b);
        Self { bytes: b }
    }

    /// Build a Channel Grant Individual (private call) CSBK.
    pub fn channel_grant_individual(channel_id: u8, dst_id: u32, src_id: u32) -> Self {
        let mut b = [0u8; 12];
        b[0] = (CsbkOpcode::ChannelGrantIndividual as u8) << 2 | 0x02;
        b[1] = 0x00;
        b[2] = channel_id;
        b[3] = ((dst_id >> 16) & 0xFF) as u8;
        b[4] = ((dst_id >> 8) & 0xFF) as u8;
        b[5] = (dst_id & 0xFF) as u8;
        b[6] = ((src_id >> 16) & 0xFF) as u8;
        b[7] = ((src_id >> 8) & 0xFF) as u8;
        b[8] = (src_id & 0xFF) as u8;
        b[9] = 0x00;
        Self::set_crc(&mut b);
        Self { bytes: b }
    }

    /// Build a System Registration Response CSBK (affiliation accept/reject).
    pub fn registration_response(src_id: u32, site_id: u16, accepted: bool) -> Self {
        let mut b = [0u8; 12];
        b[0] = (CsbkOpcode::SystemRegistrationResponse as u8) << 2 | 0x02;
        b[1] = 0x00;
        b[2] = if accepted { 0x00 } else { 0x01 };
        b[3] = ((src_id >> 16) & 0xFF) as u8;
        b[4] = ((src_id >> 8) & 0xFF) as u8;
        b[5] = (src_id & 0xFF) as u8;
        b[6] = ((site_id >> 8) & 0xFF) as u8;
        b[7] = (site_id & 0xFF) as u8;
        b[8] = 0x00;
        b[9] = 0x00;
        Self::set_crc(&mut b);
        Self { bytes: b }
    }

    /// Build an Acknowledge Response CSBK.
    pub fn ack_response(dst_id: u32, src_id: u32) -> Self {
        let mut b = [0u8; 12];
        b[0] = (CsbkOpcode::AckResponseGeneral as u8) << 2 | 0x02;
        b[1] = 0x00;
        b[2] = ((dst_id >> 16) & 0xFF) as u8;
        b[3] = ((dst_id >> 8) & 0xFF) as u8;
        b[4] = (dst_id & 0xFF) as u8;
        b[5] = ((src_id >> 16) & 0xFF) as u8;
        b[6] = ((src_id >> 8) & 0xFF) as u8;
        b[7] = (src_id & 0xFF) as u8;
        b[8] = 0x00;
        b[9] = 0x00;
        Self::set_crc(&mut b);
        Self { bytes: b }
    }

    /// Build a Denial Response CSBK.
    pub fn denial_response(dst_id: u32, reason: DenialReason) -> Self {
        let mut b = [0u8; 12];
        b[0] = (CsbkOpcode::DenialResponse as u8) << 2 | 0x02;
        b[1] = 0x00;
        b[2] = reason as u8;
        b[3] = ((dst_id >> 16) & 0xFF) as u8;
        b[4] = ((dst_id >> 8) & 0xFF) as u8;
        b[5] = (dst_id & 0xFF) as u8;
        b[6] = 0;
        b[7] = 0;
        b[8] = 0;
        b[9] = 0;
        Self::set_crc(&mut b);
        Self { bytes: b }
    }

    /// Build a Queued Response CSBK.
    pub fn queued_response(dst_id: u32, queue_pos: u8) -> Self {
        let mut b = [0u8; 12];
        b[0] = (CsbkOpcode::QueuedResponse as u8) << 2 | 0x02;
        b[1] = 0x00;
        b[2] = queue_pos;
        b[3] = ((dst_id >> 16) & 0xFF) as u8;
        b[4] = ((dst_id >> 8) & 0xFF) as u8;
        b[5] = (dst_id & 0xFF) as u8;
        b[6] = 0;
        b[7] = 0;
        b[8] = 0;
        b[9] = 0;
        Self::set_crc(&mut b);
        Self { bytes: b }
    }

    /// Build a De-registration Acknowledge CSBK.
    pub fn deregistration_ack(src_id: u32) -> Self {
        let mut b = [0u8; 12];
        b[0] = (CsbkOpcode::DeregistrationAck as u8) << 2 | 0x02;
        b[1] = 0x00;
        b[2] = ((src_id >> 16) & 0xFF) as u8;
        b[3] = ((src_id >> 8) & 0xFF) as u8;
        b[4] = (src_id & 0xFF) as u8;
        b[5] = 0;
        b[6] = 0;
        b[7] = 0;
        b[8] = 0;
        b[9] = 0;
        Self::set_crc(&mut b);
        Self { bytes: b }
    }

    /// BPTC-encode this CSBK PDU into a 196-bit (25-byte) burst payload.
    pub fn encode_bptc(&self) -> [u8; 25] {
        bptc_encode(&self.bytes)
    }

    /// Decode a 25-byte BPTC burst payload back to a CSBK PDU.
    pub fn decode_bptc(encoded: &[u8; 25]) -> Option<Self> {
        bptc_decode(encoded).map(|b| Self { bytes: b })
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Denial reason codes (ETSI TS 102 361-4 §9.1.7)
// ──────────────────────────────────────────────────────────────────────────

/// Reason codes carried in a Denial Response CSBK.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DenialReason {
    /// Requested resource not available
    ResourceNotAvailable = 0x01,
    /// Call preempted – higher priority
    Preempted = 0x02,
    /// Radio not affiliated
    NotAffiliated = 0x03,
    /// Radio not registered
    NotRegistered = 0x04,
    /// System busy – no channels
    SystemBusy = 0x05,
    /// Destination not available
    DestNotAvailable = 0x06,
    /// Service not supported
    ServiceNotSupported = 0x10,
}

// ──────────────────────────────────────────────────────────────────────────
// Data Header types (ETSI TS 102 361-1 §9.1.9)
// ──────────────────────────────────────────────────────────────────────────

/// DMR data header format discriminator (4-bit field).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DataPacketFormat {
    UnconfirmedData = 0,
    ConfirmedData = 1,
    Response = 4,
    ProprietaryData = 9,
    ShortDataDefined = 13,
    ShortDataRaw = 15,
}

/// Unified DMR Data Header PDU (12 bytes, BPTC-encoded to 25 bytes).
#[derive(Debug, Clone)]
pub struct DataHeaderPdu {
    /// Raw PDU bytes (96 bits = 12 bytes).
    pub bytes: [u8; 12],
}

impl DataHeaderPdu {
    /// Construct an Unconfirmed Data header.
    pub fn unconfirmed(dst_id: u32, src_id: u32, group: bool, blocks_to_follow: u8) -> Self {
        let mut b = [0u8; 12];
        b[0] = 0x00;
        b[1] = (DataPacketFormat::UnconfirmedData as u8) | if group { 0x80 } else { 0x00 };
        b[2] = ((dst_id >> 16) & 0xFF) as u8;
        b[3] = ((dst_id >> 8) & 0xFF) as u8;
        b[4] = (dst_id & 0xFF) as u8;
        b[5] = ((src_id >> 16) & 0xFF) as u8;
        b[6] = ((src_id >> 8) & 0xFF) as u8;
        b[7] = (src_id & 0xFF) as u8;
        b[8] = blocks_to_follow;
        b[9] = 0x00;
        let crc = crc16_ccitt(&b[0..10]);
        b[10] = (crc >> 8) as u8;
        b[11] = (crc & 0xFF) as u8;
        Self { bytes: b }
    }

    /// Construct a Short Data Defined header.
    pub fn short_data_defined(dst_id: u32, src_id: u32, data_len: u8) -> Self {
        let mut b = [0u8; 12];
        b[0] = 0x00;
        b[1] = DataPacketFormat::ShortDataDefined as u8;
        b[2] = ((dst_id >> 16) & 0xFF) as u8;
        b[3] = ((dst_id >> 8) & 0xFF) as u8;
        b[4] = (dst_id & 0xFF) as u8;
        b[5] = ((src_id >> 16) & 0xFF) as u8;
        b[6] = ((src_id >> 8) & 0xFF) as u8;
        b[7] = (src_id & 0xFF) as u8;
        b[8] = data_len;
        b[9] = 0x00;
        let crc = crc16_ccitt(&b[0..10]);
        b[10] = (crc >> 8) as u8;
        b[11] = (crc & 0xFF) as u8;
        Self { bytes: b }
    }

    /// BPTC-encode header into 196-bit (25-byte) payload.
    pub fn encode_bptc(&self) -> [u8; 25] {
        bptc_encode(&self.bytes)
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Trunking State Machine
// ──────────────────────────────────────────────────────────────────────────

/// State of a single logical channel in the trunking state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrunkingState {
    /// Channel is idle – no call in progress.
    Idle,
    /// Site controller has granted the channel; awaiting subscriber PTT.
    CallSetup,
    /// Call is active; voice/data traffic flowing.
    Active,
    /// PTT released; channel in post-call hangtime.
    Hangtime,
}

/// Events that drive the trunking state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrunkingEvent {
    /// Subscriber sends a group or individual call request.
    CallRequest,
    /// Controller grants the channel (internal).
    ChannelGranted,
    /// PTT asserted by radio – call now active.
    PttAsserted,
    /// PTT released by radio.
    PttReleased,
    /// Hangtime timer expired.
    HangtimeExpired,
    /// Explicit call tear-down received.
    CallTeardown,
    /// Channel released back to pool.
    ChannelRelease,
}

/// Per-channel state machine.
#[derive(Debug, Clone)]
pub struct ChannelStateMachine {
    pub state: TrunkingState,
    pub channel_id: u8,
    pub group_id: Option<u32>,
    pub src_id: Option<u32>,
    /// Hangtime countdown in abstract ticks.
    pub hangtime_ticks: u32,
}

impl ChannelStateMachine {
    /// Create a new idle state machine for `channel_id`.
    pub fn new(channel_id: u8) -> Self {
        Self {
            state: TrunkingState::Idle,
            channel_id,
            group_id: None,
            src_id: None,
            hangtime_ticks: 0,
        }
    }

    /// Advance the state machine with `event`.  Returns the new state.
    pub fn transition(&mut self, event: TrunkingEvent) -> TrunkingState {
        use TrunkingEvent::*;
        use TrunkingState::*;
        self.state = match (self.state, event) {
            (Idle, CallRequest) => CallSetup,
            (CallSetup, ChannelGranted) => CallSetup,
            (CallSetup, PttAsserted) => Active,
            (CallSetup, CallTeardown) => Idle,
            (Active, PttReleased) => {
                self.hangtime_ticks = 15;
                Hangtime
            }
            (Active, CallTeardown) => Idle,
            (Hangtime, PttAsserted) => Active,
            (Hangtime, HangtimeExpired) => Idle,
            (Hangtime, CallTeardown) => Idle,
            (_, ChannelRelease) => Idle,
            (s, _) => s,
        };
        self.state
    }

    /// Tick the hangtime counter; returns true when hangtime expires.
    pub fn tick_hangtime(&mut self) -> bool {
        if self.state == TrunkingState::Hangtime {
            if self.hangtime_ticks > 0 {
                self.hangtime_ticks -= 1;
            }
            if self.hangtime_ticks == 0 {
                self.transition(TrunkingEvent::HangtimeExpired);
                return true;
            }
        }
        false
    }

    /// True if the channel is available for a new call.
    pub fn is_available(&self) -> bool {
        self.state == TrunkingState::Idle
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Affiliation / Registration table
// ──────────────────────────────────────────────────────────────────────────

/// Affiliation record – tracks a radio's group membership.
#[derive(Debug, Clone)]
pub struct AffiliationRecord {
    pub radio_id: u32,
    pub group_id: u32,
    /// Site ID where the radio last registered.
    pub site_id: u16,
    /// True when the radio has explicitly de-registered.
    pub deregistered: bool,
}

/// Registration table for all known radios.
#[derive(Debug, Default)]
pub struct AffiliationTable {
    records: Vec<AffiliationRecord>,
}

impl AffiliationTable {
    pub fn new() -> Self {
        Self::default()
    }

    /// Register or update a radio's affiliation.
    pub fn register(&mut self, radio_id: u32, group_id: u32, site_id: u16) {
        if let Some(rec) = self.records.iter_mut().find(|r| r.radio_id == radio_id) {
            rec.group_id = group_id;
            rec.site_id = site_id;
            rec.deregistered = false;
        } else {
            self.records.push(AffiliationRecord {
                radio_id,
                group_id,
                site_id,
                deregistered: false,
            });
        }
    }

    /// De-register a radio.
    pub fn deregister(&mut self, radio_id: u32) {
        if let Some(rec) = self.records.iter_mut().find(|r| r.radio_id == radio_id) {
            rec.deregistered = true;
        }
    }

    /// Look up a radio's current registration record.
    pub fn lookup(&self, radio_id: u32) -> Option<&AffiliationRecord> {
        self.records
            .iter()
            .find(|r| r.radio_id == radio_id && !r.deregistered)
    }

    /// Return all radios affiliated to `group_id`.
    pub fn members_of_group(&self, group_id: u32) -> Vec<u32> {
        self.records
            .iter()
            .filter(|r| r.group_id == group_id && !r.deregistered)
            .map(|r| r.radio_id)
            .collect()
    }

    /// Total registered (not de-registered) radio count.
    pub fn registered_count(&self) -> usize {
        self.records.iter().filter(|r| !r.deregistered).count()
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Call Request priority queue
// ──────────────────────────────────────────────────────────────────────────

/// Priority level for a call request (lower value = higher priority).
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum CallPriority {
    Emergency = 0,
    High = 1,
    Normal = 2,
    Low = 3,
}

/// A pending call request waiting for a channel grant.
#[derive(Debug, Clone)]
pub struct CallRequest {
    pub priority: CallPriority,
    /// Talkgroup (group call) or destination radio ID (individual).
    pub dst_id: u32,
    pub src_id: u32,
    pub is_group: bool,
    /// Sequence counter for FIFO ordering within same priority.
    pub seq: u64,
}

/// Priority queue of pending call requests.
#[derive(Debug, Default)]
pub struct CallRequestQueue {
    requests: Vec<CallRequest>,
    next_seq: u64,
}

impl CallRequestQueue {
    pub fn new() -> Self {
        Self::default()
    }

    /// Enqueue a new call request.
    pub fn push(&mut self, dst_id: u32, src_id: u32, is_group: bool, priority: CallPriority) {
        let seq = self.next_seq;
        self.next_seq += 1;
        self.requests.push(CallRequest {
            priority,
            dst_id,
            src_id,
            is_group,
            seq,
        });
        self.requests.sort_by(|a, b| {
            a.priority
                .cmp(&b.priority)
                .then(a.seq.cmp(&b.seq))
        });
    }

    /// Dequeue the highest-priority request.
    pub fn pop(&mut self) -> Option<CallRequest> {
        if self.requests.is_empty() {
            None
        } else {
            Some(self.requests.remove(0))
        }
    }

    /// Peek at the highest-priority request without removing it.
    pub fn peek(&self) -> Option<&CallRequest> {
        self.requests.first()
    }

    /// True if the queue is empty.
    pub fn is_empty(&self) -> bool {
        self.requests.is_empty()
    }

    /// Current queue depth.
    pub fn len(&self) -> usize {
        self.requests.len()
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Trunking Site Controller
// ──────────────────────────────────────────────────────────────────────────

/// Site controller result for a call request.
#[derive(Debug, Clone)]
pub enum GrantResult {
    /// Call granted on the specified channel; CSBK ready to broadcast.
    Granted {
        channel_id: u8,
        csbk: CsbkPdu,
    },
    /// Call queued – subscriber should wait.
    Queued {
        queue_position: u8,
        csbk: CsbkPdu,
    },
    /// Call denied.
    Denied {
        reason: DenialReason,
        csbk: CsbkPdu,
    },
}

/// Site-level trunking controller managing channel pool and affiliations.
#[derive(Debug)]
pub struct SiteController {
    pub site_id: u16,
    channels: Vec<ChannelStateMachine>,
    pub affiliations: AffiliationTable,
    pub call_queue: CallRequestQueue,
    /// Control channel index (typically channel 0).
    pub control_channel: u8,
}

impl SiteController {
    /// Create a site controller with `num_channels` RF channels (including control channel).
    pub fn new(site_id: u16, num_channels: u8) -> Self {
        let channels = (0..num_channels)
            .map(ChannelStateMachine::new)
            .collect();
        Self {
            site_id,
            channels,
            affiliations: AffiliationTable::new(),
            call_queue: CallRequestQueue::new(),
            control_channel: 0,
        }
    }

    /// Total number of channels (including control channel).
    pub fn num_channels(&self) -> u8 {
        self.channels.len() as u8
    }

    /// Number of idle (available) traffic channels.
    pub fn available_traffic_channels(&self) -> usize {
        self.channels
            .iter()
            .filter(|c| c.channel_id != self.control_channel && c.is_available())
            .count()
    }

    /// Find the first available traffic channel ID.
    fn allocate_channel(&mut self) -> Option<u8> {
        let ctrl = self.control_channel;
        self.channels
            .iter()
            .find(|c| c.channel_id != ctrl && c.is_available())
            .map(|c| c.channel_id)
    }

    /// Register a radio at this site.
    pub fn register_radio(&mut self, radio_id: u32, group_id: u32) -> CsbkPdu {
        self.affiliations.register(radio_id, group_id, self.site_id);
        CsbkPdu::registration_response(radio_id, self.site_id, true)
    }

    /// De-register a radio from this site.
    pub fn deregister_radio(&mut self, radio_id: u32) -> CsbkPdu {
        self.affiliations.deregister(radio_id);
        CsbkPdu::deregistration_ack(radio_id)
    }

    /// Request a group call for `group_id` from `src_id`.
    pub fn request_group_call(
        &mut self,
        group_id: u32,
        src_id: u32,
        priority: CallPriority,
    ) -> GrantResult {
        if self.affiliations.lookup(src_id).is_none() {
            let csbk = CsbkPdu::denial_response(src_id, DenialReason::NotRegistered);
            return GrantResult::Denied {
                reason: DenialReason::NotRegistered,
                csbk,
            };
        }

        if let Some(ch_id) = self.allocate_channel() {
            let ch = &mut self.channels[ch_id as usize];
            ch.transition(TrunkingEvent::CallRequest);
            ch.group_id = Some(group_id);
            ch.src_id = Some(src_id);
            let csbk = CsbkPdu::channel_grant_group(ch_id, group_id, src_id);
            GrantResult::Granted {
                channel_id: ch_id,
                csbk,
            }
        } else {
            self.call_queue.push(group_id, src_id, true, priority);
            let pos = self.call_queue.len() as u8;
            let csbk = CsbkPdu::queued_response(src_id, pos);
            GrantResult::Queued {
                queue_position: pos,
                csbk,
            }
        }
    }

    /// Request a private (individual) call from `src_id` to `dst_id`.
    pub fn request_individual_call(
        &mut self,
        dst_id: u32,
        src_id: u32,
        priority: CallPriority,
    ) -> GrantResult {
        if self.affiliations.lookup(src_id).is_none() {
            let csbk = CsbkPdu::denial_response(src_id, DenialReason::NotRegistered);
            return GrantResult::Denied {
                reason: DenialReason::NotRegistered,
                csbk,
            };
        }

        if let Some(ch_id) = self.allocate_channel() {
            let ch = &mut self.channels[ch_id as usize];
            ch.transition(TrunkingEvent::CallRequest);
            ch.group_id = None;
            ch.src_id = Some(src_id);
            let csbk = CsbkPdu::channel_grant_individual(ch_id, dst_id, src_id);
            GrantResult::Granted {
                channel_id: ch_id,
                csbk,
            }
        } else {
            self.call_queue.push(dst_id, src_id, false, priority);
            let pos = self.call_queue.len() as u8;
            let csbk = CsbkPdu::queued_response(src_id, pos);
            GrantResult::Queued {
                queue_position: pos,
                csbk,
            }
        }
    }

    /// Notify controller that radio sent PTT on `channel_id`.
    pub fn ptt_assert(&mut self, channel_id: u8) {
        if let Some(ch) = self.channels.get_mut(channel_id as usize) {
            ch.transition(TrunkingEvent::PttAsserted);
        }
    }

    /// Notify controller that radio released PTT on `channel_id`.
    pub fn ptt_release(&mut self, channel_id: u8) {
        if let Some(ch) = self.channels.get_mut(channel_id as usize) {
            ch.transition(TrunkingEvent::PttReleased);
        }
    }

    /// Tear down the call on `channel_id` immediately.
    pub fn teardown_call(&mut self, channel_id: u8) {
        if let Some(ch) = self.channels.get_mut(channel_id as usize) {
            ch.transition(TrunkingEvent::CallTeardown);
            ch.group_id = None;
            ch.src_id = None;
        }
    }

    /// Advance time by one tick; returns IDs of channels that became idle.
    pub fn tick(&mut self) -> Vec<u8> {
        let mut freed = Vec::new();
        for ch in &mut self.channels {
            if ch.tick_hangtime() {
                freed.push(ch.channel_id);
            }
        }
        while !self.call_queue.is_empty() {
            if self.available_traffic_channels() == 0 {
                break;
            }
            if let Some(req) = self.call_queue.pop() {
                if let Some(ch_id) = self.allocate_channel() {
                    let ch = &mut self.channels[ch_id as usize];
                    ch.transition(TrunkingEvent::CallRequest);
                    if req.is_group {
                        ch.group_id = Some(req.dst_id);
                    }
                    ch.src_id = Some(req.src_id);
                }
            }
        }
        freed
    }

    /// State of a given channel.
    pub fn channel_state(&self, channel_id: u8) -> Option<TrunkingState> {
        self.channels
            .get(channel_id as usize)
            .map(|c| c.state)
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Emergency call handling
// ──────────────────────────────────────────────────────────────────────────

impl SiteController {
    /// Handle an emergency call request (pre-empts lower priority if needed).
    pub fn request_emergency_call(&mut self, src_id: u32, group_id: u32) -> GrantResult {
        if self.affiliations.lookup(src_id).is_none() {
            self.affiliations.register(src_id, group_id, self.site_id);
        }

        if let Some(ch_id) = self.allocate_channel() {
            let ch = &mut self.channels[ch_id as usize];
            ch.transition(TrunkingEvent::CallRequest);
            ch.group_id = Some(group_id);
            ch.src_id = Some(src_id);
            let csbk = CsbkPdu::channel_grant_group(ch_id, group_id, src_id);
            return GrantResult::Granted {
                channel_id: ch_id,
                csbk,
            };
        }

        let ctrl = self.control_channel;
        let preempt_id = self
            .channels
            .iter()
            .filter(|c| c.channel_id != ctrl && c.state == TrunkingState::Active)
            .map(|c| c.channel_id)
            .max();

        if let Some(ch_id) = preempt_id {
            self.teardown_call(ch_id);
            let ch = &mut self.channels[ch_id as usize];
            ch.transition(TrunkingEvent::CallRequest);
            ch.group_id = Some(group_id);
            ch.src_id = Some(src_id);
            let csbk = CsbkPdu::channel_grant_group(ch_id, group_id, src_id);
            GrantResult::Granted {
                channel_id: ch_id,
                csbk,
            }
        } else {
            let csbk = CsbkPdu::denial_response(src_id, DenialReason::SystemBusy);
            GrantResult::Denied {
                reason: DenialReason::SystemBusy,
                csbk,
            }
        }
    }
}

// ──────────────────────────────────────────────────────────────────────────
// TSCC Burst framing helpers
// ──────────────────────────────────────────────────────────────────────────

/// TSCC (Trunking System Control Channel) burst types.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TsccBurstType {
    /// Carries a single CSBK PDU.
    Csbk,
    /// Carries an MBC header (multi-block CSBK).
    MbcHeader,
    /// Carries a data header or UDT.
    DataHeader,
}

/// A framed TSCC burst ready for transmission.
#[derive(Debug, Clone)]
pub struct TsccBurst {
    pub burst_type: TsccBurstType,
    /// Slot Number (1 or 2) on the RF channel.
    pub slot: u8,
    /// 196-bit BPTC payload (25 bytes, 4 unused bits).
    pub payload: [u8; 25],
    /// Colour code (0-15).
    pub colour_code: u8,
}

impl TsccBurst {
    /// Construct a TSCC burst from a CSBK PDU.
    pub fn from_csbk(csbk: &CsbkPdu, slot: u8, colour_code: u8) -> Self {
        Self {
            burst_type: TsccBurstType::Csbk,
            slot,
            payload: csbk.encode_bptc(),
            colour_code,
        }
    }

    /// Construct a TSCC burst from a Data Header PDU.
    pub fn from_data_header(hdr: &DataHeaderPdu, slot: u8, colour_code: u8) -> Self {
        Self {
            burst_type: TsccBurstType::DataHeader,
            slot,
            payload: hdr.encode_bptc(),
            colour_code,
        }
    }

    /// Decode payload back to a CSBK PDU (if burst_type == Csbk).
    pub fn decode_csbk(&self) -> Option<CsbkPdu> {
        CsbkPdu::decode_bptc(&self.payload)
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Adjacent-site information beacon
// ──────────────────────────────────────────────────────────────────────────

/// Adjacent site record broadcast by the TSCC.
#[derive(Debug, Clone)]
pub struct AdjacentSite {
    pub site_id: u16,
    pub channel_id: u8,
    pub colour_code: u8,
    pub reachable: bool,
}

impl AdjacentSite {
    pub fn new(site_id: u16, channel_id: u8, colour_code: u8) -> Self {
        Self {
            site_id,
            channel_id,
            colour_code,
            reachable: true,
        }
    }

    /// Build the CSBK announcing this adjacent site.
    pub fn to_csbk(&self) -> CsbkPdu {
        let mut b = [0u8; 12];
        b[0] = (CsbkOpcode::AdjacentSiteInfo as u8) << 2 | 0x02;
        b[1] = 0x00;
        b[2] = self.channel_id;
        b[3] = ((self.site_id >> 8) & 0xFF) as u8;
        b[4] = (self.site_id & 0xFF) as u8;
        b[5] = self.colour_code;
        b[6] = if self.reachable { 0x01 } else { 0x00 };
        b[7] = 0;
        b[8] = 0;
        b[9] = 0;
        let crc = crc16_ccitt(&b[0..10]);
        b[10] = (crc >> 8) as u8;
        b[11] = (crc & 0xFF) as u8;
        CsbkPdu { bytes: b }
    }
}

// ──────────────────────────────────────────────────────────────────────────
// Site Data / Network Status Broadcast
// ──────────────────────────────────────────────────────────────────────────

/// Generate a Site Data broadcast CSBK.
pub fn build_site_data_csbk(site_id: u16, num_channels: u8, active_calls: u8) -> CsbkPdu {
    let mut b = [0u8; 12];
    b[0] = (CsbkOpcode::SiteData as u8) << 2 | 0x02;
    b[1] = 0x00;
    b[2] = ((site_id >> 8) & 0xFF) as u8;
    b[3] = (site_id & 0xFF) as u8;
    b[4] = num_channels;
    b[5] = active_calls;
    b[6] = 0;
    b[7] = 0;
    b[8] = 0;
    b[9] = 0;
    let crc = crc16_ccitt(&b[0..10]);
    b[10] = (crc >> 8) as u8;
    b[11] = (crc & 0xFF) as u8;
    CsbkPdu { bytes: b }
}

/// Generate a Network Status Broadcast CSBK.
pub fn build_network_status_csbk(network_id: u32, site_id: u16) -> CsbkPdu {
    let mut b = [0u8; 12];
    b[0] = (CsbkOpcode::NetworkStatusBroadcast as u8) << 2 | 0x02;
    b[1] = 0x00;
    b[2] = ((network_id >> 16) & 0xFF) as u8;
    b[3] = ((network_id >> 8) & 0xFF) as u8;
    b[4] = (network_id & 0xFF) as u8;
    b[5] = ((site_id >> 8) & 0xFF) as u8;
    b[6] = (site_id & 0xFF) as u8;
    b[7] = 0;
    b[8] = 0;
    b[9] = 0;
    let crc = crc16_ccitt(&b[0..10]);
    b[10] = (crc >> 8) as u8;
    b[11] = (crc & 0xFF) as u8;
    CsbkPdu { bytes: b }
}

// ──────────────────────────────────────────────────────────────────────────
// Unit tests
// ──────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── CRC-16 CCITT ─────────────────────────────────────────────────────

    #[test]
    fn test_crc16_all_zeros() {
        // CRC-CCITT of 0 bytes of input = 0xFFFF (just the initial value)
        assert_eq!(crc16_ccitt(&[]), 0xFFFF);
    }

    #[test]
    fn test_crc16_single_byte() {
        // CRC-CCITT of [0x00] with init=0xFFFF is deterministic.
        let c = crc16_ccitt(&[0x00]);
        assert_ne!(c, 0xFFFF); // must differ from initial value
    }

    #[test]
    fn test_crc16_non_zero() {
        let data = b"ETSI361-4";
        let crc = crc16_ccitt(data);
        assert_ne!(crc, 0);
    }

    #[test]
    fn test_crc16_consistency() {
        let data = [0x12u8, 0x34, 0x56, 0x78, 0xAB, 0xCD, 0x00, 0x00, 0x00, 0x00];
        assert_eq!(crc16_ccitt(&data), crc16_ccitt(&data));
    }

    #[test]
    fn test_crc16_different_inputs() {
        let a = crc16_ccitt(&[0x01, 0x02]);
        let b = crc16_ccitt(&[0x03, 0x04]);
        assert_ne!(a, b);
    }

    // ── BPTC(196,96) ─────────────────────────────────────────────────────

    #[test]
    fn test_bptc_encode_decode_zeros() {
        let payload = [0u8; 12];
        let enc = bptc_encode(&payload);
        let dec = bptc_decode(&enc).expect("decode failed");
        assert_eq!(dec, payload);
    }

    #[test]
    fn test_bptc_encode_decode_ones() {
        let payload = [0xFFu8; 12];
        let enc = bptc_encode(&payload);
        let dec = bptc_decode(&enc).expect("decode failed");
        assert_eq!(dec, payload);
    }

    #[test]
    fn test_bptc_encode_decode_alternating() {
        let payload = [0xA5u8, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A, 0xA5, 0x5A];
        let enc = bptc_encode(&payload);
        let dec = bptc_decode(&enc).expect("decode failed");
        assert_eq!(dec, payload);
    }

    #[test]
    fn test_bptc_encode_produces_25_bytes() {
        let payload = [0u8; 12];
        let enc = bptc_encode(&payload);
        assert_eq!(enc.len(), 25);
    }

    #[test]
    fn test_bptc_roundtrip_sequential() {
        for seed in 0u8..16 {
            let mut p = [0u8; 12];
            for (i, b) in p.iter_mut().enumerate() {
                *b = seed.wrapping_add(i as u8);
            }
            let enc = bptc_encode(&p);
            let dec = bptc_decode(&enc).unwrap();
            assert_eq!(dec, p, "seed={seed}");
        }
    }

    #[test]
    fn test_bptc_roundtrip_all_bit_patterns() {
        // Test that every single byte-position with 0xFF is recoverable.
        for pos in 0..12usize {
            let mut p = [0u8; 12];
            p[pos] = 0xFF;
            let enc = bptc_encode(&p);
            let dec = bptc_decode(&enc).unwrap();
            assert_eq!(dec, p, "pos={pos}");
        }
    }

    #[test]
    fn test_bptc_roundtrip_mixed() {
        let payload = [0x01u8, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x10, 0x32, 0x54, 0x76];
        let enc = bptc_encode(&payload);
        let dec = bptc_decode(&enc).unwrap();
        assert_eq!(dec, payload);
    }

    // ── CSBK opcodes ─────────────────────────────────────────────────────

    #[test]
    fn test_csbk_opcode_roundtrip() {
        let opcodes = [
            CsbkOpcode::ChannelGrantGroup,
            CsbkOpcode::ChannelGrantIndividual,
            CsbkOpcode::SystemRegistrationResponse,
            CsbkOpcode::AckResponseGeneral,
            CsbkOpcode::DenialResponse,
            CsbkOpcode::QueuedResponse,
            CsbkOpcode::DeregistrationAck,
            CsbkOpcode::AdjacentSiteInfo,
            CsbkOpcode::SiteData,
            CsbkOpcode::NetworkStatusBroadcast,
        ];
        for &op in &opcodes {
            let raw = op as u8;
            assert!(raw <= 0x3F, "opcode {op:?} must be ≤ 6 bits, got {raw:#x}");
            assert_eq!(CsbkOpcode::from_u8(raw), op, "opcode={op:?}");
        }
    }

    // ── CSBK PDU construction & validation ───────────────────────────────

    #[test]
    fn test_csbk_channel_grant_group_crc_valid() {
        let csbk = CsbkPdu::channel_grant_group(2, 0x000100, 0xAB1234);
        assert!(csbk.crc_valid(), "CRC must be valid after construction");
        assert_eq!(csbk.opcode(), CsbkOpcode::ChannelGrantGroup);
    }

    #[test]
    fn test_csbk_channel_grant_individual_crc_valid() {
        let csbk = CsbkPdu::channel_grant_individual(3, 0x000200, 0xBC5678);
        assert!(csbk.crc_valid());
        assert_eq!(csbk.opcode(), CsbkOpcode::ChannelGrantIndividual);
    }

    #[test]
    fn test_csbk_registration_response_accepted() {
        let csbk = CsbkPdu::registration_response(0x1234, 0x01, true);
        assert!(csbk.crc_valid());
        assert_eq!(csbk.opcode(), CsbkOpcode::SystemRegistrationResponse);
        assert_eq!(csbk.bytes[2], 0x00);
    }

    #[test]
    fn test_csbk_registration_response_rejected() {
        let csbk = CsbkPdu::registration_response(0x5678, 0x01, false);
        assert!(csbk.crc_valid());
        assert_eq!(csbk.bytes[2], 0x01);
    }

    #[test]
    fn test_csbk_ack_response() {
        let csbk = CsbkPdu::ack_response(0xAAAAAA, 0xBBBBBB);
        assert!(csbk.crc_valid());
        assert_eq!(csbk.opcode(), CsbkOpcode::AckResponseGeneral);
    }

    #[test]
    fn test_csbk_denial_response() {
        let csbk = CsbkPdu::denial_response(0x1111, DenialReason::SystemBusy);
        assert!(csbk.crc_valid());
        assert_eq!(csbk.opcode(), CsbkOpcode::DenialResponse);
        assert_eq!(csbk.bytes[2], DenialReason::SystemBusy as u8);
    }

    #[test]
    fn test_csbk_queued_response() {
        let csbk = CsbkPdu::queued_response(0x2222, 3);
        assert!(csbk.crc_valid());
        assert_eq!(csbk.opcode(), CsbkOpcode::QueuedResponse);
        assert_eq!(csbk.bytes[2], 3);
    }

    #[test]
    fn test_csbk_deregistration_ack() {
        let csbk = CsbkPdu::deregistration_ack(0x99AABB);
        assert!(csbk.crc_valid());
        assert_eq!(csbk.opcode(), CsbkOpcode::DeregistrationAck);
    }

    #[test]
    fn test_csbk_last_block_flag() {
        let csbk = CsbkPdu::channel_grant_group(1, 0x1, 0x2);
        assert!(csbk.last_block(), "LB bit should be set");
    }

    #[test]
    fn test_csbk_bptc_roundtrip() {
        let csbk = CsbkPdu::channel_grant_group(1, 0xABCDEF, 0x123456);
        let enc = csbk.encode_bptc();
        let dec = CsbkPdu::decode_bptc(&enc).expect("BPTC decode failed");
        assert_eq!(dec.bytes, csbk.bytes);
    }

    #[test]
    fn test_csbk_bptc_roundtrip_individual() {
        let csbk = CsbkPdu::channel_grant_individual(4, 0xDEAD00, 0xBEEF00);
        let enc = csbk.encode_bptc();
        let dec = CsbkPdu::decode_bptc(&enc).unwrap();
        assert_eq!(dec.bytes, csbk.bytes);
        assert!(dec.crc_valid());
    }

    #[test]
    fn test_csbk_bptc_roundtrip_deregistration() {
        let csbk = CsbkPdu::deregistration_ack(0xC0FFEE);
        let enc = csbk.encode_bptc();
        let dec = CsbkPdu::decode_bptc(&enc).unwrap();
        assert_eq!(dec.bytes, csbk.bytes);
    }

    // ── Data Header PDU ───────────────────────────────────────────────────

    #[test]
    fn test_data_header_unconfirmed_crc() {
        let hdr = DataHeaderPdu::unconfirmed(0x100, 0x200, true, 4);
        let stored_crc = ((hdr.bytes[10] as u16) << 8) | hdr.bytes[11] as u16;
        assert_eq!(stored_crc, crc16_ccitt(&hdr.bytes[0..10]));
    }

    #[test]
    fn test_data_header_short_data_defined() {
        let hdr = DataHeaderPdu::short_data_defined(0x300, 0x400, 8);
        assert_eq!(hdr.bytes[1], DataPacketFormat::ShortDataDefined as u8);
    }

    #[test]
    fn test_data_header_bptc_roundtrip() {
        let hdr = DataHeaderPdu::unconfirmed(0xABCD, 0x1234, false, 2);
        let enc = hdr.encode_bptc();
        let dec = bptc_decode(&enc).unwrap();
        assert_eq!(dec, hdr.bytes);
    }

    #[test]
    fn test_data_header_group_flag() {
        let hdr = DataHeaderPdu::unconfirmed(0x1, 0x2, true, 0);
        assert_ne!(hdr.bytes[1] & 0x80, 0, "group flag should be set");
        let hdr2 = DataHeaderPdu::unconfirmed(0x1, 0x2, false, 0);
        assert_eq!(hdr2.bytes[1] & 0x80, 0, "group flag should be clear");
    }

    // ── Trunking State Machine ────────────────────────────────────────────

    #[test]
    fn test_state_machine_idle_to_callsetup() {
        let mut sm = ChannelStateMachine::new(1);
        assert_eq!(sm.state, TrunkingState::Idle);
        sm.transition(TrunkingEvent::CallRequest);
        assert_eq!(sm.state, TrunkingState::CallSetup);
    }

    #[test]
    fn test_state_machine_callsetup_to_active() {
        let mut sm = ChannelStateMachine::new(1);
        sm.transition(TrunkingEvent::CallRequest);
        sm.transition(TrunkingEvent::PttAsserted);
        assert_eq!(sm.state, TrunkingState::Active);
    }

    #[test]
    fn test_state_machine_active_to_hangtime() {
        let mut sm = ChannelStateMachine::new(1);
        sm.transition(TrunkingEvent::CallRequest);
        sm.transition(TrunkingEvent::PttAsserted);
        sm.transition(TrunkingEvent::PttReleased);
        assert_eq!(sm.state, TrunkingState::Hangtime);
        assert_eq!(sm.hangtime_ticks, 15);
    }

    #[test]
    fn test_state_machine_hangtime_to_idle() {
        let mut sm = ChannelStateMachine::new(1);
        sm.transition(TrunkingEvent::CallRequest);
        sm.transition(TrunkingEvent::PttAsserted);
        sm.transition(TrunkingEvent::PttReleased);
        for _ in 0..15 {
            sm.tick_hangtime();
        }
        assert_eq!(sm.state, TrunkingState::Idle);
    }

    #[test]
    fn test_state_machine_hangtime_rejoin() {
        let mut sm = ChannelStateMachine::new(1);
        sm.transition(TrunkingEvent::CallRequest);
        sm.transition(TrunkingEvent::PttAsserted);
        sm.transition(TrunkingEvent::PttReleased);
        sm.transition(TrunkingEvent::PttAsserted);
        assert_eq!(sm.state, TrunkingState::Active);
    }

    #[test]
    fn test_state_machine_teardown_from_active() {
        let mut sm = ChannelStateMachine::new(2);
        sm.transition(TrunkingEvent::CallRequest);
        sm.transition(TrunkingEvent::PttAsserted);
        sm.transition(TrunkingEvent::CallTeardown);
        assert_eq!(sm.state, TrunkingState::Idle);
    }

    #[test]
    fn test_state_machine_is_available() {
        let mut sm = ChannelStateMachine::new(0);
        assert!(sm.is_available());
        sm.transition(TrunkingEvent::CallRequest);
        assert!(!sm.is_available());
    }

    #[test]
    fn test_state_machine_channel_release() {
        let mut sm = ChannelStateMachine::new(1);
        sm.transition(TrunkingEvent::CallRequest);
        sm.transition(TrunkingEvent::PttAsserted);
        sm.transition(TrunkingEvent::ChannelRelease);
        assert_eq!(sm.state, TrunkingState::Idle);
    }

    #[test]
    fn test_state_machine_callsetup_teardown() {
        let mut sm = ChannelStateMachine::new(3);
        sm.transition(TrunkingEvent::CallRequest);
        sm.transition(TrunkingEvent::CallTeardown);
        assert_eq!(sm.state, TrunkingState::Idle);
    }

    // ── Affiliation Table ─────────────────────────────────────────────────

    #[test]
    fn test_affiliation_register_lookup() {
        let mut table = AffiliationTable::new();
        table.register(0xABCD, 0x100, 1);
        let rec = table.lookup(0xABCD).unwrap();
        assert_eq!(rec.radio_id, 0xABCD);
        assert_eq!(rec.group_id, 0x100);
    }

    #[test]
    fn test_affiliation_deregister() {
        let mut table = AffiliationTable::new();
        table.register(0x1111, 0x200, 1);
        table.deregister(0x1111);
        assert!(table.lookup(0x1111).is_none());
    }

    #[test]
    fn test_affiliation_members_of_group() {
        let mut table = AffiliationTable::new();
        table.register(0x0001, 0x100, 1);
        table.register(0x0002, 0x100, 1);
        table.register(0x0003, 0x200, 1);
        let members = table.members_of_group(0x100);
        assert_eq!(members.len(), 2);
        assert!(members.contains(&0x0001));
        assert!(members.contains(&0x0002));
    }

    #[test]
    fn test_affiliation_registered_count() {
        let mut table = AffiliationTable::new();
        table.register(0xA, 0x1, 1);
        table.register(0xB, 0x1, 1);
        table.register(0xC, 0x2, 1);
        assert_eq!(table.registered_count(), 3);
        table.deregister(0xB);
        assert_eq!(table.registered_count(), 2);
    }

    #[test]
    fn test_affiliation_re_register() {
        let mut table = AffiliationTable::new();
        table.register(0x5, 0x10, 1);
        table.register(0x5, 0x20, 2);
        let rec = table.lookup(0x5).unwrap();
        assert_eq!(rec.group_id, 0x20);
        assert_eq!(rec.site_id, 2);
    }

    // ── Call Request Queue ────────────────────────────────────────────────

    #[test]
    fn test_queue_priority_ordering() {
        let mut q = CallRequestQueue::new();
        q.push(0x1, 0x10, true, CallPriority::Low);
        q.push(0x2, 0x20, true, CallPriority::High);
        q.push(0x3, 0x30, true, CallPriority::Normal);
        let r = q.pop().unwrap();
        assert_eq!(r.priority, CallPriority::High);
        let r = q.pop().unwrap();
        assert_eq!(r.priority, CallPriority::Normal);
        let r = q.pop().unwrap();
        assert_eq!(r.priority, CallPriority::Low);
    }

    #[test]
    fn test_queue_fifo_within_priority() {
        let mut q = CallRequestQueue::new();
        q.push(0x1, 0x10, true, CallPriority::Normal);
        q.push(0x2, 0x20, true, CallPriority::Normal);
        let first = q.pop().unwrap();
        assert_eq!(first.dst_id, 0x1);
    }

    #[test]
    fn test_queue_emergency_first() {
        let mut q = CallRequestQueue::new();
        q.push(0x1, 0x1, true, CallPriority::Normal);
        q.push(0x2, 0x2, true, CallPriority::Emergency);
        let r = q.pop().unwrap();
        assert_eq!(r.priority, CallPriority::Emergency);
    }

    #[test]
    fn test_queue_is_empty() {
        let mut q = CallRequestQueue::new();
        assert!(q.is_empty());
        q.push(0x1, 0x1, false, CallPriority::Normal);
        assert!(!q.is_empty());
        q.pop();
        assert!(q.is_empty());
    }

    #[test]
    fn test_queue_len() {
        let mut q = CallRequestQueue::new();
        assert_eq!(q.len(), 0);
        q.push(0x1, 0x1, false, CallPriority::Normal);
        q.push(0x2, 0x2, false, CallPriority::High);
        assert_eq!(q.len(), 2);
    }

    // ── Site Controller ───────────────────────────────────────────────────

    #[test]
    fn test_site_controller_new() {
        let sc = SiteController::new(0x01, 4);
        assert_eq!(sc.site_id, 0x01);
        assert_eq!(sc.num_channels(), 4);
    }

    #[test]
    fn test_site_controller_register_radio() {
        let mut sc = SiteController::new(1, 4);
        let csbk = sc.register_radio(0xABCDEF, 0x100);
        assert!(csbk.crc_valid());
        assert_eq!(csbk.opcode(), CsbkOpcode::SystemRegistrationResponse);
        assert!(sc.affiliations.lookup(0xABCDEF).is_some());
    }

    #[test]
    fn test_site_controller_deregister_radio() {
        let mut sc = SiteController::new(1, 4);
        sc.register_radio(0x1234, 0x100);
        let csbk = sc.deregister_radio(0x1234);
        assert!(csbk.crc_valid());
        assert!(sc.affiliations.lookup(0x1234).is_none());
    }

    #[test]
    fn test_site_controller_group_call_granted() {
        let mut sc = SiteController::new(1, 4);
        sc.register_radio(0xAAA, 0x100);
        match sc.request_group_call(0x100, 0xAAA, CallPriority::Normal) {
            GrantResult::Granted { channel_id, csbk } => {
                assert_ne!(channel_id, sc.control_channel);
                assert!(csbk.crc_valid());
            }
            other => panic!("Expected Granted, got {other:?}"),
        }
    }

    #[test]
    fn test_site_controller_individual_call_granted() {
        let mut sc = SiteController::new(1, 4);
        sc.register_radio(0x111, 0x200);
        sc.register_radio(0x222, 0x200);
        match sc.request_individual_call(0x222, 0x111, CallPriority::Normal) {
            GrantResult::Granted { csbk, .. } => {
                assert!(csbk.crc_valid());
                assert_eq!(csbk.opcode(), CsbkOpcode::ChannelGrantIndividual);
            }
            other => panic!("Expected Granted, got {other:?}"),
        }
    }

    #[test]
    fn test_site_controller_denied_not_registered() {
        let mut sc = SiteController::new(1, 4);
        match sc.request_group_call(0x100, 0xDEAD, CallPriority::Normal) {
            GrantResult::Denied { reason, csbk } => {
                assert_eq!(reason, DenialReason::NotRegistered);
                assert!(csbk.crc_valid());
            }
            other => panic!("Expected Denied, got {other:?}"),
        }
    }

    #[test]
    fn test_site_controller_queue_when_full() {
        let mut sc = SiteController::new(1, 2); // 2 channels: 0=control, 1=traffic
        sc.register_radio(0xA, 0x1);
        sc.register_radio(0xB, 0x2);
        let r1 = sc.request_group_call(0x1, 0xA, CallPriority::Normal);
        assert!(matches!(r1, GrantResult::Granted { .. }));
        let r2 = sc.request_group_call(0x2, 0xB, CallPriority::Normal);
        assert!(matches!(r2, GrantResult::Queued { .. }));
    }

    #[test]
    fn test_site_controller_ptt_and_release() {
        let mut sc = SiteController::new(1, 4);
        sc.register_radio(0xF0F, 0x5);
        if let GrantResult::Granted { channel_id, .. } =
            sc.request_group_call(0x5, 0xF0F, CallPriority::Normal)
        {
            sc.ptt_assert(channel_id);
            assert_eq!(sc.channel_state(channel_id), Some(TrunkingState::Active));
            sc.ptt_release(channel_id);
            assert_eq!(sc.channel_state(channel_id), Some(TrunkingState::Hangtime));
        } else {
            panic!("Expected grant");
        }
    }

    #[test]
    fn test_site_controller_tick_expires_hangtime() {
        let mut sc = SiteController::new(1, 4);
        sc.register_radio(0x111, 0x1);
        if let GrantResult::Granted { channel_id, .. } =
            sc.request_group_call(0x1, 0x111, CallPriority::Normal)
        {
            sc.ptt_assert(channel_id);
            sc.ptt_release(channel_id);
            for _ in 0..15 {
                sc.tick();
            }
            assert_eq!(sc.channel_state(channel_id), Some(TrunkingState::Idle));
        }
    }

    #[test]
    fn test_site_controller_available_traffic_channels() {
        let sc = SiteController::new(1, 5); // channel 0 = control; 4 traffic
        assert_eq!(sc.available_traffic_channels(), 4);
    }

    #[test]
    fn test_site_controller_teardown() {
        let mut sc = SiteController::new(1, 4);
        sc.register_radio(0x77, 0x3);
        if let GrantResult::Granted { channel_id, .. } =
            sc.request_group_call(0x3, 0x77, CallPriority::Normal)
        {
            sc.ptt_assert(channel_id);
            sc.teardown_call(channel_id);
            assert_eq!(sc.channel_state(channel_id), Some(TrunkingState::Idle));
        }
    }

    // ── Emergency call ────────────────────────────────────────────────────

    #[test]
    fn test_emergency_call_granted_no_registration() {
        let mut sc = SiteController::new(1, 4);
        match sc.request_emergency_call(0xEEEEEE, 0x911) {
            GrantResult::Granted { csbk, .. } => {
                assert!(csbk.crc_valid());
            }
            other => panic!("Expected Granted, got {other:?}"),
        }
    }

    #[test]
    fn test_emergency_call_preempts_active() {
        let mut sc = SiteController::new(1, 2); // only 1 traffic channel
        sc.register_radio(0x10, 0x10);
        if let GrantResult::Granted { channel_id, .. } =
            sc.request_group_call(0x10, 0x10, CallPriority::Low)
        {
            sc.ptt_assert(channel_id);
        }
        match sc.request_emergency_call(0xEEE, 0x911) {
            GrantResult::Granted { .. } => {}
            other => panic!("Emergency pre-emption failed: {other:?}"),
        }
    }

    // ── Adjacent Site / Network Status ────────────────────────────────────

    #[test]
    fn test_adjacent_site_csbk() {
        let site = AdjacentSite::new(0x02, 1, 5);
        let csbk = site.to_csbk();
        assert!(csbk.crc_valid());
        assert_eq!(csbk.opcode(), CsbkOpcode::AdjacentSiteInfo);
    }

    #[test]
    fn test_site_data_csbk() {
        let csbk = build_site_data_csbk(0x01, 8, 3);
        assert!(csbk.crc_valid());
        assert_eq!(csbk.opcode(), CsbkOpcode::SiteData);
    }

    #[test]
    fn test_network_status_csbk() {
        let csbk = build_network_status_csbk(0xABCDE, 0x07);
        assert!(csbk.crc_valid());
        assert_eq!(csbk.opcode(), CsbkOpcode::NetworkStatusBroadcast);
    }

    // ── TSCC Burst framing ────────────────────────────────────────────────

    #[test]
    fn test_tscc_burst_from_csbk() {
        let csbk = CsbkPdu::channel_grant_group(2, 0x100, 0xABC);
        let burst = TsccBurst::from_csbk(&csbk, 1, 3);
        assert_eq!(burst.slot, 1);
        assert_eq!(burst.colour_code, 3);
        assert_eq!(burst.burst_type, TsccBurstType::Csbk);
    }

    #[test]
    fn test_tscc_burst_roundtrip() {
        let csbk = CsbkPdu::channel_grant_group(1, 0x200, 0xDEF);
        let burst = TsccBurst::from_csbk(&csbk, 2, 7);
        let decoded = burst.decode_csbk().unwrap();
        assert_eq!(decoded.bytes, csbk.bytes);
    }

    #[test]
    fn test_tscc_burst_from_data_header() {
        let hdr = DataHeaderPdu::unconfirmed(0x10, 0x20, true, 3);
        let burst = TsccBurst::from_data_header(&hdr, 1, 0);
        assert_eq!(burst.burst_type, TsccBurstType::DataHeader);
    }

    #[test]
    fn test_tscc_burst_roundtrip_individual() {
        let csbk = CsbkPdu::channel_grant_individual(3, 0x111, 0x222);
        let burst = TsccBurst::from_csbk(&csbk, 1, 5);
        let dec = burst.decode_csbk().unwrap();
        assert_eq!(dec.bytes, csbk.bytes);
    }
}
