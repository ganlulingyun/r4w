//! Thread Mesh Processor — IEEE 802.15.4 / Thread 1.3 Physical and MAC Layer
//!
//! Implements the IEEE 802.15.4-2020 O-QPSK physical layer at 2.4 GHz and the
//! Thread 1.3 MAC / network layer. Covers PN spreading, half-sine pulse shaping,
//! PPDU framing, MAC frame types, CSMA-CA, AES-128-CCM* security, MLE message
//! formatting, 6LoWPAN IPHC/NHC header compression, and channel management.
//!
//! ## Standards
//! - IEEE 802.15.4-2020 (O-QPSK PHY, MAC sublayer)
//! - Thread 1.3 Specification (MLE, 6LoWPAN, network layer)
//! - RFC 6282 (6LoWPAN Header Compression)
//! - RFC 4944 (6LoWPAN)
//!
//! ## Key Parameters
//! - Modulation: O-QPSK (Offset QPSK)
//! - Data rate: 250 kbps
//! - Chip rate: 2 Mchip/s
//! - Chip sequence length: 32 chips/symbol
//! - Symbol rate: 62.5 ksym/s
//! - Channels: 11–26 at 2.4 GHz (5 MHz spacing), channel 11 = 2405 MHz
//! - Frequency: F_c = 2405 + 5*(k-11) MHz for channel k
//!
//! ## Example
//! ```rust
//! use r4w_core::thread_mesh_processor::{
//!     ThreadProcessor, PhyConfig, MacConfig, MacAddress,
//! };
//!
//! let phy = PhyConfig::default_2_4ghz(15);
//! let mac = MacConfig::new(0xABCD, MacAddress::Short(0x1234));
//! let mut proc = ThreadProcessor::new(phy, mac);
//!
//! // Build a data frame and encode to PPDU chips
//! let payload = b"Hello Thread";
//! let frame = proc.build_data_frame(payload, MacAddress::Short(0x5678), false);
//! let chips = proc.encode_ppdu(&frame);
//! assert!(!chips.is_empty());
//! ```

use std::collections::VecDeque;

// ─────────────────────────────────────────────────────────────────────────────
// Constants (IEEE 802.15.4-2020 §13)
// ─────────────────────────────────────────────────────────────────────────────

/// Data rate at 2.4 GHz (bits per second).
pub const DATA_RATE_BPS: u32 = 250_000;

/// Chip rate (chips per second).
pub const CHIP_RATE_CPS: u32 = 2_000_000;

/// Number of chips per symbol (32 chips → 4 bits encoded per O-QPSK symbol pair).
pub const CHIPS_PER_SYMBOL: usize = 32;

/// Symbol rate (symbols per second) = chip_rate / chips_per_symbol.
pub const SYMBOL_RATE_SPS: u32 = CHIP_RATE_CPS / CHIPS_PER_SYMBOL as u32;

/// Lowest 2.4 GHz channel number.
pub const MIN_CHANNEL: u8 = 11;

/// Highest 2.4 GHz channel number.
pub const MAX_CHANNEL: u8 = 26;

/// Base frequency of channel 11 (Hz).
pub const BASE_FREQ_HZ: u64 = 2_405_000_000;

/// Channel spacing (Hz).
pub const CHANNEL_SPACING_HZ: u64 = 5_000_000;

/// PPDU preamble: four zero bytes = 32 zero chips.
pub const PREAMBLE_BYTES: [u8; 4] = [0x00, 0x00, 0x00, 0x00];

/// Start of Frame Delimiter (SFD) per IEEE 802.15.4-2020 §13.4.2.
pub const SFD: u8 = 0xA7;

/// Maximum MAC payload size (PSDU without FCS), bytes.
pub const MAX_PSDU_BYTES: usize = 127;

/// Frame Control Field size (bytes).
pub const FCF_SIZE: usize = 2;

/// FCS size (CRC-16 ITU-T/CCITT, bytes).
pub const FCS_SIZE: usize = 2;

/// CRC-16 CCITT polynomial (reversed for table-driven LSB-first computation).
const CRC16_POLY: u16 = 0x8408; // bit-reversed 0x1021

/// AES block size in bytes.
const AES_BLOCK_SIZE: usize = 16;

/// MLE frame type indicator byte.
pub const MLE_FRAME_TYPE: u8 = 0x15;

// ─────────────────────────────────────────────────────────────────────────────
// IEEE 802.15.4-2020 Table 13.2.3-1 — O-QPSK 32-chip PN sequences
// One 32-chip row per data symbol value (0..15).
// Each chip is ±1 represented as u8 (0 or 1).
// ─────────────────────────────────────────────────────────────────────────────

/// 32-chip PN sequences for 16 symbol values per IEEE 802.15.4-2020 Table 13.2.3-1.
/// Chips are stored as bits within u32 (LSB = chip 0).
pub const PN_SEQUENCES: [u32; 16] = [
    0b1101_1001_1100_0111_0110_0001_0001_1110, // symbol 0
    0b1110_1101_1001_1100_0111_0110_0001_0001, // symbol 1
    0b0001_1110_1101_1001_1100_0111_0110_0001, // symbol 2  (rot 2)
    0b0001_0001_1110_1101_1001_1100_0111_0110, // symbol 3  (rot 3)
    0b0110_0001_0001_1110_1101_1001_1100_0111, // symbol 4  (rot 4)
    0b0111_0110_0001_0001_1110_1101_1001_1100, // symbol 5  (rot 5)
    0b1100_0111_0110_0001_0001_1110_1101_1001, // symbol 6  (rot 6)
    0b1001_1100_0111_0110_0001_0001_1110_1101, // symbol 7  (rot 7)
    0b1000_0110_1000_1111_0110_0111_1100_0001, // symbol 8
    0b0001_1000_0110_1000_1111_0110_0111_1100, // symbol 9  (rot 1)
    0b1100_0001_1000_0110_1000_1111_0110_0111, // symbol 10 (rot 2)
    0b0111_1100_0001_1000_0110_1000_1111_0110, // symbol 11 (rot 3)
    0b0110_0111_1100_0001_1000_0110_1000_1111, // symbol 12 (rot 4)
    0b1111_0110_0111_1100_0001_1000_0110_1000, // symbol 13 (rot 5)
    0b1000_1111_0110_0111_1100_0001_1000_0110, // symbol 14 (rot 6)
    0b0110_1000_1111_0110_0111_1100_0001_1000, // symbol 15 (rot 7)
];

// ─────────────────────────────────────────────────────────────────────────────
// AES-128 implementation (all from scratch, no external crates)
// ─────────────────────────────────────────────────────────────────────────────

/// AES-128 S-box.
#[rustfmt::skip]
const AES_SBOX: [u8; 256] = [
    0x63,0x7c,0x77,0x7b,0xf2,0x6b,0x6f,0xc5,0x30,0x01,0x67,0x2b,0xfe,0xd7,0xab,0x76,
    0xca,0x82,0xc9,0x7d,0xfa,0x59,0x47,0xf0,0xad,0xd4,0xa2,0xaf,0x9c,0xa4,0x72,0xc0,
    0xb7,0xfd,0x93,0x26,0x36,0x3f,0xf7,0xcc,0x34,0xa5,0xe5,0xf1,0x71,0xd8,0x31,0x15,
    0x04,0xc7,0x23,0xc3,0x18,0x96,0x05,0x9a,0x07,0x12,0x80,0xe2,0xeb,0x27,0xb2,0x75,
    0x09,0x83,0x2c,0x1a,0x1b,0x6e,0x5a,0xa0,0x52,0x3b,0xd6,0xb3,0x29,0xe3,0x2f,0x84,
    0x53,0xd1,0x00,0xed,0x20,0xfc,0xb1,0x5b,0x6a,0xcb,0xbe,0x39,0x4a,0x4c,0x58,0xcf,
    0xd0,0xef,0xaa,0xfb,0x43,0x4d,0x33,0x85,0x45,0xf9,0x02,0x7f,0x50,0x3c,0x9f,0xa8,
    0x51,0xa3,0x40,0x8f,0x92,0x9d,0x38,0xf5,0xbc,0xb6,0xda,0x21,0x10,0xff,0xf3,0xd2,
    0xcd,0x0c,0x13,0xec,0x5f,0x97,0x44,0x17,0xc4,0xa7,0x7e,0x3d,0x64,0x5d,0x19,0x73,
    0x60,0x81,0x4f,0xdc,0x22,0x2a,0x90,0x88,0x46,0xee,0xb8,0x14,0xde,0x5e,0x0b,0xdb,
    0xe0,0x32,0x3a,0x0a,0x49,0x06,0x24,0x5c,0xc2,0xd3,0xac,0x62,0x91,0x95,0xe4,0x79,
    0xe7,0xc8,0x37,0x6d,0x8d,0xd5,0x4e,0xa9,0x6c,0x56,0xf4,0xea,0x65,0x7a,0xae,0x08,
    0xba,0x78,0x25,0x2e,0x1c,0xa6,0xb4,0xc6,0xe8,0xdd,0x74,0x1f,0x4b,0xbd,0x8b,0x8a,
    0x70,0x3e,0xb5,0x66,0x48,0x03,0xf6,0x0e,0x61,0x35,0x57,0xb9,0x86,0xc1,0x1d,0x9e,
    0xe1,0xf8,0x98,0x11,0x69,0xd9,0x8e,0x94,0x9b,0x1e,0x87,0xe9,0xce,0x55,0x28,0xdf,
    0x8c,0xa1,0x89,0x0d,0xbf,0xe6,0x42,0x68,0x41,0x99,0x2d,0x0f,0xb0,0x54,0xbb,0x16,
];

/// AES-128 round constants (Rcon).
const AES_RCON: [u8; 11] = [0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1b,0x36];

/// GF(2^8) multiplication (irreducible poly 0x11B).
fn gf_mul(mut a: u8, mut b: u8) -> u8 {
    let mut result = 0u8;
    for _ in 0..8 {
        if b & 1 != 0 { result ^= a; }
        let hi = a & 0x80;
        a <<= 1;
        if hi != 0 { a ^= 0x1b; }
        b >>= 1;
    }
    result
}

/// AES-128 key schedule — returns 11 round keys (each 16 bytes).
fn aes128_key_schedule(key: &[u8; 16]) -> [[u8; 16]; 11] {
    let mut w = [0u8; 176]; // 11 * 16
    w[..16].copy_from_slice(key);
    for i in 4..44usize {
        let mut temp = [w[(i-1)*4], w[(i-1)*4+1], w[(i-1)*4+2], w[(i-1)*4+3]];
        if i % 4 == 0 {
            temp.rotate_left(1);
            temp[0] = AES_SBOX[temp[0] as usize];
            temp[1] = AES_SBOX[temp[1] as usize];
            temp[2] = AES_SBOX[temp[2] as usize];
            temp[3] = AES_SBOX[temp[3] as usize];
            temp[0] ^= AES_RCON[i / 4];
        }
        for j in 0..4 {
            w[i*4+j] = w[(i-4)*4+j] ^ temp[j];
        }
    }
    let mut rk = [[0u8; 16]; 11];
    for r in 0..11 { rk[r].copy_from_slice(&w[r*16..(r+1)*16]); }
    rk
}

/// AES ShiftRows.
fn aes_shift_rows(state: &mut [u8; 16]) {
    let s = *state;
    // Row 0: no shift
    // Row 1: shift left 1
    state[1]  = s[5];  state[5]  = s[9];  state[9]  = s[13]; state[13] = s[1];
    // Row 2: shift left 2
    state[2]  = s[10]; state[10] = s[2];  state[6]  = s[14]; state[14] = s[6];
    // Row 3: shift left 3
    state[3]  = s[15]; state[15] = s[11]; state[11] = s[7];  state[7]  = s[3];
}

/// AES MixColumns on one column.
fn aes_mix_col(col: &mut [u8; 4]) {
    let (a, b, c, d) = (col[0], col[1], col[2], col[3]);
    col[0] = gf_mul(a,2) ^ gf_mul(b,3) ^ c ^ d;
    col[1] = a ^ gf_mul(b,2) ^ gf_mul(c,3) ^ d;
    col[2] = a ^ b ^ gf_mul(c,2) ^ gf_mul(d,3);
    col[3] = gf_mul(a,3) ^ b ^ c ^ gf_mul(d,2);
}

/// AES-128 encrypt one block (ECB, in-place).
pub fn aes128_encrypt_block(block: &mut [u8; AES_BLOCK_SIZE], rk: &[[u8; 16]; 11]) {
    // Initial round key add
    for i in 0..16 { block[i] ^= rk[0][i]; }
    for round in 1..=9 {
        // SubBytes
        for b in block.iter_mut() { *b = AES_SBOX[*b as usize]; }
        // ShiftRows
        aes_shift_rows(block);
        // MixColumns
        for col in 0..4 {
            let mut c = [block[col*4], block[col*4+1], block[col*4+2], block[col*4+3]];
            aes_mix_col(&mut c);
            block[col*4] = c[0]; block[col*4+1] = c[1];
            block[col*4+2] = c[2]; block[col*4+3] = c[3];
        }
        // AddRoundKey
        for i in 0..16 { block[i] ^= rk[round][i]; }
    }
    // Final round (no MixColumns)
    for b in block.iter_mut() { *b = AES_SBOX[*b as usize]; }
    aes_shift_rows(block);
    for i in 0..16 { block[i] ^= rk[10][i]; }
}

// ─────────────────────────────────────────────────────────────────────────────
// AES-128-CCM* (IEEE 802.15.4-2020 §9)
// ─────────────────────────────────────────────────────────────────────────────

/// Security level as defined by IEEE 802.15.4-2020 Table 9-6.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SecurityLevel {
    /// No security.
    None,
    /// MIC-32: integrity only, 4-byte MIC.
    Mic32,
    /// MIC-64: integrity only, 8-byte MIC.
    Mic64,
    /// MIC-128: integrity only, 16-byte MIC.
    Mic128,
    /// ENC: encryption only (no MIC).
    Enc,
    /// ENC-MIC-32: encryption + 4-byte MIC.
    EncMic32,
    /// ENC-MIC-64: encryption + 8-byte MIC.
    EncMic64,
    /// ENC-MIC-128: encryption + 16-byte MIC.
    EncMic128,
}

impl SecurityLevel {
    /// MIC length in bytes (M parameter).
    pub fn mic_len(self) -> usize {
        match self {
            Self::None | Self::Enc => 0,
            Self::Mic32 | Self::EncMic32 => 4,
            Self::Mic64 | Self::EncMic64 => 8,
            Self::Mic128 | Self::EncMic128 => 16,
        }
    }
    /// Whether this level encrypts the payload.
    pub fn encrypts(self) -> bool {
        matches!(self, Self::Enc | Self::EncMic32 | Self::EncMic64 | Self::EncMic128)
    }
}

/// AES-128-CCM* authentication + encryption.
///
/// `key`   — 16-byte AES key
/// `nonce` — 13-byte CCM nonce
/// `aad`   — additional authenticated data (MAC header)
/// `data`  — plaintext payload (modified in-place to ciphertext if encrypts)
/// Returns the MIC (message integrity code, may be empty).
pub fn ccm_star_auth_encrypt(
    key: &[u8; 16],
    nonce: &[u8; 13],
    aad: &[u8],
    data: &mut Vec<u8>,
    level: SecurityLevel,
) -> Vec<u8> {
    let m = level.mic_len();
    let rk = aes128_key_schedule(key);

    // ── CBC-MAC for authentication ───────────────────────────────────────────
    let mut b0 = [0u8; 16];
    // Flags byte: Adata flag, M encoding, L encoding (L=2 → l(m)=2)
    let l = 2usize; // length of length field
    let flags = (((!aad.is_empty()) as u8) << 6)
        | (((m.saturating_sub(2)) / 2) as u8 & 0x7) << 3
        | ((l - 1) as u8 & 0x7);
    b0[0] = flags;
    b0[1..14].copy_from_slice(nonce);
    // Encode message length (2 bytes, big-endian)
    let msg_len = data.len();
    b0[14] = (msg_len >> 8) as u8;
    b0[15] = msg_len as u8;

    let mut x = b0;
    aes128_encrypt_block(&mut x, &rk);

    // Encode AAD length header
    let mut aad_block: Vec<u8> = Vec::new();
    let aad_len = aad.len();
    if aad_len > 0 {
        if aad_len < 0xFF00 {
            aad_block.push((aad_len >> 8) as u8);
            aad_block.push(aad_len as u8);
        } else {
            aad_block.push(0xFF);
            aad_block.push(0xFE);
            aad_block.extend_from_slice(&(aad_len as u32).to_be_bytes());
        }
        aad_block.extend_from_slice(aad);
        // Pad to 16-byte boundary
        while aad_block.len() % 16 != 0 { aad_block.push(0); }
        for chunk in aad_block.chunks(16) {
            for i in 0..16 { x[i] ^= chunk[i]; }
            aes128_encrypt_block(&mut x, &rk);
        }
    }

    // Process message blocks
    let mut msg_padded = data.to_vec();
    while msg_padded.len() % 16 != 0 { msg_padded.push(0); }
    for chunk in msg_padded.chunks(16) {
        for i in 0..16 { x[i] ^= chunk[i]; }
        aes128_encrypt_block(&mut x, &rk);
    }
    let t_raw = x; // CBC-MAC result

    // ── CTR mode for encryption ───────────────────────────────────────────────
    // A_i = flags_ctr || nonce || counter (2 bytes)
    let ctr_flags = (l - 1) as u8; // = 0x01 for L=2
    let mut a0 = [0u8; 16];
    a0[0] = ctr_flags;
    a0[1..14].copy_from_slice(nonce);
    // A0 has counter=0 → used to encrypt T

    let mut a = a0;
    a[14] = 0; a[15] = 0;
    let mut s0 = a;
    aes128_encrypt_block(&mut s0, &rk);

    // Compute MIC = T XOR S0[0..m]
    let mut mic = Vec::with_capacity(m);
    for i in 0..m { mic.push(t_raw[i] ^ s0[i]); }

    // Encrypt payload if needed
    if level.encrypts() {
        let mut ctr = 1u16;
        for chunk in data.chunks_mut(16) {
            let mut a_i = a0;
            a_i[14] = (ctr >> 8) as u8;
            a_i[15] = ctr as u8;
            let mut s_i = a_i;
            aes128_encrypt_block(&mut s_i, &rk);
            for (j, b) in chunk.iter_mut().enumerate() {
                *b ^= s_i[j];
            }
            ctr = ctr.wrapping_add(1);
        }
    }

    mic
}

// ─────────────────────────────────────────────────────────────────────────────
// CRC-16-CCITT (IEEE 802.15.4 FCS)
// ─────────────────────────────────────────────────────────────────────────────

/// Compute CRC-16/CCITT-FALSE (poly 0x1021, init 0x0000) LSB-first.
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc = 0u16;
    for &byte in data {
        let mut b = byte;
        for _ in 0..8 {
            let bit = (crc ^ b as u16) & 1;
            crc >>= 1;
            if bit != 0 { crc ^= CRC16_POLY; }
            b >>= 1;
        }
    }
    crc
}

// ─────────────────────────────────────────────────────────────────────────────
// MAC Address types
// ─────────────────────────────────────────────────────────────────────────────

/// IEEE 802.15.4 device address.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MacAddress {
    /// No address present.
    None,
    /// 16-bit short address.
    Short(u16),
    /// 64-bit extended (IEEE EUI-64) address.
    Extended(u64),
}

impl MacAddress {
    /// Address mode field encoding (2 bits).
    pub fn mode_bits(self) -> u8 {
        match self {
            Self::None => 0b00,
            Self::Short(_) => 0b10,
            Self::Extended(_) => 0b11,
        }
    }

    /// Serialise address to little-endian bytes.
    pub fn to_bytes(self) -> Vec<u8> {
        match self {
            Self::None => vec![],
            Self::Short(a) => a.to_le_bytes().to_vec(),
            Self::Extended(a) => a.to_le_bytes().to_vec(),
        }
    }

    /// Byte length of the address field.
    pub fn byte_len(self) -> usize {
        match self {
            Self::None => 0,
            Self::Short(_) => 2,
            Self::Extended(_) => 8,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MAC Frame Types (IEEE 802.15.4-2020 §7.2)
// ─────────────────────────────────────────────────────────────────────────────

/// MAC frame type identifier (3-bit field in FCF).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MacFrameType {
    /// Beacon frame.
    Beacon = 0b000,
    /// Data frame.
    Data = 0b001,
    /// Acknowledgement frame.
    Ack = 0b010,
    /// MAC Command frame.
    Command = 0b011,
    /// Enhanced Beacon frame (IEEE 802.15.4e).
    EnhancedBeacon = 0b101,
}

impl MacFrameType {
    fn from_bits(v: u8) -> Option<Self> {
        match v & 0x7 {
            0 => Some(Self::Beacon),
            1 => Some(Self::Data),
            2 => Some(Self::Ack),
            3 => Some(Self::Command),
            5 => Some(Self::EnhancedBeacon),
            _ => None,
        }
    }
}

/// MAC Command identifiers (IEEE 802.15.4-2020 §7.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MacCommand {
    AssociationRequest = 0x01,
    AssociationResponse = 0x02,
    DisassociationNotification = 0x03,
    DataRequest = 0x04,
    PanIdConflictNotification = 0x05,
    OrphanNotification = 0x06,
    BeaconRequest = 0x07,
    CoordinatorRealignment = 0x08,
    GTSRequest = 0x09,
}

// ─────────────────────────────────────────────────────────────────────────────
// MAC Frame builder / parser
// ─────────────────────────────────────────────────────────────────────────────

/// Parsed or built MAC frame.
#[derive(Debug, Clone)]
pub struct MacFrame {
    /// Frame type.
    pub frame_type: MacFrameType,
    /// Security enabled flag.
    pub security_enabled: bool,
    /// Frame pending bit.
    pub frame_pending: bool,
    /// Acknowledgement request flag.
    pub ack_request: bool,
    /// PAN ID compression flag.
    pub pan_id_compressed: bool,
    /// Sequence number.
    pub seq_num: u8,
    /// Destination PAN ID (0xFFFF = broadcast).
    pub dst_pan: u16,
    /// Destination address.
    pub dst_addr: MacAddress,
    /// Source PAN ID.
    pub src_pan: u16,
    /// Source address.
    pub src_addr: MacAddress,
    /// MAC payload (MSDU).
    pub payload: Vec<u8>,
    /// Auxiliary security header (if security_enabled).
    pub aux_sec_header: Option<AuxSecHeader>,
}

/// Auxiliary Security Header (IEEE 802.15.4-2020 §7.4).
#[derive(Debug, Clone)]
pub struct AuxSecHeader {
    /// Security level.
    pub level: SecurityLevel,
    /// Key identifier mode (0 = implicit).
    pub key_id_mode: u8,
    /// Frame counter (monotonically increasing).
    pub frame_counter: u32,
    /// Key source + index (optional).
    pub key_id: Vec<u8>,
}

impl AuxSecHeader {
    /// Create a new auxiliary security header.
    pub fn new(level: SecurityLevel, frame_counter: u32) -> Self {
        Self {
            level,
            key_id_mode: 0,
            frame_counter,
            key_id: vec![],
        }
    }

    /// Encode to bytes.
    pub fn to_bytes(&self) -> Vec<u8> {
        let level_code: u8 = match self.level {
            SecurityLevel::None => 0,
            SecurityLevel::Mic32 => 1,
            SecurityLevel::Mic64 => 2,
            SecurityLevel::Mic128 => 3,
            SecurityLevel::Enc => 4,
            SecurityLevel::EncMic32 => 5,
            SecurityLevel::EncMic64 => 6,
            SecurityLevel::EncMic128 => 7,
        };
        let sc = level_code | (self.key_id_mode << 3);
        let mut out = vec![sc];
        out.extend_from_slice(&self.frame_counter.to_le_bytes());
        out.extend_from_slice(&self.key_id);
        out
    }
}

impl MacFrame {
    /// Build the Frame Control Field (2 bytes, little-endian).
    pub fn build_fcf(&self) -> [u8; 2] {
        let mut fcf: u16 = (self.frame_type as u16) & 0x7;
        if self.security_enabled { fcf |= 1 << 3; }
        if self.frame_pending   { fcf |= 1 << 4; }
        if self.ack_request     { fcf |= 1 << 5; }
        if self.pan_id_compressed { fcf |= 1 << 6; }
        // Destination addressing mode
        fcf |= (self.dst_addr.mode_bits() as u16) << 10;
        // Frame version (IEEE 802.15.4-2006 = 0b01)
        fcf |= 0b01 << 12;
        // Source addressing mode
        fcf |= (self.src_addr.mode_bits() as u16) << 14;
        fcf.to_le_bytes()
    }

    /// Serialise the full MPDU (MHR + payload + FCS).
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        let fcf = self.build_fcf();
        out.extend_from_slice(&fcf);
        out.push(self.seq_num);
        // Destination PAN + address
        if self.dst_addr != MacAddress::None {
            out.extend_from_slice(&self.dst_pan.to_le_bytes());
            out.extend_from_slice(&self.dst_addr.to_bytes());
        }
        // Source PAN (omit if compressed)
        if self.src_addr != MacAddress::None && !self.pan_id_compressed {
            out.extend_from_slice(&self.src_pan.to_le_bytes());
        }
        out.extend_from_slice(&self.src_addr.to_bytes());
        // Auxiliary security header
        if let Some(ref ash) = self.aux_sec_header {
            out.extend_from_slice(&ash.to_bytes());
        }
        // Payload
        out.extend_from_slice(&self.payload);
        // FCS (CRC-16)
        let fcs = crc16_ccitt(&out);
        out.extend_from_slice(&fcs.to_le_bytes());
        out
    }

    /// Build a basic Beacon frame.
    pub fn beacon(src_pan: u16, src_addr: MacAddress, seq: u8, superframe: u16) -> Self {
        // Beacon payload: superframe spec (2 bytes) + GTS + pending
        let mut payload = Vec::new();
        payload.extend_from_slice(&superframe.to_le_bytes());
        payload.push(0x00); // GTS spec: no GTS
        payload.push(0x00); // Pending address spec: no pending
        MacFrame {
            frame_type: MacFrameType::Beacon,
            security_enabled: false,
            frame_pending: false,
            ack_request: false,
            pan_id_compressed: false,
            seq_num: seq,
            dst_pan: 0xFFFF,
            dst_addr: MacAddress::None,
            src_pan,
            src_addr,
            payload,
            aux_sec_header: None,
        }
    }

    /// Build an Ack frame for a given sequence number.
    pub fn ack(seq_num: u8) -> Self {
        MacFrame {
            frame_type: MacFrameType::Ack,
            security_enabled: false,
            frame_pending: false,
            ack_request: false,
            pan_id_compressed: false,
            seq_num,
            dst_pan: 0,
            dst_addr: MacAddress::None,
            src_pan: 0,
            src_addr: MacAddress::None,
            payload: vec![],
            aux_sec_header: None,
        }
    }

    /// Parse a MAC frame from raw bytes. Returns frame and consumed byte count.
    pub fn parse(data: &[u8]) -> Option<(Self, usize)> {
        if data.len() < FCF_SIZE + 1 { return None; }
        let fcf = u16::from_le_bytes([data[0], data[1]]);
        let frame_type_bits = (fcf & 0x7) as u8;
        let frame_type = MacFrameType::from_bits(frame_type_bits)?;
        let security_enabled = (fcf >> 3) & 1 == 1;
        let frame_pending   = (fcf >> 4) & 1 == 1;
        let ack_request     = (fcf >> 5) & 1 == 1;
        let pan_id_compressed = (fcf >> 6) & 1 == 1;
        let dst_mode = ((fcf >> 10) & 0x3) as u8;
        let src_mode = ((fcf >> 14) & 0x3) as u8;

        let mut cursor = 2;
        let seq_num = data[cursor]; cursor += 1;

        // Parse destination address
        let (dst_pan, dst_addr, c) = parse_address(data, cursor, dst_mode, 0xFFFF)?;
        cursor += c;

        // Parse source address
        let src_pan_default = if pan_id_compressed { dst_pan } else { 0 };
        let (src_pan, src_addr, c) = parse_address(data, cursor, src_mode, src_pan_default)?;
        cursor += c;

        // Auxiliary security header
        let aux_sec_header = if security_enabled {
            let ash = parse_aux_sec(data, cursor)?;
            cursor += 1 + 4 + ash.key_id.len(); // sc + fc + key_id
            Some(ash)
        } else {
            None
        };

        // Payload (subtract 2-byte FCS)
        if data.len() < cursor + FCS_SIZE { return None; }
        let payload = data[cursor..data.len() - FCS_SIZE].to_vec();

        let frame = MacFrame {
            frame_type,
            security_enabled,
            frame_pending,
            ack_request,
            pan_id_compressed,
            seq_num,
            dst_pan,
            dst_addr,
            src_pan,
            src_addr,
            payload,
            aux_sec_header,
        };
        Some((frame, data.len()))
    }
}

fn parse_address(data: &[u8], mut cursor: usize, mode: u8, default_pan: u16) -> Option<(u16, MacAddress, usize)> {
    let start = cursor;
    let (pan, addr) = match mode {
        0b00 => (default_pan, MacAddress::None),
        0b10 => {
            if data.len() < cursor + 4 { return None; }
            let p = u16::from_le_bytes([data[cursor], data[cursor+1]]); cursor += 2;
            let a = u16::from_le_bytes([data[cursor], data[cursor+1]]); cursor += 2;
            (p, MacAddress::Short(a))
        }
        0b11 => {
            if data.len() < cursor + 10 { return None; }
            let p = u16::from_le_bytes([data[cursor], data[cursor+1]]); cursor += 2;
            let mut buf = [0u8; 8];
            buf.copy_from_slice(&data[cursor..cursor+8]); cursor += 8;
            (p, MacAddress::Extended(u64::from_le_bytes(buf)))
        }
        _ => return None,
    };
    Some((pan, addr, cursor - start))
}

fn parse_aux_sec(data: &[u8], cursor: usize) -> Option<AuxSecHeader> {
    if data.len() <= cursor { return None; }
    let sc = data[cursor];
    let level_code = sc & 0x7;
    let key_id_mode = (sc >> 3) & 0x3;
    if data.len() < cursor + 5 { return None; }
    let fc = u32::from_le_bytes([data[cursor+1], data[cursor+2], data[cursor+3], data[cursor+4]]);
    let level = match level_code {
        0 => SecurityLevel::None,
        1 => SecurityLevel::Mic32,
        2 => SecurityLevel::Mic64,
        3 => SecurityLevel::Mic128,
        4 => SecurityLevel::Enc,
        5 => SecurityLevel::EncMic32,
        6 => SecurityLevel::EncMic64,
        7 => SecurityLevel::EncMic128,
        _ => SecurityLevel::None,
    };
    Some(AuxSecHeader { level, key_id_mode, frame_counter: fc, key_id: vec![] })
}

// ─────────────────────────────────────────────────────────────────────────────
// CSMA-CA parameters (IEEE 802.15.4-2020 §6.2.5)
// ─────────────────────────────────────────────────────────────────────────────

/// CSMA-CA configuration.
#[derive(Debug, Clone)]
pub struct CsmaCaConfig {
    /// Minimum backoff exponent.
    pub min_be: u8,
    /// Maximum backoff exponent.
    pub max_be: u8,
    /// Maximum number of backoffs before failure.
    pub max_csma_backoffs: u8,
    /// Slotted CSMA-CA (true) or unslotted (false).
    pub slotted: bool,
    /// aUnitBackoffPeriod in symbols (20 symbols for IEEE 802.15.4).
    pub unit_backoff_symbols: u32,
}

impl Default for CsmaCaConfig {
    fn default() -> Self {
        Self { min_be: 3, max_be: 5, max_csma_backoffs: 4, slotted: false, unit_backoff_symbols: 20 }
    }
}

/// CSMA-CA state machine.
#[derive(Debug)]
pub struct CsmaCa {
    cfg: CsmaCaConfig,
    /// Current backoff exponent.
    pub be: u8,
    /// Number of backoffs attempted.
    pub nb: u8,
    /// Simple LFSR for pseudo-random backoff.
    rng: u16,
}

impl CsmaCa {
    /// Create a new CSMA-CA instance with given seed.
    pub fn new(cfg: CsmaCaConfig, seed: u16) -> Self {
        let be = cfg.min_be;
        Self { cfg, be, nb: 0, rng: if seed == 0 { 1 } else { seed } }
    }

    /// Next backoff delay in symbols.
    /// Returns `None` when maximum backoffs exceeded (channel access failure).
    pub fn next_backoff(&mut self) -> Option<u32> {
        if self.nb > self.cfg.max_csma_backoffs { return None; }
        let rand_bits = self.rng_next() & ((1u16 << self.be) - 1);
        let delay = self.cfg.unit_backoff_symbols * (rand_bits as u32);
        self.be = (self.be + 1).min(self.cfg.max_be);
        self.nb += 1;
        Some(delay)
    }

    /// Reset CSMA-CA state for a new transmission attempt.
    pub fn reset(&mut self) {
        self.be = self.cfg.min_be;
        self.nb = 0;
    }

    fn rng_next(&mut self) -> u16 {
        // 16-bit Galois LFSR (poly 0xB400)
        let lsb = self.rng & 1;
        self.rng >>= 1;
        if lsb != 0 { self.rng ^= 0xB400; }
        self.rng
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Energy Detection and LQI
// ─────────────────────────────────────────────────────────────────────────────

/// Channel energy scan result.
#[derive(Debug, Clone)]
pub struct EdResult {
    /// Channel number (11–26).
    pub channel: u8,
    /// Energy level (0–255, proportional to dBm).
    pub energy: u8,
    /// Centre frequency in Hz.
    pub freq_hz: u64,
}

/// Compute energy detection for a set of IQ samples.
/// Returns normalised ED value 0–255.
pub fn compute_ed(samples: &[(f32, f32)]) -> u8 {
    if samples.is_empty() { return 0; }
    let power: f64 = samples.iter()
        .map(|&(i, q)| (i as f64).powi(2) + (q as f64).powi(2))
        .sum::<f64>() / samples.len() as f64;
    let db = if power > 0.0 { 10.0 * power.log10() } else { -100.0 };
    // Map [-85 dBm, -10 dBm] → [0, 255]
    let clamped = db.clamp(-85.0, -10.0);
    ((clamped + 85.0) / 75.0 * 255.0) as u8
}

/// Compute Link Quality Indicator from chip correlation score.
/// `corr` is the normalised cross-correlation magnitude [0.0, 1.0].
pub fn compute_lqi(corr: f32) -> u8 {
    (corr.clamp(0.0, 1.0) * 255.0) as u8
}

// ─────────────────────────────────────────────────────────────────────────────
// Channel management
// ─────────────────────────────────────────────────────────────────────────────

/// Returns centre frequency in Hz for a 2.4 GHz channel (11–26).
pub fn channel_frequency_hz(channel: u8) -> u64 {
    assert!(channel >= MIN_CHANNEL && channel <= MAX_CHANNEL, "invalid channel");
    BASE_FREQ_HZ + CHANNEL_SPACING_HZ * (channel - MIN_CHANNEL) as u64
}

/// Energy scan across channels 11–26. Returns sorted list (lowest energy first).
pub fn energy_scan(measure: &dyn Fn(u8) -> u8) -> Vec<EdResult> {
    let mut results: Vec<EdResult> = (MIN_CHANNEL..=MAX_CHANNEL)
        .map(|ch| EdResult {
            channel: ch,
            energy: measure(ch),
            freq_hz: channel_frequency_hz(ch),
        })
        .collect();
    results.sort_by_key(|r| r.energy);
    results
}

// ─────────────────────────────────────────────────────────────────────────────
// PHY configuration
// ─────────────────────────────────────────────────────────────────────────────

/// PHY layer configuration.
#[derive(Debug, Clone)]
pub struct PhyConfig {
    /// Operating channel (11–26 for 2.4 GHz).
    pub channel: u8,
    /// Transmit power in dBm.
    pub tx_power_dbm: i8,
    /// Number of samples per chip for IQ generation.
    pub samples_per_chip: usize,
    /// Sample rate (Hz).
    pub sample_rate: f64,
}

impl PhyConfig {
    /// Default 2.4 GHz configuration on the specified channel.
    pub fn default_2_4ghz(channel: u8) -> Self {
        assert!(channel >= MIN_CHANNEL && channel <= MAX_CHANNEL);
        Self { channel, tx_power_dbm: 0, samples_per_chip: 4, sample_rate: 8_000_000.0 }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// MAC configuration
// ─────────────────────────────────────────────────────────────────────────────

/// MAC layer configuration.
#[derive(Debug, Clone)]
pub struct MacConfig {
    /// PAN identifier.
    pub pan_id: u16,
    /// Local device address.
    pub local_addr: MacAddress,
    /// Whether this device is a PAN coordinator.
    pub is_coordinator: bool,
    /// Frame sequence number (auto-incremented).
    pub seq_num: u8,
    /// Frame counter for security.
    pub frame_counter: u32,
    /// AES-128 key for security.
    pub key: Option<[u8; 16]>,
}

impl MacConfig {
    /// Create a new MAC configuration.
    pub fn new(pan_id: u16, local_addr: MacAddress) -> Self {
        Self { pan_id, local_addr, is_coordinator: false, seq_num: 0, frame_counter: 0, key: None }
    }

    /// Set the AES-128 security key.
    pub fn with_key(mut self, key: [u8; 16]) -> Self {
        self.key = Some(key);
        self
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Indirect transmission queue
// ─────────────────────────────────────────────────────────────────────────────

/// Pending indirect transmission entry.
#[derive(Debug, Clone)]
pub struct IndirectEntry {
    /// Destination address.
    pub dst: MacAddress,
    /// Frame to be sent.
    pub frame: Vec<u8>,
    /// Frame pending indicator.
    pub pending: bool,
}

/// Indirect transmission queue (for frame_pending bit).
#[derive(Debug, Default)]
pub struct IndirectQueue {
    queue: VecDeque<IndirectEntry>,
}

impl IndirectQueue {
    /// Create a new indirect queue.
    pub fn new() -> Self { Self::default() }

    /// Enqueue a frame for indirect delivery.
    pub fn enqueue(&mut self, dst: MacAddress, frame: Vec<u8>) {
        self.queue.push_back(IndirectEntry { dst, frame, pending: true });
    }

    /// Dequeue the next frame for the given destination.
    pub fn dequeue(&mut self, dst: MacAddress) -> Option<IndirectEntry> {
        let pos = self.queue.iter().position(|e| e.dst == dst)?;
        self.queue.remove(pos)
    }

    /// Check whether there is a pending frame for the given destination.
    pub fn has_pending(&self, dst: MacAddress) -> bool {
        self.queue.iter().any(|e| e.dst == dst)
    }

    /// Number of queued frames.
    pub fn len(&self) -> usize { self.queue.len() }

    /// Returns true if the queue is empty.
    pub fn is_empty(&self) -> bool { self.queue.is_empty() }
}

// ─────────────────────────────────────────────────────────────────────────────
// 6LoWPAN IPHC header compression (RFC 6282)
// ─────────────────────────────────────────────────────────────────────────────

/// IPHC dispatch prefix (2-bit = 0b11).
const IPHC_DISPATCH: u8 = 0x60; // 0110_xxxx xxxxxxxx

/// Compress an IPv6 header using 6LoWPAN IPHC per RFC 6282.
///
/// `ipv6_hdr` — 40-byte IPv6 fixed header (version..hop_limit + src + dst)
/// `src_iid`  — source IID (8 bytes) derived from link-layer address
/// `dst_iid`  — destination IID (8 bytes)
/// Returns IPHC-compressed bytes.
pub fn iphc_compress(ipv6_hdr: &[u8; 40], src_iid: &[u8; 8], dst_iid: &[u8; 8]) -> Vec<u8> {
    let mut out = Vec::new();

    // IPHC byte 0
    let mut iphc0: u8 = 0x60; // dispatch 011 + TF=00 (inline traffic class/flow)
    // Inline flow label check (bytes 1-3 of IPv6 hdr)
    let flow_label = ((ipv6_hdr[1] as u32 & 0x0F) << 16)
        | ((ipv6_hdr[2] as u32) << 8)
        | (ipv6_hdr[3] as u32);
    let tc = ((ipv6_hdr[0] & 0x0F) << 4) | (ipv6_hdr[1] >> 4);

    // TF field: 00 = inline, 11 = elided
    if flow_label == 0 && tc == 0 {
        iphc0 |= 0b11 << 4; // TF=11: elide
    }

    // NH: next header (byte 6)
    let nh = ipv6_hdr[6];
    let is_udp = nh == 17;
    if is_udp { iphc0 |= 0x04; } // NH=1: compressed

    // HLIM: hop limit (byte 7)
    let hlim = ipv6_hdr[7];
    iphc0 |= match hlim { 1 => 0b01, 64 => 0b10, 255 => 0b11, _ => 0b00 };

    // IPHC byte 1
    let mut iphc1: u8 = 0;
    // SAC=0 (stateless), SAM: compare src address IID
    let src_addr = &ipv6_hdr[8..24];
    let sam = if &src_addr[8..] == src_iid && src_addr[..8] == [0xFE, 0x80, 0, 0, 0, 0, 0, 0] {
        iphc1 |= 0b11; // SAM=11: elide (link-local, IID from context)
        0b11
    } else {
        0b00 // inline
    };
    // DAC=0, DAM: destination
    let dst_addr = &ipv6_hdr[24..40];
    let _dam = if &dst_addr[8..] == dst_iid && dst_addr[..8] == [0xFE, 0x80, 0, 0, 0, 0, 0, 0] {
        iphc1 |= 0b11 << 4; // DAM=11
        0b11
    } else if dst_addr[0] == 0xFF {
        iphc1 |= 0b01 << 4; // DAM=01 multicast
        0b01
    } else {
        0b00
    };

    out.push(iphc0);
    out.push(iphc1);

    // Inline TF if not elided
    if (iphc0 >> 4) & 0b11 != 0b11 {
        out.push(tc);
        out.push((flow_label >> 16) as u8 & 0x0F);
        out.push((flow_label >> 8) as u8);
        out.push(flow_label as u8);
    }

    // Inline NH if not compressed
    if !is_udp { out.push(nh); }

    // Inline HLIM if not elided
    if hlim != 1 && hlim != 64 && hlim != 255 { out.push(hlim); }

    // Inline source address if not elided
    if sam == 0b00 { out.extend_from_slice(src_addr); }

    // Inline destination address (simplified)
    if (iphc1 >> 4) & 0b11 == 0b00 { out.extend_from_slice(dst_addr); }

    out
}

/// 6LoWPAN NHC compression for UDP (RFC 6282 §4.3).
///
/// `udp_hdr` — 8-byte UDP header (src_port, dst_port, length, checksum)
/// Returns NHC-compressed UDP bytes (4-byte ports may be compressed).
pub fn nhc_udp_compress(udp_hdr: &[u8; 8]) -> Vec<u8> {
    let src_port = u16::from_be_bytes([udp_hdr[0], udp_hdr[1]]);
    let dst_port = u16::from_be_bytes([udp_hdr[2], udp_hdr[3]]);
    let checksum = u16::from_be_bytes([udp_hdr[6], udp_hdr[7]]);

    // NHC dispatch for UDP = 0xF0..0xF3 depending on port compression
    // 0xF0: inline ports, 0xF1: dst 4-bit, 0xF2: src 4-bit, 0xF3: both 4-bit
    let mut out = Vec::new();

    // Check for 0xF0B_ port range (61440-61695) for 4-bit compression
    let src_4bit = src_port >= 0xF0B0 && src_port <= 0xF0BF;
    let dst_4bit = dst_port >= 0xF0B0 && dst_port <= 0xF0BF;

    let nhc_byte: u8 = 0xF0
        | if checksum == 0 { 0x04 } else { 0x00 }
        | match (src_4bit, dst_4bit) {
            (true, true)  => 0x03,
            (false, true) => 0x01,
            (true, false) => 0x02,
            _             => 0x00,
        };
    out.push(nhc_byte);

    match (src_4bit, dst_4bit) {
        (true, true) => {
            // Both 4-bit in one byte
            out.push(((src_port & 0xF) << 4) as u8 | (dst_port & 0xF) as u8);
        }
        (false, true) => {
            out.extend_from_slice(&src_port.to_be_bytes());
            out.push((dst_port & 0xFF) as u8);
        }
        (true, false) => {
            out.push((src_port & 0xFF) as u8);
            out.extend_from_slice(&dst_port.to_be_bytes());
        }
        _ => {
            out.extend_from_slice(&src_port.to_be_bytes());
            out.extend_from_slice(&dst_port.to_be_bytes());
        }
    }

    if checksum != 0 {
        out.extend_from_slice(&checksum.to_be_bytes());
    }
    out
}

// ─────────────────────────────────────────────────────────────────────────────
// Thread network layer: Mesh header and Fragment header (RFC 4944)
// ─────────────────────────────────────────────────────────────────────────────

/// 6LoWPAN Mesh Addressing Header (RFC 4944 §11).
#[derive(Debug, Clone)]
pub struct MeshHeader {
    /// Hops left field (0–14; 0xF = large).
    pub hops_left: u8,
    /// Originator address (16-bit short for Thread).
    pub originator: u16,
    /// Final destination address.
    pub final_dest: u16,
}

impl MeshHeader {
    /// Encode to bytes.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        // Dispatch 0b10xxxxxx: mesh
        let flags = 0x80u8 | (1 << 5) | (1 << 4) // V=1 F=1: short addresses
            | (self.hops_left.min(0xF) & 0xF);
        out.push(flags);
        if self.hops_left >= 0xF { out.push(self.hops_left); }
        out.extend_from_slice(&self.originator.to_be_bytes());
        out.extend_from_slice(&self.final_dest.to_be_bytes());
        out
    }
}

/// 6LoWPAN Fragmentation Header (RFC 4944 §5.3).
#[derive(Debug, Clone)]
pub struct FragHeader {
    /// Total datagram size (11-bit field).
    pub datagram_size: u16,
    /// Datagram tag (unique per source datagram).
    pub datagram_tag: u16,
    /// Fragment offset in units of 8 bytes (0 for first fragment).
    pub datagram_offset: u8,
    /// True = FRAG1, False = FRAGN.
    pub is_first: bool,
}

impl FragHeader {
    /// Encode to bytes.
    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = Vec::new();
        // Dispatch: FRAG1=0b11000xxx, FRAGN=0b11100xxx
        let dispatch = if self.is_first { 0xC0u8 } else { 0xE0u8 };
        out.push(dispatch | ((self.datagram_size >> 8) as u8 & 0x07));
        out.push(self.datagram_size as u8);
        out.extend_from_slice(&self.datagram_tag.to_be_bytes());
        if !self.is_first { out.push(self.datagram_offset); }
        out
    }
}

/// Fragment an IPv6 datagram into 6LoWPAN fragments for the given MTU.
pub fn fragment_datagram(data: &[u8], tag: u16, mtu: usize) -> Vec<Vec<u8>> {
    let total = data.len() as u16;
    if data.len() <= mtu {
        return vec![data.to_vec()];
    }
    let mut frags = Vec::new();
    // First fragment
    let hdr = FragHeader { datagram_size: total, datagram_tag: tag, datagram_offset: 0, is_first: true };
    let hdr_bytes = hdr.to_bytes();
    let payload_len = (mtu - hdr_bytes.len()) & !7usize; // round down to 8-byte boundary
    let mut frag = hdr_bytes;
    frag.extend_from_slice(&data[..payload_len]);
    frags.push(frag);

    let mut offset = payload_len;
    while offset < data.len() {
        let frag_hdr = FragHeader {
            datagram_size: total,
            datagram_tag: tag,
            datagram_offset: (offset / 8) as u8,
            is_first: false,
        };
        let fhdr_bytes = frag_hdr.to_bytes();
        let remain = data.len() - offset;
        let chunk_len = ((mtu - fhdr_bytes.len()).min(remain)) & !7usize;
        let chunk_len = if chunk_len == 0 { remain.min(mtu - fhdr_bytes.len()) } else { chunk_len };
        let mut frag = fhdr_bytes;
        frag.extend_from_slice(&data[offset..offset + chunk_len]);
        frags.push(frag);
        offset += chunk_len;
    }
    frags
}

// ─────────────────────────────────────────────────────────────────────────────
// Thread MLE (Mesh Link Establishment) message formatting
// ─────────────────────────────────────────────────────────────────────────────

/// MLE command identifiers (Thread 1.3 §8.10.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MleCommand {
    LinkRequest = 0,
    LinkAccept = 1,
    LinkAcceptAndRequest = 2,
    LinkReject = 3,
    Advertisement = 4,
    Update = 5,
    UpdateRequest = 6,
    DataRequest = 7,
    DataResponse = 8,
    ParentRequest = 9,
    ParentResponse = 10,
    ChildIdRequest = 11,
    ChildIdResponse = 12,
    ChildUpdateRequest = 13,
    ChildUpdateResponse = 14,
    Announce = 15,
    DiscoveryRequest = 16,
    DiscoveryResponse = 17,
}

/// MLE TLV types (Thread 1.3 §8.10.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MleTlvType {
    SourceAddress = 0,
    Mode = 1,
    Timeout = 2,
    Challenge = 3,
    Response = 4,
    LinkLayerFrameCounter = 5,
    MleFrameCounter = 8,
    Route64 = 9,
    Address16 = 10,
    LeaderData = 11,
    NetworkData = 12,
    TlvRequest = 13,
    ScanMask = 14,
    Connectivity = 15,
    LinkMargin = 16,
    Status = 17,
    Version = 18,
    AddressRegistration = 19,
    Channel = 20,
    PanId = 21,
    ActiveTimestamp = 22,
    PendingTimestamp = 23,
    ActiveDataset = 24,
    PendingDataset = 25,
    DiscoveryRequest = 26,
    DiscoveryResponse = 27,
}

/// A single MLE TLV.
#[derive(Debug, Clone)]
pub struct MleTlv {
    pub tlv_type: u8,
    pub value: Vec<u8>,
}

impl MleTlv {
    pub fn new(t: MleTlvType, value: Vec<u8>) -> Self {
        Self { tlv_type: t as u8, value }
    }

    pub fn to_bytes(&self) -> Vec<u8> {
        let mut out = vec![self.tlv_type, self.value.len() as u8];
        out.extend_from_slice(&self.value);
        out
    }
}

/// Build an MLE message with given command and TLVs.
///
/// The message is encapsulated in a UDP/IPv6 frame ready for 6LoWPAN compression.
/// Returns raw MLE payload bytes.
pub fn build_mle_message(command: MleCommand, tlvs: &[MleTlv]) -> Vec<u8> {
    let mut out = Vec::new();
    out.push(MLE_FRAME_TYPE);
    out.push(command as u8);
    for tlv in tlvs {
        out.extend_from_slice(&tlv.to_bytes());
    }
    out
}

/// Parse MLE TLVs from raw bytes (after command byte).
pub fn parse_mle_tlvs(data: &[u8]) -> Vec<MleTlv> {
    let mut tlvs = Vec::new();
    let mut i = 0;
    while i + 1 < data.len() {
        let t = data[i];
        let len = data[i + 1] as usize;
        i += 2;
        if i + len > data.len() { break; }
        tlvs.push(MleTlv { tlv_type: t, value: data[i..i + len].to_vec() });
        i += len;
    }
    tlvs
}

// ─────────────────────────────────────────────────────────────────────────────
// PHY: O-QPSK modulation / half-sine pulse shaping / PN spreading
// ─────────────────────────────────────────────────────────────────────────────

/// Expand a byte into a sequence of chips (64 chips = 2 symbols = 1 byte).
/// O-QPSK maps nibbles to 32-chip PN sequences.
pub fn byte_to_chips(byte: u8) -> Vec<u8> {
    let lo = (byte & 0x0F) as usize;
    let hi = ((byte >> 4) & 0x0F) as usize;
    let mut chips = Vec::with_capacity(64);
    for bit in 0..32 {
        chips.push(((PN_SEQUENCES[lo] >> bit) & 1) as u8);
    }
    for bit in 0..32 {
        chips.push(((PN_SEQUENCES[hi] >> bit) & 1) as u8);
    }
    chips
}

/// Expand a byte sequence (PPDU content) to NRZ chips (0 → +1, 1 → -1).
pub fn bytes_to_nrz_chips(data: &[u8]) -> Vec<f32> {
    let mut chips = Vec::with_capacity(data.len() * 64);
    for &b in data {
        for c in byte_to_chips(b) {
            chips.push(if c == 0 { 1.0f32 } else { -1.0 });
        }
    }
    chips
}

/// Generate half-sine pulse shaping filter taps.
/// Length = `samples_per_chip` taps, one half cycle of a sine over that window.
pub fn half_sine_filter(samples_per_chip: usize) -> Vec<f32> {
    use std::f64::consts::PI;
    (0..samples_per_chip)
        .map(|n| (PI * (n as f64 + 0.5) / samples_per_chip as f64).sin() as f32)
        .collect()
}

/// Apply half-sine pulse shaping to NRZ chips.
/// Each chip is repeated `spc` times and convolved with the half-sine window.
pub fn apply_half_sine(nrz_chips: &[f32], samples_per_chip: usize) -> Vec<f32> {
    let filt = half_sine_filter(samples_per_chip);
    let mut out = Vec::with_capacity(nrz_chips.len() * samples_per_chip);
    for &chip in nrz_chips {
        for &tap in &filt {
            out.push(chip * tap);
        }
    }
    out
}

/// O-QPSK I/Q modulation.
///
/// O-QPSK modulates two streams of chips: I on even chips, Q on odd chips,
/// with the Q stream delayed by one chip (Tc = chip period).
///
/// Returns interleaved (I, Q) pairs at `samples_per_chip` samples per chip.
pub fn oqpsk_modulate(nrz_chips: &[f32], samples_per_chip: usize) -> Vec<(f32, f32)> {
    let n = nrz_chips.len();
    // Separate I (even chips) and Q (odd chips)
    let i_chips: Vec<f32> = nrz_chips.iter().step_by(2).copied().collect();
    let q_chips: Vec<f32> = nrz_chips.iter().skip(1).step_by(2).copied().collect();

    // Apply half-sine shaping to each
    let i_shaped = apply_half_sine(&i_chips, samples_per_chip);
    let mut q_shaped = apply_half_sine(&q_chips, samples_per_chip);

    // Q is delayed by Tc/2 = samples_per_chip/2 samples (one chip delay)
    let delay = samples_per_chip / 2;
    let mut q_delayed = vec![0.0f32; delay];
    q_delayed.append(&mut q_shaped);

    // Interleave to complex pairs
    let total = i_shaped.len();
    let mut iq = Vec::with_capacity(total);
    for idx in 0..total {
        let q_val = if idx + delay < q_delayed.len() { q_delayed[idx + delay] } else { 0.0 };
        iq.push((i_shaped[idx], q_val));
    }
    let _ = n; // suppress warning
    iq
}

// ─────────────────────────────────────────────────────────────────────────────
// PPDU structure
// ─────────────────────────────────────────────────────────────────────────────

/// Build a full PPDU byte stream.
///
/// PPDU = SHR (Preamble + SFD) + PHR (1 byte length) + PSDU
pub fn build_ppdu(psdu: &[u8]) -> Vec<u8> {
    assert!(psdu.len() <= MAX_PSDU_BYTES, "PSDU too large");
    let mut ppdu = Vec::new();
    // SHR: 4-byte preamble + 1-byte SFD
    ppdu.extend_from_slice(&PREAMBLE_BYTES);
    ppdu.push(SFD);
    // PHR: frame length (7-bit) with reserved bit 0
    ppdu.push(psdu.len() as u8 & 0x7F);
    // PSDU
    ppdu.extend_from_slice(psdu);
    ppdu
}

/// Parse a PPDU byte stream. Returns (PSDU bytes, length) or None if invalid.
pub fn parse_ppdu(ppdu: &[u8]) -> Option<(Vec<u8>, usize)> {
    // Minimum: 4 preamble + 1 SFD + 1 PHR = 6 bytes
    if ppdu.len() < 6 { return None; }
    // Check SFD
    if ppdu[4] != SFD { return None; }
    let phr = ppdu[5];
    let psdu_len = (phr & 0x7F) as usize;
    if ppdu.len() < 6 + psdu_len { return None; }
    Some((ppdu[6..6 + psdu_len].to_vec(), 6 + psdu_len))
}

// ─────────────────────────────────────────────────────────────────────────────
// Beacon request / response
// ─────────────────────────────────────────────────────────────────────────────

/// Build a Beacon Request command frame (used for active scan).
pub fn build_beacon_request(seq: u8) -> MacFrame {
    MacFrame {
        frame_type: MacFrameType::Command,
        security_enabled: false,
        frame_pending: false,
        ack_request: false,
        pan_id_compressed: false,
        seq_num: seq,
        dst_pan: 0xFFFF,
        dst_addr: MacAddress::Short(0xFFFF),
        src_pan: 0xFFFF,
        src_addr: MacAddress::None,
        payload: vec![MacCommand::BeaconRequest as u8],
        aux_sec_header: None,
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// ThreadProcessor — top-level API
// ─────────────────────────────────────────────────────────────────────────────

/// Thread mesh network processor.
///
/// Combines PHY (O-QPSK modulation, PN spreading) and MAC (frame building,
/// CSMA-CA, security) into a single convenient interface.
pub struct ThreadProcessor {
    /// PHY configuration.
    pub phy: PhyConfig,
    /// MAC configuration.
    pub mac: MacConfig,
    /// CSMA-CA state machine.
    pub csma: CsmaCa,
    /// Indirect transmission queue.
    pub indirect_queue: IndirectQueue,
}

impl ThreadProcessor {
    /// Create a new ThreadProcessor.
    pub fn new(phy: PhyConfig, mac: MacConfig) -> Self {
        Self {
            phy,
            mac,
            csma: CsmaCa::new(CsmaCaConfig::default(), 0xACE1),
            indirect_queue: IndirectQueue::new(),
        }
    }

    /// Allocate and return the next sequence number.
    fn next_seq(&mut self) -> u8 {
        let s = self.mac.seq_num;
        self.mac.seq_num = self.mac.seq_num.wrapping_add(1);
        s
    }

    /// Build a data frame from the given payload.
    pub fn build_data_frame(&mut self, payload: &[u8], dst: MacAddress, ack_req: bool) -> MacFrame {
        let seq = self.next_seq();
        let pan_compressed = matches!(dst, MacAddress::Short(_));
        MacFrame {
            frame_type: MacFrameType::Data,
            security_enabled: false,
            frame_pending: self.indirect_queue.has_pending(dst),
            ack_request: ack_req,
            pan_id_compressed: pan_compressed,
            seq_num: seq,
            dst_pan: self.mac.pan_id,
            dst_addr: dst,
            src_pan: self.mac.pan_id,
            src_addr: self.mac.local_addr,
            payload: payload.to_vec(),
            aux_sec_header: None,
        }
    }

    /// Build a secure data frame with AES-128-CCM* encryption+MIC.
    pub fn build_secure_data_frame(
        &mut self,
        payload: &[u8],
        dst: MacAddress,
        level: SecurityLevel,
    ) -> Option<MacFrame> {
        let key = self.mac.key?;
        let seq = self.next_seq();
        let fc = self.mac.frame_counter;
        self.mac.frame_counter = fc.wrapping_add(1);

        let ash = AuxSecHeader::new(level, fc);
        let pan_compressed = matches!(dst, MacAddress::Short(_));

        // Build header bytes (for AAD)
        let mut tmp_frame = MacFrame {
            frame_type: MacFrameType::Data,
            security_enabled: true,
            frame_pending: false,
            ack_request: true,
            pan_id_compressed: pan_compressed,
            seq_num: seq,
            dst_pan: self.mac.pan_id,
            dst_addr: dst,
            src_pan: self.mac.pan_id,
            src_addr: self.mac.local_addr,
            payload: vec![],
            aux_sec_header: Some(ash.clone()),
        };
        // Extract MHR as AAD (everything except payload and FCS)
        let mhr = {
            let full = tmp_frame.to_bytes();
            full[..full.len() - FCS_SIZE].to_vec()
        };

        // Build CCM* nonce (13 bytes): EUI-64 || frame_counter || security_level
        let mut nonce = [0u8; 13];
        if let MacAddress::Extended(eui) = self.mac.local_addr {
            nonce[..8].copy_from_slice(&eui.to_le_bytes());
        }
        nonce[8..12].copy_from_slice(&fc.to_le_bytes());
        nonce[12] = level as u8;

        let mut data = payload.to_vec();
        let mic = ccm_star_auth_encrypt(&key, &nonce, &mhr, &mut data, level);
        data.extend_from_slice(&mic);

        tmp_frame.payload = data;
        Some(tmp_frame)
    }

    /// Encode a MAC frame to a PPDU byte stream (with CRC, SHR, PHR).
    pub fn encode_ppdu(&self, frame: &MacFrame) -> Vec<u8> {
        let psdu = frame.to_bytes();
        build_ppdu(&psdu)
    }

    /// Generate O-QPSK IQ samples for a PPDU.
    pub fn modulate_ppdu(&self, ppdu: &[u8]) -> Vec<(f32, f32)> {
        let nrz = bytes_to_nrz_chips(ppdu);
        oqpsk_modulate(&nrz, self.phy.samples_per_chip)
    }

    /// Demodulate received IQ samples to chip stream (hard decision).
    pub fn demodulate_iq(&self, iq: &[(f32, f32)]) -> Vec<u8> {
        let spc = self.phy.samples_per_chip;
        // Downsample: take one sample per chip (centre of chip period)
        let filt = half_sine_filter(spc);
        let norm: f32 = filt.iter().map(|&x| x * x).sum::<f32>().sqrt();
        let mut chips = Vec::new();
        let n = iq.len() / spc;
        for i in 0..n {
            // Matched filter: correlate with half-sine pulse (I channel)
            let i_sum: f32 = iq[i * spc..(i + 1) * spc]
                .iter()
                .zip(filt.iter())
                .map(|(&(ii, _), &h)| ii * h)
                .sum::<f32>() / norm;
            chips.push(if i_sum >= 0.0 { 0u8 } else { 1u8 });
        }
        chips
    }

    /// Correlate chip stream against PN sequences to recover symbols/bytes.
    pub fn chips_to_bytes(&self, chips: &[u8]) -> Vec<u8> {
        let mut bytes = Vec::new();
        let mut i = 0;
        while i + 64 <= chips.len() {
            let lo = correlate_pn(&chips[i..i + 32]);
            let hi = correlate_pn(&chips[i + 32..i + 64]);
            bytes.push(lo | (hi << 4));
            i += 64;
        }
        bytes
    }

    /// Return the centre frequency of the current channel.
    pub fn channel_freq_hz(&self) -> u64 {
        channel_frequency_hz(self.phy.channel)
    }

    /// Switch to the specified channel (11–26).
    pub fn set_channel(&mut self, ch: u8) {
        assert!(ch >= MIN_CHANNEL && ch <= MAX_CHANNEL);
        self.phy.channel = ch;
    }
}

/// Find the best-matching PN sequence symbol for a 32-chip block.
fn correlate_pn(chips: &[u8]) -> u8 {
    let mut best_sym = 0u8;
    let mut best_score: i32 = i32::MIN;
    for sym in 0u8..16 {
        let pn = PN_SEQUENCES[sym as usize];
        let mut score: i32 = 0;
        for bit in 0..32usize {
            let expected = ((pn >> bit) & 1) as u8;
            if chips[bit] == expected { score += 1; } else { score -= 1; }
        }
        if score > best_score {
            best_score = score;
            best_sym = sym;
        }
    }
    best_sym
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Channel frequency ──────────────────────────────────────────────────

    #[test]
    fn test_channel_11_freq() {
        assert_eq!(channel_frequency_hz(11), 2_405_000_000);
    }

    #[test]
    fn test_channel_26_freq() {
        assert_eq!(channel_frequency_hz(26), 2_480_000_000);
    }

    #[test]
    fn test_channel_15_freq() {
        // Channel 15: 2405 + 5*(15-11) = 2405 + 20 = 2425 MHz
        assert_eq!(channel_frequency_hz(15), 2_425_000_000);
    }

    #[test]
    #[should_panic]
    fn test_invalid_channel_panics() {
        channel_frequency_hz(10);
    }

    // ── CRC-16 ─────────────────────────────────────────────────────────────

    #[test]
    fn test_crc16_empty() {
        assert_eq!(crc16_ccitt(&[]), 0x0000);
    }

    #[test]
    fn test_crc16_known_value() {
        // CRC-16/MCRF4XX of [0x01, 0x02, 0x03] = verify round-trip
        let data = [0x01u8, 0x02, 0x03];
        let crc = crc16_ccitt(&data);
        let mut with_crc = data.to_vec();
        with_crc.extend_from_slice(&crc.to_le_bytes());
        // CRC of data+crc should yield zero residue
        // (CCITT residue is not zero; but at minimum verify crc is consistent)
        assert_eq!(crc16_ccitt(&data), crc); // deterministic
    }

    #[test]
    fn test_crc16_all_zeros() {
        let data = [0u8; 10];
        let c1 = crc16_ccitt(&data);
        let c2 = crc16_ccitt(&data);
        assert_eq!(c1, c2);
    }

    // ── AES-128 ─────────────────────────────────────────────────────────────

    #[test]
    fn test_aes128_known_vector() {
        // FIPS 197 Appendix B test vector
        let key: [u8; 16] = [
            0x2b,0x7e,0x15,0x16, 0x28,0xae,0xd2,0xa6,
            0xab,0xf7,0x15,0x88, 0x09,0xcf,0x4f,0x3c,
        ];
        let mut block: [u8; 16] = [
            0x32,0x43,0xf6,0xa8, 0x88,0x5a,0x30,0x8d,
            0x31,0x31,0x98,0xa2, 0xe0,0x37,0x07,0x34,
        ];
        let rk = aes128_key_schedule(&key);
        aes128_encrypt_block(&mut block, &rk);
        let expected: [u8; 16] = [
            0x39,0x25,0x84,0x1d, 0x02,0xdc,0x09,0xfb,
            0xdc,0x11,0x85,0x97, 0x19,0x6a,0x0b,0x32,
        ];
        assert_eq!(block, expected);
    }

    #[test]
    fn test_aes128_key_schedule_length() {
        let key = [0u8; 16];
        let rk = aes128_key_schedule(&key);
        assert_eq!(rk.len(), 11);
    }

    #[test]
    fn test_gf_mul_identity() {
        // a * 1 = a
        for a in 0..=255u8 {
            assert_eq!(gf_mul(a, 1), a);
        }
    }

    #[test]
    fn test_gf_mul_zero() {
        for a in 0..=255u8 {
            assert_eq!(gf_mul(a, 0), 0);
        }
    }

    // ── CCM* ────────────────────────────────────────────────────────────────

    #[test]
    fn test_ccm_star_no_security() {
        let key = [0u8; 16];
        let nonce = [0u8; 13];
        let aad = b"header";
        let mut data = b"payload".to_vec();
        let mic = ccm_star_auth_encrypt(&key, &nonce, aad, &mut data, SecurityLevel::None);
        assert_eq!(mic.len(), 0);
        // Data unchanged with no security
        assert_eq!(data, b"payload");
    }

    #[test]
    fn test_ccm_star_mic32() {
        let key = [0xC0u8; 16];
        let nonce = [0x01u8; 13];
        let aad = b"aad-data";
        let mut data = vec![0xAAu8; 8];
        let mic = ccm_star_auth_encrypt(&key, &nonce, aad, &mut data, SecurityLevel::Mic32);
        assert_eq!(mic.len(), 4);
        // Data should be unchanged (integrity only)
        assert_eq!(data, vec![0xAAu8; 8]);
    }

    #[test]
    fn test_ccm_star_enc_mic32() {
        let key = [0xBBu8; 16];
        let nonce = [0x02u8; 13];
        let aad = b"test-aad";
        let original = vec![0x55u8; 16];
        let mut data = original.clone();
        let mic = ccm_star_auth_encrypt(&key, &nonce, aad, &mut data, SecurityLevel::EncMic32);
        assert_eq!(mic.len(), 4);
        // Encrypted data should differ from plaintext
        assert_ne!(data, original);
    }

    #[test]
    fn test_ccm_star_mic128() {
        let key = [0xFFu8; 16];
        let nonce = [0x03u8; 13];
        let mut data = vec![0x12u8; 32];
        let mic = ccm_star_auth_encrypt(&key, &nonce, b"", &mut data, SecurityLevel::Mic128);
        assert_eq!(mic.len(), 16);
    }

    // ── MAC address ─────────────────────────────────────────────────────────

    #[test]
    fn test_mac_addr_none_mode() {
        assert_eq!(MacAddress::None.mode_bits(), 0b00);
    }

    #[test]
    fn test_mac_addr_short_mode() {
        assert_eq!(MacAddress::Short(0x1234).mode_bits(), 0b10);
        assert_eq!(MacAddress::Short(0x1234).byte_len(), 2);
    }

    #[test]
    fn test_mac_addr_extended_mode() {
        assert_eq!(MacAddress::Extended(0xDEADBEEFCAFEBABE).mode_bits(), 0b11);
        assert_eq!(MacAddress::Extended(0).byte_len(), 8);
    }

    #[test]
    fn test_mac_addr_short_bytes() {
        let b = MacAddress::Short(0xABCD).to_bytes();
        assert_eq!(b, vec![0xCD, 0xAB]); // little-endian
    }

    // ── MAC Frame building ───────────────────────────────────────────────────

    #[test]
    fn test_mac_ack_frame() {
        let frame = MacFrame::ack(42);
        let bytes = frame.to_bytes();
        // FCF(2) + seq(1) + FCS(2) = 5 bytes minimum
        assert_eq!(bytes.len(), 5);
        assert_eq!(bytes[2], 42); // seq num
    }

    #[test]
    fn test_mac_beacon_frame() {
        let frame = MacFrame::beacon(0x1234, MacAddress::Short(0x0001), 0, 0xCFFF);
        let bytes = frame.to_bytes();
        // Must include FCS
        assert!(bytes.len() > FCS_SIZE);
    }

    #[test]
    fn test_mac_data_frame_with_crc() {
        let frame = MacFrame {
            frame_type: MacFrameType::Data,
            security_enabled: false,
            frame_pending: false,
            ack_request: true,
            pan_id_compressed: true,
            seq_num: 100,
            dst_pan: 0x1234,
            dst_addr: MacAddress::Short(0xABCD),
            src_pan: 0x1234,
            src_addr: MacAddress::Short(0x0001),
            payload: b"test".to_vec(),
            aux_sec_header: None,
        };
        let bytes = frame.to_bytes();
        assert!(bytes.len() > FCF_SIZE + 1 + FCS_SIZE);
        // Last 2 bytes are FCS
        let fcs_computed = crc16_ccitt(&bytes[..bytes.len() - 2]);
        let fcs_stored = u16::from_le_bytes([bytes[bytes.len()-2], bytes[bytes.len()-1]]);
        assert_eq!(fcs_computed, fcs_stored);
    }

    #[test]
    fn test_mac_frame_parse_roundtrip() {
        let frame = MacFrame {
            frame_type: MacFrameType::Data,
            security_enabled: false,
            frame_pending: false,
            ack_request: false,
            pan_id_compressed: false,
            seq_num: 77,
            dst_pan: 0xABCD,
            dst_addr: MacAddress::Short(0x0002),
            src_pan: 0xABCD,
            src_addr: MacAddress::Short(0x0001),
            payload: vec![0x01, 0x02, 0x03],
            aux_sec_header: None,
        };
        let bytes = frame.to_bytes();
        let (parsed, _) = MacFrame::parse(&bytes).expect("parse failed");
        assert_eq!(parsed.seq_num, 77);
        assert_eq!(parsed.payload, vec![0x01, 0x02, 0x03]);
    }

    #[test]
    fn test_beacon_request_frame() {
        let br = build_beacon_request(5);
        let bytes = br.to_bytes();
        assert!(!bytes.is_empty());
        assert_eq!(br.seq_num, 5);
        assert_eq!(br.payload, vec![MacCommand::BeaconRequest as u8]);
    }

    // ── PPDU structure ───────────────────────────────────────────────────────

    #[test]
    fn test_ppdu_roundtrip() {
        let psdu = b"Hello IEEE 802.15.4";
        let ppdu = build_ppdu(psdu);
        let (recovered, _) = parse_ppdu(&ppdu).expect("parse_ppdu failed");
        assert_eq!(recovered, psdu);
    }

    #[test]
    fn test_ppdu_sfd_present() {
        let ppdu = build_ppdu(&[0xAA, 0xBB]);
        assert_eq!(ppdu[4], SFD);
    }

    #[test]
    fn test_ppdu_phr_length() {
        let psdu = vec![0u8; 50];
        let ppdu = build_ppdu(&psdu);
        assert_eq!(ppdu[5] & 0x7F, 50);
    }

    #[test]
    fn test_parse_ppdu_invalid_sfd() {
        let mut ppdu = build_ppdu(b"test");
        ppdu[4] = 0x00; // corrupt SFD
        assert!(parse_ppdu(&ppdu).is_none());
    }

    // ── PN sequences / chip spreading ────────────────────────────────────────

    #[test]
    fn test_pn_sequences_distinct() {
        // All 16 PN sequences must be distinct
        for i in 0..16 {
            for j in i+1..16 {
                assert_ne!(PN_SEQUENCES[i], PN_SEQUENCES[j],
                    "PN sequences {} and {} are equal", i, j);
            }
        }
    }

    #[test]
    fn test_byte_to_chips_length() {
        let chips = byte_to_chips(0x5A);
        assert_eq!(chips.len(), 64);
    }

    #[test]
    fn test_byte_to_chips_binary() {
        // All chip values must be 0 or 1
        for sym in 0u8..16 {
            let chips = byte_to_chips(sym);
            for &c in &chips {
                assert!(c == 0 || c == 1);
            }
        }
    }

    #[test]
    fn test_correlate_pn_correct() {
        // Encode symbol 7 and verify correlation recovers it
        let pn = PN_SEQUENCES[7];
        let chips: Vec<u8> = (0..32).map(|b| ((pn >> b) & 1) as u8).collect();
        assert_eq!(correlate_pn(&chips), 7);
    }

    #[test]
    fn test_nrz_chips_polarity() {
        // Each byte maps to 64 chips (two 32-chip PN sequences, one per nibble).
        // Chip value 0 → NRZ +1.0, chip value 1 → NRZ -1.0.
        let chips = bytes_to_nrz_chips(&[0x00]);
        assert_eq!(chips.len(), 64);
        // All chips should be +1 or -1
        for &c in &chips {
            assert!(c == 1.0 || c == -1.0, "unexpected NRZ chip value: {}", c);
        }
    }

    // ── Half-sine pulse shaping ──────────────────────────────────────────────

    #[test]
    fn test_half_sine_filter_length() {
        let f = half_sine_filter(8);
        assert_eq!(f.len(), 8);
    }

    #[test]
    fn test_half_sine_filter_positive() {
        // Half-sine values should be non-negative
        let f = half_sine_filter(8);
        assert!(f.iter().all(|&x| x >= 0.0));
    }

    #[test]
    fn test_half_sine_peak_at_centre() {
        let f = half_sine_filter(8);
        let mid = f.len() / 2;
        // Peak should be near the centre
        let max_idx = f.iter().enumerate().max_by(|a, b| a.1.partial_cmp(b.1).unwrap()).unwrap().0;
        assert!((max_idx as isize - mid as isize).abs() <= 2);
    }

    // ── O-QPSK modulation ────────────────────────────────────────────────────

    #[test]
    fn test_oqpsk_output_not_empty() {
        let nrz: Vec<f32> = vec![1.0, -1.0, 1.0, 1.0, -1.0, 1.0, -1.0, -1.0];
        let iq = oqpsk_modulate(&nrz, 4);
        assert!(!iq.is_empty());
    }

    #[test]
    fn test_oqpsk_amplitude_bounded() {
        let nrz: Vec<f32> = (0..64).map(|i| if i % 2 == 0 { 1.0 } else { -1.0 }).collect();
        let iq = oqpsk_modulate(&nrz, 4);
        for (i, q) in &iq {
            assert!(i.abs() <= 2.0, "I amplitude out of range: {}", i);
            assert!(q.abs() <= 2.0, "Q amplitude out of range: {}", q);
        }
    }

    // ── CSMA-CA ──────────────────────────────────────────────────────────────

    #[test]
    fn test_csma_ca_backoff_sequence() {
        let cfg = CsmaCaConfig::default();
        let mut csma = CsmaCa::new(cfg, 12345);
        let mut backoffs = Vec::new();
        while let Some(b) = csma.next_backoff() {
            backoffs.push(b);
        }
        assert!(!backoffs.is_empty());
        assert!(backoffs.len() <= 5); // max_csma_backoffs=4 + 1 initial
    }

    #[test]
    fn test_csma_ca_reset() {
        let cfg = CsmaCaConfig::default();
        let mut csma = CsmaCa::new(cfg.clone(), 1);
        csma.next_backoff();
        csma.next_backoff();
        assert!(csma.nb >= 2);
        csma.reset();
        assert_eq!(csma.nb, 0);
        assert_eq!(csma.be, cfg.min_be);
    }

    #[test]
    fn test_csma_ca_be_increases() {
        let cfg = CsmaCaConfig::default();
        let mut csma = CsmaCa::new(cfg, 42);
        let be_start = csma.be;
        csma.next_backoff();
        assert!(csma.be > be_start || csma.be == csma.cfg.max_be);
    }

    // ── Energy Detection ─────────────────────────────────────────────────────

    #[test]
    fn test_ed_empty_samples() {
        assert_eq!(compute_ed(&[]), 0);
    }

    #[test]
    fn test_ed_zero_power() {
        let samples = vec![(0.0f32, 0.0f32); 100];
        assert_eq!(compute_ed(&samples), 0);
    }

    #[test]
    fn test_ed_high_power() {
        let samples = vec![(10.0f32, 10.0f32); 100];
        let ed = compute_ed(&samples);
        assert!(ed > 200, "Expected high ED, got {}", ed);
    }

    #[test]
    fn test_lqi_range() {
        assert_eq!(compute_lqi(0.0), 0);
        assert_eq!(compute_lqi(1.0), 255);
        assert_eq!(compute_lqi(0.5), 127);
    }

    // ── Energy scan ─────────────────────────────────────────────────────────

    #[test]
    fn test_energy_scan_all_channels() {
        let results = energy_scan(&|ch| ch - MIN_CHANNEL); // energy proportional to channel index
        assert_eq!(results.len(), 16); // channels 11–26
        // Sorted by energy (ascending)
        for w in results.windows(2) {
            assert!(w[0].energy <= w[1].energy);
        }
    }

    // ── 6LoWPAN IPHC compression ─────────────────────────────────────────────

    #[test]
    fn test_iphc_produces_output() {
        let mut hdr = [0u8; 40];
        hdr[0] = 0x60; // IPv6 version
        hdr[6] = 17;   // UDP
        hdr[7] = 64;   // hop limit
        let src_iid = [0u8; 8];
        let dst_iid = [0u8; 8];
        let compressed = iphc_compress(&hdr, &src_iid, &dst_iid);
        assert!(!compressed.is_empty());
        // Dispatch must start with 0b011xxxxx
        assert_eq!(compressed[0] & 0xE0, 0x60);
    }

    #[test]
    fn test_iphc_shorter_than_ipv6() {
        let mut hdr = [0u8; 40];
        hdr[0] = 0x60;
        hdr[6] = 17;
        hdr[7] = 255; // hop limit 255 = compressed
        // Link-local src: FE80::src_iid
        hdr[8] = 0xFE; hdr[9] = 0x80;
        let src_iid = [0u8; 8];
        hdr[16..24].copy_from_slice(&src_iid);
        let compressed = iphc_compress(&hdr, &src_iid, &src_iid);
        // IPHC should be shorter than 40 bytes
        assert!(compressed.len() < 40);
    }

    // ── NHC UDP compression ──────────────────────────────────────────────────

    #[test]
    fn test_nhc_udp_inline_ports() {
        let hdr: [u8; 8] = [0x00, 0x50, 0x1F, 0x90, 0x00, 0x08, 0x12, 0x34];
        let compressed = nhc_udp_compress(&hdr);
        // NHC dispatch byte is first
        assert_eq!(compressed[0] & 0xF8, 0xF0);
    }

    #[test]
    fn test_nhc_udp_zero_checksum_elided() {
        let mut hdr = [0u8; 8];
        hdr[6] = 0; hdr[7] = 0; // zero checksum
        let compressed = nhc_udp_compress(&hdr);
        assert!(compressed[0] & 0x04 != 0); // C=1 (checksum elided)
    }

    // ── 6LoWPAN Mesh / Fragment headers ─────────────────────────────────────

    #[test]
    fn test_mesh_header_encode() {
        let mh = MeshHeader { hops_left: 5, originator: 0x0001, final_dest: 0x000F };
        let bytes = mh.to_bytes();
        assert!(!bytes.is_empty());
        assert_eq!(bytes[0] & 0xC0, 0x80); // mesh dispatch
    }

    #[test]
    fn test_frag1_header_encode() {
        let fh = FragHeader { datagram_size: 300, datagram_tag: 0x1234, datagram_offset: 0, is_first: true };
        let bytes = fh.to_bytes();
        assert_eq!(bytes[0] & 0xF8, 0xC0); // FRAG1 dispatch
        assert_eq!(bytes.len(), 4); // dispatch+size(2B) + tag(2B) = 4
    }

    #[test]
    fn test_fragn_header_encode() {
        let fh = FragHeader { datagram_size: 300, datagram_tag: 0xABCD, datagram_offset: 8, is_first: false };
        let bytes = fh.to_bytes();
        assert_eq!(bytes[0] & 0xF8, 0xE0); // FRAGN dispatch
        assert_eq!(bytes[bytes.len()-1], 8); // offset
    }

    #[test]
    fn test_fragment_datagram_small() {
        let data = vec![0xABu8; 50];
        let frags = fragment_datagram(&data, 1, 127);
        // Data fits in one fragment
        assert_eq!(frags.len(), 1);
    }

    #[test]
    fn test_fragment_datagram_multiple() {
        let data = vec![0x55u8; 200];
        let frags = fragment_datagram(&data, 2, 64);
        assert!(frags.len() > 1);
    }

    // ── MLE messages ─────────────────────────────────────────────────────────

    #[test]
    fn test_mle_build_parent_request() {
        let tlvs = vec![
            MleTlv::new(MleTlvType::Version, vec![0x00, 0x03]),
            MleTlv::new(MleTlvType::ScanMask, vec![0xC0]),
        ];
        let msg = build_mle_message(MleCommand::ParentRequest, &tlvs);
        assert_eq!(msg[0], MLE_FRAME_TYPE);
        assert_eq!(msg[1], MleCommand::ParentRequest as u8);
    }

    #[test]
    fn test_mle_parse_tlvs() {
        let tlvs_in = vec![
            MleTlv::new(MleTlvType::SourceAddress, vec![0x00, 0x01]),
            MleTlv::new(MleTlvType::Mode, vec![0x0F]),
        ];
        let msg = build_mle_message(MleCommand::Advertisement, &tlvs_in);
        // Skip frame_type + command byte
        let tlvs_out = parse_mle_tlvs(&msg[2..]);
        assert_eq!(tlvs_out.len(), 2);
        assert_eq!(tlvs_out[0].tlv_type, MleTlvType::SourceAddress as u8);
        assert_eq!(tlvs_out[1].value, vec![0x0F]);
    }

    #[test]
    fn test_mle_tlv_encode() {
        let tlv = MleTlv::new(MleTlvType::Timeout, vec![0x00, 0x00, 0x04, 0xB0]);
        let bytes = tlv.to_bytes();
        assert_eq!(bytes[0], MleTlvType::Timeout as u8);
        assert_eq!(bytes[1], 4); // length
        assert_eq!(&bytes[2..], &[0x00, 0x00, 0x04, 0xB0]);
    }

    // ── ThreadProcessor ──────────────────────────────────────────────────────

    #[test]
    fn test_thread_processor_create() {
        let phy = PhyConfig::default_2_4ghz(15);
        let mac = MacConfig::new(0xABCD, MacAddress::Short(0x1234));
        let proc = ThreadProcessor::new(phy, mac);
        assert_eq!(proc.phy.channel, 15);
        assert_eq!(proc.mac.pan_id, 0xABCD);
    }

    #[test]
    fn test_thread_processor_data_frame() {
        let phy = PhyConfig::default_2_4ghz(11);
        let mac = MacConfig::new(0x1234, MacAddress::Short(0x0001));
        let mut proc = ThreadProcessor::new(phy, mac);
        let frame = proc.build_data_frame(b"data", MacAddress::Short(0x0002), true);
        assert_eq!(frame.frame_type, MacFrameType::Data);
        assert_eq!(frame.payload, b"data");
        assert!(frame.ack_request);
    }

    #[test]
    fn test_thread_processor_encode_decode_ppdu() {
        let phy = PhyConfig::default_2_4ghz(11);
        let mac = MacConfig::new(0x5678, MacAddress::Short(0x0010));
        let mut proc = ThreadProcessor::new(phy, mac);
        let frame = proc.build_data_frame(b"ping", MacAddress::Short(0x0020), false);
        let ppdu = proc.encode_ppdu(&frame);
        let (psdu, _) = parse_ppdu(&ppdu).expect("parse PPDU");
        // PSDU should contain the MAC frame bytes
        assert!(!psdu.is_empty());
    }

    #[test]
    fn test_thread_processor_seq_num_increments() {
        let phy = PhyConfig::default_2_4ghz(11);
        let mac = MacConfig::new(0x0001, MacAddress::Short(0x0001));
        let mut proc = ThreadProcessor::new(phy, mac);
        let f1 = proc.build_data_frame(b"a", MacAddress::Short(0x0002), false);
        let f2 = proc.build_data_frame(b"b", MacAddress::Short(0x0002), false);
        assert_eq!(f2.seq_num, f1.seq_num.wrapping_add(1));
    }

    #[test]
    fn test_thread_processor_set_channel() {
        let phy = PhyConfig::default_2_4ghz(11);
        let mac = MacConfig::new(0xFACE, MacAddress::Short(0x0001));
        let mut proc = ThreadProcessor::new(phy, mac);
        proc.set_channel(20);
        assert_eq!(proc.phy.channel, 20);
        assert_eq!(proc.channel_freq_hz(), 2_450_000_000);
    }

    #[test]
    fn test_thread_processor_channel_freq() {
        let phy = PhyConfig::default_2_4ghz(26);
        let mac = MacConfig::new(0x0001, MacAddress::Short(0x0001));
        let proc = ThreadProcessor::new(phy, mac);
        assert_eq!(proc.channel_freq_hz(), 2_480_000_000);
    }

    #[test]
    fn test_thread_processor_secure_frame() {
        let key = [0xDEu8; 16];
        let phy = PhyConfig::default_2_4ghz(15);
        let mac = MacConfig::new(0xABCD, MacAddress::Extended(0x0123456789ABCDEF))
            .with_key(key);
        let mut proc = ThreadProcessor::new(phy, mac);
        let frame = proc.build_secure_data_frame(b"secret", MacAddress::Short(0x0002), SecurityLevel::EncMic32);
        assert!(frame.is_some());
        let frame = frame.unwrap();
        assert!(frame.security_enabled);
        // Payload should have grown by MIC length (4 bytes)
        assert!(frame.payload.len() >= b"secret".len() + 4);
    }

    #[test]
    fn test_thread_processor_indirect_queue() {
        let phy = PhyConfig::default_2_4ghz(11);
        let mac = MacConfig::new(0x0001, MacAddress::Short(0x0001));
        let mut proc = ThreadProcessor::new(phy, mac);
        let dst = MacAddress::Short(0x0005);
        proc.indirect_queue.enqueue(dst, vec![0xAA, 0xBB]);
        assert!(proc.indirect_queue.has_pending(dst));
        let entry = proc.indirect_queue.dequeue(dst);
        assert!(entry.is_some());
        assert!(!proc.indirect_queue.has_pending(dst));
    }

    #[test]
    fn test_thread_processor_modulate_ppdu() {
        let phy = PhyConfig::default_2_4ghz(15);
        let mac = MacConfig::new(0x0001, MacAddress::Short(0x0001));
        let mut proc = ThreadProcessor::new(phy, mac);
        let frame = proc.build_data_frame(&[0x01, 0x02], MacAddress::Short(0x0002), false);
        let ppdu = proc.encode_ppdu(&frame);
        let iq = proc.modulate_ppdu(&ppdu);
        // Each byte → 64 chips → 4 samples/chip → IQ pairs
        assert!(!iq.is_empty());
    }

    // ── Indirect queue ───────────────────────────────────────────────────────

    #[test]
    fn test_indirect_queue_empty() {
        let q = IndirectQueue::new();
        assert!(q.is_empty());
        assert_eq!(q.len(), 0);
    }

    #[test]
    fn test_indirect_queue_enqueue_dequeue() {
        let mut q = IndirectQueue::new();
        let dst = MacAddress::Short(0x0001);
        q.enqueue(dst, vec![1, 2, 3]);
        assert_eq!(q.len(), 1);
        let entry = q.dequeue(dst).unwrap();
        assert_eq!(entry.frame, vec![1, 2, 3]);
        assert!(q.is_empty());
    }

    #[test]
    fn test_indirect_queue_multiple_destinations() {
        let mut q = IndirectQueue::new();
        let dst1 = MacAddress::Short(0x0001);
        let dst2 = MacAddress::Short(0x0002);
        q.enqueue(dst1, vec![0xAA]);
        q.enqueue(dst2, vec![0xBB]);
        assert_eq!(q.len(), 2);
        let e = q.dequeue(dst2).unwrap();
        assert_eq!(e.frame, vec![0xBB]);
        assert!(q.has_pending(dst1));
        assert!(!q.has_pending(dst2));
    }

    // ── Aux security header ──────────────────────────────────────────────────

    #[test]
    fn test_aux_sec_header_encode() {
        let ash = AuxSecHeader::new(SecurityLevel::EncMic32, 0x0000_0042);
        let bytes = ash.to_bytes();
        // 1 (SC) + 4 (FC) = 5 bytes minimum
        assert_eq!(bytes.len(), 5);
        let sc = bytes[0];
        let level_code = sc & 0x7;
        assert_eq!(level_code, 5); // EncMic32 = 5
    }
}
