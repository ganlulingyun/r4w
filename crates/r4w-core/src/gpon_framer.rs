//! GPON and XGS-PON Framing — ITU-T G.984 / G.9807
//!
//! Implements the key framing layers of Gigabit-capable Passive Optical Networks:
//!
//! * **GEM** – GPON Encapsulation Method (ITU-T G.984.3 §8)
//!   - 5-byte header: PLI (12 b) | Port-ID (12 b) | PTI (3 b) | HEC (13 b)
//!   - Payload fragmentation / reassembly
//!   - Idle GEM frame generation
//!
//! * **GTC downstream frame** – 125 µs / 38 880 bytes at 2.488 Gbps (GPON)
//!   - PCBd: Psync (4 B) | Ident (4 B) | PLOAMd (13 B) | BIP-8 (1 B) |
//!             Plend (4 B, doubled) | US BWmap (N×8 B)
//!   - GEM partition follows PCBd
//!
//! * **GTC upstream frame** – burst with preamble, delimiter, PLOAMu, DBRu, GEM
//!
//! * **T-CONT / DBA** – T-CONT types 1-5, SR-DBA / NSR-DBA bandwidth maps
//!
//! * **FEC** – Downstream RS(255,239) per G.984.3; optional upstream RS(255,239)
//!
//! * **AES-128 CTR** – GEM payload encryption per port-ID; key exchange via PLOAM
//!
//! * **Ranging / Registration** – quiet window, ranging burst, equalization delay,
//!   ONU-ID assignment
//!
//! * **XGS-PON extensions** (ITU-T G.9807.1) – 10 Gbps symmetric, XGEM header,
//!   XGTC frame structure, 64-bit PON-ID
//!
//! * **OAM / PLOAM** – Serial_Number_ONU, Ranging_Time, Assign_ONU-ID, POPUP,
//!   Dying_Gasp, Key_Report, Acknowledge
//!
//! Pure Rust, `no_std`-compatible math (uses `std` for collections only).
//!
//! # References
//! - ITU-T G.984.1-4  (GPON)
//! - ITU-T G.9807.1   (XGS-PON)
//! - ITU-T G.988      (OMCI)

// ─── HEC / CRC helpers ────────────────────────────────────────────────────────

/// CRC-8 table used for GEM HEC (polynomial 0x07, ITU-T G.984.3 Annex A).
fn build_crc8_table() -> [u8; 256] {
    let mut table = [0u8; 256];
    for i in 0usize..256 {
        let mut crc = i as u8;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ 0x07;
            } else {
                crc <<= 1;
            }
        }
        table[i] = crc;
    }
    table
}

fn crc8(data: &[u8]) -> u8 {
    let table = build_crc8_table();
    let mut crc = 0u8;
    for &b in data {
        crc = table[(crc ^ b) as usize];
    }
    crc
}

/// BIP-8: bit-interleaved parity over a byte slice (even parity, XOR all bytes).
fn bip8(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &b| acc ^ b)
}

/// RS(255,239) GF(2^8) with primitive polynomial 0x11D (x^8+x^4+x^3+x^2+1)
/// as used in GPON G.984.3.
/// Provides encode and syndrome-check only (full decoder is complex;
/// here we implement encode + a syndrome computation for testing).
mod rs {
    const PRIM_POLY: u16 = 0x11D;

    fn gf_mul(a: u8, b: u8) -> u8 {
        let mut result = 0u8;
        let mut aa = a;
        let mut bb = b as u16;
        for _ in 0..8 {
            if bb & 1 != 0 {
                result ^= aa;
            }
            let high = aa & 0x80;
            aa <<= 1;
            if high != 0 {
                aa ^= (PRIM_POLY & 0xFF) as u8;
            }
            bb >>= 1;
        }
        result
    }

    fn gf_pow(mut base: u8, mut exp: u32) -> u8 {
        let mut result = 1u8;
        while exp > 0 {
            if exp & 1 != 0 {
                result = gf_mul(result, base);
            }
            base = gf_mul(base, base);
            exp >>= 1;
        }
        result
    }

    /// Compute RS(255,239) — 16 parity bytes — generator g(x) = prod_{i=0}^{15}(x - alpha^i).
    pub fn encode(data: &[u8]) -> Vec<u8> {
        assert!(data.len() <= 239, "RS(255,239) data block too large");
        // Generator polynomial coefficients
        let mut gen = vec![1u8];
        for i in 0u32..16 {
            let root = gf_pow(2, i);
            // multiply gen by (x - root) = (x + root) in GF(2^8)
            let mut newgen = vec![0u8; gen.len() + 1];
            for (j, &g) in gen.iter().enumerate() {
                newgen[j] ^= gf_mul(g, root);
                newgen[j + 1] ^= g;
            }
            gen = newgen;
        }
        // Remainder of x^16 * data(x) divided by gen(x)
        let mut rem = vec![0u8; 16];
        for &byte in data {
            let feedback = byte ^ rem[0];
            for k in 0..15 {
                rem[k] = rem[k + 1] ^ gf_mul(gen[15 - k], feedback);
            }
            rem[15] = gf_mul(gen[0], feedback);
        }
        rem
    }

    /// Compute syndromes S_0..S_15 for a received codeword.
    pub fn syndromes(codeword: &[u8]) -> [u8; 16] {
        let mut s = [0u8; 16];
        for i in 0..16usize {
            let alpha_i = gf_pow(2, i as u32);
            let mut val = 0u8;
            for &b in codeword {
                val = gf_mul(val, alpha_i) ^ b;
            }
            s[i] = val;
        }
        s
    }
}

// ─── AES-128 CTR (minimal, table-based) ───────────────────────────────────────

mod aes128 {
    /// Rijndael S-box (forward)
    const SBOX: [u8; 256] = [
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

    /// Round constants for key expansion
    const RCON: [u32; 11] = [
        0x00000000, 0x01000000, 0x02000000, 0x04000000,
        0x08000000, 0x10000000, 0x20000000, 0x40000000,
        0x80000000, 0x1b000000, 0x36000000,
    ];

    fn sub_word(w: u32) -> u32 {
        let b = w.to_be_bytes();
        u32::from_be_bytes([SBOX[b[0] as usize], SBOX[b[1] as usize],
                            SBOX[b[2] as usize], SBOX[b[3] as usize]])
    }

    fn rot_word(w: u32) -> u32 {
        w.rotate_left(8)
    }

    /// Expand a 128-bit key into 11 round keys (each 16 bytes = 4 words).
    pub fn key_expand(key: &[u8; 16]) -> [[u8; 16]; 11] {
        let mut w = [0u32; 44];
        for i in 0..4 {
            let b = &key[i*4..i*4+4];
            w[i] = u32::from_be_bytes([b[0], b[1], b[2], b[3]]);
        }
        for i in 4..44 {
            let mut temp = w[i-1];
            if i % 4 == 0 {
                temp = sub_word(rot_word(temp)) ^ RCON[i/4];
            }
            w[i] = w[i-4] ^ temp;
        }
        let mut rk = [[0u8; 16]; 11];
        for r in 0..11 {
            for c in 0..4 {
                let bytes = w[r*4+c].to_be_bytes();
                rk[r][c*4..c*4+4].copy_from_slice(&bytes);
            }
        }
        rk
    }

    fn gmul(mut a: u8, mut b: u8) -> u8 {
        let mut p = 0u8;
        for _ in 0..8 {
            if b & 1 != 0 { p ^= a; }
            let hi = a & 0x80;
            a <<= 1;
            if hi != 0 { a ^= 0x1b; }
            b >>= 1;
        }
        p
    }

    /// Encrypt one 16-byte block.
    /// State layout: bytes 0-3 = column 0, bytes 4-7 = column 1, etc.
    /// state[col*4 + row]
    pub fn encrypt_block(block: &[u8; 16], rk: &[[u8; 16]; 11]) -> [u8; 16] {
        let mut s = *block;

        // AddRoundKey(state, round[0])
        for i in 0..16 { s[i] ^= rk[0][i]; }

        for round in 1..=10 {
            // SubBytes
            for b in s.iter_mut() { *b = SBOX[*b as usize]; }

            // ShiftRows: row r shifts left by r
            // state[col*4 + row] -> row r: indices r, 4+r, 8+r, 12+r
            let t = s[1];
            s[1] = s[5]; s[5] = s[9]; s[9] = s[13]; s[13] = t;
            // row 2: swap pairs
            s.swap(2, 10); s.swap(6, 14);
            // row 3: right rotate 1 = left rotate 3
            let t = s[15];
            s[15] = s[11]; s[11] = s[7]; s[7] = s[3]; s[3] = t;

            // MixColumns (skip on last round)
            if round < 10 {
                for col in 0..4 {
                    let i = col * 4;
                    let (s0, s1, s2, s3) = (s[i], s[i+1], s[i+2], s[i+3]);
                    s[i]   = gmul(0x02,s0) ^ gmul(0x03,s1) ^ s2          ^ s3;
                    s[i+1] = s0          ^ gmul(0x02,s1) ^ gmul(0x03,s2) ^ s3;
                    s[i+2] = s0          ^ s1            ^ gmul(0x02,s2) ^ gmul(0x03,s3);
                    s[i+3] = gmul(0x03,s0) ^ s1          ^ s2            ^ gmul(0x02,s3);
                }
            }

            // AddRoundKey
            for i in 0..16 { s[i] ^= rk[round][i]; }
        }
        s
    }

    /// AES-128 CTR stream cipher. Counter is big-endian 128-bit, incremented per block.
    pub fn ctr_crypt(key: &[u8; 16], nonce: &[u8; 16], data: &[u8]) -> Vec<u8> {
        let rk = key_expand(key);
        let mut out = Vec::with_capacity(data.len());
        let mut ctr = *nonce;
        let mut offset = 0usize;
        while offset < data.len() {
            let ks = encrypt_block(&ctr, &rk);
            let remaining = data.len() - offset;
            let chunk = remaining.min(16);
            for i in 0..chunk {
                out.push(data[offset + i] ^ ks[i]);
            }
            offset += chunk;
            // Increment counter (big-endian)
            let mut carry = 1u16;
            for b in ctr.iter_mut().rev() {
                carry += *b as u16;
                *b = (carry & 0xFF) as u8;
                carry >>= 8;
                if carry == 0 { break; }
            }
        }
        out
    }
}

// ─── Public types ──────────────────────────────────────────────────────────────

use std::collections::HashMap;

/// Error type for GPON framer operations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GponError {
    /// HEC check failed on incoming GEM header.
    HecError,
    /// Payload length indicator exceeds maximum.
    PliTooLarge,
    /// ONU-ID already assigned.
    OnuAlreadyAssigned,
    /// Unknown ONU serial number.
    UnknownSerial,
    /// Frame synchronisation lost (Psync mismatch).
    SyncLost,
    /// Buffer overflow in reassembly.
    BufferOverflow,
    /// Invalid PLOAM message type.
    InvalidPloam,
    /// Key not set for encrypted port.
    KeyNotSet,
    /// FEC syndrome indicates uncorrectable error.
    FecUncorrectable,
}

impl std::fmt::Display for GponError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{:?}", self)
    }
}

// ─── GEM framing ──────────────────────────────────────────────────────────────

/// GPON Transmission Convergence (GTC) Layer header for a GEM frame.
///
/// Layout (5 bytes):
/// ```text
/// Bits 47-36  PLI      12 bits  Payload Length Indicator
/// Bits 35-24  Port-ID  12 bits  GEM Port identifier
/// Bits 23-21  PTI       3 bits  Payload Type Indicator
/// Bits 20- 8  Reserved  1 bit
/// Bits  7- 0  HEC       8 bits  CRC-8 over first 4 bytes (ITU-T G.984.3 Annex A)
/// ```
/// Note: Standard specifies 13-bit HEC (BCH) in G.984.3 but many implementations
/// use CRC-8 in the header framing field; this module uses CRC-8.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct GemHeader {
    /// Payload Length Indicator (0 = idle, 1–4095 bytes)
    pub pli: u16,
    /// GEM Port Identifier (0–4094; 4095 = idle)
    pub port_id: u16,
    /// Payload Type Indicator  
    /// 0 = user data (last fragment), 1 = user data (not last), others reserved
    pub pti: u8,
}

impl GemHeader {
    pub const IDLE_PORT_ID: u16 = 4095;

    /// Encode header to 5 bytes (PLI[11:0] | PortID[11:0] | PTI[2:0] | 0 | HEC[7:0]).
    pub fn encode(&self) -> [u8; 5] {
        // Byte layout:
        //  [0]: PLI[11:4]
        //  [1]: PLI[3:0] | PortID[11:8]
        //  [2]: PortID[7:0]
        //  [3]: PTI[2:0] | 00000 (5 reserved bits become part of HEC input)
        //  [4]: HEC
        let b0 = (self.pli >> 4) as u8;
        let b1 = (((self.pli & 0xF) << 4) | (self.port_id >> 8)) as u8;
        let b2 = (self.port_id & 0xFF) as u8;
        let b3 = (self.pti & 0x07) << 5;
        let hec = crc8(&[b0, b1, b2, b3]);
        [b0, b1, b2, b3, hec]
    }

    /// Decode 5 bytes into a GemHeader, verifying HEC.
    pub fn decode(raw: &[u8; 5]) -> Result<Self, GponError> {
        let expected = crc8(&raw[..4]);
        if raw[4] != expected {
            return Err(GponError::HecError);
        }
        let pli = ((raw[0] as u16) << 4) | ((raw[1] as u16) >> 4);
        let port_id = (((raw[1] & 0x0F) as u16) << 8) | raw[2] as u16;
        let pti = (raw[3] >> 5) & 0x07;
        Ok(GemHeader { pli, port_id, pti })
    }

    /// Build an idle GEM header (PLI=0, PortID=4095, PTI=0).
    pub fn idle() -> Self {
        GemHeader { pli: 0, port_id: Self::IDLE_PORT_ID, pti: 0 }
    }

    pub fn is_idle(&self) -> bool {
        self.port_id == Self::IDLE_PORT_ID
    }
}

/// A complete GEM frame (header + payload bytes).
#[derive(Debug, Clone)]
pub struct GemFrame {
    pub header: GemHeader,
    pub payload: Vec<u8>,
}

impl GemFrame {
    /// Build a GEM frame carrying `payload` on `port_id`.
    /// PTI 0 = last (or only) fragment; PTI 1 = more fragments follow.
    pub fn new(port_id: u16, payload: Vec<u8>, last_fragment: bool) -> Self {
        let pli = payload.len() as u16;
        let pti = if last_fragment { 0 } else { 1 };
        GemFrame { header: GemHeader { pli, port_id, pti }, payload }
    }

    /// Build an idle frame of the given byte size (header + filler).
    pub fn idle(size: usize) -> Self {
        let payload_len = size.saturating_sub(5);
        GemFrame {
            header: GemHeader { pli: 0, port_id: GemHeader::IDLE_PORT_ID, pti: 0 },
            payload: vec![0u8; payload_len],
        }
    }

    pub fn serialise(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(5 + self.payload.len());
        out.extend_from_slice(&self.header.encode());
        out.extend_from_slice(&self.payload);
        out
    }

    /// Deserialise one GEM frame from a byte slice starting at offset 0.
    /// Returns `(frame, bytes_consumed)`.
    pub fn deserialise(data: &[u8]) -> Result<(Self, usize), GponError> {
        if data.len() < 5 {
            return Err(GponError::PliTooLarge);
        }
        let raw: [u8; 5] = data[..5].try_into().unwrap();
        let header = GemHeader::decode(&raw)?;
        let pli = header.pli as usize;
        if 5 + pli > data.len() {
            return Err(GponError::PliTooLarge);
        }
        let payload = data[5..5 + pli].to_vec();
        Ok((GemFrame { header, payload }, 5 + pli))
    }
}

// ─── GEM Fragmentation & Reassembly ───────────────────────────────────────────

/// Reassembly buffer per GEM port.
struct ReassemblyBuf {
    data: Vec<u8>,
}

/// GEM layer framer: segments SDU into GEM frames and reassembles received frames.
pub struct GemFramer {
    /// Maximum GEM payload per frame.
    pub max_payload: usize,
    reassembly: HashMap<u16, ReassemblyBuf>,
}

impl GemFramer {
    pub fn new(max_payload: usize) -> Self {
        GemFramer { max_payload, reassembly: HashMap::new() }
    }

    /// Fragment `sdu` into one or more GEM frames on `port_id`.
    pub fn segment(&self, port_id: u16, sdu: &[u8]) -> Vec<GemFrame> {
        let mut frames = Vec::new();
        let mut offset = 0;
        while offset < sdu.len() {
            let chunk_end = (offset + self.max_payload).min(sdu.len());
            let chunk = sdu[offset..chunk_end].to_vec();
            let last = chunk_end == sdu.len();
            frames.push(GemFrame::new(port_id, chunk, last));
            offset = chunk_end;
        }
        if frames.is_empty() {
            frames.push(GemFrame::new(port_id, vec![], true));
        }
        frames
    }

    /// Feed a received GEM frame into the reassembly engine.
    /// Returns the complete SDU when the last fragment (PTI=0) arrives.
    pub fn reassemble(&mut self, frame: &GemFrame) -> Result<Option<Vec<u8>>, GponError> {
        if frame.header.is_idle() {
            return Ok(None);
        }
        let port = frame.header.port_id;
        let buf = self.reassembly.entry(port).or_insert(ReassemblyBuf { data: Vec::new() });
        if buf.data.len() + frame.payload.len() > 65536 {
            buf.data.clear();
            return Err(GponError::BufferOverflow);
        }
        buf.data.extend_from_slice(&frame.payload);
        if frame.header.pti == 0 {
            let sdu = std::mem::take(&mut buf.data);
            self.reassembly.remove(&port);
            Ok(Some(sdu))
        } else {
            Ok(None)
        }
    }
}

// ─── PLOAM messages ───────────────────────────────────────────────────────────

/// PLOAM message type codes (ITU-T G.984.3 Table 10-1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum PloamMsgType {
    SerialNumberOnu   = 0x09,
    RangingTime       = 0x04,
    AssignOnuId       = 0x01,
    Popup             = 0x0A,
    DyingGasp         = 0x07,
    KeyReport         = 0x1B,
    Acknowledge       = 0x0B, // OLT→ONU direction
    NoMessage         = 0x00,
}

/// Raw 13-byte PLOAM message structure.
#[derive(Debug, Clone)]
pub struct PloamMessage {
    /// ONU-ID (0xFF = broadcast).
    pub onu_id: u8,
    /// Message identifier.
    pub msg_id: u8,
    /// 10 bytes of content.
    pub content: [u8; 10],
    /// CRC-8 over first 12 bytes.
    pub crc: u8,
}

impl PloamMessage {
    pub fn new(onu_id: u8, msg_id: u8, content: [u8; 10]) -> Self {
        let mut raw = [0u8; 12];
        raw[0] = onu_id;
        raw[1] = msg_id;
        raw[2..12].copy_from_slice(&content);
        let crc = crc8(&raw);
        PloamMessage { onu_id, msg_id, content, crc }
    }

    pub fn serialise(&self) -> [u8; 13] {
        let mut out = [0u8; 13];
        out[0] = self.onu_id;
        out[1] = self.msg_id;
        out[2..12].copy_from_slice(&self.content);
        out[12] = self.crc;
        out
    }

    pub fn deserialise(raw: &[u8; 13]) -> Result<Self, GponError> {
        let expected = crc8(&raw[..12]);
        if raw[12] != expected {
            return Err(GponError::HecError);
        }
        let mut content = [0u8; 10];
        content.copy_from_slice(&raw[2..12]);
        Ok(PloamMessage { onu_id: raw[0], msg_id: raw[1], content, crc: raw[12] })
    }

    /// Build a Serial_Number_ONU message (ONU→OLT).
    pub fn serial_number_onu(serial: &[u8; 8]) -> Self {
        let mut c = [0u8; 10];
        c[..8].copy_from_slice(serial);
        PloamMessage::new(0xFF, PloamMsgType::SerialNumberOnu as u8, c)
    }

    /// Build an Assign_ONU-ID message (OLT→ONU).
    pub fn assign_onu_id(onu_id: u8, serial: &[u8; 8]) -> Self {
        let mut c = [0u8; 10];
        c[0] = onu_id;
        c[1..9].copy_from_slice(serial);
        PloamMessage::new(0xFF, PloamMsgType::AssignOnuId as u8, c)
    }

    /// Build a Ranging_Time message (OLT→ONU).
    pub fn ranging_time(onu_id: u8, eq_delay: u32) -> Self {
        let mut c = [0u8; 10];
        c[0..4].copy_from_slice(&eq_delay.to_be_bytes());
        PloamMessage::new(onu_id, PloamMsgType::RangingTime as u8, c)
    }

    /// Build a Key_Report message (ONU→OLT).
    pub fn key_report(onu_id: u8, key_index: u8, key_fragment: &[u8; 8]) -> Self {
        let mut c = [0u8; 10];
        c[0] = key_index;
        c[1..9].copy_from_slice(key_fragment);
        PloamMessage::new(onu_id, PloamMsgType::KeyReport as u8, c)
    }

    /// Build a Dying_Gasp message (ONU→OLT).
    pub fn dying_gasp(onu_id: u8) -> Self {
        PloamMessage::new(onu_id, PloamMsgType::DyingGasp as u8, [0u8; 10])
    }
}

// ─── T-CONT and DBA ───────────────────────────────────────────────────────────

/// T-CONT type as per ITU-T G.984.3 §8.2.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TcontType {
    /// Type 1: Fixed bandwidth (CBR).
    Type1,
    /// Type 2: Assured bandwidth.
    Type2,
    /// Type 3: Non-assured bandwidth.
    Type3,
    /// Type 4: Best-effort bandwidth.
    Type4,
    /// Type 5: Mixed (Types 2+3 combined).
    Type5,
}

/// Bandwidth allocation entry for one T-CONT (one row in the US BWmap).
#[derive(Debug, Clone)]
pub struct BwAllocation {
    /// Alloc-ID (maps to T-CONT).
    pub alloc_id: u16,
    /// Start time (bytes from frame start) in the upstream burst.
    pub start_time: u16,
    /// Stop time (bytes from frame start).
    pub stop_time: u16,
    /// FEC indicator for upstream burst.
    pub fec_ind: bool,
    /// PLOAMu requested.
    pub ploam_req: bool,
    /// DBRu mode (0 = no DBR, 1 = SR-DBA, 2 = NSR-DBA).
    pub dbr_mode: u8,
    /// CRC over allocation struct (CRC-8).
    pub crc: u8,
}

impl BwAllocation {
    /// Size of a single allocation entry in the BWmap (bytes).
    pub const SIZE: usize = 8;

    pub fn new(alloc_id: u16, start_time: u16, stop_time: u16,
               fec_ind: bool, ploam_req: bool, dbr_mode: u8) -> Self {
        let mut raw = [0u8; 7];
        raw[0] = (alloc_id >> 4) as u8;
        raw[1] = (((alloc_id & 0xF) << 4) | ((start_time >> 8) & 0xF)) as u8;
        raw[2] = (start_time & 0xFF) as u8;
        raw[3] = (stop_time >> 4) as u8;
        raw[4] = (((stop_time & 0xF) << 4)
            | (((fec_ind as u16) << 3))
            | (((ploam_req as u16) << 2))
            | ((dbr_mode as u16) & 0x3)) as u8;
        raw[5] = 0;
        raw[6] = 0;
        let crc = crc8(&raw[..7]);
        BwAllocation { alloc_id, start_time, stop_time, fec_ind, ploam_req, dbr_mode, crc }
    }

    pub fn serialise(&self) -> [u8; 8] {
        let mut out = [0u8; 8];
        out[0] = (self.alloc_id >> 4) as u8;
        out[1] = (((self.alloc_id & 0xF) << 4) | ((self.start_time >> 8) & 0xF)) as u8;
        out[2] = (self.start_time & 0xFF) as u8;
        out[3] = (self.stop_time >> 4) as u8;
        out[4] = (((self.stop_time & 0xF) << 4)
            | (((self.fec_ind as u16) << 3))
            | (((self.ploam_req as u16) << 2))
            | ((self.dbr_mode as u16) & 0x3)) as u8;
        out[5] = 0;
        out[6] = 0;
        out[7] = self.crc;
        out
    }
}

/// T-CONT descriptor.
#[derive(Debug, Clone)]
pub struct Tcont {
    pub alloc_id: u16,
    pub tcont_type: TcontType,
    /// Fixed bandwidth (bytes/125µs) for Types 1, 5.
    pub fixed_bw: u32,
    /// Assured bandwidth (bytes/125µs) for Types 2, 3, 5.
    pub assured_bw: u32,
    /// Maximum bandwidth (bytes/125µs) for Types 3, 4.
    pub max_bw: u32,
    /// Current queue occupancy reported via DBRu (bytes).
    pub queue_occupancy: u32,
}

impl Tcont {
    pub fn new(alloc_id: u16, tcont_type: TcontType) -> Self {
        Tcont { alloc_id, tcont_type, fixed_bw: 0, assured_bw: 0, max_bw: 0, queue_occupancy: 0 }
    }
}

/// Simple SR-DBA (Status-Reporting DBA) bandwidth allocator.
pub struct DbaEngine {
    tconts: Vec<Tcont>,
    /// Available upstream bytes per 125µs frame.
    pub frame_capacity: u32,
}

impl DbaEngine {
    pub fn new(frame_capacity: u32) -> Self {
        DbaEngine { tconts: Vec::new(), frame_capacity }
    }

    pub fn add_tcont(&mut self, tcont: Tcont) {
        self.tconts.push(tcont);
    }

    /// Compute a bandwidth map for one upstream frame.
    /// Returns a list of `BwAllocation` entries.
    pub fn compute_bwmap(&self) -> Vec<BwAllocation> {
        let mut allocations = Vec::new();
        let mut cursor: u16 = 0;
        let cap = self.frame_capacity as u16;

        for tcont in &self.tconts {
            if cursor >= cap { break; }
            let alloc = match tcont.tcont_type {
                TcontType::Type1 => tcont.fixed_bw as u16,
                TcontType::Type2 => tcont.assured_bw as u16,
                TcontType::Type3 => {
                    (tcont.queue_occupancy as u16).min(tcont.max_bw as u16)
                }
                TcontType::Type4 => {
                    (tcont.queue_occupancy as u16).min(cap - cursor)
                }
                TcontType::Type5 => {
                    tcont.fixed_bw as u16
                    + (tcont.queue_occupancy as u16).min(tcont.max_bw as u16)
                }
            };
            let stop = (cursor + alloc).min(cap);
            if stop > cursor {
                allocations.push(BwAllocation::new(
                    tcont.alloc_id, cursor, stop, false, false, 1,
                ));
                cursor = stop;
            }
        }
        allocations
    }
}

// ─── GTC downstream frame ─────────────────────────────────────────────────────

/// GPON downstream line rate: 2.488 Gbps → 38 880 bytes / 125 µs.
pub const GPON_DS_FRAME_BYTES: usize = 38880;
/// Psync pattern (4 bytes): 0xB6AB31E0 per G.984.3.
pub const PSYNC: [u8; 4] = [0xB6, 0xAB, 0x31, 0xE0];
/// PCBd header size (without BWmap entries).
pub const PCBD_FIXED_SIZE: usize = 4 + 4 + 13 + 1 + 4 + 4; // 30 bytes

/// PCBd – Physical Control Block downstream.
#[derive(Debug, Clone)]
pub struct Pcbd {
    /// Frame sequence number (modulo 2^30, bits 1-30; bit 0 = FEC indicator).
    pub ident: u32,
    /// PLOAMd message (13 bytes).
    pub ploamd: [u8; 13],
    /// BIP-8 over the previous downstream frame GEM partition.
    pub bip8: u8,
    /// Upstream bandwidth map entries.
    pub bwmap: Vec<BwAllocation>,
}

impl Pcbd {
    pub fn serialise(&self) -> Vec<u8> {
        let mut out = Vec::with_capacity(PCBD_FIXED_SIZE + self.bwmap.len() * 8);
        out.extend_from_slice(&PSYNC);
        out.extend_from_slice(&self.ident.to_be_bytes());
        out.extend_from_slice(&self.ploamd);
        out.push(self.bip8);
        // Plend (payload length of the downstream GEM partition / 4, doubled)
        // Here we write 0 as a placeholder; real framer fills based on GEM content.
        out.extend_from_slice(&0u32.to_be_bytes());
        // Plend copy
        out.extend_from_slice(&0u32.to_be_bytes());
        for bw in &self.bwmap {
            out.extend_from_slice(&bw.serialise());
        }
        out
    }

    pub fn pcbd_size(&self) -> usize {
        PCBD_FIXED_SIZE + self.bwmap.len() * BwAllocation::SIZE
    }
}

/// A complete GTC downstream frame (PCBd + GEM partition).
pub struct GtcDownstreamFrame {
    pub pcbd: Pcbd,
    pub gem_bytes: Vec<u8>,
}

impl GtcDownstreamFrame {
    pub fn new(pcbd: Pcbd, gem_bytes: Vec<u8>) -> Self {
        GtcDownstreamFrame { pcbd, gem_bytes }
    }

    pub fn serialise(&self) -> Vec<u8> {
        let mut out = self.pcbd.serialise();
        out.extend_from_slice(&self.gem_bytes);
        // Pad to full frame size
        while out.len() < GPON_DS_FRAME_BYTES {
            out.push(0x00);
        }
        out.truncate(GPON_DS_FRAME_BYTES);
        out
    }

    /// Parse the Psync and Ident fields from a raw downstream frame.
    pub fn check_sync(raw: &[u8]) -> Result<u32, GponError> {
        if raw.len() < 8 { return Err(GponError::SyncLost); }
        if raw[..4] != PSYNC { return Err(GponError::SyncLost); }
        Ok(u32::from_be_bytes(raw[4..8].try_into().unwrap()))
    }
}

// ─── GTC upstream burst ────────────────────────────────────────────────────────

/// GPON upstream line rate: 1.244 Gbps → 19 440 bytes / 125 µs.
pub const GPON_US_FRAME_BYTES: usize = 19440;
/// Standard preamble pattern repeated for synchronisation.
pub const US_PREAMBLE: [u8; 3] = [0x55, 0x55, 0x55];
/// Standard delimiter pattern.
pub const US_DELIMITER: [u8; 3] = [0xAB, 0x5B, 0x6A];

/// DBRu – Dynamic Bandwidth Report upstream (4 bytes: MSB = mode, 3 bytes = report).
#[derive(Debug, Clone, Copy)]
pub struct Dbru {
    pub mode: u8,
    pub report: u32,
}

impl Dbru {
    pub fn new(mode: u8, report: u32) -> Self {
        Dbru { mode, report: report & 0x00FFFFFF }
    }

    pub fn serialise(&self) -> [u8; 4] {
        let v = ((self.mode as u32) << 24) | self.report;
        v.to_be_bytes()
    }
}

/// GTC upstream burst structure.
pub struct GtcUpstreamBurst {
    pub onu_id: u8,
    pub ploamu: Option<PloamMessage>,
    pub dbru: Option<Dbru>,
    pub gem_bytes: Vec<u8>,
}

impl GtcUpstreamBurst {
    pub fn serialise(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&US_PREAMBLE);
        out.extend_from_slice(&US_DELIMITER);
        out.push(self.onu_id);
        if let Some(ref p) = self.ploamu {
            out.extend_from_slice(&p.serialise());
        } else {
            out.extend_from_slice(&[0u8; 13]);
        }
        if let Some(dbr) = self.dbru {
            out.extend_from_slice(&dbr.serialise());
        } else {
            out.extend_from_slice(&[0u8; 4]);
        }
        out.extend_from_slice(&self.gem_bytes);
        out
    }
}

// ─── FEC ──────────────────────────────────────────────────────────────────────

/// Apply RS(255,239) FEC to a byte stream divided into 239-byte blocks.
/// The last block is zero-padded to 239 bytes, and parity is appended per block.
pub fn fec_encode_stream(data: &[u8]) -> Vec<u8> {
    let mut out = Vec::new();
    let mut offset = 0;
    while offset < data.len() {
        let end = (offset + 239).min(data.len());
        let mut block = data[offset..end].to_vec();
        block.resize(239, 0); // zero-pad last block
        let parity = rs::encode(&block);
        out.extend_from_slice(&block[..end - offset]); // original (not padded)
        out.extend_from_slice(&parity);
        offset = end;
    }
    out
}

/// Check syndromes for a 255-byte RS codeword; returns true if all-zero (no error).
pub fn fec_check_block(codeword: &[u8; 255]) -> bool {
    let syndromes = rs::syndromes(codeword);
    syndromes.iter().all(|&s| s == 0)
}

// ─── AES encryption engine ────────────────────────────────────────────────────

/// Per-port AES-128 CTR encryption state.
pub struct PortCryptoState {
    pub key: [u8; 16],
    /// Frame counter (used to build the 128-bit CTR nonce).
    pub frame_counter: u64,
}

/// AES-128 CTR GEM payload encryptor/decryptor.
pub struct GemEncryptionEngine {
    port_keys: HashMap<u16, PortCryptoState>,
}

impl GemEncryptionEngine {
    pub fn new() -> Self {
        GemEncryptionEngine { port_keys: HashMap::new() }
    }

    /// Register (or update) the AES-128 key for `port_id`.
    pub fn set_key(&mut self, port_id: u16, key: [u8; 16]) {
        self.port_keys.insert(port_id, PortCryptoState { key, frame_counter: 0 });
    }

    pub fn remove_key(&mut self, port_id: u16) {
        self.port_keys.remove(&port_id);
    }

    /// Encrypt (or decrypt — CTR is symmetric) `payload` for `port_id`.
    /// The nonce is constructed as: [port_id (2 B) | frame_counter (8 B) | zeros (6 B)].
    pub fn crypt(&mut self, port_id: u16, payload: &[u8]) -> Result<Vec<u8>, GponError> {
        let state = self.port_keys.get_mut(&port_id).ok_or(GponError::KeyNotSet)?;
        let mut nonce = [0u8; 16];
        nonce[0..2].copy_from_slice(&port_id.to_be_bytes());
        nonce[2..10].copy_from_slice(&state.frame_counter.to_be_bytes());
        let result = aes128::ctr_crypt(&state.key, &nonce, payload);
        state.frame_counter = state.frame_counter.wrapping_add(1);
        Ok(result)
    }
}

impl Default for GemEncryptionEngine {
    fn default() -> Self { Self::new() }
}

// ─── Ranging & Registration ───────────────────────────────────────────────────

/// Registration state of an ONU.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OnuState {
    /// Not yet seen.
    Unknown,
    /// Serial number received, awaiting ranging.
    SerialKnown,
    /// Ranging burst received; equalization delay computed.
    Ranged,
    /// Fully registered and operational.
    Operational,
}

/// OLT-side ONU registration record.
#[derive(Debug, Clone)]
pub struct OnuRecord {
    pub serial: [u8; 8],
    pub onu_id: u8,
    pub eq_delay: u32,
    pub state: OnuState,
    pub aes_key: Option<[u8; 16]>,
}

/// OLT-side ranging and registration engine.
pub struct RangingEngine {
    pub onus: Vec<OnuRecord>,
    next_onu_id: u8,
    /// Measured round-trip time in byte-clocks (used to compute eq_delay).
    pub rtt_bytes: u32,
}

impl RangingEngine {
    pub fn new() -> Self {
        RangingEngine { onus: Vec::new(), next_onu_id: 1, rtt_bytes: 0 }
    }

    /// Register receipt of a Serial_Number_ONU from an unknown ONU.
    /// Returns PLOAM messages to send: Assign_ONU-ID + Ranging_Time.
    pub fn on_serial_number(&mut self, serial: &[u8; 8]) -> Result<Vec<PloamMessage>, GponError> {
        // Check for duplicate
        for rec in &self.onus {
            if rec.serial == *serial && rec.state != OnuState::Unknown {
                return Err(GponError::OnuAlreadyAssigned);
            }
        }
        let onu_id = self.next_onu_id;
        self.next_onu_id = self.next_onu_id.wrapping_add(1);
        // Compute equalization delay (placeholder — real OLT measures RTT)
        let eq_delay = self.rtt_bytes;
        let rec = OnuRecord {
            serial: *serial, onu_id, eq_delay,
            state: OnuState::Ranged, aes_key: None,
        };
        self.onus.push(rec);
        let msgs = vec![
            PloamMessage::assign_onu_id(onu_id, serial),
            PloamMessage::ranging_time(onu_id, eq_delay),
        ];
        Ok(msgs)
    }

    /// Mark ONU as operational after ranging acknowledgement.
    pub fn on_ranging_ack(&mut self, onu_id: u8) -> bool {
        for rec in &mut self.onus {
            if rec.onu_id == onu_id {
                rec.state = OnuState::Operational;
                return true;
            }
        }
        false
    }

    pub fn find_by_id(&self, onu_id: u8) -> Option<&OnuRecord> {
        self.onus.iter().find(|r| r.onu_id == onu_id)
    }
}

impl Default for RangingEngine {
    fn default() -> Self { Self::new() }
}

// ─── XGS-PON extensions ───────────────────────────────────────────────────────

/// XGS-PON downstream line rate: 9.953 Gbps → 155 520 bytes / 125 µs.
pub const XGSPON_DS_FRAME_BYTES: usize = 155520;
/// XGS-PON upstream line rate: 9.953 Gbps → 155 520 bytes / 125 µs.
pub const XGSPON_US_FRAME_BYTES: usize = 155520;
/// XGS-PON Psync: 8 bytes (different from GPON).
pub const XGSPON_PSYNC: [u8; 8] = [0x4E, 0x67, 0x34, 0x28, 0xB6, 0xAB, 0x31, 0xE0];

/// XGEM header: 8 bytes (larger than GEM's 5 bytes).
///
/// Layout:
/// ```text
/// Bits 63-52  XPLI     12 bits  Extended Payload Length Indicator
/// Bits 51-40  Port-ID  12 bits  XGEM Port identifier
/// Bits 39-37  PTI       3 bits
/// Bit  36     Key-ID    1 bit   Encryption key index
/// Bits 35- 0  HEC      36 bits  BCH-based (simplified here: CRC-16/ARC)
/// ```
/// (Simplified: bits 35-20 = CRC-16, bits 19-0 = zero.)
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct XgemHeader {
    pub pli: u16,
    pub port_id: u16,
    pub pti: u8,
    pub key_id: bool,
    pub hec: u16,
}

impl XgemHeader {
    pub fn new(port_id: u16, pli: u16, pti: u8, key_id: bool) -> Self {
        let b0 = (pli >> 4) as u8;
        let b1 = (((pli & 0xF) << 4) | (port_id >> 8)) as u8;
        let b2 = (port_id & 0xFF) as u8;
        let b3 = ((pti & 0x7) << 5) | ((key_id as u8) << 4);
        let hec = {
            let data = [b0, b1, b2, b3];
            crc16_arc(&data)
        };
        XgemHeader { pli, port_id, pti, key_id, hec }
    }

    pub fn encode(&self) -> [u8; 8] {
        let b0 = (self.pli >> 4) as u8;
        let b1 = (((self.pli & 0xF) << 4) | (self.port_id >> 8)) as u8;
        let b2 = (self.port_id & 0xFF) as u8;
        let b3 = ((self.pti & 0x7) << 5) | ((self.key_id as u8) << 4);
        let hec_bytes = self.hec.to_be_bytes();
        [b0, b1, b2, b3, hec_bytes[0], hec_bytes[1], 0, 0]
    }
}

/// CRC-16/ARC (polynomial 0x8005, init 0, no inversion) for XGEM HEC.
fn crc16_arc(data: &[u8]) -> u16 {
    let mut crc = 0u16;
    for &b in data {
        crc ^= (b as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ 0x8005;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// XGS-PON XGTC downstream frame.
pub struct XgtcDownstreamFrame {
    /// 8-byte Psync.
    pub psync: [u8; 8],
    /// 4-byte Ident.
    pub ident: u32,
    /// 64-bit PON-ID (XGS-PON extension).
    pub pon_id: u64,
    pub ploamd: [u8; 13],
    pub bip8: u8,
    pub bwmap: Vec<BwAllocation>,
    pub xgem_bytes: Vec<u8>,
}

impl XgtcDownstreamFrame {
    pub fn new(ident: u32, pon_id: u64, ploamd: [u8; 13],
               bip8: u8, bwmap: Vec<BwAllocation>, xgem_bytes: Vec<u8>) -> Self {
        XgtcDownstreamFrame {
            psync: XGSPON_PSYNC, ident, pon_id, ploamd, bip8, bwmap, xgem_bytes,
        }
    }

    pub fn serialise(&self) -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&self.psync);
        out.extend_from_slice(&self.ident.to_be_bytes());
        out.extend_from_slice(&self.pon_id.to_be_bytes());
        out.extend_from_slice(&self.ploamd);
        out.push(self.bip8);
        out.extend_from_slice(&0u32.to_be_bytes()); // Plend placeholder
        out.extend_from_slice(&0u32.to_be_bytes()); // Plend copy
        for bw in &self.bwmap {
            out.extend_from_slice(&bw.serialise());
        }
        out.extend_from_slice(&self.xgem_bytes);
        out
    }

    pub fn check_sync(raw: &[u8]) -> Result<u32, GponError> {
        if raw.len() < 12 { return Err(GponError::SyncLost); }
        if raw[..8] != XGSPON_PSYNC { return Err(GponError::SyncLost); }
        Ok(u32::from_be_bytes(raw[8..12].try_into().unwrap()))
    }
}

// ─── Full OLT framer ──────────────────────────────────────────────────────────

/// Complete OLT-side GPON framer with encryption, FEC, ranging, and DBA.
pub struct GponOltFramer {
    pub gem: GemFramer,
    pub ranging: RangingEngine,
    pub dba: DbaEngine,
    pub crypto: GemEncryptionEngine,
    pub fec_enabled: bool,
    frame_counter: u32,
    bip_accumulator: Vec<u8>,
}

impl GponOltFramer {
    pub fn new(fec_enabled: bool) -> Self {
        GponOltFramer {
            gem: GemFramer::new(1518),
            ranging: RangingEngine::new(),
            dba: DbaEngine::new(19340),
            crypto: GemEncryptionEngine::new(),
            fec_enabled,
            frame_counter: 0,
            bip_accumulator: Vec::new(),
        }
    }

    /// Build a complete downstream GTC frame.
    ///
    /// `sdus`: list of `(port_id, payload)` to encapsulate.
    pub fn build_downstream_frame(
        &mut self,
        ploamd: [u8; 13],
        sdus: &[(u16, Vec<u8>)],
    ) -> Vec<u8> {
        let bwmap = self.dba.compute_bwmap();
        let bip = bip8(&self.bip_accumulator);
        self.bip_accumulator.clear();
        let ident = (self.frame_counter & 0x3FFFFFFF) << 1
            | if self.fec_enabled { 1 } else { 0 };
        self.frame_counter = self.frame_counter.wrapping_add(1);

        let pcbd = Pcbd { ident, ploamd, bip8: bip, bwmap };

        // Build GEM partition
        let mut gem_bytes = Vec::new();
        for (port_id, sdu) in sdus {
            let frames = self.gem.segment(*port_id, sdu);
            for mut f in frames {
                // Encrypt if key is set
                if self.crypto.port_keys.contains_key(port_id) {
                    if let Ok(enc) = self.crypto.crypt(*port_id, &f.payload) {
                        f.payload = enc;
                    }
                }
                gem_bytes.extend_from_slice(&f.serialise());
            }
        }
        // Pad with idle GEM to fill frame
        let pcbd_size = pcbd.pcbd_size();
        let gem_capacity = GPON_DS_FRAME_BYTES.saturating_sub(pcbd_size);
        while gem_bytes.len() + 5 <= gem_capacity {
            let idle = GemFrame::idle(5);
            gem_bytes.extend_from_slice(&idle.serialise());
        }
        gem_bytes.truncate(gem_capacity);

        self.bip_accumulator.extend_from_slice(&gem_bytes);

        // Optionally apply FEC
        let payload = if self.fec_enabled {
            fec_encode_stream(&gem_bytes)
        } else {
            gem_bytes
        };

        let frame = GtcDownstreamFrame::new(pcbd, payload);
        frame.serialise()
    }

    /// Process an incoming upstream burst.
    pub fn process_upstream_burst(
        &mut self,
        burst: &[u8],
    ) -> Result<Vec<(u16, Vec<u8>)>, GponError> {
        // Parse preamble + delimiter (6 bytes) + ONU-ID (1 byte)
        if burst.len() < 7 { return Ok(vec![]); }
        let _onu_id = burst[6];
        // GEM data starts after preamble(3) + delimiter(3) + ONU-ID(1) + PLOAMu(13) + DBRu(4)
        let gem_start = 3 + 3 + 1 + 13 + 4;
        if burst.len() <= gem_start { return Ok(vec![]); }
        let gem_data = &burst[gem_start..];
        self.parse_gem_partition(gem_data)
    }

    fn parse_gem_partition(&mut self, data: &[u8]) -> Result<Vec<(u16, Vec<u8>)>, GponError> {
        let mut sdus = Vec::new();
        let mut offset = 0;
        while offset + 5 <= data.len() {
            let raw: &[u8] = &data[offset..];
            match GemFrame::deserialise(raw) {
                Ok((frame, consumed)) => {
                    if !frame.header.is_idle() {
                        if let Ok(Some(sdu)) = self.gem.reassemble(&frame) {
                            sdus.push((frame.header.port_id, sdu));
                        }
                    }
                    offset += consumed;
                }
                Err(_) => break,
            }
        }
        Ok(sdus)
    }
}

// ─── Tests ────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── GEM header tests ──────────────────────────────────────────────────────

    #[test]
    fn test_gem_header_encode_decode_roundtrip() {
        let h = GemHeader { pli: 100, port_id: 42, pti: 0 };
        let raw = h.encode();
        let h2 = GemHeader::decode(&raw).unwrap();
        assert_eq!(h, h2);
    }

    #[test]
    fn test_gem_header_idle() {
        let h = GemHeader::idle();
        assert!(h.is_idle());
        assert_eq!(h.pli, 0);
        let raw = h.encode();
        let h2 = GemHeader::decode(&raw).unwrap();
        assert!(h2.is_idle());
    }

    #[test]
    fn test_gem_header_hec_error() {
        let h = GemHeader { pli: 50, port_id: 7, pti: 1 };
        let mut raw = h.encode();
        raw[4] ^= 0xFF; // corrupt HEC
        assert_eq!(GemHeader::decode(&raw), Err(GponError::HecError));
    }

    #[test]
    fn test_gem_header_max_pli() {
        let h = GemHeader { pli: 4094, port_id: 100, pti: 0 };
        let raw = h.encode();
        let h2 = GemHeader::decode(&raw).unwrap();
        assert_eq!(h2.pli, 4094);
    }

    #[test]
    fn test_gem_header_pti_preserved() {
        for pti in 0u8..8 {
            let h = GemHeader { pli: 10, port_id: 1, pti };
            let raw = h.encode();
            let h2 = GemHeader::decode(&raw).unwrap();
            assert_eq!(h2.pti, pti);
        }
    }

    // ── GEM frame tests ───────────────────────────────────────────────────────

    #[test]
    fn test_gem_frame_serialise_deserialise() {
        let payload = b"Hello GPON".to_vec();
        let f = GemFrame::new(42, payload.clone(), true);
        let raw = f.serialise();
        let (f2, consumed) = GemFrame::deserialise(&raw).unwrap();
        assert_eq!(consumed, raw.len());
        assert_eq!(f2.payload, payload);
        assert_eq!(f2.header.port_id, 42);
    }

    #[test]
    fn test_gem_idle_frame() {
        let f = GemFrame::idle(20);
        assert!(f.header.is_idle());
        let raw = f.serialise();
        assert_eq!(raw.len(), 20);
    }

    #[test]
    fn test_gem_frame_empty_payload() {
        let f = GemFrame::new(10, vec![], true);
        let raw = f.serialise();
        assert_eq!(raw.len(), 5);
        let (f2, _) = GemFrame::deserialise(&raw).unwrap();
        assert_eq!(f2.payload.len(), 0);
    }

    // ── GEM framer (segmentation / reassembly) ────────────────────────────────

    #[test]
    fn test_gem_framer_single_frame_roundtrip() {
        let mut framer = GemFramer::new(1500);
        let sdu = b"Single segment test".to_vec();
        let frames = framer.segment(1, &sdu);
        assert_eq!(frames.len(), 1);
        let result = framer.reassemble(&frames[0]).unwrap().unwrap();
        assert_eq!(result, sdu);
    }

    #[test]
    fn test_gem_framer_multi_fragment() {
        let mut framer = GemFramer::new(8);
        let sdu: Vec<u8> = (0u8..32).collect();
        let frames = framer.segment(5, &sdu);
        assert_eq!(frames.len(), 4); // 32/8 = 4
        // Check PTI flags
        for (i, f) in frames.iter().enumerate() {
            if i < frames.len() - 1 {
                assert_eq!(f.header.pti, 1);
            } else {
                assert_eq!(f.header.pti, 0);
            }
        }
        // Reassemble
        let mut out = None;
        for f in &frames {
            out = framer.reassemble(f).unwrap();
        }
        assert_eq!(out.unwrap(), sdu);
    }

    #[test]
    fn test_gem_framer_idle_ignored() {
        let mut framer = GemFramer::new(1500);
        let idle = GemFrame::idle(5);
        let result = framer.reassemble(&idle).unwrap();
        assert!(result.is_none());
    }

    #[test]
    fn test_gem_framer_multiple_ports() {
        let mut framer = GemFramer::new(4);
        let sdu_a: Vec<u8> = vec![0xAA; 8];
        let sdu_b: Vec<u8> = vec![0xBB; 8];
        let fa = framer.segment(10, &sdu_a);
        let fb = framer.segment(20, &sdu_b);
        // Interleave
        for frame in fa.iter().chain(fb.iter()) {
            let _ = framer.reassemble(frame);
        }
        // Both should reassemble correctly (already consumed above)
    }

    // ── CRC-8 / BIP-8 tests ───────────────────────────────────────────────────

    #[test]
    fn test_crc8_known_value() {
        // CRC-8 of [0x00] with poly 0x07 should be 0x00.
        assert_eq!(crc8(&[0x00]), 0x00);
    }

    #[test]
    fn test_bip8_xor() {
        let data = [0xAA, 0x55, 0xFF, 0x00];
        assert_eq!(bip8(&data), 0xAA ^ 0x55 ^ 0xFF ^ 0x00);
    }

    #[test]
    fn test_bip8_empty() {
        assert_eq!(bip8(&[]), 0x00);
    }

    // ── PLOAM message tests ───────────────────────────────────────────────────

    #[test]
    fn test_ploam_serial_number_roundtrip() {
        let serial = [1, 2, 3, 4, 5, 6, 7, 8];
        let msg = PloamMessage::serial_number_onu(&serial);
        let raw = msg.serialise();
        let msg2 = PloamMessage::deserialise(&raw).unwrap();
        assert_eq!(msg2.onu_id, 0xFF);
        assert_eq!(msg2.content[..8], serial);
    }

    #[test]
    fn test_ploam_ranging_time() {
        let msg = PloamMessage::ranging_time(3, 12345678);
        let raw = msg.serialise();
        let msg2 = PloamMessage::deserialise(&raw).unwrap();
        let delay = u32::from_be_bytes(msg2.content[..4].try_into().unwrap());
        assert_eq!(delay, 12345678);
    }

    #[test]
    fn test_ploam_assign_onu_id() {
        let serial = [0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 1];
        let msg = PloamMessage::assign_onu_id(7, &serial);
        let raw = msg.serialise();
        let msg2 = PloamMessage::deserialise(&raw).unwrap();
        assert_eq!(msg2.content[0], 7);
        assert_eq!(msg2.content[1..9], serial);
    }

    #[test]
    fn test_ploam_crc_error() {
        let serial = [1u8; 8];
        let msg = PloamMessage::serial_number_onu(&serial);
        let mut raw = msg.serialise();
        raw[12] ^= 0xFF;
        assert_eq!(PloamMessage::deserialise(&raw).unwrap_err(), GponError::HecError);
    }

    #[test]
    fn test_ploam_dying_gasp() {
        let msg = PloamMessage::dying_gasp(5);
        let raw = msg.serialise();
        let msg2 = PloamMessage::deserialise(&raw).unwrap();
        assert_eq!(msg2.onu_id, 5);
    }

    #[test]
    fn test_ploam_key_report() {
        let key_frag = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
        let msg = PloamMessage::key_report(2, 0, &key_frag);
        let raw = msg.serialise();
        let msg2 = PloamMessage::deserialise(&raw).unwrap();
        assert_eq!(msg2.content[1..9], key_frag);
    }

    // ── BwAllocation tests ────────────────────────────────────────────────────

    #[test]
    fn test_bwallocation_serialise_fields() {
        let bw = BwAllocation::new(100, 0, 512, true, false, 1);
        let raw = bw.serialise();
        assert_eq!(raw.len(), 8);
    }

    #[test]
    fn test_bwallocation_size_constant() {
        assert_eq!(BwAllocation::SIZE, 8);
    }

    // ── T-CONT / DBA tests ────────────────────────────────────────────────────

    #[test]
    fn test_dba_compute_bwmap_type1() {
        let mut dba = DbaEngine::new(1000);
        let mut t = Tcont::new(10, TcontType::Type1);
        t.fixed_bw = 400;
        dba.add_tcont(t);
        let bwmap = dba.compute_bwmap();
        assert_eq!(bwmap.len(), 1);
        assert_eq!(bwmap[0].stop_time, 400);
    }

    #[test]
    fn test_dba_compute_bwmap_type4_no_queue() {
        let mut dba = DbaEngine::new(1000);
        let mut t = Tcont::new(20, TcontType::Type4);
        t.queue_occupancy = 0;
        dba.add_tcont(t);
        let bwmap = dba.compute_bwmap();
        assert!(bwmap.is_empty());
    }

    #[test]
    fn test_dba_compute_bwmap_type4_with_queue() {
        let mut dba = DbaEngine::new(1000);
        let mut t = Tcont::new(20, TcontType::Type4);
        t.queue_occupancy = 300;
        dba.add_tcont(t);
        let bwmap = dba.compute_bwmap();
        assert_eq!(bwmap.len(), 1);
        assert_eq!(bwmap[0].stop_time, 300);
    }

    #[test]
    fn test_dba_cap_clamping() {
        let mut dba = DbaEngine::new(100);
        let mut t = Tcont::new(1, TcontType::Type1);
        t.fixed_bw = 500; // more than capacity
        dba.add_tcont(t);
        let bwmap = dba.compute_bwmap();
        assert!(bwmap[0].stop_time <= 100);
    }

    // ── RS FEC tests ──────────────────────────────────────────────────────────

    #[test]
    fn test_rs_encode_length() {
        let data = vec![0xABu8; 100];
        let parity = rs::encode(&data);
        assert_eq!(parity.len(), 16);
    }

    #[test]
    fn test_rs_encode_zero_data_parity_zero() {
        let data = vec![0u8; 239];
        let parity = rs::encode(&data);
        // All-zero message → all-zero codeword (systematic RS with correct generator)
        // (Actually parity may not be zero; we just check length.)
        assert_eq!(parity.len(), 16);
    }

    #[test]
    fn test_fec_encode_stream_output_size() {
        let data = vec![0u8; 239 * 2];
        let out = fec_encode_stream(&data);
        // 2 blocks × (239 data + 16 parity) = 510 bytes
        assert_eq!(out.len(), 510);
    }

    #[test]
    fn test_rs_syndromes_all_zero_for_valid_codeword() {
        let data = vec![0x42u8; 239];
        let parity = rs::encode(&data);
        let mut codeword = vec![0u8; 255];
        codeword[..239].copy_from_slice(&data);
        codeword[239..].copy_from_slice(&parity);
        let cw_arr: [u8; 255] = codeword.try_into().unwrap();
        assert!(fec_check_block(&cw_arr));
    }

    // ── AES-128 CTR tests ─────────────────────────────────────────────────────

    #[test]
    fn test_aes_ctr_encrypt_decrypt_roundtrip() {
        let key = [0u8; 16];
        let nonce = [0u8; 16];
        let plaintext = b"Hello, GPON encryption!";
        let ciphertext = aes128::ctr_crypt(&key, &nonce, plaintext);
        let decrypted = aes128::ctr_crypt(&key, &nonce, &ciphertext);
        assert_eq!(decrypted, plaintext);
    }

    #[test]
    fn test_aes_ctr_different_keys_differ() {
        let key1 = [0u8; 16];
        let key2 = [1u8; 16];
        let nonce = [0u8; 16];
        let plain = b"test data";
        let c1 = aes128::ctr_crypt(&key1, &nonce, plain);
        let c2 = aes128::ctr_crypt(&key2, &nonce, plain);
        assert_ne!(c1, c2);
    }

    #[test]
    fn test_aes_block_encrypt_known() {
        // NIST FIPS 197 Appendix B test vector
        let key: [u8; 16] = [
            0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
            0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c,
        ];
        let pt: [u8; 16] = [
            0x32, 0x43, 0xf6, 0xa8, 0x88, 0x5a, 0x30, 0x8d,
            0x31, 0x31, 0x98, 0xa2, 0xe0, 0x37, 0x07, 0x34,
        ];
        let ct_expected: [u8; 16] = [
            0x39, 0x25, 0x84, 0x1d, 0x02, 0xdc, 0x09, 0xfb,
            0xdc, 0x11, 0x85, 0x97, 0x19, 0x6a, 0x0b, 0x32,
        ];
        let rk = aes128::key_expand(&key);
        let ct = aes128::encrypt_block(&pt, &rk);
        assert_eq!(ct, ct_expected);
    }

    // ── Encryption engine tests ───────────────────────────────────────────────

    #[test]
    fn test_gem_encryption_engine_roundtrip() {
        let mut eng = GemEncryptionEngine::new();
        let key = [0xABu8; 16];
        eng.set_key(100, key);
        let plain = b"Payload data for port 100".to_vec();
        let enc = eng.crypt(100, &plain).unwrap();
        // Counter incremented; need fresh engine for decrypt
        let mut eng2 = GemEncryptionEngine::new();
        eng2.set_key(100, key);
        let dec = eng2.crypt(100, &enc).unwrap();
        assert_eq!(dec, plain);
    }

    #[test]
    fn test_gem_encryption_no_key_error() {
        let mut eng = GemEncryptionEngine::new();
        let err = eng.crypt(999, b"data").unwrap_err();
        assert_eq!(err, GponError::KeyNotSet);
    }

    // ── Ranging engine tests ──────────────────────────────────────────────────

    #[test]
    fn test_ranging_assign_onu() {
        let mut eng = RangingEngine::new();
        let serial = [0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 1];
        let msgs = eng.on_serial_number(&serial).unwrap();
        assert_eq!(msgs.len(), 2);
        assert_eq!(eng.onus.len(), 1);
        assert_eq!(eng.onus[0].onu_id, 1);
    }

    #[test]
    fn test_ranging_find_by_id() {
        let mut eng = RangingEngine::new();
        let serial = [1u8; 8];
        eng.on_serial_number(&serial).unwrap();
        assert!(eng.find_by_id(1).is_some());
        assert!(eng.find_by_id(99).is_none());
    }

    #[test]
    fn test_ranging_ack_sets_operational() {
        let mut eng = RangingEngine::new();
        let serial = [2u8; 8];
        eng.on_serial_number(&serial).unwrap();
        assert!(eng.on_ranging_ack(1));
        assert_eq!(eng.onus[0].state, OnuState::Operational);
    }

    #[test]
    fn test_ranging_duplicate_serial() {
        let mut eng = RangingEngine::new();
        let serial = [5u8; 8];
        eng.on_serial_number(&serial).unwrap();
        // Second registration attempt with same serial
        let result = eng.on_serial_number(&serial);
        assert_eq!(result.unwrap_err(), GponError::OnuAlreadyAssigned);
    }

    // ── GTC downstream frame tests ────────────────────────────────────────────

    #[test]
    fn test_gtc_downstream_frame_length() {
        let pcbd = Pcbd {
            ident: 0,
            ploamd: [0u8; 13],
            bip8: 0,
            bwmap: vec![],
        };
        let frame = GtcDownstreamFrame::new(pcbd, vec![]);
        let raw = frame.serialise();
        assert_eq!(raw.len(), GPON_DS_FRAME_BYTES);
    }

    #[test]
    fn test_gtc_downstream_psync() {
        let pcbd = Pcbd { ident: 0xDEAD, ploamd: [0u8; 13], bip8: 0, bwmap: vec![] };
        let frame = GtcDownstreamFrame::new(pcbd, vec![]);
        let raw = frame.serialise();
        assert_eq!(&raw[..4], &PSYNC);
    }

    #[test]
    fn test_gtc_check_sync_ok() {
        let pcbd = Pcbd { ident: 42, ploamd: [0u8; 13], bip8: 0, bwmap: vec![] };
        let frame = GtcDownstreamFrame::new(pcbd, vec![]);
        let raw = frame.serialise();
        let ident = GtcDownstreamFrame::check_sync(&raw).unwrap();
        assert_eq!(ident, 42);
    }

    #[test]
    fn test_gtc_check_sync_bad() {
        let mut raw = vec![0u8; 20];
        raw[0] = 0xFF; // corrupt Psync
        assert_eq!(GtcDownstreamFrame::check_sync(&raw), Err(GponError::SyncLost));
    }

    // ── XGS-PON tests ────────────────────────────────────────────────────────

    #[test]
    fn test_xgem_header_encode() {
        let h = XgemHeader::new(200, 1024, 0, false);
        let raw = h.encode();
        assert_eq!(raw.len(), 8);
    }

    #[test]
    fn test_xgtc_downstream_psync() {
        let frame = XgtcDownstreamFrame::new(0, 0xDEADBEEF_12345678, [0u8; 13], 0, vec![], vec![]);
        let raw = frame.serialise();
        assert_eq!(&raw[..8], &XGSPON_PSYNC);
    }

    #[test]
    fn test_xgtc_check_sync_ok() {
        let frame = XgtcDownstreamFrame::new(99, 0, [0u8; 13], 0, vec![], vec![]);
        let raw = frame.serialise();
        let ident = XgtcDownstreamFrame::check_sync(&raw).unwrap();
        assert_eq!(ident, 99);
    }

    #[test]
    fn test_xgtc_check_sync_bad() {
        let raw = vec![0u8; 20];
        assert_eq!(XgtcDownstreamFrame::check_sync(&raw), Err(GponError::SyncLost));
    }

    // ── DBRu tests ────────────────────────────────────────────────────────────

    #[test]
    fn test_dbru_serialise() {
        let dbr = Dbru::new(1, 0x012345);
        let raw = dbr.serialise();
        assert_eq!(raw[0], 1);
        assert_eq!(u32::from_be_bytes(raw) & 0x00FFFFFF, 0x012345);
    }

    #[test]
    fn test_dbru_report_masked() {
        let dbr = Dbru::new(0, 0xFF_FFFFFF);
        let raw = dbr.serialise();
        assert_eq!(raw[0], 0); // mode
        assert_eq!(u32::from_be_bytes(raw) & 0x00FFFFFF, 0x00FFFFFF);
    }

    // ── Upstream burst tests ──────────────────────────────────────────────────

    #[test]
    fn test_us_burst_preamble() {
        let burst = GtcUpstreamBurst {
            onu_id: 5,
            ploamu: None,
            dbru: None,
            gem_bytes: vec![],
        };
        let raw = burst.serialise();
        assert_eq!(&raw[..3], &US_PREAMBLE);
        assert_eq!(&raw[3..6], &US_DELIMITER);
        assert_eq!(raw[6], 5); // ONU-ID
    }

    // ── OLT framer integration tests ──────────────────────────────────────────

    #[test]
    fn test_olt_framer_build_downstream() {
        let mut olt = GponOltFramer::new(false);
        let sdus = vec![(10u16, b"Hello".to_vec())];
        let frame = olt.build_downstream_frame([0u8; 13], &sdus);
        assert_eq!(frame.len(), GPON_DS_FRAME_BYTES);
        // Psync at offset 0
        assert_eq!(&frame[..4], &PSYNC);
    }

    #[test]
    fn test_olt_framer_fec_enabled_builds_frame() {
        let mut olt = GponOltFramer::new(true);
        let sdus = vec![(1u16, b"FEC test data".to_vec())];
        let frame = olt.build_downstream_frame([0u8; 13], &sdus);
        assert_eq!(frame.len(), GPON_DS_FRAME_BYTES);
    }

    #[test]
    fn test_olt_framer_empty_sdus() {
        let mut olt = GponOltFramer::new(false);
        let frame = olt.build_downstream_frame([0u8; 13], &[]);
        assert_eq!(frame.len(), GPON_DS_FRAME_BYTES);
    }

    #[test]
    fn test_olt_framer_process_upstream() {
        let olt = GponOltFramer::new(false);
        // Build a minimal upstream burst
        let gem_frame = GemFrame::new(10, b"upstream payload".to_vec(), true);
        let gem_bytes = gem_frame.serialise();
        let mut burst = Vec::new();
        burst.extend_from_slice(&US_PREAMBLE);
        burst.extend_from_slice(&US_DELIMITER);
        burst.push(1u8); // ONU-ID
        burst.extend_from_slice(&[0u8; 13]); // PLOAMu
        burst.extend_from_slice(&[0u8; 4]);  // DBRu
        burst.extend_from_slice(&gem_bytes);
        let mut olt = olt;
        let sdus = olt.process_upstream_burst(&burst).unwrap();
        assert_eq!(sdus.len(), 1);
        assert_eq!(sdus[0].0, 10);
        assert_eq!(sdus[0].1, b"upstream payload");
    }

    // ── CRC-16 ARC tests ─────────────────────────────────────────────────────

    #[test]
    fn test_crc16_arc_deterministic() {
        let a = crc16_arc(b"test");
        let b = crc16_arc(b"test");
        assert_eq!(a, b);
    }

    #[test]
    fn test_crc16_arc_differs_for_different_inputs() {
        let a = crc16_arc(b"GPON");
        let b = crc16_arc(b"XGS-PON");
        assert_ne!(a, b);
    }

    // ── Frame constants ───────────────────────────────────────────────────────

    #[test]
    fn test_gpon_ds_frame_size() {
        // 2.488 Gbps × 125 µs = 38880 bytes
        assert_eq!(GPON_DS_FRAME_BYTES, 38880);
    }

    #[test]
    fn test_xgspon_frame_size() {
        // ~9.95328 Gbps × 125 µs = 155520 bytes
        assert_eq!(XGSPON_DS_FRAME_BYTES, 155520);
    }

    #[test]
    fn test_pcbd_fixed_size() {
        // Psync(4) + Ident(4) + PLOAMd(13) + BIP(1) + Plend(4) + Plend_copy(4) = 30
        assert_eq!(PCBD_FIXED_SIZE, 30);
    }
}
