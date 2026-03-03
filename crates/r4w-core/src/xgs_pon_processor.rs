//! XGS-PON (10-Gigabit Symmetric Passive Optical Network) Processor
//!
//! Implements ITU-T G.9807.1 XGS-PON physical and transmission convergence layers:
//!
//! - XGTC framing: 125 µs downstream frames at 9.953 Gbps (155 520 bytes/frame)
//! - XGEM/GEM encapsulation: 8-byte XGEM header, PLI, Port-ID, PTI, Key Index, HEC (CRC-13)
//! - PLOAM messaging: Serial_Number, Ranging, Activate_ONU, DBA_Report, etc.
//! - DBA: T-CONT types 1–5, bandwidth map (BWmap), upstream scheduling
//! - Upstream burst mode: preamble, delimiter, guard time, ONU activation
//! - FEC: RS(248,216) downstream / RS(248,232) upstream (GF(2^8), poly 0x11D)
//! - Power budget: ODN classes N1/N2/E1/E2 (up to 35 dB)
//! - Wavelength plan: DS 1577 nm, US 1270 nm, GPON coexistence
//! - AES-128 CTR mode per-GEM-port encryption (educational)
//! - ONU ranging: equalization delay, round-trip time, quiet window
//! - Traffic management: priority queuing, T-CONT scheduling

// trace:XGS-PON-G9807 | ai:claude

/// Speed of light in fibre (m/s), effective refractive index ~1.5
const C_FIBRE: f64 = 2.0e8;

/// Downstream line rate in bits per second (9.953 Gbps)
pub const DS_LINE_RATE_BPS: u64 = 9_953_280_000;

/// Upstream line rate in bits per second (9.953 Gbps, symmetric)
pub const US_LINE_RATE_BPS: u64 = 9_953_280_000;

/// Frame period in microseconds (125 µs)
pub const FRAME_PERIOD_US: f64 = 125.0;

/// Downstream frame size in bytes at 9.953 Gbps for 125 µs
/// 9_953_280_000 * 125e-6 / 8 = 155_520 bytes
pub const DS_FRAME_BYTES: usize = 155_520;

/// XGTC downstream header size (bytes)
pub const XGTC_DS_HEADER_BYTES: usize = 8;

/// XGEM header size (bytes)
pub const XGEM_HEADER_BYTES: usize = 8;

/// RS(248,216) parity bytes for downstream FEC (32 bytes)
pub const RS_DS_PARITY: usize = 32;

/// RS(248,232) parity bytes for upstream FEC (16 bytes)
pub const RS_US_PARITY: usize = 16;

/// RS codeword size (bytes)
pub const RS_CODEWORD_SIZE: usize = 248;

/// Downstream wavelength (nm)
pub const DS_WAVELENGTH_NM: u32 = 1577;

/// Upstream wavelength (nm)
pub const US_WAVELENGTH_NM: u32 = 1270;

/// Maximum ODN reach in km for class E2
pub const MAX_REACH_KM: f64 = 40.0;

/// AES block size (bytes)
pub const AES_BLOCK_SIZE: usize = 16;

/// AES key size for AES-128 (bytes)
pub const AES_KEY_SIZE: usize = 16;

/// Maximum number of ONU IDs per PON
pub const MAX_ONU_ID: usize = 256;

/// Number of T-CONT types
pub const TCONT_TYPES: usize = 5;

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

/// ODN power class (optical distribution network)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OdnClass {
    /// N1: nominal ODN class 1, 29 dB optical budget
    N1,
    /// N2: nominal ODN class 2, 31 dB optical budget
    N2,
    /// E1: extended ODN class 1, 33 dB optical budget
    E1,
    /// E2: extended ODN class 2, 35 dB optical budget
    E2,
}

impl OdnClass {
    /// Maximum differential logical reach (km)
    pub fn max_reach_km(self) -> f64 {
        match self {
            OdnClass::N1 => 20.0,
            OdnClass::N2 => 20.0,
            OdnClass::E1 => 40.0,
            OdnClass::E2 => 40.0,
        }
    }

    /// Optical power budget (dB)
    pub fn power_budget_db(self) -> f64 {
        match self {
            OdnClass::N1 => 29.0,
            OdnClass::N2 => 31.0,
            OdnClass::E1 => 33.0,
            OdnClass::E2 => 35.0,
        }
    }

    /// Minimum OLT transmit power (dBm)
    pub fn olt_tx_power_dbm(self) -> f64 {
        match self {
            OdnClass::N1 => 2.0,
            OdnClass::N2 => 4.0,
            OdnClass::E1 => 4.0,
            OdnClass::E2 => 7.0,
        }
    }
}

/// T-CONT (Transmission Container) type for upstream scheduling
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TContType {
    /// Type 1: fixed bandwidth (CBR)
    Type1,
    /// Type 2: assured bandwidth (guaranteed minimum)
    Type2,
    /// Type 3: assured + non-assured bandwidth
    Type3,
    /// Type 4: best-effort bandwidth
    Type4,
    /// Type 5: mixed (types 1+2+3+4 combined alloc IDs)
    Type5,
}

/// PLOAM message type identifiers (G.9807.1 Table C.2)
///
/// Note: In the standard some type codes are shared between OLT→ONU and ONU→OLT
/// directions (the direction is implied by context). Here we use separate variant
/// names but may share numeric values — variants are differentiated by naming
/// convention only and the raw byte value is stored in `PloamMessage.msg_type`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PloamMsgType {
    /// OLT → ONU: Grant_Burst_Profile (0x01)
    GrantBurstProfile,
    /// OLT → ONU: Assign_ONU_ID (0x03)
    AssignOnuId,
    /// OLT → ONU / ONU → OLT: Ranging_Time / Registration (0x04)
    RangingTime,
    /// OLT → ONU: Deactivate_ONU_ID (0x05)
    DeactivateOnuId,
    /// OLT → ONU: Disable_Serial_Number (0x06)
    DisableSerialNumber,
    /// OLT → ONU / ONU → OLT: Request_Password / Password (0x09)
    RequestPassword,
    /// OLT → ONU: Assign_Alloc_ID (0x0A)
    AssignAllocId,
    /// OLT → ONU: Key_Control (0x0D)
    KeyControl,
    /// OLT → ONU: Sleep_Allow (0x14)
    SleepAllow,
    /// ONU → OLT: Serial_Number_ONU (0x01 upstream direction)
    SerialNumberOnu,
    /// ONU → OLT: Registration (0x04 upstream direction)
    Registration,
    /// ONU → OLT: Password (0x09 upstream direction)
    Password,
    /// ONU → OLT: Acknowledge
    Acknowledge,
    /// ONU → OLT: Sleep_Request (0x14 upstream direction)
    SleepRequest,
}

impl PloamMsgType {
    /// Return the raw byte value per G.9807.1 Table C.2
    pub fn byte_value(self) -> u8 {
        match self {
            PloamMsgType::GrantBurstProfile  => 0x01,
            PloamMsgType::SerialNumberOnu    => 0x01,
            PloamMsgType::AssignOnuId        => 0x03,
            PloamMsgType::RangingTime        => 0x04,
            PloamMsgType::Registration       => 0x04,
            PloamMsgType::DeactivateOnuId    => 0x05,
            PloamMsgType::DisableSerialNumber => 0x06,
            PloamMsgType::RequestPassword    => 0x09,
            PloamMsgType::Password           => 0x09,
            PloamMsgType::Acknowledge        => 0x09,
            PloamMsgType::AssignAllocId      => 0x0A,
            PloamMsgType::KeyControl         => 0x0D,
            PloamMsgType::SleepAllow         => 0x14,
            PloamMsgType::SleepRequest       => 0x14,
        }
    }
}

/// XGS-PON processor configuration
#[derive(Debug, Clone)]
pub struct XgsPonConfig {
    /// ODN class
    pub odn_class: OdnClass,
    /// Enable FEC on downstream
    pub fec_downstream: bool,
    /// Enable FEC on upstream
    pub fec_upstream: bool,
    /// Enable AES-128 encryption
    pub encryption_enabled: bool,
    /// OLT serial number (8 bytes: 4 ASCII vendor + 4 hex digits)
    pub olt_serial: [u8; 8],
    /// Number of configured ONUs
    pub num_onus: usize,
}

impl Default for XgsPonConfig {
    fn default() -> Self {
        XgsPonConfig {
            odn_class: OdnClass::N2,
            fec_downstream: true,
            fec_upstream: true,
            encryption_enabled: false,
            olt_serial: *b"ALCL0001",
            num_onus: 0,
        }
    }
}

// ---------------------------------------------------------------------------
// XGEM frame
// ---------------------------------------------------------------------------

/// PTI (Payload Type Identifier) field (3 bits)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pti {
    /// User data fragment (not the last fragment)
    UserDataNotLast = 0b000,
    /// User data end (last or only fragment)
    UserDataLast    = 0b001,
    /// OAM content
    Oam             = 0b010,
    /// DBA report
    DbaReport       = 0b100,
    /// Reserved / idle frame
    Idle            = 0b111,
}

impl Pti {
    fn from_bits(b: u8) -> Self {
        match b & 0x07 {
            0b000 => Pti::UserDataNotLast,
            0b001 => Pti::UserDataLast,
            0b010 => Pti::Oam,
            0b100 => Pti::DbaReport,
            _     => Pti::Idle,
        }
    }
    fn bits(self) -> u8 {
        self as u8
    }
}

/// Parsed XGEM frame
#[derive(Debug, Clone)]
pub struct XgemFrame {
    /// Payload Length Indicator (12 bits, 0–4095 bytes)
    pub pli: u16,
    /// GEM Port-ID (16 bits)
    pub port_id: u16,
    /// Payload Type Indicator
    pub pti: Pti,
    /// Key Index (2 bits)
    pub key_index: u8,
    /// HEC (CRC-13 over first 5 header bytes)
    pub hec: u16,
    /// Payload bytes
    pub payload: Vec<u8>,
}

// ---------------------------------------------------------------------------
// XGTC frame structures
// ---------------------------------------------------------------------------

/// Downstream BWmap allocation entry (4 bytes per entry, simplified)
#[derive(Debug, Clone, Copy)]
pub struct BwmapEntry {
    /// Alloc-ID (12 bits)
    pub alloc_id: u16,
    /// Flags: FEC indicator, access start
    pub flags: u8,
    /// Start time (bytes offset in upstream frame)
    pub start_time: u16,
    /// Stop time (inclusive)
    pub stop_time: u16,
    /// Force enqueue (DBA hint)
    pub force_enqueue: bool,
}

impl BwmapEntry {
    /// Encode to 8 bytes (simplified G.9807.1 BWmap entry)
    pub fn encode(&self) -> [u8; 8] {
        let mut b = [0u8; 8];
        b[0] = ((self.alloc_id >> 4) & 0xFF) as u8;
        b[1] = (((self.alloc_id & 0xF) << 4) | ((self.flags & 0x0F) as u16)) as u8;
        b[2] = (self.start_time >> 8) as u8;
        b[3] = (self.start_time & 0xFF) as u8;
        b[4] = (self.stop_time >> 8) as u8;
        b[5] = (self.stop_time & 0xFF) as u8;
        b[6] = if self.force_enqueue { 0x80 } else { 0x00 };
        // Byte 7: CRC-8 over first 7 bytes (simplified BIP)
        let mut bip = 0u8;
        for i in 0..7 { bip ^= b[i]; }
        b[7] = bip;
        b
    }

    /// Decode from 8-byte slice
    pub fn decode(b: &[u8]) -> Result<Self, &'static str> {
        if b.len() < 8 { return Err("BWmap entry too short"); }
        let alloc_id = ((b[0] as u16) << 4) | ((b[1] as u16) >> 4);
        let flags = b[1] & 0x0F;
        let start_time = ((b[2] as u16) << 8) | (b[3] as u16);
        let stop_time  = ((b[4] as u16) << 8) | (b[5] as u16);
        let force_enqueue = (b[6] & 0x80) != 0;
        let mut bip = 0u8;
        for i in 0..7 { bip ^= b[i]; }
        if bip != b[7] { return Err("BWmap entry BIP error"); }
        Ok(BwmapEntry { alloc_id, flags, start_time, stop_time, force_enqueue })
    }
}

/// PLOAM message (48 bytes fixed in G.9807.1)
#[derive(Debug, Clone)]
pub struct PloamMessage {
    /// ONU-ID (0–253: valid ONU, 255: broadcast)
    pub onu_id: u8,
    /// Message type
    pub msg_type: u8,
    /// Sequence counter (8 bits)
    pub sequence_no: u8,
    /// Message content (40 bytes)
    pub content: [u8; 40],
    /// MIC (4 bytes, simplified CRC-32)
    pub mic: [u8; 4],
}

impl PloamMessage {
    /// Build a new PLOAM message
    pub fn new(onu_id: u8, msg_type: u8, sequence_no: u8, content: [u8; 40]) -> Self {
        let mic = compute_crc32_bytes(&content);
        PloamMessage { onu_id, msg_type, sequence_no, content, mic }
    }

    /// Encode to 48 bytes
    pub fn encode(&self) -> [u8; 48] {
        let mut buf = [0u8; 48];
        buf[0] = self.onu_id;
        buf[1] = self.msg_type;
        buf[2] = self.sequence_no;
        buf[3..43].copy_from_slice(&self.content);
        buf[43] = 0; // padding
        buf[44..48].copy_from_slice(&self.mic);
        buf
    }

    /// Decode from 48-byte slice
    pub fn decode(b: &[u8]) -> Result<Self, &'static str> {
        if b.len() < 48 { return Err("PLOAM message too short"); }
        let onu_id      = b[0];
        let msg_type    = b[1];
        let sequence_no = b[2];
        let mut content = [0u8; 40];
        content.copy_from_slice(&b[3..43]);
        let mut mic = [0u8; 4];
        mic.copy_from_slice(&b[44..48]);
        let expected = compute_crc32_bytes(&content);
        if mic != expected { return Err("PLOAM MIC mismatch"); }
        Ok(PloamMessage { onu_id, msg_type, sequence_no, content, mic })
    }
}

/// XGTC downstream frame
#[derive(Debug, Clone)]
pub struct XgtcFrame {
    /// Frame counter (superframe / frame sequence number)
    pub frame_counter: u32,
    /// PLOAMd messages in this frame (0 or more)
    pub ploam_messages: Vec<PloamMessage>,
    /// Bandwidth map entries
    pub bwmap: Vec<BwmapEntry>,
    /// XGEM payloads (already encoded bytes)
    pub xgem_data: Vec<u8>,
    /// FEC applied flag
    pub fec_applied: bool,
}

// ---------------------------------------------------------------------------
// T-CONT and DBA
// ---------------------------------------------------------------------------

/// A single T-CONT scheduling record
#[derive(Debug, Clone)]
pub struct TContRecord {
    /// Alloc-ID
    pub alloc_id: u16,
    /// ONU-ID this T-CONT belongs to
    pub onu_id: u8,
    /// T-CONT type
    pub tcont_type: TContType,
    /// Fixed bandwidth allocation (bytes per frame) for type 1
    pub fixed_bw_bytes: u32,
    /// Assured bandwidth (bytes per frame) for types 2/3
    pub assured_bw_bytes: u32,
    /// Maximum bandwidth (bytes per frame) for types 3/4/5
    pub max_bw_bytes: u32,
    /// Reported queue occupancy (bytes), updated by DBA
    pub queue_occupancy: u32,
}

impl TContRecord {
    /// Compute bytes to grant for this T-CONT this frame
    pub fn compute_grant(&self) -> u32 {
        match self.tcont_type {
            TContType::Type1 => self.fixed_bw_bytes,
            TContType::Type2 => self.assured_bw_bytes,
            TContType::Type3 => {
                let base = self.assured_bw_bytes;
                let extra = self.queue_occupancy.saturating_sub(base);
                (base + extra).min(self.max_bw_bytes)
            }
            TContType::Type4 => self.queue_occupancy.min(self.max_bw_bytes),
            TContType::Type5 => {
                let guaranteed = self.fixed_bw_bytes + self.assured_bw_bytes;
                let dynamic = self.queue_occupancy.saturating_sub(guaranteed);
                (guaranteed + dynamic).min(self.max_bw_bytes)
            }
        }
    }
}

// ---------------------------------------------------------------------------
// ONU state
// ---------------------------------------------------------------------------

/// ONU activation state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OnuState {
    /// Initial state (not yet discovered)
    O1Initial,
    /// Standby (serial number sent)
    O2Standby,
    /// Serial-number detection
    O3SerialNumDetect,
    /// Ranging (equalization delay being determined)
    O4Ranging,
    /// Operation (fully activated)
    O5Operation,
    /// Intermittent LODS
    O6IntermittentLods,
    /// Emergency stop
    O7EmergencyStop,
}

/// ONU context maintained by the OLT
#[derive(Debug, Clone)]
pub struct OnuContext {
    /// ONU-ID assigned (0–253)
    pub onu_id: u8,
    /// Serial number (8 bytes)
    pub serial: [u8; 8],
    /// Current activation state
    pub state: OnuState,
    /// Equalization delay (word clocks, 16-bit)
    pub equalization_delay: u32,
    /// Round-trip time (nanoseconds)
    pub rtt_ns: f64,
    /// AES encryption key per key index (2 keys)
    pub aes_keys: [[u8; AES_KEY_SIZE]; 2],
    /// T-CONT records
    pub tconts: Vec<TContRecord>,
}

impl OnuContext {
    /// Create a new ONU context
    pub fn new(onu_id: u8, serial: [u8; 8]) -> Self {
        OnuContext {
            onu_id,
            serial,
            state: OnuState::O1Initial,
            equalization_delay: 0,
            rtt_ns: 0.0,
            aes_keys: [[0u8; AES_KEY_SIZE]; 2],
            tconts: Vec::new(),
        }
    }
}

// ---------------------------------------------------------------------------
// GF(2^8) arithmetic for Reed-Solomon (poly 0x11D = x^8+x^4+x^3+x^2+1)
// ---------------------------------------------------------------------------

const RS_POLY: u32 = 0x11D; // x^8 + x^4 + x^3 + x^2 + 1

fn gf_mul(mut a: u8, mut b: u8) -> u8 {
    let mut result = 0u8;
    while b > 0 {
        if b & 1 != 0 { result ^= a; }
        let high = a & 0x80;
        a <<= 1;
        if high != 0 { a ^= 0x1D; } // 0x11D mod 0x100 = 0x1D
        b >>= 1;
    }
    result
}

fn gf_pow(mut base: u8, mut exp: u32) -> u8 {
    let mut result = 1u8;
    while exp > 0 {
        if exp & 1 != 0 { result = gf_mul(result, base); }
        base = gf_mul(base, base);
        exp >>= 1;
    }
    result
}

fn gf_inv(a: u8) -> u8 {
    if a == 0 { panic!("gf_inv(0) undefined"); }
    gf_pow(a, 254)
}

fn gf_div(a: u8, b: u8) -> u8 {
    gf_mul(a, gf_inv(b))
}

// ---------------------------------------------------------------------------
// CRC-13 for XGEM HEC
// ---------------------------------------------------------------------------

/// Compute CRC-13 for XGEM HEC over 5 bytes (bit polynomial 0x108B)
///
/// ITU-T G.9807.1 specifies the HEC protecting the 5-byte header prefix.
pub fn xgem_hec(header_bytes: &[u8; 5]) -> u16 {
    // Poly: x^13 + x^7 + x^3 + x + 1 (0x108B)
    const POLY: u16 = 0x108B;
    let mut crc: u16 = 0;
    for &byte in header_bytes.iter() {
        for shift in (0..8).rev() {
            let bit = ((byte >> shift) as u16) & 1;
            let fb  = ((crc >> 12) & 1) ^ bit;
            crc <<= 1;
            crc &= 0x1FFF;
            if fb != 0 {
                crc ^= POLY;
            }
        }
    }
    crc & 0x1FFF
}

// ---------------------------------------------------------------------------
// CRC-32 helper (ITU-T V.42)
// ---------------------------------------------------------------------------

fn crc32_update(mut crc: u32, data: &[u8]) -> u32 {
    for &b in data {
        crc ^= b as u32;
        for _ in 0..8 {
            if crc & 1 != 0 {
                crc = (crc >> 1) ^ 0xEDB88320;
            } else {
                crc >>= 1;
            }
        }
    }
    crc
}

fn compute_crc32_bytes(data: &[u8]) -> [u8; 4] {
    let crc = !crc32_update(!0u32, data);
    [
        (crc >> 24) as u8,
        (crc >> 16) as u8,
        (crc >>  8) as u8,
        (crc       ) as u8,
    ]
}

// ---------------------------------------------------------------------------
// AES-128 CTR mode (educational, pure Rust)
// ---------------------------------------------------------------------------

/// AES-128 round key schedule
struct Aes128 {
    round_keys: [[u8; 16]; 11],
}

/// AES S-box
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

const RCON: [u8; 10] = [0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80,0x1b,0x36];

fn sub_word(w: [u8; 4]) -> [u8; 4] {
    [SBOX[w[0] as usize], SBOX[w[1] as usize], SBOX[w[2] as usize], SBOX[w[3] as usize]]
}

fn rot_word(w: [u8; 4]) -> [u8; 4] { [w[1], w[2], w[3], w[0]] }

fn xor4(a: [u8; 4], b: [u8; 4]) -> [u8; 4] {
    [a[0]^b[0], a[1]^b[1], a[2]^b[2], a[3]^b[3]]
}

impl Aes128 {
    fn new(key: &[u8; AES_KEY_SIZE]) -> Self {
        let mut w = [[0u8; 4]; 44];
        for i in 0..4 {
            w[i] = [key[4*i], key[4*i+1], key[4*i+2], key[4*i+3]];
        }
        for i in 4..44 {
            let mut temp = w[i-1];
            if i % 4 == 0 {
                temp = xor4(sub_word(rot_word(temp)), [RCON[i/4-1], 0, 0, 0]);
            }
            w[i] = xor4(w[i-4], temp);
        }
        let mut round_keys = [[0u8; 16]; 11];
        for r in 0..11 {
            for c in 0..4 {
                round_keys[r][4*c..4*c+4].copy_from_slice(&w[4*r+c]);
            }
        }
        Aes128 { round_keys }
    }

    fn xtime(b: u8) -> u8 {
        let h = b & 0x80;
        let mut r = b << 1;
        if h != 0 { r ^= 0x1B; }
        r
    }

    fn mix_column(c: &mut [u8; 4]) {
        let a = *c;
        c[0] = Self::xtime(a[0]) ^ Self::xtime(a[1]) ^ a[1] ^ a[2] ^ a[3];
        c[1] = a[0] ^ Self::xtime(a[1]) ^ Self::xtime(a[2]) ^ a[2] ^ a[3];
        c[2] = a[0] ^ a[1] ^ Self::xtime(a[2]) ^ Self::xtime(a[3]) ^ a[3];
        c[3] = Self::xtime(a[0]) ^ a[0] ^ a[1] ^ a[2] ^ Self::xtime(a[3]);
    }

    fn encrypt_block(&self, block: &mut [u8; 16]) {
        // AddRoundKey (round 0)
        for i in 0..16 { block[i] ^= self.round_keys[0][i]; }
        for round in 1..=10 {
            // SubBytes
            for i in 0..16 { block[i] = SBOX[block[i] as usize]; }
            // ShiftRows
            let b = *block;
            block[0]=b[0]; block[1]=b[5]; block[2]=b[10]; block[3]=b[15];
            block[4]=b[4]; block[5]=b[9]; block[6]=b[14]; block[7]=b[3];
            block[8]=b[8]; block[9]=b[13]; block[10]=b[2]; block[11]=b[7];
            block[12]=b[12]; block[13]=b[1]; block[14]=b[6]; block[15]=b[11];
            // MixColumns (skip last round)
            if round < 10 {
                for col in 0..4 {
                    let mut c = [block[4*col], block[4*col+1], block[4*col+2], block[4*col+3]];
                    Self::mix_column(&mut c);
                    block[4*col..4*col+4].copy_from_slice(&c);
                }
            }
            // AddRoundKey
            for i in 0..16 { block[i] ^= self.round_keys[round][i]; }
        }
    }
}

/// AES-128 CTR mode encryption/decryption (symmetric)
///
/// `key`   – 16-byte AES key
/// `nonce` – 16-byte initial counter block (IV)
/// `data`  – plaintext or ciphertext (modified in place)
pub fn aes128_ctr(key: &[u8; AES_KEY_SIZE], nonce: &[u8; 16], data: &mut [u8]) {
    let aes = Aes128::new(key);
    let mut counter = *nonce;
    let mut keystream_block = [0u8; 16];
    let mut offset = 0usize;
    while offset < data.len() {
        keystream_block = counter;
        aes.encrypt_block(&mut keystream_block);
        // Increment counter (big-endian)
        let mut carry = 1u16;
        for i in (0..16).rev() {
            carry += counter[i] as u16;
            counter[i] = carry as u8;
            carry >>= 8;
        }
        let end = (offset + 16).min(data.len());
        for i in offset..end {
            data[i] ^= keystream_block[i - offset];
        }
        offset += 16;
    }
}

// ---------------------------------------------------------------------------
// Reed-Solomon RS(n,k) encoder/decoder over GF(2^8)
// Primitive poly: 0x11D (x^8 + x^4 + x^3 + x^2 + 1), primitive element α=2
// ---------------------------------------------------------------------------

/// Build RS generator polynomial for `parity` parity symbols.
///
/// Returns coefficients in ascending degree order: result[0] = constant term,
/// result[parity] = leading coefficient (always 1).
/// g(x) = prod_{i=0}^{parity-1} (x - α^i) = prod_{i=0}^{parity-1} (x + α^i)
fn rs_generator_poly(parity: usize) -> Vec<u8> {
    // Start with g(x) = 1, i.e., [1] in ascending-degree form.
    let mut g = vec![1u8]; // g[0]=1, degree 0
    for i in 0..parity {
        let root = gf_pow(2, i as u32); // α^i
        // Multiply g by (x + root): new_g[j] = g[j-1] + root*g[j]
        let deg = g.len(); // current degree + 1
        let mut ng = vec![0u8; deg + 1];
        // constant term of (x+root)*g = root*g[0]
        ng[0] = gf_mul(root, g[0]);
        for j in 1..deg {
            ng[j] = gf_mul(root, g[j]) ^ g[j - 1];
        }
        // leading term: g[deg-1] * x  → coefficient 1 at new leading position
        ng[deg] = g[deg - 1];
        g = ng;
    }
    g // g[0]=constant, g[parity]=1 (monic)
}

/// Reed-Solomon encoder (systematic)
///
/// `data`   – information bytes (k bytes)
/// `parity` – number of parity bytes (n - k)
///
/// Returns `data || parity_bytes` (n bytes total).
///
/// The codeword is [m(x) | r(x)] where r(x) = x^parity * m(x) mod g(x).
/// All syndromes S_i = codeword(α^i) = 0 for i = 0..parity-1.
pub fn rs_encode_raw(data: &[u8], parity: usize) -> Vec<u8> {
    let g = rs_generator_poly(parity);
    // g[0]=constant, g[parity]=1 (leading).
    // We compute r = x^parity * m(x) mod g(x).
    // Work with polynomials in ascending-degree order.
    // poly = x^parity * m(x): poly[parity..parity+k] = data (data[0] = lowest degree)
    // But we prefer working highest-degree first for long division.
    // Use LFSR shift-register method: process data bytes from highest to lowest degree.
    // rem[0..parity] = remainder, rem[0]=coefficient of x^(parity-1), rem[parity-1]=constant.
    let mut rem = vec![0u8; parity];
    // g in descending order: g_desc[j] = g[parity - j] (g_desc[0]=1=leading, g_desc[parity]=constant)
    let g_desc: Vec<u8> = (0..=parity).map(|j| g[parity - j]).collect();
    for &d in data.iter() {
        // d is the new term (multiplied by x^parity brings it to degree parity+k-1-i)
        let feedback = d ^ rem[0]; // XOR with current highest-degree term of remainder
        for j in 0..parity - 1 {
            rem[j] = rem[j + 1] ^ gf_mul(feedback, g_desc[j + 1]);
        }
        rem[parity - 1] = gf_mul(feedback, g_desc[parity]);
    }
    // rem[0..parity] are the parity bytes in descending degree (rem[0]=x^(parity-1))
    let mut out = Vec::with_capacity(data.len() + parity);
    out.extend_from_slice(data);
    out.extend_from_slice(&rem);
    out
}

/// Reed-Solomon decoder (errors + erasures not yet implemented; returns data portion
/// with basic syndrome check)
///
/// Returns `Ok(data_bytes)` if syndromes are zero, `Err` otherwise.
pub fn rs_decode_raw(codeword: &[u8], parity: usize) -> Result<Vec<u8>, &'static str> {
    if codeword.len() < parity {
        return Err("Codeword too short");
    }
    let n = codeword.len();
    let k = n - parity;
    // Compute syndromes S_i = codeword(α^i) for i = 0..2t-1
    let mut has_error = false;
    for i in 0..parity {
        let root = gf_pow(2, i as u32);
        let mut s = 0u8;
        for &c in codeword.iter() {
            s = gf_mul(s, root) ^ c;
        }
        if s != 0 {
            has_error = true;
            break;
        }
    }
    if has_error {
        Err("RS syndrome non-zero: uncorrected error")
    } else {
        Ok(codeword[..k].to_vec())
    }
}

// ---------------------------------------------------------------------------
// Main processor
// ---------------------------------------------------------------------------

/// XGS-PON processor (OLT-side)
pub struct XgsPonProcessor {
    /// Configuration
    pub config: XgsPonConfig,
    /// Current downstream frame counter
    pub frame_counter: u32,
    /// ONU contexts (indexed by ONU-ID)
    onus: Vec<Option<OnuContext>>,
    /// T-CONT table
    tconts: Vec<TContRecord>,
}

impl XgsPonProcessor {
    /// Create a new XGS-PON processor
    pub fn new(config: XgsPonConfig) -> Self {
        let mut onus = Vec::with_capacity(MAX_ONU_ID);
        for _ in 0..MAX_ONU_ID { onus.push(None); }
        XgsPonProcessor {
            config,
            frame_counter: 0,
            onus,
            tconts: Vec::new(),
        }
    }

    // -----------------------------------------------------------------------
    // XGEM framing
    // -----------------------------------------------------------------------

    /// Encode payload into one or more XGEM frames (with fragmentation)
    ///
    /// An empty payload produces one XGEM idle/padding frame (header only, PLI=0).
    /// Returns raw bytes of the encoded XGEM frame(s).
    pub fn build_xgem(&self, port_id: u16, payload: &[u8]) -> Vec<u8> {
        let mut out = Vec::new();
        if payload.is_empty() {
            // Produce a single frame with PLI=0 (idle / padding)
            out.extend_from_slice(&self.encode_xgem_frame(port_id, Pti::UserDataLast, 0, &[]));
            return out;
        }
        let max_payload = 4095usize;
        let mut offset = 0;
        while offset < payload.len() {
            let end = (offset + max_payload).min(payload.len());
            let chunk = &payload[offset..end];
            let is_last = end == payload.len();
            let pti = if is_last { Pti::UserDataLast } else { Pti::UserDataNotLast };
            out.extend_from_slice(&self.encode_xgem_frame(port_id, pti, 0, chunk));
            offset = end;
        }
        out
    }

    /// Encode a single XGEM frame
    fn encode_xgem_frame(&self, port_id: u16, pti: Pti, key_index: u8, payload: &[u8]) -> Vec<u8> {
        let pli = payload.len() as u16;
        // Build 5-byte prefix for HEC computation
        // Byte layout (per G.9807.1 Table 14):
        //   [PLI 12b][Port-ID 16b][PTI 3b][Key-Idx 2b][HEC 13b]
        //   = 46 bits = 5 bytes + 6 bits → we pack 6 bytes total header
        // Simplified packing:
        //   B0..B1 : PLI (12b) | Port-ID[15:12] (4b)
        //   B2..B3 : Port-ID[11:0] (12b) | PTI (3b) | Key-Idx[1] (1b)
        //   B4     : Key-Idx[0] (1b) | HEC[12:8] (5b)
        //   B5     : HEC[7:0]
        let pli12: u16  = pli & 0x0FFF;
        let pid15_12: u8 = ((port_id >> 12) & 0xF) as u8;
        let pid11_0: u16 = port_id & 0x0FFF;
        let b0: u8 = ((pli12 >> 4) & 0xFF) as u8;
        let b1: u8 = (((pli12 & 0xF) << 4) as u8) | pid15_12;
        let b2: u8 = ((pid11_0 >> 4) & 0xFF) as u8;
        let b3: u8 = ((pid11_0 & 0xF) as u8) << 4
                   | ((pti.bits() & 0x7) << 1)
                   | ((key_index >> 1) & 1);
        let b4: u8 = (key_index & 1) << 7; // Key-Idx[0] in MSB of b4, rest = HEC[12:8]
        let header5: [u8; 5] = [b0, b1, b2, b3, b4];
        let hec = xgem_hec(&header5);
        let b4_full = (b4 & 0x80) | ((hec >> 8) as u8 & 0x1F);
        let b5 = (hec & 0xFF) as u8;
        let mut frame = Vec::with_capacity(XGEM_HEADER_BYTES + payload.len());
        frame.push(b0);
        frame.push(b1);
        frame.push(b2);
        frame.push(b3);
        frame.push(b4_full);
        frame.push(b5);
        // 2 padding bytes (reserved)
        frame.push(0);
        frame.push(0);
        frame.extend_from_slice(payload);
        frame
    }

    /// Parse an XGEM frame from raw bytes
    pub fn parse_xgem(&self, data: &[u8]) -> Result<XgemFrame, &'static str> {
        if data.len() < XGEM_HEADER_BYTES {
            return Err("XGEM frame too short");
        }
        let b0 = data[0];
        let b1 = data[1];
        let b2 = data[2];
        let b3 = data[3];
        let b4 = data[4];
        let b5 = data[5];

        let pli = ((b0 as u16) << 4) | ((b1 as u16) >> 4);
        let pid15_12 = (b1 & 0x0F) as u16;
        let pid11_0  = ((b2 as u16) << 4) | ((b3 as u16) >> 4);
        let port_id  = (pid15_12 << 12) | pid11_0;
        let pti_bits = (b3 >> 1) & 0x07;
        let key_index = ((b3 & 1) << 1) | ((b4 >> 7) & 1);
        let hec_received = (((b4 & 0x1F) as u16) << 8) | (b5 as u16);

        // Verify HEC (re-compute over first 5 bytes with HEC field zeroed)
        let header5_check: [u8; 5] = [b0, b1, b2, b3, b4 & 0xE0];
        let hec_computed = xgem_hec(&header5_check);
        if hec_computed != hec_received {
            return Err("XGEM HEC mismatch");
        }
        let payload_len = pli as usize;
        if data.len() < XGEM_HEADER_BYTES + payload_len {
            return Err("XGEM payload truncated");
        }
        let payload = data[XGEM_HEADER_BYTES..XGEM_HEADER_BYTES + payload_len].to_vec();
        Ok(XgemFrame {
            pli,
            port_id,
            pti: Pti::from_bits(pti_bits),
            key_index,
            hec: hec_received,
            payload,
        })
    }

    // -----------------------------------------------------------------------
    // XGTC frame assembly
    // -----------------------------------------------------------------------

    /// Build a complete downstream XGTC frame from GEM frames
    pub fn build_downstream_frame(&mut self, gems: &[XgemFrame]) -> XgtcFrame {
        let fc = self.frame_counter;
        self.frame_counter = self.frame_counter.wrapping_add(1);

        let mut xgem_data = Vec::new();
        for g in gems {
            let raw = self.encode_xgem_frame(g.port_id, g.pti, g.key_index, &g.payload);
            xgem_data.extend_from_slice(&raw);
        }
        // Fill remainder of payload area with idle XGEM frames
        let header_overhead = XGTC_DS_HEADER_BYTES;
        let total_user_area = DS_FRAME_BYTES.saturating_sub(header_overhead);
        while xgem_data.len() < total_user_area {
            let idle_payload = [0u8; 0];
            let idle_raw = self.encode_xgem_frame(0xFFFF, Pti::Idle, 0, &idle_payload);
            if xgem_data.len() + idle_raw.len() > total_user_area { break; }
            xgem_data.extend_from_slice(&idle_raw);
        }
        // Truncate to frame size
        xgem_data.truncate(total_user_area);

        XgtcFrame {
            frame_counter: fc,
            ploam_messages: Vec::new(),
            bwmap: Vec::new(),
            xgem_data,
            fec_applied: self.config.fec_downstream,
        }
    }

    /// Parse downstream XGTC frame (simplified: extract XGEMs from payload)
    pub fn parse_downstream_frame(&self, data: &[u8]) -> Result<XgtcFrame, &'static str> {
        if data.len() < XGTC_DS_HEADER_BYTES {
            return Err("XGTC frame too short");
        }
        // Read frame counter from first 4 bytes
        let frame_counter =
            ((data[0] as u32) << 24) | ((data[1] as u32) << 16) |
            ((data[2] as u32) << 8)  |  (data[3] as u32);

        let payload = &data[XGTC_DS_HEADER_BYTES..];
        let mut gems = Vec::new();
        let mut offset = 0;
        while offset + XGEM_HEADER_BYTES <= payload.len() {
            match self.parse_xgem(&payload[offset..]) {
                Ok(g) => {
                    let advance = XGEM_HEADER_BYTES + g.pli as usize;
                    gems.push(g);
                    offset += advance;
                }
                Err(_) => break,
            }
        }
        Ok(XgtcFrame {
            frame_counter,
            ploam_messages: Vec::new(),
            bwmap: Vec::new(),
            xgem_data: payload.to_vec(),
            fec_applied: false,
        })
    }

    // -----------------------------------------------------------------------
    // FEC
    // -----------------------------------------------------------------------

    /// Encode data with RS FEC for downstream (RS(248,216), parity=32)
    ///
    /// Input is split into 216-byte chunks; each chunk gets 32 parity bytes appended.
    pub fn rs_encode(&self, data: &[u8], parity_bytes: usize) -> Vec<u8> {
        let k = RS_CODEWORD_SIZE - parity_bytes;
        let mut out = Vec::new();
        let mut offset = 0;
        while offset < data.len() {
            let end = (offset + k).min(data.len());
            let chunk = &data[offset..end];
            // Pad short last chunk
            let padded: Vec<u8> = if chunk.len() < k {
                let mut v = chunk.to_vec();
                v.resize(k, 0);
                v
            } else {
                chunk.to_vec()
            };
            let encoded = rs_encode_raw(&padded, parity_bytes);
            out.extend_from_slice(&encoded);
            offset = end;
        }
        out
    }

    /// Decode data with RS FEC, stripping parity
    pub fn rs_decode(&self, data: &[u8], parity_bytes: usize) -> Result<Vec<u8>, &'static str> {
        let n = RS_CODEWORD_SIZE;
        if n < parity_bytes { return Err("invalid parity"); }
        let mut out = Vec::new();
        let mut offset = 0;
        while offset < data.len() {
            let end = (offset + n).min(data.len());
            let codeword = &data[offset..end];
            match rs_decode_raw(codeword, parity_bytes) {
                Ok(decoded) => out.extend_from_slice(&decoded),
                Err(e) => return Err(e),
            }
            offset = end;
        }
        Ok(out)
    }

    // -----------------------------------------------------------------------
    // Power budget / link budget
    // -----------------------------------------------------------------------

    /// Calculate maximum allowable optical loss (dB) for the configured ODN class
    pub fn calculate_link_budget(&self, odn_class: OdnClass) -> f64 {
        odn_class.power_budget_db()
    }

    /// Estimate path loss at the downstream wavelength for a given distance
    ///
    /// Uses ITU-T G.652.D fibre: 0.2 dB/km at 1550 nm, ~0.19 dB/km at 1577 nm.
    /// Adds connector losses and splitter insertion loss.
    pub fn path_loss_db(&self, distance_km: f64, split_ratio: u32) -> f64 {
        let fibre_loss = 0.20 * distance_km; // dB/km
        let connector_loss = 1.0; // dB (both ends)
        let splitter_loss = 10.0 * (split_ratio as f64).log10();
        fibre_loss + connector_loss + splitter_loss
    }

    /// Check if an ONU at a given distance and split ratio is reachable
    pub fn is_reachable(&self, distance_km: f64, split_ratio: u32) -> bool {
        let loss = self.path_loss_db(distance_km, split_ratio);
        let budget = self.config.odn_class.power_budget_db();
        loss <= budget
    }

    // -----------------------------------------------------------------------
    // ONU ranging
    // -----------------------------------------------------------------------

    /// Compute equalization delay for a given ONU distance
    ///
    /// Equalization delay compensates for propagation delay so all ONUs appear
    /// at the same logical distance (maximum differential logical reach).
    ///
    /// Returns the equalization delay in units of 1/rate (byte periods).
    pub fn ranging_delay(&self, distance_km: f64) -> f64 {
        // Round-trip propagation delay
        let rtt_s = 2.0 * (distance_km * 1000.0) / C_FIBRE;
        // Equalization delay = (max_rtt - actual_rtt) in byte periods
        let max_reach = self.config.odn_class.max_reach_km();
        let max_rtt_s = 2.0 * (max_reach * 1000.0) / C_FIBRE;
        let eq_delay_s = max_rtt_s - rtt_s;
        // Convert to byte periods at upstream line rate
        eq_delay_s * (US_LINE_RATE_BPS as f64 / 8.0)
    }

    /// Round-trip time for a given distance (ns)
    pub fn round_trip_time_ns(&self, distance_km: f64) -> f64 {
        2.0 * (distance_km * 1000.0) / C_FIBRE * 1e9
    }

    // -----------------------------------------------------------------------
    // ONU management
    // -----------------------------------------------------------------------

    /// Register a new ONU (assign ID and initialize context)
    pub fn register_onu(&mut self, onu_id: u8, serial: [u8; 8]) -> Result<(), &'static str> {
        if onu_id as usize >= MAX_ONU_ID {
            return Err("ONU-ID out of range");
        }
        if self.onus[onu_id as usize].is_some() {
            return Err("ONU-ID already registered");
        }
        self.onus[onu_id as usize] = Some(OnuContext::new(onu_id, serial));
        Ok(())
    }

    /// Deregister an ONU
    pub fn deregister_onu(&mut self, onu_id: u8) {
        if (onu_id as usize) < MAX_ONU_ID {
            self.onus[onu_id as usize] = None;
        }
    }

    /// Get ONU context (immutable)
    pub fn onu(&self, onu_id: u8) -> Option<&OnuContext> {
        self.onus.get(onu_id as usize)?.as_ref()
    }

    /// Get ONU context (mutable)
    pub fn onu_mut(&mut self, onu_id: u8) -> Option<&mut OnuContext> {
        self.onus.get_mut(onu_id as usize)?.as_mut()
    }

    /// Set equalization delay for an ONU after ranging
    pub fn set_equalization_delay(
        &mut self,
        onu_id: u8,
        delay: u32,
        rtt_ns: f64,
    ) -> Result<(), &'static str> {
        let ctx = self.onu_mut(onu_id).ok_or("ONU not found")?;
        ctx.equalization_delay = delay;
        ctx.rtt_ns = rtt_ns;
        ctx.state = OnuState::O5Operation;
        Ok(())
    }

    /// Set AES key for an ONU (key_index 0 or 1)
    pub fn set_aes_key(
        &mut self,
        onu_id: u8,
        key_index: usize,
        key: &[u8; AES_KEY_SIZE],
    ) -> Result<(), &'static str> {
        if key_index > 1 { return Err("key_index must be 0 or 1"); }
        let ctx = self.onu_mut(onu_id).ok_or("ONU not found")?;
        ctx.aes_keys[key_index] = *key;
        Ok(())
    }

    // -----------------------------------------------------------------------
    // PLOAM messaging
    // -----------------------------------------------------------------------

    /// Build a Serial_Number_ONU PLOAM message (ONU → OLT direction, simulated)
    pub fn build_serial_number_ploam(serial: &[u8; 8], seq: u8) -> PloamMessage {
        let mut content = [0u8; 40];
        content[0..8].copy_from_slice(serial);
        PloamMessage::new(255, PloamMsgType::SerialNumberOnu.byte_value(), seq, content)
    }

    /// Build an Assign_ONU_ID PLOAM message (OLT → ONU)
    pub fn build_assign_onu_id_ploam(
        serial: &[u8; 8],
        new_onu_id: u8,
        seq: u8,
    ) -> PloamMessage {
        let mut content = [0u8; 40];
        content[0..8].copy_from_slice(serial);
        content[8] = new_onu_id;
        PloamMessage::new(255, PloamMsgType::AssignOnuId.byte_value(), seq, content)
    }

    /// Build a Ranging_Time PLOAM message (OLT → ONU)
    pub fn build_ranging_time_ploam(
        onu_id: u8,
        equalization_delay: u32,
        seq: u8,
    ) -> PloamMessage {
        let mut content = [0u8; 40];
        content[0] = (equalization_delay >> 24) as u8;
        content[1] = (equalization_delay >> 16) as u8;
        content[2] = (equalization_delay >>  8) as u8;
        content[3] = (equalization_delay       ) as u8;
        PloamMessage::new(onu_id, PloamMsgType::RangingTime.byte_value(), seq, content)
    }

    /// Build a Key_Control PLOAM message (OLT → ONU)
    pub fn build_key_control_ploam(
        onu_id: u8,
        key_index: u8,
        key: &[u8; AES_KEY_SIZE],
        seq: u8,
    ) -> PloamMessage {
        let mut content = [0u8; 40];
        content[0] = key_index;
        content[1..17].copy_from_slice(key);
        PloamMessage::new(onu_id, PloamMsgType::KeyControl.byte_value(), seq, content)
    }

    // -----------------------------------------------------------------------
    // DBA / T-CONT management
    // -----------------------------------------------------------------------

    /// Add a T-CONT record
    pub fn add_tcont(&mut self, tcont: TContRecord) {
        self.tconts.push(tcont);
    }

    /// Run DBA and generate BWmap for next upstream frame
    ///
    /// Returns list of (alloc_id, granted_bytes) pairs.
    pub fn run_dba(&mut self) -> Vec<(u16, u32)> {
        let mut grants = Vec::new();
        let mut time_pointer: u16 = 0;
        // Simple DBA: grant T-CONT types in priority order 1→5
        for priority_type in [TContType::Type1, TContType::Type2, TContType::Type3,
                               TContType::Type4, TContType::Type5] {
            for tcont in self.tconts.iter_mut() {
                if tcont.tcont_type == priority_type {
                    let grant = tcont.compute_grant();
                    if grant > 0 {
                        grants.push((tcont.alloc_id, grant));
                        time_pointer = time_pointer.saturating_add(grant as u16);
                    }
                }
            }
        }
        grants
    }

    /// Update T-CONT queue occupancy (called on receipt of DBRu report)
    pub fn update_queue_occupancy(&mut self, alloc_id: u16, occupancy: u32) {
        for tcont in self.tconts.iter_mut() {
            if tcont.alloc_id == alloc_id {
                tcont.queue_occupancy = occupancy;
            }
        }
    }

    // -----------------------------------------------------------------------
    // Upstream burst mode
    // -----------------------------------------------------------------------

    /// Build upstream burst header (preamble + delimiter)
    ///
    /// G.9807.1 upstream burst overhead:
    ///   - Guard time: 64 bytes (programmable)
    ///   - Preamble: 40 bytes (alternating 0xAA / 0x55 pattern)
    ///   - Delimiter: 8 bytes (unique word 0xB6AB31E0 repeated)
    pub fn build_upstream_burst_header(&self) -> Vec<u8> {
        let mut header = Vec::new();
        // Preamble: 40 bytes alternating
        for i in 0..40 {
            header.push(if i % 2 == 0 { 0xAA } else { 0x55 });
        }
        // Delimiter: 8 bytes
        let delim: [u8; 4] = [0xB6, 0xAB, 0x31, 0xE0];
        header.extend_from_slice(&delim);
        header.extend_from_slice(&delim);
        header
    }

    /// Validate upstream burst header
    pub fn validate_upstream_burst_header(&self, data: &[u8]) -> bool {
        if data.len() < 48 { return false; }
        // Check preamble pattern
        for i in 0..40 {
            let expected = if i % 2 == 0 { 0xAA } else { 0x55 };
            if data[i] != expected { return false; }
        }
        // Check delimiter
        let expected_delim = [0xB6u8, 0xAB, 0x31, 0xE0, 0xB6, 0xAB, 0x31, 0xE0];
        data[40..48] == expected_delim
    }

    // -----------------------------------------------------------------------
    // GEM reassembly (multi-fragment)
    // -----------------------------------------------------------------------

    /// Reassemble fragmented GEM payload from ordered XGEM frames
    pub fn reassemble_gem(frames: &[XgemFrame]) -> Result<Vec<u8>, &'static str> {
        if frames.is_empty() { return Err("no frames"); }
        // Check all frames have the same port_id
        let pid = frames[0].port_id;
        let mut payload = Vec::new();
        for (i, f) in frames.iter().enumerate() {
            if f.port_id != pid { return Err("port_id mismatch in reassembly"); }
            let is_last = matches!(f.pti, Pti::UserDataLast);
            payload.extend_from_slice(&f.payload);
            if is_last {
                if i != frames.len() - 1 {
                    return Err("last fragment not at end of slice");
                }
                return Ok(payload);
            }
        }
        Err("no last fragment found")
    }

    // -----------------------------------------------------------------------
    // Wavelength plan
    // -----------------------------------------------------------------------

    /// Return the downstream wavelength in nanometres (1577 nm)
    pub fn downstream_wavelength_nm(&self) -> u32 { DS_WAVELENGTH_NM }

    /// Return the upstream wavelength in nanometres (1270 nm)
    pub fn upstream_wavelength_nm(&self) -> u32 { US_WAVELENGTH_NM }

    /// Check coexistence with GPON (1490/1310 nm)
    ///
    /// XGS-PON upstream (1270 nm) and GPON upstream (1310 nm) are spectrally
    /// separated. Returns true if a wavelength mux/demux can separate them.
    pub fn gpon_coexistence_possible(&self) -> bool {
        // 1270 nm (XGS-PON US) vs 1310 nm (GPON US) – >40 nm separation, feasible
        true
    }

    // -----------------------------------------------------------------------
    // Traffic management: priority queue
    // -----------------------------------------------------------------------

    /// Simple priority queue scheduler for GEM frames
    ///
    /// Returns frames sorted by port_id as a proxy for priority (lower = higher
    /// priority). In a real system this maps GEM ports to T-CONTs and priorities.
    pub fn schedule_gems<'a>(&self, gems: &'a [XgemFrame]) -> Vec<&'a XgemFrame> {
        let mut sorted: Vec<&XgemFrame> = gems.iter().collect();
        sorted.sort_by_key(|g| g.port_id);
        sorted
    }
}

// ---------------------------------------------------------------------------
// Standalone helper functions
// ---------------------------------------------------------------------------

/// Compute the guard time in bytes at 9.953 Gbps
///
/// Guard time prevents upstream burst collisions. Default is 64 ns = ~80 bytes.
pub fn guard_time_bytes(guard_ns: f64) -> usize {
    let bytes_per_ns = US_LINE_RATE_BPS as f64 / 8.0 / 1e9;
    (guard_ns * bytes_per_ns).ceil() as usize
}

/// Compute maximum ONU count for a given split ratio
///
/// G.9807.1 supports up to 256 ONU-IDs (8-bit) per PON port.
pub fn max_onu_count(split_ratio: u32) -> u32 {
    256_u32.min(split_ratio)
}

/// Convert equalization delay (byte periods) to nanoseconds
pub fn equalization_delay_to_ns(delay_bytes: u32) -> f64 {
    (delay_bytes as f64) * 8.0 / (US_LINE_RATE_BPS as f64) * 1e9
}

/// Downstream throughput efficiency accounting for FEC and framing overhead
pub fn ds_efficiency_percent(fec_enabled: bool) -> f64 {
    if fec_enabled {
        // RS(248,216): 216/248 = 87.1%
        216.0 / 248.0 * 100.0
    } else {
        100.0
    }
}

/// Upstream throughput efficiency accounting for FEC overhead
pub fn us_efficiency_percent(fec_enabled: bool) -> f64 {
    if fec_enabled {
        // RS(248,232): 232/248 = 93.5%
        232.0 / 248.0 * 100.0
    } else {
        100.0
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_processor() -> XgsPonProcessor {
        XgsPonProcessor::new(XgsPonConfig::default())
    }

    // ------- GF(2^8) arithmetic -------

    #[test]
    fn test_gf_mul_identity() {
        assert_eq!(gf_mul(1, 1), 1);
        assert_eq!(gf_mul(5, 1), 5);
        assert_eq!(gf_mul(0, 7), 0);
    }

    #[test]
    fn test_gf_mul_commutativity() {
        assert_eq!(gf_mul(13, 17), gf_mul(17, 13));
    }

    #[test]
    fn test_gf_pow() {
        assert_eq!(gf_pow(2, 0), 1);
        assert_eq!(gf_pow(2, 1), 2);
        assert_eq!(gf_pow(2, 8), 0x1D); // α^8 mod 0x11D
    }

    #[test]
    fn test_gf_inv() {
        let a = 7u8;
        let inv_a = gf_inv(a);
        assert_eq!(gf_mul(a, inv_a), 1);
    }

    #[test]
    fn test_gf_div() {
        let a = 42u8;
        let b = 19u8;
        let q = gf_div(a, b);
        assert_eq!(gf_mul(q, b), a);
    }

    // ------- CRC-13 HEC -------

    #[test]
    fn test_xgem_hec_deterministic() {
        let h1: [u8; 5] = [0x01, 0x23, 0x45, 0x67, 0x89];
        let h2 = h1;
        assert_eq!(xgem_hec(&h1), xgem_hec(&h2));
    }

    #[test]
    fn test_xgem_hec_sensitivity() {
        let h1: [u8; 5] = [0x01, 0x23, 0x45, 0x67, 0x89];
        let mut h2 = h1;
        h2[2] ^= 0x01; // flip one bit
        assert_ne!(xgem_hec(&h1), xgem_hec(&h2));
    }

    #[test]
    fn test_xgem_hec_zero() {
        let h: [u8; 5] = [0; 5];
        // CRC of all zeros should be 0
        assert_eq!(xgem_hec(&h), 0);
    }

    #[test]
    fn test_xgem_hec_max() {
        let h: [u8; 5] = [0xFF; 5];
        let hec = xgem_hec(&h);
        // Must fit in 13 bits
        assert!(hec < 0x2000);
    }

    // ------- XGEM frame encode/decode -------

    #[test]
    fn test_xgem_encode_decode_roundtrip() {
        let p = default_processor();
        let payload = b"Hello XGS-PON!".to_vec();
        let raw = p.build_xgem(0x0A0B, &payload);
        let frame = p.parse_xgem(&raw).expect("parse failed");
        assert_eq!(frame.port_id, 0x0A0B);
        assert_eq!(frame.payload, payload);
        assert!(matches!(frame.pti, Pti::UserDataLast));
    }

    #[test]
    fn test_xgem_empty_payload() {
        let p = default_processor();
        let raw = p.build_xgem(0x0001, &[]);
        assert!(raw.len() >= XGEM_HEADER_BYTES);
    }

    #[test]
    fn test_xgem_fragmentation() {
        let p = default_processor();
        let large_payload: Vec<u8> = (0..5000u16).map(|i| i as u8).collect();
        let raw = p.build_xgem(0x0002, &large_payload);
        // First frame: parse and check PTI is UserDataNotLast
        let first = p.parse_xgem(&raw).expect("parse first");
        assert!(matches!(first.pti, Pti::UserDataNotLast));
    }

    #[test]
    fn test_xgem_hec_mismatch_detected() {
        let p = default_processor();
        let payload = b"test".to_vec();
        let mut raw = p.build_xgem(0x0001, &payload);
        // Corrupt HEC byte
        raw[5] ^= 0xFF;
        assert!(p.parse_xgem(&raw).is_err());
    }

    #[test]
    fn test_xgem_key_index() {
        let p = default_processor();
        let raw = p.encode_xgem_frame(0x0C0D, Pti::UserDataLast, 1, b"key1data");
        let frame = p.parse_xgem(&raw).expect("parse");
        assert_eq!(frame.key_index, 1);
    }

    #[test]
    fn test_xgem_pti_oam() {
        let p = default_processor();
        let raw = p.encode_xgem_frame(0x0EEE, Pti::Oam, 0, b"oam payload");
        let frame = p.parse_xgem(&raw).expect("parse");
        assert!(matches!(frame.pti, Pti::Oam));
    }

    #[test]
    fn test_xgem_pti_idle() {
        let p = default_processor();
        let raw = p.encode_xgem_frame(0xFFFF, Pti::Idle, 0, &[]);
        let frame = p.parse_xgem(&raw).expect("parse");
        assert!(matches!(frame.pti, Pti::Idle));
    }

    // ------- GEM reassembly -------

    #[test]
    fn test_gem_reassembly_single() {
        let frame = XgemFrame {
            pli: 5,
            port_id: 0x0001,
            pti: Pti::UserDataLast,
            key_index: 0,
            hec: 0,
            payload: vec![1, 2, 3, 4, 5],
        };
        let reassembled = XgsPonProcessor::reassemble_gem(&[frame]).unwrap();
        assert_eq!(reassembled, vec![1, 2, 3, 4, 5]);
    }

    #[test]
    fn test_gem_reassembly_two_fragments() {
        let f1 = XgemFrame { pli: 3, port_id: 0x0002, pti: Pti::UserDataNotLast,
                             key_index: 0, hec: 0, payload: vec![10, 20, 30] };
        let f2 = XgemFrame { pli: 2, port_id: 0x0002, pti: Pti::UserDataLast,
                             key_index: 0, hec: 0, payload: vec![40, 50] };
        let out = XgsPonProcessor::reassemble_gem(&[f1, f2]).unwrap();
        assert_eq!(out, vec![10, 20, 30, 40, 50]);
    }

    #[test]
    fn test_gem_reassembly_port_id_mismatch() {
        let f1 = XgemFrame { pli: 1, port_id: 0x0001, pti: Pti::UserDataNotLast,
                             key_index: 0, hec: 0, payload: vec![1] };
        let f2 = XgemFrame { pli: 1, port_id: 0x0002, pti: Pti::UserDataLast,
                             key_index: 0, hec: 0, payload: vec![2] };
        assert!(XgsPonProcessor::reassemble_gem(&[f1, f2]).is_err());
    }

    // ------- Reed-Solomon FEC -------

    #[test]
    fn test_rs_generator_poly_degree() {
        let g = rs_generator_poly(16);
        assert_eq!(g.len(), 17); // degree 2t, 2t+1 coefficients
    }

    #[test]
    fn test_rs_encode_length() {
        let data = vec![0xABu8; 216];
        let encoded = rs_encode_raw(&data, 32);
        assert_eq!(encoded.len(), 248);
    }

    #[test]
    fn test_rs_encode_decode_noerror() {
        let data: Vec<u8> = (0u8..216).collect();
        let encoded = rs_encode_raw(&data, 32);
        let decoded = rs_decode_raw(&encoded, 32).expect("decode failed");
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_rs_encode_decode_small() {
        let data = vec![1u8, 2, 3, 4];
        let encoded = rs_encode_raw(&data, 4);
        let decoded = rs_decode_raw(&encoded, 4).expect("decode");
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_rs_encode_with_fec_api() {
        let p = default_processor();
        let data: Vec<u8> = (0..216).map(|i| i as u8).collect();
        let encoded = p.rs_encode(&data, RS_DS_PARITY);
        let decoded = p.rs_decode(&encoded, RS_DS_PARITY).expect("decode");
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_rs_upstream_parity() {
        let data: Vec<u8> = (0..232).map(|i| i as u8).collect();
        let encoded = rs_encode_raw(&data, RS_US_PARITY);
        assert_eq!(encoded.len(), RS_CODEWORD_SIZE);
        let decoded = rs_decode_raw(&encoded, RS_US_PARITY).expect("decode");
        assert_eq!(decoded, data);
    }

    #[test]
    fn test_rs_detect_error() {
        let data: Vec<u8> = (0..216).collect();
        let mut encoded = rs_encode_raw(&data, 32);
        encoded[10] ^= 0xFF; // inject error
        assert!(rs_decode_raw(&encoded, 32).is_err());
    }

    // ------- AES-128 CTR -------

    #[test]
    fn test_aes_ctr_encrypt_decrypt() {
        let key = [0u8; AES_KEY_SIZE];
        let nonce = [0u8; 16];
        let plaintext = b"XGS-PON AES test payload 12345678".to_vec();
        let mut ct = plaintext.clone();
        aes128_ctr(&key, &nonce, &mut ct);
        assert_ne!(ct, plaintext);
        aes128_ctr(&key, &nonce, &mut ct); // decrypt
        assert_eq!(ct, plaintext);
    }

    #[test]
    fn test_aes_ctr_different_keys() {
        let key1 = [0x01u8; AES_KEY_SIZE];
        let key2 = [0x02u8; AES_KEY_SIZE];
        let nonce = [0u8; 16];
        let plaintext = b"test data".to_vec();
        let mut ct1 = plaintext.clone();
        let mut ct2 = plaintext.clone();
        aes128_ctr(&key1, &nonce, &mut ct1);
        aes128_ctr(&key2, &nonce, &mut ct2);
        assert_ne!(ct1, ct2);
    }

    #[test]
    fn test_aes_ctr_known_vector() {
        // NIST AES-128-CTR: key=0x00*16, nonce/counter=0x00*16
        // First block keystream expected starts with 0x66e94bd4ef8a2c3b
        let key = [0u8; 16];
        let nonce = [0u8; 16];
        let mut block = [0u8; 16];
        aes128_ctr(&key, &nonce, &mut block);
        // Check first byte matches NIST vector
        assert_eq!(block[0], 0x66);
        assert_eq!(block[1], 0xe9);
    }

    #[test]
    fn test_aes_ctr_long_message() {
        let key = [0xABu8; AES_KEY_SIZE];
        let nonce = [0x12u8; 16];
        let plaintext: Vec<u8> = (0..100).collect();
        let mut ct = plaintext.clone();
        aes128_ctr(&key, &nonce, &mut ct);
        aes128_ctr(&key, &nonce, &mut ct);
        assert_eq!(ct, plaintext);
    }

    // ------- ONU registration and ranging -------

    #[test]
    fn test_register_deregister_onu() {
        let mut p = default_processor();
        let serial = *b"ALCL0001";
        p.register_onu(5, serial).expect("register");
        assert!(p.onu(5).is_some());
        p.deregister_onu(5);
        assert!(p.onu(5).is_none());
    }

    #[test]
    fn test_register_onu_duplicate() {
        let mut p = default_processor();
        p.register_onu(1, *b"SER00001").expect("first ok");
        assert!(p.register_onu(1, *b"SER00002").is_err());
    }

    #[test]
    fn test_ranging_delay_zero_distance() {
        let p = default_processor();
        let delay = p.ranging_delay(0.0);
        // At distance 0, equalization delay = max delay
        assert!(delay > 0.0);
    }

    #[test]
    fn test_ranging_delay_max_distance() {
        let p = default_processor();
        let max_km = p.config.odn_class.max_reach_km();
        let delay = p.ranging_delay(max_km);
        // At max reach, equalization delay ≈ 0
        assert!(delay.abs() < 1.0);
    }

    #[test]
    fn test_round_trip_time() {
        let p = default_processor();
        let rtt = p.round_trip_time_ns(10.0);
        // 2 * 10 km / 2e8 m/s = 100 µs = 100_000 ns
        let expected = 100_000.0;
        assert!((rtt - expected).abs() < 1.0);
    }

    #[test]
    fn test_set_equalization_delay() {
        let mut p = default_processor();
        p.register_onu(2, *b"SER00002").unwrap();
        p.set_equalization_delay(2, 12345, 50_000.0).unwrap();
        let ctx = p.onu(2).unwrap();
        assert_eq!(ctx.equalization_delay, 12345);
        assert!(matches!(ctx.state, OnuState::O5Operation));
    }

    // ------- Power budget -------

    #[test]
    fn test_odn_class_budgets() {
        assert_eq!(OdnClass::N1.power_budget_db(), 29.0);
        assert_eq!(OdnClass::N2.power_budget_db(), 31.0);
        assert_eq!(OdnClass::E1.power_budget_db(), 33.0);
        assert_eq!(OdnClass::E2.power_budget_db(), 35.0);
    }

    #[test]
    fn test_link_budget_n2() {
        let p = default_processor();
        let budget = p.calculate_link_budget(OdnClass::N2);
        assert_eq!(budget, 31.0);
    }

    #[test]
    fn test_path_loss_realistic() {
        let p = default_processor();
        // 10 km, 1:64 split ratio, should be ~22 dB loss
        let loss = p.path_loss_db(10.0, 64);
        // 0.20*10 + 1.0 + 10*log10(64) ≈ 2 + 1 + 18.06 ≈ 21.06 dB
        assert!((loss - 21.06).abs() < 0.5);
    }

    #[test]
    fn test_is_reachable_n2_short() {
        let p = default_processor();
        // 5 km, 1:32 split: loss ≈ 0.2*5 + 1 + 15 ≈ 17 dB < 31 dB
        assert!(p.is_reachable(5.0, 32));
    }

    #[test]
    fn test_is_reachable_n1_too_far() {
        let mut p = default_processor();
        p.config.odn_class = OdnClass::N1;
        // 20 km, 1:128 split: loss ≈ 4 + 1 + 21 = 26 dB; budget=29 — marginal
        // With split 256: loss ≈ 4 + 1 + 24 = 29 dB — exactly at budget
        // With split 512: loss ≈ 4 + 1 + 27 = 32 > 29 → not reachable
        assert!(!p.is_reachable(20.0, 512));
    }

    // ------- PLOAM messages -------

    #[test]
    fn test_ploam_encode_decode_roundtrip() {
        let content = [0xAAu8; 40];
        let msg = PloamMessage::new(10, 0x03, 1, content);
        let encoded = msg.encode();
        let decoded = PloamMessage::decode(&encoded).expect("decode");
        assert_eq!(decoded.onu_id, 10);
        assert_eq!(decoded.msg_type, 0x03);
        assert_eq!(decoded.content, content);
    }

    #[test]
    fn test_ploam_mic_error_detected() {
        let content = [0u8; 40];
        let msg = PloamMessage::new(5, 0x03, 1, content);
        let mut encoded = msg.encode();
        encoded[44] ^= 0xFF; // corrupt MIC
        assert!(PloamMessage::decode(&encoded).is_err());
    }

    #[test]
    fn test_serial_number_ploam() {
        let serial = *b"HUAW1234";
        let msg = XgsPonProcessor::build_serial_number_ploam(&serial, 0);
        assert_eq!(msg.onu_id, 255); // broadcast
        assert_eq!(&msg.content[0..8], &serial);
    }

    #[test]
    fn test_assign_onu_id_ploam() {
        let serial = *b"CALIX001";
        let msg = XgsPonProcessor::build_assign_onu_id_ploam(&serial, 42, 1);
        assert_eq!(msg.content[8], 42);
    }

    #[test]
    fn test_ranging_time_ploam() {
        let msg = XgsPonProcessor::build_ranging_time_ploam(7, 0x00012345, 3);
        assert_eq!(msg.onu_id, 7);
        let delay = ((msg.content[0] as u32) << 24) | ((msg.content[1] as u32) << 16)
                  | ((msg.content[2] as u32) << 8)  |  (msg.content[3] as u32);
        assert_eq!(delay, 0x00012345);
    }

    // ------- DBA / T-CONT -------

    #[test]
    fn test_tcont_type1_fixed() {
        let tc = TContRecord {
            alloc_id: 0x001, onu_id: 1, tcont_type: TContType::Type1,
            fixed_bw_bytes: 500, assured_bw_bytes: 0, max_bw_bytes: 500,
            queue_occupancy: 0,
        };
        assert_eq!(tc.compute_grant(), 500);
    }

    #[test]
    fn test_tcont_type4_best_effort() {
        let tc = TContRecord {
            alloc_id: 0x002, onu_id: 1, tcont_type: TContType::Type4,
            fixed_bw_bytes: 0, assured_bw_bytes: 0, max_bw_bytes: 1000,
            queue_occupancy: 750,
        };
        assert_eq!(tc.compute_grant(), 750);
    }

    #[test]
    fn test_tcont_type4_capped_by_max() {
        let tc = TContRecord {
            alloc_id: 0x003, onu_id: 1, tcont_type: TContType::Type4,
            fixed_bw_bytes: 0, assured_bw_bytes: 0, max_bw_bytes: 500,
            queue_occupancy: 2000,
        };
        assert_eq!(tc.compute_grant(), 500);
    }

    #[test]
    fn test_tcont_type3_dynamic() {
        let tc = TContRecord {
            alloc_id: 0x004, onu_id: 1, tcont_type: TContType::Type3,
            fixed_bw_bytes: 0, assured_bw_bytes: 200, max_bw_bytes: 800,
            queue_occupancy: 600,
        };
        // base=200, extra=400, total=600 <= 800
        assert_eq!(tc.compute_grant(), 600);
    }

    #[test]
    fn test_run_dba() {
        let mut p = default_processor();
        p.add_tcont(TContRecord {
            alloc_id: 0x001, onu_id: 1, tcont_type: TContType::Type1,
            fixed_bw_bytes: 100, assured_bw_bytes: 0, max_bw_bytes: 100,
            queue_occupancy: 0,
        });
        p.add_tcont(TContRecord {
            alloc_id: 0x002, onu_id: 2, tcont_type: TContType::Type4,
            fixed_bw_bytes: 0, assured_bw_bytes: 0, max_bw_bytes: 500,
            queue_occupancy: 300,
        });
        let grants = p.run_dba();
        assert_eq!(grants.len(), 2);
        let g1 = grants.iter().find(|(id,_)| *id == 0x001).unwrap();
        let g2 = grants.iter().find(|(id,_)| *id == 0x002).unwrap();
        assert_eq!(g1.1, 100);
        assert_eq!(g2.1, 300);
    }

    // ------- Upstream burst -------

    #[test]
    fn test_upstream_burst_header_length() {
        let p = default_processor();
        let hdr = p.build_upstream_burst_header();
        assert_eq!(hdr.len(), 48);
    }

    #[test]
    fn test_upstream_burst_header_valid() {
        let p = default_processor();
        let hdr = p.build_upstream_burst_header();
        assert!(p.validate_upstream_burst_header(&hdr));
    }

    #[test]
    fn test_upstream_burst_header_corrupted() {
        let p = default_processor();
        let mut hdr = p.build_upstream_burst_header();
        hdr[0] = 0xFF; // corrupt preamble
        assert!(!p.validate_upstream_burst_header(&hdr));
    }

    // ------- XGTC frame -------

    #[test]
    fn test_build_downstream_frame_size() {
        let mut p = default_processor();
        let gems = vec![];
        let frame = p.build_downstream_frame(&gems);
        // payload area must be filled to DS_FRAME_BYTES - header
        let expected = DS_FRAME_BYTES - XGTC_DS_HEADER_BYTES;
        assert!(frame.xgem_data.len() <= expected);
    }

    #[test]
    fn test_build_downstream_frame_counter() {
        let mut p = default_processor();
        p.frame_counter = 100;
        let f1 = p.build_downstream_frame(&[]);
        let f2 = p.build_downstream_frame(&[]);
        assert_eq!(f1.frame_counter, 100);
        assert_eq!(f2.frame_counter, 101);
    }

    // ------- BWmap -------

    #[test]
    fn test_bwmap_encode_decode() {
        let entry = BwmapEntry {
            alloc_id: 0x123,
            flags: 0x5,
            start_time: 0x0010,
            stop_time: 0x00FF,
            force_enqueue: true,
        };
        let encoded = entry.encode();
        let decoded = BwmapEntry::decode(&encoded).expect("decode");
        assert_eq!(decoded.alloc_id, entry.alloc_id);
        assert_eq!(decoded.start_time, entry.start_time);
        assert_eq!(decoded.stop_time, entry.stop_time);
        assert_eq!(decoded.force_enqueue, entry.force_enqueue);
    }

    #[test]
    fn test_bwmap_bip_error_detected() {
        let entry = BwmapEntry {
            alloc_id: 0x001, flags: 0, start_time: 0, stop_time: 100, force_enqueue: false,
        };
        let mut encoded = entry.encode();
        encoded[0] ^= 0x01; // corrupt byte, BIP will fail
        assert!(BwmapEntry::decode(&encoded).is_err());
    }

    // ------- Helper functions -------

    #[test]
    fn test_guard_time_bytes() {
        // 64 ns guard time at 9.953 Gbps
        let bytes = guard_time_bytes(64.0);
        // 9.953e9 / 8 / 1e9 * 64 ≈ 79.6 → 80
        assert_eq!(bytes, 80);
    }

    #[test]
    fn test_max_onu_count() {
        assert_eq!(max_onu_count(64), 64);
        assert_eq!(max_onu_count(512), 256);
    }

    #[test]
    fn test_equalization_delay_to_ns() {
        let ns = equalization_delay_to_ns(0);
        assert_eq!(ns, 0.0);
        let ns = equalization_delay_to_ns(1);
        // 1 byte period at 9.953 Gbps ≈ 0.807 ns
        assert!((ns - 0.807).abs() < 0.01);
    }

    #[test]
    fn test_ds_efficiency_with_fec() {
        let eff = ds_efficiency_percent(true);
        // 216/248 ≈ 87.1%
        assert!((eff - 87.1).abs() < 0.1);
    }

    #[test]
    fn test_us_efficiency_with_fec() {
        let eff = us_efficiency_percent(true);
        // 232/248 ≈ 93.55%
        assert!((eff - 93.55).abs() < 0.1);
    }

    #[test]
    fn test_ds_efficiency_no_fec() {
        assert_eq!(ds_efficiency_percent(false), 100.0);
    }

    // ------- Wavelength plan -------

    #[test]
    fn test_wavelengths() {
        let p = default_processor();
        assert_eq!(p.downstream_wavelength_nm(), 1577);
        assert_eq!(p.upstream_wavelength_nm(), 1270);
    }

    #[test]
    fn test_gpon_coexistence() {
        let p = default_processor();
        assert!(p.gpon_coexistence_possible());
    }

    // ------- Traffic scheduling -------

    #[test]
    fn test_schedule_gems_sorted_by_port_id() {
        let p = default_processor();
        let g1 = XgemFrame { pli: 1, port_id: 0x0010, pti: Pti::UserDataLast,
                             key_index: 0, hec: 0, payload: vec![1] };
        let g2 = XgemFrame { pli: 1, port_id: 0x0001, pti: Pti::UserDataLast,
                             key_index: 0, hec: 0, payload: vec![2] };
        let g3 = XgemFrame { pli: 1, port_id: 0x0005, pti: Pti::UserDataLast,
                             key_index: 0, hec: 0, payload: vec![3] };
        let gems = [g1, g2, g3];
        let scheduled = p.schedule_gems(&gems);
        assert_eq!(scheduled[0].port_id, 0x0001);
        assert_eq!(scheduled[1].port_id, 0x0005);
        assert_eq!(scheduled[2].port_id, 0x0010);
    }

    // ------- ODN class reach -------

    #[test]
    fn test_odn_max_reach() {
        assert_eq!(OdnClass::N1.max_reach_km(), 20.0);
        assert_eq!(OdnClass::E2.max_reach_km(), 40.0);
    }

    #[test]
    fn test_odn_tx_power() {
        assert!(OdnClass::E2.olt_tx_power_dbm() > OdnClass::N1.olt_tx_power_dbm());
    }

    // ------- AES key management -------

    #[test]
    fn test_set_aes_key_and_encrypt() {
        let mut p = default_processor();
        p.register_onu(0, *b"SER00000").unwrap();
        let key = [0x42u8; AES_KEY_SIZE];
        p.set_aes_key(0, 0, &key).unwrap();
        let ctx = p.onu(0).unwrap();
        assert_eq!(ctx.aes_keys[0], key);
    }

    #[test]
    fn test_set_aes_key_invalid_index() {
        let mut p = default_processor();
        p.register_onu(3, *b"SER00003").unwrap();
        let key = [0u8; AES_KEY_SIZE];
        assert!(p.set_aes_key(3, 2, &key).is_err());
    }

    // ------- Frame counter wrapping -------

    #[test]
    fn test_frame_counter_wraps() {
        let mut p = default_processor();
        p.frame_counter = u32::MAX;
        let f = p.build_downstream_frame(&[]);
        assert_eq!(f.frame_counter, u32::MAX);
        let f2 = p.build_downstream_frame(&[]);
        assert_eq!(f2.frame_counter, 0);
    }

    // ------- ONU state transitions -------

    #[test]
    fn test_onu_initial_state() {
        let mut p = default_processor();
        p.register_onu(10, *b"SER00010").unwrap();
        let ctx = p.onu(10).unwrap();
        assert!(matches!(ctx.state, OnuState::O1Initial));
    }

    #[test]
    fn test_onu_transitions_to_operation() {
        let mut p = default_processor();
        p.register_onu(11, *b"SER00011").unwrap();
        p.set_equalization_delay(11, 5000, 25000.0).unwrap();
        let ctx = p.onu(11).unwrap();
        assert!(matches!(ctx.state, OnuState::O5Operation));
    }
}
