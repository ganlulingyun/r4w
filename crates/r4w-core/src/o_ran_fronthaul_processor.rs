//! O-RAN Fronthaul Processor — WG4 7.2x Functional Split
//!
//! Implements the O-RAN Alliance WG4 7.2x lower-layer split (LLS) interface
//! between the O-DU (Distributed Unit) and O-RU (Radio Unit).
//!
//! # Architecture
//!
//! ```text
//!   O-DU (High-PHY + MAC + RLC)
//!       |
//!    eCPRI / Ethernet
//!       |
//!   O-RU (Low-PHY: FFT/iFFT, CP, BF)
//! ```
//!
//! # Planes
//! - **C-Plane**: Scheduling / control messages (Section Types 0-7)
//! - **U-Plane**: IQ sample transport (compressed or raw)
//! - **M-Plane**: Management (out of scope here — NETCONF/YANG)
//! - **S-Plane**: Synchronization (PTP/SyncE — timing model only)
//!
//! # References
//! - O-RAN.WG4.CUS.0-v12.00 (eCPRI/CUS specification)
//! - eCPRI Specification v1.2

use std::collections::VecDeque;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// eCPRI protocol revision field (4-bit value; 0x01 = revision 1.x per spec Table 3)
pub const ECPRI_VERSION: u8 = 0x01;
/// Maximum Ethernet jumbo frame payload
pub const MAX_ETHERNET_PAYLOAD: usize = 9000;
/// Maximum number of beams
pub const MAX_BEAMS: usize = 64;
/// Maximum PRBs (numerology 0, 100 MHz BW)
pub const MAX_PRBS: usize = 273;
/// Symbols per slot
pub const SYMBOLS_PER_SLOT: usize = 14;
/// Subframes per radio frame
pub const SUBFRAMES_PER_FRAME: usize = 10;
/// Timing accuracy Cat-A O-RU in nanoseconds (+-65 ns)
pub const TIMING_ACCURACY_CAT_A_NS: u32 = 65;
/// Timing accuracy Cat-B O-RU in nanoseconds (+-65 ns relaxed)
pub const TIMING_ACCURACY_CAT_B_NS: u32 = 65;

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// eCPRI message type identifiers (spec Table 3 & 4)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum EcpriMessageType {
    IqData = 0x00,
    BitSequence = 0x01,
    RealTimeControl = 0x02,
    GenericDataTransfer = 0x03,
    RemoteMemoryAccess = 0x04,
    OneWayDelayMeasurement = 0x05,
    RemoteReset = 0x06,
    EventIndication = 0x07,
    IqDataWithTimestamp = 0x08,
}

impl EcpriMessageType {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0x00 => Some(Self::IqData),
            0x01 => Some(Self::BitSequence),
            0x02 => Some(Self::RealTimeControl),
            0x03 => Some(Self::GenericDataTransfer),
            0x04 => Some(Self::RemoteMemoryAccess),
            0x05 => Some(Self::OneWayDelayMeasurement),
            0x06 => Some(Self::RemoteReset),
            0x07 => Some(Self::EventIndication),
            0x08 => Some(Self::IqDataWithTimestamp),
            _ => None,
        }
    }
}

/// C-Plane section types per O-RAN WG4 spec
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SectionType {
    Unused = 0,
    MostChannels = 1,
    Prach = 3,
    UlUserData = 5,
    ChannelInfo = 6,
    Laa = 7,
}

impl SectionType {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Unused),
            1 => Some(Self::MostChannels),
            3 => Some(Self::Prach),
            5 => Some(Self::UlUserData),
            6 => Some(Self::ChannelInfo),
            7 => Some(Self::Laa),
            _ => None,
        }
    }
}

/// Data direction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataDirection {
    Downlink = 0,
    Uplink = 1,
}

/// Subcarrier spacing (numerology mu)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubcarrierSpacing {
    Scs15kHz = 0,
    Scs30kHz = 1,
    Scs60kHz = 2,
    Scs120kHz = 3,
}

impl SubcarrierSpacing {
    pub fn slots_per_subframe(&self) -> usize {
        match self {
            Self::Scs15kHz => 1,
            Self::Scs30kHz => 2,
            Self::Scs60kHz => 4,
            Self::Scs120kHz => 8,
        }
    }

    pub fn hz(&self) -> u32 {
        match self {
            Self::Scs15kHz => 15_000,
            Self::Scs30kHz => 30_000,
            Self::Scs60kHz => 60_000,
            Self::Scs120kHz => 120_000,
        }
    }

    pub fn mu(&self) -> u8 {
        *self as u8
    }

    pub fn from_mu(mu: u8) -> Option<Self> {
        match mu {
            0 => Some(Self::Scs15kHz),
            1 => Some(Self::Scs30kHz),
            2 => Some(Self::Scs60kHz),
            3 => Some(Self::Scs120kHz),
            _ => None,
        }
    }
}

/// IQ compression method
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompressionMethod {
    NoCompression = 0,
    BlockFloatingPoint = 1,
    BlockScaling = 2,
    ULaw = 3,
    ModCompression = 4,
    BfpWithSelective = 5,
    ModCompWithSelective = 6,
}

impl CompressionMethod {
    pub fn from_u8(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::NoCompression),
            1 => Some(Self::BlockFloatingPoint),
            2 => Some(Self::BlockScaling),
            3 => Some(Self::ULaw),
            4 => Some(Self::ModCompression),
            5 => Some(Self::BfpWithSelective),
            6 => Some(Self::ModCompWithSelective),
            _ => None,
        }
    }
}

/// IQ sample bit width (iqWidth field; 0 => 16 bits on the wire)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IqWidth {
    Bits8 = 8,
    Bits9 = 9,
    Bits12 = 12,
    Bits14 = 14,
    Bits16 = 0,
}

impl IqWidth {
    pub fn bits(&self) -> u8 {
        match self {
            Self::Bits16 => 16,
            other => *other as u8,
        }
    }

    pub fn encoded(&self) -> u8 {
        *self as u8
    }

    pub fn from_encoded(v: u8) -> Option<Self> {
        match v {
            0 => Some(Self::Bits16),
            8 => Some(Self::Bits8),
            9 => Some(Self::Bits9),
            12 => Some(Self::Bits12),
            14 => Some(Self::Bits14),
            _ => None,
        }
    }
}

// ---------------------------------------------------------------------------
// eCPRI Header
// ---------------------------------------------------------------------------

/// eCPRI common header (4 bytes)
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EcpriHeader {
    pub version: u8,
    pub reserved: bool,
    pub concatenated: bool,
    pub message_type: EcpriMessageType,
    pub payload_size: u16,
}

impl EcpriHeader {
    pub fn new(msg_type: EcpriMessageType, payload_size: u16) -> Self {
        Self {
            version: ECPRI_VERSION,
            reserved: false,
            concatenated: false,
            message_type: msg_type,
            payload_size,
        }
    }

    pub fn serialize(&self) -> [u8; 4] {
        let byte0 = (self.version & 0x0F) << 4
            | if self.reserved { 0x02 } else { 0 }
            | if self.concatenated { 0x01 } else { 0 };
        [
            byte0,
            self.message_type as u8,
            (self.payload_size >> 8) as u8,
            self.payload_size as u8,
        ]
    }

    pub fn deserialize(buf: &[u8]) -> Option<Self> {
        if buf.len() < 4 {
            return None;
        }
        let version = (buf[0] >> 4) & 0x0F;
        let reserved = (buf[0] & 0x02) != 0;
        let concatenated = (buf[0] & 0x01) != 0;
        let message_type = EcpriMessageType::from_u8(buf[1])?;
        let payload_size = ((buf[2] as u16) << 8) | buf[3] as u16;
        Some(Self {
            version,
            reserved,
            concatenated,
            message_type,
            payload_size,
        })
    }

    pub fn total_len(&self) -> usize {
        4 + self.payload_size as usize
    }
}

// ---------------------------------------------------------------------------
// C-Plane structures
// ---------------------------------------------------------------------------

/// C-Plane Application Header (4 bytes)
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct CplaneAppHeader {
    pub data_direction: DataDirection,
    pub payload_version: u8,
    pub filter_index: u8,
    pub frame_id: u8,
    pub subframe_id: u8,
    pub slot_id: u8,
    pub symbol_id: u8,
}

impl CplaneAppHeader {
    pub fn new(dir: DataDirection, frame_id: u8, subframe_id: u8, slot_id: u8, symbol_id: u8) -> Self {
        Self {
            data_direction: dir,
            payload_version: 0x01,
            filter_index: 0,
            frame_id,
            subframe_id,
            slot_id,
            symbol_id,
        }
    }

    pub fn serialize(&self) -> [u8; 4] {
        let byte0 = ((self.data_direction as u8) << 7)
            | ((self.payload_version & 0x07) << 4)
            | (self.filter_index & 0x0F);
        let byte2 = ((self.subframe_id & 0x0F) << 4) | (self.slot_id & 0x0F);
        let byte3 = self.symbol_id & 0x3F;
        [byte0, self.frame_id, byte2, byte3]
    }

    pub fn deserialize(buf: &[u8]) -> Option<Self> {
        if buf.len() < 4 {
            return None;
        }
        let data_direction = if (buf[0] >> 7) & 1 == 1 {
            DataDirection::Uplink
        } else {
            DataDirection::Downlink
        };
        let payload_version = (buf[0] >> 4) & 0x07;
        let filter_index = buf[0] & 0x0F;
        let frame_id = buf[1];
        let subframe_id = (buf[2] >> 4) & 0x0F;
        let slot_id = buf[2] & 0x0F;
        let symbol_id = buf[3] & 0x3F;
        Some(Self {
            data_direction,
            payload_version,
            filter_index,
            frame_id,
            subframe_id,
            slot_id,
            symbol_id,
        })
    }
}

/// Section Type 1 body — DL/UL scheduling (8 bytes)
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SectionType1 {
    pub section_id: u16,
    pub rb: u8,
    pub sym_inc: u8,
    pub start_prb: u16,
    pub num_prb: u16,
    pub num_sym: u8,
    pub ef: bool,
    pub beam_id: u16,
    pub re_mask: u16,
}

impl SectionType1 {
    pub fn new(section_id: u16, start_prb: u16, num_prb: u16, beam_id: u16) -> Self {
        Self {
            section_id,
            rb: 0,
            sym_inc: 0,
            start_prb,
            num_prb,
            num_sym: 14,
            ef: false,
            beam_id,
            re_mask: 0xFFF,
        }
    }

    pub fn serialize(&self) -> [u8; 8] {
        let b0 = ((self.section_id >> 4) & 0xFF) as u8;
        let b1 = (((self.section_id & 0x0F) as u8) << 4)
            | ((self.rb & 0x01) << 3)
            | ((self.sym_inc & 0x01) << 2)
            | (((self.start_prb >> 8) & 0x03) as u8);
        let b2 = (self.start_prb & 0xFF) as u8;
        let b3 = (self.num_prb & 0xFF) as u8;
        let b4 = ((self.re_mask >> 4) & 0xFF) as u8;
        let b5 = (((self.re_mask & 0x0F) as u8) << 4) | (self.num_sym & 0x0F);
        let b6 = (if self.ef { 0x80 } else { 0 }) | (((self.beam_id >> 8) & 0x7F) as u8);
        let b7 = (self.beam_id & 0xFF) as u8;
        [b0, b1, b2, b3, b4, b5, b6, b7]
    }

    pub fn deserialize(buf: &[u8]) -> Option<Self> {
        if buf.len() < 8 {
            return None;
        }
        let section_id = ((buf[0] as u16) << 4) | ((buf[1] >> 4) as u16);
        let rb = (buf[1] >> 3) & 0x01;
        let sym_inc = (buf[1] >> 2) & 0x01;
        let start_prb = (((buf[1] & 0x03) as u16) << 8) | buf[2] as u16;
        let num_prb = buf[3] as u16;
        let re_mask = ((buf[4] as u16) << 4) | ((buf[5] >> 4) as u16);
        let num_sym = buf[5] & 0x0F;
        let ef = (buf[6] & 0x80) != 0;
        let beam_id = (((buf[6] & 0x7F) as u16) << 8) | buf[7] as u16;
        Some(Self {
            section_id,
            rb,
            sym_inc,
            start_prb,
            num_prb,
            num_sym,
            ef,
            beam_id,
            re_mask,
        })
    }
}

/// Section Type 3 — PRACH scheduling (12 bytes)
#[allow(non_snake_case)]
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SectionType3 {
    pub section_id: u16,
    pub start_prb: u16,
    pub num_prb: u16,
    pub num_sym: u8,
    pub beam_id: u16,
    pub re_mask: u16,
    pub time_offset: u16,
    pub frame_structure: u8,
    pub cpLength: u16,
    pub ef: bool,
}

impl SectionType3 {
    pub fn new(section_id: u16, start_prb: u16, num_prb: u16, beam_id: u16, time_offset: u16) -> Self {
        Self {
            section_id,
            start_prb,
            num_prb,
            num_sym: 1,
            beam_id,
            re_mask: 0xFFF,
            time_offset,
            frame_structure: 0xFF,
            cpLength: 0,
            ef: false,
        }
    }

    pub fn serialize(&self) -> [u8; 12] {
        let b0 = ((self.section_id >> 4) & 0xFF) as u8;
        let b1 = (((self.section_id & 0x0F) as u8) << 4)
            | (((self.start_prb >> 8) & 0x03) as u8);
        let b2 = (self.start_prb & 0xFF) as u8;
        let b3 = (self.num_prb & 0xFF) as u8;
        let b4 = ((self.re_mask >> 4) & 0xFF) as u8;
        let b5 = (((self.re_mask & 0x0F) as u8) << 4) | (self.num_sym & 0x0F);
        let b6 = (if self.ef { 0x80 } else { 0 }) | (((self.beam_id >> 8) & 0x7F) as u8);
        let b7 = (self.beam_id & 0xFF) as u8;
        let b8 = (self.time_offset >> 8) as u8;
        let b9 = (self.time_offset & 0xFF) as u8;
        let b10 = self.frame_structure;
        let b11 = (self.cpLength & 0xFF) as u8;
        [b0, b1, b2, b3, b4, b5, b6, b7, b8, b9, b10, b11]
    }
}

// ---------------------------------------------------------------------------
// U-Plane structures
// ---------------------------------------------------------------------------

/// U-Plane Application Header
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UplaneAppHeader {
    pub data_direction: DataDirection,
    pub payload_version: u8,
    pub filter_index: u8,
    pub frame_id: u8,
    pub subframe_id: u8,
    pub slot_id: u8,
    pub symbol_id: u8,
}

impl UplaneAppHeader {
    pub fn new(dir: DataDirection, frame_id: u8, subframe_id: u8, slot_id: u8, symbol_id: u8) -> Self {
        Self {
            data_direction: dir,
            payload_version: 0x01,
            filter_index: 0,
            frame_id,
            subframe_id,
            slot_id,
            symbol_id,
        }
    }

    pub fn serialize(&self) -> [u8; 4] {
        CplaneAppHeader {
            data_direction: self.data_direction,
            payload_version: self.payload_version,
            filter_index: self.filter_index,
            frame_id: self.frame_id,
            subframe_id: self.subframe_id,
            slot_id: self.slot_id,
            symbol_id: self.symbol_id,
        }
        .serialize()
    }
}

/// U-Plane section header (6 bytes)
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UplaneSectionHeader {
    pub section_id: u16,
    pub rb: u8,
    pub sym_inc: u8,
    pub start_prb: u16,
    pub num_prb: u16,
    pub iq_width: IqWidth,
    pub comp_meth: CompressionMethod,
}

impl UplaneSectionHeader {
    pub fn new(section_id: u16, start_prb: u16, num_prb: u16) -> Self {
        Self {
            section_id,
            rb: 0,
            sym_inc: 0,
            start_prb,
            num_prb,
            iq_width: IqWidth::Bits16,
            comp_meth: CompressionMethod::NoCompression,
        }
    }

    pub fn with_compression(mut self, iq_width: IqWidth, comp_meth: CompressionMethod) -> Self {
        self.iq_width = iq_width;
        self.comp_meth = comp_meth;
        self
    }

    pub fn serialize(&self) -> [u8; 6] {
        let b0 = ((self.section_id >> 4) & 0xFF) as u8;
        let b1 = (((self.section_id & 0x0F) as u8) << 4)
            | ((self.rb & 0x01) << 3)
            | ((self.sym_inc & 0x01) << 2)
            | (((self.start_prb >> 8) & 0x03) as u8);
        let b2 = (self.start_prb & 0xFF) as u8;
        let b3 = (self.num_prb & 0xFF) as u8;
        let iqw = self.iq_width.encoded() & 0x0F;
        let comp = (self.comp_meth as u8) & 0x0F;
        let b4 = (iqw << 4) | comp;
        [b0, b1, b2, b3, b4, 0u8]
    }

    pub fn deserialize(buf: &[u8]) -> Option<Self> {
        if buf.len() < 6 {
            return None;
        }
        let section_id = ((buf[0] as u16) << 4) | ((buf[1] >> 4) as u16);
        let rb = (buf[1] >> 3) & 0x01;
        let sym_inc = (buf[1] >> 2) & 0x01;
        let start_prb = (((buf[1] & 0x03) as u16) << 8) | buf[2] as u16;
        let num_prb = buf[3] as u16;
        let iq_width = IqWidth::from_encoded((buf[4] >> 4) & 0x0F)?;
        let comp_meth = CompressionMethod::from_u8(buf[4] & 0x0F)?;
        Some(Self {
            section_id,
            rb,
            sym_inc,
            start_prb,
            num_prb,
            iq_width,
            comp_meth,
        })
    }
}

// ---------------------------------------------------------------------------
// IQ Sample types
// ---------------------------------------------------------------------------

/// 16-bit IQ sample
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct IqSample {
    pub i: i16,
    pub q: i16,
}

impl IqSample {
    pub fn new(i: i16, q: i16) -> Self {
        Self { i, q }
    }

    pub fn zero() -> Self {
        Self { i: 0, q: 0 }
    }

    pub fn power_sq(&self) -> u32 {
        (self.i as i32).pow(2) as u32 + (self.q as i32).pow(2) as u32
    }
}

// ---------------------------------------------------------------------------
// Block Floating Point (BFP) Compression
// ---------------------------------------------------------------------------

/// BFP compressed block
#[derive(Debug, Clone)]
pub struct BfpBlock {
    pub exponent: u8,
    pub mantissa_bits: Vec<u8>,
    pub num_samples: usize,
    pub bit_width: u8,
}

/// BFP compressor/decompressor
pub struct BfpCodec {
    pub bit_width: u8,
}

impl BfpCodec {
    pub fn new(bit_width: u8) -> Self {
        assert!(matches!(bit_width, 8 | 9 | 12 | 14 | 16));
        Self { bit_width }
    }

    fn compute_exponent(&self, samples: &[IqSample]) -> u8 {
        let mantissa_bits = self.bit_width as u32 - 1;
        let max_mantissa: i32 = (1 << mantissa_bits) - 1;
        let max_abs = samples.iter().fold(0i32, |acc, s| {
            acc.max((s.i as i32).abs()).max((s.q as i32).abs())
        });
        if max_abs == 0 {
            return 0;
        }
        let mut exp = 0u8;
        let mut shifted = max_abs;
        while shifted > max_mantissa && exp < 15 {
            shifted >>= 1;
            exp += 1;
        }
        exp
    }

    pub fn compress(&self, samples: &[IqSample]) -> BfpBlock {
        let exponent = self.compute_exponent(samples);
        let bit_width = self.bit_width;
        let num_samples = samples.len();
        let total_bits = num_samples * 2 * bit_width as usize;
        let total_bytes = (total_bits + 7) / 8;
        let mut packed = vec![0u8; total_bytes];
        let mask: u32 = (1u32 << bit_width) - 1;
        let mut bit_pos = 0usize;

        for sample in samples {
            let i_m = ((sample.i >> exponent) as u32) & mask;
            let q_m = ((sample.q >> exponent) as u32) & mask;
            for &val in &[i_m, q_m] {
                for b in (0..bit_width as usize).rev() {
                    let bit = (val >> b) & 1;
                    let byte_idx = bit_pos / 8;
                    let bit_idx = 7 - (bit_pos % 8);
                    packed[byte_idx] |= (bit as u8) << bit_idx;
                    bit_pos += 1;
                }
            }
        }

        BfpBlock { exponent, mantissa_bits: packed, num_samples, bit_width }
    }

    pub fn decompress(&self, block: &BfpBlock) -> Vec<IqSample> {
        let bit_width = block.bit_width as usize;
        let exponent = block.exponent;
        let num_components = block.num_samples * 2;
        let mut components = Vec::with_capacity(num_components);
        let mask = (1u32 << bit_width) - 1;
        let sign_bit = 1u32 << (bit_width - 1);
        let mut bit_pos = 0usize;

        for _ in 0..num_components {
            let mut val = 0u32;
            for b in (0..bit_width).rev() {
                let byte_idx = bit_pos / 8;
                let bit_idx = 7 - (bit_pos % 8);
                if byte_idx < block.mantissa_bits.len() {
                    let bit = (block.mantissa_bits[byte_idx] >> bit_idx) & 1;
                    val |= (bit as u32) << b;
                }
                bit_pos += 1;
            }
            let signed_val = if val & sign_bit != 0 {
                (val | !mask) as i32
            } else {
                val as i32
            };
            let expanded = signed_val << exponent;
            components.push(expanded.clamp(i16::MIN as i32, i16::MAX as i32) as i16);
        }

        components.chunks(2).map(|c| IqSample::new(c[0], c[1])).collect()
    }
}

// ---------------------------------------------------------------------------
// Mu-law IQ Compression
// ---------------------------------------------------------------------------

/// Mu-law codec for IQ sample compression
pub struct MuLawCodec {
    pub bit_width: u8,
    pub mu: u32,
}

impl MuLawCodec {
    pub fn new(bit_width: u8) -> Self {
        let mu = (1u32 << bit_width) - 1;
        Self { bit_width, mu }
    }

    pub fn encode_sample(&self, sample: i16) -> u8 {
        let sign = if sample < 0 { 1u8 } else { 0u8 };
        let abs_val = (sample.abs() as u32).min(32767);
        let mu_val = self.mu;
        let scaled = (abs_val * mu_val) / 32767;
        let numerator = (scaled + 1) as f64;
        let encoded_f = (numerator.ln() / (1.0 + mu_val as f64).ln()
            * ((1u32 << (self.bit_width - 1)) as f64 - 1.0))
            .round() as u8;
        let encoded = encoded_f.min((1u8 << (self.bit_width - 1)) - 1);
        (sign << (self.bit_width - 1)) | encoded
    }

    pub fn decode_sample(&self, encoded: u8) -> i16 {
        let sign = (encoded >> (self.bit_width - 1)) & 1;
        let magnitude = encoded & ((1 << (self.bit_width - 1)) - 1);
        let mu_val = self.mu as f64;
        let max_out = ((1u32 << (self.bit_width - 1)) - 1) as f64;
        let frac = magnitude as f64 / max_out;
        let decoded_f = ((1.0 + mu_val).powf(frac) - 1.0) / mu_val * 32767.0;
        let decoded = decoded_f.round().min(32767.0) as i16;
        if sign == 1 { -decoded } else { decoded }
    }

    pub fn compress(&self, samples: &[IqSample]) -> Vec<u8> {
        let mut out = Vec::with_capacity(samples.len() * 2);
        for s in samples {
            out.push(self.encode_sample(s.i));
            out.push(self.encode_sample(s.q));
        }
        out
    }

    pub fn decompress(&self, data: &[u8]) -> Vec<IqSample> {
        data.chunks(2)
            .filter(|c| c.len() == 2)
            .map(|c| IqSample::new(self.decode_sample(c[0]), self.decode_sample(c[1])))
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Beamforming structures
// ---------------------------------------------------------------------------

/// Beam weight entry
#[derive(Debug, Clone)]
pub struct BeamWeight {
    pub beam_id: u16,
    pub prb_index: u16,
    pub weights: Vec<(i16, i16)>,
}

impl BeamWeight {
    pub fn new(beam_id: u16, prb_index: u16, weights: Vec<(i16, i16)>) -> Self {
        Self { beam_id, prb_index, weights }
    }

    pub fn unity(beam_id: u16, prb_index: u16, num_antennas: usize) -> Self {
        Self::new(beam_id, prb_index, vec![(32767i16, 0i16); num_antennas])
    }

    /// Steered ULA beam weights
    pub fn steered_ula(
        beam_id: u16,
        prb_index: u16,
        num_antennas: usize,
        theta_deg: f64,
        wavelength: f64,
        spacing: f64,
    ) -> Self {
        use std::f64::consts::PI;
        let theta_rad = theta_deg * PI / 180.0;
        let d_lambda = spacing / wavelength;
        let scale = 32767.0;
        let weights: Vec<(i16, i16)> = (0..num_antennas)
            .map(|n| {
                let phase = 2.0 * PI * n as f64 * d_lambda * theta_rad.sin();
                ((scale * phase.cos()) as i16, (scale * phase.sin()) as i16)
            })
            .collect();
        Self::new(beam_id, prb_index, weights)
    }
}

/// Beamforming configuration
#[derive(Debug, Clone)]
pub struct BeamformingConfig {
    pub num_antennas: usize,
    pub max_beams: usize,
    pub prb_beam_map: Vec<u16>,
    pub beam_weights: Vec<Option<BeamWeight>>,
}

impl BeamformingConfig {
    pub fn new(num_antennas: usize, num_prbs: usize) -> Self {
        let max_beams = MAX_BEAMS;
        let mut beam_weights = Vec::with_capacity(max_beams);
        beam_weights.resize_with(max_beams, || None);
        Self { num_antennas, max_beams, prb_beam_map: vec![0u16; num_prbs], beam_weights }
    }

    pub fn set_single_beam(&mut self, beam_id: u16, weights: Vec<(i16, i16)>) {
        for prb in self.prb_beam_map.iter_mut() {
            *prb = beam_id;
        }
        let idx = beam_id as usize;
        if idx < self.max_beams {
            self.beam_weights[idx] = Some(BeamWeight::new(beam_id, 0, weights));
        }
    }

    pub fn beam_id_for_prb(&self, prb: usize) -> u16 {
        self.prb_beam_map.get(prb).copied().unwrap_or(0)
    }
}

// ---------------------------------------------------------------------------
// Section Extensions
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SectionExtType {
    BfWeights = 1,
    DlPrecoderConfig = 3,
    GroupConfig = 11,
}

/// SE-1: Beamforming weight extension
#[derive(Debug, Clone)]
pub struct Se1BfWeights {
    pub beam_id: u16,
    pub num_weights: u8,
    pub weights: Vec<(i16, i16)>,
    pub ef: bool,
}

impl Se1BfWeights {
    pub fn new(beam_id: u16, weights: Vec<(i16, i16)>) -> Self {
        let num_weights = weights.len() as u8;
        Self { beam_id, num_weights, weights, ef: false }
    }

    pub fn serialize(&self) -> Vec<u8> {
        let mut v = Vec::new();
        v.push((SectionExtType::BfWeights as u8) | if self.ef { 0x80 } else { 0x00 });
        v.push(0x00);
        v.push((self.beam_id >> 8) as u8);
        v.push((self.beam_id & 0xFF) as u8);
        v.push(self.num_weights);
        v.push(0x00);
        for (wi, wq) in &self.weights {
            v.push((wi >> 8) as u8);
            v.push((*wi & 0xFF) as u8);
            v.push((wq >> 8) as u8);
            v.push((*wq & 0xFF) as u8);
        }
        v
    }
}

/// SE-11: Group configuration extension
#[derive(Debug, Clone)]
pub struct Se11GroupConfig {
    pub group_id: u8,
    pub num_ants: u8,
    pub ant_ids: Vec<u8>,
    pub ef: bool,
}

impl Se11GroupConfig {
    pub fn new(group_id: u8, ant_ids: Vec<u8>) -> Self {
        let num_ants = ant_ids.len() as u8;
        Self { group_id, num_ants, ant_ids, ef: false }
    }

    pub fn serialize(&self) -> Vec<u8> {
        let mut v = Vec::new();
        v.push(SectionExtType::GroupConfig as u8 | if self.ef { 0x80 } else { 0 });
        v.push(self.group_id);
        v.push(self.num_ants);
        v.extend_from_slice(&self.ant_ids);
        v
    }
}

// ---------------------------------------------------------------------------
// Timing model
// ---------------------------------------------------------------------------

/// O-RAN timing parameters
#[derive(Debug, Clone)]
pub struct OranTiming {
    pub t12_max_us: u32,
    pub t12_min_us: u32,
    pub t34_max_us: u32,
    pub t34_min_us: u32,
    pub ta3_min_us: u32,
    pub ta3_max_us: u32,
    pub accuracy_ns: u32,
    pub mu: u8,
}

impl OranTiming {
    pub fn cat_a(mu: u8) -> Self {
        let slot_us = 1000u32 / (1u32 << mu as u32);
        Self {
            t12_max_us: slot_us * 4,
            t12_min_us: slot_us,
            t34_max_us: slot_us * 2,
            t34_min_us: 0,
            ta3_min_us: slot_us,
            ta3_max_us: slot_us * 3,
            accuracy_ns: TIMING_ACCURACY_CAT_A_NS,
            mu,
        }
    }

    pub fn cat_b(mu: u8) -> Self {
        let slot_us = 1000u32 / (1u32 << mu as u32);
        Self {
            t12_max_us: slot_us * 8,
            t12_min_us: 0,
            t34_max_us: slot_us * 4,
            t34_min_us: 0,
            ta3_min_us: 0,
            ta3_max_us: slot_us * 6,
            accuracy_ns: TIMING_ACCURACY_CAT_B_NS,
            mu,
        }
    }

    pub fn slot_duration_us(&self) -> u32 {
        1000u32 / (1u32 << self.mu as u32)
    }

    pub fn check_ta3(&self, ta_us: u32) -> bool {
        ta_us >= self.ta3_min_us && ta_us <= self.ta3_max_us
    }

    pub fn check_t12(&self, advance_us: u32) -> bool {
        advance_us >= self.t12_min_us && advance_us <= self.t12_max_us
    }
}

// ---------------------------------------------------------------------------
// Frame structure
// ---------------------------------------------------------------------------

/// NR Radio Frame timing reference
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RadioFrameRef {
    pub sfn: u16,
    pub subframe: u8,
    pub slot: u8,
    pub symbol: u8,
    pub mu: u8,
}

impl RadioFrameRef {
    pub fn new(sfn: u16, subframe: u8, slot: u8, symbol: u8, mu: u8) -> Self {
        Self {
            sfn: sfn & 0x3FF,
            subframe: subframe % 10,
            slot,
            symbol: symbol % SYMBOLS_PER_SLOT as u8,
            mu,
        }
    }

    pub fn frame_slot(&self) -> u16 {
        let slots_per_subframe = (1u16) << self.mu as u16;
        self.subframe as u16 * slots_per_subframe + self.slot as u16
    }

    pub fn frame_symbol(&self) -> u32 {
        self.frame_slot() as u32 * SYMBOLS_PER_SLOT as u32 + self.symbol as u32
    }

    pub fn next_symbol(&self) -> Self {
        let slots_per_subframe = (1u8) << self.mu;
        let mut sym = self.symbol + 1;
        let mut slot = self.slot;
        let mut subframe = self.subframe;
        let mut sfn = self.sfn;
        if sym >= SYMBOLS_PER_SLOT as u8 {
            sym = 0;
            slot += 1;
            if slot >= slots_per_subframe {
                slot = 0;
                subframe += 1;
                if subframe >= SUBFRAMES_PER_FRAME as u8 {
                    subframe = 0;
                    sfn = (sfn + 1) & 0x3FF;
                }
            }
        }
        Self { sfn, subframe, slot, symbol: sym, mu: self.mu }
    }
}

// ---------------------------------------------------------------------------
// Complete eCPRI message frames
// ---------------------------------------------------------------------------

/// A complete C-Plane message
#[derive(Debug, Clone)]
pub struct CplaneMessage {
    pub ecpri_hdr: EcpriHeader,
    pub pc_id: u16,
    pub seq_id: u16,
    pub app_hdr: CplaneAppHeader,
    pub section_type: SectionType,
    pub sections_type1: Vec<SectionType1>,
    pub sections_type3: Vec<SectionType3>,
}

impl CplaneMessage {
    pub fn dl_type1(pc_id: u16, seq_id: u16, frame_ref: RadioFrameRef, sections: Vec<SectionType1>) -> Self {
        let app_hdr = CplaneAppHeader::new(
            DataDirection::Downlink,
            frame_ref.sfn as u8,
            frame_ref.subframe,
            frame_ref.slot,
            frame_ref.symbol,
        );
        let payload_size = 2 + 2 + 4 + 1 + sections.len() as u16 * 8;
        let ecpri_hdr = EcpriHeader::new(EcpriMessageType::RealTimeControl, payload_size);
        Self {
            ecpri_hdr,
            pc_id,
            seq_id,
            app_hdr,
            section_type: SectionType::MostChannels,
            sections_type1: sections,
            sections_type3: Vec::new(),
        }
    }

    pub fn serialize(&self) -> Vec<u8> {
        let mut buf = Vec::new();
        buf.extend_from_slice(&self.ecpri_hdr.serialize());
        buf.push((self.pc_id >> 8) as u8);
        buf.push((self.pc_id & 0xFF) as u8);
        buf.push((self.seq_id >> 8) as u8);
        buf.push((self.seq_id & 0xFF) as u8);
        buf.extend_from_slice(&self.app_hdr.serialize());
        buf.push(self.section_type as u8);
        for sec in &self.sections_type1 {
            buf.extend_from_slice(&sec.serialize());
        }
        for sec in &self.sections_type3 {
            buf.extend_from_slice(&sec.serialize());
        }
        buf
    }
}

/// A complete U-Plane message
#[derive(Debug, Clone)]
pub struct UplaneMessage {
    pub ecpri_hdr: EcpriHeader,
    pub pc_id: u16,
    pub seq_id: u16,
    pub app_hdr: UplaneAppHeader,
    pub sections: Vec<UplaneSectionHeader>,
    pub iq_data: Vec<Vec<u8>>,
}

impl UplaneMessage {
    pub fn new_uncompressed(
        pc_id: u16,
        seq_id: u16,
        frame_ref: RadioFrameRef,
        dir: DataDirection,
        section_id: u16,
        start_prb: u16,
        samples: &[IqSample],
    ) -> Self {
        let num_prb = (samples.len() / 12) as u16;
        let app_hdr = UplaneAppHeader::new(dir, frame_ref.sfn as u8, frame_ref.subframe, frame_ref.slot, frame_ref.symbol);
        let sec = UplaneSectionHeader::new(section_id, start_prb, num_prb);
        let mut iq_bytes = Vec::with_capacity(samples.len() * 4);
        for s in samples {
            iq_bytes.push((s.i >> 8) as u8);
            iq_bytes.push((s.i & 0xFF) as u8);
            iq_bytes.push((s.q >> 8) as u8);
            iq_bytes.push((s.q & 0xFF) as u8);
        }
        let payload_size = 2 + 2 + 4 + 6 + iq_bytes.len() as u16;
        let ecpri_hdr = EcpriHeader::new(EcpriMessageType::IqData, payload_size);
        Self { ecpri_hdr, pc_id, seq_id, app_hdr, sections: vec![sec], iq_data: vec![iq_bytes] }
    }

    pub fn new_bfp(
        pc_id: u16,
        seq_id: u16,
        frame_ref: RadioFrameRef,
        dir: DataDirection,
        section_id: u16,
        start_prb: u16,
        samples: &[IqSample],
        bit_width: u8,
    ) -> Self {
        let codec = BfpCodec::new(bit_width);
        let block = codec.compress(samples);
        let num_prb = (samples.len() / 12) as u16;
        let app_hdr = UplaneAppHeader::new(dir, frame_ref.sfn as u8, frame_ref.subframe, frame_ref.slot, frame_ref.symbol);
        let iq_w = IqWidth::from_encoded(if bit_width == 16 { 0 } else { bit_width }).unwrap();
        let sec = UplaneSectionHeader::new(section_id, start_prb, num_prb)
            .with_compression(iq_w, CompressionMethod::BlockFloatingPoint);
        let mut iq_bytes = vec![block.exponent];
        iq_bytes.extend_from_slice(&block.mantissa_bits);
        let payload_size = 2 + 2 + 4 + 6 + iq_bytes.len() as u16;
        let ecpri_hdr = EcpriHeader::new(EcpriMessageType::IqData, payload_size);
        Self { ecpri_hdr, pc_id, seq_id, app_hdr, sections: vec![sec], iq_data: vec![iq_bytes] }
    }

    pub fn serialize(&self) -> Vec<u8> {
        let mut buf = Vec::new();
        buf.extend_from_slice(&self.ecpri_hdr.serialize());
        buf.push((self.pc_id >> 8) as u8);
        buf.push((self.pc_id & 0xFF) as u8);
        buf.push((self.seq_id >> 8) as u8);
        buf.push((self.seq_id & 0xFF) as u8);
        buf.extend_from_slice(&self.app_hdr.serialize());
        for (sec, iq) in self.sections.iter().zip(self.iq_data.iter()) {
            buf.extend_from_slice(&sec.serialize());
            buf.extend_from_slice(iq);
        }
        buf
    }

    pub fn total_size(&self) -> usize {
        4 + self.ecpri_hdr.payload_size as usize
    }
}

// ---------------------------------------------------------------------------
// O-RAN Fronthaul Processor
// ---------------------------------------------------------------------------

/// Configuration
#[derive(Debug, Clone)]
pub struct OranConfig {
    pub scs: SubcarrierSpacing,
    pub num_prbs: usize,
    pub num_antennas: usize,
    pub iq_width: IqWidth,
    pub compression: CompressionMethod,
    pub timing: OranTiming,
}

impl OranConfig {
    pub fn new(scs: SubcarrierSpacing, num_prbs: usize, num_antennas: usize) -> Self {
        let mu = scs.mu();
        Self {
            scs,
            num_prbs,
            num_antennas,
            iq_width: IqWidth::Bits16,
            compression: CompressionMethod::NoCompression,
            timing: OranTiming::cat_a(mu),
        }
    }

    pub fn with_bfp(mut self, bit_width: u8) -> Self {
        self.iq_width = IqWidth::from_encoded(if bit_width == 16 { 0 } else { bit_width })
            .unwrap_or(IqWidth::Bits16);
        self.compression = CompressionMethod::BlockFloatingPoint;
        self
    }

    pub fn with_mu_law(mut self) -> Self {
        self.iq_width = IqWidth::Bits8;
        self.compression = CompressionMethod::ULaw;
        self
    }
}

/// Statistics counters
#[derive(Debug, Default, Clone)]
pub struct FronthaulStats {
    pub cplane_tx: u64,
    pub cplane_rx: u64,
    pub uplane_tx: u64,
    pub uplane_rx: u64,
    pub bytes_tx: u64,
    pub bytes_rx: u64,
    pub seq_errors: u64,
    pub timing_violations: u64,
}

/// O-RAN Fronthaul Processor — main orchestrating struct
pub struct OranFronthaulProcessor {
    pub config: OranConfig,
    pub beamforming: BeamformingConfig,
    pub stats: FronthaulStats,
    seq_counter: u16,
    tx_queue: VecDeque<Vec<u8>>,
    pub frame_ref: RadioFrameRef,
}

impl OranFronthaulProcessor {
    pub fn new(config: OranConfig) -> Self {
        let num_prbs = config.num_prbs;
        let num_antennas = config.num_antennas;
        let mu = config.scs.mu();
        Self {
            beamforming: BeamformingConfig::new(num_antennas, num_prbs),
            stats: FronthaulStats::default(),
            seq_counter: 0,
            tx_queue: VecDeque::new(),
            frame_ref: RadioFrameRef::new(0, 0, 0, 0, mu),
            config,
        }
    }

    fn next_seq(&mut self) -> u16 {
        let s = self.seq_counter;
        self.seq_counter = self.seq_counter.wrapping_add(1);
        s
    }

    pub fn schedule_dl_slot(
        &mut self,
        pc_id: u16,
        sections: Vec<SectionType1>,
        iq_per_section: Vec<Vec<IqSample>>,
    ) -> Vec<Vec<u8>> {
        let mut messages = Vec::new();
        let seq = self.next_seq();
        let cplane = CplaneMessage::dl_type1(pc_id, seq, self.frame_ref, sections);
        let cp_bytes = cplane.serialize();
        self.stats.bytes_tx += cp_bytes.len() as u64;
        self.stats.cplane_tx += 1;
        messages.push(cp_bytes);

        for (i, iq) in iq_per_section.into_iter().enumerate() {
            let seq = self.next_seq();
            let uplane = match self.config.compression {
                CompressionMethod::BlockFloatingPoint => UplaneMessage::new_bfp(
                    pc_id, seq, self.frame_ref, DataDirection::Downlink,
                    i as u16, 0, &iq, self.config.iq_width.bits(),
                ),
                _ => UplaneMessage::new_uncompressed(
                    pc_id, seq, self.frame_ref, DataDirection::Downlink,
                    i as u16, 0, &iq,
                ),
            };
            let up_bytes = uplane.serialize();
            self.stats.bytes_tx += up_bytes.len() as u64;
            self.stats.uplane_tx += 1;
            messages.push(up_bytes);
        }
        messages
    }

    pub fn advance_symbol(&mut self) {
        self.frame_ref = self.frame_ref.next_symbol();
    }

    pub fn advance_slot(&mut self) {
        for _ in 0..SYMBOLS_PER_SLOT {
            self.frame_ref = self.frame_ref.next_symbol();
        }
    }

    pub fn receive_uplane(&mut self, data: &[u8]) -> Option<(UplaneSectionHeader, Vec<IqSample>)> {
        if data.len() < 18 {
            return None;
        }
        // eCPRI(4) + pc_id(2) + seq_id(2) + app_hdr(4) = 12 bytes offset
        let offset = 12;
        let sec_hdr = UplaneSectionHeader::deserialize(&data[offset..])?;
        let iq_offset = offset + 6;
        let iq_data = &data[iq_offset..];
        self.stats.uplane_rx += 1;
        self.stats.bytes_rx += data.len() as u64;

        let samples = match sec_hdr.comp_meth {
            CompressionMethod::BlockFloatingPoint => {
                if iq_data.is_empty() {
                    return None;
                }
                let exponent = iq_data[0];
                let mantissa = iq_data[1..].to_vec();
                let bit_width = sec_hdr.iq_width.bits();
                let num_samples = sec_hdr.num_prb as usize * 12;
                let block = BfpBlock { exponent, mantissa_bits: mantissa, num_samples, bit_width };
                BfpCodec::new(bit_width).decompress(&block)
            }
            CompressionMethod::ULaw => {
                MuLawCodec::new(8).decompress(iq_data)
            }
            CompressionMethod::NoCompression => {
                iq_data.chunks(4).filter(|c| c.len() == 4).map(|c| {
                    let i = ((c[0] as i16) << 8) | c[1] as i16;
                    let q = ((c[2] as i16) << 8) | c[3] as i16;
                    IqSample::new(i, q)
                }).collect()
            }
            _ => return None,
        };

        Some((sec_hdr, samples))
    }

    pub fn check_timing_advance(&self, ta_us: u32) -> bool {
        self.config.timing.check_ta3(ta_us)
    }

    pub fn num_prbs(&self) -> usize {
        self.config.num_prbs
    }

    pub fn num_antennas(&self) -> usize {
        self.config.num_antennas
    }
}

// ---------------------------------------------------------------------------
// Sequence tracking
// ---------------------------------------------------------------------------

pub struct SequenceTracker {
    expected_seq: u16,
    reorder_depth: usize,
    buffer: VecDeque<(u16, Vec<u8>)>,
    pub gap_count: u64,
    pub reorder_count: u64,
}

impl SequenceTracker {
    pub fn new(reorder_depth: usize) -> Self {
        Self {
            expected_seq: 0,
            reorder_depth,
            buffer: VecDeque::new(),
            gap_count: 0,
            reorder_count: 0,
        }
    }

    pub fn insert(&mut self, seq_id: u16, data: Vec<u8>) -> Vec<Vec<u8>> {
        if seq_id == self.expected_seq {
            self.expected_seq = self.expected_seq.wrapping_add(1);
            let mut out = vec![data];
            loop {
                if let Some(pos) = self.buffer.iter().position(|(s, _)| *s == self.expected_seq) {
                    let (_, d) = self.buffer.remove(pos).unwrap();
                    out.push(d);
                    self.expected_seq = self.expected_seq.wrapping_add(1);
                } else {
                    break;
                }
            }
            out
        } else if seq_id.wrapping_sub(self.expected_seq) < 0x8000 {
            self.buffer.push_back((seq_id, data));
            self.reorder_count += 1;
            if self.buffer.len() > self.reorder_depth {
                self.gap_count += 1;
                let (_, old) = self.buffer.pop_front().unwrap();
                self.expected_seq = self.expected_seq.wrapping_add(1);
                vec![old]
            } else {
                Vec::new()
            }
        } else {
            self.gap_count += 1;
            Vec::new()
        }
    }

    pub fn reset(&mut self) {
        self.expected_seq = 0;
        self.buffer.clear();
        self.gap_count = 0;
        self.reorder_count = 0;
    }
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

pub const fn re_per_prb() -> usize { 12 }

pub fn total_re(num_prbs: usize) -> usize { num_prbs * re_per_prb() }

pub fn uncompressed_iq_bytes(num_samples: usize) -> usize { num_samples * 4 }

pub fn bfp_iq_bytes(num_samples: usize, bit_width: u8) -> usize {
    let bits = num_samples * 2 * bit_width as usize;
    1 + (bits + 7) / 8
}

pub fn compression_ratio(num_samples: usize, comp_bytes: usize) -> f64 {
    uncompressed_iq_bytes(num_samples) as f64 / comp_bytes as f64
}

pub fn scs_khz(mu: u8) -> Option<u32> {
    SubcarrierSpacing::from_mu(mu).map(|s| s.hz() / 1000)
}

pub fn max_prbs_for_bandwidth(bandwidth_mhz: u32, mu: u8) -> Option<u32> {
    match (bandwidth_mhz, mu) {
        (5, 0) => Some(25),
        (10, 0) => Some(52),
        (15, 0) => Some(79),
        (20, 0) => Some(106),
        (25, 0) => Some(133),
        (30, 0) => Some(160),
        (40, 0) => Some(216),
        (50, 0) => Some(270),
        (10, 1) => Some(24),
        (15, 1) => Some(38),
        (20, 1) => Some(51),
        (25, 1) => Some(65),
        (30, 1) => Some(78),
        (40, 1) => Some(106),
        (50, 1) => Some(133),
        (60, 1) => Some(162),
        (80, 1) => Some(217),
        (100, 1) => Some(273),
        (50, 2) => Some(128),
        (100, 2) => Some(264),
        (50, 3) => Some(66),
        (100, 3) => Some(132),
        (200, 3) => Some(264),
        _ => None,
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_ecpri_header_serialize_deserialize() {
        let hdr = EcpriHeader::new(EcpriMessageType::IqData, 512);
        let bytes = hdr.serialize();
        let decoded = EcpriHeader::deserialize(&bytes).unwrap();
        assert_eq!(decoded.message_type, EcpriMessageType::IqData);
        assert_eq!(decoded.payload_size, 512);
        assert_eq!(decoded.version, ECPRI_VERSION);
    }

    #[test]
    fn test_ecpri_header_concatenated() {
        let mut hdr = EcpriHeader::new(EcpriMessageType::RealTimeControl, 100);
        hdr.concatenated = true;
        let bytes = hdr.serialize();
        let decoded = EcpriHeader::deserialize(&bytes).unwrap();
        assert!(decoded.concatenated);
    }

    #[test]
    fn test_ecpri_header_total_len() {
        let hdr = EcpriHeader::new(EcpriMessageType::IqData, 1000);
        assert_eq!(hdr.total_len(), 1004);
    }

    #[test]
    fn test_ecpri_message_type_roundtrip() {
        for mt in [
            EcpriMessageType::IqData,
            EcpriMessageType::RealTimeControl,
            EcpriMessageType::BitSequence,
            EcpriMessageType::OneWayDelayMeasurement,
            EcpriMessageType::IqDataWithTimestamp,
        ] {
            assert_eq!(EcpriMessageType::from_u8(mt as u8).unwrap(), mt);
        }
    }

    #[test]
    fn test_ecpri_header_too_short() {
        assert!(EcpriHeader::deserialize(&[0x12, 0x00]).is_none());
    }

    #[test]
    fn test_cplane_app_header_dl() {
        let hdr = CplaneAppHeader::new(DataDirection::Downlink, 5, 3, 1, 7);
        let bytes = hdr.serialize();
        let dec = CplaneAppHeader::deserialize(&bytes).unwrap();
        assert_eq!(dec.data_direction, DataDirection::Downlink);
        assert_eq!(dec.frame_id, 5);
        assert_eq!(dec.subframe_id, 3);
        assert_eq!(dec.slot_id, 1);
        assert_eq!(dec.symbol_id, 7);
    }

    #[test]
    fn test_cplane_app_header_ul() {
        let hdr = CplaneAppHeader::new(DataDirection::Uplink, 0, 0, 0, 0);
        let bytes = hdr.serialize();
        let dec = CplaneAppHeader::deserialize(&bytes).unwrap();
        assert_eq!(dec.data_direction, DataDirection::Uplink);
    }

    #[test]
    fn test_cplane_app_header_payload_version() {
        let hdr = CplaneAppHeader::new(DataDirection::Downlink, 1, 2, 0, 0);
        let bytes = hdr.serialize();
        let dec = CplaneAppHeader::deserialize(&bytes).unwrap();
        assert_eq!(dec.payload_version, 0x01);
    }

    #[test]
    fn test_section_type1_serialize_deserialize() {
        let sec = SectionType1::new(42, 10, 52, 3);
        let bytes = sec.serialize();
        let dec = SectionType1::deserialize(&bytes).unwrap();
        assert_eq!(dec.section_id, 42);
        assert_eq!(dec.start_prb, 10);
        assert_eq!(dec.num_prb, 52);
        assert_eq!(dec.beam_id, 3);
    }

    #[test]
    fn test_section_type1_re_mask() {
        let sec = SectionType1::new(1, 0, 106, 0);
        let bytes = sec.serialize();
        let dec = SectionType1::deserialize(&bytes).unwrap();
        assert_eq!(dec.re_mask, 0xFFF);
    }

    #[test]
    fn test_section_type1_extension_flag() {
        let mut sec = SectionType1::new(1, 0, 10, 0);
        sec.ef = true;
        let bytes = sec.serialize();
        let dec = SectionType1::deserialize(&bytes).unwrap();
        assert!(dec.ef);
    }

    #[test]
    fn test_section_type1_too_short() {
        assert!(SectionType1::deserialize(&[0u8; 4]).is_none());
    }

    #[test]
    fn test_section_type3_serialize() {
        let sec = SectionType3::new(100, 0, 6, 0, 500);
        let bytes = sec.serialize();
        assert_eq!(bytes.len(), 12);
    }

    #[test]
    fn test_section_type3_frame_structure_default() {
        let sec = SectionType3::new(1, 0, 1, 0, 0);
        let bytes = sec.serialize();
        assert_eq!(bytes[10], 0xFF);
    }

    #[test]
    fn test_uplane_section_header_no_compression() {
        let hdr = UplaneSectionHeader::new(10, 0, 52);
        let bytes = hdr.serialize();
        let dec = UplaneSectionHeader::deserialize(&bytes).unwrap();
        assert_eq!(dec.section_id, 10);
        assert_eq!(dec.num_prb, 52);
        assert_eq!(dec.comp_meth, CompressionMethod::NoCompression);
    }

    #[test]
    fn test_uplane_section_header_bfp() {
        let hdr = UplaneSectionHeader::new(1, 0, 25)
            .with_compression(IqWidth::Bits9, CompressionMethod::BlockFloatingPoint);
        let bytes = hdr.serialize();
        let dec = UplaneSectionHeader::deserialize(&bytes).unwrap();
        assert_eq!(dec.comp_meth, CompressionMethod::BlockFloatingPoint);
        assert_eq!(dec.iq_width, IqWidth::Bits9);
    }

    #[test]
    fn test_uplane_section_header_too_short() {
        assert!(UplaneSectionHeader::deserialize(&[0u8; 3]).is_none());
    }

    #[test]
    fn test_bfp_codec_identity_16bit() {
        let codec = BfpCodec::new(16);
        let samples = vec![
            IqSample::new(1000, -500),
            IqSample::new(-32000, 32000),
            IqSample::new(0, 0),
        ];
        let block = codec.compress(&samples);
        assert_eq!(block.exponent, 0);
        let recovered = codec.decompress(&block);
        for (orig, rec) in samples.iter().zip(recovered.iter()) {
            assert_eq!(orig.i, rec.i);
            assert_eq!(orig.q, rec.q);
        }
    }

    #[test]
    fn test_bfp_codec_8bit() {
        let codec = BfpCodec::new(8);
        let samples = vec![IqSample::new(1000, -500), IqSample::new(256, 128)];
        let block = codec.compress(&samples);
        let recovered = codec.decompress(&block);
        assert_eq!(recovered.len(), 2);
        let max_err = (1i32 << (block.exponent + 1)) as i32;
        for (orig, rec) in samples.iter().zip(recovered.iter()) {
            assert!((orig.i as i32 - rec.i as i32).abs() <= max_err,
                "I mismatch: {} vs {}", orig.i, rec.i);
        }
    }

    #[test]
    fn test_bfp_codec_12bit() {
        let codec = BfpCodec::new(12);
        let samples: Vec<IqSample> = (0..12)
            .map(|i| IqSample::new(i as i16 * 100, -(i as i16 * 50)))
            .collect();
        let block = codec.compress(&samples);
        let recovered = codec.decompress(&block);
        assert_eq!(recovered.len(), 12);
        let max_err = (1i32 << (block.exponent + 1)).max(1) * 4;
        for (orig, rec) in samples.iter().zip(recovered.iter()) {
            assert!((orig.i as i32 - rec.i as i32).abs() <= max_err);
        }
    }

    #[test]
    fn test_bfp_codec_zero_samples() {
        let codec = BfpCodec::new(8);
        let samples = vec![IqSample::zero(); 12];
        let block = codec.compress(&samples);
        assert_eq!(block.exponent, 0);
        let recovered = codec.decompress(&block);
        for s in &recovered {
            assert_eq!(s.i, 0);
            assert_eq!(s.q, 0);
        }
    }

    #[test]
    fn test_bfp_codec_large_values() {
        let codec = BfpCodec::new(9);
        let samples = vec![IqSample::new(i16::MAX, i16::MIN), IqSample::new(-16384, 16384)];
        let block = codec.compress(&samples);
        assert!(block.exponent > 0);
        let _recovered = codec.decompress(&block);
    }

    #[test]
    fn test_bfp_block_roundtrip_14bit() {
        let codec = BfpCodec::new(14);
        let samples: Vec<IqSample> = (0..24)
            .map(|i| IqSample::new((i * 300 - 3600) as i16, (i * 200 - 2400) as i16))
            .collect();
        let block = codec.compress(&samples);
        let recovered = codec.decompress(&block);
        let max_err = (1i32 << (block.exponent + 1)).max(1) * 4;
        for (orig, rec) in samples.iter().zip(recovered.iter()) {
            assert!((orig.i as i32 - rec.i as i32).abs() <= max_err);
        }
    }

    #[test]
    fn test_mulaw_encode_decode_zero() {
        let codec = MuLawCodec::new(8);
        let enc = codec.encode_sample(0);
        let dec = codec.decode_sample(enc);
        assert!(dec.abs() <= 2, "decode(encode(0))={}", dec);
    }

    #[test]
    fn test_mulaw_encode_positive() {
        let codec = MuLawCodec::new(8);
        let enc = codec.encode_sample(1000);
        assert!(enc < 128);
    }

    #[test]
    fn test_mulaw_encode_negative() {
        let codec = MuLawCodec::new(8);
        let enc = codec.encode_sample(-1000);
        assert!(enc >= 128);
    }

    #[test]
    fn test_mulaw_compress_decompress_sign() {
        let codec = MuLawCodec::new(8);
        let samples = vec![
            IqSample::new(500, -500),
            IqSample::new(1000, 200),
            IqSample::new(-300, 800),
        ];
        let compressed = codec.compress(&samples);
        assert_eq!(compressed.len(), 6);
        let decompressed = codec.decompress(&compressed);
        assert_eq!(decompressed.len(), 3);
        for (orig, rec) in samples.iter().zip(decompressed.iter()) {
            if orig.i != 0 {
                assert_eq!(orig.i.signum(), rec.i.signum());
            }
        }
    }

    #[test]
    fn test_iq_width_bits16_encoded_as_zero() {
        assert_eq!(IqWidth::Bits16.encoded(), 0);
        assert_eq!(IqWidth::Bits16.bits(), 16);
    }

    #[test]
    fn test_iq_width_roundtrip() {
        for w in [IqWidth::Bits8, IqWidth::Bits9, IqWidth::Bits12, IqWidth::Bits14, IqWidth::Bits16] {
            let enc = w.encoded();
            let dec = IqWidth::from_encoded(enc).unwrap();
            assert_eq!(dec, w);
        }
    }

    #[test]
    fn test_scs_slots_per_subframe() {
        assert_eq!(SubcarrierSpacing::Scs15kHz.slots_per_subframe(), 1);
        assert_eq!(SubcarrierSpacing::Scs30kHz.slots_per_subframe(), 2);
        assert_eq!(SubcarrierSpacing::Scs60kHz.slots_per_subframe(), 4);
        assert_eq!(SubcarrierSpacing::Scs120kHz.slots_per_subframe(), 8);
    }

    #[test]
    fn test_scs_hz() {
        assert_eq!(SubcarrierSpacing::Scs15kHz.hz(), 15_000);
        assert_eq!(SubcarrierSpacing::Scs120kHz.hz(), 120_000);
    }

    #[test]
    fn test_scs_mu_roundtrip() {
        for mu in 0u8..4 {
            let scs = SubcarrierSpacing::from_mu(mu).unwrap();
            assert_eq!(scs.mu(), mu);
        }
    }

    #[test]
    fn test_scs_invalid_mu() {
        assert!(SubcarrierSpacing::from_mu(4).is_none());
        assert!(SubcarrierSpacing::from_mu(255).is_none());
    }

    #[test]
    fn test_timing_cat_a_slot_duration_mu0() {
        let t = OranTiming::cat_a(0);
        assert_eq!(t.slot_duration_us(), 1000);
    }

    #[test]
    fn test_timing_cat_a_slot_duration_mu1() {
        let t = OranTiming::cat_a(1);
        assert_eq!(t.slot_duration_us(), 500);
    }

    #[test]
    fn test_timing_cat_a_slot_duration_mu3() {
        let t = OranTiming::cat_a(3);
        assert_eq!(t.slot_duration_us(), 125);
    }

    #[test]
    fn test_timing_ta3_check() {
        let t = OranTiming::cat_a(0);
        assert!(t.check_ta3(t.ta3_min_us));
        assert!(t.check_ta3(t.ta3_max_us));
        assert!(!t.check_ta3(t.ta3_max_us + 1000));
    }

    #[test]
    fn test_timing_t12_check() {
        let t = OranTiming::cat_a(0);
        assert!(t.check_t12(t.t12_min_us));
        assert!(t.check_t12(t.t12_max_us));
        assert!(!t.check_t12(t.t12_max_us + 5000));
    }

    #[test]
    fn test_timing_accuracy_cat_a() {
        let t = OranTiming::cat_a(0);
        assert_eq!(t.accuracy_ns, TIMING_ACCURACY_CAT_A_NS);
    }

    #[test]
    fn test_timing_cat_b() {
        let t = OranTiming::cat_b(0);
        // cat_b has t12_max = 8 slots, cat_a has 4 slots
        let ta = OranTiming::cat_a(0);
        assert!(t.t12_max_us > ta.t12_max_us);
    }

    #[test]
    fn test_radio_frame_ref_new() {
        let r = RadioFrameRef::new(100, 3, 1, 7, 1);
        assert_eq!(r.sfn, 100);
        assert_eq!(r.subframe, 3);
        assert_eq!(r.slot, 1);
        assert_eq!(r.symbol, 7);
    }

    #[test]
    fn test_radio_frame_ref_frame_slot_mu0() {
        let r = RadioFrameRef::new(0, 5, 0, 0, 0);
        assert_eq!(r.frame_slot(), 5);
    }

    #[test]
    fn test_radio_frame_ref_frame_slot_mu1() {
        let r = RadioFrameRef::new(0, 5, 1, 0, 1);
        assert_eq!(r.frame_slot(), 11);
    }

    #[test]
    fn test_radio_frame_ref_next_symbol_wraps_slot() {
        // mu=1 has 2 slots/subframe, so slot 0 -> slot 1 wraps within the same subframe
        let r = RadioFrameRef::new(0, 0, 0, 13, 1);
        let n = r.next_symbol();
        assert_eq!(n.symbol, 0);
        assert_eq!(n.slot, 1);
        assert_eq!(n.subframe, 0); // still in subframe 0
    }

    #[test]
    fn test_radio_frame_ref_next_symbol_wraps_subframe() {
        let r = RadioFrameRef::new(0, 0, 0, 13, 0);
        let n = r.next_symbol();
        assert_eq!(n.subframe, 1);
        assert_eq!(n.slot, 0);
    }

    #[test]
    fn test_radio_frame_ref_sfn_wraps() {
        let r = RadioFrameRef::new(1023, 9, 0, 13, 0);
        let n = r.next_symbol();
        assert_eq!(n.sfn, 0);
    }

    #[test]
    fn test_radio_frame_ref_frame_symbol() {
        let r = RadioFrameRef::new(0, 0, 0, 7, 0);
        assert_eq!(r.frame_symbol(), 7);
    }

    #[test]
    fn test_cplane_message_dl_type1_ecpri_type() {
        let frame = RadioFrameRef::new(10, 2, 0, 0, 0);
        let sections = vec![SectionType1::new(1, 0, 52, 5)];
        let msg = CplaneMessage::dl_type1(0x0001, 42, frame, sections);
        let bytes = msg.serialize();
        let hdr = EcpriHeader::deserialize(&bytes).unwrap();
        assert_eq!(hdr.message_type, EcpriMessageType::RealTimeControl);
    }

    #[test]
    fn test_cplane_message_multiple_sections_payload_size() {
        let frame = RadioFrameRef::new(0, 0, 0, 0, 0);
        let sections = vec![
            SectionType1::new(0, 0, 25, 0),
            SectionType1::new(1, 25, 25, 1),
            SectionType1::new(2, 50, 27, 2),
        ];
        let msg = CplaneMessage::dl_type1(1, 0, frame, sections);
        let bytes = msg.serialize();
        let hdr = EcpriHeader::deserialize(&bytes).unwrap();
        assert_eq!(hdr.payload_size, 2 + 2 + 4 + 1 + 3 * 8);
    }

    #[test]
    fn test_uplane_message_uncompressed_type() {
        let frame = RadioFrameRef::new(0, 0, 0, 0, 0);
        let samples: Vec<IqSample> = (0..12).map(|i| IqSample::new(i * 100, -(i * 50))).collect();
        let msg = UplaneMessage::new_uncompressed(1, 5, frame, DataDirection::Downlink, 0, 0, &samples);
        let bytes = msg.serialize();
        let hdr = EcpriHeader::deserialize(&bytes).unwrap();
        assert_eq!(hdr.message_type, EcpriMessageType::IqData);
    }

    #[test]
    fn test_uplane_message_bfp_smaller_than_raw() {
        let frame = RadioFrameRef::new(0, 0, 0, 0, 0);
        let samples: Vec<IqSample> = (0..12).map(|i| IqSample::new(i * 200, i * 100)).collect();
        let raw = UplaneMessage::new_uncompressed(1, 0, frame, DataDirection::Downlink, 0, 0, &samples);
        let bfp = UplaneMessage::new_bfp(1, 0, frame, DataDirection::Downlink, 0, 0, &samples, 9);
        assert!(bfp.serialize().len() < raw.serialize().len());
    }

    #[test]
    fn test_uplane_message_total_size_consistency() {
        let frame = RadioFrameRef::new(0, 0, 0, 0, 0);
        let samples = vec![IqSample::zero(); 12];
        let msg = UplaneMessage::new_uncompressed(1, 0, frame, DataDirection::Uplink, 0, 0, &samples);
        assert_eq!(msg.serialize().len(), msg.total_size());
    }

    #[test]
    fn test_beam_weight_unity() {
        let bw = BeamWeight::unity(0, 0, 4);
        assert_eq!(bw.weights.len(), 4);
        assert_eq!(bw.weights[0], (32767, 0));
    }

    #[test]
    fn test_beam_weight_steered_ula_broadside() {
        let bw = BeamWeight::steered_ula(0, 0, 4, 0.0, 1.0, 0.5);
        for (i, q) in &bw.weights {
            assert_eq!(*i, 32767);
            assert!((*q).abs() <= 2);
        }
    }

    #[test]
    fn test_beamforming_config_single_beam() {
        let mut bf = BeamformingConfig::new(4, 25);
        bf.set_single_beam(3, vec![(16384, 0); 4]);
        for prb in 0..25 {
            assert_eq!(bf.beam_id_for_prb(prb), 3);
        }
    }

    #[test]
    fn test_beamforming_config_oob_prb() {
        let bf = BeamformingConfig::new(4, 10);
        assert_eq!(bf.beam_id_for_prb(9999), 0);
    }

    #[test]
    fn test_se1_bf_weights_serialize_length() {
        let ext = Se1BfWeights::new(5, vec![(16384, 0), (-16384, 8192)]);
        let bytes = ext.serialize();
        assert!(bytes.len() >= 6 + 2 * 4);
    }

    #[test]
    fn test_se1_bf_weights_beam_id() {
        let ext = Se1BfWeights::new(5, vec![(0, 0)]);
        let bytes = ext.serialize();
        assert_eq!(bytes[2], 0x00);
        assert_eq!(bytes[3], 0x05);
    }

    #[test]
    fn test_se11_group_config_serialize() {
        let ext = Se11GroupConfig::new(2, vec![0, 1, 2, 3]);
        let bytes = ext.serialize();
        assert!(bytes.len() >= 3 + 4);
        assert_eq!(bytes[1], 2);
        assert_eq!(bytes[2], 4);
    }

    #[test]
    fn test_seq_tracker_in_order() {
        let mut tracker = SequenceTracker::new(4);
        for seq in 0u16..5 {
            let out = tracker.insert(seq, vec![seq as u8]);
            assert_eq!(out.len(), 1);
            assert_eq!(out[0][0], seq as u8);
        }
        assert_eq!(tracker.gap_count, 0);
    }

    #[test]
    fn test_seq_tracker_out_of_order() {
        let mut tracker = SequenceTracker::new(4);
        let out1 = tracker.insert(1, vec![1]);
        assert_eq!(out1.len(), 0);
        let out0 = tracker.insert(0, vec![0]);
        assert_eq!(out0.len(), 2);
        assert_eq!(out0[0][0], 0);
        assert_eq!(out0[1][0], 1);
    }

    #[test]
    fn test_seq_tracker_gap_detection() {
        let mut tracker = SequenceTracker::new(2);
        tracker.insert(2, vec![2]);
        tracker.insert(3, vec![3]);
        tracker.insert(4, vec![4]);
        assert!(tracker.gap_count > 0 || tracker.reorder_count > 0);
    }

    #[test]
    fn test_seq_tracker_reset() {
        let mut tracker = SequenceTracker::new(4);
        tracker.insert(5, vec![]);
        tracker.reset();
        assert_eq!(tracker.gap_count, 0);
        assert_eq!(tracker.reorder_count, 0);
        let out = tracker.insert(0, vec![99]);
        assert_eq!(out.len(), 1);
    }

    #[test]
    fn test_re_per_prb() {
        assert_eq!(re_per_prb(), 12);
    }

    #[test]
    fn test_total_re() {
        assert_eq!(total_re(25), 300);
        assert_eq!(total_re(106), 1272);
    }

    #[test]
    fn test_uncompressed_iq_bytes() {
        assert_eq!(uncompressed_iq_bytes(12), 48);
    }

    #[test]
    fn test_bfp_iq_bytes_less_than_raw() {
        let raw = uncompressed_iq_bytes(12);
        let bfp8 = bfp_iq_bytes(12, 8);
        assert!(bfp8 < raw);
    }

    #[test]
    fn test_compression_ratio() {
        let bfp9 = bfp_iq_bytes(12, 9);
        let ratio = compression_ratio(12, bfp9);
        assert!(ratio > 1.0);
    }

    #[test]
    fn test_scs_khz() {
        assert_eq!(scs_khz(0), Some(15));
        assert_eq!(scs_khz(1), Some(30));
        assert_eq!(scs_khz(2), Some(60));
        assert_eq!(scs_khz(3), Some(120));
        assert_eq!(scs_khz(4), None);
    }

    #[test]
    fn test_max_prbs_known_configs() {
        assert_eq!(max_prbs_for_bandwidth(20, 0), Some(106));
        assert_eq!(max_prbs_for_bandwidth(100, 1), Some(273));
        assert_eq!(max_prbs_for_bandwidth(100, 3), Some(132));
    }

    #[test]
    fn test_max_prbs_unknown_config() {
        assert!(max_prbs_for_bandwidth(999, 0).is_none());
    }

    #[test]
    fn test_processor_new() {
        let cfg = OranConfig::new(SubcarrierSpacing::Scs30kHz, 106, 4);
        let proc = OranFronthaulProcessor::new(cfg);
        assert_eq!(proc.num_prbs(), 106);
        assert_eq!(proc.num_antennas(), 4);
    }

    #[test]
    fn test_processor_schedule_dl_slot() {
        let cfg = OranConfig::new(SubcarrierSpacing::Scs15kHz, 52, 2);
        let mut proc = OranFronthaulProcessor::new(cfg);
        let sections = vec![SectionType1::new(0, 0, 52, 0)];
        let iq: Vec<IqSample> = (0..12).map(|_| IqSample::new(100, -100)).collect();
        let msgs = proc.schedule_dl_slot(1, sections, vec![iq]);
        assert_eq!(msgs.len(), 2);
        assert!(proc.stats.cplane_tx > 0);
        assert!(proc.stats.uplane_tx > 0);
    }

    #[test]
    fn test_processor_advance_symbol() {
        let cfg = OranConfig::new(SubcarrierSpacing::Scs15kHz, 25, 2);
        let mut proc = OranFronthaulProcessor::new(cfg);
        assert_eq!(proc.frame_ref.symbol, 0);
        proc.advance_symbol();
        assert_eq!(proc.frame_ref.symbol, 1);
    }

    #[test]
    fn test_processor_advance_slot() {
        let cfg = OranConfig::new(SubcarrierSpacing::Scs15kHz, 25, 2);
        let mut proc = OranFronthaulProcessor::new(cfg);
        proc.advance_slot();
        assert_eq!(proc.frame_ref.subframe, 1);
        assert_eq!(proc.frame_ref.slot, 0);
    }

    #[test]
    fn test_processor_with_bfp_config() {
        let cfg = OranConfig::new(SubcarrierSpacing::Scs30kHz, 106, 4).with_bfp(9);
        let mut proc = OranFronthaulProcessor::new(cfg);
        let sections = vec![SectionType1::new(0, 0, 106, 0)];
        let iq: Vec<IqSample> = (0..12).map(|i| IqSample::new(i * 100, 0)).collect();
        let msgs = proc.schedule_dl_slot(1, sections, vec![iq]);
        assert!(msgs.len() >= 2);
    }

    #[test]
    fn test_processor_receive_uplane_roundtrip() {
        let frame = RadioFrameRef::new(0, 0, 0, 0, 0);
        let samples: Vec<IqSample> = (0..12)
            .map(|i| IqSample::new(i * 100, -(i * 50)))
            .collect();
        let msg = UplaneMessage::new_uncompressed(1, 0, frame, DataDirection::Uplink, 0, 0, &samples);
        let bytes = msg.serialize();
        let cfg = OranConfig::new(SubcarrierSpacing::Scs15kHz, 52, 2);
        let mut proc = OranFronthaulProcessor::new(cfg);
        let result = proc.receive_uplane(&bytes);
        assert!(result.is_some());
        let (_hdr, recovered) = result.unwrap();
        assert_eq!(recovered.len(), samples.len());
        for (orig, rec) in samples.iter().zip(recovered.iter()) {
            assert_eq!(orig.i, rec.i);
            assert_eq!(orig.q, rec.q);
        }
    }

    #[test]
    fn test_processor_timing_advance_check() {
        let cfg = OranConfig::new(SubcarrierSpacing::Scs15kHz, 25, 2);
        let proc = OranFronthaulProcessor::new(cfg);
        assert!(proc.check_timing_advance(proc.config.timing.ta3_min_us));
        assert!(!proc.check_timing_advance(u32::MAX));
    }

    #[test]
    fn test_processor_seq_counter_increments() {
        let cfg = OranConfig::new(SubcarrierSpacing::Scs15kHz, 25, 2);
        let mut proc = OranFronthaulProcessor::new(cfg);
        let sections = vec![SectionType1::new(0, 0, 25, 0)];
        let iq = vec![IqSample::zero(); 12];
        proc.schedule_dl_slot(1, sections.clone(), vec![iq.clone()]);
        proc.schedule_dl_slot(1, sections, vec![iq]);
        assert_eq!(proc.seq_counter, 4);
    }

    #[test]
    fn test_section_type_from_u8() {
        assert_eq!(SectionType::from_u8(1), Some(SectionType::MostChannels));
        assert_eq!(SectionType::from_u8(3), Some(SectionType::Prach));
        assert_eq!(SectionType::from_u8(255), None);
    }

    #[test]
    fn test_iq_sample_power_sq() {
        let s = IqSample::new(3, 4);
        assert_eq!(s.power_sq(), 25);
    }
}
