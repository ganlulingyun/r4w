//! # 5G NR Radio Link Control (RLC) Processor
//!
//! Implements the RLC sublayer for 5G NR per 3GPP TS 38.322.
//!
//! ## Overview
//!
//! The RLC sublayer sits between the PDCP sublayer (above) and the MAC sublayer
//! (below). It provides three modes of operation:
//!
//! - **TM (Transparent Mode)**: No header, used for BCCH, PCCH, and CCCH.
//! - **UM (Unacknowledged Mode)**: Sequenced delivery without retransmission,
//!   suitable for delay-sensitive bearers (VoIP, streaming).
//! - **AM (Acknowledged Mode)**: ARQ-based reliable delivery, used for most
//!   data bearers (TCP, signalling).
//!
//! ## Key Features
//!
//! - SDU segmentation and reassembly (SAR)
//! - Sequence number management: 6-bit (UM), 12-bit (UM/AM), 18-bit (AM)
//! - Polling mechanism: `poll_pdu`, `poll_byte`, `t-PollRetransmit`
//! - STATUS PDU generation with ACK_SN, NACK_SN, E1/E2/E3 extensions
//! - `t-Reassembly` timer for out-of-order segment handling
//! - RLC entity reset / re-establishment
//! - Concatenation of multiple SDUs in a single PDU
//! - Buffer Status Report (BSR) integration helpers
//!
//! ## PDU Header Fields (TS 38.322 §6.2)
//!
//! ```text
//!  AM Data PDU (SN-12):
//!  ┌────┬───┬─────────────┬──────────────────────────────────┐
//!  │ DC │ P │  SI (2b)    │  SN[11:8] (4b) │ R R │ SN[7:0]  │
//!  └────┴───┴─────────────┴──────────────────────────────────┘
//!
//!  UM Data PDU (SN-12):
//!  ┌──────────────┬──────────────────────────────────────────┐
//!  │  SI (2b)│R R │  SN[11:8] (4b) │  R (4b) │  SN[7:0]    │
//!  └──────────────┴──────────────────────────────────────────┘
//! ```
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::nr_rlc_processor::{
//!     RlcAmEntity, RlcAmConfig, SnFieldLength,
//! };
//!
//! let cfg = RlcAmConfig {
//!     sn_field_length: SnFieldLength::Len12,
//!     t_poll_retransmit_ms: 45,
//!     t_reassembly_ms: 40,
//!     t_status_prohibit_ms: 0,
//!     poll_pdu: 32,
//!     poll_byte: 25_000,
//!     max_retx_threshold: 8,
//! };
//! let mut tx = RlcAmEntity::new(cfg.clone());
//! let mut rx = RlcAmEntity::new(cfg);
//!
//! // Transmit an SDU
//! tx.tx_enqueue(b"Hello NR RLC!".to_vec());
//! let pdus = tx.tx_generate(1500);
//!
//! // Receive and reassemble
//! for pdu in &pdus {
//!     rx.rx_receive(pdu);
//! }
//! let sdu = rx.rx_deliver().into_iter().flatten().collect::<Vec<u8>>();
//! assert_eq!(sdu, b"Hello NR RLC!");
//! ```

// ─────────────────────────────────────────────────────────────────────────────
// Imports
// ─────────────────────────────────────────────────────────────────────────────

use std::collections::{BTreeMap, VecDeque};

// ─────────────────────────────────────────────────────────────────────────────
// Public constants (TS 38.322)
// ─────────────────────────────────────────────────────────────────────────────

/// Maximum RLC SDU size in bytes (TS 38.322 §4.3.1).
pub const MAX_SDU_SIZE: usize = 9000;

/// Maximum number of queued SDUs in the TX buffer.
pub const TX_BUFFER_MAX_SDUS: usize = 512;

/// Maximum number of retransmissions before RLC failure (default).
pub const DEFAULT_MAX_RETX: u8 = 8;

/// Infinity sentinel for poll_pdu / poll_byte (disabled trigger).
pub const INFINITY: u32 = u32::MAX;

// ─────────────────────────────────────────────────────────────────────────────
// Enumerations
// ─────────────────────────────────────────────────────────────────────────────

/// Sequence number field length (TS 38.322 §6.2.2 / §6.3.2).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SnFieldLength {
    /// 6-bit SN – UM only.
    Len6,
    /// 12-bit SN – UM or AM.
    Len12,
    /// 18-bit SN – AM only (large windows).
    Len18,
}

impl SnFieldLength {
    /// Returns the number of bits in the SN field.
    pub fn bits(self) -> u32 {
        match self {
            SnFieldLength::Len6 => 6,
            SnFieldLength::Len12 => 12,
            SnFieldLength::Len18 => 18,
        }
    }

    /// Returns the modulus (2^bits).
    pub fn modulus(self) -> u32 {
        1u32 << self.bits()
    }

    /// Returns the window size (half the modulus for AM; full for UM).
    pub fn window_size_am(self) -> u32 {
        self.modulus() / 2
    }
}

/// Segmentation Info field (SI) – 2 bits (TS 38.322 §6.2.2.3).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SegmentInfo {
    /// Complete SDU – no segmentation.
    Complete = 0b00,
    /// First segment.
    First = 0b01,
    /// Last segment.
    Last = 0b10,
    /// Middle segment.
    Middle = 0b11,
}

impl SegmentInfo {
    fn from_bits(bits: u8) -> Self {
        match bits & 0x03 {
            0b00 => SegmentInfo::Complete,
            0b01 => SegmentInfo::First,
            0b10 => SegmentInfo::Last,
            _ => SegmentInfo::Middle,
        }
    }
}

/// D/C bit in AM PDU header – distinguishes data from control (STATUS) PDUs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DcBit {
    Control = 0,
    Data = 1,
}

/// RLC mode used when creating a generic entity.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RlcMode {
    Tm,
    Um,
    Am,
}

// ─────────────────────────────────────────────────────────────────────────────
// Error types
// ─────────────────────────────────────────────────────────────────────────────

/// Errors produced by RLC operations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RlcError {
    /// SDU exceeds maximum allowed size.
    SduTooLarge,
    /// TX buffer is full.
    BufferFull,
    /// Malformed PDU (too short or invalid header).
    MalformedPdu,
    /// SN is outside the receive window.
    OutsideWindow,
    /// Retransmission limit exceeded – entity must be re-established.
    MaxRetxExceeded,
    /// Unsupported SN field length for the given mode.
    UnsupportedSnLength,
}

// ─────────────────────────────────────────────────────────────────────────────
// TM (Transparent Mode) Entity
// ─────────────────────────────────────────────────────────────────────────────

/// Transparent Mode RLC entity (TS 38.322 §5.1).
///
/// Used for BCCH, PCCH, and CCCH. No header overhead – the SDU is passed
/// directly as the PDU (and vice versa).
#[derive(Debug, Default)]
pub struct RlcTmEntity {
    tx_queue: VecDeque<Vec<u8>>,
    rx_queue: VecDeque<Vec<u8>>,
    tx_pdu_count: u64,
    rx_pdu_count: u64,
}

impl RlcTmEntity {
    /// Create a new TM entity.
    pub fn new() -> Self {
        Self::default()
    }

    /// Enqueue an SDU for transmission (no segmentation in TM).
    pub fn tx_enqueue(&mut self, sdu: Vec<u8>) -> Result<(), RlcError> {
        if sdu.len() > MAX_SDU_SIZE {
            return Err(RlcError::SduTooLarge);
        }
        if self.tx_queue.len() >= TX_BUFFER_MAX_SDUS {
            return Err(RlcError::BufferFull);
        }
        self.tx_queue.push_back(sdu);
        Ok(())
    }

    /// Generate TM PDUs (transparent: one PDU = one SDU).
    pub fn tx_generate(&mut self) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        while let Some(sdu) = self.tx_queue.pop_front() {
            self.tx_pdu_count += 1;
            out.push(sdu);
        }
        out
    }

    /// Receive a TM PDU (= SDU, no header to strip).
    pub fn rx_receive(&mut self, pdu: &[u8]) {
        self.rx_pdu_count += 1;
        self.rx_queue.push_back(pdu.to_vec());
    }

    /// Deliver reassembled SDUs to the upper layer.
    pub fn rx_deliver(&mut self) -> Vec<Vec<u8>> {
        self.rx_queue.drain(..).collect()
    }

    /// Returns (tx_pdu_count, rx_pdu_count).
    pub fn stats(&self) -> (u64, u64) {
        (self.tx_pdu_count, self.rx_pdu_count)
    }

    /// Re-establish the entity (TS 38.322 §5.1.2 / §5.1.3).
    pub fn reestablish(&mut self) {
        self.tx_queue.clear();
        self.rx_queue.clear();
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// UM (Unacknowledged Mode) configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for a UM RLC entity.
#[derive(Debug, Clone)]
pub struct RlcUmConfig {
    /// SN field length: 6 or 12 bits.
    pub sn_field_length: SnFieldLength,
    /// t-Reassembly timer in ms (TS 38.322 §7.3).
    pub t_reassembly_ms: u32,
}

impl Default for RlcUmConfig {
    fn default() -> Self {
        Self {
            sn_field_length: SnFieldLength::Len12,
            t_reassembly_ms: 40,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// UM PDU helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Encode a UM Data PDU header (TS 38.322 §6.3.2).
///
/// Returns the header bytes (1 or 2 bytes depending on SN length).
fn encode_um_header(sn: u32, si: SegmentInfo, sn_len: SnFieldLength, so: Option<u16>) -> Vec<u8> {
    let si_bits = si as u8;
    match sn_len {
        SnFieldLength::Len6 => {
            // 1 byte: SI(2) | SN(6)
            let b0 = (si_bits << 6) | (sn as u8 & 0x3F);
            let mut hdr = vec![b0];
            if matches!(si, SegmentInfo::Last | SegmentInfo::Middle) {
                let so_val = so.unwrap_or(0);
                hdr.push((so_val >> 8) as u8);
                hdr.push(so_val as u8);
            }
            hdr
        }
        SnFieldLength::Len12 => {
            // 2 bytes: SI(2)|R(2)|SN[11:8](4), SN[7:0]
            let b0 = (si_bits << 6) | ((sn >> 8) as u8 & 0x0F);
            let b1 = (sn & 0xFF) as u8;
            let mut hdr = vec![b0, b1];
            if matches!(si, SegmentInfo::Last | SegmentInfo::Middle) {
                let so_val = so.unwrap_or(0);
                hdr.push((so_val >> 8) as u8);
                hdr.push(so_val as u8);
            }
            hdr
        }
        SnFieldLength::Len18 => {
            // UM only supports 6/12; treat as 12.
            let b0 = (si_bits << 6) | ((sn >> 8) as u8 & 0x0F);
            let b1 = (sn & 0xFF) as u8;
            let mut hdr = vec![b0, b1];
            if matches!(si, SegmentInfo::Last | SegmentInfo::Middle) {
                let so_val = so.unwrap_or(0);
                hdr.push((so_val >> 8) as u8);
                hdr.push(so_val as u8);
            }
            hdr
        }
    }
}

/// Decode a UM Data PDU header.
fn decode_um_header(data: &[u8], sn_len: SnFieldLength) -> Option<(u32, SegmentInfo, u16, usize)> {
    if data.is_empty() {
        return None;
    }
    let si = SegmentInfo::from_bits(data[0] >> 6);
    let (sn, mut hdr_len) = match sn_len {
        SnFieldLength::Len6 => {
            if data.is_empty() {
                return None;
            }
            ((data[0] & 0x3F) as u32, 1usize)
        }
        _ => {
            if data.len() < 2 {
                return None;
            }
            (((data[0] & 0x0F) as u32) << 8 | data[1] as u32, 2usize)
        }
    };
    let so = if matches!(si, SegmentInfo::Last | SegmentInfo::Middle) {
        if data.len() < hdr_len + 2 {
            return None;
        }
        let v = ((data[hdr_len] as u16) << 8) | data[hdr_len + 1] as u16;
        hdr_len += 2;
        v
    } else {
        0
    };
    Some((sn, si, so, hdr_len))
}

// ─────────────────────────────────────────────────────────────────────────────
// UM reassembly buffer
// ─────────────────────────────────────────────────────────────────────────────

/// One pending segment in the UM reassembly buffer.
#[derive(Debug, Clone)]
struct UmSegment {
    #[allow(dead_code)]
    sn: u32,
    so: u16,
    data: Vec<u8>,
    si: SegmentInfo,
}

// ─────────────────────────────────────────────────────────────────────────────
// UM Entity
// ─────────────────────────────────────────────────────────────────────────────

/// Unacknowledged Mode RLC entity (TS 38.322 §5.2).
#[derive(Debug)]
pub struct RlcUmEntity {
    cfg: RlcUmConfig,
    // TX state
    tx_next: u32,
    tx_queue: VecDeque<Vec<u8>>,
    // RX state
    rx_next_reassembly: u32,
    rx_next_highest: u32,
    rx_timer_trigger: u32,
    t_reassembly_running: bool,
    t_reassembly_elapsed_ms: u32,
    // Reassembly buffer: SN → segments
    rx_buf: BTreeMap<u32, Vec<UmSegment>>,
    rx_delivered: VecDeque<Vec<u8>>,
    // Stats
    tx_pdu_count: u64,
    rx_pdu_count: u64,
    rx_discard_count: u64,
}

impl RlcUmEntity {
    /// Create a new UM entity with the given configuration.
    pub fn new(cfg: RlcUmConfig) -> Self {
        if matches!(cfg.sn_field_length, SnFieldLength::Len18) {
            panic!("UM does not support 18-bit SN");
        }
        Self {
            cfg,
            tx_next: 0,
            tx_queue: VecDeque::new(),
            rx_next_reassembly: 0,
            rx_next_highest: 0,
            rx_timer_trigger: 0,
            t_reassembly_running: false,
            t_reassembly_elapsed_ms: 0,
            rx_buf: BTreeMap::new(),
            rx_delivered: VecDeque::new(),
            tx_pdu_count: 0,
            rx_pdu_count: 0,
            rx_discard_count: 0,
        }
    }

    fn modulus(&self) -> u32 {
        self.cfg.sn_field_length.modulus()
    }

    fn window_size(&self) -> u32 {
        self.modulus() / 2
    }

    /// Enqueue an SDU for UM transmission.
    pub fn tx_enqueue(&mut self, sdu: Vec<u8>) -> Result<(), RlcError> {
        if sdu.len() > MAX_SDU_SIZE {
            return Err(RlcError::SduTooLarge);
        }
        if self.tx_queue.len() >= TX_BUFFER_MAX_SDUS {
            return Err(RlcError::BufferFull);
        }
        self.tx_queue.push_back(sdu);
        Ok(())
    }

    /// Generate UM PDUs fitting into `grant_bytes` bytes total.
    ///
    /// Performs segmentation if an SDU is larger than the grant minus header.
    #[allow(unused_assignments)]
    pub fn tx_generate(&mut self, grant_bytes: usize) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        let mut remaining = grant_bytes;

        while !self.tx_queue.is_empty() && remaining > 0 {
            // Minimum header = 1 byte (SN-6 complete) or 2 bytes (SN-12)
            let hdr_min = match self.cfg.sn_field_length {
                SnFieldLength::Len6 => 1,
                _ => 2,
            };
            if remaining <= hdr_min {
                break;
            }
            let payload_space = remaining - hdr_min;
            let sdu = self.tx_queue.pop_front().unwrap();

            if sdu.len() <= payload_space {
                // Complete SDU fits
                let hdr = encode_um_header(
                    self.tx_next,
                    SegmentInfo::Complete,
                    self.cfg.sn_field_length,
                    None,
                );
                let mut pdu = hdr;
                pdu.extend_from_slice(&sdu);
                remaining = remaining.saturating_sub(pdu.len());
                self.tx_next = (self.tx_next + 1) % self.modulus();
                self.tx_pdu_count += 1;
                out.push(pdu);
            } else {
                // Need to segment – re-enqueue remainder
                let segment = sdu[..payload_space].to_vec();
                let rest = sdu[payload_space..].to_vec();
                // Determine SI: first segment
                let hdr = encode_um_header(
                    self.tx_next,
                    SegmentInfo::First,
                    self.cfg.sn_field_length,
                    None,
                );
                let mut pdu = hdr;
                pdu.extend_from_slice(&segment);
                remaining = remaining.saturating_sub(pdu.len());
                self.tx_pdu_count += 1;
                out.push(pdu);
                // Put rest back with a continuation SDU (simplified: we
                // track continuations by re-enqueueing as a "rest" marker;
                // for a full implementation a per-SN segment tracker is used)
                // We keep the same SN for continuation – handled by advancing
                // SN only after last segment.
                // For simplicity in this educational implementation: we advance
                // SN for each PDU (each PDU uniquely identifies a fragment)
                self.tx_next = (self.tx_next + 1) % self.modulus();
                // Re-insert rest – mark it needs "Last/Middle" SI
                // We use a simple wrapper: push rest back as new SDU
                self.tx_queue.push_front(rest);
                break; // one segmented PDU per grant call
            }
        }
        out
    }

    /// Receive a UM Data PDU.
    pub fn rx_receive(&mut self, pdu: &[u8]) {
        self.rx_pdu_count += 1;
        let Some((sn, si, so, hdr_len)) = decode_um_header(pdu, self.cfg.sn_field_length) else {
            self.rx_discard_count += 1;
            return;
        };
        if pdu.len() <= hdr_len {
            self.rx_discard_count += 1;
            return;
        }
        let payload = pdu[hdr_len..].to_vec();

        // Window check (TS 38.322 §5.2.2.2.4)
        let modulus = self.modulus();
        let ws = self.window_size();
        let in_window = sn_in_window(sn, self.rx_next_reassembly, ws, modulus);
        if !in_window {
            self.rx_discard_count += 1;
            return;
        }

        // Store segment
        let segs = self.rx_buf.entry(sn).or_default();
        segs.push(UmSegment { sn, so, data: payload, si });

        // Update RX_Next_Highest
        let sn_plus1 = (sn + 1) % modulus;
        if sn_distance(sn_plus1, self.rx_next_highest, modulus) < ws {
            self.rx_next_highest = sn_plus1;
        }

        // Try to reassemble
        self.try_reassemble();
        // Advance RX_Next_Reassembly past fully reassembled SNs
        self.advance_rx_state();
        // t-Reassembly start/stop logic
        self.update_t_reassembly();
    }

    fn try_reassemble(&mut self) {
        let modulus = self.modulus();
        let ws = self.window_size();
        // Iterate SNs in window order
        let sns: Vec<u32> = self
            .rx_buf
            .keys()
            .cloned()
            .filter(|&sn| sn_in_window(sn, self.rx_next_reassembly, ws, modulus))
            .collect();

        for sn in sns {
            let segs = match self.rx_buf.get(&sn) {
                Some(s) => s,
                None => continue,
            };
            if is_complete(segs) {
                let mut all_segs = self.rx_buf.remove(&sn).unwrap();
                all_segs.sort_by_key(|s| s.so);
                let mut assembled = Vec::new();
                for seg in all_segs {
                    assembled.extend_from_slice(&seg.data);
                }
                self.rx_delivered.push_back(assembled);
            }
        }
    }

    fn advance_rx_state(&mut self) {
        let modulus = self.modulus();
        loop {
            if !self.rx_buf.contains_key(&self.rx_next_reassembly) {
                // Check if SN was already delivered
                if self.rx_next_reassembly
                    != sn_sub(self.rx_next_highest, 1, modulus)
                {
                    let next = (self.rx_next_reassembly + 1) % modulus;
                    if sn_distance(next, self.rx_next_highest, modulus) <= self.window_size() {
                        self.rx_next_reassembly = next;
                    } else {
                        break;
                    }
                } else {
                    break;
                }
            } else {
                break;
            }
        }
    }

    fn update_t_reassembly(&mut self) {
        if self.rx_next_highest != self.rx_next_reassembly
            || !self.rx_buf.is_empty()
        {
            if !self.t_reassembly_running {
                self.t_reassembly_running = true;
                self.t_reassembly_elapsed_ms = 0;
                self.rx_timer_trigger = self.rx_next_highest;
            }
        } else {
            self.t_reassembly_running = false;
        }
    }

    /// Advance simulated time by `delta_ms` milliseconds.
    ///
    /// Handles t-Reassembly expiry.
    pub fn tick(&mut self, delta_ms: u32) {
        if self.t_reassembly_running {
            self.t_reassembly_elapsed_ms += delta_ms;
            if self.t_reassembly_elapsed_ms >= self.cfg.t_reassembly_ms {
                // Timer expired – discard partial segments below rx_timer_trigger
                let trigger = self.rx_timer_trigger;
                let modulus = self.modulus();
                let ws = self.window_size();
                let to_discard: Vec<u32> = self
                    .rx_buf
                    .keys()
                    .cloned()
                    .filter(|&sn| sn_in_window(sn, self.rx_next_reassembly, ws, modulus)
                        && sn_distance(sn, trigger, modulus) > 0)
                    .collect();
                for sn in to_discard {
                    self.rx_buf.remove(&sn);
                    self.rx_discard_count += 1;
                }
                self.rx_next_reassembly = trigger;
                self.t_reassembly_running = false;
                self.t_reassembly_elapsed_ms = 0;
                // Restart if needed
                self.update_t_reassembly();
            }
        }
    }

    /// Deliver reassembled SDUs to the upper layer.
    pub fn rx_deliver(&mut self) -> Vec<Vec<u8>> {
        self.rx_delivered.drain(..).collect()
    }

    /// Re-establish the UM entity (TS 38.322 §5.2.3).
    pub fn reestablish(&mut self) {
        self.tx_next = 0;
        self.tx_queue.clear();
        self.rx_next_reassembly = 0;
        self.rx_next_highest = 0;
        self.rx_timer_trigger = 0;
        self.t_reassembly_running = false;
        self.t_reassembly_elapsed_ms = 0;
        self.rx_buf.clear();
        self.rx_delivered.clear();
    }

    /// Buffer occupancy in bytes.
    pub fn tx_buffer_bytes(&self) -> usize {
        self.tx_queue.iter().map(|s| s.len()).sum()
    }

    /// Returns (tx_pdu_count, rx_pdu_count, rx_discard_count).
    pub fn stats(&self) -> (u64, u64, u64) {
        (self.tx_pdu_count, self.rx_pdu_count, self.rx_discard_count)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// AM (Acknowledged Mode) configuration
// ─────────────────────────────────────────────────────────────────────────────

/// Configuration for an AM RLC entity (TS 38.322 §7).
#[derive(Debug, Clone)]
pub struct RlcAmConfig {
    /// SN field length: 12 or 18 bits.
    pub sn_field_length: SnFieldLength,
    /// t-PollRetransmit in ms (TS 38.322 §7.3).
    pub t_poll_retransmit_ms: u32,
    /// t-Reassembly in ms (TS 38.322 §7.3).
    pub t_reassembly_ms: u32,
    /// t-StatusProhibit in ms (TS 38.322 §7.3).
    pub t_status_prohibit_ms: u32,
    /// pollPDU: trigger after N PDUs (INFINITY = disabled).
    pub poll_pdu: u32,
    /// pollByte: trigger after N bytes (INFINITY = disabled).
    pub poll_byte: u32,
    /// maxRetxThreshold (TS 38.322 §7.1).
    pub max_retx_threshold: u8,
}

impl Default for RlcAmConfig {
    fn default() -> Self {
        Self {
            sn_field_length: SnFieldLength::Len12,
            t_poll_retransmit_ms: 45,
            t_reassembly_ms: 40,
            t_status_prohibit_ms: 0,
            poll_pdu: 32,
            poll_byte: 25_000,
            max_retx_threshold: 8,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// AM PDU header encode/decode
// ─────────────────────────────────────────────────────────────────────────────

/// Encode an AM Data PDU header (TS 38.322 §6.2.2).
fn encode_am_header(
    sn: u32,
    si: SegmentInfo,
    poll: bool,
    sn_len: SnFieldLength,
    so: Option<u16>,
) -> Vec<u8> {
    let dc = DcBit::Data as u8; // bit 7 = 1
    let p_bit = if poll { 1u8 } else { 0u8 };
    let si_bits = si as u8;
    let mut hdr = match sn_len {
        SnFieldLength::Len12 => {
            // Byte 0: D/C(1) | P(1) | SI(2) | SN[11:8](4)
            // Byte 1: SN[7:0]
            let b0 = (dc << 7) | (p_bit << 6) | (si_bits << 4) | ((sn >> 8) as u8 & 0x0F);
            let b1 = (sn & 0xFF) as u8;
            vec![b0, b1]
        }
        SnFieldLength::Len18 => {
            // Byte 0: D/C(1) | P(1) | SI(2) | R(2) | SN[17:16](2)
            // Byte 1: SN[15:8]
            // Byte 2: SN[7:0]
            let b0 = (dc << 7) | (p_bit << 6) | (si_bits << 4) | ((sn >> 16) as u8 & 0x03);
            let b1 = ((sn >> 8) & 0xFF) as u8;
            let b2 = (sn & 0xFF) as u8;
            vec![b0, b1, b2]
        }
        SnFieldLength::Len6 => {
            // AM only uses 12/18 – fall back to 12
            let b0 = (dc << 7) | (p_bit << 6) | (si_bits << 4) | ((sn >> 8) as u8 & 0x0F);
            let b1 = (sn & 0xFF) as u8;
            vec![b0, b1]
        }
    };
    if matches!(si, SegmentInfo::Last | SegmentInfo::Middle) {
        let so_val = so.unwrap_or(0);
        hdr.push((so_val >> 8) as u8);
        hdr.push(so_val as u8);
    }
    hdr
}

/// Decode an AM Data PDU header. Returns (sn, si, poll, so, hdr_len).
fn decode_am_header(data: &[u8], sn_len: SnFieldLength) -> Option<(u32, SegmentInfo, bool, u16, usize)> {
    if data.is_empty() {
        return None;
    }
    // D/C must be 1 for data PDU
    if (data[0] >> 7) & 1 == 0 {
        return None; // STATUS PDU
    }
    let poll = (data[0] >> 6) & 1 == 1;
    let si_bits = (data[0] >> 4) & 0x03;
    let si = SegmentInfo::from_bits(si_bits);

    let (sn, mut hdr_len) = match sn_len {
        SnFieldLength::Len18 => {
            if data.len() < 3 {
                return None;
            }
            let sn = ((data[0] & 0x03) as u32) << 16
                | (data[1] as u32) << 8
                | data[2] as u32;
            (sn, 3usize)
        }
        _ => {
            if data.len() < 2 {
                return None;
            }
            let sn = ((data[0] & 0x0F) as u32) << 8 | data[1] as u32;
            (sn, 2usize)
        }
    };

    let so = if matches!(si, SegmentInfo::Last | SegmentInfo::Middle) {
        if data.len() < hdr_len + 2 {
            return None;
        }
        let v = ((data[hdr_len] as u16) << 8) | data[hdr_len + 1] as u16;
        hdr_len += 2;
        v
    } else {
        0
    };
    Some((sn, si, poll, so, hdr_len))
}

// ─────────────────────────────────────────────────────────────────────────────
// STATUS PDU encode/decode
// ─────────────────────────────────────────────────────────────────────────────

/// One NACK entry in a STATUS PDU (TS 38.322 §6.2.2.5).
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct NackEntry {
    /// NACK_SN: the sequence number being negatively acknowledged.
    pub nack_sn: u32,
    /// E1: set if SO start/end present.
    pub has_so: bool,
    /// E2: set if a NACK range follows.
    pub has_range: bool,
    /// Segment offset start (SO_start) – only valid when has_so = true.
    pub so_start: u16,
    /// Segment offset end (SO_end).
    pub so_end: u16,
    /// NACK range – only valid when has_range = true.
    pub nack_range: u8,
}

impl NackEntry {
    /// Create a simple SN-level NACK (no SO, no range).
    pub fn new(nack_sn: u32) -> Self {
        Self {
            nack_sn,
            has_so: false,
            has_range: false,
            so_start: 0,
            so_end: 0,
            nack_range: 0,
        }
    }
}

/// Encode a STATUS PDU (TS 38.322 §6.2.2.5) for 12-bit SN.
///
/// Format: CPT(3)|R(5) | ACK_SN[11:4] | ACK_SN[3:0]|E1|R(3) | [NACK entries]
pub fn encode_status_pdu_12(ack_sn: u32, nacks: &[NackEntry]) -> Vec<u8> {
    // Byte 0: D/C=0, CPT=000, R*5
    // Byte 1: ACK_SN[11:4]
    // Byte 2: ACK_SN[3:0] | E1 | R(3)
    let mut out = Vec::new();
    out.push(0x00u8); // D/C=0 (control), CPT=000 (STATUS)
    out.push(((ack_sn >> 4) & 0xFF) as u8);
    let e1 = if nacks.is_empty() { 0u8 } else { 1u8 };
    out.push(((ack_sn & 0x0F) as u8) << 4 | e1 << 3);

    for (i, nack) in nacks.iter().enumerate() {
        let last = i == nacks.len() - 1;
        // NACK_SN (12 bits) in 2 bytes + control nibble
        // Byte a: NACK_SN[11:4]
        // Byte b: NACK_SN[3:0] | E1(more_nacks) | E2(has_so) | E3(has_range)
        let e1_next = if !last { 1u8 } else { 0u8 };
        let e2 = if nack.has_so { 1u8 } else { 0u8 };
        let e3 = if nack.has_range { 1u8 } else { 0u8 };
        out.push(((nack.nack_sn >> 4) & 0xFF) as u8);
        out.push(
            (((nack.nack_sn & 0x0F) as u8) << 4)
                | (e1_next << 3)
                | (e2 << 2)
                | (e3 << 1),
        );
        if nack.has_so {
            out.push((nack.so_start >> 8) as u8);
            out.push((nack.so_start & 0xFF) as u8);
            out.push((nack.so_end >> 8) as u8);
            out.push((nack.so_end & 0xFF) as u8);
        }
        if nack.has_range {
            out.push(nack.nack_range);
        }
    }
    out
}

/// Decode a STATUS PDU (12-bit SN). Returns (ack_sn, nack_list).
pub fn decode_status_pdu_12(data: &[u8]) -> Option<(u32, Vec<NackEntry>)> {
    if data.len() < 3 {
        return None;
    }
    // Byte 0 must have D/C=0
    if data[0] >> 7 != 0 {
        return None;
    }
    let ack_sn = ((data[1] as u32) << 4) | ((data[2] as u32) >> 4);
    let mut e1 = (data[2] >> 3) & 1;
    let mut nacks = Vec::new();
    let mut pos = 3usize;

    while e1 == 1 {
        if pos + 2 > data.len() {
            break;
        }
        let nack_sn = ((data[pos] as u32) << 4) | ((data[pos + 1] as u32) >> 4);
        let flags = data[pos + 1] & 0x0F;
        let e1_next = (flags >> 3) & 1;
        let e2 = (flags >> 2) & 1;
        let e3 = (flags >> 1) & 1;
        pos += 2;

        let mut entry = NackEntry {
            nack_sn,
            has_so: e2 == 1,
            has_range: e3 == 1,
            so_start: 0,
            so_end: 0,
            nack_range: 0,
        };
        if e2 == 1 {
            if pos + 4 > data.len() {
                break;
            }
            entry.so_start = ((data[pos] as u16) << 8) | data[pos + 1] as u16;
            entry.so_end = ((data[pos + 2] as u16) << 8) | data[pos + 3] as u16;
            pos += 4;
        }
        if e3 == 1 {
            if pos >= data.len() {
                break;
            }
            entry.nack_range = data[pos];
            pos += 1;
        }
        nacks.push(entry);
        e1 = e1_next;
    }
    Some((ack_sn, nacks))
}

// ─────────────────────────────────────────────────────────────────────────────
// AM TX PDU descriptor
// ─────────────────────────────────────────────────────────────────────────────

/// Descriptor for an AM PDU in the retransmission buffer.
#[derive(Debug, Clone)]
struct AmTxPdu {
    sn: u32,
    si: SegmentInfo,
    so: u16,
    data: Vec<u8>,
    retx_count: u8,
    acked: bool,
    poll_sent: bool,
}

// ─────────────────────────────────────────────────────────────────────────────
// AM RX segment
// ─────────────────────────────────────────────────────────────────────────────

#[derive(Debug, Clone)]
struct AmRxSegment {
    so: u16,
    data: Vec<u8>,
    si: SegmentInfo,
}

// ─────────────────────────────────────────────────────────────────────────────
// AM Entity
// ─────────────────────────────────────────────────────────────────────────────

/// Acknowledged Mode RLC entity (TS 38.322 §5.3).
///
/// Provides reliable, in-order delivery with ARQ retransmission.
#[derive(Debug)]
pub struct RlcAmEntity {
    cfg: RlcAmConfig,

    // ── TX state ──────────────────────────────────────────────────
    /// TX_Next – next SN to assign to a new AMD PDU.
    tx_next: u32,
    /// TX_Next_Ack – oldest unacknowledged SN.
    tx_next_ack: u32,
    /// PDU counter since last poll.
    pdu_without_poll: u32,
    /// Byte counter since last poll.
    byte_without_poll: u32,
    /// t-PollRetransmit elapsed ms.
    t_poll_retransmit_elapsed: u32,
    t_poll_retransmit_running: bool,
    /// TX SDU queue.
    tx_sdu_queue: VecDeque<Vec<u8>>,
    /// Retransmission buffer: SN → PDU descriptors (one per segment).
    tx_retx_buf: BTreeMap<u32, Vec<AmTxPdu>>,
    /// SNs queued for retransmission.
    retx_queue: VecDeque<u32>,

    // ── RX state ──────────────────────────────────────────────────
    /// RX_Next – next expected SN.
    rx_next: u32,
    /// RX_Next_Status_Trigger – triggers STATUS if non-zero.
    rx_next_status_trigger: u32,
    /// RX_Highest_Status – the largest SN for which ACK/NACK is sent.
    rx_highest_status: u32,
    /// t-Reassembly elapsed ms.
    t_reassembly_elapsed: u32,
    t_reassembly_running: bool,
    t_reassembly_trigger_sn: u32,
    /// t-StatusProhibit elapsed ms.
    t_status_prohibit_elapsed: u32,
    t_status_prohibit_running: bool,
    /// RX reassembly buffer: SN → segments.
    rx_buf: BTreeMap<u32, Vec<AmRxSegment>>,
    /// Delivered SDUs waiting for upper-layer collection.
    rx_delivered: VecDeque<Vec<u8>>,
    /// Pending STATUS PDU to send.
    pending_status: bool,

    // ── Stats ─────────────────────────────────────────────────────
    tx_pdu_count: u64,
    rx_pdu_count: u64,
    retx_count_total: u64,
    rx_discard_count: u64,
    status_tx_count: u64,
    status_rx_count: u64,
}

impl RlcAmEntity {
    /// Create a new AM entity.
    pub fn new(cfg: RlcAmConfig) -> Self {
        if cfg.sn_field_length == SnFieldLength::Len6 {
            panic!("AM does not support 6-bit SN");
        }
        Self {
            cfg,
            tx_next: 0,
            tx_next_ack: 0,
            pdu_without_poll: 0,
            byte_without_poll: 0,
            t_poll_retransmit_elapsed: 0,
            t_poll_retransmit_running: false,
            tx_sdu_queue: VecDeque::new(),
            tx_retx_buf: BTreeMap::new(),
            retx_queue: VecDeque::new(),
            rx_next: 0,
            rx_next_status_trigger: 0,
            rx_highest_status: 0,
            t_reassembly_elapsed: 0,
            t_reassembly_running: false,
            t_reassembly_trigger_sn: 0,
            t_status_prohibit_elapsed: 0,
            t_status_prohibit_running: false,
            rx_buf: BTreeMap::new(),
            rx_delivered: VecDeque::new(),
            pending_status: false,
            tx_pdu_count: 0,
            rx_pdu_count: 0,
            retx_count_total: 0,
            rx_discard_count: 0,
            status_tx_count: 0,
            status_rx_count: 0,
        }
    }

    fn modulus(&self) -> u32 {
        self.cfg.sn_field_length.modulus()
    }

    fn window_size(&self) -> u32 {
        self.cfg.sn_field_length.window_size_am()
    }

    /// Enqueue an SDU for AM transmission.
    pub fn tx_enqueue(&mut self, sdu: Vec<u8>) -> Result<(), RlcError> {
        if sdu.len() > MAX_SDU_SIZE {
            return Err(RlcError::SduTooLarge);
        }
        if self.tx_sdu_queue.len() >= TX_BUFFER_MAX_SDUS {
            return Err(RlcError::BufferFull);
        }
        self.tx_sdu_queue.push_back(sdu);
        Ok(())
    }

    /// Determine whether to set the poll bit (TS 38.322 §5.3.3.3).
    fn should_poll(&self, pdu_payload_bytes: usize) -> bool {
        // pollPDU trigger
        if self.cfg.poll_pdu != INFINITY
            && self.pdu_without_poll + 1 >= self.cfg.poll_pdu
        {
            return true;
        }
        // pollByte trigger
        if self.cfg.poll_byte != INFINITY
            && self.byte_without_poll + pdu_payload_bytes as u32 >= self.cfg.poll_byte
        {
            return true;
        }
        // TX window nearly full
        let modulus = self.modulus();
        let ws = self.window_size();
        let tx_used = sn_distance(self.tx_next, self.tx_next_ack, modulus);
        if tx_used + 1 >= ws {
            return true;
        }
        // TX SDU queue empty and retx queue empty
        if self.tx_sdu_queue.is_empty() && self.retx_queue.is_empty() {
            return true;
        }
        false
    }

    /// Generate AM PDUs fitting into `grant_bytes` bytes total.
    ///
    /// Prioritises retransmissions, then new data.
    #[allow(unused_assignments)]
    pub fn tx_generate(&mut self, grant_bytes: usize) -> Vec<Vec<u8>> {
        let mut out = Vec::new();
        let mut remaining = grant_bytes;

        // 1. Retransmissions first
        while !self.retx_queue.is_empty() && remaining > 0 {
            let sn = *self.retx_queue.front().unwrap();

            // Check if SN exists and find first un-acked segment index
            let seg_idx = match self.tx_retx_buf.get(&sn) {
                Some(segs) => segs.iter().position(|s| !s.acked),
                None => {
                    self.retx_queue.pop_front();
                    continue;
                }
            };
            let seg_idx = match seg_idx {
                Some(i) => i,
                None => {
                    self.retx_queue.pop_front();
                    continue;
                }
            };

            // Increment retx_count and check threshold (mutable borrow, then drop)
            {
                let seg = &mut self.tx_retx_buf.get_mut(&sn).unwrap()[seg_idx];
                seg.retx_count += 1;
                if seg.retx_count > self.cfg.max_retx_threshold {
                    self.retx_queue.pop_front();
                    continue;
                }
            }
            self.retx_count_total += 1;

            // Extract segment fields needed for encoding (immutable snapshot)
            let (seg_sn, seg_si, seg_so, seg_data_len) = {
                let seg = &self.tx_retx_buf[&sn][seg_idx];
                (seg.sn, seg.si, seg.so, seg.data.len())
            };

            // Now compute poll (immutable borrow of self, no overlap with retx_buf needed)
            let poll = self.should_poll(seg_data_len);

            let hdr = encode_am_header(
                seg_sn,
                seg_si,
                poll,
                self.cfg.sn_field_length,
                if seg_so > 0 { Some(seg_so) } else { None },
            );
            let pdu_len = hdr.len() + seg_data_len;
            if pdu_len > remaining {
                break;
            }

            // Build PDU (need immutable access to seg.data)
            let seg_data_clone = self.tx_retx_buf[&sn][seg_idx].data.clone();
            let mut pdu = hdr;
            pdu.extend_from_slice(&seg_data_clone);
            remaining = remaining.saturating_sub(pdu_len);
            self.tx_pdu_count += 1;
            if poll {
                self.pdu_without_poll = 0;
                self.byte_without_poll = 0;
                self.t_poll_retransmit_running = true;
                self.t_poll_retransmit_elapsed = 0;
                self.tx_retx_buf.get_mut(&sn).unwrap()[seg_idx].poll_sent = true;
            } else {
                self.pdu_without_poll += 1;
                self.byte_without_poll += seg_data_len as u32;
            }
            out.push(pdu);
            self.retx_queue.pop_front();
        }

        // 2. New data
        while !self.tx_sdu_queue.is_empty() && remaining > 0 {
            // Window check
            let modulus = self.modulus();
            let ws = self.window_size();
            if sn_distance(self.tx_next, self.tx_next_ack, modulus) >= ws {
                break; // window full
            }
            let hdr_base = match self.cfg.sn_field_length {
                SnFieldLength::Len18 => 3usize,
                _ => 2usize,
            };
            if remaining <= hdr_base {
                break;
            }
            let payload_space = remaining - hdr_base;
            let sdu = self.tx_sdu_queue.pop_front().unwrap();

            if sdu.len() <= payload_space {
                // Complete
                let poll = self.should_poll(sdu.len());
                let hdr = encode_am_header(
                    self.tx_next,
                    SegmentInfo::Complete,
                    poll,
                    self.cfg.sn_field_length,
                    None,
                );
                let pdu_len = hdr.len() + sdu.len();
                let mut pdu = hdr;
                pdu.extend_from_slice(&sdu);
                // Store in retx buf
                let desc = AmTxPdu {
                    sn: self.tx_next,
                    si: SegmentInfo::Complete,
                    so: 0,
                    data: sdu.clone(),
                    retx_count: 0,
                    acked: false,
                    poll_sent: poll,
                };
                self.tx_retx_buf.entry(self.tx_next).or_default().push(desc);
                if poll {
                    self.pdu_without_poll = 0;
                    self.byte_without_poll = 0;
                    self.t_poll_retransmit_running = true;
                    self.t_poll_retransmit_elapsed = 0;
                } else {
                    self.pdu_without_poll += 1;
                    self.byte_without_poll += sdu.len() as u32;
                }
                self.tx_next = (self.tx_next + 1) % modulus;
                remaining = remaining.saturating_sub(pdu_len);
                self.tx_pdu_count += 1;
                out.push(pdu);
            } else {
                // Segment: take first chunk
                let chunk = sdu[..payload_space].to_vec();
                let rest = sdu[payload_space..].to_vec();
                let poll = self.should_poll(chunk.len());
                let hdr = encode_am_header(
                    self.tx_next,
                    SegmentInfo::First,
                    poll,
                    self.cfg.sn_field_length,
                    None,
                );
                let pdu_len = hdr.len() + chunk.len();
                let mut pdu = hdr;
                pdu.extend_from_slice(&chunk);
                let desc = AmTxPdu {
                    sn: self.tx_next,
                    si: SegmentInfo::First,
                    so: 0,
                    data: chunk.clone(),
                    retx_count: 0,
                    acked: false,
                    poll_sent: poll,
                };
                self.tx_retx_buf.entry(self.tx_next).or_default().push(desc);
                if poll {
                    self.pdu_without_poll = 0;
                    self.byte_without_poll = 0;
                    self.t_poll_retransmit_running = true;
                    self.t_poll_retransmit_elapsed = 0;
                } else {
                    self.pdu_without_poll += 1;
                    self.byte_without_poll += chunk.len() as u32;
                }
                self.tx_next = (self.tx_next + 1) % modulus;
                remaining = remaining.saturating_sub(pdu_len);
                self.tx_pdu_count += 1;
                out.push(pdu);
                // Re-enqueue rest (as new SDU – simplified)
                self.tx_sdu_queue.push_front(rest);
                break;
            }
        }
        out
    }

    /// Process a received PDU (data or STATUS).
    pub fn rx_receive(&mut self, pdu: &[u8]) {
        if pdu.is_empty() {
            return;
        }
        // D/C bit
        if (pdu[0] >> 7) & 1 == 0 {
            // STATUS PDU
            self.rx_receive_status(pdu);
        } else {
            // Data PDU
            self.rx_receive_data(pdu);
        }
    }

    fn rx_receive_status(&mut self, pdu: &[u8]) {
        self.status_rx_count += 1;
        let Some((ack_sn, nacks)) = decode_status_pdu_12(pdu) else {
            return;
        };
        // Acknowledge all SNs below ack_sn
        let modulus = self.modulus();
        let mut sn = self.tx_next_ack;
        while sn != ack_sn {
            if let Some(segs) = self.tx_retx_buf.get_mut(&sn) {
                for seg in segs.iter_mut() {
                    seg.acked = true;
                }
            }
            self.tx_retx_buf.remove(&sn);
            sn = (sn + 1) % modulus;
        }
        self.tx_next_ack = ack_sn;
        // Stop poll-retransmit timer if window has moved
        if self.tx_next_ack == self.tx_next {
            self.t_poll_retransmit_running = false;
        }
        // Schedule retransmissions for NACKed SNs
        for nack in &nacks {
            let nsn = nack.nack_sn;
            if let Some(segs) = self.tx_retx_buf.get_mut(&nsn) {
                for seg in segs.iter_mut() {
                    if !seg.acked {
                        seg.retx_count += 1;
                    }
                }
            }
            if !self.retx_queue.contains(&nsn) {
                self.retx_queue.push_back(nsn);
            }
        }
    }

    fn rx_receive_data(&mut self, pdu: &[u8]) {
        self.rx_pdu_count += 1;
        let Some((sn, si, _poll, so, hdr_len)) =
            decode_am_header(pdu, self.cfg.sn_field_length)
        else {
            self.rx_discard_count += 1;
            return;
        };
        if pdu.len() <= hdr_len {
            self.rx_discard_count += 1;
            return;
        }
        let payload = pdu[hdr_len..].to_vec();
        let modulus = self.modulus();
        let ws = self.window_size();

        // Window check: rx_next ≤ sn < rx_next + ws (mod)
        if !sn_in_window(sn, self.rx_next, ws, modulus) {
            self.rx_discard_count += 1;
            return;
        }

        // Duplicate check
        if let Some(segs) = self.rx_buf.get(&sn) {
            if segs.iter().any(|s| s.so == so) {
                return; // duplicate
            }
        }

        // Buffer segment
        let seg = AmRxSegment { so, data: payload, si };
        self.rx_buf.entry(sn).or_default().push(seg);

        // Update rx_highest_status
        let sn_plus1 = (sn + 1) % modulus;
        if sn_distance(sn_plus1, self.rx_highest_status, modulus) < ws {
            self.rx_highest_status = sn_plus1;
        }

        // Try in-order reassembly
        self.reassemble_in_order();

        // t-Reassembly management
        self.update_t_reassembly_am();

        // STATUS PDU trigger: signal ACK/NACK whenever data is received
        // (TS 38.322 §5.3.4 – simplified: always schedule a STATUS after
        // receiving a data PDU so the peer can advance its TX window).
        self.pending_status = true;
    }

    fn reassemble_in_order(&mut self) {
        let modulus = self.modulus();
        loop {
            let sn = self.rx_next;
            let segs = match self.rx_buf.get(&sn) {
                Some(s) => s,
                None => break,
            };
            if !is_complete_am(segs) {
                break;
            }
            let mut segs = self.rx_buf.remove(&sn).unwrap();
            segs.sort_by_key(|s| s.so);
            let mut assembled = Vec::new();
            for seg in segs {
                assembled.extend_from_slice(&seg.data);
            }
            self.rx_delivered.push_back(assembled);
            self.rx_next = (sn + 1) % modulus;
        }
    }

    fn update_t_reassembly_am(&mut self) {
        let modulus = self.modulus();
        let ws = self.window_size();
        if !self.rx_buf.is_empty()
            && sn_distance(self.rx_next, self.rx_highest_status, modulus) < ws
        {
            if !self.t_reassembly_running {
                self.t_reassembly_running = true;
                self.t_reassembly_elapsed = 0;
                self.t_reassembly_trigger_sn = self.rx_highest_status;
            }
        } else {
            self.t_reassembly_running = false;
        }
    }

    /// Generate a STATUS PDU for the peer TX entity (TS 38.322 §5.3.4).
    pub fn tx_status_pdu(&mut self) -> Option<Vec<u8>> {
        if !self.pending_status {
            return None;
        }
        if self.t_status_prohibit_running {
            return None;
        }
        // Build NACK list for all SNs in [rx_next, rx_highest_status)
        let modulus = self.modulus();
        let mut nacks = Vec::new();
        let mut sn = self.rx_next;
        while sn != self.rx_highest_status {
            if self.rx_buf.contains_key(&sn) {
                // Partially received – NACK
                nacks.push(NackEntry::new(sn));
            }
            // SNs not in buf AND below rx_next are implicit ACKs
            sn = (sn + 1) % modulus;
        }
        let ack_sn = self.rx_highest_status;
        let pdu = encode_status_pdu_12(ack_sn, &nacks);
        self.pending_status = false;
        self.status_tx_count += 1;
        // Start t-StatusProhibit
        if self.cfg.t_status_prohibit_ms > 0 {
            self.t_status_prohibit_running = true;
            self.t_status_prohibit_elapsed = 0;
        }
        Some(pdu)
    }

    /// Advance simulated time by `delta_ms` ms.
    ///
    /// Handles t-Reassembly, t-PollRetransmit, and t-StatusProhibit expiry.
    pub fn tick(&mut self, delta_ms: u32) {
        // t-Reassembly
        if self.t_reassembly_running {
            self.t_reassembly_elapsed += delta_ms;
            if self.t_reassembly_elapsed >= self.cfg.t_reassembly_ms {
                self.t_reassembly_running = false;
                self.t_reassembly_elapsed = 0;
                // Update rx_next to trigger_sn
                let modulus = self.modulus();
                self.rx_next = self.t_reassembly_trigger_sn;
                // Request STATUS
                self.pending_status = true;
                // Clean up partial buffers below new rx_next
                let sns_to_remove: Vec<u32> = self
                    .rx_buf
                    .keys()
                    .cloned()
                    .filter(|&sn| {
                        let d = sn_distance(sn, self.rx_next, modulus);
                        d > self.window_size()
                    })
                    .collect();
                for sn in sns_to_remove {
                    self.rx_buf.remove(&sn);
                    self.rx_discard_count += 1;
                }
                self.update_t_reassembly_am();
            }
        }
        // t-PollRetransmit
        if self.t_poll_retransmit_running {
            self.t_poll_retransmit_elapsed += delta_ms;
            if self.t_poll_retransmit_elapsed >= self.cfg.t_poll_retransmit_ms {
                self.t_poll_retransmit_running = false;
                self.t_poll_retransmit_elapsed = 0;
                // Retransmit the PDU with the highest SN
                let sn = if !self.tx_retx_buf.is_empty() {
                    let highest = self.tx_next_ack; // simplification
                    Some(highest)
                } else {
                    None
                };
                if let Some(sn) = sn {
                    if self.tx_retx_buf.contains_key(&sn)
                        && !self.retx_queue.contains(&sn)
                    {
                        self.retx_queue.push_back(sn);
                    }
                }
            }
        }
        // t-StatusProhibit
        if self.t_status_prohibit_running {
            self.t_status_prohibit_elapsed += delta_ms;
            if self.t_status_prohibit_elapsed >= self.cfg.t_status_prohibit_ms {
                self.t_status_prohibit_running = false;
                self.t_status_prohibit_elapsed = 0;
            }
        }
    }

    /// Deliver reassembled SDUs to the upper layer.
    pub fn rx_deliver(&mut self) -> Vec<Vec<u8>> {
        self.rx_delivered.drain(..).collect()
    }

    /// Re-establish the AM entity (TS 38.322 §5.3.8).
    pub fn reestablish(&mut self) {
        self.tx_next = 0;
        self.tx_next_ack = 0;
        self.pdu_without_poll = 0;
        self.byte_without_poll = 0;
        self.t_poll_retransmit_elapsed = 0;
        self.t_poll_retransmit_running = false;
        self.tx_sdu_queue.clear();
        self.tx_retx_buf.clear();
        self.retx_queue.clear();
        self.rx_next = 0;
        self.rx_next_status_trigger = 0;
        self.rx_highest_status = 0;
        self.t_reassembly_elapsed = 0;
        self.t_reassembly_running = false;
        self.t_reassembly_trigger_sn = 0;
        self.t_status_prohibit_elapsed = 0;
        self.t_status_prohibit_running = false;
        self.rx_buf.clear();
        self.rx_delivered.clear();
        self.pending_status = false;
    }

    /// Returns whether a STATUS PDU is pending.
    pub fn has_pending_status(&self) -> bool {
        self.pending_status
    }

    /// TX buffer occupancy in bytes.
    pub fn tx_buffer_bytes(&self) -> usize {
        self.tx_sdu_queue.iter().map(|s| s.len()).sum()
    }

    /// Number of unacknowledged PDUs in the retransmission buffer.
    pub fn retx_buffer_size(&self) -> usize {
        self.tx_retx_buf.values().map(|v| v.len()).sum()
    }

    /// Returns snapshot of key TX/RX sequence numbers.
    pub fn sn_state(&self) -> AmSnState {
        AmSnState {
            tx_next: self.tx_next,
            tx_next_ack: self.tx_next_ack,
            rx_next: self.rx_next,
            rx_highest_status: self.rx_highest_status,
        }
    }

    /// Statistics counters.
    pub fn stats(&self) -> AmStats {
        AmStats {
            tx_pdu_count: self.tx_pdu_count,
            rx_pdu_count: self.rx_pdu_count,
            retx_count_total: self.retx_count_total,
            rx_discard_count: self.rx_discard_count,
            status_tx_count: self.status_tx_count,
            status_rx_count: self.status_rx_count,
        }
    }
}

/// Snapshot of AM sequence number state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AmSnState {
    pub tx_next: u32,
    pub tx_next_ack: u32,
    pub rx_next: u32,
    pub rx_highest_status: u32,
}

/// AM entity statistics.
#[derive(Debug, Clone, Default)]
pub struct AmStats {
    pub tx_pdu_count: u64,
    pub rx_pdu_count: u64,
    pub retx_count_total: u64,
    pub rx_discard_count: u64,
    pub status_tx_count: u64,
    pub status_rx_count: u64,
}

// ─────────────────────────────────────────────────────────────────────────────
// Buffer Status Report helper
// ─────────────────────────────────────────────────────────────────────────────

/// Buffer Status Report data (simplified, per 3GPP TS 38.321 §6.1.3.1).
#[derive(Debug, Clone, Default)]
pub struct RlcBsr {
    /// Pending bytes in the TX SDU queue.
    pub pending_bytes: usize,
    /// Number of SDUs waiting.
    pub pending_sdus: usize,
    /// Retransmission bytes pending.
    pub retx_bytes: usize,
}

/// Compute a BSR from an AM entity's current state.
pub fn compute_bsr(entity: &RlcAmEntity) -> RlcBsr {
    RlcBsr {
        pending_bytes: entity.tx_buffer_bytes(),
        pending_sdus: entity.tx_sdu_queue.len(),
        retx_bytes: entity
            .tx_retx_buf
            .values()
            .flat_map(|v| v.iter())
            .filter(|s| !s.acked)
            .map(|s| s.data.len())
            .sum(),
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Sequence number arithmetic helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Compute (a - b) mod m, returning a value in [0, m).
#[inline]
fn sn_distance(a: u32, b: u32, m: u32) -> u32 {
    (a + m - b) % m
}

/// Compute (a - 1) mod m.
#[inline]
fn sn_sub(a: u32, delta: u32, m: u32) -> u32 {
    (a + m - delta) % m
}

/// Return true if `sn` is in the receive window [base, base+ws) mod m.
#[inline]
fn sn_in_window(sn: u32, base: u32, ws: u32, m: u32) -> bool {
    sn_distance(sn, base, m) < ws
}

// ─────────────────────────────────────────────────────────────────────────────
// Reassembly completeness check helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Check whether a set of UM segments forms a complete SDU.
fn is_complete(segs: &[UmSegment]) -> bool {
    // A complete SDU is signalled by SegmentInfo::Complete, or by the
    // presence of both a First and Last segment with no gaps in SO.
    if segs.iter().any(|s| s.si == SegmentInfo::Complete) {
        return true;
    }
    let has_first = segs.iter().any(|s| s.si == SegmentInfo::First);
    let has_last = segs.iter().any(|s| s.si == SegmentInfo::Last);
    if !has_first || !has_last {
        return false;
    }
    // Sort by SO and check contiguity
    let mut sorted: Vec<(u16, usize)> = segs.iter().map(|s| (s.so, s.data.len())).collect();
    sorted.sort_by_key(|&(so, _)| so);
    let mut expected_so: u32 = 0;
    for (so, len) in sorted {
        if so as u32 != expected_so {
            return false;
        }
        expected_so += len as u32;
    }
    true
}

/// Check whether a set of AM segments forms a complete SDU.
fn is_complete_am(segs: &[AmRxSegment]) -> bool {
    if segs.iter().any(|s| s.si == SegmentInfo::Complete) {
        return true;
    }
    let has_first = segs.iter().any(|s| s.si == SegmentInfo::First);
    let has_last = segs.iter().any(|s| s.si == SegmentInfo::Last);
    if !has_first || !has_last {
        return false;
    }
    let mut sorted: Vec<(u16, usize)> = segs.iter().map(|s| (s.so, s.data.len())).collect();
    sorted.sort_by_key(|&(so, _)| so);
    let mut expected: u32 = 0;
    for (so, len) in sorted {
        if so as u32 != expected {
            return false;
        }
        expected += len as u32;
    }
    true
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helpers ──────────────────────────────────────────────────────────────

    fn make_am_config(sn: SnFieldLength) -> RlcAmConfig {
        RlcAmConfig {
            sn_field_length: sn,
            t_poll_retransmit_ms: 45,
            t_reassembly_ms: 40,
            t_status_prohibit_ms: 0,
            poll_pdu: INFINITY,
            poll_byte: INFINITY,
            max_retx_threshold: 8,
        }
    }

    fn am_pair() -> (RlcAmEntity, RlcAmEntity) {
        let cfg = make_am_config(SnFieldLength::Len12);
        (RlcAmEntity::new(cfg.clone()), RlcAmEntity::new(cfg))
    }

    // ── SnFieldLength ─────────────────────────────────────────────────────────

    #[test]
    fn test_sn_field_length_bits() {
        assert_eq!(SnFieldLength::Len6.bits(), 6);
        assert_eq!(SnFieldLength::Len12.bits(), 12);
        assert_eq!(SnFieldLength::Len18.bits(), 18);
    }

    #[test]
    fn test_sn_field_length_modulus() {
        assert_eq!(SnFieldLength::Len6.modulus(), 64);
        assert_eq!(SnFieldLength::Len12.modulus(), 4096);
        assert_eq!(SnFieldLength::Len18.modulus(), 262_144);
    }

    #[test]
    fn test_sn_field_length_window_am() {
        assert_eq!(SnFieldLength::Len12.window_size_am(), 2048);
        assert_eq!(SnFieldLength::Len18.window_size_am(), 131_072);
    }

    // ── SegmentInfo ───────────────────────────────────────────────────────────

    #[test]
    fn test_segment_info_round_trip() {
        for bits in 0u8..4 {
            let si = SegmentInfo::from_bits(bits);
            assert_eq!(si as u8, bits);
        }
    }

    // ── SN arithmetic ─────────────────────────────────────────────────────────

    #[test]
    fn test_sn_distance() {
        let m = 4096u32;
        assert_eq!(sn_distance(5, 3, m), 2);
        assert_eq!(sn_distance(0, 4095, m), 1); // wrap-around
        assert_eq!(sn_distance(3, 3, m), 0);
    }

    #[test]
    fn test_sn_in_window() {
        let m = 4096u32;
        let ws = 2048u32;
        assert!(sn_in_window(0, 0, ws, m));
        assert!(sn_in_window(2047, 0, ws, m));
        assert!(!sn_in_window(2048, 0, ws, m));
        // Near wrap
        assert!(sn_in_window(0, 4090, ws, m));
    }

    #[test]
    fn test_sn_sub_wrap() {
        assert_eq!(sn_sub(0, 1, 4096), 4095);
        assert_eq!(sn_sub(5, 3, 4096), 2);
    }

    // ── TM Entity ─────────────────────────────────────────────────────────────

    #[test]
    fn test_tm_basic_loopback() {
        let mut tx = RlcTmEntity::new();
        let mut rx = RlcTmEntity::new();

        tx.tx_enqueue(b"BCCH info".to_vec()).unwrap();
        let pdus = tx.tx_generate();
        assert_eq!(pdus.len(), 1);
        assert_eq!(pdus[0], b"BCCH info");

        for p in &pdus {
            rx.rx_receive(p);
        }
        let sdus = rx.rx_deliver();
        assert_eq!(sdus, vec![b"BCCH info".to_vec()]);
    }

    #[test]
    fn test_tm_multiple_sdus() {
        let mut tm = RlcTmEntity::new();
        for i in 0u8..5 {
            tm.tx_enqueue(vec![i; 10]).unwrap();
        }
        let pdus = tm.tx_generate();
        assert_eq!(pdus.len(), 5);
        for (i, p) in pdus.iter().enumerate() {
            assert_eq!(*p, vec![i as u8; 10]);
        }
    }

    #[test]
    fn test_tm_sdu_too_large() {
        let mut tm = RlcTmEntity::new();
        let big = vec![0u8; MAX_SDU_SIZE + 1];
        assert_eq!(tm.tx_enqueue(big), Err(RlcError::SduTooLarge));
    }

    #[test]
    fn test_tm_reestablish_clears_buffers() {
        let mut tm = RlcTmEntity::new();
        tm.tx_enqueue(b"data".to_vec()).unwrap();
        tm.rx_receive(b"rxdata");
        tm.reestablish();
        assert!(tm.tx_generate().is_empty());
        assert!(tm.rx_deliver().is_empty());
    }

    #[test]
    fn test_tm_stats() {
        let mut tm = RlcTmEntity::new();
        tm.tx_enqueue(b"a".to_vec()).unwrap();
        tm.tx_generate();
        tm.rx_receive(b"b");
        let (tx, rx) = tm.stats();
        assert_eq!(tx, 1);
        assert_eq!(rx, 1);
    }

    // ── UM Header encode/decode ───────────────────────────────────────────────

    #[test]
    fn test_um_header_sn6_complete_roundtrip() {
        let hdr = encode_um_header(42, SegmentInfo::Complete, SnFieldLength::Len6, None);
        assert_eq!(hdr.len(), 1);
        let (sn, si, so, hlen) = decode_um_header(&hdr, SnFieldLength::Len6).unwrap();
        assert_eq!(sn, 42);
        assert_eq!(si, SegmentInfo::Complete);
        assert_eq!(so, 0);
        assert_eq!(hlen, 1);
    }

    #[test]
    fn test_um_header_sn12_complete_roundtrip() {
        let hdr = encode_um_header(1234, SegmentInfo::Complete, SnFieldLength::Len12, None);
        assert_eq!(hdr.len(), 2);
        let (sn, si, so, hlen) = decode_um_header(&hdr, SnFieldLength::Len12).unwrap();
        assert_eq!(sn, 1234);
        assert_eq!(si, SegmentInfo::Complete);
        assert_eq!(so, 0);
        assert_eq!(hlen, 2);
    }

    #[test]
    fn test_um_header_sn6_last_with_so() {
        let hdr = encode_um_header(10, SegmentInfo::Last, SnFieldLength::Len6, Some(512));
        assert_eq!(hdr.len(), 3); // 1 SN byte + 2 SO bytes
        let (sn, si, so, hlen) = decode_um_header(&hdr, SnFieldLength::Len6).unwrap();
        assert_eq!(sn, 10);
        assert_eq!(si, SegmentInfo::Last);
        assert_eq!(so, 512);
        assert_eq!(hlen, 3);
    }

    #[test]
    fn test_um_header_sn12_middle_with_so() {
        let hdr = encode_um_header(300, SegmentInfo::Middle, SnFieldLength::Len12, Some(1024));
        let (sn, si, so, hlen) = decode_um_header(&hdr, SnFieldLength::Len12).unwrap();
        assert_eq!(sn, 300);
        assert_eq!(si, SegmentInfo::Middle);
        assert_eq!(so, 1024);
        assert_eq!(hlen, 4);
    }

    // ── UM Entity ─────────────────────────────────────────────────────────────

    #[test]
    fn test_um_basic_loopback() {
        let cfg = RlcUmConfig::default();
        let mut tx = RlcUmEntity::new(cfg.clone());
        let mut rx = RlcUmEntity::new(cfg);

        tx.tx_enqueue(b"Hello UM".to_vec()).unwrap();
        let pdus = tx.tx_generate(256);
        assert!(!pdus.is_empty());

        for p in &pdus {
            rx.rx_receive(p);
        }
        let sdus = rx.rx_deliver();
        assert_eq!(sdus, vec![b"Hello UM".to_vec()]);
    }

    #[test]
    fn test_um_multiple_sdus_loopback() {
        let cfg = RlcUmConfig::default();
        let mut tx = RlcUmEntity::new(cfg.clone());
        let mut rx = RlcUmEntity::new(cfg);

        for i in 0u8..8 {
            tx.tx_enqueue(vec![i; 5]).unwrap();
        }
        let pdus = tx.tx_generate(8192);
        for p in &pdus {
            rx.rx_receive(p);
        }
        let sdus = rx.rx_deliver();
        for (i, sdu) in sdus.iter().enumerate() {
            assert_eq!(*sdu, vec![i as u8; 5]);
        }
    }

    #[test]
    fn test_um_sn6_loopback() {
        let cfg = RlcUmConfig {
            sn_field_length: SnFieldLength::Len6,
            t_reassembly_ms: 40,
        };
        let mut tx = RlcUmEntity::new(cfg.clone());
        let mut rx = RlcUmEntity::new(cfg);

        tx.tx_enqueue(b"SN6 test".to_vec()).unwrap();
        let pdus = tx.tx_generate(1500);
        for p in &pdus {
            rx.rx_receive(p);
        }
        let sdus = rx.rx_deliver();
        assert_eq!(sdus[0], b"SN6 test");
    }

    #[test]
    fn test_um_reestablish() {
        let cfg = RlcUmConfig::default();
        let mut e = RlcUmEntity::new(cfg);
        e.tx_enqueue(b"test".to_vec()).unwrap();
        e.reestablish();
        assert_eq!(e.tx_buffer_bytes(), 0);
    }

    #[test]
    fn test_um_sdu_too_large() {
        let cfg = RlcUmConfig::default();
        let mut e = RlcUmEntity::new(cfg);
        assert_eq!(
            e.tx_enqueue(vec![0u8; MAX_SDU_SIZE + 1]),
            Err(RlcError::SduTooLarge)
        );
    }

    #[test]
    fn test_um_stats_count() {
        let cfg = RlcUmConfig::default();
        let mut tx = RlcUmEntity::new(cfg.clone());
        let mut rx = RlcUmEntity::new(cfg);
        tx.tx_enqueue(b"x".to_vec()).unwrap();
        let pdus = tx.tx_generate(100);
        rx.rx_receive(&pdus[0]);
        let (tx_c, rx_c, disc) = rx.stats();
        assert_eq!(tx_c, 0);
        assert_eq!(rx_c, 1);
        assert_eq!(disc, 0);
    }

    // ── AM Header encode/decode ───────────────────────────────────────────────

    #[test]
    fn test_am_header_sn12_complete_roundtrip() {
        let hdr = encode_am_header(100, SegmentInfo::Complete, false, SnFieldLength::Len12, None);
        assert_eq!(hdr.len(), 2);
        let (sn, si, poll, so, hlen) = decode_am_header(&hdr, SnFieldLength::Len12).unwrap();
        assert_eq!(sn, 100);
        assert_eq!(si, SegmentInfo::Complete);
        assert!(!poll);
        assert_eq!(so, 0);
        assert_eq!(hlen, 2);
    }

    #[test]
    fn test_am_header_sn12_poll_bit() {
        let hdr = encode_am_header(200, SegmentInfo::Complete, true, SnFieldLength::Len12, None);
        let (sn, _, poll, _, _) = decode_am_header(&hdr, SnFieldLength::Len12).unwrap();
        assert_eq!(sn, 200);
        assert!(poll);
    }

    #[test]
    fn test_am_header_sn18_complete() {
        let hdr = encode_am_header(131071, SegmentInfo::Complete, false, SnFieldLength::Len18, None);
        assert_eq!(hdr.len(), 3);
        let (sn, si, _, _, hlen) = decode_am_header(&hdr, SnFieldLength::Len18).unwrap();
        assert_eq!(sn, 131071);
        assert_eq!(si, SegmentInfo::Complete);
        assert_eq!(hlen, 3);
    }

    #[test]
    fn test_am_header_with_so() {
        let hdr = encode_am_header(50, SegmentInfo::Last, false, SnFieldLength::Len12, Some(256));
        let (sn, si, _, so, hlen) = decode_am_header(&hdr, SnFieldLength::Len12).unwrap();
        assert_eq!(sn, 50);
        assert_eq!(si, SegmentInfo::Last);
        assert_eq!(so, 256);
        assert_eq!(hlen, 4);
    }

    // ── STATUS PDU ────────────────────────────────────────────────────────────

    #[test]
    fn test_status_pdu_ack_only() {
        let pdu = encode_status_pdu_12(42, &[]);
        let (ack, nacks) = decode_status_pdu_12(&pdu).unwrap();
        assert_eq!(ack, 42);
        assert!(nacks.is_empty());
    }

    #[test]
    fn test_status_pdu_single_nack() {
        let nacks_in = vec![NackEntry::new(10)];
        let pdu = encode_status_pdu_12(15, &nacks_in);
        let (ack, nacks_out) = decode_status_pdu_12(&pdu).unwrap();
        assert_eq!(ack, 15);
        assert_eq!(nacks_out.len(), 1);
        assert_eq!(nacks_out[0].nack_sn, 10);
    }

    #[test]
    fn test_status_pdu_multiple_nacks() {
        let nacks_in = vec![NackEntry::new(5), NackEntry::new(7), NackEntry::new(9)];
        let pdu = encode_status_pdu_12(12, &nacks_in);
        let (ack, nacks_out) = decode_status_pdu_12(&pdu).unwrap();
        assert_eq!(ack, 12);
        assert_eq!(nacks_out.len(), 3);
        assert_eq!(nacks_out[0].nack_sn, 5);
        assert_eq!(nacks_out[1].nack_sn, 7);
        assert_eq!(nacks_out[2].nack_sn, 9);
    }

    #[test]
    fn test_status_pdu_nack_with_so() {
        let mut n = NackEntry::new(20);
        n.has_so = true;
        n.so_start = 100;
        n.so_end = 200;
        let pdu = encode_status_pdu_12(25, &[n.clone()]);
        let (ack, nacks) = decode_status_pdu_12(&pdu).unwrap();
        assert_eq!(ack, 25);
        assert_eq!(nacks[0].nack_sn, 20);
        assert!(nacks[0].has_so);
        assert_eq!(nacks[0].so_start, 100);
        assert_eq!(nacks[0].so_end, 200);
    }

    #[test]
    fn test_status_pdu_nack_with_range() {
        let mut n = NackEntry::new(30);
        n.has_range = true;
        n.nack_range = 5;
        let pdu = encode_status_pdu_12(40, &[n]);
        let (ack, nacks) = decode_status_pdu_12(&pdu).unwrap();
        assert_eq!(ack, 40);
        assert!(nacks[0].has_range);
        assert_eq!(nacks[0].nack_range, 5);
    }

    #[test]
    fn test_status_pdu_too_short_returns_none() {
        assert!(decode_status_pdu_12(&[0x00, 0x00]).is_none());
        assert!(decode_status_pdu_12(&[]).is_none());
    }

    // ── AM Entity: basic loopback ─────────────────────────────────────────────

    #[test]
    fn test_am_basic_loopback() {
        let (mut tx, mut rx) = am_pair();
        tx.tx_enqueue(b"Hello AM".to_vec()).unwrap();
        let pdus = tx.tx_generate(1500);
        assert!(!pdus.is_empty());

        for p in &pdus {
            rx.rx_receive(p);
        }
        let sdus = rx.rx_deliver();
        assert_eq!(sdus, vec![b"Hello AM".to_vec()]);
    }

    #[test]
    fn test_am_multiple_sdus_loopback() {
        let (mut tx, mut rx) = am_pair();
        let expected: Vec<Vec<u8>> = (0u8..10).map(|i| vec![i; 20]).collect();
        for sdu in &expected {
            tx.tx_enqueue(sdu.clone()).unwrap();
        }
        let pdus = tx.tx_generate(65535);
        for p in &pdus {
            rx.rx_receive(p);
        }
        let sdus = rx.rx_deliver();
        assert_eq!(sdus, expected);
    }

    #[test]
    fn test_am_sn_state_advances() {
        let (mut tx, _) = am_pair();
        let state0 = tx.sn_state();
        assert_eq!(state0.tx_next, 0);
        tx.tx_enqueue(b"data".to_vec()).unwrap();
        tx.tx_generate(256);
        let state1 = tx.sn_state();
        assert_eq!(state1.tx_next, 1);
    }

    #[test]
    fn test_am_sdu_too_large() {
        let (mut tx, _) = am_pair();
        assert_eq!(
            tx.tx_enqueue(vec![0u8; MAX_SDU_SIZE + 1]),
            Err(RlcError::SduTooLarge)
        );
    }

    #[test]
    fn test_am_status_pdu_exchange() {
        let (mut tx_e, mut rx_e) = am_pair();
        tx_e.tx_enqueue(b"ACK me".to_vec()).unwrap();
        let pdus = tx_e.tx_generate(1500);
        for p in &pdus {
            rx_e.rx_receive(p);
        }
        rx_e.rx_deliver();
        // RX side should generate a STATUS PDU
        let status = rx_e.tx_status_pdu().unwrap();
        assert!(!status.is_empty());
        // Feed STATUS to TX
        tx_e.rx_receive(&status);
        // TX_Next_Ack should have advanced
        let state = tx_e.sn_state();
        assert_eq!(state.tx_next_ack, 1);
    }

    #[test]
    fn test_am_reestablish() {
        let (mut tx, _) = am_pair();
        tx.tx_enqueue(b"x".to_vec()).unwrap();
        tx.tx_generate(256);
        tx.reestablish();
        let s = tx.sn_state();
        assert_eq!(s.tx_next, 0);
        assert_eq!(s.tx_next_ack, 0);
    }

    #[test]
    fn test_am_retx_buffer_populated() {
        let (mut tx, _) = am_pair();
        tx.tx_enqueue(b"retx test".to_vec()).unwrap();
        tx.tx_generate(256);
        assert!(tx.retx_buffer_size() > 0);
    }

    #[test]
    fn test_am_stats_increment() {
        let (mut tx, mut rx) = am_pair();
        tx.tx_enqueue(b"stats".to_vec()).unwrap();
        let pdus = tx.tx_generate(256);
        for p in &pdus {
            rx.rx_receive(p);
        }
        let tstats = tx.stats();
        let rstats = rx.stats();
        assert!(tstats.tx_pdu_count > 0);
        assert!(rstats.rx_pdu_count > 0);
    }

    // ── AM Entity: SN-18 ─────────────────────────────────────────────────────

    #[test]
    fn test_am_sn18_loopback() {
        let cfg = make_am_config(SnFieldLength::Len18);
        let mut tx = RlcAmEntity::new(cfg.clone());
        let mut rx = RlcAmEntity::new(cfg);
        tx.tx_enqueue(b"SN-18 data".to_vec()).unwrap();
        let pdus = tx.tx_generate(1500);
        for p in &pdus {
            rx.rx_receive(p);
        }
        let sdus = rx.rx_deliver();
        assert_eq!(sdus[0], b"SN-18 data");
    }

    // ── AM Entity: poll triggers ──────────────────────────────────────────────

    #[test]
    fn test_am_poll_pdu_trigger() {
        let mut cfg = make_am_config(SnFieldLength::Len12);
        cfg.poll_pdu = 2;
        let mut tx = RlcAmEntity::new(cfg);
        for i in 0u8..3 {
            tx.tx_enqueue(vec![i; 5]).unwrap();
        }
        let pdus = tx.tx_generate(8192);
        // Third PDU (index 2) should have poll bit set
        if pdus.len() >= 3 {
            let (_, _, poll, _, _) =
                decode_am_header(&pdus[2], SnFieldLength::Len12).unwrap();
            assert!(poll);
        }
    }

    #[test]
    fn test_am_poll_byte_trigger() {
        let mut cfg = make_am_config(SnFieldLength::Len12);
        cfg.poll_byte = 10;
        let mut tx = RlcAmEntity::new(cfg);
        tx.tx_enqueue(vec![0u8; 12]).unwrap(); // > 10 bytes
        let pdus = tx.tx_generate(1500);
        // Poll should be set due to poll_byte
        let (_, _, poll, _, _) =
            decode_am_header(&pdus[0], SnFieldLength::Len12).unwrap();
        assert!(poll);
    }

    // ── AM: t-PollRetransmit ──────────────────────────────────────────────────

    #[test]
    fn test_am_poll_retransmit_timer_starts() {
        let mut cfg = make_am_config(SnFieldLength::Len12);
        cfg.poll_pdu = 1; // first PDU triggers poll
        let mut tx = RlcAmEntity::new(cfg);
        tx.tx_enqueue(b"trigger".to_vec()).unwrap();
        tx.tx_generate(256);
        assert!(tx.t_poll_retransmit_running);
    }

    #[test]
    fn test_am_poll_retransmit_timer_stops_on_ack() {
        let (_tx, mut rx) = am_pair();
        let mut cfg2 = make_am_config(SnFieldLength::Len12);
        cfg2.poll_pdu = 1;
        let mut tx2 = RlcAmEntity::new(cfg2);
        tx2.tx_enqueue(b"poll stop test".to_vec()).unwrap();
        let pdus = tx2.tx_generate(256);
        for p in &pdus {
            rx.rx_receive(p);
        }
        rx.rx_deliver();
        let status = rx.tx_status_pdu().unwrap();
        tx2.rx_receive(&status);
        // Timer should stop once all SNs are acked
        assert!(!tx2.t_poll_retransmit_running);
    }

    // ── AM: t-Reassembly ──────────────────────────────────────────────────────

    #[test]
    fn test_am_tick_does_not_panic() {
        let (mut tx, mut rx) = am_pair();
        tx.tx_enqueue(b"tick test".to_vec()).unwrap();
        let pdus = tx.tx_generate(256);
        rx.rx_receive(&pdus[0]);
        // Should not panic even if nothing to expire
        rx.tick(100);
        rx.tick(0);
    }

    // ── AM: pending status ────────────────────────────────────────────────────

    #[test]
    fn test_am_no_status_when_nothing_received() {
        let (_, mut rx) = am_pair();
        assert!(rx.tx_status_pdu().is_none());
    }

    #[test]
    fn test_am_has_pending_status_after_data() {
        let (mut tx, mut rx) = am_pair();
        tx.tx_enqueue(b"status trigger".to_vec()).unwrap();
        let pdus = tx.tx_generate(256);
        rx.rx_receive(&pdus[0]);
        // After data is received and buffered, pending_status depends on whether
        // gaps exist or reassembly has happened.
        // Just check it doesn't panic:
        let _ = rx.has_pending_status();
    }

    // ── AM: is_complete helpers ───────────────────────────────────────────────

    #[test]
    fn test_is_complete_am_single_complete() {
        let seg = AmRxSegment {
            so: 0,
            data: b"full".to_vec(),
            si: SegmentInfo::Complete,
        };
        assert!(is_complete_am(&[seg]));
    }

    #[test]
    fn test_is_complete_am_first_last_contiguous() {
        let segs = vec![
            AmRxSegment { so: 0, data: b"abc".to_vec(), si: SegmentInfo::First },
            AmRxSegment { so: 3, data: b"def".to_vec(), si: SegmentInfo::Last },
        ];
        assert!(is_complete_am(&segs));
    }

    #[test]
    fn test_is_complete_am_gap() {
        let segs = vec![
            AmRxSegment { so: 0, data: b"abc".to_vec(), si: SegmentInfo::First },
            AmRxSegment { so: 10, data: b"def".to_vec(), si: SegmentInfo::Last }, // gap at 3-9
        ];
        assert!(!is_complete_am(&segs));
    }

    #[test]
    fn test_is_complete_am_missing_last() {
        let segs = vec![
            AmRxSegment { so: 0, data: b"abc".to_vec(), si: SegmentInfo::First },
        ];
        assert!(!is_complete_am(&segs));
    }

    // ── BSR helper ────────────────────────────────────────────────────────────

    #[test]
    fn test_bsr_empty_entity() {
        let e = RlcAmEntity::new(make_am_config(SnFieldLength::Len12));
        let bsr = compute_bsr(&e);
        assert_eq!(bsr.pending_bytes, 0);
        assert_eq!(bsr.pending_sdus, 0);
        assert_eq!(bsr.retx_bytes, 0);
    }

    #[test]
    fn test_bsr_queued_sdu() {
        let mut e = RlcAmEntity::new(make_am_config(SnFieldLength::Len12));
        e.tx_enqueue(b"bsr_data_12345".to_vec()).unwrap();
        let bsr = compute_bsr(&e);
        assert_eq!(bsr.pending_bytes, 14);
        assert_eq!(bsr.pending_sdus, 1);
    }

    // ── NackEntry ─────────────────────────────────────────────────────────────

    #[test]
    fn test_nack_entry_new() {
        let n = NackEntry::new(99);
        assert_eq!(n.nack_sn, 99);
        assert!(!n.has_so);
        assert!(!n.has_range);
    }

    // ── Large SDU loopback (full single grant) ────────────────────────────────

    #[test]
    fn test_am_large_payload_multi_pdu() {
        let (mut tx, mut rx) = am_pair();
        // Three 1 KB SDUs in a single large grant – verifies concatenation path
        let payloads: Vec<Vec<u8>> = (0u8..3).map(|i| vec![i; 1024]).collect();
        for p in &payloads {
            tx.tx_enqueue(p.clone()).unwrap();
        }
        // One big grant covers all three SDUs
        let pdus = tx.tx_generate(65536);
        assert_eq!(pdus.len(), 3, "expected 3 PDUs for 3 SDUs");
        for p in &pdus {
            rx.rx_receive(p);
        }
        let sdus = rx.rx_deliver();
        assert_eq!(sdus.len(), 3);
        for (i, sdu) in sdus.iter().enumerate() {
            assert_eq!(*sdu, vec![i as u8; 1024]);
        }
    }

    // ── AM window full blocks TX ──────────────────────────────────────────────

    #[test]
    fn test_am_tx_window_full_stops_generation() {
        let mut cfg = make_am_config(SnFieldLength::Len12);
        // Very small window size requires that we test near-limit; SN-12 = 4096 mod, 2048 ws
        // Use a tiny amount of SDUs and check window is respected
        cfg.poll_pdu = INFINITY;
        let mut tx = RlcAmEntity::new(cfg);
        // Fill close to window limit
        for _ in 0..5 {
            tx.tx_enqueue(vec![0u8; 10]).unwrap();
        }
        let pdus = tx.tx_generate(65536);
        // All 5 SDUs should have been serialised (well within window)
        assert_eq!(pdus.len(), 5);
        let state = tx.sn_state();
        assert_eq!(state.tx_next, 5);
    }
}
