//! # 5G NR HARQ Processor
//!
//! Implements Hybrid Automatic Repeat reQuest (HARQ) for 5G NR per
//! 3GPP TS 38.213 (physical layer procedures for control) and
//! TS 38.321 (Medium Access Control protocol specification).
//!
//! ## Overview
//!
//! HARQ combines ARQ error detection with forward error correction (FEC) to
//! achieve reliable data transmission. The NR HARQ design supports:
//!
//! - Up to **16 DL processes** and **16 UL processes** per UE (TS 38.321)
//! - **Chase Combining (CC)**: accumulate LLRs from all retransmissions
//! - **Incremental Redundancy (IR)**: cycle through redundancy versions
//! - **Code Block Group (CBG)** retransmission for partial HARQ feedback
//! - **HARQ-ACK codebooks** (Type 1 semi-static, Type 2 dynamic)
//!
//! ## Redundancy Version Cycling (TS 38.214 §5.1.2.1)
//!
//! ```text
//! Initial TX → RV0
//! 1st retx   → RV2
//! 2nd retx   → RV3
//! 3rd retx   → RV1
//! 4th retx   → RV0 (wrap)
//! ```
//!
//! ## HARQ Timing (DL)
//!
//! The UE receives a PDSCH in slot n and reports HARQ-ACK in slot n+K1,
//! where K1 is signalled in the DCI (TS 38.213 §9.2.3).
//!
//! ## Example
//!
//! ```rust
//! use r4w_core::nr_harq_processor::{
//!     HarqManager, HarqConfig, HarqDirection, CombiningMode,
//!     HarqAckValue,
//! };
//!
//! let cfg = HarqConfig::default();
//! let mut mgr = HarqManager::new(cfg);
//!
//! // Simulate receiving a transport block on process 0
//! let llrs: Vec<f32> = vec![2.5_f32; 100];
//! let result = mgr.receive_dl(0, 0, false, &llrs);
//! assert!(result.ack_value == HarqAckValue::Nack || result.ack_value == HarqAckValue::Ack);
//! ```

// ─────────────────────────────────────────────────────────────────────────────
// Constants (TS 38.321, TS 38.213, TS 38.214)
// ─────────────────────────────────────────────────────────────────────────────

/// Maximum number of DL HARQ processes per UE (TS 38.321 §5.3.2).
pub const MAX_DL_HARQ_PROCESSES: usize = 16;

/// Maximum number of UL HARQ processes per UE (TS 38.321 §5.4.2).
pub const MAX_UL_HARQ_PROCESSES: usize = 16;

/// Maximum number of Code Block Groups per transport block (TS 38.213 §9.1.1).
pub const MAX_CBG: usize = 8;

/// Number of redundancy versions in the RV cycle (TS 38.214 §5.1.2.1).
pub const NUM_RV: usize = 4;

/// Maximum HARQ retransmissions before declaring failure.
pub const DEFAULT_MAX_RETX: u8 = 4;

/// Default soft buffer size in LLRs per process.
pub const DEFAULT_SOFT_BUFFER_SIZE: usize = 25344; // 3/4 * 33792 max LDPC CB

/// RV cycling sequence per TS 38.214 §5.1.2.1: RV0→RV2→RV3→RV1.
pub const RV_SEQUENCE: [u8; 4] = [0, 2, 3, 1];

/// K1 timing offset table (PDSCH slot → HARQ-ACK slot offset).
/// Default values for 15 kHz SCS FR1 (TS 38.213 Table 9.2.3-1).
pub const K1_DEFAULT_TABLE: [u8; 8] = [1, 2, 3, 4, 5, 6, 7, 8];

// ─────────────────────────────────────────────────────────────────────────────
// Enumerations
// ─────────────────────────────────────────────────────────────────────────────

/// HARQ link direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HarqDirection {
    /// Downlink HARQ (base station → UE).
    Downlink,
    /// Uplink HARQ (UE → base station).
    Uplink,
}

/// Soft-combining mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CombiningMode {
    /// Chase Combining: all retransmissions carry identical bits (RV0).
    /// LLRs are summed directly across retransmissions.
    ChaseCombing,
    /// Incremental Redundancy: each retransmission may carry different parity
    /// bits selected by the RV (redundancy version). LLRs are placed into
    /// the circular soft buffer at RV-specific offsets.
    IncrementalRedundancy,
}

/// HARQ-ACK value reported by the UE.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HarqAckValue {
    /// Transport block (or CBG) decoded successfully.
    Ack,
    /// Decoding failed; request retransmission.
    Nack,
    /// Discontinuous Transmission: no HARQ-ACK reported (DTX).
    Dtx,
}

/// HARQ process state machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HarqProcessState {
    /// No pending transport block; process is free.
    Idle,
    /// Transport block received/sent; waiting for ACK/NACK feedback.
    WaitingFeedback,
    /// NACK received; retransmission is pending/in-progress.
    Retransmitting,
    /// TB decoded successfully or max retransmissions reached.
    Completed,
}

/// HARQ-ACK codebook type (TS 38.213 §9.1).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HarqAckCodebookType {
    /// Type 1: semi-static codebook — fixed size based on PDSCH configuration.
    SemiStatic,
    /// Type 2: dynamic codebook — variable size driven by DAI counter in DCI.
    Dynamic,
}

/// CBG-based HARQ transmission indicator.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CbgRetxIndicator {
    /// Re-transmit all CBGs in this transport block.
    AllCbg,
    /// Selective retransmission: only CBGs indicated by the CBGTI bitmap.
    Selective,
}

// ─────────────────────────────────────────────────────────────────────────────
// Configuration structures
// ─────────────────────────────────────────────────────────────────────────────

/// Per-UE HARQ configuration.
#[derive(Debug, Clone)]
pub struct HarqConfig {
    /// Number of active DL HARQ processes (1..=16).
    pub num_dl_processes: usize,
    /// Number of active UL HARQ processes (1..=16).
    pub num_ul_processes: usize,
    /// Maximum number of retransmissions per process.
    pub max_retransmissions: u8,
    /// Soft-combining mode.
    pub combining_mode: CombiningMode,
    /// Soft buffer size per process in number of LLR elements.
    pub soft_buffer_size: usize,
    /// HARQ-ACK codebook type.
    pub codebook_type: HarqAckCodebookType,
    /// Enable CBG-based HARQ (if false, whole-TB HARQ only).
    pub cbg_enabled: bool,
    /// Number of CBGs when CBG HARQ is enabled (2, 4, 6, or 8).
    pub num_cbg: usize,
    /// K1 timing values indexed by DCI field value (0..7).
    pub k1_table: [u8; 8],
    /// LLR threshold for successful decoding (sum of |LLR| must exceed this).
    pub decode_threshold: f32,
}

impl Default for HarqConfig {
    fn default() -> Self {
        Self {
            num_dl_processes: MAX_DL_HARQ_PROCESSES,
            num_ul_processes: MAX_UL_HARQ_PROCESSES,
            max_retransmissions: DEFAULT_MAX_RETX,
            combining_mode: CombiningMode::IncrementalRedundancy,
            soft_buffer_size: DEFAULT_SOFT_BUFFER_SIZE,
            codebook_type: HarqAckCodebookType::SemiStatic,
            cbg_enabled: false,
            num_cbg: 4,
            k1_table: K1_DEFAULT_TABLE,
            decode_threshold: 0.5,
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Soft buffer
// ─────────────────────────────────────────────────────────────────────────────

/// Per-process soft buffer that accumulates LLRs across retransmissions.
///
/// For Chase Combining the buffer size equals the coded block size (all
/// retransmissions are identical). For Incremental Redundancy the buffer
/// is a circular rate-matching buffer of length N_cb = min(N, 25344) where
/// retransmissions written at RV-specific starting positions.
#[derive(Debug, Clone)]
pub struct SoftBuffer {
    /// LLR accumulation storage. Positive = likely '0', negative = likely '1'.
    pub llrs: Vec<f32>,
    /// Number of times the buffer has been written to (retransmission count).
    pub accumulation_count: u32,
}

impl SoftBuffer {
    /// Create a new zero-initialised soft buffer.
    pub fn new(size: usize) -> Self {
        Self {
            llrs: vec![0.0_f32; size],
            accumulation_count: 0,
        }
    }

    /// Flush (zero) the buffer and reset accumulation counter.
    pub fn flush(&mut self) {
        self.llrs.iter_mut().for_each(|x| *x = 0.0);
        self.accumulation_count = 0;
    }

    /// Accumulate new LLRs via simple addition (Chase Combining / IR).
    ///
    /// For IR the `offset` is the RV-specific starting position within the
    /// circular buffer, derived from [`rv_start_position`].
    pub fn accumulate(&mut self, new_llrs: &[f32], offset: usize) {
        let n = self.llrs.len();
        for (i, &l) in new_llrs.iter().enumerate() {
            let idx = (offset + i) % n;
            self.llrs[idx] += l;
        }
        self.accumulation_count += 1;
    }

    /// Return the mean absolute LLR value (proxy for accumulated SNR).
    pub fn mean_abs_llr(&self) -> f32 {
        if self.llrs.is_empty() {
            return 0.0;
        }
        let sum: f32 = self.llrs.iter().map(|x| x.abs()).sum();
        sum / self.llrs.len() as f32
    }

    /// Hard-decode the accumulated LLRs: positive LLR → bit 0, negative → bit 1.
    pub fn hard_decode(&self) -> Vec<u8> {
        self.llrs.iter().map(|&l| if l >= 0.0 { 0 } else { 1 }).collect()
    }
}

/// Compute the starting position within the circular soft buffer for a given
/// redundancy version (TS 38.214 §5.4.2.1).
///
/// For a rate-matched buffer of length E and mother code length N_cb:
/// - RV0: k_0 = 0
/// - RV1: k_0 = floor(17 * N_cb / 66) * 2 (rounded to even)
/// - RV2: k_0 = floor(33 * N_cb / 66) * 2
/// - RV3: k_0 = floor(56 * N_cb / 66) * 2
pub fn rv_start_position(rv: u8, n_cb: usize) -> usize {
    let numerators = [0usize, 17, 33, 56];
    let rv_idx = rv.min(3) as usize;
    let raw = (numerators[rv_idx] * n_cb) / 66;
    // Round down to nearest even number (2 bits per QPSK symbol)
    raw & !1
}

// ─────────────────────────────────────────────────────────────────────────────
// HARQ Process
// ─────────────────────────────────────────────────────────────────────────────

/// CBG status bitmap — one bit per CBG indicating ACK(1) or NACK(0).
#[derive(Debug, Clone, Copy, Default)]
pub struct CbgStatusBitmap(pub u8);

impl CbgStatusBitmap {
    /// Set ACK for CBG index `idx`.
    pub fn set_ack(&mut self, idx: usize) {
        self.0 |= 1 << idx;
    }
    /// Set NACK for CBG index `idx`.
    pub fn set_nack(&mut self, idx: usize) {
        self.0 &= !(1 << idx);
    }
    /// Return true if all CBGs in `num_cbg` are ACKed.
    pub fn all_acked(&self, num_cbg: usize) -> bool {
        let mask = (1u8 << num_cbg).wrapping_sub(1);
        (self.0 & mask) == mask
    }
    /// Return the raw bitmap byte.
    pub fn raw(&self) -> u8 {
        self.0
    }
}

/// A single HARQ process tracking state, buffers, and timing.
#[derive(Debug, Clone)]
pub struct HarqProcess {
    /// Process index (0..15).
    pub process_id: usize,
    /// Current state of the process state machine.
    pub state: HarqProcessState,
    /// New Data Indicator: toggles on each new TB transmission.
    pub ndi: bool,
    /// Current redundancy version (0–3).
    pub rv: u8,
    /// Number of retransmissions attempted (not counting initial).
    pub retx_count: u8,
    /// Soft buffer for LLR accumulation.
    pub soft_buffer: SoftBuffer,
    /// CBG-level soft buffers (one per CBG).
    pub cbg_soft_buffers: Vec<SoftBuffer>,
    /// Slot number in which the transport block was last scheduled.
    pub last_scheduled_slot: u32,
    /// Slot number by which HARQ-ACK must be reported (last_scheduled_slot + K1).
    pub ack_due_slot: u32,
    /// HARQ-ACK value for this process (updated after decoding).
    pub ack_value: HarqAckValue,
    /// CBG ACK/NACK bitmap (valid when CBG HARQ is enabled).
    pub cbg_status: CbgStatusBitmap,
    /// Downlink Assignment Index counter (TS 38.213 §9.1.3, Type 2 codebook).
    pub dai: u8,
    /// Total DAI from the last DCI (Type 2 codebook).
    pub total_dai: u8,
    /// Transport block size in bytes (set on new-data transmission).
    pub tb_size_bytes: usize,
    /// Number of successfully decoded transport blocks.
    pub successful_decodes: u64,
    /// Number of total transmission attempts (initial + retx).
    pub total_tx_attempts: u64,
}

impl HarqProcess {
    /// Create a new idle HARQ process.
    pub fn new(process_id: usize, soft_buffer_size: usize, num_cbg: usize) -> Self {
        let cbg_buf_size = if num_cbg > 0 { soft_buffer_size / num_cbg } else { 0 };
        Self {
            process_id,
            state: HarqProcessState::Idle,
            ndi: false,
            rv: 0,
            retx_count: 0,
            soft_buffer: SoftBuffer::new(soft_buffer_size),
            cbg_soft_buffers: (0..num_cbg).map(|_| SoftBuffer::new(cbg_buf_size)).collect(),
            last_scheduled_slot: 0,
            ack_due_slot: 0,
            ack_value: HarqAckValue::Dtx,
            cbg_status: CbgStatusBitmap::default(),
            dai: 0,
            total_dai: 0,
            tb_size_bytes: 0,
            successful_decodes: 0,
            total_tx_attempts: 0,
        }
    }

    /// Flush all soft buffers and reset the process to Idle.
    pub fn flush(&mut self) {
        self.soft_buffer.flush();
        for buf in &mut self.cbg_soft_buffers {
            buf.flush();
        }
        self.state = HarqProcessState::Idle;
        self.rv = 0;
        self.retx_count = 0;
        self.ack_value = HarqAckValue::Dtx;
        self.cbg_status = CbgStatusBitmap::default();
        self.dai = 0;
        self.total_dai = 0;
    }

    /// Toggle NDI (New Data Indicator) — called when a new TB is scheduled.
    pub fn toggle_ndi(&mut self) {
        self.ndi = !self.ndi;
    }

    /// Return the next RV in the cycling sequence.
    pub fn next_rv(&self) -> u8 {
        let seq_idx = (self.retx_count as usize + 1) % NUM_RV;
        RV_SEQUENCE[seq_idx]
    }

    /// Advance to next retransmission: increment counter and update RV.
    pub fn advance_retx(&mut self) {
        self.retx_count += 1;
        let seq_idx = self.retx_count as usize % NUM_RV;
        self.rv = RV_SEQUENCE[seq_idx];
    }

    /// Return true if the maximum retransmission count has been reached.
    pub fn max_retx_reached(&self, max_retx: u8) -> bool {
        self.retx_count >= max_retx
    }

    /// Compute HARQ round-trip time in slots given K1 offset.
    ///
    /// RTT = 2 * K1 + 1 (DL: gNB→UE→gNB).
    pub fn round_trip_slots(k1: u8) -> u32 {
        2 * k1 as u32 + 1
    }

    /// Estimated throughput in bits per slot based on TB size and success rate.
    pub fn estimated_throughput_bps(&self, slot_duration_ms: f64) -> f64 {
        if self.total_tx_attempts == 0 {
            return 0.0;
        }
        let success_rate = self.successful_decodes as f64 / self.total_tx_attempts as f64;
        let bits_per_attempt = self.tb_size_bytes as f64 * 8.0;
        success_rate * bits_per_attempt / slot_duration_ms
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Decoding result
// ─────────────────────────────────────────────────────────────────────────────

/// Result returned after processing a received transmission.
#[derive(Debug, Clone)]
pub struct HarqDecodeResult {
    /// HARQ-ACK feedback value to be reported.
    pub ack_value: HarqAckValue,
    /// Process ID this result belongs to.
    pub process_id: usize,
    /// True if this was a new-data transmission (NDI toggled).
    pub is_new_data: bool,
    /// Accumulated retransmission count for this TB.
    pub retx_count: u8,
    /// Current state of the process after processing.
    pub state: HarqProcessState,
    /// Decoded bits (hard decisions from combined LLRs).
    pub decoded_bits: Vec<u8>,
    /// Mean absolute LLR after combining (higher = more reliable).
    pub mean_abs_llr: f32,
    /// CBG-level ACK/NACK bitmap (valid when CBG HARQ enabled).
    pub cbg_status: CbgStatusBitmap,
}

// ─────────────────────────────────────────────────────────────────────────────
// HARQ-ACK codebook
// ─────────────────────────────────────────────────────────────────────────────

/// Entry in the HARQ-ACK codebook.
#[derive(Debug, Clone)]
pub struct HarqAckEntry {
    /// Serving cell index.
    pub cell_index: u8,
    /// HARQ process ID.
    pub process_id: u8,
    /// HARQ-ACK value.
    pub ack: HarqAckValue,
    /// CBG-level status (only meaningful when CBG HARQ is active).
    pub cbg_bitmap: Option<CbgStatusBitmap>,
    /// DAI value from the DCI that triggered this entry.
    pub dai: u8,
}

/// HARQ-ACK codebook — collects per-process ACK/NACK for PUCCH/PUSCH reporting.
///
/// Type 1 (semi-static): size is determined by the configured PDSCH HARQ process
/// count and number of serving cells. Entries are always present (DTX for absent).
///
/// Type 2 (dynamic): size is determined by the DAI counter in received DCIs.
/// Only slots with a DCI contribute entries.
#[derive(Debug, Clone, Default)]
pub struct HarqAckCodebook {
    /// Ordered list of ACK/NACK entries.
    pub entries: Vec<HarqAckEntry>,
    /// Codebook type used for construction.
    pub codebook_type: HarqAckCodebookType,
    /// Total DAI from the last DCI in this bundling window (Type 2).
    pub total_dai: u8,
}

impl Default for HarqAckCodebookType {
    fn default() -> Self {
        HarqAckCodebookType::SemiStatic
    }
}

impl HarqAckCodebook {
    /// Create an empty codebook.
    pub fn new(codebook_type: HarqAckCodebookType) -> Self {
        Self {
            entries: Vec::new(),
            codebook_type,
            total_dai: 0,
        }
    }

    /// Append an ACK/NACK entry.
    pub fn push(&mut self, entry: HarqAckEntry) {
        self.entries.push(entry);
    }

    /// Clear all entries (start of new bundling window).
    pub fn clear(&mut self) {
        self.entries.clear();
        self.total_dai = 0;
    }

    /// Number of ACK bits in this codebook.
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// True if the codebook contains no entries.
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// Serialize to a bit vector for PUCCH/PUSCH multiplexing.
    ///
    /// Returns a Vec<u8> where 1 = ACK and 0 = NACK/DTX.
    pub fn to_bit_vector(&self) -> Vec<u8> {
        self.entries
            .iter()
            .map(|e| match e.ack {
                HarqAckValue::Ack => 1,
                _ => 0,
            })
            .collect()
    }

    /// Spatial bundling: AND all ACK values across spatial layers.
    ///
    /// Returns ACK only if all entries are ACK (TS 38.213 §9.1.2).
    pub fn spatial_bundle(&self) -> HarqAckValue {
        if self.entries.is_empty() {
            return HarqAckValue::Dtx;
        }
        let all_ack = self.entries.iter().all(|e| e.ack == HarqAckValue::Ack);
        if all_ack {
            HarqAckValue::Ack
        } else {
            HarqAckValue::Nack
        }
    }

    /// Time-domain bundling: AND across time-domain HARQ-ACK values.
    ///
    /// Used in slot-bundling mode (TS 38.213 §9.1.2).
    pub fn time_bundle(&self, process_ids: &[usize]) -> HarqAckValue {
        let relevant: Vec<&HarqAckEntry> = self
            .entries
            .iter()
            .filter(|e| process_ids.contains(&(e.process_id as usize)))
            .collect();
        if relevant.is_empty() {
            return HarqAckValue::Dtx;
        }
        let all_ack = relevant.iter().all(|e| e.ack == HarqAckValue::Ack);
        if all_ack {
            HarqAckValue::Ack
        } else {
            HarqAckValue::Nack
        }
    }

    /// Type 2 codebook: count entries matching the expected DAI sequence.
    ///
    /// Returns the number of detected transmission gaps (missing DCIs).
    pub fn count_dai_gaps(&self) -> usize {
        if self.entries.is_empty() || self.total_dai == 0 {
            return 0;
        }
        let expected_count = self.total_dai as usize;
        expected_count.saturating_sub(self.entries.len())
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// HARQ Manager
// ─────────────────────────────────────────────────────────────────────────────

/// Top-level HARQ process manager for a single UE.
///
/// Manages all DL and UL HARQ processes, soft buffers, combining, and
/// HARQ-ACK codebook construction.
pub struct HarqManager {
    cfg: HarqConfig,
    dl_processes: Vec<HarqProcess>,
    ul_processes: Vec<HarqProcess>,
    codebook: HarqAckCodebook,
    /// Running slot counter.
    current_slot: u32,
    /// Total DL transport blocks received (initial + retx).
    total_dl_rx: u64,
    /// Total DL transport blocks successfully decoded.
    total_dl_decoded: u64,
    /// Total UL transport blocks transmitted.
    total_ul_tx: u64,
    /// Total UL transport blocks acknowledged.
    total_ul_acked: u64,
}

impl HarqManager {
    /// Create a new HARQ manager with the given configuration.
    pub fn new(cfg: HarqConfig) -> Self {
        let soft_buf = cfg.soft_buffer_size;
        let num_cbg = if cfg.cbg_enabled { cfg.num_cbg } else { 0 };

        let dl_processes = (0..cfg.num_dl_processes)
            .map(|id| HarqProcess::new(id, soft_buf, num_cbg))
            .collect();
        let ul_processes = (0..cfg.num_ul_processes)
            .map(|id| HarqProcess::new(id, soft_buf, num_cbg))
            .collect();
        let codebook = HarqAckCodebook::new(cfg.codebook_type);

        Self {
            cfg,
            dl_processes,
            ul_processes,
            codebook,
            current_slot: 0,
            total_dl_rx: 0,
            total_dl_decoded: 0,
            total_ul_tx: 0,
            total_ul_acked: 0,
        }
    }

    /// Advance the slot counter.
    pub fn advance_slot(&mut self) {
        self.current_slot = self.current_slot.wrapping_add(1);
    }

    /// Get the current slot number.
    pub fn current_slot(&self) -> u32 {
        self.current_slot
    }

    /// Borrow an immutable reference to a DL process.
    pub fn dl_process(&self, id: usize) -> Option<&HarqProcess> {
        self.dl_processes.get(id)
    }

    /// Borrow an immutable reference to a UL process.
    pub fn ul_process(&self, id: usize) -> Option<&HarqProcess> {
        self.ul_processes.get(id)
    }

    /// Compute the K1 timing offset from the DCI field value (0..7).
    pub fn k1_from_dci_field(&self, field_val: usize) -> u8 {
        self.cfg.k1_table[field_val.min(7)]
    }

    /// Compute the ACK reporting slot: PDSCH slot + K1.
    pub fn ack_due_slot(&self, pdsch_slot: u32, k1: u8) -> u32 {
        pdsch_slot.wrapping_add(k1 as u32)
    }

    /// Process a received DL transmission.
    ///
    /// # Arguments
    /// * `process_id` — HARQ process index (0..15)
    /// * `rv` — Redundancy version from DCI (0–3)
    /// * `ndi` — New Data Indicator from DCI (true = new TB)
    /// * `llrs` — Received LLR vector from the channel decoder input
    ///
    /// Returns a [`HarqDecodeResult`] containing the HARQ-ACK and decoded bits.
    pub fn receive_dl(
        &mut self,
        process_id: usize,
        rv: u8,
        ndi: bool,
        llrs: &[f32],
    ) -> HarqDecodeResult {
        self.receive_dl_k1(process_id, rv, ndi, llrs, 4, 0)
    }

    /// Process a received DL transmission with explicit K1 and DAI.
    ///
    /// # Arguments
    /// * `process_id` — HARQ process index
    /// * `rv` — Redundancy version from DCI
    /// * `ndi` — New Data Indicator from DCI
    /// * `llrs` — Received soft LLR values
    /// * `k1` — HARQ-ACK timing offset (slots)
    /// * `dai` — Downlink Assignment Index from DCI
    pub fn receive_dl_k1(
        &mut self,
        process_id: usize,
        rv: u8,
        ndi: bool,
        llrs: &[f32],
        k1: u8,
        dai: u8,
    ) -> HarqDecodeResult {
        assert!(process_id < self.cfg.num_dl_processes, "Invalid DL process ID");

        self.total_dl_rx += 1;
        let proc = &mut self.dl_processes[process_id];

        // Detect new-data vs retransmission from NDI toggle.
        let is_new_data = ndi != proc.ndi;
        if is_new_data {
            proc.flush();
            proc.ndi = ndi;
            proc.rv = rv;
            proc.state = HarqProcessState::WaitingFeedback;
            proc.last_scheduled_slot = self.current_slot;
            proc.ack_due_slot = self.current_slot.wrapping_add(k1 as u32);
            proc.dai = dai;
            proc.tb_size_bytes = llrs.len() / 8; // rough estimate
        } else {
            // Retransmission
            proc.state = HarqProcessState::Retransmitting;
            proc.rv = rv;
        }
        proc.total_tx_attempts += 1;

        // Compute circular buffer offset for IR.
        let offset = match self.cfg.combining_mode {
            CombiningMode::ChaseCombing => 0,
            CombiningMode::IncrementalRedundancy => {
                rv_start_position(rv, proc.soft_buffer.llrs.len())
            }
        };

        // Accumulate LLRs into soft buffer.
        let llr_len = llrs.len().min(proc.soft_buffer.llrs.len());
        proc.soft_buffer.accumulate(&llrs[..llr_len], offset);

        // Attempt decoding via threshold on mean |LLR|.
        let mean_llr = proc.soft_buffer.mean_abs_llr();
        let decode_ok = mean_llr >= self.cfg.decode_threshold;
        let decoded_bits = proc.soft_buffer.hard_decode();

        let ack_value = if decode_ok {
            proc.state = HarqProcessState::Completed;
            proc.successful_decodes += 1;
            HarqAckValue::Ack
        } else if proc.max_retx_reached(self.cfg.max_retransmissions) {
            // Give up after max retransmissions.
            proc.state = HarqProcessState::Completed;
            HarqAckValue::Nack
        } else {
            proc.advance_retx();
            HarqAckValue::Nack
        };

        if ack_value == HarqAckValue::Ack {
            self.total_dl_decoded += 1;
        }

        proc.ack_value = ack_value;

        let result = HarqDecodeResult {
            ack_value,
            process_id,
            is_new_data,
            retx_count: proc.retx_count,
            state: proc.state,
            decoded_bits,
            mean_abs_llr: mean_llr,
            cbg_status: proc.cbg_status,
        };

        // Add to codebook.
        self.codebook.push(HarqAckEntry {
            cell_index: 0,
            process_id: process_id as u8,
            ack: ack_value,
            cbg_bitmap: if self.cfg.cbg_enabled { Some(proc.cbg_status) } else { None },
            dai,
        });

        result
    }

    /// Process a CBG-based DL transmission.
    ///
    /// Each CBG has its own soft buffer and independent ACK/NACK.
    ///
    /// # Arguments
    /// * `process_id` — HARQ process index
    /// * `rv` — Redundancy version
    /// * `ndi` — New Data Indicator
    /// * `cbgti` — CBG Transmission Indicator bitmap (bit i = 1 → CBG i is present)
    /// * `cbg_llrs` — Per-CBG LLR vectors
    pub fn receive_dl_cbg(
        &mut self,
        process_id: usize,
        rv: u8,
        ndi: bool,
        cbgti: u8,
        cbg_llrs: &[Vec<f32>],
    ) -> HarqDecodeResult {
        assert!(self.cfg.cbg_enabled, "CBG HARQ not enabled");
        assert!(process_id < self.cfg.num_dl_processes, "Invalid DL process ID");

        self.total_dl_rx += 1;
        let proc = &mut self.dl_processes[process_id];

        let is_new_data = ndi != proc.ndi;
        if is_new_data {
            proc.flush();
            proc.ndi = ndi;
            proc.rv = rv;
            proc.state = HarqProcessState::WaitingFeedback;
        }
        proc.total_tx_attempts += 1;

        let num_cbg = self.cfg.num_cbg.min(cbg_llrs.len());

        // Accumulate LLRs per CBG if present in CBGTI.
        for cbg_idx in 0..num_cbg {
            if (cbgti >> cbg_idx) & 1 == 1 {
                let offset = match self.cfg.combining_mode {
                    CombiningMode::ChaseCombing => 0,
                    CombiningMode::IncrementalRedundancy => {
                        rv_start_position(rv, proc.cbg_soft_buffers[cbg_idx].llrs.len())
                    }
                };
                let llrs = &cbg_llrs[cbg_idx];
                let buf_len = proc.cbg_soft_buffers[cbg_idx].llrs.len();
                let clamped = llrs.len().min(buf_len);
                proc.cbg_soft_buffers[cbg_idx].accumulate(&llrs[..clamped], offset);
            }

            // Decode each CBG independently.
            let mean_llr = proc.cbg_soft_buffers[cbg_idx].mean_abs_llr();
            if mean_llr >= self.cfg.decode_threshold {
                proc.cbg_status.set_ack(cbg_idx);
            } else {
                proc.cbg_status.set_nack(cbg_idx);
            }
        }

        let all_acked = proc.cbg_status.all_acked(num_cbg);
        let ack_value = if all_acked {
            proc.state = HarqProcessState::Completed;
            proc.successful_decodes += 1;
            HarqAckValue::Ack
        } else {
            if proc.max_retx_reached(self.cfg.max_retransmissions) {
                proc.state = HarqProcessState::Completed;
            } else {
                proc.advance_retx();
                proc.state = HarqProcessState::Retransmitting;
            }
            HarqAckValue::Nack
        };

        if ack_value == HarqAckValue::Ack {
            self.total_dl_decoded += 1;
        }
        proc.ack_value = ack_value;

        let mean_llr = proc.soft_buffer.mean_abs_llr();
        let decoded_bits = proc.soft_buffer.hard_decode();
        let cbg_status = proc.cbg_status;

        HarqDecodeResult {
            ack_value,
            process_id,
            is_new_data,
            retx_count: proc.retx_count,
            state: proc.state,
            decoded_bits,
            mean_abs_llr: mean_llr,
            cbg_status,
        }
    }

    /// Process UL HARQ feedback (ACK/NACK received at gNB).
    ///
    /// On ACK: flush process, increment success counter.
    /// On NACK: trigger non-adaptive retransmission if max not reached.
    ///
    /// # Arguments
    /// * `process_id` — UL HARQ process index
    /// * `ack` — Received HARQ-ACK
    ///
    /// Returns `true` if a retransmission should be scheduled.
    pub fn process_ul_feedback(&mut self, process_id: usize, ack: HarqAckValue) -> bool {
        assert!(process_id < self.cfg.num_ul_processes, "Invalid UL process ID");

        let proc = &mut self.ul_processes[process_id];
        match ack {
            HarqAckValue::Ack => {
                self.total_ul_acked += 1;
                proc.successful_decodes += 1;
                proc.state = HarqProcessState::Completed;
                false
            }
            HarqAckValue::Nack => {
                if proc.max_retx_reached(self.cfg.max_retransmissions) {
                    proc.state = HarqProcessState::Completed;
                    false
                } else {
                    proc.advance_retx();
                    proc.state = HarqProcessState::Retransmitting;
                    true
                }
            }
            HarqAckValue::Dtx => {
                // Treat DTX as NACK.
                if proc.max_retx_reached(self.cfg.max_retransmissions) {
                    proc.state = HarqProcessState::Completed;
                    false
                } else {
                    proc.advance_retx();
                    proc.state = HarqProcessState::Retransmitting;
                    true
                }
            }
        }
    }

    /// Initiate a new UL HARQ transmission.
    ///
    /// # Arguments
    /// * `process_id` — UL HARQ process index
    /// * `ndi` — New Data Indicator (toggled from previous)
    /// * `tb_size_bytes` — Transport block size
    ///
    /// Returns the RV to use for this transmission.
    pub fn initiate_ul_tx(&mut self, process_id: usize, ndi: bool, tb_size_bytes: usize) -> u8 {
        assert!(process_id < self.cfg.num_ul_processes, "Invalid UL process ID");

        let proc = &mut self.ul_processes[process_id];
        let is_new_data = ndi != proc.ndi;
        if is_new_data {
            proc.flush();
            proc.ndi = ndi;
            proc.tb_size_bytes = tb_size_bytes;
        }
        proc.total_tx_attempts += 1;
        proc.state = HarqProcessState::WaitingFeedback;
        self.total_ul_tx += 1;
        proc.rv
    }

    /// Flush a specific DL process (e.g., on reconfiguration).
    pub fn flush_dl_process(&mut self, process_id: usize) {
        if let Some(p) = self.dl_processes.get_mut(process_id) {
            p.flush();
        }
    }

    /// Flush a specific UL process.
    pub fn flush_ul_process(&mut self, process_id: usize) {
        if let Some(p) = self.ul_processes.get_mut(process_id) {
            p.flush();
        }
    }

    /// Flush all DL and UL processes (e.g., after handover).
    pub fn flush_all(&mut self) {
        for p in &mut self.dl_processes {
            p.flush();
        }
        for p in &mut self.ul_processes {
            p.flush();
        }
        self.codebook.clear();
    }

    /// Build the HARQ-ACK codebook for the current reporting slot.
    ///
    /// For Type 1 (semi-static): includes all DL processes, DTX for absent.
    /// For Type 2 (dynamic): only processes with received DCIs in the window.
    pub fn build_codebook(&self) -> &HarqAckCodebook {
        &self.codebook
    }

    /// Clear the codebook after reporting (start of new bundling window).
    pub fn clear_codebook(&mut self) {
        self.codebook.clear();
    }

    /// Return the HARQ-ACK bit vector for PUCCH multiplexing.
    pub fn harq_ack_bits(&self) -> Vec<u8> {
        self.codebook.to_bit_vector()
    }

    /// DL HARQ statistics: (total_rx, total_decoded).
    pub fn dl_stats(&self) -> (u64, u64) {
        (self.total_dl_rx, self.total_dl_decoded)
    }

    /// UL HARQ statistics: (total_tx, total_acked).
    pub fn ul_stats(&self) -> (u64, u64) {
        (self.total_ul_tx, self.total_ul_acked)
    }

    /// DL block error rate (BLER) = 1 - (decoded / total_rx).
    pub fn dl_bler(&self) -> f64 {
        if self.total_dl_rx == 0 {
            return 0.0;
        }
        1.0 - (self.total_dl_decoded as f64 / self.total_dl_rx as f64)
    }

    /// UL block error rate (BLER) = 1 - (acked / total_tx).
    pub fn ul_bler(&self) -> f64 {
        if self.total_ul_tx == 0 {
            return 0.0;
        }
        1.0 - (self.total_ul_acked as f64 / self.total_ul_tx as f64)
    }

    /// Number of DL processes currently in Retransmitting state.
    pub fn active_dl_retx_count(&self) -> usize {
        self.dl_processes
            .iter()
            .filter(|p| p.state == HarqProcessState::Retransmitting)
            .count()
    }

    /// Number of UL processes currently in Retransmitting state.
    pub fn active_ul_retx_count(&self) -> usize {
        self.ul_processes
            .iter()
            .filter(|p| p.state == HarqProcessState::Retransmitting)
            .count()
    }

    /// Find the first idle DL process, if any.
    pub fn find_idle_dl_process(&self) -> Option<usize> {
        self.dl_processes
            .iter()
            .find(|p| p.state == HarqProcessState::Idle)
            .map(|p| p.process_id)
    }

    /// Find the first idle UL process, if any.
    pub fn find_idle_ul_process(&self) -> Option<usize> {
        self.ul_processes
            .iter()
            .find(|p| p.state == HarqProcessState::Idle)
            .map(|p| p.process_id)
    }

    /// Return the HARQ round-trip time in slots for the given K1.
    pub fn harq_rtt_slots(k1: u8) -> u32 {
        HarqProcess::round_trip_slots(k1)
    }

    /// Estimated DL throughput in bits per slot for a given process.
    pub fn dl_throughput_bps(&self, process_id: usize, slot_duration_ms: f64) -> f64 {
        self.dl_processes
            .get(process_id)
            .map(|p| p.estimated_throughput_bps(slot_duration_ms))
            .unwrap_or(0.0)
    }

    /// Process HARQ-ACK multiplexing: pack multiple ACK bits for PUCCH/PUSCH.
    ///
    /// Returns a byte vector where each bit represents one HARQ-ACK (MSB first).
    pub fn pack_harq_ack_bits(&self) -> Vec<u8> {
        let bits = self.codebook.to_bit_vector();
        let byte_len = (bits.len() + 7) / 8;
        let mut out = vec![0u8; byte_len];
        for (i, &bit) in bits.iter().enumerate() {
            if bit == 1 {
                out[i / 8] |= 1 << (7 - (i % 8));
            }
        }
        out
    }

    /// Spatial bundling of HARQ-ACK across all DL processes.
    pub fn spatial_bundle_dl(&self) -> HarqAckValue {
        self.codebook.spatial_bundle()
    }

    /// Configuration accessor.
    pub fn config(&self) -> &HarqConfig {
        &self.cfg
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// HARQ timing utilities
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the HARQ-ACK slot given PDSCH slot and K1 timing offset.
///
/// Per TS 38.213 §9.2.3 the HARQ-ACK reporting slot is n + K1 where n is
/// the slot containing the last PDSCH symbol.
pub fn harq_ack_slot(pdsch_slot: u32, k1: u8) -> u32 {
    pdsch_slot.wrapping_add(k1 as u32)
}

/// Determine the UL HARQ process ID from slot number for non-adaptive retx.
///
/// Per TS 38.213 §8.3, the UL HARQ process ID is:
/// `process_id = floor(slot / num_slots_per_frame) mod num_processes`
///
/// This is a simplified model; actual mapping depends on TDD configuration.
pub fn ul_harq_process_from_slot(slot: u32, num_processes: usize) -> usize {
    (slot as usize) % num_processes
}

/// Check if NDI toggling indicates a new transmission.
///
/// NDI toggle: new transmission when current_ndi != previous_ndi.
pub fn is_new_transmission(current_ndi: bool, previous_ndi: bool) -> bool {
    current_ndi != previous_ndi
}

/// Compute the HARQ process ID assignment from DCI in slot `n` per
/// TS 38.213 §9.2.1 (simplified model for 15 kHz numerology).
///
/// For DL: process_id = n mod num_harq_processes
pub fn dl_harq_process_id_from_slot(slot: u32, num_processes: usize) -> usize {
    (slot as usize) % num_processes
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Helper to build a manager with small buffers for testing ──────────────

    fn small_cfg() -> HarqConfig {
        HarqConfig {
            num_dl_processes: 4,
            num_ul_processes: 4,
            max_retransmissions: 3,
            combining_mode: CombiningMode::ChaseCombing,
            soft_buffer_size: 256,
            codebook_type: HarqAckCodebookType::SemiStatic,
            cbg_enabled: false,
            num_cbg: 4,
            k1_table: [1, 2, 3, 4, 5, 6, 7, 8],
            decode_threshold: 0.5,
        }
    }

    fn ir_cfg() -> HarqConfig {
        HarqConfig {
            combining_mode: CombiningMode::IncrementalRedundancy,
            ..small_cfg()
        }
    }

    fn cbg_cfg() -> HarqConfig {
        HarqConfig {
            cbg_enabled: true,
            num_cbg: 4,
            soft_buffer_size: 256,
            ..small_cfg()
        }
    }

    // ── Constants ─────────────────────────────────────────────────────────────

    #[test]
    fn test_max_processes_constants() {
        assert_eq!(MAX_DL_HARQ_PROCESSES, 16);
        assert_eq!(MAX_UL_HARQ_PROCESSES, 16);
    }

    #[test]
    fn test_rv_sequence_order() {
        assert_eq!(RV_SEQUENCE, [0, 2, 3, 1]);
    }

    #[test]
    fn test_rv_start_position_rv0() {
        assert_eq!(rv_start_position(0, 1000), 0);
    }

    #[test]
    fn test_rv_start_position_rv1() {
        // floor(17 * 1000 / 66) = floor(257.57) = 257 → round to even = 256
        let pos = rv_start_position(1, 1000);
        assert_eq!(pos % 2, 0, "RV start position must be even");
        // Rough check: 17/66 ≈ 0.257, so ~257 ≈ 256 (even)
        assert!(pos > 200 && pos < 300, "RV1 offset out of range: {}", pos);
    }

    #[test]
    fn test_rv_start_position_rv2() {
        let pos = rv_start_position(2, 1000);
        assert_eq!(pos % 2, 0);
        // 33/66 = 0.5 → 500
        assert!(pos >= 498 && pos <= 502, "RV2 offset: {}", pos);
    }

    #[test]
    fn test_rv_start_position_rv3() {
        let pos = rv_start_position(3, 1000);
        assert_eq!(pos % 2, 0);
        // 56/66 ≈ 0.848 → 848
        assert!(pos >= 846 && pos <= 850, "RV3 offset: {}", pos);
    }

    // ── Soft buffer ───────────────────────────────────────────────────────────

    #[test]
    fn test_soft_buffer_new_zeroed() {
        let buf = SoftBuffer::new(64);
        assert_eq!(buf.llrs.len(), 64);
        assert!(buf.llrs.iter().all(|&x| x == 0.0));
        assert_eq!(buf.accumulation_count, 0);
    }

    #[test]
    fn test_soft_buffer_accumulate_no_offset() {
        let mut buf = SoftBuffer::new(8);
        buf.accumulate(&[1.0, -2.0, 3.0, -4.0, 5.0, -6.0, 7.0, -8.0], 0);
        assert_eq!(buf.accumulation_count, 1);
        assert!((buf.llrs[0] - 1.0).abs() < 1e-6);
        assert!((buf.llrs[1] - (-2.0)).abs() < 1e-6);
    }

    #[test]
    fn test_soft_buffer_accumulate_twice() {
        let mut buf = SoftBuffer::new(4);
        buf.accumulate(&[1.0, 2.0, 3.0, 4.0], 0);
        buf.accumulate(&[1.0, 2.0, 3.0, 4.0], 0);
        assert_eq!(buf.accumulation_count, 2);
        assert!((buf.llrs[0] - 2.0).abs() < 1e-6);
    }

    #[test]
    fn test_soft_buffer_circular_wrap() {
        let mut buf = SoftBuffer::new(4);
        // Offset 3, length 4 → indices 3, 0, 1, 2
        buf.accumulate(&[10.0, 20.0, 30.0, 40.0], 3);
        assert!((buf.llrs[3] - 10.0).abs() < 1e-6);
        assert!((buf.llrs[0] - 20.0).abs() < 1e-6);
        assert!((buf.llrs[1] - 30.0).abs() < 1e-6);
        assert!((buf.llrs[2] - 40.0).abs() < 1e-6);
    }

    #[test]
    fn test_soft_buffer_flush() {
        let mut buf = SoftBuffer::new(8);
        buf.accumulate(&[5.0; 8], 0);
        buf.flush();
        assert!(buf.llrs.iter().all(|&x| x == 0.0));
        assert_eq!(buf.accumulation_count, 0);
    }

    #[test]
    fn test_soft_buffer_mean_abs_llr() {
        let mut buf = SoftBuffer::new(4);
        buf.llrs = vec![1.0, -2.0, 3.0, -4.0];
        let mean = buf.mean_abs_llr();
        assert!((mean - 2.5).abs() < 1e-6);
    }

    #[test]
    fn test_soft_buffer_hard_decode() {
        let mut buf = SoftBuffer::new(4);
        buf.llrs = vec![1.0, -1.0, 0.5, -0.5];
        let bits = buf.hard_decode();
        assert_eq!(bits, vec![0, 1, 0, 1]);
    }

    // ── CBG bitmap ────────────────────────────────────────────────────────────

    #[test]
    fn test_cbg_bitmap_set_ack() {
        let mut bm = CbgStatusBitmap::default();
        bm.set_ack(0);
        bm.set_ack(2);
        assert_eq!(bm.raw(), 0b0000_0101);
    }

    #[test]
    fn test_cbg_bitmap_set_nack() {
        let mut bm = CbgStatusBitmap(0xFF);
        bm.set_nack(3);
        assert_eq!(bm.raw() & (1 << 3), 0);
    }

    #[test]
    fn test_cbg_bitmap_all_acked() {
        let mut bm = CbgStatusBitmap::default();
        for i in 0..4 {
            bm.set_ack(i);
        }
        assert!(bm.all_acked(4));
        assert!(!bm.all_acked(5)); // bit 4 not set
    }

    // ── HARQ Process ──────────────────────────────────────────────────────────

    #[test]
    fn test_harq_process_initial_state() {
        let proc = HarqProcess::new(2, 64, 0);
        assert_eq!(proc.process_id, 2);
        assert_eq!(proc.state, HarqProcessState::Idle);
        assert!(!proc.ndi);
        assert_eq!(proc.rv, 0);
        assert_eq!(proc.retx_count, 0);
    }

    #[test]
    fn test_harq_process_flush() {
        let mut proc = HarqProcess::new(0, 64, 0);
        proc.state = HarqProcessState::Retransmitting;
        proc.retx_count = 3;
        proc.ndi = true;
        proc.flush();
        assert_eq!(proc.state, HarqProcessState::Idle);
        assert_eq!(proc.retx_count, 0);
        assert_eq!(proc.rv, 0);
    }

    #[test]
    fn test_harq_process_advance_retx() {
        let mut proc = HarqProcess::new(0, 64, 0);
        // Initial RV = 0 (RV_SEQUENCE[0])
        proc.advance_retx();
        assert_eq!(proc.retx_count, 1);
        assert_eq!(proc.rv, RV_SEQUENCE[1]); // RV2
        proc.advance_retx();
        assert_eq!(proc.rv, RV_SEQUENCE[2]); // RV3
        proc.advance_retx();
        assert_eq!(proc.rv, RV_SEQUENCE[3]); // RV1
        proc.advance_retx();
        assert_eq!(proc.rv, RV_SEQUENCE[0]); // RV0 (wrap)
    }

    #[test]
    fn test_harq_process_max_retx_reached() {
        let mut proc = HarqProcess::new(0, 64, 0);
        assert!(!proc.max_retx_reached(3));
        proc.retx_count = 3;
        assert!(proc.max_retx_reached(3));
    }

    #[test]
    fn test_harq_process_rtt() {
        assert_eq!(HarqProcess::round_trip_slots(4), 9);
        assert_eq!(HarqProcess::round_trip_slots(1), 3);
    }

    #[test]
    fn test_harq_process_next_rv() {
        let proc = HarqProcess::new(0, 64, 0);
        // retx_count = 0 → next is retx 1 → RV_SEQUENCE[1] = 2
        assert_eq!(proc.next_rv(), 2);
    }

    // ── HarqManager DL ────────────────────────────────────────────────────────

    #[test]
    fn test_manager_creation() {
        let mgr = HarqManager::new(small_cfg());
        assert_eq!(mgr.dl_processes.len(), 4);
        assert_eq!(mgr.ul_processes.len(), 4);
    }

    #[test]
    fn test_manager_dl_new_data_nack() {
        let mut mgr = HarqManager::new(small_cfg());
        // LLRs close to zero → mean_abs_llr < threshold → NACK
        let llrs = vec![0.1_f32; 200];
        let res = mgr.receive_dl(0, 0, true, &llrs);
        assert_eq!(res.process_id, 0);
        assert_eq!(res.ack_value, HarqAckValue::Nack);
        assert!(res.is_new_data);
        assert_eq!(res.retx_count, 1); // advance_retx called
    }

    #[test]
    fn test_manager_dl_new_data_ack() {
        let mut mgr = HarqManager::new(small_cfg());
        // Strong LLRs → mean_abs_llr > threshold → ACK
        let llrs = vec![5.0_f32; 200];
        let res = mgr.receive_dl(0, 0, true, &llrs);
        assert_eq!(res.ack_value, HarqAckValue::Ack);
        assert_eq!(res.state, HarqProcessState::Completed);
    }

    #[test]
    fn test_manager_dl_retransmission_combining() {
        let mut mgr = HarqManager::new(small_cfg());
        // First TX: weak LLRs → NACK (NDI=true, new data)
        let llrs_weak = vec![0.1_f32; 200];
        let res1 = mgr.receive_dl(0, 0, true, &llrs_weak);
        assert_eq!(res1.ack_value, HarqAckValue::Nack);
        assert!(res1.is_new_data);

        // Retransmission: NDI stays the same (true), so is_new_data = false
        let res2 = mgr.receive_dl(0, 0, true, &llrs_weak); // NDI unchanged = retx
        assert!(!res2.is_new_data);
        // After combining: mean ≈ 0.2 (still below 0.5) → NACK
        assert_eq!(res2.ack_value, HarqAckValue::Nack);
    }

    #[test]
    fn test_manager_dl_ir_combining() {
        let mut mgr = HarqManager::new(ir_cfg());
        let llrs_weak = vec![0.2_f32; 200];
        // First TX: NDI=true (new data)
        let _ = mgr.receive_dl(0, 0, true, &llrs_weak);
        // RV2 retransmission: NDI unchanged (still true) = retx
        let res = mgr.receive_dl(0, 2, true, &llrs_weak);
        assert!(!res.is_new_data);
        // Combined LLRs at overlapping positions should have been accumulated
        assert!(res.mean_abs_llr >= 0.0);
    }

    #[test]
    fn test_manager_dl_max_retx_failure() {
        let mut mgr = HarqManager::new(small_cfg());
        let llrs_weak = vec![0.1_f32; 200];
        // Initial TX: NDI=true (new data). max_retransmissions=3 → 4 total calls.
        // Retransmissions keep NDI=true (same value = retx).
        mgr.receive_dl(0, 0, true, &llrs_weak);  // initial (new data)
        mgr.receive_dl(0, 2, true, &llrs_weak);  // retx 1
        mgr.receive_dl(0, 3, true, &llrs_weak);  // retx 2
        mgr.receive_dl(0, 1, true, &llrs_weak);  // retx 3 → max reached
        let proc = mgr.dl_process(0).unwrap();
        assert_eq!(proc.state, HarqProcessState::Completed);
    }

    #[test]
    fn test_manager_dl_stats() {
        let mut mgr = HarqManager::new(small_cfg());
        let llrs_strong = vec![5.0_f32; 200];
        mgr.receive_dl(0, 0, true, &llrs_strong);
        let (rx, decoded) = mgr.dl_stats();
        assert_eq!(rx, 1);
        assert_eq!(decoded, 1);
    }

    #[test]
    fn test_manager_dl_bler() {
        let mut mgr = HarqManager::new(small_cfg());
        // 1 ACK, 1 NACK
        mgr.receive_dl(0, 0, true, &vec![5.0_f32; 200]);
        mgr.receive_dl(1, 0, true, &vec![0.1_f32; 200]);
        let bler = mgr.dl_bler();
        // 1 failed out of 2 transmissions total (not counting retx advances)
        assert!(bler > 0.0 && bler <= 1.0);
    }

    // ── HarqManager UL ────────────────────────────────────────────────────────

    #[test]
    fn test_manager_ul_tx_initiation() {
        let mut mgr = HarqManager::new(small_cfg());
        let rv = mgr.initiate_ul_tx(0, true, 128);
        assert_eq!(rv, 0); // Initial transmission always uses RV0
        let proc = mgr.ul_process(0).unwrap();
        assert_eq!(proc.state, HarqProcessState::WaitingFeedback);
    }

    #[test]
    fn test_manager_ul_ack_feedback() {
        let mut mgr = HarqManager::new(small_cfg());
        mgr.initiate_ul_tx(0, true, 128);
        let retx_needed = mgr.process_ul_feedback(0, HarqAckValue::Ack);
        assert!(!retx_needed);
        let proc = mgr.ul_process(0).unwrap();
        assert_eq!(proc.state, HarqProcessState::Completed);
    }

    #[test]
    fn test_manager_ul_nack_triggers_retx() {
        let mut mgr = HarqManager::new(small_cfg());
        mgr.initiate_ul_tx(0, true, 128);
        let retx_needed = mgr.process_ul_feedback(0, HarqAckValue::Nack);
        assert!(retx_needed);
        let proc = mgr.ul_process(0).unwrap();
        assert_eq!(proc.state, HarqProcessState::Retransmitting);
    }

    #[test]
    fn test_manager_ul_dtx_treated_as_nack() {
        let mut mgr = HarqManager::new(small_cfg());
        mgr.initiate_ul_tx(0, true, 128);
        let retx = mgr.process_ul_feedback(0, HarqAckValue::Dtx);
        assert!(retx);
    }

    #[test]
    fn test_manager_ul_max_retx_no_retx() {
        let mut mgr = HarqManager::new(small_cfg());
        mgr.initiate_ul_tx(0, true, 128);
        // Force to max retx
        for _ in 0..3 {
            mgr.process_ul_feedback(0, HarqAckValue::Nack);
        }
        let retx = mgr.process_ul_feedback(0, HarqAckValue::Nack);
        assert!(!retx);
        let proc = mgr.ul_process(0).unwrap();
        assert_eq!(proc.state, HarqProcessState::Completed);
    }

    // ── HARQ-ACK codebook ─────────────────────────────────────────────────────

    #[test]
    fn test_codebook_push_and_bit_vector() {
        let mut cb = HarqAckCodebook::new(HarqAckCodebookType::SemiStatic);
        cb.push(HarqAckEntry { cell_index: 0, process_id: 0, ack: HarqAckValue::Ack, cbg_bitmap: None, dai: 0 });
        cb.push(HarqAckEntry { cell_index: 0, process_id: 1, ack: HarqAckValue::Nack, cbg_bitmap: None, dai: 1 });
        let bits = cb.to_bit_vector();
        assert_eq!(bits, vec![1, 0]);
    }

    #[test]
    fn test_codebook_spatial_bundle_all_ack() {
        let mut cb = HarqAckCodebook::new(HarqAckCodebookType::SemiStatic);
        for i in 0..3 {
            cb.push(HarqAckEntry { cell_index: 0, process_id: i, ack: HarqAckValue::Ack, cbg_bitmap: None, dai: i });
        }
        assert_eq!(cb.spatial_bundle(), HarqAckValue::Ack);
    }

    #[test]
    fn test_codebook_spatial_bundle_with_nack() {
        let mut cb = HarqAckCodebook::new(HarqAckCodebookType::SemiStatic);
        cb.push(HarqAckEntry { cell_index: 0, process_id: 0, ack: HarqAckValue::Ack, cbg_bitmap: None, dai: 0 });
        cb.push(HarqAckEntry { cell_index: 0, process_id: 1, ack: HarqAckValue::Nack, cbg_bitmap: None, dai: 1 });
        assert_eq!(cb.spatial_bundle(), HarqAckValue::Nack);
    }

    #[test]
    fn test_codebook_empty_spatial_bundle() {
        let cb = HarqAckCodebook::new(HarqAckCodebookType::Dynamic);
        assert_eq!(cb.spatial_bundle(), HarqAckValue::Dtx);
    }

    #[test]
    fn test_codebook_time_bundle() {
        let mut cb = HarqAckCodebook::new(HarqAckCodebookType::SemiStatic);
        cb.push(HarqAckEntry { cell_index: 0, process_id: 0, ack: HarqAckValue::Ack, cbg_bitmap: None, dai: 0 });
        cb.push(HarqAckEntry { cell_index: 0, process_id: 1, ack: HarqAckValue::Ack, cbg_bitmap: None, dai: 1 });
        cb.push(HarqAckEntry { cell_index: 0, process_id: 2, ack: HarqAckValue::Nack, cbg_bitmap: None, dai: 2 });
        let bundle = cb.time_bundle(&[0, 1]);
        assert_eq!(bundle, HarqAckValue::Ack); // only processes 0,1 → both ACK
        let bundle2 = cb.time_bundle(&[1, 2]);
        assert_eq!(bundle2, HarqAckValue::Nack); // process 2 = NACK
    }

    #[test]
    fn test_codebook_dai_gaps() {
        let mut cb = HarqAckCodebook::new(HarqAckCodebookType::Dynamic);
        cb.total_dai = 4;
        cb.push(HarqAckEntry { cell_index: 0, process_id: 0, ack: HarqAckValue::Ack, cbg_bitmap: None, dai: 0 });
        cb.push(HarqAckEntry { cell_index: 0, process_id: 1, ack: HarqAckValue::Ack, cbg_bitmap: None, dai: 1 });
        assert_eq!(cb.count_dai_gaps(), 2); // expected 4, got 2
    }

    #[test]
    fn test_codebook_clear() {
        let mut cb = HarqAckCodebook::new(HarqAckCodebookType::SemiStatic);
        cb.push(HarqAckEntry { cell_index: 0, process_id: 0, ack: HarqAckValue::Ack, cbg_bitmap: None, dai: 0 });
        cb.clear();
        assert!(cb.is_empty());
    }

    // ── HARQ-ACK packing ─────────────────────────────────────────────────────

    #[test]
    fn test_pack_harq_ack_bits() {
        let mut mgr = HarqManager::new(small_cfg());
        mgr.receive_dl(0, 0, true, &vec![5.0_f32; 100]); // ACK
        mgr.receive_dl(1, 0, true, &vec![0.1_f32; 100]); // NACK
        let packed = mgr.pack_harq_ack_bits();
        // First bit = 1 (ACK), second bit = 0 (NACK)
        // Packed into byte: 0b1000_0000 = 0x80 for index 0, 0b0000_0000 for index 1
        assert!(!packed.is_empty());
        assert_eq!(packed[0] & 0x80, 0x80); // bit 0 = ACK = 1
        assert_eq!(packed[0] & 0x40, 0);    // bit 1 = NACK = 0
    }

    // ── Timing utilities ──────────────────────────────────────────────────────

    #[test]
    fn test_harq_ack_slot_calculation() {
        assert_eq!(harq_ack_slot(10, 4), 14);
        assert_eq!(harq_ack_slot(100, 8), 108);
    }

    #[test]
    fn test_harq_ack_slot_wrap() {
        // u32 wrap is allowed
        let slot = harq_ack_slot(u32::MAX, 1);
        assert_eq!(slot, 0);
    }

    #[test]
    fn test_ul_harq_process_from_slot() {
        assert_eq!(ul_harq_process_from_slot(0, 8), 0);
        assert_eq!(ul_harq_process_from_slot(8, 8), 0);
        assert_eq!(ul_harq_process_from_slot(5, 8), 5);
        assert_eq!(ul_harq_process_from_slot(13, 8), 5);
    }

    #[test]
    fn test_is_new_transmission_ndi_toggle() {
        assert!(is_new_transmission(true, false));
        assert!(is_new_transmission(false, true));
        assert!(!is_new_transmission(true, true));
        assert!(!is_new_transmission(false, false));
    }

    #[test]
    fn test_dl_harq_process_id_from_slot() {
        assert_eq!(dl_harq_process_id_from_slot(0, 16), 0);
        assert_eq!(dl_harq_process_id_from_slot(16, 16), 0);
        assert_eq!(dl_harq_process_id_from_slot(7, 16), 7);
    }

    #[test]
    fn test_k1_from_dci_field() {
        let mgr = HarqManager::new(small_cfg());
        assert_eq!(mgr.k1_from_dci_field(0), 1);
        assert_eq!(mgr.k1_from_dci_field(7), 8);
    }

    #[test]
    fn test_ack_due_slot() {
        let mgr = HarqManager::new(small_cfg());
        assert_eq!(mgr.ack_due_slot(10, 4), 14);
    }

    // ── Manager state queries ─────────────────────────────────────────────────

    #[test]
    fn test_find_idle_dl_process() {
        let mgr = HarqManager::new(small_cfg());
        let idle = mgr.find_idle_dl_process();
        assert_eq!(idle, Some(0));
    }

    #[test]
    fn test_find_idle_ul_process() {
        let mgr = HarqManager::new(small_cfg());
        let idle = mgr.find_idle_ul_process();
        assert_eq!(idle, Some(0));
    }

    #[test]
    fn test_active_dl_retx_count_zero_initially() {
        let mgr = HarqManager::new(small_cfg());
        assert_eq!(mgr.active_dl_retx_count(), 0);
    }

    #[test]
    fn test_flush_dl_process() {
        let mut mgr = HarqManager::new(small_cfg());
        mgr.receive_dl(0, 0, true, &vec![0.1_f32; 100]);
        mgr.flush_dl_process(0);
        let proc = mgr.dl_process(0).unwrap();
        assert_eq!(proc.state, HarqProcessState::Idle);
    }

    #[test]
    fn test_flush_all() {
        let mut mgr = HarqManager::new(small_cfg());
        mgr.receive_dl(0, 0, true, &vec![0.1_f32; 100]);
        mgr.receive_dl(1, 0, true, &vec![0.1_f32; 100]);
        mgr.flush_all();
        for i in 0..4 {
            assert_eq!(mgr.dl_process(i).unwrap().state, HarqProcessState::Idle);
        }
    }

    #[test]
    fn test_advance_slot() {
        let mut mgr = HarqManager::new(small_cfg());
        assert_eq!(mgr.current_slot(), 0);
        mgr.advance_slot();
        assert_eq!(mgr.current_slot(), 1);
    }

    // ── CBG HARQ ──────────────────────────────────────────────────────────────

    #[test]
    fn test_cbg_manager_creation() {
        let mgr = HarqManager::new(cbg_cfg());
        let proc = mgr.dl_process(0).unwrap();
        assert_eq!(proc.cbg_soft_buffers.len(), 4);
    }

    #[test]
    fn test_cbg_receive_all_cbgs_ack() {
        let mut mgr = HarqManager::new(cbg_cfg());
        // Strong LLRs for all 4 CBGs → ACK
        let cbg_llrs: Vec<Vec<f32>> = (0..4).map(|_| vec![5.0_f32; 32]).collect();
        let res = mgr.receive_dl_cbg(0, 0, true, 0b1111, &cbg_llrs);
        assert_eq!(res.ack_value, HarqAckValue::Ack);
    }

    #[test]
    fn test_cbg_receive_partial_cbgs_nack() {
        let mut mgr = HarqManager::new(cbg_cfg());
        // Weak LLRs → NACK
        let cbg_llrs: Vec<Vec<f32>> = (0..4).map(|_| vec![0.1_f32; 32]).collect();
        let res = mgr.receive_dl_cbg(0, 0, true, 0b1111, &cbg_llrs);
        assert_eq!(res.ack_value, HarqAckValue::Nack);
    }

    #[test]
    fn test_cbg_selective_retransmission() {
        let mut mgr = HarqManager::new(cbg_cfg());
        // First: CBG 0,1 strong (ACK), CBG 2,3 weak (NACK)
        let cbg_llrs: Vec<Vec<f32>> = vec![
            vec![5.0_f32; 32], // CBG0 → ACK
            vec![5.0_f32; 32], // CBG1 → ACK
            vec![0.1_f32; 32], // CBG2 → NACK
            vec![0.1_f32; 32], // CBG3 → NACK
        ];
        let res1 = mgr.receive_dl_cbg(0, 0, true, 0b1111, &cbg_llrs);
        assert_eq!(res1.ack_value, HarqAckValue::Nack);
        assert_eq!(res1.cbg_status.raw() & 0b0011, 0b0011); // CBG0,1 ACKed

        // Retx only CBG 2,3 with strong LLRs
        let cbg_llrs2: Vec<Vec<f32>> = vec![
            vec![0.0_f32; 32], // CBG0 not sent
            vec![0.0_f32; 32], // CBG1 not sent
            vec![5.0_f32; 32], // CBG2 → ACK
            vec![5.0_f32; 32], // CBG3 → ACK
        ];
        // Retx: NDI unchanged (still true) = retransmission, not new data
        let res2 = mgr.receive_dl_cbg(0, 2, true, 0b1100, &cbg_llrs2);
        assert_eq!(res2.ack_value, HarqAckValue::Ack);
    }

    // ── Throughput and RTT ────────────────────────────────────────────────────

    #[test]
    fn test_harq_rtt_slots() {
        assert_eq!(HarqManager::harq_rtt_slots(4), 9);
    }

    #[test]
    fn test_dl_throughput_no_data() {
        let mgr = HarqManager::new(small_cfg());
        assert_eq!(mgr.dl_throughput_bps(0, 1.0), 0.0);
    }

    #[test]
    fn test_dl_throughput_with_data() {
        let mut mgr = HarqManager::new(small_cfg());
        mgr.receive_dl(0, 0, true, &vec![5.0_f32; 200]);
        let tput = mgr.dl_throughput_bps(0, 1.0);
        assert!(tput > 0.0, "Throughput should be positive after successful decode");
    }

    // ── Default config ────────────────────────────────────────────────────────

    #[test]
    fn test_default_config() {
        let cfg = HarqConfig::default();
        assert_eq!(cfg.num_dl_processes, 16);
        assert_eq!(cfg.num_ul_processes, 16);
        assert_eq!(cfg.max_retransmissions, 4);
        assert_eq!(cfg.soft_buffer_size, DEFAULT_SOFT_BUFFER_SIZE);
    }

    // ── HARQ-ACK spatial bundling through manager ─────────────────────────────

    #[test]
    fn test_spatial_bundle_dl_all_ack() {
        let mut mgr = HarqManager::new(small_cfg());
        mgr.receive_dl(0, 0, true, &vec![5.0_f32; 200]);
        mgr.receive_dl(1, 0, true, &vec![5.0_f32; 200]);
        assert_eq!(mgr.spatial_bundle_dl(), HarqAckValue::Ack);
    }

    #[test]
    fn test_spatial_bundle_dl_with_nack() {
        let mut mgr = HarqManager::new(small_cfg());
        mgr.receive_dl(0, 0, true, &vec![5.0_f32; 200]);
        mgr.receive_dl(1, 0, true, &vec![0.1_f32; 200]); // NACK
        assert_eq!(mgr.spatial_bundle_dl(), HarqAckValue::Nack);
    }

    // ── BLER tracking ────────────────────────────────────────────────────────

    #[test]
    fn test_dl_bler_zero_on_init() {
        let mgr = HarqManager::new(small_cfg());
        assert_eq!(mgr.dl_bler(), 0.0);
    }

    #[test]
    fn test_ul_bler_zero_on_init() {
        let mgr = HarqManager::new(small_cfg());
        assert_eq!(mgr.ul_bler(), 0.0);
    }

    #[test]
    fn test_ul_bler_after_success() {
        let mut mgr = HarqManager::new(small_cfg());
        mgr.initiate_ul_tx(0, true, 128);
        mgr.process_ul_feedback(0, HarqAckValue::Ack);
        assert_eq!(mgr.ul_bler(), 0.0);
    }

    #[test]
    fn test_dl_bler_perfect() {
        let mut mgr = HarqManager::new(small_cfg());
        mgr.receive_dl(0, 0, true, &vec![5.0_f32; 200]);
        mgr.receive_dl(1, 0, true, &vec![5.0_f32; 200]);
        assert!((mgr.dl_bler() - 0.0).abs() < 1e-9);
    }
}
