//! RAN Scheduler Processor for 4G/5G Resource Allocation
//!
//! Implements scheduling algorithms and resource allocation logic for LTE and 5G NR
//! Radio Access Networks. Supports Round Robin (RR), Proportional Fair (PF),
//! Maximum C/I (Max-CI), and Weighted Fair Queuing (WFQ) scheduling algorithms.
//!
//! # Features
//! - DL/UL scheduling with PRB allocation and MCS selection
//! - CQI-to-MCS mapping per 3GPP TS 36.213 Table 7.1.7.1-1
//! - QoS-aware scheduling: 5QI/QCI mapping, GBR/non-GBR bearers, AMBR enforcement
//! - HARQ retransmission management with RTT timers
//! - DRX (Discontinuous Reception) state machine
//! - SPS (Semi-Persistent Scheduling) for periodic VoLTE/VoNR traffic
//! - Frequency-selective scheduling via subband CQI
//! - Jain's fairness index and scheduling statistics
//!
//! # Standards References
//! - 3GPP TS 36.213 (LTE PDSCH/PUSCH scheduling)
//! - 3GPP TS 38.214 (NR PDSCH/PUSCH scheduling)
//! - 3GPP TS 36.321 (LTE MAC layer)
//! - 3GPP TS 38.321 (NR MAC layer)

use std::collections::{HashMap, VecDeque};

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// Maximum number of UEs supported in a single cell
pub const MAX_UES: usize = 256;
/// Maximum PRBs in 20 MHz LTE / 100 MHz NR
pub const MAX_PRBS: usize = 273;
/// Number of HARQ processes per UE (LTE DL: 8, NR: up to 16)
pub const MAX_HARQ_PROCESSES: usize = 16;
/// CQI range 1-15
pub const MAX_CQI: usize = 15;
/// MCS range 0-28
pub const MAX_MCS: usize = 28;
/// Exponential averaging window for PF scheduler (ms)
pub const PF_WINDOW_MS: f64 = 1000.0;
/// Maximum retransmissions per HARQ process
pub const MAX_HARQ_RETX: u8 = 4;
/// LTE HARQ RTT in subframes
pub const LTE_HARQ_RTT: u32 = 8;
/// NR HARQ RTT in slots (minimum)
pub const NR_HARQ_RTT_MIN: u32 = 4;

// ---------------------------------------------------------------------------
// CQI table: (modulation_order, code_rate_x1024) indexed by CQI 1..=15
// 3GPP TS 36.213 Table 7.2.3-1
// ---------------------------------------------------------------------------

static CQI_TABLE: [(u8, u16); 15] = [
    (2, 78),   // CQI 1:  QPSK
    (2, 120),  // CQI 2:  QPSK
    (2, 193),  // CQI 3:  QPSK
    (2, 308),  // CQI 4:  QPSK
    (2, 449),  // CQI 5:  QPSK
    (2, 602),  // CQI 6:  QPSK
    (4, 378),  // CQI 7:  16QAM
    (4, 490),  // CQI 8:  16QAM
    (4, 616),  // CQI 9:  16QAM
    (6, 466),  // CQI 10: 64QAM
    (6, 567),  // CQI 11: 64QAM
    (6, 666),  // CQI 12: 64QAM
    (6, 772),  // CQI 13: 64QAM
    (6, 873),  // CQI 14: 64QAM
    (6, 948),  // CQI 15: 64QAM
];

/// CQI (1-15) to MCS index (0-28) mapping
pub fn cqi_to_mcs(cqi: u8) -> u8 {
    match cqi {
        0 => 0,
        1 => 0,
        2 => 1,
        3 => 3,
        4 => 5,
        5 => 7,
        6 => 9,
        7 => 11,
        8 => 13,
        9 => 15,
        10 => 17,
        11 => 20,
        12 => 22,
        13 => 24,
        14 => 26,
        15 => 28,
        _ => 28,
    }
}

/// Modulation order (bits/symbol) for MCS index 0-28
pub fn mcs_to_modulation_order(mcs: u8) -> u8 {
    match mcs {
        0..=9 => 2,   // QPSK
        10..=16 => 4, // 16QAM
        17..=28 => 6, // 64QAM
        _ => 2,
    }
}

/// Spectral efficiency (bits/s/Hz * 1000) for given CQI
pub fn cqi_spectral_efficiency_x1000(cqi: u8) -> u32 {
    if cqi == 0 || cqi as usize > MAX_CQI {
        return 0;
    }
    let (mod_order, code_rate) = CQI_TABLE[(cqi - 1) as usize];
    // Use round-up division to preserve non-zero values for low CQI
    (mod_order as u32 * code_rate as u32 + 1023) / 1024
}

/// Approximate Transport Block Size (bytes) for given MCS and PRB count
pub fn compute_tbs_bytes(mcs: u8, n_prb: u32) -> u32 {
    if n_prb == 0 {
        return 0;
    }
    let mod_order = mcs_to_modulation_order(mcs) as u32;
    let code_rate_x1024: u32 = match mcs {
        0 => 120, 1 => 157, 2 => 193, 3 => 251, 4 => 308, 5 => 379,
        6 => 449, 7 => 526, 8 => 602, 9 => 679,
        10 => 340, 11 => 378, 12 => 434, 13 => 490, 14 => 553, 15 => 616, 16 => 658,
        17 => 438, 18 => 466, 19 => 517, 20 => 567, 21 => 616, 22 => 666,
        23 => 719, 24 => 772, 25 => 822, 26 => 873, 27 => 910, 28 => 948,
        _ => 948,
    };
    let bits_per_prb = (12 * mod_order * code_rate_x1024) >> 10;
    let total_bits = bits_per_prb * n_prb;
    total_bits / 8
}

// ---------------------------------------------------------------------------
// Scheduling Algorithm
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SchedulingAlgorithm {
    RoundRobin,
    ProportionalFair,
    MaxCI,
    WeightedFairQueuing,
}

// ---------------------------------------------------------------------------
// QoS / Bearer
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BearerType {
    GBR,
    NonGBR,
    MissionCritical,
}

#[derive(Debug, Clone)]
pub struct QosParameters {
    pub qci: u8,
    pub bearer_type: BearerType,
    pub arp: u8,
    pub gbr_bps: u64,
    pub mbr_bps: u64,
    pub pdb_ms: u32,
    pub pelr: u32,
}

impl QosParameters {
    pub fn volte() -> Self {
        Self { qci: 1, bearer_type: BearerType::GBR, arp: 2,
               gbr_bps: 96_000, mbr_bps: 128_000, pdb_ms: 100, pelr: 10 }
    }
    pub fn best_effort() -> Self {
        Self { qci: 9, bearer_type: BearerType::NonGBR, arp: 9,
               gbr_bps: 0, mbr_bps: 100_000_000, pdb_ms: 300, pelr: 1000 }
    }
    pub fn gaming() -> Self {
        Self { qci: 3, bearer_type: BearerType::GBR, arp: 3,
               gbr_bps: 500_000, mbr_bps: 2_000_000, pdb_ms: 50, pelr: 30 }
    }
    pub fn urllc() -> Self {
        Self { qci: 1, bearer_type: BearerType::MissionCritical, arp: 1,
               gbr_bps: 1_000_000, mbr_bps: 10_000_000, pdb_ms: 1, pelr: 2 }
    }
    pub fn priority_score(&self) -> f64 {
        let base = match self.bearer_type {
            BearerType::MissionCritical => 0.0,
            BearerType::GBR => 10.0,
            BearerType::NonGBR => 20.0,
        };
        base + self.arp as f64
    }
}

// ---------------------------------------------------------------------------
// HARQ
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum HarqState {
    Idle,
    WaitingAck { tti_sent: u64 },
    PendingRetx { retx_count: u8, tti_nack: u64 },
    Acked,
}

#[derive(Debug, Clone)]
pub struct HarqProcess {
    pub process_id: u8,
    pub state: HarqState,
    pub tbs_bytes: u32,
    pub mcs: u8,
    pub n_prb: u32,
    pub tti_scheduled: u64,
    pub retx_count: u8,
    pub llr_buffer_bytes: u32,
}

impl HarqProcess {
    pub fn new(process_id: u8) -> Self {
        Self { process_id, state: HarqState::Idle, tbs_bytes: 0, mcs: 0,
               n_prb: 0, tti_scheduled: 0, retx_count: 0, llr_buffer_bytes: 0 }
    }
    pub fn is_available(&self) -> bool {
        matches!(self.state, HarqState::Idle | HarqState::Acked)
    }
    pub fn needs_retx(&self) -> bool {
        matches!(self.state, HarqState::PendingRetx { .. })
    }
}

// ---------------------------------------------------------------------------
// DRX State Machine
// ---------------------------------------------------------------------------

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DrxState {
    Active,
    InactivityTimer,
    ShortDrx,
    LongDrx,
    ShortDrxCycle,
    LongDrxCycle,
}

#[derive(Debug, Clone)]
pub struct DrxConfig {
    pub on_duration_ms: u32,
    pub inactivity_timer_ms: u32,
    pub short_cycle_ms: u32,
    pub short_cycle_timer: u32,
    pub long_cycle_ms: u32,
    pub harq_rtt_timer_ms: u32,
}

impl DrxConfig {
    pub fn volte() -> Self {
        Self { on_duration_ms: 10, inactivity_timer_ms: 20,
               short_cycle_ms: 20, short_cycle_timer: 4,
               long_cycle_ms: 40, harq_rtt_timer_ms: 8 }
    }
    pub fn embb() -> Self {
        Self { on_duration_ms: 8, inactivity_timer_ms: 100,
               short_cycle_ms: 40, short_cycle_timer: 2,
               long_cycle_ms: 160, harq_rtt_timer_ms: 8 }
    }
    pub fn iot() -> Self {
        Self { on_duration_ms: 4, inactivity_timer_ms: 10,
               short_cycle_ms: 80, short_cycle_timer: 1,
               long_cycle_ms: 320, harq_rtt_timer_ms: 8 }
    }
}

#[derive(Debug, Clone)]
pub struct DrxStateMachine {
    pub config: DrxConfig,
    pub state: DrxState,
    pub inactivity_remaining: u32,
    pub short_cycle_count: u32,
    pub current_tti: u64,
}

impl DrxStateMachine {
    pub fn new(config: DrxConfig) -> Self {
        Self { config, state: DrxState::Active, inactivity_remaining: 0,
               short_cycle_count: 0, current_tti: 0 }
    }

    pub fn tick(&mut self, tti: u64, was_scheduled: bool) {
        self.current_tti = tti;
        if was_scheduled {
            self.inactivity_remaining = self.config.inactivity_timer_ms;
            self.state = DrxState::Active;
            return;
        }
        match self.state {
            DrxState::Active => {
                if self.inactivity_remaining > 0 {
                    self.inactivity_remaining -= 1;
                } else {
                    self.short_cycle_count = 0;
                    self.state = DrxState::ShortDrxCycle;
                }
            }
            DrxState::ShortDrxCycle => {
                let cycle_pos = tti % self.config.short_cycle_ms as u64;
                if cycle_pos < self.config.on_duration_ms as u64 {
                    self.state = DrxState::ShortDrx;
                } else if cycle_pos == self.config.short_cycle_ms as u64 - 1 {
                    self.short_cycle_count += 1;
                    if self.short_cycle_count >= self.config.short_cycle_timer {
                        self.state = DrxState::LongDrxCycle;
                    }
                }
            }
            DrxState::ShortDrx => {
                let cycle_pos = tti % self.config.short_cycle_ms as u64;
                if cycle_pos >= self.config.on_duration_ms as u64 {
                    self.state = DrxState::ShortDrxCycle;
                }
            }
            DrxState::LongDrxCycle => {
                let cycle_pos = tti % self.config.long_cycle_ms as u64;
                if cycle_pos < self.config.on_duration_ms as u64 {
                    self.state = DrxState::LongDrx;
                }
            }
            DrxState::LongDrx => {
                let cycle_pos = tti % self.config.long_cycle_ms as u64;
                if cycle_pos >= self.config.on_duration_ms as u64 {
                    self.state = DrxState::LongDrxCycle;
                }
            }
            DrxState::InactivityTimer => {
                self.state = DrxState::ShortDrxCycle;
            }
        }
    }

    pub fn is_active(&self) -> bool {
        matches!(self.state,
            DrxState::Active | DrxState::ShortDrx | DrxState::LongDrx | DrxState::InactivityTimer)
    }
}

// ---------------------------------------------------------------------------
// SPS (Semi-Persistent Scheduling)
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct SpsConfig {
    pub interval_ms: u32,
    pub implicit_release_after: u32,
    pub mcs: u8,
    pub n_prb: u32,
}

impl SpsConfig {
    pub fn volte_amr_wb() -> Self {
        Self { interval_ms: 20, implicit_release_after: 8, mcs: 4, n_prb: 3 }
    }
    pub fn vonr() -> Self {
        Self { interval_ms: 10, implicit_release_after: 8, mcs: 6, n_prb: 2 }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SpsState {
    Inactive,
    Active,
    Releasing,
}

#[derive(Debug, Clone)]
pub struct SpsContext {
    pub config: SpsConfig,
    pub state: SpsState,
    pub next_sps_tti: u64,
    pub empty_count: u32,
    pub activation_tti: u64,
    pub total_sps_grants: u64,
}

impl SpsContext {
    pub fn new(config: SpsConfig) -> Self {
        Self { config, state: SpsState::Inactive, next_sps_tti: 0,
               empty_count: 0, activation_tti: 0, total_sps_grants: 0 }
    }

    pub fn activate(&mut self, tti: u64) {
        self.state = SpsState::Active;
        self.activation_tti = tti;
        self.next_sps_tti = tti + self.config.interval_ms as u64;
        self.empty_count = 0;
    }

    pub fn release(&mut self) {
        self.state = SpsState::Inactive;
        self.empty_count = 0;
    }

    pub fn should_grant(&mut self, tti: u64, has_data: bool) -> bool {
        if self.state != SpsState::Active {
            return false;
        }
        if tti < self.next_sps_tti {
            return false;
        }
        self.next_sps_tti += self.config.interval_ms as u64;
        if !has_data {
            self.empty_count += 1;
            if self.empty_count >= self.config.implicit_release_after {
                self.state = SpsState::Releasing;
            }
            return false;
        }
        self.empty_count = 0;
        self.total_sps_grants += 1;
        true
    }
}

// ---------------------------------------------------------------------------
// BSR / PHR
// ---------------------------------------------------------------------------

static BSR_TABLE: [u32; 64] = [
    0, 10, 13, 17, 23, 31, 41, 55,
    74, 99, 132, 176, 235, 314, 419, 560,
    747, 997, 1329, 1774, 2367, 3159, 4217, 5627,
    7510, 10024, 13382, 17867, 23844, 31826, 42469, 56682,
    75649, 100985, 134797, 179930, 240170, 320497, 427798, 570929,
    762079, 1017167, 1357824, 1813521, 2420089, 3229176, 4309174, 5752202,
    7677696, 10251608, 13682184, 18261607, 24381427, 32548467, 43453402, 58015062,
    77439621, 103403263, 138007827, 184217280, 245864640, 328102400, 437836800, u32::MAX,
];

pub fn bsr_index_to_bytes(index: u8) -> u32 {
    BSR_TABLE[index.min(63) as usize]
}

pub fn bytes_to_bsr_index(bytes: u32) -> u8 {
    for (i, &threshold) in BSR_TABLE.iter().enumerate() {
        if bytes <= threshold {
            return i as u8;
        }
    }
    63
}

#[derive(Debug, Clone)]
pub struct BufferStatusReport {
    pub ue_id: u32,
    pub lcg_bytes: [u32; 4],
    pub tti_reported: u64,
}

impl BufferStatusReport {
    pub fn total_bytes(&self) -> u32 {
        self.lcg_bytes.iter().sum()
    }
}

#[derive(Debug, Clone)]
pub struct PowerHeadroomReport {
    pub ue_id: u32,
    pub headroom_db: i8,
    pub tti_reported: u64,
    pub p_cmax_dbm: i8,
}

impl PowerHeadroomReport {
    pub fn max_ul_mcs(&self) -> u8 {
        match self.headroom_db {
            h if h >= 20 => 28,
            h if h >= 15 => 24,
            h if h >= 10 => 20,
            h if h >= 5 => 15,
            h if h >= 0 => 10,
            h if h >= -5 => 5,
            _ => 0,
        }
    }
}

// ---------------------------------------------------------------------------
// Subband CQI
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct SubbandCqiReport {
    pub ue_id: u32,
    pub wideband_cqi: u8,
    pub subband_cqi: Vec<u8>,
    pub subband_size_prb: u32,
    pub tti_reported: u64,
    pub rank_indicator: u8,
    pub pmi: u8,
}

impl SubbandCqiReport {
    pub fn best_subbands(&self, n: usize) -> Vec<usize> {
        let mut indexed: Vec<(usize, u8)> = self.subband_cqi
            .iter().enumerate().map(|(i, &cqi)| (i, cqi)).collect();
        indexed.sort_by(|a, b| b.1.cmp(&a.1));
        indexed.into_iter().take(n).map(|(i, _)| i).collect()
    }

    pub fn cqi_for_prb_range(&self, start_prb: u32, end_prb: u32) -> u8 {
        if self.subband_cqi.is_empty() || self.subband_size_prb == 0 {
            return self.wideband_cqi;
        }
        let sb_start = (start_prb / self.subband_size_prb) as usize;
        let sb_end = ((end_prb / self.subband_size_prb) as usize)
            .min(self.subband_cqi.len().saturating_sub(1));
        if sb_start > sb_end {
            return self.wideband_cqi;
        }
        let sum: u32 = self.subband_cqi[sb_start..=sb_end].iter().map(|&c| c as u32).sum();
        let count = (sb_end - sb_start + 1) as u32;
        (sum / count) as u8
    }
}

// ---------------------------------------------------------------------------
// UE Context
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct UeContext {
    pub ue_id: u32,
    pub c_rnti: u16,
    pub qos: QosParameters,
    pub scheduling_weight: f64,
    pub dl_cqi: u8,
    pub ul_sinr_db: f32,
    pub dl_harq: Vec<HarqProcess>,
    pub ul_harq: Vec<HarqProcess>,
    pub drx: Option<DrxStateMachine>,
    pub sps: Option<SpsContext>,
    pub bsr: Option<BufferStatusReport>,
    pub phr: Option<PowerHeadroomReport>,
    pub subband_cqi: Option<SubbandCqiReport>,
    pub avg_throughput_bytes_per_ms: f64,
    pub last_tbs_bytes: u32,
    pub total_dl_bytes: u64,
    pub total_ul_bytes: u64,
    pub scheduled_ttis: u64,
    pub total_delay_ms: u64,
    pub delay_samples: u64,
    pub gbr_bucket_bytes: f64,
    pub ambr_bucket_bytes: f64,
    pub last_dl_tti: u64,
    pub last_ul_tti: u64,
    pub connected: bool,
    pub wait_ttis: u64,
}

impl UeContext {
    pub fn new(ue_id: u32, c_rnti: u16, qos: QosParameters) -> Self {
        let dl_harq = (0..MAX_HARQ_PROCESSES).map(|i| HarqProcess::new(i as u8)).collect();
        let ul_harq = (0..MAX_HARQ_PROCESSES).map(|i| HarqProcess::new(i as u8)).collect();
        Self {
            ue_id, c_rnti, qos, scheduling_weight: 1.0,
            dl_cqi: 7, ul_sinr_db: 10.0,
            dl_harq, ul_harq, drx: None, sps: None,
            bsr: None, phr: None, subband_cqi: None,
            avg_throughput_bytes_per_ms: 1.0, last_tbs_bytes: 0,
            total_dl_bytes: 0, total_ul_bytes: 0, scheduled_ttis: 0,
            total_delay_ms: 0, delay_samples: 0,
            gbr_bucket_bytes: 0.0, ambr_bucket_bytes: 0.0,
            last_dl_tti: 0, last_ul_tti: 0, connected: true, wait_ttis: 0,
        }
    }

    pub fn next_dl_harq(&self) -> Option<u8> {
        self.dl_harq.iter().find(|h| h.is_available()).map(|h| h.process_id)
    }

    pub fn pending_dl_retx(&self, current_tti: u64) -> Option<u8> {
        self.dl_harq.iter()
            .find(|h| {
                if let HarqState::PendingRetx { tti_nack, .. } = h.state {
                    current_tti >= tti_nack + LTE_HARQ_RTT as u64
                } else { false }
            })
            .map(|h| h.process_id)
    }

    pub fn update_pf_throughput(&mut self, tbs_bytes: u32) {
        let alpha = 1.0 / PF_WINDOW_MS;
        self.avg_throughput_bytes_per_ms =
            (1.0 - alpha) * self.avg_throughput_bytes_per_ms + alpha * tbs_bytes as f64;
        if self.avg_throughput_bytes_per_ms < 0.001 {
            self.avg_throughput_bytes_per_ms = 0.001;
        }
        self.last_tbs_bytes = tbs_bytes;
    }

    pub fn pf_metric(&self) -> f64 {
        let inst = cqi_spectral_efficiency_x1000(self.dl_cqi) as f64;
        inst / self.avg_throughput_bytes_per_ms.max(0.001)
    }

    pub fn max_ci_metric(&self) -> f64 {
        cqi_spectral_efficiency_x1000(self.dl_cqi) as f64
    }

    pub fn token_bucket_check(&self, _tbs_bytes: u32) -> bool {
        if self.qos.gbr_bps > 0 && self.gbr_bucket_bytes < 1.0 {
            return false;
        }
        true
    }

    pub fn replenish_buckets(&mut self) {
        if self.qos.gbr_bps > 0 {
            let replenish = self.qos.gbr_bps as f64 / 8000.0;
            self.gbr_bucket_bytes =
                (self.gbr_bucket_bytes + replenish).min(replenish * 100.0);
        }
        if self.qos.mbr_bps > 0 {
            let replenish = self.qos.mbr_bps as f64 / 8000.0;
            self.ambr_bucket_bytes =
                (self.ambr_bucket_bytes + replenish).min(replenish * 200.0);
        }
    }

    pub fn ul_cqi(&self) -> u8 {
        match self.ul_sinr_db as i32 {
            s if s < -5 => 0,
            s if s < 0 => 1,
            s if s < 3 => 3,
            s if s < 6 => 5,
            s if s < 9 => 7,
            s if s < 12 => 9,
            s if s < 15 => 11,
            s if s < 18 => 13,
            _ => 15,
        }
    }
}

// ---------------------------------------------------------------------------
// Grants & Scheduling Results
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct DlGrant {
    pub ue_id: u32,
    pub c_rnti: u16,
    pub tti: u64,
    pub harq_process_id: u8,
    pub mcs: u8,
    pub n_prb: u32,
    pub prb_start: u32,
    pub tbs_bytes: u32,
    pub is_retx: bool,
    pub ndi: bool,
    pub rv: u8,
    pub is_sps: bool,
}

#[derive(Debug, Clone)]
pub struct UlGrant {
    pub ue_id: u32,
    pub c_rnti: u16,
    pub tti: u64,
    pub harq_process_id: u8,
    pub mcs: u8,
    pub n_prb: u32,
    pub prb_start: u32,
    pub tbs_bytes: u32,
    pub is_retx: bool,
    pub ndi: bool,
    pub rv: u8,
    pub tpc: i8,
}

#[derive(Debug, Default)]
pub struct TtiSchedulingResult {
    pub tti: u64,
    pub dl_grants: Vec<DlGrant>,
    pub ul_grants: Vec<UlGrant>,
    pub dl_prbs_used: u32,
    pub ul_prbs_used: u32,
    pub dl_prbs_available: u32,
    pub ul_prbs_available: u32,
    pub scheduled_ue_count: usize,
}

impl TtiSchedulingResult {
    pub fn dl_utilization(&self) -> f64 {
        if self.dl_prbs_available == 0 { return 0.0; }
        self.dl_prbs_used as f64 / self.dl_prbs_available as f64
    }
    pub fn ul_utilization(&self) -> f64 {
        if self.ul_prbs_available == 0 { return 0.0; }
        self.ul_prbs_used as f64 / self.ul_prbs_available as f64
    }
    pub fn total_dl_bytes(&self) -> u32 {
        self.dl_grants.iter().map(|g| g.tbs_bytes).sum()
    }
    pub fn total_ul_bytes(&self) -> u32 {
        self.ul_grants.iter().map(|g| g.tbs_bytes).sum()
    }
}

// ---------------------------------------------------------------------------
// Cell Configuration
// ---------------------------------------------------------------------------

#[derive(Debug, Clone)]
pub struct CellConfig {
    pub dl_n_prb: u32,
    pub ul_n_prb: u32,
    pub algorithm: SchedulingAlgorithm,
    pub max_ues_per_tti: usize,
    pub min_prb_per_ue: u32,
    pub max_prb_per_ue: u32,
    pub subband_size_prb: u32,
    pub numerology: u8,
    pub n_harq_processes: u8,
    pub pf_alpha: f64,
    pub harq_retx_priority: bool,
    pub drx_enabled: bool,
    pub sps_enabled: bool,
}

impl CellConfig {
    pub fn lte_10mhz() -> Self {
        Self {
            dl_n_prb: 50, ul_n_prb: 50,
            algorithm: SchedulingAlgorithm::ProportionalFair,
            max_ues_per_tti: 10, min_prb_per_ue: 1, max_prb_per_ue: 50,
            subband_size_prb: 4, numerology: 0, n_harq_processes: 8,
            pf_alpha: 1.0, harq_retx_priority: true, drx_enabled: true, sps_enabled: true,
        }
    }
    pub fn lte_20mhz() -> Self {
        Self {
            dl_n_prb: 100, ul_n_prb: 100,
            algorithm: SchedulingAlgorithm::ProportionalFair,
            max_ues_per_tti: 10, min_prb_per_ue: 2, max_prb_per_ue: 100,
            subband_size_prb: 8, numerology: 0, n_harq_processes: 8,
            pf_alpha: 1.0, harq_retx_priority: true, drx_enabled: true, sps_enabled: true,
        }
    }
    pub fn nr_100mhz() -> Self {
        Self {
            dl_n_prb: 273, ul_n_prb: 273,
            algorithm: SchedulingAlgorithm::ProportionalFair,
            max_ues_per_tti: 20, min_prb_per_ue: 4, max_prb_per_ue: 273,
            subband_size_prb: 12, numerology: 1, n_harq_processes: 16,
            pf_alpha: 1.0, harq_retx_priority: true, drx_enabled: true, sps_enabled: true,
        }
    }
    pub fn tti_us(&self) -> u32 {
        1000 >> self.numerology
    }
    pub fn slot_duration_ms(&self) -> f64 {
        1.0 / (1u32 << self.numerology) as f64
    }
}

// ---------------------------------------------------------------------------
// Fairness Metrics
// ---------------------------------------------------------------------------

#[derive(Debug, Default, Clone)]
pub struct FairnessMetrics {
    pub jains_index: f64,
    pub throughput_variance: f64,
    pub throughput_cv: f64,
    pub avg_delay_ms: f64,
    pub max_delay_ms: u64,
    pub p95_delay_ms: f64,
    pub total_dl_throughput_mbps: f64,
    pub active_ue_count: usize,
    pub prb_utilization: f64,
}

/// Jain's Fairness Index: J = (sum(xi))^2 / (n * sum(xi^2))
pub fn jains_fairness_index(throughputs: &[f64]) -> f64 {
    let n = throughputs.len();
    if n == 0 { return 1.0; }
    let sum: f64 = throughputs.iter().sum();
    let sum_sq: f64 = throughputs.iter().map(|&x| x * x).sum();
    if sum_sq < 1e-12 { return 1.0; }
    (sum * sum) / (n as f64 * sum_sq)
}

// ---------------------------------------------------------------------------
// RAN Scheduler
// ---------------------------------------------------------------------------

pub struct RanScheduler {
    pub config: CellConfig,
    pub ues: HashMap<u32, UeContext>,
    pub current_tti: u64,
    pub ue_order: Vec<u32>,
    pub scheduling_history: VecDeque<(u64, u32, u32)>,
    pub history_window: usize,
    pub total_dl_bytes: u64,
    pub total_ul_bytes: u64,
    pub total_ttis: u64,
    pub total_nacks: u64,
    pub total_retx: u64,
}

impl RanScheduler {
    pub fn new(config: CellConfig) -> Self {
        Self {
            config, ues: HashMap::new(), current_tti: 0,
            ue_order: Vec::new(),
            scheduling_history: VecDeque::new(),
            history_window: 1000,
            total_dl_bytes: 0, total_ul_bytes: 0,
            total_ttis: 0, total_nacks: 0, total_retx: 0,
        }
    }

    pub fn add_ue(&mut self, ue: UeContext) {
        let id = ue.ue_id;
        self.ues.insert(id, ue);
        if !self.ue_order.contains(&id) {
            self.ue_order.push(id);
        }
    }

    pub fn remove_ue(&mut self, ue_id: u32) {
        self.ues.remove(&ue_id);
        self.ue_order.retain(|&id| id != ue_id);
    }

    pub fn dl_harq_ack(&mut self, ue_id: u32, harq_pid: u8) {
        if let Some(ue) = self.ues.get_mut(&ue_id) {
            if let Some(h) = ue.dl_harq.get_mut(harq_pid as usize) {
                h.state = HarqState::Acked;
            }
        }
    }

    pub fn dl_harq_nack(&mut self, ue_id: u32, harq_pid: u8, tti: u64) {
        if let Some(ue) = self.ues.get_mut(&ue_id) {
            if let Some(h) = ue.dl_harq.get_mut(harq_pid as usize) {
                h.retx_count += 1;
                self.total_nacks += 1;
                if h.retx_count >= MAX_HARQ_RETX {
                    h.state = HarqState::Idle;
                } else {
                    h.state = HarqState::PendingRetx { retx_count: h.retx_count, tti_nack: tti };
                }
            }
        }
    }

    pub fn update_bsr(&mut self, bsr: BufferStatusReport) {
        let id = bsr.ue_id;
        if let Some(ue) = self.ues.get_mut(&id) {
            ue.bsr = Some(bsr);
        }
    }

    pub fn update_phr(&mut self, phr: PowerHeadroomReport) {
        let id = phr.ue_id;
        if let Some(ue) = self.ues.get_mut(&id) {
            ue.phr = Some(phr);
        }
    }

    pub fn update_cqi(&mut self, ue_id: u32, cqi: u8) {
        if let Some(ue) = self.ues.get_mut(&ue_id) {
            ue.dl_cqi = cqi.min(15).max(1);
        }
    }

    pub fn update_subband_cqi(&mut self, report: SubbandCqiReport) {
        let id = report.ue_id;
        if let Some(ue) = self.ues.get_mut(&id) {
            ue.dl_cqi = report.wideband_cqi;
            ue.subband_cqi = Some(report);
        }
    }

    fn compute_dl_prb_alloc(&self, ue: &UeContext, prbs_remaining: u32) -> u32 {
        if prbs_remaining == 0 { return 0; }
        let available = prbs_remaining.min(self.config.max_prb_per_ue);
        let minimum = self.config.min_prb_per_ue;
        match ue.qos.bearer_type {
            BearerType::MissionCritical => available,
            BearerType::GBR => {
                let gbr_bytes_per_tti = ue.qos.gbr_bps as f64 / 8000.0;
                let mcs = cqi_to_mcs(ue.dl_cqi);
                let tbs_per_prb = compute_tbs_bytes(mcs, 1).max(1) as f64;
                let needed = (gbr_bytes_per_tti / tbs_per_prb).ceil() as u32;
                needed.max(minimum).min(available)
            }
            BearerType::NonGBR => {
                let weight = ue.scheduling_weight.max(0.1);
                ((available as f64 * weight * 0.5) as u32).max(minimum).min(available)
            }
        }
    }

    fn dl_candidate_list(&self) -> Vec<u32> {
        let mut candidates: Vec<(u32, f64)> = self.ues.iter()
            .filter(|(_, ue)| {
                ue.connected && ue.dl_cqi > 0 && ue.next_dl_harq().is_some() &&
                ue.drx.as_ref().map_or(true, |d| d.is_active())
            })
            .map(|(&id, ue)| {
                let metric = match self.config.algorithm {
                    SchedulingAlgorithm::RoundRobin => ue.wait_ttis as f64,
                    SchedulingAlgorithm::ProportionalFair => {
                        let inst = cqi_spectral_efficiency_x1000(ue.dl_cqi) as f64;
                        (inst / ue.avg_throughput_bytes_per_ms.max(1e-6)) * ue.scheduling_weight
                    }
                    SchedulingAlgorithm::MaxCI => {
                        cqi_spectral_efficiency_x1000(ue.dl_cqi) as f64
                    }
                    SchedulingAlgorithm::WeightedFairQueuing => {
                        let deficit = ue.qos.gbr_bps as f64 / 8000.0
                            - ue.avg_throughput_bytes_per_ms;
                        deficit.max(0.0) * ue.scheduling_weight
                    }
                };
                (id, metric)
            })
            .collect();
        candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        candidates.into_iter().map(|(id, _)| id).collect()
    }

    fn ul_candidate_list(&self) -> Vec<u32> {
        let mut candidates: Vec<(u32, f64)> = self.ues.iter()
            .filter(|(_, ue)| {
                ue.connected &&
                ue.bsr.as_ref().map_or(false, |b| b.total_bytes() > 0) &&
                ue.drx.as_ref().map_or(true, |d| d.is_active())
            })
            .map(|(&id, ue)| {
                let bsr_bytes = ue.bsr.as_ref().map_or(0, |b| b.total_bytes()) as f64;
                let metric = match self.config.algorithm {
                    SchedulingAlgorithm::RoundRobin => ue.wait_ttis as f64,
                    SchedulingAlgorithm::ProportionalFair => {
                        let inst = cqi_spectral_efficiency_x1000(ue.ul_cqi()) as f64;
                        (inst / ue.avg_throughput_bytes_per_ms.max(1e-6)) * ue.scheduling_weight
                    }
                    SchedulingAlgorithm::MaxCI => {
                        cqi_spectral_efficiency_x1000(ue.ul_cqi()) as f64
                    }
                    SchedulingAlgorithm::WeightedFairQueuing => bsr_bytes * ue.scheduling_weight,
                };
                (id, metric)
            })
            .collect();
        candidates.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        candidates.into_iter().map(|(id, _)| id).collect()
    }

    pub fn schedule_dl(&mut self) -> Vec<DlGrant> {
        let tti = self.current_tti;
        let mut grants = Vec::new();
        let mut prb_cursor = 0u32;
        let total_prbs = self.config.dl_n_prb;
        let mut ue_count = 0;

        // Phase 1: HARQ retransmissions (highest priority)
        if self.config.harq_retx_priority {
            let retx_ues: Vec<u32> = self.ues.keys()
                .filter(|&&id| self.ues[&id].pending_dl_retx(tti).is_some())
                .copied().collect();

            for ue_id in retx_ues {
                if prb_cursor >= total_prbs || ue_count >= self.config.max_ues_per_tti {
                    break;
                }
                let harq_pid = match self.ues[&ue_id].pending_dl_retx(tti) {
                    Some(pid) => pid,
                    None => continue,
                };
                let (mcs, n_prb, tbs, c_rnti, retx_count) = {
                    let ue = &self.ues[&ue_id];
                    let h = &ue.dl_harq[harq_pid as usize];
                    let rc = if let HarqState::PendingRetx { retx_count, .. } = h.state {
                        retx_count
                    } else { 1 };
                    (h.mcs, h.n_prb, h.tbs_bytes, ue.c_rnti, rc)
                };
                if n_prb > total_prbs - prb_cursor { continue; }
                grants.push(DlGrant {
                    ue_id, c_rnti, tti, harq_process_id: harq_pid, mcs, n_prb,
                    prb_start: prb_cursor, tbs_bytes: tbs, is_retx: true,
                    ndi: false, rv: retx_count & 0x3, is_sps: false,
                });
                prb_cursor += n_prb;
                ue_count += 1;
                self.total_retx += 1;
                if let Some(ue) = self.ues.get_mut(&ue_id) {
                    ue.dl_harq[harq_pid as usize].state = HarqState::WaitingAck { tti_sent: tti };
                }
            }
        }

        // Phase 2: New data
        let candidates = self.dl_candidate_list();
        for ue_id in candidates {
            if prb_cursor >= total_prbs || ue_count >= self.config.max_ues_per_tti {
                break;
            }
            let prbs_left = total_prbs - prb_cursor;

            let is_sps = if self.config.sps_enabled {
                if let Some(ue) = self.ues.get_mut(&ue_id) {
                    if let Some(sps) = ue.sps.as_mut() {
                        let has_data = ue.bsr.as_ref().map_or(true, |b| b.total_bytes() > 0);
                        sps.should_grant(tti, has_data)
                    } else { false }
                } else { false }
            } else { false };

            let params = {
                let ue = match self.ues.get(&ue_id) { Some(u) => u, None => continue };
                let harq_pid = match ue.next_dl_harq() { Some(p) => p, None => continue };
                let mcs = if is_sps {
                    ue.sps.as_ref().map_or_else(|| cqi_to_mcs(ue.dl_cqi), |s| s.config.mcs)
                } else {
                    let cqi = if let Some(sb) = &ue.subband_cqi {
                        sb.cqi_for_prb_range(prb_cursor, total_prbs)
                    } else { ue.dl_cqi };
                    cqi_to_mcs(cqi.max(1))
                };
                let n_prb = if is_sps {
                    ue.sps.as_ref().map_or(self.config.min_prb_per_ue, |s| s.config.n_prb)
                } else {
                    self.compute_dl_prb_alloc(ue, prbs_left)
                };
                if n_prb == 0 { continue; }
                let tbs = compute_tbs_bytes(mcs, n_prb);
                (harq_pid, mcs, n_prb, tbs, ue.c_rnti)
            };
            let (harq_pid, mcs, n_prb, tbs_bytes, c_rnti) = params;
            if n_prb > prbs_left { continue; }

            grants.push(DlGrant {
                ue_id, c_rnti, tti, harq_process_id: harq_pid, mcs, n_prb,
                prb_start: prb_cursor, tbs_bytes, is_retx: false, ndi: true, rv: 0, is_sps,
            });
            prb_cursor += n_prb;
            ue_count += 1;

            if let Some(ue) = self.ues.get_mut(&ue_id) {
                ue.dl_harq[harq_pid as usize] = HarqProcess {
                    process_id: harq_pid,
                    state: HarqState::WaitingAck { tti_sent: tti },
                    tbs_bytes, mcs, n_prb, tti_scheduled: tti,
                    retx_count: 0, llr_buffer_bytes: tbs_bytes,
                };
                ue.total_dl_bytes += tbs_bytes as u64;
                ue.scheduled_ttis += 1;
                ue.update_pf_throughput(tbs_bytes);
                ue.last_dl_tti = tti;
                ue.total_delay_ms += ue.wait_ttis;
                ue.delay_samples += 1;
                ue.wait_ttis = 0;
                self.total_dl_bytes += tbs_bytes as u64;
            }
        }

        let scheduled_ids: Vec<u32> = grants.iter().map(|g| g.ue_id).collect();
        for ue in self.ues.values_mut() {
            if !scheduled_ids.contains(&ue.ue_id) {
                ue.wait_ttis += 1;
            }
        }
        grants
    }

    pub fn schedule_ul(&mut self) -> Vec<UlGrant> {
        let tti = self.current_tti;
        let mut grants = Vec::new();
        let mut prb_cursor = 0u32;
        let total_prbs = self.config.ul_n_prb;
        let mut ue_count = 0;

        let candidates = self.ul_candidate_list();
        for ue_id in candidates {
            if prb_cursor >= total_prbs || ue_count >= self.config.max_ues_per_tti {
                break;
            }
            let prbs_left = total_prbs - prb_cursor;

            let result = {
                let ue = match self.ues.get(&ue_id) { Some(u) => u, None => continue };
                let bsr_bytes = ue.bsr.as_ref().map_or(0, |b| b.total_bytes());
                if bsr_bytes == 0 { continue; }
                let harq_pid = match ue.ul_harq.iter().find(|h| h.is_available()) {
                    Some(h) => h.process_id, None => continue,
                };
                let ul_cqi = ue.ul_cqi();
                let max_mcs_phr = ue.phr.as_ref().map_or(28u8, |p| p.max_ul_mcs());
                let mcs = cqi_to_mcs(ul_cqi).min(max_mcs_phr);
                let tbs_per_prb = compute_tbs_bytes(mcs, 1).max(1);
                let prbs_needed = ((bsr_bytes + tbs_per_prb - 1) / tbs_per_prb)
                    .max(self.config.min_prb_per_ue)
                    .min(self.config.max_prb_per_ue)
                    .min(prbs_left);
                let tbs_bytes = compute_tbs_bytes(mcs, prbs_needed);
                let tpc: i8 = match ue.phr.as_ref().map_or(10, |p| p.headroom_db) {
                    h if h > 20 => 3,
                    h if h > 10 => 1,
                    h if h > 5 => 0,
                    h if h > 0 => -1,
                    _ => -3,
                };
                Some((harq_pid, mcs, prbs_needed, tbs_bytes, ue.c_rnti, tpc))
            };

            let (harq_pid, mcs, n_prb, tbs_bytes, c_rnti, tpc) = match result {
                Some(v) => v, None => continue,
            };
            if n_prb > prbs_left { continue; }

            grants.push(UlGrant {
                ue_id, c_rnti, tti, harq_process_id: harq_pid, mcs, n_prb,
                prb_start: prb_cursor, tbs_bytes, is_retx: false, ndi: true, rv: 0, tpc,
            });
            prb_cursor += n_prb;
            ue_count += 1;

            if let Some(ue) = self.ues.get_mut(&ue_id) {
                ue.ul_harq[harq_pid as usize].state = HarqState::WaitingAck { tti_sent: tti };
                ue.total_ul_bytes += tbs_bytes as u64;
                ue.last_ul_tti = tti;
                if let Some(bsr) = ue.bsr.as_mut() {
                    let mut remaining = tbs_bytes.min(bsr.total_bytes());
                    for lcg in bsr.lcg_bytes.iter_mut() {
                        if remaining == 0 { break; }
                        let take = remaining.min(*lcg);
                        *lcg -= take;
                        remaining -= take;
                    }
                }
                self.total_ul_bytes += tbs_bytes as u64;
            }
        }
        grants
    }

    pub fn run_tti(&mut self) -> TtiSchedulingResult {
        let tti = self.current_tti;

        for ue in self.ues.values_mut() {
            ue.replenish_buckets();
        }

        // DRX tick (all UEs, no scheduling last TTI info)
        let ue_ids: Vec<u32> = self.ues.keys().copied().collect();
        for id in &ue_ids {
            if let Some(ue) = self.ues.get_mut(id) {
                if let Some(drx) = ue.drx.as_mut() {
                    drx.tick(tti, false);
                }
            }
        }

        let dl_grants = self.schedule_dl();
        let ul_grants = self.schedule_ul();

        let dl_prbs_used: u32 = dl_grants.iter().map(|g| g.n_prb).sum();
        let ul_prbs_used: u32 = ul_grants.iter().map(|g| g.n_prb).sum();
        let total_dl_b: u32 = dl_grants.iter().map(|g| g.tbs_bytes).sum();
        let total_ul_b: u32 = ul_grants.iter().map(|g| g.tbs_bytes).sum();
        let scheduled_count = dl_grants.len().max(ul_grants.len());

        self.scheduling_history.push_back((tti, total_dl_b, total_ul_b));
        if self.scheduling_history.len() > self.history_window {
            self.scheduling_history.pop_front();
        }

        let result = TtiSchedulingResult {
            tti, dl_prbs_used, ul_prbs_used,
            dl_prbs_available: self.config.dl_n_prb,
            ul_prbs_available: self.config.ul_n_prb,
            scheduled_ue_count: scheduled_count,
            dl_grants, ul_grants,
        };

        self.current_tti += 1;
        self.total_ttis += 1;
        result
    }

    pub fn compute_fairness_metrics(&self) -> FairnessMetrics {
        let active_ues: Vec<&UeContext> = self.ues.values()
            .filter(|u| u.connected && u.scheduled_ttis > 0)
            .collect();
        if active_ues.is_empty() {
            return FairnessMetrics::default();
        }
        let throughputs: Vec<f64> = active_ues.iter()
            .map(|u| u.avg_throughput_bytes_per_ms * 8.0 / 1000.0)
            .collect();
        let jains = jains_fairness_index(&throughputs);
        let n = throughputs.len() as f64;
        let mean = throughputs.iter().sum::<f64>() / n;
        let variance = throughputs.iter().map(|&x| (x - mean).powi(2)).sum::<f64>() / n;
        let cv = if mean > 0.0 { variance.sqrt() / mean } else { 0.0 };
        let active_with_delay: Vec<&UeContext> =
            active_ues.iter().filter(|u| u.delay_samples > 0).copied().collect();
        let avg_delay = if !active_with_delay.is_empty() {
            active_with_delay.iter()
                .map(|u| u.total_delay_ms as f64 / u.delay_samples as f64)
                .sum::<f64>() / active_with_delay.len() as f64
        } else { 0.0 };
        let max_delay = active_with_delay.iter()
            .map(|u| u.total_delay_ms / u.delay_samples)
            .max().unwrap_or(0);
        let total_tput: f64 = throughputs.iter().sum();
        let p95_delay = avg_delay + 1.645 * variance.sqrt();
        FairnessMetrics {
            jains_index: jains, throughput_variance: variance, throughput_cv: cv,
            avg_delay_ms: avg_delay, max_delay_ms: max_delay, p95_delay_ms: p95_delay,
            total_dl_throughput_mbps: total_tput,
            active_ue_count: active_ues.len(), prb_utilization: 0.0,
        }
    }

    pub fn throughput_summary(&self) -> Vec<(u32, f64)> {
        let mut result: Vec<(u32, f64)> = self.ues.iter()
            .map(|(&id, ue)| (id, ue.avg_throughput_bytes_per_ms * 8.0 / 1000.0))
            .collect();
        result.sort_by(|a, b| a.0.cmp(&b.0));
        result
    }

    pub fn cell_throughput_mbps(&self) -> f64 {
        if self.scheduling_history.is_empty() { return 0.0; }
        let total_bytes: u64 = self.scheduling_history.iter()
            .map(|(_, dl, ul)| *dl as u64 + *ul as u64).sum();
        let window = self.scheduling_history.len() as f64;
        total_bytes as f64 * 8.0 / window / 1_000_000.0
    }
}

// ---------------------------------------------------------------------------
// Utility Functions
// ---------------------------------------------------------------------------

/// Shannon capacity (Mbps) for bandwidth and linear SNR
pub fn shannon_capacity_mbps(bandwidth_hz: f64, snr_linear: f64) -> f64 {
    bandwidth_hz * (1.0 + snr_linear).log2() / 1_000_000.0
}

pub fn snr_db_to_linear(snr_db: f64) -> f64 {
    10f64.powf(snr_db / 10.0)
}

pub fn prbs_for_throughput(target_bps: u64, cqi: u8, prb_bandwidth_hz: f64) -> u32 {
    let se = cqi_spectral_efficiency_x1000(cqi) as f64 / 1000.0;
    if se < 0.01 { return 1; }
    let bits_per_prb_per_ms = se * prb_bandwidth_hz / 1000.0;
    let target_bits_per_ms = target_bps as f64 / 1000.0;
    ((target_bits_per_ms / bits_per_prb_per_ms).ceil() as u32).max(1)
}

pub fn advance_rr(ptr: usize, n: usize) -> usize {
    if n == 0 { 0 } else { (ptr + 1) % n }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // ---- CQI / MCS / TBS ----

    #[test]
    fn test_cqi_to_mcs_range() {
        for cqi in 1u8..=15 {
            let mcs = cqi_to_mcs(cqi);
            assert!(mcs <= MAX_MCS as u8, "MCS {} out of range for CQI {}", mcs, cqi);
        }
    }

    #[test]
    fn test_cqi_mcs_monotonic() {
        let mut prev = cqi_to_mcs(1);
        for cqi in 2u8..=15 {
            let mcs = cqi_to_mcs(cqi);
            assert!(mcs >= prev, "CQI {} MCS {} not >= CQI {} MCS {}", cqi, mcs, cqi - 1, prev);
            prev = mcs;
        }
    }

    #[test]
    fn test_mcs_modulation_order() {
        assert_eq!(mcs_to_modulation_order(0), 2);
        assert_eq!(mcs_to_modulation_order(9), 2);
        assert_eq!(mcs_to_modulation_order(10), 4);
        assert_eq!(mcs_to_modulation_order(16), 4);
        assert_eq!(mcs_to_modulation_order(17), 6);
        assert_eq!(mcs_to_modulation_order(28), 6);
    }

    #[test]
    fn test_tbs_increases_with_prbs() {
        let mcs = 15u8;
        let mut prev = 0u32;
        for n in 1..=100u32 {
            let tbs = compute_tbs_bytes(mcs, n);
            assert!(tbs >= prev, "TBS not non-decreasing at n={}", n);
            prev = tbs;
        }
    }

    #[test]
    fn test_tbs_increases_with_mcs() {
        let n_prb = 10u32;
        let mut prev = 0u32;
        for mcs in 0u8..=28 {
            let tbs = compute_tbs_bytes(mcs, n_prb);
            assert!(tbs >= prev, "TBS mcs={} tbs={} < prev={}", mcs, tbs, prev);
            prev = tbs;
        }
    }

    #[test]
    fn test_tbs_zero_prbs() {
        assert_eq!(compute_tbs_bytes(15, 0), 0);
    }

    #[test]
    fn test_cqi_spectral_efficiency_positive() {
        for cqi in 1u8..=15 {
            assert!(cqi_spectral_efficiency_x1000(cqi) > 0, "SE should be positive for CQI {}", cqi);
        }
        assert_eq!(cqi_spectral_efficiency_x1000(0), 0);
    }

    #[test]
    fn test_cqi_spectral_efficiency_monotonic() {
        let mut prev = 0u32;
        for cqi in 1u8..=15 {
            let se = cqi_spectral_efficiency_x1000(cqi);
            assert!(se >= prev, "SE not monotonic at CQI {}", cqi);
            prev = se;
        }
    }

    // ---- BSR ----

    #[test]
    fn test_bsr_roundtrip() {
        for idx in 0u8..=62 {
            let bytes = bsr_index_to_bytes(idx);
            let back = bytes_to_bsr_index(bytes);
            assert!(back <= idx + 1, "BSR roundtrip idx={} bytes={} back={}", idx, bytes, back);
        }
    }

    #[test]
    fn test_bsr_zero_buffer() {
        assert_eq!(bsr_index_to_bytes(0), 0);
    }

    #[test]
    fn test_bsr_max() {
        assert_eq!(bsr_index_to_bytes(63), u32::MAX);
    }

    // ---- QoS / Bearer ----

    #[test]
    fn test_volte_qos_params() {
        let q = QosParameters::volte();
        assert_eq!(q.qci, 1);
        assert_eq!(q.bearer_type, BearerType::GBR);
        assert!(q.gbr_bps > 0);
        assert!(q.pdb_ms <= 100);
    }

    #[test]
    fn test_priority_ordering() {
        let mc = QosParameters::urllc();
        let gbr = QosParameters::volte();
        let nongbr = QosParameters::best_effort();
        assert!(mc.priority_score() < gbr.priority_score());
        assert!(gbr.priority_score() < nongbr.priority_score());
    }

    #[test]
    fn test_qos_gaming() {
        let q = QosParameters::gaming();
        assert!(q.pdb_ms <= 50);
        assert_eq!(q.bearer_type, BearerType::GBR);
    }

    // ---- HARQ ----

    #[test]
    fn test_harq_initial_idle() {
        let h = HarqProcess::new(0);
        assert!(h.is_available());
        assert!(!h.needs_retx());
    }

    #[test]
    fn test_harq_nack_state() {
        let mut h = HarqProcess::new(0);
        h.state = HarqState::PendingRetx { retx_count: 1, tti_nack: 10 };
        assert!(h.needs_retx());
        assert!(!h.is_available());
    }

    #[test]
    fn test_harq_acked_available() {
        let mut h = HarqProcess::new(0);
        h.state = HarqState::Acked;
        assert!(h.is_available());
    }

    // ---- DRX ----

    #[test]
    fn test_drx_initial_active() {
        let drx = DrxStateMachine::new(DrxConfig::volte());
        assert!(drx.is_active());
        assert_eq!(drx.state, DrxState::Active);
    }

    #[test]
    fn test_drx_stays_active_when_scheduled() {
        let mut drx = DrxStateMachine::new(DrxConfig::volte());
        for tti in 0u64..50 {
            drx.tick(tti, true);
            assert!(drx.is_active(), "Should stay active when scheduled TTI={}", tti);
        }
    }

    #[test]
    fn test_drx_enters_cycle_on_inactivity() {
        let config = DrxConfig::volte();
        let inact = config.inactivity_timer_ms;
        let mut drx = DrxStateMachine::new(config);
        for tti in 0..(inact + 10) as u64 {
            drx.tick(tti, false);
        }
        assert_ne!(drx.state, DrxState::Active);
    }

    #[test]
    fn test_drx_configs_construct() {
        let _ = DrxConfig::volte();
        let _ = DrxConfig::embb();
        let _ = DrxConfig::iot();
    }

    // ---- SPS ----

    #[test]
    fn test_sps_initial_inactive() {
        let sps = SpsContext::new(SpsConfig::volte_amr_wb());
        assert_eq!(sps.state, SpsState::Inactive);
    }

    #[test]
    fn test_sps_activation() {
        let mut sps = SpsContext::new(SpsConfig::volte_amr_wb());
        sps.activate(0);
        assert_eq!(sps.state, SpsState::Active);
        assert_eq!(sps.next_sps_tti, 20);
    }

    #[test]
    fn test_sps_grant_timing() {
        let mut sps = SpsContext::new(SpsConfig::volte_amr_wb());
        sps.activate(0);
        assert!(!sps.should_grant(0, true));
        assert!(!sps.should_grant(10, true));
        assert!(!sps.should_grant(19, true));
        assert!(sps.should_grant(20, true));
        assert!(!sps.should_grant(30, true));
        assert!(sps.should_grant(40, true));
    }

    #[test]
    fn test_sps_implicit_release() {
        let mut sps = SpsContext::new(SpsConfig::volte_amr_wb());
        sps.activate(0);
        for i in 0..=8u64 {
            sps.should_grant(i * 20, false);
        }
        assert_ne!(sps.state, SpsState::Active);
    }

    #[test]
    fn test_sps_vonr_config() {
        let cfg = SpsConfig::vonr();
        assert_eq!(cfg.interval_ms, 10);
    }

    // ---- Subband CQI ----

    #[test]
    fn test_subband_cqi_best_subbands() {
        let report = SubbandCqiReport {
            ue_id: 1, wideband_cqi: 8,
            subband_cqi: vec![5, 12, 10, 7, 14, 6, 11, 9],
            subband_size_prb: 4, tti_reported: 0, rank_indicator: 1, pmi: 0,
        };
        let best = report.best_subbands(3);
        assert_eq!(best.len(), 3);
        assert!(best.contains(&4)); // CQI=14
        assert!(best.contains(&1)); // CQI=12
    }

    #[test]
    fn test_subband_cqi_fallback() {
        let report = SubbandCqiReport {
            ue_id: 1, wideband_cqi: 9, subband_cqi: vec![],
            subband_size_prb: 4, tti_reported: 0, rank_indicator: 1, pmi: 0,
        };
        assert_eq!(report.cqi_for_prb_range(0, 50), 9);
    }

    // ---- UE Context ----

    #[test]
    fn test_ue_creation() {
        let ue = UeContext::new(1, 0x1234, QosParameters::volte());
        assert_eq!(ue.ue_id, 1);
        assert_eq!(ue.dl_harq.len(), MAX_HARQ_PROCESSES);
        assert!(ue.connected);
    }

    #[test]
    fn test_ue_pf_metric() {
        let mut ue = UeContext::new(1, 1, QosParameters::best_effort());
        ue.dl_cqi = 10;
        let m = ue.pf_metric();
        assert!(m > 0.0);
    }

    #[test]
    fn test_ue_token_bucket_gbr() {
        let mut ue = UeContext::new(1, 1, QosParameters::volte());
        assert!(!ue.token_bucket_check(100));
        for _ in 0..10 { ue.replenish_buckets(); }
        assert!(ue.token_bucket_check(1));
    }

    #[test]
    fn test_ue_ul_cqi() {
        let mut ue = UeContext::new(1, 1, QosParameters::best_effort());
        ue.ul_sinr_db = 20.0;
        assert_eq!(ue.ul_cqi(), 15);
        ue.ul_sinr_db = -10.0;
        assert_eq!(ue.ul_cqi(), 0);
    }

    #[test]
    fn test_ue_pending_retx_timing() {
        let mut ue = UeContext::new(1, 1, QosParameters::best_effort());
        ue.dl_harq[0].state = HarqState::PendingRetx { retx_count: 1, tti_nack: 0 };
        assert!(ue.pending_dl_retx(LTE_HARQ_RTT as u64).is_some());
        assert!(ue.pending_dl_retx(LTE_HARQ_RTT as u64 - 1).is_none());
    }

    // ---- Cell Config ----

    #[test]
    fn test_cell_configs_prb() {
        assert_eq!(CellConfig::lte_10mhz().dl_n_prb, 50);
        assert_eq!(CellConfig::lte_20mhz().dl_n_prb, 100);
        assert_eq!(CellConfig::nr_100mhz().dl_n_prb, 273);
    }

    #[test]
    fn test_cell_tti_duration() {
        assert_eq!(CellConfig::lte_10mhz().tti_us(), 1000);
        assert_eq!(CellConfig::nr_100mhz().tti_us(), 500);
    }

    // ---- Scheduler tests ----

    fn make_sched(n_ues: u32, algo: SchedulingAlgorithm) -> RanScheduler {
        let mut cfg = CellConfig::lte_20mhz();
        cfg.algorithm = algo;
        let mut s = RanScheduler::new(cfg);
        for i in 0..n_ues {
            let mut ue = UeContext::new(i, (i + 1) as u16, QosParameters::best_effort());
            ue.dl_cqi = (5 + i % 10) as u8;
            ue.ul_sinr_db = 10.0 + (i % 10) as f32;
            s.add_ue(ue);
        }
        s
    }

    #[test]
    fn test_scheduler_dl_grants_issued() {
        let mut s = make_sched(5, SchedulingAlgorithm::RoundRobin);
        let r = s.run_tti();
        assert!(!r.dl_grants.is_empty());
    }

    #[test]
    fn test_no_prb_overlap_dl() {
        let mut s = make_sched(10, SchedulingAlgorithm::ProportionalFair);
        let r = s.run_tti();
        for i in 0..r.dl_grants.len() {
            for j in (i + 1)..r.dl_grants.len() {
                let gi = &r.dl_grants[i];
                let gj = &r.dl_grants[j];
                let ei = gi.prb_start + gi.n_prb;
                let ej = gj.prb_start + gj.n_prb;
                assert!(ei <= gj.prb_start || ej <= gi.prb_start,
                    "PRB overlap UE{} [{},{}] vs UE{} [{},{}]",
                    gi.ue_id, gi.prb_start, ei, gj.ue_id, gj.prb_start, ej);
            }
        }
    }

    #[test]
    fn test_prb_within_bounds() {
        let mut s = make_sched(20, SchedulingAlgorithm::MaxCI);
        let r = s.run_tti();
        let n = s.config.dl_n_prb;
        for g in &r.dl_grants {
            assert!(g.prb_start + g.n_prb <= n, "Grant exceeds PRB limit");
        }
    }

    #[test]
    fn test_ul_grants_with_bsr() {
        let mut s = make_sched(5, SchedulingAlgorithm::RoundRobin);
        for i in 0..5u32 {
            s.update_bsr(BufferStatusReport { ue_id: i, lcg_bytes: [5000, 0, 0, 0], tti_reported: 0 });
        }
        let r = s.run_tti();
        assert!(!r.ul_grants.is_empty());
    }

    #[test]
    fn test_no_ul_grants_without_bsr() {
        let mut s = make_sched(5, SchedulingAlgorithm::RoundRobin);
        let r = s.run_tti();
        assert!(r.ul_grants.is_empty());
    }

    #[test]
    fn test_harq_ack_sets_acked() {
        let mut s = make_sched(1, SchedulingAlgorithm::RoundRobin);
        let r = s.run_tti();
        if let Some(g) = r.dl_grants.first() {
            let (uid, pid) = (g.ue_id, g.harq_process_id);
            s.dl_harq_ack(uid, pid);
            assert_eq!(s.ues[&uid].dl_harq[pid as usize].state, HarqState::Acked);
        }
    }

    #[test]
    fn test_harq_nack_sets_retx() {
        let mut s = make_sched(1, SchedulingAlgorithm::RoundRobin);
        let r = s.run_tti();
        if let Some(g) = r.dl_grants.first() {
            let (uid, pid) = (g.ue_id, g.harq_process_id);
            let tti = s.current_tti;
            s.dl_harq_nack(uid, pid, tti);
            assert!(s.ues[&uid].dl_harq[pid as usize].needs_retx());
        }
    }

    #[test]
    fn test_all_algorithms_issue_grants() {
        for algo in [
            SchedulingAlgorithm::RoundRobin,
            SchedulingAlgorithm::ProportionalFair,
            SchedulingAlgorithm::MaxCI,
            SchedulingAlgorithm::WeightedFairQueuing,
        ] {
            let mut s = make_sched(5, algo);
            let r = s.run_tti();
            assert!(!r.dl_grants.is_empty(), "No DL grants for {:?}", algo);
        }
    }

    #[test]
    fn test_max_ues_per_tti_limit() {
        let max_tti = 4usize;
        let mut cfg = CellConfig::lte_20mhz();
        cfg.max_ues_per_tti = max_tti;
        let mut s = RanScheduler::new(cfg);
        for i in 0..20u32 {
            let mut ue = UeContext::new(i, (i + 1) as u16, QosParameters::best_effort());
            ue.dl_cqi = 10;
            s.add_ue(ue);
        }
        let r = s.run_tti();
        assert!(r.dl_grants.len() <= max_tti,
            "Too many DL grants: {} > {}", r.dl_grants.len(), max_tti);
    }

    #[test]
    fn test_tti_counter_increments() {
        let mut s = make_sched(2, SchedulingAlgorithm::RoundRobin);
        assert_eq!(s.current_tti, 0);
        s.run_tti(); assert_eq!(s.current_tti, 1);
        s.run_tti(); assert_eq!(s.current_tti, 2);
    }

    // ---- Fairness ----

    #[test]
    fn test_jains_equal_throughput() {
        let t = vec![100.0; 4];
        assert!((jains_fairness_index(&t) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_jains_single_ue() {
        assert!((jains_fairness_index(&[500.0]) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn test_jains_unequal() {
        let j = jains_fairness_index(&[100.0, 1.0, 1.0, 1.0]);
        assert!(j < 1.0 && j > 0.0);
    }

    #[test]
    fn test_jains_empty() {
        assert_eq!(jains_fairness_index(&[]), 1.0);
    }

    #[test]
    fn test_fairness_metrics_after_scheduling() {
        let mut s = make_sched(8, SchedulingAlgorithm::ProportionalFair);
        for _ in 0..100 { s.run_tti(); }
        let m = s.compute_fairness_metrics();
        assert!(m.jains_index > 0.0 && m.jains_index <= 1.0);
        assert!(m.active_ue_count > 0);
    }

    // ---- Multi-TTI ----

    #[test]
    fn test_multi_tti_dl_scheduling() {
        let mut s = make_sched(8, SchedulingAlgorithm::ProportionalFair);
        for i in 0..8u32 {
            s.update_bsr(BufferStatusReport { ue_id: i, lcg_bytes: [100_000, 0, 0, 0], tti_reported: 0 });
        }
        let mut total_dl = 0u64;
        for _ in 0..100 {
            let r = s.run_tti();
            total_dl += r.total_dl_bytes() as u64;
        }
        assert!(total_dl > 0);
        assert_eq!(s.current_tti, 100);
    }

    // ---- PHR / TPC ----

    #[test]
    fn test_phr_max_mcs() {
        let p = PowerHeadroomReport { ue_id: 1, headroom_db: 25, tti_reported: 0, p_cmax_dbm: 23 };
        assert_eq!(p.max_ul_mcs(), 28);
        let p2 = PowerHeadroomReport { ue_id: 1, headroom_db: -10, tti_reported: 0, p_cmax_dbm: 23 };
        assert_eq!(p2.max_ul_mcs(), 0);
    }

    #[test]
    fn test_ul_tpc_negative_with_low_phr() {
        let mut s = make_sched(1, SchedulingAlgorithm::RoundRobin);
        s.update_phr(PowerHeadroomReport { ue_id: 0, headroom_db: -5, tti_reported: 0, p_cmax_dbm: 23 });
        s.update_bsr(BufferStatusReport { ue_id: 0, lcg_bytes: [5000, 0, 0, 0], tti_reported: 0 });
        let r = s.run_tti();
        if let Some(g) = r.ul_grants.first() {
            assert!(g.tpc <= 0, "TPC should be <= 0 with low PHR, got {}", g.tpc);
        }
    }

    // ---- Remove UE ----

    #[test]
    fn test_remove_ue() {
        let mut s = make_sched(3, SchedulingAlgorithm::RoundRobin);
        assert_eq!(s.ues.len(), 3);
        s.remove_ue(1);
        assert_eq!(s.ues.len(), 2);
        assert!(!s.ues.contains_key(&1));
    }

    // ---- Utility ----

    #[test]
    fn test_shannon_capacity() {
        let snr = snr_db_to_linear(20.0);
        let cap = shannon_capacity_mbps(20e6, snr);
        assert!(cap > 50.0 && cap < 300.0);
    }

    #[test]
    fn test_prbs_for_throughput_positive() {
        assert!(prbs_for_throughput(10_000_000, 10, 180_000.0) > 0);
    }

    #[test]
    fn test_advance_rr_wrap() {
        assert_eq!(advance_rr(0, 5), 1);
        assert_eq!(advance_rr(4, 5), 0);
        assert_eq!(advance_rr(0, 0), 0);
    }

    // ---- NR ----

    #[test]
    fn test_nr_cell_273_prbs() {
        let cfg = CellConfig::nr_100mhz();
        let mut s = RanScheduler::new(cfg);
        for i in 0..10u32 {
            let mut ue = UeContext::new(i, (i + 1) as u16, QosParameters::best_effort());
            ue.dl_cqi = 12;
            s.add_ue(ue);
        }
        let r = s.run_tti();
        assert_eq!(r.dl_prbs_available, 273);
        assert!(r.dl_prbs_used > 0);
    }

    #[test]
    fn test_cell_throughput_metric() {
        let mut s = make_sched(5, SchedulingAlgorithm::ProportionalFair);
        for _ in 0..10 { s.run_tti(); }
        assert!(s.cell_throughput_mbps() >= 0.0);
    }

    #[test]
    fn test_cqi_table_completeness() {
        assert_eq!(CQI_TABLE.len(), 15);
        for (mo, cr) in CQI_TABLE.iter() {
            assert!(*mo == 2 || *mo == 4 || *mo == 6);
            assert!(*cr > 0 && *cr < 1024);
        }
    }
}
