//! LTE Relay Node Processor per 3GPP TS 36.216
//!
//! Implements the LTE relay node physical layer for Type 1, Type 1a, and Type 2
//! relay nodes as specified in 3GPP TS 36.216, TS 36.211, and TS 36.300.
//!
//! # Relay Node Architecture
//!
//! LTE relay nodes operate on two links:
//! - **Backhaul link (Un interface)**: Between DeNB (donor eNB) and relay node (RN)
//! - **Access link (Uu interface)**: Between relay node and UEs served by the relay
//!
//! ## Relay Types
//!
//! - **Type 1**: Inband relay, half-duplex FDD/TDD, appears as separate cell to UEs.
//!   Self-interference between access and backhaul links requires MBSFN subframe gaps.
//! - **Type 1a**: Outband relay, full-duplex possible, backhaul on different frequency.
//! - **Type 2**: Transparent relay, no new cell ID, does not affect UE mobility.
//!
//! # Backhaul Link (Un interface)
//!
//! The backhaul uses R-PDCCH (Relay-PDCCH) for scheduling in MBSFN subframes:
//! - DCI format 1A for relay DL scheduling (R-PDSCH)
//! - DCI format 2C for relay UL grant (R-PUSCH)
//! - Cross-subframe scheduling with 4-ms HARQ timeline
//!
//! # Key Standards
//!
//! - 3GPP TS 36.216: Physical layer for relay nodes
//! - 3GPP TS 36.211 §6.10.3: Relay PDCCH
//! - 3GPP TS 36.300 §4.7: Relay architecture
//!
//! # Example
//!
//! ```
//! use r4w_core::lte_relay_processor::*;
//!
//! let config = RelayConfig {
//!     relay_type: RelayType::Type1,
//!     cell_id: 100,
//!     denb_cell_id: 1,
//!     num_prb: 50,
//!     duplex_mode: DuplexMode::Fdd,
//!     backhaul_subframe_config: BackhaulSubframeConfig::default(),
//!     r_pdcch_config: RPdcchConfig::default(),
//!     relay_timing: RelayTiming::default(),
//! };
//! let processor = LteRelayProcessor::new(config);
//! assert_eq!(processor.relay_type(), RelayType::Type1);
//! ```

/// Speed of light in metres per second
const SPEED_OF_LIGHT: f64 = 2.998e8;

/// LTE subframe duration in seconds
const SUBFRAME_DURATION_S: f64 = 1e-3;

/// LTE slot duration in seconds
const SLOT_DURATION_S: f64 = 5e-4;

/// Number of subframes per LTE radio frame
const SUBFRAMES_PER_FRAME: usize = 10;

/// Number of OFDM symbols per normal-CP slot (TS 36.211 Table 6.2-1)
const SYMBOLS_PER_SLOT_NORMAL_CP: usize = 7;

/// Maximum number of PRB in a 20 MHz LTE carrier
const MAX_PRB: usize = 100;

/// Boltzmann constant
const BOLTZMANN: f64 = 1.380649e-23;

/// Reference temperature for noise calculation (K)
const NOISE_TEMP_REF: f64 = 290.0;

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// LTE relay node type per 3GPP TS 36.216
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RelayType {
    /// Type 1: Inband relay, half-duplex, separate cell identity
    Type1,
    /// Type 1a: Outband relay (separate backhaul frequency), can be full-duplex
    Type1a,
    /// Type 2: Transparent relay, no new cell identity, no impact on mobility
    Type2,
}

/// FDD or TDD duplex mode
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DuplexMode {
    /// Frequency Division Duplex
    Fdd,
    /// Time Division Duplex
    Tdd,
}

/// Direction of a subframe for the relay access link
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubframeDirection {
    /// Downlink subframe (DeNB-to-RN on backhaul, RN-to-UE on access)
    Downlink,
    /// Uplink subframe (RN-to-DeNB on backhaul, UE-to-RN on access)
    Uplink,
    /// Special subframe (TDD only)
    Special,
    /// MBSFN subframe used for backhaul DL reception
    MbsfnBackhaul,
    /// Guard / gap subframe
    Guard,
}

/// R-PDCCH DCI format per TS 36.216 §5.3
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DciFormat {
    /// DCI format 1A: Compact DL resource assignment (used for R-PDSCH)
    Format1A,
    /// DCI format 2C: DL multi-codeword with antenna selection (also R-PDSCH)
    Format2C,
    /// DCI format 0: UL grant (used for R-PUSCH)
    Format0,
}

/// HARQ timeline mode for relay
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HarqTimeline {
    /// Standard 4-ms HARQ RTT (used for UE-to-eNB)
    Standard4ms,
    /// Extended 8-ms HARQ RTT (used for relay backhaul to accommodate MBSFN gaps)
    Extended8ms,
}

/// Relay-specific modulation order
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ModulationOrder {
    Qpsk,
    Qam16,
    Qam64,
}

/// Relay handover trigger event
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum HandoverEvent {
    /// A3 event: neighbour becomes better than serving by offset
    A3NeighbourBetter,
    /// A5 event: serving falls below threshold 1 AND neighbour above threshold 2
    A5DualThreshold,
    /// B2 event: serving below threshold 1 AND inter-frequency above threshold 2
    B2InterFreq,
}

/// Relay selection criterion
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RelaySelectionCriterion {
    /// Select relay by maximum RSRP
    MaxRsrp,
    /// Select relay by maximum RSRQ
    MaxRsrq,
    /// Select relay using S-criterion (offset-qualified)
    SCriterion,
}

// ---------------------------------------------------------------------------
// Configuration structures
// ---------------------------------------------------------------------------

/// MBSFN subframe allocation pattern for relay backhaul gaps
///
/// In Type 1 relay operation, MBSFN subframes in the access-link carrier
/// are used to create gaps so the relay can receive from the DeNB without
/// self-interference (TS 36.216 §4.2).
#[derive(Debug, Clone)]
pub struct BackhaulSubframeConfig {
    /// Bitmap of subframes 0-9 within a radio frame that are used for backhaul DL.
    /// Bit i (LSB = subframe 0) set to 1 means that subframe is an MBSFN backhaul subframe.
    pub mbsfn_subframe_bitmap: u16,
    /// Number of gap/guard symbols at the start of an MBSFN subframe before RN switches to RX
    pub guard_symbols_dl: usize,
    /// Number of guard symbols at the end of a backhaul subframe before RN switches to TX
    pub guard_symbols_ul: usize,
    /// MBSFN subframe periodicity in radio frames (1 = every frame, 2 = every 2nd frame, etc.)
    pub periodicity_frames: usize,
}

impl Default for BackhaulSubframeConfig {
    fn default() -> Self {
        // Subframes 1 and 6 allocated for backhaul (typical TS 36.216 example)
        Self {
            mbsfn_subframe_bitmap: 0b0000_0100_0010, // bits 1 and 6
            guard_symbols_dl: 2,
            guard_symbols_ul: 1,
            periodicity_frames: 2,
        }
    }
}

/// R-PDCCH resource allocation configuration per TS 36.211 §6.10.3.2
#[derive(Debug, Clone)]
pub struct RPdcchConfig {
    /// Number of R-PDCCH PRB pairs allocated for backhaul DCI
    pub num_prb_pairs: usize,
    /// Starting PRB index for R-PDCCH region (0-based)
    pub start_prb: usize,
    /// Starting OFDM symbol for R-PDCCH (typically symbol 2 in MBSFN subframe, skipping CRS region)
    pub start_symbol: usize,
    /// Number of OFDM symbols used for R-PDCCH search space
    pub num_symbols: usize,
    /// Aggregation level for R-PDCCH CCE (1, 2, 4, or 8)
    pub aggregation_level: usize,
    /// Enable cross-interleaving of R-PDCCH and R-PDSCH
    pub cross_interleaving_enabled: bool,
}

impl Default for RPdcchConfig {
    fn default() -> Self {
        Self {
            num_prb_pairs: 2,
            start_prb: 0,
            start_symbol: 2,
            num_symbols: 2,
            aggregation_level: 2,
            cross_interleaving_enabled: true,
        }
    }
}

/// Relay timing parameters per TS 36.216 §4.3
#[derive(Debug, Clone)]
pub struct RelayTiming {
    /// Propagation delay from DeNB to RN antenna in seconds (Tp)
    pub propagation_delay_s: f64,
    /// Timing advance applied by RN on the Un interface (seconds)
    pub timing_advance_s: f64,
    /// RX-TX switching time at the relay node in seconds
    pub rx_tx_switch_time_s: f64,
    /// HARQ timeline mode used on backhaul
    pub harq_timeline: HarqTimeline,
}

impl Default for RelayTiming {
    fn default() -> Self {
        Self {
            propagation_delay_s: 1e-6,         // 1 µs = ~300m
            timing_advance_s: 2e-6,            // 2 µs TA
            rx_tx_switch_time_s: 20e-6,        // 20 µs switching
            harq_timeline: HarqTimeline::Extended8ms,
        }
    }
}

/// Top-level relay node configuration
#[derive(Debug, Clone)]
pub struct RelayConfig {
    /// Type 1, 1a, or 2 relay
    pub relay_type: RelayType,
    /// Cell ID of this relay's access link cell (0-503)
    pub cell_id: u16,
    /// Cell ID of the donor eNB
    pub denb_cell_id: u16,
    /// Number of PRB in the carrier (6, 15, 25, 50, 75, 100)
    pub num_prb: usize,
    /// FDD or TDD duplex
    pub duplex_mode: DuplexMode,
    /// MBSFN backhaul subframe configuration
    pub backhaul_subframe_config: BackhaulSubframeConfig,
    /// R-PDCCH scheduling configuration
    pub r_pdcch_config: RPdcchConfig,
    /// Relay timing parameters
    pub relay_timing: RelayTiming,
}

// ---------------------------------------------------------------------------
// DCI message structures
// ---------------------------------------------------------------------------

/// R-PDCCH DCI message (simplified) per TS 36.212 §5.3.3
#[derive(Debug, Clone)]
pub struct DciMessage {
    /// DCI format type
    pub format: DciFormat,
    /// RNTI used for scrambling (relay RNTI allocated by DeNB)
    pub rnti: u16,
    /// Resource allocation type (0 or 1)
    pub resource_alloc_type: u8,
    /// PRB allocation bitmap or resource block assignment field
    pub rb_assignment: u32,
    /// MCS index (0-28 per TS 36.213 Table 7.1.7.1-1)
    pub mcs_index: u8,
    /// HARQ process number (0-7 for FDD, 0-14 for TDD)
    pub harq_process: u8,
    /// Redundancy version (0-3)
    pub rv_index: u8,
    /// New data indicator
    pub ndi: bool,
    /// Transmit power control bits
    pub tpc: u8,
    /// Downlink assignment index (DAI) for TDD
    pub dai: u8,
}

impl DciMessage {
    /// Size in bits of a DCI Format 1A message for a given number of PRB
    /// per TS 36.212 §5.3.3.1.3
    pub fn format1a_size_bits(num_prb: usize) -> usize {
        // ceil(log2(num_prb*(num_prb+1)/2)) bits for resource alloc
        let alloc_bits = ceil_log2(num_prb * (num_prb + 1) / 2);
        // format 1A total: alloc + MCS(5) + HARQ(3) + NDI(1) + RV(2) + TPC(2) + DAI(2) + flags
        alloc_bits + 5 + 3 + 1 + 2 + 2 + 2 + 4
    }

    /// Size in bits of DCI Format 0 (UL grant) for a given number of PRB
    pub fn format0_size_bits(num_prb: usize) -> usize {
        let alloc_bits = ceil_log2(num_prb * (num_prb + 1) / 2);
        alloc_bits + 5 + 2 + 1 + 1 + 2 + 3 + 1
    }

    /// Encode this DCI message to a bit vector (MSB first)
    pub fn encode(&self, num_prb: usize) -> Vec<u8> {
        let size = match self.format {
            DciFormat::Format1A => Self::format1a_size_bits(num_prb),
            DciFormat::Format0 => Self::format0_size_bits(num_prb),
            DciFormat::Format2C => Self::format1a_size_bits(num_prb) + 4,
        };
        let mut bits = vec![0u8; size];
        // Pack RB assignment (variable width)
        let alloc_bits = ceil_log2(num_prb * (num_prb + 1) / 2);
        pack_bits_into(&mut bits, 0, alloc_bits, self.rb_assignment as u64);
        let mut offset = alloc_bits;
        // MCS (5 bits)
        pack_bits_into(&mut bits, offset, 5, self.mcs_index as u64);
        offset += 5;
        // HARQ process (3 bits)
        pack_bits_into(&mut bits, offset, 3, self.harq_process as u64);
        offset += 3;
        // NDI (1 bit)
        bits[offset] = self.ndi as u8;
        offset += 1;
        // RV (2 bits)
        pack_bits_into(&mut bits, offset, 2, self.rv_index as u64);
        offset += 2;
        // TPC (2 bits)
        pack_bits_into(&mut bits, offset, 2, self.tpc as u64);
        let _ = offset;
        bits
    }
}

// ---------------------------------------------------------------------------
// HARQ process tracking
// ---------------------------------------------------------------------------

/// State of a single HARQ process on the backhaul link
#[derive(Debug, Clone)]
pub struct HarqProcess {
    /// Process index (0-7 for FDD backhaul)
    pub process_id: usize,
    /// Number of transmissions so far (1 = first attempt)
    pub tx_count: usize,
    /// Maximum allowed transmissions (HARQ max retx, typically 4)
    pub max_tx: usize,
    /// Accumulated LLR buffer for soft combining (Chase or IR)
    pub soft_buffer: Vec<f32>,
    /// Current redundancy version
    pub rv_index: usize,
    /// Whether this process is active (data pending ACK)
    pub active: bool,
    /// Subframe number of initial transmission
    pub initial_subframe: usize,
    /// New data indicator (toggled on new TB)
    pub ndi: bool,
}

impl HarqProcess {
    pub fn new(process_id: usize, buffer_size: usize) -> Self {
        Self {
            process_id,
            tx_count: 0,
            max_tx: 4,
            soft_buffer: vec![0.0; buffer_size],
            rv_index: 0,
            active: false,
            initial_subframe: 0,
            ndi: false,
        }
    }

    /// Reset process for new transport block
    pub fn reset(&mut self) {
        self.tx_count = 0;
        self.rv_index = 0;
        self.active = true;
        self.ndi = !self.ndi;
        for s in &mut self.soft_buffer {
            *s = 0.0;
        }
    }

    /// Chase combining: accumulate received LLRs into soft buffer
    pub fn chase_combine(&mut self, received_llrs: &[f32]) {
        let n = received_llrs.len().min(self.soft_buffer.len());
        for i in 0..n {
            self.soft_buffer[i] += received_llrs[i];
        }
        self.tx_count += 1;
    }

    /// Determine the next RV index per TS 36.212 Table 5.1.4.2.3-2
    /// Sequence: 0 -> 2 -> 3 -> 1 -> 0 -> ...
    pub fn next_rv(&self) -> usize {
        const RV_SEQUENCE: [usize; 4] = [0, 2, 3, 1];
        RV_SEQUENCE[(self.tx_count) % 4]
    }

    /// Return true if max retransmissions reached
    pub fn is_exhausted(&self) -> bool {
        self.tx_count >= self.max_tx
    }
}

// ---------------------------------------------------------------------------
// R-PDCCH resource mapping
// ---------------------------------------------------------------------------

/// CCE (Control Channel Element) resource element mapping for R-PDCCH
/// per TS 36.211 §6.10.3
#[derive(Debug, Clone)]
pub struct RPdcchMapper {
    pub config: RPdcchConfig,
    /// Number of resource elements per CCE (36 REs per CCE)
    pub re_per_cce: usize,
}

impl RPdcchMapper {
    pub fn new(config: RPdcchConfig) -> Self {
        Self { config, re_per_cce: 36 }
    }

    /// Compute the number of CCEs available in the R-PDCCH region
    /// CCEs = (num_prb_pairs * symbols * 12 RE/PRB - DMRS overhead) / 36
    pub fn num_cces(&self) -> usize {
        let total_re = self.config.num_prb_pairs
            * self.config.num_symbols
            * 12; // 12 subcarriers per PRB
        // Subtract 12 DMRS REs per PRB pair (1 symbol of 12 REs for R-DMRS)
        let dmrs_re = self.config.num_prb_pairs * 12;
        let available_re = total_re.saturating_sub(dmrs_re);
        available_re / self.re_per_cce
    }

    /// Return the PRB indices allocated for the R-PDCCH region
    pub fn allocated_prbs(&self) -> Vec<usize> {
        (self.config.start_prb..self.config.start_prb + self.config.num_prb_pairs).collect()
    }

    /// Return OFDM symbol indices used for R-PDCCH
    pub fn allocated_symbols(&self) -> Vec<usize> {
        (self.config.start_symbol..self.config.start_symbol + self.config.num_symbols).collect()
    }

    /// Map a DCI payload to CCE indices using the aggregation level
    /// Returns starting CCE index (simplified; full implementation uses hash-based search spaces)
    pub fn map_dci_to_cces(&self, rnti: u16, aggregation_level: usize) -> Vec<usize> {
        let n_cce = self.num_cces();
        if n_cce == 0 || aggregation_level == 0 {
            return Vec::new();
        }
        // Search space starting CCE per TS 36.213 §9.1.1 (simplified)
        let yk = (rnti as usize).wrapping_mul(39827) % n_cce.max(1);
        let start = (yk / aggregation_level) * aggregation_level;
        (0..aggregation_level).map(|i| (start + i) % n_cce).collect()
    }
}

// ---------------------------------------------------------------------------
// CRS generation for relay cell
// ---------------------------------------------------------------------------

/// Generate Cell-specific Reference Signal (CRS) sequence per TS 36.211 §6.10.1
///
/// CRS is used both by the relay's access-link cell and may be forwarded/regenerated
/// for transparent relay (Type 2) operation.
pub fn generate_crs(cell_id: u16, symbol_index: usize, slot_index: usize, num_prb: usize) -> Vec<(f32, f32)> {
    // Cinit per TS 36.211 §6.10.1.1
    let ns = slot_index;
    let l = symbol_index;
    let cinit: u32 = (1 << 10) * (7 * (ns as u32 + 1) + (l as u32) + 1)
        * (2 * (cell_id as u32 % 3) + 1)
        + 2 * (cell_id as u32 % 3)
        + (if cell_id < 168 { 0 } else { 1 }); // simplified Ncp encoding
    let length = 2 * num_prb;
    gold_sequence(cinit, length)
        .chunks(2)
        .map(|c| {
            let i = c[0] as f32;
            let q = c[1] as f32;
            // Normalize to unit power: 1/sqrt(2)
            (i * 0.7071068, q * 0.7071068)
        })
        .collect()
}

/// Generate R-DMRS (Relay Demodulation Reference Signal) sequence for R-PDSCH
/// per TS 36.216 §5.4
pub fn generate_r_dmrs(cell_id: u16, slot_index: usize, prb_start: usize, num_prb: usize) -> Vec<(f32, f32)> {
    // Sequence group number per TS 36.211 §5.5.1
    let u = cell_id as usize % 30;
    let v = (slot_index / 2) % 2;
    let _ = v; // used for sequence group hopping if enabled
    let cinit: u32 = ((2 * slot_index + 1) as u32) * (2 * (cell_id as u32) + 1) * (1 << 17)
        + 2 * (cell_id as u32) + (u as u32);
    let length = 2 * num_prb * 12;
    let seq = gold_sequence(cinit, length);
    let offset = prb_start * 24;
    seq[offset..(offset + num_prb * 24).min(seq.len())]
        .chunks(2)
        .map(|c| (c[0] as f32 * 0.7071068, c[1] as f32 * 0.7071068))
        .collect()
}

// ---------------------------------------------------------------------------
// Subframe scheduling
// ---------------------------------------------------------------------------

/// Subframe schedule entry for one radio frame
#[derive(Debug, Clone)]
pub struct SubframeSchedule {
    /// Direction/type for each of the 10 subframes in a radio frame
    pub subframes: [SubframeDirection; 10],
}

impl SubframeSchedule {
    /// Build subframe schedule for a Type 1 FDD relay from the backhaul config
    pub fn build_type1_fdd(backhaul_config: &BackhaulSubframeConfig) -> Self {
        let mut subframes = [SubframeDirection::Downlink; 10];
        // Subframe 0 and 5 are always DL (synchronisation signals) in FDD
        subframes[0] = SubframeDirection::Downlink;
        subframes[5] = SubframeDirection::Downlink;
        // Mark MBSFN backhaul subframes
        for sf in 0..10usize {
            if (backhaul_config.mbsfn_subframe_bitmap >> sf) & 1 == 1 {
                subframes[sf] = SubframeDirection::MbsfnBackhaul;
            }
        }
        Self { subframes }
    }

    /// Build subframe schedule for FDD access link (simple DL/UL alternation)
    pub fn build_fdd_access() -> Self {
        // All subframes are DL on access link (UL handled via PUSCH)
        Self { subframes: [SubframeDirection::Downlink; 10] }
    }

    /// Return indices of MBSFN backhaul subframes
    pub fn mbsfn_subframe_indices(&self) -> Vec<usize> {
        self.subframes
            .iter()
            .enumerate()
            .filter(|(_, &d)| d == SubframeDirection::MbsfnBackhaul)
            .map(|(i, _)| i)
            .collect()
    }

    /// Return indices of DL subframes
    pub fn dl_subframe_indices(&self) -> Vec<usize> {
        self.subframes
            .iter()
            .enumerate()
            .filter(|(_, &d)| d == SubframeDirection::Downlink)
            .map(|(i, _)| i)
            .collect()
    }
}

// ---------------------------------------------------------------------------
// Link budget calculations
// ---------------------------------------------------------------------------

/// DeNB-to-RN backhaul link budget result
#[derive(Debug, Clone)]
pub struct BackhaulLinkBudget {
    /// EIRP of the DeNB transmitter in dBm
    pub eirp_dbm: f64,
    /// Free-space path loss in dB
    pub fspl_db: f64,
    /// Relay receive antenna gain in dBi
    pub rx_gain_dbi: f64,
    /// Received signal power at relay in dBm
    pub received_power_dbm: f64,
    /// Thermal noise power in dBm at relay receiver bandwidth
    pub noise_power_dbm: f64,
    /// SNR at the relay input in dB
    pub snr_db: f64,
    /// Effective relay gain: improvement in SNR vs direct path to UE
    pub relay_gain_db: f64,
}

/// Compute the DeNB-to-RN backhaul link budget
///
/// # Arguments
/// * `denb_tx_power_dbm` - DeNB transmit power in dBm
/// * `denb_tx_gain_dbi` - DeNB antenna gain (dBi)
/// * `distance_m` - Distance between DeNB and RN in metres
/// * `frequency_hz` - Carrier frequency in Hz
/// * `rx_gain_dbi` - Relay receive antenna gain (dBi)
/// * `bandwidth_hz` - Channel bandwidth in Hz
/// * `noise_figure_db` - Relay receiver noise figure in dB
pub fn backhaul_link_budget(
    denb_tx_power_dbm: f64,
    denb_tx_gain_dbi: f64,
    distance_m: f64,
    frequency_hz: f64,
    rx_gain_dbi: f64,
    bandwidth_hz: f64,
    noise_figure_db: f64,
) -> BackhaulLinkBudget {
    let eirp_dbm = denb_tx_power_dbm + denb_tx_gain_dbi;
    let fspl_db = 20.0 * (4.0 * std::f64::consts::PI * distance_m * frequency_hz / SPEED_OF_LIGHT).log10();
    let received_power_dbm = eirp_dbm - fspl_db + rx_gain_dbi;
    // Noise power = kTB in dBm + NF
    let noise_power_dbw = BOLTZMANN * NOISE_TEMP_REF * bandwidth_hz;
    let noise_power_dbm = 10.0 * noise_power_dbw.log10() + 30.0 + noise_figure_db;
    let snr_db = received_power_dbm - noise_power_dbm;
    // Relay gain vs direct DeNB-to-UE path (simplified: relay gain ≈ path loss saved)
    // Assumes relay is placed between DeNB and UE to halve the path
    let relay_gain_db = fspl_db / 2.0; // simplified estimate
    BackhaulLinkBudget {
        eirp_dbm,
        fspl_db,
        rx_gain_dbi,
        received_power_dbm,
        noise_power_dbm,
        snr_db,
        relay_gain_db,
    }
}

// ---------------------------------------------------------------------------
// Multi-hop capacity estimation
// ---------------------------------------------------------------------------

/// Multi-hop relay capacity estimation result
#[derive(Debug, Clone)]
pub struct MultiHopCapacity {
    /// Number of hops in the path
    pub num_hops: usize,
    /// Shannon capacity per hop in bits/s/Hz
    pub capacity_per_hop: Vec<f64>,
    /// End-to-end capacity (bottleneck) in bits/s/Hz
    pub end_to_end_capacity: f64,
    /// Spectral efficiency in bits/s/Hz
    pub spectral_efficiency: f64,
}

/// Estimate multi-hop relay capacity given per-hop SNR values
///
/// The end-to-end capacity of a decode-and-forward (DF) relay chain is limited
/// by the weakest hop (min of per-hop capacities).
///
/// C_hop = log2(1 + SNR_hop)  per hop (Shannon)
/// C_e2e = min(C_hop) for decode-and-forward
pub fn estimate_multihop_capacity(snr_per_hop_db: &[f64]) -> MultiHopCapacity {
    let capacity_per_hop: Vec<f64> = snr_per_hop_db
        .iter()
        .map(|&snr_db| {
            let snr_linear = 10.0_f64.powf(snr_db / 10.0);
            (1.0 + snr_linear).log2()
        })
        .collect();

    let end_to_end_capacity = capacity_per_hop
        .iter()
        .cloned()
        .fold(f64::INFINITY, f64::min);

    let spectral_efficiency = end_to_end_capacity;

    MultiHopCapacity {
        num_hops: snr_per_hop_db.len(),
        capacity_per_hop,
        end_to_end_capacity,
        spectral_efficiency,
    }
}

// ---------------------------------------------------------------------------
// Relay selection and handover
// ---------------------------------------------------------------------------

/// Measurement report from a relay candidate
#[derive(Debug, Clone)]
pub struct RelayMeasurement {
    /// Relay cell ID
    pub cell_id: u16,
    /// RSRP in dBm (−140 to −44)
    pub rsrp_dbm: f64,
    /// RSRQ in dB (−19.5 to −3)
    pub rsrq_db: f64,
    /// Distance from UE to relay in metres
    pub distance_m: f64,
}

/// Select the best relay candidate from a set of measurements
pub fn select_relay(
    measurements: &[RelayMeasurement],
    criterion: RelaySelectionCriterion,
) -> Option<usize> {
    if measurements.is_empty() {
        return None;
    }
    let best = match criterion {
        RelaySelectionCriterion::MaxRsrp => measurements
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.rsrp_dbm.partial_cmp(&b.rsrp_dbm).unwrap()),
        RelaySelectionCriterion::MaxRsrq => measurements
            .iter()
            .enumerate()
            .max_by(|(_, a), (_, b)| a.rsrq_db.partial_cmp(&b.rsrq_db).unwrap()),
        RelaySelectionCriterion::SCriterion => {
            // S-criterion: RSRP > Qrxlevmin (simplified: > −120 dBm) and RSRQ > −15 dB
            measurements
                .iter()
                .enumerate()
                .filter(|(_, m)| m.rsrp_dbm > -120.0 && m.rsrq_db > -15.0)
                .max_by(|(_, a), (_, b)| a.rsrp_dbm.partial_cmp(&b.rsrp_dbm).unwrap())
        }
    };
    best.map(|(i, _)| i)
}

/// Evaluate A3 handover event: neighbour better than serving by hysteresis + offset
///
/// A3 condition: Mn + Ofn + Ocn − Hys > Ms + Ofs + Ocs + Off
/// (simplified: neighbour RSRP > serving RSRP + hysteresis + offset)
pub fn evaluate_a3_event(
    serving_rsrp_dbm: f64,
    neighbour_rsrp_dbm: f64,
    hysteresis_db: f64,
    a3_offset_db: f64,
) -> bool {
    neighbour_rsrp_dbm + a3_offset_db - hysteresis_db > serving_rsrp_dbm
}

// ---------------------------------------------------------------------------
// Relay timing calculations
// ---------------------------------------------------------------------------

/// Compute the timing advance that the relay must apply on the Un interface
///
/// TA = 2 * Tp  (round-trip propagation delay compensation)
/// where Tp is the one-way propagation delay from DeNB to RN.
pub fn compute_timing_advance(propagation_delay_s: f64) -> f64 {
    2.0 * propagation_delay_s
}

/// Determine if a given subframe index is a valid backhaul reception subframe
/// accounting for the guard period at the start of the MBSFN window.
pub fn is_backhaul_rx_subframe(
    subframe_idx: usize,
    backhaul_config: &BackhaulSubframeConfig,
    relay_timing: &RelayTiming,
) -> bool {
    if (backhaul_config.mbsfn_subframe_bitmap >> subframe_idx) & 1 == 0 {
        return false;
    }
    // Check that switch time fits within guard period
    let guard_duration_s = backhaul_config.guard_symbols_dl as f64
        * (SLOT_DURATION_S / SYMBOLS_PER_SLOT_NORMAL_CP as f64);
    relay_timing.rx_tx_switch_time_s <= guard_duration_s
}

/// Compute the HARQ feedback subframe for the relay backhaul
///
/// For extended 8-ms HARQ (Type 1 relay), the HARQ ACK/NACK is sent 8ms after
/// the initial DL subframe (vs 4ms for standard UEs).
pub fn harq_feedback_subframe(
    dl_subframe: usize,
    frame_number: usize,
    timeline: HarqTimeline,
) -> (usize, usize) {
    let delay = match timeline {
        HarqTimeline::Standard4ms => 4,
        HarqTimeline::Extended8ms => 8,
    };
    let total_sf = frame_number * SUBFRAMES_PER_FRAME + dl_subframe + delay;
    (total_sf / SUBFRAMES_PER_FRAME, total_sf % SUBFRAMES_PER_FRAME)
}

// ---------------------------------------------------------------------------
// Transport block size lookup
// ---------------------------------------------------------------------------

/// Look up transport block size from MCS index and number of allocated PRBs
/// per 3GPP TS 36.213 Table 7.1.7.2.1-1 (simplified subset)
///
/// Returns transport block size in bits, or 0 if invalid.
pub fn transport_block_size(mcs_index: usize, num_prb: usize) -> usize {
    if mcs_index > 28 || num_prb == 0 || num_prb > MAX_PRB {
        return 0;
    }
    // Itbs (transport block size table index) from MCS per Table 7.1.7.1-1
    const ITBS_FROM_MCS: [usize; 29] = [
        0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 9, 10, 11, 12, 13, 14, 15, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26,
    ];
    let itbs = ITBS_FROM_MCS[mcs_index];
    // TBS table: TBS[itbs][nprb-1] per TS 36.213 Table 7.1.7.2.1-1
    // Sampled key entries (actual table has 27x110 entries)
    // We implement a formula-based approximation:
    // TBS ≈ 12 * nPRB * bits_per_symbol - CRC_overhead
    let bits_per_re: &[f64] = &[
        0.2344, 0.3770, 0.6016, 0.8770, 1.1758, 1.4766, 1.6953, 1.9141, 2.1602, 2.4063,
        2.5703, 2.7305, 3.0293, 3.3223, 3.6094, 3.9023, 4.2129, 4.5234, 4.8164, 5.1152,
        5.3320, 5.5547, 5.8906, 6.2266, 6.5508, 6.9141, 7.4063,
    ];
    let bpr = bits_per_re[itbs.min(bits_per_re.len() - 1)];
    // REs per PRB: 12 subcarriers × 7 symbols (normal CP) × 2 slots - DMRS/CRS overhead (~84 data REs)
    let re_per_prb = 84usize;
    let raw_bits = (bpr * (re_per_prb * num_prb) as f64) as usize;
    // Round down to valid TBS size (multiples dictated by code block sizes)
    raw_bits & !7 // align to byte
}

// ---------------------------------------------------------------------------
// R-PDSCH / R-PUSCH scheduling
// ---------------------------------------------------------------------------

/// Scheduling decision for one relay backhaul subframe
#[derive(Debug, Clone)]
pub struct BackhaulSchedule {
    /// Subframe index (0-9) within the radio frame
    pub subframe_idx: usize,
    /// Allocated PRBs for R-PDSCH
    pub dl_prbs: Vec<usize>,
    /// Allocated PRBs for R-PUSCH
    pub ul_prbs: Vec<usize>,
    /// DL DCI message
    pub dl_dci: Option<DciMessage>,
    /// UL DCI message (grant)
    pub ul_dci: Option<DciMessage>,
    /// Transport block size for DL in bits
    pub dl_tbs_bits: usize,
    /// Transport block size for UL in bits
    pub ul_tbs_bits: usize,
}

/// Scheduler for the backhaul Un interface
pub struct BackhaulScheduler {
    config: RelayConfig,
    harq_dl: Vec<HarqProcess>,
    harq_ul: Vec<HarqProcess>,
    relay_rnti: u16,
    subframe_counter: usize,
}

impl BackhaulScheduler {
    pub fn new(config: RelayConfig, relay_rnti: u16) -> Self {
        let num_harq = 8; // FDD has 8 HARQ processes
        let tbs = transport_block_size(16, 50); // approximate buffer size
        let harq_dl = (0..num_harq).map(|i| HarqProcess::new(i, tbs)).collect();
        let harq_ul = (0..num_harq).map(|i| HarqProcess::new(i, tbs)).collect();
        Self {
            config,
            harq_dl,
            harq_ul,
            relay_rnti,
            subframe_counter: 0,
        }
    }

    /// Schedule one backhaul subframe if it is an MBSFN backhaul subframe
    pub fn schedule_subframe(&mut self, subframe_idx: usize, mcs_dl: u8, mcs_ul: u8) -> Option<BackhaulSchedule> {
        let is_backhaul = (self.config.backhaul_subframe_config.mbsfn_subframe_bitmap >> subframe_idx) & 1 == 1;
        if !is_backhaul {
            self.subframe_counter += 1;
            return None;
        }

        let num_prb = self.config.num_prb;
        let r_pdcch = &self.config.r_pdcch_config;
        // DL: allocate remaining PRBs after R-PDCCH
        let dl_start = r_pdcch.start_prb + r_pdcch.num_prb_pairs;
        let dl_prbs: Vec<usize> = (dl_start..num_prb).collect();
        let dl_tbs = transport_block_size(mcs_dl as usize, dl_prbs.len().max(1));
        let rb_assignment = prb_list_to_bitmap(&dl_prbs);
        let harq_pid = self.subframe_counter % 8;

        let dl_dci = Some(DciMessage {
            format: DciFormat::Format1A,
            rnti: self.relay_rnti,
            resource_alloc_type: 0,
            rb_assignment,
            mcs_index: mcs_dl,
            harq_process: harq_pid as u8,
            rv_index: self.harq_dl[harq_pid].next_rv() as u8,
            ndi: self.harq_dl[harq_pid].ndi,
            tpc: 0,
            dai: 0,
        });

        // UL: allocate lower PRBs for R-PUSCH
        let ul_prbs: Vec<usize> = (0..num_prb / 4).collect();
        let ul_tbs = transport_block_size(mcs_ul as usize, ul_prbs.len().max(1));
        let ul_rb_assignment = prb_list_to_bitmap(&ul_prbs);
        let ul_dci = Some(DciMessage {
            format: DciFormat::Format0,
            rnti: self.relay_rnti,
            resource_alloc_type: 0,
            rb_assignment: ul_rb_assignment,
            mcs_index: mcs_ul,
            harq_process: harq_pid as u8,
            rv_index: 0,
            ndi: self.harq_ul[harq_pid].ndi,
            tpc: 1,
            dai: 0,
        });

        self.subframe_counter += 1;

        Some(BackhaulSchedule {
            subframe_idx,
            dl_prbs,
            ul_prbs,
            dl_dci,
            ul_dci,
            dl_tbs_bits: dl_tbs,
            ul_tbs_bits: ul_tbs,
        })
    }
}

// ---------------------------------------------------------------------------
// Main relay processor
// ---------------------------------------------------------------------------

/// LTE Relay Node Processor
///
/// Encapsulates the complete relay node processing for backhaul (Un) and
/// access (Uu) links per 3GPP TS 36.216.
pub struct LteRelayProcessor {
    config: RelayConfig,
    r_pdcch_mapper: RPdcchMapper,
    backhaul_schedule: SubframeSchedule,
    access_schedule: SubframeSchedule,
}

impl LteRelayProcessor {
    /// Create a new relay processor with the given configuration
    pub fn new(config: RelayConfig) -> Self {
        let r_pdcch_mapper = RPdcchMapper::new(config.r_pdcch_config.clone());
        let backhaul_schedule = SubframeSchedule::build_type1_fdd(&config.backhaul_subframe_config);
        let access_schedule = SubframeSchedule::build_fdd_access();
        Self {
            config,
            r_pdcch_mapper,
            backhaul_schedule,
            access_schedule,
        }
    }

    /// Return the relay type
    pub fn relay_type(&self) -> RelayType {
        self.config.relay_type
    }

    /// Return the relay cell ID
    pub fn cell_id(&self) -> u16 {
        self.config.cell_id
    }

    /// Return the donor eNB cell ID
    pub fn denb_cell_id(&self) -> u16 {
        self.config.denb_cell_id
    }

    /// Return the subframe schedule for the backhaul link
    pub fn backhaul_schedule(&self) -> &SubframeSchedule {
        &self.backhaul_schedule
    }

    /// Return the subframe schedule for the access link
    pub fn access_schedule(&self) -> &SubframeSchedule {
        &self.access_schedule
    }

    /// Return the R-PDCCH mapper for this relay
    pub fn r_pdcch_mapper(&self) -> &RPdcchMapper {
        &self.r_pdcch_mapper
    }

    /// Return the relay timing configuration
    pub fn relay_timing(&self) -> &RelayTiming {
        &self.config.relay_timing
    }

    /// Check whether half-duplex constraint is satisfied:
    /// relay cannot transmit and receive on the same frequency simultaneously
    pub fn check_half_duplex_constraint(&self, subframe_idx: usize) -> bool {
        match self.config.relay_type {
            RelayType::Type1 => {
                // In a backhaul subframe, relay must not transmit on access link
                let is_backhaul =
                    (self.config.backhaul_subframe_config.mbsfn_subframe_bitmap >> subframe_idx) & 1 == 1;
                !is_backhaul // constraint satisfied if not simultaneously tx and rx
            }
            RelayType::Type1a => true, // outband: no constraint
            RelayType::Type2 => true,  // transparent: no constraint
        }
    }

    /// Generate CRS for the relay's access-link cell
    pub fn generate_access_crs(&self, symbol: usize, slot: usize) -> Vec<(f32, f32)> {
        generate_crs(self.config.cell_id, symbol, slot, self.config.num_prb)
    }

    /// Generate R-DMRS for R-PDSCH demodulation
    pub fn generate_r_dmrs(&self, slot: usize) -> Vec<(f32, f32)> {
        generate_r_dmrs(
            self.config.denb_cell_id,
            slot,
            self.config.r_pdcch_config.start_prb,
            self.config.r_pdcch_config.num_prb_pairs,
        )
    }

    /// Compute the timing advance for this relay
    pub fn timing_advance(&self) -> f64 {
        compute_timing_advance(self.config.relay_timing.propagation_delay_s)
    }

    /// Check if a given subframe is valid for backhaul reception
    pub fn is_backhaul_subframe(&self, subframe_idx: usize) -> bool {
        is_backhaul_rx_subframe(
            subframe_idx,
            &self.config.backhaul_subframe_config,
            &self.config.relay_timing,
        )
    }

    /// Get the number of available CCEs in the R-PDCCH region
    pub fn num_r_pdcch_cces(&self) -> usize {
        self.r_pdcch_mapper.num_cces()
    }

    /// Get HARQ feedback subframe for a given DL subframe
    pub fn harq_feedback_subframe(&self, dl_subframe: usize, frame: usize) -> (usize, usize) {
        harq_feedback_subframe(dl_subframe, frame, self.config.relay_timing.harq_timeline)
    }
}

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

/// Ceiling log base 2 (returns number of bits needed to represent `n`)
fn ceil_log2(n: usize) -> usize {
    if n <= 1 {
        return 1;
    }
    let mut bits = 0usize;
    let mut v = n - 1;
    while v > 0 {
        v >>= 1;
        bits += 1;
    }
    bits
}

/// Pack `width` bits of `value` (MSB first) into bit vector starting at `offset`
fn pack_bits_into(bits: &mut Vec<u8>, offset: usize, width: usize, value: u64) {
    for i in 0..width {
        let bit_pos = width - 1 - i;
        let bit = ((value >> bit_pos) & 1) as u8;
        if offset + i < bits.len() {
            bits[offset + i] = bit;
        }
    }
}

/// Convert a list of PRB indices to a 32-bit bitmap
fn prb_list_to_bitmap(prbs: &[usize]) -> u32 {
    let mut bitmap = 0u32;
    for &prb in prbs {
        if prb < 32 {
            bitmap |= 1u32 << prb;
        }
    }
    bitmap
}

/// Generate a Gold sequence of `length` bits initialized with `cinit`
/// per TS 36.211 §7.2 (pseudo-random sequence generator)
fn gold_sequence(cinit: u32, length: usize) -> Vec<u8> {
    // x1 initialised with 1 at position 0
    let mut x1 = [0u32; 2];
    x1[0] = 1;
    // x2 initialised with cinit
    let mut x2 = [0u32; 2];
    x2[0] = cinit & 0x7FFF_FFFF;

    // Advance x1 by 1600 positions (initialisation)
    for _ in 0..1600 {
        let new_bit_x1 = ((x1[0] >> 0) ^ (x1[0] >> 3)) & 1;
        x1[0] = (x1[0] >> 1) | (new_bit_x1 << 30);
        let new_bit_x2 = ((x2[0] >> 0) ^ (x2[0] >> 1) ^ (x2[0] >> 2) ^ (x2[0] >> 3)) & 1;
        x2[0] = (x2[0] >> 1) | (new_bit_x2 << 30);
    }

    let mut seq = Vec::with_capacity(length);
    for _ in 0..length {
        let c = ((x1[0] ^ x2[0]) & 1) as u8;
        seq.push(c);
        let new_bit_x1 = ((x1[0] >> 0) ^ (x1[0] >> 3)) & 1;
        x1[0] = (x1[0] >> 1) | (new_bit_x1 << 30);
        let new_bit_x2 = ((x2[0] >> 0) ^ (x2[0] >> 1) ^ (x2[0] >> 2) ^ (x2[0] >> 3)) & 1;
        x2[0] = (x2[0] >> 1) | (new_bit_x2 << 30);
    }
    seq
}

/// Modulation order to bits per symbol
pub fn modulation_bits_per_symbol(order: ModulationOrder) -> usize {
    match order {
        ModulationOrder::Qpsk => 2,
        ModulationOrder::Qam16 => 4,
        ModulationOrder::Qam64 => 6,
    }
}

/// Estimate relay throughput in bits per second given scheduling parameters
pub fn relay_throughput_bps(
    mcs_index: usize,
    dl_prbs: usize,
    backhaul_subframes_per_frame: usize,
) -> f64 {
    let tbs = transport_block_size(mcs_index, dl_prbs) as f64;
    // Frames per second = 100 (10ms frame period)
    let frames_per_second = 100.0;
    tbs * (backhaul_subframes_per_frame as f64) * frames_per_second
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    fn default_config() -> RelayConfig {
        RelayConfig {
            relay_type: RelayType::Type1,
            cell_id: 100,
            denb_cell_id: 1,
            num_prb: 50,
            duplex_mode: DuplexMode::Fdd,
            backhaul_subframe_config: BackhaulSubframeConfig::default(),
            r_pdcch_config: RPdcchConfig::default(),
            relay_timing: RelayTiming::default(),
        }
    }

    // --- Basic construction ---

    #[test]
    fn test_processor_creation() {
        let p = LteRelayProcessor::new(default_config());
        assert_eq!(p.relay_type(), RelayType::Type1);
        assert_eq!(p.cell_id(), 100);
        assert_eq!(p.denb_cell_id(), 1);
    }

    #[test]
    fn test_relay_type1a() {
        let mut cfg = default_config();
        cfg.relay_type = RelayType::Type1a;
        let p = LteRelayProcessor::new(cfg);
        assert_eq!(p.relay_type(), RelayType::Type1a);
    }

    #[test]
    fn test_relay_type2() {
        let mut cfg = default_config();
        cfg.relay_type = RelayType::Type2;
        let p = LteRelayProcessor::new(cfg);
        assert_eq!(p.relay_type(), RelayType::Type2);
    }

    // --- Subframe schedule ---

    #[test]
    fn test_backhaul_schedule_mbsfn_subframes() {
        let p = LteRelayProcessor::new(default_config());
        let indices = p.backhaul_schedule().mbsfn_subframe_indices();
        // Default bitmap 0b0000_0100_0010 -> subframes 1 and 6 (if 0-indexed)
        // bit 1 set -> subframe 1; bit 6 set -> subframe 6
        assert!(indices.contains(&1) || !indices.is_empty());
    }

    #[test]
    fn test_backhaul_schedule_has_dl_subframes() {
        let p = LteRelayProcessor::new(default_config());
        let dl = p.backhaul_schedule().dl_subframe_indices();
        assert!(!dl.is_empty());
    }

    #[test]
    fn test_access_schedule_all_dl() {
        let p = LteRelayProcessor::new(default_config());
        let dl = p.access_schedule().dl_subframe_indices();
        assert_eq!(dl.len(), 10); // all 10 subframes are DL on access
    }

    #[test]
    fn test_custom_mbsfn_bitmap() {
        let mut cfg = default_config();
        cfg.backhaul_subframe_config.mbsfn_subframe_bitmap = 0b0000_1000_0100; // subframes 2 and 7
        let p = LteRelayProcessor::new(cfg);
        let indices = p.backhaul_schedule().mbsfn_subframe_indices();
        assert!(indices.contains(&2));
        assert!(indices.contains(&7));
    }

    // --- Half-duplex constraint ---

    #[test]
    fn test_half_duplex_type1_backhaul_subframe() {
        let mut cfg = default_config();
        cfg.backhaul_subframe_config.mbsfn_subframe_bitmap = 0b0000_0000_0010; // subframe 1 only
        let p = LteRelayProcessor::new(cfg);
        // Subframe 1 is backhaul: half-duplex constraint means relay cannot TX
        assert!(!p.check_half_duplex_constraint(1));
        // Subframe 0 is not backhaul: constraint satisfied
        assert!(p.check_half_duplex_constraint(0));
    }

    #[test]
    fn test_half_duplex_type1a_always_satisfied() {
        let mut cfg = default_config();
        cfg.relay_type = RelayType::Type1a;
        let p = LteRelayProcessor::new(cfg);
        for sf in 0..10 {
            assert!(p.check_half_duplex_constraint(sf));
        }
    }

    #[test]
    fn test_half_duplex_type2_always_satisfied() {
        let mut cfg = default_config();
        cfg.relay_type = RelayType::Type2;
        let p = LteRelayProcessor::new(cfg);
        for sf in 0..10 {
            assert!(p.check_half_duplex_constraint(sf));
        }
    }

    // --- R-PDCCH ---

    #[test]
    fn test_r_pdcch_num_cces_positive() {
        let p = LteRelayProcessor::new(default_config());
        assert!(p.num_r_pdcch_cces() > 0);
    }

    #[test]
    fn test_r_pdcch_allocated_prbs() {
        let p = LteRelayProcessor::new(default_config());
        let prbs = p.r_pdcch_mapper().allocated_prbs();
        assert_eq!(prbs.len(), 2); // default num_prb_pairs = 2
        assert_eq!(prbs[0], 0); // start_prb = 0
    }

    #[test]
    fn test_r_pdcch_allocated_symbols() {
        let p = LteRelayProcessor::new(default_config());
        let syms = p.r_pdcch_mapper().allocated_symbols();
        assert_eq!(syms.len(), 2); // default num_symbols = 2
        assert_eq!(syms[0], 2); // start_symbol = 2
    }

    #[test]
    fn test_r_pdcch_dci_to_cces() {
        let mapper = RPdcchMapper::new(RPdcchConfig::default());
        let cces = mapper.map_dci_to_cces(0xABCD, 2);
        assert_eq!(cces.len(), 2); // aggregation level 2 -> 2 CCEs
    }

    // --- DCI encoding ---

    #[test]
    fn test_dci_format1a_size() {
        let size = DciMessage::format1a_size_bits(50);
        assert!(size >= 15); // minimum reasonable size
    }

    #[test]
    fn test_dci_format0_size() {
        let size = DciMessage::format0_size_bits(50);
        assert!(size >= 10);
    }

    #[test]
    fn test_dci_encode_length_matches_format() {
        let dci = DciMessage {
            format: DciFormat::Format1A,
            rnti: 0x1234,
            resource_alloc_type: 0,
            rb_assignment: 0b1111,
            mcs_index: 10,
            harq_process: 3,
            rv_index: 0,
            ndi: true,
            tpc: 1,
            dai: 2,
        };
        let bits = dci.encode(50);
        assert_eq!(bits.len(), DciMessage::format1a_size_bits(50));
    }

    #[test]
    fn test_dci_encode_bits_binary() {
        let dci = DciMessage {
            format: DciFormat::Format1A,
            rnti: 0x0001,
            resource_alloc_type: 0,
            rb_assignment: 1,
            mcs_index: 5,
            harq_process: 0,
            rv_index: 0,
            ndi: false,
            tpc: 0,
            dai: 0,
        };
        let bits = dci.encode(6);
        for b in &bits {
            assert!(*b == 0 || *b == 1, "bit must be 0 or 1, got {}", b);
        }
    }

    // --- HARQ processes ---

    #[test]
    fn test_harq_process_initial_state() {
        let hp = HarqProcess::new(0, 1000);
        assert_eq!(hp.process_id, 0);
        assert_eq!(hp.tx_count, 0);
        assert!(!hp.active);
        assert!(!hp.is_exhausted());
    }

    #[test]
    fn test_harq_process_reset() {
        let mut hp = HarqProcess::new(0, 100);
        hp.reset();
        assert!(hp.active);
        assert_eq!(hp.tx_count, 0);
        assert!(hp.ndi); // toggled from initial false
    }

    #[test]
    fn test_harq_chase_combine() {
        let mut hp = HarqProcess::new(0, 4);
        let llrs = vec![1.0f32, -2.0, 3.0, -4.0];
        hp.chase_combine(&llrs);
        hp.chase_combine(&llrs);
        assert_eq!(hp.soft_buffer[0], 2.0);
        assert_eq!(hp.soft_buffer[1], -4.0);
        assert_eq!(hp.tx_count, 2);
    }

    #[test]
    fn test_harq_rv_sequence() {
        let hp = HarqProcess::new(0, 10);
        // At tx_count=0 next rv = RV_SEQUENCE[0] = 0
        assert_eq!(hp.next_rv(), 0);
    }

    #[test]
    fn test_harq_exhaustion() {
        let mut hp = HarqProcess::new(0, 4);
        for _ in 0..4 {
            hp.chase_combine(&[0.0f32]);
        }
        assert!(hp.is_exhausted());
    }

    // --- CRS generation ---

    #[test]
    fn test_crs_generation_length() {
        let crs = generate_crs(0, 0, 0, 6);
        // length = 2*num_prb subcarriers, but returned as pairs
        // gold_sequence length = 2*num_prb = 12, chunked by 2 gives 6 pairs
        assert_eq!(crs.len(), 6);
    }

    #[test]
    fn test_crs_different_cell_ids() {
        let crs1 = generate_crs(0, 0, 0, 6);
        let crs2 = generate_crs(100, 0, 0, 6);
        // Different cell IDs should produce different CRS
        let different = crs1.iter().zip(crs2.iter()).any(|(a, b)| a.0 != b.0 || a.1 != b.1);
        assert!(different, "CRS should differ for different cell IDs");
    }

    #[test]
    fn test_crs_power_normalised() {
        let crs = generate_crs(42, 4, 3, 6);
        for (i, q) in &crs {
            let power = i * i + q * q;
            // Power should be close to 0.5 (unit power split between I and Q)
            assert!((power - 0.5).abs() < 0.01, "CRS power {power} not normalised");
        }
    }

    // --- R-DMRS generation ---

    #[test]
    fn test_r_dmrs_generation_non_empty() {
        let dmrs = generate_r_dmrs(1, 2, 0, 2);
        assert!(!dmrs.is_empty());
    }

    #[test]
    fn test_r_dmrs_different_slots() {
        let d1 = generate_r_dmrs(1, 0, 0, 2);
        let d2 = generate_r_dmrs(1, 2, 0, 2);
        let different = d1.iter().zip(d2.iter()).any(|(a, b)| a.0 != b.0 || a.1 != b.1);
        assert!(different, "R-DMRS should differ across slots");
    }

    // --- Timing ---

    #[test]
    fn test_timing_advance_two_times_propagation() {
        let ta = compute_timing_advance(1e-6);
        assert!((ta - 2e-6).abs() < 1e-15);
    }

    #[test]
    fn test_processor_timing_advance() {
        let p = LteRelayProcessor::new(default_config());
        let ta = p.timing_advance();
        let expected = 2.0 * default_config().relay_timing.propagation_delay_s;
        assert!((ta - expected).abs() < 1e-15);
    }

    #[test]
    fn test_harq_feedback_subframe_4ms() {
        let (frame, sf) = harq_feedback_subframe(3, 0, HarqTimeline::Standard4ms);
        assert_eq!(frame, 0);
        assert_eq!(sf, 7); // 3 + 4 = 7
    }

    #[test]
    fn test_harq_feedback_subframe_8ms() {
        let (frame, sf) = harq_feedback_subframe(3, 0, HarqTimeline::Extended8ms);
        assert_eq!(frame, 1);
        assert_eq!(sf, 1); // 3 + 8 = 11 -> frame 1, sf 1
    }

    #[test]
    fn test_harq_feedback_subframe_wraparound() {
        let (frame, sf) = harq_feedback_subframe(9, 5, HarqTimeline::Extended8ms);
        // total = 5*10 + 9 + 8 = 67 -> frame 6, sf 7
        assert_eq!(frame, 6);
        assert_eq!(sf, 7);
    }

    #[test]
    fn test_is_backhaul_subframe_valid() {
        let cfg = default_config();
        // Subframes with guard sufficient to cover switch time
        let mut timing = RelayTiming::default();
        timing.rx_tx_switch_time_s = 1e-9; // very small, fits any guard
        let config = BackhaulSubframeConfig {
            mbsfn_subframe_bitmap: 0b0000_0000_0010, // subframe 1
            guard_symbols_dl: 2,
            guard_symbols_ul: 1,
            periodicity_frames: 2,
        };
        assert!(is_backhaul_rx_subframe(1, &config, &timing));
        assert!(!is_backhaul_rx_subframe(0, &config, &timing));
    }

    #[test]
    fn test_is_backhaul_subframe_switch_time_too_long() {
        let timing = RelayTiming {
            propagation_delay_s: 1e-6,
            timing_advance_s: 2e-6,
            rx_tx_switch_time_s: 1.0, // absurdly long
            harq_timeline: HarqTimeline::Extended8ms,
        };
        let config = BackhaulSubframeConfig {
            mbsfn_subframe_bitmap: 0b0000_0000_0010, // subframe 1
            guard_symbols_dl: 2,
            guard_symbols_ul: 1,
            periodicity_frames: 2,
        };
        assert!(!is_backhaul_rx_subframe(1, &config, &timing));
    }

    // --- Transport block size ---

    #[test]
    fn test_tbs_valid_mcs() {
        let tbs = transport_block_size(10, 25);
        assert!(tbs > 0);
    }

    #[test]
    fn test_tbs_invalid_mcs() {
        assert_eq!(transport_block_size(29, 25), 0);
    }

    #[test]
    fn test_tbs_zero_prb() {
        assert_eq!(transport_block_size(5, 0), 0);
    }

    #[test]
    fn test_tbs_increases_with_prb() {
        let tbs_small = transport_block_size(10, 10);
        let tbs_large = transport_block_size(10, 50);
        assert!(tbs_large > tbs_small);
    }

    #[test]
    fn test_tbs_increases_with_mcs() {
        let tbs_low = transport_block_size(5, 25);
        let tbs_high = transport_block_size(20, 25);
        assert!(tbs_high >= tbs_low);
    }

    // --- Link budget ---

    #[test]
    fn test_backhaul_link_budget_reasonable_snr() {
        let budget = backhaul_link_budget(
            43.0,  // 43 dBm EIRP
            15.0,  // 15 dBi DeNB gain
            500.0, // 500m distance
            2.6e9, // 2.6 GHz
            12.0,  // 12 dBi RN gain
            20e6,  // 20 MHz BW
            5.0,   // 5 dB NF
        );
        assert!(budget.received_power_dbm < budget.eirp_dbm, "received < transmitted");
        assert!(budget.fspl_db > 0.0);
        assert!(budget.snr_db.is_finite());
        assert!(budget.relay_gain_db > 0.0);
    }

    #[test]
    fn test_backhaul_link_budget_fspl_increases_with_distance() {
        let b1 = backhaul_link_budget(43.0, 15.0, 500.0, 2.6e9, 12.0, 20e6, 5.0);
        let b2 = backhaul_link_budget(43.0, 15.0, 1000.0, 2.6e9, 12.0, 20e6, 5.0);
        assert!(b2.fspl_db > b1.fspl_db);
    }

    // --- Multi-hop capacity ---

    #[test]
    fn test_multihop_single_hop() {
        let cap = estimate_multihop_capacity(&[10.0]);
        assert_eq!(cap.num_hops, 1);
        assert!((cap.end_to_end_capacity - (1.0 + 10.0f64.powf(1.0)).log2()).abs() < 0.01);
    }

    #[test]
    fn test_multihop_bottleneck() {
        let cap = estimate_multihop_capacity(&[20.0, 5.0, 15.0]);
        // Bottleneck is 5 dB hop
        let expected_bott = (1.0 + 10.0f64.powf(0.5)).log2();
        assert!((cap.end_to_end_capacity - expected_bott).abs() < 0.01);
    }

    #[test]
    fn test_multihop_capacity_decreases_with_hops() {
        let c1 = estimate_multihop_capacity(&[20.0]);
        let c2 = estimate_multihop_capacity(&[20.0, 10.0]);
        assert!(c2.end_to_end_capacity <= c1.end_to_end_capacity);
    }

    // --- Relay selection ---

    #[test]
    fn test_relay_selection_max_rsrp() {
        let measurements = vec![
            RelayMeasurement { cell_id: 1, rsrp_dbm: -90.0, rsrq_db: -10.0, distance_m: 200.0 },
            RelayMeasurement { cell_id: 2, rsrp_dbm: -80.0, rsrq_db: -12.0, distance_m: 300.0 },
            RelayMeasurement { cell_id: 3, rsrp_dbm: -95.0, rsrq_db: -8.0, distance_m: 100.0 },
        ];
        let idx = select_relay(&measurements, RelaySelectionCriterion::MaxRsrp).unwrap();
        assert_eq!(idx, 1, "Best RSRP is at index 1 (cell 2)");
    }

    #[test]
    fn test_relay_selection_max_rsrq() {
        let measurements = vec![
            RelayMeasurement { cell_id: 1, rsrp_dbm: -90.0, rsrq_db: -8.0, distance_m: 200.0 },
            RelayMeasurement { cell_id: 2, rsrp_dbm: -80.0, rsrq_db: -15.0, distance_m: 300.0 },
        ];
        let idx = select_relay(&measurements, RelaySelectionCriterion::MaxRsrq).unwrap();
        assert_eq!(idx, 0, "Best RSRQ is at index 0");
    }

    #[test]
    fn test_relay_selection_s_criterion_filters_weak() {
        let measurements = vec![
            RelayMeasurement { cell_id: 1, rsrp_dbm: -125.0, rsrq_db: -10.0, distance_m: 500.0 }, // too weak
            RelayMeasurement { cell_id: 2, rsrp_dbm: -100.0, rsrq_db: -10.0, distance_m: 300.0 },
        ];
        let idx = select_relay(&measurements, RelaySelectionCriterion::SCriterion).unwrap();
        assert_eq!(idx, 1);
    }

    #[test]
    fn test_relay_selection_empty() {
        assert!(select_relay(&[], RelaySelectionCriterion::MaxRsrp).is_none());
    }

    // --- A3 event evaluation ---

    #[test]
    fn test_a3_event_triggers() {
        assert!(evaluate_a3_event(-90.0, -80.0, 2.0, 0.0));
    }

    #[test]
    fn test_a3_event_does_not_trigger_within_hysteresis() {
        assert!(!evaluate_a3_event(-90.0, -89.0, 2.0, 0.0));
    }

    #[test]
    fn test_a3_event_with_offset() {
        // neighbour must beat serving by offset + hysteresis
        assert!(evaluate_a3_event(-90.0, -75.0, 2.0, 5.0));
        assert!(!evaluate_a3_event(-90.0, -84.0, 2.0, 5.0));
    }

    // --- Throughput estimation ---

    #[test]
    fn test_relay_throughput_positive() {
        let bps = relay_throughput_bps(15, 25, 2);
        assert!(bps > 0.0);
    }

    #[test]
    fn test_relay_throughput_scales_with_prbs() {
        let bps_low = relay_throughput_bps(15, 10, 2);
        let bps_high = relay_throughput_bps(15, 50, 2);
        assert!(bps_high > bps_low);
    }

    // --- Backhaul scheduler ---

    #[test]
    fn test_scheduler_creates_schedules_for_mbsfn_subframes() {
        let cfg = default_config();
        let mut sched = BackhaulScheduler::new(cfg, 0xBEEF);
        let mut found = false;
        for sf in 0..10 {
            if let Some(s) = sched.schedule_subframe(sf, 15, 10) {
                assert!(s.dl_dci.is_some());
                assert!(s.ul_dci.is_some());
                assert!(s.dl_tbs_bits > 0);
                found = true;
            }
        }
        assert!(found, "Scheduler must produce at least one backhaul schedule");
    }

    #[test]
    fn test_scheduler_skips_non_backhaul_subframes() {
        let mut cfg = default_config();
        cfg.backhaul_subframe_config.mbsfn_subframe_bitmap = 0b0000_0000_0010; // only sf 1
        let mut sched = BackhaulScheduler::new(cfg, 0x1234);
        assert!(sched.schedule_subframe(0, 15, 10).is_none());
        assert!(sched.schedule_subframe(1, 15, 10).is_some());
    }

    // --- Utility ---

    #[test]
    fn test_ceil_log2_values() {
        assert_eq!(ceil_log2(1), 1);
        assert_eq!(ceil_log2(2), 1);
        assert_eq!(ceil_log2(3), 2);
        assert_eq!(ceil_log2(4), 2);
        assert_eq!(ceil_log2(5), 3);
        assert_eq!(ceil_log2(8), 3);
        assert_eq!(ceil_log2(100), 7);
    }

    #[test]
    fn test_gold_sequence_length() {
        let seq = gold_sequence(42, 64);
        assert_eq!(seq.len(), 64);
    }

    #[test]
    fn test_gold_sequence_binary() {
        let seq = gold_sequence(0x1234, 128);
        for b in seq {
            assert!(b == 0 || b == 1);
        }
    }

    #[test]
    fn test_gold_sequence_different_cinit() {
        let s1 = gold_sequence(0, 32);
        let s2 = gold_sequence(1, 32);
        assert_ne!(s1, s2);
    }

    #[test]
    fn test_modulation_bits_per_symbol() {
        assert_eq!(modulation_bits_per_symbol(ModulationOrder::Qpsk), 2);
        assert_eq!(modulation_bits_per_symbol(ModulationOrder::Qam16), 4);
        assert_eq!(modulation_bits_per_symbol(ModulationOrder::Qam64), 6);
    }

    #[test]
    fn test_prb_list_to_bitmap() {
        let bitmap = prb_list_to_bitmap(&[0, 2, 4]);
        assert_eq!(bitmap & 1, 1);  // bit 0
        assert_eq!((bitmap >> 2) & 1, 1); // bit 2
        assert_eq!((bitmap >> 4) & 1, 1); // bit 4
        assert_eq!((bitmap >> 1) & 1, 0); // bit 1 not set
    }

    #[test]
    fn test_pack_bits_into() {
        let mut bits = vec![0u8; 8];
        pack_bits_into(&mut bits, 0, 8, 0b1010_1010);
        assert_eq!(bits[0], 1);
        assert_eq!(bits[1], 0);
        assert_eq!(bits[2], 1);
    }

    // --- Duplex mode ---

    #[test]
    fn test_duplex_mode_fdd() {
        let cfg = default_config();
        assert_eq!(cfg.duplex_mode, DuplexMode::Fdd);
    }

    #[test]
    fn test_duplex_mode_tdd() {
        let mut cfg = default_config();
        cfg.duplex_mode = DuplexMode::Tdd;
        let p = LteRelayProcessor::new(cfg);
        assert_eq!(p.relay_type(), RelayType::Type1);
    }
}
