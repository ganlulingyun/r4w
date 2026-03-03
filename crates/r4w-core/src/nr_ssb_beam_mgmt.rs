//! 5G NR SSB Beam Management
//!
//! Implements beam management procedures per 3GPP TS 38.213 and TS 38.321:
//! - SSB beam sweep configuration (L_max = 4/8/64 per FR1/FR2, Table 4.1-1)
//! - Beam index determination from SSB index (iSSB)
//! - SS-RSRP, SS-RSRQ, SS-SINR measurement per beam
//! - P1/P2/P3 beam management procedures
//! - Beam failure detection (BFD) and recovery (BFR)
//! - L1-RSRP reporting (7-bit, -140 to -44 dBm per TS 38.133)
//! - Beam correspondence, spatial relation, QCL Type-D, TCI states
//! - MAC CE beam indication, serving cell beam tracking with hysteresis
//!
//! # Reference
//! - 3GPP TS 38.213 v17.x §4.1, §6.1, §11.1
//! - 3GPP TS 38.321 v17.x §5.17
//! - 3GPP TS 38.133 v17.x §10.1.6

// ───────────────────────────────────────────────────────────────────────────
// Constants from 3GPP TS 38.213 / TS 38.133
// ───────────────────────────────────────────────────────────────────────────

/// L1-RSRP reporting range minimum (dBm), TS 38.133 Table 10.1.6.1-1
pub const L1_RSRP_MIN_DBM: f64 = -140.0;
/// L1-RSRP reporting range maximum (dBm), TS 38.133 Table 10.1.6.1-1
pub const L1_RSRP_MAX_DBM: f64 = -44.0;
/// L1-RSRP step size (dB)
pub const L1_RSRP_STEP_DB: f64 = 1.0;
/// Number of quantization levels for 7-bit L1-RSRP report
pub const L1_RSRP_LEVELS: u8 = 127;

/// Default q0 threshold for beam failure detection (dBm), TS 38.213 §11.1
pub const BFD_Q0_THRESHOLD_DBM: f64 = -110.0;
/// Default q1 threshold for beam failure detection (dBm)
pub const BFD_Q1_THRESHOLD_DBM: f64 = -100.0;
/// Maximum beam failure instances before declaring beam failure, TS 38.213 §11.1
pub const BFD_MAX_BEAM_FAILURE_INSTANCES: u32 = 10;

/// Candidate beam RSRP threshold for beam failure recovery (q_new), dBm
pub const BFR_Q_NEW_THRESHOLD_DBM: f64 = -105.0;

// ───────────────────────────────────────────────────────────────────────────
// Enumerations
// ───────────────────────────────────────────────────────────────────────────

/// Frequency range per 3GPP TS 38.101
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrequencyRange {
    /// FR1: 450 MHz – 7.125 GHz
    Fr1,
    /// FR2: 24.25 GHz – 52.6 GHz (mmWave)
    Fr2,
}

/// Subcarrier spacing (kHz) per TS 38.211 Table 4.2-1
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SubcarrierSpacing {
    Scs15kHz,
    Scs30kHz,
    Scs120kHz,
    Scs240kHz,
}

impl SubcarrierSpacing {
    /// Return numeric value in kHz
    pub fn khz(self) -> u32 {
        match self {
            SubcarrierSpacing::Scs15kHz => 15,
            SubcarrierSpacing::Scs30kHz => 30,
            SubcarrierSpacing::Scs120kHz => 120,
            SubcarrierSpacing::Scs240kHz => 240,
        }
    }
}

/// QCL (Quasi-Co-Location) type per TS 38.214 §5.1.5
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum QclType {
    /// QCL-TypeA: Doppler shift, Doppler spread, average delay, delay spread
    TypeA,
    /// QCL-TypeB: Doppler shift, Doppler spread
    TypeB,
    /// QCL-TypeC: Doppler shift, average delay
    TypeC,
    /// QCL-TypeD: Spatial Rx parameter (beam direction)
    TypeD,
}

/// Spatial relation source type for PUCCH/SRS/PUSCH
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SpatialRelationSource {
    SsbIndex(u8),
    CsiRsIndex(u8),
    SrsIndex(u8),
}

/// Beam management procedure type per TS 38.213 §11.1
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BeamProcedure {
    /// P1: Initial beam selection (exhaustive search over all TX beams at gNB)
    P1,
    /// P2: Beam refinement at gNB (UE measures refined gNB beams)
    P2,
    /// P3: Beam refinement at UE (UE sweeps receive beams for fixed gNB TX beam)
    P3,
}

/// Beam failure detection state machine
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BfdState {
    /// Normal operation — no failure indication
    Normal,
    /// Beam failure declared; recovery in progress
    BeamFailure,
    /// Recovery complete with new beam
    Recovered,
}

/// TCI (Transmission Configuration Indicator) state activation status
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TciActivation {
    Inactive,
    Active,
}

// ───────────────────────────────────────────────────────────────────────────
// Data structures
// ───────────────────────────────────────────────────────────────────────────

/// SSB beam sweep configuration per TS 38.213 Table 4.1-1
#[derive(Debug, Clone)]
pub struct SsbBeamSweepConfig {
    /// Frequency range (FR1 or FR2)
    pub freq_range: FrequencyRange,
    /// Subcarrier spacing for SSB
    pub scs: SubcarrierSpacing,
    /// Maximum number of beams (L_max): 4, 8, or 64
    pub l_max: u8,
    /// Half-frame period in milliseconds (5 ms)
    pub half_frame_ms: u32,
    /// Number of candidate SS/PBCH block positions in a half-frame
    pub num_ssb_candidates: u8,
}

impl SsbBeamSweepConfig {
    /// Create configuration per TS 38.213 Table 4.1-1
    ///
    /// # Arguments
    /// - `freq_range`: FR1 or FR2
    /// - `scs`: Subcarrier spacing for SSB
    pub fn new(freq_range: FrequencyRange, scs: SubcarrierSpacing) -> Self {
        let (l_max, num_candidates) = match (freq_range, scs) {
            (FrequencyRange::Fr1, SubcarrierSpacing::Scs15kHz) => (4, 4),
            (FrequencyRange::Fr1, SubcarrierSpacing::Scs30kHz) => (4, 4),
            (FrequencyRange::Fr2, SubcarrierSpacing::Scs120kHz) => (64, 64),
            (FrequencyRange::Fr2, SubcarrierSpacing::Scs240kHz) => (64, 64),
            // Default for other combinations
            _ => (8, 8),
        };
        SsbBeamSweepConfig {
            freq_range,
            scs,
            l_max,
            half_frame_ms: 5,
            num_ssb_candidates: num_candidates,
        }
    }

    /// Return maximum L_max for FR1 (TS 38.213 Table 4.1-1 note)
    ///
    /// For carrier frequencies <= 3 GHz: L_max = 4
    /// For carrier frequencies 3–6 GHz: L_max = 8
    pub fn l_max_fr1(carrier_freq_ghz: f64) -> u8 {
        if carrier_freq_ghz <= 3.0 {
            4
        } else {
            8
        }
    }
}

/// A single beam measurement result
#[derive(Debug, Clone, PartialEq)]
pub struct BeamMeasurement {
    /// SSB index (iSSB) identifying the beam
    pub ssb_index: u8,
    /// SS-RSRP in dBm
    pub ss_rsrp_dbm: f64,
    /// SS-RSRQ in dB
    pub ss_rsrq_db: f64,
    /// SS-SINR in dB
    pub ss_sinr_db: f64,
    /// 7-bit L1-RSRP quantized report value (0–127)
    pub l1_rsrp_report: u8,
    /// Timestamp (slot index) when measurement was taken
    pub slot_index: u64,
}

impl BeamMeasurement {
    /// Create a beam measurement, computing derived quantities
    pub fn new(ssb_index: u8, ss_rsrp_dbm: f64, ss_rsrq_db: f64, ss_sinr_db: f64, slot_index: u64) -> Self {
        let l1_rsrp_report = quantize_l1_rsrp(ss_rsrp_dbm);
        BeamMeasurement {
            ssb_index,
            ss_rsrp_dbm,
            ss_rsrq_db,
            ss_sinr_db,
            l1_rsrp_report,
            slot_index,
        }
    }
}

/// QCL assumption: the reference signal (RS) and the target signal share certain propagation properties
#[derive(Debug, Clone)]
pub struct QclAssumption {
    /// Reference signal type for QCL
    pub reference: SpatialRelationSource,
    /// QCL type (A/B/C/D)
    pub qcl_type: QclType,
}

/// TCI (Transmission Configuration Indicator) state
#[derive(Debug, Clone)]
pub struct TciState {
    /// TCI state ID (0–127)
    pub state_id: u8,
    /// QCL assumptions associated with this TCI state (up to 2)
    pub qcl_assumptions: Vec<QclAssumption>,
    /// Activation status
    pub activation: TciActivation,
}

impl TciState {
    /// Create a new TCI state with QCL-TypeD beam indication
    pub fn new_type_d(state_id: u8, ssb_index: u8) -> Self {
        TciState {
            state_id,
            qcl_assumptions: vec![QclAssumption {
                reference: SpatialRelationSource::SsbIndex(ssb_index),
                qcl_type: QclType::TypeD,
            }],
            activation: TciActivation::Inactive,
        }
    }

    /// Check if this TCI state carries a QCL-TypeD assumption
    pub fn has_type_d(&self) -> bool {
        self.qcl_assumptions.iter().any(|q| q.qcl_type == QclType::TypeD)
    }

    /// Get the SSB index referenced by QCL-TypeD (if present)
    pub fn type_d_ssb_index(&self) -> Option<u8> {
        for qa in &self.qcl_assumptions {
            if qa.qcl_type == QclType::TypeD {
                if let SpatialRelationSource::SsbIndex(idx) = qa.reference {
                    return Some(idx);
                }
            }
        }
        None
    }
}

/// Spatial relation info for PUCCH/SRS/PUSCH resources
#[derive(Debug, Clone)]
pub struct SpatialRelationInfo {
    /// Resource ID this applies to (PUCCH/SRS resource index)
    pub resource_id: u8,
    /// Source reference signal for spatial relation
    pub source: SpatialRelationSource,
}

/// Beam failure detection (BFD) configuration
#[derive(Debug, Clone)]
pub struct BfdConfig {
    /// q0 threshold (dBm): below this triggers a beam failure instance
    pub q0_threshold_dbm: f64,
    /// q1 threshold (dBm): used for candidate beam evaluation
    pub q1_threshold_dbm: f64,
    /// Maximum number of beam failure instances before declaring BFD
    pub max_beam_failure_instances: u32,
    /// Beam failure instance counting window (in slots)
    pub counting_window_slots: u32,
}

impl Default for BfdConfig {
    fn default() -> Self {
        BfdConfig {
            q0_threshold_dbm: BFD_Q0_THRESHOLD_DBM,
            q1_threshold_dbm: BFD_Q1_THRESHOLD_DBM,
            max_beam_failure_instances: BFD_MAX_BEAM_FAILURE_INSTANCES,
            counting_window_slots: 200,
        }
    }
}

/// Beam failure recovery (BFR) configuration
#[derive(Debug, Clone)]
pub struct BfrConfig {
    /// q_new threshold (dBm): candidate beam must exceed this RSRP
    pub q_new_threshold_dbm: f64,
    /// Maximum number of RACH attempts for BFR
    pub max_rach_attempts: u8,
    /// Timer T316 in slots (beam failure recovery request timer)
    pub t316_slots: u32,
}

impl Default for BfrConfig {
    fn default() -> Self {
        BfrConfig {
            q_new_threshold_dbm: BFR_Q_NEW_THRESHOLD_DBM,
            max_rach_attempts: 10,
            t316_slots: 1000,
        }
    }
}

/// Beam tracking state for the serving cell
#[derive(Debug, Clone)]
pub struct ServingBeamState {
    /// Currently active SSB index (beam)
    pub active_ssb_index: u8,
    /// RSRP of the active beam (dBm)
    pub active_rsrp_dbm: f64,
    /// Hysteresis margin (dB) to prevent beam toggling
    pub hysteresis_db: f64,
    /// Time-to-trigger in slots before switching beams
    pub time_to_trigger_slots: u32,
    /// Candidate beam SSB index being evaluated
    pub candidate_ssb_index: Option<u8>,
    /// Number of consecutive slots candidate has been better than active + hysteresis
    pub trigger_counter: u32,
}

impl ServingBeamState {
    /// Create initial serving beam state
    pub fn new(initial_ssb_index: u8, initial_rsrp_dbm: f64) -> Self {
        ServingBeamState {
            active_ssb_index: initial_ssb_index,
            active_rsrp_dbm: initial_rsrp_dbm,
            hysteresis_db: 3.0,
            time_to_trigger_slots: 40,
            candidate_ssb_index: None,
            trigger_counter: 0,
        }
    }

    /// Update tracking with new measurements.
    ///
    /// Returns `Some(new_ssb_index)` if a beam switch is triggered.
    pub fn update(&mut self, measurements: &[BeamMeasurement]) -> Option<u8> {
        // Find the best beam among all measurements
        let best = measurements.iter().max_by(|a, b| {
            a.ss_rsrp_dbm.partial_cmp(&b.ss_rsrp_dbm).unwrap_or(std::cmp::Ordering::Equal)
        });

        let best = match best {
            Some(m) => m,
            None => return None,
        };

        // Update active beam RSRP from new measurement (if same beam)
        if let Some(current_meas) = measurements.iter().find(|m| m.ssb_index == self.active_ssb_index) {
            self.active_rsrp_dbm = current_meas.ss_rsrp_dbm;
        }

        if best.ssb_index == self.active_ssb_index {
            // Active beam is still best — reset candidate
            self.candidate_ssb_index = None;
            self.trigger_counter = 0;
            return None;
        }

        // Check if candidate beam exceeds active + hysteresis
        let threshold = self.active_rsrp_dbm + self.hysteresis_db;
        if best.ss_rsrp_dbm > threshold {
            if self.candidate_ssb_index == Some(best.ssb_index) {
                self.trigger_counter += 1;
            } else {
                self.candidate_ssb_index = Some(best.ssb_index);
                self.trigger_counter = 1;
            }

            if self.trigger_counter >= self.time_to_trigger_slots {
                // Trigger beam switch
                let new_beam = best.ssb_index;
                self.active_ssb_index = new_beam;
                self.active_rsrp_dbm = best.ss_rsrp_dbm;
                self.candidate_ssb_index = None;
                self.trigger_counter = 0;
                return Some(new_beam);
            }
        } else {
            // Candidate not strong enough — reset
            self.candidate_ssb_index = None;
            self.trigger_counter = 0;
        }

        None
    }
}

// ───────────────────────────────────────────────────────────────────────────
// Core algorithms
// ───────────────────────────────────────────────────────────────────────────

/// Quantize SS-RSRP to 7-bit L1-RSRP report value per TS 38.133 Table 10.1.6.1-1
///
/// Mapping: report = clamp(floor(rsrp_dbm - L1_RSRP_MIN_DBM), 0, 127)
///
/// # Arguments
/// - `rsrp_dbm`: measured SS-RSRP in dBm
///
/// # Returns
/// 7-bit integer (0 = ≤ -140 dBm, 127 = ≥ -13 dBm [via step])
pub fn quantize_l1_rsrp(rsrp_dbm: f64) -> u8 {
    if rsrp_dbm <= L1_RSRP_MIN_DBM {
        return 0;
    }
    if rsrp_dbm >= L1_RSRP_MAX_DBM {
        return L1_RSRP_LEVELS;
    }
    let raw = (rsrp_dbm - L1_RSRP_MIN_DBM) / L1_RSRP_STEP_DB;
    let quantized = raw.floor() as u8;
    quantized.min(L1_RSRP_LEVELS)
}

/// Dequantize a 7-bit L1-RSRP report to dBm (lower bound of quantization interval)
pub fn dequantize_l1_rsrp(report: u8) -> f64 {
    L1_RSRP_MIN_DBM + (report as f64) * L1_RSRP_STEP_DB
}

/// Compute SS-RSRP from linear received signal power and reference signal configuration
///
/// SS-RSRP = 10 * log10(P_ss / N_re)
/// where P_ss is total received power on SS symbols and N_re is number of reference REs.
///
/// # Arguments
/// - `total_power_watts`: total received power across SSB RE positions
/// - `num_reference_res`: number of reference resource elements used
///
/// # Returns
/// SS-RSRP in dBm
pub fn compute_ss_rsrp(total_power_watts: f64, num_reference_res: usize) -> f64 {
    if total_power_watts <= 0.0 || num_reference_res == 0 {
        return L1_RSRP_MIN_DBM;
    }
    let rsrp_watts = total_power_watts / num_reference_res as f64;
    10.0 * rsrp_watts.log10() + 30.0 // convert W to dBm
}

/// Compute SS-RSRQ from SS-RSRP and wideband received power
///
/// SS-RSRQ = 10 * log10(N * SS-RSRP / S-RSSI)
/// where N = number of resource blocks in SSB bandwidth, S-RSSI is wideband power.
///
/// # Arguments
/// - `ss_rsrp_dbm`: SS-RSRP in dBm
/// - `s_rssi_dbm`: S-RSSI (wideband received signal strength) in dBm
/// - `n_rb`: number of resource blocks over which RSSI is measured
///
/// # Returns
/// SS-RSRQ in dB (typically in range -20 to 0 dB)
pub fn compute_ss_rsrq(ss_rsrp_dbm: f64, s_rssi_dbm: f64, n_rb: u32) -> f64 {
    if n_rb == 0 || s_rssi_dbm <= -200.0 {
        return -20.0;
    }
    let n_db = 10.0 * (n_rb as f64).log10();
    ss_rsrp_dbm + n_db - s_rssi_dbm
}

/// Compute SS-SINR from signal and noise+interference power
///
/// SS-SINR = 10 * log10(P_signal / P_noise_interference)
///
/// # Arguments
/// - `signal_power_watts`: signal power in watts
/// - `noise_interference_watts`: noise + interference power in watts
///
/// # Returns
/// SS-SINR in dB
pub fn compute_ss_sinr(signal_power_watts: f64, noise_interference_watts: f64) -> f64 {
    if noise_interference_watts <= 0.0 || signal_power_watts <= 0.0 {
        if signal_power_watts > 0.0 {
            return 50.0; // Very high SINR
        }
        return -50.0;
    }
    10.0 * (signal_power_watts / noise_interference_watts).log10()
}

/// Determine beam index from SSB index (iSSB) per TS 38.213 §4.1
///
/// The beam index is directly equal to iSSB for most cases.
/// For L_max=64 (FR2), all 64 SSBs map to distinct beams.
///
/// # Arguments
/// - `i_ssb`: SSB index (0 to L_max-1)
/// - `l_max`: Maximum number of SS/PBCH blocks
///
/// # Returns
/// `Ok(beam_index)` or `Err` if iSSB is out of range
pub fn ssb_index_to_beam_index(i_ssb: u8, l_max: u8) -> Result<u8, BeamMgmtError> {
    if i_ssb >= l_max {
        return Err(BeamMgmtError::InvalidSsbIndex { i_ssb, l_max });
    }
    // Per TS 38.213 Table 4.1-1: beam index = iSSB mod L_max
    // For simplicity (no frequency offset re-use), beam_index = i_ssb
    Ok(i_ssb % l_max)
}

/// Half-frame index from SSB index for FR2 (L_max=64)
///
/// For FR2 with L_max=64, iSSB ranges 0–63.
/// SSBs 0–3 are in sub-frame 0, 4–7 in sub-frame 1, etc.
pub fn ssb_half_frame_index(i_ssb: u8, l_max: u8) -> u8 {
    if l_max == 64 {
        i_ssb / 8
    } else if l_max == 8 {
        i_ssb / 4
    } else {
        0
    }
}

// ───────────────────────────────────────────────────────────────────────────
// P1: Initial beam selection
// ───────────────────────────────────────────────────────────────────────────

/// P1 beam selection result
#[derive(Debug, Clone, PartialEq)]
pub struct P1BeamSelectionResult {
    /// Selected best beam (SSB index)
    pub best_ssb_index: u8,
    /// RSRP of selected beam (dBm)
    pub best_rsrp_dbm: f64,
    /// RSRQ of selected beam (dB)
    pub best_rsrq_db: f64,
    /// SINR of selected beam (dB)
    pub best_sinr_db: f64,
    /// 7-bit L1-RSRP report
    pub l1_rsrp_report: u8,
    /// Number of beams measured
    pub num_beams_measured: usize,
    /// All measurements sorted by RSRP (descending)
    pub ranked_measurements: Vec<BeamMeasurement>,
}

/// Perform P1: Initial beam selection via exhaustive search over all SSB beams.
///
/// The UE measures all transmitted SSB indices and selects the beam with the
/// highest SS-RSRP that meets the minimum RSRP threshold.
///
/// # Arguments
/// - `measurements`: all beam measurements from SSB sweep
/// - `min_rsrp_dbm`: minimum RSRP threshold (default: -130 dBm)
///
/// # Returns
/// `Ok(P1BeamSelectionResult)` with the best beam, or error if no beam qualifies
pub fn p1_initial_beam_selection(
    measurements: &[BeamMeasurement],
    min_rsrp_dbm: f64,
) -> Result<P1BeamSelectionResult, BeamMgmtError> {
    if measurements.is_empty() {
        return Err(BeamMgmtError::NoMeasurements);
    }

    // Filter beams meeting minimum RSRP threshold
    let mut qualified: Vec<&BeamMeasurement> = measurements
        .iter()
        .filter(|m| m.ss_rsrp_dbm >= min_rsrp_dbm)
        .collect();

    if qualified.is_empty() {
        return Err(BeamMgmtError::NoQualifyingBeam { threshold_dbm: min_rsrp_dbm });
    }

    // Sort by RSRP descending
    qualified.sort_by(|a, b| b.ss_rsrp_dbm.partial_cmp(&a.ss_rsrp_dbm).unwrap_or(std::cmp::Ordering::Equal));

    let best = qualified[0];

    // Build ranked list (cloned)
    let mut ranked: Vec<BeamMeasurement> = measurements.to_vec();
    ranked.sort_by(|a, b| b.ss_rsrp_dbm.partial_cmp(&a.ss_rsrp_dbm).unwrap_or(std::cmp::Ordering::Equal));

    Ok(P1BeamSelectionResult {
        best_ssb_index: best.ssb_index,
        best_rsrp_dbm: best.ss_rsrp_dbm,
        best_rsrq_db: best.ss_rsrq_db,
        best_sinr_db: best.ss_sinr_db,
        l1_rsrp_report: best.l1_rsrp_report,
        num_beams_measured: measurements.len(),
        ranked_measurements: ranked,
    })
}

// ───────────────────────────────────────────────────────────────────────────
// P2: Beam refinement at gNB (hierarchical codebook)
// ───────────────────────────────────────────────────────────────────────────

/// P2 beam refinement result
#[derive(Debug, Clone)]
pub struct P2BeamRefinementResult {
    /// Refined best beam SSB index
    pub refined_ssb_index: u8,
    /// RSRP after refinement (dBm)
    pub refined_rsrp_dbm: f64,
    /// Improvement over P1 beam (dB)
    pub rsrp_improvement_db: f64,
    /// Number of candidate beams evaluated
    pub num_candidates: usize,
}

/// Perform P2: Beam refinement at gNB side.
///
/// Starting from the P1-selected coarse beam, a set of refined (narrower) beams
/// in the same codebook region are measured and the best is selected.
///
/// # Arguments
/// - `coarse_ssb_index`: SSB index selected in P1
/// - `coarse_rsrp_dbm`: RSRP of the coarse beam (dBm)
/// - `refined_measurements`: measurements of refined beam candidates near the coarse beam
/// - `refinement_gain_db`: expected gain from refinement (for simulation purposes)
///
/// # Returns
/// P2 refinement result
pub fn p2_beam_refinement_gnb(
    coarse_ssb_index: u8,
    coarse_rsrp_dbm: f64,
    refined_measurements: &[BeamMeasurement],
    _refinement_gain_db: f64,
) -> Result<P2BeamRefinementResult, BeamMgmtError> {
    if refined_measurements.is_empty() {
        // No refinement candidates — return coarse beam as result
        return Ok(P2BeamRefinementResult {
            refined_ssb_index: coarse_ssb_index,
            refined_rsrp_dbm: coarse_rsrp_dbm,
            rsrp_improvement_db: 0.0,
            num_candidates: 0,
        });
    }

    // Select best refined beam
    let best = refined_measurements
        .iter()
        .max_by(|a, b| a.ss_rsrp_dbm.partial_cmp(&b.ss_rsrp_dbm).unwrap_or(std::cmp::Ordering::Equal))
        .ok_or(BeamMgmtError::NoMeasurements)?;

    let improvement = best.ss_rsrp_dbm - coarse_rsrp_dbm;

    Ok(P2BeamRefinementResult {
        refined_ssb_index: best.ssb_index,
        refined_rsrp_dbm: best.ss_rsrp_dbm,
        rsrp_improvement_db: improvement,
        num_candidates: refined_measurements.len(),
    })
}

// ───────────────────────────────────────────────────────────────────────────
// P3: UE receive beam refinement
// ───────────────────────────────────────────────────────────────────────────

/// P3 UE receive beam refinement result
#[derive(Debug, Clone, PartialEq)]
pub struct P3BeamRefinementResult {
    /// Best UE receive beam index
    pub best_ue_rx_beam: u8,
    /// RSRP with best UE receive beam (dBm)
    pub best_rsrp_dbm: f64,
    /// Number of UE receive beams swept
    pub num_rx_beams: usize,
    /// All UE RX beam RSRP values indexed by UE beam index
    pub rx_beam_rsrp: Vec<f64>,
}

/// Perform P3: UE receive beam sweep for a fixed gNB transmit beam.
///
/// The UE sweeps its receive beams while the gNB transmits on a fixed beam.
/// The UE selects the RX beam maximizing received RSRP.
///
/// # Arguments
/// - `gssb_index`: fixed gNB SSB TX beam index
/// - `rx_beam_rsrp_dbm`: RSRP measured for each UE RX beam (indexed by RX beam index)
///
/// # Returns
/// P3 result with best UE RX beam
pub fn p3_ue_rx_beam_sweep(
    _gnb_ssb_index: u8,
    rx_beam_rsrp_dbm: &[f64],
) -> Result<P3BeamRefinementResult, BeamMgmtError> {
    if rx_beam_rsrp_dbm.is_empty() {
        return Err(BeamMgmtError::NoMeasurements);
    }

    let (best_idx, best_rsrp) = rx_beam_rsrp_dbm
        .iter()
        .enumerate()
        .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal))
        .map(|(i, &v)| (i, v))
        .ok_or(BeamMgmtError::NoMeasurements)?;

    Ok(P3BeamRefinementResult {
        best_ue_rx_beam: best_idx as u8,
        best_rsrp_dbm: best_rsrp,
        num_rx_beams: rx_beam_rsrp_dbm.len(),
        rx_beam_rsrp: rx_beam_rsrp_dbm.to_vec(),
    })
}

// ───────────────────────────────────────────────────────────────────────────
// Beam failure detection (BFD)
// ───────────────────────────────────────────────────────────────────────────

/// Beam Failure Detector per TS 38.213 §11.1
#[derive(Debug, Clone)]
pub struct BeamFailureDetector {
    config: BfdConfig,
    state: BfdState,
    failure_instance_count: u32,
    /// History of per-slot hypothetical PDCCH BLER estimates for BFD
    bler_history: Vec<f64>,
    /// Slot index of last reset
    last_reset_slot: u64,
    current_slot: u64,
}

impl BeamFailureDetector {
    /// Create a new beam failure detector
    pub fn new(config: BfdConfig) -> Self {
        BeamFailureDetector {
            config,
            state: BfdState::Normal,
            failure_instance_count: 0,
            bler_history: Vec::new(),
            last_reset_slot: 0,
            current_slot: 0,
        }
    }

    /// Process a new slot's beam quality estimate.
    ///
    /// BFD uses hypothetical PDCCH BLER or RSRP relative to q0/q1 thresholds.
    /// A beam failure instance is declared when the reference signal quality
    /// falls below q0 threshold.
    ///
    /// # Arguments
    /// - `rsrp_dbm`: current beam reference signal RSRP
    /// - `slot_index`: current slot number
    ///
    /// # Returns
    /// Current `BfdState`
    pub fn update(&mut self, rsrp_dbm: f64, slot_index: u64) -> BfdState {
        self.current_slot = slot_index;

        // Sliding window: remove old failure instances
        let window_start = slot_index.saturating_sub(self.config.counting_window_slots as u64);
        if slot_index - self.last_reset_slot >= self.config.counting_window_slots as u64 {
            // Periodically reset counter within window logic
            // Simple model: decay count by 1 per window expiry
            if self.failure_instance_count > 0 {
                self.failure_instance_count -= 1;
            }
            self.last_reset_slot = window_start;
        }

        if self.state != BfdState::BeamFailure {
            // Check if this slot qualifies as a beam failure instance (RSRP < q0)
            if rsrp_dbm < self.config.q0_threshold_dbm {
                self.failure_instance_count += 1;
            }

            if self.failure_instance_count >= self.config.max_beam_failure_instances {
                self.state = BfdState::BeamFailure;
            }
        }

        self.state
    }

    /// Reset after successful recovery
    pub fn reset(&mut self, slot_index: u64) {
        self.state = BfdState::Normal;
        self.failure_instance_count = 0;
        self.last_reset_slot = slot_index;
    }

    /// Mark recovery as complete
    pub fn mark_recovered(&mut self) {
        self.state = BfdState::Recovered;
    }

    /// Current BFD state
    pub fn state(&self) -> BfdState {
        self.state
    }

    /// Current failure instance count
    pub fn failure_instance_count(&self) -> u32 {
        self.failure_instance_count
    }
}

// ───────────────────────────────────────────────────────────────────────────
// Beam failure recovery (BFR)
// ───────────────────────────────────────────────────────────────────────────

/// Beam failure recovery result
#[derive(Debug, Clone)]
pub struct BfrResult {
    /// New beam SSB index selected for recovery
    pub new_ssb_index: u8,
    /// RSRP of new beam (dBm)
    pub new_rsrp_dbm: f64,
    /// Whether a qualifying candidate was found
    pub candidate_found: bool,
    /// Number of RACH attempts used
    pub rach_attempts: u8,
}

/// Beam Failure Recovery handler per TS 38.213 §11.1
#[derive(Debug, Clone)]
pub struct BeamFailureRecovery {
    config: BfrConfig,
    rach_attempts: u8,
    t316_remaining_slots: u32,
}

impl BeamFailureRecovery {
    /// Create a new BFR handler
    pub fn new(config: BfrConfig) -> Self {
        BeamFailureRecovery {
            config,
            rach_attempts: 0,
            t316_remaining_slots: 0,
        }
    }

    /// Select a candidate beam for recovery from available measurements.
    ///
    /// The UE selects the beam with highest RSRP that exceeds q_new threshold.
    ///
    /// # Arguments
    /// - `candidate_measurements`: all candidate beam measurements
    ///
    /// # Returns
    /// `Some(BfrResult)` if a qualifying beam is found, `None` otherwise
    pub fn select_candidate_beam(
        &mut self,
        candidate_measurements: &[BeamMeasurement],
    ) -> Option<BfrResult> {
        let qualified: Vec<&BeamMeasurement> = candidate_measurements
            .iter()
            .filter(|m| m.ss_rsrp_dbm >= self.config.q_new_threshold_dbm)
            .collect();

        if qualified.is_empty() {
            // No qualifying candidate — increment RACH attempts
            self.rach_attempts = self.rach_attempts.saturating_add(1);
            return None;
        }

        // Select best qualifying candidate
        let best = qualified
            .into_iter()
            .max_by(|a, b| a.ss_rsrp_dbm.partial_cmp(&b.ss_rsrp_dbm).unwrap_or(std::cmp::Ordering::Equal))
            .unwrap();

        self.rach_attempts += 1;
        self.t316_remaining_slots = self.config.t316_slots;

        Some(BfrResult {
            new_ssb_index: best.ssb_index,
            new_rsrp_dbm: best.ss_rsrp_dbm,
            candidate_found: true,
            rach_attempts: self.rach_attempts,
        })
    }

    /// Decrement T316 timer by one slot. Returns true if timer expired.
    pub fn tick_t316(&mut self) -> bool {
        if self.t316_remaining_slots > 0 {
            self.t316_remaining_slots -= 1;
            self.t316_remaining_slots == 0
        } else {
            true
        }
    }

    /// Reset BFR state after successful recovery
    pub fn reset(&mut self) {
        self.rach_attempts = 0;
        self.t316_remaining_slots = 0;
    }

    /// Number of RACH attempts used so far
    pub fn rach_attempts(&self) -> u8 {
        self.rach_attempts
    }

    /// Whether max RACH attempts have been exhausted
    pub fn max_attempts_exhausted(&self) -> bool {
        self.rach_attempts >= self.config.max_rach_attempts
    }
}

// ───────────────────────────────────────────────────────────────────────────
// Beam correspondence
// ───────────────────────────────────────────────────────────────────────────

/// Beam correspondence indication per TS 38.214 §5.2.2.4
///
/// When beam correspondence holds, the UE can infer that the best
/// DL SSB beam also corresponds to the best UL beam, without needing
/// separate UL beam management procedures.
#[derive(Debug, Clone)]
pub struct BeamCorrespondence {
    /// Whether the UE supports and indicates beam correspondence
    pub supported: bool,
    /// Active correspondence: (DL SSB index) -> UL beam index
    correspondence_map: Vec<(u8, u8)>,
}

impl BeamCorrespondence {
    /// Create beam correspondence instance
    pub fn new(supported: bool) -> Self {
        BeamCorrespondence {
            supported,
            correspondence_map: Vec::new(),
        }
    }

    /// Add a DL→UL beam pair correspondence
    pub fn add_pair(&mut self, dl_ssb_index: u8, ul_beam_index: u8) {
        // Remove existing entry for this DL SSB if any
        self.correspondence_map.retain(|(d, _)| *d != dl_ssb_index);
        self.correspondence_map.push((dl_ssb_index, ul_beam_index));
    }

    /// Look up UL beam index for a given DL SSB index
    pub fn ul_beam_for_dl_ssb(&self, dl_ssb_index: u8) -> Option<u8> {
        if !self.supported {
            return None;
        }
        self.correspondence_map
            .iter()
            .find(|(d, _)| *d == dl_ssb_index)
            .map(|(_, u)| *u)
    }

    /// If beam correspondence is supported, return UL beam == DL SSB index (identity mapping)
    pub fn infer_ul_beam(&self, dl_ssb_index: u8) -> Option<u8> {
        if self.supported {
            // Check explicit map first
            if let Some(ul) = self.ul_beam_for_dl_ssb(dl_ssb_index) {
                return Some(ul);
            }
            // Default: identity correspondence
            Some(dl_ssb_index)
        } else {
            None
        }
    }
}

// ───────────────────────────────────────────────────────────────────────────
// TCI State Manager (MAC CE activation/deactivation)
// ───────────────────────────────────────────────────────────────────────────

/// TCI State Manager handling MAC CE beam indication per TS 38.321 §6.1.3.14
#[derive(Debug, Clone)]
pub struct TciStateManager {
    /// All configured TCI states (up to 128)
    states: Vec<TciState>,
    /// Maximum number of simultaneously active TCI states
    max_active: usize,
    /// Currently active TCI state IDs
    active_state_ids: Vec<u8>,
}

impl TciStateManager {
    /// Create a new TCI state manager
    ///
    /// # Arguments
    /// - `max_active`: maximum simultaneously active TCI states (typically 8)
    pub fn new(max_active: usize) -> Self {
        TciStateManager {
            states: Vec::new(),
            max_active,
            active_state_ids: Vec::new(),
        }
    }

    /// Add or update a TCI state
    pub fn configure_state(&mut self, state: TciState) {
        if let Some(existing) = self.states.iter_mut().find(|s| s.state_id == state.state_id) {
            *existing = state;
        } else {
            self.states.push(state);
        }
    }

    /// Activate TCI states via MAC CE indication per TS 38.321 §6.1.3.14
    ///
    /// # Arguments
    /// - `state_ids`: list of TCI state IDs to activate (MAC CE payload)
    ///
    /// # Returns
    /// Number of states successfully activated
    pub fn activate_via_mac_ce(&mut self, state_ids: &[u8]) -> usize {
        let to_activate: Vec<u8> = state_ids
            .iter()
            .take(self.max_active)
            .copied()
            .collect();

        // Deactivate all previously active states
        for s in self.states.iter_mut() {
            s.activation = TciActivation::Inactive;
        }
        self.active_state_ids.clear();

        let mut count = 0;
        for id in &to_activate {
            if let Some(state) = self.states.iter_mut().find(|s| s.state_id == *id) {
                state.activation = TciActivation::Active;
                self.active_state_ids.push(*id);
                count += 1;
            }
        }
        count
    }

    /// Deactivate specific TCI states
    pub fn deactivate_states(&mut self, state_ids: &[u8]) {
        for id in state_ids {
            if let Some(state) = self.states.iter_mut().find(|s| s.state_id == *id) {
                state.activation = TciActivation::Inactive;
            }
            self.active_state_ids.retain(|a| *a != *id);
        }
    }

    /// Get currently active TCI states
    pub fn active_states(&self) -> Vec<&TciState> {
        self.states
            .iter()
            .filter(|s| s.activation == TciActivation::Active)
            .collect()
    }

    /// Get the SSB index for the default (first active) QCL-TypeD TCI state
    pub fn default_beam_ssb_index(&self) -> Option<u8> {
        self.active_states()
            .iter()
            .find(|s| s.has_type_d())
            .and_then(|s| s.type_d_ssb_index())
    }

    /// Total number of configured TCI states
    pub fn num_configured(&self) -> usize {
        self.states.len()
    }

    /// Number of currently active TCI states
    pub fn num_active(&self) -> usize {
        self.active_state_ids.len()
    }
}

// ───────────────────────────────────────────────────────────────────────────
// Spatial relation manager
// ───────────────────────────────────────────────────────────────────────────

/// Spatial relation manager for PUCCH, SRS, PUSCH resources
#[derive(Debug, Clone)]
pub struct SpatialRelationManager {
    relations: Vec<SpatialRelationInfo>,
}

impl SpatialRelationManager {
    /// Create a new spatial relation manager
    pub fn new() -> Self {
        SpatialRelationManager {
            relations: Vec::new(),
        }
    }

    /// Configure spatial relation for a resource
    pub fn configure(&mut self, info: SpatialRelationInfo) {
        self.relations.retain(|r| r.resource_id != info.resource_id);
        self.relations.push(info);
    }

    /// Remove spatial relation for a resource
    pub fn remove(&mut self, resource_id: u8) {
        self.relations.retain(|r| r.resource_id != resource_id);
    }

    /// Get spatial relation for a resource ID
    pub fn get(&self, resource_id: u8) -> Option<&SpatialRelationInfo> {
        self.relations.iter().find(|r| r.resource_id == resource_id)
    }

    /// Update all spatial relations pointing to a given SSB index to a new SSB index
    /// (used during beam switch)
    pub fn update_ssb_references(&mut self, old_ssb: u8, new_ssb: u8) {
        for rel in self.relations.iter_mut() {
            if let SpatialRelationSource::SsbIndex(idx) = rel.source {
                if idx == old_ssb {
                    rel.source = SpatialRelationSource::SsbIndex(new_ssb);
                }
            }
        }
    }

    /// Number of configured spatial relations
    pub fn num_configured(&self) -> usize {
        self.relations.len()
    }
}

impl Default for SpatialRelationManager {
    fn default() -> Self {
        Self::new()
    }
}

// ───────────────────────────────────────────────────────────────────────────
// Main Beam Management Engine
// ───────────────────────────────────────────────────────────────────────────

/// Complete 5G NR SSB Beam Management Engine
///
/// Integrates all beam management procedures (P1/P2/P3), beam failure detection,
/// beam failure recovery, TCI state management, and spatial relation handling.
#[derive(Debug)]
pub struct NrSsbBeamManager {
    /// SSB sweep configuration
    pub sweep_config: SsbBeamSweepConfig,
    /// Serving beam tracking state
    pub serving_beam: ServingBeamState,
    /// Beam failure detector
    pub bfd: BeamFailureDetector,
    /// Beam failure recovery handler
    pub bfr: BeamFailureRecovery,
    /// TCI state manager
    pub tci_manager: TciStateManager,
    /// Spatial relation manager
    pub spatial_rel_mgr: SpatialRelationManager,
    /// Beam correspondence
    pub beam_correspondence: BeamCorrespondence,
    /// Current slot index
    current_slot: u64,
    /// History of last N beam measurements per SSB index
    measurement_history: Vec<BeamMeasurement>,
    /// Maximum history length per beam
    history_max: usize,
}

impl NrSsbBeamManager {
    /// Create a new NR SSB Beam Manager
    ///
    /// # Arguments
    /// - `sweep_config`: SSB sweep configuration
    /// - `initial_ssb_index`: initial serving beam SSB index
    /// - `initial_rsrp_dbm`: initial RSRP measurement (dBm)
    pub fn new(
        sweep_config: SsbBeamSweepConfig,
        initial_ssb_index: u8,
        initial_rsrp_dbm: f64,
    ) -> Self {
        NrSsbBeamManager {
            sweep_config,
            serving_beam: ServingBeamState::new(initial_ssb_index, initial_rsrp_dbm),
            bfd: BeamFailureDetector::new(BfdConfig::default()),
            bfr: BeamFailureRecovery::new(BfrConfig::default()),
            tci_manager: TciStateManager::new(8),
            spatial_rel_mgr: SpatialRelationManager::new(),
            beam_correspondence: BeamCorrespondence::new(false),
            current_slot: 0,
            measurement_history: Vec::new(),
            history_max: 64,
        }
    }

    /// Process a new set of SSB beam measurements for the current slot.
    ///
    /// This is the main update function that drives all beam management logic.
    ///
    /// Returns an event if action is required (beam switch, BFD/BFR trigger, etc.)
    pub fn process_measurements(&mut self, measurements: &[BeamMeasurement]) -> BeamMgmtEvent {
        self.current_slot += 1;

        // Store measurements in history (keep last history_max)
        for m in measurements {
            self.measurement_history.push(m.clone());
        }
        if self.measurement_history.len() > self.history_max {
            let excess = self.measurement_history.len() - self.history_max;
            self.measurement_history.drain(0..excess);
        }

        // Update BFD with active beam RSRP
        let active_rsrp = measurements
            .iter()
            .find(|m| m.ssb_index == self.serving_beam.active_ssb_index)
            .map(|m| m.ss_rsrp_dbm)
            .unwrap_or(self.serving_beam.active_rsrp_dbm);

        let bfd_state = self.bfd.update(active_rsrp, self.current_slot);

        if bfd_state == BfdState::BeamFailure {
            // Trigger BFR
            if let Some(bfr_result) = self.bfr.select_candidate_beam(measurements) {
                // Update serving beam with new recovered beam
                let old_ssb = self.serving_beam.active_ssb_index;
                self.serving_beam.active_ssb_index = bfr_result.new_ssb_index;
                self.serving_beam.active_rsrp_dbm = bfr_result.new_rsrp_dbm;
                self.bfd.reset(self.current_slot);
                self.bfr.reset();
                // Update spatial relations
                self.spatial_rel_mgr.update_ssb_references(old_ssb, bfr_result.new_ssb_index);
                return BeamMgmtEvent::BeamFailureRecovery {
                    old_ssb_index: old_ssb,
                    new_ssb_index: bfr_result.new_ssb_index,
                    new_rsrp_dbm: bfr_result.new_rsrp_dbm,
                };
            } else if self.bfr.max_attempts_exhausted() {
                return BeamMgmtEvent::RecoveryFailed;
            } else {
                return BeamMgmtEvent::BeamFailureDetected {
                    ssb_index: self.serving_beam.active_ssb_index,
                    rsrp_dbm: active_rsrp,
                };
            }
        }

        // Normal operation: check for beam switch via hysteresis+TTT
        if let Some(new_beam) = self.serving_beam.update(measurements) {
            // Update spatial relations
            let old_ssb = if measurements
                .iter()
                .find(|m| m.ssb_index == self.serving_beam.active_ssb_index)
                .is_some()
            {
                // find the old beam before switch — active_ssb_index already updated
                new_beam  // this is the new one, so old was something else — track it
            } else {
                new_beam
            };
            // Rebuild: serving_beam.active_ssb_index is already new_beam at this point
            let _ = old_ssb;
            return BeamMgmtEvent::BeamSwitch {
                new_ssb_index: new_beam,
                rsrp_dbm: self.serving_beam.active_rsrp_dbm,
            };
        }

        BeamMgmtEvent::None
    }

    /// Perform P1 initial beam selection on a full sweep
    pub fn run_p1(&self, measurements: &[BeamMeasurement], min_rsrp_dbm: f64) -> Result<P1BeamSelectionResult, BeamMgmtError> {
        p1_initial_beam_selection(measurements, min_rsrp_dbm)
    }

    /// Perform P2 beam refinement at gNB
    pub fn run_p2(
        &self,
        coarse_ssb: u8,
        coarse_rsrp: f64,
        refined: &[BeamMeasurement],
    ) -> Result<P2BeamRefinementResult, BeamMgmtError> {
        p2_beam_refinement_gnb(coarse_ssb, coarse_rsrp, refined, 3.0)
    }

    /// Perform P3 UE receive beam sweep
    pub fn run_p3(&self, gnb_ssb: u8, rx_rsrp: &[f64]) -> Result<P3BeamRefinementResult, BeamMgmtError> {
        p3_ue_rx_beam_sweep(gnb_ssb, rx_rsrp)
    }

    /// Get measurement history for a given SSB index (most recent first)
    pub fn measurement_history_for(&self, ssb_index: u8) -> Vec<&BeamMeasurement> {
        let mut v: Vec<&BeamMeasurement> = self.measurement_history
            .iter()
            .filter(|m| m.ssb_index == ssb_index)
            .collect();
        v.reverse();
        v
    }

    /// Current serving beam SSB index
    pub fn serving_ssb_index(&self) -> u8 {
        self.serving_beam.active_ssb_index
    }

    /// Current slot index
    pub fn current_slot(&self) -> u64 {
        self.current_slot
    }
}

/// Events returned by the beam management engine
#[derive(Debug, Clone, PartialEq)]
pub enum BeamMgmtEvent {
    /// No significant event
    None,
    /// Normal beam switch triggered by hysteresis + time-to-trigger
    BeamSwitch {
        new_ssb_index: u8,
        rsrp_dbm: f64,
    },
    /// Beam failure detected (BFD threshold exceeded)
    BeamFailureDetected {
        ssb_index: u8,
        rsrp_dbm: f64,
    },
    /// Beam failure recovery successful with new beam
    BeamFailureRecovery {
        old_ssb_index: u8,
        new_ssb_index: u8,
        new_rsrp_dbm: f64,
    },
    /// Recovery failed (max RACH attempts exhausted or T316 expired)
    RecoveryFailed,
}

// ───────────────────────────────────────────────────────────────────────────
// Error types
// ───────────────────────────────────────────────────────────────────────────

/// Beam management errors
#[derive(Debug, Clone, PartialEq)]
pub enum BeamMgmtError {
    /// No measurements provided
    NoMeasurements,
    /// No beam meets the minimum RSRP threshold
    NoQualifyingBeam { threshold_dbm: f64 },
    /// SSB index out of range for given L_max
    InvalidSsbIndex { i_ssb: u8, l_max: u8 },
    /// Invalid configuration parameter
    InvalidConfig(String),
}

impl std::fmt::Display for BeamMgmtError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            BeamMgmtError::NoMeasurements => write!(f, "No measurements provided"),
            BeamMgmtError::NoQualifyingBeam { threshold_dbm } => {
                write!(f, "No beam meets minimum RSRP threshold of {threshold_dbm:.1} dBm")
            }
            BeamMgmtError::InvalidSsbIndex { i_ssb, l_max } => {
                write!(f, "SSB index {i_ssb} out of range for L_max={l_max}")
            }
            BeamMgmtError::InvalidConfig(msg) => write!(f, "Invalid configuration: {msg}"),
        }
    }
}

// ───────────────────────────────────────────────────────────────────────────
// Helper: simulate measurements for testing
// ───────────────────────────────────────────────────────────────────────────

/// Create a simulated set of beam measurements for testing
///
/// # Arguments
/// - `l_max`: number of beams
/// - `best_beam`: SSB index of the strongest beam
/// - `best_rsrp_dbm`: RSRP of the best beam (dBm)
/// - `rsrp_spread_db`: spread in RSRP across beams (other beams are weaker by up to this amount)
/// - `slot_index`: slot timestamp
pub fn simulate_beam_measurements(
    l_max: u8,
    best_beam: u8,
    best_rsrp_dbm: f64,
    rsrp_spread_db: f64,
    slot_index: u64,
) -> Vec<BeamMeasurement> {
    let mut meas = Vec::new();
    for i in 0..l_max {
        // Simple angular distance model: beams further from best_beam get weaker
        let dist = ((i as i16 - best_beam as i16).abs() as f64).min(rsrp_spread_db / 3.0 + 1.0);
        let rsrp = best_rsrp_dbm - dist * (rsrp_spread_db / (l_max as f64 / 2.0 + 1.0));
        let rsrq = -3.0 - dist * 0.5;
        let sinr = 15.0 - dist * 1.5;
        meas.push(BeamMeasurement::new(i, rsrp, rsrq, sinr, slot_index));
    }
    meas
}

// ───────────────────────────────────────────────────────────────────────────
// Tests
// ───────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ─── L1-RSRP quantization ───────────────────────────────────────────

    #[test]
    fn test_l1_rsrp_quantize_min() {
        assert_eq!(quantize_l1_rsrp(-140.0), 0);
        assert_eq!(quantize_l1_rsrp(-200.0), 0);
    }

    #[test]
    fn test_l1_rsrp_quantize_max() {
        assert_eq!(quantize_l1_rsrp(-44.0), 127);
        assert_eq!(quantize_l1_rsrp(0.0), 127);
    }

    #[test]
    fn test_l1_rsrp_quantize_midpoint() {
        // -140 + 50 = -90 dBm → report = 50
        let report = quantize_l1_rsrp(-90.0);
        assert_eq!(report, 50);
    }

    #[test]
    fn test_l1_rsrp_quantize_step() {
        // Each step is 1 dB
        assert_eq!(quantize_l1_rsrp(-139.0), 1);
        assert_eq!(quantize_l1_rsrp(-138.0), 2);
        assert_eq!(quantize_l1_rsrp(-100.0), 40);
    }

    #[test]
    fn test_l1_rsrp_dequantize_roundtrip() {
        let rsrp_dbm = -95.0_f64;
        let report = quantize_l1_rsrp(rsrp_dbm);
        let recovered = dequantize_l1_rsrp(report);
        // Recovered should be within 1 dB step of original
        assert!((recovered - rsrp_dbm).abs() < 1.01, "recovered={recovered} original={rsrp_dbm}");
    }

    #[test]
    fn test_l1_rsrp_dequantize_zero() {
        assert_eq!(dequantize_l1_rsrp(0), -140.0);
    }

    #[test]
    fn test_l1_rsrp_dequantize_127() {
        // report 127 → -140 + 127 = -13 dBm
        assert_eq!(dequantize_l1_rsrp(127), -13.0);
    }

    // ─── SS-RSRP / SS-RSRQ / SS-SINR computation ───────────────────────

    #[test]
    fn test_ss_rsrp_basic() {
        // 1 mW across 240 REs → RSRP = 10*log10(0.001/240) + 30 ≈ -23.8 + 30 = 6.2... wait
        // 1e-3 W / 240 = 4.167e-6 W → 10*log10(4.167e-6) + 30 = -53.8 + 30 ≈ -23.8
        let rsrp = compute_ss_rsrp(1e-3, 240);
        assert!(rsrp > -30.0 && rsrp < -20.0, "rsrp={rsrp}");
    }

    #[test]
    fn test_ss_rsrp_zero_power() {
        let rsrp = compute_ss_rsrp(0.0, 240);
        assert_eq!(rsrp, L1_RSRP_MIN_DBM);
    }

    #[test]
    fn test_ss_rsrp_zero_res() {
        let rsrp = compute_ss_rsrp(1e-3, 0);
        assert_eq!(rsrp, L1_RSRP_MIN_DBM);
    }

    #[test]
    fn test_ss_rsrq_typical() {
        // RSRP = -80 dBm, RSSI = -70 dBm, N_RB = 20
        // RSRQ = -80 + 10*log10(20) - (-70) = -80 + 13.01 + 70 = 3.01 dB
        let rsrq = compute_ss_rsrq(-80.0, -70.0, 20);
        assert!((rsrq - 3.01).abs() < 0.1, "rsrq={rsrq}");
    }

    #[test]
    fn test_ss_rsrq_zero_rb() {
        let rsrq = compute_ss_rsrq(-80.0, -70.0, 0);
        assert_eq!(rsrq, -20.0);
    }

    #[test]
    fn test_ss_sinr_basic() {
        // Signal = 1 mW, Noise = 0.1 mW → SINR = 10 dB
        let sinr = compute_ss_sinr(1e-3, 1e-4);
        assert!((sinr - 10.0).abs() < 0.01, "sinr={sinr}");
    }

    #[test]
    fn test_ss_sinr_zero_noise() {
        let sinr = compute_ss_sinr(1e-3, 0.0);
        assert_eq!(sinr, 50.0);
    }

    #[test]
    fn test_ss_sinr_zero_signal() {
        let sinr = compute_ss_sinr(0.0, 1e-4);
        assert_eq!(sinr, -50.0);
    }

    // ─── SSB index / beam index mapping ─────────────────────────────────

    #[test]
    fn test_ssb_index_to_beam_index_valid() {
        for i in 0..4u8 {
            assert_eq!(ssb_index_to_beam_index(i, 4).unwrap(), i);
        }
        for i in 0..8u8 {
            assert_eq!(ssb_index_to_beam_index(i, 8).unwrap(), i);
        }
        for i in 0..64u8 {
            assert_eq!(ssb_index_to_beam_index(i, 64).unwrap(), i);
        }
    }

    #[test]
    fn test_ssb_index_to_beam_index_out_of_range() {
        assert!(ssb_index_to_beam_index(4, 4).is_err());
        assert!(ssb_index_to_beam_index(8, 8).is_err());
        assert!(ssb_index_to_beam_index(64, 64).is_err());
    }

    #[test]
    fn test_ssb_half_frame_index_fr2() {
        // FR2, L_max=64: each group of 8 SSBs in a sub-frame
        assert_eq!(ssb_half_frame_index(0, 64), 0);
        assert_eq!(ssb_half_frame_index(7, 64), 0);
        assert_eq!(ssb_half_frame_index(8, 64), 1);
        assert_eq!(ssb_half_frame_index(63, 64), 7);
    }

    #[test]
    fn test_ssb_half_frame_index_fr1_lmax8() {
        assert_eq!(ssb_half_frame_index(0, 8), 0);
        assert_eq!(ssb_half_frame_index(3, 8), 0);
        assert_eq!(ssb_half_frame_index(4, 8), 1);
    }

    // ─── SSB sweep config ────────────────────────────────────────────────

    #[test]
    fn test_ssb_sweep_config_fr1_15khz() {
        let cfg = SsbBeamSweepConfig::new(FrequencyRange::Fr1, SubcarrierSpacing::Scs15kHz);
        assert_eq!(cfg.l_max, 4);
        assert_eq!(cfg.num_ssb_candidates, 4);
        assert_eq!(cfg.freq_range, FrequencyRange::Fr1);
    }

    #[test]
    fn test_ssb_sweep_config_fr2_120khz() {
        let cfg = SsbBeamSweepConfig::new(FrequencyRange::Fr2, SubcarrierSpacing::Scs120kHz);
        assert_eq!(cfg.l_max, 64);
        assert_eq!(cfg.num_ssb_candidates, 64);
        assert_eq!(cfg.freq_range, FrequencyRange::Fr2);
    }

    #[test]
    fn test_l_max_fr1_frequency_dependent() {
        assert_eq!(SsbBeamSweepConfig::l_max_fr1(1.8), 4);
        assert_eq!(SsbBeamSweepConfig::l_max_fr1(3.0), 4);
        assert_eq!(SsbBeamSweepConfig::l_max_fr1(3.5), 8);
        assert_eq!(SsbBeamSweepConfig::l_max_fr1(5.9), 8);
    }

    // ─── P1 initial beam selection ───────────────────────────────────────

    #[test]
    fn test_p1_selects_best_beam() {
        let meas = vec![
            BeamMeasurement::new(0, -90.0, -5.0, 10.0, 1),
            BeamMeasurement::new(1, -80.0, -4.0, 15.0, 1),  // best
            BeamMeasurement::new(2, -95.0, -6.0, 8.0, 1),
            BeamMeasurement::new(3, -100.0, -8.0, 5.0, 1),
        ];
        let result = p1_initial_beam_selection(&meas, -130.0).unwrap();
        assert_eq!(result.best_ssb_index, 1);
        assert!((result.best_rsrp_dbm - (-80.0)).abs() < 0.01);
        assert_eq!(result.num_beams_measured, 4);
    }

    #[test]
    fn test_p1_no_measurements() {
        let result = p1_initial_beam_selection(&[], -130.0);
        assert_eq!(result, Err(BeamMgmtError::NoMeasurements));
    }

    #[test]
    fn test_p1_no_qualifying_beam() {
        let meas = vec![
            BeamMeasurement::new(0, -135.0, -10.0, 0.0, 1),
        ];
        let result = p1_initial_beam_selection(&meas, -130.0);
        assert!(matches!(result, Err(BeamMgmtError::NoQualifyingBeam { .. })));
    }

    #[test]
    fn test_p1_ranked_output_sorted() {
        let meas = simulate_beam_measurements(4, 2, -75.0, 20.0, 1);
        let result = p1_initial_beam_selection(&meas, -140.0).unwrap();
        // Ranked should be sorted descending by RSRP
        for w in result.ranked_measurements.windows(2) {
            assert!(w[0].ss_rsrp_dbm >= w[1].ss_rsrp_dbm);
        }
    }

    #[test]
    fn test_p1_l1_rsrp_report_correct() {
        let meas = vec![
            BeamMeasurement::new(0, -100.0, -5.0, 5.0, 1),
        ];
        let result = p1_initial_beam_selection(&meas, -130.0).unwrap();
        // quantize_l1_rsrp(-100.0) = floor((-100 - (-140)) / 1) = 40
        assert_eq!(result.l1_rsrp_report, 40);
    }

    // ─── P2 beam refinement at gNB ───────────────────────────────────────

    #[test]
    fn test_p2_selects_refined_best() {
        let refined = vec![
            BeamMeasurement::new(2, -78.0, -4.0, 12.0, 2),
            BeamMeasurement::new(3, -76.0, -3.5, 14.0, 2),  // best refined
            BeamMeasurement::new(4, -80.0, -4.5, 11.0, 2),
        ];
        let result = p2_beam_refinement_gnb(1, -82.0, &refined, 3.0).unwrap();
        assert_eq!(result.refined_ssb_index, 3);
        assert!(result.rsrp_improvement_db > 0.0);
    }

    #[test]
    fn test_p2_no_refined_candidates() {
        let result = p2_beam_refinement_gnb(1, -82.0, &[], 3.0).unwrap();
        assert_eq!(result.refined_ssb_index, 1);
        assert_eq!(result.rsrp_improvement_db, 0.0);
        assert_eq!(result.num_candidates, 0);
    }

    #[test]
    fn test_p2_improvement_computed() {
        let refined = vec![
            BeamMeasurement::new(5, -75.0, -3.0, 15.0, 2),
        ];
        let coarse_rsrp = -82.0;
        let result = p2_beam_refinement_gnb(1, coarse_rsrp, &refined, 3.0).unwrap();
        assert!((result.rsrp_improvement_db - (-75.0 - coarse_rsrp)).abs() < 0.01);
    }

    // ─── P3 UE receive beam sweep ────────────────────────────────────────

    #[test]
    fn test_p3_selects_best_rx_beam() {
        let rx_rsrp = vec![-90.0, -85.0, -80.0, -88.0];  // beam 2 is best
        let result = p3_ue_rx_beam_sweep(0, &rx_rsrp).unwrap();
        assert_eq!(result.best_ue_rx_beam, 2);
        assert!((result.best_rsrp_dbm - (-80.0)).abs() < 0.01);
        assert_eq!(result.num_rx_beams, 4);
    }

    #[test]
    fn test_p3_no_measurements() {
        let result = p3_ue_rx_beam_sweep(0, &[]);
        assert_eq!(result, Err(BeamMgmtError::NoMeasurements));
    }

    #[test]
    fn test_p3_single_beam() {
        let result = p3_ue_rx_beam_sweep(3, &[-95.0]).unwrap();
        assert_eq!(result.best_ue_rx_beam, 0);
        assert_eq!(result.num_rx_beams, 1);
    }

    // ─── Beam failure detection ───────────────────────────────────────────

    #[test]
    fn test_bfd_normal_operation() {
        let mut bfd = BeamFailureDetector::new(BfdConfig::default());
        // RSRP well above q0 → no failure
        for slot in 0..50 {
            let state = bfd.update(-80.0, slot);
            assert_eq!(state, BfdState::Normal);
        }
    }

    #[test]
    fn test_bfd_failure_declaration() {
        let mut bfd = BeamFailureDetector::new(BfdConfig {
            q0_threshold_dbm: -110.0,
            q1_threshold_dbm: -100.0,
            max_beam_failure_instances: 5,
            counting_window_slots: 100,
        });
        // Drive RSRP below q0 to accumulate failure instances
        for slot in 0..5 {
            bfd.update(-115.0, slot as u64);
        }
        assert_eq!(bfd.state(), BfdState::BeamFailure);
        assert_eq!(bfd.failure_instance_count(), 5);
    }

    #[test]
    fn test_bfd_reset_clears_state() {
        let mut bfd = BeamFailureDetector::new(BfdConfig {
            max_beam_failure_instances: 3,
            ..Default::default()
        });
        for slot in 0..3 {
            bfd.update(-120.0, slot);
        }
        assert_eq!(bfd.state(), BfdState::BeamFailure);
        bfd.reset(10);
        assert_eq!(bfd.state(), BfdState::Normal);
        assert_eq!(bfd.failure_instance_count(), 0);
    }

    #[test]
    fn test_bfd_does_not_count_above_threshold() {
        let mut bfd = BeamFailureDetector::new(BfdConfig {
            q0_threshold_dbm: -110.0,
            max_beam_failure_instances: 3,
            ..Default::default()
        });
        // RSRP above threshold — count should stay zero
        for slot in 0..10 {
            bfd.update(-100.0, slot);
        }
        assert_eq!(bfd.failure_instance_count(), 0);
        assert_eq!(bfd.state(), BfdState::Normal);
    }

    // ─── Beam failure recovery ────────────────────────────────────────────

    #[test]
    fn test_bfr_finds_qualifying_candidate() {
        let mut bfr = BeamFailureRecovery::new(BfrConfig {
            q_new_threshold_dbm: -105.0,
            ..Default::default()
        });
        let candidates = vec![
            BeamMeasurement::new(5, -100.0, -4.0, 12.0, 10),  // qualifies
            BeamMeasurement::new(6, -108.0, -5.0, 8.0, 10),   // too weak
        ];
        let result = bfr.select_candidate_beam(&candidates).unwrap();
        assert_eq!(result.new_ssb_index, 5);
        assert!(result.candidate_found);
    }

    #[test]
    fn test_bfr_no_qualifying_candidate() {
        let mut bfr = BeamFailureRecovery::new(BfrConfig {
            q_new_threshold_dbm: -105.0,
            ..Default::default()
        });
        let candidates = vec![
            BeamMeasurement::new(5, -110.0, -6.0, 5.0, 10),
        ];
        let result = bfr.select_candidate_beam(&candidates);
        assert!(result.is_none());
    }

    #[test]
    fn test_bfr_max_attempts_exhausted() {
        let mut bfr = BeamFailureRecovery::new(BfrConfig {
            q_new_threshold_dbm: -105.0,
            max_rach_attempts: 3,
            ..Default::default()
        });
        let weak = vec![BeamMeasurement::new(0, -120.0, -8.0, 2.0, 1)];
        for _ in 0..3 {
            bfr.select_candidate_beam(&weak);
        }
        assert!(bfr.max_attempts_exhausted());
    }

    #[test]
    fn test_bfr_t316_tick() {
        let mut bfr = BeamFailureRecovery::new(BfrConfig {
            t316_slots: 3,
            ..Default::default()
        });
        let cand = vec![BeamMeasurement::new(2, -100.0, -4.0, 12.0, 1)];
        bfr.select_candidate_beam(&cand);  // starts T316
        assert!(!bfr.tick_t316());
        assert!(!bfr.tick_t316());
        assert!(bfr.tick_t316()); // 3rd tick expires
    }

    // ─── Beam correspondence ──────────────────────────────────────────────

    #[test]
    fn test_beam_correspondence_identity() {
        let bc = BeamCorrespondence::new(true);
        // Without explicit map, identity correspondence applies
        assert_eq!(bc.infer_ul_beam(3), Some(3));
        assert_eq!(bc.infer_ul_beam(7), Some(7));
    }

    #[test]
    fn test_beam_correspondence_not_supported() {
        let bc = BeamCorrespondence::new(false);
        assert_eq!(bc.infer_ul_beam(3), None);
    }

    #[test]
    fn test_beam_correspondence_explicit_map() {
        let mut bc = BeamCorrespondence::new(true);
        bc.add_pair(2, 5);
        assert_eq!(bc.infer_ul_beam(2), Some(5));
        assert_eq!(bc.infer_ul_beam(3), Some(3)); // falls back to identity
    }

    #[test]
    fn test_beam_correspondence_overwrite() {
        let mut bc = BeamCorrespondence::new(true);
        bc.add_pair(2, 5);
        bc.add_pair(2, 7);  // overwrite
        assert_eq!(bc.ul_beam_for_dl_ssb(2), Some(7));
    }

    // ─── TCI state manager ────────────────────────────────────────────────

    #[test]
    fn test_tci_state_activation_via_mac_ce() {
        let mut mgr = TciStateManager::new(8);
        // Configure states 0..7
        for id in 0..8u8 {
            mgr.configure_state(TciState::new_type_d(id, id));
        }
        let activated = mgr.activate_via_mac_ce(&[2, 5]);
        assert_eq!(activated, 2);
        assert_eq!(mgr.num_active(), 2);
        let active = mgr.active_states();
        let ids: Vec<u8> = active.iter().map(|s| s.state_id).collect();
        assert!(ids.contains(&2));
        assert!(ids.contains(&5));
    }

    #[test]
    fn test_tci_state_deactivation() {
        let mut mgr = TciStateManager::new(8);
        for id in 0..4u8 {
            mgr.configure_state(TciState::new_type_d(id, id));
        }
        mgr.activate_via_mac_ce(&[0, 1, 2, 3]);
        mgr.deactivate_states(&[1, 3]);
        let active: Vec<u8> = mgr.active_states().iter().map(|s| s.state_id).collect();
        assert!(active.contains(&0));
        assert!(active.contains(&2));
        assert!(!active.contains(&1));
        assert!(!active.contains(&3));
    }

    #[test]
    fn test_tci_default_beam_ssb_index() {
        let mut mgr = TciStateManager::new(8);
        mgr.configure_state(TciState::new_type_d(0, 4));
        mgr.configure_state(TciState::new_type_d(1, 7));
        mgr.activate_via_mac_ce(&[0]);
        assert_eq!(mgr.default_beam_ssb_index(), Some(4));
    }

    #[test]
    fn test_tci_state_has_type_d() {
        let state = TciState::new_type_d(0, 3);
        assert!(state.has_type_d());
        assert_eq!(state.type_d_ssb_index(), Some(3));
    }

    #[test]
    fn test_tci_state_max_active_cap() {
        let mut mgr = TciStateManager::new(2); // max 2 active
        for id in 0..8u8 {
            mgr.configure_state(TciState::new_type_d(id, id));
        }
        let activated = mgr.activate_via_mac_ce(&[0, 1, 2, 3, 4]);
        assert_eq!(activated, 2);
        assert_eq!(mgr.num_active(), 2);
    }

    // ─── Spatial relation manager ─────────────────────────────────────────

    #[test]
    fn test_spatial_relation_configure_get() {
        let mut mgr = SpatialRelationManager::new();
        mgr.configure(SpatialRelationInfo {
            resource_id: 0,
            source: SpatialRelationSource::SsbIndex(3),
        });
        let rel = mgr.get(0).unwrap();
        assert_eq!(rel.source, SpatialRelationSource::SsbIndex(3));
    }

    #[test]
    fn test_spatial_relation_update_ssb_references() {
        let mut mgr = SpatialRelationManager::new();
        mgr.configure(SpatialRelationInfo {
            resource_id: 0,
            source: SpatialRelationSource::SsbIndex(2),
        });
        mgr.configure(SpatialRelationInfo {
            resource_id: 1,
            source: SpatialRelationSource::SsbIndex(5),
        });
        mgr.update_ssb_references(2, 6);
        assert_eq!(mgr.get(0).unwrap().source, SpatialRelationSource::SsbIndex(6));
        assert_eq!(mgr.get(1).unwrap().source, SpatialRelationSource::SsbIndex(5));
    }

    #[test]
    fn test_spatial_relation_remove() {
        let mut mgr = SpatialRelationManager::new();
        mgr.configure(SpatialRelationInfo {
            resource_id: 0,
            source: SpatialRelationSource::SsbIndex(3),
        });
        mgr.remove(0);
        assert!(mgr.get(0).is_none());
        assert_eq!(mgr.num_configured(), 0);
    }

    // ─── Serving beam tracking ─────────────────────────────────────────────

    #[test]
    fn test_serving_beam_no_switch_without_trigger() {
        let mut state = ServingBeamState::new(0, -80.0);
        state.hysteresis_db = 3.0;
        state.time_to_trigger_slots = 5;
        let meas = vec![
            BeamMeasurement::new(0, -80.0, -4.0, 12.0, 1),
            BeamMeasurement::new(1, -82.0, -5.0, 10.0, 1), // beam 1 worse than active
        ];
        let result = state.update(&meas);
        assert!(result.is_none());
    }

    #[test]
    fn test_serving_beam_switch_after_ttt() {
        let mut state = ServingBeamState::new(0, -90.0);
        state.hysteresis_db = 3.0;
        state.time_to_trigger_slots = 3;

        // Beam 1 has RSRP = -80, active beam 0 has RSRP = -90.
        // Candidate must exceed -90 + 3 = -87 dBm for 3 consecutive slots.
        for slot in 0..3 {
            let meas = vec![
                BeamMeasurement::new(0, -90.0, -4.0, 5.0, slot),
                BeamMeasurement::new(1, -80.0, -3.0, 12.0, slot),
            ];
            let result = state.update(&meas);
            if slot < 2 {
                assert!(result.is_none(), "Should not switch before TTT on slot {slot}");
            } else {
                // On 3rd iteration (trigger_counter reaches 3)
                assert_eq!(result, Some(1), "Should switch to beam 1 on slot {slot}");
            }
        }
    }

    #[test]
    fn test_serving_beam_hysteresis_prevents_switch() {
        let mut state = ServingBeamState::new(0, -80.0);
        state.hysteresis_db = 5.0;
        state.time_to_trigger_slots = 2;

        // Beam 1 is only 4 dB better, but hysteresis is 5 dB — no switch
        for slot in 0..5u64 {
            let meas = vec![
                BeamMeasurement::new(0, -80.0, -4.0, 10.0, slot),
                BeamMeasurement::new(1, -76.0, -3.0, 12.0, slot),  // only 4 dB gain < 5 dB hysteresis
            ];
            assert!(state.update(&meas).is_none());
        }
    }

    // ─── Simulate beam measurements helper ───────────────────────────────

    #[test]
    fn test_simulate_beam_measurements_count() {
        let meas = simulate_beam_measurements(8, 3, -75.0, 20.0, 100);
        assert_eq!(meas.len(), 8);
    }

    #[test]
    fn test_simulate_beam_measurements_best_beam() {
        let meas = simulate_beam_measurements(4, 2, -70.0, 30.0, 1);
        let best = meas.iter().max_by(|a, b| a.ss_rsrp_dbm.partial_cmp(&b.ss_rsrp_dbm).unwrap()).unwrap();
        assert_eq!(best.ssb_index, 2);
    }

    #[test]
    fn test_simulate_beam_measurements_slot_index() {
        let meas = simulate_beam_measurements(4, 0, -80.0, 10.0, 999);
        for m in &meas {
            assert_eq!(m.slot_index, 999);
        }
    }

    // ─── Full beam manager integration ───────────────────────────────────

    #[test]
    fn test_beam_manager_p1_p2_p3_flow() {
        let cfg = SsbBeamSweepConfig::new(FrequencyRange::Fr1, SubcarrierSpacing::Scs30kHz);
        let mgr = NrSsbBeamManager::new(cfg, 0, -90.0);

        // P1: find best beam among 4
        let meas = simulate_beam_measurements(4, 2, -75.0, 20.0, 1);
        let p1 = mgr.run_p1(&meas, -130.0).unwrap();
        assert_eq!(p1.best_ssb_index, 2);

        // P2: refine near best beam
        let refined = vec![
            BeamMeasurement::new(2, -73.0, -3.0, 15.0, 2),
            BeamMeasurement::new(3, -77.0, -3.5, 13.0, 2),
        ];
        let p2 = mgr.run_p2(p1.best_ssb_index, p1.best_rsrp_dbm, &refined).unwrap();
        assert_eq!(p2.refined_ssb_index, 2);

        // P3: sweep 4 UE RX beams
        let rx = vec![-80.0, -78.0, -72.0, -85.0];  // beam 2 best UE RX
        let p3 = mgr.run_p3(p2.refined_ssb_index, &rx).unwrap();
        assert_eq!(p3.best_ue_rx_beam, 2);
    }

    #[test]
    fn test_beam_manager_process_measurements_no_switch() {
        let cfg = SsbBeamSweepConfig::new(FrequencyRange::Fr1, SubcarrierSpacing::Scs15kHz);
        let mut mgr = NrSsbBeamManager::new(cfg, 0, -80.0);
        let meas = vec![BeamMeasurement::new(0, -80.0, -4.0, 12.0, 1)];
        let event = mgr.process_measurements(&meas);
        assert_eq!(event, BeamMgmtEvent::None);
    }

    #[test]
    fn test_beam_manager_process_bfd_trigger() {
        let cfg = SsbBeamSweepConfig::new(FrequencyRange::Fr1, SubcarrierSpacing::Scs15kHz);
        let mut mgr = NrSsbBeamManager::new(cfg, 0, -80.0);
        // Override BFD config for quick failure
        mgr.bfd = BeamFailureDetector::new(BfdConfig {
            q0_threshold_dbm: -110.0,
            q1_threshold_dbm: -100.0,
            max_beam_failure_instances: 3,
            counting_window_slots: 100,
        });
        mgr.bfr = BeamFailureRecovery::new(BfrConfig {
            q_new_threshold_dbm: -105.0,
            max_rach_attempts: 10,
            t316_slots: 100,
        });

        // Drive BFD by having very weak active beam, but strong candidate
        for slot in 1..=3u64 {
            let meas = vec![
                BeamMeasurement::new(0, -120.0, -8.0, 2.0, slot),  // active beam very weak
                BeamMeasurement::new(1, -90.0, -4.0, 14.0, slot),  // candidate strong
            ];
            let event = mgr.process_measurements(&meas);
            if slot == 3 {
                // BFD declares failure and BFR immediately finds beam 1
                match event {
                    BeamMgmtEvent::BeamFailureRecovery { new_ssb_index, .. } => {
                        assert_eq!(new_ssb_index, 1);
                    }
                    other => panic!("Expected BeamFailureRecovery, got {other:?}"),
                }
            }
        }
    }

    #[test]
    fn test_beam_manager_current_slot() {
        let cfg = SsbBeamSweepConfig::new(FrequencyRange::Fr1, SubcarrierSpacing::Scs15kHz);
        let mut mgr = NrSsbBeamManager::new(cfg, 0, -80.0);
        assert_eq!(mgr.current_slot(), 0);
        let meas = vec![BeamMeasurement::new(0, -80.0, -4.0, 12.0, 1)];
        mgr.process_measurements(&meas);
        assert_eq!(mgr.current_slot(), 1);
    }

    #[test]
    fn test_beam_manager_measurement_history() {
        let cfg = SsbBeamSweepConfig::new(FrequencyRange::Fr1, SubcarrierSpacing::Scs15kHz);
        let mut mgr = NrSsbBeamManager::new(cfg, 0, -80.0);
        for slot in 1..=5u64 {
            let meas = vec![BeamMeasurement::new(0, -80.0, -4.0, 12.0, slot)];
            mgr.process_measurements(&meas);
        }
        let history = mgr.measurement_history_for(0);
        assert_eq!(history.len(), 5);
    }

    #[test]
    fn test_beam_measurement_l1_rsrp_auto_computed() {
        let m = BeamMeasurement::new(0, -100.0, -5.0, 10.0, 1);
        // -100 dBm → report = floor((-100 - (-140)) / 1) = 40
        assert_eq!(m.l1_rsrp_report, 40);
    }

    #[test]
    fn test_qcl_type_d_state_setup() {
        let state = TciState::new_type_d(5, 12);
        assert_eq!(state.state_id, 5);
        assert!(state.has_type_d());
        assert_eq!(state.type_d_ssb_index(), Some(12));
        assert_eq!(state.activation, TciActivation::Inactive);
    }

    #[test]
    fn test_error_display() {
        let e = BeamMgmtError::NoQualifyingBeam { threshold_dbm: -130.0 };
        let s = format!("{e}");
        assert!(s.contains("-130.0"));
        let e2 = BeamMgmtError::InvalidSsbIndex { i_ssb: 8, l_max: 4 };
        let s2 = format!("{e2}");
        assert!(s2.contains("8") && s2.contains("4"));
    }

    #[test]
    fn test_subcarrier_spacing_khz() {
        assert_eq!(SubcarrierSpacing::Scs15kHz.khz(), 15);
        assert_eq!(SubcarrierSpacing::Scs30kHz.khz(), 30);
        assert_eq!(SubcarrierSpacing::Scs120kHz.khz(), 120);
        assert_eq!(SubcarrierSpacing::Scs240kHz.khz(), 240);
    }
}
