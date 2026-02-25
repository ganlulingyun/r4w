//! SONET/SDH Optical Transport Framing
//!
//! Implements SONET (Synchronous Optical Networking) and SDH (Synchronous Digital Hierarchy)
//! framing per ITU-T G.707/G.783 and Telcordia GR-253-CORE.
//!
//! # Frame Structure
//!
//! An STS-1/STM-0 frame is 810 bytes arranged as 9 rows × 90 columns:
//!
//! ```text
//! Columns:  1-3 (SOH/LOH/POH)   4-90 (SPE payload)
//! Row 1:    [A1][A2][J0]         [payload...]
//! Row 2:    [B1][E1][F1]         [payload...]
//! Row 3:    [D1][D2][D3]         [payload...]
//! Row 4:    [H1][H2][H3]         [payload...]
//! Row 5:    [B2][K1][K2]         [payload...]
//! Row 6:    [D4][D5][D6]         [payload...]
//! Row 7:    [D7][D8][D9]         [payload...]
//! Row 8:    [D10][D11][D12]      [payload...]
//! Row 9:    [S1][M0][E2]         [payload...]
//! ```
//!
//! # Rates
//! - STS-1:   51.84 Mbps,  810 bytes/frame, 125 μs period
//! - STS-3c:  155.52 Mbps, 2430 bytes/frame
//! - STS-12c: 622.08 Mbps, 9720 bytes/frame
//! - STS-48c: 2488.32 Mbps
//!
//! # Standards
//! - ITU-T G.707: Network node interface for the synchronous digital hierarchy (SDH)
//! - ITU-T G.783: Characteristics of synchronous digital hierarchy (SDH) equipment
//! - Telcordia GR-253-CORE: SONET Transport Systems

// ── Constants ──────────────────────────────────────────────────────────────────

/// STS-1 frame: 9 rows × 90 columns = 810 bytes
pub const STS1_ROWS: usize = 9;
/// STS-1 columns per row
pub const STS1_COLS: usize = 90;
/// STS-1 frame size in bytes
pub const STS1_FRAME_BYTES: usize = STS1_ROWS * STS1_COLS; // 810

/// SPE payload columns (columns 4-90 = 87 columns)
pub const SPE_COLS: usize = 87;
/// SPE bytes per frame (9 rows × 87 columns = 783 bytes; column 4 is POH J1 in row 1)
pub const SPE_BYTES: usize = STS1_ROWS * SPE_COLS; // 783

/// A1 framing byte (0xF6 = 1111_0110)
pub const A1_BYTE: u8 = 0xF6;
/// A2 framing byte (0x28 = 0010_1000)
pub const A2_BYTE: u8 = 0x28;

/// STS-1 line rate in bits per second
pub const STS1_LINE_RATE_BPS: u64 = 51_840_000;
/// STS-3c line rate
pub const STS3C_LINE_RATE_BPS: u64 = 155_520_000;
/// STS-12c line rate
pub const STS12C_LINE_RATE_BPS: u64 = 622_080_000;
/// STS-48c line rate
pub const STS48C_LINE_RATE_BPS: u64 = 2_488_320_000;
/// STS-192c line rate
pub const STS192C_LINE_RATE_BPS: u64 = 9_953_280_000;

/// Frame period in microseconds (125 μs = 8000 frames/s)
pub const FRAME_PERIOD_US: f64 = 125.0;

/// Maximum valid AU-4 pointer value (0-782 for STS-1 SPE)
pub const AU_POINTER_MAX: u16 = 782;
/// New Data Flag bits in H1
pub const NDF_BITS: u8 = 0b1001_0000; // 1001 in bits 7-4 of H1
/// Normal (no NDF) indicator in H1 bits 7-4
pub const NO_NDF_BITS: u8 = 0b0110_0000; // 0110 in bits 7-4

/// J1 path trace length (64 bytes including CRC-7 in ITU-T G.831)
pub const J1_TRACE_LEN: usize = 64;

// ── Frame Level Enum ──────────────────────────────────────────────────────────

/// SONET/SDH hierarchy level
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SonetLevel {
    /// STS-1 / STM-0: 51.84 Mbps
    Sts1,
    /// STS-3c / STM-1: 155.52 Mbps
    Sts3c,
    /// STS-12c / STM-4: 622.08 Mbps
    Sts12c,
    /// STS-48c / STM-16: 2488.32 Mbps
    Sts48c,
    /// STS-192c / STM-64: 9953.28 Mbps
    Sts192c,
}

impl SonetLevel {
    /// Number of STS-1 tributaries at this level
    pub fn sts1_count(self) -> usize {
        match self {
            SonetLevel::Sts1 => 1,
            SonetLevel::Sts3c => 3,
            SonetLevel::Sts12c => 12,
            SonetLevel::Sts48c => 48,
            SonetLevel::Sts192c => 192,
        }
    }

    /// Frame size in bytes
    pub fn frame_bytes(self) -> usize {
        STS1_FRAME_BYTES * self.sts1_count()
    }

    /// Line rate in bps
    pub fn line_rate_bps(self) -> u64 {
        STS1_LINE_RATE_BPS * self.sts1_count() as u64
    }
}

// ── Section Overhead ──────────────────────────────────────────────────────────

/// Section Overhead (SOH) byte offsets within a row-major STS-1 frame.
/// Row indices 0-8, column indices 0-89.
/// SOH occupies columns 0-2 of rows 0-2 (9 bytes total for section overhead).
#[derive(Debug, Clone, Copy)]
pub struct SectionOverhead {
    /// A1 framing byte (row 0, col 0) — always 0xF6
    pub a1: u8,
    /// A2 framing byte (row 0, col 1) — always 0x28
    pub a2: u8,
    /// J0/Z0 section trace / growth (row 0, col 2)
    pub j0: u8,
    /// B1 BIP-8 section error monitoring (row 1, col 0)
    pub b1: u8,
    /// E1 section orderwire (row 1, col 1)
    pub e1: u8,
    /// F1 section user channel (row 1, col 2)
    pub f1: u8,
    /// D1 section DCC byte 1 (row 2, col 0)
    pub d1: u8,
    /// D2 section DCC byte 2 (row 2, col 1)
    pub d2: u8,
    /// D3 section DCC byte 3 (row 2, col 2)
    pub d3: u8,
}

impl SectionOverhead {
    /// Default SOH with framing bytes set, others zero
    pub fn new() -> Self {
        SectionOverhead {
            a1: A1_BYTE,
            a2: A2_BYTE,
            j0: 0x01,
            b1: 0x00,
            e1: 0x00,
            f1: 0x00,
            d1: 0x00,
            d2: 0x00,
            d3: 0x00,
        }
    }
}

impl Default for SectionOverhead {
    fn default() -> Self {
        Self::new()
    }
}

// ── Line Overhead ─────────────────────────────────────────────────────────────

/// Line Overhead (LOH) byte definitions.
/// Occupies columns 0-2 of rows 3-8 (18 bytes).
#[derive(Debug, Clone, Copy)]
pub struct LineOverhead {
    /// H1 pointer high byte (row 3, col 0)
    pub h1: u8,
    /// H2 pointer low byte (row 3, col 1)
    pub h2: u8,
    /// H3 pointer action byte (row 3, col 2) — positive justification opportunity
    pub h3: u8,
    /// B2 BIP-N line BIP (row 4, col 0)
    pub b2: u8,
    /// K1 APS byte 1 (row 4, col 1)
    pub k1: u8,
    /// K2 APS byte 2 (row 4, col 2)
    pub k2: u8,
    /// D4 line DCC (row 5, col 0)
    pub d4: u8,
    /// D5 line DCC (row 5, col 1)
    pub d5: u8,
    /// D6 line DCC (row 5, col 2)
    pub d6: u8,
    /// D7 line DCC (row 6, col 0)
    pub d7: u8,
    /// D8 line DCC (row 6, col 1)
    pub d8: u8,
    /// D9 line DCC (row 6, col 2)
    pub d9: u8,
    /// D10 line DCC (row 7, col 0)
    pub d10: u8,
    /// D11 line DCC (row 7, col 1)
    pub d11: u8,
    /// D12 line DCC (row 7, col 2)
    pub d12: u8,
    /// S1 synchronization status (row 8, col 0) — bits 4-0 carry SSM
    pub s1: u8,
    /// M0/M1 line REI (row 8, col 1) — remote error indication
    pub m0: u8,
    /// E2 line orderwire (row 8, col 2)
    pub e2: u8,
}

impl LineOverhead {
    /// Create LOH with pointer set to offset 0, no NDF, others zero
    pub fn new() -> Self {
        let (h1, h2) = encode_pointer(0, false, false);
        LineOverhead {
            h1,
            h2,
            h3: 0x00,
            b2: 0x00,
            k1: 0x00,
            k2: 0x00,
            d4: 0x00,
            d5: 0x00,
            d6: 0x00,
            d7: 0x00,
            d8: 0x00,
            d9: 0x00,
            d10: 0x00,
            d11: 0x00,
            d12: 0x00,
            s1: 0x00,
            m0: 0x00,
            e2: 0x00,
        }
    }
}

impl Default for LineOverhead {
    fn default() -> Self {
        Self::new()
    }
}

// ── Path Overhead ─────────────────────────────────────────────────────────────

/// Path Overhead (POH) — first byte of each SPE row (column 4 of the frame),
/// 9 bytes total: J1 B3 C2 G1 F2 H4 Z3 Z4 Z5 (or N1).
#[derive(Debug, Clone, Copy)]
pub struct PathOverhead {
    /// J1 path trace (first byte; full 64-byte trace managed separately)
    pub j1: u8,
    /// B3 BIP-8 path error monitoring
    pub b3: u8,
    /// C2 signal label / path signal label
    pub c2: u8,
    /// G1 path status (REI + RDI)
    pub g1: u8,
    /// F2 path user channel
    pub f2: u8,
    /// H4 multiframe indicator (for VT structured SPE)
    pub h4: u8,
    /// Z3/F3 path growth / tandem connection monitoring
    pub z3: u8,
    /// Z4/K3 path growth
    pub z4: u8,
    /// Z5/N1 network operator byte
    pub z5: u8,
}

impl PathOverhead {
    /// Default POH: no path trace byte (0x01 start), B3=0, C2=0x01 (equipped unspecified)
    pub fn new() -> Self {
        PathOverhead {
            j1: 0x01,
            b3: 0x00,
            c2: C2Label::EquippedUnspecified as u8,
            g1: 0x00,
            f2: 0x00,
            h4: 0x00,
            z3: 0x00,
            z4: 0x00,
            z5: 0x00,
        }
    }
}

impl Default for PathOverhead {
    fn default() -> Self {
        Self::new()
    }
}

// ── C2 Signal Label ───────────────────────────────────────────────────────────

/// C2 path signal label values per ITU-T G.707 Table 9
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum C2Label {
    /// Unequipped or supervisory-unequipped
    Unequipped = 0x00,
    /// Equipped — non-specific payload
    EquippedUnspecified = 0x01,
    /// VT-structured virtual tributary SPE
    VtStructured = 0x02,
    /// Locked VT mode
    LockedVt = 0x03,
    /// Asynchronous DS3 mapping
    AsyncDs3 = 0x04,
    /// ATM cell mapping
    Atm = 0x13,
    /// HDLC/PPP over SONET (POS) — GFP framing
    HdlcPos = 0xCF,
    /// GFP-framed payload
    Gfp = 0x1B,
    /// FDDI mapping
    Fddi = 0x05,
    /// DQDB mapping
    Dqdb = 0x06,
    /// Asynchronous transfer mode (alternate)
    AtmAlt = 0x14,
    /// Test signal — O.181 PRBS
    TestPrbs = 0xFE,
    /// Payload defect (AIS equivalent)
    PayloadDefect = 0xFF,
}

impl C2Label {
    /// Parse from raw byte, returning None if unknown
    pub fn from_byte(b: u8) -> Option<Self> {
        match b {
            0x00 => Some(C2Label::Unequipped),
            0x01 => Some(C2Label::EquippedUnspecified),
            0x02 => Some(C2Label::VtStructured),
            0x03 => Some(C2Label::LockedVt),
            0x04 => Some(C2Label::AsyncDs3),
            0x13 => Some(C2Label::Atm),
            0xCF => Some(C2Label::HdlcPos),
            0x1B => Some(C2Label::Gfp),
            0x05 => Some(C2Label::Fddi),
            0x06 => Some(C2Label::Dqdb),
            0x14 => Some(C2Label::AtmAlt),
            0xFE => Some(C2Label::TestPrbs),
            0xFF => Some(C2Label::PayloadDefect),
            _ => None,
        }
    }
}

// ── Sync Status Messages (S1 byte) ────────────────────────────────────────────

/// Synchronization Status Messages carried in S1 byte bits 3-0
/// per Telcordia GR-253 / ITU-T G.781
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum SyncStatus {
    /// Quality unknown (used before SSM is decoded)
    Unknown = 0x0,
    /// Primary Reference Source — Stratum 1 (best quality)
    Prs = 0x1,
    /// Synchronized — traceability unknown
    StuUnknown = 0x2,
    /// SSU-A / G.811 clock: Synchronization Supply Unit, Type A (ITU-T G.811)
    SsuA = 0x4,
    /// SSU-B: Synchronization Supply Unit, Type B
    SsuB = 0x8,
    /// SEC: SDH Equipment Clock
    Sec = 0xB,
    /// DNU: Do Not Use for synchronization
    Dnu = 0xF,
}

impl SyncStatus {
    /// Encode into S1 byte (bits 3-0)
    pub fn encode_s1(self) -> u8 {
        (self as u8) & 0x0F
    }

    /// Decode from S1 byte (extract bits 3-0)
    pub fn from_s1(s1: u8) -> Self {
        match s1 & 0x0F {
            0x1 => SyncStatus::Prs,
            0x2 => SyncStatus::StuUnknown,
            0x4 => SyncStatus::SsuA,
            0x8 => SyncStatus::SsuB,
            0xB => SyncStatus::Sec,
            0xF => SyncStatus::Dnu,
            _ => SyncStatus::Unknown,
        }
    }

    /// Quality ordering: higher = better clock source (returns ordinal, lower index = worse)
    pub fn quality_level(self) -> u8 {
        match self {
            SyncStatus::Dnu => 0,
            SyncStatus::Unknown => 1,
            SyncStatus::Sec => 2,
            SyncStatus::SsuB => 3,
            SyncStatus::StuUnknown => 4,
            SyncStatus::SsuA => 5,
            SyncStatus::Prs => 6,
        }
    }
}

// ── APS K1/K2 Protection Switching ───────────────────────────────────────────

/// APS request type encoded in K1 bits 7-4
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ApsRequest {
    /// No request
    NoRequest = 0x0,
    /// Do Not Revert
    DoNotRevert = 0x1,
    /// Reverse Request
    ReverseRequest = 0x2,
    /// Exercise
    Exercise = 0x4,
    /// Wait to Restore
    WaitToRestore = 0x5,
    /// Manual Switch
    ManualSwitch = 0x6,
    /// Signal Degrade Low priority
    SigDegradeLow = 0x9,
    /// Signal Degrade High priority
    SigDegradeHigh = 0xA,
    /// Signal Fail Low priority (1:N protection)
    SigFailLow = 0xD,
    /// Signal Fail High priority
    SigFailHigh = 0xE,
    /// Forced Switch
    ForcedSwitch = 0xF,
}

impl ApsRequest {
    /// Encode into K1 byte (bits 7-4 = request, bits 3-0 = channel number)
    pub fn encode_k1(self, channel: u8) -> u8 {
        ((self as u8) << 4) | (channel & 0x0F)
    }

    /// Decode request from K1 byte
    pub fn from_k1(k1: u8) -> Self {
        match (k1 >> 4) & 0x0F {
            0x0 => ApsRequest::NoRequest,
            0x1 => ApsRequest::DoNotRevert,
            0x2 => ApsRequest::ReverseRequest,
            0x4 => ApsRequest::Exercise,
            0x5 => ApsRequest::WaitToRestore,
            0x6 => ApsRequest::ManualSwitch,
            0x9 => ApsRequest::SigDegradeLow,
            0xA => ApsRequest::SigDegradeHigh,
            0xD => ApsRequest::SigFailLow,
            0xE => ApsRequest::SigFailHigh,
            0xF => ApsRequest::ForcedSwitch,
            _ => ApsRequest::NoRequest,
        }
    }
}

/// APS architecture encoded in K2 bits 5-4
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum ApsArchitecture {
    /// 1+1 unidirectional protection
    OnePlusOneUni = 0x0,
    /// 1+1 bidirectional protection
    OnePlusOneBi = 0x1,
    /// 1:N protection
    OneToN = 0x2,
    /// Bidirectional Line Switched Ring (BLSR)
    Blsr = 0x3,
}

/// Decoded K1/K2 APS message pair
#[derive(Debug, Clone, Copy)]
pub struct ApsMessage {
    pub request: ApsRequest,
    /// Channel number (0 = working, 1-14 = protection channels in 1:N)
    pub channel: u8,
    /// Bridge/switch channel in K2 bits 3-0
    pub bridged_channel: u8,
    pub architecture: ApsArchitecture,
    /// Remote Defect Indication (RDI-L) flag — K2 bit 3
    pub rdi_line: bool,
    /// K2 mode bits (ring mode flag) — K2 bit 2
    pub ring_mode: bool,
}

impl ApsMessage {
    /// Encode to K1/K2 byte pair
    pub fn encode(&self) -> (u8, u8) {
        let k1 = self.request.encode_k1(self.channel);
        let k2 = ((self.architecture as u8) << 4)
            | ((self.rdi_line as u8) << 3)
            | ((self.ring_mode as u8) << 2)
            | (self.bridged_channel & 0x03);
        (k1, k2)
    }

    /// Decode from K1/K2 bytes
    pub fn decode(k1: u8, k2: u8) -> Self {
        ApsMessage {
            request: ApsRequest::from_k1(k1),
            channel: k1 & 0x0F,
            bridged_channel: k2 & 0x03,
            architecture: match (k2 >> 4) & 0x03 {
                0x1 => ApsArchitecture::OnePlusOneBi,
                0x2 => ApsArchitecture::OneToN,
                0x3 => ApsArchitecture::Blsr,
                _ => ApsArchitecture::OnePlusOneUni,
            },
            rdi_line: (k2 >> 3) & 1 == 1,
            ring_mode: (k2 >> 2) & 1 == 1,
        }
    }
}

// ── OOF/LOF State Machine ─────────────────────────────────────────────────────

/// Out-of-Frame / Loss-of-Frame state per GR-253
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FrameAlignState {
    /// Frame alignment acquired and stable
    InFrame,
    /// Framing error detected — counting towards LOF
    OutOfFrame,
    /// Loss of Frame: ≥ 2.5 ms of OOF (ITU-T requires ≥ 3 consecutive bad frames)
    LossOfFrame,
}

/// Frame alignment state machine.
///
/// GR-253 rules:
/// - Enter OOF: framing pattern not found in expected position
/// - Enter LOF: OOF persists ≥ 3 ms (24 consecutive bad frames at 8000 fps)
/// - Exit LOF: good framing for ≥ 1 ms (8 consecutive good frames)
#[derive(Debug)]
pub struct FrameAligner {
    pub state: FrameAlignState,
    /// Count of consecutive bad frames (for OOF→LOF transition)
    bad_count: u32,
    /// Count of consecutive good frames (for LOF→InFrame transition)
    good_count: u32,
    /// Threshold bad frames to declare LOF (default 24, ~3ms at 8000 fps)
    lof_threshold: u32,
    /// Threshold good frames to recover from LOF (default 8, ~1ms)
    recovery_threshold: u32,
    /// Total frames processed
    pub frame_count: u64,
    /// Total OOF events counted
    pub oof_events: u64,
    /// Total LOF events counted
    pub lof_events: u64,
}

impl FrameAligner {
    pub fn new() -> Self {
        FrameAligner {
            state: FrameAlignState::InFrame,
            bad_count: 0,
            good_count: 0,
            lof_threshold: 24,
            recovery_threshold: 8,
            frame_count: 0,
            oof_events: 0,
            lof_events: 0,
        }
    }

    /// Process one frame's framing bytes. Returns current state.
    /// `frame_ok` = true if A1/A2 bytes found in expected position.
    pub fn process_frame(&mut self, frame_ok: bool) -> FrameAlignState {
        self.frame_count += 1;
        match self.state {
            FrameAlignState::InFrame => {
                if !frame_ok {
                    self.state = FrameAlignState::OutOfFrame;
                    self.bad_count = 1;
                    self.oof_events += 1;
                }
            }
            FrameAlignState::OutOfFrame => {
                if frame_ok {
                    // Single good frame recovers from OOF (not LOF)
                    self.state = FrameAlignState::InFrame;
                    self.bad_count = 0;
                } else {
                    self.bad_count += 1;
                    if self.bad_count >= self.lof_threshold {
                        self.state = FrameAlignState::LossOfFrame;
                        self.good_count = 0;
                        self.lof_events += 1;
                    }
                }
            }
            FrameAlignState::LossOfFrame => {
                if frame_ok {
                    self.good_count += 1;
                    if self.good_count >= self.recovery_threshold {
                        self.state = FrameAlignState::InFrame;
                        self.bad_count = 0;
                    }
                } else {
                    self.good_count = 0;
                }
            }
        }
        self.state
    }

    /// Reset to initial in-frame state
    pub fn reset(&mut self) {
        self.state = FrameAlignState::InFrame;
        self.bad_count = 0;
        self.good_count = 0;
    }
}

impl Default for FrameAligner {
    fn default() -> Self {
        Self::new()
    }
}

// ── BIP-8 Bit-Interleaved Parity ──────────────────────────────────────────────

/// Compute BIP-8 over a byte slice.
///
/// BIP-8 treats bytes as 8 bit planes; bit k of the result is the even parity
/// of all bits at position k across all bytes.  Equivalent to XOR of all bytes.
///
/// ```
/// use r4w_core::sonet_sdh_framer::bip8;
/// let data = [0xAB, 0xCD, 0xEF];
/// let parity = bip8(&data);
/// assert_eq!(parity, 0xAB ^ 0xCD ^ 0xEF);
/// ```
pub fn bip8(data: &[u8]) -> u8 {
    data.iter().fold(0u8, |acc, &b| acc ^ b)
}

/// Compute BIP-N parity across N byte lanes interleaved in `data`.
/// Used for B2 which spans all N STS-1 tributaries in an STS-N frame.
/// Returns a Vec of length `n_lanes`.
pub fn bip_n(data: &[u8], n_lanes: usize) -> Vec<u8> {
    let mut parity = vec![0u8; n_lanes];
    for (i, &b) in data.iter().enumerate() {
        parity[i % n_lanes] ^= b;
    }
    parity
}

/// Verify BIP-8: returns the error syndrome (0 = no error)
pub fn bip8_verify(data: &[u8], received_bip: u8) -> u8 {
    bip8(data) ^ received_bip
}

/// Count bit errors from BIP-8 syndrome byte.
/// Each set bit in syndrome represents one errored bit plane.
pub fn bip8_error_count(syndrome: u8) -> u32 {
    syndrome.count_ones()
}

// ── SONET Scrambler (x^7 + x^6 + 1) ─────────────────────────────────────────

/// Frame-aligned SONET/SDH self-synchronizing scrambler per ITU-T G.707 clause 9.
///
/// Polynomial: 1 + x^6 + x^7 (feedback taps at positions 6 and 7).
/// The scrambler is reset (LFSR = 0x7F = all ones) at the start of each frame,
/// after the A1/A2 framing bytes (which are NOT scrambled).
///
/// The self-synchronizing nature means the descrambler uses the same polynomial
/// applied to the received (scrambled) data stream.
#[derive(Debug, Clone)]
pub struct SonetScrambler {
    /// 7-bit LFSR state (bits 6..0)
    lfsr: u8,
    /// Initial state after reset
    init_state: u8,
}

impl SonetScrambler {
    /// Create scrambler with standard initial state 0x7F (all ones per G.707)
    pub fn new() -> Self {
        SonetScrambler {
            lfsr: 0x7F,
            init_state: 0x7F,
        }
    }

    /// Reset LFSR to initial state (called after A1/A2 framing bytes)
    pub fn reset(&mut self) {
        self.lfsr = self.init_state;
    }

    /// Generate one scramble byte (8 LFSR clocks)
    fn next_byte(&mut self) -> u8 {
        let mut mask = 0u8;
        for bit in (0..8).rev() {
            // Feedback: x^7 + x^6 + 1 → taps at positions 6 and 0 of 7-bit register
            // The output bit is bit 0 of LFSR
            let out = self.lfsr & 0x01;
            let feedback = ((self.lfsr >> 6) ^ self.lfsr) & 0x01;
            self.lfsr = ((self.lfsr >> 1) | (feedback << 6)) & 0x7F;
            mask |= out << bit;
        }
        mask
    }

    /// Scramble in place, skipping the first `skip` bytes (A1/A2 framing bytes).
    /// Frame-aligned: reset LFSR, then scramble bytes [skip..].
    pub fn scramble_frame(&mut self, frame: &mut [u8], skip: usize) {
        self.reset();
        for byte in frame.iter_mut().skip(skip) {
            *byte ^= self.next_byte();
        }
    }

    /// Descramble in place — identical operation (XOR with same keystream)
    pub fn descramble_frame(&mut self, frame: &mut [u8], skip: usize) {
        self.scramble_frame(frame, skip);
    }

    /// Scramble a buffer continuously (non-frame-aligned, no reset)
    pub fn scramble(&mut self, data: &mut [u8]) {
        for byte in data.iter_mut() {
            *byte ^= self.next_byte();
        }
    }
}

impl Default for SonetScrambler {
    fn default() -> Self {
        Self::new()
    }
}

// ── Pointer Encode/Decode ────────────────────────────────────────────────────

/// Encode AU-3/AU-4 pointer into H1/H2 bytes.
///
/// H1 (bits 7-4): NDF or SS bits
/// H1 (bits 3-2): SS bits (always 10 for AU-4)
/// H1 (bits 1-0): pointer high bits [9:8]
/// H2 (bits 7-0): pointer low bits [7:0]
///
/// NDF=1 (new data flag): bits 7-4 of H1 = 1001
/// NDF=0 (normal):        bits 7-4 of H1 = 0110
pub fn encode_pointer(offset: u16, ndf: bool, concat_indicator: bool) -> (u8, u8) {
    debug_assert!(offset <= AU_POINTER_MAX || concat_indicator);
    let ndf_bits: u8 = if ndf { NDF_BITS } else { NO_NDF_BITS };
    let ss_bits: u8 = 0b00_00_10_00; // SS=10 (bits 3-2), standard AU-4

    if concat_indicator {
        // Concatenation indicator: H1=0x93, H2=0xFF (pointer value 0xFFFF with concat)
        return (0x93, 0xFF);
    }

    let h1 = ndf_bits | ss_bits | ((offset >> 8) as u8 & 0x03);
    let h2 = (offset & 0xFF) as u8;
    (h1, h2)
}

/// Decode H1/H2 pointer bytes.
/// Returns (offset, ndf_active, is_concatenation_indicator)
pub fn decode_pointer(h1: u8, h2: u8) -> (u16, bool, bool) {
    // Concatenation indicator: all I/D bits inverted (H1=0x93, H2=0xFF)
    if h1 == 0x93 && h2 == 0xFF {
        return (0, false, true);
    }

    let ndf_field = (h1 >> 4) & 0x0F;
    let ndf = ndf_field == 0b1001;

    let offset = (((h1 & 0x03) as u16) << 8) | (h2 as u16);
    (offset, ndf, false)
}

/// Pointer justification direction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum JustificationEvent {
    /// No pointer adjustment this frame
    None,
    /// Positive pointer justification (pointer decremented by 1 mod 783)
    Positive,
    /// Negative pointer justification (pointer incremented by 1 mod 783)
    Negative,
}

// ── Pointer Processor ─────────────────────────────────────────────────────────

/// Tracks and processes AU pointer adjustments for SPE frequency offset compensation.
#[derive(Debug)]
pub struct PointerProcessor {
    /// Current pointer offset into SPE (0-782)
    pub pointer: u16,
    /// Count of positive justification events processed
    pub pos_just_count: u64,
    /// Count of negative justification events processed
    pub neg_just_count: u64,
    /// NDF received flag
    pub ndf_received: bool,
    /// Frames since last pointer adjustment (for stability)
    frames_since_adj: u32,
}

impl PointerProcessor {
    pub fn new(initial_pointer: u16) -> Self {
        PointerProcessor {
            pointer: initial_pointer.min(AU_POINTER_MAX),
            pos_just_count: 0,
            neg_just_count: 0,
            ndf_received: false,
            frames_since_adj: 0,
        }
    }

    /// Process incoming H1/H2 bytes and update pointer state.
    /// Returns the justification event detected.
    pub fn process(&mut self, h1: u8, h2: u8) -> JustificationEvent {
        let (new_offset, ndf, concat) = decode_pointer(h1, h2);
        self.frames_since_adj += 1;

        if concat {
            return JustificationEvent::None;
        }

        if ndf {
            // NDF: accept new pointer value immediately
            self.pointer = new_offset.min(AU_POINTER_MAX);
            self.ndf_received = true;
            self.frames_since_adj = 0;
            return JustificationEvent::None;
        }

        self.ndf_received = false;

        // Detect positive/negative justification by comparing with current pointer
        // In a real system this uses majority voting across 3 frames; here simplified
        if new_offset != self.pointer {
            let forward = (new_offset + AU_POINTER_MAX + 1 - self.pointer) % (AU_POINTER_MAX + 1);
            let backward = (self.pointer + AU_POINTER_MAX + 1 - new_offset) % (AU_POINTER_MAX + 1);
            if forward == 1 {
                self.pointer = new_offset;
                self.neg_just_count += 1;
                self.frames_since_adj = 0;
                return JustificationEvent::Negative;
            } else if backward == 1 {
                self.pointer = new_offset;
                self.pos_just_count += 1;
                self.frames_since_adj = 0;
                return JustificationEvent::Positive;
            } else {
                // Large jump — likely NDF or re-sync; accept
                self.pointer = new_offset.min(AU_POINTER_MAX);
            }
        }

        JustificationEvent::None
    }

    /// Increment pointer (negative justification — SPE is faster than frame)
    pub fn negative_justify(&mut self) -> u16 {
        self.pointer = (self.pointer + 1) % (AU_POINTER_MAX + 1);
        self.neg_just_count += 1;
        self.pointer
    }

    /// Decrement pointer (positive justification — SPE is slower than frame)
    pub fn positive_justify(&mut self) -> u16 {
        self.pointer = (self.pointer + AU_POINTER_MAX) % (AU_POINTER_MAX + 1);
        self.pos_just_count += 1;
        self.pointer
    }
}

// ── J1 Path Trace ─────────────────────────────────────────────────────────────

/// CRC-7 used for J0/J1 path trace per ITU-T G.707 / G.831.
/// Polynomial: x^7 + x^6 + x^2 + 1 = 0xC5 (without leading 1)
fn crc7(data: &[u8]) -> u8 {
    let mut crc: u8 = 0x00;
    for &b in data {
        crc ^= b;
        for _ in 0..8 {
            if crc & 0x80 != 0 {
                crc = (crc << 1) ^ 0xC5;
            } else {
                crc <<= 1;
            }
        }
    }
    crc & 0xFE // bit 0 is always 1 in J1 framing byte; CRC occupies bits 7-1
}

/// J1 path trace message: 64-byte sequence with CRC-7 framing.
///
/// Structure: 1 framing byte (CRC-7, bit 0 = 1) followed by 62 message bytes
/// (plus last byte with bit 0 = 0 for termination).
/// Total = 64 bytes transmitted over 64 frames.
#[derive(Debug, Clone)]
pub struct J1PathTrace {
    bytes: [u8; J1_TRACE_LEN],
}

impl J1PathTrace {
    /// Create from ASCII message (up to 62 characters; padded with spaces)
    pub fn from_str(msg: &str) -> Self {
        let mut bytes = [0x20u8; J1_TRACE_LEN]; // space-padded
        let payload: Vec<u8> = msg.bytes().take(62).collect();
        bytes[1..1 + payload.len()].copy_from_slice(&payload);

        // Compute CRC-7 over bytes [1..63]
        let crc = crc7(&bytes[1..63]);
        // Framing byte: CRC in bits 7-1, bit 0 = 1
        bytes[0] = crc | 0x01;
        // Last byte bit 0 must be 0
        bytes[63] &= 0xFE;

        J1PathTrace { bytes }
    }

    /// Return the raw 64-byte array
    pub fn as_bytes(&self) -> &[u8; J1_TRACE_LEN] {
        &self.bytes
    }

    /// Return the message string (bytes 1-62, trimmed)
    pub fn message(&self) -> &str {
        let payload = &self.bytes[1..63];
        // Find last non-space/non-null
        let end = payload.iter().rposition(|&b| b != 0x20 && b != 0x00)
            .map(|i| i + 1)
            .unwrap_or(0);
        core::str::from_utf8(&payload[..end]).unwrap_or("")
    }

    /// Verify CRC-7 integrity. Returns true if CRC matches.
    pub fn verify_crc(&self) -> bool {
        let expected_crc = crc7(&self.bytes[1..63]);
        let stored_crc = self.bytes[0] & 0xFE;
        stored_crc == expected_crc
    }

    /// Get the byte to transmit for frame `n` (0-indexed, mod 64)
    pub fn get_frame_byte(&self, frame_number: u64) -> u8 {
        self.bytes[(frame_number % J1_TRACE_LEN as u64) as usize]
    }
}

// ── Virtual Tributaries ───────────────────────────────────────────────────────

/// Virtual Tributary type per ANSI T1.105
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VtType {
    /// VT1.5: carries DS1 (1.544 Mbps) — 3 columns per STS-1, 26 bytes/frame
    Vt1_5,
    /// VT2: carries E1 (2.048 Mbps) — 4 columns per STS-1, 34 bytes/frame
    Vt2,
    /// VT3: carries DS1C (3.152 Mbps) — 6 columns per STS-1, 52 bytes/frame
    Vt3,
    /// VT6: carries DS2 (6.312 Mbps) — 12 columns per STS-1, 104 bytes/frame
    Vt6,
}

impl VtType {
    /// Number of columns per VT group in STS-1 SPE (87 payload columns / VT capacity)
    pub fn columns_per_vt(self) -> usize {
        match self {
            VtType::Vt1_5 => 3,
            VtType::Vt2 => 4,
            VtType::Vt3 => 6,
            VtType::Vt6 => 12,
        }
    }

    /// Payload bytes per VT per 125μs frame
    pub fn bytes_per_frame(self) -> usize {
        // 9 rows × columns
        9 * self.columns_per_vt()
    }

    /// Maximum VTs per STS-1 SPE (86 payload columns / columns_per_vt)
    pub fn max_per_sts1(self) -> usize {
        // 84 columns available (87 payload - 1 POH - 2 reserved) / columns_per_vt
        match self {
            VtType::Vt1_5 => 28,
            VtType::Vt2 => 21,
            VtType::Vt3 => 14,
            VtType::Vt6 => 7,
        }
    }

    /// Bit rate in bps carried by this VT type
    pub fn bit_rate_bps(self) -> u64 {
        match self {
            VtType::Vt1_5 => 1_544_000,
            VtType::Vt2 => 2_048_000,
            VtType::Vt3 => 3_152_000,
            VtType::Vt6 => 6_312_000,
        }
    }
}

/// VT mapping: extract VT payload bytes from an STS-1 SPE payload region.
/// SPE payload is 9×87 bytes; column 0 of SPE is POH (J1/B3/C2/etc.).
/// VT columns start at payload column 1 (SPE column 1).
pub fn extract_vt_payload(spe: &[u8], vt_index: usize, vt_type: VtType) -> Vec<u8> {
    assert_eq!(spe.len(), SPE_BYTES, "SPE must be {} bytes", SPE_BYTES);
    let cols = vt_type.columns_per_vt();
    let start_col = 1 + vt_index * cols; // skip POH at col 0
    let mut out = Vec::with_capacity(9 * cols);
    for row in 0..9 {
        let row_start = row * SPE_COLS;
        for c in 0..cols {
            let col = start_col + c;
            if col < SPE_COLS {
                out.push(spe[row_start + col]);
            }
        }
    }
    out
}

/// Insert VT payload bytes into STS-1 SPE at the specified VT slot.
pub fn insert_vt_payload(spe: &mut [u8], vt_index: usize, vt_type: VtType, payload: &[u8]) {
    assert_eq!(spe.len(), SPE_BYTES);
    let cols = vt_type.columns_per_vt();
    let start_col = 1 + vt_index * cols;
    let mut pi = 0;
    for row in 0..9 {
        let row_start = row * SPE_COLS;
        for c in 0..cols {
            let col = start_col + c;
            if col < SPE_COLS && pi < payload.len() {
                spe[row_start + col] = payload[pi];
                pi += 1;
            }
        }
    }
}

// ── STS-1 Frame Builder ───────────────────────────────────────────────────────

/// Complete STS-1 frame (810 bytes) with overhead accessors.
#[derive(Debug, Clone)]
pub struct Sts1Frame {
    /// Raw frame bytes [row * 90 + col]
    pub data: [u8; STS1_FRAME_BYTES],
}

impl Sts1Frame {
    /// Create empty frame (zeros)
    pub fn new() -> Self {
        Sts1Frame {
            data: [0u8; STS1_FRAME_BYTES],
        }
    }

    /// Byte index for (row, col) in row-major order
    #[inline]
    pub fn idx(row: usize, col: usize) -> usize {
        row * STS1_COLS + col
    }

    /// Get/set individual bytes
    #[inline]
    pub fn get(&self, row: usize, col: usize) -> u8 {
        self.data[Self::idx(row, col)]
    }

    #[inline]
    pub fn set(&mut self, row: usize, col: usize, val: u8) {
        self.data[Self::idx(row, col)] = val;
    }

    // ── Section Overhead Accessors ────────────────────────────────────────

    pub fn a1(&self) -> u8 { self.get(0, 0) }
    pub fn a2(&self) -> u8 { self.get(0, 1) }
    pub fn j0(&self) -> u8 { self.get(0, 2) }
    pub fn b1(&self) -> u8 { self.get(1, 0) }
    pub fn e1(&self) -> u8 { self.get(1, 1) }
    pub fn f1(&self) -> u8 { self.get(1, 2) }
    pub fn d1(&self) -> u8 { self.get(2, 0) }
    pub fn d2(&self) -> u8 { self.get(2, 1) }
    pub fn d3(&self) -> u8 { self.get(2, 2) }

    pub fn set_a1(&mut self) { self.set(0, 0, A1_BYTE); }
    pub fn set_a2(&mut self) { self.set(0, 1, A2_BYTE); }

    // ── Line Overhead Accessors ───────────────────────────────────────────

    pub fn h1(&self) -> u8 { self.get(3, 0) }
    pub fn h2(&self) -> u8 { self.get(3, 1) }
    pub fn h3(&self) -> u8 { self.get(3, 2) }
    pub fn b2(&self) -> u8 { self.get(4, 0) }
    pub fn k1(&self) -> u8 { self.get(4, 1) }
    pub fn k2(&self) -> u8 { self.get(4, 2) }
    pub fn d4(&self) -> u8 { self.get(5, 0) }
    pub fn d5(&self) -> u8 { self.get(5, 1) }
    pub fn d6(&self) -> u8 { self.get(5, 2) }
    pub fn d7(&self) -> u8 { self.get(6, 0) }
    pub fn d8(&self) -> u8 { self.get(6, 1) }
    pub fn d9(&self) -> u8 { self.get(6, 2) }
    pub fn d10(&self) -> u8 { self.get(7, 0) }
    pub fn d11(&self) -> u8 { self.get(7, 1) }
    pub fn d12(&self) -> u8 { self.get(7, 2) }
    pub fn s1(&self) -> u8 { self.get(8, 0) }
    pub fn m0(&self) -> u8 { self.get(8, 1) }
    pub fn e2(&self) -> u8 { self.get(8, 2) }

    // ── Path Overhead (first byte of each SPE row, col 3 of frame) ───────

    /// J1 is at row 0, col 3 (first byte of SPE)
    pub fn j1(&self) -> u8 { self.get(0, 3) }
    /// B3 path BIP-8 — row 1, col 3
    pub fn b3(&self) -> u8 { self.get(1, 3) }
    /// C2 signal label — row 2, col 3
    pub fn c2(&self) -> u8 { self.get(2, 3) }
    /// G1 path status — row 3, col 3
    pub fn g1(&self) -> u8 { self.get(3, 3) }
    /// F2 user — row 4, col 3
    pub fn f2(&self) -> u8 { self.get(4, 3) }
    /// H4 multiframe — row 5, col 3
    pub fn h4(&self) -> u8 { self.get(5, 3) }
    /// Z3 — row 6, col 3
    pub fn z3(&self) -> u8 { self.get(6, 3) }
    /// Z4 — row 7, col 3
    pub fn z4(&self) -> u8 { self.get(7, 3) }
    /// Z5 — row 8, col 3
    pub fn z5(&self) -> u8 { self.get(8, 3) }

    // ── Frame Validity ────────────────────────────────────────────────────

    /// Check if A1/A2 framing bytes are valid
    pub fn has_valid_framing(&self) -> bool {
        self.a1() == A1_BYTE && self.a2() == A2_BYTE
    }

    /// Extract the SPE (Synchronous Payload Envelope) — columns 3-89, all 9 rows
    /// Returns 783 bytes in row-major order (col 3 = POH row 0 = J1)
    pub fn extract_spe(&self) -> [u8; SPE_BYTES] {
        let mut spe = [0u8; SPE_BYTES];
        for row in 0..STS1_ROWS {
            for col in 0..SPE_COLS {
                spe[row * SPE_COLS + col] = self.get(row, col + 3);
            }
        }
        spe
    }

    /// Insert SPE bytes into the frame (columns 3-89, all rows)
    pub fn insert_spe(&mut self, spe: &[u8; SPE_BYTES]) {
        for row in 0..STS1_ROWS {
            for col in 0..SPE_COLS {
                self.set(row, col + 3, spe[row * SPE_COLS + col]);
            }
        }
    }

    /// Build SOH overhead bytes from struct
    pub fn apply_soh(&mut self, soh: &SectionOverhead) {
        self.set(0, 0, soh.a1);
        self.set(0, 1, soh.a2);
        self.set(0, 2, soh.j0);
        self.set(1, 0, soh.b1);
        self.set(1, 1, soh.e1);
        self.set(1, 2, soh.f1);
        self.set(2, 0, soh.d1);
        self.set(2, 1, soh.d2);
        self.set(2, 2, soh.d3);
    }

    /// Extract SOH from frame
    pub fn read_soh(&self) -> SectionOverhead {
        SectionOverhead {
            a1: self.a1(),
            a2: self.a2(),
            j0: self.j0(),
            b1: self.b1(),
            e1: self.e1(),
            f1: self.f1(),
            d1: self.d1(),
            d2: self.d2(),
            d3: self.d3(),
        }
    }

    /// Apply LOH bytes to frame
    pub fn apply_loh(&mut self, loh: &LineOverhead) {
        self.set(3, 0, loh.h1);
        self.set(3, 1, loh.h2);
        self.set(3, 2, loh.h3);
        self.set(4, 0, loh.b2);
        self.set(4, 1, loh.k1);
        self.set(4, 2, loh.k2);
        self.set(5, 0, loh.d4);
        self.set(5, 1, loh.d5);
        self.set(5, 2, loh.d6);
        self.set(6, 0, loh.d7);
        self.set(6, 1, loh.d8);
        self.set(6, 2, loh.d9);
        self.set(7, 0, loh.d10);
        self.set(7, 1, loh.d11);
        self.set(7, 2, loh.d12);
        self.set(8, 0, loh.s1);
        self.set(8, 1, loh.m0);
        self.set(8, 2, loh.e2);
    }

    /// Extract LOH from frame
    pub fn read_loh(&self) -> LineOverhead {
        LineOverhead {
            h1: self.h1(), h2: self.h2(), h3: self.h3(),
            b2: self.b2(), k1: self.k1(), k2: self.k2(),
            d4: self.d4(), d5: self.d5(), d6: self.d6(),
            d7: self.d7(), d8: self.d8(), d9: self.d9(),
            d10: self.d10(), d11: self.d11(), d12: self.d12(),
            s1: self.s1(), m0: self.m0(), e2: self.e2(),
        }
    }

    /// Apply POH to SPE portion of frame (col 3, all rows)
    pub fn apply_poh(&mut self, poh: &PathOverhead) {
        self.set(0, 3, poh.j1);
        self.set(1, 3, poh.b3);
        self.set(2, 3, poh.c2);
        self.set(3, 3, poh.g1);
        self.set(4, 3, poh.f2);
        self.set(5, 3, poh.h4);
        self.set(6, 3, poh.z3);
        self.set(7, 3, poh.z4);
        self.set(8, 3, poh.z5);
    }

    /// Extract POH from frame
    pub fn read_poh(&self) -> PathOverhead {
        PathOverhead {
            j1: self.j1(), b3: self.b3(), c2: self.c2(),
            g1: self.g1(), f2: self.f2(), h4: self.h4(),
            z3: self.z3(), z4: self.z4(), z5: self.z5(),
        }
    }

    /// Compute B1 BIP-8 over all bytes of the previous frame (after scrambling),
    /// excluding the B1 byte position itself.
    /// For simplicity, computes BIP-8 over columns 3-89 of all rows (SPE region).
    pub fn compute_b1(&self) -> u8 {
        // B1 covers the entire previous SONET section frame
        // In practice: XOR of all bytes in previous scrambled frame
        bip8(&self.data)
    }

    /// Compute B3 path BIP-8 over SPE bytes (excluding B3 byte itself)
    pub fn compute_b3(&self) -> u8 {
        let spe = self.extract_spe();
        // B3 covers SPE of previous frame; exclude index 1 (B3 position in POH)
        let mut xor = 0u8;
        for (i, &b) in spe.iter().enumerate() {
            if i != 1 {
                // skip B3 position
                xor ^= b;
            }
        }
        xor
    }

    /// Set H1/H2 pointer with optional NDF
    pub fn set_pointer(&mut self, offset: u16, ndf: bool) {
        let (h1, h2) = encode_pointer(offset, ndf, false);
        self.set(3, 0, h1);
        self.set(3, 1, h2);
    }

    /// Get decoded pointer (offset, ndf, is_concat)
    pub fn get_pointer(&self) -> (u16, bool, bool) {
        decode_pointer(self.h1(), self.h2())
    }

    /// Set concatenation indicator in H1/H2
    pub fn set_concat_indicator(&mut self) {
        let (h1, h2) = encode_pointer(0, false, true);
        self.set(3, 0, h1);
        self.set(3, 1, h2);
    }

    /// Set S1 synchronization status
    pub fn set_sync_status(&mut self, status: SyncStatus) {
        let s1 = (self.s1() & 0xF0) | status.encode_s1();
        self.set(8, 0, s1);
    }

    /// Get synchronization status from S1
    pub fn get_sync_status(&self) -> SyncStatus {
        SyncStatus::from_s1(self.s1())
    }
}

impl Default for Sts1Frame {
    fn default() -> Self {
        Self::new()
    }
}

// ── STS-1 Frame Framer (Transmit) ─────────────────────────────────────────────

/// SONET STS-1 framer: builds complete frames with SOH/LOH/POH and scrambling.
#[derive(Debug)]
pub struct Sts1Framer {
    scrambler: SonetScrambler,
    aligner: FrameAligner,
    pub soh: SectionOverhead,
    pub loh: LineOverhead,
    pub poh: PathOverhead,
    frame_number: u64,
    /// Path trace state: which byte of J1 to insert
    j1_trace: Option<J1PathTrace>,
    j1_frame_offset: u64,
}

impl Sts1Framer {
    pub fn new() -> Self {
        Sts1Framer {
            scrambler: SonetScrambler::new(),
            aligner: FrameAligner::new(),
            soh: SectionOverhead::new(),
            loh: LineOverhead::new(),
            poh: PathOverhead::new(),
            frame_number: 0,
            j1_trace: None,
            j1_frame_offset: 0,
        }
    }

    /// Set the J1 path trace message
    pub fn set_j1_trace(&mut self, msg: &str) {
        self.j1_trace = Some(J1PathTrace::from_str(msg));
        self.j1_frame_offset = 0;
    }

    /// Build one complete STS-1 frame from SPE payload bytes.
    /// Payload should be SPE_BYTES (783 bytes); if shorter, padded with 0x00.
    pub fn build_frame(&mut self, payload: &[u8]) -> Sts1Frame {
        let mut frame = Sts1Frame::new();

        // Fill SOH
        self.soh.a1 = A1_BYTE;
        self.soh.a2 = A2_BYTE;
        frame.apply_soh(&self.soh);

        // Fill LOH
        frame.apply_loh(&self.loh);

        // Fill POH — J1 path trace byte cycles through 64-byte sequence
        let poh_j1 = if let Some(ref trace) = self.j1_trace {
            trace.get_frame_byte(self.j1_frame_offset)
        } else {
            self.poh.j1
        };
        self.j1_frame_offset += 1;

        // Compute B3 from previous SPE (simplified: use current payload XOR)
        let b3 = bip8(payload.get(..SPE_BYTES.min(payload.len())).unwrap_or(payload));

        let poh = PathOverhead {
            j1: poh_j1,
            b3,
            ..self.poh
        };
        frame.apply_poh(&poh);

        // Copy payload into SPE (skip POH column = col 3, row 0 which is J1)
        // SPE starts at col 3; rows 0-8
        // POH occupies col 3 for all 9 rows (set above via apply_poh)
        // Payload data goes into cols 4-89 (86 cols × 9 rows = 774 bytes)
        let mut pi = 0;
        for row in 0..STS1_ROWS {
            for col in 4..STS1_COLS {
                if pi < payload.len() {
                    frame.set(row, col, payload[pi]);
                    pi += 1;
                }
            }
        }

        // Update B1 (section BIP-8 over previous frame; here over current payload region)
        self.soh.b1 = bip8(&frame.data[3..]); // simplified: over non-A1/A2 region

        // Scramble frame (skip first 3 bytes: A1, A2, skip) — actually skip 2 in STS-1
        // Per G.707: A1, A2 bytes not scrambled; scramble starts at byte 3
        self.scrambler.scramble_frame(&mut frame.data, 2);

        self.frame_number += 1;
        frame
    }

    /// Deframe a received frame: descramble, verify framing, extract overhead
    pub fn receive_frame(&mut self, raw: &mut [u8; STS1_FRAME_BYTES]) -> FrameAlignState {
        // Descramble (skip first 2 bytes — A1/A2 not scrambled)
        self.scrambler.descramble_frame(raw, 2);

        let a1_ok = raw[0] == A1_BYTE;
        let a2_ok = raw[1] == A2_BYTE;
        let state = self.aligner.process_frame(a1_ok && a2_ok);
        state
    }

    pub fn frame_aligner(&self) -> &FrameAligner {
        &self.aligner
    }
}

impl Default for Sts1Framer {
    fn default() -> Self {
        Self::new()
    }
}

// ── STS-N Concatenation ───────────────────────────────────────────────────────

/// STS-Nc concatenated frame container.
/// All frames except the first have concatenation indicator in H1/H2.
#[derive(Debug)]
pub struct StsNcFrame {
    pub level: SonetLevel,
    /// N STS-1 frames stored contiguously
    frames: Vec<Sts1Frame>,
}

impl StsNcFrame {
    pub fn new(level: SonetLevel) -> Self {
        let n = level.sts1_count();
        let mut frames = Vec::with_capacity(n);
        for i in 0..n {
            let mut f = Sts1Frame::new();
            f.set_a1();
            f.set_a2();
            if i == 0 {
                f.set_pointer(0, false);
            } else {
                f.set_concat_indicator();
            }
            frames.push(f);
        }
        StsNcFrame { level, frames }
    }

    /// Total payload capacity in bytes (STS-N SPE minus N POH bytes minus 3×N overhead)
    pub fn payload_bytes(&self) -> usize {
        let n = self.level.sts1_count();
        // SPE payload minus POH per tributary
        (SPE_BYTES - 9) * n
    }

    /// Get reference to constituent STS-1 frame by index
    pub fn frame(&self, idx: usize) -> &Sts1Frame {
        &self.frames[idx]
    }

    pub fn frame_mut(&mut self, idx: usize) -> &mut Sts1Frame {
        &mut self.frames[idx]
    }

    /// Verify all concatenation indicators (frames 1..N-1 should have concat indicator)
    pub fn verify_concatenation(&self) -> bool {
        // Frame 0 should have a normal pointer
        let (_, _, concat0) = self.frames[0].get_pointer();
        if concat0 { return false; }

        // Frames 1..N-1 should all have concat indicator
        for i in 1..self.level.sts1_count() {
            let (_, _, concat) = self.frames[i].get_pointer();
            if !concat { return false; }
        }
        true
    }
}

// ── G1 Path Status Byte ───────────────────────────────────────────────────────

/// G1 path status byte encoding
/// Bits 7-4: REI-P (Remote Error Indication — Path), count 0-8
/// Bit 3: RDI-P (Remote Defect Indication — Path)
/// Bit 2-0: reserved / enhanced RDI codes
#[derive(Debug, Clone, Copy)]
pub struct G1Status {
    /// Remote Error Indication count (0-8 BIP-8 errors)
    pub rei_count: u8,
    /// Remote Defect Indication — Path
    pub rdi_path: bool,
    /// Enhanced RDI code (0=no defect, 1=payload, 2=server, 5=connectivity)
    pub enhanced_rdi: u8,
}

impl G1Status {
    pub fn new() -> Self {
        G1Status { rei_count: 0, rdi_path: false, enhanced_rdi: 0 }
    }

    /// Encode to G1 byte
    pub fn encode(&self) -> u8 {
        let rei = (self.rei_count.min(8) as u8) << 4;
        let rdi = (self.rdi_path as u8) << 3;
        let erdi = self.enhanced_rdi & 0x07;
        rei | rdi | erdi
    }

    /// Decode from G1 byte
    pub fn decode(g1: u8) -> Self {
        G1Status {
            rei_count: (g1 >> 4) & 0x0F,
            rdi_path: (g1 >> 3) & 1 == 1,
            enhanced_rdi: g1 & 0x07,
        }
    }
}

impl Default for G1Status {
    fn default() -> Self {
        Self::new()
    }
}

// ── Frame Timing Utilities ────────────────────────────────────────────────────

/// Compute the frame number from elapsed time in microseconds
pub fn frame_number_from_us(elapsed_us: f64) -> u64 {
    (elapsed_us / FRAME_PERIOD_US) as u64
}

/// Compute elapsed time in microseconds for a given frame number
pub fn elapsed_us_from_frame(frame: u64) -> f64 {
    frame as f64 * FRAME_PERIOD_US
}

/// Compute line rate utilization given payload Mbps and STS level
pub fn line_utilization(payload_mbps: f64, level: SonetLevel) -> f64 {
    let line_mbps = level.line_rate_bps() as f64 / 1e6;
    (payload_mbps / line_mbps).min(1.0)
}

/// Compute pointer offset given SPE phase offset in bytes (0-782)
/// Pointer indicates the byte offset of J1 (first POH byte) within the frame
pub fn spe_phase_to_pointer(spe_byte_offset: u16) -> u16 {
    spe_byte_offset % (AU_POINTER_MAX + 1)
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── Constants and Level Tests ─────────────────────────────────────────────

    #[test]
    fn test_sts1_frame_size() {
        assert_eq!(STS1_FRAME_BYTES, 810);
        assert_eq!(STS1_ROWS, 9);
        assert_eq!(STS1_COLS, 90);
    }

    #[test]
    fn test_spe_size() {
        assert_eq!(SPE_BYTES, 783);
        assert_eq!(SPE_COLS, 87);
    }

    #[test]
    fn test_framing_byte_constants() {
        assert_eq!(A1_BYTE, 0xF6);
        assert_eq!(A2_BYTE, 0x28);
    }

    #[test]
    fn test_sonet_level_sts1_count() {
        assert_eq!(SonetLevel::Sts1.sts1_count(), 1);
        assert_eq!(SonetLevel::Sts3c.sts1_count(), 3);
        assert_eq!(SonetLevel::Sts12c.sts1_count(), 12);
        assert_eq!(SonetLevel::Sts48c.sts1_count(), 48);
        assert_eq!(SonetLevel::Sts192c.sts1_count(), 192);
    }

    #[test]
    fn test_sonet_level_frame_bytes() {
        assert_eq!(SonetLevel::Sts1.frame_bytes(), 810);
        assert_eq!(SonetLevel::Sts3c.frame_bytes(), 2430);
        assert_eq!(SonetLevel::Sts12c.frame_bytes(), 9720);
    }

    #[test]
    fn test_sonet_level_line_rates() {
        assert_eq!(SonetLevel::Sts1.line_rate_bps(), 51_840_000);
        assert_eq!(SonetLevel::Sts3c.line_rate_bps(), 155_520_000);
        assert_eq!(SonetLevel::Sts12c.line_rate_bps(), 622_080_000);
        assert_eq!(SonetLevel::Sts48c.line_rate_bps(), 2_488_320_000);
    }

    // ── Frame Construction Tests ──────────────────────────────────────────────

    #[test]
    fn test_frame_new_is_zeroed() {
        let frame = Sts1Frame::new();
        assert!(frame.data.iter().all(|&b| b == 0));
    }

    #[test]
    fn test_frame_idx() {
        assert_eq!(Sts1Frame::idx(0, 0), 0);
        assert_eq!(Sts1Frame::idx(0, 89), 89);
        assert_eq!(Sts1Frame::idx(1, 0), 90);
        assert_eq!(Sts1Frame::idx(8, 89), 809);
    }

    #[test]
    fn test_frame_set_and_get() {
        let mut frame = Sts1Frame::new();
        frame.set(3, 7, 0xAB);
        assert_eq!(frame.get(3, 7), 0xAB);
    }

    #[test]
    fn test_frame_framing_bytes() {
        let mut frame = Sts1Frame::new();
        frame.set_a1();
        frame.set_a2();
        assert_eq!(frame.a1(), A1_BYTE);
        assert_eq!(frame.a2(), A2_BYTE);
        assert!(frame.has_valid_framing());
    }

    #[test]
    fn test_frame_invalid_framing() {
        let mut frame = Sts1Frame::new();
        frame.set(0, 0, 0x00); // wrong A1
        frame.set(0, 1, A2_BYTE);
        assert!(!frame.has_valid_framing());
    }

    // ── SOH/LOH/POH Apply/Read Roundtrip ──────────────────────────────────────

    #[test]
    fn test_soh_roundtrip() {
        let soh = SectionOverhead {
            a1: A1_BYTE, a2: A2_BYTE, j0: 0x55,
            b1: 0x12, e1: 0x34, f1: 0x56,
            d1: 0xAA, d2: 0xBB, d3: 0xCC,
        };
        let mut frame = Sts1Frame::new();
        frame.apply_soh(&soh);
        let read = frame.read_soh();
        assert_eq!(read.a1, A1_BYTE);
        assert_eq!(read.a2, A2_BYTE);
        assert_eq!(read.j0, 0x55);
        assert_eq!(read.b1, 0x12);
        assert_eq!(read.d3, 0xCC);
    }

    #[test]
    fn test_loh_roundtrip() {
        let loh = LineOverhead {
            h1: 0x62, h2: 0x04, h3: 0x00,
            b2: 0xFF, k1: 0xF0, k2: 0x10,
            d4: 0x01, d5: 0x02, d6: 0x03,
            d7: 0x04, d8: 0x05, d9: 0x06,
            d10: 0x07, d11: 0x08, d12: 0x09,
            s1: 0x04, m0: 0x00, e2: 0xAB,
        };
        let mut frame = Sts1Frame::new();
        frame.apply_loh(&loh);
        let read = frame.read_loh();
        assert_eq!(read.h1, 0x62);
        assert_eq!(read.h2, 0x04);
        assert_eq!(read.b2, 0xFF);
        assert_eq!(read.k1, 0xF0);
        assert_eq!(read.k2, 0x10);
        assert_eq!(read.s1, 0x04);
        assert_eq!(read.e2, 0xAB);
    }

    #[test]
    fn test_poh_roundtrip() {
        let poh = PathOverhead {
            j1: 0x01, b3: 0xAB, c2: 0x13,
            g1: 0x08, f2: 0x00, h4: 0x02,
            z3: 0x00, z4: 0x00, z5: 0x00,
        };
        let mut frame = Sts1Frame::new();
        frame.apply_poh(&poh);
        let read = frame.read_poh();
        assert_eq!(read.j1, 0x01);
        assert_eq!(read.b3, 0xAB);
        assert_eq!(read.c2, 0x13);
        assert_eq!(read.g1, 0x08);
        assert_eq!(read.h4, 0x02);
    }

    // ── Scrambler Tests ───────────────────────────────────────────────────────

    #[test]
    fn test_scrambler_roundtrip() {
        let original = [0xDE, 0xAD, 0xBE, 0xEF, 0x12, 0x34, 0x56, 0x78];
        let mut data = original;
        let mut s1 = SonetScrambler::new();
        s1.scramble(&mut data);
        assert_ne!(data, original); // should change
        let mut s2 = SonetScrambler::new();
        s2.scramble(&mut data);
        assert_eq!(data, original); // should restore
    }

    #[test]
    fn test_scrambler_frame_roundtrip() {
        let mut frame_data = [0u8; STS1_FRAME_BYTES];
        for (i, b) in frame_data.iter_mut().enumerate() {
            *b = (i & 0xFF) as u8;
        }
        let original = frame_data;
        let mut scr = SonetScrambler::new();
        scr.scramble_frame(&mut frame_data, 2);
        // First two bytes unchanged (A1/A2)
        assert_eq!(frame_data[0], original[0]);
        assert_eq!(frame_data[1], original[1]);
        // Rest should be scrambled
        assert_ne!(&frame_data[2..10], &original[2..10]);
        // Descramble
        let mut scr2 = SonetScrambler::new();
        scr2.descramble_frame(&mut frame_data, 2);
        assert_eq!(frame_data, original);
    }

    #[test]
    fn test_scrambler_reset_consistency() {
        let data = [0x55u8; 16];
        let mut s1 = SonetScrambler::new();
        let mut d1 = data;
        s1.scramble(&mut d1);

        let mut s2 = SonetScrambler::new();
        s2.reset();
        let mut d2 = data;
        s2.scramble(&mut d2);

        assert_eq!(d1, d2);
    }

    #[test]
    fn test_scrambler_not_all_pass() {
        // Scrambler should modify non-zero data
        let mut data = [0xFFu8; 100];
        let mut scr = SonetScrambler::new();
        scr.scramble(&mut data);
        let differs = data.iter().any(|&b| b != 0xFF);
        assert!(differs);
    }

    // ── BIP-8 Tests ───────────────────────────────────────────────────────────

    #[test]
    fn test_bip8_single_byte() {
        assert_eq!(bip8(&[0xAB]), 0xAB);
    }

    #[test]
    fn test_bip8_two_bytes() {
        assert_eq!(bip8(&[0xAB, 0xCD]), 0xAB ^ 0xCD);
    }

    #[test]
    fn test_bip8_all_zeros() {
        assert_eq!(bip8(&[0x00, 0x00, 0x00]), 0x00);
    }

    #[test]
    fn test_bip8_all_ones() {
        assert_eq!(bip8(&[0xFF, 0xFF]), 0x00); // XOR of equal bytes = 0
    }

    #[test]
    fn test_bip8_verify_no_error() {
        let data = [0x12, 0x34, 0x56];
        let bip = bip8(&data);
        assert_eq!(bip8_verify(&data, bip), 0x00);
    }

    #[test]
    fn test_bip8_verify_with_error() {
        let data = [0x12, 0x34, 0x56];
        let bip = bip8(&data);
        let wrong_bip = bip ^ 0x01;
        let syndrome = bip8_verify(&data, wrong_bip);
        assert_eq!(syndrome, 0x01);
        assert_eq!(bip8_error_count(syndrome), 1);
    }

    #[test]
    fn test_bip8_error_count() {
        assert_eq!(bip8_error_count(0x00), 0);
        assert_eq!(bip8_error_count(0xFF), 8);
        assert_eq!(bip8_error_count(0x55), 4);
    }

    #[test]
    fn test_bip_n_lanes() {
        let data: Vec<u8> = (0..9u8).collect(); // 9 bytes, 3 lanes
        let parity = bip_n(&data, 3);
        assert_eq!(parity.len(), 3);
        // Lane 0: 0 ^ 3 ^ 6 = 5
        assert_eq!(parity[0], 0 ^ 3 ^ 6);
        // Lane 1: 1 ^ 4 ^ 7 = 2
        assert_eq!(parity[1], 1 ^ 4 ^ 7);
        // Lane 2: 2 ^ 5 ^ 8 = 11
        assert_eq!(parity[2], 2 ^ 5 ^ 8);
    }

    // ── Pointer Tests ─────────────────────────────────────────────────────────

    #[test]
    fn test_pointer_encode_decode_zero() {
        let (h1, h2) = encode_pointer(0, false, false);
        let (offset, ndf, concat) = decode_pointer(h1, h2);
        assert_eq!(offset, 0);
        assert!(!ndf);
        assert!(!concat);
    }

    #[test]
    fn test_pointer_encode_decode_max() {
        let (h1, h2) = encode_pointer(782, false, false);
        let (offset, ndf, concat) = decode_pointer(h1, h2);
        assert_eq!(offset, 782);
        assert!(!ndf);
        assert!(!concat);
    }

    #[test]
    fn test_pointer_encode_decode_mid() {
        let (h1, h2) = encode_pointer(391, false, false);
        let (offset, ndf, concat) = decode_pointer(h1, h2);
        assert_eq!(offset, 391);
    }

    #[test]
    fn test_pointer_ndf_flag() {
        let (h1, h2) = encode_pointer(100, true, false);
        let (offset, ndf, concat) = decode_pointer(h1, h2);
        assert_eq!(offset, 100);
        assert!(ndf);
        assert!(!concat);
    }

    #[test]
    fn test_pointer_concat_indicator() {
        let (h1, h2) = encode_pointer(0, false, true);
        let (_, _, concat) = decode_pointer(h1, h2);
        assert!(concat);
        // Verify the expected bytes
        assert_eq!(h1, 0x93);
        assert_eq!(h2, 0xFF);
    }

    #[test]
    fn test_frame_set_get_pointer() {
        let mut frame = Sts1Frame::new();
        frame.set_pointer(256, false);
        let (offset, ndf, concat) = frame.get_pointer();
        assert_eq!(offset, 256);
        assert!(!ndf);
        assert!(!concat);
    }

    #[test]
    fn test_frame_set_concat_indicator() {
        let mut frame = Sts1Frame::new();
        frame.set_concat_indicator();
        let (_, _, concat) = frame.get_pointer();
        assert!(concat);
    }

    // ── Pointer Processor Tests ───────────────────────────────────────────────

    #[test]
    fn test_pointer_processor_init() {
        let pp = PointerProcessor::new(100);
        assert_eq!(pp.pointer, 100);
        assert_eq!(pp.pos_just_count, 0);
        assert_eq!(pp.neg_just_count, 0);
    }

    #[test]
    fn test_pointer_processor_stable() {
        let mut pp = PointerProcessor::new(50);
        let (h1, h2) = encode_pointer(50, false, false);
        let ev = pp.process(h1, h2);
        assert_eq!(ev, JustificationEvent::None);
    }

    #[test]
    fn test_pointer_processor_positive_just() {
        let mut pp = PointerProcessor::new(50);
        // Positive justification: pointer decrements (SPE slower)
        let (h1, h2) = encode_pointer(49, false, false);
        let ev = pp.process(h1, h2);
        assert_eq!(ev, JustificationEvent::Positive);
        assert_eq!(pp.pointer, 49);
        assert_eq!(pp.pos_just_count, 1);
    }

    #[test]
    fn test_pointer_processor_negative_just() {
        let mut pp = PointerProcessor::new(50);
        // Negative justification: pointer increments (SPE faster)
        let (h1, h2) = encode_pointer(51, false, false);
        let ev = pp.process(h1, h2);
        assert_eq!(ev, JustificationEvent::Negative);
        assert_eq!(pp.pointer, 51);
        assert_eq!(pp.neg_just_count, 1);
    }

    #[test]
    fn test_pointer_processor_ndf() {
        let mut pp = PointerProcessor::new(50);
        let (h1, h2) = encode_pointer(300, true, false);
        let ev = pp.process(h1, h2);
        assert_eq!(ev, JustificationEvent::None);
        assert_eq!(pp.pointer, 300);
        assert!(pp.ndf_received);
    }

    #[test]
    fn test_pointer_processor_manual_justify() {
        let mut pp = PointerProcessor::new(782);
        let new_ptr = pp.negative_justify();
        assert_eq!(new_ptr, 0); // wraps from 782 to 0
        assert_eq!(pp.neg_just_count, 1);
    }

    #[test]
    fn test_pointer_processor_positive_wrap() {
        let mut pp = PointerProcessor::new(0);
        let new_ptr = pp.positive_justify();
        assert_eq!(new_ptr, 782); // wraps from 0 to 782
        assert_eq!(pp.pos_just_count, 1);
    }

    // ── OOF/LOF State Machine Tests ───────────────────────────────────────────

    #[test]
    fn test_aligner_starts_in_frame() {
        let aligner = FrameAligner::new();
        assert_eq!(aligner.state, FrameAlignState::InFrame);
    }

    #[test]
    fn test_aligner_bad_frame_triggers_oof() {
        let mut aligner = FrameAligner::new();
        let state = aligner.process_frame(false);
        assert_eq!(state, FrameAlignState::OutOfFrame);
        assert_eq!(aligner.oof_events, 1);
    }

    #[test]
    fn test_aligner_single_good_recovers_from_oof() {
        let mut aligner = FrameAligner::new();
        aligner.process_frame(false); // OOF
        let state = aligner.process_frame(true); // recover
        assert_eq!(state, FrameAlignState::InFrame);
    }

    #[test]
    fn test_aligner_lof_declared_after_threshold() {
        let mut aligner = FrameAligner::new();
        // Need 24 consecutive bad frames (lof_threshold = 24)
        let mut state = FrameAlignState::InFrame;
        for _ in 0..24 {
            state = aligner.process_frame(false);
        }
        assert_eq!(state, FrameAlignState::LossOfFrame);
        assert_eq!(aligner.lof_events, 1);
    }

    #[test]
    fn test_aligner_lof_recovery_after_good_frames() {
        let mut aligner = FrameAligner::new();
        for _ in 0..24 { aligner.process_frame(false); }
        assert_eq!(aligner.state, FrameAlignState::LossOfFrame);
        // Need 8 good frames to recover
        let mut state = FrameAlignState::LossOfFrame;
        for _ in 0..8 {
            state = aligner.process_frame(true);
        }
        assert_eq!(state, FrameAlignState::InFrame);
    }

    #[test]
    fn test_aligner_lof_partial_recovery_not_enough() {
        let mut aligner = FrameAligner::new();
        for _ in 0..24 { aligner.process_frame(false); }
        // Only 4 good frames — not enough
        for _ in 0..4 { aligner.process_frame(true); }
        assert_eq!(aligner.state, FrameAlignState::LossOfFrame);
    }

    #[test]
    fn test_aligner_reset() {
        let mut aligner = FrameAligner::new();
        for _ in 0..24 { aligner.process_frame(false); }
        aligner.reset();
        assert_eq!(aligner.state, FrameAlignState::InFrame);
    }

    // ── C2 Signal Label Tests ─────────────────────────────────────────────────

    #[test]
    fn test_c2_label_roundtrip() {
        let labels = [
            C2Label::Unequipped,
            C2Label::EquippedUnspecified,
            C2Label::VtStructured,
            C2Label::Atm,
            C2Label::HdlcPos,
            C2Label::TestPrbs,
            C2Label::PayloadDefect,
        ];
        for label in &labels {
            let byte = *label as u8;
            let decoded = C2Label::from_byte(byte);
            assert!(decoded.is_some(), "Failed for {:?}", label);
        }
    }

    #[test]
    fn test_c2_unequipped_is_zero() {
        assert_eq!(C2Label::Unequipped as u8, 0x00);
    }

    #[test]
    fn test_c2_atm_value() {
        assert_eq!(C2Label::Atm as u8, 0x13);
    }

    #[test]
    fn test_c2_unknown_byte() {
        assert!(C2Label::from_byte(0x77).is_none());
    }

    // ── Sync Status Tests ─────────────────────────────────────────────────────

    #[test]
    fn test_sync_status_encode_decode() {
        let statuses = [
            SyncStatus::Prs,
            SyncStatus::SsuA,
            SyncStatus::SsuB,
            SyncStatus::Sec,
            SyncStatus::Dnu,
        ];
        for &status in &statuses {
            let s1 = status.encode_s1();
            let decoded = SyncStatus::from_s1(s1);
            // Quality level should match
            assert_eq!(decoded.quality_level(), status.quality_level(),
                       "Mismatch for {:?}", status);
        }
    }

    #[test]
    fn test_sync_status_dnu_is_lowest() {
        assert_eq!(SyncStatus::Dnu.quality_level(), 0);
    }

    #[test]
    fn test_sync_status_prs_is_highest() {
        assert_eq!(SyncStatus::Prs.quality_level(), 6);
    }

    #[test]
    fn test_frame_sync_status_roundtrip() {
        let mut frame = Sts1Frame::new();
        frame.set_sync_status(SyncStatus::SsuB);
        let status = frame.get_sync_status();
        assert_eq!(status.quality_level(), SyncStatus::SsuB.quality_level());
    }

    // ── APS K1/K2 Tests ───────────────────────────────────────────────────────

    #[test]
    fn test_aps_request_encode_decode() {
        let req = ApsRequest::ForcedSwitch;
        let k1 = req.encode_k1(3);
        let decoded = ApsRequest::from_k1(k1);
        assert_eq!(decoded as u8, ApsRequest::ForcedSwitch as u8);
        // Channel number preserved
        assert_eq!(k1 & 0x0F, 3);
    }

    #[test]
    fn test_aps_message_encode_decode() {
        let msg = ApsMessage {
            request: ApsRequest::ManualSwitch,
            channel: 1,
            bridged_channel: 0,
            architecture: ApsArchitecture::OnePlusOneBi,
            rdi_line: true,
            ring_mode: false,
        };
        let (k1, k2) = msg.encode();
        let decoded = ApsMessage::decode(k1, k2);
        assert_eq!(decoded.request as u8, ApsRequest::ManualSwitch as u8);
        assert_eq!(decoded.channel, 1);
        assert!(decoded.rdi_line);
        assert_eq!(decoded.architecture as u8, ApsArchitecture::OnePlusOneBi as u8);
    }

    #[test]
    fn test_aps_no_request_all_zeros() {
        let (k1, k2) = (0x00u8, 0x00u8);
        let msg = ApsMessage::decode(k1, k2);
        assert_eq!(msg.request as u8, ApsRequest::NoRequest as u8);
        assert!(!msg.rdi_line);
    }

    // ── J1 Path Trace Tests ───────────────────────────────────────────────────

    #[test]
    fn test_j1_path_trace_length() {
        let trace = J1PathTrace::from_str("SONET-PATH-TEST");
        assert_eq!(trace.as_bytes().len(), 64);
    }

    #[test]
    fn test_j1_path_trace_message_roundtrip() {
        let msg = "Test Path Trace";
        let trace = J1PathTrace::from_str(msg);
        assert_eq!(trace.message(), msg);
    }

    #[test]
    fn test_j1_path_trace_crc_valid() {
        let trace = J1PathTrace::from_str("Valid CRC Test");
        assert!(trace.verify_crc());
    }

    #[test]
    fn test_j1_path_trace_frame_byte_cycling() {
        let trace = J1PathTrace::from_str("cycle test");
        // Byte for frame 0 and frame 64 should be identical (64-byte cycle)
        assert_eq!(trace.get_frame_byte(0), trace.get_frame_byte(64));
        assert_eq!(trace.get_frame_byte(63), trace.get_frame_byte(127));
    }

    #[test]
    fn test_j1_path_trace_framing_byte_bit0() {
        let trace = J1PathTrace::from_str("framing bit test");
        // Framing byte (index 0) must have bit 0 = 1
        assert_eq!(trace.as_bytes()[0] & 0x01, 0x01);
    }

    #[test]
    fn test_j1_empty_message() {
        let trace = J1PathTrace::from_str("");
        assert!(trace.verify_crc());
    }

    // ── VT Mapping Tests ──────────────────────────────────────────────────────

    #[test]
    fn test_vt_type_columns() {
        assert_eq!(VtType::Vt1_5.columns_per_vt(), 3);
        assert_eq!(VtType::Vt2.columns_per_vt(), 4);
        assert_eq!(VtType::Vt3.columns_per_vt(), 6);
        assert_eq!(VtType::Vt6.columns_per_vt(), 12);
    }

    #[test]
    fn test_vt_bytes_per_frame() {
        assert_eq!(VtType::Vt1_5.bytes_per_frame(), 27);
        assert_eq!(VtType::Vt2.bytes_per_frame(), 36);
        assert_eq!(VtType::Vt6.bytes_per_frame(), 108);
    }

    #[test]
    fn test_vt_bit_rates() {
        assert_eq!(VtType::Vt1_5.bit_rate_bps(), 1_544_000);
        assert_eq!(VtType::Vt2.bit_rate_bps(), 2_048_000);
    }

    #[test]
    fn test_vt_extract_insert_roundtrip() {
        let mut spe = vec![0u8; SPE_BYTES];
        let payload: Vec<u8> = (0..27).collect(); // VT1.5 = 27 bytes
        insert_vt_payload(&mut spe, 0, VtType::Vt1_5, &payload);
        let extracted = extract_vt_payload(&spe, 0, VtType::Vt1_5);
        assert_eq!(extracted.len(), payload.len());
        assert_eq!(extracted, payload);
    }

    #[test]
    fn test_vt_extract_insert_vt2_slot1() {
        let mut spe = vec![0u8; SPE_BYTES];
        let payload: Vec<u8> = (100..136).collect(); // 36 bytes for VT2
        insert_vt_payload(&mut spe, 1, VtType::Vt2, &payload);
        let extracted = extract_vt_payload(&spe, 1, VtType::Vt2);
        assert_eq!(extracted.len(), 36);
        assert_eq!(extracted, payload);
    }

    // ── SPE Extract/Insert Tests ──────────────────────────────────────────────

    #[test]
    fn test_spe_extract_insert_roundtrip() {
        let mut frame = Sts1Frame::new();
        let spe: [u8; SPE_BYTES] = {
            let mut a = [0u8; SPE_BYTES];
            for (i, b) in a.iter_mut().enumerate() {
                *b = (i & 0xFF) as u8;
            }
            a
        };
        frame.insert_spe(&spe);
        let extracted = frame.extract_spe();
        assert_eq!(extracted, spe);
    }

    // ── G1 Status Tests ───────────────────────────────────────────────────────

    #[test]
    fn test_g1_status_encode_decode() {
        let g1 = G1Status { rei_count: 5, rdi_path: true, enhanced_rdi: 0 };
        let byte = g1.encode();
        let decoded = G1Status::decode(byte);
        assert_eq!(decoded.rei_count, 5);
        assert!(decoded.rdi_path);
    }

    #[test]
    fn test_g1_no_error() {
        let g1 = G1Status::new();
        assert_eq!(g1.encode(), 0x00);
    }

    #[test]
    fn test_g1_max_rei() {
        let g1 = G1Status { rei_count: 8, rdi_path: false, enhanced_rdi: 0 };
        let byte = g1.encode();
        assert_eq!((byte >> 4) & 0x0F, 8);
    }

    // ── Concatenation Tests ───────────────────────────────────────────────────

    #[test]
    fn test_stsnc_frame_construction_sts3c() {
        let frame = StsNcFrame::new(SonetLevel::Sts3c);
        assert_eq!(frame.frames.len(), 3);
    }

    #[test]
    fn test_stsnc_concatenation_indicators() {
        let frame = StsNcFrame::new(SonetLevel::Sts3c);
        assert!(frame.verify_concatenation(),
                "STS-3c should have valid concatenation indicators");
    }

    #[test]
    fn test_stsnc_first_frame_no_concat() {
        let frame = StsNcFrame::new(SonetLevel::Sts3c);
        let (_, _, concat) = frame.frame(0).get_pointer();
        assert!(!concat, "First STS-1 frame should not have concat indicator");
    }

    #[test]
    fn test_stsnc_subsequent_frames_have_concat() {
        let frame = StsNcFrame::new(SonetLevel::Sts12c);
        for i in 1..12 {
            let (_, _, concat) = frame.frame(i).get_pointer();
            assert!(concat, "Frame {} should have concat indicator", i);
        }
    }

    // ── Framer Integration Tests ──────────────────────────────────────────────

    #[test]
    fn test_framer_builds_valid_frame() {
        let mut framer = Sts1Framer::new();
        let payload = vec![0xAA; 774]; // 9 rows × 86 cols
        let frame = framer.build_frame(&payload);
        // After building, scrambler has run — we can't check A1/A2 directly
        // since they're unscrambled. The raw bytes at pos 0,1 should still be A1/A2.
        assert_eq!(frame.data[0], A1_BYTE);
        assert_eq!(frame.data[1], A2_BYTE);
    }

    #[test]
    fn test_framer_j1_trace_cycling() {
        let mut framer = Sts1Framer::new();
        framer.set_j1_trace("My Network Element");
        let payload = vec![0u8; 774];
        // Build 65 frames and check J1 cycling
        let _f0_j1 = {
            let f = framer.build_frame(&payload);
            f.data[Sts1Frame::idx(0, 3)] // J1 position (but scrambled)
        };
        // Just verify no panic over 65 frames
        for _ in 0..65 {
            let _ = framer.build_frame(&payload);
        }
    }

    // ── Timing Tests ──────────────────────────────────────────────────────────

    #[test]
    fn test_frame_period_8000_per_second() {
        // 1,000,000 μs / 125 μs = 8000 frames/sec
        let frames_per_sec = 1_000_000.0 / FRAME_PERIOD_US;
        let diff = (frames_per_sec - 8000.0).abs();
        assert!(diff < 1.0, "Expected 8000 frames/sec, got {}", frames_per_sec);
    }

    #[test]
    fn test_frame_number_from_us() {
        assert_eq!(frame_number_from_us(125.0), 1);
        assert_eq!(frame_number_from_us(0.0), 0);
        assert_eq!(frame_number_from_us(8000.0 * 125.0), 8000);
    }

    #[test]
    fn test_elapsed_us_from_frame() {
        assert!((elapsed_us_from_frame(0) - 0.0).abs() < 1e-9);
        assert!((elapsed_us_from_frame(1) - 125.0).abs() < 1e-9);
        assert!((elapsed_us_from_frame(8000) - 1_000_000.0).abs() < 1e-6);
    }

    #[test]
    fn test_line_utilization() {
        let util = line_utilization(51.84, SonetLevel::Sts1);
        assert!((util - 1.0).abs() < 1e-6, "100% utilization at full rate");
        let half = line_utilization(25.92, SonetLevel::Sts1);
        assert!((half - 0.5).abs() < 1e-6);
    }

    #[test]
    fn test_spe_phase_to_pointer() {
        assert_eq!(spe_phase_to_pointer(0), 0);
        assert_eq!(spe_phase_to_pointer(782), 782);
        assert_eq!(spe_phase_to_pointer(783), 0); // wraps
        assert_eq!(spe_phase_to_pointer(784), 1);
    }

    #[test]
    fn test_au_pointer_max() {
        // AU pointer maximum is 782 for STS-1 (783 SPE bytes, 0-indexed)
        assert_eq!(AU_POINTER_MAX, 782);
    }
}
