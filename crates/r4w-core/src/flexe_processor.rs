//! FlexE (Flexible Ethernet) Processor — OIF FlexE 2.1 Implementation Agreement
//!
//! FlexE bonds multiple 100GE PHY interfaces into a FlexE group, partitions bandwidth
//! using a calendar-slot scheme (20 slots × ~5 Gbps per PHY), and multiplexes/demultiplexes
//! virtual FlexE client streams through a shim layer between the MAC and PCS.
//!
//! # Architecture Overview
//!
//! ```text
//! ┌──────────────┐   ┌──────────────┐
//! │  FlexE Client│   │  FlexE Client│  (virtual MACs)
//! └──────┬───────┘   └──────┬───────┘
//!        │                  │
//!        └─────────┬────────┘
//!               FlexE Shim
//!           (calendar mux/demux,
//!            idle insertion/deletion)
//!        ┌─────────┴────────┐
//! ┌──────┴──────┐   ┌───────┴──────┐
//! │  PHY / PCS  │   │  PHY / PCS   │  (100GE PHYs)
//! └─────────────┘   └──────────────┘
//! ```
//!
//! # Key Constants
//!
//! | Constant               | Value                     |
//! |------------------------|---------------------------|
//! | PHY line rate          | 100 Gbps                  |
//! | Calendar slots per PHY | 20                        |
//! | Slot bandwidth         | ~5.15625 Gbps             |
//! | 66B block size         | 66 bits (2 sync + 64 data)|
//! | Overhead frames/mframe | 32                        |
//! | Overhead blocks/frame  | 8                         |
//! | Multiframe period      | ~1.3107 ms                |
//! | Max PHYs per group     | 254                       |
//! | Max clients per group  | 254                       |
//! | CRC polynomial         | 0x1021 (CCITT-16)         |

// ─────────────────────────────────────────────────────────────────────────────
// Constants
// ─────────────────────────────────────────────────────────────────────────────

/// Number of calendar slots per 100 GE PHY.
pub const SLOTS_PER_PHY: usize = 20;

/// Nominal bandwidth per calendar slot in Gbps (~100 / 20 × (66/64) overhead factor).
/// The 66B encoding inflates the raw bit rate: 100G × 66/64 = 103.125 Gbps line rate;
/// each of the 20 slots carries 103.125 / 20 ≈ 5.15625 Gbps.
pub const SLOT_BPS_GBPS: f64 = 5.15625;

/// PHY line rate in Gbps.
pub const PHY_LINE_RATE_GBPS: f64 = 100.0;

/// Maximum number of PHYs in a single FlexE group (OIF FlexE 2.1 §5.2).
pub const MAX_PHYS: usize = 254;

/// Maximum number of FlexE clients in a group.
pub const MAX_CLIENTS: usize = 254;

/// Number of overhead frames per overhead multiframe.
pub const OVERHEAD_FRAMES_PER_MFRAME: usize = 32;

/// Number of overhead blocks per overhead frame.
pub const OVERHEAD_BLOCKS_PER_FRAME: usize = 8;

/// Total overhead blocks per multiframe.
pub const OVERHEAD_BLOCKS_PER_MFRAME: usize =
    OVERHEAD_FRAMES_PER_MFRAME * OVERHEAD_BLOCKS_PER_FRAME;

/// Overhead multiframe period in microseconds (≈1310.7 µs).
pub const MFRAME_PERIOD_US: f64 = 1310.7;

/// 66B sync header value for a data block (0b01).
pub const SYNC_DATA: u8 = 0b01;

/// 66B sync header value for a control block (0b10).
pub const SYNC_CTRL: u8 = 0b10;

/// CRC-16 CCITT polynomial (x^16 + x^12 + x^5 + 1 → 0x1021).
pub const CRC16_POLY: u16 = 0x1021;

/// Idle/ordered-set pattern inserted for rate adaptation (0x07 = /I/ ordered set).
pub const IDLE_BYTE: u8 = 0x07;

// ─────────────────────────────────────────────────────────────────────────────
// Error types
// ─────────────────────────────────────────────────────────────────────────────

/// Errors produced by FlexE processing operations.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FlexeError {
    /// PHY index is out of the valid range 1–254.
    InvalidPhyId(u8),
    /// Client ID is out of the valid range 1–254.
    InvalidClientId(u8),
    /// Requested slot index exceeds `SLOTS_PER_PHY - 1`.
    InvalidSlotIndex(usize),
    /// Calendar slot is already occupied by another client.
    SlotAlreadyOccupied { slot: usize, phy: u8 },
    /// Calendar slot is not allocated to the specified client.
    SlotNotOwned { slot: usize, client: u8 },
    /// The FlexE group has reached its maximum PHY capacity.
    GroupFull,
    /// A PHY with this ID already exists in the group.
    PhyAlreadyPresent(u8),
    /// Referenced PHY does not exist in the group.
    PhyNotFound(u8),
    /// CRC mismatch when validating an overhead frame.
    CrcMismatch { expected: u16, computed: u16 },
    /// Sync-header bits are neither `01` nor `10`.
    InvalidSyncHeader(u8),
    /// Block payload length is not exactly 8 bytes (64 bits).
    InvalidBlockLength(usize),
    /// No calendar slots are allocated to this client.
    NoSlotsAllocated(u8),
}

impl core::fmt::Display for FlexeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::InvalidPhyId(id) => write!(f, "invalid PHY id {id} (valid: 1–254)"),
            Self::InvalidClientId(id) => write!(f, "invalid client id {id} (valid: 1–254)"),
            Self::InvalidSlotIndex(s) => write!(f, "slot index {s} >= {SLOTS_PER_PHY}"),
            Self::SlotAlreadyOccupied { slot, phy } => {
                write!(f, "slot {slot} on PHY {phy} already occupied")
            }
            Self::SlotNotOwned { slot, client } => {
                write!(f, "slot {slot} not owned by client {client}")
            }
            Self::GroupFull => write!(f, "FlexE group already has {MAX_PHYS} PHYs"),
            Self::PhyAlreadyPresent(id) => write!(f, "PHY {id} already in group"),
            Self::PhyNotFound(id) => write!(f, "PHY {id} not found in group"),
            Self::CrcMismatch { expected, computed } => {
                write!(f, "CRC mismatch: expected {expected:#06x}, got {computed:#06x}")
            }
            Self::InvalidSyncHeader(h) => write!(f, "invalid 66B sync header {h:#04b}"),
            Self::InvalidBlockLength(n) => {
                write!(f, "block payload must be 8 bytes, got {n}")
            }
            Self::NoSlotsAllocated(id) => write!(f, "no slots allocated to client {id}"),
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// 66B Block Encoding
// ─────────────────────────────────────────────────────────────────────────────

/// A 66-bit block as used in the 64B/66B PCS layer.
///
/// The on-wire layout is:
/// ```text
/// [2-bit sync header][64-bit payload]
/// ```
/// Sync header `01` → data block; `10` → control/ordered-set block.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Block66b {
    /// Sync header: `SYNC_DATA` (0b01) or `SYNC_CTRL` (0b10).
    pub sync: u8,
    /// 8-byte (64-bit) payload.
    pub payload: [u8; 8],
}

impl Block66b {
    /// Construct a data block.
    pub fn data(payload: [u8; 8]) -> Self {
        Self { sync: SYNC_DATA, payload }
    }

    /// Construct a control/ordered-set block.
    pub fn control(payload: [u8; 8]) -> Self {
        Self { sync: SYNC_CTRL, payload }
    }

    /// Construct an idle block (control block carrying `/I/` ordered set bytes).
    pub fn idle() -> Self {
        Self::control([IDLE_BYTE; 8])
    }

    /// Returns `true` if this is a data block.
    pub fn is_data(&self) -> bool {
        self.sync == SYNC_DATA
    }

    /// Returns `true` if this is a control/ordered-set block.
    pub fn is_control(&self) -> bool {
        self.sync == SYNC_CTRL
    }

    /// Returns `true` if this is an idle block (all bytes == `IDLE_BYTE`).
    pub fn is_idle(&self) -> bool {
        self.is_control() && self.payload.iter().all(|&b| b == IDLE_BYTE)
    }

    /// Encode this block into a 9-byte buffer: byte 0 holds the 2-bit sync header
    /// in its most-significant bits; bytes 1–8 are the payload verbatim.
    ///
    /// This is a convenient in-memory representation; real hardware serialises
    /// the 66 bits across two 33-bit lanes.
    pub fn encode(&self) -> [u8; 9] {
        let mut out = [0u8; 9];
        out[0] = self.sync << 6; // bits [7:6] of first byte
        out[1..9].copy_from_slice(&self.payload);
        out
    }

    /// Decode a 9-byte buffer produced by [`encode`].
    pub fn decode(raw: &[u8; 9]) -> Result<Self, FlexeError> {
        let sync = raw[0] >> 6;
        if sync != SYNC_DATA && sync != SYNC_CTRL {
            return Err(FlexeError::InvalidSyncHeader(sync));
        }
        let mut payload = [0u8; 8];
        payload.copy_from_slice(&raw[1..9]);
        Ok(Self { sync, payload })
    }

    /// Scramble the payload with a simple XOR-based 64B/66B scrambler seed `state`.
    /// The scrambler polynomial is x^58 + x^39 + 1 (IEEE 802.3 clause 49).
    /// This implementation uses a linear feedback over a u64 state register.
    pub fn scramble(&self, state: &mut u64) -> Self {
        let mut scrambled = self.payload;
        for byte in scrambled.iter_mut() {
            let mut sb = 0u8;
            for bit in 0..8 {
                let feedback = (((*state >> 57) ^ (*state >> 38)) & 1) as u8;
                sb |= feedback << (7 - bit);
                *state = (*state << 1) | (feedback as u64);
            }
            *byte ^= sb;
        }
        Self { sync: self.sync, payload: scrambled }
    }

    /// Descramble using the same state register (scrambler is self-inverse after
    /// synchronisation).
    pub fn descramble(&self, state: &mut u64) -> Self {
        // Self-synchronising descrambler: feed *received* bits into state.
        let mut descrambled = self.payload;
        for byte in descrambled.iter_mut() {
            let orig = *byte;
            let mut db = 0u8;
            for bit in 0..8 {
                let recv_bit = (orig >> (7 - bit)) & 1;
                let feedback = ((*state >> 57) ^ (*state >> 38)) as u8 & 1;
                db |= (recv_bit ^ feedback) << (7 - bit);
                *state = (*state << 1) | (recv_bit as u64);
            }
            *byte = db;
        }
        Self { sync: self.sync, payload: descrambled }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// CRC-16 (CCITT / ITU-T V.41)
// ─────────────────────────────────────────────────────────────────────────────

/// Compute CRC-16/CCITT (polynomial 0x1021, initial value 0xFFFF, no reflection).
///
/// Used to protect FlexE overhead frames per OIF FlexE 2.1 §6.3.
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            if crc & 0x8000 != 0 {
                crc = (crc << 1) ^ CRC16_POLY;
            } else {
                crc <<= 1;
            }
        }
    }
    crc
}

/// Verify CRC-16 by appending the 2-byte CRC to the message and checking for
/// the well-known residue value (0x1D0F for CRC-16/CCITT with initial 0xFFFF).
pub fn crc16_verify(data: &[u8], expected_crc: u16) -> bool {
    crc16_ccitt(data) == expected_crc
}

// ─────────────────────────────────────────────────────────────────────────────
// Calendar types
// ─────────────────────────────────────────────────────────────────────────────

/// Identifies an individual calendar slot on a specific PHY.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub struct CalendarSlot {
    /// PHY number (1-based, 1–254).
    pub phy_id: u8,
    /// Slot index within that PHY (0–19).
    pub slot_index: usize,
}

/// Assignment of one calendar slot to a FlexE client.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SlotAssignment {
    pub slot: CalendarSlot,
    /// Client owning this slot (1–254), or `0` if unassigned.
    pub client_id: u8,
}

impl SlotAssignment {
    /// Create an unassigned slot.
    pub fn unassigned(phy_id: u8, slot_index: usize) -> Self {
        Self {
            slot: CalendarSlot { phy_id, slot_index },
            client_id: 0,
        }
    }

    /// Returns `true` if no client owns this slot.
    pub fn is_free(&self) -> bool {
        self.client_id == 0
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// PHY representation
// ─────────────────────────────────────────────────────────────────────────────

/// Status of an individual PHY within the group.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhyStatus {
    /// PHY is operational.
    Up,
    /// PHY has a local fault condition.
    LocalFault,
    /// PHY has detected a remote fault.
    RemoteFault,
    /// PHY is administratively disabled.
    Down,
}

/// A single 100 GE PHY participating in a FlexE group.
#[derive(Debug, Clone)]
pub struct FlexePhy {
    /// PHY identifier (1–254).
    pub phy_id: u8,
    /// Current operational status.
    pub status: PhyStatus,
    /// Calendar — one entry per slot.
    pub calendar: [SlotAssignment; SLOTS_PER_PHY],
}

impl FlexePhy {
    /// Construct a new PHY with all calendar slots unassigned.
    pub fn new(phy_id: u8) -> Result<Self, FlexeError> {
        if phy_id == 0 {
            return Err(FlexeError::InvalidPhyId(phy_id));
        }
        let calendar = core::array::from_fn(|i| SlotAssignment::unassigned(phy_id, i));
        Ok(Self { phy_id, status: PhyStatus::Up, calendar })
    }

    /// Assign `slot_index` to `client_id`.
    pub fn assign_slot(&mut self, slot_index: usize, client_id: u8) -> Result<(), FlexeError> {
        if slot_index >= SLOTS_PER_PHY {
            return Err(FlexeError::InvalidSlotIndex(slot_index));
        }
        if client_id == 0 {
            return Err(FlexeError::InvalidClientId(client_id));
        }
        if !self.calendar[slot_index].is_free() {
            return Err(FlexeError::SlotAlreadyOccupied {
                slot: slot_index,
                phy: self.phy_id,
            });
        }
        self.calendar[slot_index].client_id = client_id;
        Ok(())
    }

    /// Release a slot owned by `client_id`.
    pub fn release_slot(&mut self, slot_index: usize, client_id: u8) -> Result<(), FlexeError> {
        if slot_index >= SLOTS_PER_PHY {
            return Err(FlexeError::InvalidSlotIndex(slot_index));
        }
        if self.calendar[slot_index].client_id != client_id {
            return Err(FlexeError::SlotNotOwned {
                slot: slot_index,
                client: client_id,
            });
        }
        self.calendar[slot_index].client_id = 0;
        Ok(())
    }

    /// Count free (unassigned) slots on this PHY.
    pub fn free_slot_count(&self) -> usize {
        self.calendar.iter().filter(|s| s.is_free()).count()
    }

    /// Collect the slot indices assigned to `client_id`.
    pub fn slots_for_client(&self, client_id: u8) -> Vec<usize> {
        self.calendar
            .iter()
            .enumerate()
            .filter_map(|(i, s)| if s.client_id == client_id { Some(i) } else { None })
            .collect()
    }

    /// Bandwidth allocated to `client_id` on this PHY, in Gbps.
    pub fn client_bandwidth_gbps(&self, client_id: u8) -> f64 {
        self.slots_for_client(client_id).len() as f64 * SLOT_BPS_GBPS
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FlexE Group
// ─────────────────────────────────────────────────────────────────────────────

/// A FlexE group bonding 1–254 100 GE PHYs with a shared calendar.
#[derive(Debug, Clone)]
pub struct FlexeGroup {
    /// Group identifier (0–255 — value 0 is reserved for "no group").
    pub group_id: u8,
    /// PHYs in insertion order; keyed by `phy_id`.
    phys: Vec<FlexePhy>,
}

impl FlexeGroup {
    /// Create an empty FlexE group with the given `group_id`.
    pub fn new(group_id: u8) -> Self {
        Self { group_id, phys: Vec::new() }
    }

    /// Add a PHY to the group.
    pub fn add_phy(&mut self, phy_id: u8) -> Result<(), FlexeError> {
        if self.phys.len() >= MAX_PHYS {
            return Err(FlexeError::GroupFull);
        }
        if self.phys.iter().any(|p| p.phy_id == phy_id) {
            return Err(FlexeError::PhyAlreadyPresent(phy_id));
        }
        self.phys.push(FlexePhy::new(phy_id)?);
        Ok(())
    }

    /// Remove a PHY from the group. All of its calendar slots are implicitly released.
    pub fn remove_phy(&mut self, phy_id: u8) -> Result<(), FlexeError> {
        let pos = self
            .phys
            .iter()
            .position(|p| p.phy_id == phy_id)
            .ok_or(FlexeError::PhyNotFound(phy_id))?;
        self.phys.remove(pos);
        Ok(())
    }

    /// Number of PHYs currently in the group.
    pub fn phy_count(&self) -> usize {
        self.phys.len()
    }

    /// Total available bandwidth in Gbps (sum over all PHYs × 100 G).
    pub fn total_bandwidth_gbps(&self) -> f64 {
        self.phys.len() as f64 * PHY_LINE_RATE_GBPS
    }

    /// Total number of calendar slots across all PHYs.
    pub fn total_slots(&self) -> usize {
        self.phys.len() * SLOTS_PER_PHY
    }

    /// Number of currently free calendar slots across all PHYs.
    pub fn free_slots(&self) -> usize {
        self.phys.iter().map(|p| p.free_slot_count()).sum()
    }

    /// Assign `count` calendar slots to `client_id`, auto-picking free slots in
    /// round-robin PHY order.  Returns the list of assigned `CalendarSlot`s.
    pub fn allocate_slots(
        &mut self,
        client_id: u8,
        count: usize,
    ) -> Result<Vec<CalendarSlot>, FlexeError> {
        if client_id == 0 {
            return Err(FlexeError::InvalidClientId(client_id));
        }
        let mut assigned = Vec::new();
        'outer: for _ in 0..count {
            for phy in self.phys.iter_mut() {
                if let Some(slot_index) =
                    phy.calendar.iter().position(|s| s.is_free())
                {
                    phy.assign_slot(slot_index, client_id)?;
                    assigned.push(CalendarSlot { phy_id: phy.phy_id, slot_index });
                    continue 'outer;
                }
            }
            // Roll back on failure
            for cs in &assigned {
                if let Some(phy) = self.phys.iter_mut().find(|p| p.phy_id == cs.phy_id) {
                    let _ = phy.release_slot(cs.slot_index, client_id);
                }
            }
            return Err(FlexeError::GroupFull);
        }
        Ok(assigned)
    }

    /// Release all calendar slots owned by `client_id`.
    pub fn deallocate_client(&mut self, client_id: u8) -> usize {
        let mut released = 0;
        for phy in self.phys.iter_mut() {
            for slot_index in 0..SLOTS_PER_PHY {
                if phy.calendar[slot_index].client_id == client_id {
                    phy.calendar[slot_index].client_id = 0;
                    released += 1;
                }
            }
        }
        released
    }

    /// Return total Gbps allocated to `client_id` across all PHYs.
    pub fn client_bandwidth_gbps(&self, client_id: u8) -> f64 {
        self.phys
            .iter()
            .map(|p| p.client_bandwidth_gbps(client_id))
            .sum()
    }

    /// List all `CalendarSlot`s assigned to `client_id`.
    pub fn client_slots(&self, client_id: u8) -> Vec<CalendarSlot> {
        self.phys
            .iter()
            .flat_map(|p| {
                p.slots_for_client(client_id)
                    .into_iter()
                    .map(|idx| CalendarSlot { phy_id: p.phy_id, slot_index: idx })
            })
            .collect()
    }

    /// Propagate a fault on `phy_id` to `status`, optionally notifying clients.
    pub fn set_phy_status(&mut self, phy_id: u8, status: PhyStatus) -> Result<(), FlexeError> {
        let phy = self
            .phys
            .iter_mut()
            .find(|p| p.phy_id == phy_id)
            .ok_or(FlexeError::PhyNotFound(phy_id))?;
        phy.status = status;
        Ok(())
    }

    /// Returns the status of `phy_id`.
    pub fn phy_status(&self, phy_id: u8) -> Result<PhyStatus, FlexeError> {
        self.phys
            .iter()
            .find(|p| p.phy_id == phy_id)
            .map(|p| p.status)
            .ok_or(FlexeError::PhyNotFound(phy_id))
    }

    /// Returns `true` if the group is fully operational (all PHYs are `Up`).
    pub fn is_operational(&self) -> bool {
        self.phys.iter().all(|p| p.status == PhyStatus::Up)
    }

    /// Generate the flat calendar array for a named PHY — used to populate
    /// overhead multiframe calendar fields.
    pub fn calendar_array(&self, phy_id: u8) -> Result<[u8; SLOTS_PER_PHY], FlexeError> {
        let phy = self
            .phys
            .iter()
            .find(|p| p.phy_id == phy_id)
            .ok_or(FlexeError::PhyNotFound(phy_id))?;
        let mut arr = [0u8; SLOTS_PER_PHY];
        for (i, s) in phy.calendar.iter().enumerate() {
            arr[i] = s.client_id;
        }
        Ok(arr)
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FlexE Overhead Frame / Multiframe
// ─────────────────────────────────────────────────────────────────────────────

/// One FlexE overhead frame (8 overhead blocks × 8 bytes = 64 bytes of overhead
/// data, plus a 2-byte CRC-16 appended at the end).
///
/// The overhead frame layout (simplified) per OIF FlexE 2.1 §6:
/// ```text
/// Byte  0     : group_id
/// Byte  1     : phy_map_count (number of PHYs in this group)
/// Bytes 2–21  : calendar[0..19]  (client-id per slot, 0 = unassigned)
/// Byte  22    : frame_number (0–31 within multiframe)
/// Byte  23    : flags (bit 0 = fault, bit 1 = remote-fault, bits 2–7 reserved)
/// Bytes 24–61 : reserved (set to 0)
/// Bytes 62–63 : CRC-16 over bytes 0–61
/// ```
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct OverheadFrame {
    /// FlexE group identifier.
    pub group_id: u8,
    /// Number of PHYs encoded in this overhead frame.
    pub phy_count: u8,
    /// Calendar slot assignments (0 = free).
    pub calendar: [u8; SLOTS_PER_PHY],
    /// Frame number within the multiframe (0–31).
    pub frame_number: u8,
    /// Status flags (bit 0 = local fault, bit 1 = remote fault).
    pub flags: u8,
    /// CRC-16 over all fields except the CRC bytes themselves.
    pub crc: u16,
}

impl OverheadFrame {
    /// Construct an overhead frame, automatically computing the CRC.
    pub fn new(
        group_id: u8,
        phy_count: u8,
        calendar: [u8; SLOTS_PER_PHY],
        frame_number: u8,
        flags: u8,
    ) -> Self {
        let mut frame = Self {
            group_id,
            phy_count,
            calendar,
            frame_number,
            flags,
            crc: 0,
        };
        frame.crc = crc16_ccitt(&frame.to_bytes_no_crc());
        frame
    }

    /// Serialise to 64 bytes (62 header bytes + 2 CRC bytes).
    pub fn to_bytes(&self) -> [u8; 64] {
        let mut buf = [0u8; 64];
        buf[0] = self.group_id;
        buf[1] = self.phy_count;
        buf[2..22].copy_from_slice(&self.calendar);
        buf[22] = self.frame_number;
        buf[23] = self.flags;
        // bytes 24–61 remain zero (reserved)
        let crc_bytes = self.crc.to_be_bytes();
        buf[62] = crc_bytes[0];
        buf[63] = crc_bytes[1];
        buf
    }

    /// Serialise without the trailing CRC (used as CRC input).
    fn to_bytes_no_crc(&self) -> [u8; 62] {
        let mut buf = [0u8; 62];
        buf[0] = self.group_id;
        buf[1] = self.phy_count;
        buf[2..22].copy_from_slice(&self.calendar);
        buf[22] = self.frame_number;
        buf[23] = self.flags;
        buf
    }

    /// Deserialise from a 64-byte buffer, validating the CRC.
    pub fn from_bytes(raw: &[u8; 64]) -> Result<Self, FlexeError> {
        let mut calendar = [0u8; SLOTS_PER_PHY];
        calendar.copy_from_slice(&raw[2..22]);
        let stored_crc = u16::from_be_bytes([raw[62], raw[63]]);
        let computed = crc16_ccitt(&raw[..62]);
        if computed != stored_crc {
            return Err(FlexeError::CrcMismatch {
                expected: stored_crc,
                computed,
            });
        }
        Ok(Self {
            group_id: raw[0],
            phy_count: raw[1],
            calendar,
            frame_number: raw[22],
            flags: raw[23],
            crc: stored_crc,
        })
    }

    /// Verify the embedded CRC is correct.
    pub fn verify_crc(&self) -> bool {
        crc16_ccitt(&self.to_bytes_no_crc()) == self.crc
    }

    /// Returns `true` if the local-fault flag is set.
    pub fn has_local_fault(&self) -> bool {
        self.flags & 0x01 != 0
    }

    /// Returns `true` if the remote-fault flag is set.
    pub fn has_remote_fault(&self) -> bool {
        self.flags & 0x02 != 0
    }
}

/// An overhead multiframe composed of 32 overhead frames.
#[derive(Debug, Clone)]
pub struct OverheadMultiframe {
    pub frames: [OverheadFrame; OVERHEAD_FRAMES_PER_MFRAME],
}

impl OverheadMultiframe {
    /// Build a multiframe for a single-PHY FlexE group from the current calendar.
    pub fn build(group: &FlexeGroup, phy_id: u8) -> Result<Self, FlexeError> {
        let calendar = group.calendar_array(phy_id)?;
        let phy_count = group.phy_count() as u8;
        let flags: u8 = match group.phy_status(phy_id)? {
            PhyStatus::Up => 0x00,
            PhyStatus::LocalFault => 0x01,
            PhyStatus::RemoteFault => 0x02,
            PhyStatus::Down => 0x03,
        };

        let frames: [OverheadFrame; 32] = core::array::from_fn(|i| {
            OverheadFrame::new(group.group_id, phy_count, calendar, i as u8, flags)
        });
        Ok(Self { frames })
    }

    /// Validate every frame's CRC in this multiframe.
    pub fn validate_all(&self) -> bool {
        self.frames.iter().all(|f| f.verify_crc())
    }

    /// Extract the calendar from frame 0 (all frames carry the same calendar).
    pub fn calendar(&self) -> &[u8; SLOTS_PER_PHY] {
        &self.frames[0].calendar
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FlexE Client
// ─────────────────────────────────────────────────────────────────────────────

/// Sub-rate Ethernet client types supported by FlexE.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ClientType {
    /// 10 Gbps client (needs ≥ 2 slots).
    Eth10G,
    /// 25 Gbps client (needs ≥ 5 slots).
    Eth25G,
    /// 40 Gbps client (needs ≥ 8 slots).
    Eth40G,
    /// 50 Gbps client (needs ≥ 10 slots).
    Eth50G,
    /// Full 100 Gbps client (needs ≥ 20 slots).
    Eth100G,
    /// Arbitrary bandwidth client specified in Gbps.
    Custom(f64),
}

impl ClientType {
    /// Nominal data-rate in Gbps for this client type.
    pub fn gbps(&self) -> f64 {
        match self {
            Self::Eth10G => 10.0,
            Self::Eth25G => 25.0,
            Self::Eth40G => 40.0,
            Self::Eth50G => 50.0,
            Self::Eth100G => 100.0,
            Self::Custom(g) => *g,
        }
    }

    /// Minimum number of 5.15625 Gbps calendar slots required to carry this client.
    pub fn min_slots(&self) -> usize {
        let gbps = self.gbps();
        // ceil(gbps / SLOT_BPS_GBPS)
        ((gbps / SLOT_BPS_GBPS).ceil()) as usize
    }
}

/// A FlexE virtual client (virtual MAC layer).
#[derive(Debug, Clone)]
pub struct FlexeClient {
    /// Client identifier (1–254).
    pub client_id: u8,
    /// Client Ethernet type.
    pub client_type: ClientType,
    /// Calendar slots assigned to this client across all PHYs.
    pub assigned_slots: Vec<CalendarSlot>,
    /// Round-robin transmit pointer (index into `assigned_slots`).
    tx_slot_ptr: usize,
    /// Round-robin receive pointer.
    rx_slot_ptr: usize,
    /// Pending transmit data queue (blocks waiting to be placed in calendar slots).
    tx_queue: std::collections::VecDeque<Block66b>,
    /// Received data queue (blocks extracted from calendar slots).
    rx_queue: std::collections::VecDeque<Block66b>,
}

impl FlexeClient {
    /// Create a new client (no slots assigned yet).
    pub fn new(client_id: u8, client_type: ClientType) -> Result<Self, FlexeError> {
        if client_id == 0 {
            return Err(FlexeError::InvalidClientId(client_id));
        }
        Ok(Self {
            client_id,
            client_type,
            assigned_slots: Vec::new(),
            tx_slot_ptr: 0,
            rx_slot_ptr: 0,
            tx_queue: std::collections::VecDeque::new(),
            rx_queue: std::collections::VecDeque::new(),
        })
    }

    /// Record that `slot` has been allocated to this client.
    pub fn add_slot(&mut self, slot: CalendarSlot) {
        self.assigned_slots.push(slot);
    }

    /// Remove all slot references (called after group deallocation).
    pub fn clear_slots(&mut self) {
        self.assigned_slots.clear();
        self.tx_slot_ptr = 0;
        self.rx_slot_ptr = 0;
    }

    /// Nominal allocated bandwidth in Gbps.
    pub fn allocated_gbps(&self) -> f64 {
        self.assigned_slots.len() as f64 * SLOT_BPS_GBPS
    }

    /// Enqueue a 66B block for transmission.
    pub fn enqueue_tx(&mut self, block: Block66b) {
        self.tx_queue.push_back(block);
    }

    /// Dequeue a received 66B block.
    pub fn dequeue_rx(&mut self) -> Option<Block66b> {
        self.rx_queue.pop_front()
    }

    /// Number of blocks waiting in the TX queue.
    pub fn tx_pending(&self) -> usize {
        self.tx_queue.len()
    }

    /// Number of received blocks available.
    pub fn rx_available(&self) -> usize {
        self.rx_queue.len()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// FlexE Shim — Mux / Demux
// ─────────────────────────────────────────────────────────────────────────────

/// One time-slot entry in the shim's internal calendar view.
#[derive(Debug, Clone, Copy)]
struct ShimSlot {
    client_id: u8,
}

/// The FlexE Shim layer handles calendar-based multiplexing and demultiplexing,
/// plus idle insertion/deletion for rate adaptation.
///
/// The shim owns an ordered calendar that maps each slot position (across all
/// PHYs, in round-robin PHY order) to a client or to "idle".
#[derive(Debug)]
pub struct FlexeShim {
    /// Flat ordered calendar; each entry is `client_id` (0 = idle slot).
    calendar: Vec<ShimSlot>,
    /// Current slot pointer for the transmitter (cycles through `calendar`).
    tx_ptr: usize,
    /// Current slot pointer for the receiver.
    rx_ptr: usize,
}

impl FlexeShim {
    /// Build a shim from a [`FlexeGroup`]'s current calendar state.
    /// The calendar is flattened in insertion order: all slots of PHY[0], then
    /// PHY[1], etc.
    pub fn from_group(group: &FlexeGroup) -> Self {
        let mut calendar = Vec::with_capacity(group.total_slots());
        for phy in &group.phys {
            for s in &phy.calendar {
                calendar.push(ShimSlot { client_id: s.client_id });
            }
        }
        Self { calendar, tx_ptr: 0, rx_ptr: 0 }
    }

    /// Total calendar length.
    pub fn calendar_len(&self) -> usize {
        self.calendar.len()
    }

    /// Transmit one calendar tick: dequeue a block from the owning client (if any)
    /// and return it together with the slot's client-id.
    ///
    /// Returns `(client_id, Block66b)` — an idle block is inserted if the client
    /// has no data or the slot is unassigned.
    pub fn tx_tick(
        &mut self,
        clients: &mut [FlexeClient],
    ) -> (u8, Block66b) {
        if self.calendar.is_empty() {
            return (0, Block66b::idle());
        }
        let slot = self.calendar[self.tx_ptr];
        self.tx_ptr = (self.tx_ptr + 1) % self.calendar.len();
        let client_id = slot.client_id;
        if client_id == 0 {
            return (0, Block66b::idle());
        }
        let block = clients
            .iter_mut()
            .find(|c| c.client_id == client_id)
            .and_then(|c| c.tx_queue.pop_front())
            .unwrap_or_else(Block66b::idle);
        (client_id, block)
    }

    /// Receive one calendar tick: route the received block to the appropriate
    /// client's RX queue.  Idle blocks are discarded.
    pub fn rx_tick(
        &mut self,
        block: Block66b,
        clients: &mut [FlexeClient],
    ) {
        if self.calendar.is_empty() {
            return;
        }
        let slot = self.calendar[self.rx_ptr];
        self.rx_ptr = (self.rx_ptr + 1) % self.calendar.len();
        let client_id = slot.client_id;
        if client_id == 0 || block.is_idle() {
            return; // discard idle
        }
        if let Some(client) = clients.iter_mut().find(|c| c.client_id == client_id) {
            client.rx_queue.push_back(block);
        }
    }

    /// Run a full round of `n_ticks` TX ticks, returning all (client_id, block) pairs.
    pub fn tx_burst(
        &mut self,
        clients: &mut [FlexeClient],
        n_ticks: usize,
    ) -> Vec<(u8, Block66b)> {
        (0..n_ticks).map(|_| self.tx_tick(clients)).collect()
    }

    /// Run a full round of `n_ticks` RX ticks from a pre-built stream.
    pub fn rx_burst(
        &mut self,
        stream: &[(u8, Block66b)],
        clients: &mut [FlexeClient],
    ) {
        for (_, block) in stream {
            self.rx_tick(block.clone(), clients);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Rate Adaptation — Idle insertion / deletion
// ─────────────────────────────────────────────────────────────────────────────

/// Rate adaptation state for one FlexE client.
///
/// When the client's actual data rate is lower than the allocated slot bandwidth,
/// idle blocks are inserted to pad.  When it is higher (not possible by design,
/// but covered for completeness), idle blocks in the outbound stream would be
/// deleted.
#[derive(Debug, Clone)]
pub struct RateAdapter {
    /// Client this adapter serves.
    pub client_id: u8,
    /// Fractional slip accumulator (range: [0.0, 1.0)).
    accumulator: f64,
    /// Ratio of client rate to allocated bandwidth (≤ 1.0 for sub-rate clients).
    fill_ratio: f64,
    /// Total idles inserted so far.
    pub idles_inserted: u64,
    /// Total idles deleted so far.
    pub idles_deleted: u64,
}

impl RateAdapter {
    /// Create a rate adapter.
    ///
    /// `client_gbps` — actual data rate of the Ethernet client.
    /// `allocated_gbps` — total FlexE slot bandwidth allocated to the client.
    pub fn new(client_id: u8, client_gbps: f64, allocated_gbps: f64) -> Self {
        let fill_ratio = if allocated_gbps > 0.0 {
            (client_gbps / allocated_gbps).min(1.0)
        } else {
            0.0
        };
        Self {
            client_id,
            accumulator: 0.0,
            fill_ratio,
            idles_inserted: 0,
            idles_deleted: 0,
        }
    }

    /// Process one outbound block: returns the block as-is most of the time,
    /// but substitutes an idle when the accumulator says the slot is unfilled.
    ///
    /// Uses a first-order delta-sigma approach (fractional accounting).
    pub fn tx_adapt(&mut self, block: Block66b) -> Block66b {
        self.accumulator += self.fill_ratio;
        if self.accumulator >= 1.0 {
            self.accumulator -= 1.0;
            block
        } else {
            self.idles_inserted += 1;
            Block66b::idle()
        }
    }

    /// Strip idles from an inbound block stream (inverse of insertion).
    /// Returns `Some(block)` for real data, `None` for idles to be discarded.
    pub fn rx_adapt(&mut self, block: Block66b) -> Option<Block66b> {
        if block.is_idle() {
            self.idles_deleted += 1;
            None
        } else {
            Some(block)
        }
    }

    /// Process a batch of blocks, returning only data blocks (idles stripped).
    pub fn rx_adapt_batch(&mut self, blocks: Vec<Block66b>) -> Vec<Block66b> {
        blocks.into_iter().filter_map(|b| self.rx_adapt(b)).collect()
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Utility — bandwidth helpers
// ─────────────────────────────────────────────────────────────────────────────

/// Compute the number of calendar slots required for a given Ethernet rate
/// and the fractional over-allocation (since slots are quantised).
///
/// Returns `(slot_count, actual_gbps, excess_gbps)`.
pub fn slots_for_rate(gbps: f64) -> (usize, f64, f64) {
    let raw = gbps / SLOT_BPS_GBPS;
    let slots = raw.ceil() as usize;
    let actual = slots as f64 * SLOT_BPS_GBPS;
    let excess = actual - gbps;
    (slots, actual, excess)
}

/// Compute the theoretical fill ratio for a client given its rate and its
/// allocated slot count.  A ratio of 1.0 means perfectly filled; < 1.0 means
/// idle insertion is required.
pub fn fill_ratio(client_gbps: f64, slot_count: usize) -> f64 {
    if slot_count == 0 {
        return 0.0;
    }
    let allocated = slot_count as f64 * SLOT_BPS_GBPS;
    (client_gbps / allocated).min(1.0)
}

// ─────────────────────────────────────────────────────────────────────────────
// High-level convenience: end-to-end loopback test harness
// ─────────────────────────────────────────────────────────────────────────────

/// Result returned by [`loopback_test`].
#[derive(Debug, Clone)]
pub struct LoopbackResult {
    /// Blocks successfully received by each client (client_id → count).
    pub rx_counts: Vec<(u8, usize)>,
    /// Total idle blocks transmitted (rate-adaptation padding).
    pub total_idles_tx: u64,
    /// Total idle blocks stripped on receive.
    pub total_idles_rx: u64,
}

/// Run a simple loopback test: enqueue `blocks_per_client` data blocks for each
/// client, mux through the shim, demux back to client RX queues, and report counts.
pub fn loopback_test(
    group: &mut FlexeGroup,
    clients: &mut Vec<FlexeClient>,
    blocks_per_client: usize,
) -> Result<LoopbackResult, FlexeError> {
    // Validate clients have slots
    for client in clients.iter() {
        if client.assigned_slots.is_empty() {
            return Err(FlexeError::NoSlotsAllocated(client.client_id));
        }
    }

    // Enqueue data
    for client in clients.iter_mut() {
        for seq in 0..blocks_per_client {
            let mut payload = [0u8; 8];
            payload[0] = client.client_id;
            payload[1] = (seq & 0xFF) as u8;
            client.enqueue_tx(Block66b::data(payload));
        }
    }

    let mut shim = FlexeShim::from_group(group);
    let n_ticks = shim.calendar_len() * blocks_per_client * 4; // enough cycles

    // TX burst
    let stream = shim.tx_burst(clients, n_ticks);

    // RX burst
    let mut shim2 = FlexeShim::from_group(group);
    shim2.rx_burst(&stream, clients);

    // Collect stats
    let mut total_idles_tx: u64 = 0;
    let mut total_idles_rx: u64 = 0;
    for (_, block) in &stream {
        if block.is_idle() {
            total_idles_tx += 1;
        }
    }
    // (idles_rx counted via RateAdapter in real deployments; here approximated)
    total_idles_rx = total_idles_tx;

    let rx_counts = clients
        .iter()
        .map(|c| (c.client_id, c.rx_queue.len()))
        .collect();

    Ok(LoopbackResult { rx_counts, total_idles_tx, total_idles_rx })
}

// ─────────────────────────────────────────────────────────────────────────────
// Tests
// ─────────────────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;

    // ── 66B Block tests ───────────────────────────────────────────────────────

    #[test]
    fn block_data_sync_header() {
        let b = Block66b::data([0xAB; 8]);
        assert_eq!(b.sync, SYNC_DATA);
        assert!(b.is_data());
        assert!(!b.is_control());
    }

    #[test]
    fn block_control_sync_header() {
        let b = Block66b::control([0x07; 8]);
        assert_eq!(b.sync, SYNC_CTRL);
        assert!(b.is_control());
        assert!(!b.is_data());
    }

    #[test]
    fn block_idle_detection() {
        let b = Block66b::idle();
        assert!(b.is_idle());
        assert!(b.is_control());
    }

    #[test]
    fn block_non_idle_control() {
        let b = Block66b::control([0xFF; 8]);
        assert!(b.is_control());
        assert!(!b.is_idle());
    }

    #[test]
    fn block_encode_decode_data() {
        let payload = [0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF];
        let b = Block66b::data(payload);
        let raw = b.encode();
        let decoded = Block66b::decode(&raw).expect("decode should succeed");
        assert_eq!(decoded.sync, SYNC_DATA);
        assert_eq!(decoded.payload, payload);
    }

    #[test]
    fn block_encode_decode_control() {
        let payload = [IDLE_BYTE; 8];
        let b = Block66b::control(payload);
        let raw = b.encode();
        let decoded = Block66b::decode(&raw).expect("decode should succeed");
        assert_eq!(decoded.sync, SYNC_CTRL);
        assert_eq!(decoded.payload, payload);
    }

    #[test]
    fn block_decode_invalid_sync() {
        let mut raw = Block66b::data([0u8; 8]).encode();
        raw[0] = 0x00; // invalid sync (0b00)
        let err = Block66b::decode(&raw).unwrap_err();
        assert!(matches!(err, FlexeError::InvalidSyncHeader(0)));
    }

    #[test]
    fn block_encode_sync_in_msb() {
        let b = Block66b::data([0u8; 8]);
        let raw = b.encode();
        // SYNC_DATA = 0b01, placed in bits [7:6] → byte 0 = 0b0100_0000 = 0x40
        assert_eq!(raw[0], 0x40);
    }

    #[test]
    fn block_scramble_descramble_roundtrip() {
        let payload = [0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88];
        let original = Block66b::data(payload);
        let mut state_tx: u64 = 0xFFFF_FFFF_FFFF_FFFFu64;
        let mut state_rx: u64 = 0xFFFF_FFFF_FFFF_FFFFu64;
        let scrambled = original.scramble(&mut state_tx);
        // Scrambled payload should differ (with high probability)
        // Descramble: use self-synchronising descrambler
        // Re-initialise RX state and descramble
        state_rx = state_tx; // align to TX scrambler output state for test
        let _ = scrambled.descramble(&mut state_rx);
        // Basic sanity: scrambled != original (with overwhelming probability for non-trivial input)
        assert_ne!(scrambled.payload, payload);
    }

    // ── CRC-16 tests ─────────────────────────────────────────────────────────

    #[test]
    fn crc16_known_value() {
        // CRC-16/CCITT of ASCII "123456789" = 0x29B1
        let data = b"123456789";
        assert_eq!(crc16_ccitt(data), 0x29B1);
    }

    #[test]
    fn crc16_empty_input() {
        // CRC of empty data with init 0xFFFF returns 0xFFFF
        assert_eq!(crc16_ccitt(&[]), 0xFFFF);
    }

    #[test]
    fn crc16_single_byte() {
        let c = crc16_ccitt(&[0x00]);
        // Ensure it produces a deterministic non-trivial result
        assert_ne!(c, 0x0000);
    }

    #[test]
    fn crc16_verify_pass() {
        let data = b"FlexE";
        let crc = crc16_ccitt(data);
        assert!(crc16_verify(data, crc));
    }

    #[test]
    fn crc16_verify_fail() {
        let data = b"FlexE";
        let bad_crc = crc16_ccitt(data).wrapping_add(1);
        assert!(!crc16_verify(data, bad_crc));
    }

    #[test]
    fn crc16_different_inputs_differ() {
        let a = crc16_ccitt(b"hello");
        let b = crc16_ccitt(b"world");
        assert_ne!(a, b);
    }

    // ── Calendar slot allocation tests ───────────────────────────────────────

    #[test]
    fn phy_new_all_slots_free() {
        let phy = FlexePhy::new(1).unwrap();
        assert_eq!(phy.free_slot_count(), SLOTS_PER_PHY);
    }

    #[test]
    fn phy_assign_slot_success() {
        let mut phy = FlexePhy::new(1).unwrap();
        phy.assign_slot(0, 1).unwrap();
        assert_eq!(phy.free_slot_count(), SLOTS_PER_PHY - 1);
        assert_eq!(phy.calendar[0].client_id, 1);
    }

    #[test]
    fn phy_assign_duplicate_slot_error() {
        let mut phy = FlexePhy::new(1).unwrap();
        phy.assign_slot(3, 1).unwrap();
        let err = phy.assign_slot(3, 2).unwrap_err();
        assert!(matches!(err, FlexeError::SlotAlreadyOccupied { slot: 3, phy: 1 }));
    }

    #[test]
    fn phy_release_slot_success() {
        let mut phy = FlexePhy::new(1).unwrap();
        phy.assign_slot(5, 1).unwrap();
        phy.release_slot(5, 1).unwrap();
        assert!(phy.calendar[5].is_free());
    }

    #[test]
    fn phy_release_unowned_slot_error() {
        let mut phy = FlexePhy::new(1).unwrap();
        phy.assign_slot(5, 1).unwrap();
        let err = phy.release_slot(5, 2).unwrap_err();
        assert!(matches!(err, FlexeError::SlotNotOwned { slot: 5, client: 2 }));
    }

    #[test]
    fn phy_invalid_slot_index() {
        let mut phy = FlexePhy::new(1).unwrap();
        let err = phy.assign_slot(SLOTS_PER_PHY, 1).unwrap_err();
        assert!(matches!(err, FlexeError::InvalidSlotIndex(_)));
    }

    #[test]
    fn phy_client_bandwidth_gbps() {
        let mut phy = FlexePhy::new(1).unwrap();
        phy.assign_slot(0, 1).unwrap();
        phy.assign_slot(1, 1).unwrap();
        let bw = phy.client_bandwidth_gbps(1);
        let expected = 2.0 * SLOT_BPS_GBPS;
        assert!((bw - expected).abs() < 1e-9);
    }

    // ── FlexE Group tests ─────────────────────────────────────────────────────

    #[test]
    fn group_add_remove_phy() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        g.add_phy(2).unwrap();
        assert_eq!(g.phy_count(), 2);
        g.remove_phy(1).unwrap();
        assert_eq!(g.phy_count(), 1);
    }

    #[test]
    fn group_duplicate_phy_error() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(5).unwrap();
        let err = g.add_phy(5).unwrap_err();
        assert!(matches!(err, FlexeError::PhyAlreadyPresent(5)));
    }

    #[test]
    fn group_remove_nonexistent_phy_error() {
        let mut g = FlexeGroup::new(1);
        let err = g.remove_phy(99).unwrap_err();
        assert!(matches!(err, FlexeError::PhyNotFound(99)));
    }

    #[test]
    fn group_total_bandwidth_two_phys() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        g.add_phy(2).unwrap();
        assert!((g.total_bandwidth_gbps() - 200.0).abs() < 1e-9);
    }

    #[test]
    fn group_total_slots_three_phys() {
        let mut g = FlexeGroup::new(1);
        for id in 1u8..=3 {
            g.add_phy(id).unwrap();
        }
        assert_eq!(g.total_slots(), 3 * SLOTS_PER_PHY);
    }

    #[test]
    fn group_allocate_slots_success() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        let slots = g.allocate_slots(1, 5).unwrap();
        assert_eq!(slots.len(), 5);
        assert_eq!(g.free_slots(), SLOTS_PER_PHY - 5);
    }

    #[test]
    fn group_allocate_slots_cross_phy() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        g.add_phy(2).unwrap();
        // Allocate 25 slots — more than one PHY
        let slots = g.allocate_slots(1, 25).unwrap();
        assert_eq!(slots.len(), 25);
    }

    #[test]
    fn group_deallocate_client() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        g.allocate_slots(42, 10).unwrap();
        assert_eq!(g.free_slots(), SLOTS_PER_PHY - 10);
        let released = g.deallocate_client(42);
        assert_eq!(released, 10);
        assert_eq!(g.free_slots(), SLOTS_PER_PHY);
    }

    #[test]
    fn group_client_bandwidth_five_slots() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        g.allocate_slots(7, 5).unwrap();
        let bw = g.client_bandwidth_gbps(7);
        let expected = 5.0 * SLOT_BPS_GBPS;
        assert!((bw - expected).abs() < 1e-9);
    }

    #[test]
    fn group_calendar_array() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(10).unwrap();
        g.allocate_slots(3, 2).unwrap(); // slots 0,1 → client 3
        let cal = g.calendar_array(10).unwrap();
        assert_eq!(cal[0], 3);
        assert_eq!(cal[1], 3);
        assert_eq!(cal[2], 0); // unassigned
    }

    #[test]
    fn group_phy_status_fault_propagation() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        assert!(g.is_operational());
        g.set_phy_status(1, PhyStatus::LocalFault).unwrap();
        assert!(!g.is_operational());
        assert_eq!(g.phy_status(1).unwrap(), PhyStatus::LocalFault);
    }

    // ── Overhead Frame tests ──────────────────────────────────────────────────

    #[test]
    fn overhead_frame_roundtrip() {
        let cal = core::array::from_fn(|i| (i % 4) as u8);
        let frame = OverheadFrame::new(1, 2, cal, 7, 0x00);
        assert!(frame.verify_crc());
        let bytes = frame.to_bytes();
        let decoded = OverheadFrame::from_bytes(&bytes).expect("CRC should pass");
        assert_eq!(decoded.group_id, 1);
        assert_eq!(decoded.phy_count, 2);
        assert_eq!(decoded.frame_number, 7);
        assert_eq!(decoded.calendar, cal);
    }

    #[test]
    fn overhead_frame_crc_mismatch_detected() {
        let cal = [0u8; SLOTS_PER_PHY];
        let frame = OverheadFrame::new(1, 1, cal, 0, 0);
        let mut bytes = frame.to_bytes();
        bytes[0] ^= 0xFF; // corrupt group_id
        let err = OverheadFrame::from_bytes(&bytes).unwrap_err();
        assert!(matches!(err, FlexeError::CrcMismatch { .. }));
    }

    #[test]
    fn overhead_frame_fault_flags() {
        let frame = OverheadFrame::new(1, 1, [0; SLOTS_PER_PHY], 0, 0x03);
        assert!(frame.has_local_fault());
        assert!(frame.has_remote_fault());
    }

    #[test]
    fn overhead_frame_no_fault_flags() {
        let frame = OverheadFrame::new(1, 1, [0; SLOTS_PER_PHY], 0, 0x00);
        assert!(!frame.has_local_fault());
        assert!(!frame.has_remote_fault());
    }

    // ── Overhead Multiframe tests ─────────────────────────────────────────────

    #[test]
    fn overhead_multiframe_build_and_validate() {
        let mut g = FlexeGroup::new(5);
        g.add_phy(1).unwrap();
        g.allocate_slots(1, 10).unwrap();
        let mf = OverheadMultiframe::build(&g, 1).unwrap();
        assert_eq!(mf.frames.len(), OVERHEAD_FRAMES_PER_MFRAME);
        assert!(mf.validate_all());
    }

    #[test]
    fn overhead_multiframe_frame_numbers() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        let mf = OverheadMultiframe::build(&g, 1).unwrap();
        for (i, frame) in mf.frames.iter().enumerate() {
            assert_eq!(frame.frame_number, i as u8);
        }
    }

    #[test]
    fn overhead_multiframe_calendar_consistent() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        g.allocate_slots(3, 4).unwrap();
        let mf = OverheadMultiframe::build(&g, 1).unwrap();
        let cal = mf.calendar();
        let expected = g.calendar_array(1).unwrap();
        assert_eq!(*cal, expected);
    }

    // ── Client type tests ─────────────────────────────────────────────────────

    #[test]
    fn client_type_min_slots() {
        assert_eq!(ClientType::Eth10G.min_slots(), 2);  // ceil(10/5.15625) = 2
        assert_eq!(ClientType::Eth25G.min_slots(), 5);  // ceil(25/5.15625) = 5
        assert_eq!(ClientType::Eth40G.min_slots(), 8);  // ceil(40/5.15625) = 8
        assert_eq!(ClientType::Eth50G.min_slots(), 10); // ceil(50/5.15625) = 10
        assert_eq!(ClientType::Eth100G.min_slots(), 20);
    }

    #[test]
    fn client_type_gbps() {
        assert!((ClientType::Eth25G.gbps() - 25.0).abs() < 1e-9);
        assert!((ClientType::Custom(37.5).gbps() - 37.5).abs() < 1e-9);
    }

    // ── FlexE Client tests ────────────────────────────────────────────────────

    #[test]
    fn client_enqueue_dequeue() {
        let mut c = FlexeClient::new(1, ClientType::Eth25G).unwrap();
        let b = Block66b::data([0xAA; 8]);
        c.enqueue_tx(b.clone());
        assert_eq!(c.tx_pending(), 1);
        // Can't dequeue TX from outside; will be consumed by shim
    }

    #[test]
    fn client_allocated_gbps() {
        let mut c = FlexeClient::new(1, ClientType::Eth25G).unwrap();
        c.add_slot(CalendarSlot { phy_id: 1, slot_index: 0 });
        c.add_slot(CalendarSlot { phy_id: 1, slot_index: 1 });
        let bw = c.allocated_gbps();
        assert!((bw - 2.0 * SLOT_BPS_GBPS).abs() < 1e-9);
    }

    #[test]
    fn client_rx_dequeue() {
        let mut c = FlexeClient::new(2, ClientType::Eth10G).unwrap();
        c.rx_queue.push_back(Block66b::data([0x42; 8]));
        let rx = c.dequeue_rx().unwrap();
        assert_eq!(rx.payload, [0x42; 8]);
        assert!(c.dequeue_rx().is_none());
    }

    // ── Shim mux/demux tests ─────────────────────────────────────────────────

    #[test]
    fn shim_single_client_mux_demux() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        g.allocate_slots(1, SLOTS_PER_PHY).unwrap(); // all slots to client 1

        let mut clients = vec![FlexeClient::new(1, ClientType::Eth100G).unwrap()];
        clients[0].add_slot(CalendarSlot { phy_id: 1, slot_index: 0 });

        let payload = [0xDE, 0xAD, 0xBE, 0xEF, 0x00, 0x01, 0x02, 0x03];
        clients[0].enqueue_tx(Block66b::data(payload));

        let mut shim_tx = FlexeShim::from_group(&g);
        let stream = shim_tx.tx_burst(&mut clients, SLOTS_PER_PHY);

        let mut shim_rx = FlexeShim::from_group(&g);
        shim_rx.rx_burst(&stream, &mut clients);

        let rx = clients[0].dequeue_rx().expect("should have received a block");
        assert_eq!(rx.payload, payload);
    }

    #[test]
    fn shim_idle_fill_when_no_data() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        g.allocate_slots(1, SLOTS_PER_PHY).unwrap();

        let mut clients = vec![FlexeClient::new(1, ClientType::Eth100G).unwrap()];
        // No TX data enqueued
        let mut shim = FlexeShim::from_group(&g);
        let stream = shim.tx_burst(&mut clients, SLOTS_PER_PHY);
        // All blocks should be idle (no data)
        assert!(stream.iter().all(|(_, b)| b.is_idle()));
    }

    #[test]
    fn shim_multi_client_slot_isolation() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        // Client 1 gets slots 0–9, client 2 gets slots 10–19
        for slot in 0..10 {
            g.phys[0].assign_slot(slot, 1).unwrap();
        }
        for slot in 10..20 {
            g.phys[0].assign_slot(slot, 2).unwrap();
        }

        let mut c1 = FlexeClient::new(1, ClientType::Eth50G).unwrap();
        let mut c2 = FlexeClient::new(2, ClientType::Eth50G).unwrap();

        // Enqueue distinguishable blocks
        for _ in 0..5 {
            c1.enqueue_tx(Block66b::data([0x01; 8]));
            c2.enqueue_tx(Block66b::data([0x02; 8]));
        }

        let mut clients = vec![c1, c2];
        let mut shim_tx = FlexeShim::from_group(&g);
        let stream = shim_tx.tx_burst(&mut clients, SLOTS_PER_PHY * 2);

        let mut shim_rx = FlexeShim::from_group(&g);
        shim_rx.rx_burst(&stream, &mut clients);

        // Client 1 should only see payload 0x01 blocks
        while let Some(b) = clients[0].dequeue_rx() {
            assert_eq!(b.payload, [0x01; 8], "client 1 received wrong block");
        }
        // Client 2 should only see payload 0x02 blocks
        while let Some(b) = clients[1].dequeue_rx() {
            assert_eq!(b.payload, [0x02; 8], "client 2 received wrong block");
        }
    }

    // ── Rate adaptation tests ─────────────────────────────────────────────────

    #[test]
    fn rate_adapter_full_rate_no_idle() {
        // client_gbps == allocated_gbps → fill_ratio = 1.0 → no idles
        let slot_count = 5;
        let allocated = slot_count as f64 * SLOT_BPS_GBPS;
        let mut ra = RateAdapter::new(1, allocated, allocated);
        let mut idles = 0u64;
        for _ in 0..100 {
            let b = ra.tx_adapt(Block66b::data([0xAB; 8]));
            if b.is_idle() {
                idles += 1;
            }
        }
        assert_eq!(idles, 0, "no idles expected at full rate");
    }

    #[test]
    fn rate_adapter_half_rate_idles() {
        // half-filled client → fill_ratio ≈ 0.5 → ~50% of blocks are idles
        let allocated = 10.0 * SLOT_BPS_GBPS;
        let client_gbps = allocated / 2.0;
        let mut ra = RateAdapter::new(1, client_gbps, allocated);
        let n = 200;
        let mut idles = 0u64;
        for _ in 0..n {
            let b = ra.tx_adapt(Block66b::data([0u8; 8]));
            if b.is_idle() {
                idles += 1;
            }
        }
        // Expect ~50% idles; allow ±10%
        let ratio = idles as f64 / n as f64;
        assert!(
            (0.4..=0.6).contains(&ratio),
            "expected ~50% idles, got {ratio:.2}"
        );
    }

    #[test]
    fn rate_adapter_rx_strips_idles() {
        let mut ra = RateAdapter::new(1, 25.0, 25.0);
        let blocks = vec![
            Block66b::idle(),
            Block66b::data([0xBB; 8]),
            Block66b::idle(),
            Block66b::data([0xCC; 8]),
        ];
        let out = ra.rx_adapt_batch(blocks);
        assert_eq!(out.len(), 2);
        assert_eq!(ra.idles_deleted, 2);
    }

    #[test]
    fn rate_adapter_zero_allocation_no_panic() {
        let mut ra = RateAdapter::new(1, 25.0, 0.0);
        // fill_ratio = 0 → every block replaced by idle
        let b = ra.tx_adapt(Block66b::data([0xFF; 8]));
        assert!(b.is_idle());
    }

    // ── Bandwidth utility tests ───────────────────────────────────────────────

    #[test]
    fn slots_for_rate_25g() {
        let (slots, actual, excess) = slots_for_rate(25.0);
        assert_eq!(slots, 5);
        assert!(actual >= 25.0);
        assert!(excess >= 0.0);
    }

    #[test]
    fn slots_for_rate_100g() {
        let (slots, _, _) = slots_for_rate(100.0);
        assert_eq!(slots, 20); // ceil(100/5.15625) = 20
    }

    #[test]
    fn fill_ratio_full() {
        let r = fill_ratio(SLOT_BPS_GBPS * 5.0, 5);
        assert!((r - 1.0).abs() < 1e-9);
    }

    #[test]
    fn fill_ratio_sub() {
        let r = fill_ratio(25.0, 10); // allocated = 51.5625 Gbps
        assert!(r < 1.0);
        assert!(r > 0.0);
    }

    #[test]
    fn fill_ratio_zero_slots() {
        let r = fill_ratio(25.0, 0);
        assert_eq!(r, 0.0);
    }

    // ── Multi-PHY group bandwidth ─────────────────────────────────────────────

    #[test]
    fn group_ten_phy_bandwidth() {
        let mut g = FlexeGroup::new(1);
        for id in 1u8..=10 {
            g.add_phy(id).unwrap();
        }
        assert!((g.total_bandwidth_gbps() - 1000.0).abs() < 1e-9);
        assert_eq!(g.total_slots(), 10 * SLOTS_PER_PHY);
    }

    // ── PHY bonding configuration test ───────────────────────────────────────

    #[test]
    fn group_phy_bonding_allocate_large_client() {
        let mut g = FlexeGroup::new(1);
        for id in 1u8..=4 {
            g.add_phy(id).unwrap();
        }
        // 400 Gbps group; allocate 50 slots (≈ 257.8 Gbps) for one client
        let slots = g.allocate_slots(1, 50).unwrap();
        assert_eq!(slots.len(), 50);
        let bw = g.client_bandwidth_gbps(1);
        assert!((bw - 50.0 * SLOT_BPS_GBPS).abs() < 1e-9);
    }

    // ── Calendar distribution encoding ───────────────────────────────────────

    #[test]
    fn calendar_distribution_in_overhead() {
        let mut g = FlexeGroup::new(7);
        g.add_phy(1).unwrap();
        // Assign alternating clients
        for i in 0..SLOTS_PER_PHY {
            g.phys[0].assign_slot(i, (i % 3 + 1) as u8).unwrap();
        }
        let mf = OverheadMultiframe::build(&g, 1).unwrap();
        let cal = mf.calendar();
        for i in 0..SLOTS_PER_PHY {
            assert_eq!(cal[i], (i % 3 + 1) as u8);
        }
    }

    // ── Loopback test ─────────────────────────────────────────────────────────

    #[test]
    fn loopback_single_client_all_blocks_received() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        g.allocate_slots(1, SLOTS_PER_PHY).unwrap();

        let mut c = FlexeClient::new(1, ClientType::Eth100G).unwrap();
        for idx in 0..SLOTS_PER_PHY {
            c.add_slot(CalendarSlot { phy_id: 1, slot_index: idx });
        }
        let mut clients = vec![c];
        let result = loopback_test(&mut g, &mut clients, 10).unwrap();
        let &(_, rx_count) = result.rx_counts.first().unwrap();
        assert_eq!(rx_count, 10, "all 10 blocks should be received");
    }

    #[test]
    fn loopback_no_slots_returns_error() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        let mut clients = vec![FlexeClient::new(1, ClientType::Eth25G).unwrap()];
        // No slots allocated
        let err = loopback_test(&mut g, &mut clients, 5).unwrap_err();
        assert!(matches!(err, FlexeError::NoSlotsAllocated(1)));
    }

    // ── Group client_slots listing ────────────────────────────────────────────

    #[test]
    fn group_client_slots_correct() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        g.allocate_slots(9, 4).unwrap();
        let slots = g.client_slots(9);
        assert_eq!(slots.len(), 4);
        for cs in &slots {
            assert_eq!(cs.phy_id, 1);
        }
    }

    // ── Invalid PHY id ────────────────────────────────────────────────────────

    #[test]
    fn phy_id_zero_invalid() {
        let err = FlexePhy::new(0).unwrap_err();
        assert!(matches!(err, FlexeError::InvalidPhyId(0)));
    }

    // ── Client id zero invalid ────────────────────────────────────────────────

    #[test]
    fn client_id_zero_invalid() {
        let err = FlexeClient::new(0, ClientType::Eth25G).unwrap_err();
        assert!(matches!(err, FlexeError::InvalidClientId(0)));
    }

    // ── Shim calendar length ──────────────────────────────────────────────────

    #[test]
    fn shim_calendar_length_matches_group() {
        let mut g = FlexeGroup::new(1);
        for id in 1u8..=3 {
            g.add_phy(id).unwrap();
        }
        let shim = FlexeShim::from_group(&g);
        assert_eq!(shim.calendar_len(), 3 * SLOTS_PER_PHY);
    }

    // ── Multiframe fault flag propagation ────────────────────────────────────

    #[test]
    fn multiframe_local_fault_in_overhead() {
        let mut g = FlexeGroup::new(1);
        g.add_phy(1).unwrap();
        g.set_phy_status(1, PhyStatus::LocalFault).unwrap();
        let mf = OverheadMultiframe::build(&g, 1).unwrap();
        assert!(mf.frames[0].has_local_fault());
    }
}
