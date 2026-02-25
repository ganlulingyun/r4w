//! P25 Phase II TDMA Digital Voice Processing
//!
//! Implements TIA-102.BBAC P25 Phase II digital voice processing for public safety radio.
//! P25 Phase II migrates from FDMA (Phase I) to 2-slot TDMA within the same 12.5 kHz channel,
//! doubling spectral efficiency using π/4-DQPSK (H-DQPSK) modulation.
//!
//! # Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────┐
//! │                     P25 Phase II System                         │
//! │                                                                 │
//! │  Voice ──► AMBE+2 Vocoder ──► TCM Encoder ──► H-DQPSK Mod     │
//! │                                                                 │
//! │  TDMA Frame (180ms):                                            │
//! │  ┌──────────┬──────────┬──────────┬──────────┐                 │
//! │  │  Slot 0  │  Guard   │  Slot 1  │  Guard   │                 │
//! │  │  (ISCH)  │          │  (voice) │          │                 │
//! │  └──────────┴──────────┴──────────┴──────────┘                 │
//! │                                                                 │
//! │  Superframe (720ms) = 4 × Frame                                 │
//! └─────────────────────────────────────────────────────────────────┘
//! ```
//!
//! # Key Parameters
//!
//! - Symbol rate: 6000 symbols/s
//! - Modulation: π/4-DQPSK (H-DQPSK)
//! - Bit rate: 12000 bps gross
//! - TDMA slots: 2 per 12.5 kHz channel
//! - Frame duration: 180 ms
//! - Superframe: 720 ms (4 frames)
//! - Voice codec: AMBE+2 at 2450 bps (+ 1950 bps FEC = 4400 bps)
//!
//! # Standards
//!
//! - TIA-102.BBAC: P25 Phase II Air Interface
//! - TIA-102.BBAL: H-DQPSK modulation
//! - TIA-102.BBAB: TDMA frame structure

// ============================================================
// Constants
// ============================================================

/// Symbol rate in symbols per second
pub const SYMBOL_RATE: u32 = 6000;

/// Raw bit rate (2 bits per π/4-DQPSK symbol)
pub const BIT_RATE: u32 = 12000;

/// Frame duration in milliseconds
pub const FRAME_DURATION_MS: u32 = 180;

/// Number of TDMA slots per frame
pub const SLOTS_PER_FRAME: usize = 2;

/// Superframe duration in milliseconds
pub const SUPERFRAME_DURATION_MS: u32 = 720;

/// Number of frames per superframe
pub const FRAMES_PER_SUPERFRAME: usize = 4;

/// Symbols per TDMA slot (6000 sym/s × 0.09 s = 540)
pub const SYMBOLS_PER_SLOT: usize = 540;

/// Bits per TDMA slot (2 bits/symbol × 540 symbols)
pub const BITS_PER_SLOT: usize = 1080;

/// AMBE+2 vocoder bits per voice frame (20ms @ 2450 bps net = 49 bits)
pub const AMBE2_VOICE_BITS: usize = 49;

/// Total AMBE+2 frame bits including FEC (4400 bps total × 0.02s = 88 bits)
pub const AMBE2_FRAME_TOTAL_BITS: usize = 88;

/// Number of voice frames per TDMA slot
pub const VOICE_FRAMES_PER_SLOT: usize = 4;

/// Reed-Solomon(36,20,17) codeword length in symbols
pub const RS_N: usize = 36;
/// Reed-Solomon(36,20,17) message length in symbols
pub const RS_K: usize = 20;
/// Reed-Solomon(36,20,17) minimum distance
pub const RS_D: usize = 17;
/// Reed-Solomon error correction capability t = floor((d-1)/2)
pub const RS_T: usize = 8;

/// Golay(24,12,8) codeword length
pub const GOLAY_N: usize = 24;
/// Golay(24,12,8) message length
pub const GOLAY_K: usize = 12;

/// CRC-9 polynomial 0x1A3 (x^9 + x^7 + x^5 + x + 1)
pub const CRC9_POLY: u16 = 0x1A3;

/// Slot sync word for TDMA slot (48 bits, TIA-102.BBAC Table 9.11)
pub const SLOT_SYNC_WORD: u64 = 0x5575F5FF77FF_u64;

/// FACCH sync word
pub const FACCH_SYNC_WORD: u64 = 0xAA8AF7FF77FF_u64;

// ============================================================
// π/4-DQPSK (H-DQPSK) Modulation
// ============================================================

/// π/4-DQPSK constellation point (complex baseband sample)
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Qpsk {
    pub i: f64,
    pub q: f64,
}

impl Qpsk {
    pub fn new(i: f64, q: f64) -> Self {
        Self { i, q }
    }

    /// Magnitude of the complex symbol
    pub fn magnitude(&self) -> f64 {
        (self.i * self.i + self.q * self.q).sqrt()
    }

    /// Phase angle in radians [-π, +π]
    pub fn phase(&self) -> f64 {
        self.q.atan2(self.i)
    }
}

/// π/4-DQPSK phase shifts per dibit (Gray-coded, TIA-102.BBAL Table 1)
///
/// dibit (b1,b0): 00 → +π/4, 01 → +3π/4, 10 → −π/4, 11 → −3π/4
pub const DQPSK_PHASE_MAP: [f64; 4] = [
    core::f64::consts::FRAC_PI_4,               // 00 → +45°
    3.0 * core::f64::consts::FRAC_PI_4,         // 01 → +135°
    -core::f64::consts::FRAC_PI_4,              // 10 → −45°
    -3.0 * core::f64::consts::FRAC_PI_4,        // 11 → −135°
];

/// H-DQPSK modulator — differential phase encoding of dibits to IQ symbols
#[derive(Debug, Clone)]
pub struct HDqpskModulator {
    /// Accumulated carrier phase in radians
    pub phase: f64,
}

impl HDqpskModulator {
    pub fn new() -> Self {
        Self { phase: 0.0 }
    }

    /// Reset carrier phase to zero
    pub fn reset(&mut self) {
        self.phase = 0.0;
    }

    /// Encode one dibit (2 bits packed as `bits[1]<<1 | bits[0]`) to a complex symbol.
    /// The phase of the output symbol is the accumulated carrier phase after adding
    /// the dibit's differential phase shift.
    pub fn modulate_dibit(&mut self, dibit: u8) -> Qpsk {
        let delta_phi = DQPSK_PHASE_MAP[(dibit & 0x3) as usize];
        self.phase += delta_phi;
        // Wrap to (−π, +π]
        while self.phase > core::f64::consts::PI {
            self.phase -= 2.0 * core::f64::consts::PI;
        }
        while self.phase <= -core::f64::consts::PI {
            self.phase += 2.0 * core::f64::consts::PI;
        }
        Qpsk::new(self.phase.cos(), self.phase.sin())
    }

    /// Modulate a bit stream (pairs of bits → dibits → symbols).
    /// Input length must be even.
    pub fn modulate_bits(&mut self, bits: &[u8]) -> Vec<Qpsk> {
        let n_sym = bits.len() / 2;
        let mut symbols = Vec::with_capacity(n_sym);
        for i in 0..n_sym {
            let dibit = (bits[2 * i] << 1) | (bits[2 * i + 1] & 1);
            symbols.push(self.modulate_dibit(dibit));
        }
        symbols
    }
}

impl Default for HDqpskModulator {
    fn default() -> Self {
        Self::new()
    }
}

/// H-DQPSK demodulator — recover dibits from differential phase between consecutive symbols
#[derive(Debug, Clone)]
pub struct HDqpskDemodulator {
    /// Phase of the previous received symbol
    pub prev_phase: f64,
}

impl HDqpskDemodulator {
    pub fn new() -> Self {
        Self { prev_phase: 0.0 }
    }

    /// Demodulate one symbol to a dibit by measuring differential phase
    pub fn demodulate_symbol(&mut self, sym: Qpsk) -> u8 {
        let current_phase = sym.phase();
        let mut delta = current_phase - self.prev_phase;
        // Wrap delta to (−π, +π]
        while delta > core::f64::consts::PI {
            delta -= 2.0 * core::f64::consts::PI;
        }
        while delta <= -core::f64::consts::PI {
            delta += 2.0 * core::f64::consts::PI;
        }
        self.prev_phase = current_phase;
        nearest_dqpsk_dibit(delta)
    }

    /// Demodulate a symbol stream to a bit stream (2 bits per symbol)
    pub fn demodulate_symbols(&mut self, symbols: &[Qpsk]) -> Vec<u8> {
        let mut bits = Vec::with_capacity(symbols.len() * 2);
        for &sym in symbols {
            let dibit = self.demodulate_symbol(sym);
            bits.push((dibit >> 1) & 1);
            bits.push(dibit & 1);
        }
        bits
    }
}

impl Default for HDqpskDemodulator {
    fn default() -> Self {
        Self::new()
    }
}

/// Find the nearest π/4-DQPSK dibit from a measured differential phase
fn nearest_dqpsk_dibit(delta_phase: f64) -> u8 {
    let mut best_dibit = 0u8;
    let mut best_dist = f64::MAX;
    for (i, &mapped) in DQPSK_PHASE_MAP.iter().enumerate() {
        let mut diff = (delta_phase - mapped).abs();
        if diff > core::f64::consts::PI {
            diff = 2.0 * core::f64::consts::PI - diff;
        }
        if diff < best_dist {
            best_dist = diff;
            best_dibit = i as u8;
        }
    }
    best_dibit
}

/// Gray-code encode a 2-bit dibit
pub fn gray_encode_dibit(b: u8) -> u8 {
    (b >> 1) ^ b
}

/// Gray-code decode a 2-bit Gray-coded dibit back to binary
pub fn gray_decode_dibit(g: u8) -> u8 {
    let b1 = (g >> 1) & 1;
    let b0 = g ^ b1;
    (b1 << 1) | (b0 & 1)
}

// ============================================================
// TDMA Frame Structure
// ============================================================

/// TDMA slot type identifiers per TIA-102.BBAC
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SlotType {
    /// Inbound Subscriber Channel (uplink voice)
    ISCH,
    /// Outbound Subscriber Channel (downlink voice)
    OSCH,
    /// Fast Associated Control Channel
    FACCH,
    /// Slow Associated Control Channel (embedded in voice)
    SACCH,
    /// Idle / null slot
    Idle,
    /// Loopback data
    Loopback,
}

/// A single P25 Phase II TDMA slot (90ms, 540 symbols, 1080 payload bits)
#[derive(Debug, Clone)]
pub struct TdmaSlot {
    pub slot_id: u8,
    pub slot_type: SlotType,
    /// Payload bits (1080 bits)
    pub bits: Vec<u8>,
    /// Sync word bytes (48 bits = 6 bytes)
    pub sync_word: [u8; 6],
}

impl TdmaSlot {
    /// Create a new empty TDMA slot with the default voice sync word
    pub fn new(slot_id: u8, slot_type: SlotType) -> Self {
        Self {
            slot_id,
            slot_type,
            bits: vec![0u8; BITS_PER_SLOT],
            sync_word: sync_word_to_bytes(SLOT_SYNC_WORD),
        }
    }

    /// Serialize the slot to a flat bit stream: [48 sync bits | 1080 payload bits]
    pub fn to_bit_stream(&self) -> Vec<u8> {
        let mut stream = Vec::with_capacity(48 + self.bits.len());
        for byte in &self.sync_word {
            for bit in 0..8 {
                stream.push((byte >> (7 - bit)) & 1);
            }
        }
        stream.extend_from_slice(&self.bits);
        stream
    }

    /// Deserialize a flat bit stream (sync word + payload) back into a slot
    pub fn from_bit_stream(slot_id: u8, slot_type: SlotType, stream: &[u8]) -> Self {
        let sync_bits = &stream[..48.min(stream.len())];
        let mut sync_word = [0u8; 6];
        for (i, chunk) in sync_bits.chunks(8).enumerate() {
            if i >= 6 { break; }
            let mut byte = 0u8;
            for (j, &bit) in chunk.iter().enumerate() {
                byte |= bit << (7 - j);
            }
            sync_word[i] = byte;
        }
        let payload_bits = if stream.len() > 48 {
            stream[48..].to_vec()
        } else {
            vec![0u8; BITS_PER_SLOT]
        };
        Self {
            slot_id,
            slot_type,
            bits: payload_bits,
            sync_word,
        }
    }
}

/// Convert a 48-bit (low 48 bits of u64) sync word to a 6-byte big-endian array
fn sync_word_to_bytes(word: u64) -> [u8; 6] {
    let mut bytes = [0u8; 6];
    for i in 0..6 {
        bytes[5 - i] = ((word >> (i * 8)) & 0xFF) as u8;
    }
    bytes
}

/// A complete P25 Phase II TDMA frame (180ms, 2 slots)
#[derive(Debug, Clone)]
pub struct TdmaFrame {
    pub frame_number: u32,
    pub slots: [TdmaSlot; 2],
}

impl TdmaFrame {
    /// Construct a new TDMA frame with default slot configuration
    pub fn new(frame_number: u32) -> Self {
        Self {
            frame_number,
            slots: [
                TdmaSlot::new(0, SlotType::ISCH),
                TdmaSlot::new(1, SlotType::OSCH),
            ],
        }
    }

    /// Get a reference to a slot by index (0 or 1)
    pub fn get_slot(&self, index: usize) -> Option<&TdmaSlot> {
        self.slots.get(index)
    }

    /// Serialize frame to flat bit stream with 4-symbol (8-bit) guard intervals
    pub fn to_bit_stream(&self) -> Vec<u8> {
        let mut stream = Vec::new();
        for slot in &self.slots {
            stream.extend(slot.to_bit_stream());
            // 4-symbol guard interval represented as 8 zero bits
            stream.extend_from_slice(&[0u8; 8]);
        }
        stream
    }
}

/// A P25 Phase II superframe consisting of 4 TDMA frames (720ms total)
#[derive(Debug, Clone)]
pub struct Superframe {
    pub superframe_number: u32,
    pub frames: Vec<TdmaFrame>,
}

impl Superframe {
    /// Create a new superframe with 4 empty frames
    pub fn new(superframe_number: u32) -> Self {
        let base_fn = superframe_number * FRAMES_PER_SUPERFRAME as u32;
        let frames = (0..FRAMES_PER_SUPERFRAME)
            .map(|i| TdmaFrame::new(base_fn + i as u32))
            .collect();
        Self { superframe_number, frames }
    }

    /// Validate the superframe has the correct structure
    pub fn validate(&self) -> bool {
        self.frames.len() == FRAMES_PER_SUPERFRAME
            && self.frames.iter().all(|f| f.slots.len() == SLOTS_PER_FRAME)
    }

    /// Count the total number of bits in the serialized superframe
    pub fn total_bits(&self) -> usize {
        self.frames.iter().map(|f| f.to_bit_stream().len()).sum()
    }
}

// ============================================================
// AMBE+2 Vocoder Frame Framing
// ============================================================

/// AMBE+2 voice frame structure — 88 bits per 20ms voice period
///
/// Bit layout (TIA-102.BBAC §7):
/// - bits  0..48 : 49 vocoder (speech coding) bits at 2450 bps net
/// - bits 49..87 : 39 FEC / channel-coding bits
#[derive(Debug, Clone)]
pub struct Ambe2Frame {
    /// 49 vocoder bits
    pub voice_bits: [u8; 49],
    /// 39 FEC protection bits
    pub fec_bits: [u8; 39],
}

impl Ambe2Frame {
    pub fn new() -> Self {
        Self {
            voice_bits: [0u8; 49],
            fec_bits: [0u8; 39],
        }
    }

    /// Assemble the 88-bit frame (voice bits first, then FEC bits)
    pub fn to_bits(&self) -> [u8; 88] {
        let mut out = [0u8; 88];
        out[..49].copy_from_slice(&self.voice_bits);
        out[49..88].copy_from_slice(&self.fec_bits);
        out
    }

    /// Disassemble an 88-bit frame into voice and FEC parts
    pub fn from_bits(bits: &[u8; 88]) -> Self {
        let mut frame = Self::new();
        frame.voice_bits.copy_from_slice(&bits[..49]);
        frame.fec_bits.copy_from_slice(&bits[49..88]);
        frame
    }

    /// Compute simple XOR parity over 13-bit voice groups → fill first 3 FEC bits
    ///
    /// Real AMBE+2 uses rate-3/4 Golay/RS protection per TIA-102.BBAC §7;
    /// this simplified version is used for frame integrity checking only.
    pub fn compute_fec(&mut self) {
        for i in 0..3 {
            let start = i * 13;
            let end = (start + 13).min(49);
            let parity = self.voice_bits[start..end]
                .iter()
                .fold(0u8, |acc, &b| acc ^ b);
            self.fec_bits[i] = parity;
        }
    }

    /// Check the first 3 FEC parity bits (returns `true` if consistent)
    pub fn check_fec(&self) -> bool {
        for i in 0..3 {
            let start = i * 13;
            let end = (start + 13).min(49);
            let parity = self.voice_bits[start..end]
                .iter()
                .fold(0u8, |acc, &b| acc ^ b);
            if parity != self.fec_bits[i] {
                return false;
            }
        }
        true
    }
}

impl Default for Ambe2Frame {
    fn default() -> Self {
        Self::new()
    }
}

/// Voice slot payload: 4 AMBE+2 frames packed into one TDMA slot payload
#[derive(Debug, Clone)]
pub struct VoiceSlotPayload {
    pub frames: [Ambe2Frame; 4],
    /// Sequence indicator (frame counter mod 256)
    pub sequence: u8,
}

impl VoiceSlotPayload {
    pub fn new(sequence: u8) -> Self {
        Self {
            frames: [
                Ambe2Frame::new(),
                Ambe2Frame::new(),
                Ambe2Frame::new(),
                Ambe2Frame::new(),
            ],
            sequence,
        }
    }

    /// Assemble all 4 AMBE+2 frames into 352 bits (4 × 88)
    pub fn to_bits(&self) -> Vec<u8> {
        let mut bits = Vec::with_capacity(4 * 88);
        for frame in &self.frames {
            bits.extend_from_slice(&frame.to_bits());
        }
        bits
    }

    /// Disassemble 352 bits into 4 AMBE+2 frames
    pub fn from_bits(bits: &[u8], sequence: u8) -> Self {
        let mut payload = Self::new(sequence);
        for (i, frame) in payload.frames.iter_mut().enumerate() {
            let start = i * 88;
            let end = start + 88;
            if end <= bits.len() {
                let arr: [u8; 88] = bits[start..end].try_into().unwrap_or([0u8; 88]);
                *frame = Ambe2Frame::from_bits(&arr);
            }
        }
        payload
    }
}

// ============================================================
// Trellis Coded Modulation (TCM) — Rate 3/4
// ============================================================

/// Rate 3/4 TCM encoder (TIA-102.BAAL)
///
/// Encodes 3 input bits into 4 output bits:
/// - 1 input bit feeds a rate-1/2 convolutional encoder (K=3, generators G1=0b111, G2=0b101)
/// - 2 input bits pass through uncoded
/// - 2 convolutional output bits + 2 pass-through bits form the 4-bit codeword
#[derive(Debug, Clone)]
pub struct TcmEncoder {
    /// 2-bit shift register state
    state: u8,
    g1: u8,
    g2: u8,
}

impl TcmEncoder {
    pub fn new() -> Self {
        Self { state: 0, g1: 0b111, g2: 0b101 }
    }

    pub fn reset(&mut self) {
        self.state = 0;
    }

    /// Encode 3 input bits → 4 output bits
    ///
    /// Input `bits[0]` is the coded MSB fed to the convolutional encoder.
    /// `bits[1]` and `bits[2]` are uncoded pass-through bits.
    pub fn encode_tribits(&mut self, bits: &[u8; 3]) -> [u8; 4] {
        let u0 = bits[0];
        let u1 = bits[1];
        let u2 = bits[2];
        let sr = ((self.state << 1) | u0) & 0x03;
        let c3 = popcount_u8(sr & self.g1) % 2;
        let c0 = popcount_u8(sr & self.g2) % 2;
        self.state = sr;
        [c3, u1, u2, c0]
    }

    /// Encode a stream of bits in groups of 3 (input length must be multiple of 3)
    pub fn encode_bits(&mut self, input: &[u8]) -> Vec<u8> {
        let groups = input.len() / 3;
        let mut output = Vec::with_capacity(groups * 4);
        for i in 0..groups {
            let tribits = [input[3 * i], input[3 * i + 1], input[3 * i + 2]];
            let coded = self.encode_tribits(&tribits);
            output.extend_from_slice(&coded);
        }
        output
    }
}

impl Default for TcmEncoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Viterbi decoder for rate 3/4 TCM
#[derive(Debug, Clone)]
pub struct TcmDecoder {
    num_states: usize,
    g1: u8,
    g2: u8,
}

impl TcmDecoder {
    pub fn new() -> Self {
        Self { num_states: 4, g1: 0b111, g2: 0b101 }
    }

    /// Compute branch metric (Hamming distance on coded bits only)
    fn branch_metric(received: &[u8; 4], state: u8, input_bit: u8) -> u32 {
        let sr = ((state << 1) | input_bit) & 0x03;
        let c3 = popcount_u8(sr & 0b111) % 2;
        let c0 = popcount_u8(sr & 0b101) % 2;
        (received[0] ^ c3) as u32 + (received[3] ^ c0) as u32
    }

    /// Viterbi-decode a stream of 4-bit codewords back to 3-bit tribits
    pub fn decode_bits(&self, coded: &[u8]) -> Vec<u8> {
        let num_groups = coded.len() / 4;
        if num_groups == 0 {
            return vec![];
        }

        let mut path_metrics = vec![u32::MAX / 2; self.num_states];
        path_metrics[0] = 0;

        // survivors[t][state] = (prev_state, input_bit)
        let mut survivors: Vec<Vec<(u8, u8)>> =
            vec![vec![(0u8, 0u8); self.num_states]; num_groups];

        for t in 0..num_groups {
            let start = t * 4;
            let received: [u8; 4] = [
                coded[start], coded[start + 1], coded[start + 2], coded[start + 3],
            ];
            let mut new_metrics = vec![u32::MAX / 2; self.num_states];

            for state in 0..self.num_states {
                for input_bit in 0..2u8 {
                    let next_state = ((state << 1) | input_bit as usize) & 0x03;
                    let bm = Self::branch_metric(&received, state as u8, input_bit);
                    let candidate = path_metrics[state].saturating_add(bm);
                    if candidate < new_metrics[next_state] {
                        new_metrics[next_state] = candidate;
                        survivors[t][next_state] = (state as u8, input_bit);
                    }
                }
            }
            path_metrics = new_metrics;
        }

        // Traceback from the lowest-metric final state
        let final_state = path_metrics
            .iter()
            .enumerate()
            .min_by_key(|(_, &m)| m)
            .map(|(s, _)| s)
            .unwrap_or(0);

        let mut input_bits = vec![0u8; num_groups];
        let mut state = final_state;
        for t in (0..num_groups).rev() {
            let (prev_state, ib) = survivors[t][state];
            input_bits[t] = ib;
            state = prev_state as usize;
        }

        let mut decoded = Vec::with_capacity(num_groups * 3);
        for (t, &ib) in input_bits.iter().enumerate() {
            let start = t * 4;
            decoded.push(ib);
            decoded.push(coded[start + 1]);  // pass-through u1
            decoded.push(coded[start + 2]);  // pass-through u2
        }
        decoded
    }
}

impl Default for TcmDecoder {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// Golay(24,12,8) Encoder/Decoder
// ============================================================

/// Generator polynomial for the (23,12) Golay code:
/// g(x) = x^11 + x^10 + x^6 + x^5 + x^4 + x^2 + 1 = 0xC75
pub const GOLAY23_POLY: u32 = 0xC75;

/// Divide `data` (a 23-bit value with message in bits 22..11 and parity in 10..0)
/// by the degree-11 Golay polynomial. Returns the 11-bit remainder.
fn golay23_poly_div(data: u32) -> u32 {
    let mut r = data;
    for i in (0..12).rev() {
        if (r >> (i + 11)) & 1 != 0 {
            r ^= GOLAY23_POLY << i;
        }
    }
    r & 0x7FF
}

/// Encode a 12-bit message into a 23-bit (23,12) Golay codeword.
/// The message occupies bits 22..11 and the parity occupies bits 10..0.
fn golay23_encode(msg: u16) -> u32 {
    let shifted = (msg as u32) << 11; // message in high 12 bits of 23-bit word
    let parity = golay23_poly_div(shifted);
    shifted | parity
}

/// Encode a 12-bit message into a 24-bit extended Golay(24,12,8) codeword.
/// Bit 0 of the result is an overall parity bit (even parity over all 24 bits).
pub fn golay_encode(msg: u16) -> u32 {
    let cw23 = golay23_encode(msg);
    let overall_parity = popcount_u32(cw23) % 2;
    (cw23 << 1) | overall_parity
}

/// Decode a 24-bit Golay codeword, correcting up to 3 bit errors.
/// Returns the corrected 12-bit message, or `None` if uncorrectable.
pub fn golay_decode(received: u32) -> Option<u16> {
    // Strip the overall parity bit (LSB) to get the 23-bit word
    let cw23 = received >> 1;
    let s = golay23_poly_div(cw23);

    // No errors
    if s == 0 {
        return Some(((cw23 >> 11) & 0xFFF) as u16);
    }

    let sw = popcount_u32(s);

    // Weight ≤ 3 means all errors are in the parity part (low 11 bits);
    // message bits (high 12) are error-free.
    if sw <= 3 {
        return Some(((cw23 >> 11) & 0xFFF) as u16);
    }

    // One error in the message part: try flipping each of the 12 message bits
    for i in 0..12u32 {
        let e = 1u32 << (i + 11);
        let ts = golay23_poly_div(cw23 ^ e);
        if popcount_u32(ts) <= 2 {
            let corrected = cw23 ^ e ^ ts;
            return Some(((corrected >> 11) & 0xFFF) as u16);
        }
    }

    // One error in the parity part: try flipping each of the 11 parity bits
    for i in 0..11u32 {
        let e = 1u32 << i;
        let ts = golay23_poly_div(cw23 ^ e);
        if popcount_u32(ts) <= 2 {
            // Parity error doesn't affect the message; fix the parity via syndrome
            let corrected = cw23 ^ ts;
            return Some(((corrected >> 11) & 0xFFF) as u16);
        }
    }

    // Two errors in the message part: try all combinations
    for i in 0..12u32 {
        for j in (i + 1)..12u32 {
            let e = (1u32 << (i + 11)) | (1u32 << (j + 11));
            let ts = golay23_poly_div(cw23 ^ e);
            if popcount_u32(ts) <= 1 {
                let corrected = cw23 ^ e ^ ts;
                return Some(((corrected >> 11) & 0xFFF) as u16);
            }
        }
    }

    None // Uncorrectable
}

// ============================================================
// Reed-Solomon(36,20,17) over GF(2^6)
// ============================================================

/// GF(2^6) primitive polynomial: x^6 + x + 1 = 0x43
const GF6_PRIM: u8 = 0x43;

/// GF(2^6) tables for fast multiplication and division
struct Gf6 {
    /// alog[i] = α^i  (antilog / exponent table)
    alog: [u8; 64],
    /// log[a] = i such that α^i = a  (log[0] = 0xFF sentinel)
    log: [u8; 64],
}

impl Gf6 {
    fn new() -> Self {
        let mut alog = [0u8; 64];
        let mut log = [0xFFu8; 64];
        let mut x = 1u8;
        for i in 0..63usize {
            alog[i] = x;
            log[x as usize] = i as u8;
            // Multiply by α (= 2) in GF(2^6)
            let carry = x & 0x20;
            x = (x << 1) & 0x3F;
            if carry != 0 {
                x ^= GF6_PRIM & 0x3F; // x^6 ≡ x + 1, so XOR with low 6 bits of poly
            }
        }
        Gf6 { alog, log }
    }

    #[inline]
    fn mul(&self, a: u8, b: u8) -> u8 {
        if a == 0 || b == 0 { return 0; }
        let la = self.log[a as usize] as usize;
        let lb = self.log[b as usize] as usize;
        self.alog[(la + lb) % 63]
    }

    #[inline]
    fn div(&self, a: u8, b: u8) -> u8 {
        if a == 0 { return 0; }
        debug_assert!(b != 0, "Division by zero in GF(2^6)");
        let la = self.log[a as usize] as usize;
        let lb = self.log[b as usize] as usize;
        self.alog[(la + 63 - lb) % 63]
    }
}

/// Build the RS(36,20) generator polynomial g(x) = ∏_{i=0}^{15} (x + α^i) over GF(2^6).
///
/// Returns coefficients in ascending degree order: `poly[k]` = coefficient of x^k.
/// `poly[16]` = 1 (monic, leading coefficient).
fn rs_build_generator(gf: &Gf6) -> Vec<u8> {
    let mut poly = vec![1u8]; // start as the constant polynomial 1
    for i in 0..16usize {
        let root = gf.alog[i];
        // Multiply poly(x) by (x + root) = (x + α^i)
        let prev_len = poly.len();
        let mut new_poly = vec![0u8; prev_len + 1];
        for (j, &c) in poly.iter().enumerate() {
            // Coefficient of x^(j+1) gets +c
            new_poly[j + 1] ^= c;
            // Coefficient of x^j gets + c * root
            new_poly[j] ^= gf.mul(c, root);
        }
        poly = new_poly;
    }
    // poly[16] should be 1
    poly
}

/// Reed-Solomon(36,20,17) encoder over GF(2^6)
///
/// Each GF(2^6) symbol is a 6-bit value (low 6 bits used).
pub struct RsEncoder {
    gf: Gf6,
    /// Generator polynomial in descending degree order for long division:
    /// `gen_rev[0]` = leading coefficient (= 1), `gen_rev[k]` = coeff of x^(16-k)
    gen_rev: Vec<u8>,
}

impl RsEncoder {
    pub fn new() -> Self {
        let gf = Gf6::new();
        let gen = rs_build_generator(&gf);
        // Reverse so gen_rev[0] = gen[16] (leading coeff), gen_rev[16] = gen[0]
        let gen_rev: Vec<u8> = gen.iter().rev().cloned().collect();
        Self { gf, gen_rev }
    }

    /// Encode k=20 message symbols (6-bit each) to n=36 systematic codeword.
    ///
    /// Output layout: [message[0..20] | parity[0..16]]
    pub fn encode(&self, message: &[u8]) -> Vec<u8> {
        assert_eq!(message.len(), RS_K);
        let msg: Vec<u8> = message.iter().map(|&b| b & 0x3F).collect();

        // Polynomial long division: divide m(x)·x^16 by g(x),
        // where m(x) = msg[0]·x^19 + ... + msg[19] (big-endian coefficient order)
        // and g(x) = gen_rev[0]·x^16 + ... + gen_rev[16] (gen_rev[0]=1)
        let mut buf: Vec<u8> = Vec::with_capacity(RS_N);
        buf.extend_from_slice(&msg);
        buf.extend(core::iter::repeat(0u8).take(16));

        for i in 0..RS_K {
            let lc = buf[i];
            if lc == 0 { continue; }
            // Subtract lc × g(x) shifted to align with buf[i]
            for j in 0..=16usize {
                buf[i + j] ^= self.gf.mul(lc, self.gen_rev[j]);
            }
        }
        // msg part is now unchanged (systematic); parity is buf[20..36]
        let mut codeword = msg;
        codeword.extend_from_slice(&buf[RS_K..RS_N]);
        codeword
    }
}

impl Default for RsEncoder {
    fn default() -> Self {
        Self::new()
    }
}

/// Reed-Solomon(36,20,17) decoder using Berlekamp-Massey + Chien + Forney
pub struct RsDecoder {
    gf: Gf6,
    gen_rev: Vec<u8>,
}

impl RsDecoder {
    pub fn new() -> Self {
        let gf = Gf6::new();
        let gen = rs_build_generator(&gf);
        let gen_rev: Vec<u8> = gen.iter().rev().cloned().collect();
        Self { gf, gen_rev }
    }

    /// Compute 16 syndromes: S_i = r(α^i) for i = 0..15, using Horner's method.
    ///
    /// `received` is stored big-endian: `received[0]` = coefficient of x^35.
    fn syndromes(&self, received: &[u8]) -> Vec<u8> {
        (0..16)
            .map(|i| {
                let ai = self.gf.alog[i];
                let mut s = 0u8;
                for &c in received {
                    s = self.gf.mul(s, ai) ^ c;
                }
                s
            })
            .collect()
    }

    /// Berlekamp-Massey algorithm to find the error-locator polynomial σ(x).
    fn berlekamp_massey(&self, syn: &[u8]) -> Vec<u8> {
        let n = syn.len();
        let mut sigma = vec![1u8];     // error locator (starts as 1)
        let mut prev = vec![1u8];      // "b" register
        let mut l = 0usize;
        let mut m = 1usize;
        let mut b_scalar = 1u8;        // discrepancy at last update ("bb")

        for k in 0..n {
            // Compute discrepancy Δ_k = S_k + Σ_{j=1}^{L} σ_j * S_{k-j}
            let mut delta = syn[k];
            for j in 1..sigma.len() {
                if k >= j {
                    delta ^= self.gf.mul(sigma[j], syn[k - j]);
                }
            }
            if delta == 0 {
                m += 1;
                continue;
            }
            let t = sigma.clone();
            let scale = self.gf.div(delta, b_scalar);
            // σ(x) ← σ(x) + (Δ/b) * x^m * prev(x)
            let new_len = sigma.len().max(prev.len() + m);
            sigma.resize(new_len, 0);
            for i in 0..prev.len() {
                sigma[i + m] ^= self.gf.mul(scale, prev[i]);
            }
            if 2 * l <= k {
                l = k + 1 - l;
                prev = t;
                b_scalar = delta;
                m = 1;
            } else {
                m += 1;
            }
        }
        sigma
    }

    /// Chien search: find positions where σ(α^{−i}) = 0 (i.e., error locations).
    fn chien_search(&self, sigma: &[u8]) -> Vec<usize> {
        // Big-endian convention: cw[k] is the coefficient of x^{n-1-k} = x^{35-k}.
        // An error at position k corresponds to error locator root X_k = α^{35-k}.
        // σ(X_k^{-1}) = 0 where X_k^{-1} = α^{k-35} = α^{(k+28) mod 63}.
        let mut roots = Vec::new();
        for k in 0..RS_N {
            let exp = (k + 28) % 63; // (k - 35 + 63) mod 63 = (k + 28) mod 63
            let eval_point = self.gf.alog[exp];
            let mut val = 0u8;
            let mut power = 1u8;
            for &coef in sigma {
                val ^= self.gf.mul(coef, power);
                power = self.gf.mul(power, eval_point);
            }
            if val == 0 {
                roots.push(k);
            }
        }
        roots
    }

    /// Forney algorithm: compute error magnitudes given error positions.
    fn forney(&self, syn: &[u8], sigma: &[u8], positions: &[usize]) -> Vec<u8> {
        let n2 = syn.len(); // = 16
        // Ω(x) = S(x) · σ(x) mod x^16  (error evaluator polynomial)
        let mut omega = vec![0u8; n2];
        for i in 0..n2 {
            for j in 0..sigma.len().min(i + 1) {
                omega[i] ^= self.gf.mul(syn[i - j], sigma[j]);
            }
        }

        positions.iter().map(|&pos| {
            // X_k = α^{35-pos}, X_k^{-1} = α^{(pos+28) mod 63}
            let exp_inv = (pos + 28) % 63;
            let alpha_inv = self.gf.alog[exp_inv];

            // Evaluate Ω(α^{−pos})
            let mut omega_val = 0u8;
            let mut power = 1u8;
            for &w in &omega {
                omega_val ^= self.gf.mul(w, power);
                power = self.gf.mul(power, alpha_inv);
            }

            // Evaluate σ'(α^{−pos}) — formal derivative (odd-indexed terms only in GF(2))
            let mut sigma_prime = 0u8;
            let mut ap = 1u8;
            for (j, &s) in sigma.iter().enumerate() {
                if j % 2 == 1 {
                    sigma_prime ^= self.gf.mul(s, ap);
                }
                ap = self.gf.mul(ap, alpha_inv);
            }

            if sigma_prime == 0 {
                0
            } else {
                // Error magnitude = Ω(X_k^{-1}) / σ'(X_k^{-1})
                // (No X_k factor in the big-endian / Blahut convention used here)
                self.gf.div(omega_val, sigma_prime)
            }
        }).collect()
    }

    /// Decode a received 36-symbol codeword.
    /// Returns the corrected 20-symbol message, or `None` if uncorrectable.
    pub fn decode(&self, received: &[u8]) -> Option<Vec<u8>> {
        let r: Vec<u8> = received.iter().map(|&b| b & 0x3F).collect();
        let syn = self.syndromes(&r);

        // All-zero syndromes → no errors detected
        if syn.iter().all(|&s| s == 0) {
            return Some(r[..RS_K].to_vec());
        }

        let sigma = self.berlekamp_massey(&syn);
        let num_errors = sigma.len() - 1;
        if num_errors > RS_T {
            return None;
        }

        let positions = self.chien_search(&sigma);
        if positions.len() != num_errors {
            return None;
        }

        let magnitudes = self.forney(&syn, &sigma, &positions);
        let mut corrected = r.clone();
        for (&pos, &mag) in positions.iter().zip(magnitudes.iter()) {
            if pos < corrected.len() {
                corrected[pos] ^= mag;
            }
        }

        // Verify correction by re-computing syndromes
        let check = self.syndromes(&corrected);
        if check.iter().all(|&s| s == 0) {
            Some(corrected[..RS_K].to_vec())
        } else {
            None
        }
    }
}

impl Default for RsDecoder {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// CRC-9 Computation
// ============================================================

/// Compute CRC-9 over a bit stream.
///
/// Polynomial: x^9 + x^7 + x^5 + x + 1 (0x1A3), init = 0x1FF.
/// Per TIA-102.BBAC §8.4.
pub fn crc9(bits: &[u8]) -> u16 {
    let mut crc: u16 = 0x1FF;
    for &bit in bits {
        let msb = ((crc >> 8) & 1) as u8;
        crc = (crc << 1) & 0x1FF;
        if (bit ^ msb) != 0 {
            crc ^= CRC9_POLY;
        }
    }
    crc & 0x1FF
}

/// Verify CRC-9: appends `crc_bits` to `data_bits` and checks the combined CRC is zero.
pub fn crc9_verify(data_bits: &[u8], crc_bits: &[u8; 9]) -> bool {
    let mut all: Vec<u8> = data_bits.to_vec();
    all.extend_from_slice(crc_bits);
    crc9(&all) == 0
}

/// Convert a 9-bit CRC value to a bit array (MSB first)
pub fn crc9_to_bits(crc: u16) -> [u8; 9] {
    let mut bits = [0u8; 9];
    for i in 0..9 {
        bits[8 - i] = ((crc >> i) & 1) as u8;
    }
    bits
}

// ============================================================
// PN Scrambling
// ============================================================

/// PN scrambler — 23-bit Galois LFSR
///
/// Polynomial: x^23 + x^18 + 1 per TIA-102.BBAL.
/// Used to whiten TDMA slot payloads before transmission.
#[derive(Debug, Clone)]
pub struct PnScrambler {
    state: u32,
    /// Feedback mask (bits corresponding to the non-x^23 terms of the polynomial)
    feedback_mask: u32,
}

impl PnScrambler {
    /// Create with the default P25 Phase II seed (all-ones)
    pub fn new() -> Self {
        Self {
            state: 0x7FFFFF,
            feedback_mask: (1 << 18) | 1, // x^18 + x^0 terms
        }
    }

    /// Create with a custom 23-bit seed
    pub fn with_seed(seed: u32) -> Self {
        Self {
            state: seed & 0x7FFFFF,
            feedback_mask: (1 << 18) | 1,
        }
    }

    /// Reset the LFSR to the all-ones initial state
    pub fn reset(&mut self) {
        self.state = 0x7FFFFF;
    }

    /// Generate one PN bit (output = LSB of state before shift)
    fn next_bit(&mut self) -> u8 {
        let bit = (self.state & 1) as u8;
        let feedback = self.state & 1;
        self.state >>= 1;
        if feedback != 0 {
            self.state ^= self.feedback_mask & 0x3FFFFF;
        }
        bit
    }

    /// XOR-scramble a mutable bit slice in-place
    pub fn scramble(&mut self, bits: &mut [u8]) {
        for b in bits.iter_mut() {
            *b ^= self.next_bit();
        }
    }

    /// Generate `n` PN bits
    pub fn generate(&mut self, n: usize) -> Vec<u8> {
        (0..n).map(|_| self.next_bit()).collect()
    }
}

impl Default for PnScrambler {
    fn default() -> Self {
        Self::new()
    }
}

// ============================================================
// Encryption Key Management
// ============================================================

/// Encryption algorithm identifiers per TIA-102.AACA
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AlgorithmId {
    /// No encryption (clear mode)
    Clear    = 0x80,
    /// DES in OFB mode
    DesOfb   = 0x81,
    /// Triple-DES in OFB mode
    TripleDes = 0x82,
    /// AES-128
    Aes128   = 0x84,
    /// AES-256
    Aes256   = 0x85,
    /// Vendor-defined
    Custom   = 0xFF,
}

impl TryFrom<u8> for AlgorithmId {
    type Error = &'static str;
    fn try_from(v: u8) -> Result<Self, Self::Error> {
        match v {
            0x80 => Ok(AlgorithmId::Clear),
            0x81 => Ok(AlgorithmId::DesOfb),
            0x82 => Ok(AlgorithmId::TripleDes),
            0x84 => Ok(AlgorithmId::Aes128),
            0x85 => Ok(AlgorithmId::Aes256),
            0xFF => Ok(AlgorithmId::Custom),
            _    => Err("Unknown algorithm ID"),
        }
    }
}

/// Message Indicator (MI) — 72-bit initialization vector for encryption
#[derive(Debug, Clone, PartialEq)]
pub struct MessageIndicator {
    pub bytes: [u8; 9],
}

impl MessageIndicator {
    pub fn new(bytes: [u8; 9]) -> Self {
        Self { bytes }
    }

    /// Construct an MI from a 64-bit seed and an 8-bit counter byte
    pub fn from_seed(seed: u64, counter: u8) -> Self {
        let mut bytes = [0u8; 9];
        bytes[0] = counter;
        for i in 0..8 {
            bytes[1 + i] = ((seed >> (56 - i * 8)) & 0xFF) as u8;
        }
        Self { bytes }
    }

    /// Increment the MI as a 72-bit big-endian counter (with carry propagation)
    pub fn increment(&mut self) {
        for i in (0..9).rev() {
            self.bytes[i] = self.bytes[i].wrapping_add(1);
            if self.bytes[i] != 0 {
                break;
            }
        }
    }

    /// Convert to a u128 for numeric comparison (only low 72 bits are meaningful)
    pub fn to_u128(&self) -> u128 {
        self.bytes.iter().fold(0u128, |acc, &b| (acc << 8) | b as u128)
    }
}

/// Encryption context for a P25 Phase II link
#[derive(Debug, Clone)]
pub struct EncryptionContext {
    pub algorithm_id: AlgorithmId,
    pub key_id: u16,
    pub mi: MessageIndicator,
    key: [u8; 32],
    key_len: usize,
}

impl EncryptionContext {
    /// Create a clear-mode (unencrypted) context
    pub fn clear() -> Self {
        Self {
            algorithm_id: AlgorithmId::Clear,
            key_id: 0,
            mi: MessageIndicator::new([0u8; 9]),
            key: [0u8; 32],
            key_len: 0,
        }
    }

    /// Create an AES-256 encryption context
    pub fn aes256(key_id: u16, key: &[u8; 32], mi: MessageIndicator) -> Self {
        let mut k = [0u8; 32];
        k.copy_from_slice(key);
        Self { algorithm_id: AlgorithmId::Aes256, key_id, mi, key: k, key_len: 32 }
    }

    /// Create a DES-OFB encryption context (key = first 8 bytes)
    pub fn des_ofb(key_id: u16, key: &[u8; 8], mi: MessageIndicator) -> Self {
        let mut k = [0u8; 32];
        k[..8].copy_from_slice(key);
        Self { algorithm_id: AlgorithmId::DesOfb, key_id, mi, key: k, key_len: 8 }
    }

    /// Serialize encryption parameters for Link Control / TSBK (12 bytes)
    pub fn to_esn_bytes(&self) -> [u8; 12] {
        let mut esn = [0u8; 12];
        esn[0] = self.algorithm_id as u8;
        esn[1] = (self.key_id >> 8) as u8;
        esn[2] = (self.key_id & 0xFF) as u8;
        esn[3..12].copy_from_slice(&self.mi.bytes);
        esn
    }

    /// Generate one keystream byte at the given offset (placeholder; XOR-based)
    pub fn keystream_byte(&self, offset: usize) -> u8 {
        let mi_byte = self.mi.bytes[offset % 9];
        let key_byte = self.key[offset % self.key_len.max(1)];
        mi_byte ^ key_byte ^ (offset as u8)
    }
}

// ============================================================
// Sync Pattern Detection
// ============================================================

/// Type of synchronization pattern detected
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SyncType {
    VoiceSlot,
    Facch,
    Unknown,
}

/// Result of a sync pattern search
#[derive(Debug, Clone)]
pub struct SyncResult {
    pub found: bool,
    pub position: usize,
    pub errors: u32,
    pub sync_type: SyncType,
}

/// P25 Phase II sync-word detector (searches for 48-bit patterns in a bit stream)
pub struct SyncDetector {
    /// Maximum Hamming distance tolerated for a sync word match
    pub max_errors: u32,
}

impl SyncDetector {
    pub fn new(max_errors: u32) -> Self {
        Self { max_errors }
    }

    /// Search for slot sync or FACCH sync patterns in a bit stream
    pub fn find_sync(&self, bits: &[u8]) -> SyncResult {
        let voice_ref = u64_to_bits_48(SLOT_SYNC_WORD);
        let facch_ref  = u64_to_bits_48(FACCH_SYNC_WORD);

        for pos in 0..bits.len().saturating_sub(47) {
            let slice = &bits[pos..pos + 48];

            let ev = hamming_distance_48(slice, &voice_ref);
            if ev <= self.max_errors {
                return SyncResult { found: true, position: pos, errors: ev, sync_type: SyncType::VoiceSlot };
            }

            let ef = hamming_distance_48(slice, &facch_ref);
            if ef <= self.max_errors {
                return SyncResult { found: true, position: pos, errors: ef, sync_type: SyncType::Facch };
            }
        }
        SyncResult { found: false, position: 0, errors: 48, sync_type: SyncType::Unknown }
    }
}

/// Convert the low 48 bits of a u64 to a 48-element bit array (MSB first)
fn u64_to_bits_48(word: u64) -> [u8; 48] {
    let mut bits = [0u8; 48];
    for i in 0..48 {
        bits[47 - i] = ((word >> i) & 1) as u8;
    }
    bits
}

/// Hamming distance between a slice and a reference 48-bit array
fn hamming_distance_48(a: &[u8], b: &[u8; 48]) -> u32 {
    a.iter().zip(b.iter()).map(|(&x, &y)| (x ^ y) as u32).sum()
}

// ============================================================
// TSBK — Trunking Signaling Block
// ============================================================

/// TSBK opcode values per TIA-102.AABC Table 9
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum TsbkOpcode {
    GroupVoiceChannelGrant   = 0x20,
    UnitVoiceChannelGrant    = 0x24,
    ChannelGrantUpdate       = 0x28,
    UnitRegistrationResponse = 0x2C,
    AuthChallenge            = 0x08,
    AuthResponse             = 0x09,
    QueuedResponse           = 0x33,
    DenyResponse             = 0x35,
    AckResponse              = 0x37,
    NetworkStatusBcast       = 0x3C,
    AdjacentSiteBcast        = 0x3E,
}

impl TryFrom<u8> for TsbkOpcode {
    type Error = ();
    fn try_from(v: u8) -> Result<Self, ()> {
        match v {
            0x20 => Ok(Self::GroupVoiceChannelGrant),
            0x24 => Ok(Self::UnitVoiceChannelGrant),
            0x28 => Ok(Self::ChannelGrantUpdate),
            0x2C => Ok(Self::UnitRegistrationResponse),
            0x08 => Ok(Self::AuthChallenge),
            0x09 => Ok(Self::AuthResponse),
            0x33 => Ok(Self::QueuedResponse),
            0x35 => Ok(Self::DenyResponse),
            0x37 => Ok(Self::AckResponse),
            0x3C => Ok(Self::NetworkStatusBcast),
            0x3E => Ok(Self::AdjacentSiteBcast),
            _    => Err(()),
        }
    }
}

/// Trunking Signaling Block (96 bits = 12 bytes, TIA-102.AABC §7.5)
#[derive(Debug, Clone)]
pub struct Tsbk {
    /// Last-block indicator: `true` if this is the final TSBK in a series
    pub last_block: bool,
    /// Protected/encrypted flag
    pub protected: bool,
    /// 6-bit opcode
    pub opcode: u8,
    /// Manufacturer Feature ID (0x00 = standard TIA opcode)
    pub mfid: u8,
    /// 64-bit argument payload
    pub args: [u8; 8],
    /// CRC-16/CCITT over first 10 bytes
    pub crc16: u16,
}

impl Tsbk {
    /// Construct a Group Voice Channel Grant TSBK
    pub fn group_voice_grant(channel: u16, group_id: u16, source_id: u32, encrypted: bool) -> Self {
        let mut args = [0u8; 8];
        args[0] = (channel >> 8) as u8 & 0x0F;
        args[1] = channel as u8;
        args[2] = (group_id >> 8) as u8;
        args[3] = group_id as u8;
        args[4] = (source_id >> 16) as u8;
        args[5] = (source_id >> 8) as u8;
        args[6] = source_id as u8;
        args[7] = if encrypted { 0x80 } else { 0 };

        let mut tsbk = Self {
            last_block: true,
            protected: encrypted,
            opcode: TsbkOpcode::GroupVoiceChannelGrant as u8,
            mfid: 0x00,
            args,
            crc16: 0,
        };
        tsbk.crc16 = tsbk.compute_crc();
        tsbk
    }

    /// Serialize the TSBK to 12 bytes
    pub fn to_bytes(&self) -> [u8; 12] {
        let mut bytes = [0u8; 12];
        bytes[0] = ((self.last_block as u8) << 7) | ((self.protected as u8) << 6) | (self.opcode & 0x3F);
        bytes[1] = self.mfid;
        bytes[2..10].copy_from_slice(&self.args);
        bytes[10] = (self.crc16 >> 8) as u8;
        bytes[11] = self.crc16 as u8;
        bytes
    }

    /// Deserialize from 12 bytes
    pub fn from_bytes(bytes: &[u8; 12]) -> Self {
        let mut args = [0u8; 8];
        args.copy_from_slice(&bytes[2..10]);
        let stored_crc = ((bytes[10] as u16) << 8) | bytes[11] as u16;
        Self {
            last_block: (bytes[0] >> 7) != 0,
            protected: ((bytes[0] >> 6) & 1) != 0,
            opcode: bytes[0] & 0x3F,
            mfid: bytes[1],
            args,
            crc16: stored_crc,
        }
    }

    /// Compute CRC-16/CCITT over first 10 bytes
    pub fn compute_crc(&self) -> u16 {
        let bytes = self.to_bytes();
        crc16_ccitt(&bytes[..10])
    }

    /// Return `true` if the stored CRC matches the computed CRC
    pub fn verify_crc(&self) -> bool {
        self.crc16 == self.compute_crc()
    }
}

/// CRC-16/CCITT (polynomial 0x1021, init 0xFFFF)
pub fn crc16_ccitt(data: &[u8]) -> u16 {
    let mut crc: u16 = 0xFFFF;
    for &byte in data {
        crc ^= (byte as u16) << 8;
        for _ in 0..8 {
            crc = if crc & 0x8000 != 0 { (crc << 1) ^ 0x1021 } else { crc << 1 };
        }
    }
    crc
}

// ============================================================
// Link Control (LC) Word
// ============================================================

/// Service Options byte for voice calls (TIA-102.AABC §7.7.2)
#[derive(Debug, Clone, Copy)]
pub struct ServiceOptions {
    pub emergency: bool,
    pub encrypted: bool,
    pub duplex: bool,
    pub packet_switched: bool,
    /// Priority level (3 bits, 0–7)
    pub priority: u8,
}

impl ServiceOptions {
    pub fn new() -> Self {
        Self { emergency: false, encrypted: false, duplex: false, packet_switched: false, priority: 0 }
    }

    pub fn to_byte(&self) -> u8 {
        ((self.emergency as u8) << 7)
            | ((self.encrypted as u8) << 6)
            | ((self.duplex as u8) << 5)
            | ((self.packet_switched as u8) << 4)
            | (self.priority & 0x07)
    }

    pub fn from_byte(b: u8) -> Self {
        Self {
            emergency: (b >> 7) != 0,
            encrypted: ((b >> 6) & 1) != 0,
            duplex: ((b >> 5) & 1) != 0,
            packet_switched: ((b >> 4) & 1) != 0,
            priority: b & 0x07,
        }
    }
}

impl Default for ServiceOptions {
    fn default() -> Self { Self::new() }
}

/// LC word opcode values (TIA-102.AABC Table 8)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum LcOpcode {
    GroupVoiceChannelUser       = 0x00,
    GroupVoiceChannelUpdate     = 0x02,
    UnitToUnitVoiceChannelUser  = 0x03,
    GroupAffiliationQuery       = 0x04,
    UnitToUnitAnswerRequest     = 0x05,
    TelephoneInterconnectVoice  = 0x08,
    EncryptionSyncParameters    = 0x09,
}

/// P25 Phase II Link Control word (72 bits = 9 bytes)
///
/// Structure: `[PF(1)|LCF(1)|LCOP(6) | MFID(8) | Args(48) | CRC8(8)]`
#[derive(Debug, Clone)]
pub struct LinkControl {
    pub protect: bool,
    pub implicit_format: bool,
    pub opcode: u8,
    pub mfid: u8,
    pub args: [u8; 6],
    pub crc8: u8,
}

impl LinkControl {
    /// Construct a Group Voice Channel User LC word
    pub fn group_voice_user(so: ServiceOptions, group_id: u16, source_id: u32) -> Self {
        let mut args = [0u8; 6];
        args[0] = so.to_byte();
        args[1] = (group_id >> 8) as u8;
        args[2] = group_id as u8;
        args[3] = (source_id >> 16) as u8;
        args[4] = (source_id >> 8) as u8;
        args[5] = source_id as u8;
        let mut lc = Self { protect: false, implicit_format: true, opcode: LcOpcode::GroupVoiceChannelUser as u8, mfid: 0x00, args, crc8: 0 };
        lc.crc8 = lc.compute_crc8();
        lc
    }

    /// Construct an Encryption Sync Parameters LC word
    pub fn encryption_sync(alg_id: AlgorithmId, key_id: u16, mi: &MessageIndicator) -> Self {
        let mut args = [0u8; 6];
        args[0] = alg_id as u8;
        args[1] = (key_id >> 8) as u8;
        args[2] = key_id as u8;
        args[3] = mi.bytes[0];
        args[4] = mi.bytes[1];
        args[5] = mi.bytes[2];
        let mut lc = Self { protect: false, implicit_format: true, opcode: LcOpcode::EncryptionSyncParameters as u8, mfid: 0x00, args, crc8: 0 };
        lc.crc8 = lc.compute_crc8();
        lc
    }

    /// Serialize to 9 bytes
    pub fn to_bytes(&self) -> [u8; 9] {
        let mut bytes = [0u8; 9];
        bytes[0] = ((self.protect as u8) << 7) | ((self.implicit_format as u8) << 6) | (self.opcode & 0x3F);
        bytes[1] = self.mfid;
        bytes[2..8].copy_from_slice(&self.args);
        bytes[8] = self.crc8;
        bytes
    }

    /// Deserialize from 9 bytes
    pub fn from_bytes(bytes: &[u8; 9]) -> Self {
        let mut args = [0u8; 6];
        args.copy_from_slice(&bytes[2..8]);
        Self { protect: (bytes[0] >> 7) != 0, implicit_format: ((bytes[0] >> 6) & 1) != 0, opcode: bytes[0] & 0x3F, mfid: bytes[1], args, crc8: bytes[8] }
    }

    /// Compute CRC-8 over the first 8 bytes using P25 polynomial (0x9B, init 0xFF)
    pub fn compute_crc8(&self) -> u8 {
        let bytes = self.to_bytes();
        crc8_p25(&bytes[..8])
    }

    /// Return `true` if stored CRC matches recomputed CRC
    pub fn verify_crc(&self) -> bool {
        self.crc8 == self.compute_crc8()
    }
}

/// CRC-8 per P25 Air Interface (polynomial 0x9B, init 0xFF)
pub fn crc8_p25(data: &[u8]) -> u8 {
    let mut crc: u8 = 0xFF;
    for &byte in data {
        crc ^= byte;
        for _ in 0..8 {
            crc = if crc & 0x80 != 0 { (crc << 1) ^ 0x9B } else { crc << 1 };
        }
    }
    crc
}

// ============================================================
// Utility Functions
// ============================================================

/// Count set bits in a u8 (popcount)
pub fn popcount_u8(x: u8) -> u8 {
    let mut n = x;
    n = (n & 0x55) + ((n >> 1) & 0x55);
    n = (n & 0x33) + ((n >> 2) & 0x33);
    n = (n & 0x0F) + ((n >> 4) & 0x0F);
    n
}

/// Count set bits in a u32 (popcount)
pub fn popcount_u32(x: u32) -> u32 {
    let mut n = x;
    n = (n & 0x55555555) + ((n >> 1) & 0x55555555);
    n = (n & 0x33333333) + ((n >> 2) & 0x33333333);
    n = (n & 0x0F0F0F0F) + ((n >> 4) & 0x0F0F0F0F);
    n.wrapping_mul(0x01010101) >> 24
}

/// Compute Hamming distance between two equal-length bit slices
pub fn hamming_distance(a: &[u8], b: &[u8]) -> u32 {
    a.iter().zip(b.iter()).map(|(&x, &y)| (x ^ y) as u32).sum()
}

/// Interleave bits: output[i] = input[(i * step) % n]
pub fn bit_interleave(src: &[u8], step: usize) -> Vec<u8> {
    let n = src.len();
    let mut out = vec![0u8; n];
    for i in 0..n {
        out[i] = src[(i * step) % n];
    }
    out
}

/// Deinterleave bits: the inverse of `bit_interleave`
pub fn bit_deinterleave(src: &[u8], step: usize) -> Vec<u8> {
    let n = src.len();
    let mut out = vec![0u8; n];
    for i in 0..n {
        out[(i * step) % n] = src[i];
    }
    out
}

// ============================================================
// Unit Tests
// ============================================================

#[cfg(test)]
mod tests {
    use super::*;

    // ---- H-DQPSK modulation ----

    #[test]
    fn test_hdqpsk_dibit_00_phase() {
        let mut m = HDqpskModulator::new();
        let sym = m.modulate_dibit(0b00);
        assert!((sym.phase() - core::f64::consts::FRAC_PI_4).abs() < 1e-10);
    }

    #[test]
    fn test_hdqpsk_dibit_01_phase() {
        let mut m = HDqpskModulator::new();
        let sym = m.modulate_dibit(0b01);
        assert!((sym.phase() - 3.0 * core::f64::consts::FRAC_PI_4).abs() < 1e-10);
    }

    #[test]
    fn test_hdqpsk_dibit_10_phase() {
        let mut m = HDqpskModulator::new();
        let sym = m.modulate_dibit(0b10);
        assert!((sym.phase() + core::f64::consts::FRAC_PI_4).abs() < 1e-10);
    }

    #[test]
    fn test_hdqpsk_dibit_11_phase() {
        let mut m = HDqpskModulator::new();
        let sym = m.modulate_dibit(0b11);
        assert!((sym.phase() + 3.0 * core::f64::consts::FRAC_PI_4).abs() < 1e-10);
    }

    #[test]
    fn test_hdqpsk_symbol_unit_magnitude() {
        let mut m = HDqpskModulator::new();
        for dibit in 0..4u8 {
            let sym = m.modulate_dibit(dibit);
            assert!((sym.magnitude() - 1.0).abs() < 1e-10, "magnitude={}", sym.magnitude());
        }
    }

    #[test]
    fn test_hdqpsk_modulate_demodulate_roundtrip() {
        let bits: Vec<u8> = vec![1, 0, 0, 1, 1, 1, 0, 0, 1, 0, 0, 0];
        let mut m = HDqpskModulator::new();
        let mut d = HDqpskDemodulator::new();
        let syms = m.modulate_bits(&bits);
        let rec  = d.demodulate_symbols(&syms);
        assert_eq!(rec, bits);
    }

    #[test]
    fn test_hdqpsk_accumulates_phase() {
        // Two consecutive +π/4 steps → accumulated phase = π/2
        let mut m = HDqpskModulator::new();
        m.modulate_dibit(0b00);
        let sym = m.modulate_dibit(0b00);
        assert!((sym.phase() - core::f64::consts::FRAC_PI_2).abs() < 1e-10);
    }

    #[test]
    fn test_hdqpsk_all_transitions_roundtrip() {
        let dibits_in: Vec<u8> = vec![0, 1, 2, 3, 0, 1, 2, 3];
        let mut bits_in = Vec::new();
        for &d in &dibits_in {
            bits_in.push((d >> 1) & 1);
            bits_in.push(d & 1);
        }
        let mut m = HDqpskModulator::new();
        let mut d = HDqpskDemodulator::new();
        let syms = m.modulate_bits(&bits_in);
        let bits_out = d.demodulate_symbols(&syms);
        assert_eq!(bits_in, bits_out);
    }

    // ---- Gray coding ----

    #[test]
    fn test_gray_encode_known_values() {
        assert_eq!(gray_encode_dibit(0), 0);
        assert_eq!(gray_encode_dibit(1), 1);
        assert_eq!(gray_encode_dibit(2), 3);
        assert_eq!(gray_encode_dibit(3), 2);
    }

    #[test]
    fn test_gray_decode_roundtrip() {
        for v in 0..4u8 {
            assert_eq!(gray_decode_dibit(gray_encode_dibit(v)) & 0x3, v);
        }
    }

    // ---- TDMA frame structure ----

    #[test]
    fn test_tdma_slot_bit_stream_length() {
        let slot = TdmaSlot::new(0, SlotType::ISCH);
        let stream = slot.to_bit_stream();
        assert_eq!(stream.len(), 48 + BITS_PER_SLOT);
    }

    #[test]
    fn test_slot_from_bit_stream_roundtrip() {
        let mut slot = TdmaSlot::new(0, SlotType::ISCH);
        slot.bits[0] = 1;
        slot.bits[7] = 1;
        slot.bits[1079] = 1;
        let stream = slot.to_bit_stream();
        let r = TdmaSlot::from_bit_stream(0, SlotType::ISCH, &stream);
        assert_eq!(r.bits[0], 1);
        assert_eq!(r.bits[7], 1);
        assert_eq!(r.bits[1079], 1);
    }

    #[test]
    fn test_tdma_frame_has_two_slots() {
        let frame = TdmaFrame::new(0);
        assert_eq!(frame.slots.len(), 2);
        assert_eq!(frame.slots[0].slot_id, 0);
        assert_eq!(frame.slots[1].slot_id, 1);
    }

    #[test]
    fn test_tdma_frame_to_bit_stream_length() {
        let frame = TdmaFrame::new(0);
        let stream = frame.to_bit_stream();
        // 2 × (48 + 1080 payload) + 2 × 8 guard
        let expected = 2 * (48 + BITS_PER_SLOT) + 2 * 8;
        assert_eq!(stream.len(), expected);
    }

    #[test]
    fn test_superframe_has_four_frames() {
        let sf = Superframe::new(0);
        assert_eq!(sf.frames.len(), FRAMES_PER_SUPERFRAME);
    }

    #[test]
    fn test_superframe_validate() {
        let sf = Superframe::new(0);
        assert!(sf.validate());
    }

    #[test]
    fn test_superframe_frame_numbers_sequential() {
        let sf = Superframe::new(2);
        for (i, frame) in sf.frames.iter().enumerate() {
            assert_eq!(frame.frame_number, 8 + i as u32);
        }
    }

    #[test]
    fn test_superframe_total_bits() {
        let sf = Superframe::new(0);
        let expected = FRAMES_PER_SUPERFRAME * (2 * (48 + BITS_PER_SLOT) + 2 * 8);
        assert_eq!(sf.total_bits(), expected);
    }

    // ---- AMBE+2 framing ----

    #[test]
    fn test_ambe2_frame_roundtrip() {
        let mut frame = Ambe2Frame::new();
        for i in 0..49 { frame.voice_bits[i] = (i % 2) as u8; }
        frame.compute_fec();
        let bits = frame.to_bits();
        let rec = Ambe2Frame::from_bits(&bits);
        assert_eq!(rec.voice_bits, frame.voice_bits);
        assert_eq!(rec.fec_bits[..3], frame.fec_bits[..3]);
    }

    #[test]
    fn test_ambe2_fec_passes_correct_data() {
        let mut frame = Ambe2Frame::new();
        for i in 0..49 { frame.voice_bits[i] = (i & 1) as u8; }
        frame.compute_fec();
        assert!(frame.check_fec());
    }

    #[test]
    fn test_ambe2_fec_fails_corrupted_data() {
        let mut frame = Ambe2Frame::new();
        frame.compute_fec();
        frame.voice_bits[5] ^= 1;
        assert!(!frame.check_fec());
    }

    #[test]
    fn test_voice_slot_payload_roundtrip() {
        let mut payload = VoiceSlotPayload::new(42);
        for frame in &mut payload.frames {
            for i in 0..49 { frame.voice_bits[i] = (i % 3) as u8; }
            frame.compute_fec();
        }
        let bits = payload.to_bits();
        assert_eq!(bits.len(), 4 * 88);
        let rec = VoiceSlotPayload::from_bits(&bits, 42);
        assert_eq!(rec.sequence, 42);
        for (a, b) in payload.frames.iter().zip(rec.frames.iter()) {
            assert_eq!(a.voice_bits, b.voice_bits);
        }
    }

    // ---- TCM encoder/decoder ----

    #[test]
    fn test_tcm_encode_all_zeros() {
        let mut enc = TcmEncoder::new();
        let out = enc.encode_tribits(&[0, 0, 0]);
        assert_eq!(out[1], 0);
        assert_eq!(out[2], 0);
    }

    #[test]
    fn test_tcm_encode_stream_length() {
        let mut enc = TcmEncoder::new();
        let input: Vec<u8> = (0..12).map(|i| (i % 2) as u8).collect();
        let output = enc.encode_bits(&input);
        assert_eq!(output.len(), 16); // 12 / 3 × 4 = 16
    }

    #[test]
    fn test_tcm_decode_all_zeros() {
        let dec = TcmDecoder::new();
        let coded = vec![0u8; 8];
        let decoded = dec.decode_bits(&coded);
        assert_eq!(decoded.len(), 6);
    }

    #[test]
    fn test_tcm_pass_through_bits_preserved() {
        let mut enc = TcmEncoder::new();
        let dec = TcmDecoder::new();
        let input: Vec<u8> = vec![1, 0, 1, 0, 1, 0, 1, 0, 1];
        let coded = enc.encode_bits(&input);
        let decoded = dec.decode_bits(&coded);
        for group in 0..3 {
            assert_eq!(decoded[group * 3 + 1], input[group * 3 + 1]);
            assert_eq!(decoded[group * 3 + 2], input[group * 3 + 2]);
        }
    }

    // ---- Golay(24,12,8) ----

    #[test]
    fn test_golay_encode_zero_message() {
        let cw = golay_encode(0);
        assert_eq!(cw, 0); // all-zero message → all-zero codeword
    }

    #[test]
    fn test_golay_decode_no_error() {
        let msg: u16 = 0b110011001100;
        let decoded = golay_decode(golay_encode(msg));
        assert_eq!(decoded, Some(msg));
    }

    #[test]
    fn test_golay_decode_single_bit_error() {
        let msg: u16 = 0b101010000101;
        let cw = golay_encode(msg);
        // Flip a bit in the parity half (LSB area of the 24-bit word)
        let decoded = golay_decode(cw ^ (1 << 1));
        assert_eq!(decoded, Some(msg));
    }

    #[test]
    fn test_golay_decode_two_bit_errors() {
        let msg: u16 = 0b111000111000;
        let cw = golay_encode(msg);
        let decoded = golay_decode(cw ^ (1 << 2) ^ (1 << 4));
        assert_eq!(decoded, Some(msg));
    }

    #[test]
    fn test_golay_decode_three_bit_errors() {
        let msg: u16 = 0b000011110000;
        let cw = golay_encode(msg);
        // Three errors in the parity portion (low bits of 24-bit word)
        let decoded = golay_decode(cw ^ 1 ^ (1 << 2) ^ (1 << 4));
        assert_eq!(decoded, Some(msg));
    }

    #[test]
    fn test_golay_encode_decode_all_small_messages() {
        for msg in 0u16..64 {
            let cw = golay_encode(msg);
            let decoded = golay_decode(cw);
            assert_eq!(decoded, Some(msg), "Failed for msg={}", msg);
        }
    }

    #[test]
    fn test_golay_codeword_length() {
        let cw = golay_encode(0b101010101010);
        // 24-bit codeword must fit in 24 bits
        assert!(cw <= 0xFFFFFF);
    }

    // ---- Reed-Solomon(36,20,17) ----

    #[test]
    fn test_rs_encode_produces_correct_length() {
        let enc = RsEncoder::new();
        let msg: Vec<u8> = (0..RS_K as u8).collect();
        let cw = enc.encode(&msg);
        assert_eq!(cw.len(), RS_N);
    }

    #[test]
    fn test_rs_encode_systematic_prefix() {
        let enc = RsEncoder::new();
        let msg: Vec<u8> = (0..RS_K as u8).map(|x| x & 0x3F).collect();
        let cw = enc.encode(&msg);
        assert_eq!(&cw[..RS_K], msg.as_slice());
    }

    #[test]
    fn test_rs_all_zero_message_gives_zero_parity() {
        let enc = RsEncoder::new();
        let msg = vec![0u8; RS_K];
        let cw = enc.encode(&msg);
        // All-zero message → all-zero parity (since any element times zero is zero)
        assert!(cw[RS_K..].iter().all(|&x| x == 0), "Parity should be all zeros");
    }

    #[test]
    fn test_rs_decode_no_errors() {
        let enc = RsEncoder::new();
        let dec = RsDecoder::new();
        let msg: Vec<u8> = (0..RS_K as u8).map(|x| x & 0x3F).collect();
        let cw = enc.encode(&msg);
        let decoded = dec.decode(&cw);
        assert_eq!(decoded, Some(msg));
    }

    #[test]
    fn test_rs_decode_single_symbol_error() {
        let enc = RsEncoder::new();
        let dec = RsDecoder::new();
        let msg: Vec<u8> = vec![1,3,5,7,9,11,13,15,17,19,21,23,25,27,29,31,2,4,6,8]
            .iter().map(|&x: &u8| x & 0x3F).collect();
        let mut cw = enc.encode(&msg);
        cw[0] ^= 0x05; // corrupt first symbol
        let decoded = dec.decode(&cw);
        assert_eq!(decoded, Some(msg));
    }

    #[test]
    fn test_rs_decode_parity_symbol_error() {
        let enc = RsEncoder::new();
        let dec = RsDecoder::new();
        let msg: Vec<u8> = (0..RS_K as u8).map(|x| x & 0x3F).collect();
        let mut cw = enc.encode(&msg);
        cw[RS_K + 2] ^= 0x07; // corrupt a parity symbol
        let decoded = dec.decode(&cw);
        assert_eq!(decoded, Some(msg));
    }

    // ---- CRC-9 ----

    #[test]
    fn test_crc9_empty_input_equals_init() {
        // CRC-9 with no bits processed returns the init value 0x1FF
        assert_eq!(crc9(&[]), 0x1FF);
    }

    #[test]
    fn test_crc9_to_bits_length() {
        let bits = crc9_to_bits(0x155);
        assert_eq!(bits.len(), 9);
    }

    #[test]
    fn test_crc9_verify_roundtrip() {
        let data: Vec<u8> = (0..79).map(|i| (i % 2) as u8).collect();
        let crc_val = crc9(&data);
        let crc_bits = crc9_to_bits(crc_val);
        assert!(crc9_verify(&data, &crc_bits));
    }

    #[test]
    fn test_crc9_detects_corruption() {
        let mut data: Vec<u8> = (0..79).map(|i| (i % 2) as u8).collect();
        let crc_val = crc9(&data);
        let crc_bits = crc9_to_bits(crc_val);
        data[10] ^= 1;
        assert!(!crc9_verify(&data, &crc_bits));
    }

    // ---- PN Scrambler ----

    #[test]
    fn test_pn_scrambler_roundtrip() {
        let mut bits: Vec<u8> = (0..64).map(|i| (i % 2) as u8).collect();
        let original = bits.clone();
        let mut scr1 = PnScrambler::new();
        scr1.scramble(&mut bits);
        assert_ne!(bits, original);
        let mut scr2 = PnScrambler::new();
        scr2.scramble(&mut bits);
        assert_eq!(bits, original);
    }

    #[test]
    fn test_pn_scrambler_reset_gives_same_sequence() {
        let mut scr = PnScrambler::new();
        let s1 = scr.generate(32);
        scr.reset();
        let s2 = scr.generate(32);
        assert_eq!(s1, s2);
    }

    #[test]
    fn test_pn_scrambler_custom_seed_reproducible() {
        let s1 = PnScrambler::with_seed(0x1234).generate(16);
        let s2 = PnScrambler::with_seed(0x1234).generate(16);
        assert_eq!(s1, s2);
    }

    #[test]
    fn test_pn_scrambler_different_seeds_differ() {
        let s1 = PnScrambler::with_seed(0x1234).generate(16);
        let s2 = PnScrambler::with_seed(0x5678).generate(16);
        assert_ne!(s1, s2);
    }

    #[test]
    fn test_pn_scrambler_output_binary() {
        let seq = PnScrambler::new().generate(100);
        assert!(seq.iter().all(|&b| b == 0 || b == 1));
    }

    // ---- Encryption ----

    #[test]
    fn test_message_indicator_increment() {
        let mut mi = MessageIndicator::new([0, 0, 0, 0, 0, 0, 0, 0, 0xFF]);
        mi.increment();
        assert_eq!(mi.bytes[8], 0);
        assert_eq!(mi.bytes[7], 1); // carry propagated
    }

    #[test]
    fn test_message_indicator_from_seed() {
        let mi = MessageIndicator::from_seed(0xDEADBEEFCAFE1234, 0xAB);
        assert_eq!(mi.bytes[0], 0xAB);
    }

    #[test]
    fn test_encryption_context_esn_bytes() {
        let key = [0u8; 32];
        let mi = MessageIndicator::from_seed(0x0102030405060708, 0x00);
        let ctx = EncryptionContext::aes256(0x0042, &key, mi.clone());
        let esn = ctx.to_esn_bytes();
        assert_eq!(esn[0], AlgorithmId::Aes256 as u8);
        assert_eq!(esn[2], 0x42); // key_id low byte
        assert_eq!(&esn[3..12], &mi.bytes[..]);
    }

    #[test]
    fn test_encryption_context_clear_mode() {
        let ctx = EncryptionContext::clear();
        assert_eq!(ctx.algorithm_id, AlgorithmId::Clear);
        assert_eq!(ctx.key_id, 0);
    }

    #[test]
    fn test_algorithm_id_try_from() {
        assert_eq!(AlgorithmId::try_from(0x80), Ok(AlgorithmId::Clear));
        assert_eq!(AlgorithmId::try_from(0x85), Ok(AlgorithmId::Aes256));
        assert!(AlgorithmId::try_from(0x42).is_err());
    }

    #[test]
    fn test_mi_to_u128_ordering() {
        let mi1 = MessageIndicator::new([0, 0, 0, 0, 0, 0, 0, 0, 1]);
        let mi2 = MessageIndicator::new([0, 0, 0, 0, 0, 0, 0, 0, 2]);
        assert!(mi1.to_u128() < mi2.to_u128());
    }

    // ---- Sync detection ----

    #[test]
    fn test_sync_finds_voice_sync_at_start() {
        let sync_bits = u64_to_bits_48(SLOT_SYNC_WORD);
        let mut stream = vec![0u8; 200];
        stream[..48].copy_from_slice(&sync_bits);
        let det = SyncDetector::new(2);
        let r = det.find_sync(&stream);
        assert!(r.found);
        assert_eq!(r.position, 0);
        assert_eq!(r.sync_type, SyncType::VoiceSlot);
        assert_eq!(r.errors, 0);
    }

    #[test]
    fn test_sync_finds_sync_at_offset() {
        let sync_bits = u64_to_bits_48(SLOT_SYNC_WORD);
        let mut stream = vec![0u8; 200];
        stream[50..98].copy_from_slice(&sync_bits);
        let det = SyncDetector::new(1);
        let r = det.find_sync(&stream);
        assert!(r.found);
        assert_eq!(r.position, 50);
    }

    #[test]
    fn test_sync_tolerates_one_error() {
        let mut sync_bits = u64_to_bits_48(SLOT_SYNC_WORD);
        sync_bits[10] ^= 1;
        let mut stream = vec![0u8; 200];
        stream[..48].copy_from_slice(&sync_bits);
        let det = SyncDetector::new(2);
        let r = det.find_sync(&stream);
        assert!(r.found);
        assert_eq!(r.errors, 1);
    }

    // ---- TSBK ----

    #[test]
    fn test_tsbk_serialize_opcode_field() {
        let tsbk = Tsbk::group_voice_grant(0x0001, 0x0100, 0x001234, false);
        let bytes = tsbk.to_bytes();
        assert_eq!(bytes[0] & 0x3F, TsbkOpcode::GroupVoiceChannelGrant as u8);
        assert_eq!(bytes[1], 0x00); // standard MFID
    }

    #[test]
    fn test_tsbk_crc_verification() {
        let tsbk = Tsbk::group_voice_grant(0x0002, 0x0200, 0x005678, true);
        assert!(tsbk.verify_crc());
    }

    #[test]
    fn test_tsbk_roundtrip_bytes() {
        let orig = Tsbk::group_voice_grant(0x0003, 0x0300, 0x009ABC, false);
        let bytes = orig.to_bytes();
        let rec = Tsbk::from_bytes(&bytes);
        assert_eq!(rec.opcode, orig.opcode);
        assert_eq!(rec.mfid,   orig.mfid);
        assert_eq!(rec.args,   orig.args);
    }

    #[test]
    fn test_tsbk_crc_fails_after_corruption() {
        let mut tsbk = Tsbk::group_voice_grant(0x0001, 0x0001, 0x000001, false);
        tsbk.args[0] ^= 0xFF;
        assert!(!tsbk.verify_crc());
    }

    #[test]
    fn test_tsbk_opcode_try_from() {
        assert_eq!(TsbkOpcode::try_from(0x20), Ok(TsbkOpcode::GroupVoiceChannelGrant));
        assert!(TsbkOpcode::try_from(0xFF).is_err());
    }

    // ---- Link Control ----

    #[test]
    fn test_lc_group_voice_user_roundtrip() {
        let so = ServiceOptions { emergency: false, encrypted: true, duplex: false, packet_switched: false, priority: 3 };
        let lc = LinkControl::group_voice_user(so, 0x1234, 0xABCDEF);
        let bytes = lc.to_bytes();
        let rec = LinkControl::from_bytes(&bytes);
        assert_eq!(rec.opcode, lc.opcode);
        assert_eq!(rec.args, lc.args);
    }

    #[test]
    fn test_lc_crc8_verification() {
        let lc = LinkControl::group_voice_user(ServiceOptions::new(), 0x0001, 0x000001);
        assert!(lc.verify_crc());
    }

    #[test]
    fn test_lc_encryption_sync_roundtrip() {
        let mi = MessageIndicator::from_seed(0x123456789ABCDEF0, 0x07);
        let lc = LinkControl::encryption_sync(AlgorithmId::Aes256, 0x0042, &mi);
        assert!(lc.verify_crc());
        let bytes = lc.to_bytes();
        let rec = LinkControl::from_bytes(&bytes);
        assert_eq!(rec.opcode, lc.opcode);
        assert_eq!(rec.args[0], AlgorithmId::Aes256 as u8);
    }

    #[test]
    fn test_service_options_roundtrip() {
        let so = ServiceOptions { emergency: true, encrypted: true, duplex: true, packet_switched: false, priority: 5 };
        let rec = ServiceOptions::from_byte(so.to_byte());
        assert_eq!(rec.emergency, so.emergency);
        assert_eq!(rec.encrypted, so.encrypted);
        assert_eq!(rec.duplex,    so.duplex);
        assert_eq!(rec.priority,  so.priority);
    }

    // ---- Utilities ----

    #[test]
    fn test_popcount_u8() {
        assert_eq!(popcount_u8(0), 0);
        assert_eq!(popcount_u8(0xFF), 8);
        assert_eq!(popcount_u8(0b10110100), 4);
    }

    #[test]
    fn test_popcount_u32() {
        assert_eq!(popcount_u32(0), 0);
        assert_eq!(popcount_u32(u32::MAX), 32);
        assert_eq!(popcount_u32(0xAAAAAAAA), 16);
    }

    #[test]
    fn test_hamming_distance_equal() {
        let a = [1u8, 0, 1, 0];
        assert_eq!(hamming_distance(&a, &a), 0);
    }

    #[test]
    fn test_hamming_distance_all_different() {
        let a = [1u8, 1, 1, 1];
        let b = [0u8, 0, 0, 0];
        assert_eq!(hamming_distance(&a, &b), 4);
    }

    #[test]
    fn test_bit_interleave_deinterleave_roundtrip() {
        let src: Vec<u8> = (0..9).map(|i| (i % 2) as u8).collect();
        let step = 4;
        let interleaved = bit_interleave(&src, step);
        let deinterleaved = bit_deinterleave(&interleaved, step);
        assert_eq!(deinterleaved, src);
    }

    #[test]
    fn test_crc16_ccitt_standard_vector() {
        // CCITT CRC-16 of "123456789" = 0x29B1
        let crc = crc16_ccitt(b"123456789");
        assert_eq!(crc, 0x29B1);
    }

    #[test]
    fn test_sync_word_to_bytes_high_byte() {
        let bytes = sync_word_to_bytes(SLOT_SYNC_WORD);
        // High byte of 0x5575F5FF77FF is 0x55
        assert_eq!(bytes[0], 0x55);
        // Low byte is 0xFF
        assert_eq!(bytes[5], 0xFF);
    }

    #[test]
    fn test_constants_consistency() {
        // 6000 sym/s × 0.18 s = 1080 symbols per frame = 2 × 540
        let syms_per_frame = (SYMBOL_RATE as usize) * (FRAME_DURATION_MS as usize) / 1000;
        assert_eq!(syms_per_frame, SLOTS_PER_FRAME * SYMBOLS_PER_SLOT);
    }

    #[test]
    fn test_voice_frames_fit_in_slot() {
        // 4 × 88 bits = 352 bits ≤ 1080 slot payload bits
        assert!(VOICE_FRAMES_PER_SLOT * AMBE2_FRAME_TOTAL_BITS <= BITS_PER_SLOT);
    }
}
