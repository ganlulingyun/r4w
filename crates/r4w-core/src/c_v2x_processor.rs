//! C-V2X (Cellular Vehicle-to-Everything) Processor
//!
//! Implements 3GPP TS 36.211 / TS 36.213 Mode 4 sidelink physical layer for
//! direct vehicle-to-vehicle (V2V) and vehicle-to-infrastructure (V2I) communications
//! without network assistance (autonomous resource selection).
//!
//! # Key Features
//!
//! - **SC-FDMA / DFT-s-OFDM PHY**: 10/20 MHz bandwidth, 15 kHz SCS, 12 subcarriers/RB
//! - **Sidelink Resource Pool**: Mode 4 autonomous resource selection, sensing-based SPS
//! - **SCI Format 0/1**: Sidelink Control Information encoding (priority, MCS, reservation)
//! - **PSCCH / PSSCH**: Physical Sidelink Control/Shared Channel generation and mapping
//! - **DMRS**: Sidelink Demodulation Reference Signals (Zadoff-Chu based)
//! - **HARQ**: Blind retransmission (no ACK/NACK feedback), configurable max retx
//! - **Sensing & Selection**: RSSI sensing, CBR (Channel Busy Ratio), SPS reselection
//! - **BSM**: SAE J2945 Basic Safety Message formatting
//! - **Link Budget**: ITS-band (5.9 GHz) V2X LOS/NLOS path loss models
//!
//! # Standards References
//!
//! - 3GPP TS 36.211 §14: Sidelink physical channels
//! - 3GPP TS 36.213 §14: Sidelink resource allocation (Mode 4)
//! - SAE J2945/1: Basic Safety Message
//! - ETSI EN 302 571: ITS 5.9 GHz band
//!
//! # Example
//!
//! ```rust
//! use r4w_core::c_v2x_processor::{CV2xProcessor, CV2xConfig, BsmPayload};
//!
//! let config = CV2xConfig::default_10mhz();
//! let mut proc = CV2xProcessor::new(config);
//!
//! let bsm = BsmPayload {
//!     msg_count: 1,
//!     latitude_deg7: 377490000,
//!     longitude_deg7: -1221680000,
//!     elevation_dm: 500,
//!     speed_ms_x100: 1000,
//!     heading_deg2: 9000,
//!     accel_lon_x100: 0,
//!     accel_lat_x100: 0,
//!     accel_vert_x100: 0,
//! };
//!
//! let (pscch_symbols, pssch_symbols) = proc.transmit_bsm(&bsm);
//! assert!(!pscch_symbols.is_empty());
//! assert!(!pssch_symbols.is_empty());
//! ```

use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants (3GPP TS 36.211, Table 14.x)
// ---------------------------------------------------------------------------

/// Subcarrier spacing for sidelink: 15 kHz
pub const SUBCARRIER_SPACING_HZ: f64 = 15_000.0;

/// Number of subcarriers per resource block
pub const SUBCARRIERS_PER_RB: usize = 12;

/// Useful OFDM symbol duration (1/15 kHz ≈ 66.7 µs)
pub const SYMBOL_DURATION_S: f64 = 1.0 / SUBCARRIER_SPACING_HZ;

/// Normal cyclic prefix duration (4.7 µs for most symbols)
pub const CP_NORMAL_S: f64 = 4.69e-6;

/// Total OFDM symbol period including CP
pub const SYMBOL_PERIOD_S: f64 = SYMBOL_DURATION_S + CP_NORMAL_S;

/// Number of OFDM symbols per slot (0.5 ms)
pub const SYMBOLS_PER_SLOT: usize = 7;

/// Number of slots per subframe (1 ms)
pub const SLOTS_PER_SUBFRAME: usize = 2;

/// Number of subframes per frame (10 ms)
pub const SUBFRAMES_PER_FRAME: usize = 10;

/// ITS 5.9 GHz band centre (Hz)
pub const ITS_CARRIER_HZ: f64 = 5.89e9;

/// Boltzmann constant
const K_BOLTZMANN: f64 = 1.380649e-23;

/// Speed of light (m/s)
const C_LIGHT: f64 = 2.998e8;

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------

/// Bandwidth configuration
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BandwidthConfig {
    /// 10 MHz → 50 RBs
    Bw10Mhz,
    /// 20 MHz → 100 RBs
    Bw20Mhz,
}

impl BandwidthConfig {
    /// Number of resource blocks
    pub fn num_rbs(self) -> usize {
        match self {
            BandwidthConfig::Bw10Mhz => 50,
            BandwidthConfig::Bw20Mhz => 100,
        }
    }

    /// FFT size (NFFT)
    pub fn fft_size(self) -> usize {
        match self {
            BandwidthConfig::Bw10Mhz => 1024,
            BandwidthConfig::Bw20Mhz => 2048,
        }
    }

    /// Sampling rate (Hz)
    pub fn sample_rate(self) -> f64 {
        match self {
            BandwidthConfig::Bw10Mhz => 15.36e6,
            BandwidthConfig::Bw20Mhz => 30.72e6,
        }
    }
}

/// Modulation and Coding Scheme index (per 3GPP TS 36.213 Table 8.6.1-1)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum McsIndex {
    /// QPSK rate ~0.12 (robust)
    Mcs0,
    /// QPSK rate ~0.19
    Mcs3,
    /// QPSK rate ~0.30
    Mcs6,
    /// QPSK rate ~0.44
    Mcs9,
    /// 16-QAM rate ~0.44
    Mcs12,
    /// 16-QAM rate ~0.59
    Mcs15,
    /// 16-QAM rate ~0.74
    Mcs18,
    /// 64-QAM rate ~0.60
    Mcs21,
    /// 64-QAM rate ~0.73
    Mcs24,
    /// 64-QAM rate ~0.85
    Mcs27,
}

impl McsIndex {
    /// Bits per symbol for this MCS
    pub fn bits_per_symbol(self) -> usize {
        match self {
            McsIndex::Mcs0 | McsIndex::Mcs3 | McsIndex::Mcs6 | McsIndex::Mcs9 => 2, // QPSK
            McsIndex::Mcs12 | McsIndex::Mcs15 | McsIndex::Mcs18 => 4,                // 16-QAM
            McsIndex::Mcs21 | McsIndex::Mcs24 | McsIndex::Mcs27 => 6,                // 64-QAM
        }
    }

    /// Approximate code rate (numerator/1024)
    pub fn code_rate_x1024(self) -> u32 {
        match self {
            McsIndex::Mcs0 => 120,
            McsIndex::Mcs3 => 193,
            McsIndex::Mcs6 => 308,
            McsIndex::Mcs9 => 449,
            McsIndex::Mcs12 => 449,
            McsIndex::Mcs15 => 602,
            McsIndex::Mcs18 => 762,
            McsIndex::Mcs21 => 616,
            McsIndex::Mcs24 => 753,
            McsIndex::Mcs27 => 873,
        }
    }

    /// Index value (0..27)
    pub fn index(self) -> u8 {
        match self {
            McsIndex::Mcs0 => 0,
            McsIndex::Mcs3 => 3,
            McsIndex::Mcs6 => 6,
            McsIndex::Mcs9 => 9,
            McsIndex::Mcs12 => 12,
            McsIndex::Mcs15 => 15,
            McsIndex::Mcs18 => 18,
            McsIndex::Mcs21 => 21,
            McsIndex::Mcs24 => 24,
            McsIndex::Mcs27 => 27,
        }
    }
}

/// Resource reservation interval (RRI) for SPS scheduling
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ResourceReservationInterval {
    /// 20 ms
    Ms20,
    /// 50 ms
    Ms50,
    /// 100 ms
    Ms100,
    /// 200 ms (optional extension)
    Ms200,
}

impl ResourceReservationInterval {
    /// Duration in milliseconds
    pub fn ms(self) -> u32 {
        match self {
            ResourceReservationInterval::Ms20 => 20,
            ResourceReservationInterval::Ms50 => 50,
            ResourceReservationInterval::Ms100 => 100,
            ResourceReservationInterval::Ms200 => 200,
        }
    }

    /// 3-bit encoding per TS 36.213 Table 14.2.1-1
    pub fn encoded(self) -> u8 {
        match self {
            ResourceReservationInterval::Ms20 => 0b001,
            ResourceReservationInterval::Ms50 => 0b010,
            ResourceReservationInterval::Ms100 => 0b011,
            ResourceReservationInterval::Ms200 => 0b100,
        }
    }
}

/// Propagation environment for path loss
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PropagationModel {
    /// Line-of-sight (WINNER+ B1)
    Los,
    /// Non-line-of-sight urban (WINNER+ C2)
    NlosUrban,
    /// Non-line-of-sight suburban
    NlosSuburban,
}

// ---------------------------------------------------------------------------
// Structures
// ---------------------------------------------------------------------------

/// Complex sample (I + jQ) using f32 for memory efficiency
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Complex32 {
    pub re: f32,
    pub im: f32,
}

impl Complex32 {
    pub fn new(re: f32, im: f32) -> Self {
        Self { re, im }
    }

    pub fn zero() -> Self {
        Self { re: 0.0, im: 0.0 }
    }

    pub fn magnitude(&self) -> f32 {
        (self.re * self.re + self.im * self.im).sqrt()
    }

    pub fn magnitude_sq(&self) -> f32 {
        self.re * self.re + self.im * self.im
    }

    pub fn mul(&self, other: &Complex32) -> Complex32 {
        Complex32 {
            re: self.re * other.re - self.im * other.im,
            im: self.re * other.im + self.im * other.re,
        }
    }

    pub fn add(&self, other: &Complex32) -> Complex32 {
        Complex32 {
            re: self.re + other.re,
            im: self.im + other.im,
        }
    }
}

/// SCI Format 1 (Sidelink Control Information, TS 36.212 §5.4.3.1.2)
/// 32-bit payload for PSCCH
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SciFormat1 {
    /// Priority: 0 (highest) … 7 (lowest), 3 bits
    pub priority: u8,
    /// Resource reservation interval, 4 bits (encoded per TS 36.213 Table 14.2.1-1)
    pub resource_reservation: u8,
    /// Frequency resource location (bitmap over pool RBs), 8 bits
    pub freq_resource_loc: u8,
    /// Time gap (T2-T1) in subframes, 4 bits
    pub time_gap: u8,
    /// MCS index, 5 bits
    pub mcs_index: u8,
    /// Retransmission index (0=initial, 1=retx), 1 bit
    pub retx_index: u8,
    /// Reserved (padding to 32 bits), 7 bits
    pub reserved: u8,
}

impl SciFormat1 {
    /// Encode to 32-bit word (MSB first)
    pub fn encode(&self) -> u32 {
        let mut word: u32 = 0;
        word |= ((self.priority as u32) & 0x7) << 29;
        word |= ((self.resource_reservation as u32) & 0xF) << 25;
        word |= ((self.freq_resource_loc as u32) & 0xFF) << 17;
        word |= ((self.time_gap as u32) & 0xF) << 13;
        word |= ((self.mcs_index as u32) & 0x1F) << 8;
        word |= ((self.retx_index as u32) & 0x1) << 7;
        word |= (self.reserved as u32) & 0x7F;
        word
    }

    /// Decode from 32-bit word
    pub fn decode(word: u32) -> Self {
        Self {
            priority: ((word >> 29) & 0x7) as u8,
            resource_reservation: ((word >> 25) & 0xF) as u8,
            freq_resource_loc: ((word >> 17) & 0xFF) as u8,
            time_gap: ((word >> 13) & 0xF) as u8,
            mcs_index: ((word >> 8) & 0x1F) as u8,
            retx_index: ((word >> 7) & 0x1) as u8,
            reserved: (word & 0x7F) as u8,
        }
    }
}

/// SAE J2945 Basic Safety Message payload (simplified)
#[derive(Debug, Clone, Copy)]
pub struct BsmPayload {
    /// Message count (0..127), 1 byte
    pub msg_count: u8,
    /// Latitude in 1/10 µdeg (WGS-84), 32-bit signed
    pub latitude_deg7: i32,
    /// Longitude in 1/10 µdeg (WGS-84), 32-bit signed
    pub longitude_deg7: i32,
    /// Elevation in decimetres above MSL, 16-bit signed
    pub elevation_dm: i16,
    /// Speed in 0.01 m/s steps (0..16383), 16-bit
    pub speed_ms_x100: u16,
    /// Heading 0.1° steps clockwise from North (0..3599), 16-bit
    pub heading_deg2: u16,
    /// Longitudinal acceleration in 0.01 m/s², signed 16-bit
    pub accel_lon_x100: i16,
    /// Lateral acceleration in 0.01 m/s², signed 16-bit
    pub accel_lat_x100: i16,
    /// Vertical acceleration in 0.01 m/s², signed 16-bit
    pub accel_vert_x100: i16,
}

impl BsmPayload {
    /// Serialise to bytes (19 bytes; no ASN.1 overhead for simplicity)
    pub fn to_bytes(&self) -> [u8; 19] {
        let mut buf = [0u8; 19];
        buf[0] = self.msg_count;
        buf[1..5].copy_from_slice(&self.latitude_deg7.to_be_bytes());
        buf[5..9].copy_from_slice(&self.longitude_deg7.to_be_bytes());
        buf[9..11].copy_from_slice(&self.elevation_dm.to_be_bytes());
        buf[11..13].copy_from_slice(&self.speed_ms_x100.to_be_bytes());
        buf[13..15].copy_from_slice(&self.heading_deg2.to_be_bytes());
        buf[15..17].copy_from_slice(&self.accel_lon_x100.to_be_bytes());
        buf[17..19].copy_from_slice(&self.accel_lat_x100.to_be_bytes());
        buf
    }

    /// Deserialise from bytes
    pub fn from_bytes(buf: &[u8]) -> Option<Self> {
        if buf.len() < 19 {
            return None;
        }
        Some(Self {
            msg_count: buf[0],
            latitude_deg7: i32::from_be_bytes([buf[1], buf[2], buf[3], buf[4]]),
            longitude_deg7: i32::from_be_bytes([buf[5], buf[6], buf[7], buf[8]]),
            elevation_dm: i16::from_be_bytes([buf[9], buf[10]]),
            speed_ms_x100: u16::from_be_bytes([buf[11], buf[12]]),
            heading_deg2: u16::from_be_bytes([buf[13], buf[14]]),
            accel_lon_x100: i16::from_be_bytes([buf[15], buf[16]]),
            accel_lat_x100: i16::from_be_bytes([buf[17], buf[18]]),
            accel_vert_x100: 0,
        })
    }
}

/// Sidelink resource pool configuration (TS 36.331 §6.3.8 SL-ResourcePool)
#[derive(Debug, Clone)]
pub struct ResourcePool {
    /// First RB index of pool (within channel bandwidth)
    pub start_rb: usize,
    /// Number of RBs in pool
    pub num_rbs: usize,
    /// Period of resource pool in subframes
    pub sc_period_subframes: u32,
    /// Bitmap of active subframes within one period
    pub sc_bitmap: Vec<bool>,
    /// Number of PSCCH RBs (typically 2)
    pub pscch_rbs: usize,
}

impl ResourcePool {
    /// Default 10 MHz pool: 48 data RBs, PSCCH 2 RBs, 100 ms period
    pub fn default_10mhz() -> Self {
        let period = 100usize;
        let mut bitmap = vec![false; period];
        // First 40 subframes active within the 100 ms period
        for i in 0..40 {
            bitmap[i] = true;
        }
        Self {
            start_rb: 1,
            num_rbs: 48,
            sc_period_subframes: period as u32,
            sc_bitmap: bitmap,
            pscch_rbs: 2,
        }
    }

    /// Number of PSSCH data RBs (total pool minus PSCCH RBs)
    pub fn pssch_rbs(&self) -> usize {
        self.num_rbs.saturating_sub(self.pscch_rbs)
    }
}

/// HARQ process state
#[derive(Debug, Clone)]
struct HarqProcess {
    /// Transport block data
    data: Vec<u8>,
    /// Number of transmissions so far (0-indexed)
    tx_count: u8,
    /// Maximum re-transmissions
    max_retx: u8,
    /// SCI associated with this process
    sci: SciFormat1,
    /// Process active
    active: bool,
}

impl HarqProcess {
    fn new(max_retx: u8) -> Self {
        Self {
            data: Vec::new(),
            tx_count: 0,
            max_retx,
            sci: SciFormat1 {
                priority: 4,
                resource_reservation: 0,
                freq_resource_loc: 0,
                time_gap: 0,
                mcs_index: 0,
                retx_index: 0,
                reserved: 0,
            },
            active: false,
        }
    }

    fn needs_retx(&self) -> bool {
        self.active && self.tx_count < self.max_retx + 1
    }
}

/// Sensing state per resource slot (for Mode 4 autonomous selection)
#[derive(Debug, Clone)]
struct SensingEntry {
    /// Subframe number
    subframe_idx: u32,
    /// RB index
    rb_idx: usize,
    /// RSSI measurement (linear power)
    rssi_linear: f32,
    /// Whether this slot appears occupied (RSSI above threshold)
    occupied: bool,
}

/// Configuration for C-V2X Mode 4
#[derive(Debug, Clone)]
pub struct CV2xConfig {
    /// Bandwidth configuration
    pub bandwidth: BandwidthConfig,
    /// Carrier frequency (default: 5.89 GHz)
    pub carrier_hz: f64,
    /// Transmit power in dBm (default: 23 dBm EIRP)
    pub tx_power_dbm: f64,
    /// Default MCS
    pub mcs: McsIndex,
    /// Resource reservation interval
    pub rri: ResourceReservationInterval,
    /// Resource pool configuration
    pub pool: ResourcePool,
    /// Maximum HARQ blind retransmissions (0=no retx, 1=1 retx, …)
    pub max_harq_retx: u8,
    /// Sensing window length in subframes (default: 1000)
    pub sensing_window_subframes: u32,
    /// CBR threshold for congestion control (0..1)
    pub cbr_threshold: f32,
    /// SPS reselection counter range (min, max)
    pub sps_resel_counter_range: (u8, u8),
    /// Noise figure in dB (for sensitivity calculation)
    pub noise_figure_db: f64,
    /// Receiver bandwidth in Hz (for noise floor)
    pub receiver_bw_hz: f64,
}

impl CV2xConfig {
    /// Default 10 MHz C-V2X configuration
    pub fn default_10mhz() -> Self {
        Self {
            bandwidth: BandwidthConfig::Bw10Mhz,
            carrier_hz: ITS_CARRIER_HZ,
            tx_power_dbm: 23.0,
            mcs: McsIndex::Mcs9,
            rri: ResourceReservationInterval::Ms100,
            pool: ResourcePool::default_10mhz(),
            max_harq_retx: 1,
            sensing_window_subframes: 1000,
            cbr_threshold: 0.65,
            sps_resel_counter_range: (5, 15),
            noise_figure_db: 9.0,
            receiver_bw_hz: 9.0e6,
        }
    }

    /// Default 20 MHz C-V2X configuration
    pub fn default_20mhz() -> Self {
        let mut cfg = Self::default_10mhz();
        cfg.bandwidth = BandwidthConfig::Bw20Mhz;
        cfg.pool.num_rbs = 98;
        cfg.pool.start_rb = 1;
        cfg.receiver_bw_hz = 18.0e6;
        cfg
    }
}

/// Link budget result
#[derive(Debug, Clone)]
pub struct LinkBudget {
    /// Distance (m)
    pub distance_m: f64,
    /// Path loss (dB)
    pub path_loss_db: f64,
    /// Received power (dBm)
    pub rx_power_dbm: f64,
    /// Thermal noise floor (dBm)
    pub noise_floor_dbm: f64,
    /// SNR at receiver (dB)
    pub snr_db: f64,
    /// Propagation model used
    pub model: PropagationModel,
}

/// Sensing report for a resource
#[derive(Debug, Clone)]
pub struct SensingReport {
    /// RB index
    pub rb_idx: usize,
    /// Average RSSI (dBm)
    pub avg_rssi_dbm: f32,
    /// Estimated occupancy (0.0 .. 1.0)
    pub occupancy: f32,
    /// Candidate for selection (true = low occupancy)
    pub is_candidate: bool,
}

/// Channel Busy Ratio measurement
#[derive(Debug, Clone, Copy)]
pub struct CbrMeasurement {
    /// CBR value (0.0 .. 1.0)
    pub cbr: f32,
    /// Threshold used
    pub threshold: f32,
    /// Whether CBR exceeded threshold (congestion)
    pub congested: bool,
    /// Recommended MCS based on CBR
    pub recommended_mcs: u8,
}

// ---------------------------------------------------------------------------
// FFT (radix-2 DIT, in-place, for DFT-s-OFDM)
// ---------------------------------------------------------------------------

fn fft_inplace(buf: &mut Vec<Complex32>, inverse: bool) {
    let n = buf.len();
    if n <= 1 {
        return;
    }
    // Bit-reversal permutation
    let mut j = 0usize;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;
        if i < j {
            buf.swap(i, j);
        }
    }
    // Cooley-Tukey butterfly
    let sign: f64 = if inverse { 1.0 } else { -1.0 };
    let mut len = 2usize;
    while len <= n {
        let half = len / 2;
        let ang = sign * 2.0 * PI / (len as f64);
        for i in (0..n).step_by(len) {
            for k in 0..half {
                let theta = ang * (k as f64);
                let wr = theta.cos() as f32;
                let wi = theta.sin() as f32;
                let u = buf[i + k];
                let v_re = buf[i + k + half].re * wr - buf[i + k + half].im * wi;
                let v_im = buf[i + k + half].re * wi + buf[i + k + half].im * wr;
                let v = Complex32::new(v_re, v_im);
                buf[i + k] = u.add(&v);
                buf[i + k + half] = Complex32::new(u.re - v.re, u.im - v.im);
            }
        }
        len <<= 1;
    }
    if inverse {
        let inv_n = 1.0 / (n as f32);
        for s in buf.iter_mut() {
            s.re *= inv_n;
            s.im *= inv_n;
        }
    }
}

// ---------------------------------------------------------------------------
// DFT-s-OFDM modulator (SC-FDMA TX)
// ---------------------------------------------------------------------------

/// Return smallest power of two >= n (minimum 1)
fn next_power_of_two(n: usize) -> usize {
    if n == 0 {
        return 1;
    }
    let mut p = 1usize;
    while p < n {
        p <<= 1;
    }
    p
}

/// DFT-s-OFDM modulator: applies M-point DFT then maps to N-point IFFT subcarriers
pub struct DftSOfdm {
    fft_size: usize,
    num_data_subcarriers: usize,
    start_subcarrier: usize,
}

impl DftSOfdm {
    pub fn new(fft_size: usize, num_rbs: usize, start_rb: usize) -> Self {
        let sc = num_rbs * SUBCARRIERS_PER_RB;
        let start = start_rb * SUBCARRIERS_PER_RB;
        Self {
            fft_size,
            num_data_subcarriers: sc,
            start_subcarrier: start,
        }
    }

    /// Modulate `symbols` (one slot worth) to time-domain samples with CP
    /// Returns OFDM symbol time samples (including cyclic prefix)
    pub fn modulate_slot(&self, symbols: &[Complex32]) -> Vec<Complex32> {
        let m = self.num_data_subcarriers;
        // Zero-pad input to M if needed
        let mut dft_in: Vec<Complex32> = symbols[..m.min(symbols.len())].to_vec();
        while dft_in.len() < m {
            dft_in.push(Complex32::zero());
        }

        // M-point DFT precoding: zero-pad to next power-of-2 for FFT,
        // then take only the first M outputs for subcarrier mapping.
        let dft_fft_size = next_power_of_two(m);
        let mut dft_padded: Vec<Complex32> = Vec::with_capacity(dft_fft_size);
        dft_padded.extend_from_slice(&dft_in);
        while dft_padded.len() < dft_fft_size {
            dft_padded.push(Complex32::zero());
        }
        fft_inplace(&mut dft_padded, false);
        // Use only the first m bins as DFT output (zero-padded DFT approximation)
        let dft_out = &dft_padded[..m];

        // Map to OFDM subcarriers (frequency-domain)
        let mut freq_buf: Vec<Complex32> = vec![Complex32::zero(); self.fft_size];
        for k in 0..m {
            let idx = (self.start_subcarrier + k) % self.fft_size;
            freq_buf[idx] = dft_out[k];
        }

        // N-point IFFT
        fft_inplace(&mut freq_buf, true);

        // Add cyclic prefix (normal CP: 1/4 of fft_size as approximation)
        let cp_len = self.fft_size / 8;
        let total = cp_len + self.fft_size;
        let mut td: Vec<Complex32> = Vec::with_capacity(total);
        // CP from end of OFDM symbol
        td.extend_from_slice(&freq_buf[self.fft_size - cp_len..]);
        td.extend_from_slice(&freq_buf);
        td
    }
}

// ---------------------------------------------------------------------------
// Zadoff-Chu DMRS generator (TS 36.211 §14.2.1)
// ---------------------------------------------------------------------------

/// Generate a Zadoff-Chu sequence for sidelink DMRS
///
/// # Arguments
/// * `u` - Root index (1..30)
/// * `length` - Sequence length M_sc
///
/// Returns a vector of length `length` complex samples
pub fn generate_zadoff_chu(u: usize, length: usize) -> Vec<Complex32> {
    assert!(u > 0, "ZC root u must be >= 1");
    let u_f = u as f64;
    let n_f = length as f64;
    (0..length)
        .map(|n| {
            let n_d = n as f64;
            let phase = -PI * u_f * n_d * (n_d + 1.0) / n_f;
            Complex32::new(phase.cos() as f32, phase.sin() as f32)
        })
        .collect()
}

/// Generate sidelink DMRS for a given slot
///
/// Uses Zadoff-Chu root u derived from physical layer identity
/// per TS 36.211 §14.2.1.2
pub fn generate_dmrs(num_rbs: usize, sl_identity: u16) -> Vec<Complex32> {
    let length = num_rbs * SUBCARRIERS_PER_RB;
    // Root sequence: u = (sl_identity mod 30) + 1
    let u = ((sl_identity % 30) as usize) + 1;
    generate_zadoff_chu(u, length)
}

// ---------------------------------------------------------------------------
// QPSK / 16-QAM / 64-QAM constellation mappers
// ---------------------------------------------------------------------------

/// QPSK mapper: 2 bits → 1 symbol (Gray-coded)
pub fn qpsk_map(bits: &[u8]) -> Vec<Complex32> {
    let norm: f32 = (0.5f32).sqrt(); // 1/sqrt(2)
    bits.chunks(2)
        .map(|b| {
            let b0 = b[0] & 1;
            let b1 = if b.len() > 1 { b[1] & 1 } else { 0 };
            let i = if b0 == 0 { norm } else { -norm };
            let q = if b1 == 0 { norm } else { -norm };
            Complex32::new(i, q)
        })
        .collect()
}

/// 16-QAM mapper: 4 bits → 1 symbol (Gray-coded, normalised)
pub fn qam16_map(bits: &[u8]) -> Vec<Complex32> {
    let norm: f32 = 1.0 / (10.0f32).sqrt();
    bits.chunks(4)
        .map(|b| {
            let b: [u8; 4] = [
                b[0] & 1,
                if b.len() > 1 { b[1] & 1 } else { 0 },
                if b.len() > 2 { b[2] & 1 } else { 0 },
                if b.len() > 3 { b[3] & 1 } else { 0 },
            ];
            // I: b[0], b[2]; Q: b[1], b[3]
            let i = (if b[0] == 0 { 3.0f32 } else { 1.0 }) * (if b[2] == 0 { 1.0 } else { -1.0 });
            let q = (if b[1] == 0 { 3.0f32 } else { 1.0 }) * (if b[3] == 0 { 1.0 } else { -1.0 });
            Complex32::new(i * norm, q * norm)
        })
        .collect()
}

/// 64-QAM mapper: 6 bits → 1 symbol (Gray-coded, normalised)
pub fn qam64_map(bits: &[u8]) -> Vec<Complex32> {
    let norm: f32 = 1.0 / (42.0f32).sqrt();
    bits.chunks(6)
        .map(|b| {
            let get = |i: usize| if i < b.len() { b[i] & 1 } else { 0 };
            // I: b0, b2, b4; Q: b1, b3, b5
            let i_level = match (get(0), get(2), get(4)) {
                (0, 0, 0) => 7.0f32,
                (0, 0, 1) => 5.0,
                (0, 1, 0) => 3.0,
                (0, 1, 1) => 1.0,
                (1, 0, 0) => -7.0,
                (1, 0, 1) => -5.0,
                (1, 1, 0) => -3.0,
                _ => -1.0,
            };
            let q_level = match (get(1), get(3), get(5)) {
                (0, 0, 0) => 7.0f32,
                (0, 0, 1) => 5.0,
                (0, 1, 0) => 3.0,
                (0, 1, 1) => 1.0,
                (1, 0, 0) => -7.0,
                (1, 0, 1) => -5.0,
                (1, 1, 0) => -3.0,
                _ => -1.0,
            };
            Complex32::new(i_level * norm, q_level * norm)
        })
        .collect()
}

/// Map bits using MCS-specified modulation
pub fn modulate_bits(bits: &[u8], mcs: McsIndex) -> Vec<Complex32> {
    match mcs.bits_per_symbol() {
        2 => qpsk_map(bits),
        4 => qam16_map(bits),
        6 => qam64_map(bits),
        _ => qpsk_map(bits),
    }
}

// ---------------------------------------------------------------------------
// Turbo-like rate matching (simplified scrambling + repetition)
// ---------------------------------------------------------------------------

/// Scramble bytes using LFSR (simplified Gold-like scrambler per TS 36.211 §14.3.5)
pub fn sidelink_scramble(data: &[u8], cinit: u32) -> Vec<u8> {
    // 31-bit LFSR: x^31 + x^3 + 1 (simplified)
    let mut lfsr: u32 = cinit & 0x7FFF_FFFF;
    if lfsr == 0 {
        lfsr = 1;
    }
    data.iter()
        .map(|&b| {
            let mut out = 0u8;
            for i in 0..8 {
                let bit = ((b >> i) & 1) ^ (lfsr & 1) as u8;
                out |= bit << i;
                let feedback = ((lfsr >> 30) ^ (lfsr >> 2)) & 1;
                lfsr = (lfsr >> 1) | (feedback << 30);
            }
            out
        })
        .collect()
}

/// Convert byte slice to bit vector
pub fn bytes_to_bits(data: &[u8]) -> Vec<u8> {
    let mut bits = Vec::with_capacity(data.len() * 8);
    for &b in data {
        for i in (0..8).rev() {
            bits.push((b >> i) & 1);
        }
    }
    bits
}

/// Convert bit vector to bytes (pad with zeros)
pub fn bits_to_bytes(bits: &[u8]) -> Vec<u8> {
    bits.chunks(8)
        .map(|chunk| {
            let mut b = 0u8;
            for (i, &bit) in chunk.iter().enumerate() {
                b |= (bit & 1) << (7 - i);
            }
            b
        })
        .collect()
}

// ---------------------------------------------------------------------------
// Path loss models for 5.9 GHz ITS band
// ---------------------------------------------------------------------------

/// Compute free-space path loss (Friis equation) in dB
///
/// FSPL = 20·log₁₀(4πd·fc/c)
pub fn fspl_db(distance_m: f64, carrier_hz: f64) -> f64 {
    if distance_m <= 0.0 {
        return 0.0;
    }
    20.0 * (4.0 * PI * distance_m * carrier_hz / C_LIGHT).log10()
}

/// WINNER+ B1 LOS path loss model for V2X at 5.9 GHz
///
/// PL(d) = 22.7·log₁₀(d) + 27.0 + 20·log₁₀(f_GHz)   for d ≥ 3 m
pub fn winner_b1_los_db(distance_m: f64, carrier_hz: f64) -> f64 {
    let d = distance_m.max(3.0);
    let f_ghz = carrier_hz / 1e9;
    22.7 * d.log10() + 27.0 + 20.0 * f_ghz.log10()
}

/// WINNER+ C2 NLOS urban path loss model at 5.9 GHz
///
/// PL(d) = 44.9·log₁₀(d) + 31.5 + 20·log₁₀(f_GHz)
pub fn winner_c2_nlos_db(distance_m: f64, carrier_hz: f64) -> f64 {
    let d = distance_m.max(3.0);
    let f_ghz = carrier_hz / 1e9;
    44.9 * d.log10() + 31.5 + 20.0 * f_ghz.log10()
}

/// NLOS suburban model (between LOS and urban NLOS)
pub fn nlos_suburban_db(distance_m: f64, carrier_hz: f64) -> f64 {
    let d = distance_m.max(3.0);
    let f_ghz = carrier_hz / 1e9;
    36.7 * d.log10() + 26.0 + 20.0 * f_ghz.log10()
}

// ---------------------------------------------------------------------------
// Main C-V2X Processor
// ---------------------------------------------------------------------------

/// C-V2X Mode 4 processor (autonomous sidelink)
pub struct CV2xProcessor {
    config: CV2xConfig,
    harq: HarqProcess,
    /// Sensing history: ring buffer
    sensing_buf: Vec<SensingEntry>,
    sensing_write_idx: usize,
    /// Current subframe counter
    subframe_count: u32,
    /// SPS reselection counter
    sps_counter: u8,
    /// Currently selected resource (RB index)
    selected_rb: usize,
    /// Physical layer sidelink identity (derived from L2 source ID)
    sl_identity: u16,
    /// DFT-s-OFDM modulator
    modulator: DftSOfdm,
}

impl CV2xProcessor {
    /// Create a new C-V2X processor
    pub fn new(config: CV2xConfig) -> Self {
        let sensing_cap = (config.sensing_window_subframes * config.pool.num_rbs as u32) as usize;
        let fft_size = config.bandwidth.fft_size();
        let pool = config.pool.clone();
        let sl_id = 0x5A3Cu16; // default L2 source ID hash
        let modulator = DftSOfdm::new(fft_size, pool.num_rbs, pool.start_rb);
        let max_retx = config.max_harq_retx;
        Self {
            config,
            harq: HarqProcess::new(max_retx),
            sensing_buf: Vec::with_capacity(sensing_cap.min(100_000)),
            sensing_write_idx: 0,
            subframe_count: 0,
            sps_counter: 10,
            selected_rb: 0,
            sl_identity: sl_id,
            modulator,
        }
    }

    /// Advance by one subframe tick; performs sensing update and SPS countdown
    pub fn tick_subframe(&mut self) {
        self.subframe_count = self.subframe_count.wrapping_add(1);
        if self.sps_counter > 0 {
            self.sps_counter -= 1;
        }
    }

    /// Record an RSSI measurement for a resource slot
    pub fn record_rssi(&mut self, rb_idx: usize, rssi_dbm: f32) {
        let threshold_dbm = self.sensitivity_dbm() as f32 + 3.0;
        let rssi_lin = 10.0f32.powf(rssi_dbm / 10.0);
        let entry = SensingEntry {
            subframe_idx: self.subframe_count,
            rb_idx,
            rssi_linear: rssi_lin,
            occupied: rssi_dbm > threshold_dbm,
        };
        if self.sensing_buf.len() < 100_000 {
            self.sensing_buf.push(entry);
        } else {
            let idx = self.sensing_write_idx % self.sensing_buf.len();
            self.sensing_buf[idx] = entry;
            self.sensing_write_idx += 1;
        }
    }

    /// Receiver sensitivity (dBm) given noise figure and bandwidth
    pub fn sensitivity_dbm(&self) -> f64 {
        let noise_floor_w = K_BOLTZMANN * 290.0 * self.config.receiver_bw_hz;
        let noise_floor_dbm = 10.0 * (noise_floor_w * 1000.0).log10();
        noise_floor_dbm + self.config.noise_figure_db
    }

    /// Compute Channel Busy Ratio from recent sensing history
    pub fn compute_cbr(&self) -> CbrMeasurement {
        if self.sensing_buf.is_empty() {
            return CbrMeasurement {
                cbr: 0.0,
                threshold: self.config.cbr_threshold,
                congested: false,
                recommended_mcs: self.config.mcs.index(),
            };
        }
        let occupied_count = self.sensing_buf.iter().filter(|e| e.occupied).count();
        let cbr = occupied_count as f32 / self.sensing_buf.len() as f32;
        let congested = cbr > self.config.cbr_threshold;
        // Reduce MCS under congestion
        let recommended_mcs = if congested {
            // Step down two MCS indices
            self.config.mcs.index().saturating_sub(6)
        } else {
            self.config.mcs.index()
        };
        CbrMeasurement {
            cbr,
            threshold: self.config.cbr_threshold,
            congested,
            recommended_mcs,
        }
    }

    /// Perform sensing-based resource selection (Mode 4 algorithm)
    /// Returns best candidate RB index
    pub fn select_resource(&mut self) -> usize {
        let num_rbs = self.config.pool.pssch_rbs();
        if num_rbs == 0 {
            return 0;
        }

        // Compute per-RB occupancy from sensing buffer
        let mut rb_occ: Vec<u32> = vec![0u32; num_rbs];
        let mut rb_cnt: Vec<u32> = vec![0u32; num_rbs];
        for entry in &self.sensing_buf {
            if entry.rb_idx < num_rbs {
                rb_cnt[entry.rb_idx] += 1;
                if entry.occupied {
                    rb_occ[entry.rb_idx] += 1;
                }
            }
        }

        // Candidate set: RBs with occupancy < 20%
        let mut best_rb = 0usize;
        let mut best_score = f32::MAX;
        for rb in 0..num_rbs {
            let cnt = rb_cnt[rb];
            let occ_frac = if cnt > 0 {
                rb_occ[rb] as f32 / cnt as f32
            } else {
                0.0
            };
            if occ_frac < best_score {
                best_score = occ_frac;
                best_rb = rb;
            }
        }

        self.selected_rb = best_rb;
        best_rb
    }

    /// Generate sensing reports for all pool RBs
    pub fn sensing_reports(&self) -> Vec<SensingReport> {
        let num_rbs = self.config.pool.pssch_rbs();
        let mut reports = Vec::with_capacity(num_rbs);
        for rb in 0..num_rbs {
            let entries: Vec<&SensingEntry> =
                self.sensing_buf.iter().filter(|e| e.rb_idx == rb).collect();
            if entries.is_empty() {
                reports.push(SensingReport {
                    rb_idx: rb,
                    avg_rssi_dbm: self.sensitivity_dbm() as f32 - 10.0,
                    occupancy: 0.0,
                    is_candidate: true,
                });
                continue;
            }
            let avg_lin: f32 =
                entries.iter().map(|e| e.rssi_linear).sum::<f32>() / entries.len() as f32;
            let avg_dbm = 10.0 * avg_lin.log10();
            let occ = entries.iter().filter(|e| e.occupied).count() as f32 / entries.len() as f32;
            reports.push(SensingReport {
                rb_idx: rb,
                avg_rssi_dbm: avg_dbm,
                occupancy: occ,
                is_candidate: occ < 0.2,
            });
        }
        reports
    }

    /// Build SCI Format 1 for current transmission
    pub fn build_sci(&self, retx_index: u8) -> SciFormat1 {
        SciFormat1 {
            priority: 3,
            resource_reservation: self.config.rri.encoded(),
            freq_resource_loc: (self.selected_rb & 0xFF) as u8,
            time_gap: if retx_index == 0 { 0 } else { 1 },
            mcs_index: self.config.mcs.index(),
            retx_index,
            reserved: 0,
        }
    }

    /// Generate PSCCH symbols (SCI encoded and modulated)
    ///
    /// PSCCH occupies `pscch_rbs` RBs in the resource pool.
    /// Uses QPSK modulation as per TS 36.211 §14.4.
    pub fn generate_pscch(&self, sci: &SciFormat1) -> Vec<Complex32> {
        let sci_word = sci.encode();
        // 32 bits, each bit as u8
        let mut bits: Vec<u8> = Vec::with_capacity(32);
        for i in (0..32).rev() {
            bits.push(((sci_word >> i) & 1) as u8);
        }
        // CRC-24A append (simplified: 3 zero bytes)
        for _ in 0..24 {
            bits.push(0);
        }
        // QPSK modulation (all PSCCH uses QPSK)
        let pscch_rbs = self.config.pool.pscch_rbs;
        let available_re = pscch_rbs * SUBCARRIERS_PER_RB;
        let symbols = qpsk_map(&bits);
        // Truncate or zero-pad to fill PSCCH RBs
        let mut out = vec![Complex32::zero(); available_re];
        for (i, &s) in symbols.iter().enumerate().take(available_re) {
            out[i] = s;
        }
        // Insert DMRS at symbol positions (3rd and 6th position per slot)
        let dmrs = generate_dmrs(pscch_rbs, self.sl_identity);
        for (i, d) in dmrs.iter().enumerate().take(out.len()) {
            if i % 7 == 3 || i % 7 == 6 {
                out[i] = *d;
            }
        }
        out
    }

    /// Generate PSSCH symbols (transport block encoded and modulated)
    ///
    /// Transport block is scrambled, mapped via MCS constellation.
    pub fn generate_pssch(&self, tb: &[u8]) -> Vec<Complex32> {
        // Scramble with cinit based on SL identity
        let cinit = (self.sl_identity as u32) | ((self.subframe_count & 0xFF) << 16);
        let scrambled = sidelink_scramble(tb, cinit);
        let bits = bytes_to_bits(&scrambled);
        let symbols = modulate_bits(&bits, self.config.mcs);
        let pssch_rbs = self.config.pool.pssch_rbs();
        let available_re = pssch_rbs * SUBCARRIERS_PER_RB;
        // Fill resource elements
        let mut out = vec![Complex32::zero(); available_re];
        for (i, &s) in symbols.iter().enumerate().take(available_re) {
            out[i] = s;
        }
        // Insert DMRS at pilot positions
        let dmrs = generate_dmrs(pssch_rbs, self.sl_identity);
        for (i, d) in dmrs.iter().enumerate().take(out.len()) {
            if i % 7 == 3 || i % 7 == 6 {
                out[i] = *d;
            }
        }
        out
    }

    /// Transmit a BSM: encodes, selects resource, generates PSCCH + PSSCH
    ///
    /// Returns `(pscch_symbols, pssch_symbols)` frequency-domain resource elements.
    pub fn transmit_bsm(&mut self, bsm: &BsmPayload) -> (Vec<Complex32>, Vec<Complex32>) {
        let tb = bsm.to_bytes();

        // Select resource if SPS counter expired or no resource assigned
        if self.sps_counter == 0 {
            self.select_resource();
            let (min_c, max_c) = self.config.sps_resel_counter_range;
            // Pseudo-random counter in [min_c, max_c]
            let range = (max_c - min_c) as u32 + 1;
            self.sps_counter = min_c + ((self.subframe_count % range) as u8);
        }

        // Build SCI
        let sci = self.build_sci(0);

        // Store in HARQ process
        self.harq.data = tb.to_vec();
        self.harq.tx_count = 1;
        self.harq.active = true;
        self.harq.sci = sci;

        let pscch = self.generate_pscch(&sci);
        let pssch = self.generate_pssch(&tb);

        self.tick_subframe();
        (pscch, pssch)
    }

    /// Attempt HARQ blind retransmission if enabled
    ///
    /// Returns `Some((pscch, pssch))` if retransmission occurs, `None` otherwise.
    pub fn retransmit(&mut self) -> Option<(Vec<Complex32>, Vec<Complex32>)> {
        if !self.harq.needs_retx() {
            return None;
        }
        let sci = self.build_sci(self.harq.tx_count.min(1));
        let data = self.harq.data.clone();
        let pscch = self.generate_pscch(&sci);
        let pssch = self.generate_pssch(&data);
        self.harq.tx_count += 1;
        if self.harq.tx_count > self.harq.max_retx + 1 {
            self.harq.active = false;
        }
        self.tick_subframe();
        Some((pscch, pssch))
    }

    /// Generate time-domain waveform for a single PSCCH + PSSCH slot
    ///
    /// Applies DFT-s-OFDM (SC-FDMA) to produce IQ samples.
    pub fn generate_waveform(
        &self,
        pscch: &[Complex32],
        pssch: &[Complex32],
    ) -> Vec<Complex32> {
        // Combine PSCCH and PSSCH in frequency domain (different RBs)
        let pscch_rbs = self.config.pool.pscch_rbs;
        let pssch_rbs = self.config.pool.pssch_rbs();
        let total_rbs = pscch_rbs + pssch_rbs;
        let mut combined = vec![Complex32::zero(); total_rbs * SUBCARRIERS_PER_RB];
        // PSCCH occupies first pscch_rbs
        for (i, &s) in pscch.iter().enumerate().take(pscch_rbs * SUBCARRIERS_PER_RB) {
            combined[i] = s;
        }
        // PSSCH follows
        let offset = pscch_rbs * SUBCARRIERS_PER_RB;
        for (i, &s) in pssch.iter().enumerate().take(pssch_rbs * SUBCARRIERS_PER_RB) {
            if offset + i < combined.len() {
                combined[offset + i] = s;
            }
        }
        // Apply DFT-s-OFDM via the modulator
        self.modulator.modulate_slot(&combined)
    }

    /// Compute V2X link budget
    pub fn link_budget(
        &self,
        distance_m: f64,
        model: PropagationModel,
    ) -> LinkBudget {
        let carrier = self.config.carrier_hz;
        let path_loss = match model {
            PropagationModel::Los => winner_b1_los_db(distance_m, carrier),
            PropagationModel::NlosUrban => winner_c2_nlos_db(distance_m, carrier),
            PropagationModel::NlosSuburban => nlos_suburban_db(distance_m, carrier),
        };
        let rx_power_dbm = self.config.tx_power_dbm - path_loss;
        let noise_floor_dbm = self.sensitivity_dbm() - self.config.noise_figure_db;
        let snr_db = rx_power_dbm - (noise_floor_dbm + self.config.noise_figure_db);
        LinkBudget {
            distance_m,
            path_loss_db: path_loss,
            rx_power_dbm,
            noise_floor_dbm,
            snr_db,
            model,
        }
    }

    /// Current subframe count
    pub fn subframe_count(&self) -> u32 {
        self.subframe_count
    }

    /// Currently selected RB
    pub fn selected_rb(&self) -> usize {
        self.selected_rb
    }

    /// Reference to configuration
    pub fn config(&self) -> &CV2xConfig {
        &self.config
    }
}

// ---------------------------------------------------------------------------
// CRC-24A (simplified version for PSCCH)
// ---------------------------------------------------------------------------

/// CRC-24A computation per 3GPP TS 36.212 §5.1.1
/// Polynomial: x^24 + x^23 + x^6 + x^5 + x + 1 (0x864CFB)
pub fn crc24a(data: &[u8]) -> u32 {
    const POLY: u32 = 0x864CFB;
    let mut crc: u32 = 0;
    for &byte in data {
        for i in (0..8).rev() {
            let bit = ((byte >> i) & 1) as u32;
            let feedback = (crc >> 23) & 1;
            crc = ((crc << 1) | bit) ^ (if feedback ^ bit != 0 { POLY } else { 0 });
        }
    }
    crc & 0x00FF_FFFF
}

// ---------------------------------------------------------------------------
// Transport block size lookup (simplified)
// ---------------------------------------------------------------------------

/// Compute approximate transport block size for given MCS and number of RBs
///
/// Per 3GPP TS 36.213 Table 7.1.7.1-1 (simplified formula)
pub fn transport_block_size(mcs: McsIndex, num_rbs: usize) -> usize {
    let bits_per_re = mcs.bits_per_symbol();
    let code_rate = mcs.code_rate_x1024() as f64 / 1024.0;
    let n_re_per_rb = SUBCARRIERS_PER_RB * SYMBOLS_PER_SLOT * SLOTS_PER_SUBFRAME
        - 4; // subtract DMRS REs
    let total_re = num_rbs * n_re_per_rb;
    let tbs_bits = (total_re as f64 * bits_per_re as f64 * code_rate) as usize;
    // Round down to nearest byte
    (tbs_bits / 8) * 8
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- Config tests ---

    #[test]
    fn test_config_default_10mhz() {
        let cfg = CV2xConfig::default_10mhz();
        assert_eq!(cfg.bandwidth, BandwidthConfig::Bw10Mhz);
        assert_eq!(cfg.bandwidth.num_rbs(), 50);
        assert_eq!(cfg.bandwidth.fft_size(), 1024);
        assert!((cfg.bandwidth.sample_rate() - 15.36e6).abs() < 1.0);
        assert_eq!(cfg.pool.num_rbs, 48);
        assert_eq!(cfg.pool.pscch_rbs, 2);
    }

    #[test]
    fn test_config_default_20mhz() {
        let cfg = CV2xConfig::default_20mhz();
        assert_eq!(cfg.bandwidth, BandwidthConfig::Bw20Mhz);
        assert_eq!(cfg.bandwidth.num_rbs(), 100);
        assert_eq!(cfg.bandwidth.fft_size(), 2048);
        assert!((cfg.bandwidth.sample_rate() - 30.72e6).abs() < 1.0);
    }

    // --- SCI encoding tests ---

    #[test]
    fn test_sci_encode_decode_roundtrip() {
        let sci = SciFormat1 {
            priority: 3,
            resource_reservation: ResourceReservationInterval::Ms100.encoded(),
            freq_resource_loc: 42,
            time_gap: 5,
            mcs_index: McsIndex::Mcs9.index(),
            retx_index: 0,
            reserved: 0,
        };
        let encoded = sci.encode();
        let decoded = SciFormat1::decode(encoded);
        assert_eq!(decoded.priority, sci.priority);
        assert_eq!(decoded.resource_reservation, sci.resource_reservation);
        assert_eq!(decoded.freq_resource_loc, sci.freq_resource_loc);
        assert_eq!(decoded.time_gap, sci.time_gap);
        assert_eq!(decoded.mcs_index, sci.mcs_index);
        assert_eq!(decoded.retx_index, sci.retx_index);
    }

    #[test]
    fn test_sci_priority_range() {
        for prio in 0u8..8 {
            let sci = SciFormat1 {
                priority: prio,
                resource_reservation: 0,
                freq_resource_loc: 0,
                time_gap: 0,
                mcs_index: 0,
                retx_index: 0,
                reserved: 0,
            };
            let decoded = SciFormat1::decode(sci.encode());
            assert_eq!(decoded.priority, prio);
        }
    }

    #[test]
    fn test_sci_retx_index() {
        let mut sci = SciFormat1 {
            priority: 0,
            resource_reservation: 0,
            freq_resource_loc: 0,
            time_gap: 0,
            mcs_index: 0,
            retx_index: 0,
            reserved: 0,
        };
        sci.retx_index = 0;
        assert_eq!(SciFormat1::decode(sci.encode()).retx_index, 0);
        sci.retx_index = 1;
        assert_eq!(SciFormat1::decode(sci.encode()).retx_index, 1);
    }

    // --- RRI tests ---

    #[test]
    fn test_rri_encoding() {
        assert_eq!(ResourceReservationInterval::Ms20.ms(), 20);
        assert_eq!(ResourceReservationInterval::Ms50.ms(), 50);
        assert_eq!(ResourceReservationInterval::Ms100.ms(), 100);
        assert_eq!(ResourceReservationInterval::Ms200.ms(), 200);
        assert_eq!(ResourceReservationInterval::Ms20.encoded(), 0b001);
        assert_eq!(ResourceReservationInterval::Ms100.encoded(), 0b011);
    }

    // --- BSM serialisation tests ---

    #[test]
    fn test_bsm_serialise_deserialise() {
        let bsm = BsmPayload {
            msg_count: 42,
            latitude_deg7: 377490000,
            longitude_deg7: -1221680000,
            elevation_dm: 500,
            speed_ms_x100: 2500,
            heading_deg2: 1800,
            accel_lon_x100: 50,
            accel_lat_x100: -30,
            accel_vert_x100: 0,
        };
        let bytes = bsm.to_bytes();
        assert_eq!(bytes.len(), 19);
        let recovered = BsmPayload::from_bytes(&bytes).unwrap();
        assert_eq!(recovered.msg_count, bsm.msg_count);
        assert_eq!(recovered.latitude_deg7, bsm.latitude_deg7);
        assert_eq!(recovered.longitude_deg7, bsm.longitude_deg7);
        assert_eq!(recovered.elevation_dm, bsm.elevation_dm);
        assert_eq!(recovered.speed_ms_x100, bsm.speed_ms_x100);
        assert_eq!(recovered.heading_deg2, bsm.heading_deg2);
        assert_eq!(recovered.accel_lon_x100, bsm.accel_lon_x100);
    }

    #[test]
    fn test_bsm_from_bytes_too_short() {
        let buf = [0u8; 10];
        assert!(BsmPayload::from_bytes(&buf).is_none());
    }

    // --- QPSK mapper tests ---

    #[test]
    fn test_qpsk_map_four_points() {
        let bits = vec![0, 0, 0, 1, 1, 0, 1, 1];
        let syms = qpsk_map(&bits);
        assert_eq!(syms.len(), 4);
        let norm = (0.5f32).sqrt();
        // (0,0) → (+norm, +norm)
        assert!((syms[0].re - norm).abs() < 1e-5);
        assert!((syms[0].im - norm).abs() < 1e-5);
        // (0,1) → (+norm, -norm)
        assert!((syms[1].re - norm).abs() < 1e-5);
        assert!((syms[1].im + norm).abs() < 1e-5);
        // (1,0) → (-norm, +norm)
        assert!((syms[2].re + norm).abs() < 1e-5);
        assert!((syms[2].im - norm).abs() < 1e-5);
        // (1,1) → (-norm, -norm)
        assert!((syms[3].re + norm).abs() < 1e-5);
        assert!((syms[3].im + norm).abs() < 1e-5);
    }

    #[test]
    fn test_qpsk_unit_power() {
        let bits: Vec<u8> = (0..128).map(|i| (i % 2) as u8).collect();
        let syms = qpsk_map(&bits);
        for s in &syms {
            let power = s.re * s.re + s.im * s.im;
            assert!((power - 1.0).abs() < 1e-5, "QPSK power != 1: {}", power);
        }
    }

    #[test]
    fn test_qam16_map_count() {
        let bits: Vec<u8> = vec![0u8; 40];
        let syms = qam16_map(&bits);
        assert_eq!(syms.len(), 10);
    }

    #[test]
    fn test_qam64_map_count() {
        let bits: Vec<u8> = vec![0u8; 60];
        let syms = qam64_map(&bits);
        assert_eq!(syms.len(), 10);
    }

    // --- Zadoff-Chu tests ---

    #[test]
    fn test_zadoff_chu_unit_magnitude() {
        let seq = generate_zadoff_chu(1, 72);
        assert_eq!(seq.len(), 72);
        for s in &seq {
            let mag = s.magnitude();
            assert!((mag - 1.0).abs() < 1e-4, "ZC magnitude != 1: {}", mag);
        }
    }

    #[test]
    fn test_zadoff_chu_length() {
        for u in 1..=5 {
            for len in [12, 24, 36, 72] {
                let seq = generate_zadoff_chu(u, len);
                assert_eq!(seq.len(), len);
            }
        }
    }

    #[test]
    fn test_dmrs_generation() {
        let dmrs = generate_dmrs(2, 0x1234);
        assert_eq!(dmrs.len(), 2 * SUBCARRIERS_PER_RB);
        for s in &dmrs {
            let mag = s.magnitude();
            assert!((mag - 1.0).abs() < 1e-4);
        }
    }

    // --- Scrambler tests ---

    #[test]
    fn test_scramble_length_preserved() {
        let data: Vec<u8> = (0..32).map(|i| i as u8).collect();
        let scrambled = sidelink_scramble(&data, 0xABCD_1234);
        assert_eq!(scrambled.len(), data.len());
    }

    #[test]
    fn test_scramble_different_cinit() {
        let data: Vec<u8> = vec![0xAA; 16];
        let s1 = sidelink_scramble(&data, 1);
        let s2 = sidelink_scramble(&data, 2);
        // Different cinit → different output (with overwhelming probability)
        assert_ne!(s1, s2);
    }

    #[test]
    fn test_bytes_bits_roundtrip() {
        let data: Vec<u8> = vec![0xA5, 0x3C, 0xFF, 0x00];
        let bits = bytes_to_bits(&data);
        assert_eq!(bits.len(), 32);
        let recovered = bits_to_bytes(&bits);
        assert_eq!(recovered, data);
    }

    // --- Path loss models tests ---

    #[test]
    fn test_fspl_increases_with_distance() {
        let f = ITS_CARRIER_HZ;
        let pl10 = fspl_db(10.0, f);
        let pl100 = fspl_db(100.0, f);
        let pl1000 = fspl_db(1000.0, f);
        assert!(pl10 < pl100, "FSPL should increase with distance");
        assert!(pl100 < pl1000);
    }

    #[test]
    fn test_winner_b1_los_minimum_distance() {
        // At 3 m and 5.9 GHz
        let pl = winner_b1_los_db(3.0, ITS_CARRIER_HZ);
        assert!(pl > 0.0);
        assert!(pl < 100.0, "Path loss at 3m too high: {}", pl);
    }

    #[test]
    fn test_winner_c2_nlos_greater_than_los() {
        let d = 100.0;
        let f = ITS_CARRIER_HZ;
        let los = winner_b1_los_db(d, f);
        let nlos = winner_c2_nlos_db(d, f);
        assert!(nlos > los, "NLOS should be worse than LOS at same distance");
    }

    #[test]
    fn test_nlos_suburban_between_los_nlos() {
        let d = 100.0;
        let f = ITS_CARRIER_HZ;
        let los = winner_b1_los_db(d, f);
        let sub = nlos_suburban_db(d, f);
        let nlos = winner_c2_nlos_db(d, f);
        assert!(sub > los);
        assert!(sub < nlos);
    }

    // --- Link budget tests ---

    #[test]
    fn test_link_budget_los_100m() {
        let cfg = CV2xConfig::default_10mhz();
        let proc = CV2xProcessor::new(cfg);
        let lb = proc.link_budget(100.0, PropagationModel::Los);
        assert_eq!(lb.model, PropagationModel::Los);
        assert!(lb.path_loss_db > 0.0);
        assert!(lb.rx_power_dbm < 23.0, "Rx power must be less than Tx");
        // At 100m LOS, SNR should be adequate (>5 dB typical)
        assert!(lb.snr_db > 0.0, "SNR should be positive at 100m LOS");
    }

    #[test]
    fn test_link_budget_nlos_300m() {
        let cfg = CV2xConfig::default_10mhz();
        let proc = CV2xProcessor::new(cfg);
        let lb_los = proc.link_budget(300.0, PropagationModel::Los);
        let lb_nlos = proc.link_budget(300.0, PropagationModel::NlosUrban);
        assert!(lb_nlos.rx_power_dbm < lb_los.rx_power_dbm);
        assert!(lb_nlos.snr_db < lb_los.snr_db);
    }

    #[test]
    fn test_link_budget_sensitivity() {
        let cfg = CV2xConfig::default_10mhz();
        let proc = CV2xProcessor::new(cfg);
        let sens = proc.sensitivity_dbm();
        // 9 MHz BW @ 9 dB NF: approx -96 dBm
        assert!(sens < -80.0, "Sensitivity too poor: {} dBm", sens);
        assert!(sens > -110.0, "Sensitivity unrealistically good: {} dBm", sens);
    }

    // --- Resource pool tests ---

    #[test]
    fn test_resource_pool_pssch_rbs() {
        let pool = ResourcePool::default_10mhz();
        assert_eq!(pool.pssch_rbs(), pool.num_rbs - pool.pscch_rbs);
    }

    #[test]
    fn test_resource_pool_bitmap_length() {
        let pool = ResourcePool::default_10mhz();
        assert_eq!(pool.sc_bitmap.len(), pool.sc_period_subframes as usize);
        let active: usize = pool.sc_bitmap.iter().filter(|&&b| b).count();
        assert!(active > 0, "At least some subframes should be active");
    }

    // --- CBR tests ---

    #[test]
    fn test_cbr_empty_sensing() {
        let cfg = CV2xConfig::default_10mhz();
        let proc = CV2xProcessor::new(cfg);
        let cbr = proc.compute_cbr();
        assert_eq!(cbr.cbr, 0.0);
        assert!(!cbr.congested);
    }

    #[test]
    fn test_cbr_congested() {
        let cfg = CV2xConfig::default_10mhz();
        let mut proc = CV2xProcessor::new(cfg);
        // Simulate heavily occupied channel (all RBs busy)
        for i in 0..200 {
            proc.record_rssi(i % 46, -60.0); // well above threshold
        }
        let cbr = proc.compute_cbr();
        assert!(cbr.cbr > 0.0, "CBR should be non-zero with occupied channel");
    }

    #[test]
    fn test_cbr_recommended_mcs_reduced_on_congestion() {
        let cfg = CV2xConfig::default_10mhz();
        let original_mcs = cfg.mcs.index();
        let mut proc = CV2xProcessor::new(cfg);
        // Fill sensing with occupied entries
        for i in 0..500 {
            proc.record_rssi(i % 46, -50.0);
        }
        let cbr = proc.compute_cbr();
        if cbr.congested {
            assert!(
                cbr.recommended_mcs < original_mcs,
                "MCS should be reduced under congestion"
            );
        }
    }

    // --- PSCCH / PSSCH generation tests ---

    #[test]
    fn test_pscch_generation_length() {
        let cfg = CV2xConfig::default_10mhz();
        let proc = CV2xProcessor::new(cfg.clone());
        let sci = proc.build_sci(0);
        let pscch = proc.generate_pscch(&sci);
        assert_eq!(pscch.len(), cfg.pool.pscch_rbs * SUBCARRIERS_PER_RB);
    }

    #[test]
    fn test_pssch_generation_non_empty() {
        let cfg = CV2xConfig::default_10mhz();
        let proc = CV2xProcessor::new(cfg.clone());
        let tb = vec![0u8; 50];
        let pssch = proc.generate_pssch(&tb);
        assert!(!pssch.is_empty());
        assert_eq!(pssch.len(), cfg.pool.pssch_rbs() * SUBCARRIERS_PER_RB);
    }

    #[test]
    fn test_pscch_contains_dmrs_unit_magnitude() {
        let cfg = CV2xConfig::default_10mhz();
        let proc = CV2xProcessor::new(cfg);
        let sci = proc.build_sci(0);
        let pscch = proc.generate_pscch(&sci);
        // DMRS positions: i % 7 == 3 or 6 → check unit magnitude
        for (i, s) in pscch.iter().enumerate() {
            if i % 7 == 3 || i % 7 == 6 {
                let mag = s.magnitude();
                assert!((mag - 1.0).abs() < 1e-4, "DMRS magnitude at idx {}: {}", i, mag);
            }
        }
    }

    // --- Transmit BSM tests ---

    #[test]
    fn test_transmit_bsm_returns_symbols() {
        let cfg = CV2xConfig::default_10mhz();
        let mut proc = CV2xProcessor::new(cfg);
        let bsm = BsmPayload {
            msg_count: 1,
            latitude_deg7: 377490000,
            longitude_deg7: -1221680000,
            elevation_dm: 500,
            speed_ms_x100: 1000,
            heading_deg2: 9000,
            accel_lon_x100: 0,
            accel_lat_x100: 0,
            accel_vert_x100: 0,
        };
        let (pscch, pssch) = proc.transmit_bsm(&bsm);
        assert!(!pscch.is_empty(), "PSCCH should not be empty");
        assert!(!pssch.is_empty(), "PSSCH should not be empty");
    }

    #[test]
    fn test_subframe_counter_increments() {
        let cfg = CV2xConfig::default_10mhz();
        let mut proc = CV2xProcessor::new(cfg);
        let initial = proc.subframe_count();
        proc.tick_subframe();
        proc.tick_subframe();
        assert_eq!(proc.subframe_count(), initial + 2);
    }

    // --- HARQ retransmission tests ---

    #[test]
    fn test_harq_retx_after_initial_tx() {
        let cfg = CV2xConfig::default_10mhz();
        let mut proc = CV2xProcessor::new(cfg);
        let bsm = BsmPayload {
            msg_count: 0,
            latitude_deg7: 0,
            longitude_deg7: 0,
            elevation_dm: 0,
            speed_ms_x100: 0,
            heading_deg2: 0,
            accel_lon_x100: 0,
            accel_lat_x100: 0,
            accel_vert_x100: 0,
        };
        let _ = proc.transmit_bsm(&bsm);
        let retx = proc.retransmit();
        assert!(retx.is_some(), "HARQ retx should succeed after initial TX");
        let (p, s) = retx.unwrap();
        assert!(!p.is_empty());
        assert!(!s.is_empty());
    }

    #[test]
    fn test_harq_max_retx_respected() {
        let mut cfg = CV2xConfig::default_10mhz();
        cfg.max_harq_retx = 1;
        let mut proc = CV2xProcessor::new(cfg);
        let bsm = BsmPayload {
            msg_count: 0,
            latitude_deg7: 0,
            longitude_deg7: 0,
            elevation_dm: 0,
            speed_ms_x100: 0,
            heading_deg2: 0,
            accel_lon_x100: 0,
            accel_lat_x100: 0,
            accel_vert_x100: 0,
        };
        let _ = proc.transmit_bsm(&bsm);
        let _r1 = proc.retransmit(); // 1st retx (allowed)
        let r2 = proc.retransmit(); // 2nd retx (should be None)
        assert!(r2.is_none(), "Max retx exceeded: should return None");
    }

    // --- Sensing / resource selection tests ---

    #[test]
    fn test_resource_selection_returns_valid_rb() {
        let cfg = CV2xConfig::default_10mhz();
        let mut proc = CV2xProcessor::new(cfg.clone());
        let rb = proc.select_resource();
        assert!(rb < cfg.pool.pssch_rbs(), "Selected RB {} out of range", rb);
    }

    #[test]
    fn test_sensing_reports_count() {
        let cfg = CV2xConfig::default_10mhz();
        let proc = CV2xProcessor::new(cfg.clone());
        let reports = proc.sensing_reports();
        assert_eq!(reports.len(), cfg.pool.pssch_rbs());
    }

    #[test]
    fn test_sensing_prefers_less_occupied_rb() {
        let cfg = CV2xConfig::default_10mhz();
        let mut proc = CV2xProcessor::new(cfg);
        // Mark RB 0 as heavily occupied
        for _ in 0..20 {
            proc.record_rssi(0, -50.0);
        }
        // RB 1 stays quiet
        for _ in 0..20 {
            proc.record_rssi(1, -100.0);
        }
        let rb = proc.select_resource();
        // RB 1 should be preferred
        assert_eq!(rb, 1, "Sensing should prefer less occupied RB 1");
    }

    // --- DFT-s-OFDM modulator tests ---

    #[test]
    fn test_dft_s_ofdm_output_length() {
        let cfg = CV2xConfig::default_10mhz();
        let fft_size = cfg.bandwidth.fft_size();
        let modulator = DftSOfdm::new(fft_size, cfg.pool.num_rbs, cfg.pool.start_rb);
        let input: Vec<Complex32> = (0..cfg.pool.num_rbs * SUBCARRIERS_PER_RB)
            .map(|_| Complex32::new(0.5, 0.0))
            .collect();
        let td = modulator.modulate_slot(&input);
        // Output = IFFT + CP
        let cp_len = fft_size / 8;
        assert_eq!(td.len(), fft_size + cp_len);
    }

    #[test]
    fn test_generate_waveform_non_empty() {
        let cfg = CV2xConfig::default_10mhz();
        let mut proc = CV2xProcessor::new(cfg);
        let bsm = BsmPayload {
            msg_count: 5,
            latitude_deg7: 100_000_000,
            longitude_deg7: -200_000_000,
            elevation_dm: 100,
            speed_ms_x100: 800,
            heading_deg2: 450,
            accel_lon_x100: 0,
            accel_lat_x100: 0,
            accel_vert_x100: 0,
        };
        let (pscch, pssch) = proc.transmit_bsm(&bsm);
        let waveform = proc.generate_waveform(&pscch, &pssch);
        assert!(!waveform.is_empty());
    }

    // --- CRC-24A tests ---

    #[test]
    fn test_crc24a_nonzero_for_nonzero_input() {
        let data = [0x01u8, 0x02, 0x03, 0x04];
        let crc = crc24a(&data);
        assert_ne!(crc, 0, "CRC-24A of non-zero data should be non-zero");
    }

    #[test]
    fn test_crc24a_differs_from_all_zeros() {
        let d1 = [0x00u8; 4];
        let d2 = [0xFFu8; 4];
        let c1 = crc24a(&d1);
        let c2 = crc24a(&d2);
        assert_ne!(c1, c2);
    }

    #[test]
    fn test_crc24a_length_mask() {
        let data = [0xABu8, 0xCD];
        let crc = crc24a(&data);
        assert!(crc <= 0x00FF_FFFF, "CRC-24A must fit 24 bits");
    }

    // --- Transport block size tests ---

    #[test]
    fn test_tbs_increases_with_rbs() {
        let tbs_2 = transport_block_size(McsIndex::Mcs9, 2);
        let tbs_4 = transport_block_size(McsIndex::Mcs9, 4);
        let tbs_8 = transport_block_size(McsIndex::Mcs9, 8);
        assert!(tbs_2 < tbs_4, "TBS should increase with more RBs");
        assert!(tbs_4 < tbs_8);
    }

    #[test]
    fn test_tbs_increases_with_mcs() {
        let tbs_low = transport_block_size(McsIndex::Mcs0, 10);
        let tbs_mid = transport_block_size(McsIndex::Mcs12, 10);
        let tbs_high = transport_block_size(McsIndex::Mcs24, 10);
        assert!(tbs_low < tbs_mid, "Higher MCS → larger TBS");
        assert!(tbs_mid < tbs_high);
    }

    #[test]
    fn test_tbs_byte_aligned() {
        for mcs in [
            McsIndex::Mcs0,
            McsIndex::Mcs6,
            McsIndex::Mcs12,
            McsIndex::Mcs18,
            McsIndex::Mcs24,
        ] {
            let tbs = transport_block_size(mcs, 10);
            assert_eq!(tbs % 8, 0, "TBS must be byte-aligned");
        }
    }

    // --- FFT tests ---

    #[test]
    fn test_fft_dc_bin() {
        // A sequence of all 1s → FFT has DC = N, rest ≈ 0
        let n = 16usize;
        let mut buf: Vec<Complex32> = vec![Complex32::new(1.0, 0.0); n];
        fft_inplace(&mut buf, false);
        // DC bin magnitude = n
        assert!((buf[0].re - n as f32).abs() < 1e-3, "DC bin: {}", buf[0].re);
        // Other bins near zero
        for k in 1..n {
            let mag = buf[k].magnitude();
            assert!(mag < 1e-3, "Non-DC bin {} has magnitude {}", k, mag);
        }
    }

    #[test]
    fn test_ifft_is_inverse_of_fft() {
        let n = 64usize;
        let original: Vec<Complex32> = (0..n)
            .map(|i| Complex32::new((i as f32) * 0.1 - 3.0, (i as f32) * 0.05))
            .collect();
        let mut buf = original.clone();
        fft_inplace(&mut buf, false);
        fft_inplace(&mut buf, true);
        for (orig, rec) in original.iter().zip(buf.iter()) {
            assert!((orig.re - rec.re).abs() < 1e-4, "IFFT(FFT) != identity (re)");
            assert!((orig.im - rec.im).abs() < 1e-4, "IFFT(FFT) != identity (im)");
        }
    }

    // --- MCS tests ---

    #[test]
    fn test_mcs_bits_per_symbol() {
        assert_eq!(McsIndex::Mcs0.bits_per_symbol(), 2);  // QPSK
        assert_eq!(McsIndex::Mcs9.bits_per_symbol(), 2);  // QPSK
        assert_eq!(McsIndex::Mcs12.bits_per_symbol(), 4); // 16-QAM
        assert_eq!(McsIndex::Mcs18.bits_per_symbol(), 4); // 16-QAM
        assert_eq!(McsIndex::Mcs21.bits_per_symbol(), 6); // 64-QAM
        assert_eq!(McsIndex::Mcs27.bits_per_symbol(), 6); // 64-QAM
    }

    #[test]
    fn test_mcs_index_values() {
        assert_eq!(McsIndex::Mcs0.index(), 0);
        assert_eq!(McsIndex::Mcs9.index(), 9);
        assert_eq!(McsIndex::Mcs27.index(), 27);
    }

    // --- Complex32 tests ---

    #[test]
    fn test_complex32_multiply() {
        // (1 + j0) * (0 + j1) = (0 + j1)
        let a = Complex32::new(1.0, 0.0);
        let b = Complex32::new(0.0, 1.0);
        let c = a.mul(&b);
        assert!((c.re - 0.0).abs() < 1e-6);
        assert!((c.im - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_complex32_magnitude() {
        let c = Complex32::new(3.0, 4.0);
        assert!((c.magnitude() - 5.0).abs() < 1e-5);
    }
}
