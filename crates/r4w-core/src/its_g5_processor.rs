//! ITS-G5 (European Cooperative ITS) Processor
//!
//! Implements the ITS-G5 physical layer and protocol stack per:
//! - ETSI EN 302 663: ITS-G5 Access Layer Specification (IEEE 802.11p-based OFDM PHY)
//! - ETSI TS 102 687: Decentralized Congestion Control (DCC) mechanisms
//! - ETSI EN 302 636-4-1: GeoNetworking (GeoUnicast, GeoBroadcast, TopoBroadcast)
//! - ETSI EN 302 636-5-1: Basic Transport Protocol (BTP-A, BTP-B)
//! - ETSI EN 302 637-2: Cooperative Awareness Messages (CAM)
//! - ETSI EN 302 637-3: Decentralized Environmental Notification Messages (DENM)
//! - ETSI EN 302 571: Radiated Radio Equipment
//!
//! ## ITS-G5 Channel Plan (ETSI EN 302 663)
//!
//! ```text
//! ITS-G5A (Safety):     5875-5905 MHz  (channels 172,174,176,178,180)
//! ITS-G5B (Non-safety): 5905-5925 MHz  (channels 182,184)
//! ITS-G5C (Other):      5470-5725 MHz  (channels 96-116, even)
//! Control Channel (CCH): 5900 MHz (channel 180, EU default)
//! Service Channel (SCH): 5880 MHz (channel 176) or 5890 MHz (178)
//! ```
//!
//! ## OFDM PHY Parameters (IEEE 802.11p / 10 MHz variant)
//!
//! - FFT size: 64 points
//! - Data subcarriers: 48
//! - Pilot subcarriers: 4 (at ±7, ±21)
//! - Guard interval: 1.6 µs (short) / 0.8 µs (long)
//! - Symbol duration: 8 µs (vs 4 µs for 802.11a at 20 MHz)
//! - Bandwidth: 10 MHz (half the 802.11a 20 MHz)
//! - Modulations: BPSK, QPSK, 16-QAM, 64-QAM
//!
//! ## DCC State Machine (ETSI TS 102 687)
//!
//! ```text
//! Relaxed  → Active    : CBR > DCC_CBR_L1 (0.30)
//! Active   → Relaxed   : CBR < DCC_CBR_L1 AND t > T_DCC_RELAXED
//! Active   → Restrictive: CBR > DCC_CBR_L2 (0.60)
//! Restrictive → Active : CBR < DCC_CBR_L2 AND t > T_DCC_ACTIVE
//! ```

use std::collections::VecDeque;
use std::f64::consts::PI;

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------

/// ITS-G5A lower band edge (Hz) – includes all channels 172-180 per ETSI EN 302 663 Table 1
pub const ITS_G5A_LOWER_HZ: f64 = 5_855_000_000.0;
/// ITS-G5A upper band edge (Hz)
pub const ITS_G5A_UPPER_HZ: f64 = 5_905_000_000.0;
/// ITS-G5B lower band edge (Hz)
pub const ITS_G5B_LOWER_HZ: f64 = 5_905_000_000.0;
/// ITS-G5B upper band edge (Hz)
pub const ITS_G5B_UPPER_HZ: f64 = 5_925_000_000.0;
/// ITS-G5C lower band edge (Hz)
pub const ITS_G5C_LOWER_HZ: f64 = 5_470_000_000.0;
/// ITS-G5C upper band edge (Hz)
pub const ITS_G5C_UPPER_HZ: f64 = 5_725_000_000.0;

/// Channel spacing (Hz)
pub const CHANNEL_SPACING_HZ: f64 = 10_000_000.0;
/// European Control Channel (CCH) number
pub const EU_CONTROL_CHANNEL: u8 = 180;
/// Default service channel number
pub const DEFAULT_SCH: u8 = 176;

/// OFDM FFT size
pub const FFT_SIZE: usize = 64;
/// Number of data subcarriers
pub const DATA_SUBCARRIERS: usize = 48;
/// Number of pilot subcarriers
pub const PILOT_SUBCARRIERS: usize = 4;
/// Cyclic prefix samples (1.6 µs at 10 MHz sample rate = 16 samples)
pub const CP_LEN: usize = 16;
/// OFDM symbol duration (samples) = FFT_SIZE + CP_LEN
pub const SYMBOL_SAMPLES: usize = FFT_SIZE + CP_LEN;

/// DCC CBR threshold L1 (Relaxed → Active)
pub const DCC_CBR_L1: f64 = 0.30;
/// DCC CBR threshold L2 (Active → Restrictive)
pub const DCC_CBR_L2: f64 = 0.60;
/// DCC minimum inter-packet time in Relaxed state (ms)
pub const DCC_T_RELAXED_MS: f64 = 25.0;
/// DCC minimum inter-packet time in Active state (ms)
pub const DCC_T_ACTIVE_MS: f64 = 50.0;
/// DCC minimum inter-packet time in Restrictive state (ms)
pub const DCC_T_RESTRICTIVE_MS: f64 = 200.0;
/// CBR measurement window (ms)
pub const CBR_WINDOW_MS: f64 = 100.0;

/// Maximum EIRP per ETSI EN 302 571 for ITS-G5A (dBm)
pub const MAX_EIRP_ITS_G5A_DBM: f64 = 33.0;
/// Maximum EIRP for ITS-G5B (dBm)
pub const MAX_EIRP_ITS_G5B_DBM: f64 = 23.0;
/// Default Tx power in Relaxed DCC state (dBm)
pub const TX_POWER_RELAXED_DBM: f64 = 23.0;
/// Default Tx power in Active DCC state (dBm)
pub const TX_POWER_ACTIVE_DBM: f64 = 20.0;
/// Default Tx power in Restrictive DCC state (dBm)
pub const TX_POWER_RESTRICTIVE_DBM: f64 = 17.0;

/// GeoNetworking protocol version
pub const GN_PROTOCOL_VERSION: u8 = 1;
/// GeoNetworking default hop limit
pub const GN_DEFAULT_HOP_LIMIT: u8 = 10;
/// GeoNetworking maximum lifetime (s)
pub const GN_MAX_LIFETIME_S: f64 = 600.0;

/// CAM generation interval minimum (ms)
pub const CAM_T_GEN_MIN_MS: u32 = 100;
/// CAM generation interval maximum (ms)
pub const CAM_T_GEN_MAX_MS: u32 = 1000;
/// DENM repetition interval (ms)
pub const DENM_T_REPETITION_MS: u32 = 500;

// ---------------------------------------------------------------------------
// Channel Plan
// ---------------------------------------------------------------------------

/// ITS-G5 frequency band classification
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ItsBand {
    /// 5875-5905 MHz safety-of-life applications
    ItsG5A,
    /// 5905-5925 MHz non-safety applications
    ItsG5B,
    /// 5470-5725 MHz other applications
    ItsG5C,
    /// Outside defined ITS bands
    Unknown,
}

/// ITS-G5 channel descriptor per ETSI EN 302 663 Table 1
#[derive(Debug, Clone)]
pub struct ItsChannel {
    /// Channel number (e.g., 172, 174, 176, 178, 180, 182, 184)
    pub number: u8,
    /// Center frequency in Hz
    pub center_freq_hz: f64,
    /// Channel bandwidth in Hz
    pub bandwidth_hz: f64,
    /// Band classification
    pub band: ItsBand,
    /// Whether this is the control channel
    pub is_control: bool,
}

impl ItsChannel {
    /// Create channel from channel number per ETSI EN 302 663 Table 1
    pub fn from_number(ch: u8) -> Self {
        // Center frequency = 5000 MHz + ch * 5 MHz (IEEE 802.11 channel numbering)
        let center_hz = 5_000_000_000.0 + (ch as f64) * 5_000_000.0;
        let band = if center_hz >= ITS_G5A_LOWER_HZ && center_hz < ITS_G5A_UPPER_HZ {
            ItsBand::ItsG5A
        } else if center_hz >= ITS_G5B_LOWER_HZ && center_hz <= ITS_G5B_UPPER_HZ {
            ItsBand::ItsG5B
        } else if center_hz >= ITS_G5C_LOWER_HZ && center_hz <= ITS_G5C_UPPER_HZ {
            ItsBand::ItsG5C
        } else {
            ItsBand::Unknown
        };
        ItsChannel {
            number: ch,
            center_freq_hz: center_hz,
            bandwidth_hz: CHANNEL_SPACING_HZ,
            band,
            is_control: ch == EU_CONTROL_CHANNEL,
        }
    }

    /// Maximum allowed EIRP for this channel per ETSI EN 302 571
    pub fn max_eirp_dbm(&self) -> f64 {
        match self.band {
            ItsBand::ItsG5A => MAX_EIRP_ITS_G5A_DBM,
            ItsBand::ItsG5B => MAX_EIRP_ITS_G5B_DBM,
            ItsBand::ItsG5C => 30.0, // Reduced power for co-existence
            ItsBand::Unknown => 10.0, // Conservative default
        }
    }

    /// Returns list of all ITS-G5A channels per ETSI EN 302 663
    pub fn its_g5a_channels() -> Vec<ItsChannel> {
        vec![172u8, 174, 176, 178, 180]
            .into_iter()
            .map(ItsChannel::from_number)
            .collect()
    }

    /// Returns list of all ITS-G5B channels
    pub fn its_g5b_channels() -> Vec<ItsChannel> {
        vec![182u8, 184]
            .into_iter()
            .map(ItsChannel::from_number)
            .collect()
    }
}

// ---------------------------------------------------------------------------
// OFDM PHY (IEEE 802.11p / 10 MHz ITS-G5)
// ---------------------------------------------------------------------------

/// MCS (Modulation and Coding Scheme) for ITS-G5 OFDM
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ItsMcs {
    /// BPSK r=1/2, 3 Mbps
    Bpsk1_2,
    /// BPSK r=3/4, 4.5 Mbps
    Bpsk3_4,
    /// QPSK r=1/2, 6 Mbps
    Qpsk1_2,
    /// QPSK r=3/4, 9 Mbps
    Qpsk3_4,
    /// 16-QAM r=1/2, 12 Mbps
    Qam16_1_2,
    /// 16-QAM r=3/4, 18 Mbps
    Qam16_3_4,
    /// 64-QAM r=2/3, 24 Mbps
    Qam64_2_3,
    /// 64-QAM r=3/4, 27 Mbps
    Qam64_3_4,
}

impl ItsMcs {
    /// Data rate in Mbps for 10 MHz ITS-G5
    pub fn data_rate_mbps(&self) -> f64 {
        match self {
            ItsMcs::Bpsk1_2 => 3.0,
            ItsMcs::Bpsk3_4 => 4.5,
            ItsMcs::Qpsk1_2 => 6.0,
            ItsMcs::Qpsk3_4 => 9.0,
            ItsMcs::Qam16_1_2 => 12.0,
            ItsMcs::Qam16_3_4 => 18.0,
            ItsMcs::Qam64_2_3 => 24.0,
            ItsMcs::Qam64_3_4 => 27.0,
        }
    }

    /// Bits per OFDM symbol (= bits_per_subcarrier * data_subcarriers * code_rate)
    pub fn coded_bits_per_symbol(&self) -> usize {
        match self {
            ItsMcs::Bpsk1_2 | ItsMcs::Bpsk3_4 => DATA_SUBCARRIERS,         // 1 bit/carrier
            ItsMcs::Qpsk1_2 | ItsMcs::Qpsk3_4 => DATA_SUBCARRIERS * 2,     // 2 bits/carrier
            ItsMcs::Qam16_1_2 | ItsMcs::Qam16_3_4 => DATA_SUBCARRIERS * 4, // 4 bits/carrier
            ItsMcs::Qam64_2_3 | ItsMcs::Qam64_3_4 => DATA_SUBCARRIERS * 6, // 6 bits/carrier
        }
    }

    /// Coding rate numerator
    pub fn code_rate_num(&self) -> u8 {
        match self {
            ItsMcs::Bpsk1_2 | ItsMcs::Qpsk1_2 | ItsMcs::Qam16_1_2 => 1,
            ItsMcs::Bpsk3_4 | ItsMcs::Qpsk3_4 | ItsMcs::Qam16_3_4 | ItsMcs::Qam64_3_4 => 3,
            ItsMcs::Qam64_2_3 => 2,
        }
    }

    /// Coding rate denominator
    pub fn code_rate_den(&self) -> u8 {
        match self {
            ItsMcs::Bpsk1_2 | ItsMcs::Qpsk1_2 | ItsMcs::Qam16_1_2 => 2,
            ItsMcs::Bpsk3_4 | ItsMcs::Qpsk3_4 | ItsMcs::Qam16_3_4 | ItsMcs::Qam64_3_4 => 4,
            ItsMcs::Qam64_2_3 => 3,
        }
    }

    /// Data bits per OFDM symbol (after FEC decoding)
    pub fn data_bits_per_symbol(&self) -> usize {
        let coded = self.coded_bits_per_symbol();
        (coded * self.code_rate_num() as usize) / self.code_rate_den() as usize
    }

    /// Signal field encoding (4-bit rate indicator per IEEE 802.11p Table 18-6)
    pub fn signal_rate_field(&self) -> u8 {
        match self {
            ItsMcs::Bpsk1_2 => 0b1101,
            ItsMcs::Bpsk3_4 => 0b1111,
            ItsMcs::Qpsk1_2 => 0b0101,
            ItsMcs::Qpsk3_4 => 0b0111,
            ItsMcs::Qam16_1_2 => 0b1001,
            ItsMcs::Qam16_3_4 => 0b1011,
            ItsMcs::Qam64_2_3 => 0b0001,
            ItsMcs::Qam64_3_4 => 0b0011,
        }
    }
}

/// Subcarrier index mapping for 64-point ITS-G5 OFDM
/// Pilot subcarriers at indices -21, -7, +7, +21 (relative to DC)
pub fn pilot_subcarrier_indices() -> [i32; 4] {
    [-21, -7, 7, 21]
}

/// Data subcarrier indices (48 carriers, excluding pilots and DC null)
pub fn data_subcarrier_indices() -> Vec<i32> {
    let pilots: [i32; 4] = pilot_subcarrier_indices();
    let mut indices = Vec::with_capacity(DATA_SUBCARRIERS);
    for i in -26i32..=26i32 {
        if i == 0 {
            continue; // DC null
        }
        if pilots.contains(&i) {
            continue; // Skip pilots
        }
        indices.push(i);
    }
    indices
}

/// ITS-G5 OFDM PHY frame processor
#[derive(Debug, Clone)]
pub struct ItsG5Phy {
    /// Active channel
    pub channel: ItsChannel,
    /// Current MCS
    pub mcs: ItsMcs,
    /// Nominal sample rate (10 MHz)
    pub sample_rate_hz: f64,
    /// Pilot phase tracking state [f64; 4]
    pilot_phases: [f64; 4],
}

impl ItsG5Phy {
    /// Create new ITS-G5 PHY for given channel
    pub fn new(channel_num: u8) -> Self {
        ItsG5Phy {
            channel: ItsChannel::from_number(channel_num),
            mcs: ItsMcs::Qpsk1_2,
            sample_rate_hz: 10_000_000.0,
            pilot_phases: [1.0, 1.0, -1.0, 1.0], // Pilot polarity per 802.11p
        }
    }

    /// Generate OFDM symbol from complex frequency-domain subcarrier values.
    /// Input length must be FFT_SIZE (64). Returns time-domain samples with CP.
    pub fn ifft_and_add_cp(&self, freq_domain: &[(f64, f64)]) -> Vec<(f64, f64)> {
        assert_eq!(freq_domain.len(), FFT_SIZE);
        let n = FFT_SIZE;
        let mut time = vec![(0.0f64, 0.0f64); n];

        // DFT synthesis: x[t] = (1/N) * sum_k X[k] * exp(j*2*pi*k*t/N)
        for t in 0..n {
            let mut re = 0.0f64;
            let mut im = 0.0f64;
            for k in 0..n {
                let angle = 2.0 * PI * (k as f64) * (t as f64) / (n as f64);
                re += freq_domain[k].0 * angle.cos() - freq_domain[k].1 * angle.sin();
                im += freq_domain[k].0 * angle.sin() + freq_domain[k].1 * angle.cos();
            }
            time[t] = (re / n as f64, im / n as f64);
        }

        // Prepend cyclic prefix (last CP_LEN samples)
        let mut output = Vec::with_capacity(SYMBOL_SAMPLES);
        for i in (n - CP_LEN)..n {
            output.push(time[i]);
        }
        output.extend_from_slice(&time);
        output
    }

    /// Demodulate OFDM symbol: remove CP and perform FFT
    pub fn remove_cp_and_fft(&self, samples: &[(f64, f64)]) -> Vec<(f64, f64)> {
        assert!(samples.len() >= SYMBOL_SAMPLES);
        // Remove CP
        let data = &samples[CP_LEN..CP_LEN + FFT_SIZE];
        let n = FFT_SIZE;
        let mut freq = vec![(0.0f64, 0.0f64); n];

        // DFT analysis: X[k] = sum_t x[t] * exp(-j*2*pi*k*t/N)
        for k in 0..n {
            let mut re = 0.0f64;
            let mut im = 0.0f64;
            for t in 0..n {
                let angle = 2.0 * PI * (k as f64) * (t as f64) / (n as f64);
                re += data[t].0 * angle.cos() + data[t].1 * angle.sin();
                im += -data[t].0 * angle.sin() + data[t].1 * angle.cos();
            }
            freq[k] = (re, im);
        }
        freq
    }

    /// Map payload bytes to OFDM subcarrier symbols using BPSK or QPSK
    /// Returns 48 complex values (one per data subcarrier)
    pub fn modulate_bytes(&self, payload: &[u8]) -> Vec<(f64, f64)> {
        let mut symbols = Vec::with_capacity(DATA_SUBCARRIERS);
        let bits: Vec<u8> = payload.iter().flat_map(|&b| (0..8).rev().map(move |i| (b >> i) & 1)).collect();
        let mut bit_idx = 0;

        for _ in 0..DATA_SUBCARRIERS {
            let sym = match self.mcs {
                ItsMcs::Bpsk1_2 | ItsMcs::Bpsk3_4 => {
                    let b = if bit_idx < bits.len() { bits[bit_idx] } else { 0 };
                    bit_idx += 1;
                    // BPSK: 0 -> +1, 1 -> -1
                    if b == 0 { (1.0f64, 0.0f64) } else { (-1.0f64, 0.0f64) }
                }
                ItsMcs::Qpsk1_2 | ItsMcs::Qpsk3_4 => {
                    let b0 = if bit_idx < bits.len() { bits[bit_idx] } else { 0 };
                    let b1 = if bit_idx + 1 < bits.len() { bits[bit_idx + 1] } else { 0 };
                    bit_idx += 2;
                    // Gray-coded QPSK
                    let re = if b0 == 0 { 1.0f64 / 2.0f64.sqrt() } else { -1.0f64 / 2.0f64.sqrt() };
                    let im = if b1 == 0 { 1.0f64 / 2.0f64.sqrt() } else { -1.0f64 / 2.0f64.sqrt() };
                    (re, im)
                }
                _ => {
                    // 16-QAM / 64-QAM simplified: QPSK fallback for test purposes
                    let b0 = if bit_idx < bits.len() { bits[bit_idx] } else { 0 };
                    bit_idx += 1;
                    if b0 == 0 { (1.0f64, 0.0f64) } else { (-1.0f64, 0.0f64) }
                }
            };
            symbols.push(sym);
        }
        symbols
    }

    /// Demodulate data subcarrier symbols to bytes
    pub fn demodulate_symbols(&self, symbols: &[(f64, f64)]) -> Vec<u8> {
        let mut bits = Vec::new();
        for &(re, im) in symbols {
            match self.mcs {
                ItsMcs::Bpsk1_2 | ItsMcs::Bpsk3_4 => {
                    bits.push(if re >= 0.0 { 0u8 } else { 1u8 });
                }
                ItsMcs::Qpsk1_2 | ItsMcs::Qpsk3_4 => {
                    bits.push(if re >= 0.0 { 0u8 } else { 1u8 });
                    bits.push(if im >= 0.0 { 0u8 } else { 1u8 });
                }
                _ => {
                    bits.push(if re >= 0.0 { 0u8 } else { 1u8 });
                }
            }
        }
        // Pack bits into bytes
        bits.chunks(8).map(|chunk| {
            let mut b = 0u8;
            for (i, &bit) in chunk.iter().enumerate() {
                b |= bit << (7 - i);
            }
            b
        }).collect()
    }

    /// Calculate channel CBR (Channel Busy Ratio) from power measurements
    /// cbr_samples: Vec of (power_dBm, threshold_dBm) per sample
    pub fn measure_cbr(samples: &[(f64, f64)]) -> f64 {
        if samples.is_empty() {
            return 0.0;
        }
        let busy = samples.iter().filter(|&&(p, th)| p > th).count();
        busy as f64 / samples.len() as f64
    }

    /// Estimate SNR from pilot subcarriers (simplified Least-Squares)
    /// pilots: 4 received pilot values, known_pilots: 4 ideal pilot values
    pub fn estimate_snr_from_pilots(received: &[(f64, f64)], known: &[(f64, f64)]) -> f64 {
        assert_eq!(received.len(), known.len());
        let mut sig_power = 0.0f64;
        let mut noise_power = 0.0f64;
        for (&(r_re, r_im), &(k_re, k_im)) in received.iter().zip(known.iter()) {
            let s_re = k_re;
            let s_im = k_im;
            let e_re = r_re - s_re;
            let e_im = r_im - s_im;
            sig_power += s_re * s_re + s_im * s_im;
            noise_power += e_re * e_re + e_im * e_im;
        }
        if noise_power < 1e-30 {
            return 40.0; // High SNR limit
        }
        10.0 * (sig_power / noise_power).log10()
    }
}

// ---------------------------------------------------------------------------
// DCC – Decentralized Congestion Control (ETSI TS 102 687)
// ---------------------------------------------------------------------------

/// DCC operational state machine state
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DccState {
    /// Low channel load – maximum transmission allowed
    Relaxed,
    /// Moderate channel load – rate/power reduction applied
    Active,
    /// High channel load – severe restrictions
    Restrictive,
}

impl DccState {
    /// Minimum inter-packet time (ms) for this state
    pub fn min_ipt_ms(&self) -> f64 {
        match self {
            DccState::Relaxed => DCC_T_RELAXED_MS,
            DccState::Active => DCC_T_ACTIVE_MS,
            DccState::Restrictive => DCC_T_RESTRICTIVE_MS,
        }
    }

    /// Maximum Tx power (dBm) for this state
    pub fn tx_power_dbm(&self) -> f64 {
        match self {
            DccState::Relaxed => TX_POWER_RELAXED_DBM,
            DccState::Active => TX_POWER_ACTIVE_DBM,
            DccState::Restrictive => TX_POWER_RESTRICTIVE_DBM,
        }
    }

    /// Maximum duty cycle (0-1) for this state
    pub fn max_duty_cycle(&self) -> f64 {
        match self {
            DccState::Relaxed => 1.0,
            DccState::Active => 0.5,
            DccState::Restrictive => 0.1,
        }
    }
}

/// Channel load measurement entry (time, power_dBm)
#[derive(Debug, Clone, Copy)]
struct ChannelSample {
    time_ms: f64,
    power_dbm: f64,
}

/// DCC Engine per ETSI TS 102 687 Section 5
#[derive(Debug)]
pub struct DccEngine {
    /// Current DCC state
    pub state: DccState,
    /// Current measured CBR (0.0 - 1.0)
    pub cbr: f64,
    /// Time of last state transition (ms)
    state_change_time_ms: f64,
    /// Channel busy/idle sample history (sliding window)
    samples: VecDeque<ChannelSample>,
    /// Channel sensing threshold (dBm) – busy if above
    pub sensing_threshold_dbm: f64,
    /// Current simulation time (ms)
    current_time_ms: f64,
    /// Last transmission time (ms)
    last_tx_time_ms: f64,
    /// Accumulated on-air time in current window (ms)
    on_air_time_ms: f64,
}

impl DccEngine {
    /// Create DCC engine with given sensing threshold
    pub fn new(sensing_threshold_dbm: f64) -> Self {
        DccEngine {
            state: DccState::Relaxed,
            cbr: 0.0,
            state_change_time_ms: 0.0,
            samples: VecDeque::new(),
            sensing_threshold_dbm,
            current_time_ms: 0.0,
            last_tx_time_ms: -1000.0, // Initialized well in past
            on_air_time_ms: 0.0,
        }
    }

    /// Advance simulation time and add channel power sample
    pub fn update(&mut self, time_ms: f64, channel_power_dbm: f64) {
        self.current_time_ms = time_ms;
        self.samples.push_back(ChannelSample {
            time_ms,
            power_dbm: channel_power_dbm,
        });
        // Remove samples outside CBR window
        let window_start = time_ms - CBR_WINDOW_MS;
        while self.samples.front().map_or(false, |s| s.time_ms < window_start) {
            self.samples.pop_front();
        }
        // Compute CBR
        let total = self.samples.len();
        let busy = self.samples.iter().filter(|s| s.power_dbm >= self.sensing_threshold_dbm).count();
        self.cbr = if total > 0 { busy as f64 / total as f64 } else { 0.0 };
        // Run state machine
        self.update_state();
    }

    fn update_state(&mut self) {
        let time_in_state = self.current_time_ms - self.state_change_time_ms;
        let new_state = match self.state {
            DccState::Relaxed => {
                if self.cbr > DCC_CBR_L1 {
                    DccState::Active
                } else {
                    DccState::Relaxed
                }
            }
            DccState::Active => {
                if self.cbr > DCC_CBR_L2 {
                    DccState::Restrictive
                } else if self.cbr < DCC_CBR_L1 && time_in_state > 200.0 {
                    DccState::Relaxed
                } else {
                    DccState::Active
                }
            }
            DccState::Restrictive => {
                if self.cbr < DCC_CBR_L2 && time_in_state > 200.0 {
                    DccState::Active
                } else {
                    DccState::Restrictive
                }
            }
        };
        if new_state != self.state {
            self.state = new_state;
            self.state_change_time_ms = self.current_time_ms;
        }
    }

    /// Check if transmission is allowed now (respects minimum IPT)
    pub fn can_transmit(&self) -> bool {
        let elapsed = self.current_time_ms - self.last_tx_time_ms;
        elapsed >= self.state.min_ipt_ms()
    }

    /// Record a transmission at current time
    pub fn record_transmission(&mut self, duration_ms: f64) {
        self.last_tx_time_ms = self.current_time_ms;
        self.on_air_time_ms += duration_ms;
    }

    /// Current allowed Tx power in dBm
    pub fn allowed_tx_power_dbm(&self) -> f64 {
        self.state.tx_power_dbm()
    }

    /// Duty cycle in current measurement window (0-1)
    pub fn duty_cycle(&self) -> f64 {
        if CBR_WINDOW_MS > 0.0 {
            (self.on_air_time_ms / CBR_WINDOW_MS).min(1.0)
        } else {
            0.0
        }
    }
}

// ---------------------------------------------------------------------------
// GeoNetworking (ETSI EN 302 636-4-1)
// ---------------------------------------------------------------------------

/// GeoNetworking Next Header field values
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GnNextHeader {
    /// Any transport protocol
    Any = 0,
    /// Basic Transport Protocol (BTP-A)
    BtpA = 1,
    /// Basic Transport Protocol (BTP-B)
    BtpB = 2,
    /// IPv6
    Ipv6 = 3,
}

/// GeoNetworking Header Type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum GnHeaderType {
    Any = 0,
    Beacon = 1,
    GeoUnicast = 2,
    GeoBroadcastCircle = 4,
    GeoBroadcastRect = 5,
    GeoBroadcastElips = 6,
    TopoScopedBroadcast1Hop = 8,
    TopoScopedBroadcastMultiHop = 9,
    LocationServiceRequest = 10,
    LocationServiceReply = 11,
}

/// GeoNetworking Long Position Vector (LPV)
#[derive(Debug, Clone, Copy)]
pub struct LongPositionVector {
    /// GN_ADDR (station type + country code + MAC) - simplified as 64-bit
    pub gn_addr: u64,
    /// Timestamp (TAI milliseconds since 2004-01-01)
    pub timestamp: u32,
    /// Latitude in 1/10 microdegrees (WGS84)
    pub latitude: i32,
    /// Longitude in 1/10 microdegrees (WGS84)
    pub longitude: i32,
    /// Position accuracy indicator
    pub pai: bool,
    /// Speed in 0.01 m/s (signed)
    pub speed: i16,
    /// Heading in 0.1 degree (0-3599)
    pub heading: u16,
}

impl LongPositionVector {
    /// Create LPV from latitude/longitude in degrees and speed/heading
    pub fn new(gn_addr: u64, lat_deg: f64, lon_deg: f64, speed_ms: f64, heading_deg: f64) -> Self {
        LongPositionVector {
            gn_addr,
            timestamp: 0, // Simplified
            latitude: (lat_deg * 1_000_000.0 / 10.0) as i32,
            longitude: (lon_deg * 1_000_000.0 / 10.0) as i32,
            pai: true,
            speed: (speed_ms * 100.0).clamp(-32768.0, 32767.0) as i16,
            heading: ((heading_deg % 360.0) * 10.0) as u16,
        }
    }

    /// Get latitude in degrees
    pub fn latitude_deg(&self) -> f64 {
        self.latitude as f64 * 10.0 / 1_000_000.0
    }

    /// Get longitude in degrees
    pub fn longitude_deg(&self) -> f64 {
        self.longitude as f64 * 10.0 / 1_000_000.0
    }

    /// Get speed in m/s
    pub fn speed_ms(&self) -> f64 {
        self.speed as f64 / 100.0
    }

    /// Serialize to bytes (24-byte LPV per ETSI EN 302 636-4-1)
    pub fn to_bytes(&self) -> [u8; 24] {
        let mut buf = [0u8; 24];
        buf[0..8].copy_from_slice(&self.gn_addr.to_be_bytes());
        buf[8..12].copy_from_slice(&self.timestamp.to_be_bytes());
        buf[12..16].copy_from_slice(&self.latitude.to_be_bytes());
        buf[16..20].copy_from_slice(&self.longitude.to_be_bytes());
        let pai_speed = ((self.pai as u16) << 15) | (self.speed as u16 & 0x7FFF);
        buf[20..22].copy_from_slice(&pai_speed.to_be_bytes());
        buf[22..24].copy_from_slice(&self.heading.to_be_bytes());
        buf
    }

    /// Deserialize from bytes
    pub fn from_bytes(buf: &[u8; 24]) -> Self {
        let gn_addr = u64::from_be_bytes(buf[0..8].try_into().unwrap());
        let timestamp = u32::from_be_bytes(buf[8..12].try_into().unwrap());
        let latitude = i32::from_be_bytes(buf[12..16].try_into().unwrap());
        let longitude = i32::from_be_bytes(buf[16..20].try_into().unwrap());
        let pai_speed = u16::from_be_bytes(buf[20..22].try_into().unwrap());
        let pai = (pai_speed >> 15) != 0;
        let speed = (pai_speed & 0x7FFF) as i16;
        let heading = u16::from_be_bytes(buf[22..24].try_into().unwrap());
        LongPositionVector { gn_addr, timestamp, latitude, longitude, pai, speed, heading }
    }
}

/// GeoNetworking Basic Header (4 bytes per ETSI EN 302 636-4-1 Clause 8.6)
#[derive(Debug, Clone, Copy)]
pub struct GnBasicHeader {
    /// Protocol version (must be 1)
    pub version: u8,
    /// Next Header type
    pub next_header: GnNextHeader,
    /// Remaining hop limit
    pub hop_limit: u8,
    /// Lifetime encoded as multiplier * 50ms or 1s
    pub lifetime: u8,
}

impl GnBasicHeader {
    pub fn new(next_header: GnNextHeader, lifetime: u8, hop_limit: u8) -> Self {
        GnBasicHeader {
            version: GN_PROTOCOL_VERSION,
            next_header,
            hop_limit,
            lifetime,
        }
    }

    /// Lifetime in seconds (approximate)
    pub fn lifetime_secs(&self) -> f64 {
        let base = self.lifetime & 0x3F;
        let mult = self.lifetime >> 6;
        let base_ms = base as f64 * 50.0;
        match mult {
            0 => base_ms / 1000.0,
            1 => base_ms * 10.0 / 1000.0,
            2 => base_ms * 100.0 / 1000.0,
            _ => base_ms * 1000.0 / 1000.0,
        }
    }

    pub fn to_bytes(&self) -> [u8; 4] {
        let nh = self.next_header as u8;
        [
            (self.version << 4) | (nh & 0x0F),
            0, // Reserved
            self.lifetime,
            self.hop_limit,
        ]
    }

    pub fn from_bytes(buf: &[u8; 4]) -> Self {
        let version = buf[0] >> 4;
        let next_header = match buf[0] & 0x0F {
            1 => GnNextHeader::BtpA,
            2 => GnNextHeader::BtpB,
            3 => GnNextHeader::Ipv6,
            _ => GnNextHeader::Any,
        };
        GnBasicHeader {
            version,
            next_header,
            hop_limit: buf[3],
            lifetime: buf[2],
        }
    }
}

/// GeoNetworking Common Header (8 bytes per ETSI EN 302 636-4-1 Clause 8.7)
#[derive(Debug, Clone, Copy)]
pub struct GnCommonHeader {
    /// Next Header after GeoNetworking
    pub next_header: GnNextHeader,
    /// Header type
    pub header_type: GnHeaderType,
    /// Traffic class (SCF, channel offload, TC ID)
    pub traffic_class: u8,
    /// Flags (mobile, reserved)
    pub flags: u8,
    /// Payload length in octets
    pub payload_length: u16,
    /// Maximum hop limit
    pub max_hop_limit: u8,
    /// Reserved
    reserved: u8,
}

impl GnCommonHeader {
    pub fn new(
        next_header: GnNextHeader,
        header_type: GnHeaderType,
        traffic_class: u8,
        payload_length: u16,
    ) -> Self {
        GnCommonHeader {
            next_header,
            header_type,
            traffic_class,
            flags: 0,
            payload_length,
            max_hop_limit: GN_DEFAULT_HOP_LIMIT,
            reserved: 0,
        }
    }

    pub fn to_bytes(&self) -> [u8; 8] {
        let nh = self.next_header as u8;
        let ht = self.header_type as u8;
        let mut buf = [0u8; 8];
        buf[0] = (nh << 4) | (ht >> 1);
        buf[1] = (ht & 0x01) << 7 | 0; // HS in bits 6-4 (simplified)
        buf[2] = self.flags;
        buf[3] = self.traffic_class;
        buf[4..6].copy_from_slice(&self.payload_length.to_be_bytes());
        buf[6] = self.max_hop_limit;
        buf[7] = self.reserved;
        buf
    }
}

/// Geo Area definition for GeoBroadcast
#[derive(Debug, Clone, Copy)]
pub enum GeoArea {
    /// Circular area: (latitude_deg, longitude_deg, radius_m)
    Circle {
        lat: f64,
        lon: f64,
        radius_m: f64,
    },
    /// Rectangular area: (center_lat, center_lon, dist_a_m, dist_b_m, angle_deg)
    Rectangle {
        lat: f64,
        lon: f64,
        dist_a_m: f64,
        dist_b_m: f64,
        angle_deg: f64,
    },
    /// Ellipsoidal area: (center_lat, center_lon, semi_major_m, semi_minor_m, angle_deg)
    Ellipse {
        lat: f64,
        lon: f64,
        semi_major_m: f64,
        semi_minor_m: f64,
        angle_deg: f64,
    },
}

impl GeoArea {
    /// Check if given position is inside this geo area (simplified 2D check)
    pub fn contains(&self, lat_deg: f64, lon_deg: f64) -> bool {
        const EARTH_R: f64 = 6_371_000.0; // meters
        match self {
            GeoArea::Circle { lat, lon, radius_m } => {
                let dlat = (lat_deg - lat).to_radians();
                let dlon = (lon_deg - lon).to_radians();
                let a = (dlat / 2.0).sin().powi(2)
                    + lat_deg.to_radians().cos() * lat.to_radians().cos() * (dlon / 2.0).sin().powi(2);
                let dist = 2.0 * EARTH_R * a.sqrt().atan2((1.0 - a).sqrt());
                dist <= *radius_m
            }
            GeoArea::Rectangle { lat, lon, dist_a_m, dist_b_m, angle_deg } => {
                let dlat_m = (lat_deg - lat).to_radians() * EARTH_R;
                let dlon_m = (lon_deg - lon).to_radians() * EARTH_R * lat.to_radians().cos();
                let ang = angle_deg.to_radians();
                let x = dlat_m * ang.cos() + dlon_m * ang.sin();
                let y = -dlat_m * ang.sin() + dlon_m * ang.cos();
                x.abs() <= *dist_a_m && y.abs() <= *dist_b_m
            }
            GeoArea::Ellipse { lat, lon, semi_major_m, semi_minor_m, angle_deg } => {
                let dlat_m = (lat_deg - lat).to_radians() * EARTH_R;
                let dlon_m = (lon_deg - lon).to_radians() * EARTH_R * lat.to_radians().cos();
                let ang = angle_deg.to_radians();
                let x = dlat_m * ang.cos() + dlon_m * ang.sin();
                let y = -dlat_m * ang.sin() + dlon_m * ang.cos();
                (x / semi_major_m).powi(2) + (y / semi_minor_m).powi(2) <= 1.0
            }
        }
    }

    /// Header type for this geo area
    pub fn header_type(&self) -> GnHeaderType {
        match self {
            GeoArea::Circle { .. } => GnHeaderType::GeoBroadcastCircle,
            GeoArea::Rectangle { .. } => GnHeaderType::GeoBroadcastRect,
            GeoArea::Ellipse { .. } => GnHeaderType::GeoBroadcastElips,
        }
    }

    /// Encode geo area into 12 bytes (lat, lon, dist_a, dist_b, angle, reserved)
    pub fn to_bytes(&self) -> [u8; 12] {
        let (lat, lon, a, b, angle) = match self {
            GeoArea::Circle { lat, lon, radius_m } => (*lat, *lon, *radius_m, *radius_m, 0.0),
            GeoArea::Rectangle { lat, lon, dist_a_m, dist_b_m, angle_deg } =>
                (*lat, *lon, *dist_a_m, *dist_b_m, *angle_deg),
            GeoArea::Ellipse { lat, lon, semi_major_m, semi_minor_m, angle_deg } =>
                (*lat, *lon, *semi_major_m, *semi_minor_m, *angle_deg),
        };
        let lat_enc = (lat * 1e7) as i32;
        let lon_enc = (lon * 1e7) as i32;
        let a_enc = a as u16;
        let b_enc = b as u16;
        let angle_enc = (angle / 360.0 * 65535.0) as u16;
        let mut buf = [0u8; 12];
        buf[0..4].copy_from_slice(&lat_enc.to_be_bytes());
        buf[4..8].copy_from_slice(&lon_enc.to_be_bytes());
        buf[8..10].copy_from_slice(&a_enc.to_be_bytes());
        buf[10..12].copy_from_slice(&b_enc.to_be_bytes());
        // angle not stored in fixed 12 bytes here; simplified
        let _ = angle_enc;
        buf
    }
}

/// GeoUnicast packet (addressed to specific GN_ADDR)
#[derive(Debug, Clone)]
pub struct GeoUnicastPacket {
    pub basic_header: GnBasicHeader,
    pub common_header: GnCommonHeader,
    pub source_lpv: LongPositionVector,
    pub dest_gn_addr: u64,
    pub sequence_number: u16,
    pub payload: Vec<u8>,
}

/// GeoBroadcast packet (addressed to geographical area)
#[derive(Debug, Clone)]
pub struct GeoBroadcastPacket {
    pub basic_header: GnBasicHeader,
    pub common_header: GnCommonHeader,
    pub source_lpv: LongPositionVector,
    pub area: GeoArea,
    pub sequence_number: u16,
    pub payload: Vec<u8>,
}

impl GeoBroadcastPacket {
    /// Create a GeoBroadcast packet
    pub fn new(source_lpv: LongPositionVector, area: GeoArea, payload: Vec<u8>) -> Self {
        let area_ht = area.header_type();
        let common = GnCommonHeader::new(
            GnNextHeader::BtpB,
            area_ht,
            0,
            payload.len() as u16,
        );
        let basic = GnBasicHeader::new(GnNextHeader::BtpB, 0x3F, GN_DEFAULT_HOP_LIMIT);
        GeoBroadcastPacket {
            basic_header: basic,
            common_header: common,
            source_lpv,
            area,
            sequence_number: 0,
            payload,
        }
    }

    /// Check if given position is inside the destination area
    pub fn is_in_area(&self, lat_deg: f64, lon_deg: f64) -> bool {
        self.area.contains(lat_deg, lon_deg)
    }

    /// Serialize the full GeoNetworking packet to bytes
    pub fn serialize(&self) -> Vec<u8> {
        let mut buf = Vec::new();
        buf.extend_from_slice(&self.basic_header.to_bytes());
        buf.extend_from_slice(&self.common_header.to_bytes());
        buf.extend_from_slice(&self.source_lpv.to_bytes());
        buf.extend_from_slice(&self.sequence_number.to_be_bytes());
        buf.extend_from_slice(&[0u8; 2]); // Reserved
        buf.extend_from_slice(&self.area.to_bytes());
        buf.extend_from_slice(&self.payload);
        buf
    }
}

// ---------------------------------------------------------------------------
// BTP – Basic Transport Protocol (ETSI EN 302 636-5-1)
// ---------------------------------------------------------------------------

/// BTP port numbers (ETSI EN 302 636-5-1 Annex B)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum BtpPort {
    /// CAM – Cooperative Awareness Messages
    Cam = 2001,
    /// DENM – Decentralized Environmental Notification Messages
    Denm = 2002,
    /// MAP/SPATEM – MAP and Signal Phase & Timing
    MapSpatem = 2003,
    /// IVIM – Infrastructure to Vehicle Information
    Ivim = 2004,
    /// SREM/SSEM – Signal Request / Status Extended Message
    SremSsem = 2006,
    /// CPM – Collective Perception Messages
    Cpm = 2009,
    /// IMZM – Interference Management Zone Messages
    Imzm = 2010,
    /// VAM – VRU Awareness Messages
    Vam = 2018,
    /// RTCMEM – RTCM Extended Messages
    Rtcmem = 2019,
}

/// BTP-A header (interactive, 4 bytes: dest_port + dest_port_info)
#[derive(Debug, Clone, Copy)]
pub struct BtpAHeader {
    /// Destination port number
    pub dest_port: u16,
    /// Destination port info (application defined)
    pub dest_port_info: u16,
}

impl BtpAHeader {
    pub fn new(dest_port: BtpPort) -> Self {
        BtpAHeader {
            dest_port: dest_port as u16,
            dest_port_info: 0,
        }
    }

    pub fn to_bytes(&self) -> [u8; 4] {
        let mut buf = [0u8; 4];
        buf[0..2].copy_from_slice(&self.dest_port.to_be_bytes());
        buf[2..4].copy_from_slice(&self.dest_port_info.to_be_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8; 4]) -> Self {
        BtpAHeader {
            dest_port: u16::from_be_bytes(buf[0..2].try_into().unwrap()),
            dest_port_info: u16::from_be_bytes(buf[2..4].try_into().unwrap()),
        }
    }
}

/// BTP-B header (non-interactive, 4 bytes: dest_port + dest_port_info)
#[derive(Debug, Clone, Copy)]
pub struct BtpBHeader {
    /// Destination port number
    pub dest_port: u16,
    /// Destination port info (application defined)
    pub dest_port_info: u16,
}

impl BtpBHeader {
    pub fn new(dest_port: BtpPort) -> Self {
        BtpBHeader {
            dest_port: dest_port as u16,
            dest_port_info: 0,
        }
    }

    pub fn to_bytes(&self) -> [u8; 4] {
        let mut buf = [0u8; 4];
        buf[0..2].copy_from_slice(&self.dest_port.to_be_bytes());
        buf[2..4].copy_from_slice(&self.dest_port_info.to_be_bytes());
        buf
    }

    pub fn from_bytes(buf: &[u8; 4]) -> Self {
        BtpBHeader {
            dest_port: u16::from_be_bytes(buf[0..2].try_into().unwrap()),
            dest_port_info: u16::from_be_bytes(buf[2..4].try_into().unwrap()),
        }
    }
}

// ---------------------------------------------------------------------------
// CAM – Cooperative Awareness Message (ETSI EN 302 637-2)
// ---------------------------------------------------------------------------

/// ITS station type
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StationType {
    Unknown = 0,
    Pedestrian = 1,
    Cyclist = 2,
    Moped = 3,
    Motorcycle = 4,
    PassengerCar = 5,
    Bus = 6,
    LightTruck = 7,
    HeavyTruck = 8,
    Trailer = 9,
    SpecialVehicle = 10,
    Tram = 11,
    RoadSideUnit = 15,
}

/// Vehicle drive direction
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DriveDirection {
    Forward = 0,
    Backward = 1,
    Unavailable = 2,
}

/// Vehicle role (for LF container)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleRole {
    Default = 0,
    PublicTransport = 1,
    SpecialTransport = 2,
    DangerousGoods = 3,
    RoadWork = 4,
    Rescue = 5,
    Emergency = 6,
    SafetyCar = 7,
}

/// CAM Basic Container
#[derive(Debug, Clone)]
pub struct CamBasicContainer {
    /// ITS station type
    pub station_type: StationType,
    /// Reference position (LPV)
    pub reference_position: LongPositionVector,
}

/// CAM High-Frequency Container (mandatory for vehicles)
#[derive(Debug, Clone)]
pub struct CamHighFreqContainer {
    /// Heading: 0.1 degree (0-3599), 3600 = unavailable
    pub heading: u16,
    /// Speed: 0.01 m/s (0-16382), 16383 = unavailable
    pub speed: u16,
    /// Drive direction
    pub drive_direction: DriveDirection,
    /// Vehicle length: 0.1m (1-1022), 1023 = unavailable
    pub vehicle_length: u16,
    /// Vehicle width: 0.1m (1-61), 62 = unavailable
    pub vehicle_width: u8,
    /// Longitudinal acceleration: 0.1 m/s² (-160 to +160), 161 = unavailable
    pub longitudinal_accel: i16,
    /// Curvature: 0.0001 m⁻¹ (-1023 to +1023), 1023 = unavailable
    pub curvature: i16,
    /// Yaw rate: 0.01 deg/s (-32766 to +32766), 32767 = unavailable
    pub yaw_rate: i16,
}

impl CamHighFreqContainer {
    pub fn new_vehicle(
        heading_deg: f64,
        speed_ms: f64,
        direction: DriveDirection,
        length_m: f64,
        width_m: f64,
    ) -> Self {
        CamHighFreqContainer {
            heading: (heading_deg * 10.0).clamp(0.0, 3599.0) as u16,
            speed: (speed_ms * 100.0).clamp(0.0, 16382.0) as u16,
            drive_direction: direction,
            vehicle_length: (length_m * 10.0).clamp(1.0, 1022.0) as u16,
            vehicle_width: (width_m * 10.0).clamp(1.0, 61.0) as u8,
            longitudinal_accel: 161, // unavailable
            curvature: 1023,         // unavailable
            yaw_rate: 32767,         // unavailable
        }
    }
}

/// CAM Low-Frequency Container (optional)
#[derive(Debug, Clone)]
pub struct CamLowFreqContainer {
    pub vehicle_role: VehicleRole,
    /// Exterior lights bitmask (low beam, high beam, etc.)
    pub exterior_lights: u8,
    /// Path history count (0..40)
    pub path_history_length: u8,
}

impl CamLowFreqContainer {
    pub fn new(role: VehicleRole, lights: u8) -> Self {
        CamLowFreqContainer {
            vehicle_role: role,
            exterior_lights: lights,
            path_history_length: 0,
        }
    }
}

/// Full CAM per ETSI EN 302 637-2
#[derive(Debug, Clone)]
pub struct CooperativeAwarenessMessage {
    /// ITS PDU Header
    pub protocol_version: u8,
    pub message_id: u8, // 2 = CAM
    pub station_id: u32,
    /// Generation delta time (mod 65536, unit 1ms)
    pub generation_delta_time: u16,
    pub basic_container: CamBasicContainer,
    pub high_freq_container: CamHighFreqContainer,
    pub low_freq_container: Option<CamLowFreqContainer>,
}

impl CooperativeAwarenessMessage {
    /// Create a CAM for a vehicle
    pub fn new_vehicle(
        station_id: u32,
        lpv: LongPositionVector,
        heading_deg: f64,
        speed_ms: f64,
        time_ms: u32,
    ) -> Self {
        let hf = CamHighFreqContainer::new_vehicle(
            heading_deg,
            speed_ms,
            DriveDirection::Forward,
            4.5, // default car length
            1.8, // default car width
        );
        CooperativeAwarenessMessage {
            protocol_version: 2,
            message_id: 2,
            station_id,
            generation_delta_time: (time_ms % 65536) as u16,
            basic_container: CamBasicContainer {
                station_type: StationType::PassengerCar,
                reference_position: lpv,
            },
            high_freq_container: hf,
            low_freq_container: None,
        }
    }

    /// Serialize to simplified byte representation
    pub fn serialize(&self) -> Vec<u8> {
        let mut buf = Vec::new();
        buf.push(self.protocol_version);
        buf.push(self.message_id);
        buf.extend_from_slice(&self.station_id.to_be_bytes());
        buf.extend_from_slice(&self.generation_delta_time.to_be_bytes());
        // Basic container
        buf.push(self.basic_container.station_type as u8);
        buf.extend_from_slice(&self.basic_container.reference_position.to_bytes());
        // HF container
        buf.extend_from_slice(&self.high_freq_container.heading.to_be_bytes());
        buf.extend_from_slice(&self.high_freq_container.speed.to_be_bytes());
        buf.push(self.high_freq_container.drive_direction as u8);
        buf.extend_from_slice(&self.high_freq_container.vehicle_length.to_be_bytes());
        buf.push(self.high_freq_container.vehicle_width);
        // LF container (optional)
        if let Some(ref lf) = self.low_freq_container {
            buf.push(1); // present flag
            buf.push(lf.vehicle_role as u8);
            buf.push(lf.exterior_lights);
            buf.push(lf.path_history_length);
        } else {
            buf.push(0); // absent flag
        }
        buf
    }

    /// Compute CAM generation interval based on dynamics (ETSI EN 302 637-2 Annex B)
    /// Returns interval in milliseconds
    pub fn adaptive_generation_interval(
        speed_ms: f64,
        heading_change_deg: f64,
        accel_ms2: f64,
    ) -> u32 {
        // High-frequency trigger conditions
        if speed_ms > 0.5 && (heading_change_deg.abs() > 4.0 || accel_ms2.abs() > 0.5) {
            CAM_T_GEN_MIN_MS
        } else if speed_ms < 0.1 {
            CAM_T_GEN_MAX_MS
        } else {
            // Linear interpolation based on speed
            let t = ((speed_ms / 25.0).clamp(0.0, 1.0) * (CAM_T_GEN_MAX_MS - CAM_T_GEN_MIN_MS) as f64) as u32;
            CAM_T_GEN_MAX_MS - t
        }
    }
}

// ---------------------------------------------------------------------------
// DENM – Decentralized Environmental Notification (ETSI EN 302 637-3)
// ---------------------------------------------------------------------------

/// DENM Cause Code (ETSI EN 302 637-3 Table A.2)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum DenmCauseCode {
    Reserved = 0,
    TrafficCondition = 1,
    Accident = 2,
    Roadworks = 3,
    AdverseWeatherCondition = 6,
    HazardousLocation = 9,
    HumanPresenceOnRoad = 12,
    WrongWayDriving = 14,
    RescueAndRecovery = 15,
    SlowVehicle = 17,
    DangerousEndOfQueue = 27,
    VehicleBreakdown = 91,
    PostCrash = 92,
    HumanProblem = 93,
    StationaryVehicle = 94,
    EmergencyVehicleApproaching = 95,
    HazardousLocationSurfaceCondition = 96,
    CollisionRisk = 97,
    SignalViolation = 98,
    DangerousSituation = 99,
}

/// DENM Action ID (originating station ID + sequence number)
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DenmActionId {
    pub originating_station_id: u32,
    pub sequence_number: u16,
}

impl DenmActionId {
    pub fn to_bytes(&self) -> [u8; 6] {
        let mut buf = [0u8; 6];
        buf[0..4].copy_from_slice(&self.originating_station_id.to_be_bytes());
        buf[4..6].copy_from_slice(&self.sequence_number.to_be_bytes());
        buf
    }
}

/// DENM Management Container
#[derive(Debug, Clone)]
pub struct DenmManagementContainer {
    pub action_id: DenmActionId,
    /// Detection time (TAI ms)
    pub detection_time: u64,
    /// Reference time (TAI ms)
    pub reference_time: u64,
    /// Event position LPV
    pub event_position: LongPositionVector,
    /// Validity duration in seconds (0 = unlimited)
    pub validity_duration: u16,
    /// Station type of transmitting station
    pub station_type: StationType,
}

/// DENM Situation Container
#[derive(Debug, Clone)]
pub struct DenmSituationContainer {
    /// Informational quality (0-7)
    pub information_quality: u8,
    pub cause_code: DenmCauseCode,
    pub sub_cause_code: u8,
    /// Linked cause (optional)
    pub linked_cause: Option<DenmCauseCode>,
}

/// DENM Location Container
#[derive(Debug, Clone)]
pub struct DenmLocationContainer {
    /// Event speed: 0.01 m/s (optional)
    pub event_speed: Option<u16>,
    /// Event position heading: 0.1 degree (optional)
    pub event_heading: Option<u16>,
    /// Number of path trace points
    pub traces_count: u8,
}

/// Full DENM per ETSI EN 302 637-3
#[derive(Debug, Clone)]
pub struct DecentralizedEnvironmentalNotificationMessage {
    pub protocol_version: u8,
    pub message_id: u8, // 1 = DENM
    pub station_id: u32,
    pub management: DenmManagementContainer,
    pub situation: Option<DenmSituationContainer>,
    pub location: Option<DenmLocationContainer>,
}

impl DecentralizedEnvironmentalNotificationMessage {
    /// Create an accident DENM
    pub fn new_accident(
        station_id: u32,
        action_seq: u16,
        lpv: LongPositionVector,
        time_ms: u64,
    ) -> Self {
        let action_id = DenmActionId {
            originating_station_id: station_id,
            sequence_number: action_seq,
        };
        let mgmt = DenmManagementContainer {
            action_id,
            detection_time: time_ms,
            reference_time: time_ms,
            event_position: lpv,
            validity_duration: 600,
            station_type: StationType::PassengerCar,
        };
        let sit = DenmSituationContainer {
            information_quality: 7,
            cause_code: DenmCauseCode::Accident,
            sub_cause_code: 1,
            linked_cause: None,
        };
        DecentralizedEnvironmentalNotificationMessage {
            protocol_version: 2,
            message_id: 1,
            station_id,
            management: mgmt,
            situation: Some(sit),
            location: None,
        }
    }

    /// Serialize to bytes
    pub fn serialize(&self) -> Vec<u8> {
        let mut buf = Vec::new();
        buf.push(self.protocol_version);
        buf.push(self.message_id);
        buf.extend_from_slice(&self.station_id.to_be_bytes());
        // Management container
        buf.extend_from_slice(&self.management.action_id.to_bytes());
        buf.extend_from_slice(&self.management.detection_time.to_be_bytes());
        buf.extend_from_slice(&self.management.reference_time.to_be_bytes());
        buf.extend_from_slice(&self.management.event_position.to_bytes());
        buf.extend_from_slice(&self.management.validity_duration.to_be_bytes());
        buf.push(self.management.station_type as u8);
        // Situation container
        if let Some(ref sit) = self.situation {
            buf.push(1); // present
            buf.push(sit.information_quality);
            buf.push(sit.cause_code as u8);
            buf.push(sit.sub_cause_code);
            if let Some(lc) = sit.linked_cause {
                buf.push(1);
                buf.push(lc as u8);
            } else {
                buf.push(0);
            }
        } else {
            buf.push(0);
        }
        // Location container
        if let Some(ref loc) = self.location {
            buf.push(1);
            buf.push(if loc.event_speed.is_some() { 1 } else { 0 });
            if let Some(s) = loc.event_speed {
                buf.extend_from_slice(&s.to_be_bytes());
            }
            buf.push(if loc.event_heading.is_some() { 1 } else { 0 });
            if let Some(h) = loc.event_heading {
                buf.extend_from_slice(&h.to_be_bytes());
            }
            buf.push(loc.traces_count);
        } else {
            buf.push(0);
        }
        buf
    }
}

// ---------------------------------------------------------------------------
// Tx Power Control (ETSI EN 302 571)
// ---------------------------------------------------------------------------

/// Tx power controller per ETSI EN 302 571 and TS 102 687
#[derive(Debug, Clone)]
pub struct TxPowerController {
    /// Nominal Tx power (dBm)
    pub nominal_power_dbm: f64,
    /// Current Tx power (dBm)
    pub current_power_dbm: f64,
    /// Maximum allowed EIRP for this band (dBm)
    pub max_eirp_dbm: f64,
    /// Antenna gain (dBi)
    pub antenna_gain_dbi: f64,
    /// Path loss model exponent (free space = 2.0)
    pub path_loss_exponent: f64,
    /// Reference distance for power control (m)
    pub reference_distance_m: f64,
}

impl TxPowerController {
    /// Create controller for given channel band
    pub fn new(band: ItsBand) -> Self {
        let max_eirp = match band {
            ItsBand::ItsG5A => MAX_EIRP_ITS_G5A_DBM,
            ItsBand::ItsG5B => MAX_EIRP_ITS_G5B_DBM,
            ItsBand::ItsG5C => 30.0,
            ItsBand::Unknown => 20.0,
        };
        TxPowerController {
            nominal_power_dbm: 23.0,
            current_power_dbm: 23.0,
            max_eirp_dbm: max_eirp,
            antenna_gain_dbi: 3.0,
            path_loss_exponent: 2.0,
            reference_distance_m: 10.0,
        }
    }

    /// Effective EIRP in dBm
    pub fn eirp_dbm(&self) -> f64 {
        self.current_power_dbm + self.antenna_gain_dbi
    }

    /// Check ETSI EN 302 571 compliance
    pub fn is_compliant(&self) -> bool {
        self.eirp_dbm() <= self.max_eirp_dbm
    }

    /// Estimate received signal strength at given distance (dBm)
    /// Uses log-distance path loss model
    pub fn estimated_rssi_dbm(&self, distance_m: f64, freq_hz: f64) -> f64 {
        if distance_m <= 0.0 {
            return self.eirp_dbm();
        }
        // FSPL at reference distance
        let lambda = 3e8 / freq_hz;
        let fspl_ref = 20.0 * (4.0 * PI * self.reference_distance_m / lambda).log10();
        // Path loss at actual distance
        let path_loss = fspl_ref + 10.0 * self.path_loss_exponent
            * (distance_m / self.reference_distance_m).log10();
        self.eirp_dbm() - path_loss
    }

    /// Distance-based power adjustment: reduce power if target RSSI already met
    pub fn adjust_for_distance(&mut self, target_rssi_dbm: f64, distance_m: f64, freq_hz: f64) {
        // Find minimum power needed to achieve target RSSI at given distance
        let current_rssi = self.estimated_rssi_dbm(distance_m, freq_hz);
        let excess = current_rssi - target_rssi_dbm;
        if excess > 2.0 {
            // Reduce power, minimum 0 dBm
            self.current_power_dbm = (self.current_power_dbm - excess + 2.0).max(0.0);
        } else if excess < -2.0 {
            // Increase power, cap at max compliant level
            let max_pa = self.max_eirp_dbm - self.antenna_gain_dbi;
            self.current_power_dbm = (self.current_power_dbm - excess - 2.0).min(max_pa);
        }
        // Enforce compliance
        let max_pa = self.max_eirp_dbm - self.antenna_gain_dbi;
        self.current_power_dbm = self.current_power_dbm.min(max_pa);
    }

    /// Apply DCC power reduction
    pub fn apply_dcc_state(&mut self, state: DccState) {
        let dcc_max = state.tx_power_dbm();
        if self.current_power_dbm > dcc_max {
            self.current_power_dbm = dcc_max;
        }
    }
}

// ---------------------------------------------------------------------------
// ITS-G5 Station (top-level processor)
// ---------------------------------------------------------------------------

/// ITS-G5 V2X station processor
#[derive(Debug)]
pub struct ItsG5Station {
    /// Station identifier
    pub station_id: u32,
    /// Station type
    pub station_type: StationType,
    /// Active PHY
    pub phy: ItsG5Phy,
    /// DCC engine
    pub dcc: DccEngine,
    /// Tx power controller
    pub tx_ctrl: TxPowerController,
    /// Current position
    pub position: LongPositionVector,
    /// CAM sequence counter
    cam_seq: u16,
    /// DENM sequence counter
    denm_seq: u16,
}

impl ItsG5Station {
    /// Create new ITS-G5 station on control channel 180
    pub fn new(station_id: u32, station_type: StationType) -> Self {
        let phy = ItsG5Phy::new(EU_CONTROL_CHANNEL);
        let band = phy.channel.band;
        ItsG5Station {
            station_id,
            station_type,
            phy,
            dcc: DccEngine::new(-85.0), // Typical CCA threshold
            tx_ctrl: TxPowerController::new(band),
            position: LongPositionVector::new(station_id as u64, 0.0, 0.0, 0.0, 0.0),
            cam_seq: 0,
            denm_seq: 0,
        }
    }

    /// Update station position
    pub fn update_position(&mut self, lat: f64, lon: f64, speed_ms: f64, heading_deg: f64) {
        self.position = LongPositionVector::new(
            self.station_id as u64, lat, lon, speed_ms, heading_deg,
        );
    }

    /// Generate CAM and wrap in GeoNetworking (GeoBroadcast)
    pub fn generate_cam(&mut self, time_ms: u32) -> GeoBroadcastPacket {
        self.cam_seq = self.cam_seq.wrapping_add(1);
        let cam = CooperativeAwarenessMessage::new_vehicle(
            self.station_id,
            self.position,
            self.position.heading as f64 / 10.0,
            self.position.speed as f64 / 100.0,
            time_ms,
        );
        let payload = cam.serialize();
        let btp = BtpBHeader::new(BtpPort::Cam);
        let mut full_payload = btp.to_bytes().to_vec();
        full_payload.extend_from_slice(&payload);

        // CAM is broadcast in a 1000m circle around sender
        let area = GeoArea::Circle {
            lat: self.position.latitude_deg(),
            lon: self.position.longitude_deg(),
            radius_m: 1000.0,
        };
        let mut pkt = GeoBroadcastPacket::new(self.position, area, full_payload);
        pkt.sequence_number = self.cam_seq;
        pkt
    }

    /// Generate DENM for accident event
    pub fn generate_accident_denm(&mut self, time_ms: u64, radius_m: f64) -> GeoBroadcastPacket {
        self.denm_seq = self.denm_seq.wrapping_add(1);
        let denm = DecentralizedEnvironmentalNotificationMessage::new_accident(
            self.station_id,
            self.denm_seq,
            self.position,
            time_ms,
        );
        let payload = denm.serialize();
        let btp = BtpBHeader::new(BtpPort::Denm);
        let mut full_payload = btp.to_bytes().to_vec();
        full_payload.extend_from_slice(&payload);

        let area = GeoArea::Circle {
            lat: self.position.latitude_deg(),
            lon: self.position.longitude_deg(),
            radius_m,
        };
        let mut pkt = GeoBroadcastPacket::new(self.position, area, full_payload);
        pkt.sequence_number = self.denm_seq;
        pkt
    }

    /// Process received channel power sample and update DCC
    pub fn process_channel_sample(&mut self, time_ms: f64, power_dbm: f64) {
        self.dcc.update(time_ms, power_dbm);
        self.tx_ctrl.apply_dcc_state(self.dcc.state);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // --- Channel Plan Tests ---

    #[test]
    fn test_channel_center_frequency_172() {
        let ch = ItsChannel::from_number(172);
        // 5000 MHz + 172 * 5 MHz = 5860 MHz
        assert_eq!(ch.center_freq_hz as u64, 5_860_000_000);
        // Per ETSI EN 302 663 Table 1, ch 172 is ITS-G5A (lower bound extended to 5855 MHz)
        assert_eq!(ch.band, ItsBand::ItsG5A);
    }

    #[test]
    fn test_channel_176_is_its_g5a() {
        let ch = ItsChannel::from_number(176);
        // 5000 + 176*5 = 5880 MHz → within ITS-G5A
        assert!((ch.center_freq_hz - 5_880_000_000.0).abs() < 1.0);
        assert_eq!(ch.band, ItsBand::ItsG5A);
    }

    #[test]
    fn test_control_channel_180() {
        let ch = ItsChannel::from_number(180);
        assert!(ch.is_control);
        // 5000 + 180*5 = 5900 MHz
        assert!((ch.center_freq_hz - 5_900_000_000.0).abs() < 1.0);
        assert_eq!(ch.band, ItsBand::ItsG5A);
    }

    #[test]
    fn test_channel_182_is_its_g5b() {
        let ch = ItsChannel::from_number(182);
        assert_eq!(ch.band, ItsBand::ItsG5B);
    }

    #[test]
    fn test_its_g5a_channels_list() {
        let chans = ItsChannel::its_g5a_channels();
        assert_eq!(chans.len(), 5);
        // All 5 channels (172-180) should be classified as ITS-G5A
        let g5a_count = chans.iter().filter(|c| c.band == ItsBand::ItsG5A).count();
        assert_eq!(g5a_count, 5);
    }

    #[test]
    fn test_its_g5b_channels_list() {
        let chans = ItsChannel::its_g5b_channels();
        assert_eq!(chans.len(), 2);
        assert!(chans.iter().all(|c| c.band == ItsBand::ItsG5B));
    }

    #[test]
    fn test_max_eirp_its_g5a() {
        let ch = ItsChannel::from_number(180);
        assert_eq!(ch.max_eirp_dbm(), MAX_EIRP_ITS_G5A_DBM);
    }

    #[test]
    fn test_max_eirp_its_g5b() {
        let ch = ItsChannel::from_number(182);
        assert_eq!(ch.max_eirp_dbm(), MAX_EIRP_ITS_G5B_DBM);
    }

    // --- MCS Tests ---

    #[test]
    fn test_mcs_data_rates() {
        assert_eq!(ItsMcs::Bpsk1_2.data_rate_mbps(), 3.0);
        assert_eq!(ItsMcs::Qpsk1_2.data_rate_mbps(), 6.0);
        assert_eq!(ItsMcs::Qam16_1_2.data_rate_mbps(), 12.0);
        assert_eq!(ItsMcs::Qam64_3_4.data_rate_mbps(), 27.0);
    }

    #[test]
    fn test_mcs_coded_bits_per_symbol() {
        assert_eq!(ItsMcs::Bpsk1_2.coded_bits_per_symbol(), 48);
        assert_eq!(ItsMcs::Qpsk1_2.coded_bits_per_symbol(), 96);
        assert_eq!(ItsMcs::Qam16_1_2.coded_bits_per_symbol(), 192);
        assert_eq!(ItsMcs::Qam64_3_4.coded_bits_per_symbol(), 288);
    }

    #[test]
    fn test_mcs_data_bits_per_symbol() {
        // BPSK 1/2: 48 coded bits * 1/2 = 24 data bits
        assert_eq!(ItsMcs::Bpsk1_2.data_bits_per_symbol(), 24);
        // QPSK 3/4: 96 coded bits * 3/4 = 72 data bits
        assert_eq!(ItsMcs::Qpsk3_4.data_bits_per_symbol(), 72);
    }

    #[test]
    fn test_signal_rate_field() {
        assert_eq!(ItsMcs::Bpsk1_2.signal_rate_field(), 0b1101);
        assert_eq!(ItsMcs::Qpsk1_2.signal_rate_field(), 0b0101);
    }

    // --- Subcarrier Index Tests ---

    #[test]
    fn test_pilot_subcarrier_count() {
        let pilots = pilot_subcarrier_indices();
        assert_eq!(pilots.len(), PILOT_SUBCARRIERS);
    }

    #[test]
    fn test_data_subcarrier_count() {
        let data = data_subcarrier_indices();
        assert_eq!(data.len(), DATA_SUBCARRIERS);
    }

    #[test]
    fn test_no_dc_in_data_subcarriers() {
        let data = data_subcarrier_indices();
        assert!(!data.contains(&0));
    }

    #[test]
    fn test_no_pilots_in_data_subcarriers() {
        let data = data_subcarrier_indices();
        let pilots = pilot_subcarrier_indices();
        for p in &pilots {
            assert!(!data.contains(p));
        }
    }

    // --- OFDM PHY Tests ---

    #[test]
    fn test_phy_symbol_samples_count() {
        let phy = ItsG5Phy::new(EU_CONTROL_CHANNEL);
        let freq = vec![(1.0f64, 0.0f64); FFT_SIZE];
        let sym = phy.ifft_and_add_cp(&freq);
        assert_eq!(sym.len(), SYMBOL_SAMPLES);
    }

    #[test]
    fn test_phy_fft_ifft_roundtrip() {
        let phy = ItsG5Phy::new(EU_CONTROL_CHANNEL);
        // Single tone in bin 5
        let mut freq_in = vec![(0.0f64, 0.0f64); FFT_SIZE];
        freq_in[5] = (1.0, 0.0);
        let time_samples = phy.ifft_and_add_cp(&freq_in);
        let freq_out = phy.remove_cp_and_fft(&time_samples);
        assert_eq!(freq_out.len(), FFT_SIZE);
        // Bin 5 should have approximately magnitude 1
        let mag5 = (freq_out[5].0.powi(2) + freq_out[5].1.powi(2)).sqrt();
        assert!((mag5 - 1.0).abs() < 0.01, "magnitude={}", mag5);
    }

    #[test]
    fn test_phy_bpsk_modulate() {
        let mut phy = ItsG5Phy::new(EU_CONTROL_CHANNEL);
        phy.mcs = ItsMcs::Bpsk1_2;
        let data = vec![0xFFu8]; // All ones
        let syms = phy.modulate_bytes(&data);
        assert_eq!(syms.len(), DATA_SUBCARRIERS);
        // All 1s -> all should be (-1, 0)
        for s in syms.iter().take(8) {
            assert!((s.0 + 1.0).abs() < 1e-9 && s.1.abs() < 1e-9, "sym={:?}", s);
        }
    }

    #[test]
    fn test_phy_qpsk_demodulate_roundtrip() {
        let mut phy = ItsG5Phy::new(EU_CONTROL_CHANNEL);
        phy.mcs = ItsMcs::Qpsk1_2;
        let data = vec![0b10110100u8];
        let syms = phy.modulate_bytes(&data);
        let recovered = phy.demodulate_symbols(&syms);
        // First byte should match
        assert_eq!(recovered[0], 0b10110100u8);
    }

    #[test]
    fn test_cbr_measurement() {
        // 30 out of 100 samples above threshold → CBR = 0.30
        let samples: Vec<(f64, f64)> = (0..100).map(|i| {
            let power = if i < 30 { -80.0 } else { -100.0 };
            (power, -85.0)
        }).collect();
        let cbr = ItsG5Phy::measure_cbr(&samples);
        assert!((cbr - 0.30).abs() < 0.01);
    }

    #[test]
    fn test_snr_estimation_from_pilots() {
        // Perfect reception → high SNR
        let known = vec![(1.0f64, 0.0f64); 4];
        let received = vec![(1.0f64, 0.0f64); 4];
        let snr = ItsG5Phy::estimate_snr_from_pilots(&received, &known);
        assert!(snr > 30.0, "snr={}", snr);
    }

    #[test]
    fn test_snr_estimation_noisy() {
        let known = vec![(1.0f64, 0.0f64); 4];
        let noise = 0.1f64;
        let received = vec![(1.0 + noise, noise); 4];
        let snr = ItsG5Phy::estimate_snr_from_pilots(&received, &known);
        assert!(snr > 10.0 && snr < 35.0, "snr={}", snr);
    }

    // --- DCC Tests ---

    #[test]
    fn test_dcc_initial_state_relaxed() {
        let dcc = DccEngine::new(-85.0);
        assert_eq!(dcc.state, DccState::Relaxed);
    }

    #[test]
    fn test_dcc_transition_to_active_on_high_cbr() {
        let mut dcc = DccEngine::new(-85.0);
        // Feed 200ms of idle first (fills window with 0% CBR), then transition to 40% busy.
        // This ensures CBR stays below L2 when it transitions from Relaxed to Active.
        // Phase 1: 200 idle samples at 1ms intervals (200ms idle, CBR=0%)
        for i in 0..200 {
            dcc.update(i as f64, -100.0); // all idle
        }
        assert_eq!(dcc.state, DccState::Relaxed);
        // Phase 2: 200 samples at 1ms intervals, 40% busy (above L1=0.30, below L2=0.60)
        // Pattern: busy on 0,1,2 out of every 10 (30% would not trigger; use 4/10=40%)
        for i in 0..200 {
            let t = 200.0 + i as f64;
            let power = if i % 10 < 4 { -80.0 } else { -100.0 }; // 40% busy
            dcc.update(t, power);
        }
        assert_eq!(dcc.state, DccState::Active);
    }

    #[test]
    fn test_dcc_transition_to_restrictive_on_very_high_cbr() {
        let mut dcc = DccEngine::new(-85.0);
        // Feed 70% busy (above DCC_CBR_L2=0.60)
        for i in 0..200 {
            let power = if i % 10 < 7 { -80.0 } else { -100.0 };
            dcc.update(i as f64 * 0.5, power);
        }
        assert_eq!(dcc.state, DccState::Restrictive);
    }

    #[test]
    fn test_dcc_state_min_ipt() {
        assert_eq!(DccState::Relaxed.min_ipt_ms(), DCC_T_RELAXED_MS);
        assert_eq!(DccState::Active.min_ipt_ms(), DCC_T_ACTIVE_MS);
        assert_eq!(DccState::Restrictive.min_ipt_ms(), DCC_T_RESTRICTIVE_MS);
    }

    #[test]
    fn test_dcc_can_transmit_after_ipt() {
        let mut dcc = DccEngine::new(-85.0);
        dcc.current_time_ms = 1000.0;
        dcc.last_tx_time_ms = 970.0; // 30ms ago = > 25ms IPT
        assert!(dcc.can_transmit());
    }

    #[test]
    fn test_dcc_cannot_transmit_before_ipt() {
        let mut dcc = DccEngine::new(-85.0);
        dcc.current_time_ms = 1000.0;
        dcc.last_tx_time_ms = 990.0; // 10ms ago < 25ms IPT
        assert!(!dcc.can_transmit());
    }

    #[test]
    fn test_dcc_tx_power_limits() {
        assert!(DccState::Relaxed.tx_power_dbm() > DccState::Active.tx_power_dbm());
        assert!(DccState::Active.tx_power_dbm() > DccState::Restrictive.tx_power_dbm());
    }

    #[test]
    fn test_dcc_duty_cycle_tracking() {
        let mut dcc = DccEngine::new(-85.0);
        dcc.current_time_ms = 50.0;
        dcc.record_transmission(10.0);
        let dc = dcc.duty_cycle();
        assert!((dc - 0.1).abs() < 1e-9);
    }

    // --- GeoNetworking Tests ---

    #[test]
    fn test_lpv_roundtrip() {
        let lpv = LongPositionVector::new(0x1234567890ABCDEF, 48.8566, 2.3522, 13.9, 90.0);
        let bytes = lpv.to_bytes();
        let lpv2 = LongPositionVector::from_bytes(&bytes);
        assert_eq!(lpv.gn_addr, lpv2.gn_addr);
        assert!((lpv.latitude_deg() - lpv2.latitude_deg()).abs() < 0.01);
        assert!((lpv.longitude_deg() - lpv2.longitude_deg()).abs() < 0.01);
    }

    #[test]
    fn test_basic_header_roundtrip() {
        let hdr = GnBasicHeader::new(GnNextHeader::BtpB, 0x3F, 10);
        let bytes = hdr.to_bytes();
        let hdr2 = GnBasicHeader::from_bytes(&bytes);
        assert_eq!(hdr.version, hdr2.version);
        assert_eq!(hdr.hop_limit, hdr2.hop_limit);
    }

    #[test]
    fn test_geobroadcast_packet_serialization() {
        let lpv = LongPositionVector::new(1, 48.8566, 2.3522, 0.0, 0.0);
        let area = GeoArea::Circle { lat: 48.8566, lon: 2.3522, radius_m: 500.0 };
        let pkt = GeoBroadcastPacket::new(lpv, area, vec![0x01, 0x02, 0x03]);
        let bytes = pkt.serialize();
        // Basic header (4) + common header (8) + LPV (24) + seq+reserved (4) + area (12) + payload (3)
        assert_eq!(bytes.len(), 4 + 8 + 24 + 4 + 12 + 3);
    }

    #[test]
    fn test_geo_area_circle_contains() {
        let area = GeoArea::Circle { lat: 48.85, lon: 2.35, radius_m: 1000.0 };
        // Center point
        assert!(area.contains(48.85, 2.35));
        // Point ~500m away (rough approximation)
        assert!(area.contains(48.855, 2.35));
        // Point far away
        assert!(!area.contains(49.0, 3.0));
    }

    #[test]
    fn test_geo_area_rectangle_contains() {
        let area = GeoArea::Rectangle {
            lat: 48.85,
            lon: 2.35,
            dist_a_m: 500.0,
            dist_b_m: 200.0,
            angle_deg: 0.0,
        };
        assert!(area.contains(48.85, 2.35)); // center
        assert!(!area.contains(49.0, 3.0));  // far away
    }

    #[test]
    fn test_is_in_area() {
        let lpv = LongPositionVector::new(1, 48.85, 2.35, 0.0, 0.0);
        let area = GeoArea::Circle { lat: 48.85, lon: 2.35, radius_m: 1000.0 };
        let pkt = GeoBroadcastPacket::new(lpv, area, vec![]);
        assert!(pkt.is_in_area(48.85, 2.35));
        assert!(!pkt.is_in_area(52.0, 5.0));
    }

    // --- BTP Tests ---

    #[test]
    fn test_btp_a_header_roundtrip() {
        let hdr = BtpAHeader::new(BtpPort::Cam);
        let bytes = hdr.to_bytes();
        let hdr2 = BtpAHeader::from_bytes(&bytes);
        assert_eq!(hdr.dest_port, hdr2.dest_port);
        assert_eq!(hdr.dest_port, BtpPort::Cam as u16);
    }

    #[test]
    fn test_btp_b_header_roundtrip() {
        let hdr = BtpBHeader::new(BtpPort::Denm);
        let bytes = hdr.to_bytes();
        let hdr2 = BtpBHeader::from_bytes(&bytes);
        assert_eq!(hdr.dest_port, hdr2.dest_port);
        assert_eq!(hdr.dest_port, BtpPort::Denm as u16);
    }

    #[test]
    fn test_btp_port_values() {
        assert_eq!(BtpPort::Cam as u16, 2001);
        assert_eq!(BtpPort::Denm as u16, 2002);
        assert_eq!(BtpPort::MapSpatem as u16, 2003);
        assert_eq!(BtpPort::Cpm as u16, 2009);
    }

    // --- CAM Tests ---

    #[test]
    fn test_cam_serialization_non_empty() {
        let lpv = LongPositionVector::new(42, 52.37, 4.89, 14.0, 270.0);
        let cam = CooperativeAwarenessMessage::new_vehicle(42, lpv, 270.0, 14.0, 1000);
        let bytes = cam.serialize();
        assert!(!bytes.is_empty());
        assert_eq!(bytes[0], 2); // protocol version
        assert_eq!(bytes[1], 2); // message_id = CAM
    }

    #[test]
    fn test_cam_station_id_in_serialized() {
        let station_id: u32 = 0xDEADBEEF;
        let lpv = LongPositionVector::new(station_id as u64, 0.0, 0.0, 0.0, 0.0);
        let cam = CooperativeAwarenessMessage::new_vehicle(station_id, lpv, 0.0, 0.0, 0);
        let bytes = cam.serialize();
        let extracted_id = u32::from_be_bytes(bytes[2..6].try_into().unwrap());
        assert_eq!(extracted_id, station_id);
    }

    #[test]
    fn test_cam_adaptive_generation_interval_stationary() {
        let interval = CooperativeAwarenessMessage::adaptive_generation_interval(0.0, 0.0, 0.0);
        assert_eq!(interval, CAM_T_GEN_MAX_MS);
    }

    #[test]
    fn test_cam_adaptive_generation_interval_maneuvering() {
        let interval = CooperativeAwarenessMessage::adaptive_generation_interval(10.0, 10.0, 1.0);
        assert_eq!(interval, CAM_T_GEN_MIN_MS);
    }

    #[test]
    fn test_cam_high_freq_speed_encoding() {
        let hf = CamHighFreqContainer::new_vehicle(90.0, 13.89, DriveDirection::Forward, 4.5, 1.8);
        // 13.89 m/s * 100 = 1389
        assert_eq!(hf.speed, 1389);
    }

    // --- DENM Tests ---

    #[test]
    fn test_denm_serialization_non_empty() {
        let lpv = LongPositionVector::new(99, 51.5, -0.1, 0.0, 0.0);
        let denm = DecentralizedEnvironmentalNotificationMessage::new_accident(99, 1, lpv, 1_000_000);
        let bytes = denm.serialize();
        assert!(!bytes.is_empty());
        assert_eq!(bytes[0], 2); // protocol version
        assert_eq!(bytes[1], 1); // message_id = DENM
    }

    #[test]
    fn test_denm_action_id_encoding() {
        let action = DenmActionId {
            originating_station_id: 0x12345678,
            sequence_number: 0xABCD,
        };
        let bytes = action.to_bytes();
        let sid = u32::from_be_bytes(bytes[0..4].try_into().unwrap());
        let seq = u16::from_be_bytes(bytes[4..6].try_into().unwrap());
        assert_eq!(sid, 0x12345678);
        assert_eq!(seq, 0xABCD);
    }

    #[test]
    fn test_denm_cause_codes() {
        assert_eq!(DenmCauseCode::Accident as u8, 2);
        assert_eq!(DenmCauseCode::EmergencyVehicleApproaching as u8, 95);
        assert_eq!(DenmCauseCode::CollisionRisk as u8, 97);
    }

    // --- Tx Power Control Tests ---

    #[test]
    fn test_tx_power_eirp_calculation() {
        let ctrl = TxPowerController::new(ItsBand::ItsG5A);
        // 23 dBm + 3 dBi = 26 dBm
        assert!((ctrl.eirp_dbm() - 26.0).abs() < 0.01);
    }

    #[test]
    fn test_tx_power_compliance_check() {
        let ctrl = TxPowerController::new(ItsBand::ItsG5A);
        assert!(ctrl.is_compliant()); // 26 < 33 dBm max EIRP
    }

    #[test]
    fn test_tx_power_non_compliant() {
        let mut ctrl = TxPowerController::new(ItsBand::ItsG5B);
        // Set power such that EIRP > 23 dBm (G5B limit)
        ctrl.current_power_dbm = 25.0; // EIRP = 25 + 3 = 28 > 23
        assert!(!ctrl.is_compliant());
    }

    #[test]
    fn test_tx_power_rssi_at_distance() {
        let ctrl = TxPowerController::new(ItsBand::ItsG5A);
        let rssi_100m = ctrl.estimated_rssi_dbm(100.0, 5.9e9);
        let rssi_1000m = ctrl.estimated_rssi_dbm(1000.0, 5.9e9);
        // RSSI should decrease with distance (more negative)
        assert!(rssi_100m > rssi_1000m);
    }

    #[test]
    fn test_tx_power_dcc_reduction() {
        let mut ctrl = TxPowerController::new(ItsBand::ItsG5A);
        ctrl.current_power_dbm = 23.0;
        ctrl.apply_dcc_state(DccState::Restrictive);
        assert!(ctrl.current_power_dbm <= TX_POWER_RESTRICTIVE_DBM);
    }

    // --- Station Integration Tests ---

    #[test]
    fn test_station_creation() {
        let station = ItsG5Station::new(12345, StationType::PassengerCar);
        assert_eq!(station.station_id, 12345);
        assert_eq!(station.phy.channel.number, EU_CONTROL_CHANNEL);
    }

    #[test]
    fn test_station_cam_generation() {
        let mut station = ItsG5Station::new(1, StationType::PassengerCar);
        station.update_position(52.37, 4.89, 10.0, 90.0);
        let pkt = station.generate_cam(1000);
        assert!(!pkt.payload.is_empty());
        assert_eq!(pkt.sequence_number, 1);
    }

    #[test]
    fn test_station_denm_generation() {
        let mut station = ItsG5Station::new(2, StationType::PassengerCar);
        station.update_position(48.13, 11.57, 0.0, 0.0);
        let pkt = station.generate_accident_denm(5_000_000, 500.0);
        assert!(!pkt.payload.is_empty());
    }

    #[test]
    fn test_station_dcc_channel_monitoring() {
        let mut station = ItsG5Station::new(3, StationType::RoadSideUnit);
        // Simulate busy channel
        for i in 0..200 {
            station.process_channel_sample(i as f64 * 0.5, -80.0);
        }
        // Should have transitioned to Restrictive
        assert!(station.dcc.cbr > 0.5);
    }

    #[test]
    fn test_station_cam_sequence_increments() {
        let mut station = ItsG5Station::new(10, StationType::PassengerCar);
        station.update_position(0.0, 0.0, 0.0, 0.0);
        let p1 = station.generate_cam(0);
        let p2 = station.generate_cam(100);
        let p3 = station.generate_cam(200);
        assert_eq!(p1.sequence_number, 1);
        assert_eq!(p2.sequence_number, 2);
        assert_eq!(p3.sequence_number, 3);
    }

    #[test]
    fn test_lpv_speed_encoding() {
        let lpv = LongPositionVector::new(1, 0.0, 0.0, 25.5, 180.0);
        // 25.5 m/s * 100 = 2550 → stored in i16 as 2550
        assert_eq!(lpv.speed, 2550);
        assert!((lpv.speed_ms() - 25.5).abs() < 0.01);
    }

    #[test]
    fn test_gn_basic_header_lifetime_secs() {
        // lifetime byte: mult=0 (bits 7-6), base=20 (bits 5-0) → 20*50ms = 1.0s
        let hdr = GnBasicHeader { version: 1, next_header: GnNextHeader::BtpB, hop_limit: 10, lifetime: 20 };
        assert!((hdr.lifetime_secs() - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_geo_area_ellipse_contains() {
        let area = GeoArea::Ellipse {
            lat: 48.85,
            lon: 2.35,
            semi_major_m: 500.0,
            semi_minor_m: 200.0,
            angle_deg: 0.0,
        };
        assert!(area.contains(48.85, 2.35)); // center
        assert!(!area.contains(49.0, 2.35)); // far north
    }
}
